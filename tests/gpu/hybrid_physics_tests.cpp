/**
 * @file hybrid_physics_tests.cpp
 * @brief Tests and benchmarks for hybrid CPU/GPU physics system
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "jaguar/gpu/hybrid_physics.h"
#include "jaguar/gpu/compute_backend.h"
#include "jaguar/physics/entity.h"
#include <chrono>
#include <random>
#include <numeric>

using namespace jaguar;
using namespace jaguar::gpu;
using namespace jaguar::physics;

// ============================================================================
// Test Fixtures
// ============================================================================

class HybridPhysicsTest : public ::testing::Test {
protected:
    void SetUp() override {
        m_entity_storage = std::make_unique<EntityStateStorage>();
    }

    void TearDown() override {
        if (m_hybrid) {
            m_hybrid->shutdown();
            m_hybrid.reset();
        }
        m_entity_storage.reset();
    }

    void create_entities(SizeT count) {
        m_entity_storage->reserve(count);

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<Real> pos_dist(-1000.0, 1000.0);
        std::uniform_real_distribution<Real> vel_dist(-10.0, 10.0);
        std::uniform_real_distribution<Real> mass_dist(1.0, 100.0);

        for (SizeT i = 0; i < count; ++i) {
            UInt32 idx = m_entity_storage->allocate();

            m_entity_storage->position(idx) = Vec3{
                pos_dist(gen), pos_dist(gen), pos_dist(gen)
            };
            m_entity_storage->velocity(idx) = Vec3{
                vel_dist(gen), vel_dist(gen), vel_dist(gen)
            };
            m_entity_storage->acceleration(idx) = Vec3{0.0, 0.0, -9.81};
            m_entity_storage->mass(idx) = mass_dist(gen);
            m_entity_storage->set_active(idx, true);
        }
    }

    std::unique_ptr<EntityStateStorage> m_entity_storage;
    std::unique_ptr<HybridPhysicsSystem> m_hybrid;
};

// ============================================================================
// Configuration Tests
// ============================================================================

TEST_F(HybridPhysicsTest, DefaultConfiguration) {
    HybridPhysicsConfig config;

    EXPECT_EQ(config.strategy, RoutingStrategy::Adaptive);
    EXPECT_EQ(config.gpu_threshold, 200);
    EXPECT_EQ(config.gpu_min_batch, 50);
    EXPECT_EQ(config.transfer_mode, TransferMode::Asynchronous);
    EXPECT_TRUE(config.enable_dynamic_adjustment);
    EXPECT_TRUE(config.use_memory_pool);
}

TEST_F(HybridPhysicsTest, PhysicsOptimizedConfiguration) {
    auto config = HybridPhysicsConfig::physics_optimized();

    EXPECT_EQ(config.strategy, RoutingStrategy::Adaptive);
    EXPECT_EQ(config.gpu_threshold, 100);
    EXPECT_TRUE(config.enable_dynamic_adjustment);
    EXPECT_FALSE(config.workload_thresholds.empty());
}

TEST_F(HybridPhysicsTest, LowLatencyConfiguration) {
    auto config = HybridPhysicsConfig::low_latency();

    EXPECT_EQ(config.strategy, RoutingStrategy::ThresholdBased);
    EXPECT_EQ(config.gpu_threshold, 500);
    EXPECT_EQ(config.transfer_mode, TransferMode::Synchronous);
    EXPECT_FALSE(config.enable_dynamic_adjustment);
}

TEST_F(HybridPhysicsTest, HighThroughputConfiguration) {
    auto config = HybridPhysicsConfig::high_throughput();

    EXPECT_EQ(config.strategy, RoutingStrategy::ForceGPU);
    EXPECT_EQ(config.gpu_threshold, 50);
    EXPECT_EQ(config.transfer_mode, TransferMode::PipelinedTriple);
}

// ============================================================================
// Initialization Tests
// ============================================================================

TEST_F(HybridPhysicsTest, InitializeWithDefaultConfig) {
    create_entities(100);

    m_hybrid = std::make_unique<HybridPhysicsSystem>();
    HybridPhysicsConfig config;

    auto result = m_hybrid->initialize(config, m_entity_storage.get());
    EXPECT_EQ(result, BackendResult::Success);
    EXPECT_TRUE(m_hybrid->is_initialized());
}

TEST_F(HybridPhysicsTest, InitializeWithNullStorage) {
    m_hybrid = std::make_unique<HybridPhysicsSystem>();
    HybridPhysicsConfig config;

    auto result = m_hybrid->initialize(config, nullptr);
    EXPECT_EQ(result, BackendResult::InvalidArgument);
    EXPECT_FALSE(m_hybrid->is_initialized());
}

TEST_F(HybridPhysicsTest, DoubleInitialize) {
    create_entities(100);

    m_hybrid = std::make_unique<HybridPhysicsSystem>();
    HybridPhysicsConfig config;

    auto result = m_hybrid->initialize(config, m_entity_storage.get());
    EXPECT_EQ(result, BackendResult::Success);

    // Second initialize should return NotSupported
    result = m_hybrid->initialize(config, m_entity_storage.get());
    EXPECT_EQ(result, BackendResult::NotSupported);
}

TEST_F(HybridPhysicsTest, ShutdownReinitialize) {
    create_entities(100);

    m_hybrid = std::make_unique<HybridPhysicsSystem>();
    HybridPhysicsConfig config;

    auto result = m_hybrid->initialize(config, m_entity_storage.get());
    EXPECT_EQ(result, BackendResult::Success);

    m_hybrid->shutdown();
    EXPECT_FALSE(m_hybrid->is_initialized());

    // Should be able to reinitialize after shutdown
    result = m_hybrid->initialize(config, m_entity_storage.get());
    EXPECT_EQ(result, BackendResult::Success);
    EXPECT_TRUE(m_hybrid->is_initialized());
}

// ============================================================================
// Routing Decision Tests
// ============================================================================

TEST_F(HybridPhysicsTest, ForceCPURouting) {
    create_entities(1000);  // Above threshold

    m_hybrid = std::make_unique<HybridPhysicsSystem>();
    HybridPhysicsConfig config;
    config.strategy = RoutingStrategy::ForceCPU;

    m_hybrid->initialize(config, m_entity_storage.get());

    Real dt = 0.016;
    m_hybrid->step(dt);

    const auto& decision = m_hybrid->last_decision();
    EXPECT_FALSE(decision.use_gpu);
    EXPECT_EQ(decision.reason, "Strategy: Force CPU");
}

TEST_F(HybridPhysicsTest, ThresholdBasedRouting_BelowThreshold) {
    create_entities(50);  // Below default threshold of 200

    m_hybrid = std::make_unique<HybridPhysicsSystem>();
    HybridPhysicsConfig config;
    config.strategy = RoutingStrategy::ThresholdBased;
    config.gpu_threshold = 200;

    m_hybrid->initialize(config, m_entity_storage.get());

    Real dt = 0.016;
    m_hybrid->step(dt);

    const auto& decision = m_hybrid->last_decision();
    EXPECT_FALSE(decision.use_gpu);
}

TEST_F(HybridPhysicsTest, ThresholdBasedRouting_AboveThreshold) {
    create_entities(500);  // Above threshold

    m_hybrid = std::make_unique<HybridPhysicsSystem>();
    HybridPhysicsConfig config;
    config.strategy = RoutingStrategy::ThresholdBased;
    config.gpu_threshold = 200;

    m_hybrid->initialize(config, m_entity_storage.get());

    Real dt = 0.016;
    m_hybrid->step(dt);

    const auto& decision = m_hybrid->last_decision();
    // GPU usage depends on availability
    if (m_hybrid->is_gpu_available()) {
        EXPECT_TRUE(decision.use_gpu);
    }
}

TEST_F(HybridPhysicsTest, ForceNextPathCPU) {
    create_entities(1000);

    m_hybrid = std::make_unique<HybridPhysicsSystem>();
    HybridPhysicsConfig config;

    m_hybrid->initialize(config, m_entity_storage.get());
    m_hybrid->force_next_path(false);  // Force CPU

    Real dt = 0.016;
    m_hybrid->step(dt);

    const auto& decision = m_hybrid->last_decision();
    EXPECT_FALSE(decision.use_gpu);
}

// ============================================================================
// Physics Step Tests
// ============================================================================

TEST_F(HybridPhysicsTest, StepWithZeroEntities) {
    m_hybrid = std::make_unique<HybridPhysicsSystem>();
    HybridPhysicsConfig config;

    m_hybrid->initialize(config, m_entity_storage.get());

    Real dt = 0.016;
    auto result = m_hybrid->step(dt);
    EXPECT_EQ(result, BackendResult::Success);
}

TEST_F(HybridPhysicsTest, StepWithSmallEntityCount) {
    create_entities(10);

    m_hybrid = std::make_unique<HybridPhysicsSystem>();
    HybridPhysicsConfig config;
    config.strategy = RoutingStrategy::ForceCPU;

    m_hybrid->initialize(config, m_entity_storage.get());

    // Store initial positions
    std::vector<Vec3> initial_positions(10);
    std::vector<Vec3> initial_velocities(10);
    for (SizeT i = 0; i < 10; ++i) {
        initial_positions[i] = m_entity_storage->position(i);
        initial_velocities[i] = m_entity_storage->velocity(i);
    }

    Real dt = 0.1;  // 100ms time step
    auto result = m_hybrid->step(dt);
    EXPECT_EQ(result, BackendResult::Success);

    // Check positions updated (should have moved based on velocity)
    for (SizeT i = 0; i < 10; ++i) {
        const Vec3& new_pos = m_entity_storage->position(i);
        const Vec3& init_pos = initial_positions[i];
        const Vec3& init_vel = initial_velocities[i];

        // Position should have changed approximately by velocity * dt
        Real expected_dx = init_vel.x * dt;
        Real expected_dy = init_vel.y * dt;
        Real expected_dz = init_vel.z * dt;

        Real actual_dx = new_pos.x - init_pos.x;
        Real actual_dy = new_pos.y - init_pos.y;
        Real actual_dz = new_pos.z - init_pos.z;

        EXPECT_NEAR(actual_dx, expected_dx, 0.01);
        EXPECT_NEAR(actual_dy, expected_dy, 0.01);
        EXPECT_NEAR(actual_dz, expected_dz, 0.01);
    }
}

TEST_F(HybridPhysicsTest, MultipleSteps) {
    create_entities(100);

    m_hybrid = std::make_unique<HybridPhysicsSystem>();
    HybridPhysicsConfig config;
    config.strategy = RoutingStrategy::ForceCPU;

    m_hybrid->initialize(config, m_entity_storage.get());

    Real dt = 0.016;
    for (int i = 0; i < 100; ++i) {
        auto result = m_hybrid->step(dt);
        EXPECT_EQ(result, BackendResult::Success);
    }

    // Check statistics
    const auto& stats = m_hybrid->stats();
    EXPECT_EQ(stats.total_frames, 100);
    EXPECT_EQ(stats.cpu_only_frames, 100);
}

// ============================================================================
// Statistics Tests
// ============================================================================

TEST_F(HybridPhysicsTest, StatisticsCollection) {
    create_entities(100);

    m_hybrid = std::make_unique<HybridPhysicsSystem>();
    HybridPhysicsConfig config;
    config.strategy = RoutingStrategy::ForceCPU;

    m_hybrid->initialize(config, m_entity_storage.get());

    Real dt = 0.016;
    for (int i = 0; i < 10; ++i) {
        m_hybrid->step(dt);
    }

    const auto& stats = m_hybrid->stats();
    EXPECT_EQ(stats.total_frames, 10);
    EXPECT_GT(stats.avg_frame_time_ms, 0.0);

    // Check workload stats
    const auto* integration_stats = stats.get_workload_stats(WorkloadType::Integration);
    if (integration_stats) {
        EXPECT_GT(integration_stats->cpu_executions, 0);
    }
}

TEST_F(HybridPhysicsTest, ResetStatistics) {
    create_entities(100);

    m_hybrid = std::make_unique<HybridPhysicsSystem>();
    HybridPhysicsConfig config;

    m_hybrid->initialize(config, m_entity_storage.get());

    Real dt = 0.016;
    m_hybrid->step(dt);
    m_hybrid->step(dt);

    EXPECT_EQ(m_hybrid->stats().total_frames, 2);

    m_hybrid->reset_stats();

    EXPECT_EQ(m_hybrid->stats().total_frames, 0);
    EXPECT_EQ(m_hybrid->stats().cpu_only_frames, 0);
}

// ============================================================================
// Routing Callback Test
// ============================================================================

TEST_F(HybridPhysicsTest, RoutingCallback) {
    create_entities(100);

    m_hybrid = std::make_unique<HybridPhysicsSystem>();
    HybridPhysicsConfig config;

    m_hybrid->initialize(config, m_entity_storage.get());

    int callback_count = 0;
    m_hybrid->set_routing_callback([&callback_count](const RoutingDecision& decision) {
        callback_count++;
        EXPECT_EQ(decision.entity_count, 100);
    });

    Real dt = 0.016;
    m_hybrid->step(dt);
    m_hybrid->step(dt);

    EXPECT_EQ(callback_count, 2);
}

// ============================================================================
// Workload Threshold Tests
// ============================================================================

TEST_F(HybridPhysicsTest, SetWorkloadThreshold) {
    create_entities(100);

    m_hybrid = std::make_unique<HybridPhysicsSystem>();
    HybridPhysicsConfig config;

    m_hybrid->initialize(config, m_entity_storage.get());

    m_hybrid->set_workload_threshold(WorkloadType::Integration, 500);
    EXPECT_EQ(m_hybrid->get_workload_threshold(WorkloadType::Integration), 500);

    m_hybrid->set_workload_threshold(WorkloadType::CollisionBroad, 1000);
    EXPECT_EQ(m_hybrid->get_workload_threshold(WorkloadType::CollisionBroad), 1000);
}

// ============================================================================
// Entity Dirty Tracking Tests
// ============================================================================

TEST_F(HybridPhysicsTest, MarkEntitiesDirty) {
    create_entities(100);

    m_hybrid = std::make_unique<HybridPhysicsSystem>();
    HybridPhysicsConfig config;

    m_hybrid->initialize(config, m_entity_storage.get());

    // Mark some entities as dirty
    m_hybrid->mark_entities_dirty({0, 5, 10, 50});

    // Step should still work
    Real dt = 0.016;
    auto result = m_hybrid->step(dt);
    EXPECT_EQ(result, BackendResult::Success);
}

TEST_F(HybridPhysicsTest, MarkAllDirty) {
    create_entities(100);

    m_hybrid = std::make_unique<HybridPhysicsSystem>();
    HybridPhysicsConfig config;

    m_hybrid->initialize(config, m_entity_storage.get());

    m_hybrid->mark_all_dirty();

    Real dt = 0.016;
    auto result = m_hybrid->step(dt);
    EXPECT_EQ(result, BackendResult::Success);
}

// ============================================================================
// Active Entity Tests
// ============================================================================

TEST_F(HybridPhysicsTest, SetActiveEntities) {
    create_entities(100);

    m_hybrid = std::make_unique<HybridPhysicsSystem>();
    HybridPhysicsConfig config;

    m_hybrid->initialize(config, m_entity_storage.get());

    // Only process a subset of entities
    std::vector<UInt32> active = {0, 10, 20, 30, 40};
    m_hybrid->set_active_entities(active);

    Real dt = 0.016;
    m_hybrid->step(dt);

    const auto& decision = m_hybrid->last_decision();
    EXPECT_EQ(decision.entity_count, 5);
}

// ============================================================================
// Performance Benchmarks
// ============================================================================

class HybridPhysicsBenchmark : public HybridPhysicsTest {
protected:
    struct BenchmarkResult {
        SizeT entity_count;
        Real cpu_time_ms;
        Real gpu_time_ms;
        Real speedup;
    };

    BenchmarkResult run_benchmark(SizeT entity_count, int iterations = 100) {
        create_entities(entity_count);

        BenchmarkResult result;
        result.entity_count = entity_count;

        // CPU benchmark
        {
            m_hybrid = std::make_unique<HybridPhysicsSystem>();
            HybridPhysicsConfig config;
            config.strategy = RoutingStrategy::ForceCPU;
            m_hybrid->initialize(config, m_entity_storage.get());

            auto start = std::chrono::high_resolution_clock::now();
            Real dt = 0.016;
            for (int i = 0; i < iterations; ++i) {
                m_hybrid->step(dt);
            }
            auto end = std::chrono::high_resolution_clock::now();

            result.cpu_time_ms = std::chrono::duration<Real, std::milli>(end - start).count() / iterations;
            m_hybrid->shutdown();
        }

        // Reset entity positions for GPU benchmark
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<Real> pos_dist(-1000.0, 1000.0);
        std::uniform_real_distribution<Real> vel_dist(-10.0, 10.0);

        for (SizeT i = 0; i < entity_count; ++i) {
            m_entity_storage->position(i) = Vec3{pos_dist(gen), pos_dist(gen), pos_dist(gen)};
            m_entity_storage->velocity(i) = Vec3{vel_dist(gen), vel_dist(gen), vel_dist(gen)};
        }

        // GPU benchmark (if available)
        {
            m_hybrid = std::make_unique<HybridPhysicsSystem>();
            HybridPhysicsConfig config;
            config.strategy = RoutingStrategy::ForceGPU;
            m_hybrid->initialize(config, m_entity_storage.get());

            if (m_hybrid->is_gpu_available()) {
                // Warm-up
                Real dt = 0.016;
                for (int i = 0; i < 10; ++i) {
                    m_hybrid->step(dt);
                }

                auto start = std::chrono::high_resolution_clock::now();
                for (int i = 0; i < iterations; ++i) {
                    m_hybrid->step(dt);
                }
                auto end = std::chrono::high_resolution_clock::now();

                result.gpu_time_ms = std::chrono::duration<Real, std::milli>(end - start).count() / iterations;
            } else {
                result.gpu_time_ms = result.cpu_time_ms;  // No GPU available
            }
        }

        result.speedup = result.cpu_time_ms / result.gpu_time_ms;
        return result;
    }
};

TEST_F(HybridPhysicsBenchmark, Benchmark_100_Entities) {
    auto result = run_benchmark(100, 50);

    std::cout << "Benchmark Results (100 entities):\n";
    std::cout << "  CPU time: " << result.cpu_time_ms << " ms\n";
    std::cout << "  GPU time: " << result.gpu_time_ms << " ms\n";
    std::cout << "  Speedup:  " << result.speedup << "x\n";

    EXPECT_GT(result.cpu_time_ms, 0.0);
}

TEST_F(HybridPhysicsBenchmark, Benchmark_1000_Entities) {
    auto result = run_benchmark(1000, 50);

    std::cout << "Benchmark Results (1000 entities):\n";
    std::cout << "  CPU time: " << result.cpu_time_ms << " ms\n";
    std::cout << "  GPU time: " << result.gpu_time_ms << " ms\n";
    std::cout << "  Speedup:  " << result.speedup << "x\n";

    EXPECT_GT(result.cpu_time_ms, 0.0);
}

TEST_F(HybridPhysicsBenchmark, Benchmark_10000_Entities) {
    auto result = run_benchmark(10000, 20);

    std::cout << "Benchmark Results (10000 entities):\n";
    std::cout << "  CPU time: " << result.cpu_time_ms << " ms\n";
    std::cout << "  GPU time: " << result.gpu_time_ms << " ms\n";
    std::cout << "  Speedup:  " << result.speedup << "x\n";

    EXPECT_GT(result.cpu_time_ms, 0.0);
}

TEST_F(HybridPhysicsBenchmark, Benchmark_50000_Entities) {
    auto result = run_benchmark(50000, 10);

    std::cout << "Benchmark Results (50000 entities):\n";
    std::cout << "  CPU time: " << result.cpu_time_ms << " ms\n";
    std::cout << "  GPU time: " << result.gpu_time_ms << " ms\n";
    std::cout << "  Speedup:  " << result.speedup << "x\n";

    EXPECT_GT(result.cpu_time_ms, 0.0);
}

// ============================================================================
// String Conversion Tests
// ============================================================================

TEST(HybridPhysicsUtilTest, RoutingStrategyToString) {
    EXPECT_STREQ(to_string(RoutingStrategy::ThresholdBased), "ThresholdBased");
    EXPECT_STREQ(to_string(RoutingStrategy::PerformanceBased), "PerformanceBased");
    EXPECT_STREQ(to_string(RoutingStrategy::ForceCPU), "ForceCPU");
    EXPECT_STREQ(to_string(RoutingStrategy::ForceGPU), "ForceGPU");
    EXPECT_STREQ(to_string(RoutingStrategy::Adaptive), "Adaptive");
}

TEST(HybridPhysicsUtilTest, TransferModeToString) {
    EXPECT_STREQ(to_string(TransferMode::Synchronous), "Synchronous");
    EXPECT_STREQ(to_string(TransferMode::Asynchronous), "Asynchronous");
    EXPECT_STREQ(to_string(TransferMode::PipelinedDouble), "PipelinedDouble");
    EXPECT_STREQ(to_string(TransferMode::PipelinedTriple), "PipelinedTriple");
}

TEST(HybridPhysicsUtilTest, WorkloadTypeToString) {
    EXPECT_STREQ(to_string(WorkloadType::Integration), "Integration");
    EXPECT_STREQ(to_string(WorkloadType::CollisionBroad), "CollisionBroad");
    EXPECT_STREQ(to_string(WorkloadType::Aerodynamics), "Aerodynamics");
    EXPECT_STREQ(to_string(WorkloadType::All), "All");
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
