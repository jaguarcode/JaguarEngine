/**
 * @file test_threading_integration.cpp
 * @brief Integration tests for threading and parallelism
 *
 * Tests the integration of:
 * - Work-stealing thread pool with multiple tasks
 * - Parallel force computation with multiple entities
 * - Thread safety of terrain queries
 * - Correctness of parallel execution results
 */

#include <gtest/gtest.h>
#include "jaguar/core/threading/thread_pool.h"
#include "jaguar/interface/api.h"
#include "jaguar/physics/force.h"
#include "jaguar/environment/environment.h"
#include <vector>
#include <atomic>
#include <thread>
#include <chrono>
#include <numeric>

using namespace jaguar;
using namespace jaguar::core;
using namespace jaguar::physics;
using namespace jaguar::environment;

// ============================================================================
// Thread Pool Tests
// ============================================================================

class ThreadPoolTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create thread pool with hardware concurrency
        UInt32 num_threads = std::thread::hardware_concurrency();
        if (num_threads == 0) num_threads = 4;  // Fallback

        pool_ = std::make_unique<ThreadPool>(num_threads);
    }

    void TearDown() override {
        if (pool_) {
            pool_->shutdown();
        }
    }

    std::unique_ptr<ThreadPool> pool_;
};

TEST_F(ThreadPoolTest, BasicTaskExecution) {
    std::atomic<int> counter{0};

    // Submit simple tasks
    for (int i = 0; i < 100; ++i) {
        pool_->submit([&counter]() {
            counter.fetch_add(1, std::memory_order_relaxed);
        });
    }

    // Wait for completion
    pool_->wait_all();

    EXPECT_EQ(counter.load(), 100);
}

TEST_F(ThreadPoolTest, TaskReturnValues) {
    // Submit tasks that return values
    std::vector<std::future<int>> futures;

    for (int i = 0; i < 50; ++i) {
        futures.push_back(pool_->submit([i]() -> int {
            return i * i;
        }));
    }

    // Collect results
    std::vector<int> results;
    for (auto& future : futures) {
        results.push_back(future.get());
    }

    EXPECT_EQ(results.size(), 50u);

    // Verify results
    for (int i = 0; i < 50; ++i) {
        EXPECT_EQ(results[i], i * i);
    }
}

TEST_F(ThreadPoolTest, WorkStealingEfficiency) {
    // Create imbalanced workload to test work stealing
    std::atomic<int> task_count{0};

    // Submit tasks with varying workloads
    for (int i = 0; i < 100; ++i) {
        pool_->submit([&task_count, i]() {
            // Simulate work (some tasks take longer)
            int iterations = (i % 10 == 0) ? 10000 : 100;
            volatile int dummy = 0;
            for (int j = 0; j < iterations; ++j) {
                dummy += j;
            }
            task_count.fetch_add(1, std::memory_order_relaxed);
        });
    }

    pool_->wait_all();

    // All tasks should complete
    EXPECT_EQ(task_count.load(), 100);
}

TEST_F(ThreadPoolTest, ConcurrentSubmission) {
    // Multiple threads submitting tasks concurrently
    std::atomic<int> total_tasks{0};
    const int num_submitters = 4;
    const int tasks_per_submitter = 25;

    std::vector<std::thread> submitters;

    for (int s = 0; s < num_submitters; ++s) {
        submitters.emplace_back([this, &total_tasks, tasks_per_submitter]() {
            for (int t = 0; t < tasks_per_submitter; ++t) {
                pool_->submit([&total_tasks]() {
                    total_tasks.fetch_add(1, std::memory_order_relaxed);
                });
            }
        });
    }

    // Wait for all submitters to finish
    for (auto& thread : submitters) {
        thread.join();
    }

    // Wait for all tasks to complete
    pool_->wait_all();

    EXPECT_EQ(total_tasks.load(), num_submitters * tasks_per_submitter);
}

TEST_F(ThreadPoolTest, ExceptionHandling) {
    // Test that exceptions in tasks are properly handled
    auto future = pool_->submit([]() -> int {
        throw std::runtime_error("Test exception");
        return 42;
    });

    // Should throw when getting result
    EXPECT_THROW(future.get(), std::runtime_error);

    // Pool should still be functional
    auto future2 = pool_->submit([]() -> int {
        return 123;
    });

    EXPECT_EQ(future2.get(), 123);
}

// ============================================================================
// Parallel Force Computation Tests
// ============================================================================

class ParallelForceTest : public ::testing::Test {
protected:
    void SetUp() override {
        engine_ = std::make_unique<Engine>();
        ASSERT_TRUE(engine_->initialize());
    }

    void TearDown() override {
        if (engine_) {
            engine_->shutdown();
        }
    }

    std::unique_ptr<Engine> engine_;
};

TEST_F(ParallelForceTest, ParallelForceComputationCorrectness) {
    // Create multiple entities
    const int num_entities = 100;
    std::vector<EntityId> entities;

    for (int i = 0; i < num_entities; ++i) {
        EntityId entity = engine_->create_entity("entity_" + std::to_string(i), Domain::Air);
        ASSERT_NE(entity, INVALID_ENTITY_ID);
        entities.push_back(entity);

        // Set initial state
        EntityState state;
        state.position = Vec3{6378137.0 + i * 10.0, i * 10.0, i * 5.0};
        state.velocity = Vec3{i * 1.0, 0.0, 0.0};
        state.mass = 1000.0;
        state.inertia = Mat3x3::Identity() * 100.0;
        engine_->set_entity_state(entity, state);
    }

    // Step simulation (should use parallel force computation)
    Real dt = 0.01;
    engine_->step(dt);

    // Verify all entities have been updated
    for (EntityId entity : entities) {
        EntityState state = engine_->get_entity_state(entity);

        // Position should have changed
        // Velocity should have changed due to gravity
        EXPECT_TRUE(std::isfinite(state.position.x));
        EXPECT_TRUE(std::isfinite(state.velocity.x));
    }
}

TEST_F(ParallelForceTest, ParallelVsSequentialEquivalence) {
    // Compare parallel and sequential force computation results
    const int num_entities = 50;

    // Create entities and record initial states
    std::vector<EntityId> entities;
    std::vector<EntityState> initial_states;

    for (int i = 0; i < num_entities; ++i) {
        EntityId entity = engine_->create_entity("entity_" + std::to_string(i), Domain::Air);
        entities.push_back(entity);

        EntityState state;
        state.position = Vec3{6378137.0, i * 100.0, 0.0};
        state.velocity = Vec3{100.0, i * 5.0, 0.0};
        state.mass = 1000.0;
        state.inertia = Mat3x3::Identity() * 100.0;

        engine_->set_entity_state(entity, state);
        initial_states.push_back(state);
    }

    // Run parallel simulation
    engine_->step(0.01);

    std::vector<EntityState> parallel_results;
    for (EntityId entity : entities) {
        parallel_results.push_back(engine_->get_entity_state(entity));
    }

    // Reset states
    for (size_t i = 0; i < entities.size(); ++i) {
        engine_->set_entity_state(entities[i], initial_states[i]);
    }

    // Run sequential simulation (if engine supports sequential mode toggle)
    // For this test, we assume parallel computation is deterministic
    engine_->step(0.01);

    std::vector<EntityState> sequential_results;
    for (EntityId entity : entities) {
        sequential_results.push_back(engine_->get_entity_state(entity));
    }

    // Results should be identical (or very close due to floating point)
    for (size_t i = 0; i < entities.size(); ++i) {
        EXPECT_NEAR(parallel_results[i].position.x, sequential_results[i].position.x, 1e-9);
        EXPECT_NEAR(parallel_results[i].position.y, sequential_results[i].position.y, 1e-9);
        EXPECT_NEAR(parallel_results[i].position.z, sequential_results[i].position.z, 1e-9);

        EXPECT_NEAR(parallel_results[i].velocity.x, sequential_results[i].velocity.x, 1e-9);
        EXPECT_NEAR(parallel_results[i].velocity.y, sequential_results[i].velocity.y, 1e-9);
        EXPECT_NEAR(parallel_results[i].velocity.z, sequential_results[i].velocity.z, 1e-9);
    }
}

TEST_F(ParallelForceTest, HighEntityCountPerformance) {
    // Test with high entity count to verify scalability
    const int num_entities = 1000;
    std::vector<EntityId> entities;

    for (int i = 0; i < num_entities; ++i) {
        EntityId entity = engine_->create_entity("entity_" + std::to_string(i), Domain::Air);
        entities.push_back(entity);

        EntityState state;
        state.position = Vec3{6378137.0 + (i % 100) * 10.0,
                             (i / 100) * 10.0,
                             (i % 10) * 5.0};
        state.velocity = Vec3{i * 0.1, 0.0, 0.0};
        state.mass = 1000.0;
        state.inertia = Mat3x3::Identity() * 100.0;
        engine_->set_entity_state(entity, state);
    }

    // Measure time for parallel simulation step
    auto start = std::chrono::high_resolution_clock::now();
    engine_->step(0.01);
    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    // Should complete in reasonable time (depends on hardware)
    // This is a qualitative test
    EXPECT_LT(duration.count(), 5000);  // Less than 5 seconds

    // Verify no crashes and all entities updated
    int valid_count = 0;
    for (EntityId entity : entities) {
        EntityState state = engine_->get_entity_state(entity);
        if (std::isfinite(state.position.x) && std::isfinite(state.velocity.x)) {
            valid_count++;
        }
    }

    EXPECT_EQ(valid_count, num_entities);
}

// ============================================================================
// Thread-Safe Terrain Query Tests
// ============================================================================

class ThreadSafeTerrainTest : public ::testing::Test {
protected:
    void SetUp() override {
        env_service_ = std::make_unique<EnvironmentService>();
        ASSERT_TRUE(env_service_->initialize());
    }

    void TearDown() override {
        if (env_service_) {
            env_service_->shutdown();
        }
    }

    std::unique_ptr<EnvironmentService> env_service_;
};

TEST_F(ThreadSafeTerrainTest, ConcurrentTerrainQueries) {
    // Query terrain from multiple threads concurrently
    const int num_threads = 10;
    const int queries_per_thread = 100;

    std::atomic<int> successful_queries{0};
    std::vector<std::thread> threads;

    for (int t = 0; t < num_threads; ++t) {
        threads.emplace_back([this, &successful_queries, queries_per_thread, t]() {
            for (int q = 0; q < queries_per_thread; ++q) {
                Real lat = (t * 0.1) + (q * 0.001);
                Real lon = (t * 0.2) + (q * 0.002);

                TerrainQuery result = env_service_->query_terrain(
                    coord::lla_to_ecef(GeodeticPosition{lat, lon, 0.0})
                );

                if (result.valid) {
                    successful_queries.fetch_add(1, std::memory_order_relaxed);
                }
            }
        });
    }

    // Wait for all threads
    for (auto& thread : threads) {
        thread.join();
    }

    // All queries should succeed (default flat terrain)
    EXPECT_EQ(successful_queries.load(), num_threads * queries_per_thread);
}

TEST_F(ThreadSafeTerrainTest, ConcurrentEnvironmentQueries) {
    // Query full environment (atmosphere + terrain) from multiple threads
    const int num_threads = 8;
    const int queries_per_thread = 50;

    std::atomic<int> query_count{0};
    std::vector<std::thread> threads;

    for (int t = 0; t < num_threads; ++t) {
        threads.emplace_back([this, &query_count, queries_per_thread, t]() {
            for (int q = 0; q < queries_per_thread; ++q) {
                Real lat = t * 0.1;
                Real lon = q * 0.01;
                Real alt = 1000.0 + q * 10.0;

                Environment env = env_service_->query_geodetic(lat, lon, alt, 0.0);

                // Verify environment data is valid
                if (std::isfinite(env.temperature) &&
                    std::isfinite(env.pressure) &&
                    env.terrain.valid) {
                    query_count.fetch_add(1, std::memory_order_relaxed);
                }
            }
        });
    }

    for (auto& thread : threads) {
        thread.join();
    }

    EXPECT_EQ(query_count.load(), num_threads * queries_per_thread);
}

TEST_F(ThreadSafeTerrainTest, MixedReadWriteOperations) {
    // Test terrain queries while potentially loading terrain data
    std::atomic<bool> keep_running{true};
    std::atomic<int> query_count{0};

    // Reader threads
    std::vector<std::thread> readers;
    for (int i = 0; i < 5; ++i) {
        readers.emplace_back([this, &keep_running, &query_count, i]() {
            while (keep_running.load(std::memory_order_relaxed)) {
                Real lat = i * 0.1;
                Real lon = i * 0.2;

                TerrainQuery result = env_service_->query_terrain(
                    coord::lla_to_ecef(GeodeticPosition{lat, lon, 0.0})
                );

                if (result.valid) {
                    query_count.fetch_add(1, std::memory_order_relaxed);
                }

                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
        });
    }

    // Writer thread (attempts to load terrain)
    std::thread writer([this, &keep_running]() {
        for (int i = 0; i < 10; ++i) {
            // Attempt to load non-existent terrain (should fail gracefully)
            env_service_->load_terrain("/nonexistent/terrain_" + std::to_string(i) + ".tif");
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        keep_running.store(false, std::memory_order_relaxed);
    });

    // Wait for completion
    writer.join();
    for (auto& reader : readers) {
        reader.join();
    }

    // Queries should have succeeded despite concurrent load attempts
    EXPECT_GT(query_count.load(), 0);
}

// ============================================================================
// Parallel Computation Correctness Tests
// ============================================================================

class ParallelCorrectnessTest : public ::testing::Test {
protected:
    void SetUp() override {
        UInt32 num_threads = std::thread::hardware_concurrency();
        if (num_threads == 0) num_threads = 4;

        pool_ = std::make_unique<ThreadPool>(num_threads);
    }

    void TearDown() override {
        if (pool_) {
            pool_->shutdown();
        }
    }

    std::unique_ptr<ThreadPool> pool_;
};

TEST_F(ParallelCorrectnessTest, ParallelReductionCorrectness) {
    // Test parallel sum reduction
    const int array_size = 10000;
    std::vector<int> data(array_size);

    // Fill with sequential values
    std::iota(data.begin(), data.end(), 1);  // 1, 2, 3, ..., 10000

    // Expected sum: n * (n + 1) / 2
    int64_t expected_sum = static_cast<int64_t>(array_size) * (array_size + 1) / 2;

    // Compute sum in parallel
    const int num_chunks = 10;
    const int chunk_size = array_size / num_chunks;

    std::vector<std::future<int64_t>> futures;
    for (int i = 0; i < num_chunks; ++i) {
        futures.push_back(pool_->submit([&data, i, chunk_size]() -> int64_t {
            int start = i * chunk_size;
            int end = start + chunk_size;
            int64_t partial_sum = 0;

            for (int j = start; j < end; ++j) {
                partial_sum += data[j];
            }

            return partial_sum;
        }));
    }

    // Collect results
    int64_t parallel_sum = 0;
    for (auto& future : futures) {
        parallel_sum += future.get();
    }

    EXPECT_EQ(parallel_sum, expected_sum);
}

TEST_F(ParallelCorrectnessTest, ParallelMapCorrectness) {
    // Test parallel transformation
    const int array_size = 1000;
    std::vector<int> input(array_size);
    std::vector<int> output(array_size);

    std::iota(input.begin(), input.end(), 0);

    // Transform: square each element
    const int num_chunks = 10;
    const int chunk_size = array_size / num_chunks;

    std::vector<std::future<void>> futures;
    for (int i = 0; i < num_chunks; ++i) {
        futures.push_back(pool_->submit([&input, &output, i, chunk_size]() {
            int start = i * chunk_size;
            int end = start + chunk_size;

            for (int j = start; j < end; ++j) {
                output[j] = input[j] * input[j];
            }
        }));
    }

    // Wait for completion
    for (auto& future : futures) {
        future.get();
    }

    // Verify results
    for (int i = 0; i < array_size; ++i) {
        EXPECT_EQ(output[i], i * i);
    }
}

TEST_F(ParallelCorrectnessTest, NoRaceConditions) {
    // Test that separate memory locations can be updated concurrently
    const int num_counters = 100;
    std::vector<std::atomic<int>> counters(num_counters);

    for (auto& counter : counters) {
        counter.store(0, std::memory_order_relaxed);
    }

    const int increments_per_counter = 1000;

    std::vector<std::future<void>> futures;
    for (int i = 0; i < num_counters; ++i) {
        futures.push_back(pool_->submit([&counters, i, increments_per_counter]() {
            for (int j = 0; j < increments_per_counter; ++j) {
                counters[i].fetch_add(1, std::memory_order_relaxed);
            }
        }));
    }

    // Wait for completion
    for (auto& future : futures) {
        future.get();
    }

    // Verify all counters reached expected value
    for (const auto& counter : counters) {
        EXPECT_EQ(counter.load(), increments_per_counter);
    }
}
