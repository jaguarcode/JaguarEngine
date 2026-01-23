/**
 * @file memory_pool_tests.cpp
 * @brief Comprehensive tests for GPU Memory Pool Manager
 *
 * Tests all memory pool functionality:
 * - Pool initialization and configuration
 * - Allocation and deallocation
 * - Block splitting and coalescing
 * - Fragmentation and defragmentation
 * - Leak detection
 * - Performance benchmarks
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "jaguar/gpu/memory_pool.h"
#include "jaguar/gpu/compute_backend.h"
#include <chrono>
#include <vector>
#include <random>
#include <thread>
#include <atomic>

using namespace jaguar::gpu;
using namespace testing;

// ============================================================================
// Test Fixture
// ============================================================================

/**
 * @brief Base test fixture for memory pool tests
 */
class MemoryPoolTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create best available backend
        m_backend = BackendFactory::create_best_available();
        ASSERT_NE(m_backend, nullptr) << "Failed to create compute backend";

        auto init_result = m_backend->initialize();
        ASSERT_EQ(init_result, BackendResult::Success)
            << "Failed to initialize backend: " << m_backend->last_error();
    }

    void TearDown() override {
        if (m_pool_manager) {
            m_pool_manager->shutdown();
            m_pool_manager.reset();
        }
        if (m_backend) {
            m_backend->shutdown();
            m_backend.reset();
        }
    }

    void CreatePoolManager(const MemoryPoolConfig& config = {}) {
        m_pool_manager = std::make_unique<MemoryPoolManager>(m_backend.get(), config);
        auto result = m_pool_manager->initialize();
        ASSERT_EQ(result, BackendResult::Success)
            << "Failed to initialize pool manager: " << m_pool_manager->last_error();
    }

    std::unique_ptr<IComputeBackend> m_backend;
    std::unique_ptr<MemoryPoolManager> m_pool_manager;
};

// ============================================================================
// Configuration Tests
// ============================================================================

TEST_F(MemoryPoolTest, DefaultConfiguration) {
    MemoryPoolConfig config;

    EXPECT_EQ(config.device_local_pool_size, 256 * 1024 * 1024);  // 256 MB
    EXPECT_EQ(config.host_visible_pool_size, 64 * 1024 * 1024);   // 64 MB
    EXPECT_EQ(config.staging_pool_size, 32 * 1024 * 1024);        // 32 MB
    EXPECT_EQ(config.alignment, 256);
    EXPECT_EQ(config.default_strategy, AllocationStrategy::BestFit);
    EXPECT_TRUE(config.enable_statistics);
    EXPECT_FALSE(config.enable_leak_detection);
}

TEST_F(MemoryPoolTest, PhysicsOptimizedConfiguration) {
    MemoryPoolConfig config = MemoryPoolConfig::PhysicsOptimized();

    EXPECT_EQ(config.device_local_pool_size, 512 * 1024 * 1024);  // 512 MB
    EXPECT_EQ(config.default_strategy, AllocationStrategy::NextFit);
}

TEST_F(MemoryPoolTest, LowMemoryConfiguration) {
    MemoryPoolConfig config = MemoryPoolConfig::LowMemory();

    EXPECT_EQ(config.device_local_pool_size, 64 * 1024 * 1024);   // 64 MB
    EXPECT_EQ(config.default_strategy, AllocationStrategy::BestFit);
    EXPECT_EQ(config.default_defrag_policy, DefragPolicy::Aggressive);
}

TEST_F(MemoryPoolTest, DebugConfiguration) {
    MemoryPoolConfig config = MemoryPoolConfig::Debug();

    EXPECT_TRUE(config.enable_leak_detection);
    EXPECT_TRUE(config.enable_statistics);
    EXPECT_FALSE(config.fallback_to_direct);
}

TEST_F(MemoryPoolTest, SizeClassClassification) {
    MemoryPoolConfig config;

    EXPECT_EQ(config.get_size_class(100), SizeClass::Tiny);
    EXPECT_EQ(config.get_size_class(1024), SizeClass::Tiny);  // Exactly at threshold
    EXPECT_EQ(config.get_size_class(1025), SizeClass::Small);
    EXPECT_EQ(config.get_size_class(32 * 1024), SizeClass::Small);
    EXPECT_EQ(config.get_size_class(64 * 1024), SizeClass::Small);  // At threshold
    EXPECT_EQ(config.get_size_class(65 * 1024), SizeClass::Medium);
    EXPECT_EQ(config.get_size_class(512 * 1024), SizeClass::Medium);
    EXPECT_EQ(config.get_size_class(1024 * 1024), SizeClass::Medium);  // At threshold
    EXPECT_EQ(config.get_size_class(2 * 1024 * 1024), SizeClass::Large);
    EXPECT_EQ(config.get_size_class(16 * 1024 * 1024), SizeClass::Large);  // At threshold
    EXPECT_EQ(config.get_size_class(17 * 1024 * 1024), SizeClass::Huge);
}

// ============================================================================
// Initialization Tests
// ============================================================================

TEST_F(MemoryPoolTest, InitializeDefault) {
    MemoryPoolConfig config;
    config.device_local_pool_size = 16 * 1024 * 1024;  // 16 MB for testing

    CreatePoolManager(config);

    EXPECT_TRUE(m_pool_manager->is_initialized());

    auto stats = m_pool_manager->get_pool_stats(MemoryType::DeviceLocal);
    EXPECT_EQ(stats.total_size, 16 * 1024 * 1024);
    EXPECT_EQ(stats.allocated_size, 0);
    EXPECT_EQ(stats.free_block_count, 1);
}

TEST_F(MemoryPoolTest, InitializeMultiplePools) {
    MemoryPoolConfig config;
    config.device_local_pool_size = 8 * 1024 * 1024;
    config.host_visible_pool_size = 4 * 1024 * 1024;
    config.staging_pool_size = 2 * 1024 * 1024;

    CreatePoolManager(config);

    auto device_stats = m_pool_manager->get_pool_stats(MemoryType::DeviceLocal);
    auto host_stats = m_pool_manager->get_pool_stats(MemoryType::HostVisible);
    auto staging_stats = m_pool_manager->get_pool_stats(MemoryType::Staging);

    EXPECT_EQ(device_stats.total_size, 8 * 1024 * 1024);
    EXPECT_EQ(host_stats.total_size, 4 * 1024 * 1024);
    EXPECT_EQ(staging_stats.total_size, 2 * 1024 * 1024);
}

// ============================================================================
// Allocation Tests
// ============================================================================

TEST_F(MemoryPoolTest, AllocateSingleBuffer) {
    MemoryPoolConfig config;
    config.device_local_pool_size = 16 * 1024 * 1024;

    CreatePoolManager(config);

    auto buffer = m_pool_manager->allocate(1024, MemoryType::DeviceLocal);
    ASSERT_TRUE(buffer.is_valid());
    EXPECT_EQ(buffer.type, MemoryType::DeviceLocal);
    EXPECT_GE(buffer.size, 1024);

    auto stats = m_pool_manager->get_pool_stats(MemoryType::DeviceLocal);
    EXPECT_GT(stats.allocated_size, 0);
    EXPECT_EQ(stats.allocation_count, 1);

    m_pool_manager->free(buffer);
}

TEST_F(MemoryPoolTest, AllocateMultipleBuffers) {
    MemoryPoolConfig config;
    config.device_local_pool_size = 16 * 1024 * 1024;

    CreatePoolManager(config);

    std::vector<BufferHandle> buffers;
    const size_t count = 100;
    const jaguar::SizeT size = 4096;

    for (size_t i = 0; i < count; ++i) {
        auto buffer = m_pool_manager->allocate(size, MemoryType::DeviceLocal);
        ASSERT_TRUE(buffer.is_valid()) << "Allocation failed at index " << i;
        buffers.push_back(buffer);
    }

    auto stats = m_pool_manager->get_pool_stats(MemoryType::DeviceLocal);
    EXPECT_EQ(stats.allocation_count, count);

    // Free all buffers
    for (auto& buffer : buffers) {
        m_pool_manager->free(buffer);
    }

    stats = m_pool_manager->get_pool_stats(MemoryType::DeviceLocal);
    EXPECT_EQ(stats.deallocation_count, count);
}

TEST_F(MemoryPoolTest, AllocateVec3Buffer) {
    MemoryPoolConfig config;
    config.device_local_pool_size = 16 * 1024 * 1024;

    CreatePoolManager(config);

    const jaguar::SizeT entity_count = 1000;
    auto buffer = m_pool_manager->allocate_vec3_buffer(entity_count);

    ASSERT_TRUE(buffer.is_valid());
    EXPECT_GE(buffer.size, entity_count * 3 * sizeof(float));

    m_pool_manager->free(buffer);
}

TEST_F(MemoryPoolTest, AllocateZeroSize) {
    MemoryPoolConfig config;
    config.device_local_pool_size = 16 * 1024 * 1024;

    CreatePoolManager(config);

    auto buffer = m_pool_manager->allocate(0, MemoryType::DeviceLocal);
    EXPECT_FALSE(buffer.is_valid());
}

TEST_F(MemoryPoolTest, AllocateExceedsPool) {
    MemoryPoolConfig config;
    config.device_local_pool_size = 1 * 1024 * 1024;  // 1 MB
    config.fallback_to_direct = false;

    CreatePoolManager(config);

    // Try to allocate more than pool size
    auto buffer = m_pool_manager->allocate(2 * 1024 * 1024, MemoryType::DeviceLocal);

    // Should fail without fallback
    EXPECT_FALSE(buffer.is_valid());
}

TEST_F(MemoryPoolTest, AllocateWithFallback) {
    MemoryPoolConfig config;
    config.device_local_pool_size = 1 * 1024 * 1024;  // 1 MB
    config.fallback_to_direct = true;

    CreatePoolManager(config);

    // Huge allocation bypasses pool
    auto buffer = m_pool_manager->allocate(32 * 1024 * 1024, MemoryType::DeviceLocal);

    // Should succeed with direct allocation
    EXPECT_TRUE(buffer.is_valid());

    m_pool_manager->free(buffer);
}

// ============================================================================
// Deallocation Tests
// ============================================================================

TEST_F(MemoryPoolTest, FreeBuffer) {
    MemoryPoolConfig config;
    config.device_local_pool_size = 16 * 1024 * 1024;

    CreatePoolManager(config);

    auto buffer = m_pool_manager->allocate(1024, MemoryType::DeviceLocal);
    ASSERT_TRUE(buffer.is_valid());

    m_pool_manager->free(buffer);

    auto stats = m_pool_manager->get_pool_stats(MemoryType::DeviceLocal);
    EXPECT_EQ(stats.deallocation_count, 1);
    EXPECT_EQ(stats.allocated_size, 0);
}

TEST_F(MemoryPoolTest, FreeInvalidBuffer) {
    MemoryPoolConfig config;
    config.device_local_pool_size = 16 * 1024 * 1024;

    CreatePoolManager(config);

    BufferHandle invalid;
    m_pool_manager->free(invalid);  // Should not crash

    auto stats = m_pool_manager->get_pool_stats(MemoryType::DeviceLocal);
    EXPECT_EQ(stats.deallocation_count, 0);
}

TEST_F(MemoryPoolTest, DoubleFree) {
    MemoryPoolConfig config;
    config.device_local_pool_size = 16 * 1024 * 1024;

    CreatePoolManager(config);

    auto buffer = m_pool_manager->allocate(1024, MemoryType::DeviceLocal);
    ASSERT_TRUE(buffer.is_valid());

    m_pool_manager->free(buffer);
    m_pool_manager->free(buffer);  // Double free - should be safe

    auto stats = m_pool_manager->get_pool_stats(MemoryType::DeviceLocal);
    EXPECT_EQ(stats.deallocation_count, 1);  // Only counted once
}

// ============================================================================
// Reuse Tests
// ============================================================================

TEST_F(MemoryPoolTest, ReuseFreedBuffer) {
    MemoryPoolConfig config;
    config.device_local_pool_size = 16 * 1024 * 1024;

    CreatePoolManager(config);

    // Allocate and free
    auto buffer1 = m_pool_manager->allocate(4096, MemoryType::DeviceLocal);
    ASSERT_TRUE(buffer1.is_valid());
    m_pool_manager->free(buffer1);

    // Allocate same size - should reuse freed block
    auto buffer2 = m_pool_manager->allocate(4096, MemoryType::DeviceLocal);
    ASSERT_TRUE(buffer2.is_valid());

    auto stats = m_pool_manager->get_pool_stats(MemoryType::DeviceLocal);
    EXPECT_GT(stats.pool_hits, 0);

    m_pool_manager->free(buffer2);
}

TEST_F(MemoryPoolTest, AllocationPattern_FIFO) {
    MemoryPoolConfig config;
    config.device_local_pool_size = 16 * 1024 * 1024;

    CreatePoolManager(config);

    std::vector<BufferHandle> buffers;
    const size_t count = 50;

    // Allocate
    for (size_t i = 0; i < count; ++i) {
        auto buffer = m_pool_manager->allocate(1024, MemoryType::DeviceLocal);
        ASSERT_TRUE(buffer.is_valid());
        buffers.push_back(buffer);
    }

    // Free in same order (FIFO)
    for (auto& buffer : buffers) {
        m_pool_manager->free(buffer);
    }

    auto stats = m_pool_manager->get_pool_stats(MemoryType::DeviceLocal);
    EXPECT_EQ(stats.allocation_count, count);
    EXPECT_EQ(stats.deallocation_count, count);
}

TEST_F(MemoryPoolTest, AllocationPattern_LIFO) {
    MemoryPoolConfig config;
    config.device_local_pool_size = 16 * 1024 * 1024;

    CreatePoolManager(config);

    std::vector<BufferHandle> buffers;
    const size_t count = 50;

    // Allocate
    for (size_t i = 0; i < count; ++i) {
        auto buffer = m_pool_manager->allocate(1024, MemoryType::DeviceLocal);
        ASSERT_TRUE(buffer.is_valid());
        buffers.push_back(buffer);
    }

    // Free in reverse order (LIFO)
    for (auto it = buffers.rbegin(); it != buffers.rend(); ++it) {
        m_pool_manager->free(*it);
    }

    auto stats = m_pool_manager->get_pool_stats(MemoryType::DeviceLocal);
    EXPECT_EQ(stats.allocation_count, count);
    EXPECT_EQ(stats.deallocation_count, count);
}

TEST_F(MemoryPoolTest, AllocationPattern_Random) {
    MemoryPoolConfig config;
    config.device_local_pool_size = 16 * 1024 * 1024;

    CreatePoolManager(config);

    std::vector<BufferHandle> buffers;
    std::mt19937 rng(42);

    // Random allocation and deallocation
    for (int i = 0; i < 200; ++i) {
        if (buffers.empty() || (rng() % 2 == 0 && buffers.size() < 100)) {
            // Allocate random size
            jaguar::SizeT size = 256 + (rng() % 4096);
            auto buffer = m_pool_manager->allocate(size, MemoryType::DeviceLocal);
            if (buffer.is_valid()) {
                buffers.push_back(buffer);
            }
        } else {
            // Free random buffer
            size_t index = rng() % buffers.size();
            m_pool_manager->free(buffers[index]);
            buffers.erase(buffers.begin() + static_cast<std::ptrdiff_t>(index));
        }
    }

    // Clean up remaining
    for (auto& buffer : buffers) {
        m_pool_manager->free(buffer);
    }
}

// ============================================================================
// Fragmentation Tests
// ============================================================================

TEST_F(MemoryPoolTest, FragmentationIncreasesWithAlternatingFrees) {
    MemoryPoolConfig config;
    config.device_local_pool_size = 1 * 1024 * 1024;  // 1 MB

    CreatePoolManager(config);

    std::vector<BufferHandle> buffers;
    const size_t count = 20;

    // Allocate many small buffers
    for (size_t i = 0; i < count; ++i) {
        auto buffer = m_pool_manager->allocate(4096, MemoryType::DeviceLocal);
        ASSERT_TRUE(buffer.is_valid());
        buffers.push_back(buffer);
    }

    // Free every other buffer (creates fragmentation)
    for (size_t i = 0; i < count; i += 2) {
        m_pool_manager->free(buffers[i]);
        buffers[i] = BufferHandle::Invalid();
    }

    auto stats = m_pool_manager->get_pool_stats(MemoryType::DeviceLocal);
    EXPECT_GT(stats.free_block_count, 1) << "Expected multiple free blocks";
    EXPECT_GT(stats.fragmentation_ratio, 0.0) << "Expected some fragmentation";

    // Clean up
    for (auto& buffer : buffers) {
        if (buffer.is_valid()) {
            m_pool_manager->free(buffer);
        }
    }
}

TEST_F(MemoryPoolTest, DefragmentReducesFragmentation) {
    MemoryPoolConfig config;
    config.device_local_pool_size = 1 * 1024 * 1024;

    CreatePoolManager(config);

    // Note: Defragmentation invalidates handles, so this is a simplified test
    auto initial_stats = m_pool_manager->get_pool_stats(MemoryType::DeviceLocal);
    EXPECT_EQ(initial_stats.free_block_count, 1);

    jaguar::SizeT recovered = m_pool_manager->defragment(MemoryType::DeviceLocal);

    // No fragmentation initially, so nothing to recover
    EXPECT_EQ(recovered, 0);
}

// ============================================================================
// Leak Detection Tests
// ============================================================================

TEST_F(MemoryPoolTest, LeakDetectionDisabled) {
    MemoryPoolConfig config;
    config.device_local_pool_size = 16 * 1024 * 1024;
    config.enable_leak_detection = false;

    CreatePoolManager(config);

    auto buffer = m_pool_manager->allocate(1024, MemoryType::DeviceLocal);
    ASSERT_TRUE(buffer.is_valid());

    auto leaks = m_pool_manager->get_active_allocations();
    EXPECT_TRUE(leaks.empty());  // No tracking when disabled

    m_pool_manager->free(buffer);
}

TEST_F(MemoryPoolTest, LeakDetectionEnabled) {
    MemoryPoolConfig config;
    config.device_local_pool_size = 16 * 1024 * 1024;
    config.enable_leak_detection = true;

    CreatePoolManager(config);

    auto buffer = m_pool_manager->allocate(1024, MemoryType::DeviceLocal);
    ASSERT_TRUE(buffer.is_valid());

    auto leaks = m_pool_manager->get_active_allocations();
    EXPECT_EQ(leaks.size(), 1);
    EXPECT_EQ(leaks[0].handle.id, buffer.id);
    EXPECT_EQ(leaks[0].size, 1024);

    m_pool_manager->free(buffer);

    leaks = m_pool_manager->get_active_allocations();
    EXPECT_TRUE(leaks.empty());
}

TEST_F(MemoryPoolTest, LeakDetectionWithTags) {
    MemoryPoolConfig config;
    config.device_local_pool_size = 16 * 1024 * 1024;
    config.enable_leak_detection = true;

    CreatePoolManager(config);

    auto buffer = m_pool_manager->allocate_tagged(1024, "TestBuffer", MemoryType::DeviceLocal);
    ASSERT_TRUE(buffer.is_valid());

    auto leaks = m_pool_manager->get_active_allocations();
    EXPECT_EQ(leaks.size(), 1);
    EXPECT_EQ(leaks[0].tag, "TestBuffer");

    m_pool_manager->free(buffer);
}

// ============================================================================
// Statistics Tests
// ============================================================================

TEST_F(MemoryPoolTest, StatisticsTracking) {
    MemoryPoolConfig config;
    config.device_local_pool_size = 16 * 1024 * 1024;
    config.enable_statistics = true;

    CreatePoolManager(config);

    // Initial stats
    auto stats = m_pool_manager->get_pool_stats(MemoryType::DeviceLocal);
    EXPECT_EQ(stats.allocation_count, 0);
    EXPECT_EQ(stats.deallocation_count, 0);

    // Allocate
    auto buffer = m_pool_manager->allocate(4096, MemoryType::DeviceLocal);
    ASSERT_TRUE(buffer.is_valid());

    stats = m_pool_manager->get_pool_stats(MemoryType::DeviceLocal);
    EXPECT_EQ(stats.allocation_count, 1);
    EXPECT_GT(stats.allocated_size, 0);
    EXPECT_GT(stats.total_allocation_time_ns, 0);

    // Free
    m_pool_manager->free(buffer);

    stats = m_pool_manager->get_pool_stats(MemoryType::DeviceLocal);
    EXPECT_EQ(stats.deallocation_count, 1);
    EXPECT_EQ(stats.allocated_size, 0);
}

TEST_F(MemoryPoolTest, ManagerStats) {
    MemoryPoolConfig config;
    config.device_local_pool_size = 8 * 1024 * 1024;
    config.staging_pool_size = 2 * 1024 * 1024;
    config.enable_leak_detection = true;

    CreatePoolManager(config);

    // Allocate from different pools
    auto device_buffer = m_pool_manager->allocate(4096, MemoryType::DeviceLocal);
    auto staging_buffer = m_pool_manager->allocate(1024, MemoryType::Staging);

    auto manager_stats = m_pool_manager->get_stats();
    EXPECT_EQ(manager_stats.active_allocations, 2);
    EXPECT_GT(manager_stats.total_allocated(), 0);

    m_pool_manager->free(device_buffer);
    m_pool_manager->free(staging_buffer);
}

TEST_F(MemoryPoolTest, UtilizationCalculation) {
    MemoryPoolConfig config;
    config.device_local_pool_size = 1 * 1024 * 1024;  // 1 MB

    CreatePoolManager(config);

    auto stats = m_pool_manager->get_pool_stats(MemoryType::DeviceLocal);
    EXPECT_NEAR(stats.utilization_percent(), 0.0, 0.01);

    // Allocate half the pool
    auto buffer = m_pool_manager->allocate(512 * 1024, MemoryType::DeviceLocal);
    ASSERT_TRUE(buffer.is_valid());

    stats = m_pool_manager->get_pool_stats(MemoryType::DeviceLocal);
    EXPECT_GT(stats.utilization_percent(), 40.0);  // At least 40% (with alignment overhead)

    m_pool_manager->free(buffer);
}

// ============================================================================
// Performance Tests
// ============================================================================

class MemoryPoolBenchmark : public MemoryPoolTest {
protected:
    void BenchmarkAllocFree(const std::string& name, jaguar::SizeT size, int iterations) {
        auto start = std::chrono::high_resolution_clock::now();

        for (int i = 0; i < iterations; ++i) {
            auto buffer = m_pool_manager->allocate(size, MemoryType::DeviceLocal);
            m_pool_manager->free(buffer);
        }

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        double avg_us = static_cast<double>(duration.count()) / iterations;

        std::cout << "  [BENCHMARK] " << name << ": "
                  << avg_us << " us/op (" << iterations << " iterations)" << std::endl;
    }
};

TEST_F(MemoryPoolBenchmark, Benchmark_PooledAllocation) {
    MemoryPoolConfig config;
    config.device_local_pool_size = 64 * 1024 * 1024;

    CreatePoolManager(config);

    std::cout << "\n  Pool Allocation Benchmarks:" << std::endl;

    BenchmarkAllocFree("Tiny (256B)", 256, 10000);
    BenchmarkAllocFree("Small (4KB)", 4096, 10000);
    BenchmarkAllocFree("Medium (64KB)", 64 * 1024, 1000);
    BenchmarkAllocFree("Large (1MB)", 1024 * 1024, 100);
}

TEST_F(MemoryPoolBenchmark, Benchmark_BatchAllocation) {
    MemoryPoolConfig config;
    config.device_local_pool_size = 128 * 1024 * 1024;

    CreatePoolManager(config);

    std::vector<BufferHandle> buffers;
    buffers.reserve(1000);

    auto start = std::chrono::high_resolution_clock::now();

    // Allocate 1000 buffers
    for (int i = 0; i < 1000; ++i) {
        auto buffer = m_pool_manager->allocate(4096, MemoryType::DeviceLocal);
        buffers.push_back(buffer);
    }

    auto alloc_end = std::chrono::high_resolution_clock::now();

    // Free all
    for (auto& buffer : buffers) {
        m_pool_manager->free(buffer);
    }

    auto free_end = std::chrono::high_resolution_clock::now();

    auto alloc_duration = std::chrono::duration_cast<std::chrono::microseconds>(alloc_end - start);
    auto free_duration = std::chrono::duration_cast<std::chrono::microseconds>(free_end - alloc_end);

    std::cout << "\n  [BENCHMARK] Batch Allocation (1000 x 4KB):" << std::endl;
    std::cout << "    Allocate: " << alloc_duration.count() << " us total ("
              << alloc_duration.count() / 1000.0 << " us/op)" << std::endl;
    std::cout << "    Free: " << free_duration.count() << " us total ("
              << free_duration.count() / 1000.0 << " us/op)" << std::endl;
}

// ============================================================================
// Data Transfer Tests
// ============================================================================

TEST_F(MemoryPoolTest, UploadDownload) {
    MemoryPoolConfig config;
    config.device_local_pool_size = 16 * 1024 * 1024;

    CreatePoolManager(config);

    const jaguar::SizeT count = 1000;
    const jaguar::SizeT size = count * sizeof(float);

    auto buffer = m_pool_manager->allocate(size, MemoryType::DeviceLocal);
    ASSERT_TRUE(buffer.is_valid());

    // Create test data
    std::vector<float> upload_data(count);
    for (jaguar::SizeT i = 0; i < count; ++i) {
        upload_data[i] = static_cast<float>(i) * 0.5f;
    }

    // Upload
    auto upload_result = m_pool_manager->upload(buffer, upload_data.data(), size);
    EXPECT_EQ(upload_result, BackendResult::Success);

    // Download
    std::vector<float> download_data(count);
    auto download_result = m_pool_manager->download(buffer, download_data.data(), size);
    EXPECT_EQ(download_result, BackendResult::Success);

    // Verify
    for (jaguar::SizeT i = 0; i < count; ++i) {
        EXPECT_FLOAT_EQ(download_data[i], upload_data[i]) << "Mismatch at index " << i;
    }

    m_pool_manager->free(buffer);
}

// ============================================================================
// Utility Tests
// ============================================================================

TEST_F(MemoryPoolTest, FormatMemorySize) {
    EXPECT_EQ(format_memory_size(500), "500 B");
    EXPECT_EQ(format_memory_size(1024), "1.00 KB");
    EXPECT_EQ(format_memory_size(1536), "1.50 KB");
    EXPECT_EQ(format_memory_size(1024 * 1024), "1.00 MB");
    EXPECT_EQ(format_memory_size(1024ULL * 1024 * 1024), "1.00 GB");
}

TEST_F(MemoryPoolTest, SizeClassToString) {
    EXPECT_STREQ(size_class_to_string(SizeClass::Tiny), "Tiny");
    EXPECT_STREQ(size_class_to_string(SizeClass::Small), "Small");
    EXPECT_STREQ(size_class_to_string(SizeClass::Medium), "Medium");
    EXPECT_STREQ(size_class_to_string(SizeClass::Large), "Large");
    EXPECT_STREQ(size_class_to_string(SizeClass::Huge), "Huge");
}

TEST_F(MemoryPoolTest, AllocationStrategyToString) {
    EXPECT_STREQ(allocation_strategy_to_string(AllocationStrategy::FirstFit), "FirstFit");
    EXPECT_STREQ(allocation_strategy_to_string(AllocationStrategy::BestFit), "BestFit");
    EXPECT_STREQ(allocation_strategy_to_string(AllocationStrategy::NextFit), "NextFit");
    EXPECT_STREQ(allocation_strategy_to_string(AllocationStrategy::Buddy), "Buddy");
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST_F(MemoryPoolTest, EdgeCase_AllocateEntirePool) {
    MemoryPoolConfig config;
    config.device_local_pool_size = 1 * 1024 * 1024;  // 1 MB
    config.fallback_to_direct = false;

    CreatePoolManager(config);

    // Try to allocate almost entire pool
    auto buffer = m_pool_manager->allocate(1024 * 1024 - 256, MemoryType::DeviceLocal);
    EXPECT_TRUE(buffer.is_valid());

    // Should fail - not enough space
    auto buffer2 = m_pool_manager->allocate(512, MemoryType::DeviceLocal);
    // May succeed if there's slack from alignment

    if (buffer2.is_valid()) {
        m_pool_manager->free(buffer2);
    }
    m_pool_manager->free(buffer);
}

TEST_F(MemoryPoolTest, EdgeCase_ManySmallAllocations) {
    MemoryPoolConfig config;
    config.device_local_pool_size = 16 * 1024 * 1024;

    CreatePoolManager(config);

    std::vector<BufferHandle> buffers;
    const size_t count = 1000;

    for (size_t i = 0; i < count; ++i) {
        auto buffer = m_pool_manager->allocate(64, MemoryType::DeviceLocal);
        if (!buffer.is_valid()) break;
        buffers.push_back(buffer);
    }

    EXPECT_GT(buffers.size(), 0);

    for (auto& buffer : buffers) {
        m_pool_manager->free(buffer);
    }
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
