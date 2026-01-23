/**
 * @file memory_pool.h
 * @brief GPU Memory Pool Manager for efficient buffer allocation
 *
 * This file defines the memory pooling system for GPU compute backends,
 * providing efficient buffer allocation with:
 * - Pre-allocated memory pools per MemoryType
 * - Size-segregated allocation for reduced fragmentation
 * - Free-list based O(1) allocation/deallocation
 * - Defragmentation and compaction support
 * - Unified memory detection and optimization
 * - Memory leak detection and diagnostics
 *
 * Design Principles:
 * 1. Wrap existing IComputeBackend allocation calls
 * 2. Minimize GPU allocation API calls (expensive)
 * 3. Support multiple allocation strategies per pool
 * 4. Thread-safe for concurrent physics kernel execution
 * 5. Zero-overhead fallback when pools are exhausted
 *
 * Usage:
 * @code
 * auto backend = BackendFactory::create_best_available();
 * backend->initialize();
 *
 * MemoryPoolConfig config;
 * config.device_local_pool_size = 512 * 1024 * 1024;  // 512 MB
 * config.enable_leak_detection = true;
 *
 * MemoryPoolManager pool(backend.get(), config);
 * pool.initialize();
 *
 * // Allocations from pool are fast
 * auto buffer = pool.allocate(1024 * sizeof(float), MemoryType::DeviceLocal);
 * pool.free(buffer);  // Returns to pool for reuse
 *
 * // Check statistics
 * auto stats = pool.get_stats(MemoryType::DeviceLocal);
 * std::cout << "Pool utilization: " << stats.utilization_percent() << "%" << std::endl;
 * @endcode
 */

#pragma once

#include "jaguar/gpu/compute_backend.h"
#include <mutex>
#include <atomic>
#include <chrono>
#include <unordered_map>
#include <vector>
#include <list>
#include <set>

namespace jaguar::gpu {

// ============================================================================
// Forward Declarations
// ============================================================================

class MemoryPoolManager;
class MemoryPool;
struct MemoryBlock;

// ============================================================================
// Configuration Structures
// ============================================================================

/**
 * @brief Size class for pool allocation strategy
 *
 * Buffers are allocated from size-segregated pools:
 * - Tiny: < 1KB (scalar buffers)
 * - Small: 1KB - 64KB (small arrays)
 * - Medium: 64KB - 1MB (typical physics buffers)
 * - Large: 1MB - 16MB (large simulations)
 * - Huge: > 16MB (allocated directly, not pooled)
 */
enum class SizeClass : UInt8 {
    Tiny    = 0,    ///< < 1KB
    Small   = 1,    ///< 1KB - 64KB
    Medium  = 2,    ///< 64KB - 1MB
    Large   = 3,    ///< 1MB - 16MB
    Huge    = 4     ///< > 16MB (direct allocation)
};

/**
 * @brief Allocation strategy for a pool
 */
enum class AllocationStrategy : UInt8 {
    FirstFit    = 0,    ///< First block that fits (fast)
    BestFit     = 1,    ///< Smallest block that fits (minimal waste)
    NextFit     = 2,    ///< Continue from last allocation (good locality)
    Buddy       = 3     ///< Power-of-2 buddy system (fast, predictable)
};

/**
 * @brief Defragmentation policy
 */
enum class DefragPolicy : UInt8 {
    Never       = 0,    ///< Never defragment automatically
    OnDemand    = 1,    ///< Defragment when requested
    Periodic    = 2,    ///< Defragment periodically (background)
    Aggressive  = 3     ///< Defragment on every free (slow but compact)
};

/**
 * @brief Configuration for a single memory pool
 */
struct PoolConfig {
    SizeT initial_size{0};              ///< Initial pool size (0 = default)
    SizeT max_size{0};                  ///< Maximum pool size (0 = unlimited)
    SizeT growth_increment{0};          ///< Size to grow by when full (0 = double)
    AllocationStrategy strategy{AllocationStrategy::BestFit};
    DefragPolicy defrag_policy{DefragPolicy::OnDemand};
    SizeT min_block_size{64};           ///< Minimum block size (alignment)
    bool enable_coalescing{true};       ///< Coalesce adjacent free blocks
};

/**
 * @brief Memory pool manager configuration
 */
struct MemoryPoolConfig {
    // Per-type pool sizes
    SizeT device_local_pool_size{256 * 1024 * 1024};    ///< 256 MB default
    SizeT host_visible_pool_size{64 * 1024 * 1024};     ///< 64 MB default
    SizeT host_cached_pool_size{64 * 1024 * 1024};      ///< 64 MB default
    SizeT staging_pool_size{32 * 1024 * 1024};          ///< 32 MB default

    // Global settings
    AllocationStrategy default_strategy{AllocationStrategy::BestFit};
    DefragPolicy default_defrag_policy{DefragPolicy::OnDemand};
    SizeT alignment{256};                               ///< Default alignment
    SizeT min_block_size{64};                           ///< Minimum allocation

    // Size class thresholds
    SizeT tiny_threshold{1024};                         ///< 1 KB
    SizeT small_threshold{64 * 1024};                   ///< 64 KB
    SizeT medium_threshold{1024 * 1024};                ///< 1 MB
    SizeT large_threshold{16 * 1024 * 1024};            ///< 16 MB

    // Behavior settings
    bool enable_leak_detection{false};                  ///< Track allocations for leaks
    bool enable_statistics{true};                       ///< Collect usage statistics
    bool enable_unified_memory{false};                  ///< Prefer unified memory
    bool fallback_to_direct{true};                      ///< Direct alloc if pool exhausted
    bool thread_safe{true};                             ///< Enable thread safety

    // Growth settings
    Real growth_factor{2.0};                            ///< Pool growth multiplier
    SizeT max_growth_size{512 * 1024 * 1024};           ///< Max single growth
    UInt32 max_growth_count{10};                        ///< Max growth operations

    // Defragmentation settings
    Real defrag_threshold{0.3};                         ///< Trigger defrag at 30% fragmentation
    UInt32 defrag_check_interval{100};                  ///< Check every N allocations

    /**
     * @brief Get pool config for a memory type
     */
    PoolConfig get_pool_config(MemoryType type) const {
        PoolConfig config;
        config.strategy = default_strategy;
        config.defrag_policy = default_defrag_policy;
        config.min_block_size = min_block_size;
        config.enable_coalescing = true;

        switch (type) {
            case MemoryType::DeviceLocal:
                config.initial_size = device_local_pool_size;
                break;
            case MemoryType::HostVisible:
                config.initial_size = host_visible_pool_size;
                break;
            case MemoryType::HostCached:
                config.initial_size = host_cached_pool_size;
                break;
            case MemoryType::Staging:
                config.initial_size = staging_pool_size;
                break;
            case MemoryType::Shared:
                config.initial_size = device_local_pool_size / 2;  // Half of device local
                break;
        }

        return config;
    }

    /**
     * @brief Get size class for a given allocation size
     */
    SizeClass get_size_class(SizeT size) const {
        if (size <= tiny_threshold) return SizeClass::Tiny;
        if (size <= small_threshold) return SizeClass::Small;
        if (size <= medium_threshold) return SizeClass::Medium;
        if (size <= large_threshold) return SizeClass::Large;
        return SizeClass::Huge;
    }

    /**
     * @brief Create configuration optimized for physics simulations
     */
    static MemoryPoolConfig PhysicsOptimized() {
        MemoryPoolConfig config;
        config.device_local_pool_size = 512 * 1024 * 1024;  // 512 MB
        config.staging_pool_size = 64 * 1024 * 1024;        // 64 MB
        config.default_strategy = AllocationStrategy::NextFit;  // Good locality
        config.enable_statistics = true;
        return config;
    }

    /**
     * @brief Create configuration optimized for low memory usage
     */
    static MemoryPoolConfig LowMemory() {
        MemoryPoolConfig config;
        config.device_local_pool_size = 64 * 1024 * 1024;   // 64 MB
        config.host_visible_pool_size = 16 * 1024 * 1024;   // 16 MB
        config.staging_pool_size = 8 * 1024 * 1024;         // 8 MB
        config.default_strategy = AllocationStrategy::BestFit;  // Minimize waste
        config.default_defrag_policy = DefragPolicy::Aggressive;
        return config;
    }

    /**
     * @brief Create configuration for debugging
     */
    static MemoryPoolConfig Debug() {
        MemoryPoolConfig config;
        config.enable_leak_detection = true;
        config.enable_statistics = true;
        config.fallback_to_direct = false;  // Fail instead of fallback
        return config;
    }
};

// ============================================================================
// Statistics Structures
// ============================================================================

/**
 * @brief Statistics for a single memory pool
 */
struct MemoryPoolStats {
    // Size metrics
    SizeT total_size{0};                ///< Total pool size
    SizeT allocated_size{0};            ///< Currently allocated
    SizeT free_size{0};                 ///< Currently free
    SizeT largest_free_block{0};        ///< Largest contiguous free block
    SizeT smallest_free_block{0};       ///< Smallest free block

    // Fragmentation metrics
    Real fragmentation_ratio{0.0};      ///< 0.0 = no fragmentation
    UInt32 free_block_count{0};         ///< Number of free blocks
    UInt32 used_block_count{0};         ///< Number of allocated blocks

    // Operation counts
    UInt64 allocation_count{0};         ///< Total allocations
    UInt64 deallocation_count{0};       ///< Total deallocations
    UInt64 failed_allocations{0};       ///< Failed allocation attempts
    UInt64 pool_hits{0};                ///< Allocations served from pool
    UInt64 pool_misses{0};              ///< Allocations that needed direct alloc

    // Timing (nanoseconds)
    UInt64 total_allocation_time_ns{0};
    UInt64 total_deallocation_time_ns{0};
    UInt64 peak_allocation_time_ns{0};
    UInt64 peak_deallocation_time_ns{0};

    // Growth tracking
    UInt32 growth_count{0};             ///< Number of pool growths
    SizeT total_growth_size{0};         ///< Total size added by growth

    /**
     * @brief Calculate utilization percentage
     */
    Real utilization_percent() const {
        return total_size > 0 ?
            (static_cast<Real>(allocated_size) / total_size) * 100.0 : 0.0;
    }

    /**
     * @brief Calculate average allocation time (microseconds)
     */
    Real avg_allocation_time_us() const {
        return allocation_count > 0 ?
            static_cast<Real>(total_allocation_time_ns) / allocation_count / 1000.0 : 0.0;
    }

    /**
     * @brief Calculate pool hit rate
     */
    Real hit_rate() const {
        UInt64 total = pool_hits + pool_misses;
        return total > 0 ?
            static_cast<Real>(pool_hits) / total * 100.0 : 100.0;
    }

    /**
     * @brief Check if fragmentation is high
     */
    bool is_fragmented(Real threshold = 0.3) const {
        return fragmentation_ratio > threshold;
    }
};

/**
 * @brief Aggregated statistics for all pools
 */
struct MemoryPoolManagerStats {
    MemoryPoolStats device_local;
    MemoryPoolStats host_visible;
    MemoryPoolStats host_cached;
    MemoryPoolStats staging;
    MemoryPoolStats shared;

    // Global counters
    UInt64 total_allocations{0};
    UInt64 total_deallocations{0};
    UInt64 active_allocations{0};
    SizeT peak_memory_usage{0};

    /**
     * @brief Get stats for a memory type
     */
    const MemoryPoolStats& get(MemoryType type) const {
        switch (type) {
            case MemoryType::DeviceLocal: return device_local;
            case MemoryType::HostVisible: return host_visible;
            case MemoryType::HostCached: return host_cached;
            case MemoryType::Staging: return staging;
            case MemoryType::Shared: return shared;
        }
        return device_local;  // Default
    }

    /**
     * @brief Get total memory usage across all pools
     */
    SizeT total_allocated() const {
        return device_local.allocated_size +
               host_visible.allocated_size +
               host_cached.allocated_size +
               staging.allocated_size +
               shared.allocated_size;
    }

    /**
     * @brief Get total pool capacity
     */
    SizeT total_capacity() const {
        return device_local.total_size +
               host_visible.total_size +
               host_cached.total_size +
               staging.total_size +
               shared.total_size;
    }
};

// ============================================================================
// Memory Block
// ============================================================================

/**
 * @brief Internal representation of a memory block in a pool
 */
struct MemoryBlock {
    SizeT offset{0};            ///< Offset within pool
    SizeT size{0};              ///< Block size
    SizeT requested_size{0};    ///< Original requested size (before alignment)
    bool is_free{true};         ///< Free or allocated
    UInt64 allocation_id{0};    ///< Unique ID for leak tracking
    UInt64 timestamp{0};        ///< Allocation timestamp (for debugging)

    // Linked list for free list management
    MemoryBlock* prev_physical{nullptr};  ///< Previous block in physical order
    MemoryBlock* next_physical{nullptr};  ///< Next block in physical order
    MemoryBlock* prev_free{nullptr};      ///< Previous free block
    MemoryBlock* next_free{nullptr};      ///< Next free block

    SizeT end_offset() const { return offset + size; }

    bool can_satisfy(SizeT request_size) const {
        return is_free && size >= request_size;
    }
};

/**
 * @brief Allocation record for leak detection
 */
struct AllocationRecord {
    BufferHandle handle;
    SizeT size{0};
    MemoryType type{MemoryType::DeviceLocal};
    std::chrono::steady_clock::time_point timestamp;
    std::string tag;            ///< Optional tag for debugging
    bool from_pool{false};      ///< Whether allocated from pool
};

// ============================================================================
// Memory Pool
// ============================================================================

/**
 * @brief Single memory pool for a specific MemoryType
 *
 * Manages a contiguous block of GPU memory with efficient allocation
 * and deallocation using a free-list based approach.
 */
class MemoryPool {
public:
    MemoryPool(IComputeBackend* backend, MemoryType type, const PoolConfig& config);
    ~MemoryPool();

    // Non-copyable
    MemoryPool(const MemoryPool&) = delete;
    MemoryPool& operator=(const MemoryPool&) = delete;

    /**
     * @brief Initialize the pool
     * @return Success or error code
     */
    BackendResult initialize();

    /**
     * @brief Shutdown and release resources
     */
    void shutdown();

    /**
     * @brief Check if pool is initialized
     */
    bool is_initialized() const { return m_initialized; }

    /**
     * @brief Allocate a buffer from the pool
     * @param size Requested size in bytes
     * @param access Access flags
     * @return Buffer handle or invalid handle on failure
     */
    BufferHandle allocate(SizeT size, MemoryAccess access = MemoryAccess::ReadWrite);

    /**
     * @brief Free a buffer back to the pool
     * @param buffer Buffer to free
     * @return true if freed to pool, false if not from this pool
     */
    bool free(BufferHandle buffer);

    /**
     * @brief Check if a buffer is from this pool
     */
    bool owns(BufferHandle buffer) const;

    /**
     * @brief Get pool statistics
     */
    MemoryPoolStats get_stats() const;

    /**
     * @brief Get memory type
     */
    MemoryType memory_type() const { return m_type; }

    /**
     * @brief Get pool size
     */
    SizeT total_size() const { return m_total_size; }

    /**
     * @brief Get allocated size
     */
    SizeT allocated_size() const { return m_allocated_size; }

    /**
     * @brief Get available size
     */
    SizeT available_size() const { return m_total_size - m_allocated_size; }

    /**
     * @brief Defragment the pool
     *
     * Compacts allocated blocks to reduce fragmentation.
     * Note: This invalidates all existing buffer handles!
     *
     * @return Number of bytes recovered
     */
    SizeT defragment();

    /**
     * @brief Try to grow the pool
     * @param additional_size Size to add
     * @return Success or error code
     */
    BackendResult grow(SizeT additional_size);

    /**
     * @brief Calculate fragmentation ratio
     */
    Real calculate_fragmentation() const;

private:
    IComputeBackend* m_backend;
    MemoryType m_type;
    PoolConfig m_config;
    bool m_initialized{false};

    // Pool backing buffer
    BufferHandle m_pool_buffer;
    SizeT m_total_size{0};
    SizeT m_allocated_size{0};

    // Block management
    std::list<MemoryBlock> m_blocks;
    MemoryBlock* m_free_list_head{nullptr};
    MemoryBlock* m_last_allocation{nullptr};  // For NextFit

    // Mapping from buffer ID to block
    std::unordered_map<UInt64, MemoryBlock*> m_buffer_to_block;

    // Statistics
    MemoryPoolStats m_stats;
    std::atomic<UInt64> m_next_buffer_id{1};

    // Thread safety
    mutable std::mutex m_mutex;

    // Internal helpers
    MemoryBlock* find_free_block(SizeT size);
    MemoryBlock* find_first_fit(SizeT size);
    MemoryBlock* find_best_fit(SizeT size);
    MemoryBlock* find_next_fit(SizeT size);
    void split_block(MemoryBlock* block, SizeT size);
    void coalesce_with_neighbors(MemoryBlock* block);
    void add_to_free_list(MemoryBlock* block);
    void remove_from_free_list(MemoryBlock* block);
    SizeT align_size(SizeT size) const;
    void update_stats_allocation(SizeT size, UInt64 time_ns);
    void update_stats_deallocation(SizeT size, UInt64 time_ns);
};

// ============================================================================
// Memory Pool Manager
// ============================================================================

/**
 * @brief Manager for all GPU memory pools
 *
 * MemoryPoolManager provides a unified interface for memory allocation
 * that transparently uses pools for efficient allocation and falls back
 * to direct allocation when necessary.
 */
class MemoryPoolManager {
public:
    /**
     * @brief Construct pool manager with backend
     * @param backend Compute backend (not owned)
     * @param config Pool configuration
     */
    explicit MemoryPoolManager(IComputeBackend* backend,
                                const MemoryPoolConfig& config = {});

    ~MemoryPoolManager();

    // Non-copyable
    MemoryPoolManager(const MemoryPoolManager&) = delete;
    MemoryPoolManager& operator=(const MemoryPoolManager&) = delete;

    // ========================================================================
    // Lifecycle
    // ========================================================================

    /**
     * @brief Initialize all pools
     */
    BackendResult initialize();

    /**
     * @brief Shutdown and release all resources
     */
    void shutdown();

    /**
     * @brief Check if manager is initialized
     */
    bool is_initialized() const { return m_initialized; }

    // ========================================================================
    // Allocation
    // ========================================================================

    /**
     * @brief Allocate a buffer
     *
     * Tries to allocate from the appropriate pool first, then falls back
     * to direct allocation if configured.
     *
     * @param size Size in bytes
     * @param type Memory type
     * @param access Access flags
     * @return Buffer handle or invalid handle on failure
     */
    BufferHandle allocate(SizeT size,
                          MemoryType type = MemoryType::DeviceLocal,
                          MemoryAccess access = MemoryAccess::ReadWrite);

    /**
     * @brief Allocate with a debug tag
     */
    BufferHandle allocate_tagged(SizeT size,
                                  const std::string& tag,
                                  MemoryType type = MemoryType::DeviceLocal,
                                  MemoryAccess access = MemoryAccess::ReadWrite);

    /**
     * @brief Free a buffer
     *
     * Returns the buffer to its pool if pooled, or directly frees it.
     */
    void free(BufferHandle buffer);

    // ========================================================================
    // Convenience Allocation Methods
    // ========================================================================

    /**
     * @brief Allocate buffer for float3 vectors
     */
    BufferHandle allocate_vec3_buffer(SizeT count,
                                       MemoryType type = MemoryType::DeviceLocal) {
        return allocate(count * 3 * sizeof(float), type);
    }

    /**
     * @brief Allocate buffer for float4 vectors
     */
    BufferHandle allocate_vec4_buffer(SizeT count,
                                       MemoryType type = MemoryType::DeviceLocal) {
        return allocate(count * 4 * sizeof(float), type);
    }

    /**
     * @brief Allocate buffer for scalars
     */
    BufferHandle allocate_scalar_buffer(SizeT count,
                                         MemoryType type = MemoryType::DeviceLocal) {
        return allocate(count * sizeof(float), type);
    }

    // ========================================================================
    // Data Transfer (delegates to backend)
    // ========================================================================

    /**
     * @brief Upload data to buffer
     */
    BackendResult upload(BufferHandle buffer, const void* data, SizeT size,
                         SizeT offset = 0);

    /**
     * @brief Download data from buffer
     */
    BackendResult download(BufferHandle buffer, void* data, SizeT size,
                           SizeT offset = 0);

    // ========================================================================
    // Pool Management
    // ========================================================================

    /**
     * @brief Get statistics for a memory type
     */
    MemoryPoolStats get_pool_stats(MemoryType type) const;

    /**
     * @brief Get aggregated statistics
     */
    MemoryPoolManagerStats get_stats() const;

    /**
     * @brief Defragment a specific pool
     * @return Bytes recovered
     */
    SizeT defragment(MemoryType type);

    /**
     * @brief Defragment all pools
     * @return Total bytes recovered
     */
    SizeT defragment_all();

    /**
     * @brief Try to grow a pool
     */
    BackendResult grow_pool(MemoryType type, SizeT additional_size);

    /**
     * @brief Trim unused memory from pools
     */
    void trim();

    // ========================================================================
    // Leak Detection
    // ========================================================================

    /**
     * @brief Get all active allocations (for leak detection)
     */
    std::vector<AllocationRecord> get_active_allocations() const;

    /**
     * @brief Report potential memory leaks
     * @return Number of leaked allocations
     */
    UInt32 report_leaks() const;

    /**
     * @brief Clear leak detection records
     */
    void clear_leak_records();

    // ========================================================================
    // Configuration
    // ========================================================================

    /**
     * @brief Get configuration
     */
    const MemoryPoolConfig& config() const { return m_config; }

    /**
     * @brief Get backend
     */
    IComputeBackend* backend() const { return m_backend; }

    /**
     * @brief Get last error message
     */
    const std::string& last_error() const { return m_last_error; }

private:
    IComputeBackend* m_backend;
    MemoryPoolConfig m_config;
    bool m_initialized{false};
    std::string m_last_error;

    // Pools per memory type
    std::unique_ptr<MemoryPool> m_device_local_pool;
    std::unique_ptr<MemoryPool> m_host_visible_pool;
    std::unique_ptr<MemoryPool> m_host_cached_pool;
    std::unique_ptr<MemoryPool> m_staging_pool;
    std::unique_ptr<MemoryPool> m_shared_pool;

    // Direct allocations (not from pool)
    std::unordered_map<UInt64, BufferHandle> m_direct_allocations;

    // Leak detection
    std::unordered_map<UInt64, AllocationRecord> m_allocation_records;
    std::atomic<UInt64> m_allocation_counter{0};

    // Statistics
    mutable std::mutex m_mutex;
    SizeT m_peak_memory{0};

    // Internal helpers
    MemoryPool* get_pool(MemoryType type);
    const MemoryPool* get_pool(MemoryType type) const;
    void record_allocation(BufferHandle handle, SizeT size, MemoryType type,
                           bool from_pool, const std::string& tag = "");
    void remove_allocation_record(BufferHandle handle);
};

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Get human-readable name for size class
 */
const char* size_class_to_string(SizeClass size_class);

/**
 * @brief Get human-readable name for allocation strategy
 */
const char* allocation_strategy_to_string(AllocationStrategy strategy);

/**
 * @brief Format memory size for display (e.g., "1.5 GB")
 */
std::string format_memory_size(SizeT bytes);

/**
 * @brief Print pool statistics to string
 */
std::string format_pool_stats(const MemoryPoolStats& stats);

/**
 * @brief Print manager statistics to string
 */
std::string format_manager_stats(const MemoryPoolManagerStats& stats);

} // namespace jaguar::gpu
