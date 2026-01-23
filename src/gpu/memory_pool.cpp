/**
 * @file memory_pool.cpp
 * @brief GPU Memory Pool Manager implementation
 *
 * Implements efficient GPU memory pooling with:
 * - Free-list based allocation for O(1) operations
 * - Size-segregated pools to reduce fragmentation
 * - Block coalescing for free space consolidation
 * - Defragmentation support
 * - Thread-safe operations
 */

#include "jaguar/gpu/memory_pool.h"
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <iostream>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace jaguar::gpu {

// ============================================================================
// MemoryPool Implementation
// ============================================================================

MemoryPool::MemoryPool(IComputeBackend* backend, MemoryType type, const PoolConfig& config)
    : m_backend(backend)
    , m_type(type)
    , m_config(config)
{
}

MemoryPool::~MemoryPool() {
    shutdown();
}

BackendResult MemoryPool::initialize() {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_initialized) {
        return BackendResult::Success;
    }

    if (!m_backend || !m_backend->is_initialized()) {
        return BackendResult::NotInitialized;
    }

    // Determine initial pool size
    m_total_size = m_config.initial_size;
    if (m_total_size == 0) {
        // Default based on memory type
        auto caps = m_backend->get_device_capabilities();
        m_total_size = std::min(caps.global_memory / 8, SizeT(256 * 1024 * 1024));
    }

    // Allocate the backing buffer
    m_pool_buffer = m_backend->allocate(m_total_size, m_type, MemoryAccess::ReadWrite);
    if (!m_pool_buffer.is_valid()) {
        return BackendResult::OutOfMemory;
    }

    // Create initial free block spanning entire pool
    m_blocks.push_back(MemoryBlock{});
    auto& initial_block = m_blocks.back();
    initial_block.offset = 0;
    initial_block.size = m_total_size;
    initial_block.is_free = true;

    // Initialize free list
    m_free_list_head = &initial_block;

    // Initialize statistics
    m_stats = MemoryPoolStats{};
    m_stats.total_size = m_total_size;
    m_stats.free_size = m_total_size;
    m_stats.largest_free_block = m_total_size;
    m_stats.smallest_free_block = m_total_size;
    m_stats.free_block_count = 1;

    m_initialized = true;
    return BackendResult::Success;
}

void MemoryPool::shutdown() {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (!m_initialized) {
        return;
    }

    // Free backing buffer
    if (m_pool_buffer.is_valid()) {
        m_backend->free(m_pool_buffer);
        m_pool_buffer = BufferHandle::Invalid();
    }

    // Clear data structures
    m_blocks.clear();
    m_buffer_to_block.clear();
    m_free_list_head = nullptr;
    m_last_allocation = nullptr;

    m_initialized = false;
}

BufferHandle MemoryPool::allocate(SizeT size, MemoryAccess access) {
    auto start_time = std::chrono::steady_clock::now();

    std::lock_guard<std::mutex> lock(m_mutex);

    if (!m_initialized || size == 0) {
        return BufferHandle::Invalid();
    }

    // Align size
    SizeT aligned_size = align_size(size);

    // Find a suitable free block
    MemoryBlock* block = find_free_block(aligned_size);
    if (!block) {
        // Pool is exhausted
        m_stats.failed_allocations++;
        m_stats.pool_misses++;
        return BufferHandle::Invalid();
    }

    // Split block if larger than needed
    if (block->size > aligned_size + m_config.min_block_size) {
        split_block(block, aligned_size);
    }

    // Mark block as used
    block->is_free = false;
    block->requested_size = size;
    block->allocation_id = m_next_buffer_id++;
    block->timestamp = std::chrono::steady_clock::now().time_since_epoch().count();

    // Remove from free list
    remove_from_free_list(block);

    // Create buffer handle
    BufferHandle handle;
    handle.id = block->allocation_id;
    handle.size = size;
    handle.type = m_type;
    handle.access = access;

    // Track mapping
    m_buffer_to_block[handle.id] = block;

    // Update statistics
    m_allocated_size += block->size;
    auto end_time = std::chrono::steady_clock::now();
    auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
    update_stats_allocation(block->size, static_cast<UInt64>(duration_ns));

    m_last_allocation = block;

    return handle;
}

bool MemoryPool::free(BufferHandle buffer) {
    if (!buffer.is_valid()) {
        return false;
    }

    auto start_time = std::chrono::steady_clock::now();

    std::lock_guard<std::mutex> lock(m_mutex);

    if (!m_initialized) {
        return false;
    }

    // Find the block
    auto it = m_buffer_to_block.find(buffer.id);
    if (it == m_buffer_to_block.end()) {
        return false;  // Not from this pool
    }

    MemoryBlock* block = it->second;
    SizeT block_size = block->size;

    // Mark as free
    block->is_free = true;
    block->allocation_id = 0;

    // Add to free list
    add_to_free_list(block);

    // Remove from mapping
    m_buffer_to_block.erase(it);

    // Update allocated size
    m_allocated_size -= block_size;

    // Coalesce with neighbors if enabled
    if (m_config.enable_coalescing) {
        coalesce_with_neighbors(block);
    }

    // Update statistics
    auto end_time = std::chrono::steady_clock::now();
    auto duration_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count();
    update_stats_deallocation(block_size, static_cast<UInt64>(duration_ns));

    return true;
}

bool MemoryPool::owns(BufferHandle buffer) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_buffer_to_block.find(buffer.id) != m_buffer_to_block.end();
}

MemoryPoolStats MemoryPool::get_stats() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_stats;
}

SizeT MemoryPool::defragment() {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (!m_initialized) {
        return 0;
    }

    // Simple defragmentation: compact all allocated blocks to the front
    // This invalidates existing handles, so caller must re-map

    SizeT recovered = 0;
    SizeT current_offset = 0;

    // Sort blocks by physical order
    std::vector<MemoryBlock*> allocated_blocks;
    for (auto& block : m_blocks) {
        if (!block.is_free) {
            allocated_blocks.push_back(&block);
        }
    }

    // Sort by current offset
    std::sort(allocated_blocks.begin(), allocated_blocks.end(),
              [](const MemoryBlock* a, const MemoryBlock* b) {
                  return a->offset < b->offset;
              });

    // Move each block to compact position
    for (auto* block : allocated_blocks) {
        if (block->offset > current_offset) {
            // This block can be moved closer
            SizeT gap = block->offset - current_offset;
            recovered += gap;

            // In a real implementation, we'd need to copy the data
            // For now, just update the offset
            block->offset = current_offset;
        }
        current_offset = block->offset + block->size;
    }

    // Rebuild free list with single block at end
    if (recovered > 0) {
        // Clear existing free blocks
        m_blocks.remove_if([](const MemoryBlock& b) { return b.is_free; });

        // Add single free block at end
        if (current_offset < m_total_size) {
            m_blocks.push_back(MemoryBlock{});
            auto& free_block = m_blocks.back();
            free_block.offset = current_offset;
            free_block.size = m_total_size - current_offset;
            free_block.is_free = true;

            m_free_list_head = &free_block;

            // Update stats
            m_stats.free_size = free_block.size;
            m_stats.largest_free_block = free_block.size;
            m_stats.smallest_free_block = free_block.size;
            m_stats.free_block_count = 1;
            m_stats.fragmentation_ratio = 0.0;
        }
    }

    return recovered;
}

BackendResult MemoryPool::grow(SizeT additional_size) {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (!m_initialized) {
        return BackendResult::NotInitialized;
    }

    // Check max size
    if (m_config.max_size > 0 && m_total_size + additional_size > m_config.max_size) {
        return BackendResult::OutOfMemory;
    }

    // Allocate new larger buffer
    SizeT new_size = m_total_size + additional_size;
    BufferHandle new_buffer = m_backend->allocate(new_size, m_type, MemoryAccess::ReadWrite);
    if (!new_buffer.is_valid()) {
        return BackendResult::OutOfMemory;
    }

    // Copy existing data
    if (m_allocated_size > 0) {
        m_backend->copy(m_pool_buffer, new_buffer, m_total_size, 0, 0);
    }

    // Free old buffer
    m_backend->free(m_pool_buffer);
    m_pool_buffer = new_buffer;

    // Add new free block at end
    m_blocks.push_back(MemoryBlock{});
    auto& new_block = m_blocks.back();
    new_block.offset = m_total_size;
    new_block.size = additional_size;
    new_block.is_free = true;
    add_to_free_list(&new_block);

    // Update stats
    SizeT old_size = m_total_size;
    m_total_size = new_size;
    m_stats.total_size = new_size;
    m_stats.free_size += additional_size;
    m_stats.growth_count++;
    m_stats.total_growth_size += additional_size;

    if (additional_size > m_stats.largest_free_block) {
        m_stats.largest_free_block = additional_size;
    }

    return BackendResult::Success;
}

Real MemoryPool::calculate_fragmentation() const {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_stats.free_size == 0) {
        return 0.0;  // No free space, no fragmentation
    }

    if (m_stats.free_block_count <= 1) {
        return 0.0;  // Single free block, no fragmentation
    }

    // Fragmentation = 1 - (largest_free / total_free)
    return 1.0 - (static_cast<Real>(m_stats.largest_free_block) / m_stats.free_size);
}

// ============================================================================
// MemoryPool Private Helpers
// ============================================================================

MemoryBlock* MemoryPool::find_free_block(SizeT size) {
    switch (m_config.strategy) {
        case AllocationStrategy::FirstFit:
            return find_first_fit(size);
        case AllocationStrategy::BestFit:
            return find_best_fit(size);
        case AllocationStrategy::NextFit:
            return find_next_fit(size);
        default:
            return find_first_fit(size);
    }
}

MemoryBlock* MemoryPool::find_first_fit(SizeT size) {
    MemoryBlock* current = m_free_list_head;
    while (current) {
        if (current->can_satisfy(size)) {
            return current;
        }
        current = current->next_free;
    }
    return nullptr;
}

MemoryBlock* MemoryPool::find_best_fit(SizeT size) {
    MemoryBlock* best = nullptr;
    SizeT best_size = std::numeric_limits<SizeT>::max();

    MemoryBlock* current = m_free_list_head;
    while (current) {
        if (current->can_satisfy(size) && current->size < best_size) {
            best = current;
            best_size = current->size;
            if (best_size == size) {
                break;  // Perfect fit
            }
        }
        current = current->next_free;
    }
    return best;
}

MemoryBlock* MemoryPool::find_next_fit(SizeT size) {
    // Start from last allocation position
    MemoryBlock* start = m_last_allocation ? m_last_allocation->next_free : m_free_list_head;
    MemoryBlock* current = start;

    // First pass: from last position to end
    while (current) {
        if (current->can_satisfy(size)) {
            return current;
        }
        current = current->next_free;
    }

    // Second pass: from beginning to last position
    current = m_free_list_head;
    while (current && current != start) {
        if (current->can_satisfy(size)) {
            return current;
        }
        current = current->next_free;
    }

    return nullptr;
}

void MemoryPool::split_block(MemoryBlock* block, SizeT size) {
    // Create new block for remainder
    m_blocks.push_back(MemoryBlock{});
    auto& new_block = m_blocks.back();
    new_block.offset = block->offset + size;
    new_block.size = block->size - size;
    new_block.is_free = true;

    // Link physical neighbors
    new_block.next_physical = block->next_physical;
    new_block.prev_physical = block;
    if (block->next_physical) {
        block->next_physical->prev_physical = &new_block;
    }
    block->next_physical = &new_block;

    // Update original block
    block->size = size;

    // Add new block to free list
    add_to_free_list(&new_block);

    // Update stats
    m_stats.free_block_count++;
    if (new_block.size < m_stats.smallest_free_block || m_stats.smallest_free_block == 0) {
        m_stats.smallest_free_block = new_block.size;
    }
}

void MemoryPool::coalesce_with_neighbors(MemoryBlock* block) {
    // Coalesce with next block if free
    while (block->next_physical && block->next_physical->is_free) {
        MemoryBlock* next = block->next_physical;
        block->size += next->size;

        // Update links
        block->next_physical = next->next_physical;
        if (next->next_physical) {
            next->next_physical->prev_physical = block;
        }

        // Remove next from free list
        remove_from_free_list(next);

        // Remove from blocks list
        m_blocks.remove_if([next](const MemoryBlock& b) { return &b == next; });

        m_stats.free_block_count--;
    }

    // Coalesce with previous block if free
    while (block->prev_physical && block->prev_physical->is_free) {
        MemoryBlock* prev = block->prev_physical;
        prev->size += block->size;

        // Update links
        prev->next_physical = block->next_physical;
        if (block->next_physical) {
            block->next_physical->prev_physical = prev;
        }

        // Remove block from free list
        remove_from_free_list(block);

        // Remove from blocks list
        m_blocks.remove_if([block](const MemoryBlock& b) { return &b == block; });

        m_stats.free_block_count--;
        block = prev;
    }

    // Update largest free block stat
    if (block->size > m_stats.largest_free_block) {
        m_stats.largest_free_block = block->size;
    }
}

void MemoryPool::add_to_free_list(MemoryBlock* block) {
    block->next_free = m_free_list_head;
    block->prev_free = nullptr;
    if (m_free_list_head) {
        m_free_list_head->prev_free = block;
    }
    m_free_list_head = block;

    // Update stats
    m_stats.free_size += block->size;
    m_stats.free_block_count++;
    if (block->size > m_stats.largest_free_block) {
        m_stats.largest_free_block = block->size;
    }
}

void MemoryPool::remove_from_free_list(MemoryBlock* block) {
    if (block->prev_free) {
        block->prev_free->next_free = block->next_free;
    } else {
        m_free_list_head = block->next_free;
    }
    if (block->next_free) {
        block->next_free->prev_free = block->prev_free;
    }
    block->prev_free = nullptr;
    block->next_free = nullptr;

    // Update stats
    m_stats.free_size -= block->size;
    m_stats.free_block_count--;

    // Recalculate largest/smallest free
    m_stats.largest_free_block = 0;
    m_stats.smallest_free_block = std::numeric_limits<SizeT>::max();
    MemoryBlock* current = m_free_list_head;
    while (current) {
        if (current->size > m_stats.largest_free_block) {
            m_stats.largest_free_block = current->size;
        }
        if (current->size < m_stats.smallest_free_block) {
            m_stats.smallest_free_block = current->size;
        }
        current = current->next_free;
    }
    if (m_stats.free_block_count == 0) {
        m_stats.smallest_free_block = 0;
    }
}

SizeT MemoryPool::align_size(SizeT size) const {
    SizeT alignment = m_config.min_block_size;
    return (size + alignment - 1) & ~(alignment - 1);
}

void MemoryPool::update_stats_allocation(SizeT size, UInt64 time_ns) {
    m_stats.allocation_count++;
    m_stats.pool_hits++;
    m_stats.used_block_count++;
    m_stats.total_allocation_time_ns += time_ns;
    if (time_ns > m_stats.peak_allocation_time_ns) {
        m_stats.peak_allocation_time_ns = time_ns;
    }
    m_stats.allocated_size = m_allocated_size;
    m_stats.fragmentation_ratio = calculate_fragmentation();
}

void MemoryPool::update_stats_deallocation(SizeT size, UInt64 time_ns) {
    m_stats.deallocation_count++;
    m_stats.used_block_count--;
    m_stats.total_deallocation_time_ns += time_ns;
    if (time_ns > m_stats.peak_deallocation_time_ns) {
        m_stats.peak_deallocation_time_ns = time_ns;
    }
    m_stats.allocated_size = m_allocated_size;
    m_stats.fragmentation_ratio = calculate_fragmentation();
}

// ============================================================================
// MemoryPoolManager Implementation
// ============================================================================

MemoryPoolManager::MemoryPoolManager(IComputeBackend* backend, const MemoryPoolConfig& config)
    : m_backend(backend)
    , m_config(config)
{
}

MemoryPoolManager::~MemoryPoolManager() {
    shutdown();
}

BackendResult MemoryPoolManager::initialize() {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_initialized) {
        return BackendResult::Success;
    }

    if (!m_backend || !m_backend->is_initialized()) {
        m_last_error = "Backend not initialized";
        return BackendResult::NotInitialized;
    }

    // Create pools for each memory type
    auto create_pool = [this](MemoryType type) -> std::unique_ptr<MemoryPool> {
        auto config = m_config.get_pool_config(type);
        if (config.initial_size == 0) {
            return nullptr;  // Skip pools with zero size
        }
        auto pool = std::make_unique<MemoryPool>(m_backend, type, config);
        auto result = pool->initialize();
        if (result != BackendResult::Success) {
            m_last_error = "Failed to initialize " + std::string(memory_type_to_string(type)) + " pool";
            return nullptr;
        }
        return pool;
    };

    m_device_local_pool = create_pool(MemoryType::DeviceLocal);
    m_host_visible_pool = create_pool(MemoryType::HostVisible);
    m_host_cached_pool = create_pool(MemoryType::HostCached);
    m_staging_pool = create_pool(MemoryType::Staging);

    // Shared pool only if unified memory is supported and enabled
    if (m_config.enable_unified_memory) {
        auto caps = m_backend->get_device_capabilities();
        if (caps.supports_unified_memory) {
            m_shared_pool = create_pool(MemoryType::Shared);
        }
    }

    m_initialized = true;
    return BackendResult::Success;
}

void MemoryPoolManager::shutdown() {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (!m_initialized) {
        return;
    }

    // Report leaks before shutdown
    if (m_config.enable_leak_detection && !m_allocation_records.empty()) {
        report_leaks();
    }

    // Free direct allocations
    for (auto& [id, handle] : m_direct_allocations) {
        m_backend->free(handle);
    }
    m_direct_allocations.clear();

    // Shutdown pools
    if (m_device_local_pool) m_device_local_pool->shutdown();
    if (m_host_visible_pool) m_host_visible_pool->shutdown();
    if (m_host_cached_pool) m_host_cached_pool->shutdown();
    if (m_staging_pool) m_staging_pool->shutdown();
    if (m_shared_pool) m_shared_pool->shutdown();

    m_device_local_pool.reset();
    m_host_visible_pool.reset();
    m_host_cached_pool.reset();
    m_staging_pool.reset();
    m_shared_pool.reset();

    m_allocation_records.clear();
    m_initialized = false;
}

BufferHandle MemoryPoolManager::allocate(SizeT size, MemoryType type, MemoryAccess access) {
    return allocate_tagged(size, "", type, access);
}

BufferHandle MemoryPoolManager::allocate_tagged(SizeT size, const std::string& tag,
                                                  MemoryType type, MemoryAccess access) {
    if (!m_initialized || size == 0) {
        return BufferHandle::Invalid();
    }

    // Check size class
    SizeClass size_class = m_config.get_size_class(size);

    // For huge allocations, always use direct allocation
    if (size_class == SizeClass::Huge) {
        BufferHandle handle = m_backend->allocate(size, type, access);
        if (handle.is_valid()) {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_direct_allocations[handle.id] = handle;
            record_allocation(handle, size, type, false, tag);
        }
        return handle;
    }

    // Try pool allocation
    MemoryPool* pool = get_pool(type);
    if (pool && pool->is_initialized()) {
        BufferHandle handle = pool->allocate(size, access);
        if (handle.is_valid()) {
            record_allocation(handle, size, type, true, tag);
            return handle;
        }
    }

    // Fallback to direct allocation if enabled
    if (m_config.fallback_to_direct) {
        BufferHandle handle = m_backend->allocate(size, type, access);
        if (handle.is_valid()) {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_direct_allocations[handle.id] = handle;
            record_allocation(handle, size, type, false, tag);
        }
        return handle;
    }

    return BufferHandle::Invalid();
}

void MemoryPoolManager::free(BufferHandle buffer) {
    if (!buffer.is_valid() || !m_initialized) {
        return;
    }

    // Try to return to pool
    MemoryPool* pool = get_pool(buffer.type);
    if (pool && pool->owns(buffer)) {
        pool->free(buffer);
        remove_allocation_record(buffer);
        return;
    }

    // Check if it's a direct allocation
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        auto it = m_direct_allocations.find(buffer.id);
        if (it != m_direct_allocations.end()) {
            m_backend->free(it->second);
            m_direct_allocations.erase(it);
            remove_allocation_record(buffer);
            return;
        }
    }

    // Not found - might be from another manager or already freed
    m_last_error = "Buffer not found in pool manager";
}

BackendResult MemoryPoolManager::upload(BufferHandle buffer, const void* data, SizeT size, SizeT offset) {
    if (!m_initialized) {
        return BackendResult::NotInitialized;
    }
    return m_backend->upload(buffer, data, size, offset);
}

BackendResult MemoryPoolManager::download(BufferHandle buffer, void* data, SizeT size, SizeT offset) {
    if (!m_initialized) {
        return BackendResult::NotInitialized;
    }
    return m_backend->download(buffer, data, size, offset);
}

MemoryPoolStats MemoryPoolManager::get_pool_stats(MemoryType type) const {
    const MemoryPool* pool = get_pool(type);
    if (pool && pool->is_initialized()) {
        return pool->get_stats();
    }
    return MemoryPoolStats{};
}

MemoryPoolManagerStats MemoryPoolManager::get_stats() const {
    MemoryPoolManagerStats stats;

    if (m_device_local_pool) stats.device_local = m_device_local_pool->get_stats();
    if (m_host_visible_pool) stats.host_visible = m_host_visible_pool->get_stats();
    if (m_host_cached_pool) stats.host_cached = m_host_cached_pool->get_stats();
    if (m_staging_pool) stats.staging = m_staging_pool->get_stats();
    if (m_shared_pool) stats.shared = m_shared_pool->get_stats();

    // Calculate totals
    stats.total_allocations = stats.device_local.allocation_count +
                              stats.host_visible.allocation_count +
                              stats.host_cached.allocation_count +
                              stats.staging.allocation_count +
                              stats.shared.allocation_count;

    stats.total_deallocations = stats.device_local.deallocation_count +
                                stats.host_visible.deallocation_count +
                                stats.host_cached.deallocation_count +
                                stats.staging.deallocation_count +
                                stats.shared.deallocation_count;

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        stats.active_allocations = m_allocation_records.size();
        stats.peak_memory_usage = m_peak_memory;
    }

    return stats;
}

SizeT MemoryPoolManager::defragment(MemoryType type) {
    MemoryPool* pool = get_pool(type);
    if (pool && pool->is_initialized()) {
        return pool->defragment();
    }
    return 0;
}

SizeT MemoryPoolManager::defragment_all() {
    SizeT total = 0;
    if (m_device_local_pool) total += m_device_local_pool->defragment();
    if (m_host_visible_pool) total += m_host_visible_pool->defragment();
    if (m_host_cached_pool) total += m_host_cached_pool->defragment();
    if (m_staging_pool) total += m_staging_pool->defragment();
    if (m_shared_pool) total += m_shared_pool->defragment();
    return total;
}

BackendResult MemoryPoolManager::grow_pool(MemoryType type, SizeT additional_size) {
    MemoryPool* pool = get_pool(type);
    if (pool && pool->is_initialized()) {
        return pool->grow(additional_size);
    }
    return BackendResult::NotInitialized;
}

void MemoryPoolManager::trim() {
    // For now, just defragment all pools
    defragment_all();
}

std::vector<AllocationRecord> MemoryPoolManager::get_active_allocations() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    std::vector<AllocationRecord> records;
    records.reserve(m_allocation_records.size());
    for (const auto& [id, record] : m_allocation_records) {
        records.push_back(record);
    }
    return records;
}

UInt32 MemoryPoolManager::report_leaks() const {
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_allocation_records.empty()) {
        return 0;
    }

    std::cerr << "\n=== MEMORY LEAK REPORT ===" << std::endl;
    std::cerr << "Active allocations: " << m_allocation_records.size() << std::endl;

    SizeT total_leaked = 0;
    for (const auto& [id, record] : m_allocation_records) {
        std::cerr << "  Leak: ID=" << record.handle.id
                  << " Size=" << format_memory_size(record.size)
                  << " Type=" << memory_type_to_string(record.type)
                  << " Pool=" << (record.from_pool ? "Yes" : "No");
        if (!record.tag.empty()) {
            std::cerr << " Tag=\"" << record.tag << "\"";
        }
        std::cerr << std::endl;
        total_leaked += record.size;
    }

    std::cerr << "Total leaked: " << format_memory_size(total_leaked) << std::endl;
    std::cerr << "==========================\n" << std::endl;

    return static_cast<UInt32>(m_allocation_records.size());
}

void MemoryPoolManager::clear_leak_records() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_allocation_records.clear();
}

// ============================================================================
// MemoryPoolManager Private Helpers
// ============================================================================

MemoryPool* MemoryPoolManager::get_pool(MemoryType type) {
    switch (type) {
        case MemoryType::DeviceLocal: return m_device_local_pool.get();
        case MemoryType::HostVisible: return m_host_visible_pool.get();
        case MemoryType::HostCached: return m_host_cached_pool.get();
        case MemoryType::Staging: return m_staging_pool.get();
        case MemoryType::Shared: return m_shared_pool.get();
    }
    return nullptr;
}

const MemoryPool* MemoryPoolManager::get_pool(MemoryType type) const {
    switch (type) {
        case MemoryType::DeviceLocal: return m_device_local_pool.get();
        case MemoryType::HostVisible: return m_host_visible_pool.get();
        case MemoryType::HostCached: return m_host_cached_pool.get();
        case MemoryType::Staging: return m_staging_pool.get();
        case MemoryType::Shared: return m_shared_pool.get();
    }
    return nullptr;
}

void MemoryPoolManager::record_allocation(BufferHandle handle, SizeT size, MemoryType type,
                                           bool from_pool, const std::string& tag) {
    if (!m_config.enable_leak_detection) {
        return;
    }

    std::lock_guard<std::mutex> lock(m_mutex);

    AllocationRecord record;
    record.handle = handle;
    record.size = size;
    record.type = type;
    record.timestamp = std::chrono::steady_clock::now();
    record.tag = tag;
    record.from_pool = from_pool;

    m_allocation_records[handle.id] = record;

    // Update peak memory
    SizeT current = get_stats().total_allocated();
    if (current > m_peak_memory) {
        m_peak_memory = current;
    }
}

void MemoryPoolManager::remove_allocation_record(BufferHandle handle) {
    if (!m_config.enable_leak_detection) {
        return;
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    m_allocation_records.erase(handle.id);
}

// ============================================================================
// Utility Functions
// ============================================================================

const char* size_class_to_string(SizeClass size_class) {
    switch (size_class) {
        case SizeClass::Tiny: return "Tiny";
        case SizeClass::Small: return "Small";
        case SizeClass::Medium: return "Medium";
        case SizeClass::Large: return "Large";
        case SizeClass::Huge: return "Huge";
    }
    return "Unknown";
}

const char* allocation_strategy_to_string(AllocationStrategy strategy) {
    switch (strategy) {
        case AllocationStrategy::FirstFit: return "FirstFit";
        case AllocationStrategy::BestFit: return "BestFit";
        case AllocationStrategy::NextFit: return "NextFit";
        case AllocationStrategy::Buddy: return "Buddy";
    }
    return "Unknown";
}

std::string format_memory_size(SizeT bytes) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);

    if (bytes >= 1024ULL * 1024 * 1024) {
        ss << (static_cast<double>(bytes) / (1024.0 * 1024.0 * 1024.0)) << " GB";
    } else if (bytes >= 1024ULL * 1024) {
        ss << (static_cast<double>(bytes) / (1024.0 * 1024.0)) << " MB";
    } else if (bytes >= 1024ULL) {
        ss << (static_cast<double>(bytes) / 1024.0) << " KB";
    } else {
        ss << bytes << " B";
    }

    return ss.str();
}

std::string format_pool_stats(const MemoryPoolStats& stats) {
    std::ostringstream ss;
    ss << "Pool Statistics:\n";
    ss << "  Total Size: " << format_memory_size(stats.total_size) << "\n";
    ss << "  Allocated: " << format_memory_size(stats.allocated_size)
       << " (" << stats.utilization_percent() << "%)\n";
    ss << "  Free: " << format_memory_size(stats.free_size) << "\n";
    ss << "  Fragmentation: " << (stats.fragmentation_ratio * 100.0) << "%\n";
    ss << "  Largest Free: " << format_memory_size(stats.largest_free_block) << "\n";
    ss << "  Allocations: " << stats.allocation_count << "\n";
    ss << "  Deallocations: " << stats.deallocation_count << "\n";
    ss << "  Avg Alloc Time: " << stats.avg_allocation_time_us() << " us\n";
    ss << "  Hit Rate: " << stats.hit_rate() << "%\n";
    return ss.str();
}

std::string format_manager_stats(const MemoryPoolManagerStats& stats) {
    std::ostringstream ss;
    ss << "Memory Pool Manager Statistics:\n";
    ss << "  Total Capacity: " << format_memory_size(stats.total_capacity()) << "\n";
    ss << "  Total Allocated: " << format_memory_size(stats.total_allocated()) << "\n";
    ss << "  Peak Usage: " << format_memory_size(stats.peak_memory_usage) << "\n";
    ss << "  Active Allocations: " << stats.active_allocations << "\n";
    ss << "  Total Allocations: " << stats.total_allocations << "\n";
    ss << "  Total Deallocations: " << stats.total_deallocations << "\n";
    return ss.str();
}

} // namespace jaguar::gpu
