/**
 * @file hybrid_physics.cpp
 * @brief Implementation of hybrid CPU/GPU physics system
 */

#include "jaguar/gpu/hybrid_physics.h"
#include <algorithm>
#include <cmath>
#include <cstring>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace jaguar::gpu {

// ============================================================================
// Constants
// ============================================================================

namespace {

// Default workload thresholds (entity count)
constexpr SizeT DEFAULT_INTEGRATION_THRESHOLD = 200;
constexpr SizeT DEFAULT_COLLISION_THRESHOLD = 500;
constexpr SizeT DEFAULT_AERO_THRESHOLD = 100;
constexpr SizeT DEFAULT_TERRAIN_THRESHOLD = 200;

// Time estimation coefficients (microseconds per entity)
constexpr Real CPU_INTEGRATION_US_PER_ENTITY = 0.5;
constexpr Real CPU_COLLISION_US_PER_ENTITY = 2.0;
constexpr Real GPU_INTEGRATION_US_PER_ENTITY = 0.05;
constexpr Real GPU_COLLISION_US_PER_ENTITY = 0.1;
constexpr Real GPU_TRANSFER_OVERHEAD_US = 500.0;
constexpr Real GPU_DISPATCH_OVERHEAD_US = 100.0;

// Buffer growth factor
constexpr Real BUFFER_GROWTH_FACTOR = 1.5;

// Statistics smoothing
constexpr Real STATS_EMA_ALPHA = 0.1;

} // anonymous namespace

// ============================================================================
// DefaultCPUPhysicsProcessor Implementation
// ============================================================================

void DefaultCPUPhysicsProcessor::integrate_positions(
    physics::EntityStateStorage& storage,
    const std::vector<UInt32>& indices,
    Real dt)
{
    const SizeT count = indices.size();

#ifdef _OPENMP
    #pragma omp parallel for schedule(static)
#endif
    for (Int64 omp_i = 0; omp_i < static_cast<Int64>(count); ++omp_i) { SizeT i = static_cast<SizeT>(omp_i);
        const UInt32 idx = indices[i];
        if (storage.is_active(idx)) {
            Vec3& pos = storage.position(idx);
            const Vec3& vel = storage.velocity(idx);
            pos.x += vel.x * dt;
            pos.y += vel.y * dt;
            pos.z += vel.z * dt;
        }
    }
}

void DefaultCPUPhysicsProcessor::integrate_velocities(
    physics::EntityStateStorage& storage,
    const std::vector<UInt32>& indices,
    Real dt)
{
    const SizeT count = indices.size();

#ifdef _OPENMP
    #pragma omp parallel for schedule(static)
#endif
    for (Int64 omp_i = 0; omp_i < static_cast<Int64>(count); ++omp_i) { SizeT i = static_cast<SizeT>(omp_i);
        const UInt32 idx = indices[i];
        if (storage.is_active(idx)) {
            Vec3& vel = storage.velocity(idx);
            const Vec3& acc = storage.acceleration(idx);
            vel.x += acc.x * dt;
            vel.y += acc.y * dt;
            vel.z += acc.z * dt;
        }
    }
}

void DefaultCPUPhysicsProcessor::compute_accelerations(
    physics::EntityStateStorage& storage,
    const std::vector<UInt32>& indices)
{
    const SizeT count = indices.size();

#ifdef _OPENMP
    #pragma omp parallel for schedule(static)
#endif
    for (Int64 omp_i = 0; omp_i < static_cast<Int64>(count); ++omp_i) { SizeT i = static_cast<SizeT>(omp_i);
        const UInt32 idx = indices[i];
        if (storage.is_active(idx)) {
            const Real mass = storage.mass(idx);
            if (mass > 0.0) {
                const Real inv_mass = 1.0 / mass;
                const auto& forces = storage.forces(idx);
                Vec3& acc = storage.acceleration(idx);
                acc.x = forces.force.x * inv_mass;
                acc.y = forces.force.y * inv_mass;
                acc.z = forces.force.z * inv_mass;
            }
        }
    }
}

void DefaultCPUPhysicsProcessor::clear_forces(
    physics::EntityStateStorage& storage,
    const std::vector<UInt32>& indices)
{
    const SizeT count = indices.size();

#ifdef _OPENMP
    #pragma omp parallel for schedule(static)
#endif
    for (Int64 omp_i = 0; omp_i < static_cast<Int64>(count); ++omp_i) { SizeT i = static_cast<SizeT>(omp_i);
        const UInt32 idx = indices[i];
        storage.forces(idx).clear();
    }
}

SizeT DefaultCPUPhysicsProcessor::broad_phase_collision(
    const physics::EntityStateStorage& storage,
    const std::vector<UInt32>& indices,
    std::vector<CollisionPair>& pairs)
{
    pairs.clear();

    const SizeT count = indices.size();
    if (count < 2) return 0;

    // Simple O(n^2) broad phase for CPU fallback
    // Real implementation would use spatial hashing
    for (SizeT i = 0; i < count - 1; ++i) {
        const UInt32 idx_a = indices[i];
        if (!storage.is_active(idx_a)) continue;

        const Vec3& pos_a = storage.position(idx_a);

        for (SizeT j = i + 1; j < count; ++j) {
            const UInt32 idx_b = indices[j];
            if (!storage.is_active(idx_b)) continue;

            const Vec3& pos_b = storage.position(idx_b);

            // Simple distance check (would use AABBs in real implementation)
            const Real dx = pos_b.x - pos_a.x;
            const Real dy = pos_b.y - pos_a.y;
            const Real dz = pos_b.z - pos_a.z;
            const Real dist_sq = dx*dx + dy*dy + dz*dz;

            // Assume max extent of 10m for broad phase
            constexpr Real MAX_EXTENT_SQ = 100.0;
            if (dist_sq < MAX_EXTENT_SQ) {
                pairs.push_back({idx_a, idx_b});
            }
        }
    }

    return pairs.size();
}

// ============================================================================
// HybridPhysicsSystem Implementation
// ============================================================================

HybridPhysicsSystem::HybridPhysicsSystem()
    : m_last_adjustment_time(std::chrono::steady_clock::now())
{
    // Initialize workload thresholds with defaults
    m_workload_thresholds.resize(static_cast<size_t>(WorkloadType::All) + 1);
    m_cpu_time_estimates.resize(m_workload_thresholds.size());
    m_gpu_time_estimates.resize(m_workload_thresholds.size());

    m_workload_thresholds[static_cast<size_t>(WorkloadType::Integration)] = DEFAULT_INTEGRATION_THRESHOLD;
    m_workload_thresholds[static_cast<size_t>(WorkloadType::CollisionBroad)] = DEFAULT_COLLISION_THRESHOLD;
    m_workload_thresholds[static_cast<size_t>(WorkloadType::CollisionNarrow)] = DEFAULT_COLLISION_THRESHOLD;
    m_workload_thresholds[static_cast<size_t>(WorkloadType::Aerodynamics)] = DEFAULT_AERO_THRESHOLD;
    m_workload_thresholds[static_cast<size_t>(WorkloadType::TerrainSampling)] = DEFAULT_TERRAIN_THRESHOLD;
    m_workload_thresholds[static_cast<size_t>(WorkloadType::WaveSimulation)] = DEFAULT_TERRAIN_THRESHOLD;
    m_workload_thresholds[static_cast<size_t>(WorkloadType::ForceAccumulation)] = DEFAULT_INTEGRATION_THRESHOLD;
    m_workload_thresholds[static_cast<size_t>(WorkloadType::All)] = DEFAULT_INTEGRATION_THRESHOLD;
}

HybridPhysicsSystem::~HybridPhysicsSystem()
{
    shutdown();
}

BackendResult HybridPhysicsSystem::initialize(
    const HybridPhysicsConfig& config,
    physics::EntityStateStorage* entity_storage,
    std::unique_ptr<ICPUPhysicsProcessor> cpu_processor)
{
    // Create best available backend
    auto backend = BackendFactory::create_best_available();
    if (!backend) {
        // Fallback to CPU backend
        backend = BackendFactory::create(BackendType::CPU);
    }

    return initialize(config, entity_storage, std::move(backend), std::move(cpu_processor));
}

BackendResult HybridPhysicsSystem::initialize(
    const HybridPhysicsConfig& config,
    physics::EntityStateStorage* entity_storage,
    std::unique_ptr<IComputeBackend> backend,
    std::unique_ptr<ICPUPhysicsProcessor> cpu_processor)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_initialized) {
        return BackendResult::NotSupported;  // Already initialized
    }

    if (!entity_storage) {
        return BackendResult::InvalidArgument;
    }

    m_config = config;
    m_entity_storage = entity_storage;
    m_backend = std::move(backend);

    // Initialize CPU processor
    if (cpu_processor) {
        m_cpu_processor = std::move(cpu_processor);
    } else {
        m_cpu_processor = std::make_unique<DefaultCPUPhysicsProcessor>();
    }

    // Initialize GPU resources if backend is available and not CPU-only
    if (m_backend && m_backend->type() != BackendType::CPU) {
        // Initialize backend
        auto result = m_backend->initialize();
        if (result != BackendResult::Success) {
            // Fall back to CPU-only mode
            m_backend.reset();
        } else {
            // Create kernel manager
            m_kernel_manager = std::make_unique<PhysicsKernelManager>(m_backend.get());
            result = m_kernel_manager->initialize();
            if (result != BackendResult::Success) {
                m_kernel_manager.reset();
                m_backend.reset();
            }

            // Create memory pool if enabled
            if (m_backend && config.use_memory_pool) {
                MemoryPoolConfig pool_config = MemoryPoolConfig::PhysicsOptimized();
                m_memory_pool = std::make_unique<MemoryPoolManager>(m_backend.get(), pool_config);
                result = m_memory_pool->initialize();
                if (result != BackendResult::Success) {
                    m_memory_pool.reset();
                }
            }

            // Create streams for async operations
            if (m_backend && config.transfer_mode != TransferMode::Synchronous) {
                m_upload_stream = m_backend->create_stream();
                m_download_stream = m_backend->create_stream();
                m_compute_stream = m_backend->create_stream();
            }
        }
    }

    // Apply workload-specific thresholds from config
    for (const auto& threshold : config.workload_thresholds) {
        const auto idx = static_cast<size_t>(threshold.type);
        if (idx < m_workload_thresholds.size()) {
            m_workload_thresholds[idx] = threshold.gpu_preferred;
        }
    }

    // Initialize statistics
    m_stats = HybridPhysicsStats{};
    m_stats.workload_stats.resize(static_cast<size_t>(WorkloadType::All) + 1);
    for (size_t i = 0; i <= static_cast<size_t>(WorkloadType::All); ++i) {
        m_stats.workload_stats[i].type = static_cast<WorkloadType>(i);
    }
    m_stats.current_threshold = config.gpu_threshold;

    // Mark all entities as dirty (need initial upload)
    m_all_dirty = true;

    m_initialized = true;
    return BackendResult::Success;
}

void HybridPhysicsSystem::shutdown()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    if (!m_initialized) return;

    // Release GPU buffers
    release_gpu_buffers();

    // Destroy streams
    if (m_backend) {
        if (m_upload_stream.id != 0) {
            m_backend->destroy_stream(m_upload_stream);
            m_upload_stream = {};
        }
        if (m_download_stream.id != 0) {
            m_backend->destroy_stream(m_download_stream);
            m_download_stream = {};
        }
        if (m_compute_stream.id != 0) {
            m_backend->destroy_stream(m_compute_stream);
            m_compute_stream = {};
        }
    }

    // Release managers
    if (m_kernel_manager) {
        m_kernel_manager->shutdown();
        m_kernel_manager.reset();
    }

    if (m_memory_pool) {
        m_memory_pool->shutdown();
        m_memory_pool.reset();
    }

    if (m_backend) {
        m_backend->shutdown();
        m_backend.reset();
    }

    m_cpu_processor.reset();
    m_entity_storage = nullptr;
    m_initialized = false;
}

bool HybridPhysicsSystem::is_gpu_available() const
{
    return m_backend && m_backend->type() != BackendType::CPU && m_kernel_manager;
}

// ============================================================================
// Physics Step
// ============================================================================

BackendResult HybridPhysicsSystem::step(Real dt)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    if (!m_initialized) {
        return BackendResult::NotInitialized;
    }

    m_frame_start_time = std::chrono::steady_clock::now();

    // Build active entity list if not provided
    if (m_active_indices.empty()) {
        const SizeT count = m_entity_storage->size();
        m_active_indices.reserve(count);
        for (UInt32 i = 0; i < count; ++i) {
            if (m_entity_storage->is_active(i)) {
                m_active_indices.push_back(i);
            }
        }
    }

    const SizeT entity_count = m_active_indices.size();
    if (entity_count == 0) {
        return BackendResult::Success;
    }

    // Make routing decision
    m_last_decision = make_routing_decision(WorkloadType::All, entity_count);

    // Notify callback if set
    if (m_routing_callback) {
        m_routing_callback(m_last_decision);
    }

    BackendResult result = BackendResult::Success;

    if (m_last_decision.use_gpu && is_gpu_available()) {
        // GPU path
        result = execute_gpu_integration(dt);
        if (result == BackendResult::Success) {
            m_stats.gpu_only_frames++;
        }
    } else {
        // CPU path
        result = execute_cpu_integration(dt);
        if (result == BackendResult::Success) {
            m_stats.cpu_only_frames++;
        }
    }

    // Update frame statistics
    m_stats.total_frames++;
    auto frame_end = std::chrono::steady_clock::now();
    Real frame_time_ms = std::chrono::duration<Real, std::milli>(
        frame_end - m_frame_start_time).count();
    m_stats.last_frame_time_ms = frame_time_ms;
    m_stats.avg_frame_time_ms = m_stats.avg_frame_time_ms * 0.9 + frame_time_ms * 0.1;
    m_stats.max_frame_time_ms = std::max(m_stats.max_frame_time_ms, frame_time_ms);

    // Check if dynamic adjustment is needed
    if (m_config.enable_dynamic_adjustment) {
        auto now = std::chrono::steady_clock::now();
        Real elapsed = std::chrono::duration<Real>(now - m_last_adjustment_time).count();
        if (elapsed >= m_config.adjustment_interval_sec) {
            update_dynamic_thresholds();
            m_last_adjustment_time = now;
        }
    }

    // Clear force override flags
    m_force_gpu.store(false);
    m_force_cpu.store(false);

    return result;
}

BackendResult HybridPhysicsSystem::execute_workload(WorkloadType type, Real dt)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    if (!m_initialized) {
        return BackendResult::NotInitialized;
    }

    const SizeT entity_count = m_active_indices.size();
    if (entity_count == 0) {
        return BackendResult::Success;
    }

    auto decision = make_routing_decision(type, entity_count);

    if (decision.use_gpu && is_gpu_available()) {
        switch (type) {
            case WorkloadType::Integration:
                return execute_gpu_integration(dt);
            case WorkloadType::CollisionBroad:
                return execute_gpu_collision();
            default:
                return execute_gpu_integration(dt);
        }
    } else {
        switch (type) {
            case WorkloadType::Integration:
                return execute_cpu_integration(dt);
            case WorkloadType::CollisionBroad:
                return execute_cpu_collision();
            default:
                return execute_cpu_integration(dt);
        }
    }
}

void HybridPhysicsSystem::force_next_path(bool use_gpu)
{
    if (use_gpu) {
        m_force_gpu.store(true);
        m_force_cpu.store(false);
    } else {
        m_force_cpu.store(true);
        m_force_gpu.store(false);
    }
}

// ============================================================================
// Routing Decision
// ============================================================================

RoutingDecision HybridPhysicsSystem::make_routing_decision(
    WorkloadType type,
    SizeT entity_count)
{
    RoutingDecision decision;
    decision.workload = type;
    decision.entity_count = entity_count;
    decision.batch_size = entity_count;

    // Check force flags
    if (m_force_cpu.load()) {
        decision.use_gpu = false;
        decision.reason = "Forced CPU path";
        return decision;
    }

    if (m_force_gpu.load()) {
        decision.use_gpu = is_gpu_available();
        decision.reason = decision.use_gpu ? "Forced GPU path" : "GPU forced but unavailable";
        return decision;
    }

    // Check GPU availability
    if (!is_gpu_available()) {
        decision.use_gpu = false;
        decision.reason = "GPU not available";
        return decision;
    }

    // Get threshold for this workload type
    const auto type_idx = static_cast<size_t>(type);
    const SizeT threshold = (type_idx < m_workload_thresholds.size()) ?
        m_workload_thresholds[type_idx] : m_config.gpu_threshold;

    // Estimate execution times
    decision.estimated_cpu_time_ms = estimate_cpu_time(type, entity_count) / 1000.0;
    decision.estimated_gpu_time_ms = estimate_gpu_time(type, entity_count) / 1000.0;

    switch (m_config.strategy) {
        case RoutingStrategy::ForceCPU:
            decision.use_gpu = false;
            decision.reason = "Strategy: Force CPU";
            break;

        case RoutingStrategy::ForceGPU:
            decision.use_gpu = true;
            decision.reason = "Strategy: Force GPU";
            break;

        case RoutingStrategy::ThresholdBased:
            decision.use_gpu = entity_count >= threshold;
            decision.reason = decision.use_gpu ?
                "Entity count >= threshold" : "Entity count < threshold";
            break;

        case RoutingStrategy::PerformanceBased:
            decision.use_gpu = decision.estimated_gpu_time_ms < decision.estimated_cpu_time_ms;
            decision.reason = decision.use_gpu ?
                "GPU estimated faster" : "CPU estimated faster";
            break;

        case RoutingStrategy::Adaptive:
        default:
            // Combine threshold and performance estimation
            if (entity_count < m_config.gpu_min_batch) {
                decision.use_gpu = false;
                decision.reason = "Below minimum GPU batch size";
            } else if (entity_count >= threshold) {
                decision.use_gpu = true;
                decision.reason = "Above threshold - using GPU";
            } else {
                // In transition zone - use performance estimation
                decision.use_gpu = decision.estimated_gpu_time_ms < decision.estimated_cpu_time_ms * 0.8;
                decision.reason = decision.use_gpu ?
                    "Adaptive: GPU significantly faster" : "Adaptive: CPU preferred in transition zone";
            }
            break;
    }

    return decision;
}

Real HybridPhysicsSystem::estimate_cpu_time(WorkloadType type, SizeT count) const
{
    Real us_per_entity = CPU_INTEGRATION_US_PER_ENTITY;

    switch (type) {
        case WorkloadType::Integration:
            us_per_entity = CPU_INTEGRATION_US_PER_ENTITY;
            break;
        case WorkloadType::CollisionBroad:
        case WorkloadType::CollisionNarrow:
            us_per_entity = CPU_COLLISION_US_PER_ENTITY;
            break;
        default:
            us_per_entity = CPU_INTEGRATION_US_PER_ENTITY;
            break;
    }

    // Use measured average if available
    const auto type_idx = static_cast<size_t>(type);
    if (type_idx < m_stats.workload_stats.size() &&
        m_stats.workload_stats[type_idx].avg_cpu_time_us > 0 &&
        m_stats.workload_stats[type_idx].cpu_executions > 10) {
        SizeT avg_entities = m_stats.workload_stats[type_idx].total_entities_processed /
            m_stats.workload_stats[type_idx].cpu_executions;
        if (avg_entities > 0) {
            us_per_entity = m_stats.workload_stats[type_idx].avg_cpu_time_us /
                static_cast<Real>(avg_entities);
        }
    }

    return us_per_entity * static_cast<Real>(count);
}

Real HybridPhysicsSystem::estimate_gpu_time(WorkloadType type, SizeT count) const
{
    Real us_per_entity = GPU_INTEGRATION_US_PER_ENTITY;

    switch (type) {
        case WorkloadType::Integration:
            us_per_entity = GPU_INTEGRATION_US_PER_ENTITY;
            break;
        case WorkloadType::CollisionBroad:
        case WorkloadType::CollisionNarrow:
            us_per_entity = GPU_COLLISION_US_PER_ENTITY;
            break;
        default:
            us_per_entity = GPU_INTEGRATION_US_PER_ENTITY;
            break;
    }

    // Include transfer and dispatch overhead
    Real overhead = GPU_TRANSFER_OVERHEAD_US + GPU_DISPATCH_OVERHEAD_US;

    // Reduce overhead if data is already on GPU
    if (!m_all_dirty && m_dirty_count == 0) {
        overhead = GPU_DISPATCH_OVERHEAD_US;
    }

    // Use measured average if available
    const auto type_idx = static_cast<size_t>(type);
    if (type_idx < m_stats.workload_stats.size() &&
        m_stats.workload_stats[type_idx].avg_gpu_time_us > 0 &&
        m_stats.workload_stats[type_idx].gpu_executions > 10) {
        SizeT avg_entities = m_stats.workload_stats[type_idx].total_entities_processed /
            m_stats.workload_stats[type_idx].gpu_executions;
        if (avg_entities > 0) {
            us_per_entity = m_stats.workload_stats[type_idx].avg_gpu_time_us /
                static_cast<Real>(avg_entities);
        }
    }

    return overhead + us_per_entity * static_cast<Real>(count);
}

void HybridPhysicsSystem::update_dynamic_thresholds()
{
    // Analyze recent statistics to adjust thresholds
    for (size_t i = 0; i < m_stats.workload_stats.size(); ++i) {
        const auto& ws = m_stats.workload_stats[i];

        // Need enough data points for reliable adjustment
        if (ws.cpu_executions < 10 || ws.gpu_executions < 10) continue;

        // Calculate crossover point where GPU becomes faster
        Real cpu_throughput = ws.cpu_throughput_per_us();
        Real gpu_throughput = ws.gpu_throughput_per_us();

        if (cpu_throughput > 0 && gpu_throughput > 0) {
            // Adjust threshold based on performance ratio
            Real ratio = gpu_throughput / cpu_throughput;

            if (ratio > 2.0) {
                // GPU is much faster - lower threshold
                m_workload_thresholds[i] = static_cast<SizeT>(
                    m_workload_thresholds[i] * 0.9);
            } else if (ratio < 0.5) {
                // CPU is faster - raise threshold
                m_workload_thresholds[i] = static_cast<SizeT>(
                    m_workload_thresholds[i] * 1.1);
            }

            // Clamp to reasonable range
            m_workload_thresholds[i] = std::clamp(
                m_workload_thresholds[i],
                m_config.gpu_min_batch,
                SizeT(10000));
        }
    }

    m_stats.threshold_adjustments++;
    m_stats.current_threshold = m_workload_thresholds[static_cast<size_t>(WorkloadType::All)];
}

// ============================================================================
// Entity Management
// ============================================================================

void HybridPhysicsSystem::mark_entities_dirty(const std::vector<UInt32>& indices)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_dirty_flags.size() < m_entity_storage->size()) {
        m_dirty_flags.resize(m_entity_storage->size(), false);
    }

    for (UInt32 idx : indices) {
        if (idx < m_dirty_flags.size() && !m_dirty_flags[idx]) {
            m_dirty_flags[idx] = true;
            m_dirty_count++;
        }
    }
}

void HybridPhysicsSystem::mark_all_dirty()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_all_dirty = true;
}

void HybridPhysicsSystem::set_active_entities(const std::vector<UInt32>& indices)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_active_indices = indices;
}

BackendResult HybridPhysicsSystem::sync_to_cpu()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    if (!is_gpu_available() || !m_gpu_buffers.is_valid()) {
        return BackendResult::Success;
    }

    return batch_download();
}

BackendResult HybridPhysicsSystem::sync_to_gpu()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    if (!is_gpu_available()) {
        return BackendResult::Success;
    }

    return batch_upload();
}

// ============================================================================
// Configuration
// ============================================================================

void HybridPhysicsSystem::set_config(const HybridPhysicsConfig& config)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_config = config;

    // Apply workload thresholds
    for (const auto& threshold : config.workload_thresholds) {
        const auto idx = static_cast<size_t>(threshold.type);
        if (idx < m_workload_thresholds.size()) {
            m_workload_thresholds[idx] = threshold.gpu_preferred;
        }
    }
}

void HybridPhysicsSystem::set_workload_threshold(WorkloadType type, SizeT threshold)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    const auto idx = static_cast<size_t>(type);
    if (idx < m_workload_thresholds.size()) {
        m_workload_thresholds[idx] = threshold;
    }
}

SizeT HybridPhysicsSystem::get_workload_threshold(WorkloadType type) const
{
    const auto idx = static_cast<size_t>(type);
    if (idx < m_workload_thresholds.size()) {
        return m_workload_thresholds[idx];
    }
    return m_config.gpu_threshold;
}

void HybridPhysicsSystem::set_routing_strategy(RoutingStrategy strategy)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_config.strategy = strategy;
}

// ============================================================================
// Statistics
// ============================================================================

void HybridPhysicsSystem::reset_stats()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    m_stats = HybridPhysicsStats{};
    m_stats.workload_stats.resize(static_cast<size_t>(WorkloadType::All) + 1);
    for (size_t i = 0; i <= static_cast<size_t>(WorkloadType::All); ++i) {
        m_stats.workload_stats[i].type = static_cast<WorkloadType>(i);
    }
    m_stats.current_threshold = m_config.gpu_threshold;
}

void HybridPhysicsSystem::set_routing_callback(
    std::function<void(const RoutingDecision&)> callback)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_routing_callback = std::move(callback);
}

void HybridPhysicsSystem::record_cpu_execution(
    WorkloadType type, SizeT count, UInt64 time_us)
{
    const auto idx = static_cast<size_t>(type);
    if (idx >= m_stats.workload_stats.size()) return;

    auto& ws = m_stats.workload_stats[idx];
    ws.cpu_executions++;
    ws.total_entities_processed += count;
    ws.cpu_time_total_us += time_us;

    // EMA for average time
    if (ws.avg_cpu_time_us == 0.0) {
        ws.avg_cpu_time_us = static_cast<Real>(time_us);
    } else {
        ws.avg_cpu_time_us = ws.avg_cpu_time_us * (1.0 - STATS_EMA_ALPHA) +
                            static_cast<Real>(time_us) * STATS_EMA_ALPHA;
    }
}

void HybridPhysicsSystem::record_gpu_execution(
    WorkloadType type, SizeT count, UInt64 time_us)
{
    const auto idx = static_cast<size_t>(type);
    if (idx >= m_stats.workload_stats.size()) return;

    auto& ws = m_stats.workload_stats[idx];
    ws.gpu_executions++;
    ws.total_entities_processed += count;
    ws.gpu_time_total_us += time_us;

    // EMA for average time
    if (ws.avg_gpu_time_us == 0.0) {
        ws.avg_gpu_time_us = static_cast<Real>(time_us);
    } else {
        ws.avg_gpu_time_us = ws.avg_gpu_time_us * (1.0 - STATS_EMA_ALPHA) +
                            static_cast<Real>(time_us) * STATS_EMA_ALPHA;
    }
}

void HybridPhysicsSystem::record_transfer(SizeT bytes, bool upload, UInt64 time_us)
{
    if (upload) {
        m_stats.total_uploads++;
        m_stats.total_bytes_uploaded += bytes;
    } else {
        m_stats.total_downloads++;
        m_stats.total_bytes_downloaded += bytes;
    }

    // Record transfer time in All workload
    const auto idx = static_cast<size_t>(WorkloadType::All);
    if (idx < m_stats.workload_stats.size()) {
        m_stats.workload_stats[idx].transfer_time_total_us += time_us;
        auto& avg = m_stats.workload_stats[idx].avg_transfer_time_us;
        if (avg == 0.0) {
            avg = static_cast<Real>(time_us);
        } else {
            avg = avg * (1.0 - STATS_EMA_ALPHA) + static_cast<Real>(time_us) * STATS_EMA_ALPHA;
        }
    }
}

// ============================================================================
// Buffer Management
// ============================================================================

BackendResult HybridPhysicsSystem::ensure_gpu_buffers(SizeT capacity)
{
    if (!m_backend) {
        return BackendResult::NotInitialized;
    }

    if (m_gpu_buffers.capacity >= capacity && m_gpu_buffers.is_valid()) {
        return BackendResult::Success;
    }

    // Need to resize
    return resize_gpu_buffers(capacity);
}

BackendResult HybridPhysicsSystem::resize_gpu_buffers(SizeT new_capacity)
{
    // Release old buffers
    release_gpu_buffers();

    // Calculate sizes
    const SizeT vec3_size = new_capacity * sizeof(float) * 3;
    const SizeT vec4_size = new_capacity * sizeof(float) * 4;
    const SizeT scalar_size = new_capacity * sizeof(float);
    const SizeT aabb_size = new_capacity * sizeof(AABB);

    // Allocate new buffers
    auto alloc_buffer = [this](SizeT size, MemoryType type) -> BufferHandle {
        if (m_memory_pool) {
            return m_memory_pool->allocate(size, type, MemoryAccess::ReadWrite);
        } else {
            return m_backend->allocate(size, type, MemoryAccess::ReadWrite);
        }
    };

    m_gpu_buffers.positions = alloc_buffer(vec3_size, MemoryType::DeviceLocal);
    m_gpu_buffers.velocities = alloc_buffer(vec3_size, MemoryType::DeviceLocal);
    m_gpu_buffers.orientations = alloc_buffer(vec4_size, MemoryType::DeviceLocal);
    m_gpu_buffers.angular_velocities = alloc_buffer(vec3_size, MemoryType::DeviceLocal);
    m_gpu_buffers.accelerations = alloc_buffer(vec3_size, MemoryType::DeviceLocal);
    m_gpu_buffers.angular_accelerations = alloc_buffer(vec3_size, MemoryType::DeviceLocal);
    m_gpu_buffers.masses = alloc_buffer(scalar_size, MemoryType::DeviceLocal);
    m_gpu_buffers.inv_masses = alloc_buffer(scalar_size, MemoryType::DeviceLocal);
    m_gpu_buffers.forces = alloc_buffer(vec3_size, MemoryType::DeviceLocal);
    m_gpu_buffers.torques = alloc_buffer(vec3_size, MemoryType::DeviceLocal);
    m_gpu_buffers.extents = alloc_buffer(vec3_size, MemoryType::DeviceLocal);
    m_gpu_buffers.aabbs = alloc_buffer(aabb_size, MemoryType::DeviceLocal);

    // Collision pairs buffer (worst case: n^2 / 2)
    const SizeT max_pairs = std::min(new_capacity * new_capacity / 2, SizeT(1000000));
    m_gpu_buffers.collision_pairs = alloc_buffer(max_pairs * sizeof(CollisionPair),
                                                  MemoryType::DeviceLocal);

    // Staging buffers for transfers
    if (m_config.transfer_mode != TransferMode::Synchronous) {
        const SizeT staging_size = m_config.staging_buffer_size;
        m_gpu_buffers.staging_upload = alloc_buffer(staging_size, MemoryType::Staging);
        m_gpu_buffers.staging_download = alloc_buffer(staging_size, MemoryType::Staging);
    }

    m_gpu_buffers.capacity = new_capacity;

    // Update memory stats
    m_stats.gpu_memory_used = vec3_size * 9 + vec4_size + scalar_size * 2 + aabb_size;
    m_stats.gpu_memory_peak = std::max(m_stats.gpu_memory_peak, m_stats.gpu_memory_used);

    // Mark all as dirty since buffers are new
    m_all_dirty = true;

    return BackendResult::Success;
}

void HybridPhysicsSystem::release_gpu_buffers()
{
    if (!m_backend) return;

    auto free_buffer = [this](BufferHandle& buffer) {
        if (buffer.id != 0) {
            if (m_memory_pool) {
                m_memory_pool->free(buffer);
            } else {
                m_backend->free(buffer);
            }
            buffer = {};
        }
    };

    free_buffer(m_gpu_buffers.positions);
    free_buffer(m_gpu_buffers.velocities);
    free_buffer(m_gpu_buffers.orientations);
    free_buffer(m_gpu_buffers.angular_velocities);
    free_buffer(m_gpu_buffers.accelerations);
    free_buffer(m_gpu_buffers.angular_accelerations);
    free_buffer(m_gpu_buffers.masses);
    free_buffer(m_gpu_buffers.inv_masses);
    free_buffer(m_gpu_buffers.forces);
    free_buffer(m_gpu_buffers.torques);
    free_buffer(m_gpu_buffers.extents);
    free_buffer(m_gpu_buffers.aabbs);
    free_buffer(m_gpu_buffers.collision_pairs);
    free_buffer(m_gpu_buffers.staging_upload);
    free_buffer(m_gpu_buffers.staging_download);

    m_gpu_buffers.capacity = 0;
    m_gpu_buffers.entity_count = 0;
}

// ============================================================================
// Transfer Operations
// ============================================================================

BackendResult HybridPhysicsSystem::batch_upload()
{
    const SizeT count = m_active_indices.size();
    if (count == 0) return BackendResult::Success;

    auto start = std::chrono::steady_clock::now();

    // Ensure GPU buffers are large enough
    auto result = ensure_gpu_buffers(static_cast<SizeT>(count * BUFFER_GROWTH_FACTOR));
    if (result != BackendResult::Success) {
        return result;
    }

    // Prepare staging data
    m_staging_positions.resize(count * 3);
    m_staging_velocities.resize(count * 3);

    // Pack positions and velocities into flat arrays
    for (Int64 omp_i = 0; omp_i < static_cast<Int64>(count); ++omp_i) { SizeT i = static_cast<SizeT>(omp_i);
        const UInt32 idx = m_active_indices[i];
        const Vec3& pos = m_entity_storage->position(idx);
        const Vec3& vel = m_entity_storage->velocity(idx);

        m_staging_positions[i * 3 + 0] = static_cast<float>(pos.x);
        m_staging_positions[i * 3 + 1] = static_cast<float>(pos.y);
        m_staging_positions[i * 3 + 2] = static_cast<float>(pos.z);

        m_staging_velocities[i * 3 + 0] = static_cast<float>(vel.x);
        m_staging_velocities[i * 3 + 1] = static_cast<float>(vel.y);
        m_staging_velocities[i * 3 + 2] = static_cast<float>(vel.z);
    }

    // Upload to GPU
    SizeT pos_size = count * sizeof(float) * 3;
    result = m_backend->upload(m_gpu_buffers.positions,
                                m_staging_positions.data(),
                                pos_size,
                                0,
                                m_upload_stream);
    if (result != BackendResult::Success) return result;

    result = m_backend->upload(m_gpu_buffers.velocities,
                                m_staging_velocities.data(),
                                pos_size,
                                0,
                                m_upload_stream);
    if (result != BackendResult::Success) return result;

    // Wait for uploads to complete
    if (m_upload_stream.id != 0) {
        m_backend->synchronize_stream(m_upload_stream);
    }

    m_gpu_buffers.entity_count = count;
    m_all_dirty = false;
    m_dirty_count = 0;
    std::fill(m_dirty_flags.begin(), m_dirty_flags.end(), false);

    auto end = std::chrono::steady_clock::now();
    UInt64 time_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    record_transfer(pos_size * 2, true, time_us);

    return BackendResult::Success;
}

BackendResult HybridPhysicsSystem::batch_download()
{
    const SizeT count = m_gpu_buffers.entity_count;
    if (count == 0 || count > m_active_indices.size()) {
        return BackendResult::Success;
    }

    auto start = std::chrono::steady_clock::now();

    // Ensure staging buffers are large enough
    m_staging_positions.resize(count * 3);
    m_staging_velocities.resize(count * 3);

    // Download from GPU
    SizeT pos_size = count * sizeof(float) * 3;
    auto result = m_backend->download(m_gpu_buffers.positions,
                                       m_staging_positions.data(),
                                       pos_size,
                                       0,
                                       m_download_stream);
    if (result != BackendResult::Success) return result;

    result = m_backend->download(m_gpu_buffers.velocities,
                                  m_staging_velocities.data(),
                                  pos_size,
                                  0,
                                  m_download_stream);
    if (result != BackendResult::Success) return result;

    // Wait for downloads to complete
    if (m_download_stream.id != 0) {
        m_backend->synchronize_stream(m_download_stream);
    }

    // Unpack to entity storage
    for (Int64 omp_i = 0; omp_i < static_cast<Int64>(count); ++omp_i) { SizeT i = static_cast<SizeT>(omp_i);
        const UInt32 idx = m_active_indices[i];
        Vec3& pos = m_entity_storage->position(idx);
        Vec3& vel = m_entity_storage->velocity(idx);

        pos.x = static_cast<Real>(m_staging_positions[i * 3 + 0]);
        pos.y = static_cast<Real>(m_staging_positions[i * 3 + 1]);
        pos.z = static_cast<Real>(m_staging_positions[i * 3 + 2]);

        vel.x = static_cast<Real>(m_staging_velocities[i * 3 + 0]);
        vel.y = static_cast<Real>(m_staging_velocities[i * 3 + 1]);
        vel.z = static_cast<Real>(m_staging_velocities[i * 3 + 2]);
    }

    auto end = std::chrono::steady_clock::now();
    UInt64 time_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    record_transfer(pos_size * 2, false, time_us);

    return BackendResult::Success;
}

BackendResult HybridPhysicsSystem::upload_entities(const std::vector<UInt32>& indices)
{
    // For partial updates - just mark all dirty for now
    // A more sophisticated implementation would do incremental updates
    m_all_dirty = true;
    return batch_upload();
}

BackendResult HybridPhysicsSystem::download_entities(const std::vector<UInt32>& indices)
{
    // For partial downloads - download all for now
    return batch_download();
}

// ============================================================================
// CPU Execution
// ============================================================================

BackendResult HybridPhysicsSystem::execute_cpu_integration(Real dt)
{
    auto start = std::chrono::steady_clock::now();

    // Clear forces, compute accelerations, integrate
    m_cpu_processor->clear_forces(*m_entity_storage, m_active_indices);
    m_cpu_processor->compute_accelerations(*m_entity_storage, m_active_indices);
    m_cpu_processor->integrate_velocities(*m_entity_storage, m_active_indices, dt);
    m_cpu_processor->integrate_positions(*m_entity_storage, m_active_indices, dt);

    auto end = std::chrono::steady_clock::now();
    UInt64 time_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    record_cpu_execution(WorkloadType::Integration, m_active_indices.size(), time_us);

    // Mark GPU buffers as needing update
    m_all_dirty = true;

    return BackendResult::Success;
}

BackendResult HybridPhysicsSystem::execute_cpu_collision()
{
    auto start = std::chrono::steady_clock::now();

    std::vector<CollisionPair> pairs;
    m_cpu_processor->broad_phase_collision(*m_entity_storage, m_active_indices, pairs);

    auto end = std::chrono::steady_clock::now();
    UInt64 time_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    record_cpu_execution(WorkloadType::CollisionBroad, m_active_indices.size(), time_us);

    return BackendResult::Success;
}

// ============================================================================
// GPU Execution
// ============================================================================

BackendResult HybridPhysicsSystem::execute_gpu_integration(Real dt)
{
    auto start = std::chrono::steady_clock::now();

    // Upload data if dirty
    if (m_all_dirty || m_dirty_count > 0) {
        auto result = batch_upload();
        if (result != BackendResult::Success) {
            // Fall back to CPU
            return execute_cpu_integration(dt);
        }
    }

    const SizeT count = m_gpu_buffers.entity_count;

    // Run integration kernels
    auto result = m_kernel_manager->clear_forces(
        m_gpu_buffers.forces, m_gpu_buffers.torques, count);
    if (result != BackendResult::Success) return result;

    result = m_kernel_manager->integrate_velocities(
        m_gpu_buffers.velocities,
        m_gpu_buffers.forces,
        m_gpu_buffers.masses,
        dt, count);
    if (result != BackendResult::Success) return result;

    result = m_kernel_manager->integrate_positions(
        m_gpu_buffers.positions,
        m_gpu_buffers.velocities,
        dt, count);
    if (result != BackendResult::Success) return result;

    // Download results
    result = batch_download();
    if (result != BackendResult::Success) return result;

    auto end = std::chrono::steady_clock::now();
    UInt64 time_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    record_gpu_execution(WorkloadType::Integration, count, time_us);

    return BackendResult::Success;
}

BackendResult HybridPhysicsSystem::execute_gpu_collision()
{
    auto start = std::chrono::steady_clock::now();

    // Upload data if dirty
    if (m_all_dirty || m_dirty_count > 0) {
        auto result = batch_upload();
        if (result != BackendResult::Success) {
            return execute_cpu_collision();
        }
    }

    const SizeT count = m_gpu_buffers.entity_count;

    // Update AABBs
    auto result = m_kernel_manager->update_aabbs(
        m_gpu_buffers.positions,
        m_gpu_buffers.extents,
        m_gpu_buffers.aabbs,
        count);
    if (result != BackendResult::Success) return result;

    // Run broad-phase collision detection
    SizeT pair_count = 0;
    result = m_kernel_manager->collision_broad_phase(
        m_gpu_buffers.aabbs,
        count,
        m_gpu_buffers.collision_pairs,
        &pair_count);
    if (result != BackendResult::Success) return result;

    auto end = std::chrono::steady_clock::now();
    UInt64 time_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    record_gpu_execution(WorkloadType::CollisionBroad, count, time_us);

    return BackendResult::Success;
}

} // namespace jaguar::gpu
