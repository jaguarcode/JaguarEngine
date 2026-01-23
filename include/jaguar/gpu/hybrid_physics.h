/**
 * @file hybrid_physics.h
 * @brief Hybrid CPU/GPU physics system with automatic workload distribution
 *
 * This file defines the HybridPhysicsSystem that intelligently routes physics
 * computations between CPU and GPU based on entity count, workload characteristics,
 * and runtime performance metrics.
 *
 * Design Principles:
 * 1. Automatic workload routing based on configurable thresholds
 * 2. Dynamic threshold adjustment using runtime performance metrics
 * 3. Efficient CPU-GPU synchronization with async transfers
 * 4. Batch optimization for GPU transfers
 * 5. Graceful fallback to CPU when GPU is unavailable
 *
 * Usage:
 * @code
 * HybridPhysicsConfig config;
 * config.gpu_threshold = 200;  // Use GPU when > 200 entities
 *
 * HybridPhysicsSystem hybrid;
 * hybrid.initialize(config, entity_manager);
 *
 * // Each frame:
 * hybrid.step(dt);  // Automatically routes to CPU or GPU
 * @endcode
 */

#pragma once

#include "jaguar/gpu/compute_backend.h"
#include "jaguar/gpu/physics_kernels.h"
#include "jaguar/gpu/memory_pool.h"
#include "jaguar/physics/entity.h"
#include "jaguar/core/types.h"
#include <memory>
#include <vector>
#include <chrono>
#include <functional>
#include <atomic>
#include <mutex>

namespace jaguar::gpu {

// Forward declarations
class HybridPhysicsSystem;

// ============================================================================
// Configuration
// ============================================================================

/**
 * @brief Workload routing strategy
 */
enum class RoutingStrategy : UInt8 {
    ThresholdBased,     ///< Route based on entity count threshold
    PerformanceBased,   ///< Route based on measured performance
    ForceCPU,           ///< Always use CPU
    ForceGPU,           ///< Always use GPU (with CPU fallback)
    Adaptive            ///< Dynamic adjustment based on metrics
};

/**
 * @brief GPU transfer mode
 */
enum class TransferMode : UInt8 {
    Synchronous,        ///< Blocking transfers (simple, safe)
    Asynchronous,       ///< Async transfers with events (faster)
    PipelinedDouble,    ///< Double-buffered pipelining (lowest latency)
    PipelinedTriple     ///< Triple-buffered pipelining (best throughput)
};

/**
 * @brief Workload type classification
 */
enum class WorkloadType : UInt8 {
    Integration,        ///< Position/velocity integration
    CollisionBroad,     ///< Broad-phase collision detection
    CollisionNarrow,    ///< Narrow-phase collision detection
    Aerodynamics,       ///< Aerodynamic force computation
    TerrainSampling,    ///< Terrain height/normal queries
    WaveSimulation,     ///< Ocean wave spectrum computation
    ForceAccumulation,  ///< Force and torque accumulation
    All                 ///< All workload types
};

/**
 * @brief Per-workload threshold configuration
 */
struct WorkloadThreshold {
    WorkloadType type{WorkloadType::All};
    SizeT cpu_max{100};              ///< Max entities for CPU (exclusive)
    SizeT gpu_min{50};               ///< Min entities for GPU to be efficient
    SizeT gpu_preferred{200};        ///< Preferred GPU threshold
    Real cpu_time_budget_ms{1.0};    ///< Max CPU time before switching
    Real gpu_overhead_ms{0.5};       ///< GPU dispatch overhead estimate
};

/**
 * @brief Configuration for hybrid physics system
 */
struct HybridPhysicsConfig {
    // Routing configuration
    RoutingStrategy strategy{RoutingStrategy::Adaptive};
    SizeT gpu_threshold{200};        ///< Default threshold for GPU usage
    SizeT gpu_min_batch{50};         ///< Minimum batch size for GPU

    // Transfer configuration
    TransferMode transfer_mode{TransferMode::Asynchronous};
    SizeT staging_buffer_size{64 * 1024 * 1024};  ///< 64MB staging
    bool use_memory_pool{true};      ///< Use MemoryPoolManager

    // Dynamic adjustment
    bool enable_dynamic_adjustment{true};
    Real adjustment_interval_sec{1.0};
    Real adjustment_smoothing{0.9};  ///< EMA smoothing factor

    // Performance targets
    Real target_frame_time_ms{16.67}; ///< 60 FPS target
    Real cpu_utilization_target{0.7}; ///< Target CPU utilization
    Real gpu_utilization_target{0.8}; ///< Target GPU utilization

    // Workload-specific thresholds
    std::vector<WorkloadThreshold> workload_thresholds;

    // Debug/profiling
    bool enable_profiling{false};
    bool log_routing_decisions{false};

    /**
     * @brief Create default configuration for physics-heavy simulations
     */
    static HybridPhysicsConfig physics_optimized() {
        HybridPhysicsConfig config;
        config.strategy = RoutingStrategy::Adaptive;
        config.gpu_threshold = 100;
        config.transfer_mode = TransferMode::Asynchronous;
        config.enable_dynamic_adjustment = true;

        // Add workload-specific thresholds
        config.workload_thresholds = {
            {WorkloadType::Integration, 150, 50, 200, 0.5, 0.3},
            {WorkloadType::CollisionBroad, 200, 100, 500, 1.0, 0.5},
            {WorkloadType::Aerodynamics, 50, 20, 100, 0.5, 0.2},
            {WorkloadType::TerrainSampling, 100, 50, 200, 0.5, 0.3}
        };
        return config;
    }

    /**
     * @brief Create configuration for low-latency requirements
     */
    static HybridPhysicsConfig low_latency() {
        HybridPhysicsConfig config;
        config.strategy = RoutingStrategy::ThresholdBased;
        config.gpu_threshold = 500;  // Higher threshold to avoid GPU overhead
        config.transfer_mode = TransferMode::Synchronous;
        config.enable_dynamic_adjustment = false;
        config.target_frame_time_ms = 8.33;  // 120 FPS
        return config;
    }

    /**
     * @brief Create configuration for maximum throughput
     */
    static HybridPhysicsConfig high_throughput() {
        HybridPhysicsConfig config;
        config.strategy = RoutingStrategy::ForceGPU;
        config.gpu_threshold = 50;
        config.transfer_mode = TransferMode::PipelinedTriple;
        config.enable_dynamic_adjustment = true;
        config.staging_buffer_size = 128 * 1024 * 1024;  // 128MB
        return config;
    }
};

// ============================================================================
// Statistics
// ============================================================================

/**
 * @brief Performance statistics for a single workload type
 */
struct WorkloadStats {
    WorkloadType type{WorkloadType::All};

    // Counts
    UInt64 cpu_executions{0};
    UInt64 gpu_executions{0};
    UInt64 total_entities_processed{0};

    // Timing (microseconds)
    UInt64 cpu_time_total_us{0};
    UInt64 gpu_time_total_us{0};
    UInt64 transfer_time_total_us{0};

    // Running averages
    Real avg_cpu_time_us{0.0};
    Real avg_gpu_time_us{0.0};
    Real avg_transfer_time_us{0.0};

    // Computed metrics
    Real cpu_throughput_per_us() const {
        return cpu_time_total_us > 0 ?
            static_cast<Real>(total_entities_processed) / cpu_time_total_us : 0.0;
    }

    Real gpu_throughput_per_us() const {
        return gpu_time_total_us > 0 ?
            static_cast<Real>(total_entities_processed) / gpu_time_total_us : 0.0;
    }

    Real gpu_cpu_ratio() const {
        return avg_cpu_time_us > 0 ? avg_gpu_time_us / avg_cpu_time_us : 1.0;
    }
};

/**
 * @brief Overall hybrid system statistics
 */
struct HybridPhysicsStats {
    // Per-workload statistics
    std::vector<WorkloadStats> workload_stats;

    // Frame statistics
    UInt64 total_frames{0};
    UInt64 cpu_only_frames{0};
    UInt64 gpu_only_frames{0};
    UInt64 hybrid_frames{0};

    // Timing
    Real last_frame_time_ms{0.0};
    Real avg_frame_time_ms{0.0};
    Real max_frame_time_ms{0.0};

    // Transfer statistics
    UInt64 total_uploads{0};
    UInt64 total_downloads{0};
    SizeT total_bytes_uploaded{0};
    SizeT total_bytes_downloaded{0};

    // Threshold adjustments
    UInt64 threshold_adjustments{0};
    SizeT current_threshold{0};

    // Memory
    SizeT gpu_memory_used{0};
    SizeT gpu_memory_peak{0};

    /**
     * @brief Get statistics for a specific workload type
     */
    const WorkloadStats* get_workload_stats(WorkloadType type) const {
        for (const auto& ws : workload_stats) {
            if (ws.type == type) return &ws;
        }
        return nullptr;
    }

    /**
     * @brief Calculate GPU efficiency (useful work / total time)
     */
    Real gpu_efficiency() const {
        Real total_gpu_time = 0.0;
        Real total_transfer_time = 0.0;
        for (const auto& ws : workload_stats) {
            total_gpu_time += ws.gpu_time_total_us;
            total_transfer_time += ws.transfer_time_total_us;
        }
        Real total = total_gpu_time + total_transfer_time;
        return total > 0 ? total_gpu_time / total : 0.0;
    }
};

// ============================================================================
// Routing Decision
// ============================================================================

/**
 * @brief Result of workload routing decision
 */
struct RoutingDecision {
    bool use_gpu{false};
    WorkloadType workload{WorkloadType::All};
    SizeT entity_count{0};
    SizeT batch_size{0};
    Real estimated_cpu_time_ms{0.0};
    Real estimated_gpu_time_ms{0.0};
    std::string reason;
};

// ============================================================================
// GPU Buffer Set
// ============================================================================

/**
 * @brief Persistent GPU buffers for entity state
 */
struct HybridGPUBuffers {
    // Core state buffers
    BufferHandle positions;
    BufferHandle velocities;
    BufferHandle orientations;
    BufferHandle angular_velocities;
    BufferHandle accelerations;
    BufferHandle angular_accelerations;
    BufferHandle masses;
    BufferHandle inv_masses;
    BufferHandle forces;
    BufferHandle torques;

    // Collision buffers
    BufferHandle aabbs;
    BufferHandle collision_pairs;
    BufferHandle extents;

    // Staging buffers (for async transfers)
    BufferHandle staging_upload;
    BufferHandle staging_download;

    // Current capacity
    SizeT capacity{0};
    SizeT entity_count{0};

    bool is_valid() const {
        return positions.id != 0 && velocities.id != 0;
    }
};

// ============================================================================
// CPU Physics Interface
// ============================================================================

/**
 * @brief Interface for CPU physics operations
 *
 * The hybrid system delegates to this interface when using CPU path.
 * Default implementation uses EntityStateStorage batch operations.
 */
class ICPUPhysicsProcessor {
public:
    virtual ~ICPUPhysicsProcessor() = default;

    virtual void integrate_positions(
        physics::EntityStateStorage& storage,
        const std::vector<UInt32>& indices,
        Real dt) = 0;

    virtual void integrate_velocities(
        physics::EntityStateStorage& storage,
        const std::vector<UInt32>& indices,
        Real dt) = 0;

    virtual void compute_accelerations(
        physics::EntityStateStorage& storage,
        const std::vector<UInt32>& indices) = 0;

    virtual void clear_forces(
        physics::EntityStateStorage& storage,
        const std::vector<UInt32>& indices) = 0;

    virtual SizeT broad_phase_collision(
        const physics::EntityStateStorage& storage,
        const std::vector<UInt32>& indices,
        std::vector<CollisionPair>& pairs) = 0;
};

// ============================================================================
// Hybrid Physics System
// ============================================================================

/**
 * @brief Main hybrid CPU/GPU physics system
 *
 * HybridPhysicsSystem provides automatic workload distribution between
 * CPU and GPU based on entity count, performance metrics, and configuration.
 */
class HybridPhysicsSystem {
public:
    /**
     * @brief Construct hybrid system
     */
    HybridPhysicsSystem();

    /**
     * @brief Destructor - releases all resources
     */
    ~HybridPhysicsSystem();

    // Non-copyable
    HybridPhysicsSystem(const HybridPhysicsSystem&) = delete;
    HybridPhysicsSystem& operator=(const HybridPhysicsSystem&) = delete;

    // ========================================================================
    // Initialization
    // ========================================================================

    /**
     * @brief Initialize hybrid system with configuration
     *
     * @param config System configuration
     * @param entity_storage Pointer to entity state storage (not owned)
     * @param cpu_processor Optional CPU processor (default provided if null)
     * @return Success or error code
     */
    BackendResult initialize(
        const HybridPhysicsConfig& config,
        physics::EntityStateStorage* entity_storage,
        std::unique_ptr<ICPUPhysicsProcessor> cpu_processor = nullptr);

    /**
     * @brief Initialize with specific backend
     */
    BackendResult initialize(
        const HybridPhysicsConfig& config,
        physics::EntityStateStorage* entity_storage,
        std::unique_ptr<IComputeBackend> backend,
        std::unique_ptr<ICPUPhysicsProcessor> cpu_processor = nullptr);

    /**
     * @brief Shutdown and release all resources
     */
    void shutdown();

    /**
     * @brief Check if system is initialized
     */
    bool is_initialized() const { return m_initialized; }

    /**
     * @brief Check if GPU is available and active
     */
    bool is_gpu_available() const;

    // ========================================================================
    // Physics Step
    // ========================================================================

    /**
     * @brief Execute a complete physics step
     *
     * This method automatically routes workloads between CPU and GPU
     * based on entity count and configuration.
     *
     * @param dt Time step in seconds
     * @return Success or error code
     */
    BackendResult step(Real dt);

    /**
     * @brief Execute specific workload type
     */
    BackendResult execute_workload(WorkloadType type, Real dt);

    /**
     * @brief Force a specific execution path for the next step
     */
    void force_next_path(bool use_gpu);

    // ========================================================================
    // Entity Management
    // ========================================================================

    /**
     * @brief Mark entities as dirty (need GPU upload)
     */
    void mark_entities_dirty(const std::vector<UInt32>& indices);

    /**
     * @brief Mark all entities as dirty
     */
    void mark_all_dirty();

    /**
     * @brief Set active entity indices for processing
     *
     * Only these entities will be processed in step().
     */
    void set_active_entities(const std::vector<UInt32>& indices);

    /**
     * @brief Synchronize entity state from GPU to CPU
     *
     * Call after GPU execution if you need to read state on CPU.
     */
    BackendResult sync_to_cpu();

    /**
     * @brief Upload entity state from CPU to GPU
     *
     * Typically called automatically, but can be forced.
     */
    BackendResult sync_to_gpu();

    // ========================================================================
    // Configuration
    // ========================================================================

    /**
     * @brief Update configuration
     */
    void set_config(const HybridPhysicsConfig& config);

    /**
     * @brief Get current configuration
     */
    const HybridPhysicsConfig& config() const { return m_config; }

    /**
     * @brief Set threshold for a specific workload
     */
    void set_workload_threshold(WorkloadType type, SizeT threshold);

    /**
     * @brief Get current threshold for a workload type
     */
    SizeT get_workload_threshold(WorkloadType type) const;

    /**
     * @brief Set routing strategy
     */
    void set_routing_strategy(RoutingStrategy strategy);

    // ========================================================================
    // Statistics & Debugging
    // ========================================================================

    /**
     * @brief Get performance statistics
     */
    const HybridPhysicsStats& stats() const { return m_stats; }

    /**
     * @brief Reset statistics
     */
    void reset_stats();

    /**
     * @brief Get last routing decision
     */
    const RoutingDecision& last_decision() const { return m_last_decision; }

    /**
     * @brief Set callback for routing decisions (for debugging)
     */
    void set_routing_callback(
        std::function<void(const RoutingDecision&)> callback);

    /**
     * @brief Get the compute backend (may be null if GPU unavailable)
     */
    IComputeBackend* backend() const { return m_backend.get(); }

    /**
     * @brief Get the physics kernel manager
     */
    PhysicsKernelManager* kernel_manager() const { return m_kernel_manager.get(); }

    /**
     * @brief Get the memory pool manager
     */
    MemoryPoolManager* memory_pool() const { return m_memory_pool.get(); }

private:
    // ========================================================================
    // Internal Methods
    // ========================================================================

    // Routing
    RoutingDecision make_routing_decision(WorkloadType type, SizeT entity_count);
    void update_dynamic_thresholds();
    Real estimate_cpu_time(WorkloadType type, SizeT count) const;
    Real estimate_gpu_time(WorkloadType type, SizeT count) const;

    // Buffer management
    BackendResult ensure_gpu_buffers(SizeT capacity);
    BackendResult resize_gpu_buffers(SizeT new_capacity);
    void release_gpu_buffers();

    // Transfer operations
    BackendResult upload_entities(const std::vector<UInt32>& indices);
    BackendResult download_entities(const std::vector<UInt32>& indices);
    BackendResult batch_upload();
    BackendResult batch_download();

    // CPU execution
    BackendResult execute_cpu_integration(Real dt);
    BackendResult execute_cpu_collision();

    // GPU execution
    BackendResult execute_gpu_integration(Real dt);
    BackendResult execute_gpu_collision();

    // Statistics
    void record_cpu_execution(WorkloadType type, SizeT count, UInt64 time_us);
    void record_gpu_execution(WorkloadType type, SizeT count, UInt64 time_us);
    void record_transfer(SizeT bytes, bool upload, UInt64 time_us);

    // ========================================================================
    // Member Variables
    // ========================================================================

    // Configuration
    HybridPhysicsConfig m_config;
    bool m_initialized{false};

    // External references (not owned)
    physics::EntityStateStorage* m_entity_storage{nullptr};

    // GPU resources
    std::unique_ptr<IComputeBackend> m_backend;
    std::unique_ptr<PhysicsKernelManager> m_kernel_manager;
    std::unique_ptr<MemoryPoolManager> m_memory_pool;
    HybridGPUBuffers m_gpu_buffers;
    StreamHandle m_upload_stream;
    StreamHandle m_download_stream;
    StreamHandle m_compute_stream;

    // CPU processor
    std::unique_ptr<ICPUPhysicsProcessor> m_cpu_processor;

    // Entity tracking
    std::vector<UInt32> m_active_indices;
    std::vector<bool> m_dirty_flags;
    bool m_all_dirty{true};
    SizeT m_dirty_count{0};

    // Pipelining state
    std::vector<Real> m_staging_positions;
    std::vector<Real> m_staging_velocities;
    UInt32 m_current_buffer{0};

    // Statistics
    HybridPhysicsStats m_stats;
    RoutingDecision m_last_decision;
    std::function<void(const RoutingDecision&)> m_routing_callback;

    // Timing
    std::chrono::steady_clock::time_point m_last_adjustment_time;
    std::chrono::steady_clock::time_point m_frame_start_time;

    // Thread safety
    mutable std::mutex m_mutex;
    std::atomic<bool> m_force_gpu{false};
    std::atomic<bool> m_force_cpu{false};

    // Per-workload thresholds (computed from config)
    std::vector<SizeT> m_workload_thresholds;
    std::vector<Real> m_cpu_time_estimates;
    std::vector<Real> m_gpu_time_estimates;
};

// ============================================================================
// Default CPU Processor
// ============================================================================

/**
 * @brief Default CPU physics processor using EntityStateStorage
 */
class DefaultCPUPhysicsProcessor : public ICPUPhysicsProcessor {
public:
    void integrate_positions(
        physics::EntityStateStorage& storage,
        const std::vector<UInt32>& indices,
        Real dt) override;

    void integrate_velocities(
        physics::EntityStateStorage& storage,
        const std::vector<UInt32>& indices,
        Real dt) override;

    void compute_accelerations(
        physics::EntityStateStorage& storage,
        const std::vector<UInt32>& indices) override;

    void clear_forces(
        physics::EntityStateStorage& storage,
        const std::vector<UInt32>& indices) override;

    SizeT broad_phase_collision(
        const physics::EntityStateStorage& storage,
        const std::vector<UInt32>& indices,
        std::vector<CollisionPair>& pairs) override;
};

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Get string representation of routing strategy
 */
inline const char* to_string(RoutingStrategy strategy) {
    switch (strategy) {
        case RoutingStrategy::ThresholdBased: return "ThresholdBased";
        case RoutingStrategy::PerformanceBased: return "PerformanceBased";
        case RoutingStrategy::ForceCPU: return "ForceCPU";
        case RoutingStrategy::ForceGPU: return "ForceGPU";
        case RoutingStrategy::Adaptive: return "Adaptive";
        default: return "Unknown";
    }
}

/**
 * @brief Get string representation of transfer mode
 */
inline const char* to_string(TransferMode mode) {
    switch (mode) {
        case TransferMode::Synchronous: return "Synchronous";
        case TransferMode::Asynchronous: return "Asynchronous";
        case TransferMode::PipelinedDouble: return "PipelinedDouble";
        case TransferMode::PipelinedTriple: return "PipelinedTriple";
        default: return "Unknown";
    }
}

/**
 * @brief Get string representation of workload type
 */
inline const char* to_string(WorkloadType type) {
    switch (type) {
        case WorkloadType::Integration: return "Integration";
        case WorkloadType::CollisionBroad: return "CollisionBroad";
        case WorkloadType::CollisionNarrow: return "CollisionNarrow";
        case WorkloadType::Aerodynamics: return "Aerodynamics";
        case WorkloadType::TerrainSampling: return "TerrainSampling";
        case WorkloadType::WaveSimulation: return "WaveSimulation";
        case WorkloadType::ForceAccumulation: return "ForceAccumulation";
        case WorkloadType::All: return "All";
        default: return "Unknown";
    }
}

} // namespace jaguar::gpu
