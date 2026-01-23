/**
 * @file physics_kernels.h
 * @brief GPU-accelerated physics computation kernels
 *
 * This file defines the interface for GPU physics kernels that can run
 * on any compute backend (CPU, CUDA, OpenCL, Metal). The kernels are
 * designed for high-throughput batch processing of physics entities.
 *
 * Kernel Categories:
 * - Collision: Broad-phase and narrow-phase collision detection
 * - Integration: Rigid body state integration
 * - Forces: Aerodynamic, hydrodynamic, gravitational force computation
 * - Terrain: Batch terrain height/normal sampling
 * - Constraints: Parallel constraint solving
 *
 * Design Principles:
 * 1. Backend-agnostic kernel interface
 * 2. Structure-of-Arrays (SoA) data layout for GPU efficiency
 * 3. Batch processing for maximum throughput
 * 4. CPU fallback for all kernels
 *
 * Usage:
 * @code
 * auto backend = BackendFactory::create_best_available();
 * PhysicsKernelManager kernels(backend.get());
 *
 * // Allocate SoA buffers
 * auto positions = kernels.allocate_vec3_buffer(entity_count);
 * auto velocities = kernels.allocate_vec3_buffer(entity_count);
 *
 * // Run integration kernel
 * kernels.integrate_positions(positions, velocities, dt, entity_count);
 * @endcode
 */

#pragma once

#include "jaguar/gpu/compute_backend.h"
#include "jaguar/core/types.h"
#include <memory>
#include <vector>
#include <string>

namespace jaguar::gpu {

// ============================================================================
// Forward Declarations
// ============================================================================

class PhysicsKernelManager;
struct CollisionPair;
struct AABB;

// ============================================================================
// Data Structures for GPU Computation
// ============================================================================

/**
 * @brief Axis-Aligned Bounding Box for collision detection
 */
struct alignas(32) AABB {
    float min_x, min_y, min_z;
    float max_x, max_y, max_z;
    UInt32 entity_id;
    UInt32 padding;  // Align to 32 bytes

    static AABB from_center_extents(float cx, float cy, float cz,
                                     float ex, float ey, float ez,
                                     UInt32 id) {
        return AABB{
            cx - ex, cy - ey, cz - ez,
            cx + ex, cy + ey, cz + ez,
            id, 0
        };
    }

    bool intersects(const AABB& other) const {
        return min_x <= other.max_x && max_x >= other.min_x &&
               min_y <= other.max_y && max_y >= other.min_y &&
               min_z <= other.max_z && max_z >= other.min_z;
    }
};

/**
 * @brief Collision pair output from broad-phase
 */
struct alignas(8) CollisionPair {
    UInt32 entity_a;
    UInt32 entity_b;

    bool operator==(const CollisionPair& other) const {
        return (entity_a == other.entity_a && entity_b == other.entity_b) ||
               (entity_a == other.entity_b && entity_b == other.entity_a);
    }
};

/**
 * @brief Rigid body state for integration (SoA layout)
 */
struct RigidBodyStateBuffers {
    BufferHandle positions;      // float3 * count
    BufferHandle velocities;     // float3 * count
    BufferHandle orientations;   // float4 (quaternion) * count
    BufferHandle angular_vels;   // float3 * count
    BufferHandle forces;         // float3 * count
    BufferHandle torques;        // float3 * count
    BufferHandle masses;         // float * count
    BufferHandle inv_inertia;    // float3x3 * count (diagonal only for now)
    SizeT count{0};
};

/**
 * @brief Terrain sample request
 */
struct alignas(16) TerrainSampleRequest {
    float x, y, z;      // World position to sample
    UInt32 entity_id;   // Entity making the request
};

/**
 * @brief Terrain sample result
 */
struct alignas(32) TerrainSampleResult {
    float height;           // Terrain height at position
    float normal_x, normal_y, normal_z;  // Surface normal
    float friction;         // Surface friction coefficient
    UInt32 material_id;     // Terrain material type
    UInt32 entity_id;       // Requesting entity
    UInt32 padding;
};

/**
 * @brief Aerodynamic computation input
 */
struct alignas(64) AeroInput {
    // Aircraft state
    float pos_x, pos_y, pos_z;
    float vel_x, vel_y, vel_z;
    float quat_w, quat_x, quat_y, quat_z;
    float ang_vel_x, ang_vel_y, ang_vel_z;

    // Atmospheric conditions
    float air_density;
    float speed_of_sound;
    float wind_x, wind_y, wind_z;

    // Control surfaces
    float aileron;      // -1 to 1
    float elevator;     // -1 to 1
    float rudder;       // -1 to 1
    float throttle;     // 0 to 1
    float flaps;        // 0 to 1

    UInt32 entity_id;
    UInt32 aero_model_id;
    float padding[2];
};

/**
 * @brief Aerodynamic computation output
 */
struct alignas(64) AeroOutput {
    // Forces in body frame
    float force_x, force_y, force_z;
    // Moments in body frame
    float moment_x, moment_y, moment_z;

    // Computed values for telemetry
    float alpha;            // Angle of attack
    float beta;             // Sideslip angle
    float mach;             // Mach number
    float dynamic_pressure; // q = 0.5 * rho * V^2
    float lift_coeff;       // CL
    float drag_coeff;       // CD

    UInt32 entity_id;
    UInt32 padding[3];
};

/**
 * @brief Spatial hash cell for broad-phase collision
 */
struct SpatialHashConfig {
    float cell_size{10.0f};         // Size of each hash cell
    float world_min_x{-10000.0f};   // World bounds
    float world_min_y{-10000.0f};
    float world_min_z{-10000.0f};
    float world_max_x{10000.0f};
    float world_max_y{10000.0f};
    float world_max_z{10000.0f};
    UInt32 max_entities_per_cell{64};
};

// ============================================================================
// Kernel Configuration
// ============================================================================

/**
 * @brief Configuration for physics kernel execution
 */
struct PhysicsKernelConfig {
    // Collision detection
    SpatialHashConfig spatial_hash;
    UInt32 max_collision_pairs{100000};

    // Integration
    bool use_symplectic{true};      // Use symplectic integration
    Real gravity_x{0.0};
    Real gravity_y{0.0};
    Real gravity_z{-9.81};

    // Terrain
    UInt32 max_terrain_samples{10000};

    // Aerodynamics
    UInt32 max_aero_entities{1000};

    // Performance tuning
    SizeT preferred_workgroup_size{256};
    bool enable_profiling{false};
};

// ============================================================================
// Physics Kernel Manager
// ============================================================================

/**
 * @brief Manager for GPU physics kernels
 *
 * PhysicsKernelManager provides high-level interface for running
 * physics computations on GPU or CPU backends.
 */
class PhysicsKernelManager {
public:
    /**
     * @brief Construct kernel manager with backend
     * @param backend Compute backend to use (takes ownership)
     */
    explicit PhysicsKernelManager(IComputeBackend* backend);

    ~PhysicsKernelManager();

    // Non-copyable
    PhysicsKernelManager(const PhysicsKernelManager&) = delete;
    PhysicsKernelManager& operator=(const PhysicsKernelManager&) = delete;

    /**
     * @brief Initialize kernel manager with configuration
     */
    BackendResult initialize(const PhysicsKernelConfig& config = {});

    /**
     * @brief Shutdown and release resources
     */
    void shutdown();

    /**
     * @brief Check if manager is initialized
     */
    bool is_initialized() const { return m_initialized; }

    /**
     * @brief Get the compute backend
     */
    IComputeBackend* backend() const { return m_backend; }

    // ========================================================================
    // Buffer Allocation
    // ========================================================================

    /**
     * @brief Allocate buffer for float3 vectors
     */
    BufferHandle allocate_vec3_buffer(SizeT count);

    /**
     * @brief Allocate buffer for float4 (quaternions)
     */
    BufferHandle allocate_vec4_buffer(SizeT count);

    /**
     * @brief Allocate buffer for scalars
     */
    BufferHandle allocate_scalar_buffer(SizeT count);

    /**
     * @brief Allocate AABB buffer for collision detection
     */
    BufferHandle allocate_aabb_buffer(SizeT count);

    /**
     * @brief Allocate collision pair buffer
     */
    BufferHandle allocate_collision_pair_buffer(SizeT max_pairs);

    /**
     * @brief Free a buffer
     */
    void free_buffer(BufferHandle buffer);

    // ========================================================================
    // Collision Detection Kernels
    // ========================================================================

    /**
     * @brief Update AABBs from positions and extents
     *
     * @param positions Input position buffer (float3 * count)
     * @param extents Input half-extents buffer (float3 * count)
     * @param aabbs Output AABB buffer
     * @param count Number of entities
     */
    BackendResult update_aabbs(
        BufferHandle positions,
        BufferHandle extents,
        BufferHandle aabbs,
        SizeT count);

    /**
     * @brief Broad-phase collision detection using spatial hashing
     *
     * @param aabbs Input AABB buffer
     * @param count Number of AABBs
     * @param pairs Output collision pairs
     * @param pair_count Output: number of pairs found
     * @return Success or error code
     */
    BackendResult collision_broad_phase(
        BufferHandle aabbs,
        SizeT count,
        BufferHandle pairs,
        SizeT* pair_count);

    /**
     * @brief Sort AABBs along an axis for sweep-and-prune
     *
     * @param aabbs AABB buffer to sort
     * @param count Number of AABBs
     * @param axis Axis to sort along (0=X, 1=Y, 2=Z)
     */
    BackendResult sort_aabbs(
        BufferHandle aabbs,
        SizeT count,
        UInt32 axis = 0);

    // ========================================================================
    // Rigid Body Integration Kernels
    // ========================================================================

    /**
     * @brief Integrate positions using velocities
     *
     * positions += velocities * dt
     *
     * @param positions Position buffer (float3 * count)
     * @param velocities Velocity buffer (float3 * count)
     * @param dt Time step
     * @param count Number of entities
     */
    BackendResult integrate_positions(
        BufferHandle positions,
        BufferHandle velocities,
        Real dt,
        SizeT count);

    /**
     * @brief Integrate velocities using forces
     *
     * velocities += (forces / masses + gravity) * dt
     *
     * @param velocities Velocity buffer (float3 * count)
     * @param forces Force buffer (float3 * count)
     * @param masses Mass buffer (float * count)
     * @param dt Time step
     * @param count Number of entities
     */
    BackendResult integrate_velocities(
        BufferHandle velocities,
        BufferHandle forces,
        BufferHandle masses,
        Real dt,
        SizeT count);

    /**
     * @brief Integrate orientations using angular velocities
     *
     * Quaternion integration: q += 0.5 * omega * q * dt
     *
     * @param orientations Quaternion buffer (float4 * count)
     * @param angular_velocities Angular velocity buffer (float3 * count)
     * @param dt Time step
     * @param count Number of entities
     */
    BackendResult integrate_orientations(
        BufferHandle orientations,
        BufferHandle angular_velocities,
        Real dt,
        SizeT count);

    /**
     * @brief Full symplectic Euler integration step
     *
     * Combines velocity and position integration in correct order.
     */
    BackendResult integrate_symplectic(
        RigidBodyStateBuffers& state,
        Real dt);

    /**
     * @brief Clear force and torque accumulators
     */
    BackendResult clear_forces(
        BufferHandle forces,
        BufferHandle torques,
        SizeT count);

    // ========================================================================
    // Terrain Sampling Kernels
    // ========================================================================

    /**
     * @brief Set terrain heightmap for GPU sampling
     *
     * @param heightmap 2D heightmap buffer (float * width * height)
     * @param width Heightmap width
     * @param height Heightmap height
     * @param origin World origin of heightmap (x, y)
     * @param scale World scale per texel
     */
    BackendResult set_terrain_heightmap(
        BufferHandle heightmap,
        UInt32 width,
        UInt32 height,
        Real origin_x,
        Real origin_y,
        Real scale);

    /**
     * @brief Batch sample terrain heights
     *
     * @param requests Sample request buffer
     * @param results Output result buffer
     * @param count Number of samples
     */
    BackendResult sample_terrain_batch(
        BufferHandle requests,
        BufferHandle results,
        SizeT count);

    // ========================================================================
    // Aerodynamic Force Kernels
    // ========================================================================

    /**
     * @brief Compute aerodynamic forces for batch of aircraft
     *
     * @param inputs Aero input buffer
     * @param outputs Aero output buffer
     * @param count Number of aircraft
     */
    BackendResult compute_aero_forces(
        BufferHandle inputs,
        BufferHandle outputs,
        SizeT count);

    /**
     * @brief Set aerodynamic coefficient table
     *
     * @param model_id Model identifier
     * @param alpha_table Angle of attack breakpoints
     * @param cl_table Lift coefficient table
     * @param cd_table Drag coefficient table
     * @param cm_table Moment coefficient table
     */
    BackendResult set_aero_model(
        UInt32 model_id,
        const std::vector<Real>& alpha_table,
        const std::vector<Real>& cl_table,
        const std::vector<Real>& cd_table,
        const std::vector<Real>& cm_table);

    // ========================================================================
    // Ocean Wave Spectrum Kernels
    // ========================================================================

    /**
     * @brief Compute wave spectrum heights using FFT-based ocean simulation
     *
     * Implements Phillips spectrum or JONSWAP spectrum for realistic ocean waves.
     *
     * @param spectrum_buffer Output spectrum buffer (complex, N x N)
     * @param resolution Grid resolution (power of 2, e.g., 256, 512)
     * @param wind_speed Wind speed in m/s
     * @param wind_direction Wind direction in radians
     * @param time Current simulation time
     */
    BackendResult compute_wave_spectrum(
        BufferHandle spectrum_buffer,
        UInt32 resolution,
        Real wind_speed,
        Real wind_direction,
        Real time);

    /**
     * @brief Sample wave heights at multiple positions
     *
     * @param positions Input positions (float2 * count, x/z world coordinates)
     * @param heights Output heights (float * count)
     * @param normals Output normals (float3 * count, optional)
     * @param spectrum_buffer Precomputed wave spectrum
     * @param resolution Spectrum resolution
     * @param patch_size World size of one spectrum tile
     * @param count Number of sample positions
     */
    BackendResult sample_wave_heights(
        BufferHandle positions,
        BufferHandle heights,
        BufferHandle normals,
        BufferHandle spectrum_buffer,
        UInt32 resolution,
        Real patch_size,
        SizeT count);

    // ========================================================================
    // Atmosphere Density Kernels
    // ========================================================================

    /**
     * @brief Compute atmospheric density at multiple altitudes
     *
     * Uses ISA (International Standard Atmosphere) model or custom atmosphere.
     *
     * @param altitudes Input altitudes (float * count, in meters)
     * @param densities Output air densities (float * count, in kg/m³)
     * @param temperatures Output temperatures (float * count, in Kelvin, optional)
     * @param pressures Output pressures (float * count, in Pa, optional)
     * @param count Number of sample points
     */
    BackendResult compute_atmosphere_density(
        BufferHandle altitudes,
        BufferHandle densities,
        BufferHandle temperatures,
        BufferHandle pressures,
        SizeT count);

    /**
     * @brief Batch compute atmospheric properties for aircraft
     *
     * Optimized version that takes 3D positions and returns all aero-relevant
     * atmospheric properties in a single kernel call.
     *
     * @param positions Input 3D positions (float3 * count)
     * @param air_density Output air density (float * count)
     * @param speed_of_sound Output speed of sound (float * count)
     * @param temperature Output temperature (float * count)
     * @param count Number of aircraft
     */
    BackendResult compute_atmosphere_batch(
        BufferHandle positions,
        BufferHandle air_density,
        BufferHandle speed_of_sound,
        BufferHandle temperature,
        SizeT count);

    // ========================================================================
    // Utility Kernels
    // ========================================================================

    /**
     * @brief Apply gravity to all forces
     *
     * forces += mass * gravity
     */
    BackendResult apply_gravity(
        BufferHandle forces,
        BufferHandle masses,
        SizeT count);

    /**
     * @brief Apply linear damping to velocities
     *
     * velocities *= (1 - damping * dt)
     */
    BackendResult apply_damping(
        BufferHandle velocities,
        Real damping,
        Real dt,
        SizeT count);

    /**
     * @brief Clamp velocities to maximum speed
     */
    BackendResult clamp_velocities(
        BufferHandle velocities,
        Real max_speed,
        SizeT count);

    // ========================================================================
    // Synchronization
    // ========================================================================

    /**
     * @brief Wait for all pending kernel operations to complete
     */
    BackendResult synchronize();

    /**
     * @brief Get last error message
     */
    const std::string& last_error() const { return m_last_error; }

private:
    IComputeBackend* m_backend{nullptr};
    PhysicsKernelConfig m_config;
    bool m_initialized{false};
    std::string m_last_error;

    // Terrain data
    BufferHandle m_terrain_heightmap;
    UInt32 m_terrain_width{0};
    UInt32 m_terrain_height{0};
    Real m_terrain_origin_x{0};
    Real m_terrain_origin_y{0};
    Real m_terrain_scale{1.0};

    // Collision detection buffers
    BufferHandle m_spatial_hash_cells;
    BufferHandle m_spatial_hash_counts;

    // Compiled kernels (if backend supports)
    std::unique_ptr<IKernel> m_integrate_positions_kernel;
    std::unique_ptr<IKernel> m_integrate_velocities_kernel;
    std::unique_ptr<IKernel> m_integrate_orientations_kernel;
    std::unique_ptr<IKernel> m_collision_broad_phase_kernel;
    std::unique_ptr<IKernel> m_aero_forces_kernel;
    std::unique_ptr<IKernel> m_terrain_sample_kernel;

    // Helper methods
    BackendResult compile_kernels();
    void register_cpu_kernel_functions();
};

// ============================================================================
// CPU Kernel Function Signatures
// ============================================================================

namespace cpu_kernels {

/**
 * @brief CPU implementation of position integration
 */
void integrate_positions_cpu(
    float* positions,       // [count * 3]
    const float* velocities, // [count * 3]
    float dt,
    SizeT count);

/**
 * @brief CPU implementation of velocity integration
 */
void integrate_velocities_cpu(
    float* velocities,      // [count * 3]
    const float* forces,    // [count * 3]
    const float* masses,    // [count]
    float gravity_x,
    float gravity_y,
    float gravity_z,
    float dt,
    SizeT count);

/**
 * @brief CPU implementation of quaternion integration
 */
void integrate_orientations_cpu(
    float* orientations,           // [count * 4] (w, x, y, z)
    const float* angular_velocities, // [count * 3]
    float dt,
    SizeT count);

/**
 * @brief CPU implementation of AABB update
 */
void update_aabbs_cpu(
    const float* positions,  // [count * 3]
    const float* extents,    // [count * 3]
    AABB* aabbs,             // [count]
    SizeT count);

/**
 * @brief CPU implementation of broad-phase collision
 */
SizeT collision_broad_phase_cpu(
    const AABB* aabbs,
    SizeT count,
    CollisionPair* pairs,
    SizeT max_pairs,
    const SpatialHashConfig& config);

/**
 * @brief CPU implementation of aerodynamic forces
 */
void compute_aero_forces_cpu(
    const AeroInput* inputs,
    AeroOutput* outputs,
    SizeT count);

/**
 * @brief CPU implementation of terrain sampling
 */
void sample_terrain_cpu(
    const TerrainSampleRequest* requests,
    TerrainSampleResult* results,
    const float* heightmap,
    UInt32 width,
    UInt32 height,
    float origin_x,
    float origin_y,
    float scale,
    SizeT count);

/**
 * @brief CPU implementation of gravity application
 */
void apply_gravity_cpu(
    float* forces,          // [count * 3]
    const float* masses,    // [count]
    float gravity_x,
    float gravity_y,
    float gravity_z,
    SizeT count);

/**
 * @brief CPU implementation of damping
 */
void apply_damping_cpu(
    float* velocities,      // [count * 3]
    float damping,
    float dt,
    SizeT count);

/**
 * @brief CPU implementation of wave spectrum computation
 */
void compute_wave_spectrum_cpu(
    float* spectrum_real,       // [resolution * resolution]
    float* spectrum_imag,       // [resolution * resolution]
    UInt32 resolution,
    float wind_speed,
    float wind_direction,
    float time);

/**
 * @brief CPU implementation of wave height sampling
 */
void sample_wave_heights_cpu(
    const float* positions,     // [count * 2] (x, z)
    float* heights,             // [count]
    float* normals,             // [count * 3] (optional, can be nullptr)
    const float* spectrum_real, // [resolution * resolution]
    const float* spectrum_imag, // [resolution * resolution]
    UInt32 resolution,
    float patch_size,
    SizeT count);

/**
 * @brief CPU implementation of ISA atmosphere model
 */
void compute_atmosphere_density_cpu(
    const float* altitudes,     // [count]
    float* densities,           // [count]
    float* temperatures,        // [count] (optional, can be nullptr)
    float* pressures,           // [count] (optional, can be nullptr)
    SizeT count);

/**
 * @brief CPU implementation of batch atmosphere properties
 */
void compute_atmosphere_batch_cpu(
    const float* positions,     // [count * 3]
    float* air_density,         // [count]
    float* speed_of_sound,      // [count]
    float* temperature,         // [count]
    SizeT count);

} // namespace cpu_kernels

} // namespace jaguar::gpu
