#pragma once
/**
 * @file simd.h
 * @brief SIMD operations for batch entity updates
 *
 * Provides AVX2-optimized batch operations for physics simulation.
 * Falls back to scalar operations when SIMD is not available.
 */

#include "jaguar/core/types.h"
#include <cstddef>

// Detect SIMD availability
#if defined(__AVX2__) && defined(__FMA__)
    #define JAGUAR_HAS_AVX2 1
    #include <immintrin.h>
#else
    #define JAGUAR_HAS_AVX2 0
#endif

namespace jaguar::simd {

// ============================================================================
// SIMD Configuration
// ============================================================================

/// Number of doubles per AVX register
constexpr SizeT AVX_DOUBLES = 4;

/// Alignment requirement for SIMD operations (64 bytes = cache line)
constexpr SizeT SIMD_ALIGNMENT = 64;

// ============================================================================
// Batch Position-Velocity Integration
// ============================================================================

/**
 * @brief Batch update positions using Euler step: p = p + v * dt
 *
 * Processes multiple entities in parallel using SIMD instructions.
 * Arrays must be aligned to SIMD_ALIGNMENT bytes.
 *
 * @param positions Array of position components (x,y,z interleaved or separate)
 * @param velocities Array of velocity components
 * @param count Number of elements to process
 * @param dt Time step in seconds
 */
void batch_position_update(
    Real* __restrict positions_x,
    Real* __restrict positions_y,
    Real* __restrict positions_z,
    const Real* __restrict velocities_x,
    const Real* __restrict velocities_y,
    const Real* __restrict velocities_z,
    SizeT count,
    Real dt);

/**
 * @brief Batch update velocities using acceleration: v = v + a * dt
 *
 * @param velocities Array of velocity components
 * @param accelerations Array of acceleration components
 * @param count Number of elements to process
 * @param dt Time step in seconds
 */
void batch_velocity_update(
    Real* __restrict velocities_x,
    Real* __restrict velocities_y,
    Real* __restrict velocities_z,
    const Real* __restrict accelerations_x,
    const Real* __restrict accelerations_y,
    const Real* __restrict accelerations_z,
    SizeT count,
    Real dt);

// ============================================================================
// Batch Force Accumulation
// ============================================================================

/**
 * @brief Batch compute accelerations from forces: a = F / m
 *
 * @param accelerations_xyz Output acceleration arrays
 * @param forces_xyz Input force arrays
 * @param masses Array of masses
 * @param count Number of entities
 */
void batch_force_to_acceleration(
    Real* __restrict accel_x,
    Real* __restrict accel_y,
    Real* __restrict accel_z,
    const Real* __restrict force_x,
    const Real* __restrict force_y,
    const Real* __restrict force_z,
    const Real* __restrict masses,
    SizeT count);

/**
 * @brief Batch accumulate gravity forces: F += m * g
 *
 * @param forces_z Z-component of forces (gravity acts in -Z for NED)
 * @param masses Array of masses
 * @param g Gravitational acceleration (positive value)
 * @param count Number of entities
 */
void batch_add_gravity(
    Real* __restrict forces_z,
    const Real* __restrict masses,
    Real g,
    SizeT count);

// ============================================================================
// Batch Vector Operations
// ============================================================================

/**
 * @brief Batch scale vectors: out = in * scalar
 */
void batch_scale(
    Real* __restrict out,
    const Real* __restrict in,
    Real scalar,
    SizeT count);

/**
 * @brief Batch add vectors: out = a + b
 */
void batch_add(
    Real* __restrict out,
    const Real* __restrict a,
    const Real* __restrict b,
    SizeT count);

/**
 * @brief Batch fused multiply-add: out = a + b * scalar
 */
void batch_fma(
    Real* __restrict out,
    const Real* __restrict a,
    const Real* __restrict b,
    Real scalar,
    SizeT count);

/**
 * @brief Batch dot product for arrays of Vec3
 *
 * Computes dot products: out[i] = a[i].dot(b[i])
 */
void batch_dot_product(
    Real* __restrict out,
    const Real* __restrict ax, const Real* __restrict ay, const Real* __restrict az,
    const Real* __restrict bx, const Real* __restrict by, const Real* __restrict bz,
    SizeT count);

/**
 * @brief Batch vector magnitude squared: out[i] = v[i].length_squared()
 */
void batch_length_squared(
    Real* __restrict out,
    const Real* __restrict vx,
    const Real* __restrict vy,
    const Real* __restrict vz,
    SizeT count);

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Check if a pointer is properly aligned for SIMD operations
 */
inline bool is_simd_aligned(const void* ptr) {
    return (reinterpret_cast<std::uintptr_t>(ptr) % SIMD_ALIGNMENT) == 0;
}

/**
 * @brief Check if SIMD is available at runtime
 */
bool simd_available();

/**
 * @brief Get the SIMD instruction set name
 */
const char* simd_instruction_set();

} // namespace jaguar::simd
