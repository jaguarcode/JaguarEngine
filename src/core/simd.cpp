/**
 * @file simd.cpp
 * @brief SIMD batch operations implementation
 *
 * Provides AVX2-optimized batch operations for physics simulation.
 * Falls back to scalar operations when SIMD is not available.
 */

#include "jaguar/core/simd.h"

namespace jaguar::simd {

// ============================================================================
// Utility Functions
// ============================================================================

bool simd_available() {
#if JAGUAR_HAS_AVX2
    return true;
#else
    return false;
#endif
}

const char* simd_instruction_set() {
#if JAGUAR_HAS_AVX2
    return "AVX2+FMA";
#else
    return "Scalar";
#endif
}

// ============================================================================
// SIMD Implementation (AVX2)
// ============================================================================

#if JAGUAR_HAS_AVX2

void batch_position_update(
    Real* __restrict px, Real* __restrict py, Real* __restrict pz,
    const Real* __restrict vx, const Real* __restrict vy, const Real* __restrict vz,
    SizeT count, Real dt)
{
    const __m256d dt_vec = _mm256_set1_pd(dt);

    // Process 4 entities at a time
    SizeT i = 0;
    for (; i + AVX_DOUBLES <= count; i += AVX_DOUBLES) {
        // Load positions
        __m256d pos_x = _mm256_loadu_pd(&px[i]);
        __m256d pos_y = _mm256_loadu_pd(&py[i]);
        __m256d pos_z = _mm256_loadu_pd(&pz[i]);

        // Load velocities
        __m256d vel_x = _mm256_loadu_pd(&vx[i]);
        __m256d vel_y = _mm256_loadu_pd(&vy[i]);
        __m256d vel_z = _mm256_loadu_pd(&vz[i]);

        // p = p + v * dt (using FMA)
        pos_x = _mm256_fmadd_pd(vel_x, dt_vec, pos_x);
        pos_y = _mm256_fmadd_pd(vel_y, dt_vec, pos_y);
        pos_z = _mm256_fmadd_pd(vel_z, dt_vec, pos_z);

        // Store results
        _mm256_storeu_pd(&px[i], pos_x);
        _mm256_storeu_pd(&py[i], pos_y);
        _mm256_storeu_pd(&pz[i], pos_z);
    }

    // Handle remaining elements
    for (; i < count; ++i) {
        px[i] += vx[i] * dt;
        py[i] += vy[i] * dt;
        pz[i] += vz[i] * dt;
    }
}

void batch_velocity_update(
    Real* __restrict vx, Real* __restrict vy, Real* __restrict vz,
    const Real* __restrict ax, const Real* __restrict ay, const Real* __restrict az,
    SizeT count, Real dt)
{
    const __m256d dt_vec = _mm256_set1_pd(dt);

    SizeT i = 0;
    for (; i + AVX_DOUBLES <= count; i += AVX_DOUBLES) {
        // Load velocities
        __m256d vel_x = _mm256_loadu_pd(&vx[i]);
        __m256d vel_y = _mm256_loadu_pd(&vy[i]);
        __m256d vel_z = _mm256_loadu_pd(&vz[i]);

        // Load accelerations
        __m256d acc_x = _mm256_loadu_pd(&ax[i]);
        __m256d acc_y = _mm256_loadu_pd(&ay[i]);
        __m256d acc_z = _mm256_loadu_pd(&az[i]);

        // v = v + a * dt (using FMA)
        vel_x = _mm256_fmadd_pd(acc_x, dt_vec, vel_x);
        vel_y = _mm256_fmadd_pd(acc_y, dt_vec, vel_y);
        vel_z = _mm256_fmadd_pd(acc_z, dt_vec, vel_z);

        // Store results
        _mm256_storeu_pd(&vx[i], vel_x);
        _mm256_storeu_pd(&vy[i], vel_y);
        _mm256_storeu_pd(&vz[i], vel_z);
    }

    // Handle remaining elements
    for (; i < count; ++i) {
        vx[i] += ax[i] * dt;
        vy[i] += ay[i] * dt;
        vz[i] += az[i] * dt;
    }
}

void batch_force_to_acceleration(
    Real* __restrict ax, Real* __restrict ay, Real* __restrict az,
    const Real* __restrict fx, const Real* __restrict fy, const Real* __restrict fz,
    const Real* __restrict masses,
    SizeT count)
{
    SizeT i = 0;
    for (; i + AVX_DOUBLES <= count; i += AVX_DOUBLES) {
        // Load masses and compute 1/m
        __m256d m = _mm256_loadu_pd(&masses[i]);
        __m256d inv_m = _mm256_div_pd(_mm256_set1_pd(1.0), m);

        // Load forces
        __m256d force_x = _mm256_loadu_pd(&fx[i]);
        __m256d force_y = _mm256_loadu_pd(&fy[i]);
        __m256d force_z = _mm256_loadu_pd(&fz[i]);

        // a = F / m
        __m256d acc_x = _mm256_mul_pd(force_x, inv_m);
        __m256d acc_y = _mm256_mul_pd(force_y, inv_m);
        __m256d acc_z = _mm256_mul_pd(force_z, inv_m);

        // Store results
        _mm256_storeu_pd(&ax[i], acc_x);
        _mm256_storeu_pd(&ay[i], acc_y);
        _mm256_storeu_pd(&az[i], acc_z);
    }

    // Handle remaining elements
    for (; i < count; ++i) {
        Real inv_m = 1.0 / masses[i];
        ax[i] = fx[i] * inv_m;
        ay[i] = fy[i] * inv_m;
        az[i] = fz[i] * inv_m;
    }
}

void batch_add_gravity(
    Real* __restrict fz,
    const Real* __restrict masses,
    Real g,
    SizeT count)
{
    const __m256d g_vec = _mm256_set1_pd(g);

    SizeT i = 0;
    for (; i + AVX_DOUBLES <= count; i += AVX_DOUBLES) {
        __m256d force_z = _mm256_loadu_pd(&fz[i]);
        __m256d m = _mm256_loadu_pd(&masses[i]);

        // F_z += m * g (FMA)
        force_z = _mm256_fmadd_pd(m, g_vec, force_z);

        _mm256_storeu_pd(&fz[i], force_z);
    }

    for (; i < count; ++i) {
        fz[i] += masses[i] * g;
    }
}

void batch_scale(
    Real* __restrict out,
    const Real* __restrict in,
    Real scalar,
    SizeT count)
{
    const __m256d s = _mm256_set1_pd(scalar);

    SizeT i = 0;
    for (; i + AVX_DOUBLES <= count; i += AVX_DOUBLES) {
        __m256d v = _mm256_loadu_pd(&in[i]);
        v = _mm256_mul_pd(v, s);
        _mm256_storeu_pd(&out[i], v);
    }

    for (; i < count; ++i) {
        out[i] = in[i] * scalar;
    }
}

void batch_add(
    Real* __restrict out,
    const Real* __restrict a,
    const Real* __restrict b,
    SizeT count)
{
    SizeT i = 0;
    for (; i + AVX_DOUBLES <= count; i += AVX_DOUBLES) {
        __m256d va = _mm256_loadu_pd(&a[i]);
        __m256d vb = _mm256_loadu_pd(&b[i]);
        __m256d result = _mm256_add_pd(va, vb);
        _mm256_storeu_pd(&out[i], result);
    }

    for (; i < count; ++i) {
        out[i] = a[i] + b[i];
    }
}

void batch_fma(
    Real* __restrict out,
    const Real* __restrict a,
    const Real* __restrict b,
    Real scalar,
    SizeT count)
{
    const __m256d s = _mm256_set1_pd(scalar);

    SizeT i = 0;
    for (; i + AVX_DOUBLES <= count; i += AVX_DOUBLES) {
        __m256d va = _mm256_loadu_pd(&a[i]);
        __m256d vb = _mm256_loadu_pd(&b[i]);
        // out = a + b * scalar
        __m256d result = _mm256_fmadd_pd(vb, s, va);
        _mm256_storeu_pd(&out[i], result);
    }

    for (; i < count; ++i) {
        out[i] = a[i] + b[i] * scalar;
    }
}

void batch_dot_product(
    Real* __restrict out,
    const Real* __restrict ax, const Real* __restrict ay, const Real* __restrict az,
    const Real* __restrict bx, const Real* __restrict by, const Real* __restrict bz,
    SizeT count)
{
    SizeT i = 0;
    for (; i + AVX_DOUBLES <= count; i += AVX_DOUBLES) {
        __m256d vax = _mm256_loadu_pd(&ax[i]);
        __m256d vay = _mm256_loadu_pd(&ay[i]);
        __m256d vaz = _mm256_loadu_pd(&az[i]);

        __m256d vbx = _mm256_loadu_pd(&bx[i]);
        __m256d vby = _mm256_loadu_pd(&by[i]);
        __m256d vbz = _mm256_loadu_pd(&bz[i]);

        // dot = ax*bx + ay*by + az*bz
        __m256d dot = _mm256_mul_pd(vax, vbx);
        dot = _mm256_fmadd_pd(vay, vby, dot);
        dot = _mm256_fmadd_pd(vaz, vbz, dot);

        _mm256_storeu_pd(&out[i], dot);
    }

    for (; i < count; ++i) {
        out[i] = ax[i] * bx[i] + ay[i] * by[i] + az[i] * bz[i];
    }
}

void batch_length_squared(
    Real* __restrict out,
    const Real* __restrict vx,
    const Real* __restrict vy,
    const Real* __restrict vz,
    SizeT count)
{
    SizeT i = 0;
    for (; i + AVX_DOUBLES <= count; i += AVX_DOUBLES) {
        __m256d x = _mm256_loadu_pd(&vx[i]);
        __m256d y = _mm256_loadu_pd(&vy[i]);
        __m256d z = _mm256_loadu_pd(&vz[i]);

        // len_sq = x*x + y*y + z*z
        __m256d len_sq = _mm256_mul_pd(x, x);
        len_sq = _mm256_fmadd_pd(y, y, len_sq);
        len_sq = _mm256_fmadd_pd(z, z, len_sq);

        _mm256_storeu_pd(&out[i], len_sq);
    }

    for (; i < count; ++i) {
        out[i] = vx[i] * vx[i] + vy[i] * vy[i] + vz[i] * vz[i];
    }
}

#else // Scalar fallback

// ============================================================================
// Scalar Fallback Implementation
// ============================================================================

void batch_position_update(
    Real* __restrict px, Real* __restrict py, Real* __restrict pz,
    const Real* __restrict vx, const Real* __restrict vy, const Real* __restrict vz,
    SizeT count, Real dt)
{
    for (SizeT i = 0; i < count; ++i) {
        px[i] += vx[i] * dt;
        py[i] += vy[i] * dt;
        pz[i] += vz[i] * dt;
    }
}

void batch_velocity_update(
    Real* __restrict vx, Real* __restrict vy, Real* __restrict vz,
    const Real* __restrict ax, const Real* __restrict ay, const Real* __restrict az,
    SizeT count, Real dt)
{
    for (SizeT i = 0; i < count; ++i) {
        vx[i] += ax[i] * dt;
        vy[i] += ay[i] * dt;
        vz[i] += az[i] * dt;
    }
}

void batch_force_to_acceleration(
    Real* __restrict ax, Real* __restrict ay, Real* __restrict az,
    const Real* __restrict fx, const Real* __restrict fy, const Real* __restrict fz,
    const Real* __restrict masses,
    SizeT count)
{
    for (SizeT i = 0; i < count; ++i) {
        Real inv_m = 1.0 / masses[i];
        ax[i] = fx[i] * inv_m;
        ay[i] = fy[i] * inv_m;
        az[i] = fz[i] * inv_m;
    }
}

void batch_add_gravity(
    Real* __restrict fz,
    const Real* __restrict masses,
    Real g,
    SizeT count)
{
    for (SizeT i = 0; i < count; ++i) {
        fz[i] += masses[i] * g;
    }
}

void batch_scale(
    Real* __restrict out,
    const Real* __restrict in,
    Real scalar,
    SizeT count)
{
    for (SizeT i = 0; i < count; ++i) {
        out[i] = in[i] * scalar;
    }
}

void batch_add(
    Real* __restrict out,
    const Real* __restrict a,
    const Real* __restrict b,
    SizeT count)
{
    for (SizeT i = 0; i < count; ++i) {
        out[i] = a[i] + b[i];
    }
}

void batch_fma(
    Real* __restrict out,
    const Real* __restrict a,
    const Real* __restrict b,
    Real scalar,
    SizeT count)
{
    for (SizeT i = 0; i < count; ++i) {
        out[i] = a[i] + b[i] * scalar;
    }
}

void batch_dot_product(
    Real* __restrict out,
    const Real* __restrict ax, const Real* __restrict ay, const Real* __restrict az,
    const Real* __restrict bx, const Real* __restrict by, const Real* __restrict bz,
    SizeT count)
{
    for (SizeT i = 0; i < count; ++i) {
        out[i] = ax[i] * bx[i] + ay[i] * by[i] + az[i] * bz[i];
    }
}

void batch_length_squared(
    Real* __restrict out,
    const Real* __restrict vx,
    const Real* __restrict vy,
    const Real* __restrict vz,
    SizeT count)
{
    for (SizeT i = 0; i < count; ++i) {
        out[i] = vx[i] * vx[i] + vy[i] * vy[i] + vz[i] * vz[i];
    }
}

#endif // JAGUAR_HAS_AVX2

} // namespace jaguar::simd
