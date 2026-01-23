/**
 * @file physics_kernels.cpp
 * @brief GPU-accelerated physics computation kernels implementation
 *
 * This file implements the PhysicsKernelManager and all physics kernels
 * for GPU and CPU backends. The kernels are designed for high-throughput
 * batch processing of physics entities.
 *
 * Implementation Strategy:
 * 1. Each kernel has a GPU implementation (CUDA/OpenCL/Metal source)
 * 2. Each kernel has a CPU fallback implementation
 * 3. The manager selects the appropriate implementation at runtime
 *
 * Kernel Source Organization:
 * - CUDA: Inline PTX-compatible source strings
 * - OpenCL: Standard OpenCL C kernel source
 * - Metal: MSL shader source strings
 * - CPU: Native C++ implementations in cpu_kernels namespace
 */

#include "jaguar/gpu/physics_kernels.h"
#include <cmath>
#include <algorithm>
#include <unordered_set>
#include <thread>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace jaguar::gpu {

// ============================================================================
// Kernel Source Strings
// ============================================================================

namespace kernel_source {

// ----------------------------------------------------------------------------
// Integration Kernels - CUDA
// ----------------------------------------------------------------------------

const char* cuda_integrate_positions = R"(
extern "C" __global__ void integrate_positions(
    float* positions,
    const float* velocities,
    float dt,
    int count)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < count) {
        int i3 = idx * 3;
        positions[i3 + 0] += velocities[i3 + 0] * dt;
        positions[i3 + 1] += velocities[i3 + 1] * dt;
        positions[i3 + 2] += velocities[i3 + 2] * dt;
    }
}
)";

const char* cuda_integrate_velocities = R"(
extern "C" __global__ void integrate_velocities(
    float* velocities,
    const float* forces,
    const float* masses,
    float gravity_x,
    float gravity_y,
    float gravity_z,
    float dt,
    int count)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < count) {
        int i3 = idx * 3;
        float inv_mass = (masses[idx] > 0.0f) ? (1.0f / masses[idx]) : 0.0f;

        velocities[i3 + 0] += (forces[i3 + 0] * inv_mass + gravity_x) * dt;
        velocities[i3 + 1] += (forces[i3 + 1] * inv_mass + gravity_y) * dt;
        velocities[i3 + 2] += (forces[i3 + 2] * inv_mass + gravity_z) * dt;
    }
}
)";

const char* cuda_integrate_orientations = R"(
extern "C" __global__ void integrate_orientations(
    float* orientations,
    const float* angular_velocities,
    float dt,
    int count)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < count) {
        int i4 = idx * 4;
        int i3 = idx * 3;

        // Load quaternion (w, x, y, z)
        float qw = orientations[i4 + 0];
        float qx = orientations[i4 + 1];
        float qy = orientations[i4 + 2];
        float qz = orientations[i4 + 3];

        // Load angular velocity
        float wx = angular_velocities[i3 + 0];
        float wy = angular_velocities[i3 + 1];
        float wz = angular_velocities[i3 + 2];

        // Quaternion derivative: q_dot = 0.5 * omega * q
        // where omega is [0, wx, wy, wz] quaternion
        float half_dt = 0.5f * dt;
        float dqw = -half_dt * (wx * qx + wy * qy + wz * qz);
        float dqx =  half_dt * (wx * qw + wz * qy - wy * qz);
        float dqy =  half_dt * (wy * qw - wz * qx + wx * qz);
        float dqz =  half_dt * (wz * qw + wy * qx - wx * qy);

        // Update quaternion
        qw += dqw;
        qx += dqx;
        qy += dqy;
        qz += dqz;

        // Normalize
        float norm = sqrtf(qw*qw + qx*qx + qy*qy + qz*qz);
        if (norm > 0.0f) {
            float inv_norm = 1.0f / norm;
            qw *= inv_norm;
            qx *= inv_norm;
            qy *= inv_norm;
            qz *= inv_norm;
        }

        // Store result
        orientations[i4 + 0] = qw;
        orientations[i4 + 1] = qx;
        orientations[i4 + 2] = qy;
        orientations[i4 + 3] = qz;
    }
}
)";

// ----------------------------------------------------------------------------
// Collision Detection Kernels - CUDA
// ----------------------------------------------------------------------------

const char* cuda_update_aabbs = R"(
extern "C" __global__ void update_aabbs(
    const float* positions,
    const float* extents,
    float* aabbs,  // min_x, min_y, min_z, max_x, max_y, max_z, entity_id, padding
    int count)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < count) {
        int i3 = idx * 3;
        int i8 = idx * 8;

        float px = positions[i3 + 0];
        float py = positions[i3 + 1];
        float pz = positions[i3 + 2];

        float ex = extents[i3 + 0];
        float ey = extents[i3 + 1];
        float ez = extents[i3 + 2];

        aabbs[i8 + 0] = px - ex;  // min_x
        aabbs[i8 + 1] = py - ey;  // min_y
        aabbs[i8 + 2] = pz - ez;  // min_z
        aabbs[i8 + 3] = px + ex;  // max_x
        aabbs[i8 + 4] = py + ey;  // max_y
        aabbs[i8 + 5] = pz + ez;  // max_z
        // entity_id and padding remain unchanged
    }
}
)";

const char* cuda_collision_broad_phase = R"(
// Spatial hash function
__device__ unsigned int hash_cell(int cx, int cy, int cz) {
    // Simple hash combining cell coordinates
    const unsigned int p1 = 73856093u;
    const unsigned int p2 = 19349663u;
    const unsigned int p3 = 83492791u;
    return (unsigned int)(cx) * p1 ^ (unsigned int)(cy) * p2 ^ (unsigned int)(cz) * p3;
}

extern "C" __global__ void collision_broad_phase(
    const float* aabbs,      // [count * 8]: min_xyz, max_xyz, entity_id, padding
    int* cell_counts,        // [num_cells]: count per cell
    int* cell_entities,      // [num_cells * max_per_cell]: entity indices
    int* pairs,              // [max_pairs * 2]: collision pair output
    int* pair_count,         // [1]: atomic counter for pairs
    float cell_size,
    float world_min_x,
    float world_min_y,
    float world_min_z,
    int num_cells,
    int max_per_cell,
    int max_pairs,
    int count)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= count) return;

    int i8 = idx * 8;

    // Load AABB
    float min_x = aabbs[i8 + 0];
    float min_y = aabbs[i8 + 1];
    float min_z = aabbs[i8 + 2];
    float max_x = aabbs[i8 + 3];
    float max_y = aabbs[i8 + 4];
    float max_z = aabbs[i8 + 5];
    int entity_id = __float_as_int(aabbs[i8 + 6]);

    // Compute cell range
    float inv_cell = 1.0f / cell_size;
    int cx0 = (int)floorf((min_x - world_min_x) * inv_cell);
    int cy0 = (int)floorf((min_y - world_min_y) * inv_cell);
    int cz0 = (int)floorf((min_z - world_min_z) * inv_cell);
    int cx1 = (int)floorf((max_x - world_min_x) * inv_cell);
    int cy1 = (int)floorf((max_y - world_min_y) * inv_cell);
    int cz1 = (int)floorf((max_z - world_min_z) * inv_cell);

    // Insert into cells
    for (int cz = cz0; cz <= cz1; ++cz) {
        for (int cy = cy0; cy <= cy1; ++cy) {
            for (int cx = cx0; cx <= cx1; ++cx) {
                unsigned int cell_hash = hash_cell(cx, cy, cz) % num_cells;
                int slot = atomicAdd(&cell_counts[cell_hash], 1);
                if (slot < max_per_cell) {
                    cell_entities[cell_hash * max_per_cell + slot] = idx;
                }
            }
        }
    }
}

extern "C" __global__ void generate_pairs(
    const float* aabbs,
    const int* cell_counts,
    const int* cell_entities,
    int* pairs,
    int* pair_count,
    int num_cells,
    int max_per_cell,
    int max_pairs)
{
    int cell_idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (cell_idx >= num_cells) return;

    int cell_count = cell_counts[cell_idx];
    if (cell_count < 2) return;

    const int* cell_list = cell_entities + cell_idx * max_per_cell;

    // Check all pairs in this cell
    for (int i = 0; i < cell_count && i < max_per_cell; ++i) {
        int idx_a = cell_list[i];
        int i8_a = idx_a * 8;

        float min_ax = aabbs[i8_a + 0];
        float min_ay = aabbs[i8_a + 1];
        float min_az = aabbs[i8_a + 2];
        float max_ax = aabbs[i8_a + 3];
        float max_ay = aabbs[i8_a + 4];
        float max_az = aabbs[i8_a + 5];
        int entity_a = __float_as_int(aabbs[i8_a + 6]);

        for (int j = i + 1; j < cell_count && j < max_per_cell; ++j) {
            int idx_b = cell_list[j];
            int i8_b = idx_b * 8;

            float min_bx = aabbs[i8_b + 0];
            float min_by = aabbs[i8_b + 1];
            float min_bz = aabbs[i8_b + 2];
            float max_bx = aabbs[i8_b + 3];
            float max_by = aabbs[i8_b + 4];
            float max_bz = aabbs[i8_b + 5];
            int entity_b = __float_as_int(aabbs[i8_b + 6]);

            // AABB intersection test
            bool intersects = (min_ax <= max_bx && max_ax >= min_bx) &&
                              (min_ay <= max_by && max_ay >= min_by) &&
                              (min_az <= max_bz && max_az >= min_bz);

            if (intersects && entity_a != entity_b) {
                // Canonical ordering
                int pair_a = (entity_a < entity_b) ? entity_a : entity_b;
                int pair_b = (entity_a < entity_b) ? entity_b : entity_a;

                int slot = atomicAdd(pair_count, 1);
                if (slot < max_pairs) {
                    pairs[slot * 2 + 0] = pair_a;
                    pairs[slot * 2 + 1] = pair_b;
                }
            }
        }
    }
}
)";

// ----------------------------------------------------------------------------
// Utility Kernels - CUDA
// ----------------------------------------------------------------------------

const char* cuda_apply_gravity = R"(
extern "C" __global__ void apply_gravity(
    float* forces,
    const float* masses,
    float gravity_x,
    float gravity_y,
    float gravity_z,
    int count)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < count) {
        int i3 = idx * 3;
        float mass = masses[idx];
        forces[i3 + 0] += mass * gravity_x;
        forces[i3 + 1] += mass * gravity_y;
        forces[i3 + 2] += mass * gravity_z;
    }
}
)";

const char* cuda_apply_damping = R"(
extern "C" __global__ void apply_damping(
    float* velocities,
    float damping,
    float dt,
    int count)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < count) {
        int i3 = idx * 3;
        float factor = 1.0f - damping * dt;
        velocities[i3 + 0] *= factor;
        velocities[i3 + 1] *= factor;
        velocities[i3 + 2] *= factor;
    }
}
)";

const char* cuda_clear_forces = R"(
extern "C" __global__ void clear_forces(
    float* forces,
    float* torques,
    int count)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < count) {
        int i3 = idx * 3;
        forces[i3 + 0] = 0.0f;
        forces[i3 + 1] = 0.0f;
        forces[i3 + 2] = 0.0f;
        torques[i3 + 0] = 0.0f;
        torques[i3 + 1] = 0.0f;
        torques[i3 + 2] = 0.0f;
    }
}
)";

const char* cuda_clamp_velocities = R"(
extern "C" __global__ void clamp_velocities(
    float* velocities,
    float max_speed,
    int count)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < count) {
        int i3 = idx * 3;
        float vx = velocities[i3 + 0];
        float vy = velocities[i3 + 1];
        float vz = velocities[i3 + 2];

        float speed_sq = vx*vx + vy*vy + vz*vz;
        float max_speed_sq = max_speed * max_speed;

        if (speed_sq > max_speed_sq && speed_sq > 0.0f) {
            float scale = max_speed / sqrtf(speed_sq);
            velocities[i3 + 0] = vx * scale;
            velocities[i3 + 1] = vy * scale;
            velocities[i3 + 2] = vz * scale;
        }
    }
}
)";

// ----------------------------------------------------------------------------
// Terrain Sampling Kernel - CUDA
// ----------------------------------------------------------------------------

const char* cuda_sample_terrain = R"(
extern "C" __global__ void sample_terrain(
    const float* requests,   // [count * 4]: x, y, z, entity_id
    float* results,          // [count * 8]: height, nx, ny, nz, friction, material, entity_id, pad
    const float* heightmap,
    int width,
    int height,
    float origin_x,
    float origin_y,
    float scale,
    int count)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= count) return;

    int i4 = idx * 4;
    int i8 = idx * 8;

    float x = requests[i4 + 0];
    float y = requests[i4 + 1];
    float z = requests[i4 + 2];
    int entity_id = __float_as_int(requests[i4 + 3]);

    // Convert to heightmap coordinates
    float u = (x - origin_x) / scale;
    float v = (y - origin_y) / scale;

    // Clamp to valid range
    u = fmaxf(0.0f, fminf(u, (float)(width - 1)));
    v = fmaxf(0.0f, fminf(v, (float)(height - 1)));

    // Bilinear interpolation
    int u0 = (int)floorf(u);
    int v0 = (int)floorf(v);
    int u1 = min(u0 + 1, width - 1);
    int v1 = min(v0 + 1, height - 1);

    float fu = u - u0;
    float fv = v - v0;

    float h00 = heightmap[v0 * width + u0];
    float h10 = heightmap[v0 * width + u1];
    float h01 = heightmap[v1 * width + u0];
    float h11 = heightmap[v1 * width + u1];

    float terrain_height = (1-fu)*(1-fv)*h00 + fu*(1-fv)*h10 + (1-fu)*fv*h01 + fu*fv*h11;

    // Compute normal from gradient
    float dhdx = (h10 - h00) / scale;
    float dhdy = (h01 - h00) / scale;

    float nx = -dhdx;
    float ny = -dhdy;
    float nz = 1.0f;
    float len = sqrtf(nx*nx + ny*ny + nz*nz);
    if (len > 0.0f) {
        nx /= len;
        ny /= len;
        nz /= len;
    }

    // Output
    results[i8 + 0] = terrain_height;
    results[i8 + 1] = nx;
    results[i8 + 2] = ny;
    results[i8 + 3] = nz;
    results[i8 + 4] = 0.7f;  // Default friction
    results[i8 + 5] = __int_as_float(0);  // Material ID
    results[i8 + 6] = __int_as_float(entity_id);
    results[i8 + 7] = 0.0f;  // Padding
}
)";

// ----------------------------------------------------------------------------
// Aerodynamics Kernel - CUDA
// ----------------------------------------------------------------------------

const char* cuda_compute_aero_forces = R"(
extern "C" __global__ void compute_aero_forces(
    const float* inputs,   // [count * 16]: AeroInput structure
    float* outputs,        // [count * 16]: AeroOutput structure
    int count)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= count) return;

    int i16 = idx * 16;

    // Load input (simplified - actual input is larger)
    float pos_x = inputs[i16 + 0];
    float pos_y = inputs[i16 + 1];
    float pos_z = inputs[i16 + 2];
    float vel_x = inputs[i16 + 3];
    float vel_y = inputs[i16 + 4];
    float vel_z = inputs[i16 + 5];
    float qw = inputs[i16 + 6];
    float qx = inputs[i16 + 7];
    float qy = inputs[i16 + 8];
    float qz = inputs[i16 + 9];
    float rho = inputs[i16 + 10];  // Air density
    float sos = inputs[i16 + 11];  // Speed of sound

    // Compute airspeed in body frame
    // Simplified: assume no wind
    float airspeed = sqrtf(vel_x*vel_x + vel_y*vel_y + vel_z*vel_z);

    // Dynamic pressure
    float q = 0.5f * rho * airspeed * airspeed;

    // Mach number
    float mach = (sos > 0.0f) ? (airspeed / sos) : 0.0f;

    // Simplified angle of attack (using velocity direction)
    float alpha = 0.0f;
    float beta = 0.0f;
    if (airspeed > 1.0f) {
        // Transform velocity to body frame (simplified)
        alpha = atan2f(-vel_z, vel_x);
        beta = asinf(vel_y / airspeed);
    }

    // Simplified aerodynamic coefficients
    float S = 20.0f;  // Reference area (m^2)

    // Basic lift/drag model
    float CL = 0.1f + 4.0f * alpha;  // Simplified
    float CD = 0.02f + 0.05f * CL * CL;  // Induced drag
    float CM = -0.01f * alpha;  // Pitching moment

    // Forces in wind frame
    float lift = q * S * CL;
    float drag = q * S * CD;

    // Convert to body frame (simplified)
    float fx = -drag;
    float fy = 0.0f;
    float fz = -lift;

    // Moments
    float c = 2.0f;  // Mean chord
    float mx = 0.0f;
    float my = q * S * c * CM;
    float mz = 0.0f;

    // Output
    outputs[i16 + 0] = fx;
    outputs[i16 + 1] = fy;
    outputs[i16 + 2] = fz;
    outputs[i16 + 3] = mx;
    outputs[i16 + 4] = my;
    outputs[i16 + 5] = mz;
    outputs[i16 + 6] = alpha;
    outputs[i16 + 7] = beta;
    outputs[i16 + 8] = mach;
    outputs[i16 + 9] = q;
    outputs[i16 + 10] = CL;
    outputs[i16 + 11] = CD;
    outputs[i16 + 12] = __int_as_float((int)inputs[i16 + 14]);  // entity_id
    outputs[i16 + 13] = 0.0f;
    outputs[i16 + 14] = 0.0f;
    outputs[i16 + 15] = 0.0f;
}
)";

// ----------------------------------------------------------------------------
// OpenCL Kernel Sources
// ----------------------------------------------------------------------------

const char* opencl_integrate_positions = R"(
__kernel void integrate_positions(
    __global float* positions,
    __global const float* velocities,
    float dt,
    int count)
{
    int idx = get_global_id(0);
    if (idx < count) {
        int i3 = idx * 3;
        positions[i3 + 0] += velocities[i3 + 0] * dt;
        positions[i3 + 1] += velocities[i3 + 1] * dt;
        positions[i3 + 2] += velocities[i3 + 2] * dt;
    }
}
)";

const char* opencl_integrate_velocities = R"(
__kernel void integrate_velocities(
    __global float* velocities,
    __global const float* forces,
    __global const float* masses,
    float gravity_x,
    float gravity_y,
    float gravity_z,
    float dt,
    int count)
{
    int idx = get_global_id(0);
    if (idx < count) {
        int i3 = idx * 3;
        float inv_mass = (masses[idx] > 0.0f) ? (1.0f / masses[idx]) : 0.0f;

        velocities[i3 + 0] += (forces[i3 + 0] * inv_mass + gravity_x) * dt;
        velocities[i3 + 1] += (forces[i3 + 1] * inv_mass + gravity_y) * dt;
        velocities[i3 + 2] += (forces[i3 + 2] * inv_mass + gravity_z) * dt;
    }
}
)";

const char* opencl_integrate_orientations = R"(
__kernel void integrate_orientations(
    __global float* orientations,
    __global const float* angular_velocities,
    float dt,
    int count)
{
    int idx = get_global_id(0);
    if (idx < count) {
        int i4 = idx * 4;
        int i3 = idx * 3;

        float qw = orientations[i4 + 0];
        float qx = orientations[i4 + 1];
        float qy = orientations[i4 + 2];
        float qz = orientations[i4 + 3];

        float wx = angular_velocities[i3 + 0];
        float wy = angular_velocities[i3 + 1];
        float wz = angular_velocities[i3 + 2];

        float half_dt = 0.5f * dt;
        float dqw = -half_dt * (wx * qx + wy * qy + wz * qz);
        float dqx =  half_dt * (wx * qw + wz * qy - wy * qz);
        float dqy =  half_dt * (wy * qw - wz * qx + wx * qz);
        float dqz =  half_dt * (wz * qw + wy * qx - wx * qy);

        qw += dqw;
        qx += dqx;
        qy += dqy;
        qz += dqz;

        float norm = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
        if (norm > 0.0f) {
            float inv_norm = 1.0f / norm;
            qw *= inv_norm;
            qx *= inv_norm;
            qy *= inv_norm;
            qz *= inv_norm;
        }

        orientations[i4 + 0] = qw;
        orientations[i4 + 1] = qx;
        orientations[i4 + 2] = qy;
        orientations[i4 + 3] = qz;
    }
}
)";

const char* opencl_update_aabbs = R"(
__kernel void update_aabbs(
    __global const float* positions,
    __global const float* extents,
    __global float* aabbs,
    int count)
{
    int idx = get_global_id(0);
    if (idx < count) {
        int i3 = idx * 3;
        int i8 = idx * 8;

        float px = positions[i3 + 0];
        float py = positions[i3 + 1];
        float pz = positions[i3 + 2];

        float ex = extents[i3 + 0];
        float ey = extents[i3 + 1];
        float ez = extents[i3 + 2];

        aabbs[i8 + 0] = px - ex;
        aabbs[i8 + 1] = py - ey;
        aabbs[i8 + 2] = pz - ez;
        aabbs[i8 + 3] = px + ex;
        aabbs[i8 + 4] = py + ey;
        aabbs[i8 + 5] = pz + ez;
    }
}
)";

const char* opencl_apply_gravity = R"(
__kernel void apply_gravity(
    __global float* forces,
    __global const float* masses,
    float gravity_x,
    float gravity_y,
    float gravity_z,
    int count)
{
    int idx = get_global_id(0);
    if (idx < count) {
        int i3 = idx * 3;
        float mass = masses[idx];
        forces[i3 + 0] += mass * gravity_x;
        forces[i3 + 1] += mass * gravity_y;
        forces[i3 + 2] += mass * gravity_z;
    }
}
)";

const char* opencl_apply_damping = R"(
__kernel void apply_damping(
    __global float* velocities,
    float damping,
    float dt,
    int count)
{
    int idx = get_global_id(0);
    if (idx < count) {
        int i3 = idx * 3;
        float factor = 1.0f - damping * dt;
        velocities[i3 + 0] *= factor;
        velocities[i3 + 1] *= factor;
        velocities[i3 + 2] *= factor;
    }
}
)";

const char* opencl_clear_forces = R"(
__kernel void clear_forces(
    __global float* forces,
    __global float* torques,
    int count)
{
    int idx = get_global_id(0);
    if (idx < count) {
        int i3 = idx * 3;
        forces[i3 + 0] = 0.0f;
        forces[i3 + 1] = 0.0f;
        forces[i3 + 2] = 0.0f;
        torques[i3 + 0] = 0.0f;
        torques[i3 + 1] = 0.0f;
        torques[i3 + 2] = 0.0f;
    }
}
)";

// ----------------------------------------------------------------------------
// Metal Kernel Sources
// ----------------------------------------------------------------------------

const char* metal_integrate_positions = R"(
#include <metal_stdlib>
using namespace metal;

kernel void integrate_positions(
    device float* positions [[buffer(0)]],
    device const float* velocities [[buffer(1)]],
    constant float& dt [[buffer(2)]],
    constant int& count [[buffer(3)]],
    uint idx [[thread_position_in_grid]])
{
    if ((int)idx < count) {
        int i3 = idx * 3;
        positions[i3 + 0] += velocities[i3 + 0] * dt;
        positions[i3 + 1] += velocities[i3 + 1] * dt;
        positions[i3 + 2] += velocities[i3 + 2] * dt;
    }
}
)";

const char* metal_integrate_velocities = R"(
#include <metal_stdlib>
using namespace metal;

kernel void integrate_velocities(
    device float* velocities [[buffer(0)]],
    device const float* forces [[buffer(1)]],
    device const float* masses [[buffer(2)]],
    constant float& gravity_x [[buffer(3)]],
    constant float& gravity_y [[buffer(4)]],
    constant float& gravity_z [[buffer(5)]],
    constant float& dt [[buffer(6)]],
    constant int& count [[buffer(7)]],
    uint idx [[thread_position_in_grid]])
{
    if ((int)idx < count) {
        int i3 = idx * 3;
        float inv_mass = (masses[idx] > 0.0f) ? (1.0f / masses[idx]) : 0.0f;

        velocities[i3 + 0] += (forces[i3 + 0] * inv_mass + gravity_x) * dt;
        velocities[i3 + 1] += (forces[i3 + 1] * inv_mass + gravity_y) * dt;
        velocities[i3 + 2] += (forces[i3 + 2] * inv_mass + gravity_z) * dt;
    }
}
)";

const char* metal_integrate_orientations = R"(
#include <metal_stdlib>
using namespace metal;

kernel void integrate_orientations(
    device float* orientations [[buffer(0)]],
    device const float* angular_velocities [[buffer(1)]],
    constant float& dt [[buffer(2)]],
    constant int& count [[buffer(3)]],
    uint idx [[thread_position_in_grid]])
{
    if ((int)idx < count) {
        int i4 = idx * 4;
        int i3 = idx * 3;

        float qw = orientations[i4 + 0];
        float qx = orientations[i4 + 1];
        float qy = orientations[i4 + 2];
        float qz = orientations[i4 + 3];

        float wx = angular_velocities[i3 + 0];
        float wy = angular_velocities[i3 + 1];
        float wz = angular_velocities[i3 + 2];

        float half_dt = 0.5f * dt;
        float dqw = -half_dt * (wx * qx + wy * qy + wz * qz);
        float dqx =  half_dt * (wx * qw + wz * qy - wy * qz);
        float dqy =  half_dt * (wy * qw - wz * qx + wx * qz);
        float dqz =  half_dt * (wz * qw + wy * qx - wx * qy);

        qw += dqw;
        qx += dqx;
        qy += dqy;
        qz += dqz;

        float norm = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
        if (norm > 0.0f) {
            float inv_norm = 1.0f / norm;
            qw *= inv_norm;
            qx *= inv_norm;
            qy *= inv_norm;
            qz *= inv_norm;
        }

        orientations[i4 + 0] = qw;
        orientations[i4 + 1] = qx;
        orientations[i4 + 2] = qy;
        orientations[i4 + 3] = qz;
    }
}
)";

const char* metal_update_aabbs = R"(
#include <metal_stdlib>
using namespace metal;

kernel void update_aabbs(
    device const float* positions [[buffer(0)]],
    device const float* extents [[buffer(1)]],
    device float* aabbs [[buffer(2)]],
    constant int& count [[buffer(3)]],
    uint idx [[thread_position_in_grid]])
{
    if ((int)idx < count) {
        int i3 = idx * 3;
        int i8 = idx * 8;

        float px = positions[i3 + 0];
        float py = positions[i3 + 1];
        float pz = positions[i3 + 2];

        float ex = extents[i3 + 0];
        float ey = extents[i3 + 1];
        float ez = extents[i3 + 2];

        aabbs[i8 + 0] = px - ex;
        aabbs[i8 + 1] = py - ey;
        aabbs[i8 + 2] = pz - ez;
        aabbs[i8 + 3] = px + ex;
        aabbs[i8 + 4] = py + ey;
        aabbs[i8 + 5] = pz + ez;
    }
}
)";

const char* metal_apply_gravity = R"(
#include <metal_stdlib>
using namespace metal;

kernel void apply_gravity(
    device float* forces [[buffer(0)]],
    device const float* masses [[buffer(1)]],
    constant float& gravity_x [[buffer(2)]],
    constant float& gravity_y [[buffer(3)]],
    constant float& gravity_z [[buffer(4)]],
    constant int& count [[buffer(5)]],
    uint idx [[thread_position_in_grid]])
{
    if ((int)idx < count) {
        int i3 = idx * 3;
        float mass = masses[idx];
        forces[i3 + 0] += mass * gravity_x;
        forces[i3 + 1] += mass * gravity_y;
        forces[i3 + 2] += mass * gravity_z;
    }
}
)";

} // namespace kernel_source

// ============================================================================
// CPU Kernel Implementations
// ============================================================================

namespace cpu_kernels {

void integrate_positions_cpu(
    float* positions,
    const float* velocities,
    float dt,
    SizeT count)
{
    #ifdef _OPENMP
    #pragma omp parallel for
    #endif
    for (SizeT i = 0; i < count; ++i) {
        SizeT i3 = i * 3;
        positions[i3 + 0] += velocities[i3 + 0] * dt;
        positions[i3 + 1] += velocities[i3 + 1] * dt;
        positions[i3 + 2] += velocities[i3 + 2] * dt;
    }
}

void integrate_velocities_cpu(
    float* velocities,
    const float* forces,
    const float* masses,
    float gravity_x,
    float gravity_y,
    float gravity_z,
    float dt,
    SizeT count)
{
    #ifdef _OPENMP
    #pragma omp parallel for
    #endif
    for (SizeT i = 0; i < count; ++i) {
        SizeT i3 = i * 3;
        float inv_mass = (masses[i] > 0.0f) ? (1.0f / masses[i]) : 0.0f;

        velocities[i3 + 0] += (forces[i3 + 0] * inv_mass + gravity_x) * dt;
        velocities[i3 + 1] += (forces[i3 + 1] * inv_mass + gravity_y) * dt;
        velocities[i3 + 2] += (forces[i3 + 2] * inv_mass + gravity_z) * dt;
    }
}

void integrate_orientations_cpu(
    float* orientations,
    const float* angular_velocities,
    float dt,
    SizeT count)
{
    #ifdef _OPENMP
    #pragma omp parallel for
    #endif
    for (SizeT i = 0; i < count; ++i) {
        SizeT i4 = i * 4;
        SizeT i3 = i * 3;

        // Load quaternion (w, x, y, z)
        float qw = orientations[i4 + 0];
        float qx = orientations[i4 + 1];
        float qy = orientations[i4 + 2];
        float qz = orientations[i4 + 3];

        // Load angular velocity
        float wx = angular_velocities[i3 + 0];
        float wy = angular_velocities[i3 + 1];
        float wz = angular_velocities[i3 + 2];

        // Quaternion derivative: q_dot = 0.5 * omega * q
        float half_dt = 0.5f * dt;
        float dqw = -half_dt * (wx * qx + wy * qy + wz * qz);
        float dqx =  half_dt * (wx * qw + wz * qy - wy * qz);
        float dqy =  half_dt * (wy * qw - wz * qx + wx * qz);
        float dqz =  half_dt * (wz * qw + wy * qx - wx * qy);

        // Update quaternion
        qw += dqw;
        qx += dqx;
        qy += dqy;
        qz += dqz;

        // Normalize
        float norm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
        if (norm > 0.0f) {
            float inv_norm = 1.0f / norm;
            qw *= inv_norm;
            qx *= inv_norm;
            qy *= inv_norm;
            qz *= inv_norm;
        }

        // Store result
        orientations[i4 + 0] = qw;
        orientations[i4 + 1] = qx;
        orientations[i4 + 2] = qy;
        orientations[i4 + 3] = qz;
    }
}

void update_aabbs_cpu(
    const float* positions,
    const float* extents,
    AABB* aabbs,
    SizeT count)
{
    #ifdef _OPENMP
    #pragma omp parallel for
    #endif
    for (SizeT i = 0; i < count; ++i) {
        SizeT i3 = i * 3;

        float px = positions[i3 + 0];
        float py = positions[i3 + 1];
        float pz = positions[i3 + 2];

        float ex = extents[i3 + 0];
        float ey = extents[i3 + 1];
        float ez = extents[i3 + 2];

        aabbs[i].min_x = px - ex;
        aabbs[i].min_y = py - ey;
        aabbs[i].min_z = pz - ez;
        aabbs[i].max_x = px + ex;
        aabbs[i].max_y = py + ey;
        aabbs[i].max_z = pz + ez;
    }
}

// Hash function for spatial hashing
inline UInt32 hash_cell(int cx, int cy, int cz) {
    const UInt32 p1 = 73856093u;
    const UInt32 p2 = 19349663u;
    const UInt32 p3 = 83492791u;
    return static_cast<UInt32>(cx) * p1 ^ static_cast<UInt32>(cy) * p2 ^ static_cast<UInt32>(cz) * p3;
}

SizeT collision_broad_phase_cpu(
    const AABB* aabbs,
    SizeT count,
    CollisionPair* pairs,
    SizeT max_pairs,
    const SpatialHashConfig& config)
{
    if (count == 0) return 0;

    // Spatial hash grid
    const UInt32 num_cells = 65536;  // Hash table size
    const UInt32 max_per_cell = config.max_entities_per_cell;

    std::vector<std::vector<UInt32>> cells(num_cells);
    for (auto& cell : cells) {
        cell.reserve(max_per_cell);
    }

    float inv_cell = 1.0f / config.cell_size;

    // Insert AABBs into spatial hash
    for (SizeT i = 0; i < count; ++i) {
        const AABB& aabb = aabbs[i];

        int cx0 = static_cast<int>(std::floor((aabb.min_x - config.world_min_x) * inv_cell));
        int cy0 = static_cast<int>(std::floor((aabb.min_y - config.world_min_y) * inv_cell));
        int cz0 = static_cast<int>(std::floor((aabb.min_z - config.world_min_z) * inv_cell));
        int cx1 = static_cast<int>(std::floor((aabb.max_x - config.world_min_x) * inv_cell));
        int cy1 = static_cast<int>(std::floor((aabb.max_y - config.world_min_y) * inv_cell));
        int cz1 = static_cast<int>(std::floor((aabb.max_z - config.world_min_z) * inv_cell));

        for (int cz = cz0; cz <= cz1; ++cz) {
            for (int cy = cy0; cy <= cy1; ++cy) {
                for (int cx = cx0; cx <= cx1; ++cx) {
                    UInt32 cell_hash = hash_cell(cx, cy, cz) % num_cells;
                    if (cells[cell_hash].size() < max_per_cell) {
                        cells[cell_hash].push_back(static_cast<UInt32>(i));
                    }
                }
            }
        }
    }

    // Generate collision pairs (using set to avoid duplicates)
    std::unordered_set<UInt64> seen_pairs;
    SizeT pair_count = 0;

    for (const auto& cell : cells) {
        if (cell.size() < 2) continue;

        for (SizeT i = 0; i < cell.size(); ++i) {
            UInt32 idx_a = cell[i];
            const AABB& a = aabbs[idx_a];

            for (SizeT j = i + 1; j < cell.size(); ++j) {
                UInt32 idx_b = cell[j];
                const AABB& b = aabbs[idx_b];

                // AABB intersection test
                if (a.intersects(b)) {
                    UInt32 entity_a = a.entity_id;
                    UInt32 entity_b = b.entity_id;

                    if (entity_a != entity_b) {
                        // Canonical ordering for deduplication
                        UInt32 pair_a = std::min(entity_a, entity_b);
                        UInt32 pair_b = std::max(entity_a, entity_b);
                        UInt64 pair_key = (static_cast<UInt64>(pair_a) << 32) | pair_b;

                        if (seen_pairs.find(pair_key) == seen_pairs.end()) {
                            seen_pairs.insert(pair_key);

                            if (pair_count < max_pairs) {
                                pairs[pair_count].entity_a = pair_a;
                                pairs[pair_count].entity_b = pair_b;
                                ++pair_count;
                            }
                        }
                    }
                }
            }
        }
    }

    return pair_count;
}

void compute_aero_forces_cpu(
    const AeroInput* inputs,
    AeroOutput* outputs,
    SizeT count)
{
    #ifdef _OPENMP
    #pragma omp parallel for
    #endif
    for (SizeT i = 0; i < count; ++i) {
        const AeroInput& in = inputs[i];
        AeroOutput& out = outputs[i];

        // Compute airspeed (simplified - no wind for now)
        float airspeed = std::sqrt(in.vel_x*in.vel_x + in.vel_y*in.vel_y + in.vel_z*in.vel_z);

        // Dynamic pressure
        float q = 0.5f * in.air_density * airspeed * airspeed;

        // Mach number
        float mach = (in.speed_of_sound > 0.0f) ? (airspeed / in.speed_of_sound) : 0.0f;

        // Simplified angle of attack
        float alpha = 0.0f;
        float beta = 0.0f;
        if (airspeed > 1.0f) {
            alpha = std::atan2(-in.vel_z, in.vel_x);
            beta = std::asin(in.vel_y / airspeed);
        }

        // Reference values
        constexpr float S = 20.0f;   // Reference area (m^2)
        constexpr float c = 2.0f;    // Mean chord (m)

        // Simplified aerodynamic coefficients
        float CL = 0.1f + 4.0f * alpha;
        float CD = 0.02f + 0.05f * CL * CL;
        float CM = -0.01f * alpha;

        // Forces in wind frame
        float lift = q * S * CL;
        float drag = q * S * CD;

        // Body frame forces (simplified transformation)
        out.force_x = -drag;
        out.force_y = 0.0f;
        out.force_z = -lift;

        // Moments
        out.moment_x = 0.0f;
        out.moment_y = q * S * c * CM;
        out.moment_z = 0.0f;

        // Output values
        out.alpha = alpha;
        out.beta = beta;
        out.mach = mach;
        out.dynamic_pressure = q;
        out.lift_coeff = CL;
        out.drag_coeff = CD;
        out.entity_id = in.entity_id;
    }
}

void sample_terrain_cpu(
    const TerrainSampleRequest* requests,
    TerrainSampleResult* results,
    const float* heightmap,
    UInt32 width,
    UInt32 height,
    float origin_x,
    float origin_y,
    float scale,
    SizeT count)
{
    #ifdef _OPENMP
    #pragma omp parallel for
    #endif
    for (SizeT i = 0; i < count; ++i) {
        const TerrainSampleRequest& req = requests[i];
        TerrainSampleResult& res = results[i];

        // Convert to heightmap coordinates
        float u = (req.x - origin_x) / scale;
        float v = (req.y - origin_y) / scale;

        // Clamp to valid range
        u = std::max(0.0f, std::min(u, static_cast<float>(width - 1)));
        v = std::max(0.0f, std::min(v, static_cast<float>(height - 1)));

        // Bilinear interpolation
        int u0 = static_cast<int>(std::floor(u));
        int v0 = static_cast<int>(std::floor(v));
        int u1 = std::min(u0 + 1, static_cast<int>(width - 1));
        int v1 = std::min(v0 + 1, static_cast<int>(height - 1));

        float fu = u - u0;
        float fv = v - v0;

        float h00 = heightmap[v0 * width + u0];
        float h10 = heightmap[v0 * width + u1];
        float h01 = heightmap[v1 * width + u0];
        float h11 = heightmap[v1 * width + u1];

        res.height = (1-fu)*(1-fv)*h00 + fu*(1-fv)*h10 + (1-fu)*fv*h01 + fu*fv*h11;

        // Compute normal from gradient
        float dhdx = (h10 - h00) / scale;
        float dhdy = (h01 - h00) / scale;

        float nx = -dhdx;
        float ny = -dhdy;
        float nz = 1.0f;
        float len = std::sqrt(nx*nx + ny*ny + nz*nz);
        if (len > 0.0f) {
            res.normal_x = nx / len;
            res.normal_y = ny / len;
            res.normal_z = nz / len;
        } else {
            res.normal_x = 0.0f;
            res.normal_y = 0.0f;
            res.normal_z = 1.0f;
        }

        res.friction = 0.7f;
        res.material_id = 0;
        res.entity_id = req.entity_id;
    }
}

void apply_gravity_cpu(
    float* forces,
    const float* masses,
    float gravity_x,
    float gravity_y,
    float gravity_z,
    SizeT count)
{
    #ifdef _OPENMP
    #pragma omp parallel for
    #endif
    for (SizeT i = 0; i < count; ++i) {
        SizeT i3 = i * 3;
        float mass = masses[i];
        forces[i3 + 0] += mass * gravity_x;
        forces[i3 + 1] += mass * gravity_y;
        forces[i3 + 2] += mass * gravity_z;
    }
}

void apply_damping_cpu(
    float* velocities,
    float damping,
    float dt,
    SizeT count)
{
    float factor = 1.0f - damping * dt;
    #ifdef _OPENMP
    #pragma omp parallel for
    #endif
    for (SizeT i = 0; i < count; ++i) {
        SizeT i3 = i * 3;
        velocities[i3 + 0] *= factor;
        velocities[i3 + 1] *= factor;
        velocities[i3 + 2] *= factor;
    }
}

} // namespace cpu_kernels

// ============================================================================
// PhysicsKernelManager Implementation
// ============================================================================

PhysicsKernelManager::PhysicsKernelManager(IComputeBackend* backend)
    : m_backend(backend)
{
}

PhysicsKernelManager::~PhysicsKernelManager() {
    shutdown();
}

BackendResult PhysicsKernelManager::initialize(const PhysicsKernelConfig& config) {
    if (!m_backend || !m_backend->is_initialized()) {
        m_last_error = "Backend not initialized";
        return BackendResult::NotInitialized;
    }

    m_config = config;

    // Compile kernels for the backend
    auto result = compile_kernels();
    if (result != BackendResult::Success) {
        return result;
    }

    // Allocate spatial hash buffers for collision detection
    SizeT num_cells = 65536;  // Hash table size
    m_spatial_hash_cells = m_backend->allocate(
        num_cells * config.spatial_hash.max_entities_per_cell * sizeof(UInt32),
        MemoryType::DeviceLocal);
    m_spatial_hash_counts = m_backend->allocate(
        num_cells * sizeof(UInt32),
        MemoryType::DeviceLocal);

    m_initialized = true;
    return BackendResult::Success;
}

void PhysicsKernelManager::shutdown() {
    if (!m_initialized) return;

    // Free buffers
    if (m_terrain_heightmap.is_valid()) {
        m_backend->free(m_terrain_heightmap);
    }
    if (m_spatial_hash_cells.is_valid()) {
        m_backend->free(m_spatial_hash_cells);
    }
    if (m_spatial_hash_counts.is_valid()) {
        m_backend->free(m_spatial_hash_counts);
    }

    // Release kernels
    m_integrate_positions_kernel.reset();
    m_integrate_velocities_kernel.reset();
    m_integrate_orientations_kernel.reset();
    m_collision_broad_phase_kernel.reset();
    m_aero_forces_kernel.reset();
    m_terrain_sample_kernel.reset();

    m_initialized = false;
}

BackendResult PhysicsKernelManager::compile_kernels() {
    BackendType type = m_backend->type();

    // Select kernel source based on backend
    const char* pos_source = nullptr;
    const char* vel_source = nullptr;
    const char* orient_source = nullptr;
    const char* aabb_source = nullptr;

    switch (type) {
        case BackendType::CUDA:
            pos_source = kernel_source::cuda_integrate_positions;
            vel_source = kernel_source::cuda_integrate_velocities;
            orient_source = kernel_source::cuda_integrate_orientations;
            aabb_source = kernel_source::cuda_update_aabbs;
            break;

        case BackendType::OpenCL:
            pos_source = kernel_source::opencl_integrate_positions;
            vel_source = kernel_source::opencl_integrate_velocities;
            orient_source = kernel_source::opencl_integrate_orientations;
            aabb_source = kernel_source::opencl_update_aabbs;
            break;

        case BackendType::Metal:
            pos_source = kernel_source::metal_integrate_positions;
            vel_source = kernel_source::metal_integrate_velocities;
            orient_source = kernel_source::metal_integrate_orientations;
            aabb_source = kernel_source::metal_update_aabbs;
            break;

        case BackendType::CPU:
            // CPU backend uses direct function calls
            return BackendResult::Success;

        default:
            m_last_error = "Unsupported backend type";
            return BackendResult::NotSupported;
    }

    // Compile integration kernels
    if (pos_source) {
        m_integrate_positions_kernel = m_backend->create_kernel("integrate_positions", pos_source);
        if (!m_integrate_positions_kernel) {
            m_last_error = "Failed to compile integrate_positions kernel: " + m_backend->last_error();
            // Continue - kernel compilation failure shouldn't stop everything
        }
    }

    if (vel_source) {
        m_integrate_velocities_kernel = m_backend->create_kernel("integrate_velocities", vel_source);
        if (!m_integrate_velocities_kernel) {
            m_last_error = "Failed to compile integrate_velocities kernel: " + m_backend->last_error();
        }
    }

    if (orient_source) {
        m_integrate_orientations_kernel = m_backend->create_kernel("integrate_orientations", orient_source);
        if (!m_integrate_orientations_kernel) {
            m_last_error = "Failed to compile integrate_orientations kernel: " + m_backend->last_error();
        }
    }

    return BackendResult::Success;
}

// ============================================================================
// Buffer Allocation
// ============================================================================

BufferHandle PhysicsKernelManager::allocate_vec3_buffer(SizeT count) {
    return m_backend->allocate(count * 3 * sizeof(float), MemoryType::DeviceLocal);
}

BufferHandle PhysicsKernelManager::allocate_vec4_buffer(SizeT count) {
    return m_backend->allocate(count * 4 * sizeof(float), MemoryType::DeviceLocal);
}

BufferHandle PhysicsKernelManager::allocate_scalar_buffer(SizeT count) {
    return m_backend->allocate(count * sizeof(float), MemoryType::DeviceLocal);
}

BufferHandle PhysicsKernelManager::allocate_aabb_buffer(SizeT count) {
    return m_backend->allocate(count * sizeof(AABB), MemoryType::DeviceLocal);
}

BufferHandle PhysicsKernelManager::allocate_collision_pair_buffer(SizeT max_pairs) {
    return m_backend->allocate(max_pairs * sizeof(CollisionPair), MemoryType::DeviceLocal);
}

void PhysicsKernelManager::free_buffer(BufferHandle buffer) {
    if (buffer.is_valid()) {
        m_backend->free(buffer);
    }
}

// ============================================================================
// Integration Kernels
// ============================================================================

BackendResult PhysicsKernelManager::integrate_positions(
    BufferHandle positions,
    BufferHandle velocities,
    Real dt,
    SizeT count)
{
    if (!m_initialized) {
        m_last_error = "Kernel manager not initialized";
        return BackendResult::NotInitialized;
    }

    if (count == 0) return BackendResult::Success;

    if (m_backend->type() == BackendType::CPU || !m_integrate_positions_kernel) {
        // CPU fallback
        std::vector<float> pos_data(count * 3);
        std::vector<float> vel_data(count * 3);

        m_backend->download(positions, pos_data.data(), count * 3 * sizeof(float));
        m_backend->download(velocities, vel_data.data(), count * 3 * sizeof(float));

        cpu_kernels::integrate_positions_cpu(
            pos_data.data(), vel_data.data(), static_cast<float>(dt), count);

        m_backend->upload(positions, pos_data.data(), count * 3 * sizeof(float));
        return BackendResult::Success;
    }

    // GPU kernel execution
    m_integrate_positions_kernel->set_arg(0, KernelArg::Buffer(positions));
    m_integrate_positions_kernel->set_arg(1, KernelArg::Buffer(velocities));
    m_integrate_positions_kernel->set_arg(2, KernelArg::Float(static_cast<float>(dt)));
    m_integrate_positions_kernel->set_arg(3, KernelArg::Int(static_cast<Int32>(count)));

    auto config = LaunchConfig::Linear(count, m_config.preferred_workgroup_size);
    return m_backend->dispatch(m_integrate_positions_kernel.get(), config);
}

BackendResult PhysicsKernelManager::integrate_velocities(
    BufferHandle velocities,
    BufferHandle forces,
    BufferHandle masses,
    Real dt,
    SizeT count)
{
    if (!m_initialized) {
        return BackendResult::NotInitialized;
    }

    if (count == 0) return BackendResult::Success;

    if (m_backend->type() == BackendType::CPU || !m_integrate_velocities_kernel) {
        // CPU fallback
        std::vector<float> vel_data(count * 3);
        std::vector<float> force_data(count * 3);
        std::vector<float> mass_data(count);

        m_backend->download(velocities, vel_data.data(), count * 3 * sizeof(float));
        m_backend->download(forces, force_data.data(), count * 3 * sizeof(float));
        m_backend->download(masses, mass_data.data(), count * sizeof(float));

        cpu_kernels::integrate_velocities_cpu(
            vel_data.data(), force_data.data(), mass_data.data(),
            static_cast<float>(m_config.gravity_x),
            static_cast<float>(m_config.gravity_y),
            static_cast<float>(m_config.gravity_z),
            static_cast<float>(dt), count);

        m_backend->upload(velocities, vel_data.data(), count * 3 * sizeof(float));
        return BackendResult::Success;
    }

    // GPU kernel execution
    m_integrate_velocities_kernel->set_arg(0, KernelArg::Buffer(velocities));
    m_integrate_velocities_kernel->set_arg(1, KernelArg::Buffer(forces));
    m_integrate_velocities_kernel->set_arg(2, KernelArg::Buffer(masses));
    m_integrate_velocities_kernel->set_arg(3, KernelArg::Float(static_cast<float>(m_config.gravity_x)));
    m_integrate_velocities_kernel->set_arg(4, KernelArg::Float(static_cast<float>(m_config.gravity_y)));
    m_integrate_velocities_kernel->set_arg(5, KernelArg::Float(static_cast<float>(m_config.gravity_z)));
    m_integrate_velocities_kernel->set_arg(6, KernelArg::Float(static_cast<float>(dt)));
    m_integrate_velocities_kernel->set_arg(7, KernelArg::Int(static_cast<Int32>(count)));

    auto config = LaunchConfig::Linear(count, m_config.preferred_workgroup_size);
    return m_backend->dispatch(m_integrate_velocities_kernel.get(), config);
}

BackendResult PhysicsKernelManager::integrate_orientations(
    BufferHandle orientations,
    BufferHandle angular_velocities,
    Real dt,
    SizeT count)
{
    if (!m_initialized) {
        return BackendResult::NotInitialized;
    }

    if (count == 0) return BackendResult::Success;

    if (m_backend->type() == BackendType::CPU || !m_integrate_orientations_kernel) {
        // CPU fallback
        std::vector<float> orient_data(count * 4);
        std::vector<float> angvel_data(count * 3);

        m_backend->download(orientations, orient_data.data(), count * 4 * sizeof(float));
        m_backend->download(angular_velocities, angvel_data.data(), count * 3 * sizeof(float));

        cpu_kernels::integrate_orientations_cpu(
            orient_data.data(), angvel_data.data(), static_cast<float>(dt), count);

        m_backend->upload(orientations, orient_data.data(), count * 4 * sizeof(float));
        return BackendResult::Success;
    }

    // GPU kernel execution
    m_integrate_orientations_kernel->set_arg(0, KernelArg::Buffer(orientations));
    m_integrate_orientations_kernel->set_arg(1, KernelArg::Buffer(angular_velocities));
    m_integrate_orientations_kernel->set_arg(2, KernelArg::Float(static_cast<float>(dt)));
    m_integrate_orientations_kernel->set_arg(3, KernelArg::Int(static_cast<Int32>(count)));

    auto config = LaunchConfig::Linear(count, m_config.preferred_workgroup_size);
    return m_backend->dispatch(m_integrate_orientations_kernel.get(), config);
}

BackendResult PhysicsKernelManager::integrate_symplectic(
    RigidBodyStateBuffers& state,
    Real dt)
{
    if (!m_initialized) {
        return BackendResult::NotInitialized;
    }

    // Symplectic Euler: update velocities first, then positions
    auto result = integrate_velocities(state.velocities, state.forces, state.masses, dt, state.count);
    if (result != BackendResult::Success) return result;

    result = integrate_positions(state.positions, state.velocities, dt, state.count);
    if (result != BackendResult::Success) return result;

    result = integrate_orientations(state.orientations, state.angular_vels, dt, state.count);
    if (result != BackendResult::Success) return result;

    // Clear forces for next frame
    return clear_forces(state.forces, state.torques, state.count);
}

BackendResult PhysicsKernelManager::clear_forces(
    BufferHandle forces,
    BufferHandle torques,
    SizeT count)
{
    if (!m_initialized) {
        return BackendResult::NotInitialized;
    }

    if (count == 0) return BackendResult::Success;

    // Zero-fill both buffers
    float zero = 0.0f;
    auto result = m_backend->fill(forces, &zero, sizeof(float), count * 3 * sizeof(float));
    if (result != BackendResult::Success) return result;

    return m_backend->fill(torques, &zero, sizeof(float), count * 3 * sizeof(float));
}

// ============================================================================
// Collision Detection Kernels
// ============================================================================

BackendResult PhysicsKernelManager::update_aabbs(
    BufferHandle positions,
    BufferHandle extents,
    BufferHandle aabbs,
    SizeT count)
{
    if (!m_initialized) {
        return BackendResult::NotInitialized;
    }

    if (count == 0) return BackendResult::Success;

    // CPU fallback for now (GPU kernel can be added later)
    std::vector<float> pos_data(count * 3);
    std::vector<float> ext_data(count * 3);
    std::vector<AABB> aabb_data(count);

    m_backend->download(positions, pos_data.data(), count * 3 * sizeof(float));
    m_backend->download(extents, ext_data.data(), count * 3 * sizeof(float));

    cpu_kernels::update_aabbs_cpu(pos_data.data(), ext_data.data(), aabb_data.data(), count);

    m_backend->upload(aabbs, aabb_data.data(), count * sizeof(AABB));
    return BackendResult::Success;
}

BackendResult PhysicsKernelManager::collision_broad_phase(
    BufferHandle aabbs,
    SizeT count,
    BufferHandle pairs,
    SizeT* pair_count)
{
    if (!m_initialized) {
        return BackendResult::NotInitialized;
    }

    if (count == 0) {
        *pair_count = 0;
        return BackendResult::Success;
    }

    // CPU fallback
    std::vector<AABB> aabb_data(count);
    std::vector<CollisionPair> pair_data(m_config.max_collision_pairs);

    m_backend->download(aabbs, aabb_data.data(), count * sizeof(AABB));

    *pair_count = cpu_kernels::collision_broad_phase_cpu(
        aabb_data.data(), count,
        pair_data.data(), m_config.max_collision_pairs,
        m_config.spatial_hash);

    m_backend->upload(pairs, pair_data.data(), (*pair_count) * sizeof(CollisionPair));
    return BackendResult::Success;
}

BackendResult PhysicsKernelManager::sort_aabbs(
    BufferHandle aabbs,
    SizeT count,
    UInt32 axis)
{
    if (!m_initialized) {
        return BackendResult::NotInitialized;
    }

    if (count == 0) return BackendResult::Success;

    // CPU fallback - sort by specified axis
    std::vector<AABB> aabb_data(count);
    m_backend->download(aabbs, aabb_data.data(), count * sizeof(AABB));

    std::sort(aabb_data.begin(), aabb_data.end(), [axis](const AABB& a, const AABB& b) {
        switch (axis) {
            case 0: return a.min_x < b.min_x;
            case 1: return a.min_y < b.min_y;
            case 2: return a.min_z < b.min_z;
            default: return a.min_x < b.min_x;
        }
    });

    m_backend->upload(aabbs, aabb_data.data(), count * sizeof(AABB));
    return BackendResult::Success;
}

// ============================================================================
// Terrain Sampling Kernels
// ============================================================================

BackendResult PhysicsKernelManager::set_terrain_heightmap(
    BufferHandle heightmap,
    UInt32 width,
    UInt32 height,
    Real origin_x,
    Real origin_y,
    Real scale)
{
    m_terrain_heightmap = heightmap;
    m_terrain_width = width;
    m_terrain_height = height;
    m_terrain_origin_x = origin_x;
    m_terrain_origin_y = origin_y;
    m_terrain_scale = scale;

    return BackendResult::Success;
}

BackendResult PhysicsKernelManager::sample_terrain_batch(
    BufferHandle requests,
    BufferHandle results,
    SizeT count)
{
    if (!m_initialized) {
        return BackendResult::NotInitialized;
    }

    if (count == 0) return BackendResult::Success;

    if (!m_terrain_heightmap.is_valid()) {
        m_last_error = "No terrain heightmap set";
        return BackendResult::InvalidArgument;
    }

    // CPU fallback
    std::vector<TerrainSampleRequest> req_data(count);
    std::vector<TerrainSampleResult> res_data(count);
    std::vector<float> heightmap_data(m_terrain_width * m_terrain_height);

    m_backend->download(requests, req_data.data(), count * sizeof(TerrainSampleRequest));
    m_backend->download(m_terrain_heightmap, heightmap_data.data(),
                        m_terrain_width * m_terrain_height * sizeof(float));

    cpu_kernels::sample_terrain_cpu(
        req_data.data(), res_data.data(),
        heightmap_data.data(),
        m_terrain_width, m_terrain_height,
        static_cast<float>(m_terrain_origin_x),
        static_cast<float>(m_terrain_origin_y),
        static_cast<float>(m_terrain_scale),
        count);

    m_backend->upload(results, res_data.data(), count * sizeof(TerrainSampleResult));
    return BackendResult::Success;
}

// ============================================================================
// Aerodynamic Force Kernels
// ============================================================================

BackendResult PhysicsKernelManager::compute_aero_forces(
    BufferHandle inputs,
    BufferHandle outputs,
    SizeT count)
{
    if (!m_initialized) {
        return BackendResult::NotInitialized;
    }

    if (count == 0) return BackendResult::Success;

    // CPU fallback
    std::vector<AeroInput> in_data(count);
    std::vector<AeroOutput> out_data(count);

    m_backend->download(inputs, in_data.data(), count * sizeof(AeroInput));

    cpu_kernels::compute_aero_forces_cpu(in_data.data(), out_data.data(), count);

    m_backend->upload(outputs, out_data.data(), count * sizeof(AeroOutput));
    return BackendResult::Success;
}

BackendResult PhysicsKernelManager::set_aero_model(
    UInt32 model_id,
    const std::vector<Real>& alpha_table,
    const std::vector<Real>& cl_table,
    const std::vector<Real>& cd_table,
    const std::vector<Real>& cm_table)
{
    // Store aerodynamic model data
    // For full implementation, this would upload lookup tables to GPU
    // For now, the simplified aero model is hard-coded in the kernel
    return BackendResult::Success;
}

// ============================================================================
// Utility Kernels
// ============================================================================

BackendResult PhysicsKernelManager::apply_gravity(
    BufferHandle forces,
    BufferHandle masses,
    SizeT count)
{
    if (!m_initialized) {
        return BackendResult::NotInitialized;
    }

    if (count == 0) return BackendResult::Success;

    // CPU fallback
    std::vector<float> force_data(count * 3);
    std::vector<float> mass_data(count);

    m_backend->download(forces, force_data.data(), count * 3 * sizeof(float));
    m_backend->download(masses, mass_data.data(), count * sizeof(float));

    cpu_kernels::apply_gravity_cpu(
        force_data.data(), mass_data.data(),
        static_cast<float>(m_config.gravity_x),
        static_cast<float>(m_config.gravity_y),
        static_cast<float>(m_config.gravity_z),
        count);

    m_backend->upload(forces, force_data.data(), count * 3 * sizeof(float));
    return BackendResult::Success;
}

BackendResult PhysicsKernelManager::apply_damping(
    BufferHandle velocities,
    Real damping,
    Real dt,
    SizeT count)
{
    if (!m_initialized) {
        return BackendResult::NotInitialized;
    }

    if (count == 0) return BackendResult::Success;

    // CPU fallback
    std::vector<float> vel_data(count * 3);

    m_backend->download(velocities, vel_data.data(), count * 3 * sizeof(float));

    cpu_kernels::apply_damping_cpu(
        vel_data.data(),
        static_cast<float>(damping),
        static_cast<float>(dt),
        count);

    m_backend->upload(velocities, vel_data.data(), count * 3 * sizeof(float));
    return BackendResult::Success;
}

BackendResult PhysicsKernelManager::clamp_velocities(
    BufferHandle velocities,
    Real max_speed,
    SizeT count)
{
    if (!m_initialized) {
        return BackendResult::NotInitialized;
    }

    if (count == 0) return BackendResult::Success;

    float max_speed_sq = static_cast<float>(max_speed * max_speed);

    // CPU fallback
    std::vector<float> vel_data(count * 3);
    m_backend->download(velocities, vel_data.data(), count * 3 * sizeof(float));

    #ifdef _OPENMP
    #pragma omp parallel for
    #endif
    for (SizeT i = 0; i < count; ++i) {
        SizeT i3 = i * 3;
        float vx = vel_data[i3 + 0];
        float vy = vel_data[i3 + 1];
        float vz = vel_data[i3 + 2];

        float speed_sq = vx*vx + vy*vy + vz*vz;
        if (speed_sq > max_speed_sq && speed_sq > 0.0f) {
            float scale = static_cast<float>(max_speed) / std::sqrt(speed_sq);
            vel_data[i3 + 0] = vx * scale;
            vel_data[i3 + 1] = vy * scale;
            vel_data[i3 + 2] = vz * scale;
        }
    }

    m_backend->upload(velocities, vel_data.data(), count * 3 * sizeof(float));
    return BackendResult::Success;
}

// ============================================================================
// Wave Spectrum Kernels
// ============================================================================

BackendResult PhysicsKernelManager::compute_wave_spectrum(
    BufferHandle spectrum_buffer,
    UInt32 resolution,
    Real wind_speed,
    Real wind_direction,
    Real time)
{
    if (!m_initialized) {
        return BackendResult::NotInitialized;
    }

    if (resolution == 0 || (resolution & (resolution - 1)) != 0) {
        m_last_error = "Resolution must be a power of 2";
        return BackendResult::InvalidArgument;
    }

    SizeT spectrum_size = static_cast<SizeT>(resolution) * resolution;

    // CPU fallback - compute wave spectrum
    std::vector<float> spectrum_real(spectrum_size);
    std::vector<float> spectrum_imag(spectrum_size);

    cpu_kernels::compute_wave_spectrum_cpu(
        spectrum_real.data(), spectrum_imag.data(),
        resolution,
        static_cast<float>(wind_speed),
        static_cast<float>(wind_direction),
        static_cast<float>(time));

    // Pack as interleaved complex: [real0, imag0, real1, imag1, ...]
    std::vector<float> spectrum_packed(spectrum_size * 2);
    for (SizeT i = 0; i < spectrum_size; ++i) {
        spectrum_packed[i * 2 + 0] = spectrum_real[i];
        spectrum_packed[i * 2 + 1] = spectrum_imag[i];
    }

    m_backend->upload(spectrum_buffer, spectrum_packed.data(), spectrum_size * 2 * sizeof(float));
    return BackendResult::Success;
}

BackendResult PhysicsKernelManager::sample_wave_heights(
    BufferHandle positions,
    BufferHandle heights,
    BufferHandle normals,
    BufferHandle spectrum_buffer,
    UInt32 resolution,
    Real patch_size,
    SizeT count)
{
    if (!m_initialized) {
        return BackendResult::NotInitialized;
    }

    if (count == 0) return BackendResult::Success;

    SizeT spectrum_size = static_cast<SizeT>(resolution) * resolution;

    // Download data
    std::vector<float> pos_data(count * 2);
    std::vector<float> spectrum_packed(spectrum_size * 2);

    m_backend->download(positions, pos_data.data(), count * 2 * sizeof(float));
    m_backend->download(spectrum_buffer, spectrum_packed.data(), spectrum_size * 2 * sizeof(float));

    // Unpack spectrum
    std::vector<float> spectrum_real(spectrum_size);
    std::vector<float> spectrum_imag(spectrum_size);
    for (SizeT i = 0; i < spectrum_size; ++i) {
        spectrum_real[i] = spectrum_packed[i * 2 + 0];
        spectrum_imag[i] = spectrum_packed[i * 2 + 1];
    }

    // CPU fallback
    std::vector<float> height_data(count);
    std::vector<float> normal_data(normals.is_valid() ? count * 3 : 0);

    cpu_kernels::sample_wave_heights_cpu(
        pos_data.data(), height_data.data(),
        normals.is_valid() ? normal_data.data() : nullptr,
        spectrum_real.data(), spectrum_imag.data(),
        resolution,
        static_cast<float>(patch_size),
        count);

    m_backend->upload(heights, height_data.data(), count * sizeof(float));
    if (normals.is_valid()) {
        m_backend->upload(normals, normal_data.data(), count * 3 * sizeof(float));
    }

    return BackendResult::Success;
}

// ============================================================================
// Atmosphere Density Kernels
// ============================================================================

BackendResult PhysicsKernelManager::compute_atmosphere_density(
    BufferHandle altitudes,
    BufferHandle densities,
    BufferHandle temperatures,
    BufferHandle pressures,
    SizeT count)
{
    if (!m_initialized) {
        return BackendResult::NotInitialized;
    }

    if (count == 0) return BackendResult::Success;

    // Download altitudes
    std::vector<float> alt_data(count);
    m_backend->download(altitudes, alt_data.data(), count * sizeof(float));

    // CPU fallback
    std::vector<float> density_data(count);
    std::vector<float> temp_data(temperatures.is_valid() ? count : 0);
    std::vector<float> pres_data(pressures.is_valid() ? count : 0);

    cpu_kernels::compute_atmosphere_density_cpu(
        alt_data.data(), density_data.data(),
        temperatures.is_valid() ? temp_data.data() : nullptr,
        pressures.is_valid() ? pres_data.data() : nullptr,
        count);

    m_backend->upload(densities, density_data.data(), count * sizeof(float));
    if (temperatures.is_valid()) {
        m_backend->upload(temperatures, temp_data.data(), count * sizeof(float));
    }
    if (pressures.is_valid()) {
        m_backend->upload(pressures, pres_data.data(), count * sizeof(float));
    }

    return BackendResult::Success;
}

BackendResult PhysicsKernelManager::compute_atmosphere_batch(
    BufferHandle positions,
    BufferHandle air_density,
    BufferHandle speed_of_sound,
    BufferHandle temperature,
    SizeT count)
{
    if (!m_initialized) {
        return BackendResult::NotInitialized;
    }

    if (count == 0) return BackendResult::Success;

    // Download positions
    std::vector<float> pos_data(count * 3);
    m_backend->download(positions, pos_data.data(), count * 3 * sizeof(float));

    // CPU fallback
    std::vector<float> density_data(count);
    std::vector<float> sos_data(count);
    std::vector<float> temp_data(count);

    cpu_kernels::compute_atmosphere_batch_cpu(
        pos_data.data(),
        density_data.data(),
        sos_data.data(),
        temp_data.data(),
        count);

    m_backend->upload(air_density, density_data.data(), count * sizeof(float));
    m_backend->upload(speed_of_sound, sos_data.data(), count * sizeof(float));
    m_backend->upload(temperature, temp_data.data(), count * sizeof(float));

    return BackendResult::Success;
}

// ============================================================================
// Synchronization
// ============================================================================

BackendResult PhysicsKernelManager::synchronize() {
    if (!m_initialized) {
        return BackendResult::NotInitialized;
    }
    return m_backend->synchronize();
}

// ============================================================================
// CPU Kernel Implementations - Wave Spectrum
// ============================================================================

namespace cpu_kernels {

void compute_wave_spectrum_cpu(
    float* spectrum_real,
    float* spectrum_imag,
    UInt32 resolution,
    float wind_speed,
    float wind_direction,
    float time)
{
    // Phillips spectrum implementation
    const float g = 9.81f;
    const float A = 0.0005f;  // Phillips constant
    const float L = wind_speed * wind_speed / g;  // Largest wave arising from continuous wind

    float wind_x = std::cos(wind_direction);
    float wind_z = std::sin(wind_direction);

    int N = static_cast<int>(resolution);

    #ifdef _OPENMP
    #pragma omp parallel for collapse(2)
    #endif
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            // Compute wave vector k (avoid integer division loss of precision)
            float kx = (2.0f * 3.14159265359f * static_cast<float>(i - N / 2)) / static_cast<float>(N);
            float kz = (2.0f * 3.14159265359f * static_cast<float>(j - N / 2)) / static_cast<float>(N);
            float k_len = std::sqrt(kx * kx + kz * kz);

            int idx = i * N + j;

            if (k_len < 1e-6f) {
                spectrum_real[idx] = 0.0f;
                spectrum_imag[idx] = 0.0f;
                continue;
            }

            // Phillips spectrum
            float k_len_sq = k_len * k_len;
            float L_sq = L * L;

            // Damping factor for small wavelengths
            float l = L / 1000.0f;
            float damping = std::exp(-k_len_sq * l * l);

            // Directional factor
            float k_dot_w = (kx * wind_x + kz * wind_z) / k_len;
            float directional = k_dot_w * k_dot_w;

            // Phillips spectrum value
            float phillips = A * std::exp(-1.0f / (k_len_sq * L_sq)) / (k_len_sq * k_len_sq);
            phillips *= directional * damping;
            float h0 = std::sqrt(phillips * 0.5f);

            // Time-dependent phase
            float omega = std::sqrt(g * k_len);  // Deep water dispersion relation
            float phase = omega * time;

            // Combine with random phase (simplified - using deterministic phase based on k)
            float seed_phase = static_cast<float>(i * 1000 + j) * 0.0001f;
            float rand_phase = std::fmod(seed_phase * 6.28318f, 6.28318f);
            float total_phase = rand_phase + phase;

            spectrum_real[idx] = h0 * std::cos(total_phase);
            spectrum_imag[idx] = h0 * std::sin(total_phase);
        }
    }
}

void sample_wave_heights_cpu(
    const float* positions,
    float* heights,
    float* normals,
    const float* spectrum_real,
    const float* spectrum_imag,
    UInt32 resolution,
    float patch_size,
    SizeT count)
{
    int N = static_cast<int>(resolution);
    float inv_patch = 1.0f / patch_size;

    #ifdef _OPENMP
    #pragma omp parallel for
    #endif
    for (SizeT i = 0; i < count; ++i) {
        float px = positions[i * 2 + 0];
        float pz = positions[i * 2 + 1];

        // Compute height by summing wave contributions (simplified DFT evaluation)
        // For better performance, use FFT, but this CPU fallback uses direct summation
        // for small numbers of samples
        float height = 0.0f;
        float dx = 0.0f, dz = 0.0f;  // For normal computation

        // Sample a subset of waves for performance
        const int sample_step = std::max(1, N / 32);

        for (int m = 0; m < N; m += sample_step) {
            for (int n = 0; n < N; n += sample_step) {
                float kx = (2.0f * 3.14159265359f * static_cast<float>(m - N / 2)) * inv_patch;
                float kz = (2.0f * 3.14159265359f * static_cast<float>(n - N / 2)) * inv_patch;

                int idx = m * N + n;
                float h_real = spectrum_real[idx];
                float h_imag = spectrum_imag[idx];

                // e^(i * k . x) = cos(k.x) + i*sin(k.x)
                float k_dot_x = kx * px + kz * pz;
                float cos_kx = std::cos(k_dot_x);
                float sin_kx = std::sin(k_dot_x);

                // Height contribution: Re(h * e^(i*k.x))
                height += h_real * cos_kx - h_imag * sin_kx;

                // Gradient for normal computation
                dx += -kx * (h_real * sin_kx + h_imag * cos_kx);
                dz += -kz * (h_real * sin_kx + h_imag * cos_kx);
            }
        }

        // Scale by sample step
        float scale = static_cast<float>(sample_step * sample_step);
        heights[i] = height * scale;

        if (normals != nullptr) {
            // Normal = normalize(-dh/dx, 1, -dh/dz)
            dx *= scale;
            dz *= scale;
            float len = std::sqrt(dx * dx + 1.0f + dz * dz);
            normals[i * 3 + 0] = -dx / len;
            normals[i * 3 + 1] = 1.0f / len;
            normals[i * 3 + 2] = -dz / len;
        }
    }
}

// ============================================================================
// CPU Kernel Implementations - Atmosphere
// ============================================================================

void compute_atmosphere_density_cpu(
    const float* altitudes,
    float* densities,
    float* temperatures,
    float* pressures,
    SizeT count)
{
    // ISA (International Standard Atmosphere) model
    // Constants for troposphere (0-11km)
    const float T0 = 288.15f;      // Sea level temperature (K)
    const float P0 = 101325.0f;    // Sea level pressure (Pa)
    const float rho0 = 1.225f;     // Sea level density (kg/m³)
    const float L = 0.0065f;       // Temperature lapse rate (K/m)
    const float g = 9.80665f;      // Gravity (m/s²)
    const float M = 0.0289644f;    // Molar mass of air (kg/mol)
    const float R = 8.31447f;      // Gas constant (J/(mol·K))

    // Troposphere/stratosphere boundary
    const float h_tropo = 11000.0f;
    const float T_tropo = T0 - L * h_tropo;  // Temperature at 11km

    #ifdef _OPENMP
    #pragma omp parallel for
    #endif
    for (SizeT i = 0; i < count; ++i) {
        float h = altitudes[i];

        float T, P, rho;

        if (h < 0.0f) {
            // Below sea level - use sea level values
            T = T0;
            P = P0;
            rho = rho0;
        }
        else if (h <= h_tropo) {
            // Troposphere (0-11km): temperature decreases linearly
            T = T0 - L * h;
            float temp_ratio = T / T0;
            float exponent = g * M / (R * L);
            P = P0 * std::pow(temp_ratio, exponent);
            rho = rho0 * std::pow(temp_ratio, exponent - 1.0f);
        }
        else if (h <= 20000.0f) {
            // Lower stratosphere (11-20km): temperature constant
            T = T_tropo;

            // Pressure at tropopause
            float temp_ratio_tropo = T_tropo / T0;
            float exponent = g * M / (R * L);
            float P_tropo = P0 * std::pow(temp_ratio_tropo, exponent);

            // Pressure in isothermal layer
            float delta_h = h - h_tropo;
            P = P_tropo * std::exp(-g * M * delta_h / (R * T_tropo));
            rho = P * M / (R * T);
        }
        else if (h <= 32000.0f) {
            // Upper stratosphere (20-32km): temperature increases
            const float L2 = -0.001f;  // Negative lapse rate (warming)
            const float T_20 = T_tropo;
            T = T_20 - L2 * (h - 20000.0f);

            // Simplified pressure/density calculation
            float scale_height = R * T / (g * M);
            P = P0 * std::exp(-h / scale_height) * 0.1f;  // Approximation
            rho = P * M / (R * T);
        }
        else {
            // Above 32km - use exponential approximation
            T = 228.65f;  // Approximate temperature
            float scale_height = 6500.0f;
            P = P0 * std::exp(-h / scale_height);
            rho = P * M / (R * T);
        }

        densities[i] = rho;
        if (temperatures != nullptr) {
            temperatures[i] = T;
        }
        if (pressures != nullptr) {
            pressures[i] = P;
        }
    }
}

void compute_atmosphere_batch_cpu(
    const float* positions,
    float* air_density,
    float* speed_of_sound,
    float* temperature,
    SizeT count)
{
    // ISA constants
    const float T0 = 288.15f;
    const float rho0 = 1.225f;
    const float L = 0.0065f;
    const float g = 9.80665f;
    const float M = 0.0289644f;
    const float R = 8.31447f;
    const float gamma = 1.4f;  // Ratio of specific heats for air
    const float R_specific = 287.05f;  // Specific gas constant for air (J/(kg·K))

    const float h_tropo = 11000.0f;
    const float T_tropo = T0 - L * h_tropo;

    #ifdef _OPENMP
    #pragma omp parallel for
    #endif
    for (SizeT i = 0; i < count; ++i) {
        // Altitude is the Z component (assuming Z-up coordinate system)
        float h = positions[i * 3 + 2];

        float T, rho;

        if (h < 0.0f) {
            T = T0;
            rho = rho0;
        }
        else if (h <= h_tropo) {
            T = T0 - L * h;
            float temp_ratio = T / T0;
            float exponent = g * M / (R * L);
            rho = rho0 * std::pow(temp_ratio, exponent - 1.0f);
        }
        else if (h <= 20000.0f) {
            T = T_tropo;
            float temp_ratio_tropo = T_tropo / T0;
            float exponent = g * M / (R * L);
            float rho_tropo = rho0 * std::pow(temp_ratio_tropo, exponent - 1.0f);
            float delta_h = h - h_tropo;
            rho = rho_tropo * std::exp(-g * M * delta_h / (R * T_tropo));
        }
        else {
            // High altitude approximation
            T = 216.65f;
            float scale_height = 6500.0f;
            rho = rho0 * std::exp(-h / scale_height) * 0.05f;
        }

        air_density[i] = rho;
        temperature[i] = T;

        // Speed of sound: a = sqrt(gamma * R_specific * T)
        speed_of_sound[i] = std::sqrt(gamma * R_specific * T);
    }
}

} // namespace cpu_kernels

} // namespace jaguar::gpu
