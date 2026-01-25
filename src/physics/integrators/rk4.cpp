/**
 * @file rk4.cpp
 * @brief RK4 integrator implementation
 *
 * Implements fourth-order Runge-Kutta integration for rigid body dynamics
 * with full inertia tensor support and Euler's equations of motion.
 */

#include "jaguar/physics/solver.h"
#include "jaguar/core/constants.h"
#include <cmath>
#include <algorithm>

namespace jaguar::physics {

RK4Integrator::RK4Integrator() = default;

namespace {

/**
 * @brief Compute angular acceleration using Euler's equations
 *
 * Euler's equations for a rigid body:
 * I * ω̇ = τ - ω × (I * ω)
 *
 * Where:
 * - I is the inertia tensor
 * - ω is the angular velocity
 * - τ is the applied torque
 * - ω̇ is the angular acceleration
 */
Vec3 compute_angular_accel(const Mat3x3& inertia, const Mat3x3& inv_inertia,
                           const Vec3& omega, const Vec3& torque) {
    // Compute I * ω
    Vec3 I_omega = inertia * omega;

    // Compute ω × (I * ω) - gyroscopic term
    Vec3 gyroscopic = omega.cross(I_omega);

    // ω̇ = I⁻¹ * (τ - ω × (I * ω))
    return inv_inertia * (torque - gyroscopic);
}

/**
 * @brief Compute inverse of inertia tensor
 */
Mat3x3 compute_inverse_inertia(const Mat3x3& inertia) {
    Real det = inertia.determinant();
    if (std::abs(det) < 1e-10) {
        // Fallback to diagonal inverse for near-singular matrices
        Mat3x3 inv = Mat3x3::Identity();
        Real Ixx = inertia(0, 0);
        Real Iyy = inertia(1, 1);
        Real Izz = inertia(2, 2);
        if (Ixx > 1e-10) inv(0, 0) = 1.0 / Ixx;
        if (Iyy > 1e-10) inv(1, 1) = 1.0 / Iyy;
        if (Izz > 1e-10) inv(2, 2) = 1.0 / Izz;
        return inv;
    }
    return inertia.inverse();
}

} // anonymous namespace

void RK4Integrator::integrate(EntityState& state, const EntityForces& forces, Real dt) {
    // Skip integration for massless entities
    if (state.mass < constants::MASS_EPSILON) {
        return;
    }

    // Clamp time step to safe bounds
    dt = std::clamp(dt, constants::TIME_STEP_MIN, constants::TIME_STEP_MAX);

    // Compute inverse mass and inertia
    Real inv_mass = 1.0 / state.mass;
    Mat3x3 inv_inertia = compute_inverse_inertia(state.inertia);

    // Forces are in body frame, need to transform to ECEF for integration
    // orientation is body-to-ECEF, so we rotate force from body to ECEF
    Vec3 force_ecef = state.orientation.rotate(forces.force);

    // Linear acceleration in ECEF: a = F/m
    Vec3 accel = force_ecef * inv_mass;

    // ========================================================================
    // Translational Motion (RK4)
    // ========================================================================

    // k1: derivatives at t
    Vec3 k1_p = state.velocity;
    Vec3 k1_v = accel;

    // k2: derivatives at t + dt/2 using k1
    Vec3 k2_p = state.velocity + k1_v * (dt * 0.5);
    Vec3 k2_v = accel;  // Constant force assumption

    // k3: derivatives at t + dt/2 using k2
    Vec3 k3_p = state.velocity + k2_v * (dt * 0.5);
    Vec3 k3_v = accel;

    // k4: derivatives at t + dt using k3
    Vec3 k4_p = state.velocity + k3_v * dt;
    Vec3 k4_v = accel;

    // Weighted average (RK4 formula)
    state.position = state.position +
        (k1_p + k2_p * 2.0 + k3_p * 2.0 + k4_p) * (dt / 6.0);
    state.velocity = state.velocity +
        (k1_v + k2_v * 2.0 + k3_v * 2.0 + k4_v) * (dt / 6.0);

    // ========================================================================
    // Rotational Motion (RK4 with Euler's Equations)
    // ========================================================================

    // k1: angular acceleration at current state
    Vec3 omega1 = state.angular_velocity;
    Vec3 k1_omega = compute_angular_accel(state.inertia, inv_inertia, omega1, forces.torque);

    // k2: angular acceleration at t + dt/2
    Vec3 omega2 = state.angular_velocity + k1_omega * (dt * 0.5);
    Vec3 k2_omega = compute_angular_accel(state.inertia, inv_inertia, omega2, forces.torque);

    // k3: angular acceleration at t + dt/2 (using k2)
    Vec3 omega3 = state.angular_velocity + k2_omega * (dt * 0.5);
    Vec3 k3_omega = compute_angular_accel(state.inertia, inv_inertia, omega3, forces.torque);

    // k4: angular acceleration at t + dt
    Vec3 omega4 = state.angular_velocity + k3_omega * dt;
    Vec3 k4_omega = compute_angular_accel(state.inertia, inv_inertia, omega4, forces.torque);

    // Update angular velocity
    state.angular_velocity = state.angular_velocity +
        (k1_omega + k2_omega * 2.0 + k3_omega * 2.0 + k4_omega) * (dt / 6.0);

    // ========================================================================
    // Quaternion Integration (RK4)
    // ========================================================================

    // Use the RK4 quaternion integration from quaternion_utils
    state.orientation = quaternion_utils::integrate_rk4(
        state.orientation, state.angular_velocity, dt);

    // ========================================================================
    // Store computed accelerations for debugging/output
    // ========================================================================

    state.acceleration = accel;
    state.angular_accel = k1_omega;  // Use initial angular acceleration
}

} // namespace jaguar::physics
