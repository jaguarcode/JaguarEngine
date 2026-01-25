/**
 * @file abm4.cpp
 * @brief Adams-Bashforth-Moulton integrator implementation (stub)
 */

#include "jaguar/physics/solver.h"
#include "jaguar/core/constants.h"
#include <cmath>
#include <algorithm>

namespace jaguar::physics {

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

ABM4Integrator::ABM4Integrator() {
    reset();
}

void ABM4Integrator::integrate(EntityState& state, const EntityForces& forces, Real dt) {
    // Skip integration for massless entities
    if (state.mass < constants::MASS_EPSILON) {
        return;
    }

    // Clamp time step to safe bounds
    dt = std::clamp(dt, constants::TIME_STEP_MIN, constants::TIME_STEP_MAX);

    // Transform forces from body frame to ECEF
    // orientation is body-to-ECEF, so we rotate force from body to ECEF
    Vec3 force_ecef = state.orientation.rotate(forces.force);

    // Compute inverse inertia tensor
    Mat3x3 inv_inertia = compute_inverse_inertia(state.inertia);

    // For first 4 steps, use RK4 to build history
    if (step_count_ < 4) {
        // Store acceleration history (in ECEF)
        Vec3 accel = force_ecef * (1.0 / state.mass);
        accel_history_[step_count_] = accel;

        // Store angular acceleration history (in body frame)
        Vec3 angular_accel = compute_angular_accel(
            state.inertia, inv_inertia, state.angular_velocity, forces.torque);
        angular_accel_history_[step_count_] = angular_accel;

        // Use RK4 for startup
        RK4Integrator rk4;
        rk4.integrate(state, forces, dt);

        step_count_++;
        return;
    }

    // Full ABM4 integration
    Vec3 accel = force_ecef * (1.0 / state.mass);

    // Compute current angular acceleration
    Vec3 angular_accel = compute_angular_accel(
        state.inertia, inv_inertia, state.angular_velocity, forces.torque);

    // ========================================================================
    // Linear Motion (ABM4)
    // ========================================================================

    // Adams-Bashforth predictor (4th order)
    Vec3 v_pred = state.velocity + (
        accel_history_[3] * 55.0 -
        accel_history_[2] * 59.0 +
        accel_history_[1] * 37.0 -
        accel_history_[0] * 9.0
    ) * (dt / 24.0);

    // Adams-Moulton corrector (4th order) for velocity
    state.velocity = state.velocity + (
        accel * 9.0 +
        accel_history_[3] * 19.0 -
        accel_history_[2] * 5.0 +
        accel_history_[1] * 1.0
    ) * (dt / 24.0);

    // Position update using trapezoidal rule with predicted and corrected velocities
    // This is a simplified but stable approach since we don't store velocity history
    state.position = state.position + (v_pred + state.velocity) * (dt * 0.5);

    // ========================================================================
    // Angular Motion (ABM4)
    // ========================================================================

    // Adams-Bashforth predictor (4th order) for angular velocity
    Vec3 omega_pred = state.angular_velocity + (
        angular_accel_history_[3] * 55.0 -
        angular_accel_history_[2] * 59.0 +
        angular_accel_history_[1] * 37.0 -
        angular_accel_history_[0] * 9.0
    ) * (dt / 24.0);

    // Adams-Moulton corrector (4th order) for angular velocity
    state.angular_velocity = state.angular_velocity + (
        angular_accel * 9.0 +
        angular_accel_history_[3] * 19.0 -
        angular_accel_history_[2] * 5.0 +
        angular_accel_history_[1] * 1.0
    ) * (dt / 24.0);

    // Quaternion integration using average of predicted and corrected angular velocities
    Vec3 omega_avg = (omega_pred + state.angular_velocity) * 0.5;
    state.orientation = quaternion_utils::integrate(
        state.orientation, omega_avg, dt);

    // ========================================================================
    // Update History Arrays
    // ========================================================================

    // Shift linear acceleration history
    accel_history_[0] = accel_history_[1];
    accel_history_[1] = accel_history_[2];
    accel_history_[2] = accel_history_[3];
    accel_history_[3] = accel;

    // Shift angular acceleration history
    angular_accel_history_[0] = angular_accel_history_[1];
    angular_accel_history_[1] = angular_accel_history_[2];
    angular_accel_history_[2] = angular_accel_history_[3];
    angular_accel_history_[3] = angular_accel;

    // ========================================================================
    // Store computed accelerations for debugging/output
    // ========================================================================

    state.acceleration = accel;
    state.angular_accel = angular_accel;
}

void ABM4Integrator::reset() {
    step_count_ = 0;
    for (auto& a : accel_history_) {
        a = Vec3{0, 0, 0};
    }
    for (auto& a : angular_accel_history_) {
        a = Vec3{0, 0, 0};
    }
}

// Euler integrator
void EulerIntegrator::integrate(EntityState& state, const EntityForces& forces, Real dt) {
    // Skip integration for massless entities
    if (state.mass < constants::MASS_EPSILON) {
        return;
    }

    // Clamp time step to safe bounds
    dt = std::clamp(dt, constants::TIME_STEP_MIN, constants::TIME_STEP_MAX);

    // Transform forces from body frame to ECEF
    Vec3 force_ecef = state.orientation.rotate(forces.force);
    Vec3 accel = force_ecef * (1.0 / state.mass);

    state.position = state.position + state.velocity * dt;
    state.velocity = state.velocity + accel * dt;

    state.orientation = quaternion_utils::integrate(
        state.orientation, state.angular_velocity, dt);
}

} // namespace jaguar::physics
