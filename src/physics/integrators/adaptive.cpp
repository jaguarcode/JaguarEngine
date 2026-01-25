/**
 * @file adaptive.cpp
 * @brief Adaptive time-stepping integrator implementations
 *
 * Implements adaptive integrators using:
 * - Richardson extrapolation (AdaptiveIntegrator)
 * - Embedded Runge-Kutta-Fehlberg (RK45Integrator)
 */

#include "jaguar/physics/solver.h"
#include "jaguar/core/constants.h"
#include <cmath>
#include <algorithm>

namespace jaguar::physics {

// ============================================================================
// AdaptiveIntegrator Implementation (Richardson Extrapolation)
// ============================================================================

AdaptiveIntegrator::AdaptiveIntegrator()
    : base_integrator_(std::make_unique<RK4Integrator>()) {
}

AdaptiveIntegrator::AdaptiveIntegrator(std::unique_ptr<IStatePropagator> base_integrator)
    : base_integrator_(std::move(base_integrator)) {
    if (!base_integrator_) {
        base_integrator_ = std::make_unique<RK4Integrator>();
    }
}

void AdaptiveIntegrator::reset() {
    if (base_integrator_) {
        base_integrator_->reset();
    }
    suggested_dt_ = 0.01;
    last_result_ = AdaptiveStepResult{};
}

Real AdaptiveIntegrator::estimate_error(
    const EntityState& state_full,
    const EntityState& state_half,
    int order) const {
    // Richardson extrapolation error estimate
    // Error ≈ |y_h - y_{h/2}| / (2^p - 1)
    // For 4th order: 2^4 - 1 = 15
    Real factor = std::pow(2.0, order) - 1.0;

    // Position error (dominant for trajectory accuracy)
    Vec3 pos_diff = state_full.position - state_half.position;
    Real pos_error = pos_diff.length() / factor;

    // Velocity error
    Vec3 vel_diff = state_full.velocity - state_half.velocity;
    Real vel_error = vel_diff.length() / factor;

    // Orientation error (angular distance between quaternions)
    // Angular distance = 2 * arccos(|q1 · q2|)
    Real dot = std::abs(
        state_full.orientation.w * state_half.orientation.w +
        state_full.orientation.x * state_half.orientation.x +
        state_full.orientation.y * state_half.orientation.y +
        state_full.orientation.z * state_half.orientation.z
    );
    // Clamp for numerical stability
    dot = std::min(dot, static_cast<Real>(1.0));
    Real orient_error = (2.0 * std::acos(dot)) / factor;

    // Combined error (weighted)
    return pos_error + 0.1 * vel_error + 0.01 * orient_error;
}

Real AdaptiveIntegrator::compute_new_dt(Real dt, Real error, int order) const {
    if (error < 1e-15) {
        return std::min(dt * config_.growth_limit, config_.max_dt);
    }

    // Optimal step size formula: dt_new = dt * safety * (tol/error)^(1/(p+1))
    Real exponent = 1.0 / (order + 1);
    Real ratio = std::pow(config_.tolerance / error, exponent);

    // Apply safety factor and limits
    Real new_dt = dt * config_.safety_factor * ratio;

    // Clamp to growth/shrink limits
    new_dt = std::max(new_dt, dt * config_.shrink_limit);
    new_dt = std::min(new_dt, dt * config_.growth_limit);

    // Clamp to absolute bounds
    new_dt = std::clamp(new_dt, config_.min_dt, config_.max_dt);

    return new_dt;
}

AdaptiveStepResult AdaptiveIntegrator::adaptive_step(
    EntityState& state,
    const EntityForces& forces,
    Real dt) {

    AdaptiveStepResult result;
    result.actual_dt = 0.0;
    result.substeps = 0;

    if (!base_integrator_) {
        result.accepted = false;
        return result;
    }

    int order = base_integrator_->order();
    EntityState original_state = state;

    // Full step
    EntityState state_full = state;
    base_integrator_->integrate(state_full, forces, dt);

    // Two half steps
    EntityState state_half = state;
    base_integrator_->integrate(state_half, forces, dt * 0.5);
    base_integrator_->integrate(state_half, forces, dt * 0.5);

    // Estimate error
    Real error = estimate_error(state_full, state_half, order);
    result.error_estimate = error;
    result.substeps = 1;

    if (error <= config_.tolerance) {
        // Step accepted - use Richardson extrapolated result (more accurate)
        // y_accurate ≈ y_{h/2} + (y_{h/2} - y_h) / (2^p - 1)
        Real factor = std::pow(2.0, order) - 1.0;

        state.position = state_half.position +
            (state_half.position - state_full.position) * (1.0 / factor);
        state.velocity = state_half.velocity +
            (state_half.velocity - state_full.velocity) * (1.0 / factor);
        state.acceleration = state_half.acceleration;
        state.angular_velocity = state_half.angular_velocity;
        state.angular_accel = state_half.angular_accel;

        // For quaternion, just use the half-step result (already normalized)
        state.orientation = state_half.orientation;

        result.accepted = true;
        result.actual_dt = dt;
        result.suggested_dt = compute_new_dt(dt, error, order);
    } else {
        // Step rejected - restore original state
        state = original_state;
        result.accepted = false;
        result.actual_dt = 0.0;
        result.suggested_dt = compute_new_dt(dt, error, order);
    }

    return result;
}

void AdaptiveIntegrator::integrate(EntityState& state, const EntityForces& forces, Real dt) {
    // Skip integration for massless entities
    if (state.mass < constants::MASS_EPSILON) {
        return;
    }

    // Clamp time step to safe bounds
    dt = std::clamp(dt, constants::TIME_STEP_MIN, constants::TIME_STEP_MAX);

    Real time_remaining = dt;
    Real current_dt = std::min(suggested_dt_, dt);

    last_result_ = AdaptiveStepResult{};
    last_result_.substeps = 0;

    int substep_count = 0;

    while (time_remaining > 1e-15 && substep_count < config_.max_substeps) {
        // Don't overshoot
        current_dt = std::min(current_dt, time_remaining);
        current_dt = std::max(current_dt, config_.min_dt);

        AdaptiveStepResult step_result = adaptive_step(state, forces, current_dt);
        substep_count++;

        if (step_result.accepted) {
            time_remaining -= step_result.actual_dt;
            current_dt = step_result.suggested_dt;
            last_result_.error_estimate = std::max(last_result_.error_estimate,
                                                    step_result.error_estimate);
        } else {
            // Reduce step size and retry
            current_dt = step_result.suggested_dt;
        }
    }

    last_result_.substeps = substep_count;
    last_result_.actual_dt = dt - time_remaining;
    last_result_.accepted = (time_remaining < 1e-10);
    last_result_.suggested_dt = current_dt;
    suggested_dt_ = current_dt;
}

// ============================================================================
// RK45Integrator Implementation (Runge-Kutta-Fehlberg)
// ============================================================================

RK45Integrator::RK45Integrator() = default;

namespace {

/**
 * @brief Compute angular acceleration using Euler's equations
 */
Vec3 compute_angular_accel_rk45(const Mat3x3& inertia, const Vec3& omega, const Vec3& torque) {
    // Compute I * ω
    Vec3 I_omega = inertia * omega;

    // Compute ω × (I * ω) - gyroscopic term
    Vec3 gyroscopic = omega.cross(I_omega);

    // Compute inverse inertia (simplified for diagonal dominance)
    Real det = inertia.determinant();
    Mat3x3 inv_inertia;
    if (std::abs(det) < 1e-10) {
        inv_inertia = Mat3x3::Identity();
        Real Ixx = inertia(0, 0);
        Real Iyy = inertia(1, 1);
        Real Izz = inertia(2, 2);
        if (Ixx > 1e-10) inv_inertia(0, 0) = 1.0 / Ixx;
        if (Iyy > 1e-10) inv_inertia(1, 1) = 1.0 / Iyy;
        if (Izz > 1e-10) inv_inertia(2, 2) = 1.0 / Izz;
    } else {
        inv_inertia = inertia.inverse();
    }

    return inv_inertia * (torque - gyroscopic);
}

} // anonymous namespace

void RK45Integrator::integrate(EntityState& state, const EntityForces& forces, Real dt) {
    // Skip integration for massless entities
    if (state.mass < constants::MASS_EPSILON) {
        return;
    }

    // Clamp time step to safe bounds
    dt = std::clamp(dt, constants::TIME_STEP_MIN, constants::TIME_STEP_MAX);

    // RK45 Butcher tableau coefficients (Fehlberg)
    // c:  0, 1/4, 3/8, 12/13, 1, 1/2
    // a21 = 1/4
    // a31 = 3/32, a32 = 9/32
    // a41 = 1932/2197, a42 = -7200/2197, a43 = 7296/2197
    // a51 = 439/216, a52 = -8, a53 = 3680/513, a54 = -845/4104
    // a61 = -8/27, a62 = 2, a63 = -3544/2565, a64 = 1859/4104, a65 = -11/40

    // 5th order weights (b):
    // b1 = 16/135, b2 = 0, b3 = 6656/12825, b4 = 28561/56430, b5 = -9/50, b6 = 2/55

    // 4th order weights (b*):
    // b1* = 25/216, b2* = 0, b3* = 1408/2565, b4* = 2197/4104, b5* = -1/5, b6* = 0

    Real inv_mass = 1.0 / state.mass;
    Vec3 force_ecef = state.orientation.rotate(forces.force);
    Vec3 accel = force_ecef * inv_mass;

    // For constant force, acceleration doesn't change with position
    // This is a simplification; for more complex force models, we'd need
    // to recompute acceleration at each stage

    // ========================================================================
    // Translational Motion (RK45)
    // ========================================================================

    // k1
    Vec3 k1_v = accel;
    Vec3 k1_p = state.velocity;

    // k2 at t + dt/4
    // Note: k2_p/k2_v used in intermediate stages but b2=0 in RK45 weights
    Vec3 v2 = state.velocity + k1_v * (dt * 0.25);
    Vec3 k2_v = accel;
    [[maybe_unused]] Vec3 k2_p = v2;

    // k3 at t + 3*dt/8
    Vec3 v3 = state.velocity + (k1_v * (3.0/32.0) + k2_v * (9.0/32.0)) * dt;
    Vec3 k3_v = accel;
    Vec3 k3_p = v3;

    // k4 at t + 12*dt/13
    Vec3 v4 = state.velocity + (k1_v * (1932.0/2197.0) - k2_v * (7200.0/2197.0)
                                + k3_v * (7296.0/2197.0)) * dt;
    Vec3 k4_v = accel;
    Vec3 k4_p = v4;

    // k5 at t + dt
    Vec3 v5 = state.velocity + (k1_v * (439.0/216.0) - k2_v * 8.0
                                + k3_v * (3680.0/513.0) - k4_v * (845.0/4104.0)) * dt;
    Vec3 k5_v = accel;
    Vec3 k5_p = v5;

    // k6 at t + dt/2
    Vec3 v6 = state.velocity + (-k1_v * (8.0/27.0) + k2_v * 2.0
                                - k3_v * (3544.0/2565.0) + k4_v * (1859.0/4104.0)
                                - k5_v * (11.0/40.0)) * dt;
    Vec3 k6_v = accel;
    Vec3 k6_p = v6;

    // 5th order solution
    Vec3 vel_5 = state.velocity + (k1_v * (16.0/135.0) + k3_v * (6656.0/12825.0)
                                    + k4_v * (28561.0/56430.0) - k5_v * (9.0/50.0)
                                    + k6_v * (2.0/55.0)) * dt;

    Vec3 pos_5 = state.position + (k1_p * (16.0/135.0) + k3_p * (6656.0/12825.0)
                                    + k4_p * (28561.0/56430.0) - k5_p * (9.0/50.0)
                                    + k6_p * (2.0/55.0)) * dt;

    // 4th order solution (for error estimation)
    Vec3 vel_4 = state.velocity + (k1_v * (25.0/216.0) + k3_v * (1408.0/2565.0)
                                    + k4_v * (2197.0/4104.0) - k5_v * (1.0/5.0)) * dt;

    Vec3 pos_4 = state.position + (k1_p * (25.0/216.0) + k3_p * (1408.0/2565.0)
                                    + k4_p * (2197.0/4104.0) - k5_p * (1.0/5.0)) * dt;

    // Error estimation
    Vec3 pos_err = pos_5 - pos_4;
    Vec3 vel_err = vel_5 - vel_4;
    error_estimate_ = pos_err.length() + 0.1 * vel_err.length();

    // ========================================================================
    // Rotational Motion (RK45 with simplified approach)
    // ========================================================================

    // For angular motion, use RK4 approach (simpler for rotation)
    Vec3 omega = state.angular_velocity;
    Vec3 alpha1 = compute_angular_accel_rk45(state.inertia, omega, forces.torque);

    Vec3 omega2 = omega + alpha1 * (dt * 0.25);
    Vec3 alpha2 = compute_angular_accel_rk45(state.inertia, omega2, forces.torque);

    Vec3 omega3 = omega + (alpha1 * (3.0/32.0) + alpha2 * (9.0/32.0)) * dt;
    Vec3 alpha3 = compute_angular_accel_rk45(state.inertia, omega3, forces.torque);

    Vec3 omega4 = omega + (alpha1 * (1932.0/2197.0) - alpha2 * (7200.0/2197.0)
                           + alpha3 * (7296.0/2197.0)) * dt;
    Vec3 alpha4 = compute_angular_accel_rk45(state.inertia, omega4, forces.torque);

    Vec3 omega5 = omega + (alpha1 * (439.0/216.0) - alpha2 * 8.0
                           + alpha3 * (3680.0/513.0) - alpha4 * (845.0/4104.0)) * dt;
    Vec3 alpha5 = compute_angular_accel_rk45(state.inertia, omega5, forces.torque);

    Vec3 omega6 = omega + (-alpha1 * (8.0/27.0) + alpha2 * 2.0
                           - alpha3 * (3544.0/2565.0) + alpha4 * (1859.0/4104.0)
                           - alpha5 * (11.0/40.0)) * dt;
    Vec3 alpha6 = compute_angular_accel_rk45(state.inertia, omega6, forces.torque);

    Vec3 omega_new = omega + (alpha1 * (16.0/135.0) + alpha3 * (6656.0/12825.0)
                              + alpha4 * (28561.0/56430.0) - alpha5 * (9.0/50.0)
                              + alpha6 * (2.0/55.0)) * dt;

    // ========================================================================
    // Apply Results
    // ========================================================================

    // Accept step if error within tolerance
    if (error_estimate_ <= tolerance_) {
        state.position = pos_5;
        state.velocity = vel_5;
        state.angular_velocity = omega_new;
        state.acceleration = accel;
        state.angular_accel = alpha1;

        // Quaternion integration (use RK4 approach)
        state.orientation = quaternion_utils::integrate_rk4(
            state.orientation, state.angular_velocity, dt);
    } else {
        // Use 4th order solution as fallback (less accurate but stable)
        state.position = pos_4;
        state.velocity = vel_4;
        state.angular_velocity = omega + (alpha1 * (25.0/216.0) + alpha3 * (1408.0/2565.0)
                                          + alpha4 * (2197.0/4104.0) - alpha5 * (1.0/5.0)) * dt;
        state.acceleration = accel;
        state.angular_accel = alpha1;

        state.orientation = quaternion_utils::integrate_rk4(
            state.orientation, state.angular_velocity, dt);
    }

    // Compute suggested next step size
    if (error_estimate_ > 1e-15) {
        Real ratio = std::pow(tolerance_ / error_estimate_, 0.2); // 1/(p+1) = 1/5
        suggested_dt_ = dt * 0.9 * ratio; // 0.9 safety factor
        suggested_dt_ = std::clamp(suggested_dt_, 1e-9, 0.1);
    } else {
        suggested_dt_ = std::min(dt * 2.0, 0.1);
    }
}

} // namespace jaguar::physics
