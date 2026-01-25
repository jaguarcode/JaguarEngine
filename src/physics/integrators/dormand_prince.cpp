/**
 * @file dormand_prince.cpp
 * @brief Dormand-Prince (DOPRI5) integrator implementation
 *
 * Implements the Dormand-Prince 5(4) embedded Runge-Kutta method with
 * FSAL (First Same As Last) optimization and adaptive step size control.
 *
 * This is the standard ODE solver used in MATLAB's ode45 and is considered
 * one of the most efficient general-purpose integrators available.
 *
 * References:
 * - Dormand, J. R.; Prince, P. J. (1980), "A family of embedded Runge-Kutta
 *   formulae", Journal of Computational and Applied Mathematics
 * - Hairer, E.; Nørsett, S. P.; Wanner, G. (1993), "Solving Ordinary
 *   Differential Equations I: Nonstiff Problems", Springer
 */

#include "jaguar/physics/integrators/dormand_prince.h"
#include "jaguar/core/constants.h"
#include <cmath>
#include <algorithm>

namespace jaguar::physics {

DormandPrinceIntegrator::DormandPrinceIntegrator() = default;

void DormandPrinceIntegrator::reset() {
    fsal_valid_ = false;
    fsal_accel_ = Vec3::Zero();
    fsal_angular_accel_ = Vec3::Zero();
    error_estimate_ = 0.0;
    suggested_dt_ = 0.01;
    last_substeps_ = 0;
    rejected_steps_ = 0;
    function_evals_ = 0;
}

Vec3 DormandPrinceIntegrator::compute_accel(
    const EntityState& state, const EntityForces& forces) const {
    if (state.mass < constants::MASS_EPSILON) return Vec3::Zero();

    // Transform forces from body to ECEF frame
    Vec3 force_ecef = state.orientation.rotate(forces.force);
    return force_ecef * (1.0 / state.mass);
}

Vec3 DormandPrinceIntegrator::compute_angular_accel(
    const EntityState& state, const EntityForces& forces) const {
    // Euler's equations: I * ω̇ = τ - ω × (I * ω)
    Vec3 I_omega = state.inertia * state.angular_velocity;
    Vec3 gyroscopic = state.angular_velocity.cross(I_omega);

    // Compute inverse inertia
    Real det = state.inertia.determinant();
    Mat3x3 inv_inertia;
    if (std::abs(det) < 1e-10) {
        inv_inertia = Mat3x3::Identity();
        Real Ixx = state.inertia(0, 0);
        Real Iyy = state.inertia(1, 1);
        Real Izz = state.inertia(2, 2);
        if (Ixx > 1e-10) inv_inertia(0, 0) = 1.0 / Ixx;
        if (Iyy > 1e-10) inv_inertia(1, 1) = 1.0 / Iyy;
        if (Izz > 1e-10) inv_inertia(2, 2) = 1.0 / Izz;
    } else {
        inv_inertia = state.inertia.inverse();
    }

    return inv_inertia * (forces.torque - gyroscopic);
}

Real DormandPrinceIntegrator::error_norm(
    const Vec3& pos_err, const Vec3& vel_err,
    const EntityState& state) const {
    // Mixed error norm: err_i / (atol + rtol * max(|y_i|, |y_i+1|))
    // We use a simplified version with position and velocity

    Real pos_scale = atol_ + rtol_ * state.position.length();
    Real vel_scale = atol_ + rtol_ * state.velocity.length();

    Real pos_norm = pos_err.length() / std::max(pos_scale, 1e-15);
    Real vel_norm = vel_err.length() / std::max(vel_scale, 1e-15);

    // Combined norm (position dominates for trajectory)
    return std::sqrt((pos_norm * pos_norm + 0.1 * vel_norm * vel_norm) / 1.1);
}

Real DormandPrinceIntegrator::compute_new_dt(
    Real dt, Real error, Real prev_error) const {
    // PI controller for step size
    // h_new = h * min(fac_max, max(fac_min, fac * (1/err)^(1/5) * (prev_err)^beta))

    if (error < 1e-15) {
        return std::min(dt * fac_max_, max_dt_);
    }

    Real fac = safety_ * std::pow(error, -0.2);

    // Add PI control term if we have previous error
    if (prev_error > 1e-15) {
        fac *= std::pow(prev_error, beta_);
    }

    // Apply limits
    fac = std::clamp(fac, fac_min_, fac_max_);

    Real new_dt = dt * fac;
    return std::clamp(new_dt, min_dt_, max_dt_);
}

Real DormandPrinceIntegrator::dopri_step(
    const EntityState& state, const EntityForces& forces,
    Real dt, EntityState& new_state) {

    // Stage 1: Use FSAL cache if valid, otherwise compute
    Vec3 k1_a, k1_alpha;
    if (fsal_valid_) {
        k1_a = fsal_accel_;
        k1_alpha = fsal_angular_accel_;
    } else {
        k1_a = compute_accel(state, forces);
        k1_alpha = compute_angular_accel(state, forces);
        function_evals_++;
    }
    Vec3 k1_v = state.velocity;

    // Stage 2: t + c2*dt = t + 0.2*dt
    EntityState s2 = state;
    s2.position = state.position + k1_v * (a21_ * dt);
    s2.velocity = state.velocity + k1_a * (a21_ * dt);
    s2.angular_velocity = state.angular_velocity + k1_alpha * (a21_ * dt);
    Vec3 k2_a = compute_accel(s2, forces);
    Vec3 k2_alpha = compute_angular_accel(s2, forces);
    Vec3 k2_v = s2.velocity;
    function_evals_++;

    // Stage 3: t + c3*dt = t + 0.3*dt
    EntityState s3 = state;
    s3.position = state.position + (k1_v * a31_ + k2_v * a32_) * dt;
    s3.velocity = state.velocity + (k1_a * a31_ + k2_a * a32_) * dt;
    s3.angular_velocity = state.angular_velocity + (k1_alpha * a31_ + k2_alpha * a32_) * dt;
    Vec3 k3_a = compute_accel(s3, forces);
    Vec3 k3_alpha = compute_angular_accel(s3, forces);
    Vec3 k3_v = s3.velocity;
    function_evals_++;

    // Stage 4: t + c4*dt = t + 0.8*dt
    EntityState s4 = state;
    s4.position = state.position + (k1_v * a41_ + k2_v * a42_ + k3_v * a43_) * dt;
    s4.velocity = state.velocity + (k1_a * a41_ + k2_a * a42_ + k3_a * a43_) * dt;
    s4.angular_velocity = state.angular_velocity +
        (k1_alpha * a41_ + k2_alpha * a42_ + k3_alpha * a43_) * dt;
    Vec3 k4_a = compute_accel(s4, forces);
    Vec3 k4_alpha = compute_angular_accel(s4, forces);
    Vec3 k4_v = s4.velocity;
    function_evals_++;

    // Stage 5: t + c5*dt = t + (8/9)*dt
    EntityState s5 = state;
    s5.position = state.position +
        (k1_v * a51_ + k2_v * a52_ + k3_v * a53_ + k4_v * a54_) * dt;
    s5.velocity = state.velocity +
        (k1_a * a51_ + k2_a * a52_ + k3_a * a53_ + k4_a * a54_) * dt;
    s5.angular_velocity = state.angular_velocity +
        (k1_alpha * a51_ + k2_alpha * a52_ + k3_alpha * a53_ + k4_alpha * a54_) * dt;
    Vec3 k5_a = compute_accel(s5, forces);
    Vec3 k5_alpha = compute_angular_accel(s5, forces);
    Vec3 k5_v = s5.velocity;
    function_evals_++;

    // Stage 6: t + dt
    EntityState s6 = state;
    s6.position = state.position +
        (k1_v * a61_ + k2_v * a62_ + k3_v * a63_ + k4_v * a64_ + k5_v * a65_) * dt;
    s6.velocity = state.velocity +
        (k1_a * a61_ + k2_a * a62_ + k3_a * a63_ + k4_a * a64_ + k5_a * a65_) * dt;
    s6.angular_velocity = state.angular_velocity +
        (k1_alpha * a61_ + k2_alpha * a62_ + k3_alpha * a63_ +
         k4_alpha * a64_ + k5_alpha * a65_) * dt;
    Vec3 k6_a = compute_accel(s6, forces);
    Vec3 k6_alpha = compute_angular_accel(s6, forces);
    Vec3 k6_v = s6.velocity;
    function_evals_++;

    // Stage 7: t + dt (FSAL - this becomes k1 for next step)
    // The 5th order solution
    new_state = state;
    new_state.position = state.position +
        (k1_v * b1_ + k3_v * b3_ + k4_v * b4_ + k5_v * b5_ + k6_v * b6_) * dt;
    new_state.velocity = state.velocity +
        (k1_a * b1_ + k3_a * b3_ + k4_a * b4_ + k5_a * b5_ + k6_a * b6_) * dt;
    new_state.angular_velocity = state.angular_velocity +
        (k1_alpha * b1_ + k3_alpha * b3_ + k4_alpha * b4_ +
         k5_alpha * b5_ + k6_alpha * b6_) * dt;

    Vec3 k7_a = compute_accel(new_state, forces);
    Vec3 k7_alpha = compute_angular_accel(new_state, forces);
    function_evals_++;

    // Store FSAL cache for potential next step
    fsal_accel_ = k7_a;
    fsal_angular_accel_ = k7_alpha;

    // Compute error estimate: difference between 5th and 4th order solutions
    // Using error coefficients (e = b - b*)
    Vec3 pos_err = (k1_v * e1_ + k3_v * e3_ + k4_v * e4_ +
                    k5_v * e5_ + k6_v * e6_ + new_state.velocity * e7_) * dt;
    Vec3 vel_err = (k1_a * e1_ + k3_a * e3_ + k4_a * e4_ +
                    k5_a * e5_ + k6_a * e6_ + k7_a * e7_) * dt;

    // Compute error norm
    Real error = error_norm(pos_err, vel_err, new_state);

    // Update acceleration and angular acceleration
    new_state.acceleration = k7_a;
    new_state.angular_accel = k7_alpha;

    // Copy other state data
    new_state.mass = state.mass;
    new_state.inertia = state.inertia;

    // Integrate quaternion using angular velocity
    new_state.orientation = quaternion_utils::integrate_rk4(
        state.orientation, new_state.angular_velocity, dt);

    return error;
}

void DormandPrinceIntegrator::integrate(
    EntityState& state, const EntityForces& forces, Real dt) {

    // Skip integration for massless entities
    if (state.mass < constants::MASS_EPSILON) {
        return;
    }

    // Clamp time step to safe bounds
    dt = std::clamp(dt, constants::TIME_STEP_MIN, constants::TIME_STEP_MAX);

    Real time_remaining = dt;
    Real current_dt = std::min(suggested_dt_, dt);
    Real prev_error = 0.0;

    last_substeps_ = 0;
    rejected_steps_ = 0;

    while (time_remaining > 1e-15 && last_substeps_ < max_substeps_) {
        // Don't overshoot
        current_dt = std::min(current_dt, time_remaining);
        current_dt = std::max(current_dt, min_dt_);

        // Perform DOPRI5 step
        EntityState new_state;
        Real error = dopri_step(state, forces, current_dt, new_state);
        error_estimate_ = error;

        last_substeps_++;

        if (error <= 1.0) {
            // Step accepted
            state = new_state;
            time_remaining -= current_dt;
            fsal_valid_ = true;  // FSAL cache is valid for next step

            // Compute new step size
            Real new_dt = compute_new_dt(current_dt, error, prev_error);
            prev_error = error;
            current_dt = new_dt;
        } else {
            // Step rejected - reduce step size and retry
            Real new_dt = compute_new_dt(current_dt, error, 0.0);
            current_dt = std::max(new_dt, min_dt_);
            fsal_valid_ = false;  // FSAL cache invalid after rejection
            rejected_steps_++;
        }
    }

    suggested_dt_ = current_dt;
}

} // namespace jaguar::physics
