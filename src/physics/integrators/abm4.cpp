/**
 * @file abm4.cpp
 * @brief Adams-Bashforth-Moulton integrator implementation (stub)
 */

#include "jaguar/physics/solver.h"

namespace jaguar::physics {

ABM4Integrator::ABM4Integrator() {
    reset();
}

void ABM4Integrator::integrate(EntityState& state, const EntityForces& forces, Real dt) {
    // Transform forces from body frame to ECEF
    // orientation is body-to-ECEF, so we rotate force from body to ECEF
    Vec3 force_ecef = state.orientation.rotate(forces.force);

    // For first 4 steps, use RK4 to build history
    if (step_count_ < 4) {
        // Store acceleration history (in ECEF)
        Vec3 accel = force_ecef * (1.0 / state.mass);
        accel_history_[step_count_] = accel;

        // Use RK4 for startup
        RK4Integrator rk4;
        rk4.integrate(state, forces, dt);

        step_count_++;
        return;
    }

    // Full ABM4 integration
    Vec3 accel = force_ecef * (1.0 / state.mass);

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

    // Shift history
    accel_history_[0] = accel_history_[1];
    accel_history_[1] = accel_history_[2];
    accel_history_[2] = accel_history_[3];
    accel_history_[3] = accel;

    // Quaternion integration (same as RK4)
    state.orientation = quaternion_utils::integrate(
        state.orientation, state.angular_velocity, dt);
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
    if (state.mass < 1e-10) return;

    // Transform forces from body frame to ECEF
    Vec3 force_ecef = state.orientation.rotate(forces.force);
    Vec3 accel = force_ecef * (1.0 / state.mass);

    state.position = state.position + state.velocity * dt;
    state.velocity = state.velocity + accel * dt;

    state.orientation = quaternion_utils::integrate(
        state.orientation, state.angular_velocity, dt);
}

} // namespace jaguar::physics
