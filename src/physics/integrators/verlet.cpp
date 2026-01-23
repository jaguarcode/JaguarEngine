/**
 * @file verlet.cpp
 * @brief Verlet integrator implementations
 *
 * Implements Velocity Verlet and Position Verlet methods for rigid body
 * dynamics with full inertia tensor support. These integrators are designed
 * for long-duration simulations where energy conservation is critical.
 *
 * The Verlet methods are second-order accurate and symplectic, providing
 * bounded energy error for arbitrarily long simulations.
 */

#include "jaguar/physics/integrators/verlet.h"
#include <cmath>

namespace jaguar::physics {

// ============================================================================
// Helper Functions
// ============================================================================

namespace {

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

// ============================================================================
// VelocityVerletIntegrator Implementation
// ============================================================================

VelocityVerletIntegrator::VelocityVerletIntegrator() = default;

Vec3 VelocityVerletIntegrator::compute_angular_accel(
    const Mat3x3& inertia, const Mat3x3& inv_inertia,
    const Vec3& omega, const Vec3& torque) const {
    // Euler's equations: I * ω̇ = τ - ω × (I * ω)
    Vec3 I_omega = inertia * omega;
    Vec3 gyroscopic = omega.cross(I_omega);
    return inv_inertia * (torque - gyroscopic);
}

Real VelocityVerletIntegrator::compute_mechanical_energy(const EntityState& state) const {
    // Translational kinetic energy: (1/2) * m * |v|²
    Real kinetic_trans = 0.5 * state.mass * state.velocity.length_squared();

    // Rotational kinetic energy: (1/2) * ω · (I * ω)
    Vec3 I_omega = state.inertia * state.angular_velocity;
    Real kinetic_rot = 0.5 * state.angular_velocity.dot(I_omega);

    return kinetic_trans + kinetic_rot;
}

void VelocityVerletIntegrator::integrate(EntityState& state, const EntityForces& forces, Real dt) {
    if (state.mass < 1e-10) return;

    Real inv_mass = 1.0 / state.mass;
    Mat3x3 inv_inertia = compute_inverse_inertia(state.inertia);

    // Transform forces from body to ECEF frame
    Vec3 force_ecef = state.orientation.rotate(forces.force);
    Vec3 accel = force_ecef * inv_mass;

    // Compute angular acceleration using Euler's equations
    Vec3 angular_accel = compute_angular_accel(
        state.inertia, inv_inertia,
        state.angular_velocity, forces.torque);

    // On first step, initialize previous acceleration
    if (first_step_) {
        prev_accel_ = accel;
        prev_angular_accel_ = angular_accel;
        first_step_ = false;
    }

    // Energy tracking (pre-integration)
    if (track_energy_) {
        Real current_energy = compute_mechanical_energy(state);
        if (!energy_initialized_) {
            initial_energy_ = current_energy;
            energy_initialized_ = true;
        }
        energy_drift_ = current_energy - initial_energy_;
    }

    // ========================================================================
    // Velocity Verlet (Leapfrog) Algorithm
    // ========================================================================
    //
    // The Velocity Verlet method is a second-order symplectic integrator:
    //
    // Step 1: Update position using current velocity and acceleration
    //   x(t+dt) = x(t) + v(t)*dt + 0.5*a(t)*dt²
    //
    // Step 2: Compute new acceleration a(t+dt) from x(t+dt)
    //   (In our case, forces are provided externally, so we use the
    //   current acceleration as an approximation)
    //
    // Step 3: Update velocity using average of old and new acceleration
    //   v(t+dt) = v(t) + 0.5*(a(t) + a(t+dt))*dt

    // ========================================================================
    // Translational Motion
    // ========================================================================

    // Step 1: Update position (half using old velocity, half using acceleration)
    // x_new = x + v*dt + 0.5*a*dt²
    state.position = state.position + state.velocity * dt + prev_accel_ * (0.5 * dt * dt);

    // Step 3: Update velocity using average acceleration
    // v_new = v + 0.5*(a_old + a_new)*dt
    state.velocity = state.velocity + (prev_accel_ + accel) * (0.5 * dt);

    // ========================================================================
    // Rotational Motion
    // ========================================================================

    // Update angular velocity using average angular acceleration
    state.angular_velocity = state.angular_velocity +
        (prev_angular_accel_ + angular_accel) * (0.5 * dt);

    // Integrate quaternion orientation
    // For rotational Verlet, we use an approximation since rotations don't
    // commute. We integrate the quaternion using the updated angular velocity.
    state.orientation = quaternion_utils::integrate(
        state.orientation, state.angular_velocity, dt);

    // ========================================================================
    // Store accelerations for next step and output
    // ========================================================================

    prev_accel_ = accel;
    prev_angular_accel_ = angular_accel;
    state.acceleration = accel;
    state.angular_accel = angular_accel;

    // Energy tracking (post-integration)
    if (track_energy_) {
        Real current_energy = compute_mechanical_energy(state);
        energy_drift_ = current_energy - initial_energy_;
    }
}

void VelocityVerletIntegrator::set_previous_acceleration(
    const Vec3& accel, const Vec3& angular_accel) {
    prev_accel_ = accel;
    prev_angular_accel_ = angular_accel;
    first_step_ = false;
}

void VelocityVerletIntegrator::reset_energy_tracking() {
    energy_initialized_ = false;
    initial_energy_ = 0.0;
    energy_drift_ = 0.0;
}

void VelocityVerletIntegrator::reset() {
    first_step_ = true;
    prev_accel_ = Vec3::Zero();
    prev_angular_accel_ = Vec3::Zero();
    reset_energy_tracking();
}

// ============================================================================
// PositionVerletIntegrator Implementation
// ============================================================================

PositionVerletIntegrator::PositionVerletIntegrator() = default;

Vec3 PositionVerletIntegrator::compute_angular_accel(
    const Mat3x3& inertia, const Mat3x3& inv_inertia,
    const Vec3& omega, const Vec3& torque) const {
    Vec3 I_omega = inertia * omega;
    Vec3 gyroscopic = omega.cross(I_omega);
    return inv_inertia * (torque - gyroscopic);
}

Real PositionVerletIntegrator::compute_mechanical_energy(const EntityState& state) const {
    Real kinetic_trans = 0.5 * state.mass * state.velocity.length_squared();
    Vec3 I_omega = state.inertia * state.angular_velocity;
    Real kinetic_rot = 0.5 * state.angular_velocity.dot(I_omega);
    return kinetic_trans + kinetic_rot;
}

void PositionVerletIntegrator::integrate(EntityState& state, const EntityForces& forces, Real dt) {
    if (state.mass < 1e-10) return;

    Real inv_mass = 1.0 / state.mass;
    Mat3x3 inv_inertia = compute_inverse_inertia(state.inertia);

    // Transform forces from body to ECEF frame
    Vec3 force_ecef = state.orientation.rotate(forces.force);
    Vec3 accel = force_ecef * inv_mass;

    // Compute angular acceleration
    Vec3 angular_accel = compute_angular_accel(
        state.inertia, inv_inertia,
        state.angular_velocity, forces.torque);

    // Energy tracking (pre-integration)
    if (track_energy_) {
        Real current_energy = compute_mechanical_energy(state);
        if (!energy_initialized_) {
            initial_energy_ = current_energy;
            energy_initialized_ = true;
        }
        energy_drift_ = current_energy - initial_energy_;
    }

    // ========================================================================
    // Position Verlet (Störmer-Verlet) Algorithm
    // ========================================================================
    //
    // The Position Verlet method uses positions at two time levels:
    //   x(t+dt) = 2*x(t) - x(t-dt) + a(t)*dt²
    //
    // Velocity is derived from position:
    //   v(t) ≈ (x(t+dt) - x(t-dt)) / (2*dt)
    //
    // On the first step, we use symplectic Euler to initialize.

    if (first_step_) {
        // First step: use symplectic Euler to bootstrap
        // Store current position as previous
        prev_position_ = state.position;
        prev_orientation_ = state.orientation;
        prev_dt_ = dt;

        // Symplectic Euler step
        state.velocity = state.velocity + accel * dt;
        state.position = state.position + state.velocity * dt;

        state.angular_velocity = state.angular_velocity + angular_accel * dt;
        state.orientation = quaternion_utils::integrate(
            state.orientation, state.angular_velocity, dt);

        first_step_ = false;
    } else {
        // ====================================================================
        // Translational Motion (Position Verlet)
        // ====================================================================

        // Store current position for velocity calculation and next step
        Vec3 current_position = state.position;

        // x(t+dt) = 2*x(t) - x(t-dt) + a(t)*dt²
        Vec3 new_position = state.position * 2.0 - prev_position_ + accel * (dt * dt);

        // Compute velocity from position difference (central difference)
        // v(t) ≈ (x(t+dt) - x(t-dt)) / (2*dt)
        state.velocity = (new_position - prev_position_) * (1.0 / (2.0 * dt));

        // Update position
        state.position = new_position;
        prev_position_ = current_position;

        // ====================================================================
        // Rotational Motion
        // ====================================================================
        //
        // Rotational Verlet is more complex due to non-commutative nature.
        // We use a simplified approach: update angular velocity using Verlet
        // and integrate orientation normally.

        // Update angular velocity using modified Verlet
        state.angular_velocity = state.angular_velocity + angular_accel * dt;

        // Integrate quaternion
        state.orientation = quaternion_utils::integrate(
            state.orientation, state.angular_velocity, dt);
    }

    // Store accelerations for output
    state.acceleration = accel;
    state.angular_accel = angular_accel;

    // Energy tracking (post-integration)
    if (track_energy_) {
        Real current_energy = compute_mechanical_energy(state);
        energy_drift_ = current_energy - initial_energy_;
    }
}

void PositionVerletIntegrator::reset_energy_tracking() {
    energy_initialized_ = false;
    initial_energy_ = 0.0;
    energy_drift_ = 0.0;
}

void PositionVerletIntegrator::reset() {
    first_step_ = true;
    prev_position_ = Vec3::Zero();
    prev_orientation_ = Quat::Identity();
    prev_dt_ = 0.0;
    reset_energy_tracking();
}

} // namespace jaguar::physics
