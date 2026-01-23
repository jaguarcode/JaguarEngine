/**
 * @file symplectic_euler.cpp
 * @brief Symplectic Euler integrator implementation
 *
 * Implements the semi-implicit Euler method (symplectic Euler) for rigid body
 * dynamics with full inertia tensor support. This integrator is designed for
 * long-duration simulations where energy conservation is critical, such as
 * orbital mechanics and N-body systems.
 *
 * The symplectic property ensures that the integrator preserves phase space
 * volume, resulting in bounded energy oscillation rather than secular drift.
 */

#include "jaguar/physics/integrators/symplectic_euler.h"
#include <cmath>

namespace jaguar::physics {

SymplecticEulerIntegrator::SymplecticEulerIntegrator() = default;

namespace {

/**
 * @brief Compute inverse of inertia tensor
 *
 * Uses full matrix inversion when possible, falling back to diagonal
 * inversion for near-singular matrices.
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

Vec3 SymplecticEulerIntegrator::compute_angular_accel(
    const Mat3x3& inertia, const Mat3x3& inv_inertia,
    const Vec3& omega, const Vec3& torque) const {
    // Euler's equations for a rigid body:
    // I * ω̇ = τ - ω × (I * ω)
    //
    // Where:
    // - I is the inertia tensor
    // - ω is the angular velocity
    // - τ is the applied torque
    // - ω̇ is the angular acceleration

    // Compute I * ω
    Vec3 I_omega = inertia * omega;

    // Compute ω × (I * ω) - gyroscopic coupling term
    Vec3 gyroscopic = omega.cross(I_omega);

    // ω̇ = I⁻¹ * (τ - ω × (I * ω))
    return inv_inertia * (torque - gyroscopic);
}

Real SymplecticEulerIntegrator::compute_mechanical_energy(const EntityState& state) const {
    // Total mechanical energy = kinetic + rotational kinetic
    // E = (1/2) * m * v² + (1/2) * ω · (I * ω)
    //
    // Note: Potential energy would require force generators and is not
    // tracked here. This computes kinetic energy only.

    // Translational kinetic energy: (1/2) * m * |v|²
    Real kinetic_trans = 0.5 * state.mass * state.velocity.length_squared();

    // Rotational kinetic energy: (1/2) * ω · (I * ω)
    Vec3 I_omega = state.inertia * state.angular_velocity;
    Real kinetic_rot = 0.5 * state.angular_velocity.dot(I_omega);

    return kinetic_trans + kinetic_rot;
}

void SymplecticEulerIntegrator::integrate(EntityState& state, const EntityForces& forces, Real dt) {
    // Skip integration for zero-mass entities (static objects)
    if (state.mass < 1e-10) return;

    // Compute inverse mass and inertia
    Real inv_mass = 1.0 / state.mass;
    Mat3x3 inv_inertia = compute_inverse_inertia(state.inertia);

    // ========================================================================
    // Energy Tracking (Pre-Integration)
    // ========================================================================

    if (track_energy_) {
        Real current_energy = compute_mechanical_energy(state);
        if (!energy_initialized_) {
            initial_energy_ = current_energy;
            energy_initialized_ = true;
        }
        energy_drift_ = current_energy - initial_energy_;
    }

    // ========================================================================
    // Forces are in body frame, transform to ECEF for integration
    // orientation is body-to-ECEF, so we rotate force from body to ECEF
    // ========================================================================

    Vec3 force_ecef = state.orientation.rotate(forces.force);

    // Linear acceleration in ECEF: a = F/m
    Vec3 accel = force_ecef * inv_mass;

    // ========================================================================
    // Translational Motion (Symplectic Euler)
    // ========================================================================
    //
    // The key to symplectic Euler is the ordering of updates:
    //   1. Update velocity FIRST using current acceleration
    //   2. Update position using the NEW velocity
    //
    // This ordering ensures the method is symplectic (preserves phase space
    // volume) and provides bounded energy error for Hamiltonian systems.
    //
    // Standard Euler:     v_new = v + a*dt,  x_new = x + v*dt     (non-symplectic)
    // Symplectic Euler:   v_new = v + a*dt,  x_new = x + v_new*dt (symplectic)

    // Step 1: Update velocity using current acceleration
    state.velocity = state.velocity + accel * dt;

    // Step 2: Update position using NEW velocity (key for symplecticity!)
    state.position = state.position + state.velocity * dt;

    // ========================================================================
    // Rotational Motion (Symplectic Euler with Euler's Equations)
    // ========================================================================
    //
    // Apply the same symplectic ordering to rotational dynamics:
    //   1. Update angular velocity using Euler's equations
    //   2. Update orientation using the new angular velocity

    // Compute angular acceleration using Euler's equations
    Vec3 angular_accel = compute_angular_accel(
        state.inertia, inv_inertia,
        state.angular_velocity, forces.torque);

    // Step 1: Update angular velocity using current angular acceleration
    state.angular_velocity = state.angular_velocity + angular_accel * dt;

    // Step 2: Update orientation using NEW angular velocity
    // Use first-order quaternion integration: q_new = q + 0.5 * q * omega * dt
    // This maintains the symplectic structure for rotational motion.
    state.orientation = quaternion_utils::integrate(
        state.orientation, state.angular_velocity, dt);

    // ========================================================================
    // Store computed accelerations for debugging/output
    // ========================================================================

    state.acceleration = accel;
    state.angular_accel = angular_accel;

    // ========================================================================
    // Energy Tracking (Post-Integration)
    // ========================================================================

    if (track_energy_) {
        Real current_energy = compute_mechanical_energy(state);
        energy_drift_ = current_energy - initial_energy_;
    }
}

void SymplecticEulerIntegrator::reset_energy_tracking() {
    energy_initialized_ = false;
    initial_energy_ = 0.0;
    energy_drift_ = 0.0;
}

void SymplecticEulerIntegrator::reset() {
    reset_energy_tracking();
}

} // namespace jaguar::physics
