/**
 * @file boris.cpp
 * @brief Boris integrator implementation for charged particle dynamics
 *
 * Implements the Boris algorithm (1970) for charged particle motion in
 * electromagnetic fields. This is the standard algorithm used in particle-in-cell
 * (PIC) codes and plasma physics simulations.
 *
 * The Boris algorithm separates the Lorentz force into electric and magnetic
 * components and handles them in a specific order that preserves the symplectic
 * structure of the equations of motion.
 *
 * References:
 * - Boris, J. P. (1970), "Relativistic plasma simulation - optimization of a
 *   hybrid code", Proceedings of 4th Conference on Numerical Simulation of Plasmas
 * - Birdsall, C. K. & Langdon, A. B. (1991), "Plasma Physics via Computer Simulation"
 */

#include "jaguar/physics/integrators/boris.h"
#include <cmath>

namespace jaguar::physics {

BorisIntegrator::BorisIntegrator() = default;

void BorisIntegrator::reset() {
    reset_energy_tracking();
    gyrofrequency_ = 0.0;
    gyroradius_ = 0.0;
    magnetic_moment_ = 0.0;
    is_magnetized_ = false;
}

void BorisIntegrator::reset_energy_tracking() {
    energy_initialized_ = false;
    initial_energy_ = 0.0;
    energy_drift_ = 0.0;
}

Real BorisIntegrator::compute_mechanical_energy(const EntityState& state) const {
    // Kinetic energy: (1/2) * m * |v|²
    Real kinetic_trans = 0.5 * state.mass * state.velocity.length_squared();

    // Rotational kinetic energy: (1/2) * ω · (I * ω)
    Vec3 I_omega = state.inertia * state.angular_velocity;
    Real kinetic_rot = 0.5 * state.angular_velocity.dot(I_omega);

    return kinetic_trans + kinetic_rot;
}

Vec3 BorisIntegrator::compute_angular_accel(
    const Mat3x3& inertia, const Mat3x3& inv_inertia,
    const Vec3& omega, const Vec3& torque) const {
    // Euler's equations: I * ω̇ = τ - ω × (I * ω)
    Vec3 I_omega = inertia * omega;
    Vec3 gyroscopic = omega.cross(I_omega);
    return inv_inertia * (torque - gyroscopic);
}

Vec3 BorisIntegrator::boris_rotation(
    const Vec3& v, const Vec3& B, Real q_over_m, Real dt) const {
    // Boris rotation formula:
    //
    // The rotation is performed in two steps using intermediate vectors t and s:
    //   t = (q/m) * B * dt/2      (half-angle tangent vector)
    //   s = 2*t / (1 + |t|²)      (full-angle sine vector)
    //
    // The velocity rotation:
    //   v' = v + v × t            (half rotation)
    //   v+ = v + v' × s           (full rotation using v')
    //
    // This exactly preserves |v| and gives the correct gyration angle.

    // Compute the half-angle tangent vector
    Vec3 t = B * (q_over_m * dt * 0.5);

    // Compute the full-angle sine vector
    Real t_mag_sq = t.length_squared();
    Vec3 s = t * (2.0 / (1.0 + t_mag_sq));

    // First half rotation: v' = v + v × t
    Vec3 v_prime = v + v.cross(t);

    // Full rotation: v+ = v + v' × s
    Vec3 v_plus = v + v_prime.cross(s);

    return v_plus;
}

void BorisIntegrator::update_diagnostics(
    const EntityState& state, const Vec3& B, Real charge, Real dt) {

    Real B_mag = std::sqrt(B.length_squared());

    if (state.mass > 1e-10 && std::abs(charge) > 1e-20 && B_mag > 1e-15) {
        // Gyrofrequency (cyclotron frequency): ω_c = |q|*|B| / m
        gyrofrequency_ = std::abs(charge) * B_mag / state.mass;

        // Velocity component perpendicular to B
        Vec3 B_unit = B * (1.0 / B_mag);
        Real v_parallel = state.velocity.dot(B_unit);
        Vec3 v_perp_vec = state.velocity - B_unit * v_parallel;
        Real v_perp = std::sqrt(v_perp_vec.length_squared());

        // Gyroradius (Larmor radius): r_L = m * v_perp / (|q| * |B|)
        if (std::abs(charge) * B_mag > 1e-20) {
            gyroradius_ = state.mass * v_perp / (std::abs(charge) * B_mag);
        } else {
            gyroradius_ = 0.0;
        }

        // Magnetic moment (first adiabatic invariant): μ = m * v_perp² / (2 * |B|)
        if (B_mag > 1e-15) {
            magnetic_moment_ = state.mass * v_perp * v_perp / (2.0 * B_mag);
        } else {
            magnetic_moment_ = 0.0;
        }

        // Check if particle is magnetized (gyroperiod >> dt)
        // Particle is magnetized if ω_c * dt > 0.1 (10+ steps per gyration)
        is_magnetized_ = (gyrofrequency_ * dt) > 0.1;
    } else {
        gyrofrequency_ = 0.0;
        gyroradius_ = 0.0;
        magnetic_moment_ = 0.0;
        is_magnetized_ = false;
    }
}

void BorisIntegrator::integrate(EntityState& state, const EntityForces& forces, Real dt) {
    // Check if we have background fields configured
    // If so, create an ElectromagneticForces structure and use Boris algorithm
    Real B_mag_sq = background_B_.length_squared();

    if (B_mag_sq > 1e-30) {
        // We have a magnetic field - use Boris algorithm
        // Assume unit charge for particles in magnetic field if not specified otherwise
        ElectromagneticForces em_forces;
        em_forces.force = forces.force;
        em_forces.torque = forces.torque;
        em_forces.electric_field = background_E_;
        em_forces.magnetic_field = background_B_;
        em_forces.charge = 1.0;  // Default to unit charge
        integrate_charged(state, em_forces, dt);
    } else {
        // Fall back to symplectic Euler for neutral particles
        if (state.mass < 1e-10) return;

        Real inv_mass = 1.0 / state.mass;

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

        // Transform forces from body to ECEF frame
        Vec3 force_ecef = state.orientation.rotate(forces.force);
        Vec3 accel = force_ecef * inv_mass;

        // Symplectic Euler: update velocity first, then position
        state.velocity = state.velocity + accel * dt;
        state.position = state.position + state.velocity * dt;

        // Angular motion
        Vec3 angular_accel = compute_angular_accel(
            state.inertia, inv_inertia, state.angular_velocity, forces.torque);
        state.angular_velocity = state.angular_velocity + angular_accel * dt;
        state.orientation = quaternion_utils::integrate(
            state.orientation, state.angular_velocity, dt);

        state.acceleration = accel;
        state.angular_accel = angular_accel;
    }
}

void BorisIntegrator::integrate_charged(
    EntityState& state, const ElectromagneticForces& em_forces, Real dt) {

    if (state.mass < 1e-10) return;

    Real inv_mass = 1.0 / state.mass;
    Real q_over_m = em_forces.charge / state.mass;

    // Total electromagnetic fields (background + local)
    Vec3 E = em_forces.electric_field + background_E_;
    Vec3 B = em_forces.magnetic_field + background_B_;

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
    // Boris Algorithm for Translational Motion
    // ========================================================================
    //
    // The Boris algorithm for the Lorentz force:
    //   F = q*(E + v × B)
    //
    // Step 1: Half electric acceleration
    //   v⁻ = v^n + (q*E/m) * dt/2
    //
    // Step 2: Magnetic rotation (preserves |v|)
    //   v⁺ = boris_rotation(v⁻, B, q/m, dt)
    //
    // Step 3: Half electric acceleration
    //   v^(n+1) = v⁺ + (q*E/m) * dt/2
    //
    // Step 4: Position update (leapfrog)
    //   x^(n+1) = x^n + v^(n+1) * dt

    // Transform any mechanical forces from body to ECEF frame
    Vec3 mech_force_ecef = state.orientation.rotate(em_forces.force);
    Vec3 mech_accel = mech_force_ecef * inv_mass;

    // Electric field acceleration: a_E = q*E/m
    Vec3 E_accel = E * q_over_m;

    // Total non-magnetic acceleration
    Vec3 total_E_accel = E_accel + mech_accel;

    // Step 1: Half electric push
    Vec3 v_minus = state.velocity + total_E_accel * (0.5 * dt);

    // Step 2: Magnetic rotation
    Vec3 v_plus = boris_rotation(v_minus, B, q_over_m, dt);

    // Step 3: Second half electric push
    state.velocity = v_plus + total_E_accel * (0.5 * dt);

    // Step 4: Position update
    state.position = state.position + state.velocity * dt;

    // Store acceleration for output
    // Total Lorentz acceleration: a = (q/m) * (E + v × B) + F_mech/m
    Vec3 lorentz_accel = E_accel + state.velocity.cross(B) * q_over_m;
    state.acceleration = lorentz_accel + mech_accel;

    // ========================================================================
    // Rotational Motion (Standard Symplectic Euler)
    // ========================================================================
    //
    // Rotational dynamics typically don't couple with electromagnetic fields
    // for macroscopic objects, so we use standard symplectic Euler.

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

    Vec3 angular_accel = compute_angular_accel(
        state.inertia, inv_inertia, state.angular_velocity, em_forces.torque);

    state.angular_velocity = state.angular_velocity + angular_accel * dt;
    state.orientation = quaternion_utils::integrate(
        state.orientation, state.angular_velocity, dt);
    state.angular_accel = angular_accel;

    // ========================================================================
    // Update Diagnostics
    // ========================================================================

    update_diagnostics(state, B, em_forces.charge, dt);

    // Energy tracking (post-integration)
    if (track_energy_) {
        Real current_energy = compute_mechanical_energy(state);
        energy_drift_ = current_energy - initial_energy_;
    }
}

} // namespace jaguar::physics
