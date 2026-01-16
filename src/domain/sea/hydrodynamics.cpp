/**
 * @file hydrodynamics.cpp
 * @brief Hydrodynamics (MMG) model implementation
 *
 * Implements the Maneuvering Mathematical Group (MMG) model for
 * surface vessel hydrodynamics.
 *
 * Key components:
 * - Hull forces: Surge (X), Sway (Y), and Yaw (N)
 * - Propeller forces: Thrust and induced velocities
 * - Rudder forces: Lateral force and yaw moment
 *
 * MMG equations:
 * X = X_H + X_P + X_R (hull + propeller + rudder)
 * Y = Y_H + Y_P + Y_R
 * N = N_H + N_P + N_R
 *
 * Reference: Principles of Naval Architecture (SNAME), MMG Model (Ogawa)
 */

#include "jaguar/domain/sea.h"
#include "jaguar/environment/environment.h"
#include <cmath>
#include <algorithm>

namespace jaguar::domain::sea {

// ============================================================================
// HydrodynamicsModel Implementation
// ============================================================================

namespace {

/**
 * @brief Calculate hull hydrodynamic forces
 *
 * Hull forces depend on drift angle (beta) and yaw rate (r).
 * Using polynomial approximation of hydrodynamic derivatives.
 */
void compute_hull_forces(
    Real u, Real v, Real r,  // Velocities: surge, sway, yaw rate
    Real L,                   // Ship length
    Real rho,                 // Water density
    Real x_vv, Real x_rr,    // Surge coefficients
    Real y_v, Real y_r,      // Sway coefficients
    Real n_v, Real n_r,      // Yaw coefficients
    Real& X_H, Real& Y_H, Real& N_H)
{
    // Non-dimensional velocities
    Real U = std::sqrt(u * u + v * v);  // Total velocity
    if (U < 0.01) {
        X_H = Y_H = N_H = 0.0;
        return;
    }

    Real v_prime = v / U;
    Real r_prime = r * L / U;

    // Hull resistance coefficient (simplified)
    constexpr Real C_R = 0.005;  // Total resistance coefficient

    // Hull resistance (surge force)
    Real R_T = 0.5 * rho * C_R * L * L * U * U;

    // Cross-flow drag on hull due to drift
    Real C_D_cross = 0.8;  // Cross-flow drag coefficient
    Real A_L = L * 5.0;    // Lateral projected area (simplified)
    Real Y_cross = -0.5 * rho * C_D_cross * A_L * v * std::abs(v);

    // MMG hull forces (non-dimensional form to dimensional)
    // X_H = X_vv * v'^2 + X_rr * r'^2
    X_H = -R_T + 0.5 * rho * L * L * U * U * (x_vv * v_prime * v_prime + x_rr * r_prime * r_prime);

    // Y_H = Y_v * v' + Y_r * r'
    Y_H = Y_cross + 0.5 * rho * L * L * U * U * (y_v * v_prime + y_r * r_prime);

    // N_H = N_v * v' + N_r * r'
    N_H = 0.5 * rho * L * L * L * U * U * (n_v * v_prime + n_r * r_prime);
}

/**
 * @brief Calculate propeller thrust
 *
 * Using actuator disk theory with wake and thrust deduction.
 */
Real compute_propeller_thrust(
    Real rpm,
    Real diameter,
    [[maybe_unused]] Real pitch_ratio,  // For future K_T polynomial
    Real u,           // Advance velocity
    Real rho)
{
    // Propeller rotational speed (rev/s)
    Real n = rpm / 60.0;

    if (std::abs(n) < 0.01) {
        return 0.0;
    }

    // Wake fraction (simplified)
    constexpr Real w = 0.25;  // Typical for single screw
    Real V_A = u * (1.0 - w);  // Advance velocity at propeller

    // Advance coefficient
    Real J = (std::abs(n) > 0.01) ? V_A / (n * diameter) : 0.0;

    // Thrust coefficient (simplified Wageningen B-series approximation)
    // K_T = a0 + a1*J + a2*J^2
    Real a0 = 0.4;  // Simplified coefficients
    Real a1 = -0.3;
    Real a2 = -0.1;
    Real K_T = a0 + a1 * J + a2 * J * J;
    K_T = std::max(K_T, 0.0);

    // Thrust: T = K_T * rho * n^2 * D^4
    Real D4 = diameter * diameter * diameter * diameter;
    Real T = K_T * rho * n * std::abs(n) * D4;

    // Thrust deduction (hull-propeller interaction)
    constexpr Real t = 0.2;  // Thrust deduction factor
    T *= (1.0 - t);

    return T;
}

/**
 * @brief Calculate rudder forces
 *
 * Rudder modeled as a lifting surface with stall characteristics.
 */
void compute_rudder_forces(
    Real delta,       // Rudder angle (rad)
    Real u, Real v,   // Velocities
    Real area,        // Rudder area
    Real aspect_ratio,
    Real rho,
    Real x_R,         // Longitudinal position of rudder
    Real& X_R, Real& Y_R, Real& N_R)
{
    Real U = std::sqrt(u * u + v * v);
    if (U < 0.01) {
        X_R = Y_R = N_R = 0.0;
        return;
    }

    // Effective angle of attack on rudder
    Real beta = std::atan2(-v, u);  // Drift angle
    Real alpha_R = delta - beta;    // Effective rudder angle

    // Lift curve slope (wing theory)
    Real CL_alpha = constants::PI * aspect_ratio / (1.0 + aspect_ratio);

    // Stall modeling (simplified)
    constexpr Real STALL_ANGLE = 0.35;  // ~20 degrees
    Real CL;
    if (std::abs(alpha_R) < STALL_ANGLE) {
        CL = CL_alpha * alpha_R;
    } else {
        // Post-stall: reduced lift
        Real sign = (alpha_R >= 0.0) ? 1.0 : -1.0;
        CL = sign * CL_alpha * STALL_ANGLE * 0.8;
    }

    // Induced drag
    Real e = 0.85;  // Oswald efficiency
    Real CD_i = CL * CL / (constants::PI * aspect_ratio * e);

    // Profile drag
    constexpr Real CD_0 = 0.01;
    Real CD = CD_0 + CD_i;

    // Dynamic pressure
    Real q = 0.5 * rho * U * U;

    // Rudder lift and drag
    Real L_R = q * area * CL;
    Real D_R = q * area * CD;

    // Transform to ship coordinates
    // Lift is perpendicular to flow, drag is parallel
    Y_R = -L_R * std::cos(beta) - D_R * std::sin(beta);
    X_R = -L_R * std::sin(beta) + D_R * std::cos(beta);

    // Yaw moment from rudder force
    N_R = Y_R * x_R;
}

} // anonymous namespace

HydrodynamicsModel::HydrodynamicsModel() = default;
HydrodynamicsModel::~HydrodynamicsModel() = default;

void HydrodynamicsModel::compute_forces(
    const physics::EntityState& state,
    const environment::Environment& env,
    [[maybe_unused]] Real dt,
    physics::EntityForces& out_forces)
{
    if (!enabled_) {
        draft_ = 0.0;
        heel_ = 0.0;
        trim_ = 0.0;
        return;
    }

    // Check if in water
    if (!env.over_water) {
        draft_ = 0.0;
        return;
    }

    Real rho = constants::RHO_WATER;

    // Get velocities in body frame
    Vec3 v_body = state.orientation.conjugate().rotate(state.velocity);
    Real u = v_body.x;   // Surge velocity
    Real v = v_body.y;   // Sway velocity
    Real r = state.angular_velocity.z;  // Yaw rate

    // Ship length (estimate from mass - simplified)
    // Assume L = k * m^(1/3) for displacement ships
    Real L = 50.0;  // Default ship length (should be configurable)

    // Calculate hull forces
    Real X_H, Y_H, N_H;
    compute_hull_forces(u, v, r, L, rho,
                       x_vv_, x_rr_, y_v_, y_r_, n_v_, n_r_,
                       X_H, Y_H, N_H);

    // Calculate propeller thrust
    Real X_P = compute_propeller_thrust(propeller_rpm_, propeller_diameter_,
                                        propeller_pitch_ratio_, u, rho);

    // Calculate rudder forces
    Real X_R, Y_R, N_R;
    Real x_R = -L * 0.45;  // Rudder position (near stern)
    compute_rudder_forces(rudder_angle_, u, v,
                         rudder_area_, rudder_aspect_ratio_, rho, x_R,
                         X_R, Y_R, N_R);

    // Total forces
    Vec3 force;
    force.x = X_H + X_P + X_R;
    force.y = Y_H + Y_R;
    force.z = 0.0;  // Vertical forces handled by buoyancy model

    Vec3 torque;
    torque.x = 0.0;  // Roll moment (from other sources)
    torque.y = 0.0;  // Pitch moment
    torque.z = N_H + N_R;  // Yaw moment

    out_forces.add_force(force);
    out_forces.add_torque(torque);

    // Update draft (simplified - from buoyancy equilibrium)
    // In reality, this comes from solving the hydrostatic balance
    Real water_surface = env.ocean.surface_elevation;
    draft_ = water_surface - env.altitude;
    if (draft_ < 0.0) draft_ = 0.0;

    // Get heel and trim from orientation
    Real roll, pitch, yaw;
    state.orientation.to_euler(roll, pitch, yaw);
    heel_ = roll;
    trim_ = pitch;
}

Real HydrodynamicsModel::get_buoyancy() const {
    // Buoyancy is handled by BuoyancyModel
    // This returns an approximate value based on draft
    Real V_displaced = draft_ * 50.0 * 10.0;  // Simplified: L*B*T
    return constants::RHO_WATER * constants::G0 * V_displaced;
}

void HydrodynamicsModel::set_hull_coefficients(
    Real x_vv, Real x_rr,
    Real y_v, Real y_r,
    Real n_v, Real n_r)
{
    x_vv_ = x_vv;
    x_rr_ = x_rr;
    y_v_ = y_v;
    y_r_ = y_r;
    n_v_ = n_v;
    n_r_ = n_r;
}

void HydrodynamicsModel::set_rudder_parameters(Real area, Real aspect_ratio) {
    rudder_area_ = std::max(area, 0.1);
    rudder_aspect_ratio_ = std::max(aspect_ratio, 0.5);
}

void HydrodynamicsModel::set_propeller_parameters(Real diameter, Real pitch_ratio) {
    propeller_diameter_ = std::max(diameter, 0.1);
    propeller_pitch_ratio_ = std::max(pitch_ratio, 0.5);
}

} // namespace jaguar::domain::sea
