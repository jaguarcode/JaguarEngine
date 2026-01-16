/**
 * @file propulsion.cpp
 * @brief Propulsion model implementation
 *
 * Implements a generic turbofan/turbojet engine model with:
 * - Thrust variation with altitude and Mach number
 * - Fuel consumption modeling
 * - Engine start/stop logic
 * - Thrust vectoring support (optional)
 *
 * Reference: Gas Turbine Theory (Saravanamuttoo), Aircraft Propulsion (Farokhi)
 */

#include "jaguar/domain/air.h"
#include "jaguar/environment/environment.h"
#include <cmath>
#include <algorithm>

namespace jaguar::domain::air {

// ============================================================================
// PropulsionModel Implementation
// ============================================================================

namespace {

/**
 * @brief Simple altitude-Mach thrust correction factor
 *
 * Models the reduction in thrust with altitude due to lower air density,
 * and ram drag effects at high Mach numbers.
 *
 * @param altitude_m Altitude in meters
 * @param mach Mach number
 * @param density Current air density (kg/m³)
 * @return Thrust correction factor (0-1+, typically 0.1 to 1.2)
 */
Real compute_thrust_factor(Real altitude_m, Real mach, Real density) {
    // Sea level reference density
    constexpr Real RHO_SL = 1.225;  // kg/m³

    // Density ratio effect (primary altitude correction)
    Real sigma = density / RHO_SL;

    // Ram effect (thrust increases slightly with Mach at low Mach)
    // Then decreases at high Mach due to inlet losses
    Real ram_factor = 1.0;
    if (mach < 0.8) {
        // Ram recovery increases available thrust
        ram_factor = 1.0 + 0.15 * mach;
    } else if (mach < 1.2) {
        // Transonic: slight reduction
        ram_factor = 1.12 - 0.1 * (mach - 0.8);
    } else {
        // Supersonic: significant reduction
        ram_factor = std::max(0.5, 1.08 - 0.3 * (mach - 1.2));
    }

    // Altitude factor (using power law approximation)
    // Full thrust at low altitude, reduces with decreasing density
    // Using sigma^0.7 as typical turbofan characteristic
    Real alt_factor = std::pow(sigma, 0.7);

    // Limit very high altitude degradation (flameout protection)
    alt_factor = std::max(alt_factor, 0.05);

    // Very low altitude slightly reduces efficiency due to high density
    if (altitude_m < 0.0) {
        alt_factor *= (1.0 - 0.01 * std::abs(altitude_m) / 1000.0);
        alt_factor = std::max(alt_factor, 0.9);
    }

    return alt_factor * ram_factor;
}

/**
 * @brief Compute specific fuel consumption variation
 *
 * SFC varies with altitude and throttle setting.
 * Lower at cruise conditions, higher at low altitude and idle.
 *
 * @param throttle Throttle setting (0-1)
 * @param altitude_m Altitude in meters
 * @param mach Mach number
 * @return SFC multiplier
 */
Real compute_sfc_factor(Real throttle, Real altitude_m, Real mach) {
    // Base SFC at optimal cruise conditions
    Real sfc_factor = 1.0;

    // Throttle effect: SFC increases at very low and very high throttle
    if (throttle < 0.3) {
        // Inefficient at low throttle
        sfc_factor *= (1.0 + 0.5 * (0.3 - throttle));
    } else if (throttle > 0.9) {
        // Afterburner or high thrust region (less efficient)
        sfc_factor *= (1.0 + 0.3 * (throttle - 0.9) / 0.1);
    }

    // Altitude effect: more efficient at cruise altitudes (lower temp, optimized)
    // Typical cruise at 10-12 km
    constexpr Real OPTIMAL_ALT = 11000.0;  // m
    Real alt_diff = std::abs(altitude_m - OPTIMAL_ALT);
    if (alt_diff > 3000.0) {
        sfc_factor *= (1.0 + 0.1 * (alt_diff - 3000.0) / 10000.0);
    }

    // Mach effect: optimized around cruise Mach
    constexpr Real OPTIMAL_MACH = 0.8;
    Real mach_diff = std::abs(mach - OPTIMAL_MACH);
    if (mach_diff > 0.2) {
        sfc_factor *= (1.0 + 0.2 * (mach_diff - 0.2));
    }

    return std::clamp(sfc_factor, 0.8, 2.0);
}

} // anonymous namespace

PropulsionModel::PropulsionModel() = default;
PropulsionModel::~PropulsionModel() = default;

void PropulsionModel::compute_forces(
    const physics::EntityState& state,
    const environment::Environment& env,
    Real dt,
    physics::EntityForces& out_forces)
{
    if (!enabled_ || !running_) {
        thrust_ = 0.0;
        fuel_flow_ = 0.0;
        return;
    }

    // Check for fuel
    if (fuel_remaining_ <= 0.0) {
        stop();
        thrust_ = 0.0;
        fuel_flow_ = 0.0;
        return;
    }

    // Get velocity in body frame for Mach calculation
    Vec3 v_body = state.orientation.conjugate().rotate(state.velocity);
    Real V_total = v_body.length();

    // Compute Mach number
    Real speed_of_sound = env.atmosphere.speed_of_sound;
    Real mach = (speed_of_sound > 0.0) ? V_total / speed_of_sound : 0.0;

    // Get altitude and density
    Real altitude = env.altitude;
    Real density = env.atmosphere.density;

    // Compute thrust correction factors
    Real thrust_factor = compute_thrust_factor(altitude, mach, density);

    // Compute actual thrust
    // Clamp throttle to valid range
    Real effective_throttle = std::clamp(throttle_, 0.0, 1.0);

    // Thrust response (simple first-order lag could be added)
    thrust_ = max_thrust_ * effective_throttle * thrust_factor;

    // Compute fuel consumption
    Real sfc_factor = compute_sfc_factor(effective_throttle, altitude, mach);
    fuel_flow_ = sfc_ * thrust_ * sfc_factor;

    // Update fuel remaining
    fuel_remaining_ -= fuel_flow_ * dt;
    fuel_remaining_ = std::max(fuel_remaining_, 0.0);

    // Apply thrust force in body X direction (forward)
    // Thrust vectoring could be added here
    Vec3 thrust_force{thrust_, 0.0, 0.0};

    // If we had thrust vectoring angles, we'd rotate here:
    // thrust_force = rotate_by_angles(thrust_force, pitch_angle, yaw_angle);

    out_forces.add_force(thrust_force);

    // Engine gyroscopic effects could be added here as torque
    // For now, we don't model engine spool angular momentum
}

void PropulsionModel::start() {
    if (fuel_remaining_ > 0.0) {
        running_ = true;
    }
}

void PropulsionModel::stop() {
    running_ = false;
    thrust_ = 0.0;
    fuel_flow_ = 0.0;
}

} // namespace jaguar::domain::air
