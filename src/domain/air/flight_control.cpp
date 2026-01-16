/**
 * @file flight_control.cpp
 * @brief Flight control system implementation
 *
 * Implements a basic flight control system (FCS) that converts
 * pilot/autopilot commands to control surface deflections.
 *
 * Features:
 * - Control input to surface deflection mapping
 * - Rate limiting for realistic actuator dynamics
 * - Optional position limiting
 *
 * Reference: Aircraft Control and Simulation (Stevens & Lewis)
 */

#include "jaguar/domain/air.h"
#include <cmath>
#include <algorithm>

namespace jaguar::domain::air {

// ============================================================================
// FlightControlSystem Implementation
// ============================================================================

namespace {

/**
 * @brief Apply rate limiting to a value change
 *
 * @param current Current value
 * @param target Target value
 * @param max_rate Maximum rate of change per second
 * @param dt Time step in seconds
 * @return Rate-limited new value
 */
Real apply_rate_limit(Real current, Real target, Real max_rate, Real dt) {
    Real delta = target - current;
    Real max_delta = max_rate * dt;

    if (std::abs(delta) <= max_delta) {
        return target;
    }

    return current + std::copysign(max_delta, delta);
}

/**
 * @brief Clamp value to range
 */
Real clamp_to_range(Real value, Real min_val, Real max_val) {
    return std::clamp(value, min_val, max_val);
}

} // anonymous namespace

FlightControlSystem::ControlOutputs FlightControlSystem::process(
    const ControlInputs& inputs,
    Real dt)
{
    ControlOutputs outputs;

    // Default rate limits (deg/s) - typical for fighter aircraft
    constexpr Real ELEV_RATE_LIMIT = 40.0;  // deg/s
    constexpr Real AIL_RATE_LIMIT = 80.0;   // deg/s (typically faster)
    constexpr Real RUD_RATE_LIMIT = 50.0;   // deg/s

    // Convert commands (-1 to +1) to target surface deflections
    // Positive pitch command (stick back) = positive elevator (nose up)
    Real target_elevator = inputs.pitch_cmd * elev_max_;
    if (inputs.pitch_cmd < 0.0) {
        target_elevator = inputs.pitch_cmd * (-elev_min_);  // Scale by negative limit
    }

    // Positive roll command (stick right) = positive aileron
    // (right aileron up, left aileron down for right roll)
    Real target_aileron = inputs.roll_cmd * ail_max_;
    if (inputs.roll_cmd < 0.0) {
        target_aileron = inputs.roll_cmd * (-ail_min_);
    }

    // Positive yaw command (right pedal) = positive rudder (nose right)
    Real target_rudder = inputs.yaw_cmd * rud_max_;
    if (inputs.yaw_cmd < 0.0) {
        target_rudder = inputs.yaw_cmd * (-rud_min_);
    }

    // Apply rate limits for realistic actuator dynamics
    // (Using static to maintain state between calls - in production,
    // these would be member variables)
    static Real current_elevator = 0.0;
    static Real current_aileron = 0.0;
    static Real current_rudder = 0.0;

    current_elevator = apply_rate_limit(current_elevator, target_elevator, ELEV_RATE_LIMIT, dt);
    current_aileron = apply_rate_limit(current_aileron, target_aileron, AIL_RATE_LIMIT, dt);
    current_rudder = apply_rate_limit(current_rudder, target_rudder, RUD_RATE_LIMIT, dt);

    // Apply position limits
    outputs.elevator_deg = clamp_to_range(current_elevator, elev_min_, elev_max_);
    outputs.aileron_deg = clamp_to_range(current_aileron, ail_min_, ail_max_);
    outputs.rudder_deg = clamp_to_range(current_rudder, rud_min_, rud_max_);

    return outputs;
}

void FlightControlSystem::set_elevator_range(Real min_deg, Real max_deg) {
    elev_min_ = min_deg;
    elev_max_ = max_deg;
}

void FlightControlSystem::set_aileron_range(Real min_deg, Real max_deg) {
    ail_min_ = min_deg;
    ail_max_ = max_deg;
}

void FlightControlSystem::set_rudder_range(Real min_deg, Real max_deg) {
    rud_min_ = min_deg;
    rud_max_ = max_deg;
}

} // namespace jaguar::domain::air
