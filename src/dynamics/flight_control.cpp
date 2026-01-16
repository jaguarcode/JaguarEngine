/**
 * @file flight_control.cpp
 * @brief Implementation of Flight Control System
 * @version 0.5.0
 */

#include "jaguar/dynamics/flight_control.h"
#include <algorithm>
#include <cmath>

namespace jaguar {
namespace dynamics {

//=============================================================================
// Constants
//=============================================================================

namespace {
    constexpr double DEG_TO_RAD = M_PI / 180.0;
    constexpr double RAD_TO_DEG = 180.0 / M_PI;
    constexpr double GRAVITY = 9.81;
    constexpr double TWO_PI = 2.0 * M_PI;
}

//=============================================================================
// PIDController Implementation
//=============================================================================

PIDController::PIDController(const PIDGains& gains)
    : gains_(gains) {}

double PIDController::update(double error, double dt, double speed) {
    if (dt <= 0.0) return 0.0;

    // Apply gain scheduling based on speed
    double kp = gains_.kp * schedule_gain(speed);

    // Proportional term
    double p_term = kp * error;

    // Integral term with anti-windup
    integral_ += error * dt;
    integral_ = std::clamp(integral_, -gains_.integral_limit, gains_.integral_limit);
    double i_term = gains_.ki * integral_;

    // Derivative term (on error, not measurement)
    double d_term = 0.0;
    if (!first_update_) {
        double derivative = (error - prev_error_) / dt;
        d_term = gains_.kd * derivative;
    }
    first_update_ = false;
    prev_error_ = error;

    // Sum and limit output
    double output = p_term + i_term + d_term;
    output = std::clamp(output, -gains_.output_limit, gains_.output_limit);

    return output;
}

void PIDController::reset() {
    integral_ = 0.0;
    prev_error_ = 0.0;
    first_update_ = true;
}

void PIDController::set_gains(const PIDGains& gains) {
    gains_ = gains;
}

double PIDController::schedule_gain(double speed) const {
    if (speed <= gains_.schedule_speed_min) {
        return gains_.kp_schedule_min;
    }
    if (speed >= gains_.schedule_speed_max) {
        return gains_.kp_schedule_max;
    }

    // Linear interpolation
    double t = (speed - gains_.schedule_speed_min) /
               (gains_.schedule_speed_max - gains_.schedule_speed_min);
    return gains_.kp_schedule_min + t * (gains_.kp_schedule_max - gains_.kp_schedule_min);
}

//=============================================================================
// SASController Implementation
//=============================================================================

SASController::SASController(const ControlLawConfig& config)
    : config_(config) {
    pitch_rate_controller_.set_gains(config.pitch_rate);
    roll_rate_controller_.set_gains(config.roll_rate);
    yaw_rate_controller_.set_gains(config.yaw_rate);
}

SurfacePositions SASController::calculate(const ControlInput& input,
                                          const FlightState& state,
                                          double dt) {
    SurfacePositions surfaces;

    // SAS provides rate damping - pilot commands are direct + rate feedback
    // Command = stick input - rate feedback

    // Pitch axis
    double pitch_command = input.pitch * config_.pitch_stick_gain;
    double pitch_rate_fb = pitch_rate_controller_.update(
        -state.pitch_rate, dt, state.airspeed);  // Negative rate for damping

    double elevator_cmd = pitch_command + pitch_rate_fb + input.pitch_trim;
    elevator_cmd *= config_.elevator_max;  // Scale to degrees
    elevator_cmd = std::clamp(elevator_cmd, config_.elevator_min, config_.elevator_max);
    surfaces.elevator = rate_limit(elevator_cmd, prev_elevator_,
                                   config_.elevator_rate, dt);
    prev_elevator_ = surfaces.elevator;

    // Roll axis
    double roll_command = input.roll * config_.roll_stick_gain;
    double roll_rate_fb = roll_rate_controller_.update(
        -state.roll_rate, dt, state.airspeed);

    double aileron_cmd = roll_command + roll_rate_fb + input.roll_trim;
    aileron_cmd *= config_.aileron_max;
    aileron_cmd = std::clamp(aileron_cmd, -config_.aileron_max, config_.aileron_max);
    double aileron_limited = rate_limit(aileron_cmd, prev_aileron_,
                                        config_.aileron_rate, dt);
    prev_aileron_ = aileron_limited;
    surfaces.aileron_left = aileron_limited;
    surfaces.aileron_right = -aileron_limited;

    // Yaw axis
    double yaw_command = input.yaw * config_.yaw_stick_gain;
    double yaw_rate_fb = yaw_rate_controller_.update(
        -state.yaw_rate, dt, state.airspeed);

    // Auto-coordination: add aileron-rudder interconnect
    double auto_rudder = 0.0;
    if (config_.auto_rudder) {
        auto_rudder = aileron_limited * config_.aileron_rudder_interconnect;
    }

    double rudder_cmd = yaw_command + yaw_rate_fb + auto_rudder + input.yaw_trim;
    rudder_cmd *= config_.rudder_max;
    rudder_cmd = std::clamp(rudder_cmd, -config_.rudder_max, config_.rudder_max);
    surfaces.rudder = rate_limit(rudder_cmd, prev_rudder_,
                                 config_.rudder_rate, dt);
    prev_rudder_ = surfaces.rudder;

    // Pass through throttle
    surfaces.throttle_position = input.throttle;

    // Flaps and speedbrake
    surfaces.flap_left = input.flaps * 40.0;   // 40 degrees max
    surfaces.flap_right = input.flaps * 40.0;
    surfaces.spoiler_left = input.speedbrake * 60.0;
    surfaces.spoiler_right = input.speedbrake * 60.0;

    return surfaces;
}

void SASController::reset() {
    pitch_rate_controller_.reset();
    roll_rate_controller_.reset();
    yaw_rate_controller_.reset();
    prev_elevator_ = 0.0;
    prev_aileron_ = 0.0;
    prev_rudder_ = 0.0;
}

void SASController::set_config(const ControlLawConfig& config) {
    config_ = config;
    pitch_rate_controller_.set_gains(config.pitch_rate);
    roll_rate_controller_.set_gains(config.roll_rate);
    yaw_rate_controller_.set_gains(config.yaw_rate);
}

double SASController::rate_limit(double current, double previous,
                                 double max_rate, double dt) {
    double max_change = max_rate * dt;
    double delta = current - previous;
    delta = std::clamp(delta, -max_change, max_change);
    return previous + delta;
}

//=============================================================================
// CASController Implementation
//=============================================================================

CASController::CASController(const ControlLawConfig& config)
    : config_(config) {
    // Inner loop (rate) controllers
    pitch_rate_controller_.set_gains(config.pitch_rate);
    roll_rate_controller_.set_gains(config.roll_rate);
    yaw_rate_controller_.set_gains(config.yaw_rate);

    // Outer loop (attitude) controllers
    pitch_attitude_controller_.set_gains(config.pitch_attitude);
    roll_attitude_controller_.set_gains(config.roll_attitude);
}

SurfacePositions CASController::calculate(const ControlInput& input,
                                          const FlightState& state,
                                          double dt) {
    SurfacePositions surfaces;

    // Update attitude reference based on stick position
    update_attitude_reference(input, state);

    // Pitch axis - RCAH (Rate Command Attitude Hold)
    double pitch_rate_cmd;
    if (pitch_stick_centered_ && attitude_hold_enabled_) {
        // Attitude hold mode - command rate to return to reference
        double pitch_error = reference_pitch_ - state.pitch;
        pitch_rate_cmd = pitch_attitude_controller_.update(pitch_error, dt, state.airspeed);
    } else {
        // Rate command mode - stick commands rate directly
        pitch_rate_cmd = input.pitch * config_.pitch_stick_gain * 0.5;  // 0.5 rad/s max
    }

    double pitch_rate_error = pitch_rate_cmd - state.pitch_rate;
    double elevator_cmd = pitch_rate_controller_.update(pitch_rate_error, dt, state.airspeed);
    elevator_cmd += input.pitch_trim;
    elevator_cmd *= config_.elevator_max;
    elevator_cmd = std::clamp(elevator_cmd, config_.elevator_min, config_.elevator_max);
    surfaces.elevator = rate_limit(elevator_cmd, prev_elevator_,
                                   config_.elevator_rate, dt);
    prev_elevator_ = surfaces.elevator;

    // Roll axis - RCAH
    double roll_rate_cmd;
    if (roll_stick_centered_ && attitude_hold_enabled_) {
        // Attitude hold - maintain wings level or reference bank
        double roll_error = reference_roll_ - state.roll;
        roll_rate_cmd = roll_attitude_controller_.update(roll_error, dt, state.airspeed);
    } else {
        // Rate command - stick commands roll rate
        roll_rate_cmd = input.roll * config_.roll_stick_gain * 3.0;  // 3 rad/s max
    }

    double roll_rate_error = roll_rate_cmd - state.roll_rate;
    double aileron_cmd = roll_rate_controller_.update(roll_rate_error, dt, state.airspeed);
    aileron_cmd += input.roll_trim;
    aileron_cmd *= config_.aileron_max;
    aileron_cmd = std::clamp(aileron_cmd, -config_.aileron_max, config_.aileron_max);
    double aileron_limited = rate_limit(aileron_cmd, prev_aileron_,
                                        config_.aileron_rate, dt);
    prev_aileron_ = aileron_limited;
    surfaces.aileron_left = aileron_limited;
    surfaces.aileron_right = -aileron_limited;

    // Yaw axis - turn coordination + direct yaw
    double yaw_rate_cmd = input.yaw * config_.yaw_stick_gain * 0.5;

    // Turn coordination
    if (config_.auto_rudder && std::abs(state.roll) > 0.05) {
        // In a bank, coordinate the turn
        double coordinated_yaw = std::tan(state.roll) * GRAVITY / std::max(state.airspeed, 50.0);
        yaw_rate_cmd += coordinated_yaw * config_.turn_coordination_gain;
    }

    double yaw_rate_error = yaw_rate_cmd - state.yaw_rate;
    double rudder_cmd = yaw_rate_controller_.update(yaw_rate_error, dt, state.airspeed);
    rudder_cmd += input.yaw_trim;

    // ARI - aileron/rudder interconnect to reduce adverse yaw
    rudder_cmd += aileron_limited * config_.aileron_rudder_interconnect;

    rudder_cmd *= config_.rudder_max;
    rudder_cmd = std::clamp(rudder_cmd, -config_.rudder_max, config_.rudder_max);
    surfaces.rudder = rate_limit(rudder_cmd, prev_rudder_,
                                 config_.rudder_rate, dt);
    prev_rudder_ = surfaces.rudder;

    // Pass through controls
    surfaces.throttle_position = input.throttle;
    surfaces.flap_left = input.flaps * 40.0;
    surfaces.flap_right = input.flaps * 40.0;
    surfaces.spoiler_left = input.speedbrake * 60.0;
    surfaces.spoiler_right = input.speedbrake * 60.0;

    return surfaces;
}

void CASController::reset() {
    pitch_rate_controller_.reset();
    roll_rate_controller_.reset();
    yaw_rate_controller_.reset();
    pitch_attitude_controller_.reset();
    roll_attitude_controller_.reset();

    reference_pitch_ = 0.0;
    reference_roll_ = 0.0;
    pitch_stick_centered_ = true;
    roll_stick_centered_ = true;

    prev_elevator_ = 0.0;
    prev_aileron_ = 0.0;
    prev_rudder_ = 0.0;
}

void CASController::set_config(const ControlLawConfig& config) {
    config_ = config;
    pitch_rate_controller_.set_gains(config.pitch_rate);
    roll_rate_controller_.set_gains(config.roll_rate);
    yaw_rate_controller_.set_gains(config.yaw_rate);
    pitch_attitude_controller_.set_gains(config.pitch_attitude);
    roll_attitude_controller_.set_gains(config.roll_attitude);
}

double CASController::rate_limit(double current, double previous,
                                 double max_rate, double dt) {
    double max_change = max_rate * dt;
    double delta = current - previous;
    delta = std::clamp(delta, -max_change, max_change);
    return previous + delta;
}

void CASController::update_attitude_reference(const ControlInput& input,
                                              const FlightState& state) {
    bool was_pitch_centered = pitch_stick_centered_;
    bool was_roll_centered = roll_stick_centered_;

    pitch_stick_centered_ = std::abs(input.pitch) < STICK_DEADZONE;
    roll_stick_centered_ = std::abs(input.roll) < STICK_DEADZONE;

    // Capture reference when stick returns to center
    if (pitch_stick_centered_ && !was_pitch_centered) {
        reference_pitch_ = state.pitch;
        pitch_attitude_controller_.reset();
    }

    if (roll_stick_centered_ && !was_roll_centered) {
        reference_roll_ = state.roll;
        roll_attitude_controller_.reset();
    }
}

//=============================================================================
// Autopilot Implementation
//=============================================================================

Autopilot::Autopilot(const ControlLawConfig& config)
    : config_(config) {
    // Initialize controllers
    PIDGains alt_gains{0.1, 0.01, 0.05, 100.0, 0.5};
    altitude_controller_.set_gains(alt_gains);

    PIDGains vs_gains{0.2, 0.02, 0.1, 50.0, 0.3};
    vertical_speed_controller_.set_gains(vs_gains);

    PIDGains hdg_gains{1.5, 0.05, 0.2, 1.0, 0.6};
    heading_controller_.set_gains(hdg_gains);

    PIDGains spd_gains{0.05, 0.005, 0.02, 0.5, 0.5};
    speed_controller_.set_gains(spd_gains);

    PIDGains trk_gains{1.0, 0.02, 0.1, 500.0, 0.5};
    track_controller_.set_gains(trk_gains);
}

ControlInput Autopilot::calculate(const AutopilotCommand& command,
                                  const FlightState& state,
                                  double dt) {
    ControlInput output;

    if (!engaged_) {
        return output;
    }

    switch (current_mode_) {
        case AutopilotMode::ATTITUDE_HOLD:
            output.pitch = calculate_altitude_command(target_altitude_, state, dt) * 0.5;
            output.roll = 0.0;  // Wings level
            break;

        case AutopilotMode::ALTITUDE_HOLD:
            output.pitch = calculate_altitude_command(command.target_altitude, state, dt);
            break;

        case AutopilotMode::HEADING_HOLD:
            output.roll = calculate_heading_command(command.target_heading, state, dt);
            break;

        case AutopilotMode::SPEED_HOLD:
            output.throttle = calculate_speed_command(command.target_speed, state, dt);
            break;

        case AutopilotMode::VERTICAL_SPEED: {
            double vs_error = command.target_vertical_speed - state.velocity_ned.z;
            output.pitch = vertical_speed_controller_.update(vs_error, dt, state.airspeed);
            output.pitch = std::clamp(output.pitch, -max_pitch_command_, max_pitch_command_);
            break;
        }

        case AutopilotMode::NAV:
            output = calculate_navigation(command, state, dt);
            break;

        case AutopilotMode::INTERCEPT:
            output = calculate_intercept(command, state, dt);
            break;

        default:
            break;
    }

    // Apply limits
    output.pitch = std::clamp(output.pitch, -1.0, 1.0);
    output.roll = std::clamp(output.roll, -1.0, 1.0);
    output.yaw = std::clamp(output.yaw, -1.0, 1.0);
    output.throttle = std::clamp(output.throttle, 0.0, 1.0);

    return output;
}

void Autopilot::engage(AutopilotMode mode, const FlightState& state) {
    engaged_ = true;
    current_mode_ = mode;

    // Capture current state as initial targets
    target_altitude_ = state.altitude_msl;
    target_heading_ = state.heading;
    target_speed_ = state.airspeed;

    // Reset capture flags
    altitude_captured_ = false;
    heading_captured_ = false;
    speed_captured_ = false;

    // Reset controllers for bumpless transfer
    altitude_controller_.reset();
    vertical_speed_controller_.reset();
    heading_controller_.reset();
    speed_controller_.reset();
    track_controller_.reset();
}

void Autopilot::disengage() {
    engaged_ = false;
    current_mode_ = AutopilotMode::OFF;
}

void Autopilot::reset() {
    disengage();
    altitude_controller_.reset();
    vertical_speed_controller_.reset();
    heading_controller_.reset();
    speed_controller_.reset();
    track_controller_.reset();
    distance_to_waypoint_ = 0.0;
    bearing_to_waypoint_ = 0.0;
    cross_track_error_ = 0.0;
}

bool Autopilot::is_captured(AutopilotMode mode) const {
    switch (mode) {
        case AutopilotMode::ALTITUDE_HOLD:
            return altitude_captured_;
        case AutopilotMode::HEADING_HOLD:
            return heading_captured_;
        case AutopilotMode::SPEED_HOLD:
            return speed_captured_;
        default:
            return true;
    }
}

double Autopilot::calculate_altitude_command(double target,
                                             const FlightState& state,
                                             double dt) {
    double alt_error = target - state.altitude_msl;

    // Check for capture
    if (std::abs(alt_error) < 30.0) {  // Within 30m
        altitude_captured_ = true;
    }

    // Two-loop altitude control: altitude -> vertical speed -> pitch
    double vs_cmd = altitude_controller_.update(alt_error, dt, state.airspeed);
    vs_cmd = std::clamp(vs_cmd, -20.0, 20.0);  // Limit to ±20 m/s VS

    double vs_error = vs_cmd - (-state.velocity_ned.z);  // NED, so negate
    double pitch_cmd = vertical_speed_controller_.update(vs_error, dt, state.airspeed);

    return std::clamp(pitch_cmd, -max_pitch_command_, max_pitch_command_);
}

double Autopilot::calculate_heading_command(double target,
                                            const FlightState& state,
                                            double dt) {
    double hdg_error = heading_error(target, state.heading);

    // Check for capture
    if (std::abs(hdg_error) < 0.05) {  // Within ~3 degrees
        heading_captured_ = true;
    }

    // Heading -> bank angle
    double bank_cmd = heading_controller_.update(hdg_error, dt, state.airspeed);
    bank_cmd = std::clamp(bank_cmd, -max_roll_command_, max_roll_command_);

    // Convert to stick input (normalized)
    return bank_cmd / max_roll_command_;
}

double Autopilot::calculate_speed_command(double target,
                                          const FlightState& state,
                                          double dt) {
    double spd_error = target - state.airspeed;

    // Check for capture
    if (std::abs(spd_error) < 5.0) {  // Within 5 m/s
        speed_captured_ = true;
    }

    double throttle_cmd = speed_controller_.update(spd_error, dt, state.airspeed);
    return std::clamp(0.5 + throttle_cmd, 0.0, 1.0);  // Center throttle + correction
}

ControlInput Autopilot::calculate_navigation(const AutopilotCommand& cmd,
                                             const FlightState& state,
                                             double dt) {
    ControlInput output;

    if (cmd.flight_plan.empty() || cmd.active_waypoint >= cmd.flight_plan.size()) {
        return output;
    }

    const Waypoint& wp = cmd.flight_plan[cmd.active_waypoint];

    // Calculate vector to waypoint
    math::Vec3 to_wp = wp.position - state.position;
    distance_to_waypoint_ = to_wp.length();

    // Bearing to waypoint
    bearing_to_waypoint_ = std::atan2(to_wp.x, to_wp.y);  // Assuming NED

    // Cross-track error (simplified)
    double desired_track = wp.course;
    double track_error = bearing_to_waypoint_ - desired_track;
    track_error = normalize_heading(track_error);
    cross_track_error_ = distance_to_waypoint_ * std::sin(track_error);

    // Calculate heading to intercept course
    double intercept_heading = bearing_to_waypoint_;

    // Add correction for cross-track error
    double xte_correction = track_controller_.update(cross_track_error_, dt, state.airspeed);
    xte_correction = std::clamp(xte_correction, -0.5, 0.5);
    intercept_heading += xte_correction;

    // Heading control
    output.roll = calculate_heading_command(intercept_heading, state, dt);

    // Altitude control
    output.pitch = calculate_altitude_command(wp.altitude, state, dt);

    // Speed control
    if (wp.speed > 0.0) {
        output.throttle = calculate_speed_command(wp.speed, state, dt);
    } else {
        output.throttle = calculate_speed_command(target_speed_, state, dt);
    }

    return output;
}

ControlInput Autopilot::calculate_intercept(const AutopilotCommand& cmd,
                                            const FlightState& state,
                                            double dt) {
    ControlInput output;

    // Proportional navigation for missile guidance
    math::Vec3 to_target = cmd.target_position - state.position;
    math::Vec3 relative_velocity = state.velocity_ned - cmd.target_velocity;

    double closing_velocity = -to_target.normalized().dot(relative_velocity);
    double time_to_go = to_target.length() / std::max(closing_velocity, 10.0);

    // Predicted intercept point
    math::Vec3 intercept_point = cmd.target_position + cmd.target_velocity * time_to_go;

    // Lead angle
    math::Vec3 to_intercept = intercept_point - state.position;
    to_intercept = to_intercept.normalized();

    // Calculate required heading and pitch
    double required_heading = std::atan2(to_intercept.x, to_intercept.y);
    double required_pitch = std::asin(-to_intercept.z);

    // Command heading and pitch
    output.roll = calculate_heading_command(required_heading, state, dt);
    output.pitch = (required_pitch - state.pitch) / max_pitch_command_;
    output.pitch = std::clamp(output.pitch, -1.0, 1.0);

    // Full throttle for intercept
    output.throttle = 1.0;

    return output;
}

double Autopilot::normalize_heading(double heading) {
    while (heading > M_PI) heading -= TWO_PI;
    while (heading < -M_PI) heading += TWO_PI;
    return heading;
}

double Autopilot::heading_error(double target, double current) {
    double error = target - current;
    return normalize_heading(error);
}

//=============================================================================
// EnvelopeProtection Implementation
//=============================================================================

EnvelopeProtection::EnvelopeProtection(const FlightEnvelope& envelope)
    : envelope_(envelope) {}

ControlInput EnvelopeProtection::apply(const ControlInput& input,
                                       const FlightState& state) {
    if (!enabled_) {
        return input;
    }

    ControlInput limited = input;
    current_limit_ = EnvelopeLimit::NONE;

    // AOA protection
    double aoa_margin = envelope_.aoa_max - state.angle_of_attack;
    if (aoa_margin < 0.05) {  // Within 3 degrees of limit
        aoa_limit_factor_ = std::max(0.0, aoa_margin / 0.05);
        // Reduce pull authority
        if (limited.pitch > 0.0) {
            limited.pitch *= aoa_limit_factor_;
        }
        current_limit_ = EnvelopeLimit::AOA_HIGH;
    } else {
        aoa_limit_factor_ = 1.0;
    }

    // Low AOA protection
    double low_aoa_margin = state.angle_of_attack - envelope_.aoa_min;
    if (low_aoa_margin < 0.05) {
        double factor = std::max(0.0, low_aoa_margin / 0.05);
        if (limited.pitch < 0.0) {
            limited.pitch *= factor;
        }
        current_limit_ = EnvelopeLimit::AOA_LOW;
    }

    // G limit protection
    if (state.normal_load > envelope_.g_warning) {
        double g_margin = envelope_.g_max - state.normal_load;
        g_limit_factor_ = std::max(0.0, g_margin / (envelope_.g_max - envelope_.g_warning));
        if (limited.pitch > 0.0) {
            limited.pitch *= g_limit_factor_;
        }
        current_limit_ = EnvelopeLimit::G_HIGH;
    } else if (state.normal_load < -envelope_.g_warning / 3.0) {
        double g_margin = state.normal_load - envelope_.g_min;
        g_limit_factor_ = std::max(0.0, g_margin / (-envelope_.g_min + envelope_.g_warning / 3.0));
        if (limited.pitch < 0.0) {
            limited.pitch *= g_limit_factor_;
        }
        current_limit_ = EnvelopeLimit::G_LOW;
    } else {
        g_limit_factor_ = 1.0;
    }

    // Overspeed protection
    if (state.airspeed > envelope_.vmo) {
        double speed_margin = envelope_.vne - state.airspeed;
        speed_limit_factor_ = std::max(0.0, speed_margin / (envelope_.vne - envelope_.vmo));

        // Reduce throttle, increase pitch
        limited.throttle *= speed_limit_factor_;
        if (limited.pitch < 0.0) {
            limited.pitch *= speed_limit_factor_;
        }
        current_limit_ = EnvelopeLimit::SPEED_HIGH;
    }

    // Low speed protection (stall)
    double min_speed = envelope_.vs * (1.0 + (1.0 - input.flaps) * 0.2);  // Adjust for flaps
    if (state.airspeed < min_speed * 1.2) {  // 20% margin
        // Add nose-down input
        double margin = (state.airspeed - min_speed) / (min_speed * 0.2);
        margin = std::clamp(margin, 0.0, 1.0);

        if (limited.pitch > 0.0) {
            limited.pitch *= margin;
        }
        current_limit_ = EnvelopeLimit::SPEED_LOW;
    }

    // Roll rate limiting
    if (std::abs(state.roll_rate) > envelope_.roll_rate_max * 0.9) {
        double rate_margin = envelope_.roll_rate_max - std::abs(state.roll_rate);
        double factor = std::max(0.0, rate_margin / (envelope_.roll_rate_max * 0.1));

        if ((state.roll_rate > 0 && limited.roll > 0) ||
            (state.roll_rate < 0 && limited.roll < 0)) {
            limited.roll *= factor;
        }
        current_limit_ = EnvelopeLimit::ROLL_RATE;
    }

    return limited;
}

EnvelopeLimit EnvelopeProtection::check_status(const FlightState& state) const {
    if (state.angle_of_attack > envelope_.aoa_warning) return EnvelopeLimit::AOA_HIGH;
    if (state.angle_of_attack < envelope_.aoa_min + 0.05) return EnvelopeLimit::AOA_LOW;
    if (state.normal_load > envelope_.g_warning) return EnvelopeLimit::G_HIGH;
    if (state.normal_load < envelope_.g_min + 1.0) return EnvelopeLimit::G_LOW;
    if (state.airspeed > envelope_.vmo) return EnvelopeLimit::SPEED_HIGH;
    if (state.airspeed < envelope_.vs * 1.1) return EnvelopeLimit::SPEED_LOW;
    return EnvelopeLimit::NONE;
}

//=============================================================================
// FlightControlSystem Implementation
//=============================================================================

FlightControlSystem::FlightControlSystem(const ControlLawConfig& config,
                                         const FlightEnvelope& envelope)
    : config_(config)
    , sas_controller_(std::make_unique<SASController>(config))
    , cas_controller_(std::make_unique<CASController>(config))
    , autopilot_(config)
    , envelope_protection_(envelope) {}

FlightControlSystem::~FlightControlSystem() = default;

SurfacePositions FlightControlSystem::update(const ControlInput& input,
                                             const FlightState& state,
                                             double dt) {
    // Handle autopilot engage/disengage buttons
    handle_autopilot_buttons(input, state);

    // Get base control input (may be from pilot or autopilot)
    ControlInput effective_input = input;

    if (autopilot_.is_engaged()) {
        // Get autopilot control commands
        ControlInput ap_input = autopilot_.calculate(ap_command_, state, dt);

        // Blend with pilot input (pilot override)
        // If pilot moves stick significantly, use pilot input
        if (std::abs(input.pitch) > 0.1 || std::abs(input.roll) > 0.1) {
            // Pilot override - disable autopilot for this axis
            effective_input.pitch = input.pitch;
            effective_input.roll = input.roll;
        } else {
            effective_input.pitch = ap_input.pitch;
            effective_input.roll = ap_input.roll;
        }

        // Yaw always from pilot unless coordinated turn
        if (std::abs(input.yaw) > 0.05) {
            effective_input.yaw = input.yaw;
        } else {
            effective_input.yaw = ap_input.yaw;
        }

        // Throttle from autopilot in speed mode
        if (ap_command_.mode == AutopilotMode::SPEED_HOLD ||
            ap_command_.mode == AutopilotMode::NAV) {
            effective_input.throttle = ap_input.throttle;
        }
    }

    // Apply envelope protection
    ControlInput protected_input = envelope_protection_.apply(effective_input, state);

    // Check for warnings
    EnvelopeLimit warning = envelope_protection_.check_status(state);
    if (warning != EnvelopeLimit::NONE && warning_callback_) {
        warning_callback_(warning);
    }

    // Calculate surface positions through selected control law
    SurfacePositions surfaces;

    switch (control_mode_) {
        case ControlMode::MANUAL:
            // Direct stick-to-surface
            surfaces.elevator = protected_input.pitch * config_.elevator_max;
            surfaces.aileron_left = protected_input.roll * config_.aileron_max;
            surfaces.aileron_right = -protected_input.roll * config_.aileron_max;
            surfaces.rudder = protected_input.yaw * config_.rudder_max;
            surfaces.throttle_position = protected_input.throttle;
            break;

        case ControlMode::SAS:
            surfaces = sas_controller_->calculate(protected_input, state, dt);
            break;

        case ControlMode::CAS:
        case ControlMode::AUTOPILOT:
            surfaces = cas_controller_->calculate(protected_input, state, dt);
            break;

        case ControlMode::EMERGENCY:
            // Degraded mode - minimal augmentation
            surfaces = sas_controller_->calculate(protected_input, state, dt);
            break;
    }

    // Update auto-trim
    if (auto_trim_enabled_) {
        update_auto_trim(state, dt);
    }

    // Apply control mixing (e.g., elevon mixing for delta wings)
    surfaces = apply_mixing(surfaces, state);

    prev_surfaces_ = surfaces;
    return surfaces;
}

void FlightControlSystem::set_control_mode(ControlMode mode) {
    if (mode != control_mode_) {
        ControlMode old_mode = control_mode_;
        control_mode_ = mode;

        // Reset controllers on mode change for bumpless transfer
        sas_controller_->reset();
        cas_controller_->reset();

        if (mode != ControlMode::AUTOPILOT) {
            autopilot_.disengage();
        }

        if (mode_change_callback_) {
            mode_change_callback_(old_mode, mode);
        }
    }
}

void FlightControlSystem::set_autopilot_command(const AutopilotCommand& command) {
    ap_command_ = command;
}

void FlightControlSystem::reset() {
    control_mode_ = ControlMode::CAS;
    sas_controller_->reset();
    cas_controller_->reset();
    autopilot_.reset();

    trim_state_ = TrimState{};
    prev_surfaces_ = SurfacePositions{};
}

std::string FlightControlSystem::get_active_control_law_name() const {
    switch (control_mode_) {
        case ControlMode::MANUAL:
            return "MANUAL";
        case ControlMode::SAS:
            return sas_controller_->name();
        case ControlMode::CAS:
            return cas_controller_->name();
        case ControlMode::AUTOPILOT:
            return "AUTOPILOT";
        case ControlMode::EMERGENCY:
            return "EMERGENCY";
        default:
            return "UNKNOWN";
    }
}

EnvelopeLimit FlightControlSystem::get_active_warning() const {
    return envelope_protection_.check_status(FlightState{});  // Would need cached state
}

void FlightControlSystem::set_mode_change_callback(ModeChangeCallback callback) {
    mode_change_callback_ = std::move(callback);
}

void FlightControlSystem::set_warning_callback(WarningCallback callback) {
    warning_callback_ = std::move(callback);
}

void FlightControlSystem::update_auto_trim(const FlightState& state, double dt) {
    // Slowly adjust trim to reduce control forces
    // Only trim when in steady flight (low rates, moderate G)

    if (std::abs(state.pitch_rate) < 0.02 &&
        std::abs(state.roll_rate) < 0.05 &&
        std::abs(state.normal_load - 1.0) < 0.2) {

        // Trim in direction of current control deflection
        double pitch_trim_delta = prev_surfaces_.elevator / config_.elevator_max * trim_rate_ * dt;
        double roll_trim_delta = prev_surfaces_.aileron_left / config_.aileron_max * trim_rate_ * dt;

        trim_state_.pitch_trim += pitch_trim_delta;
        trim_state_.roll_trim += roll_trim_delta;

        // Limit trim
        trim_state_.pitch_trim = std::clamp(trim_state_.pitch_trim, -0.3, 0.3);
        trim_state_.roll_trim = std::clamp(trim_state_.roll_trim, -0.2, 0.2);
    }
}

SurfacePositions FlightControlSystem::apply_mixing(const SurfacePositions& raw,
                                                   const FlightState& state) {
    SurfacePositions mixed = raw;

    // Example: Elevon mixing for delta/flying wing
    if (config_.aircraft_type == AircraftType::FIXED_WING) {
        // Could detect elevon-equipped aircraft and apply mixing
        // For conventional: pass through
    }

    // Roll-pitch coupling compensation at high AOA
    if (std::abs(state.angle_of_attack) > 0.2) {
        double coupling = config_.roll_pitch_compensation * raw.aileron_left *
                          std::sin(state.angle_of_attack);
        mixed.elevator += coupling;
    }

    return mixed;
}

void FlightControlSystem::handle_autopilot_buttons(const ControlInput& input,
                                                   const FlightState& state) {
    if (input.autopilot_disconnect) {
        autopilot_.disengage();
        if (control_mode_ == ControlMode::AUTOPILOT) {
            set_control_mode(ControlMode::CAS);
        }
    }

    if (input.autopilot_engage && !autopilot_.is_engaged()) {
        autopilot_.engage(ap_command_.mode, state);
        set_control_mode(ControlMode::AUTOPILOT);
    }
}

//=============================================================================
// ProportionalNavigation Implementation
//=============================================================================

ProportionalNavigation::ProportionalNavigation(double navigation_constant)
    : nav_constant_(navigation_constant) {}

math::Vec3 ProportionalNavigation::calculate(const math::Vec3& pursuer_position,
                                             const math::Vec3& pursuer_velocity,
                                             const math::Vec3& target_position,
                                             const math::Vec3& target_velocity) {
    // Line of sight vector
    math::Vec3 los = target_position - pursuer_position;
    double range = los.length();

    if (range < 1.0) {
        return math::Vec3{0.0, 0.0, 0.0};  // Too close
    }

    math::Vec3 los_unit = los / range;

    // LOS rate
    math::Vec3 relative_velocity = target_velocity - pursuer_velocity;
    double closing_velocity = -los_unit.dot(relative_velocity);

    // Calculate LOS rate (omega = V_perpendicular / R)
    math::Vec3 v_perpendicular = relative_velocity - los_unit * relative_velocity.dot(los_unit);
    math::Vec3 los_rate = v_perpendicular / range;

    // Commanded acceleration: a_c = N' * Vc * omega
    math::Vec3 commanded_accel = los_rate * nav_constant_ * closing_velocity;

    return commanded_accel;
}

//=============================================================================
// AugmentedPN Implementation
//=============================================================================

AugmentedPN::AugmentedPN(double navigation_constant, double target_accel_gain)
    : ProportionalNavigation(navigation_constant)
    , target_accel_gain_(target_accel_gain) {}

math::Vec3 AugmentedPN::calculate(const math::Vec3& pursuer_position,
                                  const math::Vec3& pursuer_velocity,
                                  const math::Vec3& target_position,
                                  const math::Vec3& target_velocity,
                                  const math::Vec3& target_acceleration) {
    // Get base PN command
    math::Vec3 pn_cmd = ProportionalNavigation::calculate(
        pursuer_position, pursuer_velocity, target_position, target_velocity);

    // Add target acceleration compensation
    // a_augmented = a_PN + (N'/2) * a_target_perpendicular
    math::Vec3 los = target_position - pursuer_position;
    math::Vec3 los_unit = los.normalized();

    math::Vec3 target_accel_perp = target_acceleration -
        los_unit * target_acceleration.dot(los_unit);

    math::Vec3 augmentation = target_accel_perp * target_accel_gain_ *
                               get_navigation_constant() / 2.0;

    return pn_cmd + augmentation;
}

//=============================================================================
// FlightControlUtils Implementation
//=============================================================================

namespace FlightControlUtils {

double coordinated_turn_bank(double velocity, double turn_rate) {
    // tan(bank) = V * omega / g
    if (velocity < 1.0 || std::abs(turn_rate) < 0.001) return 0.0;
    return std::atan(velocity * turn_rate / GRAVITY);
}

double turn_radius(double velocity, double bank_angle) {
    // R = V^2 / (g * tan(bank))
    if (std::abs(bank_angle) < 0.01) return std::numeric_limits<double>::infinity();
    return velocity * velocity / (GRAVITY * std::tan(bank_angle));
}

double level_off_time(double current_altitude, double target_altitude,
                      double vertical_speed, double pitch_rate) {
    // Simplified level-off calculation
    double altitude_to_go = target_altitude - current_altitude;

    if (std::abs(vertical_speed) < 0.5) return 0.0;  // Already level

    // Time to pitch over
    double pitch_time = std::abs(vertical_speed) / (GRAVITY * pitch_rate);

    // Distance covered during pitch-over (approximately)
    double pitch_altitude = vertical_speed * pitch_time / 2.0;

    // Time to start level-off
    double remaining_altitude = altitude_to_go - pitch_altitude;
    double time_to_level = remaining_altitude / vertical_speed;

    return std::max(0.0, time_to_level);
}

math::Vec3 calculate_ground_speed(const math::Vec3& tas_vector,
                                  const math::Vec3& wind_vector) {
    return tas_vector + wind_vector;
}

} // namespace FlightControlUtils

//=============================================================================
// Factory Functions Implementation
//=============================================================================

std::unique_ptr<FlightControlSystem> create_fighter_fcs() {
    ControlLawConfig config;
    config.aircraft_type = AircraftType::FIXED_WING;

    // High-performance fighter gains
    config.pitch_rate = {0.8, 0.1, 0.2, 0.5, 1.0, 0.5, 1.5, 100.0, 400.0};
    config.roll_rate = {0.5, 0.05, 0.15, 0.3, 1.0, 0.7, 1.3, 100.0, 400.0};
    config.yaw_rate = {1.0, 0.1, 0.2, 0.3, 1.0};

    config.pitch_attitude = {2.0, 0.2, 0.5, 0.5, 0.3};
    config.roll_attitude = {3.0, 0.1, 0.3, 0.3, 0.6};

    config.elevator_max = 25.0;
    config.elevator_min = -20.0;
    config.aileron_max = 25.0;
    config.rudder_max = 30.0;

    config.elevator_rate = 80.0;
    config.aileron_rate = 100.0;
    config.rudder_rate = 80.0;

    config.aileron_rudder_interconnect = 0.15;
    config.roll_pitch_compensation = 0.1;
    config.auto_rudder = true;

    FlightEnvelope envelope;
    envelope.aoa_max = 0.45;   // ~26 degrees
    envelope.aoa_min = -0.26;  // ~-15 degrees
    envelope.g_max = 9.0;
    envelope.g_min = -3.0;
    envelope.vne = 450.0;
    envelope.vmo = 400.0;
    envelope.vs = 70.0;
    envelope.mach_max = 2.0;
    envelope.roll_rate_max = 5.0;

    return std::make_unique<FlightControlSystem>(config, envelope);
}

std::unique_ptr<FlightControlSystem> create_transport_fcs() {
    ControlLawConfig config;
    config.aircraft_type = AircraftType::FIXED_WING;

    // Smooth, predictable transport gains
    config.pitch_rate = {0.5, 0.08, 0.15, 0.3, 1.0};
    config.roll_rate = {0.4, 0.04, 0.1, 0.2, 1.0};
    config.yaw_rate = {0.8, 0.08, 0.15, 0.2, 1.0};

    config.pitch_attitude = {1.5, 0.15, 0.4, 0.3, 0.2};
    config.roll_attitude = {2.0, 0.08, 0.25, 0.2, 0.4};

    config.elevator_max = 20.0;
    config.elevator_min = -12.0;
    config.aileron_max = 15.0;
    config.rudder_max = 25.0;

    config.elevator_rate = 40.0;
    config.aileron_rate = 50.0;
    config.rudder_rate = 40.0;

    config.aileron_rudder_interconnect = 0.1;
    config.auto_rudder = true;

    FlightEnvelope envelope;
    envelope.aoa_max = 0.26;   // ~15 degrees
    envelope.g_max = 2.5;
    envelope.g_min = -1.0;
    envelope.vne = 200.0;
    envelope.vmo = 180.0;
    envelope.vs = 55.0;
    envelope.mach_max = 0.85;
    envelope.roll_rate_max = 0.5;

    return std::make_unique<FlightControlSystem>(config, envelope);
}

std::unique_ptr<FlightControlSystem> create_helicopter_fcs() {
    ControlLawConfig config;
    config.aircraft_type = AircraftType::ROTORCRAFT;

    // Helicopter control gains
    config.pitch_rate = {1.0, 0.15, 0.25, 0.5, 1.0};
    config.roll_rate = {1.0, 0.15, 0.25, 0.5, 1.0};
    config.yaw_rate = {1.2, 0.1, 0.2, 0.3, 1.0};

    config.pitch_attitude = {2.5, 0.2, 0.6, 0.5, 0.3};
    config.roll_attitude = {2.5, 0.2, 0.6, 0.5, 0.3};

    // Rotorcraft doesn't use traditional surfaces
    config.elevator_max = 15.0;  // Cyclic pitch
    config.aileron_max = 15.0;   // Cyclic roll
    config.rudder_max = 20.0;    // Tail rotor

    config.auto_rudder = true;

    FlightEnvelope envelope;
    envelope.aoa_max = 0.35;
    envelope.g_max = 3.5;
    envelope.g_min = -0.5;
    envelope.vne = 90.0;
    envelope.vmo = 80.0;
    envelope.vs = 0.0;  // Helicopters can hover
    envelope.roll_rate_max = 1.5;

    return std::make_unique<FlightControlSystem>(config, envelope);
}

std::unique_ptr<FlightControlSystem> create_missile_fcs() {
    ControlLawConfig config;
    config.aircraft_type = AircraftType::MISSILE;

    // Agile missile gains
    config.pitch_rate = {2.0, 0.3, 0.5, 1.0, 1.0};
    config.roll_rate = {0.0, 0.0, 0.0, 0.0, 0.0};  // Bank-to-turn or skid-to-turn
    config.yaw_rate = {2.0, 0.3, 0.5, 1.0, 1.0};

    config.elevator_max = 30.0;
    config.rudder_max = 30.0;
    config.aileron_max = 0.0;  // Missiles typically don't roll

    config.elevator_rate = 200.0;
    config.rudder_rate = 200.0;

    FlightEnvelope envelope;
    envelope.aoa_max = 0.52;   // ~30 degrees
    envelope.g_max = 40.0;     // High-G capable
    envelope.g_min = -40.0;
    envelope.vne = 1500.0;
    envelope.mach_max = 4.0;
    envelope.roll_rate_max = 10.0;

    return std::make_unique<FlightControlSystem>(config, envelope);
}

} // namespace dynamics
} // namespace jaguar
