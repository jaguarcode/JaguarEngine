/**
 * @file flight_control.h
 * @brief Flight Control System (FCS) and Autopilot for JaguarEngine
 * @version 0.5.0
 *
 * Provides comprehensive flight control including:
 * - Stability Augmentation System (SAS)
 * - Control Augmentation System (CAS)
 * - Autopilot modes (altitude hold, heading hold, waypoint nav)
 * - Fly-by-wire envelope protection
 * - Auto-trim and automatic flight control
 *
 * Applicable to:
 * - Fixed-wing aircraft
 * - Rotorcraft
 * - Missiles and UAVs
 */

#pragma once

#include "jaguar/core/types.h"
#include "jaguar/core/math/vector.h"
#include "jaguar/core/math/quaternion.h"
#include <vector>
#include <memory>
#include <functional>
#include <string>
#include <cmath>
#include <unordered_map>

namespace jaguar {
namespace dynamics {

// Forward declarations
class FlightControlSystem;
class Autopilot;
class ControlLaw;

//=============================================================================
// Enumerations
//=============================================================================

/**
 * @brief Aircraft type for control law selection
 */
enum class AircraftType : uint8_t {
    FIXED_WING,         ///< Conventional fixed-wing aircraft
    ROTORCRAFT,         ///< Helicopter
    TILTROTOR,          ///< V-22 type tiltrotor
    MULTIROTOR,         ///< Quadcopter, hexacopter, etc.
    MISSILE,            ///< Guided missile
    SPACECRAFT          ///< Space vehicle (reaction control)
};

/**
 * @brief Control surface types
 */
enum class ControlSurface : uint8_t {
    ELEVATOR,           ///< Pitch control
    AILERON,            ///< Roll control
    RUDDER,             ///< Yaw control
    FLAP,               ///< High-lift device
    SPOILER,            ///< Drag/lift dump
    STABILATOR,         ///< All-moving horizontal tail
    CANARD,             ///< Forward control surface
    ELEVON,             ///< Combined elevator/aileron
    RUDDERVATOR,        ///< Combined rudder/elevator (V-tail)
    COLLECTIVE,         ///< Helicopter collective pitch
    CYCLIC_PITCH,       ///< Helicopter cyclic pitch
    CYCLIC_ROLL,        ///< Helicopter cyclic roll
    TAIL_ROTOR,         ///< Helicopter tail rotor/yaw
    THRUST_VECTOR       ///< Thrust vectoring
};

/**
 * @brief Flight control modes
 */
enum class ControlMode : uint8_t {
    MANUAL,             ///< Direct stick-to-surface
    SAS,                ///< Stability Augmentation System
    CAS,                ///< Control Augmentation System
    AUTOPILOT,          ///< Automatic flight control
    EMERGENCY           ///< Emergency/degraded mode
};

/**
 * @brief Autopilot modes
 */
enum class AutopilotMode : uint8_t {
    OFF,                ///< Autopilot disengaged
    ATTITUDE_HOLD,      ///< Hold current attitude
    ALTITUDE_HOLD,      ///< Hold current altitude
    HEADING_HOLD,       ///< Hold current heading
    SPEED_HOLD,         ///< Hold current airspeed
    VERTICAL_SPEED,     ///< Hold vertical speed
    NAV,                ///< Navigation (waypoint following)
    APPROACH,           ///< ILS/GPS approach
    AUTOLAND,           ///< Automatic landing
    TERRAIN_FOLLOW,     ///< Terrain following
    FORMATION,          ///< Formation flying
    INTERCEPT           ///< Target intercept (missiles)
};

/**
 * @brief Flight envelope limits
 */
enum class EnvelopeLimit : uint8_t {
    NONE,               ///< No limit active
    AOA_HIGH,           ///< High angle of attack
    AOA_LOW,            ///< Low/negative AOA
    G_HIGH,             ///< High positive G
    G_LOW,              ///< High negative G
    ROLL_RATE,          ///< Roll rate limit
    PITCH_RATE,         ///< Pitch rate limit
    YAW_RATE,           ///< Yaw rate limit
    SPEED_HIGH,         ///< Overspeed
    SPEED_LOW,          ///< Stall warning
    ALTITUDE_HIGH,      ///< Service ceiling
    ALTITUDE_LOW        ///< Terrain warning
};

//=============================================================================
// Data Structures
//=============================================================================

/**
 * @brief Pilot control inputs
 */
struct ControlInput {
    double pitch = 0.0;     ///< Pitch stick (-1 to +1, forward = negative)
    double roll = 0.0;      ///< Roll stick (-1 to +1, right = positive)
    double yaw = 0.0;       ///< Rudder pedals (-1 to +1, right = positive)
    double throttle = 0.0;  ///< Throttle (0 to 1)

    // Additional inputs
    double collective = 0.0;///< Helicopter collective (0 to 1)
    double flaps = 0.0;     ///< Flap position (0 to 1)
    double speedbrake = 0.0;///< Speed brake (0 to 1)

    // Trim adjustments
    double pitch_trim = 0.0;
    double roll_trim = 0.0;
    double yaw_trim = 0.0;

    // Buttons/switches
    bool autopilot_engage = false;
    bool autopilot_disconnect = false;
    bool trim_release = false;
};

/**
 * @brief Control surface positions
 */
struct SurfacePositions {
    double elevator = 0.0;      ///< degrees, positive = trailing edge up
    double aileron_left = 0.0;  ///< degrees
    double aileron_right = 0.0; ///< degrees
    double rudder = 0.0;        ///< degrees, positive = trailing edge left
    double stabilator = 0.0;    ///< degrees
    double flap_left = 0.0;     ///< degrees
    double flap_right = 0.0;    ///< degrees
    double spoiler_left = 0.0;  ///< degrees
    double spoiler_right = 0.0; ///< degrees

    // Rotorcraft
    double collective_pitch = 0.0;  ///< degrees
    double cyclic_pitch = 0.0;      ///< degrees
    double cyclic_roll = 0.0;       ///< degrees
    double tail_rotor = 0.0;        ///< degrees

    // Engine
    double throttle_position = 0.0; ///< 0-1
    double thrust_vector_pitch = 0.0; ///< degrees
    double thrust_vector_yaw = 0.0;   ///< degrees
};

/**
 * @brief Aircraft flight state for control laws
 */
struct FlightState {
    // Position and velocity
    math::Vec3 position;            ///< ECEF or local position (m)
    math::Vec3 velocity;            ///< Velocity in body frame (m/s)
    math::Vec3 velocity_ned;        ///< Velocity in NED frame (m/s)

    // Attitude
    math::Quat orientation;         ///< Body to world quaternion
    double roll = 0.0;              ///< Roll angle (rad)
    double pitch = 0.0;             ///< Pitch angle (rad)
    double heading = 0.0;           ///< True heading (rad)

    // Angular rates
    math::Vec3 angular_velocity;    ///< Body angular rates (rad/s)
    double roll_rate = 0.0;         ///< Roll rate (rad/s)
    double pitch_rate = 0.0;        ///< Pitch rate (rad/s)
    double yaw_rate = 0.0;          ///< Yaw rate (rad/s)

    // Accelerations
    math::Vec3 acceleration;        ///< Body accelerations (m/s²)
    double normal_load = 1.0;       ///< Normal load factor (G)
    double lateral_load = 0.0;      ///< Lateral load factor (G)
    double axial_load = 0.0;        ///< Axial load factor (G)

    // Air data
    double altitude_msl = 0.0;      ///< Altitude MSL (m)
    double altitude_agl = 0.0;      ///< Altitude AGL (m)
    double airspeed = 0.0;          ///< True airspeed (m/s)
    double indicated_airspeed = 0.0;///< IAS (m/s)
    double mach = 0.0;              ///< Mach number
    double angle_of_attack = 0.0;   ///< AOA (rad)
    double sideslip = 0.0;          ///< Sideslip angle (rad)
    double dynamic_pressure = 0.0;  ///< qbar (Pa)

    // Environmental
    double air_density = 1.225;     ///< kg/m³
    double temperature = 288.15;    ///< K
    double pressure = 101325.0;     ///< Pa
    math::Vec3 wind_ned;            ///< Wind in NED (m/s)
};

/**
 * @brief Waypoint for navigation
 */
struct Waypoint {
    std::string name;
    math::Vec3 position;            ///< Target position
    double altitude = 0.0;          ///< Target altitude (m)
    double speed = 0.0;             ///< Target speed (m/s), 0 = maintain
    double course = 0.0;            ///< Desired track (rad)

    enum class Type {
        FLY_BY,         ///< Fly-by waypoint (smooth turn)
        FLY_OVER,       ///< Fly-over waypoint (cross directly)
        HOLD,           ///< Holding pattern
        INTERCEPT       ///< Intercept (missiles)
    } type = Type::FLY_BY;

    double turn_radius = 0.0;       ///< Turn radius for fly-by (m)
    double hold_time = 0.0;         ///< Time to hold (s)
};

/**
 * @brief Autopilot commands/targets
 */
struct AutopilotCommand {
    AutopilotMode mode = AutopilotMode::OFF;

    // Targets based on mode
    double target_altitude = 0.0;       ///< Target altitude (m)
    double target_heading = 0.0;        ///< Target heading (rad)
    double target_speed = 0.0;          ///< Target speed (m/s)
    double target_vertical_speed = 0.0; ///< Target VS (m/s)
    double target_roll = 0.0;           ///< Target bank angle (rad)
    double target_pitch = 0.0;          ///< Target pitch (rad)

    // Navigation
    std::vector<Waypoint> flight_plan;
    size_t active_waypoint = 0;

    // Intercept (missiles)
    EntityId target_id = 0;
    math::Vec3 target_position;
    math::Vec3 target_velocity;
};

/**
 * @brief Flight envelope definition
 */
struct FlightEnvelope {
    // Angle of attack limits
    double aoa_max = 0.35;          ///< rad (~20 deg)
    double aoa_min = -0.17;         ///< rad (~-10 deg)
    double aoa_warning = 0.30;      ///< Stall warning threshold

    // Load factor limits
    double g_max = 9.0;             ///< Maximum positive G
    double g_min = -3.0;            ///< Maximum negative G
    double g_warning = 7.5;         ///< G warning threshold

    // Rate limits
    double roll_rate_max = 5.0;     ///< rad/s
    double pitch_rate_max = 1.0;    ///< rad/s
    double yaw_rate_max = 1.0;      ///< rad/s

    // Speed limits
    double vne = 400.0;             ///< Never exceed speed (m/s)
    double vmo = 350.0;             ///< Maximum operating speed (m/s)
    double vs = 60.0;               ///< Stall speed, clean (m/s)
    double vs_full_flap = 50.0;     ///< Stall speed, full flaps

    // Altitude limits
    double ceiling = 15000.0;       ///< Service ceiling (m)
    double floor = 0.0;             ///< Minimum altitude (m)

    // Mach limits
    double mach_max = 0.85;
    double mach_min = 0.0;
};

/**
 * @brief PID controller gains
 */
struct PIDGains {
    double kp = 0.0;    ///< Proportional gain
    double ki = 0.0;    ///< Integral gain
    double kd = 0.0;    ///< Derivative gain

    double integral_limit = 1.0;    ///< Anti-windup limit
    double output_limit = 1.0;      ///< Output saturation limit

    // Gain scheduling parameters
    double kp_schedule_min = 1.0;   ///< Kp multiplier at low speed
    double kp_schedule_max = 1.0;   ///< Kp multiplier at high speed
    double schedule_speed_min = 50.0;
    double schedule_speed_max = 300.0;
};

/**
 * @brief Control law configuration
 */
struct ControlLawConfig {
    AircraftType aircraft_type = AircraftType::FIXED_WING;

    // Surface limits (degrees)
    double elevator_max = 25.0;
    double elevator_min = -15.0;
    double aileron_max = 20.0;
    double rudder_max = 30.0;

    // Surface rates (deg/s)
    double elevator_rate = 60.0;
    double aileron_rate = 80.0;
    double rudder_rate = 60.0;

    // Stick-to-surface gearing
    double pitch_stick_gain = 1.0;
    double roll_stick_gain = 1.0;
    double yaw_stick_gain = 1.0;

    // PID gains for each axis
    PIDGains pitch_rate;
    PIDGains roll_rate;
    PIDGains yaw_rate;

    PIDGains pitch_attitude;
    PIDGains roll_attitude;
    PIDGains heading;

    PIDGains altitude;
    PIDGains speed;
    PIDGains vertical_speed;

    // Cross-coupling compensation
    double aileron_rudder_interconnect = 0.0;
    double roll_pitch_compensation = 0.0;

    // Turn coordination
    double turn_coordination_gain = 1.0;
    bool auto_rudder = true;
};

//=============================================================================
// PID Controller
//=============================================================================

/**
 * @brief PID controller with anti-windup and gain scheduling
 */
class PIDController {
public:
    PIDController() = default;
    explicit PIDController(const PIDGains& gains);

    /**
     * @brief Update controller with new error
     * @param error Current error (setpoint - measured)
     * @param dt Time step (s)
     * @param speed Current speed for gain scheduling
     * @return Control output
     */
    double update(double error, double dt, double speed = 0.0);

    /**
     * @brief Reset controller state
     */
    void reset();

    /**
     * @brief Set new gains
     */
    void set_gains(const PIDGains& gains);

    /**
     * @brief Get current integral value
     */
    double get_integral() const { return integral_; }

    /**
     * @brief Manually set integral (for bumpless transfer)
     */
    void set_integral(double value) { integral_ = value; }

private:
    PIDGains gains_;
    double integral_ = 0.0;
    double prev_error_ = 0.0;
    bool first_update_ = true;

    double schedule_gain(double speed) const;
};

//=============================================================================
// Control Law Base
//=============================================================================

/**
 * @brief Base class for control laws
 */
class ControlLaw {
public:
    virtual ~ControlLaw() = default;

    /**
     * @brief Calculate control surface commands
     * @param input Pilot input
     * @param state Current flight state
     * @param dt Time step (s)
     * @return Surface positions
     */
    virtual SurfacePositions calculate(const ControlInput& input,
                                       const FlightState& state,
                                       double dt) = 0;

    /**
     * @brief Reset control law state
     */
    virtual void reset() = 0;

    /**
     * @brief Get control law name
     */
    virtual std::string name() const = 0;
};

//=============================================================================
// Stability Augmentation System (SAS)
//=============================================================================

/**
 * @brief Stability Augmentation System
 *
 * Adds damping to improve aircraft handling qualities
 * by feeding back angular rates.
 */
class SASController : public ControlLaw {
public:
    SASController(const ControlLawConfig& config);

    SurfacePositions calculate(const ControlInput& input,
                              const FlightState& state,
                              double dt) override;

    void reset() override;
    std::string name() const override { return "SAS"; }

    void set_config(const ControlLawConfig& config);

private:
    ControlLawConfig config_;

    // Rate feedback controllers
    PIDController pitch_rate_controller_;
    PIDController roll_rate_controller_;
    PIDController yaw_rate_controller_;

    // Rate limiters
    double prev_elevator_ = 0.0;
    double prev_aileron_ = 0.0;
    double prev_rudder_ = 0.0;

    double rate_limit(double current, double previous, double max_rate, double dt);
};

//=============================================================================
// Control Augmentation System (CAS)
//=============================================================================

/**
 * @brief Control Augmentation System
 *
 * Provides rate command/attitude hold (RCAH) response
 * Stick inputs command angular rates, returns to neutral attitude when released
 */
class CASController : public ControlLaw {
public:
    CASController(const ControlLawConfig& config);

    SurfacePositions calculate(const ControlInput& input,
                              const FlightState& state,
                              double dt) override;

    void reset() override;
    std::string name() const override { return "CAS"; }

    void set_config(const ControlLawConfig& config);

    /**
     * @brief Enable/disable attitude hold when stick centered
     */
    void set_attitude_hold(bool enable) { attitude_hold_enabled_ = enable; }

private:
    ControlLawConfig config_;

    // Inner loop (rate) controllers
    PIDController pitch_rate_controller_;
    PIDController roll_rate_controller_;
    PIDController yaw_rate_controller_;

    // Outer loop (attitude) controllers
    PIDController pitch_attitude_controller_;
    PIDController roll_attitude_controller_;

    // Attitude references for hold mode
    double reference_pitch_ = 0.0;
    double reference_roll_ = 0.0;
    bool attitude_hold_enabled_ = true;

    // Dead zone detection
    bool pitch_stick_centered_ = true;
    bool roll_stick_centered_ = true;

    // Rate limiters
    double prev_elevator_ = 0.0;
    double prev_aileron_ = 0.0;
    double prev_rudder_ = 0.0;

    static constexpr double STICK_DEADZONE = 0.05;

    double rate_limit(double current, double previous, double max_rate, double dt);
    void update_attitude_reference(const ControlInput& input,
                                   const FlightState& state);
};

//=============================================================================
// Autopilot
//=============================================================================

/**
 * @brief Autopilot system
 *
 * Provides automatic flight control modes:
 * - Altitude hold
 * - Heading hold
 * - Speed hold
 * - Navigation (waypoint following)
 * - Approach and autoland
 */
class Autopilot {
public:
    Autopilot(const ControlLawConfig& config);

    /**
     * @brief Calculate control inputs for autopilot
     * @param command Autopilot commands/targets
     * @param state Current flight state
     * @param dt Time step
     * @return Synthetic pilot inputs to feed to FCS
     */
    ControlInput calculate(const AutopilotCommand& command,
                          const FlightState& state,
                          double dt);

    /**
     * @brief Engage autopilot with specific mode
     */
    void engage(AutopilotMode mode, const FlightState& state);

    /**
     * @brief Disengage autopilot
     */
    void disengage();

    /**
     * @brief Check if autopilot is engaged
     */
    bool is_engaged() const { return engaged_; }

    /**
     * @brief Get current mode
     */
    AutopilotMode get_mode() const { return current_mode_; }

    /**
     * @brief Reset autopilot state
     */
    void reset();

    /**
     * @brief Check if mode capture is complete
     */
    bool is_captured(AutopilotMode mode) const;

    /**
     * @brief Get distance to active waypoint
     */
    double get_distance_to_waypoint() const { return distance_to_waypoint_; }

    /**
     * @brief Get bearing to active waypoint
     */
    double get_bearing_to_waypoint() const { return bearing_to_waypoint_; }

private:
    ControlLawConfig config_;

    bool engaged_ = false;
    AutopilotMode current_mode_ = AutopilotMode::OFF;

    // Altitude control
    PIDController altitude_controller_;
    PIDController vertical_speed_controller_;
    double target_altitude_ = 0.0;
    bool altitude_captured_ = false;

    // Heading control
    PIDController heading_controller_;
    double target_heading_ = 0.0;
    bool heading_captured_ = false;

    // Speed control
    PIDController speed_controller_;
    double target_speed_ = 0.0;
    bool speed_captured_ = false;

    // Navigation
    double distance_to_waypoint_ = 0.0;
    double bearing_to_waypoint_ = 0.0;
    double cross_track_error_ = 0.0;
    PIDController track_controller_;

    // Output limits
    double max_pitch_command_ = 0.26;   // ~15 degrees
    double max_roll_command_ = 0.52;    // ~30 degrees
    double max_throttle_rate_ = 0.2;    // per second

    // Internal methods
    double calculate_altitude_command(double target, const FlightState& state, double dt);
    double calculate_heading_command(double target, const FlightState& state, double dt);
    double calculate_speed_command(double target, const FlightState& state, double dt);
    ControlInput calculate_navigation(const AutopilotCommand& cmd,
                                      const FlightState& state, double dt);
    ControlInput calculate_intercept(const AutopilotCommand& cmd,
                                     const FlightState& state, double dt);

    double normalize_heading(double heading);
    double heading_error(double target, double current);
};

//=============================================================================
// Envelope Protection
//=============================================================================

/**
 * @brief Flight envelope protection system
 *
 * Prevents exceeding structural and aerodynamic limits
 */
class EnvelopeProtection {
public:
    EnvelopeProtection(const FlightEnvelope& envelope);

    /**
     * @brief Apply envelope protection to control inputs
     * @param input Raw control inputs
     * @param state Current flight state
     * @return Limited control inputs
     */
    ControlInput apply(const ControlInput& input, const FlightState& state);

    /**
     * @brief Check current envelope status
     * @return Active limit (if any)
     */
    EnvelopeLimit check_status(const FlightState& state) const;

    /**
     * @brief Enable/disable envelope protection
     */
    void set_enabled(bool enabled) { enabled_ = enabled; }
    bool is_enabled() const { return enabled_; }

    /**
     * @brief Get current G load limit factor (0-1)
     */
    double get_g_limit_factor() const { return g_limit_factor_; }

    /**
     * @brief Get current AOA limit factor (0-1)
     */
    double get_aoa_limit_factor() const { return aoa_limit_factor_; }

private:
    FlightEnvelope envelope_;
    bool enabled_ = true;

    double g_limit_factor_ = 1.0;
    double aoa_limit_factor_ = 1.0;
    double speed_limit_factor_ = 1.0;

    EnvelopeLimit current_limit_ = EnvelopeLimit::NONE;
};

//=============================================================================
// Flight Control System
//=============================================================================

/**
 * @brief Complete Flight Control System
 *
 * Integrates all FCS components:
 * - Control law selection (SAS/CAS/Autopilot)
 * - Envelope protection
 * - Auto-trim
 * - Control mixing
 */
class FlightControlSystem {
public:
    FlightControlSystem(const ControlLawConfig& config,
                       const FlightEnvelope& envelope);
    ~FlightControlSystem();

    /**
     * @brief Update FCS with new inputs and state
     * @param input Pilot control inputs
     * @param state Current flight state
     * @param dt Time step (s)
     * @return Control surface positions
     */
    SurfacePositions update(const ControlInput& input,
                           const FlightState& state,
                           double dt);

    /**
     * @brief Set control mode
     */
    void set_control_mode(ControlMode mode);
    ControlMode get_control_mode() const { return control_mode_; }

    /**
     * @brief Get autopilot interface
     */
    Autopilot& autopilot() { return autopilot_; }
    const Autopilot& autopilot() const { return autopilot_; }

    /**
     * @brief Set autopilot command
     */
    void set_autopilot_command(const AutopilotCommand& command);
    const AutopilotCommand& get_autopilot_command() const { return ap_command_; }

    /**
     * @brief Get envelope protection interface
     */
    EnvelopeProtection& envelope_protection() { return envelope_protection_; }
    const EnvelopeProtection& envelope_protection() const { return envelope_protection_; }

    /**
     * @brief Enable/disable auto-trim
     */
    void set_auto_trim(bool enabled) { auto_trim_enabled_ = enabled; }
    bool is_auto_trim_enabled() const { return auto_trim_enabled_; }

    /**
     * @brief Get current trim values
     */
    struct TrimState {
        double pitch_trim = 0.0;
        double roll_trim = 0.0;
        double yaw_trim = 0.0;
    };
    TrimState get_trim_state() const { return trim_state_; }

    /**
     * @brief Get current control law name
     */
    std::string get_active_control_law_name() const;

    /**
     * @brief Check for any active warnings
     */
    EnvelopeLimit get_active_warning() const;

    /**
     * @brief Reset FCS to initial state
     */
    void reset();

    /**
     * @brief Register callback for mode changes
     */
    using ModeChangeCallback = std::function<void(ControlMode, ControlMode)>;
    void set_mode_change_callback(ModeChangeCallback callback);

    /**
     * @brief Register callback for envelope warnings
     */
    using WarningCallback = std::function<void(EnvelopeLimit)>;
    void set_warning_callback(WarningCallback callback);

private:
    ControlLawConfig config_;
    ControlMode control_mode_ = ControlMode::CAS;

    // Control law implementations
    std::unique_ptr<SASController> sas_controller_;
    std::unique_ptr<CASController> cas_controller_;
    Autopilot autopilot_;
    AutopilotCommand ap_command_;

    // Envelope protection
    EnvelopeProtection envelope_protection_;

    // Auto-trim
    bool auto_trim_enabled_ = true;
    TrimState trim_state_;
    double trim_rate_ = 0.01;  // Trim rate per second

    // Previous outputs for rate limiting
    SurfacePositions prev_surfaces_;

    // Callbacks
    ModeChangeCallback mode_change_callback_;
    WarningCallback warning_callback_;

    // Internal methods
    void update_auto_trim(const FlightState& state, double dt);
    SurfacePositions apply_mixing(const SurfacePositions& raw,
                                  const FlightState& state);
    void handle_autopilot_buttons(const ControlInput& input,
                                  const FlightState& state);
};

//=============================================================================
// Guidance Laws (for missiles)
//=============================================================================

/**
 * @brief Proportional Navigation guidance law
 */
class ProportionalNavigation {
public:
    /**
     * @brief Constructor
     * @param navigation_constant N' (typically 3-5)
     */
    explicit ProportionalNavigation(double navigation_constant = 4.0);

    /**
     * @brief Calculate commanded acceleration
     * @param pursuer_position Missile position
     * @param pursuer_velocity Missile velocity
     * @param target_position Target position
     * @param target_velocity Target velocity
     * @return Commanded acceleration vector (m/s²)
     */
    math::Vec3 calculate(const math::Vec3& pursuer_position,
                        const math::Vec3& pursuer_velocity,
                        const math::Vec3& target_position,
                        const math::Vec3& target_velocity);

    void set_navigation_constant(double n) { nav_constant_ = n; }
    double get_navigation_constant() const { return nav_constant_; }

private:
    double nav_constant_;
    math::Vec3 prev_los_;
    bool first_update_ = true;
};

/**
 * @brief Augmented Proportional Navigation (APN)
 */
class AugmentedPN : public ProportionalNavigation {
public:
    explicit AugmentedPN(double navigation_constant = 4.0,
                         double target_accel_gain = 0.5);

    math::Vec3 calculate(const math::Vec3& pursuer_position,
                        const math::Vec3& pursuer_velocity,
                        const math::Vec3& target_position,
                        const math::Vec3& target_velocity,
                        const math::Vec3& target_acceleration);

private:
    double target_accel_gain_;
};

//=============================================================================
// Utility Functions
//=============================================================================

namespace FlightControlUtils {

/**
 * @brief Calculate load factor from acceleration
 */
inline double acceleration_to_g(double acceleration) {
    return acceleration / 9.81;
}

/**
 * @brief Calculate bank angle for coordinated turn
 * @param velocity True airspeed (m/s)
 * @param turn_rate Desired turn rate (rad/s)
 * @return Required bank angle (rad)
 */
double coordinated_turn_bank(double velocity, double turn_rate);

/**
 * @brief Calculate turn radius
 * @param velocity True airspeed (m/s)
 * @param bank_angle Bank angle (rad)
 * @return Turn radius (m)
 */
double turn_radius(double velocity, double bank_angle);

/**
 * @brief Calculate time to level off at vertical speed
 * @param current_altitude Current altitude (m)
 * @param target_altitude Target altitude (m)
 * @param vertical_speed Current vertical speed (m/s)
 * @param pitch_rate Maximum pitch rate (rad/s)
 * @return Time to begin level-off (s)
 */
double level_off_time(double current_altitude, double target_altitude,
                      double vertical_speed, double pitch_rate);

/**
 * @brief Standard rate turn (3 deg/s)
 */
constexpr double STANDARD_RATE_TURN = 0.0524;  // rad/s

/**
 * @brief Calculate ground speed from TAS and wind
 */
math::Vec3 calculate_ground_speed(const math::Vec3& tas_vector,
                                  const math::Vec3& wind_vector);

} // namespace FlightControlUtils

//=============================================================================
// Factory Functions
//=============================================================================

/**
 * @brief Create FCS for fighter aircraft
 */
std::unique_ptr<FlightControlSystem> create_fighter_fcs();

/**
 * @brief Create FCS for transport aircraft
 */
std::unique_ptr<FlightControlSystem> create_transport_fcs();

/**
 * @brief Create FCS for helicopter
 */
std::unique_ptr<FlightControlSystem> create_helicopter_fcs();

/**
 * @brief Create FCS for missile
 */
std::unique_ptr<FlightControlSystem> create_missile_fcs();

} // namespace dynamics
} // namespace jaguar
