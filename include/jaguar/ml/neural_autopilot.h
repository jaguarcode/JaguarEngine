#pragma once
/**
 * @file neural_autopilot.h
 * @brief Neural network-based autopilot for autonomous entity control
 *
 * This file provides ML-powered autopilot infrastructure for controlling
 * aircraft, ships, submarines, and other vehicles using trained neural networks.
 * Implements observation normalization, action execution, safety monitoring,
 * and multiple flight/navigation modes.
 *
 * Key features:
 * - ONNX neural network model loading and inference
 * - Multiple autopilot modes (waypoint, altitude hold, speed hold, etc.)
 * - Safety constraint enforcement with real-time monitoring
 * - Observation normalization and action denormalization
 * - Support for multiple vehicle types (aircraft, ships, etc.)
 * - Configurable control frequencies and safety limits
 * - Performance statistics and telemetry
 */

#include "jaguar/core/types.h"
#include <vector>
#include <memory>
#include <string>
#include <chrono>

namespace jaguar::ml {

// ============================================================================
// Forward Declarations
// ============================================================================

class IObservationNormalizer;
class IActionDenormalizer;
class ISafetyMonitor;
class NeuralAutopilot;

// ============================================================================
// Autopilot Result Enum
// ============================================================================

/**
 * @brief Result codes for autopilot operations
 */
enum class AutopilotResult : UInt8 {
    Success = 0,

    // Configuration errors
    InvalidConfiguration,
    InvalidModel,
    InvalidObservation,

    // Runtime errors
    InferenceFailed,
    ComputationError,
    SafetyViolation,

    // State errors
    NotInitialized,
    AlreadyInitialized,
    ModelNotLoaded,

    // Resource errors
    OutOfMemory
};

/**
 * @brief Convert AutopilotResult to string
 */
inline const char* autopilot_result_to_string(AutopilotResult result) {
    switch (result) {
        case AutopilotResult::Success: return "Success";
        case AutopilotResult::InvalidConfiguration: return "InvalidConfiguration";
        case AutopilotResult::InvalidModel: return "InvalidModel";
        case AutopilotResult::InvalidObservation: return "InvalidObservation";
        case AutopilotResult::InferenceFailed: return "InferenceFailed";
        case AutopilotResult::ComputationError: return "ComputationError";
        case AutopilotResult::SafetyViolation: return "SafetyViolation";
        case AutopilotResult::NotInitialized: return "NotInitialized";
        case AutopilotResult::AlreadyInitialized: return "AlreadyInitialized";
        case AutopilotResult::ModelNotLoaded: return "ModelNotLoaded";
        case AutopilotResult::OutOfMemory: return "OutOfMemory";
        default: return "Unknown";
    }
}

// ============================================================================
// Autopilot Mode Enum
// ============================================================================

/**
 * @brief Autopilot operational modes
 */
enum class AutopilotMode : UInt8 {
    Manual,      ///< No autopilot control - pilot commands only
    Waypoint,    ///< Navigate to waypoints
    Altitude,    ///< Maintain altitude
    Speed,       ///< Maintain speed
    Course,      ///< Maintain heading/course
    Formation,   ///< Formation flying/sailing
    Intercept,   ///< Intercept target
    Loiter       ///< Orbit/station keeping
};

/**
 * @brief Convert AutopilotMode to string
 */
inline const char* autopilot_mode_to_string(AutopilotMode mode) {
    switch (mode) {
        case AutopilotMode::Manual: return "Manual";
        case AutopilotMode::Waypoint: return "Waypoint";
        case AutopilotMode::Altitude: return "Altitude";
        case AutopilotMode::Speed: return "Speed";
        case AutopilotMode::Course: return "Course";
        case AutopilotMode::Formation: return "Formation";
        case AutopilotMode::Intercept: return "Intercept";
        case AutopilotMode::Loiter: return "Loiter";
        default: return "Unknown";
    }
}

// ============================================================================
// Vehicle Type Enum
// ============================================================================

/**
 * @brief Vehicle types for autopilot specialization
 */
enum class VehicleType : UInt8 {
    Aircraft,       ///< Fixed-wing aircraft
    Rotorcraft,     ///< Helicopter/quadcopter
    Ship,           ///< Surface vessel
    Submarine,      ///< Underwater vehicle
    GroundVehicle   ///< Land vehicle
};

/**
 * @brief Convert VehicleType to string
 */
inline const char* vehicle_type_to_string(VehicleType type) {
    switch (type) {
        case VehicleType::Aircraft: return "Aircraft";
        case VehicleType::Rotorcraft: return "Rotorcraft";
        case VehicleType::Ship: return "Ship";
        case VehicleType::Submarine: return "Submarine";
        case VehicleType::GroundVehicle: return "GroundVehicle";
        default: return "Unknown";
    }
}

// ============================================================================
// Quaternion (if not imported from elsewhere)
// ============================================================================

/**
 * @brief Quaternion for orientation representation
 *
 * Using Hamilton convention: q = w + xi + yj + zk
 */
struct Quaternion {
    Real w{1.0};  // Scalar part
    Real x{0.0};  // i component
    Real y{0.0};  // j component
    Real z{0.0};  // k component

    constexpr Quaternion() noexcept = default;
    constexpr Quaternion(Real w_, Real x_, Real y_, Real z_) noexcept
        : w(w_), x(x_), y(y_), z(z_) {}

    /**
     * @brief Convert quaternion to Euler angles (roll, pitch, yaw)
     * @param roll Roll angle in radians (output)
     * @param pitch Pitch angle in radians (output)
     * @param yaw Yaw angle in radians (output)
     */
    void to_euler(Real& roll, Real& pitch, Real& yaw) const noexcept;

    /**
     * @brief Create quaternion from Euler angles
     * @param roll Roll angle in radians
     * @param pitch Pitch angle in radians
     * @param yaw Yaw angle in radians
     * @return Quaternion representing the rotation
     */
    static Quaternion from_euler(Real roll, Real pitch, Real yaw) noexcept;

    /**
     * @brief Identity quaternion (no rotation)
     */
    static constexpr Quaternion Identity() noexcept {
        return {1.0, 0.0, 0.0, 0.0};
    }
};

// ============================================================================
// Structs
// ============================================================================

/**
 * @brief Autopilot observation/state vector
 *
 * Contains all sensor readings and state information needed for the
 * neural network to make control decisions.
 */
struct AutopilotObservation {
    Vec3 position{0.0, 0.0, 0.0};              ///< World coordinates (ECEF or local)
    Vec3 velocity{0.0, 0.0, 0.0};              ///< Body frame velocity
    Quaternion orientation{1.0, 0.0, 0.0, 0.0};///< Current orientation
    Vec3 angular_velocity{0.0, 0.0, 0.0};      ///< Body angular rates (roll, pitch, yaw rates)

    Real altitude{0.0};                         ///< Altitude above ground/sea level (m)
    Real airspeed{0.0};                         ///< Airspeed / groundspeed (m/s)
    Real heading{0.0};                          ///< Heading angle (radians)
    Real pitch{0.0};                            ///< Pitch angle (radians)
    Real roll{0.0};                             ///< Roll angle (radians)

    Vec3 target_position{0.0, 0.0, 0.0};       ///< Current waypoint/target position
    Real target_altitude{0.0};                  ///< Target altitude (m)
    Real target_speed{0.0};                     ///< Target speed (m/s)
    Real target_heading{0.0};                   ///< Target heading (radians)

    std::chrono::system_clock::time_point timestamp; ///< Observation timestamp
};

/**
 * @brief Autopilot control action/commands
 *
 * Contains all control surface and actuator commands output by the
 * neural network, normalized to standard ranges.
 */
struct AutopilotAction {
    Real elevator{0.0};     ///< Elevator control (-1 to 1, pitch control)
    Real aileron{0.0};      ///< Aileron control (-1 to 1, roll control)
    Real rudder{0.0};       ///< Rudder control (-1 to 1, yaw control)
    Real throttle{0.0};     ///< Throttle (0 to 1)
    Real collective{0.0};   ///< Collective pitch for rotorcraft (0 to 1)
    Real flaps{0.0};        ///< Flap setting (0 to 1)
    Real speedbrake{0.0};   ///< Speed brake deployment (0 to 1)
    bool gear_down{true};   ///< Landing gear state

    /**
     * @brief Create neutral control action (no commands)
     */
    static constexpr AutopilotAction neutral() noexcept {
        AutopilotAction action;
        action.elevator = 0.0;
        action.aileron = 0.0;
        action.rudder = 0.0;
        action.throttle = 0.0;
        action.collective = 0.0;
        action.flaps = 0.0;
        action.speedbrake = 0.0;
        action.gear_down = true;
        return action;
    }

    /**
     * @brief Create full throttle action
     */
    static constexpr AutopilotAction full_throttle() noexcept {
        AutopilotAction action = neutral();
        action.throttle = 1.0;
        return action;
    }

    /**
     * @brief Create idle throttle action
     */
    static constexpr AutopilotAction idle() noexcept {
        AutopilotAction action = neutral();
        action.throttle = 0.0;
        return action;
    }
};

/**
 * @brief Safety constraints for autopilot operation
 *
 * Defines operational limits to prevent dangerous maneuvers or
 * damage to the vehicle.
 */
struct SafetyConstraints {
    Real max_pitch_rate{10.0};      ///< Maximum pitch rate (deg/s)
    Real max_roll_rate{30.0};       ///< Maximum roll rate (deg/s)
    Real max_yaw_rate{10.0};        ///< Maximum yaw rate (deg/s)
    Real max_g_load{4.0};           ///< Maximum g-force
    Real min_altitude{0.0};         ///< Minimum altitude (m)
    Real max_altitude{15000.0};     ///< Maximum altitude (m)
    Real min_speed{20.0};           ///< Minimum safe speed (m/s)
    Real max_speed{250.0};          ///< Maximum safe speed (m/s)
    Real max_bank_angle{60.0};      ///< Maximum bank angle (degrees)
    Real max_pitch_angle{30.0};     ///< Maximum pitch angle (degrees)

    /**
     * @brief Default constraints for fixed-wing aircraft
     */
    static constexpr SafetyConstraints default_aircraft() noexcept {
        SafetyConstraints c;
        c.max_pitch_rate = 10.0;
        c.max_roll_rate = 30.0;
        c.max_yaw_rate = 10.0;
        c.max_g_load = 4.0;
        c.min_altitude = 0.0;
        c.max_altitude = 15000.0;
        c.min_speed = 20.0;
        c.max_speed = 250.0;
        c.max_bank_angle = 60.0;
        c.max_pitch_angle = 30.0;
        return c;
    }

    /**
     * @brief Default constraints for ships
     */
    static constexpr SafetyConstraints default_ship() noexcept {
        SafetyConstraints c;
        c.max_pitch_rate = 2.0;
        c.max_roll_rate = 5.0;
        c.max_yaw_rate = 5.0;
        c.max_g_load = 1.5;
        c.min_altitude = -10.0;      // Sea level with tolerance
        c.max_altitude = 10.0;
        c.min_speed = 0.0;
        c.max_speed = 30.0;
        c.max_bank_angle = 20.0;
        c.max_pitch_angle = 15.0;
        return c;
    }

    /**
     * @brief Permissive constraints for testing
     */
    static constexpr SafetyConstraints permissive() noexcept {
        SafetyConstraints c;
        c.max_pitch_rate = 30.0;
        c.max_roll_rate = 90.0;
        c.max_yaw_rate = 30.0;
        c.max_g_load = 9.0;
        c.min_altitude = -100.0;
        c.max_altitude = 30000.0;
        c.min_speed = 0.0;
        c.max_speed = 500.0;
        c.max_bank_angle = 90.0;
        c.max_pitch_angle = 60.0;
        return c;
    }

    /**
     * @brief Strict constraints for safety-critical applications
     */
    static constexpr SafetyConstraints strict() noexcept {
        SafetyConstraints c;
        c.max_pitch_rate = 5.0;
        c.max_roll_rate = 15.0;
        c.max_yaw_rate = 5.0;
        c.max_g_load = 2.5;
        c.min_altitude = 100.0;
        c.max_altitude = 10000.0;
        c.min_speed = 30.0;
        c.max_speed = 150.0;
        c.max_bank_angle = 30.0;
        c.max_pitch_angle = 20.0;
        return c;
    }
};

/**
 * @brief Autopilot configuration
 */
struct AutopilotConfig {
    VehicleType vehicle_type{VehicleType::Aircraft};    ///< Vehicle type
    std::string model_path;                              ///< Path to ONNX model file
    SafetyConstraints safety{SafetyConstraints::default_aircraft()}; ///< Safety limits
    Real control_frequency{50.0};                        ///< Control update rate (Hz)
    bool enable_safety_limits{true};                     ///< Enable safety constraint enforcement
    bool log_actions{false};                             ///< Log all actions for debugging

    /**
     * @brief Create configuration for aircraft
     * @param path Path to ONNX model
     */
    static AutopilotConfig aircraft(const std::string& path) noexcept {
        AutopilotConfig config;
        config.vehicle_type = VehicleType::Aircraft;
        config.model_path = path;
        config.safety = SafetyConstraints::default_aircraft();
        config.control_frequency = 50.0;
        config.enable_safety_limits = true;
        config.log_actions = false;
        return config;
    }

    /**
     * @brief Create configuration for ship
     * @param path Path to ONNX model
     */
    static AutopilotConfig ship(const std::string& path) noexcept {
        AutopilotConfig config;
        config.vehicle_type = VehicleType::Ship;
        config.model_path = path;
        config.safety = SafetyConstraints::default_ship();
        config.control_frequency = 10.0;  // Ships update slower
        config.enable_safety_limits = true;
        config.log_actions = false;
        return config;
    }

    /**
     * @brief Create configuration for rotorcraft
     * @param path Path to ONNX model
     */
    static AutopilotConfig rotorcraft(const std::string& path) noexcept {
        AutopilotConfig config;
        config.vehicle_type = VehicleType::Rotorcraft;
        config.model_path = path;
        config.safety = SafetyConstraints::default_aircraft();
        config.safety.max_roll_rate = 45.0;  // Rotorcraft can roll faster
        config.safety.min_speed = 0.0;       // Can hover
        config.control_frequency = 100.0;    // Higher frequency for stability
        config.enable_safety_limits = true;
        config.log_actions = false;
        return config;
    }
};

/**
 * @brief Autopilot performance statistics
 */
struct AutopilotStats {
    UInt64 total_steps{0};              ///< Total inference steps performed
    UInt64 safety_interventions{0};     ///< Number of safety constraint violations
    Real average_inference_ms{0.0};     ///< Average inference time (milliseconds)
    std::chrono::system_clock::time_point last_update; ///< Last update timestamp
};

// ============================================================================
// Interfaces
// ============================================================================

/**
 * @brief Interface for observation normalization
 *
 * Converts raw sensor readings to normalized neural network inputs.
 */
class IObservationNormalizer {
public:
    virtual ~IObservationNormalizer() = default;

    /**
     * @brief Normalize observation to neural network input
     * @param observation Raw observation data
     * @return Normalized vector for neural network
     */
    virtual std::vector<Real> normalize(const AutopilotObservation& observation) = 0;

    /**
     * @brief Denormalize neural network input back to observation
     * @param normalized Normalized vector
     * @return Reconstructed observation
     */
    virtual AutopilotObservation denormalize(const std::vector<Real>& normalized) = 0;
};

/**
 * @brief Interface for action denormalization
 *
 * Converts neural network outputs to actual control commands.
 */
class IActionDenormalizer {
public:
    virtual ~IActionDenormalizer() = default;

    /**
     * @brief Denormalize neural network output to control action
     * @param normalized Normalized neural network output
     * @return Control action
     */
    virtual AutopilotAction denormalize(const std::vector<Real>& normalized) = 0;

    /**
     * @brief Normalize control action to neural network output format
     * @param action Control action
     * @return Normalized vector
     */
    virtual std::vector<Real> normalize(const AutopilotAction& action) = 0;
};

/**
 * @brief Interface for safety monitoring
 *
 * Enforces safety constraints on autopilot actions.
 */
class ISafetyMonitor {
public:
    virtual ~ISafetyMonitor() = default;

    /**
     * @brief Apply safety constraints to an action
     * @param action Proposed action from neural network
     * @param observation Current state
     * @param constraints Safety constraints to enforce
     * @return Constrained action (modified if necessary)
     */
    virtual AutopilotAction apply_constraints(
        const AutopilotAction& action,
        const AutopilotObservation& observation,
        const SafetyConstraints& constraints) = 0;

    /**
     * @brief Check if an action is safe
     * @param action Proposed action
     * @param observation Current state
     * @param constraints Safety constraints
     * @return True if action is safe
     */
    virtual bool is_safe(
        const AutopilotAction& action,
        const AutopilotObservation& observation,
        const SafetyConstraints& constraints) = 0;
};

// ============================================================================
// Neural Autopilot Class
// ============================================================================

/**
 * @brief Neural network-based autopilot controller
 *
 * Uses trained ONNX neural network models to control vehicles autonomously.
 * Provides safety monitoring, observation normalization, and multiple
 * operational modes.
 *
 * Example usage:
 * @code
 * AutopilotConfig config = AutopilotConfig::aircraft("models/f16_autopilot.onnx");
 * auto autopilot = create_neural_autopilot(config);
 * autopilot->initialize();
 *
 * // Set waypoint mode
 * autopilot->set_mode(AutopilotMode::Waypoint);
 * autopilot->set_target(Vec3{1000.0, 2000.0, 500.0});
 *
 * // Compute control action
 * AutopilotObservation obs;
 * obs.position = Vec3{0.0, 0.0, 100.0};
 * obs.velocity = Vec3{50.0, 0.0, 0.0};
 * obs.orientation = Quaternion::Identity();
 * // ... fill in other observation data
 *
 * AutopilotAction action;
 * auto result = autopilot->compute(obs, action);
 * if (result == AutopilotResult::Success) {
 *     // Apply action to vehicle
 * }
 * @endcode
 */
class NeuralAutopilot {
public:
    /**
     * @brief Construct autopilot with configuration
     * @param config Autopilot configuration
     */
    explicit NeuralAutopilot(const AutopilotConfig& config);
    virtual ~NeuralAutopilot() = default;

    // ========================================================================
    // Lifecycle
    // ========================================================================

    /**
     * @brief Initialize the autopilot system
     * @return Success or error code
     */
    virtual AutopilotResult initialize() = 0;

    /**
     * @brief Shutdown the autopilot system
     * @return Success or error code
     */
    virtual AutopilotResult shutdown() = 0;

    /**
     * @brief Check if autopilot is initialized
     */
    virtual bool is_initialized() const = 0;

    // ========================================================================
    // Model Management
    // ========================================================================

    /**
     * @brief Load neural network model
     * @param path Path to ONNX model file
     * @return Success or error code
     */
    virtual AutopilotResult load_model(const std::string& path) = 0;

    // ========================================================================
    // Mode Control
    // ========================================================================

    /**
     * @brief Set autopilot mode
     * @param mode Desired autopilot mode
     * @return Success or error code
     */
    virtual AutopilotResult set_mode(AutopilotMode mode) = 0;

    /**
     * @brief Get current autopilot mode
     */
    virtual AutopilotMode get_mode() const = 0;

    // ========================================================================
    // Control Computation
    // ========================================================================

    /**
     * @brief Compute control action from observation
     * @param obs Current observation/state
     * @param action Output action (modified by function)
     * @return Success or error code
     */
    virtual AutopilotResult compute(const AutopilotObservation& obs,
                                    AutopilotAction& action) = 0;

    // ========================================================================
    // Target Setting
    // ========================================================================

    /**
     * @brief Set target position (for waypoint mode)
     * @param position Target position in world coordinates
     * @return Success or error code
     */
    virtual AutopilotResult set_target(const Vec3& position) = 0;

    /**
     * @brief Set target altitude
     * @param altitude Target altitude in meters
     * @return Success or error code
     */
    virtual AutopilotResult set_target_altitude(Real altitude) = 0;

    /**
     * @brief Set target speed
     * @param speed Target speed in m/s
     * @return Success or error code
     */
    virtual AutopilotResult set_target_speed(Real speed) = 0;

    /**
     * @brief Set target heading
     * @param heading Target heading in radians
     * @return Success or error code
     */
    virtual AutopilotResult set_target_heading(Real heading) = 0;

    // ========================================================================
    // Safety Configuration
    // ========================================================================

    /**
     * @brief Set safety constraints
     * @param constraints New safety constraints
     * @return Success or error code
     */
    virtual AutopilotResult set_safety_constraints(const SafetyConstraints& constraints) = 0;

    /**
     * @brief Get current safety constraints
     */
    virtual SafetyConstraints get_safety_constraints() const = 0;

    // ========================================================================
    // Statistics
    // ========================================================================

    /**
     * @brief Get performance statistics
     * @return Statistics summary
     */
    virtual AutopilotStats get_stats() const = 0;

    /**
     * @brief Get current configuration
     */
    virtual const AutopilotConfig& get_config() const = 0;
};

// ============================================================================
// Factory Functions
// ============================================================================

/**
 * @brief Create a neural autopilot instance
 * @param config Configuration for the autopilot
 * @return Unique pointer to the autopilot
 */
std::unique_ptr<NeuralAutopilot> create_neural_autopilot(
    const AutopilotConfig& config);

/**
 * @brief Create an aircraft autopilot with default settings
 * @param model_path Path to ONNX model
 * @return Unique pointer to the autopilot
 */
std::unique_ptr<NeuralAutopilot> create_aircraft_autopilot(
    const std::string& model_path);

/**
 * @brief Create a ship autopilot with default settings
 * @param model_path Path to ONNX model
 * @return Unique pointer to the autopilot
 */
std::unique_ptr<NeuralAutopilot> create_ship_autopilot(
    const std::string& model_path);

/**
 * @brief Create default observation normalizer for vehicle type
 * @param vehicle_type Type of vehicle
 * @return Unique pointer to normalizer
 */
std::unique_ptr<IObservationNormalizer> create_default_normalizer(
    VehicleType vehicle_type);

/**
 * @brief Create default action denormalizer for vehicle type
 * @param vehicle_type Type of vehicle
 * @return Unique pointer to denormalizer
 */
std::unique_ptr<IActionDenormalizer> create_default_denormalizer(
    VehicleType vehicle_type);

/**
 * @brief Create default safety monitor
 * @return Unique pointer to safety monitor
 */
std::unique_ptr<ISafetyMonitor> create_default_safety_monitor();

// ============================================================================
// Inline Helper Functions
// ============================================================================

/**
 * @brief Clamp value to range
 */
inline Real clamp(Real value, Real min_val, Real max_val) noexcept {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

/**
 * @brief Clamp action to valid ranges
 * @param action Action to clamp
 * @return Clamped action
 */
inline AutopilotAction clamp_action(const AutopilotAction& action) noexcept {
    AutopilotAction clamped = action;
    clamped.elevator = clamp(action.elevator, -1.0, 1.0);
    clamped.aileron = clamp(action.aileron, -1.0, 1.0);
    clamped.rudder = clamp(action.rudder, -1.0, 1.0);
    clamped.throttle = clamp(action.throttle, 0.0, 1.0);
    clamped.collective = clamp(action.collective, 0.0, 1.0);
    clamped.flaps = clamp(action.flaps, 0.0, 1.0);
    clamped.speedbrake = clamp(action.speedbrake, 0.0, 1.0);
    return clamped;
}

/**
 * @brief Validate observation data
 * @param obs Observation to validate
 * @return True if observation is valid
 */
inline bool validate_observation(const AutopilotObservation& obs) noexcept {
    // Check for NaN or Inf values
    auto is_finite = [](Real x) {
        return x == x && x != std::numeric_limits<Real>::infinity() &&
               x != -std::numeric_limits<Real>::infinity();
    };

    if (!is_finite(obs.position.x) || !is_finite(obs.position.y) || !is_finite(obs.position.z))
        return false;
    if (!is_finite(obs.velocity.x) || !is_finite(obs.velocity.y) || !is_finite(obs.velocity.z))
        return false;
    if (!is_finite(obs.orientation.w) || !is_finite(obs.orientation.x) ||
        !is_finite(obs.orientation.y) || !is_finite(obs.orientation.z))
        return false;
    if (!is_finite(obs.altitude) || !is_finite(obs.airspeed) || !is_finite(obs.heading))
        return false;
    if (!is_finite(obs.pitch) || !is_finite(obs.roll))
        return false;

    // Check for reasonable ranges
    if (obs.altitude < -500.0 || obs.altitude > 50000.0) return false;  // Altitude bounds
    if (obs.airspeed < 0.0 || obs.airspeed > 1000.0) return false;      // Speed bounds

    return true;
}

/**
 * @brief Validate action data
 * @param action Action to validate
 * @return True if action is valid
 */
inline bool validate_action(const AutopilotAction& action) noexcept {
    auto is_finite = [](Real x) {
        return x == x && x != std::numeric_limits<Real>::infinity() &&
               x != -std::numeric_limits<Real>::infinity();
    };

    if (!is_finite(action.elevator) || !is_finite(action.aileron) || !is_finite(action.rudder))
        return false;
    if (!is_finite(action.throttle) || !is_finite(action.collective))
        return false;
    if (!is_finite(action.flaps) || !is_finite(action.speedbrake))
        return false;

    return true;
}

/**
 * @brief Calculate distance to target
 * @param current Current position
 * @param target Target position
 * @return Euclidean distance
 */
inline Real distance_to_target(const Vec3& current, const Vec3& target) noexcept {
    Vec3 delta = target - current;
    return std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
}

/**
 * @brief Calculate heading error
 * @param current_heading Current heading (radians)
 * @param target_heading Target heading (radians)
 * @return Heading error in range [-pi, pi]
 */
inline Real heading_error(Real current_heading, Real target_heading) noexcept {
    Real error = target_heading - current_heading;
    // Normalize to [-pi, pi]
    while (error > constants::PI) error -= 2.0 * constants::PI;
    while (error < -constants::PI) error += 2.0 * constants::PI;
    return error;
}

/**
 * @brief Calculate altitude error
 * @param current_altitude Current altitude (m)
 * @param target_altitude Target altitude (m)
 * @return Altitude error
 */
inline Real altitude_error(Real current_altitude, Real target_altitude) noexcept {
    return target_altitude - current_altitude;
}

/**
 * @brief Calculate speed error
 * @param current_speed Current speed (m/s)
 * @param target_speed Target speed (m/s)
 * @return Speed error
 */
inline Real speed_error(Real current_speed, Real target_speed) noexcept {
    return target_speed - current_speed;
}

/**
 * @brief Calculate bearing to target
 * @param current Current position
 * @param target Target position
 * @return Bearing in radians (0 = North, increases clockwise)
 */
inline Real bearing_to_target(const Vec3& current, const Vec3& target) noexcept {
    Vec3 delta = target - current;
    return std::atan2(delta.x, delta.y);  // atan2(east, north) for NED frame
}

/**
 * @brief Convert degrees to radians
 */
inline constexpr Real deg_to_rad(Real degrees) noexcept {
    return degrees * constants::DEG_TO_RAD;
}

/**
 * @brief Convert radians to degrees
 */
inline constexpr Real rad_to_deg(Real radians) noexcept {
    return radians * constants::RAD_TO_DEG;
}

/**
 * @brief Interpolate between two actions
 * @param a First action
 * @param b Second action
 * @param t Interpolation parameter (0 = a, 1 = b)
 * @return Interpolated action
 */
inline AutopilotAction interpolate_actions(const AutopilotAction& a,
                                           const AutopilotAction& b,
                                           Real t) noexcept {
    t = clamp(t, 0.0, 1.0);
    AutopilotAction result;
    result.elevator = a.elevator + t * (b.elevator - a.elevator);
    result.aileron = a.aileron + t * (b.aileron - a.aileron);
    result.rudder = a.rudder + t * (b.rudder - a.rudder);
    result.throttle = a.throttle + t * (b.throttle - a.throttle);
    result.collective = a.collective + t * (b.collective - a.collective);
    result.flaps = a.flaps + t * (b.flaps - a.flaps);
    result.speedbrake = a.speedbrake + t * (b.speedbrake - a.speedbrake);
    result.gear_down = t < 0.5 ? a.gear_down : b.gear_down;
    return result;
}

/**
 * @brief Calculate rate of change for a control surface
 * @param current Current value
 * @param target Target value
 * @param max_rate Maximum rate of change per second
 * @param dt Time delta in seconds
 * @return New value respecting rate limit
 */
inline Real rate_limited_change(Real current, Real target, Real max_rate, Real dt) noexcept {
    Real max_change = max_rate * dt;
    Real desired_change = target - current;
    if (desired_change > max_change) return current + max_change;
    if (desired_change < -max_change) return current - max_change;
    return target;
}

/**
 * @brief Apply rate limits to action
 * @param current Current action
 * @param target Target action
 * @param max_rate Maximum rate of change (per second)
 * @param dt Time delta in seconds
 * @return Rate-limited action
 */
inline AutopilotAction apply_rate_limits(const AutopilotAction& current,
                                        const AutopilotAction& target,
                                        Real max_rate,
                                        Real dt) noexcept {
    AutopilotAction result;
    result.elevator = rate_limited_change(current.elevator, target.elevator, max_rate, dt);
    result.aileron = rate_limited_change(current.aileron, target.aileron, max_rate, dt);
    result.rudder = rate_limited_change(current.rudder, target.rudder, max_rate, dt);
    result.throttle = rate_limited_change(current.throttle, target.throttle, max_rate * 0.5, dt);
    result.collective = rate_limited_change(current.collective, target.collective, max_rate, dt);
    result.flaps = rate_limited_change(current.flaps, target.flaps, max_rate * 0.3, dt);
    result.speedbrake = rate_limited_change(current.speedbrake, target.speedbrake, max_rate, dt);
    result.gear_down = target.gear_down;
    return result;
}

/**
 * @brief Check if vehicle is at target altitude
 * @param current_altitude Current altitude
 * @param target_altitude Target altitude
 * @param tolerance Tolerance in meters
 * @return True if within tolerance
 */
inline bool at_target_altitude(Real current_altitude, Real target_altitude,
                               Real tolerance = 10.0) noexcept {
    return std::abs(current_altitude - target_altitude) <= tolerance;
}

/**
 * @brief Check if vehicle is at target speed
 * @param current_speed Current speed
 * @param target_speed Target speed
 * @param tolerance Tolerance in m/s
 * @return True if within tolerance
 */
inline bool at_target_speed(Real current_speed, Real target_speed,
                           Real tolerance = 2.0) noexcept {
    return std::abs(current_speed - target_speed) <= tolerance;
}

/**
 * @brief Check if vehicle is at target heading
 * @param current_heading Current heading (radians)
 * @param target_heading Target heading (radians)
 * @param tolerance Tolerance in radians
 * @return True if within tolerance
 */
inline bool at_target_heading(Real current_heading, Real target_heading,
                              Real tolerance = deg_to_rad(5.0)) noexcept {
    return std::abs(heading_error(current_heading, target_heading)) <= tolerance;
}

/**
 * @brief Check if vehicle has reached waypoint
 * @param current Current position
 * @param target Target position
 * @param tolerance Tolerance in meters
 * @return True if within tolerance
 */
inline bool at_waypoint(const Vec3& current, const Vec3& target,
                       Real tolerance = 50.0) noexcept {
    return distance_to_target(current, target) <= tolerance;
}

/**
 * @brief Calculate control surface effectiveness based on airspeed
 * @param airspeed Current airspeed (m/s)
 * @param min_speed Minimum speed for full effectiveness
 * @return Effectiveness factor (0-1)
 */
inline Real control_effectiveness(Real airspeed, Real min_speed = 20.0) noexcept {
    if (airspeed >= min_speed) return 1.0;
    if (airspeed <= 0.0) return 0.0;
    return airspeed / min_speed;
}

/**
 * @brief Estimate time to target
 * @param distance Distance to target (m)
 * @param current_speed Current speed (m/s)
 * @return Estimated time in seconds
 */
inline Real time_to_target(Real distance, Real current_speed) noexcept {
    if (current_speed <= 0.0) return std::numeric_limits<Real>::infinity();
    return distance / current_speed;
}

/**
 * @brief Calculate turn radius for coordinated turn
 * @param velocity Velocity (m/s)
 * @param bank_angle Bank angle (radians)
 * @param g Gravitational acceleration (m/s²)
 * @return Turn radius in meters
 */
inline Real turn_radius(Real velocity, Real bank_angle,
                       Real g = constants::G0) noexcept {
    if (std::abs(bank_angle) < 0.001) return std::numeric_limits<Real>::infinity();
    return (velocity * velocity) / (g * std::tan(bank_angle));
}

/**
 * @brief Calculate required bank angle for turn
 * @param velocity Velocity (m/s)
 * @param desired_radius Desired turn radius (m)
 * @param g Gravitational acceleration (m/s²)
 * @return Required bank angle in radians
 */
inline Real required_bank_angle(Real velocity, Real desired_radius,
                               Real g = constants::G0) noexcept {
    if (desired_radius <= 0.0) return 0.0;
    return std::atan((velocity * velocity) / (g * desired_radius));
}

/**
 * @brief Calculate load factor (g-loading) from bank angle
 * @param bank_angle Bank angle (radians)
 * @return Load factor (1.0 = level flight)
 */
inline Real load_factor_from_bank(Real bank_angle) noexcept {
    return 1.0 / std::cos(bank_angle);
}

/**
 * @brief Generate observation report string
 * @param obs Observation to report
 * @return Human-readable string
 */
inline std::string generate_observation_report(const AutopilotObservation& obs) {
    std::string report = "Observation Report:\n";
    report += "Position: (" + std::to_string(obs.position.x) + ", " +
              std::to_string(obs.position.y) + ", " +
              std::to_string(obs.position.z) + ")\n";
    report += "Velocity: (" + std::to_string(obs.velocity.x) + ", " +
              std::to_string(obs.velocity.y) + ", " +
              std::to_string(obs.velocity.z) + ")\n";
    report += "Altitude: " + std::to_string(obs.altitude) + " m\n";
    report += "Airspeed: " + std::to_string(obs.airspeed) + " m/s\n";
    report += "Heading: " + std::to_string(rad_to_deg(obs.heading)) + " deg\n";
    report += "Pitch: " + std::to_string(rad_to_deg(obs.pitch)) + " deg\n";
    report += "Roll: " + std::to_string(rad_to_deg(obs.roll)) + " deg\n";
    return report;
}

/**
 * @brief Generate action report string
 * @param action Action to report
 * @return Human-readable string
 */
inline std::string generate_action_report(const AutopilotAction& action) {
    std::string report = "Action Report:\n";
    report += "Elevator: " + std::to_string(action.elevator) + "\n";
    report += "Aileron: " + std::to_string(action.aileron) + "\n";
    report += "Rudder: " + std::to_string(action.rudder) + "\n";
    report += "Throttle: " + std::to_string(action.throttle * 100.0) + "%\n";
    report += "Collective: " + std::to_string(action.collective * 100.0) + "%\n";
    report += "Flaps: " + std::to_string(action.flaps * 100.0) + "%\n";
    report += "Speedbrake: " + std::to_string(action.speedbrake * 100.0) + "%\n";
    report += "Gear: " + std::string(action.gear_down ? "DOWN" : "UP") + "\n";
    return report;
}

/**
 * @brief Create observation with target from waypoint
 * @param obs Base observation
 * @param waypoint Target waypoint
 * @return Observation with updated target fields
 */
inline AutopilotObservation set_waypoint_target(AutopilotObservation obs,
                                               const Vec3& waypoint) noexcept {
    obs.target_position = waypoint;
    obs.target_altitude = waypoint.z;
    return obs;
}

/**
 * @brief Calculate stall speed estimate (basic)
 * @param mass Vehicle mass (kg)
 * @param wing_area Wing area (m²)
 * @param cl_max Maximum lift coefficient
 * @param air_density Air density (kg/m³)
 * @return Estimated stall speed (m/s)
 */
inline Real estimate_stall_speed(Real mass, Real wing_area, Real cl_max,
                                Real air_density = constants::RHO0) noexcept {
    if (wing_area <= 0.0 || cl_max <= 0.0 || air_density <= 0.0) return 0.0;
    Real weight = mass * constants::G0;
    return std::sqrt((2.0 * weight) / (air_density * wing_area * cl_max));
}

/**
 * @brief Normalize angle to [-pi, pi]
 * @param angle Angle in radians
 * @return Normalized angle
 */
inline Real normalize_angle(Real angle) noexcept {
    while (angle > constants::PI) angle -= 2.0 * constants::PI;
    while (angle < -constants::PI) angle += 2.0 * constants::PI;
    return angle;
}

} // namespace jaguar::ml
