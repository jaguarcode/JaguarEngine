#pragma once
/**
 * @file sensor.h
 * @brief Sensor framework interface and base classes
 *
 * This file defines the core sensor abstraction for JaguarEngine, providing
 * a unified interface for all sensor types including IMU, GPS, radar, lidar,
 * and custom sensors. The framework supports configurable noise models,
 * sampling rates, and failure modes.
 *
 * Key Features:
 * - Noise modeling (bias, random walk, white noise)
 * - Configurable update rates and latency
 * - Sensor fusion support
 * - Failure mode simulation
 * - Event system integration
 */

#include "jaguar/core/types.h"
#include <string>
#include <memory>
#include <functional>
#include <random>
#include <vector>

// Forward declarations
namespace jaguar::events {
    class EventDispatcher;
    enum class EventType : UInt32;
}

namespace jaguar::physics {
    struct EntityState;
    class EntityManager;
}

namespace jaguar::sensors {

// ============================================================================
// Sensor Types and Configuration
// ============================================================================

/**
 * @brief Enumeration of sensor types
 */
enum class SensorType : UInt32 {
    IMU             = 0,    ///< Inertial Measurement Unit (accelerometer + gyroscope)
    GPS             = 1,    ///< Global Positioning System
    Altimeter       = 2,    ///< Barometric/radar altimeter
    Airspeed        = 3,    ///< Pitot-static airspeed sensor
    Magnetometer    = 4,    ///< Magnetic heading sensor
    Radar           = 5,    ///< Radar sensor (range/velocity)
    Lidar           = 6,    ///< Light Detection and Ranging
    Camera          = 7,    ///< Vision sensor
    Infrared        = 8,    ///< Infrared/thermal sensor
    Sonar           = 9,    ///< Underwater acoustic sensor
    DepthGauge      = 10,   ///< Pressure-based depth sensor
    Custom          = 100   ///< User-defined sensor type
};

/**
 * @brief Sensor operational state
 */
enum class SensorState : UInt8 {
    Uninitialized   = 0,    ///< Not yet initialized
    Initializing    = 1,    ///< Warming up / calibrating
    Operational     = 2,    ///< Normal operation
    Degraded        = 3,    ///< Operating with reduced accuracy
    Failed          = 4,    ///< Complete failure
    Disabled        = 5     ///< Manually disabled
};

/**
 * @brief Sensor failure mode enumeration
 */
enum class FailureMode : UInt8 {
    None            = 0,    ///< No failure
    Bias            = 1,    ///< Constant bias error
    Drift           = 2,    ///< Time-varying drift
    Noise           = 3,    ///< Increased noise level
    Saturation      = 4,    ///< Output clipped to limits
    Dropout         = 5,    ///< Intermittent signal loss
    Stuck           = 6,    ///< Output frozen at last value
    Oscillation     = 7,    ///< Spurious oscillations
    Complete        = 8     ///< Total failure (no output)
};

/**
 * @brief Get human-readable name for sensor type
 */
const char* get_sensor_type_name(SensorType type);

/**
 * @brief Get human-readable name for sensor state
 */
const char* get_sensor_state_name(SensorState state);

// ============================================================================
// Noise Models
// ============================================================================

/**
 * @brief White noise (Gaussian) parameters
 */
struct WhiteNoiseParams {
    Real sigma{0.0};        ///< Standard deviation
    bool enabled{true};     ///< Enable/disable this noise source
};

/**
 * @brief Bias instability parameters (flicker noise / 1/f noise)
 *
 * Bias instability models the slowly varying component of sensor bias
 * that cannot be calibrated out. It's characterized by Allan variance
 * at long correlation times.
 */
struct BiasInstabilityParams {
    Real sigma{0.0};        ///< Bias instability (units/sqrt(Hz))
    Real correlation_time{100.0}; ///< Correlation time constant (s)
    bool enabled{true};
};

/**
 * @brief Random walk noise parameters (integrated white noise)
 *
 * Random walk models the accumulation of noise over time, particularly
 * important for rate-integrating sensors like gyroscopes.
 */
struct RandomWalkParams {
    Real sigma{0.0};        ///< Random walk coefficient (units/sqrt(s))
    bool enabled{true};
};

/**
 * @brief Quantization noise parameters
 */
struct QuantizationParams {
    Real resolution{0.0};   ///< Minimum resolvable change
    bool enabled{false};
};

/**
 * @brief Complete noise model for a single axis
 */
struct AxisNoiseModel {
    WhiteNoiseParams white_noise;
    BiasInstabilityParams bias_instability;
    RandomWalkParams random_walk;
    QuantizationParams quantization;

    Real constant_bias{0.0};    ///< Fixed calibration bias
    Real scale_factor_error{0.0}; ///< Scale factor error (fraction)
    Real misalignment{0.0};     ///< Axis misalignment (rad)

    /**
     * @brief Check if any noise sources are enabled
     */
    bool has_noise() const {
        return white_noise.enabled || bias_instability.enabled ||
               random_walk.enabled || quantization.enabled;
    }
};

/**
 * @brief 3-axis noise model for vector sensors
 */
struct Vec3NoiseModel {
    AxisNoiseModel x;
    AxisNoiseModel y;
    AxisNoiseModel z;

    /**
     * @brief Set uniform noise parameters for all axes
     */
    void set_uniform(const AxisNoiseModel& model) {
        x = y = z = model;
    }

    /**
     * @brief Set only white noise with specified sigma for all axes
     */
    void set_white_noise(Real sigma) {
        x.white_noise.sigma = sigma;
        y.white_noise.sigma = sigma;
        z.white_noise.sigma = sigma;
    }
};

// ============================================================================
// Sensor Configuration
// ============================================================================

/**
 * @brief Base sensor configuration
 */
struct SensorConfig {
    std::string name;           ///< Sensor instance name
    SensorType type{SensorType::Custom};
    EntityId attached_entity{INVALID_ENTITY_ID}; ///< Entity sensor is mounted on

    // Timing
    Real update_rate{100.0};    ///< Update frequency (Hz)
    Real latency{0.0};          ///< Measurement latency (s)
    Real warmup_time{0.0};      ///< Time to reach operational state (s)

    // Mounting
    Vec3 mount_position{0.0, 0.0, 0.0};  ///< Position in body frame (m)
    Quat mount_orientation{1.0, 0.0, 0.0, 0.0}; ///< Orientation in body frame

    // Range limits
    Real range_min{-1e10};      ///< Minimum measurable value
    Real range_max{1e10};       ///< Maximum measurable value

    // Initial state
    SensorState initial_state{SensorState::Operational};
    bool auto_initialize{true}; ///< Auto-transition to operational after warmup
};

// ============================================================================
// Sensor Measurement
// ============================================================================

/**
 * @brief Sensor measurement result with metadata
 */
template<typename T>
struct SensorMeasurement {
    T value;                    ///< Measured value
    Real timestamp{0.0};        ///< Simulation time of measurement
    Real measurement_time{0.0}; ///< Time when physical measurement occurred
    bool valid{true};           ///< Is measurement valid
    Real confidence{1.0};       ///< Confidence level [0,1]
    UInt64 sequence_number{0};  ///< Measurement sequence for tracking

    SensorMeasurement() = default;
    explicit SensorMeasurement(const T& v, Real ts = 0.0)
        : value(v), timestamp(ts), measurement_time(ts), valid(true) {}
};

// Common measurement types
using ScalarMeasurement = SensorMeasurement<Real>;
using Vec3Measurement = SensorMeasurement<Vec3>;
using QuatMeasurement = SensorMeasurement<Quat>;

// ============================================================================
// Sensor Interface
// ============================================================================

/**
 * @brief Abstract base class for all sensors
 *
 * ISensor defines the common interface that all sensor implementations must
 * provide. It handles noise generation, state management, and event integration.
 *
 * Derived classes must implement:
 * - do_update(): Compute the ideal (noise-free) measurement
 * - do_apply_noise(): Apply sensor-specific noise models
 *
 * Usage:
 * @code
 * auto imu = std::make_unique<IMUSensor>(config);
 * imu->initialize();
 *
 * // In simulation loop:
 * imu->update(entity_state, dt);
 * auto accel = imu->get_acceleration();
 * auto gyro = imu->get_angular_velocity();
 * @endcode
 */
class ISensor {
public:
    explicit ISensor(const SensorConfig& config);
    virtual ~ISensor() = default;

    // Non-copyable but movable
    ISensor(const ISensor&) = delete;
    ISensor& operator=(const ISensor&) = delete;
    ISensor(ISensor&&) = default;
    ISensor& operator=(ISensor&&) = default;

    // ========================================================================
    // Core Interface
    // ========================================================================

    /**
     * @brief Initialize sensor (start warmup if configured)
     * @return true if initialization successful
     */
    virtual bool initialize();

    /**
     * @brief Update sensor with current entity state
     *
     * This is the main update method called each simulation step.
     * It handles timing, noise injection, and state management.
     *
     * @param entity_state Current state of attached entity
     * @param dt Simulation time step (s)
     */
    void update(const physics::EntityState& entity_state, Real dt);

    /**
     * @brief Reset sensor to initial state
     */
    virtual void reset();

    /**
     * @brief Shutdown sensor
     */
    virtual void shutdown();

    // ========================================================================
    // State Management
    // ========================================================================

    /**
     * @brief Get current sensor state
     */
    SensorState get_state() const { return state_; }

    /**
     * @brief Set sensor state directly (for failure injection)
     */
    void set_state(SensorState state);

    /**
     * @brief Check if sensor is operational
     */
    bool is_operational() const {
        return state_ == SensorState::Operational ||
               state_ == SensorState::Degraded;
    }

    /**
     * @brief Enable/disable sensor
     */
    void set_enabled(bool enabled);

    /**
     * @brief Check if sensor is enabled
     */
    bool is_enabled() const { return enabled_; }

    // ========================================================================
    // Failure Injection
    // ========================================================================

    /**
     * @brief Inject a failure mode
     * @param mode Failure mode to inject
     * @param magnitude Severity of failure (mode-specific)
     */
    void inject_failure(FailureMode mode, Real magnitude = 1.0);

    /**
     * @brief Clear all injected failures
     */
    void clear_failures();

    /**
     * @brief Get current failure mode
     */
    FailureMode get_failure_mode() const { return failure_mode_; }

    // ========================================================================
    // Configuration and Metadata
    // ========================================================================

    /**
     * @brief Get sensor name
     */
    const std::string& name() const { return config_.name; }

    /**
     * @brief Get sensor type
     */
    SensorType type() const { return config_.type; }

    /**
     * @brief Get sensor ID
     */
    UInt64 id() const { return sensor_id_; }

    /**
     * @brief Get attached entity ID
     */
    EntityId attached_entity() const { return config_.attached_entity; }

    /**
     * @brief Get sensor configuration
     */
    const SensorConfig& config() const { return config_; }

    /**
     * @brief Get update rate (Hz)
     */
    Real update_rate() const { return config_.update_rate; }

    /**
     * @brief Get time since last update
     */
    Real time_since_update() const { return time_since_update_; }

    /**
     * @brief Get total number of updates
     */
    UInt64 update_count() const { return update_count_; }

    // ========================================================================
    // Event System Integration
    // ========================================================================

    /**
     * @brief Set event dispatcher for sensor events
     */
    void set_event_dispatcher(events::EventDispatcher* dispatcher) {
        event_dispatcher_ = dispatcher;
    }

    /**
     * @brief Set current simulation time (for event timestamps)
     */
    void set_simulation_time(Real time) { simulation_time_ = time; }

    // ========================================================================
    // Random Number Generation
    // ========================================================================

    /**
     * @brief Set random seed for reproducible noise
     */
    void set_seed(UInt64 seed);

    /**
     * @brief Get reference to random engine
     */
    std::mt19937_64& rng() { return rng_; }

protected:
    // ========================================================================
    // Virtual Methods for Derived Classes
    // ========================================================================

    /**
     * @brief Compute ideal measurement (derived class implements)
     *
     * This method should compute the sensor measurement assuming perfect
     * sensor behavior (no noise, no errors).
     *
     * @param entity_state Current entity state
     */
    virtual void do_update(const physics::EntityState& entity_state) = 0;

    /**
     * @brief Apply noise model to measurement (derived class implements)
     *
     * This method applies sensor-specific noise models including
     * white noise, bias, random walk, etc.
     *
     * @param dt Time since last update
     */
    virtual void do_apply_noise(Real dt) = 0;

    /**
     * @brief Apply failure effects (derived class can override)
     */
    virtual void do_apply_failure();

    /**
     * @brief Handle state transition (derived class can override)
     */
    virtual void on_state_change(SensorState old_state, SensorState new_state);

    // ========================================================================
    // Noise Generation Utilities
    // ========================================================================

    /**
     * @brief Generate Gaussian white noise
     */
    Real generate_white_noise(Real sigma);

    /**
     * @brief Generate 3D white noise vector
     */
    Vec3 generate_white_noise_vec3(const Vec3NoiseModel& model);

    /**
     * @brief Apply bias instability (first-order Gauss-Markov process)
     */
    Real apply_bias_instability(Real& bias_state,
                                const BiasInstabilityParams& params, Real dt);

    /**
     * @brief Apply random walk
     */
    Real apply_random_walk(Real& walk_state,
                           const RandomWalkParams& params, Real dt);

    /**
     * @brief Apply quantization
     */
    Real apply_quantization(Real value, Real resolution);

    /**
     * @brief Clamp value to sensor range
     */
    Real clamp_to_range(Real value) const;

    /**
     * @brief Emit sensor event
     */
    void emit_event(events::EventType event_type);

    // Configuration
    SensorConfig config_;

    // State
    SensorState state_{SensorState::Uninitialized};
    bool enabled_{true};
    FailureMode failure_mode_{FailureMode::None};
    Real failure_magnitude_{0.0};

    // Timing
    Real time_since_update_{0.0};
    Real warmup_elapsed_{0.0};
    Real simulation_time_{0.0};
    UInt64 update_count_{0};
    UInt64 sequence_number_{0};

    // Random number generation
    std::mt19937_64 rng_;
    std::normal_distribution<Real> normal_dist_{0.0, 1.0};

    // Event system
    events::EventDispatcher* event_dispatcher_{nullptr};

private:
    static UInt64 next_sensor_id_;
    UInt64 sensor_id_;

    // Check if update is due based on update rate
    bool is_update_due(Real dt);

    // Handle warmup phase
    void handle_warmup(Real dt);
};

// ============================================================================
// Sensor Manager
// ============================================================================

/**
 * @brief Manages multiple sensors attached to entities
 *
 * SensorManager provides centralized management of all sensors in a simulation,
 * handling updates, event dispatch, and sensor queries.
 */
class SensorManager {
public:
    SensorManager();
    ~SensorManager();

    /**
     * @brief Add a sensor to the manager
     * @param sensor Sensor to add (takes ownership)
     * @return Sensor ID
     */
    UInt64 add_sensor(std::unique_ptr<ISensor> sensor);

    /**
     * @brief Remove a sensor by ID
     */
    void remove_sensor(UInt64 sensor_id);

    /**
     * @brief Get sensor by ID
     */
    ISensor* get_sensor(UInt64 sensor_id);
    const ISensor* get_sensor(UInt64 sensor_id) const;

    /**
     * @brief Get all sensors attached to an entity
     */
    std::vector<ISensor*> get_sensors_for_entity(EntityId entity_id);

    /**
     * @brief Get all sensors of a specific type
     */
    std::vector<ISensor*> get_sensors_by_type(SensorType type);

    /**
     * @brief Update all sensors with entity states
     *
     * @param entity_manager Entity manager for state lookup
     * @param dt Time step
     */
    void update_all(physics::EntityManager& entity_manager, Real dt);

    /**
     * @brief Initialize all sensors
     */
    void initialize_all();

    /**
     * @brief Reset all sensors
     */
    void reset_all();

    /**
     * @brief Shutdown all sensors
     */
    void shutdown_all();

    /**
     * @brief Get total number of sensors
     */
    SizeT sensor_count() const { return sensors_.size(); }

    /**
     * @brief Set event dispatcher for all sensors
     */
    void set_event_dispatcher(events::EventDispatcher* dispatcher);

    /**
     * @brief Set simulation time for all sensors
     */
    void set_simulation_time(Real time);

    /**
     * @brief Iterate over all sensors
     */
    template<typename Func>
    void for_each(Func&& func) {
        for (auto& [id, sensor] : sensors_) {
            func(*sensor);
        }
    }

private:
    std::unordered_map<UInt64, std::unique_ptr<ISensor>> sensors_;
    events::EventDispatcher* event_dispatcher_{nullptr};
};

} // namespace jaguar::sensors
