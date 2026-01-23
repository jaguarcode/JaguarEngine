#pragma once
/**
 * @file imu_sensor.h
 * @brief Inertial Measurement Unit (IMU) sensor simulation
 *
 * This file implements a high-fidelity IMU sensor model including:
 * - 3-axis accelerometer with configurable noise
 * - 3-axis gyroscope with configurable noise
 * - MIL-SPEC compliant noise models
 * - Bias instability and random walk
 * - Temperature effects (optional)
 * - G-sensitivity of gyros
 *
 * The IMU model follows IEEE/AIAA standards for inertial sensor modeling
 * and supports validation against MIL-STD-1553 and MIL-F-8785C specifications.
 *
 * Noise Model Components:
 * 1. White noise (angle/velocity random walk)
 * 2. Bias instability (1/f flicker noise)
 * 3. Bias random walk (rate random walk)
 * 4. Quantization noise
 * 5. Scale factor errors
 * 6. Axis misalignment
 * 7. G-sensitivity (gyro only)
 *
 * Reference Standards:
 * - IEEE Std 952-1997: Specification Format for Single-Axis IMU
 * - MIL-STD-1553: Aircraft Internal Time Division Multiplex Data Bus
 * - MIL-F-8785C: Flying Qualities of Piloted Aircraft
 */

#include "jaguar/sensors/sensor.h"

namespace jaguar::sensors {

// ============================================================================
// IMU Configuration
// ============================================================================

/**
 * @brief IMU grade/quality levels
 *
 * These presets configure noise parameters typical of various IMU grades.
 */
enum class IMUGrade : UInt8 {
    Consumer        = 0,    ///< MEMS consumer grade (smartphone)
    Industrial      = 1,    ///< Industrial MEMS grade
    Tactical        = 2,    ///< Tactical grade (medium performance)
    Navigation      = 3,    ///< Navigation grade (high performance)
    Strategic       = 4,    ///< Strategic/Marine grade (highest performance)
    Custom          = 10    ///< User-defined parameters
};

/**
 * @brief Accelerometer-specific configuration
 */
struct AccelerometerConfig {
    Vec3NoiseModel noise;       ///< Per-axis noise model

    // Range and resolution
    Real range{16.0 * 9.81};    ///< Maximum measurable acceleration (m/s²)
    Real resolution{0.001};     ///< Minimum resolution (m/s²)

    // Cross-axis sensitivity
    Real cross_axis_sensitivity{0.01}; ///< Cross-axis coupling (fraction)

    /**
     * @brief Configure from IMU grade preset
     */
    void configure_from_grade(IMUGrade grade);
};

/**
 * @brief Gyroscope-specific configuration
 */
struct GyroscopeConfig {
    Vec3NoiseModel noise;       ///< Per-axis noise model

    // Range and resolution
    Real range{2000.0 * constants::DEG_TO_RAD}; ///< Maximum rate (rad/s) [default: 2000 deg/s]
    Real resolution{0.001 * constants::DEG_TO_RAD}; ///< Minimum resolution (rad/s)

    // G-sensitivity (gyro output error due to linear acceleration)
    Vec3 g_sensitivity{0.0, 0.0, 0.0}; ///< G-sensitivity (rad/s per g)

    // Cross-axis sensitivity
    Real cross_axis_sensitivity{0.01};

    /**
     * @brief Configure from IMU grade preset
     */
    void configure_from_grade(IMUGrade grade);
};

/**
 * @brief Complete IMU configuration
 */
struct IMUConfig : public SensorConfig {
    AccelerometerConfig accelerometer;
    GyroscopeConfig gyroscope;

    IMUGrade grade{IMUGrade::Tactical};

    // Temperature effects
    bool enable_temperature_effects{false};
    Real operating_temperature{25.0};   ///< Current temperature (°C)
    Real reference_temperature{25.0};   ///< Calibration temperature (°C)
    Real temp_sensitivity{0.01};        ///< Sensitivity to temperature (fraction/°C)

    // Coning/sculling compensation
    bool enable_coning_compensation{true};
    bool enable_sculling_compensation{true};

    IMUConfig() {
        type = SensorType::IMU;
        name = "IMU";
        update_rate = 200.0;  // 200 Hz default
    }

    /**
     * @brief Configure entire IMU from grade preset
     */
    void configure_from_grade(IMUGrade new_grade);
};

// ============================================================================
// IMU Sensor Implementation
// ============================================================================

/**
 * @brief High-fidelity IMU sensor simulation
 *
 * Simulates a 6-DOF inertial measurement unit with realistic noise models.
 * The sensor provides body-frame specific force (accelerometer) and
 * angular rate (gyroscope) measurements.
 *
 * Accelerometer Output:
 * - Measures specific force (acceleration - gravity) in body frame
 * - Units: m/s² or g (configurable)
 * - Includes all noise sources and failure modes
 *
 * Gyroscope Output:
 * - Measures angular velocity in body frame
 * - Units: rad/s or deg/s (configurable)
 * - Includes bias, random walk, and g-sensitivity effects
 *
 * Usage:
 * @code
 * IMUConfig config;
 * config.configure_from_grade(IMUGrade::Tactical);
 * config.attached_entity = aircraft_id;
 *
 * IMUSensor imu(config);
 * imu.initialize();
 *
 * // In simulation loop:
 * imu.update(entity_state, dt);
 *
 * Vec3Measurement accel = imu.get_acceleration();
 * Vec3Measurement gyro = imu.get_angular_velocity();
 *
 * if (accel.valid) {
 *     // Use accel.value for navigation
 * }
 * @endcode
 *
 * @note Accelerometer measures specific force, not total acceleration.
 *       To get inertial acceleration: a_inertial = a_specific + R * g
 *       where R is rotation from NED to body and g is gravity vector.
 */
class IMUSensor : public ISensor {
public:
    /**
     * @brief Construct IMU with given configuration
     */
    explicit IMUSensor(const IMUConfig& config);

    /**
     * @brief Construct IMU with grade preset
     */
    IMUSensor(const std::string& name, IMUGrade grade, EntityId entity);

    ~IMUSensor() override = default;

    // ========================================================================
    // Measurement Access
    // ========================================================================

    /**
     * @brief Get latest accelerometer measurement (specific force)
     *
     * Returns the specific force measured in body frame, including all
     * noise and error sources. Specific force is the non-gravitational
     * acceleration experienced by the sensor.
     *
     * @return Acceleration measurement in m/s²
     */
    const Vec3Measurement& get_acceleration() const { return accel_measurement_; }

    /**
     * @brief Get latest gyroscope measurement (angular velocity)
     *
     * Returns the angular velocity measured in body frame, including all
     * noise and error sources.
     *
     * @return Angular velocity measurement in rad/s
     */
    const Vec3Measurement& get_angular_velocity() const { return gyro_measurement_; }

    /**
     * @brief Get raw (noise-free) accelerometer value
     *
     * Useful for debugging and validation.
     */
    const Vec3& get_true_acceleration() const { return true_accel_; }

    /**
     * @brief Get raw (noise-free) gyroscope value
     */
    const Vec3& get_true_angular_velocity() const { return true_gyro_; }

    // ========================================================================
    // Integrated Outputs (for navigation)
    // ========================================================================

    /**
     * @brief Get integrated velocity change (delta-V)
     *
     * Provides sculling-compensated velocity increment since last read.
     * Automatically cleared after reading.
     *
     * @return Delta-V in m/s (body frame)
     */
    Vec3 get_delta_v();

    /**
     * @brief Get integrated angle change (delta-theta)
     *
     * Provides coning-compensated angular increment since last read.
     * Automatically cleared after reading.
     *
     * @return Delta-theta in radians (body frame)
     */
    Vec3 get_delta_theta();

    // ========================================================================
    // Configuration
    // ========================================================================

    /**
     * @brief Get IMU-specific configuration
     */
    const IMUConfig& imu_config() const { return imu_config_; }

    /**
     * @brief Update accelerometer noise model
     */
    void set_accelerometer_noise(const Vec3NoiseModel& noise);

    /**
     * @brief Update gyroscope noise model
     */
    void set_gyroscope_noise(const Vec3NoiseModel& noise);

    /**
     * @brief Set operating temperature (affects noise if enabled)
     */
    void set_temperature(Real temperature);

    /**
     * @brief Get current operating temperature
     */
    Real get_temperature() const { return imu_config_.operating_temperature; }

    // ========================================================================
    // Bias Estimation (for Kalman filter integration)
    // ========================================================================

    /**
     * @brief Get current accelerometer bias estimate
     *
     * Returns the accumulated bias state for external estimation algorithms.
     */
    const Vec3& get_accel_bias() const { return accel_bias_state_; }

    /**
     * @brief Get current gyroscope bias estimate
     */
    const Vec3& get_gyro_bias() const { return gyro_bias_state_; }

    /**
     * @brief Set accelerometer bias (for bias correction)
     */
    void set_accel_bias(const Vec3& bias) { accel_bias_state_ = bias; }

    /**
     * @brief Set gyroscope bias (for bias correction)
     */
    void set_gyro_bias(const Vec3& bias) { gyro_bias_state_ = bias; }

    // ========================================================================
    // Overrides
    // ========================================================================

    bool initialize() override;
    void reset() override;

protected:
    void do_update(const physics::EntityState& entity_state) override;
    void do_apply_noise(Real dt) override;
    void do_apply_failure() override;

private:
    IMUConfig imu_config_;

    // True (noise-free) values
    Vec3 true_accel_{0.0, 0.0, 0.0};
    Vec3 true_gyro_{0.0, 0.0, 0.0};

    // Output measurements
    Vec3Measurement accel_measurement_;
    Vec3Measurement gyro_measurement_;

    // Noise state variables (for correlated noise)
    Vec3 accel_bias_state_{0.0, 0.0, 0.0};
    Vec3 gyro_bias_state_{0.0, 0.0, 0.0};
    Vec3 accel_walk_state_{0.0, 0.0, 0.0};
    Vec3 gyro_walk_state_{0.0, 0.0, 0.0};

    // Integrated outputs
    Vec3 delta_v_{0.0, 0.0, 0.0};
    Vec3 delta_theta_{0.0, 0.0, 0.0};

    // Previous values for coning/sculling
    Vec3 prev_accel_{0.0, 0.0, 0.0};
    Vec3 prev_gyro_{0.0, 0.0, 0.0};

    // Apply noise to a single axis
    Real apply_axis_noise(Real true_value, Real& bias_state, Real& walk_state,
                          const AxisNoiseModel& noise, Real dt);

    // Compute coning/sculling compensation
    void compute_integrated_outputs(Real dt);
};

// ============================================================================
// Factory Functions
// ============================================================================

/**
 * @brief Create IMU with consumer-grade noise parameters
 *
 * Typical smartphone MEMS IMU:
 * - Accel noise: 400 µg/√Hz, bias instability: 40 µg
 * - Gyro noise: 0.01 °/s/√Hz, bias instability: 10 °/hr
 */
std::unique_ptr<IMUSensor> create_consumer_imu(const std::string& name, EntityId entity);

/**
 * @brief Create IMU with tactical-grade noise parameters
 *
 * Medium-performance MEMS IMU:
 * - Accel noise: 100 µg/√Hz, bias instability: 10 µg
 * - Gyro noise: 0.003 °/s/√Hz, bias instability: 1 °/hr
 */
std::unique_ptr<IMUSensor> create_tactical_imu(const std::string& name, EntityId entity);

/**
 * @brief Create IMU with navigation-grade noise parameters
 *
 * High-performance IMU (ring laser gyro or fiber optic):
 * - Accel noise: 10 µg/√Hz, bias instability: 1 µg
 * - Gyro noise: 0.001 °/s/√Hz, bias instability: 0.01 °/hr
 */
std::unique_ptr<IMUSensor> create_navigation_imu(const std::string& name, EntityId entity);

/**
 * @brief Get default noise parameters for an IMU grade
 */
void get_imu_grade_parameters(IMUGrade grade,
                              AccelerometerConfig& accel_config,
                              GyroscopeConfig& gyro_config);

} // namespace jaguar::sensors
