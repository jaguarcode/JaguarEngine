/**
 * @file imu_sensor.cpp
 * @brief IMU sensor implementation with MIL-SPEC noise models
 */

#include "jaguar/sensors/imu_sensor.h"
#include "jaguar/physics/entity.h"
#include <cmath>
#include <algorithm>

namespace jaguar::sensors {

// ============================================================================
// Constants
// ============================================================================

// Conversion factors
constexpr Real MICRO_G_TO_M_S2 = 9.81e-6;      // µg to m/s²
constexpr Real DEG_PER_HOUR_TO_RAD_S = 4.848e-6; // °/hr to rad/s
constexpr Real DEG_PER_S_TO_RAD_S = constants::DEG_TO_RAD;
constexpr Real DEG_PER_ROOT_HOUR_TO_RAD_ROOT_S = constants::DEG_TO_RAD / 60.0;

// ============================================================================
// IMU Grade Parameters
// ============================================================================

/**
 * IMU Grade specifications (typical values):
 *
 * Consumer (MEMS smartphone):
 *   Accel: 400 µg/√Hz ARW, 40 µg bias instability
 *   Gyro:  0.01 °/s/√Hz ARW, 10 °/hr bias instability
 *
 * Industrial (industrial MEMS):
 *   Accel: 100 µg/√Hz ARW, 20 µg bias instability
 *   Gyro:  0.005 °/s/√Hz ARW, 3 °/hr bias instability
 *
 * Tactical (high-end MEMS):
 *   Accel: 50 µg/√Hz ARW, 10 µg bias instability
 *   Gyro:  0.003 °/s/√Hz ARW, 1 °/hr bias instability
 *
 * Navigation (RLG/FOG):
 *   Accel: 10 µg/√Hz ARW, 1 µg bias instability
 *   Gyro:  0.001 °/s/√Hz ARW, 0.01 °/hr bias instability
 *
 * Strategic (Marine/Strategic):
 *   Accel: 1 µg/√Hz ARW, 0.1 µg bias instability
 *   Gyro:  0.0001 °/s/√Hz ARW, 0.001 °/hr bias instability
 */

void get_imu_grade_parameters(IMUGrade grade,
                              AccelerometerConfig& accel_config,
                              GyroscopeConfig& gyro_config) {
    // Set default noise parameters based on grade
    AxisNoiseModel accel_noise;
    AxisNoiseModel gyro_noise;

    switch (grade) {
        case IMUGrade::Consumer:
            // Accelerometer: 400 µg/√Hz, 40 µg bias, 100 µg/√hr random walk
            accel_noise.white_noise.sigma = 400.0 * MICRO_G_TO_M_S2;
            accel_noise.bias_instability.sigma = 40.0 * MICRO_G_TO_M_S2;
            accel_noise.bias_instability.correlation_time = 100.0;
            accel_noise.random_walk.sigma = 100.0 * MICRO_G_TO_M_S2 / 60.0;
            accel_noise.scale_factor_error = 0.005;  // 0.5%
            accel_config.range = 16.0 * 9.81;  // ±16g

            // Gyroscope: 0.01 °/s/√Hz, 10 °/hr bias, 0.5 °/√hr random walk
            gyro_noise.white_noise.sigma = 0.01 * DEG_PER_S_TO_RAD_S;
            gyro_noise.bias_instability.sigma = 10.0 * DEG_PER_HOUR_TO_RAD_S;
            gyro_noise.bias_instability.correlation_time = 100.0;
            gyro_noise.random_walk.sigma = 0.5 * DEG_PER_ROOT_HOUR_TO_RAD_ROOT_S;
            gyro_noise.scale_factor_error = 0.01;  // 1%
            gyro_config.range = 2000.0 * constants::DEG_TO_RAD;  // ±2000°/s
            break;

        case IMUGrade::Industrial:
            accel_noise.white_noise.sigma = 100.0 * MICRO_G_TO_M_S2;
            accel_noise.bias_instability.sigma = 20.0 * MICRO_G_TO_M_S2;
            accel_noise.bias_instability.correlation_time = 200.0;
            accel_noise.random_walk.sigma = 50.0 * MICRO_G_TO_M_S2 / 60.0;
            accel_noise.scale_factor_error = 0.002;
            accel_config.range = 8.0 * 9.81;

            gyro_noise.white_noise.sigma = 0.005 * DEG_PER_S_TO_RAD_S;
            gyro_noise.bias_instability.sigma = 3.0 * DEG_PER_HOUR_TO_RAD_S;
            gyro_noise.bias_instability.correlation_time = 200.0;
            gyro_noise.random_walk.sigma = 0.2 * DEG_PER_ROOT_HOUR_TO_RAD_ROOT_S;
            gyro_noise.scale_factor_error = 0.005;
            gyro_config.range = 500.0 * constants::DEG_TO_RAD;
            break;

        case IMUGrade::Tactical:
            accel_noise.white_noise.sigma = 50.0 * MICRO_G_TO_M_S2;
            accel_noise.bias_instability.sigma = 10.0 * MICRO_G_TO_M_S2;
            accel_noise.bias_instability.correlation_time = 300.0;
            accel_noise.random_walk.sigma = 20.0 * MICRO_G_TO_M_S2 / 60.0;
            accel_noise.scale_factor_error = 0.001;
            accel_config.range = 10.0 * 9.81;

            gyro_noise.white_noise.sigma = 0.003 * DEG_PER_S_TO_RAD_S;
            gyro_noise.bias_instability.sigma = 1.0 * DEG_PER_HOUR_TO_RAD_S;
            gyro_noise.bias_instability.correlation_time = 300.0;
            gyro_noise.random_walk.sigma = 0.1 * DEG_PER_ROOT_HOUR_TO_RAD_ROOT_S;
            gyro_noise.scale_factor_error = 0.002;
            gyro_config.range = 400.0 * constants::DEG_TO_RAD;
            break;

        case IMUGrade::Navigation:
            accel_noise.white_noise.sigma = 10.0 * MICRO_G_TO_M_S2;
            accel_noise.bias_instability.sigma = 1.0 * MICRO_G_TO_M_S2;
            accel_noise.bias_instability.correlation_time = 600.0;
            accel_noise.random_walk.sigma = 5.0 * MICRO_G_TO_M_S2 / 60.0;
            accel_noise.scale_factor_error = 0.0003;
            accel_config.range = 6.0 * 9.81;

            gyro_noise.white_noise.sigma = 0.001 * DEG_PER_S_TO_RAD_S;
            gyro_noise.bias_instability.sigma = 0.01 * DEG_PER_HOUR_TO_RAD_S;
            gyro_noise.bias_instability.correlation_time = 600.0;
            gyro_noise.random_walk.sigma = 0.01 * DEG_PER_ROOT_HOUR_TO_RAD_ROOT_S;
            gyro_noise.scale_factor_error = 0.0005;
            gyro_config.range = 300.0 * constants::DEG_TO_RAD;
            break;

        case IMUGrade::Strategic:
            accel_noise.white_noise.sigma = 1.0 * MICRO_G_TO_M_S2;
            accel_noise.bias_instability.sigma = 0.1 * MICRO_G_TO_M_S2;
            accel_noise.bias_instability.correlation_time = 1000.0;
            accel_noise.random_walk.sigma = 0.5 * MICRO_G_TO_M_S2 / 60.0;
            accel_noise.scale_factor_error = 0.0001;
            accel_config.range = 4.0 * 9.81;

            gyro_noise.white_noise.sigma = 0.0001 * DEG_PER_S_TO_RAD_S;
            gyro_noise.bias_instability.sigma = 0.001 * DEG_PER_HOUR_TO_RAD_S;
            gyro_noise.bias_instability.correlation_time = 1000.0;
            gyro_noise.random_walk.sigma = 0.001 * DEG_PER_ROOT_HOUR_TO_RAD_ROOT_S;
            gyro_noise.scale_factor_error = 0.0001;
            gyro_config.range = 200.0 * constants::DEG_TO_RAD;
            break;

        default:  // Custom - use defaults
            accel_noise.white_noise.sigma = 50.0 * MICRO_G_TO_M_S2;
            accel_noise.bias_instability.sigma = 10.0 * MICRO_G_TO_M_S2;
            gyro_noise.white_noise.sigma = 0.003 * DEG_PER_S_TO_RAD_S;
            gyro_noise.bias_instability.sigma = 1.0 * DEG_PER_HOUR_TO_RAD_S;
            break;
    }

    accel_config.noise.set_uniform(accel_noise);
    gyro_config.noise.set_uniform(gyro_noise);
}

void AccelerometerConfig::configure_from_grade(IMUGrade grade) {
    GyroscopeConfig dummy;
    get_imu_grade_parameters(grade, *this, dummy);
}

void GyroscopeConfig::configure_from_grade(IMUGrade grade) {
    AccelerometerConfig dummy;
    get_imu_grade_parameters(grade, dummy, *this);
}

void IMUConfig::configure_from_grade(IMUGrade new_grade) {
    grade = new_grade;
    get_imu_grade_parameters(grade, accelerometer, gyroscope);
}

// ============================================================================
// IMUSensor Implementation
// ============================================================================

IMUSensor::IMUSensor(const IMUConfig& config)
    : ISensor(config)
    , imu_config_(config) {
}

IMUSensor::IMUSensor(const std::string& name, IMUGrade grade, EntityId entity)
    : ISensor(SensorConfig{}) {
    imu_config_.name = name;
    imu_config_.type = SensorType::IMU;
    imu_config_.attached_entity = entity;
    imu_config_.configure_from_grade(grade);
    config_ = imu_config_;
}

bool IMUSensor::initialize() {
    if (!ISensor::initialize()) {
        return false;
    }

    // Reset noise states
    accel_bias_state_ = Vec3{0.0, 0.0, 0.0};
    gyro_bias_state_ = Vec3{0.0, 0.0, 0.0};
    accel_walk_state_ = Vec3{0.0, 0.0, 0.0};
    gyro_walk_state_ = Vec3{0.0, 0.0, 0.0};

    // Clear integrated outputs
    delta_v_ = Vec3{0.0, 0.0, 0.0};
    delta_theta_ = Vec3{0.0, 0.0, 0.0};

    // Initialize previous values
    prev_accel_ = Vec3{0.0, 0.0, 0.0};
    prev_gyro_ = Vec3{0.0, 0.0, 0.0};

    return true;
}

void IMUSensor::reset() {
    ISensor::reset();

    accel_bias_state_ = Vec3{0.0, 0.0, 0.0};
    gyro_bias_state_ = Vec3{0.0, 0.0, 0.0};
    accel_walk_state_ = Vec3{0.0, 0.0, 0.0};
    gyro_walk_state_ = Vec3{0.0, 0.0, 0.0};

    delta_v_ = Vec3{0.0, 0.0, 0.0};
    delta_theta_ = Vec3{0.0, 0.0, 0.0};

    prev_accel_ = Vec3{0.0, 0.0, 0.0};
    prev_gyro_ = Vec3{0.0, 0.0, 0.0};

    accel_measurement_ = Vec3Measurement{};
    gyro_measurement_ = Vec3Measurement{};
}

void IMUSensor::do_update(const physics::EntityState& entity_state) {
    // Extract true values from entity state
    // Accelerometer measures specific force = acceleration - gravity (in body frame)
    // We need to rotate gravity from ECEF/NED to body frame

    // Get the body-frame acceleration (which already includes specific force)
    true_accel_ = entity_state.acceleration;

    // Angular velocity is already in body frame
    true_gyro_ = entity_state.angular_velocity;

    // Apply mounting offset transformation if needed
    if (config_.mount_position.x != 0.0 ||
        config_.mount_position.y != 0.0 ||
        config_.mount_position.z != 0.0) {
        // Lever arm effect: a_sensor = a_cg + ω × (ω × r) + α × r
        Vec3 r = config_.mount_position;
        Vec3 omega = entity_state.angular_velocity;
        Vec3 alpha = entity_state.angular_accel;

        // Centripetal acceleration: ω × (ω × r)
        Vec3 centripetal = omega.cross(omega.cross(r));

        // Angular acceleration contribution: α × r
        Vec3 angular_contrib = alpha.cross(r);

        true_accel_ = true_accel_ + centripetal + angular_contrib;
    }

    // Store timestamp
    accel_measurement_.timestamp = simulation_time_;
    accel_measurement_.measurement_time = simulation_time_ - config_.latency;
    accel_measurement_.sequence_number = sequence_number_;

    gyro_measurement_.timestamp = simulation_time_;
    gyro_measurement_.measurement_time = simulation_time_ - config_.latency;
    gyro_measurement_.sequence_number = sequence_number_;
}

void IMUSensor::do_apply_noise(Real dt) {
    // Apply noise to accelerometer
    Vec3 noisy_accel;
    noisy_accel.x = apply_axis_noise(true_accel_.x, accel_bias_state_.x,
                                        accel_walk_state_.x,
                                        imu_config_.accelerometer.noise.x, dt);
    noisy_accel.y = apply_axis_noise(true_accel_.y, accel_bias_state_.y,
                                        accel_walk_state_.y,
                                        imu_config_.accelerometer.noise.y, dt);
    noisy_accel.z = apply_axis_noise(true_accel_.z, accel_bias_state_.z,
                                        accel_walk_state_.z,
                                        imu_config_.accelerometer.noise.z, dt);

    // Clamp to range
    noisy_accel.x = std::clamp(noisy_accel.x,
                                  -imu_config_.accelerometer.range,
                                  imu_config_.accelerometer.range);
    noisy_accel.y = std::clamp(noisy_accel.y,
                                  -imu_config_.accelerometer.range,
                                  imu_config_.accelerometer.range);
    noisy_accel.z = std::clamp(noisy_accel.z,
                                  -imu_config_.accelerometer.range,
                                  imu_config_.accelerometer.range);

    // Apply noise to gyroscope
    Vec3 noisy_gyro;
    noisy_gyro.x = apply_axis_noise(true_gyro_.x, gyro_bias_state_.x,
                                       gyro_walk_state_.x,
                                       imu_config_.gyroscope.noise.x, dt);
    noisy_gyro.y = apply_axis_noise(true_gyro_.y, gyro_bias_state_.y,
                                       gyro_walk_state_.y,
                                       imu_config_.gyroscope.noise.y, dt);
    noisy_gyro.z = apply_axis_noise(true_gyro_.z, gyro_bias_state_.z,
                                       gyro_walk_state_.z,
                                       imu_config_.gyroscope.noise.z, dt);

    // Apply g-sensitivity to gyroscope (gyro error due to linear acceleration)
    if (imu_config_.gyroscope.g_sensitivity.x != 0.0 ||
        imu_config_.gyroscope.g_sensitivity.y != 0.0 ||
        imu_config_.gyroscope.g_sensitivity.z != 0.0) {
        Real g = 9.81;
        noisy_gyro.x += imu_config_.gyroscope.g_sensitivity.x * noisy_accel.x / g;
        noisy_gyro.y += imu_config_.gyroscope.g_sensitivity.y * noisy_accel.y / g;
        noisy_gyro.z += imu_config_.gyroscope.g_sensitivity.z * noisy_accel.z / g;
    }

    // Clamp to range
    noisy_gyro.x = std::clamp(noisy_gyro.x,
                                 -imu_config_.gyroscope.range,
                                 imu_config_.gyroscope.range);
    noisy_gyro.y = std::clamp(noisy_gyro.y,
                                 -imu_config_.gyroscope.range,
                                 imu_config_.gyroscope.range);
    noisy_gyro.z = std::clamp(noisy_gyro.z,
                                 -imu_config_.gyroscope.range,
                                 imu_config_.gyroscope.range);

    // Apply temperature effects if enabled
    if (imu_config_.enable_temperature_effects) {
        Real temp_delta = imu_config_.operating_temperature -
                          imu_config_.reference_temperature;
        Real temp_factor = 1.0 + imu_config_.temp_sensitivity * temp_delta;

        noisy_accel = noisy_accel * temp_factor;
        noisy_gyro = noisy_gyro * temp_factor;
    }

    // Store measurements
    accel_measurement_.value = noisy_accel;
    accel_measurement_.valid = true;
    accel_measurement_.confidence = (state_ == SensorState::Operational) ? 1.0 : 0.5;

    gyro_measurement_.value = noisy_gyro;
    gyro_measurement_.valid = true;
    gyro_measurement_.confidence = (state_ == SensorState::Operational) ? 1.0 : 0.5;

    // Compute integrated outputs (delta-v, delta-theta)
    compute_integrated_outputs(dt);

    // Store for next iteration
    prev_accel_ = noisy_accel;
    prev_gyro_ = noisy_gyro;
}

void IMUSensor::do_apply_failure() {
    switch (failure_mode_) {
        case FailureMode::Bias:
            // Add constant bias error
            accel_measurement_.value = accel_measurement_.value +
                Vec3{failure_magnitude_, failure_magnitude_, failure_magnitude_};
            gyro_measurement_.value = gyro_measurement_.value +
                Vec3{failure_magnitude_ * 0.01, failure_magnitude_ * 0.01, failure_magnitude_ * 0.01};
            break;

        case FailureMode::Drift:
            // Increasing error over time (already handled by bias state drift)
            break;

        case FailureMode::Noise:
            // Increased noise level
            {
                Real extra_sigma = failure_magnitude_ * 0.01;  // Scale by magnitude
                accel_measurement_.value = accel_measurement_.value +
                    Vec3{generate_white_noise(extra_sigma),
                         generate_white_noise(extra_sigma),
                         generate_white_noise(extra_sigma)};
                gyro_measurement_.value = gyro_measurement_.value +
                    Vec3{generate_white_noise(extra_sigma * 0.01),
                         generate_white_noise(extra_sigma * 0.01),
                         generate_white_noise(extra_sigma * 0.01)};
            }
            break;

        case FailureMode::Saturation:
            // Already handled by range clamping
            break;

        case FailureMode::Dropout:
            // Randomly invalidate measurement
            if (normal_dist_(rng_) > failure_magnitude_) {
                accel_measurement_.valid = false;
                gyro_measurement_.valid = false;
            }
            break;

        case FailureMode::Stuck:
            // Keep previous value (don't update)
            accel_measurement_.value = prev_accel_;
            gyro_measurement_.value = prev_gyro_;
            break;

        case FailureMode::Oscillation:
            // Add sinusoidal oscillation
            {
                Real t = simulation_time_ * 10.0;  // 10 Hz oscillation
                Real osc = std::sin(t) * failure_magnitude_;
                accel_measurement_.value = accel_measurement_.value +
                    Vec3{osc, osc, osc};
            }
            break;

        case FailureMode::Complete:
            // Total failure - invalidate
            accel_measurement_.valid = false;
            gyro_measurement_.valid = false;
            accel_measurement_.confidence = 0.0;
            gyro_measurement_.confidence = 0.0;
            break;

        default:
            break;
    }
}

Real IMUSensor::apply_axis_noise(Real true_value, Real& bias_state, Real& walk_state,
                                  const AxisNoiseModel& noise, Real dt) {
    Real output = true_value;

    // Apply scale factor error
    output *= (1.0 + noise.scale_factor_error);

    // Add constant bias
    output += noise.constant_bias;

    // Add bias instability (Gauss-Markov)
    output += apply_bias_instability(bias_state, noise.bias_instability, dt);

    // Add random walk
    output += apply_random_walk(walk_state, noise.random_walk, dt);

    // Add white noise (scaled by sqrt of update rate)
    if (noise.white_noise.enabled && noise.white_noise.sigma > 0.0) {
        // White noise power scales with bandwidth
        // For discrete time: σ_discrete = σ_continuous * sqrt(fs)
        Real fs = config_.update_rate > 0 ? config_.update_rate : 1.0 / dt;
        Real sigma_discrete = noise.white_noise.sigma * std::sqrt(fs);
        output += generate_white_noise(sigma_discrete);
    }

    // Apply quantization
    if (noise.quantization.enabled && noise.quantization.resolution > 0.0) {
        output = apply_quantization(output, noise.quantization.resolution);
    }

    return output;
}

void IMUSensor::compute_integrated_outputs(Real dt) {
    if (!imu_config_.enable_sculling_compensation && !imu_config_.enable_coning_compensation) {
        // Simple integration without compensation
        delta_v_ = delta_v_ + accel_measurement_.value * dt;
        delta_theta_ = delta_theta_ + gyro_measurement_.value * dt;
        return;
    }

    // Sculling compensation for delta-v
    // ΔV_sculling = 0.5 * Δθ × (a_prev + a_curr) * dt / 2
    if (imu_config_.enable_sculling_compensation) {
        Vec3 avg_accel = (prev_accel_ + accel_measurement_.value) * 0.5;
        Vec3 sculling_correction = gyro_measurement_.value.cross(avg_accel) * dt * dt * 0.5;
        delta_v_ = delta_v_ + accel_measurement_.value * dt + sculling_correction;
    } else {
        delta_v_ = delta_v_ + accel_measurement_.value * dt;
    }

    // Coning compensation for delta-theta
    // Δθ_coning = (1/12) * Δθ_prev × Δθ_curr
    if (imu_config_.enable_coning_compensation) {
        Vec3 delta_theta_curr = gyro_measurement_.value * dt;
        Vec3 delta_theta_prev = prev_gyro_ * dt;
        Vec3 coning_correction = delta_theta_prev.cross(delta_theta_curr) * (1.0 / 12.0);
        delta_theta_ = delta_theta_ + delta_theta_curr + coning_correction;
    } else {
        delta_theta_ = delta_theta_ + gyro_measurement_.value * dt;
    }
}

Vec3 IMUSensor::get_delta_v() {
    Vec3 result = delta_v_;
    delta_v_ = Vec3{0.0, 0.0, 0.0};
    return result;
}

Vec3 IMUSensor::get_delta_theta() {
    Vec3 result = delta_theta_;
    delta_theta_ = Vec3{0.0, 0.0, 0.0};
    return result;
}

void IMUSensor::set_accelerometer_noise(const Vec3NoiseModel& noise) {
    imu_config_.accelerometer.noise = noise;
}

void IMUSensor::set_gyroscope_noise(const Vec3NoiseModel& noise) {
    imu_config_.gyroscope.noise = noise;
}

void IMUSensor::set_temperature(Real temperature) {
    imu_config_.operating_temperature = temperature;
}

// ============================================================================
// Factory Functions
// ============================================================================

std::unique_ptr<IMUSensor> create_consumer_imu(const std::string& name, EntityId entity) {
    IMUConfig config;
    config.name = name;
    config.attached_entity = entity;
    config.configure_from_grade(IMUGrade::Consumer);
    config.update_rate = 100.0;  // 100 Hz typical for consumer
    return std::make_unique<IMUSensor>(config);
}

std::unique_ptr<IMUSensor> create_tactical_imu(const std::string& name, EntityId entity) {
    IMUConfig config;
    config.name = name;
    config.attached_entity = entity;
    config.configure_from_grade(IMUGrade::Tactical);
    config.update_rate = 200.0;  // 200 Hz for tactical
    return std::make_unique<IMUSensor>(config);
}

std::unique_ptr<IMUSensor> create_navigation_imu(const std::string& name, EntityId entity) {
    IMUConfig config;
    config.name = name;
    config.attached_entity = entity;
    config.configure_from_grade(IMUGrade::Navigation);
    config.update_rate = 400.0;  // 400 Hz for navigation grade
    return std::make_unique<IMUSensor>(config);
}

} // namespace jaguar::sensors
