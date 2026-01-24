# Sensors API Reference

Sensors module (`jaguar_sensors`) provides high-fidelity sensor simulation with realistic noise models based on MIL-SPEC standards, including Dryden and Von Karman turbulence models.

## Overview

The Sensors module enables:
- **Sensor Base**: Generic sensor interface with configurable noise
- **IMU Sensor**: MIL-SPEC IMU with grade-appropriate noise characteristics
- **Noise Models**: White noise, bias instability, random walk

---

## Sensor Framework

### Header

```cpp
#include <jaguar/sensors/sensor.h>
```

### Sensor Types

```cpp
namespace jaguar::sensors {

// Supported sensor types
enum class SensorType {
    IMU,                // Inertial Measurement Unit
    GPS,                // Global Positioning System
    Radar,              // Radio Detection and Ranging
    LIDAR,              // Light Detection and Ranging
    Camera,             // Visual camera
    Altimeter,          // Altitude sensor (barometric, radar)
    Airspeed,           // Pitot-static airspeed
    MagneticCompass,    // Magnetometer
    StarTracker,        // Celestial navigation
    Sonar,              // Underwater acoustic
    FLIR,               // Forward Looking Infrared
    RWR,                // Radar Warning Receiver
    Custom              // User-defined
};

// Sensor operational states
enum class SensorState {
    Off,                // Powered off
    Initializing,       // Warming up / calibrating
    Ready,              // Operational
    Degraded,           // Reduced accuracy
    Failed              // Non-functional
};

// Sensor failure modes
enum class FailureMode {
    None,               // Normal operation
    Bias,               // Constant offset
    Drift,              // Time-varying offset
    Noise,              // Increased noise
    Stuck,              // Frozen output
    Intermittent,       // Random failures
    Total               // Complete failure
};

}  // namespace jaguar::sensors
```

### Noise Model Types

```cpp
namespace jaguar::sensors {

// White noise parameters
struct WhiteNoiseParams {
    double power_spectral_density;  // PSD (units^2/Hz)
    double bandwidth;               // Noise bandwidth (Hz)

    // Compute standard deviation
    double std_dev() const {
        return std::sqrt(power_spectral_density * bandwidth);
    }
};

// Bias instability parameters (Allan variance)
struct BiasInstabilityParams {
    double bias_instability;        // 1-sigma value (units)
    double correlation_time;        // Correlation time (seconds)
};

// Random walk parameters
struct RandomWalkParams {
    double random_walk;             // Random walk coefficient (units/sqrt(Hz))
};

// Combined axis noise model
struct AxisNoiseModel {
    WhiteNoiseParams white_noise;
    BiasInstabilityParams bias_instability;
    RandomWalkParams random_walk;

    // Scale factor error (ppm)
    double scale_factor_error = 0.0;

    // Misalignment (radians)
    double misalignment = 0.0;

    // Nonlinearity (ppm)
    double nonlinearity = 0.0;
};

// 3-axis noise model
struct Vec3NoiseModel {
    AxisNoiseModel x;
    AxisNoiseModel y;
    AxisNoiseModel z;

    // Cross-axis sensitivity matrix
    std::array<std::array<double, 3>, 3> cross_axis = {{{1,0,0},{0,1,0},{0,0,1}}};
};

}  // namespace jaguar::sensors
```

### Sensor Configuration

```cpp
namespace jaguar::sensors {

// Generic sensor configuration
struct SensorConfig {
    std::string name;
    SensorType type;
    double sample_rate;             // Hz
    double latency;                 // seconds
    bool enabled = true;

    // Mounting offset (body frame)
    Vec3 position_offset = {0, 0, 0};
    Quaternion orientation_offset = {1, 0, 0, 0};

    // Failure configuration
    double mtbf = 10000.0;          // Mean time between failures (hours)
    FailureMode failure_mode = FailureMode::None;
    double failure_probability = 0.0;

    // Environmental limits
    double min_temperature = -40.0; // Celsius
    double max_temperature = 85.0;
    double max_vibration = 20.0;    // g
    double max_shock = 100.0;       // g
};

// Sensor measurement template
template<typename T>
struct SensorMeasurement {
    T value;
    std::chrono::nanoseconds timestamp;
    bool valid = true;
    double confidence = 1.0;        // 0.0 - 1.0
    SensorState state = SensorState::Ready;
};

}  // namespace jaguar::sensors
```

### ISensor Interface

```cpp
namespace jaguar::sensors {

class ISensor {
public:
    virtual ~ISensor() = default;

    // Lifecycle
    virtual bool initialize(const SensorConfig& config) = 0;
    virtual void shutdown() = 0;
    virtual void reset() = 0;

    // State
    virtual SensorState get_state() const = 0;
    virtual SensorType get_type() const = 0;
    virtual std::string get_name() const = 0;

    // Update
    virtual void update(double dt) = 0;

    // Configuration
    virtual void set_enabled(bool enabled) = 0;
    virtual bool is_enabled() const = 0;
    virtual double get_sample_rate() const = 0;
    virtual void set_sample_rate(double rate) = 0;

    // Failure injection
    virtual void inject_failure(FailureMode mode, double magnitude = 1.0) = 0;
    virtual void clear_failure() = 0;
    virtual FailureMode get_failure_mode() const = 0;

    // Statistics
    virtual double get_uptime() const = 0;  // seconds
    virtual size_t get_sample_count() const = 0;
};

}  // namespace jaguar::sensors
```

### SensorManager

```cpp
namespace jaguar::sensors {

class SensorManager {
public:
    // Add sensor
    void add_sensor(std::unique_ptr<ISensor> sensor);

    // Get sensor by name
    ISensor* get_sensor(const std::string& name);

    // Get sensors by type
    std::vector<ISensor*> get_sensors_by_type(SensorType type);

    // Update all sensors
    void update_all(double dt);

    // Enable/disable all sensors
    void enable_all();
    void disable_all();

    // Get all sensor states
    std::map<std::string, SensorState> get_all_states() const;

    // Bulk failure injection (for testing)
    void inject_failure_all(FailureMode mode);
    void clear_all_failures();
};

}  // namespace jaguar::sensors
```

---

## IMU Sensor

### Header

```cpp
#include <jaguar/sensors/imu_sensor.h>
```

### IMU Grades

```cpp
namespace jaguar::sensors {

// IMU grade levels (based on MIL-STD specifications)
enum class IMUGrade {
    Consumer,           // Commercial MEMS
    Industrial,         // Industrial MEMS
    Tactical,           // Tactical grade
    Navigation,         // Navigation grade
    Strategic           // Strategic/Marine grade
};

// Typical specifications by grade (1-sigma values)
//
// Grade        | Gyro Bias    | Gyro ARW     | Accel Bias  | Accel VRW
// -------------|--------------|--------------|-------------|-------------
// Consumer     | 100 deg/h    | 0.5 deg/√h   | 10 mg       | 0.1 m/s/√h
// Industrial   | 10 deg/h     | 0.1 deg/√h   | 1 mg        | 0.05 m/s/√h
// Tactical     | 1 deg/h      | 0.01 deg/√h  | 0.1 mg      | 0.01 m/s/√h
// Navigation   | 0.01 deg/h   | 0.001 deg/√h | 0.01 mg     | 0.003 m/s/√h
// Strategic    | 0.001 deg/h  | 0.0003 deg/√h| 0.001 mg    | 0.001 m/s/√h

}  // namespace jaguar::sensors
```

### IMU Configuration

```cpp
namespace jaguar::sensors {

// Accelerometer configuration
struct AccelerometerConfig {
    Vec3NoiseModel noise_model;

    // Range
    double max_acceleration = 16.0 * 9.80665;  // m/s^2 (16g)

    // Resolution
    double resolution = 0.001;      // m/s^2

    // Bandwidth
    double bandwidth = 100.0;       // Hz

    // Temperature sensitivity
    double temp_bias_coeff = 0.0;   // (m/s^2)/degC
    double temp_scale_coeff = 0.0;  // ppm/degC
};

// Gyroscope configuration
struct GyroscopeConfig {
    Vec3NoiseModel noise_model;

    // Range
    double max_rate = 2000.0 * M_PI / 180.0;  // rad/s (2000 deg/s)

    // Resolution
    double resolution = 0.0001;     // rad/s

    // Bandwidth
    double bandwidth = 100.0;       // Hz

    // Temperature sensitivity
    double temp_bias_coeff = 0.0;   // (rad/s)/degC
    double temp_scale_coeff = 0.0;  // ppm/degC

    // G-sensitivity
    double g_sensitivity = 0.0;     // (rad/s)/g
};

// Complete IMU configuration
struct IMUConfig : SensorConfig {
    AccelerometerConfig accelerometer;
    GyroscopeConfig gyroscope;
    IMUGrade grade = IMUGrade::Tactical;

    // Alignment errors (radians)
    Vec3 gyro_misalignment = {0, 0, 0};
    Vec3 accel_misalignment = {0, 0, 0};

    // Internal temperature
    double operating_temperature = 25.0;  // Celsius

    // Vibration rectification coefficient
    double vibration_rectification = 0.0;

    // Factory to create grade-appropriate config
    static IMUConfig from_grade(IMUGrade grade);
};

}  // namespace jaguar::sensors
```

### IMU Measurement

```cpp
namespace jaguar::sensors {

// IMU measurement output
struct IMUMeasurement {
    Vec3 specific_force;            // m/s^2 (body frame)
    Vec3 angular_velocity;          // rad/s (body frame)
    std::chrono::nanoseconds timestamp;
    double temperature;             // Celsius
    bool valid = true;
    SensorState state = SensorState::Ready;
};

}  // namespace jaguar::sensors
```

### IMUSensor Class

```cpp
namespace jaguar::sensors {

class IMUSensor : public ISensor {
public:
    // Initialize with configuration
    bool initialize(const IMUConfig& config);

    // Set true values (from simulation)
    void set_true_specific_force(const Vec3& force);
    void set_true_angular_velocity(const Vec3& rate);
    void set_true_state(const Vec3& force, const Vec3& rate);

    // Get measurement (with noise applied)
    IMUMeasurement get_measurement();

    // Get raw (noiseless) measurement
    IMUMeasurement get_true_measurement() const;

    // Get individual components
    Vec3 get_accelerometer_output() const;
    Vec3 get_gyroscope_output() const;

    // Temperature effects
    void set_temperature(double celsius);
    double get_temperature() const;

    // Calibration
    void calibrate(double duration_seconds);
    bool is_calibrating() const;
    Vec3 get_accelerometer_bias() const;
    Vec3 get_gyroscope_bias() const;

    // Allan variance analysis (for validation)
    struct AllanVarianceResult {
        std::vector<double> tau;            // Averaging times
        std::vector<double> adev_gyro_x;    // Allan deviation
        std::vector<double> adev_gyro_y;
        std::vector<double> adev_gyro_z;
        std::vector<double> adev_accel_x;
        std::vector<double> adev_accel_y;
        std::vector<double> adev_accel_z;
    };
    AllanVarianceResult compute_allan_variance(double duration_seconds,
                                               double sample_rate);

    // From ISensor
    bool initialize(const SensorConfig& config) override;
    void shutdown() override;
    void reset() override;
    SensorState get_state() const override;
    SensorType get_type() const override { return SensorType::IMU; }
    std::string get_name() const override;
    void update(double dt) override;
    void set_enabled(bool enabled) override;
    bool is_enabled() const override;
    double get_sample_rate() const override;
    void set_sample_rate(double rate) override;
    void inject_failure(FailureMode mode, double magnitude) override;
    void clear_failure() override;
    FailureMode get_failure_mode() const override;
    double get_uptime() const override;
    size_t get_sample_count() const override;

private:
    // Internal noise generation
    Vec3 apply_accelerometer_noise(const Vec3& true_value);
    Vec3 apply_gyroscope_noise(const Vec3& true_value);
    double generate_white_noise(double std_dev);
    double generate_bias_instability(double& state, const BiasInstabilityParams& params, double dt);
    double generate_random_walk(double& state, const RandomWalkParams& params, double dt);
};

// Factory functions
std::unique_ptr<IMUSensor> create_imu_sensor(const IMUConfig& config);
std::unique_ptr<IMUSensor> create_consumer_imu();
std::unique_ptr<IMUSensor> create_industrial_imu();
std::unique_ptr<IMUSensor> create_tactical_imu();
std::unique_ptr<IMUSensor> create_navigation_imu();
std::unique_ptr<IMUSensor> create_strategic_imu();

}  // namespace jaguar::sensors
```

---

## Usage Examples

### Basic IMU Simulation

```cpp
#include <jaguar/sensors/imu_sensor.h>

using namespace jaguar::sensors;

// Create tactical-grade IMU
auto imu = create_tactical_imu();

// Or configure manually
IMUConfig config = IMUConfig::from_grade(IMUGrade::Tactical);
config.name = "Main_IMU";
config.sample_rate = 100.0;  // 100 Hz

// Customize accelerometer noise
config.accelerometer.noise_model.x.white_noise.power_spectral_density = 1e-6;
config.accelerometer.noise_model.x.bias_instability.bias_instability = 0.001 * 9.80665;

auto custom_imu = create_imu_sensor(config);
custom_imu->initialize(config);

// Simulation loop
while (running) {
    // Get true values from simulation
    auto state = engine.get_entity_state(aircraft);

    // Compute specific force (acceleration - gravity in body frame)
    Vec3 gravity_body = rotate_to_body(Vec3{0, 0, 9.80665}, state.orientation);
    Vec3 specific_force = state.acceleration - gravity_body;

    // Set true values
    imu->set_true_state(specific_force, state.angular_velocity);

    // Update sensor (applies noise)
    imu->update(dt);

    // Get noisy measurement
    auto measurement = imu->get_measurement();

    if (measurement.valid) {
        // Use in navigation filter
        navigation_filter.process_imu(
            measurement.specific_force,
            measurement.angular_velocity,
            measurement.timestamp
        );
    }

    engine.step(dt);
}
```

### Multi-Sensor Fusion

```cpp
#include <jaguar/sensors/imu_sensor.h>
#include <jaguar/sensors/sensor.h>

using namespace jaguar::sensors;

// Create sensor manager
SensorManager sensor_mgr;

// Add multiple IMUs (redundancy)
auto imu1 = create_navigation_imu();
imu1->initialize(make_config("IMU_1", Vec3{0.5, 0, 0}));

auto imu2 = create_navigation_imu();
imu2->initialize(make_config("IMU_2", Vec3{-0.5, 0, 0}));

auto imu3 = create_navigation_imu();
imu3->initialize(make_config("IMU_3", Vec3{0, 0.5, 0}));

sensor_mgr.add_sensor(std::move(imu1));
sensor_mgr.add_sensor(std::move(imu2));
sensor_mgr.add_sensor(std::move(imu3));

// Simulation loop
while (running) {
    // Update all sensors
    sensor_mgr.update_all(dt);

    // Get all IMU sensors
    auto imus = sensor_mgr.get_sensors_by_type(SensorType::IMU);

    std::vector<IMUMeasurement> measurements;
    for (auto* sensor : imus) {
        auto* imu = static_cast<IMUSensor*>(sensor);
        if (imu->get_state() == SensorState::Ready) {
            measurements.push_back(imu->get_measurement());
        }
    }

    // Sensor fusion
    if (measurements.size() >= 2) {
        auto fused = fuse_imu_measurements(measurements);
        navigation_filter.process_fused_imu(fused);
    }

    engine.step(dt);
}
```

### Failure Injection Testing

```cpp
#include <jaguar/sensors/imu_sensor.h>

using namespace jaguar::sensors;

// Create IMU for testing
auto imu = create_tactical_imu();

// Normal operation
for (int i = 0; i < 1000; ++i) {
    imu->update(dt);
    auto m = imu->get_measurement();
    log_measurement(m);
}

// Inject bias failure
std::cout << "Injecting bias failure...\n";
imu->inject_failure(FailureMode::Bias, 0.01);  // 0.01 rad/s bias

for (int i = 0; i < 1000; ++i) {
    imu->update(dt);
    auto m = imu->get_measurement();
    log_measurement(m);

    // Navigation filter should detect degraded performance
}

// Inject stuck failure
std::cout << "Injecting stuck failure...\n";
imu->inject_failure(FailureMode::Stuck);

for (int i = 0; i < 100; ++i) {
    imu->update(dt);
    auto m = imu->get_measurement();

    // Output will be frozen at last valid value
    assert(m.angular_velocity == last_valid_rate);
}

// Clear failures
imu->clear_failure();

// Verify recovery
for (int i = 0; i < 100; ++i) {
    imu->update(dt);
    auto m = imu->get_measurement();
    assert(m.valid);
}
```

### Allan Variance Analysis

```cpp
#include <jaguar/sensors/imu_sensor.h>
#include <fstream>

using namespace jaguar::sensors;

// Create IMU
IMUConfig config = IMUConfig::from_grade(IMUGrade::Navigation);
auto imu = create_imu_sensor(config);
imu->initialize(config);

// Set constant input (for Allan variance test)
imu->set_true_state(Vec3{0, 0, 9.80665}, Vec3{0, 0, 0});

// Collect data and compute Allan variance
double test_duration = 3600.0;  // 1 hour
double sample_rate = 100.0;     // 100 Hz

std::cout << "Running Allan variance test for " << test_duration << " seconds...\n";

auto result = imu->compute_allan_variance(test_duration, sample_rate);

// Output results
std::ofstream csv("allan_variance.csv");
csv << "tau,adev_gyro_x,adev_gyro_y,adev_gyro_z,"
    << "adev_accel_x,adev_accel_y,adev_accel_z\n";

for (size_t i = 0; i < result.tau.size(); ++i) {
    csv << result.tau[i] << ","
        << result.adev_gyro_x[i] << ","
        << result.adev_gyro_y[i] << ","
        << result.adev_gyro_z[i] << ","
        << result.adev_accel_x[i] << ","
        << result.adev_accel_y[i] << ","
        << result.adev_accel_z[i] << "\n";
}

// Verify noise parameters match specification
double expected_arw = 0.001 * M_PI / 180.0;  // 0.001 deg/sqrt(hr)
double measured_arw = result.adev_gyro_x[0] * std::sqrt(result.tau[0]);

std::cout << "Expected ARW: " << expected_arw << " rad/sqrt(s)\n";
std::cout << "Measured ARW: " << measured_arw << " rad/sqrt(s)\n";
std::cout << "Error: " << std::abs(measured_arw - expected_arw) / expected_arw * 100 << "%\n";
```

### INS/GPS Integration

```cpp
#include <jaguar/sensors/imu_sensor.h>

using namespace jaguar::sensors;

class INSGPSNavigator {
    std::unique_ptr<IMUSensor> imu_;

    // Navigation state
    Vec3 position_;
    Vec3 velocity_;
    Quaternion attitude_;

    // Kalman filter state
    Eigen::VectorXd x_;  // 15-state INS error model
    Eigen::MatrixXd P_;  // Covariance

public:
    void initialize() {
        // Create navigation-grade IMU
        imu_ = create_navigation_imu();

        // Initialize Kalman filter
        x_ = Eigen::VectorXd::Zero(15);  // pos, vel, att, gyro_bias, accel_bias
        P_ = Eigen::MatrixXd::Identity(15, 15);

        // Set initial uncertainties based on IMU specifications
        auto config = IMUConfig::from_grade(IMUGrade::Navigation);
        double gyro_bias_std = config.gyroscope.noise_model.x.bias_instability.bias_instability;
        double accel_bias_std = config.accelerometer.noise_model.x.bias_instability.bias_instability;

        P_.block<3,3>(9, 9) = Eigen::Matrix3d::Identity() * gyro_bias_std * gyro_bias_std;
        P_.block<3,3>(12, 12) = Eigen::Matrix3d::Identity() * accel_bias_std * accel_bias_std;
    }

    void process_imu(double dt) {
        auto meas = imu_->get_measurement();

        // Remove estimated biases
        Vec3 corrected_gyro = meas.angular_velocity - get_gyro_bias_estimate();
        Vec3 corrected_accel = meas.specific_force - get_accel_bias_estimate();

        // Mechanization: integrate navigation equations
        attitude_ = integrate_attitude(attitude_, corrected_gyro, dt);
        Vec3 accel_nav = rotate_to_nav(corrected_accel, attitude_);
        velocity_ = velocity_ + (accel_nav + gravity_) * dt;
        position_ = position_ + velocity_ * dt;

        // Propagate covariance
        propagate_covariance(dt, corrected_gyro, corrected_accel);
    }

    void process_gps(const Vec3& gps_position, const Vec3& gps_velocity) {
        // Kalman update with GPS measurement
        Eigen::VectorXd z(6);
        z << (position_.x - gps_position.x),
             (position_.y - gps_position.y),
             (position_.z - gps_position.z),
             (velocity_.x - gps_velocity.x),
             (velocity_.y - gps_velocity.y),
             (velocity_.z - gps_velocity.z);

        // Update state and covariance
        kalman_update(z);

        // Apply corrections
        apply_corrections();
    }

    Vec3 get_position() const { return position_; }
    Vec3 get_velocity() const { return velocity_; }
    Quaternion get_attitude() const { return attitude_; }

private:
    Vec3 get_gyro_bias_estimate() const {
        return Vec3{x_(9), x_(10), x_(11)};
    }

    Vec3 get_accel_bias_estimate() const {
        return Vec3{x_(12), x_(13), x_(14)};
    }

    // ... Kalman filter implementation ...
};
```

---

## See Also

- [Architecture](../advanced/architecture.md) - System architecture overview
- [Air Domain](air.md) - Aircraft-specific sensors
- [Configuration](configuration.md) - Engine configuration
