# Sensors Module Documentation

The Sensors module provides high-fidelity simulation of inertial measurement units (IMUs) and other sensor systems. Includes realistic noise models following IEEE/AIAA standards, bias instability, random walk, temperature effects, and failure modes for comprehensive sensor simulation.

## Headers

| Header | Purpose |
|--------|---------|
| `jaguar/sensors/sensor.h` | Base sensor interface and noise models |
| `jaguar/sensors/imu_sensor.h` | IMU simulation (accelerometer + gyroscope) |

## Sensor Base Class (`sensor.h`)

### ISensor Interface

```cpp
class ISensor {
public:
    virtual bool initialize() = 0;
    virtual void reset() = 0;
    virtual void update(const physics::EntityState& entity_state, Real dt) = 0;
    virtual SensorType type() const = 0;
    virtual const std::string& name() const = 0;
    virtual bool is_valid() const = 0;
};
```

### Noise Model Components

The sensor noise model follows IEEE Std 952-1997 for inertial sensor specification:

```cpp
struct AxisNoiseModel {
    // White noise (Angle/Velocity Random Walk)
    Real white_noise;           // Units: rad/s/sqrt(Hz) or m/s^2/sqrt(Hz)

    // Bias instability (1/f flicker noise)
    Real bias_instability;      // Units: rad/s or m/s^2
    Real bias_time_constant;    // Time constant (seconds)

    // Rate/Velocity random walk
    Real rate_random_walk;      // Units: rad/s^2/sqrt(Hz) or m/s^3/sqrt(Hz)

    // Scale factor error
    Real scale_factor_error;    // Fractional error (e.g., 0.001 = 0.1%)

    // Quantization
    Real quantization;          // Units: rad or m/s
};

struct Vec3NoiseModel {
    AxisNoiseModel x, y, z;
};
```

### Measurement Output

```cpp
struct Vec3Measurement {
    Vec3 value;                 // Measured value
    Vec3 uncertainty;           // 1-sigma uncertainty
    bool valid{true};           // Measurement validity
    Real timestamp{0.0};        // Measurement timestamp
};
```

## IMU Sensor (`imu_sensor.h`)

### IMU Grades

```cpp
enum class IMUGrade : UInt8 {
    Consumer    = 0,    // Smartphone MEMS
    Industrial  = 1,    // Industrial MEMS
    Tactical    = 2,    // Tactical grade
    Navigation  = 3,    // Navigation grade (ring laser/FOG)
    Strategic   = 4,    // Strategic/Marine grade
    Custom      = 10    // User-defined parameters
};
```

### Grade Specifications

| Grade | Accel Noise (ug/sqrt(Hz)) | Accel Bias (ug) | Gyro Noise (deg/s/sqrt(Hz)) | Gyro Bias (deg/hr) |
|-------|--------------------------|-----------------|-----------------------------|--------------------|
| Consumer | 400 | 40 | 0.01 | 10 |
| Industrial | 200 | 20 | 0.005 | 5 |
| Tactical | 100 | 10 | 0.003 | 1 |
| Navigation | 10 | 1 | 0.001 | 0.01 |
| Strategic | 1 | 0.1 | 0.0003 | 0.001 |

### IMU Configuration

```cpp
IMUConfig config;

// Select grade preset
config.configure_from_grade(IMUGrade::Tactical);

// Or custom configuration
config.accelerometer.noise.x.white_noise = 100e-6 * 9.81;  // 100 ug
config.accelerometer.noise.x.bias_instability = 10e-6 * 9.81;
config.accelerometer.range = 16.0 * 9.81;  // +/- 16g

config.gyroscope.noise.x.white_noise = 0.003 * DEG_TO_RAD;  // 0.003 deg/s
config.gyroscope.noise.x.bias_instability = 1.0 / 3600 * DEG_TO_RAD;  // 1 deg/hr
config.gyroscope.range = 2000.0 * DEG_TO_RAD;  // +/- 2000 deg/s

// G-sensitivity (gyro error from acceleration)
config.gyroscope.g_sensitivity = Vec3{0.001, 0.001, 0.001};  // rad/s per g

// Temperature effects
config.enable_temperature_effects = true;
config.operating_temperature = 25.0;   // Celsius
config.reference_temperature = 25.0;
config.temp_sensitivity = 0.01;        // 1% per degree C

// Coning/sculling compensation
config.enable_coning_compensation = true;
config.enable_sculling_compensation = true;

// Update rate
config.update_rate = 200.0;  // 200 Hz
```

### Creating IMU Sensors

```cpp
// From configuration
IMUSensor imu(config);

// From grade preset
IMUSensor imu("tactical_imu", IMUGrade::Tactical, entity_id);

// Factory functions
auto consumer_imu = create_consumer_imu("smartphone", entity_id);
auto tactical_imu = create_tactical_imu("flight_imu", entity_id);
auto nav_imu = create_navigation_imu("ins", entity_id);
```

### IMU Usage

```cpp
IMUConfig config;
config.configure_from_grade(IMUGrade::Tactical);
config.attached_entity = aircraft_id;

IMUSensor imu(config);
imu.initialize();

// In simulation loop
imu.update(entity_state, dt);

// Get measurements
const Vec3Measurement& accel = imu.get_acceleration();
const Vec3Measurement& gyro = imu.get_angular_velocity();

if (accel.valid) {
    Vec3 specific_force = accel.value;  // m/s^2 in body frame
    Vec3 accel_sigma = accel.uncertainty;
}

if (gyro.valid) {
    Vec3 angular_rate = gyro.value;  // rad/s in body frame
    Vec3 gyro_sigma = gyro.uncertainty;
}
```

### True vs Measured Values

```cpp
// Noise-free (truth) values for validation
Vec3 true_accel = imu.get_true_acceleration();
Vec3 true_gyro = imu.get_true_angular_velocity();

// Measured values (with noise)
Vec3 measured_accel = imu.get_acceleration().value;
Vec3 measured_gyro = imu.get_angular_velocity().value;

// Noise = measured - true
Vec3 accel_error = measured_accel - true_accel;
```

### Integrated Outputs

For strapdown inertial navigation:

```cpp
// Get velocity increment (sculling-compensated)
Vec3 delta_v = imu.get_delta_v();  // m/s, body frame

// Get angle increment (coning-compensated)
Vec3 delta_theta = imu.get_delta_theta();  // radians, body frame

// Note: These are auto-cleared after reading
```

### Bias Estimation

For Kalman filter integration:

```cpp
// Get current bias estimates
Vec3 accel_bias = imu.get_accel_bias();
Vec3 gyro_bias = imu.get_gyro_bias();

// Apply external bias correction
imu.set_accel_bias(estimated_accel_bias);
imu.set_gyro_bias(estimated_gyro_bias);
```

### Temperature Effects

```cpp
// Enable temperature modeling
config.enable_temperature_effects = true;
config.reference_temperature = 25.0;  // Calibration temperature

// During simulation
imu.set_temperature(30.0);  // Current temperature

Real temp = imu.get_temperature();
```

## Noise Model Details

### White Noise (Random Walk)

Models high-frequency noise:
- Accelerometer: Velocity Random Walk (VRW)
- Gyroscope: Angle Random Walk (ARW)

```cpp
// Allan variance coefficient
Real arw = 0.003 * DEG_TO_RAD;  // rad/s/sqrt(Hz)

// Convert to sigma per sample at dt
Real sigma_per_sample = arw * std::sqrt(1.0 / dt);
```

### Bias Instability

Models slowly varying bias:

```cpp
// Bias instability (1/f flicker noise)
Real bias_instability = 1.0 / 3600 * DEG_TO_RAD;  // 1 deg/hr

// Time constant for Markov model
Real tau = 100.0;  // seconds
```

### Rate Random Walk

Models bias drift:

```cpp
// Rate random walk coefficient
Real rrw = 1e-5 * DEG_TO_RAD;  // rad/s^2/sqrt(Hz)
```

### Scale Factor Error

Models gain error:

```cpp
// 0.1% scale factor error
Real sf_error = 0.001;

// Applied as: measured = true * (1 + sf_error)
```

## Sensor Failure Modes

```cpp
enum class SensorFailureMode : UInt8 {
    None = 0,
    Stuck = 1,          // Output frozen at last value
    Drift = 2,          // Accelerating bias drift
    Noise = 3,          // Increased noise level
    Saturated = 4,      // Output at maximum value
    Dead = 5            // No output
};

// Apply failure
imu.set_failure_mode(SensorFailureMode::Drift);

// Check status
bool healthy = imu.is_valid();
```

## Cross-Axis Sensitivity

```cpp
// Accelerometer cross-axis coupling
config.accelerometer.cross_axis_sensitivity = 0.01;  // 1%

// This models:
// a_measured_x = a_true_x + 0.01 * a_true_y + 0.01 * a_true_z
```

## Integration with Navigation

### Strapdown INS Update

```cpp
void strapdown_update(IMUSensor& imu, NavigationState& nav, Real dt) {
    // Get compensated increments
    Vec3 delta_v = imu.get_delta_v();
    Vec3 delta_theta = imu.get_delta_theta();

    // Apply bias corrections
    delta_v -= imu.get_accel_bias() * dt;
    delta_theta -= imu.get_gyro_bias() * dt;

    // Update attitude (quaternion integration)
    Quat dq = Quat::from_rotation_vector(delta_theta);
    nav.attitude = nav.attitude * dq;
    nav.attitude = nav.attitude.normalized();

    // Rotate velocity increment to navigation frame
    Vec3 delta_v_nav = nav.attitude.rotate(delta_v);

    // Add gravity and Coriolis corrections
    delta_v_nav.z += 9.81 * dt;  // Simplified gravity

    // Update velocity and position
    nav.velocity += delta_v_nav;
    nav.position += nav.velocity * dt;
}
```

### Kalman Filter Integration

```cpp
// IMU provides measurement for EKF
Vec3Measurement accel = imu.get_acceleration();
Vec3Measurement gyro = imu.get_angular_velocity();

// Use uncertainty for measurement noise
Matrix3x3 R_accel = Matrix3x3::Diagonal(
    accel.uncertainty.x * accel.uncertainty.x,
    accel.uncertainty.y * accel.uncertainty.y,
    accel.uncertainty.z * accel.uncertainty.z
);

// Bias states can be estimated
nav_filter.set_accel_bias_state(imu.get_accel_bias());
nav_filter.set_gyro_bias_state(imu.get_gyro_bias());

// After filter update, apply corrections
imu.set_accel_bias(nav_filter.estimated_accel_bias());
imu.set_gyro_bias(nav_filter.estimated_gyro_bias());
```

## Performance Validation

### Allan Variance Analysis

```cpp
// Collect long time series at static condition
std::vector<Vec3> gyro_data;
for (int i = 0; i < 360000; ++i) {  // 1 hour at 100 Hz
    imu.update(static_state, 0.01);
    gyro_data.push_back(imu.get_angular_velocity().value);
}

// Compute Allan variance
// Angle Random Walk: slope = -0.5 at short tau
// Bias Instability: minimum of Allan deviation
// Rate Random Walk: slope = +0.5 at long tau
```

### Static Test

```cpp
// Zero-input test
physics::EntityState static_state;
static_state.velocity = Vec3::Zero();
static_state.angular_velocity = Vec3::Zero();
static_state.acceleration = Vec3{0, 0, -9.81};  // Gravity

for (int i = 0; i < 6000; ++i) {  // 1 minute at 100 Hz
    imu.update(static_state, 0.01);

    Vec3 accel = imu.get_acceleration().value;
    Vec3 gyro = imu.get_angular_velocity().value;

    // Accelerometer should read approximately [0, 0, 9.81]
    // Gyroscope should read approximately [0, 0, 0]
}
```

## Best Practices

### Initialization

```cpp
// Allow warm-up period for bias to stabilize
Real warmup_time = 30.0;  // seconds
while (sim_time < warmup_time) {
    imu.update(entity_state, dt);
}
// Now begin navigation
```

### Update Rate

```cpp
// IMU should update at higher rate than dynamics
config.update_rate = 200.0;  // 200 Hz for IMU
// Dynamics at 100 Hz
// Navigation filter at 50 Hz
```

### Coordinate Frames

```cpp
// IMU measures in body frame:
// X: forward
// Y: right
// Z: down

// Accelerometer measures specific force (not gravity)
// At rest: accel_z = +9.81 (up acceleration to counteract gravity)

// Convert specific force to inertial acceleration:
// a_inertial = a_specific + R_body_to_ned * g_ned
```

## Reference Standards

- **IEEE Std 952-1997**: Specification Format for Single-Axis IMU
- **MIL-STD-1553**: Aircraft Internal Time Division Multiplex Data Bus
- **MIL-F-8785C**: Flying Qualities of Piloted Aircraft
