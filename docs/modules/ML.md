# ML Module Documentation

The ML module provides infrastructure for machine learning integration in JaguarEngine, including ONNX Runtime-based model inference, model repository management, neural network-based autopilot control, and reinforcement learning environment support.

## Headers

| Header | Purpose |
|--------|---------|
| `jaguar/ml/inference_session.h` | ONNX Runtime wrapper for ML model inference |
| `jaguar/ml/model_repository.h` | ML model management and versioning system |
| `jaguar/ml/neural_autopilot.h` | Neural network-based autonomous vehicle control |
| `jaguar/ml/rl_environment.h` | OpenAI Gym-compatible RL training environment |

## InferenceSession - Model Inference

The InferenceSession provides a high-level interface for running ONNX neural network models with support for multiple execution providers, asynchronous execution, and comprehensive performance tracking.

### Execution Providers

Supports acceleration on various hardware:

```cpp
enum class ExecutionProvider {
    CPU,          // Default CPU execution
    CUDA,         // NVIDIA GPU via CUDA
    TensorRT,     // NVIDIA TensorRT optimization
    CoreML,       // Apple macOS/iOS devices
    DirectML,     // Windows DirectX 12 GPUs
    OpenVINO      // Intel hardware
};
```

Check provider availability at runtime:

```cpp
if (ml::is_provider_available(ExecutionProvider::CUDA)) {
    config = ml::InferenceConfig::cuda();
} else {
    config = ml::InferenceConfig::cpu();
}
```

### Data Types

Tensors support multiple data types:

```cpp
enum class DataType {
    Float32,   // 32-bit floating point (default)
    Float64,   // 64-bit floating point
    Int32,     // 32-bit signed integer
    Int64,     // 64-bit signed integer
    UInt8,     // 8-bit unsigned integer
    Bool       // Boolean (1 byte)
};
```

### TensorShape and TensorInfo

Define tensor geometry:

```cpp
// TensorShape represents tensor dimensions
TensorShape shape{1, 3, 224, 224};     // Batch=1, Channels=3, Height=224, Width=224
Int64 element_count = shape.element_count();  // Returns 150528
bool is_dynamic = shape.is_dynamic();         // Checks for -1 dimensions

// TensorInfo includes metadata
TensorInfo input_info("input_image",
                      DataType::Float32,
                      TensorShape{1, 3, 224, 224},
                      true);  // is_input

UInt64 size_bytes = input_info.size_in_bytes();  // Returns 150528 * 4 = 602112
```

### Tensor - Data Container

Type-safe tensor data access:

```cpp
// Create tensor
Tensor tensor("input", DataType::Float32, TensorShape{1, 3, 224, 224});

// Set data from typed vector
std::vector<float> image_data(150528);
// ... fill image_data ...
tensor.set_data(image_data);

// Access typed data
float* ptr = tensor.data_as<float>();
if (ptr) {
    // Use pointer safely
}

// Query properties
Int64 elements = tensor.element_count();      // 150528
UInt64 bytes = tensor.size_in_bytes();        // 602112
```

Helper function for tensor creation:

```cpp
// Type-safe tensor creation
std::vector<float> data{1.0f, 2.0f, 3.0f, ...};
auto tensor = ml::create_tensor("input",
                                TensorShape{1, 3, 224, 224},
                                data);

// Validate tensors against expected schema
auto input_info = session->get_input_info();
bool valid = ml::validate_tensors(input_tensors, input_info, true);

// Calculate memory usage
UInt64 total_mem = ml::calculate_tensor_memory(tensors);
```

### InferenceConfig - Configuration

Pre-configured settings for common scenarios:

```cpp
// CPU configuration
auto cpu_config = InferenceConfig::cpu(4);  // 4 threads

// CUDA GPU configuration
auto cuda_config = InferenceConfig::cuda(0);  // Device 0

// TensorRT with longer timeout for compilation
auto tensorrt_config = InferenceConfig::tensorrt();

// Low-latency configuration (minimal threading)
auto low_lat = InferenceConfig::low_latency();

// High-performance CPU configuration
auto fast = InferenceConfig::cpu_fast();

// Profiling configuration
auto prof = InferenceConfig::profiling();

// Custom configuration
InferenceConfig custom;
custom.provider = ExecutionProvider::CUDA;
custom.device_id = 0;
custom.intra_op_threads = 8;
custom.inter_op_threads = 4;
custom.enable_profiling = true;
custom.timeout = std::chrono::milliseconds(5000);
custom.enable_memory_arena = true;
```

### Inference Session Lifecycle

```cpp
#include <jaguar/ml/inference_session.h>

// Create session
auto config = ml::InferenceConfig::cuda();
auto session = ml::create_inference_session(config);

// Initialize
if (session->initialize() != ml::InferenceResult::Success) {
    // Handle error
}

// Load model
auto result = session->load_model("models/autopilot_network.onnx");
if (result != ml::InferenceResult::Success) {
    std::cout << "Load failed: " << ml::inference_result_to_string(result) << std::endl;
}

// Get model signature
auto inputs = session->get_input_info();
auto outputs = session->get_output_info();

for (const auto& input : inputs) {
    std::cout << "Input: " << ml::format_tensor_info(input) << std::endl;
}

// Warmup for stable performance
session->warmup(5);

// Run inference
std::vector<ml::Tensor> input_tensors;
// ... populate input_tensors ...

std::vector<ml::Tensor> output_tensors;
result = session->run(input_tensors, output_tensors);

// Get statistics
auto stats = session->get_stats();
std::cout << "Inference time: " << stats.average_inference_time().count() << " ns" << std::endl;
std::cout << "Success rate: " << stats.success_rate() << std::endl;
std::cout << "Throughput: " << stats.throughput() << " inferences/sec" << std::endl;

// Shutdown
session->shutdown();
```

### Asynchronous Inference

Non-blocking inference with callbacks:

```cpp
session->run_async(input_tensors,
    [](ml::InferenceResult result, std::vector<ml::Tensor> outputs) {
        if (result == ml::InferenceResult::Success) {
            float* output_ptr = outputs[0].data_as<float>();
            // Process results
        }
    });

// Continue without waiting for inference
```

### Inference Statistics

Track performance metrics:

```cpp
auto stats = session->get_stats();

// Basic metrics
UInt64 total = stats.total_inferences;
UInt64 successful = stats.successful_inferences;
UInt64 failed = stats.failed_inferences;

// Timing analysis
auto avg_time = stats.average_inference_time();
auto min_time = stats.min_inference_time;
auto max_time = stats.max_inference_time;

// Aggregated metrics
Real success_rate = stats.success_rate();          // 0.0 to 1.0
Real throughput = stats.throughput();              // inferences per second

// Reset statistics
session->reset_stats();
```

## ModelRepository - Model Management

The ModelRepository manages ML models with semantic versioning, integrity verification, caching, and deployment support.

### Semantic Versioning

Version comparison and compatibility:

```cpp
ml::SemanticVersion v1(1, 2, 3);   // 1.2.3
ml::SemanticVersion v2(1, 3, 0);   // 1.3.0

bool newer = v2 > v1;              // true
bool compatible = v1.is_compatible_with(v2);  // true (same major)

// Parse from string
auto v3 = ml::SemanticVersion::parse("2.0.0");
std::string version_str = v1.to_string();  // "1.2.3"
```

### Model Types

Categorize models by purpose:

```cpp
enum class ModelType {
    Autopilot,      // Flight/ship control
    Behavior,       // Tactical AI decisions
    Perception,     // Sensor processing
    Prediction,     // Trajectory/state prediction
    Classification, // Object classification
    Custom          // User-defined
};
```

### ModelMetadata - Model Information

Comprehensive model metadata:

```cpp
ml::ModelMetadata metadata("autopilot-v1",
                           "F-16 Neural Autopilot",
                           ml::ModelType::Autopilot,
                           ml::SemanticVersion(1, 0, 0));

metadata.author = "ML Research Team";
metadata.description = "Deep learning autopilot for fighter aircraft";
metadata.checksum = "abc123...";  // SHA256
metadata.size_bytes = 5242880;    // 5MB

// Add tensor specifications
metadata.inputs.push_back(
    ml::TensorInfo("state", {1, 24}, "float32",
                   "Aircraft state vector [pos, vel, att, ...]"));
metadata.outputs.push_back(
    ml::TensorInfo("actions", {1, 4}, "float32",
                   "Control actions [elevator, aileron, rudder, throttle]"));

// Tag models for organization
metadata.add_tag("architecture", "mlp");
metadata.add_tag("training_date", "2026-01-15");
metadata.add_tag("platform", "f16");

// Get full identifier
std::string full_id = metadata.full_id();  // "autopilot-v1@1.0.0"

// Retrieve tags
if (auto tag = metadata.get_tag("architecture")) {
    std::cout << "Architecture: " << *tag << std::endl;
}
```

### ModelQuery - Searching Models

Find models by various criteria:

```cpp
// Query all Autopilot models
auto query = ml::ModelQuery::for_type(ml::ModelType::Autopilot);

// Query by name pattern (regex)
auto query = ml::ModelQuery::for_name(".*autopilot.*");

// Query by version range
auto query = ml::ModelQuery::for_version_range(
    ml::SemanticVersion(1, 0, 0),
    ml::SemanticVersion(2, 0, 0));

// Query by tag
auto query = ml::ModelQuery::for_tag("platform:f16");

// Custom query with multiple filters
ml::ModelQuery query;
query.type = ml::ModelType::Autopilot;
query.name_pattern = "neural.*";
query.min_version = ml::SemanticVersion(1, 0, 0);
query.limit = 50;
query.include_deprecated = false;
```

### Cache Configuration

Manage memory and model availability:

```cpp
// Default cache (1GB, 100 models, 30min timeout)
auto cache = ml::CacheConfig::default_config();

// Aggressive cache (256MB, 10 models, 10min timeout)
auto aggressive = ml::CacheConfig::aggressive();

// Relaxed cache (4GB, 500 models, 2hr timeout)
auto relaxed = ml::CacheConfig::relaxed();

// Custom cache
ml::CacheConfig custom;
custom.max_memory_bytes = 2147483648;  // 2GB
custom.max_models = 50;
custom.eviction_timeout = std::chrono::minutes(15);
custom.enable_preloading = true;
```

### Repository Configuration

```cpp
// Default repository configuration
auto config = ml::RepositoryConfig::default_config();
config.base_path = "./models";
config.cache = ml::CacheConfig::default_config();
config.verify_checksums = true;
config.auto_update = false;

// In-memory repository (no persistence)
auto in_memory = ml::RepositoryConfig::in_memory();

// Persistent disk-backed repository
auto persistent = ml::RepositoryConfig::persistent("./model_store");
```

### Repository Lifecycle

```cpp
// Create repository
auto config = ml::RepositoryConfig::default_config();
auto repo = ml::create_model_repository(config);

// Initialize
repo->initialize();

// Register a model
ml::ModelMetadata meta("autopilot-v1", "Autopilot",
                       ml::ModelType::Autopilot,
                       ml::SemanticVersion(1, 0, 0));
meta.description = "Neural autopilot for fighters";
meta.add_tag("architecture", "mlp");

repo->register_model("./models/autopilot_v1.onnx", meta);

// Query models
auto query = ml::ModelQuery::for_type(ml::ModelType::Autopilot);
auto results = repo->query(query);

for (const auto& entry : results) {
    std::cout << "Model: " << entry.metadata.full_id() << std::endl;
    std::cout << "Status: " << ml::model_status_to_string(entry.status) << std::endl;
}

// Load model data
if (auto data = repo->load_model_data("autopilot-v1")) {
    // Use model data for inference
}

// Find latest version of a model
auto entries = repo->query(ml::ModelQuery::for_type(ml::ModelType::Autopilot));
if (auto latest = ml::find_latest_version(entries)) {
    std::cout << "Latest: " << latest->metadata.full_id() << std::endl;
}

// Export catalog for deployment
repo->export_catalog("./model_catalog.json");

// Import catalog
repo->import_catalog("./deployment_models.json");

// Get repository statistics
auto stats = repo->get_stats();
std::cout << "Total models: " << stats.total_models << std::endl;
std::cout << "Cached models: " << stats.cached_models << std::endl;
std::cout << "Cache utilization: " << (stats.cache_utilization(config.cache) * 100.0) << "%" << std::endl;
std::cout << "Cache hit rate: " << (stats.cache_hit_rate() * 100.0) << "%" << std::endl;
```

### Repository Statistics

```cpp
auto stats = repo->get_stats();

// Model counts
std::cout << "Total: " << stats.total_models << std::endl;
std::cout << "Cached: " << stats.cached_models << std::endl;

// Cache analysis
std::cout << "Memory used: " << stats.cache_memory_used / 1024 / 1024 << " MB" << std::endl;
std::cout << "Hit rate: " << (stats.cache_hit_rate() * 100.0) << "%" << std::endl;

// Models by type
auto autopilot_count = stats.count_for_type(ml::ModelType::Autopilot);
```

## NeuralAutopilot - Autonomous Control

The NeuralAutopilot uses trained neural networks to control vehicles autonomously with safety constraint enforcement and observation normalization.

### Autopilot Modes

Different operational modes:

```cpp
enum class AutopilotMode {
    Manual,      // Pilot control only
    Waypoint,    // Navigate to waypoint
    Altitude,    // Maintain altitude
    Speed,       // Maintain speed
    Course,      // Maintain heading/course
    Formation,   // Formation flying/sailing
    Intercept,   // Intercept target
    Loiter       // Orbit/station keeping
};
```

### Vehicle Types

Autopilot variants for different platforms:

```cpp
enum class VehicleType {
    Aircraft,      // Fixed-wing
    Rotorcraft,    // Helicopter/quadcopter
    Ship,          // Surface vessel
    Submarine,     // Underwater vehicle
    GroundVehicle  // Land vehicle
};
```

### AutopilotObservation - State Vector

Complete vehicle state:

```cpp
ml::AutopilotObservation obs;

// Position and velocity
obs.position = Vec3{1000.0, 2000.0, 500.0};    // World coordinates
obs.velocity = Vec3{50.0, 0.0, -5.0};          // Body frame

// Orientation
obs.orientation = ml::Quaternion::Identity();
obs.angular_velocity = Vec3{0.0, 0.1, 0.0};    // Roll, pitch, yaw rates

// Flight state
obs.altitude = 500.0;      // meters
obs.airspeed = 50.0;       // m/s
obs.heading = 0.0;         // radians (0 = North)
obs.pitch = 0.05;          // radians (~3 degrees)
obs.roll = 0.0;

// Target parameters
obs.target_position = Vec3{5000.0, 5000.0, 500.0};
obs.target_altitude = 500.0;
obs.target_speed = 50.0;
obs.target_heading = 0.0;

obs.timestamp = std::chrono::system_clock::now();
```

### AutopilotAction - Control Commands

Normalized control outputs:

```cpp
ml::AutopilotAction action;

// Control surfaces (-1 to 1)
action.elevator = 0.1;     // Pitch control (up)
action.aileron = -0.2;     // Roll control (left)
action.rudder = 0.0;       // Yaw control

// Power
action.throttle = 0.8;     // 0 to 1
action.collective = 0.5;   // For rotorcraft, 0 to 1

// Configuration
action.flaps = 0.0;        // 0 to 1
action.speedbrake = 0.0;   // 0 to 1
action.gear_down = true;   // Landing gear

// Preset actions
auto neutral = ml::AutopilotAction::neutral();
auto full_power = ml::AutopilotAction::full_throttle();
auto idle = ml::AutopilotAction::idle();
```

### Safety Constraints

Enforce operational limits:

```cpp
// Default aircraft constraints
auto constraints = ml::SafetyConstraints::default_aircraft();
constraints.max_pitch_rate = 10.0;   // deg/s
constraints.max_roll_rate = 30.0;
constraints.max_yaw_rate = 10.0;
constraints.max_g_load = 4.0;
constraints.min_altitude = 0.0;
constraints.max_altitude = 15000.0;  // meters
constraints.min_speed = 20.0;        // m/s
constraints.max_speed = 250.0;
constraints.max_bank_angle = 60.0;   // degrees
constraints.max_pitch_angle = 30.0;

// Ship constraints (gentler)
auto ship = ml::SafetyConstraints::default_ship();

// Strict safety constraints
auto strict = ml::SafetyConstraints::strict();

// Permissive constraints (testing)
auto permissive = ml::SafetyConstraints::permissive();
```

### AutopilotConfig - Configuration

```cpp
// Aircraft autopilot
auto config = ml::AutopilotConfig::aircraft("models/f16_autopilot.onnx");

// Ship autopilot
auto config = ml::AutopilotConfig::ship("models/destroyer_autopilot.onnx");

// Rotorcraft autopilot
auto config = ml::AutopilotConfig::rotorcraft("models/helicopter_autopilot.onnx");

// Custom configuration
ml::AutopilotConfig config;
config.vehicle_type = ml::VehicleType::Aircraft;
config.model_path = "models/custom_autopilot.onnx";
config.safety = ml::SafetyConstraints::default_aircraft();
config.control_frequency = 50.0;  // Hz
config.enable_safety_limits = true;
config.log_actions = false;
```

### Neural Autopilot Usage

```cpp
#include <jaguar/ml/neural_autopilot.h>

// Create autopilot
auto config = ml::AutopilotConfig::aircraft("models/f16_autopilot.onnx");
auto autopilot = ml::create_neural_autopilot(config);

// Initialize
autopilot->initialize();

// Load model
autopilot->load_model("models/f16_autopilot.onnx");

// Set mode and targets
autopilot->set_mode(ml::AutopilotMode::Waypoint);
autopilot->set_target(Vec3{5000.0, 5000.0, 1000.0});
autopilot->set_target_altitude(1000.0);
autopilot->set_target_speed(100.0);

// In simulation loop
ml::AutopilotObservation obs;
// ... populate observation from vehicle state ...

ml::AutopilotAction action;
auto result = autopilot->compute(obs, action);

if (result == ml::AutopilotResult::Success) {
    // Apply action to vehicle
    vehicle.set_elevator(action.elevator);
    vehicle.set_aileron(action.aileron);
    vehicle.set_rudder(action.rudder);
    vehicle.set_throttle(action.throttle);
}

// Get statistics
auto stats = autopilot->get_stats();
std::cout << "Steps: " << stats.total_steps << std::endl;
std::cout << "Safety interventions: " << stats.safety_interventions << std::endl;
std::cout << "Avg inference time: " << stats.average_inference_ms << " ms" << std::endl;

// Shutdown
autopilot->shutdown();
```

### Observation Normalization

Custom normalizer interface:

```cpp
class MyNormalizer : public ml::IObservationNormalizer {
public:
    std::vector<Real> normalize(const ml::AutopilotObservation& obs) override {
        std::vector<Real> normalized;
        // Normalize position, velocity, attitude, etc.
        // Custom scaling and offset logic
        return normalized;
    }

    ml::AutopilotObservation denormalize(const std::vector<Real>& norm) override {
        ml::AutopilotObservation obs;
        // Reverse normalization
        return obs;
    }
};

auto normalizer = std::make_shared<MyNormalizer>();
```

### Helper Functions

Convenience utilities for autopilot development:

```cpp
// Validate observation and action
bool valid_obs = ml::validate_observation(obs);
bool valid_action = ml::validate_action(action);

// Clamp action to valid ranges
auto clamped = ml::clamp_action(action);

// Control surface effectiveness based on airspeed
Real effectiveness = ml::control_effectiveness(airspeed, 20.0);

// Distance to target
Real dist = ml::distance_to_target(obs.position, obs.target_position);

// Heading error in [-pi, pi]
Real err = ml::heading_error(obs.heading, obs.target_heading);

// Altitude error
Real alt_err = ml::altitude_error(obs.altitude, obs.target_altitude);

// Speed error
Real speed_err = ml::speed_error(obs.airspeed, obs.target_speed);

// Bearing to target
Real bearing = ml::bearing_to_target(obs.position, obs.target_position);

// Coordinate conversions
Real rad = ml::deg_to_rad(45.0);  // 45 degrees to radians
Real deg = ml::rad_to_deg(3.14159 / 4.0);

// Aerodynamic calculations
Real stall_speed = ml::estimate_stall_speed(mass, wing_area, cl_max);
Real turn_radius = ml::turn_radius(velocity, bank_angle);
Real required_bank = ml::required_bank_angle(velocity, desired_radius);
Real g_load = ml::load_factor_from_bank(bank_angle);

// Goal checking
bool at_alt = ml::at_target_altitude(current_alt, target_alt, 10.0);
bool at_speed = ml::at_target_speed(current_speed, target_speed, 2.0);
bool at_heading = ml::at_target_heading(curr_hdg, tgt_hdg, ml::deg_to_rad(5.0));
bool at_waypoint = ml::at_waypoint(position, target, 50.0);

// Rate limiting
auto limited = ml::apply_rate_limits(current_action, target_action, 1.0, dt);

// Interpolation
auto blended = ml::interpolate_actions(action1, action2, 0.5);  // 50% blend

// Reporting
auto report = ml::generate_observation_report(obs);
auto action_report = ml::generate_action_report(action);
```

## RLEnvironment - Reinforcement Learning

OpenAI Gym-compatible environment for training RL agents in aerospace scenarios.

### Space Types

Define observation and action spaces:

```cpp
enum class SpaceType {
    Discrete,        // Single value 0 to n-1
    Box,             // Continuous bounded space
    MultiDiscrete,   // Multiple discrete values
    MultiBinary,     // Binary vector
    Dict,            // Named spaces
    Tuple            // Composite spaces
};

// Create discrete space (for discrete actions)
auto action_space = ml::Space::discrete(9);  // 9 actions

// Create continuous box space
std::vector<Real> low = {-100, -100, -100, -100, -100, -100};
std::vector<Real> high = {100, 100, 100, 100, 100, 100};
auto obs_space = ml::Space::box(low, high, {6});

// Create multi-discrete space (multiple independent discrete values)
auto multi_space = ml::Space::multi_discrete({4, 3, 2});  // 4, 3, 2 discrete choices

// Create multi-binary space (binary vector)
auto binary_space = ml::Space::multi_binary(8);  // 8 binary values
```

### Observation and Action

Data containers with space information:

```cpp
// Create observation
ml::Observation obs(observation_space);
obs.data = {0.0, 1.0, 2.0, ...};  // Actual observation values

// Create continuous action
std::vector<Real> action_values = {0.5, -0.2, 0.8};
auto action = ml::Action::continuous(action_values, action_space);

// Create discrete action
auto discrete_action = ml::Action::discrete(3, action_space);
```

### Reward Configuration

Customize reward shaping:

```cpp
// Default balanced rewards
auto rewards = ml::RewardConfig::default_config();
rewards.success_reward = 100.0;
rewards.failure_penalty = -100.0;
rewards.step_penalty = -0.1;
rewards.distance_weight = 1.0;
rewards.time_weight = 0.1;
rewards.safety_penalty = -10.0;

// Sparse reward (only on success/failure)
auto sparse = ml::RewardConfig::sparse();

// Dense reward (continuous feedback)
auto dense = ml::RewardConfig::dense();
```

### Environment Configuration

Pre-configured tasks:

```cpp
// Waypoint navigation task
auto config = ml::EnvironmentConfig::waypoint_navigation("scenario.json");
// Obs: [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, goal_x, goal_y, goal_z, dist_to_goal]
// Act: [thrust, pitch_rate, yaw_rate, roll_rate]

// Formation flight task
auto config = ml::EnvironmentConfig::formation_flight("scenario.json");
// Obs: own state + leader state + relative position
// Act: continuous control

// Intercept task
auto config = ml::EnvironmentConfig::intercept("scenario.json");
// Obs: own state + target state + relative state (26 dims)
// Act: discrete maneuver selection (9 options)

// Custom configuration
ml::EnvironmentConfig config;
config.scenario_path = "my_scenario.json";
config.max_steps = 1000;
config.time_step = 0.02;  // 50Hz
config.observation_space = obs_space;
config.action_space = action_space;
config.reward_config = rewards;
config.render_enabled = false;
config.seed = 42;
```

### RL Environment Lifecycle

```cpp
#include <jaguar/ml/rl_environment.h>

// Create environment
auto config = ml::EnvironmentConfig::waypoint_navigation("scenario.json");
auto env = ml::create_rl_environment(config);

// Initialize
env->initialize();

// Set seed for reproducibility
env->set_seed(42);

// Reset to initial state
ml::Observation initial_obs;
env->reset(initial_obs);

// Training loop
for (int episode = 0; episode < 100; ++episode) {
    env->reset(initial_obs);

    Real episode_reward = 0.0;
    int step = 0;

    while (step < config.max_steps) {
        // Choose action (from RL agent)
        ml::Action action = agent.select_action(initial_obs);

        // Take environment step
        ml::StepResult result;
        env->step(action, result);

        // Collect experience
        episode_reward += result.reward;
        agent.learn(initial_obs, action, result);

        initial_obs = result.observation;
        step++;

        if (result.is_done()) {
            break;
        }
    }

    // Episode statistics
    auto episode_info = env->get_episode_info();
    std::cout << ml::format_episode_info(episode_info) << std::endl;
}

// Get environment statistics
auto stats = env->get_stats();
std::cout << "Total episodes: " << stats.total_episodes << std::endl;
std::cout << "Avg reward: " << stats.average_reward << std::endl;
std::cout << "Success rate: " << (stats.success_rate * 100.0) << "%" << std::endl;

// Shutdown
env->shutdown();
```

### Custom Reward Function

Implement custom reward logic:

```cpp
class DistanceReward : public ml::IRewardFunction {
private:
    Real weight_;

public:
    DistanceReward(Real weight) : weight_(weight) {}

    Real compute(const ml::Observation& obs,
                 const ml::Action& action,
                 const ml::Observation& next_obs,
                 bool done) override {
        // Example: reward negative distance to goal
        if (obs.data.size() >= 10) {
            Real current_dist = obs.data[9];     // distance_to_goal
            Real next_dist = next_obs.data[9];
            Real dist_reward = (current_dist - next_dist) * weight_;
            return dist_reward;
        }
        return 0.0;
    }

    void reset() override {}
};

auto reward_fn = std::make_shared<DistanceReward>(1.0);
env->set_reward_function(reward_fn);
```

### Custom Termination Condition

Define episode ending:

```cpp
class GoalTermination : public ml::ITerminationCondition {
private:
    Real threshold_;
    std::string reason_;

public:
    GoalTermination(Real threshold) : threshold_(threshold) {}

    bool is_terminated(const ml::Observation& obs, UInt32 step) override {
        if (obs.data.size() >= 10) {
            Real distance = obs.data[9];  // distance_to_goal
            if (distance < threshold_) {
                reason_ = "Goal reached";
                return true;
            }
        }
        return false;
    }

    bool is_truncated(const ml::Observation& obs, UInt32 step) override {
        if (step >= 1000) {
            reason_ = "Step limit exceeded";
            return true;
        }
        return false;
    }

    std::string get_reason() const override { return reason_; }
};

auto termination = std::make_shared<GoalTermination>(10.0);
env->set_termination_condition(termination);
```

### Step Result

Complete step information:

```cpp
ml::StepResult result;

// Observation after action
const ml::Observation& next_obs = result.observation;

// Reward for this step
Real reward = result.reward;

// Episode completion status
bool episode_ended = result.terminated;      // Natural termination
bool episode_truncated = result.truncated;   // Time limit, etc.
bool is_done = result.is_done();             // Either terminated or truncated

// Additional info
if (auto reason = result.info.find("termination_reason");
    reason != result.info.end()) {
    std::cout << "Reason: " << reason->second << std::endl;
}
```

### Episode Statistics

```cpp
auto episode_info = env->get_episode_info();

UInt64 episode_num = episode_info.episode_number;
UInt32 steps = episode_info.step_count;
Real total_reward = episode_info.total_reward;
bool success = episode_info.success;
std::string reason = episode_info.termination_reason;

auto duration = episode_info.get_duration();
```

### Environment Statistics

```cpp
auto stats = env->get_stats();

UInt64 episodes = stats.total_episodes;
UInt64 total_steps = stats.total_steps;
Real avg_reward = stats.average_reward;
Real avg_length = stats.average_episode_length;
Real success_rate = stats.success_rate;
UInt64 successes = stats.successful_episodes;
```

### Helper Functions

Utility functions for RL:

```cpp
// Space manipulation
Int64 size = ml::get_space_size(action_space);
bool is_continuous = ml::is_continuous_space(obs_space);
bool is_discrete = ml::is_discrete_space(action_space);

// Value clipping
auto clipped = ml::clip_to_space(action_values, action_space);

// Normalization
auto normalized = ml::normalize_from_space(values, space);      // To [0, 1]
auto denormalized = ml::denormalize_to_space(norm, space);     // From [0, 1]

// Formatting
std::string ep_str = ml::format_episode_info(episode_info);

// Statistics calculation
auto [mean, std, min, max] = ml::calculate_reward_stats(episode_history);
std::cout << "Mean reward: " << mean << " +/- " << std << std::endl;
std::cout << "Range: [" << min << ", " << max << "]" << std::endl;
```

## Complete Example - Neural Autopilot with RL Training

```cpp
#include <jaguar/ml/inference_session.h>
#include <jaguar/ml/neural_autopilot.h>
#include <jaguar/ml/rl_environment.h>
#include <jaguar/ml/model_repository.h>

// Step 1: Initialize model repository
auto repo_config = ml::RepositoryConfig::default_config();
auto repo = ml::create_model_repository(repo_config);
repo->initialize();

// Step 2: Register trained models
ml::ModelMetadata autopilot_meta("autopilot-f16", "F-16 Autopilot",
                                  ml::ModelType::Autopilot,
                                  ml::SemanticVersion(1, 0, 0));
autopilot_meta.add_tag("platform", "f16");
repo->register_model("models/f16_autopilot_v1.onnx", autopilot_meta);

// Step 3: Create neural autopilot
auto ap_config = ml::AutopilotConfig::aircraft("models/f16_autopilot_v1.onnx");
auto autopilot = ml::create_aircraft_autopilot("models/f16_autopilot_v1.onnx");
autopilot->initialize();

// Step 4: Create RL training environment
auto env_config = ml::EnvironmentConfig::waypoint_navigation("scenarios/f16_nav.json");
auto env = ml::create_rl_environment(env_config);
env->initialize();

// Step 5: Training loop
for (int episode = 0; episode < 100; ++episode) {
    ml::Observation obs;
    env->reset(obs);

    Real episode_reward = 0.0;

    for (int step = 0; step < env_config.max_steps; ++step) {
        // Convert environment observation to autopilot observation
        ml::AutopilotObservation ap_obs;
        // ... map environment obs to ap_obs ...

        // Get autopilot action
        ml::AutopilotAction ap_action;
        auto result = autopilot->compute(ap_obs, ap_action);

        // Convert to environment action
        ml::Action env_action = ml::Action::continuous(
            {ap_action.throttle, ap_action.elevator,
             ap_action.aileron, ap_action.rudder},
            env_config.action_space);

        // Step environment
        ml::StepResult step_result;
        env->step(env_action, step_result);

        episode_reward += step_result.reward;

        if (step_result.is_done()) {
            break;
        }
    }
}

// Shutdown
autopilot->shutdown();
env->shutdown();
```

## Thread Safety

- **InferenceSession**: Not thread-safe. Synchronize access or use separate sessions per thread.
- **ModelRepository**: Thread-safe for read operations, synchronize write operations.
- **NeuralAutopilot**: Not thread-safe. Use separate instances per control thread.
- **RLEnvironment**: Not thread-safe. Synchronize step() calls across threads.

## Performance Considerations

### Inference Session
- **GPU acceleration**: Use CUDA or TensorRT for significant speedup (10-100x vs CPU)
- **Batch processing**: Larger batches improve GPU utilization
- **Warmup**: Call warmup() before timing/profiling to stabilize performance
- **Memory arena**: Enable for faster allocation in high-frequency loops
- **Provider selection**: Choose based on deployment target (CUDA for NVIDIA, CoreML for Apple, etc.)

### Model Repository
- **Cache strategy**: Aggressive cache for memory-limited systems, relaxed for servers
- **Checksum verification**: Disable in trusted environments to improve load speed
- **Preloading**: Enable for frequently-used models to reduce latency

### Neural Autopilot
- **Control frequency**: 50Hz for aircraft, 10Hz for ships, 100Hz+ for rotorcraft
- **Safety monitoring**: Minimal overhead, always recommended for safety-critical applications
- **Inference latency**: Must be < control period to avoid stale decisions

### RL Environment
- **Vectorization**: Run multiple parallel environments for faster training
- **Seed control**: Set seed(0) for random environments, fixed seed for reproducible evaluation
- **Rendering**: Disable in training for maximum speed

## Result Codes

### InferenceResult
- `Success` - Operation completed successfully
- `InvalidConfiguration` - Bad config parameter
- `InvalidModel` - Model structure/format error
- `InvalidInput/Output` - Tensor mismatch
- `RuntimeError` - ONNX Runtime error
- `ExecutionFailed` - Inference execution failed
- `TimeoutError` - Operation exceeded timeout
- `NotInitialized` - Session not initialized
- `ModelNotLoaded` - Model not loaded
- `OutOfMemory` - Insufficient memory
- `DeviceUnavailable` - GPU/device not available
- `ProviderNotSupported` - Provider not available on system

### ModelResult
- Similar to InferenceResult
- Plus: `ChecksumMismatch`, `SchemaViolation`, `IncompatibleVersion`

### AutopilotResult
- Similar result codes for autopilot operations

### RLResult
- Similar result codes for environment operations

## References

- ONNX Runtime: https://onnx.ai/
- OpenAI Gym: https://gym.openai.com/
- Neural Network Control: Goodfellow, Bengio, Courville - "Deep Learning"
- Reinforcement Learning: Sutton & Barto - "Reinforcement Learning: An Introduction"
