# Machine Learning API Reference

Machine Learning module (`jaguar_ml`) provides neural network-based autopilot systems, model repository management, and reinforcement learning environments with Gym-compatible APIs.

## Overview

The ML module enables:
- **Neural Autopilot**: NN-based flight control with safety constraints
- **Model Repository**: Versioned model storage with caching
- **RL Environment**: Gym-compatible training environments

---

## Neural Autopilot

### Header

```cpp
#include <jaguar/ml/neural_autopilot.h>
```

### Core Types

```cpp
namespace jaguar::ml {

// Autopilot operating modes
enum class AutopilotMode {
    Manual,             // No automation
    Waypoint,           // Follow waypoint route
    Altitude,           // Maintain altitude
    Speed,              // Maintain airspeed
    Course,             // Maintain heading
    AltitudeSpeed,      // Combined altitude + speed hold
    FullAuto,           // Complete autonomous control
    Landing,            // Automated landing
    Takeoff,            // Automated takeoff
    Formation,          // Formation flight
    Terrain,            // Terrain following
    Emergency           // Emergency procedures
};

// Vehicle types for autopilot specialization
enum class VehicleType {
    FixedWing,          // Conventional aircraft
    Rotorcraft,         // Helicopters, multirotors
    VTOL,               // Vertical takeoff/landing
    Ship,               // Surface vessels
    Submarine,          // Underwater vehicles
    GroundVehicle,      // Wheeled/tracked vehicles
    Spacecraft          // Orbital vehicles
};

// Autopilot observation (inputs to neural network)
struct AutopilotObservation {
    Vec3 position;              // ECEF or NED
    Vec3 velocity;              // Body or world frame
    Quaternion orientation;
    Vec3 angular_velocity;
    double altitude;
    double airspeed;
    double vertical_speed;
    double heading;
    double pitch;
    double roll;
    Vec3 target_position;
    double target_altitude;
    double target_speed;
    double target_heading;
    std::vector<Vec3> waypoints;
    double distance_to_target;
    double time_to_target;
    // Environment
    Vec3 wind_velocity;
    double air_density;
    double temperature;
};

// Autopilot action (outputs from neural network)
struct AutopilotAction {
    double throttle;            // 0.0 - 1.0
    double elevator;            // -1.0 to 1.0
    double aileron;             // -1.0 to 1.0
    double rudder;              // -1.0 to 1.0
    double flaps;               // 0.0 - 1.0
    double speed_brake;         // 0.0 - 1.0
    double collective;          // For rotorcraft: 0.0 - 1.0
    double cyclic_pitch;        // For rotorcraft: -1.0 to 1.0
    double cyclic_roll;         // For rotorcraft: -1.0 to 1.0
    double tail_rotor;          // For rotorcraft: -1.0 to 1.0
};

// Safety constraints
struct SafetyConstraints {
    double max_bank_angle = 60.0;       // degrees
    double max_pitch_angle = 30.0;      // degrees
    double max_g_load = 4.0;            // g
    double min_altitude = 100.0;        // meters AGL
    double max_altitude = 15000.0;      // meters
    double min_airspeed = 50.0;         // m/s
    double max_airspeed = 300.0;        // m/s
    double max_vertical_speed = 50.0;   // m/s
    double max_roll_rate = 60.0;        // deg/s
    double max_pitch_rate = 30.0;       // deg/s
    double max_yaw_rate = 30.0;         // deg/s
    bool enable_stall_protection = true;
    bool enable_overspeed_protection = true;
    bool enable_terrain_avoidance = true;
};

// Autopilot result
struct AutopilotResult {
    bool success;
    std::string error_message;
    AutopilotAction action;
    bool safety_intervention;
    std::string intervention_reason;
    double confidence;
    std::chrono::microseconds inference_time;
};

// Configuration
struct AutopilotConfig {
    VehicleType vehicle_type = VehicleType::FixedWing;
    AutopilotMode default_mode = AutopilotMode::Manual;
    std::string model_path;
    SafetyConstraints safety;
    double control_frequency = 50.0;    // Hz
    bool normalize_inputs = true;
    bool denormalize_outputs = true;
    bool enable_safety_monitor = true;
};

}  // namespace jaguar::ml
```

### NeuralAutopilot Class

```cpp
class NeuralAutopilot {
public:
    // Initialize with configuration
    AutopilotResult initialize(const AutopilotConfig& config);

    // Load neural network model
    AutopilotResult load_model(const std::string& model_path);

    // Shutdown
    void shutdown();

    // Set operating mode
    void set_mode(AutopilotMode mode);
    AutopilotMode get_mode() const;

    // Set target (depends on mode)
    void set_target_waypoint(const Vec3& waypoint);
    void set_target_altitude(double altitude_meters);
    void set_target_speed(double speed_mps);
    void set_target_heading(double heading_rad);
    void set_waypoint_route(const std::vector<Vec3>& waypoints);

    // Compute control action
    AutopilotResult compute_action(const AutopilotObservation& observation);

    // Compute with safety constraints
    AutopilotResult compute_safe_action(const AutopilotObservation& observation);

    // Update safety constraints
    void set_safety_constraints(const SafetyConstraints& constraints);
    SafetyConstraints get_safety_constraints() const;

    // Manual override
    void engage_manual_override();
    void disengage_manual_override();
    bool is_manual_override() const;

    // Get last action
    AutopilotAction get_last_action() const;

    // Get neural network info
    struct ModelInfo {
        std::string model_name;
        std::string model_version;
        size_t input_size;
        size_t output_size;
        size_t hidden_layers;
        size_t total_parameters;
        std::string activation_function;
    };
    ModelInfo get_model_info() const;

    // Statistics
    struct AutopilotStats {
        size_t total_inferences;
        size_t safety_interventions;
        std::chrono::microseconds avg_inference_time;
        std::chrono::microseconds max_inference_time;
        double avg_confidence;
    };
    AutopilotStats get_statistics() const;
};

// Factory functions
std::unique_ptr<NeuralAutopilot> create_neural_autopilot(
    const AutopilotConfig& config);
std::unique_ptr<NeuralAutopilot> create_aircraft_autopilot(
    const std::string& model_path);
std::unique_ptr<NeuralAutopilot> create_rotorcraft_autopilot(
    const std::string& model_path);
std::unique_ptr<NeuralAutopilot> create_ship_autopilot(
    const std::string& model_path);
```

---

## Model Repository

### Header

```cpp
#include <jaguar/ml/model_repository.h>
```

### Core Types

```cpp
namespace jaguar::ml {

// Model types
enum class ModelType {
    Autopilot,          // Flight control models
    Behavior,           // AI behavior models
    Perception,         // Sensor processing models
    Prediction,         // State prediction models
    Classification,     // Object classification
    Regression,         // General regression
    Reinforcement,      // RL policy models
    Anomaly,            // Anomaly detection
    Custom              // User-defined
};

// Model status
enum class ModelStatus {
    Draft,              // Under development
    Testing,            // In testing
    Staging,            // Staged for deployment
    Production,         // Active production
    Deprecated,         // No longer recommended
    Archived            // Archived, not for use
};

// Semantic version
struct SemanticVersion {
    uint32_t major;
    uint32_t minor;
    uint32_t patch;
    std::string prerelease;     // e.g., "alpha", "beta.1"
    std::string build_metadata;

    bool operator<(const SemanticVersion& other) const;
    bool operator==(const SemanticVersion& other) const;
    std::string to_string() const;
    static SemanticVersion parse(const std::string& version_str);
};

// Tensor info for model I/O
struct TensorInfo {
    std::string name;
    std::vector<int64_t> shape;
    std::string dtype;          // "float32", "float64", "int32", etc.
    bool is_optional = false;
};

// Model metadata
struct ModelMetadata {
    std::string model_id;
    std::string name;
    std::string description;
    ModelType type;
    SemanticVersion version;
    ModelStatus status;
    std::string author;
    std::chrono::system_clock::time_point created_at;
    std::chrono::system_clock::time_point updated_at;
    std::vector<TensorInfo> inputs;
    std::vector<TensorInfo> outputs;
    std::map<std::string, std::string> tags;
    std::map<std::string, double> metrics;  // accuracy, loss, etc.
    size_t file_size;
    std::string checksum;
    std::string framework;      // "onnx", "tensorflow", "pytorch"
    std::vector<std::string> dependencies;
};

// Model entry in repository
struct ModelEntry {
    ModelMetadata metadata;
    std::string storage_path;
    bool is_cached;
    std::chrono::system_clock::time_point last_accessed;
    size_t access_count;
};

// Model query
struct ModelQuery {
    std::optional<std::string> name_pattern;
    std::optional<ModelType> type;
    std::optional<ModelStatus> status;
    std::optional<SemanticVersion> min_version;
    std::optional<SemanticVersion> max_version;
    std::map<std::string, std::string> required_tags;
    size_t limit = 100;
    size_t offset = 0;
    bool latest_only = false;
};

// Cache configuration
struct CacheConfig {
    bool enabled = true;
    size_t max_cache_size = 1024 * 1024 * 1024;  // 1 GB
    size_t max_cached_models = 50;
    std::chrono::hours cache_ttl{24};
    bool preload_production = true;
};

// Model result
struct ModelResult {
    bool success;
    std::string error_message;
    std::string model_id;
    SemanticVersion version;
};

// Repository configuration
struct RepositoryConfig {
    std::string storage_path = "./models";
    std::string remote_url;
    CacheConfig cache;
    bool auto_sync = false;
    std::chrono::minutes sync_interval{60};
};

}  // namespace jaguar::ml
```

### ModelRepository Class

```cpp
class ModelRepository {
public:
    // Initialize repository
    ModelResult initialize(const RepositoryConfig& config);

    // Shutdown
    void shutdown();

    // Register a new model
    ModelResult register_model(const ModelMetadata& metadata,
                              const std::vector<uint8_t>& model_data);

    // Register from file
    ModelResult register_model_from_file(const ModelMetadata& metadata,
                                         const std::string& file_path);

    // Get model by ID
    std::optional<ModelEntry> get_model(const std::string& model_id) const;

    // Get model by name and version
    std::optional<ModelEntry> get_model(const std::string& name,
                                        const SemanticVersion& version) const;

    // Get latest version of model
    std::optional<ModelEntry> get_latest_model(const std::string& name) const;

    // Get latest production version
    std::optional<ModelEntry> get_production_model(const std::string& name) const;

    // Query models
    std::vector<ModelEntry> query_models(const ModelQuery& query) const;

    // List all versions of a model
    std::vector<ModelEntry> list_versions(const std::string& name) const;

    // Load model data
    std::vector<uint8_t> load_model_data(const std::string& model_id);

    // Load model to path
    ModelResult load_model_to_path(const std::string& model_id,
                                   const std::string& destination);

    // Update model metadata
    ModelResult update_metadata(const std::string& model_id,
                               const ModelMetadata& metadata);

    // Update model status
    ModelResult set_status(const std::string& model_id, ModelStatus status);

    // Promote model (draft -> testing -> staging -> production)
    ModelResult promote_model(const std::string& model_id);

    // Deprecate model
    ModelResult deprecate_model(const std::string& model_id);

    // Delete model
    ModelResult delete_model(const std::string& model_id);

    // Cache management
    void preload_to_cache(const std::string& model_id);
    void evict_from_cache(const std::string& model_id);
    void clear_cache();

    // Sync with remote repository
    ModelResult sync_with_remote();

    // Statistics
    struct RepositoryStats {
        size_t total_models;
        size_t production_models;
        size_t cached_models;
        size_t total_storage_bytes;
        size_t cache_size_bytes;
        double cache_hit_rate;
    };
    RepositoryStats get_statistics() const;
};

// Factory functions
std::unique_ptr<ModelRepository> create_model_repository(
    const RepositoryConfig& config);
std::unique_ptr<IModelRepository> create_memory_repository();
std::unique_ptr<IModelRepository> create_file_repository(
    const std::string& path);
```

---

## RL Environment

### Header

```cpp
#include <jaguar/ml/rl_environment.h>
```

### Core Types

```cpp
namespace jaguar::ml {

// Space types (Gym-compatible)
enum class SpaceType {
    Discrete,           // Single integer action
    Box,                // Continuous bounded space
    MultiDiscrete,      // Multiple discrete actions
    MultiBinary,        // Multiple binary actions
    Dict,               // Dictionary of spaces
    Tuple               // Tuple of spaces
};

// Space definition
struct Space {
    SpaceType type;
    std::vector<int64_t> shape;
    std::vector<double> low;    // Lower bounds (for Box)
    std::vector<double> high;   // Upper bounds (for Box)
    int64_t n = 0;              // Number of options (for Discrete)
    std::vector<int64_t> nvec;  // Sizes per dimension (for MultiDiscrete)

    // Factory methods
    static Space discrete(int64_t n);
    static Space box(const std::vector<int64_t>& shape,
                    double low, double high);
    static Space box(const std::vector<int64_t>& shape,
                    const std::vector<double>& low,
                    const std::vector<double>& high);
    static Space multi_discrete(const std::vector<int64_t>& nvec);
    static Space multi_binary(int64_t n);
};

// Observation
struct Observation {
    std::vector<double> data;
    std::map<std::string, std::vector<double>> dict_data;  // For Dict spaces

    double operator[](size_t i) const { return data[i]; }
    double& operator[](size_t i) { return data[i]; }
    size_t size() const { return data.size(); }
};

// Action
struct Action {
    std::vector<double> continuous;     // For Box spaces
    int64_t discrete = 0;               // For Discrete spaces
    std::vector<int64_t> multi;         // For MultiDiscrete/MultiBinary

    static Action from_discrete(int64_t action);
    static Action from_continuous(const std::vector<double>& action);
};

// Step result
struct StepResult {
    Observation observation;
    double reward;
    bool terminated;            // Episode ended (goal reached, failure)
    bool truncated;             // Episode truncated (time limit, etc.)
    std::map<std::string, double> info;
};

// Episode information
struct EpisodeInfo {
    uint64_t episode_id;
    size_t total_steps;
    double total_reward;
    std::chrono::milliseconds duration;
    bool successful;
    std::string termination_reason;
};

// Reward configuration
struct RewardConfig {
    double goal_reward = 100.0;
    double step_penalty = -0.1;
    double collision_penalty = -50.0;
    double out_of_bounds_penalty = -25.0;
    double fuel_penalty_per_kg = -0.01;
    double time_penalty_per_second = -0.1;
    bool shaped_rewards = true;         // Include intermediate rewards
    double discount_factor = 0.99;
};

// Environment configuration
struct EnvironmentConfig {
    std::string env_name;
    Space observation_space;
    Space action_space;
    RewardConfig rewards;
    size_t max_episode_steps = 1000;
    double dt = 0.02;                   // Simulation timestep
    uint32_t seed = 0;
    bool render_enabled = false;
    std::string render_mode = "human";  // "human", "rgb_array"
};

// RL result
struct RLResult {
    bool success;
    std::string error_message;
    size_t steps;
    double total_reward;
};

// Environment statistics
struct RLEnvironmentStats {
    size_t total_episodes;
    size_t total_steps;
    double avg_episode_reward;
    double avg_episode_length;
    double success_rate;
    std::chrono::microseconds avg_step_time;
};

}  // namespace jaguar::ml
```

### RLEnvironment Class

```cpp
class RLEnvironment {
public:
    // Initialize environment
    RLResult initialize(const EnvironmentConfig& config);

    // Shutdown
    void shutdown();

    // Gym-compatible interface
    Observation reset(uint32_t seed = 0);
    StepResult step(const Action& action);
    void render();
    void close();

    // Get spaces
    Space get_observation_space() const;
    Space get_action_space() const;

    // Sample random action
    Action sample_action() const;

    // Check if action is valid
    bool is_valid_action(const Action& action) const;

    // Episode control
    bool is_episode_done() const;
    EpisodeInfo get_episode_info() const;

    // Set scenario
    void set_initial_state(const Observation& state);
    void set_goal(const Vec3& goal_position);
    void set_obstacles(const std::vector<AABB>& obstacles);

    // Configuration
    void set_reward_config(const RewardConfig& config);
    void set_max_episode_steps(size_t steps);

    // Get underlying simulation engine
    interface::Engine& get_engine();

    // Statistics
    RLEnvironmentStats get_statistics() const;

    // Vectorized environment support
    std::vector<Observation> reset_batch(size_t num_envs, uint32_t seed = 0);
    std::vector<StepResult> step_batch(const std::vector<Action>& actions);
};

// Interface for custom environments
class IRLEnvironment {
public:
    virtual ~IRLEnvironment() = default;

    virtual Observation reset(uint32_t seed) = 0;
    virtual StepResult step(const Action& action) = 0;
    virtual Space get_observation_space() const = 0;
    virtual Space get_action_space() const = 0;
    virtual void render() = 0;
    virtual void close() = 0;
};

// Reward function interface
class IRewardFunction {
public:
    virtual ~IRewardFunction() = default;

    virtual double compute_reward(
        const Observation& observation,
        const Action& action,
        const Observation& next_observation,
        bool terminated,
        const std::map<std::string, double>& info) = 0;
};

// Termination condition interface
class ITerminationCondition {
public:
    virtual ~ITerminationCondition() = default;

    virtual bool check_terminated(
        const Observation& observation,
        size_t step,
        const std::map<std::string, double>& info) = 0;

    virtual bool check_truncated(
        const Observation& observation,
        size_t step,
        const std::map<std::string, double>& info) = 0;
};

// Factory functions
std::unique_ptr<RLEnvironment> create_rl_environment(
    const EnvironmentConfig& config);
std::unique_ptr<RLEnvironment> create_waypoint_environment(
    const Vec3& start, const Vec3& goal);
std::unique_ptr<RLEnvironment> create_landing_environment();
std::unique_ptr<RLEnvironment> create_formation_environment(
    size_t num_aircraft);

// Reward function factories
std::unique_ptr<IRewardFunction> create_distance_reward(
    const Vec3& goal, double scale);
std::unique_ptr<IRewardFunction> create_sparse_reward(
    double goal_reward, double step_penalty);
std::unique_ptr<IRewardFunction> create_composite_reward(
    std::vector<std::unique_ptr<IRewardFunction>> rewards,
    std::vector<double> weights);
```

---

## Usage Examples

### Neural Autopilot

```cpp
#include <jaguar/ml/neural_autopilot.h>

using namespace jaguar::ml;

// Configure autopilot
AutopilotConfig config;
config.vehicle_type = VehicleType::FixedWing;
config.model_path = "models/autopilot_v1.2.0.onnx";
config.default_mode = AutopilotMode::Waypoint;
config.control_frequency = 50.0;

// Safety constraints
config.safety.max_bank_angle = 45.0;
config.safety.max_g_load = 3.0;
config.safety.min_altitude = 500.0;
config.safety.enable_terrain_avoidance = true;

auto autopilot = create_neural_autopilot(config);
autopilot->initialize(config);

// Set waypoint route
std::vector<Vec3> waypoints = {
    {1000, 0, -5000},
    {5000, 2000, -6000},
    {8000, 1000, -5500}
};
autopilot->set_waypoint_route(waypoints);
autopilot->set_mode(AutopilotMode::Waypoint);

// Simulation loop
while (running) {
    // Get current aircraft state
    auto state = engine.get_entity_state(aircraft);
    auto env = engine.get_environment(aircraft);

    // Build observation
    AutopilotObservation obs;
    obs.position = state.position;
    obs.velocity = state.velocity;
    obs.orientation = state.orientation;
    obs.altitude = -state.position.z;
    obs.airspeed = magnitude(state.velocity);
    obs.waypoints = waypoints;
    obs.wind_velocity = env.wind;

    // Compute safe action
    auto result = autopilot->compute_safe_action(obs);

    if (result.success) {
        // Apply control action
        apply_controls(aircraft, result.action);

        if (result.safety_intervention) {
            std::cout << "Safety intervention: "
                      << result.intervention_reason << "\n";
        }
    }

    engine.step(dt);
}
```

### Model Repository Management

```cpp
#include <jaguar/ml/model_repository.h>

using namespace jaguar::ml;

// Configure repository
RepositoryConfig config;
config.storage_path = "./model_registry";
config.cache.max_cache_size = 2ULL * 1024 * 1024 * 1024;  // 2 GB
config.cache.preload_production = true;

auto repo = create_model_repository(config);
repo->initialize(config);

// Register a new model
ModelMetadata metadata;
metadata.name = "aircraft_autopilot";
metadata.description = "Neural autopilot for fixed-wing aircraft";
metadata.type = ModelType::Autopilot;
metadata.version = SemanticVersion{1, 2, 0};
metadata.status = ModelStatus::Testing;
metadata.author = "ML Team";
metadata.framework = "onnx";

metadata.inputs.push_back({"observation", {1, 24}, "float32"});
metadata.outputs.push_back({"action", {1, 6}, "float32"});

metadata.metrics["accuracy"] = 0.95;
metadata.metrics["avg_reward"] = 850.0;

metadata.tags["vehicle"] = "fixed_wing";
metadata.tags["domain"] = "air";

auto model_data = load_file("trained_model.onnx");
repo->register_model(metadata, model_data);

// Query models
ModelQuery query;
query.type = ModelType::Autopilot;
query.status = ModelStatus::Production;
query.required_tags["vehicle"] = "fixed_wing";

auto models = repo->query_models(query);
for (const auto& model : models) {
    std::cout << model.metadata.name << " v"
              << model.metadata.version.to_string() << "\n";
}

// Load and use model
auto entry = repo->get_production_model("aircraft_autopilot");
if (entry) {
    auto data = repo->load_model_data(entry->metadata.model_id);
    autopilot->load_model_from_memory(data);
}

// Promote model through lifecycle
repo->set_status(model_id, ModelStatus::Staging);
// After validation...
repo->promote_model(model_id);  // Now in Production
```

### RL Training Environment

```cpp
#include <jaguar/ml/rl_environment.h>

using namespace jaguar::ml;

// Configure environment
EnvironmentConfig config;
config.env_name = "WaypointNavigation-v1";
config.observation_space = Space::box({24}, -1.0, 1.0);
config.action_space = Space::box({6}, -1.0, 1.0);
config.max_episode_steps = 1000;
config.dt = 0.02;

config.rewards.goal_reward = 100.0;
config.rewards.step_penalty = -0.01;
config.rewards.collision_penalty = -50.0;
config.rewards.shaped_rewards = true;

auto env = create_rl_environment(config);
env->initialize(config);

// Set scenario
env->set_goal({10000, 0, -5000});  // 10 km away, 5 km altitude

// Training loop (pseudocode - typically done with Python/Ray)
for (int episode = 0; episode < 10000; ++episode) {
    auto obs = env->reset(episode);  // Seed with episode number
    double total_reward = 0.0;

    while (!env->is_episode_done()) {
        // Get action from policy (would typically be a neural network)
        Action action = policy.get_action(obs);

        // Step environment
        auto result = env->step(action);

        // Store transition for training
        replay_buffer.add(obs, action, result.reward,
                         result.observation, result.terminated);

        obs = result.observation;
        total_reward += result.reward;
    }

    auto info = env->get_episode_info();
    std::cout << "Episode " << episode
              << " reward: " << total_reward
              << " steps: " << info.total_steps
              << " success: " << (info.successful ? "yes" : "no") << "\n";
}

// Get statistics
auto stats = env->get_statistics();
std::cout << "Success rate: " << stats.success_rate * 100 << "%\n";
std::cout << "Avg reward: " << stats.avg_episode_reward << "\n";
```

### Custom Reward Function

```cpp
#include <jaguar/ml/rl_environment.h>

using namespace jaguar::ml;

// Custom reward function for formation flight
class FormationReward : public IRewardFunction {
    std::vector<Vec3> formation_offsets_;
    double formation_scale_ = 1.0;

public:
    FormationReward(const std::vector<Vec3>& offsets, double scale)
        : formation_offsets_(offsets), formation_scale_(scale) {}

    double compute_reward(
            const Observation& obs,
            const Action& action,
            const Observation& next_obs,
            bool terminated,
            const std::map<std::string, double>& info) override {

        double reward = 0.0;

        // Formation keeping reward
        double formation_error = info.at("formation_error");
        reward -= formation_error * formation_scale_;

        // Smooth control reward
        double control_effort = 0.0;
        for (double a : action.continuous) {
            control_effort += a * a;
        }
        reward -= 0.01 * control_effort;

        // Collision avoidance
        if (info.count("min_separation") && info.at("min_separation") < 50.0) {
            reward -= 10.0;
        }

        // Goal bonus
        if (terminated && info.count("goal_reached") && info.at("goal_reached") > 0) {
            reward += 100.0;
        }

        return reward;
    }
};

// Use custom reward
auto custom_reward = std::make_unique<FormationReward>(offsets, 0.5);
env->set_reward_function(std::move(custom_reward));
```

---

## See Also

- [Architecture](../advanced/architecture.md) - System architecture overview
- [Digital Thread API](thread.md) - Lifecycle tracking
- [GPU Compute API](gpu.md) - GPU acceleration for inference
- [Configuration](configuration.md) - Engine configuration
