#pragma once
/**
 * @file rl_environment.h
 * @brief OpenAI Gym-compatible reinforcement learning environment interface
 *
 * This file provides the RL environment infrastructure for training agents
 * in aerospace simulation scenarios. Supports standard Gym interfaces with
 * flexible observation/action spaces, customizable reward functions, and
 * termination conditions.
 *
 * Key features:
 * - Gym-compatible API (reset, step, render, close)
 * - Flexible space definitions (Discrete, Box, MultiDiscrete, etc.)
 * - Customizable reward functions and termination conditions
 * - Episode tracking with detailed statistics
 * - Seed control for reproducibility
 * - Support for continuous, discrete, and hybrid action spaces
 * - Built-in reward configurations for common scenarios
 */

#include "jaguar/core/types.h"
#include <vector>
#include <unordered_map>
#include <memory>
#include <optional>
#include <chrono>
#include <string>

namespace jaguar::ml {

// ============================================================================
// Forward Declarations
// ============================================================================

class IRewardFunction;
class ITerminationCondition;
class IRLEnvironment;
class RLEnvironment;

// ============================================================================
// RL Result Enum
// ============================================================================

/**
 * @brief Result codes for RL environment operations
 */
enum class RLResult : UInt8 {
    Success = 0,

    // Configuration errors
    InvalidConfiguration,
    InvalidAction,
    InvalidObservation,

    // Runtime errors
    SimulationError,
    RewardError,
    StepFailed,

    // State errors
    NotInitialized,
    AlreadyInitialized,
    EpisodeNotStarted,
    EpisodeDone,

    // Resource errors
    OutOfMemory
};

/**
 * @brief Convert RLResult to string
 */
inline const char* rl_result_to_string(RLResult result) {
    switch (result) {
        case RLResult::Success: return "Success";
        case RLResult::InvalidConfiguration: return "InvalidConfiguration";
        case RLResult::InvalidAction: return "InvalidAction";
        case RLResult::InvalidObservation: return "InvalidObservation";
        case RLResult::SimulationError: return "SimulationError";
        case RLResult::RewardError: return "RewardError";
        case RLResult::StepFailed: return "StepFailed";
        case RLResult::NotInitialized: return "NotInitialized";
        case RLResult::AlreadyInitialized: return "AlreadyInitialized";
        case RLResult::EpisodeNotStarted: return "EpisodeNotStarted";
        case RLResult::EpisodeDone: return "EpisodeDone";
        case RLResult::OutOfMemory: return "OutOfMemory";
        default: return "Unknown";
    }
}

// ============================================================================
// Space Type Enum
// ============================================================================

/**
 * @brief Types of observation/action spaces
 */
enum class SpaceType : UInt8 {
    /// Single discrete value from 0 to n-1
    Discrete = 0,

    /// Continuous bounded/unbounded n-dimensional space
    Box,

    /// Multiple discrete values with different ranges
    MultiDiscrete,

    /// Multiple binary values (0 or 1)
    MultiBinary,

    /// Dictionary of named spaces
    Dict,

    /// Tuple of spaces
    Tuple
};

/**
 * @brief Convert SpaceType to string
 */
inline const char* space_type_to_string(SpaceType type) {
    switch (type) {
        case SpaceType::Discrete: return "Discrete";
        case SpaceType::Box: return "Box";
        case SpaceType::MultiDiscrete: return "MultiDiscrete";
        case SpaceType::MultiBinary: return "MultiBinary";
        case SpaceType::Dict: return "Dict";
        case SpaceType::Tuple: return "Tuple";
        default: return "Unknown";
    }
}

// ============================================================================
// Bound Type Enum
// ============================================================================

/**
 * @brief Types of bounds for Box spaces
 */
enum class BoundType : UInt8 {
    /// No bounds (infinite range)
    Unbounded = 0,

    /// Both lower and upper bounds
    Bounded,

    /// Only one bound specified
    SemiBounded
};

// ============================================================================
// Data Type Enum
// ============================================================================

/**
 * @brief Data types for space values
 */
enum class DataType : UInt8 {
    Float32 = 0,
    Float64,
    Int32,
    Int64,
    UInt8,
    Bool
};

// ============================================================================
// Space Definition
// ============================================================================

/**
 * @brief Defines the structure and constraints of an observation/action space
 *
 * Supports multiple space types compatible with OpenAI Gym:
 * - Discrete: Single integer from 0 to n-1
 * - Box: Bounded/unbounded continuous space
 * - MultiDiscrete: Multiple discrete values
 * - MultiBinary: Binary vector
 * - Dict/Tuple: Composite spaces (for advanced use)
 */
struct Space {
    /// Type of this space
    SpaceType type{SpaceType::Box};

    /// Shape of the space (e.g., [4] for 4D vector)
    std::vector<Int64> shape;

    /// Data type for values in this space
    DataType dtype{DataType::Float32};

    /// Lower bounds (for Box spaces)
    std::optional<std::vector<Real>> low;

    /// Upper bounds (for Box spaces)
    std::optional<std::vector<Real>> high;

    /// Number of discrete values (for Discrete spaces)
    std::optional<Int64> n;

    /// Number of discrete values per dimension (for MultiDiscrete)
    std::optional<std::vector<Int64>> nvec;

    Space() = default;

    /**
     * @brief Check if a value is within this space
     * @param value Value to check
     * @return True if value is valid for this space
     */
    bool contains(const std::vector<Real>& value) const;

    /**
     * @brief Generate a random sample from this space
     * @return Random valid value
     */
    std::vector<Real> sample() const;

    // ========================================================================
    // Factory Methods
    // ========================================================================

    /**
     * @brief Create a discrete space with n values [0, n-1]
     * @param n Number of discrete values
     * @return Discrete space
     */
    static Space discrete(Int64 n) {
        Space space;
        space.type = SpaceType::Discrete;
        space.shape = {1};
        space.dtype = DataType::Int64;
        space.n = n;
        return space;
    }

    /**
     * @brief Create a continuous box space with bounds
     * @param low_bounds Lower bounds for each dimension
     * @param high_bounds Upper bounds for each dimension
     * @param shape_ Shape of the space
     * @param dtype_ Data type (default Float32)
     * @return Box space
     */
    static Space box(const std::vector<Real>& low_bounds,
                     const std::vector<Real>& high_bounds,
                     const std::vector<Int64>& shape_,
                     DataType dtype_ = DataType::Float32) {
        Space space;
        space.type = SpaceType::Box;
        space.shape = shape_;
        space.dtype = dtype_;
        space.low = low_bounds;
        space.high = high_bounds;
        return space;
    }

    /**
     * @brief Create a multi-discrete space with different ranges per dimension
     * @param nvec_ Number of values for each dimension
     * @return MultiDiscrete space
     */
    static Space multi_discrete(const std::vector<Int64>& nvec_) {
        Space space;
        space.type = SpaceType::MultiDiscrete;
        space.shape = {static_cast<Int64>(nvec_.size())};
        space.dtype = DataType::Int64;
        space.nvec = nvec_;
        return space;
    }

    /**
     * @brief Create a multi-binary space with n binary values
     * @param n Number of binary values
     * @return MultiBinary space
     */
    static Space multi_binary(Int64 n) {
        Space space;
        space.type = SpaceType::MultiBinary;
        space.shape = {n};
        space.dtype = DataType::UInt8;
        return space;
    }
};

// ============================================================================
// Observation
// ============================================================================

/**
 * @brief Observation returned from environment
 *
 * Contains the current state observation and metadata about the observation space.
 */
struct Observation {
    /// Primary observation data (for Box, Discrete, etc.)
    std::vector<Real> data;

    /// Space definition for this observation
    Space space;

    /// Dictionary data (for Dict space type)
    std::unordered_map<std::string, std::vector<Real>> dict_data;

    Observation() = default;

    explicit Observation(const Space& space_)
        : space(space_) {
        // Pre-allocate data based on space shape
        if (!space.shape.empty()) {
            Int64 size = 1;
            for (auto dim : space.shape) {
                size *= dim;
            }
            data.resize(static_cast<size_t>(size), 0.0);
        }
    }
};

// ============================================================================
// Action
// ============================================================================

/**
 * @brief Action to apply to environment
 *
 * Supports both continuous and discrete actions.
 */
struct Action {
    /// Continuous action data (for Box spaces)
    std::vector<Real> data;

    /// Space definition for this action
    Space space;

    /// Discrete action value (for Discrete spaces)
    Int64 discrete_value{0};

    Action() = default;

    explicit Action(const Space& space_)
        : space(space_) {
        if (space.type == SpaceType::Discrete) {
            discrete_value = 0;
        } else if (!space.shape.empty()) {
            Int64 size = 1;
            for (auto dim : space.shape) {
                size *= dim;
            }
            data.resize(static_cast<size_t>(size), 0.0);
        }
    }

    /**
     * @brief Create discrete action
     * @param value Discrete action value
     * @param space_ Action space
     * @return Action instance
     */
    static Action discrete(Int64 value, const Space& space_) {
        Action action(space_);
        action.discrete_value = value;
        return action;
    }

    /**
     * @brief Create continuous action
     * @param values Action values
     * @param space_ Action space
     * @return Action instance
     */
    static Action continuous(const std::vector<Real>& values, const Space& space_) {
        Action action(space_);
        action.data = values;
        return action;
    }
};

// ============================================================================
// Step Result
// ============================================================================

/**
 * @brief Result from a single environment step
 *
 * Compatible with Gym's step() return format:
 * observation, reward, terminated, truncated, info
 */
struct StepResult {
    /// Observation after taking action
    Observation observation;

    /// Reward received for this step
    Real reward{0.0};

    /// True if episode ended naturally (goal reached, failure, etc.)
    bool terminated{false};

    /// True if episode was cut short (time limit, external intervention)
    bool truncated{false};

    /// Additional information (for debugging, logging, etc.)
    std::unordered_map<std::string, std::string> info;

    StepResult() = default;

    /**
     * @brief Check if episode is done (either terminated or truncated)
     */
    bool is_done() const noexcept {
        return terminated || truncated;
    }
};

// ============================================================================
// Reward Configuration
// ============================================================================

/**
 * @brief Configuration for reward shaping
 *
 * Provides tunable parameters for common reward structures in aerospace tasks.
 */
struct RewardConfig {
    /// Reward for successfully completing objective
    Real success_reward{100.0};

    /// Penalty for failure (crash, bounds violation, etc.)
    Real failure_penalty{-100.0};

    /// Small penalty per time step (encourages efficiency)
    Real step_penalty{-0.1};

    /// Weight for distance-based reward shaping
    Real distance_weight{1.0};

    /// Weight for time-based reward shaping
    Real time_weight{0.1};

    /// Penalty for unsafe behaviors (too close to obstacles, high G-forces, etc.)
    Real safety_penalty{-10.0};

    /// Normalize rewards to [-1, 1] range
    bool normalize_rewards{true};

    // ========================================================================
    // Factory Methods
    // ========================================================================

    /**
     * @brief Default balanced reward configuration
     */
    static RewardConfig default_config() noexcept {
        return RewardConfig{};
    }

    /**
     * @brief Sparse reward (only on success/failure)
     */
    static RewardConfig sparse() noexcept {
        RewardConfig config;
        config.success_reward = 100.0;
        config.failure_penalty = -100.0;
        config.step_penalty = 0.0;
        config.distance_weight = 0.0;
        config.time_weight = 0.0;
        config.safety_penalty = -100.0;
        config.normalize_rewards = false;
        return config;
    }

    /**
     * @brief Dense reward with continuous feedback
     */
    static RewardConfig dense() noexcept {
        RewardConfig config;
        config.success_reward = 100.0;
        config.failure_penalty = -100.0;
        config.step_penalty = -0.01;
        config.distance_weight = 10.0;
        config.time_weight = 1.0;
        config.safety_penalty = -5.0;
        config.normalize_rewards = true;
        return config;
    }
};

// ============================================================================
// Environment Configuration
// ============================================================================

/**
 * @brief Configuration for RL environment
 */
struct EnvironmentConfig {
    /// Path to scenario file defining initial conditions
    std::string scenario_path;

    /// Maximum steps per episode
    UInt32 max_steps{1000};

    /// Simulation time step (seconds)
    Real time_step{0.02}; // 50Hz default

    /// Observation space definition
    Space observation_space;

    /// Action space definition
    Space action_space;

    /// Reward configuration
    RewardConfig reward_config;

    /// Enable rendering/visualization
    bool render_enabled{false};

    /// Random seed (0 = random)
    UInt32 seed{0};

    // ========================================================================
    // Factory Methods
    // ========================================================================

    /**
     * @brief Configuration for waypoint navigation task
     * @param scenario Scenario file path
     * @return Environment config for waypoint navigation
     */
    static EnvironmentConfig waypoint_navigation(const std::string& scenario = "") {
        EnvironmentConfig config;
        config.scenario_path = scenario;
        config.max_steps = 1000;
        config.time_step = 0.02;

        // Observation: [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z,
        //                goal_x, goal_y, goal_z, dist_to_goal]
        config.observation_space = Space::box(
            {-10000, -10000, -10000, -500, -500, -500, -10000, -10000, -10000, 0},
            {10000, 10000, 10000, 500, 500, 500, 10000, 10000, 10000, 20000},
            {10},
            DataType::Float32
        );

        // Action: [thrust, pitch_rate, yaw_rate, roll_rate]
        config.action_space = Space::box(
            {0, -1, -1, -1},
            {1, 1, 1, 1},
            {4},
            DataType::Float32
        );

        config.reward_config = RewardConfig::dense();
        return config;
    }

    /**
     * @brief Configuration for formation flight task
     * @param scenario Scenario file path
     * @return Environment config for formation flight
     */
    static EnvironmentConfig formation_flight(const std::string& scenario = "") {
        EnvironmentConfig config;
        config.scenario_path = scenario;
        config.max_steps = 2000;
        config.time_step = 0.02;

        // Observation: [own_pos (3), own_vel (3), own_attitude (4),
        //                leader_pos (3), leader_vel (3), relative_pos (3)]
        config.observation_space = Space::box(
            std::vector<Real>(19, -10000.0),
            std::vector<Real>(19, 10000.0),
            {19},
            DataType::Float32
        );

        // Action: continuous control
        config.action_space = Space::box(
            {-1, -1, -1, -1},
            {1, 1, 1, 1},
            {4},
            DataType::Float32
        );

        config.reward_config = RewardConfig::dense();
        return config;
    }

    /**
     * @brief Configuration for intercept task
     * @param scenario Scenario file path
     * @return Environment config for intercept
     */
    static EnvironmentConfig intercept(const std::string& scenario = "") {
        EnvironmentConfig config;
        config.scenario_path = scenario;
        config.max_steps = 500;
        config.time_step = 0.01; // Faster for interception

        // Observation: [own_state (10), target_state (10), relative_state (6)]
        config.observation_space = Space::box(
            std::vector<Real>(26, -10000.0),
            std::vector<Real>(26, 10000.0),
            {26},
            DataType::Float32
        );

        // Action: discrete maneuver selection
        config.action_space = Space::discrete(9); // 8 directions + no-op

        config.reward_config = RewardConfig::sparse();
        return config;
    }
};

// ============================================================================
// Episode Info
// ============================================================================

/**
 * @brief Information about a completed or ongoing episode
 */
struct EpisodeInfo {
    /// Episode number (increments each reset)
    UInt64 episode_number{0};

    /// Number of steps taken in this episode
    UInt32 step_count{0};

    /// Cumulative reward for this episode
    Real total_reward{0.0};

    /// Episode duration in seconds
    Real episode_length_seconds{0.0};

    /// True if episode ended successfully
    bool success{false};

    /// Reason for episode termination
    std::string termination_reason;

    /// Episode start time
    std::chrono::system_clock::time_point start_time;

    /// Episode end time (if completed)
    std::chrono::system_clock::time_point end_time;

    EpisodeInfo() = default;

    /**
     * @brief Get episode duration
     */
    std::chrono::seconds get_duration() const noexcept {
        if (step_count == 0) {
            return std::chrono::seconds(0);
        }
        auto end = (end_time.time_since_epoch().count() > 0) ? end_time : std::chrono::system_clock::now();
        return std::chrono::duration_cast<std::chrono::seconds>(end - start_time);
    }
};

// ============================================================================
// RL Environment Statistics
// ============================================================================

/**
 * @brief Statistics about RL environment performance
 */
struct RLEnvironmentStats {
    /// Total episodes completed
    UInt64 total_episodes{0};

    /// Total steps across all episodes
    UInt64 total_steps{0};

    /// Average reward per episode
    Real average_reward{0.0};

    /// Average episode length in steps
    Real average_episode_length{0.0};

    /// Number of successful episodes
    UInt64 successful_episodes{0};

    /// Success rate (successful_episodes / total_episodes)
    Real success_rate{0.0};

    /**
     * @brief Reset all statistics
     */
    void reset() noexcept {
        *this = RLEnvironmentStats{};
    }

    /**
     * @brief Update statistics with completed episode
     * @param episode Episode info to incorporate
     */
    void update(const EpisodeInfo& episode) noexcept {
        total_episodes++;
        total_steps += episode.step_count;

        // Incremental average update
        Real alpha = 1.0 / static_cast<Real>(total_episodes);
        average_reward = average_reward * (1.0 - alpha) + episode.total_reward * alpha;
        average_episode_length = average_episode_length * (1.0 - alpha) +
                                 static_cast<Real>(episode.step_count) * alpha;

        if (episode.success) {
            successful_episodes++;
        }
        success_rate = static_cast<Real>(successful_episodes) / static_cast<Real>(total_episodes);
    }
};

// ============================================================================
// Reward Function Interface
// ============================================================================

/**
 * @brief Interface for custom reward functions
 *
 * Implement this interface to define custom reward shaping for specific tasks.
 */
class IRewardFunction {
public:
    virtual ~IRewardFunction() = default;

    /**
     * @brief Compute reward for a transition
     * @param obs Current observation
     * @param action Action taken
     * @param next_obs Resulting observation
     * @param done True if episode ended
     * @return Reward value
     */
    virtual Real compute(const Observation& obs,
                        const Action& action,
                        const Observation& next_obs,
                        bool done) = 0;

    /**
     * @brief Reset reward function state (if stateful)
     */
    virtual void reset() = 0;
};

// ============================================================================
// Termination Condition Interface
// ============================================================================

/**
 * @brief Interface for custom termination conditions
 *
 * Implement this interface to define when episodes should end.
 */
class ITerminationCondition {
public:
    virtual ~ITerminationCondition() = default;

    /**
     * @brief Check if episode should terminate naturally
     * @param obs Current observation
     * @param step Current step number
     * @return True if episode should terminate
     */
    virtual bool is_terminated(const Observation& obs, UInt32 step) = 0;

    /**
     * @brief Check if episode should be truncated
     * @param obs Current observation
     * @param step Current step number
     * @return True if episode should be truncated
     */
    virtual bool is_truncated(const Observation& obs, UInt32 step) = 0;

    /**
     * @brief Get reason for termination/truncation
     * @return Termination reason string
     */
    virtual std::string get_reason() const = 0;
};

// ============================================================================
// RL Environment Interface
// ============================================================================

/**
 * @brief Core interface for RL environments (Gym-compatible)
 *
 * Follows OpenAI Gym API conventions for compatibility with standard RL libraries.
 */
class IRLEnvironment {
public:
    virtual ~IRLEnvironment() = default;

    /**
     * @brief Reset environment to initial state
     * @param initial_obs Output parameter for initial observation
     * @return Success or error code
     */
    virtual RLResult reset(Observation& initial_obs) = 0;

    /**
     * @brief Execute one step in the environment
     * @param action Action to take
     * @param result Output parameter for step result
     * @return Success or error code
     */
    virtual RLResult step(const Action& action, StepResult& result) = 0;

    /**
     * @brief Render current state (for visualization)
     * @return Success or error code
     */
    virtual RLResult render() = 0;

    /**
     * @brief Close environment and release resources
     * @return Success or error code
     */
    virtual RLResult close() = 0;

    /**
     * @brief Get observation space definition
     */
    virtual Space get_observation_space() const = 0;

    /**
     * @brief Get action space definition
     */
    virtual Space get_action_space() const = 0;

    /**
     * @brief Get current step count in episode
     */
    virtual UInt32 get_current_step() const = 0;

    /**
     * @brief Check if current episode is done
     */
    virtual bool is_episode_done() const = 0;
};

// ============================================================================
// RL Environment
// ============================================================================

/**
 * @brief Main RL environment implementation
 *
 * Provides a complete RL training environment for aerospace simulations with:
 * - Configurable observation and action spaces
 * - Custom reward functions and termination conditions
 * - Episode tracking and statistics
 * - Reproducible random seeding
 * - Gym-compatible API
 */
class RLEnvironment : public IRLEnvironment {
public:
    /**
     * @brief Construct RL environment with configuration
     * @param config Environment configuration
     */
    explicit RLEnvironment(const EnvironmentConfig& config = EnvironmentConfig{});

    ~RLEnvironment();

    // Non-copyable, movable
    RLEnvironment(const RLEnvironment&) = delete;
    RLEnvironment& operator=(const RLEnvironment&) = delete;
    RLEnvironment(RLEnvironment&&) noexcept;
    RLEnvironment& operator=(RLEnvironment&&) noexcept;

    // ========================================================================
    // Lifecycle
    // ========================================================================

    /**
     * @brief Initialize environment
     * @return Success or error code
     */
    RLResult initialize();

    /**
     * @brief Shutdown and cleanup
     * @return Success or error code
     */
    RLResult shutdown();

    /**
     * @brief Check if initialized
     */
    bool is_initialized() const noexcept;

    // ========================================================================
    // IRLEnvironment Interface
    // ========================================================================

    RLResult reset(Observation& initial_obs) override;
    RLResult step(const Action& action, StepResult& result) override;
    RLResult render() override;
    RLResult close() override;
    Space get_observation_space() const override;
    Space get_action_space() const override;
    UInt32 get_current_step() const override;
    bool is_episode_done() const override;

    // ========================================================================
    // Configuration
    // ========================================================================

    /**
     * @brief Set random seed for reproducibility
     * @param seed Random seed (0 = random)
     * @return Success or error code
     */
    RLResult set_seed(UInt32 seed);

    /**
     * @brief Set custom reward function
     * @param reward_fn Reward function implementation
     * @return Success or error code
     */
    RLResult set_reward_function(std::shared_ptr<IRewardFunction> reward_fn);

    /**
     * @brief Set custom termination condition
     * @param termination_cond Termination condition implementation
     * @return Success or error code
     */
    RLResult set_termination_condition(std::shared_ptr<ITerminationCondition> termination_cond);

    /**
     * @brief Get current configuration
     */
    const EnvironmentConfig& get_config() const noexcept;

    // ========================================================================
    // Episode Information
    // ========================================================================

    /**
     * @brief Get information about current/last episode
     */
    EpisodeInfo get_episode_info() const;

    /**
     * @brief Get environment statistics
     */
    RLEnvironmentStats get_stats() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// ============================================================================
// Factory Functions
// ============================================================================

/**
 * @brief Create RL environment with configuration
 * @param config Environment configuration
 * @return Unique pointer to RL environment
 */
std::unique_ptr<RLEnvironment> create_rl_environment(
    const EnvironmentConfig& config = EnvironmentConfig{});

/**
 * @brief Create environment for waypoint navigation task
 * @param scenario_path Path to scenario file
 * @return Unique pointer to configured environment
 */
std::unique_ptr<RLEnvironment> create_waypoint_environment(
    const std::string& scenario_path = "");

/**
 * @brief Create distance-based reward function
 * @param weight Weight for distance term
 * @return Unique pointer to reward function
 */
std::unique_ptr<IRewardFunction> create_distance_reward(Real weight = 1.0);

/**
 * @brief Create sparse reward function (only on success/failure)
 * @param success_value Reward for success
 * @param failure_value Reward for failure
 * @return Unique pointer to reward function
 */
std::unique_ptr<IRewardFunction> create_sparse_reward(
    Real success_value = 100.0,
    Real failure_value = -100.0);

/**
 * @brief Create goal-based termination condition
 * @param threshold Distance threshold for goal achievement
 * @return Unique pointer to termination condition
 */
std::unique_ptr<ITerminationCondition> create_goal_termination(Real threshold = 10.0);

/**
 * @brief Create time-based termination condition
 * @param max_steps Maximum steps before truncation
 * @return Unique pointer to termination condition
 */
std::unique_ptr<ITerminationCondition> create_time_termination(UInt32 max_steps = 1000);

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * @brief Calculate total size of a space
 * @param space Space to measure
 * @return Total number of elements
 */
inline Int64 get_space_size(const Space& space) {
    if (space.type == SpaceType::Discrete) {
        return space.n.value_or(1);
    }

    if (space.shape.empty()) {
        return 0;
    }

    Int64 size = 1;
    for (auto dim : space.shape) {
        size *= dim;
    }
    return size;
}

/**
 * @brief Check if space is continuous (Box type)
 * @param space Space to check
 * @return True if continuous
 */
inline bool is_continuous_space(const Space& space) {
    return space.type == SpaceType::Box;
}

/**
 * @brief Check if space is discrete
 * @param space Space to check
 * @return True if discrete
 */
inline bool is_discrete_space(const Space& space) {
    return space.type == SpaceType::Discrete ||
           space.type == SpaceType::MultiDiscrete ||
           space.type == SpaceType::MultiBinary;
}

/**
 * @brief Clip value to space bounds
 * @param value Value to clip
 * @param space Space defining bounds
 * @return Clipped value
 */
inline std::vector<Real> clip_to_space(const std::vector<Real>& value, const Space& space) {
    if (space.type != SpaceType::Box || !space.low || !space.high) {
        return value;
    }

    std::vector<Real> clipped = value;
    const auto& low = *space.low;
    const auto& high = *space.high;

    for (size_t i = 0; i < clipped.size() && i < low.size() && i < high.size(); ++i) {
        clipped[i] = std::max(low[i], std::min(high[i], clipped[i]));
    }

    return clipped;
}

/**
 * @brief Normalize value from space range to [0, 1]
 * @param value Value to normalize
 * @param space Space defining range
 * @return Normalized value
 */
inline std::vector<Real> normalize_from_space(const std::vector<Real>& value, const Space& space) {
    if (space.type != SpaceType::Box || !space.low || !space.high) {
        return value;
    }

    std::vector<Real> normalized = value;
    const auto& low = *space.low;
    const auto& high = *space.high;

    for (size_t i = 0; i < normalized.size() && i < low.size() && i < high.size(); ++i) {
        Real range = high[i] - low[i];
        if (range > 1e-10) {
            normalized[i] = (value[i] - low[i]) / range;
        }
    }

    return normalized;
}

/**
 * @brief Denormalize value from [0, 1] to space range
 * @param normalized Normalized value
 * @param space Space defining range
 * @return Denormalized value
 */
inline std::vector<Real> denormalize_to_space(const std::vector<Real>& normalized, const Space& space) {
    if (space.type != SpaceType::Box || !space.low || !space.high) {
        return normalized;
    }

    std::vector<Real> denormalized = normalized;
    const auto& low = *space.low;
    const auto& high = *space.high;

    for (size_t i = 0; i < denormalized.size() && i < low.size() && i < high.size(); ++i) {
        Real range = high[i] - low[i];
        denormalized[i] = normalized[i] * range + low[i];
    }

    return denormalized;
}

/**
 * @brief Format episode info as human-readable string
 * @param episode Episode info to format
 * @return Formatted string
 */
inline std::string format_episode_info(const EpisodeInfo& episode) {
    std::string result = "Episode " + std::to_string(episode.episode_number);
    result += " | Steps: " + std::to_string(episode.step_count);
    result += " | Reward: " + std::to_string(episode.total_reward);
    result += " | Duration: " + std::to_string(episode.episode_length_seconds) + "s";
    result += " | ";
    result += episode.success ? "SUCCESS" : "FAILED";
    if (!episode.termination_reason.empty()) {
        result += " (" + episode.termination_reason + ")";
    }
    return result;
}

/**
 * @brief Calculate reward statistics from episode history
 * @param episodes Vector of episode info
 * @return Tuple of (mean, std, min, max)
 */
inline std::tuple<Real, Real, Real, Real> calculate_reward_stats(
    const std::vector<EpisodeInfo>& episodes) {

    if (episodes.empty()) {
        return {0.0, 0.0, 0.0, 0.0};
    }

    Real mean = 0.0;
    Real min_reward = episodes[0].total_reward;
    Real max_reward = episodes[0].total_reward;

    for (const auto& ep : episodes) {
        mean += ep.total_reward;
        min_reward = std::min(min_reward, ep.total_reward);
        max_reward = std::max(max_reward, ep.total_reward);
    }
    mean /= static_cast<Real>(episodes.size());

    Real variance = 0.0;
    for (const auto& ep : episodes) {
        Real diff = ep.total_reward - mean;
        variance += diff * diff;
    }
    variance /= static_cast<Real>(episodes.size());
    Real std_dev = std::sqrt(variance);

    return {mean, std_dev, min_reward, max_reward};
}

} // namespace jaguar::ml
