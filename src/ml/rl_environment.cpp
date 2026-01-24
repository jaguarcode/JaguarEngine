/**
 * @file rl_environment.cpp
 * @brief Implementation of OpenAI Gym-compatible RL environment
 */

#include "jaguar/ml/rl_environment.h"
#include <algorithm>
#include <cmath>
#include <random>
#include <mutex>
#include <atomic>
#include <limits>

namespace jaguar::ml {

// ============================================================================
// Space Implementation
// ============================================================================

bool Space::contains(const std::vector<Real>& value) const {
    switch (type) {
        case SpaceType::Discrete: {
            if (value.size() != 1 || !n) return false;
            Int64 val = static_cast<Int64>(value[0]);
            return val >= 0 && val < *n;
        }

        case SpaceType::Box: {
            if (!low || !high) return true; // Unbounded
            if (value.size() != low->size()) return false;

            for (size_t i = 0; i < value.size(); ++i) {
                if (value[i] < (*low)[i] || value[i] > (*high)[i]) {
                    return false;
                }
            }
            return true;
        }

        case SpaceType::MultiDiscrete: {
            if (!nvec || value.size() != nvec->size()) return false;

            for (size_t i = 0; i < value.size(); ++i) {
                Int64 val = static_cast<Int64>(value[i]);
                if (val < 0 || val >= (*nvec)[i]) {
                    return false;
                }
            }
            return true;
        }

        case SpaceType::MultiBinary: {
            if (shape.empty() || value.size() != static_cast<size_t>(shape[0])) {
                return false;
            }

            for (auto val : value) {
                if (val != 0.0 && val != 1.0) {
                    return false;
                }
            }
            return true;
        }

        default:
            return false;
    }
}

std::vector<Real> Space::sample() const {
    static thread_local std::mt19937 gen(std::random_device{}());
    std::vector<Real> result;

    switch (type) {
        case SpaceType::Discrete: {
            if (!n) return {0.0};
            std::uniform_int_distribution<Int64> dist(0, *n - 1);
            result.push_back(static_cast<Real>(dist(gen)));
            break;
        }

        case SpaceType::Box: {
            if (!low || !high) {
                // Unbounded - sample from standard normal
                std::normal_distribution<Real> dist(0.0, 1.0);
                Int64 size = get_space_size(*this);
                for (Int64 i = 0; i < size; ++i) {
                    result.push_back(dist(gen));
                }
            } else {
                // Bounded - uniform sample
                for (size_t i = 0; i < low->size(); ++i) {
                    std::uniform_real_distribution<Real> dist((*low)[i], (*high)[i]);
                    result.push_back(dist(gen));
                }
            }
            break;
        }

        case SpaceType::MultiDiscrete: {
            if (!nvec) return {};
            for (auto n_val : *nvec) {
                std::uniform_int_distribution<Int64> dist(0, n_val - 1);
                result.push_back(static_cast<Real>(dist(gen)));
            }
            break;
        }

        case SpaceType::MultiBinary: {
            if (shape.empty()) return {};
            std::bernoulli_distribution dist(0.5);
            for (Int64 i = 0; i < shape[0]; ++i) {
                result.push_back(dist(gen) ? 1.0 : 0.0);
            }
            break;
        }

        default:
            break;
    }

    return result;
}

// ============================================================================
// DistanceRewardFunction
// ============================================================================

class DistanceRewardFunction : public IRewardFunction {
public:
    explicit DistanceRewardFunction(Real weight, Real goal_threshold = 10.0)
        : distance_weight_(weight)
        , goal_threshold_(goal_threshold)
        , previous_distance_(std::numeric_limits<Real>::infinity()) {
    }

    Real compute(const Observation& obs,
                [[maybe_unused]] const Action& action,
                const Observation& next_obs,
                [[maybe_unused]] bool done) override {
        (void)obs; // Previous observation not used in this reward function

        // Assume observation contains position and goal
        // For waypoint navigation: [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z,
        //                           goal_x, goal_y, goal_z, dist_to_goal]
        if (next_obs.data.size() < 10) {
            return 0.0;
        }

        // Extract distance to goal (last element)
        Real current_distance = next_obs.data[9];

        // Compute distance-based reward (negative distance)
        Real distance_reward = -distance_weight_ * current_distance;

        // Bonus for reducing distance
        if (!std::isinf(previous_distance_)) {
            Real distance_delta = previous_distance_ - current_distance;
            distance_reward += distance_delta * 0.1; // Small bonus for progress
        }

        previous_distance_ = current_distance;

        // Large bonus for reaching goal
        if (current_distance < goal_threshold_) {
            distance_reward += 100.0;
        }

        return distance_reward;
    }

    void reset() override {
        previous_distance_ = std::numeric_limits<Real>::infinity();
    }

private:
    Real distance_weight_;
    Real goal_threshold_;
    Real previous_distance_;
};

// ============================================================================
// SparseRewardFunction
// ============================================================================

class SparseRewardFunction : public IRewardFunction {
public:
    SparseRewardFunction(Real success_value, Real failure_value, Real goal_threshold = 10.0)
        : success_reward_(success_value)
        , failure_penalty_(failure_value)
        , goal_threshold_(goal_threshold) {
    }

    Real compute([[maybe_unused]] const Observation& obs,
                [[maybe_unused]] const Action& action,
                const Observation& next_obs,
                bool done) override {

        if (!done) {
            return 0.0; // No reward during episode
        }

        // Check if goal was reached
        if (next_obs.data.size() >= 10) {
            Real distance = next_obs.data[9];
            if (distance < goal_threshold_) {
                return success_reward_;
            }
        }

        return failure_penalty_;
    }

    void reset() override {
        // Stateless
    }

private:
    Real success_reward_;
    Real failure_penalty_;
    Real goal_threshold_;
};

// ============================================================================
// GoalTerminationCondition
// ============================================================================

class GoalTerminationCondition : public ITerminationCondition {
public:
    explicit GoalTerminationCondition(Real threshold)
        : threshold_(threshold)
        , reason_("") {
    }

    bool is_terminated(const Observation& obs, [[maybe_unused]] UInt32 step) override {
        // Check if goal is reached
        if (obs.data.size() >= 10) {
            Real distance = obs.data[9];
            if (distance < threshold_) {
                reason_ = "Goal reached";
                return true;
            }
        }

        return false;
    }

    bool is_truncated([[maybe_unused]] const Observation& obs, [[maybe_unused]] UInt32 step) override {
        return false; // This condition only handles goal termination
    }

    std::string get_reason() const override {
        return reason_;
    }

private:
    Real threshold_;
    mutable std::string reason_;
};

// ============================================================================
// TimeTerminationCondition
// ============================================================================

class TimeTerminationCondition : public ITerminationCondition {
public:
    explicit TimeTerminationCondition(UInt32 max_steps)
        : max_steps_(max_steps)
        , reason_("") {
    }

    bool is_terminated([[maybe_unused]] const Observation& obs, [[maybe_unused]] UInt32 step) override {
        return false; // Time limit is truncation, not termination
    }

    bool is_truncated([[maybe_unused]] const Observation& obs, UInt32 step) override {
        if (step >= max_steps_) {
            reason_ = "Time limit reached";
            return true;
        }
        return false;
    }

    std::string get_reason() const override {
        return reason_;
    }

private:
    UInt32 max_steps_;
    mutable std::string reason_;
};

// ============================================================================
// CombinedTerminationCondition
// ============================================================================

class CombinedTerminationCondition : public ITerminationCondition {
public:
    CombinedTerminationCondition(
        std::shared_ptr<ITerminationCondition> goal_cond,
        std::shared_ptr<ITerminationCondition> time_cond)
        : goal_condition_(std::move(goal_cond))
        , time_condition_(std::move(time_cond))
        , reason_("") {
    }

    bool is_terminated(const Observation& obs, UInt32 step) override {
        if (goal_condition_ && goal_condition_->is_terminated(obs, step)) {
            reason_ = goal_condition_->get_reason();
            return true;
        }
        return false;
    }

    bool is_truncated(const Observation& obs, UInt32 step) override {
        if (time_condition_ && time_condition_->is_truncated(obs, step)) {
            reason_ = time_condition_->get_reason();
            return true;
        }
        return false;
    }

    std::string get_reason() const override {
        return reason_;
    }

private:
    std::shared_ptr<ITerminationCondition> goal_condition_;
    std::shared_ptr<ITerminationCondition> time_condition_;
    mutable std::string reason_;
};

// ============================================================================
// RLEnvironment::Impl
// ============================================================================

struct RLEnvironment::Impl {
    // Configuration
    EnvironmentConfig config;

    // Custom components
    std::shared_ptr<IRewardFunction> reward_fn;
    std::shared_ptr<ITerminationCondition> termination;

    // Episode state
    Observation current_obs;
    UInt32 current_step{0};
    UInt64 episode_count{0};
    bool episode_done{false};
    EpisodeInfo current_episode;

    // State tracking
    std::atomic<bool> initialized{false};
    std::mutex mutex;

    // Statistics
    RLEnvironmentStats stats;

    // Random number generation
    std::mt19937 rng;
    UInt32 seed{0};

    // Simulation state (for stub physics)
    std::vector<Real> position{0.0, 0.0, 0.0};
    std::vector<Real> velocity{0.0, 0.0, 0.0};
    std::vector<Real> goal{1000.0, 1000.0, 1000.0};

    Impl() {
        // Seed with random device
        std::random_device rd;
        rng.seed(rd());
    }

    Real compute_distance_to_goal() const {
        Real dx = goal[0] - position[0];
        Real dy = goal[1] - position[1];
        Real dz = goal[2] - position[2];
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }

    void update_observation() {
        // Update observation with current state
        // [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, goal_x, goal_y, goal_z, dist_to_goal]
        if (current_obs.data.size() >= 10) {
            current_obs.data[0] = position[0];
            current_obs.data[1] = position[1];
            current_obs.data[2] = position[2];
            current_obs.data[3] = velocity[0];
            current_obs.data[4] = velocity[1];
            current_obs.data[5] = velocity[2];
            current_obs.data[6] = goal[0];
            current_obs.data[7] = goal[1];
            current_obs.data[8] = goal[2];
            current_obs.data[9] = compute_distance_to_goal();
        }
    }

    void reset_simulation() {
        // Reset to initial state with some randomness
        std::uniform_real_distribution<Real> pos_dist(-100.0, 100.0);
        std::uniform_real_distribution<Real> goal_dist(800.0, 1200.0);

        position[0] = pos_dist(rng);
        position[1] = pos_dist(rng);
        position[2] = pos_dist(rng);

        velocity[0] = 0.0;
        velocity[1] = 0.0;
        velocity[2] = 0.0;

        goal[0] = goal_dist(rng);
        goal[1] = goal_dist(rng);
        goal[2] = goal_dist(rng);

        update_observation();
    }

    void simulate_step(const Action& action) {
        // Simple physics simulation for waypoint navigation
        // Action: [thrust, pitch_rate, yaw_rate, roll_rate]

        if (action.space.type == SpaceType::Box && action.data.size() >= 4) {
            // Extract action components
            Real thrust = action.data[0];
            Real pitch_rate = action.data[1];
            Real yaw_rate = action.data[2];
            // roll_rate = action.data[3]; (not used in simple model)

            // Compute direction to goal
            Real dx = goal[0] - position[0];
            Real dy = goal[1] - position[1];
            Real dz = goal[2] - position[2];
            Real dist = compute_distance_to_goal();

            if (dist > 1e-6) {
                // Normalize direction
                dx /= dist;
                dy /= dist;
                dz /= dist;

                // Apply thrust in direction influenced by control inputs
                Real control_scale = 50.0;
                velocity[0] += thrust * dx * config.time_step * control_scale + pitch_rate * 10.0 * config.time_step;
                velocity[1] += thrust * dy * config.time_step * control_scale + yaw_rate * 10.0 * config.time_step;
                velocity[2] += thrust * dz * config.time_step * control_scale;
            }

            // Apply drag
            Real drag = 0.95;
            velocity[0] *= drag;
            velocity[1] *= drag;
            velocity[2] *= drag;

            // Clamp velocity
            Real max_vel = 100.0;
            for (auto& v : velocity) {
                v = std::max(-max_vel, std::min(max_vel, v));
            }

        } else if (action.space.type == SpaceType::Discrete) {
            // Discrete action space: 8 directions + no-op
            Real move_speed = 50.0;

            switch (action.discrete_value) {
                case 0: break; // No-op
                case 1: velocity[0] += move_speed; break; // +X
                case 2: velocity[0] -= move_speed; break; // -X
                case 3: velocity[1] += move_speed; break; // +Y
                case 4: velocity[1] -= move_speed; break; // -Y
                case 5: velocity[2] += move_speed; break; // +Z
                case 6: velocity[2] -= move_speed; break; // -Z
                case 7: // Move toward goal
                {
                    Real dx = goal[0] - position[0];
                    Real dy = goal[1] - position[1];
                    Real dz = goal[2] - position[2];
                    Real dist = compute_distance_to_goal();
                    if (dist > 1e-6) {
                        velocity[0] += move_speed * dx / dist;
                        velocity[1] += move_speed * dy / dist;
                        velocity[2] += move_speed * dz / dist;
                    }
                    break;
                }
                case 8: // Brake
                    velocity[0] *= 0.5;
                    velocity[1] *= 0.5;
                    velocity[2] *= 0.5;
                    break;
                default:
                    break;
            }
        }

        // Update position
        position[0] += velocity[0] * config.time_step;
        position[1] += velocity[1] * config.time_step;
        position[2] += velocity[2] * config.time_step;

        // Update observation
        update_observation();
    }
};

// ============================================================================
// RLEnvironment Implementation
// ============================================================================

RLEnvironment::RLEnvironment(const EnvironmentConfig& config)
    : impl_(std::make_unique<Impl>()) {
    impl_->config = config;
    impl_->current_obs = Observation(config.observation_space);
}

RLEnvironment::~RLEnvironment() {
    if (impl_ && impl_->initialized.load()) {
        shutdown();
    }
}

RLEnvironment::RLEnvironment(RLEnvironment&&) noexcept = default;
RLEnvironment& RLEnvironment::operator=(RLEnvironment&&) noexcept = default;

RLResult RLEnvironment::initialize() {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (impl_->initialized.load()) {
        return RLResult::AlreadyInitialized;
    }

    // Initialize random number generator
    if (impl_->config.seed != 0) {
        impl_->rng.seed(impl_->config.seed);
        impl_->seed = impl_->config.seed;
    }

    // Create default reward function if none set
    if (!impl_->reward_fn) {
        if (impl_->config.reward_config.distance_weight > 0.0) {
            impl_->reward_fn = std::make_shared<DistanceRewardFunction>(
                impl_->config.reward_config.distance_weight
            );
        } else {
            impl_->reward_fn = std::make_shared<SparseRewardFunction>(
                impl_->config.reward_config.success_reward,
                impl_->config.reward_config.failure_penalty
            );
        }
    }

    // Create default termination condition if none set
    if (!impl_->termination) {
        auto goal_term = std::make_shared<GoalTerminationCondition>(10.0);
        auto time_term = std::make_shared<TimeTerminationCondition>(impl_->config.max_steps);
        impl_->termination = std::make_shared<CombinedTerminationCondition>(
            goal_term, time_term
        );
    }

    // Initialize observation
    impl_->current_obs = Observation(impl_->config.observation_space);

    impl_->initialized.store(true);
    return RLResult::Success;
}

RLResult RLEnvironment::shutdown() {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->initialized.load()) {
        return RLResult::NotInitialized;
    }

    // Clean up resources
    impl_->reward_fn.reset();
    impl_->termination.reset();
    impl_->stats.reset();

    impl_->initialized.store(false);
    return RLResult::Success;
}

bool RLEnvironment::is_initialized() const noexcept {
    return impl_->initialized.load();
}

RLResult RLEnvironment::reset(Observation& initial_obs) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->initialized.load()) {
        return RLResult::NotInitialized;
    }

    // Increment episode count
    impl_->episode_count++;

    // Reset episode state
    impl_->current_step = 0;
    impl_->episode_done = false;

    // Reset episode info
    impl_->current_episode = EpisodeInfo{};
    impl_->current_episode.episode_number = impl_->episode_count;
    impl_->current_episode.start_time = std::chrono::system_clock::now();

    // Reset reward function
    if (impl_->reward_fn) {
        impl_->reward_fn->reset();
    }

    // Reset simulation state
    impl_->reset_simulation();

    // Return initial observation
    initial_obs = impl_->current_obs;

    return RLResult::Success;
}

RLResult RLEnvironment::step(const Action& action, StepResult& result) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->initialized.load()) {
        return RLResult::NotInitialized;
    }

    if (impl_->episode_done) {
        return RLResult::EpisodeDone;
    }

    if (impl_->current_episode.episode_number == 0) {
        return RLResult::EpisodeNotStarted;
    }

    // Validate action
    if (!impl_->config.action_space.contains(
            action.space.type == SpaceType::Discrete ?
            std::vector<Real>{static_cast<Real>(action.discrete_value)} :
            action.data)) {
        return RLResult::InvalidAction;
    }

    // Store previous observation
    Observation prev_obs = impl_->current_obs;

    // Simulate step
    impl_->simulate_step(action);
    impl_->current_step++;

    // Compute reward
    Real reward = 0.0;
    if (impl_->reward_fn) {
        reward = impl_->reward_fn->compute(
            prev_obs,
            action,
            impl_->current_obs,
            false // Will be updated after termination check
        );

        // Apply step penalty
        reward += impl_->config.reward_config.step_penalty;
    }

    // Check termination conditions
    bool terminated = false;
    bool truncated = false;
    std::string termination_reason;

    if (impl_->termination) {
        terminated = impl_->termination->is_terminated(impl_->current_obs, impl_->current_step);
        truncated = impl_->termination->is_truncated(impl_->current_obs, impl_->current_step);

        if (terminated || truncated) {
            termination_reason = impl_->termination->get_reason();

            // Recompute reward with done flag
            if (impl_->reward_fn) {
                reward = impl_->reward_fn->compute(
                    prev_obs,
                    action,
                    impl_->current_obs,
                    true
                );
                reward += impl_->config.reward_config.step_penalty;
            }
        }
    }

    impl_->episode_done = terminated || truncated;

    // Update episode info
    impl_->current_episode.step_count = impl_->current_step;
    impl_->current_episode.total_reward += reward;
    impl_->current_episode.episode_length_seconds =
        static_cast<Real>(impl_->current_step) * impl_->config.time_step;

    if (impl_->episode_done) {
        impl_->current_episode.success = terminated; // Natural termination = success
        impl_->current_episode.termination_reason = termination_reason;
        impl_->current_episode.end_time = std::chrono::system_clock::now();

        // Update stats
        impl_->stats.update(impl_->current_episode);
    }

    // Populate result
    result.observation = impl_->current_obs;
    result.reward = reward;
    result.terminated = terminated;
    result.truncated = truncated;
    result.info["step"] = std::to_string(impl_->current_step);
    result.info["episode"] = std::to_string(impl_->episode_count);
    if (!termination_reason.empty()) {
        result.info["termination_reason"] = termination_reason;
    }

    return RLResult::Success;
}

RLResult RLEnvironment::render() {
    // Stub implementation - would connect to visualization system
    return RLResult::Success;
}

RLResult RLEnvironment::close() {
    return shutdown();
}

Space RLEnvironment::get_observation_space() const {
    return impl_->config.observation_space;
}

Space RLEnvironment::get_action_space() const {
    return impl_->config.action_space;
}

UInt32 RLEnvironment::get_current_step() const {
    return impl_->current_step;
}

bool RLEnvironment::is_episode_done() const {
    return impl_->episode_done;
}

RLResult RLEnvironment::set_seed(UInt32 seed) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    impl_->seed = seed;
    if (seed != 0) {
        impl_->rng.seed(seed);
    } else {
        std::random_device rd;
        impl_->rng.seed(rd());
    }

    return RLResult::Success;
}

RLResult RLEnvironment::set_reward_function(std::shared_ptr<IRewardFunction> reward_fn) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!reward_fn) {
        return RLResult::InvalidConfiguration;
    }

    impl_->reward_fn = std::move(reward_fn);
    return RLResult::Success;
}

RLResult RLEnvironment::set_termination_condition(
    std::shared_ptr<ITerminationCondition> termination_cond) {

    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!termination_cond) {
        return RLResult::InvalidConfiguration;
    }

    impl_->termination = std::move(termination_cond);
    return RLResult::Success;
}

const EnvironmentConfig& RLEnvironment::get_config() const noexcept {
    return impl_->config;
}

EpisodeInfo RLEnvironment::get_episode_info() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->current_episode;
}

RLEnvironmentStats RLEnvironment::get_stats() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->stats;
}

// ============================================================================
// Factory Functions
// ============================================================================

std::unique_ptr<RLEnvironment> create_rl_environment(const EnvironmentConfig& config) {
    return std::make_unique<RLEnvironment>(config);
}

std::unique_ptr<RLEnvironment> create_waypoint_environment(const std::string& scenario_path) {
    auto config = EnvironmentConfig::waypoint_navigation(scenario_path);
    auto env = std::make_unique<RLEnvironment>(config);

    // Initialize with default components
    env->initialize();

    return env;
}

std::unique_ptr<IRewardFunction> create_distance_reward(Real weight) {
    return std::make_unique<DistanceRewardFunction>(weight);
}

std::unique_ptr<IRewardFunction> create_sparse_reward(
    Real success_value,
    Real failure_value) {

    return std::make_unique<SparseRewardFunction>(success_value, failure_value);
}

std::unique_ptr<ITerminationCondition> create_goal_termination(Real threshold) {
    return std::make_unique<GoalTerminationCondition>(threshold);
}

std::unique_ptr<ITerminationCondition> create_time_termination(UInt32 max_steps) {
    return std::make_unique<TimeTerminationCondition>(max_steps);
}

} // namespace jaguar::ml
