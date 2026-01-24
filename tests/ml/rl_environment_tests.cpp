/**
 * @file rl_environment_tests.cpp
 * @brief Comprehensive unit tests for RL Environment module
 */

#include <gtest/gtest.h>
#include "jaguar/ml/rl_environment.h"
#include <cmath>
#include <algorithm>
#include <memory>

using namespace jaguar;
using namespace jaguar::ml;

// ============================================================================
// Helper Functions
// ============================================================================

namespace {

bool nearly_equal(Real a, Real b, Real eps = 1e-6) {
    return std::abs(a - b) < eps;
}

bool vector_nearly_equal(const std::vector<Real>& a, const std::vector<Real>& b, Real eps = 1e-6) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i) {
        if (!nearly_equal(a[i], b[i], eps)) return false;
    }
    return true;
}

} // anonymous namespace

// ============================================================================
// RLResult Tests
// ============================================================================

TEST(RLResultTest, EnumValues) {
    EXPECT_EQ(static_cast<UInt8>(RLResult::Success), 0);
    EXPECT_NE(RLResult::InvalidConfiguration, RLResult::Success);
    EXPECT_NE(RLResult::StepFailed, RLResult::Success);
    EXPECT_NE(RLResult::EpisodeDone, RLResult::Success);
}

TEST(RLResultTest, ToString) {
    EXPECT_STREQ(rl_result_to_string(RLResult::Success), "Success");
    EXPECT_STREQ(rl_result_to_string(RLResult::InvalidConfiguration), "InvalidConfiguration");
    EXPECT_STREQ(rl_result_to_string(RLResult::InvalidAction), "InvalidAction");
    EXPECT_STREQ(rl_result_to_string(RLResult::NotInitialized), "NotInitialized");
    EXPECT_STREQ(rl_result_to_string(RLResult::EpisodeDone), "EpisodeDone");
}

// ============================================================================
// SpaceType Tests
// ============================================================================

TEST(SpaceTypeTest, EnumValues) {
    EXPECT_EQ(static_cast<UInt8>(SpaceType::Discrete), 0);
    EXPECT_NE(SpaceType::Box, SpaceType::Discrete);
    EXPECT_NE(SpaceType::MultiDiscrete, SpaceType::MultiBinary);
}

TEST(SpaceTypeTest, ToString) {
    EXPECT_STREQ(space_type_to_string(SpaceType::Discrete), "Discrete");
    EXPECT_STREQ(space_type_to_string(SpaceType::Box), "Box");
    EXPECT_STREQ(space_type_to_string(SpaceType::MultiDiscrete), "MultiDiscrete");
    EXPECT_STREQ(space_type_to_string(SpaceType::MultiBinary), "MultiBinary");
    EXPECT_STREQ(space_type_to_string(SpaceType::Dict), "Dict");
    EXPECT_STREQ(space_type_to_string(SpaceType::Tuple), "Tuple");
}

// ============================================================================
// Space Factory Tests
// ============================================================================

TEST(SpaceFactoryTest, DiscreteSpace) {
    auto space = Space::discrete(5);

    EXPECT_EQ(space.type, SpaceType::Discrete);
    EXPECT_EQ(space.shape.size(), 1);
    EXPECT_EQ(space.shape[0], 1);
    EXPECT_TRUE(space.n.has_value());
    EXPECT_EQ(*space.n, 5);
    EXPECT_EQ(space.dtype, DataType::Int64);
}

TEST(SpaceFactoryTest, BoxSpace) {
    auto space = Space::box(
        {-1.0, -2.0, -3.0},
        {1.0, 2.0, 3.0},
        {3},
        DataType::Float32
    );

    EXPECT_EQ(space.type, SpaceType::Box);
    EXPECT_EQ(space.shape.size(), 1);
    EXPECT_EQ(space.shape[0], 3);
    EXPECT_TRUE(space.low.has_value());
    EXPECT_TRUE(space.high.has_value());
    EXPECT_EQ(space.low->size(), 3);
    EXPECT_EQ(space.high->size(), 3);
    EXPECT_DOUBLE_EQ((*space.low)[0], -1.0);
    EXPECT_DOUBLE_EQ((*space.high)[2], 3.0);
    EXPECT_EQ(space.dtype, DataType::Float32);
}

TEST(SpaceFactoryTest, MultiDiscreteSpace) {
    auto space = Space::multi_discrete({3, 5, 7});

    EXPECT_EQ(space.type, SpaceType::MultiDiscrete);
    EXPECT_EQ(space.shape.size(), 1);
    EXPECT_EQ(space.shape[0], 3);
    EXPECT_TRUE(space.nvec.has_value());
    EXPECT_EQ(space.nvec->size(), 3);
    EXPECT_EQ((*space.nvec)[0], 3);
    EXPECT_EQ((*space.nvec)[1], 5);
    EXPECT_EQ((*space.nvec)[2], 7);
    EXPECT_EQ(space.dtype, DataType::Int64);
}

TEST(SpaceFactoryTest, MultiBinarySpace) {
    auto space = Space::multi_binary(8);

    EXPECT_EQ(space.type, SpaceType::MultiBinary);
    EXPECT_EQ(space.shape.size(), 1);
    EXPECT_EQ(space.shape[0], 8);
    EXPECT_EQ(space.dtype, DataType::UInt8);
}

// ============================================================================
// Space Contains Tests
// ============================================================================

TEST(SpaceContainsTest, DiscreteValidValue) {
    auto space = Space::discrete(5);

    EXPECT_TRUE(space.contains({0.0}));
    EXPECT_TRUE(space.contains({2.0}));
    EXPECT_TRUE(space.contains({4.0}));
    EXPECT_FALSE(space.contains({5.0}));
    EXPECT_FALSE(space.contains({-1.0}));
}

TEST(SpaceContainsTest, DiscreteInvalidSize) {
    auto space = Space::discrete(5);

    EXPECT_FALSE(space.contains({}));
    EXPECT_FALSE(space.contains({0.0, 1.0}));
}

TEST(SpaceContainsTest, BoxWithinBounds) {
    auto space = Space::box({-1.0, -1.0}, {1.0, 1.0}, {2});

    EXPECT_TRUE(space.contains({0.0, 0.0}));
    EXPECT_TRUE(space.contains({-1.0, 1.0}));
    EXPECT_TRUE(space.contains({0.5, -0.5}));
}

TEST(SpaceContainsTest, BoxOutOfBounds) {
    auto space = Space::box({-1.0, -1.0}, {1.0, 1.0}, {2});

    EXPECT_FALSE(space.contains({-1.1, 0.0}));
    EXPECT_FALSE(space.contains({0.0, 1.1}));
    EXPECT_FALSE(space.contains({2.0, 2.0}));
}

TEST(SpaceContainsTest, BoxUnbounded) {
    auto space = Space::box({}, {}, {2});
    space.low.reset();
    space.high.reset();

    // Unbounded space contains everything
    EXPECT_TRUE(space.contains({-1000.0, 1000.0}));
    EXPECT_TRUE(space.contains({0.0, 0.0}));
}

TEST(SpaceContainsTest, MultiDiscreteValid) {
    auto space = Space::multi_discrete({3, 5, 2});

    EXPECT_TRUE(space.contains({0.0, 0.0, 0.0}));
    EXPECT_TRUE(space.contains({2.0, 4.0, 1.0}));
    EXPECT_FALSE(space.contains({3.0, 0.0, 0.0})); // First out of range
    EXPECT_FALSE(space.contains({0.0, 5.0, 0.0})); // Second out of range
}

TEST(SpaceContainsTest, MultiBinaryValid) {
    auto space = Space::multi_binary(4);

    EXPECT_TRUE(space.contains({0.0, 0.0, 0.0, 0.0}));
    EXPECT_TRUE(space.contains({1.0, 1.0, 1.0, 1.0}));
    EXPECT_TRUE(space.contains({0.0, 1.0, 0.0, 1.0}));
    EXPECT_FALSE(space.contains({0.0, 2.0, 0.0, 0.0})); // Non-binary value
    EXPECT_FALSE(space.contains({0.0, 0.0})); // Wrong size
}

// ============================================================================
// Space Sample Tests
// ============================================================================

TEST(SpaceSampleTest, DiscreteInRange) {
    auto space = Space::discrete(5);

    for (int i = 0; i < 100; ++i) {
        auto sample = space.sample();
        EXPECT_EQ(sample.size(), 1);
        EXPECT_TRUE(space.contains(sample));
        EXPECT_GE(sample[0], 0.0);
        EXPECT_LT(sample[0], 5.0);
    }
}

TEST(SpaceSampleTest, BoxInRange) {
    auto space = Space::box({-1.0, -2.0}, {1.0, 2.0}, {2});

    for (int i = 0; i < 100; ++i) {
        auto sample = space.sample();
        EXPECT_EQ(sample.size(), 2);
        EXPECT_TRUE(space.contains(sample));
    }
}

TEST(SpaceSampleTest, MultiDiscreteInRange) {
    auto space = Space::multi_discrete({3, 5, 7});

    for (int i = 0; i < 100; ++i) {
        auto sample = space.sample();
        EXPECT_EQ(sample.size(), 3);
        EXPECT_TRUE(space.contains(sample));
    }
}

TEST(SpaceSampleTest, MultiBinaryInRange) {
    auto space = Space::multi_binary(8);

    for (int i = 0; i < 100; ++i) {
        auto sample = space.sample();
        EXPECT_EQ(sample.size(), 8);
        EXPECT_TRUE(space.contains(sample));

        // All values should be 0 or 1
        for (auto val : sample) {
            EXPECT_TRUE(val == 0.0 || val == 1.0);
        }
    }
}

// ============================================================================
// Observation Tests
// ============================================================================

TEST(ObservationTest, DefaultConstruction) {
    Observation obs;
    EXPECT_TRUE(obs.data.empty());
    EXPECT_TRUE(obs.dict_data.empty());
}

TEST(ObservationTest, ConstructionWithSpace) {
    auto space = Space::box({-1.0, -1.0}, {1.0, 1.0}, {2});
    Observation obs(space);

    EXPECT_EQ(obs.data.size(), 2);
    EXPECT_EQ(obs.space.type, SpaceType::Box);
}

TEST(ObservationTest, ConstructionWithMultiDimensionalSpace) {
    auto space = Space::box({-1.0}, {1.0}, {4, 3}); // 4x3 = 12 elements
    Observation obs(space);

    EXPECT_EQ(obs.data.size(), 12);
}

// ============================================================================
// Action Tests
// ============================================================================

TEST(ActionTest, DefaultConstruction) {
    Action action;
    EXPECT_TRUE(action.data.empty());
    EXPECT_EQ(action.discrete_value, 0);
}

TEST(ActionTest, ConstructionWithDiscreteSpace) {
    auto space = Space::discrete(5);
    Action action(space);

    EXPECT_EQ(action.discrete_value, 0);
    EXPECT_EQ(action.space.type, SpaceType::Discrete);
}

TEST(ActionTest, ConstructionWithBoxSpace) {
    auto space = Space::box({-1.0, -1.0}, {1.0, 1.0}, {2});
    Action action(space);

    EXPECT_EQ(action.data.size(), 2);
    EXPECT_EQ(action.space.type, SpaceType::Box);
}

TEST(ActionTest, DiscreteFactoryMethod) {
    auto space = Space::discrete(5);
    auto action = Action::discrete(3, space);

    EXPECT_EQ(action.discrete_value, 3);
    EXPECT_EQ(action.space.type, SpaceType::Discrete);
}

TEST(ActionTest, ContinuousFactoryMethod) {
    auto space = Space::box({-1.0, -1.0}, {1.0, 1.0}, {2});
    auto action = Action::continuous({0.5, -0.3}, space);

    EXPECT_EQ(action.data.size(), 2);
    EXPECT_DOUBLE_EQ(action.data[0], 0.5);
    EXPECT_DOUBLE_EQ(action.data[1], -0.3);
    EXPECT_EQ(action.space.type, SpaceType::Box);
}

// ============================================================================
// StepResult Tests
// ============================================================================

TEST(StepResultTest, DefaultConstruction) {
    StepResult result;

    EXPECT_DOUBLE_EQ(result.reward, 0.0);
    EXPECT_FALSE(result.terminated);
    EXPECT_FALSE(result.truncated);
    EXPECT_FALSE(result.is_done());
    EXPECT_TRUE(result.info.empty());
}

TEST(StepResultTest, IsDoneWhenTerminated) {
    StepResult result;
    result.terminated = true;
    result.truncated = false;

    EXPECT_TRUE(result.is_done());
}

TEST(StepResultTest, IsDoneWhenTruncated) {
    StepResult result;
    result.terminated = false;
    result.truncated = true;

    EXPECT_TRUE(result.is_done());
}

TEST(StepResultTest, IsDoneWhenBoth) {
    StepResult result;
    result.terminated = true;
    result.truncated = true;

    EXPECT_TRUE(result.is_done());
}

// ============================================================================
// RewardConfig Tests
// ============================================================================

TEST(RewardConfigTest, DefaultConfig) {
    auto config = RewardConfig::default_config();

    EXPECT_DOUBLE_EQ(config.success_reward, 100.0);
    EXPECT_DOUBLE_EQ(config.failure_penalty, -100.0);
    EXPECT_DOUBLE_EQ(config.step_penalty, -0.1);
    EXPECT_DOUBLE_EQ(config.distance_weight, 1.0);
    EXPECT_TRUE(config.normalize_rewards);
}

TEST(RewardConfigTest, SparseConfig) {
    auto config = RewardConfig::sparse();

    EXPECT_DOUBLE_EQ(config.success_reward, 100.0);
    EXPECT_DOUBLE_EQ(config.failure_penalty, -100.0);
    EXPECT_DOUBLE_EQ(config.step_penalty, 0.0);
    EXPECT_DOUBLE_EQ(config.distance_weight, 0.0);
    EXPECT_DOUBLE_EQ(config.time_weight, 0.0);
    EXPECT_FALSE(config.normalize_rewards);
}

TEST(RewardConfigTest, DenseConfig) {
    auto config = RewardConfig::dense();

    EXPECT_DOUBLE_EQ(config.success_reward, 100.0);
    EXPECT_DOUBLE_EQ(config.step_penalty, -0.01);
    EXPECT_DOUBLE_EQ(config.distance_weight, 10.0);
    EXPECT_DOUBLE_EQ(config.time_weight, 1.0);
    EXPECT_TRUE(config.normalize_rewards);
}

// ============================================================================
// EnvironmentConfig Tests
// ============================================================================

TEST(EnvironmentConfigTest, DefaultConstruction) {
    EnvironmentConfig config;

    EXPECT_EQ(config.max_steps, 1000);
    EXPECT_DOUBLE_EQ(config.time_step, 0.02);
    EXPECT_FALSE(config.render_enabled);
    EXPECT_EQ(config.seed, 0);
}

TEST(EnvironmentConfigTest, WaypointNavigation) {
    auto config = EnvironmentConfig::waypoint_navigation();

    EXPECT_EQ(config.max_steps, 1000);
    EXPECT_DOUBLE_EQ(config.time_step, 0.02);
    EXPECT_EQ(config.observation_space.type, SpaceType::Box);
    EXPECT_EQ(config.action_space.type, SpaceType::Box);
    EXPECT_EQ(config.observation_space.shape[0], 10);
    EXPECT_EQ(config.action_space.shape[0], 4);
}

TEST(EnvironmentConfigTest, FormationFlight) {
    auto config = EnvironmentConfig::formation_flight();

    EXPECT_EQ(config.max_steps, 2000);
    EXPECT_EQ(config.observation_space.shape[0], 19);
    EXPECT_EQ(config.action_space.type, SpaceType::Box);
}

TEST(EnvironmentConfigTest, Intercept) {
    auto config = EnvironmentConfig::intercept();

    EXPECT_EQ(config.max_steps, 500);
    EXPECT_DOUBLE_EQ(config.time_step, 0.01);
    EXPECT_EQ(config.observation_space.shape[0], 26);
    EXPECT_EQ(config.action_space.type, SpaceType::Discrete);
    EXPECT_EQ(*config.action_space.n, 9);
}

// ============================================================================
// EpisodeInfo Tests
// ============================================================================

TEST(EpisodeInfoTest, DefaultConstruction) {
    EpisodeInfo info;

    EXPECT_EQ(info.episode_number, 0);
    EXPECT_EQ(info.step_count, 0);
    EXPECT_DOUBLE_EQ(info.total_reward, 0.0);
    EXPECT_DOUBLE_EQ(info.episode_length_seconds, 0.0);
    EXPECT_FALSE(info.success);
    EXPECT_TRUE(info.termination_reason.empty());
}

TEST(EpisodeInfoTest, GetDurationZeroSteps) {
    EpisodeInfo info;

    auto duration = info.get_duration();
    EXPECT_EQ(duration.count(), 0);
}

TEST(EpisodeInfoTest, GetDurationCompleted) {
    EpisodeInfo info;
    info.step_count = 100;
    info.start_time = std::chrono::system_clock::now() - std::chrono::seconds(5);
    info.end_time = std::chrono::system_clock::now();

    auto duration = info.get_duration();
    EXPECT_GE(duration.count(), 4); // At least 4 seconds
    EXPECT_LE(duration.count(), 6); // At most 6 seconds
}

// ============================================================================
// RLEnvironmentStats Tests
// ============================================================================

TEST(RLEnvironmentStatsTest, DefaultConstruction) {
    RLEnvironmentStats stats;

    EXPECT_EQ(stats.total_episodes, 0);
    EXPECT_EQ(stats.total_steps, 0);
    EXPECT_DOUBLE_EQ(stats.average_reward, 0.0);
    EXPECT_DOUBLE_EQ(stats.average_episode_length, 0.0);
    EXPECT_EQ(stats.successful_episodes, 0);
    EXPECT_DOUBLE_EQ(stats.success_rate, 0.0);
}

TEST(RLEnvironmentStatsTest, UpdateWithEpisode) {
    RLEnvironmentStats stats;

    EpisodeInfo episode1;
    episode1.step_count = 100;
    episode1.total_reward = 50.0;
    episode1.success = true;

    stats.update(episode1);

    EXPECT_EQ(stats.total_episodes, 1);
    EXPECT_EQ(stats.total_steps, 100);
    EXPECT_DOUBLE_EQ(stats.average_reward, 50.0);
    EXPECT_DOUBLE_EQ(stats.average_episode_length, 100.0);
    EXPECT_EQ(stats.successful_episodes, 1);
    EXPECT_DOUBLE_EQ(stats.success_rate, 1.0);
}

TEST(RLEnvironmentStatsTest, UpdateMultipleEpisodes) {
    RLEnvironmentStats stats;

    EpisodeInfo episode1;
    episode1.step_count = 100;
    episode1.total_reward = 50.0;
    episode1.success = true;
    stats.update(episode1);

    EpisodeInfo episode2;
    episode2.step_count = 200;
    episode2.total_reward = 30.0;
    episode2.success = false;
    stats.update(episode2);

    EXPECT_EQ(stats.total_episodes, 2);
    EXPECT_EQ(stats.total_steps, 300);
    EXPECT_NEAR(stats.average_reward, 40.0, 1e-6);
    EXPECT_NEAR(stats.average_episode_length, 150.0, 1e-6);
    EXPECT_EQ(stats.successful_episodes, 1);
    EXPECT_DOUBLE_EQ(stats.success_rate, 0.5);
}

TEST(RLEnvironmentStatsTest, Reset) {
    RLEnvironmentStats stats;

    EpisodeInfo episode;
    episode.step_count = 100;
    episode.total_reward = 50.0;
    stats.update(episode);

    stats.reset();

    EXPECT_EQ(stats.total_episodes, 0);
    EXPECT_EQ(stats.total_steps, 0);
    EXPECT_DOUBLE_EQ(stats.average_reward, 0.0);
}

// ============================================================================
// RLEnvironment Lifecycle Tests
// ============================================================================

class RLEnvironmentTest : public ::testing::Test {
protected:
    void SetUp() override {
        config = EnvironmentConfig::waypoint_navigation();
        config.seed = 42; // Fixed seed for reproducibility
        env = std::make_unique<RLEnvironment>(config);
    }

    EnvironmentConfig config;
    std::unique_ptr<RLEnvironment> env;
};

TEST_F(RLEnvironmentTest, InitiallyNotInitialized) {
    EXPECT_FALSE(env->is_initialized());
}

TEST_F(RLEnvironmentTest, Initialize) {
    auto result = env->initialize();
    EXPECT_EQ(result, RLResult::Success);
    EXPECT_TRUE(env->is_initialized());
}

TEST_F(RLEnvironmentTest, InitializeTwice) {
    env->initialize();
    auto result = env->initialize();
    EXPECT_EQ(result, RLResult::AlreadyInitialized);
}

TEST_F(RLEnvironmentTest, Shutdown) {
    env->initialize();
    auto result = env->shutdown();
    EXPECT_EQ(result, RLResult::Success);
    EXPECT_FALSE(env->is_initialized());
}

TEST_F(RLEnvironmentTest, ShutdownWithoutInit) {
    auto result = env->shutdown();
    EXPECT_EQ(result, RLResult::NotInitialized);
}

TEST_F(RLEnvironmentTest, GetConfig) {
    auto& retrieved_config = env->get_config();
    EXPECT_EQ(retrieved_config.max_steps, config.max_steps);
    EXPECT_DOUBLE_EQ(retrieved_config.time_step, config.time_step);
}

// ============================================================================
// RLEnvironment Space Tests
// ============================================================================

TEST_F(RLEnvironmentTest, GetObservationSpace) {
    auto obs_space = env->get_observation_space();

    EXPECT_EQ(obs_space.type, SpaceType::Box);
    EXPECT_EQ(obs_space.shape[0], 10);
    EXPECT_TRUE(obs_space.low.has_value());
    EXPECT_TRUE(obs_space.high.has_value());
}

TEST_F(RLEnvironmentTest, GetActionSpace) {
    auto action_space = env->get_action_space();

    EXPECT_EQ(action_space.type, SpaceType::Box);
    EXPECT_EQ(action_space.shape[0], 4);
}

// ============================================================================
// RLEnvironment Reset Tests
// ============================================================================

TEST_F(RLEnvironmentTest, ResetWithoutInit) {
    Observation obs;
    auto result = env->reset(obs);
    EXPECT_EQ(result, RLResult::NotInitialized);
}

TEST_F(RLEnvironmentTest, ResetSuccessful) {
    env->initialize();

    Observation obs;
    auto result = env->reset(obs);

    EXPECT_EQ(result, RLResult::Success);
    EXPECT_EQ(obs.data.size(), 10);
    EXPECT_FALSE(env->is_episode_done());
    EXPECT_EQ(env->get_current_step(), 0);
}

TEST_F(RLEnvironmentTest, ResetResetsStepCount) {
    env->initialize();

    Observation obs;
    env->reset(obs);

    // Take some steps
    Action action(env->get_action_space());
    action.data = {0.5, 0.0, 0.0, 0.0};
    StepResult step_result;
    env->step(action, step_result);
    env->step(action, step_result);

    EXPECT_EQ(env->get_current_step(), 2);

    // Reset should zero step count
    env->reset(obs);
    EXPECT_EQ(env->get_current_step(), 0);
}

TEST_F(RLEnvironmentTest, ResetIncreasesEpisodeNumber) {
    env->initialize();

    Observation obs;
    env->reset(obs);
    auto info1 = env->get_episode_info();

    env->reset(obs);
    auto info2 = env->get_episode_info();

    EXPECT_EQ(info2.episode_number, info1.episode_number + 1);
}

// ============================================================================
// RLEnvironment Step Tests
// ============================================================================

TEST_F(RLEnvironmentTest, StepWithoutInit) {
    Action action(env->get_action_space());
    StepResult result;

    auto res = env->step(action, result);
    EXPECT_EQ(res, RLResult::NotInitialized);
}

TEST_F(RLEnvironmentTest, StepWithoutReset) {
    env->initialize();

    Action action(env->get_action_space());
    StepResult result;

    auto res = env->step(action, result);
    EXPECT_EQ(res, RLResult::EpisodeNotStarted);
}

TEST_F(RLEnvironmentTest, StepSuccessful) {
    env->initialize();

    Observation obs;
    env->reset(obs);

    Action action(env->get_action_space());
    action.data = {0.5, 0.0, 0.0, 0.0};

    StepResult result;
    auto res = env->step(action, result);

    EXPECT_EQ(res, RLResult::Success);
    EXPECT_EQ(result.observation.data.size(), 10);
    EXPECT_EQ(env->get_current_step(), 1);
}

TEST_F(RLEnvironmentTest, StepIncrementsStepCount) {
    env->initialize();

    Observation obs;
    env->reset(obs);

    Action action(env->get_action_space());
    action.data = {0.5, 0.0, 0.0, 0.0};

    StepResult result;
    env->step(action, result);
    EXPECT_EQ(env->get_current_step(), 1);

    env->step(action, result);
    EXPECT_EQ(env->get_current_step(), 2);
}

TEST_F(RLEnvironmentTest, StepProducesReward) {
    env->initialize();

    Observation obs;
    env->reset(obs);

    Action action(env->get_action_space());
    action.data = {0.5, 0.0, 0.0, 0.0};

    StepResult result;
    env->step(action, result);

    // Should have some reward (likely negative due to step penalty and distance)
    EXPECT_NE(result.reward, 0.0);
}

TEST_F(RLEnvironmentTest, StepWithInvalidAction) {
    env->initialize();

    Observation obs;
    env->reset(obs);

    // Create action with out-of-bounds values
    Action action(env->get_action_space());
    action.data = {10.0, 10.0, 10.0, 10.0}; // Way out of [-1, 1] bounds

    StepResult result;
    auto res = env->step(action, result);

    EXPECT_EQ(res, RLResult::InvalidAction);
}

TEST_F(RLEnvironmentTest, StepAfterEpisodeDone) {
    env->initialize();

    Observation obs;
    env->reset(obs);

    // Force episode to end by taking max steps
    Action action(env->get_action_space());
    action.data = {0.0, 0.0, 0.0, 0.0};

    StepResult result;
    for (UInt32 i = 0; i < config.max_steps; ++i) {
        env->step(action, result);
    }

    EXPECT_TRUE(env->is_episode_done());

    // Try to step after done
    auto res = env->step(action, result);
    EXPECT_EQ(res, RLResult::EpisodeDone);
}

// ============================================================================
// RLEnvironment Termination Tests
// ============================================================================

TEST_F(RLEnvironmentTest, TruncationAtMaxSteps) {
    env->initialize();

    Observation obs;
    env->reset(obs);

    Action action(env->get_action_space());
    action.data = {0.0, 0.0, 0.0, 0.0};

    StepResult result;
    for (UInt32 i = 0; i < config.max_steps - 1; ++i) {
        env->step(action, result);
        EXPECT_FALSE(result.is_done());
    }

    // Final step should truncate
    env->step(action, result);
    EXPECT_TRUE(result.truncated);
    EXPECT_TRUE(result.is_done());
    EXPECT_TRUE(env->is_episode_done());
}

// ============================================================================
// RLEnvironment Seeding Tests
// ============================================================================

TEST_F(RLEnvironmentTest, SetSeed) {
    env->initialize();

    auto result = env->set_seed(12345);
    EXPECT_EQ(result, RLResult::Success);
}

TEST_F(RLEnvironmentTest, SeedProducesReproducibility) {
    // Create two environments with same seed
    auto env1 = std::make_unique<RLEnvironment>(config);
    auto env2 = std::make_unique<RLEnvironment>(config);

    env1->initialize();
    env2->initialize();

    env1->set_seed(42);
    env2->set_seed(42);

    Observation obs1, obs2;
    env1->reset(obs1);
    env2->reset(obs2);

    // Initial observations should be similar (both seeded)
    EXPECT_EQ(obs1.data.size(), obs2.data.size());

    // Take same actions
    Action action(env1->get_action_space());
    action.data = {0.5, 0.1, -0.2, 0.3};

    StepResult result1, result2;
    env1->step(action, result1);
    env2->step(action, result2);

    // Results should be identical with same seed
    EXPECT_TRUE(vector_nearly_equal(result1.observation.data, result2.observation.data, 1e-9));
}

TEST_F(RLEnvironmentTest, ZeroSeedUsesRandom) {
    env->initialize();

    auto result = env->set_seed(0);
    EXPECT_EQ(result, RLResult::Success);
}

// ============================================================================
// RLEnvironment Episode Info Tests
// ============================================================================

TEST_F(RLEnvironmentTest, GetEpisodeInfoBeforeReset) {
    env->initialize();

    auto info = env->get_episode_info();
    EXPECT_EQ(info.episode_number, 0);
    EXPECT_EQ(info.step_count, 0);
}

TEST_F(RLEnvironmentTest, GetEpisodeInfoAfterReset) {
    env->initialize();

    Observation obs;
    env->reset(obs);

    auto info = env->get_episode_info();
    EXPECT_EQ(info.episode_number, 1);
    EXPECT_EQ(info.step_count, 0);
}

TEST_F(RLEnvironmentTest, EpisodeInfoTracksSteps) {
    env->initialize();

    Observation obs;
    env->reset(obs);

    Action action(env->get_action_space());
    action.data = {0.5, 0.0, 0.0, 0.0};

    StepResult result;
    env->step(action, result);
    env->step(action, result);

    auto info = env->get_episode_info();
    EXPECT_EQ(info.step_count, 2);
}

TEST_F(RLEnvironmentTest, EpisodeInfoTracksTotalReward) {
    env->initialize();

    Observation obs;
    env->reset(obs);

    Action action(env->get_action_space());
    action.data = {0.5, 0.0, 0.0, 0.0};

    StepResult result;
    env->step(action, result);
    Real reward1 = result.reward;

    env->step(action, result);
    Real reward2 = result.reward;

    auto info = env->get_episode_info();
    EXPECT_NEAR(info.total_reward, reward1 + reward2, 1e-6);
}

// ============================================================================
// RLEnvironment Statistics Tests
// ============================================================================

TEST_F(RLEnvironmentTest, GetStatsInitially) {
    env->initialize();

    auto stats = env->get_stats();
    EXPECT_EQ(stats.total_episodes, 0);
    EXPECT_EQ(stats.total_steps, 0);
}

TEST_F(RLEnvironmentTest, StatsUpdatedAfterEpisode) {
    env->initialize();

    Observation obs;
    env->reset(obs);

    // Complete episode
    Action action(env->get_action_space());
    action.data = {0.0, 0.0, 0.0, 0.0};

    StepResult result;
    for (UInt32 i = 0; i < config.max_steps; ++i) {
        env->step(action, result);
    }

    auto stats = env->get_stats();
    EXPECT_EQ(stats.total_episodes, 1);
    EXPECT_EQ(stats.total_steps, config.max_steps);
}

// ============================================================================
// RLEnvironment Render Tests
// ============================================================================

TEST_F(RLEnvironmentTest, RenderStub) {
    env->initialize();

    auto result = env->render();
    EXPECT_EQ(result, RLResult::Success);
}

TEST_F(RLEnvironmentTest, Close) {
    env->initialize();

    auto result = env->close();
    EXPECT_EQ(result, RLResult::Success);
    EXPECT_FALSE(env->is_initialized());
}

// ============================================================================
// RLEnvironment Custom Reward Function Tests
// ============================================================================

class MockRewardFunction : public IRewardFunction {
public:
    Real compute(const Observation&, const Action&, const Observation&, bool) override {
        return 42.0; // Fixed reward for testing
    }

    void reset() override {
        reset_called = true;
    }

    bool reset_called = false;
};

TEST_F(RLEnvironmentTest, SetRewardFunction) {
    env->initialize();

    auto mock_reward = std::make_shared<MockRewardFunction>();
    auto result = env->set_reward_function(mock_reward);

    EXPECT_EQ(result, RLResult::Success);
}

TEST_F(RLEnvironmentTest, SetNullRewardFunction) {
    env->initialize();

    auto result = env->set_reward_function(nullptr);
    EXPECT_EQ(result, RLResult::InvalidConfiguration);
}

TEST_F(RLEnvironmentTest, CustomRewardFunctionUsed) {
    env->initialize();

    auto mock_reward = std::make_shared<MockRewardFunction>();
    env->set_reward_function(mock_reward);

    Observation obs;
    env->reset(obs);

    EXPECT_TRUE(mock_reward->reset_called);

    Action action(env->get_action_space());
    action.data = {0.5, 0.0, 0.0, 0.0};

    StepResult result;
    env->step(action, result);

    // Should have custom reward (42.0) plus step penalty
    EXPECT_NEAR(result.reward, 42.0 + config.reward_config.step_penalty, 1e-6);
}

// ============================================================================
// RLEnvironment Custom Termination Condition Tests
// ============================================================================

class MockTerminationCondition : public ITerminationCondition {
public:
    bool is_terminated(const Observation&, UInt32 step) override {
        return step >= terminate_at_step;
    }

    bool is_truncated(const Observation&, UInt32) override {
        return false;
    }

    std::string get_reason() const override {
        return "Mock termination";
    }

    UInt32 terminate_at_step = 10;
};

TEST_F(RLEnvironmentTest, SetTerminationCondition) {
    env->initialize();

    auto mock_term = std::make_shared<MockTerminationCondition>();
    auto result = env->set_termination_condition(mock_term);

    EXPECT_EQ(result, RLResult::Success);
}

TEST_F(RLEnvironmentTest, SetNullTerminationCondition) {
    env->initialize();

    auto result = env->set_termination_condition(nullptr);
    EXPECT_EQ(result, RLResult::InvalidConfiguration);
}

TEST_F(RLEnvironmentTest, CustomTerminationConditionUsed) {
    env->initialize();

    auto mock_term = std::make_shared<MockTerminationCondition>();
    mock_term->terminate_at_step = 5;
    env->set_termination_condition(mock_term);

    Observation obs;
    env->reset(obs);

    Action action(env->get_action_space());
    action.data = {0.5, 0.0, 0.0, 0.0};

    StepResult result;
    for (UInt32 i = 0; i < 4; ++i) {
        env->step(action, result);
        EXPECT_FALSE(result.is_done());
    }

    // Step 5 should terminate
    env->step(action, result);
    EXPECT_TRUE(result.terminated);
    EXPECT_TRUE(result.is_done());
}

// ============================================================================
// Helper Function Tests
// ============================================================================

TEST(HelperFunctionTest, GetSpaceSize) {
    auto discrete = Space::discrete(5);
    EXPECT_EQ(get_space_size(discrete), 5);

    auto box = Space::box({-1.0, -1.0}, {1.0, 1.0}, {2});
    EXPECT_EQ(get_space_size(box), 2);

    auto multi_discrete = Space::multi_discrete({3, 4, 5});
    EXPECT_EQ(get_space_size(multi_discrete), 3);

    auto empty = Space();
    empty.shape = {};
    EXPECT_EQ(get_space_size(empty), 0);
}

TEST(HelperFunctionTest, IsContinuousSpace) {
    auto box = Space::box({-1.0}, {1.0}, {1});
    EXPECT_TRUE(is_continuous_space(box));

    auto discrete = Space::discrete(5);
    EXPECT_FALSE(is_continuous_space(discrete));
}

TEST(HelperFunctionTest, IsDiscreteSpace) {
    auto discrete = Space::discrete(5);
    EXPECT_TRUE(is_discrete_space(discrete));

    auto multi_discrete = Space::multi_discrete({3, 4});
    EXPECT_TRUE(is_discrete_space(multi_discrete));

    auto multi_binary = Space::multi_binary(8);
    EXPECT_TRUE(is_discrete_space(multi_binary));

    auto box = Space::box({-1.0}, {1.0}, {1});
    EXPECT_FALSE(is_discrete_space(box));
}

TEST(HelperFunctionTest, ClipToSpace) {
    auto space = Space::box({-1.0, -2.0}, {1.0, 2.0}, {2});

    auto clipped = clip_to_space({0.5, 0.5}, space);
    EXPECT_TRUE(vector_nearly_equal(clipped, {0.5, 0.5}));

    clipped = clip_to_space({-2.0, 3.0}, space);
    EXPECT_TRUE(vector_nearly_equal(clipped, {-1.0, 2.0}));

    clipped = clip_to_space({2.0, -3.0}, space);
    EXPECT_TRUE(vector_nearly_equal(clipped, {1.0, -2.0}));
}

TEST(HelperFunctionTest, NormalizeFromSpace) {
    auto space = Space::box({-1.0, 0.0}, {1.0, 10.0}, {2});

    auto normalized = normalize_from_space({0.0, 5.0}, space);
    EXPECT_NEAR(normalized[0], 0.5, 1e-6);
    EXPECT_NEAR(normalized[1], 0.5, 1e-6);

    normalized = normalize_from_space({-1.0, 0.0}, space);
    EXPECT_NEAR(normalized[0], 0.0, 1e-6);
    EXPECT_NEAR(normalized[1], 0.0, 1e-6);

    normalized = normalize_from_space({1.0, 10.0}, space);
    EXPECT_NEAR(normalized[0], 1.0, 1e-6);
    EXPECT_NEAR(normalized[1], 1.0, 1e-6);
}

TEST(HelperFunctionTest, DenormalizeToSpace) {
    auto space = Space::box({-1.0, 0.0}, {1.0, 10.0}, {2});

    auto denormalized = denormalize_to_space({0.5, 0.5}, space);
    EXPECT_NEAR(denormalized[0], 0.0, 1e-6);
    EXPECT_NEAR(denormalized[1], 5.0, 1e-6);

    denormalized = denormalize_to_space({0.0, 0.0}, space);
    EXPECT_NEAR(denormalized[0], -1.0, 1e-6);
    EXPECT_NEAR(denormalized[1], 0.0, 1e-6);

    denormalized = denormalize_to_space({1.0, 1.0}, space);
    EXPECT_NEAR(denormalized[0], 1.0, 1e-6);
    EXPECT_NEAR(denormalized[1], 10.0, 1e-6);
}

TEST(HelperFunctionTest, FormatEpisodeInfo) {
    EpisodeInfo info;
    info.episode_number = 42;
    info.step_count = 150;
    info.total_reward = 75.5;
    info.episode_length_seconds = 3.0;
    info.success = true;
    info.termination_reason = "Goal reached";

    auto formatted = format_episode_info(info);

    EXPECT_NE(formatted.find("42"), std::string::npos);
    EXPECT_NE(formatted.find("150"), std::string::npos);
    EXPECT_NE(formatted.find("75.5"), std::string::npos);
    EXPECT_NE(formatted.find("SUCCESS"), std::string::npos);
    EXPECT_NE(formatted.find("Goal reached"), std::string::npos);
}

TEST(HelperFunctionTest, CalculateRewardStats) {
    std::vector<EpisodeInfo> episodes;

    EpisodeInfo ep1;
    ep1.total_reward = 10.0;
    episodes.push_back(ep1);

    EpisodeInfo ep2;
    ep2.total_reward = 20.0;
    episodes.push_back(ep2);

    EpisodeInfo ep3;
    ep3.total_reward = 30.0;
    episodes.push_back(ep3);

    auto [mean, std_dev, min_reward, max_reward] = calculate_reward_stats(episodes);

    EXPECT_NEAR(mean, 20.0, 1e-6);
    EXPECT_NEAR(min_reward, 10.0, 1e-6);
    EXPECT_NEAR(max_reward, 30.0, 1e-6);
    EXPECT_GT(std_dev, 0.0);
}

TEST(HelperFunctionTest, CalculateRewardStatsEmpty) {
    std::vector<EpisodeInfo> episodes;

    auto [mean, std_dev, min_reward, max_reward] = calculate_reward_stats(episodes);

    EXPECT_DOUBLE_EQ(mean, 0.0);
    EXPECT_DOUBLE_EQ(std_dev, 0.0);
    EXPECT_DOUBLE_EQ(min_reward, 0.0);
    EXPECT_DOUBLE_EQ(max_reward, 0.0);
}

// ============================================================================
// Factory Function Tests
// ============================================================================

TEST(FactoryFunctionTest, CreateRLEnvironment) {
    auto config = EnvironmentConfig::waypoint_navigation();
    auto env = create_rl_environment(config);

    EXPECT_NE(env, nullptr);
    EXPECT_FALSE(env->is_initialized());
}

TEST(FactoryFunctionTest, CreateWaypointEnvironment) {
    auto env = create_waypoint_environment();

    EXPECT_NE(env, nullptr);
    EXPECT_TRUE(env->is_initialized()); // Factory initializes it
}

TEST(FactoryFunctionTest, CreateDistanceReward) {
    auto reward = create_distance_reward(2.0);

    EXPECT_NE(reward, nullptr);
}

TEST(FactoryFunctionTest, CreateSparseReward) {
    auto reward = create_sparse_reward(50.0, -50.0);

    EXPECT_NE(reward, nullptr);
}

TEST(FactoryFunctionTest, CreateGoalTermination) {
    auto term = create_goal_termination(15.0);

    EXPECT_NE(term, nullptr);
}

TEST(FactoryFunctionTest, CreateTimeTermination) {
    auto term = create_time_termination(500);

    EXPECT_NE(term, nullptr);
}

// ============================================================================
// Discrete Action Space Tests
// ============================================================================

TEST(DiscreteActionTest, InterceptEnvironment) {
    auto config = EnvironmentConfig::intercept();
    auto env = std::make_unique<RLEnvironment>(config);

    env->initialize();

    Observation obs;
    env->reset(obs);

    // Test discrete actions
    for (Int64 i = 0; i < 9; ++i) {
        auto action = Action::discrete(i, env->get_action_space());

        StepResult result;
        auto res = env->step(action, result);

        if (res == RLResult::Success) {
            EXPECT_EQ(result.observation.data.size(), 26);
        }

        env->reset(obs); // Reset for next action
    }
}

// ============================================================================
// Multi-Episode Tests
// ============================================================================

TEST(MultiEpisodeTest, RunMultipleEpisodes) {
    auto config = EnvironmentConfig::waypoint_navigation();
    config.max_steps = 100; // Shorter episodes
    auto env = std::make_unique<RLEnvironment>(config);

    env->initialize();

    for (int ep = 0; ep < 5; ++ep) {
        Observation obs;
        env->reset(obs);

        Action action(env->get_action_space());
        action.data = {0.5, 0.0, 0.0, 0.0};

        StepResult result;
        while (!env->is_episode_done()) {
            env->step(action, result);
        }

        auto info = env->get_episode_info();
        EXPECT_EQ(info.episode_number, ep + 1);
        EXPECT_GT(info.step_count, 0);
    }

    auto stats = env->get_stats();
    EXPECT_EQ(stats.total_episodes, 5);
}

// ============================================================================
// Edge Case Tests
// ============================================================================

TEST(EdgeCaseTest, ZeroStepEpisode) {
    auto config = EnvironmentConfig::waypoint_navigation();
    config.max_steps = 0; // Edge case: zero steps allowed
    auto env = std::make_unique<RLEnvironment>(config);

    env->initialize();

    Observation obs;
    env->reset(obs);

    // Should immediately be truncated
    Action action(env->get_action_space());
    action.data = {0.0, 0.0, 0.0, 0.0};

    StepResult result;
    auto res = env->step(action, result);

    // Might be truncated or succeed depending on implementation
    EXPECT_TRUE(res == RLResult::Success || res == RLResult::EpisodeDone);
}

TEST(EdgeCaseTest, VeryLargeObservationSpace) {
    EnvironmentConfig config;
    config.observation_space = Space::box(
        std::vector<Real>(1000, -1.0),
        std::vector<Real>(1000, 1.0),
        {1000}
    );
    config.action_space = Space::discrete(5);

    auto env = std::make_unique<RLEnvironment>(config);
    EXPECT_NE(env, nullptr);
}

TEST(EdgeCaseTest, VerySmallTimeStep) {
    auto config = EnvironmentConfig::waypoint_navigation();
    config.time_step = 0.00001; // Very small time step

    auto env = std::make_unique<RLEnvironment>(config);
    env->initialize();

    Observation obs;
    auto result = env->reset(obs);
    EXPECT_EQ(result, RLResult::Success);
}

// ============================================================================
// Integration Test
// ============================================================================

TEST(IntegrationTest, CompleteTrainingLoop) {
    auto config = EnvironmentConfig::waypoint_navigation();
    config.max_steps = 50;
    config.seed = 42;
    auto env = create_rl_environment(config);

    env->initialize();

    const int num_episodes = 10;

    for (int episode = 0; episode < num_episodes; ++episode) {
        Observation obs;
        env->reset(obs);

        while (!env->is_episode_done()) {
            // Simple policy: always thrust forward
            Action action(env->get_action_space());
            action.data = {1.0, 0.0, 0.0, 0.0};

            StepResult result;
            env->step(action, result);

            // Verify result structure
            EXPECT_EQ(result.observation.data.size(), 10);
            EXPECT_FALSE(result.info.empty());
        }
    }

    auto stats = env->get_stats();
    EXPECT_EQ(stats.total_episodes, num_episodes);
    EXPECT_GT(stats.total_steps, 0);
    EXPECT_LE(stats.average_episode_length, config.max_steps);
}
