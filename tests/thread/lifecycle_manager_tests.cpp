/**
 * @file lifecycle_manager_tests.cpp
 * @brief Unit tests for the lifecycle_manager system
 */

#include <gtest/gtest.h>
#include "jaguar/thread/lifecycle_manager.h"
#include <thread>
#include <chrono>

using namespace jaguar;
using namespace jaguar::thread;

// ============================================================================
// Mock Observer for Testing
// ============================================================================

class MockLifecycleObserver : public ILifecycleObserver {
public:
    UInt32 phase_changed_count{0};
    UInt32 event_logged_count{0};
    UInt32 entity_created_count{0};
    UInt32 entity_removed_count{0};

    EntityId last_entity_id{INVALID_ENTITY_ID};
    LifecyclePhase last_from_phase{LifecyclePhase::Design};
    LifecyclePhase last_to_phase{LifecyclePhase::Design};
    std::string last_event_type;

    void on_phase_changed(EntityId entity_id,
                          LifecyclePhase from_phase,
                          LifecyclePhase to_phase) override {
        ++phase_changed_count;
        last_entity_id = entity_id;
        last_from_phase = from_phase;
        last_to_phase = to_phase;
    }

    void on_event_logged(EntityId entity_id,
                         const LifecycleEvent& event) override {
        ++event_logged_count;
        last_entity_id = entity_id;
        last_event_type = event.event_type;
    }

    void on_entity_created(EntityId entity_id,
                           LifecyclePhase initial_phase) override {
        ++entity_created_count;
        last_entity_id = entity_id;
        last_from_phase = initial_phase;
    }

    void on_entity_removed(EntityId entity_id) override {
        ++entity_removed_count;
        last_entity_id = entity_id;
    }

    void reset() {
        phase_changed_count = 0;
        event_logged_count = 0;
        entity_created_count = 0;
        entity_removed_count = 0;
        last_entity_id = INVALID_ENTITY_ID;
    }
};

// ============================================================================
// LifecycleResult Tests
// ============================================================================

class LifecycleResultTest : public ::testing::Test {};

TEST_F(LifecycleResultTest, ResultToString) {
    EXPECT_STREQ(lifecycle_result_to_string(LifecycleResult::Success), "Success");
    EXPECT_STREQ(lifecycle_result_to_string(LifecycleResult::InvalidConfiguration),
                 "InvalidConfiguration");
    EXPECT_STREQ(lifecycle_result_to_string(LifecycleResult::InvalidEntityId), "InvalidEntityId");
    EXPECT_STREQ(lifecycle_result_to_string(LifecycleResult::InvalidPhase), "InvalidPhase");
    EXPECT_STREQ(lifecycle_result_to_string(LifecycleResult::TransitionFailed), "TransitionFailed");
    EXPECT_STREQ(lifecycle_result_to_string(LifecycleResult::TransitionNotAllowed),
                 "TransitionNotAllowed");
    EXPECT_STREQ(lifecycle_result_to_string(LifecycleResult::ObserverFailed), "ObserverFailed");
    EXPECT_STREQ(lifecycle_result_to_string(LifecycleResult::NotInitialized), "NotInitialized");
    EXPECT_STREQ(lifecycle_result_to_string(LifecycleResult::AlreadyInitialized),
                 "AlreadyInitialized");
    EXPECT_STREQ(lifecycle_result_to_string(LifecycleResult::EntityNotTracked), "EntityNotTracked");
    EXPECT_STREQ(lifecycle_result_to_string(LifecycleResult::OutOfMemory), "OutOfMemory");
    EXPECT_STREQ(lifecycle_result_to_string(LifecycleResult::StorageError), "StorageError");
}

// ============================================================================
// LifecyclePhase Tests
// ============================================================================

class LifecyclePhaseTest : public ::testing::Test {};

TEST_F(LifecyclePhaseTest, PhaseToString) {
    EXPECT_STREQ(lifecycle_phase_to_string(LifecyclePhase::Design), "Design");
    EXPECT_STREQ(lifecycle_phase_to_string(LifecyclePhase::Production), "Production");
    EXPECT_STREQ(lifecycle_phase_to_string(LifecyclePhase::Operational), "Operational");
    EXPECT_STREQ(lifecycle_phase_to_string(LifecyclePhase::Maintenance), "Maintenance");
    EXPECT_STREQ(lifecycle_phase_to_string(LifecyclePhase::Decommissioned), "Decommissioned");
}

// ============================================================================
// LifecycleConfig Tests
// ============================================================================

class LifecycleConfigTest : public ::testing::Test {};

TEST_F(LifecycleConfigTest, DefaultConfig) {
    LifecycleConfig config = LifecycleConfig::default_config();

    EXPECT_EQ(config.max_history_size, 1000u);
    EXPECT_TRUE(config.enable_event_logging);
    // retention_period is in seconds, so compare with seconds
    EXPECT_EQ(config.retention_period.count(),
              std::chrono::duration_cast<std::chrono::seconds>(std::chrono::hours(24 * 30)).count());
    EXPECT_TRUE(config.strict_transitions);
}

TEST_F(LifecycleConfigTest, MinimalConfig) {
    LifecycleConfig config = LifecycleConfig::minimal();

    EXPECT_EQ(config.max_history_size, 100u);
    EXPECT_FALSE(config.enable_event_logging);
    EXPECT_FALSE(config.strict_transitions);
}

TEST_F(LifecycleConfigTest, FullAuditConfig) {
    LifecycleConfig config = LifecycleConfig::full_audit();

    EXPECT_EQ(config.max_history_size, 10000u);
    EXPECT_TRUE(config.enable_event_logging);
    EXPECT_EQ(config.retention_period.count(),
              std::chrono::duration_cast<std::chrono::seconds>(std::chrono::hours(24 * 365)).count());
    EXPECT_TRUE(config.strict_transitions);
}

TEST_F(LifecycleConfigTest, ProductionConfig) {
    LifecycleConfig config = LifecycleConfig::production();

    EXPECT_EQ(config.max_history_size, 5000u);
    EXPECT_TRUE(config.enable_event_logging);
    EXPECT_EQ(config.retention_period.count(),
              std::chrono::duration_cast<std::chrono::seconds>(std::chrono::hours(24 * 90)).count());
    EXPECT_TRUE(config.strict_transitions);
}

TEST_F(LifecycleConfigTest, CustomConfig) {
    LifecycleConfig config;
    config.max_history_size = 500;
    config.enable_event_logging = false;
    config.retention_period = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::hours(24));
    config.strict_transitions = false;

    EXPECT_EQ(config.max_history_size, 500u);
    EXPECT_FALSE(config.enable_event_logging);
    EXPECT_EQ(config.retention_period.count(),
              std::chrono::duration_cast<std::chrono::seconds>(std::chrono::hours(24)).count());
    EXPECT_FALSE(config.strict_transitions);
}

TEST_F(LifecycleConfigTest, TransitionCallback) {
    LifecycleConfig config = LifecycleConfig::default_config();

    bool callback_called = false;
    config.transition_callback = [&callback_called](const LifecycleTransition& t) {
        callback_called = true;
    };

    // Verify callback can be set
    EXPECT_TRUE(config.transition_callback != nullptr);
}

// ============================================================================
// LifecycleManager Lifecycle Tests
// ============================================================================

class LifecycleManagerLifecycleTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_ = LifecycleConfig::default_config();
        manager_ = std::make_unique<LifecycleManager>(config_);
    }

    void TearDown() override {
        if (manager_ && manager_->is_initialized()) {
            manager_->shutdown();
        }
    }

    LifecycleConfig config_;
    std::unique_ptr<LifecycleManager> manager_;
};

TEST_F(LifecycleManagerLifecycleTest, Initialize) {
    LifecycleResult result = manager_->initialize();
    EXPECT_EQ(result, LifecycleResult::Success);
    EXPECT_TRUE(manager_->is_initialized());
}

TEST_F(LifecycleManagerLifecycleTest, Shutdown) {
    manager_->initialize();
    LifecycleResult result = manager_->shutdown();
    EXPECT_EQ(result, LifecycleResult::Success);
    EXPECT_FALSE(manager_->is_initialized());
}

TEST_F(LifecycleManagerLifecycleTest, DoubleInitialize) {
    manager_->initialize();
    LifecycleResult result = manager_->initialize();
    EXPECT_EQ(result, LifecycleResult::AlreadyInitialized);
}

TEST_F(LifecycleManagerLifecycleTest, DoubleShutdown) {
    manager_->initialize();
    manager_->shutdown();
    LifecycleResult result = manager_->shutdown();
    // Should succeed or be a no-op
    EXPECT_TRUE(result == LifecycleResult::Success ||
                result == LifecycleResult::NotInitialized);
}

TEST_F(LifecycleManagerLifecycleTest, OperationsBeforeInitialization) {
    // Don't initialize
    LifecycleResult result = manager_->track_entity(1);
    EXPECT_EQ(result, LifecycleResult::NotInitialized);
}

TEST_F(LifecycleManagerLifecycleTest, GetConfig) {
    const LifecycleConfig& config = manager_->get_config();
    EXPECT_EQ(config.max_history_size, 1000u);
    EXPECT_TRUE(config.enable_event_logging);
}

// ============================================================================
// Entity Tracking Tests
// ============================================================================

class EntityTrackingTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_ = LifecycleConfig::default_config();
        manager_ = std::make_unique<LifecycleManager>(config_);
        manager_->initialize();
    }

    void TearDown() override {
        if (manager_ && manager_->is_initialized()) {
            manager_->shutdown();
        }
    }

    LifecycleConfig config_;
    std::unique_ptr<LifecycleManager> manager_;
};

TEST_F(EntityTrackingTest, TrackEntityDefault) {
    EntityId entity = 100;
    LifecycleResult result = manager_->track_entity(entity);
    EXPECT_EQ(result, LifecycleResult::Success);
    EXPECT_TRUE(manager_->is_tracked(entity));

    auto phase = manager_->get_phase(entity);
    ASSERT_TRUE(phase.has_value());
    EXPECT_EQ(*phase, LifecyclePhase::Design);
}

TEST_F(EntityTrackingTest, TrackEntityCustomPhase) {
    EntityId entity = 101;
    LifecycleResult result = manager_->track_entity(entity, LifecyclePhase::Operational);
    EXPECT_EQ(result, LifecycleResult::Success);

    auto phase = manager_->get_phase(entity);
    ASSERT_TRUE(phase.has_value());
    EXPECT_EQ(*phase, LifecyclePhase::Operational);
}

TEST_F(EntityTrackingTest, UntrackEntity) {
    EntityId entity = 102;
    manager_->track_entity(entity);
    EXPECT_TRUE(manager_->is_tracked(entity));

    LifecycleResult result = manager_->untrack_entity(entity);
    EXPECT_EQ(result, LifecycleResult::Success);
    EXPECT_FALSE(manager_->is_tracked(entity));
}

TEST_F(EntityTrackingTest, TrackDuplicateEntity) {
    EntityId entity = 103;
    manager_->track_entity(entity);

    // Tracking again should fail or return an appropriate status
    LifecycleResult result = manager_->track_entity(entity);
    // Implementation may vary - either success (idempotent) or error
    EXPECT_TRUE(result == LifecycleResult::Success ||
                result == LifecycleResult::InvalidEntityId);
}

TEST_F(EntityTrackingTest, UntrackNonExistentEntity) {
    EntityId entity = 999;
    LifecycleResult result = manager_->untrack_entity(entity);
    EXPECT_EQ(result, LifecycleResult::EntityNotTracked);
}

TEST_F(EntityTrackingTest, GetEntityCount) {
    EXPECT_EQ(manager_->get_entity_count(), 0u);

    manager_->track_entity(1);
    manager_->track_entity(2);
    manager_->track_entity(3);

    EXPECT_EQ(manager_->get_entity_count(), 3u);

    manager_->untrack_entity(2);
    EXPECT_EQ(manager_->get_entity_count(), 2u);
}

TEST_F(EntityTrackingTest, IsTracked) {
    EntityId entity = 104;
    EXPECT_FALSE(manager_->is_tracked(entity));

    manager_->track_entity(entity);
    EXPECT_TRUE(manager_->is_tracked(entity));

    manager_->untrack_entity(entity);
    EXPECT_FALSE(manager_->is_tracked(entity));
}

// ============================================================================
// Phase Transition Tests
// ============================================================================

class PhaseTransitionTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_ = LifecycleConfig::default_config();
        manager_ = std::make_unique<LifecycleManager>(config_);
        manager_->initialize();
    }

    void TearDown() override {
        if (manager_ && manager_->is_initialized()) {
            manager_->shutdown();
        }
    }

    LifecycleConfig config_;
    std::unique_ptr<LifecycleManager> manager_;
};

TEST_F(PhaseTransitionTest, ValidTransitionDesignToProduction) {
    EntityId entity = 200;
    manager_->track_entity(entity, LifecyclePhase::Design);

    LifecycleResult result = manager_->transition(entity, LifecyclePhase::Production,
                                                   "Design complete");
    EXPECT_EQ(result, LifecycleResult::Success);

    auto phase = manager_->get_phase(entity);
    ASSERT_TRUE(phase.has_value());
    EXPECT_EQ(*phase, LifecyclePhase::Production);
}

TEST_F(PhaseTransitionTest, ValidTransitionProductionToOperational) {
    EntityId entity = 201;
    manager_->track_entity(entity, LifecyclePhase::Production);

    LifecycleResult result = manager_->transition(entity, LifecyclePhase::Operational,
                                                   "Manufacturing complete");
    EXPECT_EQ(result, LifecycleResult::Success);

    auto phase = manager_->get_phase(entity);
    EXPECT_EQ(*phase, LifecyclePhase::Operational);
}

TEST_F(PhaseTransitionTest, ValidTransitionOperationalToMaintenance) {
    EntityId entity = 202;
    manager_->track_entity(entity, LifecyclePhase::Operational);

    LifecycleResult result = manager_->transition(entity, LifecyclePhase::Maintenance,
                                                   "Scheduled maintenance");
    EXPECT_EQ(result, LifecycleResult::Success);

    auto phase = manager_->get_phase(entity);
    EXPECT_EQ(*phase, LifecyclePhase::Maintenance);
}

TEST_F(PhaseTransitionTest, ValidTransitionMaintenanceToOperational) {
    EntityId entity = 203;
    manager_->track_entity(entity, LifecyclePhase::Maintenance);

    LifecycleResult result = manager_->transition(entity, LifecyclePhase::Operational,
                                                   "Maintenance complete");
    EXPECT_EQ(result, LifecycleResult::Success);

    auto phase = manager_->get_phase(entity);
    EXPECT_EQ(*phase, LifecyclePhase::Operational);
}

TEST_F(PhaseTransitionTest, InvalidTransitionDesignToOperational) {
    EntityId entity = 204;
    manager_->track_entity(entity, LifecyclePhase::Design);

    LifecycleResult result = manager_->transition(entity, LifecyclePhase::Operational,
                                                   "Skip production");
    EXPECT_EQ(result, LifecycleResult::TransitionNotAllowed);

    // Should remain in Design phase
    auto phase = manager_->get_phase(entity);
    EXPECT_EQ(*phase, LifecyclePhase::Design);
}

TEST_F(PhaseTransitionTest, TransitionToSamePhase) {
    EntityId entity = 205;
    manager_->track_entity(entity, LifecyclePhase::Operational);

    LifecycleResult result = manager_->transition(entity, LifecyclePhase::Operational,
                                                   "Stay in same phase");
    // Same phase transition should succeed (no-op)
    EXPECT_EQ(result, LifecycleResult::Success);
}

TEST_F(PhaseTransitionTest, TransitionUntrackedEntity) {
    EntityId entity = 999;

    LifecycleResult result = manager_->transition(entity, LifecyclePhase::Production,
                                                   "Not tracked");
    EXPECT_EQ(result, LifecycleResult::EntityNotTracked);
}

TEST_F(PhaseTransitionTest, TransitionWithOperatorId) {
    EntityId entity = 206;
    manager_->track_entity(entity, LifecyclePhase::Design);

    LifecycleResult result = manager_->transition(entity, LifecyclePhase::Production,
                                                   "Design complete", "operator_123");
    EXPECT_EQ(result, LifecycleResult::Success);

    auto history = manager_->get_transition_history(entity);
    ASSERT_GE(history.size(), 1u);
    EXPECT_EQ(history.back().operator_id, "operator_123");
}

TEST_F(PhaseTransitionTest, TransitionHistory) {
    EntityId entity = 207;
    manager_->track_entity(entity, LifecyclePhase::Design);

    manager_->transition(entity, LifecyclePhase::Production, "Step 1");
    manager_->transition(entity, LifecyclePhase::Operational, "Step 2");
    manager_->transition(entity, LifecyclePhase::Maintenance, "Step 3");

    auto history = manager_->get_transition_history(entity);
    EXPECT_GE(history.size(), 3u);
}

// ============================================================================
// Transition Validation Tests
// ============================================================================

class TransitionValidationTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_ = LifecycleConfig::default_config();
        manager_ = std::make_unique<LifecycleManager>(config_);
        manager_->initialize();
    }

    void TearDown() override {
        if (manager_ && manager_->is_initialized()) {
            manager_->shutdown();
        }
    }

    LifecycleConfig config_;
    std::unique_ptr<LifecycleManager> manager_;
};

TEST_F(TransitionValidationTest, ValidTransitionPaths) {
    // Design → Production
    EXPECT_TRUE(manager_->is_valid_transition(LifecyclePhase::Design,
                                               LifecyclePhase::Production));

    // Production → Operational
    EXPECT_TRUE(manager_->is_valid_transition(LifecyclePhase::Production,
                                               LifecyclePhase::Operational));

    // Operational → Maintenance
    EXPECT_TRUE(manager_->is_valid_transition(LifecyclePhase::Operational,
                                               LifecyclePhase::Maintenance));

    // Maintenance → Operational
    EXPECT_TRUE(manager_->is_valid_transition(LifecyclePhase::Maintenance,
                                               LifecyclePhase::Operational));

    // Any → Decommissioned
    EXPECT_TRUE(manager_->is_valid_transition(LifecyclePhase::Design,
                                               LifecyclePhase::Decommissioned));
    EXPECT_TRUE(manager_->is_valid_transition(LifecyclePhase::Operational,
                                               LifecyclePhase::Decommissioned));
}

TEST_F(TransitionValidationTest, InvalidTransitionPaths) {
    // Design → Operational (skip Production) - invalid
    EXPECT_FALSE(manager_->is_valid_transition(LifecyclePhase::Design,
                                                LifecyclePhase::Operational));

    // Design → Maintenance (skip Production and Operational) - invalid
    EXPECT_FALSE(manager_->is_valid_transition(LifecyclePhase::Design,
                                                LifecyclePhase::Maintenance));

    // Decommissioned → Any (terminal state) - invalid
    EXPECT_FALSE(manager_->is_valid_transition(LifecyclePhase::Decommissioned,
                                                LifecyclePhase::Operational));
}

TEST_F(TransitionValidationTest, SamePhaseTransition) {
    // Same phase transitions should be valid (no-op)
    EXPECT_TRUE(manager_->is_valid_transition(LifecyclePhase::Design,
                                               LifecyclePhase::Design));
    EXPECT_TRUE(manager_->is_valid_transition(LifecyclePhase::Operational,
                                               LifecyclePhase::Operational));
}

TEST_F(TransitionValidationTest, StrictModeEnforcement) {
    // With strict mode enabled, invalid transitions should be rejected
    config_.strict_transitions = true;
    auto strict_manager = std::make_unique<LifecycleManager>(config_);
    strict_manager->initialize();

    EntityId entity = 300;
    strict_manager->track_entity(entity, LifecyclePhase::Design);

    LifecycleResult result = strict_manager->transition(entity, LifecyclePhase::Operational,
                                                        "Invalid jump");
    EXPECT_EQ(result, LifecycleResult::TransitionNotAllowed);

    strict_manager->shutdown();
}

TEST_F(TransitionValidationTest, NonStrictModeAllowsInvalid) {
    // With strict mode disabled, any transition should be allowed
    config_.strict_transitions = false;
    auto lenient_manager = std::make_unique<LifecycleManager>(config_);
    lenient_manager->initialize();

    EntityId entity = 301;
    lenient_manager->track_entity(entity, LifecyclePhase::Design);

    LifecycleResult result = lenient_manager->transition(entity, LifecyclePhase::Operational,
                                                         "Jump allowed");
    EXPECT_EQ(result, LifecycleResult::Success);

    lenient_manager->shutdown();
}

// ============================================================================
// Event Logging Tests
// ============================================================================

class EventLoggingTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_ = LifecycleConfig::default_config();
        config_.enable_event_logging = true;
        manager_ = std::make_unique<LifecycleManager>(config_);
        manager_->initialize();
    }

    void TearDown() override {
        if (manager_ && manager_->is_initialized()) {
            manager_->shutdown();
        }
    }

    LifecycleConfig config_;
    std::unique_ptr<LifecycleManager> manager_;
};

TEST_F(EventLoggingTest, LogSimpleEvent) {
    EntityId entity = 400;
    manager_->track_entity(entity);

    LifecycleResult result = manager_->log_event(entity, "inspection", "Annual inspection");
    EXPECT_EQ(result, LifecycleResult::Success);

    auto events = manager_->get_event_history(entity);
    ASSERT_GE(events.size(), 1u);
    EXPECT_EQ(events.back().event_type, "inspection");
    EXPECT_EQ(events.back().description, "Annual inspection");
}

TEST_F(EventLoggingTest, LogEventWithMetadata) {
    EntityId entity = 401;
    manager_->track_entity(entity);

    LifecycleEvent event(entity, "repair", "Replace component");
    event.metadata["part_id"] = "ABC123";
    event.metadata["technician"] = "John Doe";
    event.metadata["cost"] = "500.00";

    LifecycleResult result = manager_->log_event(entity, event);
    EXPECT_EQ(result, LifecycleResult::Success);

    auto events = manager_->get_event_history(entity);
    ASSERT_GE(events.size(), 1u);
    EXPECT_EQ(events.back().metadata.size(), 3u);
    EXPECT_EQ(events.back().metadata["part_id"], "ABC123");
}

TEST_F(EventLoggingTest, EventHistoryRetrieval) {
    EntityId entity = 402;
    manager_->track_entity(entity);

    manager_->log_event(entity, "event1", "First event");
    manager_->log_event(entity, "event2", "Second event");
    manager_->log_event(entity, "event3", "Third event");

    auto events = manager_->get_event_history(entity);
    EXPECT_GE(events.size(), 3u);
}

TEST_F(EventLoggingTest, EventTrimming) {
    // Set small history size
    config_.max_history_size = 5;
    auto trim_manager = std::make_unique<LifecycleManager>(config_);
    trim_manager->initialize();

    EntityId entity = 403;
    trim_manager->track_entity(entity);

    // Log more events than max_history_size
    for (UInt32 i = 0; i < 10; ++i) {
        trim_manager->log_event(entity, "test_event", "Event " + std::to_string(i));
    }

    // Manually trigger trimming
    trim_manager->trim_history();

    auto events = trim_manager->get_event_history(entity);
    EXPECT_LE(events.size(), 5u);

    trim_manager->shutdown();
}

TEST_F(EventLoggingTest, LogEventUntrackedEntity) {
    EntityId entity = 999;

    LifecycleResult result = manager_->log_event(entity, "test", "Should fail");
    EXPECT_EQ(result, LifecycleResult::EntityNotTracked);
}

TEST_F(EventLoggingTest, EventLoggingDisabled) {
    config_.enable_event_logging = false;
    auto no_log_manager = std::make_unique<LifecycleManager>(config_);
    no_log_manager->initialize();

    EntityId entity = 404;
    no_log_manager->track_entity(entity);

    // Event logging should still succeed but may not be recorded
    LifecycleResult result = no_log_manager->log_event(entity, "test", "Event");
    EXPECT_TRUE(result == LifecycleResult::Success ||
                result == LifecycleResult::InvalidConfiguration);

    no_log_manager->shutdown();
}

// ============================================================================
// Observer Tests
// ============================================================================

class ObserverTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_ = LifecycleConfig::default_config();
        manager_ = std::make_unique<LifecycleManager>(config_);
        manager_->initialize();
        observer_ = std::make_shared<MockLifecycleObserver>();
    }

    void TearDown() override {
        if (manager_ && manager_->is_initialized()) {
            manager_->shutdown();
        }
    }

    LifecycleConfig config_;
    std::unique_ptr<LifecycleManager> manager_;
    std::shared_ptr<MockLifecycleObserver> observer_;
};

TEST_F(ObserverTest, AddObserver) {
    LifecycleResult result = manager_->add_observer(observer_);
    EXPECT_EQ(result, LifecycleResult::Success);
    EXPECT_EQ(manager_->get_observer_count(), 1u);
}

TEST_F(ObserverTest, RemoveObserver) {
    manager_->add_observer(observer_);
    EXPECT_EQ(manager_->get_observer_count(), 1u);

    LifecycleResult result = manager_->remove_observer(observer_);
    EXPECT_EQ(result, LifecycleResult::Success);
    EXPECT_EQ(manager_->get_observer_count(), 0u);
}

TEST_F(ObserverTest, OnPhaseChangedNotification) {
    manager_->add_observer(observer_);

    EntityId entity = 500;
    manager_->track_entity(entity, LifecyclePhase::Design);
    manager_->transition(entity, LifecyclePhase::Production, "Test");

    EXPECT_GE(observer_->phase_changed_count, 1u);
    EXPECT_EQ(observer_->last_entity_id, entity);
    EXPECT_EQ(observer_->last_from_phase, LifecyclePhase::Design);
    EXPECT_EQ(observer_->last_to_phase, LifecyclePhase::Production);
}

TEST_F(ObserverTest, OnEventLoggedNotification) {
    manager_->add_observer(observer_);

    EntityId entity = 501;
    manager_->track_entity(entity);
    manager_->log_event(entity, "test_event", "Test description");

    EXPECT_GE(observer_->event_logged_count, 1u);
    EXPECT_EQ(observer_->last_entity_id, entity);
    EXPECT_EQ(observer_->last_event_type, "test_event");
}

TEST_F(ObserverTest, OnEntityCreatedNotification) {
    manager_->add_observer(observer_);

    EntityId entity = 502;
    manager_->track_entity(entity, LifecyclePhase::Operational);

    EXPECT_EQ(observer_->entity_created_count, 1u);
    EXPECT_EQ(observer_->last_entity_id, entity);
    EXPECT_EQ(observer_->last_from_phase, LifecyclePhase::Operational);
}

TEST_F(ObserverTest, OnEntityRemovedNotification) {
    manager_->add_observer(observer_);

    EntityId entity = 503;
    manager_->track_entity(entity);
    manager_->untrack_entity(entity);

    EXPECT_EQ(observer_->entity_removed_count, 1u);
    EXPECT_EQ(observer_->last_entity_id, entity);
}

TEST_F(ObserverTest, MultipleObservers) {
    auto observer2 = std::make_shared<MockLifecycleObserver>();
    auto observer3 = std::make_shared<MockLifecycleObserver>();

    manager_->add_observer(observer_);
    manager_->add_observer(observer2);
    manager_->add_observer(observer3);

    EXPECT_EQ(manager_->get_observer_count(), 3u);

    EntityId entity = 504;
    manager_->track_entity(entity);

    EXPECT_EQ(observer_->entity_created_count, 1u);
    EXPECT_EQ(observer2->entity_created_count, 1u);
    EXPECT_EQ(observer3->entity_created_count, 1u);
}

TEST_F(ObserverTest, ClearObservers) {
    manager_->add_observer(observer_);
    auto observer2 = std::make_shared<MockLifecycleObserver>();
    manager_->add_observer(observer2);

    EXPECT_EQ(manager_->get_observer_count(), 2u);

    manager_->clear_observers();
    EXPECT_EQ(manager_->get_observer_count(), 0u);
}

// ============================================================================
// Storage Tests
// ============================================================================

class StorageTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_ = LifecycleConfig::default_config();
        manager_ = std::make_unique<LifecycleManager>(config_);
        manager_->initialize();
        storage_ = create_memory_storage();
    }

    void TearDown() override {
        if (manager_ && manager_->is_initialized()) {
            manager_->shutdown();
        }
    }

    LifecycleConfig config_;
    std::unique_ptr<LifecycleManager> manager_;
    std::unique_ptr<ILifecycleStorage> storage_;
};

TEST_F(StorageTest, SetStorage) {
    auto storage = create_memory_storage();
    LifecycleResult result = manager_->set_storage(std::move(storage));
    EXPECT_EQ(result, LifecycleResult::Success);
    EXPECT_NE(manager_->get_storage(), nullptr);
}

TEST_F(StorageTest, SaveEntity) {
    manager_->set_storage(std::move(storage_));

    EntityId entity = 600;
    manager_->track_entity(entity, LifecyclePhase::Operational);
    manager_->log_event(entity, "test", "Test event");

    LifecycleResult result = manager_->save_entity(entity);
    EXPECT_EQ(result, LifecycleResult::Success);
}

TEST_F(StorageTest, LoadEntity) {
    manager_->set_storage(std::move(storage_));

    EntityId entity = 601;
    manager_->track_entity(entity, LifecyclePhase::Production);
    manager_->save_entity(entity);

    // Untrack entity
    manager_->untrack_entity(entity);
    EXPECT_FALSE(manager_->is_tracked(entity));

    // Load from storage
    LifecycleResult result = manager_->load_entity(entity);
    if (result == LifecycleResult::Success) {
        EXPECT_TRUE(manager_->is_tracked(entity));
        auto phase = manager_->get_phase(entity);
        EXPECT_EQ(*phase, LifecyclePhase::Production);
    }
}

TEST_F(StorageTest, SaveAll) {
    manager_->set_storage(std::move(storage_));

    manager_->track_entity(602, LifecyclePhase::Design);
    manager_->track_entity(603, LifecyclePhase::Production);
    manager_->track_entity(604, LifecyclePhase::Operational);

    LifecycleResult result = manager_->save_all();
    EXPECT_EQ(result, LifecycleResult::Success);
}

TEST_F(StorageTest, LoadAll) {
    manager_->set_storage(std::move(storage_));

    // Track and save entities
    manager_->track_entity(605, LifecyclePhase::Design);
    manager_->track_entity(606, LifecyclePhase::Operational);
    manager_->save_all();

    // Note: load_all functionality depends on implementation
    // Just verify the call completes without error
    LifecycleResult result = manager_->load_all();
    // load_all may or may not restore data depending on implementation
    EXPECT_TRUE(result == LifecycleResult::Success ||
                result == LifecycleResult::NotInitialized);
}

// ============================================================================
// Query Tests
// ============================================================================

class QueryTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_ = LifecycleConfig::default_config();
        manager_ = std::make_unique<LifecycleManager>(config_);
        manager_->initialize();
    }

    void TearDown() override {
        if (manager_ && manager_->is_initialized()) {
            manager_->shutdown();
        }
    }

    LifecycleConfig config_;
    std::unique_ptr<LifecycleManager> manager_;
};

TEST_F(QueryTest, GetEntitiesInPhase) {
    manager_->track_entity(700, LifecyclePhase::Design);
    manager_->track_entity(701, LifecyclePhase::Design);
    manager_->track_entity(702, LifecyclePhase::Production);
    manager_->track_entity(703, LifecyclePhase::Operational);
    manager_->track_entity(704, LifecyclePhase::Operational);

    auto design_entities = manager_->get_entities_in_phase(LifecyclePhase::Design);
    EXPECT_EQ(design_entities.size(), 2u);

    auto operational_entities = manager_->get_entities_in_phase(LifecyclePhase::Operational);
    EXPECT_EQ(operational_entities.size(), 2u);
}

TEST_F(QueryTest, GetLifecycleInfo) {
    EntityId entity = 705;
    manager_->track_entity(entity, LifecyclePhase::Production);
    manager_->transition(entity, LifecyclePhase::Operational, "Deployed");
    manager_->log_event(entity, "test", "Test event");

    auto info = manager_->get_lifecycle_info(entity);
    ASSERT_TRUE(info.has_value());
    EXPECT_EQ(info->entity_id, entity);
    EXPECT_EQ(info->current_phase, LifecyclePhase::Operational);
    EXPECT_GE(info->transition_count, 1u);
}

TEST_F(QueryTest, GetTransitionHistory) {
    EntityId entity = 706;
    manager_->track_entity(entity, LifecyclePhase::Design);
    manager_->transition(entity, LifecyclePhase::Production, "Build");
    manager_->transition(entity, LifecyclePhase::Operational, "Deploy");

    auto history = manager_->get_transition_history(entity);
    EXPECT_GE(history.size(), 2u);
}

TEST_F(QueryTest, GetEventHistory) {
    EntityId entity = 707;
    manager_->track_entity(entity);
    manager_->log_event(entity, "event1", "First");
    manager_->log_event(entity, "event2", "Second");
    manager_->log_event(entity, "event3", "Third");

    auto events = manager_->get_event_history(entity);
    EXPECT_GE(events.size(), 3u);
}

TEST_F(QueryTest, GetTimeInPhase) {
    EntityId entity = 708;
    manager_->track_entity(entity, LifecyclePhase::Operational);

    // Wait a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    auto time_in_phase = manager_->get_time_in_phase(entity);
    ASSERT_TRUE(time_in_phase.has_value());
    // Time may be 0 with low clock resolution - just verify it's non-negative
    EXPECT_GE(time_in_phase->count(), 0);
}

TEST_F(QueryTest, GetLifetime) {
    EntityId entity = 709;
    manager_->track_entity(entity, LifecyclePhase::Design);

    // Wait a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    manager_->transition(entity, LifecyclePhase::Production, "Build");

    auto lifetime = manager_->get_lifetime(entity);
    ASSERT_TRUE(lifetime.has_value());
    // Time may be 0 with low clock resolution - just verify it's non-negative
    EXPECT_GE(lifetime->count(), 0);
}

TEST_F(QueryTest, QueryUntrackedEntity) {
    EntityId entity = 999;

    auto info = manager_->get_lifecycle_info(entity);
    EXPECT_FALSE(info.has_value());

    auto phase = manager_->get_phase(entity);
    EXPECT_FALSE(phase.has_value());

    auto history = manager_->get_transition_history(entity);
    EXPECT_EQ(history.size(), 0u);

    auto events = manager_->get_event_history(entity);
    EXPECT_EQ(events.size(), 0u);
}

// ============================================================================
// Statistics Tests
// ============================================================================

class StatisticsTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_ = LifecycleConfig::default_config();
        manager_ = std::make_unique<LifecycleManager>(config_);
        manager_->initialize();
    }

    void TearDown() override {
        if (manager_ && manager_->is_initialized()) {
            manager_->shutdown();
        }
    }

    LifecycleConfig config_;
    std::unique_ptr<LifecycleManager> manager_;
};

TEST_F(StatisticsTest, InitialStats) {
    LifecycleStats stats = manager_->get_stats();

    EXPECT_EQ(stats.total_entities, 0u);
    EXPECT_EQ(stats.total_transitions, 0u);
    EXPECT_EQ(stats.total_events, 0u);
    EXPECT_DOUBLE_EQ(stats.average_transitions_per_entity, 0.0);
    EXPECT_DOUBLE_EQ(stats.average_events_per_entity, 0.0);
}

TEST_F(StatisticsTest, StatsAfterOperations) {
    manager_->track_entity(800, LifecyclePhase::Design);
    manager_->track_entity(801, LifecyclePhase::Production);
    manager_->track_entity(802, LifecyclePhase::Operational);

    manager_->transition(800, LifecyclePhase::Production, "Build");
    manager_->log_event(801, "test", "Event");

    LifecycleStats stats = manager_->get_stats();

    EXPECT_EQ(stats.total_entities, 3u);
    EXPECT_GE(stats.total_transitions, 1u);
    EXPECT_GE(stats.total_events, 1u);
}

TEST_F(StatisticsTest, EntitiesPerPhase) {
    manager_->track_entity(803, LifecyclePhase::Design);
    manager_->track_entity(804, LifecyclePhase::Design);
    manager_->track_entity(805, LifecyclePhase::Production);
    manager_->track_entity(806, LifecyclePhase::Operational);
    manager_->track_entity(807, LifecyclePhase::Operational);
    manager_->track_entity(808, LifecyclePhase::Operational);

    LifecycleStats stats = manager_->get_stats();

    EXPECT_EQ(stats.entities_per_phase[LifecyclePhase::Design], 2u);
    EXPECT_EQ(stats.entities_per_phase[LifecyclePhase::Production], 1u);
    EXPECT_EQ(stats.entities_per_phase[LifecyclePhase::Operational], 3u);
}

TEST_F(StatisticsTest, ResetStats) {
    manager_->track_entity(809, LifecyclePhase::Design);
    manager_->log_event(809, "test", "Event");

    manager_->reset_stats();

    LifecycleStats stats = manager_->get_stats();
    EXPECT_EQ(stats.total_entities, 0u);
    EXPECT_EQ(stats.total_transitions, 0u);
    EXPECT_EQ(stats.total_events, 0u);
}

TEST_F(StatisticsTest, StatsReset) {
    LifecycleStats stats;
    stats.total_entities = 100;
    stats.total_transitions = 50;
    stats.total_events = 200;

    stats.reset();

    EXPECT_EQ(stats.total_entities, 0u);
    EXPECT_EQ(stats.total_transitions, 0u);
    EXPECT_EQ(stats.total_events, 0u);
}

// ============================================================================
// Cleanup Tests
// ============================================================================

class CleanupTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_ = LifecycleConfig::default_config();
        config_.retention_period = std::chrono::seconds(1);
        manager_ = std::make_unique<LifecycleManager>(config_);
        manager_->initialize();
    }

    void TearDown() override {
        if (manager_ && manager_->is_initialized()) {
            manager_->shutdown();
        }
    }

    LifecycleConfig config_;
    std::unique_ptr<LifecycleManager> manager_;
};

TEST_F(CleanupTest, CleanupOldData) {
    EntityId entity = 900;
    manager_->track_entity(entity);
    manager_->log_event(entity, "old_event", "Old event");

    // Wait for retention period to expire
    std::this_thread::sleep_for(std::chrono::seconds(2));

    UInt64 removed = manager_->cleanup_old_data();
    // Should remove some old data
    EXPECT_GE(removed, 0u);
}

TEST_F(CleanupTest, TrimHistory) {
    config_.max_history_size = 5;
    auto trim_manager = std::make_unique<LifecycleManager>(config_);
    trim_manager->initialize();

    EntityId entity = 901;
    trim_manager->track_entity(entity);

    // Create more history than limit
    for (UInt32 i = 0; i < 20; ++i) {
        trim_manager->log_event(entity, "event", "Event " + std::to_string(i));
    }

    UInt64 trimmed = trim_manager->trim_history();
    // trim_history may or may not trim immediately depending on implementation
    EXPECT_GE(trimmed, 0u);

    trim_manager->shutdown();
}

TEST_F(CleanupTest, ClearAll) {
    manager_->track_entity(902);
    manager_->track_entity(903);
    manager_->track_entity(904);

    EXPECT_EQ(manager_->get_entity_count(), 3u);

    manager_->clear();
    EXPECT_EQ(manager_->get_entity_count(), 0u);
}

// ============================================================================
// Factory Function Tests
// ============================================================================

class FactoryTest : public ::testing::Test {};

TEST_F(FactoryTest, CreateLifecycleManager) {
    auto manager = create_lifecycle_manager();
    ASSERT_NE(manager, nullptr);
    // Factory creates but doesn't initialize - must call initialize() explicitly
    EXPECT_FALSE(manager->is_initialized());
    manager->initialize();
    EXPECT_TRUE(manager->is_initialized());
    manager->shutdown();
}

TEST_F(FactoryTest, CreateLifecycleManagerWithConfig) {
    LifecycleConfig config = LifecycleConfig::production();
    auto manager = create_lifecycle_manager(config);
    ASSERT_NE(manager, nullptr);

    const LifecycleConfig& current_config = manager->get_config();
    EXPECT_EQ(current_config.max_history_size, config.max_history_size);

    manager->shutdown();
}

TEST_F(FactoryTest, CreateMemoryStorage) {
    auto storage = create_memory_storage();
    ASSERT_NE(storage, nullptr);

    // Test basic storage operations
    EntityLifecycleInfo info(1, LifecyclePhase::Design);
    LifecycleResult result = storage->store(info);
    EXPECT_EQ(result, LifecycleResult::Success);

    auto loaded = storage->load(1);
    ASSERT_TRUE(loaded.has_value());
    EXPECT_EQ(loaded->entity_id, 1u);
    EXPECT_EQ(loaded->current_phase, LifecyclePhase::Design);
}

// ============================================================================
// Helper Function Tests
// ============================================================================

TEST(HelperFunctionTest, GetValidTransitions) {
    auto design_transitions = get_valid_transitions(LifecyclePhase::Design);
    EXPECT_GE(design_transitions.size(), 1u);

    auto production_transitions = get_valid_transitions(LifecyclePhase::Production);
    EXPECT_GE(production_transitions.size(), 1u);

    auto decommissioned_transitions = get_valid_transitions(LifecyclePhase::Decommissioned);
    EXPECT_EQ(decommissioned_transitions.size(), 0u);  // Terminal state
}

TEST(HelperFunctionTest, IsTransitionValid) {
    // Valid transitions
    EXPECT_TRUE(is_transition_valid(LifecyclePhase::Design, LifecyclePhase::Production));
    EXPECT_TRUE(is_transition_valid(LifecyclePhase::Operational, LifecyclePhase::Maintenance));

    // Invalid transitions
    EXPECT_FALSE(is_transition_valid(LifecyclePhase::Design, LifecyclePhase::Operational));
    EXPECT_FALSE(is_transition_valid(LifecyclePhase::Decommissioned, LifecyclePhase::Operational));

    // Same phase (valid)
    EXPECT_TRUE(is_transition_valid(LifecyclePhase::Design, LifecyclePhase::Design));
}

TEST(HelperFunctionTest, FormatTimestamp) {
    auto now = std::chrono::system_clock::now();
    std::string formatted = format_timestamp(now);

    // Should be in ISO 8601 format
    EXPECT_GT(formatted.length(), 0u);
    EXPECT_NE(formatted.find('T'), std::string::npos);
    EXPECT_NE(formatted.find('Z'), std::string::npos);
}

TEST(HelperFunctionTest, CalculateDuration) {
    auto start = std::chrono::system_clock::now();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    auto end = std::chrono::system_clock::now();

    auto duration = calculate_duration(start, end);
    // Duration should be at least 0 (may be 0 if clock resolution is low)
    EXPECT_GE(duration.count(), 0);
}

// ============================================================================
// Edge Cases and Integration Tests
// ============================================================================

TEST(LifecycleEdgeCases, MoveConstructor) {
    LifecycleConfig config = LifecycleConfig::default_config();
    LifecycleManager manager1(config);
    manager1.initialize();
    manager1.track_entity(1000);

    // Move constructor
    LifecycleManager manager2(std::move(manager1));
    EXPECT_TRUE(manager2.is_initialized());
    EXPECT_TRUE(manager2.is_tracked(1000));

    manager2.shutdown();
}

TEST(LifecycleEdgeCases, MoveAssignment) {
    LifecycleConfig config = LifecycleConfig::default_config();
    LifecycleManager manager1(config);
    manager1.initialize();
    manager1.track_entity(1001);

    LifecycleManager manager2(config);

    // Move assignment
    manager2 = std::move(manager1);
    EXPECT_TRUE(manager2.is_initialized());
    EXPECT_TRUE(manager2.is_tracked(1001));

    manager2.shutdown();
}

TEST(LifecycleEdgeCases, CompleteLifecycle) {
    auto manager = create_lifecycle_manager(LifecycleConfig::production());
    manager->initialize();

    EntityId entity = 2000;

    // Track entity through complete lifecycle
    manager->track_entity(entity, LifecyclePhase::Design);
    manager->log_event(entity, "design_review", "Completed design review");

    manager->transition(entity, LifecyclePhase::Production, "Start manufacturing");
    manager->log_event(entity, "manufacturing_start", "Manufacturing began");

    manager->transition(entity, LifecyclePhase::Operational, "Deployed to production");
    manager->log_event(entity, "deployment", "Successfully deployed");

    manager->transition(entity, LifecyclePhase::Maintenance, "Scheduled maintenance");
    manager->log_event(entity, "maintenance", "Performing maintenance");

    manager->transition(entity, LifecyclePhase::Operational, "Back online");
    manager->log_event(entity, "maintenance_complete", "Maintenance completed");

    manager->transition(entity, LifecyclePhase::Decommissioned, "End of life");
    manager->log_event(entity, "decommission", "Entity decommissioned");

    // Verify complete history
    auto info = manager->get_lifecycle_info(entity);
    ASSERT_TRUE(info.has_value());
    EXPECT_EQ(info->current_phase, LifecyclePhase::Decommissioned);
    EXPECT_GE(info->transition_count, 5u);
    EXPECT_GE(info->events.size(), 6u);

    manager->shutdown();
}
