/**
 * @file test_events.cpp
 * @brief Unit tests for JaguarEngine event system
 */

#include <gtest/gtest.h>
#include "jaguar/events/event.h"
#include "jaguar/events/event_dispatcher.h"
#include "jaguar/events/threshold_monitor.h"
#include "jaguar/events/script_callback.h"
#include <vector>
#include <string>

using namespace jaguar;
using namespace jaguar::events;

// ============================================================================
// Event Type Tests
// ============================================================================

TEST(EventTypeTest, GetEventCategory) {
    // System events (0-99)
    EXPECT_EQ(get_event_category(EventType::SimulationStarted), EventCategory::System);
    EXPECT_EQ(get_event_category(EventType::TimeStepCompleted), EventCategory::System);

    // Entity events (100-199)
    EXPECT_EQ(get_event_category(EventType::EntityCreated), EventCategory::Entity);
    EXPECT_EQ(get_event_category(EventType::EntityDestroyed), EventCategory::Entity);

    // Physics events (200-299)
    EXPECT_EQ(get_event_category(EventType::CollisionEnter), EventCategory::Physics);
    EXPECT_EQ(get_event_category(EventType::CollisionExit), EventCategory::Physics);

    // Domain events (300-399)
    EXPECT_EQ(get_event_category(EventType::Stall), EventCategory::Domain);
    EXPECT_EQ(get_event_category(EventType::WheelTouchdown), EventCategory::Domain);
    EXPECT_EQ(get_event_category(EventType::Capsize), EventCategory::Domain);
    EXPECT_EQ(get_event_category(EventType::OrbitInsertion), EventCategory::Domain);

    // Sensor events (400-499)
    EXPECT_EQ(get_event_category(EventType::SensorActivated), EventCategory::Sensor);
    EXPECT_EQ(get_event_category(EventType::TargetAcquired), EventCategory::Sensor);

    // Threshold events (500-599)
    EXPECT_EQ(get_event_category(EventType::ThresholdExceeded), EventCategory::Threshold);
    EXPECT_EQ(get_event_category(EventType::FuelLow), EventCategory::Threshold);

    // User events (1000+)
    EXPECT_EQ(get_event_category(EventType::UserDefined), EventCategory::User);
}

TEST(EventTypeTest, GetEventTypeName) {
    EXPECT_STREQ(get_event_type_name(EventType::SimulationStarted), "SimulationStarted");
    EXPECT_STREQ(get_event_type_name(EventType::EntityCreated), "EntityCreated");
    EXPECT_STREQ(get_event_type_name(EventType::CollisionEnter), "CollisionEnter");
    EXPECT_STREQ(get_event_type_name(EventType::Stall), "Stall");
    EXPECT_STREQ(get_event_type_name(EventType::UserDefined), "UserDefined");
}

// ============================================================================
// Event Factory Tests
// ============================================================================

TEST(EventFactoryTest, CreateEntityCreated) {
    Event event = Event::create_entity_created(42, "TestEntity", Domain::Air, 1.5);

    EXPECT_EQ(event.type(), EventType::EntityCreated);
    EXPECT_EQ(event.source(), 42u);
    EXPECT_DOUBLE_EQ(event.timestamp(), 1.5);
    EXPECT_EQ(event.priority(), EventPriority::Normal);

    ASSERT_TRUE(event.has_data<EntityEventData>());
    const auto& data = event.get_data<EntityEventData>();
    EXPECT_EQ(data.entity_id, 42u);
    EXPECT_EQ(data.entity_name, "TestEntity");
    EXPECT_EQ(data.domain, Domain::Air);
}

TEST(EventFactoryTest, CreateCollisionEnter) {
    Vec3 point(1.0, 2.0, 3.0);
    Vec3 normal(0.0, 1.0, 0.0);
    Event event = Event::create_collision_enter(1, 2, point, normal, 0.05, 2.5);

    EXPECT_EQ(event.type(), EventType::CollisionEnter);
    EXPECT_EQ(event.source(), 1u);
    EXPECT_DOUBLE_EQ(event.timestamp(), 2.5);
    EXPECT_EQ(event.priority(), EventPriority::High);

    ASSERT_TRUE(event.has_data<CollisionEventData>());
    const auto& data = event.get_data<CollisionEventData>();
    EXPECT_EQ(data.entity_A, 1u);
    EXPECT_EQ(data.entity_B, 2u);
    EXPECT_DOUBLE_EQ(data.contact_point.x, 1.0);
    EXPECT_DOUBLE_EQ(data.contact_point.y, 2.0);
    EXPECT_DOUBLE_EQ(data.contact_point.z, 3.0);
    EXPECT_DOUBLE_EQ(data.penetration_depth, 0.05);
}

TEST(EventFactoryTest, CreateThresholdExceeded) {
    Event event = Event::create_threshold_exceeded(10, "speed", 150.0, 100.0, true, 3.0);

    EXPECT_EQ(event.type(), EventType::ThresholdExceeded);
    EXPECT_EQ(event.source(), 10u);
    EXPECT_DOUBLE_EQ(event.timestamp(), 3.0);
    EXPECT_EQ(event.priority(), EventPriority::High);

    ASSERT_TRUE(event.has_data<ThresholdEventData>());
    const auto& data = event.get_data<ThresholdEventData>();
    EXPECT_EQ(data.parameter_name, "speed");
    EXPECT_DOUBLE_EQ(data.current_value, 150.0);
    EXPECT_DOUBLE_EQ(data.threshold_value, 100.0);
    EXPECT_DOUBLE_EQ(data.delta, 50.0);
    EXPECT_TRUE(data.is_upper_limit);
}

// ============================================================================
// Event Dispatcher Tests
// ============================================================================

class EventDispatcherTest : public ::testing::Test {
protected:
    EventDispatcher dispatcher;
    std::vector<EventType> received_events;

    bool handler(Event& event) {
        received_events.push_back(event.type());
        return true;  // Continue propagation
    }
};

TEST_F(EventDispatcherTest, SubscribeAndDispatch) {
    auto id = dispatcher.subscribe(EventType::EntityCreated,
        [this](Event& e) { return handler(e); },
        "test_handler");

    EXPECT_NE(id, INVALID_HANDLER_ID);
    EXPECT_EQ(dispatcher.handler_count(EventType::EntityCreated), 1u);

    Event event = Event::create_entity_created(1, "Test", Domain::Generic, 0.0);
    dispatcher.dispatch(event);

    EXPECT_EQ(received_events.size(), 1u);
    EXPECT_EQ(received_events[0], EventType::EntityCreated);
}

TEST_F(EventDispatcherTest, UnsubscribeRemovesHandler) {
    auto id = dispatcher.subscribe(EventType::EntityCreated,
        [this](Event& e) { return handler(e); },
        "test_handler");

    EXPECT_TRUE(dispatcher.unsubscribe(id));
    EXPECT_EQ(dispatcher.handler_count(EventType::EntityCreated), 0u);

    Event event = Event::create_entity_created(1, "Test", Domain::Generic, 0.0);
    dispatcher.dispatch(event);

    EXPECT_EQ(received_events.size(), 0u);
}

TEST_F(EventDispatcherTest, SubscribeCategoryReceivesMatchingEvents) {
    dispatcher.subscribe_category(EventCategory::Physics,
        [this](Event& e) { return handler(e); },
        "physics_handler");

    Event collision = Event::create_collision_enter(1, 2, Vec3(), Vec3(), 0.0, 0.0);
    Event entity = Event::create_entity_created(1, "Test", Domain::Generic, 0.0);

    dispatcher.dispatch(collision);
    dispatcher.dispatch(entity);

    EXPECT_EQ(received_events.size(), 1u);
    EXPECT_EQ(received_events[0], EventType::CollisionEnter);
}

TEST_F(EventDispatcherTest, SubscribeAllReceivesAllEvents) {
    dispatcher.subscribe_all(
        [this](Event& e) { return handler(e); },
        "all_handler");

    Event collision = Event::create_collision_enter(1, 2, Vec3(), Vec3(), 0.0, 0.0);
    Event entity = Event::create_entity_created(1, "Test", Domain::Generic, 0.0);

    dispatcher.dispatch(collision);
    dispatcher.dispatch(entity);

    EXPECT_EQ(received_events.size(), 2u);
}

TEST_F(EventDispatcherTest, EventConsumptionStopsPropagation) {
    dispatcher.subscribe(EventType::EntityCreated,
        [](Event& e) {
            e.consume();
            return false;
        },
        "consumer", 0);

    dispatcher.subscribe(EventType::EntityCreated,
        [this](Event& e) { return handler(e); },
        "second_handler", 1);

    Event event = Event::create_entity_created(1, "Test", Domain::Generic, 0.0);
    dispatcher.dispatch(event);

    EXPECT_EQ(received_events.size(), 0u);
    EXPECT_TRUE(event.is_consumed());
}

TEST_F(EventDispatcherTest, HandlerOrderRespected) {
    std::vector<int> order;

    dispatcher.subscribe(EventType::EntityCreated,
        [&order](Event&) { order.push_back(2); return true; },
        "second", 2);

    dispatcher.subscribe(EventType::EntityCreated,
        [&order](Event&) { order.push_back(1); return true; },
        "first", 1);

    dispatcher.subscribe(EventType::EntityCreated,
        [&order](Event&) { order.push_back(3); return true; },
        "third", 3);

    Event event = Event::create_entity_created(1, "Test", Domain::Generic, 0.0);
    dispatcher.dispatch(event);

    ASSERT_EQ(order.size(), 3u);
    EXPECT_EQ(order[0], 1);
    EXPECT_EQ(order[1], 2);
    EXPECT_EQ(order[2], 3);
}

// ============================================================================
// Event Queue Tests
// ============================================================================

TEST_F(EventDispatcherTest, QueueAndFlush) {
    dispatcher.subscribe(EventType::EntityCreated,
        [this](Event& e) { return handler(e); },
        "test_handler");

    auto event1 = Event::create_entity_created(1, "Test1", Domain::Generic, 0.0);
    auto event2 = Event::create_entity_created(2, "Test2", Domain::Generic, 0.0);
    EXPECT_TRUE(dispatcher.queue(std::move(event1)));
    EXPECT_TRUE(dispatcher.queue(std::move(event2)));

    EXPECT_EQ(dispatcher.queue_size(), 2u);
    EXPECT_EQ(received_events.size(), 0u);

    SizeT processed = dispatcher.flush_queue();

    EXPECT_EQ(processed, 2u);
    EXPECT_EQ(dispatcher.queue_size(), 0u);
    EXPECT_EQ(received_events.size(), 2u);
}

TEST_F(EventDispatcherTest, QueueRespectsPriority) {
    std::vector<EventPriority> priorities;

    dispatcher.subscribe_all(
        [&priorities](Event& e) {
            priorities.push_back(e.priority());
            return true;
        },
        "priority_checker");

    Event low;
    low.base.type = EventType::UserDefined;
    low.base.priority = EventPriority::Low;

    Event high;
    high.base.type = EventType::UserDefined;
    high.base.priority = EventPriority::High;

    Event normal;
    normal.base.type = EventType::UserDefined;
    normal.base.priority = EventPriority::Normal;

    dispatcher.queue(low);
    dispatcher.queue(high);
    dispatcher.queue(normal);

    dispatcher.flush_queue();

    ASSERT_EQ(priorities.size(), 3u);
    EXPECT_EQ(priorities[0], EventPriority::High);
    EXPECT_EQ(priorities[1], EventPriority::Normal);
    EXPECT_EQ(priorities[2], EventPriority::Low);
}

// ============================================================================
// Event Subscription RAII Tests
// ============================================================================

TEST(EventSubscriptionTest, AutomaticUnsubscription) {
    EventDispatcher dispatcher;
    std::vector<int> calls;

    {
        EventSubscription sub(dispatcher, EventType::EntityCreated,
            [&calls](Event&) { calls.push_back(1); return true; },
            "temp_handler");

        Event event = Event::create_entity_created(1, "Test", Domain::Generic, 0.0);
        dispatcher.dispatch(event);

        EXPECT_EQ(calls.size(), 1u);
        EXPECT_TRUE(sub.valid());
    }

    // After scope exit, handler should be unsubscribed
    Event event = Event::create_entity_created(2, "Test2", Domain::Generic, 0.0);
    dispatcher.dispatch(event);

    EXPECT_EQ(calls.size(), 1u);  // Should not have increased
}

TEST(EventSubscriptionTest, ReleasePreservesHandler) {
    EventDispatcher dispatcher;
    std::vector<int> calls;

    EventHandlerId id;
    {
        EventSubscription sub(dispatcher, EventType::EntityCreated,
            [&calls](Event&) { calls.push_back(1); return true; },
            "temp_handler");

        id = sub.release();
        EXPECT_FALSE(sub.valid());
    }

    // Handler should still be active after scope exit
    Event event = Event::create_entity_created(1, "Test", Domain::Generic, 0.0);
    dispatcher.dispatch(event);

    EXPECT_EQ(calls.size(), 1u);

    // Clean up manually
    dispatcher.unsubscribe(id);
}

// ============================================================================
// Event Statistics Tests
// ============================================================================

TEST_F(EventDispatcherTest, StatisticsTracking) {
    dispatcher.subscribe(EventType::EntityCreated,
        [](Event&) { return true; },
        "handler1");
    dispatcher.subscribe(EventType::EntityCreated,
        [](Event&) { return true; },
        "handler2");

    Event event = Event::create_entity_created(1, "Test", Domain::Generic, 0.0);
    dispatcher.dispatch(event);
    dispatcher.dispatch(event);

    const auto& stats = dispatcher.get_statistics();
    EXPECT_EQ(stats.total_events_dispatched, 2u);
    EXPECT_EQ(stats.total_handlers_called, 4u);  // 2 handlers * 2 events
    auto it = stats.events_by_type.find(EventType::EntityCreated);
    ASSERT_NE(it, stats.events_by_type.end());
    EXPECT_EQ(it->second, 2u);
}

// ============================================================================
// Event History Tests
// ============================================================================

TEST_F(EventDispatcherTest, EventHistory) {
    EventDispatcherConfig config;
    config.enable_event_history = true;
    config.history_size = 100;
    EventDispatcher hist_dispatcher(config);

    EXPECT_TRUE(hist_dispatcher.history_enabled());

    Event event1 = Event::create_entity_created(1, "Test1", Domain::Generic, 1.0);
    Event event2 = Event::create_collision_enter(1, 2, Vec3(), Vec3(), 0.0, 2.0);
    Event event3 = Event::create_entity_destroyed(1, "Test1", 3.0);

    hist_dispatcher.dispatch(event1);
    hist_dispatcher.dispatch(event2);
    hist_dispatcher.dispatch(event3);

    const auto& history = hist_dispatcher.get_history();
    EXPECT_EQ(history.size(), 3u);

    // Query by type
    auto entity_events = hist_dispatcher.query_history(EventType::EntityCreated);
    EXPECT_EQ(entity_events.size(), 1u);

    // Query by category
    auto physics_events = hist_dispatcher.query_history(EventCategory::Physics);
    EXPECT_EQ(physics_events.size(), 1u);

    // Query by time range
    auto time_events = hist_dispatcher.query_history(1.5, 2.5);
    EXPECT_EQ(time_events.size(), 1u);
}

// ============================================================================
// Handler Enable/Disable Tests
// ============================================================================

TEST_F(EventDispatcherTest, HandlerEnableDisable) {
    auto id = dispatcher.subscribe(EventType::EntityCreated,
        [this](Event& e) { return handler(e); },
        "test_handler");

    EXPECT_TRUE(dispatcher.is_handler_enabled(id));

    Event event = Event::create_entity_created(1, "Test", Domain::Generic, 0.0);
    dispatcher.dispatch(event);
    EXPECT_EQ(received_events.size(), 1u);

    dispatcher.set_handler_enabled(id, false);
    EXPECT_FALSE(dispatcher.is_handler_enabled(id));

    dispatcher.dispatch(event);
    EXPECT_EQ(received_events.size(), 1u);  // Should not increase

    dispatcher.set_handler_enabled(id, true);
    dispatcher.dispatch(event);
    EXPECT_EQ(received_events.size(), 2u);  // Should increase
}

// ============================================================================
// Threshold Monitor Tests
// ============================================================================

class ThresholdMonitorTest : public ::testing::Test {
protected:
    EventDispatcher dispatcher;
    ThresholdMonitor monitor;
    std::vector<EventType> received_events;
    std::vector<Real> received_values;

    void SetUp() override {
        monitor.set_event_dispatcher(&dispatcher);
        monitor.set_entity_id(42);
        monitor.set_current_time(0.0);

        dispatcher.subscribe(EventType::ThresholdExceeded,
            [this](Event& e) {
                received_events.push_back(e.type());
                if (e.has_data<ThresholdEventData>()) {
                    received_values.push_back(e.get_data<ThresholdEventData>().current_value);
                }
                return true;
            },
            "threshold_exceeded_handler");

        dispatcher.subscribe(EventType::ThresholdRecovered,
            [this](Event& e) {
                received_events.push_back(e.type());
                if (e.has_data<ThresholdEventData>()) {
                    received_values.push_back(e.get_data<ThresholdEventData>().current_value);
                }
                return true;
            },
            "threshold_recovered_handler");
    }
};

TEST_F(ThresholdMonitorTest, UpperThresholdExceeded) {
    monitor.add_threshold("speed", ThresholdConfig{
        .parameter_name = "speed",
        .upper_threshold = 100.0,
        .lower_threshold = std::nullopt,
        .hysteresis = 5.0
    });

    // Below threshold - no event
    EXPECT_FALSE(monitor.update("speed", 90.0));
    EXPECT_EQ(received_events.size(), 0u);
    EXPECT_EQ(monitor.get_state("speed"), ThresholdState::Normal);

    // Cross upper threshold
    EXPECT_TRUE(monitor.update("speed", 105.0));
    EXPECT_EQ(received_events.size(), 1u);
    EXPECT_EQ(received_events[0], EventType::ThresholdExceeded);
    EXPECT_EQ(monitor.get_state("speed"), ThresholdState::UpperExceeded);

    // Stay above threshold - no new event
    EXPECT_FALSE(monitor.update("speed", 110.0));
    EXPECT_EQ(received_events.size(), 1u);

    // Go below threshold but within hysteresis - no recovery yet
    EXPECT_FALSE(monitor.update("speed", 98.0));  // 100 - 5 = 95 is recovery point
    EXPECT_EQ(received_events.size(), 1u);
    EXPECT_EQ(monitor.get_state("speed"), ThresholdState::UpperExceeded);

    // Go below hysteresis - recovery event
    EXPECT_TRUE(monitor.update("speed", 90.0));
    EXPECT_EQ(received_events.size(), 2u);
    EXPECT_EQ(received_events[1], EventType::ThresholdRecovered);
    EXPECT_EQ(monitor.get_state("speed"), ThresholdState::Normal);
}

TEST_F(ThresholdMonitorTest, LowerThresholdExceeded) {
    monitor.add_threshold("altitude", ThresholdConfig{
        .parameter_name = "altitude",
        .upper_threshold = std::nullopt,
        .lower_threshold = 100.0,
        .hysteresis = 10.0
    });

    // Above threshold - no event
    EXPECT_FALSE(monitor.update("altitude", 500.0));
    EXPECT_EQ(monitor.get_state("altitude"), ThresholdState::Normal);

    // Cross lower threshold
    EXPECT_TRUE(monitor.update("altitude", 80.0));
    EXPECT_EQ(received_events.size(), 1u);
    EXPECT_EQ(received_events[0], EventType::ThresholdExceeded);
    EXPECT_EQ(monitor.get_state("altitude"), ThresholdState::LowerExceeded);

    // Within hysteresis - no recovery
    EXPECT_FALSE(monitor.update("altitude", 105.0));  // 100 + 10 = 110 is recovery point
    EXPECT_EQ(monitor.get_state("altitude"), ThresholdState::LowerExceeded);

    // Above hysteresis - recovery
    EXPECT_TRUE(monitor.update("altitude", 120.0));
    EXPECT_EQ(received_events.size(), 2u);
    EXPECT_EQ(received_events[1], EventType::ThresholdRecovered);
}

TEST_F(ThresholdMonitorTest, BothThresholds) {
    monitor.add_threshold("temperature", ThresholdConfig{
        .parameter_name = "temperature",
        .upper_threshold = 100.0,
        .lower_threshold = 0.0,
        .hysteresis = 5.0
    });

    // Normal range
    EXPECT_FALSE(monitor.update("temperature", 50.0));
    EXPECT_EQ(monitor.get_state("temperature"), ThresholdState::Normal);

    // Exceed upper
    EXPECT_TRUE(monitor.update("temperature", 105.0));
    EXPECT_EQ(monitor.get_state("temperature"), ThresholdState::UpperExceeded);

    // Recover
    EXPECT_TRUE(monitor.update("temperature", 90.0));
    EXPECT_EQ(monitor.get_state("temperature"), ThresholdState::Normal);

    // Exceed lower
    EXPECT_TRUE(monitor.update("temperature", -5.0));
    EXPECT_EQ(monitor.get_state("temperature"), ThresholdState::LowerExceeded);
}

TEST_F(ThresholdMonitorTest, MultipleThresholds) {
    monitor.add_threshold("speed", speed_threshold(200.0, 10.0));
    monitor.add_threshold("fuel", fuel_threshold(20.0));
    monitor.add_threshold("temp", temperature_threshold(100.0));

    EXPECT_EQ(monitor.threshold_count(), 3u);

    // Exceed multiple thresholds
    std::unordered_map<std::string, Real> values = {
        {"speed", 210.0},
        {"fuel", 15.0},
        {"temp", 80.0}
    };

    SizeT events = monitor.update_batch(values);
    EXPECT_EQ(events, 2u);  // speed and fuel exceeded

    auto exceeded = monitor.get_exceeded_thresholds();
    EXPECT_EQ(exceeded.size(), 2u);
    EXPECT_TRUE(monitor.any_exceeded());
}

TEST_F(ThresholdMonitorTest, NoRecoveryEvent) {
    monitor.add_threshold("test", ThresholdConfig{
        .parameter_name = "test",
        .upper_threshold = 100.0,
        .lower_threshold = std::nullopt,
        .hysteresis = 5.0,
        .emit_on_recover = false  // No recovery events
    });

    monitor.update("test", 110.0);  // Exceed
    EXPECT_EQ(received_events.size(), 1u);

    monitor.update("test", 50.0);   // Recover
    EXPECT_EQ(received_events.size(), 1u);  // No recovery event
    EXPECT_EQ(monitor.get_state("test"), ThresholdState::Normal);
}

TEST_F(ThresholdMonitorTest, ResetState) {
    monitor.add_threshold("test", ThresholdConfig{
        .parameter_name = "test",
        .upper_threshold = 100.0,
        .lower_threshold = std::nullopt,
        .hysteresis = 5.0
    });

    monitor.update("test", 110.0);
    EXPECT_EQ(monitor.get_state("test"), ThresholdState::UpperExceeded);

    monitor.reset_state("test");
    EXPECT_EQ(monitor.get_state("test"), ThresholdState::Normal);
}

TEST_F(ThresholdMonitorTest, RemoveThreshold) {
    monitor.add_threshold("test", ThresholdConfig{
        .parameter_name = "test",
        .upper_threshold = 100.0,
        .lower_threshold = std::nullopt,
        .hysteresis = 5.0
    });

    EXPECT_EQ(monitor.threshold_count(), 1u);
    EXPECT_TRUE(monitor.remove_threshold("test"));
    EXPECT_EQ(monitor.threshold_count(), 0u);
    EXPECT_FALSE(monitor.remove_threshold("test"));  // Already removed
}

// ============================================================================
// Domain-Specific Event Factory Tests
// ============================================================================

TEST(DomainEventFactoryTest, CreateStallEvent) {
    Event event = Event::create_stall(1, 0.3, 50.0, 3000.0, 0.15, true, 10.5);

    EXPECT_EQ(event.type(), EventType::Stall);
    EXPECT_EQ(event.source(), 1u);
    EXPECT_DOUBLE_EQ(event.timestamp(), 10.5);
    EXPECT_EQ(event.priority(), EventPriority::Immediate);  // Critical stall

    ASSERT_TRUE(event.has_data<AirEventData>());
    const auto& data = event.get_data<AirEventData>();
    EXPECT_DOUBLE_EQ(data.angle_of_attack, 0.3);
    EXPECT_DOUBLE_EQ(data.indicated_airspeed, 50.0);
    EXPECT_DOUBLE_EQ(data.altitude, 3000.0);
    EXPECT_DOUBLE_EQ(data.mach_number, 0.15);
    EXPECT_TRUE(data.critical);
}

TEST(DomainEventFactoryTest, CreateStallRecoveryEvent) {
    Event event = Event::create_stall_recovery(1, 0.1, 80.0, 2500.0, 0.25, 15.0);

    EXPECT_EQ(event.type(), EventType::StallRecovery);
    EXPECT_EQ(event.priority(), EventPriority::Normal);

    ASSERT_TRUE(event.has_data<AirEventData>());
    const auto& data = event.get_data<AirEventData>();
    EXPECT_FALSE(data.critical);
}

TEST(DomainEventFactoryTest, CreateOverspeedEvent) {
    Event event = Event::create_overspeed(2, 350.0, 1.1, 10000.0, true, 20.0);

    EXPECT_EQ(event.type(), EventType::OverspeedWarning);
    EXPECT_EQ(event.source(), 2u);
    EXPECT_EQ(event.priority(), EventPriority::Immediate);

    ASSERT_TRUE(event.has_data<AirEventData>());
    const auto& data = event.get_data<AirEventData>();
    EXPECT_DOUBLE_EQ(data.indicated_airspeed, 350.0);
    EXPECT_DOUBLE_EQ(data.mach_number, 1.1);
}

TEST(DomainEventFactoryTest, CreateWheelTouchdownEvent) {
    Vec3 contact(100.0, 200.0, 0.0);
    Event event = Event::create_wheel_touchdown(3, 0, contact, -5.0, 1.0, 30.0);

    EXPECT_EQ(event.type(), EventType::WheelTouchdown);
    EXPECT_EQ(event.source(), 3u);
    EXPECT_EQ(event.priority(), EventPriority::High);

    ASSERT_TRUE(event.has_data<LandEventData>());
    const auto& data = event.get_data<LandEventData>();
    EXPECT_EQ(data.wheel_index, 0);
    EXPECT_DOUBLE_EQ(data.vertical_speed, -5.0);
    EXPECT_DOUBLE_EQ(data.lateral_speed, 1.0);
    EXPECT_DOUBLE_EQ(data.contact_point.x, 100.0);
}

TEST(DomainEventFactoryTest, CreateWheelLiftoffEvent) {
    Event event = Event::create_wheel_liftoff(3, 1, 35.0);

    EXPECT_EQ(event.type(), EventType::WheelLiftoff);
    EXPECT_EQ(event.priority(), EventPriority::Normal);

    ASSERT_TRUE(event.has_data<LandEventData>());
    EXPECT_EQ(event.get_data<LandEventData>().wheel_index, 1);
}

TEST(DomainEventFactoryTest, CreateVehicleRolloverEvent) {
    Vec3 pos(500.0, 600.0, 50.0);
    Event event = Event::create_vehicle_rollover(4, 1.57, pos, 40.0);

    EXPECT_EQ(event.type(), EventType::VehicleRollover);
    EXPECT_EQ(event.priority(), EventPriority::Immediate);

    ASSERT_TRUE(event.has_data<LandEventData>());
    const auto& data = event.get_data<LandEventData>();
    EXPECT_EQ(data.wheel_index, -1);  // Vehicle-wide
}

TEST(DomainEventFactoryTest, CreateCapsizeEvent) {
    Event event = Event::create_capsize(5, 1.2, 5.0, 50.0);

    EXPECT_EQ(event.type(), EventType::Capsize);
    EXPECT_EQ(event.source(), 5u);
    EXPECT_EQ(event.priority(), EventPriority::Immediate);

    ASSERT_TRUE(event.has_data<SeaEventData>());
    const auto& data = event.get_data<SeaEventData>();
    EXPECT_DOUBLE_EQ(data.heel_angle, 1.2);
    EXPECT_DOUBLE_EQ(data.wave_height, 5.0);
}

TEST(DomainEventFactoryTest, CreateGroundingEvent) {
    Vec3 impact(1000.0, 2000.0, -10.0);
    Event event = Event::create_grounding(5, impact, 8.5, 55.0);

    EXPECT_EQ(event.type(), EventType::Grounding);
    EXPECT_EQ(event.priority(), EventPriority::Immediate);

    ASSERT_TRUE(event.has_data<SeaEventData>());
    const auto& data = event.get_data<SeaEventData>();
    EXPECT_DOUBLE_EQ(data.draft, 8.5);
    EXPECT_DOUBLE_EQ(data.impact_point.x, 1000.0);
}

TEST(DomainEventFactoryTest, CreateOrbitInsertionEvent) {
    Event event = Event::create_orbit_insertion(6, 400000.0, 7800.0, 0.001, 0.9, 100.0);

    EXPECT_EQ(event.type(), EventType::OrbitInsertion);
    EXPECT_EQ(event.source(), 6u);
    EXPECT_EQ(event.priority(), EventPriority::High);

    ASSERT_TRUE(event.has_data<SpaceEventData>());
    const auto& data = event.get_data<SpaceEventData>();
    EXPECT_DOUBLE_EQ(data.altitude, 400000.0);
    EXPECT_DOUBLE_EQ(data.velocity, 7800.0);
    EXPECT_DOUBLE_EQ(data.eccentricity, 0.001);
    EXPECT_DOUBLE_EQ(data.inclination, 0.9);
}

TEST(DomainEventFactoryTest, CreateOrbitDecayEvent) {
    Event event = Event::create_orbit_decay(6, 150000.0, 7900.0, 200.0);

    EXPECT_EQ(event.type(), EventType::OrbitDecay);
    EXPECT_EQ(event.priority(), EventPriority::High);

    ASSERT_TRUE(event.has_data<SpaceEventData>());
    const auto& data = event.get_data<SpaceEventData>();
    EXPECT_DOUBLE_EQ(data.altitude, 150000.0);
    EXPECT_DOUBLE_EQ(data.velocity, 7900.0);
}

TEST(DomainEventFactoryTest, CreateDockingEvent) {
    Event event = Event::create_docking(7, 8, 0.05, 300.0);

    EXPECT_EQ(event.type(), EventType::Docking);
    EXPECT_EQ(event.source(), 7u);
    EXPECT_EQ(event.priority(), EventPriority::High);

    ASSERT_TRUE(event.has_data<SpaceEventData>());
    const auto& data = event.get_data<SpaceEventData>();
    EXPECT_EQ(data.target_entity, 8u);
    EXPECT_DOUBLE_EQ(data.velocity, 0.05);
}

// ============================================================================
// Script Callback Tests
// ============================================================================

TEST(ScriptCallbackTest, ToScriptDataConversion) {
    Event event = Event::create_entity_created(42, "TestEntity", Domain::Air, 10.5);

    ScriptEventData data = to_script_data(event);

    EXPECT_EQ(data.type, EventType::EntityCreated);
    EXPECT_EQ(data.category, EventCategory::Entity);
    EXPECT_EQ(data.source_entity, 42u);
    EXPECT_DOUBLE_EQ(data.timestamp, 10.5);

    // Check entity-specific fields
    EXPECT_TRUE(data.has_field("entity_id"));
    EXPECT_TRUE(data.has_field("entity_name"));
    EXPECT_TRUE(data.has_field("domain"));

    EXPECT_EQ(data.get_field<EntityId>("entity_id"), 42u);
    EXPECT_EQ(data.get_field<std::string>("entity_name"), "TestEntity");
    EXPECT_EQ(data.get_field<int>("domain"), static_cast<int>(Domain::Air));
}

TEST(ScriptCallbackTest, CollisionDataConversion) {
    Vec3 point(1.0, 2.0, 3.0);
    Vec3 normal(0.0, 1.0, 0.0);
    Event event = Event::create_collision_enter(1, 2, point, normal, 0.05, 5.0);

    ScriptEventData data = to_script_data(event);

    EXPECT_EQ(data.type, EventType::CollisionEnter);
    EXPECT_DOUBLE_EQ(data.get_field<Real>("contact_x"), 1.0);
    EXPECT_DOUBLE_EQ(data.get_field<Real>("contact_y"), 2.0);
    EXPECT_DOUBLE_EQ(data.get_field<Real>("contact_z"), 3.0);
    EXPECT_DOUBLE_EQ(data.get_field<Real>("penetration"), 0.05);
}

TEST(ScriptCallbackTest, LambdaCallbackInvocation) {
    std::vector<EventType> received;

    auto callback = make_callback(
        [&received](const ScriptEventData& data) {
            received.push_back(data.type);
            return true;
        },
        "test_lambda"
    );

    EXPECT_TRUE(callback->is_valid());
    EXPECT_EQ(callback->get_name(), "test_lambda");

    Event event = Event::create_entity_created(1, "Test", Domain::Generic, 0.0);
    ScriptEventData data = to_script_data(event);

    EXPECT_TRUE(callback->invoke(data));
    EXPECT_EQ(received.size(), 1u);
    EXPECT_EQ(received[0], EventType::EntityCreated);
}

TEST(ScriptCallbackTest, CFunctionCallback) {
    static int call_count = 0;
    call_count = 0;

    auto c_handler = [](const ScriptEventData* data, void* user_data) -> bool {
        int* counter = static_cast<int*>(user_data);
        (*counter)++;
        return true;
    };

    int counter = 0;
    auto callback = make_c_callback(c_handler, &counter, "c_func");

    Event event = Event::create_entity_created(1, "Test", Domain::Generic, 0.0);
    ScriptEventData data = to_script_data(event);

    callback->invoke(data);
    EXPECT_EQ(counter, 1);

    callback->invoke(data);
    EXPECT_EQ(counter, 2);
}

class ScriptCallbackRegistryTest : public ::testing::Test {
protected:
    EventDispatcher dispatcher;
    std::vector<EventType> received_events;
};

TEST_F(ScriptCallbackRegistryTest, SubscribeAndReceive) {
    ScriptCallbackRegistry registry(dispatcher);

    auto callback = make_callback(
        [this](const ScriptEventData& data) {
            received_events.push_back(data.type);
            return true;
        },
        "registry_test"
    );

    ScriptHandlerId id = registry.subscribe(EventType::EntityCreated, callback);
    EXPECT_NE(id, INVALID_SCRIPT_HANDLER_ID);
    EXPECT_EQ(registry.callback_count(), 1u);

    Event event = Event::create_entity_created(1, "Test", Domain::Generic, 0.0);
    dispatcher.dispatch(event);

    EXPECT_EQ(received_events.size(), 1u);
    EXPECT_EQ(received_events[0], EventType::EntityCreated);
}

TEST_F(ScriptCallbackRegistryTest, SubscribeCategory) {
    ScriptCallbackRegistry registry(dispatcher);

    auto callback = make_callback(
        [this](const ScriptEventData& data) {
            received_events.push_back(data.type);
            return true;
        },
        "category_test"
    );

    registry.subscribe_category(EventCategory::Entity, callback);

    Event created = Event::create_entity_created(1, "Test", Domain::Generic, 0.0);
    Event destroyed = Event::create_entity_destroyed(1, "Test", 1.0);
    Event collision = Event::create_collision_enter(1, 2, Vec3(), Vec3(), 0.0, 0.0);

    dispatcher.dispatch(created);
    dispatcher.dispatch(destroyed);
    dispatcher.dispatch(collision);

    EXPECT_EQ(received_events.size(), 2u);  // Only entity events
}

TEST_F(ScriptCallbackRegistryTest, SubscribeAll) {
    ScriptCallbackRegistry registry(dispatcher);

    auto callback = make_callback(
        [this](const ScriptEventData& data) {
            received_events.push_back(data.type);
            return true;
        },
        "all_test"
    );

    registry.subscribe_all(callback);

    Event created = Event::create_entity_created(1, "Test", Domain::Generic, 0.0);
    Event collision = Event::create_collision_enter(1, 2, Vec3(), Vec3(), 0.0, 0.0);

    dispatcher.dispatch(created);
    dispatcher.dispatch(collision);

    EXPECT_EQ(received_events.size(), 2u);
}

TEST_F(ScriptCallbackRegistryTest, Unsubscribe) {
    ScriptCallbackRegistry registry(dispatcher);

    auto callback = make_callback(
        [this](const ScriptEventData& data) {
            received_events.push_back(data.type);
            return true;
        },
        "unsub_test"
    );

    ScriptHandlerId id = registry.subscribe(EventType::EntityCreated, callback);
    EXPECT_EQ(registry.callback_count(), 1u);

    Event event = Event::create_entity_created(1, "Test", Domain::Generic, 0.0);
    dispatcher.dispatch(event);
    EXPECT_EQ(received_events.size(), 1u);

    EXPECT_TRUE(registry.unsubscribe(id));
    EXPECT_EQ(registry.callback_count(), 0u);

    dispatcher.dispatch(event);
    EXPECT_EQ(received_events.size(), 1u);  // No increase
}

TEST(ScriptCallbackTest, SerializeEventData) {
    Event event = Event::create_entity_created(42, "TestEntity", Domain::Air, 10.5);
    ScriptEventData data = to_script_data(event);

    std::string serialized = serialize_event_data(data);

    EXPECT_TRUE(serialized.find("type=") != std::string::npos);
    EXPECT_TRUE(serialized.find("source=42") != std::string::npos);
    EXPECT_TRUE(serialized.find("entity_name=\"TestEntity\"") != std::string::npos);
}
