# Events Module Documentation

The Events module provides a thread-safe, type-safe event system for decoupled communication between simulation components. Supports immediate and deferred dispatch, priority queuing, category filtering, and handler management for reactive simulation programming.

## Headers

| Header | Purpose |
|--------|---------|
| `jaguar/events/event.h` | Event types, data structures, factory methods |
| `jaguar/events/event_dispatcher.h` | Event routing, handler management, queuing |
| `jaguar/events/threshold_monitor.h` | Parameter threshold monitoring |
| `jaguar/events/script_callback.h` | Script language event bindings |

## Event Categories (`event.h`)

```cpp
enum class EventCategory : UInt32 {
    None        = 0,
    System      = 1 << 0,   // Engine system events
    Entity      = 1 << 1,   // Entity lifecycle events
    Physics     = 1 << 2,   // Physics/collision events
    Domain      = 1 << 3,   // Domain-specific (air, land, sea, space)
    Sensor      = 1 << 4,   // Sensor measurement events
    Threshold   = 1 << 5,   // Threshold exceeded events
    User        = 1 << 6,   // User-defined events
    All         = 0xFFFFFFFF
};

// Category checking
bool is_physics = has_category(event.category(), EventCategory::Physics);
```

## Event Types

### System Events

```cpp
EventType::SimulationStarted
EventType::SimulationPaused
EventType::SimulationResumed
EventType::SimulationStopped
EventType::TimeStepCompleted
EventType::FrameCompleted
```

### Entity Events

```cpp
EventType::EntityCreated
EventType::EntityDestroyed
EventType::EntityActivated
EventType::EntityDeactivated
EventType::EntityStateChanged
EventType::ComponentAdded
EventType::ComponentRemoved
```

### Physics Events

```cpp
EventType::CollisionEnter
EventType::CollisionExit
EventType::CollisionStay
EventType::TriggerEnter
EventType::TriggerExit
EventType::ConstraintBroken
EventType::ConstraintCreated
```

### Domain Events

```cpp
// Air
EventType::Stall
EventType::StallRecovery
EventType::OverspeedWarning
EventType::EngineFlameout
EventType::EngineRestart

// Land
EventType::WheelTouchdown
EventType::WheelLiftoff
EventType::VehicleRollover

// Sea
EventType::Capsize
EventType::Grounding
EventType::Flooding
EventType::WaveImpact

// Space
EventType::OrbitInsertion
EventType::OrbitDecay
EventType::Reentry
EventType::Docking
```

### Sensor Events

```cpp
EventType::SensorActivated
EventType::SensorDeactivated
EventType::SensorUpdate
EventType::SensorFailure
EventType::TargetAcquired
EventType::TargetLost
```

### Threshold Events

```cpp
EventType::ThresholdExceeded
EventType::ThresholdRecovered
EventType::SpeedLimitExceeded
EventType::AltitudeLimitExceeded
EventType::FuelLow
EventType::FuelCritical
```

## Event Priority

```cpp
enum class EventPriority : UInt8 {
    Immediate   = 0,    // Process immediately (system-critical)
    High        = 1,    // Before normal events
    Normal      = 2,    // Standard priority
    Low         = 3,    // After normal events
    Deferred    = 4     // At end of frame
};
```

## Event Data Structures

### Base Event Data

```cpp
struct EventBase {
    EventType type;
    EventPriority priority;
    Real timestamp;             // Simulation time
    UInt64 frame_number;
    EntityId source_entity;
    bool consumed;              // Has handler consumed this event
};
```

### Collision Event Data

```cpp
struct CollisionEventData {
    EntityId entity_A;
    EntityId entity_B;
    Vec3 contact_point;
    Vec3 contact_normal;
    Real penetration_depth;
    Real impulse_magnitude;
    Real relative_velocity;
};
```

### Threshold Event Data

```cpp
struct ThresholdEventData {
    std::string parameter_name;
    Real current_value;
    Real threshold_value;
    Real delta;                 // How much exceeded
    bool is_upper_limit;        // Upper or lower limit
};
```

### Domain Event Data

```cpp
// Air domain
struct AirEventData {
    Real angle_of_attack;
    Real indicated_airspeed;
    Real altitude;
    Real mach_number;
    bool critical;
};

// Land domain
struct LandEventData {
    int wheel_index;
    Vec3 contact_point;
    Real vertical_speed;
    Real lateral_speed;
    Real ground_slope;
};

// Sea domain
struct SeaEventData {
    Real heel_angle;
    Real draft;
    Real wave_height;
    Vec3 impact_point;
};

// Space domain
struct SpaceEventData {
    Real altitude;
    Real velocity;
    Real eccentricity;
    Real inclination;
    EntityId target_entity;     // For docking events
};
```

## Event Creation

### Factory Methods

```cpp
// Entity events
Event e1 = Event::create_entity_created(entity_id, "F16", Domain::Air, sim_time);
Event e2 = Event::create_entity_destroyed(entity_id, "F16", sim_time);

// Collision events
Event e3 = Event::create_collision_enter(
    entity_a, entity_b,
    contact_point, contact_normal,
    penetration_depth,
    sim_time
);

// Threshold events
Event e4 = Event::create_threshold_exceeded(
    source_entity,
    "airspeed",     // Parameter name
    350.0,          // Current value
    300.0,          // Threshold
    true,           // Upper limit
    sim_time
);

// Air domain events
Event e5 = Event::create_stall(
    aircraft_id,
    aoa, ias, altitude, mach,
    true,           // Critical
    sim_time
);

Event e6 = Event::create_engine_flameout(
    aircraft_id,
    engine_index,
    altitude, mach,
    sim_time
);

// Land domain events
Event e7 = Event::create_wheel_touchdown(
    vehicle_id,
    wheel_index,
    contact_point,
    vertical_speed, lateral_speed,
    sim_time
);

// Space domain events
Event e8 = Event::create_orbit_insertion(
    satellite_id,
    altitude, velocity,
    eccentricity, inclination,
    sim_time
);

Event e9 = Event::create_docking(
    spacecraft_id, target_id,
    relative_velocity,
    sim_time
);
```

### Manual Creation

```cpp
Event event;
event.base.type = EventType::UserDefined;
event.base.priority = EventPriority::Normal;
event.base.timestamp = simulation_time;
event.base.source_entity = my_entity;

UserEventData data;
data.user_type_id = 1001;
data.user_type_name = "CustomEvent";
data.payload = std::any(MyCustomData{...});
event.data = data;
```

## Event Dispatcher (`event_dispatcher.h`)

### Configuration

```cpp
EventDispatcherConfig config;
config.max_queue_size = 10000;
config.max_handlers_per_type = 100;
config.enable_event_history = true;
config.history_size = 1000;
config.thread_safe = true;
config.allow_recursive_dispatch = false;

EventDispatcher dispatcher(config);
```

### Handler Registration

```cpp
// Subscribe to specific type
EventHandlerId id1 = dispatcher.subscribe(
    EventType::CollisionEnter,
    [](Event& e) {
        auto& data = e.get_data<CollisionEventData>();
        handle_collision(data.entity_A, data.entity_B, data.contact_point);
        return true;  // Continue propagation
    },
    "collision_handler",  // Debug name
    0                     // Order (lower = earlier)
);

// Subscribe to category
EventHandlerId id2 = dispatcher.subscribe_category(
    EventCategory::Physics,
    [](Event& e) {
        log_physics_event(e);
        return true;
    }
);

// Subscribe to all events
EventHandlerId id3 = dispatcher.subscribe_all(
    [](Event& e) {
        record_event(e);
        return true;
    }
);
```

### Consuming Events

```cpp
dispatcher.subscribe(EventType::CollisionEnter,
    [](Event& e) {
        auto& data = e.get_data<CollisionEventData>();

        if (should_handle(data)) {
            handle_it(data);
            e.consume();    // Stop propagation
            return false;   // Also stops propagation
        }

        return true;  // Let other handlers see it
    }
);
```

### Unsubscription

```cpp
// Manual unsubscription
dispatcher.unsubscribe(handler_id);

// Enable/disable handler
dispatcher.set_handler_enabled(handler_id, false);
bool enabled = dispatcher.is_handler_enabled(handler_id);
```

### RAII Subscription

```cpp
{
    EventSubscription sub(dispatcher, EventType::CollisionEnter, my_handler);

    // Handler is active...

}  // Handler automatically unsubscribed

// Or release without unsubscribing
EventHandlerId id = sub.release();
```

## Event Dispatch

### Immediate Dispatch

```cpp
// Dispatch immediately (synchronous)
Event event = Event::create_collision_enter(...);
dispatcher.dispatch(event);

// Move semantics
dispatcher.dispatch(std::move(event));
```

### Deferred Dispatch

```cpp
// Queue for later processing
Event event = Event::create_stall(...);
bool queued = dispatcher.queue(std::move(event));

// Process all queued events
SizeT processed = dispatcher.flush_queue();

// Process with time limit
SizeT processed = dispatcher.flush_queue_timed(2.0);  // 2ms max

// Clear queue
dispatcher.clear_queue();

// Query queue
SizeT pending = dispatcher.queue_size();
bool empty = dispatcher.queue_empty();
```

## Event History

```cpp
// Enable history
dispatcher.enable_history(true);

// Get full history
const std::vector<Event>& history = dispatcher.get_history();

// Query by type
std::vector<Event> collisions = dispatcher.query_history(EventType::CollisionEnter);

// Query by category
std::vector<Event> physics_events = dispatcher.query_history(EventCategory::Physics);

// Query by time range
std::vector<Event> recent = dispatcher.query_history(start_time, end_time);

// Clear history
dispatcher.clear_history();
```

## Statistics

```cpp
const EventStatistics& stats = dispatcher.get_statistics();

UInt64 total_dispatched = stats.total_events_dispatched;
UInt64 total_queued = stats.total_events_queued;
UInt64 dropped = stats.total_events_dropped;
Real avg_time = stats.average_dispatch_time_us;

// Per-type counts
UInt64 collision_count = stats.events_by_type[EventType::CollisionEnter];

// Reset statistics
dispatcher.reset_statistics();
```

## Global Dispatcher

```cpp
// Access global dispatcher
EventDispatcher& global = get_global_dispatcher();

// Set custom global dispatcher
auto custom = std::make_unique<EventDispatcher>(config);
set_global_dispatcher(std::move(custom));
```

## Threshold Monitor

```cpp
class ThresholdMonitor {
public:
    void add_threshold(
        const std::string& param_name,
        Real upper_limit,
        Real lower_limit,
        EntityId entity = INVALID_ENTITY_ID
    );

    void update(const std::string& param_name, Real value, Real sim_time);

    void set_dispatcher(EventDispatcher* dispatcher);
};

// Usage
ThresholdMonitor monitor;
monitor.set_dispatcher(&dispatcher);

monitor.add_threshold("airspeed", 300.0, 50.0, aircraft_id);
monitor.add_threshold("altitude", 50000.0, 0.0, aircraft_id);

// In simulation loop
monitor.update("airspeed", current_airspeed, sim_time);
// Automatically dispatches ThresholdExceeded events
```

## Typical Usage Patterns

### Collision Response

```cpp
dispatcher.subscribe(EventType::CollisionEnter,
    [&physics_system](Event& e) {
        auto& data = e.get_data<CollisionEventData>();

        // Apply impulse
        physics_system.apply_collision_impulse(
            data.entity_A, data.entity_B,
            data.contact_point, data.contact_normal,
            data.impulse_magnitude
        );

        return true;
    }
);

dispatcher.subscribe(EventType::CollisionEnter,
    [&audio_system](Event& e) {
        auto& data = e.get_data<CollisionEventData>();

        // Play collision sound
        audio_system.play_collision_sound(
            data.contact_point,
            data.impulse_magnitude
        );

        return true;
    },
    "audio_collision",
    10  // Lower priority (higher number)
);
```

### Flight Safety Monitoring

```cpp
dispatcher.subscribe(EventType::Stall,
    [](Event& e) {
        auto& data = e.get_data<AirEventData>();

        if (data.critical) {
            trigger_stall_warning();
            activate_stick_shaker();
        } else {
            show_stall_caution();
        }

        return true;
    }
);

dispatcher.subscribe(EventType::OverspeedWarning,
    [](Event& e) {
        auto& data = e.get_data<AirEventData>();

        play_overspeed_clacker();
        display_warning("OVERSPEED", data.mach_number);

        return true;
    }
);
```

### Entity Lifecycle

```cpp
dispatcher.subscribe(EventType::EntityCreated,
    [&renderer](Event& e) {
        auto& data = e.get_data<EntityEventData>();
        renderer.create_visual(data.entity_id, data.entity_name);
        return true;
    }
);

dispatcher.subscribe(EventType::EntityDestroyed,
    [&renderer](Event& e) {
        auto& data = e.get_data<EntityEventData>();
        renderer.destroy_visual(data.entity_id);
        return true;
    }
);
```

### Frame Processing

```cpp
// Game loop integration
while (running) {
    // Process input...

    // Physics step
    physics.step(dt);

    // Process immediate events from physics
    // (collisions dispatched during step)

    // Process deferred events
    dispatcher.flush_queue();

    // Render...

    // End of frame event
    dispatcher.dispatch(Event::create_frame_completed(frame_number, sim_time));
}
```

## Thread Safety

```cpp
// Thread-safe dispatcher (default)
EventDispatcherConfig config;
config.thread_safe = true;
EventDispatcher dispatcher(config);

// Can safely dispatch from multiple threads
std::thread physics_thread([&]() {
    while (running) {
        // ... physics simulation ...
        dispatcher.dispatch(collision_event);
    }
});

std::thread sensor_thread([&]() {
    while (running) {
        // ... sensor processing ...
        dispatcher.dispatch(sensor_event);
    }
});

// Main thread processes queue
while (running) {
    dispatcher.flush_queue();
    render();
}
```

## Performance Tips

1. **Use appropriate priority**: Critical events as Immediate, logging as Deferred
2. **Consume events early**: Avoid unnecessary handler calls
3. **Batch processing**: Use queue and flush_queue_timed for controlled timing
4. **Disable history in production**: Set enable_event_history = false
5. **Limit handlers per type**: Keep max_handlers_per_type reasonable
