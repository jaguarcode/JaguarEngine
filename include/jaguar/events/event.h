#pragma once
/**
 * @file event.h
 * @brief Event system types and base event structures
 *
 * This file defines the core event types and structures for the JaguarEngine
 * reactive simulation system. Events enable decoupled communication between
 * simulation components, support callbacks, and enable recording/playback.
 */

#include "jaguar/core/types.h"
#include <string>
#include <variant>
#include <chrono>
#include <any>

namespace jaguar::events {

// ============================================================================
// Event Categories
// ============================================================================

/**
 * @brief High-level event category for filtering
 */
enum class EventCategory : UInt32 {
    None        = 0,
    System      = 1 << 0,   ///< Engine system events
    Entity      = 1 << 1,   ///< Entity lifecycle events
    Physics     = 1 << 2,   ///< Physics/collision events
    Domain      = 1 << 3,   ///< Domain-specific events (air, land, sea, space)
    Sensor      = 1 << 4,   ///< Sensor measurement events
    Threshold   = 1 << 5,   ///< Threshold exceeded events
    User        = 1 << 6,   ///< User-defined events
    All         = 0xFFFFFFFF
};

inline EventCategory operator|(EventCategory a, EventCategory b) {
    return static_cast<EventCategory>(static_cast<UInt32>(a) | static_cast<UInt32>(b));
}

inline EventCategory operator&(EventCategory a, EventCategory b) {
    return static_cast<EventCategory>(static_cast<UInt32>(a) & static_cast<UInt32>(b));
}

inline bool has_category(EventCategory mask, EventCategory category) {
    return (static_cast<UInt32>(mask) & static_cast<UInt32>(category)) != 0;
}

// ============================================================================
// Event Types
// ============================================================================

/**
 * @brief Specific event type enumeration
 */
enum class EventType : UInt32 {
    // System Events (0-99)
    SimulationStarted       = 0,
    SimulationPaused        = 1,
    SimulationResumed       = 2,
    SimulationStopped       = 3,
    TimeStepCompleted       = 4,
    FrameCompleted          = 5,

    // Entity Lifecycle Events (100-199)
    EntityCreated           = 100,
    EntityDestroyed         = 101,
    EntityActivated         = 102,
    EntityDeactivated       = 103,
    EntityStateChanged      = 104,
    ComponentAdded          = 105,
    ComponentRemoved        = 106,

    // Physics/Collision Events (200-299)
    CollisionEnter          = 200,
    CollisionExit           = 201,
    CollisionStay           = 202,
    TriggerEnter            = 203,
    TriggerExit             = 204,
    ConstraintBroken        = 205,
    ConstraintCreated       = 206,
    ConstraintDestroyed     = 207,

    // Domain-Specific Events (300-399)
    // Air domain
    Stall                   = 300,
    StallRecovery           = 301,
    OverspeedWarning        = 302,
    OverspeedRecovery       = 303,
    EngineFlameout          = 304,
    EngineRestart           = 305,

    // Land domain
    WheelTouchdown          = 310,
    WheelLiftoff            = 311,
    VehicleRollover         = 312,
    TrackSlip               = 313,

    // Sea domain
    Capsize                 = 320,
    Grounding               = 321,
    Flooding                = 322,
    WaveImpact              = 323,

    // Space domain
    OrbitInsertion          = 330,
    OrbitDecay              = 331,
    Reentry                 = 332,
    Docking                 = 333,
    Undocking               = 334,

    // Sensor Events (400-499)
    SensorActivated         = 400,
    SensorDeactivated       = 401,
    SensorUpdate            = 402,
    SensorFailure           = 403,
    TargetAcquired          = 404,
    TargetLost              = 405,

    // Threshold Events (500-599)
    ThresholdExceeded       = 500,
    ThresholdRecovered      = 501,
    SpeedLimitExceeded      = 502,
    AltitudeLimitExceeded   = 503,
    TemperatureLimitExceeded = 504,
    StressLimitExceeded     = 505,
    FuelLow                 = 506,
    FuelCritical            = 507,

    // Damage Events (600-699)
    DamageReceived          = 600,
    ComponentDamaged        = 601,
    ComponentDestroyed      = 602,
    EntityDestroyed_Damage  = 603,

    // User-Defined Events (1000+)
    UserDefined             = 1000,

    Count
};

/**
 * @brief Get event category from event type
 */
EventCategory get_event_category(EventType type);

/**
 * @brief Get human-readable name for event type
 */
const char* get_event_type_name(EventType type);

// ============================================================================
// Event Priority
// ============================================================================

/**
 * @brief Event dispatch priority
 */
enum class EventPriority : UInt8 {
    Immediate   = 0,    ///< Process immediately (system-critical)
    High        = 1,    ///< Process before normal events
    Normal      = 2,    ///< Standard priority
    Low         = 3,    ///< Process after normal events
    Deferred    = 4     ///< Process at end of frame
};

// ============================================================================
// Event Data Structures
// ============================================================================

/**
 * @brief Base event data common to all events
 */
struct EventBase {
    EventType type{EventType::UserDefined};
    EventPriority priority{EventPriority::Normal};
    Real timestamp{0.0};                    ///< Simulation time when event occurred
    UInt64 frame_number{0};                 ///< Frame when event occurred
    EntityId source_entity{INVALID_ENTITY_ID}; ///< Entity that generated the event
    bool consumed{false};                   ///< Has event been consumed by a handler
};

/**
 * @brief Entity lifecycle event data
 */
struct EntityEventData {
    EntityId entity_id{INVALID_ENTITY_ID};
    std::string entity_name;
    Domain domain{Domain::Generic};
};

/**
 * @brief Collision event data
 */
struct CollisionEventData {
    EntityId entity_A{INVALID_ENTITY_ID};
    EntityId entity_B{INVALID_ENTITY_ID};
    Vec3 contact_point{Vec3::Zero()};
    Vec3 contact_normal{Vec3::UnitZ()};
    Real penetration_depth{0.0};
    Real impulse_magnitude{0.0};
    Real relative_velocity{0.0};
};

/**
 * @brief Constraint event data
 */
struct ConstraintEventData {
    UInt64 constraint_id{0};
    EntityId entity_A{INVALID_ENTITY_ID};
    EntityId entity_B{INVALID_ENTITY_ID};
    Real breaking_force{0.0};
    Real breaking_torque{0.0};
};

/**
 * @brief Air domain event data
 */
struct AirEventData {
    Real angle_of_attack{0.0};      ///< AoA at event (rad)
    Real indicated_airspeed{0.0};   ///< IAS at event (m/s)
    Real altitude{0.0};             ///< Altitude at event (m)
    Real mach_number{0.0};          ///< Mach number at event
    bool critical{false};           ///< Is situation critical
};

/**
 * @brief Land domain event data
 */
struct LandEventData {
    int wheel_index{-1};            ///< Wheel/track index (-1 for vehicle-wide)
    Vec3 contact_point{Vec3::Zero()};
    Real vertical_speed{0.0};       ///< Vertical speed at touchdown (m/s)
    Real lateral_speed{0.0};        ///< Lateral speed (m/s)
    Real ground_slope{0.0};         ///< Ground slope angle (rad)
};

/**
 * @brief Sea domain event data
 */
struct SeaEventData {
    Real heel_angle{0.0};           ///< Roll angle (rad)
    Real draft{0.0};                ///< Current draft (m)
    Real wave_height{0.0};          ///< Wave height at event (m)
    Vec3 impact_point{Vec3::Zero()};
};

/**
 * @brief Space domain event data
 */
struct SpaceEventData {
    Real altitude{0.0};             ///< Altitude (m)
    Real velocity{0.0};             ///< Orbital velocity (m/s)
    Real eccentricity{0.0};         ///< Orbital eccentricity
    Real inclination{0.0};          ///< Orbital inclination (rad)
    EntityId target_entity{INVALID_ENTITY_ID}; ///< For docking events
};

/**
 * @brief Sensor event data
 */
struct SensorEventData {
    UInt64 sensor_id{0};
    std::string sensor_type;
    EntityId target_entity{INVALID_ENTITY_ID};
    Real range{0.0};
    Real signal_strength{0.0};
};

/**
 * @brief Threshold event data
 */
struct ThresholdEventData {
    std::string parameter_name;
    Real current_value{0.0};
    Real threshold_value{0.0};
    Real delta{0.0};                ///< How much threshold exceeded by
    bool is_upper_limit{true};      ///< true = exceeded upper, false = below lower
};

/**
 * @brief Damage event data
 */
struct DamageEventData {
    Real damage_amount{0.0};
    Real remaining_health{0.0};
    std::string damage_type;
    EntityId source_entity{INVALID_ENTITY_ID};
    Vec3 impact_point{Vec3::Zero()};
};

/**
 * @brief User-defined event data
 */
struct UserEventData {
    UInt32 user_type_id{0};
    std::string user_type_name;
    std::any payload;               ///< Arbitrary user data
};

// ============================================================================
// Event Variant
// ============================================================================

/**
 * @brief Variant type for all event data types
 */
using EventData = std::variant<
    std::monostate,         // No data
    EntityEventData,
    CollisionEventData,
    ConstraintEventData,
    AirEventData,
    LandEventData,
    SeaEventData,
    SpaceEventData,
    SensorEventData,
    ThresholdEventData,
    DamageEventData,
    UserEventData
>;

// ============================================================================
// Event Structure
// ============================================================================

/**
 * @brief Complete event structure
 *
 * Events are value types that can be copied and stored efficiently.
 * The EventData variant holds type-specific information.
 */
struct Event {
    EventBase base;
    EventData data;

    // Convenience accessors
    EventType type() const { return base.type; }
    EventPriority priority() const { return base.priority; }
    Real timestamp() const { return base.timestamp; }
    EntityId source() const { return base.source_entity; }
    bool is_consumed() const { return base.consumed; }

    void consume() { base.consumed = true; }

    EventCategory category() const { return get_event_category(base.type); }

    /**
     * @brief Check if event has specific data type
     */
    template<typename T>
    bool has_data() const {
        return std::holds_alternative<T>(data);
    }

    /**
     * @brief Get event data as specific type
     * @throws std::bad_variant_access if type doesn't match
     */
    template<typename T>
    const T& get_data() const {
        return std::get<T>(data);
    }

    /**
     * @brief Try to get event data as specific type
     * @return Pointer to data or nullptr if type doesn't match
     */
    template<typename T>
    const T* try_get_data() const {
        return std::get_if<T>(&data);
    }

    // Factory methods for common events
    static Event create_entity_created(EntityId id, const std::string& name,
                                       Domain domain, Real timestamp);
    static Event create_entity_destroyed(EntityId id, const std::string& name,
                                         Real timestamp);
    static Event create_collision_enter(EntityId a, EntityId b,
                                        const Vec3& point, const Vec3& normal,
                                        Real penetration, Real timestamp);
    static Event create_collision_exit(EntityId a, EntityId b, Real timestamp);
    static Event create_threshold_exceeded(EntityId source,
                                           const std::string& param,
                                           Real current, Real threshold,
                                           bool is_upper, Real timestamp);

    // Air domain events
    static Event create_stall(EntityId entity, Real aoa, Real ias,
                              Real altitude, Real mach, bool critical, Real timestamp);
    static Event create_stall_recovery(EntityId entity, Real aoa, Real ias,
                                       Real altitude, Real mach, Real timestamp);
    static Event create_overspeed(EntityId entity, Real ias, Real mach,
                                  Real altitude, bool critical, Real timestamp);
    static Event create_engine_flameout(EntityId entity, int engine_index,
                                        Real altitude, Real mach, Real timestamp);

    // Land domain events
    static Event create_wheel_touchdown(EntityId entity, int wheel_index,
                                        const Vec3& contact_point,
                                        Real vertical_speed, Real lateral_speed,
                                        Real timestamp);
    static Event create_wheel_liftoff(EntityId entity, int wheel_index, Real timestamp);
    static Event create_vehicle_rollover(EntityId entity, Real roll_angle,
                                         const Vec3& position, Real timestamp);

    // Sea domain events
    static Event create_capsize(EntityId entity, Real heel_angle,
                                Real wave_height, Real timestamp);
    static Event create_grounding(EntityId entity, const Vec3& impact_point,
                                  Real draft, Real timestamp);

    // Space domain events
    static Event create_orbit_insertion(EntityId entity, Real altitude, Real velocity,
                                        Real eccentricity, Real inclination, Real timestamp);
    static Event create_orbit_decay(EntityId entity, Real altitude, Real velocity,
                                    Real timestamp);
    static Event create_docking(EntityId entity, EntityId target,
                                Real relative_velocity, Real timestamp);
};

// ============================================================================
// Event Handler Types
// ============================================================================

/**
 * @brief Event handler callback signature
 *
 * Handlers can consume events by calling event.consume() to prevent
 * further propagation.
 *
 * @param event The event to handle (mutable for consume())
 * @return true if event should continue propagating, false to stop
 */
using EventHandler = std::function<bool(Event&)>;

/**
 * @brief Handle for unregistering event handlers
 */
using EventHandlerId = UInt64;

constexpr EventHandlerId INVALID_HANDLER_ID = 0;

} // namespace jaguar::events
