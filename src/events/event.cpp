/**
 * @file event.cpp
 * @brief Event system core implementation
 */

#include "jaguar/events/event.h"

namespace jaguar::events {

EventCategory get_event_category(EventType type) {
    UInt32 type_val = static_cast<UInt32>(type);

    if (type_val < 100) {
        return EventCategory::System;
    } else if (type_val < 200) {
        return EventCategory::Entity;
    } else if (type_val < 300) {
        return EventCategory::Physics;
    } else if (type_val < 400) {
        return EventCategory::Domain;
    } else if (type_val < 500) {
        return EventCategory::Sensor;
    } else if (type_val < 600) {
        return EventCategory::Threshold;
    } else if (type_val >= 1000) {
        return EventCategory::User;
    }

    return EventCategory::None;
}

const char* get_event_type_name(EventType type) {
    switch (type) {
        // System Events
        case EventType::SimulationStarted:      return "SimulationStarted";
        case EventType::SimulationPaused:       return "SimulationPaused";
        case EventType::SimulationResumed:      return "SimulationResumed";
        case EventType::SimulationStopped:      return "SimulationStopped";
        case EventType::TimeStepCompleted:      return "TimeStepCompleted";
        case EventType::FrameCompleted:         return "FrameCompleted";

        // Entity Lifecycle
        case EventType::EntityCreated:          return "EntityCreated";
        case EventType::EntityDestroyed:        return "EntityDestroyed";
        case EventType::EntityActivated:        return "EntityActivated";
        case EventType::EntityDeactivated:      return "EntityDeactivated";
        case EventType::EntityStateChanged:     return "EntityStateChanged";
        case EventType::ComponentAdded:         return "ComponentAdded";
        case EventType::ComponentRemoved:       return "ComponentRemoved";

        // Physics/Collision
        case EventType::CollisionEnter:         return "CollisionEnter";
        case EventType::CollisionExit:          return "CollisionExit";
        case EventType::CollisionStay:          return "CollisionStay";
        case EventType::TriggerEnter:           return "TriggerEnter";
        case EventType::TriggerExit:            return "TriggerExit";
        case EventType::ConstraintBroken:       return "ConstraintBroken";
        case EventType::ConstraintCreated:      return "ConstraintCreated";
        case EventType::ConstraintDestroyed:    return "ConstraintDestroyed";

        // Air Domain
        case EventType::Stall:                  return "Stall";
        case EventType::StallRecovery:          return "StallRecovery";
        case EventType::OverspeedWarning:       return "OverspeedWarning";
        case EventType::OverspeedRecovery:      return "OverspeedRecovery";
        case EventType::EngineFlameout:         return "EngineFlameout";
        case EventType::EngineRestart:          return "EngineRestart";

        // Land Domain
        case EventType::WheelTouchdown:         return "WheelTouchdown";
        case EventType::WheelLiftoff:           return "WheelLiftoff";
        case EventType::VehicleRollover:        return "VehicleRollover";
        case EventType::TrackSlip:              return "TrackSlip";

        // Sea Domain
        case EventType::Capsize:                return "Capsize";
        case EventType::Grounding:              return "Grounding";
        case EventType::Flooding:               return "Flooding";
        case EventType::WaveImpact:             return "WaveImpact";

        // Space Domain
        case EventType::OrbitInsertion:         return "OrbitInsertion";
        case EventType::OrbitDecay:             return "OrbitDecay";
        case EventType::Reentry:                return "Reentry";
        case EventType::Docking:                return "Docking";
        case EventType::Undocking:              return "Undocking";

        // Sensor
        case EventType::SensorActivated:        return "SensorActivated";
        case EventType::SensorDeactivated:      return "SensorDeactivated";
        case EventType::SensorUpdate:           return "SensorUpdate";
        case EventType::SensorFailure:          return "SensorFailure";
        case EventType::TargetAcquired:         return "TargetAcquired";
        case EventType::TargetLost:             return "TargetLost";

        // Threshold
        case EventType::ThresholdExceeded:      return "ThresholdExceeded";
        case EventType::ThresholdRecovered:     return "ThresholdRecovered";
        case EventType::SpeedLimitExceeded:     return "SpeedLimitExceeded";
        case EventType::AltitudeLimitExceeded:  return "AltitudeLimitExceeded";
        case EventType::TemperatureLimitExceeded: return "TemperatureLimitExceeded";
        case EventType::StressLimitExceeded:    return "StressLimitExceeded";
        case EventType::FuelLow:                return "FuelLow";
        case EventType::FuelCritical:           return "FuelCritical";

        // Damage
        case EventType::DamageReceived:         return "DamageReceived";
        case EventType::ComponentDamaged:       return "ComponentDamaged";
        case EventType::ComponentDestroyed:     return "ComponentDestroyed";
        case EventType::EntityDestroyed_Damage: return "EntityDestroyed_Damage";

        // User
        case EventType::UserDefined:            return "UserDefined";

        default:                                return "Unknown";
    }
}

// ============================================================================
// Event Factory Methods
// ============================================================================

Event Event::create_entity_created(EntityId id, const std::string& name,
                                   Domain domain, Real timestamp) {
    Event event;
    event.base.type = EventType::EntityCreated;
    event.base.priority = EventPriority::Normal;
    event.base.timestamp = timestamp;
    event.base.source_entity = id;

    EntityEventData data;
    data.entity_id = id;
    data.entity_name = name;
    data.domain = domain;
    event.data = data;

    return event;
}

Event Event::create_entity_destroyed(EntityId id, const std::string& name,
                                     Real timestamp) {
    Event event;
    event.base.type = EventType::EntityDestroyed;
    event.base.priority = EventPriority::Normal;
    event.base.timestamp = timestamp;
    event.base.source_entity = id;

    EntityEventData data;
    data.entity_id = id;
    data.entity_name = name;
    event.data = data;

    return event;
}

Event Event::create_collision_enter(EntityId a, EntityId b,
                                    const Vec3& point, const Vec3& normal,
                                    Real penetration, Real timestamp) {
    Event event;
    event.base.type = EventType::CollisionEnter;
    event.base.priority = EventPriority::High;
    event.base.timestamp = timestamp;
    event.base.source_entity = a;

    CollisionEventData data;
    data.entity_A = a;
    data.entity_B = b;
    data.contact_point = point;
    data.contact_normal = normal;
    data.penetration_depth = penetration;
    event.data = data;

    return event;
}

Event Event::create_collision_exit(EntityId a, EntityId b, Real timestamp) {
    Event event;
    event.base.type = EventType::CollisionExit;
    event.base.priority = EventPriority::Normal;
    event.base.timestamp = timestamp;
    event.base.source_entity = a;

    CollisionEventData data;
    data.entity_A = a;
    data.entity_B = b;
    event.data = data;

    return event;
}

Event Event::create_threshold_exceeded(EntityId source,
                                       const std::string& param,
                                       Real current, Real threshold,
                                       bool is_upper, Real timestamp) {
    Event event;
    event.base.type = EventType::ThresholdExceeded;
    event.base.priority = EventPriority::High;
    event.base.timestamp = timestamp;
    event.base.source_entity = source;

    ThresholdEventData data;
    data.parameter_name = param;
    data.current_value = current;
    data.threshold_value = threshold;
    data.delta = is_upper ? (current - threshold) : (threshold - current);
    data.is_upper_limit = is_upper;
    event.data = data;

    return event;
}

// ============================================================================
// Air Domain Events
// ============================================================================

Event Event::create_stall(EntityId entity, Real aoa, Real ias,
                          Real altitude, Real mach, bool critical, Real timestamp) {
    Event event;
    event.base.type = EventType::Stall;
    event.base.priority = critical ? EventPriority::Immediate : EventPriority::High;
    event.base.timestamp = timestamp;
    event.base.source_entity = entity;

    AirEventData data;
    data.angle_of_attack = aoa;
    data.indicated_airspeed = ias;
    data.altitude = altitude;
    data.mach_number = mach;
    data.critical = critical;
    event.data = data;

    return event;
}

Event Event::create_stall_recovery(EntityId entity, Real aoa, Real ias,
                                   Real altitude, Real mach, Real timestamp) {
    Event event;
    event.base.type = EventType::StallRecovery;
    event.base.priority = EventPriority::Normal;
    event.base.timestamp = timestamp;
    event.base.source_entity = entity;

    AirEventData data;
    data.angle_of_attack = aoa;
    data.indicated_airspeed = ias;
    data.altitude = altitude;
    data.mach_number = mach;
    data.critical = false;
    event.data = data;

    return event;
}

Event Event::create_overspeed(EntityId entity, Real ias, Real mach,
                              Real altitude, bool critical, Real timestamp) {
    Event event;
    event.base.type = EventType::OverspeedWarning;
    event.base.priority = critical ? EventPriority::Immediate : EventPriority::High;
    event.base.timestamp = timestamp;
    event.base.source_entity = entity;

    AirEventData data;
    data.angle_of_attack = 0.0;  // Not relevant for overspeed
    data.indicated_airspeed = ias;
    data.altitude = altitude;
    data.mach_number = mach;
    data.critical = critical;
    event.data = data;

    return event;
}

Event Event::create_engine_flameout(EntityId entity, int engine_index,
                                    Real altitude, Real mach, Real timestamp) {
    Event event;
    event.base.type = EventType::EngineFlameout;
    event.base.priority = EventPriority::Immediate;
    event.base.timestamp = timestamp;
    event.base.source_entity = entity;

    AirEventData data;
    data.angle_of_attack = static_cast<Real>(engine_index);  // Repurpose for engine index
    data.indicated_airspeed = 0.0;
    data.altitude = altitude;
    data.mach_number = mach;
    data.critical = true;
    event.data = data;

    return event;
}

// ============================================================================
// Land Domain Events
// ============================================================================

Event Event::create_wheel_touchdown(EntityId entity, int wheel_index,
                                    const Vec3& contact_point,
                                    Real vertical_speed, Real lateral_speed,
                                    Real timestamp) {
    Event event;
    event.base.type = EventType::WheelTouchdown;
    event.base.priority = EventPriority::High;
    event.base.timestamp = timestamp;
    event.base.source_entity = entity;

    LandEventData data;
    data.wheel_index = wheel_index;
    data.contact_point = contact_point;
    data.vertical_speed = vertical_speed;
    data.lateral_speed = lateral_speed;
    data.ground_slope = 0.0;
    event.data = data;

    return event;
}

Event Event::create_wheel_liftoff(EntityId entity, int wheel_index, Real timestamp) {
    Event event;
    event.base.type = EventType::WheelLiftoff;
    event.base.priority = EventPriority::Normal;
    event.base.timestamp = timestamp;
    event.base.source_entity = entity;

    LandEventData data;
    data.wheel_index = wheel_index;
    event.data = data;

    return event;
}

Event Event::create_vehicle_rollover(EntityId entity, Real roll_angle,
                                     const Vec3& position, Real timestamp) {
    Event event;
    event.base.type = EventType::VehicleRollover;
    event.base.priority = EventPriority::Immediate;
    event.base.timestamp = timestamp;
    event.base.source_entity = entity;

    LandEventData data;
    data.wheel_index = -1;  // Vehicle-wide event
    data.contact_point = position;
    data.lateral_speed = roll_angle;  // Store roll angle in lateral_speed
    event.data = data;

    return event;
}

// ============================================================================
// Sea Domain Events
// ============================================================================

Event Event::create_capsize(EntityId entity, Real heel_angle,
                            Real wave_height, Real timestamp) {
    Event event;
    event.base.type = EventType::Capsize;
    event.base.priority = EventPriority::Immediate;
    event.base.timestamp = timestamp;
    event.base.source_entity = entity;

    SeaEventData data;
    data.heel_angle = heel_angle;
    data.wave_height = wave_height;
    event.data = data;

    return event;
}

Event Event::create_grounding(EntityId entity, const Vec3& impact_point,
                              Real draft, Real timestamp) {
    Event event;
    event.base.type = EventType::Grounding;
    event.base.priority = EventPriority::Immediate;
    event.base.timestamp = timestamp;
    event.base.source_entity = entity;

    SeaEventData data;
    data.draft = draft;
    data.impact_point = impact_point;
    event.data = data;

    return event;
}

// ============================================================================
// Space Domain Events
// ============================================================================

Event Event::create_orbit_insertion(EntityId entity, Real altitude, Real velocity,
                                    Real eccentricity, Real inclination, Real timestamp) {
    Event event;
    event.base.type = EventType::OrbitInsertion;
    event.base.priority = EventPriority::High;
    event.base.timestamp = timestamp;
    event.base.source_entity = entity;

    SpaceEventData data;
    data.altitude = altitude;
    data.velocity = velocity;
    data.eccentricity = eccentricity;
    data.inclination = inclination;
    event.data = data;

    return event;
}

Event Event::create_orbit_decay(EntityId entity, Real altitude, Real velocity,
                                Real timestamp) {
    Event event;
    event.base.type = EventType::OrbitDecay;
    event.base.priority = EventPriority::High;
    event.base.timestamp = timestamp;
    event.base.source_entity = entity;

    SpaceEventData data;
    data.altitude = altitude;
    data.velocity = velocity;
    event.data = data;

    return event;
}

Event Event::create_docking(EntityId entity, EntityId target,
                            Real relative_velocity, Real timestamp) {
    Event event;
    event.base.type = EventType::Docking;
    event.base.priority = EventPriority::High;
    event.base.timestamp = timestamp;
    event.base.source_entity = entity;

    SpaceEventData data;
    data.target_entity = target;
    data.velocity = relative_velocity;
    event.data = data;

    return event;
}

} // namespace jaguar::events
