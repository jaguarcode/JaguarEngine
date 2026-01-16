/**
 * @file hla.h
 * @brief HLA (High Level Architecture) IEEE 1516 interface for JaguarEngine
 *
 * Provides federate interface for distributed simulation using HLA RTI.
 * Supports IEEE 1516-2010 (HLA Evolved) standard.
 *
 * @note Requires an HLA RTI implementation (Portico, MAK, Pitch)
 */

#pragma once

#include "jaguar/core/types.h"
#include "jaguar/core/math/vector.h"
#include "jaguar/core/math/quaternion.h"
#include "jaguar/physics/entity_state.h"

#include <string>
#include <vector>
#include <functional>
#include <memory>
#include <unordered_map>
#include <chrono>
#include <atomic>
#include <mutex>

namespace jaguar::interface::hla {

// ============================================================================
// HLA Data Types (IEEE 1516.2)
// ============================================================================

/**
 * @brief HLA Object Class enumeration
 */
enum class ObjectClass : uint32_t {
    HLAobjectRoot = 0,
    BaseEntity = 1,
    PhysicalEntity = 2,
    Platform = 3,
    Aircraft = 4,
    GroundVehicle = 5,
    SurfaceVessel = 6,
    Submarine = 7,
    Spacecraft = 8,
    Munition = 9,
    LifeForm = 10,
    Sensor = 11,
    Emitter = 12
};

/**
 * @brief HLA Interaction Class enumeration
 */
enum class InteractionClass : uint32_t {
    HLAinteractionRoot = 0,
    WeaponFire = 1,
    MunitionDetonation = 2,
    RadioSignal = 3,
    StartResume = 4,
    StopFreeze = 5,
    Acknowledge = 6,
    TransferOwnership = 7,
    EntityRemoved = 8
};

/**
 * @brief HLA synchronization point states
 */
enum class SyncPointState {
    Announced,
    Awaiting,
    Achieved,
    Synchronized
};

/**
 * @brief HLA time management mode
 */
enum class TimeManagementMode {
    NotRegulated,     // No time regulation
    TimeRegulating,   // This federate regulates time
    TimeConstrained,  // This federate is constrained by others
    Both              // Both regulating and constrained
};

/**
 * @brief Federate lifecycle state
 */
enum class FederateState {
    NotConnected,
    Connected,
    Joined,
    Running,
    Resigned,
    Destroyed
};

/**
 * @brief HLA attribute handle (opaque handle type)
 */
using AttributeHandle = uint32_t;
using ObjectInstanceHandle = uint64_t;
using InteractionHandle = uint32_t;
using ParameterHandle = uint32_t;
using FederateHandle = uint64_t;

// ============================================================================
// HLA Spatial Representation
// ============================================================================

/**
 * @brief HLA World Location structure (ECEF)
 */
struct WorldLocation {
    double x;  // ECEF X coordinate (meters)
    double y;  // ECEF Y coordinate (meters)
    double z;  // ECEF Z coordinate (meters)

    WorldLocation() : x(0), y(0), z(0) {}
    WorldLocation(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    static WorldLocation from_vec3(const math::Vec3& v) {
        return WorldLocation(v.x, v.y, v.z);
    }

    math::Vec3 to_vec3() const {
        return math::Vec3(x, y, z);
    }
};

/**
 * @brief HLA Velocity Vector structure (ECEF)
 */
struct VelocityVector {
    float x_velocity;  // m/s
    float y_velocity;  // m/s
    float z_velocity;  // m/s

    VelocityVector() : x_velocity(0), y_velocity(0), z_velocity(0) {}
    VelocityVector(float vx, float vy, float vz)
        : x_velocity(vx), y_velocity(vy), z_velocity(vz) {}

    static VelocityVector from_vec3(const math::Vec3& v) {
        return VelocityVector(
            static_cast<float>(v.x),
            static_cast<float>(v.y),
            static_cast<float>(v.z)
        );
    }

    math::Vec3 to_vec3() const {
        return math::Vec3(x_velocity, y_velocity, z_velocity);
    }
};

/**
 * @brief HLA Orientation (Euler angles)
 */
struct Orientation {
    float psi;    // Heading (rad)
    float theta;  // Pitch (rad)
    float phi;    // Roll (rad)

    Orientation() : psi(0), theta(0), phi(0) {}
    Orientation(float p, float t, float r) : psi(p), theta(t), phi(r) {}

    static Orientation from_quaternion(const math::Quat& q) {
        // Convert quaternion to Euler angles
        auto euler = q.to_euler();
        return Orientation(
            static_cast<float>(euler.z),  // Heading (yaw)
            static_cast<float>(euler.y),  // Pitch
            static_cast<float>(euler.x)   // Roll
        );
    }

    math::Quat to_quaternion() const {
        return math::Quat::from_euler(phi, theta, psi);
    }
};

/**
 * @brief HLA Angular Velocity
 */
struct AngularVelocity {
    float x_angular_velocity;  // rad/s
    float y_angular_velocity;  // rad/s
    float z_angular_velocity;  // rad/s

    AngularVelocity() : x_angular_velocity(0), y_angular_velocity(0), z_angular_velocity(0) {}
};

/**
 * @brief HLA Dead Reckoning Algorithm
 */
enum class DeadReckoningAlgorithm : uint8_t {
    Other = 0,
    Static = 1,
    FPW = 2,   // Fixed position, world coordinates
    RPW = 3,   // Rotating position, world coordinates
    RVW = 4,   // Rotating velocity, world coordinates
    FVW = 5,   // Fixed velocity, world coordinates
    FPB = 6,   // Fixed position, body coordinates
    RPB = 7,   // Rotating position, body coordinates
    RVB = 8,   // Rotating velocity, body coordinates
    FVB = 9    // Fixed velocity, body coordinates
};

/**
 * @brief HLA Dead Reckoning Parameters
 */
struct DeadReckoningParameters {
    DeadReckoningAlgorithm algorithm;
    float linear_acceleration[3];   // Body frame (m/s²)
    float angular_velocity[3];      // Body frame (rad/s)

    DeadReckoningParameters() : algorithm(DeadReckoningAlgorithm::RVW) {
        std::memset(linear_acceleration, 0, sizeof(linear_acceleration));
        std::memset(angular_velocity, 0, sizeof(angular_velocity));
    }
};

/**
 * @brief HLA Spatial Representation (complete)
 */
struct SpatialRepresentation {
    WorldLocation world_location;
    bool is_frozen;
    Orientation orientation;
    VelocityVector velocity;
    DeadReckoningParameters dead_reckoning;

    SpatialRepresentation() : is_frozen(false) {}
};

// ============================================================================
// HLA Entity Type (RPR FOM 2.0)
// ============================================================================

/**
 * @brief HLA Entity Type structure (matches DIS enumeration)
 */
struct EntityType {
    uint8_t entity_kind;    // Platform, Munition, etc.
    uint8_t domain;         // Land, Air, Sea, Space
    uint16_t country;       // Country code
    uint8_t category;       // Category within domain
    uint8_t subcategory;    // Subcategory
    uint8_t specific;       // Specific type
    uint8_t extra;          // Extra information

    EntityType()
        : entity_kind(0), domain(0), country(0), category(0)
        , subcategory(0), specific(0), extra(0) {}

    bool operator==(const EntityType& other) const {
        return entity_kind == other.entity_kind &&
               domain == other.domain &&
               country == other.country &&
               category == other.category &&
               subcategory == other.subcategory &&
               specific == other.specific &&
               extra == other.extra;
    }
};

/**
 * @brief Force Identifier
 */
enum class ForceIdentifier : uint8_t {
    Other = 0,
    Friendly = 1,
    Opposing = 2,
    Neutral = 3
};

// ============================================================================
// HLA Object Instance Data
// ============================================================================

/**
 * @brief HLA Platform object instance
 */
struct PlatformObject {
    ObjectInstanceHandle handle;
    std::string instance_name;
    ObjectClass object_class;

    // Required attributes
    EntityType entity_type;
    ForceIdentifier force_identifier;
    SpatialRepresentation spatial;

    // Optional attributes
    std::string entity_identifier;
    std::string marking_text;
    bool is_concealed;
    uint16_t damage_state;

    // Local tracking
    EntityId local_entity_id;
    bool is_local;  // True if we own this object
    std::chrono::steady_clock::time_point last_update;

    PlatformObject()
        : handle(0), object_class(ObjectClass::Platform)
        , force_identifier(ForceIdentifier::Neutral)
        , is_concealed(false), damage_state(0)
        , local_entity_id(INVALID_ENTITY_ID), is_local(false) {}
};

// ============================================================================
// HLA Interaction Data
// ============================================================================

/**
 * @brief Weapon Fire interaction parameters
 */
struct WeaponFireInteraction {
    EntityType munition_type;
    WorldLocation fire_location;
    VelocityVector initial_velocity;
    ObjectInstanceHandle firing_object;
    ObjectInstanceHandle target_object;
    uint32_t fire_mission_index;
    uint16_t warhead_type;
    uint16_t fuze_type;
    float quantity;
    float rate;

    WeaponFireInteraction()
        : firing_object(0), target_object(0)
        , fire_mission_index(0), warhead_type(0)
        , fuze_type(0), quantity(1), rate(0) {}
};

/**
 * @brief Munition Detonation interaction parameters
 */
struct MunitionDetonationInteraction {
    EntityType munition_type;
    WorldLocation detonation_location;
    VelocityVector final_velocity;
    ObjectInstanceHandle munition_object;
    ObjectInstanceHandle firing_object;
    ObjectInstanceHandle target_object;
    uint8_t detonation_result;
    uint16_t warhead_type;
    uint16_t fuze_type;
    float quantity;

    MunitionDetonationInteraction()
        : munition_object(0), firing_object(0), target_object(0)
        , detonation_result(0), warhead_type(0), fuze_type(0), quantity(1) {}
};

// ============================================================================
// HLA Federate Configuration
// ============================================================================

/**
 * @brief HLA Federate configuration
 */
struct HLAConfig {
    // Federation settings
    std::string federation_name = "JaguarFederation";
    std::string federate_name = "JaguarEngine";
    std::string fom_path = "RPR_FOM_v2.0.xml";  // RPR FOM 2.0

    // RTI settings
    std::string rti_host = "localhost";
    uint16_t rti_port = 8989;
    std::string local_settings = "";

    // Time management
    TimeManagementMode time_mode = TimeManagementMode::Both;
    double lookahead = 0.01;  // seconds
    double time_step = 0.01;  // seconds

    // Object management
    bool auto_publish_local = true;
    bool auto_subscribe_remote = true;
    double update_rate = 60.0;  // Hz for attribute updates

    // Dead reckoning
    double position_threshold = 1.0;   // meters
    double orientation_threshold = 0.05;  // radians (~3 degrees)
    double heartbeat_interval = 5.0;   // seconds

    // Synchronization
    std::vector<std::string> sync_points = {"ReadyToRun", "ReadyToResign"};
};

// ============================================================================
// Callback Types
// ============================================================================

using ObjectDiscoveredCallback = std::function<void(ObjectInstanceHandle, ObjectClass, const std::string&)>;
using ObjectRemovedCallback = std::function<void(ObjectInstanceHandle)>;
using AttributeUpdateCallback = std::function<void(ObjectInstanceHandle, const PlatformObject&)>;
using WeaponFireCallback = std::function<void(const WeaponFireInteraction&)>;
using DetonationCallback = std::function<void(const MunitionDetonationInteraction&)>;
using TimeAdvanceCallback = std::function<void(double)>;
using SyncPointCallback = std::function<void(const std::string&, SyncPointState)>;

// ============================================================================
// HLA Federate Ambassador
// ============================================================================

/**
 * @brief HLA Federate Ambassador (RTI callback receiver)
 *
 * This class receives callbacks from the RTI. It is separate from
 * HLAAdapter to allow for proper callback handling patterns.
 */
class FederateAmbassador {
public:
    FederateAmbassador() = default;
    virtual ~FederateAmbassador() = default;

    // Object Management callbacks
    virtual void discoverObjectInstance(
        ObjectInstanceHandle handle,
        ObjectClass object_class,
        const std::string& instance_name) = 0;

    virtual void removeObjectInstance(ObjectInstanceHandle handle) = 0;

    virtual void reflectAttributeValues(
        ObjectInstanceHandle handle,
        const std::vector<std::pair<AttributeHandle, std::vector<uint8_t>>>& attributes,
        double time) = 0;

    // Interaction callbacks
    virtual void receiveInteraction(
        InteractionClass interaction_class,
        const std::vector<std::pair<ParameterHandle, std::vector<uint8_t>>>& parameters,
        double time) = 0;

    // Time Management callbacks
    virtual void timeRegulationEnabled(double federate_time) = 0;
    virtual void timeConstrainedEnabled(double federate_time) = 0;
    virtual void timeAdvanceGrant(double granted_time) = 0;

    // Synchronization callbacks
    virtual void announceSynchronizationPoint(
        const std::string& label,
        const std::vector<uint8_t>& tag) = 0;

    virtual void federationSynchronized(const std::string& label) = 0;

    // Ownership callbacks
    virtual void requestAttributeOwnershipRelease(
        ObjectInstanceHandle handle,
        const std::vector<AttributeHandle>& attributes) = 0;
};

// ============================================================================
// HLA Adapter (Main Interface)
// ============================================================================

/**
 * @brief HLA Network Adapter for JaguarEngine
 *
 * Provides federate functionality for HLA-based distributed simulation.
 * Implements IEEE 1516-2010 (HLA Evolved) standard with RPR FOM 2.0.
 */
class HLAAdapter {
public:
    HLAAdapter();
    ~HLAAdapter();

    // Non-copyable
    HLAAdapter(const HLAAdapter&) = delete;
    HLAAdapter& operator=(const HLAAdapter&) = delete;

    // ========================================================================
    // Lifecycle Management
    // ========================================================================

    /**
     * @brief Initialize the HLA adapter with configuration
     * @param config HLA configuration settings
     * @return true if initialization successful
     */
    bool initialize(const HLAConfig& config = HLAConfig());

    /**
     * @brief Connect to RTI
     * @return true if connection successful
     */
    bool connect();

    /**
     * @brief Create and join federation
     * @return true if joined successfully
     */
    bool join_federation();

    /**
     * @brief Resign from federation and disconnect
     */
    void resign();

    /**
     * @brief Shutdown adapter and release resources
     */
    void shutdown();

    /**
     * @brief Get current federate state
     */
    FederateState get_state() const { return state_; }

    /**
     * @brief Check if federate is connected and joined
     */
    bool is_active() const {
        return state_ == FederateState::Joined || state_ == FederateState::Running;
    }

    // ========================================================================
    // Time Management
    // ========================================================================

    /**
     * @brief Request time advance to specified time
     * @param target_time Target logical time
     * @return true if request submitted
     */
    bool request_time_advance(double target_time);

    /**
     * @brief Wait for time advance grant
     * @param timeout_ms Maximum wait time in milliseconds
     * @return Granted time, or -1 if timeout
     */
    double wait_for_time_advance(uint32_t timeout_ms = 1000);

    /**
     * @brief Get current federate logical time
     */
    double get_federate_time() const { return federate_time_; }

    /**
     * @brief Get lookahead value
     */
    double get_lookahead() const { return config_.lookahead; }

    // ========================================================================
    // Object Management
    // ========================================================================

    /**
     * @brief Register a local entity as HLA object
     * @param entity_id Local entity ID
     * @param name Object instance name
     * @param entity_type HLA entity type
     * @param object_class Object class (Platform, Munition, etc.)
     * @return Object instance handle, or 0 on failure
     */
    ObjectInstanceHandle register_object(
        EntityId entity_id,
        const std::string& name,
        const EntityType& entity_type,
        ObjectClass object_class = ObjectClass::Platform);

    /**
     * @brief Update attributes for a registered object
     * @param handle Object instance handle
     * @param state Current entity state
     * @return true if update sent
     */
    bool update_object_attributes(
        ObjectInstanceHandle handle,
        const physics::EntityState& state);

    /**
     * @brief Delete a registered object
     * @param handle Object instance handle
     * @return true if deletion sent
     */
    bool delete_object(ObjectInstanceHandle handle);

    /**
     * @brief Get remote object by handle
     * @param handle Object instance handle
     * @return Pointer to platform object, or nullptr if not found
     */
    const PlatformObject* get_object(ObjectInstanceHandle handle) const;

    /**
     * @brief Get all discovered remote objects
     */
    std::vector<ObjectInstanceHandle> get_remote_objects() const;

    /**
     * @brief Map remote object handle to local entity ID
     * @param handle Remote object handle
     * @param entity_id Local entity ID to map to
     */
    void map_to_local_entity(ObjectInstanceHandle handle, EntityId entity_id);

    // ========================================================================
    // Interaction Management
    // ========================================================================

    /**
     * @brief Send weapon fire interaction
     * @param fire Weapon fire parameters
     * @return true if sent successfully
     */
    bool send_weapon_fire(const WeaponFireInteraction& fire);

    /**
     * @brief Send munition detonation interaction
     * @param detonation Detonation parameters
     * @return true if sent successfully
     */
    bool send_detonation(const MunitionDetonationInteraction& detonation);

    // ========================================================================
    // Synchronization
    // ========================================================================

    /**
     * @brief Register a synchronization point
     * @param label Sync point label
     * @return true if registration requested
     */
    bool register_sync_point(const std::string& label);

    /**
     * @brief Achieve a synchronization point
     * @param label Sync point label
     * @return true if achievement announced
     */
    bool achieve_sync_point(const std::string& label);

    /**
     * @brief Wait for federation to synchronize on point
     * @param label Sync point label
     * @param timeout_ms Maximum wait time
     * @return true if synchronized
     */
    bool wait_for_sync(const std::string& label, uint32_t timeout_ms = 30000);

    // ========================================================================
    // Callback Registration
    // ========================================================================

    void on_object_discovered(ObjectDiscoveredCallback callback) {
        object_discovered_callback_ = std::move(callback);
    }

    void on_object_removed(ObjectRemovedCallback callback) {
        object_removed_callback_ = std::move(callback);
    }

    void on_attribute_update(AttributeUpdateCallback callback) {
        attribute_update_callback_ = std::move(callback);
    }

    void on_weapon_fire(WeaponFireCallback callback) {
        weapon_fire_callback_ = std::move(callback);
    }

    void on_detonation(DetonationCallback callback) {
        detonation_callback_ = std::move(callback);
    }

    void on_time_advance(TimeAdvanceCallback callback) {
        time_advance_callback_ = std::move(callback);
    }

    void on_sync_point(SyncPointCallback callback) {
        sync_point_callback_ = std::move(callback);
    }

    // ========================================================================
    // Processing
    // ========================================================================

    /**
     * @brief Process RTI callbacks
     *
     * Should be called regularly (e.g., every simulation frame) to
     * process incoming RTI messages.
     *
     * @param timeout_ms Maximum time to wait for callbacks
     */
    void tick(uint32_t timeout_ms = 10);

    /**
     * @brief Check if object needs attribute update (dead reckoning)
     * @param handle Object instance handle
     * @param current_state Current entity state
     * @return true if update threshold exceeded
     */
    bool needs_update(ObjectInstanceHandle handle,
                      const physics::EntityState& current_state) const;

    // ========================================================================
    // Statistics
    // ========================================================================

    struct Statistics {
        uint64_t objects_registered;
        uint64_t objects_discovered;
        uint64_t attribute_updates_sent;
        uint64_t attribute_updates_received;
        uint64_t interactions_sent;
        uint64_t interactions_received;
        uint64_t time_advances_requested;
        uint64_t time_advances_granted;

        Statistics() { reset(); }
        void reset() {
            objects_registered = 0;
            objects_discovered = 0;
            attribute_updates_sent = 0;
            attribute_updates_received = 0;
            interactions_sent = 0;
            interactions_received = 0;
            time_advances_requested = 0;
            time_advances_granted = 0;
        }
    };

    const Statistics& get_statistics() const { return stats_; }

private:
    // Configuration
    HLAConfig config_;
    FederateState state_;

    // Time management
    std::atomic<double> federate_time_;
    std::atomic<double> granted_time_;
    std::atomic<bool> time_advance_pending_;
    std::atomic<bool> time_regulation_enabled_;
    std::atomic<bool> time_constrained_enabled_;

    // Object management
    std::unordered_map<ObjectInstanceHandle, PlatformObject> objects_;
    std::unordered_map<EntityId, ObjectInstanceHandle> local_entity_map_;
    mutable std::mutex objects_mutex_;

    // Synchronization
    std::unordered_map<std::string, SyncPointState> sync_points_;
    mutable std::mutex sync_mutex_;

    // Callbacks
    ObjectDiscoveredCallback object_discovered_callback_;
    ObjectRemovedCallback object_removed_callback_;
    AttributeUpdateCallback attribute_update_callback_;
    WeaponFireCallback weapon_fire_callback_;
    DetonationCallback detonation_callback_;
    TimeAdvanceCallback time_advance_callback_;
    SyncPointCallback sync_point_callback_;

    // Statistics
    Statistics stats_;

    // RTI ambassador (opaque pointer to avoid RTI header dependency)
    class RTIAmbassador;
    std::unique_ptr<RTIAmbassador> rti_;

    // Federate ambassador implementation
    class FederateAmbassadorImpl;
    std::unique_ptr<FederateAmbassadorImpl> ambassador_;

    // Internal helpers
    void publish_subscribe_setup();
    std::vector<uint8_t> encode_spatial(const physics::EntityState& state);
    void decode_spatial(const std::vector<uint8_t>& data, SpatialRepresentation& spatial);
    bool check_dead_reckoning_threshold(const PlatformObject& obj,
                                         const physics::EntityState& current) const;
};

// ============================================================================
// Conversion Utilities
// ============================================================================

namespace convert {

/**
 * @brief Convert JaguarEngine Domain to HLA ObjectClass
 */
inline ObjectClass domain_to_object_class(Domain domain) {
    switch (domain) {
        case Domain::Air:   return ObjectClass::Aircraft;
        case Domain::Land:  return ObjectClass::GroundVehicle;
        case Domain::Sea:   return ObjectClass::SurfaceVessel;
        case Domain::Space: return ObjectClass::Spacecraft;
        default:            return ObjectClass::Platform;
    }
}

/**
 * @brief Convert JaguarEngine Domain to HLA EntityType domain field
 */
inline uint8_t domain_to_entity_domain(Domain domain) {
    switch (domain) {
        case Domain::Land:  return 1;  // Land
        case Domain::Air:   return 2;  // Air
        case Domain::Sea:   return 3;  // Surface
        case Domain::Space: return 4;  // Subsurface (reused for space)
        default:            return 0;  // Other
    }
}

/**
 * @brief Convert EntityState to SpatialRepresentation
 */
inline SpatialRepresentation entity_state_to_spatial(const physics::EntityState& state) {
    SpatialRepresentation spatial;
    spatial.world_location = WorldLocation::from_vec3(state.position);
    spatial.velocity = VelocityVector::from_vec3(state.velocity);
    spatial.orientation = Orientation::from_quaternion(state.orientation);
    spatial.is_frozen = false;

    // Set dead reckoning parameters
    spatial.dead_reckoning.algorithm = DeadReckoningAlgorithm::RVW;
    spatial.dead_reckoning.linear_acceleration[0] = static_cast<float>(state.acceleration.x);
    spatial.dead_reckoning.linear_acceleration[1] = static_cast<float>(state.acceleration.y);
    spatial.dead_reckoning.linear_acceleration[2] = static_cast<float>(state.acceleration.z);
    spatial.dead_reckoning.angular_velocity[0] = static_cast<float>(state.angular_velocity.x);
    spatial.dead_reckoning.angular_velocity[1] = static_cast<float>(state.angular_velocity.y);
    spatial.dead_reckoning.angular_velocity[2] = static_cast<float>(state.angular_velocity.z);

    return spatial;
}

/**
 * @brief Convert SpatialRepresentation to EntityState
 */
inline physics::EntityState spatial_to_entity_state(const SpatialRepresentation& spatial) {
    physics::EntityState state;
    state.position = spatial.world_location.to_vec3();
    state.velocity = spatial.velocity.to_vec3();
    state.orientation = spatial.orientation.to_quaternion();

    state.acceleration = math::Vec3(
        spatial.dead_reckoning.linear_acceleration[0],
        spatial.dead_reckoning.linear_acceleration[1],
        spatial.dead_reckoning.linear_acceleration[2]
    );
    state.angular_velocity = math::Vec3(
        spatial.dead_reckoning.angular_velocity[0],
        spatial.dead_reckoning.angular_velocity[1],
        spatial.dead_reckoning.angular_velocity[2]
    );

    return state;
}

} // namespace convert

} // namespace jaguar::interface::hla
