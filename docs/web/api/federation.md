# Federation API Reference

Federation module provides complete implementations of IEEE 1278.1-2012 Distributed Interactive Simulation (DIS) protocol and IEEE 1516-2010 High Level Architecture (HLA) Runtime Infrastructure (RTI).

## Overview

The Federation module enables:
- **DIS Protocol**: Real-time entity state exchange via UDP PDUs
- **HLA RTI**: Federation-based distributed simulation with time management

---

## DIS Protocol (IEEE 1278.1-2012)

### Header

```cpp
#include <jaguar/federation/dis_protocol.h>
```

### PDU Types

```cpp
namespace jaguar::federation {

// PDU type enumeration (IEEE 1278.1-2012)
enum class PDUType : uint8_t {
    Other = 0,
    EntityState = 1,
    Fire = 2,
    Detonation = 3,
    Collision = 4,
    ServiceRequest = 5,
    ResupplyOffer = 6,
    ResupplyReceived = 7,
    ResupplyCancel = 8,
    RepairComplete = 9,
    RepairResponse = 10,
    CreateEntity = 11,
    RemoveEntity = 12,
    StartResume = 13,
    StopFreeze = 14,
    Acknowledge = 15,
    ActionRequest = 16,
    ActionResponse = 17,
    DataQuery = 18,
    SetData = 19,
    Data = 20,
    EventReport = 21,
    Comment = 22,
    ElectromagneticEmission = 23,
    Designator = 24,
    Transmitter = 25,
    Signal = 26,
    Receiver = 27,
    IFF = 28,
    UnderwaterAcoustic = 29,
    SupplementalEmission = 30,
    IntercomSignal = 31,
    IntercomControl = 32,
    AggregateState = 33,
    IsGroupOf = 34,
    TransferOwnership = 35,
    IsPartOf = 36,
    MinefieldState = 37,
    MinefieldQuery = 38,
    MinefieldData = 39,
    MinefieldResponseNACK = 40,
    EnvironmentalProcess = 41,
    GriddedData = 42,
    PointObjectState = 43,
    LinearObjectState = 44,
    ArealObjectState = 45,
    TSPI = 46,
    Appearance = 47,
    ArticulatedParts = 48,
    LEFire = 49,
    LEDetonation = 50,
    CreateEntityR = 51,
    RemoveEntityR = 52,
    StartResumeR = 53,
    StopFreezeR = 54,
    AcknowledgeR = 55,
    ActionRequestR = 56,
    ActionResponseR = 57,
    DataQueryR = 58,
    SetDataR = 59,
    DataR = 60,
    EventReportR = 61,
    CommentR = 62,
    RecordR = 63,
    SetRecordR = 64,
    RecordQueryR = 65,
    CollisionElastic = 66,
    EntityStateUpdate = 67,
    DirectedEnergyFire = 68,
    EntityDamageStatus = 69,
    InformationOperationsAction = 70,
    InformationOperationsReport = 71,
    Attribute = 72
};

}  // namespace jaguar::federation
```

### Dead Reckoning Algorithms

```cpp
namespace jaguar::federation {

// Dead reckoning algorithm enumeration
enum class DeadReckoningAlgorithm : uint8_t {
    DRM_Other = 0,
    DRM_Static = 1,           // Static entity
    DRM_FPW = 2,              // Fixed position, world coordinates
    DRM_RPW = 3,              // Rotating position, world coordinates
    DRM_RVW = 4,              // Rotating velocity, world coordinates
    DRM_FVW = 5,              // Fixed velocity, world coordinates
    DRM_FPB = 6,              // Fixed position, body coordinates
    DRM_RPB = 7,              // Rotating position, body coordinates
    DRM_RVB = 8,              // Rotating velocity, body coordinates
    DRM_FVB = 9,              // Fixed velocity, body coordinates
    DRM_FPW_HighRes = 10,     // High resolution variants
    DRM_RPW_HighRes = 11,
    DRM_RVW_HighRes = 12,
    DRM_FVW_HighRes = 13,
    DRM_FPB_HighRes = 14,
    DRM_RPB_HighRes = 15,
    DRM_RVB_HighRes = 16,
    DRM_FVB_HighRes = 17
};

// Dead reckoning calculator
class DeadReckoningCalculator {
public:
    // Set algorithm
    void set_algorithm(DeadReckoningAlgorithm algorithm);
    DeadReckoningAlgorithm get_algorithm() const;

    // Set initial state
    void set_state(const Vec3& position,
                   const Vec3& velocity,
                   const Vec3& acceleration,
                   const EulerAngles& orientation,
                   const Vec3& angular_velocity);

    // Extrapolate to time
    void extrapolate(double delta_time,
                    Vec3& out_position,
                    Vec3& out_velocity,
                    EulerAngles& out_orientation) const;

    // Check if update needed (threshold exceeded)
    bool needs_update(const Vec3& actual_position,
                     const Vec3& actual_velocity,
                     const EulerAngles& actual_orientation,
                     double position_threshold,
                     double orientation_threshold) const;
};

}  // namespace jaguar::federation
```

### Entity and Type Identifiers

```cpp
namespace jaguar::federation {

// Entity identifier (site, application, entity)
struct EntityIdentifier {
    uint16_t site_id;
    uint16_t application_id;
    uint16_t entity_id;

    bool operator==(const EntityIdentifier& other) const;
    bool operator<(const EntityIdentifier& other) const;
    std::string to_string() const;
};

// Entity type enumeration
struct EntityType {
    uint8_t entity_kind;        // Platform, Munition, Life Form, etc.
    uint8_t domain;             // Land, Air, Surface, Subsurface, Space
    uint16_t country;           // Country code (SISO-REF-010)
    uint8_t category;           // Vehicle category
    uint8_t subcategory;        // Specific type
    uint8_t specific;           // Specific variant
    uint8_t extra;              // Extra information

    bool operator==(const EntityType& other) const;
    std::string to_string() const;
};

// Force ID
enum class ForceId : uint8_t {
    Other = 0,
    Friendly = 1,
    Opposing = 2,
    Neutral = 3
};

// Entity kind
enum class EntityKind : uint8_t {
    Other = 0,
    Platform = 1,
    Munition = 2,
    LifeForm = 3,
    Environmental = 4,
    CulturalFeature = 5,
    Supply = 6,
    Radio = 7,
    Expendable = 8,
    SensorEmitter = 9
};

// Platform domain
enum class PlatformDomain : uint8_t {
    Other = 0,
    Land = 1,
    Air = 2,
    Surface = 3,
    Subsurface = 4,
    Space = 5
};

}  // namespace jaguar::federation
```

### Coordinate Types

```cpp
namespace jaguar::federation {

// Euler angles (psi, theta, phi - heading, pitch, roll)
struct EulerAngles {
    float psi;      // Heading (radians)
    float theta;    // Pitch (radians)
    float phi;      // Roll (radians)

    Quaternion to_quaternion() const;
    static EulerAngles from_quaternion(const Quaternion& q);
};

// Geodetic coordinates (WGS84)
struct GeodeticCoordinates {
    double latitude;    // Radians
    double longitude;   // Radians
    double altitude;    // Meters above ellipsoid

    Vec3 to_ecef() const;
    static GeodeticCoordinates from_ecef(const Vec3& ecef);
};

// World coordinates (ECEF - Earth-Centered Earth-Fixed)
// Represented as Vec3 {x, y, z} in meters

}  // namespace jaguar::federation
```

### PDU Structures

```cpp
namespace jaguar::federation {

// PDU Header (common to all PDUs)
struct PDUHeader {
    uint8_t protocol_version;   // 7 for IEEE 1278.1-2012
    uint8_t exercise_id;
    PDUType pdu_type;
    uint8_t protocol_family;
    uint32_t timestamp;
    uint16_t pdu_length;
    uint16_t pdu_status;
    uint8_t padding;
};

// Entity State PDU
struct EntityStatePDU {
    PDUHeader header;
    EntityIdentifier entity_id;
    ForceId force_id;
    uint8_t num_articulation_params;
    EntityType entity_type;
    EntityType alt_entity_type;
    Vec3 linear_velocity;           // ECEF, m/s
    Vec3 location;                  // ECEF, meters
    EulerAngles orientation;
    uint32_t entity_appearance;
    DeadReckoningAlgorithm dr_algorithm;
    uint8_t dr_params[15];
    Vec3 linear_acceleration;
    Vec3 angular_velocity;
    uint8_t marking[12];            // Entity marking (callsign)
    uint32_t capabilities;
    // Variable articulation parameters follow
};

// Fire PDU
struct FirePDU {
    PDUHeader header;
    EntityIdentifier firing_entity_id;
    EntityIdentifier target_entity_id;
    EntityIdentifier munition_id;
    uint32_t event_id;
    uint32_t fire_mission_index;
    Vec3 location;                  // ECEF firing location
    EntityType burst_descriptor;
    Vec3 velocity;                  // Munition velocity
    float range;
};

// Detonation PDU
struct DetonationPDU {
    PDUHeader header;
    EntityIdentifier firing_entity_id;
    EntityIdentifier target_entity_id;
    EntityIdentifier munition_id;
    uint32_t event_id;
    Vec3 velocity;                  // Munition velocity at impact
    Vec3 location;                  // Detonation location (ECEF)
    EntityType burst_descriptor;
    Vec3 location_in_entity;        // Relative to target
    uint8_t detonation_result;
    uint8_t num_articulation_params;
    uint16_t padding;
    // Variable articulation parameters follow
};

// Detonation result codes
enum class DetonationResult : uint8_t {
    Other = 0,
    EntityImpact = 1,
    EntityProximateDetonation = 2,
    GroundImpact = 3,
    GroundProximateDetonation = 4,
    Detonation = 5,
    None = 6,
    HEHitSmall = 7,
    HEHitMedium = 8,
    HEHitLarge = 9,
    ArmorPiercingHit = 10,
    DirtBlast_Small = 11,
    DirtBlast_Medium = 12,
    DirtBlast_Large = 13,
    WaterBlast_Small = 14,
    WaterBlast_Medium = 15,
    WaterBlast_Large = 16,
    AirHit = 17,
    BuildingHit_Small = 18,
    BuildingHit_Medium = 19,
    BuildingHit_Large = 20,
    MineClearingLineCharge = 21,
    EnvironmentObjectImpact = 22,
    EnvironmentObjectProximateDetonation = 23,
    WaterImpact = 24,
    AirBurst = 25,
    KillWithFragmentType1 = 26,
    KillWithFragmentType2 = 27,
    KillWithFragmentType3 = 28,
    KillWithFragmentType1After = 29,
    KillWithFragmentType2After = 30,
    MissedDueToFlyout = 31,
    MissedDueToEndGame = 32,
    MissedDueToFlyoutEndGame = 33
};

}  // namespace jaguar::federation
```

### DIS Interface Classes

```cpp
namespace jaguar::federation {

// DIS codec interface
class IDISCodec {
public:
    virtual ~IDISCodec() = default;

    // Encode PDU to bytes
    virtual std::vector<uint8_t> encode(const EntityStatePDU& pdu) = 0;
    virtual std::vector<uint8_t> encode(const FirePDU& pdu) = 0;
    virtual std::vector<uint8_t> encode(const DetonationPDU& pdu) = 0;

    // Decode bytes to PDU
    virtual PDUType get_pdu_type(const std::vector<uint8_t>& data) = 0;
    virtual EntityStatePDU decode_entity_state(const std::vector<uint8_t>& data) = 0;
    virtual FirePDU decode_fire(const std::vector<uint8_t>& data) = 0;
    virtual DetonationPDU decode_detonation(const std::vector<uint8_t>& data) = 0;
};

// DIS network interface
class IDISNetwork {
public:
    virtual ~IDISNetwork() = default;

    // Initialize network
    virtual bool initialize(const DISConfig& config) = 0;
    virtual void shutdown() = 0;

    // Send PDU
    virtual bool send(const std::vector<uint8_t>& pdu_data) = 0;

    // Receive PDUs (non-blocking)
    virtual std::vector<std::vector<uint8_t>> receive() = 0;

    // Join/leave multicast group
    virtual bool join_multicast(const std::string& group_address) = 0;
    virtual void leave_multicast(const std::string& group_address) = 0;
};

// DIS configuration
struct DISConfig {
    bool enabled = false;
    std::string broadcast_address = "255.255.255.255";
    std::string multicast_address = "";     // Optional multicast
    uint16_t port = 3000;
    uint16_t site_id = 1;
    uint16_t application_id = 1;
    uint8_t exercise_id = 1;
    DeadReckoningAlgorithm default_dr_algorithm = DeadReckoningAlgorithm::DRM_FVW;
    double position_threshold = 1.0;        // meters
    double orientation_threshold = 0.05;    // radians
    double heartbeat_interval = 5.0;        // seconds
    bool enable_ieee_2012 = true;           // Use IEEE 1278.1-2012 features
};

// Factory functions
std::unique_ptr<IDISCodec> create_dis_codec(bool ieee_2012 = true);
std::unique_ptr<IDISNetwork> create_dis_network(const DISConfig& config);

}  // namespace jaguar::federation
```

---

## HLA RTI (IEEE 1516-2010)

### Header

```cpp
#include <jaguar/federation/hla_rti.h>
```

### Handle Types

```cpp
namespace jaguar::federation {

// Strongly-typed handles
struct ObjectInstanceHandle {
    uint64_t value;
    bool is_valid() const { return value != 0; }
    bool operator==(const ObjectInstanceHandle& other) const;
    bool operator<(const ObjectInstanceHandle& other) const;
};

struct ObjectClassHandle {
    uint64_t value;
    bool is_valid() const { return value != 0; }
    bool operator==(const ObjectClassHandle& other) const;
};

struct AttributeHandle {
    uint64_t value;
    bool is_valid() const { return value != 0; }
    bool operator==(const AttributeHandle& other) const;
};

struct InteractionClassHandle {
    uint64_t value;
    bool is_valid() const { return value != 0; }
};

struct ParameterHandle {
    uint64_t value;
    bool is_valid() const { return value != 0; }
};

struct FederateHandle {
    uint64_t value;
    bool is_valid() const { return value != 0; }
};

// Handle sets
using AttributeHandleSet = std::set<AttributeHandle>;
using ObjectInstanceHandleSet = std::set<ObjectInstanceHandle>;

}  // namespace jaguar::federation
```

### Time Management

```cpp
namespace jaguar::federation {

// Logical time (HLA time representation)
class LogicalTime {
public:
    LogicalTime();
    explicit LogicalTime(double time);

    double get_time() const;
    void set_time(double time);

    LogicalTime operator+(const LogicalTimeInterval& interval) const;
    LogicalTime operator-(const LogicalTimeInterval& interval) const;
    bool operator<(const LogicalTime& other) const;
    bool operator<=(const LogicalTime& other) const;
    bool operator==(const LogicalTime& other) const;

private:
    double time_;
};

// Logical time interval (duration)
class LogicalTimeInterval {
public:
    LogicalTimeInterval();
    explicit LogicalTimeInterval(double interval);

    double get_interval() const;
    void set_interval(double interval);

    bool operator<(const LogicalTimeInterval& other) const;
    bool operator==(const LogicalTimeInterval& other) const;

private:
    double interval_;
};

}  // namespace jaguar::federation
```

### RTI Ambassador Interface

```cpp
namespace jaguar::federation {

// RTI Ambassador - main interface to RTI
class IRTIAmbassador {
public:
    virtual ~IRTIAmbassador() = default;

    // Federation management
    virtual void connect(const std::string& local_settings_designator) = 0;
    virtual void disconnect() = 0;

    virtual void create_federation_execution(
        const std::string& federation_name,
        const std::vector<std::string>& fom_modules) = 0;

    virtual void destroy_federation_execution(
        const std::string& federation_name) = 0;

    virtual FederateHandle join_federation_execution(
        const std::string& federate_name,
        const std::string& federate_type,
        const std::string& federation_name) = 0;

    virtual void resign_federation_execution(
        int resign_action) = 0;

    // Object management
    virtual ObjectClassHandle get_object_class_handle(
        const std::string& class_name) = 0;

    virtual std::string get_object_class_name(
        ObjectClassHandle class_handle) = 0;

    virtual AttributeHandle get_attribute_handle(
        ObjectClassHandle class_handle,
        const std::string& attribute_name) = 0;

    virtual void publish_object_class_attributes(
        ObjectClassHandle class_handle,
        const AttributeHandleSet& attributes) = 0;

    virtual void subscribe_object_class_attributes(
        ObjectClassHandle class_handle,
        const AttributeHandleSet& attributes) = 0;

    virtual ObjectInstanceHandle register_object_instance(
        ObjectClassHandle class_handle) = 0;

    virtual ObjectInstanceHandle register_object_instance(
        ObjectClassHandle class_handle,
        const std::string& instance_name) = 0;

    virtual void update_attribute_values(
        ObjectInstanceHandle object_handle,
        const std::map<AttributeHandle, std::vector<uint8_t>>& attributes,
        const std::vector<uint8_t>& user_tag) = 0;

    virtual void update_attribute_values(
        ObjectInstanceHandle object_handle,
        const std::map<AttributeHandle, std::vector<uint8_t>>& attributes,
        const std::vector<uint8_t>& user_tag,
        const LogicalTime& time) = 0;

    virtual void delete_object_instance(
        ObjectInstanceHandle object_handle,
        const std::vector<uint8_t>& user_tag) = 0;

    // Interaction management
    virtual InteractionClassHandle get_interaction_class_handle(
        const std::string& interaction_name) = 0;

    virtual ParameterHandle get_parameter_handle(
        InteractionClassHandle interaction_handle,
        const std::string& parameter_name) = 0;

    virtual void publish_interaction_class(
        InteractionClassHandle interaction_handle) = 0;

    virtual void subscribe_interaction_class(
        InteractionClassHandle interaction_handle) = 0;

    virtual void send_interaction(
        InteractionClassHandle interaction_handle,
        const std::map<ParameterHandle, std::vector<uint8_t>>& parameters,
        const std::vector<uint8_t>& user_tag) = 0;

    virtual void send_interaction(
        InteractionClassHandle interaction_handle,
        const std::map<ParameterHandle, std::vector<uint8_t>>& parameters,
        const std::vector<uint8_t>& user_tag,
        const LogicalTime& time) = 0;

    // Time management
    virtual void enable_time_regulation(
        const LogicalTimeInterval& lookahead) = 0;

    virtual void disable_time_regulation() = 0;

    virtual void enable_time_constrained() = 0;

    virtual void disable_time_constrained() = 0;

    virtual void time_advance_request(const LogicalTime& time) = 0;

    virtual void time_advance_request_available(const LogicalTime& time) = 0;

    virtual void next_message_request(const LogicalTime& time) = 0;

    virtual void next_message_request_available(const LogicalTime& time) = 0;

    virtual void flush_queue_request(const LogicalTime& time) = 0;

    virtual void modify_lookahead(const LogicalTimeInterval& lookahead) = 0;

    virtual LogicalTimeInterval query_lookahead() = 0;

    // Ownership management
    virtual void request_attribute_ownership_assumption(
        ObjectInstanceHandle object_handle,
        const AttributeHandleSet& attributes,
        const std::vector<uint8_t>& user_tag) = 0;

    virtual void unconditional_attribute_ownership_divestiture(
        ObjectInstanceHandle object_handle,
        const AttributeHandleSet& attributes) = 0;

    // DDM (Data Distribution Management)
    virtual void create_region(/* ... */) = 0;
    virtual void delete_region(/* ... */) = 0;

    // Tick (process callbacks)
    virtual bool tick() = 0;
    virtual bool tick(double min_seconds, double max_seconds) = 0;
};

}  // namespace jaguar::federation
```

### Federate Ambassador Interface

```cpp
namespace jaguar::federation {

// Federate Ambassador - callbacks from RTI
class IFederateAmbassador {
public:
    virtual ~IFederateAmbassador() = default;

    // Federation management callbacks
    virtual void connection_lost(const std::string& fault_description) = 0;

    virtual void synchronization_point_registration_succeeded(
        const std::string& label) = 0;

    virtual void synchronization_point_registration_failed(
        const std::string& label,
        int reason) = 0;

    virtual void announce_synchronization_point(
        const std::string& label,
        const std::vector<uint8_t>& user_tag) = 0;

    virtual void federation_synchronized(const std::string& label) = 0;

    // Object management callbacks
    virtual void discover_object_instance(
        ObjectInstanceHandle object_handle,
        ObjectClassHandle object_class,
        const std::string& instance_name) = 0;

    virtual void reflect_attribute_values(
        ObjectInstanceHandle object_handle,
        const std::map<AttributeHandle, std::vector<uint8_t>>& attributes,
        const std::vector<uint8_t>& user_tag,
        int order_type,
        int transport_type) = 0;

    virtual void reflect_attribute_values(
        ObjectInstanceHandle object_handle,
        const std::map<AttributeHandle, std::vector<uint8_t>>& attributes,
        const std::vector<uint8_t>& user_tag,
        int order_type,
        int transport_type,
        const LogicalTime& time) = 0;

    virtual void remove_object_instance(
        ObjectInstanceHandle object_handle,
        const std::vector<uint8_t>& user_tag,
        int order_type) = 0;

    // Interaction callbacks
    virtual void receive_interaction(
        InteractionClassHandle interaction_handle,
        const std::map<ParameterHandle, std::vector<uint8_t>>& parameters,
        const std::vector<uint8_t>& user_tag,
        int order_type,
        int transport_type) = 0;

    virtual void receive_interaction(
        InteractionClassHandle interaction_handle,
        const std::map<ParameterHandle, std::vector<uint8_t>>& parameters,
        const std::vector<uint8_t>& user_tag,
        int order_type,
        int transport_type,
        const LogicalTime& time) = 0;

    // Time management callbacks
    virtual void time_regulation_enabled(const LogicalTime& time) = 0;

    virtual void time_constrained_enabled(const LogicalTime& time) = 0;

    virtual void time_advance_grant(const LogicalTime& time) = 0;

    // Ownership callbacks
    virtual void request_attribute_ownership_release(
        ObjectInstanceHandle object_handle,
        const AttributeHandleSet& attributes,
        const std::vector<uint8_t>& user_tag) = 0;

    virtual void attribute_ownership_unavailable(
        ObjectInstanceHandle object_handle,
        const AttributeHandleSet& attributes) = 0;

    virtual void attribute_ownership_acquisition_notification(
        ObjectInstanceHandle object_handle,
        const AttributeHandleSet& attributes,
        const std::vector<uint8_t>& user_tag) = 0;
};

}  // namespace jaguar::federation
```

### HLA Configuration

```cpp
namespace jaguar::federation {

// HLA configuration
struct HLAConfiguration {
    bool enabled = false;
    std::string federation_name;
    std::string federate_name;
    std::string federate_type;
    std::vector<std::string> fom_modules;
    std::string rti_designator;         // RTI-specific connection string
    LogicalTimeInterval lookahead{0.1};
    bool time_regulating = true;
    bool time_constrained = true;
    int resign_action = 0;              // NO_ACTION, DELETE_OBJECTS, etc.
};

// RTI types supported
enum class RTIType {
    Portico,
    MAK,
    Pitch,
    OpenRTI,
    Generic
};

// Factory function
std::unique_ptr<IRTIAmbassador> create_rti_ambassador(RTIType type = RTIType::Generic);

}  // namespace jaguar::federation
```

---

## Usage Examples

### DIS Entity State Publication

```cpp
#include <jaguar/federation/dis_protocol.h>

using namespace jaguar::federation;

// Configure DIS
DISConfig config;
config.enabled = true;
config.broadcast_address = "192.168.1.255";
config.port = 3000;
config.site_id = 1;
config.application_id = 1;
config.exercise_id = 1;
config.default_dr_algorithm = DeadReckoningAlgorithm::DRM_RVW;
config.position_threshold = 1.0;
config.orientation_threshold = 0.05;

auto codec = create_dis_codec(true);
auto network = create_dis_network(config);
network->initialize(config);

// Create dead reckoning calculator
DeadReckoningCalculator dr;
dr.set_algorithm(DeadReckoningAlgorithm::DRM_RVW);

// Entity state
EntityIdentifier entity_id{1, 1, 100};
EntityType entity_type{1, 2, 225, 1, 2, 0, 0};  // US F-16

// Simulation loop
Vec3 last_published_position;
EulerAngles last_published_orientation;

while (running) {
    auto state = engine.get_entity_state(aircraft);

    // Convert to ECEF
    Vec3 ecef_position = ned_to_ecef(state.position, reference_point);
    Vec3 ecef_velocity = ned_to_ecef_velocity(state.velocity, reference_point);
    EulerAngles orientation = quaternion_to_euler(state.orientation);

    // Check if update needed
    if (dr.needs_update(ecef_position, ecef_velocity, orientation,
                       config.position_threshold, config.orientation_threshold)) {

        // Build Entity State PDU
        EntityStatePDU pdu;
        pdu.header.protocol_version = 7;
        pdu.header.exercise_id = config.exercise_id;
        pdu.header.pdu_type = PDUType::EntityState;
        pdu.header.timestamp = get_dis_timestamp();

        pdu.entity_id = entity_id;
        pdu.force_id = ForceId::Friendly;
        pdu.entity_type = entity_type;
        pdu.location = ecef_position;
        pdu.linear_velocity = ecef_velocity;
        pdu.orientation = orientation;
        pdu.dr_algorithm = config.default_dr_algorithm;

        // Set marking (callsign)
        std::string callsign = "VIPER01";
        std::copy(callsign.begin(), callsign.end(), pdu.marking);

        // Encode and send
        auto data = codec->encode(pdu);
        network->send(data);

        // Update dead reckoning state
        dr.set_state(ecef_position, ecef_velocity,
                    state.acceleration, orientation, state.angular_velocity);
        last_published_position = ecef_position;
        last_published_orientation = orientation;
    }

    // Receive and process incoming PDUs
    auto received = network->receive();
    for (const auto& pdu_data : received) {
        auto type = codec->get_pdu_type(pdu_data);

        switch (type) {
            case PDUType::EntityState: {
                auto pdu = codec->decode_entity_state(pdu_data);
                update_remote_entity(pdu);
                break;
            }
            case PDUType::Fire: {
                auto pdu = codec->decode_fire(pdu_data);
                handle_fire_event(pdu);
                break;
            }
            case PDUType::Detonation: {
                auto pdu = codec->decode_detonation(pdu_data);
                handle_detonation(pdu);
                break;
            }
            default:
                break;
        }
    }

    engine.step(dt);
}

network->shutdown();
```

### HLA Federation

```cpp
#include <jaguar/federation/hla_rti.h>

using namespace jaguar::federation;

// Custom federate ambassador
class MyFederateAmbassador : public IFederateAmbassador {
    bool time_advance_granted_ = false;
    LogicalTime current_time_;
    std::map<ObjectInstanceHandle, RemoteEntity> remote_entities_;

public:
    void discover_object_instance(
            ObjectInstanceHandle handle,
            ObjectClassHandle class_handle,
            const std::string& name) override {
        std::cout << "Discovered object: " << name << "\n";
        remote_entities_[handle] = RemoteEntity{name, class_handle};
    }

    void reflect_attribute_values(
            ObjectInstanceHandle handle,
            const std::map<AttributeHandle, std::vector<uint8_t>>& attributes,
            const std::vector<uint8_t>& tag,
            int order, int transport) override {
        auto& entity = remote_entities_[handle];
        for (const auto& [attr, data] : attributes) {
            entity.update_attribute(attr, data);
        }
    }

    void time_advance_grant(const LogicalTime& time) override {
        current_time_ = time;
        time_advance_granted_ = true;
    }

    bool is_time_advance_granted() const { return time_advance_granted_; }
    void reset_time_advance() { time_advance_granted_ = false; }
    LogicalTime get_current_time() const { return current_time_; }
};

// Main simulation
int main() {
    // Create RTI ambassador
    auto rti = create_rti_ambassador(RTIType::Portico);
    auto federate = std::make_unique<MyFederateAmbassador>();

    // Connect to RTI
    rti->connect("");

    // Create or join federation
    try {
        rti->create_federation_execution("AirCombat",
            {"RPR_FOM_v2.0.xml", "JaguarExtensions.xml"});
    } catch (...) {
        // Federation already exists
    }

    auto federate_handle = rti->join_federation_execution(
        "JaguarSim", "Simulation", "AirCombat");

    // Get handles
    auto aircraft_class = rti->get_object_class_handle("BaseEntity.PhysicalEntity.Platform.Aircraft");
    auto position_attr = rti->get_attribute_handle(aircraft_class, "WorldLocation");
    auto velocity_attr = rti->get_attribute_handle(aircraft_class, "VelocityVector");
    auto orientation_attr = rti->get_attribute_handle(aircraft_class, "Orientation");

    // Publish and subscribe
    AttributeHandleSet attributes{position_attr, velocity_attr, orientation_attr};
    rti->publish_object_class_attributes(aircraft_class, attributes);
    rti->subscribe_object_class_attributes(aircraft_class, attributes);

    // Enable time management
    rti->enable_time_regulation(LogicalTimeInterval{0.1});
    rti->enable_time_constrained();

    // Register local aircraft
    auto aircraft_handle = rti->register_object_instance(aircraft_class, "VIPER01");

    // Simulation loop
    LogicalTime sim_time{0.0};
    double dt = 0.02;

    while (running) {
        // Request time advance
        federate->reset_time_advance();
        rti->time_advance_request(sim_time + LogicalTimeInterval{dt});

        // Process callbacks until grant
        while (!federate->is_time_advance_granted()) {
            rti->tick();
        }

        // Run simulation step
        engine.step(dt);

        // Update HLA attributes
        auto state = engine.get_entity_state(aircraft);
        auto position_data = serialize_position(state.position);
        auto velocity_data = serialize_velocity(state.velocity);
        auto orientation_data = serialize_orientation(state.orientation);

        std::map<AttributeHandle, std::vector<uint8_t>> attr_values;
        attr_values[position_attr] = position_data;
        attr_values[velocity_attr] = velocity_data;
        attr_values[orientation_attr] = orientation_data;

        rti->update_attribute_values(aircraft_handle, attr_values, {}, sim_time);

        sim_time = federate->get_current_time();
    }

    // Cleanup
    rti->delete_object_instance(aircraft_handle, {});
    rti->resign_federation_execution(0);  // NO_ACTION
    rti->destroy_federation_execution("AirCombat");
    rti->disconnect();

    return 0;
}
```

---

## See Also

- [Network Integration](../advanced/networking.md) - High-level networking guide
- [Architecture](../advanced/architecture.md) - System architecture overview
- [Cloud Burst API](cloud.md) - Distributed simulation support
- [Configuration](configuration.md) - Engine configuration
