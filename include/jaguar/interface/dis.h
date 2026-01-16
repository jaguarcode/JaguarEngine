#pragma once
/**
 * @file dis.h
 * @brief DIS (Distributed Interactive Simulation) IEEE 1278 protocol interface
 *
 * Implements DIS protocol for networked simulation interoperability.
 * Supports PDU version 6 and 7 with backward compatibility.
 */

#include "jaguar/core/types.h"
#include "jaguar/physics/entity.h"

#include <cstdint>
#include <memory>
#include <functional>
#include <vector>
#include <string>
#include <chrono>

namespace jaguar::interface::dis {

// ============================================================================
// DIS Enumerations (IEEE 1278.1)
// ============================================================================

/**
 * @brief DIS Protocol Version
 */
enum class ProtocolVersion : uint8_t {
    DIS_PDU_VERSION_5 = 5,  // IEEE 1278-1995
    DIS_PDU_VERSION_6 = 6,  // IEEE 1278.1-1995
    DIS_PDU_VERSION_7 = 7   // IEEE 1278.1a-1998
};

/**
 * @brief PDU Type enumeration
 */
enum class PduType : uint8_t {
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
    IFF_ATC_NAVAIDS = 28,
    UnderwaterAcoustic = 29,
    SupplementalEmissionEntityState = 30,
    IntercomSignal = 31,
    IntercomControl = 32,
    AggregateState = 33,
    IsGroupOf = 34,
    TransferControl = 35,
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

/**
 * @brief Protocol Family enumeration
 */
enum class ProtocolFamily : uint8_t {
    Other = 0,
    EntityInformation = 1,
    Warfare = 2,
    Logistics = 3,
    RadioCommunication = 4,
    SimulationManagement = 5,
    DistributedEmission = 6,
    EntityManagement = 7,
    Minefield = 8,
    SyntheticEnvironment = 9,
    SimulationManagementReliable = 10,
    LiveEntity = 11,
    NonRealTime = 12
};

/**
 * @brief Force ID enumeration
 */
enum class ForceId : uint8_t {
    Other = 0,
    Friendly = 1,
    Opposing = 2,
    Neutral = 3
};

/**
 * @brief Dead Reckoning Algorithm enumeration
 */
enum class DeadReckoningAlgorithm : uint8_t {
    Other = 0,
    Static = 1,                  // DRM_FPW - Fixed position
    DRM_FPW = 2,                 // Fixed position world
    DRM_RPW = 3,                 // Rotation position world
    DRM_RVW = 4,                 // Rotation velocity world
    DRM_FVW = 5,                 // Fixed velocity world
    DRM_FPB = 6,                 // Fixed position body
    DRM_RPB = 7,                 // Rotation position body
    DRM_RVB = 8,                 // Rotation velocity body
    DRM_FVB = 9                  // Fixed velocity body
};

// ============================================================================
// DIS Data Types
// ============================================================================

/**
 * @brief Entity Identifier
 */
struct EntityIdentifier {
    uint16_t site_id{0};
    uint16_t application_id{0};
    uint16_t entity_id{0};

    bool operator==(const EntityIdentifier& other) const {
        return site_id == other.site_id &&
               application_id == other.application_id &&
               entity_id == other.entity_id;
    }

    bool operator!=(const EntityIdentifier& other) const {
        return !(*this == other);
    }
};

/**
 * @brief Entity Type (DIS SISO enumeration)
 */
struct EntityType {
    uint8_t kind{0};           // Platform, Munition, Life form, etc.
    uint8_t domain{0};         // Land, Air, Surface, Subsurface, Space
    uint16_t country{0};       // Country code
    uint8_t category{0};       // Main category
    uint8_t subcategory{0};    // Subcategory
    uint8_t specific{0};       // Specific type
    uint8_t extra{0};          // Extra info

    bool operator==(const EntityType& other) const {
        return kind == other.kind && domain == other.domain &&
               country == other.country && category == other.category &&
               subcategory == other.subcategory && specific == other.specific &&
               extra == other.extra;
    }
};

/**
 * @brief World Coordinates (double precision geocentric)
 */
struct WorldCoordinates {
    double x{0.0};  // ECEF X (meters)
    double y{0.0};  // ECEF Y (meters)
    double z{0.0};  // ECEF Z (meters)
};

/**
 * @brief Linear Velocity Vector
 */
struct VelocityVector {
    float x{0.0f};  // m/s
    float y{0.0f};
    float z{0.0f};
};

/**
 * @brief Euler Angles (orientation)
 */
struct EulerAngles {
    float psi{0.0f};    // Heading (rad)
    float theta{0.0f};  // Pitch (rad)
    float phi{0.0f};    // Roll (rad)
};

/**
 * @brief Angular Velocity
 */
struct AngularVelocity {
    float x{0.0f};  // rad/s about body X
    float y{0.0f};  // rad/s about body Y
    float z{0.0f};  // rad/s about body Z
};

/**
 * @brief Linear Acceleration
 */
struct LinearAcceleration {
    float x{0.0f};  // m/s²
    float y{0.0f};
    float z{0.0f};
};

/**
 * @brief Dead Reckoning Parameters
 */
struct DeadReckoningParameters {
    DeadReckoningAlgorithm algorithm{DeadReckoningAlgorithm::DRM_FVW};
    uint8_t other_params[15]{0};
    LinearAcceleration linear_acceleration;
    AngularVelocity angular_velocity;
};

/**
 * @brief Entity Marking (callsign)
 */
struct EntityMarking {
    uint8_t character_set{1};  // 1 = ASCII
    char characters[11]{0};    // Null-terminated string
};

/**
 * @brief Articulation Parameter
 */
struct ArticulationParameter {
    uint8_t parameter_type_designator{0};
    uint8_t change_indicator{0};
    uint16_t articulation_attachment_id{0};
    uint32_t parameter_type{0};
    uint64_t parameter_value{0};
};

/**
 * @brief Burst Descriptor (for Fire/Detonation PDUs)
 */
struct BurstDescriptor {
    EntityType munition;
    uint16_t warhead{0};
    uint16_t fuse{0};
    uint16_t quantity{0};
    uint16_t rate{0};
};

/**
 * @brief Event Identifier
 */
struct EventIdentifier {
    uint16_t site_id{0};
    uint16_t application_id{0};
    uint16_t event_number{0};
};

// ============================================================================
// PDU Structures
// ============================================================================

/**
 * @brief PDU Header (common to all PDUs)
 */
struct PduHeader {
    ProtocolVersion protocol_version{ProtocolVersion::DIS_PDU_VERSION_7};
    uint8_t exercise_id{1};
    PduType pdu_type{PduType::EntityState};
    ProtocolFamily protocol_family{ProtocolFamily::EntityInformation};
    uint32_t timestamp{0};
    uint16_t pdu_length{0};
    uint16_t padding{0};
};

/**
 * @brief Entity State PDU
 *
 * Communicates position, orientation, velocity, and appearance
 * of entities in the simulation.
 */
struct EntityStatePdu {
    PduHeader header;

    // Entity identification
    EntityIdentifier entity_id;
    ForceId force_id{ForceId::Other};
    uint8_t num_articulation_params{0};
    EntityType entity_type;
    EntityType alternative_entity_type;

    // Kinematic state
    VelocityVector linear_velocity;
    WorldCoordinates location;
    EulerAngles orientation;

    // Appearance
    uint32_t appearance{0};
    DeadReckoningParameters dead_reckoning;

    // Marking
    EntityMarking marking;

    // Capabilities
    uint32_t capabilities{0};

    // Articulation parameters (variable length)
    std::vector<ArticulationParameter> articulation_params;

    EntityStatePdu() {
        header.pdu_type = PduType::EntityState;
        header.protocol_family = ProtocolFamily::EntityInformation;
    }
};

/**
 * @brief Fire PDU
 *
 * Communicates the firing of a weapon.
 */
struct FirePdu {
    PduHeader header;

    EntityIdentifier firing_entity_id;
    EntityIdentifier target_entity_id;
    EntityIdentifier munition_id;
    EventIdentifier event_id;

    uint32_t fire_mission_index{0};
    WorldCoordinates location_in_world;
    BurstDescriptor burst_descriptor;
    VelocityVector velocity;
    float range{0.0f};

    FirePdu() {
        header.pdu_type = PduType::Fire;
        header.protocol_family = ProtocolFamily::Warfare;
    }
};

/**
 * @brief Detonation PDU
 *
 * Communicates the impact or detonation of a munition.
 */
struct DetonationPdu {
    PduHeader header;

    EntityIdentifier firing_entity_id;
    EntityIdentifier target_entity_id;
    EntityIdentifier munition_id;
    EventIdentifier event_id;

    VelocityVector velocity;
    WorldCoordinates location_in_world;
    BurstDescriptor burst_descriptor;

    // Location relative to target entity
    struct {
        float x{0.0f};
        float y{0.0f};
        float z{0.0f};
    } location_in_entity;

    uint8_t detonation_result{0};
    uint8_t num_articulation_params{0};
    uint16_t padding{0};

    std::vector<ArticulationParameter> articulation_params;

    DetonationPdu() {
        header.pdu_type = PduType::Detonation;
        header.protocol_family = ProtocolFamily::Warfare;
    }
};

// ============================================================================
// DIS Adapter Interface
// ============================================================================

/**
 * @brief Network configuration for DIS
 */
struct DISConfig {
    std::string broadcast_address{"239.1.2.3"};
    uint16_t port{3000};
    uint16_t site_id{1};
    uint16_t application_id{1};
    uint8_t exercise_id{1};
    ProtocolVersion version{ProtocolVersion::DIS_PDU_VERSION_7};
    double heartbeat_interval{5.0};     // seconds
    double dead_reckoning_threshold{1.0}; // meters
    bool enable_receive{true};
    bool enable_send{true};
};

/**
 * @brief Callback types for received PDUs
 */
using EntityStateCallback = std::function<void(const EntityStatePdu&)>;
using FireCallback = std::function<void(const FirePdu&)>;
using DetonationCallback = std::function<void(const DetonationPdu&)>;

/**
 * @brief DIS Network Adapter
 *
 * Manages DIS protocol communication for networked simulation.
 */
class DISAdapter {
public:
    DISAdapter();
    explicit DISAdapter(const DISConfig& config);
    ~DISAdapter();

    // Non-copyable
    DISAdapter(const DISAdapter&) = delete;
    DISAdapter& operator=(const DISAdapter&) = delete;

    // ========================================================================
    // Lifecycle
    // ========================================================================

    /**
     * @brief Initialize network connection
     * @return true if successful
     */
    bool initialize();

    /**
     * @brief Shutdown network connection
     */
    void shutdown();

    /**
     * @brief Check if adapter is connected
     */
    bool is_connected() const;

    /**
     * @brief Get current configuration
     */
    const DISConfig& get_config() const;

    /**
     * @brief Update configuration (requires reconnect)
     */
    void set_config(const DISConfig& config);

    // ========================================================================
    // Send Operations
    // ========================================================================

    /**
     * @brief Send Entity State PDU
     * @param pdu Entity state to transmit
     * @return true if sent successfully
     */
    bool send_entity_state(const EntityStatePdu& pdu);

    /**
     * @brief Send Fire PDU
     * @param pdu Fire event to transmit
     * @return true if sent successfully
     */
    bool send_fire(const FirePdu& pdu);

    /**
     * @brief Send Detonation PDU
     * @param pdu Detonation event to transmit
     * @return true if sent successfully
     */
    bool send_detonation(const DetonationPdu& pdu);

    /**
     * @brief Send Entity State for JaguarEngine entity
     *
     * Convenience function that converts engine entity state to DIS PDU.
     *
     * @param entity_id JaguarEngine entity ID
     * @param state Entity state
     * @param dis_id DIS entity identifier
     * @param type DIS entity type
     * @return true if sent successfully
     */
    bool send_entity_state(
        EntityId entity_id,
        const physics::EntityState& state,
        const EntityIdentifier& dis_id,
        const EntityType& type);

    // ========================================================================
    // Receive Operations
    // ========================================================================

    /**
     * @brief Process incoming PDUs
     *
     * Call this periodically to process received network data.
     * Invokes registered callbacks for each received PDU.
     */
    void update();

    /**
     * @brief Register callback for Entity State PDUs
     */
    void on_entity_state(EntityStateCallback callback);

    /**
     * @brief Register callback for Fire PDUs
     */
    void on_fire(FireCallback callback);

    /**
     * @brief Register callback for Detonation PDUs
     */
    void on_detonation(DetonationCallback callback);

    // ========================================================================
    // Entity Mapping
    // ========================================================================

    /**
     * @brief Register mapping between engine entity and DIS identifier
     */
    void register_entity(EntityId engine_id, const EntityIdentifier& dis_id);

    /**
     * @brief Unregister entity mapping
     */
    void unregister_entity(EntityId engine_id);

    /**
     * @brief Get DIS identifier for engine entity
     * @return DIS identifier, or default if not found
     */
    EntityIdentifier get_dis_id(EntityId engine_id) const;

    /**
     * @brief Get engine entity ID for DIS identifier
     * @return Engine entity ID, or INVALID_ENTITY_ID if not found
     */
    EntityId get_engine_id(const EntityIdentifier& dis_id) const;

    // ========================================================================
    // Dead Reckoning
    // ========================================================================

    /**
     * @brief Check if entity state has drifted beyond threshold
     *
     * Compares current state with dead-reckoned state to determine
     * if a new Entity State PDU should be sent.
     *
     * @param entity_id Entity to check
     * @param current_state Current entity state
     * @return true if state update should be sent
     */
    bool needs_update(EntityId entity_id, const physics::EntityState& current_state) const;

    /**
     * @brief Get dead-reckoned state for remote entity
     *
     * Extrapolates received state using dead reckoning algorithm.
     *
     * @param dis_id DIS entity identifier
     * @param current_time Current simulation time
     * @return Extrapolated state
     */
    physics::EntityState get_dead_reckoned_state(
        const EntityIdentifier& dis_id,
        Real current_time) const;

    // ========================================================================
    // Statistics
    // ========================================================================

    /**
     * @brief Get count of PDUs sent
     */
    uint64_t get_pdus_sent() const;

    /**
     * @brief Get count of PDUs received
     */
    uint64_t get_pdus_received() const;

    /**
     * @brief Get bytes sent
     */
    uint64_t get_bytes_sent() const;

    /**
     * @brief Get bytes received
     */
    uint64_t get_bytes_received() const;

    /**
     * @brief Reset statistics
     */
    void reset_statistics();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// ============================================================================
// Conversion Utilities
// ============================================================================

namespace convert {

/**
 * @brief Convert JaguarEngine Vec3 (NED) to DIS WorldCoordinates (ECEF)
 */
WorldCoordinates ned_to_ecef(const Vec3& ned, const Vec3& origin_ecef);

/**
 * @brief Convert DIS WorldCoordinates (ECEF) to JaguarEngine Vec3 (NED)
 */
Vec3 ecef_to_ned(const WorldCoordinates& ecef, const Vec3& origin_ecef);

/**
 * @brief Convert JaguarEngine Quat to DIS EulerAngles
 */
EulerAngles quat_to_euler(const Quat& q);

/**
 * @brief Convert DIS EulerAngles to JaguarEngine Quat
 */
Quat euler_to_quat(const EulerAngles& euler);

/**
 * @brief Convert JaguarEngine Domain to DIS domain code
 */
uint8_t domain_to_dis(Domain domain);

/**
 * @brief Convert DIS domain code to JaguarEngine Domain
 */
Domain dis_to_domain(uint8_t dis_domain);

/**
 * @brief Convert JaguarEngine EntityState to DIS EntityStatePdu
 */
EntityStatePdu state_to_pdu(
    const physics::EntityState& state,
    const EntityIdentifier& entity_id,
    const EntityType& entity_type,
    const Vec3& origin_ecef);

/**
 * @brief Convert DIS EntityStatePdu to JaguarEngine EntityState
 */
physics::EntityState pdu_to_state(
    const EntityStatePdu& pdu,
    const Vec3& origin_ecef);

} // namespace convert

} // namespace jaguar::interface::dis
