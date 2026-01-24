// Copyright JaguarEngine Team. All Rights Reserved.
//
// HLA RTI Interface - IEEE 1516-2010 (Evolved) Implementation
//
// Provides comprehensive RTI Ambassador and Federate Ambassador interfaces
// for High Level Architecture distributed simulations. Implements HLA 1516-2010
// (Evolved) standard with support for time management, object/interaction
// management, ownership management, and federation management.

#pragma once

#include "jaguar/core/types.h"
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>
#include <memory>
#include <functional>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <chrono>
#include <variant>

namespace jaguar::federation::hla {

//==============================================================================
// Forward Declarations
//==============================================================================

class IRTIAmbassador;
class IFederateAmbassador;
class HLAEncoder;
class HLADecoder;

//==============================================================================
// HLA Handle Types (IEEE 1516-2010)
//==============================================================================

/// Opaque handle to an object instance in the federation
struct ObjectInstanceHandle {
    UInt64 value{0};

    constexpr ObjectInstanceHandle() noexcept = default;
    constexpr explicit ObjectInstanceHandle(UInt64 v) noexcept : value(v) {}

    constexpr bool is_valid() const noexcept { return value != 0; }
    constexpr bool operator==(const ObjectInstanceHandle& other) const noexcept {
        return value == other.value;
    }
    constexpr bool operator!=(const ObjectInstanceHandle& other) const noexcept {
        return value != other.value;
    }

    static constexpr ObjectInstanceHandle invalid() noexcept { return ObjectInstanceHandle{0}; }
};

/// Opaque handle to an object class
struct ObjectClassHandle {
    UInt32 value{0};

    constexpr ObjectClassHandle() noexcept = default;
    constexpr explicit ObjectClassHandle(UInt32 v) noexcept : value(v) {}

    constexpr bool is_valid() const noexcept { return value != 0; }
    constexpr bool operator==(const ObjectClassHandle& other) const noexcept {
        return value == other.value;
    }
    constexpr bool operator!=(const ObjectClassHandle& other) const noexcept {
        return value != other.value;
    }

    static constexpr ObjectClassHandle invalid() noexcept { return ObjectClassHandle{0}; }
};

/// Opaque handle to an attribute
struct AttributeHandle {
    UInt32 value{0};

    constexpr AttributeHandle() noexcept = default;
    constexpr explicit AttributeHandle(UInt32 v) noexcept : value(v) {}

    constexpr bool is_valid() const noexcept { return value != 0; }
    constexpr bool operator==(const AttributeHandle& other) const noexcept {
        return value == other.value;
    }
    constexpr bool operator!=(const AttributeHandle& other) const noexcept {
        return value != other.value;
    }

    static constexpr AttributeHandle invalid() noexcept { return AttributeHandle{0}; }
};

/// Opaque handle to an interaction class
struct InteractionClassHandle {
    UInt32 value{0};

    constexpr InteractionClassHandle() noexcept = default;
    constexpr explicit InteractionClassHandle(UInt32 v) noexcept : value(v) {}

    constexpr bool is_valid() const noexcept { return value != 0; }
    constexpr bool operator==(const InteractionClassHandle& other) const noexcept {
        return value == other.value;
    }
    constexpr bool operator!=(const InteractionClassHandle& other) const noexcept {
        return value != other.value;
    }

    static constexpr InteractionClassHandle invalid() noexcept { return InteractionClassHandle{0}; }
};

/// Opaque handle to a parameter
struct ParameterHandle {
    UInt32 value{0};

    constexpr ParameterHandle() noexcept = default;
    constexpr explicit ParameterHandle(UInt32 v) noexcept : value(v) {}

    constexpr bool is_valid() const noexcept { return value != 0; }
    constexpr bool operator==(const ParameterHandle& other) const noexcept {
        return value == other.value;
    }
    constexpr bool operator!=(const ParameterHandle& other) const noexcept {
        return value != other.value;
    }

    static constexpr ParameterHandle invalid() noexcept { return ParameterHandle{0}; }
};

/// Opaque handle to a federate
struct FederateHandle {
    UInt64 value{0};

    constexpr FederateHandle() noexcept = default;
    constexpr explicit FederateHandle(UInt64 v) noexcept : value(v) {}

    constexpr bool is_valid() const noexcept { return value != 0; }
    constexpr bool operator==(const FederateHandle& other) const noexcept {
        return value == other.value;
    }
    constexpr bool operator!=(const FederateHandle& other) const noexcept {
        return value != other.value;
    }

    static constexpr FederateHandle invalid() noexcept { return FederateHandle{0}; }
};

/// Opaque handle to a dimension
struct DimensionHandle {
    UInt32 value{0};

    constexpr DimensionHandle() noexcept = default;
    constexpr explicit DimensionHandle(UInt32 v) noexcept : value(v) {}

    constexpr bool is_valid() const noexcept { return value != 0; }
    constexpr bool operator==(const DimensionHandle& other) const noexcept {
        return value == other.value;
    }
    constexpr bool operator!=(const DimensionHandle& other) const noexcept {
        return value != other.value;
    }

    static constexpr DimensionHandle invalid() noexcept { return DimensionHandle{0}; }
};

/// Opaque handle to a region
struct RegionHandle {
    UInt64 value{0};

    constexpr RegionHandle() noexcept = default;
    constexpr explicit RegionHandle(UInt64 v) noexcept : value(v) {}

    constexpr bool is_valid() const noexcept { return value != 0; }
    constexpr bool operator==(const RegionHandle& other) const noexcept {
        return value == other.value;
    }
    constexpr bool operator!=(const RegionHandle& other) const noexcept {
        return value != other.value;
    }

    static constexpr RegionHandle invalid() noexcept { return RegionHandle{0}; }
};

//==============================================================================
// Hash Functions for Handle Types
//==============================================================================

struct ObjectInstanceHandleHash {
    size_t operator()(const ObjectInstanceHandle& h) const noexcept {
        return std::hash<UInt64>{}(h.value);
    }
};

struct ObjectClassHandleHash {
    size_t operator()(const ObjectClassHandle& h) const noexcept {
        return std::hash<UInt32>{}(h.value);
    }
};

struct AttributeHandleHash {
    size_t operator()(const AttributeHandle& h) const noexcept {
        return std::hash<UInt32>{}(h.value);
    }
};

struct InteractionClassHandleHash {
    size_t operator()(const InteractionClassHandle& h) const noexcept {
        return std::hash<UInt32>{}(h.value);
    }
};

struct ParameterHandleHash {
    size_t operator()(const ParameterHandle& h) const noexcept {
        return std::hash<UInt32>{}(h.value);
    }
};

struct FederateHandleHash {
    size_t operator()(const FederateHandle& h) const noexcept {
        return std::hash<UInt64>{}(h.value);
    }
};

//==============================================================================
// Time Management Types
//==============================================================================

/// Logical time in the federation
struct LogicalTime {
    Float64 time{0.0};

    constexpr LogicalTime() noexcept = default;
    constexpr explicit LogicalTime(Float64 t) noexcept : time(t) {}

    constexpr bool operator==(const LogicalTime& other) const noexcept {
        return time == other.time;
    }
    constexpr bool operator!=(const LogicalTime& other) const noexcept {
        return time != other.time;
    }
    constexpr bool operator<(const LogicalTime& other) const noexcept {
        return time < other.time;
    }
    constexpr bool operator<=(const LogicalTime& other) const noexcept {
        return time <= other.time;
    }
    constexpr bool operator>(const LogicalTime& other) const noexcept {
        return time > other.time;
    }
    constexpr bool operator>=(const LogicalTime& other) const noexcept {
        return time >= other.time;
    }

    constexpr LogicalTime operator+(const LogicalTime& other) const noexcept {
        return LogicalTime{time + other.time};
    }
    constexpr LogicalTime operator-(const LogicalTime& other) const noexcept {
        return LogicalTime{time - other.time};
    }

    constexpr Float64 value() const noexcept { return time; }
};

/// Logical time interval
struct LogicalTimeInterval {
    Float64 interval{0.0};

    constexpr LogicalTimeInterval() noexcept = default;
    constexpr explicit LogicalTimeInterval(Float64 i) noexcept : interval(i) {}

    constexpr bool operator==(const LogicalTimeInterval& other) const noexcept {
        return interval == other.interval;
    }
    constexpr bool operator!=(const LogicalTimeInterval& other) const noexcept {
        return interval != other.interval;
    }

    constexpr Float64 value() const noexcept { return interval; }
};

/// Time regulation status for a federate
enum class TimeRegulationStatus : UInt8 {
    None,           ///< Neither regulating nor constrained
    Regulating,     ///< Time regulating only
    Constrained,    ///< Time constrained only
    Both            ///< Both regulating and constrained
};

/// Convert TimeRegulationStatus to string
inline const char* time_regulation_status_to_string(TimeRegulationStatus status) {
    switch (status) {
        case TimeRegulationStatus::None: return "None";
        case TimeRegulationStatus::Regulating: return "Regulating";
        case TimeRegulationStatus::Constrained: return "Constrained";
        case TimeRegulationStatus::Both: return "Both";
        default: return "Unknown";
    }
}

/// Time advancement state
enum class TimeAdvanceState : UInt8 {
    Idle,               ///< No time advance pending
    RequestPending,     ///< Time advance request pending
    AvailablePending,   ///< Next message available request pending
    Granted             ///< Time advance granted
};

//==============================================================================
// Order Types
//==============================================================================

/// Order type for messages (reflects HLA order semantics)
enum class OrderType : UInt8 {
    Receive,    ///< Receive order (timestamp order)
    Timestamp   ///< Timestamp order (logical time order)
};

/// Transport type for messages
enum class TransportationType : UInt8 {
    Reliable,       ///< Guaranteed delivery (HLA reliable)
    BestEffort      ///< Best effort delivery (HLA best effort)
};

//==============================================================================
// Resignation Actions
//==============================================================================

/// Action to take when resigning from federation
enum class ResignAction : UInt8 {
    UnconditionallyDivestAttributes,        ///< Divest all attributes
    DeleteObjects,                           ///< Delete owned objects
    CancelPendingOwnershipAcquisitions,     ///< Cancel ownership transfers
    DeleteObjectsThenDivest,                ///< Delete objects then divest
    CancelThenDeleteThenDivest,             ///< Cancel, delete, then divest
    NoAction                                 ///< Take no action
};

//==============================================================================
// Ownership Management
//==============================================================================

/// Ownership state for an attribute
enum class OwnershipState : UInt8 {
    Unowned,                ///< No federate owns this attribute
    OwnedByOther,          ///< Another federate owns this attribute
    OwnedByUs,             ///< This federate owns this attribute
    AcquisitionPending,    ///< Ownership acquisition in progress
    DivesturePending       ///< Ownership divestiture in progress
};

/// Attribute ownership divestiture notification
struct AttributeOwnershipDivestiture {
    ObjectInstanceHandle object;
    std::vector<AttributeHandle> attributes;
    std::string reason;
};

/// Attribute ownership acquisition notification
struct AttributeOwnershipAcquisition {
    ObjectInstanceHandle object;
    std::vector<AttributeHandle> attributes;
    FederateHandle previous_owner;
};

//==============================================================================
// Federation Management
//==============================================================================

/// Information about a federation execution
struct FederationExecutionInfo {
    std::string federation_name;
    std::string fom_module_designator;
    LogicalTime current_time{0.0};
    UInt32 federate_count{0};
    bool time_management_enabled{false};
};

/// Information about a federate
struct FederateInfo {
    FederateHandle handle;
    std::string federate_name;
    std::string federate_type;
    TimeRegulationStatus time_status{TimeRegulationStatus::None};
    LogicalTime current_time{0.0};
    LogicalTimeInterval lookahead{0.0};
    bool is_alive{true};
};

/// Synchronization point status
enum class SyncPointStatus : UInt8 {
    Announced,      ///< Sync point has been announced
    Registered,     ///< Sync point registration succeeded
    RegisterFailed, ///< Sync point registration failed
    Achieved,       ///< This federate achieved the sync point
    Synchronized    ///< All federates synchronized on this point
};

/// Synchronization point information
struct SyncPointInfo {
    std::string label;
    SyncPointStatus status{SyncPointStatus::Announced};
    std::vector<UInt8> tag;
    std::chrono::system_clock::time_point announced_at;
    std::chrono::system_clock::time_point achieved_at;
};

//==============================================================================
// Object Management
//==============================================================================

/// Attribute definition
struct AttributeDefinition {
    AttributeHandle handle;
    std::string name;
    OrderType order{OrderType::Receive};
    TransportationType transport{TransportationType::Reliable};
    std::string datatype;
};

/// Object class definition
struct ObjectClassDefinition {
    ObjectClassHandle handle;
    std::string name;
    std::vector<AttributeDefinition> attributes;
    std::optional<ObjectClassHandle> parent_class;
};

/// Object instance representation
struct ObjectInstance {
    ObjectInstanceHandle handle;
    ObjectClassHandle class_handle;
    std::string instance_name;
    FederateHandle owner;
    std::unordered_map<AttributeHandle, std::vector<UInt8>, AttributeHandleHash> attribute_values;
    std::unordered_map<AttributeHandle, OwnershipState, AttributeHandleHash> ownership_states;
    LogicalTime last_update_time{0.0};
    bool is_local{false};  // True if registered by this federate
};

/// Attribute update set
struct AttributeValueSet {
    std::unordered_map<AttributeHandle, std::vector<UInt8>, AttributeHandleHash> values;

    void set_attribute(AttributeHandle handle, std::vector<UInt8> value) {
        values[handle] = std::move(value);
    }

    std::optional<std::vector<UInt8>> get_attribute(AttributeHandle handle) const {
        auto it = values.find(handle);
        if (it != values.end()) {
            return it->second;
        }
        return std::nullopt;
    }

    bool has_attribute(AttributeHandle handle) const {
        return values.find(handle) != values.end();
    }

    size_t size() const { return values.size(); }
    bool empty() const { return values.empty(); }
};

//==============================================================================
// Interaction Management
//==============================================================================

/// Parameter definition
struct ParameterDefinition {
    ParameterHandle handle;
    std::string name;
    std::string datatype;
};

/// Interaction class definition
struct InteractionClassDefinition {
    InteractionClassHandle handle;
    std::string name;
    std::vector<ParameterDefinition> parameters;
    std::optional<InteractionClassHandle> parent_class;
    OrderType order{OrderType::Receive};
    TransportationType transport{TransportationType::Reliable};
};

/// Parameter value set
struct ParameterValueSet {
    std::unordered_map<ParameterHandle, std::vector<UInt8>, ParameterHandleHash> values;

    void set_parameter(ParameterHandle handle, std::vector<UInt8> value) {
        values[handle] = std::move(value);
    }

    std::optional<std::vector<UInt8>> get_parameter(ParameterHandle handle) const {
        auto it = values.find(handle);
        if (it != values.end()) {
            return it->second;
        }
        return std::nullopt;
    }

    bool has_parameter(ParameterHandle handle) const {
        return values.find(handle) != values.end();
    }

    size_t size() const { return values.size(); }
    bool empty() const { return values.empty(); }
};

//==============================================================================
// Data Distribution Management (DDM)
//==============================================================================

/// Dimension definition for routing spaces
struct DimensionDefinition {
    DimensionHandle handle;
    std::string name;
    UInt64 lower_bound{0};
    UInt64 upper_bound{0};
};

/// Routing space definition
struct RoutingSpaceDefinition {
    std::string name;
    std::vector<DimensionDefinition> dimensions;
};

/// Region extent (range along a dimension)
struct RegionExtent {
    DimensionHandle dimension;
    UInt64 lower_bound{0};
    UInt64 upper_bound{0};
};

/// Region for data distribution management
struct Region {
    RegionHandle handle;
    std::vector<RegionExtent> extents;
};

//==============================================================================
// Save/Restore
//==============================================================================

/// Save status
enum class SaveStatus : UInt8 {
    NotInitiated,
    Initiated,
    InProgress,
    Completed,
    Failed
};

/// Restore status
enum class RestoreStatus : UInt8 {
    NotInitiated,
    RequestPending,
    InProgress,
    Completed,
    Failed
};

//==============================================================================
// HLA Configuration
//==============================================================================

/// RTI connection settings
struct RTIConnectionSettings {
    std::string rti_host = "localhost";
    UInt16 rti_port = 8989;
    std::string rti_type = "HLA_EVOLVER";  // HLA_EVOLVER, Portico, MAK, Pitch
    std::chrono::milliseconds connect_timeout{30000};
    std::chrono::milliseconds operation_timeout{10000};
    bool use_tls = false;
    std::string tls_cert_path;
    std::string tls_key_path;
};

/// Publication settings for object classes and attributes
struct PublicationSettings {
    std::unordered_set<ObjectClassHandle, ObjectClassHandleHash> published_object_classes;
    std::unordered_map<ObjectClassHandle, std::unordered_set<AttributeHandle, AttributeHandleHash>, ObjectClassHandleHash> published_attributes;
    std::unordered_set<InteractionClassHandle, InteractionClassHandleHash> published_interactions;
};

/// Subscription settings for object classes and attributes
struct SubscriptionSettings {
    std::unordered_set<ObjectClassHandle, ObjectClassHandleHash> subscribed_object_classes;
    std::unordered_map<ObjectClassHandle, std::unordered_set<AttributeHandle, AttributeHandleHash>, ObjectClassHandleHash> subscribed_attributes;
    std::unordered_set<InteractionClassHandle, InteractionClassHandleHash> subscribed_interactions;
    bool passive_subscription = false;  // Passive vs. active subscription
};

/// Time management settings
struct TimeManagementSettings {
    bool enable_time_regulation = true;
    bool enable_time_constrained = true;
    LogicalTimeInterval lookahead{0.1};  // Default 100ms lookahead
    bool enable_asynchronous_delivery = false;
};

/// HLA configuration
struct HLAConfiguration {
    // Federation identity
    std::string federation_name = "JaguarFederation";
    std::string federate_name = "JaguarEngine";
    std::string federate_type = "SimulationEngine";

    // FOM modules
    std::vector<std::string> fom_module_paths;

    // RTI connection
    RTIConnectionSettings connection;

    // Time management
    TimeManagementSettings time_management;

    // Publication/subscription
    PublicationSettings publication;
    SubscriptionSettings subscription;

    // Optional settings
    bool auto_provide_update = true;
    bool auto_request_update = false;
    std::chrono::milliseconds heartbeat_interval{5000};

    /// Create default configuration
    static HLAConfiguration default_config() {
        HLAConfiguration config;
        config.fom_module_paths.push_back("RPR_FOM_v2.xml");
        return config;
    }
};

//==============================================================================
// Exception Types
//==============================================================================

/// HLA exception result codes
enum class HLAResult : UInt32 {
    Success = 0,

    // Connection errors
    ConnectionFailed,
    NotConnected,
    AlreadyConnected,

    // Federation management errors
    FederationExecutionAlreadyExists,
    FederationExecutionDoesNotExist,
    CouldNotCreateLogicalTimeFactory,
    FederateAlreadyExecutionMember,
    FederateNotExecutionMember,
    SaveInProgress,
    RestoreInProgress,

    // Object management errors
    ObjectClassNotDefined,
    ObjectClassNotPublished,
    ObjectInstanceNotKnown,
    ObjectInstanceNameNotReserved,
    ObjectInstanceNameInUse,
    AttributeNotDefined,
    AttributeNotOwned,
    AttributeAlreadyOwned,
    AttributeNotPublished,

    // Interaction errors
    InteractionClassNotDefined,
    InteractionClassNotPublished,
    InteractionParameterNotDefined,

    // Time management errors
    InvalidLogicalTime,
    LogicalTimeAlreadyPassed,
    TimeConstrainedAlreadyEnabled,
    TimeRegulationAlreadyEnabled,
    TimeAdvanceAlreadyInProgress,
    TimeAdvanceWasNotInProgress,
    RequestForTimeRegulationPending,
    RequestForTimeConstrainedPending,

    // Ownership errors
    AttributeAlreadyBeingDivested,
    AttributeAlreadyBeingAcquired,

    // DDM errors
    RegionNotCreatedByThisFederate,
    RegionDoesNotContainSpecifiedDimension,
    InvalidRegion,

    // Save/restore errors
    SaveNotInitiated,
    RestoreNotRequested,

    // General errors
    RTIinternalError,
    InvalidHandleProvided,
    NameNotFound,
    InvalidName,
    EncodingHelperNotFound,
    InternalError
};

/// Convert HLAResult to string
const char* hla_result_to_string(HLAResult result);

//==============================================================================
// RTI Ambassador Interface
//==============================================================================

/**
 * @brief RTI Ambassador interface (federate to RTI communication)
 *
 * This interface represents the services provided by the RTI to the federate.
 * All methods are synchronous and may throw exceptions on error.
 */
class IRTIAmbassador {
public:
    virtual ~IRTIAmbassador() = default;

    //==========================================================================
    // Federation Management
    //==========================================================================

    /**
     * @brief Create a new federation execution
     * @param federation_name Name of the federation
     * @param fom_modules List of FOM module paths
     * @return Success or error code
     */
    virtual HLAResult create_federation_execution(
        std::string_view federation_name,
        const std::vector<std::string>& fom_modules) = 0;

    /**
     * @brief Destroy a federation execution
     * @param federation_name Name of the federation to destroy
     * @return Success or error code
     */
    virtual HLAResult destroy_federation_execution(
        std::string_view federation_name) = 0;

    /**
     * @brief Join a federation execution
     * @param federate_name Name of this federate
     * @param federate_type Type of this federate
     * @param federation_name Name of the federation to join
     * @return Success or error code (sets federate handle internally)
     */
    virtual HLAResult join_federation_execution(
        std::string_view federate_name,
        std::string_view federate_type,
        std::string_view federation_name) = 0;

    /**
     * @brief Resign from federation execution
     * @param action Action to take on resign
     * @return Success or error code
     */
    virtual HLAResult resign_federation_execution(
        ResignAction action) = 0;

    /**
     * @brief Register federation synchronization point
     * @param label Synchronization point label
     * @param tag Optional user-supplied tag
     * @return Success or error code
     */
    virtual HLAResult register_federation_synchronization_point(
        std::string_view label,
        const std::vector<UInt8>& tag = {}) = 0;

    /**
     * @brief Announce synchronization point achieved
     * @param label Synchronization point label
     * @return Success or error code
     */
    virtual HLAResult synchronization_point_achieved(
        std::string_view label) = 0;

    /**
     * @brief Request federation save
     * @param label Save label
     * @return Success or error code
     */
    virtual HLAResult request_federation_save(
        std::string_view label) = 0;

    /**
     * @brief Request federation restore
     * @param label Restore label
     * @return Success or error code
     */
    virtual HLAResult request_federation_restore(
        std::string_view label) = 0;

    //==========================================================================
    // Declaration Management
    //==========================================================================

    /**
     * @brief Publish object class attributes
     * @param object_class Object class handle
     * @param attributes Set of attribute handles to publish
     * @return Success or error code
     */
    virtual HLAResult publish_object_class_attributes(
        ObjectClassHandle object_class,
        const std::vector<AttributeHandle>& attributes) = 0;

    /**
     * @brief Unpublish object class attributes
     * @param object_class Object class handle
     * @return Success or error code
     */
    virtual HLAResult unpublish_object_class_attributes(
        ObjectClassHandle object_class) = 0;

    /**
     * @brief Subscribe to object class attributes
     * @param object_class Object class handle
     * @param attributes Set of attribute handles to subscribe
     * @param passive Whether to use passive subscription
     * @return Success or error code
     */
    virtual HLAResult subscribe_object_class_attributes(
        ObjectClassHandle object_class,
        const std::vector<AttributeHandle>& attributes,
        bool passive = false) = 0;

    /**
     * @brief Unsubscribe from object class attributes
     * @param object_class Object class handle
     * @return Success or error code
     */
    virtual HLAResult unsubscribe_object_class_attributes(
        ObjectClassHandle object_class) = 0;

    /**
     * @brief Publish interaction class
     * @param interaction_class Interaction class handle
     * @return Success or error code
     */
    virtual HLAResult publish_interaction_class(
        InteractionClassHandle interaction_class) = 0;

    /**
     * @brief Unpublish interaction class
     * @param interaction_class Interaction class handle
     * @return Success or error code
     */
    virtual HLAResult unpublish_interaction_class(
        InteractionClassHandle interaction_class) = 0;

    /**
     * @brief Subscribe to interaction class
     * @param interaction_class Interaction class handle
     * @param passive Whether to use passive subscription
     * @return Success or error code
     */
    virtual HLAResult subscribe_interaction_class(
        InteractionClassHandle interaction_class,
        bool passive = false) = 0;

    /**
     * @brief Unsubscribe from interaction class
     * @param interaction_class Interaction class handle
     * @return Success or error code
     */
    virtual HLAResult unsubscribe_interaction_class(
        InteractionClassHandle interaction_class) = 0;

    //==========================================================================
    // Object Management
    //==========================================================================

    /**
     * @brief Register object instance
     * @param object_class Object class handle
     * @param instance_name Optional instance name
     * @param[out] handle Receives the registered object handle
     * @return Success or error code
     */
    virtual HLAResult register_object_instance(
        ObjectClassHandle object_class,
        ObjectInstanceHandle& handle,
        std::string_view instance_name = "") = 0;

    /**
     * @brief Update attribute values for an object instance
     * @param object Object instance handle
     * @param attributes Attribute value set
     * @param tag Optional user-supplied tag
     * @param time Optional logical time for timestamp order
     * @return Success or error code
     */
    virtual HLAResult update_attribute_values(
        ObjectInstanceHandle object,
        const AttributeValueSet& attributes,
        const std::vector<UInt8>& tag = {},
        const std::optional<LogicalTime>& time = std::nullopt) = 0;

    /**
     * @brief Delete object instance
     * @param object Object instance handle
     * @param tag Optional user-supplied tag
     * @param time Optional logical time for timestamp order
     * @return Success or error code
     */
    virtual HLAResult delete_object_instance(
        ObjectInstanceHandle object,
        const std::vector<UInt8>& tag = {},
        const std::optional<LogicalTime>& time = std::nullopt) = 0;

    /**
     * @brief Request attribute value update
     * @param object Object instance handle
     * @param attributes Attributes to request update for
     * @param tag Optional user-supplied tag
     * @return Success or error code
     */
    virtual HLAResult request_attribute_value_update(
        ObjectInstanceHandle object,
        const std::vector<AttributeHandle>& attributes,
        const std::vector<UInt8>& tag = {}) = 0;

    /**
     * @brief Request attribute value update for object class
     * @param object_class Object class handle
     * @param attributes Attributes to request update for
     * @param tag Optional user-supplied tag
     * @return Success or error code
     */
    virtual HLAResult request_attribute_value_update_class(
        ObjectClassHandle object_class,
        const std::vector<AttributeHandle>& attributes,
        const std::vector<UInt8>& tag = {}) = 0;

    //==========================================================================
    // Interaction Management
    //==========================================================================

    /**
     * @brief Send interaction
     * @param interaction Interaction class handle
     * @param parameters Parameter value set
     * @param tag Optional user-supplied tag
     * @param time Optional logical time for timestamp order
     * @return Success or error code
     */
    virtual HLAResult send_interaction(
        InteractionClassHandle interaction,
        const ParameterValueSet& parameters,
        const std::vector<UInt8>& tag = {},
        const std::optional<LogicalTime>& time = std::nullopt) = 0;

    //==========================================================================
    // Time Management
    //==========================================================================

    /**
     * @brief Enable time regulation
     * @param lookahead Lookahead interval
     * @return Success or error code
     */
    virtual HLAResult enable_time_regulation(
        const LogicalTimeInterval& lookahead) = 0;

    /**
     * @brief Disable time regulation
     * @return Success or error code
     */
    virtual HLAResult disable_time_regulation() = 0;

    /**
     * @brief Enable time constrained
     * @return Success or error code
     */
    virtual HLAResult enable_time_constrained() = 0;

    /**
     * @brief Disable time constrained
     * @return Success or error code
     */
    virtual HLAResult disable_time_constrained() = 0;

    /**
     * @brief Request time advance
     * @param time Target logical time
     * @return Success or error code
     */
    virtual HLAResult time_advance_request(
        const LogicalTime& time) = 0;

    /**
     * @brief Request time advance available
     * @param time Target logical time
     * @return Success or error code
     */
    virtual HLAResult time_advance_request_available(
        const LogicalTime& time) = 0;

    /**
     * @brief Request next message event
     * @param time Target logical time
     * @return Success or error code
     */
    virtual HLAResult next_message_request(
        const LogicalTime& time) = 0;

    /**
     * @brief Request next message event available
     * @param time Target logical time
     * @return Success or error code
     */
    virtual HLAResult next_message_request_available(
        const LogicalTime& time) = 0;

    /**
     * @brief Flush queue request (process all messages up to time)
     * @param time Logical time to flush to
     * @return Success or error code
     */
    virtual HLAResult flush_queue_request(
        const LogicalTime& time) = 0;

    /**
     * @brief Enable asynchronous delivery
     * @return Success or error code
     */
    virtual HLAResult enable_asynchronous_delivery() = 0;

    /**
     * @brief Disable asynchronous delivery
     * @return Success or error code
     */
    virtual HLAResult disable_asynchronous_delivery() = 0;

    /**
     * @brief Query GALT (Greatest Available Logical Time)
     * @param[out] time Receives the GALT
     * @return Success or error code
     */
    virtual HLAResult query_galt(LogicalTime& time) = 0;

    /**
     * @brief Query LITS (Logical time of the next message)
     * @param[out] time Receives the LITS
     * @return Success or error code
     */
    virtual HLAResult query_lits(LogicalTime& time) = 0;

    /**
     * @brief Modify lookahead
     * @param lookahead New lookahead interval
     * @return Success or error code
     */
    virtual HLAResult modify_lookahead(
        const LogicalTimeInterval& lookahead) = 0;

    /**
     * @brief Query lookahead
     * @param[out] lookahead Receives current lookahead
     * @return Success or error code
     */
    virtual HLAResult query_lookahead(
        LogicalTimeInterval& lookahead) = 0;

    //==========================================================================
    // Ownership Management
    //==========================================================================

    /**
     * @brief Unconditionally divest attribute ownership
     * @param object Object instance handle
     * @param attributes Attributes to divest
     * @return Success or error code
     */
    virtual HLAResult unconditional_attribute_ownership_divestiture(
        ObjectInstanceHandle object,
        const std::vector<AttributeHandle>& attributes) = 0;

    /**
     * @brief Negotiate attribute ownership divestiture
     * @param object Object instance handle
     * @param attributes Attributes to divest
     * @param tag Optional user-supplied tag
     * @return Success or error code
     */
    virtual HLAResult negotiated_attribute_ownership_divestiture(
        ObjectInstanceHandle object,
        const std::vector<AttributeHandle>& attributes,
        const std::vector<UInt8>& tag = {}) = 0;

    /**
     * @brief Confirm attribute divestiture
     * @param object Object instance handle
     * @param attributes Attributes to confirm divestiture
     * @return Success or error code
     */
    virtual HLAResult confirm_divestiture(
        ObjectInstanceHandle object,
        const std::vector<AttributeHandle>& attributes) = 0;

    /**
     * @brief Request attribute ownership acquisition
     * @param object Object instance handle
     * @param attributes Attributes to acquire
     * @param tag Optional user-supplied tag
     * @return Success or error code
     */
    virtual HLAResult attribute_ownership_acquisition(
        ObjectInstanceHandle object,
        const std::vector<AttributeHandle>& attributes,
        const std::vector<UInt8>& tag = {}) = 0;

    /**
     * @brief Acquire if available
     * @param object Object instance handle
     * @param attributes Attributes to acquire if available
     * @return Success or error code
     */
    virtual HLAResult attribute_ownership_acquisition_if_available(
        ObjectInstanceHandle object,
        const std::vector<AttributeHandle>& attributes) = 0;

    /**
     * @brief Cancel ownership acquisition
     * @param object Object instance handle
     * @param attributes Attributes to cancel acquisition
     * @return Success or error code
     */
    virtual HLAResult cancel_attribute_ownership_acquisition(
        ObjectInstanceHandle object,
        const std::vector<AttributeHandle>& attributes) = 0;

    /**
     * @brief Query attribute ownership
     * @param object Object instance handle
     * @param attribute Attribute to query
     * @param[out] owner Receives the owner federate handle
     * @return Success or error code
     */
    virtual HLAResult query_attribute_ownership(
        ObjectInstanceHandle object,
        AttributeHandle attribute,
        FederateHandle& owner) = 0;

    /**
     * @brief Check if attribute is owned by this federate
     * @param object Object instance handle
     * @param attribute Attribute to check
     * @param[out] owned Receives true if owned
     * @return Success or error code
     */
    virtual HLAResult is_attribute_owned_by_federate(
        ObjectInstanceHandle object,
        AttributeHandle attribute,
        bool& owned) = 0;

    //==========================================================================
    // Support Services (FOM queries)
    //==========================================================================

    /**
     * @brief Get object class handle by name
     * @param name Object class name
     * @param[out] handle Receives the object class handle
     * @return Success or error code
     */
    virtual HLAResult get_object_class_handle(
        std::string_view name,
        ObjectClassHandle& handle) = 0;

    /**
     * @brief Get object class name
     * @param handle Object class handle
     * @param[out] name Receives the object class name
     * @return Success or error code
     */
    virtual HLAResult get_object_class_name(
        ObjectClassHandle handle,
        std::string& name) = 0;

    /**
     * @brief Get attribute handle
     * @param object_class Object class handle
     * @param attribute_name Attribute name
     * @param[out] handle Receives the attribute handle
     * @return Success or error code
     */
    virtual HLAResult get_attribute_handle(
        ObjectClassHandle object_class,
        std::string_view attribute_name,
        AttributeHandle& handle) = 0;

    /**
     * @brief Get attribute name
     * @param object_class Object class handle
     * @param attribute Attribute handle
     * @param[out] name Receives the attribute name
     * @return Success or error code
     */
    virtual HLAResult get_attribute_name(
        ObjectClassHandle object_class,
        AttributeHandle attribute,
        std::string& name) = 0;

    /**
     * @brief Get interaction class handle by name
     * @param name Interaction class name
     * @param[out] handle Receives the interaction class handle
     * @return Success or error code
     */
    virtual HLAResult get_interaction_class_handle(
        std::string_view name,
        InteractionClassHandle& handle) = 0;

    /**
     * @brief Get interaction class name
     * @param handle Interaction class handle
     * @param[out] name Receives the interaction class name
     * @return Success or error code
     */
    virtual HLAResult get_interaction_class_name(
        InteractionClassHandle handle,
        std::string& name) = 0;

    /**
     * @brief Get parameter handle
     * @param interaction_class Interaction class handle
     * @param parameter_name Parameter name
     * @param[out] handle Receives the parameter handle
     * @return Success or error code
     */
    virtual HLAResult get_parameter_handle(
        InteractionClassHandle interaction_class,
        std::string_view parameter_name,
        ParameterHandle& handle) = 0;

    /**
     * @brief Get parameter name
     * @param interaction_class Interaction class handle
     * @param parameter Parameter handle
     * @param[out] name Receives the parameter name
     * @return Success or error code
     */
    virtual HLAResult get_parameter_name(
        InteractionClassHandle interaction_class,
        ParameterHandle parameter,
        std::string& name) = 0;

    /**
     * @brief Get object instance handle by name
     * @param name Object instance name
     * @param[out] handle Receives the object instance handle
     * @return Success or error code
     */
    virtual HLAResult get_object_instance_handle(
        std::string_view name,
        ObjectInstanceHandle& handle) = 0;

    /**
     * @brief Get object instance name
     * @param handle Object instance handle
     * @param[out] name Receives the object instance name
     * @return Success or error code
     */
    virtual HLAResult get_object_instance_name(
        ObjectInstanceHandle handle,
        std::string& name) = 0;

    //==========================================================================
    // Processing
    //==========================================================================

    /**
     * @brief Process RTI callbacks (tick)
     * @param min_wait Minimum wait time
     * @param max_wait Maximum wait time
     * @return Success or error code
     */
    virtual HLAResult evo_callback(
        std::chrono::milliseconds min_wait,
        std::chrono::milliseconds max_wait) = 0;
};

//==============================================================================
// Federate Ambassador Interface
//==============================================================================

/**
 * @brief Federate Ambassador interface (RTI to federate callbacks)
 *
 * This interface must be implemented by federates to receive callbacks
 * from the RTI. All methods are called by the RTI during evo_callback().
 */
class IFederateAmbassador {
public:
    virtual ~IFederateAmbassador() = default;

    //==========================================================================
    // Federation Management Callbacks
    //==========================================================================

    /**
     * @brief Synchronization point registered callback
     * @param label Synchronization point label
     * @param success True if registration succeeded
     */
    virtual void synchronization_point_registered(
        std::string_view label,
        bool success) = 0;

    /**
     * @brief Announce synchronization point callback
     * @param label Synchronization point label
     * @param tag User-supplied tag
     */
    virtual void announce_synchronization_point(
        std::string_view label,
        const std::vector<UInt8>& tag) = 0;

    /**
     * @brief Federation synchronized callback
     * @param label Synchronization point label
     */
    virtual void federation_synchronized(
        std::string_view label) = 0;

    //==========================================================================
    // Object Management Callbacks
    //==========================================================================

    /**
     * @brief Discover object instance callback
     * @param object Object instance handle
     * @param object_class Object class handle
     * @param instance_name Object instance name
     */
    virtual void discover_object_instance(
        ObjectInstanceHandle object,
        ObjectClassHandle object_class,
        std::string_view instance_name) = 0;

    /**
     * @brief Reflect attribute values callback (receive order)
     * @param object Object instance handle
     * @param attributes Attribute values received
     * @param tag User-supplied tag
     */
    virtual void reflect_attribute_values(
        ObjectInstanceHandle object,
        const AttributeValueSet& attributes,
        const std::vector<UInt8>& tag) = 0;

    /**
     * @brief Reflect attribute values callback (timestamp order)
     * @param object Object instance handle
     * @param attributes Attribute values received
     * @param tag User-supplied tag
     * @param time Logical time of update
     */
    virtual void reflect_attribute_values_with_time(
        ObjectInstanceHandle object,
        const AttributeValueSet& attributes,
        const std::vector<UInt8>& tag,
        const LogicalTime& time) = 0;

    /**
     * @brief Remove object instance callback
     * @param object Object instance handle
     * @param tag User-supplied tag
     */
    virtual void remove_object_instance(
        ObjectInstanceHandle object,
        const std::vector<UInt8>& tag) = 0;

    /**
     * @brief Remove object instance callback (with time)
     * @param object Object instance handle
     * @param tag User-supplied tag
     * @param time Logical time of removal
     */
    virtual void remove_object_instance_with_time(
        ObjectInstanceHandle object,
        const std::vector<UInt8>& tag,
        const LogicalTime& time) = 0;

    /**
     * @brief Provide attribute value update callback
     * @param object Object instance handle
     * @param attributes Attributes requested
     * @param tag User-supplied tag
     */
    virtual void provide_attribute_value_update(
        ObjectInstanceHandle object,
        const std::vector<AttributeHandle>& attributes,
        const std::vector<UInt8>& tag) = 0;

    //==========================================================================
    // Interaction Callbacks
    //==========================================================================

    /**
     * @brief Receive interaction callback (receive order)
     * @param interaction Interaction class handle
     * @param parameters Parameter values received
     * @param tag User-supplied tag
     */
    virtual void receive_interaction(
        InteractionClassHandle interaction,
        const ParameterValueSet& parameters,
        const std::vector<UInt8>& tag) = 0;

    /**
     * @brief Receive interaction callback (timestamp order)
     * @param interaction Interaction class handle
     * @param parameters Parameter values received
     * @param tag User-supplied tag
     * @param time Logical time of interaction
     */
    virtual void receive_interaction_with_time(
        InteractionClassHandle interaction,
        const ParameterValueSet& parameters,
        const std::vector<UInt8>& tag,
        const LogicalTime& time) = 0;

    //==========================================================================
    // Time Management Callbacks
    //==========================================================================

    /**
     * @brief Time regulation enabled callback
     * @param time Logical time when regulation is enabled
     */
    virtual void time_regulation_enabled(
        const LogicalTime& time) = 0;

    /**
     * @brief Time constrained enabled callback
     * @param time Logical time when constrained is enabled
     */
    virtual void time_constrained_enabled(
        const LogicalTime& time) = 0;

    /**
     * @brief Time advance grant callback
     * @param time Granted logical time
     */
    virtual void time_advance_grant(
        const LogicalTime& time) = 0;

    //==========================================================================
    // Ownership Management Callbacks
    //==========================================================================

    /**
     * @brief Request attribute ownership assumption callback
     * @param object Object instance handle
     * @param attributes Attributes being offered
     * @param tag User-supplied tag
     */
    virtual void request_attribute_ownership_assumption(
        ObjectInstanceHandle object,
        const std::vector<AttributeHandle>& attributes,
        const std::vector<UInt8>& tag) = 0;

    /**
     * @brief Request divestiture confirmation callback
     * @param object Object instance handle
     * @param attributes Attributes to divest
     */
    virtual void request_divestiture_confirmation(
        ObjectInstanceHandle object,
        const std::vector<AttributeHandle>& attributes) = 0;

    /**
     * @brief Attribute ownership acquisition notification callback
     * @param object Object instance handle
     * @param attributes Attributes acquired
     */
    virtual void attribute_ownership_acquisition_notification(
        ObjectInstanceHandle object,
        const std::vector<AttributeHandle>& attributes) = 0;

    /**
     * @brief Attribute ownership unavailable callback
     * @param object Object instance handle
     * @param attributes Attributes that are unavailable
     */
    virtual void attribute_ownership_unavailable(
        ObjectInstanceHandle object,
        const std::vector<AttributeHandle>& attributes) = 0;

    /**
     * @brief Attribute ownership divestiture notification callback
     * @param object Object instance handle
     * @param attributes Attributes divested
     */
    virtual void attribute_ownership_divestiture_notification(
        ObjectInstanceHandle object,
        const std::vector<AttributeHandle>& attributes) = 0;

    /**
     * @brief Confirm attribute ownership acquisition cancellation callback
     * @param object Object instance handle
     * @param attributes Attributes whose acquisition was cancelled
     */
    virtual void confirm_attribute_ownership_acquisition_cancellation(
        ObjectInstanceHandle object,
        const std::vector<AttributeHandle>& attributes) = 0;
};

//==============================================================================
// Factory Functions
//==============================================================================

/**
 * @brief Create an RTI Ambassador instance
 * @param config HLA configuration
 * @return Unique pointer to RTI Ambassador
 */
std::unique_ptr<IRTIAmbassador> create_rti_ambassador(
    const HLAConfiguration& config);

/**
 * @brief Create a default Federate Ambassador implementation
 * @return Unique pointer to Federate Ambassador
 */
std::unique_ptr<IFederateAmbassador> create_federate_ambassador();

} // namespace jaguar::federation::hla
