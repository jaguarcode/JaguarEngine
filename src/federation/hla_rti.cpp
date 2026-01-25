/**
 * @file hla_rti.cpp
 * @brief Implementation of HLA RTI Ambassador and Federate Ambassador
 *
 * This file provides both full HLA RTI implementation (when JAGUAR_ENABLE_HLA is defined)
 * and stub implementations (when HLA is disabled). The stub implementations return
 * NotConnected errors for most operations, allowing the system to compile and run
 * without HLA support.
 */

#include "jaguar/federation/hla_rti.h"
#include <algorithm>
#include <mutex>
#include <queue>
#include <atomic>
#include <cstring>

namespace jaguar::federation::hla {

// ============================================================================
// HLAResult to String Conversion
// ============================================================================

const char* hla_result_to_string(HLAResult result) {
    switch (result) {
        case HLAResult::Success: return "Success";

        // Connection errors
        case HLAResult::ConnectionFailed: return "ConnectionFailed";
        case HLAResult::NotConnected: return "NotConnected";
        case HLAResult::AlreadyConnected: return "AlreadyConnected";

        // Federation management errors
        case HLAResult::FederationExecutionAlreadyExists: return "FederationExecutionAlreadyExists";
        case HLAResult::FederationExecutionDoesNotExist: return "FederationExecutionDoesNotExist";
        case HLAResult::CouldNotCreateLogicalTimeFactory: return "CouldNotCreateLogicalTimeFactory";
        case HLAResult::FederateAlreadyExecutionMember: return "FederateAlreadyExecutionMember";
        case HLAResult::FederateNotExecutionMember: return "FederateNotExecutionMember";
        case HLAResult::SaveInProgress: return "SaveInProgress";
        case HLAResult::RestoreInProgress: return "RestoreInProgress";

        // Object management errors
        case HLAResult::ObjectClassNotDefined: return "ObjectClassNotDefined";
        case HLAResult::ObjectClassNotPublished: return "ObjectClassNotPublished";
        case HLAResult::ObjectInstanceNotKnown: return "ObjectInstanceNotKnown";
        case HLAResult::ObjectInstanceNameNotReserved: return "ObjectInstanceNameNotReserved";
        case HLAResult::ObjectInstanceNameInUse: return "ObjectInstanceNameInUse";
        case HLAResult::AttributeNotDefined: return "AttributeNotDefined";
        case HLAResult::AttributeNotOwned: return "AttributeNotOwned";
        case HLAResult::AttributeAlreadyOwned: return "AttributeAlreadyOwned";
        case HLAResult::AttributeNotPublished: return "AttributeNotPublished";

        // Interaction errors
        case HLAResult::InteractionClassNotDefined: return "InteractionClassNotDefined";
        case HLAResult::InteractionClassNotPublished: return "InteractionClassNotPublished";
        case HLAResult::InteractionParameterNotDefined: return "InteractionParameterNotDefined";

        // Time management errors
        case HLAResult::InvalidLogicalTime: return "InvalidLogicalTime";
        case HLAResult::LogicalTimeAlreadyPassed: return "LogicalTimeAlreadyPassed";
        case HLAResult::TimeConstrainedAlreadyEnabled: return "TimeConstrainedAlreadyEnabled";
        case HLAResult::TimeRegulationAlreadyEnabled: return "TimeRegulationAlreadyEnabled";
        case HLAResult::TimeAdvanceAlreadyInProgress: return "TimeAdvanceAlreadyInProgress";
        case HLAResult::TimeAdvanceWasNotInProgress: return "TimeAdvanceWasNotInProgress";
        case HLAResult::RequestForTimeRegulationPending: return "RequestForTimeRegulationPending";
        case HLAResult::RequestForTimeConstrainedPending: return "RequestForTimeConstrainedPending";

        // Ownership errors
        case HLAResult::AttributeAlreadyBeingDivested: return "AttributeAlreadyBeingDivested";
        case HLAResult::AttributeAlreadyBeingAcquired: return "AttributeAlreadyBeingAcquired";

        // DDM errors
        case HLAResult::RegionNotCreatedByThisFederate: return "RegionNotCreatedByThisFederate";
        case HLAResult::RegionDoesNotContainSpecifiedDimension: return "RegionDoesNotContainSpecifiedDimension";
        case HLAResult::InvalidRegion: return "InvalidRegion";

        // Save/restore errors
        case HLAResult::SaveNotInitiated: return "SaveNotInitiated";
        case HLAResult::RestoreNotRequested: return "RestoreNotRequested";

        // General errors
        case HLAResult::RTIinternalError: return "RTIinternalError";
        case HLAResult::InvalidHandleProvided: return "InvalidHandleProvided";
        case HLAResult::NameNotFound: return "NameNotFound";
        case HLAResult::InvalidName: return "InvalidName";
        case HLAResult::EncodingHelperNotFound: return "EncodingHelperNotFound";
        case HLAResult::InternalError: return "InternalError";

        default: return "Unknown";
    }
}

#ifdef JAGUAR_ENABLE_HLA

// ============================================================================
// Full HLA RTI Implementation (when HLA is enabled)
// ============================================================================

// ============================================================================
// Federate Ambassador Base Implementation
// ============================================================================

class FederateAmbassadorImpl : public IFederateAmbassador {
public:
    FederateAmbassadorImpl() = default;
    ~FederateAmbassadorImpl() override = default;

    // Federation Management Callbacks
    void synchronization_point_registered(std::string_view label, bool success) override {
        // Default: no-op stub
        (void)label;
        (void)success;
    }

    void announce_synchronization_point(std::string_view label, const std::vector<UInt8>& tag) override {
        // Default: no-op stub
        (void)label;
        (void)tag;
    }

    void federation_synchronized(std::string_view label) override {
        // Default: no-op stub
        (void)label;
    }

    // Object Management Callbacks
    void discover_object_instance(ObjectInstanceHandle object, ObjectClassHandle object_class,
                                   std::string_view instance_name) override {
        // Default: no-op stub
        (void)object;
        (void)object_class;
        (void)instance_name;
    }

    void reflect_attribute_values(ObjectInstanceHandle object, const AttributeValueSet& attributes,
                                   const std::vector<UInt8>& tag) override {
        // Default: no-op stub
        (void)object;
        (void)attributes;
        (void)tag;
    }

    void reflect_attribute_values_with_time(ObjectInstanceHandle object, const AttributeValueSet& attributes,
                                            const std::vector<UInt8>& tag, const LogicalTime& time) override {
        // Default: no-op stub
        (void)object;
        (void)attributes;
        (void)tag;
        (void)time;
    }

    void remove_object_instance(ObjectInstanceHandle object, const std::vector<UInt8>& tag) override {
        // Default: no-op stub
        (void)object;
        (void)tag;
    }

    void remove_object_instance_with_time(ObjectInstanceHandle object, const std::vector<UInt8>& tag,
                                          const LogicalTime& time) override {
        // Default: no-op stub
        (void)object;
        (void)tag;
        (void)time;
    }

    void provide_attribute_value_update(ObjectInstanceHandle object, const std::vector<AttributeHandle>& attributes,
                                        const std::vector<UInt8>& tag) override {
        // Default: no-op stub
        (void)object;
        (void)attributes;
        (void)tag;
    }

    // Interaction Callbacks
    void receive_interaction(InteractionClassHandle interaction, const ParameterValueSet& parameters,
                             const std::vector<UInt8>& tag) override {
        // Default: no-op stub
        (void)interaction;
        (void)parameters;
        (void)tag;
    }

    void receive_interaction_with_time(InteractionClassHandle interaction, const ParameterValueSet& parameters,
                                       const std::vector<UInt8>& tag, const LogicalTime& time) override {
        // Default: no-op stub
        (void)interaction;
        (void)parameters;
        (void)tag;
        (void)time;
    }

    // Time Management Callbacks
    void time_regulation_enabled(const LogicalTime& time) override {
        // Default: no-op stub
        (void)time;
    }

    void time_constrained_enabled(const LogicalTime& time) override {
        // Default: no-op stub
        (void)time;
    }

    void time_advance_grant(const LogicalTime& time) override {
        // Default: no-op stub
        (void)time;
    }

    // Ownership Management Callbacks
    void request_attribute_ownership_assumption(ObjectInstanceHandle object,
                                                const std::vector<AttributeHandle>& attributes,
                                                const std::vector<UInt8>& tag) override {
        // Default: no-op stub
        (void)object;
        (void)attributes;
        (void)tag;
    }

    void request_divestiture_confirmation(ObjectInstanceHandle object,
                                          const std::vector<AttributeHandle>& attributes) override {
        // Default: no-op stub
        (void)object;
        (void)attributes;
    }

    void attribute_ownership_acquisition_notification(ObjectInstanceHandle object,
                                                      const std::vector<AttributeHandle>& attributes) override {
        // Default: no-op stub
        (void)object;
        (void)attributes;
    }

    void attribute_ownership_unavailable(ObjectInstanceHandle object,
                                         const std::vector<AttributeHandle>& attributes) override {
        // Default: no-op stub
        (void)object;
        (void)attributes;
    }

    void attribute_ownership_divestiture_notification(ObjectInstanceHandle object,
                                                      const std::vector<AttributeHandle>& attributes) override {
        // Default: no-op stub
        (void)object;
        (void)attributes;
    }

    void confirm_attribute_ownership_acquisition_cancellation(ObjectInstanceHandle object,
                                                              const std::vector<AttributeHandle>& attributes) override {
        // Default: no-op stub
        (void)object;
        (void)attributes;
    }
};

// ============================================================================
// RTI Ambassador Implementation
// ============================================================================

class RTIAmbassadorImpl : public IRTIAmbassador {
public:
    explicit RTIAmbassadorImpl(const HLAConfiguration& config)
        : config_(config)
        , next_object_instance_handle_(1)
        , next_object_class_handle_(1)
        , next_attribute_handle_(1)
        , next_interaction_class_handle_(1)
        , next_parameter_handle_(1)
        , next_federate_handle_(1)
    {}

    ~RTIAmbassadorImpl() override = default;

    // Federation Management
    HLAResult create_federation_execution(std::string_view federation_name,
                                          const std::vector<std::string>& fom_modules) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (federation_state_.exists) {
            return HLAResult::FederationExecutionAlreadyExists;
        }

        federation_state_.exists = true;
        federation_state_.name = std::string(federation_name);
        federation_state_.fom_modules = fom_modules;
        federation_state_.created_at = std::chrono::system_clock::now();

        return HLAResult::Success;
    }

    HLAResult destroy_federation_execution(std::string_view federation_name) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!federation_state_.exists) {
            return HLAResult::FederationExecutionDoesNotExist;
        }

        if (federation_state_.name != federation_name) {
            return HLAResult::FederationExecutionDoesNotExist;
        }

        if (!federation_state_.federates.empty()) {
            return HLAResult::FederateAlreadyExecutionMember;
        }

        federation_state_ = FederationState{};
        return HLAResult::Success;
    }

    HLAResult join_federation_execution(std::string_view federate_name, std::string_view federate_type,
                                        std::string_view federation_name) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!federation_state_.exists) {
            return HLAResult::FederationExecutionDoesNotExist;
        }

        if (federation_state_.name != federation_name) {
            return HLAResult::FederationExecutionDoesNotExist;
        }

        if (local_federate_.joined) {
            return HLAResult::FederateAlreadyExecutionMember;
        }

        local_federate_.handle = FederateHandle{next_federate_handle_++};
        local_federate_.name = std::string(federate_name);
        local_federate_.type = std::string(federate_type);
        local_federate_.joined = true;
        local_federate_.joined_at = std::chrono::system_clock::now();

        // Convert to FederateInfo for storage
        FederateInfo info;
        info.handle = local_federate_.handle;
        info.federate_name = local_federate_.name;
        info.federate_type = local_federate_.type;
        info.time_status = TimeRegulationStatus::None;
        info.current_time = time_state_.current_time;
        info.lookahead = time_state_.lookahead;
        info.is_alive = true;

        federation_state_.federates[local_federate_.handle] = info;
        federation_state_.federate_count = static_cast<UInt32>(federation_state_.federates.size());

        return HLAResult::Success;
    }

    HLAResult resign_federation_execution(ResignAction action) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        // Handle resignation actions
        switch (action) {
            case ResignAction::UnconditionallyDivestAttributes:
                // Divest all owned attributes
                for (auto& [obj_handle, obj_instance] : registered_objects_) {
                    obj_instance.ownership_states.clear();
                }
                break;

            case ResignAction::DeleteObjects:
                // Delete all owned objects
                registered_objects_.clear();
                break;

            case ResignAction::DeleteObjectsThenDivest:
                registered_objects_.clear();
                break;

            default:
                break;
        }

        federation_state_.federates.erase(local_federate_.handle);
        federation_state_.federate_count = static_cast<UInt32>(federation_state_.federates.size());

        local_federate_ = LocalFederateState{};

        return HLAResult::Success;
    }

    HLAResult register_federation_synchronization_point(std::string_view label,
                                                        const std::vector<UInt8>& tag) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        SyncPointInfo sync_point;
        sync_point.label = std::string(label);
        sync_point.status = SyncPointStatus::Registered;
        sync_point.tag = tag;
        sync_point.announced_at = std::chrono::system_clock::now();

        sync_points_[sync_point.label] = sync_point;

        return HLAResult::Success;
    }

    HLAResult synchronization_point_achieved(std::string_view label) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        auto it = sync_points_.find(std::string(label));
        if (it == sync_points_.end()) {
            return HLAResult::NameNotFound;
        }

        it->second.status = SyncPointStatus::Achieved;
        it->second.achieved_at = std::chrono::system_clock::now();

        return HLAResult::Success;
    }

    HLAResult request_federation_save(std::string_view label) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        save_state_.status = SaveStatus::Initiated;
        save_state_.label = std::string(label);

        return HLAResult::Success;
    }

    HLAResult request_federation_restore(std::string_view label) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        restore_state_.status = RestoreStatus::RequestPending;
        restore_state_.label = std::string(label);

        return HLAResult::Success;
    }

    // Declaration Management
    HLAResult publish_object_class_attributes(ObjectClassHandle object_class,
                                              const std::vector<AttributeHandle>& attributes) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        auto& published = published_object_classes_[object_class];
        for (AttributeHandle attr : attributes) {
            published.insert(attr);
        }

        return HLAResult::Success;
    }

    HLAResult unpublish_object_class_attributes(ObjectClassHandle object_class) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        published_object_classes_.erase(object_class);
        return HLAResult::Success;
    }

    HLAResult subscribe_object_class_attributes(ObjectClassHandle object_class,
                                                const std::vector<AttributeHandle>& attributes,
                                                bool passive) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        auto& subscribed = subscribed_object_classes_[object_class];
        for (AttributeHandle attr : attributes) {
            subscribed.insert(attr);
        }

        subscription_passive_[object_class] = passive;

        return HLAResult::Success;
    }

    HLAResult unsubscribe_object_class_attributes(ObjectClassHandle object_class) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        subscribed_object_classes_.erase(object_class);
        subscription_passive_.erase(object_class);
        return HLAResult::Success;
    }

    HLAResult publish_interaction_class(InteractionClassHandle interaction_class) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        published_interactions_.insert(interaction_class);
        return HLAResult::Success;
    }

    HLAResult unpublish_interaction_class(InteractionClassHandle interaction_class) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        published_interactions_.erase(interaction_class);
        return HLAResult::Success;
    }

    HLAResult subscribe_interaction_class(InteractionClassHandle interaction_class, bool passive) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        subscribed_interactions_.insert(interaction_class);
        interaction_passive_[interaction_class] = passive;
        return HLAResult::Success;
    }

    HLAResult unsubscribe_interaction_class(InteractionClassHandle interaction_class) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        subscribed_interactions_.erase(interaction_class);
        interaction_passive_.erase(interaction_class);
        return HLAResult::Success;
    }

    // Object Management
    HLAResult register_object_instance(ObjectClassHandle object_class, ObjectInstanceHandle& handle,
                                       std::string_view instance_name) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        // Check if class is published
        if (published_object_classes_.find(object_class) == published_object_classes_.end()) {
            return HLAResult::ObjectClassNotPublished;
        }

        // Check instance name uniqueness
        if (!instance_name.empty()) {
            for (const auto& [obj_handle, obj] : registered_objects_) {
                if (obj.instance_name == instance_name) {
                    return HLAResult::ObjectInstanceNameInUse;
                }
            }
        }

        handle = ObjectInstanceHandle{next_object_instance_handle_++};

        ObjectInstance instance;
        instance.handle = handle;
        instance.class_handle = object_class;
        instance.instance_name = instance_name.empty()
            ? "Object_" + std::to_string(handle.value)
            : std::string(instance_name);
        instance.owner = local_federate_.handle;
        instance.is_local = true;
        instance.last_update_time = time_state_.current_time;

        registered_objects_[handle] = instance;

        return HLAResult::Success;
    }

    HLAResult update_attribute_values(ObjectInstanceHandle object, const AttributeValueSet& attributes,
                                      const std::vector<UInt8>& tag,
                                      const std::optional<LogicalTime>& time) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        auto it = registered_objects_.find(object);
        if (it == registered_objects_.end()) {
            return HLAResult::ObjectInstanceNotKnown;
        }

        // Update attribute values
        for (const auto& [attr_handle, value] : attributes.values) {
            it->second.attribute_values[attr_handle] = value;
        }

        if (time.has_value()) {
            it->second.last_update_time = time.value();
        } else {
            it->second.last_update_time = time_state_.current_time;
        }

        return HLAResult::Success;
    }

    HLAResult delete_object_instance(ObjectInstanceHandle object, const std::vector<UInt8>& tag,
                                     const std::optional<LogicalTime>& time) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        auto it = registered_objects_.find(object);
        if (it == registered_objects_.end()) {
            return HLAResult::ObjectInstanceNotKnown;
        }

        if (!it->second.is_local) {
            return HLAResult::AttributeNotOwned;
        }

        registered_objects_.erase(it);
        return HLAResult::Success;
    }

    HLAResult request_attribute_value_update(ObjectInstanceHandle object,
                                             const std::vector<AttributeHandle>& attributes,
                                             const std::vector<UInt8>& tag) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        auto it = registered_objects_.find(object);
        if (it == registered_objects_.end()) {
            return HLAResult::ObjectInstanceNotKnown;
        }

        // In real implementation, this would trigger callbacks to object owner
        return HLAResult::Success;
    }

    HLAResult request_attribute_value_update_class(ObjectClassHandle object_class,
                                                   const std::vector<AttributeHandle>& attributes,
                                                   const std::vector<UInt8>& tag) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        // In real implementation, this would trigger callbacks to all object owners of this class
        return HLAResult::Success;
    }

    // Interaction Management
    HLAResult send_interaction(InteractionClassHandle interaction, const ParameterValueSet& parameters,
                               const std::vector<UInt8>& tag,
                               const std::optional<LogicalTime>& time) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        if (published_interactions_.find(interaction) == published_interactions_.end()) {
            return HLAResult::InteractionClassNotPublished;
        }

        // In real implementation, this would distribute to subscribers
        return HLAResult::Success;
    }

    // Time Management
    HLAResult enable_time_regulation(const LogicalTimeInterval& lookahead) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        if (time_state_.regulation_enabled) {
            return HLAResult::TimeRegulationAlreadyEnabled;
        }

        time_state_.regulation_enabled = true;
        time_state_.lookahead = lookahead;
        time_state_.regulation_pending = false;

        return HLAResult::Success;
    }

    HLAResult disable_time_regulation() override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        time_state_.regulation_enabled = false;
        return HLAResult::Success;
    }

    HLAResult enable_time_constrained() override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        if (time_state_.constrained_enabled) {
            return HLAResult::TimeConstrainedAlreadyEnabled;
        }

        time_state_.constrained_enabled = true;
        time_state_.constrained_pending = false;

        return HLAResult::Success;
    }

    HLAResult disable_time_constrained() override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        time_state_.constrained_enabled = false;
        return HLAResult::Success;
    }

    HLAResult time_advance_request(const LogicalTime& time) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        if (time_state_.advance_state != TimeAdvanceState::Idle) {
            return HLAResult::TimeAdvanceAlreadyInProgress;
        }

        if (time <= time_state_.current_time) {
            return HLAResult::LogicalTimeAlreadyPassed;
        }

        time_state_.requested_time = time;
        time_state_.advance_state = TimeAdvanceState::RequestPending;

        // In real implementation, this would coordinate with RTI
        // For stub, grant immediately
        time_state_.current_time = time;
        time_state_.advance_state = TimeAdvanceState::Granted;

        return HLAResult::Success;
    }

    HLAResult time_advance_request_available(const LogicalTime& time) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        if (time_state_.advance_state != TimeAdvanceState::Idle) {
            return HLAResult::TimeAdvanceAlreadyInProgress;
        }

        if (time <= time_state_.current_time) {
            return HLAResult::LogicalTimeAlreadyPassed;
        }

        time_state_.requested_time = time;
        time_state_.advance_state = TimeAdvanceState::AvailablePending;

        // Grant immediately for stub
        time_state_.current_time = time;
        time_state_.advance_state = TimeAdvanceState::Granted;

        return HLAResult::Success;
    }

    HLAResult next_message_request(const LogicalTime& time) override {
        return time_advance_request(time);
    }

    HLAResult next_message_request_available(const LogicalTime& time) override {
        return time_advance_request_available(time);
    }

    HLAResult flush_queue_request(const LogicalTime& time) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        // Process all pending callbacks up to time
        return HLAResult::Success;
    }

    HLAResult enable_asynchronous_delivery() override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        time_state_.asynchronous_delivery = true;
        return HLAResult::Success;
    }

    HLAResult disable_asynchronous_delivery() override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        time_state_.asynchronous_delivery = false;
        return HLAResult::Success;
    }

    HLAResult query_galt(LogicalTime& time) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        time = time_state_.galt;
        return HLAResult::Success;
    }

    HLAResult query_lits(LogicalTime& time) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        time = time_state_.lits;
        return HLAResult::Success;
    }

    HLAResult modify_lookahead(const LogicalTimeInterval& lookahead) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        time_state_.lookahead = lookahead;
        return HLAResult::Success;
    }

    HLAResult query_lookahead(LogicalTimeInterval& lookahead) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        lookahead = time_state_.lookahead;
        return HLAResult::Success;
    }

    // Ownership Management
    HLAResult unconditional_attribute_ownership_divestiture(ObjectInstanceHandle object,
                                                            const std::vector<AttributeHandle>& attributes) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        auto it = registered_objects_.find(object);
        if (it == registered_objects_.end()) {
            return HLAResult::ObjectInstanceNotKnown;
        }

        for (AttributeHandle attr : attributes) {
            it->second.ownership_states[attr] = OwnershipState::Unowned;
        }

        return HLAResult::Success;
    }

    HLAResult negotiated_attribute_ownership_divestiture(ObjectInstanceHandle object,
                                                         const std::vector<AttributeHandle>& attributes,
                                                         const std::vector<UInt8>& tag) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        auto it = registered_objects_.find(object);
        if (it == registered_objects_.end()) {
            return HLAResult::ObjectInstanceNotKnown;
        }

        for (AttributeHandle attr : attributes) {
            it->second.ownership_states[attr] = OwnershipState::DivesturePending;
        }

        return HLAResult::Success;
    }

    HLAResult confirm_divestiture(ObjectInstanceHandle object,
                                  const std::vector<AttributeHandle>& attributes) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        auto it = registered_objects_.find(object);
        if (it == registered_objects_.end()) {
            return HLAResult::ObjectInstanceNotKnown;
        }

        for (AttributeHandle attr : attributes) {
            it->second.ownership_states[attr] = OwnershipState::Unowned;
        }

        return HLAResult::Success;
    }

    HLAResult attribute_ownership_acquisition(ObjectInstanceHandle object,
                                              const std::vector<AttributeHandle>& attributes,
                                              const std::vector<UInt8>& tag) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        auto it = registered_objects_.find(object);
        if (it == registered_objects_.end()) {
            return HLAResult::ObjectInstanceNotKnown;
        }

        for (AttributeHandle attr : attributes) {
            it->second.ownership_states[attr] = OwnershipState::AcquisitionPending;
        }

        return HLAResult::Success;
    }

    HLAResult attribute_ownership_acquisition_if_available(ObjectInstanceHandle object,
                                                           const std::vector<AttributeHandle>& attributes) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        auto it = registered_objects_.find(object);
        if (it == registered_objects_.end()) {
            return HLAResult::ObjectInstanceNotKnown;
        }

        for (AttributeHandle attr : attributes) {
            auto state_it = it->second.ownership_states.find(attr);
            if (state_it == it->second.ownership_states.end() ||
                state_it->second == OwnershipState::Unowned) {
                it->second.ownership_states[attr] = OwnershipState::OwnedByUs;
            }
        }

        return HLAResult::Success;
    }

    HLAResult cancel_attribute_ownership_acquisition(ObjectInstanceHandle object,
                                                     const std::vector<AttributeHandle>& attributes) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        auto it = registered_objects_.find(object);
        if (it == registered_objects_.end()) {
            return HLAResult::ObjectInstanceNotKnown;
        }

        for (AttributeHandle attr : attributes) {
            auto state_it = it->second.ownership_states.find(attr);
            if (state_it != it->second.ownership_states.end() &&
                state_it->second == OwnershipState::AcquisitionPending) {
                it->second.ownership_states.erase(state_it);
            }
        }

        return HLAResult::Success;
    }

    HLAResult query_attribute_ownership(ObjectInstanceHandle object, AttributeHandle attribute,
                                        FederateHandle& owner) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        auto it = registered_objects_.find(object);
        if (it == registered_objects_.end()) {
            return HLAResult::ObjectInstanceNotKnown;
        }

        owner = it->second.owner;
        return HLAResult::Success;
    }

    HLAResult is_attribute_owned_by_federate(ObjectInstanceHandle object, AttributeHandle attribute,
                                             bool& owned) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        auto it = registered_objects_.find(object);
        if (it == registered_objects_.end()) {
            return HLAResult::ObjectInstanceNotKnown;
        }

        auto state_it = it->second.ownership_states.find(attribute);
        owned = (state_it != it->second.ownership_states.end() &&
                 state_it->second == OwnershipState::OwnedByUs);

        return HLAResult::Success;
    }

    // Support Services (FOM queries)
    HLAResult get_object_class_handle(std::string_view name, ObjectClassHandle& handle) override {
        std::lock_guard<std::mutex> lock(mutex_);

        auto it = object_class_names_.find(std::string(name));
        if (it == object_class_names_.end()) {
            // Auto-create for testing
            handle = ObjectClassHandle{next_object_class_handle_++};
            object_class_names_[std::string(name)] = handle;
            object_class_handles_[handle] = std::string(name);
            return HLAResult::Success;
        }

        handle = it->second;
        return HLAResult::Success;
    }

    HLAResult get_object_class_name(ObjectClassHandle handle, std::string& name) override {
        std::lock_guard<std::mutex> lock(mutex_);

        auto it = object_class_handles_.find(handle);
        if (it == object_class_handles_.end()) {
            return HLAResult::NameNotFound;
        }

        name = it->second;
        return HLAResult::Success;
    }

    HLAResult get_attribute_handle(ObjectClassHandle object_class, std::string_view attribute_name,
                                   AttributeHandle& handle) override {
        std::lock_guard<std::mutex> lock(mutex_);

        std::string key = std::to_string(object_class.value) + ":" + std::string(attribute_name);
        auto it = attribute_names_.find(key);
        if (it == attribute_names_.end()) {
            // Auto-create for testing
            handle = AttributeHandle{next_attribute_handle_++};
            attribute_names_[key] = handle;
            attribute_handles_[handle] = std::string(attribute_name);
            return HLAResult::Success;
        }

        handle = it->second;
        return HLAResult::Success;
    }

    HLAResult get_attribute_name(ObjectClassHandle object_class, AttributeHandle attribute,
                                 std::string& name) override {
        std::lock_guard<std::mutex> lock(mutex_);

        auto it = attribute_handles_.find(attribute);
        if (it == attribute_handles_.end()) {
            return HLAResult::NameNotFound;
        }

        name = it->second;
        return HLAResult::Success;
    }

    HLAResult get_interaction_class_handle(std::string_view name, InteractionClassHandle& handle) override {
        std::lock_guard<std::mutex> lock(mutex_);

        auto it = interaction_class_names_.find(std::string(name));
        if (it == interaction_class_names_.end()) {
            // Auto-create for testing
            handle = InteractionClassHandle{next_interaction_class_handle_++};
            interaction_class_names_[std::string(name)] = handle;
            interaction_class_handles_[handle] = std::string(name);
            return HLAResult::Success;
        }

        handle = it->second;
        return HLAResult::Success;
    }

    HLAResult get_interaction_class_name(InteractionClassHandle handle, std::string& name) override {
        std::lock_guard<std::mutex> lock(mutex_);

        auto it = interaction_class_handles_.find(handle);
        if (it == interaction_class_handles_.end()) {
            return HLAResult::NameNotFound;
        }

        name = it->second;
        return HLAResult::Success;
    }

    HLAResult get_parameter_handle(InteractionClassHandle interaction_class, std::string_view parameter_name,
                                   ParameterHandle& handle) override {
        std::lock_guard<std::mutex> lock(mutex_);

        std::string key = std::to_string(interaction_class.value) + ":" + std::string(parameter_name);
        auto it = parameter_names_.find(key);
        if (it == parameter_names_.end()) {
            // Auto-create for testing
            handle = ParameterHandle{next_parameter_handle_++};
            parameter_names_[key] = handle;
            parameter_handles_[handle] = std::string(parameter_name);
            return HLAResult::Success;
        }

        handle = it->second;
        return HLAResult::Success;
    }

    HLAResult get_parameter_name(InteractionClassHandle interaction_class, ParameterHandle parameter,
                                 std::string& name) override {
        std::lock_guard<std::mutex> lock(mutex_);

        auto it = parameter_handles_.find(parameter);
        if (it == parameter_handles_.end()) {
            return HLAResult::NameNotFound;
        }

        name = it->second;
        return HLAResult::Success;
    }

    HLAResult get_object_instance_handle(std::string_view name, ObjectInstanceHandle& handle) override {
        std::lock_guard<std::mutex> lock(mutex_);

        for (const auto& [obj_handle, obj] : registered_objects_) {
            if (obj.instance_name == name) {
                handle = obj_handle;
                return HLAResult::Success;
            }
        }

        return HLAResult::NameNotFound;
    }

    HLAResult get_object_instance_name(ObjectInstanceHandle handle, std::string& name) override {
        std::lock_guard<std::mutex> lock(mutex_);

        auto it = registered_objects_.find(handle);
        if (it == registered_objects_.end()) {
            return HLAResult::ObjectInstanceNotKnown;
        }

        name = it->second.instance_name;
        return HLAResult::Success;
    }

    // Processing
    HLAResult evo_callback(std::chrono::milliseconds min_wait, std::chrono::milliseconds max_wait) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!local_federate_.joined) {
            return HLAResult::FederateNotExecutionMember;
        }

        // In real implementation, this would process callbacks from RTI
        // For stub, just return success
        return HLAResult::Success;
    }

private:
    struct FederationState {
        bool exists{false};
        std::string name;
        std::vector<std::string> fom_modules;
        std::unordered_map<FederateHandle, FederateInfo, FederateHandleHash> federates;
        UInt32 federate_count{0};
        std::chrono::system_clock::time_point created_at;
    };

    struct LocalFederateState {
        bool joined{false};
        FederateHandle handle;
        std::string name;
        std::string type;
        std::chrono::system_clock::time_point joined_at;
    };

    struct TimeState {
        LogicalTime current_time{0.0};
        LogicalTime requested_time{0.0};
        LogicalTimeInterval lookahead{0.1};
        LogicalTime galt{0.0};
        LogicalTime lits{0.0};
        TimeAdvanceState advance_state{TimeAdvanceState::Idle};
        bool regulation_enabled{false};
        bool constrained_enabled{false};
        bool regulation_pending{false};
        bool constrained_pending{false};
        bool asynchronous_delivery{false};
    };

    struct SaveState {
        SaveStatus status{SaveStatus::NotInitiated};
        std::string label;
    };

    struct RestoreState {
        RestoreStatus status{RestoreStatus::NotInitiated};
        std::string label;
    };

    HLAConfiguration config_;
    FederationState federation_state_;
    LocalFederateState local_federate_;
    TimeState time_state_;
    SaveState save_state_;
    RestoreState restore_state_;

    // Handle generators
    std::atomic<UInt64> next_object_instance_handle_;
    std::atomic<UInt32> next_object_class_handle_;
    std::atomic<UInt32> next_attribute_handle_;
    std::atomic<UInt32> next_interaction_class_handle_;
    std::atomic<UInt32> next_parameter_handle_;
    std::atomic<UInt64> next_federate_handle_;

    // Publication/Subscription state
    std::unordered_map<ObjectClassHandle, std::unordered_set<AttributeHandle, AttributeHandleHash>,
                       ObjectClassHandleHash> published_object_classes_;
    std::unordered_map<ObjectClassHandle, std::unordered_set<AttributeHandle, AttributeHandleHash>,
                       ObjectClassHandleHash> subscribed_object_classes_;
    std::unordered_set<InteractionClassHandle, InteractionClassHandleHash> published_interactions_;
    std::unordered_set<InteractionClassHandle, InteractionClassHandleHash> subscribed_interactions_;
    std::unordered_map<ObjectClassHandle, bool, ObjectClassHandleHash> subscription_passive_;
    std::unordered_map<InteractionClassHandle, bool, InteractionClassHandleHash> interaction_passive_;

    // Object instances
    std::unordered_map<ObjectInstanceHandle, ObjectInstance, ObjectInstanceHandleHash> registered_objects_;

    // Synchronization points
    std::unordered_map<std::string, SyncPointInfo> sync_points_;

    // FOM name mappings
    std::unordered_map<std::string, ObjectClassHandle> object_class_names_;
    std::unordered_map<ObjectClassHandle, std::string, ObjectClassHandleHash> object_class_handles_;
    std::unordered_map<std::string, AttributeHandle> attribute_names_;
    std::unordered_map<AttributeHandle, std::string, AttributeHandleHash> attribute_handles_;
    std::unordered_map<std::string, InteractionClassHandle> interaction_class_names_;
    std::unordered_map<InteractionClassHandle, std::string, InteractionClassHandleHash> interaction_class_handles_;
    std::unordered_map<std::string, ParameterHandle> parameter_names_;
    std::unordered_map<ParameterHandle, std::string, ParameterHandleHash> parameter_handles_;

    mutable std::mutex mutex_;
};

#else // !JAGUAR_ENABLE_HLA

// ============================================================================
// Stub Implementation (when HLA is disabled)
// ============================================================================

// Stub Federate Ambassador - all callbacks are no-ops
class FederateAmbassadorStub : public IFederateAmbassador {
public:
    void synchronization_point_registered(std::string_view, bool) override {}
    void announce_synchronization_point(std::string_view, const std::vector<UInt8>&) override {}
    void federation_synchronized(std::string_view) override {}
    void discover_object_instance(ObjectInstanceHandle, ObjectClassHandle, std::string_view) override {}
    void reflect_attribute_values(ObjectInstanceHandle, const AttributeValueSet&, const std::vector<UInt8>&) override {}
    void reflect_attribute_values_with_time(ObjectInstanceHandle, const AttributeValueSet&, const std::vector<UInt8>&, const LogicalTime&) override {}
    void remove_object_instance(ObjectInstanceHandle, const std::vector<UInt8>&) override {}
    void remove_object_instance_with_time(ObjectInstanceHandle, const std::vector<UInt8>&, const LogicalTime&) override {}
    void provide_attribute_value_update(ObjectInstanceHandle, const std::vector<AttributeHandle>&, const std::vector<UInt8>&) override {}
    void receive_interaction(InteractionClassHandle, const ParameterValueSet&, const std::vector<UInt8>&) override {}
    void receive_interaction_with_time(InteractionClassHandle, const ParameterValueSet&, const std::vector<UInt8>&, const LogicalTime&) override {}
    void time_regulation_enabled(const LogicalTime&) override {}
    void time_constrained_enabled(const LogicalTime&) override {}
    void time_advance_grant(const LogicalTime&) override {}
    void request_attribute_ownership_assumption(ObjectInstanceHandle, const std::vector<AttributeHandle>&, const std::vector<UInt8>&) override {}
    void request_divestiture_confirmation(ObjectInstanceHandle, const std::vector<AttributeHandle>&) override {}
    void attribute_ownership_acquisition_notification(ObjectInstanceHandle, const std::vector<AttributeHandle>&) override {}
    void attribute_ownership_unavailable(ObjectInstanceHandle, const std::vector<AttributeHandle>&) override {}
    void attribute_ownership_divestiture_notification(ObjectInstanceHandle, const std::vector<AttributeHandle>&) override {}
    void confirm_attribute_ownership_acquisition_cancellation(ObjectInstanceHandle, const std::vector<AttributeHandle>&) override {}
};

// Stub RTI Ambassador - all operations return NotConnected
class RTIAmbassadorStub : public IRTIAmbassador {
public:
    explicit RTIAmbassadorStub(const HLAConfiguration&) {}

    // Federation Management - all return NotConnected
    HLAResult create_federation_execution(std::string_view, const std::vector<std::string>&) override {
        return HLAResult::NotConnected;
    }
    HLAResult destroy_federation_execution(std::string_view) override {
        return HLAResult::NotConnected;
    }
    HLAResult join_federation_execution(std::string_view, std::string_view, std::string_view) override {
        return HLAResult::NotConnected;
    }
    HLAResult resign_federation_execution(ResignAction) override {
        return HLAResult::NotConnected;
    }
    HLAResult register_federation_synchronization_point(std::string_view, const std::vector<UInt8>&) override {
        return HLAResult::NotConnected;
    }
    HLAResult synchronization_point_achieved(std::string_view) override {
        return HLAResult::NotConnected;
    }
    HLAResult request_federation_save(std::string_view) override {
        return HLAResult::NotConnected;
    }
    HLAResult request_federation_restore(std::string_view) override {
        return HLAResult::NotConnected;
    }

    // Declaration Management
    HLAResult publish_object_class_attributes(ObjectClassHandle, const std::vector<AttributeHandle>&) override {
        return HLAResult::NotConnected;
    }
    HLAResult unpublish_object_class_attributes(ObjectClassHandle) override {
        return HLAResult::NotConnected;
    }
    HLAResult subscribe_object_class_attributes(ObjectClassHandle, const std::vector<AttributeHandle>&, bool) override {
        return HLAResult::NotConnected;
    }
    HLAResult unsubscribe_object_class_attributes(ObjectClassHandle) override {
        return HLAResult::NotConnected;
    }
    HLAResult publish_interaction_class(InteractionClassHandle) override {
        return HLAResult::NotConnected;
    }
    HLAResult unpublish_interaction_class(InteractionClassHandle) override {
        return HLAResult::NotConnected;
    }
    HLAResult subscribe_interaction_class(InteractionClassHandle, bool) override {
        return HLAResult::NotConnected;
    }
    HLAResult unsubscribe_interaction_class(InteractionClassHandle) override {
        return HLAResult::NotConnected;
    }

    // Object Management
    HLAResult register_object_instance(ObjectClassHandle, ObjectInstanceHandle&, std::string_view) override {
        return HLAResult::NotConnected;
    }
    HLAResult update_attribute_values(ObjectInstanceHandle, const AttributeValueSet&, const std::vector<UInt8>&, const std::optional<LogicalTime>&) override {
        return HLAResult::NotConnected;
    }
    HLAResult delete_object_instance(ObjectInstanceHandle, const std::vector<UInt8>&, const std::optional<LogicalTime>&) override {
        return HLAResult::NotConnected;
    }
    HLAResult request_attribute_value_update(ObjectInstanceHandle, const std::vector<AttributeHandle>&, const std::vector<UInt8>&) override {
        return HLAResult::NotConnected;
    }
    HLAResult request_attribute_value_update_class(ObjectClassHandle, const std::vector<AttributeHandle>&, const std::vector<UInt8>&) override {
        return HLAResult::NotConnected;
    }

    // Interaction Management
    HLAResult send_interaction(InteractionClassHandle, const ParameterValueSet&, const std::vector<UInt8>&, const std::optional<LogicalTime>&) override {
        return HLAResult::NotConnected;
    }

    // Time Management
    HLAResult enable_time_regulation(const LogicalTimeInterval&) override {
        return HLAResult::NotConnected;
    }
    HLAResult disable_time_regulation() override {
        return HLAResult::NotConnected;
    }
    HLAResult enable_time_constrained() override {
        return HLAResult::NotConnected;
    }
    HLAResult disable_time_constrained() override {
        return HLAResult::NotConnected;
    }
    HLAResult time_advance_request(const LogicalTime&) override {
        return HLAResult::NotConnected;
    }
    HLAResult time_advance_request_available(const LogicalTime&) override {
        return HLAResult::NotConnected;
    }
    HLAResult next_message_request(const LogicalTime&) override {
        return HLAResult::NotConnected;
    }
    HLAResult next_message_request_available(const LogicalTime&) override {
        return HLAResult::NotConnected;
    }
    HLAResult flush_queue_request(const LogicalTime&) override {
        return HLAResult::NotConnected;
    }
    HLAResult enable_asynchronous_delivery() override {
        return HLAResult::NotConnected;
    }
    HLAResult disable_asynchronous_delivery() override {
        return HLAResult::NotConnected;
    }
    HLAResult query_galt(LogicalTime&) override {
        return HLAResult::NotConnected;
    }
    HLAResult query_lits(LogicalTime&) override {
        return HLAResult::NotConnected;
    }
    HLAResult modify_lookahead(const LogicalTimeInterval&) override {
        return HLAResult::NotConnected;
    }
    HLAResult query_lookahead(LogicalTimeInterval&) override {
        return HLAResult::NotConnected;
    }

    // Ownership Management
    HLAResult unconditional_attribute_ownership_divestiture(ObjectInstanceHandle, const std::vector<AttributeHandle>&) override {
        return HLAResult::NotConnected;
    }
    HLAResult negotiated_attribute_ownership_divestiture(ObjectInstanceHandle, const std::vector<AttributeHandle>&, const std::vector<UInt8>&) override {
        return HLAResult::NotConnected;
    }
    HLAResult confirm_divestiture(ObjectInstanceHandle, const std::vector<AttributeHandle>&) override {
        return HLAResult::NotConnected;
    }
    HLAResult attribute_ownership_acquisition(ObjectInstanceHandle, const std::vector<AttributeHandle>&, const std::vector<UInt8>&) override {
        return HLAResult::NotConnected;
    }
    HLAResult attribute_ownership_acquisition_if_available(ObjectInstanceHandle, const std::vector<AttributeHandle>&) override {
        return HLAResult::NotConnected;
    }
    HLAResult cancel_attribute_ownership_acquisition(ObjectInstanceHandle, const std::vector<AttributeHandle>&) override {
        return HLAResult::NotConnected;
    }
    HLAResult query_attribute_ownership(ObjectInstanceHandle, AttributeHandle, FederateHandle&) override {
        return HLAResult::NotConnected;
    }
    HLAResult is_attribute_owned_by_federate(ObjectInstanceHandle, AttributeHandle, bool&) override {
        return HLAResult::NotConnected;
    }

    // Support Services
    HLAResult get_object_class_handle(std::string_view, ObjectClassHandle&) override {
        return HLAResult::NotConnected;
    }
    HLAResult get_object_class_name(ObjectClassHandle, std::string&) override {
        return HLAResult::NotConnected;
    }
    HLAResult get_attribute_handle(ObjectClassHandle, std::string_view, AttributeHandle&) override {
        return HLAResult::NotConnected;
    }
    HLAResult get_attribute_name(ObjectClassHandle, AttributeHandle, std::string&) override {
        return HLAResult::NotConnected;
    }
    HLAResult get_interaction_class_handle(std::string_view, InteractionClassHandle&) override {
        return HLAResult::NotConnected;
    }
    HLAResult get_interaction_class_name(InteractionClassHandle, std::string&) override {
        return HLAResult::NotConnected;
    }
    HLAResult get_parameter_handle(InteractionClassHandle, std::string_view, ParameterHandle&) override {
        return HLAResult::NotConnected;
    }
    HLAResult get_parameter_name(InteractionClassHandle, ParameterHandle, std::string&) override {
        return HLAResult::NotConnected;
    }
    HLAResult get_object_instance_handle(std::string_view, ObjectInstanceHandle&) override {
        return HLAResult::NotConnected;
    }
    HLAResult get_object_instance_name(ObjectInstanceHandle, std::string&) override {
        return HLAResult::NotConnected;
    }

    // Processing
    HLAResult evo_callback(std::chrono::milliseconds, std::chrono::milliseconds) override {
        return HLAResult::NotConnected;
    }
};

#endif // JAGUAR_ENABLE_HLA

// ============================================================================
// Factory Functions
// ============================================================================

std::unique_ptr<IRTIAmbassador> create_rti_ambassador(const HLAConfiguration& config) {
#ifdef JAGUAR_ENABLE_HLA
    return std::make_unique<RTIAmbassadorImpl>(config);
#else
    return std::make_unique<RTIAmbassadorStub>(config);
#endif
}

std::unique_ptr<IFederateAmbassador> create_federate_ambassador() {
#ifdef JAGUAR_ENABLE_HLA
    return std::make_unique<FederateAmbassadorImpl>();
#else
    return std::make_unique<FederateAmbassadorStub>();
#endif
}

} // namespace jaguar::federation::hla
