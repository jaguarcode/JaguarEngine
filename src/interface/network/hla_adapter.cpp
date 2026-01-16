/**
 * @file hla_adapter.cpp
 * @brief HLA Network Adapter implementation for JaguarEngine
 *
 * Implements IEEE 1516-2010 (HLA Evolved) federate functionality.
 *
 * @note This implementation provides a reference structure. Actual RTI
 *       calls require linking against an HLA RTI implementation
 *       (Portico, MAK RTI, Pitch RTI, etc.)
 */

#include "jaguar/interface/hla.h"

#include <cstring>
#include <algorithm>
#include <iostream>
#include <thread>

// Conditional RTI headers (uncomment when linking with actual RTI)
// #ifdef JAGUAR_HAS_HLA
// #include <RTI/RTI1516.h>
// #include <RTI/RTIambassador.h>
// #include <RTI/FederateAmbassador.h>
// #endif

namespace jaguar::interface::hla {

// ============================================================================
// RTI Ambassador Wrapper (Stub Implementation)
// ============================================================================

/**
 * @brief RTI Ambassador wrapper class
 *
 * This class wraps the actual RTI ambassador and provides a clean interface.
 * When compiled without an actual RTI, it provides stub implementations
 * that simulate basic HLA behavior for testing.
 */
class HLAAdapter::RTIAmbassador {
public:
    RTIAmbassador() : connected_(false), joined_(false) {}
    ~RTIAmbassador() = default;

    // Connection
    bool connect(const std::string& /*host*/, uint16_t /*port*/) {
        // In actual implementation:
        // rti::RTIambassadorFactory factory;
        // ambassador_ = factory.getRTIambassador();
        // ambassador_->connect(...);
        connected_ = true;
        return true;
    }

    bool disconnect() {
        connected_ = false;
        return true;
    }

    bool is_connected() const { return connected_; }

    // Federation Management
    bool create_federation(const std::string& name, const std::string& /*fom_path*/) {
        federation_name_ = name;
        // In actual implementation:
        // ambassador_->createFederationExecution(name, fom_modules);
        return true;
    }

    bool join_federation(const std::string& federate_name,
                         const std::string& federation_name,
                         FederateAmbassador* /*ambassador*/) {
        // In actual implementation:
        // handle_ = ambassador_->joinFederationExecution(federate_name, ...);
        federate_name_ = federate_name;
        federation_name_ = federation_name;
        joined_ = true;
        return true;
    }

    bool resign_federation() {
        // In actual implementation:
        // ambassador_->resignFederationExecution(NO_ACTION);
        joined_ = false;
        return true;
    }

    bool is_joined() const { return joined_; }

    // Publishing and Subscribing
    bool publish_object_class(ObjectClass /*object_class*/,
                               const std::vector<AttributeHandle>& /*attributes*/) {
        // In actual implementation:
        // ambassador_->publishObjectClassAttributes(classHandle, attributeSet);
        return true;
    }

    bool subscribe_object_class(ObjectClass /*object_class*/,
                                 const std::vector<AttributeHandle>& /*attributes*/) {
        // In actual implementation:
        // ambassador_->subscribeObjectClassAttributes(classHandle, attributeSet);
        return true;
    }

    bool publish_interaction(InteractionClass /*interaction_class*/) {
        // In actual implementation:
        // ambassador_->publishInteractionClass(classHandle);
        return true;
    }

    bool subscribe_interaction(InteractionClass /*interaction_class*/) {
        // In actual implementation:
        // ambassador_->subscribeInteractionClass(classHandle);
        return true;
    }

    // Object Management
    ObjectInstanceHandle register_object(ObjectClass /*object_class*/,
                                         const std::string& name) {
        // In actual implementation:
        // return ambassador_->registerObjectInstance(classHandle, name);
        static ObjectInstanceHandle next_handle = 1000;
        instance_names_[next_handle] = name;
        return next_handle++;
    }

    bool update_attribute_values(ObjectInstanceHandle /*handle*/,
                                  const std::vector<std::pair<AttributeHandle, std::vector<uint8_t>>>& /*values*/,
                                  double /*time*/) {
        // In actual implementation:
        // ambassador_->updateAttributeValues(handle, attributes, tag, time);
        return true;
    }

    bool delete_object(ObjectInstanceHandle handle) {
        // In actual implementation:
        // ambassador_->deleteObjectInstance(handle, tag);
        instance_names_.erase(handle);
        return true;
    }

    // Interaction Management
    bool send_interaction(InteractionClass /*interaction_class*/,
                          const std::vector<std::pair<ParameterHandle, std::vector<uint8_t>>>& /*parameters*/,
                          double /*time*/) {
        // In actual implementation:
        // ambassador_->sendInteraction(classHandle, parameters, tag, time);
        return true;
    }

    // Time Management
    bool enable_time_regulation(double /*lookahead*/) {
        // In actual implementation:
        // ambassador_->enableTimeRegulation(lookahead);
        time_regulating_ = true;
        return true;
    }

    bool enable_time_constrained() {
        // In actual implementation:
        // ambassador_->enableTimeConstrained();
        time_constrained_ = true;
        return true;
    }

    bool time_advance_request(double time) {
        // In actual implementation:
        // ambassador_->timeAdvanceRequest(LogicalTime(time));
        requested_time_ = time;
        // Simulate immediate grant in stub
        granted_time_ = time;
        return true;
    }

    double get_granted_time() const { return granted_time_; }

    // Synchronization
    bool register_sync_point(const std::string& label) {
        // In actual implementation:
        // ambassador_->registerFederationSynchronizationPoint(label, tag);
        sync_points_[label] = true;
        return true;
    }

    bool achieve_sync_point(const std::string& label) {
        // In actual implementation:
        // ambassador_->synchronizationPointAchieved(label);
        return sync_points_.count(label) > 0;
    }

    // Callback Processing
    void evoke_callbacks(uint32_t /*timeout_ms*/) {
        // In actual implementation:
        // ambassador_->evokeCallback(timeout);
        // or ambassador_->evokeMultipleCallbacks(min, max);
    }

private:
    bool connected_;
    bool joined_;
    bool time_regulating_ = false;
    bool time_constrained_ = false;
    double requested_time_ = 0.0;
    double granted_time_ = 0.0;
    std::string federation_name_;
    std::string federate_name_;
    std::unordered_map<ObjectInstanceHandle, std::string> instance_names_;
    std::unordered_map<std::string, bool> sync_points_;
};

// ============================================================================
// Federate Ambassador Implementation
// ============================================================================

class HLAAdapter::FederateAmbassadorImpl : public FederateAmbassador {
public:
    explicit FederateAmbassadorImpl(HLAAdapter& adapter) : adapter_(adapter) {}

    void discoverObjectInstance(
            ObjectInstanceHandle handle,
            ObjectClass object_class,
            const std::string& instance_name) override {
        {
            std::lock_guard<std::mutex> lock(adapter_.objects_mutex_);
            PlatformObject obj;
            obj.handle = handle;
            obj.object_class = object_class;
            obj.instance_name = instance_name;
            obj.is_local = false;
            obj.last_update = std::chrono::steady_clock::now();
            adapter_.objects_[handle] = obj;
        }

        adapter_.stats_.objects_discovered++;

        if (adapter_.object_discovered_callback_) {
            adapter_.object_discovered_callback_(handle, object_class, instance_name);
        }
    }

    void removeObjectInstance(ObjectInstanceHandle handle) override {
        {
            std::lock_guard<std::mutex> lock(adapter_.objects_mutex_);
            adapter_.objects_.erase(handle);
        }

        if (adapter_.object_removed_callback_) {
            adapter_.object_removed_callback_(handle);
        }
    }

    void reflectAttributeValues(
            ObjectInstanceHandle handle,
            const std::vector<std::pair<AttributeHandle, std::vector<uint8_t>>>& attributes,
            double /*time*/) override {
        std::lock_guard<std::mutex> lock(adapter_.objects_mutex_);

        auto it = adapter_.objects_.find(handle);
        if (it == adapter_.objects_.end()) return;

        // Decode attributes
        for (const auto& [attr_handle, data] : attributes) {
            // Attribute handles would be looked up from class handle
            // For now, assume spatial data
            if (attr_handle == 1) {  // Spatial attribute
                adapter_.decode_spatial(data, it->second.spatial);
            }
            // Additional attributes would be decoded here
        }

        it->second.last_update = std::chrono::steady_clock::now();
        adapter_.stats_.attribute_updates_received++;

        if (adapter_.attribute_update_callback_) {
            adapter_.attribute_update_callback_(handle, it->second);
        }
    }

    void receiveInteraction(
            InteractionClass interaction_class,
            const std::vector<std::pair<ParameterHandle, std::vector<uint8_t>>>& parameters,
            double /*time*/) override {
        adapter_.stats_.interactions_received++;

        switch (interaction_class) {
            case InteractionClass::WeaponFire: {
                WeaponFireInteraction fire;
                // Decode parameters into fire structure
                decode_weapon_fire(parameters, fire);
                if (adapter_.weapon_fire_callback_) {
                    adapter_.weapon_fire_callback_(fire);
                }
                break;
            }
            case InteractionClass::MunitionDetonation: {
                MunitionDetonationInteraction det;
                // Decode parameters into detonation structure
                decode_detonation(parameters, det);
                if (adapter_.detonation_callback_) {
                    adapter_.detonation_callback_(det);
                }
                break;
            }
            default:
                break;
        }
    }

    void timeRegulationEnabled(double federate_time) override {
        adapter_.time_regulation_enabled_ = true;
        adapter_.federate_time_ = federate_time;
    }

    void timeConstrainedEnabled(double federate_time) override {
        adapter_.time_constrained_enabled_ = true;
        adapter_.federate_time_ = federate_time;
    }

    void timeAdvanceGrant(double granted_time) override {
        adapter_.granted_time_ = granted_time;
        adapter_.federate_time_ = granted_time;
        adapter_.time_advance_pending_ = false;
        adapter_.stats_.time_advances_granted++;

        if (adapter_.time_advance_callback_) {
            adapter_.time_advance_callback_(granted_time);
        }
    }

    void announceSynchronizationPoint(
            const std::string& label,
            const std::vector<uint8_t>& /*tag*/) override {
        {
            std::lock_guard<std::mutex> lock(adapter_.sync_mutex_);
            adapter_.sync_points_[label] = SyncPointState::Announced;
        }

        if (adapter_.sync_point_callback_) {
            adapter_.sync_point_callback_(label, SyncPointState::Announced);
        }
    }

    void federationSynchronized(const std::string& label) override {
        {
            std::lock_guard<std::mutex> lock(adapter_.sync_mutex_);
            adapter_.sync_points_[label] = SyncPointState::Synchronized;
        }

        if (adapter_.sync_point_callback_) {
            adapter_.sync_point_callback_(label, SyncPointState::Synchronized);
        }
    }

    void requestAttributeOwnershipRelease(
            ObjectInstanceHandle /*handle*/,
            const std::vector<AttributeHandle>& /*attributes*/) override {
        // Handle ownership transfer requests
    }

private:
    HLAAdapter& adapter_;

    void decode_weapon_fire(
            const std::vector<std::pair<ParameterHandle, std::vector<uint8_t>>>& params,
            WeaponFireInteraction& fire) {
        // Decode each parameter based on handle
        for (const auto& [handle, data] : params) {
            // Parameter decoding would go here
            (void)handle;
            (void)data;
        }
        (void)fire;
    }

    void decode_detonation(
            const std::vector<std::pair<ParameterHandle, std::vector<uint8_t>>>& params,
            MunitionDetonationInteraction& det) {
        // Decode each parameter based on handle
        for (const auto& [handle, data] : params) {
            // Parameter decoding would go here
            (void)handle;
            (void)data;
        }
        (void)det;
    }
};

// ============================================================================
// HLA Adapter Implementation
// ============================================================================

HLAAdapter::HLAAdapter()
    : state_(FederateState::NotConnected)
    , federate_time_(0.0)
    , granted_time_(0.0)
    , time_advance_pending_(false)
    , time_regulation_enabled_(false)
    , time_constrained_enabled_(false) {
}

HLAAdapter::~HLAAdapter() {
    shutdown();
}

bool HLAAdapter::initialize(const HLAConfig& config) {
    config_ = config;
    stats_.reset();

    // Create RTI ambassador
    rti_ = std::make_unique<RTIAmbassador>();

    // Create federate ambassador
    ambassador_ = std::make_unique<FederateAmbassadorImpl>(*this);

    return true;
}

bool HLAAdapter::connect() {
    if (!rti_) return false;

    if (!rti_->connect(config_.rti_host, config_.rti_port)) {
        return false;
    }

    state_ = FederateState::Connected;
    return true;
}

bool HLAAdapter::join_federation() {
    if (!rti_ || state_ != FederateState::Connected) return false;

    // Try to create federation (may already exist)
    rti_->create_federation(config_.federation_name, config_.fom_path);

    // Join federation
    if (!rti_->join_federation(config_.federate_name,
                                config_.federation_name,
                                ambassador_.get())) {
        return false;
    }

    state_ = FederateState::Joined;

    // Setup publishing and subscribing
    publish_subscribe_setup();

    // Enable time management
    if (config_.time_mode == TimeManagementMode::TimeRegulating ||
        config_.time_mode == TimeManagementMode::Both) {
        rti_->enable_time_regulation(config_.lookahead);
    }

    if (config_.time_mode == TimeManagementMode::TimeConstrained ||
        config_.time_mode == TimeManagementMode::Both) {
        rti_->enable_time_constrained();
    }

    state_ = FederateState::Running;
    return true;
}

void HLAAdapter::resign() {
    if (state_ != FederateState::Running && state_ != FederateState::Joined) return;

    // Delete all local objects
    {
        std::lock_guard<std::mutex> lock(objects_mutex_);
        for (const auto& [handle, obj] : objects_) {
            if (obj.is_local) {
                rti_->delete_object(handle);
            }
        }
        objects_.clear();
        local_entity_map_.clear();
    }

    if (rti_) {
        rti_->resign_federation();
    }

    state_ = FederateState::Resigned;
}

void HLAAdapter::shutdown() {
    resign();

    if (rti_) {
        rti_->disconnect();
    }

    state_ = FederateState::Destroyed;
    rti_.reset();
    ambassador_.reset();
}

bool HLAAdapter::request_time_advance(double target_time) {
    if (!rti_ || !is_active()) return false;

    if (time_advance_pending_) return false;

    time_advance_pending_ = true;
    stats_.time_advances_requested++;

    return rti_->time_advance_request(target_time);
}

double HLAAdapter::wait_for_time_advance(uint32_t timeout_ms) {
    if (!time_advance_pending_) return federate_time_;

    auto start = std::chrono::steady_clock::now();
    auto timeout = std::chrono::milliseconds(timeout_ms);

    while (time_advance_pending_) {
        tick(10);

        auto elapsed = std::chrono::steady_clock::now() - start;
        if (elapsed > timeout) {
            return -1.0;  // Timeout
        }
    }

    return granted_time_;
}

ObjectInstanceHandle HLAAdapter::register_object(
        EntityId entity_id,
        const std::string& name,
        const EntityType& entity_type,
        ObjectClass object_class) {
    if (!rti_ || !is_active()) return 0;

    ObjectInstanceHandle handle = rti_->register_object(object_class, name);
    if (handle == 0) return 0;

    // Create local tracking
    {
        std::lock_guard<std::mutex> lock(objects_mutex_);
        PlatformObject obj;
        obj.handle = handle;
        obj.instance_name = name;
        obj.object_class = object_class;
        obj.entity_type = entity_type;
        obj.local_entity_id = entity_id;
        obj.is_local = true;
        obj.last_update = std::chrono::steady_clock::now();
        objects_[handle] = obj;
        local_entity_map_[entity_id] = handle;
    }

    stats_.objects_registered++;
    return handle;
}

bool HLAAdapter::update_object_attributes(
        ObjectInstanceHandle handle,
        const physics::EntityState& state) {
    if (!rti_ || !is_active()) return false;

    std::lock_guard<std::mutex> lock(objects_mutex_);

    auto it = objects_.find(handle);
    if (it == objects_.end() || !it->second.is_local) return false;

    // Update local copy
    it->second.spatial = convert::entity_state_to_spatial(state);
    it->second.last_update = std::chrono::steady_clock::now();

    // Encode spatial data
    std::vector<uint8_t> spatial_data = encode_spatial(state);

    // Send update
    std::vector<std::pair<AttributeHandle, std::vector<uint8_t>>> attributes;
    attributes.emplace_back(1, spatial_data);  // Spatial attribute handle = 1

    if (!rti_->update_attribute_values(handle, attributes, federate_time_)) {
        return false;
    }

    stats_.attribute_updates_sent++;
    return true;
}

bool HLAAdapter::delete_object(ObjectInstanceHandle handle) {
    if (!rti_ || !is_active()) return false;

    {
        std::lock_guard<std::mutex> lock(objects_mutex_);
        auto it = objects_.find(handle);
        if (it == objects_.end() || !it->second.is_local) return false;

        // Remove from local entity map
        local_entity_map_.erase(it->second.local_entity_id);
        objects_.erase(it);
    }

    return rti_->delete_object(handle);
}

const PlatformObject* HLAAdapter::get_object(ObjectInstanceHandle handle) const {
    std::lock_guard<std::mutex> lock(objects_mutex_);
    auto it = objects_.find(handle);
    return (it != objects_.end()) ? &it->second : nullptr;
}

std::vector<ObjectInstanceHandle> HLAAdapter::get_remote_objects() const {
    std::lock_guard<std::mutex> lock(objects_mutex_);
    std::vector<ObjectInstanceHandle> handles;
    for (const auto& [handle, obj] : objects_) {
        if (!obj.is_local) {
            handles.push_back(handle);
        }
    }
    return handles;
}

void HLAAdapter::map_to_local_entity(ObjectInstanceHandle handle, EntityId entity_id) {
    std::lock_guard<std::mutex> lock(objects_mutex_);
    auto it = objects_.find(handle);
    if (it != objects_.end()) {
        it->second.local_entity_id = entity_id;
    }
}

bool HLAAdapter::send_weapon_fire(const WeaponFireInteraction& fire) {
    if (!rti_ || !is_active()) return false;

    // Encode interaction parameters
    std::vector<std::pair<ParameterHandle, std::vector<uint8_t>>> parameters;

    // Munition type
    std::vector<uint8_t> type_data(8);
    std::memcpy(type_data.data(), &fire.munition_type, sizeof(EntityType));
    parameters.emplace_back(1, type_data);

    // Fire location
    std::vector<uint8_t> loc_data(24);
    std::memcpy(loc_data.data(), &fire.fire_location, sizeof(WorldLocation));
    parameters.emplace_back(2, loc_data);

    // Initial velocity
    std::vector<uint8_t> vel_data(12);
    std::memcpy(vel_data.data(), &fire.initial_velocity, sizeof(VelocityVector));
    parameters.emplace_back(3, vel_data);

    // Object handles
    std::vector<uint8_t> firing_data(8);
    std::memcpy(firing_data.data(), &fire.firing_object, sizeof(ObjectInstanceHandle));
    parameters.emplace_back(4, firing_data);

    if (!rti_->send_interaction(InteractionClass::WeaponFire, parameters, federate_time_)) {
        return false;
    }

    stats_.interactions_sent++;
    return true;
}

bool HLAAdapter::send_detonation(const MunitionDetonationInteraction& detonation) {
    if (!rti_ || !is_active()) return false;

    // Encode interaction parameters
    std::vector<std::pair<ParameterHandle, std::vector<uint8_t>>> parameters;

    // Munition type
    std::vector<uint8_t> type_data(8);
    std::memcpy(type_data.data(), &detonation.munition_type, sizeof(EntityType));
    parameters.emplace_back(1, type_data);

    // Detonation location
    std::vector<uint8_t> loc_data(24);
    std::memcpy(loc_data.data(), &detonation.detonation_location, sizeof(WorldLocation));
    parameters.emplace_back(2, loc_data);

    // Final velocity
    std::vector<uint8_t> vel_data(12);
    std::memcpy(vel_data.data(), &detonation.final_velocity, sizeof(VelocityVector));
    parameters.emplace_back(3, vel_data);

    // Detonation result
    std::vector<uint8_t> result_data(1);
    result_data[0] = detonation.detonation_result;
    parameters.emplace_back(4, result_data);

    if (!rti_->send_interaction(InteractionClass::MunitionDetonation, parameters, federate_time_)) {
        return false;
    }

    stats_.interactions_sent++;
    return true;
}

bool HLAAdapter::register_sync_point(const std::string& label) {
    if (!rti_ || !is_active()) return false;
    return rti_->register_sync_point(label);
}

bool HLAAdapter::achieve_sync_point(const std::string& label) {
    if (!rti_ || !is_active()) return false;

    {
        std::lock_guard<std::mutex> lock(sync_mutex_);
        sync_points_[label] = SyncPointState::Achieved;
    }

    return rti_->achieve_sync_point(label);
}

bool HLAAdapter::wait_for_sync(const std::string& label, uint32_t timeout_ms) {
    auto start = std::chrono::steady_clock::now();
    auto timeout = std::chrono::milliseconds(timeout_ms);

    while (true) {
        {
            std::lock_guard<std::mutex> lock(sync_mutex_);
            auto it = sync_points_.find(label);
            if (it != sync_points_.end() &&
                it->second == SyncPointState::Synchronized) {
                return true;
            }
        }

        tick(10);

        auto elapsed = std::chrono::steady_clock::now() - start;
        if (elapsed > timeout) {
            return false;
        }
    }
}

void HLAAdapter::tick(uint32_t timeout_ms) {
    if (!rti_) return;
    rti_->evoke_callbacks(timeout_ms);
}

bool HLAAdapter::needs_update(ObjectInstanceHandle handle,
                               const physics::EntityState& current_state) const {
    std::lock_guard<std::mutex> lock(objects_mutex_);

    auto it = objects_.find(handle);
    if (it == objects_.end()) return false;

    return check_dead_reckoning_threshold(it->second, current_state);
}

// ============================================================================
// Private Helper Methods
// ============================================================================

void HLAAdapter::publish_subscribe_setup() {
    if (!rti_) return;

    // Define attribute handles (would be looked up from RTI in real implementation)
    std::vector<AttributeHandle> platform_attributes = {
        1,  // Spatial
        2,  // EntityType
        3,  // EntityIdentifier
        4,  // ForceIdentifier
        5,  // Marking
        6,  // DamageState
    };

    // Publish and subscribe to Platform class and subclasses
    std::vector<ObjectClass> classes = {
        ObjectClass::Platform,
        ObjectClass::Aircraft,
        ObjectClass::GroundVehicle,
        ObjectClass::SurfaceVessel,
        ObjectClass::Spacecraft,
        ObjectClass::Munition
    };

    for (auto obj_class : classes) {
        if (config_.auto_publish_local) {
            rti_->publish_object_class(obj_class, platform_attributes);
        }
        if (config_.auto_subscribe_remote) {
            rti_->subscribe_object_class(obj_class, platform_attributes);
        }
    }

    // Publish and subscribe to interactions
    std::vector<InteractionClass> interactions = {
        InteractionClass::WeaponFire,
        InteractionClass::MunitionDetonation,
        InteractionClass::StartResume,
        InteractionClass::StopFreeze
    };

    for (auto interaction : interactions) {
        rti_->publish_interaction(interaction);
        rti_->subscribe_interaction(interaction);
    }
}

std::vector<uint8_t> HLAAdapter::encode_spatial(const physics::EntityState& state) {
    SpatialRepresentation spatial = convert::entity_state_to_spatial(state);

    // Calculate buffer size
    // WorldLocation (24 bytes) + isFrozen (1) + Orientation (12) +
    // Velocity (12) + DeadReckoning (1 + 12 + 12 = 25)
    constexpr size_t spatial_size = 24 + 1 + 12 + 12 + 25;
    std::vector<uint8_t> buffer(spatial_size);

    size_t offset = 0;

    // World Location (double x 3)
    std::memcpy(buffer.data() + offset, &spatial.world_location, 24);
    offset += 24;

    // Is Frozen
    buffer[offset++] = spatial.is_frozen ? 1 : 0;

    // Orientation (float x 3)
    std::memcpy(buffer.data() + offset, &spatial.orientation, 12);
    offset += 12;

    // Velocity (float x 3)
    std::memcpy(buffer.data() + offset, &spatial.velocity, 12);
    offset += 12;

    // Dead Reckoning
    buffer[offset++] = static_cast<uint8_t>(spatial.dead_reckoning.algorithm);
    std::memcpy(buffer.data() + offset, spatial.dead_reckoning.linear_acceleration, 12);
    offset += 12;
    std::memcpy(buffer.data() + offset, spatial.dead_reckoning.angular_velocity, 12);

    return buffer;
}

void HLAAdapter::decode_spatial(const std::vector<uint8_t>& data,
                                 SpatialRepresentation& spatial) {
    if (data.size() < 74) return;  // Minimum spatial size

    size_t offset = 0;

    // World Location
    std::memcpy(&spatial.world_location, data.data() + offset, 24);
    offset += 24;

    // Is Frozen
    spatial.is_frozen = (data[offset++] != 0);

    // Orientation
    std::memcpy(&spatial.orientation, data.data() + offset, 12);
    offset += 12;

    // Velocity
    std::memcpy(&spatial.velocity, data.data() + offset, 12);
    offset += 12;

    // Dead Reckoning
    spatial.dead_reckoning.algorithm = static_cast<DeadReckoningAlgorithm>(data[offset++]);
    std::memcpy(spatial.dead_reckoning.linear_acceleration, data.data() + offset, 12);
    offset += 12;
    std::memcpy(spatial.dead_reckoning.angular_velocity, data.data() + offset, 12);
}

bool HLAAdapter::check_dead_reckoning_threshold(
        const PlatformObject& obj,
        const physics::EntityState& current) const {
    // Check time since last update (heartbeat)
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration<double>(now - obj.last_update).count();

    if (elapsed >= config_.heartbeat_interval) {
        return true;
    }

    // Check position threshold
    math::Vec3 last_pos = obj.spatial.world_location.to_vec3();
    math::Vec3 delta_pos = current.position - last_pos;
    double pos_error = delta_pos.length();

    if (pos_error >= config_.position_threshold) {
        return true;
    }

    // Check orientation threshold
    math::Quat last_ori = obj.spatial.orientation.to_quaternion();
    math::Quat delta_ori = current.orientation * last_ori.inverse();

    // Get rotation angle from quaternion
    double angle = 2.0 * std::acos(std::min(1.0, std::abs(delta_ori.w)));

    if (angle >= config_.orientation_threshold) {
        return true;
    }

    return false;
}

} // namespace jaguar::interface::hla
