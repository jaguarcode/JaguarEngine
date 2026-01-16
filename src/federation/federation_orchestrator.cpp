// Copyright JaguarEngine Team. All Rights Reserved.

#include "jaguar/federation/federation_orchestrator.h"

#include <algorithm>
#include <condition_variable>
#include <mutex>
#include <sstream>
#include <thread>

namespace jaguar::federation {

//==============================================================================
// EntityIdentifier Implementation
//==============================================================================

std::string EntityIdentifier::to_string() const {
    std::ostringstream oss;
    oss << site_id << ":" << application_id << ":" << entity_number;
    return oss.str();
}

EntityIdentifier EntityIdentifier::from_string(std::string_view str) {
    EntityIdentifier id;
    // Parse "site:app:entity" format
    size_t first_colon = str.find(':');
    size_t second_colon = str.find(':', first_colon + 1);

    if (first_colon != std::string::npos && second_colon != std::string::npos) {
        id.site_id = static_cast<uint16_t>(
            std::stoi(std::string(str.substr(0, first_colon))));
        id.application_id = static_cast<uint16_t>(
            std::stoi(std::string(str.substr(first_colon + 1, second_colon - first_colon - 1))));
        id.entity_number = static_cast<uint16_t>(
            std::stoi(std::string(str.substr(second_colon + 1))));
    }
    return id;
}

//==============================================================================
// EntityFilter Implementation
//==============================================================================

bool EntityFilter::matches(const EntityIdentifier& entity) const {
    switch (type) {
        case FilterType::All:
            return true;

        case FilterType::ByDomain:
            // Would check entity type domain
            return true;

        case FilterType::ByEntityType:
            // Would check entity type tuple
            return true;

        case FilterType::ByGeographic:
            // Would check geographic position
            return true;

        case FilterType::Custom:
            if (custom_filter) {
                return custom_filter(entity);
            }
            return true;

        default:
            return true;
    }
}

//==============================================================================
// TimeManager Implementation
//==============================================================================

struct TimeManager::Impl {
    std::mutex mutex;
    std::condition_variable cv;
    std::chrono::high_resolution_clock::time_point last_request_time;

    // Statistics
    uint64_t total_requests = 0;
    uint64_t total_grants = 0;
    double total_wait_ms = 0.0;
    double max_wait_ms = 0.0;
};

TimeManager::TimeManager()
    : impl_(std::make_unique<Impl>()) {}

TimeManager::~TimeManager() = default;

void TimeManager::set_mode(TimeManagementMode mode) {
    mode_ = mode;
}

void TimeManager::set_lookahead(double lookahead) {
    lookahead_ = lookahead;
}

void TimeManager::request_advance(double target_time) {
    requested_time_.store(target_time, std::memory_order_release);
    advance_pending_.store(true, std::memory_order_release);

    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->last_request_time = std::chrono::high_resolution_clock::now();
    impl_->total_requests++;
}

void TimeManager::grant_advance(double granted_time) {
    logical_time_.store(granted_time, std::memory_order_release);
    advance_pending_.store(false, std::memory_order_release);

    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->total_grants++;

    auto now = std::chrono::high_resolution_clock::now();
    double wait_ms = std::chrono::duration<double, std::milli>(
        now - impl_->last_request_time).count();

    impl_->total_wait_ms += wait_ms;
    impl_->max_wait_ms = std::max(impl_->max_wait_ms, wait_ms);

    impl_->cv.notify_all();
}

bool TimeManager::wait_for_grant(std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(impl_->mutex);
    return impl_->cv.wait_for(lock, timeout, [this]() {
        return !advance_pending_.load(std::memory_order_acquire);
    });
}

TimeManager::Stats TimeManager::get_stats() const {
    Stats stats;
    std::lock_guard<std::mutex> lock(impl_->mutex);
    stats.requests = impl_->total_requests;
    stats.grants = impl_->total_grants;
    stats.max_wait_ms = impl_->max_wait_ms;
    if (impl_->total_grants > 0) {
        stats.avg_wait_ms = impl_->total_wait_ms / impl_->total_grants;
    }
    return stats;
}

//==============================================================================
// EntityRouter Implementation
//==============================================================================

struct EntityRouter::Impl {
    std::mutex mutex;

    struct EntityInfo {
        std::string federation;
        OwnershipState ownership = OwnershipState::Unowned;
    };

    std::unordered_map<EntityIdentifier, EntityInfo, EntityIdentifierHash> entities;
};

EntityRouter::EntityRouter()
    : impl_(std::make_unique<Impl>()) {}

EntityRouter::~EntityRouter() = default;

void EntityRouter::register_entity(const EntityIdentifier& entity,
                                   const std::string& federation) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->entities[entity].federation = federation;
}

void EntityRouter::unregister_entity(const EntityIdentifier& entity) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->entities.erase(entity);
}

std::optional<std::string> EntityRouter::get_federation(
    const EntityIdentifier& entity) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    auto it = impl_->entities.find(entity);
    if (it != impl_->entities.end()) {
        return it->second.federation;
    }
    return std::nullopt;
}

std::vector<EntityIdentifier> EntityRouter::get_entities(
    const std::string& federation) const {
    std::vector<EntityIdentifier> result;
    std::lock_guard<std::mutex> lock(impl_->mutex);

    for (const auto& [entity, info] : impl_->entities) {
        if (info.federation == federation) {
            result.push_back(entity);
        }
    }
    return result;
}

bool EntityRouter::should_route(const EntityIdentifier& entity,
                                const BridgeConfig& bridge) const {
    auto federation = get_federation(entity);
    if (!federation || *federation != bridge.source_federation) {
        return false;
    }
    return bridge.entity_filter.matches(entity);
}

OwnershipState EntityRouter::get_ownership_state(
    const EntityIdentifier& entity) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    auto it = impl_->entities.find(entity);
    if (it != impl_->entities.end()) {
        return it->second.ownership;
    }
    return OwnershipState::Unowned;
}

void EntityRouter::set_ownership_state(const EntityIdentifier& entity,
                                       OwnershipState state) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    auto it = impl_->entities.find(entity);
    if (it != impl_->entities.end()) {
        it->second.ownership = state;
    }
}

//==============================================================================
// Federation Implementation
//==============================================================================

struct Federation::Impl {
    std::mutex mutex;
    std::vector<SyncPoint> sync_points;
    std::condition_variable sync_cv;

    // Statistics
    uint32_t local_entity_count = 0;
    uint32_t remote_entity_count = 0;
    uint64_t interactions_sent = 0;
    uint64_t interactions_received = 0;
    uint64_t bytes_sent = 0;
    uint64_t bytes_received = 0;
};

Federation::Federation(const FederationConfig& config)
    : config_(config)
    , time_manager_(std::make_unique<TimeManager>())
    , impl_(std::make_unique<Impl>()) {

    time_manager_->set_mode(config.time_mode);
    time_manager_->set_lookahead(config.lookahead);
}

Federation::~Federation() {
    if (state_ != FederationState::Disconnected) {
        resign();
        disconnect();
    }
}

bool Federation::connect() {
    if (state_ != FederationState::Disconnected) {
        return false;
    }

    state_ = FederationState::Connecting;

    // Would connect to RTI here
    // For now, simulate successful connection

    state_ = FederationState::Connected;

    FederationEvent event;
    event.type = FederationEventType::Connected;
    event.federation_name = config_.federation_name;
    event.timestamp = std::chrono::system_clock::now();
    emit_event(event);

    return true;
}

void Federation::disconnect() {
    if (state_ == FederationState::Disconnected) {
        return;
    }

    if (state_ == FederationState::Joined) {
        resign();
    }

    state_ = FederationState::Disconnected;

    FederationEvent event;
    event.type = FederationEventType::Disconnected;
    event.federation_name = config_.federation_name;
    event.timestamp = std::chrono::system_clock::now();
    emit_event(event);
}

bool Federation::create_and_join() {
    if (state_ != FederationState::Connected) {
        return false;
    }

    // Would create federation execution and join
    // For now, simulate successful join

    state_ = FederationState::Joined;

    FederationEvent event;
    event.type = FederationEventType::Joined;
    event.federation_name = config_.federation_name;
    event.timestamp = std::chrono::system_clock::now();
    emit_event(event);

    return true;
}

bool Federation::join() {
    if (state_ != FederationState::Connected) {
        return false;
    }

    // Would join existing federation
    state_ = FederationState::Joined;

    FederationEvent event;
    event.type = FederationEventType::Joined;
    event.federation_name = config_.federation_name;
    event.timestamp = std::chrono::system_clock::now();
    emit_event(event);

    return true;
}

void Federation::resign() {
    if (state_ != FederationState::Joined) {
        return;
    }

    // Would resign from federation
    state_ = FederationState::Resigned;

    FederationEvent event;
    event.type = FederationEventType::Resigned;
    event.federation_name = config_.federation_name;
    event.timestamp = std::chrono::system_clock::now();
    emit_event(event);
}

bool Federation::announce_sync_point(const std::string& label) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    SyncPoint sp;
    sp.label = label;
    sp.achieved = false;
    sp.announced_at = std::chrono::system_clock::now();
    impl_->sync_points.push_back(sp);

    FederationEvent event;
    event.type = FederationEventType::SyncPointAnnounced;
    event.federation_name = config_.federation_name;
    event.timestamp = std::chrono::system_clock::now();
    event.data = label;
    emit_event(event);

    return true;
}

bool Federation::achieve_sync_point(const std::string& label) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    for (auto& sp : impl_->sync_points) {
        if (sp.label == label && !sp.achieved) {
            sp.achieved = true;
            sp.achieved_at = std::chrono::system_clock::now();

            FederationEvent event;
            event.type = FederationEventType::SyncPointAchieved;
            event.federation_name = config_.federation_name;
            event.timestamp = std::chrono::system_clock::now();
            event.data = label;
            emit_event(event);

            impl_->sync_cv.notify_all();
            return true;
        }
    }
    return false;
}

bool Federation::wait_for_sync_point(const std::string& label,
                                      std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(impl_->mutex);
    return impl_->sync_cv.wait_for(lock, timeout, [this, &label]() {
        for (const auto& sp : impl_->sync_points) {
            if (sp.label == label && sp.achieved) {
                return true;
            }
        }
        return false;
    });
}

void Federation::on_event(FederationEventCallback callback) {
    callbacks_.push_back(std::move(callback));
}

FederationStats Federation::get_stats() const {
    FederationStats stats;
    stats.federation_name = config_.federation_name;
    stats.federate_name = config_.federate_name;
    stats.state = state_;

    std::lock_guard<std::mutex> lock(impl_->mutex);
    stats.local_entities = impl_->local_entity_count;
    stats.remote_entities = impl_->remote_entity_count;
    stats.interactions_sent = impl_->interactions_sent;
    stats.interactions_received = impl_->interactions_received;
    stats.bytes_sent = impl_->bytes_sent;
    stats.bytes_received = impl_->bytes_received;

    stats.logical_time = time_manager_->logical_time();
    stats.lookahead = time_manager_->lookahead();
    stats.time_regulating = (time_manager_->mode() == TimeManagementMode::Regulating ||
                             time_manager_->mode() == TimeManagementMode::Both);
    stats.time_constrained = (time_manager_->mode() == TimeManagementMode::Constrained ||
                              time_manager_->mode() == TimeManagementMode::Both);

    auto tm_stats = time_manager_->get_stats();
    stats.time_advance_requests = tm_stats.requests;
    stats.time_advance_grants = tm_stats.grants;
    stats.avg_advance_time_ms = tm_stats.avg_wait_ms;
    stats.max_advance_time_ms = tm_stats.max_wait_ms;

    return stats;
}

void Federation::tick(double dt) {
    if (state_ != FederationState::Joined) {
        return;
    }

    // Request time advance
    double current_time = time_manager_->logical_time();
    double target_time = current_time + dt;

    time_manager_->request_advance(target_time);

    // In a real implementation, would wait for RTI callback
    // For now, simulate immediate grant
    time_manager_->grant_advance(target_time);

    // Process any pending callbacks/updates
    // Would handle attribute reflections, interactions, etc.
}

void Federation::emit_event(const FederationEvent& event) {
    for (auto& callback : callbacks_) {
        callback(event);
    }
}

//==============================================================================
// FederationBridge Implementation
//==============================================================================

struct FederationBridge::Impl {
    std::mutex mutex;

    // Statistics
    uint64_t entities_routed = 0;
    uint64_t updates_forwarded = 0;
    uint64_t updates_filtered = 0;
    double total_latency_ms = 0.0;
};

FederationBridge::FederationBridge(const BridgeConfig& config,
                                   Federation& source,
                                   Federation& target)
    : config_(config)
    , source_(source)
    , target_(target)
    , impl_(std::make_unique<Impl>()) {}

FederationBridge::~FederationBridge() = default;

void FederationBridge::set_enabled(bool enabled) {
    enabled_.store(enabled, std::memory_order_release);
}

void FederationBridge::process_updates() {
    if (!enabled_.load(std::memory_order_acquire)) {
        return;
    }

    // Would process entity updates from source and forward to target
    // Applying filters and transformations as configured

    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->updates_forwarded++;
}

FederationBridge::Stats FederationBridge::get_stats() const {
    Stats stats;
    std::lock_guard<std::mutex> lock(impl_->mutex);
    stats.entities_routed = impl_->entities_routed;
    stats.updates_forwarded = impl_->updates_forwarded;
    stats.updates_filtered = impl_->updates_filtered;
    if (impl_->updates_forwarded > 0) {
        stats.avg_latency_ms = impl_->total_latency_ms / impl_->updates_forwarded;
    }
    return stats;
}

//==============================================================================
// FederationOrchestrator Implementation
//==============================================================================

struct FederationOrchestrator::Impl {
    std::mutex mutex;
    std::thread tick_thread;
    std::condition_variable cv;
    std::chrono::milliseconds tick_interval{16};  // ~60 Hz
};

FederationOrchestrator::FederationOrchestrator()
    : entity_router_(std::make_unique<EntityRouter>())
    , impl_(std::make_unique<Impl>()) {}

FederationOrchestrator::~FederationOrchestrator() {
    shutdown();
}

FederationOrchestrator& FederationOrchestrator::instance() {
    static FederationOrchestrator instance;
    return instance;
}

Federation& FederationOrchestrator::create_federation(const FederationConfig& config) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    auto federation = std::make_unique<Federation>(config);
    auto& ref = *federation;

    // Register global callbacks
    for (const auto& callback : global_callbacks_) {
        federation->on_event(callback);
    }

    federations_[config.federation_name] = std::move(federation);
    return ref;
}

Federation* FederationOrchestrator::get_federation(const std::string& name) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    auto it = federations_.find(name);
    if (it != federations_.end()) {
        return it->second.get();
    }
    return nullptr;
}

void FederationOrchestrator::remove_federation(const std::string& name) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    auto it = federations_.find(name);
    if (it != federations_.end()) {
        it->second->resign();
        it->second->disconnect();
        federations_.erase(it);
    }

    // Remove any bridges involving this federation
    bridges_.erase(
        std::remove_if(bridges_.begin(), bridges_.end(),
            [&name](const auto& bridge) {
                return bridge->config().source_federation == name ||
                       bridge->config().target_federation == name;
            }),
        bridges_.end());
}

FederationBridge& FederationOrchestrator::create_bridge(const BridgeConfig& config) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    auto* source = get_federation(config.source_federation);
    auto* target = get_federation(config.target_federation);

    if (!source || !target) {
        throw std::runtime_error("Source or target federation not found");
    }

    auto bridge = std::make_unique<FederationBridge>(config, *source, *target);
    auto& ref = *bridge;
    bridges_.push_back(std::move(bridge));

    // Create reverse bridge if bidirectional
    if (config.bidirectional) {
        BridgeConfig reverse_config = config;
        reverse_config.source_federation = config.target_federation;
        reverse_config.target_federation = config.source_federation;

        auto reverse_bridge = std::make_unique<FederationBridge>(
            reverse_config, *target, *source);
        bridges_.push_back(std::move(reverse_bridge));
    }

    return ref;
}

void FederationOrchestrator::remove_bridge(const std::string& source,
                                           const std::string& target) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    bridges_.erase(
        std::remove_if(bridges_.begin(), bridges_.end(),
            [&source, &target](const auto& bridge) {
                return bridge->config().source_federation == source &&
                       bridge->config().target_federation == target;
            }),
        bridges_.end());
}

void FederationOrchestrator::on_event(FederationEventCallback callback) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    global_callbacks_.push_back(callback);

    // Register with existing federations
    for (auto& [name, federation] : federations_) {
        federation->on_event(callback);
    }
}

void FederationOrchestrator::start() {
    if (running_.exchange(true)) {
        return;  // Already running
    }

    impl_->tick_thread = std::thread([this]() {
        auto last_tick = std::chrono::high_resolution_clock::now();

        while (running_.load(std::memory_order_acquire)) {
            auto now = std::chrono::high_resolution_clock::now();
            double dt = std::chrono::duration<double>(now - last_tick).count();
            last_tick = now;

            tick(dt);

            std::unique_lock<std::mutex> lock(impl_->mutex);
            impl_->cv.wait_for(lock, impl_->tick_interval, [this]() {
                return !running_.load(std::memory_order_acquire);
            });
        }
    });
}

void FederationOrchestrator::stop() {
    if (!running_.exchange(false)) {
        return;  // Not running
    }

    impl_->cv.notify_all();
    if (impl_->tick_thread.joinable()) {
        impl_->tick_thread.join();
    }
}

void FederationOrchestrator::tick(double dt) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    // Tick all federations
    for (auto& [name, federation] : federations_) {
        federation->tick(dt);
    }

    // Process all bridges
    for (auto& bridge : bridges_) {
        bridge->process_updates();
    }
}

std::vector<FederationStats> FederationOrchestrator::get_all_stats() const {
    std::vector<FederationStats> stats;
    std::lock_guard<std::mutex> lock(impl_->mutex);

    for (const auto& [name, federation] : federations_) {
        stats.push_back(federation->get_stats());
    }

    return stats;
}

bool FederationOrchestrator::global_sync(const std::string& label,
                                          std::chrono::milliseconds timeout) {
    // Announce sync point in all federations
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        for (auto& [name, federation] : federations_) {
            federation->announce_sync_point(label);
        }
    }

    // Achieve sync point in all federations
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        for (auto& [name, federation] : federations_) {
            federation->achieve_sync_point(label);
        }
    }

    // Wait for all to sync
    auto start = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        for (auto& [name, federation] : federations_) {
            auto remaining = timeout - std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start);
            if (remaining.count() <= 0) {
                return false;
            }
            if (!federation->wait_for_sync_point(label, remaining)) {
                return false;
            }
        }
    }

    return true;
}

void FederationOrchestrator::shutdown() {
    stop();

    std::lock_guard<std::mutex> lock(impl_->mutex);

    // Disconnect all bridges first
    bridges_.clear();

    // Then disconnect all federations
    for (auto& [name, federation] : federations_) {
        federation->resign();
        federation->disconnect();
    }
    federations_.clear();
}

} // namespace jaguar::federation
