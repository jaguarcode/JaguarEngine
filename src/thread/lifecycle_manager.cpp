/**
 * @file lifecycle_manager.cpp
 * @brief Implementation of entity lifecycle management for Digital Thread
 */

#include "jaguar/thread/lifecycle_manager.h"
#include <algorithm>
#include <mutex>
#include <atomic>

namespace jaguar::thread {

// ============================================================================
// Simple Lifecycle Storage Implementation
// ============================================================================

/**
 * @brief In-memory storage implementation for lifecycle data
 */
class SimpleLifecycleStorage : public ILifecycleStorage {
public:
    SimpleLifecycleStorage() = default;
    ~SimpleLifecycleStorage() override = default;

    LifecycleResult store(const EntityLifecycleInfo& info) override {
        std::lock_guard<std::mutex> lock(mutex_);
        storage_[info.entity_id] = info;
        return LifecycleResult::Success;
    }

    std::optional<EntityLifecycleInfo> load(EntityId entity_id) override {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = storage_.find(entity_id);
        if (it == storage_.end()) {
            return std::nullopt;
        }
        return it->second;
    }

    LifecycleResult remove(EntityId entity_id) override {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = storage_.find(entity_id);
        if (it == storage_.end()) {
            return LifecycleResult::EntityNotTracked;
        }
        storage_.erase(it);
        return LifecycleResult::Success;
    }

    std::vector<EntityId> list_entities() override {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<EntityId> result;
        result.reserve(storage_.size());
        for (const auto& [entity_id, info] : storage_) {
            result.push_back(entity_id);
        }
        return result;
    }

private:
    std::unordered_map<EntityId, EntityLifecycleInfo> storage_;
    mutable std::mutex mutex_;
};

// ============================================================================
// Lifecycle Manager Implementation
// ============================================================================

struct LifecycleManager::Impl {
    // Configuration
    LifecycleConfig config;

    // State
    std::unordered_map<EntityId, EntityLifecycleInfo> entities;
    std::vector<std::shared_ptr<ILifecycleObserver>> observers;
    std::shared_ptr<ILifecycleStorage> storage;
    std::atomic<bool> initialized{false};

    // Statistics
    LifecycleStats stats;

    // Thread safety
    mutable std::mutex mutex;
};

LifecycleManager::LifecycleManager(const LifecycleConfig& config)
    : impl_(std::make_unique<Impl>()) {
    impl_->config = config;
}

LifecycleManager::~LifecycleManager() = default;

LifecycleManager::LifecycleManager(LifecycleManager&&) noexcept = default;
LifecycleManager& LifecycleManager::operator=(LifecycleManager&&) noexcept = default;

// ============================================================================
// Lifecycle
// ============================================================================

LifecycleResult LifecycleManager::initialize() {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (impl_->initialized.load()) {
        return LifecycleResult::AlreadyInitialized;
    }

    impl_->initialized.store(true);
    return LifecycleResult::Success;
}

LifecycleResult LifecycleManager::shutdown() {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->initialized.load()) {
        return LifecycleResult::NotInitialized;
    }

    // Save all data if storage is configured
    if (impl_->storage) {
        for (const auto& [entity_id, info] : impl_->entities) {
            impl_->storage->store(info);
        }
    }

    // Clear state
    impl_->entities.clear();
    impl_->observers.clear();
    impl_->stats.reset();

    impl_->initialized.store(false);
    return LifecycleResult::Success;
}

bool LifecycleManager::is_initialized() const noexcept {
    return impl_->initialized.load();
}

const LifecycleConfig& LifecycleManager::get_config() const noexcept {
    return impl_->config;
}

// ============================================================================
// Entity Tracking
// ============================================================================

LifecycleResult LifecycleManager::track_entity(EntityId entity_id,
                                                LifecyclePhase initial_phase) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->initialized.load()) {
        return LifecycleResult::NotInitialized;
    }

    if (entity_id == INVALID_ENTITY_ID) {
        return LifecycleResult::InvalidEntityId;
    }

    // Check if already tracked
    if (impl_->entities.count(entity_id)) {
        return LifecycleResult::Success; // Already tracked
    }

    // Create new lifecycle info
    EntityLifecycleInfo info(entity_id, initial_phase);
    impl_->entities[entity_id] = info;

    // Update statistics
    impl_->stats.total_entities++;
    impl_->stats.entities_per_phase[initial_phase]++;

    // Store if storage configured
    if (impl_->storage) {
        impl_->storage->store(info);
    }

    // Notify observers
    for (const auto& observer : impl_->observers) {
        if (observer) {
            observer->on_entity_created(entity_id, initial_phase);
        }
    }

    return LifecycleResult::Success;
}

LifecycleResult LifecycleManager::untrack_entity(EntityId entity_id) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->initialized.load()) {
        return LifecycleResult::NotInitialized;
    }

    auto it = impl_->entities.find(entity_id);
    if (it == impl_->entities.end()) {
        return LifecycleResult::EntityNotTracked;
    }

    // Update statistics
    LifecyclePhase phase = it->second.current_phase;
    impl_->stats.total_entities--;
    if (impl_->stats.entities_per_phase[phase] > 0) {
        impl_->stats.entities_per_phase[phase]--;
    }

    // Remove from storage
    if (impl_->storage) {
        impl_->storage->remove(entity_id);
    }

    // Notify observers
    for (const auto& observer : impl_->observers) {
        if (observer) {
            observer->on_entity_removed(entity_id);
        }
    }

    // Remove from entities
    impl_->entities.erase(it);

    return LifecycleResult::Success;
}

bool LifecycleManager::is_tracked(EntityId entity_id) const noexcept {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->entities.count(entity_id) > 0;
}

UInt64 LifecycleManager::get_entity_count() const noexcept {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->stats.total_entities;
}

// ============================================================================
// Phase Transitions
// ============================================================================

LifecycleResult LifecycleManager::transition(EntityId entity_id,
                                              LifecyclePhase to_phase,
                                              std::string reason,
                                              std::string operator_id) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->initialized.load()) {
        return LifecycleResult::NotInitialized;
    }

    auto it = impl_->entities.find(entity_id);
    if (it == impl_->entities.end()) {
        return LifecycleResult::EntityNotTracked;
    }

    EntityLifecycleInfo& info = it->second;
    LifecyclePhase from_phase = info.current_phase;

    // Check if transition is valid
    if (impl_->config.strict_transitions && !is_valid_transition(from_phase, to_phase)) {
        return LifecycleResult::TransitionNotAllowed;
    }

    // Same phase - no-op
    if (from_phase == to_phase) {
        return LifecycleResult::Success;
    }

    // Create transition record
    LifecycleTransition transition(entity_id, from_phase, to_phase,
                                     std::move(reason), std::move(operator_id));

    // Update entity info
    info.current_phase = to_phase;
    info.last_transition = transition.timestamp;
    info.transition_count++;
    info.transition_history.push_back(transition);

    // Trim history if needed
    if (info.transition_history.size() > impl_->config.max_history_size) {
        auto excess = static_cast<std::ptrdiff_t>(
            info.transition_history.size() - impl_->config.max_history_size);
        info.transition_history.erase(
            info.transition_history.begin(),
            info.transition_history.begin() + excess
        );
    }

    // Update statistics
    impl_->stats.total_transitions++;
    if (impl_->stats.entities_per_phase[from_phase] > 0) {
        impl_->stats.entities_per_phase[from_phase]--;
    }
    impl_->stats.entities_per_phase[to_phase]++;
    impl_->stats.average_transitions_per_entity =
        static_cast<Real>(impl_->stats.total_transitions) /
        static_cast<Real>(impl_->stats.total_entities > 0 ? impl_->stats.total_entities : 1);

    // Store if storage configured
    if (impl_->storage) {
        impl_->storage->store(info);
    }

    // Invoke transition callback if configured
    if (impl_->config.transition_callback) {
        impl_->config.transition_callback(transition);
    }

    // Notify observers
    for (const auto& observer : impl_->observers) {
        if (observer) {
            observer->on_phase_changed(entity_id, from_phase, to_phase);
        }
    }

    return LifecycleResult::Success;
}

std::optional<LifecyclePhase> LifecycleManager::get_phase(EntityId entity_id) const noexcept {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    auto it = impl_->entities.find(entity_id);
    if (it == impl_->entities.end()) {
        return std::nullopt;
    }

    return it->second.current_phase;
}

bool LifecycleManager::is_valid_transition(LifecyclePhase from_phase,
                                             LifecyclePhase to_phase) const noexcept {
    // Same phase is always valid
    if (from_phase == to_phase) {
        return true;
    }

    // Check valid transitions based on phase
    switch (from_phase) {
        case LifecyclePhase::Design:
            // From Design: can go to Production or Decommissioned (cancelled)
            return to_phase == LifecyclePhase::Production ||
                   to_phase == LifecyclePhase::Decommissioned;

        case LifecyclePhase::Production:
            // From Production: can go to Operational, Maintenance, or Decommissioned
            return to_phase == LifecyclePhase::Operational ||
                   to_phase == LifecyclePhase::Maintenance ||
                   to_phase == LifecyclePhase::Decommissioned;

        case LifecyclePhase::Operational:
            // From Operational: can go to Maintenance or Decommissioned
            return to_phase == LifecyclePhase::Maintenance ||
                   to_phase == LifecyclePhase::Decommissioned;

        case LifecyclePhase::Maintenance:
            // From Maintenance: can return to Operational or be Decommissioned
            return to_phase == LifecyclePhase::Operational ||
                   to_phase == LifecyclePhase::Decommissioned;

        case LifecyclePhase::Decommissioned:
            // From Decommissioned: terminal state, no transitions allowed
            return false;

        default:
            return false;
    }
}

// ============================================================================
// Event Logging
// ============================================================================

LifecycleResult LifecycleManager::log_event(EntityId entity_id,
                                             LifecycleEvent event) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->initialized.load()) {
        return LifecycleResult::NotInitialized;
    }

    if (!impl_->config.enable_event_logging) {
        return LifecycleResult::Success; // Silently ignore if disabled
    }

    auto it = impl_->entities.find(entity_id);
    if (it == impl_->entities.end()) {
        return LifecycleResult::EntityNotTracked;
    }

    EntityLifecycleInfo& info = it->second;

    // Set entity_id in event if not set
    if (event.entity_id == INVALID_ENTITY_ID) {
        event.entity_id = entity_id;
    }

    // Add event to history
    info.events.push_back(event);

    // Trim events if needed
    if (info.events.size() > impl_->config.max_history_size) {
        auto excess = static_cast<std::ptrdiff_t>(
            info.events.size() - impl_->config.max_history_size);
        info.events.erase(
            info.events.begin(),
            info.events.begin() + excess
        );
    }

    // Update statistics
    impl_->stats.total_events++;
    impl_->stats.average_events_per_entity =
        static_cast<Real>(impl_->stats.total_events) /
        static_cast<Real>(impl_->stats.total_entities > 0 ? impl_->stats.total_entities : 1);

    // Store if storage configured
    if (impl_->storage) {
        impl_->storage->store(info);
    }

    // Notify observers
    for (const auto& observer : impl_->observers) {
        if (observer) {
            observer->on_event_logged(entity_id, event);
        }
    }

    return LifecycleResult::Success;
}

LifecycleResult LifecycleManager::log_event(EntityId entity_id,
                                             const std::string& event_type,
                                             const std::string& description) {
    LifecycleEvent event(entity_id, event_type, description);
    return log_event(entity_id, std::move(event));
}

// ============================================================================
// Queries
// ============================================================================

std::optional<EntityLifecycleInfo> LifecycleManager::get_lifecycle_info(
    EntityId entity_id) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    auto it = impl_->entities.find(entity_id);
    if (it == impl_->entities.end()) {
        return std::nullopt;
    }

    return it->second;
}

std::vector<EntityId> LifecycleManager::get_entities_in_phase(LifecyclePhase phase) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    std::vector<EntityId> result;
    for (const auto& [entity_id, info] : impl_->entities) {
        if (info.current_phase == phase) {
            result.push_back(entity_id);
        }
    }

    return result;
}

std::vector<LifecycleTransition> LifecycleManager::get_transition_history(
    EntityId entity_id) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    auto it = impl_->entities.find(entity_id);
    if (it == impl_->entities.end()) {
        return {};
    }

    return it->second.transition_history;
}

std::vector<LifecycleEvent> LifecycleManager::get_event_history(EntityId entity_id) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    auto it = impl_->entities.find(entity_id);
    if (it == impl_->entities.end()) {
        return {};
    }

    return it->second.events;
}

std::optional<std::chrono::seconds> LifecycleManager::get_time_in_phase(
    EntityId entity_id) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    auto it = impl_->entities.find(entity_id);
    if (it == impl_->entities.end()) {
        return std::nullopt;
    }

    auto now = std::chrono::system_clock::now();
    return std::chrono::duration_cast<std::chrono::seconds>(
        now - it->second.last_transition);
}

std::optional<std::chrono::seconds> LifecycleManager::get_lifetime(
    EntityId entity_id) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    auto it = impl_->entities.find(entity_id);
    if (it == impl_->entities.end()) {
        return std::nullopt;
    }

    auto now = std::chrono::system_clock::now();
    return std::chrono::duration_cast<std::chrono::seconds>(
        now - it->second.created_at);
}

// ============================================================================
// Observers
// ============================================================================

LifecycleResult LifecycleManager::add_observer(
    std::shared_ptr<ILifecycleObserver> observer) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!observer) {
        return LifecycleResult::InvalidConfiguration;
    }

    impl_->observers.push_back(observer);
    return LifecycleResult::Success;
}

LifecycleResult LifecycleManager::remove_observer(
    std::shared_ptr<ILifecycleObserver> observer) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    auto it = std::find(impl_->observers.begin(), impl_->observers.end(), observer);
    if (it == impl_->observers.end()) {
        return LifecycleResult::ObserverFailed;
    }

    impl_->observers.erase(it);
    return LifecycleResult::Success;
}

void LifecycleManager::clear_observers() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->observers.clear();
}

UInt32 LifecycleManager::get_observer_count() const noexcept {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return static_cast<UInt32>(impl_->observers.size());
}

// ============================================================================
// Storage
// ============================================================================

LifecycleResult LifecycleManager::set_storage(
    std::shared_ptr<ILifecycleStorage> storage) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    impl_->storage = storage;
    return LifecycleResult::Success;
}

std::shared_ptr<ILifecycleStorage> LifecycleManager::get_storage() const noexcept {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->storage;
}

LifecycleResult LifecycleManager::save_all() {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->storage) {
        return LifecycleResult::InvalidConfiguration;
    }

    for (const auto& [entity_id, info] : impl_->entities) {
        auto result = impl_->storage->store(info);
        if (result != LifecycleResult::Success) {
            return result;
        }
    }

    return LifecycleResult::Success;
}

LifecycleResult LifecycleManager::load_all() {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->storage) {
        return LifecycleResult::InvalidConfiguration;
    }

    auto entity_ids = impl_->storage->list_entities();
    for (EntityId entity_id : entity_ids) {
        auto info_opt = impl_->storage->load(entity_id);
        if (info_opt.has_value()) {
            impl_->entities[entity_id] = info_opt.value();
        }
    }

    // Recalculate statistics
    impl_->stats.reset();
    impl_->stats.total_entities = static_cast<UInt64>(impl_->entities.size());

    for (const auto& [entity_id, info] : impl_->entities) {
        impl_->stats.entities_per_phase[info.current_phase]++;
        impl_->stats.total_transitions += info.transition_count;
        impl_->stats.total_events += static_cast<UInt64>(info.events.size());
    }

    if (impl_->stats.total_entities > 0) {
        impl_->stats.average_transitions_per_entity =
            static_cast<Real>(impl_->stats.total_transitions) /
            static_cast<Real>(impl_->stats.total_entities);
        impl_->stats.average_events_per_entity =
            static_cast<Real>(impl_->stats.total_events) /
            static_cast<Real>(impl_->stats.total_entities);
    }

    return LifecycleResult::Success;
}

LifecycleResult LifecycleManager::save_entity(EntityId entity_id) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->storage) {
        return LifecycleResult::InvalidConfiguration;
    }

    auto it = impl_->entities.find(entity_id);
    if (it == impl_->entities.end()) {
        return LifecycleResult::EntityNotTracked;
    }

    return impl_->storage->store(it->second);
}

LifecycleResult LifecycleManager::load_entity(EntityId entity_id) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->storage) {
        return LifecycleResult::InvalidConfiguration;
    }

    auto info_opt = impl_->storage->load(entity_id);
    if (!info_opt.has_value()) {
        return LifecycleResult::EntityNotTracked;
    }

    impl_->entities[entity_id] = info_opt.value();
    return LifecycleResult::Success;
}

// ============================================================================
// Statistics
// ============================================================================

LifecycleStats LifecycleManager::get_stats() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->stats;
}

void LifecycleManager::reset_stats() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->stats.reset();
}

// ============================================================================
// Maintenance
// ============================================================================

UInt64 LifecycleManager::cleanup_old_data() {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    UInt64 removed_count = 0;
    auto now = std::chrono::system_clock::now();
    auto retention_period = impl_->config.retention_period;

    for (auto& [entity_id, info] : impl_->entities) {
        // Remove old transitions
        auto transition_it = info.transition_history.begin();
        while (transition_it != info.transition_history.end()) {
            auto age = std::chrono::duration_cast<std::chrono::seconds>(
                now - transition_it->timestamp);
            if (age > retention_period) {
                transition_it = info.transition_history.erase(transition_it);
                removed_count++;
            } else {
                ++transition_it;
            }
        }

        // Remove old events
        auto event_it = info.events.begin();
        while (event_it != info.events.end()) {
            auto age = std::chrono::duration_cast<std::chrono::seconds>(
                now - event_it->timestamp);
            if (age > retention_period) {
                event_it = info.events.erase(event_it);
                removed_count++;
            } else {
                ++event_it;
            }
        }

        // Save updated info to storage if configured
        if (impl_->storage) {
            impl_->storage->store(info);
        }
    }

    return removed_count;
}

UInt64 LifecycleManager::trim_history() {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    UInt64 removed_count = 0;
    UInt32 max_history = impl_->config.max_history_size;

    for (auto& [entity_id, info] : impl_->entities) {
        // Trim transition history
        if (info.transition_history.size() > max_history) {
            UInt32 to_remove = static_cast<UInt32>(info.transition_history.size()) - max_history;
            info.transition_history.erase(
                info.transition_history.begin(),
                info.transition_history.begin() + to_remove);
            removed_count += to_remove;
        }

        // Trim event history
        if (info.events.size() > max_history) {
            UInt32 to_remove = static_cast<UInt32>(info.events.size()) - max_history;
            info.events.erase(
                info.events.begin(),
                info.events.begin() + to_remove);
            removed_count += to_remove;
        }

        // Save updated info to storage if configured
        if (impl_->storage) {
            impl_->storage->store(info);
        }
    }

    return removed_count;
}

void LifecycleManager::clear() {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    impl_->entities.clear();
    impl_->stats.reset();

    // Clear storage if configured
    if (impl_->storage) {
        auto entity_ids = impl_->storage->list_entities();
        for (EntityId entity_id : entity_ids) {
            impl_->storage->remove(entity_id);
        }
    }
}

// ============================================================================
// Factory Functions
// ============================================================================

std::unique_ptr<LifecycleManager> create_lifecycle_manager(
    const LifecycleConfig& config) {
    return std::make_unique<LifecycleManager>(config);
}

std::unique_ptr<ILifecycleStorage> create_memory_storage() {
    return std::make_unique<SimpleLifecycleStorage>();
}

} // namespace jaguar::thread
