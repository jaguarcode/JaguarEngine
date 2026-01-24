#pragma once
/**
 * @file lifecycle_manager.h
 * @brief Entity lifecycle management for Digital Thread
 *
 * This file provides the lifecycle infrastructure for tracking entities
 * throughout their complete lifecycle from design through decommissioning.
 * Supports phase transitions, event logging, and observer notifications.
 *
 * Key features:
 * - Lifecycle phase tracking (Design, Production, Operational, Maintenance, Decommissioned)
 * - Transition history with timestamps and audit trail
 * - Event logging for significant lifecycle events
 * - Observer pattern for lifecycle change notifications
 * - Persistent storage interface for long-term tracking
 * - Configurable transition validation and retention policies
 */

#include "jaguar/core/types.h"
#include <vector>
#include <unordered_map>
#include <memory>
#include <functional>
#include <optional>
#include <chrono>
#include <string>

namespace jaguar::thread {

// ============================================================================
// Forward Declarations
// ============================================================================

class ILifecycleObserver;
class ILifecycleStorage;
class LifecycleManager;

// ============================================================================
// Lifecycle Result Enum
// ============================================================================

/**
 * @brief Result codes for lifecycle operations
 */
enum class LifecycleResult : UInt8 {
    Success = 0,

    // Configuration errors
    InvalidConfiguration,
    InvalidEntityId,
    InvalidPhase,

    // Operational errors
    TransitionFailed,
    TransitionNotAllowed,
    ObserverFailed,

    // State errors
    NotInitialized,
    AlreadyInitialized,
    EntityNotTracked,

    // Resource errors
    OutOfMemory,
    StorageError
};

/**
 * @brief Convert LifecycleResult to string
 */
inline const char* lifecycle_result_to_string(LifecycleResult result) {
    switch (result) {
        case LifecycleResult::Success: return "Success";
        case LifecycleResult::InvalidConfiguration: return "InvalidConfiguration";
        case LifecycleResult::InvalidEntityId: return "InvalidEntityId";
        case LifecycleResult::InvalidPhase: return "InvalidPhase";
        case LifecycleResult::TransitionFailed: return "TransitionFailed";
        case LifecycleResult::TransitionNotAllowed: return "TransitionNotAllowed";
        case LifecycleResult::ObserverFailed: return "ObserverFailed";
        case LifecycleResult::NotInitialized: return "NotInitialized";
        case LifecycleResult::AlreadyInitialized: return "AlreadyInitialized";
        case LifecycleResult::EntityNotTracked: return "EntityNotTracked";
        case LifecycleResult::OutOfMemory: return "OutOfMemory";
        case LifecycleResult::StorageError: return "StorageError";
        default: return "Unknown";
    }
}

// ============================================================================
// Lifecycle Phase Enum
// ============================================================================

/**
 * @brief Lifecycle phases for entity tracking
 */
enum class LifecyclePhase : UInt8 {
    /// Entity in design/planning phase
    Design = 0,

    /// Entity being manufactured/built
    Production,

    /// Entity actively in use
    Operational,

    /// Entity undergoing maintenance
    Maintenance,

    /// Entity retired from service
    Decommissioned
};

/**
 * @brief Convert LifecyclePhase to string
 */
inline const char* lifecycle_phase_to_string(LifecyclePhase phase) {
    switch (phase) {
        case LifecyclePhase::Design: return "Design";
        case LifecyclePhase::Production: return "Production";
        case LifecyclePhase::Operational: return "Operational";
        case LifecyclePhase::Maintenance: return "Maintenance";
        case LifecyclePhase::Decommissioned: return "Decommissioned";
        default: return "Unknown";
    }
}

// ============================================================================
// Lifecycle Transition
// ============================================================================

/**
 * @brief Record of a lifecycle phase transition
 */
struct LifecycleTransition {
    /// Entity that transitioned
    EntityId entity_id{INVALID_ENTITY_ID};

    /// Phase transitioning from
    LifecyclePhase from_phase{LifecyclePhase::Design};

    /// Phase transitioning to
    LifecyclePhase to_phase{LifecyclePhase::Design};

    /// When transition occurred
    std::chrono::system_clock::time_point timestamp;

    /// Reason for transition
    std::string reason;

    /// Who made the change (user ID, system name, etc.)
    std::string operator_id;

    LifecycleTransition() = default;

    LifecycleTransition(EntityId id, LifecyclePhase from, LifecyclePhase to,
                        std::string reason_ = "", std::string operator_ = "")
        : entity_id(id)
        , from_phase(from)
        , to_phase(to)
        , timestamp(std::chrono::system_clock::now())
        , reason(std::move(reason_))
        , operator_id(std::move(operator_)) {}
};

// ============================================================================
// Lifecycle Event
// ============================================================================

/**
 * @brief Generic lifecycle event for significant occurrences
 */
struct LifecycleEvent {
    /// Entity the event applies to
    EntityId entity_id{INVALID_ENTITY_ID};

    /// Type of event (e.g., "inspection", "repair", "upgrade", "incident")
    std::string event_type;

    /// Detailed description of the event
    std::string description;

    /// When event occurred
    std::chrono::system_clock::time_point timestamp;

    /// Additional metadata as key-value pairs
    std::unordered_map<std::string, std::string> metadata;

    LifecycleEvent() = default;

    LifecycleEvent(EntityId id, std::string type, std::string desc = "")
        : entity_id(id)
        , event_type(std::move(type))
        , description(std::move(desc))
        , timestamp(std::chrono::system_clock::now()) {}
};

// ============================================================================
// Entity Lifecycle Info
// ============================================================================

/**
 * @brief Complete lifecycle information for an entity
 */
struct EntityLifecycleInfo {
    /// Entity identifier
    EntityId entity_id{INVALID_ENTITY_ID};

    /// Current lifecycle phase
    LifecyclePhase current_phase{LifecyclePhase::Design};

    /// When entity was first tracked
    std::chrono::system_clock::time_point created_at;

    /// When last transition occurred
    std::chrono::system_clock::time_point last_transition;

    /// Total number of transitions
    UInt32 transition_count{0};

    /// Complete transition history
    std::vector<LifecycleTransition> transition_history;

    /// All logged events
    std::vector<LifecycleEvent> events;

    EntityLifecycleInfo() = default;

    explicit EntityLifecycleInfo(EntityId id, LifecyclePhase initial = LifecyclePhase::Design)
        : entity_id(id)
        , current_phase(initial)
        , created_at(std::chrono::system_clock::now())
        , last_transition(created_at)
        , transition_count(0) {}
};

// ============================================================================
// Lifecycle Configuration
// ============================================================================

/**
 * @brief Configuration for the lifecycle manager
 */
struct LifecycleConfig {
    /// Maximum number of history entries to retain per entity
    UInt32 max_history_size{1000};

    /// Enable event logging
    bool enable_event_logging{true};

    /// How long to retain lifecycle data
    std::chrono::seconds retention_period{std::chrono::hours(24 * 30)}; // 30 days default

    /// Enforce valid transition paths (prevent invalid phase changes)
    bool strict_transitions{true};

    /// Callback invoked on every transition (optional)
    std::function<void(const LifecycleTransition&)> transition_callback;

    /// Factory methods for common configurations
    static LifecycleConfig default_config() noexcept {
        return LifecycleConfig{};
    }

    static LifecycleConfig minimal() noexcept {
        LifecycleConfig config;
        config.max_history_size = 100;
        config.enable_event_logging = false;
        config.strict_transitions = false;
        return config;
    }

    static LifecycleConfig full_audit() noexcept {
        LifecycleConfig config;
        config.max_history_size = 10000;
        config.enable_event_logging = true;
        config.retention_period = std::chrono::hours(24 * 365); // 1 year
        config.strict_transitions = true;
        return config;
    }

    static LifecycleConfig production() noexcept {
        LifecycleConfig config;
        config.max_history_size = 5000;
        config.enable_event_logging = true;
        config.retention_period = std::chrono::hours(24 * 90); // 90 days
        config.strict_transitions = true;
        return config;
    }
};

// ============================================================================
// Lifecycle Statistics
// ============================================================================

/**
 * @brief Statistics about lifecycle operations
 */
struct LifecycleStats {
    /// Total entities being tracked
    UInt64 total_entities{0};

    /// Total transitions recorded
    UInt64 total_transitions{0};

    /// Total events logged
    UInt64 total_events{0};

    /// Number of entities in each phase
    std::unordered_map<LifecyclePhase, UInt64> entities_per_phase;

    /// Average transitions per entity
    Real average_transitions_per_entity{0.0};

    /// Average events per entity
    Real average_events_per_entity{0.0};

    /// Reset all counters
    void reset() noexcept {
        *this = LifecycleStats{};
    }
};

// ============================================================================
// Lifecycle Observer Interface
// ============================================================================

/**
 * @brief Interface for observing lifecycle changes
 *
 * Implement this interface to receive notifications about lifecycle events.
 * All methods have default implementations (no-op) so you only override what you need.
 */
class ILifecycleObserver {
public:
    virtual ~ILifecycleObserver() = default;

    /**
     * @brief Called when an entity transitions between phases
     * @param entity_id Entity that transitioned
     * @param from_phase Previous phase
     * @param to_phase New phase
     */
    virtual void on_phase_changed(EntityId entity_id,
                                   LifecyclePhase from_phase,
                                   LifecyclePhase to_phase) {
        // Default: no-op
        (void)entity_id;
        (void)from_phase;
        (void)to_phase;
    }

    /**
     * @brief Called when an event is logged for an entity
     * @param entity_id Entity the event applies to
     * @param event The logged event
     */
    virtual void on_event_logged(EntityId entity_id,
                                  const LifecycleEvent& event) {
        // Default: no-op
        (void)entity_id;
        (void)event;
    }

    /**
     * @brief Called when a new entity starts being tracked
     * @param entity_id New entity
     * @param initial_phase Starting phase
     */
    virtual void on_entity_created(EntityId entity_id,
                                    LifecyclePhase initial_phase) {
        // Default: no-op
        (void)entity_id;
        (void)initial_phase;
    }

    /**
     * @brief Called when an entity is removed from tracking
     * @param entity_id Removed entity
     */
    virtual void on_entity_removed(EntityId entity_id) {
        // Default: no-op
        (void)entity_id;
    }
};

// ============================================================================
// Lifecycle Storage Interface
// ============================================================================

/**
 * @brief Interface for persistent lifecycle storage
 *
 * Implement this interface to provide persistent storage for lifecycle data.
 * Allows lifecycle information to survive application restarts.
 */
class ILifecycleStorage {
public:
    virtual ~ILifecycleStorage() = default;

    /**
     * @brief Store lifecycle information for an entity
     * @param info Complete lifecycle info to store
     * @return Success or error code
     */
    virtual LifecycleResult store(const EntityLifecycleInfo& info) = 0;

    /**
     * @brief Load lifecycle information for an entity
     * @param entity_id Entity to load
     * @return Lifecycle info if found, nullopt otherwise
     */
    virtual std::optional<EntityLifecycleInfo> load(EntityId entity_id) = 0;

    /**
     * @brief Remove lifecycle information for an entity
     * @param entity_id Entity to remove
     * @return Success or error code
     */
    virtual LifecycleResult remove(EntityId entity_id) = 0;

    /**
     * @brief List all entities in storage
     * @return Vector of entity IDs
     */
    virtual std::vector<EntityId> list_entities() = 0;
};

// ============================================================================
// Lifecycle Manager
// ============================================================================

/**
 * @brief Main lifecycle manager for Digital Thread entity tracking
 *
 * Manages complete lifecycle tracking for entities from design through
 * decommissioning. Tracks phase transitions, logs events, notifies observers,
 * and optionally persists data for long-term audit trails.
 */
class LifecycleManager {
public:
    /**
     * @brief Construct lifecycle manager with configuration
     * @param config Lifecycle configuration
     */
    explicit LifecycleManager(const LifecycleConfig& config = LifecycleConfig::default_config());

    ~LifecycleManager();

    // Non-copyable, movable
    LifecycleManager(const LifecycleManager&) = delete;
    LifecycleManager& operator=(const LifecycleManager&) = delete;
    LifecycleManager(LifecycleManager&&) noexcept;
    LifecycleManager& operator=(LifecycleManager&&) noexcept;

    // ========================================================================
    // Lifecycle
    // ========================================================================

    /**
     * @brief Initialize lifecycle manager
     * @return Success or error code
     */
    LifecycleResult initialize();

    /**
     * @brief Shutdown and cleanup
     * @return Success or error code
     */
    LifecycleResult shutdown();

    /**
     * @brief Check if initialized
     */
    bool is_initialized() const noexcept;

    /**
     * @brief Get current configuration
     */
    const LifecycleConfig& get_config() const noexcept;

    // ========================================================================
    // Entity Tracking
    // ========================================================================

    /**
     * @brief Start tracking an entity
     * @param entity_id Entity to track
     * @param initial_phase Starting lifecycle phase
     * @return Success or error code
     */
    LifecycleResult track_entity(EntityId entity_id,
                                  LifecyclePhase initial_phase = LifecyclePhase::Design);

    /**
     * @brief Stop tracking an entity
     * @param entity_id Entity to untrack
     * @return Success or error code
     */
    LifecycleResult untrack_entity(EntityId entity_id);

    /**
     * @brief Check if entity is being tracked
     * @param entity_id Entity to check
     * @return True if tracked, false otherwise
     */
    bool is_tracked(EntityId entity_id) const noexcept;

    /**
     * @brief Get number of tracked entities
     */
    UInt64 get_entity_count() const noexcept;

    // ========================================================================
    // Phase Transitions
    // ========================================================================

    /**
     * @brief Transition entity to new phase
     * @param entity_id Entity to transition
     * @param to_phase Target phase
     * @param reason Reason for transition (for audit trail)
     * @param operator_id Who initiated the transition (optional)
     * @return Success or error code
     */
    LifecycleResult transition(EntityId entity_id,
                                LifecyclePhase to_phase,
                                std::string reason,
                                std::string operator_id = "");

    /**
     * @brief Get current phase of an entity
     * @param entity_id Entity to query
     * @return Current phase if tracked, nullopt otherwise
     */
    std::optional<LifecyclePhase> get_phase(EntityId entity_id) const noexcept;

    /**
     * @brief Check if a transition is valid
     * @param from_phase Source phase
     * @param to_phase Target phase
     * @return True if transition is allowed
     */
    bool is_valid_transition(LifecyclePhase from_phase,
                              LifecyclePhase to_phase) const noexcept;

    // ========================================================================
    // Event Logging
    // ========================================================================

    /**
     * @brief Log an event for an entity
     * @param entity_id Entity the event applies to
     * @param event Event to log
     * @return Success or error code
     */
    LifecycleResult log_event(EntityId entity_id,
                               LifecycleEvent event);

    /**
     * @brief Log a simple event
     * @param entity_id Entity the event applies to
     * @param event_type Type of event
     * @param description Event description
     * @return Success or error code
     */
    LifecycleResult log_event(EntityId entity_id,
                               const std::string& event_type,
                               const std::string& description);

    // ========================================================================
    // Queries
    // ========================================================================

    /**
     * @brief Get complete lifecycle information for an entity
     * @param entity_id Entity to query
     * @return Lifecycle info if tracked, nullopt otherwise
     */
    std::optional<EntityLifecycleInfo> get_lifecycle_info(EntityId entity_id) const;

    /**
     * @brief Get all entities in a specific phase
     * @param phase Phase to query
     * @return Vector of entity IDs
     */
    std::vector<EntityId> get_entities_in_phase(LifecyclePhase phase) const;

    /**
     * @brief Get transition history for an entity
     * @param entity_id Entity to query
     * @return Vector of transitions (empty if not tracked)
     */
    std::vector<LifecycleTransition> get_transition_history(EntityId entity_id) const;

    /**
     * @brief Get event history for an entity
     * @param entity_id Entity to query
     * @return Vector of events (empty if not tracked)
     */
    std::vector<LifecycleEvent> get_event_history(EntityId entity_id) const;

    /**
     * @brief Get time in current phase
     * @param entity_id Entity to query
     * @return Duration in current phase, or nullopt if not tracked
     */
    std::optional<std::chrono::seconds> get_time_in_phase(EntityId entity_id) const;

    /**
     * @brief Get total lifetime of entity
     * @param entity_id Entity to query
     * @return Duration since first tracked, or nullopt if not tracked
     */
    std::optional<std::chrono::seconds> get_lifetime(EntityId entity_id) const;

    // ========================================================================
    // Observers
    // ========================================================================

    /**
     * @brief Add an observer to receive lifecycle notifications
     * @param observer Shared pointer to observer
     * @return Success or error code
     */
    LifecycleResult add_observer(std::shared_ptr<ILifecycleObserver> observer);

    /**
     * @brief Remove an observer
     * @param observer Observer to remove
     * @return Success or error code
     */
    LifecycleResult remove_observer(std::shared_ptr<ILifecycleObserver> observer);

    /**
     * @brief Remove all observers
     */
    void clear_observers();

    /**
     * @brief Get number of registered observers
     */
    UInt32 get_observer_count() const noexcept;

    // ========================================================================
    // Storage
    // ========================================================================

    /**
     * @brief Set persistent storage backend
     * @param storage Shared pointer to storage implementation
     * @return Success or error code
     */
    LifecycleResult set_storage(std::shared_ptr<ILifecycleStorage> storage);

    /**
     * @brief Get current storage backend
     * @return Storage backend or nullptr if none set
     */
    std::shared_ptr<ILifecycleStorage> get_storage() const noexcept;

    /**
     * @brief Save all tracked entities to storage
     * @return Success or error code
     */
    LifecycleResult save_all();

    /**
     * @brief Load all entities from storage
     * @return Success or error code
     */
    LifecycleResult load_all();

    /**
     * @brief Save specific entity to storage
     * @param entity_id Entity to save
     * @return Success or error code
     */
    LifecycleResult save_entity(EntityId entity_id);

    /**
     * @brief Load specific entity from storage
     * @param entity_id Entity to load
     * @return Success or error code
     */
    LifecycleResult load_entity(EntityId entity_id);

    // ========================================================================
    // Statistics
    // ========================================================================

    /**
     * @brief Get lifecycle statistics
     */
    LifecycleStats get_stats() const;

    /**
     * @brief Reset statistics
     */
    void reset_stats();

    // ========================================================================
    // Maintenance
    // ========================================================================

    /**
     * @brief Clean up old data based on retention policy
     * @return Number of entries removed
     */
    UInt64 cleanup_old_data();

    /**
     * @brief Trim history to max_history_size for all entities
     * @return Number of entries removed
     */
    UInt64 trim_history();

    /**
     * @brief Clear all data (reset to empty state)
     */
    void clear();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// ============================================================================
// Factory Functions
// ============================================================================

/**
 * @brief Create lifecycle manager with configuration
 * @param config Lifecycle configuration
 * @return Unique pointer to lifecycle manager
 */
std::unique_ptr<LifecycleManager> create_lifecycle_manager(
    const LifecycleConfig& config = LifecycleConfig::default_config());

/**
 * @brief Create in-memory storage backend
 * @return Unique pointer to storage implementation
 */
std::unique_ptr<ILifecycleStorage> create_memory_storage();

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * @brief Get all valid transitions from a given phase
 * @param phase Source phase
 * @return Vector of valid target phases
 */
inline std::vector<LifecyclePhase> get_valid_transitions(LifecyclePhase phase) {
    std::vector<LifecyclePhase> valid;

    switch (phase) {
        case LifecyclePhase::Design:
            // From Design: can go to Production or Decommissioned (cancelled)
            valid.push_back(LifecyclePhase::Production);
            valid.push_back(LifecyclePhase::Decommissioned);
            break;

        case LifecyclePhase::Production:
            // From Production: can go to Operational or back to Design (rework)
            valid.push_back(LifecyclePhase::Operational);
            valid.push_back(LifecyclePhase::Design);
            break;

        case LifecyclePhase::Operational:
            // From Operational: can go to Maintenance or Decommissioned
            valid.push_back(LifecyclePhase::Maintenance);
            valid.push_back(LifecyclePhase::Decommissioned);
            break;

        case LifecyclePhase::Maintenance:
            // From Maintenance: can return to Operational or be Decommissioned
            valid.push_back(LifecyclePhase::Operational);
            valid.push_back(LifecyclePhase::Decommissioned);
            break;

        case LifecyclePhase::Decommissioned:
            // From Decommissioned: terminal state, no valid transitions
            break;
    }

    return valid;
}

/**
 * @brief Check if transition is valid according to standard rules
 * @param from_phase Source phase
 * @param to_phase Target phase
 * @return True if transition is allowed
 */
inline bool is_transition_valid(LifecyclePhase from_phase, LifecyclePhase to_phase) {
    // Same phase is always valid (no-op)
    if (from_phase == to_phase) {
        return true;
    }

    auto valid = get_valid_transitions(from_phase);
    for (auto phase : valid) {
        if (phase == to_phase) {
            return true;
        }
    }

    return false;
}

/**
 * @brief Format timestamp as ISO 8601 string
 * @param time_point Time to format
 * @return ISO 8601 formatted string
 */
inline std::string format_timestamp(const std::chrono::system_clock::time_point& time_point) {
    auto time_t = std::chrono::system_clock::to_time_t(time_point);
    char buffer[32];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", std::gmtime(&time_t));
    return std::string(buffer);
}

/**
 * @brief Calculate duration between two time points
 * @param start Start time
 * @param end End time
 * @return Duration in seconds
 */
inline std::chrono::seconds calculate_duration(
    const std::chrono::system_clock::time_point& start,
    const std::chrono::system_clock::time_point& end) {
    return std::chrono::duration_cast<std::chrono::seconds>(end - start);
}

} // namespace jaguar::thread
