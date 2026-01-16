// Copyright JaguarEngine Team. All Rights Reserved.
//
// Multi-Federation Orchestration System
// Coordinates multiple HLA federations for large-scale distributed simulations

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <variant>
#include <vector>

namespace jaguar::federation {

//==============================================================================
// Forward Declarations
//==============================================================================

class Federation;
class FederationBridge;
class FederationOrchestrator;
class TimeManager;
class EntityRouter;

//==============================================================================
// Types and Enumerations
//==============================================================================

/// Federation connection state
enum class FederationState {
    Disconnected,
    Connecting,
    Connected,
    Joined,
    Resigned,
    Error
};

/// Time management mode
enum class TimeManagementMode {
    Regulating,        ///< This federate controls time advancement
    Constrained,       ///< This federate follows time advancement
    Both,              ///< Both regulating and constrained
    None               ///< No time management (best effort)
};

/// Entity ownership state
enum class OwnershipState {
    Unowned,
    Owned,
    AcquisitionPending,
    DivesturePending
};

/// Federation synchronization point
struct SyncPoint {
    std::string label;
    bool achieved = false;
    std::chrono::system_clock::time_point announced_at;
    std::chrono::system_clock::time_point achieved_at;
};

/// Federation statistics
struct FederationStats {
    std::string federation_name;
    std::string federate_name;
    FederationState state = FederationState::Disconnected;

    // Entity counts
    uint32_t local_entities = 0;
    uint32_t remote_entities = 0;
    uint32_t reflected_entities = 0;

    // Interaction counts
    uint64_t interactions_sent = 0;
    uint64_t interactions_received = 0;

    // Time management
    double logical_time = 0.0;
    double lookahead = 0.0;
    bool time_regulating = false;
    bool time_constrained = false;

    // Performance
    double avg_advance_time_ms = 0.0;
    double max_advance_time_ms = 0.0;
    uint64_t time_advance_requests = 0;
    uint64_t time_advance_grants = 0;

    // Network
    uint64_t bytes_sent = 0;
    uint64_t bytes_received = 0;
    double latency_ms = 0.0;
};

//==============================================================================
// Entity Identifier
//==============================================================================

/// Unique entity identifier across federations
struct EntityIdentifier {
    uint16_t site_id = 0;
    uint16_t application_id = 0;
    uint16_t entity_number = 0;

    bool operator==(const EntityIdentifier& other) const {
        return site_id == other.site_id &&
               application_id == other.application_id &&
               entity_number == other.entity_number;
    }

    bool operator!=(const EntityIdentifier& other) const {
        return !(*this == other);
    }

    std::string to_string() const;
    static EntityIdentifier from_string(std::string_view str);
};

/// Hash function for EntityIdentifier
struct EntityIdentifierHash {
    size_t operator()(const EntityIdentifier& id) const {
        return (static_cast<size_t>(id.site_id) << 32) |
               (static_cast<size_t>(id.application_id) << 16) |
               static_cast<size_t>(id.entity_number);
    }
};

//==============================================================================
// Federation Configuration
//==============================================================================

/// RTI connection configuration
struct RTIConfig {
    std::string rti_host = "localhost";
    uint16_t rti_port = 8989;
    std::string rti_type = "OpenRTI";  // OpenRTI, Pitch, MAK, etc.
    std::chrono::milliseconds connect_timeout{30000};
    std::chrono::milliseconds operation_timeout{10000};
    bool use_tls = false;
    std::string tls_cert_path;
    std::string tls_key_path;
};

/// Federation configuration
struct FederationConfig {
    std::string federation_name;
    std::string federate_name;
    std::vector<std::string> fom_modules;

    // Time management
    TimeManagementMode time_mode = TimeManagementMode::Both;
    double lookahead = 0.1;

    // RTI connection
    RTIConfig rti_config;

    // Entity management
    uint16_t site_id = 1;
    uint16_t application_id = 1;

    // Optional settings
    bool auto_reconnect = true;
    std::chrono::milliseconds reconnect_delay{5000};
    uint32_t max_reconnect_attempts = 10;
};

//==============================================================================
// Federation Bridge Configuration
//==============================================================================

/// Entity filter for bridge routing
struct EntityFilter {
    enum class FilterType {
        All,              ///< All entities
        ByDomain,         ///< Filter by domain (air, land, sea, etc.)
        ByEntityType,     ///< Filter by DIS entity type
        ByGeographic,     ///< Filter by geographic region
        Custom            ///< Custom filter function
    };

    FilterType type = FilterType::All;

    // Domain filter
    std::vector<uint8_t> domains;

    // Entity type filter
    std::vector<std::tuple<uint8_t, uint8_t, uint8_t>> entity_types;

    // Geographic filter (lat_min, lat_max, lon_min, lon_max)
    std::optional<std::tuple<double, double, double, double>> geo_bounds;

    // Custom filter
    std::function<bool(const EntityIdentifier&)> custom_filter;

    bool matches(const EntityIdentifier& entity) const;
};

/// Bridge configuration between federations
struct BridgeConfig {
    std::string source_federation;
    std::string target_federation;

    // Entity routing
    EntityFilter entity_filter;
    bool bidirectional = false;

    // Data transformation
    bool transform_coordinates = false;  // E.g., different datums
    bool transform_time = false;         // E.g., different time scales

    // Performance
    double update_rate_hz = 60.0;
    bool dead_reckoning_enabled = true;
    double dead_reckoning_threshold = 1.0;  // meters
};

//==============================================================================
// Event Types
//==============================================================================

/// Federation event type
enum class FederationEventType {
    Connected,
    Disconnected,
    Joined,
    Resigned,
    SyncPointAnnounced,
    SyncPointAchieved,
    TimeAdvanceGrant,
    EntityDiscovered,
    EntityRemoved,
    AttributeUpdated,
    InteractionReceived,
    OwnershipAcquired,
    OwnershipDivested,
    Error
};

/// Federation event
struct FederationEvent {
    FederationEventType type;
    std::string federation_name;
    std::chrono::system_clock::time_point timestamp;

    // Event-specific data
    std::variant<
        std::monostate,
        std::string,                    // Error message, sync point label
        double,                         // Time value
        EntityIdentifier                // Entity ID
    > data;
};

/// Event callback type
using FederationEventCallback = std::function<void(const FederationEvent&)>;

//==============================================================================
// Time Manager
//==============================================================================

class TimeManager {
public:
    TimeManager();
    ~TimeManager();

    /// Set time management mode
    void set_mode(TimeManagementMode mode);
    TimeManagementMode mode() const { return mode_; }

    /// Set lookahead value
    void set_lookahead(double lookahead);
    double lookahead() const { return lookahead_; }

    /// Get current logical time
    double logical_time() const { return logical_time_; }

    /// Request time advance
    void request_advance(double target_time);

    /// Grant time advance (called by RTI callback)
    void grant_advance(double granted_time);

    /// Check if advance is pending
    bool advance_pending() const { return advance_pending_; }

    /// Wait for time advance grant
    bool wait_for_grant(std::chrono::milliseconds timeout);

    /// Get statistics
    struct Stats {
        uint64_t requests = 0;
        uint64_t grants = 0;
        double avg_wait_ms = 0.0;
        double max_wait_ms = 0.0;
    };
    Stats get_stats() const;

private:
    TimeManagementMode mode_ = TimeManagementMode::Both;
    double lookahead_ = 0.1;
    std::atomic<double> logical_time_{0.0};
    std::atomic<double> requested_time_{0.0};
    std::atomic<bool> advance_pending_{false};

    struct Impl;
    std::unique_ptr<Impl> impl_;
};

//==============================================================================
// Entity Router
//==============================================================================

class EntityRouter {
public:
    EntityRouter();
    ~EntityRouter();

    /// Register entity with federation
    void register_entity(const EntityIdentifier& entity,
                        const std::string& federation);

    /// Unregister entity
    void unregister_entity(const EntityIdentifier& entity);

    /// Get federation for entity
    std::optional<std::string> get_federation(const EntityIdentifier& entity) const;

    /// Get all entities in federation
    std::vector<EntityIdentifier> get_entities(const std::string& federation) const;

    /// Check if entity should be routed through bridge
    bool should_route(const EntityIdentifier& entity,
                     const BridgeConfig& bridge) const;

    /// Get ownership state
    OwnershipState get_ownership_state(const EntityIdentifier& entity) const;

    /// Set ownership state
    void set_ownership_state(const EntityIdentifier& entity,
                            OwnershipState state);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

//==============================================================================
// Federation
//==============================================================================

class Federation {
public:
    explicit Federation(const FederationConfig& config);
    ~Federation();

    // Non-copyable
    Federation(const Federation&) = delete;
    Federation& operator=(const Federation&) = delete;

    /// Get configuration
    const FederationConfig& config() const { return config_; }

    /// Get current state
    FederationState state() const { return state_; }

    /// Connect to RTI
    bool connect();

    /// Disconnect from RTI
    void disconnect();

    /// Create and join federation
    bool create_and_join();

    /// Join existing federation
    bool join();

    /// Resign from federation
    void resign();

    /// Synchronize at sync point
    bool announce_sync_point(const std::string& label);
    bool achieve_sync_point(const std::string& label);
    bool wait_for_sync_point(const std::string& label,
                             std::chrono::milliseconds timeout);

    /// Time management
    TimeManager& time_manager() { return *time_manager_; }
    const TimeManager& time_manager() const { return *time_manager_; }

    /// Register event callback
    void on_event(FederationEventCallback callback);

    /// Get statistics
    FederationStats get_stats() const;

    /// Tick federation (process callbacks, advance time)
    void tick(double dt);

private:
    FederationConfig config_;
    std::atomic<FederationState> state_{FederationState::Disconnected};
    std::unique_ptr<TimeManager> time_manager_;
    std::vector<FederationEventCallback> callbacks_;

    struct Impl;
    std::unique_ptr<Impl> impl_;

    void emit_event(const FederationEvent& event);
};

//==============================================================================
// Federation Bridge
//==============================================================================

class FederationBridge {
public:
    FederationBridge(const BridgeConfig& config,
                    Federation& source,
                    Federation& target);
    ~FederationBridge();

    /// Get configuration
    const BridgeConfig& config() const { return config_; }

    /// Enable/disable bridge
    void set_enabled(bool enabled);
    bool is_enabled() const { return enabled_; }

    /// Process entity updates
    void process_updates();

    /// Get statistics
    struct Stats {
        uint64_t entities_routed = 0;
        uint64_t updates_forwarded = 0;
        uint64_t updates_filtered = 0;
        double avg_latency_ms = 0.0;
    };
    Stats get_stats() const;

private:
    BridgeConfig config_;
    Federation& source_;
    Federation& target_;
    std::atomic<bool> enabled_{true};

    struct Impl;
    std::unique_ptr<Impl> impl_;
};

//==============================================================================
// Federation Orchestrator
//==============================================================================

class FederationOrchestrator {
public:
    /// Get singleton instance
    static FederationOrchestrator& instance();

    /// Create a new federation
    Federation& create_federation(const FederationConfig& config);

    /// Get federation by name
    Federation* get_federation(const std::string& name);

    /// Remove federation
    void remove_federation(const std::string& name);

    /// Create bridge between federations
    FederationBridge& create_bridge(const BridgeConfig& config);

    /// Remove bridge
    void remove_bridge(const std::string& source, const std::string& target);

    /// Get entity router
    EntityRouter& entity_router() { return *entity_router_; }

    /// Register global event callback
    void on_event(FederationEventCallback callback);

    /// Start orchestrator
    void start();

    /// Stop orchestrator
    void stop();

    /// Tick all federations
    void tick(double dt);

    /// Get all federation statistics
    std::vector<FederationStats> get_all_stats() const;

    /// Synchronize all federations at sync point
    bool global_sync(const std::string& label,
                    std::chrono::milliseconds timeout);

    /// Shutdown all federations
    void shutdown();

private:
    FederationOrchestrator();
    ~FederationOrchestrator();

    FederationOrchestrator(const FederationOrchestrator&) = delete;
    FederationOrchestrator& operator=(const FederationOrchestrator&) = delete;

    std::unique_ptr<EntityRouter> entity_router_;
    std::unordered_map<std::string, std::unique_ptr<Federation>> federations_;
    std::vector<std::unique_ptr<FederationBridge>> bridges_;
    std::vector<FederationEventCallback> global_callbacks_;
    std::atomic<bool> running_{false};

    struct Impl;
    std::unique_ptr<Impl> impl_;
};

//==============================================================================
// Convenience Functions
//==============================================================================

/// Get orchestrator instance
inline FederationOrchestrator& orchestrator() {
    return FederationOrchestrator::instance();
}

/// Quick federation setup
inline Federation& setup_federation(
    const std::string& name,
    const std::string& federate_name,
    const std::vector<std::string>& fom_modules,
    const std::string& rti_host = "localhost",
    uint16_t rti_port = 8989) {

    FederationConfig config;
    config.federation_name = name;
    config.federate_name = federate_name;
    config.fom_modules = fom_modules;
    config.rti_config.rti_host = rti_host;
    config.rti_config.rti_port = rti_port;

    return orchestrator().create_federation(config);
}

/// Quick bridge setup
inline FederationBridge& setup_bridge(
    const std::string& source,
    const std::string& target,
    bool bidirectional = false) {

    BridgeConfig config;
    config.source_federation = source;
    config.target_federation = target;
    config.bidirectional = bidirectional;

    return orchestrator().create_bridge(config);
}

} // namespace jaguar::federation
