#pragma once
/**
 * @file state_sync.h
 * @brief State synchronization system for distributed simulation
 *
 * This file provides state synchronization infrastructure for maintaining
 * consistency across distributed compute nodes. Implements delta-based
 * synchronization, snapshot management, and optimistic conflict resolution.
 *
 * Key features:
 * - Delta-based state synchronization with vector clocks
 * - Snapshot manager for consistency checkpoints
 * - Optimistic conflict resolution with rollback support
 * - < 30 second failover handling
 * - State synchronization during entity migration
 * - Integration with partition manager and distributed time
 */

#include "jaguar/core/types.h"
#include "jaguar/cloud/distributed_time.h"
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <functional>
#include <optional>
#include <chrono>
#include <mutex>
#include <atomic>
#include <string>
#include <deque>

namespace jaguar::cloud {

// ============================================================================
// Forward Declarations
// ============================================================================

class IStateSerializer;
class IConflictResolver;
class ISnapshotManager;
class ISyncTransport;
class StateSynchronizer;

// Forward declare types from distributed_time.h
using VectorClock = VectorClock;
using NodeId = NodeId;

// ============================================================================
// Sync Result Enum
// ============================================================================

/**
 * @brief Result codes for state synchronization operations
 */
enum class SyncResult : UInt8 {
    Success = 0,

    // Configuration errors
    InvalidConfiguration,
    InvalidNodeId,
    InvalidEntityId,
    InvalidPartitionId,
    InvalidSnapshotId,
    InvalidVersion,

    // State errors
    NotInitialized,
    AlreadyInitialized,
    ShuttingDown,
    StateTooLarge,
    StateCorrupted,

    // Synchronization errors
    SyncFailed,
    ConflictDetected,
    RollbackRequired,
    RollbackFailed,
    VersionMismatch,
    CausalityViolation,

    // Delta errors
    DeltaTooLarge,
    DeltaApplicationFailed,
    InvalidDelta,
    MissingBaseline,

    // Snapshot errors
    SnapshotFailed,
    SnapshotNotFound,
    SnapshotCorrupted,
    RestoreFailed,
    CheckpointFailed,

    // Network errors
    NetworkError,
    Timeout,
    NodeUnreachable,
    TransportError,
    SerializationFailed,
    DeserializationFailed,

    // Resource errors
    OutOfMemory,
    StorageFull,
    TooManySnapshots,

    // Migration errors
    MigrationInProgress,
    MigrationFailed,
    SourceNodeUnavailable,
    TargetNodeUnavailable
};

/**
 * @brief Convert SyncResult to string
 */
inline const char* sync_result_to_string(SyncResult result) {
    switch (result) {
        case SyncResult::Success: return "Success";
        case SyncResult::InvalidConfiguration: return "InvalidConfiguration";
        case SyncResult::InvalidNodeId: return "InvalidNodeId";
        case SyncResult::InvalidEntityId: return "InvalidEntityId";
        case SyncResult::InvalidPartitionId: return "InvalidPartitionId";
        case SyncResult::InvalidSnapshotId: return "InvalidSnapshotId";
        case SyncResult::InvalidVersion: return "InvalidVersion";
        case SyncResult::NotInitialized: return "NotInitialized";
        case SyncResult::AlreadyInitialized: return "AlreadyInitialized";
        case SyncResult::ShuttingDown: return "ShuttingDown";
        case SyncResult::StateTooLarge: return "StateTooLarge";
        case SyncResult::StateCorrupted: return "StateCorrupted";
        case SyncResult::SyncFailed: return "SyncFailed";
        case SyncResult::ConflictDetected: return "ConflictDetected";
        case SyncResult::RollbackRequired: return "RollbackRequired";
        case SyncResult::RollbackFailed: return "RollbackFailed";
        case SyncResult::VersionMismatch: return "VersionMismatch";
        case SyncResult::CausalityViolation: return "CausalityViolation";
        case SyncResult::DeltaTooLarge: return "DeltaTooLarge";
        case SyncResult::DeltaApplicationFailed: return "DeltaApplicationFailed";
        case SyncResult::InvalidDelta: return "InvalidDelta";
        case SyncResult::MissingBaseline: return "MissingBaseline";
        case SyncResult::SnapshotFailed: return "SnapshotFailed";
        case SyncResult::SnapshotNotFound: return "SnapshotNotFound";
        case SyncResult::SnapshotCorrupted: return "SnapshotCorrupted";
        case SyncResult::RestoreFailed: return "RestoreFailed";
        case SyncResult::CheckpointFailed: return "CheckpointFailed";
        case SyncResult::NetworkError: return "NetworkError";
        case SyncResult::Timeout: return "Timeout";
        case SyncResult::NodeUnreachable: return "NodeUnreachable";
        case SyncResult::TransportError: return "TransportError";
        case SyncResult::SerializationFailed: return "SerializationFailed";
        case SyncResult::DeserializationFailed: return "DeserializationFailed";
        case SyncResult::OutOfMemory: return "OutOfMemory";
        case SyncResult::StorageFull: return "StorageFull";
        case SyncResult::TooManySnapshots: return "TooManySnapshots";
        case SyncResult::MigrationInProgress: return "MigrationInProgress";
        case SyncResult::MigrationFailed: return "MigrationFailed";
        case SyncResult::SourceNodeUnavailable: return "SourceNodeUnavailable";
        case SyncResult::TargetNodeUnavailable: return "TargetNodeUnavailable";
        default: return "Unknown";
    }
}

// ============================================================================
// Synchronization Strategy
// ============================================================================

/**
 * @brief Strategy for state synchronization
 */
enum class SyncStrategy : UInt8 {
    /// Full state transfer (slow but simple)
    Full,

    /// Delta-based (only changes since last sync)
    Delta,

    /// Incremental (cumulative deltas)
    Incremental
};

/**
 * @brief Convert SyncStrategy to string
 */
inline const char* sync_strategy_to_string(SyncStrategy strategy) {
    switch (strategy) {
        case SyncStrategy::Full: return "Full";
        case SyncStrategy::Delta: return "Delta";
        case SyncStrategy::Incremental: return "Incremental";
        default: return "Unknown";
    }
}

// ============================================================================
// Conflict Resolution Strategy
// ============================================================================

/**
 * @brief Strategy for resolving state conflicts
 */
enum class ConflictResolution : UInt8 {
    /// Last write wins (by wall clock time)
    LastWriterWins,

    /// First write wins
    FirstWriterWins,

    /// Merge changes if possible
    Merge,

    /// Rollback conflicting node
    Rollback,

    /// Custom conflict resolver
    Custom
};

/**
 * @brief Convert ConflictResolution to string
 */
inline const char* conflict_resolution_to_string(ConflictResolution resolution) {
    switch (resolution) {
        case ConflictResolution::LastWriterWins: return "LastWriterWins";
        case ConflictResolution::FirstWriterWins: return "FirstWriterWins";
        case ConflictResolution::Merge: return "Merge";
        case ConflictResolution::Rollback: return "Rollback";
        case ConflictResolution::Custom: return "Custom";
        default: return "Unknown";
    }
}

// ============================================================================
// Snapshot Type
// ============================================================================

/**
 * @brief Type of snapshot
 */
enum class SnapshotType : UInt8 {
    /// Complete state snapshot
    Full,

    /// Incremental changes since last full snapshot
    Incremental,

    /// Differential changes since baseline
    Differential
};

/**
 * @brief Convert SnapshotType to string
 */
inline const char* snapshot_type_to_string(SnapshotType type) {
    switch (type) {
        case SnapshotType::Full: return "Full";
        case SnapshotType::Incremental: return "Incremental";
        case SnapshotType::Differential: return "Differential";
        default: return "Unknown";
    }
}

// ============================================================================
// Type Aliases
// ============================================================================

using PartitionId = UInt64;
using SnapshotId = UInt64;
using VersionNumber = UInt64;
using WallClockTime = std::chrono::steady_clock::time_point;
using Duration = std::chrono::nanoseconds;

constexpr PartitionId INVALID_PARTITION_ID = std::numeric_limits<PartitionId>::max();
constexpr SnapshotId INVALID_SNAPSHOT_ID = std::numeric_limits<SnapshotId>::max();

// ============================================================================
// Structs
// ============================================================================

/**
 * @brief Version information for state
 */
struct StateVersion {
    VersionNumber version{0};               ///< Monotonic version number
    VectorClock vector_clock;               ///< Vector clock for causality
    LogicalTime logical_time{0};            ///< Lamport logical timestamp
    SimulationTime sim_time{0.0};           ///< Simulation time of this version
    WallClockTime wall_time;                ///< Wall clock time when created
    NodeId author_node;                     ///< Node that created this version
    std::string description;                ///< Optional description

    /**
     * @brief Check if this version happens-before another
     */
    bool happens_before(const StateVersion& other) const {
        return vector_clock.happens_before(other.vector_clock);
    }

    /**
     * @brief Check if versions are concurrent
     */
    bool concurrent_with(const StateVersion& other) const {
        return vector_clock.concurrent_with(other.vector_clock);
    }
};

/**
 * @brief Delta representing changes since last sync
 */
struct StateDelta {
    EntityId entity_id{INVALID_ENTITY_ID}; ///< Entity this delta applies to
    StateVersion base_version;              ///< Version this delta is based on
    StateVersion new_version;               ///< Version after applying delta
    std::vector<UInt8> delta_data;          ///< Serialized delta data
    UInt64 uncompressed_size{0};            ///< Size before compression
    UInt64 compressed_size{0};              ///< Size after compression
    bool is_compressed{false};              ///< Whether data is compressed
    std::unordered_set<PropertyId> changed_properties; ///< Properties that changed
    WallClockTime created_at;               ///< When delta was created

    /**
     * @brief Get compression ratio
     */
    Real compression_ratio() const noexcept {
        if (uncompressed_size == 0) return 1.0;
        return static_cast<Real>(compressed_size) / uncompressed_size;
    }
};

/**
 * @brief Complete state snapshot
 */
struct StateSnapshot {
    SnapshotId snapshot_id{INVALID_SNAPSHOT_ID}; ///< Unique snapshot identifier
    SnapshotType type{SnapshotType::Full};        ///< Snapshot type
    StateVersion version;                         ///< Version information
    std::vector<EntityId> entities;               ///< Entities in this snapshot
    std::vector<UInt8> snapshot_data;             ///< Serialized snapshot data
    UInt64 uncompressed_size{0};                  ///< Size before compression
    UInt64 compressed_size{0};                    ///< Size after compression
    bool is_compressed{false};                    ///< Whether data is compressed
    WallClockTime created_at;                     ///< When snapshot was created
    PartitionId partition_id{INVALID_PARTITION_ID}; ///< Partition this snapshot covers
    SnapshotId baseline_snapshot{INVALID_SNAPSHOT_ID}; ///< For incremental/differential
    std::string checksum;                         ///< Data integrity checksum
    bool is_verified{false};                      ///< Checksum verified

    /**
     * @brief Get compression ratio
     */
    Real compression_ratio() const noexcept {
        if (uncompressed_size == 0) return 1.0;
        return static_cast<Real>(compressed_size) / uncompressed_size;
    }
};

/**
 * @brief Consistency checkpoint
 */
struct SyncCheckpoint {
    UInt64 checkpoint_id{0};                ///< Unique checkpoint identifier
    SimulationTime sim_time{0.0};           ///< Simulation time of checkpoint
    WallClockTime wall_time;                ///< Wall clock time when created
    std::unordered_map<PartitionId, SnapshotId> partition_snapshots; ///< Snapshots per partition
    std::unordered_map<NodeId, StateVersion> node_versions; ///< Version per node
    std::vector<EntityId> entities;         ///< All entities at checkpoint
    bool is_consistent{false};              ///< Whether globally consistent
    bool is_verified{false};                ///< Whether verified
    std::string description;                ///< Optional description
};

/**
 * @brief Information about detected conflict
 */
struct ConflictInfo {
    EntityId entity_id{INVALID_ENTITY_ID}; ///< Conflicting entity
    PartitionId partition_id{INVALID_PARTITION_ID}; ///< Partition with conflict
    StateVersion version_a;                 ///< First conflicting version
    StateVersion version_b;                 ///< Second conflicting version
    NodeId node_a;                          ///< First node involved
    NodeId node_b;                          ///< Second node involved
    std::unordered_set<PropertyId> conflicting_properties; ///< Properties in conflict
    ConflictResolution resolution{ConflictResolution::LastWriterWins}; ///< Chosen resolution
    bool is_resolved{false};                ///< Whether conflict is resolved
    WallClockTime detected_at;              ///< When conflict was detected
    WallClockTime resolved_at;              ///< When conflict was resolved
    std::string resolution_reason;          ///< Why this resolution was chosen
};

/**
 * @brief Serialized entity state for synchronization
 */
struct EntityState {
    EntityId entity_id{INVALID_ENTITY_ID}; ///< Entity identifier
    StateVersion version;                   ///< Current version
    Vec3 position;                          ///< Position (ECEF)
    Vec3 velocity;                          ///< Velocity
    Vec3 acceleration;                      ///< Acceleration
    Quat orientation;                       ///< Orientation quaternion
    Vec3 angular_velocity;                  ///< Angular velocity
    Vec3 angular_acceleration;              ///< Angular acceleration
    Real mass{0.0};                         ///< Mass (kg)
    Mat3x3 inertia_tensor;                  ///< Inertia tensor
    Domain domain{Domain::Generic};         ///< Entity domain
    std::unordered_map<PropertyId, Real> properties; ///< Dynamic properties
    std::vector<UInt8> custom_data;         ///< Custom component data
    bool is_active{true};                   ///< Is entity active
    PartitionId partition_id{INVALID_PARTITION_ID}; ///< Current partition
    NodeId owner_node;                      ///< Owning node
};

/**
 * @brief Configuration for state synchronization
 */
struct StateSyncConfig {
    // Synchronization settings
    SyncStrategy sync_strategy{SyncStrategy::Delta}; ///< Primary sync strategy
    ConflictResolution conflict_resolution{ConflictResolution::LastWriterWins};
    Duration sync_interval{std::chrono::milliseconds(100)}; ///< How often to sync
    bool auto_sync{true};                   ///< Enable automatic synchronization

    // Delta settings
    bool enable_delta_compression{true};    ///< Compress delta data
    UInt32 max_delta_size_bytes{1024 * 1024}; ///< Max delta size (1MB)
    UInt32 delta_merge_threshold{10};       ///< Merge into full after N deltas
    bool track_property_changes{true};      ///< Track individual property changes

    // Snapshot settings
    Duration snapshot_interval{std::chrono::seconds(10)}; ///< Full snapshot interval
    UInt32 max_snapshots_per_partition{10}; ///< Max snapshots to keep
    bool enable_snapshot_compression{true}; ///< Compress snapshot data
    bool verify_checksums{true};            ///< Verify snapshot checksums
    bool auto_cleanup_old_snapshots{true};  ///< Delete old snapshots

    // Consistency settings
    bool enable_vector_clocks{true};        ///< Use vector clocks for causality
    bool enable_consistency_checks{true};   ///< Verify consistency
    Duration consistency_check_interval{std::chrono::seconds(5)};
    bool require_ack{true};                 ///< Require acknowledgment for sync

    // Failover settings
    Duration failover_timeout{std::chrono::seconds(30)}; ///< Max failover time
    bool enable_automatic_failover{true};   ///< Auto-failover on node failure
    UInt32 failover_retry_attempts{3};      ///< Retry attempts for failover

    // Migration settings
    bool sync_during_migration{true};       ///< Sync state during entity migration
    Duration migration_sync_timeout{std::chrono::seconds(5)};
    bool verify_migration_state{true};      ///< Verify state after migration

    // Network settings
    Duration network_timeout{std::chrono::seconds(5)};
    UInt32 max_retry_attempts{3};
    Duration retry_backoff{std::chrono::milliseconds(100)};
    UInt32 max_concurrent_syncs{100};       ///< Max parallel sync operations

    // Performance settings
    UInt32 max_entities_per_sync{1000};     ///< Max entities per sync batch
    UInt64 max_sync_data_bytes{10 * 1024 * 1024}; ///< Max sync data (10MB)
    bool enable_async_sync{true};           ///< Use async synchronization
    UInt32 worker_thread_count{4};          ///< Threads for async sync

    /**
     * @brief Create default configuration
     */
    static StateSyncConfig default_config() noexcept {
        return StateSyncConfig{};
    }

    /**
     * @brief Create configuration for low-latency sync
     */
    static StateSyncConfig low_latency() noexcept {
        StateSyncConfig config;
        config.sync_interval = std::chrono::milliseconds(50);
        config.snapshot_interval = std::chrono::seconds(5);
        config.consistency_check_interval = std::chrono::seconds(2);
        config.max_entities_per_sync = 500;
        return config;
    }

    /**
     * @brief Create configuration for high-consistency sync
     */
    static StateSyncConfig high_consistency() noexcept {
        StateSyncConfig config;
        config.enable_vector_clocks = true;
        config.enable_consistency_checks = true;
        config.verify_checksums = true;
        config.verify_migration_state = true;
        config.require_ack = true;
        config.conflict_resolution = ConflictResolution::Merge;
        return config;
    }

    /**
     * @brief Create configuration for large-scale simulations
     */
    static StateSyncConfig large_scale() noexcept {
        StateSyncConfig config;
        config.enable_delta_compression = true;
        config.enable_snapshot_compression = true;
        config.max_entities_per_sync = 5000;
        config.max_sync_data_bytes = 50 * 1024 * 1024; // 50MB
        config.worker_thread_count = 8;
        config.auto_cleanup_old_snapshots = true;
        return config;
    }
};

/**
 * @brief Statistics for state synchronization
 */
struct StateSyncStats {
    // Sync operations
    UInt64 total_syncs{0};                  ///< Total sync operations
    UInt64 successful_syncs{0};             ///< Successful syncs
    UInt64 failed_syncs{0};                 ///< Failed syncs
    UInt64 full_syncs{0};                   ///< Full state syncs
    UInt64 delta_syncs{0};                  ///< Delta syncs
    Duration avg_sync_time{0};              ///< Average sync duration
    Duration max_sync_time{0};              ///< Maximum sync duration

    // Data transfer
    UInt64 total_bytes_sent{0};             ///< Total bytes sent
    UInt64 total_bytes_received{0};         ///< Total bytes received
    UInt64 compressed_bytes_sent{0};        ///< Compressed bytes sent
    Real avg_compression_ratio{1.0};        ///< Average compression ratio

    // Conflicts
    UInt64 conflicts_detected{0};           ///< Total conflicts detected
    UInt64 conflicts_resolved{0};           ///< Conflicts resolved
    UInt64 rollbacks_performed{0};          ///< Rollbacks performed
    UInt64 rollback_failures{0};            ///< Failed rollbacks

    // Snapshots
    UInt64 snapshots_created{0};            ///< Total snapshots created
    UInt64 snapshots_restored{0};           ///< Snapshots restored
    UInt64 snapshot_failures{0};            ///< Failed snapshot operations
    UInt64 total_snapshot_bytes{0};         ///< Total snapshot storage

    // Consistency
    UInt64 consistency_checks{0};           ///< Consistency checks performed
    UInt64 consistency_violations{0};       ///< Detected violations
    UInt64 causality_violations{0};         ///< Causality violations

    // Failover
    UInt64 failovers_initiated{0};          ///< Failover operations
    UInt64 successful_failovers{0};         ///< Successful failovers
    Duration avg_failover_time{0};          ///< Average failover duration
    Duration max_failover_time{0};          ///< Maximum failover duration

    // Migration
    UInt64 migration_syncs{0};              ///< Syncs during migration
    UInt64 migration_sync_failures{0};      ///< Failed migration syncs
    Duration avg_migration_sync_time{0};    ///< Average migration sync time

    // Errors
    UInt64 network_errors{0};               ///< Network-related errors
    UInt64 timeout_errors{0};               ///< Timeout errors
    UInt64 serialization_errors{0};         ///< Serialization errors
    UInt64 corruption_errors{0};            ///< Data corruption errors

    /**
     * @brief Reset all statistics
     */
    void reset() noexcept {
        *this = StateSyncStats{};
    }
};

// ============================================================================
// Interfaces
// ============================================================================

/**
 * @brief Interface for serializing/deserializing entity state
 */
class IStateSerializer {
public:
    virtual ~IStateSerializer() = default;

    /**
     * @brief Serialize entity state
     * @param state Entity state to serialize
     * @param output Output buffer
     * @return Success or error code
     */
    virtual SyncResult serialize(const EntityState& state,
                                  std::vector<UInt8>& output) = 0;

    /**
     * @brief Deserialize entity state
     * @param data Input data
     * @param state Output state
     * @return Success or error code
     */
    virtual SyncResult deserialize(const std::vector<UInt8>& data,
                                    EntityState& state) = 0;

    /**
     * @brief Serialize state delta
     * @param old_state Previous state
     * @param new_state New state
     * @param delta Output delta
     * @return Success or error code
     */
    virtual SyncResult serialize_delta(const EntityState& old_state,
                                        const EntityState& new_state,
                                        StateDelta& delta) = 0;

    /**
     * @brief Apply delta to state
     * @param base_state Base state
     * @param delta Delta to apply
     * @param new_state Output state
     * @return Success or error code
     */
    virtual SyncResult apply_delta(const EntityState& base_state,
                                    const StateDelta& delta,
                                    EntityState& new_state) = 0;

    /**
     * @brief Compress data
     */
    virtual std::vector<UInt8> compress(const std::vector<UInt8>& data) = 0;

    /**
     * @brief Decompress data
     */
    virtual std::vector<UInt8> decompress(const std::vector<UInt8>& data) = 0;

    /**
     * @brief Calculate checksum
     */
    virtual std::string calculate_checksum(const std::vector<UInt8>& data) = 0;

    /**
     * @brief Verify checksum
     */
    virtual bool verify_checksum(const std::vector<UInt8>& data,
                                  const std::string& checksum) = 0;
};

/**
 * @brief Interface for conflict resolution
 */
class IConflictResolver {
public:
    virtual ~IConflictResolver() = default;

    /**
     * @brief Detect conflict between two states
     * @param state_a First state
     * @param state_b Second state
     * @return Conflict info if conflict detected
     */
    virtual std::optional<ConflictInfo> detect_conflict(
        const EntityState& state_a,
        const EntityState& state_b) = 0;

    /**
     * @brief Resolve conflict between states
     * @param conflict Conflict information
     * @param state_a First state
     * @param state_b Second state
     * @param resolved_state Output resolved state
     * @return Success or error code
     */
    virtual SyncResult resolve_conflict(const ConflictInfo& conflict,
                                         const EntityState& state_a,
                                         const EntityState& state_b,
                                         EntityState& resolved_state) = 0;

    /**
     * @brief Check if rollback is required
     */
    virtual bool requires_rollback(const ConflictInfo& conflict) const = 0;

    /**
     * @brief Set conflict resolution strategy
     */
    virtual void set_strategy(ConflictResolution strategy) = 0;

    /**
     * @brief Get current strategy
     */
    virtual ConflictResolution get_strategy() const = 0;
};

/**
 * @brief Interface for snapshot management
 */
class ISnapshotManager {
public:
    virtual ~ISnapshotManager() = default;

    /**
     * @brief Create full snapshot
     * @param entities Entities to snapshot
     * @param snapshot Output snapshot
     * @return Success or error code
     */
    virtual SyncResult create_full_snapshot(
        const std::vector<EntityState>& entities,
        StateSnapshot& snapshot) = 0;

    /**
     * @brief Create incremental snapshot
     * @param entities Current entities
     * @param baseline_id Baseline snapshot
     * @param snapshot Output snapshot
     * @return Success or error code
     */
    virtual SyncResult create_incremental_snapshot(
        const std::vector<EntityState>& entities,
        SnapshotId baseline_id,
        StateSnapshot& snapshot) = 0;

    /**
     * @brief Restore snapshot
     * @param snapshot Snapshot to restore
     * @param entities Output entities
     * @return Success or error code
     */
    virtual SyncResult restore_snapshot(const StateSnapshot& snapshot,
                                         std::vector<EntityState>& entities) = 0;

    /**
     * @brief Store snapshot
     */
    virtual SyncResult store_snapshot(const StateSnapshot& snapshot) = 0;

    /**
     * @brief Load snapshot
     */
    virtual std::optional<StateSnapshot> load_snapshot(SnapshotId snapshot_id) = 0;

    /**
     * @brief Delete snapshot
     */
    virtual SyncResult delete_snapshot(SnapshotId snapshot_id) = 0;

    /**
     * @brief Get all snapshots for partition
     */
    virtual std::vector<SnapshotId> get_snapshots(PartitionId partition_id) = 0;

    /**
     * @brief Cleanup old snapshots
     */
    virtual SyncResult cleanup_old_snapshots(PartitionId partition_id,
                                              UInt32 keep_count) = 0;

    /**
     * @brief Verify snapshot integrity
     */
    virtual bool verify_snapshot(const StateSnapshot& snapshot) = 0;

    /**
     * @brief Get snapshot storage size
     */
    virtual UInt64 get_storage_size() const = 0;
};

/**
 * @brief Interface for network transport
 */
class ISyncTransport {
public:
    virtual ~ISyncTransport() = default;

    /**
     * @brief Send state to node
     */
    virtual SyncResult send_state(const NodeId& target_node,
                                   const EntityState& state) = 0;

    /**
     * @brief Send delta to node
     */
    virtual SyncResult send_delta(const NodeId& target_node,
                                   const StateDelta& delta) = 0;

    /**
     * @brief Send snapshot to node
     */
    virtual SyncResult send_snapshot(const NodeId& target_node,
                                      const StateSnapshot& snapshot) = 0;

    /**
     * @brief Request state from node
     */
    virtual std::optional<EntityState> request_state(
        const NodeId& source_node,
        EntityId entity_id,
        Duration timeout) = 0;

    /**
     * @brief Broadcast state to all nodes
     */
    virtual SyncResult broadcast_state(const EntityState& state) = 0;

    /**
     * @brief Check if node is reachable
     */
    virtual bool is_node_reachable(const NodeId& node_id) const = 0;

    /**
     * @brief Get network latency to node
     */
    virtual Duration get_latency(const NodeId& node_id) const = 0;
};

// ============================================================================
// Callback Types
// ============================================================================

using SyncCompleteCallback = std::function<void(EntityId, SyncResult)>;
using ConflictCallback = std::function<void(const ConflictInfo&)>;
using SnapshotCallback = std::function<void(SnapshotId, SyncResult)>;
using FailoverCallback = std::function<void(const NodeId&, bool success)>;

// ============================================================================
// Main State Synchronizer
// ============================================================================

/**
 * @brief Central manager for state synchronization
 *
 * Coordinates delta-based synchronization, snapshot management, conflict
 * resolution, and failover handling across distributed simulation nodes.
 *
 * Example usage:
 * @code
 * StateSyncConfig config = StateSyncConfig::default_config();
 * config.sync_strategy = SyncStrategy::Delta;
 * config.conflict_resolution = ConflictResolution::Merge;
 *
 * auto synchronizer = create_state_synchronizer(config);
 * synchronizer->initialize(partition_manager, time_manager);
 *
 * // Sync entity
 * EntityState state = get_entity_state(entity_id);
 * synchronizer->sync_entity(entity_id, state);
 *
 * // Create snapshot
 * auto snapshot_id = synchronizer->create_snapshot(partition_id);
 * @endcode
 */
class StateSynchronizer {
public:
    virtual ~StateSynchronizer() = default;

    // ========================================================================
    // Lifecycle
    // ========================================================================

    /**
     * @brief Initialize the state synchronizer
     * @param partition_manager Partition manager instance
     * @param time_manager Distributed time manager
     * @return Success or error code
     */
    virtual SyncResult initialize(void* partition_manager,
                                   void* time_manager) = 0;

    /**
     * @brief Shutdown the synchronizer
     * @return Success or error code
     */
    virtual SyncResult shutdown() = 0;

    /**
     * @brief Check if initialized
     */
    virtual bool is_initialized() const = 0;

    // ========================================================================
    // Entity Synchronization
    // ========================================================================

    /**
     * @brief Synchronize entity state
     * @param entity_id Entity to synchronize
     * @param state Current entity state
     * @return Success or error code
     */
    virtual SyncResult sync_entity(EntityId entity_id,
                                    const EntityState& state) = 0;

    /**
     * @brief Synchronize multiple entities
     */
    virtual SyncResult sync_entities(const std::vector<EntityId>& entity_ids,
                                      const std::vector<EntityState>& states) = 0;

    /**
     * @brief Request entity state from owner node
     */
    virtual std::optional<EntityState> request_entity_state(
        EntityId entity_id,
        const NodeId& owner_node) = 0;

    /**
     * @brief Get latest local entity state
     */
    virtual std::optional<EntityState> get_local_state(EntityId entity_id) const = 0;

    /**
     * @brief Update entity state version
     */
    virtual SyncResult update_version(EntityId entity_id,
                                       const StateVersion& version) = 0;

    // ========================================================================
    // Partition Synchronization
    // ========================================================================

    /**
     * @brief Synchronize entire partition
     * @param partition_id Partition to synchronize
     * @return Success or error code
     */
    virtual SyncResult sync_partition(PartitionId partition_id) = 0;

    /**
     * @brief Get partition consistency status
     */
    virtual bool is_partition_consistent(PartitionId partition_id) const = 0;

    /**
     * @brief Force consistency check
     */
    virtual SyncResult check_consistency(PartitionId partition_id) = 0;

    // ========================================================================
    // Snapshot Management
    // ========================================================================

    /**
     * @brief Create snapshot of partition
     * @param partition_id Partition to snapshot
     * @param type Snapshot type
     * @return Snapshot ID or INVALID_SNAPSHOT_ID on failure
     */
    virtual SnapshotId create_snapshot(PartitionId partition_id,
                                        SnapshotType type = SnapshotType::Full) = 0;

    /**
     * @brief Restore partition from snapshot
     * @param partition_id Partition to restore
     * @param snapshot_id Snapshot to restore from
     * @return Success or error code
     */
    virtual SyncResult restore_snapshot(PartitionId partition_id,
                                         SnapshotId snapshot_id) = 0;

    /**
     * @brief Get snapshot information
     */
    virtual std::optional<StateSnapshot> get_snapshot_info(
        SnapshotId snapshot_id) const = 0;

    /**
     * @brief List all snapshots for partition
     */
    virtual std::vector<SnapshotId> list_snapshots(
        PartitionId partition_id) const = 0;

    /**
     * @brief Delete old snapshots
     */
    virtual SyncResult cleanup_snapshots(PartitionId partition_id) = 0;

    // ========================================================================
    // Conflict Resolution
    // ========================================================================

    /**
     * @brief Handle detected conflict
     * @param conflict Conflict information
     * @return Success or error code
     */
    virtual SyncResult handle_conflict(const ConflictInfo& conflict) = 0;

    /**
     * @brief Get pending conflicts
     */
    virtual std::vector<ConflictInfo> get_pending_conflicts() const = 0;

    /**
     * @brief Rollback entity to previous state
     */
    virtual SyncResult rollback_entity(EntityId entity_id,
                                        const StateVersion& target_version) = 0;

    /**
     * @brief Set conflict resolution strategy
     */
    virtual void set_conflict_resolution(ConflictResolution strategy) = 0;

    // ========================================================================
    // Failover
    // ========================================================================

    /**
     * @brief Handle node failure
     * @param failed_node Node that failed
     * @return Success or error code
     */
    virtual SyncResult handle_node_failure(const NodeId& failed_node) = 0;

    /**
     * @brief Initiate failover to target node
     */
    virtual SyncResult initiate_failover(const NodeId& failed_node,
                                          const NodeId& target_node) = 0;

    /**
     * @brief Check if failover is in progress
     */
    virtual bool is_failover_in_progress() const = 0;

    /**
     * @brief Get failover status
     */
    virtual Real get_failover_progress() const = 0;

    // ========================================================================
    // Migration Support
    // ========================================================================

    /**
     * @brief Prepare entity for migration
     */
    virtual SyncResult prepare_migration(EntityId entity_id,
                                          const NodeId& target_node) = 0;

    /**
     * @brief Complete entity migration
     */
    virtual SyncResult complete_migration(EntityId entity_id,
                                           const NodeId& source_node) = 0;

    /**
     * @brief Cancel migration
     */
    virtual SyncResult cancel_migration(EntityId entity_id) = 0;

    // ========================================================================
    // Checkpoints
    // ========================================================================

    /**
     * @brief Create consistency checkpoint
     */
    virtual UInt64 create_checkpoint() = 0;

    /**
     * @brief Restore from checkpoint
     */
    virtual SyncResult restore_checkpoint(UInt64 checkpoint_id) = 0;

    /**
     * @brief Get checkpoint information
     */
    virtual std::optional<SyncCheckpoint> get_checkpoint_info(
        UInt64 checkpoint_id) const = 0;

    // ========================================================================
    // Update
    // ========================================================================

    /**
     * @brief Update synchronizer (call each simulation step)
     * @param dt Time step in seconds
     */
    virtual void update(Real dt) = 0;

    // ========================================================================
    // Statistics
    // ========================================================================

    /**
     * @brief Get synchronization statistics
     */
    virtual StateSyncStats get_statistics() const = 0;

    /**
     * @brief Reset statistics
     */
    virtual void reset_statistics() = 0;

    // ========================================================================
    // Configuration
    // ========================================================================

    /**
     * @brief Get current configuration
     */
    virtual const StateSyncConfig& get_config() const = 0;

    /**
     * @brief Update sync interval
     */
    virtual SyncResult set_sync_interval(Duration interval) = 0;

    /**
     * @brief Update snapshot interval
     */
    virtual SyncResult set_snapshot_interval(Duration interval) = 0;

    /**
     * @brief Enable/disable auto sync
     */
    virtual void set_auto_sync(bool enabled) = 0;

    // ========================================================================
    // Callbacks
    // ========================================================================

    /**
     * @brief Set callback for sync completion
     */
    virtual void set_sync_callback(SyncCompleteCallback callback) = 0;

    /**
     * @brief Set callback for conflicts
     */
    virtual void set_conflict_callback(ConflictCallback callback) = 0;

    /**
     * @brief Set callback for snapshots
     */
    virtual void set_snapshot_callback(SnapshotCallback callback) = 0;

    /**
     * @brief Set callback for failover
     */
    virtual void set_failover_callback(FailoverCallback callback) = 0;
};

// ============================================================================
// Factory Functions
// ============================================================================

/**
 * @brief Create state synchronizer
 * @param config Configuration for state sync
 * @return Unique pointer to the synchronizer
 */
std::unique_ptr<StateSynchronizer> create_state_synchronizer(
    const StateSyncConfig& config);

/**
 * @brief Create snapshot manager
 * @param config Configuration
 * @return Unique pointer to the snapshot manager
 */
std::unique_ptr<ISnapshotManager> create_snapshot_manager(
    const StateSyncConfig& config);

/**
 * @brief Create conflict resolver
 * @param strategy Conflict resolution strategy
 * @return Unique pointer to the conflict resolver
 */
std::unique_ptr<IConflictResolver> create_conflict_resolver(
    ConflictResolution strategy);

/**
 * @brief Create state serializer
 * @return Unique pointer to the serializer
 */
std::unique_ptr<IStateSerializer> create_state_serializer();

/**
 * @brief Create sync transport
 * @return Unique pointer to the transport
 */
std::unique_ptr<ISyncTransport> create_sync_transport();

} // namespace jaguar::cloud
