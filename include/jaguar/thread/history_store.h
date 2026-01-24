#pragma once
/**
 * @file history_store.h
 * @brief Entity history store for time-series state snapshots
 *
 * This file provides infrastructure for capturing, storing, querying, and exporting
 * entity state history over time. Enables time-series analysis, replay, debugging,
 * and data export for offline processing.
 *
 * Key features:
 * - Time-series state snapshots (full, delta, checkpoint)
 * - Configurable retention policies with auto-cleanup
 * - Advanced temporal queries with filtering
 * - Pluggable storage backends (memory, disk, database)
 * - Multi-format export (Parquet, CSV, JSON, Binary)
 * - Efficient indexing and compression
 * - Integrity verification with state hashing
 */

#include "jaguar/core/types.h"
#include <vector>
#include <unordered_map>
#include <memory>
#include <optional>
#include <chrono>
#include <string>

namespace jaguar::thread {

// ============================================================================
// Forward Declarations
// ============================================================================

class IHistoryBackend;
class IHistoryExporter;
class HistoryStore;

// ============================================================================
// History Result Enum
// ============================================================================

/**
 * @brief Result codes for history store operations
 */
enum class HistoryResult : UInt8 {
    Success = 0,

    // Configuration errors
    InvalidConfiguration,
    InvalidEntityId,
    InvalidTimeRange,
    InvalidQuery,

    // Operational errors
    StoreFailed,
    RetrieveFailed,
    DeleteFailed,
    ExportFailed,

    // State errors
    NotInitialized,
    AlreadyInitialized,
    EntityNotFound,
    SnapshotNotFound,

    // Resource errors
    OutOfMemory,
    StorageError,
    CapacityExceeded,

    // Export errors
    ExportNotSupported,
    ExportIOError
};

/**
 * @brief Convert HistoryResult to string
 */
inline const char* history_result_to_string(HistoryResult result) {
    switch (result) {
        case HistoryResult::Success: return "Success";
        case HistoryResult::InvalidConfiguration: return "InvalidConfiguration";
        case HistoryResult::InvalidEntityId: return "InvalidEntityId";
        case HistoryResult::InvalidTimeRange: return "InvalidTimeRange";
        case HistoryResult::InvalidQuery: return "InvalidQuery";
        case HistoryResult::StoreFailed: return "StoreFailed";
        case HistoryResult::RetrieveFailed: return "RetrieveFailed";
        case HistoryResult::DeleteFailed: return "DeleteFailed";
        case HistoryResult::ExportFailed: return "ExportFailed";
        case HistoryResult::NotInitialized: return "NotInitialized";
        case HistoryResult::AlreadyInitialized: return "AlreadyInitialized";
        case HistoryResult::EntityNotFound: return "EntityNotFound";
        case HistoryResult::SnapshotNotFound: return "SnapshotNotFound";
        case HistoryResult::OutOfMemory: return "OutOfMemory";
        case HistoryResult::StorageError: return "StorageError";
        case HistoryResult::CapacityExceeded: return "CapacityExceeded";
        case HistoryResult::ExportNotSupported: return "ExportNotSupported";
        case HistoryResult::ExportIOError: return "ExportIOError";
        default: return "Unknown";
    }
}

// ============================================================================
// Snapshot Type Enum
// ============================================================================

/**
 * @brief Type of state snapshot
 */
enum class SnapshotType : UInt8 {
    Full,       ///< Complete entity state
    Delta,      ///< Changes since last snapshot
    Checkpoint  ///< Periodic full snapshot for recovery
};

/**
 * @brief Convert SnapshotType to string
 */
inline const char* snapshot_type_to_string(SnapshotType type) {
    switch (type) {
        case SnapshotType::Full: return "Full";
        case SnapshotType::Delta: return "Delta";
        case SnapshotType::Checkpoint: return "Checkpoint";
        default: return "Unknown";
    }
}

// ============================================================================
// Export Format Enum
// ============================================================================

/**
 * @brief Format for history data export
 */
enum class ExportFormat : UInt8 {
    Parquet,    ///< Apache Parquet format (columnar)
    CSV,        ///< Comma-separated values
    JSON,       ///< JSON lines format (newline-delimited)
    Binary      ///< Raw binary format
};

/**
 * @brief Convert ExportFormat to string
 */
inline const char* export_format_to_string(ExportFormat format) {
    switch (format) {
        case ExportFormat::Parquet: return "Parquet";
        case ExportFormat::CSV: return "CSV";
        case ExportFormat::JSON: return "JSON";
        case ExportFormat::Binary: return "Binary";
        default: return "Unknown";
    }
}

// ============================================================================
// Structs
// ============================================================================

/**
 * @brief Time range for queries
 */
struct QueryTimeRange {
    std::chrono::system_clock::time_point start;    ///< Range start time
    std::chrono::system_clock::time_point end;      ///< Range end time
    bool inclusive_start{true};                     ///< Include start boundary
    bool inclusive_end{true};                       ///< Include end boundary

    /**
     * @brief Default constructor (empty range)
     */
    QueryTimeRange() = default;

    /**
     * @brief Construct time range
     */
    QueryTimeRange(std::chrono::system_clock::time_point start_,
                   std::chrono::system_clock::time_point end_,
                   bool inclusive_start_ = true,
                   bool inclusive_end_ = true)
        : start(start_)
        , end(end_)
        , inclusive_start(inclusive_start_)
        , inclusive_end(inclusive_end_) {}

    /**
     * @brief Check if time is within range
     */
    bool contains(std::chrono::system_clock::time_point time) const {
        bool after_start = inclusive_start ? (time >= start) : (time > start);
        bool before_end = inclusive_end ? (time <= end) : (time < end);
        return after_start && before_end;
    }

    /**
     * @brief Get duration of the range
     */
    std::chrono::nanoseconds duration() const {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    }
};

/**
 * @brief State snapshot for an entity at a specific time
 */
struct StateSnapshot {
    EntityId entity_id{INVALID_ENTITY_ID};          ///< Entity identifier
    UInt64 sequence_number{0};                      ///< Snapshot sequence number
    SnapshotType type{SnapshotType::Full};          ///< Snapshot type
    std::chrono::system_clock::time_point timestamp;///< When snapshot was captured
    std::vector<UInt8> state_data;                  ///< Serialized entity state
    UInt64 state_hash{0};                           ///< Hash for integrity verification
    std::string source_node;                        ///< Node that captured this snapshot

    /**
     * @brief Default constructor
     */
    StateSnapshot() = default;

    /**
     * @brief Construct snapshot
     */
    StateSnapshot(EntityId entity_id_,
                  UInt64 sequence_number_,
                  SnapshotType type_,
                  std::chrono::system_clock::time_point timestamp_,
                  std::vector<UInt8> state_data_,
                  UInt64 state_hash_,
                  std::string source_node_ = "")
        : entity_id(entity_id_)
        , sequence_number(sequence_number_)
        , type(type_)
        , timestamp(timestamp_)
        , state_data(std::move(state_data_))
        , state_hash(state_hash_)
        , source_node(std::move(source_node_)) {}

    /**
     * @brief Get size of snapshot in bytes
     */
    UInt64 size_bytes() const {
        return state_data.size();
    }

    /**
     * @brief Validate snapshot integrity
     */
    bool verify_integrity() const;
};

/**
 * @brief Query parameters for history retrieval
 */
struct HistoryQuery {
    std::optional<EntityId> entity_id;              ///< Filter by entity (nullopt = all)
    std::optional<QueryTimeRange> time_range;       ///< Filter by time range
    std::optional<SnapshotType> snapshot_type;      ///< Filter by snapshot type
    UInt32 limit{100};                              ///< Maximum results to return
    UInt32 offset{0};                               ///< Skip first N results
    bool ascending{false};                          ///< Sort order (false = newest first)

    /**
     * @brief Default constructor (query all)
     */
    HistoryQuery() = default;

    /**
     * @brief Query for specific entity
     */
    static HistoryQuery for_entity(EntityId id, UInt32 limit = 100) {
        HistoryQuery query;
        query.entity_id = id;
        query.limit = limit;
        return query;
    }

    /**
     * @brief Query for time range
     */
    static HistoryQuery for_time_range(QueryTimeRange range, UInt32 limit = 100) {
        HistoryQuery query;
        query.time_range = range;
        query.limit = limit;
        return query;
    }

    /**
     * @brief Query for specific snapshot type
     */
    static HistoryQuery for_type(SnapshotType type, UInt32 limit = 100) {
        HistoryQuery query;
        query.snapshot_type = type;
        query.limit = limit;
        return query;
    }
};

/**
 * @brief Result of a history query
 */
struct HistoryQueryResult {
    std::vector<StateSnapshot> snapshots;           ///< Retrieved snapshots
    UInt64 total_count{0};                          ///< Total matching snapshots
    bool has_more{false};                           ///< More results available

    /**
     * @brief Default constructor
     */
    HistoryQueryResult() = default;

    /**
     * @brief Construct query result
     */
    HistoryQueryResult(std::vector<StateSnapshot> snapshots_,
                       UInt64 total_count_,
                       bool has_more_)
        : snapshots(std::move(snapshots_))
        , total_count(total_count_)
        , has_more(has_more_) {}

    /**
     * @brief Get total size of result in bytes
     */
    UInt64 size_bytes() const {
        UInt64 total = 0;
        for (const auto& snapshot : snapshots) {
            total += snapshot.size_bytes();
        }
        return total;
    }

    /**
     * @brief Check if result is empty
     */
    bool empty() const {
        return snapshots.empty();
    }

    /**
     * @brief Get count of returned snapshots
     */
    UInt64 count() const {
        return snapshots.size();
    }
};

/**
 * @brief Retention policy for automatic cleanup
 */
struct RetentionPolicy {
    std::chrono::hours full_retention{720};         ///< Keep full snapshots for 30 days
    std::chrono::hours delta_retention{168};        ///< Keep deltas for 7 days
    std::chrono::hours checkpoint_retention{2160};  ///< Keep checkpoints for 90 days
    UInt64 max_snapshots_per_entity{10000};         ///< Max snapshots per entity
    bool auto_cleanup{true};                        ///< Enable automatic cleanup
    std::chrono::minutes cleanup_interval{60};      ///< Run cleanup every hour

    /**
     * @brief Create default retention policy
     */
    static RetentionPolicy default_policy() noexcept {
        return RetentionPolicy{};
    }

    /**
     * @brief Create aggressive retention (short-term storage)
     */
    static RetentionPolicy aggressive() noexcept {
        RetentionPolicy policy;
        policy.full_retention = std::chrono::hours(24);      // 1 day
        policy.delta_retention = std::chrono::hours(6);      // 6 hours
        policy.checkpoint_retention = std::chrono::hours(72); // 3 days
        policy.max_snapshots_per_entity = 1000;
        policy.cleanup_interval = std::chrono::minutes(15);
        return policy;
    }

    /**
     * @brief Create relaxed retention (long-term storage)
     */
    static RetentionPolicy relaxed() noexcept {
        RetentionPolicy policy;
        policy.full_retention = std::chrono::hours(2160);     // 90 days
        policy.delta_retention = std::chrono::hours(720);     // 30 days
        policy.checkpoint_retention = std::chrono::hours(8760); // 1 year
        policy.max_snapshots_per_entity = 100000;
        policy.cleanup_interval = std::chrono::minutes(120);
        return policy;
    }

    /**
     * @brief Check if snapshot should be retained
     */
    bool should_retain(const StateSnapshot& snapshot,
                       std::chrono::system_clock::time_point current_time) const {
        auto age = std::chrono::duration_cast<std::chrono::hours>(
            current_time - snapshot.timestamp);

        switch (snapshot.type) {
            case SnapshotType::Full:
                return age < full_retention;
            case SnapshotType::Delta:
                return age < delta_retention;
            case SnapshotType::Checkpoint:
                return age < checkpoint_retention;
            default:
                return false;
        }
    }
};

/**
 * @brief Configuration for history store
 */
struct HistoryStoreConfig {
    RetentionPolicy retention;                      ///< Retention policy
    UInt64 max_total_snapshots{10000000};           ///< Max total snapshots (10M)
    UInt64 max_memory_bytes{1073741824};            ///< Max memory usage (1GB)
    bool enable_compression{true};                  ///< Compress snapshot data
    bool enable_indexing{true};                     ///< Build indexes for fast queries
    std::string storage_path;                       ///< Path for persistent storage

    /**
     * @brief Create default configuration
     */
    static HistoryStoreConfig default_config() noexcept {
        return HistoryStoreConfig{};
    }

    /**
     * @brief Create in-memory configuration (no persistence)
     */
    static HistoryStoreConfig in_memory() noexcept {
        HistoryStoreConfig config;
        config.max_total_snapshots = 1000000;  // 1M
        config.max_memory_bytes = 536870912;   // 512MB
        config.storage_path = "";  // No persistence
        config.retention = RetentionPolicy::aggressive();
        return config;
    }

    /**
     * @brief Create persistent configuration (disk-backed)
     */
    static HistoryStoreConfig persistent(const std::string& path) noexcept {
        HistoryStoreConfig config;
        config.max_total_snapshots = 100000000;  // 100M
        config.max_memory_bytes = 2147483648;    // 2GB
        config.storage_path = path;
        config.retention = RetentionPolicy::relaxed();
        return config;
    }

    /**
     * @brief Create configuration for high-frequency capture
     */
    static HistoryStoreConfig high_frequency() noexcept {
        HistoryStoreConfig config;
        config.max_total_snapshots = 50000000;  // 50M
        config.max_memory_bytes = 4294967296;   // 4GB
        config.enable_compression = true;
        config.enable_indexing = true;
        config.retention = RetentionPolicy::default_policy();
        return config;
    }
};

/**
 * @brief Configuration for data export
 */
struct ExportConfig {
    ExportFormat format{ExportFormat::Parquet};     ///< Export format
    std::string output_path;                        ///< Output file path
    HistoryQuery query;                             ///< What to export
    bool compress{true};                            ///< Compress output
    UInt32 batch_size{10000};                       ///< Records per batch

    /**
     * @brief Default constructor
     */
    ExportConfig() = default;

    /**
     * @brief Construct export config
     */
    ExportConfig(ExportFormat format_,
                 std::string output_path_,
                 HistoryQuery query_ = HistoryQuery{})
        : format(format_)
        , output_path(std::move(output_path_))
        , query(std::move(query_)) {}

    /**
     * @brief Create Parquet export config
     */
    static ExportConfig to_parquet(const std::string& path, const HistoryQuery& query = {}) {
        return ExportConfig(ExportFormat::Parquet, path, query);
    }

    /**
     * @brief Create CSV export config
     */
    static ExportConfig to_csv(const std::string& path, const HistoryQuery& query = {}) {
        return ExportConfig(ExportFormat::CSV, path, query);
    }

    /**
     * @brief Create JSON export config
     */
    static ExportConfig to_json(const std::string& path, const HistoryQuery& query = {}) {
        return ExportConfig(ExportFormat::JSON, path, query);
    }
};

/**
 * @brief Statistics for history store
 */
struct HistoryStoreStats {
    UInt64 total_snapshots{0};                      ///< Total snapshot count
    UInt64 total_entities{0};                       ///< Total unique entities
    UInt64 storage_bytes{0};                        ///< Total storage used
    UInt64 oldest_snapshot_age_hours{0};            ///< Age of oldest snapshot
    std::unordered_map<SnapshotType, UInt64> snapshots_by_type; ///< Breakdown by type

    /**
     * @brief Default constructor
     */
    HistoryStoreStats() = default;

    /**
     * @brief Get snapshot count for a type
     */
    UInt64 count_for_type(SnapshotType type) const {
        auto it = snapshots_by_type.find(type);
        return it != snapshots_by_type.end() ? it->second : 0;
    }

    /**
     * @brief Get average snapshot size
     */
    UInt64 average_snapshot_size() const {
        return total_snapshots > 0 ? (storage_bytes / total_snapshots) : 0;
    }

    /**
     * @brief Check if nearing capacity
     */
    bool is_near_capacity(const HistoryStoreConfig& config) const {
        return (total_snapshots >= config.max_total_snapshots * 0.9) ||
               (storage_bytes >= config.max_memory_bytes * 0.9);
    }
};

// ============================================================================
// Interfaces
// ============================================================================

/**
 * @brief Interface for history storage backend
 *
 * Allows pluggable storage implementations (in-memory, disk, database, etc.)
 */
class IHistoryBackend {
public:
    virtual ~IHistoryBackend() = default;

    /**
     * @brief Store a snapshot
     * @param snapshot Snapshot to store
     * @return Success or error code
     */
    virtual HistoryResult store(const StateSnapshot& snapshot) = 0;

    /**
     * @brief Retrieve a snapshot by entity and sequence
     * @param entity_id Entity identifier
     * @param sequence Sequence number
     * @return Snapshot if found, nullopt otherwise
     */
    virtual std::optional<StateSnapshot> get(EntityId entity_id, UInt64 sequence) const = 0;

    /**
     * @brief Query snapshots
     * @param query Query parameters
     * @return Query result
     */
    virtual HistoryQueryResult query(const HistoryQuery& query) const = 0;

    /**
     * @brief Remove a specific snapshot
     * @param entity_id Entity identifier
     * @param sequence Sequence number
     * @return Success or error code
     */
    virtual HistoryResult remove(EntityId entity_id, UInt64 sequence) = 0;

    /**
     * @brief Remove all snapshots before a time
     * @param cutoff_time Remove snapshots before this time
     * @return Success or error code
     */
    virtual HistoryResult remove_before(std::chrono::system_clock::time_point cutoff_time) = 0;

    /**
     * @brief Get total snapshot count
     */
    virtual UInt64 count() const = 0;

    /**
     * @brief Get total storage size in bytes
     */
    virtual UInt64 size_bytes() const = 0;
};

/**
 * @brief Interface for history data export
 *
 * Allows pluggable export implementations for different formats
 */
class IHistoryExporter {
public:
    virtual ~IHistoryExporter() = default;

    /**
     * @brief Export history data
     * @param config Export configuration
     * @return Success or error code
     */
    virtual HistoryResult export_data(const ExportConfig& config) = 0;

    /**
     * @brief Check if format is supported
     * @param format Format to check
     * @return True if supported
     */
    virtual bool supports_format(ExportFormat format) const = 0;
};

// ============================================================================
// Main History Store Class
// ============================================================================

/**
 * @brief Central store for entity state history
 *
 * Manages time-series snapshots of entity state with configurable retention,
 * efficient querying, and multi-format export capabilities.
 *
 * Example usage:
 * @code
 * HistoryStoreConfig config = HistoryStoreConfig::default_config();
 * auto store = create_history_store(config);
 * store->initialize();
 *
 * // Store a snapshot
 * StateSnapshot snapshot;
 * snapshot.entity_id = 123;
 * snapshot.sequence_number = 1;
 * snapshot.type = SnapshotType::Full;
 * snapshot.timestamp = std::chrono::system_clock::now();
 * snapshot.state_data = serialize_entity_state(entity);
 * snapshot.state_hash = compute_hash(snapshot.state_data);
 * store->store_snapshot(snapshot);
 *
 * // Query history
 * HistoryQuery query = HistoryQuery::for_entity(123, 10);
 * auto result = store->query(query);
 * for (const auto& snap : result.snapshots) {
 *     // Process snapshot
 * }
 *
 * // Export to Parquet
 * ExportConfig export_cfg = ExportConfig::to_parquet("/data/history.parquet");
 * store->export_history(export_cfg);
 * @endcode
 */
class HistoryStore {
public:
    virtual ~HistoryStore() = default;

    // ========================================================================
    // Lifecycle
    // ========================================================================

    /**
     * @brief Initialize the history store
     * @return Success or error code
     */
    virtual HistoryResult initialize() = 0;

    /**
     * @brief Shutdown the history store
     * @return Success or error code
     */
    virtual HistoryResult shutdown() = 0;

    /**
     * @brief Check if initialized
     */
    virtual bool is_initialized() const = 0;

    // ========================================================================
    // Snapshot Storage
    // ========================================================================

    /**
     * @brief Store a state snapshot
     * @param snapshot Snapshot to store
     * @return Success or error code
     */
    virtual HistoryResult store_snapshot(StateSnapshot snapshot) = 0;

    /**
     * @brief Store multiple snapshots in batch
     * @param snapshots Vector of snapshots to store
     * @return Success or error code
     */
    virtual HistoryResult store_snapshots(const std::vector<StateSnapshot>& snapshots) = 0;

    // ========================================================================
    // Snapshot Retrieval
    // ========================================================================

    /**
     * @brief Get a specific snapshot by entity and sequence
     * @param entity_id Entity identifier
     * @param sequence Sequence number
     * @return Snapshot if found, nullopt otherwise
     */
    virtual std::optional<StateSnapshot> get_snapshot(EntityId entity_id, UInt64 sequence) const = 0;

    /**
     * @brief Get the latest snapshot for an entity
     * @param entity_id Entity identifier
     * @return Latest snapshot if found, nullopt otherwise
     */
    virtual std::optional<StateSnapshot> get_latest_snapshot(EntityId entity_id) const = 0;

    /**
     * @brief Get the first snapshot for an entity
     * @param entity_id Entity identifier
     * @return First snapshot if found, nullopt otherwise
     */
    virtual std::optional<StateSnapshot> get_first_snapshot(EntityId entity_id) const = 0;

    /**
     * @brief Get snapshot closest to a specific time
     * @param entity_id Entity identifier
     * @param time Target time
     * @return Closest snapshot if found, nullopt otherwise
     */
    virtual std::optional<StateSnapshot> get_snapshot_at_time(
        EntityId entity_id,
        std::chrono::system_clock::time_point time) const = 0;

    // ========================================================================
    // Querying
    // ========================================================================

    /**
     * @brief Query snapshots with filtering
     * @param query Query parameters
     * @return Query result with matching snapshots
     */
    virtual HistoryQueryResult query(const HistoryQuery& query) const = 0;

    /**
     * @brief Get all snapshots for an entity
     * @param entity_id Entity identifier
     * @param limit Maximum results
     * @return Query result
     */
    virtual HistoryQueryResult get_entity_history(EntityId entity_id, UInt32 limit = 1000) const = 0;

    /**
     * @brief Get snapshots in a time range
     * @param time_range Time range to query
     * @param limit Maximum results
     * @return Query result
     */
    virtual HistoryQueryResult get_snapshots_in_range(const QueryTimeRange& time_range,
                                                      UInt32 limit = 1000) const = 0;

    // ========================================================================
    // Deletion
    // ========================================================================

    /**
     * @brief Delete all history for an entity
     * @param entity_id Entity identifier
     * @return Success or error code
     */
    virtual HistoryResult delete_entity_history(EntityId entity_id) = 0;

    /**
     * @brief Delete a specific snapshot
     * @param entity_id Entity identifier
     * @param sequence Sequence number
     * @return Success or error code
     */
    virtual HistoryResult delete_snapshot(EntityId entity_id, UInt64 sequence) = 0;

    /**
     * @brief Delete snapshots older than specified time
     * @param cutoff_time Delete snapshots before this time
     * @return Success or error code
     */
    virtual HistoryResult delete_before(std::chrono::system_clock::time_point cutoff_time) = 0;

    /**
     * @brief Clear all snapshots
     * @return Success or error code
     */
    virtual HistoryResult clear() = 0;

    // ========================================================================
    // Retention Management
    // ========================================================================

    /**
     * @brief Apply retention policy (delete old snapshots)
     * @return Success or error code
     */
    virtual HistoryResult apply_retention() = 0;

    /**
     * @brief Update retention policy
     * @param policy New retention policy
     * @return Success or error code
     */
    virtual HistoryResult set_retention_policy(const RetentionPolicy& policy) = 0;

    /**
     * @brief Get current retention policy
     */
    virtual const RetentionPolicy& get_retention_policy() const = 0;

    // ========================================================================
    // Export
    // ========================================================================

    /**
     * @brief Export history data to file
     * @param config Export configuration
     * @return Success or error code
     */
    virtual HistoryResult export_history(const ExportConfig& config) = 0;

    // ========================================================================
    // Backend Management
    // ========================================================================

    /**
     * @brief Set storage backend
     * @param backend Storage backend implementation
     * @return Success or error code
     */
    virtual HistoryResult set_backend(std::shared_ptr<IHistoryBackend> backend) = 0;

    /**
     * @brief Get current backend
     */
    virtual std::shared_ptr<IHistoryBackend> get_backend() const = 0;

    /**
     * @brief Set exporter
     * @param exporter Export implementation
     * @return Success or error code
     */
    virtual HistoryResult set_exporter(std::shared_ptr<IHistoryExporter> exporter) = 0;

    /**
     * @brief Get current exporter
     */
    virtual std::shared_ptr<IHistoryExporter> get_exporter() const = 0;

    // ========================================================================
    // Statistics
    // ========================================================================

    /**
     * @brief Get history store statistics
     */
    virtual HistoryStoreStats get_stats() const = 0;

    /**
     * @brief Get configuration
     */
    virtual const HistoryStoreConfig& get_config() const = 0;

    /**
     * @brief Get entity count
     */
    virtual UInt64 get_entity_count() const = 0;

    /**
     * @brief Get snapshot count for entity
     * @param entity_id Entity identifier
     * @return Snapshot count
     */
    virtual UInt64 get_entity_snapshot_count(EntityId entity_id) const = 0;

    /**
     * @brief Check if entity has history
     * @param entity_id Entity identifier
     * @return True if entity has snapshots
     */
    virtual bool has_entity_history(EntityId entity_id) const = 0;
};

// ============================================================================
// Factory Functions
// ============================================================================

/**
 * @brief Create a history store
 * @param config History store configuration
 * @return Unique pointer to the history store
 */
std::unique_ptr<HistoryStore> create_history_store(const HistoryStoreConfig& config);

/**
 * @brief Create an in-memory storage backend
 * @return Unique pointer to the backend
 */
std::unique_ptr<IHistoryBackend> create_memory_backend();

/**
 * @brief Create a disk-based storage backend
 * @param storage_path Path to storage directory
 * @return Unique pointer to the backend
 */
std::unique_ptr<IHistoryBackend> create_disk_backend(const std::string& storage_path);

/**
 * @brief Create a Parquet exporter
 * @return Unique pointer to the exporter
 */
std::unique_ptr<IHistoryExporter> create_parquet_exporter();

/**
 * @brief Create a CSV exporter
 * @return Unique pointer to the exporter
 */
std::unique_ptr<IHistoryExporter> create_csv_exporter();

/**
 * @brief Create a JSON exporter
 * @return Unique pointer to the exporter
 */
std::unique_ptr<IHistoryExporter> create_json_exporter();

/**
 * @brief Create a multi-format exporter (supports all formats)
 * @return Unique pointer to the exporter
 */
std::unique_ptr<IHistoryExporter> create_multi_format_exporter();

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * @brief Compute hash of state data for integrity verification
 * @param data State data to hash
 * @return Hash value
 */
UInt64 compute_state_hash(const std::vector<UInt8>& data);

/**
 * @brief Verify snapshot integrity
 * @param snapshot Snapshot to verify
 * @return True if hash matches data
 */
bool verify_snapshot_integrity(const StateSnapshot& snapshot);

/**
 * @brief Compress snapshot data
 * @param data Uncompressed data
 * @return Compressed data
 */
std::vector<UInt8> compress_snapshot_data(const std::vector<UInt8>& data);

/**
 * @brief Decompress snapshot data
 * @param data Compressed data
 * @return Decompressed data
 */
std::vector<UInt8> decompress_snapshot_data(const std::vector<UInt8>& data);

/**
 * @brief Calculate age of snapshot
 * @param snapshot Snapshot to check
 * @param current_time Current time
 * @return Age in hours
 */
inline UInt64 calculate_snapshot_age_hours(
    const StateSnapshot& snapshot,
    std::chrono::system_clock::time_point current_time = std::chrono::system_clock::now()) {
    auto duration = current_time - snapshot.timestamp;
    auto hours = std::chrono::duration_cast<std::chrono::hours>(duration).count();
    return hours > 0 ? static_cast<UInt64>(hours) : 0;
}

/**
 * @brief Create time range from now back N hours
 * @param hours Hours to look back
 * @return Time range
 */
inline QueryTimeRange time_range_last_hours(UInt64 hours) {
    auto now = std::chrono::system_clock::now();
    auto start = now - std::chrono::hours(hours);
    return QueryTimeRange(start, now);
}

/**
 * @brief Create time range from now back N days
 * @param days Days to look back
 * @return Time range
 */
inline QueryTimeRange time_range_last_days(UInt64 days) {
    auto now = std::chrono::system_clock::now();
    auto start = now - std::chrono::hours(days * 24);
    return QueryTimeRange(start, now);
}

} // namespace jaguar::thread
