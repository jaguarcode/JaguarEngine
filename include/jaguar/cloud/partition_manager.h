#pragma once
/**
 * @file partition_manager.h
 * @brief Entity partitioning system for distributed simulation
 *
 * This file provides the partitioning infrastructure for distributing
 * simulation entities across multiple compute nodes. Supports spatial,
 * domain-based, and hybrid partitioning strategies with load balancing.
 *
 * Key features:
 * - Spatial partitioning using octree for geographic locality
 * - Domain-based partitioning for physics specialization
 * - Load balancing with automatic rebalancing
 * - Entity migration between partitions
 * - Predictive migration for reduced latency
 */

#include "jaguar/core/types.h"
#include "jaguar/physics/entity.h"
#include <vector>
#include <array>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <functional>
#include <optional>
#include <chrono>
#include <mutex>
#include <atomic>
#include <string>

namespace jaguar::cloud {

// ============================================================================
// Forward Declarations
// ============================================================================

class IPartitionStrategy;
class ISpatialPartitioner;
class IDomainPartitioner;
class ILoadBalancer;
class IEntityMigrator;
class PartitionManager;

// ============================================================================
// Partition Result Enum
// ============================================================================

/**
 * @brief Result codes for partition operations
 */
enum class PartitionResult : UInt8 {
    Success = 0,

    // Configuration errors
    InvalidConfiguration,
    InvalidPartitionId,
    InvalidNodeId,
    InvalidEntityId,
    InvalidBounds,

    // Operational errors
    PartitionNotFound,
    NodeNotFound,
    EntityNotFound,
    MigrationFailed,
    BalancingFailed,
    SplitFailed,
    MergeFailed,

    // State errors
    NotInitialized,
    AlreadyInitialized,
    OperationInProgress,
    PartitionLocked,

    // Resource errors
    OutOfMemory,
    CapacityExceeded,
    NodeUnavailable,

    // Network errors
    NetworkError,
    Timeout,
    ConnectionLost
};

/**
 * @brief Convert PartitionResult to string
 */
inline const char* partition_result_to_string(PartitionResult result) {
    switch (result) {
        case PartitionResult::Success: return "Success";
        case PartitionResult::InvalidConfiguration: return "InvalidConfiguration";
        case PartitionResult::InvalidPartitionId: return "InvalidPartitionId";
        case PartitionResult::InvalidNodeId: return "InvalidNodeId";
        case PartitionResult::InvalidEntityId: return "InvalidEntityId";
        case PartitionResult::InvalidBounds: return "InvalidBounds";
        case PartitionResult::PartitionNotFound: return "PartitionNotFound";
        case PartitionResult::NodeNotFound: return "NodeNotFound";
        case PartitionResult::EntityNotFound: return "EntityNotFound";
        case PartitionResult::MigrationFailed: return "MigrationFailed";
        case PartitionResult::BalancingFailed: return "BalancingFailed";
        case PartitionResult::SplitFailed: return "SplitFailed";
        case PartitionResult::MergeFailed: return "MergeFailed";
        case PartitionResult::NotInitialized: return "NotInitialized";
        case PartitionResult::AlreadyInitialized: return "AlreadyInitialized";
        case PartitionResult::OperationInProgress: return "OperationInProgress";
        case PartitionResult::PartitionLocked: return "PartitionLocked";
        case PartitionResult::OutOfMemory: return "OutOfMemory";
        case PartitionResult::CapacityExceeded: return "CapacityExceeded";
        case PartitionResult::NodeUnavailable: return "NodeUnavailable";
        case PartitionResult::NetworkError: return "NetworkError";
        case PartitionResult::Timeout: return "Timeout";
        case PartitionResult::ConnectionLost: return "ConnectionLost";
        default: return "Unknown";
    }
}

// ============================================================================
// Partition Strategy Enum
// ============================================================================

/**
 * @brief Partitioning strategy for distributing entities
 */
enum class PartitionStrategy : UInt8 {
    /// Spatial partitioning using octree (geography-based)
    Spatial,

    /// Domain-based partitioning (Air, Land, Sea, Space)
    Domain,

    /// Hybrid: Domain first, then spatial within domain
    DomainThenSpatial,

    /// Hybrid: Spatial first, then domain within region
    SpatialThenDomain,

    /// Round-robin assignment for load testing
    RoundRobin,

    /// Hash-based assignment for consistent distribution
    HashBased,

    /// Custom partitioning using user-defined function
    Custom
};

/**
 * @brief Convert PartitionStrategy to string
 */
inline const char* partition_strategy_to_string(PartitionStrategy strategy) {
    switch (strategy) {
        case PartitionStrategy::Spatial: return "Spatial";
        case PartitionStrategy::Domain: return "Domain";
        case PartitionStrategy::DomainThenSpatial: return "DomainThenSpatial";
        case PartitionStrategy::SpatialThenDomain: return "SpatialThenDomain";
        case PartitionStrategy::RoundRobin: return "RoundRobin";
        case PartitionStrategy::HashBased: return "HashBased";
        case PartitionStrategy::Custom: return "Custom";
        default: return "Unknown";
    }
}

// ============================================================================
// Load Balancing Strategy
// ============================================================================

/**
 * @brief Strategy for load balancing across nodes
 */
enum class LoadBalanceStrategy : UInt8 {
    /// No automatic balancing
    None,

    /// Balance based on entity count
    EntityCount,

    /// Balance based on CPU load
    CPULoad,

    /// Balance based on memory usage
    MemoryUsage,

    /// Balance based on network bandwidth
    NetworkBandwidth,

    /// Weighted combination of multiple factors
    Weighted,

    /// ML-based predictive balancing
    Predictive
};

// ============================================================================
// Migration Priority
// ============================================================================

/**
 * @brief Priority level for entity migration
 */
enum class MigrationPriority : UInt8 {
    /// Low priority - migrate during idle time
    Low,

    /// Normal priority - migrate when convenient
    Normal,

    /// High priority - migrate soon
    High,

    /// Critical - migrate immediately
    Critical
};

// ============================================================================
// Axis-Aligned Bounding Box
// ============================================================================

/**
 * @brief 3D axis-aligned bounding box for spatial queries
 */
struct AABB {
    Vec3 min{0.0, 0.0, 0.0};  ///< Minimum corner
    Vec3 max{0.0, 0.0, 0.0};  ///< Maximum corner

    constexpr AABB() noexcept = default;
    constexpr AABB(const Vec3& min_, const Vec3& max_) noexcept
        : min(min_), max(max_) {}

    /// Get center point
    constexpr Vec3 center() const noexcept {
        return (min + max) * 0.5;
    }

    /// Get half-extents (size / 2)
    constexpr Vec3 half_extents() const noexcept {
        return (max - min) * 0.5;
    }

    /// Get full size
    constexpr Vec3 size() const noexcept {
        return max - min;
    }

    /// Get volume
    constexpr Real volume() const noexcept {
        Vec3 s = size();
        return s.x * s.y * s.z;
    }

    /// Check if point is inside
    constexpr bool contains(const Vec3& point) const noexcept {
        return point.x >= min.x && point.x <= max.x &&
               point.y >= min.y && point.y <= max.y &&
               point.z >= min.z && point.z <= max.z;
    }

    /// Check if another AABB overlaps
    constexpr bool overlaps(const AABB& other) const noexcept {
        return min.x <= other.max.x && max.x >= other.min.x &&
               min.y <= other.max.y && max.y >= other.min.y &&
               min.z <= other.max.z && max.z >= other.min.z;
    }

    /// Expand to include a point
    void expand(const Vec3& point) noexcept {
        if (point.x < min.x) min.x = point.x;
        if (point.y < min.y) min.y = point.y;
        if (point.z < min.z) min.z = point.z;
        if (point.x > max.x) max.x = point.x;
        if (point.y > max.y) max.y = point.y;
        if (point.z > max.z) max.z = point.z;
    }

    /// Expand to include another AABB
    void expand(const AABB& other) noexcept {
        expand(other.min);
        expand(other.max);
    }

    /// Create AABB for Earth in ECEF coordinates
    static AABB earth_bounds() noexcept {
        // Earth radius ~6.4e6 meters, with margin
        constexpr Real extent = 1.0e7;
        return AABB{
            Vec3{-extent, -extent, -extent},
            Vec3{extent, extent, extent}
        };
    }

    /// Create AABB from center and half-extents
    static constexpr AABB from_center_half_extents(const Vec3& center,
                                                    const Vec3& half_extents) noexcept {
        return AABB{center - half_extents, center + half_extents};
    }
};

// ============================================================================
// Node Identifier
// ============================================================================

/**
 * @brief Unique identifier for a compute node
 */
using NodeId = UInt32;

constexpr NodeId INVALID_NODE_ID = std::numeric_limits<NodeId>::max();
constexpr NodeId LOCAL_NODE_ID = 0;

// ============================================================================
// Partition Identifier
// ============================================================================

/**
 * @brief Unique identifier for a partition
 */
using PartitionId = UInt64;

constexpr PartitionId INVALID_PARTITION_ID = std::numeric_limits<PartitionId>::max();

// ============================================================================
// Node Information
// ============================================================================

/**
 * @brief Information about a compute node
 */
struct NodeInfo {
    NodeId id{INVALID_NODE_ID};        ///< Unique node identifier
    std::string hostname;               ///< Network hostname
    std::string ip_address;             ///< IP address
    UInt16 port{0};                     ///< Communication port
    UInt32 cpu_cores{0};                ///< Available CPU cores
    UInt64 memory_bytes{0};             ///< Total memory in bytes
    bool has_gpu{false};                ///< GPU acceleration available
    bool is_local{false};               ///< Is this the local node
    bool is_online{false};              ///< Is node currently online

    /// Current load metrics
    Real cpu_usage{0.0};                ///< CPU usage (0.0 - 1.0)
    Real memory_usage{0.0};             ///< Memory usage (0.0 - 1.0)
    Real network_bandwidth{0.0};        ///< Network usage (bytes/sec)
    UInt32 entity_count{0};             ///< Number of entities on this node
    UInt32 partition_count{0};          ///< Number of partitions on this node

    /// Timing
    std::chrono::steady_clock::time_point last_heartbeat;
    Real latency_ms{0.0};               ///< Network latency to this node
};

// ============================================================================
// Partition Information
// ============================================================================

/**
 * @brief Information about a partition
 */
struct PartitionInfo {
    PartitionId id{INVALID_PARTITION_ID}; ///< Unique partition identifier
    NodeId owner_node{INVALID_NODE_ID};   ///< Node that owns this partition
    AABB bounds;                           ///< Spatial bounds
    Domain domain{Domain::Generic};        ///< Primary domain (if domain-based)
    UInt32 depth{0};                       ///< Depth in octree (for spatial)
    UInt32 entity_count{0};                ///< Number of entities
    UInt64 memory_bytes{0};                ///< Estimated memory usage
    Real load_factor{0.0};                 ///< Current load (0.0 - 1.0)
    bool is_leaf{true};                    ///< Is leaf partition (no children)
    bool is_locked{false};                 ///< Is partition locked for migration

    /// Parent and children (for hierarchical partitioning)
    PartitionId parent{INVALID_PARTITION_ID};
    std::vector<PartitionId> children;

    /// Entities in this partition
    std::unordered_set<EntityId> entities;

    /// Neighboring partitions (for boundary handling)
    std::unordered_set<PartitionId> neighbors;
};

// ============================================================================
// Entity Location
// ============================================================================

/**
 * @brief Tracks which partition an entity belongs to
 */
struct EntityLocation {
    EntityId entity_id{INVALID_ENTITY_ID};
    PartitionId partition_id{INVALID_PARTITION_ID};
    NodeId node_id{INVALID_NODE_ID};
    Vec3 position;                         ///< Cached position
    Domain domain{Domain::Generic};        ///< Entity domain
    bool is_boundary{false};               ///< Near partition boundary
    Real boundary_distance{0.0};           ///< Distance to nearest boundary
};

// ============================================================================
// Migration Request
// ============================================================================

/**
 * @brief Request to migrate an entity between partitions/nodes
 */
struct MigrationRequest {
    EntityId entity_id{INVALID_ENTITY_ID};
    PartitionId source_partition{INVALID_PARTITION_ID};
    PartitionId target_partition{INVALID_PARTITION_ID};
    NodeId source_node{INVALID_NODE_ID};
    NodeId target_node{INVALID_NODE_ID};
    MigrationPriority priority{MigrationPriority::Normal};
    std::string reason;                    ///< Reason for migration

    /// Timing
    std::chrono::steady_clock::time_point requested_at;
    std::chrono::steady_clock::time_point started_at;
    std::chrono::steady_clock::time_point completed_at;

    /// State
    bool is_pending{true};
    bool is_in_progress{false};
    bool is_completed{false};
    bool is_failed{false};
    PartitionResult result{PartitionResult::Success};
};

// ============================================================================
// Partition Configuration
// ============================================================================

/**
 * @brief Configuration for the partition manager
 */
struct PartitionConfig {
    /// Primary partitioning strategy
    PartitionStrategy strategy{PartitionStrategy::Spatial};

    /// Load balancing strategy
    LoadBalanceStrategy balance_strategy{LoadBalanceStrategy::EntityCount};

    /// Spatial partitioning settings
    AABB world_bounds{AABB::earth_bounds()}; ///< World bounds for spatial partitioning
    UInt32 max_octree_depth{8};              ///< Maximum octree depth
    UInt32 min_entities_per_partition{100};  ///< Minimum entities before splitting
    UInt32 max_entities_per_partition{10000}; ///< Maximum entities before splitting

    /// Load balancing settings
    Real balance_threshold{0.2};             ///< Imbalance threshold to trigger rebalancing
    Real balance_interval_seconds{10.0};     ///< How often to check balance
    bool auto_balance{true};                 ///< Enable automatic load balancing

    /// Migration settings
    UInt32 max_migrations_per_step{100};     ///< Max migrations per simulation step
    Real migration_cooldown_seconds{1.0};    ///< Cooldown between migrations for same entity
    bool predictive_migration{true};         ///< Enable predictive migration
    Real prediction_horizon_seconds{5.0};    ///< How far ahead to predict

    /// Boundary handling
    Real boundary_overlap_meters{1000.0};    ///< Overlap region at boundaries
    bool replicate_boundary_entities{true};  ///< Replicate entities near boundaries

    /// Network settings
    Real heartbeat_interval_seconds{1.0};    ///< Node heartbeat interval
    Real node_timeout_seconds{5.0};          ///< Consider node dead after timeout
    UInt32 max_retry_attempts{3};            ///< Max retries for failed operations

    /// Factory methods for common configurations
    static PartitionConfig default_config() noexcept {
        return PartitionConfig{};
    }

    static PartitionConfig small_scale() noexcept {
        PartitionConfig config;
        config.max_octree_depth = 4;
        config.min_entities_per_partition = 10;
        config.max_entities_per_partition = 1000;
        config.max_migrations_per_step = 10;
        return config;
    }

    static PartitionConfig large_scale() noexcept {
        PartitionConfig config;
        config.max_octree_depth = 10;
        config.min_entities_per_partition = 1000;
        config.max_entities_per_partition = 50000;
        config.max_migrations_per_step = 500;
        config.balance_interval_seconds = 5.0;
        return config;
    }

    static PartitionConfig domain_based() noexcept {
        PartitionConfig config;
        config.strategy = PartitionStrategy::Domain;
        config.balance_strategy = LoadBalanceStrategy::CPULoad;
        return config;
    }
};

// ============================================================================
// Partition Statistics
// ============================================================================

/**
 * @brief Statistics about partitioning operations
 */
struct PartitionStats {
    /// Counts
    UInt32 total_partitions{0};
    UInt32 active_partitions{0};
    UInt32 total_nodes{0};
    UInt32 online_nodes{0};
    UInt64 total_entities{0};

    /// Load metrics
    Real average_load{0.0};
    Real max_load{0.0};
    Real min_load{0.0};
    Real load_imbalance{0.0};

    /// Migration metrics
    UInt64 total_migrations{0};
    UInt64 successful_migrations{0};
    UInt64 failed_migrations{0};
    UInt64 pending_migrations{0};
    Real average_migration_time_ms{0.0};

    /// Timing
    Real last_balance_time_ms{0.0};
    Real total_balance_time_ms{0.0};
    UInt64 balance_count{0};

    /// Memory
    UInt64 total_memory_bytes{0};
    UInt64 partition_overhead_bytes{0};

    /// Reset all counters
    void reset() noexcept {
        *this = PartitionStats{};
    }
};

// ============================================================================
// Partition Callbacks
// ============================================================================

/// Callback when entity is assigned to a partition
using EntityAssignedCallback = std::function<void(EntityId, PartitionId, NodeId)>;

/// Callback when entity migrates between partitions
using EntityMigratedCallback = std::function<void(const MigrationRequest&)>;

/// Callback when partition is split or merged
using PartitionChangedCallback = std::function<void(PartitionId, bool is_split)>;

/// Callback when load balancing occurs
using BalanceCallback = std::function<void(const PartitionStats&)>;

/// Custom partition assignment function
using CustomPartitionFunc = std::function<PartitionId(EntityId, const Vec3&, Domain)>;

// ============================================================================
// Octree Node (Spatial Partitioner)
// ============================================================================

/**
 * @brief Octree node for spatial partitioning
 */
struct OctreeNode {
    PartitionId partition_id{INVALID_PARTITION_ID};
    AABB bounds;
    UInt32 depth{0};
    bool is_leaf{true};

    /// Children (8 for octree: -x-y-z, +x-y-z, -x+y-z, +x+y-z, etc.)
    std::array<std::unique_ptr<OctreeNode>, 8> children;

    /// Entities at this node (only if leaf)
    std::unordered_set<EntityId> entities;

    /// Get child index for a position
    UInt32 get_child_index(const Vec3& position) const noexcept {
        Vec3 center = bounds.center();
        UInt32 index = 0;
        if (position.x >= center.x) index |= 1;
        if (position.y >= center.y) index |= 2;
        if (position.z >= center.z) index |= 4;
        return index;
    }

    /// Get bounds for a child
    AABB get_child_bounds(UInt32 child_index) const noexcept {
        Vec3 center = bounds.center();
        Vec3 half = bounds.half_extents() * 0.5;
        Vec3 child_center = center;

        if (child_index & 1) child_center.x += half.x; else child_center.x -= half.x;
        if (child_index & 2) child_center.y += half.y; else child_center.y -= half.y;
        if (child_index & 4) child_center.z += half.z; else child_center.z -= half.z;

        return AABB::from_center_half_extents(child_center, half);
    }
};

// ============================================================================
// Spatial Partitioner Interface
// ============================================================================

/**
 * @brief Interface for spatial partitioning algorithms
 */
class ISpatialPartitioner {
public:
    virtual ~ISpatialPartitioner() = default;

    /// Initialize with world bounds
    virtual PartitionResult initialize(const AABB& world_bounds,
                                        UInt32 max_depth) = 0;

    /// Insert entity at position
    virtual PartitionResult insert(EntityId entity_id,
                                    const Vec3& position) = 0;

    /// Remove entity
    virtual PartitionResult remove(EntityId entity_id) = 0;

    /// Update entity position
    virtual PartitionResult update(EntityId entity_id,
                                    const Vec3& new_position) = 0;

    /// Find partition containing position
    virtual PartitionId find_partition(const Vec3& position) const = 0;

    /// Find all entities in region
    virtual std::vector<EntityId> query_region(const AABB& region) const = 0;

    /// Find all partitions overlapping region
    virtual std::vector<PartitionId> query_partitions(const AABB& region) const = 0;

    /// Get partition bounds
    virtual std::optional<AABB> get_partition_bounds(PartitionId partition_id) const = 0;

    /// Split partition into children
    virtual PartitionResult split(PartitionId partition_id) = 0;

    /// Merge child partitions back to parent
    virtual PartitionResult merge(PartitionId parent_id) = 0;

    /// Get all leaf partitions
    virtual std::vector<PartitionId> get_leaf_partitions() const = 0;

    /// Get entity count in partition
    virtual UInt32 get_entity_count(PartitionId partition_id) const = 0;
};

// ============================================================================
// Domain Partitioner Interface
// ============================================================================

/**
 * @brief Interface for domain-based partitioning
 */
class IDomainPartitioner {
public:
    virtual ~IDomainPartitioner() = default;

    /// Initialize with domains to partition
    virtual PartitionResult initialize(const std::vector<Domain>& domains) = 0;

    /// Assign entity to domain partition
    virtual PartitionResult assign(EntityId entity_id, Domain domain) = 0;

    /// Remove entity from domain partition
    virtual PartitionResult remove(EntityId entity_id) = 0;

    /// Get partition for domain
    virtual PartitionId get_domain_partition(Domain domain) const = 0;

    /// Get all entities in domain
    virtual std::vector<EntityId> get_entities(Domain domain) const = 0;

    /// Get entity count per domain
    virtual std::unordered_map<Domain, UInt32> get_domain_counts() const = 0;
};

// ============================================================================
// Load Balancer Interface
// ============================================================================

/**
 * @brief Interface for load balancing algorithms
 */
class ILoadBalancer {
public:
    virtual ~ILoadBalancer() = default;

    /// Initialize with nodes and config
    virtual PartitionResult initialize(const std::vector<NodeInfo>& nodes,
                                        const PartitionConfig& config) = 0;

    /// Update node metrics
    virtual void update_node_metrics(const NodeInfo& node) = 0;

    /// Calculate load imbalance (0.0 = perfect, 1.0 = completely unbalanced)
    virtual Real calculate_imbalance() const = 0;

    /// Generate migration plan to balance load
    virtual std::vector<MigrationRequest> generate_migration_plan(
        const std::unordered_map<PartitionId, PartitionInfo>& partitions) = 0;

    /// Find best node for new partition
    virtual NodeId find_best_node_for_partition(const PartitionInfo& partition) const = 0;

    /// Get load per node
    virtual std::unordered_map<NodeId, Real> get_node_loads() const = 0;

    /// Should rebalancing occur?
    virtual bool should_rebalance() const = 0;
};

// ============================================================================
// Entity Migrator Interface
// ============================================================================

/**
 * @brief Interface for entity migration between nodes
 */
class IEntityMigrator {
public:
    virtual ~IEntityMigrator() = default;

    /// Initialize migrator
    virtual PartitionResult initialize(const PartitionConfig& config) = 0;

    /// Request migration (async)
    virtual PartitionResult request_migration(const MigrationRequest& request) = 0;

    /// Cancel pending migration
    virtual PartitionResult cancel_migration(EntityId entity_id) = 0;

    /// Process pending migrations (call each step)
    virtual std::vector<MigrationRequest> process_migrations(UInt32 max_count) = 0;

    /// Get pending migration count
    virtual UInt32 pending_count() const = 0;

    /// Get in-progress migration count
    virtual UInt32 in_progress_count() const = 0;

    /// Check if entity is being migrated
    virtual bool is_migrating(EntityId entity_id) const = 0;

    /// Serialize entity state for migration
    virtual std::vector<UInt8> serialize_entity(EntityId entity_id,
                                                  const physics::EntityState& state) = 0;

    /// Deserialize entity state after migration
    virtual std::optional<physics::EntityState> deserialize_entity(
        const std::vector<UInt8>& data) = 0;
};

// ============================================================================
// Predictive Migrator
// ============================================================================

/**
 * @brief Predicts entity movement and pre-migrates
 */
class IPredictiveMigrator {
public:
    virtual ~IPredictiveMigrator() = default;

    /// Update entity trajectory prediction
    virtual void update_prediction(EntityId entity_id,
                                    const Vec3& position,
                                    const Vec3& velocity,
                                    const Vec3& acceleration) = 0;

    /// Get predicted position at future time
    virtual Vec3 predict_position(EntityId entity_id,
                                   Real seconds_ahead) const = 0;

    /// Check if entity will cross partition boundary soon
    virtual std::optional<PartitionId> will_cross_boundary(
        EntityId entity_id,
        PartitionId current_partition,
        Real horizon_seconds) const = 0;

    /// Get entities that should be pre-migrated
    virtual std::vector<MigrationRequest> get_predictive_migrations(
        Real horizon_seconds) const = 0;

    /// Clear prediction for entity
    virtual void clear_prediction(EntityId entity_id) = 0;
};

// ============================================================================
// Partition Manager
// ============================================================================

/**
 * @brief Main partition manager for distributed simulation
 *
 * Coordinates spatial partitioning, domain partitioning, load balancing,
 * and entity migration for distributed simulation across compute nodes.
 */
class PartitionManager {
public:
    PartitionManager();
    ~PartitionManager();

    // Non-copyable, movable
    PartitionManager(const PartitionManager&) = delete;
    PartitionManager& operator=(const PartitionManager&) = delete;
    PartitionManager(PartitionManager&&) noexcept;
    PartitionManager& operator=(PartitionManager&&) noexcept;

    // ========================================================================
    // Lifecycle
    // ========================================================================

    /**
     * @brief Initialize partition manager
     * @param config Partition configuration
     * @return Success or error code
     */
    PartitionResult initialize(const PartitionConfig& config);

    /**
     * @brief Shutdown and cleanup
     */
    void shutdown();

    /**
     * @brief Check if initialized
     */
    bool is_initialized() const noexcept;

    /**
     * @brief Get current configuration
     */
    const PartitionConfig& get_config() const noexcept;

    // ========================================================================
    // Node Management
    // ========================================================================

    /**
     * @brief Register a compute node
     */
    PartitionResult register_node(const NodeInfo& node);

    /**
     * @brief Unregister a compute node
     */
    PartitionResult unregister_node(NodeId node_id);

    /**
     * @brief Update node metrics
     */
    void update_node_metrics(const NodeInfo& node);

    /**
     * @brief Get node information
     */
    std::optional<NodeInfo> get_node(NodeId node_id) const;

    /**
     * @brief Get all registered nodes
     */
    std::vector<NodeInfo> get_all_nodes() const;

    /**
     * @brief Get local node ID
     */
    NodeId get_local_node_id() const noexcept;

    /**
     * @brief Set local node ID
     */
    void set_local_node_id(NodeId node_id) noexcept;

    // ========================================================================
    // Entity Assignment
    // ========================================================================

    /**
     * @brief Assign entity to a partition
     * @param entity_id Entity to assign
     * @param position Entity position (for spatial partitioning)
     * @param domain Entity domain (for domain partitioning)
     * @return Success or error code
     */
    PartitionResult assign_entity(EntityId entity_id,
                                   const Vec3& position,
                                   Domain domain = Domain::Generic);

    /**
     * @brief Remove entity from partitioning
     */
    PartitionResult remove_entity(EntityId entity_id);

    /**
     * @brief Update entity position (may trigger migration)
     */
    PartitionResult update_entity_position(EntityId entity_id,
                                            const Vec3& new_position);

    /**
     * @brief Get entity location
     */
    std::optional<EntityLocation> get_entity_location(EntityId entity_id) const;

    /**
     * @brief Check if entity is on local node
     */
    bool is_local_entity(EntityId entity_id) const;

    /**
     * @brief Get all entities in partition
     */
    std::vector<EntityId> get_entities_in_partition(PartitionId partition_id) const;

    /**
     * @brief Get all entities on node
     */
    std::vector<EntityId> get_entities_on_node(NodeId node_id) const;

    /**
     * @brief Get all local entities
     */
    std::vector<EntityId> get_local_entities() const;

    // ========================================================================
    // Partition Management
    // ========================================================================

    /**
     * @brief Get partition information
     */
    std::optional<PartitionInfo> get_partition(PartitionId partition_id) const;

    /**
     * @brief Get all partitions
     */
    std::vector<PartitionInfo> get_all_partitions() const;

    /**
     * @brief Get partitions on node
     */
    std::vector<PartitionId> get_partitions_on_node(NodeId node_id) const;

    /**
     * @brief Get partitions in region
     */
    std::vector<PartitionId> query_partitions(const AABB& region) const;

    /**
     * @brief Get neighboring partitions
     */
    std::vector<PartitionId> get_neighbors(PartitionId partition_id) const;

    /**
     * @brief Manually split a partition
     */
    PartitionResult split_partition(PartitionId partition_id);

    /**
     * @brief Manually merge partitions
     */
    PartitionResult merge_partitions(PartitionId parent_id);

    // ========================================================================
    // Load Balancing
    // ========================================================================

    /**
     * @brief Trigger load balancing
     */
    PartitionResult balance_load();

    /**
     * @brief Get current load imbalance
     */
    Real get_load_imbalance() const;

    /**
     * @brief Get load for each node
     */
    std::unordered_map<NodeId, Real> get_node_loads() const;

    /**
     * @brief Set load balancing strategy
     */
    void set_balance_strategy(LoadBalanceStrategy strategy);

    // ========================================================================
    // Migration
    // ========================================================================

    /**
     * @brief Request entity migration
     */
    PartitionResult request_migration(EntityId entity_id,
                                       PartitionId target_partition,
                                       MigrationPriority priority = MigrationPriority::Normal);

    /**
     * @brief Cancel pending migration
     */
    PartitionResult cancel_migration(EntityId entity_id);

    /**
     * @brief Process pending migrations
     */
    void process_migrations();

    /**
     * @brief Get pending migration count
     */
    UInt32 pending_migration_count() const;

    /**
     * @brief Check if entity is being migrated
     */
    bool is_migrating(EntityId entity_id) const;

    // ========================================================================
    // Predictive Migration
    // ========================================================================

    /**
     * @brief Update entity trajectory for prediction
     */
    void update_entity_trajectory(EntityId entity_id,
                                   const Vec3& position,
                                   const Vec3& velocity,
                                   const Vec3& acceleration);

    /**
     * @brief Enable/disable predictive migration
     */
    void set_predictive_migration(bool enabled);

    // ========================================================================
    // Step & Update
    // ========================================================================

    /**
     * @brief Update partition manager (call each simulation step)
     * @param dt Time step in seconds
     */
    void update(Real dt);

    // ========================================================================
    // Statistics
    // ========================================================================

    /**
     * @brief Get partition statistics
     */
    PartitionStats get_stats() const;

    /**
     * @brief Reset statistics
     */
    void reset_stats();

    // ========================================================================
    // Callbacks
    // ========================================================================

    /**
     * @brief Set callback for entity assignment
     */
    void set_entity_assigned_callback(EntityAssignedCallback callback);

    /**
     * @brief Set callback for entity migration
     */
    void set_entity_migrated_callback(EntityMigratedCallback callback);

    /**
     * @brief Set callback for partition changes
     */
    void set_partition_changed_callback(PartitionChangedCallback callback);

    /**
     * @brief Set callback for load balancing
     */
    void set_balance_callback(BalanceCallback callback);

    /**
     * @brief Set custom partition function (for Custom strategy)
     */
    void set_custom_partition_func(CustomPartitionFunc func);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// ============================================================================
// Factory Functions
// ============================================================================

/**
 * @brief Create spatial partitioner (octree-based)
 */
std::unique_ptr<ISpatialPartitioner> create_octree_partitioner();

/**
 * @brief Create domain partitioner
 */
std::unique_ptr<IDomainPartitioner> create_domain_partitioner();

/**
 * @brief Create load balancer
 */
std::unique_ptr<ILoadBalancer> create_load_balancer(LoadBalanceStrategy strategy);

/**
 * @brief Create entity migrator
 */
std::unique_ptr<IEntityMigrator> create_entity_migrator();

/**
 * @brief Create predictive migrator
 */
std::unique_ptr<IPredictiveMigrator> create_predictive_migrator();

} // namespace jaguar::cloud
