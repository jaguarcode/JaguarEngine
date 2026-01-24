# Cloud Burst API Reference

Cloud Burst module (`jaguar_cloud`) provides distributed simulation capabilities including state synchronization, spatial partitioning, and distributed time management with Raft consensus.

## Overview

The Cloud Burst module enables:
- **State Synchronization**: Distributed state sync with conflict resolution
- **Partition Management**: Spatial and domain-based workload distribution
- **Distributed Time**: Consensus-based time management with vector clocks

---

## State Synchronization

### Header

```cpp
#include <jaguar/cloud/state_sync.h>
```

### Core Types

```cpp
namespace jaguar::cloud {

// Synchronization strategies
enum class SyncStrategy {
    Full,           // Full state transfer
    Delta,          // Only changed fields
    Incremental,    // Streaming updates
    Snapshot,       // Point-in-time snapshot
    Adaptive        // Auto-selects based on change rate
};

// Conflict resolution modes
enum class ConflictResolution {
    LastWriteWins,      // Timestamp-based resolution
    FirstWriteWins,     // Original value preserved
    MergeValues,        // Attempt to merge
    CustomResolver,     // User-defined resolver
    VectorClockBased    // Causal ordering
};

// Synchronization result
struct SyncResult {
    bool success;
    std::string error_message;
    uint64_t version;
    size_t bytes_transferred;
    std::chrono::microseconds latency;
    size_t conflicts_resolved;
};

// State delta for incremental sync
struct StateDelta {
    uint64_t base_version;
    uint64_t target_version;
    std::vector<uint8_t> delta_data;
    std::chrono::system_clock::time_point timestamp;
    std::vector<std::string> changed_paths;
};

// Configuration
struct StateSyncConfig {
    SyncStrategy strategy = SyncStrategy::Delta;
    ConflictResolution conflict_resolution = ConflictResolution::LastWriteWins;
    std::chrono::milliseconds sync_interval{100};
    std::chrono::milliseconds timeout{5000};
    size_t max_batch_size = 1000;
    bool compression_enabled = true;
    int compression_level = 6;
    size_t max_delta_size = 1024 * 1024;  // 1 MB
    bool checksum_enabled = true;
};

}  // namespace jaguar::cloud
```

### StateSynchronizer Class

```cpp
class StateSynchronizer {
public:
    // Initialize with configuration
    SyncResult initialize(const StateSyncConfig& config);

    // Shutdown synchronizer
    void shutdown();

    // Synchronize local state to remote nodes
    SyncResult sync_state(const void* state_data, size_t size);

    // Receive state from remote node
    SyncResult receive_state(const void* state_data, size_t size,
                            const std::string& source_node);

    // Create snapshot
    SyncResult create_snapshot(const std::string& snapshot_id);

    // Restore from snapshot
    SyncResult restore_snapshot(const std::string& snapshot_id);

    // Get current version
    uint64_t get_version() const;

    // Check sync status
    bool is_synchronized() const;

    // Get pending changes count
    size_t get_pending_changes() const;

    // Register state change callback
    void set_state_change_callback(
        std::function<void(const StateDelta&)> callback);

    // Get synchronization statistics
    struct SyncStats {
        uint64_t total_syncs;
        uint64_t successful_syncs;
        uint64_t failed_syncs;
        uint64_t conflicts_resolved;
        size_t total_bytes_sent;
        size_t total_bytes_received;
        std::chrono::microseconds avg_latency;
    };
    SyncStats get_statistics() const;
};

// Factory function
std::unique_ptr<StateSynchronizer> create_state_synchronizer(
    const StateSyncConfig& config);
```

### Conflict Resolver Interface

```cpp
class IConflictResolver {
public:
    virtual ~IConflictResolver() = default;

    // Resolve conflict between local and remote values
    virtual std::vector<uint8_t> resolve(
        const std::vector<uint8_t>& local_value,
        const std::vector<uint8_t>& remote_value,
        const std::string& path,
        std::chrono::system_clock::time_point local_timestamp,
        std::chrono::system_clock::time_point remote_timestamp) = 0;

    // Check if values can be merged
    virtual bool can_merge(const std::string& path) const = 0;
};

// Built-in resolvers
std::unique_ptr<IConflictResolver> create_last_write_wins_resolver();
std::unique_ptr<IConflictResolver> create_vector_clock_resolver();
```

---

## Partition Management

### Header

```cpp
#include <jaguar/cloud/partition_manager.h>
```

### Core Types

```cpp
namespace jaguar::cloud {

// Partitioning strategies
enum class PartitionStrategy {
    Spatial,            // Geographic/spatial partitioning
    Domain,             // By entity domain type
    DomainThenSpatial,  // Hierarchical partitioning
    LoadBased,          // Dynamic load balancing
    Hybrid,             // Combined approach
    Custom              // User-defined strategy
};

// Load balancing strategies
enum class LoadBalanceStrategy {
    RoundRobin,         // Simple rotation
    LeastLoaded,        // Target least loaded node
    WeightedRandom,     // Weighted probability
    ConsistentHash,     // Consistent hashing
    Adaptive            // ML-based prediction
};

// Axis-aligned bounding box
struct AABB {
    Vec3 min_corner;
    Vec3 max_corner;

    bool contains(const Vec3& point) const;
    bool intersects(const AABB& other) const;
    AABB merge(const AABB& other) const;
    Vec3 center() const;
    Vec3 size() const;
    Real volume() const;
};

// Node information
struct NodeInfo {
    std::string node_id;
    std::string address;
    uint16_t port;
    Real current_load;          // 0.0 - 1.0
    size_t entity_count;
    size_t max_entities;
    std::chrono::milliseconds latency;
    bool is_healthy;
    std::chrono::system_clock::time_point last_heartbeat;
};

// Partition information
struct PartitionInfo {
    uint64_t partition_id;
    AABB bounds;
    std::string assigned_node;
    size_t entity_count;
    Real load_factor;
    std::vector<uint64_t> child_partitions;
};

// Migration request
struct MigrationRequest {
    uint64_t entity_id;
    std::string source_node;
    std::string target_node;
    uint64_t source_partition;
    uint64_t target_partition;
    std::chrono::system_clock::time_point requested_at;
    bool is_urgent;
};

// Partition result
struct PartitionResult {
    bool success;
    std::string error_message;
    uint64_t partition_id;
    size_t entities_affected;
    std::chrono::microseconds duration;
};

}  // namespace jaguar::cloud
```

### PartitionManager Class

```cpp
class PartitionManager {
public:
    // Initialize partition manager
    PartitionResult initialize(const PartitionConfig& config);

    // Shutdown
    void shutdown();

    // Register a compute node
    PartitionResult register_node(const NodeInfo& node);

    // Unregister a compute node
    PartitionResult unregister_node(const std::string& node_id);

    // Add entity to partition system
    PartitionResult add_entity(uint64_t entity_id, const Vec3& position,
                               Domain domain);

    // Remove entity from partition system
    PartitionResult remove_entity(uint64_t entity_id);

    // Update entity position (may trigger migration)
    PartitionResult update_entity_position(uint64_t entity_id,
                                          const Vec3& new_position);

    // Get partition for entity
    uint64_t get_entity_partition(uint64_t entity_id) const;

    // Get node assignment for entity
    std::string get_entity_node(uint64_t entity_id) const;

    // Get all entities in partition
    std::vector<uint64_t> get_partition_entities(uint64_t partition_id) const;

    // Trigger rebalancing
    PartitionResult rebalance();

    // Get migration queue
    std::vector<MigrationRequest> get_pending_migrations() const;

    // Execute migration
    PartitionResult execute_migration(const MigrationRequest& request);

    // Get partition tree (for octree-based spatial partitioning)
    const PartitionInfo& get_root_partition() const;

    // Query partitions intersecting region
    std::vector<uint64_t> query_partitions(const AABB& region) const;

    // Statistics
    struct PartitionStats {
        size_t total_partitions;
        size_t active_nodes;
        size_t total_entities;
        Real avg_load_factor;
        Real load_imbalance;  // std dev of load
        size_t migrations_completed;
        size_t migrations_pending;
    };
    PartitionStats get_statistics() const;
};

// Factory functions
std::unique_ptr<PartitionManager> create_partition_manager(
    const PartitionConfig& config);
std::unique_ptr<ISpatialPartitioner> create_octree_partitioner(
    const AABB& world_bounds, size_t max_entities_per_leaf);
std::unique_ptr<ILoadBalancer> create_load_balancer(
    LoadBalanceStrategy strategy);
```

---

## Distributed Time Management

### Header

```cpp
#include <jaguar/cloud/distributed_time.h>
```

### Core Types

```cpp
namespace jaguar::cloud {

// Raft consensus states
enum class RaftState {
    Follower,
    Candidate,
    Leader
};

// Time advancement modes
enum class TimeAdvancementMode {
    Conservative,   // Wait for all federates
    Optimistic,     // Advance and rollback if needed
    Bounded,        // Limited lookahead
    Adaptive        // Auto-selects based on federation
};

// Clock synchronization algorithms
enum class ClockSyncAlgorithm {
    NTP,            // Network Time Protocol
    PTP,            // Precision Time Protocol (IEEE 1588)
    Cristian,       // Cristian's algorithm
    Berkeley        // Berkeley algorithm
};

// Vector clock for causal ordering
class VectorClock {
public:
    void increment(const std::string& node_id);
    void update(const VectorClock& other);
    bool happens_before(const VectorClock& other) const;
    bool is_concurrent(const VectorClock& other) const;
    uint64_t get(const std::string& node_id) const;
    std::map<std::string, uint64_t> get_all() const;
};

// Timestamped event for ordering
struct TimestampedEvent {
    uint64_t event_id;
    std::string source_node;
    VectorClock vector_clock;
    uint64_t lamport_timestamp;
    std::chrono::system_clock::time_point wall_clock;
    std::vector<uint8_t> event_data;
};

// Barrier synchronization state
struct BarrierState {
    uint64_t barrier_id;
    std::set<std::string> expected_nodes;
    std::set<std::string> arrived_nodes;
    std::chrono::system_clock::time_point deadline;
    bool is_complete() const { return arrived_nodes == expected_nodes; }
};

// Time sync result
struct TimeSyncResult {
    bool success;
    std::string error_message;
    std::chrono::nanoseconds clock_offset;
    std::chrono::nanoseconds round_trip_time;
    double precision_estimate;
};

// Configuration
struct TimeSyncConfig {
    TimeAdvancementMode advancement_mode = TimeAdvancementMode::Conservative;
    ClockSyncAlgorithm sync_algorithm = ClockSyncAlgorithm::PTP;
    std::chrono::milliseconds sync_interval{100};
    std::chrono::milliseconds max_lookahead{1000};
    std::chrono::milliseconds barrier_timeout{5000};
    bool enable_vector_clocks = true;
    bool enable_lamport_clocks = true;
    size_t raft_election_timeout_min_ms = 150;
    size_t raft_election_timeout_max_ms = 300;
    size_t raft_heartbeat_interval_ms = 50;
};

}  // namespace jaguar::cloud
```

### DistributedTimeManager Class

```cpp
class DistributedTimeManager {
public:
    // Initialize time manager
    TimeSyncResult initialize(const TimeSyncConfig& config);

    // Shutdown
    void shutdown();

    // Join time federation
    TimeSyncResult join(const std::string& node_id,
                       const std::vector<std::string>& peer_nodes);

    // Leave time federation
    void leave();

    // Get current logical time
    uint64_t get_logical_time() const;

    // Get current simulation time
    std::chrono::nanoseconds get_simulation_time() const;

    // Advance time (may block for synchronization)
    TimeSyncResult advance_time(std::chrono::nanoseconds delta);

    // Request time advance (non-blocking)
    void request_time_advance(std::chrono::nanoseconds target_time);

    // Check if time advance is granted
    bool is_time_advance_granted() const;

    // Wait for time advance grant
    TimeSyncResult wait_for_time_advance(
        std::chrono::milliseconds timeout = std::chrono::milliseconds{5000});

    // Vector clock operations
    VectorClock get_vector_clock() const;
    void update_vector_clock(const VectorClock& received);

    // Lamport timestamp
    uint64_t get_lamport_timestamp() const;
    uint64_t increment_lamport_timestamp();
    void update_lamport_timestamp(uint64_t received);

    // Event ordering
    void submit_event(const TimestampedEvent& event);
    std::vector<TimestampedEvent> get_ordered_events();

    // Barrier synchronization
    uint64_t create_barrier(const std::set<std::string>& nodes,
                           std::chrono::milliseconds timeout);
    TimeSyncResult arrive_at_barrier(uint64_t barrier_id);
    TimeSyncResult wait_for_barrier(uint64_t barrier_id);

    // Raft consensus (for leader election)
    RaftState get_raft_state() const;
    std::string get_leader_id() const;
    bool is_leader() const;

    // Clock synchronization
    TimeSyncResult synchronize_clock();
    std::chrono::nanoseconds get_clock_offset() const;

    // Statistics
    struct TimeStats {
        uint64_t current_logical_time;
        std::chrono::nanoseconds current_sim_time;
        std::chrono::nanoseconds clock_offset;
        size_t time_advances;
        size_t barrier_syncs;
        size_t events_ordered;
        RaftState raft_state;
        std::string leader_id;
    };
    TimeStats get_statistics() const;
};

// Factory functions
std::unique_ptr<DistributedTimeManager> create_distributed_time_manager(
    const TimeSyncConfig& config);
std::unique_ptr<ITimeMaster> create_time_master(
    const std::string& node_id);
```

---

## Usage Examples

### Basic State Synchronization

```cpp
#include <jaguar/cloud/state_sync.h>

using namespace jaguar::cloud;

// Configure synchronizer
StateSyncConfig config;
config.strategy = SyncStrategy::Delta;
config.conflict_resolution = ConflictResolution::VectorClockBased;
config.sync_interval = std::chrono::milliseconds{50};
config.compression_enabled = true;

auto synchronizer = create_state_synchronizer(config);
synchronizer->initialize(config);

// Register callback for incoming state changes
synchronizer->set_state_change_callback([](const StateDelta& delta) {
    std::cout << "Received delta from version " << delta.base_version
              << " to " << delta.target_version << "\n";
    apply_delta(delta);
});

// In simulation loop
while (running) {
    engine.step(dt);

    // Sync local state
    auto state = engine.serialize_state();
    auto result = synchronizer->sync_state(state.data(), state.size());

    if (!result.success) {
        handle_sync_error(result.error_message);
    }
}
```

### Spatial Partitioning

```cpp
#include <jaguar/cloud/partition_manager.h>

using namespace jaguar::cloud;

// Define world bounds
AABB world_bounds;
world_bounds.min_corner = {-1000000, -1000000, -100000};  // 1000 km x 1000 km x 100 km
world_bounds.max_corner = {1000000, 1000000, 100000};

// Configure partition manager
PartitionConfig config;
config.strategy = PartitionStrategy::DomainThenSpatial;
config.load_balance_strategy = LoadBalanceStrategy::Adaptive;
config.world_bounds = world_bounds;
config.max_entities_per_partition = 1000;
config.rebalance_threshold = 0.2;  // Rebalance at 20% imbalance

auto manager = create_partition_manager(config);
manager->initialize(config);

// Register compute nodes
NodeInfo node1{"node-1", "192.168.1.10", 8080, 0.0, 0, 10000};
NodeInfo node2{"node-2", "192.168.1.11", 8080, 0.0, 0, 10000};
manager->register_node(node1);
manager->register_node(node2);

// Add entities
for (auto& entity : entities) {
    manager->add_entity(entity.id, entity.position, entity.domain);
}

// In simulation loop - update positions
for (auto& entity : entities) {
    manager->update_entity_position(entity.id, entity.position);
}

// Periodically rebalance
if (should_rebalance) {
    auto result = manager->rebalance();

    // Execute pending migrations
    for (auto& migration : manager->get_pending_migrations()) {
        manager->execute_migration(migration);
    }
}
```

### Distributed Time with Raft Consensus

```cpp
#include <jaguar/cloud/distributed_time.h>

using namespace jaguar::cloud;

// Configure time manager
TimeSyncConfig config;
config.advancement_mode = TimeAdvancementMode::Conservative;
config.sync_algorithm = ClockSyncAlgorithm::PTP;
config.enable_vector_clocks = true;
config.max_lookahead = std::chrono::milliseconds{100};

auto time_manager = create_distributed_time_manager(config);
time_manager->initialize(config);

// Join federation
std::vector<std::string> peers = {"node-2", "node-3", "node-4"};
time_manager->join("node-1", peers);

// Wait for leader election
while (time_manager->get_leader_id().empty()) {
    std::this_thread::sleep_for(std::chrono::milliseconds{10});
}
std::cout << "Leader elected: " << time_manager->get_leader_id() << "\n";

// Simulation loop with distributed time
std::chrono::nanoseconds dt{10000000};  // 10 ms

while (running) {
    // Request time advance
    time_manager->request_time_advance(
        time_manager->get_simulation_time() + dt);

    // Wait for grant (blocks until all nodes ready)
    auto result = time_manager->wait_for_time_advance();

    if (result.success) {
        // Execute simulation step
        engine.step(std::chrono::duration<double>(dt).count());

        // Advance time manager
        time_manager->advance_time(dt);
    }
}
```

### Barrier Synchronization

```cpp
// Create barrier for checkpoint
std::set<std::string> all_nodes = {"node-1", "node-2", "node-3"};
auto barrier_id = time_manager->create_barrier(
    all_nodes, std::chrono::milliseconds{10000});

// Each node arrives at barrier when ready
time_manager->arrive_at_barrier(barrier_id);

// Wait for all nodes
auto result = time_manager->wait_for_barrier(barrier_id);
if (result.success) {
    // All nodes synchronized, safe to checkpoint
    save_checkpoint();
}
```

---

## See Also

- [Architecture](../advanced/architecture.md) - System architecture overview
- [Network Integration](../advanced/networking.md) - DIS/HLA networking
- [Federation API](federation.md) - Federation protocols
- [Configuration](configuration.md) - Engine configuration
