# Cloud Module - Distributed Simulation Infrastructure

## Overview

The Cloud module provides distributed simulation infrastructure for JaguarEngine, enabling large-scale simulations with 100,000+ entities across multiple compute nodes. It implements sophisticated partitioning strategies, load balancing, and entity migration to optimize performance in distributed environments.

## Key Features

- **Spatial Partitioning**: Octree-based spatial subdivision for geographic locality
- **Domain Partitioning**: Physics domain-based entity grouping (Air, Land, Sea, Space)
- **Hybrid Strategies**: Combined spatial and domain partitioning approaches
- **Dynamic Load Balancing**: Real-time workload distribution optimization
- **Entity Migration**: Seamless entity transfer between partitions and nodes
- **Predictive Migration**: Trajectory-based prediction for proactive entity transfer
- **Multi-Node Support**: Distributed simulation across multiple compute nodes

## Architecture

### Core Components

```
┌─────────────────────────────────────────────────────────────────┐
│                     PartitionManager                            │
├─────────────────────────────────────────────────────────────────┤
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐  │
│  │   Spatial    │  │    Domain    │  │    Load Balancer     │  │
│  │ Partitioner  │  │ Partitioner  │  │                      │  │
│  │   (Octree)   │  │              │  │                      │  │
│  └──────────────┘  └──────────────┘  └──────────────────────┘  │
│  ┌──────────────┐  ┌──────────────────────────────────────────┐│
│  │   Entity     │  │          Predictive Migrator             ││
│  │  Migrator    │  │                                          ││
│  └──────────────┘  └──────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────┘
```

## API Reference

### PartitionManager

The central coordinator for all partitioning operations.

```cpp
#include <jaguar/cloud/partition_manager.h>

using namespace jaguar::cloud;

// Create with default configuration
auto manager = create_partition_manager(PartitionConfig::default_config());

// Create with custom configuration
PartitionConfig config;
config.strategy = PartitionStrategy::DomainThenSpatial;
config.balance_strategy = LoadBalanceStrategy::Weighted;
config.world_bounds = AABB::earth_bounds();
auto manager = create_partition_manager(config);
```

### Partition Strategies

```cpp
enum class PartitionStrategy : UInt8 {
    Spatial,           // Octree-based geographic partitioning
    Domain,            // Physics domain grouping (Air/Land/Sea/Space)
    DomainThenSpatial, // Domain first, then spatial subdivision
    SpatialThenDomain, // Spatial first, then domain grouping
    RoundRobin,        // Simple distribution for load testing
    HashBased,         // Consistent hash-based distribution
    Custom             // User-defined strategy
};
```

### Load Balance Strategies

```cpp
enum class LoadBalanceStrategy : UInt8 {
    None,         // No automatic balancing
    EntityCount,  // Balance by entity count
    CPULoad,      // Balance by CPU utilization
    MemoryUsage,  // Balance by memory consumption
    Weighted,     // Weighted combination of factors
    Custom        // User-defined balancing
};
```

### Entity Operations

```cpp
// Assign an entity to a partition
PartitionResult result = manager->assign_entity(
    entity_id,                    // Unique entity identifier
    Vec3{100.0, 200.0, 50.0},    // Position
    Domain::Air                   // Physics domain
);

// Update entity position
result = manager->update_entity_position(
    entity_id,
    Vec3{150.0, 220.0, 60.0}     // New position
);

// Get entity location
std::optional<EntityLocation> location = manager->get_entity_location(entity_id);
if (location) {
    std::cout << "Partition: " << location->partition_id << "\n";
    std::cout << "Node: " << location->node_id << "\n";
}

// Remove entity
result = manager->remove_entity(entity_id);
```

### Node Management

```cpp
// Register a compute node
NodeInfo node;
node.id = "node-001";
node.hostname = "compute01.cluster.local";
node.port = 9000;
node.cpu_cores = 16;
node.memory_mb = 65536;
node.is_active = true;

PartitionResult result = manager->register_node(node);

// Unregister a node
result = manager->unregister_node("node-001");

// Get active nodes
std::vector<NodeInfo> nodes = manager->get_nodes();

// Check node health
bool healthy = manager->is_node_healthy("node-001");
```

### Partition Operations

```cpp
// Get all partitions
std::vector<PartitionInfo> partitions = manager->get_partitions();

// Get partition info
std::optional<PartitionInfo> info = manager->get_partition_info(partition_id);

// Query entities in a region
AABB region{
    Vec3{0.0, 0.0, 0.0},      // Min corner
    Vec3{1000.0, 1000.0, 500.0} // Max corner
};
std::vector<EntityId> entities = manager->query_region(region);
```

### Migration Operations

```cpp
// Request entity migration with priority
MigrationRequest request;
request.entity_id = entity_id;
request.source_partition = current_partition;
request.target_partition = target_partition;
request.priority = MigrationPriority::High;

PartitionResult result = manager->request_migration(request);

// Process pending migrations
UInt32 processed = manager->process_pending_migrations(100); // Max 100 migrations

// Get pending migration count
UInt32 pending = manager->get_pending_migration_count();
```

### Load Balancing

```cpp
// Calculate current imbalance factor (0.0 = perfect balance, 1.0 = max imbalance)
Float64 imbalance = manager->calculate_load_imbalance();

// Trigger load balancing
if (imbalance > 0.2) { // 20% imbalance threshold
    manager->trigger_load_balance();
}
```

### Predictive Migration

```cpp
// Enable/disable predictive migration
manager->set_predictive_migration_enabled(true);

// Update entity velocity for prediction
manager->update_entity_velocity(entity_id, Vec3{100.0, 50.0, 10.0});

// Configure prediction parameters in config
PartitionConfig config;
config.enable_predictive_migration = true;
config.prediction_horizon_seconds = 10.0;     // Look ahead 10 seconds
config.velocity_decay_factor = 0.95;          // Velocity dampening
```

### Statistics

```cpp
// Get partition statistics
PartitionStats stats = manager->get_statistics();

std::cout << "Total entities: " << stats.total_entities << "\n";
std::cout << "Total partitions: " << stats.total_partitions << "\n";
std::cout << "Active nodes: " << stats.active_nodes << "\n";
std::cout << "Migrations: " << stats.migrations_completed << "\n";
std::cout << "Avg entities/partition: " << stats.average_entities_per_partition << "\n";
```

## Configuration

### PartitionConfig

```cpp
struct PartitionConfig {
    // Strategy settings
    PartitionStrategy strategy{PartitionStrategy::Spatial};
    LoadBalanceStrategy balance_strategy{LoadBalanceStrategy::EntityCount};

    // World bounds
    AABB world_bounds{AABB::earth_bounds()};

    // Octree settings
    UInt32 max_octree_depth{8};              // Maximum tree depth
    UInt32 max_entities_per_leaf{1000};      // Split threshold
    UInt32 min_entities_per_leaf{100};       // Merge threshold

    // Load balancing
    Float64 load_imbalance_threshold{0.2};   // 20% threshold
    Float64 migration_cooldown_seconds{5.0}; // Min time between balances

    // Predictive migration
    bool enable_predictive_migration{true};
    Float64 prediction_horizon_seconds{10.0};
    Float64 velocity_decay_factor{0.95};
    Float64 boundary_margin{100.0};          // Trigger margin in meters

    // Performance settings
    UInt32 max_migrations_per_tick{100};
    UInt32 node_heartbeat_interval_ms{1000};
    UInt32 node_timeout_ms{5000};

    // Factory methods
    static PartitionConfig default_config() noexcept;
    static PartitionConfig small_scale() noexcept;    // < 10K entities
    static PartitionConfig large_scale() noexcept;    // > 100K entities
    static PartitionConfig domain_based() noexcept;   // Domain-focused
};
```

### Preset Configurations

```cpp
// Small-scale simulation (< 10,000 entities)
auto config = PartitionConfig::small_scale();
// - Lower octree depth
// - Higher entities per leaf
// - Basic load balancing

// Large-scale simulation (> 100,000 entities)
auto config = PartitionConfig::large_scale();
// - Deep octree
// - Aggressive load balancing
// - Predictive migration enabled

// Domain-based simulation
auto config = PartitionConfig::domain_based();
// - Domain-then-spatial strategy
// - Optimized for physics domain separation
```

## Best Practices

### 1. Choose the Right Strategy

```cpp
// Geographic clustering (e.g., regional conflicts)
config.strategy = PartitionStrategy::Spatial;

// Domain-specific processing (e.g., separate air/ground physics)
config.strategy = PartitionStrategy::Domain;

// Large-scale mixed environments
config.strategy = PartitionStrategy::DomainThenSpatial;
```

### 2. Tune Load Balancing

```cpp
// For stable simulations with predictable patterns
config.load_imbalance_threshold = 0.3;  // Higher tolerance
config.migration_cooldown_seconds = 10.0;

// For dynamic simulations with rapid changes
config.load_imbalance_threshold = 0.1;  // Tighter balance
config.migration_cooldown_seconds = 2.0;
```

### 3. Optimize Predictive Migration

```cpp
// For fast-moving entities (aircraft, missiles)
config.prediction_horizon_seconds = 15.0;
config.boundary_margin = 500.0;  // Larger margin

// For slow-moving entities (ground vehicles)
config.prediction_horizon_seconds = 5.0;
config.boundary_margin = 50.0;
```

### 4. Handle Migration Gracefully

```cpp
// Process migrations in batches to avoid stuttering
void simulation_tick() {
    // Limit migrations per frame
    manager->process_pending_migrations(10);

    // Check for load balance periodically
    if (tick_count % 100 == 0) {
        if (manager->calculate_load_imbalance() > 0.2) {
            manager->trigger_load_balance();
        }
    }
}
```

## Integration Example

```cpp
#include <jaguar/cloud/partition_manager.h>
#include <jaguar/physics/entity.h>

using namespace jaguar;
using namespace jaguar::cloud;

class DistributedSimulation {
public:
    void initialize() {
        // Configure for large-scale simulation
        auto config = PartitionConfig::large_scale();
        config.strategy = PartitionStrategy::DomainThenSpatial;

        partition_manager_ = create_partition_manager(config);

        // Register compute nodes
        for (const auto& node : compute_cluster_) {
            partition_manager_->register_node(node);
        }
    }

    void add_entity(Entity& entity) {
        auto result = partition_manager_->assign_entity(
            entity.id(),
            entity.position(),
            entity.physics_domain()
        );

        if (result != PartitionResult::Success) {
            handle_assignment_error(result);
        }
    }

    void update(Float64 dt) {
        // Update entity positions in partition manager
        for (auto& entity : entities_) {
            partition_manager_->update_entity_position(
                entity.id(),
                entity.position()
            );

            if (entity.has_velocity()) {
                partition_manager_->update_entity_velocity(
                    entity.id(),
                    entity.velocity()
                );
            }
        }

        // Process migrations
        partition_manager_->process_pending_migrations(50);

        // Periodic load balancing
        if (should_balance()) {
            partition_manager_->trigger_load_balance();
        }
    }

    void shutdown() {
        partition_manager_->shutdown();
    }

private:
    std::unique_ptr<PartitionManager> partition_manager_;
    std::vector<NodeInfo> compute_cluster_;
    std::vector<Entity> entities_;
};
```

## Performance Considerations

### Memory Usage

- Octree nodes: ~200 bytes per node
- Entity tracking: ~64 bytes per entity
- Migration queue: ~128 bytes per pending migration

### Time Complexity

| Operation | Complexity |
|-----------|------------|
| Entity assignment | O(log n) octree depth |
| Position update | O(log n) |
| Region query | O(k + log n) where k = results |
| Load balance | O(n) entities |
| Migration | O(1) per entity |

### Scaling Guidelines

| Entity Count | Recommended Strategy | Notes |
|--------------|---------------------|-------|
| < 1,000 | Spatial | Simple, low overhead |
| 1,000 - 10,000 | Spatial or Domain | Based on clustering |
| 10,000 - 100,000 | DomainThenSpatial | Hybrid approach |
| > 100,000 | DomainThenSpatial + aggressive balancing | Full distributed |

## Thread Safety

The PartitionManager is thread-safe for all public operations:

- Entity operations (assign, update, remove) are mutex-protected
- Partition queries use read locks for concurrent access
- Migration processing is serialized for consistency
- Statistics collection is atomic

## Error Handling

```cpp
PartitionResult result = manager->assign_entity(entity_id, position, domain);

switch (result) {
    case PartitionResult::Success:
        // Entity assigned successfully
        break;
    case PartitionResult::EntityAlreadyExists:
        // Entity is already tracked
        break;
    case PartitionResult::OutOfBounds:
        // Position outside world bounds
        break;
    case PartitionResult::NoAvailableNode:
        // No compute nodes registered
        break;
    case PartitionResult::PartitionFull:
        // Partition at capacity
        break;
    case PartitionResult::InvalidConfig:
        // Configuration error
        break;
    default:
        // Handle other errors
        break;
}
```

## See Also

- [Physics Module](PHYSICS.md) - Entity physics integration
- [Core Module](CORE.md) - Basic types and utilities
- [API Reference](../API_REFERENCE.md) - Complete API documentation
