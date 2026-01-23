/**
 * @file partition_tests.cpp
 * @brief Unit tests for the partition manager system
 */

#include <gtest/gtest.h>
#include "jaguar/cloud/partition_manager.h"
#include <random>
#include <thread>
#include <chrono>

using namespace jaguar;
using namespace jaguar::cloud;

// ============================================================================
// AABB Tests
// ============================================================================

class AABBTest : public ::testing::Test {
protected:
    void SetUp() override {
        aabb_ = AABB{Vec3{-10.0, -10.0, -10.0}, Vec3{10.0, 10.0, 10.0}};
    }

    AABB aabb_;
};

TEST_F(AABBTest, DefaultConstruction) {
    AABB box;
    EXPECT_DOUBLE_EQ(box.min.x, 0.0);
    EXPECT_DOUBLE_EQ(box.max.x, 0.0);
}

TEST_F(AABBTest, Center) {
    Vec3 center = aabb_.center();
    EXPECT_DOUBLE_EQ(center.x, 0.0);
    EXPECT_DOUBLE_EQ(center.y, 0.0);
    EXPECT_DOUBLE_EQ(center.z, 0.0);
}

TEST_F(AABBTest, HalfExtents) {
    Vec3 half = aabb_.half_extents();
    EXPECT_DOUBLE_EQ(half.x, 10.0);
    EXPECT_DOUBLE_EQ(half.y, 10.0);
    EXPECT_DOUBLE_EQ(half.z, 10.0);
}

TEST_F(AABBTest, Size) {
    Vec3 size = aabb_.size();
    EXPECT_DOUBLE_EQ(size.x, 20.0);
    EXPECT_DOUBLE_EQ(size.y, 20.0);
    EXPECT_DOUBLE_EQ(size.z, 20.0);
}

TEST_F(AABBTest, Volume) {
    Real volume = aabb_.volume();
    EXPECT_DOUBLE_EQ(volume, 8000.0);  // 20 * 20 * 20
}

TEST_F(AABBTest, ContainsPoint) {
    EXPECT_TRUE(aabb_.contains(Vec3{0.0, 0.0, 0.0}));
    EXPECT_TRUE(aabb_.contains(Vec3{5.0, 5.0, 5.0}));
    EXPECT_TRUE(aabb_.contains(Vec3{-10.0, -10.0, -10.0}));
    EXPECT_TRUE(aabb_.contains(Vec3{10.0, 10.0, 10.0}));

    EXPECT_FALSE(aabb_.contains(Vec3{15.0, 0.0, 0.0}));
    EXPECT_FALSE(aabb_.contains(Vec3{0.0, -15.0, 0.0}));
    EXPECT_FALSE(aabb_.contains(Vec3{0.0, 0.0, 100.0}));
}

TEST_F(AABBTest, Overlaps) {
    AABB other1{Vec3{5.0, 5.0, 5.0}, Vec3{15.0, 15.0, 15.0}};
    EXPECT_TRUE(aabb_.overlaps(other1));

    AABB other2{Vec3{20.0, 20.0, 20.0}, Vec3{30.0, 30.0, 30.0}};
    EXPECT_FALSE(aabb_.overlaps(other2));

    AABB other3{Vec3{-5.0, -5.0, -5.0}, Vec3{5.0, 5.0, 5.0}};
    EXPECT_TRUE(aabb_.overlaps(other3));
}

TEST_F(AABBTest, Expand) {
    AABB box{Vec3{0.0, 0.0, 0.0}, Vec3{10.0, 10.0, 10.0}};
    box.expand(Vec3{-5.0, 15.0, 5.0});

    EXPECT_DOUBLE_EQ(box.min.x, -5.0);
    EXPECT_DOUBLE_EQ(box.max.y, 15.0);
}

TEST_F(AABBTest, FromCenterHalfExtents) {
    AABB box = AABB::from_center_half_extents(Vec3{5.0, 5.0, 5.0}, Vec3{2.0, 2.0, 2.0});

    EXPECT_DOUBLE_EQ(box.min.x, 3.0);
    EXPECT_DOUBLE_EQ(box.max.x, 7.0);
}

TEST_F(AABBTest, EarthBounds) {
    AABB earth = AABB::earth_bounds();
    EXPECT_GT(earth.volume(), 0.0);
    EXPECT_TRUE(earth.contains(Vec3{0.0, 0.0, 0.0}));
}

// ============================================================================
// Partition Result Tests
// ============================================================================

TEST(PartitionResultTest, ToString) {
    EXPECT_STREQ(partition_result_to_string(PartitionResult::Success), "Success");
    EXPECT_STREQ(partition_result_to_string(PartitionResult::InvalidConfiguration),
                 "InvalidConfiguration");
    EXPECT_STREQ(partition_result_to_string(PartitionResult::MigrationFailed),
                 "MigrationFailed");
}

TEST(PartitionStrategyTest, ToString) {
    EXPECT_STREQ(partition_strategy_to_string(PartitionStrategy::Spatial), "Spatial");
    EXPECT_STREQ(partition_strategy_to_string(PartitionStrategy::Domain), "Domain");
    EXPECT_STREQ(partition_strategy_to_string(PartitionStrategy::DomainThenSpatial),
                 "DomainThenSpatial");
}

// ============================================================================
// Partition Config Tests
// ============================================================================

class PartitionConfigTest : public ::testing::Test {};

TEST_F(PartitionConfigTest, DefaultConfig) {
    PartitionConfig config = PartitionConfig::default_config();

    EXPECT_EQ(config.strategy, PartitionStrategy::Spatial);
    EXPECT_EQ(config.balance_strategy, LoadBalanceStrategy::EntityCount);
    EXPECT_EQ(config.max_octree_depth, 8u);
    EXPECT_TRUE(config.auto_balance);
    EXPECT_TRUE(config.predictive_migration);
}

TEST_F(PartitionConfigTest, SmallScaleConfig) {
    PartitionConfig config = PartitionConfig::small_scale();

    EXPECT_EQ(config.max_octree_depth, 4u);
    EXPECT_EQ(config.min_entities_per_partition, 10u);
    EXPECT_EQ(config.max_entities_per_partition, 1000u);
}

TEST_F(PartitionConfigTest, LargeScaleConfig) {
    PartitionConfig config = PartitionConfig::large_scale();

    EXPECT_EQ(config.max_octree_depth, 10u);
    EXPECT_EQ(config.min_entities_per_partition, 1000u);
    EXPECT_EQ(config.max_entities_per_partition, 50000u);
}

TEST_F(PartitionConfigTest, DomainBasedConfig) {
    PartitionConfig config = PartitionConfig::domain_based();

    EXPECT_EQ(config.strategy, PartitionStrategy::Domain);
    EXPECT_EQ(config.balance_strategy, LoadBalanceStrategy::CPULoad);
}

// ============================================================================
// Octree Partitioner Tests
// ============================================================================

class OctreePartitionerTest : public ::testing::Test {
protected:
    void SetUp() override {
        partitioner_ = create_octree_partitioner();
        AABB bounds{Vec3{-100.0, -100.0, -100.0}, Vec3{100.0, 100.0, 100.0}};
        EXPECT_EQ(partitioner_->initialize(bounds, 4), PartitionResult::Success);
    }

    std::unique_ptr<ISpatialPartitioner> partitioner_;
};

TEST_F(OctreePartitionerTest, InsertEntity) {
    EntityId entity1 = 1;
    Vec3 pos1{0.0, 0.0, 0.0};

    auto result = partitioner_->insert(entity1, pos1);
    EXPECT_EQ(result, PartitionResult::Success);

    PartitionId partition = partitioner_->find_partition(pos1);
    EXPECT_NE(partition, INVALID_PARTITION_ID);
}

TEST_F(OctreePartitionerTest, InsertMultipleEntities) {
    for (EntityId i = 1; i <= 100; ++i) {
        Vec3 pos{static_cast<Real>(i % 10) * 10.0 - 50.0,
                 static_cast<Real>(i / 10) * 10.0 - 50.0,
                 0.0};
        auto result = partitioner_->insert(i, pos);
        EXPECT_EQ(result, PartitionResult::Success);
    }
}

TEST_F(OctreePartitionerTest, InsertOutOfBounds) {
    EntityId entity1 = 1;
    Vec3 pos_outside{500.0, 0.0, 0.0};

    auto result = partitioner_->insert(entity1, pos_outside);
    EXPECT_EQ(result, PartitionResult::InvalidBounds);
}

TEST_F(OctreePartitionerTest, RemoveEntity) {
    EntityId entity1 = 1;
    Vec3 pos1{0.0, 0.0, 0.0};

    partitioner_->insert(entity1, pos1);
    auto result = partitioner_->remove(entity1);
    EXPECT_EQ(result, PartitionResult::Success);
}

TEST_F(OctreePartitionerTest, RemoveNonexistent) {
    auto result = partitioner_->remove(999);
    EXPECT_EQ(result, PartitionResult::EntityNotFound);
}

TEST_F(OctreePartitionerTest, UpdatePosition) {
    EntityId entity1 = 1;
    Vec3 pos1{0.0, 0.0, 0.0};
    Vec3 pos2{50.0, 50.0, 50.0};

    partitioner_->insert(entity1, pos1);

    auto result = partitioner_->update(entity1, pos2);
    EXPECT_EQ(result, PartitionResult::Success);
}

TEST_F(OctreePartitionerTest, FindPartition) {
    Vec3 pos{25.0, 25.0, 25.0};
    PartitionId partition = partitioner_->find_partition(pos);
    EXPECT_NE(partition, INVALID_PARTITION_ID);
}

TEST_F(OctreePartitionerTest, QueryRegion) {
    // Insert some entities
    for (EntityId i = 1; i <= 10; ++i) {
        Vec3 pos{static_cast<Real>(i) * 5.0, 0.0, 0.0};
        partitioner_->insert(i, pos);
    }

    // Query region containing some entities
    AABB region{Vec3{0.0, -10.0, -10.0}, Vec3{30.0, 10.0, 10.0}};
    auto entities = partitioner_->query_region(region);

    EXPECT_GE(entities.size(), 1u);
}

TEST_F(OctreePartitionerTest, QueryPartitions) {
    AABB region{Vec3{0.0, 0.0, 0.0}, Vec3{50.0, 50.0, 50.0}};
    auto partitions = partitioner_->query_partitions(region);

    EXPECT_GE(partitions.size(), 1u);
}

TEST_F(OctreePartitionerTest, GetPartitionBounds) {
    Vec3 pos{0.0, 0.0, 0.0};
    PartitionId partition = partitioner_->find_partition(pos);

    auto bounds = partitioner_->get_partition_bounds(partition);
    EXPECT_TRUE(bounds.has_value());
    EXPECT_TRUE(bounds->contains(pos));
}

TEST_F(OctreePartitionerTest, Split) {
    // Insert many entities to necessitate splitting
    for (EntityId i = 1; i <= 100; ++i) {
        Vec3 pos{static_cast<Real>(i % 10) * 2.0,
                 static_cast<Real>(i / 10) * 2.0,
                 0.0};
        partitioner_->insert(i, pos);
    }

    PartitionId partition = partitioner_->find_partition(Vec3{0.0, 0.0, 0.0});
    auto result = partitioner_->split(partition);
    EXPECT_EQ(result, PartitionResult::Success);

    // After split, should have more leaf partitions
    auto leaves = partitioner_->get_leaf_partitions();
    EXPECT_GE(leaves.size(), 8u);
}

TEST_F(OctreePartitionerTest, GetLeafPartitions) {
    auto leaves = partitioner_->get_leaf_partitions();
    EXPECT_GE(leaves.size(), 1u);
}

TEST_F(OctreePartitionerTest, GetEntityCount) {
    EntityId entity1 = 1;
    Vec3 pos1{0.0, 0.0, 0.0};

    partitioner_->insert(entity1, pos1);

    PartitionId partition = partitioner_->find_partition(pos1);
    UInt32 count = partitioner_->get_entity_count(partition);
    EXPECT_EQ(count, 1u);
}

// ============================================================================
// Domain Partitioner Tests
// ============================================================================

class DomainPartitionerTest : public ::testing::Test {
protected:
    void SetUp() override {
        partitioner_ = create_domain_partitioner();
        std::vector<Domain> domains = {
            Domain::Air, Domain::Land, Domain::Sea, Domain::Space
        };
        EXPECT_EQ(partitioner_->initialize(domains), PartitionResult::Success);
    }

    std::unique_ptr<IDomainPartitioner> partitioner_;
};

TEST_F(DomainPartitionerTest, AssignEntity) {
    EntityId entity1 = 1;
    auto result = partitioner_->assign(entity1, Domain::Air);
    EXPECT_EQ(result, PartitionResult::Success);
}

TEST_F(DomainPartitionerTest, AssignMultipleDomains) {
    partitioner_->assign(1, Domain::Air);
    partitioner_->assign(2, Domain::Land);
    partitioner_->assign(3, Domain::Sea);
    partitioner_->assign(4, Domain::Space);

    auto air_entities = partitioner_->get_entities(Domain::Air);
    EXPECT_EQ(air_entities.size(), 1u);

    auto land_entities = partitioner_->get_entities(Domain::Land);
    EXPECT_EQ(land_entities.size(), 1u);
}

TEST_F(DomainPartitionerTest, RemoveEntity) {
    partitioner_->assign(1, Domain::Air);
    auto result = partitioner_->remove(1);
    EXPECT_EQ(result, PartitionResult::Success);

    auto air_entities = partitioner_->get_entities(Domain::Air);
    EXPECT_EQ(air_entities.size(), 0u);
}

TEST_F(DomainPartitionerTest, GetDomainPartition) {
    PartitionId air_partition = partitioner_->get_domain_partition(Domain::Air);
    EXPECT_NE(air_partition, INVALID_PARTITION_ID);

    PartitionId land_partition = partitioner_->get_domain_partition(Domain::Land);
    EXPECT_NE(land_partition, INVALID_PARTITION_ID);
    EXPECT_NE(air_partition, land_partition);
}

TEST_F(DomainPartitionerTest, GetDomainCounts) {
    partitioner_->assign(1, Domain::Air);
    partitioner_->assign(2, Domain::Air);
    partitioner_->assign(3, Domain::Land);

    auto counts = partitioner_->get_domain_counts();
    EXPECT_EQ(counts[Domain::Air], 2u);
    EXPECT_EQ(counts[Domain::Land], 1u);
}

// ============================================================================
// Load Balancer Tests
// ============================================================================

class LoadBalancerTest : public ::testing::Test {
protected:
    void SetUp() override {
        balancer_ = create_load_balancer(LoadBalanceStrategy::EntityCount);

        std::vector<NodeInfo> nodes;
        for (UInt32 i = 0; i < 4; ++i) {
            NodeInfo node;
            node.id = i;
            node.is_online = true;
            node.entity_count = i * 1000;  // Uneven distribution
            nodes.push_back(node);
        }

        PartitionConfig config;
        EXPECT_EQ(balancer_->initialize(nodes, config), PartitionResult::Success);
    }

    std::unique_ptr<ILoadBalancer> balancer_;
};

TEST_F(LoadBalancerTest, CalculateImbalance) {
    Real imbalance = balancer_->calculate_imbalance();
    EXPECT_GT(imbalance, 0.0);  // Should be unbalanced
}

TEST_F(LoadBalancerTest, GetNodeLoads) {
    auto loads = balancer_->get_node_loads();
    EXPECT_EQ(loads.size(), 4u);
}

TEST_F(LoadBalancerTest, FindBestNode) {
    PartitionInfo partition;
    partition.entity_count = 100;

    NodeId best = balancer_->find_best_node_for_partition(partition);
    EXPECT_NE(best, INVALID_NODE_ID);
    EXPECT_EQ(best, 0u);  // Node 0 has lowest load
}

TEST_F(LoadBalancerTest, ShouldRebalance) {
    EXPECT_TRUE(balancer_->should_rebalance());
}

TEST_F(LoadBalancerTest, UpdateNodeMetrics) {
    NodeInfo updated_node;
    updated_node.id = 0;
    updated_node.is_online = true;
    updated_node.entity_count = 5000;

    balancer_->update_node_metrics(updated_node);

    auto loads = balancer_->get_node_loads();
    EXPECT_GT(loads[0], 0.0);
}

// ============================================================================
// Entity Migrator Tests
// ============================================================================

class EntityMigratorTest : public ::testing::Test {
protected:
    void SetUp() override {
        migrator_ = create_entity_migrator();
        PartitionConfig config;
        EXPECT_EQ(migrator_->initialize(config), PartitionResult::Success);
    }

    std::unique_ptr<IEntityMigrator> migrator_;
};

TEST_F(EntityMigratorTest, RequestMigration) {
    MigrationRequest request;
    request.entity_id = 1;
    request.source_partition = 1;
    request.target_partition = 2;
    request.priority = MigrationPriority::Normal;

    auto result = migrator_->request_migration(request);
    EXPECT_EQ(result, PartitionResult::Success);
    EXPECT_EQ(migrator_->pending_count(), 1u);
}

TEST_F(EntityMigratorTest, DuplicateMigration) {
    MigrationRequest request;
    request.entity_id = 1;
    request.source_partition = 1;
    request.target_partition = 2;

    migrator_->request_migration(request);
    auto result = migrator_->request_migration(request);
    EXPECT_EQ(result, PartitionResult::OperationInProgress);
}

TEST_F(EntityMigratorTest, CancelMigration) {
    MigrationRequest request;
    request.entity_id = 1;
    request.source_partition = 1;
    request.target_partition = 2;

    migrator_->request_migration(request);
    auto result = migrator_->cancel_migration(1);
    EXPECT_EQ(result, PartitionResult::Success);
    EXPECT_EQ(migrator_->pending_count(), 0u);
}

TEST_F(EntityMigratorTest, ProcessMigrations) {
    for (EntityId i = 1; i <= 5; ++i) {
        MigrationRequest request;
        request.entity_id = i;
        request.source_partition = 1;
        request.target_partition = 2;
        migrator_->request_migration(request);
    }

    EXPECT_EQ(migrator_->pending_count(), 5u);

    auto completed = migrator_->process_migrations(3);
    EXPECT_EQ(completed.size(), 3u);
    EXPECT_EQ(migrator_->pending_count(), 2u);
}

TEST_F(EntityMigratorTest, IsMigrating) {
    MigrationRequest request;
    request.entity_id = 1;
    request.source_partition = 1;
    request.target_partition = 2;

    migrator_->request_migration(request);
    EXPECT_TRUE(migrator_->is_migrating(1));
    EXPECT_FALSE(migrator_->is_migrating(2));
}

TEST_F(EntityMigratorTest, SerializeDeserialize) {
    EntityId entity_id = 42;
    physics::EntityState state;
    state.position = Vec3{100.0, 200.0, 300.0};
    state.velocity = Vec3{10.0, 20.0, 30.0};
    state.mass = 1000.0;

    auto data = migrator_->serialize_entity(entity_id, state);
    EXPECT_GT(data.size(), 0u);

    auto restored = migrator_->deserialize_entity(data);
    EXPECT_TRUE(restored.has_value());
    EXPECT_DOUBLE_EQ(restored->position.x, 100.0);
    EXPECT_DOUBLE_EQ(restored->mass, 1000.0);
}

TEST_F(EntityMigratorTest, MigrationPriority) {
    // Add low priority
    MigrationRequest low;
    low.entity_id = 1;
    low.priority = MigrationPriority::Low;
    migrator_->request_migration(low);

    // Add critical priority
    MigrationRequest critical;
    critical.entity_id = 2;
    critical.priority = MigrationPriority::Critical;
    migrator_->request_migration(critical);

    // Process one - should be critical first
    auto completed = migrator_->process_migrations(1);
    EXPECT_EQ(completed.size(), 1u);
    EXPECT_EQ(completed[0].entity_id, 2u);  // Critical processed first
}

// ============================================================================
// Predictive Migrator Tests
// ============================================================================

class PredictiveMigratorTest : public ::testing::Test {
protected:
    void SetUp() override {
        migrator_ = create_predictive_migrator();
    }

    std::unique_ptr<IPredictiveMigrator> migrator_;
};

TEST_F(PredictiveMigratorTest, UpdatePrediction) {
    EntityId entity_id = 1;
    Vec3 position{0.0, 0.0, 0.0};
    Vec3 velocity{10.0, 0.0, 0.0};
    Vec3 acceleration{0.0, 0.0, 0.0};

    // Should not throw
    migrator_->update_prediction(entity_id, position, velocity, acceleration);
}

TEST_F(PredictiveMigratorTest, PredictPosition) {
    EntityId entity_id = 1;
    Vec3 position{0.0, 0.0, 0.0};
    Vec3 velocity{10.0, 0.0, 0.0};
    Vec3 acceleration{0.0, 0.0, 0.0};

    migrator_->update_prediction(entity_id, position, velocity, acceleration);

    Vec3 predicted = migrator_->predict_position(entity_id, 5.0);
    EXPECT_DOUBLE_EQ(predicted.x, 50.0);  // 10 * 5 = 50
    EXPECT_DOUBLE_EQ(predicted.y, 0.0);
    EXPECT_DOUBLE_EQ(predicted.z, 0.0);
}

TEST_F(PredictiveMigratorTest, PredictWithAcceleration) {
    EntityId entity_id = 1;
    Vec3 position{0.0, 0.0, 0.0};
    Vec3 velocity{0.0, 0.0, 0.0};
    Vec3 acceleration{2.0, 0.0, 0.0};  // 2 m/s^2

    migrator_->update_prediction(entity_id, position, velocity, acceleration);

    Vec3 predicted = migrator_->predict_position(entity_id, 4.0);
    // p = 0 + 0*4 + 0.5*2*16 = 16
    EXPECT_DOUBLE_EQ(predicted.x, 16.0);
}

TEST_F(PredictiveMigratorTest, ClearPrediction) {
    EntityId entity_id = 1;
    migrator_->update_prediction(entity_id, Vec3{0, 0, 0}, Vec3{10, 0, 0}, Vec3{0, 0, 0});
    migrator_->clear_prediction(entity_id);

    Vec3 predicted = migrator_->predict_position(entity_id, 5.0);
    EXPECT_DOUBLE_EQ(predicted.x, 0.0);  // Should return zero for cleared entity
}

// ============================================================================
// Partition Manager Tests
// ============================================================================

class PartitionManagerTest : public ::testing::Test {
protected:
    void SetUp() override {
        manager_ = std::make_unique<PartitionManager>();
        config_ = PartitionConfig::default_config();
        config_.world_bounds = AABB{Vec3{-1000, -1000, -1000}, Vec3{1000, 1000, 1000}};
        config_.max_octree_depth = 4;

        EXPECT_EQ(manager_->initialize(config_), PartitionResult::Success);
    }

    void TearDown() override {
        manager_->shutdown();
    }

    std::unique_ptr<PartitionManager> manager_;
    PartitionConfig config_;
};

TEST_F(PartitionManagerTest, Initialize) {
    EXPECT_TRUE(manager_->is_initialized());
}

TEST_F(PartitionManagerTest, DoubleInitialize) {
    auto result = manager_->initialize(config_);
    EXPECT_EQ(result, PartitionResult::AlreadyInitialized);
}

TEST_F(PartitionManagerTest, GetConfig) {
    const auto& config = manager_->get_config();
    EXPECT_EQ(config.strategy, PartitionStrategy::Spatial);
}

TEST_F(PartitionManagerTest, RegisterNode) {
    NodeInfo node;
    node.id = 1;
    node.hostname = "node1";
    node.is_online = true;

    auto result = manager_->register_node(node);
    EXPECT_EQ(result, PartitionResult::Success);

    auto retrieved = manager_->get_node(1);
    EXPECT_TRUE(retrieved.has_value());
    EXPECT_EQ(retrieved->hostname, "node1");
}

TEST_F(PartitionManagerTest, UnregisterNode) {
    NodeInfo node;
    node.id = 1;
    node.is_online = true;

    manager_->register_node(node);
    auto result = manager_->unregister_node(1);
    EXPECT_EQ(result, PartitionResult::Success);

    auto retrieved = manager_->get_node(1);
    EXPECT_FALSE(retrieved.has_value());
}

TEST_F(PartitionManagerTest, GetAllNodes) {
    NodeInfo node1, node2;
    node1.id = 1;
    node2.id = 2;

    manager_->register_node(node1);
    manager_->register_node(node2);

    auto nodes = manager_->get_all_nodes();
    EXPECT_GE(nodes.size(), 2u);  // Including local node
}

TEST_F(PartitionManagerTest, LocalNodeId) {
    EXPECT_EQ(manager_->get_local_node_id(), LOCAL_NODE_ID);

    manager_->set_local_node_id(5);
    EXPECT_EQ(manager_->get_local_node_id(), 5u);
}

TEST_F(PartitionManagerTest, AssignEntity) {
    EntityId entity_id = 1;
    Vec3 position{100.0, 100.0, 100.0};

    auto result = manager_->assign_entity(entity_id, position, Domain::Air);
    EXPECT_EQ(result, PartitionResult::Success);

    auto location = manager_->get_entity_location(entity_id);
    EXPECT_TRUE(location.has_value());
    EXPECT_EQ(location->entity_id, entity_id);
}

TEST_F(PartitionManagerTest, RemoveEntity) {
    EntityId entity_id = 1;
    Vec3 position{100.0, 100.0, 100.0};

    manager_->assign_entity(entity_id, position);
    auto result = manager_->remove_entity(entity_id);
    EXPECT_EQ(result, PartitionResult::Success);

    auto location = manager_->get_entity_location(entity_id);
    EXPECT_FALSE(location.has_value());
}

TEST_F(PartitionManagerTest, UpdateEntityPosition) {
    EntityId entity_id = 1;
    Vec3 position1{100.0, 100.0, 100.0};
    Vec3 position2{-100.0, -100.0, -100.0};

    manager_->assign_entity(entity_id, position1);
    auto result = manager_->update_entity_position(entity_id, position2);
    EXPECT_EQ(result, PartitionResult::Success);

    auto location = manager_->get_entity_location(entity_id);
    EXPECT_DOUBLE_EQ(location->position.x, -100.0);
}

TEST_F(PartitionManagerTest, IsLocalEntity) {
    EntityId entity_id = 1;
    Vec3 position{100.0, 100.0, 100.0};

    manager_->assign_entity(entity_id, position);
    EXPECT_TRUE(manager_->is_local_entity(entity_id));
}

TEST_F(PartitionManagerTest, GetEntitiesInPartition) {
    Vec3 position{100.0, 100.0, 100.0};
    manager_->assign_entity(1, position);
    manager_->assign_entity(2, position);

    auto location = manager_->get_entity_location(1);
    ASSERT_TRUE(location.has_value());

    auto entities = manager_->get_entities_in_partition(location->partition_id);
    EXPECT_GE(entities.size(), 2u);
}

TEST_F(PartitionManagerTest, GetLocalEntities) {
    manager_->assign_entity(1, Vec3{100, 100, 100});
    manager_->assign_entity(2, Vec3{-100, -100, -100});

    auto entities = manager_->get_local_entities();
    EXPECT_EQ(entities.size(), 2u);
}

TEST_F(PartitionManagerTest, GetAllPartitions) {
    manager_->assign_entity(1, Vec3{100, 100, 100});

    auto partitions = manager_->get_all_partitions();
    EXPECT_GE(partitions.size(), 1u);
}

TEST_F(PartitionManagerTest, QueryPartitions) {
    AABB region{Vec3{0, 0, 0}, Vec3{500, 500, 500}};
    auto partitions = manager_->query_partitions(region);
    EXPECT_GE(partitions.size(), 1u);
}

TEST_F(PartitionManagerTest, RequestMigration) {
    manager_->assign_entity(1, Vec3{100, 100, 100});

    auto location = manager_->get_entity_location(1);
    ASSERT_TRUE(location.has_value());

    // Request migration to a different partition
    auto result = manager_->request_migration(1, location->partition_id + 1);
    // May fail if partition doesn't exist, but should not crash
    EXPECT_TRUE(result == PartitionResult::Success ||
                result == PartitionResult::PartitionNotFound);
}

TEST_F(PartitionManagerTest, PendingMigrationCount) {
    EXPECT_EQ(manager_->pending_migration_count(), 0u);
}

TEST_F(PartitionManagerTest, ProcessMigrations) {
    // Should not throw even with no migrations
    manager_->process_migrations();
}

TEST_F(PartitionManagerTest, UpdateEntityTrajectory) {
    manager_->assign_entity(1, Vec3{0, 0, 0});

    // Should not throw
    manager_->update_entity_trajectory(1, Vec3{0, 0, 0}, Vec3{10, 0, 0}, Vec3{0, 0, 0});
}

TEST_F(PartitionManagerTest, SetPredictiveMigration) {
    manager_->set_predictive_migration(false);
    manager_->set_predictive_migration(true);
    // Should not throw
}

TEST_F(PartitionManagerTest, Update) {
    manager_->assign_entity(1, Vec3{0, 0, 0});

    // Should not throw
    manager_->update(0.016);  // ~60 fps
}

TEST_F(PartitionManagerTest, GetStats) {
    manager_->assign_entity(1, Vec3{0, 0, 0});
    manager_->assign_entity(2, Vec3{100, 100, 100});

    auto stats = manager_->get_stats();
    EXPECT_EQ(stats.total_entities, 2u);
}

TEST_F(PartitionManagerTest, ResetStats) {
    manager_->assign_entity(1, Vec3{0, 0, 0});
    manager_->reset_stats();

    auto stats = manager_->get_stats();
    EXPECT_EQ(stats.total_migrations, 0u);
}

TEST_F(PartitionManagerTest, GetLoadImbalance) {
    Real imbalance = manager_->get_load_imbalance();
    EXPECT_GE(imbalance, 0.0);
    EXPECT_LE(imbalance, 1.0);
}

TEST_F(PartitionManagerTest, Callbacks) {
    bool entity_assigned = false;
    bool entity_migrated = false;

    manager_->set_entity_assigned_callback(
        [&](EntityId, PartitionId, NodeId) {
            entity_assigned = true;
        });

    manager_->set_entity_migrated_callback(
        [&](const MigrationRequest&) {
            entity_migrated = true;
        });

    manager_->assign_entity(1, Vec3{0, 0, 0});
    EXPECT_TRUE(entity_assigned);
}

// ============================================================================
// Domain Strategy Test
// ============================================================================

TEST(PartitionManagerDomainTest, DomainBasedPartitioning) {
    PartitionManager manager;
    auto config = PartitionConfig::domain_based();
    config.world_bounds = AABB{Vec3{-1000, -1000, -1000}, Vec3{1000, 1000, 1000}};

    ASSERT_EQ(manager.initialize(config), PartitionResult::Success);

    // Assign entities to different domains
    manager.assign_entity(1, Vec3{0, 0, 0}, Domain::Air);
    manager.assign_entity(2, Vec3{0, 0, 0}, Domain::Land);
    manager.assign_entity(3, Vec3{0, 0, 0}, Domain::Sea);

    // Each should be in a different partition based on domain
    auto loc1 = manager.get_entity_location(1);
    auto loc2 = manager.get_entity_location(2);
    auto loc3 = manager.get_entity_location(3);

    EXPECT_TRUE(loc1.has_value());
    EXPECT_TRUE(loc2.has_value());
    EXPECT_TRUE(loc3.has_value());

    // Different domains should have different partitions
    EXPECT_NE(loc1->partition_id, loc2->partition_id);
    EXPECT_NE(loc2->partition_id, loc3->partition_id);

    manager.shutdown();
}

// ============================================================================
// Performance Tests
// ============================================================================

TEST(PartitionManagerPerformanceTest, BulkEntityAssignment) {
    PartitionManager manager;
    auto config = PartitionConfig::large_scale();
    config.world_bounds = AABB{Vec3{-10000, -10000, -10000}, Vec3{10000, 10000, 10000}};

    ASSERT_EQ(manager.initialize(config), PartitionResult::Success);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<Real> dist(-9000.0, 9000.0);

    auto start = std::chrono::high_resolution_clock::now();

    const UInt32 entity_count = 10000;
    for (EntityId i = 1; i <= entity_count; ++i) {
        Vec3 pos{dist(gen), dist(gen), dist(gen)};
        manager.assign_entity(i, pos);
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    std::cout << "Assigned " << entity_count << " entities in "
              << duration.count() << " ms" << std::endl;

    auto stats = manager.get_stats();
    EXPECT_EQ(stats.total_entities, entity_count);

    // Should complete in reasonable time (< 5 seconds)
    EXPECT_LT(duration.count(), 5000);

    manager.shutdown();
}

TEST(PartitionManagerPerformanceTest, UpdatePerformance) {
    PartitionManager manager;
    auto config = PartitionConfig::default_config();
    config.world_bounds = AABB{Vec3{-10000, -10000, -10000}, Vec3{10000, 10000, 10000}};

    ASSERT_EQ(manager.initialize(config), PartitionResult::Success);

    // Assign 1000 entities
    for (EntityId i = 1; i <= 1000; ++i) {
        Vec3 pos{static_cast<Real>(i), 0.0, 0.0};
        manager.assign_entity(i, pos);
    }

    // Measure update performance
    auto start = std::chrono::high_resolution_clock::now();

    for (int frame = 0; frame < 100; ++frame) {
        manager.update(0.016);  // 60 fps
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    std::cout << "100 update frames in " << duration.count() << " ms" << std::endl;

    // Should be fast enough for real-time (< 1 second for 100 frames)
    EXPECT_LT(duration.count(), 1000);

    manager.shutdown();
}

// ============================================================================
// Thread Safety Tests
// ============================================================================

TEST(PartitionManagerThreadTest, ConcurrentAccess) {
    PartitionManager manager;
    auto config = PartitionConfig::default_config();
    config.world_bounds = AABB{Vec3{-10000, -10000, -10000}, Vec3{10000, 10000, 10000}};

    ASSERT_EQ(manager.initialize(config), PartitionResult::Success);

    std::atomic<UInt32> assigned_count{0};
    std::atomic<bool> running{true};

    // Writer thread - assigns entities
    std::thread writer([&]() {
        for (EntityId i = 1; i <= 100 && running; ++i) {
            Vec3 pos{static_cast<Real>(i), 0.0, 0.0};
            if (manager.assign_entity(i, pos) == PartitionResult::Success) {
                assigned_count++;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    // Reader thread - reads stats
    std::thread reader([&]() {
        for (int i = 0; i < 100 && running; ++i) {
            auto stats = manager.get_stats();
            (void)stats;  // Suppress unused warning
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    writer.join();
    running = false;
    reader.join();

    EXPECT_GT(assigned_count.load(), 0u);

    manager.shutdown();
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST(PartitionManagerEdgeCases, Uninitialized) {
    PartitionManager manager;

    auto result = manager.assign_entity(1, Vec3{0, 0, 0});
    EXPECT_EQ(result, PartitionResult::NotInitialized);
}

TEST(PartitionManagerEdgeCases, EmptyWorld) {
    PartitionManager manager;
    PartitionConfig config;
    config.world_bounds = AABB{Vec3{0, 0, 0}, Vec3{0, 0, 0}};

    auto result = manager.initialize(config);
    EXPECT_EQ(result, PartitionResult::Success);

    manager.shutdown();
}

TEST(PartitionManagerEdgeCases, ShutdownAndReinit) {
    PartitionManager manager;
    PartitionConfig config;
    config.world_bounds = AABB{Vec3{-100, -100, -100}, Vec3{100, 100, 100}};

    ASSERT_EQ(manager.initialize(config), PartitionResult::Success);
    manager.shutdown();

    EXPECT_FALSE(manager.is_initialized());

    // Can reinitialize after shutdown
    EXPECT_EQ(manager.initialize(config), PartitionResult::Success);
    EXPECT_TRUE(manager.is_initialized());

    manager.shutdown();
}
