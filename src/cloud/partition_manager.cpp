/**
 * @file partition_manager.cpp
 * @brief Implementation of entity partitioning system for distributed simulation
 */

#include "jaguar/cloud/partition_manager.h"
#include <algorithm>
#include <queue>
#include <cmath>
#include <mutex>
#include <atomic>
#include <cstring>

namespace jaguar::cloud {

// ============================================================================
// Octree Partitioner Implementation
// ============================================================================

class OctreePartitioner : public ISpatialPartitioner {
public:
    OctreePartitioner() = default;
    ~OctreePartitioner() override = default;

    PartitionResult initialize(const AABB& world_bounds, UInt32 max_depth) override {
        world_bounds_ = world_bounds;
        max_depth_ = max_depth;
        next_partition_id_ = 1;

        // Create root node
        root_ = std::make_unique<OctreeNode>();
        root_->partition_id = next_partition_id_++;
        root_->bounds = world_bounds;
        root_->depth = 0;
        root_->is_leaf = true;

        partition_nodes_[root_->partition_id] = root_.get();

        return PartitionResult::Success;
    }

    PartitionResult insert(EntityId entity_id, const Vec3& position) override {
        if (!root_) {
            return PartitionResult::NotInitialized;
        }

        if (!world_bounds_.contains(position)) {
            return PartitionResult::InvalidBounds;
        }

        // Find or create appropriate leaf node
        OctreeNode* node = find_or_create_leaf(position);
        if (!node) {
            return PartitionResult::OutOfMemory;
        }

        node->entities.insert(entity_id);
        entity_positions_[entity_id] = position;
        entity_partitions_[entity_id] = node->partition_id;

        return PartitionResult::Success;
    }

    PartitionResult remove(EntityId entity_id) override {
        auto it = entity_partitions_.find(entity_id);
        if (it == entity_partitions_.end()) {
            return PartitionResult::EntityNotFound;
        }

        PartitionId partition_id = it->second;
        auto node_it = partition_nodes_.find(partition_id);
        if (node_it != partition_nodes_.end()) {
            node_it->second->entities.erase(entity_id);
        }

        entity_partitions_.erase(it);
        entity_positions_.erase(entity_id);

        return PartitionResult::Success;
    }

    PartitionResult update(EntityId entity_id, const Vec3& new_position) override {
        auto it = entity_partitions_.find(entity_id);
        if (it == entity_partitions_.end()) {
            return PartitionResult::EntityNotFound;
        }

        if (!world_bounds_.contains(new_position)) {
            return PartitionResult::InvalidBounds;
        }

        PartitionId old_partition = it->second;
        auto node_it = partition_nodes_.find(old_partition);

        // Check if position is still in same partition
        if (node_it != partition_nodes_.end()) {
            OctreeNode* old_node = node_it->second;
            if (old_node->bounds.contains(new_position)) {
                // Still in same partition, just update position
                entity_positions_[entity_id] = new_position;
                return PartitionResult::Success;
            }
        }

        // Need to move to different partition
        remove(entity_id);
        return insert(entity_id, new_position);
    }

    PartitionId find_partition(const Vec3& position) const override {
        if (!root_ || !world_bounds_.contains(position)) {
            return INVALID_PARTITION_ID;
        }

        const OctreeNode* node = root_.get();
        while (!node->is_leaf) {
            UInt32 child_idx = node->get_child_index(position);
            if (!node->children[child_idx]) {
                return node->partition_id;
            }
            node = node->children[child_idx].get();
        }

        return node->partition_id;
    }

    std::vector<EntityId> query_region(const AABB& region) const override {
        std::vector<EntityId> result;

        if (!root_) {
            return result;
        }

        std::queue<const OctreeNode*> queue;
        queue.push(root_.get());

        while (!queue.empty()) {
            const OctreeNode* node = queue.front();
            queue.pop();

            if (!node->bounds.overlaps(region)) {
                continue;
            }

            if (node->is_leaf) {
                for (EntityId entity_id : node->entities) {
                    auto pos_it = entity_positions_.find(entity_id);
                    if (pos_it != entity_positions_.end() && region.contains(pos_it->second)) {
                        result.push_back(entity_id);
                    }
                }
            } else {
                for (const auto& child : node->children) {
                    if (child) {
                        queue.push(child.get());
                    }
                }
            }
        }

        return result;
    }

    std::vector<PartitionId> query_partitions(const AABB& region) const override {
        std::vector<PartitionId> result;

        if (!root_) {
            return result;
        }

        std::queue<const OctreeNode*> queue;
        queue.push(root_.get());

        while (!queue.empty()) {
            const OctreeNode* node = queue.front();
            queue.pop();

            if (!node->bounds.overlaps(region)) {
                continue;
            }

            if (node->is_leaf) {
                result.push_back(node->partition_id);
            } else {
                for (const auto& child : node->children) {
                    if (child) {
                        queue.push(child.get());
                    }
                }
            }
        }

        return result;
    }

    std::optional<AABB> get_partition_bounds(PartitionId partition_id) const override {
        auto it = partition_nodes_.find(partition_id);
        if (it == partition_nodes_.end()) {
            return std::nullopt;
        }
        return it->second->bounds;
    }

    PartitionResult split(PartitionId partition_id) override {
        auto it = partition_nodes_.find(partition_id);
        if (it == partition_nodes_.end()) {
            return PartitionResult::PartitionNotFound;
        }

        OctreeNode* node = it->second;
        if (!node->is_leaf) {
            return PartitionResult::InvalidConfiguration;
        }

        if (node->depth >= max_depth_) {
            return PartitionResult::CapacityExceeded;
        }

        // Create 8 children
        node->is_leaf = false;
        for (UInt32 i = 0; i < 8; ++i) {
            node->children[i] = std::make_unique<OctreeNode>();
            node->children[i]->partition_id = next_partition_id_++;
            node->children[i]->bounds = node->get_child_bounds(i);
            node->children[i]->depth = node->depth + 1;
            node->children[i]->is_leaf = true;

            partition_nodes_[node->children[i]->partition_id] = node->children[i].get();
        }

        // Redistribute entities to children
        for (EntityId entity_id : node->entities) {
            auto pos_it = entity_positions_.find(entity_id);
            if (pos_it != entity_positions_.end()) {
                UInt32 child_idx = node->get_child_index(pos_it->second);
                node->children[child_idx]->entities.insert(entity_id);
                entity_partitions_[entity_id] = node->children[child_idx]->partition_id;
            }
        }
        node->entities.clear();

        return PartitionResult::Success;
    }

    PartitionResult merge(PartitionId parent_id) override {
        auto it = partition_nodes_.find(parent_id);
        if (it == partition_nodes_.end()) {
            return PartitionResult::PartitionNotFound;
        }

        OctreeNode* node = it->second;
        if (node->is_leaf) {
            return PartitionResult::InvalidConfiguration;
        }

        // Check all children are leaves
        for (const auto& child : node->children) {
            if (child && !child->is_leaf) {
                return PartitionResult::InvalidConfiguration;
            }
        }

        // Collect all entities from children
        for (const auto& child : node->children) {
            if (child) {
                for (EntityId entity_id : child->entities) {
                    node->entities.insert(entity_id);
                    entity_partitions_[entity_id] = parent_id;
                }
                partition_nodes_.erase(child->partition_id);
            }
        }

        // Remove children
        for (auto& child : node->children) {
            child.reset();
        }
        node->is_leaf = true;

        return PartitionResult::Success;
    }

    std::vector<PartitionId> get_leaf_partitions() const override {
        std::vector<PartitionId> result;

        if (!root_) {
            return result;
        }

        std::queue<const OctreeNode*> queue;
        queue.push(root_.get());

        while (!queue.empty()) {
            const OctreeNode* node = queue.front();
            queue.pop();

            if (node->is_leaf) {
                result.push_back(node->partition_id);
            } else {
                for (const auto& child : node->children) {
                    if (child) {
                        queue.push(child.get());
                    }
                }
            }
        }

        return result;
    }

    UInt32 get_entity_count(PartitionId partition_id) const override {
        auto it = partition_nodes_.find(partition_id);
        if (it == partition_nodes_.end()) {
            return 0;
        }
        return static_cast<UInt32>(it->second->entities.size());
    }

private:
    OctreeNode* find_or_create_leaf(const Vec3& position) {
        if (!root_) {
            return nullptr;
        }

        OctreeNode* node = root_.get();
        while (!node->is_leaf) {
            UInt32 child_idx = node->get_child_index(position);
            if (!node->children[child_idx]) {
                // Create missing child
                node->children[child_idx] = std::make_unique<OctreeNode>();
                node->children[child_idx]->partition_id = next_partition_id_++;
                node->children[child_idx]->bounds = node->get_child_bounds(child_idx);
                node->children[child_idx]->depth = node->depth + 1;
                node->children[child_idx]->is_leaf = true;

                partition_nodes_[node->children[child_idx]->partition_id] =
                    node->children[child_idx].get();
            }
            node = node->children[child_idx].get();
        }

        return node;
    }

    AABB world_bounds_;
    UInt32 max_depth_{8};
    PartitionId next_partition_id_{1};
    std::unique_ptr<OctreeNode> root_;
    std::unordered_map<PartitionId, OctreeNode*> partition_nodes_;
    std::unordered_map<EntityId, Vec3> entity_positions_;
    std::unordered_map<EntityId, PartitionId> entity_partitions_;
};

// ============================================================================
// Domain Partitioner Implementation
// ============================================================================

class SimpleDomainPartitioner : public IDomainPartitioner {
public:
    SimpleDomainPartitioner() = default;
    ~SimpleDomainPartitioner() override = default;

    PartitionResult initialize(const std::vector<Domain>& domains) override {
        next_partition_id_ = 1;

        for (Domain domain : domains) {
            PartitionId partition_id = next_partition_id_++;
            domain_partitions_[domain] = partition_id;
            partition_domains_[partition_id] = domain;
        }

        return PartitionResult::Success;
    }

    PartitionResult assign(EntityId entity_id, Domain domain) override {
        auto it = domain_partitions_.find(domain);
        if (it == domain_partitions_.end()) {
            // Create new partition for this domain
            PartitionId partition_id = next_partition_id_++;
            domain_partitions_[domain] = partition_id;
            partition_domains_[partition_id] = domain;
            it = domain_partitions_.find(domain);
        }

        entity_domains_[entity_id] = domain;
        partition_entities_[it->second].insert(entity_id);

        return PartitionResult::Success;
    }

    PartitionResult remove(EntityId entity_id) override {
        auto it = entity_domains_.find(entity_id);
        if (it == entity_domains_.end()) {
            return PartitionResult::EntityNotFound;
        }

        Domain domain = it->second;
        auto part_it = domain_partitions_.find(domain);
        if (part_it != domain_partitions_.end()) {
            partition_entities_[part_it->second].erase(entity_id);
        }

        entity_domains_.erase(it);

        return PartitionResult::Success;
    }

    PartitionId get_domain_partition(Domain domain) const override {
        auto it = domain_partitions_.find(domain);
        if (it == domain_partitions_.end()) {
            return INVALID_PARTITION_ID;
        }
        return it->second;
    }

    std::vector<EntityId> get_entities(Domain domain) const override {
        std::vector<EntityId> result;

        auto part_it = domain_partitions_.find(domain);
        if (part_it != domain_partitions_.end()) {
            auto ent_it = partition_entities_.find(part_it->second);
            if (ent_it != partition_entities_.end()) {
                result.reserve(ent_it->second.size());
                for (EntityId id : ent_it->second) {
                    result.push_back(id);
                }
            }
        }

        return result;
    }

    std::unordered_map<Domain, UInt32> get_domain_counts() const override {
        std::unordered_map<Domain, UInt32> result;

        for (const auto& [domain, partition_id] : domain_partitions_) {
            auto it = partition_entities_.find(partition_id);
            result[domain] = (it != partition_entities_.end())
                ? static_cast<UInt32>(it->second.size()) : 0;
        }

        return result;
    }

private:
    PartitionId next_partition_id_{1};
    std::unordered_map<Domain, PartitionId> domain_partitions_;
    std::unordered_map<PartitionId, Domain> partition_domains_;
    std::unordered_map<EntityId, Domain> entity_domains_;
    std::unordered_map<PartitionId, std::unordered_set<EntityId>> partition_entities_;
};

// ============================================================================
// Load Balancer Implementation
// ============================================================================

class SimpleLoadBalancer : public ILoadBalancer {
public:
    explicit SimpleLoadBalancer(LoadBalanceStrategy strategy)
        : strategy_(strategy) {}

    ~SimpleLoadBalancer() override = default;

    PartitionResult initialize(const std::vector<NodeInfo>& nodes,
                                const PartitionConfig& config) override {
        config_ = config;
        nodes_.clear();

        for (const auto& node : nodes) {
            nodes_[node.id] = node;
        }

        return PartitionResult::Success;
    }

    void update_node_metrics(const NodeInfo& node) override {
        nodes_[node.id] = node;
    }

    Real calculate_imbalance() const override {
        if (nodes_.empty()) {
            return 0.0;
        }

        std::vector<Real> loads;
        loads.reserve(nodes_.size());

        for (const auto& [id, node] : nodes_) {
            if (node.is_online) {
                loads.push_back(get_node_load(node));
            }
        }

        if (loads.empty()) {
            return 0.0;
        }

        Real min_load = *std::min_element(loads.begin(), loads.end());
        Real max_load = *std::max_element(loads.begin(), loads.end());

        if (max_load <= 0.0) {
            return 0.0;
        }

        return (max_load - min_load) / max_load;
    }

    std::vector<MigrationRequest> generate_migration_plan(
        const std::unordered_map<PartitionId, PartitionInfo>& partitions) override {

        std::vector<MigrationRequest> migrations;

        Real imbalance = calculate_imbalance();
        if (imbalance < config_.balance_threshold) {
            return migrations;
        }

        // Find overloaded and underloaded nodes
        std::vector<NodeId> overloaded;
        std::vector<NodeId> underloaded;

        Real avg_load = 0.0;
        UInt32 online_count = 0;
        for (const auto& [id, node] : nodes_) {
            if (node.is_online) {
                avg_load += get_node_load(node);
                online_count++;
            }
        }

        if (online_count == 0) {
            return migrations;
        }

        avg_load /= online_count;

        for (const auto& [id, node] : nodes_) {
            if (!node.is_online) continue;

            Real load = get_node_load(node);
            if (load > avg_load * 1.2) {
                overloaded.push_back(id);
            } else if (load < avg_load * 0.8) {
                underloaded.push_back(id);
            }
        }

        // Generate migration requests from overloaded to underloaded
        // (Simplified - real implementation would be more sophisticated)
        UInt32 migration_count = 0;
        for (NodeId from_node : overloaded) {
            if (underloaded.empty()) break;
            if (migration_count >= config_.max_migrations_per_step) break;

            // Find partitions on overloaded node
            for (const auto& [part_id, part_info] : partitions) {
                if (part_info.owner_node != from_node) continue;
                if (part_info.entities.empty()) continue;

                NodeId to_node = underloaded.back();

                // Migrate one entity at a time
                EntityId entity_id = *part_info.entities.begin();

                MigrationRequest request;
                request.entity_id = entity_id;
                request.source_partition = part_id;
                request.source_node = from_node;
                request.target_node = to_node;
                request.priority = MigrationPriority::Normal;
                request.reason = "Load balancing";
                request.requested_at = std::chrono::steady_clock::now();

                migrations.push_back(request);
                migration_count++;

                if (migration_count >= config_.max_migrations_per_step) break;
            }
        }

        return migrations;
    }

    NodeId find_best_node_for_partition(const PartitionInfo& /*partition*/) const override {
        NodeId best_node = INVALID_NODE_ID;
        Real best_score = -1.0;

        for (const auto& [id, node] : nodes_) {
            if (!node.is_online) continue;

            Real load = get_node_load(node);
            Real score = 1.0 - load;  // Lower load = higher score

            if (score > best_score) {
                best_score = score;
                best_node = id;
            }
        }

        return best_node;
    }

    std::unordered_map<NodeId, Real> get_node_loads() const override {
        std::unordered_map<NodeId, Real> result;

        for (const auto& [id, node] : nodes_) {
            result[id] = get_node_load(node);
        }

        return result;
    }

    bool should_rebalance() const override {
        return calculate_imbalance() >= config_.balance_threshold;
    }

private:
    Real get_node_load(const NodeInfo& node) const {
        switch (strategy_) {
            case LoadBalanceStrategy::EntityCount:
                // Normalize entity count (assume 10000 entities = full load)
                return std::min(1.0, node.entity_count / 10000.0);

            case LoadBalanceStrategy::CPULoad:
                return node.cpu_usage;

            case LoadBalanceStrategy::MemoryUsage:
                return node.memory_usage;

            case LoadBalanceStrategy::Weighted:
                return node.cpu_usage * 0.4 +
                       node.memory_usage * 0.3 +
                       (node.entity_count / 10000.0) * 0.3;

            default:
                return node.entity_count / 10000.0;
        }
    }

    LoadBalanceStrategy strategy_;
    PartitionConfig config_;
    std::unordered_map<NodeId, NodeInfo> nodes_;
};

// ============================================================================
// Entity Migrator Implementation
// ============================================================================

class SimpleEntityMigrator : public IEntityMigrator {
public:
    SimpleEntityMigrator() = default;
    ~SimpleEntityMigrator() override = default;

    PartitionResult initialize(const PartitionConfig& config) override {
        config_ = config;
        return PartitionResult::Success;
    }

    PartitionResult request_migration(const MigrationRequest& request) override {
        if (migrating_entities_.count(request.entity_id)) {
            return PartitionResult::OperationInProgress;
        }

        pending_migrations_.push_back(request);
        migrating_entities_.insert(request.entity_id);

        return PartitionResult::Success;
    }

    PartitionResult cancel_migration(EntityId entity_id) override {
        auto it = std::find_if(pending_migrations_.begin(), pending_migrations_.end(),
            [entity_id](const MigrationRequest& req) {
                return req.entity_id == entity_id;
            });

        if (it != pending_migrations_.end()) {
            pending_migrations_.erase(it);
            migrating_entities_.erase(entity_id);
            return PartitionResult::Success;
        }

        return PartitionResult::EntityNotFound;
    }

    std::vector<MigrationRequest> process_migrations(UInt32 max_count) override {
        std::vector<MigrationRequest> completed;
        UInt32 processed = 0;

        // Sort by priority
        std::sort(pending_migrations_.begin(), pending_migrations_.end(),
            [](const MigrationRequest& a, const MigrationRequest& b) {
                return static_cast<UInt8>(a.priority) > static_cast<UInt8>(b.priority);
            });

        while (!pending_migrations_.empty() && processed < max_count) {
            MigrationRequest request = pending_migrations_.front();
            pending_migrations_.erase(pending_migrations_.begin());

            request.started_at = std::chrono::steady_clock::now();
            request.is_pending = false;
            request.is_in_progress = true;

            // In real implementation, this would do actual network transfer
            // For now, we simulate immediate completion
            request.completed_at = std::chrono::steady_clock::now();
            request.is_in_progress = false;
            request.is_completed = true;
            request.result = PartitionResult::Success;

            migrating_entities_.erase(request.entity_id);
            completed.push_back(request);
            processed++;
        }

        return completed;
    }

    UInt32 pending_count() const override {
        return static_cast<UInt32>(pending_migrations_.size());
    }

    UInt32 in_progress_count() const override {
        // In real implementation, track in-progress separately
        return 0;
    }

    bool is_migrating(EntityId entity_id) const override {
        return migrating_entities_.count(entity_id) > 0;
    }

    std::vector<UInt8> serialize_entity(EntityId entity_id,
                                          const physics::EntityState& state) override {
        std::vector<UInt8> data;

        // Simple serialization - append raw bytes
        // In real implementation, use proper serialization framework
        data.resize(sizeof(EntityId) + sizeof(physics::EntityState));

        std::memcpy(data.data(), &entity_id, sizeof(EntityId));
        std::memcpy(data.data() + sizeof(EntityId), &state, sizeof(physics::EntityState));

        return data;
    }

    std::optional<physics::EntityState> deserialize_entity(
        const std::vector<UInt8>& data) override {

        if (data.size() < sizeof(EntityId) + sizeof(physics::EntityState)) {
            return std::nullopt;
        }

        physics::EntityState state;
        std::memcpy(&state, data.data() + sizeof(EntityId), sizeof(physics::EntityState));

        return state;
    }

private:
    PartitionConfig config_;
    std::vector<MigrationRequest> pending_migrations_;
    std::unordered_set<EntityId> migrating_entities_;
};

// ============================================================================
// Predictive Migrator Implementation
// ============================================================================

class SimplePredictiveMigrator : public IPredictiveMigrator {
public:
    SimplePredictiveMigrator() = default;
    ~SimplePredictiveMigrator() override = default;

    void update_prediction(EntityId entity_id,
                           const Vec3& position,
                           const Vec3& velocity,
                           const Vec3& acceleration) override {
        EntityTrajectory trajectory;
        trajectory.position = position;
        trajectory.velocity = velocity;
        trajectory.acceleration = acceleration;
        trajectory.timestamp = std::chrono::steady_clock::now();

        trajectories_[entity_id] = trajectory;
    }

    Vec3 predict_position(EntityId entity_id, Real seconds_ahead) const override {
        auto it = trajectories_.find(entity_id);
        if (it == trajectories_.end()) {
            return Vec3::Zero();
        }

        const EntityTrajectory& traj = it->second;

        // Simple kinematic prediction: p = p0 + v*t + 0.5*a*t^2
        return traj.position +
               traj.velocity * seconds_ahead +
               traj.acceleration * (0.5 * seconds_ahead * seconds_ahead);
    }

    std::optional<PartitionId> will_cross_boundary(
        EntityId entity_id,
        PartitionId current_partition,
        Real horizon_seconds) const override {

        auto it = trajectories_.find(entity_id);
        if (it == trajectories_.end()) {
            return std::nullopt;
        }

        auto bounds_it = partition_bounds_.find(current_partition);
        if (bounds_it == partition_bounds_.end()) {
            return std::nullopt;
        }

        const AABB& bounds = bounds_it->second;
        Vec3 predicted_pos = predict_position(entity_id, horizon_seconds);

        if (!bounds.contains(predicted_pos)) {
            // Will exit current partition - find target
            for (const auto& [part_id, part_bounds] : partition_bounds_) {
                if (part_id != current_partition && part_bounds.contains(predicted_pos)) {
                    return part_id;
                }
            }
        }

        return std::nullopt;
    }

    std::vector<MigrationRequest> get_predictive_migrations(Real horizon_seconds) const override {
        std::vector<MigrationRequest> requests;

        for (const auto& [entity_id, trajectory] : trajectories_) {
            auto part_it = entity_partitions_.find(entity_id);
            if (part_it == entity_partitions_.end()) continue;

            auto target = will_cross_boundary(entity_id, part_it->second, horizon_seconds);
            if (target.has_value()) {
                MigrationRequest request;
                request.entity_id = entity_id;
                request.source_partition = part_it->second;
                request.target_partition = target.value();
                request.priority = MigrationPriority::Normal;
                request.reason = "Predictive migration";
                request.requested_at = std::chrono::steady_clock::now();

                requests.push_back(request);
            }
        }

        return requests;
    }

    void clear_prediction(EntityId entity_id) override {
        trajectories_.erase(entity_id);
    }

    // Additional methods for partition bounds management
    void set_partition_bounds(PartitionId partition_id, const AABB& bounds) {
        partition_bounds_[partition_id] = bounds;
    }

    void set_entity_partition(EntityId entity_id, PartitionId partition_id) {
        entity_partitions_[entity_id] = partition_id;
    }

private:
    struct EntityTrajectory {
        Vec3 position;
        Vec3 velocity;
        Vec3 acceleration;
        std::chrono::steady_clock::time_point timestamp;
    };

    std::unordered_map<EntityId, EntityTrajectory> trajectories_;
    std::unordered_map<PartitionId, AABB> partition_bounds_;
    std::unordered_map<EntityId, PartitionId> entity_partitions_;
};

// ============================================================================
// Partition Manager Implementation
// ============================================================================

struct PartitionManager::Impl {
    PartitionConfig config;
    bool initialized{false};
    NodeId local_node_id{LOCAL_NODE_ID};

    // Partitioners
    std::unique_ptr<ISpatialPartitioner> spatial_partitioner;
    std::unique_ptr<IDomainPartitioner> domain_partitioner;
    std::unique_ptr<ILoadBalancer> load_balancer;
    std::unique_ptr<IEntityMigrator> migrator;
    std::unique_ptr<SimplePredictiveMigrator> predictive_migrator;

    // State
    std::unordered_map<NodeId, NodeInfo> nodes;
    std::unordered_map<PartitionId, PartitionInfo> partitions;
    std::unordered_map<EntityId, EntityLocation> entity_locations;

    // Statistics
    PartitionStats stats;

    // Callbacks
    EntityAssignedCallback on_entity_assigned;
    EntityMigratedCallback on_entity_migrated;
    PartitionChangedCallback on_partition_changed;
    BalanceCallback on_balance;
    CustomPartitionFunc custom_partition_func;

    // Timing
    Real time_since_last_balance{0.0};

    // Thread safety
    mutable std::mutex mutex;
};

PartitionManager::PartitionManager()
    : impl_(std::make_unique<Impl>()) {}

PartitionManager::~PartitionManager() = default;

PartitionManager::PartitionManager(PartitionManager&&) noexcept = default;
PartitionManager& PartitionManager::operator=(PartitionManager&&) noexcept = default;

PartitionResult PartitionManager::initialize(const PartitionConfig& config) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (impl_->initialized) {
        return PartitionResult::AlreadyInitialized;
    }

    impl_->config = config;

    // Create partitioners based on strategy
    if (config.strategy == PartitionStrategy::Spatial ||
        config.strategy == PartitionStrategy::DomainThenSpatial ||
        config.strategy == PartitionStrategy::SpatialThenDomain) {

        impl_->spatial_partitioner = create_octree_partitioner();
        auto result = impl_->spatial_partitioner->initialize(
            config.world_bounds, config.max_octree_depth);
        if (result != PartitionResult::Success) {
            return result;
        }
    }

    if (config.strategy == PartitionStrategy::Domain ||
        config.strategy == PartitionStrategy::DomainThenSpatial ||
        config.strategy == PartitionStrategy::SpatialThenDomain) {

        impl_->domain_partitioner = create_domain_partitioner();
        auto result = impl_->domain_partitioner->initialize({
            Domain::Air, Domain::Land, Domain::Sea, Domain::Space, Domain::Generic
        });
        if (result != PartitionResult::Success) {
            return result;
        }
    }

    // Create load balancer
    impl_->load_balancer = create_load_balancer(config.balance_strategy);
    impl_->load_balancer->initialize({}, config);

    // Create migrator
    impl_->migrator = create_entity_migrator();
    impl_->migrator->initialize(config);

    // Create predictive migrator
    if (config.predictive_migration) {
        impl_->predictive_migrator = std::make_unique<SimplePredictiveMigrator>();
    }

    // Register local node
    NodeInfo local_node;
    local_node.id = impl_->local_node_id;
    local_node.is_local = true;
    local_node.is_online = true;
    local_node.last_heartbeat = std::chrono::steady_clock::now();
    impl_->nodes[local_node.id] = local_node;

    impl_->initialized = true;
    return PartitionResult::Success;
}

void PartitionManager::shutdown() {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    impl_->spatial_partitioner.reset();
    impl_->domain_partitioner.reset();
    impl_->load_balancer.reset();
    impl_->migrator.reset();
    impl_->predictive_migrator.reset();

    impl_->nodes.clear();
    impl_->partitions.clear();
    impl_->entity_locations.clear();

    impl_->initialized = false;
}

bool PartitionManager::is_initialized() const noexcept {
    return impl_->initialized;
}

const PartitionConfig& PartitionManager::get_config() const noexcept {
    return impl_->config;
}

PartitionResult PartitionManager::register_node(const NodeInfo& node) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    impl_->nodes[node.id] = node;
    impl_->load_balancer->update_node_metrics(node);
    impl_->stats.total_nodes++;
    if (node.is_online) {
        impl_->stats.online_nodes++;
    }

    return PartitionResult::Success;
}

PartitionResult PartitionManager::unregister_node(NodeId node_id) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    auto it = impl_->nodes.find(node_id);
    if (it == impl_->nodes.end()) {
        return PartitionResult::NodeNotFound;
    }

    if (it->second.is_online) {
        impl_->stats.online_nodes--;
    }
    impl_->stats.total_nodes--;

    impl_->nodes.erase(it);
    return PartitionResult::Success;
}

void PartitionManager::update_node_metrics(const NodeInfo& node) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    impl_->nodes[node.id] = node;
    impl_->load_balancer->update_node_metrics(node);
}

std::optional<NodeInfo> PartitionManager::get_node(NodeId node_id) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    auto it = impl_->nodes.find(node_id);
    if (it == impl_->nodes.end()) {
        return std::nullopt;
    }
    return it->second;
}

std::vector<NodeInfo> PartitionManager::get_all_nodes() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    std::vector<NodeInfo> result;
    result.reserve(impl_->nodes.size());
    for (const auto& [id, node] : impl_->nodes) {
        result.push_back(node);
    }
    return result;
}

NodeId PartitionManager::get_local_node_id() const noexcept {
    return impl_->local_node_id;
}

void PartitionManager::set_local_node_id(NodeId node_id) noexcept {
    impl_->local_node_id = node_id;
}

PartitionResult PartitionManager::assign_entity(EntityId entity_id,
                                                  const Vec3& position,
                                                  Domain domain) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->initialized) {
        return PartitionResult::NotInitialized;
    }

    PartitionId partition_id = INVALID_PARTITION_ID;
    NodeId node_id = impl_->local_node_id;

    // Assign based on strategy
    switch (impl_->config.strategy) {
        case PartitionStrategy::Spatial:
            if (impl_->spatial_partitioner) {
                auto result = impl_->spatial_partitioner->insert(entity_id, position);
                if (result != PartitionResult::Success) {
                    return result;
                }
                partition_id = impl_->spatial_partitioner->find_partition(position);
            }
            break;

        case PartitionStrategy::Domain:
            if (impl_->domain_partitioner) {
                auto result = impl_->domain_partitioner->assign(entity_id, domain);
                if (result != PartitionResult::Success) {
                    return result;
                }
                partition_id = impl_->domain_partitioner->get_domain_partition(domain);
            }
            break;

        case PartitionStrategy::Custom:
            if (impl_->custom_partition_func) {
                partition_id = impl_->custom_partition_func(entity_id, position, domain);
            }
            break;

        default:
            // Use spatial as default
            if (impl_->spatial_partitioner) {
                auto result = impl_->spatial_partitioner->insert(entity_id, position);
                if (result != PartitionResult::Success) {
                    return result;
                }
                partition_id = impl_->spatial_partitioner->find_partition(position);
            }
            break;
    }

    // Create entity location
    EntityLocation location;
    location.entity_id = entity_id;
    location.partition_id = partition_id;
    location.node_id = node_id;
    location.position = position;
    location.domain = domain;

    impl_->entity_locations[entity_id] = location;
    impl_->stats.total_entities++;

    // Update partition info
    auto& partition = impl_->partitions[partition_id];
    partition.id = partition_id;
    partition.entities.insert(entity_id);
    partition.entity_count = static_cast<UInt32>(partition.entities.size());
    partition.owner_node = node_id;

    // Callback
    if (impl_->on_entity_assigned) {
        impl_->on_entity_assigned(entity_id, partition_id, node_id);
    }

    return PartitionResult::Success;
}

PartitionResult PartitionManager::remove_entity(EntityId entity_id) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    auto it = impl_->entity_locations.find(entity_id);
    if (it == impl_->entity_locations.end()) {
        return PartitionResult::EntityNotFound;
    }

    PartitionId partition_id = it->second.partition_id;

    // Remove from spatial partitioner
    if (impl_->spatial_partitioner) {
        impl_->spatial_partitioner->remove(entity_id);
    }

    // Remove from domain partitioner
    if (impl_->domain_partitioner) {
        impl_->domain_partitioner->remove(entity_id);
    }

    // Remove from partition info
    auto part_it = impl_->partitions.find(partition_id);
    if (part_it != impl_->partitions.end()) {
        part_it->second.entities.erase(entity_id);
        part_it->second.entity_count = static_cast<UInt32>(part_it->second.entities.size());
    }

    // Remove from predictive migrator
    if (impl_->predictive_migrator) {
        impl_->predictive_migrator->clear_prediction(entity_id);
    }

    impl_->entity_locations.erase(it);
    impl_->stats.total_entities--;

    return PartitionResult::Success;
}

PartitionResult PartitionManager::update_entity_position(EntityId entity_id,
                                                          const Vec3& new_position) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    auto it = impl_->entity_locations.find(entity_id);
    if (it == impl_->entity_locations.end()) {
        return PartitionResult::EntityNotFound;
    }

    it->second.position = new_position;

    if (impl_->spatial_partitioner) {
        auto result = impl_->spatial_partitioner->update(entity_id, new_position);
        if (result != PartitionResult::Success) {
            return result;
        }

        // Check if partition changed
        PartitionId new_partition = impl_->spatial_partitioner->find_partition(new_position);
        if (new_partition != it->second.partition_id) {
            // Update partition membership
            auto old_part_it = impl_->partitions.find(it->second.partition_id);
            if (old_part_it != impl_->partitions.end()) {
                old_part_it->second.entities.erase(entity_id);
                old_part_it->second.entity_count =
                    static_cast<UInt32>(old_part_it->second.entities.size());
            }

            auto& new_part = impl_->partitions[new_partition];
            new_part.entities.insert(entity_id);
            new_part.entity_count = static_cast<UInt32>(new_part.entities.size());

            it->second.partition_id = new_partition;
        }
    }

    return PartitionResult::Success;
}

std::optional<EntityLocation> PartitionManager::get_entity_location(EntityId entity_id) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    auto it = impl_->entity_locations.find(entity_id);
    if (it == impl_->entity_locations.end()) {
        return std::nullopt;
    }
    return it->second;
}

bool PartitionManager::is_local_entity(EntityId entity_id) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    auto it = impl_->entity_locations.find(entity_id);
    if (it == impl_->entity_locations.end()) {
        return false;
    }
    return it->second.node_id == impl_->local_node_id;
}

std::vector<EntityId> PartitionManager::get_entities_in_partition(PartitionId partition_id) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    std::vector<EntityId> result;

    auto it = impl_->partitions.find(partition_id);
    if (it != impl_->partitions.end()) {
        result.reserve(it->second.entities.size());
        for (EntityId id : it->second.entities) {
            result.push_back(id);
        }
    }

    return result;
}

std::vector<EntityId> PartitionManager::get_entities_on_node(NodeId node_id) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    std::vector<EntityId> result;

    for (const auto& [id, location] : impl_->entity_locations) {
        if (location.node_id == node_id) {
            result.push_back(id);
        }
    }

    return result;
}

std::vector<EntityId> PartitionManager::get_local_entities() const {
    return get_entities_on_node(impl_->local_node_id);
}

std::optional<PartitionInfo> PartitionManager::get_partition(PartitionId partition_id) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    auto it = impl_->partitions.find(partition_id);
    if (it == impl_->partitions.end()) {
        return std::nullopt;
    }
    return it->second;
}

std::vector<PartitionInfo> PartitionManager::get_all_partitions() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    std::vector<PartitionInfo> result;
    result.reserve(impl_->partitions.size());
    for (const auto& [id, info] : impl_->partitions) {
        result.push_back(info);
    }
    return result;
}

std::vector<PartitionId> PartitionManager::get_partitions_on_node(NodeId node_id) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    std::vector<PartitionId> result;

    for (const auto& [id, info] : impl_->partitions) {
        if (info.owner_node == node_id) {
            result.push_back(id);
        }
    }

    return result;
}

std::vector<PartitionId> PartitionManager::query_partitions(const AABB& region) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (impl_->spatial_partitioner) {
        return impl_->spatial_partitioner->query_partitions(region);
    }

    return {};
}

std::vector<PartitionId> PartitionManager::get_neighbors(PartitionId partition_id) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    auto it = impl_->partitions.find(partition_id);
    if (it == impl_->partitions.end()) {
        return {};
    }

    return std::vector<PartitionId>(
        it->second.neighbors.begin(),
        it->second.neighbors.end()
    );
}

PartitionResult PartitionManager::split_partition(PartitionId partition_id) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (impl_->spatial_partitioner) {
        auto result = impl_->spatial_partitioner->split(partition_id);
        if (result == PartitionResult::Success && impl_->on_partition_changed) {
            impl_->on_partition_changed(partition_id, true);
        }
        return result;
    }

    return PartitionResult::InvalidConfiguration;
}

PartitionResult PartitionManager::merge_partitions(PartitionId parent_id) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (impl_->spatial_partitioner) {
        auto result = impl_->spatial_partitioner->merge(parent_id);
        if (result == PartitionResult::Success && impl_->on_partition_changed) {
            impl_->on_partition_changed(parent_id, false);
        }
        return result;
    }

    return PartitionResult::InvalidConfiguration;
}

PartitionResult PartitionManager::balance_load() {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->load_balancer) {
        return PartitionResult::InvalidConfiguration;
    }

    auto start_time = std::chrono::steady_clock::now();

    auto migrations = impl_->load_balancer->generate_migration_plan(impl_->partitions);

    for (const auto& migration : migrations) {
        impl_->migrator->request_migration(migration);
    }

    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time);

    impl_->stats.last_balance_time_ms = duration.count();
    impl_->stats.total_balance_time_ms += duration.count();
    impl_->stats.balance_count++;

    if (impl_->on_balance) {
        impl_->on_balance(impl_->stats);
    }

    return PartitionResult::Success;
}

Real PartitionManager::get_load_imbalance() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (impl_->load_balancer) {
        return impl_->load_balancer->calculate_imbalance();
    }

    return 0.0;
}

std::unordered_map<NodeId, Real> PartitionManager::get_node_loads() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (impl_->load_balancer) {
        return impl_->load_balancer->get_node_loads();
    }

    return {};
}

void PartitionManager::set_balance_strategy(LoadBalanceStrategy strategy) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    impl_->load_balancer = create_load_balancer(strategy);

    std::vector<NodeInfo> nodes;
    nodes.reserve(impl_->nodes.size());
    for (const auto& [id, node] : impl_->nodes) {
        nodes.push_back(node);
    }

    impl_->load_balancer->initialize(nodes, impl_->config);
}

PartitionResult PartitionManager::request_migration(EntityId entity_id,
                                                     PartitionId target_partition,
                                                     MigrationPriority priority) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    auto it = impl_->entity_locations.find(entity_id);
    if (it == impl_->entity_locations.end()) {
        return PartitionResult::EntityNotFound;
    }

    MigrationRequest request;
    request.entity_id = entity_id;
    request.source_partition = it->second.partition_id;
    request.target_partition = target_partition;
    request.source_node = it->second.node_id;
    request.priority = priority;
    request.requested_at = std::chrono::steady_clock::now();

    auto target_it = impl_->partitions.find(target_partition);
    if (target_it != impl_->partitions.end()) {
        request.target_node = target_it->second.owner_node;
    }

    return impl_->migrator->request_migration(request);
}

PartitionResult PartitionManager::cancel_migration(EntityId entity_id) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->migrator->cancel_migration(entity_id);
}

void PartitionManager::process_migrations() {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    auto completed = impl_->migrator->process_migrations(impl_->config.max_migrations_per_step);

    for (const auto& migration : completed) {
        if (migration.is_completed && migration.result == PartitionResult::Success) {
            // Update entity location
            auto it = impl_->entity_locations.find(migration.entity_id);
            if (it != impl_->entity_locations.end()) {
                it->second.partition_id = migration.target_partition;
                it->second.node_id = migration.target_node;
            }

            // Update statistics
            impl_->stats.total_migrations++;
            impl_->stats.successful_migrations++;

            // Callback
            if (impl_->on_entity_migrated) {
                impl_->on_entity_migrated(migration);
            }
        } else if (migration.is_failed) {
            impl_->stats.total_migrations++;
            impl_->stats.failed_migrations++;
        }
    }

    impl_->stats.pending_migrations = impl_->migrator->pending_count();
}

UInt32 PartitionManager::pending_migration_count() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->migrator->pending_count();
}

bool PartitionManager::is_migrating(EntityId entity_id) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->migrator->is_migrating(entity_id);
}

void PartitionManager::update_entity_trajectory(EntityId entity_id,
                                                  const Vec3& position,
                                                  const Vec3& velocity,
                                                  const Vec3& acceleration) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (impl_->predictive_migrator) {
        impl_->predictive_migrator->update_prediction(entity_id, position, velocity, acceleration);
    }
}

void PartitionManager::set_predictive_migration(bool enabled) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (enabled && !impl_->predictive_migrator) {
        impl_->predictive_migrator = std::make_unique<SimplePredictiveMigrator>();
    } else if (!enabled) {
        impl_->predictive_migrator.reset();
    }
}

void PartitionManager::update(Real dt) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    // Process pending migrations
    auto completed = impl_->migrator->process_migrations(impl_->config.max_migrations_per_step);
    for (const auto& migration : completed) {
        if (migration.is_completed && impl_->on_entity_migrated) {
            impl_->on_entity_migrated(migration);
        }
    }

    // Check for predictive migrations
    if (impl_->predictive_migrator && impl_->config.predictive_migration) {
        auto predictive = impl_->predictive_migrator->get_predictive_migrations(
            impl_->config.prediction_horizon_seconds);

        for (const auto& request : predictive) {
            impl_->migrator->request_migration(request);
        }
    }

    // Periodic load balancing
    impl_->time_since_last_balance += dt;
    if (impl_->config.auto_balance &&
        impl_->time_since_last_balance >= impl_->config.balance_interval_seconds) {

        if (impl_->load_balancer->should_rebalance()) {
            // Note: Can't call balance_load directly due to mutex, would need unlock
            auto migrations = impl_->load_balancer->generate_migration_plan(impl_->partitions);
            for (const auto& migration : migrations) {
                impl_->migrator->request_migration(migration);
            }
        }

        impl_->time_since_last_balance = 0.0;
    }

    // Update statistics
    impl_->stats.load_imbalance = impl_->load_balancer->calculate_imbalance();
    impl_->stats.pending_migrations = impl_->migrator->pending_count();
    impl_->stats.active_partitions = static_cast<UInt32>(impl_->partitions.size());
}

PartitionStats PartitionManager::get_stats() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->stats;
}

void PartitionManager::reset_stats() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->stats.reset();
}

void PartitionManager::set_entity_assigned_callback(EntityAssignedCallback callback) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->on_entity_assigned = std::move(callback);
}

void PartitionManager::set_entity_migrated_callback(EntityMigratedCallback callback) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->on_entity_migrated = std::move(callback);
}

void PartitionManager::set_partition_changed_callback(PartitionChangedCallback callback) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->on_partition_changed = std::move(callback);
}

void PartitionManager::set_balance_callback(BalanceCallback callback) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->on_balance = std::move(callback);
}

void PartitionManager::set_custom_partition_func(CustomPartitionFunc func) {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->custom_partition_func = std::move(func);
}

// ============================================================================
// Factory Functions
// ============================================================================

std::unique_ptr<ISpatialPartitioner> create_octree_partitioner() {
    return std::make_unique<OctreePartitioner>();
}

std::unique_ptr<IDomainPartitioner> create_domain_partitioner() {
    return std::make_unique<SimpleDomainPartitioner>();
}

std::unique_ptr<ILoadBalancer> create_load_balancer(LoadBalanceStrategy strategy) {
    return std::make_unique<SimpleLoadBalancer>(strategy);
}

std::unique_ptr<IEntityMigrator> create_entity_migrator() {
    return std::make_unique<SimpleEntityMigrator>();
}

std::unique_ptr<IPredictiveMigrator> create_predictive_migrator() {
    return std::make_unique<SimplePredictiveMigrator>();
}

} // namespace jaguar::cloud
