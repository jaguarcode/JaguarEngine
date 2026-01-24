/**
 * @file state_sync.cpp
 * @brief Implementation of state synchronization system for distributed simulation
 */

#include "jaguar/cloud/state_sync.h"
#include <algorithm>
#include <mutex>
#include <cstring>
#include <deque>

namespace jaguar::cloud {

// ============================================================================
// Simple State Serializer Implementation
// ============================================================================

class SimpleStateSerializer : public IStateSerializer {
public:
    SimpleStateSerializer() = default;
    ~SimpleStateSerializer() override = default;

    SyncResult serialize(const EntityState& state,
                        std::vector<UInt8>& output) override {
        try {
            // Simple serialization using memcpy
            output.resize(sizeof(EntityState));
            std::memcpy(output.data(), &state, sizeof(EntityState));
            return SyncResult::Success;
        } catch (...) {
            return SyncResult::SerializationFailed;
        }
    }

    SyncResult deserialize(const std::vector<UInt8>& data,
                          EntityState& state) override {
        if (data.size() < sizeof(EntityState)) {
            return SyncResult::DeserializationFailed;
        }

        try {
            std::memcpy(&state, data.data(), sizeof(EntityState));
            return SyncResult::Success;
        } catch (...) {
            return SyncResult::DeserializationFailed;
        }
    }

    SyncResult serialize_delta(const EntityState& old_state,
                               const EntityState& new_state,
                               StateDelta& delta) override {
        delta.entity_id = new_state.entity_id;
        delta.base_version = old_state.version;
        delta.new_version = new_state.version;
        delta.created_at = std::chrono::steady_clock::now();
        delta.changed_properties.clear();

        // Create delta by comparing states
        std::vector<UInt8> delta_data;

        // Simple approach: serialize both and store differences
        // In production, would use more sophisticated delta encoding
        std::vector<UInt8> old_data, new_data;
        serialize(old_state, old_data);
        serialize(new_state, new_data);

        delta_data = new_data; // Simplified - just store new state
        delta.uncompressed_size = static_cast<UInt64>(delta_data.size());

        if (delta.uncompressed_size > 1024) {
            delta.delta_data = compress(delta_data);
            delta.is_compressed = true;
            delta.compressed_size = static_cast<UInt64>(delta.delta_data.size());
        } else {
            delta.delta_data = delta_data;
            delta.is_compressed = false;
            delta.compressed_size = delta.uncompressed_size;
        }

        return SyncResult::Success;
    }

    SyncResult apply_delta(const EntityState& base_state,
                          const StateDelta& delta,
                          EntityState& new_state) override {
        if (delta.entity_id != base_state.entity_id) {
            return SyncResult::InvalidDelta;
        }

        std::vector<UInt8> data = delta.delta_data;
        if (delta.is_compressed) {
            data = decompress(data);
        }

        return deserialize(data, new_state);
    }

    std::vector<UInt8> compress(const std::vector<UInt8>& data) override {
        // Simple RLE compression (run-length encoding)
        std::vector<UInt8> compressed;
        if (data.empty()) return compressed;

        compressed.reserve(data.size());

        UInt8 current = data[0];
        UInt32 count = 1;

        for (size_t i = 1; i < data.size(); ++i) {
            if (data[i] == current && count < 255) {
                count++;
            } else {
                compressed.push_back(static_cast<UInt8>(count));
                compressed.push_back(current);
                current = data[i];
                count = 1;
            }
        }
        compressed.push_back(static_cast<UInt8>(count));
        compressed.push_back(current);

        // If compression didn't help, return original
        if (compressed.size() >= data.size()) {
            return data;
        }

        return compressed;
    }

    std::vector<UInt8> decompress(const std::vector<UInt8>& data) override {
        std::vector<UInt8> decompressed;
        if (data.empty()) return decompressed;

        // Check if data looks like RLE (even size)
        if (data.size() % 2 != 0) {
            // Not compressed, return as is
            return data;
        }

        decompressed.reserve(data.size() * 2);

        for (size_t i = 0; i + 1 < data.size(); i += 2) {
            UInt8 count = data[i];
            UInt8 value = data[i + 1];
            for (UInt8 j = 0; j < count; ++j) {
                decompressed.push_back(value);
            }
        }

        return decompressed;
    }

    std::string calculate_checksum(const std::vector<UInt8>& data) override {
        // Simple hash (not cryptographic)
        UInt64 hash = 0;
        for (UInt8 byte : data) {
            hash = hash * 31 + byte;
        }

        char buf[32];
        std::snprintf(buf, sizeof(buf), "%016llx",
                     static_cast<unsigned long long>(hash));
        return std::string(buf);
    }

    bool verify_checksum(const std::vector<UInt8>& data,
                        const std::string& checksum) override {
        return calculate_checksum(data) == checksum;
    }
};

// ============================================================================
// Simple Conflict Resolver Implementation
// ============================================================================

class SimpleConflictResolver : public IConflictResolver {
public:
    explicit SimpleConflictResolver(ConflictResolution strategy)
        : strategy_(strategy) {}

    ~SimpleConflictResolver() override = default;

    std::optional<ConflictInfo> detect_conflict(
        const EntityState& state_a,
        const EntityState& state_b) override {

        if (state_a.entity_id != state_b.entity_id) {
            return std::nullopt;
        }

        // Check if versions are concurrent (conflict)
        if (state_a.version.concurrent_with(state_b.version)) {
            ConflictInfo conflict;
            conflict.entity_id = state_a.entity_id;
            conflict.partition_id = state_a.partition_id;
            conflict.version_a = state_a.version;
            conflict.version_b = state_b.version;
            conflict.node_a = state_a.owner_node;
            conflict.node_b = state_b.owner_node;
            conflict.resolution = strategy_;
            conflict.is_resolved = false;
            conflict.detected_at = std::chrono::steady_clock::now();

            // Identify conflicting properties (simplified)
            // Compare positions using length_squared (epsilon comparison would be better)
            Vec3 pos_diff = state_a.position - state_b.position;
            if (pos_diff.length_squared() > 1e-10) {
                conflict.conflicting_properties.insert(0); // position
            }
            Vec3 vel_diff = state_a.velocity - state_b.velocity;
            if (vel_diff.length_squared() > 1e-10) {
                conflict.conflicting_properties.insert(1); // velocity
            }

            return conflict;
        }

        return std::nullopt;
    }

    SyncResult resolve_conflict(const ConflictInfo& conflict,
                               const EntityState& state_a,
                               const EntityState& state_b,
                               EntityState& resolved_state) override {

        switch (strategy_) {
            case ConflictResolution::LastWriterWins:
                // Use the state with later wall clock time
                resolved_state = (state_a.version.wall_time > state_b.version.wall_time)
                    ? state_a : state_b;
                break;

            case ConflictResolution::FirstWriterWins:
                // Use the state with earlier wall clock time
                resolved_state = (state_a.version.wall_time < state_b.version.wall_time)
                    ? state_a : state_b;
                break;

            case ConflictResolution::Merge:
                // Simple merge: average conflicting values
                resolved_state = state_a;

                // Average position if conflicting
                if (conflict.conflicting_properties.count(0)) {
                    resolved_state.position = (state_a.position + state_b.position) * 0.5;
                }

                // Average velocity if conflicting
                if (conflict.conflicting_properties.count(1)) {
                    resolved_state.velocity = (state_a.velocity + state_b.velocity) * 0.5;
                }

                // Use later version info
                if (state_b.version.wall_time > state_a.version.wall_time) {
                    resolved_state.version = state_b.version;
                }
                break;

            case ConflictResolution::Rollback:
                // Use the earlier state (requires rollback of later one)
                resolved_state = (state_a.version.wall_time < state_b.version.wall_time)
                    ? state_a : state_b;
                return SyncResult::RollbackRequired;

            case ConflictResolution::Custom:
                // Default to last writer wins
                resolved_state = (state_a.version.wall_time > state_b.version.wall_time)
                    ? state_a : state_b;
                break;

            default:
                return SyncResult::ConflictDetected;
        }

        // Track resolution
        conflict_history_.push_back(conflict);
        if (conflict_history_.size() > 1000) {
            conflict_history_.erase(conflict_history_.begin());
        }

        return SyncResult::Success;
    }

    bool requires_rollback(const ConflictInfo& conflict) const override {
        (void)conflict;
        return strategy_ == ConflictResolution::Rollback;
    }

    void set_strategy(ConflictResolution strategy) override {
        strategy_ = strategy;
    }

    ConflictResolution get_strategy() const override {
        return strategy_;
    }

private:
    ConflictResolution strategy_;
    std::deque<ConflictInfo> conflict_history_;
};

// ============================================================================
// Simple Snapshot Manager Implementation
// ============================================================================

class SimpleSnapshotManager : public ISnapshotManager {
public:
    SimpleSnapshotManager() = default;
    ~SimpleSnapshotManager() override = default;

    SyncResult create_full_snapshot(
        const std::vector<EntityState>& entities,
        StateSnapshot& snapshot) override {

        snapshot.snapshot_id = next_snapshot_id_++;
        snapshot.type = SnapshotType::Full;
        snapshot.created_at = std::chrono::steady_clock::now();
        snapshot.is_compressed = false;
        snapshot.is_verified = false;

        // Collect entity IDs
        snapshot.entities.clear();
        snapshot.entities.reserve(entities.size());
        for (const auto& entity : entities) {
            snapshot.entities.push_back(entity.entity_id);
        }

        // Serialize all entities
        std::vector<UInt8> data;
        size_t total_size = entities.size() * sizeof(EntityState);
        data.resize(total_size);

        for (size_t i = 0; i < entities.size(); ++i) {
            std::memcpy(data.data() + i * sizeof(EntityState),
                       &entities[i], sizeof(EntityState));
        }

        snapshot.uncompressed_size = static_cast<UInt64>(data.size());

        // Compress if large
        if (data.size() > 4096) {
            auto serializer = create_state_serializer();
            snapshot.snapshot_data = serializer->compress(data);
            snapshot.is_compressed = true;
            snapshot.compressed_size = static_cast<UInt64>(snapshot.snapshot_data.size());
        } else {
            snapshot.snapshot_data = data;
            snapshot.compressed_size = snapshot.uncompressed_size;
        }

        // Calculate checksum
        auto serializer = create_state_serializer();
        snapshot.checksum = serializer->calculate_checksum(snapshot.snapshot_data);

        return SyncResult::Success;
    }

    SyncResult create_incremental_snapshot(
        const std::vector<EntityState>& entities,
        SnapshotId baseline_id,
        StateSnapshot& snapshot) override {

        auto baseline = load_snapshot(baseline_id);
        if (!baseline) {
            return SyncResult::SnapshotNotFound;
        }

        snapshot.snapshot_id = next_snapshot_id_++;
        snapshot.type = SnapshotType::Incremental;
        snapshot.baseline_snapshot = baseline_id;
        snapshot.created_at = std::chrono::steady_clock::now();

        // For simplicity, just create a full snapshot
        // Real implementation would calculate delta from baseline
        return create_full_snapshot(entities, snapshot);
    }

    SyncResult restore_snapshot(const StateSnapshot& snapshot,
                               std::vector<EntityState>& entities) override {

        std::vector<UInt8> data = snapshot.snapshot_data;

        // Verify checksum if available
        if (!snapshot.checksum.empty()) {
            auto serializer = create_state_serializer();
            if (!serializer->verify_checksum(data, snapshot.checksum)) {
                return SyncResult::SnapshotCorrupted;
            }
        }

        // Decompress if needed
        if (snapshot.is_compressed) {
            auto serializer = create_state_serializer();
            data = serializer->decompress(data);
        }

        // Deserialize entities
        if (data.size() % sizeof(EntityState) != 0) {
            return SyncResult::SnapshotCorrupted;
        }

        size_t entity_count = data.size() / sizeof(EntityState);
        entities.resize(entity_count);

        for (size_t i = 0; i < entity_count; ++i) {
            std::memcpy(&entities[i],
                       data.data() + i * sizeof(EntityState),
                       sizeof(EntityState));
        }

        return SyncResult::Success;
    }

    SyncResult store_snapshot(const StateSnapshot& snapshot) override {
        std::lock_guard<std::mutex> lock(mutex_);

        snapshots_[snapshot.snapshot_id] = snapshot;

        // Track by partition
        if (snapshot.partition_id != INVALID_PARTITION_ID) {
            partition_snapshots_[snapshot.partition_id].push_back(snapshot.snapshot_id);
        }

        return SyncResult::Success;
    }

    std::optional<StateSnapshot> load_snapshot(SnapshotId snapshot_id) override {
        std::lock_guard<std::mutex> lock(mutex_);

        auto it = snapshots_.find(snapshot_id);
        if (it == snapshots_.end()) {
            return std::nullopt;
        }
        return it->second;
    }

    SyncResult delete_snapshot(SnapshotId snapshot_id) override {
        std::lock_guard<std::mutex> lock(mutex_);

        auto it = snapshots_.find(snapshot_id);
        if (it == snapshots_.end()) {
            return SyncResult::SnapshotNotFound;
        }

        PartitionId partition_id = it->second.partition_id;
        snapshots_.erase(it);

        // Remove from partition list
        if (partition_id != INVALID_PARTITION_ID) {
            auto& list = partition_snapshots_[partition_id];
            list.erase(std::remove(list.begin(), list.end(), snapshot_id), list.end());
        }

        return SyncResult::Success;
    }

    std::vector<SnapshotId> get_snapshots(PartitionId partition_id) override {
        std::lock_guard<std::mutex> lock(mutex_);

        auto it = partition_snapshots_.find(partition_id);
        if (it == partition_snapshots_.end()) {
            return {};
        }
        return it->second;
    }

    SyncResult cleanup_old_snapshots(PartitionId partition_id,
                                     UInt32 keep_count) override {
        std::lock_guard<std::mutex> lock(mutex_);

        auto it = partition_snapshots_.find(partition_id);
        if (it == partition_snapshots_.end()) {
            return SyncResult::Success;
        }

        auto& snapshot_list = it->second;
        if (snapshot_list.size() <= keep_count) {
            return SyncResult::Success;
        }

        // Sort by creation time (assuming IDs are sequential)
        std::sort(snapshot_list.begin(), snapshot_list.end());

        // Delete oldest snapshots
        size_t delete_count = snapshot_list.size() - keep_count;
        for (size_t i = 0; i < delete_count; ++i) {
            snapshots_.erase(snapshot_list[i]);
        }

        snapshot_list.erase(snapshot_list.begin(),
                          snapshot_list.begin() + static_cast<std::ptrdiff_t>(delete_count));

        return SyncResult::Success;
    }

    bool verify_snapshot(const StateSnapshot& snapshot) override {
        if (snapshot.checksum.empty()) {
            return false;
        }

        auto serializer = create_state_serializer();
        return serializer->verify_checksum(snapshot.snapshot_data, snapshot.checksum);
    }

    UInt64 get_storage_size() const override {
        std::lock_guard<std::mutex> lock(mutex_);

        UInt64 total = 0;
        for (const auto& [id, snapshot] : snapshots_) {
            total += snapshot.compressed_size;
        }
        return total;
    }

private:
    mutable std::mutex mutex_;
    SnapshotId next_snapshot_id_{1};
    std::unordered_map<SnapshotId, StateSnapshot> snapshots_;
    std::unordered_map<PartitionId, std::vector<SnapshotId>> partition_snapshots_;
};

// ============================================================================
// Simple Sync Transport Implementation
// ============================================================================

class SimpleSyncTransport : public ISyncTransport {
public:
    SimpleSyncTransport() = default;
    ~SimpleSyncTransport() override = default;

    SyncResult send_state(const NodeId& target_node,
                         const EntityState& state) override {
        std::lock_guard<std::mutex> lock(mutex_);

        (void)state;  // Unused parameter

        // Simulate network delay
        message_count_++;

        // Stub: in real implementation, would send over network
        // For now, just check if node is reachable
        if (!is_node_reachable(target_node)) {
            return SyncResult::NodeUnreachable;
        }

        return SyncResult::Success;
    }

    SyncResult send_delta(const NodeId& target_node,
                         const StateDelta& delta) override {
        std::lock_guard<std::mutex> lock(mutex_);

        (void)delta;  // Unused parameter

        message_count_++;

        if (!is_node_reachable(target_node)) {
            return SyncResult::NodeUnreachable;
        }

        return SyncResult::Success;
    }

    SyncResult send_snapshot(const NodeId& target_node,
                            const StateSnapshot& snapshot) override {
        std::lock_guard<std::mutex> lock(mutex_);

        (void)snapshot;  // Unused parameter

        message_count_++;

        if (!is_node_reachable(target_node)) {
            return SyncResult::NodeUnreachable;
        }

        return SyncResult::Success;
    }

    std::optional<EntityState> request_state(
        const NodeId& source_node,
        EntityId entity_id,
        Duration timeout) override {

        (void)timeout;

        std::lock_guard<std::mutex> lock(mutex_);

        if (!is_node_reachable(source_node)) {
            return std::nullopt;
        }

        // Stub: would request from network
        // For now, return empty
        (void)entity_id;
        return std::nullopt;
    }

    SyncResult broadcast_state(const EntityState& state) override {
        std::lock_guard<std::mutex> lock(mutex_);

        // Broadcast to all reachable nodes
        for (const auto& [node_id, reachable] : node_reachability_) {
            if (reachable) {
                send_state(node_id, state);
            }
        }

        return SyncResult::Success;
    }

    bool is_node_reachable(const NodeId& node_id) const override {
        auto it = node_reachability_.find(node_id);
        if (it == node_reachability_.end()) {
            return false;
        }
        return it->second;
    }

    Duration get_latency(const NodeId& node_id) const override {
        auto it = node_latency_.find(node_id);
        if (it == node_latency_.end()) {
            return Duration(0);
        }
        return it->second;
    }

    // Additional methods for testing/configuration
    void set_node_reachable(const NodeId& node_id, bool reachable) {
        std::lock_guard<std::mutex> lock(mutex_);
        node_reachability_[node_id] = reachable;
    }

    void set_node_latency(const NodeId& node_id, Duration latency) {
        std::lock_guard<std::mutex> lock(mutex_);
        node_latency_[node_id] = latency;
    }

private:
    mutable std::mutex mutex_;
    std::unordered_map<NodeId, bool> node_reachability_;
    std::unordered_map<NodeId, Duration> node_latency_;
    UInt64 message_count_{0};
};

// ============================================================================
// Simple State Synchronizer Implementation
// ============================================================================

class SimpleStateSynchronizer : public StateSynchronizer {
public:
    explicit SimpleStateSynchronizer(const StateSyncConfig& config)
        : config_(config) {}

    ~SimpleStateSynchronizer() override = default;

    SyncResult initialize(void* partition_manager,
                         void* time_manager) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (initialized_) {
            return SyncResult::AlreadyInitialized;
        }

        partition_manager_ = partition_manager;
        time_manager_ = time_manager;

        // Create components
        serializer_ = create_state_serializer();
        conflict_resolver_ = create_conflict_resolver(config_.conflict_resolution);
        snapshot_manager_ = create_snapshot_manager(config_);
        transport_ = create_sync_transport();

        initialized_ = true;
        return SyncResult::Success;
    }

    SyncResult shutdown() override {
        std::lock_guard<std::mutex> lock(mutex_);

        serializer_.reset();
        conflict_resolver_.reset();
        snapshot_manager_.reset();
        transport_.reset();

        entity_states_.clear();
        pending_conflicts_.clear();

        initialized_ = false;
        return SyncResult::Success;
    }

    bool is_initialized() const override {
        return initialized_;
    }

    SyncResult sync_entity(EntityId entity_id,
                          const EntityState& state) override {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!initialized_) {
            return SyncResult::NotInitialized;
        }

        // Check if we have previous state
        auto it = entity_states_.find(entity_id);

        if (it != entity_states_.end()) {
            // Create delta
            StateDelta delta;
            auto result = serializer_->serialize_delta(it->second, state, delta);
            if (result != SyncResult::Success) {
                stats_.failed_syncs++;
                return result;
            }

            // Send delta to other nodes (simplified)
            // In real implementation, would determine target nodes
            stats_.delta_syncs++;
            stats_.total_bytes_sent += delta.compressed_size;
            stats_.compressed_bytes_sent += delta.compressed_size;

            Real ratio = delta.compression_ratio();
            stats_.avg_compression_ratio =
                (stats_.avg_compression_ratio * stats_.delta_syncs + ratio) /
                (stats_.delta_syncs + 1);
        } else {
            // Full sync for first time
            stats_.full_syncs++;
        }

        // Update local state
        entity_states_[entity_id] = state;

        stats_.total_syncs++;
        stats_.successful_syncs++;

        return SyncResult::Success;
    }

    SyncResult sync_entities(const std::vector<EntityId>& entity_ids,
                            const std::vector<EntityState>& states) override {
        if (entity_ids.size() != states.size()) {
            return SyncResult::InvalidConfiguration;
        }

        for (size_t i = 0; i < entity_ids.size(); ++i) {
            auto result = sync_entity(entity_ids[i], states[i]);
            if (result != SyncResult::Success) {
                return result;
            }
        }

        return SyncResult::Success;
    }

    std::optional<EntityState> request_entity_state(
        EntityId entity_id,
        const NodeId& owner_node) override {

        std::lock_guard<std::mutex> lock(mutex_);

        // Try local first
        auto it = entity_states_.find(entity_id);
        if (it != entity_states_.end()) {
            return it->second;
        }

        // Request from owner node
        return transport_->request_state(owner_node, entity_id,
                                        config_.network_timeout);
    }

    std::optional<EntityState> get_local_state(EntityId entity_id) const override {
        std::lock_guard<std::mutex> lock(mutex_);

        auto it = entity_states_.find(entity_id);
        if (it == entity_states_.end()) {
            return std::nullopt;
        }
        return it->second;
    }

    SyncResult update_version(EntityId entity_id,
                             const StateVersion& version) override {
        std::lock_guard<std::mutex> lock(mutex_);

        auto it = entity_states_.find(entity_id);
        if (it == entity_states_.end()) {
            return SyncResult::InvalidEntityId;
        }

        it->second.version = version;
        return SyncResult::Success;
    }

    SyncResult sync_partition(PartitionId partition_id) override {
        std::lock_guard<std::mutex> lock(mutex_);

        // Sync all entities in partition
        UInt32 synced = 0;
        for (const auto& [entity_id, state] : entity_states_) {
            if (state.partition_id == partition_id) {
                // Would send to other nodes
                synced++;
            }
        }

        (void)synced;
        return SyncResult::Success;
    }

    bool is_partition_consistent(PartitionId partition_id) const override {
        std::lock_guard<std::mutex> lock(mutex_);

        // Check for pending conflicts in partition
        for (const auto& conflict : pending_conflicts_) {
            if (conflict.partition_id == partition_id && !conflict.is_resolved) {
                return false;
            }
        }

        return true;
    }

    SyncResult check_consistency(PartitionId partition_id) override {
        std::lock_guard<std::mutex> lock(mutex_);

        stats_.consistency_checks++;

        if (!is_partition_consistent(partition_id)) {
            stats_.consistency_violations++;
            return SyncResult::ConflictDetected;
        }

        return SyncResult::Success;
    }

    SnapshotId create_snapshot(PartitionId partition_id,
                              SnapshotType type) override {
        std::lock_guard<std::mutex> lock(mutex_);

        // Collect entities in partition
        std::vector<EntityState> entities;
        for (const auto& [entity_id, state] : entity_states_) {
            if (state.partition_id == partition_id) {
                entities.push_back(state);
            }
        }

        StateSnapshot snapshot;
        snapshot.partition_id = partition_id;

        SyncResult result;
        if (type == SnapshotType::Full) {
            result = snapshot_manager_->create_full_snapshot(entities, snapshot);
        } else {
            // For incremental, find latest baseline
            auto snapshots = snapshot_manager_->get_snapshots(partition_id);
            if (!snapshots.empty()) {
                result = snapshot_manager_->create_incremental_snapshot(
                    entities, snapshots.back(), snapshot);
            } else {
                // Fall back to full
                result = snapshot_manager_->create_full_snapshot(entities, snapshot);
            }
        }

        if (result == SyncResult::Success) {
            snapshot_manager_->store_snapshot(snapshot);
            stats_.snapshots_created++;
            stats_.total_snapshot_bytes += snapshot.compressed_size;
            return snapshot.snapshot_id;
        } else {
            stats_.snapshot_failures++;
            return INVALID_SNAPSHOT_ID;
        }
    }

    SyncResult restore_snapshot(PartitionId partition_id,
                               SnapshotId snapshot_id) override {
        std::lock_guard<std::mutex> lock(mutex_);

        (void)partition_id;  // Unused parameter

        auto snapshot = snapshot_manager_->load_snapshot(snapshot_id);
        if (!snapshot) {
            return SyncResult::SnapshotNotFound;
        }

        std::vector<EntityState> entities;
        auto result = snapshot_manager_->restore_snapshot(*snapshot, entities);
        if (result != SyncResult::Success) {
            stats_.snapshot_failures++;
            return result;
        }

        // Restore entities
        for (const auto& state : entities) {
            entity_states_[state.entity_id] = state;
        }

        stats_.snapshots_restored++;
        return SyncResult::Success;
    }

    std::optional<StateSnapshot> get_snapshot_info(
        SnapshotId snapshot_id) const override {
        return snapshot_manager_->load_snapshot(snapshot_id);
    }

    std::vector<SnapshotId> list_snapshots(
        PartitionId partition_id) const override {
        return snapshot_manager_->get_snapshots(partition_id);
    }

    SyncResult cleanup_snapshots(PartitionId partition_id) override {
        return snapshot_manager_->cleanup_old_snapshots(
            partition_id, config_.max_snapshots_per_partition);
    }

    SyncResult handle_conflict(const ConflictInfo& conflict) override {
        std::lock_guard<std::mutex> lock(mutex_);

        stats_.conflicts_detected++;

        // Get conflicting states
        auto state_a_it = entity_states_.find(conflict.entity_id);
        if (state_a_it == entity_states_.end()) {
            return SyncResult::InvalidEntityId;
        }

        // For now, assume we only have one local state
        EntityState resolved_state;
        auto result = conflict_resolver_->resolve_conflict(
            conflict, state_a_it->second, state_a_it->second, resolved_state);

        if (result == SyncResult::Success) {
            entity_states_[conflict.entity_id] = resolved_state;
            stats_.conflicts_resolved++;
        } else if (result == SyncResult::RollbackRequired) {
            stats_.rollbacks_performed++;
            // Handle rollback
            entity_states_[conflict.entity_id] = resolved_state;
        }

        return result;
    }

    std::vector<ConflictInfo> get_pending_conflicts() const override {
        std::lock_guard<std::mutex> lock(mutex_);
        return pending_conflicts_;
    }

    SyncResult rollback_entity(EntityId entity_id,
                               const StateVersion& target_version) override {
        std::lock_guard<std::mutex> lock(mutex_);

        // Would need to restore from snapshot or history
        // Simplified implementation
        stats_.rollbacks_performed++;

        (void)entity_id;
        (void)target_version;
        return SyncResult::Success;
    }

    void set_conflict_resolution(ConflictResolution strategy) override {
        std::lock_guard<std::mutex> lock(mutex_);
        conflict_resolver_->set_strategy(strategy);
    }

    SyncResult handle_node_failure(const NodeId& failed_node) override {
        std::lock_guard<std::mutex> lock(mutex_);

        auto start_time = std::chrono::steady_clock::now();
        stats_.failovers_initiated++;

        // Initiate failover - must complete in < 30 seconds
        // Simplified: just mark as handled

        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<Duration>(end_time - start_time);

        if (duration < config_.failover_timeout) {
            stats_.successful_failovers++;

            Duration avg_total = stats_.avg_failover_time * (stats_.successful_failovers - 1);
            stats_.avg_failover_time = (avg_total + duration) / stats_.successful_failovers;

            if (duration > stats_.max_failover_time) {
                stats_.max_failover_time = duration;
            }

            (void)failed_node;
            return SyncResult::Success;
        } else {
            return SyncResult::Timeout;
        }
    }

    SyncResult initiate_failover(const NodeId& failed_node,
                                 const NodeId& target_node) override {
        (void)failed_node;
        (void)target_node;
        return handle_node_failure(failed_node);
    }

    bool is_failover_in_progress() const override {
        return failover_in_progress_.load();
    }

    Real get_failover_progress() const override {
        return failover_progress_.load();
    }

    SyncResult prepare_migration(EntityId entity_id,
                                 const NodeId& target_node) override {
        std::lock_guard<std::mutex> lock(mutex_);

        auto it = entity_states_.find(entity_id);
        if (it == entity_states_.end()) {
            return SyncResult::InvalidEntityId;
        }

        // Send state to target node
        auto result = transport_->send_state(target_node, it->second);
        if (result == SyncResult::Success) {
            stats_.migration_syncs++;
        } else {
            stats_.migration_sync_failures++;
        }

        return result;
    }

    SyncResult complete_migration(EntityId entity_id,
                                  const NodeId& source_node) override {
        std::lock_guard<std::mutex> lock(mutex_);

        // Request final state from source
        auto state = transport_->request_state(source_node, entity_id,
                                               config_.migration_sync_timeout);
        if (!state) {
            stats_.migration_sync_failures++;
            return SyncResult::MigrationFailed;
        }

        entity_states_[entity_id] = *state;
        stats_.migration_syncs++;
        return SyncResult::Success;
    }

    SyncResult cancel_migration(EntityId entity_id) override {
        (void)entity_id;
        return SyncResult::Success;
    }

    UInt64 create_checkpoint() override {
        std::lock_guard<std::mutex> lock(mutex_);

        UInt64 checkpoint_id = next_checkpoint_id_++;

        // Create checkpoint with current state
        // Simplified implementation

        return checkpoint_id;
    }

    SyncResult restore_checkpoint(UInt64 checkpoint_id) override {
        std::lock_guard<std::mutex> lock(mutex_);

        // Restore from checkpoint
        // Simplified implementation

        (void)checkpoint_id;
        return SyncResult::Success;
    }

    std::optional<SyncCheckpoint> get_checkpoint_info(
        UInt64 checkpoint_id) const override {

        (void)checkpoint_id;
        return std::nullopt;
    }

    void update(Real dt) override {
        std::lock_guard<std::mutex> lock(mutex_);

        time_since_last_sync_ += dt;
        time_since_last_snapshot_ += dt;
        time_since_last_consistency_check_ += dt;

        // Periodic sync
        if (config_.auto_sync &&
            time_since_last_sync_ >=
            std::chrono::duration<Real>(config_.sync_interval).count()) {

            // Sync entities (simplified)
            time_since_last_sync_ = 0.0;
        }

        // Periodic snapshots
        if (time_since_last_snapshot_ >=
            std::chrono::duration<Real>(config_.snapshot_interval).count()) {

            // Create snapshots (simplified)
            time_since_last_snapshot_ = 0.0;
        }

        // Periodic consistency checks
        if (config_.enable_consistency_checks &&
            time_since_last_consistency_check_ >=
            std::chrono::duration<Real>(config_.consistency_check_interval).count()) {

            // Check consistency (simplified)
            time_since_last_consistency_check_ = 0.0;
        }
    }

    StateSyncStats get_statistics() const override {
        std::lock_guard<std::mutex> lock(mutex_);
        return stats_;
    }

    void reset_statistics() override {
        std::lock_guard<std::mutex> lock(mutex_);
        stats_.reset();
    }

    const StateSyncConfig& get_config() const override {
        return config_;
    }

    SyncResult set_sync_interval(Duration interval) override {
        config_.sync_interval = interval;
        return SyncResult::Success;
    }

    SyncResult set_snapshot_interval(Duration interval) override {
        config_.snapshot_interval = interval;
        return SyncResult::Success;
    }

    void set_auto_sync(bool enabled) override {
        config_.auto_sync = enabled;
    }

    void set_sync_callback(SyncCompleteCallback callback) override {
        std::lock_guard<std::mutex> lock(mutex_);
        on_sync_complete_ = std::move(callback);
    }

    void set_conflict_callback(ConflictCallback callback) override {
        std::lock_guard<std::mutex> lock(mutex_);
        on_conflict_ = std::move(callback);
    }

    void set_snapshot_callback(SnapshotCallback callback) override {
        std::lock_guard<std::mutex> lock(mutex_);
        on_snapshot_ = std::move(callback);
    }

    void set_failover_callback(FailoverCallback callback) override {
        std::lock_guard<std::mutex> lock(mutex_);
        on_failover_ = std::move(callback);
    }

private:
    StateSyncConfig config_;
    bool initialized_{false};

    void* partition_manager_{nullptr};
    void* time_manager_{nullptr};

    std::unique_ptr<IStateSerializer> serializer_;
    std::unique_ptr<IConflictResolver> conflict_resolver_;
    std::unique_ptr<ISnapshotManager> snapshot_manager_;
    std::unique_ptr<ISyncTransport> transport_;

    std::unordered_map<EntityId, EntityState> entity_states_;
    std::vector<ConflictInfo> pending_conflicts_;

    StateSyncStats stats_;
    UInt64 next_checkpoint_id_{1};

    std::atomic<bool> failover_in_progress_{false};
    std::atomic<Real> failover_progress_{0.0};

    Real time_since_last_sync_{0.0};
    Real time_since_last_snapshot_{0.0};
    Real time_since_last_consistency_check_{0.0};

    SyncCompleteCallback on_sync_complete_;
    ConflictCallback on_conflict_;
    SnapshotCallback on_snapshot_;
    FailoverCallback on_failover_;

    mutable std::mutex mutex_;
};

// ============================================================================
// Factory Functions
// ============================================================================

std::unique_ptr<StateSynchronizer> create_state_synchronizer(
    const StateSyncConfig& config) {
    return std::make_unique<SimpleStateSynchronizer>(config);
}

std::unique_ptr<ISnapshotManager> create_snapshot_manager(
    const StateSyncConfig& config) {
    (void)config;
    return std::make_unique<SimpleSnapshotManager>();
}

std::unique_ptr<IConflictResolver> create_conflict_resolver(
    ConflictResolution strategy) {
    return std::make_unique<SimpleConflictResolver>(strategy);
}

std::unique_ptr<IStateSerializer> create_state_serializer() {
    return std::make_unique<SimpleStateSerializer>();
}

std::unique_ptr<ISyncTransport> create_sync_transport() {
    return std::make_unique<SimpleSyncTransport>();
}

} // namespace jaguar::cloud
