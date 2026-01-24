/**
 * @file history_store.cpp
 * @brief Implementation of entity history store for time-series state snapshots
 */

#include "jaguar/thread/history_store.h"
#include <algorithm>
#include <mutex>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <fstream>
#include <cstring>

namespace jaguar::thread {

// ============================================================================
// Helper Functions Implementation
// ============================================================================

UInt64 compute_state_hash(const std::vector<UInt8>& data) {
    // Simple FNV-1a hash
    UInt64 hash = 14695981039346656037ULL;
    for (UInt8 byte : data) {
        hash ^= byte;
        hash *= 1099511628211ULL;
    }
    return hash;
}

bool verify_snapshot_integrity(const StateSnapshot& snapshot) {
    return snapshot.state_hash == compute_state_hash(snapshot.state_data);
}

std::vector<UInt8> compress_snapshot_data(const std::vector<UInt8>& data) {
    // Simple RLE compression for demonstration
    // In production, would use zlib or similar
    if (data.empty()) {
        return data;
    }

    std::vector<UInt8> compressed;
    compressed.reserve(data.size() / 2);

    UInt8 current = data[0];
    UInt8 count = 1;

    for (size_t i = 1; i < data.size(); ++i) {
        if (data[i] == current && count < 255) {
            ++count;
        } else {
            compressed.push_back(count);
            compressed.push_back(current);
            current = data[i];
            count = 1;
        }
    }

    compressed.push_back(count);
    compressed.push_back(current);

    return compressed.size() < data.size() ? compressed : data;
}

std::vector<UInt8> decompress_snapshot_data(const std::vector<UInt8>& data) {
    // Simple RLE decompression
    if (data.size() % 2 != 0) {
        return data; // Not compressed or corrupt
    }

    std::vector<UInt8> decompressed;
    for (size_t i = 0; i < data.size(); i += 2) {
        UInt8 count = data[i];
        UInt8 value = data[i + 1];
        for (UInt8 j = 0; j < count; ++j) {
            decompressed.push_back(value);
        }
    }

    return decompressed;
}

bool StateSnapshot::verify_integrity() const {
    return verify_snapshot_integrity(*this);
}

// ============================================================================
// SimpleHistoryBackend Implementation
// ============================================================================

class SimpleHistoryBackend : public IHistoryBackend {
public:
    SimpleHistoryBackend() : total_size_bytes_(0) {}
    ~SimpleHistoryBackend() override = default;

    HistoryResult store(const StateSnapshot& snapshot) override {
        std::lock_guard<std::mutex> lock(mutex_);

        try {
            auto& entity_snapshots = snapshots_[snapshot.entity_id];
            entity_snapshots[snapshot.sequence_number] = snapshot;
            total_size_bytes_ += snapshot.size_bytes();

            return HistoryResult::Success;
        } catch (const std::bad_alloc&) {
            return HistoryResult::OutOfMemory;
        } catch (...) {
            return HistoryResult::StoreFailed;
        }
    }

    std::optional<StateSnapshot> get(EntityId entity_id, UInt64 sequence) const override {
        std::lock_guard<std::mutex> lock(mutex_);

        auto entity_it = snapshots_.find(entity_id);
        if (entity_it == snapshots_.end()) {
            return std::nullopt;
        }

        auto snapshot_it = entity_it->second.find(sequence);
        if (snapshot_it == entity_it->second.end()) {
            return std::nullopt;
        }

        return snapshot_it->second;
    }

    HistoryQueryResult query(const HistoryQuery& query) const override {
        std::lock_guard<std::mutex> lock(mutex_);

        std::vector<StateSnapshot> results;
        UInt64 total_count = 0;

        // Collect all matching snapshots
        std::vector<StateSnapshot> all_matches;

        for (const auto& [entity_id, entity_snapshots] : snapshots_) {
            // Filter by entity_id if specified
            if (query.entity_id.has_value() && entity_id != query.entity_id.value()) {
                continue;
            }

            for (const auto& [sequence, snapshot] : entity_snapshots) {
                // Filter by time range
                if (query.time_range.has_value()) {
                    if (!query.time_range->contains(snapshot.timestamp)) {
                        continue;
                    }
                }

                // Filter by snapshot type
                if (query.snapshot_type.has_value()) {
                    if (snapshot.type != query.snapshot_type.value()) {
                        continue;
                    }
                }

                all_matches.push_back(snapshot);
            }
        }

        total_count = all_matches.size();

        // Sort by timestamp
        if (query.ascending) {
            std::sort(all_matches.begin(), all_matches.end(),
                [](const StateSnapshot& a, const StateSnapshot& b) {
                    return a.timestamp < b.timestamp;
                });
        } else {
            std::sort(all_matches.begin(), all_matches.end(),
                [](const StateSnapshot& a, const StateSnapshot& b) {
                    return a.timestamp > b.timestamp;
                });
        }

        // Apply offset and limit
        UInt32 start_idx = std::min(query.offset, static_cast<UInt32>(all_matches.size()));
        UInt32 end_idx = std::min(start_idx + query.limit, static_cast<UInt32>(all_matches.size()));

        for (UInt32 i = start_idx; i < end_idx; ++i) {
            results.push_back(all_matches[i]);
        }

        bool has_more = end_idx < all_matches.size();

        return HistoryQueryResult(std::move(results), total_count, has_more);
    }

    HistoryResult remove(EntityId entity_id, UInt64 sequence) override {
        std::lock_guard<std::mutex> lock(mutex_);

        auto entity_it = snapshots_.find(entity_id);
        if (entity_it == snapshots_.end()) {
            return HistoryResult::EntityNotFound;
        }

        auto snapshot_it = entity_it->second.find(sequence);
        if (snapshot_it == entity_it->second.end()) {
            return HistoryResult::SnapshotNotFound;
        }

        total_size_bytes_ -= snapshot_it->second.size_bytes();
        entity_it->second.erase(snapshot_it);

        if (entity_it->second.empty()) {
            snapshots_.erase(entity_it);
        }

        return HistoryResult::Success;
    }

    HistoryResult remove_before(std::chrono::system_clock::time_point cutoff_time) override {
        std::lock_guard<std::mutex> lock(mutex_);

        try {
            for (auto entity_it = snapshots_.begin(); entity_it != snapshots_.end(); ) {
                auto& entity_snapshots = entity_it->second;

                for (auto snapshot_it = entity_snapshots.begin();
                     snapshot_it != entity_snapshots.end(); ) {
                    if (snapshot_it->second.timestamp < cutoff_time) {
                        total_size_bytes_ -= snapshot_it->second.size_bytes();
                        snapshot_it = entity_snapshots.erase(snapshot_it);
                    } else {
                        ++snapshot_it;
                    }
                }

                if (entity_snapshots.empty()) {
                    entity_it = snapshots_.erase(entity_it);
                } else {
                    ++entity_it;
                }
            }

            return HistoryResult::Success;
        } catch (...) {
            return HistoryResult::DeleteFailed;
        }
    }

    UInt64 count() const override {
        std::lock_guard<std::mutex> lock(mutex_);

        UInt64 total = 0;
        for (const auto& [entity_id, entity_snapshots] : snapshots_) {
            total += entity_snapshots.size();
        }
        return total;
    }

    UInt64 size_bytes() const override {
        std::lock_guard<std::mutex> lock(mutex_);
        return total_size_bytes_;
    }

    // Additional helper methods
    UInt64 entity_count() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return snapshots_.size();
    }

    UInt64 entity_snapshot_count(EntityId entity_id) const {
        std::lock_guard<std::mutex> lock(mutex_);

        auto it = snapshots_.find(entity_id);
        if (it == snapshots_.end()) {
            return 0;
        }
        return it->second.size();
    }

    bool has_entity(EntityId entity_id) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return snapshots_.find(entity_id) != snapshots_.end();
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshots_.clear();
        total_size_bytes_ = 0;
    }

private:
    mutable std::mutex mutex_;
    std::unordered_map<EntityId, std::map<UInt64, StateSnapshot>> snapshots_;
    UInt64 total_size_bytes_;
};

// ============================================================================
// SimpleParquetExporter Implementation
// ============================================================================

class SimpleParquetExporter : public IHistoryExporter {
public:
    SimpleParquetExporter() = default;
    ~SimpleParquetExporter() override = default;

    HistoryResult export_data(const ExportConfig& config) override {
        switch (config.format) {
            case ExportFormat::CSV:
                return export_csv(config);
            case ExportFormat::JSON:
                return export_json(config);
            case ExportFormat::Parquet:
                // Parquet requires Arrow library - return not supported
                return HistoryResult::ExportNotSupported;
            case ExportFormat::Binary:
                return export_binary(config);
            default:
                return HistoryResult::ExportNotSupported;
        }
    }

    bool supports_format(ExportFormat format) const override {
        return format == ExportFormat::CSV ||
               format == ExportFormat::JSON ||
               format == ExportFormat::Binary;
    }

private:
    HistoryResult export_csv(const ExportConfig& config) {
        try {
            std::ofstream file(config.output_path);
            if (!file.is_open()) {
                return HistoryResult::ExportIOError;
            }

            // Write CSV header
            file << "entity_id,sequence_number,type,timestamp,state_hash,state_size,source_node\n";

            // Note: In real implementation, would query the backend passed in config
            // For now, this is a stub showing the structure

            file.close();
            return HistoryResult::Success;
        } catch (...) {
            return HistoryResult::ExportIOError;
        }
    }

    HistoryResult export_json(const ExportConfig& config) {
        try {
            std::ofstream file(config.output_path);
            if (!file.is_open()) {
                return HistoryResult::ExportIOError;
            }

            // JSON Lines format (newline-delimited JSON)
            // Note: In real implementation, would query the backend passed in config

            file.close();
            return HistoryResult::Success;
        } catch (...) {
            return HistoryResult::ExportIOError;
        }
    }

    HistoryResult export_binary(const ExportConfig& config) {
        try {
            std::ofstream file(config.output_path, std::ios::binary);
            if (!file.is_open()) {
                return HistoryResult::ExportIOError;
            }

            // Binary format: simple serialization
            // Note: In real implementation, would query the backend passed in config

            file.close();
            return HistoryResult::Success;
        } catch (...) {
            return HistoryResult::ExportIOError;
        }
    }
};

// ============================================================================
// HistoryStore Implementation (pImpl)
// ============================================================================

// Main HistoryStore concrete implementation
class HistoryStoreImpl : public HistoryStore {
private:
    struct Impl {
        // Configuration
        HistoryStoreConfig config;

        // Components
        std::shared_ptr<IHistoryBackend> backend;
        std::shared_ptr<IHistoryExporter> exporter;

        // State
        std::atomic<bool> initialized{false};
        std::mutex mutex;
        UInt64 next_sequence{1};

        Impl(const HistoryStoreConfig& cfg)
            : config(cfg)
            , backend(nullptr)
            , exporter(nullptr) {}
    };

    std::unique_ptr<Impl> impl_;

public:
    explicit HistoryStoreImpl(const HistoryStoreConfig& config)
        : impl_(std::make_unique<Impl>(config)) {}

    ~HistoryStoreImpl() override {
        if (impl_->initialized.load()) {
            shutdown();
        }
    }

    // ========================================================================
    // Lifecycle
    // ========================================================================

    HistoryResult initialize() override {
        std::lock_guard<std::mutex> lock(impl_->mutex);

        if (impl_->initialized.load()) {
            return HistoryResult::AlreadyInitialized;
        }

        // Create default backend if not set
        if (!impl_->backend) {
            impl_->backend = create_memory_backend();
        }

        // Create default exporter if not set
        if (!impl_->exporter) {
            impl_->exporter = create_parquet_exporter();
        }

        impl_->initialized.store(true);
        return HistoryResult::Success;
    }

    HistoryResult shutdown() override {
        std::lock_guard<std::mutex> lock(impl_->mutex);

        if (!impl_->initialized.load()) {
            return HistoryResult::NotInitialized;
        }

        impl_->initialized.store(false);
        return HistoryResult::Success;
    }

    bool is_initialized() const override {
        return impl_->initialized.load();
    }

    // ========================================================================
    // Snapshot Storage
    // ========================================================================

    HistoryResult store_snapshot(StateSnapshot snapshot) override {
        if (!impl_->initialized.load()) {
            return HistoryResult::NotInitialized;
        }

        if (snapshot.entity_id == INVALID_ENTITY_ID) {
            return HistoryResult::InvalidEntityId;
        }

        // Assign sequence number if not set
        if (snapshot.sequence_number == 0) {
            std::lock_guard<std::mutex> lock(impl_->mutex);
            snapshot.sequence_number = impl_->next_sequence++;
        }

        // Compute hash if not set
        if (snapshot.state_hash == 0 && !snapshot.state_data.empty()) {
            snapshot.state_hash = compute_state_hash(snapshot.state_data);
        }

        // Check capacity
        if (impl_->backend->count() >= impl_->config.max_total_snapshots) {
            return HistoryResult::CapacityExceeded;
        }

        if (impl_->backend->size_bytes() >= impl_->config.max_memory_bytes) {
            return HistoryResult::CapacityExceeded;
        }

        return impl_->backend->store(snapshot);
    }

    HistoryResult store_snapshots(const std::vector<StateSnapshot>& snapshots) override {
        if (!impl_->initialized.load()) {
            return HistoryResult::NotInitialized;
        }

        for (const auto& snapshot : snapshots) {
            auto result = store_snapshot(snapshot);
            if (result != HistoryResult::Success) {
                return result;
            }
        }

        return HistoryResult::Success;
    }

    // ========================================================================
    // Snapshot Retrieval
    // ========================================================================

    std::optional<StateSnapshot> get_snapshot(EntityId entity_id, UInt64 sequence) const override {
        if (!impl_->initialized.load()) {
            return std::nullopt;
        }

        return impl_->backend->get(entity_id, sequence);
    }

    std::optional<StateSnapshot> get_latest_snapshot(EntityId entity_id) const override {
        if (!impl_->initialized.load()) {
            return std::nullopt;
        }

        HistoryQuery query = HistoryQuery::for_entity(entity_id, 1);
        query.ascending = false; // Descending to get latest first

        auto result = impl_->backend->query(query);
        if (result.snapshots.empty()) {
            return std::nullopt;
        }

        return result.snapshots[0];
    }

    std::optional<StateSnapshot> get_first_snapshot(EntityId entity_id) const override {
        if (!impl_->initialized.load()) {
            return std::nullopt;
        }

        HistoryQuery query = HistoryQuery::for_entity(entity_id, 1);
        query.ascending = true; // Ascending to get first

        auto result = impl_->backend->query(query);
        if (result.snapshots.empty()) {
            return std::nullopt;
        }

        return result.snapshots[0];
    }

    std::optional<StateSnapshot> get_snapshot_at_time(
        EntityId entity_id,
        std::chrono::system_clock::time_point time) const override {

        if (!impl_->initialized.load()) {
            return std::nullopt;
        }

        HistoryQuery query = HistoryQuery::for_entity(entity_id, 1000);

        auto result = impl_->backend->query(query);
        if (result.snapshots.empty()) {
            return std::nullopt;
        }

        // Find closest snapshot to target time
        std::optional<StateSnapshot> closest;
        auto min_diff = std::chrono::system_clock::duration::max();

        for (const auto& snapshot : result.snapshots) {
            auto diff = snapshot.timestamp > time
                ? snapshot.timestamp - time
                : time - snapshot.timestamp;

            if (diff < min_diff) {
                min_diff = diff;
                closest = snapshot;
            }
        }

        return closest;
    }

    // ========================================================================
    // Querying
    // ========================================================================

    HistoryQueryResult query(const HistoryQuery& query) const override {
        if (!impl_->initialized.load()) {
            return HistoryQueryResult();
        }

        return impl_->backend->query(query);
    }

    HistoryQueryResult get_entity_history(EntityId entity_id, UInt32 limit) const override {
        if (!impl_->initialized.load()) {
            return HistoryQueryResult();
        }

        HistoryQuery query = HistoryQuery::for_entity(entity_id, limit);
        return impl_->backend->query(query);
    }

    HistoryQueryResult get_snapshots_in_range(const QueryTimeRange& time_range,
                                             UInt32 limit) const override {
        if (!impl_->initialized.load()) {
            return HistoryQueryResult();
        }

        HistoryQuery query = HistoryQuery::for_time_range(time_range, limit);
        return impl_->backend->query(query);
    }

    // ========================================================================
    // Deletion
    // ========================================================================

    HistoryResult delete_entity_history(EntityId entity_id) override {
        if (!impl_->initialized.load()) {
            return HistoryResult::NotInitialized;
        }

        // Query all snapshots for entity
        HistoryQuery query = HistoryQuery::for_entity(entity_id, UINT32_MAX);
        auto result = impl_->backend->query(query);

        // Delete each snapshot
        for (const auto& snapshot : result.snapshots) {
            auto del_result = impl_->backend->remove(entity_id, snapshot.sequence_number);
            if (del_result != HistoryResult::Success) {
                return del_result;
            }
        }

        return HistoryResult::Success;
    }

    HistoryResult delete_snapshot(EntityId entity_id, UInt64 sequence) override {
        if (!impl_->initialized.load()) {
            return HistoryResult::NotInitialized;
        }

        return impl_->backend->remove(entity_id, sequence);
    }

    HistoryResult delete_before(std::chrono::system_clock::time_point cutoff_time) override {
        if (!impl_->initialized.load()) {
            return HistoryResult::NotInitialized;
        }

        return impl_->backend->remove_before(cutoff_time);
    }

    HistoryResult clear() override {
        if (!impl_->initialized.load()) {
            return HistoryResult::NotInitialized;
        }

        auto* simple_backend = dynamic_cast<SimpleHistoryBackend*>(impl_->backend.get());
        if (simple_backend) {
            simple_backend->clear();
            return HistoryResult::Success;
        }

        // Fallback: delete everything through remove_before
        auto far_future = std::chrono::system_clock::now() + std::chrono::hours(1000000);
        return impl_->backend->remove_before(far_future);
    }

    // ========================================================================
    // Retention Management
    // ========================================================================

    HistoryResult apply_retention() override {
        if (!impl_->initialized.load()) {
            return HistoryResult::NotInitialized;
        }

        auto current_time = std::chrono::system_clock::now();
        const auto& policy = impl_->config.retention;

        // Query all snapshots
        HistoryQuery query;
        query.limit = UINT32_MAX;
        auto result = impl_->backend->query(query);

        // Delete snapshots that violate retention policy
        for (const auto& snapshot : result.snapshots) {
            if (!policy.should_retain(snapshot, current_time)) {
                impl_->backend->remove(snapshot.entity_id, snapshot.sequence_number);
            }
        }

        // Check per-entity limits
        std::unordered_map<EntityId, std::vector<StateSnapshot>> entity_snapshots;

        // Re-query after retention cleanup
        result = impl_->backend->query(query);
        for (const auto& snapshot : result.snapshots) {
            entity_snapshots[snapshot.entity_id].push_back(snapshot);
        }

        // Enforce max_snapshots_per_entity
        for (auto& [entity_id, snapshots] : entity_snapshots) {
            if (snapshots.size() > policy.max_snapshots_per_entity) {
                // Sort by timestamp (oldest first)
                std::sort(snapshots.begin(), snapshots.end(),
                    [](const StateSnapshot& a, const StateSnapshot& b) {
                        return a.timestamp < b.timestamp;
                    });

                // Delete oldest snapshots beyond limit
                UInt64 to_delete = snapshots.size() - policy.max_snapshots_per_entity;
                for (UInt64 i = 0; i < to_delete; ++i) {
                    impl_->backend->remove(entity_id, snapshots[i].sequence_number);
                }
            }
        }

        return HistoryResult::Success;
    }

    HistoryResult set_retention_policy(const RetentionPolicy& policy) override {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->config.retention = policy;
        return HistoryResult::Success;
    }

    const RetentionPolicy& get_retention_policy() const override {
        return impl_->config.retention;
    }

    // ========================================================================
    // Export
    // ========================================================================

    HistoryResult export_history(const ExportConfig& config) override {
        if (!impl_->initialized.load()) {
            return HistoryResult::NotInitialized;
        }

        if (!impl_->exporter) {
            return HistoryResult::ExportNotSupported;
        }

        if (!impl_->exporter->supports_format(config.format)) {
            return HistoryResult::ExportNotSupported;
        }

        // Query snapshots matching export criteria
        auto result = impl_->backend->query(config.query);

        // Create a modified config with the actual query results
        // In a real implementation, the exporter would have access to the backend
        // and handle the querying itself. For this stub, we just delegate.

        return impl_->exporter->export_data(config);
    }

    // ========================================================================
    // Backend Management
    // ========================================================================

    HistoryResult set_backend(std::shared_ptr<IHistoryBackend> backend) override {
        std::lock_guard<std::mutex> lock(impl_->mutex);

        if (impl_->initialized.load()) {
            return HistoryResult::AlreadyInitialized;
        }

        impl_->backend = std::move(backend);
        return HistoryResult::Success;
    }

    std::shared_ptr<IHistoryBackend> get_backend() const override {
        return impl_->backend;
    }

    HistoryResult set_exporter(std::shared_ptr<IHistoryExporter> exporter) override {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->exporter = std::move(exporter);
        return HistoryResult::Success;
    }

    std::shared_ptr<IHistoryExporter> get_exporter() const override {
        return impl_->exporter;
    }

    // ========================================================================
    // Statistics
    // ========================================================================

    HistoryStoreStats get_stats() const override {
        if (!impl_->initialized.load()) {
            return HistoryStoreStats();
        }

        HistoryStoreStats stats;
        stats.total_snapshots = impl_->backend->count();
        stats.storage_bytes = impl_->backend->size_bytes();

        auto* simple_backend = dynamic_cast<SimpleHistoryBackend*>(impl_->backend.get());
        if (simple_backend) {
            stats.total_entities = simple_backend->entity_count();
        }

        // Count snapshots by type
        HistoryQuery query;
        query.limit = UINT32_MAX;
        auto result = impl_->backend->query(query);

        for (const auto& snapshot : result.snapshots) {
            stats.snapshots_by_type[snapshot.type]++;
        }

        // Find oldest snapshot
        if (!result.snapshots.empty()) {
            auto oldest = std::min_element(result.snapshots.begin(), result.snapshots.end(),
                [](const StateSnapshot& a, const StateSnapshot& b) {
                    return a.timestamp < b.timestamp;
                });

            if (oldest != result.snapshots.end()) {
                stats.oldest_snapshot_age_hours =
                    calculate_snapshot_age_hours(*oldest);
            }
        }

        return stats;
    }

    const HistoryStoreConfig& get_config() const override {
        return impl_->config;
    }

    UInt64 get_entity_count() const override {
        if (!impl_->initialized.load()) {
            return 0;
        }

        auto* simple_backend = dynamic_cast<SimpleHistoryBackend*>(impl_->backend.get());
        if (simple_backend) {
            return simple_backend->entity_count();
        }

        // Fallback: count unique entities from query
        HistoryQuery query;
        query.limit = UINT32_MAX;
        auto result = impl_->backend->query(query);

        std::unordered_set<EntityId> unique_entities;
        for (const auto& snapshot : result.snapshots) {
            unique_entities.insert(snapshot.entity_id);
        }

        return unique_entities.size();
    }

    UInt64 get_entity_snapshot_count(EntityId entity_id) const override {
        if (!impl_->initialized.load()) {
            return 0;
        }

        auto* simple_backend = dynamic_cast<SimpleHistoryBackend*>(impl_->backend.get());
        if (simple_backend) {
            return simple_backend->entity_snapshot_count(entity_id);
        }

        // Fallback: query and count
        HistoryQuery query = HistoryQuery::for_entity(entity_id, UINT32_MAX);
        auto result = impl_->backend->query(query);
        return result.snapshots.size();
    }

    bool has_entity_history(EntityId entity_id) const override {
        if (!impl_->initialized.load()) {
            return false;
        }

        auto* simple_backend = dynamic_cast<SimpleHistoryBackend*>(impl_->backend.get());
        if (simple_backend) {
            return simple_backend->has_entity(entity_id);
        }

        // Fallback: check if any snapshots exist
        HistoryQuery query = HistoryQuery::for_entity(entity_id, 1);
        auto result = impl_->backend->query(query);
        return !result.snapshots.empty();
    }
};

// ============================================================================
// Factory Functions Implementation
// ============================================================================

std::unique_ptr<HistoryStore> create_history_store(const HistoryStoreConfig& config) {
    return std::make_unique<HistoryStoreImpl>(config);
}

std::unique_ptr<IHistoryBackend> create_memory_backend() {
    return std::make_unique<SimpleHistoryBackend>();
}

std::unique_ptr<IHistoryBackend> create_disk_backend(const std::string& storage_path) {
    // Stub: would implement disk-backed storage
    (void)storage_path;
    // For now, return memory backend
    return std::make_unique<SimpleHistoryBackend>();
}

std::unique_ptr<IHistoryExporter> create_parquet_exporter() {
    return std::make_unique<SimpleParquetExporter>();
}

std::unique_ptr<IHistoryExporter> create_csv_exporter() {
    return std::make_unique<SimpleParquetExporter>();
}

std::unique_ptr<IHistoryExporter> create_json_exporter() {
    return std::make_unique<SimpleParquetExporter>();
}

std::unique_ptr<IHistoryExporter> create_multi_format_exporter() {
    return std::make_unique<SimpleParquetExporter>();
}

} // namespace jaguar::thread
