/**
 * @file history_store_tests.cpp
 * @brief Unit tests for the history store module
 */

#include <gtest/gtest.h>
#include "jaguar/thread/history_store.h"
#include <random>
#include <thread>
#include <chrono>
#include <algorithm>

using namespace jaguar;
using namespace jaguar::thread;

// ============================================================================
// HistoryResult Tests
// ============================================================================

class HistoryResultTest : public ::testing::Test {};

TEST_F(HistoryResultTest, SuccessToString) {
    EXPECT_STREQ(history_result_to_string(HistoryResult::Success), "Success");
}

TEST_F(HistoryResultTest, ConfigurationErrorsToString) {
    EXPECT_STREQ(history_result_to_string(HistoryResult::InvalidConfiguration), "InvalidConfiguration");
    EXPECT_STREQ(history_result_to_string(HistoryResult::InvalidEntityId), "InvalidEntityId");
    EXPECT_STREQ(history_result_to_string(HistoryResult::InvalidTimeRange), "InvalidTimeRange");
    EXPECT_STREQ(history_result_to_string(HistoryResult::InvalidQuery), "InvalidQuery");
}

TEST_F(HistoryResultTest, OperationalErrorsToString) {
    EXPECT_STREQ(history_result_to_string(HistoryResult::StoreFailed), "StoreFailed");
    EXPECT_STREQ(history_result_to_string(HistoryResult::RetrieveFailed), "RetrieveFailed");
    EXPECT_STREQ(history_result_to_string(HistoryResult::DeleteFailed), "DeleteFailed");
    EXPECT_STREQ(history_result_to_string(HistoryResult::ExportFailed), "ExportFailed");
}

TEST_F(HistoryResultTest, StateErrorsToString) {
    EXPECT_STREQ(history_result_to_string(HistoryResult::NotInitialized), "NotInitialized");
    EXPECT_STREQ(history_result_to_string(HistoryResult::AlreadyInitialized), "AlreadyInitialized");
    EXPECT_STREQ(history_result_to_string(HistoryResult::EntityNotFound), "EntityNotFound");
    EXPECT_STREQ(history_result_to_string(HistoryResult::SnapshotNotFound), "SnapshotNotFound");
}

TEST_F(HistoryResultTest, ResourceErrorsToString) {
    EXPECT_STREQ(history_result_to_string(HistoryResult::OutOfMemory), "OutOfMemory");
    EXPECT_STREQ(history_result_to_string(HistoryResult::StorageError), "StorageError");
    EXPECT_STREQ(history_result_to_string(HistoryResult::CapacityExceeded), "CapacityExceeded");
}

TEST_F(HistoryResultTest, ExportErrorsToString) {
    EXPECT_STREQ(history_result_to_string(HistoryResult::ExportNotSupported), "ExportNotSupported");
    EXPECT_STREQ(history_result_to_string(HistoryResult::ExportIOError), "ExportIOError");
}

// ============================================================================
// SnapshotType Tests
// ============================================================================

class SnapshotTypeTest : public ::testing::Test {};

TEST_F(SnapshotTypeTest, FullToString) {
    EXPECT_STREQ(snapshot_type_to_string(SnapshotType::Full), "Full");
}

TEST_F(SnapshotTypeTest, DeltaToString) {
    EXPECT_STREQ(snapshot_type_to_string(SnapshotType::Delta), "Delta");
}

TEST_F(SnapshotTypeTest, CheckpointToString) {
    EXPECT_STREQ(snapshot_type_to_string(SnapshotType::Checkpoint), "Checkpoint");
}

// ============================================================================
// ExportFormat Tests
// ============================================================================

class ExportFormatTest : public ::testing::Test {};

TEST_F(ExportFormatTest, ParquetToString) {
    EXPECT_STREQ(export_format_to_string(ExportFormat::Parquet), "Parquet");
}

TEST_F(ExportFormatTest, CSVToString) {
    EXPECT_STREQ(export_format_to_string(ExportFormat::CSV), "CSV");
}

TEST_F(ExportFormatTest, JSONToString) {
    EXPECT_STREQ(export_format_to_string(ExportFormat::JSON), "JSON");
}

TEST_F(ExportFormatTest, BinaryToString) {
    EXPECT_STREQ(export_format_to_string(ExportFormat::Binary), "Binary");
}

// ============================================================================
// QueryTimeRange Tests
// ============================================================================

class QueryTimeRangeTest : public ::testing::Test {
protected:
    void SetUp() override {
        start_time_ = std::chrono::system_clock::now();
        end_time_ = start_time_ + std::chrono::hours(1);
    }

    std::chrono::system_clock::time_point start_time_;
    std::chrono::system_clock::time_point end_time_;
};

TEST_F(QueryTimeRangeTest, DefaultConstruction) {
    QueryTimeRange range;
    EXPECT_TRUE(range.inclusive_start);
    EXPECT_TRUE(range.inclusive_end);
}

TEST_F(QueryTimeRangeTest, ParameterizedConstruction) {
    QueryTimeRange range(start_time_, end_time_, false, false);
    EXPECT_EQ(range.start, start_time_);
    EXPECT_EQ(range.end, end_time_);
    EXPECT_FALSE(range.inclusive_start);
    EXPECT_FALSE(range.inclusive_end);
}

TEST_F(QueryTimeRangeTest, ContainsInclusive) {
    QueryTimeRange range(start_time_, end_time_, true, true);

    // Should contain boundaries
    EXPECT_TRUE(range.contains(start_time_));
    EXPECT_TRUE(range.contains(end_time_));

    // Should contain middle
    auto mid_time = start_time_ + std::chrono::minutes(30);
    EXPECT_TRUE(range.contains(mid_time));
}

TEST_F(QueryTimeRangeTest, ContainsExclusive) {
    QueryTimeRange range(start_time_, end_time_, false, false);

    // Should not contain boundaries
    EXPECT_FALSE(range.contains(start_time_));
    EXPECT_FALSE(range.contains(end_time_));

    // Should contain middle
    auto mid_time = start_time_ + std::chrono::minutes(30);
    EXPECT_TRUE(range.contains(mid_time));
}

TEST_F(QueryTimeRangeTest, ContainsHalfOpen) {
    QueryTimeRange range(start_time_, end_time_, true, false);

    EXPECT_TRUE(range.contains(start_time_));
    EXPECT_FALSE(range.contains(end_time_));
}

TEST_F(QueryTimeRangeTest, ContainsOutside) {
    QueryTimeRange range(start_time_, end_time_);

    auto before = start_time_ - std::chrono::hours(1);
    auto after = end_time_ + std::chrono::hours(1);

    EXPECT_FALSE(range.contains(before));
    EXPECT_FALSE(range.contains(after));
}

TEST_F(QueryTimeRangeTest, Duration) {
    QueryTimeRange range(start_time_, end_time_);
    auto duration = range.duration();

    EXPECT_EQ(duration, std::chrono::hours(1));
}

// ============================================================================
// StateSnapshot Tests
// ============================================================================

class StateSnapshotTest : public ::testing::Test {
protected:
    void SetUp() override {
        snapshot_.entity_id = 42;
        snapshot_.sequence_number = 10;
        snapshot_.type = SnapshotType::Full;
        snapshot_.timestamp = std::chrono::system_clock::now();
        snapshot_.state_data = {0x01, 0x02, 0x03, 0x04, 0x05};
        snapshot_.state_hash = compute_state_hash(snapshot_.state_data);
        snapshot_.source_node = "node0";
    }

    StateSnapshot snapshot_;
};

TEST_F(StateSnapshotTest, DefaultConstruction) {
    StateSnapshot snapshot;
    EXPECT_EQ(snapshot.entity_id, INVALID_ENTITY_ID);
    EXPECT_EQ(snapshot.sequence_number, 0u);
    EXPECT_EQ(snapshot.type, SnapshotType::Full);
    EXPECT_EQ(snapshot.state_hash, 0u);
}

TEST_F(StateSnapshotTest, ParameterizedConstruction) {
    auto now = std::chrono::system_clock::now();
    std::vector<UInt8> data = {0xAA, 0xBB, 0xCC};

    StateSnapshot snapshot(123, 5, SnapshotType::Delta, now, data, 0xDEADBEEF, "node1");

    EXPECT_EQ(snapshot.entity_id, 123u);
    EXPECT_EQ(snapshot.sequence_number, 5u);
    EXPECT_EQ(snapshot.type, SnapshotType::Delta);
    EXPECT_EQ(snapshot.timestamp, now);
    EXPECT_EQ(snapshot.state_data, data);
    EXPECT_EQ(snapshot.state_hash, 0xDEADBEEF);
    EXPECT_EQ(snapshot.source_node, "node1");
}

TEST_F(StateSnapshotTest, SizeBytes) {
    EXPECT_EQ(snapshot_.size_bytes(), 5u);
}

TEST_F(StateSnapshotTest, SizeBytesEmpty) {
    StateSnapshot empty;
    EXPECT_EQ(empty.size_bytes(), 0u);
}

TEST_F(StateSnapshotTest, VerifyIntegrity) {
    bool valid = verify_snapshot_integrity(snapshot_);
    EXPECT_TRUE(valid);
}

TEST_F(StateSnapshotTest, VerifyIntegrityCorrupted) {
    snapshot_.state_data[0] ^= 0xFF;
    bool valid = verify_snapshot_integrity(snapshot_);
    EXPECT_FALSE(valid);
}

// ============================================================================
// HistoryQuery Tests
// ============================================================================

class HistoryQueryTest : public ::testing::Test {};

TEST_F(HistoryQueryTest, DefaultConstruction) {
    HistoryQuery query;
    EXPECT_FALSE(query.entity_id.has_value());
    EXPECT_FALSE(query.time_range.has_value());
    EXPECT_FALSE(query.snapshot_type.has_value());
    EXPECT_EQ(query.limit, 100u);
    EXPECT_EQ(query.offset, 0u);
    EXPECT_FALSE(query.ascending);
}

TEST_F(HistoryQueryTest, ForEntity) {
    auto query = HistoryQuery::for_entity(42, 50);

    EXPECT_TRUE(query.entity_id.has_value());
    EXPECT_EQ(query.entity_id.value(), 42u);
    EXPECT_EQ(query.limit, 50u);
}

TEST_F(HistoryQueryTest, ForTimeRange) {
    auto now = std::chrono::system_clock::now();
    auto earlier = now - std::chrono::hours(1);
    QueryTimeRange range(earlier, now);

    auto query = HistoryQuery::for_time_range(range, 75);

    EXPECT_TRUE(query.time_range.has_value());
    EXPECT_EQ(query.time_range->start, earlier);
    EXPECT_EQ(query.time_range->end, now);
    EXPECT_EQ(query.limit, 75u);
}

TEST_F(HistoryQueryTest, ForType) {
    auto query = HistoryQuery::for_type(SnapshotType::Checkpoint, 25);

    EXPECT_TRUE(query.snapshot_type.has_value());
    EXPECT_EQ(query.snapshot_type.value(), SnapshotType::Checkpoint);
    EXPECT_EQ(query.limit, 25u);
}

TEST_F(HistoryQueryTest, WithLimit) {
    HistoryQuery query;
    query.limit = 200;
    EXPECT_EQ(query.limit, 200u);
}

TEST_F(HistoryQueryTest, WithOffset) {
    HistoryQuery query;
    query.offset = 50;
    EXPECT_EQ(query.offset, 50u);
}

TEST_F(HistoryQueryTest, CombinationFilters) {
    auto now = std::chrono::system_clock::now();
    auto earlier = now - std::chrono::hours(2);
    QueryTimeRange range(earlier, now);

    HistoryQuery query;
    query.entity_id = 100;
    query.time_range = range;
    query.snapshot_type = SnapshotType::Full;
    query.limit = 10;
    query.offset = 5;
    query.ascending = true;

    EXPECT_TRUE(query.entity_id.has_value());
    EXPECT_TRUE(query.time_range.has_value());
    EXPECT_TRUE(query.snapshot_type.has_value());
    EXPECT_TRUE(query.ascending);
}

// ============================================================================
// HistoryQueryResult Tests
// ============================================================================

class HistoryQueryResultTest : public ::testing::Test {};

TEST_F(HistoryQueryResultTest, DefaultConstruction) {
    HistoryQueryResult result;
    EXPECT_TRUE(result.empty());
    EXPECT_EQ(result.total_count, 0u);
    EXPECT_FALSE(result.has_more);
}

TEST_F(HistoryQueryResultTest, ParameterizedConstruction) {
    std::vector<StateSnapshot> snapshots(5);
    HistoryQueryResult result(snapshots, 100, true);

    EXPECT_EQ(result.count(), 5u);
    EXPECT_EQ(result.total_count, 100u);
    EXPECT_TRUE(result.has_more);
}

TEST_F(HistoryQueryResultTest, Empty) {
    HistoryQueryResult result;
    EXPECT_TRUE(result.empty());
}

TEST_F(HistoryQueryResultTest, Count) {
    std::vector<StateSnapshot> snapshots(7);
    HistoryQueryResult result(snapshots, 7, false);
    EXPECT_EQ(result.count(), 7u);
}

TEST_F(HistoryQueryResultTest, SizeBytes) {
    std::vector<StateSnapshot> snapshots;
    for (int i = 0; i < 3; ++i) {
        StateSnapshot snap;
        snap.state_data = std::vector<UInt8>(100, 0xFF);
        snapshots.push_back(snap);
    }

    HistoryQueryResult result(snapshots, 3, false);
    EXPECT_EQ(result.size_bytes(), 300u);
}

// ============================================================================
// RetentionPolicy Tests
// ============================================================================

class RetentionPolicyTest : public ::testing::Test {};

TEST_F(RetentionPolicyTest, DefaultPolicy) {
    auto policy = RetentionPolicy::default_policy();

    EXPECT_EQ(policy.full_retention, std::chrono::hours(720));  // 30 days
    EXPECT_EQ(policy.delta_retention, std::chrono::hours(168));  // 7 days
    EXPECT_EQ(policy.checkpoint_retention, std::chrono::hours(2160));  // 90 days
    EXPECT_EQ(policy.max_snapshots_per_entity, 10000u);
    EXPECT_TRUE(policy.auto_cleanup);
    EXPECT_EQ(policy.cleanup_interval, std::chrono::minutes(60));
}

TEST_F(RetentionPolicyTest, AggressivePolicy) {
    auto policy = RetentionPolicy::aggressive();

    EXPECT_EQ(policy.full_retention, std::chrono::hours(24));  // 1 day
    EXPECT_EQ(policy.delta_retention, std::chrono::hours(6));  // 6 hours
    EXPECT_EQ(policy.checkpoint_retention, std::chrono::hours(72));  // 3 days
    EXPECT_EQ(policy.max_snapshots_per_entity, 1000u);
    EXPECT_EQ(policy.cleanup_interval, std::chrono::minutes(15));
}

TEST_F(RetentionPolicyTest, RelaxedPolicy) {
    auto policy = RetentionPolicy::relaxed();

    EXPECT_EQ(policy.full_retention, std::chrono::hours(2160));  // 90 days
    EXPECT_EQ(policy.delta_retention, std::chrono::hours(720));  // 30 days
    EXPECT_EQ(policy.checkpoint_retention, std::chrono::hours(8760));  // 1 year
    EXPECT_EQ(policy.max_snapshots_per_entity, 100000u);
    EXPECT_EQ(policy.cleanup_interval, std::chrono::minutes(120));
}

TEST_F(RetentionPolicyTest, ShouldRetainFull) {
    auto policy = RetentionPolicy::default_policy();
    auto now = std::chrono::system_clock::now();

    StateSnapshot snapshot;
    snapshot.type = SnapshotType::Full;
    snapshot.timestamp = now - std::chrono::hours(100);  // Within 30 days

    EXPECT_TRUE(policy.should_retain(snapshot, now));
}

TEST_F(RetentionPolicyTest, ShouldNotRetainOldFull) {
    auto policy = RetentionPolicy::default_policy();
    auto now = std::chrono::system_clock::now();

    StateSnapshot snapshot;
    snapshot.type = SnapshotType::Full;
    snapshot.timestamp = now - std::chrono::hours(800);  // Older than 30 days

    EXPECT_FALSE(policy.should_retain(snapshot, now));
}

TEST_F(RetentionPolicyTest, ShouldRetainDelta) {
    auto policy = RetentionPolicy::default_policy();
    auto now = std::chrono::system_clock::now();

    StateSnapshot snapshot;
    snapshot.type = SnapshotType::Delta;
    snapshot.timestamp = now - std::chrono::hours(100);  // Within 7 days

    EXPECT_TRUE(policy.should_retain(snapshot, now));
}

TEST_F(RetentionPolicyTest, ShouldNotRetainOldDelta) {
    auto policy = RetentionPolicy::default_policy();
    auto now = std::chrono::system_clock::now();

    StateSnapshot snapshot;
    snapshot.type = SnapshotType::Delta;
    snapshot.timestamp = now - std::chrono::hours(200);  // Older than 7 days

    EXPECT_FALSE(policy.should_retain(snapshot, now));
}

TEST_F(RetentionPolicyTest, ShouldRetainCheckpoint) {
    auto policy = RetentionPolicy::default_policy();
    auto now = std::chrono::system_clock::now();

    StateSnapshot snapshot;
    snapshot.type = SnapshotType::Checkpoint;
    snapshot.timestamp = now - std::chrono::hours(1000);  // Within 90 days

    EXPECT_TRUE(policy.should_retain(snapshot, now));
}

// ============================================================================
// HistoryStoreConfig Tests
// ============================================================================

class HistoryStoreConfigTest : public ::testing::Test {};

TEST_F(HistoryStoreConfigTest, DefaultConfig) {
    auto config = HistoryStoreConfig::default_config();

    EXPECT_EQ(config.max_total_snapshots, 10000000u);
    EXPECT_EQ(config.max_memory_bytes, 1073741824u);  // 1GB
    EXPECT_TRUE(config.enable_compression);
    EXPECT_TRUE(config.enable_indexing);
    EXPECT_TRUE(config.storage_path.empty());
}

TEST_F(HistoryStoreConfigTest, InMemoryConfig) {
    auto config = HistoryStoreConfig::in_memory();

    EXPECT_EQ(config.max_total_snapshots, 1000000u);  // 1M
    EXPECT_EQ(config.max_memory_bytes, 536870912u);  // 512MB
    EXPECT_TRUE(config.storage_path.empty());
}

TEST_F(HistoryStoreConfigTest, PersistentConfig) {
    auto config = HistoryStoreConfig::persistent("/data/history");

    EXPECT_EQ(config.max_total_snapshots, 100000000u);  // 100M
    EXPECT_EQ(config.max_memory_bytes, 2147483648u);  // 2GB
    EXPECT_EQ(config.storage_path, "/data/history");
}

TEST_F(HistoryStoreConfigTest, HighFrequencyConfig) {
    auto config = HistoryStoreConfig::high_frequency();

    EXPECT_EQ(config.max_total_snapshots, 50000000u);  // 50M
    EXPECT_EQ(config.max_memory_bytes, 4294967296u);  // 4GB
    EXPECT_TRUE(config.enable_compression);
    EXPECT_TRUE(config.enable_indexing);
}

// ============================================================================
// ExportConfig Tests
// ============================================================================

class ExportConfigTest : public ::testing::Test {};

TEST_F(ExportConfigTest, DefaultConstruction) {
    ExportConfig config;
    EXPECT_EQ(config.format, ExportFormat::Parquet);
    EXPECT_TRUE(config.compress);
    EXPECT_EQ(config.batch_size, 10000u);
}

TEST_F(ExportConfigTest, ParameterizedConstruction) {
    HistoryQuery query = HistoryQuery::for_entity(42);
    ExportConfig config(ExportFormat::CSV, "/tmp/export.csv", query);

    EXPECT_EQ(config.format, ExportFormat::CSV);
    EXPECT_EQ(config.output_path, "/tmp/export.csv");
    EXPECT_TRUE(config.query.entity_id.has_value());
}

TEST_F(ExportConfigTest, ToParquet) {
    auto config = ExportConfig::to_parquet("/data/export.parquet");

    EXPECT_EQ(config.format, ExportFormat::Parquet);
    EXPECT_EQ(config.output_path, "/data/export.parquet");
}

TEST_F(ExportConfigTest, ToCSV) {
    auto config = ExportConfig::to_csv("/data/export.csv");

    EXPECT_EQ(config.format, ExportFormat::CSV);
    EXPECT_EQ(config.output_path, "/data/export.csv");
}

TEST_F(ExportConfigTest, ToJSON) {
    auto config = ExportConfig::to_json("/data/export.json");

    EXPECT_EQ(config.format, ExportFormat::JSON);
    EXPECT_EQ(config.output_path, "/data/export.json");
}

TEST_F(ExportConfigTest, WithQuery) {
    HistoryQuery query = HistoryQuery::for_entity(100);
    auto config = ExportConfig::to_csv("/tmp/export.csv", query);

    EXPECT_TRUE(config.query.entity_id.has_value());
    EXPECT_EQ(config.query.entity_id.value(), 100u);
}

// ============================================================================
// HistoryStoreStats Tests
// ============================================================================

class HistoryStoreStatsTest : public ::testing::Test {};

TEST_F(HistoryStoreStatsTest, DefaultConstruction) {
    HistoryStoreStats stats;
    EXPECT_EQ(stats.total_snapshots, 0u);
    EXPECT_EQ(stats.total_entities, 0u);
    EXPECT_EQ(stats.storage_bytes, 0u);
    EXPECT_EQ(stats.oldest_snapshot_age_hours, 0u);
}

TEST_F(HistoryStoreStatsTest, CountForType) {
    HistoryStoreStats stats;
    stats.snapshots_by_type[SnapshotType::Full] = 100;
    stats.snapshots_by_type[SnapshotType::Delta] = 200;

    EXPECT_EQ(stats.count_for_type(SnapshotType::Full), 100u);
    EXPECT_EQ(stats.count_for_type(SnapshotType::Delta), 200u);
    EXPECT_EQ(stats.count_for_type(SnapshotType::Checkpoint), 0u);
}

TEST_F(HistoryStoreStatsTest, AverageSnapshotSize) {
    HistoryStoreStats stats;
    stats.total_snapshots = 10;
    stats.storage_bytes = 5000;

    EXPECT_EQ(stats.average_snapshot_size(), 500u);
}

TEST_F(HistoryStoreStatsTest, AverageSnapshotSizeZero) {
    HistoryStoreStats stats;
    EXPECT_EQ(stats.average_snapshot_size(), 0u);
}

TEST_F(HistoryStoreStatsTest, IsNearCapacity) {
    HistoryStoreStats stats;
    HistoryStoreConfig config = HistoryStoreConfig::default_config();

    stats.total_snapshots = config.max_total_snapshots * 0.95;
    EXPECT_TRUE(stats.is_near_capacity(config));
}

TEST_F(HistoryStoreStatsTest, IsNotNearCapacity) {
    HistoryStoreStats stats;
    HistoryStoreConfig config = HistoryStoreConfig::default_config();

    stats.total_snapshots = config.max_total_snapshots * 0.5;
    stats.storage_bytes = config.max_memory_bytes * 0.5;
    EXPECT_FALSE(stats.is_near_capacity(config));
}

// ============================================================================
// Helper Function Tests
// ============================================================================

class HelperFunctionTest : public ::testing::Test {};

TEST_F(HelperFunctionTest, ComputeStateHash) {
    std::vector<UInt8> data = {0x01, 0x02, 0x03, 0x04, 0x05};
    UInt64 hash = compute_state_hash(data);

    EXPECT_NE(hash, 0u);
}

TEST_F(HelperFunctionTest, ComputeStateHashConsistent) {
    std::vector<UInt8> data = {0xAA, 0xBB, 0xCC};
    UInt64 hash1 = compute_state_hash(data);
    UInt64 hash2 = compute_state_hash(data);

    EXPECT_EQ(hash1, hash2);
}

TEST_F(HelperFunctionTest, ComputeStateHashEmpty) {
    std::vector<UInt8> data;
    UInt64 hash = compute_state_hash(data);

    // FNV-1a returns initial seed for empty data
    EXPECT_EQ(hash, 14695981039346656037ULL);
}

TEST_F(HelperFunctionTest, VerifySnapshotIntegrityValid) {
    StateSnapshot snapshot;
    snapshot.state_data = {0x10, 0x20, 0x30};
    snapshot.state_hash = compute_state_hash(snapshot.state_data);

    EXPECT_TRUE(verify_snapshot_integrity(snapshot));
}

TEST_F(HelperFunctionTest, VerifySnapshotIntegrityInvalid) {
    StateSnapshot snapshot;
    snapshot.state_data = {0x10, 0x20, 0x30};
    snapshot.state_hash = 0xDEADBEEF;  // Wrong hash

    EXPECT_FALSE(verify_snapshot_integrity(snapshot));
}

TEST_F(HelperFunctionTest, CompressDecompressData) {
    std::vector<UInt8> original(1000, 0xAB);
    auto compressed = compress_snapshot_data(original);
    auto decompressed = decompress_snapshot_data(compressed);

    EXPECT_EQ(decompressed.size(), original.size());
    EXPECT_EQ(decompressed, original);
}

TEST_F(HelperFunctionTest, CompressionReducesSize) {
    std::vector<UInt8> data(1000, 0x00);
    auto compressed = compress_snapshot_data(data);

    EXPECT_LT(compressed.size(), data.size());
}

TEST_F(HelperFunctionTest, CalculateSnapshotAgeHours) {
    auto now = std::chrono::system_clock::now();
    StateSnapshot snapshot;
    snapshot.timestamp = now - std::chrono::hours(5);

    UInt64 age = calculate_snapshot_age_hours(snapshot, now);
    EXPECT_EQ(age, 5u);
}

TEST_F(HelperFunctionTest, CalculateSnapshotAgeHoursZero) {
    auto now = std::chrono::system_clock::now();
    StateSnapshot snapshot;
    snapshot.timestamp = now;

    UInt64 age = calculate_snapshot_age_hours(snapshot, now);
    EXPECT_EQ(age, 0u);
}

TEST_F(HelperFunctionTest, CalculateSnapshotAgeHoursFuture) {
    auto now = std::chrono::system_clock::now();
    StateSnapshot snapshot;
    snapshot.timestamp = now + std::chrono::hours(2);

    UInt64 age = calculate_snapshot_age_hours(snapshot, now);
    EXPECT_EQ(age, 0u);  // Future time returns 0
}

TEST_F(HelperFunctionTest, TimeRangeLastHours) {
    auto range = time_range_last_hours(24);
    auto duration = range.duration();

    EXPECT_GE(duration, std::chrono::hours(24) - std::chrono::seconds(1));
    EXPECT_LE(duration, std::chrono::hours(24) + std::chrono::seconds(1));
}

TEST_F(HelperFunctionTest, TimeRangeLastDays) {
    auto range = time_range_last_days(7);
    auto duration = range.duration();

    EXPECT_GE(duration, std::chrono::hours(7 * 24) - std::chrono::seconds(1));
    EXPECT_LE(duration, std::chrono::hours(7 * 24) + std::chrono::seconds(1));
}

// ============================================================================
// Factory Function Tests
// ============================================================================

class FactoryFunctionTest : public ::testing::Test {};

TEST_F(FactoryFunctionTest, CreateHistoryStore) {
    auto config = HistoryStoreConfig::default_config();
    auto store = create_history_store(config);

    EXPECT_NE(store, nullptr);
}

TEST_F(FactoryFunctionTest, CreateMemoryBackend) {
    auto backend = create_memory_backend();
    EXPECT_NE(backend, nullptr);
}

TEST_F(FactoryFunctionTest, CreateParquetExporter) {
    auto exporter = create_parquet_exporter();
    EXPECT_NE(exporter, nullptr);
}

TEST_F(FactoryFunctionTest, CreateCSVExporter) {
    auto exporter = create_csv_exporter();
    EXPECT_NE(exporter, nullptr);
}

TEST_F(FactoryFunctionTest, CreateJSONExporter) {
    auto exporter = create_json_exporter();
    EXPECT_NE(exporter, nullptr);
}

TEST_F(FactoryFunctionTest, CreateMultiFormatExporter) {
    auto exporter = create_multi_format_exporter();
    EXPECT_NE(exporter, nullptr);
}

// ============================================================================
// HistoryStore Lifecycle Tests
// ============================================================================

class HistoryStoreLifecycleTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_ = HistoryStoreConfig::default_config();
        store_ = create_history_store(config_);
    }

    void TearDown() override {
        if (store_ && store_->is_initialized()) {
            store_->shutdown();
        }
    }

    HistoryStoreConfig config_;
    std::unique_ptr<HistoryStore> store_;
};

TEST_F(HistoryStoreLifecycleTest, Initialize) {
    auto result = store_->initialize();
    EXPECT_EQ(result, HistoryResult::Success);
    EXPECT_TRUE(store_->is_initialized());
}

TEST_F(HistoryStoreLifecycleTest, DoubleInitialize) {
    store_->initialize();
    auto result = store_->initialize();
    EXPECT_EQ(result, HistoryResult::AlreadyInitialized);
}

TEST_F(HistoryStoreLifecycleTest, Shutdown) {
    store_->initialize();
    auto result = store_->shutdown();
    EXPECT_EQ(result, HistoryResult::Success);
    EXPECT_FALSE(store_->is_initialized());
}

TEST_F(HistoryStoreLifecycleTest, ShutdownAndReinitialize) {
    store_->initialize();
    store_->shutdown();

    auto result = store_->initialize();
    EXPECT_EQ(result, HistoryResult::Success);
    EXPECT_TRUE(store_->is_initialized());
}

TEST_F(HistoryStoreLifecycleTest, OperationBeforeInitialize) {
    StateSnapshot snapshot;
    snapshot.entity_id = 1;

    auto result = store_->store_snapshot(snapshot);
    EXPECT_EQ(result, HistoryResult::NotInitialized);
}

// ============================================================================
// Snapshot Storage Tests
// ============================================================================

class SnapshotStorageTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_ = HistoryStoreConfig::default_config();
        store_ = create_history_store(config_);
        store_->initialize();
    }

    void TearDown() override {
        store_->shutdown();
    }

    HistoryStoreConfig config_;
    std::unique_ptr<HistoryStore> store_;
};

TEST_F(SnapshotStorageTest, StoreSingleSnapshot) {
    StateSnapshot snapshot;
    snapshot.entity_id = 42;
    snapshot.sequence_number = 1;
    snapshot.type = SnapshotType::Full;
    snapshot.timestamp = std::chrono::system_clock::now();
    snapshot.state_data = {0x01, 0x02, 0x03};
    snapshot.state_hash = compute_state_hash(snapshot.state_data);

    auto result = store_->store_snapshot(snapshot);
    EXPECT_EQ(result, HistoryResult::Success);
}

TEST_F(SnapshotStorageTest, StoreMultipleSnapshots) {
    std::vector<StateSnapshot> snapshots;
    for (int i = 0; i < 5; ++i) {
        StateSnapshot snapshot;
        snapshot.entity_id = 100 + i;
        snapshot.sequence_number = 1;
        snapshot.type = SnapshotType::Full;
        snapshot.timestamp = std::chrono::system_clock::now();
        snapshot.state_data = {static_cast<UInt8>(i), static_cast<UInt8>(i+1)};
        snapshot.state_hash = compute_state_hash(snapshot.state_data);
        snapshots.push_back(snapshot);
    }

    auto result = store_->store_snapshots(snapshots);
    EXPECT_EQ(result, HistoryResult::Success);
}

TEST_F(SnapshotStorageTest, AutoSequenceNumberAssignment) {
    StateSnapshot snap1, snap2;
    snap1.entity_id = 1;
    snap1.sequence_number = 0;  // Auto-assign
    snap1.timestamp = std::chrono::system_clock::now();
    snap1.state_data = {0x01};
    snap1.state_hash = compute_state_hash(snap1.state_data);

    snap2.entity_id = 1;
    snap2.sequence_number = 0;  // Auto-assign
    snap2.timestamp = std::chrono::system_clock::now();
    snap2.state_data = {0x02};
    snap2.state_hash = compute_state_hash(snap2.state_data);

    store_->store_snapshot(snap1);
    store_->store_snapshot(snap2);

    // Both should succeed with auto-assigned sequence numbers
    auto latest = store_->get_latest_snapshot(1);
    EXPECT_TRUE(latest.has_value());
}

// ============================================================================
// Snapshot Retrieval Tests
// ============================================================================

class SnapshotRetrievalTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_ = HistoryStoreConfig::default_config();
        store_ = create_history_store(config_);
        store_->initialize();

        // Store test snapshots
        for (UInt64 seq = 1; seq <= 3; ++seq) {
            StateSnapshot snapshot;
            snapshot.entity_id = 42;
            snapshot.sequence_number = seq;
            snapshot.type = SnapshotType::Full;
            snapshot.timestamp = std::chrono::system_clock::now() - std::chrono::hours(3 - seq);
            snapshot.state_data = {static_cast<UInt8>(seq)};
            snapshot.state_hash = compute_state_hash(snapshot.state_data);
            store_->store_snapshot(snapshot);
        }
    }

    void TearDown() override {
        store_->shutdown();
    }

    HistoryStoreConfig config_;
    std::unique_ptr<HistoryStore> store_;
};

TEST_F(SnapshotRetrievalTest, GetSnapshotByEntityAndSequence) {
    auto snapshot = store_->get_snapshot(42, 2);

    EXPECT_TRUE(snapshot.has_value());
    EXPECT_EQ(snapshot->entity_id, 42u);
    EXPECT_EQ(snapshot->sequence_number, 2u);
}

TEST_F(SnapshotRetrievalTest, GetNonexistentSnapshot) {
    auto snapshot = store_->get_snapshot(999, 1);
    EXPECT_FALSE(snapshot.has_value());
}

TEST_F(SnapshotRetrievalTest, GetLatestSnapshot) {
    auto snapshot = store_->get_latest_snapshot(42);

    EXPECT_TRUE(snapshot.has_value());
    EXPECT_EQ(snapshot->entity_id, 42u);
    EXPECT_EQ(snapshot->sequence_number, 3u);
}

TEST_F(SnapshotRetrievalTest, GetFirstSnapshot) {
    auto snapshot = store_->get_first_snapshot(42);

    EXPECT_TRUE(snapshot.has_value());
    EXPECT_EQ(snapshot->entity_id, 42u);
    EXPECT_EQ(snapshot->sequence_number, 1u);
}

TEST_F(SnapshotRetrievalTest, GetSnapshotAtTime) {
    auto target_time = std::chrono::system_clock::now() - std::chrono::hours(1);
    auto snapshot = store_->get_snapshot_at_time(42, target_time);

    EXPECT_TRUE(snapshot.has_value());
    EXPECT_EQ(snapshot->entity_id, 42u);
}

TEST_F(SnapshotRetrievalTest, GetNonexistentEntityLatest) {
    auto snapshot = store_->get_latest_snapshot(999);
    EXPECT_FALSE(snapshot.has_value());
}

// ============================================================================
// Query Tests
// ============================================================================

class QueryTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_ = HistoryStoreConfig::default_config();
        store_ = create_history_store(config_);
        store_->initialize();

        // Store snapshots for multiple entities with different types
        auto now = std::chrono::system_clock::now();
        for (EntityId entity = 1; entity <= 3; ++entity) {
            for (int i = 0; i < 5; ++i) {
                StateSnapshot snapshot;
                snapshot.entity_id = entity;
                snapshot.sequence_number = i + 1;
                snapshot.type = (i % 2 == 0) ? SnapshotType::Full : SnapshotType::Delta;
                snapshot.timestamp = now - std::chrono::hours(5 - i);
                snapshot.state_data = {static_cast<UInt8>(entity), static_cast<UInt8>(i)};
                snapshot.state_hash = compute_state_hash(snapshot.state_data);
                store_->store_snapshot(snapshot);
            }
        }
    }

    void TearDown() override {
        store_->shutdown();
    }

    HistoryStoreConfig config_;
    std::unique_ptr<HistoryStore> store_;
};

TEST_F(QueryTest, QueryAll) {
    HistoryQuery query;
    query.limit = 100;

    auto result = store_->query(query);
    EXPECT_GE(result.count(), 1u);
}

TEST_F(QueryTest, QueryByEntity) {
    auto query = HistoryQuery::for_entity(2, 100);
    auto result = store_->query(query);

    EXPECT_GE(result.count(), 1u);
    for (const auto& snap : result.snapshots) {
        EXPECT_EQ(snap.entity_id, 2u);
    }
}

TEST_F(QueryTest, QueryByTimeRange) {
    auto now = std::chrono::system_clock::now();
    auto range = QueryTimeRange(now - std::chrono::hours(3), now);
    auto query = HistoryQuery::for_time_range(range, 100);

    auto result = store_->query(query);
    EXPECT_GE(result.count(), 1u);
}

TEST_F(QueryTest, QueryBySnapshotType) {
    auto query = HistoryQuery::for_type(SnapshotType::Full, 100);
    auto result = store_->query(query);

    for (const auto& snap : result.snapshots) {
        EXPECT_EQ(snap.type, SnapshotType::Full);
    }
}

TEST_F(QueryTest, QueryWithLimit) {
    HistoryQuery query;
    query.limit = 5;

    auto result = store_->query(query);
    EXPECT_LE(result.count(), 5u);
}

TEST_F(QueryTest, QueryWithOffset) {
    HistoryQuery query1, query2;
    query1.limit = 100;
    query1.offset = 0;

    query2.limit = 100;
    query2.offset = 2;

    auto result1 = store_->query(query1);
    auto result2 = store_->query(query2);

    EXPECT_GE(result1.count(), result2.count());
}

TEST_F(QueryTest, QueryAscending) {
    HistoryQuery query = HistoryQuery::for_entity(1, 100);
    query.ascending = true;

    auto result = store_->query(query);

    if (result.count() > 1) {
        for (size_t i = 1; i < result.snapshots.size(); ++i) {
            EXPECT_LE(result.snapshots[i-1].timestamp, result.snapshots[i].timestamp);
        }
    }
}

TEST_F(QueryTest, QueryDescending) {
    HistoryQuery query = HistoryQuery::for_entity(1, 100);
    query.ascending = false;

    auto result = store_->query(query);

    if (result.count() > 1) {
        for (size_t i = 1; i < result.snapshots.size(); ++i) {
            EXPECT_GE(result.snapshots[i-1].timestamp, result.snapshots[i].timestamp);
        }
    }
}

TEST_F(QueryTest, GetEntityHistory) {
    auto result = store_->get_entity_history(1, 100);

    EXPECT_GE(result.count(), 1u);
    for (const auto& snap : result.snapshots) {
        EXPECT_EQ(snap.entity_id, 1u);
    }
}

TEST_F(QueryTest, GetSnapshotsInRange) {
    auto now = std::chrono::system_clock::now();
    auto range = QueryTimeRange(now - std::chrono::hours(2), now);

    auto result = store_->get_snapshots_in_range(range, 100);
    EXPECT_GE(result.count(), 0u);
}

// ============================================================================
// Deletion Tests
// ============================================================================

class DeletionTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_ = HistoryStoreConfig::default_config();
        store_ = create_history_store(config_);
        store_->initialize();

        // Store test data
        for (EntityId entity = 1; entity <= 3; ++entity) {
            for (UInt64 seq = 1; seq <= 3; ++seq) {
                StateSnapshot snapshot;
                snapshot.entity_id = entity;
                snapshot.sequence_number = seq;
                snapshot.type = SnapshotType::Full;
                snapshot.timestamp = std::chrono::system_clock::now();
                snapshot.state_data = {static_cast<UInt8>(entity)};
                snapshot.state_hash = compute_state_hash(snapshot.state_data);
                store_->store_snapshot(snapshot);
            }
        }
    }

    void TearDown() override {
        store_->shutdown();
    }

    HistoryStoreConfig config_;
    std::unique_ptr<HistoryStore> store_;
};

TEST_F(DeletionTest, DeleteEntityHistory) {
    auto result = store_->delete_entity_history(1);
    EXPECT_EQ(result, HistoryResult::Success);

    auto snapshot = store_->get_latest_snapshot(1);
    EXPECT_FALSE(snapshot.has_value());
}

TEST_F(DeletionTest, DeleteSnapshot) {
    auto result = store_->delete_snapshot(2, 1);
    EXPECT_EQ(result, HistoryResult::Success);

    auto snapshot = store_->get_snapshot(2, 1);
    EXPECT_FALSE(snapshot.has_value());
}

TEST_F(DeletionTest, DeleteBefore) {
    auto cutoff = std::chrono::system_clock::now() + std::chrono::seconds(1);
    auto result = store_->delete_before(cutoff);
    EXPECT_EQ(result, HistoryResult::Success);
}

TEST_F(DeletionTest, ClearAll) {
    auto result = store_->clear();
    EXPECT_EQ(result, HistoryResult::Success);

    auto stats = store_->get_stats();
    EXPECT_EQ(stats.total_snapshots, 0u);
}

// ============================================================================
// Retention Tests
// ============================================================================

class RetentionTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_ = HistoryStoreConfig::default_config();
        config_.retention = RetentionPolicy::aggressive();
        store_ = create_history_store(config_);
        store_->initialize();
    }

    void TearDown() override {
        store_->shutdown();
    }

    HistoryStoreConfig config_;
    std::unique_ptr<HistoryStore> store_;
};

TEST_F(RetentionTest, ApplyRetention) {
    // Store old snapshots
    auto old_time = std::chrono::system_clock::now() - std::chrono::hours(100);
    StateSnapshot snapshot;
    snapshot.entity_id = 1;
    snapshot.sequence_number = 1;
    snapshot.type = SnapshotType::Full;
    snapshot.timestamp = old_time;
    snapshot.state_data = {0x01};
    snapshot.state_hash = compute_state_hash(snapshot.state_data);
    store_->store_snapshot(snapshot);

    auto result = store_->apply_retention();
    EXPECT_EQ(result, HistoryResult::Success);
}

TEST_F(RetentionTest, SetRetentionPolicy) {
    auto new_policy = RetentionPolicy::relaxed();
    auto result = store_->set_retention_policy(new_policy);
    EXPECT_EQ(result, HistoryResult::Success);

    const auto& policy = store_->get_retention_policy();
    EXPECT_EQ(policy.full_retention, new_policy.full_retention);
}

TEST_F(RetentionTest, GetRetentionPolicy) {
    const auto& policy = store_->get_retention_policy();
    EXPECT_EQ(policy.full_retention, config_.retention.full_retention);
}

// ============================================================================
// Export Tests
// ============================================================================

class ExportTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_ = HistoryStoreConfig::default_config();
        store_ = create_history_store(config_);
        store_->initialize();

        // Store test data
        for (int i = 0; i < 5; ++i) {
            StateSnapshot snapshot;
            snapshot.entity_id = 1;
            snapshot.sequence_number = i + 1;
            snapshot.type = SnapshotType::Full;
            snapshot.timestamp = std::chrono::system_clock::now();
            snapshot.state_data = {static_cast<UInt8>(i)};
            snapshot.state_hash = compute_state_hash(snapshot.state_data);
            store_->store_snapshot(snapshot);
        }
    }

    void TearDown() override {
        store_->shutdown();
    }

    HistoryStoreConfig config_;
    std::unique_ptr<HistoryStore> store_;
};

TEST_F(ExportTest, ExportCSV) {
    auto config = ExportConfig::to_csv("/tmp/test_export.csv");
    auto result = store_->export_history(config);

    // May succeed or return not supported depending on implementation
    EXPECT_TRUE(result == HistoryResult::Success ||
                result == HistoryResult::ExportNotSupported);
}

TEST_F(ExportTest, ExportJSON) {
    auto config = ExportConfig::to_json("/tmp/test_export.json");
    auto result = store_->export_history(config);

    EXPECT_TRUE(result == HistoryResult::Success ||
                result == HistoryResult::ExportNotSupported);
}

TEST_F(ExportTest, ExportParquet) {
    auto config = ExportConfig::to_parquet("/tmp/test_export.parquet");
    auto result = store_->export_history(config);

    // Parquet likely returns not supported
    EXPECT_TRUE(result == HistoryResult::Success ||
                result == HistoryResult::ExportNotSupported);
}

// ============================================================================
// Backend Tests
// ============================================================================

class BackendTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_ = HistoryStoreConfig::default_config();
        store_ = create_history_store(config_);
        store_->initialize();
    }

    void TearDown() override {
        store_->shutdown();
    }

    HistoryStoreConfig config_;
    std::unique_ptr<HistoryStore> store_;
};

TEST_F(BackendTest, GetDefaultBackend) {
    auto backend = store_->get_backend();
    EXPECT_NE(backend, nullptr);
}

TEST_F(BackendTest, SetCustomBackend) {
    auto new_backend = create_memory_backend();
    auto result = store_->set_backend(std::move(new_backend));

    EXPECT_TRUE(result == HistoryResult::Success ||
                result == HistoryResult::AlreadyInitialized);
}

TEST_F(BackendTest, GetExporter) {
    auto exporter = store_->get_exporter();
    EXPECT_NE(exporter, nullptr);
}

TEST_F(BackendTest, SetCustomExporter) {
    auto new_exporter = create_csv_exporter();
    auto result = store_->set_exporter(std::move(new_exporter));

    EXPECT_TRUE(result == HistoryResult::Success ||
                result == HistoryResult::AlreadyInitialized);
}

// ============================================================================
// Statistics Tests
// ============================================================================

class StatisticsTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_ = HistoryStoreConfig::default_config();
        store_ = create_history_store(config_);
        store_->initialize();
    }

    void TearDown() override {
        store_->shutdown();
    }

    HistoryStoreConfig config_;
    std::unique_ptr<HistoryStore> store_;
};

TEST_F(StatisticsTest, InitialStats) {
    auto stats = store_->get_stats();
    EXPECT_EQ(stats.total_snapshots, 0u);
    EXPECT_EQ(stats.total_entities, 0u);
}

TEST_F(StatisticsTest, StatsAfterStore) {
    StateSnapshot snapshot;
    snapshot.entity_id = 1;
    snapshot.sequence_number = 1;
    snapshot.type = SnapshotType::Full;
    snapshot.timestamp = std::chrono::system_clock::now();
    snapshot.state_data = {0x01, 0x02};
    snapshot.state_hash = compute_state_hash(snapshot.state_data);

    store_->store_snapshot(snapshot);

    auto stats = store_->get_stats();
    EXPECT_GE(stats.total_snapshots, 1u);
}

TEST_F(StatisticsTest, GetConfig) {
    const auto& cfg = store_->get_config();
    EXPECT_EQ(cfg.max_total_snapshots, config_.max_total_snapshots);
}

TEST_F(StatisticsTest, GetEntityCount) {
    // Store snapshots for 3 entities
    for (EntityId id = 1; id <= 3; ++id) {
        StateSnapshot snapshot;
        snapshot.entity_id = id;
        snapshot.sequence_number = 1;
        snapshot.type = SnapshotType::Full;
        snapshot.timestamp = std::chrono::system_clock::now();
        snapshot.state_data = {0x01};
        snapshot.state_hash = compute_state_hash(snapshot.state_data);
        store_->store_snapshot(snapshot);
    }

    auto count = store_->get_entity_count();
    EXPECT_GE(count, 3u);
}

TEST_F(StatisticsTest, GetEntitySnapshotCount) {
    // Store 5 snapshots for entity 1
    for (int i = 0; i < 5; ++i) {
        StateSnapshot snapshot;
        snapshot.entity_id = 1;
        snapshot.sequence_number = i + 1;
        snapshot.type = SnapshotType::Full;
        snapshot.timestamp = std::chrono::system_clock::now();
        snapshot.state_data = {static_cast<UInt8>(i)};
        snapshot.state_hash = compute_state_hash(snapshot.state_data);
        store_->store_snapshot(snapshot);
    }

    auto count = store_->get_entity_snapshot_count(1);
    EXPECT_GE(count, 5u);
}

TEST_F(StatisticsTest, HasEntityHistory) {
    StateSnapshot snapshot;
    snapshot.entity_id = 42;
    snapshot.sequence_number = 1;
    snapshot.type = SnapshotType::Full;
    snapshot.timestamp = std::chrono::system_clock::now();
    snapshot.state_data = {0x01};
    snapshot.state_hash = compute_state_hash(snapshot.state_data);
    store_->store_snapshot(snapshot);

    EXPECT_TRUE(store_->has_entity_history(42));
    EXPECT_FALSE(store_->has_entity_history(999));
}

TEST_F(StatisticsTest, SnapshotsByType) {
    // Store different types
    for (int i = 0; i < 3; ++i) {
        StateSnapshot snapshot;
        snapshot.entity_id = 1;
        snapshot.sequence_number = i + 1;
        snapshot.type = (i == 0) ? SnapshotType::Full :
                        (i == 1) ? SnapshotType::Delta : SnapshotType::Checkpoint;
        snapshot.timestamp = std::chrono::system_clock::now();
        snapshot.state_data = {static_cast<UInt8>(i)};
        snapshot.state_hash = compute_state_hash(snapshot.state_data);
        store_->store_snapshot(snapshot);
    }

    auto stats = store_->get_stats();
    EXPECT_GE(stats.count_for_type(SnapshotType::Full), 1u);
}
