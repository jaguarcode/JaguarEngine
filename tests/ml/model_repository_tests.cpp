/**
 * @file model_repository_tests.cpp
 * @brief Comprehensive unit tests for ML model repository
 */

#include <gtest/gtest.h>
#include "jaguar/ml/model_repository.h"
#include <fstream>
#include <chrono>
#include <thread>
#include <algorithm>

using namespace jaguar;
using namespace jaguar::ml;

// ============================================================================
// SemanticVersion Tests
// ============================================================================

class SemanticVersionTest : public ::testing::Test {
protected:
    SemanticVersion v1_0_0{1, 0, 0};
    SemanticVersion v1_2_3{1, 2, 3};
    SemanticVersion v2_0_0{2, 0, 0};
    SemanticVersion v0_1_0{0, 1, 0};
};

TEST_F(SemanticVersionTest, DefaultConstruction) {
    SemanticVersion version;
    EXPECT_EQ(version.major, 0u);
    EXPECT_EQ(version.minor, 0u);
    EXPECT_EQ(version.patch, 0u);
}

TEST_F(SemanticVersionTest, ParameterizedConstruction) {
    SemanticVersion version(2, 5, 7);
    EXPECT_EQ(version.major, 2u);
    EXPECT_EQ(version.minor, 5u);
    EXPECT_EQ(version.patch, 7u);
}

TEST_F(SemanticVersionTest, ToString) {
    EXPECT_EQ(v1_0_0.to_string(), "1.0.0");
    EXPECT_EQ(v1_2_3.to_string(), "1.2.3");
    EXPECT_EQ(v2_0_0.to_string(), "2.0.0");
    EXPECT_EQ(v0_1_0.to_string(), "0.1.0");
}

TEST_F(SemanticVersionTest, ParseValid) {
    auto parsed = SemanticVersion::parse("3.14.159");
    EXPECT_EQ(parsed.major, 3u);
    EXPECT_EQ(parsed.minor, 14u);
    EXPECT_EQ(parsed.patch, 159u);
}

TEST_F(SemanticVersionTest, ParseMajorOnly) {
    auto parsed = SemanticVersion::parse("5");
    EXPECT_EQ(parsed.major, 5u);
    EXPECT_EQ(parsed.minor, 0u);
    EXPECT_EQ(parsed.patch, 0u);
}

TEST_F(SemanticVersionTest, ParseMajorMinor) {
    auto parsed = SemanticVersion::parse("2.4");
    EXPECT_EQ(parsed.major, 2u);
    EXPECT_EQ(parsed.minor, 4u);
    EXPECT_EQ(parsed.patch, 0u);
}

TEST_F(SemanticVersionTest, ParseInvalid) {
    auto parsed = SemanticVersion::parse("invalid");
    EXPECT_EQ(parsed.major, 0u);
    EXPECT_EQ(parsed.minor, 0u);
    EXPECT_EQ(parsed.patch, 0u);
}

TEST_F(SemanticVersionTest, EqualityOperator) {
    SemanticVersion v1(1, 2, 3);
    SemanticVersion v2(1, 2, 3);
    SemanticVersion v3(1, 2, 4);

    EXPECT_TRUE(v1 == v2);
    EXPECT_FALSE(v1 == v3);
}

TEST_F(SemanticVersionTest, InequalityOperator) {
    SemanticVersion v1(1, 2, 3);
    SemanticVersion v2(1, 2, 4);

    EXPECT_TRUE(v1 != v2);
    EXPECT_FALSE(v1 != v1);
}

TEST_F(SemanticVersionTest, LessThanOperator) {
    EXPECT_TRUE(v1_0_0 < v1_2_3);
    EXPECT_TRUE(v1_2_3 < v2_0_0);
    EXPECT_FALSE(v2_0_0 < v1_2_3);
    EXPECT_FALSE(v1_0_0 < v1_0_0);
}

TEST_F(SemanticVersionTest, GreaterThanOperator) {
    EXPECT_TRUE(v2_0_0 > v1_2_3);
    EXPECT_TRUE(v1_2_3 > v1_0_0);
    EXPECT_FALSE(v1_0_0 > v2_0_0);
    EXPECT_FALSE(v1_0_0 > v1_0_0);
}

TEST_F(SemanticVersionTest, LessThanOrEqualOperator) {
    EXPECT_TRUE(v1_0_0 <= v1_2_3);
    EXPECT_TRUE(v1_0_0 <= v1_0_0);
    EXPECT_FALSE(v2_0_0 <= v1_2_3);
}

TEST_F(SemanticVersionTest, GreaterThanOrEqualOperator) {
    EXPECT_TRUE(v2_0_0 >= v1_2_3);
    EXPECT_TRUE(v1_0_0 >= v1_0_0);
    EXPECT_FALSE(v1_0_0 >= v2_0_0);
}

TEST_F(SemanticVersionTest, CompatibilityCheck) {
    EXPECT_TRUE(v1_0_0.is_compatible_with(v1_2_3));
    EXPECT_TRUE(v1_2_3.is_compatible_with(v1_0_0));
    EXPECT_FALSE(v1_2_3.is_compatible_with(v2_0_0));
    EXPECT_FALSE(v2_0_0.is_compatible_with(v1_2_3));
    EXPECT_FALSE(v0_1_0.is_compatible_with(SemanticVersion(0, 2, 0)));
}

TEST_F(SemanticVersionTest, ComparisonByMajorVersion) {
    SemanticVersion v1(1, 9, 9);
    SemanticVersion v2(2, 0, 0);
    EXPECT_TRUE(v1 < v2);
}

TEST_F(SemanticVersionTest, ComparisonByMinorVersion) {
    SemanticVersion v1(1, 5, 9);
    SemanticVersion v2(1, 6, 0);
    EXPECT_TRUE(v1 < v2);
}

TEST_F(SemanticVersionTest, ComparisonByPatchVersion) {
    SemanticVersion v1(1, 2, 3);
    SemanticVersion v2(1, 2, 4);
    EXPECT_TRUE(v1 < v2);
}

// ============================================================================
// TensorInfo Tests
// ============================================================================

class ModelRepoTensorInfoTest : public ::testing::Test {
};

TEST_F(ModelRepoTensorInfoTest, DefaultConstruction) {
    TensorInfo tensor;
    EXPECT_TRUE(tensor.name.empty());
    EXPECT_TRUE(tensor.shape.empty());
    EXPECT_TRUE(tensor.dtype.empty());
}

TEST_F(ModelRepoTensorInfoTest, ParameterizedConstruction) {
    TensorInfo tensor("input", {1, 3, 224, 224}, "float32", "Input image");
    EXPECT_EQ(tensor.name, "input");
    EXPECT_EQ(tensor.shape.size(), 4u);
    EXPECT_EQ(tensor.shape[0], 1);
    EXPECT_EQ(tensor.shape[1], 3);
    EXPECT_EQ(tensor.dtype, "float32");
    EXPECT_EQ(tensor.description, "Input image");
}

TEST_F(ModelRepoTensorInfoTest, StaticShape) {
    TensorInfo tensor("data", {10, 20}, "int32");
    EXPECT_FALSE(tensor.is_dynamic());
    EXPECT_EQ(tensor.element_count(), 200);
}

TEST_F(ModelRepoTensorInfoTest, DynamicShape) {
    TensorInfo tensor("batch", {-1, 128}, "float32");
    EXPECT_TRUE(tensor.is_dynamic());
    EXPECT_EQ(tensor.element_count(), 0);
}

TEST_F(ModelRepoTensorInfoTest, ElementCountMultipleDimensions) {
    TensorInfo tensor("tensor", {2, 3, 4, 5}, "float64");
    EXPECT_EQ(tensor.element_count(), 120);
}

TEST_F(ModelRepoTensorInfoTest, EmptyShape) {
    TensorInfo tensor("scalar", {}, "int8");
    EXPECT_FALSE(tensor.is_dynamic());
    EXPECT_EQ(tensor.element_count(), 1);
}

// ============================================================================
// ModelMetadata Tests
// ============================================================================

class ModelMetadataTest : public ::testing::Test {
protected:
    ModelMetadata metadata;

    void SetUp() override {
        metadata = ModelMetadata("autopilot-v1", "Autopilot AI",
                                 ModelType::Autopilot, SemanticVersion(1, 0, 0));
    }
};

TEST_F(ModelMetadataTest, DefaultConstruction) {
    ModelMetadata empty;
    EXPECT_TRUE(empty.model_id.empty());
    EXPECT_TRUE(empty.name.empty());
    EXPECT_EQ(empty.type, ModelType::Custom);
    EXPECT_EQ(empty.version.major, 0u);
}

TEST_F(ModelMetadataTest, ParameterizedConstruction) {
    EXPECT_EQ(metadata.model_id, "autopilot-v1");
    EXPECT_EQ(metadata.name, "Autopilot AI");
    EXPECT_EQ(metadata.type, ModelType::Autopilot);
    EXPECT_EQ(metadata.version, SemanticVersion(1, 0, 0));
}

TEST_F(ModelMetadataTest, AddTag) {
    metadata.add_tag("architecture", "mlp");
    metadata.add_tag("training_date", "2026-01-15");

    EXPECT_EQ(metadata.tags.size(), 2u);
    EXPECT_TRUE(metadata.has_tag("architecture"));
    EXPECT_TRUE(metadata.has_tag("training_date"));
}

TEST_F(ModelMetadataTest, GetTag) {
    metadata.add_tag("framework", "pytorch");

    auto value = metadata.get_tag("framework");
    ASSERT_TRUE(value.has_value());
    EXPECT_EQ(value.value(), "pytorch");
}

TEST_F(ModelMetadataTest, GetNonExistentTag) {
    auto value = metadata.get_tag("nonexistent");
    EXPECT_FALSE(value.has_value());
}

TEST_F(ModelMetadataTest, HasTag) {
    metadata.add_tag("device", "cuda");
    EXPECT_TRUE(metadata.has_tag("device"));
    EXPECT_FALSE(metadata.has_tag("nonexistent"));
}

TEST_F(ModelMetadataTest, FullId) {
    EXPECT_EQ(metadata.full_id(), "autopilot-v1@1.0.0");
}

TEST_F(ModelMetadataTest, OverwriteTag) {
    metadata.add_tag("status", "draft");
    metadata.add_tag("status", "production");

    auto value = metadata.get_tag("status");
    ASSERT_TRUE(value.has_value());
    EXPECT_EQ(value.value(), "production");
}

TEST_F(ModelMetadataTest, TimestampInitialization) {
    auto now = std::chrono::system_clock::now();

    // Timestamps should be close to now
    auto created_diff = std::chrono::duration_cast<std::chrono::seconds>(
        now - metadata.created_at).count();
    auto updated_diff = std::chrono::duration_cast<std::chrono::seconds>(
        now - metadata.updated_at).count();

    EXPECT_LE(std::abs(created_diff), 1);
    EXPECT_LE(std::abs(updated_diff), 1);
}

// ============================================================================
// ModelEntry Tests
// ============================================================================

class ModelEntryTest : public ::testing::Test {
protected:
    ModelMetadata metadata;
    ModelEntry entry;

    void SetUp() override {
        metadata = ModelMetadata("test-model", "Test Model",
                                 ModelType::Custom, SemanticVersion(1, 0, 0));
        entry = ModelEntry(metadata, "/path/to/model.bin", ModelStatus::Ready);
    }
};

TEST_F(ModelEntryTest, Construction) {
    EXPECT_EQ(entry.metadata.model_id, "test-model");
    EXPECT_EQ(entry.file_path, "/path/to/model.bin");
    EXPECT_EQ(entry.status, ModelStatus::Ready);
}

TEST_F(ModelEntryTest, Touch) {
    auto before = entry.last_accessed;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    entry.touch();
    auto after = entry.last_accessed;

    EXPECT_GT(after, before);
}

TEST_F(ModelEntryTest, Age) {
    auto age = entry.age();
    EXPECT_GE(age.count(), 0);
}

TEST_F(ModelEntryTest, IsUsable) {
    entry.status = ModelStatus::Ready;
    EXPECT_TRUE(entry.is_usable());

    entry.status = ModelStatus::Loading;
    EXPECT_FALSE(entry.is_usable());

    entry.status = ModelStatus::Error;
    EXPECT_FALSE(entry.is_usable());

    entry.status = ModelStatus::Deprecated;
    EXPECT_FALSE(entry.is_usable());

    entry.status = ModelStatus::Unknown;
    EXPECT_FALSE(entry.is_usable());
}

// ============================================================================
// ModelQuery Tests
// ============================================================================

class ModelQueryTest : public ::testing::Test {
};

TEST_F(ModelQueryTest, DefaultConstruction) {
    ModelQuery query;
    EXPECT_FALSE(query.type.has_value());
    EXPECT_FALSE(query.name_pattern.has_value());
    EXPECT_FALSE(query.min_version.has_value());
    EXPECT_FALSE(query.max_version.has_value());
    EXPECT_FALSE(query.tag.has_value());
    EXPECT_EQ(query.limit, 100u);
    EXPECT_FALSE(query.include_deprecated);
}

TEST_F(ModelQueryTest, ForType) {
    auto query = ModelQuery::for_type(ModelType::Behavior, 50);
    ASSERT_TRUE(query.type.has_value());
    EXPECT_EQ(query.type.value(), ModelType::Behavior);
    EXPECT_EQ(query.limit, 50u);
}

TEST_F(ModelQueryTest, ForName) {
    auto query = ModelQuery::for_name("autopilot*", 25);
    ASSERT_TRUE(query.name_pattern.has_value());
    EXPECT_EQ(query.name_pattern.value(), "autopilot*");
    EXPECT_EQ(query.limit, 25u);
}

TEST_F(ModelQueryTest, ForVersionRange) {
    auto query = ModelQuery::for_version_range(
        SemanticVersion(1, 0, 0),
        SemanticVersion(2, 0, 0),
        10);
    ASSERT_TRUE(query.min_version.has_value());
    ASSERT_TRUE(query.max_version.has_value());
    EXPECT_EQ(query.min_version.value(), SemanticVersion(1, 0, 0));
    EXPECT_EQ(query.max_version.value(), SemanticVersion(2, 0, 0));
    EXPECT_EQ(query.limit, 10u);
}

TEST_F(ModelQueryTest, ForTag) {
    auto query = ModelQuery::for_tag("production", 200);
    ASSERT_TRUE(query.tag.has_value());
    EXPECT_EQ(query.tag.value(), "production");
    EXPECT_EQ(query.limit, 200u);
}

// ============================================================================
// CacheConfig Tests
// ============================================================================

class CacheConfigTest : public ::testing::Test {
};

TEST_F(CacheConfigTest, DefaultConfig) {
    auto config = CacheConfig::default_config();
    EXPECT_EQ(config.max_memory_bytes, 1073741824u);  // 1GB
    EXPECT_EQ(config.max_models, 100u);
    EXPECT_EQ(config.eviction_timeout.count(), 30);
    EXPECT_FALSE(config.enable_preloading);
}

TEST_F(CacheConfigTest, AggressiveConfig) {
    auto config = CacheConfig::aggressive();
    EXPECT_EQ(config.max_memory_bytes, 268435456u);  // 256MB
    EXPECT_EQ(config.max_models, 10u);
    EXPECT_EQ(config.eviction_timeout.count(), 10);
    EXPECT_FALSE(config.enable_preloading);
}

TEST_F(CacheConfigTest, RelaxedConfig) {
    auto config = CacheConfig::relaxed();
    EXPECT_EQ(config.max_memory_bytes, 4294967296u);  // 4GB
    EXPECT_EQ(config.max_models, 500u);
    EXPECT_EQ(config.eviction_timeout.count(), 120);
    EXPECT_TRUE(config.enable_preloading);
}

TEST_F(CacheConfigTest, CustomConfig) {
    CacheConfig config;
    config.max_memory_bytes = 2147483648;  // 2GB
    config.max_models = 50;
    config.eviction_timeout = std::chrono::minutes(60);
    config.enable_preloading = true;

    EXPECT_EQ(config.max_memory_bytes, 2147483648u);
    EXPECT_EQ(config.max_models, 50u);
    EXPECT_EQ(config.eviction_timeout.count(), 60);
    EXPECT_TRUE(config.enable_preloading);
}

// ============================================================================
// RepositoryConfig Tests
// ============================================================================

class RepositoryConfigTest : public ::testing::Test {
};

TEST_F(RepositoryConfigTest, DefaultConstruction) {
    RepositoryConfig config;
    EXPECT_TRUE(config.base_path.empty());
    EXPECT_TRUE(config.verify_checksums);
    EXPECT_FALSE(config.auto_update);
}

TEST_F(RepositoryConfigTest, ParameterizedConstruction) {
    RepositoryConfig config("/models", CacheConfig::aggressive());
    EXPECT_EQ(config.base_path, "/models");
    EXPECT_EQ(config.cache.max_models, 10u);
}

TEST_F(RepositoryConfigTest, DefaultConfig) {
    auto config = RepositoryConfig::default_config();
    EXPECT_EQ(config.base_path, "./models");
    EXPECT_EQ(config.cache.max_memory_bytes, 1073741824u);
    EXPECT_TRUE(config.verify_checksums);
}

TEST_F(RepositoryConfigTest, InMemoryConfig) {
    auto config = RepositoryConfig::in_memory();
    EXPECT_TRUE(config.base_path.empty());
    EXPECT_EQ(config.cache.max_models, 10u);
    EXPECT_FALSE(config.verify_checksums);
}

TEST_F(RepositoryConfigTest, PersistentConfig) {
    auto config = RepositoryConfig::persistent("/persistent/models");
    EXPECT_EQ(config.base_path, "/persistent/models");
    EXPECT_EQ(config.cache.max_models, 500u);
    EXPECT_TRUE(config.verify_checksums);
}

// ============================================================================
// RepositoryStats Tests
// ============================================================================

class RepositoryStatsTest : public ::testing::Test {
protected:
    RepositoryStats stats;
};

TEST_F(RepositoryStatsTest, DefaultConstruction) {
    EXPECT_EQ(stats.total_models, 0u);
    EXPECT_EQ(stats.cached_models, 0u);
    EXPECT_EQ(stats.cache_memory_used, 0u);
    EXPECT_EQ(stats.cache_hits, 0u);
    EXPECT_EQ(stats.cache_misses, 0u);
    EXPECT_TRUE(stats.models_by_type.empty());
}

TEST_F(RepositoryStatsTest, CountForType) {
    stats.models_by_type[ModelType::Autopilot] = 5;
    stats.models_by_type[ModelType::Behavior] = 3;

    EXPECT_EQ(stats.count_for_type(ModelType::Autopilot), 5u);
    EXPECT_EQ(stats.count_for_type(ModelType::Behavior), 3u);
    EXPECT_EQ(stats.count_for_type(ModelType::Perception), 0u);
}

TEST_F(RepositoryStatsTest, CacheHitRateZero) {
    EXPECT_DOUBLE_EQ(stats.cache_hit_rate(), 0.0);
}

TEST_F(RepositoryStatsTest, CacheHitRateCalculation) {
    stats.cache_hits = 80;
    stats.cache_misses = 20;
    EXPECT_DOUBLE_EQ(stats.cache_hit_rate(), 0.8);
}

TEST_F(RepositoryStatsTest, CacheHitRatePerfect) {
    stats.cache_hits = 100;
    stats.cache_misses = 0;
    EXPECT_DOUBLE_EQ(stats.cache_hit_rate(), 1.0);
}

TEST_F(RepositoryStatsTest, CacheUtilization) {
    CacheConfig config;
    config.max_memory_bytes = 1000;

    stats.cache_memory_used = 500;
    EXPECT_DOUBLE_EQ(stats.cache_utilization(config), 0.5);
}

TEST_F(RepositoryStatsTest, CacheUtilizationZero) {
    CacheConfig config;
    config.max_memory_bytes = 1000;

    EXPECT_DOUBLE_EQ(stats.cache_utilization(config), 0.0);
}

TEST_F(RepositoryStatsTest, IsNearCapacityByCount) {
    CacheConfig config;
    config.max_models = 100;
    config.max_memory_bytes = 10000;

    stats.cached_models = 91;
    stats.cache_memory_used = 5000;

    EXPECT_TRUE(stats.is_cache_near_capacity(config));
}

TEST_F(RepositoryStatsTest, IsNearCapacityByMemory) {
    CacheConfig config;
    config.max_models = 100;
    config.max_memory_bytes = 10000;

    stats.cached_models = 50;
    stats.cache_memory_used = 9100;

    EXPECT_TRUE(stats.is_cache_near_capacity(config));
}

TEST_F(RepositoryStatsTest, NotNearCapacity) {
    CacheConfig config;
    config.max_models = 100;
    config.max_memory_bytes = 10000;

    stats.cached_models = 50;
    stats.cache_memory_used = 5000;

    EXPECT_FALSE(stats.is_cache_near_capacity(config));
}

// ============================================================================
// Helper Function Tests
// ============================================================================

class HelperFunctionsTest : public ::testing::Test {
};

TEST_F(HelperFunctionsTest, FindLatestVersionEmpty) {
    std::vector<ModelEntry> entries;
    auto result = find_latest_version(entries);
    EXPECT_FALSE(result.has_value());
}

TEST_F(HelperFunctionsTest, FindLatestVersionSingle) {
    ModelMetadata metadata("model", "Model", ModelType::Custom, SemanticVersion(1, 0, 0));
    std::vector<ModelEntry> entries;
    entries.emplace_back(metadata, "/path", ModelStatus::Ready);

    auto result = find_latest_version(entries);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->metadata.version, SemanticVersion(1, 0, 0));
}

TEST_F(HelperFunctionsTest, FindLatestVersionMultiple) {
    std::vector<ModelEntry> entries;

    ModelMetadata m1("model", "Model", ModelType::Custom, SemanticVersion(1, 0, 0));
    ModelMetadata m2("model", "Model", ModelType::Custom, SemanticVersion(2, 5, 0));
    ModelMetadata m3("model", "Model", ModelType::Custom, SemanticVersion(1, 3, 0));

    entries.emplace_back(m1, "/path", ModelStatus::Ready);
    entries.emplace_back(m2, "/path", ModelStatus::Ready);
    entries.emplace_back(m3, "/path", ModelStatus::Ready);

    auto result = find_latest_version(entries);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->metadata.version, SemanticVersion(2, 5, 0));
}

TEST_F(HelperFunctionsTest, FindCompatibleModelsEmpty) {
    std::vector<ModelEntry> entries;
    auto result = find_compatible_models(entries, SemanticVersion(1, 0, 0));
    EXPECT_TRUE(result.empty());
}

TEST_F(HelperFunctionsTest, FindCompatibleModelsSameMajor) {
    std::vector<ModelEntry> entries;

    ModelMetadata m1("model", "Model", ModelType::Custom, SemanticVersion(1, 0, 0));
    ModelMetadata m2("model", "Model", ModelType::Custom, SemanticVersion(1, 2, 0));
    ModelMetadata m3("model", "Model", ModelType::Custom, SemanticVersion(2, 0, 0));

    entries.emplace_back(m1, "/path", ModelStatus::Ready);
    entries.emplace_back(m2, "/path", ModelStatus::Ready);
    entries.emplace_back(m3, "/path", ModelStatus::Ready);

    auto result = find_compatible_models(entries, SemanticVersion(1, 5, 0));
    EXPECT_EQ(result.size(), 2u);
}

TEST_F(HelperFunctionsTest, CalculateModelAgeHours) {
    ModelMetadata metadata("model", "Model", ModelType::Custom, SemanticVersion(1, 0, 0));

    auto past_time = std::chrono::system_clock::now() - std::chrono::hours(48);
    metadata.created_at = past_time;

    auto age = calculate_model_age_hours(metadata);
    EXPECT_GE(age, 48u);
    EXPECT_LE(age, 49u);
}

TEST_F(HelperFunctionsTest, ShouldUpdateModel) {
    SemanticVersion current(1, 0, 0);
    SemanticVersion newer_compatible(1, 2, 0);
    SemanticVersion newer_breaking(2, 0, 0);
    SemanticVersion older(0, 9, 0);

    EXPECT_TRUE(should_update_model(current, newer_compatible));
    EXPECT_FALSE(should_update_model(current, newer_breaking));
    EXPECT_FALSE(should_update_model(current, older));
}

TEST_F(HelperFunctionsTest, FormatModelSize) {
    EXPECT_EQ(format_model_size(500), "500.00 B");
    EXPECT_EQ(format_model_size(1536), "1.50 KB");
    EXPECT_EQ(format_model_size(1572864), "1.50 MB");
    EXPECT_EQ(format_model_size(1610612736), "1.50 GB");
}

// ============================================================================
// Enum Conversion Tests
// ============================================================================

TEST(EnumConversionTest, ModelResultToString) {
    EXPECT_STREQ(model_result_to_string(ModelResult::Success), "Success");
    EXPECT_STREQ(model_result_to_string(ModelResult::InvalidConfiguration), "InvalidConfiguration");
    EXPECT_STREQ(model_result_to_string(ModelResult::ModelNotFound), "ModelNotFound");
    EXPECT_STREQ(model_result_to_string(ModelResult::OutOfMemory), "OutOfMemory");
}

TEST(EnumConversionTest, ModelTypeToString) {
    EXPECT_STREQ(model_type_to_string(ModelType::Autopilot), "Autopilot");
    EXPECT_STREQ(model_type_to_string(ModelType::Behavior), "Behavior");
    EXPECT_STREQ(model_type_to_string(ModelType::Perception), "Perception");
    EXPECT_STREQ(model_type_to_string(ModelType::Custom), "Custom");
}

TEST(EnumConversionTest, ModelStatusToString) {
    EXPECT_STREQ(model_status_to_string(ModelStatus::Unknown), "Unknown");
    EXPECT_STREQ(model_status_to_string(ModelStatus::Loading), "Loading");
    EXPECT_STREQ(model_status_to_string(ModelStatus::Ready), "Ready");
    EXPECT_STREQ(model_status_to_string(ModelStatus::Error), "Error");
    EXPECT_STREQ(model_status_to_string(ModelStatus::Deprecated), "Deprecated");
}

// ============================================================================
// ModelRepository Lifecycle Tests
// ============================================================================

class ModelRepositoryTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto config = RepositoryConfig::in_memory();
        auto temp = create_model_repository(config);
        repo = temp.release();
        ASSERT_NE(repo, nullptr);
    }

    void TearDown() override {
        if (repo && repo->is_initialized()) {
            repo->shutdown();
        }
        delete repo;
        repo = nullptr;
    }

    ModelRepository* repo = nullptr;
};

TEST_F(ModelRepositoryTest, Creation) {
    EXPECT_NE(repo, nullptr);
}

TEST_F(ModelRepositoryTest, CreateWithDefaultConfig) {
    auto config = RepositoryConfig::default_config();
    auto r = create_model_repository(config);
    EXPECT_NE(r, nullptr);
}

TEST_F(ModelRepositoryTest, CreateMemoryRepository) {
    auto r = create_memory_repository();
    EXPECT_NE(r, nullptr);
}

TEST_F(ModelRepositoryTest, Initialization) {
    EXPECT_FALSE(repo->is_initialized());
    EXPECT_EQ(repo->initialize(), ModelResult::Success);
    EXPECT_TRUE(repo->is_initialized());
}

TEST_F(ModelRepositoryTest, DoubleInitialization) {
    EXPECT_EQ(repo->initialize(), ModelResult::Success);
    EXPECT_EQ(repo->initialize(), ModelResult::AlreadyInitialized);
}

TEST_F(ModelRepositoryTest, Shutdown) {
    EXPECT_EQ(repo->initialize(), ModelResult::Success);
    EXPECT_EQ(repo->shutdown(), ModelResult::Success);
    EXPECT_FALSE(repo->is_initialized());
}

TEST_F(ModelRepositoryTest, ShutdownWithoutInit) {
    EXPECT_EQ(repo->shutdown(), ModelResult::NotInitialized);
}

TEST_F(ModelRepositoryTest, GetConfig) {
    const auto& config = repo->get_config();
    EXPECT_TRUE(config.base_path.empty());  // in-memory
}

// ============================================================================
// Model Registration Tests
// ============================================================================

class ModelRegistrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto temp = create_memory_repository();
        repo = temp.release();
        ASSERT_NE(repo, nullptr);
        ASSERT_EQ(repo->initialize(), ModelResult::Success);
    }

    void TearDown() override {
        if (repo && repo->is_initialized()) {
            repo->shutdown();
        }
        delete repo;
        repo = nullptr;
    }

    ModelRepository* repo = nullptr;
};

TEST_F(ModelRegistrationTest, RegisterModel) {
    ModelMetadata metadata("model1", "Test Model", ModelType::Custom, SemanticVersion(1, 0, 0));

    auto result = repo->register_model("/test/model.bin", metadata);
    EXPECT_EQ(result, ModelResult::Success);
    EXPECT_EQ(repo->get_model_count(), 1u);
}

TEST_F(ModelRegistrationTest, RegisterMultipleModels) {
    for (int i = 0; i < 5; ++i) {
        std::string id = "model" + std::to_string(i);
        ModelMetadata metadata(id, "Model", ModelType::Custom, SemanticVersion(1, 0, 0));
        EXPECT_EQ(repo->register_model("/test/" + id + ".bin", metadata), ModelResult::Success);
    }

    EXPECT_EQ(repo->get_model_count(), 5u);
}

TEST_F(ModelRegistrationTest, RegisterDuplicateModel) {
    ModelMetadata metadata("duplicate", "Duplicate", ModelType::Custom, SemanticVersion(1, 0, 0));

    EXPECT_EQ(repo->register_model("/test/model.bin", metadata), ModelResult::Success);
    EXPECT_EQ(repo->register_model("/test/model2.bin", metadata), ModelResult::ModelAlreadyExists);
}

TEST_F(ModelRegistrationTest, RegisterWithoutInitialization) {
    auto uninit_repo = create_memory_repository();
    ModelMetadata metadata("model", "Model", ModelType::Custom, SemanticVersion(1, 0, 0));

    EXPECT_EQ(uninit_repo->register_model("/test/model.bin", metadata), ModelResult::NotInitialized);
}

TEST_F(ModelRegistrationTest, UnregisterModel) {
    ModelMetadata metadata("model1", "Model", ModelType::Custom, SemanticVersion(1, 0, 0));
    ASSERT_EQ(repo->register_model("/test/model.bin", metadata), ModelResult::Success);

    EXPECT_EQ(repo->unregister_model("model1"), ModelResult::Success);
    EXPECT_EQ(repo->get_model_count(), 0u);
}

TEST_F(ModelRegistrationTest, UnregisterNonExistentModel) {
    EXPECT_EQ(repo->unregister_model("nonexistent"), ModelResult::ModelNotFound);
}

TEST_F(ModelRegistrationTest, UnregisterWithoutInitialization) {
    auto uninit_repo = create_memory_repository();
    EXPECT_EQ(uninit_repo->unregister_model("model"), ModelResult::NotInitialized);
}

// ============================================================================
// Model Retrieval Tests
// ============================================================================

class ModelRetrievalTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto temp = create_memory_repository();
        repo = temp.release();
        ASSERT_NE(repo, nullptr);
        ASSERT_EQ(repo->initialize(), ModelResult::Success);

        // Register some test models
        register_test_model("autopilot-v1", ModelType::Autopilot, 1, 0, 0);
        register_test_model("autopilot-v2", ModelType::Autopilot, 2, 0, 0);
        register_test_model("behavior-v1", ModelType::Behavior, 1, 0, 0);
        register_test_model("perception-v1", ModelType::Perception, 1, 5, 0);
    }

    void TearDown() override {
        if (repo && repo->is_initialized()) {
            repo->shutdown();
        }
        delete repo;
        repo = nullptr;
    }

    void register_test_model(const std::string& id, ModelType type,
                            UInt16 major, UInt16 minor, UInt16 patch) {
        ModelMetadata metadata(id, id, type, SemanticVersion(major, minor, patch));
        repo->register_model("/test/" + id + ".bin", metadata);
    }

    ModelRepository* repo = nullptr;
};

TEST_F(ModelRetrievalTest, GetMetadata) {
    auto metadata = repo->get_metadata("autopilot-v1");
    ASSERT_TRUE(metadata.has_value());
    EXPECT_EQ(metadata->model_id, "autopilot-v1");
    EXPECT_EQ(metadata->type, ModelType::Autopilot);
}

TEST_F(ModelRetrievalTest, GetMetadataNonExistent) {
    auto metadata = repo->get_metadata("nonexistent");
    EXPECT_FALSE(metadata.has_value());
}

TEST_F(ModelRetrievalTest, QueryAll) {
    ModelQuery query;
    auto results = repo->query(query);
    EXPECT_EQ(results.size(), 4u);
}

TEST_F(ModelRetrievalTest, QueryByType) {
    auto query = ModelQuery::for_type(ModelType::Autopilot);
    auto results = repo->query(query);
    EXPECT_EQ(results.size(), 2u);

    for (const auto& entry : results) {
        EXPECT_EQ(entry.metadata.type, ModelType::Autopilot);
    }
}

TEST_F(ModelRetrievalTest, QueryByNamePattern) {
    auto query = ModelQuery::for_name("autopilot*");
    auto results = repo->query(query);
    EXPECT_EQ(results.size(), 2u);
}

TEST_F(ModelRetrievalTest, QueryByVersionRange) {
    auto query = ModelQuery::for_version_range(
        SemanticVersion(1, 0, 0),
        SemanticVersion(1, 9, 9));
    auto results = repo->query(query);

    // Should match: autopilot-v1 (1.0.0), behavior-v1 (1.0.0), perception-v1 (1.5.0)
    EXPECT_EQ(results.size(), 3u);
}

TEST_F(ModelRetrievalTest, QueryWithLimit) {
    ModelQuery query;
    query.limit = 2;
    auto results = repo->query(query);
    EXPECT_EQ(results.size(), 2u);
}

TEST_F(ModelRetrievalTest, QueryByTag) {
    // Add tag to one model
    auto metadata = repo->get_metadata("autopilot-v1");
    ASSERT_TRUE(metadata.has_value());
    metadata->add_tag("production", "true");
    repo->unregister_model("autopilot-v1");
    repo->register_model("/test/autopilot-v1.bin", *metadata);

    auto query = ModelQuery::for_tag("production");
    auto results = repo->query(query);
    EXPECT_EQ(results.size(), 1u);
}

TEST_F(ModelRetrievalTest, QuerySortedByVersion) {
    auto query = ModelQuery::for_type(ModelType::Autopilot);
    auto results = repo->query(query);

    // Should be sorted newest first
    ASSERT_EQ(results.size(), 2u);
    EXPECT_EQ(results[0].metadata.version.major, 2u);
    EXPECT_EQ(results[1].metadata.version.major, 1u);
}

// ============================================================================
// Cache Tests
// ============================================================================

class CacheTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create config with small cache
        auto config = RepositoryConfig::in_memory();
        config.cache.max_models = 3;
        config.cache.max_memory_bytes = 1024;

        auto temp = create_model_repository(config);
        repo = temp.release();
        ASSERT_NE(repo, nullptr);
        ASSERT_EQ(repo->initialize(), ModelResult::Success);
    }

    void TearDown() override {
        if (repo && repo->is_initialized()) {
            repo->shutdown();
        }
        delete repo;
        repo = nullptr;
    }

    ModelRepository* repo = nullptr;
};

TEST_F(CacheTest, InitialCacheEmpty) {
    EXPECT_EQ(repo->get_cached_model_count(), 0u);
}

TEST_F(CacheTest, ClearCache) {
    EXPECT_EQ(repo->clear_cache(), ModelResult::Success);
    EXPECT_EQ(repo->get_cached_model_count(), 0u);
}

TEST_F(CacheTest, ClearCacheWithoutInit) {
    auto uninit_repo = create_memory_repository();
    EXPECT_EQ(uninit_repo->clear_cache(), ModelResult::NotInitialized);
}

TEST_F(CacheTest, GetStats) {
    auto stats = repo->get_stats();
    EXPECT_EQ(stats.total_models, 0u);
    EXPECT_EQ(stats.cached_models, 0u);
    EXPECT_EQ(stats.cache_hits, 0u);
    EXPECT_EQ(stats.cache_misses, 0u);
}

TEST_F(CacheTest, StatsAfterRegistration) {
    ModelMetadata m1("model1", "Model1", ModelType::Autopilot, SemanticVersion(1, 0, 0));
    ModelMetadata m2("model2", "Model2", ModelType::Behavior, SemanticVersion(1, 0, 0));

    repo->register_model("/test/model1.bin", m1);
    repo->register_model("/test/model2.bin", m2);

    auto stats = repo->get_stats();
    EXPECT_EQ(stats.total_models, 2u);
    EXPECT_EQ(stats.count_for_type(ModelType::Autopilot), 1u);
    EXPECT_EQ(stats.count_for_type(ModelType::Behavior), 1u);
}

// ============================================================================
// Model Validation Tests
// ============================================================================

class ValidationTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto temp = create_memory_repository();
        repo = temp.release();
        ASSERT_NE(repo, nullptr);
        ASSERT_EQ(repo->initialize(), ModelResult::Success);
    }

    void TearDown() override {
        if (repo && repo->is_initialized()) {
            repo->shutdown();
        }
        delete repo;
        repo = nullptr;
    }

    ModelRepository* repo = nullptr;
};

TEST_F(ValidationTest, ValidateNonExistentModel) {
    EXPECT_EQ(repo->validate_model("nonexistent"), ModelResult::ModelNotFound);
}

TEST_F(ValidationTest, SetDeprecated) {
    ModelMetadata metadata("model1", "Model", ModelType::Custom, SemanticVersion(1, 0, 0));
    ASSERT_EQ(repo->register_model("/test/model.bin", metadata), ModelResult::Success);

    EXPECT_EQ(repo->set_deprecated("model1", true), ModelResult::Success);

    auto retrieved = repo->get_metadata("model1");
    // Note: We can't directly check status through metadata, but we can verify no error
    EXPECT_TRUE(retrieved.has_value());
}

TEST_F(ValidationTest, SetDeprecatedNonExistent) {
    EXPECT_EQ(repo->set_deprecated("nonexistent", true), ModelResult::ModelNotFound);
}

TEST_F(ValidationTest, SetDeprecatedWithoutInit) {
    auto uninit_repo = create_memory_repository();
    EXPECT_EQ(uninit_repo->set_deprecated("model", true), ModelResult::NotInitialized);
}

TEST_F(ValidationTest, QueryExcludesDeprecatedByDefault) {
    ModelMetadata m1("model1", "Model1", ModelType::Custom, SemanticVersion(1, 0, 0));
    ModelMetadata m2("model2", "Model2", ModelType::Custom, SemanticVersion(2, 0, 0));

    repo->register_model("/test/model1.bin", m1);
    repo->register_model("/test/model2.bin", m2);
    repo->set_deprecated("model1", true);

    ModelQuery query;
    query.include_deprecated = false;
    auto results = repo->query(query);

    EXPECT_EQ(results.size(), 1u);
    EXPECT_EQ(results[0].metadata.model_id, "model2");
}

TEST_F(ValidationTest, QueryIncludesDeprecatedWhenRequested) {
    ModelMetadata m1("model1", "Model1", ModelType::Custom, SemanticVersion(1, 0, 0));
    ModelMetadata m2("model2", "Model2", ModelType::Custom, SemanticVersion(2, 0, 0));

    repo->register_model("/test/model1.bin", m1);
    repo->register_model("/test/model2.bin", m2);
    repo->set_deprecated("model1", true);

    ModelQuery query;
    query.include_deprecated = true;
    auto results = repo->query(query);

    EXPECT_EQ(results.size(), 2u);
}

// ============================================================================
// Import/Export Tests
// ============================================================================

class ImportExportTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto temp = create_memory_repository();
        repo = temp.release();
        ASSERT_NE(repo, nullptr);
        ASSERT_EQ(repo->initialize(), ModelResult::Success);
    }

    void TearDown() override {
        if (repo && repo->is_initialized()) {
            repo->shutdown();
        }
        delete repo;
        repo = nullptr;
        // Clean up test file
        std::remove(test_file.c_str());
    }

    ModelRepository* repo = nullptr;
    std::string test_file = "/tmp/test_catalog.json";
};

TEST_F(ImportExportTest, ExportEmptyCatalog) {
    auto result = repo->export_catalog(test_file);
    EXPECT_EQ(result, ModelResult::Success);

    std::ifstream file(test_file);
    EXPECT_TRUE(file.good());
}

TEST_F(ImportExportTest, ExportWithModels) {
    ModelMetadata metadata("model1", "Test Model", ModelType::Autopilot, SemanticVersion(1, 0, 0));
    repo->register_model("/test/model.bin", metadata);

    auto result = repo->export_catalog(test_file);
    EXPECT_EQ(result, ModelResult::Success);

    std::ifstream file(test_file);
    EXPECT_TRUE(file.good());
}

TEST_F(ImportExportTest, ImportNonExistentFile) {
    auto result = repo->import_catalog("/nonexistent/file.json");
    EXPECT_EQ(result, ModelResult::FileNotFound);
}

TEST_F(ImportExportTest, ExportWithoutInit) {
    auto uninit_repo = create_memory_repository();
    EXPECT_EQ(uninit_repo->export_catalog(test_file), ModelResult::NotInitialized);
}

TEST_F(ImportExportTest, ImportWithoutInit) {
    auto uninit_repo = create_memory_repository();
    EXPECT_EQ(uninit_repo->import_catalog(test_file), ModelResult::NotInitialized);
}

// ============================================================================
// Factory Function Tests
// ============================================================================

TEST(FactoryTest, CreateWithCustomConfig) {
    RepositoryConfig config;
    config.base_path = "/custom/path";
    config.cache = CacheConfig::aggressive();

    auto repo = create_model_repository(config);
    EXPECT_NE(repo, nullptr);
    EXPECT_EQ(repo->get_config().base_path, "/custom/path");
}

TEST(FactoryTest, CreateMemoryRepository) {
    auto repo = create_memory_repository();
    EXPECT_NE(repo, nullptr);
    EXPECT_TRUE(repo->get_config().base_path.empty());
}

// ============================================================================
// Thread Safety Tests (Basic)
// ============================================================================

class ThreadSafetyTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto temp = create_memory_repository();
        repo = temp.release();
        ASSERT_NE(repo, nullptr);
        ASSERT_EQ(repo->initialize(), ModelResult::Success);
    }

    void TearDown() override {
        if (repo && repo->is_initialized()) {
            repo->shutdown();
        }
        delete repo;
        repo = nullptr;
    }

    ModelRepository* repo = nullptr;
};

TEST_F(ThreadSafetyTest, ConcurrentQueries) {
    // Register a model
    ModelMetadata metadata("model1", "Model", ModelType::Custom, SemanticVersion(1, 0, 0));
    repo->register_model("/test/model.bin", metadata);

    std::atomic<int> success_count{0};
    std::vector<std::thread> threads;

    for (int i = 0; i < 10; ++i) {
        threads.emplace_back([this, &success_count]() {
            ModelQuery query;
            auto results = repo->query(query);
            if (!results.empty()) {
                success_count++;
            }
        });
    }

    for (auto& t : threads) {
        t.join();
    }

    EXPECT_EQ(success_count, 10);
}
