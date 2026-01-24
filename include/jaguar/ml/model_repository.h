#pragma once
/**
 * @file model_repository.h
 * @brief ML model repository for versioning and management
 *
 * This file provides infrastructure for managing, versioning, and accessing
 * machine learning models across the game engine. Enables model registration,
 * validation, caching, and lifecycle management for ML-powered features.
 *
 * Key features:
 * - Semantic versioning with compatibility checks
 * - Model metadata and validation (checksums, schemas)
 * - Configurable caching with LRU eviction
 * - Query-based model discovery with filtering
 * - Multi-type model support (autopilot, behavior, perception, etc.)
 * - Catalog import/export for deployment pipelines
 * - Integrity verification and deprecation tracking
 */

#include "jaguar/core/types.h"
#include <vector>
#include <unordered_map>
#include <memory>
#include <optional>
#include <chrono>
#include <string>

namespace jaguar::ml {

// ============================================================================
// Forward Declarations
// ============================================================================

class IModelRepository;
class ModelRepository;
struct TensorInfo;

// ============================================================================
// Model Result Enum
// ============================================================================

/**
 * @brief Result codes for model repository operations
 */
enum class ModelResult : UInt8 {
    Success = 0,

    // Configuration errors
    InvalidConfiguration,
    InvalidPath,
    InvalidModel,
    InvalidVersion,

    // IO errors
    FileNotFound,
    ReadError,
    WriteError,
    ParseError,

    // State errors
    NotInitialized,
    AlreadyInitialized,
    ModelNotFound,
    ModelAlreadyExists,

    // Validation errors
    ChecksumMismatch,
    SchemaViolation,
    IncompatibleVersion,

    // Resource errors
    OutOfMemory,
    CacheExhausted
};

/**
 * @brief Convert ModelResult to string
 */
inline const char* model_result_to_string(ModelResult result) {
    switch (result) {
        case ModelResult::Success: return "Success";
        case ModelResult::InvalidConfiguration: return "InvalidConfiguration";
        case ModelResult::InvalidPath: return "InvalidPath";
        case ModelResult::InvalidModel: return "InvalidModel";
        case ModelResult::InvalidVersion: return "InvalidVersion";
        case ModelResult::FileNotFound: return "FileNotFound";
        case ModelResult::ReadError: return "ReadError";
        case ModelResult::WriteError: return "WriteError";
        case ModelResult::ParseError: return "ParseError";
        case ModelResult::NotInitialized: return "NotInitialized";
        case ModelResult::AlreadyInitialized: return "AlreadyInitialized";
        case ModelResult::ModelNotFound: return "ModelNotFound";
        case ModelResult::ModelAlreadyExists: return "ModelAlreadyExists";
        case ModelResult::ChecksumMismatch: return "ChecksumMismatch";
        case ModelResult::SchemaViolation: return "SchemaViolation";
        case ModelResult::IncompatibleVersion: return "IncompatibleVersion";
        case ModelResult::OutOfMemory: return "OutOfMemory";
        case ModelResult::CacheExhausted: return "CacheExhausted";
        default: return "Unknown";
    }
}

// ============================================================================
// Model Type Enum
// ============================================================================

/**
 * @brief Type of machine learning model
 */
enum class ModelType : UInt8 {
    Autopilot,      ///< Flight/ship control models
    Behavior,       ///< Tactical AI decision models
    Perception,     ///< Sensor processing models
    Prediction,     ///< Trajectory/state prediction models
    Classification, ///< Object classification models
    Custom          ///< User-defined models
};

/**
 * @brief Convert ModelType to string
 */
inline const char* model_type_to_string(ModelType type) {
    switch (type) {
        case ModelType::Autopilot: return "Autopilot";
        case ModelType::Behavior: return "Behavior";
        case ModelType::Perception: return "Perception";
        case ModelType::Prediction: return "Prediction";
        case ModelType::Classification: return "Classification";
        case ModelType::Custom: return "Custom";
        default: return "Unknown";
    }
}

// ============================================================================
// Model Status Enum
// ============================================================================

/**
 * @brief Status of a model
 */
enum class ModelStatus : UInt8 {
    Unknown,    ///< Status not determined
    Loading,    ///< Model is being loaded
    Ready,      ///< Model is ready for inference
    Error,      ///< Model failed to load
    Deprecated  ///< Model is deprecated (use newer version)
};

/**
 * @brief Convert ModelStatus to string
 */
inline const char* model_status_to_string(ModelStatus status) {
    switch (status) {
        case ModelStatus::Unknown: return "Unknown";
        case ModelStatus::Loading: return "Loading";
        case ModelStatus::Ready: return "Ready";
        case ModelStatus::Error: return "Error";
        case ModelStatus::Deprecated: return "Deprecated";
        default: return "Unknown";
    }
}

// ============================================================================
// Structs
// ============================================================================

/**
 * @brief Semantic version identifier
 */
struct SemanticVersion {
    UInt16 major{0};    ///< Major version (breaking changes)
    UInt16 minor{0};    ///< Minor version (new features)
    UInt16 patch{0};    ///< Patch version (bug fixes)

    /**
     * @brief Default constructor (version 0.0.0)
     */
    SemanticVersion() = default;

    /**
     * @brief Construct semantic version
     */
    SemanticVersion(UInt16 major_, UInt16 minor_, UInt16 patch_)
        : major(major_), minor(minor_), patch(patch_) {}

    /**
     * @brief Convert to string (e.g., "1.2.3")
     */
    std::string to_string() const {
        return std::to_string(major) + "." +
               std::to_string(minor) + "." +
               std::to_string(patch);
    }

    /**
     * @brief Parse from string (e.g., "1.2.3")
     * @param version_str String representation
     * @return Parsed version
     */
    static SemanticVersion parse(const std::string& version_str);

    /**
     * @brief Equality comparison
     */
    bool operator==(const SemanticVersion& other) const {
        return major == other.major &&
               minor == other.minor &&
               patch == other.patch;
    }

    /**
     * @brief Inequality comparison
     */
    bool operator!=(const SemanticVersion& other) const {
        return !(*this == other);
    }

    /**
     * @brief Less than comparison
     */
    bool operator<(const SemanticVersion& other) const {
        if (major != other.major) return major < other.major;
        if (minor != other.minor) return minor < other.minor;
        return patch < other.patch;
    }

    /**
     * @brief Greater than comparison
     */
    bool operator>(const SemanticVersion& other) const {
        return other < *this;
    }

    /**
     * @brief Less than or equal comparison
     */
    bool operator<=(const SemanticVersion& other) const {
        return !(other < *this);
    }

    /**
     * @brief Greater than or equal comparison
     */
    bool operator>=(const SemanticVersion& other) const {
        return !(*this < other);
    }

    /**
     * @brief Check if versions are compatible (same major version)
     */
    bool is_compatible_with(const SemanticVersion& other) const {
        return major == other.major && major > 0;
    }
};

/**
 * @brief Tensor shape and type information
 */
struct TensorInfo {
    std::string name;                   ///< Tensor name
    std::vector<Int64> shape;           ///< Tensor dimensions (-1 for dynamic)
    std::string dtype;                  ///< Data type (float32, int64, etc.)
    std::string description;            ///< Human-readable description

    /**
     * @brief Default constructor
     */
    TensorInfo() = default;

    /**
     * @brief Construct tensor info
     */
    TensorInfo(std::string name_,
               std::vector<Int64> shape_,
               std::string dtype_,
               std::string description_ = "")
        : name(std::move(name_))
        , shape(std::move(shape_))
        , dtype(std::move(dtype_))
        , description(std::move(description_)) {}

    /**
     * @brief Check if tensor has dynamic dimensions
     */
    bool is_dynamic() const {
        for (auto dim : shape) {
            if (dim < 0) return true;
        }
        return false;
    }

    /**
     * @brief Get total element count (returns 0 if dynamic)
     */
    Int64 element_count() const {
        if (is_dynamic()) return 0;
        Int64 count = 1;
        for (auto dim : shape) {
            count *= dim;
        }
        return count;
    }
};

/**
 * @brief Metadata for a machine learning model
 */
struct ModelMetadata {
    std::string model_id;                   ///< Unique identifier
    std::string name;                       ///< Human-readable name
    std::string description;                ///< Model description
    ModelType type{ModelType::Custom};      ///< Model type
    SemanticVersion version;                ///< Semantic version
    std::string author;                     ///< Model author
    std::chrono::system_clock::time_point created_at;   ///< Creation timestamp
    std::chrono::system_clock::time_point updated_at;   ///< Last update timestamp
    std::string checksum;                   ///< SHA256 checksum
    UInt64 size_bytes{0};                   ///< Model file size
    std::vector<TensorInfo> inputs;         ///< Input tensor specifications
    std::vector<TensorInfo> outputs;        ///< Output tensor specifications
    std::unordered_map<std::string, std::string> tags; ///< Key-value tags

    /**
     * @brief Default constructor
     */
    ModelMetadata() = default;

    /**
     * @brief Construct model metadata
     */
    ModelMetadata(std::string model_id_,
                  std::string name_,
                  ModelType type_,
                  SemanticVersion version_)
        : model_id(std::move(model_id_))
        , name(std::move(name_))
        , type(type_)
        , version(version_)
        , created_at(std::chrono::system_clock::now())
        , updated_at(std::chrono::system_clock::now()) {}

    /**
     * @brief Add a tag
     */
    void add_tag(const std::string& key, const std::string& value) {
        tags[key] = value;
    }

    /**
     * @brief Get a tag value
     */
    std::optional<std::string> get_tag(const std::string& key) const {
        auto it = tags.find(key);
        if (it != tags.end()) {
            return it->second;
        }
        return std::nullopt;
    }

    /**
     * @brief Check if tag exists
     */
    bool has_tag(const std::string& key) const {
        return tags.find(key) != tags.end();
    }

    /**
     * @brief Get full model identifier with version
     */
    std::string full_id() const {
        return model_id + "@" + version.to_string();
    }
};

/**
 * @brief Entry in the model repository
 */
struct ModelEntry {
    ModelMetadata metadata;                 ///< Model metadata
    std::string file_path;                  ///< Path to model file
    ModelStatus status{ModelStatus::Unknown}; ///< Current status
    std::chrono::system_clock::time_point last_accessed; ///< Last access time

    /**
     * @brief Default constructor
     */
    ModelEntry() = default;

    /**
     * @brief Construct model entry
     */
    ModelEntry(ModelMetadata metadata_,
               std::string file_path_,
               ModelStatus status_ = ModelStatus::Unknown)
        : metadata(std::move(metadata_))
        , file_path(std::move(file_path_))
        , status(status_)
        , last_accessed(std::chrono::system_clock::now()) {}

    /**
     * @brief Update last accessed time
     */
    void touch() {
        last_accessed = std::chrono::system_clock::now();
    }

    /**
     * @brief Get age since last access
     */
    std::chrono::minutes age() const {
        auto now = std::chrono::system_clock::now();
        return std::chrono::duration_cast<std::chrono::minutes>(now - last_accessed);
    }

    /**
     * @brief Check if model is usable
     */
    bool is_usable() const {
        return status == ModelStatus::Ready;
    }
};

/**
 * @brief Query parameters for model search
 */
struct ModelQuery {
    std::optional<ModelType> type;          ///< Filter by type
    std::optional<std::string> name_pattern;///< Filter by name pattern (regex)
    std::optional<SemanticVersion> min_version; ///< Minimum version (inclusive)
    std::optional<SemanticVersion> max_version; ///< Maximum version (inclusive)
    std::optional<std::string> tag;         ///< Filter by tag key
    UInt32 limit{100};                      ///< Maximum results to return
    bool include_deprecated{false};         ///< Include deprecated models

    /**
     * @brief Default constructor (query all)
     */
    ModelQuery() = default;

    /**
     * @brief Query for specific type
     */
    static ModelQuery for_type(ModelType type_, UInt32 limit_ = 100) {
        ModelQuery query;
        query.type = type_;
        query.limit = limit_;
        return query;
    }

    /**
     * @brief Query for name pattern
     */
    static ModelQuery for_name(const std::string& pattern, UInt32 limit_ = 100) {
        ModelQuery query;
        query.name_pattern = pattern;
        query.limit = limit_;
        return query;
    }

    /**
     * @brief Query for version range
     */
    static ModelQuery for_version_range(SemanticVersion min_ver,
                                        SemanticVersion max_ver,
                                        UInt32 limit_ = 100) {
        ModelQuery query;
        query.min_version = min_ver;
        query.max_version = max_ver;
        query.limit = limit_;
        return query;
    }

    /**
     * @brief Query for tag
     */
    static ModelQuery for_tag(const std::string& tag_key, UInt32 limit_ = 100) {
        ModelQuery query;
        query.tag = tag_key;
        query.limit = limit_;
        return query;
    }
};

/**
 * @brief Cache configuration
 */
struct CacheConfig {
    UInt64 max_memory_bytes{1073741824};    ///< Max memory (1GB default)
    UInt32 max_models{100};                 ///< Max cached models
    std::chrono::minutes eviction_timeout{30}; ///< Evict after timeout
    bool enable_preloading{false};          ///< Preload models on startup

    /**
     * @brief Create default cache config
     */
    static CacheConfig default_config() noexcept {
        return CacheConfig{};
    }

    /**
     * @brief Create aggressive cache config (small memory footprint)
     */
    static CacheConfig aggressive() noexcept {
        CacheConfig config;
        config.max_memory_bytes = 268435456;  // 256MB
        config.max_models = 10;
        config.eviction_timeout = std::chrono::minutes(10);
        config.enable_preloading = false;
        return config;
    }

    /**
     * @brief Create relaxed cache config (large memory footprint)
     */
    static CacheConfig relaxed() noexcept {
        CacheConfig config;
        config.max_memory_bytes = 4294967296;  // 4GB
        config.max_models = 500;
        config.eviction_timeout = std::chrono::minutes(120);
        config.enable_preloading = true;
        return config;
    }
};

/**
 * @brief Configuration for model repository
 */
struct RepositoryConfig {
    std::string base_path;                  ///< Base directory for models
    CacheConfig cache;                      ///< Cache configuration
    bool verify_checksums{true};            ///< Verify checksums on load
    bool auto_update{false};                ///< Auto-update models

    /**
     * @brief Default constructor
     */
    RepositoryConfig() = default;

    /**
     * @brief Construct repository config
     */
    RepositoryConfig(std::string base_path_, CacheConfig cache_ = CacheConfig{})
        : base_path(std::move(base_path_))
        , cache(std::move(cache_)) {}

    /**
     * @brief Create default configuration
     */
    static RepositoryConfig default_config() noexcept {
        RepositoryConfig config;
        config.base_path = "./models";
        config.cache = CacheConfig::default_config();
        return config;
    }

    /**
     * @brief Create in-memory configuration (no persistence)
     */
    static RepositoryConfig in_memory() noexcept {
        RepositoryConfig config;
        config.base_path = "";  // No persistence
        config.cache = CacheConfig::aggressive();
        config.verify_checksums = false;
        return config;
    }

    /**
     * @brief Create persistent configuration (disk-backed)
     */
    static RepositoryConfig persistent(const std::string& path) noexcept {
        RepositoryConfig config;
        config.base_path = path;
        config.cache = CacheConfig::relaxed();
        config.verify_checksums = true;
        return config;
    }
};

/**
 * @brief Statistics for model repository
 */
struct RepositoryStats {
    UInt64 total_models{0};                 ///< Total registered models
    UInt64 cached_models{0};                ///< Models in cache
    UInt64 cache_memory_used{0};            ///< Cache memory usage
    UInt64 cache_hits{0};                   ///< Cache hit count
    UInt64 cache_misses{0};                 ///< Cache miss count
    std::unordered_map<ModelType, UInt64> models_by_type; ///< Breakdown by type

    /**
     * @brief Default constructor
     */
    RepositoryStats() = default;

    /**
     * @brief Get model count for a type
     */
    UInt64 count_for_type(ModelType type) const {
        auto it = models_by_type.find(type);
        return it != models_by_type.end() ? it->second : 0;
    }

    /**
     * @brief Get cache hit rate (0.0 to 1.0)
     */
    double cache_hit_rate() const {
        UInt64 total_accesses = cache_hits + cache_misses;
        if (total_accesses == 0) return 0.0;
        return static_cast<double>(cache_hits) / static_cast<double>(total_accesses);
    }

    /**
     * @brief Get cache memory utilization (0.0 to 1.0)
     */
    double cache_utilization(const CacheConfig& config) const {
        if (config.max_memory_bytes == 0) return 0.0;
        return static_cast<double>(cache_memory_used) /
               static_cast<double>(config.max_memory_bytes);
    }

    /**
     * @brief Check if cache is near capacity
     */
    bool is_cache_near_capacity(const CacheConfig& config) const {
        return (cached_models >= config.max_models * 0.9) ||
               (cache_memory_used >= config.max_memory_bytes * 0.9);
    }
};

// ============================================================================
// Interfaces
// ============================================================================

/**
 * @brief Interface for model repository
 *
 * Defines the contract for model management operations
 */
class IModelRepository {
public:
    virtual ~IModelRepository() = default;

    // ========================================================================
    // Lifecycle
    // ========================================================================

    /**
     * @brief Initialize the repository
     * @return Success or error code
     */
    virtual ModelResult initialize() = 0;

    /**
     * @brief Shutdown the repository
     * @return Success or error code
     */
    virtual ModelResult shutdown() = 0;

    /**
     * @brief Check if initialized
     */
    virtual bool is_initialized() const = 0;

    // ========================================================================
    // Model Registration
    // ========================================================================

    /**
     * @brief Register a model
     * @param path Path to model file
     * @param metadata Model metadata
     * @return Success or error code
     */
    virtual ModelResult register_model(const std::string& path,
                                       ModelMetadata metadata) = 0;

    /**
     * @brief Unregister a model
     * @param model_id Model identifier
     * @return Success or error code
     */
    virtual ModelResult unregister_model(const std::string& model_id) = 0;

    // ========================================================================
    // Model Retrieval
    // ========================================================================

    /**
     * @brief Get model metadata
     * @param model_id Model identifier
     * @return Metadata if found, nullopt otherwise
     */
    virtual std::optional<ModelMetadata> get_metadata(const std::string& model_id) = 0;

    /**
     * @brief Query models
     * @param query Query parameters
     * @return Vector of matching model entries
     */
    virtual std::vector<ModelEntry> query(const ModelQuery& query) = 0;

    /**
     * @brief Load model data
     * @param model_id Model identifier
     * @return Model data if found, nullopt otherwise
     */
    virtual std::optional<std::vector<UInt8>> load_model_data(const std::string& model_id) = 0;

    // ========================================================================
    // Model Validation
    // ========================================================================

    /**
     * @brief Validate a model
     * @param model_id Model identifier
     * @return Success or validation error
     */
    virtual ModelResult validate_model(const std::string& model_id) = 0;

    /**
     * @brief Set model deprecation status
     * @param model_id Model identifier
     * @param deprecated Deprecation flag
     * @return Success or error code
     */
    virtual ModelResult set_deprecated(const std::string& model_id, bool deprecated) = 0;
};

// ============================================================================
// Main Model Repository Class
// ============================================================================

/**
 * @brief Central repository for ML model management
 *
 * Manages model registration, versioning, caching, and lifecycle with
 * support for semantic versioning, integrity verification, and query-based
 * discovery.
 *
 * Example usage:
 * @code
 * RepositoryConfig config = RepositoryConfig::default_config();
 * auto repo = create_model_repository(config);
 * repo->initialize();
 *
 * // Register a model
 * ModelMetadata metadata("autopilot-v1", "Autopilot Controller",
 *                        ModelType::Autopilot, SemanticVersion(1, 0, 0));
 * metadata.description = "Neural autopilot for fighter ships";
 * metadata.add_tag("architecture", "mlp");
 * metadata.add_tag("training_date", "2026-01-15");
 * repo->register_model("/models/autopilot_v1.onnx", metadata);
 *
 * // Query models
 * ModelQuery query = ModelQuery::for_type(ModelType::Autopilot);
 * auto models = repo->query(query);
 * for (const auto& entry : models) {
 *     std::cout << entry.metadata.full_id() << std::endl;
 * }
 *
 * // Load model data
 * auto data = repo->load_model_data("autopilot-v1");
 * if (data) {
 *     // Use model data for inference
 * }
 *
 * // Export catalog
 * repo->export_catalog("./model_catalog.json");
 * @endcode
 */
class ModelRepository : public IModelRepository {
public:
    /**
     * @brief Construct model repository
     * @param config Repository configuration
     */
    explicit ModelRepository(RepositoryConfig config);

    virtual ~ModelRepository();

    // ========================================================================
    // Lifecycle (from IModelRepository)
    // ========================================================================

    ModelResult initialize() override;
    ModelResult shutdown() override;
    bool is_initialized() const override;

    // ========================================================================
    // Model Registration (from IModelRepository)
    // ========================================================================

    ModelResult register_model(const std::string& path,
                               ModelMetadata metadata) override;
    ModelResult unregister_model(const std::string& model_id) override;

    // ========================================================================
    // Model Retrieval (from IModelRepository)
    // ========================================================================

    std::optional<ModelMetadata> get_metadata(const std::string& model_id) override;
    std::vector<ModelEntry> query(const ModelQuery& query) override;
    std::optional<std::vector<UInt8>> load_model_data(const std::string& model_id) override;

    // ========================================================================
    // Model Validation (from IModelRepository)
    // ========================================================================

    ModelResult validate_model(const std::string& model_id) override;
    ModelResult set_deprecated(const std::string& model_id, bool deprecated) override;

    // ========================================================================
    // Extended Features
    // ========================================================================

    /**
     * @brief Get repository configuration
     */
    const RepositoryConfig& get_config() const;

    /**
     * @brief Get total model count
     */
    UInt64 get_model_count() const;

    /**
     * @brief Get cached model count
     */
    UInt64 get_cached_model_count() const;

    /**
     * @brief Clear model cache
     * @return Success or error code
     */
    ModelResult clear_cache();

    /**
     * @brief Preload specific models into cache
     * @param model_ids Vector of model identifiers to preload
     * @return Success or error code
     */
    ModelResult preload_models(const std::vector<std::string>& model_ids);

    /**
     * @brief Export model catalog to JSON
     * @param path Output file path
     * @return Success or error code
     */
    ModelResult export_catalog(const std::string& path);

    /**
     * @brief Import model catalog from JSON
     * @param path Input file path
     * @return Success or error code
     */
    ModelResult import_catalog(const std::string& path);

    /**
     * @brief Get repository statistics
     */
    RepositoryStats get_stats() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

// ============================================================================
// Factory Functions
// ============================================================================

/**
 * @brief Create a model repository
 * @param config Repository configuration
 * @return Unique pointer to the repository
 */
std::unique_ptr<ModelRepository> create_model_repository(const RepositoryConfig& config);

/**
 * @brief Create an in-memory model repository
 * @return Unique pointer to the repository
 */
std::unique_ptr<ModelRepository> create_memory_repository();

// ============================================================================
// Serialization Functions
// ============================================================================

/**
 * @brief Parse model metadata from JSON
 * @param json JSON string
 * @return Parsed metadata
 */
ModelMetadata parse_metadata_json(const std::string& json);

/**
 * @brief Serialize model metadata to JSON
 * @param metadata Model metadata
 * @return JSON string
 */
std::string serialize_metadata_json(const ModelMetadata& metadata);

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * @brief Compute SHA256 checksum of file
 * @param file_path Path to file
 * @return Checksum string
 */
std::string compute_file_checksum(const std::string& file_path);

/**
 * @brief Verify file checksum
 * @param file_path Path to file
 * @param expected_checksum Expected checksum
 * @return True if checksum matches
 */
bool verify_file_checksum(const std::string& file_path, const std::string& expected_checksum);

/**
 * @brief Find latest version of a model
 * @param entries Vector of model entries
 * @return Entry with highest version, or nullopt if empty
 */
std::optional<ModelEntry> find_latest_version(const std::vector<ModelEntry>& entries);

/**
 * @brief Find compatible models for a version
 * @param entries Vector of model entries
 * @param target_version Target version
 * @return Vector of compatible entries
 */
std::vector<ModelEntry> find_compatible_models(const std::vector<ModelEntry>& entries,
                                               const SemanticVersion& target_version);

/**
 * @brief Get model age in hours
 * @param metadata Model metadata
 * @param current_time Current time
 * @return Age in hours
 */
inline UInt64 calculate_model_age_hours(
    const ModelMetadata& metadata,
    std::chrono::system_clock::time_point current_time = std::chrono::system_clock::now()) {
    auto duration = current_time - metadata.created_at;
    auto hours = std::chrono::duration_cast<std::chrono::hours>(duration).count();
    return hours > 0 ? static_cast<UInt64>(hours) : 0;
}

/**
 * @brief Check if model needs update
 * @param current_version Current model version
 * @param available_version Available model version
 * @return True if update recommended
 */
inline bool should_update_model(const SemanticVersion& current_version,
                                const SemanticVersion& available_version) {
    // Update if major/minor version is higher, or patch is higher with same major.minor
    return available_version > current_version &&
           available_version.is_compatible_with(current_version);
}

/**
 * @brief Format model size as human-readable string
 * @param size_bytes Size in bytes
 * @return Formatted string (e.g., "1.5 MB")
 */
inline std::string format_model_size(UInt64 size_bytes) {
    const char* units[] = {"B", "KB", "MB", "GB", "TB"};
    int unit_index = 0;
    double size = static_cast<double>(size_bytes);

    while (size >= 1024.0 && unit_index < 4) {
        size /= 1024.0;
        unit_index++;
    }

    char buffer[64];
    snprintf(buffer, sizeof(buffer), "%.2f %s", size, units[unit_index]);
    return std::string(buffer);
}

} // namespace jaguar::ml
