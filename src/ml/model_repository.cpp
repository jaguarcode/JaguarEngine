/**
 * @file model_repository.cpp
 * @brief ML model repository implementation
 */

#include "jaguar/ml/model_repository.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cstring>
#include <regex>
#include <mutex>
#include <atomic>
#include <list>

namespace jaguar::ml {

// ============================================================================
// SemanticVersion Implementation
// ============================================================================

SemanticVersion SemanticVersion::parse(const std::string& version_str) {
    SemanticVersion version;

    // Parse version string in format "major.minor.patch"
    std::istringstream iss(version_str);
    char dot;

    if (!(iss >> version.major)) {
        return SemanticVersion{0, 0, 0};
    }

    if (!(iss >> dot) || dot != '.') {
        return SemanticVersion{version.major, 0, 0};
    }

    if (!(iss >> version.minor)) {
        return SemanticVersion{version.major, 0, 0};
    }

    if (!(iss >> dot) || dot != '.') {
        return SemanticVersion{version.major, version.minor, 0};
    }

    if (!(iss >> version.patch)) {
        return SemanticVersion{version.major, version.minor, 0};
    }

    return version;
}

// ============================================================================
// Helper Functions Implementation
// ============================================================================

std::string compute_file_checksum(const std::string& file_path) {
    // Stub implementation - would need crypto library for real SHA256
    // For now, return a placeholder
    std::ifstream file(file_path, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        return "";
    }

    auto size = file.tellg();
    file.seekg(0, std::ios::beg);

    // Simple checksum based on file size and first/last bytes
    // This is NOT a real checksum - just a placeholder
    std::ostringstream oss;
    oss << "checksum_" << size;
    return oss.str();
}

bool verify_file_checksum(const std::string& file_path, const std::string& expected_checksum) {
    if (expected_checksum.empty()) {
        return true;  // No checksum to verify
    }

    std::string computed = compute_file_checksum(file_path);
    return computed == expected_checksum;
}

std::optional<ModelEntry> find_latest_version(const std::vector<ModelEntry>& entries) {
    if (entries.empty()) {
        return std::nullopt;
    }

    auto max_it = std::max_element(entries.begin(), entries.end(),
        [](const ModelEntry& a, const ModelEntry& b) {
            return a.metadata.version < b.metadata.version;
        });

    return *max_it;
}

std::vector<ModelEntry> find_compatible_models(const std::vector<ModelEntry>& entries,
                                                const SemanticVersion& target_version) {
    std::vector<ModelEntry> compatible;

    for (const auto& entry : entries) {
        if (entry.metadata.version.is_compatible_with(target_version)) {
            compatible.push_back(entry);
        }
    }

    return compatible;
}

// ============================================================================
// Serialization Functions (Stubs)
// ============================================================================

ModelMetadata parse_metadata_json(const std::string& json) {
    // Stub implementation - would need JSON library for real parsing
    (void)json;  // Suppress unused parameter warning
    ModelMetadata metadata;
    metadata.model_id = "parsed_from_json";
    return metadata;
}

std::string serialize_metadata_json(const ModelMetadata& metadata) {
    // Stub implementation - would need JSON library for real serialization
    std::ostringstream oss;
    oss << "{\n";
    oss << "  \"model_id\": \"" << metadata.model_id << "\",\n";
    oss << "  \"name\": \"" << metadata.name << "\",\n";
    oss << "  \"type\": \"" << model_type_to_string(metadata.type) << "\",\n";
    oss << "  \"version\": \"" << metadata.version.to_string() << "\"\n";
    oss << "}";
    return oss.str();
}

// ============================================================================
// ModelRepository::Impl
// ============================================================================

class ModelRepository::Impl {
public:
    RepositoryConfig config;
    std::unordered_map<std::string, ModelEntry> models;
    std::unordered_map<std::string, std::vector<UInt8>> cache;
    std::list<std::string> lru_order;
    std::atomic<bool> initialized{false};
    mutable std::mutex mutex;
    RepositoryStats stats;

    explicit Impl(RepositoryConfig config_)
        : config(std::move(config_)) {
    }

    // Helper: Check if file exists
    bool file_exists(const std::string& path) const {
        std::ifstream file(path);
        return file.good();
    }

    // Helper: Get file size
    UInt64 get_file_size(const std::string& path) const {
        std::ifstream file(path, std::ios::binary | std::ios::ate);
        if (!file.is_open()) {
            return 0;
        }
        return static_cast<UInt64>(file.tellg());
    }

    // Helper: Load file data
    std::optional<std::vector<UInt8>> load_file_data(const std::string& path) {
        std::ifstream file(path, std::ios::binary);
        if (!file.is_open()) {
            return std::nullopt;
        }

        file.seekg(0, std::ios::end);
        auto size = file.tellg();
        file.seekg(0, std::ios::beg);

        std::vector<UInt8> data(static_cast<size_t>(size));
        file.read(reinterpret_cast<char*>(data.data()), size);

        if (!file) {
            return std::nullopt;
        }

        return data;
    }

    // Helper: Match pattern with wildcards
    bool matches_pattern(const std::string& text, const std::string& pattern) const {
        // Simple wildcard matching (* and ?)
        try {
            std::string regex_pattern = pattern;
            // Replace * with .*
            size_t pos = 0;
            while ((pos = regex_pattern.find('*', pos)) != std::string::npos) {
                regex_pattern.replace(pos, 1, ".*");
                pos += 2;
            }
            // Replace ? with .
            pos = 0;
            while ((pos = regex_pattern.find('?', pos)) != std::string::npos) {
                regex_pattern.replace(pos, 1, ".");
                pos += 1;
            }

            std::regex re(regex_pattern, std::regex_constants::icase);
            return std::regex_match(text, re);
        } catch (...) {
            // If regex fails, fall back to exact match
            return text == pattern;
        }
    }

    // Helper: Evict LRU cache entries
    void evict_cache_if_needed(UInt64 required_bytes) {
        std::lock_guard<std::mutex> lock(mutex);

        // Check if we need to evict
        while (!lru_order.empty() &&
               (stats.cached_models >= config.cache.max_models ||
                stats.cache_memory_used + required_bytes > config.cache.max_memory_bytes)) {

            // Remove least recently used
            const std::string& model_id = lru_order.front();

            auto cache_it = cache.find(model_id);
            if (cache_it != cache.end()) {
                stats.cache_memory_used -= cache_it->second.size();
                cache.erase(cache_it);
                stats.cached_models--;
            }

            lru_order.pop_front();
        }
    }

    // Helper: Update LRU order
    void touch_cache_entry(const std::string& model_id) {
        // Remove from current position
        lru_order.remove(model_id);
        // Add to back (most recently used)
        lru_order.push_back(model_id);
    }

    // Helper: Add to cache
    void add_to_cache(const std::string& model_id, const std::vector<UInt8>& data) {
        std::lock_guard<std::mutex> lock(mutex);

        // Remove if already cached
        auto it = cache.find(model_id);
        if (it != cache.end()) {
            stats.cache_memory_used -= it->second.size();
            cache.erase(it);
            stats.cached_models--;
        }

        // Evict if needed
        UInt64 required_bytes = data.size();
        while (!lru_order.empty() &&
               (stats.cached_models >= config.cache.max_models ||
                stats.cache_memory_used + required_bytes > config.cache.max_memory_bytes)) {

            const std::string& evict_id = lru_order.front();
            auto cache_it = cache.find(evict_id);
            if (cache_it != cache.end()) {
                stats.cache_memory_used -= cache_it->second.size();
                cache.erase(cache_it);
                stats.cached_models--;
            }
            lru_order.pop_front();
        }

        // Add to cache
        cache[model_id] = data;
        stats.cache_memory_used += data.size();
        stats.cached_models++;
        touch_cache_entry(model_id);
    }

    // Helper: Scan base path for models
    ModelResult scan_base_path() {
        if (config.base_path.empty()) {
            return ModelResult::Success;  // No base path to scan
        }

        // Stub: In a real implementation, this would scan the directory
        // and register any model files found
        return ModelResult::Success;
    }
};

// ============================================================================
// ModelRepository Implementation
// ============================================================================

ModelRepository::ModelRepository(RepositoryConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {
}

ModelRepository::~ModelRepository() = default;

ModelResult ModelRepository::initialize() {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (impl_->initialized.load()) {
        return ModelResult::AlreadyInitialized;
    }

    // Scan base path if configured
    if (!impl_->config.base_path.empty()) {
        auto result = impl_->scan_base_path();
        if (result != ModelResult::Success) {
            return result;
        }
    }

    impl_->initialized.store(true);
    return ModelResult::Success;
}

ModelResult ModelRepository::shutdown() {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->initialized.load()) {
        return ModelResult::NotInitialized;
    }

    // Clear cache
    impl_->cache.clear();
    impl_->lru_order.clear();
    impl_->stats.cached_models = 0;
    impl_->stats.cache_memory_used = 0;

    impl_->initialized.store(false);
    return ModelResult::Success;
}

bool ModelRepository::is_initialized() const {
    return impl_->initialized.load();
}

ModelResult ModelRepository::register_model(const std::string& path,
                                             ModelMetadata metadata) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->initialized.load()) {
        return ModelResult::NotInitialized;
    }

    // Check if model already exists
    if (impl_->models.find(metadata.model_id) != impl_->models.end()) {
        return ModelResult::ModelAlreadyExists;
    }

    // Validate path exists (unless in-memory mode)
    if (!impl_->config.base_path.empty() && !impl_->file_exists(path)) {
        return ModelResult::FileNotFound;
    }

    // Get file size if path exists
    if (impl_->file_exists(path)) {
        metadata.size_bytes = impl_->get_file_size(path);
    }

    // Create model entry
    ModelEntry entry(std::move(metadata), path, ModelStatus::Ready);

    // Update stats
    impl_->stats.total_models++;
    impl_->stats.models_by_type[entry.metadata.type]++;

    // Add to registry
    impl_->models[entry.metadata.model_id] = std::move(entry);

    return ModelResult::Success;
}

ModelResult ModelRepository::unregister_model(const std::string& model_id) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->initialized.load()) {
        return ModelResult::NotInitialized;
    }

    auto it = impl_->models.find(model_id);
    if (it == impl_->models.end()) {
        return ModelResult::ModelNotFound;
    }

    // Update stats
    impl_->stats.total_models--;
    ModelType type = it->second.metadata.type;
    if (impl_->stats.models_by_type[type] > 0) {
        impl_->stats.models_by_type[type]--;
    }

    // Remove from cache if present
    auto cache_it = impl_->cache.find(model_id);
    if (cache_it != impl_->cache.end()) {
        impl_->stats.cache_memory_used -= cache_it->second.size();
        impl_->cache.erase(cache_it);
        impl_->stats.cached_models--;
        impl_->lru_order.remove(model_id);
    }

    // Remove from registry
    impl_->models.erase(it);

    return ModelResult::Success;
}

std::optional<ModelMetadata> ModelRepository::get_metadata(const std::string& model_id) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->initialized.load()) {
        return std::nullopt;
    }

    auto it = impl_->models.find(model_id);
    if (it == impl_->models.end()) {
        return std::nullopt;
    }

    return it->second.metadata;
}

std::vector<ModelEntry> ModelRepository::query(const ModelQuery& query) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->initialized.load()) {
        return {};
    }

    std::vector<ModelEntry> results;

    for (auto& [model_id, entry] : impl_->models) {
        // Filter by type
        if (query.type.has_value() && entry.metadata.type != query.type.value()) {
            continue;
        }

        // Filter by name pattern
        if (query.name_pattern.has_value()) {
            if (!impl_->matches_pattern(entry.metadata.name, query.name_pattern.value())) {
                continue;
            }
        }

        // Filter by version range
        if (query.min_version.has_value()) {
            if (entry.metadata.version < query.min_version.value()) {
                continue;
            }
        }

        if (query.max_version.has_value()) {
            if (entry.metadata.version > query.max_version.value()) {
                continue;
            }
        }

        // Filter by tag
        if (query.tag.has_value()) {
            if (!entry.metadata.has_tag(query.tag.value())) {
                continue;
            }
        }

        // Filter deprecated models
        if (!query.include_deprecated && entry.status == ModelStatus::Deprecated) {
            continue;
        }

        results.push_back(entry);

        // Check limit
        if (results.size() >= query.limit) {
            break;
        }
    }

    // Sort by version (newest first)
    std::sort(results.begin(), results.end(),
        [](const ModelEntry& a, const ModelEntry& b) {
            return a.metadata.version > b.metadata.version;
        });

    return results;
}

std::optional<std::vector<UInt8>> ModelRepository::load_model_data(const std::string& model_id) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->initialized.load()) {
        return std::nullopt;
    }

    // Find model entry
    auto model_it = impl_->models.find(model_id);
    if (model_it == impl_->models.end()) {
        impl_->stats.cache_misses++;
        return std::nullopt;
    }

    ModelEntry& entry = model_it->second;

    // Check cache first
    auto cache_it = impl_->cache.find(model_id);
    if (cache_it != impl_->cache.end()) {
        impl_->stats.cache_hits++;
        impl_->touch_cache_entry(model_id);
        entry.touch();
        return cache_it->second;
    }

    // Cache miss - load from disk
    impl_->stats.cache_misses++;

    auto data = impl_->load_file_data(entry.file_path);
    if (!data.has_value()) {
        entry.status = ModelStatus::Error;
        return std::nullopt;
    }

    // Add to cache
    impl_->add_to_cache(model_id, data.value());

    // Update entry
    entry.touch();
    entry.status = ModelStatus::Ready;

    return data;
}

ModelResult ModelRepository::validate_model(const std::string& model_id) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->initialized.load()) {
        return ModelResult::NotInitialized;
    }

    auto it = impl_->models.find(model_id);
    if (it == impl_->models.end()) {
        return ModelResult::ModelNotFound;
    }

    ModelEntry& entry = it->second;

    // Check if file exists
    if (!impl_->file_exists(entry.file_path)) {
        entry.status = ModelStatus::Error;
        return ModelResult::FileNotFound;
    }

    // Verify checksum if enabled
    if (impl_->config.verify_checksums && !entry.metadata.checksum.empty()) {
        if (!verify_file_checksum(entry.file_path, entry.metadata.checksum)) {
            entry.status = ModelStatus::Error;
            return ModelResult::ChecksumMismatch;
        }
    }

    entry.status = ModelStatus::Ready;
    return ModelResult::Success;
}

ModelResult ModelRepository::set_deprecated(const std::string& model_id, bool deprecated) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->initialized.load()) {
        return ModelResult::NotInitialized;
    }

    auto it = impl_->models.find(model_id);
    if (it == impl_->models.end()) {
        return ModelResult::ModelNotFound;
    }

    it->second.status = deprecated ? ModelStatus::Deprecated : ModelStatus::Ready;
    return ModelResult::Success;
}

const RepositoryConfig& ModelRepository::get_config() const {
    return impl_->config;
}

UInt64 ModelRepository::get_model_count() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->stats.total_models;
}

UInt64 ModelRepository::get_cached_model_count() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->stats.cached_models;
}

ModelResult ModelRepository::clear_cache() {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->initialized.load()) {
        return ModelResult::NotInitialized;
    }

    impl_->cache.clear();
    impl_->lru_order.clear();
    impl_->stats.cached_models = 0;
    impl_->stats.cache_memory_used = 0;

    return ModelResult::Success;
}

ModelResult ModelRepository::preload_models(const std::vector<std::string>& model_ids) {
    if (!impl_->initialized.load()) {
        return ModelResult::NotInitialized;
    }

    for (const auto& model_id : model_ids) {
        // Load model data (this will add it to cache)
        auto data = load_model_data(model_id);
        if (!data.has_value()) {
            // Continue loading other models even if one fails
            continue;
        }
    }

    return ModelResult::Success;
}

ModelResult ModelRepository::export_catalog(const std::string& path) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->initialized.load()) {
        return ModelResult::NotInitialized;
    }

    // Stub implementation
    std::ofstream file(path);
    if (!file.is_open()) {
        return ModelResult::WriteError;
    }

    file << "{\n";
    file << "  \"models\": [\n";

    bool first = true;
    for (const auto& [model_id, entry] : impl_->models) {
        if (!first) {
            file << ",\n";
        }
        file << "    " << serialize_metadata_json(entry.metadata);
        first = false;
    }

    file << "\n  ]\n";
    file << "}\n";

    return ModelResult::Success;
}

ModelResult ModelRepository::import_catalog(const std::string& path) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->initialized.load()) {
        return ModelResult::NotInitialized;
    }

    // Stub implementation
    std::ifstream file(path);
    if (!file.is_open()) {
        return ModelResult::FileNotFound;
    }

    // In a real implementation, this would parse JSON and register models
    return ModelResult::Success;
}

RepositoryStats ModelRepository::get_stats() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->stats;
}

// ============================================================================
// Factory Functions
// ============================================================================

std::unique_ptr<ModelRepository> create_model_repository(const RepositoryConfig& config) {
    return std::make_unique<ModelRepository>(config);
}

std::unique_ptr<ModelRepository> create_memory_repository() {
    return std::make_unique<ModelRepository>(RepositoryConfig::in_memory());
}

} // namespace jaguar::ml
