#pragma once
/**
 * @file component_force_registry.h
 * @brief Per-entity component force model registry
 *
 * Enables entity-specific force model attachment for heterogeneous
 * entity populations (e.g., different aerodynamic models per aircraft type).
 */

#include "jaguar/core/types.h"
#include "jaguar/physics/entity.h"
#include "jaguar/physics/force.h"
#include <memory>
#include <unordered_map>
#include <shared_mutex>
#include <typeindex>
#include <algorithm>

namespace jaguar::physics {

/**
 * @brief Registry for per-entity force model components
 *
 * Stores and manages force model instances attached to specific entities.
 * Thread-safe for concurrent read access during parallel force computation.
 *
 * Usage:
 * @code
 * ComponentForceRegistry registry;
 * auto aero = std::make_unique<F16AerodynamicsModel>();
 * registry.register_model<IAerodynamicsModel>(entity_id, std::move(aero));
 *
 * // Later, during force computation:
 * if (auto* model = registry.get<IAerodynamicsModel>(entity_id)) {
 *     model->compute_forces(state, env, dt, forces);
 * }
 * @endcode
 */
class ComponentForceRegistry {
public:
    ComponentForceRegistry() = default;
    ~ComponentForceRegistry() = default;

    // Non-copyable
    ComponentForceRegistry(const ComponentForceRegistry&) = delete;
    ComponentForceRegistry& operator=(const ComponentForceRegistry&) = delete;

    /**
     * @brief Register a force model for an entity
     * @tparam T Force model type (must derive from IForceGenerator)
     * @param entity_id Entity to attach the model to
     * @param model Force model instance
     */
    template<typename T>
    void register_model(EntityId entity_id, std::unique_ptr<T> model);

    /**
     * @brief Get force model for an entity
     * @tparam T Force model type to retrieve
     * @param entity_id Entity ID
     * @return Pointer to model, or nullptr if not found
     */
    template<typename T>
    T* get(EntityId entity_id) const;

    /**
     * @brief Get all force generators for an entity
     * @param entity_id Entity ID
     * @return Vector of force generator pointers
     */
    std::vector<IForceGenerator*> get_all(EntityId entity_id) const;

    /**
     * @brief Check if entity has a specific model type
     */
    template<typename T>
    bool has(EntityId entity_id) const;

    /**
     * @brief Remove all models for an entity
     */
    void remove_entity(EntityId entity_id);

    /**
     * @brief Remove specific model type from entity
     */
    template<typename T>
    void remove_model(EntityId entity_id);

    /**
     * @brief Clear all registered models
     */
    void clear();

    /**
     * @brief Get number of entities with registered models
     */
    SizeT entity_count() const;

    /**
     * @brief Get total number of registered models
     */
    SizeT model_count() const;

private:
    // Key: (entity_id, type_index) -> force model
    struct ModelKey {
        EntityId entity_id;
        std::type_index type_idx;

        bool operator==(const ModelKey& other) const {
            return entity_id == other.entity_id && type_idx == other.type_idx;
        }
    };

    struct ModelKeyHash {
        std::size_t operator()(const ModelKey& key) const {
            return std::hash<EntityId>{}(key.entity_id) ^
                   (std::hash<std::type_index>{}(key.type_idx) << 1);
        }
    };

    mutable std::shared_mutex mutex_;
    std::unordered_map<ModelKey, std::unique_ptr<IForceGenerator>, ModelKeyHash> models_;
    std::unordered_map<EntityId, std::vector<std::type_index>> entity_types_;
};

// ============================================================================
// Template Implementations
// ============================================================================

template<typename T>
void ComponentForceRegistry::register_model(EntityId entity_id, std::unique_ptr<T> model) {
    static_assert(std::is_base_of_v<IForceGenerator, T>,
                  "T must derive from IForceGenerator");

    std::unique_lock lock(mutex_);

    ModelKey key{entity_id, std::type_index(typeid(T))};
    models_[key] = std::move(model);
    entity_types_[entity_id].push_back(std::type_index(typeid(T)));
}

template<typename T>
T* ComponentForceRegistry::get(EntityId entity_id) const {
    static_assert(std::is_base_of_v<IForceGenerator, T>,
                  "T must derive from IForceGenerator");

    std::shared_lock lock(mutex_);

    ModelKey key{entity_id, std::type_index(typeid(T))};
    auto it = models_.find(key);
    if (it != models_.end()) {
        return static_cast<T*>(it->second.get());
    }
    return nullptr;
}

template<typename T>
bool ComponentForceRegistry::has(EntityId entity_id) const {
    return get<T>(entity_id) != nullptr;
}

template<typename T>
void ComponentForceRegistry::remove_model(EntityId entity_id) {
    std::unique_lock lock(mutex_);

    ModelKey key{entity_id, std::type_index(typeid(T))};
    models_.erase(key);

    auto it = entity_types_.find(entity_id);
    if (it != entity_types_.end()) {
        auto& types = it->second;
        types.erase(std::remove(types.begin(), types.end(),
                                std::type_index(typeid(T))), types.end());
        if (types.empty()) {
            entity_types_.erase(it);
        }
    }
}

} // namespace jaguar::physics
