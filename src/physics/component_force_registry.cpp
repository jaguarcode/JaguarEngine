/**
 * @file component_force_registry.cpp
 * @brief Per-entity component force registry implementation
 */

#include "jaguar/physics/component_force_registry.h"
#include <shared_mutex>

namespace jaguar::physics {

std::vector<IForceGenerator*> ComponentForceRegistry::get_all(EntityId entity_id) const {
    std::shared_lock<std::shared_mutex> lock(mutex_);

    std::vector<IForceGenerator*> result;

    auto types_it = entity_types_.find(entity_id);
    if (types_it == entity_types_.end()) {
        return result;
    }

    result.reserve(types_it->second.size());

    for (const auto& type_idx : types_it->second) {
        ModelKey key{entity_id, type_idx};
        auto model_it = models_.find(key);
        if (model_it != models_.end()) {
            result.push_back(model_it->second.get());
        }
    }

    return result;
}

void ComponentForceRegistry::remove_entity(EntityId entity_id) {
    std::unique_lock<std::shared_mutex> lock(mutex_);

    auto types_it = entity_types_.find(entity_id);
    if (types_it != entity_types_.end()) {
        for (const auto& type_idx : types_it->second) {
            ModelKey key{entity_id, type_idx};
            models_.erase(key);
        }
        entity_types_.erase(types_it);
    }
}

void ComponentForceRegistry::clear() {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    models_.clear();
    entity_types_.clear();
}

SizeT ComponentForceRegistry::entity_count() const {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return entity_types_.size();
}

SizeT ComponentForceRegistry::model_count() const {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return models_.size();
}

} // namespace jaguar::physics
