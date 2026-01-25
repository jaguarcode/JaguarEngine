/**
 * @file entity_manager.cpp
 * @brief Entity management implementation
 */

#include "jaguar/physics/entity.h"
#include "jaguar/events/event.h"
#include "jaguar/events/event_dispatcher.h"
#include "jaguar/core/simd.h"
#include <vector>

namespace jaguar::physics {

// ============================================================================
// EntityStateStorage Implementation
// ============================================================================

void EntityStateStorage::reserve(SizeT capacity) {
    positions_.reserve(capacity);
    velocities_.reserve(capacity);
    orientations_.reserve(capacity);
    angular_velocities_.reserve(capacity);
    accelerations_.reserve(capacity);
    angular_accelerations_.reserve(capacity);
    masses_.reserve(capacity);
    inertias_.reserve(capacity);
    forces_.reserve(capacity);
    active_flags_.reserve(capacity);
}

UInt32 EntityStateStorage::allocate() {
    if (!free_indices_.empty()) {
        UInt32 idx = free_indices_.back();
        free_indices_.pop_back();
        active_flags_[idx] = true;
        active_count_++;
        return idx;
    }

    UInt32 idx = static_cast<UInt32>(positions_.size());
    positions_.push_back(Vec3{});
    velocities_.push_back(Vec3{});
    orientations_.push_back(Quat::Identity());
    angular_velocities_.push_back(Vec3{});
    accelerations_.push_back(Vec3{});
    angular_accelerations_.push_back(Vec3{});
    masses_.push_back(1.0);
    inertias_.push_back(Mat3x3::Identity());
    forces_.push_back(EntityForces{});
    active_flags_.push_back(true);
    active_count_++;
    return idx;
}

void EntityStateStorage::free(UInt32 index) {
    if (index < active_flags_.size() && active_flags_[index]) {
        active_flags_[index] = false;
        free_indices_.push_back(index);
        active_count_--;
    }
}

void EntityStateStorage::set_active(UInt32 idx, bool active) {
    if (idx < active_flags_.size()) {
        if (active_flags_[idx] != active) {
            active_flags_[idx] = active;
            if (active) {
                active_count_++;
            } else {
                active_count_--;
            }
        }
    }
}

void EntityStateStorage::set_state(UInt32 idx, const EntityState& state) {
    if (idx >= positions_.size()) return;

    positions_[idx] = state.position;
    velocities_[idx] = state.velocity;
    orientations_[idx] = state.orientation;
    angular_velocities_[idx] = state.angular_velocity;
    accelerations_[idx] = state.acceleration;
    angular_accelerations_[idx] = state.angular_accel;
    masses_[idx] = state.mass;
    inertias_[idx] = state.inertia;
}

EntityState EntityStateStorage::get_state(UInt32 idx) const {
    EntityState state;
    if (idx < positions_.size()) {
        state.position = positions_[idx];
        state.velocity = velocities_[idx];
        state.orientation = orientations_[idx];
        state.angular_velocity = angular_velocities_[idx];
        state.acceleration = accelerations_[idx];
        state.angular_accel = angular_accelerations_[idx];
        state.mass = masses_[idx];
        state.inertia = inertias_[idx];
        // Mark inverse inertia cache as dirty since this is a fresh copy
        state.inverse_inertia_dirty = true;
    }
    return state;
}

void EntityStateStorage::clear_forces() {
    for (auto& f : forces_) {
        f.clear();
    }
}

// ============================================================================
// SIMD Batch Operations
// ============================================================================

void EntityStateStorage::batch_integrate_positions(Real dt) {
    SizeT count = positions_.size();
    if (count == 0) return;

    // Extract separate x, y, z arrays for SIMD processing
    // Note: For maximum efficiency, the storage could use separate x, y, z arrays
    // This approach trades some overhead for compatibility with existing code
    std::vector<Real> px(count), py(count), pz(count);
    std::vector<Real> vx(count), vy(count), vz(count);

    for (SizeT i = 0; i < count; ++i) {
        px[i] = positions_[i].x;
        py[i] = positions_[i].y;
        pz[i] = positions_[i].z;
        vx[i] = velocities_[i].x;
        vy[i] = velocities_[i].y;
        vz[i] = velocities_[i].z;
    }

    // Apply SIMD batch update
    simd::batch_position_update(
        px.data(), py.data(), pz.data(),
        vx.data(), vy.data(), vz.data(),
        count, dt);

    // Write back results (only update active entities)
    for (SizeT i = 0; i < count; ++i) {
        if (active_flags_[i]) {
            positions_[i].x = px[i];
            positions_[i].y = py[i];
            positions_[i].z = pz[i];
        }
    }
}

void EntityStateStorage::batch_integrate_velocities(Real dt) {
    SizeT count = velocities_.size();
    if (count == 0) return;

    std::vector<Real> vx(count), vy(count), vz(count);
    std::vector<Real> ax(count), ay(count), az(count);

    for (SizeT i = 0; i < count; ++i) {
        vx[i] = velocities_[i].x;
        vy[i] = velocities_[i].y;
        vz[i] = velocities_[i].z;
        ax[i] = accelerations_[i].x;
        ay[i] = accelerations_[i].y;
        az[i] = accelerations_[i].z;
    }

    simd::batch_velocity_update(
        vx.data(), vy.data(), vz.data(),
        ax.data(), ay.data(), az.data(),
        count, dt);

    for (SizeT i = 0; i < count; ++i) {
        if (active_flags_[i]) {
            velocities_[i].x = vx[i];
            velocities_[i].y = vy[i];
            velocities_[i].z = vz[i];
        }
    }
}

void EntityStateStorage::batch_compute_accelerations() {
    SizeT count = forces_.size();
    if (count == 0) return;

    std::vector<Real> ax(count), ay(count), az(count);
    std::vector<Real> fx(count), fy(count), fz(count);

    for (SizeT i = 0; i < count; ++i) {
        fx[i] = forces_[i].force.x;
        fy[i] = forces_[i].force.y;
        fz[i] = forces_[i].force.z;
    }

    simd::batch_force_to_acceleration(
        ax.data(), ay.data(), az.data(),
        fx.data(), fy.data(), fz.data(),
        masses_.data(),
        count);

    for (SizeT i = 0; i < count; ++i) {
        if (active_flags_[i]) {
            accelerations_[i].x = ax[i];
            accelerations_[i].y = ay[i];
            accelerations_[i].z = az[i];
        }
    }
}

void EntityStateStorage::batch_apply_gravity(Real g) {
    SizeT count = forces_.size();
    if (count == 0) return;

    std::vector<Real> fz(count);

    for (SizeT i = 0; i < count; ++i) {
        fz[i] = forces_[i].force.z;
    }

    simd::batch_add_gravity(fz.data(), masses_.data(), g, count);

    for (SizeT i = 0; i < count; ++i) {
        if (active_flags_[i]) {
            forces_[i].force.z = fz[i];
        }
    }
}

// ============================================================================
// EntityManager Implementation
// ============================================================================

EntityManager::EntityManager() {
    state_storage_.reserve(1000);  // Default capacity
}

EntityManager::~EntityManager() = default;

EntityId EntityManager::create_entity(const std::string& name, Domain domain) {
    EntityId id = next_id_++;

    Entity entity;
    entity.id = id;
    entity.state_index = state_storage_.allocate();
    entity.components = 0;
    entity.primary_domain = domain;
    entity.name = name;
    entity.active = true;

    entities_[id] = std::move(entity);

    // Emit EntityCreated event
    if (event_dispatcher_) {
        auto event = events::Event::create_entity_created(id, name, domain, current_time_);
        event_dispatcher_->dispatch(event);
    }

    return id;
}

void EntityManager::destroy_entity(EntityId id) {
    auto it = entities_.find(id);
    if (it != entities_.end()) {
        // Emit EntityDestroyed event before destroying
        if (event_dispatcher_) {
            auto event = events::Event::create_entity_destroyed(
                id, it->second.name, current_time_);
            event_dispatcher_->dispatch(event);
        }

        state_storage_.free(it->second.state_index);
        entities_.erase(it);
    }
}

bool EntityManager::exists(EntityId id) const {
    return entities_.count(id) > 0;
}

Entity* EntityManager::get_entity(EntityId id) {
    auto it = entities_.find(id);
    return it != entities_.end() ? &it->second : nullptr;
}

const Entity* EntityManager::get_entity(EntityId id) const {
    auto it = entities_.find(id);
    return it != entities_.end() ? &it->second : nullptr;
}

EntityState EntityManager::get_state(EntityId id) const {
    const Entity* entity = get_entity(id);
    if (entity) {
        return state_storage_.get_state(entity->state_index);
    }
    return EntityState{};
}

void EntityManager::set_state(EntityId id, const EntityState& state) {
    Entity* entity = get_entity(id);
    if (entity) {
        state_storage_.set_state(entity->state_index, state);
    }
}

SizeT EntityManager::active_entity_count() const {
    SizeT count = 0;
    for (const auto& [id, entity] : entities_) {
        if (entity.active) count++;
    }
    return count;
}

EntityState* EntityManager::get_entity_state(EntityId id) {
    Entity* entity = get_entity(id);
    if (!entity) {
        return nullptr;
    }

    // Check if already in cache
    auto it = constraint_state_cache_.find(id);
    if (it != constraint_state_cache_.end()) {
        return &it->second;
    }

    // Load state from SoA storage into cache
    EntityState state = state_storage_.get_state(entity->state_index);
    auto [inserted_it, success] = constraint_state_cache_.emplace(id, state);
    return &inserted_it->second;
}

void EntityManager::sync_entity_states() {
    // Write back all cached states to SoA storage
    for (auto& [id, state] : constraint_state_cache_) {
        Entity* entity = get_entity(id);
        if (entity) {
            state_storage_.set_state(entity->state_index, state);
        }
    }

    // Clear the cache for next frame
    constraint_state_cache_.clear();
}

void EntityManager::set_event_dispatcher(events::EventDispatcher* dispatcher) {
    event_dispatcher_ = dispatcher;
}

} // namespace jaguar::physics
