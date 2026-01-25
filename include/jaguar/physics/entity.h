#pragma once
/**
 * @file entity.h
 * @brief Entity management and state storage for physics simulation
 *
 * Implements Data-Oriented Design (DOD) patterns with Structure of Arrays (SoA)
 * memory layout for cache-efficient physics processing.
 */

#include "jaguar/core/types.h"
#include <vector>
#include <memory>
#include <string>
#include <functional>

// Forward declaration for event system integration
namespace jaguar::events {
    class EventDispatcher;
}

namespace jaguar::physics {

// ============================================================================
// Entity State
// ============================================================================

/**
 * @brief Complete physical state of an entity
 *
 * This is the "Array of Structures" view for convenient access.
 * Internally, data is stored in SoA format for performance.
 */
struct EntityState {
    Vec3 position{0.0, 0.0, 0.0};      ///< Position in ECEF (meters)
    Vec3 velocity{0.0, 0.0, 0.0};      ///< Velocity in ECEF (m/s)
    Quat orientation{1.0, 0.0, 0.0, 0.0}; ///< Orientation (body to ECEF)
    Vec3 angular_velocity{0.0, 0.0, 0.0}; ///< Angular velocity in body frame (rad/s)
    Vec3 acceleration{0.0, 0.0, 0.0};  ///< Linear acceleration (m/s²)
    Vec3 angular_accel{0.0, 0.0, 0.0}; ///< Angular acceleration (rad/s²)
    Real mass{1.0};                     ///< Mass (kg)
    Mat3x3 inertia{};                   ///< Inertia tensor in body frame (kg·m²)

    // Cached inverse inertia to avoid repeated matrix inversions
    mutable Mat3x3 inverse_inertia_cached{};  ///< Cached inverse inertia tensor
    mutable bool inverse_inertia_dirty{true}; ///< Flag indicating cache needs update

    /**
     * @brief Get inverse inertia tensor (cached)
     *
     * Computes and caches the inverse inertia tensor on first access or when
     * the inertia tensor is modified. This avoids expensive matrix inversions
     * during constraint solving iterations.
     *
     * @return Reference to cached inverse inertia tensor
     */
    const Mat3x3& get_inverse_inertia() const {
        if (inverse_inertia_dirty) {
            // Compute inverse using determinant method
            Real det = inertia.determinant();
            if (std::abs(det) > 1e-10) {
                inverse_inertia_cached = inertia.inverse();
            } else {
                // Fallback to diagonal inverse for singular/near-singular matrix
                inverse_inertia_cached = Mat3x3::Identity();
                if (inertia(0,0) > 1e-10) inverse_inertia_cached(0,0) = 1.0 / inertia(0,0);
                if (inertia(1,1) > 1e-10) inverse_inertia_cached(1,1) = 1.0 / inertia(1,1);
                if (inertia(2,2) > 1e-10) inverse_inertia_cached(2,2) = 1.0 / inertia(2,2);
            }
            inverse_inertia_dirty = false;
        }
        return inverse_inertia_cached;
    }

    /**
     * @brief Invalidate inverse inertia cache
     *
     * Call this whenever the inertia tensor is modified to ensure the cached
     * inverse is recomputed on next access.
     */
    void invalidate_inertia_cache() {
        inverse_inertia_dirty = true;
    }
};

/**
 * @brief Accumulated forces and torques acting on an entity
 */
struct EntityForces {
    Vec3 force{0.0, 0.0, 0.0};   ///< Total force in body frame (N)
    Vec3 torque{0.0, 0.0, 0.0};  ///< Total torque in body frame (N·m)

    void clear() noexcept {
        force = Vec3{0.0, 0.0, 0.0};
        torque = Vec3{0.0, 0.0, 0.0};
    }

    void add_force(const Vec3& f) noexcept {
        force += f;
    }

    void add_torque(const Vec3& t) noexcept {
        torque += t;
    }

    void add_force_at_point(const Vec3& f, const Vec3& r) noexcept {
        force += f;
        torque += r.cross(f);
    }
};

// ============================================================================
// Entity Definition
// ============================================================================

/**
 * @brief Lightweight entity handle
 *
 * Entity is just an identifier with metadata. Actual physics data
 * is stored in EntityStateStorage using the state_index.
 */
struct Entity {
    EntityId id{INVALID_ENTITY_ID};     ///< Unique identifier
    UInt32 state_index{0};              ///< Index into SoA state arrays
    ComponentMask components{0};         ///< Bitfield of attached components
    Domain primary_domain{Domain::Generic}; ///< Primary physics domain
    std::string name;                    ///< Entity name (for debugging/scripting)
    bool active{true};                   ///< Is entity being simulated

    bool is_valid() const noexcept {
        return id != INVALID_ENTITY_ID;
    }
};

// ============================================================================
// Component Types (bit positions in ComponentMask)
// ============================================================================

namespace ComponentBits {
    constexpr ComponentMask Aerodynamics   = 1ULL << 0;
    constexpr ComponentMask Propulsion     = 1ULL << 1;
    constexpr ComponentMask GroundContact  = 1ULL << 2;
    constexpr ComponentMask Terramechanics = 1ULL << 3;
    constexpr ComponentMask Hydrodynamics  = 1ULL << 4;
    constexpr ComponentMask Buoyancy       = 1ULL << 5;
    constexpr ComponentMask OrbitalDynamics= 1ULL << 6;
    constexpr ComponentMask Gravity        = 1ULL << 7;
    constexpr ComponentMask Missile        = 1ULL << 8;
    constexpr ComponentMask Seeker         = 1ULL << 9;
    constexpr ComponentMask Guidance       = 1ULL << 10;
    constexpr ComponentMask FlightControl  = 1ULL << 11;
    constexpr ComponentMask Suspension     = 1ULL << 12;
    constexpr ComponentMask Collision      = 1ULL << 13;
    constexpr ComponentMask Damageable     = 1ULL << 14;
}

// ============================================================================
// Entity State Storage (Structure of Arrays)
// ============================================================================

/**
 * @brief SoA storage for entity physics states
 *
 * This is the core data structure optimized for cache efficiency
 * and SIMD vectorization. All arrays are kept in sync by index.
 */
class EntityStateStorage {
public:
    static constexpr SizeT ALIGNMENT = 64;  // Cache line alignment

    EntityStateStorage() = default;
    ~EntityStateStorage() = default;

    // Non-copyable, movable
    EntityStateStorage(const EntityStateStorage&) = delete;
    EntityStateStorage& operator=(const EntityStateStorage&) = delete;
    EntityStateStorage(EntityStateStorage&&) noexcept = default;
    EntityStateStorage& operator=(EntityStateStorage&&) noexcept = default;

    /**
     * @brief Reserve storage for expected number of entities
     */
    void reserve(SizeT capacity);

    /**
     * @brief Allocate storage for a new entity
     * @return Index into arrays for the new entity
     */
    UInt32 allocate();

    /**
     * @brief Free storage at given index (mark for reuse)
     */
    void free(UInt32 index);

    /**
     * @brief Get total number of allocated slots
     */
    SizeT size() const noexcept { return positions_.size(); }

    /**
     * @brief Get number of active (non-free) entities
     */
    SizeT active_count() const noexcept { return active_count_; }

    // ========================================================================
    // State Access (by index)
    // ========================================================================

    // Position
    Vec3& position(UInt32 idx) { return positions_[idx]; }
    const Vec3& position(UInt32 idx) const { return positions_[idx]; }
    Vec3* positions_data() { return positions_.data(); }
    const Vec3* positions_data() const { return positions_.data(); }

    // Velocity
    Vec3& velocity(UInt32 idx) { return velocities_[idx]; }
    const Vec3& velocity(UInt32 idx) const { return velocities_[idx]; }
    Vec3* velocities_data() { return velocities_.data(); }
    const Vec3* velocities_data() const { return velocities_.data(); }

    // Orientation
    Quat& orientation(UInt32 idx) { return orientations_[idx]; }
    const Quat& orientation(UInt32 idx) const { return orientations_[idx]; }
    Quat* orientations_data() { return orientations_.data(); }
    const Quat* orientations_data() const { return orientations_.data(); }

    // Angular velocity
    Vec3& angular_velocity(UInt32 idx) { return angular_velocities_[idx]; }
    const Vec3& angular_velocity(UInt32 idx) const { return angular_velocities_[idx]; }

    // Acceleration
    Vec3& acceleration(UInt32 idx) { return accelerations_[idx]; }
    const Vec3& acceleration(UInt32 idx) const { return accelerations_[idx]; }

    // Angular acceleration
    Vec3& angular_acceleration(UInt32 idx) { return angular_accelerations_[idx]; }
    const Vec3& angular_acceleration(UInt32 idx) const { return angular_accelerations_[idx]; }

    // Mass
    Real& mass(UInt32 idx) { return masses_[idx]; }
    Real mass(UInt32 idx) const { return masses_[idx]; }

    // Inertia
    Mat3x3& inertia(UInt32 idx) { return inertias_[idx]; }
    const Mat3x3& inertia(UInt32 idx) const { return inertias_[idx]; }

    // Forces
    EntityForces& forces(UInt32 idx) { return forces_[idx]; }
    const EntityForces& forces(UInt32 idx) const { return forces_[idx]; }

    // Active flag
    bool is_active(UInt32 idx) const { return active_flags_[idx]; }
    void set_active(UInt32 idx, bool active);

    // ========================================================================
    // Bulk Operations
    // ========================================================================

    /**
     * @brief Copy complete state from AoS struct to SoA storage
     */
    void set_state(UInt32 idx, const EntityState& state);

    /**
     * @brief Copy complete state from SoA storage to AoS struct
     */
    EntityState get_state(UInt32 idx) const;

    /**
     * @brief Clear all accumulated forces
     */
    void clear_forces();

    // ========================================================================
    // SIMD Batch Operations
    // ========================================================================

    /**
     * @brief Batch update positions: p = p + v * dt
     *
     * Updates all active entities using SIMD-optimized operations.
     * @param dt Time step in seconds
     */
    void batch_integrate_positions(Real dt);

    /**
     * @brief Batch update velocities: v = v + a * dt
     *
     * Updates all active entities using SIMD-optimized operations.
     * @param dt Time step in seconds
     */
    void batch_integrate_velocities(Real dt);

    /**
     * @brief Batch compute accelerations from forces: a = F / m
     *
     * Converts accumulated forces to accelerations for all entities.
     */
    void batch_compute_accelerations();

    /**
     * @brief Batch apply gravity to all entities
     *
     * Adds gravitational force F_z += m * g to all active entities.
     * @param g Gravitational acceleration magnitude (positive value)
     */
    void batch_apply_gravity(Real g);

private:
    // State arrays (aligned for SIMD)
    std::vector<Vec3> positions_;
    std::vector<Vec3> velocities_;
    std::vector<Quat> orientations_;
    std::vector<Vec3> angular_velocities_;
    std::vector<Vec3> accelerations_;
    std::vector<Vec3> angular_accelerations_;
    std::vector<Real> masses_;
    std::vector<Mat3x3> inertias_;
    std::vector<EntityForces> forces_;
    std::vector<bool> active_flags_;

    // Free list for recycling indices
    std::vector<UInt32> free_indices_;
    SizeT active_count_{0};
};

// ============================================================================
// Entity Manager
// ============================================================================

/**
 * @brief Manages entity lifecycle and provides access to entity data
 */
class EntityManager {
public:
    EntityManager();
    ~EntityManager();

    /**
     * @brief Create a new entity
     * @param name Entity name
     * @param domain Primary physics domain
     * @return EntityId of the created entity
     */
    EntityId create_entity(const std::string& name, Domain domain = Domain::Generic);

    /**
     * @brief Destroy an entity
     * @param id Entity to destroy
     */
    void destroy_entity(EntityId id);

    /**
     * @brief Check if entity exists and is valid
     */
    bool exists(EntityId id) const;

    /**
     * @brief Get entity by ID
     * @return Pointer to entity, or nullptr if not found
     */
    Entity* get_entity(EntityId id);
    const Entity* get_entity(EntityId id) const;

    /**
     * @brief Get entity state (copy)
     */
    EntityState get_state(EntityId id) const;
    void set_state(EntityId id, const EntityState& state);

    /**
     * @brief Get mutable pointer to entity state for constraint solving
     *
     * This returns a pointer to a temporary EntityState that is valid only
     * during the current physics step. The state is synchronized with
     * the SoA storage before and after constraint solving.
     *
     * @return Pointer to EntityState, or nullptr if entity not found
     */
    EntityState* get_entity_state(EntityId id);

    /**
     * @brief Sync constraint solver state back to SoA storage
     */
    void sync_entity_states();

    /**
     * @brief Access state storage directly (for batch processing)
     */
    EntityStateStorage& get_state_storage() { return state_storage_; }
    const EntityStateStorage& get_state_storage() const { return state_storage_; }

    /**
     * @brief Iterate over all active entities
     */
    template<typename Func>
    void for_each(Func&& func) {
        for (auto& [id, entity] : entities_) {
            if (entity.active) {
                func(entity);
            }
        }
    }

    /**
     * @brief Iterate over entities with specific components
     */
    template<typename Func>
    void for_each_with(ComponentMask required_components, Func&& func) {
        for (auto& [id, entity] : entities_) {
            if (entity.active && (entity.components & required_components) == required_components) {
                func(entity);
            }
        }
    }

    /**
     * @brief Get total number of entities
     */
    SizeT entity_count() const { return entities_.size(); }

    /**
     * @brief Get number of active entities
     */
    SizeT active_entity_count() const;

    // ========================================================================
    // Event System Integration
    // ========================================================================

    /**
     * @brief Set event dispatcher for entity lifecycle events
     *
     * When set, the EntityManager will emit events on:
     * - EntityCreated: When create_entity() is called
     * - EntityDestroyed: When destroy_entity() is called
     * - EntityActivated/Deactivated: When entity active state changes
     *
     * @param dispatcher Pointer to event dispatcher (can be null to disable)
     */
    void set_event_dispatcher(events::EventDispatcher* dispatcher);

    /**
     * @brief Get current event dispatcher
     */
    events::EventDispatcher* get_event_dispatcher() const { return event_dispatcher_; }

    /**
     * @brief Set current simulation time for event timestamps
     */
    void set_current_time(Real time) { current_time_ = time; }

private:
    EntityId next_id_{0};
    std::unordered_map<EntityId, Entity> entities_;
    EntityStateStorage state_storage_;

    // Temporary state cache for constraint solving (maps entity ID to EntityState)
    std::unordered_map<EntityId, EntityState> constraint_state_cache_;

    // Event system integration
    events::EventDispatcher* event_dispatcher_{nullptr};
    Real current_time_{0.0};
};

} // namespace jaguar::physics
