#pragma once
/**
 * @file suspension.h
 * @brief Terrain-integrated suspension models
 *
 * Provides suspension systems that integrate with the terrain system
 * for realistic ground contact and force calculation.
 */

#include "jaguar/core/types.h"
#include "jaguar/physics/entity.h"

namespace jaguar::environment {
    struct TerrainQuery;
    class EnvironmentService;
}

namespace jaguar::domain::land {

// ============================================================================
// Suspension Unit
// ============================================================================

/**
 * @brief Single suspension unit (spring-damper)
 */
struct SuspensionUnit {
    Real spring_k{50000.0};     ///< Spring stiffness (N/m)
    Real damper_c{5000.0};      ///< Damping coefficient (N·s/m)
    Real preload{0.0};          ///< Preload force (N)
    Real travel_max{0.3};       ///< Maximum travel (m)
    Real travel_min{0.0};       ///< Minimum travel (m)

    // State
    Real current_position{0.0}; ///< Current compression (m)
    Real current_velocity{0.0}; ///< Compression rate (m/s)

    /**
     * @brief Calculate suspension force
     */
    Real calculate_force() const;
};

// ============================================================================
// Wheel Suspension (Terrain-Integrated)
// ============================================================================

/**
 * @brief Wheel suspension with terrain integration
 *
 * Integrates with EnvironmentService to query terrain height, normal,
 * and material properties for realistic suspension and contact forces.
 *
 * Features:
 * - Terrain-aware contact calculation
 * - Surface normal-based force direction
 * - Material-specific friction coefficients
 * - Slope and banking support
 * - Thread-safe terrain queries
 * - Graceful fallback to flat ground if no terrain available
 */
class WheelSuspension {
public:
    /**
     * @brief Create wheel suspension at body-relative position
     * @param position Position in body frame (m)
     * @param wheel_radius Wheel radius (m)
     * @param unit Suspension unit parameters
     */
    WheelSuspension(const Vec3& position, Real wheel_radius, const SuspensionUnit& unit);

    /**
     * @brief Compute suspension and contact forces
     * @param state Entity state (position, orientation, velocity)
     * @param env_service Environment service for terrain queries (can be nullptr)
     * @param dt Time step (s)
     * @return Force in world frame
     */
    Vec3 compute_forces(const physics::EntityState& state,
                       const environment::EnvironmentService* env_service,
                       Real dt);

    /**
     * @brief Get torque contribution about body center of mass
     * @param state Entity state
     * @return Torque in world frame
     */
    Vec3 compute_torque(const physics::EntityState& state) const;

    /**
     * @brief Get current compression state
     */
    Real get_compression() const { return unit_.current_position; }

    /**
     * @brief Get current contact force magnitude
     */
    Real get_contact_force() const { return last_force_magnitude_; }

    /**
     * @brief Check if wheel is in contact with ground
     */
    bool is_grounded() const { return is_grounded_; }

    /**
     * @brief Get friction coefficient from last terrain query
     */
    Real get_friction_coefficient() const { return friction_coefficient_; }

    /**
     * @brief Get position in body frame
     */
    const Vec3& position() const { return position_; }

    /**
     * @brief Get suspension unit
     */
    const SuspensionUnit& unit() const { return unit_; }
    SuspensionUnit& unit() { return unit_; }

private:
    Vec3 position_;              ///< Position in body frame
    Real wheel_radius_;          ///< Wheel radius (m)
    SuspensionUnit unit_;

    // Cached state from last computation
    bool is_grounded_{false};
    Real last_force_magnitude_{0.0};
    Real friction_coefficient_{0.7};
    Vec3 last_terrain_normal_{0, 0, 1};
    Vec3 last_force_{0, 0, 0};

    /**
     * @brief Query terrain at wheel position
     * @return TerrainQuery with elevation and normal, or default flat terrain
     */
    environment::TerrainQuery query_terrain_at_wheel(
        const physics::EntityState& state,
        const environment::EnvironmentService* env_service) const;

    /**
     * @brief Compute penetration depth into terrain
     * @param wheel_position_world Wheel center in world frame
     * @param terrain_query Terrain data at wheel location
     * @return Penetration depth (positive = penetrating, negative = airborne)
     */
    Real compute_penetration(const Vec3& wheel_position_world,
                           const environment::TerrainQuery& terrain_query) const;
};

// ============================================================================
// Multi-Wheel Suspension Model
// ============================================================================

/**
 * @brief Vehicle suspension system with multiple wheels
 *
 * Manages multiple WheelSuspension instances and computes total
 * forces and torques for vehicle dynamics.
 */
class SuspensionModel {
public:
    SuspensionModel();
    ~SuspensionModel();

    /**
     * @brief Add a wheel suspension unit
     */
    void add_wheel(const Vec3& position, Real wheel_radius, const SuspensionUnit& unit);

    /**
     * @brief Update all suspension units and compute forces
     */
    void update(const physics::EntityState& state,
               const environment::EnvironmentService* env_service,
               Real dt);

    /**
     * @brief Get total force from all suspension units (world frame)
     */
    Vec3 get_total_force() const;

    /**
     * @brief Get total torque from all suspension units (world frame)
     */
    Vec3 get_total_torque() const;

    /**
     * @brief Get number of wheels
     */
    SizeT wheel_count() const { return wheels_.size(); }

    /**
     * @brief Get wheel suspension by index
     */
    const WheelSuspension& wheel(SizeT index) const { return wheels_[index]; }
    WheelSuspension& wheel(SizeT index) { return wheels_[index]; }

    /**
     * @brief Count grounded wheels
     */
    SizeT grounded_wheel_count() const;

private:
    std::vector<WheelSuspension> wheels_;
    Vec3 total_force_{0, 0, 0};
    Vec3 total_torque_{0, 0, 0};
};

} // namespace jaguar::domain::land
