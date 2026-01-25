/**
 * @file suspension.cpp
 * @brief Suspension model implementation with terrain integration
 *
 * Implements a spring-damper suspension system for ground vehicles with
 * full terrain integration for realistic ground contact.
 *
 * Features:
 * - Multiple suspension units with independent characteristics
 * - Spring-damper force calculation with preload and travel limits
 * - Bump stop modeling at travel extremes
 * - Terrain-integrated contact calculation
 * - Surface normal-based force direction
 * - Material-specific friction coefficients
 * - Total force and torque contribution to vehicle dynamics
 * - Tracked vehicle model with sprocket-driven tracks
 *
 * Reference: Vehicle Dynamics Theory and Application (Jazar)
 */

#include "jaguar/domain/land.h"
#include "jaguar/domain/land/suspension.h"
#include "jaguar/environment/environment.h"
#include "jaguar/environment/terrain.h"
#include <cmath>
#include <algorithm>

namespace jaguar::domain::land {

// ============================================================================
// SuspensionUnit Implementation
// ============================================================================

Real SuspensionUnit::calculate_force() const {
    // Basic spring-damper model:
    // F = k * x + c * v + preload
    //
    // Where:
    // k = spring stiffness (N/m)
    // c = damping coefficient (N·s/m)
    // x = current compression (positive = compressed)
    // v = compression rate (positive = compressing)

    // Spring force (resists compression)
    Real spring_force = spring_k * current_position;

    // Damping force (resists motion)
    // Note: Asymmetric damping is common in real suspensions
    // (more damping in rebound than compression)
    Real damping_force = damper_c * current_velocity;

    // Total force with preload
    Real total_force = spring_force + damping_force + preload;

    // Add bump stop forces at travel limits
    // Model bump stops as progressive springs
    constexpr Real BUMP_STOP_STIFFNESS = 100000.0;  // N/m, very stiff
    constexpr Real BUMP_ZONE = 0.02;  // 2cm bump stop engagement zone

    // Check lower bump stop (full extension)
    if (current_position < travel_min + BUMP_ZONE) {
        Real penetration = (travel_min + BUMP_ZONE) - current_position;
        // Quadratic bump stop characteristic
        total_force -= BUMP_STOP_STIFFNESS * penetration * penetration / BUMP_ZONE;
    }

    // Check upper bump stop (full compression)
    if (current_position > travel_max - BUMP_ZONE) {
        Real penetration = current_position - (travel_max - BUMP_ZONE);
        // Quadratic bump stop characteristic
        total_force += BUMP_STOP_STIFFNESS * penetration * penetration / BUMP_ZONE;
    }

    return total_force;
}

// ============================================================================
// WheelSuspension Implementation (Terrain-Integrated)
// ============================================================================

WheelSuspension::WheelSuspension(const Vec3& position, Real wheel_radius, const SuspensionUnit& unit)
    : position_(position), wheel_radius_(wheel_radius), unit_(unit) {}

environment::TerrainQuery WheelSuspension::query_terrain_at_wheel(
    const physics::EntityState& state,
    const environment::EnvironmentService* env_service) const
{
    // Transform wheel position to world frame
    Vec3 wheel_position_world = state.position + state.orientation.rotate(position_);

    // Query terrain if environment service is available
    if (env_service) {
        // Note: EnvironmentService::query expects ECEF, we're assuming wheel_position_world is ECEF
        // In a full implementation, proper coordinate transforms would be needed
        environment::Environment env = env_service->query(wheel_position_world, 0.0);
        return env.terrain;
    }

    // Fallback: flat terrain at origin
    environment::TerrainQuery default_terrain;
    default_terrain.elevation = 0.0;
    default_terrain.normal = Vec3{0, 0, 1};
    default_terrain.slope_angle = 0.0;
    default_terrain.material.friction_coefficient = 0.7;
    default_terrain.material.rolling_resistance = 0.01;
    default_terrain.valid = true;
    return default_terrain;
}

Real WheelSuspension::compute_penetration(
    const Vec3& wheel_position_world,
    const environment::TerrainQuery& terrain_query) const
{
    // Compute penetration depth relative to terrain surface
    // Penetration is measured along terrain normal

    // Terrain contact point (on surface)
    Vec3 terrain_point = wheel_position_world;
    terrain_point.z = terrain_query.elevation;

    // Vector from terrain to wheel center
    Vec3 separation = wheel_position_world - terrain_point;

    // Project onto terrain normal
    Real distance_to_surface = separation.dot(terrain_query.normal);

    // Penetration is wheel_radius minus distance to surface
    // Positive = wheel is penetrating ground
    // Negative = wheel is above ground
    Real penetration = wheel_radius_ - distance_to_surface;

    return penetration;
}

Vec3 WheelSuspension::compute_forces(
    const physics::EntityState& state,
    const environment::EnvironmentService* env_service,
    Real dt)
{
    // 1. Query terrain at wheel position
    environment::TerrainQuery terrain = query_terrain_at_wheel(state, env_service);

    // Cache terrain properties
    last_terrain_normal_ = terrain.normal;
    friction_coefficient_ = terrain.material.friction_coefficient;

    // 2. Transform wheel position to world frame
    Vec3 wheel_position_world = state.position + state.orientation.rotate(position_);

    // 3. Compute penetration relative to terrain surface
    Real penetration = compute_penetration(wheel_position_world, terrain);

    // 4. Update suspension state
    is_grounded_ = penetration > 0.0;

    if (!is_grounded_) {
        // Wheel is airborne - no force
        unit_.current_position = 0.0;
        unit_.current_velocity = 0.0;
        last_force_magnitude_ = 0.0;
        last_force_ = Vec3{0, 0, 0};
        return Vec3{0, 0, 0};
    }

    // 5. Compute suspension compression
    // Penetration indicates how much the spring is compressed
    Real compression = penetration;
    unit_.current_position = std::clamp(compression, unit_.travel_min, unit_.travel_max);

    // 6. Compute compression velocity
    // This requires tracking previous state or computing from entity motion
    Vec3 wheel_velocity_world = state.velocity + state.angular_velocity.cross(
        state.orientation.rotate(position_));

    // Project velocity onto terrain normal (positive = compressing)
    unit_.current_velocity = -wheel_velocity_world.dot(terrain.normal);

    // 7. Calculate suspension force magnitude
    Real force_magnitude = unit_.calculate_force();
    last_force_magnitude_ = force_magnitude;

    // 8. Apply force along terrain normal (not vertical!)
    // Force opposes penetration, so it acts along positive normal
    last_force_ = terrain.normal * force_magnitude;

    return last_force_;
}

Vec3 WheelSuspension::compute_torque(const physics::EntityState& state) const {
    if (!is_grounded_) {
        return Vec3{0, 0, 0};
    }

    // Torque = r × F
    // r is in body frame, F is in world frame
    // Need to transform r to world frame
    Vec3 r_world = state.orientation.rotate(position_);
    Vec3 torque = r_world.cross(last_force_);

    return torque;
}

// ============================================================================
// SuspensionModel Implementation
// ============================================================================

SuspensionModel::SuspensionModel() = default;
SuspensionModel::~SuspensionModel() = default;

void SuspensionModel::add_wheel(const Vec3& position, Real wheel_radius, const SuspensionUnit& unit) {
    wheels_.emplace_back(position, wheel_radius, unit);
}

void SuspensionModel::update(
    const physics::EntityState& state,
    const environment::EnvironmentService* env_service,
    Real dt)
{
    total_force_ = Vec3{0, 0, 0};
    total_torque_ = Vec3{0, 0, 0};

    for (auto& wheel : wheels_) {
        Vec3 force = wheel.compute_forces(state, env_service, dt);
        Vec3 torque = wheel.compute_torque(state);

        total_force_ += force;
        total_torque_ += torque;
    }
}

Vec3 SuspensionModel::get_total_force() const {
    return total_force_;
}

Vec3 SuspensionModel::get_total_torque() const {
    return total_torque_;
}

SizeT SuspensionModel::grounded_wheel_count() const {
    SizeT count = 0;
    for (const auto& wheel : wheels_) {
        if (wheel.is_grounded()) {
            ++count;
        }
    }
    return count;
}

// ============================================================================
// TrackedVehicleModel Implementation
// ============================================================================

TrackedVehicleModel::TrackedVehicleModel() = default;
TrackedVehicleModel::~TrackedVehicleModel() = default;

void TrackedVehicleModel::set_sprocket(Real radius, Real max_torque) {
    sprocket_radius_ = std::max(radius, 0.1);
    max_torque_ = std::max(max_torque, 0.0);
}

void TrackedVehicleModel::update(
    Real drive_torque,
    Real load,
    const SoilProperties& soil,
    Real dt)
{
    // Simplified tracked vehicle model
    // Assumes both tracks receive equal torque (differential steering not modeled)

    // Clamp drive torque to maximum
    Real torque = std::clamp(drive_torque, -max_torque_, max_torque_);

    // Force at sprocket circumference
    Real drive_force = torque / sprocket_radius_;

    // Track ground contact area (assumed values)
    constexpr Real TRACK_WIDTH = 0.5;   // m
    constexpr Real TRACK_LENGTH = 4.0;  // m
    Real contact_area = TRACK_WIDTH * TRACK_LENGTH;

    // Ground pressure
    Real ground_pressure = load / (2.0 * contact_area);  // Per track

    // Maximum traction from Mohr-Coulomb (per track)
    Real max_shear = soil.c * 1000.0 + ground_pressure * std::tan(soil.phi);
    Real max_traction = contact_area * max_shear;

    // Determine slip based on force demand vs available traction
    // Simplified slip model
    Real force_demand = std::abs(drive_force) / 2.0;  // Per track

    Real slip = 0.0;
    if (max_traction > 0.0) {
        // Slip increases as demand approaches max traction
        Real traction_ratio = force_demand / max_traction;
        if (traction_ratio < 0.8) {
            // Linear slip region
            slip = 0.1 * traction_ratio;
        } else {
            // High slip region
            slip = 0.08 + 0.5 * (traction_ratio - 0.8);
        }
        slip = std::min(slip, 1.0);
    }

    // Update track states (simplified - both tracks identical for now)
    left_track_.slip = slip;
    right_track_.slip = slip;

    // Track velocity from sprocket angular velocity
    // Assuming steady state: torque / (r * effective_force_coeff) ~ angular_vel
    // Simplified: just track nominal velocity (used for slip calculation)

    // Actual velocity considering slip
    // v_actual = v_track * (1 - slip) for driving
    // For now, just update based on simple dynamics
    Real acceleration = (drive_force - slip * max_traction * 2.0) / load;
    left_track_.velocity += acceleration * dt;
    right_track_.velocity += acceleration * dt;

    // Prevent negative velocity (no reverse without reverse torque)
    if (torque >= 0 && left_track_.velocity < 0) {
        left_track_.velocity = 0;
        right_track_.velocity = 0;
    }

    // Track tension (simplified - constant for now)
    // A proper model would include track dynamics and tension variation
    constexpr Real BASE_TENSION = 10000.0;  // N
    left_track_.tension = BASE_TENSION;
    right_track_.tension = BASE_TENSION;
}

Real TrackedVehicleModel::get_propulsive_force() const {
    // Total force from both tracks
    // Simplified: use track velocity and assume some efficiency
    constexpr Real EFFICIENCY = 0.85;

    // Force = tension difference around sprocket (simplified)
    // For a more complete model, integrate tractive effort along track contact
    Real left_force = left_track_.tension * (1.0 - left_track_.slip) * EFFICIENCY;
    Real right_force = right_track_.tension * (1.0 - right_track_.slip) * EFFICIENCY;

    return left_force + right_force;
}

} // namespace jaguar::domain::land
