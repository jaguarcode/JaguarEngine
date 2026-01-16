/**
 * @file suspension.cpp
 * @brief Suspension model implementation
 *
 * Implements a spring-damper suspension system for ground vehicles.
 *
 * Features:
 * - Multiple suspension units with independent characteristics
 * - Spring-damper force calculation with preload and travel limits
 * - Bump stop modeling at travel extremes
 * - Total force and torque contribution to vehicle dynamics
 * - Tracked vehicle model with sprocket-driven tracks
 *
 * Reference: Vehicle Dynamics Theory and Application (Jazar)
 */

#include "jaguar/domain/land.h"
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
// SuspensionModel Implementation
// ============================================================================

SuspensionModel::SuspensionModel() = default;
SuspensionModel::~SuspensionModel() = default;

void SuspensionModel::add_unit(const Vec3& position, const SuspensionUnit& unit) {
    units_.push_back({position, unit});
}

void SuspensionModel::update(const physics::EntityState& state, [[maybe_unused]] Real dt) {
    // Update each suspension unit based on vehicle motion
    // This is a simplified model - a full model would:
    // 1. Transform wheel positions to world frame
    // 2. Query terrain height at each wheel
    // 3. Calculate compression based on terrain contact
    // 4. Integrate wheel vertical dynamics

    for (auto& unit_data : units_) {
        // Simplified: use body z-velocity as compression rate estimate
        // A proper model would track individual wheel contacts
        Vec3 v_body = state.orientation.conjugate().rotate(state.velocity);

        // Estimate wheel vertical velocity relative to body
        // This includes contribution from angular motion
        Vec3 r = unit_data.position;
        Vec3 omega = state.angular_velocity;
        Vec3 v_rel = omega.cross(r);  // Velocity due to rotation

        // Total vertical velocity at this suspension point
        Real v_vertical = v_body.z + v_rel.z;

        // Update suspension state
        SuspensionUnit& unit = unit_data.unit;

        // Simple integration (in reality, this would come from terrain contact)
        // For now, assume some ground contact and integrate based on force
        // This is a placeholder - proper implementation needs terrain queries

        // Update velocity
        unit.current_velocity = v_vertical;

        // Clamp position to travel limits
        unit.current_position = std::clamp(unit.current_position,
                                           unit.travel_min, unit.travel_max);
    }
}

Vec3 SuspensionModel::get_total_force() const {
    Vec3 total{0.0, 0.0, 0.0};

    for (const auto& unit_data : units_) {
        // Suspension force acts in local vertical direction (Z in body frame)
        Real force = unit_data.unit.calculate_force();
        total.z += force;
    }

    return total;
}

Vec3 SuspensionModel::get_total_torque() const {
    Vec3 total{0.0, 0.0, 0.0};

    for (const auto& unit_data : units_) {
        // Force acts at suspension position
        Real force = unit_data.unit.calculate_force();
        Vec3 force_vec{0.0, 0.0, force};

        // Torque = r × F
        Vec3 torque = unit_data.position.cross(force_vec);
        total += torque;
    }

    return total;
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
