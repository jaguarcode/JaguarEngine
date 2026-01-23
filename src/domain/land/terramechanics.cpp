/**
 * @file terramechanics.cpp
 * @brief Terramechanics (Bekker-Wong) implementation
 *
 * Implements pressure-sinkage relationships and traction mechanics
 * using the Bekker-Wong terramechanics theory.
 *
 * Key equations:
 * - Bekker pressure-sinkage: p = (k_c/b + k_phi) * z^n
 * - Motion resistance: R_c = b * integral(p * dz) from 0 to z
 * - Maximum traction (Mohr-Coulomb): T_max = A * (c + p * tan(phi))
 *
 * Reference: Theory of Ground Vehicles (Wong), Terramechanics and Off-Road Vehicles (Bekker)
 */

#include "jaguar/domain/land.h"
#include "jaguar/environment/environment.h"
#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace jaguar::domain::land {

// ============================================================================
// Soil Type Presets (Bekker-Wong parameters from experimental data)
// ============================================================================

SoilProperties SoilProperties::DrySand() {
    // Dry sand: Low cohesion, high friction
    // Values from Wong, "Theory of Ground Vehicles"
    return {
        0.99,           // k_c: cohesive modulus (kN/m^(n+1))
        1528.0,         // k_phi: frictional modulus (kN/m^(n+2))
        1.10,           // n: deformation exponent
        1.04,           // c: cohesion (kPa)
        28.0 * M_PI / 180.0  // phi: internal friction angle (rad)
    };
}

SoilProperties SoilProperties::WetSand() {
    // Wet sand: Slightly higher cohesion than dry
    return {
        5.27,           // k_c
        1515.0,         // k_phi
        0.73,           // n
        1.72,           // c
        29.0 * M_PI / 180.0  // phi
    };
}

SoilProperties SoilProperties::Clay() {
    // Clay: High cohesion, lower friction
    return {
        13.19,          // k_c
        692.0,          // k_phi
        0.50,           // n
        4.14,           // c
        13.0 * M_PI / 180.0  // phi
    };
}

SoilProperties SoilProperties::Snow() {
    // Snow: Moderate values, high compressibility
    return {
        4.37,           // k_c
        196.0,          // k_phi
        1.60,           // n
        1.03,           // c
        19.7 * M_PI / 180.0  // phi
    };
}

SoilProperties SoilProperties::Asphalt() {
    // Rigid surface: Very high values (near-zero sinkage)
    return {
        1000000.0,      // k_c
        1000000.0,      // k_phi
        0.0,            // n (doesn't matter for rigid surface)
        1000.0,         // c
        45.0 * M_PI / 180.0  // phi
    };
}

// ============================================================================
// TerramechanicsModel Implementation
// ============================================================================

namespace {

/**
 * @brief Convert terrain material to soil properties
 *
 * Maps the TerrainMaterial from the terrain system to SoilProperties
 * for terramechanics calculations.
 */
SoilProperties terrain_material_to_soil(const environment::TerrainMaterial& material) {
    SoilProperties soil;

    // Use Bekker-Wong parameters from terrain material if available
    // TerrainMaterial k_c/k_phi are in same units as SoilProperties
    soil.k_c = material.k_c;
    soil.k_phi = material.k_phi;
    soil.n = material.n;

    // Derive cohesion and friction angle from surface type if not specified
    // These are typical values based on surface type
    switch (material.type) {
        case environment::SurfaceType::Asphalt:
        case environment::SurfaceType::Concrete:
            soil.c = 1000.0;  // kPa - rigid surface
            soil.phi = 45.0 * M_PI / 180.0;
            // Override Bekker params for rigid surfaces
            if (soil.k_c < 1000.0) {
                soil.k_c = 1000000.0;
                soil.k_phi = 1000000.0;
                soil.n = 0.0;
            }
            break;

        case environment::SurfaceType::Gravel:
            soil.c = 0.5;
            soil.phi = 35.0 * M_PI / 180.0;
            if (soil.k_phi < 100.0) {
                soil.k_c = 3.0;
                soil.k_phi = 3000.0;
                soil.n = 0.90;
            }
            break;

        case environment::SurfaceType::DrySand:
            soil.c = 1.04;
            soil.phi = 28.0 * M_PI / 180.0;
            if (soil.k_phi < 100.0) {
                soil.k_c = 0.99;
                soil.k_phi = 1528.0;
                soil.n = 1.10;
            }
            break;

        case environment::SurfaceType::WetSand:
            soil.c = 1.72;
            soil.phi = 29.0 * M_PI / 180.0;
            if (soil.k_phi < 100.0) {
                soil.k_c = 5.27;
                soil.k_phi = 1515.0;
                soil.n = 0.73;
            }
            break;

        case environment::SurfaceType::Clay:
            soil.c = 4.14;
            soil.phi = 13.0 * M_PI / 180.0;
            if (soil.k_phi < 100.0) {
                soil.k_c = 13.19;
                soil.k_phi = 692.0;
                soil.n = 0.50;
            }
            break;

        case environment::SurfaceType::Mud:
            soil.c = 2.0;
            soil.phi = 5.0 * M_PI / 180.0;
            if (soil.k_phi < 10.0) {
                soil.k_c = 2.0;
                soil.k_phi = 100.0;
                soil.n = 0.80;
            }
            break;

        case environment::SurfaceType::Grass:
            // Grass over soil - moderate parameters
            soil.c = 3.0;
            soil.phi = 25.0 * M_PI / 180.0;
            if (soil.k_phi < 100.0) {
                soil.k_c = 5.0;
                soil.k_phi = 800.0;
                soil.n = 0.80;
            }
            break;

        case environment::SurfaceType::Snow:
            soil.c = 1.03;
            soil.phi = 19.7 * M_PI / 180.0;
            if (soil.k_phi < 100.0) {
                soil.k_c = 4.37;
                soil.k_phi = 196.0;
                soil.n = 1.60;
            }
            break;

        case environment::SurfaceType::Ice:
            // Ice is rigid but low friction handled elsewhere
            soil.c = 500.0;
            soil.phi = 10.0 * M_PI / 180.0;
            if (soil.k_c < 1000.0) {
                soil.k_c = 500000.0;
                soil.k_phi = 500000.0;
                soil.n = 0.0;
            }
            break;

        case environment::SurfaceType::Water:
            // Water - no ground contact
            soil.c = 0.0;
            soil.phi = 0.0;
            soil.k_c = 0.0;
            soil.k_phi = 0.0;
            soil.n = 1.0;
            break;

        case environment::SurfaceType::Unknown:
        default:
            // Default to dry sand
            return SoilProperties::DrySand();
    }

    return soil;
}

/**
 * @brief Get soil properties from terrain environment
 *
 * Queries the terrain material at the entity's location and converts
 * it to terramechanics parameters.
 */
SoilProperties get_terrain_soil(const environment::Environment& env) {
    // Check if terrain query is valid
    if (!env.terrain.valid) {
        // No terrain data available, use default
        return SoilProperties::DrySand();
    }

    // Convert terrain material to soil properties
    return terrain_material_to_soil(env.terrain.material);
}

/**
 * @brief Compute slip ratio from wheel/track velocity vs ground velocity
 *
 * Slip ratio definition:
 * - Driving: i = (V_wheel - V_ground) / V_wheel
 * - Braking: i = (V_wheel - V_ground) / V_ground
 *
 * @param wheel_velocity Linear velocity at wheel/track circumference (m/s)
 * @param ground_velocity Actual ground velocity (m/s)
 * @return Slip ratio (-1 to 1, negative = braking)
 */
Real compute_slip_ratio(Real wheel_velocity, Real ground_velocity) {
    constexpr Real MIN_VELOCITY = 0.01;  // m/s, avoid division by zero

    // Both velocities near zero
    if (std::abs(wheel_velocity) < MIN_VELOCITY && std::abs(ground_velocity) < MIN_VELOCITY) {
        return 0.0;
    }

    // Driving condition (wheel faster than ground)
    if (wheel_velocity > ground_velocity) {
        if (std::abs(wheel_velocity) < MIN_VELOCITY) {
            return 0.0;
        }
        return (wheel_velocity - ground_velocity) / wheel_velocity;
    }

    // Braking condition (ground faster than wheel)
    if (std::abs(ground_velocity) < MIN_VELOCITY) {
        return 0.0;
    }
    return (wheel_velocity - ground_velocity) / ground_velocity;
}

/**
 * @brief Compute traction force using slip-dependent model
 *
 * The traction develops with slip according to an exponential relationship.
 * At zero slip, no traction. At optimal slip (~15-20%), maximum traction.
 *
 * @param max_traction Maximum available traction from Mohr-Coulomb (N)
 * @param slip_ratio Current slip ratio
 * @return Actual traction force (N)
 */
Real compute_slip_traction(Real max_traction, Real slip_ratio) {
    // Exponential slip model: T = T_max * (1 - exp(-k * |i|))
    // k is a soil shear deformation parameter
    constexpr Real K_SHEAR = 10.0;  // Typical value for many soils

    Real abs_slip = std::abs(slip_ratio);
    Real traction_factor = 1.0 - std::exp(-K_SHEAR * abs_slip);

    // At very high slip, traction may decrease (spinning)
    // Apply reduction factor above 0.5 slip
    if (abs_slip > 0.5) {
        Real overshoot = (abs_slip - 0.5) / 0.5;  // 0 at 0.5, 1 at 1.0
        traction_factor *= (1.0 - 0.3 * overshoot);  // Up to 30% reduction
    }

    Real traction = max_traction * traction_factor;

    // Apply sign based on slip direction
    return (slip_ratio >= 0.0) ? traction : -traction;
}

} // anonymous namespace

TerramechanicsModel::TerramechanicsModel() = default;
TerramechanicsModel::~TerramechanicsModel() = default;

void TerramechanicsModel::compute_forces(
    const physics::EntityState& state,
    const environment::Environment& env,
    [[maybe_unused]] Real dt,
    physics::EntityForces& out_forces)
{
    if (!enabled_) {
        sinkage_ = 0.0;
        motion_resistance_ = 0.0;
        traction_ = 0.0;
        slip_ratio_ = 0.0;
        return;
    }

    // Get terrain soil properties
    SoilProperties soil = get_terrain_soil(env);

    // Check if vehicle is on ground
    // Use altitude above terrain (negative means below ground level)
    Real height_above_terrain = env.altitude - env.terrain_elevation;

    // If above terrain, no ground forces
    if (height_above_terrain > 0.1) {  // 10cm tolerance
        sinkage_ = 0.0;
        motion_resistance_ = 0.0;
        traction_ = 0.0;
        slip_ratio_ = 0.0;
        return;
    }

    // Calculate contact area
    Real contact_area = contact_width_ * contact_length_;

    // Calculate nominal ground pressure
    Real ground_pressure = vehicle_weight_ / contact_area;  // Pa

    // Convert to kPa for Bekker equation (soil parameters are in kN/m units)
    Real pressure_kpa = ground_pressure / 1000.0;

    // Calculate sinkage using Bekker equation
    sinkage_ = calculate_sinkage(soil, pressure_kpa);

    // Limit maximum sinkage to prevent numerical issues
    constexpr Real MAX_SINKAGE = 0.5;  // 50cm maximum
    sinkage_ = std::min(sinkage_, MAX_SINKAGE);

    // Calculate motion resistance (compaction resistance)
    // R_c = b * integral(p dz) from 0 to z
    // For Bekker equation: R_c = b * (k_c/b + k_phi) * z^(n+1) / (n+1)
    Real k_eq = soil.k_c / contact_width_ + soil.k_phi;  // Equivalent stiffness
    Real n1 = soil.n + 1.0;

    motion_resistance_ = contact_width_ * k_eq * std::pow(sinkage_, n1) / n1;
    motion_resistance_ *= 1000.0;  // Convert kN to N

    // Get vehicle velocity in body frame
    Vec3 v_body = state.orientation.conjugate().rotate(state.velocity);
    Real v_forward = v_body.x;  // Forward velocity

    // Calculate slip ratio
    // For now, assume wheel velocity equals commanded velocity (simplified)
    // A full model would track actual wheel angular velocity
    Real wheel_velocity = v_forward;  // Simplified: assume no slip command
    slip_ratio_ = compute_slip_ratio(wheel_velocity, v_forward);

    // Calculate maximum available traction using Mohr-Coulomb criterion
    // tau_max = c + sigma * tan(phi)
    // T_max = A * tau_max
    Real normal_stress = ground_pressure;  // Pa
    Real max_shear_stress = soil.c * 1000.0 + normal_stress * std::tan(soil.phi);  // Pa
    Real max_traction = contact_area * max_shear_stress;  // N

    // Apply slip-dependent traction model
    traction_ = compute_slip_traction(max_traction, slip_ratio_);

    // Build force vector in body frame
    // X: Forward (traction - resistance)
    // Y: Lateral (side forces if present)
    // Z: Normal (ground reaction)
    Vec3 force;

    // Longitudinal force = traction - motion resistance
    force.x = traction_ - motion_resistance_;

    // Lateral force (not modeled in this basic implementation)
    force.y = 0.0;

    // Normal force = vehicle weight (reaction from ground)
    // Only apply if actually on ground (sinkage > 0)
    if (sinkage_ > 0.0) {
        // Ground reaction counteracts gravity
        // In body frame, this is positive Z if vehicle is level
        force.z = vehicle_weight_;
    } else {
        force.z = 0.0;
    }

    out_forces.add_force(force);

    // Could add torque due to resistance acting at ground contact point
    // For now, we assume forces act at CG
}

Real TerramechanicsModel::calculate_sinkage(const SoilProperties& soil, Real pressure) const {
    // Bekker pressure-sinkage equation:
    // p = (k_c/b + k_phi) * z^n
    //
    // Solving for z:
    // z = (p / (k_c/b + k_phi))^(1/n)

    // Equivalent stiffness
    Real k_eq = soil.k_c / contact_width_ + soil.k_phi;

    // Avoid division by zero for very stiff soils
    if (k_eq < 1e-6) {
        return 0.0;
    }

    // Avoid issues with zero pressure
    if (pressure <= 0.0) {
        return 0.0;
    }

    // Calculate sinkage
    Real ratio = pressure / k_eq;

    // Handle special case where n = 0 (shouldn't happen in practice)
    if (std::abs(soil.n) < 1e-6) {
        return ratio;  // Linear relationship
    }

    // z = ratio^(1/n)
    Real sinkage = std::pow(ratio, 1.0 / soil.n);

    return sinkage;
}

void TerramechanicsModel::set_contact_area(Real width, Real length) {
    contact_width_ = std::max(width, 0.01);
    contact_length_ = std::max(length, 0.01);
}

void TerramechanicsModel::set_vehicle_weight(Real weight_n) {
    vehicle_weight_ = std::max(weight_n, 0.0);
}

} // namespace jaguar::domain::land
