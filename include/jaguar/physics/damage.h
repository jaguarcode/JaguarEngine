/**
 * @file damage.h
 * @brief Damage model system for JaguarEngine
 * @version 0.5.0
 *
 * Provides comprehensive damage modeling including:
 * - Kinetic energy penetration (AP rounds)
 * - Blast/overpressure effects (HE warheads)
 * - Fragment distribution and lethality
 * - Armor penetration calculations
 * - Component damage tracking
 *
 * References:
 * - JTCG/ME methodologies
 * - Thor equations for fragment penetration
 * - Hopkinson-Cranz scaling for blast
 */

#pragma once

#include "jaguar/core/types.h"
#include "jaguar/core/math/vector.h"
#include "jaguar/physics/collision.h"
#include <vector>
#include <unordered_map>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

namespace jaguar {
namespace physics {

// Forward declarations
class DamageModel;
class ArmorModel;
class VulnerabilityModel;

//=============================================================================
// Enumerations
//=============================================================================

/**
 * @brief Types of damage mechanisms
 */
enum class DamageType : uint8_t {
    KINETIC,        ///< Kinetic energy penetrator (AP, APFSDS)
    SHAPED_CHARGE,  ///< HEAT warhead, shaped charge jet
    BLAST,          ///< Overpressure/blast wave
    FRAGMENT,       ///< High-velocity fragments
    INCENDIARY,     ///< Fire/thermal damage
    EMP,            ///< Electromagnetic pulse
    SPALL           ///< Secondary fragments from armor
};

/**
 * @brief Armor material types
 */
enum class ArmorMaterial : uint8_t {
    STEEL_RHA,      ///< Rolled Homogeneous Armor (baseline)
    STEEL_HIGH_HARD,///< High-hardness steel
    ALUMINUM,       ///< Aluminum alloy
    TITANIUM,       ///< Titanium alloy
    CERAMIC,        ///< Ceramic composite
    COMPOSITE,      ///< Laminate composite armor
    ERA,            ///< Explosive Reactive Armor
    NERA,           ///< Non-Explosive Reactive Armor
    CAGE,           ///< Slat/cage armor
    APS_SOFT_KILL,  ///< Active Protection (soft-kill)
    APS_HARD_KILL   ///< Active Protection (hard-kill)
};

/**
 * @brief Component criticality levels
 */
enum class ComponentCriticality : uint8_t {
    NON_CRITICAL,   ///< Cosmetic/minor systems
    DEGRADED,       ///< Reduced capability
    MISSION_KILL,   ///< Cannot complete mission
    MOBILITY_KILL,  ///< Cannot move
    FIREPOWER_KILL, ///< Cannot engage targets
    CATASTROPHIC    ///< Complete destruction
};

/**
 * @brief Damage assessment result
 */
enum class DamageResult : uint8_t {
    NO_EFFECT,      ///< No penetration, no damage
    PARTIAL_PEN,    ///< Partial penetration
    FULL_PEN,       ///< Complete penetration
    BEHIND_ARMOR,   ///< Behind-armor effects
    KILL            ///< Target destroyed/killed
};

//=============================================================================
// Data Structures
//=============================================================================

/**
 * @brief Threat descriptor for damage calculations
 */
struct Threat {
    DamageType type = DamageType::KINETIC;

    // Common properties
    math::Vec3 position;        ///< Threat origin position
    math::Vec3 velocity;        ///< Velocity vector (m/s)
    double mass = 0.0;          ///< Mass (kg)
    double diameter = 0.0;      ///< Projectile diameter (m)

    // Kinetic penetrator properties
    double length = 0.0;        ///< Penetrator length (m)
    double density = 0.0;       ///< Material density (kg/m³)

    // Explosive properties
    double tnt_equivalent = 0.0;///< TNT equivalent mass (kg)
    double standoff = 0.0;      ///< Shaped charge standoff (m)
    double jet_velocity = 0.0;  ///< Shaped charge jet velocity (m/s)

    // Fragment properties
    int fragment_count = 0;     ///< Number of fragments
    double fragment_mass = 0.0; ///< Average fragment mass (kg)
    double spray_angle = 0.0;   ///< Fragment spray cone angle (rad)

    /**
     * @brief Calculate kinetic energy
     */
    double kinetic_energy() const {
        double speed = velocity.length();
        return 0.5 * mass * speed * speed;
    }

    /**
     * @brief Calculate momentum
     */
    double momentum() const {
        return mass * velocity.length();
    }
};

/**
 * @brief Armor plate/zone descriptor
 */
struct ArmorPlate {
    std::string name;
    ArmorMaterial material = ArmorMaterial::STEEL_RHA;

    double thickness = 0.0;         ///< Physical thickness (m)
    double rha_equivalent = 0.0;    ///< RHA equivalent thickness (m)
    double slope_angle = 0.0;       ///< Slope from vertical (rad)

    math::Vec3 normal;              ///< Outward facing normal
    math::Vec3 center;              ///< Center position
    double area = 0.0;              ///< Surface area (m²)

    // ERA/APS properties
    bool is_reactive = false;
    double era_reduction = 0.0;     ///< Penetration reduction factor
    bool is_active = true;          ///< For one-shot systems like ERA

    /**
     * @brief Calculate line-of-sight thickness
     * @param impact_angle Angle from plate normal (rad)
     * @return LOS thickness in meters
     */
    double los_thickness(double impact_angle) const {
        if (std::abs(impact_angle) >= M_PI / 2.0) {
            return std::numeric_limits<double>::infinity();
        }
        return thickness / std::cos(impact_angle);
    }

    /**
     * @brief Get RHA equivalent for given threat type
     */
    double get_rha_equivalent(DamageType threat_type) const;
};

/**
 * @brief Vulnerable component within a target
 */
struct VulnerableComponent {
    std::string name;
    ComponentCriticality criticality = ComponentCriticality::NON_CRITICAL;

    math::Vec3 position;            ///< Component center
    math::Vec3 dimensions;          ///< Bounding box size

    double presented_area = 0.0;    ///< Vulnerable area (m²)
    double kill_probability = 0.0;  ///< P(kill|hit) for this component

    // Damage state
    double health = 1.0;            ///< Current health (0-1)
    bool is_destroyed = false;

    /**
     * @brief Apply damage to component
     * @param damage_factor Normalized damage (0-1)
     * @return True if component was destroyed by this damage
     */
    bool apply_damage(double damage_factor) {
        if (is_destroyed) return false;

        health -= damage_factor;
        if (health <= 0.0) {
            health = 0.0;
            is_destroyed = true;
            return true;
        }
        return false;
    }
};

/**
 * @brief Hit location and penetration result
 */
struct HitResult {
    bool hit = false;
    math::Vec3 hit_point;
    math::Vec3 hit_normal;

    double impact_angle = 0.0;      ///< Angle from surface normal
    double obliquity = 0.0;         ///< NATO obliquity angle

    const ArmorPlate* armor = nullptr;
    DamageResult result = DamageResult::NO_EFFECT;

    double penetration_depth = 0.0; ///< How far penetrator traveled
    double residual_velocity = 0.0; ///< Exit velocity if penetrated
    double residual_mass = 0.0;     ///< Remaining mass after penetration

    // Behind-armor effects
    double spall_cone_angle = 0.0;
    int spall_fragment_count = 0;
    double spall_velocity = 0.0;
};

/**
 * @brief Blast damage assessment
 */
struct BlastResult {
    double range = 0.0;             ///< Distance from detonation
    double scaled_range = 0.0;      ///< Hopkinson-Cranz scaled range

    double peak_overpressure = 0.0; ///< Peak overpressure (Pa)
    double impulse = 0.0;           ///< Impulse (Pa·s)
    double duration = 0.0;          ///< Positive phase duration (s)

    double dynamic_pressure = 0.0;  ///< Dynamic pressure (Pa)
    double reflected_pressure = 0.0;///< Reflected pressure if applicable

    DamageResult result = DamageResult::NO_EFFECT;
};

/**
 * @brief Fragment spray result
 */
struct FragmentResult {
    int total_fragments = 0;
    int hits = 0;

    std::vector<HitResult> fragment_hits;

    double total_presented_area = 0.0;
    double expected_hits = 0.0;     ///< Statistical expected value

    DamageResult result = DamageResult::NO_EFFECT;
};

/**
 * @brief Complete damage assessment for a single engagement
 */
struct DamageAssessment {
    EntityId target_id = 0;
    EntityId attacker_id = 0;
    double timestamp = 0.0;

    Threat threat;
    HitResult hit;
    BlastResult blast;
    FragmentResult fragments;

    // Aggregate results
    DamageResult overall_result = DamageResult::NO_EFFECT;
    ComponentCriticality kill_level = ComponentCriticality::NON_CRITICAL;

    std::vector<std::pair<std::string, double>> component_damage;

    double probability_of_kill = 0.0;
    double probability_of_damage = 0.0;
};

//=============================================================================
// Penetration Equations
//=============================================================================

/**
 * @brief Armor penetration calculation methods
 */
class PenetrationEquations {
public:
    /**
     * @brief DeMarre equation for kinetic penetrators
     *
     * Classic equation for AP projectile penetration:
     * P = K * (m^0.5 * v^n) / d^0.75
     *
     * @param mass Projectile mass (kg)
     * @param velocity Impact velocity (m/s)
     * @param diameter Projectile diameter (m)
     * @param obliquity Impact obliquity (rad)
     * @return Penetration in meters of RHA
     */
    static double demarre(double mass, double velocity, double diameter,
                          double obliquity = 0.0);

    /**
     * @brief Lanz-Odermatt equation for long rod penetrators
     *
     * Modern equation for APFSDS penetration:
     * P/L = (rho_p/rho_t)^0.5 * (1 - (v_c/v)^2)^0.5
     *
     * @param length Penetrator length (m)
     * @param diameter Penetrator diameter (m)
     * @param density Penetrator density (kg/m³)
     * @param velocity Impact velocity (m/s)
     * @param target_density Target material density (kg/m³)
     * @param obliquity Impact obliquity (rad)
     * @return Penetration in meters
     */
    static double lanz_odermatt(double length, double diameter, double density,
                                 double velocity, double target_density,
                                 double obliquity = 0.0);

    /**
     * @brief Shaped charge jet penetration
     *
     * Hydrodynamic penetration:
     * P = L * sqrt(rho_j / rho_t)
     *
     * @param jet_length Effective jet length (m)
     * @param jet_velocity Jet tip velocity (m/s)
     * @param jet_density Jet material density (kg/m³)
     * @param standoff Standoff distance (m)
     * @param target_density Target density (kg/m³)
     * @return Penetration in meters
     */
    static double shaped_charge(double jet_length, double jet_velocity,
                                double jet_density, double standoff,
                                double target_density);

    /**
     * @brief Thor equation for fragment penetration
     *
     * Empirical equation for fragment perforation:
     * t = c * m^a * v^b * (sec(theta))^g
     *
     * @param fragment_mass Fragment mass (kg)
     * @param velocity Impact velocity (m/s)
     * @param material Target material
     * @param obliquity Impact angle (rad)
     * @return Thickness that can be perforated (m)
     */
    static double thor_equation(double fragment_mass, double velocity,
                                ArmorMaterial material, double obliquity = 0.0);
};

//=============================================================================
// Blast Model
//=============================================================================

/**
 * @brief Blast wave propagation and damage model
 *
 * Implements Hopkinson-Cranz scaling and Kingery-Bulmash curves
 */
class BlastModel {
public:
    /**
     * @brief Calculate scaled distance (Hopkinson-Cranz)
     * @param range Distance from detonation (m)
     * @param charge_mass TNT equivalent mass (kg)
     * @return Scaled distance Z (m/kg^(1/3))
     */
    static double scaled_distance(double range, double charge_mass);

    /**
     * @brief Calculate peak overpressure at range
     * @param scaled_range Hopkinson-Cranz scaled range
     * @return Peak overpressure (Pa)
     */
    static double peak_overpressure(double scaled_range);

    /**
     * @brief Calculate positive phase impulse
     * @param scaled_range Hopkinson-Cranz scaled range
     * @param charge_mass TNT equivalent (kg)
     * @return Impulse (Pa·s)
     */
    static double impulse(double scaled_range, double charge_mass);

    /**
     * @brief Calculate positive phase duration
     * @param scaled_range Hopkinson-Cranz scaled range
     * @param charge_mass TNT equivalent (kg)
     * @return Duration (s)
     */
    static double positive_duration(double scaled_range, double charge_mass);

    /**
     * @brief Calculate reflected pressure for surface impact
     * @param incident_pressure Incident overpressure (Pa)
     * @param angle_of_incidence Angle from surface normal (rad)
     * @return Reflected pressure (Pa)
     */
    static double reflected_pressure(double incident_pressure,
                                     double angle_of_incidence);

    /**
     * @brief Assess blast damage to target
     * @param detonation_point Point of detonation
     * @param charge_mass TNT equivalent (kg)
     * @param target_point Target location
     * @param target_area Presented area (m²)
     * @return Blast damage assessment
     */
    static BlastResult assess_blast_damage(const math::Vec3& detonation_point,
                                           double charge_mass,
                                           const math::Vec3& target_point,
                                           double target_area);

    /**
     * @brief Damage thresholds for various effects
     */
    struct DamageThresholds {
        double glass_breakage = 1000.0;      ///< Pa - window breakage
        double eardrum_rupture = 35000.0;    ///< Pa - human injury
        double lung_damage = 100000.0;       ///< Pa - lethal to humans
        double structural_damage = 70000.0;  ///< Pa - building damage
        double vehicle_damage = 200000.0;    ///< Pa - vehicle damage
        double armor_breach = 500000.0;      ///< Pa - armored vehicle
    };

    static const DamageThresholds thresholds;
};

//=============================================================================
// Fragment Model
//=============================================================================

/**
 * @brief Fragment distribution and lethality model
 *
 * Implements Mott distribution for natural fragmentation
 * and Gurney equations for fragment velocity
 */
class FragmentModel {
public:
    /**
     * @brief Calculate Gurney velocity for fragments
     * @param charge_mass Explosive mass (kg)
     * @param casing_mass Casing mass (kg)
     * @param gurney_constant Material-specific constant (m/s)
     * @return Initial fragment velocity (m/s)
     */
    static double gurney_velocity(double charge_mass, double casing_mass,
                                  double gurney_constant = 2400.0);

    /**
     * @brief Generate Mott fragment distribution
     * @param casing_mass Total casing mass (kg)
     * @param mott_constant Material constant
     * @return Vector of (fragment_mass, count) pairs
     */
    static std::vector<std::pair<double, int>>
    mott_distribution(double casing_mass, double mott_constant = 0.00035);

    /**
     * @brief Calculate fragment spray density
     * @param total_fragments Total fragment count
     * @param spray_angle Spray cone half-angle (rad)
     * @param range Distance from detonation (m)
     * @return Fragments per square meter
     */
    static double spray_density(int total_fragments, double spray_angle,
                                double range);

    /**
     * @brief Calculate expected hits on target
     * @param spray_density Fragments per m²
     * @param presented_area Target presented area (m²)
     * @return Expected number of hits
     */
    static double expected_hits(double spray_density, double presented_area);

    /**
     * @brief Assess fragment damage to target
     * @param detonation Detonation point
     * @param threat Fragment threat descriptor
     * @param target_point Target center
     * @param target_area Target presented area
     * @param armor Target armor (if any)
     * @return Fragment damage assessment
     */
    static FragmentResult assess_fragment_damage(
        const math::Vec3& detonation,
        const Threat& threat,
        const math::Vec3& target_point,
        double target_area,
        const ArmorPlate* armor = nullptr);
};

//=============================================================================
// Armor Model
//=============================================================================

/**
 * @brief Complete armor model for a target
 */
class ArmorModel {
public:
    ArmorModel() = default;

    /**
     * @brief Add armor plate to model
     */
    void add_plate(ArmorPlate plate);

    /**
     * @brief Find armor plate at ray intersection
     * @param origin Ray origin
     * @param direction Ray direction (normalized)
     * @return Pointer to plate and hit distance, or nullptr if no hit
     */
    std::pair<const ArmorPlate*, double>
    ray_cast(const math::Vec3& origin, const math::Vec3& direction) const;

    /**
     * @brief Get all plates that could be hit from direction
     */
    std::vector<const ArmorPlate*>
    get_exposed_plates(const math::Vec3& direction) const;

    /**
     * @brief Calculate total RHA equivalent from direction
     */
    double total_protection(const math::Vec3& impact_point,
                           const math::Vec3& direction) const;

    /**
     * @brief Trigger ERA if present at location
     * @return True if ERA was triggered
     */
    bool trigger_era(const math::Vec3& hit_point);

    const std::vector<ArmorPlate>& get_plates() const { return plates_; }

private:
    std::vector<ArmorPlate> plates_;
};

//=============================================================================
// Vulnerability Model
//=============================================================================

/**
 * @brief Target vulnerability model with components
 */
class VulnerabilityModel {
public:
    VulnerabilityModel() = default;

    /**
     * @brief Add vulnerable component
     */
    void add_component(VulnerableComponent component);

    /**
     * @brief Get components that could be hit by penetrating threat
     */
    std::vector<VulnerableComponent*>
    get_threatened_components(const math::Vec3& entry_point,
                              const math::Vec3& direction,
                              double penetration_depth);

    /**
     * @brief Apply damage to components along threat path
     * @return Highest criticality level reached
     */
    ComponentCriticality apply_penetration_damage(
        const math::Vec3& entry_point,
        const math::Vec3& direction,
        double penetration_depth,
        double damage_factor);

    /**
     * @brief Apply blast damage based on overpressure
     */
    ComponentCriticality apply_blast_damage(double overpressure,
                                            double impulse);

    /**
     * @brief Calculate overall kill probability
     */
    double calculate_pk(ComponentCriticality target_level) const;

    /**
     * @brief Get current damage state summary
     */
    struct DamageState {
        double overall_health = 1.0;
        bool mobility_kill = false;
        bool firepower_kill = false;
        bool mission_kill = false;
        bool catastrophic_kill = false;
        std::vector<std::string> destroyed_components;
    };

    DamageState get_damage_state() const;

    /**
     * @brief Reset all damage
     */
    void reset();

    const std::vector<VulnerableComponent>& get_components() const {
        return components_;
    }

private:
    std::vector<VulnerableComponent> components_;

    void update_kill_states();
};

//=============================================================================
// Damage Model (Main Interface)
//=============================================================================

/**
 * @brief Complete damage assessment model
 *
 * Combines armor, vulnerability, and penetration models
 * to assess damage from threats to targets.
 */
class DamageModel {
public:
    DamageModel();
    ~DamageModel();

    /**
     * @brief Register a target with its models
     * @param entity_id Target entity ID
     * @param armor Armor model for the target
     * @param vulnerability Vulnerability model for the target
     */
    void register_target(EntityId entity_id,
                        std::unique_ptr<ArmorModel> armor,
                        std::unique_ptr<VulnerabilityModel> vulnerability);

    /**
     * @brief Remove target from damage tracking
     */
    void unregister_target(EntityId entity_id);

    /**
     * @brief Assess damage from threat to target
     * @param target_id Target entity ID
     * @param threat Incoming threat
     * @param target_position Target current position
     * @param target_orientation Target orientation quaternion
     * @return Complete damage assessment
     */
    DamageAssessment assess_damage(EntityId target_id,
                                   const Threat& threat,
                                   const math::Vec3& target_position,
                                   const math::Quat& target_orientation);

    /**
     * @brief Apply assessed damage to target
     * @param assessment Previously calculated assessment
     * @return True if target was killed
     */
    bool apply_damage(const DamageAssessment& assessment);

    /**
     * @brief Get current damage state of target
     */
    VulnerabilityModel::DamageState
    get_target_state(EntityId entity_id) const;

    /**
     * @brief Check if target has been killed
     */
    bool is_target_killed(EntityId entity_id,
                          ComponentCriticality kill_level =
                              ComponentCriticality::CATASTROPHIC) const;

    /**
     * @brief Reset damage for target
     */
    void reset_target(EntityId entity_id);

    /**
     * @brief Reset all targets
     */
    void reset_all();

    /**
     * @brief Set damage assessment callback
     */
    using DamageCallback = std::function<void(const DamageAssessment&)>;
    void set_damage_callback(DamageCallback callback);

    /**
     * @brief Get armor model for target
     */
    ArmorModel* get_armor(EntityId entity_id);
    const ArmorModel* get_armor(EntityId entity_id) const;

    /**
     * @brief Get vulnerability model for target
     */
    VulnerabilityModel* get_vulnerability(EntityId entity_id);
    const VulnerabilityModel* get_vulnerability(EntityId entity_id) const;

private:
    struct TargetData {
        std::unique_ptr<ArmorModel> armor;
        std::unique_ptr<VulnerabilityModel> vulnerability;
    };

    std::unordered_map<EntityId, TargetData> targets_;
    DamageCallback damage_callback_;

    // Internal damage calculation methods
    HitResult calculate_kinetic_penetration(const Threat& threat,
                                            const ArmorPlate* armor,
                                            const math::Vec3& hit_point,
                                            const math::Vec3& hit_normal);

    HitResult calculate_shaped_charge_penetration(const Threat& threat,
                                                  const ArmorPlate* armor,
                                                  const math::Vec3& hit_point,
                                                  const math::Vec3& hit_normal);
};

//=============================================================================
// Utility Functions
//=============================================================================

/**
 * @brief Material property lookup
 */
struct MaterialProperties {
    double density;         ///< kg/m³
    double rha_multiplier;  ///< vs RHA protection
    double bhn;             ///< Brinell hardness

    static MaterialProperties get(ArmorMaterial material);
};

/**
 * @brief Convert between angle conventions
 */
namespace AngleUtils {
    /**
     * @brief NATO obliquity from impact angle
     * Obliquity is measured from the plate surface
     */
    inline double to_obliquity(double impact_angle) {
        return M_PI / 2.0 - std::abs(impact_angle);
    }

    /**
     * @brief Impact angle from NATO obliquity
     */
    inline double from_obliquity(double obliquity) {
        return M_PI / 2.0 - obliquity;
    }
}

/**
 * @brief Common warhead/projectile presets
 */
namespace Presets {
    /**
     * @brief Create kinetic penetrator threat
     */
    Threat apfsds_round(const math::Vec3& position,
                        const math::Vec3& velocity,
                        double caliber_mm = 120.0);

    /**
     * @brief Create HEAT warhead threat
     */
    Threat heat_warhead(const math::Vec3& position,
                        const math::Vec3& velocity,
                        double diameter_mm = 100.0,
                        double charge_kg = 2.0);

    /**
     * @brief Create HE-FRAG warhead threat
     */
    Threat he_frag_warhead(const math::Vec3& position,
                           double charge_kg = 5.0,
                           double casing_kg = 10.0);

    /**
     * @brief Create artillery HE shell threat
     */
    Threat artillery_he(const math::Vec3& position,
                        double caliber_mm = 155.0);
}

/**
 * @brief Common armor configurations
 */
namespace ArmorPresets {
    /**
     * @brief Create main battle tank armor model
     */
    std::unique_ptr<ArmorModel> mbt_armor();

    /**
     * @brief Create infantry fighting vehicle armor
     */
    std::unique_ptr<ArmorModel> ifv_armor();

    /**
     * @brief Create wheeled APC armor
     */
    std::unique_ptr<ArmorModel> apc_armor();

    /**
     * @brief Create helicopter armor
     */
    std::unique_ptr<ArmorModel> helicopter_armor();
}

/**
 * @brief Common vulnerability configurations
 */
namespace VulnerabilityPresets {
    /**
     * @brief Create MBT vulnerability model
     */
    std::unique_ptr<VulnerabilityModel> mbt_vulnerability();

    /**
     * @brief Create IFV vulnerability model
     */
    std::unique_ptr<VulnerabilityModel> ifv_vulnerability();

    /**
     * @brief Create helicopter vulnerability model
     */
    std::unique_ptr<VulnerabilityModel> helicopter_vulnerability();
}

} // namespace physics
} // namespace jaguar
