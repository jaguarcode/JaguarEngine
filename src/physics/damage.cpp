/**
 * @file damage.cpp
 * @brief Implementation of damage model system
 * @version 0.5.0
 */

#include "jaguar/physics/damage.h"
#include <algorithm>
#include <cmath>
#include <random>

namespace jaguar {
namespace physics {

//=============================================================================
// Constants
//=============================================================================

namespace {
    // Physical constants
    constexpr double ATMOSPHERIC_PRESSURE = 101325.0;  // Pa
    constexpr double SOUND_SPEED = 343.0;              // m/s at sea level

    // Material densities (kg/m³)
    constexpr double DENSITY_STEEL = 7850.0;
    constexpr double DENSITY_TUNGSTEN = 19300.0;
    constexpr double DENSITY_DU = 19100.0;             // Depleted uranium
    constexpr double DENSITY_COPPER = 8960.0;
    constexpr double DENSITY_ALUMINUM = 2700.0;
    constexpr double DENSITY_TITANIUM = 4500.0;

    // DeMarre constants
    constexpr double DEMARRE_K = 0.0023;               // Empirical constant
    constexpr double DEMARRE_N = 1.43;                 // Velocity exponent

    // Lanz-Odermatt constants
    constexpr double LO_CRITICAL_VELOCITY = 1100.0;    // m/s

    // Thor equation coefficients for RHA
    constexpr double THOR_C = 0.0001296;
    constexpr double THOR_A = 0.5;
    constexpr double THOR_B = 1.0;
    constexpr double THOR_G = 1.0;

    // Random generator for stochastic effects
    std::mt19937& get_rng() {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        return gen;
    }
}

//=============================================================================
// ArmorPlate Implementation
//=============================================================================

double ArmorPlate::get_rha_equivalent(DamageType threat_type) const {
    if (rha_equivalent > 0.0) {
        return rha_equivalent;
    }

    // Calculate based on material
    double multiplier = 1.0;

    switch (material) {
        case ArmorMaterial::STEEL_RHA:
            multiplier = 1.0;
            break;
        case ArmorMaterial::STEEL_HIGH_HARD:
            multiplier = 1.15;  // Better vs KE
            break;
        case ArmorMaterial::ALUMINUM:
            // Aluminum is worse vs KE, similar vs CE
            multiplier = (threat_type == DamageType::KINETIC) ? 0.35 : 0.5;
            break;
        case ArmorMaterial::TITANIUM:
            multiplier = 0.9;
            break;
        case ArmorMaterial::CERAMIC:
            // Ceramic excellent vs KE, less vs CE
            multiplier = (threat_type == DamageType::KINETIC) ? 2.5 : 1.5;
            break;
        case ArmorMaterial::COMPOSITE:
            multiplier = (threat_type == DamageType::KINETIC) ? 2.0 : 1.5;
            break;
        case ArmorMaterial::ERA:
            // ERA has base armor plus reactive effect
            multiplier = 1.0;  // Base, ERA reduction handled separately
            break;
        case ArmorMaterial::NERA:
            multiplier = 1.8;
            break;
        case ArmorMaterial::CAGE:
            // Slat armor only effective vs HEAT
            multiplier = (threat_type == DamageType::SHAPED_CHARGE) ? 0.5 : 0.05;
            break;
        default:
            multiplier = 1.0;
    }

    return thickness * multiplier;
}

//=============================================================================
// PenetrationEquations Implementation
//=============================================================================

double PenetrationEquations::demarre(double mass, double velocity,
                                     double diameter, double obliquity) {
    if (velocity <= 0.0 || diameter <= 0.0 || mass <= 0.0) {
        return 0.0;
    }

    // DeMarre equation: P = K * m^0.5 * v^n / d^0.75
    double base_pen = DEMARRE_K * std::sqrt(mass) *
                      std::pow(velocity, DEMARRE_N) /
                      std::pow(diameter, 0.75);

    // Apply obliquity effect (cos^n factor)
    double obliquity_factor = std::pow(std::cos(obliquity), 1.4);

    return base_pen * obliquity_factor;
}

double PenetrationEquations::lanz_odermatt(double length, double diameter,
                                           double density, double velocity,
                                           double target_density,
                                           double obliquity) {
    if (velocity <= LO_CRITICAL_VELOCITY || length <= 0.0) {
        return 0.0;
    }

    // Hydrodynamic limit: P/L = sqrt(rho_p / rho_t)
    double density_ratio = std::sqrt(density / target_density);

    // Velocity effect: (1 - (v_c/v)^2)^0.5
    double vel_ratio = LO_CRITICAL_VELOCITY / velocity;
    double velocity_factor = std::sqrt(1.0 - vel_ratio * vel_ratio);

    // L/D effect (longer rods are more efficient)
    double ld_ratio = length / diameter;
    double ld_factor = 1.0 + 0.1 * std::log(ld_ratio / 10.0);
    ld_factor = std::max(0.8, std::min(1.2, ld_factor));

    double base_pen = length * density_ratio * velocity_factor * ld_factor;

    // Obliquity effect
    double obliquity_factor = std::pow(std::cos(obliquity), 1.2);

    return base_pen * obliquity_factor;
}

double PenetrationEquations::shaped_charge(double jet_length, double jet_velocity,
                                           double jet_density, double standoff,
                                           double target_density) {
    if (jet_velocity <= 0.0 || jet_length <= 0.0) {
        return 0.0;
    }

    // Hydrodynamic penetration
    double base_pen = jet_length * std::sqrt(jet_density / target_density);

    // Standoff effect - optimal at ~1-2 cone diameters
    // Penetration drops at very short or very long standoffs
    double optimal_standoff = jet_length * 2.0;
    double standoff_ratio = standoff / optimal_standoff;
    double standoff_factor;

    if (standoff_ratio < 0.5) {
        // Too close - jet not fully formed
        standoff_factor = 0.7 + 0.6 * standoff_ratio;
    } else if (standoff_ratio > 2.0) {
        // Too far - jet particulates
        standoff_factor = 1.0 / (1.0 + 0.2 * (standoff_ratio - 2.0));
    } else {
        standoff_factor = 1.0;
    }

    return base_pen * standoff_factor;
}

double PenetrationEquations::thor_equation(double fragment_mass, double velocity,
                                           ArmorMaterial material,
                                           double obliquity) {
    if (velocity <= 0.0 || fragment_mass <= 0.0) {
        return 0.0;
    }

    // Get material-specific coefficients
    double c = THOR_C;
    double a = THOR_A;
    double b = THOR_B;
    double g = THOR_G;

    // Adjust for material
    switch (material) {
        case ArmorMaterial::ALUMINUM:
            c *= 2.5;  // Aluminum perforates easier
            break;
        case ArmorMaterial::STEEL_HIGH_HARD:
            c *= 0.85;
            break;
        case ArmorMaterial::COMPOSITE:
            c *= 0.6;
            break;
        default:
            break;
    }

    // Thor equation: t = c * m^a * v^b * (sec(theta))^g
    double sec_obliquity = 1.0 / std::cos(obliquity);
    double thickness = c * std::pow(fragment_mass * 1000.0, a) *  // convert to grams
                       std::pow(velocity, b) *
                       std::pow(sec_obliquity, g);

    return thickness / 1000.0;  // Return in meters
}

//=============================================================================
// BlastModel Implementation
//=============================================================================

const BlastModel::DamageThresholds BlastModel::thresholds;

double BlastModel::scaled_distance(double range, double charge_mass) {
    if (charge_mass <= 0.0) return std::numeric_limits<double>::infinity();
    return range / std::cbrt(charge_mass);
}

double BlastModel::peak_overpressure(double scaled_range) {
    if (scaled_range <= 0.0) return std::numeric_limits<double>::infinity();

    // Kingery-Bulmash curve fit (simplified)
    // Valid for Z > 0.5 m/kg^(1/3)
    if (scaled_range < 0.5) {
        scaled_range = 0.5;
    }

    // Ps = 0.84 / Z + 2.7 / Z^2 + 6.9 / Z^3 (bar)
    double ps_bar = 0.84 / scaled_range +
                    2.7 / (scaled_range * scaled_range) +
                    6.9 / (scaled_range * scaled_range * scaled_range);

    return ps_bar * 100000.0;  // Convert bar to Pa
}

double BlastModel::impulse(double scaled_range, double charge_mass) {
    if (scaled_range <= 0.0 || charge_mass <= 0.0) return 0.0;

    // Simplified Kingery-Bulmash impulse
    // is = 200 * W^(1/3) / Z (Pa·ms)
    double is = 200.0 * std::cbrt(charge_mass) / scaled_range;

    return is / 1000.0;  // Convert to Pa·s
}

double BlastModel::positive_duration(double scaled_range, double charge_mass) {
    if (scaled_range <= 0.0 || charge_mass <= 0.0) return 0.0;

    // t+ = 1.0 * W^(1/3) * Z^0.5 (ms)
    double t_ms = 1.0 * std::cbrt(charge_mass) * std::sqrt(scaled_range);

    return t_ms / 1000.0;  // Convert to seconds
}

double BlastModel::reflected_pressure(double incident_pressure,
                                      double angle_of_incidence) {
    // Reflection coefficient varies with angle and pressure
    // Simplified: Cr = 2 + 6*(P/P0) at normal incidence
    double p_ratio = incident_pressure / ATMOSPHERIC_PRESSURE;
    double cr_normal = 2.0 + 6.0 * p_ratio / (1.0 + 6.0 * p_ratio);

    // Reduce with angle
    double cos_angle = std::cos(angle_of_incidence);
    double cr = 1.0 + (cr_normal - 1.0) * cos_angle * cos_angle;

    return incident_pressure * cr;
}

BlastResult BlastModel::assess_blast_damage(const math::Vec3& detonation_point,
                                            double charge_mass,
                                            const math::Vec3& target_point,
                                            double target_area) {
    BlastResult result;

    // Calculate range
    math::Vec3 delta = target_point - detonation_point;
    result.range = delta.length();

    if (result.range < 0.1) result.range = 0.1;  // Minimum range

    // Scaled quantities
    result.scaled_range = scaled_distance(result.range, charge_mass);
    result.peak_overpressure = peak_overpressure(result.scaled_range);
    result.impulse = impulse(result.scaled_range, charge_mass);
    result.duration = positive_duration(result.scaled_range, charge_mass);

    // Dynamic pressure (wind load)
    double gamma = 1.4;  // Ratio of specific heats for air
    double p = result.peak_overpressure;
    result.dynamic_pressure = p * p / (2.0 * gamma * ATMOSPHERIC_PRESSURE + p);

    // Assess damage level
    if (result.peak_overpressure >= thresholds.armor_breach) {
        result.result = DamageResult::KILL;
    } else if (result.peak_overpressure >= thresholds.vehicle_damage) {
        result.result = DamageResult::FULL_PEN;
    } else if (result.peak_overpressure >= thresholds.structural_damage) {
        result.result = DamageResult::PARTIAL_PEN;
    } else {
        result.result = DamageResult::NO_EFFECT;
    }

    return result;
}

//=============================================================================
// FragmentModel Implementation
//=============================================================================

double FragmentModel::gurney_velocity(double charge_mass, double casing_mass,
                                      double gurney_constant) {
    if (charge_mass <= 0.0 || casing_mass <= 0.0) return 0.0;

    // Gurney equation for cylinders: V = sqrt(2E) * sqrt(C/(M + C/2))
    double c_over_m = charge_mass / casing_mass;
    double factor = std::sqrt(c_over_m / (1.0 + c_over_m / 2.0));

    return gurney_constant * factor;
}

std::vector<std::pair<double, int>>
FragmentModel::mott_distribution(double casing_mass, double mott_constant) {
    std::vector<std::pair<double, int>> distribution;

    if (casing_mass <= 0.0) return distribution;

    // Mott distribution: N(m) = (M0/2*mu^2) * exp(-sqrt(m/mu))
    // where mu is the characteristic mass

    double mu = mott_constant * casing_mass;  // Characteristic fragment mass
    double remaining_mass = casing_mass;

    // Generate discrete distribution
    std::vector<double> mass_bins = {
        0.0001, 0.0005, 0.001, 0.002, 0.005,  // 0.1g to 5g
        0.01, 0.02, 0.05, 0.1, 0.2, 0.5       // 10g to 500g
    };

    for (size_t i = 0; i < mass_bins.size() - 1; ++i) {
        double m_low = mass_bins[i];
        double m_high = mass_bins[i + 1];
        double m_avg = (m_low + m_high) / 2.0;

        // Count fragments in this bin
        double n_low = std::exp(-std::sqrt(m_low / mu));
        double n_high = std::exp(-std::sqrt(m_high / mu));
        double fraction = n_low - n_high;

        int count = static_cast<int>(fraction * casing_mass / m_avg);
        if (count > 0) {
            distribution.emplace_back(m_avg, count);
            remaining_mass -= count * m_avg;
        }
    }

    return distribution;
}

double FragmentModel::spray_density(int total_fragments, double spray_angle,
                                    double range) {
    if (range <= 0.0 || spray_angle <= 0.0) return 0.0;

    // Solid angle of spray cone: Omega = 2*pi*(1 - cos(angle))
    double solid_angle = 2.0 * M_PI * (1.0 - std::cos(spray_angle));

    // Area at range: A = r^2 * Omega
    double spray_area = range * range * solid_angle;

    return static_cast<double>(total_fragments) / spray_area;
}

double FragmentModel::expected_hits(double spray_density, double presented_area) {
    return spray_density * presented_area;
}

FragmentResult FragmentModel::assess_fragment_damage(
    const math::Vec3& detonation,
    const Threat& threat,
    const math::Vec3& target_point,
    double target_area,
    const ArmorPlate* armor) {

    FragmentResult result;

    math::Vec3 delta = target_point - detonation;
    double range = delta.length();

    if (range < 0.1) range = 0.1;

    // Calculate initial fragment velocity
    double v0 = gurney_velocity(threat.tnt_equivalent,
                                 threat.fragment_count * threat.fragment_mass,
                                 2400.0);

    // Velocity decay with range (simplified drag model)
    // V(r) = V0 * exp(-k*r)
    double drag_coeff = 0.001;  // Depends on fragment shape
    double velocity_at_range = v0 * std::exp(-drag_coeff * range);

    // Calculate spray density and expected hits
    double density = spray_density(threat.fragment_count, threat.spray_angle, range);
    result.expected_hits = expected_hits(density, target_area);
    result.total_fragments = threat.fragment_count;
    result.total_presented_area = target_area;

    // Sample actual hits (Poisson distribution)
    std::poisson_distribution<int> poisson(result.expected_hits);
    result.hits = poisson(get_rng());

    // Assess each hit
    for (int i = 0; i < result.hits; ++i) {
        HitResult hit;
        hit.hit = true;

        // Random impact angle variation
        std::uniform_real_distribution<double> angle_dist(-0.5, 0.5);
        hit.impact_angle = angle_dist(get_rng());

        if (armor != nullptr) {
            // Calculate fragment penetration
            double pen = PenetrationEquations::thor_equation(
                threat.fragment_mass, velocity_at_range,
                armor->material, hit.impact_angle);

            double los_thick = armor->los_thickness(hit.impact_angle);

            if (pen >= los_thick) {
                hit.result = DamageResult::FULL_PEN;
                hit.residual_velocity = velocity_at_range *
                    std::sqrt(1.0 - los_thick / pen);
            } else if (pen >= los_thick * 0.7) {
                hit.result = DamageResult::PARTIAL_PEN;
            } else {
                hit.result = DamageResult::NO_EFFECT;
            }
        } else {
            // Unarmored target
            hit.result = DamageResult::FULL_PEN;
            hit.residual_velocity = velocity_at_range;
        }

        result.fragment_hits.push_back(hit);
    }

    // Overall result based on penetrating hits
    int penetrating = 0;
    for (const auto& hit : result.fragment_hits) {
        if (hit.result == DamageResult::FULL_PEN) {
            penetrating++;
        }
    }

    if (penetrating >= 5) {
        result.result = DamageResult::KILL;
    } else if (penetrating >= 2) {
        result.result = DamageResult::FULL_PEN;
    } else if (penetrating >= 1) {
        result.result = DamageResult::PARTIAL_PEN;
    } else {
        result.result = DamageResult::NO_EFFECT;
    }

    return result;
}

//=============================================================================
// ArmorModel Implementation
//=============================================================================

void ArmorModel::add_plate(ArmorPlate plate) {
    plates_.push_back(std::move(plate));
}

std::pair<const ArmorPlate*, double>
ArmorModel::ray_cast(const math::Vec3& origin,
                     const math::Vec3& direction) const {
    const ArmorPlate* closest_plate = nullptr;
    double closest_dist = std::numeric_limits<double>::infinity();

    for (const auto& plate : plates_) {
        // Simple plane intersection test
        double denom = direction.dot(plate.normal);
        if (std::abs(denom) < 1e-6) continue;  // Parallel

        math::Vec3 to_center = plate.center - origin;
        double t = to_center.dot(plate.normal) / denom;

        if (t > 0.0 && t < closest_dist) {
            // Check if hit point is within plate area (simplified)
            math::Vec3 hit = origin + direction * t;
            math::Vec3 offset = hit - plate.center;

            double dist_sq = offset.length_squared();
            double radius_sq = plate.area / M_PI;  // Circular approximation

            if (dist_sq <= radius_sq) {
                closest_dist = t;
                closest_plate = &plate;
            }
        }
    }

    return {closest_plate, closest_dist};
}

std::vector<const ArmorPlate*>
ArmorModel::get_exposed_plates(const math::Vec3& direction) const {
    std::vector<const ArmorPlate*> exposed;

    for (const auto& plate : plates_) {
        // Plate is exposed if facing the threat direction
        if (plate.normal.dot(direction) < 0.0) {
            exposed.push_back(&plate);
        }
    }

    return exposed;
}

double ArmorModel::total_protection(const math::Vec3& impact_point,
                                    const math::Vec3& direction) const {
    double total = 0.0;

    for (const auto& plate : plates_) {
        // Check if plate is in the path
        double alignment = plate.normal.dot(-direction);
        if (alignment < 0.1) continue;  // Not facing threat

        // Distance from impact to plate
        math::Vec3 to_plate = plate.center - impact_point;
        double dist = to_plate.dot(direction);
        if (dist < 0.0) continue;  // Behind impact

        // Add plate protection
        double angle = std::acos(alignment);
        total += plate.los_thickness(angle);
    }

    return total;
}

bool ArmorModel::trigger_era(const math::Vec3& hit_point) {
    for (auto& plate : plates_) {
        if (!plate.is_reactive || !plate.is_active) continue;

        // Check if hit is on this plate
        math::Vec3 offset = hit_point - plate.center;
        double dist_sq = offset.length_squared();
        double radius_sq = plate.area / M_PI;

        if (dist_sq <= radius_sq) {
            plate.is_active = false;  // ERA is one-shot
            return true;
        }
    }
    return false;
}

//=============================================================================
// VulnerabilityModel Implementation
//=============================================================================

void VulnerabilityModel::add_component(VulnerableComponent component) {
    components_.push_back(std::move(component));
}

std::vector<VulnerableComponent*>
VulnerabilityModel::get_threatened_components(const math::Vec3& entry_point,
                                              const math::Vec3& direction,
                                              double penetration_depth) {
    std::vector<VulnerableComponent*> threatened;

    for (auto& comp : components_) {
        if (comp.is_destroyed) continue;

        // Ray-box intersection test
        math::Vec3 to_comp = comp.position - entry_point;
        double dist_along_ray = to_comp.dot(direction);

        if (dist_along_ray < 0.0 || dist_along_ray > penetration_depth) {
            continue;
        }

        // Check perpendicular distance
        math::Vec3 closest_point = entry_point + direction * dist_along_ray;
        math::Vec3 offset = comp.position - closest_point;

        // Simplified: use max dimension as radius
        double radius = std::max({comp.dimensions.x, comp.dimensions.y,
                                  comp.dimensions.z}) / 2.0;

        if (offset.length() <= radius) {
            threatened.push_back(&comp);
        }
    }

    return threatened;
}

ComponentCriticality VulnerabilityModel::apply_penetration_damage(
    const math::Vec3& entry_point,
    const math::Vec3& direction,
    double penetration_depth,
    double damage_factor) {

    auto threatened = get_threatened_components(entry_point, direction,
                                                 penetration_depth);

    ComponentCriticality highest = ComponentCriticality::NON_CRITICAL;

    for (auto* comp : threatened) {
        bool destroyed = comp->apply_damage(damage_factor * comp->kill_probability);

        if (destroyed && comp->criticality > highest) {
            highest = comp->criticality;
        }
    }

    update_kill_states();
    return highest;
}

ComponentCriticality VulnerabilityModel::apply_blast_damage(double overpressure,
                                                            double impulse) {
    ComponentCriticality highest = ComponentCriticality::NON_CRITICAL;

    // Damage factor based on overpressure
    double damage_factor = 0.0;
    if (overpressure > 500000.0) {
        damage_factor = 1.0;
    } else if (overpressure > 200000.0) {
        damage_factor = (overpressure - 200000.0) / 300000.0;
    }

    for (auto& comp : components_) {
        if (comp.is_destroyed) continue;

        // External components more vulnerable to blast
        double vulnerability = damage_factor;
        if (comp.criticality <= ComponentCriticality::DEGRADED) {
            vulnerability *= 1.5;  // External/exposed components
        }

        bool destroyed = comp.apply_damage(vulnerability);
        if (destroyed && comp.criticality > highest) {
            highest = comp.criticality;
        }
    }

    update_kill_states();
    return highest;
}

double VulnerabilityModel::calculate_pk(ComponentCriticality target_level) const {
    // Count components at or above target criticality
    int total = 0;
    int destroyed = 0;

    for (const auto& comp : components_) {
        if (comp.criticality >= target_level) {
            total++;
            if (comp.is_destroyed) {
                destroyed++;
            }
        }
    }

    if (total == 0) return 0.0;
    return static_cast<double>(destroyed) / total;
}

VulnerabilityModel::DamageState VulnerabilityModel::get_damage_state() const {
    DamageState state;

    double total_health = 0.0;
    int component_count = 0;

    for (const auto& comp : components_) {
        total_health += comp.health;
        component_count++;

        if (comp.is_destroyed) {
            state.destroyed_components.push_back(comp.name);

            switch (comp.criticality) {
                case ComponentCriticality::MOBILITY_KILL:
                    state.mobility_kill = true;
                    break;
                case ComponentCriticality::FIREPOWER_KILL:
                    state.firepower_kill = true;
                    break;
                case ComponentCriticality::MISSION_KILL:
                    state.mission_kill = true;
                    break;
                case ComponentCriticality::CATASTROPHIC:
                    state.catastrophic_kill = true;
                    break;
                default:
                    break;
            }
        }
    }

    if (component_count > 0) {
        state.overall_health = total_health / component_count;
    }

    return state;
}

void VulnerabilityModel::reset() {
    for (auto& comp : components_) {
        comp.health = 1.0;
        comp.is_destroyed = false;
    }
}

void VulnerabilityModel::update_kill_states() {
    // Called after damage to update aggregate states
    // Implementation is in get_damage_state() for now
}

//=============================================================================
// DamageModel Implementation
//=============================================================================

DamageModel::DamageModel() = default;
DamageModel::~DamageModel() = default;

void DamageModel::register_target(EntityId entity_id,
                                  std::unique_ptr<ArmorModel> armor,
                                  std::unique_ptr<VulnerabilityModel> vulnerability) {
    targets_[entity_id] = TargetData{std::move(armor), std::move(vulnerability)};
}

void DamageModel::unregister_target(EntityId entity_id) {
    targets_.erase(entity_id);
}

DamageAssessment DamageModel::assess_damage(EntityId target_id,
                                            const Threat& threat,
                                            const math::Vec3& target_position,
                                            const math::Quat& target_orientation) {
    DamageAssessment assessment;
    assessment.target_id = target_id;
    assessment.threat = threat;

    auto it = targets_.find(target_id);
    if (it == targets_.end()) {
        return assessment;  // Target not registered
    }

    const auto& target = it->second;

    // Calculate threat direction in target local coordinates
    math::Vec3 threat_dir = (threat.velocity).normalized();
    math::Vec3 to_target = target_position - threat.position;

    // Find armor at impact point
    if (target.armor) {
        auto [plate, dist] = target.armor->ray_cast(threat.position, threat_dir);

        if (plate) {
            assessment.hit.hit = true;
            assessment.hit.hit_point = threat.position + threat_dir * dist;
            assessment.hit.hit_normal = plate->normal;
            assessment.hit.armor = plate;

            // Calculate impact angle
            assessment.hit.impact_angle = std::acos(
                std::abs(threat_dir.dot(plate->normal)));
            assessment.hit.obliquity = AngleUtils::to_obliquity(
                assessment.hit.impact_angle);

            // Calculate penetration based on threat type
            switch (threat.type) {
                case DamageType::KINETIC:
                    assessment.hit = calculate_kinetic_penetration(
                        threat, plate, assessment.hit.hit_point,
                        assessment.hit.hit_normal);
                    break;

                case DamageType::SHAPED_CHARGE:
                    assessment.hit = calculate_shaped_charge_penetration(
                        threat, plate, assessment.hit.hit_point,
                        assessment.hit.hit_normal);
                    break;

                case DamageType::BLAST:
                    assessment.blast = BlastModel::assess_blast_damage(
                        threat.position, threat.tnt_equivalent,
                        target_position, 10.0);  // Assume 10m² presented area
                    break;

                case DamageType::FRAGMENT:
                    assessment.fragments = FragmentModel::assess_fragment_damage(
                        threat.position, threat, target_position, 10.0, plate);
                    break;

                default:
                    break;
            }
        }
    }

    // Determine overall result
    if (assessment.hit.result >= DamageResult::FULL_PEN ||
        assessment.blast.result >= DamageResult::FULL_PEN ||
        assessment.fragments.result >= DamageResult::FULL_PEN) {
        assessment.overall_result = DamageResult::FULL_PEN;
    } else if (assessment.hit.result >= DamageResult::PARTIAL_PEN ||
               assessment.blast.result >= DamageResult::PARTIAL_PEN ||
               assessment.fragments.result >= DamageResult::PARTIAL_PEN) {
        assessment.overall_result = DamageResult::PARTIAL_PEN;
    }

    return assessment;
}

HitResult DamageModel::calculate_kinetic_penetration(
    const Threat& threat,
    const ArmorPlate* armor,
    const math::Vec3& hit_point,
    const math::Vec3& hit_normal) {

    HitResult result;
    result.hit = true;
    result.hit_point = hit_point;
    result.hit_normal = hit_normal;
    result.armor = armor;

    math::Vec3 threat_dir = threat.velocity.normalized();
    result.impact_angle = std::acos(std::abs(threat_dir.dot(hit_normal)));
    result.obliquity = AngleUtils::to_obliquity(result.impact_angle);

    double velocity = threat.velocity.length();

    // Calculate penetration
    double penetration;
    if (threat.length > 0.0 && threat.length / threat.diameter > 5.0) {
        // Long rod penetrator (APFSDS)
        penetration = PenetrationEquations::lanz_odermatt(
            threat.length, threat.diameter, threat.density,
            velocity, DENSITY_STEEL, result.obliquity);
    } else {
        // Conventional AP round
        penetration = PenetrationEquations::demarre(
            threat.mass, velocity, threat.diameter, result.obliquity);
    }

    result.penetration_depth = penetration;

    // Compare to armor
    double armor_equiv = armor->get_rha_equivalent(DamageType::KINETIC);
    double los_thick = armor->los_thickness(result.impact_angle);

    if (penetration >= los_thick) {
        result.result = DamageResult::FULL_PEN;

        // Calculate residual penetration
        double excess_pen = penetration - los_thick;
        result.residual_velocity = velocity * std::sqrt(excess_pen / penetration);
        result.residual_mass = threat.mass * 0.8;  // Some mass loss

        // Spall effects
        result.spall_cone_angle = M_PI / 6.0;  // 30 degrees
        result.spall_fragment_count = static_cast<int>(50.0 * los_thick / 0.01);
        result.spall_velocity = result.residual_velocity * 0.5;

    } else if (penetration >= los_thick * 0.7) {
        result.result = DamageResult::PARTIAL_PEN;

        // Some spall even without full penetration
        result.spall_cone_angle = M_PI / 8.0;
        result.spall_fragment_count = static_cast<int>(20.0 * los_thick / 0.01);
        result.spall_velocity = velocity * 0.2;
    } else {
        result.result = DamageResult::NO_EFFECT;
    }

    return result;
}

HitResult DamageModel::calculate_shaped_charge_penetration(
    const Threat& threat,
    const ArmorPlate* armor,
    const math::Vec3& hit_point,
    const math::Vec3& hit_normal) {

    HitResult result;
    result.hit = true;
    result.hit_point = hit_point;
    result.hit_normal = hit_normal;
    result.armor = armor;

    // HEAT warhead - calculate jet penetration
    double jet_length = threat.diameter * 3.0;  // Typical jet length
    double jet_density = DENSITY_COPPER;  // Copper liner

    double penetration = PenetrationEquations::shaped_charge(
        jet_length, threat.jet_velocity, jet_density,
        threat.standoff, DENSITY_STEEL);

    result.penetration_depth = penetration;

    // Check for ERA
    bool era_triggered = false;
    double era_reduction = 1.0;
    if (armor->is_reactive && armor->is_active) {
        era_triggered = true;
        era_reduction = 1.0 - armor->era_reduction;  // Typically 0.3-0.5
    }

    penetration *= era_reduction;

    // Compare to armor
    double armor_equiv = armor->get_rha_equivalent(DamageType::SHAPED_CHARGE);
    double los_thick = armor->los_thickness(0.0);  // HEAT less affected by angle

    if (penetration >= los_thick) {
        result.result = DamageResult::FULL_PEN;
        result.residual_velocity = threat.jet_velocity * 0.3;

        // HEAT produces a narrow but deep jet behind armor
        result.spall_cone_angle = M_PI / 12.0;  // 15 degrees
        result.spall_fragment_count = 10;
        result.spall_velocity = result.residual_velocity;
    } else {
        result.result = DamageResult::NO_EFFECT;
    }

    return result;
}

bool DamageModel::apply_damage(const DamageAssessment& assessment) {
    auto it = targets_.find(assessment.target_id);
    if (it == targets_.end()) return false;

    auto& target = it->second;
    if (!target.vulnerability) return false;

    ComponentCriticality highest = ComponentCriticality::NON_CRITICAL;

    // Apply penetration damage
    if (assessment.hit.result >= DamageResult::PARTIAL_PEN) {
        math::Vec3 direction = assessment.threat.velocity.normalized();
        double damage_factor = (assessment.hit.result == DamageResult::FULL_PEN)
                               ? 1.0 : 0.5;

        auto level = target.vulnerability->apply_penetration_damage(
            assessment.hit.hit_point, direction,
            assessment.hit.penetration_depth, damage_factor);

        if (level > highest) highest = level;
    }

    // Apply blast damage
    if (assessment.blast.result != DamageResult::NO_EFFECT) {
        auto level = target.vulnerability->apply_blast_damage(
            assessment.blast.peak_overpressure, assessment.blast.impulse);

        if (level > highest) highest = level;
    }

    // Notify callback
    if (damage_callback_) {
        DamageAssessment updated = assessment;
        updated.kill_level = highest;
        damage_callback_(updated);
    }

    return highest >= ComponentCriticality::CATASTROPHIC;
}

VulnerabilityModel::DamageState
DamageModel::get_target_state(EntityId entity_id) const {
    auto it = targets_.find(entity_id);
    if (it != targets_.end() && it->second.vulnerability) {
        return it->second.vulnerability->get_damage_state();
    }
    return VulnerabilityModel::DamageState{};
}

bool DamageModel::is_target_killed(EntityId entity_id,
                                   ComponentCriticality kill_level) const {
    auto state = get_target_state(entity_id);

    switch (kill_level) {
        case ComponentCriticality::CATASTROPHIC:
            return state.catastrophic_kill;
        case ComponentCriticality::MISSION_KILL:
            return state.mission_kill || state.catastrophic_kill;
        case ComponentCriticality::MOBILITY_KILL:
            return state.mobility_kill || state.mission_kill ||
                   state.catastrophic_kill;
        case ComponentCriticality::FIREPOWER_KILL:
            return state.firepower_kill || state.mission_kill ||
                   state.catastrophic_kill;
        default:
            return false;
    }
}

void DamageModel::reset_target(EntityId entity_id) {
    auto it = targets_.find(entity_id);
    if (it != targets_.end() && it->second.vulnerability) {
        it->second.vulnerability->reset();
    }
}

void DamageModel::reset_all() {
    for (auto& [id, target] : targets_) {
        if (target.vulnerability) {
            target.vulnerability->reset();
        }
    }
}

void DamageModel::set_damage_callback(DamageCallback callback) {
    damage_callback_ = std::move(callback);
}

ArmorModel* DamageModel::get_armor(EntityId entity_id) {
    auto it = targets_.find(entity_id);
    return (it != targets_.end()) ? it->second.armor.get() : nullptr;
}

const ArmorModel* DamageModel::get_armor(EntityId entity_id) const {
    auto it = targets_.find(entity_id);
    return (it != targets_.end()) ? it->second.armor.get() : nullptr;
}

VulnerabilityModel* DamageModel::get_vulnerability(EntityId entity_id) {
    auto it = targets_.find(entity_id);
    return (it != targets_.end()) ? it->second.vulnerability.get() : nullptr;
}

const VulnerabilityModel* DamageModel::get_vulnerability(EntityId entity_id) const {
    auto it = targets_.find(entity_id);
    return (it != targets_.end()) ? it->second.vulnerability.get() : nullptr;
}

//=============================================================================
// MaterialProperties Implementation
//=============================================================================

MaterialProperties MaterialProperties::get(ArmorMaterial material) {
    MaterialProperties props;

    switch (material) {
        case ArmorMaterial::STEEL_RHA:
            props = {DENSITY_STEEL, 1.0, 300};
            break;
        case ArmorMaterial::STEEL_HIGH_HARD:
            props = {DENSITY_STEEL, 1.15, 500};
            break;
        case ArmorMaterial::ALUMINUM:
            props = {DENSITY_ALUMINUM, 0.35, 150};
            break;
        case ArmorMaterial::TITANIUM:
            props = {DENSITY_TITANIUM, 0.9, 350};
            break;
        case ArmorMaterial::CERAMIC:
            props = {3900.0, 2.5, 1500};  // Alumina ceramic
            break;
        case ArmorMaterial::COMPOSITE:
            props = {2500.0, 2.0, 0};
            break;
        default:
            props = {DENSITY_STEEL, 1.0, 300};
    }

    return props;
}

//=============================================================================
// Presets Implementation
//=============================================================================

namespace Presets {

Threat apfsds_round(const math::Vec3& position, const math::Vec3& velocity,
                    double caliber_mm) {
    Threat t;
    t.type = DamageType::KINETIC;
    t.position = position;
    t.velocity = velocity;

    // Typical APFSDS parameters scaled by caliber
    double scale = caliber_mm / 120.0;

    t.diameter = caliber_mm * 0.001 * 0.25;  // Sub-caliber penetrator
    t.length = t.diameter * 25.0;             // L/D ~25
    t.density = DENSITY_TUNGSTEN;
    t.mass = t.density * M_PI * t.diameter * t.diameter / 4.0 * t.length;

    return t;
}

Threat heat_warhead(const math::Vec3& position, const math::Vec3& velocity,
                    double diameter_mm, double charge_kg) {
    Threat t;
    t.type = DamageType::SHAPED_CHARGE;
    t.position = position;
    t.velocity = velocity;

    t.diameter = diameter_mm * 0.001;
    t.tnt_equivalent = charge_kg * 1.2;  // RDX equivalent
    t.standoff = t.diameter * 1.5;
    t.jet_velocity = 8000.0;  // Typical copper jet

    return t;
}

Threat he_frag_warhead(const math::Vec3& position, double charge_kg,
                       double casing_kg) {
    Threat t;
    t.type = DamageType::FRAGMENT;
    t.position = position;

    t.tnt_equivalent = charge_kg;
    t.fragment_count = static_cast<int>(casing_kg * 200);  // ~200 frags/kg
    t.fragment_mass = casing_kg / t.fragment_count;
    t.spray_angle = M_PI * 0.6;  // ~108 degrees half-angle

    return t;
}

Threat artillery_he(const math::Vec3& position, double caliber_mm) {
    double scale = caliber_mm / 155.0;

    double charge = 7.0 * scale * scale * scale;    // ~7kg for 155mm
    double casing = 40.0 * scale * scale * scale;   // ~40kg casing

    return he_frag_warhead(position, charge, casing);
}

} // namespace Presets

//=============================================================================
// ArmorPresets Implementation
//=============================================================================

namespace ArmorPresets {

std::unique_ptr<ArmorModel> mbt_armor() {
    auto armor = std::make_unique<ArmorModel>();

    // Front hull - composite
    ArmorPlate front_hull;
    front_hull.name = "Front Hull";
    front_hull.material = ArmorMaterial::COMPOSITE;
    front_hull.thickness = 0.6;
    front_hull.rha_equivalent = 0.8;
    front_hull.slope_angle = M_PI / 3.0;  // 60 degrees
    front_hull.normal = math::Vec3{0.0, 1.0, -0.577}.normalized();
    front_hull.center = math::Vec3{0.0, 2.0, 0.5};
    front_hull.area = 4.0;
    armor->add_plate(front_hull);

    // Turret front - composite + ERA
    ArmorPlate turret_front;
    turret_front.name = "Turret Front";
    turret_front.material = ArmorMaterial::COMPOSITE;
    turret_front.thickness = 0.8;
    turret_front.rha_equivalent = 1.0;
    turret_front.normal = math::Vec3{0.0, 1.0, 0.0};
    turret_front.center = math::Vec3{0.0, 1.5, 2.0};
    turret_front.area = 2.5;
    turret_front.is_reactive = true;
    turret_front.era_reduction = 0.5;
    armor->add_plate(turret_front);

    // Side hull - RHA
    ArmorPlate side_hull;
    side_hull.name = "Side Hull";
    side_hull.material = ArmorMaterial::STEEL_RHA;
    side_hull.thickness = 0.08;
    side_hull.normal = math::Vec3{1.0, 0.0, 0.0};
    side_hull.center = math::Vec3{1.5, 0.0, 0.75};
    side_hull.area = 8.0;
    armor->add_plate(side_hull);

    // Rear - thin RHA
    ArmorPlate rear;
    rear.name = "Rear";
    rear.material = ArmorMaterial::STEEL_RHA;
    rear.thickness = 0.04;
    rear.normal = math::Vec3{0.0, -1.0, 0.0};
    rear.center = math::Vec3{0.0, -3.0, 1.0};
    rear.area = 5.0;
    armor->add_plate(rear);

    // Top - thin
    ArmorPlate top;
    top.name = "Top";
    top.material = ArmorMaterial::STEEL_RHA;
    top.thickness = 0.03;
    top.normal = math::Vec3{0.0, 0.0, 1.0};
    top.center = math::Vec3{0.0, 0.0, 2.5};
    top.area = 15.0;
    armor->add_plate(top);

    return armor;
}

std::unique_ptr<ArmorModel> ifv_armor() {
    auto armor = std::make_unique<ArmorModel>();

    // All-round aluminum protection
    ArmorPlate front;
    front.name = "Front";
    front.material = ArmorMaterial::ALUMINUM;
    front.thickness = 0.05;
    front.slope_angle = M_PI / 4.0;
    front.normal = math::Vec3{0.0, 1.0, -1.0}.normalized();
    front.center = math::Vec3{0.0, 2.5, 1.0};
    front.area = 6.0;
    armor->add_plate(front);

    ArmorPlate side;
    side.name = "Side";
    side.material = ArmorMaterial::ALUMINUM;
    side.thickness = 0.03;
    side.normal = math::Vec3{1.0, 0.0, 0.0};
    side.center = math::Vec3{1.2, 0.0, 1.0};
    side.area = 12.0;
    armor->add_plate(side);

    return armor;
}

std::unique_ptr<ArmorModel> apc_armor() {
    auto armor = std::make_unique<ArmorModel>();

    ArmorPlate all_around;
    all_around.name = "Hull";
    all_around.material = ArmorMaterial::STEEL_RHA;
    all_around.thickness = 0.012;  // 12mm
    all_around.normal = math::Vec3{0.0, 1.0, 0.0};
    all_around.center = math::Vec3{0.0, 0.0, 1.0};
    all_around.area = 20.0;
    armor->add_plate(all_around);

    return armor;
}

std::unique_ptr<ArmorModel> helicopter_armor() {
    auto armor = std::make_unique<ArmorModel>();

    // Cockpit floor armor
    ArmorPlate cockpit;
    cockpit.name = "Cockpit Floor";
    cockpit.material = ArmorMaterial::CERAMIC;
    cockpit.thickness = 0.02;
    cockpit.normal = math::Vec3{0.0, 0.0, -1.0};
    cockpit.center = math::Vec3{0.0, 2.0, 0.5};
    cockpit.area = 3.0;
    armor->add_plate(cockpit);

    // Engine compartment
    ArmorPlate engine;
    engine.name = "Engine";
    engine.material = ArmorMaterial::STEEL_RHA;
    engine.thickness = 0.008;
    engine.normal = math::Vec3{0.0, 0.0, 1.0};
    engine.center = math::Vec3{0.0, -1.0, 1.5};
    engine.area = 2.0;
    armor->add_plate(engine);

    return armor;
}

} // namespace ArmorPresets

//=============================================================================
// VulnerabilityPresets Implementation
//=============================================================================

namespace VulnerabilityPresets {

std::unique_ptr<VulnerabilityModel> mbt_vulnerability() {
    auto vuln = std::make_unique<VulnerabilityModel>();

    // Crew
    VulnerableComponent crew;
    crew.name = "Crew Compartment";
    crew.criticality = ComponentCriticality::CATASTROPHIC;
    crew.position = math::Vec3{0.0, 0.0, 1.5};
    crew.dimensions = math::Vec3{2.0, 2.0, 1.5};
    crew.presented_area = 2.0;
    crew.kill_probability = 0.8;
    vuln->add_component(crew);

    // Ammunition
    VulnerableComponent ammo;
    ammo.name = "Ammunition Storage";
    ammo.criticality = ComponentCriticality::CATASTROPHIC;
    ammo.position = math::Vec3{0.0, -1.0, 0.5};
    ammo.dimensions = math::Vec3{1.5, 2.0, 1.0};
    ammo.presented_area = 1.5;
    ammo.kill_probability = 0.9;
    vuln->add_component(ammo);

    // Engine
    VulnerableComponent engine;
    engine.name = "Engine";
    engine.criticality = ComponentCriticality::MOBILITY_KILL;
    engine.position = math::Vec3{0.0, -2.5, 0.75};
    engine.dimensions = math::Vec3{1.5, 1.5, 1.0};
    engine.presented_area = 1.5;
    engine.kill_probability = 0.6;
    vuln->add_component(engine);

    // Gun/breech
    VulnerableComponent gun;
    gun.name = "Main Gun";
    gun.criticality = ComponentCriticality::FIREPOWER_KILL;
    gun.position = math::Vec3{0.0, 3.0, 2.0};
    gun.dimensions = math::Vec3{0.3, 5.0, 0.3};
    gun.presented_area = 0.5;
    gun.kill_probability = 0.7;
    vuln->add_component(gun);

    // Fire control
    VulnerableComponent fcs;
    fcs.name = "Fire Control System";
    fcs.criticality = ComponentCriticality::FIREPOWER_KILL;
    fcs.position = math::Vec3{0.3, 1.0, 2.5};
    fcs.dimensions = math::Vec3{0.5, 0.5, 0.5};
    fcs.presented_area = 0.2;
    fcs.kill_probability = 0.8;
    vuln->add_component(fcs);

    // Tracks
    VulnerableComponent tracks;
    tracks.name = "Track/Suspension";
    tracks.criticality = ComponentCriticality::MOBILITY_KILL;
    tracks.position = math::Vec3{1.5, 0.0, 0.3};
    tracks.dimensions = math::Vec3{0.5, 6.0, 0.6};
    tracks.presented_area = 3.0;
    tracks.kill_probability = 0.4;
    vuln->add_component(tracks);

    return vuln;
}

std::unique_ptr<VulnerabilityModel> ifv_vulnerability() {
    auto vuln = std::make_unique<VulnerabilityModel>();

    VulnerableComponent crew;
    crew.name = "Crew/Passengers";
    crew.criticality = ComponentCriticality::CATASTROPHIC;
    crew.position = math::Vec3{0.0, 0.0, 1.0};
    crew.dimensions = math::Vec3{2.0, 4.0, 1.5};
    crew.presented_area = 4.0;
    crew.kill_probability = 0.7;
    vuln->add_component(crew);

    VulnerableComponent engine;
    engine.name = "Engine";
    engine.criticality = ComponentCriticality::MOBILITY_KILL;
    engine.position = math::Vec3{0.0, 2.0, 0.75};
    engine.dimensions = math::Vec3{1.0, 1.0, 0.75};
    engine.presented_area = 1.0;
    engine.kill_probability = 0.6;
    vuln->add_component(engine);

    VulnerableComponent turret;
    turret.name = "Turret/Weapon";
    turret.criticality = ComponentCriticality::FIREPOWER_KILL;
    turret.position = math::Vec3{0.0, 1.0, 2.0};
    turret.dimensions = math::Vec3{1.0, 1.0, 0.5};
    turret.presented_area = 0.5;
    turret.kill_probability = 0.7;
    vuln->add_component(turret);

    return vuln;
}

std::unique_ptr<VulnerabilityModel> helicopter_vulnerability() {
    auto vuln = std::make_unique<VulnerabilityModel>();

    VulnerableComponent cockpit;
    cockpit.name = "Cockpit/Crew";
    cockpit.criticality = ComponentCriticality::CATASTROPHIC;
    cockpit.position = math::Vec3{0.0, 3.0, 0.5};
    cockpit.dimensions = math::Vec3{1.5, 2.0, 1.5};
    cockpit.presented_area = 2.0;
    cockpit.kill_probability = 0.9;
    vuln->add_component(cockpit);

    VulnerableComponent engine;
    engine.name = "Engine/Transmission";
    engine.criticality = ComponentCriticality::CATASTROPHIC;  // Helicopters can't glide
    engine.position = math::Vec3{0.0, -1.0, 1.5};
    engine.dimensions = math::Vec3{1.5, 2.0, 1.0};
    engine.presented_area = 2.0;
    engine.kill_probability = 0.7;
    vuln->add_component(engine);

    VulnerableComponent rotor;
    rotor.name = "Main Rotor";
    rotor.criticality = ComponentCriticality::CATASTROPHIC;
    rotor.position = math::Vec3{0.0, 0.0, 2.5};
    rotor.dimensions = math::Vec3{12.0, 12.0, 0.2};
    rotor.presented_area = 5.0;
    rotor.kill_probability = 0.3;  // Hard to hit but critical
    vuln->add_component(rotor);

    VulnerableComponent tail;
    tail.name = "Tail Rotor";
    tail.criticality = ComponentCriticality::CATASTROPHIC;
    tail.position = math::Vec3{0.0, -8.0, 1.5};
    tail.dimensions = math::Vec3{2.0, 0.2, 2.0};
    tail.presented_area = 0.5;
    tail.kill_probability = 0.5;
    vuln->add_component(tail);

    VulnerableComponent fuel;
    fuel.name = "Fuel System";
    fuel.criticality = ComponentCriticality::CATASTROPHIC;
    fuel.position = math::Vec3{0.0, 0.0, 0.5};
    fuel.dimensions = math::Vec3{1.0, 3.0, 0.5};
    fuel.presented_area = 1.5;
    fuel.kill_probability = 0.6;
    vuln->add_component(fuel);

    return vuln;
}

} // namespace VulnerabilityPresets

} // namespace physics
} // namespace jaguar
