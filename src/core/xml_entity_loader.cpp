/**
 * @file xml_entity_loader.cpp
 * @brief XML entity definition loader implementation
 *
 * Implements JSBSim-style XML entity loading using pugixml.
 * Supports multi-domain entity definitions with metrics, mass properties,
 * aerodynamics, propulsion, and other domain-specific physics data.
 */

#include "jaguar/interface/xml_entity_loader.h"
#include <pugixml.hpp>
#include <stdexcept>
#include <sstream>
#define _USE_MATH_DEFINES
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace jaguar::interface {

namespace {

// ============================================================================
// Unit Conversion Helpers
// ============================================================================

Real convert_to_si(Real value, const std::string& unit) {
    // Length conversions to meters
    if (unit == "ft") return value * 0.3048;
    if (unit == "in") return value * 0.0254;
    if (unit == "nm" || unit == "nmi") return value * 1852.0;
    if (unit == "km") return value * 1000.0;
    if (unit == "m") return value;

    // Area conversions to m²
    if (unit == "ft2" || unit == "ft^2") return value * 0.09290304;
    if (unit == "m2" || unit == "m^2") return value;

    // Mass conversions to kg
    if (unit == "lbs" || unit == "lb") return value * 0.45359237;
    if (unit == "slug") return value * 14.593903;
    if (unit == "kg") return value;

    // Force conversions to N
    if (unit == "lbf") return value * 4.4482216;
    if (unit == "kN") return value * 1000.0;
    if (unit == "N") return value;

    // Inertia conversions to kg*m²
    if (unit == "slug*ft2" || unit == "slug*ft^2") return value * 1.3558179;
    if (unit == "kg*m2" || unit == "kg*m^2") return value;

    // Angle conversions to radians
    if (unit == "deg") return value * M_PI / 180.0;
    if (unit == "rad") return value;

    // Default: assume SI
    return value;
}

Real parse_value_with_unit(const pugi::xml_node& node, const char* default_unit = "") {
    Real value = node.text().as_double(0.0);
    std::string unit = node.attribute("unit").as_string(default_unit);
    return unit.empty() ? value : convert_to_si(value, unit);
}

Vec3 parse_vec3(const pugi::xml_node& parent, const char* default_unit = "") {
    std::string unit = parent.attribute("unit").as_string(default_unit);

    Real x = parent.child("x").text().as_double(0.0);
    Real y = parent.child("y").text().as_double(0.0);
    Real z = parent.child("z").text().as_double(0.0);

    if (!unit.empty()) {
        x = convert_to_si(x, unit);
        y = convert_to_si(y, unit);
        z = convert_to_si(z, unit);
    }

    return Vec3{x, y, z};
}

} // anonymous namespace

// ============================================================================
// Domain Parsing
// ============================================================================

Domain XmlEntityLoader::parse_domain(const std::string& type) {
    if (type == "aircraft" || type == "air") return Domain::Air;
    if (type == "vehicle" || type == "land") return Domain::Land;
    if (type == "ship" || type == "vessel" || type == "sea" ||
        type == "submarine" || type == "subsurface") return Domain::Sea;
    if (type == "satellite" || type == "space") return Domain::Space;
    return Domain::Generic;
}

// ============================================================================
// Metrics Parsing
// ============================================================================

EntityMetrics XmlEntityLoader::parse_metrics(void* node_ptr) {
    auto node = static_cast<pugi::xml_node*>(node_ptr);
    EntityMetrics metrics;

    if (!node || node->empty()) {
        return metrics;
    }

    metrics.wingspan = parse_value_with_unit(node->child("wingspan"));
    metrics.length = parse_value_with_unit(node->child("length"));
    metrics.height = parse_value_with_unit(node->child("height"));
    metrics.wing_area = parse_value_with_unit(node->child("wing_area"));

    // Reference area defaults to wing area if not specified
    auto ref_area_node = node->child("reference_area");
    if (ref_area_node) {
        metrics.reference_area = parse_value_with_unit(ref_area_node);
    } else {
        metrics.reference_area = metrics.wing_area;
    }

    return metrics;
}

// ============================================================================
// Mass Balance Parsing
// ============================================================================

MassBalance XmlEntityLoader::parse_mass_balance(void* node_ptr) {
    auto node = static_cast<pugi::xml_node*>(node_ptr);
    MassBalance mass;

    if (!node || node->empty()) {
        return mass;
    }

    // Parse mass values
    mass.empty_mass = parse_value_with_unit(node->child("empty_weight"));

    auto fuel_cap = node->child("fuel_capacity");
    if (fuel_cap) {
        mass.fuel_mass = parse_value_with_unit(fuel_cap);
    }

    // Parse center of gravity
    auto cg = node->child("center_of_gravity");
    if (!cg) {
        cg = node->child("cg_location");
    }
    if (cg) {
        mass.cg_location = parse_vec3(cg);
    }

    // Parse inertia tensor
    auto inertia_node = node->child("inertia");
    if (inertia_node) {
        std::string unit = inertia_node.attribute("unit").as_string("");

        Real ixx = inertia_node.child("ixx").text().as_double(0.0);
        Real iyy = inertia_node.child("iyy").text().as_double(0.0);
        Real izz = inertia_node.child("izz").text().as_double(0.0);
        Real ixy = inertia_node.child("ixy").text().as_double(0.0);
        Real ixz = inertia_node.child("ixz").text().as_double(0.0);
        Real iyz = inertia_node.child("iyz").text().as_double(0.0);

        if (!unit.empty()) {
            ixx = convert_to_si(ixx, unit);
            iyy = convert_to_si(iyy, unit);
            izz = convert_to_si(izz, unit);
            ixy = convert_to_si(ixy, unit);
            ixz = convert_to_si(ixz, unit);
            iyz = convert_to_si(iyz, unit);
        }

        mass.inertia = Mat3x3::Inertia(ixx, iyy, izz, ixy, ixz, iyz);
    }

    return mass;
}

// ============================================================================
// Aerodynamics Parsing
// ============================================================================

AerodynamicsData XmlEntityLoader::parse_aerodynamics(void* node_ptr) {
    auto node = static_cast<pugi::xml_node*>(node_ptr);
    AerodynamicsData aero;

    if (!node || node->empty()) {
        return aero;
    }

    // Parse axes (LIFT, DRAG, SIDE, ROLL, PITCH, YAW)
    for (auto axis : node->children("axis")) {
        std::string axis_name = axis.attribute("name").as_string();

        // Parse functions within each axis
        for (auto func : axis.children("function")) {
            AeroCoefficient coeff;
            coeff.name = func.attribute("name").as_string();

            // Parse table data
            auto table = func.child("table");
            if (table) {
                // Parse independent variable breakpoints
                auto indep_var = table.child("independentVar");
                if (indep_var) {
                    // For now, we just note the variable name
                    // In a full implementation, we'd parse the breakpoints
                }

                // Parse table data (breakpoint-value pairs)
                auto table_data = table.child("tableData");
                if (table_data) {
                    std::istringstream iss(table_data.text().as_string());
                    Real bp, val;
                    while (iss >> bp >> val) {
                        coeff.breakpoints.push_back(bp);
                        coeff.values.push_back(val);
                    }
                }
            }

            // Categorize by axis
            if (axis_name == "LIFT") {
                aero.lift_tables.push_back(coeff);
            } else if (axis_name == "DRAG") {
                aero.drag_tables.push_back(coeff);
            } else {
                // ROLL, PITCH, YAW are moment coefficients
                aero.moment_tables.push_back(coeff);
            }
        }
    }

    return aero;
}

// ============================================================================
// Propulsion Parsing
// ============================================================================

PropulsionData XmlEntityLoader::parse_propulsion(void* node_ptr) {
    auto node = static_cast<pugi::xml_node*>(node_ptr);
    PropulsionData prop;

    if (!node || node->empty()) {
        return prop;
    }

    // Parse engine data
    auto engine = node->child("engine");
    if (engine) {
        prop.engine_type = engine.attribute("type").as_string();
        prop.max_thrust = parse_value_with_unit(engine.child("max_thrust"));

        auto ab_thrust = engine.child("afterburner_thrust");
        if (ab_thrust) {
            prop.afterburner_thrust = parse_value_with_unit(ab_thrust);
        }

        // Parse thrust location if specified
        auto thrust_loc = engine.child("location");
        if (thrust_loc) {
            prop.thrust_location = parse_vec3(thrust_loc);
        }
    }

    return prop;
}

// ============================================================================
// Component Mask Determination
// ============================================================================

ComponentMask XmlEntityLoader::determine_components(const EntityDefinition& def) {
    ComponentMask mask = 0;

    // Domain-specific components
    switch (def.domain) {
        case Domain::Air:
            if (def.aerodynamics.has_value()) {
                mask |= physics::ComponentBits::Aerodynamics;
            }
            if (def.propulsion.has_value()) {
                mask |= physics::ComponentBits::Propulsion;
            }
            mask |= physics::ComponentBits::FlightControl;
            break;

        case Domain::Land:
            mask |= physics::ComponentBits::Terramechanics;
            mask |= physics::ComponentBits::GroundContact;
            if (def.propulsion.has_value()) {
                mask |= physics::ComponentBits::Propulsion;
            }
            break;

        case Domain::Sea:
            mask |= physics::ComponentBits::Hydrodynamics;
            mask |= physics::ComponentBits::Buoyancy;
            if (def.propulsion.has_value()) {
                mask |= physics::ComponentBits::Propulsion;
            }
            break;

        case Domain::Space:
            mask |= physics::ComponentBits::OrbitalDynamics;
            if (def.propulsion.has_value()) {
                mask |= physics::ComponentBits::Propulsion;
            }
            break;

        case Domain::Generic:
        default:
            break;
    }

    // Always add gravity (unless in space with orbital dynamics)
    if (def.domain != Domain::Space) {
        mask |= physics::ComponentBits::Gravity;
    }

    return mask;
}

// ============================================================================
// Main Loading Functions
// ============================================================================

EntityDefinition XmlEntityLoader::load(const std::string& path) {
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(path.c_str());

    if (!result) {
        error_message_ = "Failed to load XML file: " + std::string(result.description());
        throw std::runtime_error(error_message_);
    }

    auto root = doc.child("entity");
    if (!root) {
        error_message_ = "Invalid entity XML: no <entity> root element";
        throw std::runtime_error(error_message_);
    }

    EntityDefinition def;

    // Parse basic attributes
    def.name = root.attribute("name").as_string("unnamed");
    def.type = root.attribute("type").as_string("generic");
    def.domain = parse_domain(def.type);

    // Parse metrics
    auto metrics_node = root.child("metrics");
    if (metrics_node) {
        def.metrics = parse_metrics(&metrics_node);
    }

    // Parse mass balance
    auto mass_node = root.child("mass_balance");
    if (mass_node) {
        def.mass_balance = parse_mass_balance(&mass_node);
    }

    // Parse aerodynamics (optional)
    auto aero_node = root.child("aerodynamics");
    if (aero_node) {
        def.aerodynamics = parse_aerodynamics(&aero_node);
    }

    // Parse propulsion (optional)
    auto prop_node = root.child("propulsion");
    if (prop_node) {
        def.propulsion = parse_propulsion(&prop_node);
    }

    // Determine component mask
    def.component_mask = determine_components(def);

    error_message_.clear();
    return def;
}

EntityDefinition XmlEntityLoader::load_from_string(const std::string& xml) {
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_string(xml.c_str());

    if (!result) {
        error_message_ = "Failed to parse XML string: " + std::string(result.description());
        throw std::runtime_error(error_message_);
    }

    auto root = doc.child("entity");
    if (!root) {
        error_message_ = "Invalid entity XML: no <entity> root element";
        throw std::runtime_error(error_message_);
    }

    EntityDefinition def;

    // Parse basic attributes
    def.name = root.attribute("name").as_string("unnamed");
    def.type = root.attribute("type").as_string("generic");
    def.domain = parse_domain(def.type);

    // Parse metrics
    auto metrics_node = root.child("metrics");
    if (metrics_node) {
        def.metrics = parse_metrics(&metrics_node);
    }

    // Parse mass balance
    auto mass_node = root.child("mass_balance");
    if (mass_node) {
        def.mass_balance = parse_mass_balance(&mass_node);
    }

    // Parse aerodynamics (optional)
    auto aero_node = root.child("aerodynamics");
    if (aero_node) {
        def.aerodynamics = parse_aerodynamics(&aero_node);
    }

    // Parse propulsion (optional)
    auto prop_node = root.child("propulsion");
    if (prop_node) {
        def.propulsion = parse_propulsion(&prop_node);
    }

    // Determine component mask
    def.component_mask = determine_components(def);

    error_message_.clear();
    return def;
}

} // namespace jaguar::interface
