/**
 * @file xml_entity_loader.cpp
 * @brief XML entity definition loader implementation
 */

#include "jaguar/interface/xml_entity_loader.h"
#include "jaguar/physics/entity.h"
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <regex>

// Using pugixml for XML parsing (header-only library)
#ifdef JAGUAR_USE_PUGIXML
#include <pugixml.hpp>
#endif

namespace jaguar::interface {

// Unit conversion factors
static constexpr Real FT_TO_M = 0.3048;
static constexpr Real FT2_TO_M2 = 0.09290304;
static constexpr Real LBS_TO_KG = 0.453592;
static constexpr Real LBF_TO_N = 4.44822;
static constexpr Real SLUG_FT2_TO_KG_M2 = 1.35582;
static constexpr Real IN_TO_M = 0.0254;

EntityDefinition XmlEntityLoader::load(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        error_message_ = "Failed to open file: " + path;
        throw std::runtime_error(error_message_);
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    return load_from_string(buffer.str());
}

EntityDefinition XmlEntityLoader::load_from_string(const std::string& xml) {
    error_message_.clear();
    EntityDefinition def;

#ifdef JAGUAR_USE_PUGIXML
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_string(xml.c_str());

    if (!result) {
        error_message_ = std::string("XML parse error: ") + result.description();
        throw std::runtime_error(error_message_);
    }

    pugi::xml_node entity_node = doc.child("entity");
    if (!entity_node) {
        error_message_ = "Missing <entity> root element";
        throw std::runtime_error(error_message_);
    }

    // Parse attributes
    def.type = entity_node.attribute("type").as_string("generic");
    def.name = entity_node.attribute("name").as_string("unnamed");
    def.domain = parse_domain(def.type);

    // Parse metrics
    if (auto metrics_node = entity_node.child("metrics")) {
        auto& m = def.metrics;

        auto get_value = [&](const char* name) -> Real {
            auto node = metrics_node.child(name);
            if (!node) return 0.0;
            Real value = node.text().as_double(0.0);
            std::string unit = node.attribute("unit").as_string("");

            // Convert to SI units
            if (unit == "ft") value *= FT_TO_M;
            else if (unit == "ft2") value *= FT2_TO_M2;
            else if (unit == "in") value *= IN_TO_M;

            return value;
        };

        m.wingspan = get_value("wingspan");
        m.length = get_value("length");
        m.height = get_value("height");
        m.wing_area = get_value("wing_area");
        m.reference_area = m.wing_area;  // Default to wing area
    }

    // Parse mass_balance
    if (auto mass_node = entity_node.child("mass_balance")) {
        auto& mb = def.mass_balance;

        auto get_mass = [&](const char* name) -> Real {
            auto node = mass_node.child(name);
            if (!node) return 0.0;
            Real value = node.text().as_double(0.0);
            std::string unit = node.attribute("unit").as_string("");
            if (unit == "lbs") value *= LBS_TO_KG;
            return value;
        };

        mb.empty_mass = get_mass("empty_weight");
        mb.fuel_mass = get_mass("fuel_capacity");

        // CG location
        if (auto cg_node = mass_node.child("cg_location")) {
            std::string unit = cg_node.attribute("unit").as_string("");
            Real factor = 1.0;
            if (unit == "in") factor = IN_TO_M;
            else if (unit == "ft") factor = FT_TO_M;

            mb.cg_location.x() = cg_node.child("x").text().as_double(0.0) * factor;
            mb.cg_location.y() = cg_node.child("y").text().as_double(0.0) * factor;
            mb.cg_location.z() = cg_node.child("z").text().as_double(0.0) * factor;
        }

        // Inertia tensor
        if (auto inertia_node = mass_node.child("inertia")) {
            std::string unit = inertia_node.attribute("unit").as_string("");
            Real factor = 1.0;
            if (unit == "slug*ft2") factor = SLUG_FT2_TO_KG_M2;

            mb.inertia(0, 0) = inertia_node.child("ixx").text().as_double(1.0) * factor;
            mb.inertia(1, 1) = inertia_node.child("iyy").text().as_double(1.0) * factor;
            mb.inertia(2, 2) = inertia_node.child("izz").text().as_double(1.0) * factor;
            mb.inertia(0, 2) = inertia_node.child("ixz").text().as_double(0.0) * factor;
            mb.inertia(2, 0) = mb.inertia(0, 2);  // Symmetric
        }
    }

    // Parse aerodynamics
    if (auto aero_node = entity_node.child("aerodynamics")) {
        AerodynamicsData aero;

        for (auto axis_node : aero_node.children("axis")) {
            std::string axis_name = axis_node.attribute("name").as_string("");

            for (auto func_node : axis_node.children("function")) {
                AeroCoefficient coef;
                coef.name = func_node.attribute("name").as_string("");

                if (auto table_node = func_node.child("table")) {
                    if (auto data_node = table_node.child("tableData")) {
                        std::string data_str = data_node.text().as_string("");
                        std::istringstream iss(data_str);
                        Real bp, val;
                        while (iss >> bp >> val) {
                            coef.breakpoints.push_back(bp);
                            coef.values.push_back(val);
                        }
                    }
                }

                // Categorize by axis type
                if (axis_name == "LIFT") {
                    aero.lift_tables.push_back(std::move(coef));
                } else if (axis_name == "DRAG") {
                    aero.drag_tables.push_back(std::move(coef));
                } else {
                    aero.moment_tables.push_back(std::move(coef));
                }
            }
        }

        def.aerodynamics = std::move(aero);
    }

    // Parse propulsion
    if (auto prop_node = entity_node.child("propulsion")) {
        PropulsionData prop;

        if (auto engine_node = prop_node.child("engine")) {
            prop.engine_type = engine_node.attribute("type").as_string("");

            auto get_thrust = [&](const char* name) -> Real {
                auto node = engine_node.child(name);
                if (!node) return 0.0;
                Real value = node.text().as_double(0.0);
                std::string unit = node.attribute("unit").as_string("");
                if (unit == "lbf") value *= LBF_TO_N;
                return value;
            };

            prop.max_thrust = get_thrust("max_thrust");
            prop.afterburner_thrust = get_thrust("afterburner_thrust");
        }

        def.propulsion = std::move(prop);
    }

#else
    // Minimal parsing without pugixml using regex
    std::regex entity_regex(R"(<entity\s+type=\"([^\"]+)\"\s+name=\"([^\"]+)\")");
    std::smatch match;

    if (std::regex_search(xml, match, entity_regex)) {
        def.type = match[1].str();
        def.name = match[2].str();
        def.domain = parse_domain(def.type);
    } else {
        error_message_ = "Could not parse entity attributes";
        throw std::runtime_error(error_message_);
    }

    // Set default values for minimal implementation
    def.mass_balance.empty_mass = 1000.0;  // 1000 kg default
    def.mass_balance.inertia = Mat3x3::Identity() * 1000.0;
#endif

    // Determine component mask based on parsed data
    def.component_mask = determine_components(def);

    return def;
}

Domain XmlEntityLoader::parse_domain(const std::string& type) {
    if (type == "aircraft" || type == "missile" || type == "uav") {
        return Domain::Air;
    } else if (type == "vehicle" || type == "tank" || type == "truck") {
        return Domain::Land;
    } else if (type == "vessel" || type == "ship" || type == "submarine") {
        return Domain::Sea;
    } else if (type == "spacecraft" || type == "satellite") {
        return Domain::Space;
    }
    return Domain::Generic;
}

EntityMetrics XmlEntityLoader::parse_metrics(void* /*node*/) {
    // Implemented inline in load_from_string with pugixml
    return {};
}

MassBalance XmlEntityLoader::parse_mass_balance(void* /*node*/) {
    // Implemented inline in load_from_string with pugixml
    return {};
}

AerodynamicsData XmlEntityLoader::parse_aerodynamics(void* /*node*/) {
    // Implemented inline in load_from_string with pugixml
    return {};
}

PropulsionData XmlEntityLoader::parse_propulsion(void* /*node*/) {
    // Implemented inline in load_from_string with pugixml
    return {};
}

ComponentMask XmlEntityLoader::determine_components(const EntityDefinition& def) {
    ComponentMask mask = 0;

    // Add gravity for all entities
    mask |= physics::ComponentBits::Gravity;

    if (def.aerodynamics.has_value()) {
        mask |= physics::ComponentBits::Aerodynamics;
    }

    if (def.propulsion.has_value()) {
        mask |= physics::ComponentBits::Propulsion;
    }

    // Domain-specific components
    switch (def.domain) {
        case Domain::Air:
            mask |= physics::ComponentBits::FlightControl;
            break;
        case Domain::Land:
            mask |= physics::ComponentBits::GroundContact;
            mask |= physics::ComponentBits::Suspension;
            break;
        case Domain::Sea:
            mask |= physics::ComponentBits::Hydrodynamics;
            mask |= physics::ComponentBits::Buoyancy;
            break;
        case Domain::Space:
            mask |= physics::ComponentBits::OrbitalDynamics;
            break;
        default:
            break;
    }

    return mask;
}

} // namespace jaguar::interface
