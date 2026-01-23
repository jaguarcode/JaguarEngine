/**
 * @file config_loader.cpp
 * @brief XML configuration loading implementation
 *
 * Implements JSBSim-style XML configuration loading using pugixml.
 * Supports engine configuration, entity definitions, and soil databases.
 */

#include "jaguar/interface/config.h"
#include "jaguar/physics/entity.h"
#include <pugixml.hpp>
#include <filesystem>
#include <stdexcept>
#define _USE_MATH_DEFINES
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace jaguar::config {

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

    // Pressure conversions to Pa
    if (unit == "kPa") return value * 1000.0;
    if (unit == "Pa") return value;
    if (unit == "psi") return value * 6894.757;

    // Spring/damper coefficients
    if (unit == "lbs/ft") return value * 14.593903;
    if (unit == "lbs/ft/sec") return value * 14.593903;
    if (unit == "N/m") return value;
    if (unit == "N*s/m") return value;

    // Angle conversions to radians
    if (unit == "deg") return value * M_PI / 180.0;
    if (unit == "rad") return value;

    // Bekker soil parameters
    if (unit == "kN/m^(n+1)") return value * 1000.0;
    if (unit == "kN/m^(n+2)") return value * 1000.0;

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

Domain parse_domain(const std::string& type) {
    if (type == "aircraft" || type == "air") return Domain::Air;
    if (type == "vehicle" || type == "land") return Domain::Land;
    if (type == "ship" || type == "sea" || type == "submarine" || type == "subsurface") return Domain::Sea;
    if (type == "satellite" || type == "space") return Domain::Space;
    return Domain::Generic;
}

} // anonymous namespace

// ============================================================================
// EngineConfig Implementation
// ============================================================================

EngineConfig EngineConfig::load(const std::string& path) {
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(path.c_str());

    if (!result) {
        throw std::runtime_error("Failed to load config: " + std::string(result.description()));
    }

    EngineConfig config = defaults();

    auto root = doc.child("engine_config");
    if (!root) {
        root = doc.child("config");
    }
    if (!root) {
        throw std::runtime_error("Invalid engine config XML: no root element");
    }

    // Simulation settings
    if (auto sim = root.child("simulation")) {
        config.time_step = sim.child("time_step").text().as_double(config.time_step);
        config.max_entities = static_cast<SizeT>(sim.child("max_entities").text().as_uint(
            static_cast<unsigned int>(config.max_entities)));

        std::string frame = sim.child("coordinate_frame").text().as_string("ECEF");
        if (frame == "NED") config.coordinate_frame = CoordinateFrame::NED;
        else if (frame == "ENU") config.coordinate_frame = CoordinateFrame::ENU;
        else config.coordinate_frame = CoordinateFrame::ECEF;
    }

    // Threading settings
    if (auto threading = root.child("threading")) {
        config.physics_threads = threading.child("physics_threads").text().as_int(config.physics_threads);
        config.io_threads = threading.child("io_threads").text().as_int(config.io_threads);
        config.work_stealing = threading.child("work_stealing").text().as_bool(config.work_stealing);
    }

    // Terrain settings
    if (auto terrain = root.child("terrain")) {
        config.terrain_enabled = terrain.child("enabled").text().as_bool(config.terrain_enabled);
        config.terrain_cache_mb = static_cast<SizeT>(terrain.child("cache_mb").text().as_uint(
            static_cast<unsigned int>(config.terrain_cache_mb)));
        config.terrain_tile_size = terrain.child("tile_size").text().as_int(config.terrain_tile_size);

        for (auto path_node : terrain.children("data_path")) {
            config.terrain_data_paths.push_back(path_node.text().as_string());
        }
    }

    // Atmosphere settings
    if (auto atmos = root.child("atmosphere")) {
        config.atmosphere_model = atmos.child("model").text().as_string(config.atmosphere_model.c_str());
        config.weather_enabled = atmos.child("weather_enabled").text().as_bool(config.weather_enabled);
    }

    // Network settings
    if (auto network = root.child("network")) {
        if (auto dis = network.child("dis")) {
            config.dis_enabled = dis.child("enabled").text().as_bool(config.dis_enabled);
            config.dis_port = dis.child("port").text().as_int(config.dis_port);
            config.dis_site_id = dis.child("site_id").text().as_int(config.dis_site_id);
            config.dis_app_id = dis.child("app_id").text().as_int(config.dis_app_id);
        }

        if (auto hla = network.child("hla")) {
            config.hla_enabled = hla.child("enabled").text().as_bool(config.hla_enabled);
            config.hla_federation = hla.child("federation").text().as_string();
            config.hla_federate = hla.child("federate").text().as_string();
        }
    }

    return config;
}

EngineConfig EngineConfig::defaults() {
    return EngineConfig{};
}

bool EngineConfig::save(const std::string& path) const {
    pugi::xml_document doc;

    auto decl = doc.prepend_child(pugi::node_declaration);
    decl.append_attribute("version") = "1.0";
    decl.append_attribute("encoding") = "UTF-8";

    auto root = doc.append_child("engine_config");

    // Simulation settings
    auto sim = root.append_child("simulation");
    sim.append_child("time_step").text().set(time_step);
    sim.append_child("max_entities").text().set(static_cast<unsigned int>(max_entities));

    const char* frame_str = "ECEF";
    if (coordinate_frame == CoordinateFrame::NED) frame_str = "NED";
    else if (coordinate_frame == CoordinateFrame::ENU) frame_str = "ENU";
    sim.append_child("coordinate_frame").text().set(frame_str);

    // Threading settings
    auto threading = root.append_child("threading");
    threading.append_child("physics_threads").text().set(physics_threads);
    threading.append_child("io_threads").text().set(io_threads);
    threading.append_child("work_stealing").text().set(work_stealing);

    // Terrain settings
    auto terrain = root.append_child("terrain");
    terrain.append_child("enabled").text().set(terrain_enabled);
    terrain.append_child("cache_mb").text().set(static_cast<unsigned int>(terrain_cache_mb));
    terrain.append_child("tile_size").text().set(terrain_tile_size);
    for (const auto& data_path : terrain_data_paths) {
        terrain.append_child("data_path").text().set(data_path.c_str());
    }

    // Atmosphere settings
    auto atmos = root.append_child("atmosphere");
    atmos.append_child("model").text().set(atmosphere_model.c_str());
    atmos.append_child("weather_enabled").text().set(weather_enabled);

    // Network settings
    auto network = root.append_child("network");

    auto dis = network.append_child("dis");
    dis.append_child("enabled").text().set(dis_enabled);
    dis.append_child("port").text().set(dis_port);
    dis.append_child("site_id").text().set(dis_site_id);
    dis.append_child("app_id").text().set(dis_app_id);

    auto hla = network.append_child("hla");
    hla.append_child("enabled").text().set(hla_enabled);
    hla.append_child("federation").text().set(hla_federation.c_str());
    hla.append_child("federate").text().set(hla_federate.c_str());

    return doc.save_file(path.c_str());
}

// ============================================================================
// EntityConfig Implementation
// ============================================================================

EntityConfig EntityConfig::load(const std::string& path) {
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(path.c_str());

    if (!result) {
        throw std::runtime_error("Failed to load entity config: " + std::string(result.description()));
    }

    auto root = doc.child("entity");
    if (!root) {
        throw std::runtime_error("Invalid entity config XML: no <entity> root element");
    }

    EntityConfig config;

    // Basic info
    config.name = root.attribute("name").as_string("unnamed");
    config.domain = parse_domain(root.attribute("type").as_string("generic"));

    // Metrics (geometry)
    if (auto metrics = root.child("metrics")) {
        config.wingspan = parse_value_with_unit(metrics.child("wingspan"));
        config.length = parse_value_with_unit(metrics.child("length"));
        config.height = parse_value_with_unit(metrics.child("height"));
        config.reference_area = parse_value_with_unit(metrics.child("wing_area"));
        config.reference_chord = parse_value_with_unit(metrics.child("wing_chord"));
    }

    // Mass balance
    if (auto mass = root.child("mass_balance")) {
        config.empty_mass = parse_value_with_unit(mass.child("empty_weight"));
        config.fuel_capacity = parse_value_with_unit(mass.child("fuel_capacity"));

        if (auto cg = mass.child("center_of_gravity")) {
            config.cg_location = parse_vec3(cg);
        }

        if (auto inertia_node = mass.child("inertia")) {
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

            config.inertia = Mat3x3::Inertia(ixx, iyy, izz, ixy, ixz, iyz);
        }
    }

    // Component flags based on sections present
    if (root.child("aerodynamics")) {
        config.components |= physics::ComponentBits::Aerodynamics;
    }
    if (root.child("propulsion")) {
        config.components |= physics::ComponentBits::Propulsion;
    }
    if (root.child("ground_reactions")) {
        config.components |= physics::ComponentBits::Terramechanics;
    }
    if (root.child("buoyancy") || root.child("hydrodynamics")) {
        config.components |= physics::ComponentBits::Hydrodynamics;
        config.components |= physics::ComponentBits::Buoyancy;
    }

    // Always add gravity
    config.components |= physics::ComponentBits::Gravity;

    return config;
}

// ============================================================================
// ConfigLoader Implementation
// ============================================================================

ConfigLoader::ConfigLoader() {
    // Add default search paths
    search_paths_.push_back(".");
    search_paths_.push_back("./data");
    search_paths_.push_back("./config");
}

ConfigLoader::~ConfigLoader() = default;

EngineConfig ConfigLoader::load_engine_config(const std::string& path) {
    std::string resolved = find_file(path);
    if (resolved.empty()) {
        throw std::runtime_error("Engine config file not found: " + path);
    }
    return EngineConfig::load(resolved);
}

EntityConfig ConfigLoader::load_entity_config(const std::string& path) {
    std::string resolved = find_file(path);
    if (resolved.empty()) {
        throw std::runtime_error("Entity config file not found: " + path);
    }
    return EntityConfig::load(resolved);
}

void ConfigLoader::add_search_path(const std::string& path) {
    search_paths_.push_back(path);
}

std::string ConfigLoader::find_file(const std::string& filename) const {
    // Check if it's already an absolute path that exists
    if (std::filesystem::exists(filename)) {
        return filename;
    }

    // Search in all paths
    for (const auto& search_path : search_paths_) {
        std::filesystem::path full_path = std::filesystem::path(search_path) / filename;
        if (std::filesystem::exists(full_path)) {
            return full_path.string();
        }
    }

    return "";
}

} // namespace jaguar::config
