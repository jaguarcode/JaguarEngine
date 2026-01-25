#pragma once
/**
 * @file xml_entity_loader.h
 * @brief XML entity definition loader
 */

#include "jaguar/core/types.h"
#include "jaguar/physics/entity.h"
#include <string>
#include <vector>
#include <optional>

namespace jaguar::interface {

/**
 * @brief Parsed entity metrics from XML
 */
struct EntityMetrics {
    Real wingspan{0.0};      // m
    Real length{0.0};        // m
    Real height{0.0};        // m
    Real wing_area{0.0};     // m²
    Real reference_area{0.0}; // m²
};

/**
 * @brief Parsed mass/inertia from XML
 */
struct MassBalance {
    Real empty_mass{0.0};    // kg
    Real fuel_mass{0.0};     // kg
    Vec3 cg_location{};      // m (body frame)
    Mat3x3 inertia{};        // kg·m²
};

/**
 * @brief Aerodynamic coefficient table entry
 */
struct AeroCoefficient {
    std::string name;
    std::vector<Real> breakpoints;
    std::vector<Real> values;
};

/**
 * @brief Parsed aerodynamics data
 */
struct AerodynamicsData {
    std::vector<AeroCoefficient> lift_tables;
    std::vector<AeroCoefficient> drag_tables;
    std::vector<AeroCoefficient> moment_tables;
};

/**
 * @brief Parsed propulsion data
 */
struct PropulsionData {
    std::string engine_type;
    Real max_thrust{0.0};     // N
    Real afterburner_thrust{0.0}; // N
    Vec3 thrust_location{};   // m (body frame)
};

/**
 * @brief Complete entity definition from XML
 */
struct EntityDefinition {
    std::string name;
    std::string type;          // "aircraft", "vehicle", "vessel", etc.
    Domain domain{Domain::Generic};

    EntityMetrics metrics;
    MassBalance mass_balance;

    std::optional<AerodynamicsData> aerodynamics;
    std::optional<PropulsionData> propulsion;

    ComponentMask component_mask{0};
};

/**
 * @brief XML entity definition loader
 */
class XmlEntityLoader {
public:
    XmlEntityLoader() = default;
    ~XmlEntityLoader() = default;

    /**
     * @brief Load entity definition from XML file
     * @param path Path to XML file
     * @return Parsed entity definition
     * @throws std::runtime_error on parse failure
     */
    EntityDefinition load(const std::string& path);

    /**
     * @brief Load entity definition from XML string
     * @param xml XML content
     * @return Parsed entity definition
     */
    EntityDefinition load_from_string(const std::string& xml);

    /**
     * @brief Get last error message
     */
    const std::string& get_error() const { return error_message_; }

private:
    std::string error_message_;

    Domain parse_domain(const std::string& type);
    EntityMetrics parse_metrics(void* node);  // pugi::xml_node
    MassBalance parse_mass_balance(void* node);
    AerodynamicsData parse_aerodynamics(void* node);
    PropulsionData parse_propulsion(void* node);
    ComponentMask determine_components(const EntityDefinition& def);
};

} // namespace jaguar::interface
