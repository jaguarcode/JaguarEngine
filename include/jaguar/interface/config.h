#pragma once
/**
 * @file config.h
 * @brief Configuration loading and management
 */

#include "jaguar/core/types.h"
#include <string>
#include <vector>
#include <memory>

namespace jaguar::config {

/**
 * @brief Engine configuration loaded from XML
 */
struct EngineConfig {
    // Simulation settings
    Real time_step{0.01};
    SizeT max_entities{10000};
    CoordinateFrame coordinate_frame{CoordinateFrame::ECEF};

    // Threading
    int physics_threads{-1};  // -1 = auto
    int io_threads{2};
    bool work_stealing{true};

    // Terrain
    bool terrain_enabled{true};
    SizeT terrain_cache_mb{2048};
    int terrain_tile_size{256};
    std::vector<std::string> terrain_data_paths;

    // Atmosphere
    std::string atmosphere_model{"us_standard_1976"};
    bool weather_enabled{true};

    // Network
    bool dis_enabled{false};
    int dis_port{3000};
    int dis_site_id{1};
    int dis_app_id{1};

    bool hla_enabled{false};
    std::string hla_federation;
    std::string hla_federate;

    /**
     * @brief Load configuration from XML file
     */
    static EngineConfig load(const std::string& path);

    /**
     * @brief Create default configuration
     */
    static EngineConfig defaults();

    /**
     * @brief Save configuration to XML file
     */
    bool save(const std::string& path) const;
};

/**
 * @brief Entity configuration loaded from XML
 */
struct EntityConfig {
    std::string name;
    Domain domain{Domain::Generic};

    // Mass properties
    Real empty_mass{0.0};
    Real fuel_capacity{0.0};
    Vec3 cg_location{0, 0, 0};
    Mat3x3 inertia{};

    // Geometry
    Real wingspan{0.0};
    Real length{0.0};
    Real height{0.0};
    Real reference_area{0.0};
    Real reference_chord{0.0};

    // Component flags
    ComponentMask components{0};

    /**
     * @brief Load from XML file
     */
    static EntityConfig load(const std::string& path);
};

/**
 * @brief Configuration loader service
 */
class ConfigLoader {
public:
    ConfigLoader();
    ~ConfigLoader();

    /**
     * @brief Load engine configuration
     */
    EngineConfig load_engine_config(const std::string& path);

    /**
     * @brief Load entity configuration
     */
    EntityConfig load_entity_config(const std::string& path);

    /**
     * @brief Add search path for configuration files
     */
    void add_search_path(const std::string& path);

    /**
     * @brief Find file in search paths
     */
    std::string find_file(const std::string& filename) const;

private:
    std::vector<std::string> search_paths_;
};

} // namespace jaguar::config
