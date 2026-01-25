#pragma once
/**
 * @file environment.h
 * @brief Combined environment state for physics calculations
 */

#include "jaguar/core/types.h"
#include "jaguar/environment/terrain.h"
#include "jaguar/environment/atmosphere.h"
#include "jaguar/environment/ocean.h"

namespace jaguar::environment {

/**
 * @brief Combined environment state at a point
 *
 * This is the environment data passed to force generators
 * during physics calculations.
 */
struct Environment {
    // Position context
    Real latitude{0.0};           ///< rad
    Real longitude{0.0};          ///< rad
    Real altitude{0.0};           ///< m above WGS84
    Vec3 position_ecef{0, 0, 0};  ///< ECEF position

    // Atmosphere
    AtmosphereState atmosphere;

    // Terrain (if applicable)
    TerrainQuery terrain;
    Real terrain_elevation{0.0};  ///< Terrain height below entity

    // Ocean (if applicable)
    OceanState ocean;
    bool over_water{false};

    // Gravity
    Vec3 gravity{0, 0, 9.80665};  ///< Gravity vector in local frame
};

/**
 * @brief Environment service provider
 *
 * Central service that provides environment data to all physics systems.
 */
class EnvironmentService {
public:
    EnvironmentService();
    ~EnvironmentService();

    /**
     * @brief Initialize all environment subsystems
     */
    bool initialize();

    /**
     * @brief Shutdown all subsystems
     */
    void shutdown();

    /**
     * @brief Update environment (call each frame)
     */
    void update(Real dt);

    /**
     * @brief Query environment at position
     * @param ecef Position in ECEF coordinates
     * @param time Current simulation time
     */
    Environment query(const Vec3& ecef, Real time) const;

    /**
     * @brief Query environment at geodetic position
     */
    Environment query_geodetic(Real lat, Real lon, Real alt, Real time) const;

    /**
     * @brief Load terrain data from file
     * @param path Path to GeoTIFF or other GDAL-supported raster
     * @return true if successfully loaded
     */
    bool load_terrain(const std::string& path);

    /**
     * @brief Query terrain at ECEF position
     * @param position_ecef Position in ECEF coordinates
     * @return Terrain query with elevation and surface properties
     */
    TerrainQuery query_terrain(const Vec3& position_ecef) const;

    /**
     * @brief Get terrain elevation at geodetic position
     * @param lat Latitude (radians)
     * @param lon Longitude (radians)
     * @return Elevation in meters above WGS84 ellipsoid
     */
    Real get_terrain_elevation(Real lat, Real lon) const;

    // Subsystem access
    TerrainManager& terrain() { return terrain_; }
    const TerrainManager& terrain() const { return terrain_; }

    AtmosphereManager& atmosphere() { return atmosphere_; }
    const AtmosphereManager& atmosphere() const { return atmosphere_; }

    OceanManager& ocean() { return ocean_; }
    const OceanManager& ocean() const { return ocean_; }

    /**
     * @brief Set simulation time reference
     */
    void set_time(Real time) { current_time_ = time; }

private:
    TerrainManager terrain_;
    AtmosphereManager atmosphere_;
    OceanManager ocean_;
    Real current_time_{0.0};
};

} // namespace jaguar::environment
