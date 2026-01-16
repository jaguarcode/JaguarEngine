#pragma once
/**
 * @file terrain.h
 * @brief Terrain management and GIS data handling
 */

#include "jaguar/core/types.h"
#include <string>
#include <memory>
#include <functional>

namespace jaguar::environment {

// ============================================================================
// Terrain Data Types
// ============================================================================

/**
 * @brief Soil/surface material types
 */
enum class SurfaceType : UInt8 {
    Unknown,
    Asphalt,
    Concrete,
    Gravel,
    DrySand,
    WetSand,
    Clay,
    Mud,
    Grass,
    Snow,
    Ice,
    Water
};

/**
 * @brief Terrain material properties
 */
struct TerrainMaterial {
    SurfaceType type{SurfaceType::Unknown};
    Real friction_coefficient{0.7};
    Real rolling_resistance{0.01};

    // Bekker-Wong parameters
    Real k_c{0.0};      ///< Cohesive modulus
    Real k_phi{0.0};    ///< Frictional modulus
    Real n{1.0};        ///< Deformation exponent

    // Trafficability
    Real max_slope_deg{30.0};
    Real bearing_capacity{100000.0};  // Pa
};

/**
 * @brief Terrain query result
 */
struct TerrainQuery {
    Real elevation{0.0};        ///< Height above WGS84 ellipsoid (m)
    Vec3 normal{0, 0, 1};       ///< Surface normal (NED)
    Real slope_angle{0.0};      ///< Slope angle (rad)
    TerrainMaterial material;
    bool valid{false};
};

// ============================================================================
// Terrain Manager
// ============================================================================

/**
 * @brief Central terrain data manager with async loading
 */
class TerrainManager {
public:
    TerrainManager();
    ~TerrainManager();

    // Non-copyable
    TerrainManager(const TerrainManager&) = delete;
    TerrainManager& operator=(const TerrainManager&) = delete;

    // ========================================================================
    // Configuration
    // ========================================================================

    /**
     * @brief Add data source path (DTED, GeoTIFF directory)
     */
    void add_data_path(const std::string& path);

    /**
     * @brief Set cache size in MB
     */
    void set_cache_size(SizeT mb);

    /**
     * @brief Initialize terrain system
     */
    bool initialize();

    /**
     * @brief Shutdown and release resources
     */
    void shutdown();

    // ========================================================================
    // Queries
    // ========================================================================

    /**
     * @brief Get elevation at geodetic position
     * @param lat Latitude (rad)
     * @param lon Longitude (rad)
     * @return Elevation in meters, or NaN if unavailable
     */
    Real get_elevation(Real lat, Real lon) const;

    /**
     * @brief Get elevation at ECEF position
     */
    Real get_elevation_ecef(const Vec3& ecef) const;

    /**
     * @brief Get surface normal at position
     */
    Vec3 get_surface_normal(Real lat, Real lon) const;

    /**
     * @brief Get slope angle at position
     */
    Real get_slope_angle(Real lat, Real lon) const;

    /**
     * @brief Get material at position
     */
    TerrainMaterial get_material(Real lat, Real lon) const;

    /**
     * @brief Complete terrain query
     */
    TerrainQuery query(Real lat, Real lon) const;

    /**
     * @brief Query terrain at ECEF position
     */
    TerrainQuery query_ecef(const Vec3& ecef) const;

    // ========================================================================
    // LOD Management
    // ========================================================================

    /**
     * @brief Set focus point for LOD management
     */
    void set_focus_point(const Vec3& ecef);

    /**
     * @brief Set detail radius around focus
     */
    void set_detail_radius(Real radius_m);

    /**
     * @brief Update terrain paging (call each frame)
     */
    void update();

    // ========================================================================
    // Statistics
    // ========================================================================

    /**
     * @brief Get number of loaded tiles
     */
    SizeT loaded_tile_count() const;

    /**
     * @brief Get cache memory usage
     */
    SizeT cache_usage_bytes() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// ============================================================================
// GDAL Data Loader
// ============================================================================

/**
 * @brief GDAL-based terrain data loader
 */
class GDALTerrainLoader {
public:
    GDALTerrainLoader();
    ~GDALTerrainLoader();

    /**
     * @brief Check if GDAL is available
     */
    static bool is_available();

    /**
     * @brief Open terrain dataset
     */
    bool open(const std::string& path);

    /**
     * @brief Close dataset
     */
    void close();

    /**
     * @brief Get elevation at position
     */
    Real get_elevation(Real lat, Real lon) const;

    /**
     * @brief Get dataset bounds
     */
    void get_bounds(Real& min_lat, Real& max_lat, Real& min_lon, Real& max_lon) const;

    /**
     * @brief Get resolution in degrees
     */
    Real get_resolution() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace jaguar::environment
