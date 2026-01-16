/**
 * @file terrain_manager.cpp
 * @brief Terrain manager implementation
 */

#include "jaguar/environment/terrain.h"
#include "jaguar/core/coordinates.h"

namespace jaguar::environment {

// ============================================================================
// TerrainManager Implementation
// ============================================================================

struct TerrainManager::Impl {
    std::vector<std::string> data_paths;
    SizeT cache_size_mb{256};
    bool initialized{false};

    // Focus point for LOD
    Vec3 focus_ecef{0, 0, 0};
    Real detail_radius{10000.0};

    // Default flat terrain (no data loaded)
    Real default_elevation{0.0};
    TerrainMaterial default_material;

    Impl()
    {
        // Set default material to grass
        default_material.type = SurfaceType::Grass;
        default_material.friction_coefficient = 0.6;
        default_material.rolling_resistance = 0.02;
    }

    TerrainQuery query_default([[maybe_unused]] Real lat, [[maybe_unused]] Real lon) const
    {
        TerrainQuery result;
        result.elevation = default_elevation;
        result.normal = Vec3{0.0, 0.0, 1.0};
        result.slope_angle = 0.0;
        result.material = default_material;
        result.valid = true;
        return result;
    }
};

TerrainManager::TerrainManager()
    : impl_(std::make_unique<Impl>()) {}

TerrainManager::~TerrainManager() = default;

void TerrainManager::add_data_path(const std::string& path)
{
    impl_->data_paths.push_back(path);
}

void TerrainManager::set_cache_size(SizeT mb)
{
    impl_->cache_size_mb = mb;
}

bool TerrainManager::initialize()
{
    // In a full implementation, this would:
    // 1. Scan data paths for terrain data
    // 2. Build spatial index
    // 3. Initialize cache
    impl_->initialized = true;
    return true;
}

void TerrainManager::shutdown()
{
    impl_->initialized = false;
}

Real TerrainManager::get_elevation(Real lat, Real lon) const
{
    // For now, return flat terrain
    // Full implementation would query loaded terrain data
    (void)lat;
    (void)lon;
    return impl_->default_elevation;
}

Real TerrainManager::get_elevation_ecef(const Vec3& ecef) const
{
    // Convert ECEF to geodetic
    GeodeticPosition lla = coord::ecef_to_lla(ecef);
    return get_elevation(lla.latitude, lla.longitude);
}

Vec3 TerrainManager::get_surface_normal([[maybe_unused]] Real lat, [[maybe_unused]] Real lon) const
{
    // Flat terrain has upward normal
    return Vec3{0.0, 0.0, 1.0};
}

Real TerrainManager::get_slope_angle([[maybe_unused]] Real lat, [[maybe_unused]] Real lon) const
{
    return 0.0;
}

TerrainMaterial TerrainManager::get_material([[maybe_unused]] Real lat, [[maybe_unused]] Real lon) const
{
    return impl_->default_material;
}

TerrainQuery TerrainManager::query(Real lat, Real lon) const
{
    return impl_->query_default(lat, lon);
}

TerrainQuery TerrainManager::query_ecef(const Vec3& ecef) const
{
    GeodeticPosition lla = coord::ecef_to_lla(ecef);
    return query(lla.latitude, lla.longitude);
}

void TerrainManager::set_focus_point(const Vec3& ecef)
{
    impl_->focus_ecef = ecef;
}

void TerrainManager::set_detail_radius(Real radius_m)
{
    impl_->detail_radius = radius_m;
}

void TerrainManager::update()
{
    // In a full implementation, this would:
    // 1. Check what tiles are needed around focus point
    // 2. Load/unload tiles as necessary
    // 3. Manage cache
}

SizeT TerrainManager::loaded_tile_count() const
{
    return 0;  // No tiles loaded in stub
}

SizeT TerrainManager::cache_usage_bytes() const
{
    return 0;
}

// ============================================================================
// GDALTerrainLoader Implementation (Stub)
// ============================================================================

struct GDALTerrainLoader::Impl {
    bool open{false};
    Real min_lat{0}, max_lat{0};
    Real min_lon{0}, max_lon{0};
    Real resolution{0};
};

GDALTerrainLoader::GDALTerrainLoader()
    : impl_(std::make_unique<Impl>()) {}

GDALTerrainLoader::~GDALTerrainLoader() = default;

bool GDALTerrainLoader::is_available()
{
    // GDAL not linked in this build
    return false;
}

bool GDALTerrainLoader::open([[maybe_unused]] const std::string& path)
{
    // Would use GDAL to open GeoTIFF/DTED files
    return false;
}

void GDALTerrainLoader::close()
{
    impl_->open = false;
}

Real GDALTerrainLoader::get_elevation([[maybe_unused]] Real lat, [[maybe_unused]] Real lon) const
{
    return 0.0;
}

void GDALTerrainLoader::get_bounds(Real& min_lat, Real& max_lat,
                                    Real& min_lon, Real& max_lon) const
{
    min_lat = impl_->min_lat;
    max_lat = impl_->max_lat;
    min_lon = impl_->min_lon;
    max_lon = impl_->max_lon;
}

Real GDALTerrainLoader::get_resolution() const
{
    return impl_->resolution;
}

} // namespace jaguar::environment
