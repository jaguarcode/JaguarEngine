/**
 * @file terrain_manager.cpp
 * @brief Terrain manager implementation
 */

#include "jaguar/environment/terrain.h"
#include "jaguar/core/coordinates.h"
#include <shared_mutex>
#include <cmath>

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

    // GDAL terrain loader (optional)
    GDALTerrainLoader gdal_loader;
    bool terrain_loaded{false};

    // Thread safety for parallel queries
    mutable std::shared_mutex terrain_mutex;

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

bool TerrainManager::load_terrain(const std::string& path)
{
    std::unique_lock lock(impl_->terrain_mutex);

    // Try to open the terrain file
    if (impl_->gdal_loader.open(path)) {
        impl_->terrain_loaded = true;
        return true;
    }

    return false;
}

bool TerrainManager::is_terrain_loaded() const
{
    std::shared_lock lock(impl_->terrain_mutex);
    return impl_->terrain_loaded;
}

void TerrainManager::set_cache_size(SizeT mb)
{
    impl_->cache_size_mb = mb;
}

bool TerrainManager::initialize()
{
    // Load terrain data from configured paths if available
    for (const auto& path : impl_->data_paths) {
        if (impl_->gdal_loader.open(path)) {
            impl_->terrain_loaded = true;
            break; // Use first successful terrain file
        }
    }

    impl_->initialized = true;
    return true;
}

void TerrainManager::shutdown()
{
    std::unique_lock lock(impl_->terrain_mutex);
    impl_->gdal_loader.close();
    impl_->terrain_loaded = false;
    impl_->initialized = false;
}

Real TerrainManager::get_elevation(Real lat, Real lon) const
{
    std::shared_lock lock(impl_->terrain_mutex);

    if (impl_->terrain_loaded) {
        Real elevation = impl_->gdal_loader.get_elevation(lat, lon);
        // If elevation is valid (not NaN), return it
        if (!std::isnan(elevation)) {
            return elevation;
        }
    }

    // Fall back to default flat terrain
    return impl_->default_elevation;
}

Real TerrainManager::get_elevation_ecef(const Vec3& ecef) const
{
    // Convert ECEF to geodetic
    GeodeticPosition lla = coord::ecef_to_lla(ecef);
    return get_elevation(lla.latitude, lla.longitude);
}

Vec3 TerrainManager::get_surface_normal(Real lat, Real lon) const
{
    std::shared_lock lock(impl_->terrain_mutex);

    if (impl_->terrain_loaded) {
        Vec3 normal = impl_->gdal_loader.get_normal(lat, lon);
        // If normal is valid (not default), return it
        if (normal.x != 0.0 || normal.y != 0.0 || normal.z != 1.0) {
            return normal;
        }
    }

    // Flat terrain has upward normal
    return Vec3{0.0, 0.0, 1.0};
}

Real TerrainManager::get_slope_angle(Real lat, Real lon) const
{
    std::shared_lock lock(impl_->terrain_mutex);

    if (impl_->terrain_loaded) {
        // Get the normal and compute slope from it
        Vec3 normal = impl_->gdal_loader.get_normal(lat, lon);
        // Slope angle is angle between normal and vertical (0,0,1)
        Real cos_slope = normal.z;  // Dot product with (0,0,1)
        cos_slope = std::max(-1.0, std::min(1.0, cos_slope));  // Clamp
        return std::acos(cos_slope);
    }

    return 0.0;
}

TerrainMaterial TerrainManager::get_material([[maybe_unused]] Real lat, [[maybe_unused]] Real lon) const
{
    return impl_->default_material;
}

TerrainQuery TerrainManager::query(Real lat, Real lon) const
{
    std::shared_lock lock(impl_->terrain_mutex);

    if (impl_->terrain_loaded) {
        TerrainQuery result = impl_->gdal_loader.query(lat, lon);
        if (result.valid) {
            // Add material information (not in GDAL data)
            result.material = impl_->default_material;
            return result;
        }
    }

    // Fall back to default flat terrain
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

} // namespace jaguar::environment
