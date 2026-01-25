/**
 * @file gdal_loader.cpp
 * @brief GDAL terrain loader implementation with conditional compilation
 *
 * This file provides two implementations:
 *
 * 1. Full GDAL Implementation (JAGUAR_HAS_GDAL defined):
 *    - Loads GeoTIFF and other GDAL-supported raster formats
 *    - Automatic coordinate transformation from WGS84 to dataset CRS
 *    - Bilinear interpolation for sub-pixel elevation queries
 *    - Surface normal computation from elevation gradients
 *    - Thread-safe read operations via mutex
 *    - Handles no-data values and out-of-bounds queries
 *
 * 2. Stub Implementation (GDAL not available):
 *    - Returns NaN for elevation queries
 *    - Returns default up vector for normals
 *    - All operations are no-ops
 *    - Allows code to compile and link without GDAL
 *
 * The implementation uses conditional compilation to select the appropriate
 * version at build time based on GDAL availability.
 */

#include "jaguar/environment/terrain.h"
#include <cmath>
#include <mutex>
#include <limits>

#ifdef JAGUAR_HAS_GDAL
#include <gdal_priv.h>
#include <cpl_conv.h>
#include <ogr_spatialref.h>
#endif

namespace jaguar::environment {

// ============================================================================
// GDAL Implementation (when available)
// ============================================================================

#ifdef JAGUAR_HAS_GDAL

struct GDALTerrainLoader::Impl {
    GDALDataset* dataset{nullptr};
    GDALRasterBand* band{nullptr};

    // Dataset metadata
    double geo_transform[6]{0, 1, 0, 0, 0, 1};  // Default identity transform
    int width{0};
    int height{0};
    double no_data_value{-9999.0};
    bool has_no_data{false};

    // Spatial reference
    OGRSpatialReference src_srs;
    OGRSpatialReference wgs84_srs;
    OGRCoordinateTransformation* to_dataset_coords{nullptr};

    // Thread safety
    mutable std::mutex mutex;

    // GDAL initialization flag
    static bool gdal_initialized;

    static void ensure_gdal_initialized() {
        if (!gdal_initialized) {
            GDALAllRegister();
            gdal_initialized = true;
        }
    }

    ~Impl() {
        if (to_dataset_coords) {
            delete to_dataset_coords;
        }
        if (dataset) {
            GDALClose(dataset);
        }
    }

    // Convert WGS84 lat/lon to pixel coordinates
    bool geo_to_pixel(Real lat, Real lon, double& px, double& py) const {
        double geo_x = lon;
        double geo_y = lat;

        // Transform from WGS84 to dataset CRS if needed
        if (to_dataset_coords) {
            if (!to_dataset_coords->Transform(1, &geo_x, &geo_y)) {
                return false;
            }
        }

        // Apply inverse geotransform
        double det = geo_transform[1] * geo_transform[5] - geo_transform[2] * geo_transform[4];
        if (std::abs(det) < 1e-10) {
            return false;
        }

        double dx = geo_x - geo_transform[0];
        double dy = geo_y - geo_transform[3];

        px = (geo_transform[5] * dx - geo_transform[2] * dy) / det;
        py = (-geo_transform[4] * dx + geo_transform[1] * dy) / det;

        return true;
    }

    // Read elevation with bilinear interpolation
    Real read_elevation(double px, double py) const {
        // Check bounds
        if (px < 0 || px >= width - 1 || py < 0 || py >= height - 1) {
            return std::numeric_limits<Real>::quiet_NaN();
        }

        int x0 = static_cast<int>(std::floor(px));
        int y0 = static_cast<int>(std::floor(py));
        int x1 = x0 + 1;
        int y1 = y0 + 1;

        Real fx = static_cast<Real>(px - x0);
        Real fy = static_cast<Real>(py - y0);

        // Read 2x2 neighborhood
        float values[4];
        CPLErr err = band->RasterIO(
            GF_Read,
            x0, y0,
            2, 2,
            values,
            2, 2,
            GDT_Float32,
            0, 0
        );

        if (err != CE_None) {
            return std::numeric_limits<Real>::quiet_NaN();
        }

        // Check for no-data values
        for (int i = 0; i < 4; ++i) {
            if (has_no_data && std::abs(values[i] - no_data_value) < 0.01) {
                return std::numeric_limits<Real>::quiet_NaN();
            }
        }

        // Bilinear interpolation
        Real v00 = static_cast<Real>(values[0]);
        Real v10 = static_cast<Real>(values[1]);
        Real v01 = static_cast<Real>(values[2]);
        Real v11 = static_cast<Real>(values[3]);

        Real v0 = v00 * (1.0 - fx) + v10 * fx;
        Real v1 = v01 * (1.0 - fx) + v11 * fx;

        return v0 * (1.0 - fy) + v1 * fy;
    }
};

bool GDALTerrainLoader::Impl::gdal_initialized = false;

GDALTerrainLoader::GDALTerrainLoader()
    : impl_(std::make_unique<Impl>())
{
    Impl::ensure_gdal_initialized();

    // Setup WGS84 reference
    impl_->wgs84_srs.SetWellKnownGeogCS("WGS84");
}

GDALTerrainLoader::~GDALTerrainLoader() = default;

bool GDALTerrainLoader::is_available() {
    return true;
}

bool GDALTerrainLoader::open(const std::string& path) {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    // Close existing dataset
    if (impl_->dataset) {
        GDALClose(impl_->dataset);
        impl_->dataset = nullptr;
        impl_->band = nullptr;
    }

    if (impl_->to_dataset_coords) {
        delete impl_->to_dataset_coords;
        impl_->to_dataset_coords = nullptr;
    }

    // Open dataset
    impl_->dataset = static_cast<GDALDataset*>(
        GDALOpen(path.c_str(), GA_ReadOnly)
    );

    if (!impl_->dataset) {
        return false;
    }

    // Get raster band (first band)
    impl_->band = impl_->dataset->GetRasterBand(1);
    if (!impl_->band) {
        GDALClose(impl_->dataset);
        impl_->dataset = nullptr;
        return false;
    }

    // Get dimensions
    impl_->width = impl_->dataset->GetRasterXSize();
    impl_->height = impl_->dataset->GetRasterYSize();

    // Get geotransform
    if (impl_->dataset->GetGeoTransform(impl_->geo_transform) != CE_None) {
        // Use default identity transform
        impl_->geo_transform[0] = 0.0;  // top-left x
        impl_->geo_transform[1] = 1.0;  // w-e pixel resolution
        impl_->geo_transform[2] = 0.0;  // rotation, 0 if north up
        impl_->geo_transform[3] = 0.0;  // top-left y
        impl_->geo_transform[4] = 0.0;  // rotation, 0 if north up
        impl_->geo_transform[5] = -1.0; // n-s pixel resolution (negative)
    }

    // Get no-data value
    int has_no_data_int;
    impl_->no_data_value = impl_->band->GetNoDataValue(&has_no_data_int);
    impl_->has_no_data = (has_no_data_int != 0);

    // Get projection
    const char* proj_ref = impl_->dataset->GetProjectionRef();
    if (proj_ref && strlen(proj_ref) > 0) {
        impl_->src_srs.importFromWkt(proj_ref);

        // Create coordinate transformation if not already WGS84
        if (!impl_->src_srs.IsSame(&impl_->wgs84_srs)) {
            impl_->to_dataset_coords = OGRCreateCoordinateTransformation(
                &impl_->wgs84_srs,
                &impl_->src_srs
            );
        }
    }

    return true;
}

void GDALTerrainLoader::close() {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (impl_->to_dataset_coords) {
        delete impl_->to_dataset_coords;
        impl_->to_dataset_coords = nullptr;
    }

    if (impl_->dataset) {
        GDALClose(impl_->dataset);
        impl_->dataset = nullptr;
        impl_->band = nullptr;
    }
}

Real GDALTerrainLoader::get_elevation(Real lat, Real lon) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->dataset || !impl_->band) {
        return std::numeric_limits<Real>::quiet_NaN();
    }

    // Convert to pixel coordinates
    double px, py;
    if (!impl_->geo_to_pixel(lat, lon, px, py)) {
        return std::numeric_limits<Real>::quiet_NaN();
    }

    return impl_->read_elevation(px, py);
}

void GDALTerrainLoader::get_bounds(Real& min_lat, Real& max_lat,
                                    Real& min_lon, Real& max_lon) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->dataset) {
        min_lat = max_lat = min_lon = max_lon = 0.0;
        return;
    }

    // Get corners in pixel space
    double corners_x[4] = {0, static_cast<double>(impl_->width),
                           static_cast<double>(impl_->width), 0};
    double corners_y[4] = {0, 0,
                           static_cast<double>(impl_->height),
                           static_cast<double>(impl_->height)};

    // Apply geotransform to get geo coordinates
    for (int i = 0; i < 4; ++i) {
        double x = corners_x[i];
        double y = corners_y[i];

        corners_x[i] = impl_->geo_transform[0] +
                       x * impl_->geo_transform[1] +
                       y * impl_->geo_transform[2];
        corners_y[i] = impl_->geo_transform[3] +
                       x * impl_->geo_transform[4] +
                       y * impl_->geo_transform[5];
    }

    // Transform to WGS84 if needed
    if (impl_->to_dataset_coords) {
        OGRCoordinateTransformation* from_dataset = OGRCreateCoordinateTransformation(
            &impl_->src_srs,
            &impl_->wgs84_srs
        );

        if (from_dataset) {
            from_dataset->Transform(4, corners_x, corners_y);
            delete from_dataset;
        }
    }

    // Find bounds
    min_lon = max_lon = corners_x[0];
    min_lat = max_lat = corners_y[0];

    for (int i = 1; i < 4; ++i) {
        min_lon = std::min(min_lon, static_cast<Real>(corners_x[i]));
        max_lon = std::max(max_lon, static_cast<Real>(corners_x[i]));
        min_lat = std::min(min_lat, static_cast<Real>(corners_y[i]));
        max_lat = std::max(max_lat, static_cast<Real>(corners_y[i]));
    }
}

Real GDALTerrainLoader::get_resolution() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->dataset) {
        return 0.0;
    }

    // Return approximate resolution in degrees
    // Use the pixel resolution from geotransform
    Real res_x = std::abs(static_cast<Real>(impl_->geo_transform[1]));
    Real res_y = std::abs(static_cast<Real>(impl_->geo_transform[5]));

    return std::max(res_x, res_y);
}

bool GDALTerrainLoader::is_loaded() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->dataset != nullptr;
}

TerrainDataset GDALTerrainLoader::get_dataset_info() const {
    TerrainDataset info;

    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->dataset) {
        return info;
    }

    get_bounds(info.min_lat, info.max_lat, info.min_lon, info.max_lon);
    info.resolution = get_resolution();
    info.width = impl_->width;
    info.height = impl_->height;

    // Get projection string
    const char* proj_ref = impl_->dataset->GetProjectionRef();
    if (proj_ref) {
        info.projection = proj_ref;
    }

    info.valid = true;
    return info;
}

Vec3 GDALTerrainLoader::get_normal(Real lat, Real lon) const {
    std::lock_guard<std::mutex> lock(impl_->mutex);

    if (!impl_->dataset || !impl_->band) {
        return Vec3(0, 0, 1);  // Default up vector
    }

    // Get resolution for finite difference step
    Real delta = get_resolution() * 0.5;  // Half pixel for centered differences
    if (delta < 1e-8) {
        delta = 1e-5;  // Fallback to ~1m at equator
    }

    // Get elevations at neighboring points
    Real h_center = get_elevation(lat, lon);
    Real h_north = get_elevation(lat + delta, lon);
    Real h_south = get_elevation(lat - delta, lon);
    Real h_east = get_elevation(lat, lon + delta);
    Real h_west = get_elevation(lat, lon - delta);

    // Check for invalid elevations
    if (std::isnan(h_center) || std::isnan(h_north) ||
        std::isnan(h_south) || std::isnan(h_east) || std::isnan(h_west)) {
        return Vec3(0, 0, 1);
    }

    // Compute gradients using centered differences
    // Convert lat/lon delta to meters for proper scaling
    const Real earth_radius = 6371000.0;  // meters
    Real meters_per_rad_lat = earth_radius;
    Real meters_per_rad_lon = earth_radius * std::cos(lat);

    Real dh_dlat = (h_north - h_south) / (2.0 * delta);  // m per radian
    Real dh_dlon = (h_east - h_west) / (2.0 * delta);    // m per radian

    // Convert to slope in NED frame
    // North component: -dh/dlat (negative because NED)
    // East component: dh/dlon
    // Down component: 1 (normalized later)
    Real north = -dh_dlat / meters_per_rad_lat;
    Real east = dh_dlon / meters_per_rad_lon;
    Real down = 1.0;

    // Normalize
    Real magnitude = std::sqrt(north * north + east * east + down * down);
    if (magnitude > 1e-8) {
        return Vec3(north / magnitude, east / magnitude, down / magnitude);
    } else {
        return Vec3(0, 0, 1);
    }
}

TerrainQuery GDALTerrainLoader::query(Real lat, Real lon) const {
    TerrainQuery result;

    result.elevation = get_elevation(lat, lon);
    result.valid = !std::isnan(result.elevation);

    if (result.valid) {
        result.normal = get_normal(lat, lon);

        // Compute slope angle from normal
        // Slope angle is angle between normal and vertical (0,0,1)
        Real cos_slope = result.normal.z;  // Dot product with (0,0,1)
        cos_slope = std::max(-1.0, std::min(1.0, cos_slope));  // Clamp
        result.slope_angle = std::acos(cos_slope);
    }

    return result;
}

#else

// ============================================================================
// Stub Implementation (when GDAL not available)
// ============================================================================

struct GDALTerrainLoader::Impl {
    // Empty stub
};

GDALTerrainLoader::GDALTerrainLoader()
    : impl_(std::make_unique<Impl>())
{
}

GDALTerrainLoader::~GDALTerrainLoader() = default;

bool GDALTerrainLoader::is_available() {
    return false;
}

bool GDALTerrainLoader::open(const std::string& /* path */) {
    return false;
}

void GDALTerrainLoader::close() {
    // No-op
}

Real GDALTerrainLoader::get_elevation(Real /* lat */, Real /* lon */) const {
    return std::numeric_limits<Real>::quiet_NaN();
}

void GDALTerrainLoader::get_bounds(Real& min_lat, Real& max_lat,
                                    Real& min_lon, Real& max_lon) const {
    min_lat = max_lat = min_lon = max_lon = 0.0;
}

Real GDALTerrainLoader::get_resolution() const {
    return 0.0;
}

bool GDALTerrainLoader::is_loaded() const {
    return false;
}

TerrainDataset GDALTerrainLoader::get_dataset_info() const {
    return TerrainDataset{};
}

Vec3 GDALTerrainLoader::get_normal(Real /* lat */, Real /* lon */) const {
    return Vec3(0, 0, 1);  // Default up vector
}

TerrainQuery GDALTerrainLoader::query(Real /* lat */, Real /* lon */) const {
    TerrainQuery result;
    result.valid = false;
    result.elevation = std::numeric_limits<Real>::quiet_NaN();
    result.normal = Vec3(0, 0, 1);
    result.slope_angle = 0.0;
    return result;
}

#endif // JAGUAR_HAS_GDAL

} // namespace jaguar::environment
