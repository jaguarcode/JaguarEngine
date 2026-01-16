/**
 * @file environment_service.cpp
 * @brief Environment service implementation
 *
 * Central service that provides unified environment data to all physics systems.
 */

#include "jaguar/environment/environment.h"
#include "jaguar/core/coordinates.h"

namespace jaguar::environment {

// ============================================================================
// EnvironmentService Implementation
// ============================================================================

EnvironmentService::EnvironmentService() = default;
EnvironmentService::~EnvironmentService() = default;

bool EnvironmentService::initialize()
{
    // Initialize terrain system
    if (!terrain_.initialize()) {
        return false;
    }

    // Atmosphere and ocean managers initialize themselves
    return true;
}

void EnvironmentService::shutdown()
{
    terrain_.shutdown();
}

void EnvironmentService::update(Real dt)
{
    // Update terrain (LOD, paging)
    terrain_.update();

    // Update ocean (waves)
    ocean_.update(dt);
}

Environment EnvironmentService::query(const Vec3& ecef, Real time) const
{
    // Convert ECEF to geodetic
    GeodeticPosition lla = coord::ecef_to_lla(ecef);

    return query_geodetic(lla.latitude, lla.longitude, lla.altitude, time);
}

Environment EnvironmentService::query_geodetic(Real lat, Real lon, Real alt, Real time) const
{
    Environment env;

    // Store position context
    env.latitude = lat;
    env.longitude = lon;
    env.altitude = alt;
    env.position_ecef = coord::lla_to_ecef(GeodeticPosition{lat, lon, alt});

    // Query atmosphere
    env.atmosphere = atmosphere_.get_state(lat, lon, alt);

    // Query terrain
    env.terrain = terrain_.query(lat, lon);
    env.terrain_elevation = env.terrain.elevation;

    // Query ocean if applicable
    // For now, check if altitude is below a threshold and over water
    // (Full implementation would use coastline data)
    env.over_water = (alt < 100.0 && env.terrain.elevation <= 0.0);
    if (env.over_water) {
        env.ocean = ocean_.get_state(lat, lon, time);
    }

    // Set gravity vector (in local NED frame, then convert to body-aligned if needed)
    // For most entities, gravity points "down" in local frame
    // This is a simplified model; full implementation would use gravity model
    constexpr Real G0 = 9.80665;
    env.gravity = Vec3{0.0, 0.0, G0};  // Down is +Z in NED

    return env;
}

} // namespace jaguar::environment
