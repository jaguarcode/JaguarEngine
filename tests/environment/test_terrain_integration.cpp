/**
 * @file test_terrain_integration.cpp
 * @brief Tests for terrain integration with environment service
 */

#include <gtest/gtest.h>
#include "jaguar/environment/environment.h"
#include "jaguar/core/coordinates.h"
#include <cmath>
#include <thread>
#include <atomic>
#include <vector>

namespace jaguar::test {

using namespace jaguar::environment;

class TerrainIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        ASSERT_TRUE(env_service.initialize());
    }

    void TearDown() override {
        env_service.shutdown();
    }

    EnvironmentService env_service;
};

/**
 * @brief Test default flat terrain behavior
 */
TEST_F(TerrainIntegrationTest, DefaultFlatTerrain) {
    // Query terrain at arbitrary position
    Real lat = 0.0;  // Equator
    Real lon = 0.0;  // Prime meridian
    Real alt = 100.0;  // 100m above WGS84

    // Query environment
    Environment env = env_service.query_geodetic(lat, lon, alt, 0.0);

    // Default terrain should be flat (elevation 0)
    EXPECT_DOUBLE_EQ(env.terrain_elevation, 0.0);
    EXPECT_TRUE(env.terrain.valid);
    EXPECT_DOUBLE_EQ(env.terrain.elevation, 0.0);

    // Normal should point up (NED: 0,0,1)
    EXPECT_DOUBLE_EQ(env.terrain.normal.x, 0.0);
    EXPECT_DOUBLE_EQ(env.terrain.normal.y, 0.0);
    EXPECT_DOUBLE_EQ(env.terrain.normal.z, 1.0);

    // Slope should be zero
    EXPECT_DOUBLE_EQ(env.terrain.slope_angle, 0.0);
}

/**
 * @brief Test terrain elevation query
 */
TEST_F(TerrainIntegrationTest, TerrainElevationQuery) {
    Real lat = 0.6;  // ~34 degrees North
    Real lon = -2.0; // ~115 degrees West

    Real elevation = env_service.get_terrain_elevation(lat, lon);

    // Without loaded terrain data, should return 0
    EXPECT_DOUBLE_EQ(elevation, 0.0);
}

/**
 * @brief Test terrain query from ECEF position
 */
TEST_F(TerrainIntegrationTest, TerrainQueryECEF) {
    // Position at sea level, equator
    Vec3 ecef{6378137.0, 0.0, 0.0};  // On equator at prime meridian

    TerrainQuery query = env_service.query_terrain(ecef);

    EXPECT_TRUE(query.valid);
    EXPECT_DOUBLE_EQ(query.elevation, 0.0);
    EXPECT_DOUBLE_EQ(query.normal.x, 0.0);
    EXPECT_DOUBLE_EQ(query.normal.y, 0.0);
    EXPECT_DOUBLE_EQ(query.normal.z, 1.0);
}

/**
 * @brief Test terrain manager access
 */
TEST_F(TerrainIntegrationTest, TerrainManagerAccess) {
    const TerrainManager& terrain = env_service.terrain();

    // Should be able to query terrain directly
    Real lat = 0.5;
    Real lon = 1.0;

    TerrainQuery query = terrain.query(lat, lon);
    EXPECT_TRUE(query.valid);

    Real elevation = terrain.get_elevation(lat, lon);
    EXPECT_DOUBLE_EQ(elevation, 0.0);

    Vec3 normal = terrain.get_surface_normal(lat, lon);
    EXPECT_DOUBLE_EQ(normal.z, 1.0);
}

/**
 * @brief Test terrain is not loaded initially
 */
TEST_F(TerrainIntegrationTest, NoTerrainLoadedInitially) {
    const TerrainManager& terrain = env_service.terrain();
    EXPECT_FALSE(terrain.is_terrain_loaded());
}

/**
 * @brief Test loading terrain file (will fail without actual file)
 */
TEST_F(TerrainIntegrationTest, LoadTerrainFile) {
    // Attempt to load non-existent file should fail gracefully
    bool result = env_service.load_terrain("/nonexistent/terrain.tif");
    EXPECT_FALSE(result);

    // Terrain should still not be loaded
    EXPECT_FALSE(env_service.terrain().is_terrain_loaded());

    // Should still return default flat terrain
    Real elevation = env_service.get_terrain_elevation(0.0, 0.0);
    EXPECT_DOUBLE_EQ(elevation, 0.0);
}

/**
 * @brief Test thread-safe terrain queries
 */
TEST_F(TerrainIntegrationTest, ThreadSafeQueries) {
    // Query from multiple threads should not crash
    std::vector<std::thread> threads;
    std::atomic<int> query_count{0};

    for (int i = 0; i < 10; ++i) {
        threads.emplace_back([&, i]() {
            Real lat = i * 0.1;
            Real lon = i * 0.2;
            TerrainQuery query = env_service.query_terrain(
                coord::lla_to_ecef(GeodeticPosition{lat, lon, 0.0})
            );
            EXPECT_TRUE(query.valid);
            query_count++;
        });
    }

    for (auto& t : threads) {
        t.join();
    }

    EXPECT_EQ(query_count, 10);
}

/**
 * @brief Test environment query includes terrain data
 */
TEST_F(TerrainIntegrationTest, EnvironmentIncludesTerrain) {
    Real lat = 0.8;
    Real lon = -1.5;
    Real alt = 1000.0;

    Environment env = env_service.query_geodetic(lat, lon, alt, 0.0);

    // Check that terrain is populated
    EXPECT_TRUE(env.terrain.valid);
    EXPECT_EQ(env.terrain.elevation, env.terrain_elevation);

    // Position context should match
    EXPECT_DOUBLE_EQ(env.latitude, lat);
    EXPECT_DOUBLE_EQ(env.longitude, lon);
    EXPECT_DOUBLE_EQ(env.altitude, alt);
}

} // namespace jaguar::test
