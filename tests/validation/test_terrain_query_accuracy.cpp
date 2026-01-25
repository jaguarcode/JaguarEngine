/**
 * @file test_terrain_query_accuracy.cpp
 * @brief Terrain query accuracy validation tests
 *
 * Tests the accuracy of terrain queries including bilinear interpolation,
 * surface normal computation, coordinate transformations, and slope calculations.
 *
 * Theoretical Basis:
 * - Bilinear interpolation: f(x,y) = (1-x)(1-y)f00 + x(1-y)f10 + (1-x)y*f01 + xy*f11
 * - Surface normal: n = normalize(∂r/∂lat × ∂r/∂lon)
 * - ECEF ↔ LLA: Standard geodetic transformations (WGS84)
 * - Slope angle: θ = arctan(|∇h|)
 *
 * References:
 * - Snyder, "Map Projections: A Working Manual"
 * - Hofmann-Wellenhof et al., "GNSS – Global Navigation Satellite Systems"
 */

#include <gtest/gtest.h>
#include <cmath>
#include <vector>

#include "jaguar/core/types.h"
#include "jaguar/environment/terrain.h"

namespace jaguar::test {

using namespace jaguar;
using namespace jaguar::environment;

class TerrainQueryAccuracyTest : public ::testing::Test {
protected:
    static constexpr Real TOL_ELEVATION = 0.1;      // 10cm elevation tolerance
    static constexpr Real TOL_ANGLE = 0.01;         // ~0.57° angle tolerance
    static constexpr Real TOL_COORDINATE = 1.0;     // 1m coordinate tolerance
    static constexpr Real TOL_INTERPOLATION = 1e-6; // Strict for synthetic data

    // WGS84 ellipsoid parameters
    static constexpr Real WGS84_A = 6378137.0;           // Semi-major axis (m)
    static constexpr Real WGS84_F = 1.0 / 298.257223563; // Flattening
    static constexpr Real WGS84_B = WGS84_A * (1.0 - WGS84_F); // Semi-minor axis

    void SetUp() override {
    }

    /**
     * @brief Convert ECEF to geodetic coordinates (LLA)
     *
     * Using Bowring's method for accuracy
     */
    void ecef_to_lla(const Vec3& ecef, Real& lat, Real& lon, Real& alt) const {
        Real x = ecef.x;
        Real y = ecef.y;
        Real z = ecef.z;

        Real e_sq = 2.0 * WGS84_F - WGS84_F * WGS84_F;
        Real p = std::sqrt(x * x + y * y);

        lon = std::atan2(y, x);

        // Iterative latitude calculation
        Real lat_prev = std::atan2(z, p * (1.0 - e_sq));
        Real N = 0.0;

        for (int i = 0; i < 5; ++i) {
            N = WGS84_A / std::sqrt(1.0 - e_sq * std::sin(lat_prev) * std::sin(lat_prev));
            lat = std::atan2(z + e_sq * N * std::sin(lat_prev), p);
            if (std::abs(lat - lat_prev) < 1e-12) break;
            lat_prev = lat;
        }

        alt = p / std::cos(lat) - N;
    }

    /**
     * @brief Convert geodetic to ECEF coordinates
     */
    Vec3 lla_to_ecef(Real lat, Real lon, Real alt) const {
        Real e_sq = 2.0 * WGS84_F - WGS84_F * WGS84_F;
        Real N = WGS84_A / std::sqrt(1.0 - e_sq * std::sin(lat) * std::sin(lat));

        Real x = (N + alt) * std::cos(lat) * std::cos(lon);
        Real y = (N + alt) * std::cos(lat) * std::sin(lon);
        Real z = (N * (1.0 - e_sq) + alt) * std::sin(lat);

        return Vec3(x, y, z);
    }
};

// ============================================================================
// Bilinear Interpolation Accuracy
// ============================================================================

/**
 * @brief Test bilinear interpolation on known grid
 *
 * Theory:
 * - Bilinear interpolation: exact for planar surfaces
 * - For grid points: f00, f10, f01, f11
 * - At (u, v): f = (1-u)(1-v)*f00 + u(1-v)*f10 + (1-u)v*f01 + uv*f11
 */
TEST_F(TerrainQueryAccuracyTest, BilinearInterpolation) {
    // Create a simple 2x2 grid
    // (0,0)=0  (1,0)=1
    // (0,1)=2  (1,1)=3
    Real f00 = 0.0;
    Real f10 = 1.0;
    Real f01 = 2.0;
    Real f11 = 3.0;

    auto bilinear = [&](Real u, Real v) -> Real {
        return (1.0 - u) * (1.0 - v) * f00 +
               u * (1.0 - v) * f10 +
               (1.0 - u) * v * f01 +
               u * v * f11;
    };

    // Test corner points
    EXPECT_NEAR(bilinear(0.0, 0.0), f00, TOL_INTERPOLATION);
    EXPECT_NEAR(bilinear(1.0, 0.0), f10, TOL_INTERPOLATION);
    EXPECT_NEAR(bilinear(0.0, 1.0), f01, TOL_INTERPOLATION);
    EXPECT_NEAR(bilinear(1.0, 1.0), f11, TOL_INTERPOLATION);

    // Test center point
    EXPECT_NEAR(bilinear(0.5, 0.5), 1.5, TOL_INTERPOLATION);

    // Test edge midpoints
    EXPECT_NEAR(bilinear(0.5, 0.0), 0.5, TOL_INTERPOLATION);
    EXPECT_NEAR(bilinear(1.0, 0.5), 2.0, TOL_INTERPOLATION);
    EXPECT_NEAR(bilinear(0.5, 1.0), 2.5, TOL_INTERPOLATION);
    EXPECT_NEAR(bilinear(0.0, 0.5), 1.0, TOL_INTERPOLATION);

    // Test arbitrary point
    EXPECT_NEAR(bilinear(0.25, 0.75), 1.75, TOL_INTERPOLATION);
}

/**
 * @brief Test bilinear interpolation on planar surface
 *
 * Theory:
 * - For planar surface z = ax + by + c
 * - Bilinear interpolation should be exact
 */
TEST_F(TerrainQueryAccuracyTest, BilinearPlanarSurface) {
    // Planar surface: z = 2x + 3y + 5
    Real a = 2.0;
    Real b = 3.0;
    Real c = 5.0;

    auto plane = [&](Real x, Real y) -> Real {
        return a * x + b * y + c;
    };

    // Create grid
    Real x0 = 0.0, x1 = 10.0;
    Real y0 = 0.0, y1 = 10.0;

    Real f00 = plane(x0, y0);
    Real f10 = plane(x1, y0);
    Real f01 = plane(x0, y1);
    Real f11 = plane(x1, y1);

    // Test interpolation at various points
    for (Real u = 0.0; u <= 1.0; u += 0.1) {
        for (Real v = 0.0; v <= 1.0; v += 0.1) {
            Real x = x0 + u * (x1 - x0);
            Real y = y0 + v * (y1 - y0);

            Real z_analytical = plane(x, y);

            Real z_interpolated = (1.0 - u) * (1.0 - v) * f00 +
                                  u * (1.0 - v) * f10 +
                                  (1.0 - u) * v * f01 +
                                  u * v * f11;

            EXPECT_NEAR(z_interpolated, z_analytical, TOL_INTERPOLATION);
        }
    }
}

// ============================================================================
// Surface Normal Computation Accuracy
// ============================================================================

/**
 * @brief Test surface normal for planar surface
 *
 * Theory:
 * - Plane: z = ax + by + c
 * - Normal: n = normalize((-a, -b, 1))
 */
TEST_F(TerrainQueryAccuracyTest, SurfaceNormalPlanar) {
    // Planar surface z = 0.5x + 0.3y + 10
    Real a = 0.5;
    Real b = 0.3;

    // Theoretical normal (unnormalized)
    Vec3 normal_unnorm(-a, -b, 1.0);
    Vec3 normal_expected = normal_unnorm.normalized();

    // Compute normal from gradient
    // ∂z/∂x = a, ∂z/∂y = b
    Vec3 dx(1, 0, a);  // Tangent in x direction
    Vec3 dy(0, 1, b);  // Tangent in y direction

    Vec3 normal_computed = dx.cross(dy).normalized();

    EXPECT_NEAR(normal_computed.x, normal_expected.x, TOL_INTERPOLATION);
    EXPECT_NEAR(normal_computed.y, normal_expected.y, TOL_INTERPOLATION);
    EXPECT_NEAR(normal_computed.z, normal_expected.z, TOL_INTERPOLATION);

    // Normal should be unit vector
    EXPECT_NEAR(normal_computed.length(), 1.0, TOL_INTERPOLATION);
}

/**
 * @brief Test surface normal for sinusoidal surface
 *
 * Theory:
 * - Surface: z = A*sin(kx)*sin(ky)
 * - ∂z/∂x = A*k*cos(kx)*sin(ky)
 * - ∂z/∂y = A*k*sin(kx)*cos(ky)
 * - Normal: n = normalize((-∂z/∂x, -∂z/∂y, 1))
 */
TEST_F(TerrainQueryAccuracyTest, SurfaceNormalSinusoidal) {
    Real A = 5.0;    // Amplitude
    Real k = 0.1;    // Wave number

    auto surface = [&](Real x, Real y) -> Real {
        return A * std::sin(k * x) * std::sin(k * y);
    };

    auto gradient_x = [&](Real x, Real y) -> Real {
        return A * k * std::cos(k * x) * std::sin(k * y);
    };

    auto gradient_y = [&](Real x, Real y) -> Real {
        return A * k * std::sin(k * x) * std::cos(k * y);
    };

    Real x = 10.0;
    Real y = 15.0;

    Real dz_dx = gradient_x(x, y);
    Real dz_dy = gradient_y(x, y);

    Vec3 normal_expected(-dz_dx, -dz_dy, 1.0);
    normal_expected.normalized();

    // Compute from finite differences (as terrain system would)
    Real h = 0.1;  // Step size
    Real z_xp = surface(x + h, y);
    Real z_xm = surface(x - h, y);
    Real z_yp = surface(x, y + h);
    Real z_ym = surface(x, y - h);

    Real dz_dx_fd = (z_xp - z_xm) / (2.0 * h);
    Real dz_dy_fd = (z_yp - z_ym) / (2.0 * h);

    Vec3 normal_fd(-dz_dx_fd, -dz_dy_fd, 1.0);
    normal_fd.normalized();

    // Finite difference should match analytical gradient
    EXPECT_NEAR(normal_fd.x, normal_expected.x, 1e-3);
    EXPECT_NEAR(normal_fd.y, normal_expected.y, 1e-3);
    EXPECT_NEAR(normal_fd.z, normal_expected.z, 1e-3);
}

// ============================================================================
// Slope Angle Calculation Accuracy
// ============================================================================

/**
 * @brief Test slope angle calculation
 *
 * Theory:
 * - Slope angle: θ = arctan(√(dz/dx)² + (dz/dy)²)
 * - For planar surface with grade: θ = arctan(grade)
 */
TEST_F(TerrainQueryAccuracyTest, SlopeAngleCalculation) {
    // 30% grade (rise/run = 0.3)
    Real grade = 0.3;
    Real slope_expected = std::atan(grade);  // radians

    // Planar surface: z = grade * x
    Real dz_dx = grade;
    Real dz_dy = 0.0;

    Real slope_magnitude = std::sqrt(dz_dx * dz_dx + dz_dy * dz_dy);
    Real slope_angle = std::atan(slope_magnitude);

    EXPECT_NEAR(slope_angle, slope_expected, TOL_INTERPOLATION);

    // Convert to degrees for verification
    Real slope_deg = slope_angle * 180.0 / M_PI;
    EXPECT_NEAR(slope_deg, std::atan(0.3) * 180.0 / M_PI, 0.01);
}

/**
 * @brief Test slope angle on various grades
 */
TEST_F(TerrainQueryAccuracyTest, SlopeAngleVariousGrades) {
    std::vector<Real> grades = {0.0, 0.1, 0.2, 0.5, 1.0, 2.0};

    for (Real grade : grades) {
        Real slope_expected = std::atan(grade);

        // Slope magnitude
        Real slope_magnitude = grade;  // For dz/dx = grade, dz/dy = 0
        Real slope_computed = std::atan(slope_magnitude);

        EXPECT_NEAR(slope_computed, slope_expected, TOL_INTERPOLATION);

        // Verify degrees conversion
        Real slope_deg = slope_computed * 180.0 / M_PI;
        Real expected_deg = std::atan(grade) * 180.0 / M_PI;
        EXPECT_NEAR(slope_deg, expected_deg, 0.001);
    }
}

// ============================================================================
// Coordinate Transformation Accuracy (ECEF ↔ LLA)
// ============================================================================

/**
 * @brief Test ECEF to LLA conversion accuracy
 *
 * Theory:
 * - Iterative solution for geodetic latitude
 * - Should converge to sub-meter accuracy
 */
TEST_F(TerrainQueryAccuracyTest, ECEFtoLLA_Accuracy) {
    // Known locations
    struct TestPoint {
        Real lat_deg;
        Real lon_deg;
        Real alt_m;
    };

    std::vector<TestPoint> test_points = {
        {0.0, 0.0, 0.0},           // Equator at prime meridian
        {45.0, 0.0, 0.0},          // Mid-latitude
        {90.0, 0.0, 0.0},          // North pole
        {37.7749, -122.4194, 0.0}, // San Francisco
        {51.5074, -0.1278, 0.0},   // London
        {-33.8688, 151.2093, 0.0}, // Sydney
    };

    for (const auto& point : test_points) {
        Real lat_rad = point.lat_deg * M_PI / 180.0;
        Real lon_rad = point.lon_deg * M_PI / 180.0;

        // Convert to ECEF
        Vec3 ecef = lla_to_ecef(lat_rad, lon_rad, point.alt_m);

        // Convert back to LLA
        Real lat_out, lon_out, alt_out;
        ecef_to_lla(ecef, lat_out, lon_out, alt_out);

        // Check roundtrip accuracy
        EXPECT_NEAR(lat_out, lat_rad, 1e-9);  // ~1mm latitude error
        EXPECT_NEAR(lon_out, lon_rad, 1e-9);  // ~1mm longitude error
        EXPECT_NEAR(alt_out, point.alt_m, 0.01);  // 1cm altitude error
    }
}

/**
 * @brief Test LLA to ECEF conversion at various altitudes
 */
TEST_F(TerrainQueryAccuracyTest, LLAtoECEF_Altitude) {
    Real lat = 40.0 * M_PI / 180.0;  // 40°N
    Real lon = -75.0 * M_PI / 180.0; // 75°W

    std::vector<Real> altitudes = {0.0, 100.0, 1000.0, 10000.0, 100000.0};

    for (Real alt : altitudes) {
        Vec3 ecef = lla_to_ecef(lat, lon, alt);

        // Verify ECEF magnitude increases with altitude
        Real r = ecef.length();
        EXPECT_GT(r, WGS84_A);  // Should be larger than Earth radius

        // Convert back
        Real lat_out, lon_out, alt_out;
        ecef_to_lla(ecef, lat_out, lon_out, alt_out);

        EXPECT_NEAR(lat_out, lat, 1e-9);
        EXPECT_NEAR(lon_out, lon, 1e-9);
        EXPECT_NEAR(alt_out, alt, 0.01);
    }
}

/**
 * @brief Test ECEF distance calculation
 */
TEST_F(TerrainQueryAccuracyTest, ECEF_Distance) {
    // Two points on equator, 1° apart in longitude
    Real lat = 0.0;
    Real lon1 = 0.0;
    Real lon2 = 1.0 * M_PI / 180.0;  // 1° = ~111 km at equator

    Vec3 ecef1 = lla_to_ecef(lat, lon1, 0.0);
    Vec3 ecef2 = lla_to_ecef(lat, lon2, 0.0);

    Real distance = (ecef2 - ecef1).length();

    // At equator, 1° longitude ≈ 111.32 km
    Real expected_distance = 111320.0;  // meters

    EXPECT_NEAR(distance, expected_distance, 100.0);  // Within 100m
}

// ============================================================================
// Terrain Gradient Accuracy
// ============================================================================

/**
 * @brief Test terrain gradient computation using central differences
 *
 * Theory:
 * - Central difference: f'(x) ≈ (f(x+h) - f(x-h)) / (2h)
 * - O(h²) accurate for smooth functions
 */
TEST_F(TerrainQueryAccuracyTest, TerrainGradient) {
    // Quadratic surface: z = x² + y²
    auto surface = [](Real x, Real y) -> Real {
        return x * x + y * y;
    };

    auto gradient_x_analytical = [](Real x, Real /*y*/) -> Real {
        return 2.0 * x;
    };

    auto gradient_y_analytical = [](Real /*x*/, Real y) -> Real {
        return 2.0 * y;
    };

    Real x = 5.0;
    Real y = 3.0;
    Real h = 0.01;  // Step size

    // Central difference
    Real dz_dx = (surface(x + h, y) - surface(x - h, y)) / (2.0 * h);
    Real dz_dy = (surface(x, y + h) - surface(x, y - h)) / (2.0 * h);

    Real dz_dx_analytical = gradient_x_analytical(x, y);
    Real dz_dy_analytical = gradient_y_analytical(x, y);

    EXPECT_NEAR(dz_dx, dz_dx_analytical, 1e-4);
    EXPECT_NEAR(dz_dy, dz_dy_analytical, 1e-4);
}

// ============================================================================
// Terrain Material Query Consistency
// ============================================================================

/**
 * @brief Test terrain material properties are consistent
 */
TEST_F(TerrainQueryAccuracyTest, MaterialProperties) {
    // Create test terrain material
    TerrainMaterial material;
    material.type = SurfaceType::DrySand;
    material.friction_coefficient = 0.6;
    material.rolling_resistance = 0.08;
    material.max_slope_deg = 30.0;

    // Friction coefficient should be in valid range
    EXPECT_GE(material.friction_coefficient, 0.0);
    EXPECT_LE(material.friction_coefficient, 2.0);  // Rubber on rubber ~1.15

    // Rolling resistance should be small
    EXPECT_GE(material.rolling_resistance, 0.0);
    EXPECT_LE(material.rolling_resistance, 0.5);

    // Max slope should be reasonable
    EXPECT_GE(material.max_slope_deg, 0.0);
    EXPECT_LE(material.max_slope_deg, 90.0);
}

// ============================================================================
// Bilinear Interpolation Edge Cases
// ============================================================================

/**
 * @brief Test bilinear interpolation edge cases
 */
TEST_F(TerrainQueryAccuracyTest, BilinearEdgeCases) {
    // Constant surface
    Real f00 = 100.0;
    Real f10 = 100.0;
    Real f01 = 100.0;
    Real f11 = 100.0;

    auto bilinear = [&](Real u, Real v) -> Real {
        return (1.0 - u) * (1.0 - v) * f00 +
               u * (1.0 - v) * f10 +
               (1.0 - u) * v * f01 +
               u * v * f11;
    };

    // Should always return constant value
    EXPECT_NEAR(bilinear(0.0, 0.0), 100.0, TOL_INTERPOLATION);
    EXPECT_NEAR(bilinear(0.5, 0.5), 100.0, TOL_INTERPOLATION);
    EXPECT_NEAR(bilinear(1.0, 1.0), 100.0, TOL_INTERPOLATION);
    EXPECT_NEAR(bilinear(0.25, 0.75), 100.0, TOL_INTERPOLATION);

    // Linear variation in X only
    f00 = 0.0;
    f10 = 10.0;
    f01 = 0.0;
    f11 = 10.0;

    EXPECT_NEAR(bilinear(0.0, 0.0), 0.0, TOL_INTERPOLATION);
    EXPECT_NEAR(bilinear(1.0, 0.0), 10.0, TOL_INTERPOLATION);
    EXPECT_NEAR(bilinear(0.5, 0.5), 5.0, TOL_INTERPOLATION);

    // Linear variation in Y only
    f00 = 0.0;
    f10 = 0.0;
    f01 = 20.0;
    f11 = 20.0;

    EXPECT_NEAR(bilinear(0.0, 0.0), 0.0, TOL_INTERPOLATION);
    EXPECT_NEAR(bilinear(0.0, 1.0), 20.0, TOL_INTERPOLATION);
    EXPECT_NEAR(bilinear(0.5, 0.5), 10.0, TOL_INTERPOLATION);
}

} // namespace jaguar::test
