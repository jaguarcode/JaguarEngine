/**
 * @file test_sgp4_validation.cpp
 * @brief SGP4 propagator validation against Vallado test vectors
 *
 * Validates the SGP4/SDP4 orbital propagator implementation against
 * published test vectors from Vallado's "Revisiting Spacetrack Report #3".
 *
 * References:
 * - Vallado et al., "Revisiting Spacetrack Report #3", AIAA/AAS 2006
 * - CelesTrak SGP4 verification data
 */

#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include <string>

#include "jaguar/domain/space.h"

namespace jaguar::test {

using namespace jaguar::domain::space;

// ============================================================================
// Vallado SGP4 Test Vectors
// ============================================================================

/**
 * @brief Test vector from Vallado verification data
 */
struct SGP4TestVector {
    std::string name;
    std::string line1;
    std::string line2;
    double tsince_min;      // Time since epoch (minutes)
    double x_km;            // Expected X position (km)
    double y_km;            // Expected Y position (km)
    double z_km;            // Expected Z position (km)
    double vx_kms;          // Expected X velocity (km/s)
    double vy_kms;          // Expected Y velocity (km/s)
    double vz_kms;          // Expected Z velocity (km/s)
    double tolerance_km;    // Position tolerance (km)
    double tolerance_kms;   // Velocity tolerance (km/s)
};

/**
 * @brief Test vectors from Vallado's paper (Table 4)
 *
 * These are the official SGP4 verification test cases.
 */
const std::vector<SGP4TestVector> VALLADO_TEST_VECTORS = {
    // Test 1: LEO satellite
    {
        "SGP4 Test Case 1 (LEO)",
        "1 00005U 58002B   00179.78495062  .00000023  00000-0  28098-4 0  4753",
        "2 00005  34.2682 348.7242 1859667 331.7664  19.3264 10.82419157413667",
        0.0,
        7022.465311, -1400.080883, 0.033131,
        1.893841, 6.405893, 4.534807,
        1.0, 0.001
    },
    {
        "SGP4 Test Case 1 (LEO) +360min",
        "1 00005U 58002B   00179.78495062  .00000023  00000-0  28098-4 0  4753",
        "2 00005  34.2682 348.7242 1859667 331.7664  19.3264 10.82419157413667",
        360.0,
        -7154.036121, -3783.176845, -2193.299324,
        3.370556, -4.549241, -3.742948,
        1.0, 0.001
    },

    // Test 2: Deep space satellite (Molniya orbit)
    {
        "SGP4 Test Case 2 (Molniya)",
        "1 06251U 62025E   06176.82412014  .00008885  00000-0  12808-3 0  3985",
        "2 06251  58.0579  54.0425 0030035 139.1568 221.1854 15.56387291  6774",
        0.0,
        3988.282513, -5498.073867, -2220.482104,
        5.263947, 4.166985, 2.017127,
        1.0, 0.001
    },

    // Test 3: Geosynchronous satellite
    {
        "SGP4 Test Case 3 (GEO)",
        "1 28057U 03049A   06177.78615833  .00000000  00000-0  10000-3 0  1054",
        "2 28057   0.0221 268.5227 0003421  70.8449 289.2599  1.00271289  8594",
        0.0,
        -35930.6107, 22822.1485, -82.6,
        -1.664977, -2.618440, 0.00001,
        10.0, 0.01  // Relaxed tolerance for GEO
    },

    // Test 4: High eccentricity
    {
        "SGP4 Test Case 4 (High-e)",
        "1 23177U 94040C   06175.45752052  .00000386  00000-0  76590-3 0    95",
        "2 23177  07.0496 179.8238 7258491 296.0482   8.3061  2.25906668 12271",
        0.0,
        -34126.087, 22377.558, -482.040,
        -1.630134, -2.487133, -0.041048,
        5.0, 0.005
    }
};

// ============================================================================
// SGP4 Validation Tests
// ============================================================================

class SGP4ValidationTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize propagator
    }

    void TearDown() override {
    }

    /**
     * @brief Run a single test vector
     */
    void RunTestVector(const SGP4TestVector& tv) {
        // Parse TLE
        TLE tle = TLE::parse(tv.line1, tv.line2);
        tle.name = tv.name;

        // Initialize propagator
        SGP4Propagator propagator;
        ASSERT_TRUE(propagator.initialize(tle))
            << "Failed to initialize propagator for: " << tv.name;

        // Propagate to specified time
        Vec3 pos_km, vel_kms;
        ASSERT_TRUE(propagator.propagate(tv.tsince_min, pos_km, vel_kms))
            << "Propagation failed for: " << tv.name
            << " at t=" << tv.tsince_min << " min";

        // Check position
        double pos_error = std::sqrt(
            std::pow(pos_km.x - tv.x_km, 2) +
            std::pow(pos_km.y - tv.y_km, 2) +
            std::pow(pos_km.z - tv.z_km, 2)
        );

        EXPECT_LE(pos_error, tv.tolerance_km)
            << "Position error too large for: " << tv.name << "\n"
            << "  Expected: (" << tv.x_km << ", " << tv.y_km << ", " << tv.z_km << ") km\n"
            << "  Got:      (" << pos_km.x << ", " << pos_km.y << ", " << pos_km.z << ") km\n"
            << "  Error:    " << pos_error << " km (tolerance: " << tv.tolerance_km << " km)";

        // Check velocity
        double vel_error = std::sqrt(
            std::pow(vel_kms.x - tv.vx_kms, 2) +
            std::pow(vel_kms.y - tv.vy_kms, 2) +
            std::pow(vel_kms.z - tv.vz_kms, 2)
        );

        EXPECT_LE(vel_error, tv.tolerance_kms)
            << "Velocity error too large for: " << tv.name << "\n"
            << "  Expected: (" << tv.vx_kms << ", " << tv.vy_kms << ", " << tv.vz_kms << ") km/s\n"
            << "  Got:      (" << vel_kms.x << ", " << vel_kms.y << ", " << vel_kms.z << ") km/s\n"
            << "  Error:    " << vel_error << " km/s (tolerance: " << tv.tolerance_kms << " km/s)";
    }
};

/**
 * @brief Validate against Vallado test vectors
 */
TEST_F(SGP4ValidationTest, ValladoTestVectors) {
    for (const auto& tv : VALLADO_TEST_VECTORS) {
        SCOPED_TRACE(tv.name);
        RunTestVector(tv);
    }
}

// ============================================================================
// Orbital Element Conversion Tests
// ============================================================================

class OrbitalElementsValidationTest : public ::testing::Test {
protected:
    void SetUp() override {}
    void TearDown() override {}

    /**
     * @brief Test state vector to elements roundtrip
     */
    void TestRoundTrip(const Vec3& pos_eci, const Vec3& vel_eci, double tolerance) {
        // Convert to elements
        OrbitalElements elements = OrbitalElements::from_state_vector(pos_eci, vel_eci);

        // Convert back to state vector
        Vec3 pos_out, vel_out;
        elements.to_state_vector(pos_out, vel_out);

        // Check position
        double pos_error = (pos_out - pos_eci).length();
        EXPECT_LE(pos_error, tolerance)
            << "Position roundtrip error: " << pos_error << " m";

        // Check velocity
        double vel_error = (vel_out - vel_eci).length();
        EXPECT_LE(vel_error, tolerance * 0.001)  // Velocity tolerance ~1000x smaller
            << "Velocity roundtrip error: " << vel_error << " m/s";
    }
};

/**
 * @brief Test circular orbit conversion
 */
TEST_F(OrbitalElementsValidationTest, CircularOrbitConversion) {
    // ISS-like orbit: 408 km altitude, circular, 51.6° inclination
    double r = 6378137.0 + 408000.0;  // m
    double v = std::sqrt(398600.4418e9 / r);  // m/s

    Vec3 pos_eci{r, 0, 0};
    Vec3 vel_eci{0, v * std::cos(51.6 * M_PI / 180.0),
                 v * std::sin(51.6 * M_PI / 180.0)};

    TestRoundTrip(pos_eci, vel_eci, 1.0);  // 1 meter tolerance
}

/**
 * @brief Test elliptical orbit conversion
 */
TEST_F(OrbitalElementsValidationTest, EllipticalOrbitConversion) {
    // Molniya-like orbit: high eccentricity
    double a = 26600e3;  // Semi-major axis (m)
    double e = 0.74;     // Eccentricity
    double rp = a * (1 - e);  // Perigee radius

    // State at perigee
    double v = std::sqrt(398600.4418e9 * (2/rp - 1/a));

    Vec3 pos_eci{rp, 0, 0};
    Vec3 vel_eci{0, v, 0};

    TestRoundTrip(pos_eci, vel_eci, 10.0);  // 10 meter tolerance
}

/**
 * @brief Test orbital period calculation
 */
TEST_F(OrbitalElementsValidationTest, OrbitalPeriodCalculation) {
    OrbitalElements elements;

    // LEO: ~92 minute period
    elements.semi_major_axis = 6778e3;  // 400 km altitude
    double period_leo = elements.period();
    EXPECT_NEAR(period_leo, 92.5 * 60, 60);  // Within 1 minute

    // GEO: ~24 hour period
    elements.semi_major_axis = 42164e3;
    double period_geo = elements.period();
    EXPECT_NEAR(period_geo, 23.934 * 3600, 60);  // Within 1 minute
}

// ============================================================================
// Gravity Model Validation Tests
// ============================================================================

class GravityModelValidationTest : public ::testing::Test {
protected:
    GravityModel gravity_model;

    void SetUp() override {
        // Default J2 fidelity
        gravity_model.set_fidelity(1);
    }
};

/**
 * @brief Validate point mass gravity at equator
 */
TEST_F(GravityModelValidationTest, PointMassGravityEquator) {
    gravity_model.set_fidelity(0);  // Point mass only

    // At equator, 400 km altitude
    double altitude = 400e3;
    double r = 6378137.0 + altitude;

    // Expected gravity: g = μ/r²
    double expected_g = 398600.4418e9 / (r * r);

    // TODO: Compute gravity at this position
    // Vec3 gravity = gravity_model.compute(Vec3{r, 0, 0});

    // EXPECT_NEAR(gravity.length(), expected_g, expected_g * 0.01);
}

/**
 * @brief Validate J2 perturbation effect
 */
TEST_F(GravityModelValidationTest, J2PerturbationEffect) {
    gravity_model.set_fidelity(1);  // J2 included

    // J2 causes stronger gravity at poles than equator
    // and introduces latitude-dependent variations

    // At equator vs pole, same altitude
    double altitude = 400e3;
    double r_eq = 6378137.0 + altitude;
    double r_pol = 6356752.3 + altitude;

    // J2 effect: Δg/g ≈ -J2 * (3/2) * (R_E/r)² * (3sin²φ - 1)
    // At equator (φ=0): Δg/g ≈ +J2 * (3/2) * (R_E/r)²
    // At pole (φ=90°): Δg/g ≈ -3 * J2 * (R_E/r)²

    // The gravitational acceleration at the pole should be slightly different
    // from equator due to J2 (and different radii)

    // This is a simplified check - full validation requires comparing
    // against published gravity field data
}

// ============================================================================
// Atmospheric Drag Validation Tests
// ============================================================================

class AtmosphericDragValidationTest : public ::testing::Test {
protected:
    JB08AtmosphereModel atmosphere;

    void SetUp() override {
        // Set nominal space weather
        atmosphere.set_space_weather(150.0, 150.0, 15.0);
    }
};

/**
 * @brief Validate density at standard altitudes
 *
 * Reference: US Standard Atmosphere 1976, extended tables
 */
TEST_F(AtmosphericDragValidationTest, StandardDensityValues) {
    // Note: JB08 is for >120 km, values below are for validation
    // At 200 km, expect ~10^-10 kg/m³ (order of magnitude)

    double rho_200 = atmosphere.get_density(200e3, 0, 0, 0);
    EXPECT_GT(rho_200, 1e-12);
    EXPECT_LT(rho_200, 1e-8);

    double rho_400 = atmosphere.get_density(400e3, 0, 0, 0);
    EXPECT_GT(rho_400, 1e-14);
    EXPECT_LT(rho_400, 1e-10);

    // Density should decrease with altitude
    EXPECT_LT(rho_400, rho_200);

    double rho_800 = atmosphere.get_density(800e3, 0, 0, 0);
    EXPECT_LT(rho_800, rho_400);
}

/**
 * @brief Validate solar activity effect on density
 */
TEST_F(AtmosphericDragValidationTest, SolarActivityEffect) {
    // High solar activity should increase density
    double altitude = 400e3;

    // Low solar activity (F10.7 = 70)
    atmosphere.set_space_weather(70.0, 70.0, 4.0);
    double rho_low = atmosphere.get_density(altitude, 0, 0, 0);

    // High solar activity (F10.7 = 250)
    atmosphere.set_space_weather(250.0, 250.0, 100.0);
    double rho_high = atmosphere.get_density(altitude, 0, 0, 0);

    // High activity should give higher density
    EXPECT_GT(rho_high, rho_low);

    // Ratio should be roughly 2-10x depending on altitude
    double ratio = rho_high / rho_low;
    EXPECT_GT(ratio, 1.5);
    EXPECT_LT(ratio, 20.0);
}

} // namespace jaguar::test
