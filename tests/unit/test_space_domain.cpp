/**
 * @file test_space_domain.cpp
 * @brief Unit tests for Space domain (SGP4, Gravity, Coordinate Transforms)
 */

#include <gtest/gtest.h>
#include "jaguar/domain/space.h"
#include "jaguar/environment/environment.h"
#include <cmath>

using namespace jaguar;
using namespace jaguar::domain::space;

namespace {
constexpr Real PI = 3.14159265358979323846;
constexpr Real DEG_TO_RAD = PI / 180.0;
constexpr Real MU_EARTH = 3.986004418e14;  // m³/s²
constexpr Real EARTH_RADIUS = 6378137.0;   // m
}

// ============================================================================
// OrbitalElements Tests
// ============================================================================

class OrbitalElementsTest : public ::testing::Test {
protected:
    void SetUp() override {}
};

TEST_F(OrbitalElementsTest, CircularOrbitPeriod) {
    OrbitalElements elements;
    elements.semi_major_axis = 7000000.0;  // 7000 km altitude orbit
    elements.eccentricity = 0.0;  // Circular

    Real period = elements.period();

    // Expected: T = 2π * sqrt(a³/μ)
    Real expected = 2.0 * PI * std::sqrt(std::pow(7000000.0, 3) / MU_EARTH);

    EXPECT_NEAR(period, expected, 1.0);  // Within 1 second
}

TEST_F(OrbitalElementsTest, CircularOrbitAltitude) {
    OrbitalElements elements;
    elements.semi_major_axis = EARTH_RADIUS + 400000.0;  // 400 km altitude
    elements.eccentricity = 0.0;
    elements.true_anomaly = 0.0;

    Real altitude = elements.altitude();

    EXPECT_NEAR(altitude, 400000.0, 1.0);  // Within 1 meter
}

TEST_F(OrbitalElementsTest, EllipticalOrbitAltitudePerigee) {
    OrbitalElements elements;
    elements.semi_major_axis = EARTH_RADIUS + 500000.0;  // ~500 km average
    elements.eccentricity = 0.1;
    elements.true_anomaly = 0.0;  // At perigee

    Real perigee_altitude = elements.altitude();

    // Perigee = a(1-e) - Re
    Real expected_perigee = (EARTH_RADIUS + 500000.0) * (1.0 - 0.1) - EARTH_RADIUS;

    EXPECT_NEAR(perigee_altitude, expected_perigee, 100.0);  // Within 100m
}

TEST_F(OrbitalElementsTest, ToStateVectorCircular) {
    OrbitalElements elements;
    elements.semi_major_axis = EARTH_RADIUS + 400000.0;
    elements.eccentricity = 0.0;
    elements.inclination = 51.6 * DEG_TO_RAD;  // ISS-like inclination
    elements.raan = 0.0;
    elements.arg_of_perigee = 0.0;
    elements.true_anomaly = 0.0;

    Vec3 pos, vel;
    elements.to_state_vector(pos, vel);

    // For circular orbit, position magnitude should equal semi-major axis
    Real r = std::sqrt(pos.x * pos.x + pos.y * pos.y + pos.z * pos.z);
    EXPECT_NEAR(r, elements.semi_major_axis, 10.0);  // Within 10m

    // Velocity magnitude for circular orbit: v = sqrt(μ/r)
    Real v = std::sqrt(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z);
    Real expected_v = std::sqrt(MU_EARTH / elements.semi_major_axis);
    EXPECT_NEAR(v, expected_v, 1.0);  // Within 1 m/s
}

TEST_F(OrbitalElementsTest, FromStateVectorRoundTrip) {
    OrbitalElements original;
    original.semi_major_axis = EARTH_RADIUS + 500000.0;
    original.eccentricity = 0.05;
    original.inclination = 45.0 * DEG_TO_RAD;
    original.raan = 30.0 * DEG_TO_RAD;
    original.arg_of_perigee = 60.0 * DEG_TO_RAD;
    original.true_anomaly = 90.0 * DEG_TO_RAD;

    Vec3 pos, vel;
    original.to_state_vector(pos, vel);

    OrbitalElements recovered = OrbitalElements::from_state_vector(pos, vel);

    EXPECT_NEAR(recovered.semi_major_axis, original.semi_major_axis, 1000.0);
    EXPECT_NEAR(recovered.eccentricity, original.eccentricity, 0.001);
    EXPECT_NEAR(recovered.inclination, original.inclination, 0.01);
}

// ============================================================================
// TLE Parser Tests
// ============================================================================

class TLETest : public ::testing::Test {
protected:
    // ISS TLE (example)
    const std::string iss_line1 = "1 25544U 98067A   24001.50000000  .00016717  00000-0  10270-3 0  9993";
    const std::string iss_line2 = "2 25544  51.6416 247.4627 0006703 130.5360 325.0288 15.54174397100001";
};

TEST_F(TLETest, ParseSatelliteNumber) {
    TLE tle = TLE::parse(iss_line1, iss_line2);
    EXPECT_EQ(tle.satellite_number, 25544);
}

TEST_F(TLETest, ParseInclination) {
    TLE tle = TLE::parse(iss_line1, iss_line2);
    Real expected_inc = 51.6416 * DEG_TO_RAD;
    EXPECT_NEAR(tle.inclination, expected_inc, 0.001);
}

TEST_F(TLETest, ParseEccentricity) {
    TLE tle = TLE::parse(iss_line1, iss_line2);
    // Eccentricity in TLE is "0006703" meaning 0.0006703
    EXPECT_NEAR(tle.eccentricity, 0.0006703, 1e-8);
}

TEST_F(TLETest, ParseMeanMotion) {
    TLE tle = TLE::parse(iss_line1, iss_line2);
    // Mean motion is 15.54174397 rev/day
    EXPECT_NEAR(tle.mean_motion, 15.54174397, 1e-6);
}

TEST_F(TLETest, ParseRAAN) {
    TLE tle = TLE::parse(iss_line1, iss_line2);
    Real expected_raan = 247.4627 * DEG_TO_RAD;
    EXPECT_NEAR(tle.raan, expected_raan, 0.001);
}

// ============================================================================
// SGP4 Propagator Tests
// ============================================================================

class SGP4Test : public ::testing::Test {
protected:
    SGP4Propagator propagator;

    // ISS TLE (example)
    const std::string iss_line1 = "1 25544U 98067A   24001.50000000  .00016717  00000-0  10270-3 0  9993";
    const std::string iss_line2 = "2 25544  51.6416 247.4627 0006703 130.5360 325.0288 15.54174397100001";

    void SetUp() override {}
};

TEST_F(SGP4Test, InitializeFromTLE) {
    TLE tle = TLE::parse(iss_line1, iss_line2);
    bool result = propagator.initialize(tle);
    EXPECT_TRUE(result);
}

TEST_F(SGP4Test, InitializeFailsWithEmptyTLE) {
    TLE empty_tle;
    bool result = propagator.initialize(empty_tle);
    EXPECT_FALSE(result);
}

TEST_F(SGP4Test, PropagateAtEpoch) {
    TLE tle = TLE::parse(iss_line1, iss_line2);
    propagator.initialize(tle);

    Vec3 pos, vel;
    bool result = propagator.propagate(0.0, pos, vel);

    EXPECT_TRUE(result);

    // At epoch, position should be in reasonable range for ISS
    Real r = std::sqrt(pos.x * pos.x + pos.y * pos.y + pos.z * pos.z);
    EXPECT_GT(r, 6500.0);   // Above Earth surface (km)
    EXPECT_LT(r, 6900.0);   // ISS orbit is ~400km altitude
}

TEST_F(SGP4Test, PropagateForward) {
    TLE tle = TLE::parse(iss_line1, iss_line2);
    propagator.initialize(tle);

    Vec3 pos1, vel1;
    propagator.propagate(0.0, pos1, vel1);

    Vec3 pos2, vel2;
    propagator.propagate(90.0, pos2, vel2);  // 90 minutes (about 1 orbit)

    // Position should be different
    EXPECT_NE(pos1.x, pos2.x);
    EXPECT_NE(pos1.y, pos2.y);

    // Velocity magnitude should be similar (circular orbit)
    Real v1 = std::sqrt(vel1.x * vel1.x + vel1.y * vel1.y + vel1.z * vel1.z);
    Real v2 = std::sqrt(vel2.x * vel2.x + vel2.y * vel2.y + vel2.z * vel2.z);
    EXPECT_NEAR(v1, v2, 0.1);  // Within 0.1 km/s
}

TEST_F(SGP4Test, IsNotDeepSpace) {
    // ISS is LEO, not deep space
    TLE tle = TLE::parse(iss_line1, iss_line2);
    propagator.initialize(tle);
    EXPECT_FALSE(propagator.is_deep_space());
}

// ============================================================================
// Gravity Model Tests
// ============================================================================

class GravityModelTest : public ::testing::Test {
protected:
    GravityModel model;
    physics::EntityState state;
    environment::Environment env;
    physics::EntityForces forces;

    void SetUp() override {
        state.mass = 1000.0;  // 1000 kg satellite
        state.velocity = Vec3{0, 0, 0};
        state.orientation = Quat::Identity();
        state.angular_velocity = Vec3{0, 0, 0};
        forces.clear();
    }
};

TEST_F(GravityModelTest, PointMassAtSurface) {
    // Position at equator on surface
    state.position = Vec3{EARTH_RADIUS, 0, 0};

    model.set_fidelity(0);  // Point mass only
    model.compute_forces(state, env, 0.1, forces);

    // Expected acceleration: g ≈ 9.8 m/s²
    Vec3 accel = model.get_gravity_acceleration();
    Real accel_mag = std::sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);

    EXPECT_NEAR(accel_mag, 9.8, 0.1);

    // Force should point toward center (negative X)
    EXPECT_LT(accel.x, 0.0);
    EXPECT_NEAR(accel.y, 0.0, 1e-6);
    EXPECT_NEAR(accel.z, 0.0, 1e-6);
}

TEST_F(GravityModelTest, PointMassAtOrbit) {
    // Position at 400 km altitude
    state.position = Vec3{EARTH_RADIUS + 400000.0, 0, 0};

    model.set_fidelity(0);
    model.compute_forces(state, env, 0.1, forces);

    // Expected acceleration: g ≈ μ/r²
    Real r = EARTH_RADIUS + 400000.0;
    Real expected_accel = MU_EARTH / (r * r);

    Vec3 accel = model.get_gravity_acceleration();
    Real accel_mag = std::sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);

    EXPECT_NEAR(accel_mag, expected_accel, 0.01);
}

TEST_F(GravityModelTest, J2PerturbationAtPoles) {
    // Position at North Pole (Z-axis)
    state.position = Vec3{0, 0, EARTH_RADIUS};

    model.set_fidelity(1);  // J2 model
    model.compute_forces(state, env, 0.1, forces);

    Vec3 accel_j2 = model.get_gravity_acceleration();

    // Compare to point mass
    model.set_fidelity(0);
    forces.clear();
    model.compute_forces(state, env, 0.1, forces);
    Vec3 accel_pm = model.get_gravity_acceleration();

    // J2 should produce different result at poles
    EXPECT_NE(accel_j2.z, accel_pm.z);
}

TEST_F(GravityModelTest, J2PerturbationAtEquator) {
    // Position at equator
    state.position = Vec3{EARTH_RADIUS, 0, 0};

    model.set_fidelity(1);  // J2 model
    model.compute_forces(state, env, 0.1, forces);

    Vec3 accel_j2 = model.get_gravity_acceleration();

    // Compare to point mass
    model.set_fidelity(0);
    forces.clear();
    model.compute_forces(state, env, 0.1, forces);
    Vec3 accel_pm = model.get_gravity_acceleration();

    // J2 correction should be different at equator vs poles
    Real diff_x = std::abs(accel_j2.x - accel_pm.x);
    EXPECT_GT(diff_x, 0.0);
}

TEST_F(GravityModelTest, J2J4Model) {
    state.position = Vec3{EARTH_RADIUS + 400000.0, 0, 0};

    model.set_fidelity(2);  // J2-J4 model
    model.compute_forces(state, env, 0.1, forces);

    Vec3 accel = model.get_gravity_acceleration();
    Real accel_mag = std::sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);

    // Should still be close to point mass (perturbations are small)
    Real r = EARTH_RADIUS + 400000.0;
    Real expected_accel = MU_EARTH / (r * r);

    EXPECT_NEAR(accel_mag, expected_accel, 0.1);  // Within 0.1 m/s²
}

TEST_F(GravityModelTest, DisabledModelProducesNoForce) {
    state.position = Vec3{EARTH_RADIUS, 0, 0};

    model.set_enabled(false);
    model.compute_forces(state, env, 0.1, forces);

    EXPECT_NEAR(forces.force.x, 0.0, 1e-6);
    EXPECT_NEAR(forces.force.y, 0.0, 1e-6);
    EXPECT_NEAR(forces.force.z, 0.0, 1e-6);
}

TEST_F(GravityModelTest, ForceProportionalToMass) {
    state.position = Vec3{EARTH_RADIUS + 400000.0, 0, 0};

    // Light satellite
    state.mass = 100.0;
    model.compute_forces(state, env, 0.1, forces);
    Real force_light = std::sqrt(forces.force.x * forces.force.x +
                                  forces.force.y * forces.force.y +
                                  forces.force.z * forces.force.z);

    // Heavy satellite
    forces.clear();
    state.mass = 1000.0;
    model.compute_forces(state, env, 0.1, forces);
    Real force_heavy = std::sqrt(forces.force.x * forces.force.x +
                                  forces.force.y * forces.force.y +
                                  forces.force.z * forces.force.z);

    // Force should scale with mass
    EXPECT_NEAR(force_heavy / force_light, 10.0, 0.001);
}

// ============================================================================
// Coordinate Transform Tests
// ============================================================================

class CoordinateTransformTest : public ::testing::Test {
protected:
    void SetUp() override {}
};

TEST_F(CoordinateTransformTest, GeodeticToECEFEquator) {
    // Point on equator at prime meridian
    Real lat = 0.0;
    Real lon = 0.0;
    Real alt = 0.0;

    Vec3 ecef = transforms::geodetic_to_ecef(lat, lon, alt);

    // Should be at (Re, 0, 0)
    EXPECT_NEAR(ecef.x, EARTH_RADIUS, 100.0);
    EXPECT_NEAR(ecef.y, 0.0, 1.0);
    EXPECT_NEAR(ecef.z, 0.0, 1.0);
}

TEST_F(CoordinateTransformTest, GeodeticToECEFPole) {
    // North pole
    Real lat = PI / 2.0;
    Real lon = 0.0;
    Real alt = 0.0;

    Vec3 ecef = transforms::geodetic_to_ecef(lat, lon, alt);

    // Should be at (0, 0, ~Rp) where Rp is polar radius
    Real polar_radius = EARTH_RADIUS * (1.0 - 1.0/298.257223563);
    EXPECT_NEAR(ecef.x, 0.0, 1.0);
    EXPECT_NEAR(ecef.y, 0.0, 1.0);
    EXPECT_NEAR(ecef.z, polar_radius, 1000.0);
}

TEST_F(CoordinateTransformTest, GeodeticToECEFWithAltitude) {
    // Point at equator with 400km altitude
    Real lat = 0.0;
    Real lon = 0.0;
    Real alt = 400000.0;

    Vec3 ecef = transforms::geodetic_to_ecef(lat, lon, alt);

    // Should be at (Re + alt, 0, 0)
    EXPECT_NEAR(ecef.x, EARTH_RADIUS + alt, 100.0);
    EXPECT_NEAR(ecef.y, 0.0, 1.0);
    EXPECT_NEAR(ecef.z, 0.0, 1.0);
}

TEST_F(CoordinateTransformTest, ECEFToGeodeticRoundTrip) {
    // Original geodetic coordinates
    Real lat_orig = 45.0 * DEG_TO_RAD;
    Real lon_orig = -75.0 * DEG_TO_RAD;
    Real alt_orig = 100000.0;

    // Convert to ECEF
    Vec3 ecef = transforms::geodetic_to_ecef(lat_orig, lon_orig, alt_orig);

    // Convert back to geodetic
    Real lat_back, lon_back, alt_back;
    transforms::ecef_to_geodetic(ecef, lat_back, lon_back, alt_back);

    EXPECT_NEAR(lat_back, lat_orig, 1e-8);
    EXPECT_NEAR(lon_back, lon_orig, 1e-8);
    EXPECT_NEAR(alt_back, alt_orig, 1.0);  // Within 1m
}

TEST_F(CoordinateTransformTest, ECIToECEFRoundTrip) {
    Vec3 eci_orig{7000000.0, 0.0, 0.0};
    Real time = 2451545.0;  // J2000 epoch

    Vec3 ecef = transforms::eci_to_ecef(eci_orig, time);
    Vec3 eci_back = transforms::ecef_to_eci(ecef, time);

    EXPECT_NEAR(eci_back.x, eci_orig.x, 1.0);
    EXPECT_NEAR(eci_back.y, eci_orig.y, 1.0);
    EXPECT_NEAR(eci_back.z, eci_orig.z, 1.0);
}

TEST_F(CoordinateTransformTest, GMSTAtJ2000) {
    // GMST at J2000.0 should be approximately 280.46 degrees
    Real jd = 2451545.0;
    Real gmst_rad = transforms::gmst(jd);
    Real gmst_deg = gmst_rad * 180.0 / PI;

    // GMST cycles through 360 degrees, so normalize
    while (gmst_deg < 0) gmst_deg += 360.0;
    while (gmst_deg >= 360.0) gmst_deg -= 360.0;

    // At J2000.0 midnight, GMST ≈ 280.46° (6h 41m 50.55s)
    EXPECT_NEAR(gmst_deg, 280.46, 1.0);  // Within 1 degree
}

// ============================================================================
// JB08 Atmosphere Model Tests
// ============================================================================

class JB08AtmosphereTest : public ::testing::Test {
protected:
    JB08AtmosphereModel model;
};

TEST_F(JB08AtmosphereTest, DensityDecreasesWithAltitude) {
    Real rho_400 = model.get_density(400000.0, 0.0, 0.0, 0.0);
    Real rho_500 = model.get_density(500000.0, 0.0, 0.0, 0.0);
    Real rho_600 = model.get_density(600000.0, 0.0, 0.0, 0.0);

    EXPECT_GT(rho_400, rho_500);
    EXPECT_GT(rho_500, rho_600);
}

TEST_F(JB08AtmosphereTest, DensityBelowSpaceDomain) {
    Real rho = model.get_density(50000.0, 0.0, 0.0, 0.0);  // 50km
    EXPECT_NEAR(rho, 0.0, 1e-20);  // Not in space domain
}

TEST_F(JB08AtmosphereTest, DensityAboveThreshold) {
    Real rho = model.get_density(1500000.0, 0.0, 0.0, 0.0);  // 1500km
    EXPECT_LT(rho, 1e-17);  // Very low but not zero
}

TEST_F(JB08AtmosphereTest, SolarActivityModulation) {
    // Low solar activity
    model.set_space_weather(70.0, 70.0, 5.0);
    Real rho_low = model.get_density(400000.0, 0.0, 0.0, 0.0);

    // High solar activity
    model.set_space_weather(250.0, 250.0, 30.0);
    Real rho_high = model.get_density(400000.0, 0.0, 0.0, 0.0);

    // Higher solar activity should increase density
    EXPECT_GT(rho_high, rho_low);
}

// ============================================================================
// Atmospheric Drag Model Tests
// ============================================================================

class AtmosphericDragTest : public ::testing::Test {
protected:
    AtmosphericDragModel model;
    physics::EntityState state;
    environment::Environment env;
    physics::EntityForces forces;

    void SetUp() override {
        model.set_area(10.0);  // 10 m²
        model.set_cd(2.2);

        state.mass = 1000.0;
        state.orientation = Quat::Identity();
        state.angular_velocity = Vec3{0, 0, 0};

        forces.clear();
    }
};

TEST_F(AtmosphericDragTest, NoDragAtHighAltitude) {
    // Very high altitude (negligible density)
    state.position = transforms::geodetic_to_ecef(0.0, 0.0, 2000000.0);  // 2000km
    state.velocity = Vec3{7500.0, 0.0, 0.0};

    model.compute_forces(state, env, 0.1, forces);

    // Drag should be negligible
    Real drag = std::sqrt(forces.force.x * forces.force.x +
                          forces.force.y * forces.force.y +
                          forces.force.z * forces.force.z);
    EXPECT_LT(drag, 1e-6);
}

TEST_F(AtmosphericDragTest, DragOpposesVelocity) {
    // LEO altitude
    state.position = transforms::geodetic_to_ecef(0.0, 0.0, 400000.0);  // 400km
    state.velocity = Vec3{7500.0, 0.0, 0.0};  // Moving in +X direction

    model.compute_forces(state, env, 0.1, forces);

    // Drag should oppose velocity (negative X)
    EXPECT_LT(forces.force.x, 0.0);
}

TEST_F(AtmosphericDragTest, DragProportionalToVelocitySquared) {
    state.position = transforms::geodetic_to_ecef(0.0, 0.0, 400000.0);

    // Lower velocity
    state.velocity = Vec3{5000.0, 0.0, 0.0};
    model.compute_forces(state, env, 0.1, forces);
    Real drag_slow = std::abs(forces.force.x);

    // Higher velocity
    forces.clear();
    state.velocity = Vec3{10000.0, 0.0, 0.0};
    model.compute_forces(state, env, 0.1, forces);
    Real drag_fast = std::abs(forces.force.x);

    // Drag should scale roughly as v²
    // drag_fast/drag_slow ≈ (10000/5000)² = 4
    Real ratio = drag_fast / drag_slow;
    EXPECT_NEAR(ratio, 4.0, 0.5);
}

TEST_F(AtmosphericDragTest, DisabledModelProducesNoDrag) {
    state.position = transforms::geodetic_to_ecef(0.0, 0.0, 400000.0);
    state.velocity = Vec3{7500.0, 0.0, 0.0};

    model.set_enabled(false);
    model.compute_forces(state, env, 0.1, forces);

    EXPECT_NEAR(forces.force.x, 0.0, 1e-10);
    EXPECT_NEAR(forces.force.y, 0.0, 1e-10);
    EXPECT_NEAR(forces.force.z, 0.0, 1e-10);
}

// ============================================================================
// LVLH (Local Vertical Local Horizontal) Transform Tests
// ============================================================================

class LVLHTransformTest : public ::testing::Test {
protected:
    // Circular orbit at 400 km altitude, equatorial
    // Position: (R, 0, 0), Velocity: (0, V, 0) where V = sqrt(μ/R)
    Vec3 pos_eci;
    Vec3 vel_eci;

    void SetUp() override {
        Real R = EARTH_RADIUS + 400000.0;  // 400 km altitude
        Real V = std::sqrt(MU_EARTH / R);   // Circular orbit velocity

        pos_eci = Vec3{R, 0.0, 0.0};
        vel_eci = Vec3{0.0, V, 0.0};
    }
};

TEST_F(LVLHTransformTest, ECI_LVLH_MatrixOrthogonal) {
    Mat3x3 R = transforms::eci_to_lvlh_matrix(pos_eci, vel_eci);

    // Check orthogonality: R * R^T = I
    Mat3x3 RT = R.transpose();
    Mat3x3 RRT = R * RT;

    // Diagonal elements should be 1
    EXPECT_NEAR(RRT(0, 0), 1.0, 1e-10);
    EXPECT_NEAR(RRT(1, 1), 1.0, 1e-10);
    EXPECT_NEAR(RRT(2, 2), 1.0, 1e-10);

    // Off-diagonal elements should be 0
    EXPECT_NEAR(RRT(0, 1), 0.0, 1e-10);
    EXPECT_NEAR(RRT(0, 2), 0.0, 1e-10);
    EXPECT_NEAR(RRT(1, 2), 0.0, 1e-10);
}

TEST_F(LVLHTransformTest, ECI_LVLH_RoundTrip) {
    Vec3 vec_eci{1000.0, 2000.0, 3000.0};

    Vec3 vec_lvlh = transforms::eci_to_lvlh(vec_eci, pos_eci, vel_eci);
    Vec3 vec_back = transforms::lvlh_to_eci(vec_lvlh, pos_eci, vel_eci);

    EXPECT_NEAR(vec_back.x, vec_eci.x, 1e-6);
    EXPECT_NEAR(vec_back.y, vec_eci.y, 1e-6);
    EXPECT_NEAR(vec_back.z, vec_eci.z, 1e-6);
}

TEST_F(LVLHTransformTest, RadialDirectionIsX) {
    // For circular equatorial orbit at (R, 0, 0),
    // the radial direction should be +X in ECI
    // So position transformed to LVLH should be (R, 0, 0)

    Vec3 pos_lvlh = transforms::eci_to_lvlh(pos_eci, pos_eci, vel_eci);

    Real R = std::sqrt(pos_eci.x * pos_eci.x + pos_eci.y * pos_eci.y + pos_eci.z * pos_eci.z);

    // Radial component should equal R, others should be 0
    EXPECT_NEAR(pos_lvlh.x, R, 1.0);       // Radial
    EXPECT_NEAR(pos_lvlh.y, 0.0, 1e-6);    // Along-track
    EXPECT_NEAR(pos_lvlh.z, 0.0, 1e-6);    // Cross-track
}

TEST_F(LVLHTransformTest, VelocityDirectionIsY) {
    // For circular orbit, velocity is in along-track (S/Y) direction in LVLH
    Vec3 vel_lvlh = transforms::eci_to_lvlh(vel_eci, pos_eci, vel_eci);

    Real V = std::sqrt(vel_eci.x * vel_eci.x + vel_eci.y * vel_eci.y + vel_eci.z * vel_eci.z);

    // Velocity should be mostly along-track
    EXPECT_NEAR(vel_lvlh.x, 0.0, 1.0);     // Radial (small for circular)
    EXPECT_NEAR(vel_lvlh.y, V, 10.0);      // Along-track
    EXPECT_NEAR(vel_lvlh.z, 0.0, 1e-6);    // Cross-track
}

TEST_F(LVLHTransformTest, RelativePositionColocated) {
    // Target at same position as reference
    Vec3 rel_pos = transforms::relative_position_lvlh(pos_eci, pos_eci, vel_eci);

    EXPECT_NEAR(rel_pos.x, 0.0, 1e-6);
    EXPECT_NEAR(rel_pos.y, 0.0, 1e-6);
    EXPECT_NEAR(rel_pos.z, 0.0, 1e-6);
}

TEST_F(LVLHTransformTest, RelativePositionAhead) {
    // Target 1 km ahead (in velocity direction)
    Real R = EARTH_RADIUS + 400000.0;
    Real V = std::sqrt(MU_EARTH / R);

    // Small angular displacement in velocity direction
    Real delta_angle = 1000.0 / R;  // 1 km arc length at orbital radius
    Vec3 target_pos{
        R * std::cos(delta_angle),
        R * std::sin(delta_angle),
        0.0
    };

    Vec3 rel_pos = transforms::relative_position_lvlh(target_pos, pos_eci, vel_eci);

    // Should be mostly along-track (positive Y)
    EXPECT_NEAR(rel_pos.x, 0.0, 10.0);     // Small radial (due to curvature)
    EXPECT_GT(rel_pos.y, 900.0);            // Along-track ~1km
    EXPECT_LT(rel_pos.y, 1100.0);
    EXPECT_NEAR(rel_pos.z, 0.0, 1e-6);     // No cross-track
}

TEST_F(LVLHTransformTest, RelativePositionAbove) {
    // Target 1 km higher (radial direction)
    Vec3 target_pos{pos_eci.x + 1000.0, 0.0, 0.0};

    Vec3 rel_pos = transforms::relative_position_lvlh(target_pos, pos_eci, vel_eci);

    // Should be radial (positive R)
    EXPECT_NEAR(rel_pos.x, 1000.0, 1.0);   // Radial
    EXPECT_NEAR(rel_pos.y, 0.0, 1e-6);     // Along-track
    EXPECT_NEAR(rel_pos.z, 0.0, 1e-6);     // Cross-track
}

TEST_F(LVLHTransformTest, RelativePositionCrossTrack) {
    // Target 1 km to the side (cross-track direction = +Z in ECI for equatorial orbit)
    Vec3 target_pos{pos_eci.x, 0.0, 1000.0};

    Vec3 rel_pos = transforms::relative_position_lvlh(target_pos, pos_eci, vel_eci);

    // Should be cross-track (W direction)
    EXPECT_NEAR(rel_pos.x, 0.0, 1e-6);     // Radial
    EXPECT_NEAR(rel_pos.y, 0.0, 1e-6);     // Along-track
    EXPECT_NEAR(std::abs(rel_pos.z), 1000.0, 1.0);  // Cross-track
}

TEST_F(LVLHTransformTest, AngularVelocityMagnitude) {
    Vec3 omega = transforms::lvlh_angular_velocity(pos_eci, vel_eci);

    // For circular orbit, angular velocity = V/R
    Real R = std::sqrt(pos_eci.x * pos_eci.x + pos_eci.y * pos_eci.y + pos_eci.z * pos_eci.z);
    Real V = std::sqrt(vel_eci.x * vel_eci.x + vel_eci.y * vel_eci.y + vel_eci.z * vel_eci.z);
    Real expected_omega = V / R;

    // Angular velocity should be in W direction (cross-track)
    EXPECT_NEAR(omega.x, 0.0, 1e-10);
    EXPECT_NEAR(omega.y, 0.0, 1e-10);
    EXPECT_NEAR(omega.z, expected_omega, 1e-6);
}

TEST_F(LVLHTransformTest, InclinedOrbit) {
    // Test with 45 degree inclined orbit
    Real R = EARTH_RADIUS + 400000.0;
    Real V = std::sqrt(MU_EARTH / R);
    Real inc = 45.0 * DEG_TO_RAD;

    // Position at ascending node
    Vec3 pos_inclined{R, 0.0, 0.0};
    // Velocity inclined by 45 degrees from equatorial plane
    Vec3 vel_inclined{0.0, V * std::cos(inc), V * std::sin(inc)};

    // Matrix should still be orthogonal
    Mat3x3 R_mat = transforms::eci_to_lvlh_matrix(pos_inclined, vel_inclined);
    Mat3x3 RRT = R_mat * R_mat.transpose();

    EXPECT_NEAR(RRT(0, 0), 1.0, 1e-10);
    EXPECT_NEAR(RRT(1, 1), 1.0, 1e-10);
    EXPECT_NEAR(RRT(2, 2), 1.0, 1e-10);

    // Round-trip should work
    Vec3 test_vec{1000.0, 2000.0, 3000.0};
    Vec3 transformed = transforms::eci_to_lvlh(test_vec, pos_inclined, vel_inclined);
    Vec3 back = transforms::lvlh_to_eci(transformed, pos_inclined, vel_inclined);

    EXPECT_NEAR(back.x, test_vec.x, 1e-6);
    EXPECT_NEAR(back.y, test_vec.y, 1e-6);
    EXPECT_NEAR(back.z, test_vec.z, 1e-6);
}
