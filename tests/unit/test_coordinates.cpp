/**
 * @file test_coordinates.cpp
 * @brief Unit tests for coordinate transformation utilities
 */

#include <gtest/gtest.h>
#include "jaguar/core/coordinates.h"
#include <cmath>

using namespace jaguar;
using namespace jaguar::coord;

// ============================================================================
// Helper Functions
// ============================================================================

namespace {

bool nearly_equal(Real a, Real b, Real eps = 1e-6) {
    return std::abs(a - b) < eps;
}

bool nearly_equal_vec(const Vec3& a, const Vec3& b, Real eps = 1e-6) {
    return nearly_equal(a.x, b.x, eps) &&
           nearly_equal(a.y, b.y, eps) &&
           nearly_equal(a.z, b.z, eps);
}

} // anonymous namespace

// ============================================================================
// Geodetic Position Tests
// ============================================================================

class GeodeticPositionTest : public ::testing::Test {
protected:
    static constexpr Real EPSILON = 1e-10;
};

TEST_F(GeodeticPositionTest, DefaultConstruction) {
    GeodeticPosition lla;
    EXPECT_DOUBLE_EQ(lla.latitude, 0.0);
    EXPECT_DOUBLE_EQ(lla.longitude, 0.0);
    EXPECT_DOUBLE_EQ(lla.altitude, 0.0);
}

TEST_F(GeodeticPositionTest, FromDegrees) {
    GeodeticPosition lla = GeodeticPosition::from_degrees(45.0, -122.0, 100.0);

    EXPECT_NEAR(lla.latitude, 45.0 * constants::DEG_TO_RAD, EPSILON);
    EXPECT_NEAR(lla.longitude, -122.0 * constants::DEG_TO_RAD, EPSILON);
    EXPECT_DOUBLE_EQ(lla.altitude, 100.0);
}

TEST_F(GeodeticPositionTest, ToDegrees) {
    GeodeticPosition lla{
        45.0 * constants::DEG_TO_RAD,
        -122.0 * constants::DEG_TO_RAD,
        100.0
    };

    Real lat_deg, lon_deg, alt_m;
    lla.to_degrees(lat_deg, lon_deg, alt_m);

    EXPECT_NEAR(lat_deg, 45.0, EPSILON);
    EXPECT_NEAR(lon_deg, -122.0, EPSILON);
    EXPECT_DOUBLE_EQ(alt_m, 100.0);
}

// ============================================================================
// ECEF <-> LLA Conversion Tests
// ============================================================================

class ECEFToLLATest : public ::testing::Test {
protected:
    static constexpr Real POS_EPSILON = 0.1;    // 10 cm position accuracy
    static constexpr Real ANGLE_EPSILON = 1e-8; // ~0.003 arcsec angle accuracy
};

TEST_F(ECEFToLLATest, EquatorPrimeMeridian) {
    // Point on equator at prime meridian, sea level
    GeodeticPosition lla{0.0, 0.0, 0.0};
    Vec3 ecef = lla_to_ecef(lla);

    // Should be on positive X axis at equatorial radius
    EXPECT_NEAR(ecef.x, wgs84::a, POS_EPSILON);
    EXPECT_NEAR(ecef.y, 0.0, POS_EPSILON);
    EXPECT_NEAR(ecef.z, 0.0, POS_EPSILON);

    // Round-trip test
    GeodeticPosition lla_back = ecef_to_lla(ecef);
    EXPECT_NEAR(lla_back.latitude, lla.latitude, ANGLE_EPSILON);
    EXPECT_NEAR(lla_back.longitude, lla.longitude, ANGLE_EPSILON);
    EXPECT_NEAR(lla_back.altitude, lla.altitude, POS_EPSILON);
}

TEST_F(ECEFToLLATest, NorthPole) {
    // North pole, sea level
    GeodeticPosition lla{constants::PI / 2.0, 0.0, 0.0};
    Vec3 ecef = lla_to_ecef(lla);

    // Should be on Z axis at polar radius
    EXPECT_NEAR(ecef.x, 0.0, POS_EPSILON);
    EXPECT_NEAR(ecef.y, 0.0, POS_EPSILON);
    EXPECT_NEAR(ecef.z, wgs84::b, POS_EPSILON);

    // Round-trip test
    GeodeticPosition lla_back = ecef_to_lla(ecef);
    EXPECT_NEAR(lla_back.latitude, lla.latitude, ANGLE_EPSILON);
    // Longitude is undefined at pole, don't check it
    EXPECT_NEAR(lla_back.altitude, lla.altitude, POS_EPSILON);
}

TEST_F(ECEFToLLATest, SouthPole) {
    // South pole, sea level
    GeodeticPosition lla{-constants::PI / 2.0, 0.0, 0.0};
    Vec3 ecef = lla_to_ecef(lla);

    // Should be on negative Z axis at polar radius
    EXPECT_NEAR(ecef.x, 0.0, POS_EPSILON);
    EXPECT_NEAR(ecef.y, 0.0, POS_EPSILON);
    EXPECT_NEAR(ecef.z, -wgs84::b, POS_EPSILON);

    // Round-trip test
    GeodeticPosition lla_back = ecef_to_lla(ecef);
    EXPECT_NEAR(lla_back.latitude, lla.latitude, ANGLE_EPSILON);
    EXPECT_NEAR(lla_back.altitude, lla.altitude, POS_EPSILON);
}

TEST_F(ECEFToLLATest, Equator90Degrees) {
    // Point on equator at 90 degrees east
    GeodeticPosition lla{0.0, constants::PI / 2.0, 0.0};
    Vec3 ecef = lla_to_ecef(lla);

    // Should be on positive Y axis
    EXPECT_NEAR(ecef.x, 0.0, POS_EPSILON);
    EXPECT_NEAR(ecef.y, wgs84::a, POS_EPSILON);
    EXPECT_NEAR(ecef.z, 0.0, POS_EPSILON);

    // Round-trip test
    GeodeticPosition lla_back = ecef_to_lla(ecef);
    EXPECT_NEAR(lla_back.latitude, lla.latitude, ANGLE_EPSILON);
    EXPECT_NEAR(lla_back.longitude, lla.longitude, ANGLE_EPSILON);
    EXPECT_NEAR(lla_back.altitude, lla.altitude, POS_EPSILON);
}

TEST_F(ECEFToLLATest, WithAltitude) {
    // Seattle at 10km altitude
    GeodeticPosition lla = GeodeticPosition::from_degrees(47.6062, -122.3321, 10000.0);
    Vec3 ecef = lla_to_ecef(lla);

    // Round-trip test
    GeodeticPosition lla_back = ecef_to_lla(ecef);
    EXPECT_NEAR(lla_back.latitude, lla.latitude, ANGLE_EPSILON);
    EXPECT_NEAR(lla_back.longitude, lla.longitude, ANGLE_EPSILON);
    EXPECT_NEAR(lla_back.altitude, lla.altitude, POS_EPSILON);
}

TEST_F(ECEFToLLATest, NegativeAltitude) {
    // Below sea level (Dead Sea)
    GeodeticPosition lla = GeodeticPosition::from_degrees(31.5, 35.5, -430.0);
    Vec3 ecef = lla_to_ecef(lla);

    // Round-trip test
    GeodeticPosition lla_back = ecef_to_lla(ecef);
    EXPECT_NEAR(lla_back.latitude, lla.latitude, ANGLE_EPSILON);
    EXPECT_NEAR(lla_back.longitude, lla.longitude, ANGLE_EPSILON);
    EXPECT_NEAR(lla_back.altitude, lla.altitude, 1.0);  // Looser tolerance for negative altitude
}

// ============================================================================
// NED Transformation Tests
// ============================================================================

class NEDTransformTest : public ::testing::Test {
protected:
    static constexpr Real EPSILON = 1e-6;

    GeodeticPosition seattle_lla{
        47.6062 * constants::DEG_TO_RAD,
        -122.3321 * constants::DEG_TO_RAD,
        0.0
    };
};

TEST_F(NEDTransformTest, MatrixTranspose) {
    // NED to ECEF should be transpose of ECEF to NED
    Mat3x3 ecef_to_ned = ecef_to_ned_matrix(seattle_lla);
    Mat3x3 ned_to_ecef = ned_to_ecef_matrix(seattle_lla);

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_NEAR(ecef_to_ned(i, j), ned_to_ecef(j, i), EPSILON);
        }
    }
}

TEST_F(NEDTransformTest, RoundTrip) {
    Vec3 original{100.0, 200.0, 300.0};

    Vec3 ned = ecef_to_ned(original, seattle_lla);
    Vec3 back = ned_to_ecef(ned, seattle_lla);

    EXPECT_TRUE(nearly_equal_vec(original, back, EPSILON));
}

TEST_F(NEDTransformTest, NorthVectorAtEquator) {
    GeodeticPosition equator{0.0, 0.0, 0.0};

    // At equator/prime meridian, North points toward -X in ECEF
    Vec3 north_ned{1.0, 0.0, 0.0};  // North in NED
    Vec3 north_ecef = ned_to_ecef(north_ned, equator);

    EXPECT_NEAR(north_ecef.x, 0.0, EPSILON);
    EXPECT_NEAR(north_ecef.y, 0.0, EPSILON);
    EXPECT_NEAR(north_ecef.z, 1.0, EPSILON);
}

TEST_F(NEDTransformTest, DownVectorAtEquator) {
    GeodeticPosition equator{0.0, 0.0, 0.0};

    // At equator/prime meridian, Down points toward -X in ECEF (toward center)
    Vec3 down_ned{0.0, 0.0, 1.0};  // Down in NED
    Vec3 down_ecef = ned_to_ecef(down_ned, equator);

    EXPECT_NEAR(down_ecef.x, -1.0, EPSILON);
    EXPECT_NEAR(down_ecef.y, 0.0, EPSILON);
    EXPECT_NEAR(down_ecef.z, 0.0, EPSILON);
}

// ============================================================================
// ENU Transformation Tests
// ============================================================================

class ENUTransformTest : public ::testing::Test {
protected:
    static constexpr Real EPSILON = 1e-6;

    GeodeticPosition seattle_lla{
        47.6062 * constants::DEG_TO_RAD,
        -122.3321 * constants::DEG_TO_RAD,
        0.0
    };
};

TEST_F(ENUTransformTest, RoundTrip) {
    Vec3 original{100.0, 200.0, 300.0};

    Vec3 enu = ecef_to_enu(original, seattle_lla);
    Vec3 back = enu_to_ecef(enu, seattle_lla);

    EXPECT_TRUE(nearly_equal_vec(original, back, EPSILON));
}

TEST_F(ENUTransformTest, NEDToENUConversion) {
    Vec3 ned{100.0, 200.0, 300.0};  // North, East, Down
    Vec3 enu = ned_to_enu(ned);

    // ENU = (East, North, Up) = (ned.y, ned.x, -ned.z)
    EXPECT_DOUBLE_EQ(enu.x, 200.0);  // East
    EXPECT_DOUBLE_EQ(enu.y, 100.0);  // North
    EXPECT_DOUBLE_EQ(enu.z, -300.0); // Up
}

TEST_F(ENUTransformTest, ENUToNEDConversion) {
    Vec3 enu{100.0, 200.0, 300.0};  // East, North, Up
    Vec3 ned = enu_to_ned(enu);

    // NED = (North, East, Down) = (enu.y, enu.x, -enu.z)
    EXPECT_DOUBLE_EQ(ned.x, 200.0);  // North
    EXPECT_DOUBLE_EQ(ned.y, 100.0);  // East
    EXPECT_DOUBLE_EQ(ned.z, -300.0); // Down
}

TEST_F(ENUTransformTest, NEDENURoundTrip) {
    Vec3 original{100.0, 200.0, 300.0};
    Vec3 enu = ned_to_enu(original);
    Vec3 back = enu_to_ned(enu);

    EXPECT_TRUE(nearly_equal_vec(original, back, EPSILON));
}

// ============================================================================
// Body Frame Transformation Tests
// ============================================================================

class BodyFrameTest : public ::testing::Test {
protected:
    static constexpr Real EPSILON = 1e-6;
};

TEST_F(BodyFrameTest, IdentityOrientation) {
    Quat identity = Quat::Identity();
    Vec3 ned{1.0, 2.0, 3.0};

    Vec3 body = ned_to_body(ned, identity);
    EXPECT_TRUE(nearly_equal_vec(ned, body, EPSILON));

    Vec3 back = body_to_ned(body, identity);
    EXPECT_TRUE(nearly_equal_vec(ned, back, EPSILON));
}

TEST_F(BodyFrameTest, RoundTrip) {
    // 45 degree roll
    Quat orientation = Quat::from_euler(constants::PI / 4.0, 0.0, 0.0);
    Vec3 original{100.0, 200.0, 300.0};

    Vec3 body = ned_to_body(original, orientation);
    Vec3 back = body_to_ned(body, orientation);

    EXPECT_TRUE(nearly_equal_vec(original, back, EPSILON));
}

TEST_F(BodyFrameTest, Rotation90DegreesYaw) {
    // 90 degree yaw (heading change)
    Quat orientation = Quat::from_euler(0.0, 0.0, constants::PI / 2.0);
    Vec3 north_ned{1.0, 0.0, 0.0};  // North vector

    Vec3 body = ned_to_body(north_ned, orientation);

    // After 90 degree yaw, North (NED X) becomes body -Y
    EXPECT_NEAR(body.x, 0.0, EPSILON);
    EXPECT_NEAR(body.y, -1.0, EPSILON);
    EXPECT_NEAR(body.z, 0.0, EPSILON);
}

// ============================================================================
// Geodetic Utility Tests
// ============================================================================

class GeodeticUtilityTest : public ::testing::Test {
protected:
    static constexpr Real EPSILON = 1e-3;  // 1mm accuracy
};

TEST_F(GeodeticUtilityTest, RadiusOfCurvatureN) {
    // At equator
    Real N_equator = radius_of_curvature_n(0.0);
    EXPECT_NEAR(N_equator, wgs84::a, 1.0);

    // At pole (should be larger due to flattening)
    Real N_pole = radius_of_curvature_n(constants::PI / 2.0);
    EXPECT_GT(N_pole, N_equator);
}

TEST_F(GeodeticUtilityTest, RadiusOfCurvatureM) {
    // At equator
    Real M_equator = radius_of_curvature_m(0.0);
    EXPECT_LT(M_equator, wgs84::a);  // M < a at equator

    // At pole
    Real M_pole = radius_of_curvature_m(constants::PI / 2.0);
    EXPECT_GT(M_pole, M_equator);  // M increases toward pole
}

TEST_F(GeodeticUtilityTest, GreatCircleDistanceSamePoint) {
    GeodeticPosition lla = GeodeticPosition::from_degrees(45.0, -122.0, 0.0);
    Real dist = great_circle_distance(lla, lla);
    EXPECT_NEAR(dist, 0.0, EPSILON);
}

TEST_F(GeodeticUtilityTest, GreatCircleDistanceKnownPoints) {
    // New York to London (approximately 5570 km)
    GeodeticPosition nyc = GeodeticPosition::from_degrees(40.7128, -74.0060, 0.0);
    GeodeticPosition london = GeodeticPosition::from_degrees(51.5074, -0.1278, 0.0);

    Real dist = great_circle_distance(nyc, london);
    EXPECT_NEAR(dist, 5570000.0, 20000.0);  // Within 20km tolerance
}

TEST_F(GeodeticUtilityTest, InitialBearingNorth) {
    GeodeticPosition start = GeodeticPosition::from_degrees(0.0, 0.0, 0.0);
    GeodeticPosition end = GeodeticPosition::from_degrees(10.0, 0.0, 0.0);

    Real bearing = initial_bearing(start, end);
    EXPECT_NEAR(bearing, 0.0, 0.01);  // Should be due north
}

TEST_F(GeodeticUtilityTest, InitialBearingEast) {
    GeodeticPosition start = GeodeticPosition::from_degrees(0.0, 0.0, 0.0);
    GeodeticPosition end = GeodeticPosition::from_degrees(0.0, 10.0, 0.0);

    Real bearing = initial_bearing(start, end);
    EXPECT_NEAR(bearing, constants::PI / 2.0, 0.01);  // Should be due east
}

TEST_F(GeodeticUtilityTest, DestinationPointRoundTrip) {
    GeodeticPosition start = GeodeticPosition::from_degrees(45.0, -122.0, 1000.0);
    Real bearing = constants::PI / 4.0;  // NE
    Real distance = 10000.0;  // 10 km

    GeodeticPosition dest = destination_point(start, bearing, distance);

    // Destination should be NE of start
    EXPECT_GT(dest.latitude, start.latitude);
    EXPECT_GT(dest.longitude, start.longitude);

    // Distance from start to dest should be approximately the input distance
    Real computed_dist = great_circle_distance(start, dest);
    EXPECT_NEAR(computed_dist, distance, 100.0);  // Within 100m
}

// ============================================================================
// Math Utility Tests
// ============================================================================

class MathUtilityTest : public ::testing::Test {
protected:
    static constexpr Real EPSILON = 1e-10;
};

TEST_F(MathUtilityTest, NormalizeAnglePositive) {
    Real angle = 3.0 * constants::PI;  // 540 degrees
    Real normalized = math::normalize_angle(angle);
    // 3π - 2π = π, but since our range is (-π, π], π could be represented as π or -π
    // The function normalizes to [-π, π], so 3π becomes π
    EXPECT_NEAR(std::abs(normalized), constants::PI, EPSILON);
}

TEST_F(MathUtilityTest, NormalizeAngleNegative) {
    Real angle = -3.0 * constants::PI;  // -540 degrees
    Real normalized = math::normalize_angle(angle);
    // -3π + 2π = -π, which is within [-π, π]
    EXPECT_NEAR(std::abs(normalized), constants::PI, EPSILON);
}

TEST_F(MathUtilityTest, NormalizeAngleAlreadyNormalized) {
    Real angle = constants::PI / 4.0;  // 45 degrees
    Real normalized = math::normalize_angle(angle);
    EXPECT_NEAR(normalized, angle, EPSILON);
}

TEST_F(MathUtilityTest, NormalizeAnglePositiveRange) {
    Real angle = -constants::PI / 2.0;  // -90 degrees
    Real normalized = math::normalize_angle_positive(angle);
    EXPECT_NEAR(normalized, 3.0 * constants::PI / 2.0, EPSILON);  // 270 degrees
}

TEST_F(MathUtilityTest, DegToRad) {
    EXPECT_NEAR(math::deg_to_rad(180.0), constants::PI, EPSILON);
    EXPECT_NEAR(math::deg_to_rad(90.0), constants::PI / 2.0, EPSILON);
    EXPECT_NEAR(math::deg_to_rad(45.0), constants::PI / 4.0, EPSILON);
}

TEST_F(MathUtilityTest, RadToDeg) {
    EXPECT_NEAR(math::rad_to_deg(constants::PI), 180.0, EPSILON);
    EXPECT_NEAR(math::rad_to_deg(constants::PI / 2.0), 90.0, EPSILON);
    EXPECT_NEAR(math::rad_to_deg(constants::PI / 4.0), 45.0, EPSILON);
}

// ============================================================================
// Quaternion Frame Conversion Tests
// ============================================================================

class QuaternionFrameTest : public ::testing::Test {
protected:
    static constexpr Real EPSILON = 1e-6;

    GeodeticPosition seattle_lla{
        47.6062 * constants::DEG_TO_RAD,
        -122.3321 * constants::DEG_TO_RAD,
        0.0
    };
};

TEST_F(QuaternionFrameTest, ECEFToNEDQuatRoundTrip) {
    Quat q_ecef = Quat::from_euler(0.1, 0.2, 0.3);

    Quat q_ned = ecef_quat_to_ned_quat(q_ecef, seattle_lla);
    Quat q_back = ned_quat_to_ecef_quat(q_ned, seattle_lla);

    // Should be equivalent rotation (q and -q represent same rotation)
    Real dot = q_ecef.w * q_back.w + q_ecef.x * q_back.x +
               q_ecef.y * q_back.y + q_ecef.z * q_back.z;
    EXPECT_NEAR(std::abs(dot), 1.0, EPSILON);
}

TEST_F(QuaternionFrameTest, ECEFToNEDQuatIsUnitQuat) {
    Quat q = ecef_to_ned_quat(seattle_lla);
    Real norm = q.norm();
    EXPECT_NEAR(norm, 1.0, EPSILON);
}
