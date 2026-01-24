/**
 * @file dis_protocol_tests.cpp
 * @brief Unit tests for DIS (Distributed Interactive Simulation) protocol
 */

#include <gtest/gtest.h>
#include "jaguar/federation/dis_protocol.h"
#include <cmath>

using namespace jaguar;
using namespace jaguar::federation;

// ============================================================================
// PDU Type Tests
// ============================================================================

TEST(PDUTypeTest, EnumValues) {
    EXPECT_EQ(static_cast<UInt8>(PDUType::EntityState), 1);
    EXPECT_EQ(static_cast<UInt8>(PDUType::Fire), 2);
    EXPECT_EQ(static_cast<UInt8>(PDUType::Detonation), 3);
    EXPECT_EQ(static_cast<UInt8>(PDUType::Collision), 4);
    EXPECT_EQ(static_cast<UInt8>(PDUType::Unknown), 255);
}

TEST(PDUTypeTest, ToString) {
    EXPECT_STREQ(pdu_type_to_string(PDUType::EntityState), "EntityState");
    EXPECT_STREQ(pdu_type_to_string(PDUType::Fire), "Fire");
    EXPECT_STREQ(pdu_type_to_string(PDUType::Detonation), "Detonation");
    EXPECT_STREQ(pdu_type_to_string(PDUType::Collision), "Collision");
    EXPECT_STREQ(pdu_type_to_string(PDUType::CreateEntity), "CreateEntity");
    EXPECT_STREQ(pdu_type_to_string(PDUType::RemoveEntity), "RemoveEntity");
    EXPECT_STREQ(pdu_type_to_string(PDUType::Unknown), "Unknown");
}

// ============================================================================
// Dead Reckoning Algorithm Tests
// ============================================================================

TEST(DeadReckoningAlgorithmTest, EnumValues) {
    EXPECT_EQ(static_cast<UInt8>(DeadReckoningAlgorithm::DRM_Static), 0);
    EXPECT_EQ(static_cast<UInt8>(DeadReckoningAlgorithm::DRM_FPW), 1);
    EXPECT_EQ(static_cast<UInt8>(DeadReckoningAlgorithm::DRM_RPW), 2);
    EXPECT_EQ(static_cast<UInt8>(DeadReckoningAlgorithm::DRM_RVW), 3);
    EXPECT_EQ(static_cast<UInt8>(DeadReckoningAlgorithm::DRM_FVW), 4);
}

TEST(DeadReckoningAlgorithmTest, ToString) {
    EXPECT_STREQ(dr_algorithm_to_string(DeadReckoningAlgorithm::DRM_Static), "Static");
    EXPECT_STREQ(dr_algorithm_to_string(DeadReckoningAlgorithm::DRM_FPW), "FPW");
    EXPECT_STREQ(dr_algorithm_to_string(DeadReckoningAlgorithm::DRM_RPW), "RPW");
    EXPECT_STREQ(dr_algorithm_to_string(DeadReckoningAlgorithm::DRM_RVW), "RVW");
    EXPECT_STREQ(dr_algorithm_to_string(DeadReckoningAlgorithm::DRM_FVW), "FVW");
}

// ============================================================================
// Entity Kind Tests
// ============================================================================

TEST(EntityKindTest, EnumValues) {
    EXPECT_EQ(static_cast<UInt8>(EntityKind::Other), 0);
    EXPECT_EQ(static_cast<UInt8>(EntityKind::Platform), 1);
    EXPECT_EQ(static_cast<UInt8>(EntityKind::Munition), 2);
    EXPECT_EQ(static_cast<UInt8>(EntityKind::Lifeform), 3);
    EXPECT_EQ(static_cast<UInt8>(EntityKind::Environmental), 4);
}

TEST(PlatformDomainTest, EnumValues) {
    EXPECT_EQ(static_cast<UInt8>(PlatformDomain::Other), 0);
    EXPECT_EQ(static_cast<UInt8>(PlatformDomain::Land), 1);
    EXPECT_EQ(static_cast<UInt8>(PlatformDomain::Air), 2);
    EXPECT_EQ(static_cast<UInt8>(PlatformDomain::Surface), 3);
    EXPECT_EQ(static_cast<UInt8>(PlatformDomain::Subsurface), 4);
    EXPECT_EQ(static_cast<UInt8>(PlatformDomain::Space), 5);
}

// ============================================================================
// Entity Identifier Tests
// ============================================================================

TEST(EntityIdentifierTest, DefaultConstruction) {
    EntityIdentifier id;
    EXPECT_EQ(id.site, 0);
    EXPECT_EQ(id.application, 0);
    EXPECT_EQ(id.entity, 0);
    EXPECT_FALSE(id.is_valid());
}

TEST(EntityIdentifierTest, ParameterizedConstruction) {
    EntityIdentifier id(100, 200, 300);
    EXPECT_EQ(id.site, 100);
    EXPECT_EQ(id.application, 200);
    EXPECT_EQ(id.entity, 300);
    EXPECT_TRUE(id.is_valid());
}

TEST(EntityIdentifierTest, Equality) {
    EntityIdentifier id1(1, 2, 3);
    EntityIdentifier id2(1, 2, 3);
    EntityIdentifier id3(1, 2, 4);

    EXPECT_EQ(id1, id2);
    EXPECT_NE(id1, id3);
}

TEST(EntityIdentifierTest, Hash) {
    EntityIdentifier::Hash hasher;
    EntityIdentifier id1(1, 2, 3);
    EntityIdentifier id2(1, 2, 3);
    EntityIdentifier id3(1, 2, 4);

    EXPECT_EQ(hasher(id1), hasher(id2));
    EXPECT_NE(hasher(id1), hasher(id3));
}

TEST(EntityIdentifierTest, InvalidIdentifier) {
    EXPECT_FALSE(INVALID_ENTITY_IDENTIFIER.is_valid());
    EXPECT_EQ(INVALID_ENTITY_IDENTIFIER.site, 0);
    EXPECT_EQ(INVALID_ENTITY_IDENTIFIER.application, 0);
    EXPECT_EQ(INVALID_ENTITY_IDENTIFIER.entity, 0);
}

// ============================================================================
// Event Identifier Tests
// ============================================================================

TEST(EventIdentifierTest, DefaultConstruction) {
    EventIdentifier id;
    EXPECT_EQ(id.site, 0);
    EXPECT_EQ(id.application, 0);
    EXPECT_EQ(id.event_number, 0);
}

TEST(EventIdentifierTest, ParameterizedConstruction) {
    EventIdentifier id(10, 20, 30);
    EXPECT_EQ(id.site, 10);
    EXPECT_EQ(id.application, 20);
    EXPECT_EQ(id.event_number, 30);
}

TEST(EventIdentifierTest, Equality) {
    EventIdentifier id1(1, 2, 3);
    EventIdentifier id2(1, 2, 3);
    EventIdentifier id3(1, 2, 4);

    EXPECT_EQ(id1, id2);
    EXPECT_NE(id1, id3);
}

// ============================================================================
// Entity Type Tests
// ============================================================================

TEST(EntityTypeTest, DefaultConstruction) {
    EntityType type;
    EXPECT_EQ(type.kind, EntityKind::Other);
    EXPECT_EQ(type.domain, 0);
    EXPECT_EQ(type.country, 0);
    EXPECT_EQ(type.category, 0);
    EXPECT_EQ(type.subcategory, 0);
    EXPECT_EQ(type.specific, 0);
    EXPECT_EQ(type.extra, 0);
}

TEST(EntityTypeTest, CreatePlatform) {
    // F-16C: Air platform, USA (225), Fighter category
    auto f16 = EntityType::create_platform(PlatformDomain::Air, 225, 1, 3, 0);
    EXPECT_EQ(f16.kind, EntityKind::Platform);
    EXPECT_EQ(f16.domain, static_cast<UInt8>(PlatformDomain::Air));
    EXPECT_EQ(f16.country, 225); // USA
    EXPECT_EQ(f16.category, 1);
}

TEST(EntityTypeTest, CreateMunition) {
    auto aim120 = EntityType::create_munition(225, 1, 2, 3);
    EXPECT_EQ(aim120.kind, EntityKind::Munition);
    EXPECT_EQ(aim120.country, 225);
    EXPECT_EQ(aim120.category, 1);
    EXPECT_EQ(aim120.subcategory, 2);
    EXPECT_EQ(aim120.specific, 3);
}

TEST(EntityTypeTest, Equality) {
    auto type1 = EntityType::create_platform(PlatformDomain::Land, 225, 1, 1, 1);
    auto type2 = EntityType::create_platform(PlatformDomain::Land, 225, 1, 1, 1);
    auto type3 = EntityType::create_platform(PlatformDomain::Air, 225, 1, 1, 1);

    EXPECT_EQ(type1, type2);
    EXPECT_NE(type1, type3);
}

// ============================================================================
// Geodetic Coordinates Tests
// ============================================================================

TEST(GeodeticCoordinatesTest, DefaultConstruction) {
    GeodeticCoordinates coords;
    EXPECT_DOUBLE_EQ(coords.latitude, 0.0);
    EXPECT_DOUBLE_EQ(coords.longitude, 0.0);
    EXPECT_DOUBLE_EQ(coords.altitude, 0.0);
}

TEST(GeodeticCoordinatesTest, FromDegrees) {
    // San Francisco coordinates: 37.7749 N, -122.4194 W, 16m
    auto sf = GeodeticCoordinates::from_degrees(37.7749, -122.4194, 16.0);

    EXPECT_NEAR(sf.latitude_degrees(), 37.7749, 0.0001);
    EXPECT_NEAR(sf.longitude_degrees(), -122.4194, 0.0001);
    EXPECT_DOUBLE_EQ(sf.altitude, 16.0);
}

TEST(GeodeticCoordinatesTest, RadianConversion) {
    GeodeticCoordinates coords(0.5, 1.0, 100.0);

    EXPECT_NEAR(coords.latitude_degrees(), 0.5 * constants::RAD_TO_DEG, 0.0001);
    EXPECT_NEAR(coords.longitude_degrees(), 1.0 * constants::RAD_TO_DEG, 0.0001);
}

// ============================================================================
// Euler Angles Tests
// ============================================================================

TEST(EulerAnglesTest, DefaultConstruction) {
    EulerAngles angles;
    EXPECT_DOUBLE_EQ(angles.psi, 0.0);
    EXPECT_DOUBLE_EQ(angles.theta, 0.0);
    EXPECT_DOUBLE_EQ(angles.phi, 0.0);
}

TEST(EulerAnglesTest, FromDegrees) {
    auto angles = EulerAngles::from_degrees(90.0, 45.0, 30.0);

    EXPECT_NEAR(angles.psi_degrees(), 90.0, 0.0001);
    EXPECT_NEAR(angles.theta_degrees(), 45.0, 0.0001);
    EXPECT_NEAR(angles.phi_degrees(), 30.0, 0.0001);
}

TEST(EulerAnglesTest, RadianValues) {
    EulerAngles angles(constants::PI / 2, constants::PI / 4, constants::PI / 6);

    EXPECT_NEAR(angles.psi_degrees(), 90.0, 0.0001);
    EXPECT_NEAR(angles.theta_degrees(), 45.0, 0.0001);
    EXPECT_NEAR(angles.phi_degrees(), 30.0, 0.0001);
}

// ============================================================================
// Dead Reckoning Parameters Tests
// ============================================================================

TEST(DeadReckoningParametersTest, DefaultConstruction) {
    DeadReckoningParameters params;
    EXPECT_EQ(params.algorithm, DeadReckoningAlgorithm::DRM_Static);
    EXPECT_DOUBLE_EQ(params.linear_acceleration.x, 0.0);
    EXPECT_DOUBLE_EQ(params.angular_velocity.x, 0.0);
}

TEST(DeadReckoningParametersTest, WithAlgorithm) {
    DeadReckoningParameters params(DeadReckoningAlgorithm::DRM_RVW);
    EXPECT_EQ(params.algorithm, DeadReckoningAlgorithm::DRM_RVW);
}

TEST(DeadReckoningParametersTest, FullConstruction) {
    Vec3 accel{1.0, 2.0, 3.0};
    Vec3 angvel{0.1, 0.2, 0.3};
    DeadReckoningParameters params(DeadReckoningAlgorithm::DRM_FVW, accel, angvel);

    EXPECT_EQ(params.algorithm, DeadReckoningAlgorithm::DRM_FVW);
    EXPECT_DOUBLE_EQ(params.linear_acceleration.x, 1.0);
    EXPECT_DOUBLE_EQ(params.linear_acceleration.y, 2.0);
    EXPECT_DOUBLE_EQ(params.linear_acceleration.z, 3.0);
    EXPECT_DOUBLE_EQ(params.angular_velocity.x, 0.1);
}

// ============================================================================
// Entity Marking Tests
// ============================================================================

TEST(EntityMarkingTest, DefaultConstruction) {
    EntityMarking marking;
    EXPECT_EQ(marking.character_set, 1); // ASCII
    EXPECT_STREQ(marking.characters, "");
}

TEST(EntityMarkingTest, StringConstruction) {
    EntityMarking marking("ALPHA01");
    EXPECT_EQ(marking.get_marking(), "ALPHA01");
}

TEST(EntityMarkingTest, SetMarking) {
    EntityMarking marking;
    marking.set_marking("BRAVO22");
    EXPECT_EQ(marking.get_marking(), "BRAVO22");
}

TEST(EntityMarkingTest, Truncation) {
    // Marking should be truncated to DIS_MARKING_LENGTH - 1 characters
    std::string long_marking = "VERY_LONG_MARKING_EXCEEDS_LIMIT";
    EntityMarking marking(long_marking);
    EXPECT_LE(marking.get_marking().length(), DIS_MARKING_LENGTH);
}

// ============================================================================
// Entity Appearance Tests
// ============================================================================

TEST(EntityAppearanceTest, DefaultConstruction) {
    EntityAppearance appearance;
    EXPECT_EQ(appearance.bits, 0);
    EXPECT_EQ(appearance.damage(), 0);
    EXPECT_EQ(appearance.smoke(), 0);
    EXPECT_FALSE(appearance.flaming());
    EXPECT_FALSE(appearance.lights_on());
}

TEST(EntityAppearanceTest, SetDamage) {
    EntityAppearance appearance;

    appearance.set_damage(0);
    EXPECT_EQ(appearance.damage(), 0); // None

    appearance.set_damage(1);
    EXPECT_EQ(appearance.damage(), 1); // Slight

    appearance.set_damage(2);
    EXPECT_EQ(appearance.damage(), 2); // Moderate

    appearance.set_damage(3);
    EXPECT_EQ(appearance.damage(), 3); // Destroyed
}

TEST(EntityAppearanceTest, SetSmoke) {
    EntityAppearance appearance;

    appearance.set_smoke(0);
    EXPECT_EQ(appearance.smoke(), 0); // None

    appearance.set_smoke(2);
    EXPECT_EQ(appearance.smoke(), 2); // Medium
}

TEST(EntityAppearanceTest, SetLights) {
    EntityAppearance appearance;

    EXPECT_FALSE(appearance.lights_on());
    appearance.set_lights(true);
    EXPECT_TRUE(appearance.lights_on());
    appearance.set_lights(false);
    EXPECT_FALSE(appearance.lights_on());
}

TEST(EntityAppearanceTest, SetFlaming) {
    EntityAppearance appearance;

    EXPECT_FALSE(appearance.flaming());
    appearance.set_flaming(true);
    EXPECT_TRUE(appearance.flaming());
    appearance.set_flaming(false);
    EXPECT_FALSE(appearance.flaming());
}

TEST(EntityAppearanceTest, PaintScheme) {
    EntityAppearance appearance;

    appearance.set_paint_scheme(0);
    EXPECT_EQ(appearance.paint_scheme(), 0); // Uniform

    appearance.set_paint_scheme(1);
    EXPECT_EQ(appearance.paint_scheme(), 1); // Camouflage
}

// ============================================================================
// Entity Capabilities Tests
// ============================================================================

TEST(EntityCapabilitiesTest, DefaultConstruction) {
    EntityCapabilities caps;
    EXPECT_EQ(caps.bits, 0);
    EXPECT_FALSE(caps.has_ammunition_supply());
    EXPECT_FALSE(caps.has_fuel_supply());
    EXPECT_FALSE(caps.has_recovery());
    EXPECT_FALSE(caps.has_repair());
}

TEST(EntityCapabilitiesTest, BitFields) {
    EntityCapabilities caps(0b1111); // All 4 capabilities
    EXPECT_TRUE(caps.has_ammunition_supply());
    EXPECT_TRUE(caps.has_fuel_supply());
    EXPECT_TRUE(caps.has_recovery());
    EXPECT_TRUE(caps.has_repair());
}

// ============================================================================
// PDU Header Tests
// ============================================================================

TEST(PDUHeaderTest, DefaultConstruction) {
    PDUHeader header;
    EXPECT_EQ(header.protocol_version, DIS_VERSION);
    EXPECT_EQ(header.exercise_id, 0);
    EXPECT_EQ(header.pdu_type, PDUType::Unknown);
    EXPECT_EQ(header.length, 0);
}

TEST(PDUHeaderTest, WithPDUType) {
    PDUHeader header(PDUType::EntityState);
    EXPECT_EQ(header.pdu_type, PDUType::EntityState);
    EXPECT_EQ(header.protocol_version, DIS_VERSION);
}

// ============================================================================
// Entity State PDU Tests
// ============================================================================

TEST(EntityStatePDUTest, DefaultConstruction) {
    EntityStatePDU pdu;
    EXPECT_EQ(pdu.header.pdu_type, PDUType::EntityState);
    EXPECT_EQ(pdu.header.protocol_family, 1);
    EXPECT_FALSE(pdu.entity_id.is_valid());
    EXPECT_EQ(pdu.force_id, ForceId::Other);
}

TEST(EntityStatePDUTest, SetEntityId) {
    EntityStatePDU pdu;
    pdu.entity_id = EntityIdentifier(1, 2, 3);
    EXPECT_TRUE(pdu.entity_id.is_valid());
    EXPECT_EQ(pdu.entity_id.site, 1);
    EXPECT_EQ(pdu.entity_id.application, 2);
    EXPECT_EQ(pdu.entity_id.entity, 3);
}

TEST(EntityStatePDUTest, SetPosition) {
    EntityStatePDU pdu;
    pdu.entity_location = Vec3{1000.0, 2000.0, 3000.0};
    EXPECT_DOUBLE_EQ(pdu.entity_location.x, 1000.0);
    EXPECT_DOUBLE_EQ(pdu.entity_location.y, 2000.0);
    EXPECT_DOUBLE_EQ(pdu.entity_location.z, 3000.0);
}

TEST(EntityStatePDUTest, SetVelocity) {
    EntityStatePDU pdu;
    pdu.entity_linear_velocity = Vec3{100.0, 0.0, 0.0};
    EXPECT_DOUBLE_EQ(pdu.entity_linear_velocity.x, 100.0);
}

TEST(EntityStatePDUTest, ArticulationParameters) {
    EntityStatePDU pdu;
    EXPECT_TRUE(pdu.articulation_params.empty());

    ArticulationParameter param;
    param.parameter_type = 1;
    param.parameter_value = 45.0;
    pdu.articulation_params.push_back(param);

    EXPECT_EQ(pdu.articulation_params.size(), 1);
    EXPECT_DOUBLE_EQ(pdu.articulation_params[0].parameter_value, 45.0);
}

// ============================================================================
// Fire PDU Tests
// ============================================================================

TEST(FirePDUTest, DefaultConstruction) {
    FirePDU pdu;
    EXPECT_EQ(pdu.header.pdu_type, PDUType::Fire);
    EXPECT_EQ(pdu.header.protocol_family, 1);
    EXPECT_FALSE(pdu.firing_entity_id.is_valid());
    EXPECT_FALSE(pdu.target_entity_id.is_valid());
}

TEST(FirePDUTest, SetEntities) {
    FirePDU pdu;
    pdu.firing_entity_id = EntityIdentifier(1, 1, 1);
    pdu.target_entity_id = EntityIdentifier(1, 1, 2);
    pdu.munition_id = EntityIdentifier(1, 1, 100);

    EXPECT_TRUE(pdu.firing_entity_id.is_valid());
    EXPECT_TRUE(pdu.target_entity_id.is_valid());
    EXPECT_TRUE(pdu.munition_id.is_valid());
}

TEST(FirePDUTest, SetVelocityAndRange) {
    FirePDU pdu;
    pdu.velocity = Vec3{500.0, 0.0, -10.0};
    pdu.range = 5000.0;

    EXPECT_DOUBLE_EQ(pdu.velocity.x, 500.0);
    EXPECT_DOUBLE_EQ(pdu.range, 5000.0);
}

// ============================================================================
// Detonation PDU Tests
// ============================================================================

TEST(DetonationPDUTest, DefaultConstruction) {
    DetonationPDU pdu;
    EXPECT_EQ(pdu.header.pdu_type, PDUType::Detonation);
    EXPECT_EQ(pdu.header.protocol_family, 1);
}

TEST(DetonationPDUTest, SetDetonationResult) {
    DetonationPDU pdu;
    pdu.detonation_result = DetonationResult::EntityImpact;
    EXPECT_EQ(pdu.detonation_result, DetonationResult::EntityImpact);

    pdu.detonation_result = DetonationResult::Miss;
    EXPECT_EQ(pdu.detonation_result, DetonationResult::Miss);
}

TEST(DetonationPDUTest, SetLocation) {
    DetonationPDU pdu;
    pdu.location_in_world = Vec3{10000.0, 20000.0, 100.0};
    pdu.location_in_entity = Vec3{0.5, 0.0, 0.0};

    EXPECT_DOUBLE_EQ(pdu.location_in_world.x, 10000.0);
    EXPECT_DOUBLE_EQ(pdu.location_in_entity.x, 0.5);
}

// ============================================================================
// Collision PDU Tests
// ============================================================================

TEST(CollisionPDUTest, DefaultConstruction) {
    CollisionPDU pdu;
    EXPECT_EQ(pdu.header.pdu_type, PDUType::Collision);
    EXPECT_EQ(pdu.header.protocol_family, 1);
    EXPECT_EQ(pdu.collision_type, 0);
}

TEST(CollisionPDUTest, SetMass) {
    CollisionPDU pdu;
    pdu.mass = 15000.0; // 15 tons
    pdu.velocity = Vec3{10.0, 0.0, 0.0};

    EXPECT_DOUBLE_EQ(pdu.mass, 15000.0);
}

// ============================================================================
// Start/Resume and Stop/Freeze PDU Tests
// ============================================================================

TEST(StartResumePDUTest, DefaultConstruction) {
    StartResumePDU pdu;
    EXPECT_EQ(pdu.header.pdu_type, PDUType::StartResume);
    EXPECT_EQ(pdu.header.protocol_family, 4); // Simulation Management
}

TEST(StopFreezePDUTest, DefaultConstruction) {
    StopFreezePDU pdu;
    EXPECT_EQ(pdu.header.pdu_type, PDUType::StopFreeze);
    EXPECT_EQ(pdu.header.protocol_family, 4);
}

// ============================================================================
// Burst Descriptor Tests
// ============================================================================

TEST(BurstDescriptorTest, DefaultConstruction) {
    BurstDescriptor burst;
    EXPECT_EQ(burst.warhead, 0);
    EXPECT_EQ(burst.fuse, 0);
    EXPECT_EQ(burst.quantity, 1);
    EXPECT_EQ(burst.rate, 0);
}

// ============================================================================
// Coordinate Transformation Tests
// ============================================================================

TEST(CoordinateTransformTest, LLAToGeocentricOrigin) {
    // Test at origin (0, 0, 0)
    GeodeticCoordinates origin(0.0, 0.0, 0.0);
    Vec3 ecef = lla_to_geocentric(origin);

    // At the equator on the prime meridian, ECEF X should be ~6378137m (WGS84 semi-major axis)
    EXPECT_NEAR(ecef.x, 6378137.0, 1.0);
    EXPECT_NEAR(ecef.y, 0.0, 1.0);
    EXPECT_NEAR(ecef.z, 0.0, 1.0);
}

TEST(CoordinateTransformTest, LLAToGeocentricNorthPole) {
    // North pole: 90 degrees latitude
    GeodeticCoordinates north_pole(constants::PI / 2, 0.0, 0.0);
    Vec3 ecef = lla_to_geocentric(north_pole);

    // At north pole, X and Y should be ~0, Z should be ~6356752m (semi-minor axis)
    EXPECT_NEAR(ecef.x, 0.0, 1.0);
    EXPECT_NEAR(ecef.y, 0.0, 1.0);
    EXPECT_NEAR(ecef.z, 6356752.0, 100.0); // Approximate due to WGS84 parameters
}

TEST(CoordinateTransformTest, RoundTrip) {
    // Test round-trip conversion
    GeodeticCoordinates original = GeodeticCoordinates::from_degrees(37.7749, -122.4194, 100.0);
    Vec3 ecef = lla_to_geocentric(original);
    GeodeticCoordinates converted = geocentric_to_lla(ecef);

    EXPECT_NEAR(converted.latitude_degrees(), original.latitude_degrees(), 0.0001);
    EXPECT_NEAR(converted.longitude_degrees(), original.longitude_degrees(), 0.0001);
    EXPECT_NEAR(converted.altitude, original.altitude, 1.0);
}

// ============================================================================
// Euler/Quaternion Conversion Tests
// ============================================================================

TEST(OrientationConversionTest, IdentityEuler) {
    EulerAngles euler(0.0, 0.0, 0.0);
    Quat q = euler_to_orientation(euler);

    // Identity quaternion: w=1, x=y=z=0
    EXPECT_NEAR(q.w, 1.0, 0.0001);
    EXPECT_NEAR(q.x, 0.0, 0.0001);
    EXPECT_NEAR(q.y, 0.0, 0.0001);
    EXPECT_NEAR(q.z, 0.0, 0.0001);
}

TEST(OrientationConversionTest, RoundTrip) {
    EulerAngles original = EulerAngles::from_degrees(45.0, 30.0, 15.0);
    Quat q = euler_to_orientation(original);
    EulerAngles converted = orientation_to_euler(q);

    EXPECT_NEAR(converted.psi_degrees(), original.psi_degrees(), 0.1);
    EXPECT_NEAR(converted.theta_degrees(), original.theta_degrees(), 0.1);
    EXPECT_NEAR(converted.phi_degrees(), original.phi_degrees(), 0.1);
}

// ============================================================================
// Dead Reckoning Calculator Tests
// ============================================================================

TEST(DeadReckoningCalculatorTest, StaticExtrapolation) {
    DeadReckoningCalculator calc;

    EntityStatePDU state;
    state.entity_location = Vec3{1000.0, 2000.0, 3000.0};
    state.entity_linear_velocity = Vec3{100.0, 0.0, 0.0};
    state.dead_reckoning_params.algorithm = DeadReckoningAlgorithm::DRM_Static;

    // Static algorithm should not change position
    auto extrapolated = calc.extrapolate_position(state, 1.0);
    EXPECT_DOUBLE_EQ(extrapolated.entity_location.x, 1000.0);
    EXPECT_DOUBLE_EQ(extrapolated.entity_location.y, 2000.0);
    EXPECT_DOUBLE_EQ(extrapolated.entity_location.z, 3000.0);
}

TEST(DeadReckoningCalculatorTest, FVWExtrapolation) {
    DeadReckoningCalculator calc;

    EntityStatePDU state;
    state.entity_location = Vec3{0.0, 0.0, 0.0};
    state.entity_linear_velocity = Vec3{100.0, 0.0, 0.0}; // 100 m/s along X
    state.dead_reckoning_params.algorithm = DeadReckoningAlgorithm::DRM_FVW;

    // After 1 second, should have moved 100m along X
    auto extrapolated = calc.extrapolate_position(state, 1.0);
    EXPECT_NEAR(extrapolated.entity_location.x, 100.0, 0.1);
    EXPECT_NEAR(extrapolated.entity_location.y, 0.0, 0.1);
}

TEST(DeadReckoningCalculatorTest, PositionError) {
    DeadReckoningCalculator calc;

    EntityStatePDU state1;
    state1.entity_location = Vec3{0.0, 0.0, 0.0};

    EntityStatePDU state2;
    state2.entity_location = Vec3{3.0, 4.0, 0.0};

    Real error = calc.calculate_position_error(state1, state2);
    EXPECT_NEAR(error, 5.0, 0.0001); // 3-4-5 triangle
}

TEST(DeadReckoningCalculatorTest, ShouldSendUpdate) {
    DeadReckoningCalculator calc;

    EntityStatePDU current;
    current.entity_location = Vec3{0.0, 0.0, 0.0};

    EntityStatePDU predicted;
    predicted.entity_location = Vec3{0.5, 0.0, 0.0}; // Within threshold

    // Should not send update if within threshold
    EXPECT_FALSE(calc.should_send_update(current, predicted, 1.0));

    predicted.entity_location = Vec3{5.0, 0.0, 0.0}; // Outside threshold
    EXPECT_TRUE(calc.should_send_update(current, predicted, 1.0));
}

// ============================================================================
// DIS Codec Factory Test
// ============================================================================

TEST(DISCodecTest, CreateCodec) {
    auto codec = create_dis_codec();
    EXPECT_NE(codec, nullptr);
}

// ============================================================================
// DIS Network Factory Test
// ============================================================================

TEST(DISNetworkTest, CreateNetwork) {
    auto network = create_dis_network();
    EXPECT_NE(network, nullptr);
}

// ============================================================================
// Timestamp Tests
// ============================================================================

TEST(TimestampTest, GetTimestamp) {
    UInt32 ts1 = get_dis_timestamp();
    UInt32 ts2 = get_dis_timestamp();

    // Timestamps should be non-negative and close together
    EXPECT_GE(ts2, ts1);
}

TEST(TimestampTest, ConvertToSeconds) {
    UInt32 timestamp = seconds_to_dis_timestamp(3600.0); // 1 hour
    Real seconds = dis_timestamp_to_seconds(timestamp);

    EXPECT_NEAR(seconds, 3600.0, 0.001);
}

TEST(TimestampTest, RoundTrip) {
    Real original = 1234.567;
    UInt32 timestamp = seconds_to_dis_timestamp(original);
    Real converted = dis_timestamp_to_seconds(timestamp);

    EXPECT_NEAR(converted, original, 0.001);
}

// ============================================================================
// Distance Calculation Tests
// ============================================================================

TEST(DistanceTest, ECEF) {
    Vec3 a{0.0, 0.0, 0.0};
    Vec3 b{3.0, 4.0, 0.0};

    EXPECT_NEAR(distance_ecef(a, b), 5.0, 0.0001);
}

TEST(DistanceTest, LLA) {
    // New York: 40.7128 N, -74.0060 W
    // Los Angeles: 34.0522 N, -118.2437 W
    auto ny = GeodeticCoordinates::from_degrees(40.7128, -74.0060, 0.0);
    auto la = GeodeticCoordinates::from_degrees(34.0522, -118.2437, 0.0);

    Real dist = distance_lla(ny, la);

    // Distance should be approximately 3936 km
    EXPECT_NEAR(dist, 3936000.0, 50000.0); // Within 50km tolerance
}

// ============================================================================
// Constants Tests
// ============================================================================

TEST(DISConstantsTest, ProtocolVersion) {
    EXPECT_EQ(DIS_VERSION, 7);
}

TEST(DISConstantsTest, MaxPDUSize) {
    EXPECT_EQ(DIS_MAX_PDU_SIZE, 8192);
}

TEST(DISConstantsTest, MarkingLength) {
    EXPECT_EQ(DIS_MARKING_LENGTH, 11);
}

TEST(DISConstantsTest, HeartbeatInterval) {
    EXPECT_DOUBLE_EQ(DIS_HEARTBEAT_INTERVAL, 5.0);
}

TEST(DISConstantsTest, DeadReckoningThresholds) {
    EXPECT_DOUBLE_EQ(DIS_DR_THRESHOLD_POSITION, 1.0);
    EXPECT_DOUBLE_EQ(DIS_DR_THRESHOLD_ORIENTATION, 3.0);
}

// ============================================================================
// Force ID Tests
// ============================================================================

TEST(ForceIdTest, EnumValues) {
    EXPECT_EQ(static_cast<UInt8>(ForceId::Other), 0);
    EXPECT_EQ(static_cast<UInt8>(ForceId::Friendly), 1);
    EXPECT_EQ(static_cast<UInt8>(ForceId::Opposing), 2);
    EXPECT_EQ(static_cast<UInt8>(ForceId::Neutral), 3);
}

// ============================================================================
// Detonation Result Tests
// ============================================================================

TEST(DetonationResultTest, EnumValues) {
    EXPECT_EQ(static_cast<UInt8>(DetonationResult::Other), 0);
    EXPECT_EQ(static_cast<UInt8>(DetonationResult::EntityImpact), 1);
    EXPECT_EQ(static_cast<UInt8>(DetonationResult::Miss), 30);
}
