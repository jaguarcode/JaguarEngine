/**
 * @file dis_fire_pdu_tests.cpp
 * @brief Unit tests for DIS Fire and Detonation PDU handlers
 */

#include <gtest/gtest.h>
#include "jaguar/federation/dis_fire_pdu.h"
#include <cstring>

using namespace jaguar;
using namespace jaguar::federation;

// ============================================================================
// FirePDUHandler Tests
// ============================================================================

class FirePDUHandlerTest : public ::testing::Test {
protected:
    void SetUp() override {
        handler = std::make_unique<FirePDUHandler>();
    }

    std::unique_ptr<FirePDUHandler> handler;
};

TEST_F(FirePDUHandlerTest, CreateFirePDU) {
    EntityIdentifier firing_entity{1, 1, 100};
    EntityIdentifier target_entity{1, 1, 200};
    EntityType munition_type = EntityType::create_munition(225, 1, 2);  // US missile
    Vec3 location{1000000.0, 2000000.0, 3000000.0};
    Vec3 velocity{500.0, 100.0, -50.0};
    Real range = 5000.0;

    FirePDU pdu = handler->create_fire_pdu(
        firing_entity,
        target_entity,
        munition_type,
        location,
        velocity,
        range
    );

    EXPECT_EQ(pdu.header.pdu_type, PDUType::Fire);
    EXPECT_EQ(pdu.header.protocol_family, 1);
    EXPECT_EQ(pdu.firing_entity_id, firing_entity);
    EXPECT_EQ(pdu.target_entity_id, target_entity);
    EXPECT_EQ(pdu.location_in_world, location);
    EXPECT_EQ(pdu.velocity, velocity);
    EXPECT_EQ(pdu.range, range);
    EXPECT_EQ(pdu.burst_descriptor.munition_type, munition_type);
}

TEST_F(FirePDUHandlerTest, SerializeDeserialize) {
    // Create a Fire PDU
    FirePDU original;
    original.header.pdu_type = PDUType::Fire;
    original.header.protocol_family = 1;
    original.header.protocol_version = DIS_VERSION;
    original.header.exercise_id = 1;
    original.header.timestamp = 12345678;

    original.firing_entity_id = EntityIdentifier{1, 2, 100};
    original.target_entity_id = EntityIdentifier{1, 2, 200};
    original.munition_id = EntityIdentifier{1, 2, 300};
    original.event_id = EventIdentifier{1, 2, 5000};

    original.fire_mission_index = 42;
    original.location_in_world = Vec3{1000000.0, 2000000.0, 3000000.0};
    original.velocity = Vec3{800.0, 100.0, -200.0};
    original.range = 10000.0;

    original.burst_descriptor.munition_type = EntityType::create_munition(225, 2, 1);
    original.burst_descriptor.warhead = 1000;
    original.burst_descriptor.fuse = 2000;
    original.burst_descriptor.quantity = 1;
    original.burst_descriptor.rate = 0;

    // Serialize
    UInt8 buffer[DIS_MAX_PDU_SIZE];
    SizeT encoded_size = handler->serialize(original, buffer, sizeof(buffer));

    ASSERT_GT(encoded_size, 0);
    EXPECT_EQ(encoded_size, FirePDUHandler::get_fire_pdu_size());

    // Deserialize
    auto decoded = handler->deserialize(buffer, encoded_size);

    ASSERT_TRUE(decoded.has_value());

    // Verify all fields match
    EXPECT_EQ(decoded->header.pdu_type, original.header.pdu_type);
    EXPECT_EQ(decoded->header.protocol_family, original.header.protocol_family);
    EXPECT_EQ(decoded->header.exercise_id, original.header.exercise_id);
    EXPECT_EQ(decoded->header.timestamp, original.header.timestamp);

    EXPECT_EQ(decoded->firing_entity_id, original.firing_entity_id);
    EXPECT_EQ(decoded->target_entity_id, original.target_entity_id);
    EXPECT_EQ(decoded->munition_id, original.munition_id);
    EXPECT_EQ(decoded->event_id, original.event_id);

    EXPECT_EQ(decoded->fire_mission_index, original.fire_mission_index);
    EXPECT_NEAR(decoded->location_in_world.x, original.location_in_world.x, 0.01);
    EXPECT_NEAR(decoded->location_in_world.y, original.location_in_world.y, 0.01);
    EXPECT_NEAR(decoded->location_in_world.z, original.location_in_world.z, 0.01);

    EXPECT_NEAR(decoded->velocity.x, original.velocity.x, 0.01);
    EXPECT_NEAR(decoded->velocity.y, original.velocity.y, 0.01);
    EXPECT_NEAR(decoded->velocity.z, original.velocity.z, 0.01);

    EXPECT_NEAR(decoded->range, original.range, 0.01);

    EXPECT_EQ(decoded->burst_descriptor.munition_type, original.burst_descriptor.munition_type);
    EXPECT_EQ(decoded->burst_descriptor.warhead, original.burst_descriptor.warhead);
    EXPECT_EQ(decoded->burst_descriptor.fuse, original.burst_descriptor.fuse);
    EXPECT_EQ(decoded->burst_descriptor.quantity, original.burst_descriptor.quantity);
}

TEST_F(FirePDUHandlerTest, ValidateValidPDU) {
    FirePDU pdu = handler->create_fire_pdu(
        EntityIdentifier{1, 1, 100},
        EntityIdentifier{1, 1, 200},
        EntityType::create_munition(225, 1, 2),
        Vec3{1000000.0, 2000000.0, 3000000.0},
        Vec3{500.0, 100.0, -50.0},
        5000.0
    );

    EXPECT_TRUE(handler->validate(pdu));
}

TEST_F(FirePDUHandlerTest, ValidateInvalidEntityID) {
    FirePDU pdu = handler->create_fire_pdu(
        EntityIdentifier{0, 0, 0},  // Invalid
        EntityIdentifier{1, 1, 200},
        EntityType::create_munition(225, 1, 2),
        Vec3{1000000.0, 2000000.0, 3000000.0},
        Vec3{500.0, 100.0, -50.0},
        5000.0
    );

    EXPECT_FALSE(handler->validate(pdu));
}

TEST_F(FirePDUHandlerTest, ValidateNegativeRange) {
    FirePDU pdu = handler->create_fire_pdu(
        EntityIdentifier{1, 1, 100},
        EntityIdentifier{1, 1, 200},
        EntityType::create_munition(225, 1, 2),
        Vec3{1000000.0, 2000000.0, 3000000.0},
        Vec3{500.0, 100.0, -50.0},
        -100.0  // Invalid negative range
    );

    EXPECT_FALSE(handler->validate(pdu));
}

TEST_F(FirePDUHandlerTest, SetBurstDescriptor) {
    FirePDU pdu;
    EntityType munition = EntityType::create_munition(225, 2, 3);

    handler->set_burst_descriptor(pdu, munition, 1000, 2000, 5, 600);

    EXPECT_EQ(pdu.burst_descriptor.munition_type, munition);
    EXPECT_EQ(pdu.burst_descriptor.warhead, 1000);
    EXPECT_EQ(pdu.burst_descriptor.fuse, 2000);
    EXPECT_EQ(pdu.burst_descriptor.quantity, 5);
    EXPECT_EQ(pdu.burst_descriptor.rate, 600);
}

TEST_F(FirePDUHandlerTest, SerializeBufferTooSmall) {
    FirePDU pdu = handler->create_fire_pdu(
        EntityIdentifier{1, 1, 100},
        EntityIdentifier{1, 1, 200},
        EntityType::create_munition(225, 1, 2),
        Vec3{1000000.0, 2000000.0, 3000000.0},
        Vec3{500.0, 100.0, -50.0},
        5000.0
    );

    UInt8 buffer[50];  // Too small
    SizeT encoded_size = handler->serialize(pdu, buffer, sizeof(buffer));

    EXPECT_EQ(encoded_size, 0);  // Should fail
}

TEST_F(FirePDUHandlerTest, DeserializeBufferTooSmall) {
    UInt8 buffer[50];  // Too small for Fire PDU
    std::memset(buffer, 0, sizeof(buffer));

    auto decoded = handler->deserialize(buffer, sizeof(buffer));

    EXPECT_FALSE(decoded.has_value());
}

TEST_F(FirePDUHandlerTest, DeserializeWrongPDUType) {
    // Create buffer with Entity State PDU type
    UInt8 buffer[DIS_MAX_PDU_SIZE];
    std::memset(buffer, 0, sizeof(buffer));

    buffer[0] = DIS_VERSION;
    buffer[1] = 1;  // Exercise ID
    buffer[2] = static_cast<UInt8>(PDUType::EntityState);  // Wrong type
    buffer[3] = 1;  // Protocol family

    auto decoded = handler->deserialize(buffer, FirePDUHandler::get_fire_pdu_size());

    EXPECT_FALSE(decoded.has_value());
}

// ============================================================================
// DetonationPDUHandler Tests
// ============================================================================

class DetonationPDUHandlerTest : public ::testing::Test {
protected:
    void SetUp() override {
        handler = std::make_unique<DetonationPDUHandler>();
    }

    std::unique_ptr<DetonationPDUHandler> handler;
};

TEST_F(DetonationPDUHandlerTest, CreateDetonationPDU) {
    EntityIdentifier firing_entity{1, 1, 100};
    EntityIdentifier target_entity{1, 1, 200};
    EntityIdentifier munition_id{1, 1, 300};
    EventIdentifier event_id{1, 1, 5000};
    Vec3 location{1000100.0, 2000100.0, 3000100.0};
    Vec3 velocity{400.0, 80.0, -40.0};
    DetonationResult result = DetonationResult::EntityImpact;

    DetonationPDU pdu = handler->create_detonation_pdu(
        firing_entity,
        target_entity,
        munition_id,
        event_id,
        location,
        velocity,
        result
    );

    EXPECT_EQ(pdu.header.pdu_type, PDUType::Detonation);
    EXPECT_EQ(pdu.header.protocol_family, 1);
    EXPECT_EQ(pdu.firing_entity_id, firing_entity);
    EXPECT_EQ(pdu.target_entity_id, target_entity);
    EXPECT_EQ(pdu.munition_id, munition_id);
    EXPECT_EQ(pdu.event_id, event_id);
    EXPECT_EQ(pdu.location_in_world, location);
    EXPECT_EQ(pdu.velocity, velocity);
    EXPECT_EQ(pdu.detonation_result, result);
}

TEST_F(DetonationPDUHandlerTest, SerializeDeserializeNoArticulation) {
    // Create a Detonation PDU without articulation parameters
    DetonationPDU original;
    original.header.pdu_type = PDUType::Detonation;
    original.header.protocol_family = 1;
    original.header.protocol_version = DIS_VERSION;
    original.header.exercise_id = 1;
    original.header.timestamp = 12345678;

    original.firing_entity_id = EntityIdentifier{1, 2, 100};
    original.target_entity_id = EntityIdentifier{1, 2, 200};
    original.munition_id = EntityIdentifier{1, 2, 300};
    original.event_id = EventIdentifier{1, 2, 5000};

    original.velocity = Vec3{400.0, 80.0, -40.0};
    original.location_in_world = Vec3{1000100.0, 2000100.0, 3000100.0};
    original.location_in_entity = Vec3{1.5, 0.5, -0.3};
    original.detonation_result = DetonationResult::EntityImpact;
    original.num_articulation_params = 0;

    original.burst_descriptor.munition_type = EntityType::create_munition(225, 2, 1);
    original.burst_descriptor.warhead = 1000;
    original.burst_descriptor.fuse = 2000;

    // Serialize
    UInt8 buffer[DIS_MAX_PDU_SIZE];
    SizeT encoded_size = handler->serialize(original, buffer, sizeof(buffer));

    ASSERT_GT(encoded_size, 0);
    EXPECT_EQ(encoded_size, DetonationPDUHandler::get_detonation_pdu_size(0));

    // Deserialize
    auto decoded = handler->deserialize(buffer, encoded_size);

    ASSERT_TRUE(decoded.has_value());

    // Verify all fields match
    EXPECT_EQ(decoded->header.pdu_type, original.header.pdu_type);
    EXPECT_EQ(decoded->firing_entity_id, original.firing_entity_id);
    EXPECT_EQ(decoded->target_entity_id, original.target_entity_id);
    EXPECT_EQ(decoded->munition_id, original.munition_id);
    EXPECT_EQ(decoded->event_id, original.event_id);

    EXPECT_NEAR(decoded->velocity.x, original.velocity.x, 0.01);
    EXPECT_NEAR(decoded->location_in_world.x, original.location_in_world.x, 0.01);
    EXPECT_NEAR(decoded->location_in_entity.x, original.location_in_entity.x, 0.01);

    EXPECT_EQ(decoded->detonation_result, original.detonation_result);
    EXPECT_EQ(decoded->num_articulation_params, 0);
}

TEST_F(DetonationPDUHandlerTest, SerializeDeserializeWithArticulation) {
    // Create a Detonation PDU with articulation parameters
    DetonationPDU original = handler->create_detonation_pdu(
        EntityIdentifier{1, 1, 100},
        EntityIdentifier{1, 1, 200},
        EntityIdentifier{1, 1, 300},
        EventIdentifier{1, 1, 5000},
        Vec3{1000100.0, 2000100.0, 3000100.0},
        Vec3{400.0, 80.0, -40.0},
        DetonationResult::EntityImpact
    );

    // Add articulation parameters
    ArticulationParameter param1;
    param1.parameter_type_designator = 1;
    param1.change_indicator = 2;
    param1.attachment_id = 100;
    param1.parameter_type = 1000;
    param1.parameter_value = 45.0;

    ArticulationParameter param2;
    param2.parameter_type_designator = 2;
    param2.change_indicator = 3;
    param2.attachment_id = 200;
    param2.parameter_type = 2000;
    param2.parameter_value = -30.0;

    ASSERT_TRUE(handler->add_articulation_parameter(original, param1));
    ASSERT_TRUE(handler->add_articulation_parameter(original, param2));

    EXPECT_EQ(original.num_articulation_params, 2);

    // Serialize
    UInt8 buffer[DIS_MAX_PDU_SIZE];
    SizeT encoded_size = handler->serialize(original, buffer, sizeof(buffer));

    ASSERT_GT(encoded_size, 0);
    EXPECT_EQ(encoded_size, DetonationPDUHandler::get_detonation_pdu_size(2));

    // Deserialize
    auto decoded = handler->deserialize(buffer, encoded_size);

    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->num_articulation_params, 2);
    ASSERT_EQ(decoded->articulation_params.size(), 2);

    // Verify articulation parameters
    EXPECT_EQ(decoded->articulation_params[0].parameter_type_designator,
              param1.parameter_type_designator);
    EXPECT_EQ(decoded->articulation_params[0].parameter_type, param1.parameter_type);
    EXPECT_NEAR(decoded->articulation_params[0].parameter_value, param1.parameter_value, 0.01);

    EXPECT_EQ(decoded->articulation_params[1].parameter_type_designator,
              param2.parameter_type_designator);
    EXPECT_NEAR(decoded->articulation_params[1].parameter_value, param2.parameter_value, 0.01);
}

TEST_F(DetonationPDUHandlerTest, ValidateValidPDU) {
    DetonationPDU pdu = handler->create_detonation_pdu(
        EntityIdentifier{1, 1, 100},
        EntityIdentifier{1, 1, 200},
        EntityIdentifier{1, 1, 300},
        EventIdentifier{1, 1, 5000},
        Vec3{1000100.0, 2000100.0, 3000100.0},
        Vec3{400.0, 80.0, -40.0},
        DetonationResult::EntityImpact
    );

    EXPECT_TRUE(handler->validate(pdu));
}

TEST_F(DetonationPDUHandlerTest, SetLocationInEntity) {
    DetonationPDU pdu;
    Vec3 location{2.5, 1.0, -0.5};

    handler->set_location_in_entity(pdu, location);

    EXPECT_EQ(pdu.location_in_entity, location);
}

TEST_F(DetonationPDUHandlerTest, GetDetonationPDUSize) {
    EXPECT_EQ(DetonationPDUHandler::get_detonation_pdu_size(0), 104);
    EXPECT_EQ(DetonationPDUHandler::get_detonation_pdu_size(1), 120);
    EXPECT_EQ(DetonationPDUHandler::get_detonation_pdu_size(5), 184);
}

// ============================================================================
// Utility Function Tests
// ============================================================================

TEST(FirePDUUtilityTest, MatchFireToDetonation) {
    FirePDU fire;
    fire.event_id = EventIdentifier{1, 2, 5000};
    fire.munition_id = EntityIdentifier{1, 2, 300};

    DetonationPDU detonation;
    detonation.event_id = EventIdentifier{1, 2, 5000};
    detonation.munition_id = EntityIdentifier{1, 2, 300};

    EXPECT_TRUE(match_fire_to_detonation(fire, detonation));
}

TEST(FirePDUUtilityTest, MatchFireToDetonationMismatchEvent) {
    FirePDU fire;
    fire.event_id = EventIdentifier{1, 2, 5000};
    fire.munition_id = EntityIdentifier{1, 2, 300};

    DetonationPDU detonation;
    detonation.event_id = EventIdentifier{1, 2, 6000};  // Different event
    detonation.munition_id = EntityIdentifier{1, 2, 300};

    EXPECT_FALSE(match_fire_to_detonation(fire, detonation));
}

TEST(FirePDUUtilityTest, MatchFireToDetonationMismatchMunition) {
    FirePDU fire;
    fire.event_id = EventIdentifier{1, 2, 5000};
    fire.munition_id = EntityIdentifier{1, 2, 300};

    DetonationPDU detonation;
    detonation.event_id = EventIdentifier{1, 2, 5000};
    detonation.munition_id = EntityIdentifier{1, 2, 400};  // Different munition

    EXPECT_FALSE(match_fire_to_detonation(fire, detonation));
}

TEST(FirePDUUtilityTest, DetonationResultToString) {
    EXPECT_STREQ(detonation_result_to_string(DetonationResult::EntityImpact), "EntityImpact");
    EXPECT_STREQ(detonation_result_to_string(DetonationResult::GroundImpact), "GroundImpact");
    EXPECT_STREQ(detonation_result_to_string(DetonationResult::Miss), "Miss");
    EXPECT_STREQ(detonation_result_to_string(DetonationResult::HE_Hit_Large), "HE_Hit_Large");
}

TEST(FirePDUUtilityTest, IsEntityHit) {
    EXPECT_TRUE(is_entity_hit(DetonationResult::EntityImpact));
    EXPECT_TRUE(is_entity_hit(DetonationResult::EntityProximateDetonation));
    EXPECT_TRUE(is_entity_hit(DetonationResult::ArmorPiercingHit));
    EXPECT_TRUE(is_entity_hit(DetonationResult::KillWithFragmentType1));

    EXPECT_FALSE(is_entity_hit(DetonationResult::GroundImpact));
    EXPECT_FALSE(is_entity_hit(DetonationResult::Miss));
    EXPECT_FALSE(is_entity_hit(DetonationResult::AirBurst));
}

TEST(FirePDUUtilityTest, IsGroundImpact) {
    EXPECT_TRUE(is_ground_impact(DetonationResult::GroundImpact));
    EXPECT_TRUE(is_ground_impact(DetonationResult::GroundProximateDetonation));
    EXPECT_TRUE(is_ground_impact(DetonationResult::DirtBlastLarge));

    EXPECT_FALSE(is_ground_impact(DetonationResult::EntityImpact));
    EXPECT_FALSE(is_ground_impact(DetonationResult::WaterImpact));
    EXPECT_FALSE(is_ground_impact(DetonationResult::AirBurst));
}

TEST(FirePDUUtilityTest, CalculateDamageScore) {
    EXPECT_EQ(calculate_damage_score(DetonationResult::EntityImpact), 1.0);
    EXPECT_EQ(calculate_damage_score(DetonationResult::ArmorPiercingHit), 1.0);
    EXPECT_EQ(calculate_damage_score(DetonationResult::HE_Hit_Large), 0.9);
    EXPECT_EQ(calculate_damage_score(DetonationResult::HE_Hit_Medium), 0.7);
    EXPECT_EQ(calculate_damage_score(DetonationResult::HE_Hit_Small), 0.5);
    EXPECT_EQ(calculate_damage_score(DetonationResult::Miss), 0.0);
    EXPECT_EQ(calculate_damage_score(DetonationResult::None), 0.0);
}

TEST(FirePDUUtilityTest, CalculateBallisticEndpoint) {
    Vec3 fire_location{6371000.0, 0.0, 0.0};  // Roughly Earth surface
    Vec3 fire_velocity{500.0, 0.0, 0.0};
    Real time_of_flight = 10.0;  // 10 seconds

    Vec3 endpoint = calculate_ballistic_endpoint(fire_location, fire_velocity, time_of_flight);

    // Should move roughly 5000m in x direction
    EXPECT_GT(endpoint.x, fire_location.x + 4900.0);
    EXPECT_LT(endpoint.x, fire_location.x + 5100.0);

    // Should have some gravitational drop
    Real distance = (endpoint - fire_location).length();
    EXPECT_GT(distance, 4900.0);
}

TEST(FirePDUUtilityTest, CalculateTimeOfFlight) {
    Real range = 1000.0;  // 1 km
    Real muzzle_velocity = 800.0;  // m/s
    Real elevation = 0.0;  // Level fire

    Real time = calculate_time_of_flight(range, muzzle_velocity, elevation);

    EXPECT_NEAR(time, 1.25, 0.1);  // Approximately 1.25 seconds
}

TEST(FirePDUUtilityTest, CalculateTimeOfFlightZeroVelocity) {
    Real time = calculate_time_of_flight(1000.0, 0.0, 0.0);
    EXPECT_EQ(time, 0.0);
}

TEST(FirePDUUtilityTest, CalculateTimeOfFlightZeroRange) {
    Real time = calculate_time_of_flight(0.0, 800.0, 0.0);
    EXPECT_EQ(time, 0.0);
}
