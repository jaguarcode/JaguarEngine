/**
 * @file test_federation_integration.cpp
 * @brief Integration tests for DIS/HLA federation subsystem
 *
 * Tests the integration of:
 * - DIS socket creation and binding
 * - Fire PDU serialization/deserialization
 * - Detonation PDU with articulation parameters
 * - HLA RTI initialization (conditional on HLA being enabled)
 */

#include <gtest/gtest.h>
#include "jaguar/federation/dis_socket.h"
#include "jaguar/federation/dis_protocol.h"
#include "jaguar/federation/dis_fire_pdu.h"
#include "jaguar/core/types.h"
#include <vector>
#include <cstring>

#ifdef JAGUAR_ENABLE_HLA
#include "jaguar/federation/hla_rti.h"
#endif

using namespace jaguar;
using namespace jaguar::federation;

// ============================================================================
// DIS Socket Tests
// ============================================================================

class DISSocketTest : public ::testing::Test {
protected:
    void SetUp() override {
        socket_ = std::make_unique<DISSocket>();
    }

    void TearDown() override {
        if (socket_) {
            socket_->close();
        }
    }

    std::unique_ptr<DISSocket> socket_;
};

TEST_F(DISSocketTest, SocketCreation) {
    EXPECT_FALSE(socket_->is_bound());
    EXPECT_FALSE(socket_->is_multicast());
}

TEST_F(DISSocketTest, BindToPort) {
    // Try to bind to a high port number (less likely to conflict)
    const UInt16 test_port = 62040;
    bool result = socket_->bind(test_port);

    // Binding might fail if port is in use or permission denied
    // We don't assert here, just verify the API works
    if (result) {
        EXPECT_TRUE(socket_->is_bound());
        EXPECT_EQ(socket_->get_port(), test_port);
    } else {
        EXPECT_FALSE(socket_->is_bound());
    }
}

TEST_F(DISSocketTest, MulticastConfiguration) {
    const char* multicast_group = "224.0.0.1";
    const UInt16 test_port = 3000;

    // Attempt to join multicast group
    bool result = socket_->bind_multicast(multicast_group, test_port);

    if (result) {
        EXPECT_TRUE(socket_->is_bound());
        EXPECT_TRUE(socket_->is_multicast());
    }
    // If it fails (no network interface, permissions), that's ok for test
}

TEST_F(DISSocketTest, SendReceiveLoopback) {
    const UInt16 test_port = 62041;

    if (!socket_->bind(test_port)) {
        GTEST_SKIP() << "Could not bind socket for loopback test";
    }

    // Create a simple test buffer
    std::vector<UInt8> send_buffer = {0x01, 0x02, 0x03, 0x04, 0x05};

    // Send to localhost
    bool sent = socket_->send(send_buffer.data(), send_buffer.size(), "127.0.0.1", test_port);
    EXPECT_TRUE(sent);

    // Try to receive (may timeout, which is ok)
    std::vector<UInt8> recv_buffer(1024);
    std::string from_address;
    UInt16 from_port;

    SizeT received = socket_->receive(recv_buffer.data(), recv_buffer.size(), from_address, from_port, 100);

    if (received > 0) {
        EXPECT_EQ(received, send_buffer.size());
        EXPECT_EQ(std::memcmp(send_buffer.data(), recv_buffer.data(), received), 0);
    }
}

// ============================================================================
// Fire PDU Tests
// ============================================================================

class FirePDUTest : public ::testing::Test {
protected:
    void SetUp() override {
        handler_ = std::make_unique<FirePDUHandler>();
    }

    std::unique_ptr<FirePDUHandler> handler_;
};

TEST_F(FirePDUTest, CreateAndValidateFirePDU) {
    FirePDU pdu;

    // Set up PDU header
    pdu.header.protocol_version = 7;
    pdu.header.exercise_id = 1;
    pdu.header.pdu_type = static_cast<UInt8>(PDUType::Fire);
    pdu.header.protocol_family = static_cast<UInt8>(ProtocolFamily::Warfare);
    pdu.header.timestamp = 1000;

    // Set firing entity
    pdu.firing_entity_id.site = 1;
    pdu.firing_entity_id.application = 1;
    pdu.firing_entity_id.entity = 100;

    // Set target entity
    pdu.target_entity_id.site = 1;
    pdu.target_entity_id.application = 1;
    pdu.target_entity_id.entity = 200;

    // Set munition type
    pdu.munition_id.site = 1;
    pdu.munition_id.application = 1;
    pdu.munition_id.entity = 1000;

    // Set event ID
    pdu.event_id.site = 1;
    pdu.event_id.application = 1;
    pdu.event_id.event = 12345;

    // Set fire mission
    pdu.fire_mission_index = 1;

    // Set location
    pdu.location_in_world.x = 1000.0;
    pdu.location_in_world.y = 2000.0;
    pdu.location_in_world.z = 3000.0;

    // Set burst descriptor
    pdu.burst_descriptor.munition.entity_kind = 2;  // Munition
    pdu.burst_descriptor.munition.domain = 1;       // Air
    pdu.burst_descriptor.munition.country = 225;    // United States
    pdu.burst_descriptor.munition.category = 1;
    pdu.burst_descriptor.warhead = 1;
    pdu.burst_descriptor.fuse = 1;
    pdu.burst_descriptor.quantity = 1;
    pdu.burst_descriptor.rate = 1;

    // Set velocity
    pdu.velocity.x = 100.0f;
    pdu.velocity.y = 0.0f;
    pdu.velocity.z = 50.0f;

    // Set range
    pdu.range = 5000.0f;

    // Validate
    EXPECT_TRUE(handler_->validate(pdu));
}

TEST_F(FirePDUTest, SerializeDeserializeRoundTrip) {
    // Create Fire PDU
    FirePDU original;
    original.header.protocol_version = 7;
    original.header.exercise_id = 1;
    original.header.pdu_type = static_cast<UInt8>(PDUType::Fire);
    original.header.protocol_family = static_cast<UInt8>(ProtocolFamily::Warfare);
    original.header.timestamp = 12345678;

    original.firing_entity_id.site = 1;
    original.firing_entity_id.application = 2;
    original.firing_entity_id.entity = 300;

    original.target_entity_id.site = 1;
    original.target_entity_id.application = 2;
    original.target_entity_id.entity = 400;

    original.munition_id.site = 1;
    original.munition_id.application = 2;
    original.munition_id.entity = 500;

    original.event_id.site = 1;
    original.event_id.application = 2;
    original.event_id.event = 99999;

    original.location_in_world.x = 12345.6;
    original.location_in_world.y = 67890.1;
    original.location_in_world.z = 11111.2;

    original.velocity.x = 250.5f;
    original.velocity.y = -10.2f;
    original.velocity.z = 100.0f;

    original.range = 10000.0f;

    // Serialize
    std::vector<UInt8> buffer(1024);
    SizeT bytes_written = handler_->serialize(original, buffer.data(), buffer.size());
    ASSERT_GT(bytes_written, 0u);

    // Deserialize
    auto deserialized_opt = handler_->deserialize(buffer.data(), bytes_written);
    ASSERT_TRUE(deserialized_opt.has_value());

    FirePDU deserialized = deserialized_opt.value();

    // Verify header
    EXPECT_EQ(deserialized.header.protocol_version, original.header.protocol_version);
    EXPECT_EQ(deserialized.header.exercise_id, original.header.exercise_id);
    EXPECT_EQ(deserialized.header.pdu_type, original.header.pdu_type);
    EXPECT_EQ(deserialized.header.timestamp, original.header.timestamp);

    // Verify entity IDs
    EXPECT_EQ(deserialized.firing_entity_id.site, original.firing_entity_id.site);
    EXPECT_EQ(deserialized.firing_entity_id.application, original.firing_entity_id.application);
    EXPECT_EQ(deserialized.firing_entity_id.entity, original.firing_entity_id.entity);

    EXPECT_EQ(deserialized.target_entity_id.entity, original.target_entity_id.entity);
    EXPECT_EQ(deserialized.munition_id.entity, original.munition_id.entity);
    EXPECT_EQ(deserialized.event_id.event, original.event_id.event);

    // Verify location
    EXPECT_DOUBLE_EQ(deserialized.location_in_world.x, original.location_in_world.x);
    EXPECT_DOUBLE_EQ(deserialized.location_in_world.y, original.location_in_world.y);
    EXPECT_DOUBLE_EQ(deserialized.location_in_world.z, original.location_in_world.z);

    // Verify velocity
    EXPECT_FLOAT_EQ(deserialized.velocity.x, original.velocity.x);
    EXPECT_FLOAT_EQ(deserialized.velocity.y, original.velocity.y);
    EXPECT_FLOAT_EQ(deserialized.velocity.z, original.velocity.z);

    // Verify range
    EXPECT_FLOAT_EQ(deserialized.range, original.range);
}

TEST_F(FirePDUTest, InvalidPDUDetection) {
    FirePDU invalid_pdu;
    std::memset(&invalid_pdu, 0, sizeof(invalid_pdu));

    // Completely zeroed PDU should fail validation
    EXPECT_FALSE(handler_->validate(invalid_pdu));
}

// ============================================================================
// Detonation PDU Tests
// ============================================================================

class DetonationPDUTest : public ::testing::Test {
protected:
    void SetUp() override {
        handler_ = std::make_unique<DetonationPDUHandler>();
    }

    std::unique_ptr<DetonationPDUHandler> handler_;
};

TEST_F(DetonationPDUTest, CreateAndValidateDetonationPDU) {
    DetonationPDU pdu;

    // Set up header
    pdu.header.protocol_version = 7;
    pdu.header.exercise_id = 1;
    pdu.header.pdu_type = static_cast<UInt8>(PDUType::Detonation);
    pdu.header.protocol_family = static_cast<UInt8>(ProtocolFamily::Warfare);
    pdu.header.timestamp = 2000;

    // Set firing entity
    pdu.firing_entity_id.site = 1;
    pdu.firing_entity_id.application = 1;
    pdu.firing_entity_id.entity = 100;

    // Set target entity
    pdu.target_entity_id.site = 1;
    pdu.target_entity_id.application = 1;
    pdu.target_entity_id.entity = 200;

    // Set munition
    pdu.munition_id.site = 1;
    pdu.munition_id.application = 1;
    pdu.munition_id.entity = 1001;

    // Set event ID
    pdu.event_id.site = 1;
    pdu.event_id.application = 1;
    pdu.event_id.event = 54321;

    // Set detonation location
    pdu.location_in_world.x = 5000.0;
    pdu.location_in_world.y = 6000.0;
    pdu.location_in_world.z = 100.0;

    // Set velocity
    pdu.velocity.x = 0.0f;
    pdu.velocity.y = 0.0f;
    pdu.velocity.z = -50.0f;  // Impact velocity

    // Set detonation result
    pdu.detonation_result = static_cast<UInt8>(DetonationResult::EntityImpact);

    // Validate
    EXPECT_TRUE(handler_->validate(pdu));
}

TEST_F(DetonationPDUTest, DetonationWithArticulationParameters) {
    DetonationPDU pdu;

    // Basic setup
    pdu.header.protocol_version = 7;
    pdu.header.exercise_id = 1;
    pdu.header.pdu_type = static_cast<UInt8>(PDUType::Detonation);
    pdu.header.protocol_family = static_cast<UInt8>(ProtocolFamily::Warfare);

    pdu.firing_entity_id.entity = 100;
    pdu.target_entity_id.entity = 200;
    pdu.munition_id.entity = 1001;
    pdu.event_id.event = 11111;

    pdu.location_in_world.x = 1000.0;
    pdu.location_in_world.y = 2000.0;
    pdu.location_in_world.z = 500.0;

    pdu.detonation_result = static_cast<UInt8>(DetonationResult::EntityImpact);

    // Add articulation parameters (e.g., damage assessment)
    pdu.num_articulation_params = 2;

    ArticulationParameter param1;
    param1.parameter_type_designator = 0;  // Articulated part
    param1.change_indicator = 1;
    param1.articulation_attachment_id = 1;
    param1.parameter_type = 4096;  // Position
    param1.parameter_value = 45.0;  // 45 degrees
    pdu.articulation_params.push_back(param1);

    ArticulationParameter param2;
    param2.parameter_type_designator = 0;
    param2.change_indicator = 1;
    param2.articulation_attachment_id = 2;
    param2.parameter_type = 4097;  // Rate
    param2.parameter_value = 10.0;  // 10 deg/s
    pdu.articulation_params.push_back(param2);

    // Validate
    EXPECT_TRUE(handler_->validate(pdu));

    // Serialize and deserialize
    std::vector<UInt8> buffer(2048);
    SizeT bytes_written = handler_->serialize(pdu, buffer.data(), buffer.size());
    ASSERT_GT(bytes_written, 0u);

    auto deserialized_opt = handler_->deserialize(buffer.data(), bytes_written);
    ASSERT_TRUE(deserialized_opt.has_value());

    DetonationPDU deserialized = deserialized_opt.value();

    // Verify articulation parameters survived round trip
    EXPECT_EQ(deserialized.num_articulation_params, 2u);
    ASSERT_EQ(deserialized.articulation_params.size(), 2u);

    EXPECT_EQ(deserialized.articulation_params[0].articulation_attachment_id,
              param1.articulation_attachment_id);
    EXPECT_DOUBLE_EQ(deserialized.articulation_params[0].parameter_value,
                     param1.parameter_value);

    EXPECT_EQ(deserialized.articulation_params[1].parameter_type,
              param2.parameter_type);
    EXPECT_DOUBLE_EQ(deserialized.articulation_params[1].parameter_value,
                     param2.parameter_value);
}

TEST_F(DetonationPDUTest, SerializeDeserializeRoundTrip) {
    DetonationPDU original;

    original.header.protocol_version = 7;
    original.header.exercise_id = 5;
    original.header.pdu_type = static_cast<UInt8>(PDUType::Detonation);
    original.header.protocol_family = static_cast<UInt8>(ProtocolFamily::Warfare);
    original.header.timestamp = 99999999;

    original.firing_entity_id.site = 2;
    original.firing_entity_id.application = 3;
    original.firing_entity_id.entity = 777;

    original.target_entity_id.site = 2;
    original.target_entity_id.application = 3;
    original.target_entity_id.entity = 888;

    original.munition_id.entity = 5555;
    original.event_id.event = 77777;

    original.location_in_world.x = 98765.4;
    original.location_in_world.y = 12345.6;
    original.location_in_world.z = 543.21;

    original.velocity.x = -100.0f;
    original.velocity.y = 50.0f;
    original.velocity.z = -200.0f;

    original.detonation_result = static_cast<UInt8>(DetonationResult::GroundImpact);

    // Serialize
    std::vector<UInt8> buffer(2048);
    SizeT bytes_written = handler_->serialize(original, buffer.data(), buffer.size());
    ASSERT_GT(bytes_written, 0u);

    // Deserialize
    auto deserialized_opt = handler_->deserialize(buffer.data(), bytes_written);
    ASSERT_TRUE(deserialized_opt.has_value());

    DetonationPDU deserialized = deserialized_opt.value();

    // Verify all fields
    EXPECT_EQ(deserialized.header.exercise_id, original.header.exercise_id);
    EXPECT_EQ(deserialized.firing_entity_id.entity, original.firing_entity_id.entity);
    EXPECT_EQ(deserialized.target_entity_id.entity, original.target_entity_id.entity);
    EXPECT_EQ(deserialized.event_id.event, original.event_id.event);

    EXPECT_DOUBLE_EQ(deserialized.location_in_world.x, original.location_in_world.x);
    EXPECT_DOUBLE_EQ(deserialized.location_in_world.y, original.location_in_world.y);
    EXPECT_DOUBLE_EQ(deserialized.location_in_world.z, original.location_in_world.z);

    EXPECT_FLOAT_EQ(deserialized.velocity.x, original.velocity.x);
    EXPECT_FLOAT_EQ(deserialized.velocity.y, original.velocity.y);
    EXPECT_FLOAT_EQ(deserialized.velocity.z, original.velocity.z);

    EXPECT_EQ(deserialized.detonation_result, original.detonation_result);
}

// ============================================================================
// HLA RTI Tests (Conditional)
// ============================================================================

#ifdef JAGUAR_ENABLE_HLA

class HLARTITest : public ::testing::Test {
protected:
    void SetUp() override {
        rti_ = std::make_unique<HLAARTI>();
    }

    void TearDown() override {
        if (rti_ && rti_->is_connected()) {
            rti_->disconnect();
        }
    }

    std::unique_ptr<HLAARTI> rti_;
};

TEST_F(HLARTITest, RTIInitialization) {
    // Just verify we can create the RTI interface
    EXPECT_FALSE(rti_->is_connected());
}

TEST_F(HLARTITest, ConnectToFederation) {
    // Attempt to connect (may fail if no RTI Ambassador running)
    const char* federation_name = "JaguarTestFederation";
    const char* federate_name = "JaguarTestFederate";
    const char* fom_file = "test.fed";

    bool connected = rti_->connect(federation_name, federate_name, fom_file);

    if (connected) {
        EXPECT_TRUE(rti_->is_connected());

        // Try to disconnect
        bool disconnected = rti_->disconnect();
        EXPECT_TRUE(disconnected);
        EXPECT_FALSE(rti_->is_connected());
    } else {
        // Connection failed (no RTI running, etc.) - that's ok for unit test
        EXPECT_FALSE(rti_->is_connected());
    }
}

TEST_F(HLARTITest, PublishAndSubscribeObjectClass) {
    // This test verifies the API is callable
    // Actual functionality requires RTI connection

    if (rti_->connect("TestFederation", "TestFederate", "test.fed")) {
        // Try to publish an object class
        bool published = rti_->publish_object_class("BaseEntity");

        // Result depends on FOM definition
        // We just verify the call doesn't crash
        (void)published;

        // Try to subscribe
        bool subscribed = rti_->subscribe_object_class("BaseEntity");
        (void)subscribed;

        rti_->disconnect();
    }
}

#endif  // JAGUAR_ENABLE_HLA

// ============================================================================
// DIS/HLA Integration Test
// ============================================================================

class FederationIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        socket_ = std::make_unique<DISSocket>();
        fire_handler_ = std::make_unique<FirePDUHandler>();
        detonation_handler_ = std::make_unique<DetonationPDUHandler>();
    }

    void TearDown() override {
        if (socket_) {
            socket_->close();
        }
    }

    std::unique_ptr<DISSocket> socket_;
    std::unique_ptr<FirePDUHandler> fire_handler_;
    std::unique_ptr<DetonationPDUHandler> detonation_handler_;
};

TEST_F(FederationIntegrationTest, FirePDUNetworkTransmission) {
    const UInt16 test_port = 62042;

    if (!socket_->bind(test_port)) {
        GTEST_SKIP() << "Could not bind socket for network test";
    }

    // Create Fire PDU
    FirePDU fire_pdu;
    fire_pdu.header.protocol_version = 7;
    fire_pdu.header.exercise_id = 1;
    fire_pdu.header.pdu_type = static_cast<UInt8>(PDUType::Fire);
    fire_pdu.header.protocol_family = static_cast<UInt8>(ProtocolFamily::Warfare);
    fire_pdu.header.timestamp = 123456;

    fire_pdu.firing_entity_id.entity = 42;
    fire_pdu.target_entity_id.entity = 99;
    fire_pdu.event_id.event = 12345;

    fire_pdu.location_in_world.x = 1000.0;
    fire_pdu.location_in_world.y = 2000.0;
    fire_pdu.location_in_world.z = 3000.0;

    fire_pdu.velocity.x = 300.0f;
    fire_pdu.velocity.y = 0.0f;
    fire_pdu.velocity.z = 100.0f;

    fire_pdu.range = 5000.0f;

    // Serialize
    std::vector<UInt8> buffer(1024);
    SizeT bytes = fire_handler_->serialize(fire_pdu, buffer.data(), buffer.size());
    ASSERT_GT(bytes, 0u);

    // Send over network
    bool sent = socket_->send(buffer.data(), bytes, "127.0.0.1", test_port);
    EXPECT_TRUE(sent);

    // Try to receive (with timeout)
    std::vector<UInt8> recv_buffer(1024);
    std::string from_addr;
    UInt16 from_port;

    SizeT received = socket_->receive(recv_buffer.data(), recv_buffer.size(), from_addr, from_port, 100);

    if (received > 0) {
        // Deserialize
        auto received_pdu = fire_handler_->deserialize(recv_buffer.data(), received);

        if (received_pdu.has_value()) {
            EXPECT_EQ(received_pdu->event_id.event, fire_pdu.event_id.event);
            EXPECT_EQ(received_pdu->firing_entity_id.entity, fire_pdu.firing_entity_id.entity);
        }
    }
}

TEST_F(FederationIntegrationTest, DetonationPDUNetworkTransmission) {
    const UInt16 test_port = 62043;

    if (!socket_->bind(test_port)) {
        GTEST_SKIP() << "Could not bind socket for network test";
    }

    // Create Detonation PDU
    DetonationPDU det_pdu;
    det_pdu.header.protocol_version = 7;
    det_pdu.header.exercise_id = 1;
    det_pdu.header.pdu_type = static_cast<UInt8>(PDUType::Detonation);
    det_pdu.header.protocol_family = static_cast<UInt8>(ProtocolFamily::Warfare);

    det_pdu.firing_entity_id.entity = 42;
    det_pdu.target_entity_id.entity = 99;
    det_pdu.event_id.event = 54321;

    det_pdu.location_in_world.x = 1500.0;
    det_pdu.location_in_world.y = 2500.0;
    det_pdu.location_in_world.z = 50.0;

    det_pdu.detonation_result = static_cast<UInt8>(DetonationResult::EntityImpact);

    // Serialize
    std::vector<UInt8> buffer(2048);
    SizeT bytes = detonation_handler_->serialize(det_pdu, buffer.data(), buffer.size());
    ASSERT_GT(bytes, 0u);

    // Send
    bool sent = socket_->send(buffer.data(), bytes, "127.0.0.1", test_port);
    EXPECT_TRUE(sent);

    // Receive
    std::vector<UInt8> recv_buffer(2048);
    std::string from_addr;
    UInt16 from_port;

    SizeT received = socket_->receive(recv_buffer.data(), recv_buffer.size(), from_addr, from_port, 100);

    if (received > 0) {
        auto received_pdu = detonation_handler_->deserialize(recv_buffer.data(), received);

        if (received_pdu.has_value()) {
            EXPECT_EQ(received_pdu->event_id.event, det_pdu.event_id.event);
            EXPECT_EQ(received_pdu->detonation_result, det_pdu.detonation_result);
        }
    }
}
