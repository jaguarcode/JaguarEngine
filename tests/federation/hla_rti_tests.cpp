/**
 * @file hla_rti_tests.cpp
 * @brief Unit tests for HLA RTI (High Level Architecture Runtime Infrastructure)
 */

#include <gtest/gtest.h>
#include "jaguar/federation/hla_rti.h"
#include <thread>
#include <chrono>

using namespace jaguar;
using namespace jaguar::federation::hla;

// ============================================================================
// Handle Type Tests
// ============================================================================

TEST(ObjectInstanceHandleTest, DefaultConstruction) {
    ObjectInstanceHandle handle;
    EXPECT_FALSE(handle.is_valid());
    EXPECT_EQ(handle.value, 0);
}

TEST(ObjectInstanceHandleTest, ExplicitConstruction) {
    ObjectInstanceHandle handle(42);
    EXPECT_TRUE(handle.is_valid());
    EXPECT_EQ(handle.value, 42);
}

TEST(ObjectInstanceHandleTest, Equality) {
    ObjectInstanceHandle h1(10);
    ObjectInstanceHandle h2(10);
    ObjectInstanceHandle h3(20);

    EXPECT_EQ(h1, h2);
    EXPECT_NE(h1, h3);
}

TEST(ObjectInstanceHandleTest, Invalid) {
    auto invalid = ObjectInstanceHandle::invalid();
    EXPECT_FALSE(invalid.is_valid());
    EXPECT_EQ(invalid.value, 0);
}

TEST(ObjectClassHandleTest, DefaultConstruction) {
    ObjectClassHandle handle;
    EXPECT_FALSE(handle.is_valid());
    EXPECT_EQ(handle.value, 0);
}

TEST(ObjectClassHandleTest, ExplicitConstruction) {
    ObjectClassHandle handle(100);
    EXPECT_TRUE(handle.is_valid());
    EXPECT_EQ(handle.value, 100);
}

TEST(AttributeHandleTest, DefaultConstruction) {
    AttributeHandle handle;
    EXPECT_FALSE(handle.is_valid());
}

TEST(AttributeHandleTest, ExplicitConstruction) {
    AttributeHandle handle(5);
    EXPECT_TRUE(handle.is_valid());
    EXPECT_EQ(handle.value, 5);
}

TEST(InteractionClassHandleTest, DefaultConstruction) {
    InteractionClassHandle handle;
    EXPECT_FALSE(handle.is_valid());
}

TEST(InteractionClassHandleTest, ExplicitConstruction) {
    InteractionClassHandle handle(50);
    EXPECT_TRUE(handle.is_valid());
    EXPECT_EQ(handle.value, 50);
}

TEST(ParameterHandleTest, DefaultConstruction) {
    ParameterHandle handle;
    EXPECT_FALSE(handle.is_valid());
}

TEST(ParameterHandleTest, ExplicitConstruction) {
    ParameterHandle handle(7);
    EXPECT_TRUE(handle.is_valid());
    EXPECT_EQ(handle.value, 7);
}

TEST(FederateHandleTest, DefaultConstruction) {
    FederateHandle handle;
    EXPECT_FALSE(handle.is_valid());
}

TEST(FederateHandleTest, ExplicitConstruction) {
    FederateHandle handle(1000);
    EXPECT_TRUE(handle.is_valid());
    EXPECT_EQ(handle.value, 1000);
}

TEST(DimensionHandleTest, DefaultConstruction) {
    DimensionHandle handle;
    EXPECT_FALSE(handle.is_valid());
}

TEST(RegionHandleTest, DefaultConstruction) {
    RegionHandle handle;
    EXPECT_FALSE(handle.is_valid());
}

// ============================================================================
// Handle Hash Tests
// ============================================================================

TEST(HandleHashTest, ObjectInstanceHandleHash) {
    ObjectInstanceHandleHash hasher;
    ObjectInstanceHandle h1(100);
    ObjectInstanceHandle h2(100);
    ObjectInstanceHandle h3(200);

    EXPECT_EQ(hasher(h1), hasher(h2));
    EXPECT_NE(hasher(h1), hasher(h3));
}

TEST(HandleHashTest, ObjectClassHandleHash) {
    ObjectClassHandleHash hasher;
    ObjectClassHandle h1(10);
    ObjectClassHandle h2(10);

    EXPECT_EQ(hasher(h1), hasher(h2));
}

TEST(HandleHashTest, AttributeHandleHash) {
    AttributeHandleHash hasher;
    AttributeHandle h1(5);
    AttributeHandle h2(5);

    EXPECT_EQ(hasher(h1), hasher(h2));
}

TEST(HandleHashTest, FederateHandleHash) {
    FederateHandleHash hasher;
    FederateHandle h1(999);
    FederateHandle h2(999);

    EXPECT_EQ(hasher(h1), hasher(h2));
}

// ============================================================================
// Logical Time Tests
// ============================================================================

TEST(LogicalTimeTest, DefaultConstruction) {
    LogicalTime time;
    EXPECT_DOUBLE_EQ(time.value(), 0.0);
}

TEST(LogicalTimeTest, ExplicitConstruction) {
    LogicalTime time(100.5);
    EXPECT_DOUBLE_EQ(time.value(), 100.5);
}

TEST(LogicalTimeTest, Comparison) {
    LogicalTime t1(10.0);
    LogicalTime t2(20.0);
    LogicalTime t3(10.0);

    EXPECT_LT(t1, t2);
    EXPECT_LE(t1, t2);
    EXPECT_LE(t1, t3);
    EXPECT_GT(t2, t1);
    EXPECT_GE(t2, t1);
    EXPECT_EQ(t1, t3);
    EXPECT_NE(t1, t2);
}

TEST(LogicalTimeTest, Arithmetic) {
    LogicalTime t1(10.0);
    LogicalTime t2(5.0);

    LogicalTime sum = t1 + t2;
    LogicalTime diff = t1 - t2;

    EXPECT_DOUBLE_EQ(sum.value(), 15.0);
    EXPECT_DOUBLE_EQ(diff.value(), 5.0);
}

TEST(LogicalTimeIntervalTest, DefaultConstruction) {
    LogicalTimeInterval interval;
    EXPECT_DOUBLE_EQ(interval.value(), 0.0);
}

TEST(LogicalTimeIntervalTest, ExplicitConstruction) {
    LogicalTimeInterval interval(0.5);
    EXPECT_DOUBLE_EQ(interval.value(), 0.5);
}

TEST(LogicalTimeIntervalTest, Equality) {
    LogicalTimeInterval i1(0.1);
    LogicalTimeInterval i2(0.1);
    LogicalTimeInterval i3(0.2);

    EXPECT_EQ(i1, i2);
    EXPECT_NE(i1, i3);
}

// ============================================================================
// Time Regulation Status Tests
// ============================================================================

TEST(TimeRegulationStatusTest, EnumValues) {
    EXPECT_NE(TimeRegulationStatus::None, TimeRegulationStatus::Regulating);
    EXPECT_NE(TimeRegulationStatus::Constrained, TimeRegulationStatus::Both);
}

TEST(TimeRegulationStatusTest, ToString) {
    EXPECT_STREQ(time_regulation_status_to_string(TimeRegulationStatus::None), "None");
    EXPECT_STREQ(time_regulation_status_to_string(TimeRegulationStatus::Regulating), "Regulating");
    EXPECT_STREQ(time_regulation_status_to_string(TimeRegulationStatus::Constrained), "Constrained");
    EXPECT_STREQ(time_regulation_status_to_string(TimeRegulationStatus::Both), "Both");
}

// ============================================================================
// Order Type Tests
// ============================================================================

TEST(OrderTypeTest, EnumValues) {
    EXPECT_EQ(static_cast<UInt8>(OrderType::Receive), 0);
    EXPECT_EQ(static_cast<UInt8>(OrderType::Timestamp), 1);
}

TEST(TransportationTypeTest, EnumValues) {
    EXPECT_EQ(static_cast<UInt8>(TransportationType::Reliable), 0);
    EXPECT_EQ(static_cast<UInt8>(TransportationType::BestEffort), 1);
}

// ============================================================================
// Resign Action Tests
// ============================================================================

TEST(ResignActionTest, EnumValues) {
    EXPECT_NE(ResignAction::UnconditionallyDivestAttributes, ResignAction::DeleteObjects);
    EXPECT_NE(ResignAction::NoAction, ResignAction::CancelThenDeleteThenDivest);
}

// ============================================================================
// Ownership State Tests
// ============================================================================

TEST(OwnershipStateTest, EnumValues) {
    EXPECT_EQ(static_cast<UInt8>(OwnershipState::Unowned), 0);
    EXPECT_NE(OwnershipState::OwnedByUs, OwnershipState::OwnedByOther);
}

// ============================================================================
// Synchronization Point Status Tests
// ============================================================================

TEST(SyncPointStatusTest, EnumValues) {
    EXPECT_NE(SyncPointStatus::Announced, SyncPointStatus::Synchronized);
    EXPECT_NE(SyncPointStatus::Registered, SyncPointStatus::RegisterFailed);
}

// ============================================================================
// Save/Restore Status Tests
// ============================================================================

TEST(SaveStatusTest, EnumValues) {
    EXPECT_EQ(static_cast<UInt8>(SaveStatus::NotInitiated), 0);
    EXPECT_NE(SaveStatus::Completed, SaveStatus::Failed);
}

TEST(RestoreStatusTest, EnumValues) {
    EXPECT_EQ(static_cast<UInt8>(RestoreStatus::NotInitiated), 0);
    EXPECT_NE(RestoreStatus::Completed, RestoreStatus::Failed);
}

// ============================================================================
// Federation Execution Info Tests
// ============================================================================

TEST(FederationExecutionInfoTest, DefaultConstruction) {
    FederationExecutionInfo info;
    EXPECT_TRUE(info.federation_name.empty());
    EXPECT_EQ(info.federate_count, 0);
    EXPECT_FALSE(info.time_management_enabled);
}

TEST(FederationExecutionInfoTest, SetValues) {
    FederationExecutionInfo info;
    info.federation_name = "TestFederation";
    info.fom_module_designator = "RPR_FOM_v2";
    info.current_time = LogicalTime(100.0);
    info.federate_count = 5;
    info.time_management_enabled = true;

    EXPECT_EQ(info.federation_name, "TestFederation");
    EXPECT_EQ(info.fom_module_designator, "RPR_FOM_v2");
    EXPECT_DOUBLE_EQ(info.current_time.value(), 100.0);
    EXPECT_EQ(info.federate_count, 5);
    EXPECT_TRUE(info.time_management_enabled);
}

// ============================================================================
// Federate Info Tests
// ============================================================================

TEST(FederateInfoTest, DefaultConstruction) {
    FederateInfo info;
    EXPECT_FALSE(info.handle.is_valid());
    EXPECT_TRUE(info.federate_name.empty());
    EXPECT_EQ(info.time_status, TimeRegulationStatus::None);
    EXPECT_TRUE(info.is_alive);
}

TEST(FederateInfoTest, SetValues) {
    FederateInfo info;
    info.handle = FederateHandle(123);
    info.federate_name = "TestFederate";
    info.federate_type = "Simulator";
    info.time_status = TimeRegulationStatus::Both;
    info.current_time = LogicalTime(50.0);
    info.lookahead = LogicalTimeInterval(0.1);

    EXPECT_TRUE(info.handle.is_valid());
    EXPECT_EQ(info.federate_name, "TestFederate");
    EXPECT_EQ(info.time_status, TimeRegulationStatus::Both);
}

// ============================================================================
// Attribute Value Set Tests
// ============================================================================

TEST(AttributeValueSetTest, DefaultConstruction) {
    AttributeValueSet avs;
    EXPECT_TRUE(avs.empty());
    EXPECT_EQ(avs.size(), 0);
}

TEST(AttributeValueSetTest, SetAndGetAttribute) {
    AttributeValueSet avs;
    AttributeHandle handle(1);
    std::vector<UInt8> value = {1, 2, 3, 4};

    avs.set_attribute(handle, value);

    EXPECT_FALSE(avs.empty());
    EXPECT_EQ(avs.size(), 1);
    EXPECT_TRUE(avs.has_attribute(handle));

    auto retrieved = avs.get_attribute(handle);
    ASSERT_TRUE(retrieved.has_value());
    EXPECT_EQ(retrieved.value(), value);
}

TEST(AttributeValueSetTest, GetNonExistent) {
    AttributeValueSet avs;
    AttributeHandle handle(999);

    EXPECT_FALSE(avs.has_attribute(handle));
    auto retrieved = avs.get_attribute(handle);
    EXPECT_FALSE(retrieved.has_value());
}

TEST(AttributeValueSetTest, MultipleAttributes) {
    AttributeValueSet avs;
    AttributeHandle h1(1), h2(2), h3(3);

    avs.set_attribute(h1, {0x01});
    avs.set_attribute(h2, {0x02, 0x03});
    avs.set_attribute(h3, {0x04, 0x05, 0x06});

    EXPECT_EQ(avs.size(), 3);
    EXPECT_TRUE(avs.has_attribute(h1));
    EXPECT_TRUE(avs.has_attribute(h2));
    EXPECT_TRUE(avs.has_attribute(h3));
}

// ============================================================================
// Parameter Value Set Tests
// ============================================================================

TEST(ParameterValueSetTest, DefaultConstruction) {
    ParameterValueSet pvs;
    EXPECT_TRUE(pvs.empty());
    EXPECT_EQ(pvs.size(), 0);
}

TEST(ParameterValueSetTest, SetAndGetParameter) {
    ParameterValueSet pvs;
    ParameterHandle handle(1);
    std::vector<UInt8> value = {5, 6, 7, 8};

    pvs.set_parameter(handle, value);

    EXPECT_FALSE(pvs.empty());
    EXPECT_EQ(pvs.size(), 1);
    EXPECT_TRUE(pvs.has_parameter(handle));

    auto retrieved = pvs.get_parameter(handle);
    ASSERT_TRUE(retrieved.has_value());
    EXPECT_EQ(retrieved.value(), value);
}

TEST(ParameterValueSetTest, GetNonExistent) {
    ParameterValueSet pvs;
    ParameterHandle handle(999);

    EXPECT_FALSE(pvs.has_parameter(handle));
    auto retrieved = pvs.get_parameter(handle);
    EXPECT_FALSE(retrieved.has_value());
}

// ============================================================================
// Object Instance Tests
// ============================================================================

TEST(ObjectInstanceTest, DefaultConstruction) {
    ObjectInstance instance;
    EXPECT_FALSE(instance.handle.is_valid());
    EXPECT_FALSE(instance.class_handle.is_valid());
    EXPECT_TRUE(instance.instance_name.empty());
    EXPECT_FALSE(instance.is_local);
}

TEST(ObjectInstanceTest, SetValues) {
    ObjectInstance instance;
    instance.handle = ObjectInstanceHandle(100);
    instance.class_handle = ObjectClassHandle(10);
    instance.instance_name = "Tank_01";
    instance.owner = FederateHandle(5);
    instance.is_local = true;

    EXPECT_TRUE(instance.handle.is_valid());
    EXPECT_EQ(instance.instance_name, "Tank_01");
    EXPECT_TRUE(instance.is_local);
}

// ============================================================================
// Object Class Definition Tests
// ============================================================================

TEST(ObjectClassDefinitionTest, DefaultConstruction) {
    ObjectClassDefinition def;
    EXPECT_FALSE(def.handle.is_valid());
    EXPECT_TRUE(def.name.empty());
    EXPECT_TRUE(def.attributes.empty());
    EXPECT_FALSE(def.parent_class.has_value());
}

TEST(ObjectClassDefinitionTest, SetValues) {
    ObjectClassDefinition def;
    def.handle = ObjectClassHandle(1);
    def.name = "HLAobjectRoot.BaseEntity";
    def.parent_class = ObjectClassHandle(0);

    AttributeDefinition attr;
    attr.handle = AttributeHandle(1);
    attr.name = "EntityIdentifier";
    attr.datatype = "EntityIdentifierStruct";
    def.attributes.push_back(attr);

    EXPECT_TRUE(def.handle.is_valid());
    EXPECT_EQ(def.name, "HLAobjectRoot.BaseEntity");
    EXPECT_EQ(def.attributes.size(), 1);
    EXPECT_TRUE(def.parent_class.has_value());
}

// ============================================================================
// Interaction Class Definition Tests
// ============================================================================

TEST(InteractionClassDefinitionTest, DefaultConstruction) {
    InteractionClassDefinition def;
    EXPECT_FALSE(def.handle.is_valid());
    EXPECT_TRUE(def.name.empty());
    EXPECT_TRUE(def.parameters.empty());
    EXPECT_EQ(def.order, OrderType::Receive);
    EXPECT_EQ(def.transport, TransportationType::Reliable);
}

TEST(InteractionClassDefinitionTest, SetValues) {
    InteractionClassDefinition def;
    def.handle = InteractionClassHandle(10);
    def.name = "HLAinteractionRoot.WeaponFire";
    def.order = OrderType::Timestamp;
    def.transport = TransportationType::BestEffort;

    ParameterDefinition param;
    param.handle = ParameterHandle(1);
    param.name = "FiringObjectIdentifier";
    def.parameters.push_back(param);

    EXPECT_TRUE(def.handle.is_valid());
    EXPECT_EQ(def.order, OrderType::Timestamp);
    EXPECT_EQ(def.parameters.size(), 1);
}

// ============================================================================
// RTI Connection Settings Tests
// ============================================================================

TEST(RTIConnectionSettingsTest, DefaultValues) {
    RTIConnectionSettings settings;
    EXPECT_EQ(settings.rti_host, "localhost");
    EXPECT_EQ(settings.rti_port, 8989);
    EXPECT_EQ(settings.rti_type, "HLA_EVOLVER");
    EXPECT_FALSE(settings.use_tls);
}

TEST(RTIConnectionSettingsTest, CustomValues) {
    RTIConnectionSettings settings;
    settings.rti_host = "192.168.1.100";
    settings.rti_port = 9000;
    settings.rti_type = "Pitch";
    settings.use_tls = true;
    settings.tls_cert_path = "/path/to/cert.pem";

    EXPECT_EQ(settings.rti_host, "192.168.1.100");
    EXPECT_EQ(settings.rti_port, 9000);
    EXPECT_TRUE(settings.use_tls);
}

// ============================================================================
// Time Management Settings Tests
// ============================================================================

TEST(TimeManagementSettingsTest, DefaultValues) {
    TimeManagementSettings settings;
    EXPECT_TRUE(settings.enable_time_regulation);
    EXPECT_TRUE(settings.enable_time_constrained);
    EXPECT_DOUBLE_EQ(settings.lookahead.value(), 0.1);
    EXPECT_FALSE(settings.enable_asynchronous_delivery);
}

// ============================================================================
// HLA Configuration Tests
// ============================================================================

TEST(HLAConfigurationTest, DefaultConfig) {
    auto config = HLAConfiguration::default_config();

    EXPECT_EQ(config.federation_name, "JaguarFederation");
    EXPECT_EQ(config.federate_name, "JaguarEngine");
    EXPECT_EQ(config.federate_type, "SimulationEngine");
    EXPECT_FALSE(config.fom_module_paths.empty());
    EXPECT_EQ(config.fom_module_paths[0], "RPR_FOM_v2.xml");
}

TEST(HLAConfigurationTest, CustomConfig) {
    HLAConfiguration config;
    config.federation_name = "TestFederation";
    config.federate_name = "TestFederate";
    config.connection.rti_host = "rti.example.com";
    config.connection.rti_port = 9999;
    config.time_management.lookahead = LogicalTimeInterval(0.5);

    EXPECT_EQ(config.federation_name, "TestFederation");
    EXPECT_EQ(config.connection.rti_host, "rti.example.com");
    EXPECT_DOUBLE_EQ(config.time_management.lookahead.value(), 0.5);
}

// ============================================================================
// Publication/Subscription Settings Tests
// ============================================================================

TEST(PublicationSettingsTest, DefaultConstruction) {
    PublicationSettings settings;
    EXPECT_TRUE(settings.published_object_classes.empty());
    EXPECT_TRUE(settings.published_attributes.empty());
    EXPECT_TRUE(settings.published_interactions.empty());
}

TEST(SubscriptionSettingsTest, DefaultConstruction) {
    SubscriptionSettings settings;
    EXPECT_TRUE(settings.subscribed_object_classes.empty());
    EXPECT_TRUE(settings.subscribed_interactions.empty());
    EXPECT_FALSE(settings.passive_subscription);
}

// ============================================================================
// HLA Result Tests
// ============================================================================

TEST(HLAResultTest, SuccessValue) {
    EXPECT_EQ(static_cast<UInt32>(HLAResult::Success), 0);
}

TEST(HLAResultTest, ErrorValues) {
    EXPECT_NE(HLAResult::Success, HLAResult::ConnectionFailed);
    EXPECT_NE(HLAResult::Success, HLAResult::FederationExecutionDoesNotExist);
    EXPECT_NE(HLAResult::Success, HLAResult::ObjectClassNotDefined);
    EXPECT_NE(HLAResult::Success, HLAResult::RTIinternalError);
}

TEST(HLAResultTest, ToString) {
    EXPECT_STREQ(hla_result_to_string(HLAResult::Success), "Success");
    EXPECT_STREQ(hla_result_to_string(HLAResult::ConnectionFailed), "ConnectionFailed");
    EXPECT_STREQ(hla_result_to_string(HLAResult::FederationExecutionDoesNotExist), "FederationExecutionDoesNotExist");
}

// ============================================================================
// DDM Types Tests
// ============================================================================

TEST(DimensionDefinitionTest, DefaultConstruction) {
    DimensionDefinition def;
    EXPECT_FALSE(def.handle.is_valid());
    EXPECT_TRUE(def.name.empty());
    EXPECT_EQ(def.lower_bound, 0);
    EXPECT_EQ(def.upper_bound, 0);
}

TEST(RegionExtentTest, DefaultConstruction) {
    RegionExtent extent;
    EXPECT_FALSE(extent.dimension.is_valid());
    EXPECT_EQ(extent.lower_bound, 0);
    EXPECT_EQ(extent.upper_bound, 0);
}

TEST(RegionTest, DefaultConstruction) {
    Region region;
    EXPECT_FALSE(region.handle.is_valid());
    EXPECT_TRUE(region.extents.empty());
}

TEST(RegionTest, AddExtents) {
    Region region;
    region.handle = RegionHandle(1);

    RegionExtent extent1;
    extent1.dimension = DimensionHandle(1);
    extent1.lower_bound = 0;
    extent1.upper_bound = 1000;
    region.extents.push_back(extent1);

    RegionExtent extent2;
    extent2.dimension = DimensionHandle(2);
    extent2.lower_bound = 0;
    extent2.upper_bound = 2000;
    region.extents.push_back(extent2);

    EXPECT_EQ(region.extents.size(), 2);
}

// ============================================================================
// Ownership Types Tests
// ============================================================================

TEST(AttributeOwnershipDivestitureTest, DefaultConstruction) {
    AttributeOwnershipDivestiture div;
    EXPECT_FALSE(div.object.is_valid());
    EXPECT_TRUE(div.attributes.empty());
    EXPECT_TRUE(div.reason.empty());
}

TEST(AttributeOwnershipAcquisitionTest, DefaultConstruction) {
    AttributeOwnershipAcquisition acq;
    EXPECT_FALSE(acq.object.is_valid());
    EXPECT_TRUE(acq.attributes.empty());
    EXPECT_FALSE(acq.previous_owner.is_valid());
}

// ============================================================================
// Sync Point Info Tests
// ============================================================================

TEST(SyncPointInfoTest, DefaultConstruction) {
    SyncPointInfo info;
    EXPECT_TRUE(info.label.empty());
    EXPECT_EQ(info.status, SyncPointStatus::Announced);
    EXPECT_TRUE(info.tag.empty());
}

TEST(SyncPointInfoTest, SetValues) {
    SyncPointInfo info;
    info.label = "ReadyToStart";
    info.status = SyncPointStatus::Achieved;
    info.tag = {0x01, 0x02, 0x03};
    info.announced_at = std::chrono::system_clock::now();

    EXPECT_EQ(info.label, "ReadyToStart");
    EXPECT_EQ(info.status, SyncPointStatus::Achieved);
    EXPECT_EQ(info.tag.size(), 3);
}

// ============================================================================
// Attribute Definition Tests
// ============================================================================

TEST(AttributeDefinitionTest, DefaultConstruction) {
    AttributeDefinition def;
    EXPECT_FALSE(def.handle.is_valid());
    EXPECT_TRUE(def.name.empty());
    EXPECT_EQ(def.order, OrderType::Receive);
    EXPECT_EQ(def.transport, TransportationType::Reliable);
}

TEST(AttributeDefinitionTest, SetValues) {
    AttributeDefinition def;
    def.handle = AttributeHandle(1);
    def.name = "Position";
    def.datatype = "WorldLocationStruct";
    def.order = OrderType::Timestamp;
    def.transport = TransportationType::BestEffort;

    EXPECT_TRUE(def.handle.is_valid());
    EXPECT_EQ(def.name, "Position");
    EXPECT_EQ(def.order, OrderType::Timestamp);
}

// ============================================================================
// Parameter Definition Tests
// ============================================================================

TEST(ParameterDefinitionTest, DefaultConstruction) {
    ParameterDefinition def;
    EXPECT_FALSE(def.handle.is_valid());
    EXPECT_TRUE(def.name.empty());
    EXPECT_TRUE(def.datatype.empty());
}

// ============================================================================
// Factory Function Tests
// ============================================================================

TEST(RTIAmbassadorTest, CreateAmbassador) {
    auto config = HLAConfiguration::default_config();
    auto ambassador = create_rti_ambassador(config);
    EXPECT_NE(ambassador, nullptr);
}

TEST(FederateAmbassadorTest, CreateAmbassador) {
    auto ambassador = create_federate_ambassador();
    EXPECT_NE(ambassador, nullptr);
}

// ============================================================================
// Time Advance State Tests
// ============================================================================

TEST(TimeAdvanceStateTest, EnumValues) {
    EXPECT_EQ(static_cast<UInt8>(TimeAdvanceState::Idle), 0);
    EXPECT_NE(TimeAdvanceState::RequestPending, TimeAdvanceState::Granted);
}

// ============================================================================
// Routing Space Definition Tests
// ============================================================================

TEST(RoutingSpaceDefinitionTest, DefaultConstruction) {
    RoutingSpaceDefinition def;
    EXPECT_TRUE(def.name.empty());
    EXPECT_TRUE(def.dimensions.empty());
}

TEST(RoutingSpaceDefinitionTest, AddDimensions) {
    RoutingSpaceDefinition def;
    def.name = "GeographicSpace";

    DimensionDefinition dim1;
    dim1.handle = DimensionHandle(1);
    dim1.name = "Latitude";
    dim1.lower_bound = 0;
    dim1.upper_bound = 180000000; // In 10-nanodegree units
    def.dimensions.push_back(dim1);

    DimensionDefinition dim2;
    dim2.handle = DimensionHandle(2);
    dim2.name = "Longitude";
    dim2.lower_bound = 0;
    dim2.upper_bound = 360000000;
    def.dimensions.push_back(dim2);

    EXPECT_EQ(def.dimensions.size(), 2);
    EXPECT_EQ(def.dimensions[0].name, "Latitude");
}

// ============================================================================
// Complex Scenario Tests
// ============================================================================

TEST(HLAScenarioTest, CreateObjectWithAttributes) {
    // Create an object class definition
    ObjectClassDefinition aircraft_class;
    aircraft_class.handle = ObjectClassHandle(100);
    aircraft_class.name = "HLAobjectRoot.BaseEntity.PhysicalEntity.Platform.Aircraft";

    // Add attributes
    AttributeDefinition pos_attr;
    pos_attr.handle = AttributeHandle(1);
    pos_attr.name = "SpatialStatic";
    pos_attr.order = OrderType::Receive;
    aircraft_class.attributes.push_back(pos_attr);

    AttributeDefinition vel_attr;
    vel_attr.handle = AttributeHandle(2);
    vel_attr.name = "SpatialVelocity";
    vel_attr.order = OrderType::Timestamp;
    aircraft_class.attributes.push_back(vel_attr);

    // Create an instance
    ObjectInstance f16;
    f16.handle = ObjectInstanceHandle(1000);
    f16.class_handle = aircraft_class.handle;
    f16.instance_name = "F-16_Eagle01";
    f16.is_local = true;

    // Set attribute values
    std::vector<UInt8> position_data = {0x01, 0x02, 0x03, 0x04};
    f16.attribute_values[pos_attr.handle] = position_data;

    EXPECT_EQ(f16.instance_name, "F-16_Eagle01");
    EXPECT_TRUE(f16.is_local);
    EXPECT_EQ(f16.attribute_values.size(), 1);
}

TEST(HLAScenarioTest, CreateInteractionWithParameters) {
    // Create an interaction class definition
    InteractionClassDefinition fire_interaction;
    fire_interaction.handle = InteractionClassHandle(200);
    fire_interaction.name = "HLAinteractionRoot.WeaponFire";
    fire_interaction.order = OrderType::Timestamp;
    fire_interaction.transport = TransportationType::Reliable;

    // Add parameters
    ParameterDefinition firing_entity;
    firing_entity.handle = ParameterHandle(1);
    firing_entity.name = "FiringObjectIdentifier";
    fire_interaction.parameters.push_back(firing_entity);

    ParameterDefinition target_entity;
    target_entity.handle = ParameterHandle(2);
    target_entity.name = "TargetObjectIdentifier";
    fire_interaction.parameters.push_back(target_entity);

    ParameterDefinition munition_type;
    munition_type.handle = ParameterHandle(3);
    munition_type.name = "MunitionType";
    fire_interaction.parameters.push_back(munition_type);

    // Create parameter value set
    ParameterValueSet params;
    params.set_parameter(firing_entity.handle, {0x01, 0x02});
    params.set_parameter(target_entity.handle, {0x03, 0x04});
    params.set_parameter(munition_type.handle, {0x05, 0x06, 0x07});

    EXPECT_EQ(fire_interaction.parameters.size(), 3);
    EXPECT_EQ(params.size(), 3);
    EXPECT_TRUE(params.has_parameter(firing_entity.handle));
}

TEST(HLAScenarioTest, TimeManagedFederate) {
    // Simulate a time-managed federate configuration
    FederateInfo info;
    info.handle = FederateHandle(1);
    info.federate_name = "TimeManagedSim";
    info.federate_type = "Training";
    info.time_status = TimeRegulationStatus::Both;
    info.current_time = LogicalTime(0.0);
    info.lookahead = LogicalTimeInterval(0.1);

    // Simulate time advancement
    LogicalTime requested_time(1.0);
    LogicalTime granted_time = requested_time; // Simulated grant

    info.current_time = granted_time;

    EXPECT_DOUBLE_EQ(info.current_time.value(), 1.0);
    EXPECT_EQ(info.time_status, TimeRegulationStatus::Both);
}

// ============================================================================
// Edge Case Tests
// ============================================================================

TEST(EdgeCaseTest, EmptyAttributeValueSet) {
    AttributeValueSet avs;

    EXPECT_TRUE(avs.empty());
    EXPECT_EQ(avs.size(), 0);

    // Getting non-existent attribute should return nullopt
    auto result = avs.get_attribute(AttributeHandle(1));
    EXPECT_FALSE(result.has_value());
}

TEST(EdgeCaseTest, LargeAttributeValue) {
    AttributeValueSet avs;
    AttributeHandle handle(1);

    // Create a large attribute value
    std::vector<UInt8> large_value(10000, 0xFF);
    avs.set_attribute(handle, large_value);

    auto retrieved = avs.get_attribute(handle);
    ASSERT_TRUE(retrieved.has_value());
    EXPECT_EQ(retrieved.value().size(), 10000);
}

TEST(EdgeCaseTest, HandleEdgeValues) {
    // Test with maximum values
    ObjectInstanceHandle max_handle(UINT64_MAX);
    EXPECT_TRUE(max_handle.is_valid());
    EXPECT_EQ(max_handle.value, UINT64_MAX);

    // Test with 1 (minimum valid)
    ObjectInstanceHandle min_handle(1);
    EXPECT_TRUE(min_handle.is_valid());
}

TEST(EdgeCaseTest, LogicalTimeEdgeCases) {
    LogicalTime zero(0.0);
    LogicalTime negative(-1.0);
    LogicalTime large(1e15);

    EXPECT_DOUBLE_EQ(zero.value(), 0.0);
    EXPECT_LT(negative.value(), 0.0);
    EXPECT_GT(large.value(), 1e14);

    // Time comparison with same values
    EXPECT_TRUE(zero <= LogicalTime(0.0));
    EXPECT_TRUE(zero >= LogicalTime(0.0));
}
