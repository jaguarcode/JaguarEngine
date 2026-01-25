/**
 * @file test_xml_entity_loader.cpp
 * @brief Unit tests for XML entity loader
 */

#include <gtest/gtest.h>
#include "jaguar/interface/xml_entity_loader.h"
#include <sstream>
#include <fstream>

using namespace jaguar;
using namespace jaguar::interface;

// ============================================================================
// Test Fixtures
// ============================================================================

class XmlEntityLoaderTest : public ::testing::Test {
protected:
    XmlEntityLoader loader;

    std::string create_basic_aircraft_xml() {
        return R"(<?xml version="1.0" encoding="UTF-8"?>
<entity type="aircraft" name="TestAircraft">
    <metrics>
        <wingspan unit="ft">32.8</wingspan>
        <length unit="ft">49.5</length>
        <height unit="ft">16.7</height>
        <wing_area unit="ft2">300.0</wing_area>
    </metrics>

    <mass_balance>
        <empty_weight unit="lbs">19700</empty_weight>
        <fuel_capacity unit="lbs">6972</fuel_capacity>
        <cg_location unit="in">
            <x>180.0</x><y>0.0</y><z>0.0</z>
        </cg_location>
        <inertia unit="slug*ft2">
            <ixx>9496</ixx>
            <iyy>55814</iyy>
            <izz>63100</izz>
            <ixz>982</ixz>
        </inertia>
    </mass_balance>
</entity>)";
    }

    std::string create_aircraft_with_aero_xml() {
        return R"(<?xml version="1.0" encoding="UTF-8"?>
<entity type="aircraft" name="F-16C">
    <metrics>
        <wingspan unit="ft">32.8</wingspan>
        <length unit="ft">49.5</length>
        <height unit="ft">16.7</height>
        <wing_area unit="ft2">300.0</wing_area>
    </metrics>

    <mass_balance>
        <empty_weight unit="lbs">19700</empty_weight>
        <fuel_capacity unit="lbs">6972</fuel_capacity>
        <cg_location unit="in">
            <x>180.0</x><y>0.0</y><z>0.0</z>
        </cg_location>
        <inertia unit="slug*ft2">
            <ixx>9496</ixx>
            <iyy>55814</iyy>
            <izz>63100</izz>
            <ixz>982</ixz>
        </inertia>
    </mass_balance>

    <aerodynamics>
        <axis name="LIFT">
            <function name="aero/cl">
                <table>
                    <independentVar>aero/alpha_rad</independentVar>
                    <tableData>
                        -0.20  -0.68
                         0.00   0.20
                         0.24   1.20
                         0.60   0.70
                    </tableData>
                </table>
            </function>
        </axis>
    </aerodynamics>

    <propulsion>
        <engine type="turbofan" name="F110-GE-129">
            <max_thrust unit="lbf">29000</max_thrust>
            <afterburner_thrust unit="lbf">29000</afterburner_thrust>
        </engine>
    </propulsion>
</entity>)";
    }

    std::string create_vehicle_xml() {
        return R"(<?xml version="1.0" encoding="UTF-8"?>
<entity type="vehicle" name="M1A2">
    <metrics>
        <length unit="m">7.93</length>
        <height unit="m">2.44</height>
    </metrics>

    <mass_balance>
        <empty_weight unit="kg">61690</empty_weight>
        <cg_location unit="m">
            <x>0.0</x><y>0.0</y><z>0.5</z>
        </cg_location>
        <inertia unit="kg*m2">
            <ixx>100000</ixx>
            <iyy>200000</iyy>
            <izz>250000</izz>
        </inertia>
    </mass_balance>
</entity>)";
    }
};

// ============================================================================
// Basic Loading Tests
// ============================================================================

TEST_F(XmlEntityLoaderTest, LoadBasicAircraft) {
    std::string xml = create_basic_aircraft_xml();

    EntityDefinition def = loader.load_from_string(xml);

    EXPECT_EQ(def.name, "TestAircraft");
    EXPECT_EQ(def.type, "aircraft");
    EXPECT_EQ(def.domain, Domain::Air);
}

TEST_F(XmlEntityLoaderTest, LoadMetrics) {
    std::string xml = create_basic_aircraft_xml();

    EntityDefinition def = loader.load_from_string(xml);

    // Wingspan: 32.8 ft = 10.0 m (approximately)
    EXPECT_NEAR(def.metrics.wingspan, 10.0, 0.1);

    // Length: 49.5 ft = 15.1 m (approximately)
    EXPECT_NEAR(def.metrics.length, 15.1, 0.1);

    // Height: 16.7 ft = 5.1 m (approximately)
    EXPECT_NEAR(def.metrics.height, 5.1, 0.1);

    // Wing area: 300 ft² = 27.9 m² (approximately)
    EXPECT_NEAR(def.metrics.wing_area, 27.9, 0.5);
    EXPECT_NEAR(def.metrics.reference_area, 27.9, 0.5);
}

TEST_F(XmlEntityLoaderTest, LoadMassBalance) {
    std::string xml = create_basic_aircraft_xml();

    EntityDefinition def = loader.load_from_string(xml);

    // Empty weight: 19700 lbs = 8935 kg (approximately)
    EXPECT_NEAR(def.mass_balance.empty_mass, 8935.0, 10.0);

    // Fuel capacity: 6972 lbs = 3162 kg (approximately)
    EXPECT_NEAR(def.mass_balance.fuel_mass, 3162.0, 10.0);

    // CG location: 180 in = 4.572 m
    EXPECT_NEAR(def.mass_balance.cg_location.x, 4.572, 0.01);
    EXPECT_NEAR(def.mass_balance.cg_location.y, 0.0, 0.01);
    EXPECT_NEAR(def.mass_balance.cg_location.z, 0.0, 0.01);
}

TEST_F(XmlEntityLoaderTest, LoadInertia) {
    std::string xml = create_basic_aircraft_xml();

    EntityDefinition def = loader.load_from_string(xml);

    // Inertia values should be converted from slug*ft² to kg*m²
    // 1 slug*ft² = 1.3558179 kg*m²

    // Ixx: 9496 slug*ft² = 12876 kg*m² (approximately)
    EXPECT_NEAR(def.mass_balance.inertia(0, 0), 12876.0, 10.0);

    // Iyy: 55814 slug*ft² = 75670 kg*m² (approximately)
    EXPECT_NEAR(def.mass_balance.inertia(1, 1), 75670.0, 100.0);

    // Izz: 63100 slug*ft² = 85552 kg*m² (approximately)
    EXPECT_NEAR(def.mass_balance.inertia(2, 2), 85552.0, 100.0);
}

// ============================================================================
// Aerodynamics Tests
// ============================================================================

TEST_F(XmlEntityLoaderTest, LoadAerodynamics) {
    std::string xml = create_aircraft_with_aero_xml();

    EntityDefinition def = loader.load_from_string(xml);

    ASSERT_TRUE(def.aerodynamics.has_value());

    // Should have lift tables
    EXPECT_GT(def.aerodynamics->lift_tables.size(), 0u);

    // Check first lift coefficient table
    const auto& cl_table = def.aerodynamics->lift_tables[0];
    EXPECT_EQ(cl_table.name, "aero/cl");
    EXPECT_EQ(cl_table.breakpoints.size(), 4u);
    EXPECT_EQ(cl_table.values.size(), 4u);

    // Check table values
    EXPECT_NEAR(cl_table.breakpoints[0], -0.20, 0.001);
    EXPECT_NEAR(cl_table.values[0], -0.68, 0.001);
    EXPECT_NEAR(cl_table.breakpoints[1], 0.00, 0.001);
    EXPECT_NEAR(cl_table.values[1], 0.20, 0.001);
}

// ============================================================================
// Propulsion Tests
// ============================================================================

TEST_F(XmlEntityLoaderTest, LoadPropulsion) {
    std::string xml = create_aircraft_with_aero_xml();

    EntityDefinition def = loader.load_from_string(xml);

    ASSERT_TRUE(def.propulsion.has_value());

    EXPECT_EQ(def.propulsion->engine_type, "turbofan");

    // Max thrust: 29000 lbf = 129008 N (approximately)
    EXPECT_NEAR(def.propulsion->max_thrust, 129008.0, 100.0);
    EXPECT_NEAR(def.propulsion->afterburner_thrust, 129008.0, 100.0);
}

// ============================================================================
// Component Mask Tests
// ============================================================================

TEST_F(XmlEntityLoaderTest, ComponentMaskAircraft) {
    std::string xml = create_aircraft_with_aero_xml();

    EntityDefinition def = loader.load_from_string(xml);

    // Should have aerodynamics component
    EXPECT_NE(def.component_mask & physics::ComponentBits::Aerodynamics, 0u);

    // Should have propulsion component
    EXPECT_NE(def.component_mask & physics::ComponentBits::Propulsion, 0u);

    // Should have flight control component
    EXPECT_NE(def.component_mask & physics::ComponentBits::FlightControl, 0u);
}

TEST_F(XmlEntityLoaderTest, ComponentMaskVehicle) {
    std::string xml = create_vehicle_xml();

    EntityDefinition def = loader.load_from_string(xml);

    EXPECT_EQ(def.domain, Domain::Land);

    // Should have terramechanics component
    EXPECT_NE(def.component_mask & physics::ComponentBits::Terramechanics, 0u);

    // Should have ground contact component
    EXPECT_NE(def.component_mask & physics::ComponentBits::GroundContact, 0u);

    // Should have gravity component
    EXPECT_NE(def.component_mask & physics::ComponentBits::Gravity, 0u);
}

// ============================================================================
// Domain Parsing Tests
// ============================================================================

TEST_F(XmlEntityLoaderTest, DomainParsing) {
    // Test different domain types
    {
        std::string xml = R"(<?xml version="1.0" encoding="UTF-8"?>
<entity type="aircraft" name="Test"><metrics></metrics><mass_balance></mass_balance></entity>)";
        EntityDefinition def = loader.load_from_string(xml);
        EXPECT_EQ(def.domain, Domain::Air);
    }

    {
        std::string xml = R"(<?xml version="1.0" encoding="UTF-8"?>
<entity type="vehicle" name="Test"><metrics></metrics><mass_balance></mass_balance></entity>)";
        EntityDefinition def = loader.load_from_string(xml);
        EXPECT_EQ(def.domain, Domain::Land);
    }

    {
        std::string xml = R"(<?xml version="1.0" encoding="UTF-8"?>
<entity type="ship" name="Test"><metrics></metrics><mass_balance></mass_balance></entity>)";
        EntityDefinition def = loader.load_from_string(xml);
        EXPECT_EQ(def.domain, Domain::Sea);
    }

    {
        std::string xml = R"(<?xml version="1.0" encoding="UTF-8"?>
<entity type="satellite" name="Test"><metrics></metrics><mass_balance></mass_balance></entity>)";
        EntityDefinition def = loader.load_from_string(xml);
        EXPECT_EQ(def.domain, Domain::Space);
    }
}

// ============================================================================
// Error Handling Tests
// ============================================================================

TEST_F(XmlEntityLoaderTest, InvalidXml) {
    std::string invalid_xml = "This is not XML";

    EXPECT_THROW(loader.load_from_string(invalid_xml), std::runtime_error);
}

TEST_F(XmlEntityLoaderTest, MissingRootElement) {
    std::string xml = R"(<?xml version="1.0" encoding="UTF-8"?>
<not_entity>
    <metrics></metrics>
</not_entity>)";

    EXPECT_THROW(loader.load_from_string(xml), std::runtime_error);
}

TEST_F(XmlEntityLoaderTest, ErrorMessage) {
    std::string invalid_xml = "Not XML";

    try {
        loader.load_from_string(invalid_xml);
        FAIL() << "Expected std::runtime_error";
    } catch (const std::runtime_error& e) {
        // Should have set error message
        EXPECT_FALSE(loader.get_error().empty());
    }
}

// ============================================================================
// Unit Conversion Tests
// ============================================================================

TEST_F(XmlEntityLoaderTest, MetricUnits) {
    std::string xml = R"(<?xml version="1.0" encoding="UTF-8"?>
<entity type="aircraft" name="MetricAircraft">
    <metrics>
        <wingspan unit="m">10.0</wingspan>
        <length unit="m">15.0</length>
        <height unit="m">5.0</height>
        <wing_area unit="m2">30.0</wing_area>
    </metrics>

    <mass_balance>
        <empty_weight unit="kg">10000</empty_weight>
        <cg_location unit="m">
            <x>5.0</x><y>0.0</y><z>0.0</z>
        </cg_location>
        <inertia unit="kg*m2">
            <ixx>10000</ixx>
            <iyy>50000</iyy>
            <izz>60000</izz>
        </inertia>
    </mass_balance>
</entity>)";

    EntityDefinition def = loader.load_from_string(xml);

    // Values should be unchanged (already in SI)
    EXPECT_DOUBLE_EQ(def.metrics.wingspan, 10.0);
    EXPECT_DOUBLE_EQ(def.metrics.length, 15.0);
    EXPECT_DOUBLE_EQ(def.metrics.height, 5.0);
    EXPECT_DOUBLE_EQ(def.metrics.wing_area, 30.0);
    EXPECT_DOUBLE_EQ(def.mass_balance.empty_mass, 10000.0);
    EXPECT_DOUBLE_EQ(def.mass_balance.cg_location.x, 5.0);
    EXPECT_DOUBLE_EQ(def.mass_balance.inertia(0, 0), 10000.0);
}

// ============================================================================
// Optional Sections Tests
// ============================================================================

TEST_F(XmlEntityLoaderTest, MinimalEntity) {
    std::string xml = R"(<?xml version="1.0" encoding="UTF-8"?>
<entity type="generic" name="Minimal">
    <metrics></metrics>
    <mass_balance></mass_balance>
</entity>)";

    EntityDefinition def = loader.load_from_string(xml);

    EXPECT_EQ(def.name, "Minimal");
    EXPECT_EQ(def.type, "generic");
    EXPECT_EQ(def.domain, Domain::Generic);

    // Optional sections should not be present
    EXPECT_FALSE(def.aerodynamics.has_value());
    EXPECT_FALSE(def.propulsion.has_value());
}
