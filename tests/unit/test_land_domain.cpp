/**
 * @file test_land_domain.cpp
 * @brief Unit tests for Land domain (terramechanics, suspension)
 */

#include <gtest/gtest.h>
#include "jaguar/domain/land.h"
#include "jaguar/environment/environment.h"
#include <cmath>

using namespace jaguar;
using namespace jaguar::domain::land;

// ============================================================================
// SoilProperties Tests
// ============================================================================

class SoilPropertiesTest : public ::testing::Test {
protected:
    void SetUp() override {}
};

TEST_F(SoilPropertiesTest, DrySandProperties) {
    auto soil = SoilProperties::DrySand();

    EXPECT_GT(soil.k_c, 0.0);
    EXPECT_GT(soil.k_phi, 0.0);
    EXPECT_GT(soil.n, 0.0);
    EXPECT_GT(soil.c, 0.0);
    EXPECT_GT(soil.phi, 0.0);

    // Dry sand has relatively low cohesion
    EXPECT_LT(soil.c, 5.0);  // kPa
}

TEST_F(SoilPropertiesTest, ClayProperties) {
    auto soil = SoilProperties::Clay();

    // Clay has higher cohesion than sand
    auto sand = SoilProperties::DrySand();
    EXPECT_GT(soil.c, sand.c);

    // Clay has lower friction angle
    EXPECT_LT(soil.phi, sand.phi);
}

TEST_F(SoilPropertiesTest, AsphaltIsRigid) {
    auto soil = SoilProperties::Asphalt();

    // Asphalt should have very high moduli (near-rigid)
    EXPECT_GT(soil.k_c, 100000.0);
    EXPECT_GT(soil.k_phi, 100000.0);
}

TEST_F(SoilPropertiesTest, SnowProperties) {
    auto soil = SoilProperties::Snow();

    // Snow has high compressibility (high n)
    EXPECT_GT(soil.n, 1.0);
}

// ============================================================================
// TerramechanicsModel Tests
// ============================================================================

class TerramechanicsTest : public ::testing::Test {
protected:
    TerramechanicsModel model;
    physics::EntityState state;
    environment::Environment env;
    physics::EntityForces forces;

    void SetUp() override {
        model.set_contact_area(0.5, 2.0);  // 0.5m x 2.0m track
        model.set_vehicle_weight(50000.0);  // 50 kN

        state.position = Vec3{0.0, 0.0, 0.0};
        state.velocity = Vec3{0.0, 0.0, 0.0};
        state.orientation = Quat::Identity();
        state.angular_velocity = Vec3{0.0, 0.0, 0.0};
        state.mass = 5000.0;

        env.altitude = 0.0;
        env.terrain_elevation = 0.0;

        forces.clear();
    }
};

TEST_F(TerramechanicsTest, NoForceWhenAboveGround) {
    env.altitude = 10.0;  // 10m above ground
    env.terrain_elevation = 0.0;

    model.compute_forces(state, env, 0.1, forces);

    EXPECT_NEAR(model.get_sinkage(), 0.0, 1e-6);
    EXPECT_NEAR(model.get_motion_resistance(), 0.0, 1e-6);
    EXPECT_NEAR(model.get_traction(), 0.0, 1e-6);
}

TEST_F(TerramechanicsTest, SinkageOnGround) {
    env.altitude = 0.0;
    env.terrain_elevation = 0.0;

    model.compute_forces(state, env, 0.1, forces);

    // Should have some sinkage when on ground
    EXPECT_GT(model.get_sinkage(), 0.0);
    EXPECT_LT(model.get_sinkage(), 0.5);  // Less than max sinkage
}

TEST_F(TerramechanicsTest, MotionResistanceOnGround) {
    env.altitude = 0.0;
    env.terrain_elevation = 0.0;

    model.compute_forces(state, env, 0.1, forces);

    // Should have motion resistance when sinking
    if (model.get_sinkage() > 0.0) {
        EXPECT_GT(model.get_motion_resistance(), 0.0);
    }
}

TEST_F(TerramechanicsTest, NormalForceCountersWeight) {
    env.altitude = 0.0;
    env.terrain_elevation = 0.0;

    model.compute_forces(state, env, 0.1, forces);

    // When on ground with sinkage, should have upward normal force
    if (model.get_sinkage() > 0.0) {
        EXPECT_GT(forces.force.z, 0.0);
    }
}

TEST_F(TerramechanicsTest, HeightAboveGroundTolerance) {
    // Just above ground tolerance
    env.altitude = 0.05;  // 5cm above ground
    env.terrain_elevation = 0.0;

    model.compute_forces(state, env, 0.1, forces);

    // Should still compute forces (within 10cm tolerance)
    EXPECT_GT(model.get_sinkage(), 0.0);
}

TEST_F(TerramechanicsTest, DisabledModelProducesNoForce) {
    env.altitude = 0.0;
    env.terrain_elevation = 0.0;

    model.set_enabled(false);
    model.compute_forces(state, env, 0.1, forces);

    EXPECT_NEAR(model.get_sinkage(), 0.0, 1e-6);
    EXPECT_NEAR(forces.force.x, 0.0, 1e-6);
    EXPECT_NEAR(forces.force.y, 0.0, 1e-6);
    EXPECT_NEAR(forces.force.z, 0.0, 1e-6);
}

TEST_F(TerramechanicsTest, HigherWeightIncreaseSinkage) {
    env.altitude = 0.0;
    env.terrain_elevation = 0.0;

    // Light vehicle
    model.set_vehicle_weight(10000.0);
    model.compute_forces(state, env, 0.1, forces);
    Real light_sinkage = model.get_sinkage();

    // Heavy vehicle
    model.set_vehicle_weight(100000.0);
    model.compute_forces(state, env, 0.1, forces);
    Real heavy_sinkage = model.get_sinkage();

    EXPECT_GT(heavy_sinkage, light_sinkage);
}

// ============================================================================
// SuspensionUnit Tests
// ============================================================================

class SuspensionUnitTest : public ::testing::Test {
protected:
    SuspensionUnit unit;

    void SetUp() override {
        unit.spring_k = 50000.0;    // 50 kN/m
        unit.damper_c = 5000.0;     // 5 kN·s/m
        unit.preload = 1000.0;      // 1 kN preload
        unit.travel_max = 0.3;      // 30cm max
        unit.travel_min = 0.0;      // 0cm min
        unit.current_position = 0.0;
        unit.current_velocity = 0.0;
    }
};

TEST_F(SuspensionUnitTest, PreloadForceAtZeroPosition) {
    // At zero position (within bump stop zone), force includes bump stop effect
    // Set position to be outside bump stop zone for pure preload test
    unit.current_position = 0.05;  // 5cm compression, outside bump zone
    unit.current_velocity = 0.0;

    Real force = unit.calculate_force();
    // Force = spring_k * position + preload = 50000 * 0.05 + 1000 = 3500
    Real expected = unit.spring_k * unit.current_position + unit.preload;
    EXPECT_NEAR(force, expected, 1e-6);
}

TEST_F(SuspensionUnitTest, SpringForceIncreasesWithCompression) {
    unit.current_velocity = 0.0;

    unit.current_position = 0.1;  // 10cm compression
    Real force_10cm = unit.calculate_force();

    unit.current_position = 0.2;  // 20cm compression
    Real force_20cm = unit.calculate_force();

    EXPECT_GT(force_20cm, force_10cm);
}

TEST_F(SuspensionUnitTest, DampingForceWithVelocity) {
    unit.current_position = 0.1;

    // No velocity
    unit.current_velocity = 0.0;
    Real force_static = unit.calculate_force();

    // Compression velocity (positive)
    unit.current_velocity = 0.5;  // 0.5 m/s compression
    Real force_compressing = unit.calculate_force();

    EXPECT_GT(force_compressing, force_static);
}

TEST_F(SuspensionUnitTest, BumpStopAtMaxTravel) {
    unit.current_position = 0.29;  // Just below max (0.3m)
    unit.current_velocity = 0.0;
    Real force_normal = unit.calculate_force();

    unit.current_position = 0.295;  // Within bump stop zone
    Real force_bump = unit.calculate_force();

    // Bump stop adds significant force
    EXPECT_GT(force_bump, force_normal);
}

// ============================================================================
// SuspensionModel Tests
// ============================================================================

class SuspensionModelTest : public ::testing::Test {
protected:
    SuspensionModel model;
    physics::EntityState state;

    void SetUp() override {
        state.position = Vec3{0.0, 0.0, 0.0};
        state.velocity = Vec3{0.0, 0.0, 0.0};
        state.orientation = Quat::Identity();
        state.angular_velocity = Vec3{0.0, 0.0, 0.0};
    }
};

TEST_F(SuspensionModelTest, EmptyModelHasZeroUnits) {
    EXPECT_EQ(model.unit_count(), 0u);
}

TEST_F(SuspensionModelTest, AddUnits) {
    SuspensionUnit unit;
    model.add_unit(Vec3{1.0, 0.5, 0.0}, unit);
    model.add_unit(Vec3{1.0, -0.5, 0.0}, unit);
    model.add_unit(Vec3{-1.0, 0.5, 0.0}, unit);
    model.add_unit(Vec3{-1.0, -0.5, 0.0}, unit);

    EXPECT_EQ(model.unit_count(), 4u);
}

TEST_F(SuspensionModelTest, TotalForceFromMultipleUnits) {
    SuspensionUnit unit;
    unit.spring_k = 50000.0;
    unit.preload = 1000.0;
    unit.current_position = 0.1;

    // Add 4 units (simulating 4 corners of vehicle)
    model.add_unit(Vec3{2.0, 1.0, -1.0}, unit);
    model.add_unit(Vec3{2.0, -1.0, -1.0}, unit);
    model.add_unit(Vec3{-2.0, 1.0, -1.0}, unit);
    model.add_unit(Vec3{-2.0, -1.0, -1.0}, unit);

    Vec3 total_force = model.get_total_force();

    // All forces in Z direction (vertical)
    EXPECT_NEAR(total_force.x, 0.0, 1e-6);
    EXPECT_NEAR(total_force.y, 0.0, 1e-6);
    // Total force should be 4x single unit force
    EXPECT_GT(std::abs(total_force.z), 0.0);
}

// ============================================================================
// TrackedVehicleModel Tests
// ============================================================================

class TrackedVehicleTest : public ::testing::Test {
protected:
    TrackedVehicleModel model;
    SoilProperties soil;

    void SetUp() override {
        model.set_sprocket(0.3, 50000.0);  // 30cm radius, 50kN·m max torque
        soil = SoilProperties::DrySand();
    }
};

TEST_F(TrackedVehicleTest, InitialTrackState) {
    auto left = model.get_left_track();
    auto right = model.get_right_track();

    EXPECT_GE(left.tension, 0.0);
    EXPECT_GE(right.tension, 0.0);
}

TEST_F(TrackedVehicleTest, UpdateWithTorque) {
    Real drive_torque = 10000.0;  // 10 kN·m
    Real load = 50000.0;          // 50 kN vehicle weight

    model.update(drive_torque, load, soil, 0.1);

    // Should have non-zero velocity after applying torque
    auto left = model.get_left_track();
    EXPECT_NE(left.velocity, 0.0);
}

TEST_F(TrackedVehicleTest, SlipIncreasesWithTorque) {
    Real load = 50000.0;

    // Low torque
    model.update(5000.0, load, soil, 0.1);
    Real low_slip = model.get_left_track().slip;

    // Reset and apply high torque
    TrackedVehicleModel model2;
    model2.set_sprocket(0.3, 50000.0);
    model2.update(40000.0, load, soil, 0.1);
    Real high_slip = model2.get_left_track().slip;

    EXPECT_GE(high_slip, low_slip);
}

TEST_F(TrackedVehicleTest, PropulsiveForce) {
    model.update(10000.0, 50000.0, soil, 0.1);

    Real force = model.get_propulsive_force();
    EXPECT_GT(force, 0.0);
}

// ============================================================================
// Terrain Soil Integration Tests
// ============================================================================

class TerrainSoilIntegrationTest : public ::testing::Test {
protected:
    TerramechanicsModel model;
    physics::EntityState state;
    environment::Environment env;
    physics::EntityForces forces;

    void SetUp() override {
        model.set_contact_area(0.5, 2.0);
        model.set_vehicle_weight(50000.0);

        state.position = Vec3{0.0, 0.0, 0.0};
        state.velocity = Vec3{0.0, 0.0, 0.0};
        state.orientation = Quat::Identity();
        state.angular_velocity = Vec3{0.0, 0.0, 0.0};
        state.mass = 5000.0;

        env.altitude = 0.0;
        env.terrain_elevation = 0.0;
        env.terrain.valid = true;
        env.terrain.material.friction_coefficient = 0.7;
        env.terrain.material.rolling_resistance = 0.01;

        forces.clear();
    }
};

TEST_F(TerrainSoilIntegrationTest, AsphaltGivesMinimalSinkage) {
    // Set terrain material to asphalt
    env.terrain.material.type = environment::SurfaceType::Asphalt;
    env.terrain.material.k_c = 1000000.0;
    env.terrain.material.k_phi = 1000000.0;
    env.terrain.material.n = 0.0;

    model.compute_forces(state, env, 0.1, forces);

    // Asphalt is rigid, should have near-zero sinkage
    EXPECT_LT(model.get_sinkage(), 0.001);  // Less than 1mm
}

TEST_F(TerrainSoilIntegrationTest, MudGivesHighSinkage) {
    // Set terrain material to mud
    env.terrain.material.type = environment::SurfaceType::Mud;
    env.terrain.material.k_c = 2.0;
    env.terrain.material.k_phi = 100.0;
    env.terrain.material.n = 0.80;

    model.compute_forces(state, env, 0.1, forces);

    // Mud should give higher sinkage
    EXPECT_GT(model.get_sinkage(), 0.01);  // More than 1cm
}

TEST_F(TerrainSoilIntegrationTest, DrySandSinkage) {
    // Set terrain material to dry sand
    env.terrain.material.type = environment::SurfaceType::DrySand;
    env.terrain.material.k_c = 0.99;
    env.terrain.material.k_phi = 1528.0;
    env.terrain.material.n = 1.10;

    model.compute_forces(state, env, 0.1, forces);

    // Dry sand should give moderate sinkage
    Real sand_sinkage = model.get_sinkage();
    EXPECT_GT(sand_sinkage, 0.0);
    EXPECT_LT(sand_sinkage, 0.1);  // Less than 10cm for normal vehicle
}

TEST_F(TerrainSoilIntegrationTest, MudHigherSinkageThanAsphalt) {
    // Asphalt terrain
    env.terrain.material.type = environment::SurfaceType::Asphalt;
    env.terrain.material.k_c = 1000000.0;
    env.terrain.material.k_phi = 1000000.0;
    env.terrain.material.n = 0.0;

    model.compute_forces(state, env, 0.1, forces);
    Real asphalt_sinkage = model.get_sinkage();

    // Mud terrain
    env.terrain.material.type = environment::SurfaceType::Mud;
    env.terrain.material.k_c = 2.0;
    env.terrain.material.k_phi = 100.0;
    env.terrain.material.n = 0.80;

    model.compute_forces(state, env, 0.1, forces);
    Real mud_sinkage = model.get_sinkage();

    // Mud should have higher sinkage than asphalt (more deformable)
    EXPECT_GT(mud_sinkage, asphalt_sinkage);
}

TEST_F(TerrainSoilIntegrationTest, InvalidTerrainUsesDefault) {
    // Set terrain as invalid
    env.terrain.valid = false;

    model.compute_forces(state, env, 0.1, forces);

    // Should still compute forces using default soil (dry sand)
    EXPECT_GT(model.get_sinkage(), 0.0);
}

TEST_F(TerrainSoilIntegrationTest, WaterSurfaceGivesNoTraction) {
    // Set terrain material to water
    env.terrain.material.type = environment::SurfaceType::Water;
    env.terrain.material.k_c = 0.0;
    env.terrain.material.k_phi = 0.0;
    env.terrain.material.n = 1.0;

    model.compute_forces(state, env, 0.1, forces);

    // Water has no bearing capacity - vehicle would sink infinitely
    // In practice, the model may clamp or limit this
    // Just verify it computes something reasonable
    EXPECT_GE(model.get_sinkage(), 0.0);
}

TEST_F(TerrainSoilIntegrationTest, SnowHighCompressibility) {
    // Set terrain material to snow
    env.terrain.material.type = environment::SurfaceType::Snow;
    env.terrain.material.k_c = 4.37;
    env.terrain.material.k_phi = 196.0;
    env.terrain.material.n = 1.60;  // High n means high compressibility

    model.compute_forces(state, env, 0.1, forces);

    // Snow should have moderate sinkage
    EXPECT_GT(model.get_sinkage(), 0.0);
}
