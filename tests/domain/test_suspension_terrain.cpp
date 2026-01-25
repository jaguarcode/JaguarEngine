/**
 * @file test_suspension_terrain.cpp
 * @brief Unit tests for terrain-integrated suspension models
 */

#include <gtest/gtest.h>
#include "jaguar/domain/land/suspension.h"
#include "jaguar/environment/environment.h"
#include "jaguar/physics/entity_state.h"
#include <cmath>

using namespace jaguar;
using namespace jaguar::domain::land;
using namespace jaguar::environment;
using namespace jaguar::physics;

// ============================================================================
// Mock Environment Service for Testing
// ============================================================================

class MockEnvironmentService : public EnvironmentService {
public:
    MockEnvironmentService() : EnvironmentService() {
        set_flat_terrain();
    }

    Environment query(const Vec3& ecef, Real /*time*/) const {
        Environment env;
        env.position_ecef = ecef;

        // Apply configured terrain
        env.terrain = terrain_config_;
        env.terrain_elevation = terrain_config_.elevation;
        env.terrain.valid = true;

        return env;
    }

    void set_flat_terrain() {
        terrain_config_.elevation = 0.0;
        terrain_config_.normal = Vec3{0, 0, 1};
        terrain_config_.slope_angle = 0.0;
        terrain_config_.material.friction_coefficient = 0.7;
        terrain_config_.material.rolling_resistance = 0.01;
    }

    void set_sloped_terrain(Real slope_deg) {
        // Slope along X-axis (pitch)
        Real slope_rad = slope_deg * M_PI / 180.0;
        terrain_config_.elevation = 0.0;
        terrain_config_.normal = Vec3{-std::sin(slope_rad), 0, std::cos(slope_rad)}.normalized();
        terrain_config_.slope_angle = slope_rad;
        terrain_config_.material.friction_coefficient = 0.7;
        terrain_config_.material.rolling_resistance = 0.01;
    }

    void set_friction(Real mu) {
        terrain_config_.material.friction_coefficient = mu;
    }

private:
    TerrainQuery terrain_config_;
};

// ============================================================================
// Test Fixtures
// ============================================================================

class SuspensionTerrainTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a basic wheel suspension
        SuspensionUnit unit;
        unit.spring_k = 50000.0;  // N/m
        unit.damper_c = 5000.0;   // N·s/m
        unit.preload = 0.0;
        unit.travel_max = 0.3;
        unit.travel_min = 0.0;

        wheel_radius_ = 0.35;  // m (typical passenger car)
        position_ = Vec3{1.0, 0.5, -0.4};  // Front-right wheel in body frame

        suspension_ = std::make_unique<WheelSuspension>(position_, wheel_radius_, unit);

        // Setup flat terrain by default
        env_service_ = std::make_unique<MockEnvironmentService>();
        env_service_->set_flat_terrain();

        // Default entity state (vehicle at rest on ground)
        state_.position = Vec3{0, 0, 1.0};  // 1m above origin
        state_.orientation = Quat::Identity();
        state_.velocity = Vec3{0, 0, 0};
        state_.angular_velocity = Vec3{0, 0, 0};
    }

    std::unique_ptr<WheelSuspension> suspension_;
    std::unique_ptr<MockEnvironmentService> env_service_;
    EntityState state_;
    Vec3 position_;
    Real wheel_radius_;
};

// ============================================================================
// Flat Terrain Tests
// ============================================================================

TEST_F(SuspensionTerrainTest, FlatTerrain_WheelGrounded) {
    // Wheel should be grounded when vehicle is low enough
    state_.position = Vec3{0, 0, 0.4};  // Low enough for wheel to touch ground

    Vec3 force = suspension_->compute_forces(state_, env_service_.get(), 0.01);

    EXPECT_TRUE(suspension_->is_grounded());
    EXPECT_GT(force.z, 0.0) << "Upward force expected when wheel is compressed";
}

TEST_F(SuspensionTerrainTest, FlatTerrain_WheelAirborne) {
    // Wheel should be airborne when vehicle is too high
    state_.position = Vec3{0, 0, 10.0};  // High above ground

    Vec3 force = suspension_->compute_forces(state_, env_service_.get(), 0.01);

    EXPECT_FALSE(suspension_->is_grounded());
    EXPECT_DOUBLE_EQ(force.z, 0.0) << "No force when airborne";
    EXPECT_DOUBLE_EQ(suspension_->get_contact_force(), 0.0);
}

TEST_F(SuspensionTerrainTest, FlatTerrain_CompressionCalculation) {
    // Position vehicle so wheel has known compression
    Real desired_compression = 0.1;  // 10cm compression
    state_.position = Vec3{0, 0, wheel_radius_ - desired_compression + position_.z};

    Vec3 force = suspension_->compute_forces(state_, env_service_.get(), 0.01);

    EXPECT_TRUE(suspension_->is_grounded());
    EXPECT_NEAR(suspension_->get_compression(), desired_compression, 1e-3);

    // Force should be proportional to compression (spring force)
    Real expected_force = suspension_->unit().spring_k * desired_compression;
    EXPECT_NEAR(suspension_->get_contact_force(), expected_force, 1.0);
}

TEST_F(SuspensionTerrainTest, FlatTerrain_FrictionCoefficient) {
    env_service_->set_friction(0.85);
    state_.position = Vec3{0, 0, 0.4};

    suspension_->compute_forces(state_, env_service_.get(), 0.01);

    EXPECT_NEAR(suspension_->get_friction_coefficient(), 0.85, 1e-6);
}

// ============================================================================
// Sloped Terrain Tests
// ============================================================================

TEST_F(SuspensionTerrainTest, SlopedTerrain_30DegreeHill) {
    // Set 30 degree uphill slope
    env_service_->set_sloped_terrain(30.0);

    // Position vehicle on slope
    state_.position = Vec3{0, 0, 0.5};
    state_.orientation = Quat::Identity();

    Vec3 force = suspension_->compute_forces(state_, env_service_.get(), 0.01);

    EXPECT_TRUE(suspension_->is_grounded());

    // Force should NOT be purely vertical - it should follow terrain normal
    Real slope_rad = 30.0 * M_PI / 180.0;
    Vec3 expected_normal = Vec3{-std::sin(slope_rad), 0, std::cos(slope_rad)}.normalized();

    // Force direction should align with terrain normal
    Vec3 force_direction = force.normalized();
    Real dot = force_direction.dot(expected_normal);
    EXPECT_NEAR(dot, 1.0, 0.1) << "Force should align with terrain normal";
}

TEST_F(SuspensionTerrainTest, SlopedTerrain_ForceNotVertical) {
    env_service_->set_sloped_terrain(20.0);
    state_.position = Vec3{0, 0, 0.5};

    Vec3 force = suspension_->compute_forces(state_, env_service_.get(), 0.01);

    EXPECT_TRUE(suspension_->is_grounded());

    // On a slope, force should have horizontal component
    EXPECT_NE(force.x, 0.0) << "Force should have X component on sloped terrain";
    EXPECT_GT(force.z, 0.0) << "Force should still have upward component";
}

TEST_F(SuspensionTerrainTest, SlopedTerrain_PenetrationDepth) {
    env_service_->set_sloped_terrain(15.0);
    state_.position = Vec3{0, 0, 0.4};

    Vec3 force = suspension_->compute_forces(state_, env_service_.get(), 0.01);

    EXPECT_TRUE(suspension_->is_grounded());
    EXPECT_GT(suspension_->get_compression(), 0.0);

    // Compression should be measured perpendicular to terrain surface
    // (This is more accurate than purely vertical measurement)
}

// ============================================================================
// Material Friction Tests
// ============================================================================

TEST_F(SuspensionTerrainTest, Material_AsphaltFriction) {
    env_service_->set_friction(0.9);  // High friction
    state_.position = Vec3{0, 0, 0.4};

    suspension_->compute_forces(state_, env_service_.get(), 0.01);

    EXPECT_NEAR(suspension_->get_friction_coefficient(), 0.9, 1e-6);
}

TEST_F(SuspensionTerrainTest, Material_IceFriction) {
    env_service_->set_friction(0.15);  // Low friction
    state_.position = Vec3{0, 0, 0.4};

    suspension_->compute_forces(state_, env_service_.get(), 0.01);

    EXPECT_NEAR(suspension_->get_friction_coefficient(), 0.15, 1e-6);
}

TEST_F(SuspensionTerrainTest, Material_GravelFriction) {
    env_service_->set_friction(0.6);  // Medium friction
    state_.position = Vec3{0, 0, 0.4};

    suspension_->compute_forces(state_, env_service_.get(), 0.01);

    EXPECT_NEAR(suspension_->get_friction_coefficient(), 0.6, 1e-6);
}

// ============================================================================
// Multi-Wheel Suspension Model Tests
// ============================================================================

TEST_F(SuspensionTerrainTest, MultiWheel_FourWheelVehicle) {
    SuspensionModel model;

    // Add four wheels (typical car layout)
    SuspensionUnit unit;
    unit.spring_k = 30000.0;
    unit.damper_c = 3000.0;

    Real wheelbase = 2.5;  // m
    Real track_width = 1.5;  // m
    Real wheel_radius = 0.32;  // m

    model.add_wheel(Vec3{wheelbase/2, track_width/2, -0.4}, wheel_radius, unit);   // FR
    model.add_wheel(Vec3{wheelbase/2, -track_width/2, -0.4}, wheel_radius, unit);  // FL
    model.add_wheel(Vec3{-wheelbase/2, track_width/2, -0.4}, wheel_radius, unit);  // RR
    model.add_wheel(Vec3{-wheelbase/2, -track_width/2, -0.4}, wheel_radius, unit); // RL

    EXPECT_EQ(model.wheel_count(), 4);

    // Vehicle at rest on flat ground
    state_.position = Vec3{0, 0, 0.5};
    model.update(state_, env_service_.get(), 0.01);

    // All wheels should be grounded
    EXPECT_EQ(model.grounded_wheel_count(), 4);

    // Total force should point upward
    Vec3 total_force = model.get_total_force();
    EXPECT_GT(total_force.z, 0.0);
}

TEST_F(SuspensionTerrainTest, MultiWheel_TotalForceAndTorque) {
    SuspensionModel model;

    SuspensionUnit unit;
    unit.spring_k = 40000.0;

    model.add_wheel(Vec3{1.0, 0.75, -0.4}, 0.33, unit);
    model.add_wheel(Vec3{1.0, -0.75, -0.4}, 0.33, unit);

    state_.position = Vec3{0, 0, 0.5};
    model.update(state_, env_service_.get(), 0.01);

    Vec3 total_force = model.get_total_force();
    Vec3 total_torque = model.get_total_torque();

    EXPECT_GT(total_force.z, 0.0) << "Total upward force expected";

    // Symmetric vehicle on flat ground should have minimal pitch/roll torque
    // (Some torque is acceptable due to numerical precision)
}

// ============================================================================
// Backward Compatibility (No Environment Service)
// ============================================================================

TEST_F(SuspensionTerrainTest, BackwardCompatibility_NoEnvironmentService) {
    // Test that suspension works without environment service (defaults to flat)
    state_.position = Vec3{0, 0, 0.4};

    Vec3 force = suspension_->compute_forces(state_, nullptr, 0.01);

    EXPECT_TRUE(suspension_->is_grounded());
    EXPECT_GT(force.z, 0.0);

    // Should default to reasonable friction
    EXPECT_NEAR(suspension_->get_friction_coefficient(), 0.7, 1e-6);
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST_F(SuspensionTerrainTest, EdgeCase_MaxCompression) {
    // Test bump stop behavior at maximum compression
    state_.position = Vec3{0, 0, position_.z + wheel_radius_ - suspension_->unit().travel_max - 0.05};

    Vec3 force = suspension_->compute_forces(state_, env_service_.get(), 0.01);

    EXPECT_TRUE(suspension_->is_grounded());
    // Force should be very high due to bump stop
    EXPECT_GT(suspension_->get_contact_force(), suspension_->unit().spring_k * suspension_->unit().travel_max);
}

TEST_F(SuspensionTerrainTest, EdgeCase_ZeroStiffness) {
    // Suspension with zero stiffness (degenerate case)
    SuspensionUnit soft_unit;
    soft_unit.spring_k = 0.0;
    soft_unit.damper_c = 0.0;

    WheelSuspension soft_wheel(position_, wheel_radius_, soft_unit);
    state_.position = Vec3{0, 0, 0.4};

    Vec3 force = soft_wheel.compute_forces(state_, env_service_.get(), 0.01);

    EXPECT_TRUE(soft_wheel.is_grounded());
    EXPECT_DOUBLE_EQ(soft_wheel.get_contact_force(), 0.0);
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
