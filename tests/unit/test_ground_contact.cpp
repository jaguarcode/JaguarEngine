/**
 * @file test_ground_contact.cpp
 * @brief Unit tests for GroundContactConstraint
 */

#include <gtest/gtest.h>
#include "jaguar/physics/constraints/ground_contact_constraint.h"
#include "jaguar/physics/entity.h"
#include "jaguar/environment/terrain.h"

using namespace jaguar;
using namespace jaguar::physics;
using namespace jaguar::environment;

class GroundContactTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a simple entity state
        state_.position = Vec3(0, 0, 1.0);  // 1m above ground
        state_.velocity = Vec3(0, 0, -2.0);  // Falling at 2 m/s
        state_.angular_velocity = Vec3::Zero();
        state_.orientation = Quaternion::Identity();
        state_.mass = 10.0;  // 10 kg
        state_.inverse_mass = 1.0 / 10.0;
        state_.inertia_tensor = Mat3x3::Identity() * 0.1;
        state_.inverse_inertia_tensor = state_.inertia_tensor.inverse();
    }

    EntityState state_;
};

TEST_F(GroundContactTest, ConstructorBasic) {
    ConstraintBody entity = ConstraintBody::Dynamic(EntityId(1), Vec3::Zero());
    Vec3 contact_point(0, 0, 0);
    Vec3 terrain_normal(0, 0, 1);  // Vertical
    Real penetration_depth = 0.1;  // 10cm penetration

    GroundContactConstraint constraint(entity, contact_point, terrain_normal, penetration_depth);

    EXPECT_EQ(constraint.terrain_normal(), terrain_normal);
    EXPECT_EQ(constraint.contact_point(), contact_point);
    EXPECT_DOUBLE_EQ(constraint.penetration_depth(), penetration_depth);
    EXPECT_EQ(constraint.num_rows(), 3);  // 1 normal + 2 friction
}

TEST_F(GroundContactTest, ConstructorWithParams) {
    ConstraintBody entity = ConstraintBody::Dynamic(EntityId(1), Vec3::Zero());
    Vec3 contact_point(0, 0, 0);
    Vec3 terrain_normal(0, 0, 1);
    Real penetration_depth = 0.05;

    GroundContactParams params;
    params.friction_coefficient = 0.9;
    params.restitution = 0.3;
    params.enable_friction = true;

    GroundContactConstraint constraint(entity, contact_point, terrain_normal,
                                      penetration_depth, params);

    EXPECT_EQ(constraint.ground_params().friction_coefficient, 0.9);
    EXPECT_EQ(constraint.ground_params().restitution, 0.3);
    EXPECT_TRUE(constraint.ground_params().enable_friction);
}

TEST_F(GroundContactTest, SlopedTerrain) {
    // Test with sloped terrain (45 degree angle)
    ConstraintBody entity = ConstraintBody::Dynamic(EntityId(1), Vec3::Zero());
    Vec3 contact_point(0, 0, 0);
    Vec3 terrain_normal(0, 0.707107, 0.707107);  // 45 degrees
    Real penetration_depth = 0.1;

    GroundContactConstraint constraint(entity, contact_point, terrain_normal, penetration_depth);

    // Normal should be normalized
    Vec3 normal = constraint.terrain_normal();
    EXPECT_NEAR(normal.length(), 1.0, 1e-6);
    EXPECT_NEAR(normal.y(), 0.707107, 1e-5);
    EXPECT_NEAR(normal.z(), 0.707107, 1e-5);
}

TEST_F(GroundContactTest, BuildRowsNormalContact) {
    ConstraintBody entity = ConstraintBody::Dynamic(EntityId(1), Vec3::Zero());
    Vec3 contact_point(0, 0, 0);
    Vec3 terrain_normal(0, 0, 1);
    Real penetration_depth = 0.1;

    GroundContactConstraint constraint(entity, contact_point, terrain_normal, penetration_depth);

    // Build constraint rows
    ConstraintRow rows[3];
    constraint.build_rows(&state_, nullptr, 0.01, rows);

    // Normal row (first row)
    ConstraintRow& normal_row = rows[0];

    // Check Jacobian points in correct direction
    EXPECT_EQ(normal_row.J_linear_A, terrain_normal);

    // Check it's a unilateral constraint (can only push, not pull)
    EXPECT_DOUBLE_EQ(normal_row.lower_limit, 0.0);
    EXPECT_GT(normal_row.upper_limit, 1e10);

    // Check bias is positive for penetration (pushes entity up)
    EXPECT_GT(normal_row.rhs, 0.0);
}

TEST_F(GroundContactTest, BuildRowsFriction) {
    ConstraintBody entity = ConstraintBody::Dynamic(EntityId(1), Vec3::Zero());
    Vec3 contact_point(0, 0, 0);
    Vec3 terrain_normal(0, 0, 1);
    Real penetration_depth = 0.1;

    GroundContactParams params;
    params.enable_friction = true;
    params.friction_coefficient = 0.8;

    GroundContactConstraint constraint(entity, contact_point, terrain_normal,
                                      penetration_depth, params);

    // Build constraint rows
    ConstraintRow rows[3];
    constraint.build_rows(&state_, nullptr, 0.01, rows);

    // Friction rows (rows 1 and 2)
    ConstraintRow& friction1 = rows[1];
    ConstraintRow& friction2 = rows[2];

    // Check friction Jacobians are perpendicular to normal
    EXPECT_NEAR(friction1.J_linear_A.dot(terrain_normal), 0.0, 1e-6);
    EXPECT_NEAR(friction2.J_linear_A.dot(terrain_normal), 0.0, 1e-6);

    // Check friction rows are perpendicular to each other
    EXPECT_NEAR(friction1.J_linear_A.dot(friction2.J_linear_A), 0.0, 1e-6);

    // Check friction limits are symmetric
    EXPECT_EQ(friction1.lower_limit, -friction1.upper_limit);
    EXPECT_EQ(friction2.lower_limit, -friction2.upper_limit);
}

TEST_F(GroundContactTest, NoFrictionMode) {
    ConstraintBody entity = ConstraintBody::Dynamic(EntityId(1), Vec3::Zero());
    Vec3 contact_point(0, 0, 0);
    Vec3 terrain_normal(0, 0, 1);
    Real penetration_depth = 0.1;

    GroundContactParams params;
    params.enable_friction = false;

    GroundContactConstraint constraint(entity, contact_point, terrain_normal,
                                      penetration_depth, params);

    // Should only have 1 row (normal contact, no friction)
    EXPECT_EQ(constraint.num_rows(), 1);
}

TEST_F(GroundContactTest, UpdateFromTerrainQuery) {
    ConstraintBody entity = ConstraintBody::Dynamic(EntityId(1), Vec3::Zero());
    Vec3 contact_point(0, 0, 0);
    Vec3 terrain_normal(0, 0, 1);
    Real penetration_depth = 0.1;

    GroundContactConstraint constraint(entity, contact_point, terrain_normal, penetration_depth);

    // Create a terrain query with different properties
    TerrainQuery query;
    query.valid = true;
    query.normal = Vec3(0, 0.5, 0.866025);  // 30 degree slope
    query.material.friction_coefficient = 0.6;
    query.material.rolling_resistance = 0.02;

    // Update constraint from terrain query
    constraint.update_from_terrain(query);

    // Verify properties were updated
    EXPECT_NEAR(constraint.terrain_normal().y(), 0.5, 1e-5);
    EXPECT_NEAR(constraint.terrain_normal().z(), 0.866025, 1e-5);
    EXPECT_DOUBLE_EQ(constraint.ground_params().friction_coefficient, 0.6);
    EXPECT_DOUBLE_EQ(constraint.ground_params().rolling_friction, 0.02);
}

TEST_F(GroundContactTest, IsActiveCheck) {
    ConstraintBody entity = ConstraintBody::Dynamic(EntityId(1), Vec3::Zero());
    Vec3 contact_point(0, 0, 0);
    Vec3 terrain_normal(0, 0, 1);

    // Penetrating contact - should be active
    {
        GroundContactConstraint constraint(entity, contact_point, terrain_normal, 0.1);
        EXPECT_TRUE(constraint.is_active());
    }

    // Just touching - should be active
    {
        GroundContactConstraint constraint(entity, contact_point, terrain_normal, 0.0);
        EXPECT_TRUE(constraint.is_active());
    }

    // Slightly separated - should be active (within threshold)
    {
        GroundContactConstraint constraint(entity, contact_point, terrain_normal, -0.005);
        EXPECT_TRUE(constraint.is_active());
    }

    // Far separated - should not be active
    {
        GroundContactConstraint constraint(entity, contact_point, terrain_normal, -0.02);
        EXPECT_FALSE(constraint.is_active());
    }
}

TEST_F(GroundContactTest, RestitutionBias) {
    ConstraintBody entity = ConstraintBody::Dynamic(EntityId(1), Vec3::Zero());
    Vec3 contact_point(0, 0, 0);
    Vec3 terrain_normal(0, 0, 1);
    Real penetration_depth = 0.1;

    GroundContactParams params;
    params.restitution = 0.5;  // 50% bounce
    params.restitution_threshold = 1.0;  // Minimum velocity for bounce

    GroundContactConstraint constraint(entity, contact_point, terrain_normal,
                                      penetration_depth, params);

    // Build rows with falling entity
    ConstraintRow rows[3];
    constraint.build_rows(&state_, nullptr, 0.01, rows);

    // Normal row should have positive bias (penetration correction + restitution)
    ConstraintRow& normal_row = rows[0];
    EXPECT_GT(normal_row.rhs, 0.0);
}
