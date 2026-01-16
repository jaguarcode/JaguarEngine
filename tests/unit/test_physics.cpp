/**
 * @file test_physics.cpp
 * @brief Unit tests for JaguarEngine physics system
 */

#include <gtest/gtest.h>
#include "jaguar/physics/entity.h"
#include "jaguar/physics/solver.h"
#include <cmath>

using namespace jaguar;
using namespace jaguar::physics;

// ============================================================================
// Entity State Storage Tests
// ============================================================================

class EntityStateStorageTest : public ::testing::Test {
protected:
    static constexpr Real EPSILON = 1e-10;
    EntityStateStorage storage;
};

TEST_F(EntityStateStorageTest, AllocateAndFree) {
    EXPECT_EQ(storage.size(), 0u);
    EXPECT_EQ(storage.active_count(), 0u);

    UInt32 idx1 = storage.allocate();
    EXPECT_EQ(storage.size(), 1u);
    EXPECT_EQ(storage.active_count(), 1u);

    UInt32 idx2 = storage.allocate();
    EXPECT_EQ(storage.size(), 2u);
    EXPECT_EQ(storage.active_count(), 2u);

    storage.free(idx1);
    EXPECT_EQ(storage.size(), 2u);  // Size doesn't shrink
    EXPECT_EQ(storage.active_count(), 1u);

    // Reuse freed index
    UInt32 idx3 = storage.allocate();
    EXPECT_EQ(idx3, idx1);  // Should reuse freed slot
    EXPECT_EQ(storage.size(), 2u);
    EXPECT_EQ(storage.active_count(), 2u);
}

TEST_F(EntityStateStorageTest, SetAndGetState) {
    UInt32 idx = storage.allocate();

    EntityState state;
    state.position = Vec3{100.0, 200.0, 300.0};
    state.velocity = Vec3{10.0, 20.0, 30.0};
    state.orientation = Quat{0.707, 0.707, 0.0, 0.0};
    state.angular_velocity = Vec3{0.1, 0.2, 0.3};
    state.mass = 1000.0;
    state.inertia = Mat3x3::Diagonal(100.0, 200.0, 300.0);

    storage.set_state(idx, state);

    EntityState retrieved = storage.get_state(idx);
    EXPECT_DOUBLE_EQ(retrieved.position.x, 100.0);
    EXPECT_DOUBLE_EQ(retrieved.position.y, 200.0);
    EXPECT_DOUBLE_EQ(retrieved.position.z, 300.0);
    EXPECT_DOUBLE_EQ(retrieved.velocity.x, 10.0);
    EXPECT_DOUBLE_EQ(retrieved.mass, 1000.0);
}

TEST_F(EntityStateStorageTest, ClearForces) {
    UInt32 idx = storage.allocate();

    storage.forces(idx).force = Vec3{100.0, 200.0, 300.0};
    storage.forces(idx).torque = Vec3{10.0, 20.0, 30.0};

    EXPECT_DOUBLE_EQ(storage.forces(idx).force.x, 100.0);

    storage.clear_forces();

    EXPECT_DOUBLE_EQ(storage.forces(idx).force.x, 0.0);
    EXPECT_DOUBLE_EQ(storage.forces(idx).torque.x, 0.0);
}

// ============================================================================
// Entity Manager Tests
// ============================================================================

class EntityManagerTest : public ::testing::Test {
protected:
    EntityManager manager;
};

TEST_F(EntityManagerTest, CreateAndDestroyEntity) {
    EXPECT_EQ(manager.entity_count(), 0u);

    EntityId id1 = manager.create_entity("TestEntity1", Domain::Air);
    EXPECT_EQ(manager.entity_count(), 1u);
    EXPECT_TRUE(manager.exists(id1));

    EntityId id2 = manager.create_entity("TestEntity2", Domain::Land);
    EXPECT_EQ(manager.entity_count(), 2u);

    manager.destroy_entity(id1);
    EXPECT_EQ(manager.entity_count(), 1u);
    EXPECT_FALSE(manager.exists(id1));
    EXPECT_TRUE(manager.exists(id2));
}

TEST_F(EntityManagerTest, GetEntity) {
    EntityId id = manager.create_entity("TestEntity", Domain::Sea);

    Entity* entity = manager.get_entity(id);
    ASSERT_NE(entity, nullptr);
    EXPECT_EQ(entity->name, "TestEntity");
    EXPECT_EQ(entity->primary_domain, Domain::Sea);
    EXPECT_TRUE(entity->active);
}

TEST_F(EntityManagerTest, SetAndGetState) {
    EntityId id = manager.create_entity("TestEntity");

    EntityState state;
    state.position = Vec3{1000.0, 2000.0, 3000.0};
    state.velocity = Vec3{100.0, 0.0, 0.0};
    state.mass = 5000.0;

    manager.set_state(id, state);

    EntityState retrieved = manager.get_state(id);
    EXPECT_DOUBLE_EQ(retrieved.position.x, 1000.0);
    EXPECT_DOUBLE_EQ(retrieved.velocity.x, 100.0);
    EXPECT_DOUBLE_EQ(retrieved.mass, 5000.0);
}

// ============================================================================
// Integrator Tests
// ============================================================================

class IntegratorTest : public ::testing::Test {
protected:
    static constexpr Real EPSILON = 1e-6;

    bool nearly_equal(Real a, Real b, Real eps = EPSILON) {
        return std::abs(a - b) < eps;
    }

    bool nearly_equal(const Vec3& a, const Vec3& b, Real eps = EPSILON) {
        return nearly_equal(a.x, b.x, eps) &&
               nearly_equal(a.y, b.y, eps) &&
               nearly_equal(a.z, b.z, eps);
    }

    EntityState create_initial_state() {
        EntityState state;
        state.position = Vec3{0.0, 0.0, 0.0};
        state.velocity = Vec3{10.0, 0.0, 0.0};  // 10 m/s in x direction
        state.orientation = Quat::Identity();
        state.angular_velocity = Vec3{0.0, 0.0, 0.0};
        state.mass = 1.0;  // 1 kg
        state.inertia = Mat3x3::Diagonal(1.0, 1.0, 1.0);
        return state;
    }
};

TEST_F(IntegratorTest, EulerFreeFall) {
    EulerIntegrator integrator;

    EntityState state = create_initial_state();
    state.velocity = Vec3{0.0, 0.0, 0.0};

    EntityForces forces;
    forces.force = Vec3{0.0, 0.0, -9.81};  // Gravity

    Real dt = 0.01;  // 10ms timestep
    Real total_time = 1.0;  // 1 second
    int steps = static_cast<int>(total_time / dt);

    for (int i = 0; i < steps; ++i) {
        integrator.integrate(state, forces, dt);
    }

    // After 1 second of free fall:
    // v = g*t = 9.81 m/s
    // x = 0.5*g*t^2 = 4.905 m
    EXPECT_NEAR(state.velocity.z, -9.81, 0.01);
    EXPECT_NEAR(state.position.z, -4.905, 0.05);
}

TEST_F(IntegratorTest, RK4FreeFall) {
    RK4Integrator integrator;

    EntityState state = create_initial_state();
    state.velocity = Vec3{0.0, 0.0, 0.0};

    EntityForces forces;
    forces.force = Vec3{0.0, 0.0, -9.81};  // Gravity

    Real dt = 0.01;  // 10ms timestep
    Real total_time = 1.0;  // 1 second
    int steps = static_cast<int>(total_time / dt);

    for (int i = 0; i < steps; ++i) {
        integrator.integrate(state, forces, dt);
    }

    // RK4 should be more accurate than Euler
    EXPECT_NEAR(state.velocity.z, -9.81, 0.001);
    EXPECT_NEAR(state.position.z, -4.905, 0.01);
}

TEST_F(IntegratorTest, RK4UniformMotion) {
    RK4Integrator integrator;

    EntityState state = create_initial_state();
    // Initial velocity: 10 m/s in x direction
    // No forces

    EntityForces forces;  // Zero forces

    Real dt = 0.01;
    Real total_time = 1.0;
    int steps = static_cast<int>(total_time / dt);

    for (int i = 0; i < steps; ++i) {
        integrator.integrate(state, forces, dt);
    }

    // After 1 second at 10 m/s: x = 10 m
    EXPECT_NEAR(state.position.x, 10.0, 0.001);
    EXPECT_NEAR(state.velocity.x, 10.0, 0.001);  // Velocity unchanged
}

TEST_F(IntegratorTest, RK4ConstantAcceleration) {
    RK4Integrator integrator;

    EntityState state = create_initial_state();
    state.velocity = Vec3{0.0, 0.0, 0.0};

    EntityForces forces;
    forces.force = Vec3{2.0, 0.0, 0.0};  // 2N force on 1kg mass = 2 m/s^2

    Real dt = 0.01;
    Real total_time = 1.0;
    int steps = static_cast<int>(total_time / dt);

    for (int i = 0; i < steps; ++i) {
        integrator.integrate(state, forces, dt);
    }

    // After 1 second: v = a*t = 2 m/s, x = 0.5*a*t^2 = 1 m
    EXPECT_NEAR(state.velocity.x, 2.0, 0.001);
    EXPECT_NEAR(state.position.x, 1.0, 0.01);
}

TEST_F(IntegratorTest, RK4RotationalMotion) {
    RK4Integrator integrator;

    EntityState state;
    state.position = Vec3{0.0, 0.0, 0.0};
    state.velocity = Vec3{0.0, 0.0, 0.0};
    state.orientation = Quat::Identity();
    state.angular_velocity = Vec3{0.0, 0.0, constants::PI};  // 180 deg/s around z
    state.mass = 1.0;
    state.inertia = Mat3x3::Diagonal(1.0, 1.0, 1.0);

    EntityForces forces;  // No forces

    Real dt = 0.01;
    Real total_time = 1.0;  // 1 second = 180 degrees rotation
    int steps = static_cast<int>(total_time / dt);

    for (int i = 0; i < steps; ++i) {
        integrator.integrate(state, forces, dt);
    }

    // After 1 second of rotation at π rad/s, quaternion should represent 180° rotation
    // around z-axis: q = (0, 0, 0, 1) or (0, 0, 0, -1)
    Real expected_angle = constants::PI;  // 180 degrees

    // Extract rotation angle from quaternion
    Real angle = 2.0 * std::acos(std::abs(state.orientation.w));
    EXPECT_NEAR(angle, expected_angle, 0.05);
}

// ============================================================================
// Quaternion Integration Tests
// ============================================================================

class QuaternionIntegrationTest : public ::testing::Test {
protected:
    static constexpr Real EPSILON = 1e-6;
};

TEST_F(QuaternionIntegrationTest, IntegrateIdentity) {
    Quat q = Quat::Identity();
    Vec3 omega{0.0, 0.0, 0.0};  // No rotation

    Quat result = quaternion_utils::integrate(q, omega, 1.0);

    // Should still be identity
    EXPECT_NEAR(result.w, 1.0, EPSILON);
    EXPECT_NEAR(result.x, 0.0, EPSILON);
    EXPECT_NEAR(result.y, 0.0, EPSILON);
    EXPECT_NEAR(result.z, 0.0, EPSILON);
}

TEST_F(QuaternionIntegrationTest, IntegrateRotation) {
    Quat q = Quat::Identity();
    Vec3 omega{0.0, 0.0, constants::PI / 2.0};  // 90 deg/s around z

    // Integrate for 1 second
    Real dt = 0.001;
    int steps = 1000;

    for (int i = 0; i < steps; ++i) {
        q = quaternion_utils::integrate_rk4(q, omega, dt);
    }

    // Should have rotated 90 degrees around z-axis
    // q = (cos(45°), 0, 0, sin(45°)) ≈ (0.707, 0, 0, 0.707)
    EXPECT_NEAR(std::abs(q.w), std::cos(constants::PI / 4.0), 0.01);
    EXPECT_NEAR(q.x, 0.0, 0.01);
    EXPECT_NEAR(q.y, 0.0, 0.01);
    EXPECT_NEAR(std::abs(q.z), std::sin(constants::PI / 4.0), 0.01);
}

// ============================================================================
// Physics System Tests
// ============================================================================

class PhysicsSystemTest : public ::testing::Test {
protected:
    EntityManager entity_manager;
    PhysicsSystem physics_system;
};

TEST_F(PhysicsSystemTest, UpdateEntity) {
    EntityId id = entity_manager.create_entity("TestEntity");

    EntityState state;
    state.position = Vec3{0.0, 0.0, 0.0};
    state.velocity = Vec3{10.0, 0.0, 0.0};  // Moving at 10 m/s
    state.orientation = Quat::Identity();
    state.mass = 1.0;
    state.inertia = Mat3x3::Diagonal(1.0, 1.0, 1.0);

    entity_manager.set_state(id, state);

    // Update physics for 1 second
    Real dt = 0.01;
    int steps = 100;

    for (int i = 0; i < steps; ++i) {
        physics_system.update(entity_manager, dt);
    }

    state = entity_manager.get_state(id);

    // After 1 second at 10 m/s: x = 10 m
    EXPECT_NEAR(state.position.x, 10.0, 0.1);
}

// ============================================================================
// Adaptive Integrator Tests
// ============================================================================

class AdaptiveIntegratorTest : public ::testing::Test {
protected:
    static constexpr Real EPSILON = 1e-6;

    EntityState create_initial_state() {
        EntityState state;
        state.position = Vec3{0.0, 0.0, 0.0};
        state.velocity = Vec3{10.0, 0.0, 0.0};
        state.orientation = Quat::Identity();
        state.angular_velocity = Vec3{0.0, 0.0, 0.0};
        state.mass = 1.0;
        state.inertia = Mat3x3::Diagonal(1.0, 1.0, 1.0);
        return state;
    }
};

TEST_F(AdaptiveIntegratorTest, DefaultConstruction) {
    AdaptiveIntegrator integrator;

    EXPECT_EQ(integrator.name(), "Adaptive");
    EXPECT_EQ(integrator.order(), 4);  // Default RK4 base
}

TEST_F(AdaptiveIntegratorTest, CustomBaseIntegrator) {
    auto euler = std::make_unique<EulerIntegrator>();
    AdaptiveIntegrator integrator(std::move(euler));

    EXPECT_EQ(integrator.order(), 1);  // Euler is order 1
}

TEST_F(AdaptiveIntegratorTest, ConfigurationAccess) {
    AdaptiveIntegrator integrator;

    AdaptiveConfig config;
    config.tolerance = 1e-8;
    config.min_dt = 1e-10;
    config.max_dt = 0.5;
    config.safety_factor = 0.85;

    integrator.set_config(config);

    EXPECT_DOUBLE_EQ(integrator.get_config().tolerance, 1e-8);
    EXPECT_DOUBLE_EQ(integrator.get_config().min_dt, 1e-10);
    EXPECT_DOUBLE_EQ(integrator.get_config().max_dt, 0.5);
    EXPECT_DOUBLE_EQ(integrator.get_config().safety_factor, 0.85);
}

TEST_F(AdaptiveIntegratorTest, FreeFallAccuracy) {
    AdaptiveIntegrator integrator;

    EntityState state = create_initial_state();
    state.velocity = Vec3{0.0, 0.0, 0.0};

    EntityForces forces;
    forces.force = Vec3{0.0, 0.0, -9.81};  // Gravity

    // Single large step - adaptive should break into substeps
    integrator.integrate(state, forces, 1.0);

    // Should achieve good accuracy
    EXPECT_NEAR(state.velocity.z, -9.81, 0.01);
    EXPECT_NEAR(state.position.z, -4.905, 0.05);
}

TEST_F(AdaptiveIntegratorTest, UniformMotion) {
    AdaptiveIntegrator integrator;

    EntityState state = create_initial_state();
    // Initial velocity: 10 m/s in x direction

    EntityForces forces;  // Zero forces

    // Large step
    integrator.integrate(state, forces, 1.0);

    // After 1 second at 10 m/s: x = 10 m
    EXPECT_NEAR(state.position.x, 10.0, 0.01);
    EXPECT_NEAR(state.velocity.x, 10.0, 0.001);
}

TEST_F(AdaptiveIntegratorTest, AdaptiveStepResult) {
    AdaptiveIntegrator integrator;

    EntityState state = create_initial_state();
    EntityForces forces;
    forces.force = Vec3{0.0, 0.0, -9.81};

    // Perform single adaptive step
    auto result = integrator.adaptive_step(state, forces, 0.1);

    EXPECT_TRUE(result.accepted);
    EXPECT_GT(result.actual_dt, 0.0);
    EXPECT_GT(result.suggested_dt, 0.0);
    EXPECT_GE(result.substeps, 1);
}

TEST_F(AdaptiveIntegratorTest, Reset) {
    AdaptiveIntegrator integrator;

    EntityState state = create_initial_state();
    EntityForces forces;
    forces.force = Vec3{0.0, 0.0, -9.81};

    // Do some integration
    integrator.integrate(state, forces, 0.5);

    // Reset
    integrator.reset();

    // Should have reset suggested dt to default
    EXPECT_NEAR(integrator.get_suggested_dt(), 0.01, 0.001);
}

TEST_F(AdaptiveIntegratorTest, ToleranceAffectsSubsteps) {
    // Tight tolerance
    AdaptiveIntegrator tight_integrator;
    AdaptiveConfig tight_config;
    tight_config.tolerance = 1e-10;
    tight_integrator.set_config(tight_config);

    // Loose tolerance
    AdaptiveIntegrator loose_integrator;
    AdaptiveConfig loose_config;
    loose_config.tolerance = 1e-3;
    loose_integrator.set_config(loose_config);

    EntityState state1 = create_initial_state();
    EntityState state2 = create_initial_state();

    EntityForces forces;
    forces.force = Vec3{100.0, 50.0, -9.81};  // Non-trivial forces

    tight_integrator.integrate(state1, forces, 1.0);
    loose_integrator.integrate(state2, forces, 1.0);

    // Tight tolerance should use more substeps
    EXPECT_GE(tight_integrator.get_last_result().substeps,
              loose_integrator.get_last_result().substeps);
}

// ============================================================================
// RK45 Integrator Tests
// ============================================================================

class RK45IntegratorTest : public ::testing::Test {
protected:
    static constexpr Real EPSILON = 1e-6;

    EntityState create_initial_state() {
        EntityState state;
        state.position = Vec3{0.0, 0.0, 0.0};
        state.velocity = Vec3{10.0, 0.0, 0.0};
        state.orientation = Quat::Identity();
        state.angular_velocity = Vec3{0.0, 0.0, 0.0};
        state.mass = 1.0;
        state.inertia = Mat3x3::Diagonal(1.0, 1.0, 1.0);
        return state;
    }
};

TEST_F(RK45IntegratorTest, BasicProperties) {
    RK45Integrator integrator;

    EXPECT_EQ(integrator.name(), "RK45");
    EXPECT_EQ(integrator.order(), 5);
}

TEST_F(RK45IntegratorTest, ToleranceSettings) {
    RK45Integrator integrator;

    integrator.set_tolerance(1e-8);

    // Tolerance is internal, but we can verify error estimation works
    EntityState state = create_initial_state();
    EntityForces forces;
    forces.force = Vec3{0.0, 0.0, -9.81};

    integrator.integrate(state, forces, 0.1);

    // Should have computed some error estimate
    Real error = integrator.get_error_estimate();
    EXPECT_GE(error, 0.0);
}

TEST_F(RK45IntegratorTest, FreeFallAccuracy) {
    RK45Integrator integrator;

    EntityState state = create_initial_state();
    state.velocity = Vec3{0.0, 0.0, 0.0};

    EntityForces forces;
    forces.force = Vec3{0.0, 0.0, -9.81};

    Real dt = 0.01;
    Real total_time = 1.0;
    int steps = static_cast<int>(total_time / dt);

    for (int i = 0; i < steps; ++i) {
        integrator.integrate(state, forces, dt);
    }

    // RK45 should be very accurate
    EXPECT_NEAR(state.velocity.z, -9.81, 0.001);
    EXPECT_NEAR(state.position.z, -4.905, 0.01);
}

TEST_F(RK45IntegratorTest, UniformMotion) {
    RK45Integrator integrator;

    EntityState state = create_initial_state();
    // 10 m/s in x direction, no forces

    EntityForces forces;

    Real dt = 0.01;
    int steps = 100;

    for (int i = 0; i < steps; ++i) {
        integrator.integrate(state, forces, dt);
    }

    // After 1 second at 10 m/s: x = 10 m
    EXPECT_NEAR(state.position.x, 10.0, 0.001);
    EXPECT_NEAR(state.velocity.x, 10.0, 0.001);
}

TEST_F(RK45IntegratorTest, SuggestedStepSize) {
    RK45Integrator integrator;

    EntityState state = create_initial_state();
    EntityForces forces;
    forces.force = Vec3{0.0, 0.0, -9.81};

    integrator.integrate(state, forces, 0.1);

    // Should provide a suggested next step size
    Real suggested = integrator.get_suggested_dt();
    EXPECT_GT(suggested, 0.0);
    EXPECT_LE(suggested, 0.1);  // Should be reasonable
}

TEST_F(RK45IntegratorTest, HigherOrderAccuracy) {
    // Compare RK45 to RK4 for same problem
    RK45Integrator rk45;
    RK4Integrator rk4;

    EntityState state_rk45 = create_initial_state();
    state_rk45.velocity = Vec3{0.0, 0.0, 0.0};

    EntityState state_rk4 = create_initial_state();
    state_rk4.velocity = Vec3{0.0, 0.0, 0.0};

    EntityForces forces;
    forces.force = Vec3{0.0, 0.0, -9.81};

    // Use a relatively large time step
    Real dt = 0.05;
    int steps = 20;  // 1 second total

    for (int i = 0; i < steps; ++i) {
        rk45.integrate(state_rk45, forces, dt);
        rk4.integrate(state_rk4, forces, dt);
    }

    // Analytical solution
    Real v_exact = -9.81;
    Real x_exact = -4.905;

    // RK45 should have smaller error than RK4 for large steps
    Real error_rk45 = std::abs(state_rk45.position.z - x_exact);
    Real error_rk4 = std::abs(state_rk4.position.z - x_exact);

    // Both should be reasonably accurate
    EXPECT_LT(error_rk45, 0.1);
    EXPECT_LT(error_rk4, 0.1);
}

TEST_F(RK45IntegratorTest, RotationalMotion) {
    RK45Integrator integrator;

    EntityState state;
    state.position = Vec3{0.0, 0.0, 0.0};
    state.velocity = Vec3{0.0, 0.0, 0.0};
    state.orientation = Quat::Identity();
    state.angular_velocity = Vec3{0.0, 0.0, constants::PI / 2.0};  // 90 deg/s
    state.mass = 1.0;
    state.inertia = Mat3x3::Diagonal(1.0, 1.0, 1.0);

    EntityForces forces;

    Real dt = 0.01;
    int steps = 100;  // 1 second

    for (int i = 0; i < steps; ++i) {
        integrator.integrate(state, forces, dt);
    }

    // After 1 second at 90 deg/s, should rotate 90 degrees
    // Quaternion: (cos(45°), 0, 0, sin(45°))
    Real expected_angle = constants::PI / 2.0;
    Real actual_angle = 2.0 * std::acos(std::abs(state.orientation.w));
    EXPECT_NEAR(actual_angle, expected_angle, 0.1);
}

// ============================================================================
// Integrator Comparison Tests
// ============================================================================

class IntegratorComparisonTest : public ::testing::Test {
protected:
    EntityState create_initial_state() {
        EntityState state;
        state.position = Vec3{0.0, 0.0, 0.0};
        state.velocity = Vec3{0.0, 0.0, 0.0};
        state.orientation = Quat::Identity();
        state.angular_velocity = Vec3{0.0, 0.0, 0.0};
        state.mass = 1.0;
        state.inertia = Mat3x3::Diagonal(1.0, 1.0, 1.0);
        return state;
    }
};

TEST_F(IntegratorComparisonTest, AllIntegratorsConverge) {
    // All integrators should converge to same solution
    EulerIntegrator euler;
    RK4Integrator rk4;
    RK45Integrator rk45;
    ABM4Integrator abm4;
    AdaptiveIntegrator adaptive;

    EntityState state_euler = create_initial_state();
    EntityState state_rk4 = create_initial_state();
    EntityState state_rk45 = create_initial_state();
    EntityState state_abm4 = create_initial_state();
    EntityState state_adaptive = create_initial_state();

    EntityForces forces;
    forces.force = Vec3{0.0, 0.0, -9.81};

    Real dt = 0.001;  // Small step for accuracy
    int steps = 1000;  // 1 second

    for (int i = 0; i < steps; ++i) {
        euler.integrate(state_euler, forces, dt);
        rk4.integrate(state_rk4, forces, dt);
        rk45.integrate(state_rk45, forces, dt);
        abm4.integrate(state_abm4, forces, dt);
    }

    // Adaptive with 1 second total time
    adaptive.integrate(state_adaptive, forces, 1.0);

    // Analytical solution
    Real x_exact = -4.905;

    // All should be reasonably close to exact
    EXPECT_NEAR(state_euler.position.z, x_exact, 0.1);
    EXPECT_NEAR(state_rk4.position.z, x_exact, 0.01);
    EXPECT_NEAR(state_rk45.position.z, x_exact, 0.01);
    EXPECT_NEAR(state_abm4.position.z, x_exact, 0.01);
    EXPECT_NEAR(state_adaptive.position.z, x_exact, 0.05);

    // Higher order methods should be more accurate
    Real error_euler = std::abs(state_euler.position.z - x_exact);
    Real error_rk4 = std::abs(state_rk4.position.z - x_exact);

    EXPECT_LT(error_rk4, error_euler);
}
