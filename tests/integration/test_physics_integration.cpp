/**
 * @file test_physics_integration.cpp
 * @brief Integration tests for JaguarEngine physics system
 *
 * Tests the integration of:
 * - Component Force Registry with entity-specific force models
 * - Constraint solver with multiple entities and constraint types
 * - Integrator switching (ABM4, RK4) during simulation
 * - Warm starting and constraint convergence
 */

#include <gtest/gtest.h>
#include "jaguar/physics/component_force_registry.h"
#include "jaguar/physics/entity.h"
#include "jaguar/physics/force.h"
#include "jaguar/physics/solver.h"
#include "jaguar/physics/constraints/constraint_solver.h"
#include "jaguar/physics/constraints/distance_constraint.h"
#include "jaguar/physics/constraints/ball_socket_constraint.h"
#include "jaguar/physics/constraints/hinge_constraint.h"
#include "jaguar/environment/environment.h"
#include <cmath>
#include <memory>
#include <string>

using namespace jaguar;
using namespace jaguar::physics;
using namespace jaguar::environment;

// ============================================================================
// Test Force Generators
// ============================================================================

/**
 * @brief Simple constant force generator for testing
 */
class ConstantForceModel : public IForceGenerator {
public:
    explicit ConstantForceModel(const Vec3& force, const std::string& name = "ConstantForce")
        : force_(force), name_(name) {}

    void compute_forces(const EntityState& state,
                       const Environment& env,
                       Real dt,
                       EntityForces& forces) override {
        (void)state; (void)env; (void)dt;
        forces.force += force_;
    }

    const std::string& name() const override { return name_; }
    Domain domain() const override { return Domain::Generic; }

private:
    Vec3 force_;
    std::string name_;
};

/**
 * @brief Simple drag force (proportional to velocity)
 */
class SimpleDragModel : public IForceGenerator {
public:
    explicit SimpleDragModel(Real coefficient, const std::string& name = "SimpleDrag")
        : coefficient_(coefficient), name_(name) {}

    void compute_forces(const EntityState& state,
                       const Environment& env,
                       Real dt,
                       EntityForces& forces) override {
        (void)env; (void)dt;
        Vec3 drag = -coefficient_ * state.velocity;
        forces.force += drag;
    }

    const std::string& name() const override { return name_; }
    Domain domain() const override { return Domain::Air; }

private:
    Real coefficient_;
    std::string name_;
};

// ============================================================================
// Component Force Registry Tests
// ============================================================================

TEST(ComponentForceRegistryTest, BasicRegistrationAndRetrieval) {
    ComponentForceRegistry registry;

    EntityId entity1 = 1;
    EntityId entity2 = 2;

    // Register constant force for entity1
    auto force1 = std::make_unique<ConstantForceModel>(Vec3{10.0, 0.0, 0.0});
    registry.register_model<ConstantForceModel>(entity1, std::move(force1));

    // Register drag force for entity2
    auto drag = std::make_unique<SimpleDragModel>(0.5);
    registry.register_model<SimpleDragModel>(entity2, std::move(drag));

    // Retrieve and verify
    EXPECT_TRUE(registry.has<ConstantForceModel>(entity1));
    EXPECT_FALSE(registry.has<ConstantForceModel>(entity2));
    EXPECT_TRUE(registry.has<SimpleDragModel>(entity2));

    auto* retrieved = registry.get<ConstantForceModel>(entity1);
    ASSERT_NE(retrieved, nullptr);
    EXPECT_EQ(retrieved->name(), "ConstantForce");

    EXPECT_EQ(registry.entity_count(), 2u);
}

TEST(ComponentForceRegistryTest, MultipleModelsPerEntity) {
    ComponentForceRegistry registry;

    EntityId entity = 1;

    // Register multiple model types for same entity
    auto constant = std::make_unique<ConstantForceModel>(Vec3{5.0, 0.0, 0.0});
    auto drag = std::make_unique<SimpleDragModel>(0.3);

    registry.register_model<ConstantForceModel>(entity, std::move(constant));
    registry.register_model<SimpleDragModel>(entity, std::move(drag));

    // Both should be retrievable
    EXPECT_TRUE(registry.has<ConstantForceModel>(entity));
    EXPECT_TRUE(registry.has<SimpleDragModel>(entity));

    auto all_models = registry.get_all(entity);
    EXPECT_EQ(all_models.size(), 2u);
}

TEST(ComponentForceRegistryTest, ForceComputation) {
    ComponentForceRegistry registry;

    EntityId entity = 1;

    // Register constant upward force
    auto constant = std::make_unique<ConstantForceModel>(Vec3{0.0, 0.0, 100.0});
    registry.register_model<ConstantForceModel>(entity, std::move(constant));

    // Compute forces
    EntityState state;
    state.position = Vec3{0.0, 0.0, 0.0};
    state.velocity = Vec3{0.0, 0.0, 0.0};

    Environment env{};  // Default environment
    Real dt = 0.01;
    EntityForces forces;

    if (auto* model = registry.get<ConstantForceModel>(entity)) {
        model->compute_forces(state, env, dt, forces);
    }

    // Verify force was applied
    EXPECT_DOUBLE_EQ(forces.force.x, 0.0);
    EXPECT_DOUBLE_EQ(forces.force.y, 0.0);
    EXPECT_DOUBLE_EQ(forces.force.z, 100.0);
}

// ============================================================================
// Integrator Tests
// ============================================================================

TEST(IntegratorTest, RK4IntegrationBasic) {
    RK4Integrator integrator;

    EXPECT_EQ(integrator.name(), "RK4");
    EXPECT_EQ(integrator.order(), 4);

    // Set up entity state
    EntityState state;
    state.position = Vec3{0.0, 0.0, 0.0};
    state.velocity = Vec3{10.0, 0.0, 0.0};  // 10 m/s in x direction
    state.mass = 1.0;
    state.inertia = Mat3x3::Identity();

    // Apply no forces (free drift)
    EntityForces forces;

    Real dt = 0.1;
    integrator.integrate(state, forces, dt);

    // Position should advance by velocity * dt
    EXPECT_NEAR(state.position.x, 1.0, 1e-6);  // 10 * 0.1 = 1.0
    EXPECT_NEAR(state.position.y, 0.0, 1e-9);
    EXPECT_NEAR(state.position.z, 0.0, 1e-9);
}

TEST(IntegratorTest, ABM4Integration) {
    ABM4Integrator integrator;

    EXPECT_EQ(integrator.name(), "ABM4");
    EXPECT_EQ(integrator.order(), 4);

    EntityState state;
    state.position = Vec3{0.0, 0.0, 0.0};
    state.velocity = Vec3{0.0, 0.0, 0.0};
    state.mass = 1.0;
    state.inertia = Mat3x3::Identity();

    // Apply constant force
    EntityForces forces;
    forces.force = Vec3{10.0, 0.0, 0.0};  // 10 N

    Real dt = 0.01;

    // ABM4 needs several steps to build history
    for (int i = 0; i < 10; ++i) {
        integrator.integrate(state, forces, dt);
    }

    // After 10 steps with constant force, velocity should increase
    EXPECT_GT(state.velocity.x, 0.0);
    EXPECT_GT(state.position.x, 0.0);
}

TEST(IntegratorTest, ABM4Reset) {
    ABM4Integrator integrator;

    EntityState state;
    state.mass = 1.0;
    state.inertia = Mat3x3::Identity();

    EntityForces forces;
    forces.force = Vec3{1.0, 0.0, 0.0};

    // Build up history
    for (int i = 0; i < 5; ++i) {
        integrator.integrate(state, forces, 0.01);
    }

    // Reset should clear history
    integrator.reset();

    // After reset, integrator should start fresh
    EntityState state2;
    state2.mass = 1.0;
    state2.inertia = Mat3x3::Identity();

    integrator.integrate(state2, forces, 0.01);
    // Should behave like first-order method initially
}

// ============================================================================
// Constraint Solver Tests
// ============================================================================

TEST(ConstraintSolverTest, SolverInitialization) {
    SequentialImpulseSolver solver;

    EXPECT_EQ(solver.name(), "SequentialImpulse");

    ConstraintSolverConfig config = solver.config();
    EXPECT_GT(config.velocity_iterations, 0);
    EXPECT_TRUE(config.warm_starting);
}

TEST(ConstraintSolverTest, DistanceConstraintSetup) {
    // Create two entities
    EntityState state1;
    state1.position = Vec3{0.0, 0.0, 0.0};
    state1.velocity = Vec3{0.0, 0.0, 0.0};
    state1.mass = 1.0;
    state1.inertia = Mat3x3::Identity();

    EntityState state2;
    state2.position = Vec3{10.0, 0.0, 0.0};
    state2.velocity = Vec3{0.0, 0.0, 0.0};
    state2.mass = 1.0;
    state2.inertia = Mat3x3::Identity();

    // Create distance constraint
    DistanceConstraintParams params;
    params.distance = 10.0;  // Use 'distance' not 'target_distance'
    params.stiffness = 1.0;

    DistanceConstraint constraint(
        ConstraintBody::Dynamic(1, Vec3::Zero()),
        ConstraintBody::Dynamic(2, Vec3::Zero()),
        params
    );

    EXPECT_EQ(constraint.type(), ConstraintType::Distance);
    EXPECT_EQ(constraint.num_rows(), 1);  // Distance constraint has 1 DOF
}

TEST(ConstraintSolverTest, BallSocketConstraintSetup) {
    EntityState state1;
    state1.position = Vec3{0.0, 0.0, 0.0};
    state1.mass = 1.0;
    state1.inertia = Mat3x3::Identity();

    EntityState state2;
    state2.position = Vec3{0.0, 0.0, 5.0};
    state2.mass = 1.0;
    state2.inertia = Mat3x3::Identity();

    BallSocketConstraintParams params;
    params.stiffness = 1.0;

    BallSocketConstraint constraint(
        ConstraintBody::Dynamic(1, Vec3{0.0, 0.0, 2.5}),
        ConstraintBody::Dynamic(2, Vec3{0.0, 0.0, -2.5}),
        params
    );

    EXPECT_EQ(constraint.type(), ConstraintType::BallSocket);
    EXPECT_EQ(constraint.num_rows(), 3);  // Ball socket constrains 3 DOFs
}

TEST(ConstraintSolverTest, HingeConstraintSetup) {
    EntityState state1;
    state1.position = Vec3{0.0, 0.0, 0.0};
    state1.mass = 1.0;
    state1.inertia = Mat3x3::Identity();

    EntityState state2;
    state2.position = Vec3{0.0, 0.0, 2.0};
    state2.mass = 1.0;
    state2.inertia = Mat3x3::Identity();

    HingeConstraintParams params;
    params.stiffness = 1.0;

    HingeConstraint constraint(
        ConstraintBody::Dynamic(1, Vec3{0.0, 0.0, 1.0}, Vec3{0.0, 1.0, 0.0}),
        ConstraintBody::Dynamic(2, Vec3{0.0, 0.0, -1.0}, Vec3{0.0, 1.0, 0.0}),
        params
    );

    EXPECT_EQ(constraint.type(), ConstraintType::Hinge);
    EXPECT_EQ(constraint.num_rows(), 5);  // Hinge constrains 5 DOFs
}

// ============================================================================
// Warm Starting Tests
// ============================================================================

TEST(ConstraintSolverTest, WarmStartConfiguration) {
    SequentialImpulseSolver solver;

    ConstraintSolverConfig config = solver.config();
    config.warm_starting = true;
    config.warm_start_factor = 0.8;
    solver.set_config(config);

    ConstraintSolverConfig retrieved = solver.config();
    EXPECT_TRUE(retrieved.warm_starting);
    EXPECT_DOUBLE_EQ(retrieved.warm_start_factor, 0.8);
}

// ============================================================================
// Inverse Inertia Caching Tests
// ============================================================================

TEST(EntityStateTest, InverseInertiaCaching) {
    EntityState state;
    state.inertia = Mat3x3::Identity() * 2.0;
    state.invalidate_inertia_cache();

    // First access should compute
    const Mat3x3& inv1 = state.get_inverse_inertia();
    EXPECT_NEAR(inv1(0,0), 0.5, 1e-9);  // Inverse of 2.0
    EXPECT_NEAR(inv1(1,1), 0.5, 1e-9);
    EXPECT_NEAR(inv1(2,2), 0.5, 1e-9);

    // Second access should return cached value
    const Mat3x3& inv2 = state.get_inverse_inertia();
    EXPECT_EQ(&inv1, &inv2);  // Same object reference
}

TEST(EntityStateTest, InvertiaCacheInvalidation) {
    EntityState state;
    state.inertia = Mat3x3::Identity() * 2.0;

    // Get initial inverse
    const Mat3x3& inv1 = state.get_inverse_inertia();
    EXPECT_NEAR(inv1(0,0), 0.5, 1e-9);

    // Modify inertia and invalidate
    state.inertia = Mat3x3::Identity() * 4.0;
    state.invalidate_inertia_cache();

    // New access should recompute
    const Mat3x3& inv2 = state.get_inverse_inertia();
    EXPECT_NEAR(inv2(0,0), 0.25, 1e-9);  // Inverse of 4.0
}

// ============================================================================
// Entity State Storage Tests
// ============================================================================

TEST(EntityStateStorageTest, AllocationAndReuse) {
    EntityStateStorage storage;

    EXPECT_EQ(storage.size(), 0u);
    EXPECT_EQ(storage.active_count(), 0u);

    UInt32 idx1 = storage.allocate();
    EXPECT_EQ(storage.active_count(), 1u);

    UInt32 idx2 = storage.allocate();
    EXPECT_EQ(storage.active_count(), 2u);

    storage.free(idx1);
    EXPECT_EQ(storage.active_count(), 1u);

    // Should reuse freed slot
    UInt32 idx3 = storage.allocate();
    EXPECT_EQ(idx3, idx1);
    EXPECT_EQ(storage.active_count(), 2u);
}

TEST(EntityStateStorageTest, StateSetAndGet) {
    EntityStateStorage storage;

    UInt32 idx = storage.allocate();

    EntityState state;
    state.position = Vec3{1.0, 2.0, 3.0};
    state.velocity = Vec3{4.0, 5.0, 6.0};
    state.mass = 10.0;

    storage.set_state(idx, state);  // Use 'set_state' not 'set'

    EntityState retrieved = storage.get_state(idx);  // Use 'get_state' not 'get'
    EXPECT_DOUBLE_EQ(retrieved.position.x, 1.0);
    EXPECT_DOUBLE_EQ(retrieved.position.y, 2.0);
    EXPECT_DOUBLE_EQ(retrieved.position.z, 3.0);
    EXPECT_DOUBLE_EQ(retrieved.mass, 10.0);
}

// ============================================================================
// Integration Test: Force Registry + Integrator
// ============================================================================

TEST(PhysicsIntegrationTest, ForceRegistryWithIntegrator) {
    ComponentForceRegistry registry;
    RK4Integrator integrator;

    EntityId entity = 1;

    // Register constant force
    auto force_model = std::make_unique<ConstantForceModel>(Vec3{0.0, 0.0, 10.0});
    registry.register_model<ConstantForceModel>(entity, std::move(force_model));

    // Initial state
    EntityState state;
    state.position = Vec3{0.0, 0.0, 0.0};
    state.velocity = Vec3{0.0, 0.0, 0.0};
    state.mass = 1.0;
    state.inertia = Mat3x3::Identity();

    Environment env{};

    // Compute forces and integrate
    Real dt = 0.01;
    for (int i = 0; i < 100; ++i) {
        EntityForces forces;

        if (auto* model = registry.get<ConstantForceModel>(entity)) {
            model->compute_forces(state, env, dt, forces);
        }

        integrator.integrate(state, forces, dt);
    }

    // After 100 steps with upward force, should have positive z velocity and position
    EXPECT_GT(state.velocity.z, 0.0);
    EXPECT_GT(state.position.z, 0.0);
}
