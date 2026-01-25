/**
 * @file test_component_force_integration.cpp
 * @brief Tests for entity-specific force computation via ComponentForceRegistry
 */

#include <gtest/gtest.h>
#include "jaguar/interface/api.h"
#include "jaguar/physics/component_force_registry.h"
#include "jaguar/physics/force.h"
#include "jaguar/environment/environment.h"

using namespace jaguar;
using namespace jaguar::physics;
using namespace jaguar::environment;

// ============================================================================
// Test Force Model
// ============================================================================

/**
 * @brief Simple test force generator that applies a constant force
 */
class ConstantForceGenerator : public IForceGenerator {
public:
    ConstantForceGenerator(const Vec3& force_value, const std::string& name = "ConstantForce")
        : force_value_(force_value), name_(name) {}

    void compute_forces(const EntityState& state,
                       const Environment& env,
                       Real dt,
                       EntityForces& forces) override {
        (void)state;
        (void)env;
        (void)dt;
        forces.force += force_value_;
    }

    const std::string& name() const override { return name_; }
    Domain domain() const override { return Domain::Generic; }
    bool is_enabled() const override { return enabled_; }
    void set_enabled(bool enabled) override { enabled_ = enabled; }

private:
    Vec3 force_value_;
    std::string name_;
    bool enabled_ = true;
};

// ============================================================================
// Integration Tests
// ============================================================================

class ComponentForceIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        engine_ = std::make_unique<Engine>();
        ASSERT_TRUE(engine_->initialize());
    }

    void TearDown() override {
        engine_->shutdown();
    }

    std::unique_ptr<Engine> engine_;
};

TEST_F(ComponentForceIntegrationTest, EntityWithoutComponentUsesGlobalForces) {
    // Create an entity in Air domain
    EntityId entity = engine_->create_entity("test_aircraft", Domain::Air);
    ASSERT_NE(entity, INVALID_ENTITY_ID);

    // Set initial state
    EntityState state;
    state.position = Vec3{6378137.0, 0.0, 0.0};  // On equator at sea level
    state.velocity = Vec3{0.0, 0.0, 0.0};
    state.mass = 1000.0;
    engine_->set_entity_state(entity, state);

    // Step simulation - should use global gravity
    engine_->step(0.01);

    // Get updated state
    EntityState new_state = engine_->get_entity_state(entity);

    // Velocity should have changed due to gravity (pointing inward toward Earth center)
    Real vel_magnitude = std::sqrt(new_state.velocity.x * new_state.velocity.x +
                                   new_state.velocity.y * new_state.velocity.y +
                                   new_state.velocity.z * new_state.velocity.z);
    EXPECT_GT(vel_magnitude, 0.0);
}

TEST_F(ComponentForceIntegrationTest, EntityWithComponentUsesEntitySpecificForce) {
    // Create two entities
    EntityId entity1 = engine_->create_entity("entity1", Domain::Air);
    EntityId entity2 = engine_->create_entity("entity2", Domain::Air);
    ASSERT_NE(entity1, INVALID_ENTITY_ID);
    ASSERT_NE(entity2, INVALID_ENTITY_ID);

    // Set initial states at same position
    EntityState state;
    state.position = Vec3{6378137.0, 0.0, 0.0};
    state.velocity = Vec3{0.0, 0.0, 0.0};
    state.mass = 1000.0;
    engine_->set_entity_state(entity1, state);
    engine_->set_entity_state(entity2, state);

    // Register entity-specific force for entity1 (upward force)
    auto& registry = engine_->get_component_force_registry();
    auto upward_force = std::make_unique<ConstantForceGenerator>(
        Vec3{10000.0, 0.0, 0.0}, "UpwardForce");
    registry.register_model<ConstantForceGenerator>(entity1, std::move(upward_force));

    // Step simulation
    engine_->step(0.01);

    // Get updated states
    EntityState state1 = engine_->get_entity_state(entity1);
    EntityState state2 = engine_->get_entity_state(entity2);

    // Entity1 should have upward velocity (custom force)
    // Entity2 should have inward velocity (global gravity)
    // They should be different
    EXPECT_NE(state1.velocity.x, state2.velocity.x);
}

TEST_F(ComponentForceIntegrationTest, MultipleComponentsPerEntity) {
    EntityId entity = engine_->create_entity("multi_force_entity", Domain::Air);
    ASSERT_NE(entity, INVALID_ENTITY_ID);

    // Set initial state
    EntityState state;
    state.position = Vec3{6378137.0, 0.0, 0.0};
    state.velocity = Vec3{0.0, 0.0, 0.0};
    state.mass = 1000.0;
    engine_->set_entity_state(entity, state);

    // Register multiple force models
    auto& registry = engine_->get_component_force_registry();

    // Create a custom force type to allow multiple registrations
    class ForceX : public ConstantForceGenerator {
    public:
        using ConstantForceGenerator::ConstantForceGenerator;
    };

    class ForceY : public ConstantForceGenerator {
    public:
        using ConstantForceGenerator::ConstantForceGenerator;
    };

    auto force_x = std::make_unique<ForceX>(Vec3{1000.0, 0.0, 0.0}, "ForceX");
    auto force_y = std::make_unique<ForceY>(Vec3{0.0, 1000.0, 0.0}, "ForceY");

    registry.register_model<ForceX>(entity, std::move(force_x));
    registry.register_model<ForceY>(entity, std::move(force_y));

    // Verify both models are registered
    auto all_models = registry.get_all(entity);
    EXPECT_EQ(all_models.size(), 2u);

    // Step simulation
    engine_->step(0.01);

    // Get updated state
    EntityState new_state = engine_->get_entity_state(entity);

    // Both forces should have been applied
    EXPECT_NE(new_state.velocity.x, 0.0);
    EXPECT_NE(new_state.velocity.y, 0.0);
}

TEST_F(ComponentForceIntegrationTest, RemoveComponentFallsBackToGlobal) {
    EntityId entity = engine_->create_entity("test_entity", Domain::Air);
    ASSERT_NE(entity, INVALID_ENTITY_ID);

    // Set initial state
    EntityState state;
    state.position = Vec3{6378137.0, 0.0, 0.0};
    state.velocity = Vec3{0.0, 0.0, 0.0};
    state.mass = 1000.0;
    engine_->set_entity_state(entity, state);

    // Register entity-specific force
    auto& registry = engine_->get_component_force_registry();
    auto custom_force = std::make_unique<ConstantForceGenerator>(
        Vec3{5000.0, 0.0, 0.0}, "CustomForce");
    registry.register_model<ConstantForceGenerator>(entity, std::move(custom_force));

    // Step with custom force
    engine_->step(0.01);
    EntityState state_with_custom = engine_->get_entity_state(entity);

    // Reset state
    state.velocity = Vec3{0.0, 0.0, 0.0};
    engine_->set_entity_state(entity, state);

    // Remove custom force
    registry.remove_model<ConstantForceGenerator>(entity);

    // Step without custom force (should use global)
    engine_->step(0.01);
    EntityState state_without_custom = engine_->get_entity_state(entity);

    // Velocities should be different
    EXPECT_NE(state_with_custom.velocity.x, state_without_custom.velocity.x);
}

TEST_F(ComponentForceIntegrationTest, ComponentForceRegistryAccessible) {
    // Verify we can access the registry from the engine
    auto& registry = engine_->get_component_force_registry();

    // Initially should be empty
    EXPECT_EQ(registry.entity_count(), 0u);
    EXPECT_EQ(registry.model_count(), 0u);

    // Create entity and register a model
    EntityId entity = engine_->create_entity("test", Domain::Air);
    auto force = std::make_unique<ConstantForceGenerator>(Vec3{1.0, 2.0, 3.0});
    registry.register_model<ConstantForceGenerator>(entity, std::move(force));

    // Should now have one entity and one model
    EXPECT_EQ(registry.entity_count(), 1u);
    EXPECT_EQ(registry.model_count(), 1u);
}
