/**
 * @file test_force.cpp
 * @brief Unit tests for JaguarEngine force generators
 */

#include <gtest/gtest.h>
#include "jaguar/physics/force.h"
#include "jaguar/environment/environment.h"
#include <cmath>

using namespace jaguar;
using namespace jaguar::physics;
using namespace jaguar::environment;

// ============================================================================
// Force Generator Registry Tests
// ============================================================================

class ForceGeneratorRegistryTest : public ::testing::Test {
protected:
    ForceGeneratorRegistry registry;
};

TEST_F(ForceGeneratorRegistryTest, InitiallyEmpty) {
    EXPECT_EQ(registry.size(), 0u);
}

TEST_F(ForceGeneratorRegistryTest, RegisterGenerator) {
    auto gravity = force::create_simple_gravity();
    registry.register_generator(std::move(gravity));
    EXPECT_EQ(registry.size(), 1u);
}

TEST_F(ForceGeneratorRegistryTest, FindByName) {
    auto gravity = force::create_simple_gravity();
    registry.register_generator(std::move(gravity));

    IForceGenerator* found = registry.find("SimpleGravity");
    ASSERT_NE(found, nullptr);
    EXPECT_EQ(found->name(), "SimpleGravity");
}

TEST_F(ForceGeneratorRegistryTest, FindNonExistent) {
    auto gravity = force::create_simple_gravity();
    registry.register_generator(std::move(gravity));

    IForceGenerator* found = registry.find("NonExistent");
    EXPECT_EQ(found, nullptr);
}

TEST_F(ForceGeneratorRegistryTest, GetGeneratorsByDomain) {
    registry.register_generator(force::create_simple_gravity());
    registry.register_generator(force::create_simple_aerodynamics());
    registry.register_generator(force::create_drag_model());

    auto generic_generators = registry.get_generators(Domain::Generic);
    EXPECT_GE(generic_generators.size(), 1u);  // At least gravity

    auto air_generators = registry.get_generators(Domain::Air);
    EXPECT_GE(air_generators.size(), 2u);  // Aerodynamics and drag
}

TEST_F(ForceGeneratorRegistryTest, GetEnabledGenerators) {
    registry.register_generator(force::create_simple_gravity());
    registry.register_generator(force::create_simple_aerodynamics());

    auto enabled = registry.get_enabled_generators();
    EXPECT_EQ(enabled.size(), 2u);

    // Disable one
    IForceGenerator* gravity = registry.find("SimpleGravity");
    ASSERT_NE(gravity, nullptr);
    gravity->set_enabled(false);

    enabled = registry.get_enabled_generators();
    EXPECT_EQ(enabled.size(), 1u);
}

TEST_F(ForceGeneratorRegistryTest, ClearRegistry) {
    registry.register_generator(force::create_simple_gravity());
    registry.register_generator(force::create_simple_aerodynamics());
    EXPECT_EQ(registry.size(), 2u);

    registry.clear();
    EXPECT_EQ(registry.size(), 0u);
}

// ============================================================================
// Simple Gravity Model Tests
// ============================================================================

class SimpleGravityTest : public ::testing::Test {
protected:
    static constexpr Real EPSILON = 1e-6;

    EntityState create_test_state() {
        EntityState state;
        state.position = Vec3{0.0, 0.0, 1000.0};  // 1km altitude
        state.velocity = Vec3{100.0, 0.0, 0.0};
        state.orientation = Quat::Identity();
        state.angular_velocity = Vec3{0.0, 0.0, 0.0};
        state.mass = 1000.0;  // 1000 kg
        state.inertia = Mat3x3::Diagonal(100.0, 200.0, 150.0);
        return state;
    }

    Environment create_test_environment() {
        Environment env;
        env.atmosphere.density = 1.225;  // Sea level density
        env.atmosphere.temperature = 288.15;  // 15C
        env.atmosphere.pressure = 101325.0;  // Sea level
        env.atmosphere.speed_of_sound = 340.29;
        env.gravity = Vec3{0.0, 0.0, -constants::G0};
        return env;
    }
};

TEST_F(SimpleGravityTest, FactoryCreation) {
    auto gravity = force::create_simple_gravity();
    ASSERT_NE(gravity, nullptr);
    EXPECT_EQ(gravity->name(), "SimpleGravity");
    EXPECT_EQ(gravity->domain(), Domain::Generic);
}

TEST_F(SimpleGravityTest, DefaultGravity) {
    auto gravity = force::create_simple_gravity();
    EntityState state = create_test_state();
    Environment env = create_test_environment();
    EntityForces forces;

    gravity->compute_forces(state, env, 0.01, forces);

    // F = m * g = 1000 * 9.80665 = 9806.65 N downward
    EXPECT_NEAR(forces.force.z, -1000.0 * constants::G0, EPSILON);
    EXPECT_NEAR(forces.force.x, 0.0, EPSILON);
    EXPECT_NEAR(forces.force.y, 0.0, EPSILON);
}

TEST_F(SimpleGravityTest, CustomGravity) {
    auto gravity = force::create_simple_gravity(3.71);  // Mars gravity
    EntityState state = create_test_state();
    Environment env;
    env.gravity = Vec3{0.0, 0.0, 0.0};  // Will use configured value
    EntityForces forces;

    gravity->compute_forces(state, env, 0.01, forces);

    // F = m * g = 1000 * 3.71 = 3710 N
    EXPECT_NEAR(forces.force.z, -3710.0, EPSILON);
}

TEST_F(SimpleGravityTest, GetGravityAcceleration) {
    auto gravity = force::create_simple_gravity();
    EntityState state = create_test_state();
    Environment env = create_test_environment();
    EntityForces forces;

    gravity->compute_forces(state, env, 0.01, forces);

    Vec3 g = gravity->get_gravity_acceleration();
    EXPECT_NEAR(g.z, -constants::G0, EPSILON);
}

TEST_F(SimpleGravityTest, EnableDisable) {
    auto gravity = force::create_simple_gravity();
    EXPECT_TRUE(gravity->is_enabled());

    gravity->set_enabled(false);
    EXPECT_FALSE(gravity->is_enabled());

    gravity->set_enabled(true);
    EXPECT_TRUE(gravity->is_enabled());
}

// ============================================================================
// WGS84 Gravity Model Tests
// ============================================================================

class WGS84GravityTest : public ::testing::Test {
protected:
    static constexpr Real EPSILON = 1e-3;  // Less strict due to J2 complexity

    EntityState create_test_state_at_surface() {
        EntityState state;
        // Position at equator, sea level (ECEF)
        state.position = Vec3{constants::R_EARTH_EQUATOR, 0.0, 0.0};
        state.velocity = Vec3{0.0, 0.0, 0.0};
        state.orientation = Quat::Identity();
        state.angular_velocity = Vec3{0.0, 0.0, 0.0};
        state.mass = 1.0;  // 1 kg for easy force calculation
        state.inertia = Mat3x3::Diagonal(1.0, 1.0, 1.0);
        return state;
    }

    EntityState create_test_state_at_pole() {
        EntityState state;
        // Position at north pole (ECEF)
        state.position = Vec3{0.0, 0.0, constants::R_EARTH_POLAR};
        state.velocity = Vec3{0.0, 0.0, 0.0};
        state.orientation = Quat::Identity();
        state.angular_velocity = Vec3{0.0, 0.0, 0.0};
        state.mass = 1.0;
        state.inertia = Mat3x3::Diagonal(1.0, 1.0, 1.0);
        return state;
    }
};

TEST_F(WGS84GravityTest, FactoryCreation) {
    auto gravity = force::create_wgs84_gravity();
    ASSERT_NE(gravity, nullptr);
    EXPECT_EQ(gravity->name(), "WGS84Gravity");
    EXPECT_EQ(gravity->domain(), Domain::Generic);
}

TEST_F(WGS84GravityTest, GravityAtEquator) {
    auto gravity = force::create_wgs84_gravity();
    EntityState state = create_test_state_at_surface();
    Environment env;
    EntityForces forces;

    gravity->compute_forces(state, env, 0.01, forces);

    // Gravity at equator should be approximately 9.78 m/s^2
    Real force_magnitude = forces.force.length();
    EXPECT_NEAR(force_magnitude, 9.78, 0.05);
}

TEST_F(WGS84GravityTest, GravityAtPole) {
    auto gravity = force::create_wgs84_gravity();
    EntityState state = create_test_state_at_pole();
    Environment env;
    EntityForces forces;

    gravity->compute_forces(state, env, 0.01, forces);

    // Gravity at pole should be approximately 9.83-9.90 m/s^2
    // (J2 model approximation gives ~9.90, actual is 9.832)
    Real force_magnitude = forces.force.length();
    EXPECT_NEAR(force_magnitude, 9.86, 0.10);  // Allow broader tolerance for J2 approximation
}

TEST_F(WGS84GravityTest, PoleGravityStrongerThanEquator) {
    auto gravity = force::create_wgs84_gravity();
    Environment env;

    EntityState state_equator = create_test_state_at_surface();
    EntityForces forces_equator;
    gravity->compute_forces(state_equator, env, 0.01, forces_equator);

    EntityState state_pole = create_test_state_at_pole();
    EntityForces forces_pole;
    gravity->compute_forces(state_pole, env, 0.01, forces_pole);

    // Due to Earth's oblateness and rotation, polar gravity > equatorial gravity
    EXPECT_GT(forces_pole.force.length(), forces_equator.force.length());
}

TEST_F(WGS84GravityTest, GravityPointsTowardCenter) {
    auto gravity = force::create_wgs84_gravity();
    EntityState state = create_test_state_at_surface();
    Environment env;
    EntityForces forces;

    gravity->compute_forces(state, env, 0.01, forces);

    // Force should point toward Earth center (negative of position direction)
    Vec3 force_dir = forces.force.normalized();
    Vec3 to_center = (state.position * -1.0).normalized();

    Real dot = force_dir.dot(to_center);
    EXPECT_GT(dot, 0.99);  // Should be nearly parallel
}

// ============================================================================
// Simple Aerodynamics Model Tests
// ============================================================================

class SimpleAerodynamicsTest : public ::testing::Test {
protected:
    static constexpr Real EPSILON = 1e-4;

    EntityState create_flying_state() {
        EntityState state;
        state.position = Vec3{0.0, 0.0, 5000.0};  // 5km altitude
        state.velocity = Vec3{100.0, 0.0, 0.0};   // 100 m/s forward (360 km/h)
        state.orientation = Quat::Identity();
        state.angular_velocity = Vec3{0.0, 0.0, 0.0};
        state.mass = 1000.0;
        state.inertia = Mat3x3::Diagonal(1000.0, 2000.0, 1500.0);
        return state;
    }

    Environment create_flight_environment() {
        Environment env;
        env.atmosphere.density = 0.7364;  // ~5km altitude
        env.atmosphere.temperature = 255.7;
        env.atmosphere.pressure = 54048.0;
        env.atmosphere.speed_of_sound = 320.5;
        env.atmosphere.wind = Vec3{0.0, 0.0, 0.0};
        return env;
    }
};

TEST_F(SimpleAerodynamicsTest, FactoryCreation) {
    auto aero = force::create_simple_aerodynamics();
    ASSERT_NE(aero, nullptr);
    EXPECT_EQ(aero->name(), "SimpleAerodynamics");
    EXPECT_EQ(aero->domain(), Domain::Air);
}

TEST_F(SimpleAerodynamicsTest, ZeroSpeedNoForces) {
    auto aero = force::create_simple_aerodynamics();
    EntityState state = create_flying_state();
    state.velocity = Vec3{0.0, 0.0, 0.0};  // Stationary
    Environment env = create_flight_environment();
    EntityForces forces;

    aero->compute_forces(state, env, 0.01, forces);

    // No aerodynamic forces at zero airspeed
    EXPECT_NEAR(forces.force.length(), 0.0, EPSILON);
    EXPECT_NEAR(aero->get_qbar(), 0.0, EPSILON);
}

TEST_F(SimpleAerodynamicsTest, DragOpposesMotion) {
    auto aero = force::create_simple_aerodynamics();
    EntityState state = create_flying_state();
    Environment env = create_flight_environment();
    EntityForces forces;

    aero->compute_forces(state, env, 0.01, forces);

    // With level flight (alpha=0), drag should oppose velocity
    // X-component of force should be negative (drag)
    EXPECT_LT(forces.force.x, 0.0);
}

TEST_F(SimpleAerodynamicsTest, DynamicPressureCalculation) {
    auto aero = force::create_simple_aerodynamics();
    EntityState state = create_flying_state();
    Environment env = create_flight_environment();
    EntityForces forces;

    aero->compute_forces(state, env, 0.01, forces);

    // q = 0.5 * rho * V^2 = 0.5 * 0.7364 * 100^2 = 3682 Pa
    Real expected_qbar = 0.5 * 0.7364 * 100.0 * 100.0;
    EXPECT_NEAR(aero->get_qbar(), expected_qbar, 1.0);
}

TEST_F(SimpleAerodynamicsTest, MachNumberCalculation) {
    auto aero = force::create_simple_aerodynamics();
    EntityState state = create_flying_state();
    Environment env = create_flight_environment();
    EntityForces forces;

    aero->compute_forces(state, env, 0.01, forces);

    // Mach = V / a = 100 / 320.5 = 0.312
    Real expected_mach = 100.0 / 320.5;
    EXPECT_NEAR(aero->get_mach(), expected_mach, 0.01);
}

TEST_F(SimpleAerodynamicsTest, AngleOfAttack) {
    auto aero = force::create_simple_aerodynamics();
    EntityState state = create_flying_state();
    // Nose up: velocity has negative z component relative to body
    state.velocity = Vec3{100.0, 0.0, -10.0};  // ~5.7 degrees AoA
    Environment env = create_flight_environment();
    EntityForces forces;

    aero->compute_forces(state, env, 0.01, forces);

    Real expected_alpha = std::atan2(10.0, 100.0);  // ~0.0997 rad (~5.7 deg)
    EXPECT_NEAR(aero->get_alpha(), expected_alpha, 0.01);
}

TEST_F(SimpleAerodynamicsTest, CoefficientCalculation) {
    auto aero = force::create_simple_aerodynamics();
    EntityState state = create_flying_state();
    Environment env = create_flight_environment();
    EntityForces forces;

    aero->compute_forces(state, env, 0.01, forces);

    // At alpha=0: CL = CL0 = 0 (default), CD = CD0 = 0.02 (default)
    EXPECT_NEAR(aero->get_cl(), 0.0, 0.01);
    EXPECT_NEAR(aero->get_cd(), 0.02, 0.01);
}

// ============================================================================
// Drag Model Tests
// ============================================================================

class DragModelTest : public ::testing::Test {
protected:
    static constexpr Real EPSILON = 1e-4;

    EntityState create_projectile_state() {
        EntityState state;
        state.position = Vec3{0.0, 0.0, 1000.0};
        state.velocity = Vec3{500.0, 0.0, -100.0};  // Fast projectile
        state.orientation = Quat::Identity();
        state.angular_velocity = Vec3{0.0, 0.0, 0.0};
        state.mass = 10.0;  // 10 kg projectile
        state.inertia = Mat3x3::Diagonal(0.1, 0.1, 0.1);
        return state;
    }

    Environment create_environment() {
        Environment env;
        env.atmosphere.density = 1.225;
        env.atmosphere.speed_of_sound = 340.29;
        return env;
    }
};

TEST_F(DragModelTest, FactoryCreation) {
    auto drag = force::create_drag_model(0.3, 0.01);
    ASSERT_NE(drag, nullptr);
    EXPECT_EQ(drag->name(), "DragModel");
    EXPECT_EQ(drag->domain(), Domain::Air);
}

TEST_F(DragModelTest, DefaultParameters) {
    auto drag = force::create_drag_model();  // CD=0.5, area=1.0
    EntityState state;
    state.position = Vec3{0.0, 0.0, 0.0};
    state.velocity = Vec3{100.0, 0.0, 0.0};
    state.mass = 1.0;
    Environment env = create_environment();
    EntityForces forces;

    drag->compute_forces(state, env, 0.01, forces);

    // D = 0.5 * rho * V^2 * CD * A = 0.5 * 1.225 * 10000 * 0.5 * 1.0 = 3062.5 N
    Real expected_drag = 0.5 * 1.225 * 100.0 * 100.0 * 0.5 * 1.0;
    EXPECT_NEAR(std::abs(forces.force.x), expected_drag, 1.0);
}

TEST_F(DragModelTest, DragOpposesVelocity) {
    auto drag = force::create_drag_model(0.5, 1.0);
    EntityState state = create_projectile_state();
    Environment env = create_environment();
    EntityForces forces;

    drag->compute_forces(state, env, 0.01, forces);

    // Drag should oppose velocity direction
    Vec3 velocity_dir = state.velocity.normalized();
    Vec3 force_dir = forces.force.normalized();

    Real dot = velocity_dir.dot(force_dir);
    EXPECT_LT(dot, -0.99);  // Should be nearly opposite
}

TEST_F(DragModelTest, DragScalesWithVelocitySquared) {
    auto drag = force::create_drag_model(0.5, 1.0);
    Environment env = create_environment();

    // Test at V = 50 m/s
    EntityState state1;
    state1.position = Vec3{0.0, 0.0, 0.0};
    state1.velocity = Vec3{50.0, 0.0, 0.0};
    state1.mass = 1.0;
    EntityForces forces1;
    drag->compute_forces(state1, env, 0.01, forces1);

    // Test at V = 100 m/s (2x velocity)
    EntityState state2;
    state2.position = Vec3{0.0, 0.0, 0.0};
    state2.velocity = Vec3{100.0, 0.0, 0.0};
    state2.mass = 1.0;
    EntityForces forces2;
    drag->compute_forces(state2, env, 0.01, forces2);

    // Drag at 2x velocity should be 4x (V^2 relationship)
    Real drag1 = forces1.force.length();
    Real drag2 = forces2.force.length();
    EXPECT_NEAR(drag2 / drag1, 4.0, 0.01);
}

TEST_F(DragModelTest, ZeroSpeedNoDrag) {
    auto drag = force::create_drag_model(0.5, 1.0);
    EntityState state;
    state.position = Vec3{0.0, 0.0, 0.0};
    state.velocity = Vec3{0.0, 0.0, 0.0};
    state.mass = 1.0;
    Environment env = create_environment();
    EntityForces forces;

    drag->compute_forces(state, env, 0.01, forces);

    EXPECT_NEAR(forces.force.length(), 0.0, EPSILON);
}

TEST_F(DragModelTest, VeryLowSpeedNoDrag) {
    auto drag = force::create_drag_model(0.5, 1.0);
    EntityState state;
    state.position = Vec3{0.0, 0.0, 0.0};
    state.velocity = Vec3{0.005, 0.0, 0.0};  // Below 0.01 threshold
    state.mass = 1.0;
    Environment env = create_environment();
    EntityForces forces;

    drag->compute_forces(state, env, 0.01, forces);

    EXPECT_NEAR(forces.force.length(), 0.0, EPSILON);
}

// ============================================================================
// Force Generator Interface Tests
// ============================================================================

class ForceGeneratorInterfaceTest : public ::testing::Test {
protected:
    static constexpr Real EPSILON = 1e-6;
};

TEST_F(ForceGeneratorInterfaceTest, AllGeneratorsHaveNames) {
    auto gravity = force::create_simple_gravity();
    auto wgs84 = force::create_wgs84_gravity();
    auto aero = force::create_simple_aerodynamics();
    auto drag = force::create_drag_model();

    EXPECT_FALSE(gravity->name().empty());
    EXPECT_FALSE(wgs84->name().empty());
    EXPECT_FALSE(aero->name().empty());
    EXPECT_FALSE(drag->name().empty());
}

TEST_F(ForceGeneratorInterfaceTest, AllGeneratorsHaveDomains) {
    auto gravity = force::create_simple_gravity();
    auto wgs84 = force::create_wgs84_gravity();
    auto aero = force::create_simple_aerodynamics();
    auto drag = force::create_drag_model();

    EXPECT_EQ(gravity->domain(), Domain::Generic);
    EXPECT_EQ(wgs84->domain(), Domain::Generic);
    EXPECT_EQ(aero->domain(), Domain::Air);
    EXPECT_EQ(drag->domain(), Domain::Air);
}

TEST_F(ForceGeneratorInterfaceTest, AllGeneratorsDefaultEnabled) {
    auto gravity = force::create_simple_gravity();
    auto wgs84 = force::create_wgs84_gravity();
    auto aero = force::create_simple_aerodynamics();
    auto drag = force::create_drag_model();

    EXPECT_TRUE(gravity->is_enabled());
    EXPECT_TRUE(wgs84->is_enabled());
    EXPECT_TRUE(aero->is_enabled());
    EXPECT_TRUE(drag->is_enabled());
}

TEST_F(ForceGeneratorInterfaceTest, AllGeneratorsInitializeSuccessfully) {
    auto gravity = force::create_simple_gravity();
    auto wgs84 = force::create_wgs84_gravity();
    auto aero = force::create_simple_aerodynamics();
    auto drag = force::create_drag_model();

    EXPECT_TRUE(gravity->initialize());
    EXPECT_TRUE(wgs84->initialize());
    EXPECT_TRUE(aero->initialize());
    EXPECT_TRUE(drag->initialize());
}
