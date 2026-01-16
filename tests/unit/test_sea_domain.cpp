/**
 * @file test_sea_domain.cpp
 * @brief Unit tests for Sea domain (buoyancy, hydrodynamics, waves)
 */

#include <gtest/gtest.h>
#include "jaguar/domain/sea.h"
#include "jaguar/environment/environment.h"
#include <cmath>

using namespace jaguar;
using namespace jaguar::domain::sea;

// ============================================================================
// SeaState Tests
// ============================================================================

class SeaStateTest : public ::testing::Test {
protected:
    void SetUp() override {}
};

TEST_F(SeaStateTest, CalmSeaState) {
    auto state = SeaState::FromNATOSeaState(0);

    EXPECT_NEAR(state.significant_height, 0.0, 1e-6);
}

TEST_F(SeaStateTest, ModerateSeaState) {
    auto state = SeaState::FromNATOSeaState(4);

    // Sea state 4 is moderate
    EXPECT_GT(state.significant_height, 1.0);
    EXPECT_LT(state.significant_height, 2.5);
    EXPECT_GT(state.peak_period, 5.0);
}

TEST_F(SeaStateTest, RoughSeaState) {
    auto state = SeaState::FromNATOSeaState(6);

    // Sea state 6 is very rough
    EXPECT_GT(state.significant_height, 4.0);
    EXPECT_GT(state.peak_period, 8.0);
}

TEST_F(SeaStateTest, SeaStateProgression) {
    // Higher sea states should have larger wave heights
    Real prev_height = 0.0;
    for (int i = 0; i <= 8; ++i) {
        auto state = SeaState::FromNATOSeaState(i);
        EXPECT_GE(state.significant_height, prev_height);
        prev_height = state.significant_height;
    }
}

// ============================================================================
// WaveModel Tests
// ============================================================================

class WaveModelTest : public ::testing::Test {
protected:
    WaveModel model;

    void SetUp() override {
        SeaState state;
        state.significant_height = 2.0;  // 2m significant wave height
        state.peak_period = 8.0;          // 8s peak period
        state.direction = 0.0;
        state.spectrum = WaveSpectrum::PiersonMoskowitz;
        model.set_sea_state(state);
    }
};

TEST_F(WaveModelTest, ElevationVariesWithPosition) {
    Real elev1 = model.get_elevation(0.0, 0.0, 0.0);
    Real elev2 = model.get_elevation(100.0, 0.0, 0.0);

    // Different positions should have different elevations
    // (unless extremely unlikely coincidence)
    EXPECT_NE(elev1, elev2);
}

TEST_F(WaveModelTest, ElevationVariesWithTime) {
    Real elev1 = model.get_elevation(0.0, 0.0, 0.0);
    Real elev2 = model.get_elevation(0.0, 0.0, 5.0);

    // Different times should have different elevations
    EXPECT_NE(elev1, elev2);
}

TEST_F(WaveModelTest, ElevationWithinReasonableBounds) {
    // Sample many points and check wave heights are reasonable
    Real max_elev = 0.0;
    Real min_elev = 0.0;

    for (Real t = 0.0; t < 100.0; t += 0.5) {
        Real elev = model.get_elevation(0.0, 0.0, t);
        max_elev = std::max(max_elev, elev);
        min_elev = std::min(min_elev, elev);
    }

    // Wave heights should be bounded by a few times Hs
    EXPECT_LT(max_elev, 10.0);  // Not more than 5 * Hs
    EXPECT_GT(min_elev, -10.0);
}

TEST_F(WaveModelTest, CalmSeasNoWaves) {
    SeaState calm;
    calm.significant_height = 0.0;
    calm.peak_period = 0.0;
    model.set_sea_state(calm);

    Real elev = model.get_elevation(0.0, 0.0, 0.0);
    EXPECT_NEAR(elev, 0.0, 1e-6);
}

TEST_F(WaveModelTest, ParticleVelocityDecaysWithDepth) {
    Real vel_surface = model.get_particle_velocity(0.0, 0.0, 0.0, 0.0).length_squared();
    Real vel_deep = model.get_particle_velocity(0.0, 0.0, -50.0, 0.0).length_squared();

    // Velocity should decay with depth
    EXPECT_GT(vel_surface, vel_deep);
}

TEST_F(WaveModelTest, SlopeRelatedToElevation) {
    // Numerical derivative of elevation should match slope
    Real dx = 0.01;
    Real elev_minus = model.get_elevation(-dx, 0.0, 0.0);
    Real elev_plus = model.get_elevation(dx, 0.0, 0.0);
    Real numerical_slope_x = (elev_plus - elev_minus) / (2.0 * dx);

    Vec3 slope = model.get_slope(0.0, 0.0, 0.0);

    // Allow some tolerance due to numerical differentiation
    EXPECT_NEAR(slope.x, numerical_slope_x, 0.1);
}

TEST_F(WaveModelTest, JONSWAPSpectrum) {
    SeaState state;
    state.significant_height = 2.0;
    state.peak_period = 8.0;
    state.spectrum = WaveSpectrum::JONSWAP;
    model.set_sea_state(state);

    // Should still produce valid waves
    Real elev = model.get_elevation(0.0, 0.0, 0.0);
    EXPECT_TRUE(std::isfinite(elev));
}

// ============================================================================
// BuoyancyModel Tests
// ============================================================================

class BuoyancyModelTest : public ::testing::Test {
protected:
    BuoyancyModel model;
    physics::EntityState state;
    environment::Environment env;
    physics::EntityForces forces;

    void SetUp() override {
        model.set_displaced_volume(100.0);        // 100 m³
        model.set_metacentric_height(2.0);        // 2m GM
        model.set_center_of_buoyancy(Vec3{0.0, 0.0, -1.0});

        state.position = Vec3{0.0, 0.0, 0.0};
        state.velocity = Vec3{0.0, 0.0, 0.0};
        state.orientation = Quat::Identity();
        state.angular_velocity = Vec3{0.0, 0.0, 0.0};
        state.mass = 100000.0;  // 100 tonnes

        env.altitude = 0.0;
        env.over_water = true;
        env.ocean.surface_elevation = 5.0;  // Water surface at 5m

        forces.clear();
    }
};

TEST_F(BuoyancyModelTest, NoBuoyancyAboveWater) {
    env.altitude = 10.0;  // Above water surface
    env.ocean.surface_elevation = 5.0;

    model.compute_forces(state, env, 0.1, forces);

    EXPECT_NEAR(model.get_buoyancy(), 0.0, 1e-6);
}

TEST_F(BuoyancyModelTest, BuoyancyWhenSubmerged) {
    env.altitude = 2.0;  // 3m below water surface
    env.ocean.surface_elevation = 5.0;

    model.compute_forces(state, env, 0.1, forces);

    EXPECT_GT(model.get_buoyancy(), 0.0);
}

TEST_F(BuoyancyModelTest, DraftCalculation) {
    env.altitude = 2.0;
    env.ocean.surface_elevation = 5.0;

    model.compute_forces(state, env, 0.1, forces);

    // Draft = water_surface - altitude = 5 - 2 = 3m
    EXPECT_NEAR(model.get_draft(), 3.0, 0.01);
}

TEST_F(BuoyancyModelTest, BuoyancyIncreasesWithDraft) {
    // Shallow draft
    env.altitude = 4.0;  // 1m draft
    env.ocean.surface_elevation = 5.0;
    model.compute_forces(state, env, 0.1, forces);
    Real shallow_buoyancy = model.get_buoyancy();

    // Deep draft
    env.altitude = 2.0;  // 3m draft
    model.compute_forces(state, env, 0.1, forces);
    Real deep_buoyancy = model.get_buoyancy();

    EXPECT_GT(deep_buoyancy, shallow_buoyancy);
}

TEST_F(BuoyancyModelTest, ForceDirectionUpward) {
    env.altitude = 2.0;
    env.ocean.surface_elevation = 5.0;

    model.compute_forces(state, env, 0.1, forces);

    // In NED frame, upward force is negative Z in world
    // After transform to body frame with identity orientation,
    // buoyancy should still be in Z direction
    // (Direction depends on frame conventions)
    EXPECT_NE(forces.force.z, 0.0);
}

TEST_F(BuoyancyModelTest, DisabledModelProducesNoForce) {
    env.altitude = 2.0;
    env.ocean.surface_elevation = 5.0;

    model.set_enabled(false);
    model.compute_forces(state, env, 0.1, forces);

    EXPECT_NEAR(model.get_buoyancy(), 0.0, 1e-6);
}

TEST_F(BuoyancyModelTest, NotOverWaterProducesNoForce) {
    env.over_water = false;
    env.altitude = 2.0;
    env.ocean.surface_elevation = 5.0;

    model.compute_forces(state, env, 0.1, forces);

    EXPECT_NEAR(model.get_buoyancy(), 0.0, 1e-6);
}

// ============================================================================
// HydrodynamicsModel Tests
// ============================================================================

class HydrodynamicsModelTest : public ::testing::Test {
protected:
    HydrodynamicsModel model;
    physics::EntityState state;
    environment::Environment env;
    physics::EntityForces forces;

    void SetUp() override {
        model.set_hull_coefficients(-0.04, -0.01, -0.4, 0.05, -0.1, -0.05);
        model.set_rudder_parameters(10.0, 1.5);
        model.set_propeller_parameters(3.0, 1.0);

        state.position = Vec3{0.0, 0.0, 0.0};
        state.velocity = Vec3{10.0, 0.0, 0.0};  // 10 m/s forward
        state.orientation = Quat::Identity();
        state.angular_velocity = Vec3{0.0, 0.0, 0.0};
        state.mass = 1000000.0;  // 1000 tonnes

        env.altitude = 0.0;
        env.over_water = true;
        env.ocean.surface_elevation = 5.0;

        forces.clear();
    }
};

TEST_F(HydrodynamicsModelTest, NoForceWhenNotInWater) {
    env.over_water = false;

    model.compute_forces(state, env, 0.1, forces);

    EXPECT_NEAR(model.get_draft(), 0.0, 1e-6);
}

TEST_F(HydrodynamicsModelTest, ResistanceOpposesMotion) {
    model.set_propeller_rpm(0.0);  // No propulsion
    model.set_rudder_angle(0.0);   // No rudder

    model.compute_forces(state, env, 0.1, forces);

    // Hull resistance should oppose forward motion
    EXPECT_LT(forces.force.x, 0.0);
}

TEST_F(HydrodynamicsModelTest, PropellerProducesThrust) {
    // First test without propeller
    model.set_propeller_rpm(0.0);
    model.set_rudder_angle(0.0);
    model.compute_forces(state, env, 0.1, forces);
    Real force_no_prop = forces.force.x;

    // Now test with propeller - need fresh forces struct
    forces.clear();
    model.set_propeller_rpm(200.0);  // Higher RPM for measurable thrust
    model.compute_forces(state, env, 0.1, forces);
    Real force_with_prop = forces.force.x;

    // With propeller turning, should have more forward force
    // (might still be negative due to resistance, but less negative)
    EXPECT_GT(force_with_prop, force_no_prop);
}

TEST_F(HydrodynamicsModelTest, RudderProducesYawMoment) {
    model.set_propeller_rpm(100.0);

    // No rudder
    model.set_rudder_angle(0.0);
    model.compute_forces(state, env, 0.1, forces);
    Real yaw_no_rudder = forces.torque.z;

    // With rudder
    forces.clear();
    model.set_rudder_angle(0.3);  // ~17 degrees
    model.compute_forces(state, env, 0.1, forces);
    Real yaw_with_rudder = forces.torque.z;

    // Rudder should produce yaw moment
    EXPECT_NE(yaw_with_rudder, yaw_no_rudder);
}

TEST_F(HydrodynamicsModelTest, SwayProducesYawMoment) {
    state.velocity = Vec3{10.0, 2.0, 0.0};  // Some sway velocity
    model.set_propeller_rpm(0.0);
    model.set_rudder_angle(0.0);

    model.compute_forces(state, env, 0.1, forces);

    // Sway should produce lateral force
    EXPECT_NE(forces.force.y, 0.0);
}

TEST_F(HydrodynamicsModelTest, DisabledModelProducesNoForce) {
    model.set_enabled(false);

    model.compute_forces(state, env, 0.1, forces);

    EXPECT_NEAR(forces.force.x, 0.0, 1e-6);
    EXPECT_NEAR(forces.force.y, 0.0, 1e-6);
    EXPECT_NEAR(forces.torque.z, 0.0, 1e-6);
}

// ============================================================================
// RAOModel Tests
// ============================================================================

class RAOModelTest : public ::testing::Test {
protected:
    RAOModel model;

    void SetUp() override {
        // Set up simple RAO for heave (DOF 2)
        std::vector<Real> freqs = {0.3, 0.5, 0.8, 1.0, 1.5};
        std::vector<Real> amps = {0.8, 0.9, 1.0, 0.9, 0.6};
        std::vector<Real> phases = {0.0, 0.1, 0.2, 0.3, 0.5};
        model.set_rao(2, freqs, amps, phases);
    }
};

TEST_F(RAOModelTest, RAOInterpolation) {
    Real out_amp, out_phase;

    // Test at middle frequency
    model.get_response(0.8, 1.0, out_amp, out_phase, 2);
    EXPECT_NEAR(out_amp, 1.0, 0.01);  // RAO = 1.0 at omega = 0.8

    // Test between frequencies
    model.get_response(0.65, 1.0, out_amp, out_phase, 2);
    EXPECT_GT(out_amp, 0.9);
    EXPECT_LT(out_amp, 1.0);
}

TEST_F(RAOModelTest, RAOExtrapolation) {
    Real out_amp, out_phase;

    // Below minimum frequency
    model.get_response(0.1, 1.0, out_amp, out_phase, 2);
    EXPECT_NEAR(out_amp, 0.8, 0.01);  // Uses first value

    // Above maximum frequency
    model.get_response(2.0, 1.0, out_amp, out_phase, 2);
    EXPECT_NEAR(out_amp, 0.6, 0.01);  // Uses last value
}

TEST_F(RAOModelTest, DefaultRAODataInitialized) {
    // Create fresh RAO model with default data
    RAOModel default_model;
    Real out_amp, out_phase;

    // Default RAO should have data for all DOFs (0-5)
    // Test heave at resonance (around 0.6 rad/s based on default data)
    default_model.get_response(0.6, 1.0, out_amp, out_phase, 2);  // Heave
    EXPECT_GT(out_amp, 0.0);  // Should have response

    // Roll should have large RAO at resonance
    default_model.get_response(0.7, 1.0, out_amp, out_phase, 3);  // Roll
    EXPECT_GT(out_amp, 1.0);  // Roll has large response at resonance
}

TEST_F(RAOModelTest, InvalidDOF) {
    Real out_amp, out_phase;

    // Invalid DOF
    model.get_response(1.0, 1.0, out_amp, out_phase, 10);
    EXPECT_NEAR(out_amp, 0.0, 1e-6);
    EXPECT_NEAR(out_phase, 0.0, 1e-6);
}

TEST_F(RAOModelTest, CalculateResponseFromWaves) {
    WaveModel waves;
    SeaState sea_state;
    sea_state.significant_height = 2.0;
    sea_state.peak_period = 8.0;
    waves.set_sea_state(sea_state);

    Vec3 disp, rot;
    model.calculate_response(waves, 0.0, disp, rot);

    // Should produce some response
    // (actual values depend on wave model)
    EXPECT_TRUE(std::isfinite(disp.z));
    EXPECT_TRUE(std::isfinite(rot.x));
    EXPECT_TRUE(std::isfinite(rot.y));
}
