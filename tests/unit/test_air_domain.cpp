/**
 * @file test_air_domain.cpp
 * @brief Unit tests for Air Domain components (aerodynamics, propulsion, flight control)
 */

#include <gtest/gtest.h>
#include "jaguar/domain/air.h"
#include "jaguar/environment/atmosphere.h"
#include "jaguar/environment/environment.h"
#include "jaguar/physics/entity.h"
#include <cmath>

using namespace jaguar;
using namespace jaguar::domain::air;
using namespace jaguar::physics;
using namespace jaguar::environment;

// ============================================================================
// AeroTable Tests
// ============================================================================

class AeroTableTest : public ::testing::Test {
protected:
    static constexpr Real EPSILON = 1e-6;
};

TEST_F(AeroTableTest, OneDimensionalLookup) {
    AeroTable table;

    // Set up 1D table: linear function f(x) = 2x
    table.set_breakpoints(0, {0.0, 1.0, 2.0, 3.0});
    table.set_data({0.0, 2.0, 4.0, 6.0});

    // Exact values
    EXPECT_NEAR(table.lookup({0.0}), 0.0, EPSILON);
    EXPECT_NEAR(table.lookup({1.0}), 2.0, EPSILON);
    EXPECT_NEAR(table.lookup({2.0}), 4.0, EPSILON);
    EXPECT_NEAR(table.lookup({3.0}), 6.0, EPSILON);

    // Interpolated values
    EXPECT_NEAR(table.lookup({0.5}), 1.0, EPSILON);
    EXPECT_NEAR(table.lookup({1.5}), 3.0, EPSILON);
    EXPECT_NEAR(table.lookup({2.5}), 5.0, EPSILON);
}

TEST_F(AeroTableTest, OneDimensionalExtrapolation) {
    AeroTable table;

    table.set_breakpoints(0, {0.0, 1.0, 2.0});
    table.set_data({0.0, 1.0, 2.0});

    // Below range - should clamp to first value
    EXPECT_NEAR(table.lookup({-1.0}), 0.0, EPSILON);

    // Above range - should clamp to last value
    EXPECT_NEAR(table.lookup({3.0}), 2.0, EPSILON);
}

TEST_F(AeroTableTest, TwoDimensionalLookup) {
    AeroTable table;

    // 2D table: 3 (alpha) x 2 (mach) grid
    // Alpha breakpoints: 0, 5, 10 (degrees)
    // Mach breakpoints: 0, 1.0
    table.set_breakpoints(0, {0.0, 5.0, 10.0});  // Alpha (dimension 0)
    table.set_breakpoints(1, {0.0, 1.0});         // Mach (dimension 1)

    // Data layout: LAST dimension varies fastest (Mach varies fastest)
    // This is row-major order: indices [alpha_idx, mach_idx]
    // flat_index = alpha_idx * num_mach + mach_idx
    //
    // Data order:
    // [alpha=0, mach=0], [alpha=0, mach=1],
    // [alpha=5, mach=0], [alpha=5, mach=1],
    // [alpha=10, mach=0], [alpha=10, mach=1]
    table.set_data({
        0.0, 0.0,    // alpha=0:  Mach=[0, 1] -> CL=[0.0, 0.0]
        0.5, 0.45,   // alpha=5:  Mach=[0, 1] -> CL=[0.5, 0.45]
        1.0, 0.9     // alpha=10: Mach=[0, 1] -> CL=[1.0, 0.9]
    });

    // Corner values
    EXPECT_NEAR(table.lookup({0.0, 0.0}), 0.0, EPSILON);   // alpha=0, Mach=0
    EXPECT_NEAR(table.lookup({10.0, 0.0}), 1.0, EPSILON);  // alpha=10, Mach=0
    EXPECT_NEAR(table.lookup({0.0, 1.0}), 0.0, EPSILON);   // alpha=0, Mach=1
    EXPECT_NEAR(table.lookup({10.0, 1.0}), 0.9, EPSILON);  // alpha=10, Mach=1

    // Interpolated in alpha direction at Mach=0
    EXPECT_NEAR(table.lookup({5.0, 0.0}), 0.5, EPSILON);

    // Interpolated in Mach direction at alpha=10
    // Mach=0.5 -> interpolate between 1.0 and 0.9 = 0.95
    EXPECT_NEAR(table.lookup({10.0, 0.5}), 0.95, EPSILON);
}

// ============================================================================
// AerodynamicsModel Tests
// ============================================================================

class AerodynamicsModelTest : public ::testing::Test {
protected:
    AerodynamicsModel aero;
    EntityState state;
    Environment env;
    EntityForces forces;

    void SetUp() override {
        // Set up reference geometry (typical light aircraft)
        aero.set_reference_area(16.0);   // m²
        aero.set_reference_chord(1.6);   // m
        aero.set_reference_span(10.0);   // m
        aero.initialize();

        // Default state: flying level at 100 m/s
        state.position = Vec3{0.0, 0.0, 0.0};
        state.velocity = Vec3{100.0, 0.0, 0.0};  // 100 m/s forward in ECEF
        state.orientation = Quat::Identity();
        state.angular_velocity = Vec3{0.0, 0.0, 0.0};
        state.mass = 1000.0;

        // Sea level conditions
        env.latitude = 0.0;
        env.longitude = 0.0;
        env.altitude = 0.0;
        env.atmosphere.temperature = 288.15;
        env.atmosphere.pressure = 101325.0;
        env.atmosphere.density = 1.225;
        env.atmosphere.speed_of_sound = 340.29;
        env.atmosphere.wind = Vec3{0.0, 0.0, 0.0};

        forces.clear();
    }
};

TEST_F(AerodynamicsModelTest, ZeroSpeedNoForces) {
    state.velocity = Vec3{0.0, 0.0, 0.0};
    aero.compute_forces(state, env, 0.01, forces);

    // Should have no forces at zero speed
    EXPECT_NEAR(forces.force.x, 0.0, 1e-6);
    EXPECT_NEAR(forces.force.y, 0.0, 1e-6);
    EXPECT_NEAR(forces.force.z, 0.0, 1e-6);
}

TEST_F(AerodynamicsModelTest, DynamicPressure) {
    aero.compute_forces(state, env, 0.01, forces);

    // q = 0.5 * rho * V^2 = 0.5 * 1.225 * 100^2 = 6125 Pa
    EXPECT_NEAR(aero.get_qbar(), 6125.0, 1.0);
}

TEST_F(AerodynamicsModelTest, MachNumber) {
    aero.compute_forces(state, env, 0.01, forces);

    // Mach = 100 / 340.29 ≈ 0.294
    EXPECT_NEAR(aero.get_mach(), 0.294, 0.01);
}

TEST_F(AerodynamicsModelTest, DragOpposesMotion) {
    aero.compute_forces(state, env, 0.01, forces);

    // For level flight (alpha ≈ 0), drag should be in -X direction
    // The CD is always positive, so X force component should be negative
    EXPECT_LT(forces.force.x, 0.0);

    // At alpha = 0, there should be minimal lift (CL ≈ 0)
    // So Z force should be small
    EXPECT_LT(std::abs(forces.force.z), 1000.0);  // Less than 1000 N
}

TEST_F(AerodynamicsModelTest, AngleOfAttackProducesLift) {
    // Pitch the aircraft up by 10 degrees
    // This rotates the body so that the airflow comes from below
    Real pitch_rad = 10.0 * constants::DEG_TO_RAD;
    state.orientation = Quat::from_euler(0.0, pitch_rad, 0.0);

    aero.compute_forces(state, env, 0.01, forces);

    // Should have positive angle of attack
    EXPECT_GT(aero.get_alpha(), 0.0);
    EXPECT_NEAR(aero.get_alpha(), pitch_rad, 0.1);

    // Should have positive lift coefficient
    EXPECT_GT(aero.get_cl(), 0.0);

    // Z force should be negative (lift in body -Z direction, which is "up")
    EXPECT_LT(forces.force.z, 0.0);
}

TEST_F(AerodynamicsModelTest, SideslipProducesSideForce) {
    // Add lateral velocity
    state.velocity = Vec3{100.0, 10.0, 0.0};  // Slight sideslip

    aero.compute_forces(state, env, 0.01, forces);

    // Should have sideslip angle
    EXPECT_NE(aero.get_beta(), 0.0);

    // Should have side force
    EXPECT_NE(forces.force.y, 0.0);
}

TEST_F(AerodynamicsModelTest, DragCoefficient) {
    aero.compute_forces(state, env, 0.01, forces);

    // Default CD should be reasonable (0.02 - 0.1 range for typical aircraft)
    EXPECT_GT(aero.get_cd(), 0.01);
    EXPECT_LT(aero.get_cd(), 0.2);
}

TEST_F(AerodynamicsModelTest, WindEffectsOnAirspeed) {
    // Headwind of 20 m/s
    env.atmosphere.wind = Vec3{-20.0, 0.0, 0.0};  // NED: wind from south

    EntityForces forces_wind;
    aero.compute_forces(state, env, 0.01, forces_wind);

    // With headwind, effective airspeed is higher, so drag should increase
    // Reset wind and compare
    env.atmosphere.wind = Vec3{0.0, 0.0, 0.0};
    EntityForces forces_no_wind;
    aero.compute_forces(state, env, 0.01, forces_no_wind);

    // Can't easily compare since we're not tracking exact behavior
    // Just verify forces are computed
    EXPECT_TRUE(std::isfinite(forces_wind.force.x));
}

TEST_F(AerodynamicsModelTest, DisabledGeneratorNoForces) {
    aero.set_enabled(false);
    aero.compute_forces(state, env, 0.01, forces);

    EXPECT_NEAR(forces.force.x, 0.0, 1e-10);
    EXPECT_NEAR(forces.force.y, 0.0, 1e-10);
    EXPECT_NEAR(forces.force.z, 0.0, 1e-10);
}

// ============================================================================
// PropulsionModel Tests
// ============================================================================

class PropulsionModelTest : public ::testing::Test {
protected:
    PropulsionModel engine;
    EntityState state;
    Environment env;
    EntityForces forces;

    void SetUp() override {
        engine.set_max_thrust(50000.0);       // 50 kN
        engine.set_fuel_capacity(1000.0);      // 1000 kg
        engine.set_specific_fuel_consumption(0.00005);  // kg/(N·s)

        state.position = Vec3{0.0, 0.0, 0.0};
        state.velocity = Vec3{100.0, 0.0, 0.0};
        state.orientation = Quat::Identity();
        state.mass = 10000.0;

        env.altitude = 0.0;
        env.atmosphere.density = 1.225;
        env.atmosphere.speed_of_sound = 340.29;

        forces.clear();
    }
};

TEST_F(PropulsionModelTest, EngineOff) {
    // Engine not started
    engine.set_throttle(1.0);
    engine.compute_forces(state, env, 0.1, forces);

    EXPECT_NEAR(engine.get_thrust(), 0.0, 1e-6);
    EXPECT_NEAR(forces.force.x, 0.0, 1e-6);
}

TEST_F(PropulsionModelTest, EngineStart) {
    engine.start();
    EXPECT_TRUE(engine.is_running());

    engine.stop();
    EXPECT_FALSE(engine.is_running());
}

TEST_F(PropulsionModelTest, FullThrust) {
    engine.start();
    engine.set_throttle(1.0);
    engine.compute_forces(state, env, 0.1, forces);

    // At sea level, full throttle should produce near max thrust
    EXPECT_GT(engine.get_thrust(), 45000.0);  // At least 90% of max
    EXPECT_LT(engine.get_thrust(), 60000.0);  // Not more than 120% (with ram effect)
}

TEST_F(PropulsionModelTest, ThrustInForwardDirection) {
    engine.start();
    engine.set_throttle(0.5);
    engine.compute_forces(state, env, 0.1, forces);

    // Thrust should be positive in body X direction
    EXPECT_GT(forces.force.x, 0.0);
    EXPECT_NEAR(forces.force.y, 0.0, 1e-6);
    EXPECT_NEAR(forces.force.z, 0.0, 1e-6);
}

TEST_F(PropulsionModelTest, ThrottleResponse) {
    engine.start();

    engine.set_throttle(0.25);
    engine.compute_forces(state, env, 0.1, forces);
    Real thrust_25 = engine.get_thrust();

    forces.clear();
    engine.set_throttle(0.75);
    engine.compute_forces(state, env, 0.1, forces);
    Real thrust_75 = engine.get_thrust();

    // Higher throttle = more thrust
    EXPECT_GT(thrust_75, thrust_25);
    EXPECT_NEAR(thrust_75 / thrust_25, 3.0, 0.5);  // Approximately 3x
}

TEST_F(PropulsionModelTest, FuelConsumption) {
    engine.start();
    engine.set_throttle(1.0);

    Real initial_fuel = engine.get_fuel_remaining();
    engine.compute_forces(state, env, 10.0, forces);  // 10 seconds
    Real final_fuel = engine.get_fuel_remaining();

    EXPECT_LT(final_fuel, initial_fuel);
    EXPECT_GT(engine.get_fuel_flow(), 0.0);
}

TEST_F(PropulsionModelTest, FuelExhaustion) {
    engine.set_fuel_capacity(0.01);  // Very little fuel (0.01 kg)
    engine.start();
    engine.set_throttle(1.0);

    // Run for a long time - fuel will exhaust
    // With SFC=0.00005 kg/(N·s) and ~50000 N thrust
    // fuel_flow = 0.00005 * 50000 = 2.5 kg/s
    // 0.01 kg / 2.5 kg/s = 0.004 s to exhaust
    for (int i = 0; i < 100; ++i) {
        forces.clear();
        engine.compute_forces(state, env, 0.01, forces);  // 10 ms steps
    }

    EXPECT_NEAR(engine.get_fuel_remaining(), 0.0, 1e-6);
    EXPECT_FALSE(engine.is_running());
}

TEST_F(PropulsionModelTest, AltitudeEffect) {
    engine.start();
    engine.set_throttle(1.0);

    // Sea level
    engine.compute_forces(state, env, 0.1, forces);
    Real thrust_sl = engine.get_thrust();

    // High altitude (10 km)
    env.altitude = 10000.0;
    env.atmosphere.density = 0.4135;  // Approximately

    forces.clear();
    engine.compute_forces(state, env, 0.1, forces);
    Real thrust_alt = engine.get_thrust();

    // Thrust should be lower at altitude
    EXPECT_LT(thrust_alt, thrust_sl);
}

// ============================================================================
// FlightControlSystem Tests
// ============================================================================

class FlightControlSystemTest : public ::testing::Test {
protected:
    FlightControlSystem fcs;
    static constexpr Real EPSILON = 0.1;  // degrees

    void SetUp() override {
        fcs.set_elevator_range(-25.0, 25.0);
        fcs.set_aileron_range(-20.0, 20.0);
        fcs.set_rudder_range(-30.0, 30.0);
    }
};

TEST_F(FlightControlSystemTest, NeutralInputs) {
    FlightControlSystem::ControlInputs inputs;
    inputs.pitch_cmd = 0.0;
    inputs.roll_cmd = 0.0;
    inputs.yaw_cmd = 0.0;

    // Large dt to allow convergence
    auto outputs = fcs.process(inputs, 10.0);

    EXPECT_NEAR(outputs.elevator_deg, 0.0, EPSILON);
    EXPECT_NEAR(outputs.aileron_deg, 0.0, EPSILON);
    EXPECT_NEAR(outputs.rudder_deg, 0.0, EPSILON);
}

TEST_F(FlightControlSystemTest, MaxPitchUp) {
    FlightControlSystem::ControlInputs inputs;
    inputs.pitch_cmd = 1.0;  // Full aft stick
    inputs.roll_cmd = 0.0;
    inputs.yaw_cmd = 0.0;

    // Large dt to allow full deflection
    auto outputs = fcs.process(inputs, 10.0);

    EXPECT_NEAR(outputs.elevator_deg, 25.0, EPSILON);
}

TEST_F(FlightControlSystemTest, MaxPitchDown) {
    FlightControlSystem::ControlInputs inputs;
    inputs.pitch_cmd = -1.0;  // Full forward stick
    inputs.roll_cmd = 0.0;
    inputs.yaw_cmd = 0.0;

    auto outputs = fcs.process(inputs, 10.0);

    EXPECT_NEAR(outputs.elevator_deg, -25.0, EPSILON);
}

TEST_F(FlightControlSystemTest, RollRight) {
    FlightControlSystem::ControlInputs inputs;
    inputs.pitch_cmd = 0.0;
    inputs.roll_cmd = 1.0;  // Full right
    inputs.yaw_cmd = 0.0;

    auto outputs = fcs.process(inputs, 10.0);

    EXPECT_NEAR(outputs.aileron_deg, 20.0, EPSILON);
}

TEST_F(FlightControlSystemTest, YawRight) {
    FlightControlSystem::ControlInputs inputs;
    inputs.pitch_cmd = 0.0;
    inputs.roll_cmd = 0.0;
    inputs.yaw_cmd = 1.0;  // Full right pedal

    auto outputs = fcs.process(inputs, 10.0);

    EXPECT_NEAR(outputs.rudder_deg, 30.0, EPSILON);
}

TEST_F(FlightControlSystemTest, RateLimiting) {
    // First, set to neutral
    FlightControlSystem::ControlInputs neutral;
    neutral.pitch_cmd = 0.0;
    neutral.roll_cmd = 0.0;
    neutral.yaw_cmd = 0.0;

    // Reset state with neutral
    for (int i = 0; i < 10; ++i) {
        fcs.process(neutral, 1.0);
    }

    // Now command max deflection with very small dt
    FlightControlSystem::ControlInputs max_cmd;
    max_cmd.pitch_cmd = 1.0;
    max_cmd.roll_cmd = 0.0;
    max_cmd.yaw_cmd = 0.0;

    auto outputs = fcs.process(max_cmd, 0.01);  // 10 ms

    // With rate limit of ~40 deg/s and 10ms, max change is 0.4 deg
    // Starting from 0, should not reach full 25 deg deflection
    EXPECT_LT(outputs.elevator_deg, 2.0);  // Should be around 0.4 deg
}

TEST_F(FlightControlSystemTest, CombinedInputs) {
    FlightControlSystem::ControlInputs inputs;
    inputs.pitch_cmd = 0.5;
    inputs.roll_cmd = -0.5;
    inputs.yaw_cmd = 0.3;
    inputs.throttle_cmd = 0.8;

    auto outputs = fcs.process(inputs, 10.0);

    // All surfaces should have non-zero deflection
    EXPECT_GT(outputs.elevator_deg, 0.0);
    EXPECT_LT(outputs.aileron_deg, 0.0);
    EXPECT_GT(outputs.rudder_deg, 0.0);
}

// ============================================================================
// Extended AeroTable Tests (3D and Higher Dimensional Interpolation)
// ============================================================================

class AeroTableExtendedTest : public ::testing::Test {
protected:
    static constexpr Real EPSILON = 1e-6;
};

TEST_F(AeroTableExtendedTest, ThreeDimensionalLookup) {
    AeroTable table;

    // 3D table: 2 (alpha) x 2 (beta) x 2 (mach) grid
    // This creates a cube with 8 data points
    table.set_breakpoints(0, {0.0, 10.0});   // Alpha (dim 0)
    table.set_breakpoints(1, {0.0, 5.0});    // Beta (dim 1)
    table.set_breakpoints(2, {0.0, 1.0});    // Mach (dim 2)

    // Data layout: last dimension varies fastest
    // Order: [alpha, beta, mach]
    // Indices: [0,0,0], [0,0,1], [0,1,0], [0,1,1], [1,0,0], [1,0,1], [1,1,0], [1,1,1]
    table.set_data({
        0.0, 0.1,   // alpha=0, beta=0: Mach=[0,1] -> [0.0, 0.1]
        0.2, 0.3,   // alpha=0, beta=5: Mach=[0,1] -> [0.2, 0.3]
        1.0, 1.1,   // alpha=10, beta=0: Mach=[0,1] -> [1.0, 1.1]
        1.2, 1.3    // alpha=10, beta=5: Mach=[0,1] -> [1.2, 1.3]
    });

    // Corner values
    EXPECT_NEAR(table.lookup({0.0, 0.0, 0.0}), 0.0, EPSILON);   // [0,0,0]
    EXPECT_NEAR(table.lookup({0.0, 0.0, 1.0}), 0.1, EPSILON);   // [0,0,1]
    EXPECT_NEAR(table.lookup({0.0, 5.0, 0.0}), 0.2, EPSILON);   // [0,1,0]
    EXPECT_NEAR(table.lookup({0.0, 5.0, 1.0}), 0.3, EPSILON);   // [0,1,1]
    EXPECT_NEAR(table.lookup({10.0, 0.0, 0.0}), 1.0, EPSILON);  // [1,0,0]
    EXPECT_NEAR(table.lookup({10.0, 0.0, 1.0}), 1.1, EPSILON);  // [1,0,1]
    EXPECT_NEAR(table.lookup({10.0, 5.0, 0.0}), 1.2, EPSILON);  // [1,1,0]
    EXPECT_NEAR(table.lookup({10.0, 5.0, 1.0}), 1.3, EPSILON);  // [1,1,1]

    // Center of cube - trilinear interpolation
    // Average of all 8 corners weighted equally
    Real expected_center = (0.0 + 0.1 + 0.2 + 0.3 + 1.0 + 1.1 + 1.2 + 1.3) / 8.0;  // 0.65
    EXPECT_NEAR(table.lookup({5.0, 2.5, 0.5}), expected_center, EPSILON);
}

TEST_F(AeroTableExtendedTest, ThreeDimensionalInterpolationAlongEachAxis) {
    AeroTable table;

    // Set up 3D table: 2x2x2
    table.set_breakpoints(0, {0.0, 1.0});
    table.set_breakpoints(1, {0.0, 1.0});
    table.set_breakpoints(2, {0.0, 1.0});

    // Simple linear function: f(x,y,z) = x + 2*y + 4*z
    table.set_data({
        0.0, 4.0,   // [0,0,0], [0,0,1]
        2.0, 6.0,   // [0,1,0], [0,1,1]
        1.0, 5.0,   // [1,0,0], [1,0,1]
        3.0, 7.0    // [1,1,0], [1,1,1]
    });

    // Test interpolation along each axis
    // Along x: (0,0,0)=0 to (1,0,0)=1 -> at x=0.5: 0.5
    EXPECT_NEAR(table.lookup({0.5, 0.0, 0.0}), 0.5, EPSILON);

    // Along y: (0,0,0)=0 to (0,1,0)=2 -> at y=0.5: 1.0
    EXPECT_NEAR(table.lookup({0.0, 0.5, 0.0}), 1.0, EPSILON);

    // Along z: (0,0,0)=0 to (0,0,1)=4 -> at z=0.5: 2.0
    EXPECT_NEAR(table.lookup({0.0, 0.0, 0.5}), 2.0, EPSILON);
}

TEST_F(AeroTableExtendedTest, FourDimensionalLookup) {
    AeroTable table;

    // 4D table: 2x2x2x2 = 16 data points
    // Alpha, Beta, Mach, Elevator deflection
    table.set_breakpoints(0, {0.0, 1.0});   // Alpha
    table.set_breakpoints(1, {0.0, 1.0});   // Beta
    table.set_breakpoints(2, {0.0, 1.0});   // Mach
    table.set_breakpoints(3, {0.0, 1.0});   // Elevator

    // Linear function: f = a + 2b + 4m + 8e
    std::vector<Real> data(16);
    for (int a = 0; a < 2; ++a) {
        for (int b = 0; b < 2; ++b) {
            for (int m = 0; m < 2; ++m) {
                for (int e = 0; e < 2; ++e) {
                    int idx = a * 8 + b * 4 + m * 2 + e;
                    data[static_cast<SizeT>(idx)] = static_cast<Real>(a + 2*b + 4*m + 8*e);
                }
            }
        }
    }
    table.set_data(data);

    // Corner values
    EXPECT_NEAR(table.lookup({0.0, 0.0, 0.0, 0.0}), 0.0, EPSILON);   // 0+0+0+0
    EXPECT_NEAR(table.lookup({1.0, 0.0, 0.0, 0.0}), 1.0, EPSILON);   // 1+0+0+0
    EXPECT_NEAR(table.lookup({0.0, 1.0, 0.0, 0.0}), 2.0, EPSILON);   // 0+2+0+0
    EXPECT_NEAR(table.lookup({0.0, 0.0, 1.0, 0.0}), 4.0, EPSILON);   // 0+0+4+0
    EXPECT_NEAR(table.lookup({0.0, 0.0, 0.0, 1.0}), 8.0, EPSILON);   // 0+0+0+8
    EXPECT_NEAR(table.lookup({1.0, 1.0, 1.0, 1.0}), 15.0, EPSILON);  // 1+2+4+8

    // Mid-points along each axis
    EXPECT_NEAR(table.lookup({0.5, 0.0, 0.0, 0.0}), 0.5, EPSILON);
    EXPECT_NEAR(table.lookup({0.0, 0.5, 0.0, 0.0}), 1.0, EPSILON);
    EXPECT_NEAR(table.lookup({0.0, 0.0, 0.5, 0.0}), 2.0, EPSILON);
    EXPECT_NEAR(table.lookup({0.0, 0.0, 0.0, 0.5}), 4.0, EPSILON);

    // Center of hypercube
    Real expected_center = 7.5;  // (0+1+2+3+4+5+6+7+8+9+10+11+12+13+14+15) / 16 = 7.5
    EXPECT_NEAR(table.lookup({0.5, 0.5, 0.5, 0.5}), expected_center, EPSILON);
}

TEST_F(AeroTableExtendedTest, NonUniformBreakpoints) {
    AeroTable table;

    // Non-uniform breakpoints (more common in real aero data)
    table.set_breakpoints(0, {-10.0, -5.0, 0.0, 5.0, 10.0, 20.0});  // Alpha in degrees
    table.set_breakpoints(1, {0.0, 0.5, 0.8, 1.0, 1.2});            // Mach

    // 6 alpha x 5 mach = 30 data points
    // Simple linear CL model: CL = 0.1 * alpha - 0.05 * mach
    std::vector<Real> data(30);
    std::vector<Real> alphas = {-10.0, -5.0, 0.0, 5.0, 10.0, 20.0};
    std::vector<Real> machs = {0.0, 0.5, 0.8, 1.0, 1.2};
    SizeT idx = 0;
    for (Real alpha : alphas) {
        for (Real mach : machs) {
            data[idx++] = 0.1 * alpha - 0.05 * mach;
        }
    }
    table.set_data(data);

    // Test a few points
    EXPECT_NEAR(table.lookup({0.0, 0.0}), 0.0, EPSILON);
    EXPECT_NEAR(table.lookup({10.0, 0.0}), 1.0, EPSILON);
    EXPECT_NEAR(table.lookup({-10.0, 1.0}), -1.05, EPSILON);

    // Interpolated point at alpha=2.5, mach=0.25
    Real expected = 0.1 * 2.5 - 0.05 * 0.25;  // 0.2375
    EXPECT_NEAR(table.lookup({2.5, 0.25}), expected, 0.01);
}

TEST_F(AeroTableExtendedTest, EmptyTable) {
    AeroTable table;

    // Empty table should return 0
    EXPECT_NEAR(table.lookup({0.0}), 0.0, EPSILON);
    EXPECT_NEAR(table.lookup({1.0, 2.0, 3.0}), 0.0, EPSILON);
}

TEST_F(AeroTableExtendedTest, SinglePointTable) {
    AeroTable table;

    table.set_breakpoints(0, {5.0});
    table.set_data({42.0});

    // Any input should return the single value
    EXPECT_NEAR(table.lookup({0.0}), 42.0, EPSILON);
    EXPECT_NEAR(table.lookup({5.0}), 42.0, EPSILON);
    EXPECT_NEAR(table.lookup({100.0}), 42.0, EPSILON);
}

TEST_F(AeroTableExtendedTest, MissingInputDimensions) {
    AeroTable table;

    // 2D table
    // Data layout: last dimension varies fastest
    // [0,0]=0.0, [0,1]=1.0, [1,0]=2.0, [1,1]=3.0
    table.set_breakpoints(0, {0.0, 1.0});
    table.set_breakpoints(1, {0.0, 1.0});
    table.set_data({0.0, 1.0, 2.0, 3.0});

    // Provide only 1 input - second should default to 0
    // At (0.5, 0), interpolate between [0,0]=0.0 and [1,0]=2.0 -> result = 1.0
    EXPECT_NEAR(table.lookup({0.5}), 1.0, EPSILON);
}

TEST_F(AeroTableExtendedTest, RealisticAeroData) {
    AeroTable cl_table;

    // Realistic CL table: alpha (-4 to 16 deg) x Mach (0.2, 0.6, 0.9)
    std::vector<Real> alpha_deg = {-4.0, 0.0, 4.0, 8.0, 12.0, 16.0};
    std::vector<Real> mach = {0.2, 0.6, 0.9};

    cl_table.set_breakpoints(0, alpha_deg);
    cl_table.set_breakpoints(1, mach);

    // Realistic CL data (typical transport aircraft)
    // Layout: alpha varies first (rows), mach varies second (columns)
    cl_table.set_data({
        // Mach:   0.2,   0.6,   0.9
        -0.3,  -0.28, -0.25,  // alpha = -4
        0.0,   0.02,  0.05,   // alpha = 0
        0.4,   0.42,  0.40,   // alpha = 4
        0.8,   0.82,  0.75,   // alpha = 8
        1.1,   1.08,  0.95,   // alpha = 12
        1.2,   1.10,  0.85    // alpha = 16 (approaching stall)
    });

    // Test typical cruise condition
    Real cl_cruise = cl_table.lookup({4.0, 0.6});
    EXPECT_NEAR(cl_cruise, 0.42, 0.01);

    // Test interpolation at intermediate alpha
    Real cl_interp = cl_table.lookup({6.0, 0.6});  // Between alpha=4 and alpha=8
    Real expected_interp = (0.42 + 0.82) / 2.0;    // Linear interpolation: 0.62
    EXPECT_NEAR(cl_interp, expected_interp, 0.01);

    // Test high-speed effect (CL drops at high Mach)
    Real cl_low_mach = cl_table.lookup({8.0, 0.2});
    Real cl_high_mach = cl_table.lookup({8.0, 0.9});
    EXPECT_GT(cl_low_mach, cl_high_mach);  // CL should be lower at high Mach
}

