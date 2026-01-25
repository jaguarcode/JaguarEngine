/**
 * @file test_energy_conservation.cpp
 * @brief Energy conservation validation tests
 *
 * Tests that total energy (kinetic + potential) is conserved in closed systems
 * and properly dissipated in systems with damping.
 *
 * Theoretical Basis:
 * - Total Energy: E = KE + PE = 0.5*m*v² + m*g*h
 * - Conservative systems: dE/dt = 0
 * - Dissipative systems: dE/dt = -P_dissipated
 *
 * References:
 * - Goldstein, "Classical Mechanics", Chapter 1
 * - Greenwood, "Principles of Dynamics", Chapter 3
 */

#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include <memory>
#include <string>

#include "jaguar/physics/entity.h"
#include "jaguar/physics/solver.h"

namespace jaguar::test {

using namespace jaguar::physics;

class EnergyConservationTest : public ::testing::Test {
protected:
    static constexpr Real G = 9.80665;  // m/s²
    static constexpr Real TOL_STRICT = 1e-6;   // For single-step precision
    static constexpr Real TOL_INTEGRATED = 1e-3; // For accumulated multi-step simulations

    void SetUp() override {
        manager_ = std::make_unique<EntityManager>();
    }

    /**
     * @brief Calculate kinetic energy: KE = 0.5*m*v² + 0.5*ω^T*I*ω
     */
    Real kinetic_energy(const EntityState& state) const {
        Real linear_ke = 0.5 * state.mass * state.velocity.dot(state.velocity);

        // Rotational kinetic energy in body frame
        Vec3 L = state.inertia * state.angular_velocity;
        Real rotational_ke = 0.5 * state.angular_velocity.dot(L);

        return linear_ke + rotational_ke;
    }

    /**
     * @brief Calculate gravitational potential energy: PE = m*g*h
     * Using height above z=0 as reference
     */
    Real potential_energy(const EntityState& state, Real g = G) const {
        return state.mass * g * state.position.z;
    }

    /**
     * @brief Calculate total mechanical energy
     */
    Real total_energy(const EntityState& state, Real g = G) const {
        return kinetic_energy(state) + potential_energy(state, g);
    }

    std::unique_ptr<EntityManager> manager_;
};

// ============================================================================
// Free Fall: Potential Energy → Kinetic Energy Conversion
// ============================================================================

/**
 * @brief Test free fall energy conversion
 *
 * Theory:
 * - Initial state: h=100m, v=0 → E = m*g*h
 * - Final state: h=0, v=√(2*g*h) → E = 0.5*m*v²
 * - Energy should be conserved: E_initial = E_final
 */
TEST_F(EnergyConservationTest, FreeFall) {
    EntityState state;
    state.mass = 10.0;  // kg
    state.position = Vec3(0, 0, 100.0);  // 100m height
    state.velocity = Vec3(0, 0, 0);
    state.acceleration = Vec3(0, 0, -G);
    state.inertia = Mat3x3::Identity();

    Real E_initial = total_energy(state);
    EXPECT_NEAR(E_initial, state.mass * G * 100.0, TOL_STRICT);

    // Simulate free fall using RK4
    RK4Integrator integrator;
    EntityForces forces;
    forces.force = Vec3(0, 0, -state.mass * G);  // Gravity

    Real dt = 0.01;  // 10ms timestep
    Real t = 0.0;
    Real t_fall = std::sqrt(2.0 * 100.0 / G);  // Time to hit ground

    while (t < t_fall && state.position.z > 0.0) {
        integrator.integrate(state, forces, dt);
        t += dt;

        // Check energy conservation at each step
        Real E_current = total_energy(state);
        EXPECT_NEAR(E_current, E_initial, TOL_INTEGRATED);
    }

    // Final energy check
    Real E_final = total_energy(state);
    EXPECT_NEAR(E_final, E_initial, TOL_INTEGRATED);

    // Verify final velocity matches theoretical value: v = √(2*g*h)
    Real v_theoretical = std::sqrt(2.0 * G * 100.0);
    EXPECT_NEAR(std::abs(state.velocity.z), v_theoretical, 0.1);
}

// ============================================================================
// Bouncing Ball: Energy Loss from Restitution
// ============================================================================

/**
 * @brief Test bouncing ball with coefficient of restitution
 *
 * Theory:
 * - After bounce: v' = -e*v, where e is coefficient of restitution
 * - Energy loss: ΔE = 0.5*m*v²*(1 - e²)
 */
TEST_F(EnergyConservationTest, BouncingBallEnergyLoss) {
    EntityState state;
    state.mass = 1.0;  // kg
    state.position = Vec3(0, 0, 10.0);  // 10m height
    state.velocity = Vec3(0, 0, 0);
    state.inertia = Mat3x3::Identity();

    Real e = 0.8;  // Coefficient of restitution
    Real E_initial = potential_energy(state);

    // Fall to ground
    Real v_impact = std::sqrt(2.0 * G * 10.0);
    state.velocity = Vec3(0, 0, -v_impact);
    state.position = Vec3(0, 0, 0);

    Real KE_before_bounce = kinetic_energy(state);

    // Apply bounce: v' = -e*v
    state.velocity.z = e * v_impact;

    Real KE_after_bounce = kinetic_energy(state);

    // Energy loss should be ΔE = 0.5*m*v²*(1 - e²)
    Real energy_loss_theoretical = 0.5 * state.mass * v_impact * v_impact * (1.0 - e * e);
    Real energy_loss_actual = KE_before_bounce - KE_after_bounce;

    EXPECT_NEAR(energy_loss_actual, energy_loss_theoretical, TOL_STRICT);

    // After bounce, total energy should be reduced
    Real E_after_bounce = total_energy(state);
    EXPECT_LT(E_after_bounce, E_initial);
    EXPECT_NEAR(E_after_bounce, E_initial * e * e, TOL_INTEGRATED);
}

// ============================================================================
// Spring-Mass System: Oscillatory Energy Exchange
// ============================================================================

/**
 * @brief Test spring-mass oscillator energy conservation
 *
 * Theory:
 * - Total energy: E = 0.5*m*v² + 0.5*k*x²
 * - For undamped oscillator: E = constant
 * - Period: T = 2π*√(m/k)
 */
TEST_F(EnergyConservationTest, SpringMassOscillator) {
    Real m = 1.0;    // kg
    Real k = 100.0;  // N/m
    Real x0 = 0.5;   // Initial displacement (m)

    EntityState state;
    state.mass = m;
    state.position = Vec3(x0, 0, 0);
    state.velocity = Vec3(0, 0, 0);
    state.inertia = Mat3x3::Identity();

    // Initial energy is all potential
    Real E_initial = 0.5 * k * x0 * x0;

    // Theoretical period
    Real T = 2.0 * M_PI * std::sqrt(m / k);

    RK4Integrator integrator;
    Real dt = T / 100.0;  // 100 steps per period
    Real t = 0.0;

    std::vector<Real> energies;

    // Simulate for 3 complete periods
    while (t < 3.0 * T) {
        // Spring force: F = -k*x
        EntityForces forces;
        forces.force = Vec3(-k * state.position.x, 0, 0);

        integrator.integrate(state, forces, dt);
        t += dt;

        // Calculate total energy
        Real KE = kinetic_energy(state);
        Real PE = 0.5 * k * state.position.x * state.position.x;
        Real E_total = KE + PE;

        energies.push_back(E_total);

        // Energy should be conserved
        EXPECT_NEAR(E_total, E_initial, TOL_INTEGRATED);
    }

    // Check energy variance is small
    Real E_mean = 0.0;
    for (Real E : energies) E_mean += E;
    E_mean /= energies.size();

    Real E_variance = 0.0;
    for (Real E : energies) {
        Real diff = E - E_mean;
        E_variance += diff * diff;
    }
    E_variance /= energies.size();

    EXPECT_NEAR(E_mean, E_initial, TOL_INTEGRATED);
    EXPECT_LT(std::sqrt(E_variance), 0.01 * E_initial);  // < 1% variance
}

/**
 * @brief Test spring-mass oscillator period matches theory
 */
TEST_F(EnergyConservationTest, SpringMassPeriod) {
    Real m = 2.0;    // kg
    Real k = 50.0;   // N/m
    Real x0 = 1.0;   // Initial displacement (m)

    EntityState state;
    state.mass = m;
    state.position = Vec3(x0, 0, 0);
    state.velocity = Vec3(0, 0, 0);
    state.inertia = Mat3x3::Identity();

    // Theoretical period: T = 2π√(m/k)
    Real T_theoretical = 2.0 * M_PI * std::sqrt(m / k);

    RK4Integrator integrator;
    Real dt = 0.001;  // 1ms timestep
    Real t = 0.0;

    // Find first zero crossing after positive maximum
    bool found_max = false;
    Real t_zero = 0.0;

    while (t < 2.0 * T_theoretical) {
        EntityForces forces;
        forces.force = Vec3(-k * state.position.x, 0, 0);

        Real x_prev = state.position.x;
        integrator.integrate(state, forces, dt);
        t += dt;
        Real x_curr = state.position.x;

        // Detect maximum (velocity changes sign from + to -)
        if (!found_max && std::abs(state.velocity.x) < 0.01) {
            found_max = true;
        }

        // Detect zero crossing after maximum
        if (found_max && x_prev > 0 && x_curr <= 0) {
            t_zero = t;
            break;
        }
    }

    // Half period should match theoretical value
    Real T_measured = 2.0 * t_zero;
    EXPECT_NEAR(T_measured, T_theoretical, 0.01 * T_theoretical);
}

// ============================================================================
// Integrator Comparison: Energy Drift
// ============================================================================

/**
 * @brief Compare energy conservation across different integrators
 */
TEST_F(EnergyConservationTest, IntegratorComparison) {
    Real m = 1.0;
    Real k = 10.0;
    Real x0 = 1.0;

    EntityState state_rk4, state_euler, state_abm4;
    state_rk4.mass = state_euler.mass = state_abm4.mass = m;
    state_rk4.position = state_euler.position = state_abm4.position = Vec3(x0, 0, 0);
    state_rk4.velocity = state_euler.velocity = state_abm4.velocity = Vec3(0, 0, 0);
    state_rk4.inertia = state_euler.inertia = state_abm4.inertia = Mat3x3::Identity();

    Real E_initial = 0.5 * k * x0 * x0;

    RK4Integrator rk4;
    EulerIntegrator euler;
    ABM4Integrator abm4;

    Real dt = 0.01;
    Real T = 2.0 * M_PI * std::sqrt(m / k);

    Real max_error_rk4 = 0.0;
    Real max_error_euler = 0.0;
    Real max_error_abm4 = 0.0;

    for (Real t = 0; t < 5.0 * T; t += dt) {
        EntityForces forces;

        // RK4
        forces.force = Vec3(-k * state_rk4.position.x, 0, 0);
        rk4.integrate(state_rk4, forces, dt);
        Real E_rk4 = kinetic_energy(state_rk4) + 0.5 * k * state_rk4.position.x * state_rk4.position.x;
        max_error_rk4 = std::max(max_error_rk4, std::abs(E_rk4 - E_initial) / E_initial);

        // Euler
        forces.force = Vec3(-k * state_euler.position.x, 0, 0);
        euler.integrate(state_euler, forces, dt);
        Real E_euler = kinetic_energy(state_euler) + 0.5 * k * state_euler.position.x * state_euler.position.x;
        max_error_euler = std::max(max_error_euler, std::abs(E_euler - E_initial) / E_initial);

        // ABM4
        forces.force = Vec3(-k * state_abm4.position.x, 0, 0);
        abm4.integrate(state_abm4, forces, dt);
        Real E_abm4 = kinetic_energy(state_abm4) + 0.5 * k * state_abm4.position.x * state_abm4.position.x;
        max_error_abm4 = std::max(max_error_abm4, std::abs(E_abm4 - E_initial) / E_initial);
    }

    // RK4 and ABM4 should have much better energy conservation than Euler
    EXPECT_LT(max_error_rk4, 0.01);   // < 1%
    EXPECT_LT(max_error_abm4, 0.01);  // < 1%
    EXPECT_GT(max_error_euler, 0.05); // Euler drifts more

    // RK4 and ABM4 should be comparable
    EXPECT_NEAR(max_error_rk4, max_error_abm4, 0.005);
}

// ============================================================================
// Rotational Energy Conservation
// ============================================================================

/**
 * @brief Test rotational kinetic energy conservation for free rotation
 *
 * Theory:
 * - Torque-free rotation conserves angular momentum and energy
 * - Rotational KE: KE_rot = 0.5 * ω^T * I * ω
 */
TEST_F(EnergyConservationTest, RotationalEnergyConservation) {
    EntityState state;
    state.mass = 1.0;
    state.position = Vec3(0, 0, 0);
    state.velocity = Vec3(0, 0, 0);

    // Cuboid inertia tensor (approximation)
    state.inertia = Mat3x3::Identity();
    state.inertia(0, 0) = 0.1;  // kg·m²
    state.inertia(1, 1) = 0.2;
    state.inertia(2, 2) = 0.15;

    // Initial angular velocity
    state.angular_velocity = Vec3(1.0, 2.0, 0.5);  // rad/s
    state.orientation = Quat(1, 0, 0, 0);

    Real E_rot_initial = kinetic_energy(state);

    RK4Integrator integrator;
    EntityForces forces;  // No forces or torques
    forces.force = Vec3(0, 0, 0);
    forces.torque = Vec3(0, 0, 0);

    Real dt = 0.001;
    for (Real t = 0; t < 10.0; t += dt) {
        integrator.integrate(state, forces, dt);

        Real E_rot_current = kinetic_energy(state);

        // Rotational energy should be conserved (within numerical precision)
        EXPECT_NEAR(E_rot_current, E_rot_initial, TOL_INTEGRATED);
    }
}

} // namespace jaguar::test
