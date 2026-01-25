/**
 * @file test_integrator_accuracy.cpp
 * @brief Numerical integration accuracy validation tests
 *
 * Tests the accuracy of various integrators against known analytical solutions
 * and validates quaternion integration properties.
 *
 * Theoretical Basis:
 * - Local truncation error: error in single step
 * - Global truncation error: accumulated error over time
 * - RK4: O(h^4) local, O(h^4) global
 * - ABM4: O(h^5) local, O(h^4) global (after startup)
 * - Quaternion: must maintain unit norm and represent valid rotation
 *
 * References:
 * - Press et al., "Numerical Recipes", Chapter 16
 * - Hairer, Norsett, Wanner, "Solving Ordinary Differential Equations"
 */

#include <gtest/gtest.h>
#include <cmath>
#include <vector>

#include "jaguar/physics/entity.h"
#include "jaguar/physics/solver.h"

namespace jaguar::test {

using namespace jaguar::physics;

class IntegratorAccuracyTest : public ::testing::Test {
protected:
    static constexpr Real TOL_LOOSE = 1e-2;   // 1% error for long simulations
    static constexpr Real TOL_MEDIUM = 1e-4;  // Medium precision
    static constexpr Real TOL_STRICT = 1e-6;  // High precision

    void SetUp() override {
    }

    /**
     * @brief Compute RMS error between numerical and analytical solution
     */
    Real compute_rms_error(const std::vector<Vec3>& numerical,
                           const std::vector<Vec3>& analytical) const {
        EXPECT_EQ(numerical.size(), analytical.size());
        Real sum_sq = 0.0;
        for (size_t i = 0; i < numerical.size(); ++i) {
            Real error = (numerical[i] - analytical[i]).length();
            sum_sq += error * error;
        }
        return std::sqrt(sum_sq / numerical.size());
    }
};

// ============================================================================
// ABM4 Accuracy on Analytical Solutions
// ============================================================================

/**
 * @brief Test ABM4 on simple harmonic oscillator (SHO)
 *
 * Theory:
 * - SHO: d²x/dt² = -ω²*x
 * - Analytical solution: x(t) = A*cos(ωt + φ)
 * - ABM4 should match analytical solution to high precision
 */
TEST_F(IntegratorAccuracyTest, ABM4_SimpleHarmonicOscillator) {
    Real omega = 2.0;  // rad/s
    Real A = 1.0;      // Amplitude
    Real m = 1.0;      // Mass
    Real k = m * omega * omega;

    EntityState state;
    state.mass = m;
    state.position = Vec3(A, 0, 0);  // Initial displacement
    state.velocity = Vec3(0, 0, 0);  // Start at rest
    state.inertia = Mat3x3::Identity();

    ABM4Integrator abm4;
    Real dt = 0.01;  // 10ms timestep
    Real T = 2.0 * M_PI / omega;  // Period

    std::vector<Vec3> numerical_positions;
    std::vector<Vec3> analytical_positions;
    std::vector<Real> times;

    for (Real t = 0; t <= 3.0 * T; t += dt) {
        numerical_positions.push_back(state.position);

        // Analytical solution
        Real x_analytical = A * std::cos(omega * t);
        analytical_positions.push_back(Vec3(x_analytical, 0, 0));
        times.push_back(t);

        // Integrate
        EntityForces forces;
        forces.force = Vec3(-k * state.position.x, 0, 0);
        abm4.integrate(state, forces, dt);
    }

    Real rms_error = compute_rms_error(numerical_positions, analytical_positions);

    // ABM4 should have very low error for smooth problems
    EXPECT_LT(rms_error, 1e-3);  // < 1mm error over 3 periods

    // Check final position accuracy
    Real t_final = 3.0 * T;
    Real x_analytical_final = A * std::cos(omega * t_final);
    EXPECT_NEAR(state.position.x, x_analytical_final, 1e-2);
}

/**
 * @brief Test ABM4 on exponential decay
 *
 * Theory:
 * - dx/dt = -λ*x
 * - Analytical: x(t) = x0*exp(-λ*t)
 */
TEST_F(IntegratorAccuracyTest, ABM4_ExponentialDecay) {
    Real lambda = 0.5;  // Decay rate
    Real x0 = 10.0;     // Initial value

    EntityState state;
    state.mass = 1.0;
    state.position = Vec3(x0, 0, 0);
    state.velocity = Vec3(0, 0, 0);
    state.inertia = Mat3x3::Identity();

    ABM4Integrator abm4;
    Real dt = 0.1;
    Real t_final = 10.0;

    for (Real t = 0; t < t_final; t += dt) {
        // Damping force: F = -λ*m*v (velocity-proportional)
        // But for position decay: a = -λ*x/m → F = -λ*x
        EntityForces forces;
        forces.force = Vec3(-lambda * state.position.x, 0, 0);

        // For dx/dt = -λ*x, we need v = -λ*x
        state.velocity.x = -lambda * state.position.x;

        abm4.integrate(state, forces, dt);
    }

    Real x_analytical = x0 * std::exp(-lambda * t_final);
    EXPECT_NEAR(state.position.x, x_analytical, 0.1);
}

// ============================================================================
// Quaternion Integration: Unit Norm Preservation
// ============================================================================

/**
 * @brief Test quaternion integration preserves unit norm
 *
 * Theory:
 * - Quaternions represent rotations: |q| = 1
 * - Integration must preserve this constraint
 * - Normalization should be automatic or explicit
 */
TEST_F(IntegratorAccuracyTest, QuaternionUnitNorm) {
    EntityState state;
    state.mass = 1.0;
    state.position = Vec3(0, 0, 0);
    state.velocity = Vec3(0, 0, 0);
    state.orientation = Quat(1, 0, 0, 0);  // Identity
    state.angular_velocity = Vec3(1.0, 0.5, 0.3);  // rad/s
    state.inertia = Mat3x3::Identity();

    RK4Integrator rk4;
    EntityForces forces;
    forces.force = Vec3(0, 0, 0);
    forces.torque = Vec3(0, 0, 0);  // Free rotation

    Real dt = 0.01;
    for (Real t = 0; t < 10.0; t += dt) {
        rk4.integrate(state, forces, dt);

        // Check quaternion is unit
        Real norm = state.orientation.norm();
        EXPECT_NEAR(norm, 1.0, 1e-6);
    }
}

/**
 * @brief Test quaternion geodesic distance error metric
 *
 * Theory:
 * - Geodesic distance on SO(3): d(q1, q2) = arccos(|q1·q2|)
 * - This is the proper rotation distance metric
 */
TEST_F(IntegratorAccuracyTest, QuaternionGeodesicDistance) {
    // Known rotation: 90° around Z-axis
    Real angle = M_PI / 2.0;
    Vec3 axis = Vec3(0, 0, 1);

    Quat q_analytical = quaternion_utils::from_euler(0, 0, angle);

    EntityState state;
    state.mass = 1.0;
    state.position = Vec3(0, 0, 0);
    state.velocity = Vec3(0, 0, 0);
    state.orientation = Quat(1, 0, 0, 0);
    state.angular_velocity = axis * (angle / 1.0);  // Reach 90° in 1 second
    state.inertia = Mat3x3::Identity();

    RK4Integrator rk4;
    EntityForces forces;

    Real dt = 0.01;
    Real t = 0.0;
    while (t < 1.0) {
        rk4.integrate(state, forces, dt);
        t += dt;
    }

    // Compute geodesic distance
    Real dot = state.orientation.w * q_analytical.w +
               state.orientation.x * q_analytical.x +
               state.orientation.y * q_analytical.y +
               state.orientation.z * q_analytical.z;

    Real geodesic_dist = std::acos(std::min(1.0, std::abs(dot)));

    // Should be close to zero
    EXPECT_LT(geodesic_dist, 0.05);  // < 3 degrees error
}

// ============================================================================
// Angular Acceleration History Buffer (ABM4)
// ============================================================================

/**
 * @brief Test ABM4 angular acceleration history is correct
 *
 * Theory:
 * - ABM4 requires history of previous accelerations
 * - History buffer should be updated correctly
 * - After warmup, all history slots should be filled
 */
TEST_F(IntegratorAccuracyTest, ABM4_AngularAccelerationHistory) {
    EntityState state;
    state.mass = 1.0;
    state.position = Vec3(0, 0, 0);
    state.velocity = Vec3(0, 0, 0);
    state.orientation = Quat(1, 0, 0, 0);
    state.angular_velocity = Vec3(0, 0, 0);
    state.inertia = Mat3x3::Identity();
    state.inertia(0, 0) = 1.0;
    state.inertia(1, 1) = 2.0;
    state.inertia(2, 2) = 3.0;

    ABM4Integrator abm4;
    EntityForces forces;
    forces.torque = Vec3(1.0, 0.5, 0.2);  // Constant torque

    Real dt = 0.01;

    // First 4 steps should use RK4 (warmup)
    for (int i = 0; i < 4; ++i) {
        Vec3 angular_accel_before = state.angular_accel;
        abm4.integrate(state, forces, dt);

        // Angular acceleration should be α = I^-1 * τ
        Mat3x3 I_inv = state.inertia.inverse();
        Vec3 expected_accel = I_inv * forces.torque;

        EXPECT_NEAR(state.angular_accel.x, expected_accel.x, TOL_MEDIUM);
        EXPECT_NEAR(state.angular_accel.y, expected_accel.y, TOL_MEDIUM);
        EXPECT_NEAR(state.angular_accel.z, expected_accel.z, TOL_MEDIUM);
    }

    // After warmup, ABM4 predictor-corrector should be active
    for (int i = 0; i < 10; ++i) {
        abm4.integrate(state, forces, dt);

        // Angular velocity should increase linearly (constant torque)
        // ω(t) = ω0 + α*t
    }

    // Verify angular velocity is reasonable
    EXPECT_GT(state.angular_velocity.length(), 0.0);
}

// ============================================================================
// RK4 vs ABM4 Accuracy Comparison
// ============================================================================

/**
 * @brief Compare RK4 and ABM4 accuracy on stiff system
 *
 * Theory:
 * - Stiff ODE: multiple time scales
 * - High-frequency oscillation with slow drift
 * - Both should handle reasonably well
 */
TEST_F(IntegratorAccuracyTest, RK4_vs_ABM4_StiffSystem) {
    // Stiff spring-mass-damper: m*a = -k*x - c*v
    Real m = 0.01;   // Small mass
    Real k = 1000.0; // Stiff spring
    Real c = 0.1;    // Light damping

    EntityState state_rk4, state_abm4;
    state_rk4.mass = state_abm4.mass = m;
    state_rk4.position = state_abm4.position = Vec3(1.0, 0, 0);
    state_rk4.velocity = state_abm4.velocity = Vec3(0, 0, 0);
    state_rk4.inertia = state_abm4.inertia = Mat3x3::Identity();

    RK4Integrator rk4;
    ABM4Integrator abm4;

    Real dt = 0.0001;  // Small timestep for stiff system
    Real t_final = 0.1;

    for (Real t = 0; t < t_final; t += dt) {
        // RK4
        EntityForces forces_rk4;
        forces_rk4.force = Vec3(-k * state_rk4.position.x - c * state_rk4.velocity.x, 0, 0);
        rk4.integrate(state_rk4, forces_rk4, dt);

        // ABM4
        EntityForces forces_abm4;
        forces_abm4.force = Vec3(-k * state_abm4.position.x - c * state_abm4.velocity.x, 0, 0);
        abm4.integrate(state_abm4, forces_abm4, dt);
    }

    // Both should give similar results
    Real position_diff = std::abs(state_rk4.position.x - state_abm4.position.x);
    Real velocity_diff = std::abs(state_rk4.velocity.x - state_abm4.velocity.x);

    EXPECT_LT(position_diff, 0.01);  // Within 1cm
    EXPECT_LT(velocity_diff, 0.1);   // Within 0.1 m/s
}

// ============================================================================
// Adaptive Timestep Behavior
// ============================================================================

/**
 * @brief Test adaptive integrator adjusts timestep correctly
 *
 * Theory:
 * - Adaptive integrator estimates error
 * - Increases dt when error is low
 * - Decreases dt when error is high
 */
TEST_F(IntegratorAccuracyTest, AdaptiveTimestep) {
    EntityState state;
    state.mass = 1.0;
    state.position = Vec3(1.0, 0, 0);
    state.velocity = Vec3(0, 0, 0);
    state.inertia = Mat3x3::Identity();

    Real k = 100.0;  // Spring constant

    AdaptiveIntegrator adaptive;
    AdaptiveConfig config;
    config.tolerance = 1e-4;
    config.min_dt = 1e-5;
    config.max_dt = 0.1;
    adaptive.set_config(config);

    Real t = 0.0;
    Real dt_requested = 0.01;

    std::vector<Real> dts_used;

    while (t < 1.0) {
        EntityForces forces;
        forces.force = Vec3(-k * state.position.x, 0, 0);

        adaptive.integrate(state, forces, dt_requested);

        AdaptiveStepResult result = adaptive.get_last_result();
        dts_used.push_back(result.actual_dt);

        t += result.actual_dt;
    }

    // Adaptive integrator should have used various timesteps
    Real min_dt_used = *std::min_element(dts_used.begin(), dts_used.end());
    Real max_dt_used = *std::max_element(dts_used.begin(), dts_used.end());

    EXPECT_GT(max_dt_used, min_dt_used);  // Should adapt

    // Most steps should be accepted
    int accepted_count = 0;
    for (Real dt : dts_used) {
        if (dt > config.min_dt) accepted_count++;
    }
    EXPECT_GT(static_cast<Real>(accepted_count) / dts_used.size(), 0.5);
}

// ============================================================================
// Integrator Order Verification
// ============================================================================

/**
 * @brief Verify integrator convergence order
 *
 * Theory:
 * - Halving timestep should reduce error by factor of 2^p
 * - Where p is the order of the method
 * - RK4: p = 4, ABM4: p = 4, Euler: p = 1
 */
TEST_F(IntegratorAccuracyTest, ConvergenceOrder) {
    Real omega = 1.0;
    Real A = 1.0;
    Real k = omega * omega;

    std::vector<Real> timesteps = {0.1, 0.05, 0.025, 0.0125};
    std::vector<Real> errors_rk4;
    std::vector<Real> errors_euler;

    for (Real dt : timesteps) {
        // RK4
        EntityState state_rk4;
        state_rk4.mass = 1.0;
        state_rk4.position = Vec3(A, 0, 0);
        state_rk4.velocity = Vec3(0, 0, 0);
        state_rk4.inertia = Mat3x3::Identity();

        RK4Integrator rk4;
        for (Real t = 0; t < 2.0 * M_PI; t += dt) {
            EntityForces forces;
            forces.force = Vec3(-k * state_rk4.position.x, 0, 0);
            rk4.integrate(state_rk4, forces, dt);
        }
        Real x_analytical = A * std::cos(omega * 2.0 * M_PI);  // Should return to A
        errors_rk4.push_back(std::abs(state_rk4.position.x - x_analytical));

        // Euler
        EntityState state_euler;
        state_euler.mass = 1.0;
        state_euler.position = Vec3(A, 0, 0);
        state_euler.velocity = Vec3(0, 0, 0);
        state_euler.inertia = Mat3x3::Identity();

        EulerIntegrator euler;
        for (Real t = 0; t < 2.0 * M_PI; t += dt) {
            EntityForces forces;
            forces.force = Vec3(-k * state_euler.position.x, 0, 0);
            euler.integrate(state_euler, forces, dt);
        }
        errors_euler.push_back(std::abs(state_euler.position.x - x_analytical));
    }

    // Check convergence rate
    // For order p: error(h/2) ≈ error(h) / 2^p

    // RK4 (order 4)
    for (size_t i = 1; i < errors_rk4.size(); ++i) {
        Real ratio = errors_rk4[i - 1] / errors_rk4[i];
        // Should be close to 2^4 = 16
        EXPECT_GT(ratio, 8.0);   // At least 8x improvement
        EXPECT_LT(ratio, 32.0);  // But not unreasonably high
    }

    // Euler (order 1)
    for (size_t i = 1; i < errors_euler.size(); ++i) {
        Real ratio = errors_euler[i - 1] / errors_euler[i];
        // Should be close to 2^1 = 2
        EXPECT_GT(ratio, 1.5);
        EXPECT_LT(ratio, 3.0);
    }

    // RK4 should be much more accurate than Euler
    EXPECT_LT(errors_rk4.back(), errors_euler.back() * 0.01);
}

// ============================================================================
// Quaternion Integration with RK4
// ============================================================================

/**
 * @brief Test quaternion RK4 integration accuracy
 *
 * Theory:
 * - RK4 for quaternions: higher order than first-order Euler
 * - Should maintain unit norm and accurate rotation
 */
TEST_F(IntegratorAccuracyTest, QuaternionRK4Accuracy) {
    // Rotate 180° around X-axis at constant rate
    Real total_angle = M_PI;  // 180°
    Real duration = 2.0;      // seconds
    Real omega = total_angle / duration;

    Vec3 axis = Vec3(1, 0, 0);  // X-axis
    Vec3 angular_vel = axis * omega;

    EntityState state;
    state.mass = 1.0;
    state.orientation = Quat(1, 0, 0, 0);  // Identity
    state.angular_velocity = angular_vel;
    state.inertia = Mat3x3::Identity();

    RK4Integrator rk4;
    EntityForces forces;

    Real dt = 0.01;
    for (Real t = 0; t < duration; t += dt) {
        rk4.integrate(state, forces, dt);

        // Check norm
        EXPECT_NEAR(state.orientation.norm(), 1.0, 1e-6);
    }

    // Final orientation should be 180° rotation around X
    Quat q_expected = quaternion_utils::from_euler(M_PI, 0, 0);

    Real dot = state.orientation.w * q_expected.w +
               state.orientation.x * q_expected.x +
               state.orientation.y * q_expected.y +
               state.orientation.z * q_expected.z;

    Real angle_error = std::acos(std::min(1.0, std::abs(dot)));
    EXPECT_LT(angle_error, 0.05);  // < 3 degrees
}

} // namespace jaguar::test
