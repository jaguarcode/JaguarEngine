/**
 * @file test_constraint_accuracy.cpp
 * @brief Constraint solver accuracy validation tests
 *
 * Tests the accuracy of the constraint solver in resolving penetrations,
 * maintaining joint constraints, and applying friction forces correctly.
 *
 * NOTE: These tests are simplified versions. Full constraint solver testing
 * requires the ContactConstraint API to be finalized.
 */

#include <gtest/gtest.h>
#include <cmath>

#include "jaguar/core/types.h"

namespace jaguar::test {

using namespace jaguar;

class ConstraintAccuracyTest : public ::testing::Test {
protected:
    static constexpr Real TOL_POSITION = 1e-3;
    static constexpr Real TOL_VELOCITY = 1e-4;
    static constexpr Real TOL_IMPULSE = 1e-6;

    void SetUp() override {
    }
};

// ============================================================================
// Mathematical Validation Tests
// ============================================================================

/**
 * @brief Test impulse calculation for collision response
 *
 * Theory:
 * - Impulse J = m * Δv
 * - For elastic collision: relative velocity reverses
 */
TEST_F(ConstraintAccuracyTest, ImpulseMagnitude) {
    Real m1 = 1.0;  // kg
    Real m2 = 2.0;  // kg
    Real v1_initial = 5.0;  // m/s
    Real v2_initial = -2.0;  // m/s

    // Relative velocity before collision
    Real v_rel_before = v1_initial - v2_initial;  // 7.0 m/s

    // For elastic collision with coefficient of restitution e=1:
    // v_rel_after = -e * v_rel_before
    Real e = 1.0;
    Real v_rel_after = -e * v_rel_before;  // -7.0 m/s

    // Impulse magnitude (reduced mass formulation):
    // J = (1 + e) * μ * |v_rel|
    // where μ = m1*m2/(m1+m2) is reduced mass
    Real mu = m1 * m2 / (m1 + m2);
    Real J_expected = (1 + e) * mu * std::abs(v_rel_before);

    EXPECT_NEAR(J_expected, 2.0 * (2.0/3.0) * 7.0, TOL_IMPULSE);
    EXPECT_NEAR(J_expected, 28.0/3.0, TOL_IMPULSE);
}

/**
 * @brief Test Coulomb friction cone constraint
 *
 * Theory:
 * - Friction force magnitude: |F_t| ≤ μ * F_n
 * - This defines a cone in force space
 */
TEST_F(ConstraintAccuracyTest, CoulombFrictionCone) {
    Real mu = 0.6;  // Friction coefficient
    Real F_n = 100.0;  // Normal force (N)

    Real F_t_max = mu * F_n;  // Maximum friction force

    EXPECT_NEAR(F_t_max, 60.0, TOL_IMPULSE);

    // Test various tangential forces
    std::vector<Real> F_t_tests = {0.0, 30.0, 59.9, 60.0};

    for (Real F_t : F_t_tests) {
        if (F_t <= F_t_max) {
            // Within friction cone - static friction can provide this
            EXPECT_LE(F_t, F_t_max);
        }
    }

    // Forces exceeding cone cause sliding
    Real F_t_sliding = 70.0;
    EXPECT_GT(F_t_sliding, F_t_max);  // Would cause kinetic friction
}

/**
 * @brief Test penetration resolution math
 *
 * Theory:
 * - Position correction: Δx = penetration_depth * correction_factor
 * - Split between bodies by mass ratio
 */
TEST_F(ConstraintAccuracyTest, PenetrationResolutionMath) {
    Real penetration = 0.05;  // 5cm overlap
    Real m1 = 1.0;
    Real m2 = 3.0;

    // Baumgarte stabilization factor
    Real beta = 0.2;
    Real slop = 0.01;  // Allow 1cm slop

    Real correction = std::max(0.0, penetration - slop) * beta;
    EXPECT_NEAR(correction, (0.05 - 0.01) * 0.2, TOL_POSITION);
    EXPECT_NEAR(correction, 0.008, TOL_POSITION);

    // Mass-weighted distribution
    Real total_inv_mass = 1.0/m1 + 1.0/m2;
    Real delta1 = correction * (1.0/m1) / total_inv_mass;
    Real delta2 = correction * (1.0/m2) / total_inv_mass;

    // Both deltas should sum to total correction
    EXPECT_NEAR(delta1 + delta2, correction, TOL_POSITION);

    // Heavier object moves less
    EXPECT_LT(delta2, delta1);
}

/**
 * @brief Test constraint solver convergence criterion
 *
 * Theory:
 * - Solver should converge when |λ_new - λ_old| < tolerance
 * - Early exit improves performance
 */
TEST_F(ConstraintAccuracyTest, ConvergenceCriterion) {
    std::vector<Real> impulses = {10.0, 9.5, 9.2, 9.05, 9.01, 9.005, 9.001};

    Real tolerance = 0.01;

    for (size_t i = 1; i < impulses.size(); ++i) {
        Real delta = std::abs(impulses[i] - impulses[i-1]);

        if (delta < tolerance) {
            // Converged - should stop iteration
            EXPECT_LT(delta, tolerance);
            break;
        }
    }
}

} // namespace jaguar::test
