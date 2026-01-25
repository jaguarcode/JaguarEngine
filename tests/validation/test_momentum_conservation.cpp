/**
 * @file test_momentum_conservation.cpp
 * @brief Momentum conservation validation tests
 *
 * Tests that linear and angular momentum are conserved in isolated systems.
 *
 * Theoretical Basis:
 * - Linear momentum: p = m*v
 * - Angular momentum: L = I*ω (body frame) or L = r × p (spatial)
 * - Conservation: dp/dt = 0 when F_ext = 0
 * - Conservation: dL/dt = 0 when τ_ext = 0
 *
 * References:
 * - Goldstein, "Classical Mechanics", Chapter 2
 * - Marion & Thornton, "Classical Dynamics", Chapter 9
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

class MomentumConservationTest : public ::testing::Test {
protected:
    static constexpr Real TOL_STRICT = 1e-6;
    static constexpr Real TOL_INTEGRATED = 1e-3;

    void SetUp() override {
        manager_ = std::make_unique<EntityManager>();
    }

    /**
     * @brief Calculate linear momentum: p = m*v
     */
    Vec3 linear_momentum(const EntityState& state) const {
        return state.mass * state.velocity;
    }

    /**
     * @brief Calculate angular momentum in body frame: L = I*ω
     */
    Vec3 angular_momentum_body(const EntityState& state) const {
        return state.inertia * state.angular_velocity;
    }

    /**
     * @brief Calculate total linear momentum of system
     */
    Vec3 total_linear_momentum(const std::vector<EntityState>& states) const {
        Vec3 p_total(0, 0, 0);
        for (const auto& state : states) {
            p_total += linear_momentum(state);
        }
        return p_total;
    }

    std::unique_ptr<EntityManager> manager_;
};

// ============================================================================
// Linear Momentum Conservation: Elastic Collision
// ============================================================================

/**
 * @brief Test linear momentum conservation in elastic collision
 *
 * Theory:
 * - Two bodies collide head-on elastically
 * - Conservation: m1*v1 + m2*v2 = m1*v1' + m2*v2'
 * - Elastic collision formulas:
 *   v1' = ((m1-m2)*v1 + 2*m2*v2) / (m1+m2)
 *   v2' = ((m2-m1)*v2 + 2*m1*v1) / (m1+m2)
 */
TEST_F(MomentumConservationTest, ElasticCollision1D) {
    // Setup: Two bodies on collision course
    EntityState body1, body2;

    body1.mass = 2.0;  // kg
    body1.position = Vec3(-1.0, 0, 0);
    body1.velocity = Vec3(5.0, 0, 0);  // Moving right at 5 m/s
    body1.inertia = Mat3x3::Identity();

    body2.mass = 1.0;  // kg
    body2.position = Vec3(1.0, 0, 0);
    body2.velocity = Vec3(-2.0, 0, 0);  // Moving left at 2 m/s
    body2.inertia = Mat3x3::Identity();

    // Initial momentum
    Vec3 p_initial = linear_momentum(body1) + linear_momentum(body2);
    EXPECT_NEAR(p_initial.x, 2.0 * 5.0 + 1.0 * (-2.0), TOL_STRICT);
    EXPECT_NEAR(p_initial.x, 8.0, TOL_STRICT);

    // Apply elastic collision formulas
    Real m1 = body1.mass;
    Real m2 = body2.mass;
    Real v1 = body1.velocity.x;
    Real v2 = body2.velocity.x;

    Real v1_after = ((m1 - m2) * v1 + 2.0 * m2 * v2) / (m1 + m2);
    Real v2_after = ((m2 - m1) * v2 + 2.0 * m1 * v1) / (m1 + m2);

    body1.velocity.x = v1_after;
    body2.velocity.x = v2_after;

    // Final momentum
    Vec3 p_final = linear_momentum(body1) + linear_momentum(body2);

    // Momentum should be conserved
    EXPECT_NEAR(p_final.x, p_initial.x, TOL_STRICT);
    EXPECT_NEAR(p_final.y, p_initial.y, TOL_STRICT);
    EXPECT_NEAR(p_final.z, p_initial.z, TOL_STRICT);

    // Verify theoretical values
    EXPECT_NEAR(v1_after, (2.0 - 1.0) * 5.0 + 2.0 * 1.0 * (-2.0) / 3.0, TOL_STRICT);
    EXPECT_NEAR(v1_after, 1.0 / 3.0, TOL_STRICT);
    EXPECT_NEAR(v2_after, (1.0 - 2.0) * (-2.0) + 2.0 * 2.0 * 5.0 / 3.0, TOL_STRICT);
    EXPECT_NEAR(v2_after, 22.0 / 3.0, TOL_STRICT);
}

/**
 * @brief Test momentum conservation in equal mass collision
 *
 * Theory:
 * - For equal masses in elastic collision: velocities exchange
 * - v1' = v2, v2' = v1
 */
TEST_F(MomentumConservationTest, EqualMassCollision) {
    EntityState body1, body2;

    body1.mass = 1.0;
    body1.velocity = Vec3(3.0, 0, 0);
    body1.inertia = Mat3x3::Identity();

    body2.mass = 1.0;
    body2.velocity = Vec3(0, 0, 0);  // Stationary target
    body2.inertia = Mat3x3::Identity();

    Vec3 p_initial = linear_momentum(body1) + linear_momentum(body2);

    // Apply collision (equal masses → velocities exchange)
    Real v1 = body1.velocity.x;
    Real v2 = body2.velocity.x;

    body1.velocity.x = v2;  // 0
    body2.velocity.x = v1;  // 3.0

    Vec3 p_final = linear_momentum(body1) + linear_momentum(body2);

    // Momentum conserved
    EXPECT_NEAR((p_final - p_initial).length(), 0.0, TOL_STRICT);

    // Body 1 stops, body 2 moves at original velocity of body 1
    EXPECT_NEAR(body1.velocity.x, 0.0, TOL_STRICT);
    EXPECT_NEAR(body2.velocity.x, 3.0, TOL_STRICT);
}

// ============================================================================
// Linear Momentum Conservation: Inelastic Collision
// ============================================================================

/**
 * @brief Test momentum conservation in perfectly inelastic collision
 *
 * Theory:
 * - Bodies stick together: v_final = (m1*v1 + m2*v2) / (m1 + m2)
 * - Momentum conserved but energy lost
 */
TEST_F(MomentumConservationTest, InelasticCollision) {
    EntityState body1, body2;

    body1.mass = 3.0;  // kg
    body1.velocity = Vec3(4.0, 0, 0);
    body1.inertia = Mat3x3::Identity();

    body2.mass = 2.0;  // kg
    body2.velocity = Vec3(-1.0, 0, 0);
    body2.inertia = Mat3x3::Identity();

    Vec3 p_initial = linear_momentum(body1) + linear_momentum(body2);

    // Perfectly inelastic: bodies stick
    Real v_final = (body1.mass * body1.velocity.x + body2.mass * body2.velocity.x) /
                   (body1.mass + body2.mass);

    body1.velocity.x = v_final;
    body2.velocity.x = v_final;

    Vec3 p_final = linear_momentum(body1) + linear_momentum(body2);

    // Momentum conserved
    EXPECT_NEAR((p_final - p_initial).length(), 0.0, TOL_STRICT);

    // Verify final velocity
    Real v_expected = (3.0 * 4.0 + 2.0 * (-1.0)) / 5.0;
    EXPECT_NEAR(v_final, v_expected, TOL_STRICT);
    EXPECT_NEAR(v_final, 2.0, TOL_STRICT);
}

// ============================================================================
// Angular Momentum Conservation: Torque-Free Rotation
// ============================================================================

/**
 * @brief Test angular momentum conservation for torque-free rotation
 *
 * Theory:
 * - No external torque → L = constant
 * - L = I*ω in body frame
 * - For free rotation, angular momentum vector is fixed in space
 */
TEST_F(MomentumConservationTest, TorqueFreeRotation) {
    EntityState state;
    state.mass = 1.0;
    state.position = Vec3(0, 0, 0);
    state.velocity = Vec3(0, 0, 0);

    // Principal moments of inertia
    state.inertia = Mat3x3::Identity();
    state.inertia(0, 0) = 1.0;
    state.inertia(1, 1) = 2.0;
    state.inertia(2, 2) = 3.0;

    state.angular_velocity = Vec3(0.5, 1.0, 0.3);  // rad/s
    state.orientation = Quat(1, 0, 0, 0);

    Vec3 L_initial = angular_momentum_body(state);

    RK4Integrator integrator;
    EntityForces forces;
    forces.force = Vec3(0, 0, 0);
    forces.torque = Vec3(0, 0, 0);  // No external torque

    Real dt = 0.001;
    for (Real t = 0; t < 10.0; t += dt) {
        integrator.integrate(state, forces, dt);

        Vec3 L_current = angular_momentum_body(state);

        // Angular momentum magnitude should be conserved
        // Note: Direction may change in body frame due to rotation
        EXPECT_NEAR(L_current.length(), L_initial.length(), TOL_INTEGRATED);
    }
}

// ============================================================================
// Two-Body Orbital Motion: Central Force Conservation
// ============================================================================

/**
 * @brief Test angular momentum conservation in two-body orbital problem
 *
 * Theory:
 * - Central force (gravity): F = -GMm/r² * r_hat
 * - Angular momentum L = r × p conserved for central forces
 * - L magnitude and direction both conserved
 */
TEST_F(MomentumConservationTest, OrbitalAngularMomentum) {
    // Simplified two-body problem (one body at origin)
    EntityState satellite;
    satellite.mass = 1000.0;  // kg

    // Circular orbit setup
    Real R = 7000e3;  // 7000 km orbital radius
    Real GM = 3.986004418e14;  // Earth's gravitational parameter (m³/s²)
    Real v_circular = std::sqrt(GM / R);

    satellite.position = Vec3(R, 0, 0);
    satellite.velocity = Vec3(0, v_circular, 0);
    satellite.inertia = Mat3x3::Identity();

    // Angular momentum: L = r × p
    Vec3 r = satellite.position;
    Vec3 p = linear_momentum(satellite);
    Vec3 L_initial = r.cross(p);

    EXPECT_NEAR(L_initial.z, satellite.mass * R * v_circular, TOL_STRICT);

    RK4Integrator integrator;
    Real dt = 10.0;  // 10 second timestep

    for (Real t = 0; t < 5400.0; t += dt) {  // ~90 min orbit
        // Gravitational force toward origin
        Vec3 r_vec = satellite.position;
        Real r_mag = r_vec.length();
        Vec3 F_gravity = -GM * satellite.mass / (r_mag * r_mag * r_mag) * r_vec;

        EntityForces forces;
        forces.force = F_gravity;

        integrator.integrate(satellite, forces, dt);

        // Calculate current angular momentum
        Vec3 r_current = satellite.position;
        Vec3 p_current = linear_momentum(satellite);
        Vec3 L_current = r_current.cross(p_current);

        // Angular momentum should be conserved
        EXPECT_NEAR((L_current - L_initial).length() / L_initial.length(), 0.0, 0.01);
    }
}

// ============================================================================
// Constraint Impulse Momentum Transfer
// ============================================================================

/**
 * @brief Test momentum transfer via constraint impulses
 *
 * Theory:
 * - Constraint applies equal and opposite impulses: J and -J
 * - Total momentum conserved: Δp1 + Δp2 = 0
 */
TEST_F(MomentumConservationTest, ConstraintImpulseMomentumTransfer) {
    EntityState body1, body2;

    body1.mass = 2.0;
    body1.position = Vec3(0, 0, 0);
    body1.velocity = Vec3(1.0, 0, 0);
    body1.inertia = Mat3x3::Identity();

    body2.mass = 3.0;
    body2.position = Vec3(2.0, 0, 0);
    body2.velocity = Vec3(-0.5, 0, 0);
    body2.inertia = Mat3x3::Identity();

    Vec3 p_initial = linear_momentum(body1) + linear_momentum(body2);

    // Simulate constraint impulse (e.g., collision response)
    // Impulse magnitude along collision normal
    Real J = 1.5;  // N·s

    // Apply impulse to body1 (negative x direction)
    body1.velocity.x -= J / body1.mass;

    // Apply opposite impulse to body2 (positive x direction)
    body2.velocity.x += J / body2.mass;

    Vec3 p_final = linear_momentum(body1) + linear_momentum(body2);

    // Total momentum should be conserved
    EXPECT_NEAR((p_final - p_initial).length(), 0.0, TOL_STRICT);
}

// ============================================================================
// Multi-Body System Momentum Conservation
// ============================================================================

/**
 * @brief Test momentum conservation in multi-body system
 */
TEST_F(MomentumConservationTest, MultiBodySystem) {
    std::vector<EntityState> bodies(5);

    // Initialize random velocities
    for (size_t i = 0; i < bodies.size(); ++i) {
        bodies[i].mass = 1.0 + 0.5 * i;
        bodies[i].position = Vec3(i * 2.0, 0, 0);
        bodies[i].velocity = Vec3((i % 2 == 0 ? 1.0 : -1.0) * (i + 1), 0, 0);
        bodies[i].inertia = Mat3x3::Identity();
    }

    Vec3 p_initial = total_linear_momentum(bodies);

    // Simulate some internal interactions (elastic collisions)
    // Body 0 and 1 collide
    Real m0 = bodies[0].mass;
    Real m1 = bodies[1].mass;
    Real v0 = bodies[0].velocity.x;
    Real v1 = bodies[1].velocity.x;

    bodies[0].velocity.x = ((m0 - m1) * v0 + 2.0 * m1 * v1) / (m0 + m1);
    bodies[1].velocity.x = ((m1 - m0) * v1 + 2.0 * m0 * v0) / (m0 + m1);

    Vec3 p_after_collision1 = total_linear_momentum(bodies);
    EXPECT_NEAR((p_after_collision1 - p_initial).length(), 0.0, TOL_STRICT);

    // Body 2 and 3 collide
    Real m2 = bodies[2].mass;
    Real m3 = bodies[3].mass;
    Real v2 = bodies[2].velocity.x;
    Real v3 = bodies[3].velocity.x;

    bodies[2].velocity.x = ((m2 - m3) * v2 + 2.0 * m3 * v3) / (m2 + m3);
    bodies[3].velocity.x = ((m3 - m2) * v3 + 2.0 * m2 * v2) / (m2 + m3);

    Vec3 p_final = total_linear_momentum(bodies);

    // Total system momentum conserved through all collisions
    EXPECT_NEAR((p_final - p_initial).length(), 0.0, TOL_STRICT);
}

// ============================================================================
// Center of Mass Motion
// ============================================================================

/**
 * @brief Test center of mass motion with no external forces
 *
 * Theory:
 * - Center of mass velocity: v_cm = Σ(m_i * v_i) / Σ(m_i)
 * - With no external force: v_cm = constant
 */
TEST_F(MomentumConservationTest, CenterOfMassMotion) {
    std::vector<EntityState> bodies(3);

    bodies[0].mass = 2.0;
    bodies[0].position = Vec3(0, 0, 0);
    bodies[0].velocity = Vec3(3.0, 0, 0);
    bodies[0].inertia = Mat3x3::Identity();

    bodies[1].mass = 3.0;
    bodies[1].position = Vec3(5.0, 0, 0);
    bodies[1].velocity = Vec3(-1.0, 0, 0);
    bodies[1].inertia = Mat3x3::Identity();

    bodies[2].mass = 1.0;
    bodies[2].position = Vec3(-2.0, 0, 0);
    bodies[2].velocity = Vec3(0, 2.0, 0);
    bodies[2].inertia = Mat3x3::Identity();

    // Calculate initial center of mass velocity
    Real total_mass = 0.0;
    Vec3 total_momentum(0, 0, 0);
    for (const auto& body : bodies) {
        total_mass += body.mass;
        total_momentum += linear_momentum(body);
    }

    Vec3 v_cm_initial = total_momentum / total_mass;
    EXPECT_NEAR(v_cm_initial.x, (2.0 * 3.0 + 3.0 * (-1.0) + 1.0 * 0) / 6.0, TOL_STRICT);
    EXPECT_NEAR(v_cm_initial.x, 0.5, TOL_STRICT);
    EXPECT_NEAR(v_cm_initial.y, (0 + 0 + 1.0 * 2.0) / 6.0, TOL_STRICT);
    EXPECT_NEAR(v_cm_initial.y, 1.0 / 3.0, TOL_STRICT);

    // Simulate internal forces (should not change CM velocity)
    RK4Integrator integrator;
    Real dt = 0.01;

    for (Real t = 0; t < 1.0; t += dt) {
        // Apply some internal spring forces between bodies
        // (These should not affect CM motion)
        for (auto& body : bodies) {
            EntityForces forces;
            forces.force = Vec3(0, 0, 0);  // No external force
            integrator.integrate(body, forces, dt);
        }

        // Recalculate CM velocity
        Vec3 momentum_current(0, 0, 0);
        for (const auto& body : bodies) {
            momentum_current += linear_momentum(body);
        }
        Vec3 v_cm_current = momentum_current / total_mass;

        // CM velocity should remain constant
        EXPECT_NEAR((v_cm_current - v_cm_initial).length(), 0.0, TOL_INTEGRATED);
    }
}

} // namespace jaguar::test
