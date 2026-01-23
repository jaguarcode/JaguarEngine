/**
 * @file contact_constraint.cpp
 * @brief Contact and friction constraint implementation
 */

#include "jaguar/physics/constraints/contact_constraint.h"
#include <cmath>

namespace jaguar::physics {

// ============================================================================
// ContactManifold
// ============================================================================

ContactManifold ContactManifold::combine_materials(const ContactManifold& a, const ContactManifold& b) {
    ContactManifold result;
    result.normal = a.normal;  // Use first manifold's normal
    result.penetration = (a.penetration + b.penetration) * 0.5;
    result.contact_point_A = (a.contact_point_A + b.contact_point_A) * 0.5;
    result.contact_point_B = (a.contact_point_B + b.contact_point_B) * 0.5;

    // Average restitution and friction
    result.restitution = std::sqrt(a.restitution * b.restitution);  // Geometric mean
    result.friction = std::sqrt(a.friction * b.friction);

    return result;
}

// ============================================================================
// ContactConstraint
// ============================================================================

ContactConstraint::ContactConstraint(
    const ConstraintBody& body_A,
    const ConstraintBody& body_B,
    const ContactManifold& manifold)
    : IConstraint(body_A, body_B)
    , manifold_(manifold) {
}

ContactConstraint::ContactConstraint(
    const ConstraintBody& body_A,
    const ConstraintBody& body_B,
    const ContactManifold& manifold,
    const ContactConstraintParams& params)
    : IConstraint(body_A, body_B)
    , manifold_(manifold)
    , contact_params_(params) {
    params_ = params;  // Copy base params
}

int ContactConstraint::num_rows() const {
    int rows = 1;  // Normal contact

    if (contact_params_.enable_friction) {
        rows += 2;  // Two tangent directions
    }

    return rows;
}

void ContactConstraint::compute_tangent_directions(const EntityState* state_A, const EntityState* state_B) {
    // Get relative velocity at contact point
    Vec3 vel_A = state_A ? state_A->velocity : Vec3::Zero();
    Vec3 vel_B = state_B ? state_B->velocity : Vec3::Zero();
    Vec3 omega_A = state_A ? state_A->angular_velocity : Vec3::Zero();
    Vec3 omega_B = state_B ? state_B->angular_velocity : Vec3::Zero();

    // Anchor offsets from center of mass to contact point
    Vec3 r_A = manifold_.contact_point_A - (state_A ? state_A->position : Vec3::Zero());
    Vec3 r_B = manifold_.contact_point_B - (state_B ? state_B->position : Vec3::Zero());

    // Point velocities at contact
    Vec3 point_vel_A = vel_A + omega_A.cross(r_A);
    Vec3 point_vel_B = vel_B + omega_B.cross(r_B);

    // Relative velocity
    Vec3 rel_vel = point_vel_B - point_vel_A;

    // Tangential velocity (perpendicular to normal)
    Vec3 tangent_vel = rel_vel - manifold_.normal * rel_vel.dot(manifold_.normal);
    Real tangent_speed = tangent_vel.length();

    if (tangent_speed > 1e-10) {
        // Use sliding direction
        tangent1_ = tangent_vel * (1.0 / tangent_speed);
    } else {
        // Build arbitrary tangent basis
        if (std::abs(manifold_.normal.dot(Vec3::UnitX())) < 0.9) {
            tangent1_ = manifold_.normal.cross(Vec3::UnitX()).normalized();
        } else {
            tangent1_ = manifold_.normal.cross(Vec3::UnitY()).normalized();
        }
    }

    // Second tangent perpendicular to both
    tangent2_ = manifold_.normal.cross(tangent1_).normalized();
}

void ContactConstraint::build_normal_row(
    const EntityState* state_A,
    const EntityState* state_B,
    Real dt,
    ConstraintRow& row) {

    // Anchor offsets from center of mass
    Vec3 r_A = manifold_.contact_point_A - (state_A ? state_A->position : Vec3::Zero());
    Vec3 r_B = manifold_.contact_point_B - (state_B ? state_B->position : Vec3::Zero());

    // Normal direction (from A to B)
    const Vec3& n = manifold_.normal;

    // Jacobian: J = [-n, -(r_A × n), n, (r_B × n)]
    row.J_linear_A = n * (-1.0);
    row.J_angular_A = r_A.cross(n) * (-1.0);
    row.J_linear_B = n;
    row.J_angular_B = r_B.cross(n);

    // Compute relative velocity along normal
    Vec3 vel_A = state_A ? state_A->velocity : Vec3::Zero();
    Vec3 vel_B = state_B ? state_B->velocity : Vec3::Zero();
    Vec3 omega_A = state_A ? state_A->angular_velocity : Vec3::Zero();
    Vec3 omega_B = state_B ? state_B->angular_velocity : Vec3::Zero();

    Vec3 point_vel_A = vel_A + omega_A.cross(r_A);
    Vec3 point_vel_B = vel_B + omega_B.cross(r_B);
    Real rel_vel_n = (point_vel_B - point_vel_A).dot(n);

    // ERP for position correction
    Real erp = params_.stiffness;
    Real cfm = (1.0 - params_.stiffness) * 0.1;

    // Position error (penetration depth)
    Real error = manifold_.penetration;

    // Apply slop to reduce jitter
    if (error < params_.slop) {
        error = 0.0;
    } else {
        error -= params_.slop;
    }

    // Baumgarte stabilization
    Real bias = erp * error / dt;

    // Restitution for separating velocity
    if (rel_vel_n < -contact_params_.restitution_threshold) {
        is_separating_ = false;
        // Add restitution bias
        Real restitution_bias = -contact_params_.restitution * rel_vel_n;
        bias += restitution_bias;
    } else {
        is_separating_ = rel_vel_n > 0;
    }

    row.rhs = bias;
    row.erp = erp;
    row.cfm = cfm;

    // Unilateral constraint - can only push, not pull
    row.lower_limit = 0.0;
    row.upper_limit = std::numeric_limits<Real>::infinity();
}

void ContactConstraint::build_friction_rows(
    const EntityState* state_A,
    const EntityState* state_B,
    [[maybe_unused]] Real dt,
    ConstraintRow* rows) {

    // Anchor offsets
    Vec3 r_A = manifold_.contact_point_A - (state_A ? state_A->position : Vec3::Zero());
    Vec3 r_B = manifold_.contact_point_B - (state_B ? state_B->position : Vec3::Zero());

    // Friction limit based on normal impulse (Coulomb friction)
    // This will be updated during solving based on accumulated normal impulse
    Real friction_limit = contact_params_.friction_static * std::abs(normal_impulse_);
    if (friction_limit < 1e-10) {
        // Use a small default if no normal impulse yet
        friction_limit = contact_params_.friction_static * 10.0;  // Heuristic
    }

    // Tangent 1
    {
        ConstraintRow& row = rows[0];

        row.J_linear_A = tangent1_ * (-1.0);
        row.J_angular_A = r_A.cross(tangent1_) * (-1.0);
        row.J_linear_B = tangent1_;
        row.J_angular_B = r_B.cross(tangent1_);

        row.rhs = 0.0;  // Target zero relative tangential velocity
        row.erp = 0.0;
        row.cfm = 0.01;  // Small CFM for stability

        // Friction cone limits
        row.lower_limit = -friction_limit;
        row.upper_limit = friction_limit;
    }

    // Tangent 2
    {
        ConstraintRow& row = rows[1];

        row.J_linear_A = tangent2_ * (-1.0);
        row.J_angular_A = r_A.cross(tangent2_) * (-1.0);
        row.J_linear_B = tangent2_;
        row.J_angular_B = r_B.cross(tangent2_);

        row.rhs = 0.0;
        row.erp = 0.0;
        row.cfm = 0.01;

        row.lower_limit = -friction_limit;
        row.upper_limit = friction_limit;
    }
}

void ContactConstraint::build_rows(
    const EntityState* state_A,
    const EntityState* state_B,
    Real dt,
    ConstraintRow* rows) {

    // Compute tangent directions for friction
    if (contact_params_.enable_friction) {
        compute_tangent_directions(state_A, state_B);
    }

    int row_index = 0;

    // Normal contact (1 row)
    build_normal_row(state_A, state_B, dt, rows[row_index]);
    row_index++;

    // Friction (2 rows)
    if (contact_params_.enable_friction) {
        build_friction_rows(state_A, state_B, dt, &rows[row_index]);
        row_index += 2;
    }
}

// ============================================================================
// FrictionConstraint
// ============================================================================

FrictionConstraint::FrictionConstraint(
    const ConstraintBody& body_A,
    const ConstraintBody& body_B,
    const Vec3& tangent,
    Real friction_coefficient)
    : IConstraint(body_A, body_B)
    , tangent_(tangent.normalized())
    , friction_coefficient_(friction_coefficient) {
}

void FrictionConstraint::build_rows(
    const EntityState* state_A,
    const EntityState* state_B,
    [[maybe_unused]] Real dt,
    ConstraintRow* rows) {

    // Get anchor offsets
    Vec3 r_A = state_A ? state_A->orientation.rotate(body_A_.local_anchor) : Vec3::Zero();
    Vec3 r_B = state_B ? state_B->orientation.rotate(body_B_.local_anchor) : Vec3::Zero();

    ConstraintRow& row = rows[0];

    // Jacobian for tangential constraint
    row.J_linear_A = tangent_ * (-1.0);
    row.J_angular_A = r_A.cross(tangent_) * (-1.0);
    row.J_linear_B = tangent_;
    row.J_angular_B = r_B.cross(tangent_);

    row.rhs = 0.0;  // Target zero relative tangential velocity
    row.erp = 0.0;
    row.cfm = 0.01;

    // Friction limit based on normal impulse (Coulomb friction)
    Real friction_limit = friction_coefficient_ * std::abs(normal_impulse_);
    if (friction_limit < 1e-10) {
        friction_limit = friction_coefficient_ * 10.0;  // Heuristic
    }

    row.lower_limit = -friction_limit;
    row.upper_limit = friction_limit;
}

} // namespace jaguar::physics
