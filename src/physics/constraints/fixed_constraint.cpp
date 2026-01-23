/**
 * @file fixed_constraint.cpp
 * @brief Fixed (weld) joint constraint implementation
 */

#include "jaguar/physics/constraints/fixed_constraint.h"
#include <cmath>

namespace jaguar::physics {

FixedConstraint::FixedConstraint(
    const ConstraintBody& body_A,
    const ConstraintBody& body_B)
    : IConstraint(body_A, body_B) {
}

FixedConstraint::FixedConstraint(
    const ConstraintBody& body_A,
    const ConstraintBody& body_B,
    const FixedConstraintParams& params)
    : IConstraint(body_A, body_B)
    , fixed_params_(params) {
    params_ = params;  // Copy base params
}

void FixedConstraint::capture_current_orientation(const EntityState* state_A, const EntityState* state_B) {
    Quat q_A = state_A ? state_A->orientation : Quat::Identity();
    Quat q_B = state_B ? state_B->orientation : Quat::Identity();

    // Store relative orientation: q_rel = q_A^-1 * q_B
    fixed_params_.relative_orientation = q_A.conjugate() * q_B;
    fixed_params_.use_initial_orientation = true;
    orientation_captured_ = true;
}

void FixedConstraint::build_position_rows(
    const EntityState* state_A,
    const EntityState* state_B,
    Real dt,
    ConstraintRow* rows) {

    // Get world space anchor positions
    Vec3 anchor_A = get_world_anchor_A(state_A);
    Vec3 anchor_B = get_world_anchor_B(state_B);

    // Get anchor offsets in world frame
    Vec3 r_A = state_A ? state_A->orientation.rotate(body_A_.local_anchor) : Vec3::Zero();
    Vec3 r_B = state_B ? state_B->orientation.rotate(body_B_.local_anchor) : Vec3::Zero();

    // Position error
    Vec3 error = anchor_B - anchor_A;

    // ERP and CFM from parameters
    Real erp = params_.stiffness;
    Real cfm = (1.0 - params_.stiffness) * 0.1;

    // Build 3 rows, one for each axis (X, Y, Z)
    Vec3 axes[3] = {Vec3::UnitX(), Vec3::UnitY(), Vec3::UnitZ()};

    for (int i = 0; i < 3; ++i) {
        const Vec3& n = axes[i];
        ConstraintRow& row = rows[i];

        // Jacobian: J = [-n, -(r_A × n), n, (r_B × n)]
        row.J_linear_A = n * (-1.0);
        row.J_angular_A = r_A.cross(n) * (-1.0);
        row.J_linear_B = n;
        row.J_angular_B = r_B.cross(n);

        // Positional error along this axis
        Real pos_error = error.dot(n);

        // Apply slop
        if (std::abs(pos_error) < params_.slop) {
            pos_error = 0.0;
        } else if (pos_error > 0) {
            pos_error -= params_.slop;
        } else {
            pos_error += params_.slop;
        }

        // Baumgarte stabilization
        row.rhs = erp * pos_error / dt;
        row.erp = erp;
        row.cfm = cfm;

        // Equality constraint (no limits)
        row.lower_limit = -std::numeric_limits<Real>::infinity();
        row.upper_limit = std::numeric_limits<Real>::infinity();
    }
}

void FixedConstraint::build_orientation_rows(
    const EntityState* state_A,
    const EntityState* state_B,
    Real dt,
    ConstraintRow* rows) {

    // Get current orientations
    Quat q_A = state_A ? state_A->orientation : Quat::Identity();
    Quat q_B = state_B ? state_B->orientation : Quat::Identity();

    // Capture initial orientation if not done and using initial orientation
    if (fixed_params_.use_initial_orientation && !orientation_captured_) {
        fixed_params_.relative_orientation = q_A.conjugate() * q_B;
        orientation_captured_ = true;
    }

    Quat q_error;
    if (fixed_params_.use_initial_orientation) {
        // Target orientation for B: q_target = q_A * q_rel_initial
        Quat q_target = q_A * fixed_params_.relative_orientation;

        // Orientation error: q_error = q_target^-1 * q_B
        q_error = q_target.conjugate() * q_B;
    } else {
        // Simple case: B should match A's orientation
        q_error = q_A.conjugate() * q_B;
    }

    // Convert to axis-angle (2 * vec part for small angles)
    Vec3 angle_error = Vec3(q_error.x, q_error.y, q_error.z) * 2.0;

    // If w is negative, flip to get shorter path
    if (q_error.w < 0) {
        angle_error = angle_error * (-1.0);
    }

    // ERP and CFM
    Real erp = params_.stiffness;
    Real cfm = (1.0 - params_.stiffness) * 0.1;

    // Build 3 angular constraint rows (X, Y, Z)
    Vec3 axes[3] = {Vec3::UnitX(), Vec3::UnitY(), Vec3::UnitZ()};

    for (int i = 0; i < 3; ++i) {
        const Vec3& axis = axes[i];
        ConstraintRow& row = rows[i];

        row.J_linear_A = Vec3::Zero();
        row.J_linear_B = Vec3::Zero();
        row.J_angular_A = axis * (-1.0);
        row.J_angular_B = axis;

        Real error = angle_error.dot(axis);
        row.rhs = erp * error / dt;
        row.erp = erp;
        row.cfm = cfm;

        row.lower_limit = -std::numeric_limits<Real>::infinity();
        row.upper_limit = std::numeric_limits<Real>::infinity();
    }
}

void FixedConstraint::build_rows(
    const EntityState* state_A,
    const EntityState* state_B,
    Real dt,
    ConstraintRow* rows) {

    // Position constraints (3 rows)
    build_position_rows(state_A, state_B, dt, &rows[0]);

    // Orientation constraints (3 rows)
    build_orientation_rows(state_A, state_B, dt, &rows[3]);
}

} // namespace jaguar::physics
