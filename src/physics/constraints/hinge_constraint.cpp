/**
 * @file hinge_constraint.cpp
 * @brief Hinge (revolute) joint constraint implementation
 */

#include "jaguar/physics/constraints/hinge_constraint.h"
#include <cmath>

namespace jaguar::physics {

HingeConstraint::HingeConstraint(
    const ConstraintBody& body_A,
    const ConstraintBody& body_B)
    : IConstraint(body_A, body_B) {
    // Initialize perpendicular axes based on hinge axis
    Vec3 axis = body_A_.local_axis.normalized();
    if (std::abs(axis.dot(Vec3::UnitX())) < 0.9) {
        perp_A_ = axis.cross(Vec3::UnitX()).normalized();
    } else {
        perp_A_ = axis.cross(Vec3::UnitY()).normalized();
    }
    perp_B_ = axis.cross(perp_A_).normalized();
}

HingeConstraint::HingeConstraint(
    const ConstraintBody& body_A,
    const ConstraintBody& body_B,
    const HingeConstraintParams& params)
    : IConstraint(body_A, body_B)
    , hinge_params_(params) {
    params_ = params;  // Copy base params

    // Initialize perpendicular axes
    Vec3 axis = body_A_.local_axis.normalized();
    if (std::abs(axis.dot(Vec3::UnitX())) < 0.9) {
        perp_A_ = axis.cross(Vec3::UnitX()).normalized();
    } else {
        perp_A_ = axis.cross(Vec3::UnitY()).normalized();
    }
    perp_B_ = axis.cross(perp_A_).normalized();
}

int HingeConstraint::num_rows() const {
    int rows = 5;  // Base: 3 position + 2 orientation

    // Angle limits (only one row, for lower or upper)
    if (hinge_params_.enable_limits) {
        rows += 1;
    }

    // Motor
    if (hinge_params_.enable_motor) {
        rows += 1;
    }

    return rows;
}

void HingeConstraint::compute_angle(const EntityState* state_A, const EntityState* state_B) {
    // Get world space hinge axis from body A
    Vec3 axis_A = get_world_axis_A(state_A);

    // Get world space perpendicular axes (both transformed using perp_A_ for angle computation)
    Vec3 world_perp_A = state_A ? state_A->orientation.rotate(perp_A_) : perp_A_;
    Vec3 world_perp_B = state_B ? state_B->orientation.rotate(perp_A_) : perp_A_;

    // Compute angle using atan2 for full range
    Real sin_angle = world_perp_A.cross(world_perp_B).dot(axis_A);
    Real cos_angle = world_perp_A.dot(world_perp_B);
    current_angle_ = std::atan2(sin_angle, cos_angle);

    // Compute angular velocity about hinge axis
    Vec3 omega_A = state_A ? state_A->angular_velocity : Vec3::Zero();
    Vec3 omega_B = state_B ? state_B->angular_velocity : Vec3::Zero();
    current_angular_velocity_ = (omega_B - omega_A).dot(axis_A);
}

void HingeConstraint::build_position_rows(
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

void HingeConstraint::build_orientation_rows(
    const EntityState* state_A,
    const EntityState* state_B,
    Real dt,
    ConstraintRow* rows) {

    // Get world space axes
    Vec3 axis_A = get_world_axis_A(state_A);

    // Perpendicular axes in world space from body B
    Vec3 world_perp_B_1 = state_B ? state_B->orientation.rotate(perp_A_) : perp_A_;
    Vec3 world_perp_B_2 = state_B ? state_B->orientation.rotate(perp_B_) : perp_B_;

    // ERP and CFM
    Real erp = params_.stiffness;
    Real cfm = (1.0 - params_.stiffness) * 0.1;

    // Row 0: Constraint that perp_A × axis_B is orthogonal to axis_A
    // This ensures the hinge axes are aligned
    {
        ConstraintRow& row = rows[0];

        // Jacobian for angular constraint
        // The derivative of (axis_A × axis_B) with respect to rotation
        row.J_linear_A = Vec3::Zero();
        row.J_linear_B = Vec3::Zero();
        row.J_angular_A = world_perp_B_1 * (-1.0);  // Effect of rotating A
        row.J_angular_B = world_perp_B_1;           // Effect of rotating B

        // Angular error: how much the perpendicular axis has deviated
        Real error = axis_A.dot(world_perp_B_1);

        row.rhs = erp * error / dt;
        row.erp = erp;
        row.cfm = cfm;
        row.lower_limit = -std::numeric_limits<Real>::infinity();
        row.upper_limit = std::numeric_limits<Real>::infinity();
    }

    // Row 1: Second perpendicular constraint
    {
        ConstraintRow& row = rows[1];

        row.J_linear_A = Vec3::Zero();
        row.J_linear_B = Vec3::Zero();
        row.J_angular_A = world_perp_B_2 * (-1.0);
        row.J_angular_B = world_perp_B_2;

        Real error = axis_A.dot(world_perp_B_2);

        row.rhs = erp * error / dt;
        row.erp = erp;
        row.cfm = cfm;
        row.lower_limit = -std::numeric_limits<Real>::infinity();
        row.upper_limit = std::numeric_limits<Real>::infinity();
    }
}

bool HingeConstraint::build_limit_row(
    const EntityState* state_A,
    [[maybe_unused]] const EntityState* state_B,
    Real dt,
    ConstraintRow& row) {

    if (!hinge_params_.enable_limits) {
        return false;
    }

    // Check if at limit
    bool at_lower = current_angle_ <= hinge_params_.lower_limit;
    bool at_upper = current_angle_ >= hinge_params_.upper_limit;

    if (!at_lower && !at_upper) {
        return false;  // Not at limit
    }

    // Get world space hinge axis
    Vec3 axis_A = get_world_axis_A(state_A);

    // Jacobian: angular constraint about hinge axis
    row.J_linear_A = Vec3::Zero();
    row.J_linear_B = Vec3::Zero();
    row.J_angular_A = axis_A * (-1.0);
    row.J_angular_B = axis_A;

    Real erp = params_.stiffness;
    Real cfm = (1.0 - params_.stiffness) * 0.1;

    if (at_lower) {
        Real error = current_angle_ - hinge_params_.lower_limit;
        row.rhs = erp * error / dt;
        row.lower_limit = 0.0;  // Can only push
        row.upper_limit = std::numeric_limits<Real>::infinity();
    } else {
        Real error = current_angle_ - hinge_params_.upper_limit;
        row.rhs = erp * error / dt;
        row.lower_limit = -std::numeric_limits<Real>::infinity();
        row.upper_limit = 0.0;  // Can only pull
    }

    row.erp = erp;
    row.cfm = cfm;

    return true;
}

void HingeConstraint::build_motor_row(
    const EntityState* state_A,
    [[maybe_unused]] const EntityState* state_B,
    [[maybe_unused]] Real dt,
    ConstraintRow& row) {

    // Get world space hinge axis
    Vec3 axis_A = get_world_axis_A(state_A);

    // Jacobian: angular constraint about hinge axis
    row.J_linear_A = Vec3::Zero();
    row.J_linear_B = Vec3::Zero();
    row.J_angular_A = axis_A * (-1.0);
    row.J_angular_B = axis_A;

    // Motor tries to achieve target velocity
    Real velocity_error = hinge_params_.motor_velocity - current_angular_velocity_;
    row.rhs = velocity_error;

    // CFM for motor (allows some slip)
    row.erp = 0.0;
    row.cfm = 0.01;

    // Torque limits
    row.lower_limit = -hinge_params_.max_motor_torque;
    row.upper_limit = hinge_params_.max_motor_torque;
}

void HingeConstraint::build_rows(
    const EntityState* state_A,
    const EntityState* state_B,
    Real dt,
    ConstraintRow* rows) {

    // Compute current angle and angular velocity
    compute_angle(state_A, state_B);

    int row_index = 0;

    // Position constraints (3 rows)
    build_position_rows(state_A, state_B, dt, &rows[row_index]);
    row_index += 3;

    // Orientation constraints (2 rows)
    build_orientation_rows(state_A, state_B, dt, &rows[row_index]);
    row_index += 2;

    // Angle limit (optional, 1 row)
    if (hinge_params_.enable_limits) {
        if (!build_limit_row(state_A, state_B, dt, rows[row_index])) {
            // Not at limit, zero out the row
            rows[row_index] = ConstraintRow{};
        }
        row_index++;
    }

    // Motor (optional, 1 row)
    if (hinge_params_.enable_motor) {
        build_motor_row(state_A, state_B, dt, rows[row_index]);
        row_index++;
    }
}

} // namespace jaguar::physics
