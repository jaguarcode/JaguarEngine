/**
 * @file slider_constraint.cpp
 * @brief Slider (prismatic) joint constraint implementation
 */

#include "jaguar/physics/constraints/slider_constraint.h"
#include <cmath>

namespace jaguar::physics {

SliderConstraint::SliderConstraint(
    const ConstraintBody& body_A,
    const ConstraintBody& body_B)
    : IConstraint(body_A, body_B) {
}

SliderConstraint::SliderConstraint(
    const ConstraintBody& body_A,
    const ConstraintBody& body_B,
    const SliderConstraintParams& params)
    : IConstraint(body_A, body_B)
    , slider_params_(params) {
    params_ = params;  // Copy base params
}

int SliderConstraint::num_rows() const {
    int rows = 5;  // Base: 2 position (perpendicular) + 3 orientation

    // Position limit (1 row)
    if (slider_params_.enable_limits) {
        rows += 1;
    }

    // Motor (1 row)
    if (slider_params_.enable_motor) {
        rows += 1;
    }

    return rows;
}

void SliderConstraint::capture_current_orientation(const EntityState* state_A, const EntityState* state_B) {
    Quat q_A = state_A ? state_A->orientation : Quat::Identity();
    Quat q_B = state_B ? state_B->orientation : Quat::Identity();

    // Store relative orientation: q_rel = q_A^-1 * q_B
    initial_relative_orientation_ = q_A.conjugate() * q_B;
    orientation_captured_ = true;
}

void SliderConstraint::compute_position(const EntityState* state_A, const EntityState* state_B) {
    // Get world space anchor positions
    Vec3 anchor_A = get_world_anchor_A(state_A);
    Vec3 anchor_B = get_world_anchor_B(state_B);

    // Get world space slider axis
    Vec3 axis = get_world_axis_A(state_A);

    // Delta from A to B
    Vec3 delta = anchor_B - anchor_A;

    // Position along slider axis
    current_position_ = delta.dot(axis);

    // Velocity along slider axis
    Vec3 vel_A = state_A ? state_A->velocity : Vec3::Zero();
    Vec3 vel_B = state_B ? state_B->velocity : Vec3::Zero();
    Vec3 omega_A = state_A ? state_A->angular_velocity : Vec3::Zero();
    Vec3 omega_B = state_B ? state_B->angular_velocity : Vec3::Zero();

    Vec3 r_A = state_A ? state_A->orientation.rotate(body_A_.local_anchor) : Vec3::Zero();
    Vec3 r_B = state_B ? state_B->orientation.rotate(body_B_.local_anchor) : Vec3::Zero();

    // Point velocities at anchors
    Vec3 point_vel_A = vel_A + omega_A.cross(r_A);
    Vec3 point_vel_B = vel_B + omega_B.cross(r_B);

    current_velocity_ = (point_vel_B - point_vel_A).dot(axis);
}

void SliderConstraint::build_position_rows(
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

    // Get world space slider axis
    Vec3 axis = get_world_axis_A(state_A);

    // Position error
    Vec3 error = anchor_B - anchor_A;

    // Build perpendicular axes
    Vec3 perp1, perp2;
    if (std::abs(axis.dot(Vec3::UnitX())) < 0.9) {
        perp1 = axis.cross(Vec3::UnitX()).normalized();
    } else {
        perp1 = axis.cross(Vec3::UnitY()).normalized();
    }
    perp2 = axis.cross(perp1).normalized();

    // ERP and CFM from parameters
    Real erp = params_.stiffness;
    Real cfm = (1.0 - params_.stiffness) * 0.1;

    Vec3 perps[2] = {perp1, perp2};

    for (int i = 0; i < 2; ++i) {
        const Vec3& n = perps[i];
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

        // Equality constraint
        row.lower_limit = -std::numeric_limits<Real>::infinity();
        row.upper_limit = std::numeric_limits<Real>::infinity();
    }
}

void SliderConstraint::build_orientation_rows(
    const EntityState* state_A,
    const EntityState* state_B,
    Real dt,
    ConstraintRow* rows) {

    // Get current orientations
    Quat q_A = state_A ? state_A->orientation : Quat::Identity();
    Quat q_B = state_B ? state_B->orientation : Quat::Identity();

    // Capture initial orientation if not done
    if (!orientation_captured_) {
        initial_relative_orientation_ = q_A.conjugate() * q_B;
        orientation_captured_ = true;
    }

    // Target orientation for B: q_target = q_A * q_rel_initial
    Quat q_target = q_A * initial_relative_orientation_;

    // Orientation error: q_error = q_target^-1 * q_B
    Quat q_error = q_target.conjugate() * q_B;

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

bool SliderConstraint::build_limit_row(
    const EntityState* state_A,
    [[maybe_unused]] const EntityState* state_B,
    Real dt,
    ConstraintRow& row) {

    if (!slider_params_.enable_limits) {
        return false;
    }

    // Check if at limit
    bool at_lower = current_position_ <= slider_params_.lower_limit;
    bool at_upper = current_position_ >= slider_params_.upper_limit;

    if (!at_lower && !at_upper) {
        return false;  // Not at limit
    }

    // Get world space slider axis
    Vec3 axis = get_world_axis_A(state_A);

    // Get anchor offsets
    Vec3 r_A = state_A ? state_A->orientation.rotate(body_A_.local_anchor) : Vec3::Zero();
    Vec3 r_B = state_A ? state_A->orientation.rotate(body_B_.local_anchor) : Vec3::Zero();

    // Jacobian for linear constraint along slider axis
    row.J_linear_A = axis * (-1.0);
    row.J_angular_A = r_A.cross(axis) * (-1.0);
    row.J_linear_B = axis;
    row.J_angular_B = r_B.cross(axis);

    Real erp = params_.stiffness;
    Real cfm = (1.0 - params_.stiffness) * 0.1;

    if (at_lower) {
        Real error = current_position_ - slider_params_.lower_limit;
        row.rhs = erp * error / dt;
        row.lower_limit = 0.0;  // Can only push
        row.upper_limit = std::numeric_limits<Real>::infinity();
    } else {
        Real error = current_position_ - slider_params_.upper_limit;
        row.rhs = erp * error / dt;
        row.lower_limit = -std::numeric_limits<Real>::infinity();
        row.upper_limit = 0.0;  // Can only pull
    }

    row.erp = erp;
    row.cfm = cfm;

    return true;
}

void SliderConstraint::build_motor_row(
    const EntityState* state_A,
    [[maybe_unused]] const EntityState* state_B,
    [[maybe_unused]] Real dt,
    ConstraintRow& row) {

    // Get world space slider axis
    Vec3 axis = get_world_axis_A(state_A);

    // Get anchor offsets
    Vec3 r_A = state_A ? state_A->orientation.rotate(body_A_.local_anchor) : Vec3::Zero();
    Vec3 r_B = state_A ? state_A->orientation.rotate(body_B_.local_anchor) : Vec3::Zero();

    // Jacobian for linear constraint along slider axis
    row.J_linear_A = axis * (-1.0);
    row.J_angular_A = r_A.cross(axis) * (-1.0);
    row.J_linear_B = axis;
    row.J_angular_B = r_B.cross(axis);

    // Motor tries to achieve target velocity
    Real velocity_error = slider_params_.motor_velocity - current_velocity_;
    row.rhs = velocity_error;

    // CFM for motor (allows some slip)
    row.erp = 0.0;
    row.cfm = 0.01;

    // Force limits
    row.lower_limit = -slider_params_.max_motor_force;
    row.upper_limit = slider_params_.max_motor_force;
}

void SliderConstraint::build_rows(
    const EntityState* state_A,
    const EntityState* state_B,
    Real dt,
    ConstraintRow* rows) {

    // Compute current position and velocity
    compute_position(state_A, state_B);

    int row_index = 0;

    // Position constraints (2 rows - perpendicular to slider axis)
    build_position_rows(state_A, state_B, dt, &rows[row_index]);
    row_index += 2;

    // Orientation constraints (3 rows)
    build_orientation_rows(state_A, state_B, dt, &rows[row_index]);
    row_index += 3;

    // Position limit (optional, 1 row)
    if (slider_params_.enable_limits) {
        if (!build_limit_row(state_A, state_B, dt, rows[row_index])) {
            // Not at limit, zero out the row
            rows[row_index] = ConstraintRow{};
        }
        row_index++;
    }

    // Motor (optional, 1 row)
    if (slider_params_.enable_motor) {
        build_motor_row(state_A, state_B, dt, rows[row_index]);
        row_index++;
    }
}

} // namespace jaguar::physics
