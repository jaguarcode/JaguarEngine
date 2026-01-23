/**
 * @file ball_socket_constraint.cpp
 * @brief Ball-and-socket (spherical) joint constraint implementation
 */

#include "jaguar/physics/constraints/ball_socket_constraint.h"
#include <cmath>

namespace jaguar::physics {

BallSocketConstraint::BallSocketConstraint(
    const ConstraintBody& body_A,
    const ConstraintBody& body_B)
    : IConstraint(body_A, body_B) {
}

BallSocketConstraint::BallSocketConstraint(
    const ConstraintBody& body_A,
    const ConstraintBody& body_B,
    const BallSocketConstraintParams& params)
    : IConstraint(body_A, body_B)
    , ball_params_(params) {
    params_ = params;  // Copy base params
}

int BallSocketConstraint::num_rows() const {
    int rows = 3;  // Base: 3 position

    // Cone limit (1 row)
    if (ball_params_.enable_cone_limit) {
        rows += 1;
    }

    // Twist limit (1 row)
    if (ball_params_.enable_twist_limit) {
        rows += 1;
    }

    return rows;
}

void BallSocketConstraint::compute_angles(const EntityState* state_A, const EntityState* state_B) {
    // Get world space cone axis from body A
    Vec3 cone_axis_A = state_A
        ? state_A->orientation.rotate(ball_params_.cone_axis)
        : ball_params_.cone_axis;

    // Get world space axis from body B (using the same reference axis)
    Vec3 cone_axis_B = state_B
        ? state_B->orientation.rotate(ball_params_.cone_axis)
        : ball_params_.cone_axis;

    // Compute swing angle (angle between the two cone axes)
    Real dot = cone_axis_A.dot(cone_axis_B);
    dot = std::clamp(dot, -1.0, 1.0);  // Numerical safety
    current_swing_angle_ = std::acos(dot);

    // Compute twist angle
    // Project body B's axis onto the plane perpendicular to cone_axis_A
    // and measure the rotation
    if (current_swing_angle_ < 1e-6) {
        // Axes are nearly aligned, compute twist directly
        // Use a reference perpendicular axis
        Vec3 ref_A = state_A
            ? state_A->orientation.rotate(Vec3::UnitX())
            : Vec3::UnitX();
        Vec3 ref_B = state_B
            ? state_B->orientation.rotate(Vec3::UnitX())
            : Vec3::UnitX();

        // Project onto plane perpendicular to cone axis
        ref_A = ref_A - cone_axis_A * cone_axis_A.dot(ref_A);
        ref_B = ref_B - cone_axis_A * cone_axis_A.dot(ref_B);

        if (ref_A.length_squared() > 1e-10 && ref_B.length_squared() > 1e-10) {
            ref_A = ref_A.normalized();
            ref_B = ref_B.normalized();
            Real sin_twist = ref_A.cross(ref_B).dot(cone_axis_A);
            Real cos_twist = ref_A.dot(ref_B);
            current_twist_angle_ = std::atan2(sin_twist, cos_twist);
        } else {
            current_twist_angle_ = 0.0;
        }
    } else {
        // Swing-twist decomposition
        // This is a simplified version using axis-angle decomposition
        current_twist_angle_ = 0.0;  // Simplified
    }
}

void BallSocketConstraint::build_position_rows(
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

bool BallSocketConstraint::build_cone_limit_row(
    const EntityState* state_A,
    const EntityState* state_B,
    Real dt,
    ConstraintRow& row) {

    if (!ball_params_.enable_cone_limit) {
        return false;
    }

    // Check if at cone limit
    if (current_swing_angle_ < ball_params_.cone_half_angle) {
        return false;  // Not at limit
    }

    // Get world space cone axes
    Vec3 cone_axis_A = state_A
        ? state_A->orientation.rotate(ball_params_.cone_axis)
        : ball_params_.cone_axis;
    Vec3 cone_axis_B = state_B
        ? state_B->orientation.rotate(ball_params_.cone_axis)
        : ball_params_.cone_axis;

    // Swing axis (perpendicular to both cone axes)
    Vec3 swing_axis = cone_axis_A.cross(cone_axis_B);
    Real swing_len = swing_axis.length();
    if (swing_len < 1e-10) {
        // Axes are parallel, pick arbitrary perpendicular
        if (std::abs(cone_axis_A.dot(Vec3::UnitX())) < 0.9) {
            swing_axis = cone_axis_A.cross(Vec3::UnitX()).normalized();
        } else {
            swing_axis = cone_axis_A.cross(Vec3::UnitY()).normalized();
        }
    } else {
        swing_axis = swing_axis * (1.0 / swing_len);
    }

    // Jacobian: angular constraint about swing axis
    row.J_linear_A = Vec3::Zero();
    row.J_linear_B = Vec3::Zero();
    row.J_angular_A = swing_axis * (-1.0);
    row.J_angular_B = swing_axis;

    Real erp = params_.stiffness;
    Real cfm = (1.0 - params_.stiffness) * 0.1;

    // Error: how much we've exceeded the limit
    Real error = current_swing_angle_ - ball_params_.cone_half_angle;

    row.rhs = erp * error / dt;
    row.erp = erp;
    row.cfm = cfm;

    // Unilateral constraint - can only push back
    row.lower_limit = -std::numeric_limits<Real>::infinity();
    row.upper_limit = 0.0;

    return true;
}

bool BallSocketConstraint::build_twist_limit_row(
    const EntityState* state_A,
    [[maybe_unused]] const EntityState* state_B,
    Real dt,
    ConstraintRow& row) {

    if (!ball_params_.enable_twist_limit) {
        return false;
    }

    // Check if at twist limit
    bool at_min = current_twist_angle_ <= ball_params_.min_twist;
    bool at_max = current_twist_angle_ >= ball_params_.max_twist;

    if (!at_min && !at_max) {
        return false;  // Not at limit
    }

    // Get world space cone axis (twist axis)
    Vec3 twist_axis = state_A
        ? state_A->orientation.rotate(ball_params_.cone_axis)
        : ball_params_.cone_axis;

    // Jacobian: angular constraint about twist axis
    row.J_linear_A = Vec3::Zero();
    row.J_linear_B = Vec3::Zero();
    row.J_angular_A = twist_axis * (-1.0);
    row.J_angular_B = twist_axis;

    Real erp = params_.stiffness;
    Real cfm = (1.0 - params_.stiffness) * 0.1;

    if (at_min) {
        Real error = current_twist_angle_ - ball_params_.min_twist;
        row.rhs = erp * error / dt;
        row.lower_limit = 0.0;  // Can only push
        row.upper_limit = std::numeric_limits<Real>::infinity();
    } else {
        Real error = current_twist_angle_ - ball_params_.max_twist;
        row.rhs = erp * error / dt;
        row.lower_limit = -std::numeric_limits<Real>::infinity();
        row.upper_limit = 0.0;  // Can only pull
    }

    row.erp = erp;
    row.cfm = cfm;

    return true;
}

void BallSocketConstraint::build_rows(
    const EntityState* state_A,
    const EntityState* state_B,
    Real dt,
    ConstraintRow* rows) {

    // Compute current angles
    compute_angles(state_A, state_B);

    int row_index = 0;

    // Position constraints (3 rows)
    build_position_rows(state_A, state_B, dt, &rows[row_index]);
    row_index += 3;

    // Cone limit (optional, 1 row)
    if (ball_params_.enable_cone_limit) {
        if (!build_cone_limit_row(state_A, state_B, dt, rows[row_index])) {
            // Not at limit, zero out the row
            rows[row_index] = ConstraintRow{};
        }
        row_index++;
    }

    // Twist limit (optional, 1 row)
    if (ball_params_.enable_twist_limit) {
        if (!build_twist_limit_row(state_A, state_B, dt, rows[row_index])) {
            // Not at limit, zero out the row
            rows[row_index] = ConstraintRow{};
        }
        row_index++;
    }
}

} // namespace jaguar::physics
