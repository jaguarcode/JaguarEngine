/**
 * @file distance_constraint.cpp
 * @brief Distance constraint implementation
 */

#include "jaguar/physics/constraints/distance_constraint.h"
#include <cmath>

namespace jaguar::physics {

DistanceConstraint::DistanceConstraint(
    const ConstraintBody& body_A,
    const ConstraintBody& body_B,
    Real distance)
    : IConstraint(body_A, body_B) {
    distance_params_.distance = distance;
}

DistanceConstraint::DistanceConstraint(
    const ConstraintBody& body_A,
    const ConstraintBody& body_B,
    const DistanceConstraintParams& params)
    : IConstraint(body_A, body_B)
    , distance_params_(params) {
    params_ = params;  // Copy base params
}

void DistanceConstraint::build_rows(
    const EntityState* state_A,
    const EntityState* state_B,
    Real dt,
    ConstraintRow* rows) {

    // Get world space anchor positions
    Vec3 anchor_A = get_world_anchor_A(state_A);
    Vec3 anchor_B = get_world_anchor_B(state_B);

    // Vector from A to B
    Vec3 delta = anchor_B - anchor_A;
    current_distance_ = std::sqrt(delta.length_squared());

    // Avoid division by zero
    Vec3 n;
    if (current_distance_ > 1e-10) {
        n = delta * (1.0 / current_distance_);
    } else {
        n = Vec3::UnitX();  // Arbitrary direction for degenerate case
    }

    // Compute local anchor offsets in world frame
    Vec3 r_A = state_A ? state_A->orientation.rotate(body_A_.local_anchor) : Vec3::Zero();
    Vec3 r_B = state_B ? state_B->orientation.rotate(body_B_.local_anchor) : Vec3::Zero();

    // Build constraint row
    ConstraintRow& row = rows[0];

    // Jacobian: J = [-n, -(r_A × n), n, (r_B × n)]
    row.J_linear_A = n * (-1.0);
    row.J_angular_A = r_A.cross(n) * (-1.0);
    row.J_linear_B = n;
    row.J_angular_B = r_B.cross(n);

    // Determine target distance based on limits
    Real target_distance = distance_params_.distance;
    bool use_limits = (distance_params_.min_distance > 0.0 ||
                       distance_params_.max_distance > 0.0);

    if (use_limits) {
        Real min_d = distance_params_.min_distance;
        Real max_d = distance_params_.max_distance;

        if (current_distance_ < min_d) {
            // Below minimum - push apart
            target_distance = min_d;
            row.lower_limit = 0.0;  // Can only push
            row.upper_limit = std::numeric_limits<Real>::infinity();
        } else if (max_d > 0.0 && current_distance_ > max_d) {
            // Above maximum - pull together
            target_distance = max_d;
            row.lower_limit = -std::numeric_limits<Real>::infinity();
            row.upper_limit = 0.0;  // Can only pull
        } else {
            // Within limits - no constraint needed
            row.lower_limit = 0.0;
            row.upper_limit = 0.0;
            row.rhs = 0.0;
            return;
        }
    }

    // Constraint error: C = current_distance - target_distance
    Real error = current_distance_ - target_distance;

    // Apply slop to reduce jitter
    if (std::abs(error) < params_.slop) {
        error = 0.0;
    } else if (error > 0) {
        error -= params_.slop;
    } else {
        error += params_.slop;
    }

    // Compute relative velocity along constraint direction
    Real velocity_error = 0.0;
    if (state_A) {
        velocity_error -= n.dot(state_A->velocity);
        velocity_error -= r_A.cross(n).dot(state_A->angular_velocity);
    }
    if (state_B) {
        velocity_error += n.dot(state_B->velocity);
        velocity_error += r_B.cross(n).dot(state_B->angular_velocity);
    }

    // Right-hand side: bias = erp * error / dt
    // For soft constraints, cfm adds compliance
    Real erp = params_.stiffness;  // Use stiffness as ERP
    row.erp = erp;
    row.cfm = (1.0 - params_.stiffness) * 0.1;  // CFM from softness

    // Baumgarte stabilization
    Real bias = erp * error / dt;

    // Add damping
    bias -= params_.damping * velocity_error;

    row.rhs = bias;

    // Initialize impulse limits (equality constraint)
    if (!use_limits) {
        row.lower_limit = -std::numeric_limits<Real>::infinity();
        row.upper_limit = std::numeric_limits<Real>::infinity();
    }
}

} // namespace jaguar::physics
