/**
 * @file ground_contact_constraint.cpp
 * @brief Ground contact constraint implementation
 */

#include "jaguar/physics/constraints/ground_contact_constraint.h"
#include "jaguar/environment/terrain.h"
#include <cmath>
#include <algorithm>

namespace jaguar::physics {

// ============================================================================
// GroundContactConstraint
// ============================================================================

GroundContactConstraint::GroundContactConstraint(
    const ConstraintBody& entity,
    const Vec3& contact_point,
    const Vec3& terrain_normal,
    Real penetration_depth)
    : IConstraint(entity, ConstraintBody::Static(Vec3::Zero()))
    , contact_point_(contact_point)
    , terrain_normal_(terrain_normal.normalized())
    , penetration_depth_(penetration_depth) {

    compute_tangent_basis();
}

GroundContactConstraint::GroundContactConstraint(
    const ConstraintBody& entity,
    const Vec3& contact_point,
    const Vec3& terrain_normal,
    Real penetration_depth,
    const GroundContactParams& params)
    : IConstraint(entity, ConstraintBody::Static(Vec3::Zero()))
    , contact_point_(contact_point)
    , terrain_normal_(terrain_normal.normalized())
    , penetration_depth_(penetration_depth)
    , ground_params_(params) {

    params_ = params;  // Copy base params
    compute_tangent_basis();
}

int GroundContactConstraint::num_rows() const {
    int rows = 1;  // Normal contact

    if (ground_params_.enable_friction) {
        rows += 2;  // Two tangent directions
    }

    return rows;
}

void GroundContactConstraint::compute_tangent_basis() {
    // Build orthonormal tangent basis from terrain normal
    // Choose first tangent perpendicular to normal
    if (std::abs(terrain_normal_.dot(Vec3::UnitX())) < 0.9) {
        tangent1_ = terrain_normal_.cross(Vec3::UnitX()).normalized();
    } else {
        tangent1_ = terrain_normal_.cross(Vec3::UnitY()).normalized();
    }

    // Second tangent perpendicular to both normal and first tangent
    tangent2_ = terrain_normal_.cross(tangent1_).normalized();
}

void GroundContactConstraint::update_from_terrain(const environment::TerrainQuery& query) {
    if (query.valid) {
        set_terrain_normal(query.normal);
        set_friction_coefficient(query.material.friction_coefficient);
        set_rolling_friction(query.material.rolling_resistance);
    }
}

void GroundContactConstraint::build_normal_row(
    const EntityState* state,
    Real dt,
    ConstraintRow& row) {

    // Contact point relative to entity center of mass
    Vec3 entity_pos = state ? state->position : Vec3::Zero();
    Vec3 r = contact_point_ - entity_pos;

    // Normal direction (from terrain to entity)
    const Vec3& n = terrain_normal_;

    // Jacobian: J = [n, (r × n), 0, 0] for entity vs static ground
    row.J_linear_A = n;
    row.J_angular_A = r.cross(n);
    row.J_linear_B = Vec3::Zero();
    row.J_angular_B = Vec3::Zero();

    // Compute relative velocity along normal
    Vec3 vel = state ? state->velocity : Vec3::Zero();
    Vec3 omega = state ? state->angular_velocity : Vec3::Zero();
    Vec3 point_vel = vel + omega.cross(r);
    Real rel_vel_n = point_vel.dot(n);

    // ERP for position correction (Baumgarte stabilization)
    Real erp = ground_params_.stiffness;
    Real cfm = (1.0 - ground_params_.stiffness) * 0.1;

    // Position error (penetration depth)
    Real error = penetration_depth_;

    // Apply slop to reduce jitter for small penetrations
    if (error < ground_params_.slop) {
        error = 0.0;
    } else {
        error -= ground_params_.slop;
    }

    // Baumgarte stabilization: bias = (beta / dt) * error
    Real bias = (erp * ground_params_.contact_depth_bias / dt) * error;

    // Restitution for bouncing (only if approaching fast enough)
    if (rel_vel_n < -ground_params_.restitution_threshold) {
        // Add restitution bias to "bounce" the entity
        Real restitution_bias = -ground_params_.restitution * rel_vel_n;
        bias += restitution_bias;
    }

    row.rhs = bias;
    row.erp = erp;
    row.cfm = cfm;

    // Unilateral constraint - ground can only push entity up, not pull down
    row.lower_limit = 0.0;
    row.upper_limit = std::numeric_limits<Real>::infinity();
}

void GroundContactConstraint::build_friction_rows(
    const EntityState* state,
    [[maybe_unused]] Real dt,
    ConstraintRow* rows) {

    // Contact point relative to entity center of mass
    Vec3 entity_pos = state ? state->position : Vec3::Zero();
    Vec3 r = contact_point_ - entity_pos;

    // Friction limit based on normal impulse (Coulomb friction cone)
    // This will be updated during solving as normal impulse accumulates
    Real friction_limit = ground_params_.friction_coefficient * std::abs(normal_impulse_);
    if (friction_limit < 1e-10) {
        // Use a heuristic default if no normal impulse yet
        // This helps with warm starting
        friction_limit = ground_params_.friction_coefficient * 10.0;
    }

    // First tangent direction
    {
        ConstraintRow& row = rows[0];

        row.J_linear_A = tangent1_;
        row.J_angular_A = r.cross(tangent1_);
        row.J_linear_B = Vec3::Zero();
        row.J_angular_B = Vec3::Zero();

        row.rhs = 0.0;  // Target zero relative tangential velocity
        row.erp = 0.0;
        row.cfm = 0.01;  // Small CFM for friction stability

        // Friction cone limits: -μλ_n ≤ λ_t ≤ μλ_n
        row.lower_limit = -friction_limit;
        row.upper_limit = friction_limit;
    }

    // Second tangent direction
    {
        ConstraintRow& row = rows[1];

        row.J_linear_A = tangent2_;
        row.J_angular_A = r.cross(tangent2_);
        row.J_linear_B = Vec3::Zero();
        row.J_angular_B = Vec3::Zero();

        row.rhs = 0.0;
        row.erp = 0.0;
        row.cfm = 0.01;

        row.lower_limit = -friction_limit;
        row.upper_limit = friction_limit;
    }

    // Store friction impulse magnitude for external queries
    friction_impulse_ = 0.0;  // Will be accumulated during solve
}

void GroundContactConstraint::build_rows(
    const EntityState* state_A,
    const EntityState* state_B,
    Real dt,
    ConstraintRow* rows) {

    // Note: state_B is nullptr for ground (static body)
    (void)state_B;  // Suppress unused parameter warning

    int row_index = 0;

    // Normal contact constraint (1 row)
    build_normal_row(state_A, dt, rows[row_index]);

    // Store normal impulse from previous frame for friction calculation
    normal_impulse_ = rows[row_index].accumulated_impulse;
    row_index++;

    // Friction constraints (2 rows)
    if (ground_params_.enable_friction) {
        build_friction_rows(state_A, dt, &rows[row_index]);

        // Accumulate friction impulse from both tangent directions
        friction_impulse_ = std::sqrt(
            rows[row_index].accumulated_impulse * rows[row_index].accumulated_impulse +
            rows[row_index + 1].accumulated_impulse * rows[row_index + 1].accumulated_impulse
        );

        row_index += 2;
    }
}

} // namespace jaguar::physics
