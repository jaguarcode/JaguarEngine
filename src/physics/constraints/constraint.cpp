/**
 * @file constraint.cpp
 * @brief Base constraint implementation
 */

#include "jaguar/physics/constraints/constraint.h"
#include <cmath>

namespace jaguar::physics {

// ============================================================================
// ConstraintRow Implementation
// ============================================================================

void ConstraintRow::compute_effective_mass(
    Real inv_mass_A, const Mat3x3& inv_inertia_A,
    Real inv_mass_B, const Mat3x3& inv_inertia_B) {

    // Effective mass = 1 / (J * M^-1 * J^T + CFM)
    //
    // J * M^-1 * J^T = J_lin_A^T * (1/m_A) * J_lin_A
    //                + J_ang_A^T * I_A^-1 * J_ang_A
    //                + J_lin_B^T * (1/m_B) * J_lin_B
    //                + J_ang_B^T * I_B^-1 * J_ang_B

    // Linear terms
    Real jmj = 0.0;
    jmj += J_linear_A.dot(J_linear_A) * inv_mass_A;
    jmj += J_linear_B.dot(J_linear_B) * inv_mass_B;

    // Angular terms: J_ang^T * I^-1 * J_ang = (I^-1 * J_ang) · J_ang
    Vec3 Iinv_Jang_A = inv_inertia_A * J_angular_A;
    Vec3 Iinv_Jang_B = inv_inertia_B * J_angular_B;
    jmj += J_angular_A.dot(Iinv_Jang_A);
    jmj += J_angular_B.dot(Iinv_Jang_B);

    // Add CFM for softness
    jmj += cfm;

    // Effective mass (avoid division by zero)
    if (jmj > 1e-10) {
        effective_mass = 1.0 / jmj;
    } else {
        effective_mass = 0.0;
    }
}

// ============================================================================
// IConstraint Implementation
// ============================================================================

void IConstraint::set_enabled(bool enabled) {
    if (enabled) {
        flags_ = flags_ | ConstraintFlags::Enabled;
    } else {
        flags_ = static_cast<ConstraintFlags>(
            static_cast<UInt32>(flags_) & ~static_cast<UInt32>(ConstraintFlags::Enabled));
    }
}

void IConstraint::apply_impulse(const ConstraintRow* rows,
                                 EntityState* state_A, EntityState* state_B) {
    // Default implementation: apply impulses based on Jacobians
    // J = [J_lin_A, J_ang_A, J_lin_B, J_ang_B]
    // Δv = M^-1 * J^T * λ

    int n = num_rows();
    Real total_force = 0.0;
    Real total_torque = 0.0;

    for (int i = 0; i < n; ++i) {
        const ConstraintRow& row = rows[i];
        Real lambda = row.accumulated_impulse;

        // Apply linear impulse to body A (negative Jacobian)
        if (state_A && state_A->mass > 1e-10) {
            Real inv_mass_A = 1.0 / state_A->mass;
            state_A->velocity = state_A->velocity - row.J_linear_A * (lambda * inv_mass_A);

            // Apply angular impulse to body A
            Mat3x3 inv_inertia_A = state_A->inertia.inverse();
            Vec3 angular_impulse_A = inv_inertia_A * row.J_angular_A * lambda;
            state_A->angular_velocity = state_A->angular_velocity - angular_impulse_A;

            total_force += row.J_linear_A.length() * std::abs(lambda);
            total_torque += row.J_angular_A.length() * std::abs(lambda);
        }

        // Apply linear impulse to body B (positive Jacobian)
        if (state_B && state_B->mass > 1e-10) {
            Real inv_mass_B = 1.0 / state_B->mass;
            state_B->velocity = state_B->velocity + row.J_linear_B * (lambda * inv_mass_B);

            // Apply angular impulse to body B
            Mat3x3 inv_inertia_B = state_B->inertia.inverse();
            Vec3 angular_impulse_B = inv_inertia_B * row.J_angular_B * lambda;
            state_B->angular_velocity = state_B->angular_velocity + angular_impulse_B;

            total_force += row.J_linear_B.length() * std::abs(lambda);
            total_torque += row.J_angular_B.length() * std::abs(lambda);
        }
    }

    // Update force/torque magnitudes for breakage detection
    force_magnitude_ = total_force;
    torque_magnitude_ = total_torque;

    // Check for breakage
    if (params_.break_force > 0.0 && force_magnitude_ > params_.break_force) {
        flags_ = flags_ | ConstraintFlags::Broken;
    }
    if (params_.break_torque > 0.0 && torque_magnitude_ > params_.break_torque) {
        flags_ = flags_ | ConstraintFlags::Broken;
    }
}

Vec3 IConstraint::get_world_anchor_A(const EntityState* state) const {
    if (body_A_.is_static()) {
        return body_A_.local_anchor;  // Static anchor is already in world space
    }
    if (!state) {
        return body_A_.local_anchor;
    }
    // Transform local anchor to world space
    return state->position + state->orientation.rotate(body_A_.local_anchor);
}

Vec3 IConstraint::get_world_anchor_B(const EntityState* state) const {
    if (body_B_.is_static()) {
        return body_B_.local_anchor;
    }
    if (!state) {
        return body_B_.local_anchor;
    }
    return state->position + state->orientation.rotate(body_B_.local_anchor);
}

Vec3 IConstraint::get_world_axis_A(const EntityState* state) const {
    if (body_A_.is_static()) {
        return body_A_.local_axis;
    }
    if (!state) {
        return body_A_.local_axis;
    }
    return state->orientation.rotate(body_A_.local_axis);
}

Vec3 IConstraint::get_world_axis_B(const EntityState* state) const {
    if (body_B_.is_static()) {
        return body_B_.local_axis;
    }
    if (!state) {
        return body_B_.local_axis;
    }
    return state->orientation.rotate(body_B_.local_axis);
}

} // namespace jaguar::physics
