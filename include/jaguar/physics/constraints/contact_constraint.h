#pragma once
/**
 * @file contact_constraint.h
 * @brief Contact and friction constraints for collision response
 *
 * Contact constraints prevent penetration between colliding bodies.
 * Friction constraints resist tangential sliding motion. These are
 * typically generated automatically by the collision detection system.
 */

#include "jaguar/physics/constraints/constraint.h"

namespace jaguar::physics {

/**
 * @brief Contact manifold data
 *
 * Describes the contact geometry between two bodies, potentially
 * with multiple contact points sharing the same normal.
 */
struct ContactManifold {
    Vec3 normal{Vec3::UnitZ()};           ///< Contact normal (from A to B)
    Real penetration{0.0};                 ///< Penetration depth (positive = overlapping)
    Vec3 contact_point_A{Vec3::Zero()};   ///< Contact point on body A (world space)
    Vec3 contact_point_B{Vec3::Zero()};   ///< Contact point on body B (world space)
    Real restitution{0.0};                 ///< Combined restitution coefficient
    Real friction{0.5};                    ///< Combined friction coefficient

    /**
     * @brief Compute combined material properties
     */
    static ContactManifold combine_materials(const ContactManifold& a,
                                              const ContactManifold& b);
};

/**
 * @brief Contact constraint parameters
 */
struct ContactConstraintParams : public ConstraintParams {
    Real restitution{0.0};        ///< Bounce coefficient (0-1)
    Real friction_static{0.5};    ///< Static friction coefficient
    Real friction_dynamic{0.4};   ///< Dynamic friction coefficient
    Real rolling_friction{0.01};  ///< Rolling friction coefficient
    bool enable_friction{true};   ///< Enable friction constraints
    Real restitution_threshold{0.5}; ///< Min velocity for restitution (m/s)
};

/**
 * @brief Contact constraint for collision response
 *
 * Implements a unilateral constraint that prevents penetration while
 * allowing separation. The constraint is active only when bodies are
 * in contact or approaching.
 *
 * Contact Constraint:
 *   C = (p_B - p_A) · n ≥ 0  (non-penetration)
 *   λ ≥ 0                     (can only push, not pull)
 *
 * The impulse magnitude is clamped to [0, ∞) to enforce unilaterality.
 *
 * Restitution is handled via bias:
 *   b = max(0, -e * v_rel · n)  (for separating velocity > threshold)
 *
 * Use Cases:
 * - Ground contact
 * - Object collision response
 * - Vehicle-terrain interaction
 * - Projectile impact
 */
class ContactConstraint : public IConstraint {
public:
    /**
     * @brief Create contact constraint from manifold
     *
     * @param body_A First body
     * @param body_B Second body
     * @param manifold Contact manifold data
     */
    ContactConstraint(const ConstraintBody& body_A, const ConstraintBody& body_B,
                      const ContactManifold& manifold);

    /**
     * @brief Create contact constraint with parameters
     */
    ContactConstraint(const ConstraintBody& body_A, const ConstraintBody& body_B,
                      const ContactManifold& manifold,
                      const ContactConstraintParams& params);

    ~ContactConstraint() override = default;

    ConstraintType type() const override { return ConstraintType::Contact; }
    const std::string& name() const override { return name_; }

    /**
     * @brief Get number of constraint rows
     *
     * 1 for normal contact
     * +2 if friction enabled (tangent directions)
     */
    int num_rows() const override;

    void build_rows(const EntityState* state_A, const EntityState* state_B,
                    Real dt, ConstraintRow* rows) override;

    /**
     * @brief Get contact manifold
     */
    const ContactManifold& manifold() const { return manifold_; }

    /**
     * @brief Update contact manifold
     */
    void set_manifold(const ContactManifold& manifold) { manifold_ = manifold; }

    /**
     * @brief Get contact parameters
     */
    const ContactConstraintParams& contact_params() const { return contact_params_; }

    /**
     * @brief Set contact parameters
     */
    void set_contact_params(const ContactConstraintParams& params) { contact_params_ = params; }

    /**
     * @brief Get normal impulse magnitude from last solve
     */
    Real get_normal_impulse() const { return normal_impulse_; }

    /**
     * @brief Get tangent impulse magnitude from last solve
     */
    Real get_friction_impulse() const { return friction_impulse_; }

    /**
     * @brief Check if contact is separating
     */
    bool is_separating() const { return is_separating_; }

private:
    std::string name_{"Contact"};
    ContactManifold manifold_;
    ContactConstraintParams contact_params_;

    // Computed values
    Vec3 tangent1_{Vec3::UnitX()};
    Vec3 tangent2_{Vec3::UnitY()};
    Real normal_impulse_{0.0};
    Real friction_impulse_{0.0};
    bool is_separating_{false};

    /**
     * @brief Compute tangent directions for friction
     */
    void compute_tangent_directions(const EntityState* state_A, const EntityState* state_B);

    /**
     * @brief Build normal contact row
     */
    void build_normal_row(const EntityState* state_A, const EntityState* state_B,
                          Real dt, ConstraintRow& row);

    /**
     * @brief Build friction rows (2)
     */
    void build_friction_rows(const EntityState* state_A, const EntityState* state_B,
                             Real dt, ConstraintRow* rows);
};

/**
 * @brief Friction constraint (separate from contact)
 *
 * This is an optional separate constraint for more control over friction.
 * By default, friction is handled within ContactConstraint.
 *
 * Friction Constraint (Coulomb model):
 *   |λ_t| ≤ μ * λ_n
 *
 * The friction impulse is clamped based on the normal impulse to
 * implement the Coulomb friction cone.
 */
class FrictionConstraint : public IConstraint {
public:
    /**
     * @brief Create friction constraint
     *
     * @param body_A First body
     * @param body_B Second body
     * @param tangent Friction direction (must be perpendicular to contact normal)
     * @param friction_coefficient Friction coefficient μ
     */
    FrictionConstraint(const ConstraintBody& body_A, const ConstraintBody& body_B,
                       const Vec3& tangent, Real friction_coefficient);

    ~FrictionConstraint() override = default;

    ConstraintType type() const override { return ConstraintType::Friction; }
    const std::string& name() const override { return name_; }
    int num_rows() const override { return 1; }

    void build_rows(const EntityState* state_A, const EntityState* state_B,
                    Real dt, ConstraintRow* rows) override;

    /**
     * @brief Set the normal impulse (from associated contact constraint)
     *
     * This is used to compute the friction limit: |λ_t| ≤ μ * λ_n
     */
    void set_normal_impulse(Real normal_impulse) { normal_impulse_ = normal_impulse; }

    /**
     * @brief Get friction direction
     */
    const Vec3& tangent() const { return tangent_; }

    /**
     * @brief Set friction direction
     */
    void set_tangent(const Vec3& tangent) { tangent_ = tangent.normalized(); }

    /**
     * @brief Get friction coefficient
     */
    Real friction_coefficient() const { return friction_coefficient_; }

    /**
     * @brief Set friction coefficient
     */
    void set_friction_coefficient(Real mu) { friction_coefficient_ = mu; }

private:
    std::string name_{"Friction"};
    Vec3 tangent_;
    Real friction_coefficient_;
    Real normal_impulse_{0.0};
};

} // namespace jaguar::physics
