#pragma once
/**
 * @file ground_contact_constraint.h
 * @brief Ground contact constraint using terrain normals and friction
 *
 * This constraint is specialized for entity-terrain interactions. Unlike
 * the generic ContactConstraint, it integrates with the environment system
 * to use accurate terrain normals and material-specific friction coefficients.
 */

#include "jaguar/physics/constraints/constraint.h"

namespace jaguar::environment {
    struct TerrainQuery;  // Forward declaration
}

namespace jaguar::physics {

/**
 * @brief Ground contact constraint parameters
 */
struct GroundContactParams : public ConstraintParams {
    Real restitution{0.0};              ///< Bounce coefficient (0-1)
    Real friction_coefficient{0.8};     ///< Base friction coefficient
    Real rolling_friction{0.01};        ///< Rolling resistance coefficient
    bool enable_friction{true};         ///< Enable friction constraints
    Real restitution_threshold{0.5};    ///< Min velocity for restitution (m/s)
    Real contact_depth_bias{0.1};       ///< Baumgarte bias multiplier
};

/**
 * @brief Ground contact constraint for terrain interaction
 *
 * This constraint prevents penetration into terrain and applies friction
 * based on surface properties. It uses terrain normals from the environment
 * system rather than assuming vertical gravity.
 *
 * Key Features:
 * - Uses actual terrain normal (not always vertical)
 * - Applies surface-specific friction from terrain material
 * - Handles penetration depth with Baumgarte stabilization
 * - Supports restitution for bouncing
 * - Friction cone model (Coulomb friction)
 *
 * Constraint Formulation:
 *   Normal: C = (p_entity - p_terrain) · n ≥ 0
 *   where n is the terrain normal at contact point
 *
 * Friction:
 *   |λ_t| ≤ μ * λ_n  (Coulomb friction cone)
 *
 * Use Cases:
 * - Vehicle-ground contact
 * - Entity landing on terrain
 * - Ground vehicle traction
 * - Wheeled/tracked vehicle dynamics
 */
class GroundContactConstraint : public IConstraint {
public:
    /**
     * @brief Create ground contact constraint
     *
     * @param entity Entity in contact with ground
     * @param contact_point Contact point in world space
     * @param terrain_normal Normal vector from terrain query
     * @param penetration_depth How deep entity penetrates terrain (>0 = penetrating)
     */
    GroundContactConstraint(const ConstraintBody& entity,
                           const Vec3& contact_point,
                           const Vec3& terrain_normal,
                           Real penetration_depth);

    /**
     * @brief Create ground contact with parameters
     */
    GroundContactConstraint(const ConstraintBody& entity,
                           const Vec3& contact_point,
                           const Vec3& terrain_normal,
                           Real penetration_depth,
                           const GroundContactParams& params);

    ~GroundContactConstraint() override = default;

    ConstraintType type() const override { return ConstraintType::Contact; }
    const std::string& name() const override { return name_; }

    /**
     * @brief Get number of constraint rows
     *
     * Returns:
     * - 1 row for normal constraint
     * - +2 rows if friction enabled (tangent directions)
     */
    int num_rows() const override;

    void build_rows(const EntityState* state_A, const EntityState* state_B,
                    Real dt, ConstraintRow* rows) override;

    /**
     * @brief Set terrain normal from environment query
     */
    void set_terrain_normal(const Vec3& normal) {
        terrain_normal_ = normal.normalized();
        compute_tangent_basis();
    }

    /**
     * @brief Set friction coefficient from terrain material
     */
    void set_friction_coefficient(Real mu) {
        ground_params_.friction_coefficient = mu;
    }

    /**
     * @brief Set rolling friction from terrain material
     */
    void set_rolling_friction(Real mu) {
        ground_params_.rolling_friction = mu;
    }

    /**
     * @brief Update from terrain query
     *
     * Convenience method to update all terrain properties at once
     */
    void update_from_terrain(const environment::TerrainQuery& query);

    /**
     * @brief Set contact point in world space
     */
    void set_contact_point(const Vec3& point) { contact_point_ = point; }

    /**
     * @brief Set penetration depth
     */
    void set_penetration_depth(Real depth) { penetration_depth_ = depth; }

    /**
     * @brief Get terrain normal
     */
    const Vec3& terrain_normal() const { return terrain_normal_; }

    /**
     * @brief Get contact point
     */
    const Vec3& contact_point() const { return contact_point_; }

    /**
     * @brief Get penetration depth
     */
    Real penetration_depth() const { return penetration_depth_; }

    /**
     * @brief Get ground contact parameters
     */
    const GroundContactParams& ground_params() const { return ground_params_; }

    /**
     * @brief Set ground contact parameters
     */
    void set_ground_params(const GroundContactParams& params) {
        ground_params_ = params;
        params_ = params;  // Update base params
    }

    /**
     * @brief Get normal impulse magnitude from last solve
     */
    Real get_normal_impulse() const { return normal_impulse_; }

    /**
     * @brief Get friction impulse magnitude from last solve
     */
    Real get_friction_impulse() const { return friction_impulse_; }

    /**
     * @brief Check if contact is active (penetrating or approaching)
     */
    bool is_active() const { return penetration_depth_ > -0.01; }

private:
    std::string name_{"GroundContact"};
    Vec3 contact_point_{Vec3::Zero()};      ///< Contact point in world space
    Vec3 terrain_normal_{Vec3::UnitZ()};    ///< Terrain normal (from environment)
    Real penetration_depth_{0.0};           ///< Penetration depth (positive = penetrating)
    GroundContactParams ground_params_;

    // Tangent basis (computed from terrain normal)
    Vec3 tangent1_{Vec3::UnitX()};
    Vec3 tangent2_{Vec3::UnitY()};

    // Computed values from last solve
    Real normal_impulse_{0.0};
    Real friction_impulse_{0.0};

    /**
     * @brief Compute tangent basis from terrain normal
     *
     * Constructs two perpendicular tangent vectors for friction constraints.
     */
    void compute_tangent_basis();

    /**
     * @brief Build normal contact row (prevents penetration)
     */
    void build_normal_row(const EntityState* state, Real dt, ConstraintRow& row);

    /**
     * @brief Build friction rows (2 tangent directions)
     */
    void build_friction_rows(const EntityState* state, Real dt, ConstraintRow* rows);
};

} // namespace jaguar::physics
