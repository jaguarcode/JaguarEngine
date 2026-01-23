#pragma once
/**
 * @file ball_socket_constraint.h
 * @brief Ball-and-socket (spherical) joint constraint
 *
 * The ball-socket constraint allows free rotation around all three axes
 * while keeping the anchor points coincident. This is used for joints
 * that need full rotational freedom like hip joints or gimbal mounts.
 */

#include "jaguar/physics/constraints/constraint.h"

namespace jaguar::physics {

/**
 * @brief Ball-socket constraint parameters
 */
struct BallSocketConstraintParams : public ConstraintParams {
    // Cone limit (swing angle limit)
    bool enable_cone_limit{false};   ///< Enable cone limit
    Real cone_half_angle{constants::PI}; ///< Maximum swing angle from reference axis (rad)
    Vec3 cone_axis{Vec3::UnitZ()};   ///< Reference axis for cone in body A frame

    // Twist limit
    bool enable_twist_limit{false};  ///< Enable twist limit around cone axis
    Real min_twist{-constants::PI};  ///< Minimum twist angle (rad)
    Real max_twist{constants::PI};   ///< Maximum twist angle (rad)
};

/**
 * @brief Ball-and-socket (spherical) joint constraint
 *
 * Constrains two bodies so their anchor points remain coincident,
 * while allowing free rotation around all three axes. Optionally,
 * cone limits can restrict the range of rotation.
 *
 * Constraint Equations (3 rows):
 * - Row 0-2: Position constraint (anchor points coincide)
 *   C_pos = p_B - p_A = 0
 *
 * Optional:
 * - Row 3: Cone limit (if enabled and at limit)
 * - Row 4: Twist limit (if enabled and at limit)
 *
 * The position constraint Jacobian is:
 *   J_lin_A = -I (3x3 identity)
 *   J_ang_A = -[r_A×] (skew-symmetric matrix)
 *   J_lin_B = I
 *   J_ang_B = [r_B×]
 *
 * Use Cases:
 * - Hip/shoulder joints in articulated figures
 * - Gimbal mounts
 * - Tow bar connections
 * - Universal joint without drive shaft
 * - Camera stabilization mounts
 */
class BallSocketConstraint : public IConstraint {
public:
    /**
     * @brief Create ball-socket constraint
     *
     * @param body_A First body
     * @param body_B Second body
     */
    BallSocketConstraint(const ConstraintBody& body_A, const ConstraintBody& body_B);

    /**
     * @brief Create ball-socket constraint with parameters
     */
    BallSocketConstraint(const ConstraintBody& body_A, const ConstraintBody& body_B,
                         const BallSocketConstraintParams& params);

    ~BallSocketConstraint() override = default;

    ConstraintType type() const override { return ConstraintType::BallSocket; }
    const std::string& name() const override { return name_; }

    /**
     * @brief Get number of constraint rows
     *
     * Base: 3 (position)
     * +1 if cone limit active
     * +1 if twist limit active
     */
    int num_rows() const override;

    void build_rows(const EntityState* state_A, const EntityState* state_B,
                    Real dt, ConstraintRow* rows) override;

    /**
     * @brief Get ball-socket parameters
     */
    const BallSocketConstraintParams& ball_socket_params() const { return ball_params_; }

    /**
     * @brief Set ball-socket parameters
     */
    void set_ball_socket_params(const BallSocketConstraintParams& params) {
        ball_params_ = params;
    }

    /**
     * @brief Get current swing angle (angle from cone axis)
     */
    Real get_swing_angle() const { return current_swing_angle_; }

    /**
     * @brief Get current twist angle (rotation around cone axis)
     */
    Real get_twist_angle() const { return current_twist_angle_; }

    // ========================================================================
    // Convenience setters
    // ========================================================================

    /**
     * @brief Set cone limit
     *
     * @param half_angle Maximum swing angle from reference axis (rad)
     * @param axis Reference axis in body A frame
     */
    void set_cone_limit(Real half_angle, const Vec3& axis = Vec3::UnitZ()) {
        ball_params_.enable_cone_limit = true;
        ball_params_.cone_half_angle = half_angle;
        ball_params_.cone_axis = axis.normalized();
    }

    /**
     * @brief Disable cone limit
     */
    void disable_cone_limit() { ball_params_.enable_cone_limit = false; }

    /**
     * @brief Set twist limit
     */
    void set_twist_limit(Real min_angle, Real max_angle) {
        ball_params_.enable_twist_limit = true;
        ball_params_.min_twist = min_angle;
        ball_params_.max_twist = max_angle;
    }

    /**
     * @brief Disable twist limit
     */
    void disable_twist_limit() { ball_params_.enable_twist_limit = false; }

private:
    std::string name_{"BallSocket"};
    BallSocketConstraintParams ball_params_;

    // Computed values
    Real current_swing_angle_{0.0};
    Real current_twist_angle_{0.0};

    /**
     * @brief Compute swing and twist angles
     */
    void compute_angles(const EntityState* state_A, const EntityState* state_B);

    /**
     * @brief Build position constraint rows (3)
     */
    void build_position_rows(const EntityState* state_A, const EntityState* state_B,
                             Real dt, ConstraintRow* rows);

    /**
     * @brief Build cone limit row (optional)
     */
    bool build_cone_limit_row(const EntityState* state_A, const EntityState* state_B,
                              Real dt, ConstraintRow& row);

    /**
     * @brief Build twist limit row (optional)
     */
    bool build_twist_limit_row(const EntityState* state_A, const EntityState* state_B,
                               Real dt, ConstraintRow& row);
};

} // namespace jaguar::physics
