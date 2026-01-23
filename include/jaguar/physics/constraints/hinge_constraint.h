#pragma once
/**
 * @file hinge_constraint.h
 * @brief Hinge (revolute) joint constraint
 *
 * The hinge constraint allows rotation around a single axis while preventing
 * all other relative motion. This is one of the most common joint types,
 * used for doors, wheels, propellers, and articulated mechanisms.
 */

#include "jaguar/physics/constraints/constraint.h"

namespace jaguar::physics {

/**
 * @brief Hinge constraint parameters
 */
struct HingeConstraintParams : public ConstraintParams {
    // Angle limits
    bool enable_limits{false};     ///< Enable angle limits
    Real lower_limit{-constants::PI}; ///< Minimum angle (rad)
    Real upper_limit{constants::PI};  ///< Maximum angle (rad)

    // Motor
    bool enable_motor{false};      ///< Enable motor
    Real motor_velocity{0.0};      ///< Target angular velocity (rad/s)
    Real max_motor_torque{0.0};    ///< Maximum motor torque (N·m)

    // Spring (for soft limits)
    bool enable_spring{false};     ///< Enable spring behavior
    Real spring_stiffness{0.0};    ///< Spring stiffness (N·m/rad)
    Real spring_damping{0.0};      ///< Spring damping (N·m·s/rad)
    Real spring_rest_angle{0.0};   ///< Rest angle for spring (rad)
};

/**
 * @brief Hinge (revolute) joint constraint
 *
 * Constrains two bodies to rotate relative to each other around a single axis.
 * The constraint removes 5 degrees of freedom, leaving only rotation about
 * the hinge axis.
 *
 * Constraint Equations (5 rows):
 * - Row 0-2: Position constraint (anchor points coincide)
 *   C_pos = p_B - p_A = 0
 *
 * - Row 3-4: Orientation constraint (perpendicular axes align)
 *   C_rot1 = (axis_A × perp1_B) · axis_B = 0
 *   C_rot2 = (axis_A × perp2_B) · axis_B = 0
 *
 * Optional:
 * - Row 5: Angle limit (if enabled and at limit)
 * - Row 6: Motor (if enabled)
 *
 * Use Cases:
 * - Door hinges
 * - Vehicle wheels
 * - Robot arm joints
 * - Propeller/rotor attachments
 * - Landing gear rotation
 * - Control surface deflection
 */
class HingeConstraint : public IConstraint {
public:
    /**
     * @brief Create hinge constraint
     *
     * @param body_A First body (with hinge axis)
     * @param body_B Second body
     */
    HingeConstraint(const ConstraintBody& body_A, const ConstraintBody& body_B);

    /**
     * @brief Create hinge constraint with parameters
     */
    HingeConstraint(const ConstraintBody& body_A, const ConstraintBody& body_B,
                    const HingeConstraintParams& params);

    ~HingeConstraint() override = default;

    ConstraintType type() const override { return ConstraintType::Hinge; }
    const std::string& name() const override { return name_; }

    /**
     * @brief Get number of constraint rows
     *
     * Base: 5 (3 position + 2 orientation)
     * +1 if angle limit active
     * +1 if motor enabled
     */
    int num_rows() const override;

    void build_rows(const EntityState* state_A, const EntityState* state_B,
                    Real dt, ConstraintRow* rows) override;

    /**
     * @brief Get hinge parameters
     */
    const HingeConstraintParams& hinge_params() const { return hinge_params_; }

    /**
     * @brief Set hinge parameters
     */
    void set_hinge_params(const HingeConstraintParams& params) { hinge_params_ = params; }

    /**
     * @brief Get current hinge angle (relative to initial orientation)
     */
    Real get_angle() const { return current_angle_; }

    /**
     * @brief Get current angular velocity about hinge axis
     */
    Real get_angular_velocity() const { return current_angular_velocity_; }

    // ========================================================================
    // Convenience setters
    // ========================================================================

    /**
     * @brief Set angle limits
     */
    void set_limits(Real lower, Real upper) {
        hinge_params_.enable_limits = true;
        hinge_params_.lower_limit = lower;
        hinge_params_.upper_limit = upper;
    }

    /**
     * @brief Disable angle limits
     */
    void disable_limits() { hinge_params_.enable_limits = false; }

    /**
     * @brief Set motor parameters
     */
    void set_motor(Real target_velocity, Real max_torque) {
        hinge_params_.enable_motor = true;
        hinge_params_.motor_velocity = target_velocity;
        hinge_params_.max_motor_torque = max_torque;
    }

    /**
     * @brief Disable motor
     */
    void disable_motor() { hinge_params_.enable_motor = false; }

    /**
     * @brief Set spring parameters
     */
    void set_spring(Real stiffness, Real damping, Real rest_angle = 0.0) {
        hinge_params_.enable_spring = true;
        hinge_params_.spring_stiffness = stiffness;
        hinge_params_.spring_damping = damping;
        hinge_params_.spring_rest_angle = rest_angle;
    }

private:
    std::string name_{"Hinge"};
    HingeConstraintParams hinge_params_;

    // Computed values
    Real current_angle_{0.0};
    Real current_angular_velocity_{0.0};
    Vec3 perp_A_{Vec3::UnitY()};  // Perpendicular axes for orientation constraint
    Vec3 perp_B_{Vec3::UnitZ()};

    /**
     * @brief Compute current hinge angle
     */
    void compute_angle(const EntityState* state_A, const EntityState* state_B);

    /**
     * @brief Build position constraint rows (3)
     */
    void build_position_rows(const EntityState* state_A, const EntityState* state_B,
                             Real dt, ConstraintRow* rows);

    /**
     * @brief Build orientation constraint rows (2)
     */
    void build_orientation_rows(const EntityState* state_A, const EntityState* state_B,
                                Real dt, ConstraintRow* rows);

    /**
     * @brief Build angle limit row (optional)
     */
    bool build_limit_row(const EntityState* state_A, const EntityState* state_B,
                         Real dt, ConstraintRow& row);

    /**
     * @brief Build motor row (optional)
     */
    void build_motor_row(const EntityState* state_A, const EntityState* state_B,
                         Real dt, ConstraintRow& row);
};

} // namespace jaguar::physics
