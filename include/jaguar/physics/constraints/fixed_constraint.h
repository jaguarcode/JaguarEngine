#pragma once
/**
 * @file fixed_constraint.h
 * @brief Fixed (weld) joint constraint
 *
 * The fixed constraint rigidly connects two bodies, preventing all
 * relative motion. This is used for welding parts together or creating
 * compound rigid bodies from multiple entities.
 */

#include "jaguar/physics/constraints/constraint.h"

namespace jaguar::physics {

/**
 * @brief Fixed constraint parameters
 */
struct FixedConstraintParams : public ConstraintParams {
    // The fixed constraint can optionally store the initial relative
    // orientation to maintain a specific angular offset
    bool use_initial_orientation{true};
    Quat relative_orientation{Quat::Identity()};
};

/**
 * @brief Fixed (weld) joint constraint
 *
 * Constrains two bodies to have no relative motion - both position and
 * orientation are locked. This removes all 6 degrees of freedom.
 *
 * Constraint Equations (6 rows):
 * - Row 0-2: Position constraint (anchor points coincide)
 *   C_pos = p_B - p_A = 0
 *
 * - Row 3-5: Orientation constraint (relative orientation maintained)
 *   C_rot = 2 * vec(q_error) = 0
 *   where q_error = q_A^-1 * q_B * q_initial^-1
 *
 * Use Cases:
 * - Welding parts together
 * - Creating compound rigid bodies
 * - Temporary rigid connections
 * - Breakable joints (with break_force/torque)
 */
class FixedConstraint : public IConstraint {
public:
    /**
     * @brief Create fixed constraint
     *
     * @param body_A First body
     * @param body_B Second body
     */
    FixedConstraint(const ConstraintBody& body_A, const ConstraintBody& body_B);

    /**
     * @brief Create fixed constraint with parameters
     */
    FixedConstraint(const ConstraintBody& body_A, const ConstraintBody& body_B,
                    const FixedConstraintParams& params);

    ~FixedConstraint() override = default;

    ConstraintType type() const override { return ConstraintType::Fixed; }
    const std::string& name() const override { return name_; }
    int num_rows() const override { return 6; }

    void build_rows(const EntityState* state_A, const EntityState* state_B,
                    Real dt, ConstraintRow* rows) override;

    /**
     * @brief Get fixed constraint parameters
     */
    const FixedConstraintParams& fixed_params() const { return fixed_params_; }

    /**
     * @brief Set fixed constraint parameters
     */
    void set_fixed_params(const FixedConstraintParams& params) { fixed_params_ = params; }

    /**
     * @brief Store current relative orientation as the target
     *
     * Call this after positioning bodies to lock their current relative pose.
     *
     * @param state_A State of body A
     * @param state_B State of body B
     */
    void capture_current_orientation(const EntityState* state_A, const EntityState* state_B);

    /**
     * @brief Set relative orientation directly
     */
    void set_relative_orientation(const Quat& q) {
        fixed_params_.relative_orientation = q;
        fixed_params_.use_initial_orientation = true;
    }

    /**
     * @brief Get relative orientation target
     */
    const Quat& get_relative_orientation() const { return fixed_params_.relative_orientation; }

private:
    std::string name_{"Fixed"};
    FixedConstraintParams fixed_params_;
    bool orientation_captured_{false};

    /**
     * @brief Build position constraint rows (3)
     */
    void build_position_rows(const EntityState* state_A, const EntityState* state_B,
                             Real dt, ConstraintRow* rows);

    /**
     * @brief Build orientation constraint rows (3)
     */
    void build_orientation_rows(const EntityState* state_A, const EntityState* state_B,
                                Real dt, ConstraintRow* rows);
};

} // namespace jaguar::physics
