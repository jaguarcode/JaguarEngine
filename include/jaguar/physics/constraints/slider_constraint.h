#pragma once
/**
 * @file slider_constraint.h
 * @brief Slider (prismatic) joint constraint
 *
 * The slider constraint allows translation along a single axis while
 * preventing all rotation and translation along other axes. This is
 * used for pistons, linear actuators, and sliding mechanisms.
 */

#include "jaguar/physics/constraints/constraint.h"

namespace jaguar::physics {

/**
 * @brief Slider constraint parameters
 */
struct SliderConstraintParams : public ConstraintParams {
    // Position limits
    bool enable_limits{false};       ///< Enable position limits
    Real lower_limit{-1.0};          ///< Minimum position (m)
    Real upper_limit{1.0};           ///< Maximum position (m)

    // Motor
    bool enable_motor{false};        ///< Enable motor
    Real motor_velocity{0.0};        ///< Target linear velocity (m/s)
    Real max_motor_force{0.0};       ///< Maximum motor force (N)

    // Spring
    bool enable_spring{false};       ///< Enable spring behavior
    Real spring_stiffness{0.0};      ///< Spring stiffness (N/m)
    Real spring_damping{0.0};        ///< Spring damping (N·s/m)
    Real spring_rest_position{0.0};  ///< Rest position for spring (m)
};

/**
 * @brief Slider (prismatic) joint constraint
 *
 * Constrains two bodies to translate relative to each other along a single
 * axis. The constraint removes 5 degrees of freedom, leaving only translation
 * along the slider axis.
 *
 * Constraint Equations (5 rows):
 * - Row 0-1: Position constraint perpendicular to slider axis
 *   C_perp = perp · (p_B - p_A) = 0 (for two perpendicular directions)
 *
 * - Row 2-4: Orientation constraint (relative orientation maintained)
 *   C_rot = 2 * vec(q_error) = 0
 *
 * Optional:
 * - Row 5: Position limit (if enabled and at limit)
 * - Row 6: Motor (if enabled)
 *
 * Use Cases:
 * - Shock absorbers
 * - Pistons
 * - Linear actuators
 * - Sliding doors
 * - Telescoping mechanisms
 * - Landing gear extension
 */
class SliderConstraint : public IConstraint {
public:
    /**
     * @brief Create slider constraint
     *
     * @param body_A First body (with slider axis)
     * @param body_B Second body (slides along axis)
     */
    SliderConstraint(const ConstraintBody& body_A, const ConstraintBody& body_B);

    /**
     * @brief Create slider constraint with parameters
     */
    SliderConstraint(const ConstraintBody& body_A, const ConstraintBody& body_B,
                     const SliderConstraintParams& params);

    ~SliderConstraint() override = default;

    ConstraintType type() const override { return ConstraintType::Slider; }
    const std::string& name() const override { return name_; }

    /**
     * @brief Get number of constraint rows
     *
     * Base: 5 (2 position + 3 orientation)
     * +1 if position limit active
     * +1 if motor enabled
     */
    int num_rows() const override;

    void build_rows(const EntityState* state_A, const EntityState* state_B,
                    Real dt, ConstraintRow* rows) override;

    /**
     * @brief Get slider parameters
     */
    const SliderConstraintParams& slider_params() const { return slider_params_; }

    /**
     * @brief Set slider parameters
     */
    void set_slider_params(const SliderConstraintParams& params) { slider_params_ = params; }

    /**
     * @brief Get current position along slider axis
     */
    Real get_position() const { return current_position_; }

    /**
     * @brief Get current velocity along slider axis
     */
    Real get_velocity() const { return current_velocity_; }

    // ========================================================================
    // Convenience setters
    // ========================================================================

    /**
     * @brief Set position limits
     */
    void set_limits(Real lower, Real upper) {
        slider_params_.enable_limits = true;
        slider_params_.lower_limit = lower;
        slider_params_.upper_limit = upper;
    }

    /**
     * @brief Disable position limits
     */
    void disable_limits() { slider_params_.enable_limits = false; }

    /**
     * @brief Set motor parameters
     */
    void set_motor(Real target_velocity, Real max_force) {
        slider_params_.enable_motor = true;
        slider_params_.motor_velocity = target_velocity;
        slider_params_.max_motor_force = max_force;
    }

    /**
     * @brief Disable motor
     */
    void disable_motor() { slider_params_.enable_motor = false; }

    /**
     * @brief Set spring parameters
     */
    void set_spring(Real stiffness, Real damping, Real rest_position = 0.0) {
        slider_params_.enable_spring = true;
        slider_params_.spring_stiffness = stiffness;
        slider_params_.spring_damping = damping;
        slider_params_.spring_rest_position = rest_position;
    }

    /**
     * @brief Store current relative orientation as target
     */
    void capture_current_orientation(const EntityState* state_A, const EntityState* state_B);

private:
    std::string name_{"Slider"};
    SliderConstraintParams slider_params_;
    Quat initial_relative_orientation_{Quat::Identity()};
    bool orientation_captured_{false};

    // Computed values
    Real current_position_{0.0};
    Real current_velocity_{0.0};

    /**
     * @brief Compute current slider position and velocity
     */
    void compute_position(const EntityState* state_A, const EntityState* state_B);

    /**
     * @brief Build perpendicular position constraint rows (2)
     */
    void build_position_rows(const EntityState* state_A, const EntityState* state_B,
                             Real dt, ConstraintRow* rows);

    /**
     * @brief Build orientation constraint rows (3)
     */
    void build_orientation_rows(const EntityState* state_A, const EntityState* state_B,
                                Real dt, ConstraintRow* rows);

    /**
     * @brief Build position limit row (optional)
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
