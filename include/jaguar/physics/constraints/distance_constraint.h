#pragma once
/**
 * @file distance_constraint.h
 * @brief Distance constraint for maintaining fixed distance between points
 *
 * The distance constraint maintains a specified distance between two anchor
 * points on different bodies. This is the simplest constraint type and serves
 * as the basis for more complex joints.
 */

#include "jaguar/physics/constraints/constraint.h"

namespace jaguar::physics {

/**
 * @brief Distance constraint parameters
 */
struct DistanceConstraintParams : public ConstraintParams {
    Real distance{1.0};           ///< Target distance (m)
    Real min_distance{0.0};       ///< Minimum distance (0 = use exact distance)
    Real max_distance{0.0};       ///< Maximum distance (0 = use exact distance)

    /**
     * @brief Create an exact distance constraint
     */
    static DistanceConstraintParams Exact(Real d) {
        DistanceConstraintParams p;
        p.distance = d;
        return p;
    }

    /**
     * @brief Create a distance range constraint
     */
    static DistanceConstraintParams Range(Real min_d, Real max_d) {
        DistanceConstraintParams p;
        p.distance = (min_d + max_d) * 0.5;
        p.min_distance = min_d;
        p.max_distance = max_d;
        return p;
    }
};

/**
 * @brief Distance constraint implementation
 *
 * Maintains a fixed distance between two anchor points. The constraint
 * equation is:
 *
 *   C = |p_B - p_A| - d = 0
 *
 * where p_A and p_B are world positions of the anchor points, and d is
 * the target distance.
 *
 * The Jacobian is:
 *   J = [-n, -(r_A × n), n, (r_B × n)]
 *
 * where n is the unit direction from A to B, and r_A, r_B are the local
 * anchor offsets rotated to world frame.
 *
 * Use Cases:
 * - Rope/chain simulation
 * - Simple pendulum
 * - Suspension springs (with soft constraint)
 * - Landing gear struts
 */
class DistanceConstraint : public IConstraint {
public:
    /**
     * @brief Create distance constraint
     *
     * @param body_A First body
     * @param body_B Second body
     * @param distance Target distance (m)
     */
    DistanceConstraint(const ConstraintBody& body_A, const ConstraintBody& body_B,
                       Real distance);

    /**
     * @brief Create distance constraint with full parameters
     */
    DistanceConstraint(const ConstraintBody& body_A, const ConstraintBody& body_B,
                       const DistanceConstraintParams& params);

    ~DistanceConstraint() override = default;

    ConstraintType type() const override { return ConstraintType::Distance; }
    const std::string& name() const override { return name_; }
    int num_rows() const override { return 1; }

    void build_rows(const EntityState* state_A, const EntityState* state_B,
                    Real dt, ConstraintRow* rows) override;

    /**
     * @brief Get target distance
     */
    Real get_distance() const { return distance_params_.distance; }

    /**
     * @brief Set target distance
     */
    void set_distance(Real d) { distance_params_.distance = d; }

    /**
     * @brief Get distance parameters
     */
    const DistanceConstraintParams& distance_params() const { return distance_params_; }

    /**
     * @brief Set distance parameters
     */
    void set_distance_params(const DistanceConstraintParams& params) {
        distance_params_ = params;
    }

    /**
     * @brief Get current actual distance (computed during build_rows)
     */
    Real get_current_distance() const { return current_distance_; }

private:
    std::string name_{"Distance"};
    DistanceConstraintParams distance_params_;
    Real current_distance_{0.0};
};

} // namespace jaguar::physics
