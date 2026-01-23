#pragma once
/**
 * @file constraint_solver.h
 * @brief Constraint solver interface and implementations
 *
 * Defines the solver infrastructure for resolving constraints between
 * rigid bodies. The primary solver is Sequential Impulse (SI), which
 * iteratively applies corrective impulses to satisfy all constraints.
 */

#include "jaguar/physics/constraints/constraint.h"
#include "jaguar/physics/entity.h"
#include <memory>
#include <vector>
#include <unordered_map>

namespace jaguar::physics {

/**
 * @brief Configuration for the constraint solver
 */
struct ConstraintSolverConfig {
    int velocity_iterations{8};         ///< Velocity solver iterations
    int position_iterations{3};         ///< Position correction iterations
    Real position_correction{0.2};      ///< Baumgarte stabilization factor (ERP)
    Real allowed_penetration{0.01};     ///< Slop before position correction (m)
    Real contact_bias{0.1};             ///< Contact bias velocity (m/s)
    bool warm_starting{true};           ///< Enable warm starting
    Real warm_start_factor{0.8};        ///< Warm start scaling factor
    bool split_impulse{false};          ///< Use split impulse for contacts
    Real max_correction_velocity{10.0}; ///< Maximum correction velocity (m/s)
    Real sleep_threshold{0.01};         ///< Velocity threshold for sleeping
    int sleep_frames{60};               ///< Frames below threshold to sleep
};

/**
 * @brief Result of constraint solving
 */
struct ConstraintSolverResult {
    int total_constraints{0};           ///< Total constraints processed
    int active_constraints{0};          ///< Non-sleeping constraints
    int iterations_used{0};             ///< Actual iterations performed
    Real max_error{0.0};                ///< Maximum constraint error after solve
    Real total_impulse{0.0};            ///< Total impulse applied
    int broken_count{0};                ///< Constraints that broke this frame
    Real solve_time_ms{0.0};            ///< Time spent solving (ms)
};

/**
 * @brief Abstract interface for constraint solvers
 *
 * Different solver implementations can be swapped while maintaining
 * the same API for constraint management and solving.
 */
class IConstraintSolver {
public:
    virtual ~IConstraintSolver() = default;

    /**
     * @brief Get solver name
     */
    virtual const std::string& name() const = 0;

    /**
     * @brief Initialize solver for a new simulation frame
     *
     * Called once at the start of each physics update to prepare
     * constraint data structures and warm starting.
     *
     * @param dt Time step
     */
    virtual void begin_frame(Real dt) = 0;

    /**
     * @brief Add a constraint to be solved this frame
     *
     * @param constraint Constraint to add
     * @param state_A State of body A
     * @param state_B State of body B (nullptr for static anchor)
     */
    virtual void add_constraint(IConstraint* constraint,
                                 EntityState* state_A,
                                 EntityState* state_B) = 0;

    /**
     * @brief Solve all added constraints
     *
     * Iteratively applies impulses to satisfy constraints while
     * maintaining physical plausibility.
     *
     * @return Solve result statistics
     */
    virtual ConstraintSolverResult solve() = 0;

    /**
     * @brief End frame and store warm starting data
     */
    virtual void end_frame() = 0;

    /**
     * @brief Get solver configuration
     */
    virtual const ConstraintSolverConfig& config() const = 0;

    /**
     * @brief Set solver configuration
     */
    virtual void set_config(const ConstraintSolverConfig& config) = 0;

    /**
     * @brief Clear all constraints (for new simulation)
     */
    virtual void clear() = 0;
};

/**
 * @brief Sequential Impulse constraint solver
 *
 * The Sequential Impulse (SI) method is an iterative solver that processes
 * constraints one at a time, applying corrective impulses and immediately
 * updating velocities. This is equivalent to Projected Gauss-Seidel (PGS)
 * for LCP problems.
 *
 * Key Features:
 * - Warm starting for faster convergence
 * - Clamped impulses for inequality constraints
 * - Baumgarte stabilization for position correction
 * - Split impulse option for stable stacking
 *
 * Algorithm Overview:
 * 1. Build constraint rows (Jacobians, RHS, limits)
 * 2. Warm start with previous frame's impulses
 * 3. Iterate: for each row, compute and apply impulse
 * 4. Store impulses for next frame's warm start
 *
 * Reference: Erin Catto, "Iterative Dynamics with Temporal Coherence"
 */
class SequentialImpulseSolver : public IConstraintSolver {
public:
    SequentialImpulseSolver();
    ~SequentialImpulseSolver() override = default;

    const std::string& name() const override { return name_; }

    void begin_frame(Real dt) override;
    void add_constraint(IConstraint* constraint,
                        EntityState* state_A,
                        EntityState* state_B) override;
    ConstraintSolverResult solve() override;
    void end_frame() override;

    const ConstraintSolverConfig& config() const override { return config_; }
    void set_config(const ConstraintSolverConfig& config) override { config_ = config; }

    void clear() override;

    /**
     * @brief Get number of constraints currently added
     */
    size_t constraint_count() const { return constraints_.size(); }

private:
    std::string name_{"SequentialImpulse"};
    ConstraintSolverConfig config_;
    Real dt_{0.0};

    /**
     * @brief Internal constraint data for solving
     */
    struct ConstraintData {
        IConstraint* constraint{nullptr};
        EntityState* state_A{nullptr};
        EntityState* state_B{nullptr};
        std::vector<ConstraintRow> rows;
        Real inv_mass_A{0.0};
        Real inv_mass_B{0.0};
        Mat3x3 inv_inertia_A;
        Mat3x3 inv_inertia_B;
    };

    std::vector<ConstraintData> constraints_;

    // Warm starting cache (constraint_id -> accumulated impulses)
    std::unordered_map<ConstraintId, std::vector<Real>> warm_start_cache_;

    /**
     * @brief Build constraint rows for all constraints
     */
    void build_constraints();

    /**
     * @brief Apply warm starting from previous frame
     */
    void apply_warm_start();

    /**
     * @brief Perform velocity iterations
     */
    void solve_velocity_constraints(int iterations);

    /**
     * @brief Perform position correction iterations
     */
    void solve_position_constraints(int iterations);

    /**
     * @brief Solve a single constraint row
     *
     * Computes the impulse needed to satisfy the constraint and
     * applies it to both bodies.
     *
     * @param data Constraint data
     * @param row_index Row to solve
     * @return Impulse magnitude applied
     */
    Real solve_row(ConstraintData& data, int row_index);

    /**
     * @brief Store accumulated impulses for warm starting
     */
    void store_warm_start();

    /**
     * @brief Compute inverse mass and inertia for a body
     */
    void compute_inverse_mass(const EntityState* state,
                               Real& inv_mass, Mat3x3& inv_inertia) const;
};

/**
 * @brief Constraint manager for the physics system
 *
 * Manages constraint lifecycle, entity associations, and solver invocation.
 * Provides the high-level API for creating and managing constraints.
 */
class ConstraintManager {
public:
    ConstraintManager();
    ~ConstraintManager();

    /**
     * @brief Set the constraint solver to use
     */
    void set_solver(std::unique_ptr<IConstraintSolver> solver);

    /**
     * @brief Get the current solver
     */
    IConstraintSolver* solver() { return solver_.get(); }

    /**
     * @brief Add a constraint
     *
     * @param constraint Constraint to add (takes ownership)
     * @return Constraint ID
     */
    ConstraintId add_constraint(std::unique_ptr<IConstraint> constraint);

    /**
     * @brief Remove a constraint by ID
     */
    void remove_constraint(ConstraintId id);

    /**
     * @brief Get a constraint by ID
     */
    IConstraint* get_constraint(ConstraintId id);

    /**
     * @brief Get all constraints for an entity
     */
    std::vector<IConstraint*> get_entity_constraints(EntityId entity_id);

    /**
     * @brief Remove all constraints involving an entity
     */
    void remove_entity_constraints(EntityId entity_id);

    /**
     * @brief Solve all constraints for current frame
     *
     * @param entity_manager Entity manager for state access
     * @param dt Time step
     * @return Solve result
     */
    ConstraintSolverResult solve(EntityManager& entity_manager, Real dt);

    /**
     * @brief Clear all constraints
     */
    void clear();

    /**
     * @brief Get total constraint count
     */
    size_t constraint_count() const { return constraints_.size(); }

    /**
     * @brief Get solver configuration
     */
    const ConstraintSolverConfig& config() const;

    /**
     * @brief Set solver configuration
     */
    void set_config(const ConstraintSolverConfig& config);

private:
    std::unique_ptr<IConstraintSolver> solver_;
    std::unordered_map<ConstraintId, std::unique_ptr<IConstraint>> constraints_;
    std::unordered_map<EntityId, std::vector<ConstraintId>> entity_constraints_;
    ConstraintId next_id_{0};
};

} // namespace jaguar::physics
