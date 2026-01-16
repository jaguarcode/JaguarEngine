#pragma once
/**
 * @file solver.h
 * @brief State propagation and integration interfaces
 *
 * Defines interfaces for numerical integrators and physics solvers
 * that advance entity states through time.
 */

#include "jaguar/core/types.h"
#include "jaguar/physics/entity.h"
#include <memory>
#include <string>

namespace jaguar::physics {

// ============================================================================
// State Propagator Interface
// ============================================================================

/**
 * @brief Abstract interface for state integration methods
 *
 * Implements numerical integration schemes (RK4, ABM, etc.)
 * to propagate entity states forward in time.
 */
class IStatePropagator {
public:
    virtual ~IStatePropagator() = default;

    /**
     * @brief Integrate entity state by one time step
     *
     * @param state Entity state (modified in place)
     * @param forces Total forces and torques
     * @param dt Time step (seconds)
     */
    virtual void integrate(
        EntityState& state,
        const EntityForces& forces,
        Real dt) = 0;

    /**
     * @brief Get the name of this integrator
     */
    virtual const std::string& name() const = 0;

    /**
     * @brief Get integration order (1=Euler, 4=RK4, etc.)
     */
    virtual int order() const = 0;

    /**
     * @brief Reset integrator state (for multi-step methods)
     */
    virtual void reset() {}
};

// ============================================================================
// Concrete Integrators
// ============================================================================

/**
 * @brief Fourth-order Runge-Kutta integrator
 *
 * Standard RK4 method with good accuracy/stability balance.
 * Suitable for most physics simulation needs.
 */
class RK4Integrator : public IStatePropagator {
public:
    RK4Integrator();
    ~RK4Integrator() override = default;

    void integrate(EntityState& state, const EntityForces& forces, Real dt) override;
    const std::string& name() const override { return name_; }
    int order() const override { return 4; }

private:
    std::string name_{"RK4"};
};

/**
 * @brief Adams-Bashforth-Moulton predictor-corrector integrator
 *
 * Fourth-order multi-step method. More efficient than RK4 after
 * startup, but requires history of previous states.
 */
class ABM4Integrator : public IStatePropagator {
public:
    ABM4Integrator();
    ~ABM4Integrator() override = default;

    void integrate(EntityState& state, const EntityForces& forces, Real dt) override;
    const std::string& name() const override { return name_; }
    int order() const override { return 4; }
    void reset() override;

private:
    std::string name_{"ABM4"};
    int step_count_{0};
    // History arrays for multi-step method
    Vec3 accel_history_[4];
    Vec3 angular_accel_history_[4];
};

/**
 * @brief Simple Euler integrator (for testing/comparison)
 */
class EulerIntegrator : public IStatePropagator {
public:
    EulerIntegrator() = default;
    ~EulerIntegrator() override = default;

    void integrate(EntityState& state, const EntityForces& forces, Real dt) override;
    const std::string& name() const override { return name_; }
    int order() const override { return 1; }

private:
    std::string name_{"Euler"};
};

// ============================================================================
// Adaptive Time-Stepping
// ============================================================================

/**
 * @brief Result of an adaptive integration step
 */
struct AdaptiveStepResult {
    bool accepted{true};           ///< Was the step accepted?
    Real actual_dt{0.0};           ///< Actual time step used
    Real suggested_dt{0.0};        ///< Suggested next time step
    Real error_estimate{0.0};      ///< Estimated local truncation error
    int substeps{1};               ///< Number of substeps taken
};

/**
 * @brief Configuration for adaptive time stepping
 */
struct AdaptiveConfig {
    Real tolerance{1e-6};          ///< Error tolerance for step acceptance
    Real min_dt{1e-9};             ///< Minimum allowed time step (s)
    Real max_dt{0.1};              ///< Maximum allowed time step (s)
    Real safety_factor{0.9};       ///< Safety factor for step size adjustment
    Real growth_limit{2.0};        ///< Maximum step size growth factor
    Real shrink_limit{0.1};        ///< Minimum step size shrink factor
    int max_substeps{100};         ///< Maximum substeps per integration call
};

/**
 * @brief Adaptive time-stepping integrator wrapper
 *
 * Uses Richardson extrapolation (embedded methods) to estimate error
 * and automatically adjusts step size for accuracy.
 *
 * Works by comparing solutions at dt and dt/2, using the difference
 * as an error estimate.
 */
class AdaptiveIntegrator : public IStatePropagator {
public:
    /**
     * @brief Create adaptive integrator with default RK4 base
     */
    AdaptiveIntegrator();

    /**
     * @brief Create adaptive integrator with custom base integrator
     * @param base_integrator Base integrator to use (takes ownership)
     */
    explicit AdaptiveIntegrator(std::unique_ptr<IStatePropagator> base_integrator);

    ~AdaptiveIntegrator() override = default;

    /**
     * @brief Integrate with automatic step size control
     *
     * May take multiple substeps to achieve desired accuracy.
     * The actual time advanced equals dt (unless max_substeps exceeded).
     */
    void integrate(EntityState& state, const EntityForces& forces, Real dt) override;

    const std::string& name() const override { return name_; }
    int order() const override { return base_integrator_ ? base_integrator_->order() : 4; }
    void reset() override;

    /**
     * @brief Set adaptive configuration
     */
    void set_config(const AdaptiveConfig& config) { config_ = config; }

    /**
     * @brief Get adaptive configuration
     */
    const AdaptiveConfig& get_config() const { return config_; }

    /**
     * @brief Get result of last integration
     */
    const AdaptiveStepResult& get_last_result() const { return last_result_; }

    /**
     * @brief Get current suggested time step
     */
    Real get_suggested_dt() const { return suggested_dt_; }

    /**
     * @brief Perform single adaptive step (for more control)
     *
     * @param state Entity state (modified in place)
     * @param forces Total forces and torques
     * @param dt Requested time step
     * @return Result with actual step taken and error estimate
     */
    AdaptiveStepResult adaptive_step(
        EntityState& state,
        const EntityForces& forces,
        Real dt);

private:
    std::unique_ptr<IStatePropagator> base_integrator_;
    AdaptiveConfig config_;
    AdaptiveStepResult last_result_;
    Real suggested_dt_{0.01};
    std::string name_{"Adaptive"};

    /**
     * @brief Estimate error using Richardson extrapolation
     *
     * Compares full step vs two half steps to estimate local error.
     */
    Real estimate_error(
        const EntityState& state_full,
        const EntityState& state_half,
        int order) const;

    /**
     * @brief Compute optimal new step size based on error
     */
    Real compute_new_dt(Real dt, Real error, int order) const;
};

/**
 * @brief RK45 (Runge-Kutta-Fehlberg) embedded integrator
 *
 * Uses embedded 4th/5th order method with built-in error estimation.
 * More efficient than Richardson extrapolation for adaptive stepping.
 */
class RK45Integrator : public IStatePropagator {
public:
    RK45Integrator();
    ~RK45Integrator() override = default;

    void integrate(EntityState& state, const EntityForces& forces, Real dt) override;
    const std::string& name() const override { return name_; }
    int order() const override { return 5; }

    /**
     * @brief Set error tolerance
     */
    void set_tolerance(Real tol) { tolerance_ = tol; }

    /**
     * @brief Get last error estimate
     */
    Real get_error_estimate() const { return error_estimate_; }

    /**
     * @brief Get suggested next time step
     */
    Real get_suggested_dt() const { return suggested_dt_; }

private:
    std::string name_{"RK45"};
    Real tolerance_{1e-6};
    Real error_estimate_{0.0};
    Real suggested_dt_{0.01};
};

// ============================================================================
// Physics System
// ============================================================================

/**
 * @brief Central physics processing system
 *
 * Coordinates force computation and state integration for all
 * entities using the registered force generators and integrator.
 */
class PhysicsSystem {
public:
    PhysicsSystem();
    ~PhysicsSystem();

    /**
     * @brief Set the state propagator (integrator)
     */
    void set_propagator(std::unique_ptr<IStatePropagator> propagator);

    /**
     * @brief Get current propagator
     */
    IStatePropagator* get_propagator() { return propagator_.get(); }

    /**
     * @brief Update all entities by one time step
     *
     * @param entity_manager Entity manager with state storage
     * @param dt Time step (seconds)
     */
    void update(EntityManager& entity_manager, Real dt);

    /**
     * @brief Update a single entity
     */
    void update_entity(Entity& entity, EntityStateStorage& storage, Real dt);

    /**
     * @brief Enable/disable parallel processing
     */
    void set_parallel_enabled(bool enabled) { parallel_enabled_ = enabled; }

    /**
     * @brief Check if parallel processing is enabled
     */
    bool is_parallel_enabled() const { return parallel_enabled_; }

private:
    std::unique_ptr<IStatePropagator> propagator_;
    bool parallel_enabled_{true};
};

// ============================================================================
// Quaternion Integration Utilities
// ============================================================================

namespace quaternion_utils {

/**
 * @brief Integrate quaternion with angular velocity (first-order)
 *
 * Uses the quaternion derivative: q_dot = 0.5 * q * omega
 * where omega is angular velocity in body frame as a pure quaternion.
 *
 * @param q Current orientation quaternion
 * @param omega Angular velocity in body frame (rad/s)
 * @param dt Time step
 * @return Updated quaternion (normalized)
 */
Quat integrate(const Quat& q, const Vec3& omega, Real dt);

/**
 * @brief Integrate quaternion with angular velocity (RK4)
 *
 * Fourth-order Runge-Kutta integration for higher accuracy.
 *
 * @param q Current orientation quaternion
 * @param omega Angular velocity in body frame (rad/s)
 * @param dt Time step
 * @return Updated quaternion (normalized)
 */
Quat integrate_rk4(const Quat& q, const Vec3& omega, Real dt);

/**
 * @brief Normalize quaternion
 */
Quat normalize(const Quat& q);

/**
 * @brief Convert Euler angles (rad) to quaternion
 * @param roll Roll angle (phi)
 * @param pitch Pitch angle (theta)
 * @param yaw Yaw angle (psi)
 */
Quat from_euler(Real roll, Real pitch, Real yaw);

/**
 * @brief Convert quaternion to Euler angles (rad)
 * @param q Quaternion
 * @param roll Output roll angle
 * @param pitch Output pitch angle
 * @param yaw Output yaw angle
 */
void to_euler(const Quat& q, Real& roll, Real& pitch, Real& yaw);

/**
 * @brief Create rotation matrix from quaternion
 */
Mat3x3 to_rotation_matrix(const Quat& q);

} // namespace quaternion_utils

} // namespace jaguar::physics
