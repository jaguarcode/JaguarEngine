#pragma once
/**
 * @file verlet.h
 * @brief Verlet integrators for energy-conserving simulations
 *
 * Verlet methods are second-order symplectic integrators that provide excellent
 * long-term energy conservation. They are time-reversible and preserve the
 * symplectic structure of Hamiltonian systems.
 *
 * This file provides two Verlet variants:
 * - VelocityVerletIntegrator: Velocity Verlet (leapfrog) method
 * - PositionVerletIntegrator: Störmer-Verlet (position-based) method
 */

#include "jaguar/physics/solver.h"

namespace jaguar::physics {

/**
 * @brief Velocity Verlet (Leapfrog) integrator
 *
 * Second-order symplectic integrator that is time-reversible and
 * provides excellent energy conservation for long simulations.
 *
 * The Velocity Verlet algorithm:
 *   1. x(t+dt) = x(t) + v(t)*dt + 0.5*a(t)*dt²
 *   2. Compute a(t+dt) from new position
 *   3. v(t+dt) = v(t) + 0.5*(a(t) + a(t+dt))*dt
 *
 * This is mathematically equivalent to the leapfrog method but provides
 * positions, velocities, and accelerations at the same time points.
 *
 * Properties:
 * - Second-order accuracy (O(dt²))
 * - Symplectic (preserves phase space volume)
 * - Time-reversible (symmetric)
 * - Better accuracy than Symplectic Euler with same computational cost
 *
 * Energy Behavior:
 * - Oscillates around true energy (no secular drift)
 * - Error bounded: |E - E₀| ≤ C * dt² for arbitrarily long simulations
 * - Superior to RK4 for orbital mechanics and N-body problems
 *
 * Use Cases:
 * - Molecular dynamics simulations
 * - Orbital mechanics and satellite propagation
 * - Long-duration flight simulations
 * - N-body gravitational systems
 * - Any simulation where energy conservation is critical
 *
 * Limitations:
 * - Requires force to depend only on position (not velocity)
 * - For velocity-dependent forces (drag), use modified Verlet
 */
class VelocityVerletIntegrator : public IStatePropagator {
public:
    VelocityVerletIntegrator();
    ~VelocityVerletIntegrator() override = default;

    /**
     * @brief Integrate entity state by one time step using Velocity Verlet
     *
     * Note: This implementation assumes forces depend only on position.
     * For velocity-dependent forces (e.g., drag), the acceleration is
     * approximated using the new position but old velocity.
     *
     * @param state Entity state (modified in place)
     * @param forces Total forces and torques in body frame
     * @param dt Time step (seconds)
     */
    void integrate(EntityState& state, const EntityForces& forces, Real dt) override;

    /**
     * @brief Get the name of this integrator
     * @return "VelocityVerlet"
     */
    const std::string& name() const override { return name_; }

    /**
     * @brief Get integration order (2 for Verlet methods)
     */
    int order() const override { return 2; }

    /**
     * @brief Get accumulated energy drift (for monitoring)
     */
    Real get_energy_drift() const { return energy_drift_; }

    /**
     * @brief Enable/disable energy tracking
     *
     * When enabled, the integrator tracks mechanical energy to monitor
     * energy conservation. This adds some computational overhead.
     *
     * @param enabled Whether to track energy
     */
    void set_energy_tracking(bool enabled) { track_energy_ = enabled; }

    /**
     * @brief Check if energy tracking is enabled
     */
    bool is_energy_tracking() const { return track_energy_; }

    /**
     * @brief Reset energy tracking (call when starting new simulation)
     */
    void reset_energy_tracking();

    /**
     * @brief Reset integrator state
     */
    void reset() override;

    /**
     * @brief Set the previous acceleration for first step
     *
     * For the first integration step, we need the acceleration at t=0.
     * If not set, it will be computed from the initial forces.
     *
     * @param accel Previous linear acceleration
     * @param angular_accel Previous angular acceleration
     */
    void set_previous_acceleration(const Vec3& accel, const Vec3& angular_accel);

    /**
     * @brief Check if this is the first step (no history yet)
     */
    bool is_first_step() const { return first_step_; }

private:
    std::string name_{"VelocityVerlet"};
    bool track_energy_{false};
    Real initial_energy_{0.0};
    Real energy_drift_{0.0};
    bool energy_initialized_{false};

    // Previous acceleration storage for Velocity Verlet
    bool first_step_{true};
    Vec3 prev_accel_{Vec3::Zero()};
    Vec3 prev_angular_accel_{Vec3::Zero()};

    /**
     * @brief Compute total mechanical energy for tracking
     */
    Real compute_mechanical_energy(const EntityState& state) const;

    /**
     * @brief Compute angular acceleration using Euler's equations
     */
    Vec3 compute_angular_accel(const Mat3x3& inertia, const Mat3x3& inv_inertia,
                               const Vec3& omega, const Vec3& torque) const;
};

/**
 * @brief Position Verlet (Störmer-Verlet) integrator
 *
 * Second-order symplectic integrator that uses positions at two time
 * levels. This is the original Verlet formulation.
 *
 * The Position Verlet algorithm:
 *   x(t+dt) = 2*x(t) - x(t-dt) + a(t)*dt²
 *
 * Velocity is computed as a derived quantity:
 *   v(t) ≈ (x(t+dt) - x(t-dt)) / (2*dt)
 *
 * Properties:
 * - Second-order accuracy (O(dt²))
 * - Symplectic (preserves phase space volume)
 * - Time-reversible
 * - Requires storage of previous position
 *
 * Advantages over Velocity Verlet:
 * - Slightly more numerically stable in some cases
 * - Natural for position-dependent forces
 *
 * Disadvantages:
 * - Requires startup procedure (uses Euler for first step)
 * - Velocity computed via finite difference (slight loss of accuracy)
 *
 * Use Cases:
 * - Molecular dynamics
 * - N-body simulations
 * - Cases where velocity is less important than position accuracy
 */
class PositionVerletIntegrator : public IStatePropagator {
public:
    PositionVerletIntegrator();
    ~PositionVerletIntegrator() override = default;

    /**
     * @brief Integrate entity state by one time step using Position Verlet
     *
     * On the first step, uses symplectic Euler to initialize.
     * Subsequent steps use the full Verlet formula.
     *
     * @param state Entity state (modified in place)
     * @param forces Total forces and torques in body frame
     * @param dt Time step (seconds)
     */
    void integrate(EntityState& state, const EntityForces& forces, Real dt) override;

    /**
     * @brief Get the name of this integrator
     * @return "PositionVerlet"
     */
    const std::string& name() const override { return name_; }

    /**
     * @brief Get integration order (2 for Verlet methods)
     */
    int order() const override { return 2; }

    /**
     * @brief Reset integrator state (clears position history)
     */
    void reset() override;

    /**
     * @brief Get accumulated energy drift (for monitoring)
     */
    Real get_energy_drift() const { return energy_drift_; }

    /**
     * @brief Enable/disable energy tracking
     */
    void set_energy_tracking(bool enabled) { track_energy_ = enabled; }

    /**
     * @brief Check if energy tracking is enabled
     */
    bool is_energy_tracking() const { return track_energy_; }

    /**
     * @brief Reset energy tracking
     */
    void reset_energy_tracking();

private:
    std::string name_{"PositionVerlet"};
    bool track_energy_{false};
    Real initial_energy_{0.0};
    Real energy_drift_{0.0};
    bool energy_initialized_{false};

    // Previous position storage for Position Verlet
    bool first_step_{true};
    Vec3 prev_position_{Vec3::Zero()};
    Quat prev_orientation_{Quat::Identity()};
    Real prev_dt_{0.0};

    /**
     * @brief Compute total mechanical energy for tracking
     */
    Real compute_mechanical_energy(const EntityState& state) const;

    /**
     * @brief Compute angular acceleration using Euler's equations
     */
    Vec3 compute_angular_accel(const Mat3x3& inertia, const Mat3x3& inv_inertia,
                               const Vec3& omega, const Vec3& torque) const;
};

} // namespace jaguar::physics
