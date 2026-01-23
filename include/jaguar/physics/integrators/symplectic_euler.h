#pragma once
/**
 * @file symplectic_euler.h
 * @brief Symplectic Euler integrator for energy-conserving simulations
 *
 * Symplectic integrators preserve the geometric structure of Hamiltonian systems,
 * resulting in better long-term energy conservation compared to standard methods.
 * This is particularly important for orbital mechanics and long-duration simulations.
 */

#include "jaguar/physics/solver.h"

namespace jaguar::physics {

/**
 * @brief Symplectic Euler integrator (semi-implicit Euler)
 *
 * The symplectic Euler method is a first-order symplectic integrator that
 * preserves the symplectic structure of Hamiltonian systems. While it has
 * only first-order accuracy, it provides excellent long-term energy conservation.
 *
 * The method updates velocity first using the current position, then updates
 * position using the new velocity:
 *
 *   v(t+dt) = v(t) + a(t) * dt
 *   x(t+dt) = x(t) + v(t+dt) * dt
 *
 * This ordering ensures the method is symplectic and preserves phase space volume.
 *
 * Energy Behavior:
 * - Oscillates around true energy (no secular drift)
 * - Error bounded for arbitrarily long simulations
 * - Better than RK4 for >1 hour simulations
 *
 * Use Cases:
 * - Orbital mechanics (satellite propagation)
 * - Long-duration flight simulations
 * - N-body gravitational systems
 * - Any simulation where energy conservation is critical
 *
 * Trade-offs:
 * - Lower accuracy per step than RK4 (first-order vs fourth-order)
 * - Superior long-term stability and energy conservation
 * - Computationally cheaper (single force evaluation per step)
 */
class SymplecticEulerIntegrator : public IStatePropagator {
public:
    SymplecticEulerIntegrator();
    ~SymplecticEulerIntegrator() override = default;

    /**
     * @brief Integrate entity state by one time step using symplectic Euler
     *
     * Implements the semi-implicit Euler method:
     * 1. Update velocity using current acceleration
     * 2. Update position using the NEW velocity (key for symplecticity)
     * 3. Update angular velocity using Euler's equations
     * 4. Update orientation using quaternion integration
     *
     * @param state Entity state (modified in place)
     * @param forces Total forces and torques in body frame
     * @param dt Time step (seconds)
     */
    void integrate(EntityState& state, const EntityForces& forces, Real dt) override;

    /**
     * @brief Get the name of this integrator
     * @return "SymplecticEuler"
     */
    const std::string& name() const override { return name_; }

    /**
     * @brief Get integration order (1 for Euler methods)
     */
    int order() const override { return 1; }

    /**
     * @brief Get accumulated energy drift (for monitoring)
     *
     * This is computed as the difference between current mechanical energy
     * and the initial energy (if tracking is enabled).
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

private:
    std::string name_{"SymplecticEuler"};
    bool track_energy_{false};
    Real initial_energy_{0.0};
    Real energy_drift_{0.0};
    bool energy_initialized_{false};

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
 * @brief Position Verlet (Störmer-Verlet) symplectic integrator
 *
 * Second-order symplectic integrator with excellent energy conservation.
 * Also known as the leapfrog integrator when formulated differently.
 *
 * The method is time-reversible and preserves the symplectic structure.
 * Uses positions at two time levels for higher accuracy:
 *
 *   x(t+dt) = 2*x(t) - x(t-dt) + a(t) * dt²
 *
 * Note: See verlet.h for the full Verlet integrator implementation.
 */

} // namespace jaguar::physics
