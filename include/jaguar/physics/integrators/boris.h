#pragma once
/**
 * @file boris.h
 * @brief Boris integrator for charged particle dynamics
 *
 * The Boris algorithm is a symplectic integrator specifically designed for
 * charged particle motion in electromagnetic fields. It exactly preserves
 * the particle gyration in a uniform magnetic field and maintains excellent
 * energy conservation for long-duration simulations.
 *
 * This integrator is essential for:
 * - Space weather effects on satellites
 * - Charged particle tracking (cosmic rays, radiation belts)
 * - Plasma simulations
 * - Electromagnetic weapon effects modeling
 */

#include "jaguar/physics/solver.h"

namespace jaguar::physics {

/**
 * @brief Extended forces structure with electromagnetic fields
 *
 * Extends EntityForces to include electric and magnetic field vectors
 * needed for the Boris algorithm.
 */
struct ElectromagneticForces : public EntityForces {
    Vec3 electric_field{Vec3::Zero()};   ///< Electric field (V/m)
    Vec3 magnetic_field{Vec3::Zero()};   ///< Magnetic field (Tesla)
    Real charge{0.0};                     ///< Particle charge (Coulombs)
};

/**
 * @brief Boris integrator for charged particle dynamics
 *
 * The Boris algorithm (Boris, 1970) is a second-order symplectic integrator
 * that exactly preserves the gyration of a charged particle in a uniform
 * magnetic field. It separates the velocity update into three phases:
 *
 * 1. Half acceleration from electric field: v⁻ = v^n + (q*E/m) * dt/2
 * 2. Rotation from magnetic field: v⁺ = rotation(v⁻, B)
 * 3. Half acceleration from electric field: v^(n+1) = v⁺ + (q*E/m) * dt/2
 *
 * The magnetic rotation uses the Boris rotation formula which exactly
 * preserves |v| during the rotation step, ensuring energy conservation.
 *
 * Properties:
 * - Second-order accuracy
 * - Symplectic (preserves phase space volume)
 * - Exactly preserves gyration in uniform B field
 * - Excellent long-term energy conservation
 * - Volume-preserving in velocity space
 *
 * Key Advantages:
 * - Stable for arbitrarily strong magnetic fields
 * - No artificial damping of cyclotron motion
 * - Preserves magnetic moment (adiabatic invariant)
 * - Widely validated in plasma physics community
 *
 * Use Cases:
 * - Satellite charging and radiation effects
 * - Van Allen radiation belt modeling
 * - Cosmic ray propagation
 * - Space weather prediction
 * - Electromagnetic pulse (EMP) effects
 * - Particle accelerator simulation
 * - Magnetohydrodynamic (MHD) coupling
 *
 * Limitations:
 * - Designed specifically for charged particles
 * - Assumes non-relativistic motion (use relativistic Boris for v ~ c)
 * - Electric and magnetic fields must be provided externally
 * - For neutral particles, falls back to standard integration
 */
class BorisIntegrator : public IStatePropagator {
public:
    BorisIntegrator();
    ~BorisIntegrator() override = default;

    /**
     * @brief Integrate entity state using Boris algorithm
     *
     * For charged particles (charge != 0), uses the Boris algorithm.
     * For neutral particles, falls back to symplectic Euler.
     *
     * @param state Entity state (modified in place)
     * @param forces Forces including electromagnetic fields (use ElectromagneticForces)
     * @param dt Time step (seconds)
     */
    void integrate(EntityState& state, const EntityForces& forces, Real dt) override;

    /**
     * @brief Integrate charged particle with explicit EM fields
     *
     * This version takes electromagnetic forces directly for clarity.
     *
     * @param state Entity state
     * @param em_forces Electromagnetic forces and fields
     * @param dt Time step
     */
    void integrate_charged(EntityState& state, const ElectromagneticForces& em_forces, Real dt);

    /**
     * @brief Get the name of this integrator
     * @return "Boris"
     */
    const std::string& name() const override { return name_; }

    /**
     * @brief Get integration order (2 for Boris)
     */
    int order() const override { return 2; }

    /**
     * @brief Reset integrator state
     */
    void reset() override;

    // ========================================================================
    // Electromagnetic Field Configuration
    // ========================================================================

    /**
     * @brief Set uniform background electric field
     *
     * This is added to any field specified in ElectromagneticForces.
     *
     * @param E Electric field vector (V/m)
     */
    void set_background_electric_field(const Vec3& E) { background_E_ = E; }

    /**
     * @brief Get background electric field
     */
    const Vec3& get_background_electric_field() const { return background_E_; }

    /**
     * @brief Set uniform background magnetic field
     *
     * This is added to any field specified in ElectromagneticForces.
     *
     * @param B Magnetic field vector (Tesla)
     */
    void set_background_magnetic_field(const Vec3& B) { background_B_ = B; }

    /**
     * @brief Get background magnetic field
     */
    const Vec3& get_background_magnetic_field() const { return background_B_; }

    // ========================================================================
    // Diagnostics and Monitoring
    // ========================================================================

    /**
     * @brief Enable/disable energy tracking
     */
    void set_energy_tracking(bool enabled) { track_energy_ = enabled; }

    /**
     * @brief Check if energy tracking is enabled
     */
    bool is_energy_tracking() const { return track_energy_; }

    /**
     * @brief Get accumulated energy drift
     */
    Real get_energy_drift() const { return energy_drift_; }

    /**
     * @brief Reset energy tracking
     */
    void reset_energy_tracking();

    /**
     * @brief Get the last computed gyrofrequency
     *
     * Cyclotron frequency: ω_c = |q|*|B| / m
     */
    Real get_gyrofrequency() const { return gyrofrequency_; }

    /**
     * @brief Get the last computed gyroradius
     *
     * Larmor radius: r_L = m*v_perp / (|q|*|B|)
     */
    Real get_gyroradius() const { return gyroradius_; }

    /**
     * @brief Get the last computed magnetic moment
     *
     * Magnetic moment (first adiabatic invariant): μ = m*v_perp² / (2*|B|)
     */
    Real get_magnetic_moment() const { return magnetic_moment_; }

    /**
     * @brief Check if the particle is magnetized
     *
     * A particle is considered magnetized if ω_c * dt > 0.1
     * (gyroperiod is at least 10x the time step)
     */
    bool is_magnetized() const { return is_magnetized_; }

private:
    std::string name_{"Boris"};

    // Background fields
    Vec3 background_E_{Vec3::Zero()};
    Vec3 background_B_{Vec3::Zero()};

    // Energy tracking
    bool track_energy_{false};
    Real initial_energy_{0.0};
    Real energy_drift_{0.0};
    bool energy_initialized_{false};

    // Diagnostics
    Real gyrofrequency_{0.0};
    Real gyroradius_{0.0};
    Real magnetic_moment_{0.0};
    bool is_magnetized_{false};

    /**
     * @brief Compute total mechanical energy
     */
    Real compute_mechanical_energy(const EntityState& state) const;

    /**
     * @brief Compute angular acceleration using Euler's equations
     */
    Vec3 compute_angular_accel(const Mat3x3& inertia, const Mat3x3& inv_inertia,
                               const Vec3& omega, const Vec3& torque) const;

    /**
     * @brief Perform Boris rotation step
     *
     * Rotates velocity by the magnetic force using the Boris formula:
     *   t = (q*B/m) * dt/2
     *   s = 2*t / (1 + |t|²)
     *   v' = v + v × t
     *   v+ = v + v' × s
     *
     * This exactly preserves |v| and gives correct gyration.
     *
     * @param v Velocity to rotate
     * @param B Magnetic field
     * @param q_over_m Charge-to-mass ratio
     * @param dt Time step
     * @return Rotated velocity
     */
    Vec3 boris_rotation(const Vec3& v, const Vec3& B, Real q_over_m, Real dt) const;

    /**
     * @brief Update diagnostic quantities
     */
    void update_diagnostics(const EntityState& state, const Vec3& B, Real charge, Real dt);
};

} // namespace jaguar::physics
