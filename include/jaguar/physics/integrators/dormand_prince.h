#pragma once
/**
 * @file dormand_prince.h
 * @brief Dormand-Prince (DOPRI5) adaptive integrator
 *
 * The Dormand-Prince method is an embedded 4th/5th order Runge-Kutta method
 * that is widely considered the most efficient embedded RK method for most
 * problems. It is the default method in MATLAB's ode45.
 *
 * Key advantages over Runge-Kutta-Fehlberg (RKF45):
 * - Better error estimation (uses FSAL - First Same As Last)
 * - More efficient: 6 function evaluations per step (vs 6 for RKF45)
 * - The 5th order solution is used for propagation (not the 4th)
 * - Error estimate from difference between 4th and 5th order
 *
 * The FSAL property means the last stage of step n equals the first stage
 * of step n+1, reducing computational cost for continuous integration.
 */

#include "jaguar/physics/solver.h"

namespace jaguar::physics {

/**
 * @brief Dormand-Prince 5(4) embedded Runge-Kutta integrator
 *
 * This is the DOPRI5 method (Dormand & Prince, 1980), one of the most
 * widely used adaptive ODE solvers. It uses a 7-stage formula where
 * the first stage can be reused from the previous step (FSAL).
 *
 * Butcher Tableau:
 * @verbatim
 *     0    |
 *    1/5   | 1/5
 *    3/10  | 3/40      9/40
 *    4/5   | 44/45    -56/15      32/9
 *    8/9   | 19372/6561 -25360/2187 64448/6561 -212/729
 *     1    | 9017/3168 -355/33    46732/5247  49/176  -5103/18656
 *     1    | 35/384     0         500/1113    125/192 -2187/6784   11/84
 *   -------|------------------------------------------------------------
 *   y(n+1) | 35/384     0         500/1113    125/192 -2187/6784   11/84    0
 *   z(n+1) | 5179/57600 0         7571/16695  393/640 -92097/339200 187/2100 1/40
 * @endverbatim
 *
 * The 5th order solution (y) is used for propagation.
 * The 4th order solution (z) is used for error estimation.
 * Error = y - z
 *
 * Properties:
 * - 5th order accuracy for local error
 * - 4th order accuracy for global error
 * - FSAL (First Same As Last) reduces to 6 evaluations per step
 * - Dense output available for interpolation
 *
 * Use Cases:
 * - General-purpose ODE solving
 * - Variable-accuracy trajectory propagation
 * - Cases where adaptive stepping is beneficial
 * - Standard choice when unsure which integrator to use
 */
class DormandPrinceIntegrator : public IStatePropagator {
public:
    DormandPrinceIntegrator();
    ~DormandPrinceIntegrator() override = default;

    /**
     * @brief Integrate entity state by one time step
     *
     * Performs adaptive step using Dormand-Prince 5(4) method.
     * Step size is controlled internally for error tolerance.
     *
     * @param state Entity state (modified in place)
     * @param forces Total forces and torques in body frame
     * @param dt Time step (seconds)
     */
    void integrate(EntityState& state, const EntityForces& forces, Real dt) override;

    /**
     * @brief Get the name of this integrator
     * @return "DormandPrince" or "DOPRI5"
     */
    const std::string& name() const override { return name_; }

    /**
     * @brief Get integration order (5 for Dormand-Prince)
     */
    int order() const override { return 5; }

    /**
     * @brief Reset integrator state (clears FSAL cache)
     */
    void reset() override;

    // ========================================================================
    // Error Control Parameters
    // ========================================================================

    /**
     * @brief Set absolute error tolerance
     * @param atol Absolute tolerance (default: 1e-6)
     */
    void set_absolute_tolerance(Real atol) { atol_ = atol; }

    /**
     * @brief Get absolute error tolerance
     */
    Real get_absolute_tolerance() const { return atol_; }

    /**
     * @brief Set relative error tolerance
     * @param rtol Relative tolerance (default: 1e-6)
     */
    void set_relative_tolerance(Real rtol) { rtol_ = rtol; }

    /**
     * @brief Get relative error tolerance
     */
    Real get_relative_tolerance() const { return rtol_; }

    /**
     * @brief Set safety factor for step size control
     * @param safety Safety factor (default: 0.9)
     */
    void set_safety_factor(Real safety) { safety_ = safety; }

    /**
     * @brief Set minimum step size
     * @param min_dt Minimum time step (default: 1e-10)
     */
    void set_min_step(Real min_dt) { min_dt_ = min_dt; }

    /**
     * @brief Set maximum step size
     * @param max_dt Maximum time step (default: 1.0)
     */
    void set_max_step(Real max_dt) { max_dt_ = max_dt; }

    /**
     * @brief Set maximum number of substeps per call
     * @param max_steps Maximum substeps (default: 10000)
     */
    void set_max_substeps(int max_steps) { max_substeps_ = max_steps; }

    // ========================================================================
    // Output and Diagnostics
    // ========================================================================

    /**
     * @brief Get last error estimate
     */
    Real get_error_estimate() const { return error_estimate_; }

    /**
     * @brief Get suggested next time step
     */
    Real get_suggested_dt() const { return suggested_dt_; }

    /**
     * @brief Get number of substeps taken in last call
     */
    int get_last_substep_count() const { return last_substeps_; }

    /**
     * @brief Get number of rejected steps in last call
     */
    int get_rejected_steps() const { return rejected_steps_; }

    /**
     * @brief Get total function evaluations since last reset
     */
    int get_function_evaluations() const { return function_evals_; }

    /**
     * @brief Check if FSAL cache is valid for next step
     */
    bool has_fsal_cache() const { return fsal_valid_; }

private:
    std::string name_{"DormandPrince"};

    // Error control parameters
    Real atol_{1e-6};           ///< Absolute tolerance
    Real rtol_{1e-6};           ///< Relative tolerance
    Real safety_{0.9};          ///< Safety factor for step size
    Real min_dt_{1e-10};        ///< Minimum step size
    Real max_dt_{1.0};          ///< Maximum step size
    int max_substeps_{10000};   ///< Maximum substeps per call

    // Step size control factors
    static constexpr Real beta_{0.04};      ///< PI controller beta
    static constexpr Real fac_min_{0.2};    ///< Minimum step size factor
    static constexpr Real fac_max_{10.0};   ///< Maximum step size factor

    // Output/diagnostics
    Real error_estimate_{0.0};
    Real suggested_dt_{0.01};
    int last_substeps_{0};
    int rejected_steps_{0};
    int function_evals_{0};

    // FSAL cache (k7 from previous step = k1 for next step)
    bool fsal_valid_{false};
    Vec3 fsal_accel_{Vec3::Zero()};
    Vec3 fsal_angular_accel_{Vec3::Zero()};

    // Dormand-Prince coefficients
    static constexpr Real c2_{0.2};
    static constexpr Real c3_{0.3};
    static constexpr Real c4_{0.8};
    static constexpr Real c5_{8.0/9.0};
    // c6 = c7 = 1.0

    // Matrix A (lower triangular)
    static constexpr Real a21_{0.2};
    static constexpr Real a31_{3.0/40.0};
    static constexpr Real a32_{9.0/40.0};
    static constexpr Real a41_{44.0/45.0};
    static constexpr Real a42_{-56.0/15.0};
    static constexpr Real a43_{32.0/9.0};
    static constexpr Real a51_{19372.0/6561.0};
    static constexpr Real a52_{-25360.0/2187.0};
    static constexpr Real a53_{64448.0/6561.0};
    static constexpr Real a54_{-212.0/729.0};
    static constexpr Real a61_{9017.0/3168.0};
    static constexpr Real a62_{-355.0/33.0};
    static constexpr Real a63_{46732.0/5247.0};
    static constexpr Real a64_{49.0/176.0};
    static constexpr Real a65_{-5103.0/18656.0};
    static constexpr Real a71_{35.0/384.0};
    // a72 = 0
    static constexpr Real a73_{500.0/1113.0};
    static constexpr Real a74_{125.0/192.0};
    static constexpr Real a75_{-2187.0/6784.0};
    static constexpr Real a76_{11.0/84.0};

    // 5th order weights (b) - same as a7j
    static constexpr Real b1_{35.0/384.0};
    // b2 = 0
    static constexpr Real b3_{500.0/1113.0};
    static constexpr Real b4_{125.0/192.0};
    static constexpr Real b5_{-2187.0/6784.0};
    static constexpr Real b6_{11.0/84.0};
    // b7 = 0

    // Error coefficients (b - b*)
    static constexpr Real e1_{71.0/57600.0};
    // e2 = 0
    static constexpr Real e3_{-71.0/16695.0};
    static constexpr Real e4_{71.0/1920.0};
    static constexpr Real e5_{-17253.0/339200.0};
    static constexpr Real e6_{22.0/525.0};
    static constexpr Real e7_{-1.0/40.0};

    /**
     * @brief Perform a single Dormand-Prince step
     *
     * @param state Current state
     * @param forces Applied forces
     * @param dt Step size
     * @param[out] new_state Resulting state
     * @return Local error estimate
     */
    Real dopri_step(const EntityState& state, const EntityForces& forces,
                    Real dt, EntityState& new_state);

    /**
     * @brief Compute acceleration from forces
     */
    Vec3 compute_accel(const EntityState& state, const EntityForces& forces) const;

    /**
     * @brief Compute angular acceleration using Euler's equations
     */
    Vec3 compute_angular_accel(const EntityState& state, const EntityForces& forces) const;

    /**
     * @brief Compute error norm for step size control
     */
    Real error_norm(const Vec3& pos_err, const Vec3& vel_err,
                    const EntityState& state) const;

    /**
     * @brief Compute new step size based on error
     */
    Real compute_new_dt(Real dt, Real error, Real prev_error) const;
};

} // namespace jaguar::physics
