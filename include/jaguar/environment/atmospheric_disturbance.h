#pragma once
/**
 * @file atmospheric_disturbance.h
 * @brief Atmospheric turbulence and disturbance models
 *
 * This file implements military-standard turbulence models for flight simulation:
 *
 * 1. Dryden Turbulence Model (MIL-F-8785C)
 *    - Transfer function approach
 *    - Computationally efficient
 *    - Widely used in real-time simulation
 *
 * 2. Von Karman Turbulence Model (MIL-STD-1797)
 *    - More accurate spectral representation
 *    - Higher computational cost
 *    - Preferred for high-fidelity applications
 *
 * Both models generate turbulence velocity components (u_g, v_g, w_g) and
 * angular rate components (p_g, q_g, r_g) as functions of altitude and
 * airspeed.
 *
 * Reference Standards:
 * - MIL-F-8785C: Flying Qualities of Piloted Airplanes
 * - MIL-STD-1797A: Flying Qualities of Piloted Aircraft
 * - MIL-HDBK-1797: Department of Defense Handbook
 */

#include "jaguar/core/types.h"
#include <random>
#include <memory>
#include <string>

namespace jaguar::environment {

// ============================================================================
// Turbulence Severity
// ============================================================================

/**
 * @brief Turbulence severity levels per MIL-F-8785C
 */
enum class TurbulenceSeverity : UInt8 {
    None        = 0,    ///< No turbulence
    Light       = 1,    ///< Light turbulence (occasional light buffeting)
    Moderate    = 2,    ///< Moderate turbulence (rapid bumps, strain against seat belt)
    Severe      = 3,    ///< Severe turbulence (violent buffeting, aircraft hard to control)
    Extreme     = 4     ///< Extreme turbulence (aircraft practically uncontrollable)
};

/**
 * @brief Get turbulence intensity (σ/u) from severity level
 *
 * Returns the normalized turbulence intensity appropriate for the
 * given severity level, as defined in MIL-F-8785C Table I.
 *
 * @param severity Turbulence severity level
 * @param altitude_agl Altitude above ground level (m)
 * @return Turbulence intensity scaling factor
 */
Real get_turbulence_intensity(TurbulenceSeverity severity, Real altitude_agl);

/**
 * @brief Get scale length from altitude per MIL-F-8785C
 *
 * @param altitude_agl Altitude above ground level (m)
 * @param is_vertical True for vertical scale length (Lw), false for horizontal (Lu, Lv)
 * @return Scale length in meters
 */
Real get_scale_length(Real altitude_agl, bool is_vertical);

// ============================================================================
// Turbulence Output
// ============================================================================

/**
 * @brief Turbulence velocity and rate outputs
 */
struct TurbulenceOutput {
    // Linear velocity components in body frame (m/s)
    Vec3 velocity{0.0, 0.0, 0.0};  ///< (u_g, v_g, w_g)

    // Angular rate components in body frame (rad/s)
    Vec3 angular_rate{0.0, 0.0, 0.0};  ///< (p_g, q_g, r_g)

    /**
     * @brief Check if turbulence is negligible
     */
    bool is_negligible(Real threshold = 1e-6) const {
        return velocity.length() < threshold && angular_rate.length() < threshold;
    }
};

// ============================================================================
// Turbulence Model Interface
// ============================================================================

/**
 * @brief Base class for atmospheric turbulence models
 */
class ITurbulenceModel {
public:
    virtual ~ITurbulenceModel() = default;

    /**
     * @brief Update turbulence model and compute output
     *
     * @param airspeed True airspeed (m/s)
     * @param altitude_agl Altitude above ground level (m)
     * @param wingspan Aircraft wingspan for angular rate calculation (m)
     * @param dt Time step (s)
     * @return Turbulence velocity and angular rate components
     */
    virtual TurbulenceOutput update(Real airspeed, Real altitude_agl,
                                    Real wingspan, Real dt) = 0;

    /**
     * @brief Reset turbulence state
     */
    virtual void reset() = 0;

    /**
     * @brief Set turbulence severity
     */
    virtual void set_severity(TurbulenceSeverity severity) = 0;

    /**
     * @brief Get current turbulence severity
     */
    virtual TurbulenceSeverity severity() const = 0;

    /**
     * @brief Get model name
     */
    virtual const std::string& name() const = 0;

    /**
     * @brief Enable/disable turbulence
     */
    virtual void set_enabled(bool enabled) = 0;

    /**
     * @brief Check if turbulence is enabled
     */
    virtual bool is_enabled() const = 0;

    /**
     * @brief Set random seed for reproducibility
     */
    virtual void set_seed(UInt64 seed) = 0;
};

// ============================================================================
// Dryden Turbulence Model
// ============================================================================

/**
 * @brief Dryden continuous gust model per MIL-F-8785C
 *
 * The Dryden model uses forming filters driven by white noise to generate
 * turbulence with the proper spectral characteristics. It's computationally
 * efficient and suitable for real-time simulation.
 *
 * The model generates turbulence components as the output of first and second
 * order linear filters with transfer functions:
 *
 * Longitudinal (u):
 *   H_u(s) = σ_u * sqrt(2*L_u/(π*V)) / (1 + L_u*s/V)
 *
 * Lateral (v):
 *   H_v(s) = σ_v * sqrt(L_v/(π*V)) * (1 + sqrt(3)*L_v*s/V) / (1 + L_v*s/V)²
 *
 * Vertical (w):
 *   H_w(s) = σ_w * sqrt(L_w/(π*V)) * (1 + sqrt(3)*L_w*s/V) / (1 + L_w*s/V)²
 *
 * Scale lengths (L) and intensities (σ) are functions of altitude.
 *
 * Usage:
 * @code
 * DrydenTurbulenceModel turbulence;
 * turbulence.set_severity(TurbulenceSeverity::Moderate);
 *
 * // In simulation loop:
 * TurbulenceOutput output = turbulence.update(airspeed, altitude_agl, wingspan, dt);
 *
 * // Add to aircraft velocities:
 * body_velocity += output.velocity;
 * angular_rate += output.angular_rate;
 * @endcode
 */
class DrydenTurbulenceModel : public ITurbulenceModel {
public:
    /**
     * @brief Construct with default moderate turbulence
     */
    DrydenTurbulenceModel();

    /**
     * @brief Construct with specified severity
     */
    explicit DrydenTurbulenceModel(TurbulenceSeverity severity);

    ~DrydenTurbulenceModel() override = default;

    // ITurbulenceModel interface
    TurbulenceOutput update(Real airspeed, Real altitude_agl,
                            Real wingspan, Real dt) override;
    void reset() override;
    void set_severity(TurbulenceSeverity severity) override;
    TurbulenceSeverity severity() const override { return severity_; }
    const std::string& name() const override { return name_; }
    void set_enabled(bool enabled) override { enabled_ = enabled; }
    bool is_enabled() const override { return enabled_; }
    void set_seed(UInt64 seed) override;

    // ========================================================================
    // Dryden-specific Configuration
    // ========================================================================

    /**
     * @brief Set custom intensity values (overrides severity-based)
     *
     * @param sigma_u Longitudinal intensity (m/s)
     * @param sigma_v Lateral intensity (m/s)
     * @param sigma_w Vertical intensity (m/s)
     */
    void set_intensity(Real sigma_u, Real sigma_v, Real sigma_w);

    /**
     * @brief Set custom scale lengths (overrides altitude-based)
     *
     * @param L_u Longitudinal scale length (m)
     * @param L_v Lateral scale length (m)
     * @param L_w Vertical scale length (m)
     */
    void set_scale_lengths(Real L_u, Real L_v, Real L_w);

    /**
     * @brief Enable/disable altitude-dependent parameters
     */
    void set_altitude_dependent(bool enabled) { altitude_dependent_ = enabled; }

    /**
     * @brief Get current intensity values
     */
    Vec3 get_intensity() const { return Vec3{sigma_u_, sigma_v_, sigma_w_}; }

    /**
     * @brief Get current scale lengths
     */
    Vec3 get_scale_lengths() const { return Vec3{L_u_, L_v_, L_w_}; }

    /**
     * @brief Get last computed output
     */
    const TurbulenceOutput& last_output() const { return last_output_; }

private:
    std::string name_{"Dryden"};
    TurbulenceSeverity severity_{TurbulenceSeverity::Moderate};
    bool enabled_{true};
    bool altitude_dependent_{true};

    // Turbulence parameters
    Real sigma_u_{1.0}, sigma_v_{1.0}, sigma_w_{1.0};  // Intensities (m/s)
    Real L_u_{200.0}, L_v_{200.0}, L_w_{50.0};         // Scale lengths (m)

    // Filter states for u, v, w components
    Real u_state_{0.0};
    Real v_state_[2]{0.0, 0.0};  // Second-order filter needs 2 states
    Real w_state_[2]{0.0, 0.0};

    // Random number generation
    std::mt19937_64 rng_;
    std::normal_distribution<Real> normal_dist_{0.0, 1.0};

    // Last output
    TurbulenceOutput last_output_;

    // Update altitude-dependent parameters
    void update_parameters(Real altitude_agl);

    // Generate white noise
    Real white_noise() { return normal_dist_(rng_); }

    // First-order filter: G(s) = K / (1 + τs)
    Real first_order_filter(Real& state, Real input, Real K, Real tau, Real dt);

    // Second-order filter: G(s) = K(1 + as) / (1 + bs)² (Dryden form)
    Real second_order_filter(Real* state, Real input, Real K, Real a, Real b, Real dt);
};

// ============================================================================
// Von Karman Turbulence Model
// ============================================================================

/**
 * @brief Von Karman turbulence model per MIL-STD-1797
 *
 * The Von Karman model provides a more accurate representation of atmospheric
 * turbulence spectra than the Dryden model, particularly at higher frequencies.
 * It uses fractional-order filters that more closely match measured turbulence
 * data.
 *
 * Power Spectral Densities:
 *
 * Longitudinal:
 *   Φ_u(Ω) = σ_u² * (2*L_u/π) / (1 + (1.339*L_u*Ω)²)^(5/6)
 *
 * Lateral/Vertical:
 *   Φ_v(Ω) = σ_v² * (L_v/π) * (1 + (8/3)*(2.678*L_v*Ω)²) / (1 + (2.678*L_v*Ω)²)^(11/6)
 *
 * Where Ω = ω/V is the spatial frequency.
 *
 * Implementation uses a higher-order approximation to achieve the fractional
 * filter characteristics through cascaded filter sections.
 *
 * Note: More computationally expensive than Dryden but provides better
 * spectral accuracy, especially important for:
 * - High-fidelity pilot training simulators
 * - Structural loads analysis
 * - Ride quality assessment
 */
class VonKarmanTurbulenceModel : public ITurbulenceModel {
public:
    /**
     * @brief Construct with default moderate turbulence
     */
    VonKarmanTurbulenceModel();

    /**
     * @brief Construct with specified severity
     */
    explicit VonKarmanTurbulenceModel(TurbulenceSeverity severity);

    ~VonKarmanTurbulenceModel() override = default;

    // ITurbulenceModel interface
    TurbulenceOutput update(Real airspeed, Real altitude_agl,
                            Real wingspan, Real dt) override;
    void reset() override;
    void set_severity(TurbulenceSeverity severity) override;
    TurbulenceSeverity severity() const override { return severity_; }
    const std::string& name() const override { return name_; }
    void set_enabled(bool enabled) override { enabled_ = enabled; }
    bool is_enabled() const override { return enabled_; }
    void set_seed(UInt64 seed) override;

    // ========================================================================
    // Von Karman-specific Configuration
    // ========================================================================

    /**
     * @brief Set filter order (higher = more accurate but slower)
     *
     * Default is 4, which provides good accuracy for most applications.
     * Values of 6-8 recommended for structural analysis.
     *
     * @param order Filter order (2-8)
     */
    void set_filter_order(int order);

    /**
     * @brief Get current filter order
     */
    int filter_order() const { return filter_order_; }

    /**
     * @brief Set custom intensity values
     */
    void set_intensity(Real sigma_u, Real sigma_v, Real sigma_w);

    /**
     * @brief Set custom scale lengths
     */
    void set_scale_lengths(Real L_u, Real L_v, Real L_w);

private:
    std::string name_{"VonKarman"};
    TurbulenceSeverity severity_{TurbulenceSeverity::Moderate};
    bool enabled_{true};
    int filter_order_{4};

    // Turbulence parameters
    Real sigma_u_{1.0}, sigma_v_{1.0}, sigma_w_{1.0};
    Real L_u_{200.0}, L_v_{200.0}, L_w_{50.0};

    // Filter state vectors (variable size based on filter_order_)
    static constexpr int MAX_FILTER_ORDER = 8;
    Real u_states_[MAX_FILTER_ORDER]{};
    Real v_states_[MAX_FILTER_ORDER]{};
    Real w_states_[MAX_FILTER_ORDER]{};

    // Random number generation
    std::mt19937_64 rng_;
    std::normal_distribution<Real> normal_dist_{0.0, 1.0};

    // Update parameters based on altitude
    void update_parameters(Real altitude_agl);

    // Generate filtered output using cascaded filters
    Real filter_longitudinal(Real input, Real V, Real dt);
    Real filter_lateral_vertical(Real input, Real V, Real L, Real sigma,
                                 Real* states, Real dt);
};

// ============================================================================
// Wind Model (Steady + Gusts)
// ============================================================================

/**
 * @brief Complete wind disturbance model
 *
 * Combines steady wind field with turbulence models to provide
 * total atmospheric disturbance.
 */
class WindModel {
public:
    WindModel();
    ~WindModel() = default;

    /**
     * @brief Set steady wind velocity in NED frame
     */
    void set_steady_wind(const Vec3& wind_ned);

    /**
     * @brief Set steady wind from speed and direction
     *
     * @param speed Wind speed (m/s)
     * @param heading Wind direction FROM (rad, 0 = North, π/2 = East)
     */
    void set_steady_wind(Real speed, Real heading);

    /**
     * @brief Get steady wind in NED frame
     */
    const Vec3& steady_wind() const { return steady_wind_; }

    /**
     * @brief Set wind shear gradient (change in wind with altitude)
     *
     * Models the boundary layer wind profile where wind increases with altitude.
     * Uses power law: V(h) = V_ref * (h/h_ref)^alpha
     *
     * @param alpha Wind shear exponent (typical: 0.14 for neutral stability)
     * @param reference_altitude Reference altitude for steady_wind_ (m)
     */
    void set_wind_shear(Real alpha, Real reference_altitude);

    /**
     * @brief Set turbulence model
     */
    void set_turbulence_model(std::unique_ptr<ITurbulenceModel> model);

    /**
     * @brief Get turbulence model
     */
    ITurbulenceModel* turbulence_model() { return turbulence_.get(); }
    const ITurbulenceModel* turbulence_model() const { return turbulence_.get(); }

    /**
     * @brief Update wind model and compute total disturbance
     *
     * @param position_ned Aircraft position in NED (for altitude)
     * @param body_to_ned Rotation from body to NED frame
     * @param airspeed True airspeed (m/s)
     * @param wingspan Aircraft wingspan (m)
     * @param dt Time step (s)
     * @return Wind velocity in body frame (m/s)
     */
    Vec3 update(const Vec3& position_ned, const Quat& body_to_ned,
                Real airspeed, Real wingspan, Real dt);

    /**
     * @brief Get wind at specific altitude (with shear)
     */
    Vec3 get_wind_at_altitude(Real altitude_agl) const;

    /**
     * @brief Get last turbulence output
     */
    const TurbulenceOutput& last_turbulence() const { return last_turbulence_; }

    /**
     * @brief Enable/disable model
     */
    void set_enabled(bool enabled) { enabled_ = enabled; }

    /**
     * @brief Check if model is enabled
     */
    bool is_enabled() const { return enabled_; }

    /**
     * @brief Reset model state
     */
    void reset();

private:
    bool enabled_{true};

    // Steady wind
    Vec3 steady_wind_{0.0, 0.0, 0.0};  // NED frame

    // Wind shear parameters
    bool shear_enabled_{false};
    Real shear_alpha_{0.14};
    Real shear_reference_altitude_{10.0};

    // Turbulence model
    std::unique_ptr<ITurbulenceModel> turbulence_;

    // Last outputs
    TurbulenceOutput last_turbulence_;
};

// ============================================================================
// Factory Functions
// ============================================================================

/**
 * @brief Create Dryden turbulence model with given severity
 */
std::unique_ptr<ITurbulenceModel> create_dryden_turbulence(
    TurbulenceSeverity severity = TurbulenceSeverity::Moderate);

/**
 * @brief Create Von Karman turbulence model with given severity
 */
std::unique_ptr<ITurbulenceModel> create_vonkarman_turbulence(
    TurbulenceSeverity severity = TurbulenceSeverity::Moderate);

/**
 * @brief Create complete wind model with turbulence
 */
std::unique_ptr<WindModel> create_wind_model(
    const Vec3& steady_wind = Vec3{0.0, 0.0, 0.0},
    TurbulenceSeverity turbulence = TurbulenceSeverity::Moderate,
    bool use_vonkarman = false);

} // namespace jaguar::environment
