/**
 * @file dryden_turbulence.cpp
 * @brief Dryden continuous gust turbulence model implementation
 *
 * Implements MIL-F-8785C Dryden turbulence model using forming filters.
 */

#include "jaguar/environment/atmospheric_disturbance.h"
#include <cmath>
#include <algorithm>

namespace jaguar::environment {

// ============================================================================
// Turbulence Utility Functions
// ============================================================================

Real get_turbulence_intensity(TurbulenceSeverity severity, Real altitude_agl) {
    // Turbulence intensity (σ) values per MIL-F-8785C Table I
    // Values are RMS gust velocities in ft/s, converted to m/s

    // Base intensities at 1000 ft (low altitude) in ft/s
    Real sigma_base_fps = 0.0;

    switch (severity) {
        case TurbulenceSeverity::None:
            return 0.0;

        case TurbulenceSeverity::Light:
            // Light: P(σ > value) = 10^-1
            sigma_base_fps = 3.0;  // ~1 m/s
            break;

        case TurbulenceSeverity::Moderate:
            // Moderate: P(σ > value) = 10^-2
            sigma_base_fps = 6.0;  // ~2 m/s
            break;

        case TurbulenceSeverity::Severe:
            // Severe: P(σ > value) = 10^-3
            sigma_base_fps = 12.0;  // ~4 m/s
            break;

        case TurbulenceSeverity::Extreme:
            // Extreme: P(σ > value) = 10^-5
            sigma_base_fps = 24.0;  // ~8 m/s
            break;
    }

    // Convert ft/s to m/s
    Real sigma_base = sigma_base_fps * 0.3048;

    // Altitude scaling per MIL-F-8785C
    // Low altitude (h < 1000 ft): σ = σ_base * (h/1000)^(1/3)
    // Medium altitude (1000 ft < h < 2000 ft): σ = σ_base
    // High altitude (h > 2000 ft): σ varies with wind profile

    Real h_ft = altitude_agl / 0.3048;  // Convert to feet

    if (h_ft < 10.0) {
        // Very low altitude - limit minimum
        return sigma_base * std::pow(10.0 / 1000.0, 1.0 / 3.0);
    } else if (h_ft < 1000.0) {
        // Low altitude scaling
        return sigma_base * std::pow(h_ft / 1000.0, 1.0 / 3.0);
    } else if (h_ft < 2000.0) {
        // Medium altitude - constant
        return sigma_base;
    } else {
        // High altitude - gradual decrease
        // Simplified model: decrease above 2000 ft
        return sigma_base * std::pow(2000.0 / h_ft, 0.5);
    }
}

Real get_scale_length(Real altitude_agl, bool is_vertical) {
    // Scale lengths per MIL-F-8785C
    // Low altitude (h < 1000 ft): L_u = L_v = h / (0.177 + 0.000823*h)^1.2
    //                             L_w = h
    // High altitude (h >= 1000 ft): L_u = L_v = 1750 ft, L_w = 1750 ft

    Real h_ft = altitude_agl / 0.3048;  // Convert to feet

    // Minimum altitude for calculations
    h_ft = std::max(h_ft, 10.0);

    Real L_ft;

    if (h_ft < 1000.0) {
        // Low altitude model
        if (is_vertical) {
            L_ft = h_ft;  // L_w = h
        } else {
            // L_u = L_v = h / (0.177 + 0.000823*h)^1.2
            Real denom = std::pow(0.177 + 0.000823 * h_ft, 1.2);
            L_ft = h_ft / denom;
        }
    } else {
        // High altitude - constant scale length
        L_ft = 1750.0;  // ~533 m
    }

    // Convert feet to meters
    return L_ft * 0.3048;
}

// ============================================================================
// Dryden Turbulence Model
// ============================================================================

DrydenTurbulenceModel::DrydenTurbulenceModel()
    : severity_(TurbulenceSeverity::Moderate) {
    // Initialize RNG with a default seed
    rng_.seed(42);
}

DrydenTurbulenceModel::DrydenTurbulenceModel(TurbulenceSeverity severity)
    : severity_(severity) {
    rng_.seed(42);
}

void DrydenTurbulenceModel::set_seed(UInt64 seed) {
    rng_.seed(seed);
}

void DrydenTurbulenceModel::set_severity(TurbulenceSeverity severity) {
    severity_ = severity;
}

void DrydenTurbulenceModel::reset() {
    u_state_ = 0.0;
    v_state_[0] = v_state_[1] = 0.0;
    w_state_[0] = w_state_[1] = 0.0;
    last_output_ = TurbulenceOutput{};
}

void DrydenTurbulenceModel::set_intensity(Real sigma_u, Real sigma_v, Real sigma_w) {
    sigma_u_ = sigma_u;
    sigma_v_ = sigma_v;
    sigma_w_ = sigma_w;
    altitude_dependent_ = false;  // Disable automatic calculation
}

void DrydenTurbulenceModel::set_scale_lengths(Real L_u, Real L_v, Real L_w) {
    L_u_ = L_u;
    L_v_ = L_v;
    L_w_ = L_w;
    altitude_dependent_ = false;
}

void DrydenTurbulenceModel::update_parameters(Real altitude_agl) {
    if (!altitude_dependent_) return;

    // Get intensity from severity and altitude
    Real sigma = get_turbulence_intensity(severity_, altitude_agl);
    sigma_u_ = sigma;
    sigma_v_ = sigma;
    sigma_w_ = sigma;

    // Get scale lengths
    L_u_ = get_scale_length(altitude_agl, false);
    L_v_ = get_scale_length(altitude_agl, false);
    L_w_ = get_scale_length(altitude_agl, true);
}

Real DrydenTurbulenceModel::first_order_filter(Real& state, Real input,
                                                Real K, Real tau, Real dt) {
    // First-order filter: G(s) = K / (1 + τs)
    // Discrete: y[n] = α * y[n-1] + K * (1-α) * x[n]
    // where α = exp(-dt/τ)

    if (tau <= 0.0) {
        state = K * input;
        return state;
    }

    Real alpha = std::exp(-dt / tau);
    state = alpha * state + K * (1.0 - alpha) * input;
    return state;
}

Real DrydenTurbulenceModel::second_order_filter(Real* state, Real input,
                                                 Real K, Real a, Real b, Real dt) {
    // Second-order Dryden filter: G(s) = K * (1 + a*s) / (1 + b*s)²
    // Implemented as cascade: numerator dynamics + two first-order poles

    // Using state-space form for better numerical properties
    // Let τ = b (time constant)

    if (b <= 0.0) {
        state[0] = K * input;
        state[1] = 0.0;
        return state[0];
    }

    Real alpha = std::exp(-dt / b);

    // First pole
    Real x1_new = alpha * state[0] + (1.0 - alpha) * input;

    // Second pole (cascaded)
    Real x2_new = alpha * state[1] + (1.0 - alpha) * x1_new;

    // Apply gain and derivative term
    // The (1 + as) term adds derivative action
    Real derivative_contribution = 0.0;
    if (a > 0.0 && dt > 0.0) {
        derivative_contribution = a * (x2_new - state[1]) / dt;
    }

    state[0] = x1_new;
    state[1] = x2_new;

    return K * (state[1] + derivative_contribution);
}

TurbulenceOutput DrydenTurbulenceModel::update(Real airspeed, Real altitude_agl,
                                                Real wingspan, Real dt) {
    TurbulenceOutput output;

    if (!enabled_ || severity_ == TurbulenceSeverity::None || airspeed < 1.0) {
        last_output_ = output;
        return output;
    }

    // Update altitude-dependent parameters
    update_parameters(altitude_agl);

    Real V = airspeed;

    // Generate white noise inputs
    Real noise_u = white_noise();
    Real noise_v = white_noise();
    Real noise_w = white_noise();

    // ========================================================================
    // Longitudinal (u_g) - First-order filter
    // H_u(s) = σ_u * sqrt(2*L_u/(π*V)) / (1 + L_u*s/V)
    // ========================================================================
    Real K_u = sigma_u_ * std::sqrt(2.0 * L_u_ / (constants::PI * V));
    Real tau_u = L_u_ / V;
    output.velocity.x = first_order_filter(u_state_, noise_u, K_u, tau_u, dt);

    // ========================================================================
    // Lateral (v_g) - Second-order filter
    // H_v(s) = σ_v * sqrt(L_v/(π*V)) * (1 + sqrt(3)*L_v*s/V) / (1 + L_v*s/V)²
    // ========================================================================
    Real K_v = sigma_v_ * std::sqrt(L_v_ / (constants::PI * V));
    Real a_v = std::sqrt(3.0) * L_v_ / V;  // Numerator time constant
    Real b_v = L_v_ / V;                    // Denominator time constant
    output.velocity.y = second_order_filter(v_state_, noise_v, K_v, a_v, b_v, dt);

    // ========================================================================
    // Vertical (w_g) - Second-order filter
    // H_w(s) = σ_w * sqrt(L_w/(π*V)) * (1 + sqrt(3)*L_w*s/V) / (1 + L_w*s/V)²
    // ========================================================================
    Real K_w = sigma_w_ * std::sqrt(L_w_ / (constants::PI * V));
    Real a_w = std::sqrt(3.0) * L_w_ / V;
    Real b_w = L_w_ / V;
    output.velocity.z = second_order_filter(w_state_, noise_w, K_w, a_w, b_w, dt);

    // ========================================================================
    // Angular rates from spatial gradients
    // p_g = dw_g/dy ≈ w_g / L_v (roll rate)
    // q_g = dw_g/dx ≈ dw_g/dt * 1/V (pitch rate)
    // r_g = dv_g/dx ≈ dv_g/dt * 1/V (yaw rate)
    // ========================================================================
    if (wingspan > 0.0 && dt > 0.0) {
        // Roll rate from vertical gust gradient across wingspan
        // p_g ≈ sqrt(0.8/3) * (π/(4*b))^(1/6) * (V/L_w)^(1/3) * w_g / b
        Real b = wingspan;
        output.angular_rate.x = 0.5 * output.velocity.z / (b * 0.5);

        // Pitch and yaw rates from gust derivatives
        // These are typically smaller than roll rate
        static Real prev_w = 0.0;
        static Real prev_v = 0.0;

        output.angular_rate.y = 0.3 * (output.velocity.z - prev_w) / (V * dt + 1e-6);
        output.angular_rate.z = 0.3 * (output.velocity.y - prev_v) / (V * dt + 1e-6);

        prev_w = output.velocity.z;
        prev_v = output.velocity.y;
    }

    last_output_ = output;
    return output;
}

// ============================================================================
// Von Karman Turbulence Model
// ============================================================================

VonKarmanTurbulenceModel::VonKarmanTurbulenceModel()
    : severity_(TurbulenceSeverity::Moderate)
    , filter_order_(4) {
    rng_.seed(12345);
    std::fill(std::begin(u_states_), std::end(u_states_), 0.0);
    std::fill(std::begin(v_states_), std::end(v_states_), 0.0);
    std::fill(std::begin(w_states_), std::end(w_states_), 0.0);
}

VonKarmanTurbulenceModel::VonKarmanTurbulenceModel(TurbulenceSeverity severity)
    : severity_(severity)
    , filter_order_(4) {
    rng_.seed(12345);
    std::fill(std::begin(u_states_), std::end(u_states_), 0.0);
    std::fill(std::begin(v_states_), std::end(v_states_), 0.0);
    std::fill(std::begin(w_states_), std::end(w_states_), 0.0);
}

void VonKarmanTurbulenceModel::set_seed(UInt64 seed) {
    rng_.seed(seed);
}

void VonKarmanTurbulenceModel::set_severity(TurbulenceSeverity severity) {
    severity_ = severity;
}

void VonKarmanTurbulenceModel::set_filter_order(int order) {
    filter_order_ = std::clamp(order, 2, MAX_FILTER_ORDER);
}

void VonKarmanTurbulenceModel::set_intensity(Real sigma_u, Real sigma_v, Real sigma_w) {
    sigma_u_ = sigma_u;
    sigma_v_ = sigma_v;
    sigma_w_ = sigma_w;
}

void VonKarmanTurbulenceModel::set_scale_lengths(Real L_u, Real L_v, Real L_w) {
    L_u_ = L_u;
    L_v_ = L_v;
    L_w_ = L_w;
}

void VonKarmanTurbulenceModel::reset() {
    std::fill(std::begin(u_states_), std::end(u_states_), 0.0);
    std::fill(std::begin(v_states_), std::end(v_states_), 0.0);
    std::fill(std::begin(w_states_), std::end(w_states_), 0.0);
}

void VonKarmanTurbulenceModel::update_parameters(Real altitude_agl) {
    Real sigma = get_turbulence_intensity(severity_, altitude_agl);
    sigma_u_ = sigma;
    sigma_v_ = sigma;
    sigma_w_ = sigma;

    L_u_ = get_scale_length(altitude_agl, false);
    L_v_ = get_scale_length(altitude_agl, false);
    L_w_ = get_scale_length(altitude_agl, true);
}

Real VonKarmanTurbulenceModel::filter_longitudinal(Real input, Real V, Real dt) {
    // Von Karman longitudinal spectrum:
    // Φ_u(Ω) = σ_u² * (2*L_u/π) / (1 + (1.339*L_u*Ω)²)^(5/6)
    //
    // Approximated using cascaded first-order filters

    Real a = 1.339 * L_u_ / V;  // Time constant factor
    Real K = sigma_u_ * std::sqrt(2.0 * L_u_ / (constants::PI * V));

    // Apply cascaded filters for 5/6 roll-off approximation
    Real alpha = std::exp(-dt / a);
    Real value = input;

    // Each cascaded section contributes to the overall roll-off
    for (int i = 0; i < std::min(filter_order_, MAX_FILTER_ORDER); ++i) {
        u_states_[i] = alpha * u_states_[i] + (1.0 - alpha) * value;
        value = u_states_[i];
    }

    return K * value;
}

Real VonKarmanTurbulenceModel::filter_lateral_vertical(Real input, Real V, Real L, Real sigma,
                                                        Real* states, Real dt) {
    // Von Karman lateral/vertical spectrum:
    // Φ_v(Ω) = σ² * (L/π) * (1 + (8/3)*(2.678*L*Ω)²) / (1 + (2.678*L*Ω)²)^(11/6)
    //
    // The 11/6 roll-off requires careful approximation

    Real a = 2.678 * L / V;
    Real K = sigma * std::sqrt(L / (constants::PI * V));

    Real alpha = std::exp(-dt / a);
    Real value = input;

    // Apply cascaded filters
    for (int i = 0; i < std::min(filter_order_, MAX_FILTER_ORDER); ++i) {
        states[i] = alpha * states[i] + (1.0 - alpha) * value;
        value = states[i];
    }

    // Add high-frequency content from the (1 + (8/3)*(...)²) numerator
    // Simplified: boost the output slightly based on input derivative
    Real boost = 1.0 + 0.3 * std::abs(input);

    return K * value * boost;
}

TurbulenceOutput VonKarmanTurbulenceModel::update(Real airspeed, Real altitude_agl,
                                                   Real wingspan, Real dt) {
    TurbulenceOutput output;

    if (!enabled_ || severity_ == TurbulenceSeverity::None || airspeed < 1.0) {
        return output;
    }

    update_parameters(altitude_agl);

    // Generate white noise
    Real noise_u = normal_dist_(rng_);
    Real noise_v = normal_dist_(rng_);
    Real noise_w = normal_dist_(rng_);

    // Apply Von Karman filters
    output.velocity.x = filter_longitudinal(noise_u, airspeed, dt);
    output.velocity.y = filter_lateral_vertical(noise_v, airspeed, L_v_, sigma_v_, v_states_, dt);
    output.velocity.z = filter_lateral_vertical(noise_w, airspeed, L_w_, sigma_w_, w_states_, dt);

    // Angular rates (similar to Dryden)
    if (wingspan > 0.0) {
        output.angular_rate.x = 0.5 * output.velocity.z / (wingspan * 0.5);
        output.angular_rate.y = 0.2 * output.velocity.z / (L_w_ + 1.0);
        output.angular_rate.z = 0.2 * output.velocity.y / (L_v_ + 1.0);
    }

    return output;
}

// ============================================================================
// Wind Model
// ============================================================================

WindModel::WindModel() = default;

void WindModel::set_steady_wind(const Vec3& wind_ned) {
    steady_wind_ = wind_ned;
}

void WindModel::set_steady_wind(Real speed, Real heading) {
    // Wind heading is direction FROM which wind blows
    // Convert to NED velocity (direction wind is going TO)
    steady_wind_.x = -speed * std::cos(heading);  // North component
    steady_wind_.y = -speed * std::sin(heading);  // East component
    steady_wind_.z = 0.0;                          // Down component
}

void WindModel::set_wind_shear(Real alpha, Real reference_altitude) {
    shear_enabled_ = true;
    shear_alpha_ = alpha;
    shear_reference_altitude_ = reference_altitude;
}

void WindModel::set_turbulence_model(std::unique_ptr<ITurbulenceModel> model) {
    turbulence_ = std::move(model);
}

Vec3 WindModel::get_wind_at_altitude(Real altitude_agl) const {
    Vec3 wind = steady_wind_;

    if (shear_enabled_ && altitude_agl > 0.0 && shear_reference_altitude_ > 0.0) {
        // Power law wind profile: V(h) = V_ref * (h/h_ref)^alpha
        Real ratio = std::pow(altitude_agl / shear_reference_altitude_, shear_alpha_);
        wind = wind * ratio;
    }

    return wind;
}

Vec3 WindModel::update(const Vec3& position_ned, const Quat& body_to_ned,
                       Real airspeed, Real wingspan, Real dt) {
    if (!enabled_) {
        return Vec3{0.0, 0.0, 0.0};
    }

    // Get altitude from position (NED: z is down, so altitude = -z)
    Real altitude_agl = -position_ned.z;
    altitude_agl = std::max(altitude_agl, 0.0);

    // Get steady wind at this altitude (in NED frame)
    Vec3 wind_ned = get_wind_at_altitude(altitude_agl);

    // Add turbulence if enabled
    if (turbulence_ && turbulence_->is_enabled()) {
        last_turbulence_ = turbulence_->update(airspeed, altitude_agl, wingspan, dt);

        // Turbulence is in body frame, convert to NED and add
        // For now, keep turbulence in body frame as it will be used there
    }

    // Transform steady wind from NED to body frame
    // body_to_ned rotates body->NED, so we need ned_to_body = inverse
    Quat ned_to_body = body_to_ned.conjugate();

    // Rotate wind vector to body frame
    Vec3 wind_body = ned_to_body.rotate(wind_ned);

    // Add turbulence (already in body frame)
    wind_body = wind_body + last_turbulence_.velocity;

    return wind_body;
}

void WindModel::reset() {
    if (turbulence_) {
        turbulence_->reset();
    }
    last_turbulence_ = TurbulenceOutput{};
}

// ============================================================================
// Factory Functions
// ============================================================================

std::unique_ptr<ITurbulenceModel> create_dryden_turbulence(TurbulenceSeverity severity) {
    return std::make_unique<DrydenTurbulenceModel>(severity);
}

std::unique_ptr<ITurbulenceModel> create_vonkarman_turbulence(TurbulenceSeverity severity) {
    return std::make_unique<VonKarmanTurbulenceModel>(severity);
}

std::unique_ptr<WindModel> create_wind_model(const Vec3& steady_wind,
                                              TurbulenceSeverity turbulence,
                                              bool use_vonkarman) {
    auto model = std::make_unique<WindModel>();
    model->set_steady_wind(steady_wind);

    if (turbulence != TurbulenceSeverity::None) {
        if (use_vonkarman) {
            model->set_turbulence_model(create_vonkarman_turbulence(turbulence));
        } else {
            model->set_turbulence_model(create_dryden_turbulence(turbulence));
        }
    }

    return model;
}

} // namespace jaguar::environment
