/**
 * @file standard_atmosphere.cpp
 * @brief US Standard Atmosphere 1976 implementation
 *
 * Implements the US Standard Atmosphere 1976 model for altitudes
 * from 0 to 86 km (geometric altitude). The atmosphere is divided
 * into layers with different temperature gradients.
 *
 * Reference: US Standard Atmosphere, 1976 (NOAA/NASA/USAF)
 */

#include "jaguar/environment/atmosphere.h"
#include <cmath>
#include <algorithm>
#include <vector>

namespace jaguar::environment {

// ============================================================================
// US Standard Atmosphere 1976 Constants
// ============================================================================

namespace {

// Physical constants
constexpr Real R_STAR = 8.31432;          // Universal gas constant (J/(mol·K))
constexpr Real M0 = 28.9644e-3;           // Mean molecular weight at sea level (kg/mol)
constexpr Real R_AIR = R_STAR / M0;       // Gas constant for air (287.053 J/(kg·K))
constexpr Real GAMMA = 1.4;               // Ratio of specific heats
constexpr Real G0 = 9.80665;              // Standard gravity (m/s²)
constexpr Real R_EARTH = 6356766.0;       // Effective Earth radius for geopotential (m)

// Sea level values
constexpr Real P0 = 101325.0;             // Pressure (Pa)
// Note: T0 = 288.15 K, RHO0 = 1.225 kg/m³ are defined in layer data

// Atmospheric layer structure
// Each layer is defined by:
// - Base geopotential altitude (m)
// - Base temperature (K)
// - Temperature lapse rate (K/m)

struct AtmosphereLayer {
    Real H_base;        // Geopotential altitude at base (m)
    Real T_base;        // Temperature at base (K)
    Real lapse_rate;    // Temperature gradient (K/m), negative = decreasing
    Real P_base;        // Pressure at base (Pa) - computed
};

// US Standard Atmosphere 1976 layers (geopotential altitudes)
// Layer 0: Troposphere (0-11 km)
// Layer 1: Tropopause (11-20 km) - isothermal
// Layer 2: Stratosphere 1 (20-32 km)
// Layer 3: Stratosphere 2 (32-47 km)
// Layer 4: Stratopause (47-51 km) - isothermal
// Layer 5: Mesosphere 1 (51-71 km)
// Layer 6: Mesosphere 2 (71-84.852 km)

constexpr int NUM_LAYERS = 7;

// Layer data: H_base (m), T_base (K), lapse_rate (K/m)
constexpr Real LAYER_DATA[NUM_LAYERS][3] = {
    {     0.0, 288.15, -0.0065  },   // Troposphere
    { 11000.0, 216.65,  0.0     },   // Tropopause (isothermal)
    { 20000.0, 216.65,  0.001   },   // Stratosphere 1
    { 32000.0, 228.65,  0.0028  },   // Stratosphere 2
    { 47000.0, 270.65,  0.0     },   // Stratopause (isothermal)
    { 51000.0, 270.65, -0.0028  },   // Mesosphere 1
    { 71000.0, 214.65, -0.002   }    // Mesosphere 2
};

// Top of atmosphere model (geopotential altitude)
constexpr Real H_MAX = 84852.0;  // ~86 km geometric

/**
 * @brief Get the layer index for a given geopotential altitude
 */
int get_layer_index(Real H) {
    if (H < 0.0) return 0;
    for (int i = NUM_LAYERS - 1; i >= 0; --i) {
        if (H >= LAYER_DATA[i][0]) {
            return i;
        }
    }
    return 0;
}

/**
 * @brief Compute pressure at base of each layer (called once at init)
 */
std::vector<Real> compute_layer_pressures() {
    std::vector<Real> P(static_cast<size_t>(NUM_LAYERS));
    P[0] = P0;

    for (size_t i = 0; i < static_cast<size_t>(NUM_LAYERS - 1); ++i) {
        size_t next = i + 1;
        Real H_b = LAYER_DATA[i][0];
        Real T_b = LAYER_DATA[i][1];
        Real L = LAYER_DATA[i][2];
        Real H_next = LAYER_DATA[next][0];
        Real dH = H_next - H_b;

        if (std::abs(L) < 1e-10) {
            // Isothermal layer: P = P_b * exp(-g0*M0*dH / (R*T))
            P[next] = P[i] * std::exp(-G0 * M0 * dH / (R_STAR * T_b));
        } else {
            // Gradient layer: P = P_b * (T/T_b)^(-g0*M0/(R*L))
            Real T_next = T_b + L * dH;
            Real exponent = -G0 * M0 / (R_STAR * L);
            P[next] = P[i] * std::pow(T_next / T_b, exponent);
        }
    }

    return P;
}

// Precomputed layer pressures (initialized once)
const std::vector<Real> LAYER_PRESSURES = compute_layer_pressures();

/**
 * @brief Compute dynamic viscosity using Sutherland's law
 * @param T Temperature (K)
 * @return Dynamic viscosity (Pa·s)
 */
Real sutherland_viscosity(Real T) {
    // Sutherland's formula: mu = mu_ref * (T/T_ref)^(3/2) * (T_ref + S) / (T + S)
    constexpr Real MU_REF = 1.716e-5;  // Reference viscosity at T_ref (Pa·s)
    constexpr Real T_REF = 273.15;     // Reference temperature (K)
    constexpr Real S = 110.4;          // Sutherland constant for air (K)

    return MU_REF * std::pow(T / T_REF, 1.5) * (T_REF + S) / (T + S);
}

} // anonymous namespace

// ============================================================================
// StandardAtmosphere Implementation
// ============================================================================

StandardAtmosphere::StandardAtmosphere() = default;
StandardAtmosphere::~StandardAtmosphere() = default;

Real StandardAtmosphere::geometric_to_geopotential(Real h) {
    // H = (r * h) / (r + h) where r is effective Earth radius
    return (R_EARTH * h) / (R_EARTH + h);
}

Real StandardAtmosphere::geopotential_to_geometric(Real H) {
    // h = (r * H) / (r - H)
    if (H >= R_EARTH) {
        return 1e9;  // Very large altitude
    }
    return (R_EARTH * H) / (R_EARTH - H);
}

Real StandardAtmosphere::get_temperature(Real altitude) const {
    // Convert geometric to geopotential altitude
    Real H = geometric_to_geopotential(altitude);

    // Clamp to valid range
    H = std::clamp(H, 0.0, H_MAX);

    // Find layer
    int layer = get_layer_index(H);
    Real H_base = LAYER_DATA[layer][0];
    Real T_base = LAYER_DATA[layer][1];
    Real L = LAYER_DATA[layer][2];

    // T = T_base + L * (H - H_base)
    return T_base + L * (H - H_base);
}

Real StandardAtmosphere::get_pressure(Real altitude) const {
    // Convert geometric to geopotential altitude
    Real H = geometric_to_geopotential(altitude);

    // Clamp to valid range
    H = std::clamp(H, 0.0, H_MAX);

    // Find layer
    int layer = get_layer_index(H);
    Real H_base = LAYER_DATA[layer][0];
    Real T_base = LAYER_DATA[layer][1];
    Real L = LAYER_DATA[layer][2];
    Real P_base = LAYER_PRESSURES[static_cast<size_t>(layer)];

    Real dH = H - H_base;

    if (std::abs(L) < 1e-10) {
        // Isothermal layer
        return P_base * std::exp(-G0 * M0 * dH / (R_STAR * T_base));
    } else {
        // Gradient layer
        Real T = T_base + L * dH;
        Real exponent = -G0 * M0 / (R_STAR * L);
        return P_base * std::pow(T / T_base, exponent);
    }
}

Real StandardAtmosphere::get_density(Real altitude) const {
    Real T = get_temperature(altitude);
    Real P = get_pressure(altitude);

    // Ideal gas law: rho = P / (R * T)
    return P / (R_AIR * T);
}

Real StandardAtmosphere::get_speed_of_sound(Real altitude) const {
    Real T = get_temperature(altitude);

    // Speed of sound: a = sqrt(gamma * R * T)
    return std::sqrt(GAMMA * R_AIR * T);
}

AtmosphereState StandardAtmosphere::get_state(Real altitude) const {
    AtmosphereState state;

    state.temperature = get_temperature(altitude);
    state.pressure = get_pressure(altitude);
    state.density = get_density(altitude);
    state.speed_of_sound = get_speed_of_sound(altitude);
    state.viscosity = sutherland_viscosity(state.temperature);
    state.wind = Vec3{0.0, 0.0, 0.0};  // No wind in standard atmosphere
    state.humidity = 0.0;
    state.visibility = 10000.0;  // Standard visibility

    return state;
}

// ============================================================================
// WeatherModel Implementation
// ============================================================================

struct WeatherModel::Impl {
    struct WindLayer {
        Real altitude;
        Real direction;  // From direction (rad from N)
        Real speed;      // m/s
    };

    std::vector<WindLayer> wind_layers;
    Real rain_rate{0.0};  // mm/hour
    Real fog_visibility{10000.0};  // m

    // Wind profile model
    WindProfileConfig wind_profile;
    bool use_wind_profile{false};

    /**
     * @brief Compute wind speed at altitude using power law profile
     *
     * v(z) = v_ref * (z / z_ref)^α
     *
     * Common exponent values:
     * - 0.10: Smooth open sea
     * - 0.14: Open terrain (grassland)
     * - 0.22: Suburban areas
     * - 0.33: Urban areas with tall buildings
     */
    Real compute_power_law_speed(Real altitude) const {
        if (altitude <= 0.0) return 0.0;

        Real z_ref = std::max(wind_profile.ref_altitude, 1.0);
        Real z = std::max(altitude, wind_profile.roughness_length);

        return wind_profile.ref_speed * std::pow(z / z_ref, wind_profile.power_law_exponent);
    }

    /**
     * @brief Compute wind speed at altitude using logarithmic profile
     *
     * v(z) = (u* / κ) * ln(z / z0)
     *
     * where:
     * - u* is friction velocity
     * - κ ≈ 0.41 is von Kármán constant
     * - z0 is surface roughness length
     *
     * Roughness length values:
     * - 0.0002: Calm open sea
     * - 0.005: Sand
     * - 0.03: Short grass
     * - 0.10: Farmland
     * - 0.5: Suburbs
     * - 1.0: Urban centers
     */
    Real compute_log_profile_speed(Real altitude) const {
        constexpr Real KAPPA = 0.41;  // von Kármán constant

        Real z0 = std::max(wind_profile.roughness_length, 0.001);
        Real z = std::max(altitude, z0 + 0.001);

        return (wind_profile.friction_velocity / KAPPA) * std::log(z / z0);
    }

    /**
     * @brief Compute wind direction at altitude with shear
     */
    Real compute_direction(Real altitude) const {
        Real base_dir = wind_profile.ref_direction;
        Real alt_km = altitude / 1000.0;

        // Add directional shear (Ekman spiral effect)
        return base_dir + wind_profile.direction_shear * alt_km;
    }

    /**
     * @brief Compute gust component using simple turbulence model
     *
     * Uses a sum of sinusoids to approximate turbulence.
     * More sophisticated models (Dryden, von Karman) could be added.
     */
    Vec3 compute_gust(Real altitude, Real time_s, Real mean_speed) const {
        if (!wind_profile.enable_gusts || mean_speed < 0.01) {
            return Vec3{0.0, 0.0, 0.0};
        }

        Real intensity = wind_profile.gust_intensity * mean_speed;
        Real freq = wind_profile.gust_frequency;

        // Altitude scaling (gusts decrease with altitude)
        Real alt_factor = std::exp(-altitude / 1000.0);
        intensity *= alt_factor;

        // Simple multi-frequency turbulence approximation
        Real gust_n = intensity * (
            0.5 * std::sin(2.0 * M_PI * freq * time_s) +
            0.3 * std::sin(2.0 * M_PI * freq * 2.3 * time_s + 1.2) +
            0.2 * std::sin(2.0 * M_PI * freq * 0.7 * time_s + 2.5)
        );

        Real gust_e = intensity * (
            0.4 * std::sin(2.0 * M_PI * freq * 1.1 * time_s + 0.5) +
            0.35 * std::sin(2.0 * M_PI * freq * 2.7 * time_s + 1.8) +
            0.25 * std::sin(2.0 * M_PI * freq * 0.5 * time_s + 3.1)
        );

        Real gust_d = 0.3 * intensity * (
            std::sin(2.0 * M_PI * freq * 0.8 * time_s + 0.3) +
            0.5 * std::sin(2.0 * M_PI * freq * 1.5 * time_s + 2.0)
        );

        return Vec3{gust_n, gust_e, gust_d};
    }

    /**
     * @brief Compute wind using profile model
     */
    Vec3 compute_wind_profile(Real altitude, Real time_s) const {
        Real speed = 0.0;

        switch (wind_profile.type) {
            case WindProfileType::Constant:
                speed = wind_profile.ref_speed;
                break;

            case WindProfileType::PowerLaw:
                speed = compute_power_law_speed(altitude);
                break;

            case WindProfileType::Logarithmic:
                speed = compute_log_profile_speed(altitude);
                break;

            case WindProfileType::LinearShear:
                // Linear interpolation from surface to reference
                speed = wind_profile.ref_speed * (altitude / wind_profile.ref_altitude);
                speed = std::clamp(speed, 0.0, wind_profile.ref_speed * 2.0);
                break;
        }

        // Compute direction with shear
        Real direction = compute_direction(altitude);

        // Convert to NED velocity components
        // Wind direction is "from", so we negate
        Vec3 wind{
            -speed * std::cos(direction),  // North component
            -speed * std::sin(direction),  // East component
            0.0                             // Down component
        };

        // Add gusts
        Vec3 gust = compute_gust(altitude, time_s, speed);
        wind = wind + gust;

        return wind;
    }

    Vec3 interpolate_wind(Real altitude) const {
        if (wind_layers.empty()) {
            return Vec3{0.0, 0.0, 0.0};
        }

        // Find bounding layers
        size_t lower = 0;
        size_t upper = 0;

        for (size_t i = 0; i < wind_layers.size(); ++i) {
            if (wind_layers[i].altitude <= altitude) {
                lower = i;
            }
            if (wind_layers[i].altitude >= altitude) {
                upper = i;
                break;
            }
            upper = i;
        }

        if (lower == upper || wind_layers[lower].altitude == wind_layers[upper].altitude) {
            // Use single layer
            Real dir = wind_layers[lower].direction;
            Real spd = wind_layers[lower].speed;
            // Wind direction is "from", so we negate to get wind velocity direction
            // NED frame: N is +X, E is +Y
            return Vec3{
                -spd * std::cos(dir),  // North component
                -spd * std::sin(dir),  // East component
                0.0                     // Down component (typically 0)
            };
        }

        // Linear interpolation between layers
        Real t = (altitude - wind_layers[lower].altitude) /
                 (wind_layers[upper].altitude - wind_layers[lower].altitude);

        Real dir = wind_layers[lower].direction + t * (wind_layers[upper].direction - wind_layers[lower].direction);
        Real spd = wind_layers[lower].speed + t * (wind_layers[upper].speed - wind_layers[lower].speed);

        return Vec3{
            -spd * std::cos(dir),
            -spd * std::sin(dir),
            0.0
        };
    }
};

WeatherModel::WeatherModel() : impl_(std::make_unique<Impl>()) {}
WeatherModel::~WeatherModel() = default;

void WeatherModel::set_wind_layer(Real altitude, Real direction, Real speed) {
    // Insert in sorted order by altitude
    auto& layers = impl_->wind_layers;

    Impl::WindLayer new_layer{altitude, direction, speed};

    auto it = std::lower_bound(layers.begin(), layers.end(), new_layer,
        [](const Impl::WindLayer& a, const Impl::WindLayer& b) {
            return a.altitude < b.altitude;
        });

    // Check if layer at this altitude already exists
    if (it != layers.end() && std::abs(it->altitude - altitude) < 0.1) {
        *it = new_layer;  // Update existing
    } else {
        layers.insert(it, new_layer);  // Insert new
    }
}

Vec3 WeatherModel::get_wind(Real /*lat*/, Real /*lon*/, Real alt) const {
    // Currently ignores lat/lon, only uses altitude
    return impl_->interpolate_wind(alt);
}

void WeatherModel::set_rain_rate(Real rate) {
    impl_->rain_rate = std::max(0.0, rate);
}

Real WeatherModel::get_rain_attenuation(Real frequency_ghz) const {
    // ITU-R P.838-3 rain attenuation model (simplified)
    if (impl_->rain_rate <= 0.0) {
        return 0.0;
    }

    // Coefficients for horizontal polarization (approximate)
    Real k, alpha;
    if (frequency_ghz <= 1.0) {
        k = 0.0000387;
        alpha = 0.912;
    } else if (frequency_ghz <= 10.0) {
        k = 0.0101 * std::pow(frequency_ghz, 1.276);
        alpha = 1.0 + 0.03 * frequency_ghz;
    } else {
        k = 0.0367 * std::pow(frequency_ghz, 0.5);
        alpha = 1.13;
    }

    // Specific attenuation (dB/km) = k * R^alpha
    return k * std::pow(impl_->rain_rate, alpha);
}

void WeatherModel::set_fog_visibility(Real visibility_m) {
    impl_->fog_visibility = std::max(1.0, visibility_m);
}

Real WeatherModel::get_visibility(Real /*lat*/, Real /*lon*/, Real /*alt*/) const {
    // Simple model: return fog visibility (ignores position for now)
    return impl_->fog_visibility;
}

void WeatherModel::set_wind_profile(const WindProfileConfig& config) {
    impl_->wind_profile = config;
}

const WindProfileConfig& WeatherModel::get_wind_profile() const {
    return impl_->wind_profile;
}

void WeatherModel::set_use_wind_profile(bool use_profile) {
    impl_->use_wind_profile = use_profile;
}

Vec3 WeatherModel::get_wind_profile(Real altitude, Real time_s) const {
    return impl_->compute_wind_profile(altitude, time_s);
}

// ============================================================================
// AtmosphereManager Implementation
// ============================================================================

AtmosphereManager::AtmosphereManager() = default;
AtmosphereManager::~AtmosphereManager() = default;

AtmosphereState AtmosphereManager::get_state(Real lat, Real lon, Real alt) const {
    // Start with standard atmosphere
    AtmosphereState state = standard_.get_state(alt);

    // Add weather effects if enabled
    if (weather_enabled_) {
        state.wind = weather_.get_wind(lat, lon, alt);
        state.visibility = weather_.get_visibility(lat, lon, alt);
    }

    return state;
}

} // namespace jaguar::environment
