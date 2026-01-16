#pragma once
/**
 * @file atmosphere.h
 * @brief Atmospheric models and weather effects
 */

#include "jaguar/core/types.h"
#include <memory>

namespace jaguar::environment {

/**
 * @brief Atmospheric state at a point
 */
struct AtmosphereState {
    Real temperature{288.15};     ///< K
    Real pressure{101325.0};      ///< Pa
    Real density{1.225};          ///< kg/m³
    Real speed_of_sound{340.29};  ///< m/s
    Real viscosity{1.789e-5};     ///< Pa·s
    Vec3 wind{0, 0, 0};           ///< Wind velocity in NED (m/s)
    Real humidity{0.0};           ///< Relative humidity (0-1)
    Real visibility{10000.0};     ///< Visibility (m)
};

/**
 * @brief US Standard Atmosphere 1976 model
 */
class StandardAtmosphere {
public:
    StandardAtmosphere();
    ~StandardAtmosphere();

    /**
     * @brief Get atmosphere at geometric altitude
     * @param altitude Geometric altitude (m)
     */
    AtmosphereState get_state(Real altitude) const;

    /**
     * @brief Get temperature at altitude
     */
    Real get_temperature(Real altitude) const;

    /**
     * @brief Get pressure at altitude
     */
    Real get_pressure(Real altitude) const;

    /**
     * @brief Get density at altitude
     */
    Real get_density(Real altitude) const;

    /**
     * @brief Get speed of sound at altitude
     */
    Real get_speed_of_sound(Real altitude) const;

    /**
     * @brief Convert geometric to geopotential altitude
     */
    static Real geometric_to_geopotential(Real h);

    /**
     * @brief Convert geopotential to geometric altitude
     */
    static Real geopotential_to_geometric(Real H);
};

/**
 * @brief Wind profile model type
 */
enum class WindProfileType {
    Constant,       ///< Constant wind (no altitude variation)
    PowerLaw,       ///< Power law profile for boundary layer
    Logarithmic,    ///< Logarithmic profile for surface layer
    LinearShear     ///< Linear wind shear
};

/**
 * @brief Wind profile configuration
 */
struct WindProfileConfig {
    WindProfileType type{WindProfileType::PowerLaw};

    // Reference values (at reference altitude)
    Real ref_altitude{10.0};        ///< Reference altitude (m)
    Real ref_speed{10.0};           ///< Reference wind speed (m/s)
    Real ref_direction{0.0};        ///< Reference wind direction (rad from N)

    // Power law parameters
    Real power_law_exponent{0.143}; ///< α in v = v_ref * (z/z_ref)^α

    // Logarithmic profile parameters
    Real roughness_length{0.03};    ///< Surface roughness z0 (m)
    Real friction_velocity{0.5};    ///< u* friction velocity (m/s)

    // Wind shear parameters
    Real direction_shear{0.0};      ///< Direction change per km altitude (rad/km)

    // Gust parameters
    bool enable_gusts{false};       ///< Enable turbulence/gusts
    Real gust_intensity{0.1};       ///< Gust intensity (fraction of mean wind)
    Real gust_frequency{0.1};       ///< Characteristic gust frequency (Hz)
};

/**
 * @brief Weather effects model
 */
class WeatherModel {
public:
    WeatherModel();
    ~WeatherModel();

    /**
     * @brief Set wind at altitude
     * @param altitude Base altitude (m)
     * @param direction Wind from direction (rad from N)
     * @param speed Wind speed (m/s)
     */
    void set_wind_layer(Real altitude, Real direction, Real speed);

    /**
     * @brief Get wind at position
     */
    Vec3 get_wind(Real lat, Real lon, Real alt) const;

    /**
     * @brief Set rain rate
     * @param rate mm/hour
     */
    void set_rain_rate(Real rate);

    /**
     * @brief Get rain attenuation for RF signals
     * @param frequency_ghz Frequency in GHz
     * @return Attenuation in dB/km
     */
    Real get_rain_attenuation(Real frequency_ghz) const;

    /**
     * @brief Set fog visibility
     */
    void set_fog_visibility(Real visibility_m);

    /**
     * @brief Get visibility at position
     */
    Real get_visibility(Real lat, Real lon, Real alt) const;

    // ========================================================================
    // Wind Profile Model
    // ========================================================================

    /**
     * @brief Set wind profile configuration
     */
    void set_wind_profile(const WindProfileConfig& config);

    /**
     * @brief Get wind profile configuration
     */
    const WindProfileConfig& get_wind_profile() const;

    /**
     * @brief Enable/disable wind profile model (vs discrete layers)
     */
    void set_use_wind_profile(bool use_profile);

    /**
     * @brief Get wind at altitude using wind profile model
     *
     * Uses the configured wind profile (power law, logarithmic, etc.)
     * instead of discrete layer interpolation.
     *
     * @param altitude Height above ground (m)
     * @param time_s Simulation time for gust calculation (s)
     * @return Wind velocity in NED frame (m/s)
     */
    Vec3 get_wind_profile(Real altitude, Real time_s = 0.0) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/**
 * @brief Complete atmosphere manager
 */
class AtmosphereManager {
public:
    AtmosphereManager();
    ~AtmosphereManager();

    /**
     * @brief Get atmosphere state at position
     */
    AtmosphereState get_state(Real lat, Real lon, Real alt) const;

    /**
     * @brief Get standard atmosphere (no weather)
     */
    const StandardAtmosphere& standard() const { return standard_; }

    /**
     * @brief Get weather model
     */
    WeatherModel& weather() { return weather_; }
    const WeatherModel& weather() const { return weather_; }

    /**
     * @brief Enable/disable weather effects
     */
    void set_weather_enabled(bool enabled) { weather_enabled_ = enabled; }

private:
    StandardAtmosphere standard_;
    WeatherModel weather_;
    bool weather_enabled_{true};
};

} // namespace jaguar::environment
