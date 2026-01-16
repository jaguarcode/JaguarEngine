/**
 * @file test_atmosphere.cpp
 * @brief Unit tests for US Standard Atmosphere 1976 model
 */

#include <gtest/gtest.h>
#include "jaguar/environment/atmosphere.h"
#include <cmath>

using namespace jaguar;
using namespace jaguar::environment;

// ============================================================================
// US Standard Atmosphere 1976 Reference Values
// ============================================================================
// These values come from the US Standard Atmosphere 1976 tables

namespace {

// Standard atmosphere reference data at specific geopotential altitudes
// Format: {altitude_m, temperature_K, pressure_Pa, density_kg_m3}
constexpr double REFERENCE_DATA[][4] = {
    {     0.0, 288.15,  101325.0,     1.225     },  // Sea level
    {  1000.0, 281.65,   89874.6,     1.1117    },  // 1 km
    {  5000.0, 255.65,   54019.9,     0.73612   },  // 5 km
    { 10000.0, 223.15,   26436.3,     0.41271   },  // 10 km
    { 11000.0, 216.65,   22632.1,     0.36392   },  // Tropopause
    { 15000.0, 216.65,   12044.6,     0.19367   },  // In tropopause
    { 20000.0, 216.65,    5474.89,    0.088035  },  // End of tropopause
    { 25000.0, 221.65,    2511.02,    0.039466  },  // Stratosphere
    { 32000.0, 228.65,     868.019,   0.013225  },  // Upper stratosphere
    { 47000.0, 270.65,     110.906,   0.0014275 },  // Stratopause
    { 50000.0, 270.65,      75.9448,  0.00097752},  // In stratopause
};

constexpr int NUM_REFERENCE_POINTS = sizeof(REFERENCE_DATA) / sizeof(REFERENCE_DATA[0]);

} // anonymous namespace

// ============================================================================
// Standard Atmosphere Tests
// ============================================================================

class StandardAtmosphereTest : public ::testing::Test {
protected:
    StandardAtmosphere atmosphere;

    // Tolerance levels
    // Note: The model converts geometric to geopotential altitude which introduces
    // differences at high altitudes compared to tables listed at geometric altitude.
    // At 50km, geopotential is ~49.6km, causing ~5% difference in pressure.
    static constexpr Real TEMP_TOLERANCE = 2.0;      // ±2 K (allows for geo conversion)
    static constexpr Real PRESSURE_REL_TOL = 0.06;   // ±6% (accounts for geo conversion at high alt)
    static constexpr Real DENSITY_REL_TOL = 0.06;    // ±6%
    static constexpr Real SOUND_SPEED_TOL = 1.0;     // ±1 m/s
};

TEST_F(StandardAtmosphereTest, SeaLevelConditions) {
    AtmosphereState state = atmosphere.get_state(0.0);

    EXPECT_NEAR(state.temperature, 288.15, TEMP_TOLERANCE);
    EXPECT_NEAR(state.pressure, 101325.0, 100.0);  // ±100 Pa
    EXPECT_NEAR(state.density, 1.225, 0.01);
    EXPECT_NEAR(state.speed_of_sound, 340.29, SOUND_SPEED_TOL);
}

TEST_F(StandardAtmosphereTest, TemperatureAtKnownAltitudes) {
    for (int i = 0; i < NUM_REFERENCE_POINTS; ++i) {
        Real altitude = REFERENCE_DATA[i][0];
        Real expected_temp = REFERENCE_DATA[i][1];

        Real actual_temp = atmosphere.get_temperature(altitude);

        EXPECT_NEAR(actual_temp, expected_temp, TEMP_TOLERANCE)
            << "Temperature mismatch at altitude " << altitude << " m";
    }
}

TEST_F(StandardAtmosphereTest, PressureAtKnownAltitudes) {
    for (int i = 0; i < NUM_REFERENCE_POINTS; ++i) {
        Real altitude = REFERENCE_DATA[i][0];
        Real expected_pressure = REFERENCE_DATA[i][2];

        Real actual_pressure = atmosphere.get_pressure(altitude);

        // Use relative tolerance for pressure (varies over orders of magnitude)
        Real rel_error = std::abs(actual_pressure - expected_pressure) / expected_pressure;
        EXPECT_LT(rel_error, PRESSURE_REL_TOL)
            << "Pressure mismatch at altitude " << altitude << " m"
            << " (expected: " << expected_pressure << ", actual: " << actual_pressure << ")";
    }
}

TEST_F(StandardAtmosphereTest, DensityAtKnownAltitudes) {
    for (int i = 0; i < NUM_REFERENCE_POINTS; ++i) {
        Real altitude = REFERENCE_DATA[i][0];
        Real expected_density = REFERENCE_DATA[i][3];

        Real actual_density = atmosphere.get_density(altitude);

        // Use relative tolerance for density
        Real rel_error = std::abs(actual_density - expected_density) / expected_density;
        EXPECT_LT(rel_error, DENSITY_REL_TOL)
            << "Density mismatch at altitude " << altitude << " m"
            << " (expected: " << expected_density << ", actual: " << actual_density << ")";
    }
}

TEST_F(StandardAtmosphereTest, SpeedOfSoundDecreases) {
    // Speed of sound depends only on temperature: a = sqrt(gamma * R * T)
    // So it should decrease in the troposphere where T decreases
    Real a_0 = atmosphere.get_speed_of_sound(0.0);
    Real a_10k = atmosphere.get_speed_of_sound(10000.0);

    EXPECT_GT(a_0, a_10k);  // Speed of sound at sea level > at 10 km
    EXPECT_NEAR(a_0, 340.29, SOUND_SPEED_TOL);
    EXPECT_NEAR(a_10k, 299.5, SOUND_SPEED_TOL);  // ~299.5 m/s at 10 km
}

TEST_F(StandardAtmosphereTest, TropopauseIsothermal) {
    // Between 11 km and 20 km, temperature should be constant at 216.65 K
    Real t_11k = atmosphere.get_temperature(11000.0);
    Real t_15k = atmosphere.get_temperature(15000.0);
    Real t_20k = atmosphere.get_temperature(20000.0);

    EXPECT_NEAR(t_11k, 216.65, TEMP_TOLERANCE);
    EXPECT_NEAR(t_15k, 216.65, TEMP_TOLERANCE);
    EXPECT_NEAR(t_20k, 216.65, TEMP_TOLERANCE);
}

TEST_F(StandardAtmosphereTest, StratosphereWarming) {
    // Temperature increases in the stratosphere (20-47 km)
    Real t_20k = atmosphere.get_temperature(20000.0);
    Real t_32k = atmosphere.get_temperature(32000.0);
    Real t_47k = atmosphere.get_temperature(47000.0);

    EXPECT_LT(t_20k, t_32k);  // Warming in stratosphere 1
    EXPECT_LT(t_32k, t_47k);  // Warming in stratosphere 2
}

TEST_F(StandardAtmosphereTest, GeopotentialConversion) {
    // Test conversion between geometric and geopotential altitude
    Real h = 10000.0;  // 10 km geometric
    Real H = StandardAtmosphere::geometric_to_geopotential(h);
    Real h_back = StandardAtmosphere::geopotential_to_geometric(H);

    // Geopotential should be slightly less than geometric
    EXPECT_LT(H, h);

    // Round-trip should be accurate
    EXPECT_NEAR(h_back, h, 0.01);
}

TEST_F(StandardAtmosphereTest, NegativeAltitude) {
    // Below sea level: model clamps geopotential altitude to 0
    // so values should be approximately at sea level
    AtmosphereState state = atmosphere.get_state(-100.0);

    // Temperature should be approximately sea level (geopotential clamped to 0)
    EXPECT_NEAR(state.temperature, 288.15, 1.0);

    // Pressure should be approximately sea level
    EXPECT_NEAR(state.pressure, 101325.0, 200.0);
}

TEST_F(StandardAtmosphereTest, VeryHighAltitude) {
    // Above 86 km, should clamp or extrapolate
    AtmosphereState state = atmosphere.get_state(100000.0);  // 100 km

    // Values should be reasonable (not NaN or infinite)
    EXPECT_TRUE(std::isfinite(state.temperature));
    EXPECT_TRUE(std::isfinite(state.pressure));
    EXPECT_TRUE(std::isfinite(state.density));
    EXPECT_GT(state.temperature, 0.0);
    EXPECT_GT(state.pressure, 0.0);
    EXPECT_GT(state.density, 0.0);
}

TEST_F(StandardAtmosphereTest, IdealGasLaw) {
    // Verify ideal gas law: rho = P / (R * T)
    constexpr Real R_AIR = 287.05287;  // J/(kg·K)

    for (Real alt = 0.0; alt <= 50000.0; alt += 5000.0) {
        Real T = atmosphere.get_temperature(alt);
        Real P = atmosphere.get_pressure(alt);
        Real rho = atmosphere.get_density(alt);

        Real rho_calc = P / (R_AIR * T);

        EXPECT_NEAR(rho, rho_calc, rho * 0.001)  // 0.1% tolerance
            << "Ideal gas law not satisfied at altitude " << alt << " m";
    }
}

TEST_F(StandardAtmosphereTest, ViscosityReasonable) {
    AtmosphereState state_0 = atmosphere.get_state(0.0);
    AtmosphereState state_10k = atmosphere.get_state(10000.0);

    // Sea level viscosity should be around 1.789e-5 Pa·s
    EXPECT_NEAR(state_0.viscosity, 1.789e-5, 0.05e-5);

    // Viscosity decreases with temperature (Sutherland's law)
    EXPECT_LT(state_10k.viscosity, state_0.viscosity);
}

// ============================================================================
// Weather Model Tests
// ============================================================================

class WeatherModelTest : public ::testing::Test {
protected:
    WeatherModel weather;
    static constexpr Real EPSILON = 1e-6;
};

TEST_F(WeatherModelTest, NoWindByDefault) {
    Vec3 wind = weather.get_wind(0.0, 0.0, 0.0);

    EXPECT_NEAR(wind.x, 0.0, EPSILON);
    EXPECT_NEAR(wind.y, 0.0, EPSILON);
    EXPECT_NEAR(wind.z, 0.0, EPSILON);
}

TEST_F(WeatherModelTest, SetWindLayer) {
    // Set a northerly wind at 1000m (wind from north = 0 radians)
    weather.set_wind_layer(1000.0, 0.0, 10.0);  // 10 m/s from north

    Vec3 wind = weather.get_wind(0.0, 0.0, 1000.0);

    // Wind from north means wind velocity is toward south (negative north component)
    EXPECT_NEAR(wind.x, -10.0, 0.1);  // North component
    EXPECT_NEAR(wind.y, 0.0, 0.1);    // East component
    EXPECT_NEAR(wind.z, 0.0, EPSILON);
}

TEST_F(WeatherModelTest, EasterlyWind) {
    // Set an easterly wind (wind from east = PI/2 radians)
    weather.set_wind_layer(500.0, constants::PI / 2.0, 15.0);  // 15 m/s from east

    Vec3 wind = weather.get_wind(0.0, 0.0, 500.0);

    // Wind from east means wind velocity is toward west (negative east component)
    EXPECT_NEAR(wind.x, 0.0, 0.1);
    EXPECT_NEAR(wind.y, -15.0, 0.1);
    EXPECT_NEAR(wind.z, 0.0, EPSILON);
}

TEST_F(WeatherModelTest, WindInterpolation) {
    // Set wind layers at different altitudes
    weather.set_wind_layer(0.0, 0.0, 10.0);      // 10 m/s at surface
    weather.set_wind_layer(2000.0, 0.0, 20.0);   // 20 m/s at 2000m

    // Wind at 1000m should be interpolated
    Vec3 wind_1k = weather.get_wind(0.0, 0.0, 1000.0);

    // Should be approximately 15 m/s (midpoint)
    Real speed = std::sqrt(wind_1k.x * wind_1k.x + wind_1k.y * wind_1k.y);
    EXPECT_NEAR(speed, 15.0, 0.5);
}

TEST_F(WeatherModelTest, RainAttenuation) {
    weather.set_rain_rate(10.0);  // 10 mm/hour (moderate rain)

    // Rain attenuation generally increases with frequency (at lower frequencies)
    Real atten_1ghz = weather.get_rain_attenuation(1.0);
    Real atten_10ghz = weather.get_rain_attenuation(10.0);

    // At low frequencies, attenuation increases with frequency
    EXPECT_GT(atten_10ghz, atten_1ghz);

    // All attenuations should be positive
    EXPECT_GT(atten_1ghz, 0.0);
    EXPECT_GT(atten_10ghz, 0.0);
}

TEST_F(WeatherModelTest, NoRainNoAttenuation) {
    weather.set_rain_rate(0.0);

    Real atten = weather.get_rain_attenuation(10.0);
    EXPECT_NEAR(atten, 0.0, EPSILON);
}

TEST_F(WeatherModelTest, FogVisibility) {
    weather.set_fog_visibility(500.0);  // 500m visibility (thick fog)

    Real vis = weather.get_visibility(0.0, 0.0, 0.0);
    EXPECT_NEAR(vis, 500.0, EPSILON);
}

// ============================================================================
// Atmosphere Manager Tests
// ============================================================================

class AtmosphereManagerTest : public ::testing::Test {
protected:
    AtmosphereManager manager;
};

TEST_F(AtmosphereManagerTest, CombinesStandardAndWeather) {
    // Set some weather
    manager.weather().set_wind_layer(0.0, 0.0, 10.0);
    manager.weather().set_fog_visibility(1000.0);

    AtmosphereState state = manager.get_state(0.0, 0.0, 0.0);

    // Should have standard atmosphere values
    EXPECT_NEAR(state.temperature, 288.15, 0.5);
    EXPECT_NEAR(state.pressure, 101325.0, 100.0);

    // Should have weather effects
    EXPECT_NEAR(state.wind.x, -10.0, 0.1);
    EXPECT_NEAR(state.visibility, 1000.0, 1.0);
}

TEST_F(AtmosphereManagerTest, DisableWeather) {
    manager.weather().set_wind_layer(0.0, 0.0, 10.0);
    manager.set_weather_enabled(false);

    AtmosphereState state = manager.get_state(0.0, 0.0, 0.0);

    // Wind should be zero when weather is disabled
    EXPECT_NEAR(state.wind.x, 0.0, 0.001);
    EXPECT_NEAR(state.wind.y, 0.0, 0.001);
}

TEST_F(AtmosphereManagerTest, AccessStandardAtmosphere) {
    const StandardAtmosphere& std_atm = manager.standard();

    Real temp = std_atm.get_temperature(10000.0);
    EXPECT_NEAR(temp, 223.15, 0.5);
}

// ============================================================================
// Wind Profile Model Tests
// ============================================================================

class WindProfileTest : public ::testing::Test {
protected:
    WeatherModel weather;
    static constexpr Real EPSILON = 1e-6;
};

TEST_F(WindProfileTest, PowerLawProfile) {
    WindProfileConfig config;
    config.type = WindProfileType::PowerLaw;
    config.ref_altitude = 10.0;
    config.ref_speed = 10.0;
    config.ref_direction = 0.0;  // From north
    config.power_law_exponent = 0.143;  // Open terrain
    config.enable_gusts = false;

    weather.set_wind_profile(config);

    // At reference altitude, wind should equal reference speed
    Vec3 wind_ref = weather.get_wind_profile(10.0);
    Real speed_ref = std::sqrt(wind_ref.x * wind_ref.x + wind_ref.y * wind_ref.y);
    EXPECT_NEAR(speed_ref, 10.0, 0.1);

    // At higher altitude, wind should be faster
    Vec3 wind_100 = weather.get_wind_profile(100.0);
    Real speed_100 = std::sqrt(wind_100.x * wind_100.x + wind_100.y * wind_100.y);
    EXPECT_GT(speed_100, speed_ref);

    // Power law: v(100) = v(10) * (100/10)^0.143 = 10 * 10^0.143 ≈ 13.9
    EXPECT_NEAR(speed_100, 13.9, 0.5);
}

TEST_F(WindProfileTest, LogarithmicProfile) {
    WindProfileConfig config;
    config.type = WindProfileType::Logarithmic;
    config.friction_velocity = 0.5;
    config.roughness_length = 0.03;  // Short grass
    config.ref_direction = 0.0;
    config.enable_gusts = false;

    weather.set_wind_profile(config);

    // At 10m, calculate expected speed
    // v = (u*/κ) * ln(z/z0) = (0.5/0.41) * ln(10/0.03) ≈ 7.1 m/s
    Vec3 wind_10 = weather.get_wind_profile(10.0);
    Real speed_10 = std::sqrt(wind_10.x * wind_10.x + wind_10.y * wind_10.y);
    EXPECT_NEAR(speed_10, 7.1, 0.5);

    // At higher altitude, wind should increase logarithmically
    Vec3 wind_100 = weather.get_wind_profile(100.0);
    Real speed_100 = std::sqrt(wind_100.x * wind_100.x + wind_100.y * wind_100.y);
    EXPECT_GT(speed_100, speed_10);
}

TEST_F(WindProfileTest, ConstantProfile) {
    WindProfileConfig config;
    config.type = WindProfileType::Constant;
    config.ref_speed = 15.0;
    config.ref_direction = constants::PI / 4.0;  // From NE
    config.enable_gusts = false;

    weather.set_wind_profile(config);

    // At any altitude, wind should be the same
    Vec3 wind_10 = weather.get_wind_profile(10.0);
    Vec3 wind_100 = weather.get_wind_profile(100.0);
    Vec3 wind_1000 = weather.get_wind_profile(1000.0);

    Real speed_10 = std::sqrt(wind_10.x * wind_10.x + wind_10.y * wind_10.y);
    Real speed_100 = std::sqrt(wind_100.x * wind_100.x + wind_100.y * wind_100.y);
    Real speed_1000 = std::sqrt(wind_1000.x * wind_1000.x + wind_1000.y * wind_1000.y);

    EXPECT_NEAR(speed_10, 15.0, 0.1);
    EXPECT_NEAR(speed_100, 15.0, 0.1);
    EXPECT_NEAR(speed_1000, 15.0, 0.1);
}

TEST_F(WindProfileTest, DirectionShear) {
    WindProfileConfig config;
    config.type = WindProfileType::Constant;
    config.ref_speed = 10.0;
    config.ref_direction = 0.0;  // From north at surface
    config.direction_shear = constants::PI / 18.0;  // 10 degrees per km
    config.enable_gusts = false;

    weather.set_wind_profile(config);

    // At surface, wind from north
    Vec3 wind_0 = weather.get_wind_profile(0.0);
    EXPECT_NEAR(wind_0.x, -10.0, 0.1);  // Wind toward south
    EXPECT_NEAR(wind_0.y, 0.0, 0.1);

    // At 1km, direction should have shifted by 10 degrees
    // Wind from 10 degrees east of north
    Vec3 wind_1k = weather.get_wind_profile(1000.0);
    Real dir_1k = std::atan2(-wind_1k.y, -wind_1k.x);
    EXPECT_NEAR(dir_1k, constants::PI / 18.0, 0.02);
}

TEST_F(WindProfileTest, GustsAddVariation) {
    WindProfileConfig config;
    config.type = WindProfileType::Constant;
    config.ref_speed = 10.0;
    config.ref_direction = 0.0;
    config.enable_gusts = true;
    config.gust_intensity = 0.2;  // 20% of mean wind
    config.gust_frequency = 1.0;  // 1 Hz

    weather.set_wind_profile(config);

    // Sample wind at different times
    std::vector<Real> speeds;
    for (Real t = 0.0; t < 2.0; t += 0.1) {
        Vec3 wind = weather.get_wind_profile(10.0, t);
        Real speed = std::sqrt(wind.x * wind.x + wind.y * wind.y + wind.z * wind.z);
        speeds.push_back(speed);
    }

    // Check that there's variation in wind speed
    Real min_speed = *std::min_element(speeds.begin(), speeds.end());
    Real max_speed = *std::max_element(speeds.begin(), speeds.end());

    EXPECT_GT(max_speed - min_speed, 0.5);  // At least 0.5 m/s variation
    EXPECT_LT(max_speed, 15.0);  // Not more than mean + 50%
    EXPECT_GT(min_speed, 5.0);   // Not less than mean - 50%
}

TEST_F(WindProfileTest, GustsDecreaseWithAltitude) {
    WindProfileConfig config;
    config.type = WindProfileType::Constant;
    config.ref_speed = 10.0;
    config.ref_direction = 0.0;
    config.enable_gusts = true;
    config.gust_intensity = 0.3;
    config.gust_frequency = 0.5;

    weather.set_wind_profile(config);

    // Measure gust variation at different altitudes
    auto measure_variation = [&](Real altitude) {
        std::vector<Real> speeds;
        for (Real t = 0.0; t < 5.0; t += 0.2) {
            Vec3 wind = weather.get_wind_profile(altitude, t);
            Real speed = std::sqrt(wind.x * wind.x + wind.y * wind.y + wind.z * wind.z);
            speeds.push_back(speed);
        }
        return *std::max_element(speeds.begin(), speeds.end()) -
               *std::min_element(speeds.begin(), speeds.end());
    };

    Real var_surface = measure_variation(10.0);
    Real var_high = measure_variation(2000.0);

    // Gusts should decrease with altitude
    EXPECT_GT(var_surface, var_high);
}

TEST_F(WindProfileTest, LinearShearProfile) {
    WindProfileConfig config;
    config.type = WindProfileType::LinearShear;
    config.ref_altitude = 100.0;
    config.ref_speed = 20.0;
    config.ref_direction = 0.0;
    config.enable_gusts = false;

    weather.set_wind_profile(config);

    // At reference altitude, should equal reference speed
    Vec3 wind_ref = weather.get_wind_profile(100.0);
    Real speed_ref = std::sqrt(wind_ref.x * wind_ref.x + wind_ref.y * wind_ref.y);
    EXPECT_NEAR(speed_ref, 20.0, 0.1);

    // At half the reference altitude, should be half the speed
    Vec3 wind_50 = weather.get_wind_profile(50.0);
    Real speed_50 = std::sqrt(wind_50.x * wind_50.x + wind_50.y * wind_50.y);
    EXPECT_NEAR(speed_50, 10.0, 0.1);

    // At surface, should be near zero
    Vec3 wind_0 = weather.get_wind_profile(0.0);
    Real speed_0 = std::sqrt(wind_0.x * wind_0.x + wind_0.y * wind_0.y);
    EXPECT_NEAR(speed_0, 0.0, 0.5);
}

TEST_F(WindProfileTest, ZeroAltitudeHandling) {
    WindProfileConfig config;
    config.type = WindProfileType::PowerLaw;
    config.ref_altitude = 10.0;
    config.ref_speed = 10.0;
    config.power_law_exponent = 0.143;
    config.enable_gusts = false;

    weather.set_wind_profile(config);

    // At zero altitude, should handle gracefully (use roughness length minimum)
    Vec3 wind_0 = weather.get_wind_profile(0.0);
    Real speed_0 = std::sqrt(wind_0.x * wind_0.x + wind_0.y * wind_0.y);

    EXPECT_TRUE(std::isfinite(speed_0));
    EXPECT_GE(speed_0, 0.0);
}

TEST_F(WindProfileTest, GetAndSetProfile) {
    WindProfileConfig config;
    config.type = WindProfileType::Logarithmic;
    config.friction_velocity = 0.8;
    config.roughness_length = 0.1;

    weather.set_wind_profile(config);

    const WindProfileConfig& retrieved = weather.get_wind_profile();
    EXPECT_EQ(retrieved.type, WindProfileType::Logarithmic);
    EXPECT_NEAR(retrieved.friction_velocity, 0.8, EPSILON);
    EXPECT_NEAR(retrieved.roughness_length, 0.1, EPSILON);
}
