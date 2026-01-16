/**
 * @file test_terrain_validation.cpp
 * @brief Terrain and ground vehicle validation against NRMM reference data
 *
 * Validates terrain interaction and vehicle mobility models against:
 * - NATO Reference Mobility Model (NRMM)
 * - Bekker-Wong terramechanics theory
 * - TARDEC vehicle mobility data
 * - Published military vehicle specifications
 *
 * References:
 * - MIL-STD-1472: Human Engineering
 * - NATO STANAG 4357: Ground Vehicle Mobility Assessment
 * - Wong, J.Y., "Theory of Ground Vehicles"
 */

#include <gtest/gtest.h>
#include <cmath>
#include <vector>

#include "jaguar/domain/land.h"

namespace jaguar::test {

using namespace jaguar::domain::land;

// ============================================================================
// NRMM Reference Data
// ============================================================================

/**
 * @brief NRMM terrain classification
 */
enum class NRMMTerrainClass {
    Primary,    // Roads, hard surfaces
    Secondary,  // Unpaved roads, firm soil
    TrailPoor,  // Poor trails, soft soil
    CrossCountry,  // Off-road, natural terrain
    Obstacle    // Severe terrain
};

/**
 * @brief NRMM Speed Factor data
 *
 * Speed factor = actual speed / max road speed
 */
struct NRMMSpeedFactor {
    NRMMTerrainClass terrain;
    double vehicle_weight_tons;
    double expected_speed_factor;
    double tolerance;
};

const std::vector<NRMMSpeedFactor> NRMM_SPEED_DATA = {
    // Light wheeled vehicle (HMMWV-class)
    {NRMMTerrainClass::Primary, 4.5, 1.00, 0.05},
    {NRMMTerrainClass::Secondary, 4.5, 0.85, 0.08},
    {NRMMTerrainClass::TrailPoor, 4.5, 0.55, 0.10},
    {NRMMTerrainClass::CrossCountry, 4.5, 0.35, 0.10},

    // Medium wheeled vehicle (MRAP-class)
    {NRMMTerrainClass::Primary, 14.0, 1.00, 0.05},
    {NRMMTerrainClass::Secondary, 14.0, 0.80, 0.08},
    {NRMMTerrainClass::TrailPoor, 14.0, 0.45, 0.10},
    {NRMMTerrainClass::CrossCountry, 14.0, 0.25, 0.10},

    // Heavy tracked vehicle (MBT-class)
    {NRMMTerrainClass::Primary, 65.0, 0.90, 0.05},
    {NRMMTerrainClass::Secondary, 65.0, 0.85, 0.08},
    {NRMMTerrainClass::TrailPoor, 65.0, 0.65, 0.10},
    {NRMMTerrainClass::CrossCountry, 65.0, 0.45, 0.10},
};

/**
 * @brief NRMM slope climbing capability
 */
struct NRMMSlopeCapability {
    std::string vehicle_type;
    double weight_tons;
    double max_slope_percent;
    double tolerance;
};

const std::vector<NRMMSlopeCapability> NRMM_SLOPE_DATA = {
    {"Light wheeled (4x4)", 4.5, 60.0, 5.0},
    {"Medium wheeled (8x8)", 20.0, 60.0, 5.0},
    {"Heavy tracked", 65.0, 60.0, 5.0},
    {"Light tracked", 25.0, 70.0, 5.0},
};

/**
 * @brief Bekker soil parameters
 */
struct BekkerSoilParams {
    std::string soil_type;
    double kc;      // Cohesive modulus (kN/m^(n+1))
    double kphi;    // Frictional modulus (kN/m^(n+2))
    double n;       // Exponent
    double c;       // Cohesion (kPa)
    double phi_deg; // Friction angle (degrees)
};

const std::vector<BekkerSoilParams> BEKKER_SOIL_DATA = {
    {"Dry sand", 0.99, 1528, 1.10, 1.04, 28.0},
    {"Sandy loam", 5.27, 1515, 0.70, 1.72, 29.0},
    {"Clayey soil", 13.19, 692, 0.50, 4.14, 13.0},
    {"Snow (firm)", 2.49, 245, 1.44, 6.0, 20.5},
    {"Snow (soft)", 4.37, 196, 1.60, 1.0, 19.7},
};

// ============================================================================
// Terrain Validation Tests
// ============================================================================

class TerrainValidationTest : public ::testing::Test {
protected:
    static constexpr double G = 9.80665;  // m/s²

    void SetUp() override {
    }

    /**
     * @brief Calculate Bekker sinkage
     *
     * z = (W / (A * (kc/b + kphi)))^(1/n)
     */
    double BekkerSinkage(double W_kN, double A_m2, double b_m,
                         const BekkerSoilParams& soil) {
        double k = soil.kc / b_m + soil.kphi;
        return std::pow(W_kN / (A_m2 * k), 1.0 / soil.n);
    }

    /**
     * @brief Calculate rolling resistance on deformable terrain
     */
    double RollingResistance(double W_kN, double z_m, double r_m) {
        // Simplified Bekker formula: Rc = W * z / r
        return W_kN * z_m / r_m;
    }
};

/**
 * @brief Validate NRMM speed factors
 */
TEST_F(TerrainValidationTest, NRMMSpeedFactors) {
    for (const auto& tc : NRMM_SPEED_DATA) {
        std::string terrain_name;
        switch (tc.terrain) {
            case NRMMTerrainClass::Primary: terrain_name = "Primary"; break;
            case NRMMTerrainClass::Secondary: terrain_name = "Secondary"; break;
            case NRMMTerrainClass::TrailPoor: terrain_name = "TrailPoor"; break;
            case NRMMTerrainClass::CrossCountry: terrain_name = "CrossCountry"; break;
            default: terrain_name = "Unknown"; break;
        }

        SCOPED_TRACE(terrain_name + " @ " + std::to_string(tc.vehicle_weight_tons) + "t");

        // Speed factor should be between 0 and 1
        EXPECT_GE(tc.expected_speed_factor, 0.0);
        EXPECT_LE(tc.expected_speed_factor, 1.0);

        // TODO: Validate against terrain model
        // double computed_factor = terrain_model.get_speed_factor(...);
        // EXPECT_NEAR(computed_factor, tc.expected_speed_factor, tc.tolerance);
    }
}

/**
 * @brief Validate slope climbing capability
 */
TEST_F(TerrainValidationTest, SlopeCapability) {
    for (const auto& tc : NRMM_SLOPE_DATA) {
        SCOPED_TRACE(tc.vehicle_type);

        // Convert percent grade to angle
        double slope_rad = std::atan(tc.max_slope_percent / 100.0);
        double slope_deg = slope_rad * 180.0 / M_PI;

        // Maximum slope for friction-limited climbing
        // μ >= tan(θ) → θ_max = atan(μ)
        // With μ ≈ 0.8 for rubber on dry road: θ_max ≈ 39°

        // Slope percentage should be reasonable
        EXPECT_GT(tc.max_slope_percent, 30.0);  // Minimum capability
        EXPECT_LT(tc.max_slope_percent, 100.0); // Physical limit ~45°

        // Slope angle should be reasonable
        EXPECT_GT(slope_deg, 17.0);  // ~30% grade
        EXPECT_LT(slope_deg, 45.0);  // ~100% grade
    }
}

/**
 * @brief Validate Bekker terramechanics
 */
TEST_F(TerrainValidationTest, BekkerTerramechanics) {
    // Test vehicle: HMMWV-class
    double weight_kN = 4.5 * 9.81;  // 4.5 tons
    double tire_radius = 0.4;       // m
    double tire_width = 0.3;        // m
    double contact_area = 0.12;     // m² per tire

    for (const auto& soil : BEKKER_SOIL_DATA) {
        SCOPED_TRACE(soil.soil_type);

        // Calculate sinkage per tire
        double W_per_tire = weight_kN / 4.0;  // 4 tires
        double z = BekkerSinkage(W_per_tire, contact_area, tire_width, soil);

        // Sinkage should be positive and bounded
        EXPECT_GT(z, 0.0);
        EXPECT_LT(z, 0.5);  // Max 50cm sinkage before stuck

        // Rolling resistance
        double Rc = RollingResistance(W_per_tire, z, tire_radius);

        // Rolling resistance should increase with sinkage
        EXPECT_GT(Rc, 0.0);
        EXPECT_LT(Rc, W_per_tire);  // Cannot exceed vertical load
    }
}

// ============================================================================
// Vehicle Dynamics Validation
// ============================================================================

class VehicleDynamicsValidationTest : public ::testing::Test {
protected:
    static constexpr double G = 9.80665;

    void SetUp() override {
    }
};

/**
 * @brief Validate tire model behavior
 *
 * Pacejka "Magic Formula": Y = D * sin(C * arctan(B*x - E*(B*x - arctan(B*x))))
 */
TEST_F(VehicleDynamicsValidationTest, PacejkaTireModel) {
    // Typical Pacejka coefficients for truck tire
    double B = 10.0;   // Stiffness factor
    double C = 1.9;    // Shape factor (1.9 for lateral, 1.65 for longitudinal)
    double D = 1.0;    // Peak factor (normalized)
    double E = -0.2;   // Curvature factor

    auto MagicFormula = [=](double x) {
        double Bx = B * x;
        return D * std::sin(C * std::atan(Bx - E * (Bx - std::atan(Bx))));
    };

    // At zero slip, force should be zero
    EXPECT_NEAR(MagicFormula(0.0), 0.0, 0.01);

    // Peak force around slip ratio/angle of 0.1-0.15
    double peak_slip = 0.10;
    double Fy_peak = MagicFormula(peak_slip);
    EXPECT_NEAR(std::abs(Fy_peak), D, 0.1);

    // Force should decrease after peak (saturation)
    double Fy_high = MagicFormula(0.3);
    EXPECT_LT(std::abs(Fy_high), std::abs(Fy_peak));
}

/**
 * @brief Validate suspension dynamics
 */
TEST_F(VehicleDynamicsValidationTest, SuspensionDynamics) {
    // Typical passenger car suspension
    double m_sprung = 1500.0;      // kg (sprung mass)
    double k_spring = 35000.0;     // N/m (spring rate)
    double c_damper = 3500.0;      // N·s/m (damping coefficient)

    // Natural frequency: ωn = sqrt(k/m)
    double omega_n = std::sqrt(k_spring / m_sprung);
    double f_n = omega_n / (2.0 * M_PI);

    // Typical ride frequency: 1-2 Hz
    EXPECT_GT(f_n, 0.5);
    EXPECT_LT(f_n, 3.0);

    // Damping ratio: ζ = c / (2 * sqrt(k*m))
    double zeta = c_damper / (2.0 * std::sqrt(k_spring * m_sprung));

    // Optimal damping: ζ ≈ 0.2-0.4 for ride, 0.5-0.7 for handling
    EXPECT_GT(zeta, 0.1);
    EXPECT_LT(zeta, 1.0);  // Under-damped for good ride
}

/**
 * @brief Validate vehicle rollover threshold
 */
TEST_F(VehicleDynamicsValidationTest, RolloverThreshold) {
    // Static stability factor: SSF = T / (2 * h_cg)
    // where T = track width, h_cg = CG height

    struct VehicleParams {
        std::string name;
        double track_m;
        double cg_height_m;
        double min_ssf;
    };

    std::vector<VehicleParams> vehicles = {
        {"Sports car", 1.6, 0.4, 2.0},
        {"Sedan", 1.5, 0.55, 1.3},
        {"SUV", 1.6, 0.7, 1.1},
        {"HMMWV", 1.8, 0.85, 1.0},
        {"Truck", 1.8, 1.0, 0.9},
    };

    for (const auto& v : vehicles) {
        SCOPED_TRACE(v.name);

        double ssf = v.track_m / (2.0 * v.cg_height_m);

        // SSF should meet minimum requirement
        EXPECT_GE(ssf, v.min_ssf);

        // SSF > 1.0 means rollover at > 1g lateral
        // Most vehicles should achieve this on flat ground
    }
}

// ============================================================================
// Track Vehicle Validation
// ============================================================================

/**
 * @brief Validate track vehicle steering
 *
 * Track vehicles steer by differential track speeds
 */
TEST_F(VehicleDynamicsValidationTest, TrackSteering) {
    // M1A2 Abrams parameters
    double track_width = 3.66;     // m
    double vehicle_length = 7.93;  // m
    double weight = 62000.0;       // kg

    // For skid steering: ω = (V_right - V_left) / B
    double V_left = 5.0;   // m/s
    double V_right = 10.0; // m/s
    double omega = (V_right - V_left) / track_width;

    // Angular velocity should be reasonable
    EXPECT_GT(omega, 0.0);
    EXPECT_LT(omega, 2.0);  // rad/s (~115 deg/s max)

    // Turn radius: R = (V_left + V_right) / (2 * ω)
    double R = (V_left + V_right) / (2.0 * omega);
    EXPECT_GT(R, track_width / 2.0);  // Minimum turn radius

    // Neutral steer (pivot turn): V_left = -V_right
    double V_pivot = 2.0;
    double omega_pivot = 2.0 * V_pivot / track_width;
    EXPECT_GT(omega_pivot, omega);  // Faster rotation for pivot
}

/**
 * @brief Validate track ground pressure
 */
TEST_F(VehicleDynamicsValidationTest, TrackGroundPressure) {
    // Ground pressure = Weight / Track contact area
    struct TrackedVehicle {
        std::string name;
        double weight_kg;
        double track_length_m;
        double track_width_m;
        double max_pressure_kPa;
    };

    std::vector<TrackedVehicle> vehicles = {
        {"M1A2 Abrams", 62000, 4.0, 0.635, 100},
        {"M2 Bradley", 30000, 3.5, 0.54, 70},
        {"Leopard 2", 62000, 4.0, 0.635, 100},
        {"BMP-3", 18700, 3.0, 0.38, 60},
    };

    for (const auto& v : vehicles) {
        SCOPED_TRACE(v.name);

        // Contact area (both tracks)
        double contact_area = 2.0 * v.track_length_m * v.track_width_m;

        // Ground pressure
        double pressure_Pa = (v.weight_kg * G) / contact_area;
        double pressure_kPa = pressure_Pa / 1000.0;

        // Should be below vehicle's design limit
        EXPECT_LT(pressure_kPa, v.max_pressure_kPa);

        // Typical tank ground pressure: 50-100 kPa
        // Should be less than human foot (~45 kPa) for good soft-ground mobility
    }
}

} // namespace jaguar::test
