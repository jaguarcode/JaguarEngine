/**
 * @file test_sensors.cpp
 * @brief Unit tests for JaguarEngine sensor framework
 *
 * Tests cover:
 * - IMU sensor with various noise models
 * - Dryden turbulence model
 * - Von Karman turbulence model
 * - Sensor state management
 * - Failure injection
 */

#include <gtest/gtest.h>
#include "jaguar/core/types.h"
#include "jaguar/sensors/sensor.h"
#include "jaguar/sensors/imu_sensor.h"
#include "jaguar/environment/atmospheric_disturbance.h"
#include "jaguar/physics/entity.h"
#include <cmath>
#include <numeric>
#include <algorithm>

using namespace jaguar;
using namespace jaguar::sensors;
using namespace jaguar::environment;
using namespace jaguar::physics;

// ============================================================================
// Test Fixtures
// ============================================================================

class SensorTestBase : public ::testing::Test {
protected:
    static constexpr Real EPSILON = 1e-10;
    static constexpr Real LOOSE_EPSILON = 1e-6;

    bool nearly_equal(Real a, Real b, Real eps = EPSILON) {
        return std::abs(a - b) < eps;
    }

    bool nearly_equal(const Vec3& a, const Vec3& b, Real eps = EPSILON) {
        return nearly_equal(a.x, b.x, eps) &&
               nearly_equal(a.y, b.y, eps) &&
               nearly_equal(a.z, b.z, eps);
    }

    // Create a default entity state for testing
    EntityState create_test_entity_state() {
        EntityState state;
        state.position = Vec3{0.0, 0.0, -1000.0};  // 1000m altitude (NED)
        state.velocity = Vec3{100.0, 0.0, 0.0};    // 100 m/s forward
        state.acceleration = Vec3{0.0, 0.0, 9.81}; // 1g upward in NED
        state.orientation = Quat::Identity();
        state.angular_velocity = Vec3{0.0, 0.0, 0.0};
        state.angular_accel = Vec3{0.0, 0.0, 0.0};
        state.mass = 1000.0;
        return state;
    }

    // Calculate mean of a vector
    Real calculate_mean(const std::vector<Real>& values) {
        if (values.empty()) return 0.0;
        return std::accumulate(values.begin(), values.end(), 0.0) / values.size();
    }

    // Calculate standard deviation
    Real calculate_std(const std::vector<Real>& values) {
        if (values.size() < 2) return 0.0;
        Real mean = calculate_mean(values);
        Real sum_sq = 0.0;
        for (Real v : values) {
            sum_sq += (v - mean) * (v - mean);
        }
        return std::sqrt(sum_sq / (values.size() - 1));
    }
};

// ============================================================================
// IMU Sensor Tests
// ============================================================================

class IMUSensorTest : public SensorTestBase {
protected:
    void SetUp() override {
        // Create a tactical-grade IMU for testing
        imu_ = create_tactical_imu("TestIMU", 1);
        imu_->initialize();
    }

    std::unique_ptr<IMUSensor> imu_;
};

TEST_F(IMUSensorTest, Construction) {
    auto imu = std::make_unique<IMUSensor>("TestIMU", IMUGrade::Tactical, 1);
    EXPECT_EQ(imu->get_state(), SensorState::Uninitialized);
}

TEST_F(IMUSensorTest, Initialization) {
    auto imu = std::make_unique<IMUSensor>("TestIMU", IMUGrade::Consumer, 1);
    EXPECT_FALSE(imu->is_operational());

    EXPECT_TRUE(imu->initialize());
    EXPECT_TRUE(imu->is_operational());
    EXPECT_EQ(imu->get_state(), SensorState::Operational);
}

TEST_F(IMUSensorTest, BasicMeasurement) {
    EntityState state = create_test_entity_state();
    Real dt = 0.01;  // 100 Hz

    // Disable noise for this test to get clean measurements
    Vec3NoiseModel no_noise;
    imu_->set_accelerometer_noise(no_noise);
    imu_->set_gyroscope_noise(no_noise);

    // Update sensor
    imu_->update(state, dt);

    // Get measurements
    const auto& accel = imu_->get_acceleration();
    const auto& gyro = imu_->get_angular_velocity();

    EXPECT_TRUE(accel.valid);
    EXPECT_TRUE(gyro.valid);

    // Without noise, measurements should match true values
    const auto& true_accel = imu_->get_true_acceleration();
    const auto& true_gyro = imu_->get_true_angular_velocity();

    EXPECT_TRUE(nearly_equal(accel.value, true_accel, 0.01));
    EXPECT_TRUE(nearly_equal(gyro.value, true_gyro, 0.001));
}

TEST_F(IMUSensorTest, NoiseGeneration) {
    EntityState state = create_test_entity_state();
    Real dt = 0.01;

    // Set seed for reproducibility
    imu_->set_seed(12345);

    // Run multiple updates and collect measurements
    std::vector<Real> accel_x_values;
    const int num_samples = 1000;

    for (int i = 0; i < num_samples; ++i) {
        imu_->update(state, dt);
        const auto& accel = imu_->get_acceleration();
        accel_x_values.push_back(accel.value.x);
    }

    // With noise, we should see variation in measurements
    Real std_dev = calculate_std(accel_x_values);
    EXPECT_GT(std_dev, 0.0) << "Expected non-zero standard deviation from noise";

    // Mean should be close to true value (within 3 sigma)
    Real mean = calculate_mean(accel_x_values);
    Real true_accel_x = imu_->get_true_acceleration().x;
    EXPECT_NEAR(mean, true_accel_x, 3.0 * std_dev);
}

TEST_F(IMUSensorTest, IMUGradeAffectsNoise) {
    EntityState state = create_test_entity_state();
    Real dt = 0.01;
    const int num_samples = 500;

    // Test consumer grade (high noise)
    auto consumer_imu = create_consumer_imu("ConsumerIMU", 1);
    consumer_imu->initialize();
    consumer_imu->set_seed(42);

    std::vector<Real> consumer_values;
    for (int i = 0; i < num_samples; ++i) {
        consumer_imu->update(state, dt);
        consumer_values.push_back(consumer_imu->get_acceleration().value.x);
    }
    Real consumer_std = calculate_std(consumer_values);

    // Test navigation grade (low noise)
    auto nav_imu = create_navigation_imu("NavIMU", 1);
    nav_imu->initialize();
    nav_imu->set_seed(42);

    std::vector<Real> nav_values;
    for (int i = 0; i < num_samples; ++i) {
        nav_imu->update(state, dt);
        nav_values.push_back(nav_imu->get_acceleration().value.x);
    }
    Real nav_std = calculate_std(nav_values);

    // Navigation grade should have lower noise than consumer grade
    EXPECT_LT(nav_std, consumer_std)
        << "Navigation grade IMU should have less noise than consumer grade";
}

TEST_F(IMUSensorTest, IntegratedOutputs) {
    EntityState state = create_test_entity_state();
    Real dt = 0.005;  // 200 Hz

    // Disable noise for predictable integration
    Vec3NoiseModel no_noise;
    imu_->set_accelerometer_noise(no_noise);
    imu_->set_gyroscope_noise(no_noise);

    // Apply a known angular velocity
    state.angular_velocity = Vec3{0.1, 0.0, 0.0};  // 0.1 rad/s roll rate

    // Run for multiple updates
    const int num_steps = 100;
    for (int i = 0; i < num_steps; ++i) {
        imu_->update(state, dt);
    }

    // Get integrated outputs
    Vec3 delta_theta = imu_->get_delta_theta();

    // Expected: 0.1 rad/s * 100 * 0.005s = 0.05 rad
    EXPECT_NEAR(delta_theta.x, 0.05, 0.01);
}

TEST_F(IMUSensorTest, BiasManagement) {
    Vec3 test_bias{0.01, -0.02, 0.03};

    imu_->set_accel_bias(test_bias);
    const Vec3& bias = imu_->get_accel_bias();

    EXPECT_TRUE(nearly_equal(bias, test_bias, 1e-6));
}

TEST_F(IMUSensorTest, FailureInjection) {
    EntityState state = create_test_entity_state();

    // Normal operation
    imu_->update(state, 0.01);
    EXPECT_EQ(imu_->get_failure_mode(), FailureMode::None);
    EXPECT_TRUE(imu_->get_acceleration().valid);

    // Inject complete failure
    imu_->inject_failure(FailureMode::Complete, 1.0);
    imu_->update(state, 0.01);

    EXPECT_EQ(imu_->get_failure_mode(), FailureMode::Complete);
    EXPECT_EQ(imu_->get_state(), SensorState::Failed);

    // Clear failure
    imu_->clear_failures();
    EXPECT_EQ(imu_->get_failure_mode(), FailureMode::None);
}

TEST_F(IMUSensorTest, EnableDisable) {
    EXPECT_TRUE(imu_->is_enabled());

    imu_->set_enabled(false);
    EXPECT_FALSE(imu_->is_enabled());

    imu_->set_enabled(true);
    EXPECT_TRUE(imu_->is_enabled());
}

TEST_F(IMUSensorTest, Reset) {
    EntityState state = create_test_entity_state();

    // Run some updates
    for (int i = 0; i < 100; ++i) {
        imu_->update(state, 0.01);
    }

    // Inject failure
    imu_->inject_failure(FailureMode::Drift, 0.5);

    // Reset
    imu_->reset();

    EXPECT_EQ(imu_->get_failure_mode(), FailureMode::None);
    EXPECT_EQ(imu_->get_state(), SensorState::Uninitialized);
}

// ============================================================================
// Dryden Turbulence Model Tests
// ============================================================================

class DrydenTurbulenceTest : public SensorTestBase {
protected:
    void SetUp() override {
        turbulence_ = std::make_unique<DrydenTurbulenceModel>(TurbulenceSeverity::Moderate);
        turbulence_->set_seed(42);
    }

    std::unique_ptr<DrydenTurbulenceModel> turbulence_;
};

TEST_F(DrydenTurbulenceTest, Construction) {
    DrydenTurbulenceModel model;
    EXPECT_EQ(model.severity(), TurbulenceSeverity::Moderate);
    EXPECT_TRUE(model.is_enabled());
}

TEST_F(DrydenTurbulenceTest, ConstructionWithSeverity) {
    DrydenTurbulenceModel model(TurbulenceSeverity::Severe);
    EXPECT_EQ(model.severity(), TurbulenceSeverity::Severe);
}

TEST_F(DrydenTurbulenceTest, SeverityChange) {
    turbulence_->set_severity(TurbulenceSeverity::Light);
    EXPECT_EQ(turbulence_->severity(), TurbulenceSeverity::Light);

    turbulence_->set_severity(TurbulenceSeverity::Extreme);
    EXPECT_EQ(turbulence_->severity(), TurbulenceSeverity::Extreme);
}

TEST_F(DrydenTurbulenceTest, NoTurbulenceWhenDisabled) {
    turbulence_->set_enabled(false);

    TurbulenceOutput output = turbulence_->update(100.0, 1000.0, 10.0, 0.01);

    EXPECT_TRUE(output.is_negligible());
}

TEST_F(DrydenTurbulenceTest, NoTurbulenceWhenNoneSeverity) {
    turbulence_->set_severity(TurbulenceSeverity::None);

    TurbulenceOutput output = turbulence_->update(100.0, 1000.0, 10.0, 0.01);

    EXPECT_TRUE(output.is_negligible());
}

TEST_F(DrydenTurbulenceTest, GeneratesTurbulence) {
    // Run multiple updates and verify we get non-zero output
    bool found_nonzero = false;
    for (int i = 0; i < 100; ++i) {
        TurbulenceOutput output = turbulence_->update(100.0, 1000.0, 10.0, 0.01);
        if (!output.is_negligible()) {
            found_nonzero = true;
            break;
        }
    }

    EXPECT_TRUE(found_nonzero) << "Turbulence model should generate non-zero output";
}

TEST_F(DrydenTurbulenceTest, SeverityAffectsMagnitude) {
    const int num_samples = 1000;
    Real dt = 0.01;
    Real airspeed = 100.0;
    Real altitude = 1000.0;
    Real wingspan = 10.0;

    // Light turbulence
    turbulence_->set_severity(TurbulenceSeverity::Light);
    turbulence_->set_seed(42);
    turbulence_->reset();

    std::vector<Real> light_magnitudes;
    for (int i = 0; i < num_samples; ++i) {
        TurbulenceOutput output = turbulence_->update(airspeed, altitude, wingspan, dt);
        light_magnitudes.push_back(output.velocity.length());
    }
    Real light_mean = calculate_mean(light_magnitudes);

    // Severe turbulence
    turbulence_->set_severity(TurbulenceSeverity::Severe);
    turbulence_->set_seed(42);
    turbulence_->reset();

    std::vector<Real> severe_magnitudes;
    for (int i = 0; i < num_samples; ++i) {
        TurbulenceOutput output = turbulence_->update(airspeed, altitude, wingspan, dt);
        severe_magnitudes.push_back(output.velocity.length());
    }
    Real severe_mean = calculate_mean(severe_magnitudes);

    // Severe turbulence should have larger magnitude than light
    EXPECT_GT(severe_mean, light_mean)
        << "Severe turbulence should have larger magnitude than light turbulence";
}

TEST_F(DrydenTurbulenceTest, AltitudeAffectsScaleLength) {
    // At low altitude, turbulence scale length should be smaller
    Vec3 low_alt_scales = turbulence_->get_scale_lengths();

    // Test at different altitudes (will update internal parameters)
    turbulence_->update(100.0, 100.0, 10.0, 0.01);  // 100m altitude
    Vec3 scale_100m = turbulence_->get_scale_lengths();

    turbulence_->update(100.0, 2000.0, 10.0, 0.01);  // 2000m altitude
    Vec3 scale_2000m = turbulence_->get_scale_lengths();

    // At 2000m, scale lengths should be at the high-altitude constant value
    // which is larger than the low-altitude value
    EXPECT_GT(scale_2000m.x, scale_100m.x);
}

TEST_F(DrydenTurbulenceTest, CustomIntensity) {
    turbulence_->set_intensity(5.0, 5.0, 5.0);

    Vec3 intensity = turbulence_->get_intensity();
    EXPECT_NEAR(intensity.x, 5.0, 0.001);
    EXPECT_NEAR(intensity.y, 5.0, 0.001);
    EXPECT_NEAR(intensity.z, 5.0, 0.001);
}

TEST_F(DrydenTurbulenceTest, CustomScaleLengths) {
    turbulence_->set_scale_lengths(500.0, 500.0, 100.0);

    Vec3 scales = turbulence_->get_scale_lengths();
    EXPECT_NEAR(scales.x, 500.0, 0.001);
    EXPECT_NEAR(scales.y, 500.0, 0.001);
    EXPECT_NEAR(scales.z, 100.0, 0.001);
}

TEST_F(DrydenTurbulenceTest, Reset) {
    // Run some updates to build up filter state
    for (int i = 0; i < 100; ++i) {
        turbulence_->update(100.0, 1000.0, 10.0, 0.01);
    }

    // Reset
    turbulence_->reset();

    // After reset, first output should be based on initial state
    TurbulenceOutput output = turbulence_->last_output();
    EXPECT_TRUE(output.is_negligible());
}

TEST_F(DrydenTurbulenceTest, Reproducibility) {
    turbulence_->set_seed(12345);
    turbulence_->reset();

    std::vector<Real> outputs1;
    for (int i = 0; i < 100; ++i) {
        TurbulenceOutput output = turbulence_->update(100.0, 1000.0, 10.0, 0.01);
        outputs1.push_back(output.velocity.x);
    }

    // Reset with same seed
    turbulence_->set_seed(12345);
    turbulence_->reset();

    std::vector<Real> outputs2;
    for (int i = 0; i < 100; ++i) {
        TurbulenceOutput output = turbulence_->update(100.0, 1000.0, 10.0, 0.01);
        outputs2.push_back(output.velocity.x);
    }

    // Outputs should be identical
    ASSERT_EQ(outputs1.size(), outputs2.size());
    for (size_t i = 0; i < outputs1.size(); ++i) {
        EXPECT_NEAR(outputs1[i], outputs2[i], 1e-10)
            << "Mismatch at index " << i;
    }
}

// ============================================================================
// Von Karman Turbulence Model Tests
// ============================================================================

class VonKarmanTurbulenceTest : public SensorTestBase {
protected:
    void SetUp() override {
        turbulence_ = std::make_unique<VonKarmanTurbulenceModel>(TurbulenceSeverity::Moderate);
        turbulence_->set_seed(42);
    }

    std::unique_ptr<VonKarmanTurbulenceModel> turbulence_;
};

TEST_F(VonKarmanTurbulenceTest, Construction) {
    VonKarmanTurbulenceModel model;
    EXPECT_EQ(model.severity(), TurbulenceSeverity::Moderate);
    EXPECT_TRUE(model.is_enabled());
    EXPECT_EQ(model.filter_order(), 4);  // Default filter order
}

TEST_F(VonKarmanTurbulenceTest, FilterOrderConfiguration) {
    turbulence_->set_filter_order(6);
    EXPECT_EQ(turbulence_->filter_order(), 6);

    // Test clamping to valid range
    turbulence_->set_filter_order(1);  // Below minimum
    EXPECT_GE(turbulence_->filter_order(), 2);

    turbulence_->set_filter_order(20);  // Above maximum
    EXPECT_LE(turbulence_->filter_order(), 8);
}

TEST_F(VonKarmanTurbulenceTest, GeneratesTurbulence) {
    bool found_nonzero = false;
    for (int i = 0; i < 100; ++i) {
        TurbulenceOutput output = turbulence_->update(100.0, 1000.0, 10.0, 0.01);
        if (!output.is_negligible()) {
            found_nonzero = true;
            break;
        }
    }

    EXPECT_TRUE(found_nonzero) << "Von Karman model should generate non-zero output";
}

TEST_F(VonKarmanTurbulenceTest, CompareWithDryden) {
    // Von Karman should have similar statistical properties to Dryden
    // but with potentially different spectral characteristics

    const int num_samples = 500;
    Real dt = 0.01;
    Real airspeed = 100.0;
    Real altitude = 1000.0;
    Real wingspan = 10.0;

    // Von Karman
    turbulence_->set_seed(42);
    turbulence_->reset();

    std::vector<Real> vk_values;
    for (int i = 0; i < num_samples; ++i) {
        TurbulenceOutput output = turbulence_->update(airspeed, altitude, wingspan, dt);
        vk_values.push_back(output.velocity.x);
    }
    Real vk_std = calculate_std(vk_values);

    // Dryden
    DrydenTurbulenceModel dryden(TurbulenceSeverity::Moderate);
    dryden.set_seed(42);

    std::vector<Real> dryden_values;
    for (int i = 0; i < num_samples; ++i) {
        TurbulenceOutput output = dryden.update(airspeed, altitude, wingspan, dt);
        dryden_values.push_back(output.velocity.x);
    }
    Real dryden_std = calculate_std(dryden_values);

    // Both models should produce non-trivial output
    EXPECT_GT(vk_std, 0.0);
    EXPECT_GT(dryden_std, 0.0);

    // They should be in the same order of magnitude (within 5x)
    EXPECT_GT(vk_std, dryden_std / 5.0);
    EXPECT_LT(vk_std, dryden_std * 5.0);
}

// ============================================================================
// Turbulence Utility Function Tests
// ============================================================================

TEST_F(SensorTestBase, TurbulenceIntensityByAltitude) {
    // Low altitude should have less intensity than medium altitude
    Real intensity_100m = get_turbulence_intensity(TurbulenceSeverity::Moderate, 100.0);
    Real intensity_500m = get_turbulence_intensity(TurbulenceSeverity::Moderate, 500.0);

    // Per MIL-F-8785C, intensity scales with (h/1000)^(1/3) at low altitude
    EXPECT_LT(intensity_100m, intensity_500m);

    // None severity should return 0
    Real intensity_none = get_turbulence_intensity(TurbulenceSeverity::None, 1000.0);
    EXPECT_EQ(intensity_none, 0.0);
}

TEST_F(SensorTestBase, TurbulenceIntensityBySeverity) {
    Real altitude = 1000.0;

    Real light = get_turbulence_intensity(TurbulenceSeverity::Light, altitude);
    Real moderate = get_turbulence_intensity(TurbulenceSeverity::Moderate, altitude);
    Real severe = get_turbulence_intensity(TurbulenceSeverity::Severe, altitude);
    Real extreme = get_turbulence_intensity(TurbulenceSeverity::Extreme, altitude);

    EXPECT_LT(light, moderate);
    EXPECT_LT(moderate, severe);
    EXPECT_LT(severe, extreme);
}

TEST_F(SensorTestBase, ScaleLengthByAltitude) {
    // Test horizontal scale length
    Real L_h_low = get_scale_length(100.0, false);
    Real L_h_high = get_scale_length(2000.0, false);

    // At high altitude, scale length should be constant (1750 ft)
    EXPECT_NEAR(L_h_high, 1750.0 * 0.3048, 1.0);

    // At low altitude, scale length should be smaller
    EXPECT_LT(L_h_low, L_h_high);

    // Test vertical scale length at low altitude equals altitude
    Real L_w_low = get_scale_length(100.0, true);
    EXPECT_NEAR(L_w_low, 100.0, 1.0);
}

// ============================================================================
// Factory Function Tests
// ============================================================================

TEST_F(SensorTestBase, IMUFactoryFunctions) {
    auto consumer = create_consumer_imu("Consumer", 1);
    auto tactical = create_tactical_imu("Tactical", 2);
    auto navigation = create_navigation_imu("Navigation", 3);

    EXPECT_NE(consumer, nullptr);
    EXPECT_NE(tactical, nullptr);
    EXPECT_NE(navigation, nullptr);

    // Verify correct grades were assigned
    EXPECT_EQ(consumer->imu_config().grade, IMUGrade::Consumer);
    EXPECT_EQ(tactical->imu_config().grade, IMUGrade::Tactical);
    EXPECT_EQ(navigation->imu_config().grade, IMUGrade::Navigation);
}

TEST_F(SensorTestBase, TurbulenceFactoryFunctions) {
    auto dryden = create_dryden_turbulence(TurbulenceSeverity::Light);
    auto vonkarman = create_vonkarman_turbulence(TurbulenceSeverity::Severe);

    EXPECT_NE(dryden, nullptr);
    EXPECT_NE(vonkarman, nullptr);

    EXPECT_EQ(dryden->severity(), TurbulenceSeverity::Light);
    EXPECT_EQ(vonkarman->severity(), TurbulenceSeverity::Severe);
}

// ============================================================================
// Integration Tests
// ============================================================================

class SensorIntegrationTest : public SensorTestBase {
protected:
    void SetUp() override {
        // Create IMU with tactical grade
        imu_ = create_tactical_imu("TestIMU", 1);
        imu_->initialize();

        // Create Dryden turbulence model
        turbulence_ = create_dryden_turbulence(TurbulenceSeverity::Moderate);
        turbulence_->set_seed(42);
    }

    std::unique_ptr<IMUSensor> imu_;
    std::unique_ptr<ITurbulenceModel> turbulence_;
};

TEST_F(SensorIntegrationTest, IMUWithTurbulence) {
    // Simulate an aircraft flying through turbulence
    EntityState state = create_test_entity_state();
    Real dt = 0.01;

    for (int i = 0; i < 1000; ++i) {
        // Update turbulence
        TurbulenceOutput turb = turbulence_->update(100.0, 1000.0, 10.0, dt);

        // Add turbulence to entity state
        state.velocity = state.velocity + turb.velocity;
        state.angular_velocity = state.angular_velocity + turb.angular_rate;

        // Update IMU
        imu_->update(state, dt);

        // Verify IMU is still operational
        EXPECT_TRUE(imu_->is_operational());
        EXPECT_TRUE(imu_->get_acceleration().valid);
        EXPECT_TRUE(imu_->get_angular_velocity().valid);
    }
}

TEST_F(SensorIntegrationTest, LongDurationSimulation) {
    // Test stability over a long simulation
    EntityState state = create_test_entity_state();
    Real dt = 0.01;
    Real total_time = 60.0;  // 1 minute simulation
    int steps = static_cast<int>(total_time / dt);

    std::vector<Real> accel_x_history;

    for (int i = 0; i < steps; ++i) {
        imu_->update(state, dt);
        accel_x_history.push_back(imu_->get_acceleration().value.x);
    }

    // Verify no NaN or Inf values
    for (size_t i = 0; i < accel_x_history.size(); ++i) {
        EXPECT_FALSE(std::isnan(accel_x_history[i]))
            << "NaN detected at step " << i;
        EXPECT_FALSE(std::isinf(accel_x_history[i]))
            << "Inf detected at step " << i;
    }

    // Verify sensor is still operational
    EXPECT_TRUE(imu_->is_operational());
}

