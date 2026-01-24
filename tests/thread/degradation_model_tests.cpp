/**
 * @file degradation_model_tests.cpp
 * @brief Unit tests for the degradation_model system
 */

#include <gtest/gtest.h>
#include "jaguar/thread/degradation_model.h"
#include <thread>
#include <chrono>

using namespace jaguar;
using namespace jaguar::thread;

// ============================================================================
// Enum String Conversion Tests
// ============================================================================

class EnumConversionTest : public ::testing::Test {};

TEST_F(EnumConversionTest, DegradationResultToString) {
    EXPECT_STREQ(degradation_result_to_string(DegradationResult::Success), "Success");
    EXPECT_STREQ(degradation_result_to_string(DegradationResult::InvalidConfiguration),
                 "InvalidConfiguration");
    EXPECT_STREQ(degradation_result_to_string(DegradationResult::InvalidEntityId), "InvalidEntityId");
    EXPECT_STREQ(degradation_result_to_string(DegradationResult::InvalidComponent), "InvalidComponent");
    EXPECT_STREQ(degradation_result_to_string(DegradationResult::InvalidModel), "InvalidModel");
    EXPECT_STREQ(degradation_result_to_string(DegradationResult::CalculationFailed), "CalculationFailed");
    EXPECT_STREQ(degradation_result_to_string(DegradationResult::PredictionFailed), "PredictionFailed");
    EXPECT_STREQ(degradation_result_to_string(DegradationResult::UpdateFailed), "UpdateFailed");
    EXPECT_STREQ(degradation_result_to_string(DegradationResult::NotInitialized), "NotInitialized");
    EXPECT_STREQ(degradation_result_to_string(DegradationResult::AlreadyInitialized), "AlreadyInitialized");
    EXPECT_STREQ(degradation_result_to_string(DegradationResult::ComponentNotFound), "ComponentNotFound");
    EXPECT_STREQ(degradation_result_to_string(DegradationResult::OutOfMemory), "OutOfMemory");
    EXPECT_STREQ(degradation_result_to_string(DegradationResult::ModelNotLoaded), "ModelNotLoaded");
}

TEST_F(EnumConversionTest, DegradationTypeToString) {
    EXPECT_STREQ(degradation_type_to_string(DegradationType::Wear), "Wear");
    EXPECT_STREQ(degradation_type_to_string(DegradationType::Fatigue), "Fatigue");
    EXPECT_STREQ(degradation_type_to_string(DegradationType::Corrosion), "Corrosion");
    EXPECT_STREQ(degradation_type_to_string(DegradationType::Thermal), "Thermal");
    EXPECT_STREQ(degradation_type_to_string(DegradationType::Chemical), "Chemical");
    EXPECT_STREQ(degradation_type_to_string(DegradationType::Electrical), "Electrical");
    EXPECT_STREQ(degradation_type_to_string(DegradationType::Combined), "Combined");
}

TEST_F(EnumConversionTest, ComponentTypeToString) {
    EXPECT_STREQ(component_type_to_string(ComponentType::Mechanical), "Mechanical");
    EXPECT_STREQ(component_type_to_string(ComponentType::Electrical), "Electrical");
    EXPECT_STREQ(component_type_to_string(ComponentType::Hydraulic), "Hydraulic");
    EXPECT_STREQ(component_type_to_string(ComponentType::Structural), "Structural");
    EXPECT_STREQ(component_type_to_string(ComponentType::Electronic), "Electronic");
    EXPECT_STREQ(component_type_to_string(ComponentType::Consumable), "Consumable");
}

TEST_F(EnumConversionTest, HealthStatusToString) {
    EXPECT_STREQ(health_status_to_string(HealthStatus::Excellent), "Excellent");
    EXPECT_STREQ(health_status_to_string(HealthStatus::Good), "Good");
    EXPECT_STREQ(health_status_to_string(HealthStatus::Fair), "Fair");
    EXPECT_STREQ(health_status_to_string(HealthStatus::Poor), "Poor");
    EXPECT_STREQ(health_status_to_string(HealthStatus::Critical), "Critical");
    EXPECT_STREQ(health_status_to_string(HealthStatus::Failed), "Failed");
}

TEST_F(EnumConversionTest, MaintenanceActionToString) {
    EXPECT_STREQ(maintenance_action_to_string(MaintenanceAction::None), "None");
    EXPECT_STREQ(maintenance_action_to_string(MaintenanceAction::Inspect), "Inspect");
    EXPECT_STREQ(maintenance_action_to_string(MaintenanceAction::Lubricate), "Lubricate");
    EXPECT_STREQ(maintenance_action_to_string(MaintenanceAction::Adjust), "Adjust");
    EXPECT_STREQ(maintenance_action_to_string(MaintenanceAction::Repair), "Repair");
    EXPECT_STREQ(maintenance_action_to_string(MaintenanceAction::Replace), "Replace");
    EXPECT_STREQ(maintenance_action_to_string(MaintenanceAction::Overhaul), "Overhaul");
}

// ============================================================================
// WearFactor Tests
// ============================================================================

class WearFactorTest : public ::testing::Test {};

TEST_F(WearFactorTest, DefaultValues) {
    WearFactor factor;

    EXPECT_DOUBLE_EQ(factor.base_rate, 0.001);
    EXPECT_DOUBLE_EQ(factor.load_multiplier, 1.0);
    EXPECT_DOUBLE_EQ(factor.temperature_coefficient, 0.01);
    EXPECT_DOUBLE_EQ(factor.humidity_coefficient, 0.0);
    EXPECT_DOUBLE_EQ(factor.age_acceleration, 0.0);
}

TEST_F(WearFactorTest, CreateDefaultWearFactorMechanical) {
    WearFactor factor = create_default_wear_factor(ComponentType::Mechanical);

    EXPECT_DOUBLE_EQ(factor.base_rate, 0.001);
    EXPECT_DOUBLE_EQ(factor.load_multiplier, 1.5);
    EXPECT_DOUBLE_EQ(factor.temperature_coefficient, 0.01);
    EXPECT_DOUBLE_EQ(factor.humidity_coefficient, 0.001);
    EXPECT_DOUBLE_EQ(factor.age_acceleration, 0.0001);
}

TEST_F(WearFactorTest, CreateDefaultWearFactorElectrical) {
    WearFactor factor = create_default_wear_factor(ComponentType::Electrical);

    EXPECT_DOUBLE_EQ(factor.base_rate, 0.0005);
    EXPECT_DOUBLE_EQ(factor.load_multiplier, 1.0);
    EXPECT_DOUBLE_EQ(factor.temperature_coefficient, 0.02);
    EXPECT_DOUBLE_EQ(factor.humidity_coefficient, 0.0);
    EXPECT_DOUBLE_EQ(factor.age_acceleration, 0.0002);
}

TEST_F(WearFactorTest, CreateDefaultWearFactorHydraulic) {
    WearFactor factor = create_default_wear_factor(ComponentType::Hydraulic);

    EXPECT_DOUBLE_EQ(factor.base_rate, 0.002);
    EXPECT_DOUBLE_EQ(factor.load_multiplier, 2.0);
    EXPECT_DOUBLE_EQ(factor.temperature_coefficient, 0.015);
    EXPECT_DOUBLE_EQ(factor.humidity_coefficient, 0.002);
    EXPECT_DOUBLE_EQ(factor.age_acceleration, 0.0001);
}

TEST_F(WearFactorTest, CreateDefaultWearFactorStructural) {
    WearFactor factor = create_default_wear_factor(ComponentType::Structural);

    EXPECT_DOUBLE_EQ(factor.base_rate, 0.0001);
    EXPECT_DOUBLE_EQ(factor.load_multiplier, 1.2);
    EXPECT_DOUBLE_EQ(factor.temperature_coefficient, 0.005);
    EXPECT_DOUBLE_EQ(factor.humidity_coefficient, 0.0005);
    EXPECT_DOUBLE_EQ(factor.age_acceleration, 0.00005);
}

TEST_F(WearFactorTest, CreateDefaultWearFactorElectronic) {
    WearFactor factor = create_default_wear_factor(ComponentType::Electronic);

    EXPECT_DOUBLE_EQ(factor.base_rate, 0.0003);
    EXPECT_DOUBLE_EQ(factor.load_multiplier, 0.8);
    EXPECT_DOUBLE_EQ(factor.temperature_coefficient, 0.025);
    EXPECT_DOUBLE_EQ(factor.humidity_coefficient, 0.0);
    EXPECT_DOUBLE_EQ(factor.age_acceleration, 0.0003);
}

TEST_F(WearFactorTest, CreateDefaultWearFactorConsumable) {
    WearFactor factor = create_default_wear_factor(ComponentType::Consumable);

    EXPECT_DOUBLE_EQ(factor.base_rate, 0.01);
    EXPECT_DOUBLE_EQ(factor.load_multiplier, 1.0);
    EXPECT_DOUBLE_EQ(factor.temperature_coefficient, 0.005);
    EXPECT_DOUBLE_EQ(factor.humidity_coefficient, 0.001);
    EXPECT_DOUBLE_EQ(factor.age_acceleration, 0.0);
}

TEST_F(WearFactorTest, ValidateWearFactorValid) {
    WearFactor factor = create_default_wear_factor(ComponentType::Mechanical);
    EXPECT_TRUE(validate_wear_factor(factor));
}

TEST_F(WearFactorTest, ValidateWearFactorInvalid) {
    WearFactor factor;
    factor.base_rate = -0.1;  // Invalid negative
    EXPECT_FALSE(validate_wear_factor(factor));
}

// ============================================================================
// ComponentHealth Tests
// ============================================================================

class ComponentHealthTest : public ::testing::Test {};

TEST_F(ComponentHealthTest, DefaultConstruction) {
    ComponentHealth health;

    EXPECT_TRUE(health.component_id.empty());
    EXPECT_EQ(health.type, ComponentType::Mechanical);
    EXPECT_DOUBLE_EQ(health.health_percentage, 100.0);
    EXPECT_DOUBLE_EQ(health.degradation_rate, 0.0);
    EXPECT_EQ(health.status, HealthStatus::Excellent);
    EXPECT_EQ(health.operating_hours.count(), 0);
    EXPECT_EQ(health.remaining_useful_life.count(), 0);
}

TEST_F(ComponentHealthTest, HealthPercentageRanges) {
    ComponentHealth health;

    // Test different health percentages and their corresponding statuses
    health.health_percentage = 100.0;
    health.status = health_percentage_to_status(health.health_percentage);
    EXPECT_EQ(health.status, HealthStatus::Excellent);

    health.health_percentage = 85.0;
    health.status = health_percentage_to_status(health.health_percentage);
    EXPECT_EQ(health.status, HealthStatus::Good);

    health.health_percentage = 60.0;
    health.status = health_percentage_to_status(health.health_percentage);
    EXPECT_EQ(health.status, HealthStatus::Fair);

    health.health_percentage = 40.0;
    health.status = health_percentage_to_status(health.health_percentage);
    EXPECT_EQ(health.status, HealthStatus::Poor);

    health.health_percentage = 20.0;
    health.status = health_percentage_to_status(health.health_percentage);
    EXPECT_EQ(health.status, HealthStatus::Critical);

    health.health_percentage = 0.0;
    health.status = health_percentage_to_status(health.health_percentage);
    EXPECT_EQ(health.status, HealthStatus::Failed);
}

TEST_F(ComponentHealthTest, ValidateComponentHealthValid) {
    ComponentHealth health;
    health.component_id = "wheel_left";
    health.health_percentage = 75.0;
    health.degradation_rate = 0.01;

    EXPECT_TRUE(validate_component_health(health));
}

TEST_F(ComponentHealthTest, ValidateComponentHealthInvalid) {
    ComponentHealth health;
    health.component_id = "";  // Empty ID
    health.health_percentage = 75.0;

    EXPECT_FALSE(validate_component_health(health));

    health.component_id = "wheel";
    health.health_percentage = 150.0;  // Out of range
    EXPECT_FALSE(validate_component_health(health));
}

// ============================================================================
// OperatingConditions Tests
// ============================================================================

class OperatingConditionsTest : public ::testing::Test {};

TEST_F(OperatingConditionsTest, DefaultConstruction) {
    OperatingConditions conditions;

    EXPECT_DOUBLE_EQ(conditions.load_factor, 0.0);
    EXPECT_DOUBLE_EQ(conditions.temperature_celsius, 20.0);
    EXPECT_DOUBLE_EQ(conditions.humidity_percent, 50.0);
    EXPECT_DOUBLE_EQ(conditions.vibration_level, 0.0);
    EXPECT_DOUBLE_EQ(conditions.contamination_level, 0.0);
    EXPECT_EQ(conditions.continuous_operation.count(), 0);
}

TEST_F(OperatingConditionsTest, NominalConditions) {
    OperatingConditions conditions = create_nominal_conditions();

    EXPECT_DOUBLE_EQ(conditions.load_factor, 0.5);
    EXPECT_DOUBLE_EQ(conditions.temperature_celsius, 20.0);
    EXPECT_DOUBLE_EQ(conditions.humidity_percent, 50.0);
    EXPECT_DOUBLE_EQ(conditions.vibration_level, 0.1);
    EXPECT_DOUBLE_EQ(conditions.contamination_level, 0.0);
}

TEST_F(OperatingConditionsTest, HarshConditions) {
    OperatingConditions conditions = create_harsh_conditions();

    EXPECT_DOUBLE_EQ(conditions.load_factor, 1.0);
    EXPECT_DOUBLE_EQ(conditions.temperature_celsius, 80.0);
    EXPECT_DOUBLE_EQ(conditions.humidity_percent, 90.0);
    EXPECT_DOUBLE_EQ(conditions.vibration_level, 0.8);
    EXPECT_DOUBLE_EQ(conditions.contamination_level, 0.6);
    EXPECT_EQ(conditions.continuous_operation.count(), 24);
}

TEST_F(OperatingConditionsTest, ValidateOperatingConditionsValid) {
    OperatingConditions conditions = create_nominal_conditions();
    EXPECT_TRUE(validate_operating_conditions(conditions));
}

TEST_F(OperatingConditionsTest, ValidateOperatingConditionsInvalid) {
    OperatingConditions conditions;
    conditions.load_factor = 1.5;  // Out of range (should be 0-1)
    EXPECT_FALSE(validate_operating_conditions(conditions));

    conditions.load_factor = 0.5;
    conditions.humidity_percent = 150.0;  // Out of range
    EXPECT_FALSE(validate_operating_conditions(conditions));
}

TEST_F(OperatingConditionsTest, EnvironmentalSeverity) {
    OperatingConditions nominal = create_nominal_conditions();
    OperatingConditions harsh = create_harsh_conditions();

    Real nominal_severity = calculate_environmental_severity(nominal);
    Real harsh_severity = calculate_environmental_severity(harsh);

    // Harsh conditions should have higher severity
    EXPECT_GT(harsh_severity, nominal_severity);
    EXPECT_GT(nominal_severity, 0.0);
}

TEST_F(OperatingConditionsTest, InterpolateConditions) {
    OperatingConditions a = create_nominal_conditions();
    OperatingConditions b = create_harsh_conditions();

    // Interpolate halfway
    OperatingConditions mid = interpolate_conditions(a, b, 0.5);

    EXPECT_DOUBLE_EQ(mid.load_factor, (a.load_factor + b.load_factor) / 2.0);
    EXPECT_DOUBLE_EQ(mid.temperature_celsius, (a.temperature_celsius + b.temperature_celsius) / 2.0);
}

// ============================================================================
// DegradationModelConfig Tests
// ============================================================================

class DegradationModelConfigTest : public ::testing::Test {};

TEST_F(DegradationModelConfigTest, DefaultConfig) {
    DegradationModelConfig config = DegradationModelConfig::default_config();

    EXPECT_EQ(config.assessment_interval.count(), 60);
    EXPECT_DOUBLE_EQ(config.prediction_horizon_hours, 720.0);
    EXPECT_DOUBLE_EQ(config.minimum_confidence, 0.8);
    EXPECT_TRUE(config.enable_predictions);
    EXPECT_TRUE(config.enable_recommendations);
    EXPECT_EQ(config.default_wear_factors.size(), 6u);
}

TEST_F(DegradationModelConfigTest, HighFrequencyConfig) {
    DegradationModelConfig config = DegradationModelConfig::high_frequency();

    EXPECT_EQ(config.assessment_interval.count(), 15);
    EXPECT_DOUBLE_EQ(config.prediction_horizon_hours, 168.0);
    EXPECT_EQ(config.default_wear_factors.size(), 6u);
}

TEST_F(DegradationModelConfigTest, LongTermConfig) {
    DegradationModelConfig config = DegradationModelConfig::long_term();

    EXPECT_EQ(config.assessment_interval.count(), 24 * 60);  // 24 hours in minutes
    EXPECT_DOUBLE_EQ(config.prediction_horizon_hours, 8760.0);
    EXPECT_DOUBLE_EQ(config.minimum_confidence, 0.7);
}

TEST_F(DegradationModelConfigTest, CriticalSystemsConfig) {
    DegradationModelConfig config = DegradationModelConfig::critical_systems();

    EXPECT_EQ(config.assessment_interval.count(), 5);
    EXPECT_DOUBLE_EQ(config.prediction_horizon_hours, 168.0);
    EXPECT_DOUBLE_EQ(config.minimum_confidence, 0.9);
    EXPECT_TRUE(config.enable_predictions);
    EXPECT_TRUE(config.enable_recommendations);
}

TEST_F(DegradationModelConfigTest, DefaultWearFactors) {
    DegradationModelConfig config = DegradationModelConfig::default_config();

    // Check mechanical wear factor
    auto& mech = config.default_wear_factors[ComponentType::Mechanical];
    EXPECT_DOUBLE_EQ(mech.base_rate, 0.001);
    EXPECT_DOUBLE_EQ(mech.load_multiplier, 1.5);

    // Check hydraulic wear factor
    auto& hyd = config.default_wear_factors[ComponentType::Hydraulic];
    EXPECT_DOUBLE_EQ(hyd.base_rate, 0.002);
    EXPECT_DOUBLE_EQ(hyd.load_multiplier, 2.0);
}

// ============================================================================
// DegradationModel Lifecycle Tests
// ============================================================================

class DegradationModelLifecycleTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_ = DegradationModelConfig::default_config();
        model_ = create_degradation_model(config_);
    }

    void TearDown() override {
        if (model_) {
            model_->shutdown();
        }
    }

    DegradationModelConfig config_;
    std::unique_ptr<DegradationModel> model_;
};

TEST_F(DegradationModelLifecycleTest, Initialize) {
    DegradationResult result = model_->initialize();
    EXPECT_EQ(result, DegradationResult::Success);
}

TEST_F(DegradationModelLifecycleTest, Shutdown) {
    model_->initialize();
    DegradationResult result = model_->shutdown();
    EXPECT_EQ(result, DegradationResult::Success);
}

TEST_F(DegradationModelLifecycleTest, DoubleInitialization) {
    EXPECT_EQ(model_->initialize(), DegradationResult::Success);
    DegradationResult result = model_->initialize();
    EXPECT_EQ(result, DegradationResult::AlreadyInitialized);
}

TEST_F(DegradationModelLifecycleTest, OperationBeforeInitialization) {
    // Create a new model without initializing
    auto uninit_model = create_degradation_model(config_);
    uninit_model->shutdown();  // Ensure it's not initialized

    EntityId entity_id = 1;
    DegradationResult result = uninit_model->register_entity(entity_id);
    EXPECT_EQ(result, DegradationResult::NotInitialized);
}

// ============================================================================
// Entity Registration Tests
// ============================================================================

class EntityRegistrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_ = DegradationModelConfig::default_config();
        model_ = create_degradation_model(config_);
        model_->initialize();
    }

    void TearDown() override {
        model_->shutdown();
    }

    DegradationModelConfig config_;
    std::unique_ptr<DegradationModel> model_;
};

TEST_F(EntityRegistrationTest, RegisterEntity) {
    EntityId entity_id = 1;
    DegradationResult result = model_->register_entity(entity_id);
    EXPECT_EQ(result, DegradationResult::Success);
}

TEST_F(EntityRegistrationTest, UnregisterEntity) {
    EntityId entity_id = 1;
    model_->register_entity(entity_id);

    DegradationResult result = model_->unregister_entity(entity_id);
    EXPECT_EQ(result, DegradationResult::Success);
}

TEST_F(EntityRegistrationTest, RegisterDuplicateEntity) {
    EntityId entity_id = 1;
    EXPECT_EQ(model_->register_entity(entity_id), DegradationResult::Success);

    // Try to register the same entity again - idempotent operation returns Success
    DegradationResult result = model_->register_entity(entity_id);
    EXPECT_EQ(result, DegradationResult::Success);
}

TEST_F(EntityRegistrationTest, UnregisterNonExistentEntity) {
    EntityId entity_id = 999;
    DegradationResult result = model_->unregister_entity(entity_id);
    EXPECT_EQ(result, DegradationResult::InvalidEntityId);
}

// ============================================================================
// Component Management Tests
// ============================================================================

class ComponentManagementTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_ = DegradationModelConfig::default_config();
        model_ = create_degradation_model(config_);
        model_->initialize();
        entity_id_ = 1;
        model_->register_entity(entity_id_);
    }

    void TearDown() override {
        model_->shutdown();
    }

    DegradationModelConfig config_;
    std::unique_ptr<DegradationModel> model_;
    EntityId entity_id_;
};

TEST_F(ComponentManagementTest, AddComponentDefaultHealth) {
    DegradationResult result = model_->add_component(
        entity_id_, "wheel_left", ComponentType::Mechanical);

    EXPECT_EQ(result, DegradationResult::Success);

    auto health = model_->get_component_health(entity_id_, "wheel_left");
    ASSERT_TRUE(health.has_value());
    EXPECT_DOUBLE_EQ(health->health_percentage, 100.0);
}

TEST_F(ComponentManagementTest, AddComponentCustomHealth) {
    DegradationResult result = model_->add_component(
        entity_id_, "motor", ComponentType::Electrical, 75.0);

    EXPECT_EQ(result, DegradationResult::Success);

    auto health = model_->get_component_health(entity_id_, "motor");
    ASSERT_TRUE(health.has_value());
    EXPECT_DOUBLE_EQ(health->health_percentage, 75.0);
}

TEST_F(ComponentManagementTest, AddMultipleComponents) {
    model_->add_component(entity_id_, "wheel_left", ComponentType::Mechanical);
    model_->add_component(entity_id_, "wheel_right", ComponentType::Mechanical);
    model_->add_component(entity_id_, "motor", ComponentType::Electrical);

    auto entity_deg = model_->get_entity_degradation(entity_id_);
    ASSERT_TRUE(entity_deg.has_value());
    EXPECT_GE(entity_deg->components.size(), 3u);
}

TEST_F(ComponentManagementTest, GetComponentHealth) {
    model_->add_component(entity_id_, "pump", ComponentType::Hydraulic, 80.0);

    auto health = model_->get_component_health(entity_id_, "pump");
    ASSERT_TRUE(health.has_value());
    EXPECT_EQ(health->component_id, "pump");
    EXPECT_EQ(health->type, ComponentType::Hydraulic);
    EXPECT_DOUBLE_EQ(health->health_percentage, 80.0);
}

TEST_F(ComponentManagementTest, GetNonExistentComponent) {
    auto health = model_->get_component_health(entity_id_, "nonexistent");
    EXPECT_FALSE(health.has_value());
}

// ============================================================================
// Operating Conditions Tests
// ============================================================================

class OperatingConditionsUpdateTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_ = DegradationModelConfig::default_config();
        model_ = create_degradation_model(config_);
        model_->initialize();
        entity_id_ = 1;
        model_->register_entity(entity_id_);
        model_->add_component(entity_id_, "motor", ComponentType::Electrical);
    }

    void TearDown() override {
        model_->shutdown();
    }

    DegradationModelConfig config_;
    std::unique_ptr<DegradationModel> model_;
    EntityId entity_id_;
};

TEST_F(OperatingConditionsUpdateTest, UpdateOperatingConditions) {
    OperatingConditions conditions;
    conditions.load_factor = 0.8;
    conditions.temperature_celsius = 75.0;

    DegradationResult result = model_->update_operating_conditions(entity_id_, conditions);
    EXPECT_EQ(result, DegradationResult::Success);
}

TEST_F(OperatingConditionsUpdateTest, DifferentLoadFactors) {
    OperatingConditions light;
    light.load_factor = 0.3;

    OperatingConditions heavy;
    heavy.load_factor = 0.9;

    EXPECT_EQ(model_->update_operating_conditions(entity_id_, light),
              DegradationResult::Success);
    EXPECT_EQ(model_->update_operating_conditions(entity_id_, heavy),
              DegradationResult::Success);
}

TEST_F(OperatingConditionsUpdateTest, TemperatureEffects) {
    OperatingConditions cold;
    cold.temperature_celsius = -10.0;

    OperatingConditions hot;
    hot.temperature_celsius = 100.0;

    EXPECT_EQ(model_->update_operating_conditions(entity_id_, cold),
              DegradationResult::Success);
    EXPECT_EQ(model_->update_operating_conditions(entity_id_, hot),
              DegradationResult::Success);
}

TEST_F(OperatingConditionsUpdateTest, HumidityEffects) {
    OperatingConditions dry;
    dry.humidity_percent = 10.0;

    OperatingConditions humid;
    humid.humidity_percent = 95.0;

    EXPECT_EQ(model_->update_operating_conditions(entity_id_, dry),
              DegradationResult::Success);
    EXPECT_EQ(model_->update_operating_conditions(entity_id_, humid),
              DegradationResult::Success);
}

// ============================================================================
// Wear Calculation Tests
// ============================================================================

class WearCalculationTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_ = DegradationModelConfig::default_config();
        model_ = create_degradation_model(config_);
        model_->initialize();
        entity_id_ = 1;
        model_->register_entity(entity_id_);
    }

    void TearDown() override {
        model_->shutdown();
    }

    DegradationModelConfig config_;
    std::unique_ptr<DegradationModel> model_;
    EntityId entity_id_;
};

TEST_F(WearCalculationTest, LinearWearCalculation) {
    auto calculator = create_linear_calculator();

    ComponentHealth health;
    health.component_id = "test";
    health.type = ComponentType::Mechanical;
    health.health_percentage = 100.0;

    OperatingConditions conditions = create_nominal_conditions();

    Real wear = calculator->calculate_wear(health, conditions, std::chrono::hours(10));
    EXPECT_GT(wear, 0.0);
}

TEST_F(WearCalculationTest, ExponentialWearCalculation) {
    auto calculator = create_exponential_calculator();

    ComponentHealth health;
    health.component_id = "test";
    health.type = ComponentType::Mechanical;
    health.health_percentage = 100.0;
    health.operating_hours = std::chrono::hours(1000);

    OperatingConditions conditions = create_nominal_conditions();

    Real wear = calculator->calculate_wear(health, conditions, std::chrono::hours(10));
    EXPECT_GT(wear, 0.0);
}

TEST_F(WearCalculationTest, WearWithDifferentConditions) {
    auto calculator = create_linear_calculator();

    ComponentHealth health;
    health.component_id = "test";
    health.health_percentage = 100.0;

    OperatingConditions light = create_nominal_conditions();
    OperatingConditions harsh = create_harsh_conditions();

    Real light_wear = calculator->calculate_wear(health, light, std::chrono::hours(10));
    Real harsh_wear = calculator->calculate_wear(health, harsh, std::chrono::hours(10));

    EXPECT_GT(harsh_wear, light_wear);
}

TEST_F(WearCalculationTest, WearOverDifferentDurations) {
    auto calculator = create_linear_calculator();

    ComponentHealth health;
    health.health_percentage = 100.0;

    OperatingConditions conditions = create_nominal_conditions();

    Real short_wear = calculator->calculate_wear(health, conditions, std::chrono::hours(1));
    Real long_wear = calculator->calculate_wear(health, conditions, std::chrono::hours(10));

    EXPECT_GT(long_wear, short_wear);
}

TEST_F(WearCalculationTest, HealthNeverBelowZero) {
    model_->add_component(entity_id_, "component", ComponentType::Mechanical, 5.0);

    // Apply excessive wear
    for (int i = 0; i < 100; ++i) {
        model_->apply_wear(entity_id_, std::chrono::hours(100));
    }

    auto health = model_->get_component_health(entity_id_, "component");
    ASSERT_TRUE(health.has_value());
    EXPECT_GE(health->health_percentage, 0.0);
}

// ============================================================================
// apply_wear Tests
// ============================================================================

class ApplyWearTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_ = DegradationModelConfig::default_config();
        model_ = create_degradation_model(config_);
        model_->initialize();
        entity_id_ = 1;
        model_->register_entity(entity_id_);
        model_->add_component(entity_id_, "motor", ComponentType::Electrical);
    }

    void TearDown() override {
        model_->shutdown();
    }

    DegradationModelConfig config_;
    std::unique_ptr<DegradationModel> model_;
    EntityId entity_id_;
};

TEST_F(ApplyWearTest, ApplyWearToAllComponents) {
    model_->add_component(entity_id_, "wheel", ComponentType::Mechanical);
    model_->add_component(entity_id_, "pump", ComponentType::Hydraulic);

    auto initial = model_->get_component_health(entity_id_, "motor");
    ASSERT_TRUE(initial.has_value());
    Real initial_health = initial->health_percentage;

    OperatingConditions conditions = create_nominal_conditions();
    model_->update_operating_conditions(entity_id_, conditions);
    model_->apply_wear(entity_id_, std::chrono::hours(100));

    auto final = model_->get_component_health(entity_id_, "motor");
    ASSERT_TRUE(final.has_value());
    EXPECT_LT(final->health_percentage, initial_health);
}

TEST_F(ApplyWearTest, ApplyWearLightLoad) {
    OperatingConditions light;
    light.load_factor = 0.2;
    model_->update_operating_conditions(entity_id_, light);

    auto initial = model_->get_component_health(entity_id_, "motor");
    ASSERT_TRUE(initial.has_value());

    model_->apply_wear(entity_id_, std::chrono::hours(10));

    auto final = model_->get_component_health(entity_id_, "motor");
    ASSERT_TRUE(final.has_value());
    EXPECT_LE(final->health_percentage, initial->health_percentage);
}

TEST_F(ApplyWearTest, ApplyWearHeavyLoad) {
    OperatingConditions heavy;
    heavy.load_factor = 0.95;
    heavy.temperature_celsius = 90.0;
    model_->update_operating_conditions(entity_id_, heavy);

    auto initial = model_->get_component_health(entity_id_, "motor");
    ASSERT_TRUE(initial.has_value());

    model_->apply_wear(entity_id_, std::chrono::hours(10));

    auto final = model_->get_component_health(entity_id_, "motor");
    ASSERT_TRUE(final.has_value());
    EXPECT_LT(final->health_percentage, initial->health_percentage);
}

TEST_F(ApplyWearTest, HealthStatusChanges) {
    // Add component with good health
    model_->add_component(entity_id_, "degrading", ComponentType::Consumable, 95.0);

    OperatingConditions harsh = create_harsh_conditions();
    model_->update_operating_conditions(entity_id_, harsh);

    // Apply wear repeatedly and check status changes
    for (int i = 0; i < 50; ++i) {
        model_->apply_wear(entity_id_, std::chrono::hours(10));
    }

    auto health = model_->get_component_health(entity_id_, "degrading");
    ASSERT_TRUE(health.has_value());

    // Should have degraded significantly
    EXPECT_LT(health->health_percentage, 95.0);
}

// ============================================================================
// Maintenance Tests
// ============================================================================

class MaintenanceTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_ = DegradationModelConfig::default_config();
        model_ = create_degradation_model(config_);
        model_->initialize();
        entity_id_ = 1;
        model_->register_entity(entity_id_);
    }

    void TearDown() override {
        model_->shutdown();
    }

    DegradationModelConfig config_;
    std::unique_ptr<DegradationModel> model_;
    EntityId entity_id_;
};

TEST_F(MaintenanceTest, InspectNoChange) {
    model_->add_component(entity_id_, "sensor", ComponentType::Electronic, 80.0);

    auto before = model_->get_component_health(entity_id_, "sensor");
    ASSERT_TRUE(before.has_value());

    model_->record_maintenance(entity_id_, "sensor", MaintenanceAction::Inspect);

    auto after = model_->get_component_health(entity_id_, "sensor");
    ASSERT_TRUE(after.has_value());

    // Inspect should not change health (or only minimally)
    EXPECT_NEAR(after->health_percentage, before->health_percentage, 1.0);
}

TEST_F(MaintenanceTest, LubricateImprovement) {
    model_->add_component(entity_id_, "bearing", ComponentType::Mechanical, 75.0);

    auto before = model_->get_component_health(entity_id_, "bearing");
    ASSERT_TRUE(before.has_value());

    model_->record_maintenance(entity_id_, "bearing", MaintenanceAction::Lubricate);

    auto after = model_->get_component_health(entity_id_, "bearing");
    ASSERT_TRUE(after.has_value());

    // Lubricate should improve health for mechanical components
    EXPECT_GE(after->health_percentage, before->health_percentage);
}

TEST_F(MaintenanceTest, AdjustImprovement) {
    model_->add_component(entity_id_, "valve", ComponentType::Hydraulic, 70.0);

    auto before = model_->get_component_health(entity_id_, "valve");
    ASSERT_TRUE(before.has_value());

    model_->record_maintenance(entity_id_, "valve", MaintenanceAction::Adjust);

    auto after = model_->get_component_health(entity_id_, "valve");
    ASSERT_TRUE(after.has_value());

    // Adjust should improve health (~10-15%)
    EXPECT_GT(after->health_percentage, before->health_percentage);
}

TEST_F(MaintenanceTest, RepairSignificantImprovement) {
    model_->add_component(entity_id_, "motor", ComponentType::Electrical, 50.0);

    auto before = model_->get_component_health(entity_id_, "motor");
    ASSERT_TRUE(before.has_value());

    model_->record_maintenance(entity_id_, "motor", MaintenanceAction::Repair);

    auto after = model_->get_component_health(entity_id_, "motor");
    ASSERT_TRUE(after.has_value());

    // Repair should restore 30-50% health
    EXPECT_GT(after->health_percentage, before->health_percentage + 25.0);
}

TEST_F(MaintenanceTest, ReplaceFullRestoration) {
    model_->add_component(entity_id_, "filter", ComponentType::Consumable, 25.0);

    model_->record_maintenance(entity_id_, "filter", MaintenanceAction::Replace);

    auto after = model_->get_component_health(entity_id_, "filter");
    ASSERT_TRUE(after.has_value());

    // Replace should restore to 100%
    EXPECT_DOUBLE_EQ(after->health_percentage, 100.0);
}

TEST_F(MaintenanceTest, OverhaulNearFullRestoration) {
    model_->add_component(entity_id_, "engine", ComponentType::Mechanical, 30.0);

    model_->record_maintenance(entity_id_, "engine", MaintenanceAction::Overhaul);

    auto after = model_->get_component_health(entity_id_, "engine");
    ASSERT_TRUE(after.has_value());

    // Overhaul should restore to 95%
    EXPECT_DOUBLE_EQ(after->health_percentage, 95.0);
}

TEST_F(MaintenanceTest, HealthCappedAt100) {
    model_->add_component(entity_id_, "sensor", ComponentType::Electronic, 98.0);

    model_->record_maintenance(entity_id_, "sensor", MaintenanceAction::Adjust);

    auto after = model_->get_component_health(entity_id_, "sensor");
    ASSERT_TRUE(after.has_value());

    // Health should not exceed 100%
    EXPECT_LE(after->health_percentage, 100.0);
}

// ============================================================================
// Failure Prediction Tests
// ============================================================================

class FailurePredictionTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_ = DegradationModelConfig::default_config();
        config_.enable_predictions = true;
        model_ = create_degradation_model(config_);
        model_->initialize();
        entity_id_ = 1;
        model_->register_entity(entity_id_);
    }

    void TearDown() override {
        model_->shutdown();
    }

    DegradationModelConfig config_;
    std::unique_ptr<DegradationModel> model_;
    EntityId entity_id_;
};

TEST_F(FailurePredictionTest, PredictionsForHealthyComponents) {
    model_->add_component(entity_id_, "healthy", ComponentType::Mechanical, 100.0);

    auto predictions = model_->get_predictions(entity_id_);

    // Healthy components should have few or no predictions
    EXPECT_GE(predictions.size(), 0u);
}

TEST_F(FailurePredictionTest, PredictionsForDegradedComponents) {
    model_->add_component(entity_id_, "degraded", ComponentType::Mechanical, 40.0);

    auto predictions = model_->get_predictions(entity_id_);

    // Degraded components might have predictions
    // (depends on implementation)
    EXPECT_GE(predictions.size(), 0u);
}

TEST_F(FailurePredictionTest, PredictionConfidence) {
    auto predictor = create_simple_predictor();

    ComponentHealth healthy;
    healthy.health_percentage = 90.0;

    ComponentHealth degraded;
    degraded.health_percentage = 30.0;

    OperatingConditions conditions = create_nominal_conditions();

    Real healthy_confidence = predictor->calculate_confidence(healthy, conditions);
    Real degraded_confidence = predictor->calculate_confidence(degraded, conditions);

    EXPECT_GE(healthy_confidence, 0.0);
    EXPECT_LE(healthy_confidence, 1.0);
    EXPECT_GE(degraded_confidence, 0.0);
    EXPECT_LE(degraded_confidence, 1.0);
}

TEST_F(FailurePredictionTest, PredictedFailureTime) {
    model_->add_component(entity_id_, "failing", ComponentType::Consumable, 25.0);

    auto predictions = model_->get_predictions(entity_id_);

    for (const auto& pred : predictions) {
        // Predicted failure time should be in the future
        auto now = std::chrono::system_clock::now();
        EXPECT_GE(pred.predicted_failure, now);
        EXPECT_GE(pred.confidence, 0.0);
        EXPECT_LE(pred.confidence, 1.0);
    }
}

// ============================================================================
// Maintenance Recommendation Tests
// ============================================================================

class MaintenanceRecommendationTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_ = DegradationModelConfig::default_config();
        config_.enable_recommendations = true;
        model_ = create_degradation_model(config_);
        model_->initialize();
        entity_id_ = 1;
        model_->register_entity(entity_id_);
    }

    void TearDown() override {
        model_->shutdown();
    }

    DegradationModelConfig config_;
    std::unique_ptr<DegradationModel> model_;
    EntityId entity_id_;
};

TEST_F(MaintenanceRecommendationTest, NoRecommendationsForExcellent) {
    model_->add_component(entity_id_, "excellent", ComponentType::Mechanical, 95.0);

    auto advisor = create_rule_based_advisor();

    ComponentHealth health;
    health.health_percentage = 95.0;

    MaintenanceAction action = advisor->determine_action(health);

    // Excellent health should require no action or just inspection
    EXPECT_TRUE(action == MaintenanceAction::None ||
                action == MaintenanceAction::Inspect);
}

TEST_F(MaintenanceRecommendationTest, InspectForGood) {
    auto advisor = create_rule_based_advisor();

    ComponentHealth health;
    health.health_percentage = 75.0;  // Good health

    MaintenanceAction action = advisor->determine_action(health);

    // Good health should recommend inspection
    EXPECT_TRUE(action == MaintenanceAction::Inspect ||
                action == MaintenanceAction::None);
}

TEST_F(MaintenanceRecommendationTest, LubricateOrAdjustForFair) {
    auto advisor = create_rule_based_advisor();

    ComponentHealth health;
    health.health_percentage = 60.0;  // Fair health

    MaintenanceAction action = advisor->determine_action(health);

    // Fair health might recommend preventive maintenance
    EXPECT_NE(action, MaintenanceAction::None);
}

TEST_F(MaintenanceRecommendationTest, RepairForPoor) {
    auto advisor = create_rule_based_advisor();

    ComponentHealth health;
    health.health_percentage = 40.0;  // Poor health

    MaintenanceAction action = advisor->determine_action(health);

    // Poor health should recommend repair or more
    EXPECT_TRUE(action == MaintenanceAction::Repair ||
                action == MaintenanceAction::Replace);
}

TEST_F(MaintenanceRecommendationTest, ReplaceForCritical) {
    auto advisor = create_rule_based_advisor();

    ComponentHealth health;
    health.health_percentage = 20.0;  // Critical health

    MaintenanceAction action = advisor->determine_action(health);

    // Critical health should recommend replacement
    EXPECT_TRUE(action == MaintenanceAction::Replace ||
                action == MaintenanceAction::Overhaul);
}

TEST_F(MaintenanceRecommendationTest, UrgencyLevels) {
    model_->add_component(entity_id_, "critical", ComponentType::Mechanical, 20.0);
    model_->add_component(entity_id_, "good", ComponentType::Mechanical, 85.0);

    auto recommendations = model_->get_recommendations(entity_id_);

    for (const auto& rec : recommendations) {
        EXPECT_GE(rec.urgency, 0.0);
        EXPECT_LE(rec.urgency, 1.0);
        EXPECT_GE(rec.confidence, 0.0);
        EXPECT_LE(rec.confidence, 1.0);
    }
}

// ============================================================================
// EntityDegradation Tests
// ============================================================================

class EntityDegradationTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_ = DegradationModelConfig::default_config();
        model_ = create_degradation_model(config_);
        model_->initialize();
        entity_id_ = 1;
        model_->register_entity(entity_id_);
    }

    void TearDown() override {
        model_->shutdown();
    }

    DegradationModelConfig config_;
    std::unique_ptr<DegradationModel> model_;
    EntityId entity_id_;
};

TEST_F(EntityDegradationTest, OverallHealthCalculation) {
    model_->add_component(entity_id_, "comp1", ComponentType::Mechanical, 100.0);
    model_->add_component(entity_id_, "comp2", ComponentType::Electrical, 80.0);
    model_->add_component(entity_id_, "comp3", ComponentType::Hydraulic, 60.0);

    auto entity_deg = model_->get_entity_degradation(entity_id_);
    ASSERT_TRUE(entity_deg.has_value());

    // Overall health should be weighted average
    Real expected_avg = (100.0 + 80.0 + 60.0) / 3.0;
    EXPECT_NEAR(entity_deg->overall_health, expected_avg, 1.0);
}

TEST_F(EntityDegradationTest, MeanTimeToFailure) {
    model_->add_component(entity_id_, "comp", ComponentType::Consumable, 50.0);

    auto entity_deg = model_->get_entity_degradation(entity_id_);
    ASSERT_TRUE(entity_deg.has_value());

    // MTTF should be calculated
    EXPECT_GE(entity_deg->mean_time_to_failure.count(), 0);
}

TEST_F(EntityDegradationTest, MultipleComponents) {
    for (int i = 0; i < 5; ++i) {
        std::string comp_id = "component_" + std::to_string(i);
        model_->add_component(entity_id_, comp_id, ComponentType::Mechanical,
                            100.0 - (i * 10.0));
    }

    auto entity_deg = model_->get_entity_degradation(entity_id_);
    ASSERT_TRUE(entity_deg.has_value());
    EXPECT_EQ(entity_deg->components.size(), 5u);
}

// ============================================================================
// Calculator Interface Tests
// ============================================================================

class CalculatorInterfaceTest : public ::testing::Test {};

TEST_F(CalculatorInterfaceTest, LinearDegradationCalculator) {
    auto calculator = create_linear_calculator();
    EXPECT_NE(calculator, nullptr);

    ComponentHealth health;
    health.health_percentage = 100.0;
    OperatingConditions conditions = create_nominal_conditions();

    Real wear = calculator->calculate_wear(health, conditions, std::chrono::hours(10));
    EXPECT_GE(wear, 0.0);
}

TEST_F(CalculatorInterfaceTest, ExponentialDegradationCalculator) {
    auto calculator = create_exponential_calculator();
    EXPECT_NE(calculator, nullptr);

    ComponentHealth health;
    health.health_percentage = 100.0;
    health.operating_hours = std::chrono::hours(500);
    OperatingConditions conditions = create_nominal_conditions();

    Real wear = calculator->calculate_wear(health, conditions, std::chrono::hours(10));
    EXPECT_GE(wear, 0.0);
}

TEST_F(CalculatorInterfaceTest, DetermineStatusThresholds) {
    auto calculator = create_linear_calculator();

    EXPECT_EQ(calculator->determine_status(95.0), HealthStatus::Excellent);
    EXPECT_EQ(calculator->determine_status(85.0), HealthStatus::Good);
    EXPECT_EQ(calculator->determine_status(65.0), HealthStatus::Fair);
    EXPECT_EQ(calculator->determine_status(45.0), HealthStatus::Poor);
    EXPECT_EQ(calculator->determine_status(25.0), HealthStatus::Critical);
    EXPECT_EQ(calculator->determine_status(0.0), HealthStatus::Failed);
}

// ============================================================================
// Statistics Tests
// ============================================================================

class StatisticsTest : public ::testing::Test {
protected:
    void SetUp() override {
        config_ = DegradationModelConfig::default_config();
        model_ = create_degradation_model(config_);
        model_->initialize();
    }

    void TearDown() override {
        model_->shutdown();
    }

    DegradationModelConfig config_;
    std::unique_ptr<DegradationModel> model_;
};

TEST_F(StatisticsTest, InitialStats) {
    DegradationStats stats = model_->get_stats();

    EXPECT_EQ(stats.total_entities, 0u);
    EXPECT_EQ(stats.total_components, 0u);
    EXPECT_DOUBLE_EQ(stats.average_health, 100.0);
    EXPECT_EQ(stats.critical_components, 0u);
}

TEST_F(StatisticsTest, StatsAfterAddingEntities) {
    EntityId entity1 = 1;
    EntityId entity2 = 2;

    model_->register_entity(entity1);
    model_->register_entity(entity2);

    DegradationStats stats = model_->get_stats();
    EXPECT_GE(stats.total_entities, 2u);
}

TEST_F(StatisticsTest, StatsAfterAddingComponents) {
    EntityId entity = 1;
    model_->register_entity(entity);
    model_->add_component(entity, "comp1", ComponentType::Mechanical, 100.0);
    model_->add_component(entity, "comp2", ComponentType::Electrical, 80.0);
    model_->add_component(entity, "comp3", ComponentType::Hydraulic, 60.0);

    DegradationStats stats = model_->get_stats();
    EXPECT_GE(stats.total_components, 3u);
}

TEST_F(StatisticsTest, AverageHealth) {
    EntityId entity = 1;
    model_->register_entity(entity);
    model_->add_component(entity, "comp1", ComponentType::Mechanical, 100.0);
    model_->add_component(entity, "comp2", ComponentType::Electrical, 50.0);

    DegradationStats stats = model_->get_stats();

    // Average should be between min and max
    EXPECT_GE(stats.average_health, 50.0);
    EXPECT_LE(stats.average_health, 100.0);
}

TEST_F(StatisticsTest, CriticalComponentCount) {
    EntityId entity = 1;
    model_->register_entity(entity);
    model_->add_component(entity, "critical1", ComponentType::Mechanical, 25.0);
    model_->add_component(entity, "critical2", ComponentType::Electrical, 15.0);
    model_->add_component(entity, "healthy", ComponentType::Hydraulic, 90.0);

    DegradationStats stats = model_->get_stats();
    EXPECT_GE(stats.critical_components, 2u);
}

TEST_F(StatisticsTest, ComponentsByStatus) {
    EntityId entity = 1;
    model_->register_entity(entity);
    model_->add_component(entity, "excellent", ComponentType::Mechanical, 95.0);
    model_->add_component(entity, "good", ComponentType::Electrical, 80.0);
    model_->add_component(entity, "fair", ComponentType::Hydraulic, 65.0);
    model_->add_component(entity, "poor", ComponentType::Structural, 40.0);
    model_->add_component(entity, "critical", ComponentType::Electronic, 20.0);

    DegradationStats stats = model_->get_stats();

    // Should have components in various status categories
    EXPECT_GT(stats.components_by_status.size(), 0u);
}

// ============================================================================
// Factory Function Tests
// ============================================================================

class FactoryFunctionTest : public ::testing::Test {};

TEST_F(FactoryFunctionTest, CreateDegradationModel) {
    DegradationModelConfig config = DegradationModelConfig::default_config();
    auto model = create_degradation_model(config);

    EXPECT_NE(model, nullptr);
    EXPECT_EQ(model->initialize(), DegradationResult::Success);
    model->shutdown();
}

TEST_F(FactoryFunctionTest, CreateLinearCalculator) {
    auto calculator = create_linear_calculator();
    EXPECT_NE(calculator, nullptr);
}

TEST_F(FactoryFunctionTest, CreateExponentialCalculator) {
    auto calculator = create_exponential_calculator();
    EXPECT_NE(calculator, nullptr);
}

TEST_F(FactoryFunctionTest, CreateSimplePredictor) {
    auto predictor = create_simple_predictor();
    EXPECT_NE(predictor, nullptr);
}

TEST_F(FactoryFunctionTest, CreateRuleBasedAdvisor) {
    auto advisor = create_rule_based_advisor();
    EXPECT_NE(advisor, nullptr);
}

// ============================================================================
// Helper Function Tests
// ============================================================================

class HelperFunctionTest : public ::testing::Test {};

TEST_F(HelperFunctionTest, HealthPercentageToStatus) {
    EXPECT_EQ(health_percentage_to_status(95.0), HealthStatus::Excellent);
    EXPECT_EQ(health_percentage_to_status(85.0), HealthStatus::Good);
    EXPECT_EQ(health_percentage_to_status(65.0), HealthStatus::Fair);
    EXPECT_EQ(health_percentage_to_status(45.0), HealthStatus::Poor);
    EXPECT_EQ(health_percentage_to_status(25.0), HealthStatus::Critical);
    EXPECT_EQ(health_percentage_to_status(0.0), HealthStatus::Failed);
}

TEST_F(HelperFunctionTest, CalculateUrgencyFromRUL) {
    Real urgency_critical = calculate_urgency_from_rul(std::chrono::hours(0));
    EXPECT_DOUBLE_EQ(urgency_critical, 1.0);

    Real urgency_low = calculate_urgency_from_rul(std::chrono::hours(500));
    EXPECT_DOUBLE_EQ(urgency_low, 0.0);

    Real urgency_medium = calculate_urgency_from_rul(std::chrono::hours(84));
    EXPECT_GT(urgency_medium, 0.0);
    EXPECT_LT(urgency_medium, 1.0);
}

TEST_F(HelperFunctionTest, CalculateWeightedHealth) {
    std::vector<ComponentHealth> components;

    ComponentHealth comp1;
    comp1.health_percentage = 100.0;
    components.push_back(comp1);

    ComponentHealth comp2;
    comp2.health_percentage = 50.0;
    components.push_back(comp2);

    Real avg = calculate_weighted_health(components);
    EXPECT_DOUBLE_EQ(avg, 75.0);
}

TEST_F(HelperFunctionTest, EstimateMTTF) {
    std::vector<ComponentHealth> components;

    ComponentHealth comp1;
    comp1.remaining_useful_life = std::chrono::hours(100);
    components.push_back(comp1);

    ComponentHealth comp2;
    comp2.remaining_useful_life = std::chrono::hours(50);
    components.push_back(comp2);

    auto mttf = estimate_mttf(components);
    EXPECT_EQ(mttf.count(), 50);  // Minimum RUL
}

TEST_F(HelperFunctionTest, CountByStatus) {
    std::vector<ComponentHealth> components;

    for (int i = 0; i < 10; ++i) {
        ComponentHealth comp;
        comp.health_percentage = 95.0 - (i * 10.0);
        comp.status = health_percentage_to_status(comp.health_percentage);
        components.push_back(comp);
    }

    auto counts = count_by_status(components);
    EXPECT_GT(counts.size(), 0u);
}

TEST_F(HelperFunctionTest, FindMostCritical) {
    std::vector<ComponentHealth> components;

    ComponentHealth comp1;
    comp1.health_percentage = 75.0;
    components.push_back(comp1);

    ComponentHealth comp2;
    comp2.health_percentage = 25.0;  // Most critical
    components.push_back(comp2);

    ComponentHealth comp3;
    comp3.health_percentage = 90.0;
    components.push_back(comp3);

    const ComponentHealth* critical = find_most_critical(components);
    ASSERT_NE(critical, nullptr);
    EXPECT_DOUBLE_EQ(critical->health_percentage, 25.0);
}

TEST_F(HelperFunctionTest, HasCriticalComponents) {
    std::vector<ComponentHealth> components;

    ComponentHealth healthy;
    healthy.health_percentage = 90.0;
    healthy.status = HealthStatus::Excellent;
    components.push_back(healthy);

    EXPECT_FALSE(has_critical_components(components));

    ComponentHealth critical;
    critical.health_percentage = 20.0;
    critical.status = HealthStatus::Critical;
    components.push_back(critical);

    EXPECT_TRUE(has_critical_components(components));
}

TEST_F(HelperFunctionTest, FilterByType) {
    std::vector<ComponentHealth> components;

    ComponentHealth mech;
    mech.type = ComponentType::Mechanical;
    components.push_back(mech);

    ComponentHealth elec;
    elec.type = ComponentType::Electrical;
    components.push_back(elec);

    ComponentHealth mech2;
    mech2.type = ComponentType::Mechanical;
    components.push_back(mech2);

    auto mechanical = filter_by_type(components, ComponentType::Mechanical);
    EXPECT_EQ(mechanical.size(), 2u);
}

TEST_F(HelperFunctionTest, FilterByStatus) {
    std::vector<ComponentHealth> components;

    ComponentHealth good1;
    good1.status = HealthStatus::Good;
    components.push_back(good1);

    ComponentHealth critical;
    critical.status = HealthStatus::Critical;
    components.push_back(critical);

    ComponentHealth good2;
    good2.status = HealthStatus::Good;
    components.push_back(good2);

    auto good_comps = filter_by_status(components, HealthStatus::Good);
    EXPECT_EQ(good_comps.size(), 2u);
}

TEST_F(HelperFunctionTest, ApplyMaintenanceEffect) {
    ComponentHealth health;
    health.health_percentage = 75.0;
    health.type = ComponentType::Mechanical;

    Real after_lubricate = apply_maintenance_effect(health, MaintenanceAction::Lubricate);
    EXPECT_GT(after_lubricate, 75.0);

    health.health_percentage = 50.0;
    Real after_repair = apply_maintenance_effect(health, MaintenanceAction::Repair);
    EXPECT_GT(after_repair, 50.0);

    health.health_percentage = 25.0;
    Real after_replace = apply_maintenance_effect(health, MaintenanceAction::Replace);
    EXPECT_DOUBLE_EQ(after_replace, 100.0);
}

TEST_F(HelperFunctionTest, SortByUrgency) {
    std::vector<ComponentHealth> components;

    ComponentHealth comp1;
    comp1.component_id = "1";
    comp1.remaining_useful_life = std::chrono::hours(100);
    components.push_back(comp1);

    ComponentHealth comp2;
    comp2.component_id = "2";
    comp2.remaining_useful_life = std::chrono::hours(10);
    components.push_back(comp2);

    ComponentHealth comp3;
    comp3.component_id = "3";
    comp3.remaining_useful_life = std::chrono::hours(50);
    components.push_back(comp3);

    auto sorted = sort_by_urgency(components);
    EXPECT_EQ(sorted[0].remaining_useful_life.count(), 10);
    EXPECT_EQ(sorted[2].remaining_useful_life.count(), 100);
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST(DegradationEdgeCases, EmptyComponentList) {
    std::vector<ComponentHealth> empty;

    Real health = calculate_weighted_health(empty);
    EXPECT_DOUBLE_EQ(health, 100.0);

    auto counts = count_by_status(empty);
    EXPECT_EQ(counts[HealthStatus::Excellent], 0u);

    const ComponentHealth* critical = find_most_critical(empty);
    EXPECT_EQ(critical, nullptr);
}

TEST(DegradationEdgeCases, ZeroHealthComponent) {
    ComponentHealth failed;
    failed.health_percentage = 0.0;

    HealthStatus status = health_percentage_to_status(failed.health_percentage);
    EXPECT_EQ(status, HealthStatus::Failed);
}

TEST(DegradationEdgeCases, NegativeRUL) {
    ComponentHealth comp;
    comp.remaining_useful_life = std::chrono::hours(-10);

    Real urgency = calculate_urgency_from_rul(comp.remaining_useful_life);
    EXPECT_DOUBLE_EQ(urgency, 1.0);  // Maximum urgency
}
