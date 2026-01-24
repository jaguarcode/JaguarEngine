#pragma once
/**
 * @file degradation_model.h
 * @brief Physics-based degradation modeling for predictive maintenance
 *
 * This file provides degradation modeling infrastructure for tracking component
 * health, predicting failures, and recommending maintenance actions. Implements
 * wear calculations, life estimation, and maintenance advisory systems.
 *
 * Key features:
 * - Physics-based wear models (mechanical, thermal, chemical, etc.)
 * - Health status tracking and remaining useful life estimation
 * - Failure prediction with confidence intervals
 * - Rule-based and data-driven maintenance recommendations
 * - Operating condition-aware degradation calculations
 * - Multi-component degradation aggregation
 */

#include "jaguar/core/types.h"
#include <vector>
#include <unordered_map>
#include <memory>
#include <optional>
#include <chrono>
#include <string>
#include <algorithm>
#include <cmath>

namespace jaguar::thread {

// ============================================================================
// Forward Declarations
// ============================================================================

class IDegradationCalculator;
class IFailurePredictor;
class IMaintenanceAdvisor;
class DegradationModel;

// ============================================================================
// Degradation Result Enum
// ============================================================================

/**
 * @brief Result codes for degradation modeling operations
 */
enum class DegradationResult : UInt8 {
    Success = 0,

    // Configuration errors
    InvalidConfiguration,
    InvalidEntityId,
    InvalidComponent,
    InvalidModel,

    // Operational errors
    CalculationFailed,
    PredictionFailed,
    UpdateFailed,

    // State errors
    NotInitialized,
    AlreadyInitialized,
    ComponentNotFound,

    // Resource errors
    OutOfMemory,
    ModelNotLoaded
};

/**
 * @brief Convert DegradationResult to string
 */
inline const char* degradation_result_to_string(DegradationResult result) {
    switch (result) {
        case DegradationResult::Success: return "Success";
        case DegradationResult::InvalidConfiguration: return "InvalidConfiguration";
        case DegradationResult::InvalidEntityId: return "InvalidEntityId";
        case DegradationResult::InvalidComponent: return "InvalidComponent";
        case DegradationResult::InvalidModel: return "InvalidModel";
        case DegradationResult::CalculationFailed: return "CalculationFailed";
        case DegradationResult::PredictionFailed: return "PredictionFailed";
        case DegradationResult::UpdateFailed: return "UpdateFailed";
        case DegradationResult::NotInitialized: return "NotInitialized";
        case DegradationResult::AlreadyInitialized: return "AlreadyInitialized";
        case DegradationResult::ComponentNotFound: return "ComponentNotFound";
        case DegradationResult::OutOfMemory: return "OutOfMemory";
        case DegradationResult::ModelNotLoaded: return "ModelNotLoaded";
        default: return "Unknown";
    }
}

// ============================================================================
// Degradation Type Enum
// ============================================================================

/**
 * @brief Types of degradation mechanisms
 */
enum class DegradationType : UInt8 {
    Wear,        ///< Mechanical wear from friction
    Fatigue,     ///< Stress cycle fatigue
    Corrosion,   ///< Chemical/environmental degradation
    Thermal,     ///< Heat-related degradation
    Chemical,    ///< Chemical reaction degradation
    Electrical,  ///< Electrical component degradation
    Combined     ///< Multiple degradation types
};

/**
 * @brief Convert DegradationType to string
 */
inline const char* degradation_type_to_string(DegradationType type) {
    switch (type) {
        case DegradationType::Wear: return "Wear";
        case DegradationType::Fatigue: return "Fatigue";
        case DegradationType::Corrosion: return "Corrosion";
        case DegradationType::Thermal: return "Thermal";
        case DegradationType::Chemical: return "Chemical";
        case DegradationType::Electrical: return "Electrical";
        case DegradationType::Combined: return "Combined";
        default: return "Unknown";
    }
}

// ============================================================================
// Component Type Enum
// ============================================================================

/**
 * @brief Types of components that can degrade
 */
enum class ComponentType : UInt8 {
    Mechanical,   ///< Gears, bearings, joints
    Electrical,   ///< Motors, sensors, wiring
    Hydraulic,    ///< Pumps, valves, lines
    Structural,   ///< Frame, body, supports
    Electronic,   ///< Controllers, processors
    Consumable    ///< Filters, fluids, seals
};

/**
 * @brief Convert ComponentType to string
 */
inline const char* component_type_to_string(ComponentType type) {
    switch (type) {
        case ComponentType::Mechanical: return "Mechanical";
        case ComponentType::Electrical: return "Electrical";
        case ComponentType::Hydraulic: return "Hydraulic";
        case ComponentType::Structural: return "Structural";
        case ComponentType::Electronic: return "Electronic";
        case ComponentType::Consumable: return "Consumable";
        default: return "Unknown";
    }
}

// ============================================================================
// Health Status Enum
// ============================================================================

/**
 * @brief Health status categories based on percentage
 */
enum class HealthStatus : UInt8 {
    Excellent,   ///< >90%
    Good,        ///< 70-90%
    Fair,        ///< 50-70%
    Poor,        ///< 30-50%
    Critical,    ///< <30%
    Failed       ///< 0%
};

/**
 * @brief Convert HealthStatus to string
 */
inline const char* health_status_to_string(HealthStatus status) {
    switch (status) {
        case HealthStatus::Excellent: return "Excellent";
        case HealthStatus::Good: return "Good";
        case HealthStatus::Fair: return "Fair";
        case HealthStatus::Poor: return "Poor";
        case HealthStatus::Critical: return "Critical";
        case HealthStatus::Failed: return "Failed";
        default: return "Unknown";
    }
}

// ============================================================================
// Maintenance Action Enum
// ============================================================================

/**
 * @brief Recommended maintenance actions
 */
enum class MaintenanceAction : UInt8 {
    None,        ///< No action needed
    Inspect,     ///< Visual or instrumented inspection
    Lubricate,   ///< Apply lubricant
    Adjust,      ///< Adjustment/calibration
    Repair,      ///< Repair component
    Replace,     ///< Replace component
    Overhaul     ///< Complete overhaul
};

/**
 * @brief Convert MaintenanceAction to string
 */
inline const char* maintenance_action_to_string(MaintenanceAction action) {
    switch (action) {
        case MaintenanceAction::None: return "None";
        case MaintenanceAction::Inspect: return "Inspect";
        case MaintenanceAction::Lubricate: return "Lubricate";
        case MaintenanceAction::Adjust: return "Adjust";
        case MaintenanceAction::Repair: return "Repair";
        case MaintenanceAction::Replace: return "Replace";
        case MaintenanceAction::Overhaul: return "Overhaul";
        default: return "Unknown";
    }
}

// ============================================================================
// Structs
// ============================================================================

/**
 * @brief Factors affecting wear rate
 */
struct WearFactor {
    Real base_rate{0.001};             ///< Degradation per hour (baseline)
    Real load_multiplier{1.0};         ///< Higher load = faster wear
    Real temperature_coefficient{0.01}; ///< Temp sensitivity (per degree C)
    Real humidity_coefficient{0.0};    ///< Humidity sensitivity (per %)
    Real age_acceleration{0.0};        ///< Older = faster degradation
};

/**
 * @brief Health information for a single component
 */
struct ComponentHealth {
    std::string component_id;                           ///< Unique component identifier
    ComponentType type{ComponentType::Mechanical};      ///< Component type
    Real health_percentage{100.0};                      ///< Health (0-100)
    Real degradation_rate{0.0};                         ///< Current degradation rate (per hour)
    HealthStatus status{HealthStatus::Excellent};       ///< Categorical health status
    std::chrono::system_clock::time_point last_updated; ///< Last assessment time
    std::chrono::hours operating_hours{0};              ///< Total operating time
    std::chrono::hours remaining_useful_life{0};        ///< Estimated RUL
};

/**
 * @brief Operating conditions affecting degradation
 */
struct OperatingConditions {
    Real load_factor{0.0};              ///< Load factor (0-1, 1 = max load)
    Real temperature_celsius{20.0};     ///< Operating temperature
    Real humidity_percent{50.0};        ///< Relative humidity
    Real vibration_level{0.0};          ///< Vibration magnitude
    Real contamination_level{0.0};      ///< Environmental contamination (0-1)
    std::chrono::hours continuous_operation{0}; ///< Hours of continuous operation
};

/**
 * @brief Maintenance recommendation for a component
 */
struct MaintenanceRecommendation {
    std::string component_id;           ///< Component requiring maintenance
    MaintenanceAction action{MaintenanceAction::None}; ///< Recommended action
    std::string description;            ///< Human-readable description
    Real urgency{0.0};                  ///< Urgency (0-1, 1 = immediate)
    std::chrono::hours recommended_within{0}; ///< Time window for action
    Real estimated_cost{0.0};           ///< Estimated cost (arbitrary units)
    Real confidence{1.0};               ///< Confidence in recommendation (0-1)
};

/**
 * @brief Predicted failure event
 */
struct FailurePrediction {
    std::string component_id;                           ///< Component expected to fail
    std::chrono::system_clock::time_point predicted_failure; ///< Predicted failure time
    Real confidence{0.0};                               ///< Confidence in prediction (0-1)
    std::string failure_mode;                           ///< Expected failure mode
    std::vector<MaintenanceRecommendation> preventive_actions; ///< Preventive options
};

/**
 * @brief Degradation state for an entire entity
 */
struct EntityDegradation {
    EntityId entity_id{INVALID_ENTITY_ID};              ///< Entity identifier
    Real overall_health{100.0};                         ///< Weighted average health
    std::vector<ComponentHealth> components;            ///< All component states
    std::vector<FailurePrediction> predictions;         ///< Failure predictions
    std::chrono::hours mean_time_to_failure{0};         ///< MTTF estimate
    std::chrono::system_clock::time_point last_assessment; ///< Last assessment time
};

/**
 * @brief Degradation model statistics
 */
struct DegradationStats {
    UInt64 total_entities{0};                           ///< Entities tracked
    UInt64 total_components{0};                         ///< Components tracked
    Real average_health{100.0};                         ///< Average health across all
    UInt64 critical_components{0};                      ///< Components in critical state
    UInt64 pending_recommendations{0};                  ///< Outstanding recommendations
    std::unordered_map<HealthStatus, UInt64> components_by_status; ///< Status histogram
};

/**
 * @brief Configuration for degradation modeling
 */
struct DegradationModelConfig {
    std::chrono::minutes assessment_interval{60};       ///< How often to assess (default 60 min)
    Real prediction_horizon_hours{720.0};               ///< Prediction window (default 30 days)
    Real minimum_confidence{0.8};                       ///< Minimum prediction confidence
    bool enable_predictions{true};                      ///< Enable failure predictions
    bool enable_recommendations{true};                  ///< Enable maintenance recommendations
    std::unordered_map<ComponentType, WearFactor> default_wear_factors; ///< Default wear parameters

    /**
     * @brief Create default configuration
     */
    static DegradationModelConfig default_config() noexcept {
        DegradationModelConfig config;
        // Populate default wear factors for each component type
        config.default_wear_factors[ComponentType::Mechanical] = WearFactor{
            0.001,  // base_rate
            1.5,    // load_multiplier
            0.01,   // temperature_coefficient
            0.001,  // humidity_coefficient
            0.0001  // age_acceleration
        };
        config.default_wear_factors[ComponentType::Electrical] = WearFactor{
            0.0005, 1.0, 0.02, 0.0, 0.0002
        };
        config.default_wear_factors[ComponentType::Hydraulic] = WearFactor{
            0.002, 2.0, 0.015, 0.002, 0.0001
        };
        config.default_wear_factors[ComponentType::Structural] = WearFactor{
            0.0001, 1.2, 0.005, 0.0005, 0.00005
        };
        config.default_wear_factors[ComponentType::Electronic] = WearFactor{
            0.0003, 0.8, 0.025, 0.0, 0.0003
        };
        config.default_wear_factors[ComponentType::Consumable] = WearFactor{
            0.01, 1.0, 0.005, 0.001, 0.0
        };
        return config;
    }

    /**
     * @brief Create configuration for high-frequency assessment
     */
    static DegradationModelConfig high_frequency() noexcept {
        DegradationModelConfig config = default_config();
        config.assessment_interval = std::chrono::minutes(15);
        config.prediction_horizon_hours = 168.0;  // 7 days
        return config;
    }

    /**
     * @brief Create configuration for long-term tracking
     */
    static DegradationModelConfig long_term() noexcept {
        DegradationModelConfig config = default_config();
        config.assessment_interval = std::chrono::hours(24);
        config.prediction_horizon_hours = 8760.0;  // 1 year
        config.minimum_confidence = 0.7;
        return config;
    }

    /**
     * @brief Create configuration for critical systems
     */
    static DegradationModelConfig critical_systems() noexcept {
        DegradationModelConfig config = default_config();
        config.assessment_interval = std::chrono::minutes(5);
        config.prediction_horizon_hours = 168.0;  // 7 days
        config.minimum_confidence = 0.9;
        config.enable_predictions = true;
        config.enable_recommendations = true;
        return config;
    }
};

// ============================================================================
// Interfaces
// ============================================================================

/**
 * @brief Interface for degradation calculation algorithms
 */
class IDegradationCalculator {
public:
    virtual ~IDegradationCalculator() = default;

    /**
     * @brief Calculate wear increment for a time period
     * @param health Current component health
     * @param conditions Operating conditions
     * @param duration Time period
     * @return Wear amount (health percentage lost)
     */
    virtual Real calculate_wear(const ComponentHealth& health,
                                const OperatingConditions& conditions,
                                std::chrono::hours duration) = 0;

    /**
     * @brief Estimate remaining useful life
     * @param health Current component health
     * @param conditions Expected future conditions
     * @return Estimated hours until failure
     */
    virtual Real estimate_remaining_life(const ComponentHealth& health,
                                        const OperatingConditions& conditions) = 0;

    /**
     * @brief Determine health status from percentage
     * @param health_percentage Health (0-100)
     * @return Categorical status
     */
    virtual HealthStatus determine_status(Real health_percentage) = 0;
};

/**
 * @brief Interface for failure prediction
 */
class IFailurePredictor {
public:
    virtual ~IFailurePredictor() = default;

    /**
     * @brief Predict failures for an entity
     * @param degradation Current entity degradation state
     * @return Vector of failure predictions
     */
    virtual std::vector<FailurePrediction> predict_failures(
        const EntityDegradation& degradation) = 0;

    /**
     * @brief Calculate prediction confidence
     * @param health Current component health
     * @param conditions Operating conditions
     * @return Confidence value (0-1)
     */
    virtual Real calculate_confidence(const ComponentHealth& health,
                                     const OperatingConditions& conditions) = 0;
};

/**
 * @brief Interface for maintenance advisory
 */
class IMaintenanceAdvisor {
public:
    virtual ~IMaintenanceAdvisor() = default;

    /**
     * @brief Get maintenance recommendations for an entity
     * @param degradation Current entity degradation state
     * @return Vector of recommendations
     */
    virtual std::vector<MaintenanceRecommendation> get_recommendations(
        const EntityDegradation& degradation) = 0;

    /**
     * @brief Determine appropriate maintenance action for a component
     * @param health Current component health
     * @return Recommended action
     */
    virtual MaintenanceAction determine_action(const ComponentHealth& health) = 0;
};

// ============================================================================
// Main Degradation Model Class
// ============================================================================

/**
 * @brief Central manager for component degradation and predictive maintenance
 *
 * Tracks component health, applies wear models, predicts failures, and
 * recommends maintenance actions for entities in the simulation.
 *
 * Example usage:
 * @code
 * DegradationModelConfig config = DegradationModelConfig::default_config();
 * auto model = create_degradation_model(config);
 * model->initialize();
 *
 * // Register entity and components
 * model->register_entity(entity_id);
 * model->add_component(entity_id, "left_wheel", ComponentType::Mechanical);
 * model->add_component(entity_id, "motor", ComponentType::Electrical);
 *
 * // Update operating conditions and apply wear
 * OperatingConditions conditions;
 * conditions.load_factor = 0.8;
 * conditions.temperature_celsius = 85.0;
 * model->update_operating_conditions(entity_id, conditions);
 * model->apply_wear(entity_id, std::chrono::hours(1));
 *
 * // Get health and recommendations
 * auto health = model->get_component_health(entity_id, "left_wheel");
 * auto recommendations = model->get_recommendations(entity_id);
 * @endcode
 */
class DegradationModel {
public:
    /**
     * @brief Construct degradation model with configuration
     * @param config Model configuration
     */
    explicit DegradationModel(const DegradationModelConfig& config);
    virtual ~DegradationModel() = default;

    // ========================================================================
    // Lifecycle
    // ========================================================================

    /**
     * @brief Initialize the degradation model
     * @return Success or error code
     */
    virtual DegradationResult initialize() = 0;

    /**
     * @brief Shutdown the degradation model
     * @return Success or error code
     */
    virtual DegradationResult shutdown() = 0;

    // ========================================================================
    // Entity Management
    // ========================================================================

    /**
     * @brief Register an entity for degradation tracking
     * @param entity_id Entity to track
     * @return Success or error code
     */
    virtual DegradationResult register_entity(EntityId entity_id) = 0;

    /**
     * @brief Unregister an entity
     * @param entity_id Entity to remove
     * @return Success or error code
     */
    virtual DegradationResult unregister_entity(EntityId entity_id) = 0;

    /**
     * @brief Add a component to an entity
     * @param entity_id Entity ID
     * @param component_id Component identifier
     * @param type Component type
     * @param initial_health Initial health percentage (default 100)
     * @return Success or error code
     */
    virtual DegradationResult add_component(EntityId entity_id,
                                           std::string component_id,
                                           ComponentType type,
                                           Real initial_health = 100.0) = 0;

    // ========================================================================
    // Degradation Operations
    // ========================================================================

    /**
     * @brief Update operating conditions for an entity
     * @param entity_id Entity ID
     * @param conditions Current operating conditions
     * @return Success or error code
     */
    virtual DegradationResult update_operating_conditions(EntityId entity_id,
                                                         OperatingConditions conditions) = 0;

    /**
     * @brief Apply wear for a time period
     * @param entity_id Entity ID
     * @param duration Time period to simulate
     * @return Success or error code
     */
    virtual DegradationResult apply_wear(EntityId entity_id,
                                        std::chrono::hours duration) = 0;

    /**
     * @brief Record a maintenance action performed
     * @param entity_id Entity ID
     * @param component_id Component that was maintained
     * @param action Maintenance action performed
     * @return Success or error code
     */
    virtual DegradationResult record_maintenance(EntityId entity_id,
                                                std::string component_id,
                                                MaintenanceAction action) = 0;

    // ========================================================================
    // Health Queries
    // ========================================================================

    /**
     * @brief Get health information for a specific component
     * @param entity_id Entity ID
     * @param component_id Component identifier
     * @return Component health or nullopt if not found
     */
    virtual std::optional<ComponentHealth> get_component_health(
        EntityId entity_id,
        std::string component_id) const = 0;

    /**
     * @brief Get overall degradation state for an entity
     * @param entity_id Entity ID
     * @return Entity degradation or nullopt if not found
     */
    virtual std::optional<EntityDegradation> get_entity_degradation(
        EntityId entity_id) const = 0;

    // ========================================================================
    // Predictions and Recommendations
    // ========================================================================

    /**
     * @brief Get maintenance recommendations for an entity
     * @param entity_id Entity ID
     * @return Vector of recommendations
     */
    virtual std::vector<MaintenanceRecommendation> get_recommendations(
        EntityId entity_id) const = 0;

    /**
     * @brief Get failure predictions for an entity
     * @param entity_id Entity ID
     * @return Vector of failure predictions
     */
    virtual std::vector<FailurePrediction> get_predictions(
        EntityId entity_id) const = 0;

    // ========================================================================
    // Model Configuration
    // ========================================================================

    /**
     * @brief Set custom degradation calculator
     * @param calculator Calculator implementation
     * @return Success or error code
     */
    virtual DegradationResult set_calculator(
        std::shared_ptr<IDegradationCalculator> calculator) = 0;

    /**
     * @brief Set custom failure predictor
     * @param predictor Predictor implementation
     * @return Success or error code
     */
    virtual DegradationResult set_predictor(
        std::shared_ptr<IFailurePredictor> predictor) = 0;

    /**
     * @brief Set custom maintenance advisor
     * @param advisor Advisor implementation
     * @return Success or error code
     */
    virtual DegradationResult set_advisor(
        std::shared_ptr<IMaintenanceAdvisor> advisor) = 0;

    // ========================================================================
    // Statistics
    // ========================================================================

    /**
     * @brief Get degradation statistics
     * @return Statistics summary
     */
    virtual DegradationStats get_stats() const = 0;

    /**
     * @brief Get current configuration
     */
    virtual const DegradationModelConfig& get_config() const = 0;
};

// ============================================================================
// Factory Functions
// ============================================================================

/**
 * @brief Create a degradation model instance
 * @param config Configuration for the model
 * @return Unique pointer to the model
 */
std::unique_ptr<DegradationModel> create_degradation_model(
    const DegradationModelConfig& config);

/**
 * @brief Create a linear degradation calculator
 *
 * Uses simple linear wear model:
 * wear = base_rate * load * (1 + temp_coeff * (T - T_ref)) * duration
 *
 * @return Unique pointer to calculator
 */
std::unique_ptr<IDegradationCalculator> create_linear_calculator();

/**
 * @brief Create an exponential degradation calculator
 *
 * Uses exponential wear model with age acceleration:
 * wear = base_rate * load * exp(age * age_accel) * duration
 *
 * @return Unique pointer to calculator
 */
std::unique_ptr<IDegradationCalculator> create_exponential_calculator();

/**
 * @brief Create a simple failure predictor
 *
 * Projects current degradation rate linearly to estimate failure time.
 * Confidence decreases with prediction horizon.
 *
 * @return Unique pointer to predictor
 */
std::unique_ptr<IFailurePredictor> create_simple_predictor();

/**
 * @brief Create a rule-based maintenance advisor
 *
 * Uses threshold rules:
 * - <30%: Replace
 * - 30-50%: Repair
 * - 50-70%: Inspect
 * - >70%: None
 *
 * @return Unique pointer to advisor
 */
std::unique_ptr<IMaintenanceAdvisor> create_rule_based_advisor();

/**
 * @brief Create default wear factor for a component type
 * @param type Component type
 * @return Default wear factor parameters
 */
WearFactor create_default_wear_factor(ComponentType type);

// ============================================================================
// Inline Helper Functions
// ============================================================================

/**
 * @brief Determine health status from percentage
 */
inline HealthStatus health_percentage_to_status(Real health_percentage) {
    if (health_percentage <= 0.0) return HealthStatus::Failed;
    if (health_percentage < 30.0) return HealthStatus::Critical;
    if (health_percentage < 50.0) return HealthStatus::Poor;
    if (health_percentage < 70.0) return HealthStatus::Fair;
    if (health_percentage < 90.0) return HealthStatus::Good;
    return HealthStatus::Excellent;
}

/**
 * @brief Calculate urgency from remaining useful life
 * @param rul Remaining useful life in hours
 * @param threshold Urgency threshold in hours (default 168 = 1 week)
 * @return Urgency value (0-1)
 */
inline Real calculate_urgency_from_rul(std::chrono::hours rul,
                                       std::chrono::hours threshold = std::chrono::hours(168)) {
    if (rul <= std::chrono::hours(0)) return 1.0;
    if (rul >= threshold) return 0.0;
    return 1.0 - (static_cast<Real>(rul.count()) / static_cast<Real>(threshold.count()));
}

/**
 * @brief Calculate weighted average health
 * @param components Vector of component health states
 * @return Weighted average (0-100)
 */
inline Real calculate_weighted_health(const std::vector<ComponentHealth>& components) {
    if (components.empty()) return 100.0;

    Real total_health = 0.0;
    for (const auto& comp : components) {
        total_health += comp.health_percentage;
    }
    return total_health / static_cast<Real>(components.size());
}

/**
 * @brief Estimate mean time to failure from component health
 * @param components Vector of component health states
 * @return MTTF estimate in hours
 */
inline std::chrono::hours estimate_mttf(const std::vector<ComponentHealth>& components) {
    if (components.empty()) return std::chrono::hours(0);

    std::chrono::hours min_rul = std::chrono::hours::max();
    for (const auto& comp : components) {
        if (comp.remaining_useful_life < min_rul && comp.remaining_useful_life > std::chrono::hours(0)) {
            min_rul = comp.remaining_useful_life;
        }
    }
    return min_rul == std::chrono::hours::max() ? std::chrono::hours(0) : min_rul;
}

/**
 * @brief Count components in each health status category
 * @param components Vector of component health states
 * @return Map of status to count
 */
inline std::unordered_map<HealthStatus, UInt64> count_by_status(
    const std::vector<ComponentHealth>& components) {
    std::unordered_map<HealthStatus, UInt64> counts;
    counts[HealthStatus::Excellent] = 0;
    counts[HealthStatus::Good] = 0;
    counts[HealthStatus::Fair] = 0;
    counts[HealthStatus::Poor] = 0;
    counts[HealthStatus::Critical] = 0;
    counts[HealthStatus::Failed] = 0;

    for (const auto& comp : components) {
        counts[comp.status]++;
    }
    return counts;
}

/**
 * @brief Find the most critical component (lowest health)
 * @param components Vector of component health states
 * @return Pointer to most critical component or nullptr if empty
 */
inline const ComponentHealth* find_most_critical(
    const std::vector<ComponentHealth>& components) {
    if (components.empty()) return nullptr;

    const ComponentHealth* most_critical = &components[0];
    for (const auto& comp : components) {
        if (comp.health_percentage < most_critical->health_percentage) {
            most_critical = &comp;
        }
    }
    return most_critical;
}

/**
 * @brief Check if any component is in critical or failed state
 * @param components Vector of component health states
 * @return True if any component needs immediate attention
 */
inline bool has_critical_components(const std::vector<ComponentHealth>& components) {
    for (const auto& comp : components) {
        if (comp.status == HealthStatus::Critical || comp.status == HealthStatus::Failed) {
            return true;
        }
    }
    return false;
}

/**
 * @brief Filter components by type
 * @param components Vector of all components
 * @param type Component type to filter
 * @return Vector of matching components
 */
inline std::vector<ComponentHealth> filter_by_type(
    const std::vector<ComponentHealth>& components,
    ComponentType type) {
    std::vector<ComponentHealth> filtered;
    for (const auto& comp : components) {
        if (comp.type == type) {
            filtered.push_back(comp);
        }
    }
    return filtered;
}

/**
 * @brief Filter components by health status
 * @param components Vector of all components
 * @param status Health status to filter
 * @return Vector of matching components
 */
inline std::vector<ComponentHealth> filter_by_status(
    const std::vector<ComponentHealth>& components,
    HealthStatus status) {
    std::vector<ComponentHealth> filtered;
    for (const auto& comp : components) {
        if (comp.status == status) {
            filtered.push_back(comp);
        }
    }
    return filtered;
}

/**
 * @brief Calculate total operating hours across all components
 * @param components Vector of component health states
 * @return Total operating hours
 */
inline std::chrono::hours calculate_total_operating_hours(
    const std::vector<ComponentHealth>& components) {
    std::chrono::hours total{0};
    for (const auto& comp : components) {
        total += comp.operating_hours;
    }
    return total;
}

/**
 * @brief Calculate average degradation rate
 * @param components Vector of component health states
 * @return Average degradation rate (per hour)
 */
inline Real calculate_average_degradation_rate(
    const std::vector<ComponentHealth>& components) {
    if (components.empty()) return 0.0;

    Real total_rate = 0.0;
    for (const auto& comp : components) {
        total_rate += comp.degradation_rate;
    }
    return total_rate / static_cast<Real>(components.size());
}

/**
 * @brief Estimate cost impact of component failure
 * @param health Component health state
 * @param base_replacement_cost Base cost to replace component
 * @return Estimated total cost including downtime and secondary damage
 */
inline Real estimate_failure_cost(const ComponentHealth& health,
                                  Real base_replacement_cost) {
    // Lower health = higher risk of cascading failures and downtime
    Real health_factor = 1.0 + (100.0 - health.health_percentage) / 50.0;

    // Critical components cost more due to urgency
    Real urgency_factor = (health.status == HealthStatus::Critical) ? 2.0 : 1.0;

    return base_replacement_cost * health_factor * urgency_factor;
}

/**
 * @brief Determine if maintenance is overdue based on RUL
 * @param health Component health state
 * @param threshold_hours Overdue threshold (default 24 hours)
 * @return True if maintenance is overdue
 */
inline bool is_maintenance_overdue(const ComponentHealth& health,
                                   std::chrono::hours threshold_hours = std::chrono::hours(24)) {
    return health.remaining_useful_life <= threshold_hours &&
           health.remaining_useful_life > std::chrono::hours(0);
}

/**
 * @brief Sort components by urgency (lowest RUL first)
 * @param components Vector of component health states
 * @return Sorted vector (most urgent first)
 */
inline std::vector<ComponentHealth> sort_by_urgency(std::vector<ComponentHealth> components) {
    std::sort(components.begin(), components.end(),
              [](const ComponentHealth& a, const ComponentHealth& b) {
                  return a.remaining_useful_life < b.remaining_useful_life;
              });
    return components;
}

/**
 * @brief Sort recommendations by urgency
 * @param recommendations Vector of maintenance recommendations
 * @return Sorted vector (most urgent first)
 */
inline std::vector<MaintenanceRecommendation> sort_recommendations_by_urgency(
    std::vector<MaintenanceRecommendation> recommendations) {
    std::sort(recommendations.begin(), recommendations.end(),
              [](const MaintenanceRecommendation& a, const MaintenanceRecommendation& b) {
                  return a.urgency > b.urgency;
              });
    return recommendations;
}

/**
 * @brief Calculate maintenance cost savings from preventive action
 * @param component Current component health
 * @param preventive_cost Cost of preventive maintenance
 * @param failure_cost Cost if component fails
 * @return Expected savings (negative = preventive is more expensive)
 */
inline Real calculate_maintenance_savings(const ComponentHealth& component,
                                         Real preventive_cost,
                                         Real failure_cost) {
    // Probability of failure based on health
    Real failure_probability = 1.0 - (component.health_percentage / 100.0);

    // Expected cost of failure
    Real expected_failure_cost = failure_cost * failure_probability;

    // Savings from preventive maintenance
    return expected_failure_cost - preventive_cost;
}

/**
 * @brief Generate human-readable health report
 * @param health Component health state
 * @return String description of health status
 */
inline std::string generate_health_report(const ComponentHealth& health) {
    std::string report = "Component: " + health.component_id + "\n";
    report += "Type: " + std::string(component_type_to_string(health.type)) + "\n";
    report += "Health: " + std::to_string(static_cast<int>(health.health_percentage)) + "%\n";
    report += "Status: " + std::string(health_status_to_string(health.status)) + "\n";
    report += "Operating Hours: " + std::to_string(health.operating_hours.count()) + "\n";
    report += "Remaining Life: " + std::to_string(health.remaining_useful_life.count()) + " hours\n";
    report += "Degradation Rate: " + std::to_string(health.degradation_rate) + " %/hour\n";
    return report;
}

/**
 * @brief Generate human-readable recommendation report
 * @param recommendation Maintenance recommendation
 * @return String description of recommendation
 */
inline std::string generate_recommendation_report(const MaintenanceRecommendation& recommendation) {
    std::string report = "Component: " + recommendation.component_id + "\n";
    report += "Action: " + std::string(maintenance_action_to_string(recommendation.action)) + "\n";
    report += "Description: " + recommendation.description + "\n";
    report += "Urgency: " + std::to_string(static_cast<int>(recommendation.urgency * 100.0)) + "%\n";
    report += "Recommended Within: " + std::to_string(recommendation.recommended_within.count()) + " hours\n";
    report += "Estimated Cost: $" + std::to_string(static_cast<int>(recommendation.estimated_cost)) + "\n";
    report += "Confidence: " + std::to_string(static_cast<int>(recommendation.confidence * 100.0)) + "%\n";
    return report;
}

/**
 * @brief Apply maintenance action effect to component health
 * @param health Component health to modify
 * @param action Maintenance action performed
 * @return New health percentage after maintenance
 */
inline Real apply_maintenance_effect(ComponentHealth& health, MaintenanceAction action) {
    switch (action) {
        case MaintenanceAction::None:
            return health.health_percentage;

        case MaintenanceAction::Inspect:
            // Inspection doesn't restore health but resets assessment
            return health.health_percentage;

        case MaintenanceAction::Lubricate:
            // Lubrication can restore 5-10% health for mechanical components
            if (health.type == ComponentType::Mechanical || health.type == ComponentType::Hydraulic) {
                return std::min(100.0, health.health_percentage + 7.5);
            }
            return health.health_percentage;

        case MaintenanceAction::Adjust:
            // Adjustment can restore 10-15% health
            return std::min(100.0, health.health_percentage + 12.5);

        case MaintenanceAction::Repair:
            // Repair restores 30-50% health
            return std::min(100.0, health.health_percentage + 40.0);

        case MaintenanceAction::Replace:
            // Replacement restores to 100%
            return 100.0;

        case MaintenanceAction::Overhaul:
            // Overhaul restores to 95% (not quite new)
            return 95.0;

        default:
            return health.health_percentage;
    }
}

/**
 * @brief Validate wear factor parameters
 * @param factor Wear factor to validate
 * @return True if valid
 */
inline bool validate_wear_factor(const WearFactor& factor) {
    return factor.base_rate >= 0.0 &&
           factor.load_multiplier >= 0.0 &&
           factor.temperature_coefficient >= 0.0 &&
           factor.humidity_coefficient >= 0.0 &&
           factor.age_acceleration >= 0.0;
}

/**
 * @brief Validate operating conditions
 * @param conditions Operating conditions to validate
 * @return True if valid
 */
inline bool validate_operating_conditions(const OperatingConditions& conditions) {
    return conditions.load_factor >= 0.0 && conditions.load_factor <= 1.0 &&
           conditions.temperature_celsius >= -273.15 &&  // Above absolute zero
           conditions.humidity_percent >= 0.0 && conditions.humidity_percent <= 100.0 &&
           conditions.vibration_level >= 0.0 &&
           conditions.contamination_level >= 0.0 && conditions.contamination_level <= 1.0 &&
           conditions.continuous_operation.count() >= 0;
}

/**
 * @brief Validate component health data
 * @param health Component health to validate
 * @return True if valid
 */
inline bool validate_component_health(const ComponentHealth& health) {
    return !health.component_id.empty() &&
           health.health_percentage >= 0.0 && health.health_percentage <= 100.0 &&
           health.degradation_rate >= 0.0 &&
           health.operating_hours.count() >= 0 &&
           health.remaining_useful_life.count() >= 0;
}

/**
 * @brief Create default operating conditions (nominal)
 * @return Nominal operating conditions
 */
inline OperatingConditions create_nominal_conditions() {
    OperatingConditions conditions;
    conditions.load_factor = 0.5;            // 50% load
    conditions.temperature_celsius = 20.0;   // Room temperature
    conditions.humidity_percent = 50.0;      // Moderate humidity
    conditions.vibration_level = 0.1;        // Low vibration
    conditions.contamination_level = 0.0;    // Clean environment
    conditions.continuous_operation = std::chrono::hours(0);
    return conditions;
}

/**
 * @brief Create harsh operating conditions
 * @return Harsh operating conditions
 */
inline OperatingConditions create_harsh_conditions() {
    OperatingConditions conditions;
    conditions.load_factor = 1.0;            // Maximum load
    conditions.temperature_celsius = 80.0;   // High temperature
    conditions.humidity_percent = 90.0;      // High humidity
    conditions.vibration_level = 0.8;        // High vibration
    conditions.contamination_level = 0.6;    // Contaminated
    conditions.continuous_operation = std::chrono::hours(24);
    return conditions;
}

/**
 * @brief Interpolate between two operating conditions
 * @param a First condition
 * @param b Second condition
 * @param t Interpolation factor (0-1)
 * @return Interpolated conditions
 */
inline OperatingConditions interpolate_conditions(const OperatingConditions& a,
                                                  const OperatingConditions& b,
                                                  Real t) {
    t = std::clamp(t, 0.0, 1.0);
    OperatingConditions result;
    result.load_factor = a.load_factor + t * (b.load_factor - a.load_factor);
    result.temperature_celsius = a.temperature_celsius + t * (b.temperature_celsius - a.temperature_celsius);
    result.humidity_percent = a.humidity_percent + t * (b.humidity_percent - a.humidity_percent);
    result.vibration_level = a.vibration_level + t * (b.vibration_level - a.vibration_level);
    result.contamination_level = a.contamination_level + t * (b.contamination_level - a.contamination_level);

    // Interpolate continuous operation hours
    auto hours_a = a.continuous_operation.count();
    auto hours_b = b.continuous_operation.count();
    result.continuous_operation = std::chrono::hours(
        static_cast<std::chrono::hours::rep>(hours_a + t * (hours_b - hours_a))
    );

    return result;
}

/**
 * @brief Calculate environmental severity factor
 * @param conditions Operating conditions
 * @return Severity factor (1.0 = nominal, higher = more severe)
 */
inline Real calculate_environmental_severity(const OperatingConditions& conditions) {
    Real severity = 1.0;

    // Load factor contribution
    severity += conditions.load_factor * 0.5;

    // Temperature contribution (normalized to 20C reference)
    Real temp_deviation = std::abs(conditions.temperature_celsius - 20.0);
    severity += temp_deviation / 100.0;

    // Humidity contribution (worst at extremes)
    Real humidity_deviation = std::abs(conditions.humidity_percent - 50.0);
    severity += humidity_deviation / 200.0;

    // Vibration contribution
    severity += conditions.vibration_level * 0.3;

    // Contamination contribution
    severity += conditions.contamination_level * 0.4;

    // Continuous operation penalty
    if (conditions.continuous_operation > std::chrono::hours(12)) {
        severity += 0.2;
    }

    return severity;
}

/**
 * @brief Estimate energy consumption impact of degradation
 * @param health Component health state
 * @param nominal_power Nominal power consumption
 * @return Estimated actual power consumption
 */
inline Real estimate_degradation_power_impact(const ComponentHealth& health,
                                              Real nominal_power) {
    // Degraded components often consume more power
    // Loss of efficiency typically scales with health degradation
    Real efficiency_loss = (100.0 - health.health_percentage) / 100.0;
    return nominal_power * (1.0 + efficiency_loss * 0.3);  // Up to 30% increase
}

} // namespace jaguar::thread
