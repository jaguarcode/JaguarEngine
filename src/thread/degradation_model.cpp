/**
 * @file degradation_model.cpp
 * @brief Implementation of physics-based degradation modeling for predictive maintenance
 */

#include "jaguar/thread/degradation_model.h"
#include <mutex>
#include <atomic>
#include <algorithm>
#include <cmath>

namespace jaguar::thread {

// ============================================================================
// Linear Degradation Calculator Implementation
// ============================================================================

class LinearDegradationCalculator : public IDegradationCalculator {
public:
    LinearDegradationCalculator() = default;
    ~LinearDegradationCalculator() override = default;

    Real calculate_wear(const ComponentHealth& health,
                       const OperatingConditions& conditions,
                       std::chrono::hours duration) override {
        // Get wear factor based on component type
        WearFactor factor = create_default_wear_factor(health.type);

        // Calculate temperature factor (Arrhenius-like relationship)
        // Reference temperature is 20°C
        Real temp_factor = 1.0 + factor.temperature_coefficient *
                          (conditions.temperature_celsius - 20.0);
        temp_factor = std::max(0.1, temp_factor);  // Prevent negative or zero

        // Calculate humidity factor
        Real humidity_factor = 1.0 + factor.humidity_coefficient *
                              (conditions.humidity_percent - 50.0);
        humidity_factor = std::max(0.1, humidity_factor);

        // Calculate age acceleration factor
        Real age_hours = static_cast<Real>(health.operating_hours.count());
        Real age_factor = 1.0 + factor.age_acceleration * age_hours;

        // Calculate load factor impact
        Real load_impact = factor.load_multiplier * conditions.load_factor;

        // Calculate contamination and vibration impacts
        Real contamination_impact = 1.0 + conditions.contamination_level * 0.5;
        Real vibration_impact = 1.0 + conditions.vibration_level * 0.3;

        // Calculate total wear
        Real duration_hours = static_cast<Real>(duration.count());
        Real wear = factor.base_rate * duration_hours * load_impact *
                   temp_factor * humidity_factor * age_factor *
                   contamination_impact * vibration_impact;

        return std::max(0.0, wear);
    }

    Real estimate_remaining_life(const ComponentHealth& health,
                                const OperatingConditions& conditions) override {
        if (health.degradation_rate <= 0.0) {
            return 10000.0;  // Very long life if no degradation
        }

        // Calculate current effective degradation rate
        Real current_wear = calculate_wear(health, conditions, std::chrono::hours(1));

        if (current_wear <= 0.0) {
            return 10000.0;
        }

        // Estimate hours until health reaches 0
        Real remaining_health = health.health_percentage;
        Real hours_remaining = remaining_health / current_wear;

        return std::max(0.0, hours_remaining);
    }

    HealthStatus determine_status(Real health_percentage) override {
        return health_percentage_to_status(health_percentage);
    }
};

// ============================================================================
// Exponential Degradation Calculator Implementation
// ============================================================================

class ExponentialDegradationCalculator : public IDegradationCalculator {
public:
    ExponentialDegradationCalculator() = default;
    ~ExponentialDegradationCalculator() override = default;

    Real calculate_wear(const ComponentHealth& health,
                       const OperatingConditions& conditions,
                       std::chrono::hours duration) override {
        // Get wear factor based on component type
        WearFactor factor = create_default_wear_factor(health.type);

        // Calculate age-based acceleration (exponential)
        Real age_hours = static_cast<Real>(health.operating_hours.count());
        Real age_factor = std::exp(factor.age_acceleration * age_hours / 1000.0);

        // Temperature factor (exponential Arrhenius)
        Real temp_diff = conditions.temperature_celsius - 20.0;
        Real temp_factor = std::exp(factor.temperature_coefficient * temp_diff / 10.0);

        // Load factor
        Real load_impact = 1.0 + factor.load_multiplier * conditions.load_factor;

        // Environmental factors
        Real env_factor = 1.0 + conditions.contamination_level * 0.5 +
                         conditions.vibration_level * 0.3;

        // Exponential decay model
        Real duration_hours = static_cast<Real>(duration.count());
        Real decay_rate = factor.base_rate * load_impact * temp_factor *
                         age_factor * env_factor;

        // health_loss = health * (1 - exp(-rate * time))
        Real wear = health.health_percentage *
                   (1.0 - std::exp(-decay_rate * duration_hours));

        return std::max(0.0, wear);
    }

    Real estimate_remaining_life(const ComponentHealth& health,
                                const OperatingConditions& conditions) override {
        if (health.health_percentage <= 0.0) {
            return 0.0;
        }

        // Use exponential model to estimate time to failure
        WearFactor factor = create_default_wear_factor(health.type);
        Real age_hours = static_cast<Real>(health.operating_hours.count());
        Real age_factor = std::exp(factor.age_acceleration * age_hours / 1000.0);

        Real temp_diff = conditions.temperature_celsius - 20.0;
        Real temp_factor = std::exp(factor.temperature_coefficient * temp_diff / 10.0);

        Real load_impact = 1.0 + factor.load_multiplier * conditions.load_factor;
        Real env_factor = 1.0 + conditions.contamination_level * 0.5 +
                         conditions.vibration_level * 0.3;

        Real decay_rate = factor.base_rate * load_impact * temp_factor *
                         age_factor * env_factor;

        if (decay_rate <= 0.0) {
            return 10000.0;
        }

        // Solve for time when health reaches 5% (practical failure threshold)
        // h(t) = h0 * exp(-k*t)
        // t = -ln(h_final/h0) / k
        Real failure_threshold = 5.0;
        Real time_to_failure = -std::log(failure_threshold / health.health_percentage) / decay_rate;

        return std::max(0.0, time_to_failure);
    }

    HealthStatus determine_status(Real health_percentage) override {
        return health_percentage_to_status(health_percentage);
    }
};

// ============================================================================
// Simple Failure Predictor Implementation
// ============================================================================

class SimpleFailurePredictor : public IFailurePredictor {
public:
    SimpleFailurePredictor() = default;
    ~SimpleFailurePredictor() override = default;

    std::vector<FailurePrediction> predict_failures(
        const EntityDegradation& degradation) override {

        std::vector<FailurePrediction> predictions;
        auto now = std::chrono::system_clock::now();

        for (const auto& component : degradation.components) {
            // Only predict for components below 50% health
            if (component.health_percentage < 50.0 && component.health_percentage > 0.0) {
                FailurePrediction prediction;
                prediction.component_id = component.component_id;

                // Estimate time to failure based on remaining useful life
                auto hours_to_failure = component.remaining_useful_life;
                prediction.predicted_failure = now + hours_to_failure;

                // Calculate confidence based on health status and degradation rate
                Real confidence = calculate_confidence(component, OperatingConditions{});
                prediction.confidence = confidence;

                // Determine failure mode based on component type
                prediction.failure_mode = determine_failure_mode(component);

                // Generate preventive actions
                prediction.preventive_actions = generate_preventive_actions(component);

                predictions.push_back(prediction);
            }
        }

        return predictions;
    }

    Real calculate_confidence(const ComponentHealth& health,
                            const OperatingConditions& conditions) override {
        (void)conditions;  // Not used in simple predictor

        // Confidence decreases as health decreases (more uncertain about exact failure time)
        Real health_confidence = health.health_percentage / 100.0;

        // Confidence increases with consistent degradation rate
        Real rate_confidence = 0.8;  // Assume good data quality

        // Confidence decreases with very low or very high degradation rates
        if (health.degradation_rate < 0.001 || health.degradation_rate > 10.0) {
            rate_confidence *= 0.5;
        }

        // Confidence decreases for very short or very long RUL predictions
        Real rul_hours = static_cast<Real>(health.remaining_useful_life.count());
        Real rul_confidence = 1.0;
        if (rul_hours < 10.0) {
            rul_confidence = 0.6;  // Very uncertain about imminent failures
        } else if (rul_hours > 1000.0) {
            rul_confidence = 0.7;  // Less certain about distant failures
        }

        return health_confidence * rate_confidence * rul_confidence;
    }

private:
    std::string determine_failure_mode(const ComponentHealth& health) const {
        switch (health.type) {
            case ComponentType::Mechanical:
                if (health.health_percentage < 20.0) {
                    return "Catastrophic wear/seizure";
                }
                return "Progressive wear/degradation";

            case ComponentType::Electrical:
                return "Insulation breakdown/short circuit";

            case ComponentType::Hydraulic:
                return "Seal failure/leakage";

            case ComponentType::Structural:
                return "Fatigue crack/fracture";

            case ComponentType::Electronic:
                return "Component failure/drift";

            case ComponentType::Consumable:
                return "Depletion/contamination";

            default:
                return "Unknown failure mode";
        }
    }

    std::vector<MaintenanceRecommendation> generate_preventive_actions(
        const ComponentHealth& health) const {

        std::vector<MaintenanceRecommendation> actions;

        MaintenanceRecommendation action;
        action.component_id = health.component_id;

        if (health.health_percentage < 20.0) {
            action.action = MaintenanceAction::Replace;
            action.description = "Replace component before failure";
            action.urgency = 0.9;
            action.recommended_within = std::chrono::hours(24);
            action.estimated_cost = 1000.0;
        } else if (health.health_percentage < 30.0) {
            action.action = MaintenanceAction::Repair;
            action.description = "Repair component to prevent failure";
            action.urgency = 0.7;
            action.recommended_within = std::chrono::hours(72);
            action.estimated_cost = 500.0;
        } else {
            action.action = MaintenanceAction::Inspect;
            action.description = "Inspect component and plan maintenance";
            action.urgency = 0.4;
            action.recommended_within = std::chrono::hours(168);
            action.estimated_cost = 100.0;
        }

        action.confidence = 0.8;
        actions.push_back(action);

        return actions;
    }
};

// ============================================================================
// Rule-Based Maintenance Advisor Implementation
// ============================================================================

class RuleBasedMaintenanceAdvisor : public IMaintenanceAdvisor {
public:
    RuleBasedMaintenanceAdvisor() = default;
    ~RuleBasedMaintenanceAdvisor() override = default;

    std::vector<MaintenanceRecommendation> get_recommendations(
        const EntityDegradation& degradation) override {

        std::vector<MaintenanceRecommendation> recommendations;

        for (const auto& component : degradation.components) {
            // Skip components in excellent condition
            if (component.status == HealthStatus::Excellent) {
                continue;
            }

            MaintenanceRecommendation rec;
            rec.component_id = component.component_id;
            rec.action = determine_action(component);
            rec.description = generate_description(component, rec.action);
            rec.urgency = calculate_urgency_from_rul(
                component.remaining_useful_life,
                std::chrono::hours(168)
            );
            rec.recommended_within = calculate_time_window(component);
            rec.estimated_cost = estimate_action_cost(component, rec.action);
            rec.confidence = 0.85;

            recommendations.push_back(rec);
        }

        return recommendations;
    }

    MaintenanceAction determine_action(const ComponentHealth& health) override {
        Real hp = health.health_percentage;

        if (hp <= 0.0) {
            return MaintenanceAction::Replace;
        } else if (hp < 30.0) {
            return MaintenanceAction::Replace;
        } else if (hp < 50.0) {
            return MaintenanceAction::Repair;
        } else if (hp < 70.0) {
            // Choose based on component type
            if (health.type == ComponentType::Mechanical ||
                health.type == ComponentType::Hydraulic) {
                return MaintenanceAction::Lubricate;
            }
            return MaintenanceAction::Adjust;
        } else if (hp < 90.0) {
            return MaintenanceAction::Inspect;
        }

        return MaintenanceAction::None;
    }

private:
    std::string generate_description(const ComponentHealth& health,
                                    MaintenanceAction action) const {
        std::string desc = component_type_to_string(health.type);
        desc += " component '";
        desc += health.component_id;
        desc += "' - ";

        switch (action) {
            case MaintenanceAction::None:
                desc += "No action required";
                break;
            case MaintenanceAction::Inspect:
                desc += "Schedule inspection";
                break;
            case MaintenanceAction::Lubricate:
                desc += "Apply lubrication";
                break;
            case MaintenanceAction::Adjust:
                desc += "Adjust/calibrate component";
                break;
            case MaintenanceAction::Repair:
                desc += "Repair component";
                break;
            case MaintenanceAction::Replace:
                desc += "Replace component immediately";
                break;
            case MaintenanceAction::Overhaul:
                desc += "Complete overhaul required";
                break;
        }

        desc += " (Health: ";
        desc += std::to_string(static_cast<int>(health.health_percentage));
        desc += "%)";

        return desc;
    }

    std::chrono::hours calculate_time_window(const ComponentHealth& health) const {
        Real hp = health.health_percentage;

        if (hp < 10.0) {
            return std::chrono::hours(8);   // 8 hours - very urgent
        } else if (hp < 30.0) {
            return std::chrono::hours(24);  // 1 day
        } else if (hp < 50.0) {
            return std::chrono::hours(72);  // 3 days
        } else if (hp < 70.0) {
            return std::chrono::hours(168); // 1 week
        } else {
            return std::chrono::hours(720); // 1 month
        }
    }

    Real estimate_action_cost(const ComponentHealth& health,
                            MaintenanceAction action) const {
        // Base cost by component type
        Real base_cost = 100.0;

        switch (health.type) {
            case ComponentType::Mechanical:
                base_cost = 500.0;
                break;
            case ComponentType::Electrical:
                base_cost = 300.0;
                break;
            case ComponentType::Hydraulic:
                base_cost = 400.0;
                break;
            case ComponentType::Structural:
                base_cost = 1000.0;
                break;
            case ComponentType::Electronic:
                base_cost = 600.0;
                break;
            case ComponentType::Consumable:
                base_cost = 50.0;
                break;
        }

        // Multiply by action factor
        Real action_multiplier = 1.0;
        switch (action) {
            case MaintenanceAction::None:
                return 0.0;
            case MaintenanceAction::Inspect:
                action_multiplier = 0.2;
                break;
            case MaintenanceAction::Lubricate:
                action_multiplier = 0.3;
                break;
            case MaintenanceAction::Adjust:
                action_multiplier = 0.5;
                break;
            case MaintenanceAction::Repair:
                action_multiplier = 1.5;
                break;
            case MaintenanceAction::Replace:
                action_multiplier = 2.0;
                break;
            case MaintenanceAction::Overhaul:
                action_multiplier = 3.0;
                break;
        }

        return base_cost * action_multiplier;
    }
};

// ============================================================================
// DegradationModel Implementation (pImpl)
// ============================================================================

class DegradationModelImpl : public DegradationModel {
private:
    struct Impl {
        DegradationModelConfig config;
        std::unordered_map<EntityId, EntityDegradation> entities;
        std::unordered_map<EntityId, OperatingConditions> operating_conditions;

        std::shared_ptr<IDegradationCalculator> calculator;
        std::shared_ptr<IFailurePredictor> predictor;
        std::shared_ptr<IMaintenanceAdvisor> advisor;

        std::atomic<bool> initialized{false};
        mutable std::mutex mutex;
        DegradationStats stats;
    };

    std::unique_ptr<Impl> impl_;

public:
    explicit DegradationModelImpl(const DegradationModelConfig& config)
        : DegradationModel(config)
        , impl_(std::make_unique<Impl>()) {
        impl_->config = config;
    }

    ~DegradationModelImpl() override = default;

    // ========================================================================
    // Lifecycle
    // ========================================================================

    DegradationResult initialize() override {
        std::lock_guard<std::mutex> lock(impl_->mutex);

        if (impl_->initialized) {
            return DegradationResult::AlreadyInitialized;
        }

        // Create default components
        impl_->calculator = create_linear_calculator();
        impl_->predictor = create_simple_predictor();
        impl_->advisor = create_rule_based_advisor();

        impl_->initialized = true;
        return DegradationResult::Success;
    }

    DegradationResult shutdown() override {
        std::lock_guard<std::mutex> lock(impl_->mutex);

        if (!impl_->initialized) {
            return DegradationResult::NotInitialized;
        }

        impl_->entities.clear();
        impl_->operating_conditions.clear();
        impl_->calculator.reset();
        impl_->predictor.reset();
        impl_->advisor.reset();

        impl_->initialized = false;
        return DegradationResult::Success;
    }

    // ========================================================================
    // Entity Management
    // ========================================================================

    DegradationResult register_entity(EntityId entity_id) override {
        std::lock_guard<std::mutex> lock(impl_->mutex);

        if (!impl_->initialized) {
            return DegradationResult::NotInitialized;
        }

        if (entity_id == INVALID_ENTITY_ID) {
            return DegradationResult::InvalidEntityId;
        }

        // Check if already registered
        if (impl_->entities.find(entity_id) != impl_->entities.end()) {
            return DegradationResult::Success;  // Already registered
        }

        EntityDegradation degradation;
        degradation.entity_id = entity_id;
        degradation.overall_health = 100.0;
        degradation.mean_time_to_failure = std::chrono::hours(0);
        degradation.last_assessment = std::chrono::system_clock::now();

        impl_->entities[entity_id] = degradation;

        // Initialize operating conditions
        impl_->operating_conditions[entity_id] = create_nominal_conditions();

        return DegradationResult::Success;
    }

    DegradationResult unregister_entity(EntityId entity_id) override {
        std::lock_guard<std::mutex> lock(impl_->mutex);

        if (!impl_->initialized) {
            return DegradationResult::NotInitialized;
        }

        auto it = impl_->entities.find(entity_id);
        if (it == impl_->entities.end()) {
            return DegradationResult::InvalidEntityId;
        }

        impl_->entities.erase(it);
        impl_->operating_conditions.erase(entity_id);

        return DegradationResult::Success;
    }

    DegradationResult add_component(EntityId entity_id,
                                   std::string component_id,
                                   ComponentType type,
                                   Real initial_health = 100.0) override {
        std::lock_guard<std::mutex> lock(impl_->mutex);

        if (!impl_->initialized) {
            return DegradationResult::NotInitialized;
        }

        auto it = impl_->entities.find(entity_id);
        if (it == impl_->entities.end()) {
            return DegradationResult::InvalidEntityId;
        }

        // Check if component already exists
        for (const auto& comp : it->second.components) {
            if (comp.component_id == component_id) {
                return DegradationResult::Success;  // Already exists
            }
        }

        ComponentHealth health;
        health.component_id = component_id;
        health.type = type;
        health.health_percentage = std::clamp(initial_health, 0.0, 100.0);
        health.degradation_rate = 0.0;
        health.status = health_percentage_to_status(health.health_percentage);
        health.last_updated = std::chrono::system_clock::now();
        health.operating_hours = std::chrono::hours(0);
        health.remaining_useful_life = std::chrono::hours(10000);

        it->second.components.push_back(health);

        // Update overall health
        it->second.overall_health = calculate_weighted_health(it->second.components);

        return DegradationResult::Success;
    }

    // ========================================================================
    // Degradation Operations
    // ========================================================================

    DegradationResult update_operating_conditions(EntityId entity_id,
                                                 OperatingConditions conditions) override {
        std::lock_guard<std::mutex> lock(impl_->mutex);

        if (!impl_->initialized) {
            return DegradationResult::NotInitialized;
        }

        auto it = impl_->entities.find(entity_id);
        if (it == impl_->entities.end()) {
            return DegradationResult::InvalidEntityId;
        }

        // Validate conditions
        if (!validate_operating_conditions(conditions)) {
            return DegradationResult::InvalidConfiguration;
        }

        impl_->operating_conditions[entity_id] = conditions;

        return DegradationResult::Success;
    }

    DegradationResult apply_wear(EntityId entity_id,
                                std::chrono::hours duration) override {
        std::lock_guard<std::mutex> lock(impl_->mutex);

        if (!impl_->initialized) {
            return DegradationResult::NotInitialized;
        }

        auto it = impl_->entities.find(entity_id);
        if (it == impl_->entities.end()) {
            return DegradationResult::InvalidEntityId;
        }

        if (!impl_->calculator) {
            return DegradationResult::ModelNotLoaded;
        }

        // Get operating conditions
        OperatingConditions conditions = impl_->operating_conditions[entity_id];

        // Apply wear to each component
        for (auto& component : it->second.components) {
            // Calculate wear
            Real wear = impl_->calculator->calculate_wear(component, conditions, duration);

            // Apply wear (reduce health)
            component.health_percentage = std::max(0.0, component.health_percentage - wear);

            // Update degradation rate
            if (duration.count() > 0) {
                component.degradation_rate = wear / static_cast<Real>(duration.count());
            }

            // Update status
            component.status = impl_->calculator->determine_status(component.health_percentage);

            // Update operating hours
            component.operating_hours += duration;

            // Estimate remaining useful life
            Real rul_hours = impl_->calculator->estimate_remaining_life(component, conditions);
            component.remaining_useful_life = std::chrono::hours(
                static_cast<std::chrono::hours::rep>(rul_hours)
            );

            // Update timestamp
            component.last_updated = std::chrono::system_clock::now();
        }

        // Update overall entity health
        it->second.overall_health = calculate_weighted_health(it->second.components);
        it->second.mean_time_to_failure = estimate_mttf(it->second.components);
        it->second.last_assessment = std::chrono::system_clock::now();

        return DegradationResult::Success;
    }

    DegradationResult record_maintenance(EntityId entity_id,
                                        std::string component_id,
                                        MaintenanceAction action) override {
        std::lock_guard<std::mutex> lock(impl_->mutex);

        if (!impl_->initialized) {
            return DegradationResult::NotInitialized;
        }

        auto it = impl_->entities.find(entity_id);
        if (it == impl_->entities.end()) {
            return DegradationResult::InvalidEntityId;
        }

        // Find component
        ComponentHealth* target_component = nullptr;
        for (auto& comp : it->second.components) {
            if (comp.component_id == component_id) {
                target_component = &comp;
                break;
            }
        }

        if (!target_component) {
            return DegradationResult::ComponentNotFound;
        }

        // Apply maintenance effect based on action
        switch (action) {
            case MaintenanceAction::None:
                // No effect
                break;

            case MaintenanceAction::Inspect:
                // Inspection doesn't restore health but updates assessment
                target_component->last_updated = std::chrono::system_clock::now();
                break;

            case MaintenanceAction::Lubricate:
                // Restore 5% health for mechanical/hydraulic
                if (target_component->type == ComponentType::Mechanical ||
                    target_component->type == ComponentType::Hydraulic) {
                    target_component->health_percentage =
                        std::min(100.0, target_component->health_percentage + 5.0);
                }
                break;

            case MaintenanceAction::Adjust:
                // Restore 10% health
                target_component->health_percentage =
                    std::min(100.0, target_component->health_percentage + 10.0);
                break;

            case MaintenanceAction::Repair:
                // Restore 30% health
                target_component->health_percentage =
                    std::min(100.0, target_component->health_percentage + 30.0);
                break;

            case MaintenanceAction::Replace:
                // Reset to 100% and reset operating hours
                target_component->health_percentage = 100.0;
                target_component->operating_hours = std::chrono::hours(0);
                target_component->degradation_rate = 0.0;
                break;

            case MaintenanceAction::Overhaul:
                // Reset to 95% and reset operating hours
                target_component->health_percentage = 95.0;
                target_component->operating_hours = std::chrono::hours(0);
                target_component->degradation_rate = 0.0;
                break;
        }

        // Update status and timestamp
        target_component->status = health_percentage_to_status(
            target_component->health_percentage
        );
        target_component->last_updated = std::chrono::system_clock::now();

        // Recalculate RUL
        if (impl_->calculator) {
            OperatingConditions conditions = impl_->operating_conditions[entity_id];
            Real rul_hours = impl_->calculator->estimate_remaining_life(
                *target_component, conditions
            );
            target_component->remaining_useful_life = std::chrono::hours(
                static_cast<std::chrono::hours::rep>(rul_hours)
            );
        }

        // Update overall health
        it->second.overall_health = calculate_weighted_health(it->second.components);

        return DegradationResult::Success;
    }

    // ========================================================================
    // Health Queries
    // ========================================================================

    std::optional<ComponentHealth> get_component_health(
        EntityId entity_id,
        std::string component_id) const override {

        std::lock_guard<std::mutex> lock(impl_->mutex);

        auto it = impl_->entities.find(entity_id);
        if (it == impl_->entities.end()) {
            return std::nullopt;
        }

        for (const auto& comp : it->second.components) {
            if (comp.component_id == component_id) {
                return comp;
            }
        }

        return std::nullopt;
    }

    std::optional<EntityDegradation> get_entity_degradation(
        EntityId entity_id) const override {

        std::lock_guard<std::mutex> lock(impl_->mutex);

        auto it = impl_->entities.find(entity_id);
        if (it == impl_->entities.end()) {
            return std::nullopt;
        }

        return it->second;
    }

    // ========================================================================
    // Predictions and Recommendations
    // ========================================================================

    std::vector<MaintenanceRecommendation> get_recommendations(
        EntityId entity_id) const override {

        std::lock_guard<std::mutex> lock(impl_->mutex);

        if (!impl_->advisor || !impl_->initialized) {
            return {};
        }

        auto it = impl_->entities.find(entity_id);
        if (it == impl_->entities.end()) {
            return {};
        }

        return impl_->advisor->get_recommendations(it->second);
    }

    std::vector<FailurePrediction> get_predictions(
        EntityId entity_id) const override {

        std::lock_guard<std::mutex> lock(impl_->mutex);

        if (!impl_->predictor || !impl_->initialized) {
            return {};
        }

        auto it = impl_->entities.find(entity_id);
        if (it == impl_->entities.end()) {
            return {};
        }

        return impl_->predictor->predict_failures(it->second);
    }

    // ========================================================================
    // Model Configuration
    // ========================================================================

    DegradationResult set_calculator(
        std::shared_ptr<IDegradationCalculator> calculator) override {

        std::lock_guard<std::mutex> lock(impl_->mutex);

        if (!calculator) {
            return DegradationResult::InvalidModel;
        }

        impl_->calculator = calculator;
        return DegradationResult::Success;
    }

    DegradationResult set_predictor(
        std::shared_ptr<IFailurePredictor> predictor) override {

        std::lock_guard<std::mutex> lock(impl_->mutex);

        if (!predictor) {
            return DegradationResult::InvalidModel;
        }

        impl_->predictor = predictor;
        return DegradationResult::Success;
    }

    DegradationResult set_advisor(
        std::shared_ptr<IMaintenanceAdvisor> advisor) override {

        std::lock_guard<std::mutex> lock(impl_->mutex);

        if (!advisor) {
            return DegradationResult::InvalidModel;
        }

        impl_->advisor = advisor;
        return DegradationResult::Success;
    }

    // ========================================================================
    // Statistics
    // ========================================================================

    DegradationStats get_stats() const override {
        std::lock_guard<std::mutex> lock(impl_->mutex);

        DegradationStats stats;
        stats.total_entities = static_cast<UInt64>(impl_->entities.size());
        stats.total_components = 0;
        stats.critical_components = 0;
        stats.pending_recommendations = 0;

        Real total_health = 0.0;

        for (const auto& [entity_id, degradation] : impl_->entities) {
            stats.total_components += static_cast<UInt64>(degradation.components.size());

            for (const auto& comp : degradation.components) {
                total_health += comp.health_percentage;

                if (comp.status == HealthStatus::Critical ||
                    comp.status == HealthStatus::Failed) {
                    stats.critical_components++;
                }

                stats.components_by_status[comp.status]++;
            }

            // Count recommendations
            if (impl_->advisor) {
                auto recs = impl_->advisor->get_recommendations(degradation);
                stats.pending_recommendations += static_cast<UInt64>(recs.size());
            }
        }

        if (stats.total_components > 0) {
            stats.average_health = total_health / static_cast<Real>(stats.total_components);
        } else {
            stats.average_health = 100.0;
        }

        return stats;
    }

    const DegradationModelConfig& get_config() const override {
        return impl_->config;
    }
};

// ============================================================================
// DegradationModel Base Class Implementation
// ============================================================================

DegradationModel::DegradationModel(const DegradationModelConfig& config) {
    (void)config;  // Unused in base class
}

// ============================================================================
// Factory Functions
// ============================================================================

std::unique_ptr<DegradationModel> create_degradation_model(
    const DegradationModelConfig& config) {
    return std::make_unique<DegradationModelImpl>(config);
}

std::unique_ptr<IDegradationCalculator> create_linear_calculator() {
    return std::make_unique<LinearDegradationCalculator>();
}

std::unique_ptr<IDegradationCalculator> create_exponential_calculator() {
    return std::make_unique<ExponentialDegradationCalculator>();
}

std::unique_ptr<IFailurePredictor> create_simple_predictor() {
    return std::make_unique<SimpleFailurePredictor>();
}

std::unique_ptr<IMaintenanceAdvisor> create_rule_based_advisor() {
    return std::make_unique<RuleBasedMaintenanceAdvisor>();
}

WearFactor create_default_wear_factor(ComponentType type) {
    auto config = DegradationModelConfig::default_config();

    auto it = config.default_wear_factors.find(type);
    if (it != config.default_wear_factors.end()) {
        return it->second;
    }

    // Fallback to generic defaults
    WearFactor factor;
    factor.base_rate = 0.001;
    factor.load_multiplier = 1.0;
    factor.temperature_coefficient = 0.01;
    factor.humidity_coefficient = 0.0;
    factor.age_acceleration = 0.0;

    return factor;
}

} // namespace jaguar::thread
