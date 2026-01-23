/**
 * @file sensor.cpp
 * @brief Sensor framework base implementation
 */

#include "jaguar/sensors/sensor.h"
#include "jaguar/events/event_dispatcher.h"
#include "jaguar/physics/entity.h"
#include <cmath>
#include <algorithm>

namespace jaguar::sensors {

// ============================================================================
// Utility Functions
// ============================================================================

const char* get_sensor_type_name(SensorType type) {
    switch (type) {
        case SensorType::IMU:           return "IMU";
        case SensorType::GPS:           return "GPS";
        case SensorType::Altimeter:     return "Altimeter";
        case SensorType::Airspeed:      return "Airspeed";
        case SensorType::Magnetometer:  return "Magnetometer";
        case SensorType::Radar:         return "Radar";
        case SensorType::Lidar:         return "Lidar";
        case SensorType::Camera:        return "Camera";
        case SensorType::Infrared:      return "Infrared";
        case SensorType::Sonar:         return "Sonar";
        case SensorType::DepthGauge:    return "DepthGauge";
        case SensorType::Custom:        return "Custom";
        default:                        return "Unknown";
    }
}

const char* get_sensor_state_name(SensorState state) {
    switch (state) {
        case SensorState::Uninitialized: return "Uninitialized";
        case SensorState::Initializing:  return "Initializing";
        case SensorState::Operational:   return "Operational";
        case SensorState::Degraded:      return "Degraded";
        case SensorState::Failed:        return "Failed";
        case SensorState::Disabled:      return "Disabled";
        default:                         return "Unknown";
    }
}

// ============================================================================
// ISensor Implementation
// ============================================================================

UInt64 ISensor::next_sensor_id_ = 1;

ISensor::ISensor(const SensorConfig& config)
    : config_(config)
    , sensor_id_(next_sensor_id_++) {
    // Initialize RNG with a default seed based on sensor ID
    rng_.seed(sensor_id_ * 12345 + 67890);
}

bool ISensor::initialize() {
    if (state_ != SensorState::Uninitialized &&
        state_ != SensorState::Disabled) {
        return false;  // Already initialized
    }

    warmup_elapsed_ = 0.0;
    time_since_update_ = 0.0;
    update_count_ = 0;
    sequence_number_ = 0;

    if (config_.warmup_time > 0.0) {
        set_state(SensorState::Initializing);
    } else {
        set_state(config_.initial_state);
    }

    emit_event(events::EventType::SensorActivated);
    return true;
}

void ISensor::update(const physics::EntityState& entity_state, Real dt) {
    if (!enabled_ || state_ == SensorState::Disabled ||
        state_ == SensorState::Failed) {
        return;
    }

    // Handle warmup phase
    if (state_ == SensorState::Initializing) {
        handle_warmup(dt);
        if (state_ == SensorState::Initializing) {
            return;  // Still warming up
        }
    }

    // Accumulate time and check if update is due
    time_since_update_ += dt;
    simulation_time_ += dt;

    if (!is_update_due(dt)) {
        return;  // Not time for an update yet
    }

    // Store dt for noise calculations
    Real update_dt = time_since_update_;
    time_since_update_ = 0.0;

    // Compute ideal measurement
    do_update(entity_state);

    // Apply noise model
    do_apply_noise(update_dt);

    // Apply any active failure modes
    if (failure_mode_ != FailureMode::None) {
        do_apply_failure();
    }

    update_count_++;
    sequence_number_++;
}

void ISensor::reset() {
    state_ = SensorState::Uninitialized;
    warmup_elapsed_ = 0.0;
    time_since_update_ = 0.0;
    update_count_ = 0;
    sequence_number_ = 0;
    failure_mode_ = FailureMode::None;
    failure_magnitude_ = 0.0;
}

void ISensor::shutdown() {
    set_state(SensorState::Disabled);
    emit_event(events::EventType::SensorDeactivated);
}

void ISensor::set_state(SensorState state) {
    if (state_ != state) {
        SensorState old_state = state_;
        state_ = state;
        on_state_change(old_state, state);
    }
}

void ISensor::set_enabled(bool enabled) {
    enabled_ = enabled;
    if (!enabled && state_ != SensorState::Disabled) {
        set_state(SensorState::Disabled);
    }
}

void ISensor::inject_failure(FailureMode mode, Real magnitude) {
    failure_mode_ = mode;
    failure_magnitude_ = magnitude;

    if (mode == FailureMode::Complete) {
        set_state(SensorState::Failed);
        emit_event(events::EventType::SensorFailure);
    } else if (mode != FailureMode::None) {
        set_state(SensorState::Degraded);
    }
}

void ISensor::clear_failures() {
    failure_mode_ = FailureMode::None;
    failure_magnitude_ = 0.0;

    if (state_ == SensorState::Degraded || state_ == SensorState::Failed) {
        set_state(SensorState::Operational);
    }
}

void ISensor::set_seed(UInt64 seed) {
    rng_.seed(seed);
}

void ISensor::do_apply_failure() {
    // Base implementation does nothing - derived classes override as needed
}

void ISensor::on_state_change(SensorState old_state, SensorState new_state) {
    // Base implementation just emits events
    if (new_state == SensorState::Failed) {
        emit_event(events::EventType::SensorFailure);
    }
}

// ============================================================================
// Noise Generation Utilities
// ============================================================================

Real ISensor::generate_white_noise(Real sigma) {
    if (sigma <= 0.0) return 0.0;
    return normal_dist_(rng_) * sigma;
}

Vec3 ISensor::generate_white_noise_vec3(const Vec3NoiseModel& model) {
    return Vec3{
        model.x.white_noise.enabled ? generate_white_noise(model.x.white_noise.sigma) : 0.0,
        model.y.white_noise.enabled ? generate_white_noise(model.y.white_noise.sigma) : 0.0,
        model.z.white_noise.enabled ? generate_white_noise(model.z.white_noise.sigma) : 0.0
    };
}

Real ISensor::apply_bias_instability(Real& bias_state,
                                     const BiasInstabilityParams& params, Real dt) {
    if (!params.enabled || params.sigma <= 0.0) return 0.0;

    // First-order Gauss-Markov process:
    // db/dt = -b/τ + σ_b * sqrt(2/τ) * w(t)
    // where w(t) is white noise
    Real tau = params.correlation_time;
    Real alpha = std::exp(-dt / tau);
    Real sigma_driving = params.sigma * std::sqrt(1.0 - alpha * alpha);

    bias_state = alpha * bias_state + sigma_driving * normal_dist_(rng_);
    return bias_state;
}

Real ISensor::apply_random_walk(Real& walk_state,
                                const RandomWalkParams& params, Real dt) {
    if (!params.enabled || params.sigma <= 0.0) return 0.0;

    // Random walk: integrated white noise
    // σ_integrated = σ_walk * sqrt(dt)
    Real increment = params.sigma * std::sqrt(dt) * normal_dist_(rng_);
    walk_state += increment;
    return walk_state;
}

Real ISensor::apply_quantization(Real value, Real resolution) {
    if (resolution <= 0.0) return value;
    return std::round(value / resolution) * resolution;
}

Real ISensor::clamp_to_range(Real value) const {
    return std::clamp(value, config_.range_min, config_.range_max);
}

void ISensor::emit_event(events::EventType event_type) {
    if (!event_dispatcher_) return;

    events::SensorEventData sensor_data;
    sensor_data.sensor_id = sensor_id_;
    sensor_data.sensor_type = get_sensor_type_name(config_.type);

    events::Event event;
    event.base.type = event_type;
    event.base.timestamp = simulation_time_;
    event.base.source_entity = config_.attached_entity;
    event.data = sensor_data;

    event_dispatcher_->dispatch(event);
}

bool ISensor::is_update_due(Real dt) {
    if (config_.update_rate <= 0.0) {
        return true;  // Always update if no rate limit
    }

    Real update_period = 1.0 / config_.update_rate;
    return time_since_update_ >= update_period;
}

void ISensor::handle_warmup(Real dt) {
    warmup_elapsed_ += dt;

    if (warmup_elapsed_ >= config_.warmup_time) {
        if (config_.auto_initialize) {
            set_state(SensorState::Operational);
        }
    }
}

// ============================================================================
// SensorManager Implementation
// ============================================================================

SensorManager::SensorManager() = default;

SensorManager::~SensorManager() {
    shutdown_all();
}

UInt64 SensorManager::add_sensor(std::unique_ptr<ISensor> sensor) {
    if (!sensor) return 0;

    UInt64 id = sensor->id();
    if (event_dispatcher_) {
        sensor->set_event_dispatcher(event_dispatcher_);
    }
    sensors_[id] = std::move(sensor);
    return id;
}

void SensorManager::remove_sensor(UInt64 sensor_id) {
    auto it = sensors_.find(sensor_id);
    if (it != sensors_.end()) {
        it->second->shutdown();
        sensors_.erase(it);
    }
}

ISensor* SensorManager::get_sensor(UInt64 sensor_id) {
    auto it = sensors_.find(sensor_id);
    return it != sensors_.end() ? it->second.get() : nullptr;
}

const ISensor* SensorManager::get_sensor(UInt64 sensor_id) const {
    auto it = sensors_.find(sensor_id);
    return it != sensors_.end() ? it->second.get() : nullptr;
}

std::vector<ISensor*> SensorManager::get_sensors_for_entity(EntityId entity_id) {
    std::vector<ISensor*> result;
    for (auto& [id, sensor] : sensors_) {
        if (sensor->attached_entity() == entity_id) {
            result.push_back(sensor.get());
        }
    }
    return result;
}

std::vector<ISensor*> SensorManager::get_sensors_by_type(SensorType type) {
    std::vector<ISensor*> result;
    for (auto& [id, sensor] : sensors_) {
        if (sensor->type() == type) {
            result.push_back(sensor.get());
        }
    }
    return result;
}

void SensorManager::update_all(physics::EntityManager& entity_manager, Real dt) {
    for (auto& [id, sensor] : sensors_) {
        EntityId entity_id = sensor->attached_entity();
        if (entity_id != INVALID_ENTITY_ID) {
            auto state = entity_manager.get_state(entity_id);
            sensor->update(state, dt);
        }
    }
}

void SensorManager::initialize_all() {
    for (auto& [id, sensor] : sensors_) {
        sensor->initialize();
    }
}

void SensorManager::reset_all() {
    for (auto& [id, sensor] : sensors_) {
        sensor->reset();
    }
}

void SensorManager::shutdown_all() {
    for (auto& [id, sensor] : sensors_) {
        sensor->shutdown();
    }
}

void SensorManager::set_event_dispatcher(events::EventDispatcher* dispatcher) {
    event_dispatcher_ = dispatcher;
    for (auto& [id, sensor] : sensors_) {
        sensor->set_event_dispatcher(dispatcher);
    }
}

void SensorManager::set_simulation_time(Real time) {
    for (auto& [id, sensor] : sensors_) {
        sensor->set_simulation_time(time);
    }
}

} // namespace jaguar::sensors
