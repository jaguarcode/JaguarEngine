/**
 * @file threshold_monitor.h
 * @brief Threshold monitoring system for parameter tracking
 *
 * The ThresholdMonitor provides:
 * - Upper and lower threshold tracking
 * - Hysteresis support to prevent oscillation
 * - Automatic event emission when thresholds are crossed
 * - Recovery event emission when values return to normal
 */

#pragma once

#include "jaguar/events/event.h"
#include "jaguar/events/event_dispatcher.h"
#include <unordered_map>
#include <optional>

namespace jaguar::events {

/**
 * @brief Configuration for a single threshold
 */
struct ThresholdConfig {
    std::string parameter_name;
    std::optional<Real> upper_threshold;    ///< Upper limit (null if not tracked)
    std::optional<Real> lower_threshold;    ///< Lower limit (null if not tracked)
    Real hysteresis{0.0};                   ///< Hysteresis band to prevent oscillation
    bool emit_on_recover{true};             ///< Emit recovery event when back in bounds
    EventPriority priority{EventPriority::High};
};

/**
 * @brief State of a tracked threshold
 */
enum class ThresholdState {
    Normal,         ///< Value is within bounds
    UpperExceeded,  ///< Value exceeds upper threshold
    LowerExceeded   ///< Value is below lower threshold
};

/**
 * @brief Internal tracking data for a parameter
 */
struct ThresholdTracking {
    ThresholdConfig config;
    ThresholdState state{ThresholdState::Normal};
    Real last_value{0.0};
    Real last_event_value{0.0};
};

/**
 * @brief ThresholdMonitor - Tracks parameters and emits events on threshold crossings
 *
 * Usage:
 * @code
 * ThresholdMonitor monitor;
 * monitor.set_event_dispatcher(&dispatcher);
 * monitor.set_entity_id(entity_id);
 *
 * // Configure thresholds
 * monitor.add_threshold("speed", ThresholdConfig{
 *     .parameter_name = "speed",
 *     .upper_threshold = 100.0,
 *     .lower_threshold = std::nullopt,
 *     .hysteresis = 2.0
 * });
 *
 * // Update values during simulation
 * monitor.update("speed", current_speed, sim_time);
 * @endcode
 */
class ThresholdMonitor {
public:
    ThresholdMonitor() = default;
    ~ThresholdMonitor() = default;

    // Non-copyable, movable
    ThresholdMonitor(const ThresholdMonitor&) = delete;
    ThresholdMonitor& operator=(const ThresholdMonitor&) = delete;
    ThresholdMonitor(ThresholdMonitor&&) = default;
    ThresholdMonitor& operator=(ThresholdMonitor&&) = default;

    /**
     * @brief Set the event dispatcher for emitting events
     */
    void set_event_dispatcher(EventDispatcher* dispatcher) {
        event_dispatcher_ = dispatcher;
    }

    /**
     * @brief Set the entity ID that owns these thresholds
     */
    void set_entity_id(EntityId id) {
        entity_id_ = id;
    }

    /**
     * @brief Set the current simulation time (for event timestamps)
     */
    void set_current_time(Real time) {
        current_time_ = time;
    }

    /**
     * @brief Add or update a threshold configuration
     *
     * @param name Parameter name (must be unique within this monitor)
     * @param config Threshold configuration
     */
    void add_threshold(const std::string& name, ThresholdConfig config) {
        config.parameter_name = name;
        thresholds_[name] = ThresholdTracking{
            .config = std::move(config),
            .state = ThresholdState::Normal,
            .last_value = 0.0,
            .last_event_value = 0.0
        };
    }

    /**
     * @brief Remove a threshold
     *
     * @param name Parameter name to remove
     * @return true if threshold was found and removed
     */
    bool remove_threshold(const std::string& name) {
        return thresholds_.erase(name) > 0;
    }

    /**
     * @brief Update a parameter value and check thresholds
     *
     * @param name Parameter name
     * @param value Current value
     * @param timestamp Optional timestamp override (uses current_time_ if not provided)
     * @return true if a threshold event was emitted
     */
    bool update(const std::string& name, Real value, std::optional<Real> timestamp = std::nullopt) {
        auto it = thresholds_.find(name);
        if (it == thresholds_.end()) {
            return false;
        }

        ThresholdTracking& tracking = it->second;
        const ThresholdConfig& config = tracking.config;
        tracking.last_value = value;

        Real time = timestamp.value_or(current_time_);
        bool event_emitted = false;

        // Check upper threshold
        if (config.upper_threshold.has_value()) {
            Real upper = config.upper_threshold.value();
            Real upper_recover = upper - config.hysteresis;

            if (tracking.state != ThresholdState::UpperExceeded && value > upper) {
                // Crossed upper threshold
                tracking.state = ThresholdState::UpperExceeded;
                tracking.last_event_value = value;
                emit_threshold_exceeded(tracking, value, upper, true, time);
                event_emitted = true;
            } else if (tracking.state == ThresholdState::UpperExceeded && value < upper_recover) {
                // Recovered from upper threshold
                tracking.state = ThresholdState::Normal;
                if (config.emit_on_recover) {
                    emit_threshold_recovered(tracking, value, upper, true, time);
                    event_emitted = true;
                }
            }
        }

        // Check lower threshold (only if not already in upper exceeded state)
        if (config.lower_threshold.has_value() && tracking.state != ThresholdState::UpperExceeded) {
            Real lower = config.lower_threshold.value();
            Real lower_recover = lower + config.hysteresis;

            if (tracking.state != ThresholdState::LowerExceeded && value < lower) {
                // Crossed lower threshold
                tracking.state = ThresholdState::LowerExceeded;
                tracking.last_event_value = value;
                emit_threshold_exceeded(tracking, value, lower, false, time);
                event_emitted = true;
            } else if (tracking.state == ThresholdState::LowerExceeded && value > lower_recover) {
                // Recovered from lower threshold
                tracking.state = ThresholdState::Normal;
                if (config.emit_on_recover) {
                    emit_threshold_recovered(tracking, value, lower, false, time);
                    event_emitted = true;
                }
            }
        }

        return event_emitted;
    }

    /**
     * @brief Batch update multiple parameters
     *
     * @param values Map of parameter name to current value
     * @param timestamp Optional timestamp override
     * @return Number of threshold events emitted
     */
    SizeT update_batch(const std::unordered_map<std::string, Real>& values,
                       std::optional<Real> timestamp = std::nullopt) {
        SizeT events_emitted = 0;
        for (const auto& [name, value] : values) {
            if (update(name, value, timestamp)) {
                events_emitted++;
            }
        }
        return events_emitted;
    }

    /**
     * @brief Get current state of a threshold
     *
     * @param name Parameter name
     * @return Current threshold state, or Normal if not found
     */
    ThresholdState get_state(const std::string& name) const {
        auto it = thresholds_.find(name);
        if (it != thresholds_.end()) {
            return it->second.state;
        }
        return ThresholdState::Normal;
    }

    /**
     * @brief Check if any thresholds are currently exceeded
     */
    bool any_exceeded() const {
        for (const auto& [name, tracking] : thresholds_) {
            if (tracking.state != ThresholdState::Normal) {
                return true;
            }
        }
        return false;
    }

    /**
     * @brief Get list of currently exceeded thresholds
     */
    std::vector<std::string> get_exceeded_thresholds() const {
        std::vector<std::string> exceeded;
        for (const auto& [name, tracking] : thresholds_) {
            if (tracking.state != ThresholdState::Normal) {
                exceeded.push_back(name);
            }
        }
        return exceeded;
    }

    /**
     * @brief Reset a threshold state to normal (without emitting events)
     */
    void reset_state(const std::string& name) {
        auto it = thresholds_.find(name);
        if (it != thresholds_.end()) {
            it->second.state = ThresholdState::Normal;
        }
    }

    /**
     * @brief Reset all threshold states to normal
     */
    void reset_all_states() {
        for (auto& [name, tracking] : thresholds_) {
            tracking.state = ThresholdState::Normal;
        }
    }

    /**
     * @brief Clear all thresholds
     */
    void clear() {
        thresholds_.clear();
    }

    /**
     * @brief Get the number of configured thresholds
     */
    SizeT threshold_count() const {
        return thresholds_.size();
    }

private:
    void emit_threshold_exceeded(const ThresholdTracking& tracking, Real value,
                                  Real threshold, bool is_upper, Real timestamp) {
        if (!event_dispatcher_) return;

        auto event = Event::create_threshold_exceeded(
            entity_id_,
            tracking.config.parameter_name,
            value,
            threshold,
            is_upper,
            timestamp
        );
        event.base.priority = tracking.config.priority;
        event_dispatcher_->dispatch(event);
    }

    void emit_threshold_recovered(const ThresholdTracking& tracking, Real value,
                                   Real threshold, bool is_upper, Real timestamp) {
        if (!event_dispatcher_) return;

        // Create a ThresholdRecovered event
        Event event;
        event.base.type = EventType::ThresholdRecovered;
        event.base.source_entity = entity_id_;
        event.base.timestamp = timestamp;
        event.base.priority = tracking.config.priority;

        ThresholdEventData data;
        data.parameter_name = tracking.config.parameter_name;
        data.current_value = value;
        data.threshold_value = threshold;
        data.delta = is_upper ? (threshold - value) : (value - threshold);
        data.is_upper_limit = is_upper;
        event.data = std::move(data);

        event_dispatcher_->dispatch(event);
    }

    EventDispatcher* event_dispatcher_{nullptr};
    EntityId entity_id_{INVALID_ENTITY_ID};
    Real current_time_{0.0};
    std::unordered_map<std::string, ThresholdTracking> thresholds_;
};

// ============================================================================
// Common Threshold Presets
// ============================================================================

/**
 * @brief Create a threshold config for fuel monitoring
 *
 * @param low_warning Warning threshold (e.g., 20%)
 * @param critical_low Critical threshold (e.g., 5%)
 */
inline ThresholdConfig fuel_threshold(Real low_warning, [[maybe_unused]] Real critical_low = 5.0) {
    // Note: critical_low can be used for a second tier warning system if needed
    return ThresholdConfig{
        .parameter_name = "fuel_level",
        .upper_threshold = std::nullopt,
        .lower_threshold = low_warning,
        .hysteresis = 2.0,
        .emit_on_recover = true,
        .priority = EventPriority::High
    };
}

/**
 * @brief Create a threshold config for speed monitoring
 *
 * @param max_speed Maximum safe speed
 * @param hysteresis Speed hysteresis band
 */
inline ThresholdConfig speed_threshold(Real max_speed, Real hysteresis = 5.0) {
    return ThresholdConfig{
        .parameter_name = "speed",
        .upper_threshold = max_speed,
        .lower_threshold = std::nullopt,
        .hysteresis = hysteresis,
        .emit_on_recover = true,
        .priority = EventPriority::High
    };
}

/**
 * @brief Create a threshold config for temperature monitoring
 *
 * @param max_temp Maximum safe temperature
 * @param min_temp Minimum safe temperature (optional)
 */
inline ThresholdConfig temperature_threshold(Real max_temp, std::optional<Real> min_temp = std::nullopt) {
    return ThresholdConfig{
        .parameter_name = "temperature",
        .upper_threshold = max_temp,
        .lower_threshold = min_temp,
        .hysteresis = 2.0,
        .emit_on_recover = true,
        .priority = EventPriority::High
    };
}

/**
 * @brief Create a threshold config for altitude monitoring (aircraft)
 *
 * @param max_altitude Maximum altitude (service ceiling)
 * @param min_altitude Minimum safe altitude (ground proximity)
 */
inline ThresholdConfig altitude_threshold(Real max_altitude, Real min_altitude) {
    return ThresholdConfig{
        .parameter_name = "altitude",
        .upper_threshold = max_altitude,
        .lower_threshold = min_altitude,
        .hysteresis = 50.0,  // 50m hysteresis
        .emit_on_recover = true,
        .priority = EventPriority::Immediate  // Critical for safety
    };
}

/**
 * @brief Create a threshold config for G-force monitoring
 *
 * @param max_g Maximum G-force (positive)
 * @param min_g Minimum G-force (negative)
 */
inline ThresholdConfig gforce_threshold(Real max_g, Real min_g) {
    return ThresholdConfig{
        .parameter_name = "g_force",
        .upper_threshold = max_g,
        .lower_threshold = min_g,
        .hysteresis = 0.2,
        .emit_on_recover = true,
        .priority = EventPriority::Immediate
    };
}

} // namespace jaguar::events
