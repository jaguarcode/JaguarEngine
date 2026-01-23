/**
 * @file script_callback.h
 * @brief Script callback interface for event system bindings
 *
 * This file provides:
 * - Abstract callback interface for scripting language integration
 * - Type-safe event data extraction for scripts
 * - Callback registry for managing script event handlers
 * - JSON serialization for event data exchange with scripts
 *
 * Designed to be used with:
 * - Python via pybind11
 * - Lua via sol2
 * - Any other scripting language with C++ interop
 */

#pragma once

#include "jaguar/events/event.h"
#include "jaguar/events/event_dispatcher.h"
#include <functional>
#include <memory>
#include <unordered_map>
#include <any>
#include <sstream>

namespace jaguar::events {

// ============================================================================
// Script Event Data
// ============================================================================

/**
 * @brief Simplified event data for script consumption
 *
 * Contains common fields that scripts typically need access to,
 * along with type-specific data in a map format.
 */
struct ScriptEventData {
    EventType type;
    EventCategory category;
    EntityId source_entity;
    Real timestamp;
    EventPriority priority;

    // Type-specific data as key-value pairs for easy script access
    std::unordered_map<std::string, std::any> fields;

    /**
     * @brief Get a field value with type safety
     *
     * @tparam T Expected type
     * @param name Field name
     * @param default_val Value to return if field not found
     * @return Field value or default
     */
    template <typename T>
    T get_field(const std::string& name, const T& default_val = T{}) const {
        auto it = fields.find(name);
        if (it != fields.end()) {
            try {
                return std::any_cast<T>(it->second);
            } catch (...) {
                return default_val;
            }
        }
        return default_val;
    }

    /**
     * @brief Check if a field exists
     */
    bool has_field(const std::string& name) const {
        return fields.count(name) > 0;
    }
};

// ============================================================================
// Event Data Converters
// ============================================================================

/**
 * @brief Convert a native Event to ScriptEventData
 */
inline ScriptEventData to_script_data(const Event& event) {
    ScriptEventData data;
    data.type = event.type();
    data.category = get_event_category(event.type());
    data.source_entity = event.source();
    data.timestamp = event.timestamp();
    data.priority = event.priority();

    // Extract type-specific data
    if (event.has_data<EntityEventData>()) {
        const auto& ed = event.get_data<EntityEventData>();
        data.fields["entity_id"] = ed.entity_id;
        data.fields["entity_name"] = ed.entity_name;
        data.fields["domain"] = static_cast<int>(ed.domain);
    }
    else if (event.has_data<CollisionEventData>()) {
        const auto& cd = event.get_data<CollisionEventData>();
        data.fields["entity_a"] = cd.entity_A;
        data.fields["entity_b"] = cd.entity_B;
        data.fields["contact_x"] = cd.contact_point.x;
        data.fields["contact_y"] = cd.contact_point.y;
        data.fields["contact_z"] = cd.contact_point.z;
        data.fields["normal_x"] = cd.contact_normal.x;
        data.fields["normal_y"] = cd.contact_normal.y;
        data.fields["normal_z"] = cd.contact_normal.z;
        data.fields["penetration"] = cd.penetration_depth;
        data.fields["impulse"] = cd.impulse_magnitude;
    }
    else if (event.has_data<ThresholdEventData>()) {
        const auto& td = event.get_data<ThresholdEventData>();
        data.fields["parameter_name"] = td.parameter_name;
        data.fields["current_value"] = td.current_value;
        data.fields["threshold_value"] = td.threshold_value;
        data.fields["delta"] = td.delta;
        data.fields["is_upper_limit"] = td.is_upper_limit;
    }
    else if (event.has_data<AirEventData>()) {
        const auto& ad = event.get_data<AirEventData>();
        data.fields["angle_of_attack"] = ad.angle_of_attack;
        data.fields["indicated_airspeed"] = ad.indicated_airspeed;
        data.fields["altitude"] = ad.altitude;
        data.fields["mach_number"] = ad.mach_number;
        data.fields["critical"] = ad.critical;
    }
    else if (event.has_data<LandEventData>()) {
        const auto& ld = event.get_data<LandEventData>();
        data.fields["wheel_index"] = ld.wheel_index;
        data.fields["contact_x"] = ld.contact_point.x;
        data.fields["contact_y"] = ld.contact_point.y;
        data.fields["contact_z"] = ld.contact_point.z;
        data.fields["vertical_speed"] = ld.vertical_speed;
        data.fields["lateral_speed"] = ld.lateral_speed;
        data.fields["ground_slope"] = ld.ground_slope;
    }
    else if (event.has_data<SeaEventData>()) {
        const auto& sd = event.get_data<SeaEventData>();
        data.fields["heel_angle"] = sd.heel_angle;
        data.fields["draft"] = sd.draft;
        data.fields["wave_height"] = sd.wave_height;
        data.fields["impact_x"] = sd.impact_point.x;
        data.fields["impact_y"] = sd.impact_point.y;
        data.fields["impact_z"] = sd.impact_point.z;
    }
    else if (event.has_data<SpaceEventData>()) {
        const auto& sd = event.get_data<SpaceEventData>();
        data.fields["altitude"] = sd.altitude;
        data.fields["velocity"] = sd.velocity;
        data.fields["eccentricity"] = sd.eccentricity;
        data.fields["inclination"] = sd.inclination;
        data.fields["target_entity"] = sd.target_entity;
    }
    else if (event.has_data<SensorEventData>()) {
        const auto& sd = event.get_data<SensorEventData>();
        data.fields["sensor_id"] = sd.sensor_id;
        data.fields["sensor_type"] = sd.sensor_type;
        data.fields["target_entity"] = sd.target_entity;
        data.fields["range"] = sd.range;
        data.fields["signal_strength"] = sd.signal_strength;
    }

    return data;
}

// ============================================================================
// Script Callback Interface
// ============================================================================

/**
 * @brief Abstract callback interface for script event handlers
 *
 * Implement this interface for each scripting language:
 * - PythonCallback: Wraps a Python callable
 * - LuaCallback: Wraps a Lua function
 */
class IScriptCallback {
public:
    virtual ~IScriptCallback() = default;

    /**
     * @brief Invoke the script callback with event data
     *
     * @param data Script-friendly event data
     * @return true if event should continue propagating
     */
    virtual bool invoke(const ScriptEventData& data) = 0;

    /**
     * @brief Get callback identifier for debugging
     */
    virtual std::string get_name() const = 0;

    /**
     * @brief Check if callback is still valid (script not unloaded)
     */
    virtual bool is_valid() const = 0;
};

/**
 * @brief Shared pointer type for script callbacks
 */
using ScriptCallbackPtr = std::shared_ptr<IScriptCallback>;

// ============================================================================
// C Function Callback (for C/FFI bindings)
// ============================================================================

/**
 * @brief C-style callback function signature
 */
using CEventCallback = bool (*)(const ScriptEventData* data, void* user_data);

/**
 * @brief Wrapper for C function callbacks
 */
class CFunctionCallback : public IScriptCallback {
public:
    CFunctionCallback(CEventCallback callback, void* user_data, std::string name = "c_callback")
        : callback_(callback), user_data_(user_data), name_(std::move(name)) {}

    bool invoke(const ScriptEventData& data) override {
        if (callback_) {
            return callback_(&data, user_data_);
        }
        return true;
    }

    std::string get_name() const override { return name_; }

    bool is_valid() const override { return callback_ != nullptr; }

private:
    CEventCallback callback_;
    void* user_data_;
    std::string name_;
};

// ============================================================================
// Lambda/std::function Callback (for C++ inline use)
// ============================================================================

/**
 * @brief Callback type for C++ lambdas/functions
 */
using ScriptEventHandler = std::function<bool(const ScriptEventData&)>;

/**
 * @brief Wrapper for C++ lambda/std::function callbacks
 */
class LambdaCallback : public IScriptCallback {
public:
    explicit LambdaCallback(ScriptEventHandler handler, std::string name = "lambda_callback")
        : handler_(std::move(handler)), name_(std::move(name)) {}

    bool invoke(const ScriptEventData& data) override {
        if (handler_) {
            return handler_(data);
        }
        return true;
    }

    std::string get_name() const override { return name_; }

    bool is_valid() const override { return static_cast<bool>(handler_); }

private:
    ScriptEventHandler handler_;
    std::string name_;
};

// ============================================================================
// Script Callback Registry
// ============================================================================

/**
 * @brief Handle for script callback subscriptions
 */
using ScriptHandlerId = UInt64;

constexpr ScriptHandlerId INVALID_SCRIPT_HANDLER_ID = 0;

/**
 * @brief Registry for managing script event callbacks
 *
 * Provides a bridge between the native EventDispatcher and script callbacks.
 * Handles subscription lifecycle and automatic cleanup.
 */
class ScriptCallbackRegistry {
public:
    explicit ScriptCallbackRegistry(EventDispatcher& dispatcher)
        : dispatcher_(dispatcher) {}

    ~ScriptCallbackRegistry() {
        // Unsubscribe all handlers
        for (const auto& [script_id, native_id] : handler_map_) {
            dispatcher_.unsubscribe(native_id);
        }
    }

    // Non-copyable
    ScriptCallbackRegistry(const ScriptCallbackRegistry&) = delete;
    ScriptCallbackRegistry& operator=(const ScriptCallbackRegistry&) = delete;

    /**
     * @brief Subscribe a script callback to a specific event type
     *
     * @param type Event type to subscribe to
     * @param callback Script callback implementation
     * @param priority Handler priority (lower = called first)
     * @return Script handler ID for unsubscription
     */
    ScriptHandlerId subscribe(EventType type, ScriptCallbackPtr callback, int priority = 0) {
        if (!callback || !callback->is_valid()) {
            return INVALID_SCRIPT_HANDLER_ID;
        }

        ScriptHandlerId script_id = next_id_++;
        callbacks_[script_id] = callback;

        // Create a wrapper handler that converts Event to ScriptEventData
        EventHandlerId native_id = dispatcher_.subscribe(
            type,
            [this, script_id](Event& event) {
                return invoke_callback(script_id, event);
            },
            callback->get_name(),
            priority
        );

        handler_map_[script_id] = native_id;
        return script_id;
    }

    /**
     * @brief Subscribe to all events of a category
     */
    ScriptHandlerId subscribe_category(EventCategory category, ScriptCallbackPtr callback, int priority = 0) {
        if (!callback || !callback->is_valid()) {
            return INVALID_SCRIPT_HANDLER_ID;
        }

        ScriptHandlerId script_id = next_id_++;
        callbacks_[script_id] = callback;

        EventHandlerId native_id = dispatcher_.subscribe_category(
            category,
            [this, script_id](Event& event) {
                return invoke_callback(script_id, event);
            },
            callback->get_name(),
            priority
        );

        handler_map_[script_id] = native_id;
        return script_id;
    }

    /**
     * @brief Subscribe to all events
     */
    ScriptHandlerId subscribe_all(ScriptCallbackPtr callback, int priority = 0) {
        if (!callback || !callback->is_valid()) {
            return INVALID_SCRIPT_HANDLER_ID;
        }

        ScriptHandlerId script_id = next_id_++;
        callbacks_[script_id] = callback;

        EventHandlerId native_id = dispatcher_.subscribe_all(
            [this, script_id](Event& event) {
                return invoke_callback(script_id, event);
            },
            callback->get_name(),
            priority
        );

        handler_map_[script_id] = native_id;
        return script_id;
    }

    /**
     * @brief Unsubscribe a script callback
     *
     * @param id Script handler ID
     * @return true if handler was found and removed
     */
    bool unsubscribe(ScriptHandlerId id) {
        auto map_it = handler_map_.find(id);
        if (map_it == handler_map_.end()) {
            return false;
        }

        dispatcher_.unsubscribe(map_it->second);
        handler_map_.erase(map_it);
        callbacks_.erase(id);
        return true;
    }

    /**
     * @brief Get the number of registered callbacks
     */
    SizeT callback_count() const {
        return callbacks_.size();
    }

    /**
     * @brief Clean up invalid callbacks (e.g., script unloaded)
     *
     * @return Number of callbacks removed
     */
    SizeT cleanup_invalid() {
        SizeT removed = 0;
        std::vector<ScriptHandlerId> to_remove;

        for (const auto& [id, callback] : callbacks_) {
            if (!callback || !callback->is_valid()) {
                to_remove.push_back(id);
            }
        }

        for (ScriptHandlerId id : to_remove) {
            unsubscribe(id);
            removed++;
        }

        return removed;
    }

private:
    bool invoke_callback(ScriptHandlerId id, Event& event) {
        auto it = callbacks_.find(id);
        if (it == callbacks_.end() || !it->second || !it->second->is_valid()) {
            return true;  // Continue propagation if callback invalid
        }

        ScriptEventData data = to_script_data(event);
        return it->second->invoke(data);
    }

    EventDispatcher& dispatcher_;
    ScriptHandlerId next_id_{1};
    std::unordered_map<ScriptHandlerId, ScriptCallbackPtr> callbacks_;
    std::unordered_map<ScriptHandlerId, EventHandlerId> handler_map_;
};

// ============================================================================
// JSON-like String Serialization (for scripts that prefer strings)
// ============================================================================

/**
 * @brief Serialize ScriptEventData to a simple key=value string format
 *
 * Useful for scripts that have limited C++ interop and prefer
 * parsing string data.
 */
inline std::string serialize_event_data(const ScriptEventData& data) {
    std::ostringstream oss;
    oss << "type=" << static_cast<int>(data.type) << ";";
    oss << "category=" << static_cast<int>(data.category) << ";";
    oss << "source=" << data.source_entity << ";";
    oss << "timestamp=" << data.timestamp << ";";
    oss << "priority=" << static_cast<int>(data.priority) << ";";

    for (const auto& [key, value] : data.fields) {
        oss << key << "=";
        // Try common types
        if (value.type() == typeid(Real)) {
            oss << std::any_cast<Real>(value);
        } else if (value.type() == typeid(int)) {
            oss << std::any_cast<int>(value);
        } else if (value.type() == typeid(EntityId)) {
            oss << std::any_cast<EntityId>(value);
        } else if (value.type() == typeid(UInt64)) {
            oss << std::any_cast<UInt64>(value);
        } else if (value.type() == typeid(bool)) {
            oss << (std::any_cast<bool>(value) ? "true" : "false");
        } else if (value.type() == typeid(std::string)) {
            oss << "\"" << std::any_cast<std::string>(value) << "\"";
        } else {
            oss << "(unknown)";
        }
        oss << ";";
    }

    return oss.str();
}

// ============================================================================
// Convenience Functions for Script Binding Libraries
// ============================================================================

/**
 * @brief Create a lambda callback from a std::function
 */
inline ScriptCallbackPtr make_callback(ScriptEventHandler handler, const std::string& name = "script_callback") {
    return std::make_shared<LambdaCallback>(std::move(handler), name);
}

/**
 * @brief Create a C function callback
 */
inline ScriptCallbackPtr make_c_callback(CEventCallback callback, void* user_data, const std::string& name = "c_callback") {
    return std::make_shared<CFunctionCallback>(callback, user_data, name);
}

} // namespace jaguar::events
