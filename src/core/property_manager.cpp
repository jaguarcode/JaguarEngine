/**
 * @file property_manager.cpp
 * @brief Property system implementation
 *
 * Provides JSBSim-style hierarchical property access:
 * - Bound properties (pointer-based, zero-copy access)
 * - Dynamic properties (stored internally)
 * - Observer pattern for change notifications
 * - Fast path ID resolution for high-frequency access
 */

#include "jaguar/core/property.h"
#include <unordered_map>
#include <unordered_set>
#include <algorithm>

namespace jaguar::core {

// ============================================================================
// PropertyManager Implementation
// ============================================================================

struct PropertyManager::Impl {
    // Bound properties (pointer-based access)
    std::unordered_map<std::string, Real*> real_bindings;
    std::unordered_map<std::string, Vec3*> vec3_bindings;
    std::unordered_map<std::string, bool*> bool_bindings;

    // Dynamic properties (stored internally when not bound)
    std::unordered_map<std::string, Real> real_values;
    std::unordered_map<std::string, Vec3> vec3_values;
    std::unordered_map<std::string, bool> bool_values;
    std::unordered_map<std::string, std::string> string_values;

    // Fast ID-based access for Real properties
    std::unordered_map<std::string, PropertyId> path_to_id;
    std::vector<Real*> id_to_real;
    std::vector<Real*> id_to_dynamic_real;  // For dynamic values
    PropertyId next_id{0};

    // Observer pattern
    std::unordered_map<std::string, std::vector<PropertyManager::ChangeCallback>> observers;

    // All registered paths for hierarchy traversal
    std::unordered_set<std::string> all_paths;

    // Helper to extract parent path
    static std::string get_parent_path(const std::string& path) {
        auto pos = path.rfind('/');
        if (pos == std::string::npos) {
            return "";
        }
        return path.substr(0, pos);
    }

    // Helper to extract the last segment of a path
    static std::string get_leaf_name(const std::string& path) {
        auto pos = path.rfind('/');
        if (pos == std::string::npos) {
            return path;
        }
        return path.substr(pos + 1);
    }

    // Helper to check if child is direct child of parent
    static bool is_direct_child(const std::string& parent, const std::string& child) {
        if (parent.empty()) {
            // Root level - check if child has no slashes
            return child.find('/') == std::string::npos;
        }
        // Check prefix and one more segment
        if (child.length() <= parent.length() + 1) return false;
        if (child.substr(0, parent.length()) != parent) return false;
        if (child[parent.length()] != '/') return false;

        // Check no more slashes after parent/
        std::string remainder = child.substr(parent.length() + 1);
        return remainder.find('/') == std::string::npos;
    }

    // Notify observers of a property change
    void notify_observers(const std::string& path, const PropertyValue& value) {
        auto it = observers.find(path);
        if (it != observers.end()) {
            for (auto& callback : it->second) {
                callback(path, value);
            }
        }
    }

    // Register path in hierarchy
    void register_path(const std::string& path) {
        all_paths.insert(path);
    }
};

PropertyManager::PropertyManager() : impl_(std::make_unique<Impl>()) {}
PropertyManager::~PropertyManager() = default;

// ============================================================================
// Binding Methods
// ============================================================================

void PropertyManager::bind(const std::string& path, Real* ptr) {
    impl_->real_bindings[path] = ptr;
    impl_->register_path(path);
}

void PropertyManager::bind(const std::string& path, Vec3* ptr) {
    impl_->vec3_bindings[path] = ptr;
    impl_->register_path(path);
}

void PropertyManager::bind(const std::string& path, bool* ptr) {
    impl_->bool_bindings[path] = ptr;
    impl_->register_path(path);
}

// ============================================================================
// Real Property Access
// ============================================================================

Real PropertyManager::get_real(const std::string& path) const {
    // Check bound properties first (zero-copy)
    auto it = impl_->real_bindings.find(path);
    if (it != impl_->real_bindings.end() && it->second) {
        return *it->second;
    }

    // Check dynamic properties
    auto dyn_it = impl_->real_values.find(path);
    if (dyn_it != impl_->real_values.end()) {
        return dyn_it->second;
    }

    return 0.0;
}

void PropertyManager::set_real(const std::string& path, Real value) {
    bool changed = false;
    Real old_value = 0.0;

    // Check bound properties first
    auto it = impl_->real_bindings.find(path);
    if (it != impl_->real_bindings.end() && it->second) {
        old_value = *it->second;
        *it->second = value;
        changed = (old_value != value);
    } else {
        // Store as dynamic property
        auto dyn_it = impl_->real_values.find(path);
        if (dyn_it != impl_->real_values.end()) {
            old_value = dyn_it->second;
            dyn_it->second = value;
            changed = (old_value != value);
        } else {
            impl_->real_values[path] = value;
            impl_->register_path(path);
            changed = true;
        }
    }

    // Notify observers if value changed
    if (changed) {
        impl_->notify_observers(path, PropertyValue{value});
    }
}

// ============================================================================
// Vec3 Property Access
// ============================================================================

Vec3 PropertyManager::get_vec3(const std::string& path) const {
    // Check bound properties first
    auto it = impl_->vec3_bindings.find(path);
    if (it != impl_->vec3_bindings.end() && it->second) {
        return *it->second;
    }

    // Check dynamic properties
    auto dyn_it = impl_->vec3_values.find(path);
    if (dyn_it != impl_->vec3_values.end()) {
        return dyn_it->second;
    }

    return Vec3{0, 0, 0};
}

void PropertyManager::set_vec3(const std::string& path, const Vec3& value) {
    bool changed = false;

    // Check bound properties first
    auto it = impl_->vec3_bindings.find(path);
    if (it != impl_->vec3_bindings.end() && it->second) {
        Vec3 old_value = *it->second;
        *it->second = value;
        changed = (old_value.x != value.x || old_value.y != value.y || old_value.z != value.z);
    } else {
        // Store as dynamic property
        auto dyn_it = impl_->vec3_values.find(path);
        if (dyn_it != impl_->vec3_values.end()) {
            Vec3 old_value = dyn_it->second;
            dyn_it->second = value;
            changed = (old_value.x != value.x || old_value.y != value.y || old_value.z != value.z);
        } else {
            impl_->vec3_values[path] = value;
            impl_->register_path(path);
            changed = true;
        }
    }

    // Notify observers if value changed
    if (changed) {
        impl_->notify_observers(path, PropertyValue{value});
    }
}

// ============================================================================
// Bool Property Access
// ============================================================================

bool PropertyManager::get_bool(const std::string& path) const {
    // Check bound properties first
    auto it = impl_->bool_bindings.find(path);
    if (it != impl_->bool_bindings.end() && it->second) {
        return *it->second;
    }

    // Check dynamic properties
    auto dyn_it = impl_->bool_values.find(path);
    if (dyn_it != impl_->bool_values.end()) {
        return dyn_it->second;
    }

    return false;
}

void PropertyManager::set_bool(const std::string& path, bool value) {
    bool changed = false;

    // Check bound properties first
    auto it = impl_->bool_bindings.find(path);
    if (it != impl_->bool_bindings.end() && it->second) {
        bool old_value = *it->second;
        *it->second = value;
        changed = (old_value != value);
    } else {
        // Store as dynamic property
        auto dyn_it = impl_->bool_values.find(path);
        if (dyn_it != impl_->bool_values.end()) {
            bool old_value = dyn_it->second;
            dyn_it->second = value;
            changed = (old_value != value);
        } else {
            impl_->bool_values[path] = value;
            impl_->register_path(path);
            changed = true;
        }
    }

    // Notify observers if value changed
    if (changed) {
        impl_->notify_observers(path, PropertyValue{value});
    }
}

// ============================================================================
// Fast ID-Based Access
// ============================================================================

PropertyId PropertyManager::resolve_id(const std::string& path) {
    // Check if already resolved
    auto it = impl_->path_to_id.find(path);
    if (it != impl_->path_to_id.end()) {
        return it->second;
    }

    // Try to resolve bound Real property
    auto real_it = impl_->real_bindings.find(path);
    if (real_it != impl_->real_bindings.end() && real_it->second) {
        PropertyId id = impl_->next_id++;
        impl_->path_to_id[path] = id;
        impl_->id_to_real.resize(id + 1, nullptr);
        impl_->id_to_real[id] = real_it->second;
        return id;
    }

    // Try to resolve dynamic Real property
    auto dyn_it = impl_->real_values.find(path);
    if (dyn_it != impl_->real_values.end()) {
        PropertyId id = impl_->next_id++;
        impl_->path_to_id[path] = id;
        impl_->id_to_real.resize(id + 1, nullptr);
        impl_->id_to_real[id] = &dyn_it->second;
        return id;
    }

    return INVALID_PROPERTY_ID;
}

Real PropertyManager::get_by_id(PropertyId id) const {
    if (id < impl_->id_to_real.size() && impl_->id_to_real[id]) {
        return *impl_->id_to_real[id];
    }
    return 0.0;
}

void PropertyManager::set_by_id(PropertyId id, Real value) {
    if (id < impl_->id_to_real.size() && impl_->id_to_real[id]) {
        *impl_->id_to_real[id] = value;
        // Note: Observer notification not supported for ID-based access
        // Use path-based set_real() if observers are needed
    }
}

// ============================================================================
// Query Methods
// ============================================================================

bool PropertyManager::exists(const std::string& path) const {
    return impl_->real_bindings.count(path) > 0 ||
           impl_->vec3_bindings.count(path) > 0 ||
           impl_->bool_bindings.count(path) > 0 ||
           impl_->real_values.count(path) > 0 ||
           impl_->vec3_values.count(path) > 0 ||
           impl_->bool_values.count(path) > 0 ||
           impl_->string_values.count(path) > 0;
}

std::vector<std::string> PropertyManager::list_children(const std::string& path) const {
    std::vector<std::string> children;
    std::unordered_set<std::string> seen;

    for (const auto& registered_path : impl_->all_paths) {
        if (Impl::is_direct_child(path, registered_path)) {
            std::string leaf = Impl::get_leaf_name(registered_path);
            if (seen.find(leaf) == seen.end()) {
                children.push_back(leaf);
                seen.insert(leaf);
            }
        }
    }

    // Sort for consistent ordering
    std::sort(children.begin(), children.end());
    return children;
}

// ============================================================================
// Observer Pattern
// ============================================================================

void PropertyManager::add_observer(const std::string& path, ChangeCallback callback) {
    impl_->observers[path].push_back(std::move(callback));
}

} // namespace jaguar::core
