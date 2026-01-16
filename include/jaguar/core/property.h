#pragma once
/**
 * @file property.h
 * @brief Hierarchical property system (JSBSim-inspired)
 */

#include "jaguar/core/types.h"
#include <string>
#include <unordered_map>
#include <functional>
#include <variant>

namespace jaguar::core {

/**
 * @brief Property value types
 */
using PropertyValue = std::variant<Real, Vec3, bool, std::string>;

/**
 * @brief Property node in the hierarchy
 */
struct PropertyNode {
    std::string name;
    PropertyValue value;
    std::unordered_map<std::string, std::unique_ptr<PropertyNode>> children;
};

/**
 * @brief Hierarchical property manager
 *
 * Provides JSBSim-style property access via paths like:
 * "aircraft/f16/aero/cl_alpha"
 */
class PropertyManager {
public:
    PropertyManager();
    ~PropertyManager();

    // Binding for direct memory access
    void bind(const std::string& path, Real* ptr);
    void bind(const std::string& path, Vec3* ptr);
    void bind(const std::string& path, bool* ptr);

    // Dynamic access
    Real get_real(const std::string& path) const;
    void set_real(const std::string& path, Real value);

    Vec3 get_vec3(const std::string& path) const;
    void set_vec3(const std::string& path, const Vec3& value);

    bool get_bool(const std::string& path) const;
    void set_bool(const std::string& path, bool value);

    // Fast access via pre-resolved ID
    PropertyId resolve_id(const std::string& path);
    Real get_by_id(PropertyId id) const;
    void set_by_id(PropertyId id, Real value);

    // Query
    bool exists(const std::string& path) const;
    std::vector<std::string> list_children(const std::string& path) const;

    // Observers
    using ChangeCallback = std::function<void(const std::string&, const PropertyValue&)>;
    void add_observer(const std::string& path, ChangeCallback callback);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace jaguar::core
