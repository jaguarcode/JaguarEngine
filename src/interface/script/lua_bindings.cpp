/**
 * @file lua_bindings.cpp
 * @brief Lua bindings for JaguarEngine using sol2
 *
 * Provides Lua scripting interface for:
 * - Math types (Vec3, Quat, Mat3x3)
 * - Engine lifecycle and simulation control
 * - Entity management and state manipulation
 * - Physical constants and unit conversions
 * - Domain-specific functionality
 *
 * Build with: cmake -DJAGUAR_BUILD_LUA=ON ..
 * Requires: sol2 (header-only), Lua 5.4+
 */

#include "jaguar/core/types.h"
#include "jaguar/interface/api.h"
#include "jaguar/physics/entity.h"
#include "jaguar/sensors/sensor.h"
#include "jaguar/sensors/imu_sensor.h"
#include "jaguar/environment/atmospheric_disturbance.h"

#ifdef JAGUAR_BUILD_LUA

#include <sol/sol.hpp>
#include <memory>
#include <string>
#include <cmath>

namespace jaguar::lua {

using namespace jaguar;
using namespace jaguar::physics;
using namespace jaguar::sensors;
using namespace jaguar::environment;

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * @brief Convert Vec3 to Lua table {x, y, z}
 */
sol::table vec3_to_table(sol::state_view lua, const Vec3& v) {
    sol::table t = lua.create_table();
    t["x"] = v.x;
    t["y"] = v.y;
    t["z"] = v.z;
    return t;
}

/**
 * @brief Convert Lua table {x, y, z} or {1, 2, 3} to Vec3
 */
Vec3 table_to_vec3(const sol::table& t) {
    Vec3 v;
    // Support both named keys and array indices
    if (t["x"].valid()) {
        v.x = t["x"].get<Real>();
        v.y = t["y"].get<Real>();
        v.z = t["z"].get<Real>();
    } else {
        v.x = t[1].get<Real>();
        v.y = t[2].get<Real>();
        v.z = t[3].get<Real>();
    }
    return v;
}

/**
 * @brief Convert Quat to Lua table {w, x, y, z}
 */
sol::table quat_to_table(sol::state_view lua, const Quat& q) {
    sol::table t = lua.create_table();
    t["w"] = q.w;
    t["x"] = q.x;
    t["y"] = q.y;
    t["z"] = q.z;
    return t;
}

/**
 * @brief Convert Lua table to Quat
 */
Quat table_to_quat(const sol::table& t) {
    Quat q;
    q.w = t["w"].get<Real>();
    q.x = t["x"].get<Real>();
    q.y = t["y"].get<Real>();
    q.z = t["z"].get<Real>();
    return q;
}

// ============================================================================
// Vec3 Bindings
// ============================================================================

void bind_vec3(sol::state_view& lua) {
    lua.new_usertype<Vec3>("Vec3",
        // Constructors - both .new() and direct call () syntax
        sol::constructors<
            Vec3(),
            Vec3(Real, Real, Real)
        >(),
        // Enable Vec3(x, y, z) call syntax in addition to Vec3.new(x, y, z)
        sol::call_constructor, sol::constructors<Vec3(), Vec3(Real, Real, Real)>(),

        // Properties (read/write)
        "x", &Vec3::x,
        "y", &Vec3::y,
        "z", &Vec3::z,

        // Operators
        sol::meta_function::addition, [](const Vec3& a, const Vec3& b) {
            return Vec3{a.x + b.x, a.y + b.y, a.z + b.z};
        },
        sol::meta_function::subtraction, [](const Vec3& a, const Vec3& b) {
            return Vec3{a.x - b.x, a.y - b.y, a.z - b.z};
        },
        sol::meta_function::multiplication, sol::overload(
            [](const Vec3& v, Real s) { return Vec3{v.x * s, v.y * s, v.z * s}; },
            [](Real s, const Vec3& v) { return Vec3{v.x * s, v.y * s, v.z * s}; }
        ),
        sol::meta_function::division, [](const Vec3& v, Real s) {
            return Vec3{v.x / s, v.y / s, v.z / s};
        },
        sol::meta_function::unary_minus, [](const Vec3& v) {
            return Vec3{-v.x, -v.y, -v.z};
        },
        sol::meta_function::to_string, [](const Vec3& v) {
            return "Vec3(" + std::to_string(v.x) + ", " +
                   std::to_string(v.y) + ", " + std::to_string(v.z) + ")";
        },
        sol::meta_function::equal_to, [](const Vec3& a, const Vec3& b) {
            return a.x == b.x && a.y == b.y && a.z == b.z;
        },

        // Methods
        "dot", &Vec3::dot,
        "cross", &Vec3::cross,
        "length", &Vec3::length,
        "length_squared", &Vec3::length_squared,
        "normalized", &Vec3::normalized,

        // Static factory methods
        "zero", sol::factories([]() { return Vec3::Zero(); }),
        "unit_x", sol::factories([]() { return Vec3::UnitX(); }),
        "unit_y", sol::factories([]() { return Vec3::UnitY(); }),
        "unit_z", sol::factories([]() { return Vec3::UnitZ(); }),

        // Table conversion
        "to_table", [](const Vec3& v, sol::this_state L) {
            return vec3_to_table(L, v);
        },
        "from_table", [](const sol::table& t) {
            return table_to_vec3(t);
        }
    );

    // Global Vec3 factory function
    lua["vec3"] = [](Real x, Real y, Real z) { return Vec3{x, y, z}; };
}

// ============================================================================
// Quat Bindings
// ============================================================================

void bind_quat(sol::state_view& lua) {
    lua.new_usertype<Quat>("Quat",
        // Constructors
        sol::constructors<
            Quat(),
            Quat(Real, Real, Real, Real)
        >(),
        sol::call_constructor, sol::constructors<Quat(), Quat(Real, Real, Real, Real)>(),

        // Properties
        "w", &Quat::w,
        "x", &Quat::x,
        "y", &Quat::y,
        "z", &Quat::z,

        // Operators
        sol::meta_function::multiplication, sol::overload(
            [](const Quat& q1, const Quat& q2) { return q1 * q2; },
            [](const Quat& q, const Vec3& v) { return q.rotate(v); }
        ),
        sol::meta_function::to_string, [](const Quat& q) {
            return "Quat(" + std::to_string(q.w) + ", " +
                   std::to_string(q.x) + ", " + std::to_string(q.y) + ", " +
                   std::to_string(q.z) + ")";
        },

        // Methods
        "rotate", &Quat::rotate,
        "conjugate", &Quat::conjugate,
        "inverse", &Quat::inverse,
        "normalized", &Quat::normalized,
        "norm", &Quat::norm,
        "norm_squared", &Quat::norm_squared,
        "to_euler", [](const Quat& q) {
            // Returns roll, pitch, yaw as a tuple
            Real roll, pitch, yaw;
            q.to_euler(roll, pitch, yaw);
            return std::make_tuple(roll, pitch, yaw);
        },
        "to_rotation_matrix", &Quat::to_rotation_matrix,

        // Static factory methods
        "identity", sol::factories([]() { return Quat::Identity(); }),
        "from_euler", [](Real roll, Real pitch, Real yaw) {
            return Quat::from_euler(roll, pitch, yaw);
        },
        "from_axis_angle", [](const Vec3& axis, Real angle) {
            return Quat::from_axis_angle(axis, angle);
        },

        // Table conversion
        "to_table", [](const Quat& q, sol::this_state L) {
            return quat_to_table(L, q);
        },
        "from_table", [](const sol::table& t) {
            return table_to_quat(t);
        }
    );

    // Global Quat factory
    lua["quat"] = sol::overload(
        []() { return Quat::Identity(); },
        [](Real w, Real x, Real y, Real z) { return Quat{w, x, y, z}; }
    );

    // Convenient euler factory
    lua["quat_euler"] = [](Real roll, Real pitch, Real yaw) {
        return Quat::from_euler(roll, pitch, yaw);
    };
}

// ============================================================================
// Mat3x3 Bindings
// ============================================================================

void bind_mat3x3(sol::state_view& lua) {
    lua.new_usertype<Mat3x3>("Mat3x3",
        // Constructors
        sol::constructors<Mat3x3()>(),
        sol::call_constructor, sol::constructors<Mat3x3()>(),

        // Element access
        "get", [](const Mat3x3& m, int row, int col) {
            return m(static_cast<SizeT>(row), static_cast<SizeT>(col));
        },
        "set", [](Mat3x3& m, int row, int col, Real val) {
            m(static_cast<SizeT>(row), static_cast<SizeT>(col)) = val;
        },

        // Operators
        sol::meta_function::multiplication, sol::overload(
            [](const Mat3x3& m, const Vec3& v) { return m * v; },
            [](const Mat3x3& m, Real s) { return m * s; }
        ),
        sol::meta_function::addition, [](const Mat3x3& a, const Mat3x3& b) {
            return a + b;
        },
        sol::meta_function::to_string, [](const Mat3x3& m) {
            std::string s = "Mat3x3[\n";
            for (int i = 0; i < 3; ++i) {
                s += "  [";
                for (int j = 0; j < 3; ++j) {
                    s += std::to_string(m(static_cast<SizeT>(i), static_cast<SizeT>(j)));
                    if (j < 2) s += ", ";
                }
                s += "]\n";
            }
            s += "]";
            return s;
        },

        // Methods
        "transpose", &Mat3x3::transpose,
        "determinant", &Mat3x3::determinant,
        "inverse", &Mat3x3::inverse,
        "trace", [](const Mat3x3& m) {
            return m(0, 0) + m(1, 1) + m(2, 2);
        },

        // Static factory methods
        "identity", sol::factories([]() { return Mat3x3::Identity(); }),
        "zero", sol::factories([]() { return Mat3x3::Zero(); }),
        "diagonal", [](Real a, Real b, Real c) {
            return Mat3x3::Diagonal(a, b, c);
        },
        "inertia", sol::overload(
            [](Real ixx, Real iyy, Real izz) {
                return Mat3x3::Inertia(ixx, iyy, izz);
            },
            [](Real ixx, Real iyy, Real izz, Real ixy, Real ixz, Real iyz) {
                return Mat3x3::Inertia(ixx, iyy, izz, ixy, ixz, iyz);
            }
        )
    );
}

// ============================================================================
// Enum Bindings
// ============================================================================

void bind_enums(sol::state_view& lua) {
    // Domain enum
    lua.new_enum<Domain>("Domain",
        {
            {"Air", Domain::Air},
            {"Land", Domain::Land},
            {"Sea", Domain::Sea},
            {"Space", Domain::Space},
            {"Generic", Domain::Generic}
        }
    );

    // CoordinateFrame enum
    lua.new_enum<CoordinateFrame>("CoordinateFrame",
        {
            {"ECEF", CoordinateFrame::ECEF},
            {"ECI", CoordinateFrame::ECI},
            {"NED", CoordinateFrame::NED},
            {"ENU", CoordinateFrame::ENU},
            {"Body", CoordinateFrame::Body}
        }
    );

    // SimulationState enum
    lua.new_enum<SimulationState>("SimulationState",
        {
            {"Uninitialized", SimulationState::Uninitialized},
            {"Initialized", SimulationState::Initialized},
            {"Running", SimulationState::Running},
            {"Paused", SimulationState::Paused},
            {"Stopped", SimulationState::Stopped},
            {"Error", SimulationState::Error}
        }
    );

    // SensorType enum
    lua.new_enum<SensorType>("SensorType",
        {
            {"IMU", SensorType::IMU},
            {"GPS", SensorType::GPS},
            {"Altimeter", SensorType::Altimeter},
            {"Airspeed", SensorType::Airspeed},
            {"Magnetometer", SensorType::Magnetometer},
            {"Lidar", SensorType::Lidar},
            {"Radar", SensorType::Radar},
            {"Camera", SensorType::Camera},
            {"Infrared", SensorType::Infrared},
            {"Sonar", SensorType::Sonar},
            {"DepthGauge", SensorType::DepthGauge},
            {"Custom", SensorType::Custom}
        }
    );

    // SensorState enum
    lua.new_enum<SensorState>("SensorState",
        {
            {"Uninitialized", SensorState::Uninitialized},
            {"Initializing", SensorState::Initializing},
            {"Operational", SensorState::Operational},
            {"Degraded", SensorState::Degraded},
            {"Failed", SensorState::Failed},
            {"Disabled", SensorState::Disabled}
        }
    );

    // FailureMode enum
    lua.new_enum<FailureMode>("FailureMode",
        {
            {"None", FailureMode::None},
            {"Bias", FailureMode::Bias},
            {"Drift", FailureMode::Drift},
            {"Noise", FailureMode::Noise},
            {"Saturation", FailureMode::Saturation},
            {"Dropout", FailureMode::Dropout},
            {"Stuck", FailureMode::Stuck},
            {"Oscillation", FailureMode::Oscillation},
            {"Complete", FailureMode::Complete}
        }
    );

    // IMUGrade enum
    lua.new_enum<IMUGrade>("IMUGrade",
        {
            {"Consumer", IMUGrade::Consumer},
            {"Industrial", IMUGrade::Industrial},
            {"Tactical", IMUGrade::Tactical},
            {"Navigation", IMUGrade::Navigation},
            {"Strategic", IMUGrade::Strategic},
            {"Custom", IMUGrade::Custom}
        }
    );

    // TurbulenceSeverity enum
    lua.new_enum<TurbulenceSeverity>("TurbulenceSeverity",
        {
            {"None", TurbulenceSeverity::None},
            {"Light", TurbulenceSeverity::Light},
            {"Moderate", TurbulenceSeverity::Moderate},
            {"Severe", TurbulenceSeverity::Severe},
            {"Extreme", TurbulenceSeverity::Extreme}
        }
    );
}

// ============================================================================
// EntityState Bindings
// ============================================================================

void bind_entity_state(sol::state_view& lua) {
    lua.new_usertype<EntityState>("EntityState",
        sol::constructors<EntityState()>(),
        sol::call_constructor, sol::constructors<EntityState()>(),

        // Position and orientation
        "position", &EntityState::position,
        "velocity", &EntityState::velocity,
        "acceleration", &EntityState::acceleration,
        "orientation", &EntityState::orientation,
        "angular_velocity", &EntityState::angular_velocity,
        "angular_accel", &EntityState::angular_accel,

        // Mass properties
        "mass", &EntityState::mass,
        "inertia", &EntityState::inertia,

        // String representation
        sol::meta_function::to_string, [](const EntityState& s) {
            return "EntityState(pos=(" + std::to_string(s.position.x) + ", " +
                   std::to_string(s.position.y) + ", " + std::to_string(s.position.z) +
                   "), mass=" + std::to_string(s.mass) + ")";
        }
    );
}

// ============================================================================
// EntityForces Bindings
// ============================================================================

void bind_entity_forces(sol::state_view& lua) {
    lua.new_usertype<EntityForces>("EntityForces",
        sol::constructors<EntityForces()>(),
        sol::call_constructor, sol::constructors<EntityForces()>(),

        // Force components
        "force", &EntityForces::force,
        "torque", &EntityForces::torque,

        // Methods
        "clear", &EntityForces::clear,
        "add_force", &EntityForces::add_force,
        "add_torque", &EntityForces::add_torque,
        "add_force_at_point", &EntityForces::add_force_at_point,

        // String representation
        sol::meta_function::to_string, [](const EntityForces& f) {
            return "EntityForces(force=(" + std::to_string(f.force.x) + ", " +
                   std::to_string(f.force.y) + ", " + std::to_string(f.force.z) + "))";
        }
    );
}

// ============================================================================
// Entity Bindings
// ============================================================================

void bind_entity(sol::state_view& lua) {
    lua.new_usertype<Entity>("Entity",
        // No public constructor - created by Engine
        sol::no_constructor,

        // Properties
        "id", sol::readonly(&Entity::id),
        "state_index", sol::readonly(&Entity::state_index),
        "components", sol::readonly(&Entity::components)
    );
}

// ============================================================================
// Sensor Bindings
// ============================================================================

void bind_sensors(sol::state_view& lua) {
    // Vec3Measurement binding
    lua.new_usertype<Vec3Measurement>("Vec3Measurement",
        sol::constructors<Vec3Measurement()>(),
        sol::call_constructor, sol::constructors<Vec3Measurement()>(),

        // Properties
        "value", &Vec3Measurement::value,
        "timestamp", &Vec3Measurement::timestamp,
        "measurement_time", &Vec3Measurement::measurement_time,
        "valid", &Vec3Measurement::valid,
        "confidence", &Vec3Measurement::confidence,
        "sequence_number", &Vec3Measurement::sequence_number,

        // String representation
        sol::meta_function::to_string, [](const Vec3Measurement& m) {
            return "Vec3Measurement(valid=" + std::string(m.valid ? "true" : "false") +
                   ", value=(" + std::to_string(m.value.x) + ", " +
                   std::to_string(m.value.y) + ", " + std::to_string(m.value.z) + "))";
        }
    );

    // IMUSensor binding
    lua.new_usertype<IMUSensor>("IMUSensor",
        // Constructor with grade preset
        sol::constructors<
            IMUSensor(const std::string&, IMUGrade, EntityId)
        >(),

        // Measurement access
        "get_acceleration", &IMUSensor::get_acceleration,
        "get_angular_velocity", &IMUSensor::get_angular_velocity,
        "get_true_acceleration", &IMUSensor::get_true_acceleration,
        "get_true_angular_velocity", &IMUSensor::get_true_angular_velocity,

        // Integrated outputs
        "get_delta_v", &IMUSensor::get_delta_v,
        "get_delta_theta", &IMUSensor::get_delta_theta,

        // Bias access
        "get_accel_bias", &IMUSensor::get_accel_bias,
        "get_gyro_bias", &IMUSensor::get_gyro_bias,
        "set_accel_bias", &IMUSensor::set_accel_bias,
        "set_gyro_bias", &IMUSensor::set_gyro_bias,

        // Temperature
        "set_temperature", &IMUSensor::set_temperature,
        "get_temperature", &IMUSensor::get_temperature,

        // ISensor interface methods
        "initialize", &IMUSensor::initialize,
        "reset", &IMUSensor::reset,
        "is_operational", &IMUSensor::is_operational,
        "get_state", &IMUSensor::get_state,
        "set_enabled", &IMUSensor::set_enabled,
        "is_enabled", &IMUSensor::is_enabled,

        // Failure injection
        "inject_failure", &IMUSensor::inject_failure,
        "clear_failures", &IMUSensor::clear_failures,
        "get_failure_mode", &IMUSensor::get_failure_mode,

        // String representation
        sol::meta_function::to_string, [](const IMUSensor& s) {
            return "IMUSensor(state=" + std::string(
                s.get_state() == SensorState::Operational ? "Operational" :
                s.get_state() == SensorState::Failed ? "Failed" : "Other") + ")";
        }
    );

    // Factory functions
    lua["create_consumer_imu"] = [](const std::string& name, EntityId entity) {
        return create_consumer_imu(name, entity);
    };
    lua["create_tactical_imu"] = [](const std::string& name, EntityId entity) {
        return create_tactical_imu(name, entity);
    };
    lua["create_navigation_imu"] = [](const std::string& name, EntityId entity) {
        return create_navigation_imu(name, entity);
    };
}

// ============================================================================
// Turbulence Bindings
// ============================================================================

void bind_turbulence(sol::state_view& lua) {
    // TurbulenceOutput binding
    lua.new_usertype<TurbulenceOutput>("TurbulenceOutput",
        sol::constructors<TurbulenceOutput()>(),
        sol::call_constructor, sol::constructors<TurbulenceOutput()>(),

        // Properties
        "velocity", &TurbulenceOutput::velocity,
        "angular_rate", &TurbulenceOutput::angular_rate,

        // Methods
        "is_negligible", &TurbulenceOutput::is_negligible,

        // String representation
        sol::meta_function::to_string, [](const TurbulenceOutput& t) {
            return "TurbulenceOutput(vel=(" + std::to_string(t.velocity.x) + ", " +
                   std::to_string(t.velocity.y) + ", " + std::to_string(t.velocity.z) + "))";
        }
    );

    // DrydenTurbulenceModel binding
    lua.new_usertype<DrydenTurbulenceModel>("DrydenTurbulenceModel",
        sol::constructors<
            DrydenTurbulenceModel(),
            DrydenTurbulenceModel(TurbulenceSeverity)
        >(),
        sol::call_constructor, sol::constructors<
            DrydenTurbulenceModel(),
            DrydenTurbulenceModel(TurbulenceSeverity)
        >(),

        // ITurbulenceModel interface
        "update", &DrydenTurbulenceModel::update,
        "reset", &DrydenTurbulenceModel::reset,
        "set_severity", &DrydenTurbulenceModel::set_severity,
        "severity", &DrydenTurbulenceModel::severity,
        "name", &DrydenTurbulenceModel::name,
        "set_enabled", &DrydenTurbulenceModel::set_enabled,
        "is_enabled", &DrydenTurbulenceModel::is_enabled,
        "set_seed", &DrydenTurbulenceModel::set_seed,

        // Dryden-specific methods
        "set_intensity", &DrydenTurbulenceModel::set_intensity,
        "set_scale_lengths", &DrydenTurbulenceModel::set_scale_lengths,
        "set_altitude_dependent", &DrydenTurbulenceModel::set_altitude_dependent,
        "get_intensity", &DrydenTurbulenceModel::get_intensity,
        "get_scale_lengths", &DrydenTurbulenceModel::get_scale_lengths,
        "last_output", &DrydenTurbulenceModel::last_output,

        // String representation
        sol::meta_function::to_string, [](const DrydenTurbulenceModel& m) {
            return "DrydenTurbulenceModel(severity=" + std::to_string(static_cast<int>(m.severity())) + ")";
        }
    );

    // VonKarmanTurbulenceModel binding
    lua.new_usertype<VonKarmanTurbulenceModel>("VonKarmanTurbulenceModel",
        sol::constructors<
            VonKarmanTurbulenceModel(),
            VonKarmanTurbulenceModel(TurbulenceSeverity)
        >(),
        sol::call_constructor, sol::constructors<
            VonKarmanTurbulenceModel(),
            VonKarmanTurbulenceModel(TurbulenceSeverity)
        >(),

        // ITurbulenceModel interface
        "update", &VonKarmanTurbulenceModel::update,
        "reset", &VonKarmanTurbulenceModel::reset,
        "set_severity", &VonKarmanTurbulenceModel::set_severity,
        "severity", &VonKarmanTurbulenceModel::severity,
        "name", &VonKarmanTurbulenceModel::name,
        "set_enabled", &VonKarmanTurbulenceModel::set_enabled,
        "is_enabled", &VonKarmanTurbulenceModel::is_enabled,
        "set_seed", &VonKarmanTurbulenceModel::set_seed,

        // Von Karman-specific methods
        "set_filter_order", &VonKarmanTurbulenceModel::set_filter_order,
        "filter_order", &VonKarmanTurbulenceModel::filter_order,
        "set_intensity", &VonKarmanTurbulenceModel::set_intensity,
        "set_scale_lengths", &VonKarmanTurbulenceModel::set_scale_lengths,

        // String representation
        sol::meta_function::to_string, [](const VonKarmanTurbulenceModel& m) {
            return "VonKarmanTurbulenceModel(severity=" + std::to_string(static_cast<int>(m.severity())) + ")";
        }
    );

    // Factory functions
    lua["create_dryden_turbulence"] = sol::overload(
        []() { return create_dryden_turbulence(); },
        [](TurbulenceSeverity severity) { return create_dryden_turbulence(severity); }
    );
    lua["create_vonkarman_turbulence"] = sol::overload(
        []() { return create_vonkarman_turbulence(); },
        [](TurbulenceSeverity severity) { return create_vonkarman_turbulence(severity); }
    );

    // Utility functions
    lua["get_turbulence_intensity"] = &get_turbulence_intensity;
    lua["get_scale_length"] = &get_scale_length;
}

// ============================================================================
// Engine Bindings
// ============================================================================

void bind_engine(sol::state_view& lua) {
    lua.new_usertype<Engine>("Engine",
        sol::constructors<Engine()>(),
        sol::call_constructor, sol::constructors<Engine()>(),

        // Lifecycle
        "initialize", sol::overload(
            static_cast<bool(Engine::*)()>(&Engine::initialize),
            static_cast<bool(Engine::*)(const std::string&)>(&Engine::initialize)
        ),
        "shutdown", &Engine::shutdown,
        "is_initialized", &Engine::is_initialized,

        // Simulation control
        "step", &Engine::step,
        "run", sol::overload(
            static_cast<void(Engine::*)()>(&Engine::run),
            static_cast<void(Engine::*)(Real)>(&Engine::run_for)
        ),
        "run_for", &Engine::run_for,
        "pause", &Engine::pause,
        "resume", &Engine::resume,
        "stop", &Engine::stop,

        // State queries
        "get_state", &Engine::get_state,
        "get_time", &Engine::get_time,
        "get_time_scale", &Engine::get_time_scale,
        "set_time_scale", &Engine::set_time_scale,
        "set_fixed_time_step", &Engine::set_fixed_time_step,

        // Entity management
        "create_entity", sol::overload(
            static_cast<EntityId(Engine::*)(const std::string&)>(&Engine::create_entity),
            static_cast<EntityId(Engine::*)(const std::string&, Domain)>(&Engine::create_entity),
            [](Engine& e, const std::string& name, const std::string& domain_str) {
                Domain d = Domain::Generic;
                if (domain_str == "Air" || domain_str == "air") d = Domain::Air;
                else if (domain_str == "Land" || domain_str == "land") d = Domain::Land;
                else if (domain_str == "Sea" || domain_str == "sea") d = Domain::Sea;
                else if (domain_str == "Space" || domain_str == "space") d = Domain::Space;
                return e.create_entity(name, d);
            }
        ),
        "destroy_entity", &Engine::destroy_entity,
        "entity_exists", &Engine::entity_exists,
        "get_entity_state", &Engine::get_entity_state,
        "set_entity_state", &Engine::set_entity_state,

        // Property access
        "get_property", sol::overload(
            static_cast<Real(Engine::*)(const std::string&) const>(&Engine::get_property),
            static_cast<Real(Engine::*)(EntityId, const std::string&) const>(&Engine::get_property)
        ),
        "set_property", sol::overload(
            static_cast<void(Engine::*)(const std::string&, Real)>(&Engine::set_property),
            static_cast<void(Engine::*)(EntityId, const std::string&, Real)>(&Engine::set_property)
        ),

        // String representation
        sol::meta_function::to_string, [](const Engine& e) {
            return "Engine(state=" + std::to_string(static_cast<int>(e.get_state())) +
                   ", time=" + std::to_string(e.get_time()) + ")";
        }
    );
}

// ============================================================================
// Constants Module
// ============================================================================

void bind_constants(sol::state_view& lua) {
    sol::table constants_tbl = lua.create_named_table("constants");

    // Universal constants
    constants_tbl["G"] = constants::G;                        // Gravitational constant
    constants_tbl["C"] = constants::C;                        // Speed of light
    constants_tbl["PI"] = 3.14159265358979323846;
    constants_tbl["TWO_PI"] = 2.0 * 3.14159265358979323846;
    constants_tbl["HALF_PI"] = 3.14159265358979323846 / 2.0;
    constants_tbl["DEG_TO_RAD"] = 3.14159265358979323846 / 180.0;
    constants_tbl["RAD_TO_DEG"] = 180.0 / 3.14159265358979323846;

    // Earth parameters
    constants_tbl["MU_EARTH"] = constants::MU_EARTH;          // Earth gravitational parameter
    constants_tbl["R_EARTH_EQUATOR"] = constants::R_EARTH_EQUATOR;
    constants_tbl["R_EARTH_POLAR"] = constants::R_EARTH_POLAR;
    constants_tbl["OMEGA_EARTH"] = constants::OMEGA_EARTH;    // Earth rotation rate

    // Atmosphere
    constants_tbl["G0"] = constants::G0;                      // Standard gravity
    constants_tbl["RHO0"] = constants::RHO0;                  // Sea level density
    constants_tbl["P0"] = constants::P0;                      // Sea level pressure
    constants_tbl["T0"] = constants::T0;                      // Sea level temperature
    constants_tbl["GAMMA_AIR"] = constants::GAMMA_AIR;        // Air specific heat ratio
    constants_tbl["R_AIR"] = constants::R_AIR;                // Air gas constant
    constants_tbl["R_GAS"] = constants::R_GAS;                // Universal gas constant
    constants_tbl["RHO_WATER"] = constants::RHO_WATER;        // Water density

    // Unit conversions as constants
    constants_tbl["FT_TO_M"] = 0.3048;
    constants_tbl["M_TO_FT"] = 3.28084;
    constants_tbl["KT_TO_MS"] = 0.514444;
    constants_tbl["MS_TO_KT"] = 1.94384;
    constants_tbl["NM_TO_M"] = 1852.0;
    constants_tbl["M_TO_NM"] = 0.000539957;
}

// ============================================================================
// Utility Functions Module
// ============================================================================

void bind_utilities(sol::state_view& lua) {
    constexpr Real PI = 3.14159265358979323846;
    constexpr Real DEG_TO_RAD = PI / 180.0;
    constexpr Real RAD_TO_DEG = 180.0 / PI;

    // Unit conversion functions
    lua["ft_to_m"] = [](Real ft) { return ft * 0.3048; };
    lua["m_to_ft"] = [](Real m) { return m * 3.28084; };
    lua["kt_to_ms"] = [](Real kt) { return kt * 0.514444; };
    lua["ms_to_kt"] = [](Real ms) { return ms * 1.94384; };
    lua["nm_to_m"] = [](Real nm) { return nm * 1852.0; };
    lua["m_to_nm"] = [](Real m) { return m * 0.000539957; };
    lua["deg_to_rad"] = [DEG_TO_RAD](Real deg) { return deg * DEG_TO_RAD; };
    lua["rad_to_deg"] = [RAD_TO_DEG](Real rad) { return rad * RAD_TO_DEG; };
    lua["lbs_to_kg"] = [](Real lbs) { return lbs * 0.453592; };
    lua["kg_to_lbs"] = [](Real kg) { return kg * 2.20462; };

    // Math utilities
    lua["clamp"] = [](Real val, Real min_val, Real max_val) {
        return std::max(min_val, std::min(max_val, val));
    };
    lua["lerp"] = [](Real a, Real b, Real t) {
        return a + t * (b - a);
    };
    lua["wrap_angle"] = [PI](Real angle) {
        constexpr Real TWO_PI = 2.0 * PI;
        while (angle > PI) angle -= TWO_PI;
        while (angle < -PI) angle += TWO_PI;
        return angle;
    };
    lua["wrap_angle_positive"] = [PI](Real angle) {
        constexpr Real TWO_PI = 2.0 * PI;
        while (angle >= TWO_PI) angle -= TWO_PI;
        while (angle < 0) angle += TWO_PI;
        return angle;
    };
}

// ============================================================================
// Module Initialization
// ============================================================================

/**
 * @brief Register all JaguarEngine bindings in a Lua state
 *
 * Call this function to initialize the Lua state with all
 * JaguarEngine types, functions, and constants.
 *
 * @param lua The sol::state to register bindings in
 */
void register_jaguar_bindings(sol::state& lua) {
    // Open standard libraries
    lua.open_libraries(
        sol::lib::base,
        sol::lib::math,
        sol::lib::string,
        sol::lib::table,
        sol::lib::os,
        sol::lib::io
    );

    // Register all types
    bind_vec3(lua);
    bind_quat(lua);
    bind_mat3x3(lua);
    bind_enums(lua);
    bind_entity_state(lua);
    bind_entity_forces(lua);
    bind_entity(lua);
    bind_sensors(lua);
    bind_turbulence(lua);
    bind_engine(lua);

    // Register modules
    bind_constants(lua);
    bind_utilities(lua);

    // Version info
    lua["JAGUAR_VERSION"] = "0.5.0";
    lua["JAGUAR_VERSION_MAJOR"] = 0;
    lua["JAGUAR_VERSION_MINOR"] = 5;
    lua["JAGUAR_VERSION_PATCH"] = 0;

    // Print banner function
    lua["jaguar_info"] = []() {
        return "JaguarEngine Lua Bindings v0.5.0\n"
               "Multi-domain physics simulation engine\n"
               "Domains: Air, Land, Sea, Space\n"
               "Features: Sensors, Turbulence Models (Dryden/Von Karman)\n";
    };
}

/**
 * @brief Create and initialize a new Lua state with JaguarEngine bindings
 *
 * Convenience function that creates a new sol::state and registers
 * all JaguarEngine bindings.
 *
 * @return Unique pointer to initialized Lua state
 */
std::unique_ptr<sol::state> create_jaguar_lua_state() {
    auto lua = std::make_unique<sol::state>();
    register_jaguar_bindings(*lua);
    return lua;
}

} // namespace jaguar::lua

// ============================================================================
// C API for Embedding
// ============================================================================

extern "C" {

/**
 * @brief Initialize JaguarEngine bindings in a lua_State
 *
 * C-compatible function for integrating with existing Lua states.
 *
 * @param L Raw Lua state pointer
 * @return 0 on success
 */
int luaopen_jaguar(lua_State* L) {
    sol::state_view lua(L);

    // Bind all JaguarEngine types and functions
    jaguar::lua::bind_vec3(lua);
    jaguar::lua::bind_quat(lua);
    jaguar::lua::bind_mat3x3(lua);
    jaguar::lua::bind_enums(lua);
    jaguar::lua::bind_entity_state(lua);
    jaguar::lua::bind_entity_forces(lua);
    jaguar::lua::bind_entity(lua);
    jaguar::lua::bind_sensors(lua);
    jaguar::lua::bind_turbulence(lua);
    jaguar::lua::bind_engine(lua);
    jaguar::lua::bind_constants(lua);
    jaguar::lua::bind_utilities(lua);

    return 0;
}

} // extern "C"

#else // !JAGUAR_BUILD_LUA

// Stub when Lua is not enabled
namespace jaguar::lua {

void register_jaguar_bindings(void*) {
    // No-op stub
}

} // namespace jaguar::lua

#endif // JAGUAR_BUILD_LUA
