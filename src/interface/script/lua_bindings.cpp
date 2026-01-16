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

#ifdef JAGUAR_BUILD_LUA

#include <sol/sol.hpp>
#include <memory>
#include <string>
#include <cmath>

namespace jaguar::lua {

using namespace jaguar;
using namespace jaguar::physics;

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

void bind_vec3(sol::state& lua) {
    lua.new_usertype<Vec3>("Vec3",
        // Constructors
        sol::constructors<
            Vec3(),
            Vec3(Real, Real, Real)
        >(),

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
        "normalize", &Vec3::normalize,

        // Static factory methods
        "zero", sol::factories([]() { return Vec3{0, 0, 0}; }),
        "unit_x", sol::factories([]() { return Vec3{1, 0, 0}; }),
        "unit_y", sol::factories([]() { return Vec3{0, 1, 0}; }),
        "unit_z", sol::factories([]() { return Vec3{0, 0, 1}; }),

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

void bind_quat(sol::state& lua) {
    lua.new_usertype<Quat>("Quat",
        // Constructors
        sol::constructors<
            Quat(),
            Quat(Real, Real, Real, Real)
        >(),

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
        "normalize", &Quat::normalize,
        "normalized", &Quat::normalized,
        "to_euler", [](const Quat& q) {
            // Returns roll, pitch, yaw as a table
            Real roll, pitch, yaw;
            q.to_euler(roll, pitch, yaw);
            return std::make_tuple(roll, pitch, yaw);
        },

        // Static factory methods
        "identity", sol::factories([]() { return Quat::identity(); }),
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
        []() { return Quat::identity(); },
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

void bind_mat3x3(sol::state& lua) {
    lua.new_usertype<Mat3x3>("Mat3x3",
        // Constructors
        sol::constructors<Mat3x3()>(),

        // Element access
        "get", [](const Mat3x3& m, int row, int col) {
            return m(row, col);
        },
        "set", [](Mat3x3& m, int row, int col, Real val) {
            m(row, col) = val;
        },

        // Operators
        sol::meta_function::multiplication, sol::overload(
            [](const Mat3x3& m1, const Mat3x3& m2) { return m1 * m2; },
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
                    s += std::to_string(m(i, j));
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
        "trace", &Mat3x3::trace,

        // Static factory methods
        "identity", sol::factories([]() { return Mat3x3::identity(); }),
        "zero", sol::factories([]() { return Mat3x3::zero(); }),
        "inertia", [](Real ixx, Real iyy, Real izz) {
            return Mat3x3::inertia(ixx, iyy, izz);
        },
        "inertia_full", [](Real ixx, Real iyy, Real izz,
                           Real ixy, Real ixz, Real iyz) {
            return Mat3x3::inertia(ixx, iyy, izz, ixy, ixz, iyz);
        }
    );
}

// ============================================================================
// Enum Bindings
// ============================================================================

void bind_enums(sol::state& lua) {
    // Domain enum
    lua.new_enum<Domain>("Domain",
        {
            {"Air", Domain::Air},
            {"Land", Domain::Land},
            {"Sea", Domain::Sea},
            {"Space", Domain::Space},
            {"Unknown", Domain::Unknown}
        }
    );

    // CoordinateFrame enum
    lua.new_enum<CoordinateFrame>("CoordinateFrame",
        {
            {"ECEF", CoordinateFrame::ECEF},
            {"ECI", CoordinateFrame::ECI},
            {"NED", CoordinateFrame::NED},
            {"ENU", CoordinateFrame::ENU},
            {"Body", CoordinateFrame::Body},
            {"Wind", CoordinateFrame::Wind},
            {"Stability", CoordinateFrame::Stability}
        }
    );

    // SimulationState enum
    lua.new_enum<SimulationState>("SimulationState",
        {
            {"Uninitialized", SimulationState::Uninitialized},
            {"Initialized", SimulationState::Initialized},
            {"Running", SimulationState::Running},
            {"Paused", SimulationState::Paused},
            {"Stopped", SimulationState::Stopped}
        }
    );
}

// ============================================================================
// EntityState Bindings
// ============================================================================

void bind_entity_state(sol::state& lua) {
    lua.new_usertype<EntityState>("EntityState",
        sol::constructors<EntityState()>(),

        // Position and orientation
        "position", &EntityState::position,
        "velocity", &EntityState::velocity,
        "acceleration", &EntityState::acceleration,
        "orientation", &EntityState::orientation,
        "angular_velocity", &EntityState::angular_velocity,
        "angular_acceleration", &EntityState::angular_acceleration,

        // Mass properties
        "mass", &EntityState::mass,
        "inertia", &EntityState::inertia,
        "cg_offset", &EntityState::cg_offset,

        // Metadata
        "coordinate_frame", &EntityState::coordinate_frame,
        "time", &EntityState::time,

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

void bind_entity_forces(sol::state& lua) {
    lua.new_usertype<EntityForces>("EntityForces",
        sol::constructors<EntityForces()>(),

        // Force components
        "force", &EntityForces::force,
        "moment", &EntityForces::moment,
        "gravity", &EntityForces::gravity,

        // Computed properties
        "total_force", &EntityForces::total_force,

        // Methods
        "clear", &EntityForces::clear,
        "add_force", &EntityForces::add_force,
        "add_moment", &EntityForces::add_moment,

        // String representation
        sol::meta_function::to_string, [](const EntityForces& f) {
            auto total = f.total_force();
            return "EntityForces(total=(" + std::to_string(total.x) + ", " +
                   std::to_string(total.y) + ", " + std::to_string(total.z) + "))";
        }
    );
}

// ============================================================================
// Entity Bindings
// ============================================================================

void bind_entity(sol::state& lua) {
    lua.new_usertype<Entity>("Entity",
        // No public constructor - created by Engine
        sol::no_constructor,

        // Properties
        "id", sol::readonly(&Entity::id),
        "name", sol::readonly(&Entity::name),
        "domain", sol::readonly(&Entity::domain),
        "active", &Entity::active,

        // State access
        "state", sol::property(
            [](Entity& e) -> EntityState& { return e.state; },
            [](Entity& e, const EntityState& s) { e.state = s; }
        ),
        "forces", sol::property(
            [](Entity& e) -> EntityForces& { return e.forces; },
            [](Entity& e, const EntityForces& f) { e.forces = f; }
        )
    );
}

// ============================================================================
// Engine Bindings
// ============================================================================

void bind_engine(sol::state& lua) {
    lua.new_usertype<Engine>("Engine",
        sol::constructors<Engine()>(),

        // Lifecycle
        "initialize", sol::overload(
            static_cast<bool(Engine::*)()>(&Engine::initialize),
            static_cast<bool(Engine::*)(const std::string&)>(&Engine::initialize)
        ),
        "shutdown", &Engine::shutdown,

        // Simulation control
        "step", &Engine::step,
        "run", &Engine::run,
        "pause", &Engine::pause,
        "resume", &Engine::resume,
        "stop", &Engine::stop,
        "reset", &Engine::reset,

        // State queries
        "get_state", &Engine::get_state,
        "get_time", &Engine::get_time,
        "get_dt", &Engine::get_dt,

        // Entity management
        "create_entity", sol::overload(
            static_cast<EntityId(Engine::*)(const std::string&, Domain)>(&Engine::create_entity),
            [](Engine& e, const std::string& name, const std::string& domain_str) {
                Domain d = Domain::Unknown;
                if (domain_str == "Air" || domain_str == "air") d = Domain::Air;
                else if (domain_str == "Land" || domain_str == "land") d = Domain::Land;
                else if (domain_str == "Sea" || domain_str == "sea") d = Domain::Sea;
                else if (domain_str == "Space" || domain_str == "space") d = Domain::Space;
                return e.create_entity(name, d);
            }
        ),
        "remove_entity", &Engine::remove_entity,
        "get_entity", &Engine::get_entity,
        "get_entity_state", &Engine::get_entity_state,
        "set_entity_state", &Engine::set_entity_state,
        "get_entity_forces", &Engine::get_entity_forces,
        "entity_count", &Engine::entity_count,
        "get_all_entities", &Engine::get_all_entities,

        // String representation
        sol::meta_function::to_string, [](const Engine& e) {
            return "Engine(state=" + std::to_string(static_cast<int>(e.get_state())) +
                   ", time=" + std::to_string(e.get_time()) +
                   ", entities=" + std::to_string(e.entity_count()) + ")";
        }
    );
}

// ============================================================================
// Constants Module
// ============================================================================

void bind_constants(sol::state& lua) {
    sol::table constants = lua.create_named_table("constants");

    // Universal constants
    constants["G"] = constants::G;                        // Gravitational constant
    constants["C"] = constants::C;                        // Speed of light
    constants["PI"] = constants::PI;
    constants["TWO_PI"] = constants::TWO_PI;
    constants["HALF_PI"] = constants::HALF_PI;
    constants["DEG_TO_RAD"] = constants::DEG_TO_RAD;
    constants["RAD_TO_DEG"] = constants::RAD_TO_DEG;

    // Earth parameters
    constants["MU_EARTH"] = constants::MU_EARTH;          // Earth gravitational parameter
    constants["R_EARTH_EQUATOR"] = constants::R_EARTH_EQUATOR;
    constants["R_EARTH_POLAR"] = constants::R_EARTH_POLAR;
    constants["OMEGA_EARTH"] = constants::OMEGA_EARTH;    // Earth rotation rate
    constants["J2"] = constants::J2;                      // Earth J2 coefficient
    constants["FLATTENING"] = constants::FLATTENING;      // Earth flattening

    // Atmosphere
    constants["G0"] = constants::G0;                      // Standard gravity
    constants["RHO0"] = constants::RHO0;                  // Sea level density
    constants["P0"] = constants::P0;                      // Sea level pressure
    constants["T0"] = constants::T0;                      // Sea level temperature
    constants["GAMMA_AIR"] = constants::GAMMA_AIR;        // Air specific heat ratio
    constants["R_AIR"] = constants::R_AIR;                // Air gas constant

    // Unit conversions as constants
    constants["FT_TO_M"] = 0.3048;
    constants["M_TO_FT"] = 3.28084;
    constants["KT_TO_MS"] = 0.514444;
    constants["MS_TO_KT"] = 1.94384;
    constants["NM_TO_M"] = 1852.0;
    constants["M_TO_NM"] = 0.000539957;
}

// ============================================================================
// Utility Functions Module
// ============================================================================

void bind_utilities(sol::state& lua) {
    // Unit conversion functions
    lua["ft_to_m"] = [](Real ft) { return ft * 0.3048; };
    lua["m_to_ft"] = [](Real m) { return m * 3.28084; };
    lua["kt_to_ms"] = [](Real kt) { return kt * 0.514444; };
    lua["ms_to_kt"] = [](Real ms) { return ms * 1.94384; };
    lua["nm_to_m"] = [](Real nm) { return nm * 1852.0; };
    lua["m_to_nm"] = [](Real m) { return m * 0.000539957; };
    lua["deg_to_rad"] = [](Real deg) { return deg * constants::DEG_TO_RAD; };
    lua["rad_to_deg"] = [](Real rad) { return rad * constants::RAD_TO_DEG; };
    lua["lbs_to_kg"] = [](Real lbs) { return lbs * 0.453592; };
    lua["kg_to_lbs"] = [](Real kg) { return kg * 2.20462; };

    // Math utilities
    lua["clamp"] = [](Real val, Real min_val, Real max_val) {
        return std::max(min_val, std::min(max_val, val));
    };
    lua["lerp"] = [](Real a, Real b, Real t) {
        return a + t * (b - a);
    };
    lua["wrap_angle"] = [](Real angle) {
        while (angle > constants::PI) angle -= constants::TWO_PI;
        while (angle < -constants::PI) angle += constants::TWO_PI;
        return angle;
    };
    lua["wrap_angle_positive"] = [](Real angle) {
        while (angle >= constants::TWO_PI) angle -= constants::TWO_PI;
        while (angle < 0) angle += constants::TWO_PI;
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
    bind_engine(lua);

    // Register modules
    bind_constants(lua);
    bind_utilities(lua);

    // Version info
    lua["JAGUAR_VERSION"] = "0.4.0";
    lua["JAGUAR_VERSION_MAJOR"] = 0;
    lua["JAGUAR_VERSION_MINOR"] = 4;
    lua["JAGUAR_VERSION_PATCH"] = 0;

    // Print banner function
    lua["jaguar_info"] = []() {
        return "JaguarEngine Lua Bindings v0.4.0\n"
               "Multi-domain physics simulation engine\n"
               "Domains: Air, Land, Sea, Space\n";
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

    // Use sol2 to bind everything through the existing state_view
    jaguar::lua::bind_vec3(reinterpret_cast<sol::state&>(lua));
    jaguar::lua::bind_quat(reinterpret_cast<sol::state&>(lua));
    jaguar::lua::bind_mat3x3(reinterpret_cast<sol::state&>(lua));
    jaguar::lua::bind_enums(reinterpret_cast<sol::state&>(lua));
    jaguar::lua::bind_entity_state(reinterpret_cast<sol::state&>(lua));
    jaguar::lua::bind_entity_forces(reinterpret_cast<sol::state&>(lua));
    jaguar::lua::bind_entity(reinterpret_cast<sol::state&>(lua));
    jaguar::lua::bind_engine(reinterpret_cast<sol::state&>(lua));
    jaguar::lua::bind_constants(reinterpret_cast<sol::state&>(lua));
    jaguar::lua::bind_utilities(reinterpret_cast<sol::state&>(lua));

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
