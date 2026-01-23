/**
 * @file python_bindings.cpp
 * @brief Python bindings via pybind11 for JaguarEngine
 *
 * Provides Python API for simulation control, entity management,
 * physics state access, and event system integration.
 *
 * @version 1.0.0
 *
 * Features:
 * - Core types: Vec3, Quat, Mat3x3 with NumPy interop
 * - Entity management and physics state
 * - Simulation control and time management
 * - Event system with Python callback support
 * - Threshold monitoring for parameter tracking
 *
 * Example Usage:
 * @code{.py}
 * import pyjaguar as jag
 *
 * # Create and initialize engine
 * engine = jag.Engine()
 * engine.initialize()
 *
 * # Create entities
 * aircraft = engine.create_entity("F16", jag.Domain.Air)
 *
 * # Set initial state
 * state = jag.EntityState()
 * state.position = jag.Vec3(0, 0, 10000)
 * state.velocity = jag.Vec3(200, 0, 0)
 * engine.set_entity_state(aircraft, state)
 *
 * # Run simulation
 * for _ in range(1000):
 *     engine.step(0.01)
 *
 * engine.shutdown()
 * @endcode
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>
#include <pybind11/numpy.h>
#include <pybind11/functional.h>

#include "jaguar/jaguar.h"
#include "jaguar/interface/api.h"
#include "jaguar/physics/entity.h"
#include "jaguar/core/types.h"
#include "jaguar/events/event.h"
#include "jaguar/events/event_dispatcher.h"
#include "jaguar/events/script_callback.h"
#include "jaguar/events/threshold_monitor.h"
#include "jaguar/sensors/sensor.h"
#include "jaguar/sensors/imu_sensor.h"
#include "jaguar/environment/atmospheric_disturbance.h"

#include <sstream>

namespace py = pybind11;
using namespace jaguar;
using namespace jaguar::physics;
using namespace jaguar::events;

// Use Real consistently (which is double)
using Real = jaguar::Real;

// ============================================================================
// Helper Functions
// ============================================================================

namespace {

/**
 * @brief Convert Vec3 to numpy array
 */
py::array_t<double> vec3_to_numpy(const Vec3& v) {
    auto result = py::array_t<double>(3);
    auto buf = result.mutable_unchecked<1>();
    buf(0) = v.x;
    buf(1) = v.y;
    buf(2) = v.z;
    return result;
}

/**
 * @brief Convert numpy array to Vec3
 */
Vec3 numpy_to_vec3(py::array_t<double> arr) {
    auto buf = arr.unchecked<1>();
    if (buf.shape(0) < 3) {
        throw std::runtime_error("Array must have at least 3 elements");
    }
    return Vec3{buf(0), buf(1), buf(2)};
}

/**
 * @brief Convert Quat to numpy array
 */
py::array_t<double> quat_to_numpy(const Quat& q) {
    auto result = py::array_t<double>(4);
    auto buf = result.mutable_unchecked<1>();
    buf(0) = q.w;
    buf(1) = q.x;
    buf(2) = q.y;
    buf(3) = q.z;
    return result;
}

/**
 * @brief Convert numpy array to Quat
 */
Quat numpy_to_quat(py::array_t<double> arr) {
    auto buf = arr.unchecked<1>();
    if (buf.shape(0) < 4) {
        throw std::runtime_error("Array must have at least 4 elements");
    }
    return Quat{buf(0), buf(1), buf(2), buf(3)};
}

/**
 * @brief Convert Mat3x3 to numpy 2D array
 */
py::array_t<double> mat3x3_to_numpy(const Mat3x3& m) {
    auto result = py::array_t<double>({3, 3});
    auto buf = result.mutable_unchecked<2>();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            buf(i, j) = m(i, j);
        }
    }
    return result;
}

/**
 * @brief Convert numpy 2D array to Mat3x3
 */
Mat3x3 numpy_to_mat3x3(py::array_t<double> arr) {
    auto buf = arr.unchecked<2>();
    if (buf.shape(0) < 3 || buf.shape(1) < 3) {
        throw std::runtime_error("Array must be at least 3x3");
    }
    Mat3x3 m;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            m(i, j) = buf(i, j);
        }
    }
    return m;
}

} // anonymous namespace

// ============================================================================
// Python Module Definition
// ============================================================================

PYBIND11_MODULE(pyjaguar, m) {
    m.doc() = R"pbdoc(
        JaguarEngine Python Bindings
        ============================

        Multi-domain physics simulation engine for defense modeling.

        Basic Example::

            import pyjaguar as jag

            # Create and initialize engine
            engine = jag.Engine()
            engine.initialize()

            # Create an aircraft entity
            aircraft = engine.create_entity("f16", jag.Domain.Air)

            # Set initial state
            state = engine.get_entity_state(aircraft)
            state.position = jag.Vec3(0, 0, -1000)
            state.velocity = jag.Vec3(200, 0, 0)
            engine.set_entity_state(aircraft, state)

            # Run simulation
            for i in range(1000):
                engine.step(0.01)
                pos = engine.get_entity_state(aircraft).position
                print(f"Position: {pos.x:.1f}, {pos.y:.1f}, {pos.z:.1f}")

            engine.shutdown()

        Modules:
            - Vec3: 3D vector operations
            - Quat: Quaternion rotation representation
            - Mat3x3: 3x3 matrix operations
            - EntityState: Complete physical state of entity
            - Engine: Main simulation controller
    )pbdoc";

    // ========================================================================
    // Vec3 - 3D Vector
    // ========================================================================
    py::class_<Vec3>(m, "Vec3", "3D vector for positions, velocities, forces")
        .def(py::init<>(), "Create zero vector")
        .def(py::init<Real, Real, Real>(), "Create vector with x, y, z components",
             py::arg("x"), py::arg("y"), py::arg("z"))
        .def_readwrite("x", &Vec3::x, "X component")
        .def_readwrite("y", &Vec3::y, "Y component")
        .def_readwrite("z", &Vec3::z, "Z component")

        // Arithmetic operators
        .def(py::self + py::self)
        .def(py::self - py::self)
        .def(py::self * Real())
        .def(Real() * py::self)
        .def(py::self / Real())
        .def(-py::self)
        .def(py::self += py::self)
        .def(py::self -= py::self)
        .def(py::self *= Real())
        .def(py::self /= Real())

        // Vector operations
        .def("dot", &Vec3::dot, "Dot product with another vector", py::arg("other"))
        .def("cross", &Vec3::cross, "Cross product with another vector", py::arg("other"))
        .def("length", &Vec3::length, "Vector magnitude")
        .def("length_squared", &Vec3::length_squared, "Squared magnitude (faster)")
        .def("normalized", &Vec3::normalized, "Return unit vector")

        // Static constructors
        .def_static("zero", &Vec3::Zero, "Create zero vector")
        .def_static("unit_x", &Vec3::UnitX, "Create unit X vector")
        .def_static("unit_y", &Vec3::UnitY, "Create unit Y vector")
        .def_static("unit_z", &Vec3::UnitZ, "Create unit Z vector")

        // Numpy interop
        .def("to_numpy", &vec3_to_numpy, "Convert to numpy array")
        .def_static("from_numpy", &numpy_to_vec3, "Create from numpy array",
                   py::arg("array"))

        // String representation
        .def("__repr__", [](const Vec3& v) {
            return "Vec3(" + std::to_string(v.x) + ", " +
                   std::to_string(v.y) + ", " + std::to_string(v.z) + ")";
        })
        .def("__str__", [](const Vec3& v) {
            return "(" + std::to_string(v.x) + ", " +
                   std::to_string(v.y) + ", " + std::to_string(v.z) + ")";
        })

        // Sequence protocol
        .def("__len__", [](const Vec3&) { return 3; })
        .def("__getitem__", [](const Vec3& v, int i) {
            if (i < 0) i += 3;
            if (i < 0 || i >= 3) throw py::index_error();
            return (&v.x)[i];
        })
        .def("__setitem__", [](Vec3& v, int i, Real val) {
            if (i < 0) i += 3;
            if (i < 0 || i >= 3) throw py::index_error();
            (&v.x)[i] = val;
        });

    // ========================================================================
    // Quat - Quaternion
    // ========================================================================
    py::class_<Quat>(m, "Quat", "Quaternion for rotation representation (w, x, y, z)")
        .def(py::init<>(), "Create identity quaternion")
        .def(py::init<Real, Real, Real, Real>(), "Create quaternion with w, x, y, z",
             py::arg("w"), py::arg("x"), py::arg("y"), py::arg("z"))
        .def_readwrite("w", &Quat::w, "Scalar component")
        .def_readwrite("x", &Quat::x, "i component")
        .def_readwrite("y", &Quat::y, "j component")
        .def_readwrite("z", &Quat::z, "k component")

        // Operations
        .def(py::self * py::self)
        .def(py::self * Real())
        .def(py::self + py::self)
        .def("conjugate", &Quat::conjugate, "Conjugate (inverse for unit quaternion)")
        .def("inverse", &Quat::inverse, "Multiplicative inverse")
        .def("norm", &Quat::norm, "Quaternion magnitude")
        .def("norm_squared", &Quat::norm_squared, "Squared magnitude")
        .def("normalized", &Quat::normalized, "Return normalized quaternion")
        .def("rotate", &Quat::rotate, "Rotate a vector", py::arg("v"))
        .def("to_rotation_matrix", &Quat::to_rotation_matrix, "Convert to 3x3 rotation matrix")
        .def("to_euler", [](const Quat& q) {
            Real roll, pitch, yaw;
            q.to_euler(roll, pitch, yaw);
            return py::make_tuple(roll, pitch, yaw);
        }, "Convert to Euler angles (roll, pitch, yaw) in radians")

        // Static constructors
        .def_static("identity", &Quat::Identity, "Create identity quaternion")
        .def_static("from_axis_angle", &Quat::from_axis_angle,
                   "Create from axis and angle (radians)",
                   py::arg("axis"), py::arg("angle"))
        .def_static("from_euler", &Quat::from_euler,
                   "Create from Euler angles (roll, pitch, yaw) in radians",
                   py::arg("roll"), py::arg("pitch"), py::arg("yaw"))

        // Numpy interop
        .def("to_numpy", &quat_to_numpy, "Convert to numpy array [w, x, y, z]")
        .def_static("from_numpy", &numpy_to_quat, "Create from numpy array",
                   py::arg("array"))

        // String representation
        .def("__repr__", [](const Quat& q) {
            return "Quat(" + std::to_string(q.w) + ", " + std::to_string(q.x) +
                   ", " + std::to_string(q.y) + ", " + std::to_string(q.z) + ")";
        });

    // ========================================================================
    // Mat3x3 - 3x3 Matrix
    // ========================================================================
    py::class_<Mat3x3>(m, "Mat3x3", "3x3 matrix for rotations, inertia tensors")
        .def(py::init<>(), "Create identity matrix")

        // Element access
        .def("__call__", [](const Mat3x3& m, int row, int col) {
            if (row < 0 || row >= 3 || col < 0 || col >= 3)
                throw py::index_error();
            return m(row, col);
        }, "Get element at (row, col)", py::arg("row"), py::arg("col"))
        .def("set", [](Mat3x3& m, int row, int col, Real val) {
            if (row < 0 || row >= 3 || col < 0 || col >= 3)
                throw py::index_error();
            m(row, col) = val;
        }, "Set element at (row, col)", py::arg("row"), py::arg("col"), py::arg("value"))

        // Operations
        .def(py::self * Vec3())
        .def(py::self * py::self)
        .def(py::self * Real())
        .def(py::self + py::self)
        .def("transpose", &Mat3x3::transpose, "Return transposed matrix")
        .def("determinant", &Mat3x3::determinant, "Compute determinant")
        .def("inverse", &Mat3x3::inverse, "Compute inverse matrix")

        // Static constructors
        .def_static("identity", &Mat3x3::Identity, "Create identity matrix")
        .def_static("zero", &Mat3x3::Zero, "Create zero matrix")
        .def_static("diagonal", &Mat3x3::Diagonal, "Create diagonal matrix",
                   py::arg("a"), py::arg("b"), py::arg("c"))
        .def_static("inertia", &Mat3x3::Inertia,
                   "Create inertia tensor from moments and products",
                   py::arg("ixx"), py::arg("iyy"), py::arg("izz"),
                   py::arg("ixy") = 0.0, py::arg("ixz") = 0.0, py::arg("iyz") = 0.0)

        // Numpy interop
        .def("to_numpy", &mat3x3_to_numpy, "Convert to 3x3 numpy array")
        .def_static("from_numpy", &numpy_to_mat3x3, "Create from numpy array",
                   py::arg("array"))

        // String representation
        .def("__repr__", [](const Mat3x3& m) {
            std::string s = "Mat3x3([\n";
            for (int i = 0; i < 3; ++i) {
                s += "  [";
                for (int j = 0; j < 3; ++j) {
                    s += std::to_string(m(i, j));
                    if (j < 2) s += ", ";
                }
                s += "]";
                if (i < 2) s += ",\n";
            }
            s += "\n])";
            return s;
        });

    // ========================================================================
    // Domain Enum
    // ========================================================================
    py::enum_<Domain>(m, "Domain", "Simulation domain classification")
        .value("Air", Domain::Air, "Aircraft, missiles, projectiles")
        .value("Land", Domain::Land, "Ground vehicles, tracked vehicles")
        .value("Sea", Domain::Sea, "Surface vessels, submarines")
        .value("Space", Domain::Space, "Satellites, re-entry vehicles")
        .value("Generic", Domain::Generic, "Generic rigid body")
        .export_values();

    // ========================================================================
    // CoordinateFrame Enum
    // ========================================================================
    py::enum_<CoordinateFrame>(m, "CoordinateFrame", "Coordinate reference frames")
        .value("ECEF", CoordinateFrame::ECEF, "Earth-Centered, Earth-Fixed")
        .value("ECI", CoordinateFrame::ECI, "Earth-Centered Inertial (J2000)")
        .value("NED", CoordinateFrame::NED, "North-East-Down (local)")
        .value("ENU", CoordinateFrame::ENU, "East-North-Up (local)")
        .value("Body", CoordinateFrame::Body, "Entity body-fixed frame")
        .export_values();

    // ========================================================================
    // SimulationState Enum
    // ========================================================================
    py::enum_<SimulationState>(m, "SimulationState", "Simulation execution state")
        .value("Uninitialized", SimulationState::Uninitialized)
        .value("Initialized", SimulationState::Initialized)
        .value("Running", SimulationState::Running)
        .value("Paused", SimulationState::Paused)
        .value("Stopped", SimulationState::Stopped)
        .value("Error", SimulationState::Error)
        .export_values();

    // ========================================================================
    // EntityState
    // ========================================================================
    py::class_<EntityState>(m, "EntityState", "Complete physical state of an entity")
        .def(py::init<>(), "Create default state")
        .def_readwrite("position", &EntityState::position,
                      "Position in ECEF coordinates (meters)")
        .def_readwrite("velocity", &EntityState::velocity,
                      "Velocity in ECEF frame (m/s)")
        .def_readwrite("orientation", &EntityState::orientation,
                      "Orientation quaternion (body to ECEF)")
        .def_readwrite("angular_velocity", &EntityState::angular_velocity,
                      "Angular velocity in body frame (rad/s)")
        .def_readwrite("acceleration", &EntityState::acceleration,
                      "Linear acceleration (m/s²)")
        .def_readwrite("angular_accel", &EntityState::angular_accel,
                      "Angular acceleration (rad/s²)")
        .def_readwrite("mass", &EntityState::mass, "Mass in kilograms")
        .def_readwrite("inertia", &EntityState::inertia,
                      "Inertia tensor in body frame (kg·m²)")
        .def("__repr__", [](const EntityState& s) {
            return "EntityState(pos=" + std::to_string(s.position.x) + "," +
                   std::to_string(s.position.y) + "," + std::to_string(s.position.z) +
                   ", mass=" + std::to_string(s.mass) + ")";
        });

    // ========================================================================
    // EntityForces
    // ========================================================================
    py::class_<EntityForces>(m, "EntityForces", "Accumulated forces and torques")
        .def(py::init<>())
        .def_readwrite("force", &EntityForces::force, "Total force in body frame (N)")
        .def_readwrite("torque", &EntityForces::torque, "Total torque in body frame (N·m)")
        .def("clear", &EntityForces::clear, "Reset forces and torques to zero")
        .def("add_force", &EntityForces::add_force, "Add force vector", py::arg("f"))
        .def("add_torque", &EntityForces::add_torque, "Add torque vector", py::arg("t"))
        .def("add_force_at_point", &EntityForces::add_force_at_point,
             "Add force at position (computes torque)", py::arg("f"), py::arg("r"));

    // ========================================================================
    // Entity
    // ========================================================================
    py::class_<Entity>(m, "Entity", "Lightweight entity handle")
        .def_readonly("id", &Entity::id, "Unique entity identifier")
        .def_readonly("state_index", &Entity::state_index, "Index into state arrays")
        .def_readwrite("name", &Entity::name, "Entity name")
        .def_readwrite("active", &Entity::active, "Is entity being simulated")
        .def_readonly("primary_domain", &Entity::primary_domain, "Primary physics domain")
        .def("is_valid", &Entity::is_valid, "Check if entity is valid")
        .def("__repr__", [](const Entity& e) {
            return "Entity(id=" + std::to_string(e.id) +
                   ", name='" + e.name + "', active=" +
                   (e.active ? "True" : "False") + ")";
        });

    // ========================================================================
    // Engine - Main Interface
    // ========================================================================
    py::class_<Engine>(m, "Engine", R"pbdoc(
        Main simulation engine - controls simulation lifecycle and entity management.

        Example::

            engine = jag.Engine()
            engine.initialize("config.xml")

            # Create entities
            aircraft = engine.create_entity("f16", jag.Domain.Air)
            ship = engine.create_entity("destroyer", jag.Domain.Sea)

            # Run simulation
            while engine.get_time() < 300.0:
                engine.step(0.01)

            engine.shutdown()
    )pbdoc")
        .def(py::init<>(), "Create new engine instance")

        // Lifecycle
        .def("initialize", py::overload_cast<const std::string&>(&Engine::initialize),
             "Initialize engine with configuration file", py::arg("config_path"))
        .def("initialize", py::overload_cast<>(&Engine::initialize),
             "Initialize engine with default configuration")
        .def("shutdown", &Engine::shutdown, "Shutdown engine and release resources")
        .def("is_initialized", &Engine::is_initialized, "Check if engine is initialized")

        // Simulation control
        .def("run", &Engine::run, "Run simulation continuously")
        .def("run_for", &Engine::run_for, "Run for specified duration",
             py::arg("duration_sec"))
        .def("step", &Engine::step, "Advance simulation by time step",
             py::arg("dt"))
        .def("pause", &Engine::pause, "Pause simulation")
        .def("resume", &Engine::resume, "Resume simulation")
        .def("stop", &Engine::stop, "Stop simulation")
        .def("get_state", &Engine::get_state, "Get current simulation state")

        // Time management
        .def("get_time", &Engine::get_time, "Get current simulation time (seconds)")
        .def("get_time_scale", &Engine::get_time_scale,
             "Get time scale (1.0 = real-time)")
        .def("set_time_scale", &Engine::set_time_scale, "Set time scale",
             py::arg("scale"))
        .def("set_fixed_time_step", &Engine::set_fixed_time_step,
             "Set fixed time step for integration", py::arg("dt"))

        // Entity management
        .def("create_entity", py::overload_cast<const std::string&>(&Engine::create_entity),
             "Create entity from XML file", py::arg("xml_path"))
        .def("create_entity",
             py::overload_cast<const std::string&, Domain>(&Engine::create_entity),
             "Create entity with name and domain",
             py::arg("name"), py::arg("domain") = Domain::Generic)
        .def("destroy_entity", &Engine::destroy_entity, "Destroy entity",
             py::arg("id"))
        .def("entity_exists", &Engine::entity_exists, "Check if entity exists",
             py::arg("id"))
        .def("get_entity_state", &Engine::get_entity_state, "Get entity state",
             py::arg("id"))
        .def("set_entity_state", &Engine::set_entity_state, "Set entity state",
             py::arg("id"), py::arg("state"))

        // Property access
        .def("get_property", py::overload_cast<const std::string&>(&Engine::get_property, py::const_),
             "Get global property value", py::arg("path"))
        .def("set_property", py::overload_cast<const std::string&, Real>(&Engine::set_property),
             "Set global property value", py::arg("path"), py::arg("value"))
        .def("get_entity_property",
             py::overload_cast<EntityId, const std::string&>(&Engine::get_property, py::const_),
             "Get entity property value", py::arg("id"), py::arg("property"))
        .def("set_entity_property",
             py::overload_cast<EntityId, const std::string&, Real>(&Engine::set_property),
             "Set entity property value", py::arg("id"), py::arg("property"), py::arg("value"));

    // ========================================================================
    // Physical Constants Submodule
    // ========================================================================
    auto constants_mod = m.def_submodule("constants", "Physical constants");

    constants_mod.attr("G") = constants::G;
    constants_mod.attr("MU_EARTH") = constants::MU_EARTH;
    constants_mod.attr("R_EARTH_EQUATOR") = constants::R_EARTH_EQUATOR;
    constants_mod.attr("R_EARTH_POLAR") = constants::R_EARTH_POLAR;
    constants_mod.attr("OMEGA_EARTH") = constants::OMEGA_EARTH;
    constants_mod.attr("G0") = constants::G0;
    constants_mod.attr("C") = constants::C;
    constants_mod.attr("P0") = constants::P0;
    constants_mod.attr("T0") = constants::T0;
    constants_mod.attr("RHO0") = constants::RHO0;
    constants_mod.attr("R_GAS") = constants::R_GAS;
    constants_mod.attr("R_AIR") = constants::R_AIR;
    constants_mod.attr("GAMMA_AIR") = constants::GAMMA_AIR;
    constants_mod.attr("RHO_WATER") = constants::RHO_WATER;
    constants_mod.attr("PI") = constants::PI;
    constants_mod.attr("DEG_TO_RAD") = constants::DEG_TO_RAD;
    constants_mod.attr("RAD_TO_DEG") = constants::RAD_TO_DEG;
    constants_mod.attr("FT_TO_M") = constants::FT_TO_M;
    constants_mod.attr("M_TO_FT") = constants::M_TO_FT;
    constants_mod.attr("KT_TO_MS") = constants::KT_TO_MS;
    constants_mod.attr("LBF_TO_N") = constants::LBF_TO_N;
    constants_mod.attr("SLUG_TO_KG") = constants::SLUG_TO_KG;

    // ========================================================================
    // Utility Functions
    // ========================================================================

    m.def("deg_to_rad", [](Real deg) { return deg * constants::DEG_TO_RAD; },
          "Convert degrees to radians", py::arg("degrees"));
    m.def("rad_to_deg", [](Real rad) { return rad * constants::RAD_TO_DEG; },
          "Convert radians to degrees", py::arg("radians"));
    m.def("ft_to_m", [](Real ft) { return ft * constants::FT_TO_M; },
          "Convert feet to meters", py::arg("feet"));
    m.def("m_to_ft", [](Real m) { return m * constants::M_TO_FT; },
          "Convert meters to feet", py::arg("meters"));
    m.def("kt_to_ms", [](Real kt) { return kt * constants::KT_TO_MS; },
          "Convert knots to m/s", py::arg("knots"));

    // ========================================================================
    // Invalid Entity ID Constant
    // ========================================================================
    m.attr("INVALID_ENTITY_ID") = INVALID_ENTITY_ID;

    // ========================================================================
    // Event System - Event Types and Categories
    // ========================================================================

    py::enum_<EventType>(m, "EventType", "Event type enumeration")
        // System Events
        .value("SimulationStarted", EventType::SimulationStarted)
        .value("SimulationPaused", EventType::SimulationPaused)
        .value("SimulationResumed", EventType::SimulationResumed)
        .value("SimulationStopped", EventType::SimulationStopped)
        .value("TimeStepCompleted", EventType::TimeStepCompleted)
        .value("FrameCompleted", EventType::FrameCompleted)
        // Entity Lifecycle
        .value("EntityCreated", EventType::EntityCreated)
        .value("EntityDestroyed", EventType::EntityDestroyed)
        .value("EntityActivated", EventType::EntityActivated)
        .value("EntityDeactivated", EventType::EntityDeactivated)
        .value("EntityStateChanged", EventType::EntityStateChanged)
        .value("ComponentAdded", EventType::ComponentAdded)
        .value("ComponentRemoved", EventType::ComponentRemoved)
        // Physics/Collision
        .value("CollisionEnter", EventType::CollisionEnter)
        .value("CollisionExit", EventType::CollisionExit)
        .value("CollisionStay", EventType::CollisionStay)
        .value("TriggerEnter", EventType::TriggerEnter)
        .value("TriggerExit", EventType::TriggerExit)
        .value("ConstraintBroken", EventType::ConstraintBroken)
        // Air Domain
        .value("Stall", EventType::Stall)
        .value("StallRecovery", EventType::StallRecovery)
        .value("OverspeedWarning", EventType::OverspeedWarning)
        .value("OverspeedRecovery", EventType::OverspeedRecovery)
        .value("EngineFlameout", EventType::EngineFlameout)
        .value("EngineRestart", EventType::EngineRestart)
        // Land Domain
        .value("WheelTouchdown", EventType::WheelTouchdown)
        .value("WheelLiftoff", EventType::WheelLiftoff)
        .value("VehicleRollover", EventType::VehicleRollover)
        .value("TrackSlip", EventType::TrackSlip)
        // Sea Domain
        .value("Capsize", EventType::Capsize)
        .value("Grounding", EventType::Grounding)
        .value("Flooding", EventType::Flooding)
        .value("WaveImpact", EventType::WaveImpact)
        // Space Domain
        .value("OrbitInsertion", EventType::OrbitInsertion)
        .value("OrbitDecay", EventType::OrbitDecay)
        .value("Reentry", EventType::Reentry)
        .value("Docking", EventType::Docking)
        .value("Undocking", EventType::Undocking)
        // Sensor Events
        .value("SensorActivated", EventType::SensorActivated)
        .value("SensorDeactivated", EventType::SensorDeactivated)
        .value("SensorUpdate", EventType::SensorUpdate)
        .value("SensorFailure", EventType::SensorFailure)
        .value("TargetAcquired", EventType::TargetAcquired)
        .value("TargetLost", EventType::TargetLost)
        // Threshold Events
        .value("ThresholdExceeded", EventType::ThresholdExceeded)
        .value("ThresholdRecovered", EventType::ThresholdRecovered)
        .value("SpeedLimitExceeded", EventType::SpeedLimitExceeded)
        .value("AltitudeLimitExceeded", EventType::AltitudeLimitExceeded)
        .value("TemperatureLimitExceeded", EventType::TemperatureLimitExceeded)
        .value("StressLimitExceeded", EventType::StressLimitExceeded)
        .value("FuelLow", EventType::FuelLow)
        .value("FuelCritical", EventType::FuelCritical)
        // Damage Events
        .value("DamageReceived", EventType::DamageReceived)
        .value("ComponentDamaged", EventType::ComponentDamaged)
        .value("ComponentDestroyed", EventType::ComponentDestroyed)
        .value("EntityDestroyed_Damage", EventType::EntityDestroyed_Damage)
        // User-Defined
        .value("UserDefined", EventType::UserDefined)
        .export_values();

    py::enum_<EventCategory>(m, "EventCategory", "Event category for filtering")
        .value("None_", EventCategory::None)
        .value("System", EventCategory::System)
        .value("Entity", EventCategory::Entity)
        .value("Physics", EventCategory::Physics)
        .value("Domain", EventCategory::Domain)
        .value("Sensor", EventCategory::Sensor)
        .value("Threshold", EventCategory::Threshold)
        .value("User", EventCategory::User)
        .value("All", EventCategory::All)
        .export_values();

    py::enum_<EventPriority>(m, "EventPriority", "Event dispatch priority")
        .value("Immediate", EventPriority::Immediate, "Process immediately (critical)")
        .value("High", EventPriority::High, "Process before normal events")
        .value("Normal", EventPriority::Normal, "Standard priority")
        .value("Low", EventPriority::Low, "Process after normal events")
        .value("Deferred", EventPriority::Deferred, "Process at end of frame")
        .export_values();

    // Event category utility function
    m.def("get_event_category", &get_event_category,
          "Get the category for an event type", py::arg("type"));
    m.def("get_event_type_name", &get_event_type_name,
          "Get the name string for an event type", py::arg("type"));

    // ========================================================================
    // Event System - ScriptEventData for Python callbacks
    // ========================================================================

    py::class_<ScriptEventData>(m, "ScriptEventData",
        "Simplified event data for Python callbacks")
        .def(py::init<>())
        .def_readonly("type", &ScriptEventData::type, "Event type")
        .def_readonly("category", &ScriptEventData::category, "Event category")
        .def_readonly("source_entity", &ScriptEventData::source_entity, "Source entity ID")
        .def_readonly("timestamp", &ScriptEventData::timestamp, "Event timestamp")
        .def_readonly("priority", &ScriptEventData::priority, "Event priority")
        .def("get_field", [](const ScriptEventData& data, const std::string& name) -> py::object {
            auto it = data.fields.find(name);
            if (it == data.fields.end()) {
                return py::none();
            }
            const auto& value = it->second;
            try {
                if (value.type() == typeid(Real)) {
                    return py::cast(std::any_cast<Real>(value));
                } else if (value.type() == typeid(int)) {
                    return py::cast(std::any_cast<int>(value));
                } else if (value.type() == typeid(EntityId)) {
                    return py::cast(std::any_cast<EntityId>(value));
                } else if (value.type() == typeid(UInt64)) {
                    return py::cast(std::any_cast<UInt64>(value));
                } else if (value.type() == typeid(bool)) {
                    return py::cast(std::any_cast<bool>(value));
                } else if (value.type() == typeid(std::string)) {
                    return py::cast(std::any_cast<std::string>(value));
                }
            } catch (...) {}
            return py::none();
        }, "Get a field value by name", py::arg("name"))
        .def("has_field", &ScriptEventData::has_field,
             "Check if a field exists", py::arg("name"))
        .def("__repr__", [](const ScriptEventData& data) {
            return "ScriptEventData(type=" + std::to_string(static_cast<int>(data.type)) +
                   ", source=" + std::to_string(data.source_entity) +
                   ", timestamp=" + std::to_string(data.timestamp) + ")";
        });

    // ========================================================================
    // Event System - EventDispatcher with Python callback support
    // ========================================================================

    // Python callback wrapper class
    class PyEventCallback : public IScriptCallback {
    public:
        PyEventCallback(py::function callback, std::string name = "python_handler")
            : callback_(std::move(callback)), name_(std::move(name)) {}

        bool invoke(const ScriptEventData& data) override {
            py::gil_scoped_acquire gil;
            try {
                py::object result = callback_(data);
                if (py::isinstance<py::bool_>(result)) {
                    return result.cast<bool>();
                }
                return true;
            } catch (py::error_already_set& e) {
                py::print("Python callback error:", e.what());
                return true;
            }
        }

        std::string get_name() const override { return name_; }
        bool is_valid() const override {
            py::gil_scoped_acquire gil;
            return !callback_.is_none();
        }

    private:
        py::function callback_;
        std::string name_;
    };

    py::class_<EventDispatcher>(m, "EventDispatcher",
        R"pbdoc(
        Event dispatch and subscription system.

        Allows subscribing to events with Python callbacks:

            dispatcher = jag.EventDispatcher()

            def on_collision(event_data):
                print(f"Collision at t={event_data.timestamp}")
                entity_a = event_data.get_field("entity_a")
                entity_b = event_data.get_field("entity_b")
                print(f"Entities: {entity_a} and {entity_b}")
                return True  # Continue propagation

            handler_id = dispatcher.subscribe(
                jag.EventType.CollisionEnter,
                on_collision,
                "collision_handler"
            )

            # Later...
            dispatcher.unsubscribe(handler_id)
        )pbdoc")
        .def(py::init<>())
        // Subscribe with Python callback
        .def("subscribe", [](EventDispatcher& dispatcher, EventType type,
                             py::function callback, const std::string& name, int priority) {
            auto py_callback = std::make_shared<PyEventCallback>(callback, name);
            return dispatcher.subscribe(type, [=](Event& event) {
                ScriptEventData data = to_script_data(event);
                return py_callback->invoke(data);
            }, name, priority);
        }, "Subscribe to event type with Python callback",
           py::arg("type"), py::arg("callback"),
           py::arg("name") = "python_handler", py::arg("priority") = 0)

        .def("subscribe_category", [](EventDispatcher& dispatcher, EventCategory category,
                                       py::function callback, const std::string& name, int priority) {
            auto py_callback = std::make_shared<PyEventCallback>(callback, name);
            return dispatcher.subscribe_category(category, [=](Event& event) {
                ScriptEventData data = to_script_data(event);
                return py_callback->invoke(data);
            }, name, priority);
        }, "Subscribe to all events in a category",
           py::arg("category"), py::arg("callback"),
           py::arg("name") = "python_handler", py::arg("priority") = 0)

        .def("subscribe_all", [](EventDispatcher& dispatcher,
                                  py::function callback, const std::string& name, int priority) {
            auto py_callback = std::make_shared<PyEventCallback>(callback, name);
            return dispatcher.subscribe_all([=](Event& event) {
                ScriptEventData data = to_script_data(event);
                return py_callback->invoke(data);
            }, name, priority);
        }, "Subscribe to all events",
           py::arg("callback"), py::arg("name") = "python_handler", py::arg("priority") = 0)

        .def("unsubscribe", &EventDispatcher::unsubscribe,
             "Unsubscribe a handler by ID", py::arg("handler_id"))
        .def("handler_count", &EventDispatcher::total_handler_count,
             "Get total number of registered handlers")
        .def("queue_size", &EventDispatcher::queue_size,
             "Get number of pending events in queue")
        .def("flush_queue", &EventDispatcher::flush_queue,
             "Process all pending events, returns count processed")
        .def("clear_queue", &EventDispatcher::clear_queue,
             "Clear all pending events without processing")
        .def("dispatch", static_cast<void (EventDispatcher::*)(Event&)>(&EventDispatcher::dispatch),
             "Dispatch an event immediately", py::arg("event"))
        .def("queue", &EventDispatcher::queue,
             "Queue an event for deferred dispatch", py::arg("event"));

    // ========================================================================
    // Event System - ThresholdMonitor
    // ========================================================================

    py::class_<ThresholdConfig>(m, "ThresholdConfig",
        "Configuration for a threshold to monitor")
        .def(py::init<>())
        .def_readwrite("parameter_name", &ThresholdConfig::parameter_name,
                       "Name of the parameter being monitored")
        .def_readwrite("upper_threshold", &ThresholdConfig::upper_threshold,
                       "Upper threshold value (optional)")
        .def_readwrite("lower_threshold", &ThresholdConfig::lower_threshold,
                       "Lower threshold value (optional)")
        .def_readwrite("hysteresis", &ThresholdConfig::hysteresis,
                       "Hysteresis band to prevent oscillation")
        .def_readwrite("emit_on_recover", &ThresholdConfig::emit_on_recover,
                       "Whether to emit event on recovery")
        .def_readwrite("priority", &ThresholdConfig::priority,
                       "Priority for emitted events");

    py::enum_<ThresholdState>(m, "ThresholdState", "State of a tracked threshold")
        .value("Normal", ThresholdState::Normal, "Value within bounds")
        .value("UpperExceeded", ThresholdState::UpperExceeded, "Value exceeds upper threshold")
        .value("LowerExceeded", ThresholdState::LowerExceeded, "Value below lower threshold")
        .export_values();

    py::class_<ThresholdMonitor>(m, "ThresholdMonitor",
        R"pbdoc(
        Monitor parameters and emit events when thresholds are crossed.

        Example:
            monitor = jag.ThresholdMonitor()
            monitor.set_event_dispatcher(dispatcher)
            monitor.set_entity_id(aircraft_id)

            # Configure speed threshold
            config = jag.ThresholdConfig()
            config.parameter_name = "speed"
            config.upper_threshold = 340.0  # Mach 1
            config.hysteresis = 5.0
            monitor.add_threshold("speed", config)

            # In simulation loop:
            current_speed = get_aircraft_speed()
            monitor.set_current_time(sim_time)
            monitor.update("speed", current_speed)
        )pbdoc")
        .def(py::init<>())
        .def("set_event_dispatcher", &ThresholdMonitor::set_event_dispatcher,
             "Set event dispatcher for emitting events",
             py::arg("dispatcher"), py::keep_alive<1, 2>())
        .def("set_entity_id", &ThresholdMonitor::set_entity_id,
             "Set entity ID that owns these thresholds", py::arg("id"))
        .def("set_current_time", &ThresholdMonitor::set_current_time,
             "Set current simulation time for event timestamps", py::arg("time"))
        .def("add_threshold", &ThresholdMonitor::add_threshold,
             "Add or update a threshold configuration",
             py::arg("name"), py::arg("config"))
        .def("remove_threshold", &ThresholdMonitor::remove_threshold,
             "Remove a threshold", py::arg("name"))
        .def("update", [](ThresholdMonitor& monitor, const std::string& name,
                          Real value, py::object timestamp) {
            if (timestamp.is_none()) {
                return monitor.update(name, value);
            }
            return monitor.update(name, value, timestamp.cast<Real>());
        }, "Update parameter value and check thresholds",
           py::arg("name"), py::arg("value"), py::arg("timestamp") = py::none())
        .def("get_state", &ThresholdMonitor::get_state,
             "Get current state of a threshold", py::arg("name"))
        .def("any_exceeded", &ThresholdMonitor::any_exceeded,
             "Check if any thresholds are currently exceeded")
        .def("get_exceeded_thresholds", &ThresholdMonitor::get_exceeded_thresholds,
             "Get list of currently exceeded threshold names")
        .def("reset_state", &ThresholdMonitor::reset_state,
             "Reset a threshold state to normal (without event)", py::arg("name"))
        .def("reset_all_states", &ThresholdMonitor::reset_all_states,
             "Reset all threshold states to normal")
        .def("clear", &ThresholdMonitor::clear, "Clear all thresholds")
        .def("threshold_count", &ThresholdMonitor::threshold_count,
             "Get number of configured thresholds");

    // Threshold preset factory functions
    m.def("fuel_threshold", &fuel_threshold,
          "Create fuel monitoring threshold config",
          py::arg("low_warning"), py::arg("critical_low") = 5.0);
    m.def("speed_threshold", &speed_threshold,
          "Create speed monitoring threshold config",
          py::arg("max_speed"), py::arg("hysteresis") = 5.0);
    m.def("temperature_threshold", [](Real max_temp, py::object min_temp) {
        return temperature_threshold(max_temp,
            min_temp.is_none() ? std::nullopt : std::optional<Real>(min_temp.cast<Real>()));
    }, "Create temperature monitoring threshold config",
       py::arg("max_temp"), py::arg("min_temp") = py::none());
    m.def("altitude_threshold", &altitude_threshold,
          "Create altitude monitoring threshold config (aircraft)",
          py::arg("max_altitude"), py::arg("min_altitude"));
    m.def("gforce_threshold", &gforce_threshold,
          "Create G-force monitoring threshold config",
          py::arg("max_g"), py::arg("min_g"));

    // ========================================================================
    // Component Bits Constants
    // ========================================================================

    auto components_mod = m.def_submodule("components", "Component bit masks");
    components_mod.attr("AERODYNAMICS") = ComponentBits::Aerodynamics;
    components_mod.attr("PROPULSION") = ComponentBits::Propulsion;
    components_mod.attr("GROUND_CONTACT") = ComponentBits::GroundContact;
    components_mod.attr("TERRAMECHANICS") = ComponentBits::Terramechanics;
    components_mod.attr("HYDRODYNAMICS") = ComponentBits::Hydrodynamics;
    components_mod.attr("BUOYANCY") = ComponentBits::Buoyancy;
    components_mod.attr("ORBITAL_DYNAMICS") = ComponentBits::OrbitalDynamics;
    components_mod.attr("GRAVITY") = ComponentBits::Gravity;
    components_mod.attr("MISSILE") = ComponentBits::Missile;
    components_mod.attr("SEEKER") = ComponentBits::Seeker;
    components_mod.attr("GUIDANCE") = ComponentBits::Guidance;
    components_mod.attr("FLIGHT_CONTROL") = ComponentBits::FlightControl;
    components_mod.attr("SUSPENSION") = ComponentBits::Suspension;
    components_mod.attr("COLLISION") = ComponentBits::Collision;
    components_mod.attr("DAMAGEABLE") = ComponentBits::Damageable;

    // ========================================================================
    // Sensor Framework
    // ========================================================================
    using namespace jaguar::sensors;

    // Sensor Types
    py::enum_<SensorType>(m, "SensorType", "Types of sensors")
        .value("IMU", SensorType::IMU, "Inertial Measurement Unit")
        .value("GPS", SensorType::GPS, "Global Positioning System")
        .value("Altimeter", SensorType::Altimeter, "Barometric/radar altimeter")
        .value("Airspeed", SensorType::Airspeed, "Pitot-static airspeed")
        .value("Magnetometer", SensorType::Magnetometer, "Magnetic heading")
        .value("Radar", SensorType::Radar, "Radar sensor")
        .value("Lidar", SensorType::Lidar, "Light Detection and Ranging")
        .value("Camera", SensorType::Camera, "Vision sensor")
        .value("Infrared", SensorType::Infrared, "Infrared/thermal")
        .value("Sonar", SensorType::Sonar, "Underwater acoustic")
        .value("DepthGauge", SensorType::DepthGauge, "Pressure-based depth")
        .value("Custom", SensorType::Custom, "User-defined")
        .export_values();

    // Sensor State
    py::enum_<SensorState>(m, "SensorState", "Sensor operational state")
        .value("Uninitialized", SensorState::Uninitialized)
        .value("Initializing", SensorState::Initializing)
        .value("Operational", SensorState::Operational)
        .value("Degraded", SensorState::Degraded)
        .value("Failed", SensorState::Failed)
        .value("Disabled", SensorState::Disabled)
        .export_values();

    // Failure Modes
    py::enum_<FailureMode>(m, "FailureMode", "Sensor failure modes")
        .value("None_", FailureMode::None)
        .value("Bias", FailureMode::Bias)
        .value("Drift", FailureMode::Drift)
        .value("Noise", FailureMode::Noise)
        .value("Saturation", FailureMode::Saturation)
        .value("Dropout", FailureMode::Dropout)
        .value("Stuck", FailureMode::Stuck)
        .value("Oscillation", FailureMode::Oscillation)
        .value("Complete", FailureMode::Complete)
        .export_values();

    // IMU Grade
    py::enum_<IMUGrade>(m, "IMUGrade", "IMU quality grades")
        .value("Consumer", IMUGrade::Consumer, "MEMS smartphone grade")
        .value("Industrial", IMUGrade::Industrial, "Industrial MEMS")
        .value("Tactical", IMUGrade::Tactical, "Tactical grade")
        .value("Navigation", IMUGrade::Navigation, "Navigation grade (RLG/FOG)")
        .value("Strategic", IMUGrade::Strategic, "Strategic/Marine grade")
        .value("Custom", IMUGrade::Custom, "User-defined")
        .export_values();

    // Vec3Measurement
    py::class_<Vec3Measurement>(m, "Vec3Measurement", "3D sensor measurement with metadata")
        .def(py::init<>())
        .def_readwrite("value", &Vec3Measurement::value, "Measured value")
        .def_readwrite("timestamp", &Vec3Measurement::timestamp, "Simulation time")
        .def_readwrite("measurement_time", &Vec3Measurement::measurement_time, "Physical measurement time")
        .def_readwrite("valid", &Vec3Measurement::valid, "Is measurement valid")
        .def_readwrite("confidence", &Vec3Measurement::confidence, "Confidence level [0,1]")
        .def_readwrite("sequence_number", &Vec3Measurement::sequence_number, "Sequence number")
        .def("__repr__", [](const Vec3Measurement& m) {
            return "Vec3Measurement(value=" + std::to_string(m.value.x) + "," +
                   std::to_string(m.value.y) + "," + std::to_string(m.value.z) +
                   ", valid=" + (m.valid ? "True" : "False") + ")";
        });

    // IMUSensor
    py::class_<IMUSensor>(m, "IMUSensor", R"pbdoc(
        High-fidelity IMU sensor simulation.

        Provides accelerometer and gyroscope measurements with realistic
        noise models including bias, random walk, and white noise.

        Example::

            # Create tactical-grade IMU
            imu = jag.create_tactical_imu("nav_imu", aircraft_id)
            imu.initialize()

            # In simulation loop:
            imu.update(entity_state, dt)

            accel = imu.get_acceleration()
            gyro = imu.get_angular_velocity()

            if accel.valid:
                print(f"Accel: {accel.value.x:.3f}, {accel.value.y:.3f}, {accel.value.z:.3f}")
    )pbdoc")
        .def(py::init<const std::string&, IMUGrade, EntityId>(),
             "Create IMU with grade preset",
             py::arg("name"), py::arg("grade"), py::arg("entity"))
        .def("initialize", &IMUSensor::initialize, "Initialize sensor")
        .def("update", [](IMUSensor& imu, const EntityState& state, Real dt) {
            imu.update(state, dt);
        }, "Update sensor with entity state", py::arg("state"), py::arg("dt"))
        .def("reset", &IMUSensor::reset, "Reset sensor state")
        .def("get_acceleration", &IMUSensor::get_acceleration,
             "Get latest accelerometer measurement", py::return_value_policy::reference)
        .def("get_angular_velocity", &IMUSensor::get_angular_velocity,
             "Get latest gyroscope measurement", py::return_value_policy::reference)
        .def("get_true_acceleration", &IMUSensor::get_true_acceleration,
             "Get true (noise-free) acceleration", py::return_value_policy::reference)
        .def("get_true_angular_velocity", &IMUSensor::get_true_angular_velocity,
             "Get true (noise-free) angular velocity", py::return_value_policy::reference)
        .def("get_delta_v", &IMUSensor::get_delta_v,
             "Get integrated velocity change (sculling-compensated)")
        .def("get_delta_theta", &IMUSensor::get_delta_theta,
             "Get integrated angle change (coning-compensated)")
        .def("get_accel_bias", &IMUSensor::get_accel_bias,
             "Get current accelerometer bias estimate")
        .def("get_gyro_bias", &IMUSensor::get_gyro_bias,
             "Get current gyroscope bias estimate")
        .def("set_accel_bias", &IMUSensor::set_accel_bias,
             "Set accelerometer bias correction", py::arg("bias"))
        .def("set_gyro_bias", &IMUSensor::set_gyro_bias,
             "Set gyroscope bias correction", py::arg("bias"))
        .def("set_temperature", &IMUSensor::set_temperature,
             "Set operating temperature", py::arg("temperature"))
        .def("get_temperature", &IMUSensor::get_temperature, "Get temperature")
        .def("inject_failure", &IMUSensor::inject_failure,
             "Inject sensor failure", py::arg("mode"), py::arg("magnitude") = 1.0)
        .def("clear_failures", &IMUSensor::clear_failures, "Clear all failures")
        .def("get_state", &IMUSensor::get_state, "Get sensor operational state")
        .def("is_operational", &IMUSensor::is_operational, "Check if operational")
        .def("name", &IMUSensor::name, "Get sensor name")
        .def("id", &IMUSensor::id, "Get sensor ID")
        .def("set_seed", &IMUSensor::set_seed, "Set random seed", py::arg("seed"));

    // Factory functions for IMU
    m.def("create_consumer_imu", &create_consumer_imu,
          "Create consumer-grade IMU", py::arg("name"), py::arg("entity"));
    m.def("create_tactical_imu", &create_tactical_imu,
          "Create tactical-grade IMU", py::arg("name"), py::arg("entity"));
    m.def("create_navigation_imu", &create_navigation_imu,
          "Create navigation-grade IMU", py::arg("name"), py::arg("entity"));

    // ========================================================================
    // Turbulence Models
    // ========================================================================
    using namespace jaguar::environment;

    // Turbulence Severity
    py::enum_<TurbulenceSeverity>(m, "TurbulenceSeverity", "Turbulence intensity levels")
        .value("None_", TurbulenceSeverity::None)
        .value("Light", TurbulenceSeverity::Light)
        .value("Moderate", TurbulenceSeverity::Moderate)
        .value("Severe", TurbulenceSeverity::Severe)
        .value("Extreme", TurbulenceSeverity::Extreme)
        .export_values();

    // TurbulenceOutput
    py::class_<TurbulenceOutput>(m, "TurbulenceOutput", "Turbulence velocity and rate outputs")
        .def(py::init<>())
        .def_readwrite("velocity", &TurbulenceOutput::velocity,
                      "Linear velocity components in body frame (m/s)")
        .def_readwrite("angular_rate", &TurbulenceOutput::angular_rate,
                      "Angular rate components in body frame (rad/s)")
        .def("is_negligible", &TurbulenceOutput::is_negligible,
             "Check if turbulence is negligible", py::arg("threshold") = 1e-6);

    // DrydenTurbulenceModel
    py::class_<DrydenTurbulenceModel>(m, "DrydenTurbulenceModel",
        R"pbdoc(
        Dryden continuous gust turbulence model per MIL-F-8785C.

        Example::

            turb = jag.DrydenTurbulenceModel(jag.TurbulenceSeverity.Moderate)

            # In simulation loop:
            output = turb.update(airspeed, altitude_agl, wingspan, dt)
            body_velocity += output.velocity
            angular_rate += output.angular_rate
        )pbdoc")
        .def(py::init<>())
        .def(py::init<TurbulenceSeverity>(), py::arg("severity"))
        .def("update", &DrydenTurbulenceModel::update,
             "Update and get turbulence output",
             py::arg("airspeed"), py::arg("altitude_agl"),
             py::arg("wingspan"), py::arg("dt"))
        .def("reset", &DrydenTurbulenceModel::reset, "Reset turbulence state")
        .def("set_severity", &DrydenTurbulenceModel::set_severity,
             "Set turbulence severity", py::arg("severity"))
        .def("severity", &DrydenTurbulenceModel::severity, "Get severity level")
        .def("set_enabled", &DrydenTurbulenceModel::set_enabled,
             "Enable/disable turbulence", py::arg("enabled"))
        .def("is_enabled", &DrydenTurbulenceModel::is_enabled, "Check if enabled")
        .def("set_seed", &DrydenTurbulenceModel::set_seed,
             "Set random seed", py::arg("seed"))
        .def("set_intensity", &DrydenTurbulenceModel::set_intensity,
             "Set custom intensity values",
             py::arg("sigma_u"), py::arg("sigma_v"), py::arg("sigma_w"))
        .def("set_scale_lengths", &DrydenTurbulenceModel::set_scale_lengths,
             "Set custom scale lengths",
             py::arg("L_u"), py::arg("L_v"), py::arg("L_w"))
        .def("get_intensity", &DrydenTurbulenceModel::get_intensity,
             "Get current intensity values")
        .def("get_scale_lengths", &DrydenTurbulenceModel::get_scale_lengths,
             "Get current scale lengths")
        .def("last_output", &DrydenTurbulenceModel::last_output,
             "Get last computed output", py::return_value_policy::reference);

    // VonKarmanTurbulenceModel
    py::class_<VonKarmanTurbulenceModel>(m, "VonKarmanTurbulenceModel",
        "Von Karman turbulence model per MIL-STD-1797 (higher fidelity)")
        .def(py::init<>())
        .def(py::init<TurbulenceSeverity>(), py::arg("severity"))
        .def("update", &VonKarmanTurbulenceModel::update,
             "Update and get turbulence output",
             py::arg("airspeed"), py::arg("altitude_agl"),
             py::arg("wingspan"), py::arg("dt"))
        .def("reset", &VonKarmanTurbulenceModel::reset)
        .def("set_severity", &VonKarmanTurbulenceModel::set_severity, py::arg("severity"))
        .def("severity", &VonKarmanTurbulenceModel::severity)
        .def("set_enabled", &VonKarmanTurbulenceModel::set_enabled, py::arg("enabled"))
        .def("is_enabled", &VonKarmanTurbulenceModel::is_enabled)
        .def("set_seed", &VonKarmanTurbulenceModel::set_seed, py::arg("seed"))
        .def("set_filter_order", &VonKarmanTurbulenceModel::set_filter_order,
             "Set filter order (2-8)", py::arg("order"))
        .def("filter_order", &VonKarmanTurbulenceModel::filter_order);

    // Factory functions for turbulence
    m.def("create_dryden_turbulence", &create_dryden_turbulence,
          "Create Dryden turbulence model",
          py::arg("severity") = TurbulenceSeverity::Moderate);
    m.def("create_vonkarman_turbulence", &create_vonkarman_turbulence,
          "Create Von Karman turbulence model",
          py::arg("severity") = TurbulenceSeverity::Moderate);

    // ========================================================================
    // Version Information
    // ========================================================================
    m.attr("__version__") = GetVersionString();
    m.attr("VERSION_MAJOR") = VERSION_MAJOR;
    m.attr("VERSION_MINOR") = VERSION_MINOR;
    m.attr("VERSION_PATCH") = VERSION_PATCH;
    m.attr("__author__") = "JaguarEngine Team";
}
