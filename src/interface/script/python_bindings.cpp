/**
 * @file python_bindings.cpp
 * @brief Python bindings via pybind11 for JaguarEngine
 *
 * Provides Python API for simulation control, entity management,
 * and physics state access.
 */

#ifdef JAGUAR_BUILD_PYTHON

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>
#include <pybind11/numpy.h>

#include "jaguar/jaguar.h"
#include "jaguar/interface/api.h"
#include "jaguar/physics/entity.h"
#include "jaguar/core/types.h"

namespace py = pybind11;
using namespace jaguar;
using namespace jaguar::physics;

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
    // Version Information
    // ========================================================================
    m.attr("__version__") = "1.0.0";
    m.attr("__author__") = "JaguarEngine Team";
}

#endif // JAGUAR_BUILD_PYTHON
