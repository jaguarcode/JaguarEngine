# JaguarEngine Examples

This guide provides comprehensive examples for using JaguarEngine across all supported domains.

## Table of Contents

1. [Quick Start](#quick-start)
2. [Air Domain Examples](#air-domain-examples)
3. [Land Domain Examples](#land-domain-examples)
4. [Sea Domain Examples](#sea-domain-examples)
5. [Space Domain Examples](#space-domain-examples)
6. [Multi-Domain Simulation](#multi-domain-simulation)
7. [Environment Integration](#environment-integration)
8. [Advanced Patterns](#advanced-patterns)

## Quick Start

### Minimal Example

```cpp
#include "jaguar/jaguar.h"
#include <iostream>

int main() {
    // Create and initialize engine
    jaguar::Engine engine;
    if (!engine.initialize()) {
        std::cerr << "Engine initialization failed\n";
        return 1;
    }

    // Create an entity
    auto entity = engine.create_entity("MyEntity", jaguar::Domain::Air);

    // Set initial state
    jaguar::physics::EntityState state;
    state.position = {0, 0, -1000};  // 1 km altitude
    state.velocity = {100, 0, 0};    // 100 m/s forward
    state.mass = 1000;               // 1000 kg
    engine.set_entity_state(entity, state);

    // Run simulation
    engine.run_for(10.0);  // 10 seconds

    // Get final state
    auto final_state = engine.get_entity_state(entity);
    std::cout << "Final position: " << final_state.position.x << " m\n";

    engine.shutdown();
    return 0;
}
```

### Building Examples

```bash
# Build with examples enabled
cmake -B build -DJAGUAR_BUILD_EXAMPLES=ON
cmake --build build

# Run examples
./build/examples/simple_flight/simple_flight
./build/examples/multi_domain/multi_domain
```

## Air Domain Examples

### Simple Aircraft Simulation

```cpp
#include "jaguar/jaguar.h"
#include "jaguar/domain/air.h"

class SimpleAircraft {
public:
    SimpleAircraft(jaguar::Engine& engine, const std::string& name)
        : engine_(engine)
    {
        // Create entity
        entity_id_ = engine.create_entity(name, jaguar::Domain::Air);

        // Configure aerodynamics (F-16 approximate values)
        aero_.set_reference_area(27.87);    // m²
        aero_.set_reference_chord(3.45);    // m
        aero_.set_reference_span(9.45);     // m

        // Configure propulsion (F110 engine)
        engine_model_.set_max_thrust(131000.0);  // N (with afterburner)
        engine_model_.set_fuel_capacity(3200.0); // kg
        engine_model_.set_specific_fuel_consumption(2.5e-5);
        engine_model_.start();
    }

    void set_initial_state(const jaguar::Vec3& pos, const jaguar::Vec3& vel, jaguar::Real mass) {
        jaguar::physics::EntityState state;
        state.position = pos;
        state.velocity = vel;
        state.mass = mass;
        engine_.set_entity_state(entity_id_, state);
    }

    void set_throttle(jaguar::Real throttle) {
        engine_model_.set_throttle(std::clamp(throttle, 0.0, 1.0));
    }

    void set_controls(jaguar::Real pitch, jaguar::Real roll, jaguar::Real yaw) {
        controls_.pitch_cmd = pitch;
        controls_.roll_cmd = roll;
        controls_.yaw_cmd = yaw;
    }

    void update(jaguar::Real dt) {
        auto state = engine_.get_entity_state(entity_id_);
        jaguar::environment::Environment env = engine_.get_environment(entity_id_);

        forces_.clear();

        // Aerodynamic forces
        aero_.compute_forces(state, env, dt, forces_);

        // Propulsion forces
        engine_model_.compute_forces(state, env, dt, forces_);

        // Gravity
        jaguar::Vec3 gravity{0, 0, state.mass * jaguar::constants::G0};
        forces_.add_force(gravity);

        // Apply forces to entity
        engine_.apply_forces(entity_id_, forces_);
    }

    // Telemetry
    jaguar::Real get_altitude() const {
        return -engine_.get_entity_state(entity_id_).position.z;
    }

    jaguar::Real get_airspeed() const {
        auto state = engine_.get_entity_state(entity_id_);
        return state.velocity.norm();
    }

    jaguar::Real get_mach() const { return aero_.get_mach(); }
    jaguar::Real get_alpha_deg() const { return aero_.get_alpha() * jaguar::constants::RAD_TO_DEG; }
    jaguar::Real get_fuel_remaining() const { return engine_model_.get_fuel_remaining(); }

private:
    jaguar::Engine& engine_;
    jaguar::EntityId entity_id_;
    jaguar::domain::air::AerodynamicsModel aero_;
    jaguar::domain::air::PropulsionModel engine_model_;
    jaguar::domain::air::FlightControlSystem::ControlInputs controls_;
    jaguar::physics::EntityForces forces_;
};

int main() {
    jaguar::Engine engine;
    engine.initialize();

    SimpleAircraft f16(engine, "F-16");

    // Start at 10 km altitude, 250 m/s
    f16.set_initial_state(
        {0, 0, -10000},  // position (NED, so -Z is up)
        {250, 0, 0},     // velocity
        12000            // mass (kg)
    );

    f16.set_throttle(0.85);

    // Simulation loop
    const jaguar::Real dt = 0.01;  // 100 Hz
    for (jaguar::Real t = 0; t < 60.0; t += dt) {
        f16.update(dt);
        engine.step(dt);

        // Print telemetry every 5 seconds
        if (static_cast<int>(t * 100) % 500 == 0) {
            std::cout << "T=" << t << "s  Alt=" << f16.get_altitude()
                      << "m  Mach=" << f16.get_mach()
                      << "  Fuel=" << f16.get_fuel_remaining() << "kg\n";
        }
    }

    engine.shutdown();
    return 0;
}
```

### Custom Aerodynamic Tables

```cpp
#include "jaguar/domain/air.h"

void setup_aero_tables(jaguar::domain::air::AerodynamicsModel& aero) {
    // Create CL table (alpha vs Mach)
    auto cl_table = std::make_unique<jaguar::domain::air::AeroTable>();

    // Alpha breakpoints (degrees, converted to internal use)
    cl_table->set_breakpoints(0, {-10, -5, 0, 5, 10, 15, 20, 25});

    // Mach breakpoints
    cl_table->set_breakpoints(1, {0.0, 0.4, 0.8, 1.0, 1.2, 1.5});

    // CL data (8 alpha × 6 Mach = 48 values, row-major)
    cl_table->set_data({
        // Mach:  0.0    0.4    0.8    1.0    1.2    1.5
        /*α=-10*/ -0.80, -0.82, -0.78, -0.70, -0.60, -0.50,
        /*α=-5 */ -0.40, -0.41, -0.39, -0.35, -0.30, -0.25,
        /*α= 0 */  0.00,  0.00,  0.00,  0.00,  0.00,  0.00,
        /*α= 5 */  0.40,  0.41,  0.39,  0.35,  0.30,  0.25,
        /*α=10 */  0.80,  0.82,  0.78,  0.70,  0.60,  0.50,
        /*α=15 */  1.10,  1.13,  1.07,  0.95,  0.80,  0.65,
        /*α=20 */  1.20,  1.23,  1.15,  1.00,  0.85,  0.70,
        /*α=25 */  1.10,  1.13,  1.05,  0.90,  0.75,  0.60   // Post-stall
    });

    aero.set_cl_table(std::move(cl_table));

    // Similarly create CD, Cm tables...
}
```

## Land Domain Examples

### Tank Simulation

```cpp
#include "jaguar/jaguar.h"
#include "jaguar/domain/land.h"

class Tank {
public:
    Tank(jaguar::Engine& engine, const std::string& name)
        : engine_(engine)
    {
        entity_id_ = engine.create_entity(name, jaguar::Domain::Land);

        // Configure terramechanics (M1 Abrams dimensions)
        terra_.set_contact_area(0.63, 4.6);   // Track: 63cm wide, 4.6m long
        terra_.set_vehicle_weight(549000.0);  // 56 tonnes × 9.81

        // Configure tracks
        tracks_.set_sprocket(0.33, 100000.0);  // 33cm radius, 100 kN·m max torque

        // Configure suspension (6 road wheels per side)
        jaguar::domain::land::SuspensionUnit wheel;
        wheel.spring_k = 300000.0;   // 300 kN/m
        wheel.damper_c = 30000.0;    // 30 kN·s/m
        wheel.travel_max = 0.40;     // 40cm travel

        // Left side wheels
        for (jaguar::Real x = -3.5; x <= 2.0; x += 1.1) {
            suspension_.add_unit({x, 1.8, -0.9}, wheel);
        }
        // Right side wheels
        for (jaguar::Real x = -3.5; x <= 2.0; x += 1.1) {
            suspension_.add_unit({x, -1.8, -0.9}, wheel);
        }
    }

    void set_initial_state(const jaguar::Vec3& pos, jaguar::Real heading_rad) {
        jaguar::physics::EntityState state;
        state.position = pos;
        state.orientation = jaguar::Quaternion::from_euler(0, 0, heading_rad);
        state.mass = 56000;  // kg
        engine_.set_entity_state(entity_id_, state);
    }

    void set_throttle(jaguar::Real throttle) {
        throttle_ = std::clamp(throttle, -1.0, 1.0);  // Reverse allowed
    }

    void set_steering(jaguar::Real steering) {
        steering_ = std::clamp(steering, -1.0, 1.0);
    }

    void update(jaguar::Real dt) {
        auto state = engine_.get_entity_state(entity_id_);
        auto env = engine_.get_environment(entity_id_);

        // Get soil at current position
        jaguar::domain::land::SoilProperties soil;
        if (env.terrain.valid) {
            // Use terrain material properties
            soil.k_c = env.terrain.material.k_c;
            soil.k_phi = env.terrain.material.k_phi;
            soil.n = env.terrain.material.n;
        } else {
            // Default to dry sand
            soil = jaguar::domain::land::SoilProperties::DrySand();
        }

        forces_.clear();

        // Terramechanics
        terra_.compute_forces(state, env, dt, forces_);

        // Track dynamics
        jaguar::Real drive_torque = throttle_ * 100000.0;  // Max 100 kN·m
        jaguar::Real steer_bias = steering_ * 0.3;  // 30% differential

        jaguar::Real left_torque = drive_torque * (1.0 - steer_bias);
        jaguar::Real right_torque = drive_torque * (1.0 + steer_bias);

        tracks_.update(left_torque, right_torque, state.mass * 9.81, soil, dt);

        // Propulsive force from tracks
        jaguar::Vec3 prop_force{tracks_.get_propulsive_force(), 0, 0};
        forces_.add_force(prop_force);

        // Steering moment from differential
        jaguar::Real steer_moment = (right_torque - left_torque) * 1.8;  // Track spacing
        forces_.add_torque({0, 0, steer_moment});

        // Suspension
        suspension_.update(state, dt);
        forces_.add_force(suspension_.get_total_force());
        forces_.add_torque(suspension_.get_total_torque());

        // Gravity
        forces_.add_force({0, 0, state.mass * 9.81});

        engine_.apply_forces(entity_id_, forces_);
    }

    // Telemetry
    jaguar::Real get_speed() const {
        return engine_.get_entity_state(entity_id_).velocity.norm();
    }

    jaguar::Real get_slip() const {
        return (tracks_.get_left_track().slip + tracks_.get_right_track().slip) / 2.0;
    }

    jaguar::Real get_sinkage() const { return terra_.get_sinkage(); }

private:
    jaguar::Engine& engine_;
    jaguar::EntityId entity_id_;

    jaguar::domain::land::TerramechanicsModel terra_;
    jaguar::domain::land::TrackedVehicleModel tracks_;
    jaguar::domain::land::SuspensionModel suspension_;
    jaguar::physics::EntityForces forces_;

    jaguar::Real throttle_{0.0};
    jaguar::Real steering_{0.0};
};

int main() {
    jaguar::Engine engine;
    engine.initialize();

    Tank m1(engine, "M1A2");
    m1.set_initial_state({0, 0, 0}, 0.0);  // Origin, heading north

    // Accelerate forward
    m1.set_throttle(0.8);

    const jaguar::Real dt = 0.02;  // 50 Hz
    for (jaguar::Real t = 0; t < 30.0; t += dt) {
        // Turn right at t=10s
        if (t > 10.0 && t < 15.0) {
            m1.set_steering(0.5);
        } else {
            m1.set_steering(0.0);
        }

        m1.update(dt);
        engine.step(dt);

        if (static_cast<int>(t * 50) % 50 == 0) {
            std::cout << "T=" << t << "s  Speed=" << m1.get_speed() * 3.6
                      << "km/h  Slip=" << m1.get_slip() * 100 << "%\n";
        }
    }

    engine.shutdown();
    return 0;
}
```

## Sea Domain Examples

### Surface Ship Simulation

```cpp
#include "jaguar/jaguar.h"
#include "jaguar/domain/sea.h"

class SurfaceShip {
public:
    SurfaceShip(jaguar::Engine& engine, const std::string& name)
        : engine_(engine)
    {
        entity_id_ = engine.create_entity(name, jaguar::Domain::Sea);

        // Destroyer dimensions (~DDG-51)
        const jaguar::Real length = 154.0;  // m
        const jaguar::Real beam = 20.0;     // m
        const jaguar::Real draft = 9.4;     // m

        // Buoyancy configuration
        buoyancy_.set_displaced_volume(8400.0);  // m³
        buoyancy_.set_metacentric_height(2.5);   // Good stability
        buoyancy_.set_center_of_buoyancy({0, 0, -draft/2});

        // Hydrodynamics (MMG model)
        hydro_.set_hull_coefficients(-0.04, -0.01, -0.4, 0.05, -0.1, -0.05);
        hydro_.set_rudder_parameters(18.0, 1.6);    // 18 m² rudder, AR=1.6
        hydro_.set_propeller_parameters(5.2, 1.0);  // 5.2m diameter

        // Wave model
        waves_.set_sea_state(jaguar::domain::sea::SeaState::FromNATOSeaState(4));

        // RAO (Response Amplitude Operators)
        setup_rao();
    }

    void set_initial_state(const jaguar::Vec3& pos, jaguar::Real heading) {
        jaguar::physics::EntityState state;
        state.position = pos;
        state.orientation = jaguar::Quaternion::from_euler(0, 0, heading);
        state.mass = 8600000;  // 8600 tonnes
        engine_.set_entity_state(entity_id_, state);
    }

    void set_throttle(jaguar::Real throttle) {
        hydro_.set_propeller_rpm(throttle * 180.0);  // Max 180 RPM
    }

    void set_rudder(jaguar::Real command) {
        hydro_.set_rudder_angle(command * 0.61);  // Max 35°
    }

    void update(jaguar::Real dt) {
        auto state = engine_.get_entity_state(entity_id_);
        auto env = engine_.get_environment(entity_id_);

        // Update environment with wave surface
        env.ocean.surface_elevation = waves_.get_elevation(
            state.position.x, state.position.y, sim_time_);
        env.over_water = true;

        forces_.clear();

        // Buoyancy and hydrostatic stability
        buoyancy_.compute_forces(state, env, dt, forces_);

        // Hydrodynamic forces (maneuvering)
        hydro_.compute_forces(state, env, dt, forces_);

        // Wave-induced motion (seakeeping)
        jaguar::Vec3 wave_disp, wave_rot;
        rao_.calculate_response(waves_, sim_time_, wave_disp, wave_rot);

        // Apply wave forces (simplified)
        forces_.add_force({0, 0, -wave_disp.z * state.mass * 0.1});

        // Gravity
        forces_.add_force({0, 0, state.mass * 9.81});

        engine_.apply_forces(entity_id_, forces_);
        sim_time_ += dt;
    }

    // Telemetry
    jaguar::Real get_speed_knots() const {
        auto state = engine_.get_entity_state(entity_id_);
        jaguar::Real speed_ms = std::sqrt(
            state.velocity.x * state.velocity.x +
            state.velocity.y * state.velocity.y);
        return speed_ms * 1.94384;  // m/s to knots
    }

    jaguar::Real get_heading_deg() const {
        auto state = engine_.get_entity_state(entity_id_);
        return state.get_yaw() * jaguar::constants::RAD_TO_DEG;
    }

    jaguar::Real get_roll_deg() const {
        return buoyancy_.get_heel() * jaguar::constants::RAD_TO_DEG;
    }

    jaguar::Real get_pitch_deg() const {
        return buoyancy_.get_trim() * jaguar::constants::RAD_TO_DEG;
    }

private:
    void setup_rao() {
        // Heave RAO
        std::vector<jaguar::Real> freqs = {0.3, 0.5, 0.7, 1.0, 1.5};
        std::vector<jaguar::Real> heave_amp = {0.9, 1.0, 0.95, 0.7, 0.4};
        std::vector<jaguar::Real> heave_phase = {0.0, 0.1, 0.2, 0.4, 0.6};
        rao_.set_rao(2, freqs, heave_amp, heave_phase);

        // Roll RAO
        std::vector<jaguar::Real> roll_amp = {0.02, 0.05, 0.08, 0.04, 0.02};
        std::vector<jaguar::Real> roll_phase = {0.0, 0.2, 0.5, 0.8, 1.0};
        rao_.set_rao(3, freqs, roll_amp, roll_phase);

        // Pitch RAO
        std::vector<jaguar::Real> pitch_amp = {0.01, 0.02, 0.04, 0.03, 0.01};
        std::vector<jaguar::Real> pitch_phase = {0.0, 0.1, 0.3, 0.5, 0.7};
        rao_.set_rao(4, freqs, pitch_amp, pitch_phase);
    }

    jaguar::Engine& engine_;
    jaguar::EntityId entity_id_;

    jaguar::domain::sea::BuoyancyModel buoyancy_;
    jaguar::domain::sea::HydrodynamicsModel hydro_;
    jaguar::domain::sea::WaveModel waves_;
    jaguar::domain::sea::RAOModel rao_;
    jaguar::physics::EntityForces forces_;

    jaguar::Real sim_time_{0.0};
};

int main() {
    jaguar::Engine engine;
    engine.initialize();

    SurfaceShip ddg(engine, "DDG-51");
    ddg.set_initial_state({0, 0, 0}, 0.0);  // At origin, heading north

    ddg.set_throttle(0.7);  // 70% power

    const jaguar::Real dt = 0.05;  // 20 Hz
    for (jaguar::Real t = 0; t < 120.0; t += dt) {
        // Execute turn at t=30s
        if (t > 30.0 && t < 50.0) {
            ddg.set_rudder(0.5);  // Starboard turn
        } else {
            ddg.set_rudder(0.0);
        }

        ddg.update(dt);
        engine.step(dt);

        if (static_cast<int>(t * 20) % 100 == 0) {
            std::cout << "T=" << t << "s  Speed=" << ddg.get_speed_knots()
                      << "kts  Hdg=" << ddg.get_heading_deg()
                      << "°  Roll=" << ddg.get_roll_deg() << "°\n";
        }
    }

    engine.shutdown();
    return 0;
}
```

## Space Domain Examples

### Satellite Orbit Propagation

```cpp
#include "jaguar/jaguar.h"
#include "jaguar/domain/space.h"

class Satellite {
public:
    Satellite(jaguar::Engine& engine, const std::string& name)
        : engine_(engine)
    {
        entity_id_ = engine.create_entity(name, jaguar::Domain::Space);

        // Configure gravity model (J4 for LEO accuracy)
        gravity_.set_fidelity(jaguar::domain::space::GravityFidelity::J4);

        // Configure atmospheric drag
        drag_.set_drag_coefficient(2.2);
        drag_.set_area(10.0);  // m² cross-sectional area
    }

    void set_from_tle(const std::string& line1, const std::string& line2) {
        jaguar::domain::space::TLE tle;
        if (!tle.parse(line1, line2)) {
            throw std::runtime_error("TLE parse failed");
        }

        // Initialize SGP4 propagator
        sgp4_.initialize(tle);

        // Get initial state
        jaguar::Vec3 pos_eci, vel_eci;
        sgp4_.propagate(0.0, pos_eci, vel_eci);

        jaguar::physics::EntityState state;
        state.position = pos_eci;
        state.velocity = vel_eci;
        state.mass = 500.0;  // kg
        engine_.set_entity_state(entity_id_, state);
    }

    void set_from_elements(const jaguar::domain::space::OrbitalElements& elements) {
        elements_ = elements;

        // Convert to Cartesian (simplified)
        jaguar::Vec3 pos_eci, vel_eci;
        elements_to_cartesian(elements, pos_eci, vel_eci);

        jaguar::physics::EntityState state;
        state.position = pos_eci;
        state.velocity = vel_eci;
        state.mass = 500.0;
        engine_.set_entity_state(entity_id_, state);
    }

    void update(jaguar::Real dt) {
        auto state = engine_.get_entity_state(entity_id_);
        auto env = engine_.get_environment(entity_id_);

        forces_.clear();

        // Gravitational acceleration (includes J2-J4 perturbations)
        gravity_.compute_forces(state, env, dt, forces_);

        // Atmospheric drag (significant below ~600 km)
        jaguar::Real altitude = state.position.norm() - 6378137.0;
        if (altitude < 600000.0) {
            drag_.compute_forces(state, env, dt, forces_);
        }

        engine_.apply_forces(entity_id_, forces_);
    }

    // Telemetry
    jaguar::Real get_altitude_km() const {
        auto state = engine_.get_entity_state(entity_id_);
        return (state.position.norm() - 6378137.0) / 1000.0;
    }

    jaguar::Real get_velocity_kms() const {
        auto state = engine_.get_entity_state(entity_id_);
        return state.velocity.norm() / 1000.0;
    }

    jaguar::domain::space::OrbitalElements get_elements() const {
        auto state = engine_.get_entity_state(entity_id_);
        return cartesian_to_elements(state.position, state.velocity);
    }

private:
    void elements_to_cartesian(const jaguar::domain::space::OrbitalElements& el,
                               jaguar::Vec3& pos, jaguar::Vec3& vel) {
        // Simplified conversion (see space documentation for full implementation)
        const jaguar::Real mu = 3.986004418e14;  // Earth GM

        jaguar::Real a = el.semi_major_axis;
        jaguar::Real e = el.eccentricity;
        jaguar::Real p = a * (1.0 - e * e);

        // Assume at periapsis for simplicity
        jaguar::Real r = p / (1.0 + e);
        jaguar::Real v = std::sqrt(mu * (2.0/r - 1.0/a));

        // Rotate to orbital plane
        pos = {r, 0, 0};
        vel = {0, v, 0};

        // Apply inclination and RAAN rotations (simplified)
        // Full implementation rotates by Omega, i, omega
    }

    jaguar::domain::space::OrbitalElements cartesian_to_elements(
        const jaguar::Vec3& pos, const jaguar::Vec3& vel) const {
        // Standard conversion from r, v to classical elements
        const jaguar::Real mu = 3.986004418e14;

        jaguar::domain::space::OrbitalElements el;

        jaguar::Real r = pos.norm();
        jaguar::Real v = vel.norm();

        // Specific energy
        jaguar::Real energy = v*v/2.0 - mu/r;
        el.semi_major_axis = -mu / (2.0 * energy);

        // Angular momentum
        jaguar::Vec3 h = pos.cross(vel);
        jaguar::Real h_mag = h.norm();

        // Eccentricity vector
        jaguar::Vec3 e_vec = vel.cross(h) / mu - pos / r;
        el.eccentricity = e_vec.norm();

        // Inclination
        el.inclination = std::acos(h.z / h_mag);

        // Other elements require more complex computation...
        return el;
    }

    jaguar::Engine& engine_;
    jaguar::EntityId entity_id_;

    jaguar::domain::space::OrbitalElements elements_;
    jaguar::domain::space::SGP4Propagator sgp4_;
    jaguar::domain::space::GravityModel gravity_;
    jaguar::domain::space::AtmosphericDragModel drag_;
    jaguar::physics::EntityForces forces_;
};

int main() {
    jaguar::Engine engine;
    engine.initialize();

    Satellite iss(engine, "ISS");

    // ISS-like orbit
    jaguar::domain::space::OrbitalElements orbit;
    orbit.semi_major_axis = 6778000.0;  // ~400 km altitude
    orbit.eccentricity = 0.0002;
    orbit.inclination = 51.6 * jaguar::constants::DEG_TO_RAD;
    orbit.raan = 0.0;
    orbit.arg_periapsis = 0.0;
    orbit.mean_anomaly = 0.0;
    orbit.epoch = 0.0;

    iss.set_from_elements(orbit);

    // Simulate one orbit (~90 minutes)
    const jaguar::Real dt = 10.0;  // 10 second steps
    const jaguar::Real orbit_period = 5400.0;  // ~90 minutes

    for (jaguar::Real t = 0; t < orbit_period; t += dt) {
        iss.update(dt);
        engine.step(dt);

        if (static_cast<int>(t) % 600 == 0) {  // Every 10 minutes
            auto el = iss.get_elements();
            std::cout << "T=" << t/60.0 << "min  Alt=" << iss.get_altitude_km()
                      << "km  V=" << iss.get_velocity_kms() << "km/s\n";
        }
    }

    engine.shutdown();
    return 0;
}
```

## Multi-Domain Simulation

### Combined Air-Land-Sea-Space Example

```cpp
#include "jaguar/jaguar.h"
#include <iostream>
#include <iomanip>

int main() {
    std::cout << "JaguarEngine Multi-Domain Example\n";
    std::cout << "Version: " << jaguar::GetVersionString() << "\n\n";

    jaguar::Engine engine;
    if (!engine.initialize()) {
        std::cerr << "Failed to initialize engine\n";
        return 1;
    }

    // Create entities in different domains
    auto aircraft = engine.create_entity("F-16", jaguar::Domain::Air);
    auto tank = engine.create_entity("M1A2", jaguar::Domain::Land);
    auto ship = engine.create_entity("DDG-51", jaguar::Domain::Sea);
    auto satellite = engine.create_entity("GPS-IIR", jaguar::Domain::Space);

    // Aircraft at 10 km altitude
    {
        jaguar::physics::EntityState s;
        s.position = {0, 0, -10000};
        s.velocity = {250, 0, 0};
        s.mass = 12000;
        engine.set_entity_state(aircraft, s);
    }

    // Tank on ground
    {
        jaguar::physics::EntityState s;
        s.position = {1000, 0, 0};
        s.velocity = {10, 0, 0};
        s.mass = 60000;
        engine.set_entity_state(tank, s);
    }

    // Ship at sea
    {
        jaguar::physics::EntityState s;
        s.position = {0, 5000, 0};
        s.velocity = {15, 0, 0};
        s.mass = 9000000;
        engine.set_entity_state(ship, s);
    }

    // Satellite in orbit (400 km)
    {
        jaguar::physics::EntityState s;
        s.position = {6778137, 0, 0};  // r = Re + 400km
        s.velocity = {0, 7670, 0};     // Circular orbit velocity
        s.mass = 2000;
        engine.set_entity_state(satellite, s);
    }

    // Run simulation
    std::cout << "Simulating all domains simultaneously...\n\n";

    const jaguar::Real dt = 0.01;
    for (jaguar::Real t = 0; t < 60.0; t += dt) {
        engine.step(dt);

        // Print status every 10 seconds
        if (static_cast<int>(t * 100) % 1000 == 0) {
            std::cout << "=== T = " << t << " s ===\n";

            auto air_state = engine.get_entity_state(aircraft);
            std::cout << "Aircraft: Alt=" << -air_state.position.z
                      << "m, Speed=" << air_state.velocity.norm() << "m/s\n";

            auto land_state = engine.get_entity_state(tank);
            std::cout << "Tank: Pos=(" << land_state.position.x << ","
                      << land_state.position.y << "), Speed="
                      << land_state.velocity.norm() << "m/s\n";

            auto sea_state = engine.get_entity_state(ship);
            std::cout << "Ship: Pos=(" << sea_state.position.x << ","
                      << sea_state.position.y << "), Speed="
                      << sea_state.velocity.norm() * 1.94384 << "kts\n";

            auto space_state = engine.get_entity_state(satellite);
            jaguar::Real sat_alt = (space_state.position.norm() - 6378137) / 1000;
            std::cout << "Satellite: Alt=" << sat_alt << "km, Speed="
                      << space_state.velocity.norm() / 1000 << "km/s\n\n";
        }
    }

    engine.shutdown();
    return 0;
}
```

## Environment Integration

### Weather Effects Example

```cpp
#include "jaguar/jaguar.h"
#include "jaguar/environment/environment.h"

void setup_weather(jaguar::environment::EnvironmentService& env) {
    auto& weather = env.atmosphere().weather();

    // Wind profile (jet stream pattern)
    weather.set_wind_layer(0.0, 270.0 * jaguar::constants::DEG_TO_RAD, 5.0);
    weather.set_wind_layer(3000.0, 280.0 * jaguar::constants::DEG_TO_RAD, 20.0);
    weather.set_wind_layer(9000.0, 300.0 * jaguar::constants::DEG_TO_RAD, 50.0);
    weather.set_wind_layer(12000.0, 290.0 * jaguar::constants::DEG_TO_RAD, 80.0);

    // Rain
    weather.set_rain_rate(5.0);  // Light rain

    // Reduced visibility
    weather.set_fog_visibility(5000.0);  // 5 km visibility

    // Ocean state
    auto sea_state = jaguar::domain::sea::SeaState::FromNATOSeaState(5);  // Rough
    env.ocean().set_sea_state(sea_state);
    env.ocean().set_current({0.5, 0.3, 0.0});  // Half-knot current
}

void query_environment_example(jaguar::environment::EnvironmentService& env) {
    // Query at specific location
    jaguar::Real lat = 37.5 * jaguar::constants::DEG_TO_RAD;
    jaguar::Real lon = -122.0 * jaguar::constants::DEG_TO_RAD;
    jaguar::Real alt = 5000.0;

    auto state = env.atmosphere().get_state(lat, lon, alt);

    std::cout << "Atmosphere at " << alt << "m:\n";
    std::cout << "  Temperature: " << state.temperature - 273.15 << " °C\n";
    std::cout << "  Pressure: " << state.pressure / 100 << " hPa\n";
    std::cout << "  Density: " << state.density << " kg/m³\n";
    std::cout << "  Wind: (" << state.wind.x << ", " << state.wind.y << ") m/s\n";
    std::cout << "  Visibility: " << state.visibility << " m\n";
}
```

### Terrain Integration Example

```cpp
#include "jaguar/environment/terrain.h"

void terrain_example() {
    jaguar::environment::TerrainManager terrain;

    // Configure data sources
    terrain.add_data_path("/data/dted/");
    terrain.set_cache_size(256);  // 256 MB

    if (!terrain.initialize()) {
        std::cerr << "Terrain initialization failed\n";
        return;
    }

    // Query specific location
    jaguar::Real lat = 36.0 * jaguar::constants::DEG_TO_RAD;
    jaguar::Real lon = -118.0 * jaguar::constants::DEG_TO_RAD;

    auto query = terrain.query(lat, lon);

    if (query.valid) {
        std::cout << "Terrain at location:\n";
        std::cout << "  Elevation: " << query.elevation << " m\n";
        std::cout << "  Slope: " << query.slope_angle * jaguar::constants::RAD_TO_DEG << "°\n";
        std::cout << "  Surface: " << static_cast<int>(query.material.type) << "\n";
        std::cout << "  Friction: " << query.material.friction_coefficient << "\n";
    }

    terrain.shutdown();
}
```

## Advanced Patterns

### Custom Force Generator

```cpp
#include "jaguar/physics/force.h"

class WindTurbulenceModel : public jaguar::physics::IAerodynamicsModel {
public:
    void set_intensity(jaguar::Real intensity) { intensity_ = intensity; }

    void compute_forces(const jaguar::physics::EntityState& state,
                        const jaguar::environment::Environment& env,
                        jaguar::Real dt,
                        jaguar::physics::EntityForces& forces) override {
        // Generate turbulent gusts
        jaguar::Real gust_x = intensity_ * (rand_normal() * 5.0);
        jaguar::Real gust_y = intensity_ * (rand_normal() * 5.0);
        jaguar::Real gust_z = intensity_ * (rand_normal() * 2.0);

        // Scale by dynamic pressure
        jaguar::Real qbar = 0.5 * env.atmosphere.density *
                           state.velocity.norm() * state.velocity.norm();

        jaguar::Vec3 turbulence_force{
            gust_x * qbar * reference_area_,
            gust_y * qbar * reference_area_,
            gust_z * qbar * reference_area_
        };

        forces.add_force(turbulence_force);
    }

    // IAerodynamicsModel interface
    jaguar::Real get_cl() const override { return 0.0; }
    jaguar::Real get_cd() const override { return 0.0; }
    jaguar::Real get_cm() const override { return 0.0; }
    jaguar::Real get_alpha() const override { return 0.0; }
    jaguar::Real get_beta() const override { return 0.0; }
    jaguar::Real get_mach() const override { return 0.0; }
    jaguar::Real get_qbar() const override { return 0.0; }

private:
    jaguar::Real rand_normal() {
        // Box-Muller transform
        static bool has_spare = false;
        static jaguar::Real spare;

        if (has_spare) {
            has_spare = false;
            return spare;
        }

        jaguar::Real u, v, s;
        do {
            u = (rand() / (jaguar::Real)RAND_MAX) * 2.0 - 1.0;
            v = (rand() / (jaguar::Real)RAND_MAX) * 2.0 - 1.0;
            s = u * u + v * v;
        } while (s >= 1.0 || s == 0.0);

        s = std::sqrt(-2.0 * std::log(s) / s);
        spare = v * s;
        has_spare = true;
        return u * s;
    }

    jaguar::Real intensity_{1.0};
    jaguar::Real reference_area_{30.0};  // m²
};
```

### Entity Callbacks

```cpp
#include "jaguar/jaguar.h"

class SimulationCallbacks {
public:
    void on_step(jaguar::Engine& engine, jaguar::Real dt) {
        // Check all aircraft for stall warning
        for (auto& [id, aircraft] : aircraft_map_) {
            auto state = engine.get_entity_state(id);
            jaguar::Real alpha = aircraft.aero.get_alpha();

            if (std::abs(alpha) > stall_alpha_) {
                std::cout << "WARNING: Aircraft " << id
                          << " approaching stall (alpha="
                          << alpha * jaguar::constants::RAD_TO_DEG << "°)\n";
            }
        }

        // Check ships for excessive roll
        for (auto& [id, ship] : ship_map_) {
            jaguar::Real roll = ship.buoyancy.get_heel();
            if (std::abs(roll) > max_roll_) {
                std::cout << "WARNING: Ship " << id
                          << " excessive roll ("
                          << roll * jaguar::constants::RAD_TO_DEG << "°)\n";
            }
        }
    }

private:
    std::map<jaguar::EntityId, AircraftData> aircraft_map_;
    std::map<jaguar::EntityId, ShipData> ship_map_;
    jaguar::Real stall_alpha_{0.26};  // ~15 degrees
    jaguar::Real max_roll_{0.52};     // ~30 degrees
};
```

### Performance Optimization

```cpp
// Batch entity updates for better cache performance
void update_entities_batched(jaguar::Engine& engine,
                             const std::vector<jaguar::EntityId>& entities,
                             jaguar::Real dt) {
    // Pre-fetch all states
    std::vector<jaguar::physics::EntityState> states;
    states.reserve(entities.size());
    for (auto id : entities) {
        states.push_back(engine.get_entity_state(id));
    }

    // Pre-fetch all environments
    std::vector<jaguar::environment::Environment> envs;
    envs.reserve(entities.size());
    for (const auto& state : states) {
        envs.push_back(engine.get_environment_at(state.position));
    }

    // Process all entities
    std::vector<jaguar::physics::EntityForces> all_forces(entities.size());

    #pragma omp parallel for  // OpenMP parallelization
    for (size_t i = 0; i < entities.size(); ++i) {
        // Compute forces for entity i
        compute_entity_forces(states[i], envs[i], dt, all_forces[i]);
    }

    // Apply all forces
    for (size_t i = 0; i < entities.size(); ++i) {
        engine.apply_forces(entities[i], all_forces[i]);
    }
}
```

## Next Steps

- See [API Reference](API_REFERENCE.md) for complete API documentation
- See [Configuration](CONFIGURATION.md) for XML configuration options
- See domain-specific documentation in [modules/](modules/) for detailed physics models
