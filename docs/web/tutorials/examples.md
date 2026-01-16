# Examples Guide

Comprehensive examples for using JaguarEngine across all supported domains.

## Overview

| Example | Domain | Complexity | Description |
|---------|--------|------------|-------------|
| [Minimal](#minimal-example) | Any | Beginner | Basic engine usage |
| [Simple Flight](#simple-aircraft) | Air | Beginner | Aircraft simulation |
| [Tank Simulation](#tank-simulation) | Land | Intermediate | Ground vehicle |
| [Ship Maneuvering](#ship-maneuvering) | Sea | Intermediate | Naval vessel |
| [Satellite Orbit](#satellite-orbit) | Space | Intermediate | Orbital mechanics |
| [Multi-Domain](#multi-domain-simulation) | All | Advanced | Combined domains |
| [Custom Forces](#custom-force-generator) | Any | Advanced | Extending physics |

---

## Minimal Example

The simplest possible JaguarEngine program:

```cpp
#include <jaguar/jaguar.h>
#include <iostream>

int main() {
    // Create and initialize engine
    jaguar::interface::Engine engine;
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

    // Run simulation for 10 seconds
    engine.run_for(10.0);

    // Get final state
    auto final_state = engine.get_entity_state(entity);
    std::cout << "Final X position: " << final_state.position.x << " m\n";

    engine.shutdown();
    return 0;
}
```

---

## Simple Aircraft

A complete aircraft simulation with aerodynamics and propulsion:

```cpp
#include <jaguar/jaguar.h>
#include <jaguar/domain/air.h>
#include <iostream>
#include <iomanip>

int main() {
    std::cout << "F-16 Flight Simulation\n\n";

    // Initialize engine
    jaguar::interface::Engine engine;
    engine.initialize();

    // Create aircraft entity
    auto aircraft = engine.create_entity("F-16", jaguar::Domain::Air);

    // Configure aerodynamics
    jaguar::domain::air::AerodynamicsModel aero;
    aero.set_reference_area(27.87);    // m²
    aero.set_reference_chord(3.45);    // m
    aero.set_reference_span(9.45);     // m

    // Configure propulsion
    jaguar::domain::air::PropulsionModel propulsion;
    propulsion.set_max_thrust(131000.0);  // N
    propulsion.set_fuel_capacity(3200.0); // kg
    propulsion.set_specific_fuel_consumption(2.5e-5);
    propulsion.start();
    propulsion.set_throttle(0.85);  // 85% power

    // Set initial state (10 km altitude, 250 m/s)
    jaguar::physics::EntityState state;
    state.position = {0, 0, -10000};
    state.velocity = {250, 0, 0};
    state.orientation = jaguar::Quaternion::identity();
    state.mass = 12000;
    engine.set_entity_state(aircraft, state);

    // Print header
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "Time(s) | Alt(m)    | Speed(m/s) | Mach  | Fuel(kg)\n";
    std::cout << "--------|-----------|------------|-------|----------\n";

    // Simulation loop
    const jaguar::Real dt = 0.01;  // 100 Hz
    jaguar::physics::EntityForces forces;

    for (jaguar::Real t = 0; t < 60.0; t += dt) {
        // Get current state and environment
        auto s = engine.get_entity_state(aircraft);
        auto env = engine.get_environment(aircraft);

        // Compute forces
        forces.clear();
        aero.compute_forces(s, env, dt, forces);
        propulsion.compute_forces(s, env, dt, forces);

        // Add gravity
        forces.add_force({0, 0, s.mass * jaguar::constants::G0});

        // Apply and step
        engine.apply_forces(aircraft, forces);
        engine.step(dt);

        // Print telemetry every 5 seconds
        if (static_cast<int>(t * 100) % 500 == 0) {
            std::cout << std::setw(7) << t << " | "
                      << std::setw(9) << -s.position.z << " | "
                      << std::setw(10) << s.velocity.norm() << " | "
                      << std::setw(5) << std::setprecision(2) << aero.get_mach() << " | "
                      << std::setw(8) << std::setprecision(1)
                      << propulsion.get_fuel_remaining() << "\n";
        }
    }

    engine.shutdown();
    return 0;
}
```

---

## Tank Simulation

Ground vehicle with terramechanics and suspension:

```cpp
#include <jaguar/jaguar.h>
#include <jaguar/domain/land.h>
#include <iostream>

int main() {
    std::cout << "M1A2 Abrams Tank Simulation\n\n";

    jaguar::interface::Engine engine;
    engine.initialize();

    // Create tank entity
    auto tank = engine.create_entity("M1A2", jaguar::Domain::Land);

    // Configure terramechanics (track-soil interaction)
    jaguar::domain::land::TerramechanicsModel terra;
    terra.set_contact_area(0.63, 4.6);   // Track: 63cm × 4.6m
    terra.set_vehicle_weight(549000.0);  // 56 tonnes × 9.81

    // Configure tracks
    jaguar::domain::land::TrackedVehicleModel tracks;
    tracks.set_sprocket(0.33, 100000.0); // 33cm radius, 100 kN·m max

    // Configure suspension
    jaguar::domain::land::SuspensionModel suspension;
    jaguar::domain::land::SuspensionUnit wheel;
    wheel.spring_k = 300000.0;   // 300 kN/m
    wheel.damper_c = 30000.0;    // 30 kN·s/m
    wheel.travel_max = 0.40;     // 40cm travel

    // Add road wheels (6 per side)
    for (jaguar::Real x = -3.5; x <= 2.0; x += 1.1) {
        suspension.add_unit({x, 1.8, -0.9}, wheel);   // Left
        suspension.add_unit({x, -1.8, -0.9}, wheel);  // Right
    }

    // Set initial state
    jaguar::physics::EntityState state;
    state.position = {0, 0, 0};
    state.orientation = jaguar::Quaternion::identity();
    state.mass = 56000;  // kg
    engine.set_entity_state(tank, state);

    // Simulation variables
    jaguar::Real throttle = 0.8;  // 80% power
    jaguar::Real steering = 0.0;

    std::cout << "Time(s) | Speed(km/h) | Slip(%) | Sinkage(cm)\n";
    std::cout << "--------|-------------|---------|------------\n";

    const jaguar::Real dt = 0.02;  // 50 Hz
    jaguar::physics::EntityForces forces;
    auto soil = jaguar::domain::land::SoilProperties::DrySand();

    for (jaguar::Real t = 0; t < 30.0; t += dt) {
        auto s = engine.get_entity_state(tank);
        auto env = engine.get_environment(tank);

        forces.clear();

        // Terramechanics forces
        terra.compute_forces(s, env, dt, forces);

        // Track dynamics
        jaguar::Real drive_torque = throttle * 100000.0;
        jaguar::Real steer_bias = steering * 0.3;
        jaguar::Real left_torque = drive_torque * (1.0 - steer_bias);
        jaguar::Real right_torque = drive_torque * (1.0 + steer_bias);

        tracks.update(left_torque, right_torque, s.mass * 9.81, soil, dt);
        forces.add_force({tracks.get_propulsive_force(), 0, 0});

        // Steering moment
        jaguar::Real steer_moment = (right_torque - left_torque) * 1.8;
        forces.add_torque({0, 0, steer_moment});

        // Suspension
        suspension.update(s, dt);
        forces.add_force(suspension.get_total_force());
        forces.add_torque(suspension.get_total_torque());

        // Gravity
        forces.add_force({0, 0, s.mass * 9.81});

        engine.apply_forces(tank, forces);
        engine.step(dt);

        // Turn right at t=10s
        if (t > 10.0 && t < 15.0) {
            steering = 0.5;
        } else {
            steering = 0.0;
        }

        // Print every second
        if (static_cast<int>(t * 50) % 50 == 0) {
            jaguar::Real speed_kmh = s.velocity.norm() * 3.6;
            jaguar::Real avg_slip = (tracks.get_left_track().slip +
                                     tracks.get_right_track().slip) / 2.0;
            std::cout << std::fixed << std::setprecision(1)
                      << std::setw(7) << t << " | "
                      << std::setw(11) << speed_kmh << " | "
                      << std::setw(7) << avg_slip * 100 << " | "
                      << std::setw(10) << terra.get_sinkage() * 100 << "\n";
        }
    }

    engine.shutdown();
    return 0;
}
```

---

## Ship Maneuvering

Destroyer maneuvering simulation with hydrodynamics:

```cpp
#include <jaguar/jaguar.h>
#include <jaguar/domain/sea.h>
#include <iostream>

int main() {
    std::cout << "DDG-51 Arleigh Burke Destroyer Simulation\n\n";

    jaguar::interface::Engine engine;
    engine.initialize();

    auto ship = engine.create_entity("DDG-51", jaguar::Domain::Sea);

    // Configure buoyancy (8400 tonnes displacement)
    jaguar::domain::sea::BuoyancyModel buoyancy;
    buoyancy.set_displaced_volume(8400.0);
    buoyancy.set_metacentric_height(2.5);
    buoyancy.set_center_of_buoyancy({0, 0, -4.7});

    // Configure hydrodynamics (MMG model)
    jaguar::domain::sea::HydrodynamicsModel hydro;
    hydro.set_hull_coefficients(-0.04, -0.01, -0.4, 0.05, -0.1, -0.05);
    hydro.set_rudder_parameters(18.0, 1.6);     // 18 m² rudder
    hydro.set_propeller_parameters(5.2, 1.0);   // 5.2m propeller

    // Configure waves (Sea State 4)
    jaguar::domain::sea::WaveModel waves;
    auto sea_state = jaguar::domain::sea::SeaState::FromNATOSeaState(4);
    waves.set_sea_state(sea_state);

    // Set initial state
    jaguar::physics::EntityState state;
    state.position = {0, 0, 0};
    state.orientation = jaguar::Quaternion::identity();
    state.mass = 8600000;  // 8600 tonnes in kg
    engine.set_entity_state(ship, state);

    // Control inputs
    hydro.set_propeller_rpm(126.0);  // 70% power ≈ 180 RPM max
    hydro.set_rudder_angle(0.0);

    std::cout << "Time(s) | Speed(kt) | Heading(°) | Roll(°) | Pitch(°)\n";
    std::cout << "--------|-----------|------------|---------|----------\n";

    const jaguar::Real dt = 0.05;  // 20 Hz
    jaguar::physics::EntityForces forces;
    jaguar::Real sim_time = 0.0;

    for (jaguar::Real t = 0; t < 120.0; t += dt) {
        auto s = engine.get_entity_state(ship);
        auto env = engine.get_environment(ship);

        // Update wave surface
        env.ocean.surface_elevation = waves.get_elevation(
            s.position.x, s.position.y, sim_time);
        env.over_water = true;

        forces.clear();

        // Buoyancy and stability
        buoyancy.compute_forces(s, env, dt, forces);

        // Hydrodynamic maneuvering forces
        hydro.compute_forces(s, env, dt, forces);

        // Gravity
        forces.add_force({0, 0, s.mass * 9.81});

        engine.apply_forces(ship, forces);
        engine.step(dt);
        sim_time += dt;

        // Execute turn at t=30s
        if (t > 30.0 && t < 50.0) {
            hydro.set_rudder_angle(0.35);  // 20° starboard
        } else {
            hydro.set_rudder_angle(0.0);
        }

        // Print every 5 seconds
        if (static_cast<int>(t * 20) % 100 == 0) {
            jaguar::Real speed_kt = std::sqrt(s.velocity.x * s.velocity.x +
                                              s.velocity.y * s.velocity.y) * 1.94384;
            jaguar::Real roll_deg = buoyancy.get_heel() * jaguar::constants::RAD_TO_DEG;
            jaguar::Real pitch_deg = buoyancy.get_trim() * jaguar::constants::RAD_TO_DEG;

            jaguar::Real yaw;
            s.orientation.to_euler(yaw, yaw, yaw);  // Just get yaw
            jaguar::Real hdg_deg = yaw * jaguar::constants::RAD_TO_DEG;
            if (hdg_deg < 0) hdg_deg += 360.0;

            std::cout << std::fixed << std::setprecision(1)
                      << std::setw(7) << t << " | "
                      << std::setw(9) << speed_kt << " | "
                      << std::setw(10) << hdg_deg << " | "
                      << std::setw(7) << roll_deg << " | "
                      << std::setw(8) << pitch_deg << "\n";
        }
    }

    engine.shutdown();
    return 0;
}
```

---

## Satellite Orbit

LEO satellite with orbital mechanics:

```cpp
#include <jaguar/jaguar.h>
#include <jaguar/domain/space.h>
#include <iostream>
#include <cmath>

int main() {
    std::cout << "ISS-like LEO Satellite Simulation\n\n";

    jaguar::interface::Engine engine;
    engine.initialize();

    auto satellite = engine.create_entity("ISS", jaguar::Domain::Space);

    // Configure gravity model (J4 for LEO accuracy)
    jaguar::domain::space::GravityModel gravity;
    gravity.set_fidelity(jaguar::domain::space::GravityFidelity::J4);

    // Configure atmospheric drag
    jaguar::domain::space::AtmosphericDragModel drag;
    drag.set_drag_coefficient(2.2);
    drag.set_area(2500.0);  // ~ISS area

    // Define ISS-like orbit
    jaguar::domain::space::OrbitalElements orbit;
    orbit.semi_major_axis = 6778000.0;   // ~400 km altitude
    orbit.eccentricity = 0.0002;
    orbit.inclination = 51.6 * jaguar::constants::DEG_TO_RAD;
    orbit.raan = 0.0;
    orbit.arg_periapsis = 0.0;
    orbit.mean_anomaly = 0.0;

    // Convert to Cartesian
    jaguar::Vec3 pos, vel;
    orbit.to_cartesian(pos, vel);

    jaguar::physics::EntityState state;
    state.position = pos;
    state.velocity = vel;
    state.mass = 420000;  // ~ISS mass
    engine.set_entity_state(satellite, state);

    std::cout << "Initial orbit: " << (orbit.semi_major_axis - 6378137) / 1000
              << " km altitude\n";
    std::cout << "Orbital period: " << orbit.period() / 60 << " minutes\n\n";

    std::cout << "Time(min) | Alt(km) | Speed(km/s) | Period(min)\n";
    std::cout << "----------|---------|-------------|------------\n";

    const jaguar::Real dt = 10.0;  // 10 second steps
    const jaguar::Real duration = 5400.0;  // ~1 orbit (90 min)
    jaguar::physics::EntityForces forces;

    for (jaguar::Real t = 0; t < duration; t += dt) {
        auto s = engine.get_entity_state(satellite);
        auto env = engine.get_environment(satellite);

        forces.clear();

        // Gravity (main force)
        gravity.compute_forces(s, env, dt, forces);

        // Atmospheric drag (small but significant in LEO)
        jaguar::Real alt = s.position.norm() - 6378137.0;
        if (alt < 600000.0) {  // Below 600 km
            drag.compute_forces(s, env, dt, forces);
        }

        engine.apply_forces(satellite, forces);
        engine.step(dt);

        // Print every 10 minutes
        if (static_cast<int>(t) % 600 == 0) {
            jaguar::Real alt_km = (s.position.norm() - 6378137.0) / 1000.0;
            jaguar::Real speed_kms = s.velocity.norm() / 1000.0;

            // Compute current orbital period
            jaguar::Real r = s.position.norm();
            jaguar::Real v = s.velocity.norm();
            jaguar::Real energy = v*v/2.0 - jaguar::constants::EARTH_MU/r;
            jaguar::Real a = -jaguar::constants::EARTH_MU / (2.0 * energy);
            jaguar::Real period_min = 2.0 * jaguar::constants::PI *
                std::sqrt(a*a*a / jaguar::constants::EARTH_MU) / 60.0;

            std::cout << std::fixed << std::setprecision(2)
                      << std::setw(9) << t / 60.0 << " | "
                      << std::setw(7) << alt_km << " | "
                      << std::setw(11) << speed_kms << " | "
                      << std::setw(10) << period_min << "\n";
        }
    }

    engine.shutdown();
    return 0;
}
```

---

## Multi-Domain Simulation

Simultaneous simulation across all domains:

```cpp
#include <jaguar/jaguar.h>
#include <iostream>
#include <iomanip>

int main() {
    std::cout << "JaguarEngine Multi-Domain Simulation\n";
    std::cout << "=====================================\n\n";

    jaguar::interface::Engine engine;
    engine.initialize();

    // Create entities in each domain
    auto aircraft = engine.create_entity("F-16", jaguar::Domain::Air);
    auto tank = engine.create_entity("M1A2", jaguar::Domain::Land);
    auto ship = engine.create_entity("DDG-51", jaguar::Domain::Sea);
    auto satellite = engine.create_entity("GPS-IIR", jaguar::Domain::Space);

    // Set initial states

    // Aircraft: 10 km altitude, 250 m/s
    {
        jaguar::physics::EntityState s;
        s.position = {0, 0, -10000};
        s.velocity = {250, 0, 0};
        s.mass = 12000;
        engine.set_entity_state(aircraft, s);
    }

    // Tank: on ground, moving at 10 m/s
    {
        jaguar::physics::EntityState s;
        s.position = {1000, 0, 0};
        s.velocity = {10, 0, 0};
        s.mass = 56000;
        engine.set_entity_state(tank, s);
    }

    // Ship: at sea, 15 m/s
    {
        jaguar::physics::EntityState s;
        s.position = {0, 5000, 0};
        s.velocity = {15, 0, 0};
        s.mass = 8600000;
        engine.set_entity_state(ship, s);
    }

    // Satellite: 400 km circular orbit
    {
        jaguar::physics::EntityState s;
        s.position = {6778137, 0, 0};
        s.velocity = {0, 7670, 0};
        s.mass = 2000;
        engine.set_entity_state(satellite, s);
    }

    std::cout << "Simulating all domains simultaneously...\n\n";

    // Run simulation
    const jaguar::Real dt = 0.01;
    for (jaguar::Real t = 0; t < 60.0; t += dt) {
        engine.step(dt);

        // Print every 10 seconds
        if (static_cast<int>(t * 100) % 1000 == 0) {
            std::cout << "=== T = " << std::fixed << std::setprecision(0)
                      << t << " s ===\n";

            auto air = engine.get_entity_state(aircraft);
            std::cout << "Aircraft: Alt=" << -air.position.z
                      << " m, Speed=" << air.velocity.norm() << " m/s\n";

            auto land = engine.get_entity_state(tank);
            std::cout << "Tank: X=" << land.position.x
                      << " m, Speed=" << land.velocity.norm() * 3.6 << " km/h\n";

            auto sea = engine.get_entity_state(ship);
            std::cout << "Ship: Y=" << sea.position.y
                      << " m, Speed=" << sea.velocity.norm() * 1.94384 << " kt\n";

            auto space = engine.get_entity_state(satellite);
            jaguar::Real sat_alt = (space.position.norm() - 6378137) / 1000;
            std::cout << "Satellite: Alt=" << sat_alt
                      << " km, Speed=" << space.velocity.norm() / 1000 << " km/s\n\n";
        }
    }

    engine.shutdown();
    return 0;
}
```

---

## Custom Force Generator

Creating a custom physics model:

```cpp
#include <jaguar/jaguar.h>
#include <jaguar/physics/force.h>
#include <random>

// Custom wind turbulence model
class WindTurbulence : public jaguar::physics::IForceGenerator {
public:
    WindTurbulence(jaguar::Real intensity, jaguar::Real area)
        : intensity_(intensity), area_(area), gen_(std::random_device{}()) {}

    void compute_forces(
        const jaguar::physics::EntityState& state,
        const jaguar::environment::Environment& env,
        jaguar::Real dt,
        jaguar::physics::EntityForces& forces) override
    {
        // Generate random gusts
        std::normal_distribution<jaguar::Real> dist(0.0, intensity_);

        jaguar::Real gust_x = dist(gen_);
        jaguar::Real gust_y = dist(gen_);
        jaguar::Real gust_z = dist(gen_) * 0.5;  // Less vertical turbulence

        // Scale by dynamic pressure
        jaguar::Real speed = state.velocity.norm();
        jaguar::Real qbar = 0.5 * env.atmosphere.density * speed * speed;

        jaguar::Vec3 turbulence{
            gust_x * qbar * area_,
            gust_y * qbar * area_,
            gust_z * qbar * area_
        };

        forces.add_force(turbulence);
    }

private:
    jaguar::Real intensity_;
    jaguar::Real area_;
    std::mt19937 gen_;
};

int main() {
    jaguar::interface::Engine engine;
    engine.initialize();

    auto aircraft = engine.create_entity("Test", jaguar::Domain::Air);

    // Create custom turbulence model
    WindTurbulence turbulence(5.0, 30.0);  // intensity=5, area=30 m²

    jaguar::physics::EntityState state;
    state.position = {0, 0, -5000};
    state.velocity = {200, 0, 0};
    state.mass = 10000;
    engine.set_entity_state(aircraft, state);

    jaguar::physics::EntityForces forces;
    const jaguar::Real dt = 0.01;

    for (jaguar::Real t = 0; t < 30.0; t += dt) {
        auto s = engine.get_entity_state(aircraft);
        auto env = engine.get_environment(aircraft);

        forces.clear();

        // Apply custom turbulence
        turbulence.compute_forces(s, env, dt, forces);

        // Other forces (gravity, etc.)
        forces.add_force({0, 0, s.mass * jaguar::constants::G0});

        engine.apply_forces(aircraft, forces);
        engine.step(dt);
    }

    engine.shutdown();
    return 0;
}
```

---

## Building Examples

All examples can be built with CMake:

```cmake
cmake_minimum_required(VERSION 3.25)
project(JaguarExamples)

find_package(Jaguar REQUIRED)

add_executable(minimal minimal.cpp)
target_link_libraries(minimal PRIVATE Jaguar::jaguar)

add_executable(simple_flight simple_flight.cpp)
target_link_libraries(simple_flight PRIVATE Jaguar::jaguar)

# Add more examples...
```

Build and run:

```bash
mkdir build && cd build
cmake ..
make
./simple_flight
```

---

## Next Steps

- [Air Domain Deep Dive](../domains/air.md)
- [Land Domain Deep Dive](../domains/land.md)
- [Sea Domain Deep Dive](../domains/sea.md)
- [Space Domain Deep Dive](../domains/space.md)
- [Configuration Reference](../api/configuration.md)
- [Python API Guide](../api/python.md)
