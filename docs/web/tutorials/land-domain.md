# Land Domain Tutorial

This tutorial demonstrates how to simulate ground vehicles using JaguarEngine's Land domain.

## Prerequisites

- JaguarEngine installed and configured
- Basic understanding of vehicle dynamics
- C++ development environment

## Tutorial: Tank Simulation

### Step 1: Create the Engine

```cpp
#include <jaguar/jaguar.h>

using namespace jaguar;

int main() {
    Engine engine;
    if (!engine.initialize()) {
        std::cerr << "Failed to initialize engine\n";
        return 1;
    }
```

### Step 2: Create a Ground Vehicle Entity

```cpp
    // Create tank entity
    EntityId tank = engine.create_entity("M1A2", Domain::Land);
```

### Step 3: Set Initial State

```cpp
    // Configure initial state
    physics::EntityState state;

    // Position: On ground at origin
    state.position = Vec3{0.0, 0.0, 0.0};

    // Velocity: Stationary
    state.velocity = Vec3{0.0, 0.0, 0.0};

    // Orientation: Facing North
    state.orientation = Quaternion::identity();

    // Mass: 62 tonnes combat weight
    state.mass = 62000.0;

    // Inertia tensor
    state.inertia = Mat3x3::identity();
    state.inertia.data[0][0] = 50000;   // Ixx
    state.inertia.data[1][1] = 200000;  // Iyy
    state.inertia.data[2][2] = 220000;  // Izz

    engine.set_entity_state(tank, state);
```

### Step 4: Create Physics Models

```cpp
    // Create terramechanics model
    domain::land::TerramechanicsModel terra;
    terra.set_contact_area(0.63, 4.6);  // Track width, length (m)
    terra.set_vehicle_weight(62000.0 * constants::G0);

    // Create suspension model
    domain::land::SuspensionModel suspension;

    // Configure suspension units (6 road wheels per side)
    domain::land::SuspensionUnit wheel;
    wheel.spring_k = 300000.0;   // 300 kN/m
    wheel.damper_c = 30000.0;    // 30 kN·s/m
    wheel.travel_max = 0.40;     // 40 cm travel

    // Left side
    for (Real x = -3.0; x <= 2.0; x += 1.0) {
        suspension.add_unit(Vec3{x, 1.5, -0.8}, wheel);
    }
    // Right side
    for (Real x = -3.0; x <= 2.0; x += 1.0) {
        suspension.add_unit(Vec3{x, -1.5, -0.8}, wheel);
    }

    // Create tracked vehicle model
    domain::land::TrackedVehicleModel tracks;
    tracks.set_sprocket(0.33, 100000.0);  // Radius, max torque
```

### Step 5: Run the Simulation

```cpp
    // Simulation parameters
    Real dt = 0.02;       // 50 Hz
    Real duration = 30.0; // 30 seconds
    Real throttle = 0.5;  // 50% throttle

    std::cout << "Time(s), Position(m), Speed(m/s), Sinkage(m)\n";

    for (Real t = 0; t < duration; t += dt) {
        auto state = engine.get_entity_state(tank);
        auto env = engine.get_environment(tank);

        // Get soil properties at current location
        domain::land::SoilProperties soil =
            domain::land::SoilProperties::DrySand();

        // Compute forces
        physics::EntityForces forces;
        forces.clear();

        // Terramechanics
        terra.compute_forces(state, env, dt, forces);

        // Track dynamics
        Real engine_torque = throttle * 100000.0;  // Max 100 kN·m
        tracks.update(engine_torque, state.mass * constants::G0, soil, dt);

        // Propulsive force from tracks
        Real propulsion = tracks.get_propulsive_force();
        forces.add_force(Vec3{propulsion, 0.0, 0.0});

        // Suspension
        suspension.update(state, dt);
        forces.add_force(suspension.get_total_force());
        forces.add_torque(suspension.get_total_torque());

        // Gravity
        forces.add_force(Vec3{0.0, 0.0, state.mass * constants::G0});

        // Apply forces and step
        engine.apply_forces(tank, forces);
        engine.step(dt);

        // Output telemetry every second
        if (std::fmod(t, 1.0) < dt) {
            Real distance = state.position.x;
            Real speed = state.velocity.norm();
            Real sinkage = terra.get_sinkage();

            std::cout << t << ", "
                      << distance << ", "
                      << speed << ", "
                      << sinkage << "\n";
        }
    }

    engine.shutdown();
    return 0;
}
```

## Tutorial: Wheeled Vehicle

### Creating a Wheeled Vehicle

```cpp
// Create wheeled vehicle
EntityId apc = engine.create_entity("BTR80", Domain::Land);

physics::EntityState state;
state.mass = 13600.0;  // 13.6 tonnes
engine.set_entity_state(apc, state);

// Wheeled suspension (8 wheels)
domain::land::SuspensionModel suspension;
domain::land::SuspensionUnit wheel;
wheel.spring_k = 150000.0;  // Softer than tank
wheel.damper_c = 15000.0;
wheel.travel_max = 0.30;

// 4 axles, 2 wheels each
Real axle_positions[] = {2.5, 1.0, -1.0, -2.5};
for (Real x : axle_positions) {
    suspension.add_unit(Vec3{x, 1.2, -0.6}, wheel);   // Left
    suspension.add_unit(Vec3{x, -1.2, -0.6}, wheel);  // Right
}
```

### Tire-Soil Interaction

```cpp
// For wheeled vehicles, modify contact area
domain::land::TerramechanicsModel terra;
Real tire_width = 0.35;   // 35 cm tire width
Real contact_length = 0.4; // Approximate contact patch
terra.set_contact_area(tire_width, contact_length);
terra.set_vehicle_weight(13600.0 * constants::G0 / 8);  // Per tire
```

## Tutorial: Terrain Interaction

### Different Soil Types

```cpp
// Query terrain material at position
auto env = engine.get_environment(tank);
TerrainMaterial material = env.terrain.material;

// Get soil properties
domain::land::SoilProperties soil;

switch (material.type) {
    case TerrainMaterial::Sand:
        soil = domain::land::SoilProperties::DrySand();
        break;
    case TerrainMaterial::Clay:
        soil = domain::land::SoilProperties::Clay();
        break;
    case TerrainMaterial::Snow:
        soil = domain::land::SoilProperties::Snow();
        break;
    case TerrainMaterial::Road:
        soil = domain::land::SoilProperties::Asphalt();
        break;
    default:
        soil = domain::land::SoilProperties::DrySand();
}
```

### Slope Effects

```cpp
// Get terrain slope at vehicle position
Real slope_angle = env.terrain.slope_angle;
Vec3 surface_normal = env.terrain.normal;

// Decompose gravity into normal and parallel components
Vec3 gravity{0.0, 0.0, state.mass * constants::G0};
Vec3 gravity_normal = surface_normal * gravity.dot(surface_normal);
Vec3 gravity_parallel = gravity - gravity_normal;

// Gravity parallel component affects vehicle motion on slopes
forces.add_force(gravity_parallel);
```

## XML Configuration

### Ground Vehicle Definition

```xml
<?xml version="1.0" encoding="UTF-8"?>
<entity type="ground_vehicle" name="M1A2">
    <metrics>
        <length unit="m">9.77</length>
        <width unit="m">3.66</width>
        <height unit="m">2.44</height>
    </metrics>

    <mass_balance>
        <combat_weight unit="kg">62000</combat_weight>
        <fuel_capacity unit="L">1900</fuel_capacity>
    </mass_balance>

    <tracks>
        <track_width unit="m">0.63</track_width>
        <track_length unit="m">4.6</track_length>
        <sprocket_radius unit="m">0.33</sprocket_radius>
    </tracks>

    <suspension type="torsion_bar">
        <road_wheels_per_side>7</road_wheels_per_side>
        <spring_rate unit="N/m">300000</spring_rate>
        <damping_rate unit="N*s/m">30000</damping_rate>
    </suspension>
</entity>
```

## Mobility Analysis

### Go/No-Go Terrain Assessment

```cpp
bool can_traverse(const domain::land::SoilProperties& soil,
                  Real vehicle_weight,
                  Real contact_area,
                  Real slope_deg) {
    // Check sinkage
    Real pressure = vehicle_weight / contact_area;
    Real k = soil.k_c / 0.5 + soil.k_phi;  // Assume 0.5m contact width
    Real sinkage = std::pow(pressure / k, 1.0 / soil.n);

    if (sinkage > 0.3) {  // Max 30cm sinkage
        return false;  // Vehicle may be stuck
    }

    // Check slope
    Real max_slope = std::atan(soil.phi) * constants::RAD_TO_DEG;
    if (slope_deg > max_slope * 0.8) {  // 80% of friction limit
        return false;  // Slope too steep
    }

    return true;
}
```

## Common Issues

### Vehicle Sinking

**Symptoms:** Vehicle sinks excessively or gets stuck

**Solutions:**
- Check contact area dimensions
- Verify soil properties are realistic
- Consider adding track width/length

### Oscillations

**Symptoms:** Vehicle bounces or oscillates

**Solutions:**
- Increase damping coefficient
- Reduce time step
- Check suspension preload

## Next Steps

- [Sea Domain Tutorial](sea-domain.md) - Naval simulation
- [Air Domain Tutorial](air-domain.md) - Aircraft simulation
- [Terrain Tutorial](terrain.md) - Working with terrain data
- [API Reference](../api/land.md) - Land domain API
