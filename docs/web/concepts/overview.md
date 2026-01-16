# Core Concepts

Understanding JaguarEngine's architecture and design principles.

## Design Philosophy

JaguarEngine is built on three core principles:

1. **Data-Oriented Design (DOD)** - Optimize for cache efficiency and SIMD operations
2. **Multi-Domain Unification** - Consistent interfaces across Air, Land, Sea, and Space
3. **Component-Based Physics** - Modular force generators that can be mixed and matched

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    Public API (Engine Facade)               │
└────────────────────────────┬────────────────────────────────┘
                             │
┌────────────────────────────┴────────────────────────────────┐
│                 Physics Engine Executive                    │
│  ┌──────────────┐ ┌──────────────┐ ┌──────────────────────┐ │
│  │Entity Manager│ │Physics System│ │ Property Manager     │ │
│  └──────────────┘ └──────────────┘ └──────────────────────┘ │
└────────────────────────────┬────────────────────────────────┘
                             │
┌────────────────────────────┴────────────────────────────────┐
│                    Domain Physics Layer                     │
│    ┌─────┐   ┌──────┐   ┌─────┐   ┌───────┐               │
│    │ Air │   │ Land │   │ Sea │   │ Space │               │
│    └─────┘   └──────┘   └─────┘   └───────┘               │
└────────────────────────────┬────────────────────────────────┘
                             │
┌────────────────────────────┴────────────────────────────────┐
│                   Environment Services                      │
│    ┌─────────┐   ┌────────────┐   ┌───────┐               │
│    │ Terrain │   │ Atmosphere │   │ Ocean │               │
│    └─────────┘   └────────────┘   └───────┘               │
└────────────────────────────┬────────────────────────────────┘
                             │
┌────────────────────────────┴────────────────────────────────┐
│                      Core Services                          │
│  ┌────────┐ ┌───────────┐ ┌──────┐ ┌────────┐ ┌──────────┐ │
│  │ Memory │ │ Threading │ │ Math │ │  I/O   │ │ Config   │ │
│  └────────┘ └───────────┘ └──────┘ └────────┘ └──────────┘ │
└─────────────────────────────────────────────────────────────┘
```

---

## Entities and State

### What is an Entity?

An entity represents a simulated object (aircraft, vehicle, ship, spacecraft). Entities are lightweight identifiers; the actual data is stored in contiguous arrays for cache efficiency.

```cpp
// Entity creation
EntityId aircraft = engine.create_entity("F-16", Domain::Air);
EntityId tank = engine.create_entity("M1A2", Domain::Land);
EntityId ship = engine.create_entity("DDG-51", Domain::Sea);
EntityId satellite = engine.create_entity("GPS-IIR", Domain::Space);
```

### Entity State

Each entity has a 6-DOF (6 Degrees of Freedom) state:

```cpp
struct EntityState {
    Vec3 position;           // Position (m) in ECEF or NED
    Vec3 velocity;           // Velocity (m/s)
    Quaternion orientation;  // Attitude quaternion
    Vec3 angular_velocity;   // Angular velocity (rad/s)
    Real mass;               // Mass (kg)
    Mat3x3 inertia;          // Inertia tensor (kg·m²)
};
```

### State Access

```cpp
// Set state
engine.set_entity_state(entity_id, state);

// Get state (read current values)
auto state = engine.get_entity_state(entity_id);

// Access individual components
Real altitude = -state.position.z;  // NED: negative z is altitude
Real speed = state.velocity.norm();
```

---

## Coordinate Systems

JaguarEngine supports multiple coordinate frames:

### NED (North-East-Down)

The default local frame:

| Axis | Direction | Notes |
|------|-----------|-------|
| X | North | Positive forward |
| Y | East | Positive right |
| Z | Down | Positive into Earth |

```cpp
// 10 km altitude in NED
Vec3 position{0.0, 0.0, -10000.0};

// Flying north at 250 m/s
Vec3 velocity{250.0, 0.0, 0.0};
```

### ECEF (Earth-Centered Earth-Fixed)

Global coordinate frame for large-scale simulations:

- Origin at Earth's center
- X-axis through Prime Meridian/Equator intersection
- Z-axis through North Pole
- Y-axis completes right-handed system

### ECI (Earth-Centered Inertial)

For space domain simulations:

- Non-rotating frame
- X-axis toward vernal equinox
- Z-axis toward celestial north pole

### Coordinate Conversions

```cpp
// ECEF to geodetic (lat, lon, alt)
Vec3 geodetic = transforms::ecef_to_geodetic(position_ecef);
Real lat = geodetic.x;  // radians
Real lon = geodetic.y;  // radians
Real alt = geodetic.z;  // meters

// Geodetic to ECEF
Vec3 ecef = transforms::geodetic_to_ecef(lat, lon, alt);

// ECI to ECEF (time-dependent)
Vec3 ecef = transforms::eci_to_ecef(position_eci, julian_date);
```

---

## Physics Domains

### Domain Selection

Choose the appropriate domain for your entity:

| Domain | Use Case | Physics Models |
|--------|----------|----------------|
| `Air` | Aircraft, helicopters, UAVs | Aerodynamics, propulsion |
| `Land` | Tanks, cars, trucks | Terramechanics, suspension |
| `Sea` | Ships, submarines | Hydrodynamics, buoyancy |
| `Space` | Satellites, spacecraft | Orbital mechanics, gravity |

### Domain-Specific Physics

Each domain provides specialized force generators:

```cpp
// Air domain
AerodynamicsModel aero;      // Lift, drag, moments
PropulsionModel engine;      // Thrust, fuel consumption
FlightControlSystem fcs;     // Control surface deflections

// Land domain
TerramechanicsModel terra;   // Soil-vehicle interaction
SuspensionModel suspension;  // Spring-damper dynamics
TrackedVehicleModel tracks;  // Track slip and traction

// Sea domain
BuoyancyModel buoyancy;      // Hydrostatic forces
HydrodynamicsModel hydro;    // Maneuvering forces
WaveModel waves;             // Sea state effects

// Space domain
GravityModel gravity;        // Gravitational acceleration
SGP4Propagator sgp4;         // Orbital propagation
AtmosphericDragModel drag;   // Upper atmosphere drag
```

---

## Force Generators

### Concept

Force generators compute forces and torques acting on entities. Multiple generators can be combined:

```cpp
// Force generator interface
class IForceGenerator {
public:
    virtual void compute_forces(
        const EntityState& state,
        const Environment& env,
        Real dt,
        EntityForces& forces
    ) = 0;
};
```

### Combining Forces

```cpp
// Create force accumulators
EntityForces forces;

// Compute from multiple sources
aero_model.compute_forces(state, env, dt, forces);
propulsion_model.compute_forces(state, env, dt, forces);
gravity_force = Vec3{0, 0, mass * G0};
forces.add_force(gravity_force);

// Apply to entity
engine.apply_forces(entity_id, forces);
```

### Built-in Force Generators

| Generator | Domain | Purpose |
|-----------|--------|---------|
| `AerodynamicsModel` | Air | Lift, drag, aerodynamic moments |
| `PropulsionModel` | Air | Thrust, fuel consumption |
| `TerramechanicsModel` | Land | Soil resistance, sinkage |
| `SuspensionModel` | Land | Suspension forces |
| `BuoyancyModel` | Sea | Hydrostatic lift |
| `HydrodynamicsModel` | Sea | Hull resistance, maneuvering |
| `GravityModel` | Space | Gravitational acceleration |
| `AtmosphericDragModel` | Space | Atmospheric drag |

---

## Environment System

### Environment Query

Get environmental conditions at any location:

```cpp
// Query at entity location
Environment env = engine.get_environment(entity_id);

// Query at arbitrary position
Environment env = engine.get_environment_at(position_ecef);

// Access components
Real density = env.atmosphere.density;      // kg/m³
Real temperature = env.atmosphere.temperature;  // K
Real wind_speed = env.atmosphere.wind.norm();   // m/s
Real terrain_elevation = env.terrain_elevation; // m
```

### Atmosphere

US Standard Atmosphere 1976 with weather extensions:

```cpp
struct AtmosphereState {
    Real temperature;      // K
    Real pressure;         // Pa
    Real density;          // kg/m³
    Real speed_of_sound;   // m/s
    Vec3 wind;             // m/s (NED)
    Real humidity;         // 0-1
    Real visibility;       // m
};
```

### Terrain

Digital elevation model with material properties:

```cpp
struct TerrainQuery {
    Real elevation;        // m above WGS84
    Vec3 normal;           // Surface normal
    Real slope_angle;      // rad
    TerrainMaterial material;
    bool valid;
};
```

### Ocean

Wave modeling and sea state:

```cpp
struct OceanState {
    Real water_depth;      // m
    Real surface_elevation; // m
    Vec3 current;          // m/s
    Real temperature;      // °C
    Real salinity;         // ppt
    Real density;          // kg/m³
};
```

---

## Simulation Loop

### Basic Loop Structure

```cpp
// Initialize
Engine engine;
engine.initialize();

// Create entities
EntityId entity = engine.create_entity("name", Domain::Air);
engine.set_entity_state(entity, initial_state);

// Simulation loop
const Real dt = 0.01;  // 100 Hz
while (running) {
    // 1. Update controls/inputs

    // 2. Compute custom forces
    EntityForces forces;
    my_model.compute_forces(state, env, dt, forces);
    engine.apply_forces(entity, forces);

    // 3. Advance physics
    engine.step(dt);

    // 4. Read results
    auto state = engine.get_entity_state(entity);
}

// Cleanup
engine.shutdown();
```

### Time Step Selection

| Application | Recommended dt | Notes |
|-------------|----------------|-------|
| Aircraft | 0.01 s (100 Hz) | Adequate for most dynamics |
| Ground vehicles | 0.02 s (50 Hz) | Lower frequency sufficient |
| Ships | 0.05 s (20 Hz) | Slower dynamics |
| Spacecraft | 1-10 s | Long-duration orbits |

### Variable Time Step

```cpp
// Adaptive stepping
auto last_time = std::chrono::high_resolution_clock::now();

while (running) {
    auto now = std::chrono::high_resolution_clock::now();
    Real dt = std::chrono::duration<Real>(now - last_time).count();
    last_time = now;

    // Clamp dt to prevent instability
    dt = std::min(dt, 0.1);

    engine.step(dt);
}
```

---

## Integration Methods

JaguarEngine provides multiple numerical integrators:

| Method | Order | Use Case |
|--------|-------|----------|
| RK4 | 4th | General purpose, good stability |
| ABM4 | 4th | Smooth trajectories, efficient |
| Adaptive | Variable | Variable step size |

### Selecting Integrator

```cpp
// In configuration
<physics>
    <integrator>rk4</integrator>
</physics>

// Or programmatically
engine.set_integrator("rk4");
```

---

## Property System

### Hierarchical Access

Properties provide runtime access to simulation parameters:

```
aircraft/f16/
├── position/
│   ├── latitude-geod-rad
│   ├── longitude-geoc-rad
│   └── h-sl-ft
├── velocity/
│   ├── u-fps
│   ├── v-fps
│   └── w-fps
├── aero/
│   ├── alpha-rad
│   ├── beta-rad
│   ├── cl
│   └── cd
└── propulsion/
    ├── thrust-lbs
    └── fuel-lbs
```

### Using Properties

```cpp
// Get property value
Real alpha = engine.get_property("aircraft/f16/aero/alpha-rad");

// Set property value
engine.set_property("aircraft/f16/propulsion/throttle", 0.8);

// Bind for direct access (faster)
Real* thrust_ptr;
engine.bind_property("aircraft/f16/propulsion/thrust-lbs", thrust_ptr);
// Now *thrust_ptr always has current value
```

---

## Next Steps

- [Entity Management](entities.md) - Detailed entity lifecycle
- [Force Generators](force-generators.md) - Creating custom physics
- [Environment Models](environment.md) - Terrain, atmosphere, ocean
- [Coordinate Systems](coordinates.md) - Detailed coordinate transforms
- [Integration Methods](integration.md) - Numerical integration options
