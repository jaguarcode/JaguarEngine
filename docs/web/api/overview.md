# API Reference Overview

Complete API documentation for JaguarEngine.

## Module Structure

JaguarEngine is organized into logical modules:

```
jaguar/
├── core/           # Core types, math, utilities
├── physics/        # Entity management, forces, integration
├── domain/         # Domain-specific physics (air, land, sea, space)
├── environment/    # Terrain, atmosphere, ocean
├── interface/      # Public API facade, configuration
├── cloud/          # Distributed simulation, state sync, partitioning
├── thread/         # Digital thread, history, degradation models
├── ml/             # Machine learning, neural autopilot, RL
├── federation/     # DIS/HLA network protocols
├── gpu/            # GPU compute backends (CUDA, OpenCL, Metal)
├── sensors/        # Sensor models with MIL-SPEC noise profiles
└── xr/             # XR integration, spatial audio, haptics
```

---

## Quick Reference

### Engine Lifecycle

```cpp
#include <jaguar/jaguar.h>

jaguar::interface::Engine engine;

// Initialize
bool success = engine.initialize();
bool success = engine.initialize(config);

// Run simulation
engine.step(dt);                    // Single step
engine.run_for(duration_seconds);   // Run duration
engine.run();                       // Run continuously

// Cleanup
engine.shutdown();
```

### Entity Management

```cpp
// Create
EntityId id = engine.create_entity("name", Domain::Air);
EntityId id = engine.create_entity_from_config(config);

// State access
engine.set_entity_state(id, state);
EntityState state = engine.get_entity_state(id);

// Forces
engine.apply_forces(id, forces);

// Environment
Environment env = engine.get_environment(id);

// Destroy
engine.destroy_entity(id);
```

### Core Types

```cpp
// Numeric types
using Real = double;
using EntityId = uint64_t;

// Math types
struct Vec3 { Real x, y, z; };
struct Vec4 { Real x, y, z, w; };
struct Quaternion { Real w, x, y, z; };
struct Mat3x3 { Real data[3][3]; };

// Enumerations
enum class Domain { Generic, Air, Land, Sea, Space };
enum class CoordinateFrame { ECEF, NED, ENU, ECI };
```

---

## Module Reference

### [Core Module](core.md)

Fundamental types, math operations, and utilities.

| Component | Header | Description |
|-----------|--------|-------------|
| Types | `jaguar/core/types.h` | Basic numeric types, EntityId |
| Math | `jaguar/core/math/` | Vector, quaternion, matrix |
| Coordinates | `jaguar/core/coordinates.h` | ECEF, NED, ECI transforms |
| Memory | `jaguar/core/memory.h` | Pool allocation |
| SIMD | `jaguar/core/simd.h` | AVX2/FMA wrappers |
| Property | `jaguar/core/property.h` | Property system |
| Time | `jaguar/core/time.h` | Time management |

### [Physics Module](physics.md)

Entity management and physics integration.

| Component | Header | Description |
|-----------|--------|-------------|
| Entity | `jaguar/physics/entity.h` | EntityState, EntityManager |
| Force | `jaguar/physics/force.h` | Force generators |
| Solver | `jaguar/physics/solver.h` | Numerical integrators |
| Collision | `jaguar/physics/collision.h` | Collision detection |

### [Air Domain](air.md)

Aircraft physics models.

| Component | Header | Description |
|-----------|--------|-------------|
| Aerodynamics | `jaguar/domain/air.h` | Lift, drag, moments |
| Propulsion | `jaguar/domain/air.h` | Engine models |
| Flight Control | `jaguar/domain/air.h` | FCS, autopilot |

### [Land Domain](land.md)

Ground vehicle physics models.

| Component | Header | Description |
|-----------|--------|-------------|
| Terramechanics | `jaguar/domain/land.h` | Soil interaction |
| Suspension | `jaguar/domain/land.h` | Spring-damper |
| Tracked | `jaguar/domain/land.h` | Track dynamics |

### [Sea Domain](sea.md)

Ship and submarine physics models.

| Component | Header | Description |
|-----------|--------|-------------|
| Buoyancy | `jaguar/domain/sea.h` | Hydrostatics |
| Hydrodynamics | `jaguar/domain/sea.h` | Maneuvering |
| Waves | `jaguar/domain/sea.h` | Wave spectra, RAO |

### [Space Domain](space.md)

Orbital mechanics and spacecraft physics.

| Component | Header | Description |
|-----------|--------|-------------|
| Orbital | `jaguar/domain/space.h` | Elements, TLE |
| SGP4 | `jaguar/domain/space.h` | Orbit propagation |
| Gravity | `jaguar/domain/space.h` | Gravity models |

### [Environment Module](environment.md)

Environmental models.

| Component | Header | Description |
|-----------|--------|-------------|
| Atmosphere | `jaguar/environment/atmosphere.h` | US Std 1976 |
| Terrain | `jaguar/environment/terrain.h` | DEM, materials |
| Ocean | `jaguar/environment/ocean.h` | Waves, currents |

### [Configuration](configuration.md)

Configuration and setup.

| Component | Header | Description |
|-----------|--------|-------------|
| Config | `jaguar/interface/config.h` | Engine config |
| XML | `jaguar/interface/config.h` | Entity configs |

### [Cloud Burst Module](cloud.md)

Distributed simulation and cloud scaling.

| Component | Header | Description |
|-----------|--------|-------------|
| State Sync | `jaguar/cloud/state_sync.h` | Distributed state synchronization |
| Partition Manager | `jaguar/cloud/partition_manager.h` | Spatial/domain partitioning |
| Distributed Time | `jaguar/cloud/distributed_time.h` | Raft consensus, vector clocks |

### [Digital Thread Module](thread.md)

Lifecycle management and predictive maintenance.

| Component | Header | Description |
|-----------|--------|-------------|
| History Store | `jaguar/thread/history_store.h` | State history, snapshots, export |
| Degradation Model | `jaguar/thread/degradation_model.h` | Failure prediction, maintenance |

### [Machine Learning Module](ml.md)

Neural networks and reinforcement learning.

| Component | Header | Description |
|-----------|--------|-------------|
| Neural Autopilot | `jaguar/ml/neural_autopilot.h` | NN-based flight control |
| Model Repository | `jaguar/ml/model_repository.h` | Model versioning, caching |
| RL Environment | `jaguar/ml/rl_environment.h` | Gym-compatible RL interface |

### [Federation Module](federation.md)

Distributed Interactive Simulation and HLA.

| Component | Header | Description |
|-----------|--------|-------------|
| DIS Protocol | `jaguar/federation/dis_protocol.h` | IEEE 1278.1-2012 PDUs |
| HLA RTI | `jaguar/federation/hla_rti.h` | IEEE 1516-2010 federation |

### [GPU Compute Module](gpu.md)

Hardware-accelerated computation.

| Component | Header | Description |
|-----------|--------|-------------|
| Compute Backend | `jaguar/gpu/compute_backend.h` | CUDA, OpenCL, Metal abstraction |

### [Sensors Module](sensors.md)

Sensor simulation with realistic noise models.

| Component | Header | Description |
|-----------|--------|-------------|
| Sensor Base | `jaguar/sensors/sensor.h` | Sensor interface, noise models |
| IMU Sensor | `jaguar/sensors/imu_sensor.h` | MIL-SPEC IMU simulation |

### [XR Module](xr.md)

Extended reality integration.

| Component | Header | Description |
|-----------|--------|-------------|
| OpenXR | `jaguar/xr/openxr_integration.h` | VR/AR device support |
| Spatial Audio | `jaguar/xr/spatial_audio.h` | 3D audio rendering |
| Haptics | `jaguar/xr/haptic_feedback.h` | Tactile feedback |
| Training | `jaguar/xr/training_scenario.h` | Scenario management |

---

## Constants

```cpp
namespace jaguar::constants {
    constexpr Real PI = 3.14159265358979323846;
    constexpr Real TWO_PI = 6.28318530717958647692;
    constexpr Real DEG_TO_RAD = PI / 180.0;
    constexpr Real RAD_TO_DEG = 180.0 / PI;

    constexpr Real G0 = 9.80665;              // Standard gravity (m/s²)
    constexpr Real EARTH_RADIUS = 6378137.0;  // WGS84 equatorial (m)
    constexpr Real EARTH_MU = 3.986004418e14; // Earth GM (m³/s²)
}
```

---

## Common Patterns

### Creating an Aircraft

```cpp
#include <jaguar/jaguar.h>
#include <jaguar/domain/air.h>

// Create entity
auto aircraft = engine.create_entity("F-16", Domain::Air);

// Configure aerodynamics
domain::air::AerodynamicsModel aero;
aero.set_reference_area(27.87);  // m²
aero.set_reference_chord(3.45);  // m
aero.set_reference_span(9.45);   // m

// Configure propulsion
domain::air::PropulsionModel prop;
prop.set_max_thrust(131000.0);   // N
prop.set_fuel_capacity(3200.0);  // kg
prop.start();

// Set initial state
physics::EntityState state;
state.position = {0, 0, -10000};  // 10 km altitude
state.velocity = {250, 0, 0};     // 250 m/s
state.mass = 12000;
engine.set_entity_state(aircraft, state);

// Simulation loop
while (running) {
    physics::EntityForces forces;

    auto s = engine.get_entity_state(aircraft);
    auto env = engine.get_environment(aircraft);

    aero.compute_forces(s, env, dt, forces);
    prop.compute_forces(s, env, dt, forces);
    forces.add_force({0, 0, s.mass * constants::G0});

    engine.apply_forces(aircraft, forces);
    engine.step(dt);
}
```

### Creating a Ground Vehicle

```cpp
#include <jaguar/jaguar.h>
#include <jaguar/domain/land.h>

auto tank = engine.create_entity("M1A2", Domain::Land);

domain::land::TerramechanicsModel terra;
terra.set_contact_area(0.63, 4.6);    // Track dimensions
terra.set_vehicle_weight(549000.0);   // N

domain::land::SuspensionModel suspension;
domain::land::SuspensionUnit wheel;
wheel.spring_k = 300000.0;
wheel.damper_c = 30000.0;
suspension.add_unit({-3.5, 1.8, -0.9}, wheel);
// Add more wheels...

physics::EntityState state;
state.position = {0, 0, 0};
state.mass = 56000;
engine.set_entity_state(tank, state);
```

### Creating a Ship

```cpp
#include <jaguar/jaguar.h>
#include <jaguar/domain/sea.h>

auto ship = engine.create_entity("DDG-51", Domain::Sea);

domain::sea::BuoyancyModel buoyancy;
buoyancy.set_displaced_volume(8400.0);
buoyancy.set_metacentric_height(2.5);

domain::sea::HydrodynamicsModel hydro;
hydro.set_hull_coefficients(-0.04, -0.01, -0.4, 0.05, -0.1, -0.05);

domain::sea::WaveModel waves;
waves.set_sea_state(domain::sea::SeaState::FromNATOSeaState(4));

physics::EntityState state;
state.position = {0, 0, 0};
state.mass = 8600000;  // 8600 tonnes
engine.set_entity_state(ship, state);
```

### Creating a Satellite

```cpp
#include <jaguar/jaguar.h>
#include <jaguar/domain/space.h>

auto sat = engine.create_entity("GPS-IIR", Domain::Space);

domain::space::GravityModel gravity;
gravity.set_fidelity(domain::space::GravityFidelity::J4);

domain::space::OrbitalElements orbit;
orbit.semi_major_axis = 26560000;  // GPS orbit
orbit.eccentricity = 0.01;
orbit.inclination = 55.0 * constants::DEG_TO_RAD;

Vec3 pos, vel;
orbit.to_cartesian(pos, vel);

physics::EntityState state;
state.position = pos;
state.velocity = vel;
state.mass = 2000;
engine.set_entity_state(sat, state);
```

---

## Error Handling

```cpp
// Initialization
if (!engine.initialize()) {
    // Handle initialization failure
    std::cerr << "Engine init failed\n";
}

// Entity creation
EntityId id = engine.create_entity("name", Domain::Air);
if (id == INVALID_ENTITY_ID) {
    // Handle creation failure
}

// State access
if (engine.entity_exists(id)) {
    auto state = engine.get_entity_state(id);
}
```

---

## Thread Safety

- Engine methods are **not thread-safe** by default
- Use single-threaded access or external synchronization
- Internal physics computation may use multiple threads
- Entity state access should be done from the main thread

---

## Performance Tips

1. **Batch operations**: Update multiple entities before stepping
2. **Pre-fetch states**: Get all states before computing forces
3. **Reuse force objects**: Clear and reuse `EntityForces`
4. **Property binding**: Bind frequently-accessed properties
5. **Appropriate time step**: Don't over-resolve dynamics

```cpp
// Efficient batch update pattern
std::vector<EntityId> entities = engine.get_all_entities();
std::vector<EntityState> states;
std::vector<Environment> envs;

// Pre-fetch all data
for (auto id : entities) {
    states.push_back(engine.get_entity_state(id));
    envs.push_back(engine.get_environment(id));
}

// Compute forces (can be parallelized)
std::vector<EntityForces> forces(entities.size());
for (size_t i = 0; i < entities.size(); ++i) {
    compute_entity_forces(states[i], envs[i], dt, forces[i]);
}

// Apply all forces
for (size_t i = 0; i < entities.size(); ++i) {
    engine.apply_forces(entities[i], forces[i]);
}

// Single physics step
engine.step(dt);
```

---

## See Also

- [Configuration Reference](configuration.md)
- [Python API](python.md)
- [Lua API](lua.md)
- [Examples](../tutorials/examples.md)
