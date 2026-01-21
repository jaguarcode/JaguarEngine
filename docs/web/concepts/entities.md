# Entities

Entities are the fundamental simulation objects in JaguarEngine. This guide covers entity creation, management, and state handling.

## What is an Entity?

An entity represents any simulated object in JaguarEngine:

- **Aircraft**: Fixed-wing, rotary-wing, missiles
- **Ground Vehicles**: Tanks, APCs, wheeled vehicles
- **Naval Vessels**: Surface ships, submarines
- **Spacecraft**: Satellites, space stations, launch vehicles
- **Generic**: Custom simulation objects

## Entity Architecture

Entities use a lightweight ID-based design:

```cpp
struct Entity {
    EntityId id;              // Unique identifier
    ComponentMask components; // Bitfield of attached components
    Domain primary_domain;    // Primary physics domain
    PropertyNode* properties; // Property tree root
};
```

**Key Design Principles:**

1. Entities are just identifiers (lightweight)
2. Data lives in contiguous arrays (cache-friendly)
3. Components define behavior and capabilities
4. Properties enable runtime configuration

## Creating Entities

### Basic Creation

```cpp
#include <jaguar/jaguar.h>

using namespace jaguar;

Engine engine;
engine.initialize();

// Create entities by domain
EntityId aircraft = engine.create_entity("F16", Domain::Air);
EntityId tank = engine.create_entity("M1A2", Domain::Land);
EntityId ship = engine.create_entity("DDG51", Domain::Sea);
EntityId satellite = engine.create_entity("GPS", Domain::Space);
```

### From Configuration

```cpp
// Load entity from XML configuration
config::EntityConfig cfg = config::EntityConfig::load("aircraft/f16.xml");
EntityId aircraft = engine.create_entity_from_config(cfg);
```

## Entity State

### EntityState Structure

```cpp
struct EntityState {
    Vec3 position{0, 0, 0};           // Position (m)
    Vec3 velocity{0, 0, 0};           // Velocity (m/s)
    Quaternion orientation;            // Attitude quaternion
    Vec3 angular_velocity{0, 0, 0};   // Angular velocity (rad/s)
    Real mass{1.0};                   // Mass (kg)
    Mat3x3 inertia;                   // Inertia tensor (kg·m²)
};
```

### Getting and Setting State

```cpp
// Get current state
physics::EntityState state = engine.get_entity_state(aircraft);

// Modify state
state.position = Vec3{1000.0, 0.0, -5000.0};
state.velocity = Vec3{200.0, 0.0, 0.0};
state.mass = 15000.0;

// Update entity
engine.set_entity_state(aircraft, state);
```

### Derived Quantities

```cpp
// Get Euler angles from state
Real roll = state.get_roll();
Real pitch = state.get_pitch();
Real yaw = state.get_yaw();

// Or as a vector
Vec3 euler = state.get_euler_angles();

// Get rotation matrix
Mat3x3 dcm = state.get_rotation_matrix();
```

## Entity Domains

### Domain Types

```cpp
enum class Domain : UInt8 {
    Generic = 0,  // Custom/unspecified
    Air = 1,      // Aircraft, missiles
    Land = 2,     // Ground vehicles
    Sea = 3,      // Ships, submarines
    Space = 4     // Satellites, spacecraft
};
```

### Domain-Specific Behavior

Each domain has specialized physics models:

| Domain | Physics Models |
|--------|----------------|
| Air | Aerodynamics, Propulsion, FCS |
| Land | Terramechanics, Suspension |
| Sea | Hydrodynamics, Buoyancy, RAO |
| Space | Gravity, Drag, SGP4 |

## Entity Management

### Querying Entities

```cpp
// Check if entity exists
bool exists = engine.entity_exists(aircraft);

// Get entity count
SizeT count = engine.entity_count();

// Get all entities
std::vector<EntityId> all = engine.get_all_entities();

// Get entities by domain
std::vector<EntityId> aircraft_list = engine.get_entities_by_domain(Domain::Air);
```

### Destroying Entities

```cpp
// Remove entity from simulation
engine.destroy_entity(aircraft);
```

## Entity Forces

### Applying Forces

```cpp
physics::EntityForces forces;
forces.clear();

// Add force at CG
forces.add_force(Vec3{1000.0, 0.0, 0.0});  // 1000 N forward

// Add torque
forces.add_torque(Vec3{0.0, 500.0, 0.0});  // 500 N·m pitch

// Add force at specific point (creates torque)
Vec3 force = Vec3{0.0, 0.0, -1000.0};  // Lift
Vec3 point = Vec3{2.0, 0.0, 0.0};       // Point ahead of CG
Vec3 cg = Vec3{0.0, 0.0, 0.0};          // CG position
forces.add_force_at_point(force, point, cg);

// Apply to entity
engine.apply_forces(aircraft, forces);
```

## Entity Environment

### Getting Environment Data

```cpp
// Get environment at entity location
environment::Environment env = engine.get_environment(aircraft);

// Access atmospheric data
Real density = env.atmosphere.density;
Real temperature = env.atmosphere.temperature;
Real wind_x = env.atmosphere.wind.x;

// Access terrain data
Real terrain_elevation = env.terrain_elevation;
Vec3 surface_normal = env.terrain.normal;

// Access ocean data (if over water)
if (env.over_water) {
    Real wave_height = env.ocean.surface_elevation;
    Real water_density = env.ocean.density;
}
```

## Best Practices

### Entity Lifecycle

1. **Create** entities before simulation starts
2. **Initialize** state with valid values
3. **Update** state through forces, not direct modification
4. **Query** state for telemetry and logging
5. **Destroy** entities when no longer needed

### Performance Tips

1. **Batch operations**: Create/destroy entities in groups
2. **Cache IDs**: Store EntityId rather than looking up repeatedly
3. **Use domains**: Proper domain assignment enables optimized physics

### Common Patterns

```cpp
// Entity with initial conditions
EntityId create_aircraft(Engine& engine, const Vec3& position, Real heading) {
    EntityId id = engine.create_entity("Aircraft", Domain::Air);

    physics::EntityState state;
    state.position = position;
    state.velocity = Vec3{200.0, 0.0, 0.0};  // Forward velocity
    state.orientation = Quaternion::from_euler(0.0, 0.0, heading);
    state.mass = 15000.0;
    engine.set_entity_state(id, state);

    return id;
}
```

## See Also

- [Force Generators](force-generators.md) - Physics models
- [Environment](environment.md) - Environmental services
- [Coordinates](coordinates.md) - Coordinate systems
- [API Reference](../api/core.md) - Complete API
