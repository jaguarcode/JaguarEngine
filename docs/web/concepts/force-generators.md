# Force Generators

Force generators are the physics models that compute forces and moments acting on entities. This guide covers the force generator system and available models.

## Overview

Force generators implement the `IForceGenerator` interface:

```cpp
class IForceGenerator {
public:
    virtual void compute_forces(const EntityState& state,
                                const Environment& env,
                                Real dt,
                                EntityForces& forces) = 0;
};
```

Each generator:
1. Receives entity state and environment data
2. Computes forces and moments
3. Accumulates results in the forces structure

## Force Generator Categories

### By Domain

| Domain | Force Generators |
|--------|------------------|
| Air | Aerodynamics, Propulsion, Ground Reaction |
| Land | Terramechanics, Suspension, Tracked Vehicle |
| Sea | Buoyancy, Hydrodynamics, Wave Loading |
| Space | Gravity, Atmospheric Drag, Solar Radiation |

### By Physics Type

| Type | Description |
|------|-------------|
| **Environmental** | Gravity, atmospheric forces |
| **Locomotion** | Propulsion, terramechanics |
| **Interaction** | Ground contact, collision |
| **Stability** | Buoyancy, aerodynamic damping |

## Domain-Specific Interfaces

### Aerodynamics (Air Domain)

```cpp
class IAerodynamicsModel : public IForceGenerator {
public:
    virtual Real get_cl() const = 0;    // Lift coefficient
    virtual Real get_cd() const = 0;    // Drag coefficient
    virtual Real get_cm() const = 0;    // Pitching moment
    virtual Real get_alpha() const = 0; // Angle of attack (rad)
    virtual Real get_beta() const = 0;  // Sideslip angle (rad)
    virtual Real get_mach() const = 0;  // Mach number
    virtual Real get_qbar() const = 0;  // Dynamic pressure (Pa)
};
```

### Propulsion (Air Domain)

```cpp
class IPropulsionModel : public IForceGenerator {
public:
    virtual Real get_thrust() const = 0;          // Current thrust (N)
    virtual Real get_fuel_flow() const = 0;       // Fuel consumption (kg/s)
    virtual Real get_fuel_remaining() const = 0;  // Remaining fuel (kg)
    virtual bool is_running() const = 0;          // Engine status
};
```

### Terramechanics (Land Domain)

```cpp
class ITerramechanicsModel : public IForceGenerator {
public:
    virtual Real get_sinkage() const = 0;           // Sinkage (m)
    virtual Real get_motion_resistance() const = 0; // Resistance (N)
    virtual Real get_traction() const = 0;          // Traction (N)
    virtual Real get_slip_ratio() const = 0;        // Slip ratio
};
```

### Hydrodynamics (Sea Domain)

```cpp
class IHydrodynamicsModel : public IForceGenerator {
public:
    virtual Real get_buoyancy() const = 0;  // Buoyancy force (N)
    virtual Real get_draft() const = 0;     // Draft (m)
    virtual Real get_heel() const = 0;      // Roll angle (rad)
    virtual Real get_trim() const = 0;      // Pitch angle (rad)
};
```

## Using Force Generators

### Manual Force Computation

```cpp
// Create models
domain::air::AerodynamicsModel aero;
domain::air::PropulsionModel engine_model;

// Configure
aero.set_reference_area(27.87);
aero.set_reference_chord(3.45);
engine_model.set_max_thrust(130000.0);

// In simulation loop
physics::EntityForces forces;
forces.clear();

auto state = engine.get_entity_state(aircraft);
auto env = engine.get_environment(aircraft);

// Compute forces from each model
aero.compute_forces(state, env, dt, forces);
engine_model.compute_forces(state, env, dt, forces);

// Add gravity
forces.add_force(Vec3{0.0, 0.0, state.mass * constants::G0});

// Apply accumulated forces
engine.apply_forces(aircraft, forces);
```

### Force Accumulation

The `EntityForces` structure accumulates all forces:

```cpp
struct EntityForces {
    Vec3 force{0, 0, 0};    // Total force (N)
    Vec3 torque{0, 0, 0};   // Total torque (N·m)

    void clear();
    void add_force(const Vec3& f);
    void add_torque(const Vec3& t);
    void add_force_at_point(const Vec3& f, const Vec3& point, const Vec3& cg);
};
```

## Built-in Force Generators

### Air Domain

| Model | Description |
|-------|-------------|
| `AerodynamicsModel` | Coefficient-based aerodynamics with table lookup |
| `PropulsionModel` | Turbofan/turbojet thrust with altitude-Mach correction |
| `FlightControlSystem` | Control surface mapping |

### Land Domain

| Model | Description |
|-------|-------------|
| `TerramechanicsModel` | Bekker-Wong soil-vehicle interaction |
| `SuspensionModel` | Multi-unit spring-damper system |
| `TrackedVehicleModel` | Track dynamics and propulsion |

### Sea Domain

| Model | Description |
|-------|-------------|
| `BuoyancyModel` | Hydrostatic buoyancy with metacentric stability |
| `HydrodynamicsModel` | MMG maneuvering model |
| `WaveModel` | Wave spectrum generation |
| `RAOModel` | Ship motion response |

### Space Domain

| Model | Description |
|-------|-------------|
| `GravityModel` | Point mass to EGM2008 |
| `AtmosphericDragModel` | Drag with JB08 atmosphere |
| `SGP4Propagator` | TLE-based orbit propagation |

## Force Generator Patterns

### Composite Force Generator

Combine multiple generators into one:

```cpp
class CompositeForceGenerator : public IForceGenerator {
public:
    void add_generator(std::unique_ptr<IForceGenerator> gen) {
        generators_.push_back(std::move(gen));
    }

    void compute_forces(const EntityState& state,
                        const Environment& env,
                        Real dt,
                        EntityForces& forces) override {
        for (auto& gen : generators_) {
            gen->compute_forces(state, env, dt, forces);
        }
    }

private:
    std::vector<std::unique_ptr<IForceGenerator>> generators_;
};
```

### Conditional Force Generator

Apply forces based on conditions:

```cpp
class ConditionalDrag : public IForceGenerator {
public:
    void compute_forces(const EntityState& state,
                        const Environment& env,
                        Real dt,
                        EntityForces& forces) override {
        // Only apply drag above certain altitude
        if (env.altitude < 800000.0) {  // Below 800 km
            drag_model_.compute_forces(state, env, dt, forces);
        }
    }

private:
    AtmosphericDragModel drag_model_;
};
```

## Best Practices

### Force Computation Order

1. **Environmental forces** (gravity, drag)
2. **Propulsion forces** (thrust)
3. **Aerodynamic/Hydrodynamic forces**
4. **Contact forces** (ground, collision)
5. **Control forces** (reactions, corrections)

### Numerical Stability

- Set minimum velocity thresholds
- Clamp coefficient values
- Use appropriate time steps
- Apply forces in body frame when appropriate

### Performance

- Cache intermediate calculations
- Reuse lookup tables
- Batch similar computations
- Profile hot paths

## See Also

- [Entities](entities.md) - Entity management
- [Integration](integration.md) - State propagation
- [Custom Models](../advanced/custom-models.md) - Creating custom generators
- [API Reference](../api/physics.md) - Physics API
