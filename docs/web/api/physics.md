# Physics API Reference

Entity state, forces, integrators, and physics engine components.

**Headers:** `jaguar/physics/entity.h`, `jaguar/physics/solver.h`, `jaguar/physics/force.h`

## EntityState

Complete kinematic state of an entity.

```cpp
struct EntityState {
    Vec3 position{0, 0, 0};           // Position in ECEF (m)
    Vec3 velocity{0, 0, 0};           // Velocity (m/s)
    Quaternion orientation;            // Attitude quaternion
    Vec3 angular_velocity{0, 0, 0};   // Angular velocity (rad/s)
    Real mass{1.0};                   // Mass (kg)
    Mat3x3 inertia;                   // Inertia tensor (kg·m²)

    // Derived quantities
    Real get_roll() const;            // Roll angle (rad)
    Real get_pitch() const;           // Pitch angle (rad)
    Real get_yaw() const;             // Yaw/heading angle (rad)
    Vec3 get_euler_angles() const;    // {roll, pitch, yaw}
    Mat3x3 get_rotation_matrix() const;

    // Body frame conversions
    Vec3 world_to_body(const Vec3& world) const;
    Vec3 body_to_world(const Vec3& body) const;
};
```

### Example

```cpp
physics::EntityState state;

// Set position (San Francisco, 10km altitude)
Real lat = 37.7749 * DEG_TO_RAD;
Real lon = -122.4194 * DEG_TO_RAD;
Real alt = 10000.0;
state.position = transforms::geodetic_to_ecef(lat, lon, alt);

// Set velocity (heading east at 200 m/s)
state.velocity = Vec3{200.0, 0.0, 0.0};  // Will need rotation

// Set orientation (level flight, heading east)
state.orientation = Quaternion::from_euler(0.0, 0.0, 90.0 * DEG_TO_RAD);

// Set mass properties
state.mass = 15000.0;  // 15 tonnes
state.inertia = Mat3x3::identity();
state.inertia.data[0][0] = 20000;   // Ixx
state.inertia.data[1][1] = 100000;  // Iyy
state.inertia.data[2][2] = 110000;  // Izz

// Query derived values
Real pitch = state.get_pitch() * RAD_TO_DEG;
std::cout << "Pitch: " << pitch << " degrees\n";
```

## EntityForces

Accumulated forces and torques acting on an entity.

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

### Methods

#### `clear()`
Reset forces and torques to zero.

#### `add_force(const Vec3& f)`
Add a force vector (in world coordinates).

#### `add_torque(const Vec3& t)`
Add a torque vector (in world coordinates).

#### `add_force_at_point(const Vec3& f, const Vec3& point, const Vec3& cg)`
Add a force at a specific point, automatically computing the resulting torque about the center of gravity.

### Example

```cpp
physics::EntityForces forces;
forces.clear();

// Gravity (world frame)
Vec3 gravity{0.0, 0.0, state.mass * constants::G0};
forces.add_force(gravity);

// Thrust (body frame, converted to world)
Vec3 thrust_body{50000.0, 0.0, 0.0};  // 50 kN forward
Vec3 thrust_world = state.orientation.rotate(thrust_body);
forces.add_force(thrust_world);

// Force at point (creates torque)
Vec3 wing_force{0.0, 0.0, -5000.0};  // 5 kN lift on right wing
Vec3 wing_position{0.0, 5.0, 0.0};   // 5m right of CG
Vec3 cg{0.0, 0.0, 0.0};
forces.add_force_at_point(wing_force, wing_position, cg);

// Apply to entity
engine.apply_forces(entity_id, forces);
```

## EntityManager

Manages entity creation, destruction, and state access.

```cpp
class EntityManager {
public:
    // Entity lifecycle
    EntityId create_entity(const std::string& name, Domain domain);
    void destroy_entity(EntityId id);
    bool entity_exists(EntityId id) const;

    // State access
    EntityState get_state(EntityId id) const;
    void set_state(EntityId id, const EntityState& state);

    // Properties
    Domain get_domain(EntityId id) const;
    std::string get_name(EntityId id) const;

    // Queries
    SizeT entity_count() const;
    std::vector<EntityId> get_all_entities() const;
    std::vector<EntityId> get_entities_by_domain(Domain domain) const;
};
```

### Example

```cpp
// Get entity manager from engine
auto& manager = engine.get_entity_manager();

// Create entities
EntityId fighter = manager.create_entity("F-16", Domain::Air);
EntityId tank = manager.create_entity("M1A2", Domain::Land);

// Query entities
std::cout << "Total entities: " << manager.entity_count() << "\n";

// Get all aircraft
auto aircraft = manager.get_entities_by_domain(Domain::Air);
for (EntityId id : aircraft) {
    std::cout << "Aircraft: " << manager.get_name(id) << "\n";
}

// Check existence
if (manager.entity_exists(fighter)) {
    auto state = manager.get_state(fighter);
    // ...
}

// Destroy entity
manager.destroy_entity(tank);
```

## Integrators

Numerical integration methods for state propagation.

### IStatePropagator Interface

```cpp
class IStatePropagator {
public:
    virtual void propagate(EntityState& state,
                           const EntityForces& forces,
                           Real dt) = 0;
};
```

### RK4Integrator

Fourth-order Runge-Kutta integrator.

```cpp
class RK4Integrator : public IStatePropagator {
public:
    void propagate(EntityState& state,
                   const EntityForces& forces,
                   Real dt) override;
};
```

**Characteristics:**
- Fourth-order accuracy
- Good stability
- Requires 4 derivative evaluations per step
- Recommended for general use

### ABM4Integrator

Adams-Bashforth-Moulton predictor-corrector.

```cpp
class ABM4Integrator : public IStatePropagator {
public:
    void propagate(EntityState& state,
                   const EntityForces& forces,
                   Real dt) override;

    void reset();  // Clear history for new simulation
};
```

**Characteristics:**
- Fourth-order accuracy
- More efficient than RK4 (fewer evaluations)
- Requires startup phase
- Good for long simulations with constant dt

### Example

```cpp
// Create integrator
auto integrator = std::make_unique<RK4Integrator>();

// Simulation loop
physics::EntityState state;
physics::EntityForces forces;
Real dt = 0.01;

for (Real t = 0; t < 100.0; t += dt) {
    // Compute forces
    forces.clear();
    compute_forces(state, forces);

    // Propagate state
    integrator->propagate(state, forces, dt);
}
```

## Force Generator Interface

Base interface for all force generators.

```cpp
class IForceGenerator {
public:
    virtual void compute_forces(const EntityState& state,
                                const environment::Environment& env,
                                Real dt,
                                EntityForces& forces) = 0;
};
```

### Domain-Specific Interfaces

#### IAerodynamicsModel

```cpp
class IAerodynamicsModel : public IForceGenerator {
public:
    virtual Real get_cl() const = 0;      // Lift coefficient
    virtual Real get_cd() const = 0;      // Drag coefficient
    virtual Real get_cm() const = 0;      // Pitching moment coeff
    virtual Real get_alpha() const = 0;   // Angle of attack (rad)
    virtual Real get_beta() const = 0;    // Sideslip angle (rad)
    virtual Real get_mach() const = 0;    // Mach number
    virtual Real get_qbar() const = 0;    // Dynamic pressure (Pa)
};
```

#### IPropulsionModel

```cpp
class IPropulsionModel : public IForceGenerator {
public:
    virtual Real get_thrust() const = 0;          // Current thrust (N)
    virtual Real get_fuel_flow() const = 0;       // Fuel flow (kg/s)
    virtual Real get_fuel_remaining() const = 0;  // Remaining fuel (kg)
    virtual bool is_running() const = 0;          // Engine running
};
```

#### ITerramechanicsModel

```cpp
class ITerramechanicsModel : public IForceGenerator {
public:
    virtual Real get_sinkage() const = 0;           // Sinkage depth (m)
    virtual Real get_motion_resistance() const = 0; // Resistance (N)
    virtual Real get_traction() const = 0;          // Traction force (N)
    virtual Real get_slip_ratio() const = 0;        // Slip ratio
};
```

#### IHydrodynamicsModel

```cpp
class IHydrodynamicsModel : public IForceGenerator {
public:
    virtual Real get_buoyancy() const = 0;  // Buoyancy force (N)
    virtual Real get_draft() const = 0;     // Draft depth (m)
    virtual Real get_heel() const = 0;      // Heel angle (rad)
    virtual Real get_trim() const = 0;      // Trim angle (rad)
};
```

### Example: Custom Force Generator

```cpp
class SimpleDragModel : public IForceGenerator {
public:
    void set_drag_coefficient(Real cd) { cd_ = cd; }
    void set_reference_area(Real area) { area_ = area; }

    void compute_forces(const EntityState& state,
                        const Environment& env,
                        Real dt,
                        EntityForces& forces) override {
        Real speed = state.velocity.norm();
        if (speed < 0.001) return;

        // Drag = 0.5 * rho * V^2 * Cd * A
        Real qbar = 0.5 * env.atmosphere.density * speed * speed;
        Real drag_mag = qbar * cd_ * area_;

        // Drag opposes velocity
        Vec3 drag = state.velocity.normalized() * (-drag_mag);
        forces.add_force(drag);
    }

private:
    Real cd_{1.0};
    Real area_{1.0};
};
```

## See Also

- [Core API](core.md) - Basic types and vectors
- [Air Domain API](air.md) - Aerodynamics models
- [Land Domain API](land.md) - Terramechanics models
- [Sea Domain API](sea.md) - Hydrodynamics models
- [Concepts: Integration](../concepts/integration.md) - Integration methods

