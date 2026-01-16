# Physics Module Documentation

The Physics module implements the core 6-DOF rigid body dynamics simulation, including entity management, force accumulation, and state propagation through numerical integration.

## Headers

| Header | Purpose |
|--------|---------|
| `jaguar/physics/entity.h` | Entity management, state storage (SoA) |
| `jaguar/physics/solver.h` | Numerical integrators (RK4, ABM4, Euler) |
| `jaguar/physics/force.h` | Force generator interfaces |

## Entity System

### EntityState

The complete physical state of a simulated entity:

```cpp
struct EntityState {
    Vec3 position{0.0, 0.0, 0.0};      // Position in ECEF (meters)
    Vec3 velocity{0.0, 0.0, 0.0};      // Velocity in ECEF (m/s)
    Quat orientation{1.0, 0.0, 0.0, 0.0}; // Orientation (body to ECEF)
    Vec3 angular_velocity{0.0, 0.0, 0.0}; // Angular velocity in body frame (rad/s)
    Vec3 acceleration{0.0, 0.0, 0.0};  // Linear acceleration (m/s²)
    Vec3 angular_accel{0.0, 0.0, 0.0}; // Angular acceleration (rad/s²)
    Real mass{1.0};                     // Mass (kg)
    Mat3x3 inertia{};                   // Inertia tensor in body frame (kg·m²)
};
```

### EntityForces

Accumulated forces and torques:

```cpp
struct EntityForces {
    Vec3 force{0.0, 0.0, 0.0};   // Total force in body frame (N)
    Vec3 torque{0.0, 0.0, 0.0};  // Total torque in body frame (N·m)

    void clear() noexcept;
    void add_force(const Vec3& f) noexcept;
    void add_torque(const Vec3& t) noexcept;
    void add_force_at_point(const Vec3& f, const Vec3& r) noexcept;
};
```

**Usage:**
```cpp
EntityForces forces;
forces.clear();

// Add simple force
forces.add_force(Vec3{1000.0, 0.0, 0.0});  // 1000N forward

// Add torque directly
forces.add_torque(Vec3{0.0, 500.0, 0.0});  // 500 N·m pitch

// Add force at offset (computes torque automatically)
Vec3 thrust{5000.0, 0.0, 0.0};
Vec3 engine_position{-3.0, 0.0, 0.0};  // 3m behind CG
forces.add_force_at_point(thrust, engine_position);
```

### Entity Handle

Lightweight entity handle with metadata:

```cpp
struct Entity {
    EntityId id{INVALID_ENTITY_ID};     // Unique identifier
    UInt32 state_index{0};              // Index into SoA state arrays
    ComponentMask components{0};         // Bitfield of attached components
    Domain primary_domain{Domain::Generic}; // Primary physics domain
    std::string name;                    // Entity name
    bool active{true};                   // Is entity being simulated

    bool is_valid() const noexcept;
};
```

### Component Bits

Bitmask values for component attachment:

```cpp
namespace ComponentBits {
    constexpr ComponentMask Aerodynamics   = 1ULL << 0;
    constexpr ComponentMask Propulsion     = 1ULL << 1;
    constexpr ComponentMask GroundContact  = 1ULL << 2;
    constexpr ComponentMask Terramechanics = 1ULL << 3;
    constexpr ComponentMask Hydrodynamics  = 1ULL << 4;
    constexpr ComponentMask Buoyancy       = 1ULL << 5;
    constexpr ComponentMask OrbitalDynamics= 1ULL << 6;
    constexpr ComponentMask Gravity        = 1ULL << 7;
    constexpr ComponentMask Missile        = 1ULL << 8;
    constexpr ComponentMask Seeker         = 1ULL << 9;
    constexpr ComponentMask Guidance       = 1ULL << 10;
    constexpr ComponentMask FlightControl  = 1ULL << 11;
    constexpr ComponentMask Suspension     = 1ULL << 12;
    constexpr ComponentMask Collision      = 1ULL << 13;
    constexpr ComponentMask Damageable     = 1ULL << 14;
}
```

## Entity State Storage (SoA)

The `EntityStateStorage` class implements Structure of Arrays (SoA) memory layout for cache-efficient batch processing:

```cpp
class EntityStateStorage {
public:
    static constexpr SizeT ALIGNMENT = 64;  // Cache line alignment

    // Memory management
    void reserve(SizeT capacity);
    UInt32 allocate();
    void free(UInt32 index);

    SizeT size() const noexcept;
    SizeT active_count() const noexcept;

    // State access by index
    Vec3& position(UInt32 idx);
    Vec3& velocity(UInt32 idx);
    Quat& orientation(UInt32 idx);
    Vec3& angular_velocity(UInt32 idx);
    Vec3& acceleration(UInt32 idx);
    Vec3& angular_acceleration(UInt32 idx);
    Real& mass(UInt32 idx);
    Mat3x3& inertia(UInt32 idx);
    EntityForces& forces(UInt32 idx);

    // Direct array access for batch operations
    Vec3* positions_data();
    Vec3* velocities_data();
    Quat* orientations_data();

    // Bulk operations
    void set_state(UInt32 idx, const EntityState& state);
    EntityState get_state(UInt32 idx) const;
    void clear_forces();
};
```

**Why SoA?**
- Cache-efficient: Sequential memory access patterns
- SIMD-friendly: Aligned arrays enable vectorization
- Batch processing: Update all positions, then all velocities, etc.

## Entity Manager

High-level entity lifecycle management:

```cpp
class EntityManager {
public:
    // Entity lifecycle
    EntityId create_entity(const std::string& name, Domain domain = Domain::Generic);
    void destroy_entity(EntityId id);
    bool exists(EntityId id) const;

    // Entity access
    Entity* get_entity(EntityId id);
    const Entity* get_entity(EntityId id) const;

    // State access
    EntityState get_state(EntityId id) const;
    void set_state(EntityId id, const EntityState& state);

    // Direct storage access
    EntityStateStorage& get_state_storage();

    // Iteration
    template<typename Func>
    void for_each(Func&& func);

    template<typename Func>
    void for_each_with(ComponentMask required_components, Func&& func);

    // Counts
    SizeT entity_count() const;
    SizeT active_entity_count() const;
};
```

**Usage:**
```cpp
EntityManager manager;

// Create entities
EntityId aircraft = manager.create_entity("F16", Domain::Air);
EntityId tank = manager.create_entity("M1A2", Domain::Land);

// Set initial state
EntityState state;
state.position = Vec3{0.0, 0.0, -1000.0};
state.velocity = Vec3{200.0, 0.0, 0.0};
state.mass = 12000.0;
state.inertia = Mat3x3::Inertia(5000.0, 30000.0, 35000.0);
manager.set_state(aircraft, state);

// Iterate over air domain entities
manager.for_each_with(ComponentBits::Aerodynamics, [](Entity& entity) {
    // Process aerodynamic entities
});

// Cleanup
manager.destroy_entity(aircraft);
```

## Numerical Integrators

### IStatePropagator Interface

Base interface for all integration methods:

```cpp
class IStatePropagator {
public:
    virtual void integrate(
        EntityState& state,
        const EntityForces& forces,
        Real dt) = 0;

    virtual const std::string& name() const = 0;
    virtual int order() const = 0;
    virtual void reset() {}  // For multi-step methods
};
```

### RK4 Integrator

Fourth-order Runge-Kutta integration with Euler's equations for rotational motion:

```cpp
class RK4Integrator : public IStatePropagator {
public:
    void integrate(EntityState& state, const EntityForces& forces, Real dt) override;
    const std::string& name() const override;  // Returns "RK4"
    int order() const override;  // Returns 4
};
```

**RK4 Algorithm:**
1. Compute derivatives at t (k1)
2. Compute derivatives at t + dt/2 using k1 (k2)
3. Compute derivatives at t + dt/2 using k2 (k3)
4. Compute derivatives at t + dt using k3 (k4)
5. Weighted average: x_new = x + (k1 + 2*k2 + 2*k3 + k4) * dt/6

**Euler's Equations for Rotational Motion:**
```
I * ω̇ = τ - ω × (I * ω)
```
Where:
- I = inertia tensor
- ω = angular velocity
- τ = applied torque
- ω̇ = angular acceleration

### ABM4 Integrator

Adams-Bashforth-Moulton predictor-corrector method:

```cpp
class ABM4Integrator : public IStatePropagator {
public:
    void integrate(EntityState& state, const EntityForces& forces, Real dt) override;
    const std::string& name() const override;  // Returns "ABM4"
    int order() const override;  // Returns 4
    void reset() override;  // Clear history for restart
};
```

**Characteristics:**
- More efficient than RK4 after startup (1 function evaluation vs 4)
- Requires startup phase (uses RK4 for first 4 steps)
- Maintains history of previous accelerations
- Better for long-running simulations

### Euler Integrator

Simple first-order integration (for testing/comparison):

```cpp
class EulerIntegrator : public IStatePropagator {
public:
    void integrate(EntityState& state, const EntityForces& forces, Real dt) override;
    const std::string& name() const override;  // Returns "Euler"
    int order() const override;  // Returns 1
};
```

**When to use which integrator:**
| Integrator | Accuracy | Cost | Best For |
|------------|----------|------|----------|
| Euler | Low | 1 eval/step | Testing, very stiff systems |
| RK4 | High | 4 evals/step | General purpose, short runs |
| ABM4 | High | 1 eval/step (after startup) | Long simulations |

## Physics System

Central coordinator for physics processing:

```cpp
class PhysicsSystem {
public:
    // Integrator configuration
    void set_propagator(std::unique_ptr<IStatePropagator> propagator);
    IStatePropagator* get_propagator();

    // Simulation stepping
    void update(EntityManager& entity_manager, Real dt);
    void update_entity(Entity& entity, EntityStateStorage& storage, Real dt);

    // Parallel processing
    void set_parallel_enabled(bool enabled);
    bool is_parallel_enabled() const;
};
```

**Usage:**
```cpp
PhysicsSystem physics;

// Use RK4 integrator
physics.set_propagator(std::make_unique<RK4Integrator>());

// Enable parallel processing
physics.set_parallel_enabled(true);

// Simulation loop
Real dt = 0.01;  // 100 Hz
while (running) {
    physics.update(entity_manager, dt);
}
```

## Force Generator System

### IForceGenerator Interface

Base interface for all force-producing components:

```cpp
class IForceGenerator {
public:
    virtual void compute_forces(
        const EntityState& state,
        const environment::Environment& env,
        Real dt,
        EntityForces& out_forces) = 0;

    virtual bool initialize() { return true; }
    virtual const std::string& name() const = 0;
    virtual Domain domain() const = 0;
    virtual bool is_enabled() const;
    virtual void set_enabled(bool enabled);
};
```

### Specialized Interfaces

**IAerodynamicsModel** (Air domain):
```cpp
class IAerodynamicsModel : public IForceGenerator {
public:
    virtual Real get_cl() const = 0;    // Lift coefficient
    virtual Real get_cd() const = 0;    // Drag coefficient
    virtual Real get_cm() const = 0;    // Pitching moment coefficient
    virtual Real get_alpha() const = 0; // Angle of attack (rad)
    virtual Real get_beta() const = 0;  // Sideslip angle (rad)
    virtual Real get_mach() const = 0;  // Mach number
    virtual Real get_qbar() const = 0;  // Dynamic pressure (Pa)
};
```

**IPropulsionModel** (Air/Space domain):
```cpp
class IPropulsionModel : public IForceGenerator {
public:
    virtual Real get_thrust() const = 0;        // Thrust (N)
    virtual Real get_fuel_flow() const = 0;     // Fuel flow (kg/s)
    virtual Real get_fuel_remaining() const = 0; // Fuel mass (kg)
    virtual void set_throttle(Real throttle) = 0; // 0.0-1.0
    virtual bool is_running() const = 0;
};
```

**ITerramechanicsModel** (Land domain):
```cpp
class ITerramechanicsModel : public IForceGenerator {
public:
    virtual Real get_sinkage() const = 0;           // Sinkage (m)
    virtual Real get_motion_resistance() const = 0; // Resistance (N)
    virtual Real get_traction() const = 0;          // Traction (N)
    virtual Real get_slip_ratio() const = 0;        // Slip ratio
};
```

**IHydrodynamicsModel** (Sea domain):
```cpp
class IHydrodynamicsModel : public IForceGenerator {
public:
    virtual Real get_buoyancy() const = 0; // Buoyancy (N)
    virtual Real get_draft() const = 0;    // Draft (m)
    virtual Real get_heel() const = 0;     // Heel angle (rad)
    virtual Real get_trim() const = 0;     // Trim angle (rad)
};
```

**IGravityModel** (Generic):
```cpp
class IGravityModel : public IForceGenerator {
public:
    virtual Vec3 get_gravity_acceleration() const = 0;
    virtual void set_fidelity(int level) = 0;  // 0=point, 1=J2, 2=J4, 3=EGM96
};
```

### Force Generator Registry

```cpp
class ForceGeneratorRegistry {
public:
    void register_generator(std::unique_ptr<IForceGenerator> generator);
    std::vector<IForceGenerator*> get_generators(Domain domain);
    std::vector<IForceGenerator*> get_enabled_generators();
    IForceGenerator* find(const std::string& name);
    void clear();
    SizeT size() const;
};
```

### Factory Functions

```cpp
namespace force {
    // Gravity models
    std::unique_ptr<IGravityModel> create_simple_gravity(Real g = 9.80665);
    std::unique_ptr<IGravityModel> create_wgs84_gravity();

    // Aerodynamics
    std::unique_ptr<IAerodynamicsModel> create_simple_aerodynamics();
    std::unique_ptr<IForceGenerator> create_drag_model(Real cd = 0.5, Real area = 1.0);
}
```

## Quaternion Utilities

Helper functions for quaternion operations:

```cpp
namespace quaternion_utils {
    // Integration
    Quat integrate(const Quat& q, const Vec3& omega, Real dt);
    Quat integrate_rk4(const Quat& q, const Vec3& omega, Real dt);

    // Conversions
    Quat normalize(const Quat& q);
    Quat from_euler(Real roll, Real pitch, Real yaw);
    void to_euler(const Quat& q, Real& roll, Real& pitch, Real& yaw);
    Mat3x3 to_rotation_matrix(const Quat& q);
}
```

**Quaternion derivative formula:**
```
q̇ = 0.5 * q * ω
```
Where ω is angular velocity as a pure quaternion (0, ωx, ωy, ωz).

## Simulation Loop Pattern

Complete simulation loop example:

```cpp
#include <jaguar/jaguar.h>

int main() {
    using namespace jaguar;

    // Create systems
    physics::EntityManager entities;
    physics::PhysicsSystem physics;
    physics::ForceGeneratorRegistry forces;

    // Configure physics
    physics.set_propagator(std::make_unique<physics::RK4Integrator>());
    physics.set_parallel_enabled(true);

    // Register force generators
    forces.register_generator(physics::force::create_wgs84_gravity());
    forces.register_generator(physics::force::create_simple_aerodynamics());

    // Create entity
    EntityId aircraft = entities.create_entity("TestAircraft", Domain::Air);

    physics::EntityState state;
    state.position = Vec3{0.0, 0.0, -10000.0};  // 10km altitude
    state.velocity = Vec3{250.0, 0.0, 0.0};     // 250 m/s
    state.mass = 15000.0;
    state.inertia = Mat3x3::Inertia(10000.0, 50000.0, 55000.0);
    entities.set_state(aircraft, state);

    // Simulation loop
    Real dt = 0.01;  // 100 Hz
    Real sim_time = 0.0;
    Real end_time = 60.0;  // 1 minute

    while (sim_time < end_time) {
        // Clear forces
        entities.get_state_storage().clear_forces();

        // Compute forces from all generators
        auto& storage = entities.get_state_storage();
        Entity* entity = entities.get_entity(aircraft);

        if (entity && entity->active) {
            physics::EntityState current = storage.get_state(entity->state_index);
            physics::EntityForces& entity_forces = storage.forces(entity->state_index);

            environment::Environment env;
            // ... set environment conditions ...

            for (auto* gen : forces.get_enabled_generators()) {
                gen->compute_forces(current, env, dt, entity_forces);
            }
        }

        // Propagate states
        physics.update(entities, dt);

        sim_time += dt;
    }

    // Get final state
    physics::EntityState final_state = entities.get_state(aircraft);
    // ... output results ...

    return 0;
}
```

## Performance Considerations

### Memory Layout
- SoA layout provides 2-4x speedup for batch operations
- 64-byte alignment matches cache line size
- Contiguous arrays enable SIMD vectorization

### Parallel Processing
- Entity updates are independent and parallelizable
- Force computation can be parallelized per entity
- Integration is per-entity with no data dependencies

### Timestep Selection
- Typical ranges: 10-1000 Hz (0.001-0.1 seconds)
- Higher frequency for high-speed vehicles
- Lower frequency for slow-moving vessels
- ABM4 allows larger timesteps than RK4

### Best Practices
```cpp
// Reserve storage upfront for better performance
entities.get_state_storage().reserve(1000);

// Use direct array access for batch operations
Vec3* positions = storage.positions_data();
Vec3* velocities = storage.velocities_data();

// Clear forces once per frame, not per entity
storage.clear_forces();

// Use component masks for efficient filtering
entities.for_each_with(ComponentBits::Aerodynamics | ComponentBits::Propulsion,
    [](Entity& e) { /* process aircraft */ });
```
