# Physics Module Documentation

The Physics module implements the core 6-DOF rigid body dynamics simulation, including entity management, force accumulation, and state propagation through numerical integration.

## What's New in v0.6.0

This release includes significant enhancements to constraint solving, integration, and parallel physics processing:

- **Component Force Registry** (v0.6.0): Per-entity force model management with thread-safe lookup for heterogeneous entity populations
- **Constraint Solver Enhancements** (v0.6.0):
  - Split Impulse Solver: Separate position and velocity corrections for stable stacking
  - Early Exit Convergence: Automatic termination when contacts resolve (configurable threshold)
  - Warm Starting: Impulse caching with validation and magnitude clamping
  - Inverse Inertia Caching: Lazy computation and caching of matrix inverses (2-3x speedup)
- **Ground Contact Constraints** (v0.6.0): Terrain-normal based contact with Coulomb friction and material properties
- **Integrator Improvements** (v0.6.0):
  - Fixed-size history buffer for ABM4 (eliminates unbounded memory growth)
  - Angular acceleration history tracking (bug fix)
  - Quaternion integration with geodesic error metrics
  - Standardized constants from constants.h
- **Parallel Force Computation** (v0.6.0): Work-stealing thread pool integration with thread-safe model registry

## Headers

| Header | Purpose |
|--------|---------|
| `jaguar/physics/entity.h` | Entity management, state storage (SoA), inverse inertia caching |
| `jaguar/physics/solver.h` | Numerical integrators (RK4, ABM4, Euler, Adaptive) |
| `jaguar/physics/force.h` | Force generator interfaces and base classes |
| `jaguar/physics/component_force_registry.h` | Per-entity force model registry with thread-safe lookup |
| `jaguar/physics/constraints/constraint_solver.h` | Sequential Impulse solver with split impulse, warm starting, early exit |
| `jaguar/physics/constraints/ground_contact_constraint.h` | Terrain-normal contact with Coulomb friction |

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

Adams-Bashforth-Moulton predictor-corrector method with fixed history buffer (v0.6.0):

```cpp
class ABM4Integrator : public IStatePropagator {
public:
    void integrate(EntityState& state, const EntityForces& forces, Real dt) override;
    const std::string& name() const override { return "ABM4"; }
    int order() const override { return 4; }
    void reset() override;  // Clear history for restart

private:
    int step_count_{0};
    Vec3 accel_history_[4];           ///< Fixed history buffer (v0.6.0)
    Vec3 angular_accel_history_[4];   ///< Angular acceleration history
};
```

**Characteristics:**
- Fourth-order accuracy (same as RK4)
- More efficient after startup: 1 function evaluation vs 4 per step
- Fixed history buffer prevents unbounded memory growth
- Startup phase uses RK4 for first 4 steps to build history
- Better for long-running simulations (reduced CPU vs RK4)

**Performance:**
- RK4: 4 evaluations/step × cost per evaluation
- ABM4 (after startup): 1 evaluation/step × cost per evaluation
- Typical speedup: 3-4x after initial 4-step startup

**v0.6.0 Enhancement - Fixed History Buffer:**
```cpp
// Old (unbounded growth): history array grows indefinitely
std::vector<Vec3> accel_history;  // Memory grows with simulation time

// New (v0.6.0): fixed-size circular buffer
Vec3 accel_history_[4];  // Constant 4-element array

// Benefits:
// - No dynamic allocation after startup
// - Predictable memory footprint
// - Cache-friendly fixed size
// - Eliminates per-frame allocation overhead
```

**Angular Acceleration Bug Fix:**
```cpp
// v0.6.0 fixed a bug where angular acceleration history wasn't maintained
// Now properly caches angular_accel_history_[4] for multi-step method
// Ensures rotational dynamics are as accurate as translational
```

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

### Component Force Registry (v0.6.0)

Per-entity force model management for heterogeneous entity populations. Enables different entity types to use different force models simultaneously.

**Key Features:**
- Thread-safe read operations for parallel force computation
- Type-indexed storage for efficient lookup
- Per-entity model attachment without global force registries
- Integrates seamlessly with ComponentBits for filtering

```cpp
class ComponentForceRegistry {
public:
    // Register force model for an entity
    template<typename T>
    void register_model(EntityId entity_id, std::unique_ptr<T> model);

    // Retrieve force model for an entity
    template<typename T>
    T* get(EntityId entity_id) const;

    // Get all force generators for an entity
    std::vector<IForceGenerator*> get_all(EntityId entity_id) const;

    // Check if entity has specific model type
    template<typename T>
    bool has(EntityId entity_id) const;

    // Remove models
    void remove_entity(EntityId entity_id);
    template<typename T>
    void remove_model(EntityId entity_id);

    void clear();
    SizeT entity_count() const;
    SizeT model_count() const;

private:
    // Thread-safe access with shared_mutex
    mutable std::shared_mutex mutex_;

    // Key: (entity_id, type_index) -> force model
    std::unordered_map<ModelKey, std::unique_ptr<IForceGenerator>> models_;
};
```

**Usage Pattern - Entity-Specific Models:**
```cpp
ComponentForceRegistry registry;

// Aircraft: F-16 with advanced aerodynamics
auto f16_aero = std::make_unique<F16AerodynamicsModel>();
registry.register_model<IAerodynamicsModel>(f16_id, std::move(f16_aero));

// Helicopter: Different aerodynamic model
auto helo_aero = std::make_unique<HelicopterAerodynamicsModel>();
registry.register_model<IAerodynamicsModel>(helo_id, std::move(helo_aero));

// Vehicle: Custom propulsion and terrain interaction
auto vehicle_prop = std::make_unique<DieselEngineModel>();
registry.register_model<IPropulsionModel>(tank_id, std::move(vehicle_prop));

auto vehicle_terrain = std::make_unique<TankTerramechanicsModel>();
registry.register_model<ITerramechanicsModel>(tank_id, std::move(vehicle_terrain));
```

**Using in Force Computation Loop:**
```cpp
// During physics update, retrieve and use entity-specific models
for (auto& entity : entities) {
    EntityForces forces;

    // Aerodynamics (if attached)
    if (auto* aero = registry.get<IAerodynamicsModel>(entity.id)) {
        aero->compute_forces(state, env, dt, forces);
    }

    // Propulsion (if attached)
    if (auto* prop = registry.get<IPropulsionModel>(entity.id)) {
        prop->compute_forces(state, env, dt, forces);
    }

    // Gravity (universal)
    gravity->compute_forces(state, env, dt, forces);
}
```

**Benefits:**
- **Heterogeneous Populations**: Different aircraft/vehicles in same simulation
- **ComponentBits Integration**: Use masks to filter which entities to process
- **Efficient Lookup**: O(1) type-indexed access
- **Thread-Safe**: shared_mutex allows concurrent reads during parallel force computation
- **No Global State**: Each entity's models stored independently
- **Memory Efficient**: Models only allocated for entities that need them

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

## Constraint Solver (v0.6.0)

The Sequential Impulse (SI) solver implements a robust constraint resolution system with multiple stability and performance enhancements.

### Constraint Solver Configuration

Core configuration parameters:

```cpp
struct ConstraintSolverConfig {
    // Iteration control
    int velocity_iterations{8};         ///< Iterations for velocity solver
    int position_iterations{3};         ///< Iterations for position correction

    // Stabilization parameters
    Real position_correction{0.2};      ///< ERP (Error Reduction Parameter)
    Real baumgarte{0.2};                ///< Baumgarte factor for split impulse
    Real allowed_penetration{0.01};     ///< Slop before correction activates (m)
    Real slop{0.005};                   ///< Position error tolerance (m)

    // Convergence control
    Real convergence_threshold{1e-4};   ///< Early exit if max impulse below this
    Real contact_bias{0.1};             ///< Contact bias velocity (m/s)

    // Warm starting
    bool warm_starting{true};           ///< Enable impulse caching from last frame
    Real warm_start_factor{0.8};        ///< Scaling factor for cached impulses

    // Split impulse (separates position and velocity correction)
    bool split_impulse{false};          ///< Enable split impulse method
    Real max_correction_velocity{10.0}; ///< Maximum velocity correction (m/s)

    // Sleep optimization
    Real sleep_threshold{0.01};         ///< Velocity below which to sleep (m/s)
    int sleep_frames{60};               ///< Frames inactive before sleeping
};
```

### Split Impulse Solver

Separates position and velocity corrections for improved stability and reduced jitter:

**Algorithm (when enabled):**
1. **Position Phase**: Resolve penetrations through position adjustment
   - Computes position correction impulse separately
   - Prevents overlap without affecting velocities

2. **Velocity Phase**: Resolve relative velocities through impulse
   - Standard impulse-based velocity correction
   - Works independently from position phase

3. **Separation**: Maintains contact separation while conserving momentum

**Characteristics:**
- Eliminates "jitter" at contact boundaries during extended contact
- More stable than impulse-only solvers for stacked objects
- Prevents energy drift from repeated corrections
- Ideal for vehicle suspensions and terrain contact

**Configuration:**
```cpp
config.split_impulse = true;        // Enable separate position/velocity phases
config.position_iterations = 3;      // Position solver iterations
config.velocity_iterations = 8;      // Velocity solver iterations
config.baumgarte = 0.2;              // Position correction strength
```

### Early Exit Convergence

Automatic termination when contacts are fully resolved:

```cpp
Real convergence_threshold{1e-4};   // Exit when max impulse < this value
```

**How it Works:**
- Tracks maximum impulse applied in each iteration
- When max impulse drops below threshold, solver exits early
- Reduces CPU overhead for resolved contacts
- Configurable threshold balances accuracy vs performance

**Example:**
```cpp
solver.set_config(config);
// When iterating, if max_impulse < 1e-4, solver stops automatically
auto result = solver.solve();
std::cout << "Iterations used: " << result.iterations_used << std::endl;
```

### Warm Start with Impulse Clamping

Reuse impulses from previous frame for faster convergence:

```cpp
bool warm_starting{true};           // Enable impulse caching
Real warm_start_factor{0.8};        // Scale cached impulses by 0.8
```

**Benefits:**
- Faster convergence in subsequent frames (especially for persistent contacts)
- Reduces total solver iterations needed per frame
- Prevents divergence in long-running simulations

**Clamping Safety:**
```cpp
// During warm start, impulses are clamped to prevent explosion
Real max_impulse = effective_mass * constants::MAX_WARM_START_MULTIPLIER;
warm_impulse = std::clamp(warm_impulse, -max_impulse, max_impulse);
```

The clamping ensures that stale impulses don't cause unrealistic velocities.

### Inverse Inertia Caching (v0.6.0)

Pre-compute and cache inverse inertia tensors to eliminate redundant matrix inversion operations:

```cpp
struct EntityState {
    // ...
    Mat3x3 inertia{};
    mutable Mat3x3 inverse_inertia_cached{};  ///< Cached inverse inertia
    mutable bool inverse_inertia_dirty{true}; ///< Cache invalidation flag

    // Get inverse inertia with lazy computation
    const Mat3x3& get_inverse_inertia() const {
        if (inverse_inertia_dirty) {
            // Compute and cache inverse on demand
            Real det = inertia.determinant();
            if (std::abs(det) > 1e-10) {
                inverse_inertia_cached = inertia.inverse();
            } else {
                // Fallback to diagonal inverse for singular matrices
                inverse_inertia_cached = Mat3x3::Identity();
                if (inertia(0,0) > 1e-10) inverse_inertia_cached(0,0) = 1.0 / inertia(0,0);
                if (inertia(1,1) > 1e-10) inverse_inertia_cached(1,1) = 1.0 / inertia(1,1);
                if (inertia(2,2) > 1e-10) inverse_inertia_cached(2,2) = 1.0 / inertia(2,2);
            }
            inverse_inertia_dirty = false;
        }
        return inverse_inertia_cached;
    }

    // Invalidate cache when inertia changes
    void invalidate_inertia_cache() {
        inverse_inertia_dirty = true;
    }
};
```

**Performance Characteristics:**
- Lazy computation: Cache built on first access after inertia change
- Single matrix inversion per mass/inertia change (typically rare)
- Instead of 4-5 inversions per constraint iteration during solving
- Measured 2-3x speedup in constraint solver for systems with 50+ constraints
- Fallback logic handles near-singular inertia tensors gracefully

## Quaternion Error Metric (v0.6.0)

Geodesic distance-based error calculation for rotational constraints:

```cpp
namespace quaternion_utils {
    /**
     * @brief Compute geodesic distance between two quaternions
     *
     * Uses the angular distance formula for rotation space.
     * Result is normalized to [0, π].
     */
    Real geodesic_distance(const Quat& q1, const Quat& q2);

    /**
     * @brief Compute angular distance in radians
     *
     * Equivalent to geodesic distance for unit quaternions.
     */
    Real angular_distance(const Quat& q1, const Quat& q2);

    /**
     * @brief Compute error vector for constraint solving
     *
     * Returns a 3-component vector representing the axis-angle
     * error from current to target orientation.
     */
    Vec3 error_vector(const Quat& q_target, const Quat& q_current);
}
```

**Formula (Geodesic Distance):**
```
d = 2 * arccos(|q1 · q2|)
```

Where the absolute value of the dot product accounts for quaternion double-cover (q and -q represent the same rotation).

**Benefits:**
- Rotation-invariant: Unaffected by coordinate frame choice
- Prevents gimbal lock: Works at all orientations
- Geodesic metric: Shortest path on rotation manifold SO(3)
- Improved convergence: Better for rotational constraint solving
- Numerical stability: Handles quaternion singularities gracefully

**Use in Constraint Solving:**
When solving orientation constraints (joints, alignment), the geodesic distance provides a meaningful measure of how far the current rotation deviates from the desired rotation. This enables Baumgarte stabilization with proper error metrics for orientations.

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

    // Error metrics (Phase 7B)
    Real geodesic_distance(const Quat& q1, const Quat& q2);
    Real angular_distance(const Quat& q1, const Quat& q2);
    Vec3 error_vector(const Quat& q_target, const Quat& q_current);
}
```

**Quaternion derivative formula:**
```
q̇ = 0.5 * q * ω
```
Where ω is angular velocity as a pure quaternion (0, ωx, ωy, ωz).

## Ground Contact Constraints (v0.6.0)

Terrain-aware contact modeling with normal-aligned forces and material-specific friction.

### GroundContactConstraint

Specialized constraint for entity-terrain interactions with full terrain awareness:

```cpp
struct GroundContactParams : public ConstraintParams {
    Real restitution{0.0};              ///< Bounce coefficient (0-1)
    Real friction_coefficient{0.8};     ///< Coulomb friction coefficient
    Real rolling_friction{0.01};        ///< Rolling resistance coefficient
    bool enable_friction{true};         ///< Enable friction constraints
    Real restitution_threshold{0.5};    ///< Min velocity for bounce (m/s)
    Real contact_depth_bias{0.1};       ///< Baumgarte multiplier for penetration
};

class GroundContactConstraint : public IConstraint {
public:
    // Create ground contact from terrain data
    GroundContactConstraint(
        const ConstraintBody& entity,
        const Vec3& contact_point,
        const Vec3& terrain_normal,
        Real penetration_depth);

    // Convenience: update all terrain properties at once
    void update_from_terrain(const environment::TerrainQuery& query);

    // Configuration methods
    void set_terrain_normal(const Vec3& normal);
    void set_friction_coefficient(Real mu);
    void set_rolling_friction(Real mu);

    // Query methods
    bool is_active() const;
    Real get_normal_impulse() const;
    Real get_friction_impulse() const;
};
```

### Terrain-Normal Based Contact

Uses actual terrain surface normals instead of assuming vertical contact:

```cpp
// Contact normal comes from terrain, not always vertical
Vec3 terrain_normal = terrain_query.normal;  // e.g., slope normal for incline

contact.set_terrain_normal(terrain_normal);
// Constraint forces now align with slope, preventing sinking
```

**Benefits:**
- Works on slopes, rough terrain, and angled surfaces
- No special cases needed for inclined planes
- Automatic normal computation from terrain queries

### Coulomb Friction Model (Two Tangent Directions)

Friction constraints follow Coulomb's law with two independent tangent directions:

```cpp
void GroundContactConstraint::build_friction_rows(
    const EntityState* state,
    Real dt,
    ConstraintRow* rows) {

    // Friction limit from normal impulse (Coulomb cone)
    Real friction_limit = friction_coefficient * abs(normal_impulse_);

    // Two friction rows: tangent1 and tangent2
    // Each bounded by: |λ_tangent| ≤ μ * λ_normal
}
```

**Friction Formulation:**
- Normal constraint: λ_n ≥ 0 (contact can only push, not pull)
- Friction constraints: |λ_t| ≤ μ * λ_n (Coulomb friction cone)
- Two tangent directions computed from terrain normal

**Configuration per Material:**
```cpp
terrain_query material;  // From environment::TerrainQuery

// Material-specific friction automatically applied
contact.update_from_terrain(terrain_query);
// Now uses terrain_query.material.friction_coefficient
```

### Material-Specific Properties from Terrain Queries

Friction and rolling resistance automatically come from terrain:

```cpp
struct TerrainMaterial {
    Real friction_coefficient;      // From terrain database
    Real rolling_resistance;        // From terrain database
};

// During simulation update
environment::TerrainQuery query = terrain.query_at(entity_position);
if (query.valid) {
    contact.update_from_terrain(query);
    // Friction now matches terrain surface (mud, gravel, concrete, etc.)
}
```

**Typical Material Coefficients:**
| Material | Friction | Rolling Resistance |
|----------|----------|-------------------|
| Concrete | 0.9 | 0.005 |
| Asphalt | 0.8 | 0.01 |
| Gravel | 0.6 | 0.05 |
| Mud | 0.4 | 0.15 |
| Sand | 0.5 | 0.08 |

### Penetration and Baumgarte Stabilization

Position correction prevents sinking with configurable stabilization:

```cpp
Real penetration_depth = terrain_height - entity_position.z;

if (penetration_depth > 0) {  // Entity below terrain
    // Baumgarte factor controls correction strength
    Real erp = ground_params.stiffness;
    Real correction_bias = (erp * contact_depth_bias / dt) * penetration_depth;

    // Constraint solver uses bias to push entity out
}
```

**Slop (Allowed Penetration):**
```cpp
Real slop{0.005};  // Allow 5mm penetration before correction

// Reduces jitter from high-frequency terrain queries
if (error < slop) {
    error = 0;  // Don't correct tiny penetrations
}
```

### Restitution (Bouncing)

Configurable bounce behavior when hitting terrain:

```cpp
Real restitution{0.0};                  // 0 = no bounce
Real restitution_threshold{0.5};        // Min velocity to bounce (m/s)

// Only apply restitution if approaching fast enough
if (rel_vel_n < -restitution_threshold) {
    Real bounce_bias = restitution * abs(rel_vel_n);
    // Entity bounces back proportionally
}
```

**Physics:**
- Restitution = 0: Perfectly inelastic (mud, sand)
- Restitution = 0.5: Partial bounce (grass, dirt)
- Restitution = 1.0: Perfectly elastic (smooth ice)

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

## Parallel Force Computation (v0.6.0)

Entity force computation is fully parallelizable with work-stealing thread pool integration.

### Why Parallel Force Computation?

Force computation is often the most CPU-intensive part of physics simulation:
- Aerodynamic forces require complex calculations
- Terrain queries for every contact
- Multiple force generators per entity
- Scales poorly with entity count

Parallelization allows multi-core systems to compute forces in parallel.

### Thread-Safe Architecture

The physics system uses `std::shared_mutex` for thread-safe model access:

```cpp
class ComponentForceRegistry {
private:
    mutable std::shared_mutex mutex_;  // Multiple readers allowed
    std::unordered_map<ModelKey, std::unique_ptr<IForceGenerator>> models_;
};

// Read-only operations use shared lock (multiple threads allowed)
template<typename T>
T* ComponentForceRegistry::get(EntityId entity_id) const {
    std::shared_lock lock(mutex_);  // Non-exclusive lock
    auto it = models_.find(key);
    if (it != models_.end()) {
        return static_cast<T*>(it->second.get());
    }
    return nullptr;
}
```

### Force Accumulation Pattern

Thread-safe accumulation into entity force buffers:

```cpp
// Each entity has independent force accumulator
EntityForces& entity_forces = storage.forces(entity_index);

// Multiple threads can safely call add_force/add_torque
// (as long as no two threads write to same entity simultaneously)
forces.add_force(Vec3{1000, 0, 0});
forces.add_torque(Vec3{0, 500, 0});
```

**Important:** While reads from models are thread-safe, writes to entity forces must be synchronized if multiple force generators write to the same entity.

### Work-Stealing Thread Pool Integration

Force computation with work distribution:

```cpp
WorkStealingThreadPool pool(std::thread::hardware_concurrency());

// Phase 1: Clear all forces
entity_manager.get_state_storage().clear_forces();

// Phase 2: Submit force computations in parallel
std::vector<std::future<void>> tasks;

for (EntityId entity_id = 0; entity_id < num_entities; ++entity_id) {
    auto task = pool.enqueue([entity_id, &entity_manager, &registry, &env, dt]() {
        Entity* entity = entity_manager.get_entity(entity_id);
        if (!entity || !entity->active) return;

        EntityState state = entity_manager.get_state(entity_id);
        EntityForces& forces = entity_manager.get_state_storage()
                                   .forces(entity->state_index);

        // Get entity-specific force models
        if (auto* aero = registry.get<IAerodynamicsModel>(entity_id)) {
            aero->compute_forces(state, env, dt, forces);
        }
        if (auto* prop = registry.get<IPropulsionModel>(entity_id)) {
            prop->compute_forces(state, env, dt, forces);
        }
        if (auto* terrain = registry.get<ITerramechanicsModel>(entity_id)) {
            terrain->compute_forces(state, env, dt, forces);
        }
        // ... other force generators ...
    });

    tasks.push_back(std::move(task));
}

// Phase 3: Wait for all force computations to complete
pool.synchronize();

// Phase 4: Integrate with forces
physics_system.update(entity_manager, dt);
```

### Performance Characteristics

**Scalability Example (typical values):**
| Entities | Force Computation (Single-thread) | Parallel (8 cores) | Speedup |
|----------|-----------------------------------|-------------------|---------|
| 10 | 0.1 ms | 0.05 ms | 2.0x |
| 50 | 0.8 ms | 0.15 ms | 5.3x |
| 100 | 2.0 ms | 0.35 ms | 5.7x |
| 500 | 12 ms | 2.1 ms | 5.7x |

**Limiting Factors:**
- Synchronization overhead (negligible for moderate thread counts)
- Memory bandwidth for reading from registry
- Cache coherency for accessing environment data
- Per-entity computation time (very fine-grained work doesn't parallelize well)

### Best Practices for Parallel Force Computation

**DO:**
- Parallelize outer loop over entities
- Use ComponentForceRegistry for model lookup (thread-safe)
- Keep environment data (wind, terrain) read-only during physics update
- Let work-stealing scheduler handle load balancing

**DON'T:**
- Don't parallelize within single entity's force computation
- Don't modify entity state from parallel force computation
- Don't allocate memory per task (use thread-local buffers)
- Don't hold locks while computing forces (acquire, read, release)

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
