# Land Domain Documentation

The Land domain module implements terramechanics, suspension systems, and tracked vehicle dynamics for ground vehicles.

## Headers

| Header | Purpose |
|--------|---------|
| `jaguar/domain/land.h` | Terramechanics, suspension, tracked vehicles |

## Components

### SoilProperties

Bekker-Wong soil parameters for terrain characterization:

```cpp
struct SoilProperties {
    Real k_c{0.0};      // Cohesive modulus (kN/m^(n+1))
    Real k_phi{0.0};    // Frictional modulus (kN/m^(n+2))
    Real n{1.0};        // Deformation exponent
    Real c{0.0};        // Cohesion (kPa)
    Real phi{0.0};      // Internal friction angle (rad)

    // Preset soil types
    static SoilProperties DrySand();
    static SoilProperties WetSand();
    static SoilProperties Clay();
    static SoilProperties Snow();
    static SoilProperties Asphalt();  // Rigid surface
};
```

**Preset Values:**

| Soil Type | k_c | k_phi | n | c (kPa) | φ (deg) |
|-----------|-----|-------|---|---------|---------|
| Dry Sand | 0.99 | 1528 | 1.10 | 1.04 | 28° |
| Wet Sand | 5.27 | 1515 | 0.73 | 1.72 | 29° |
| Clay | 13.19 | 692 | 0.50 | 4.14 | 13° |
| Snow | 4.37 | 196 | 1.60 | 1.03 | 19.7° |
| Asphalt | 10⁶ | 10⁶ | 0 | 1000 | 45° |

**Usage:**
```cpp
// Use preset soil type
SoilProperties sand = SoilProperties::DrySand();

// Create custom soil
SoilProperties custom;
custom.k_c = 8.0;
custom.k_phi = 1200.0;
custom.n = 0.9;
custom.c = 2.5;
custom.phi = 25.0 * constants::DEG_TO_RAD;
```

### TerramechanicsModel

Bekker-Wong terramechanics force generator implementing `ITerramechanicsModel`:

```cpp
class TerramechanicsModel : public physics::ITerramechanicsModel {
public:
    // Configuration
    void set_contact_area(Real width, Real length);  // Track/tire dimensions
    void set_vehicle_weight(Real weight_n);          // Total weight (N)

    // State queries
    Real get_sinkage() const;           // Current sinkage (m)
    Real get_motion_resistance() const; // Compaction resistance (N)
    Real get_traction() const;          // Available traction (N)
    Real get_slip_ratio() const;        // Current slip ratio
};
```

**Bekker-Wong Theory:**

The pressure-sinkage relationship:
```
p = (k_c/b + k_phi) × z^n
```
Where:
- p = ground pressure (kPa)
- k_c = cohesive modulus
- k_phi = frictional modulus
- b = contact width (m)
- z = sinkage (m)
- n = deformation exponent

Solving for sinkage:
```cpp
z = (p / (k_c/b + k_phi))^(1/n)
```

**Motion Resistance:**

Compaction resistance from soil deformation:
```
R_c = b × (k_c/b + k_phi) × z^(n+1) / (n+1)
```

**Traction Model (Mohr-Coulomb):**

Maximum shear stress:
```
τ_max = c + σ × tan(φ)
```

Maximum traction force:
```
T_max = A × τ_max
```

Where A = contact area.

**Slip-Traction Relationship:**

Exponential slip model:
```cpp
T = T_max × (1 - exp(-k × |slip|))
```
Where k ≈ 10 for typical soils.

At high slip (>50%), traction reduces due to soil shearing:
```cpp
if (slip > 0.5) {
    T *= (1 - 0.3 × (slip - 0.5) / 0.5)
}
```

**Usage:**
```cpp
TerramechanicsModel terra;

// Configure for tank track
terra.set_contact_area(0.5, 4.0);    // 50cm wide, 4m long track
terra.set_vehicle_weight(500000.0);  // 50 tonnes

// In simulation loop
physics::EntityForces forces;
forces.clear();
terra.compute_forces(state, env, dt, forces);

// Check mobility
Real sinkage = terra.get_sinkage();
if (sinkage > 0.3) {
    // Vehicle may be stuck
}

Real traction = terra.get_traction();
Real resistance = terra.get_motion_resistance();
Real net_force = traction - resistance;
```

### SuspensionUnit

Single spring-damper suspension element:

```cpp
struct SuspensionUnit {
    Real spring_k{50000.0};     // Spring stiffness (N/m)
    Real damper_c{5000.0};      // Damping coefficient (N·s/m)
    Real preload{0.0};          // Preload force (N)
    Real travel_max{0.3};       // Maximum travel (m)
    Real travel_min{0.0};       // Minimum travel (m)

    // State
    Real current_position{0.0}; // Current compression (m)
    Real current_velocity{0.0}; // Compression rate (m/s)

    Real calculate_force() const;
};
```

**Spring-Damper Force Model:**
```
F = k × x + c × v + preload
```
Where:
- k = spring stiffness (N/m)
- c = damping coefficient (N·s/m)
- x = compression (m, positive = compressed)
- v = compression rate (m/s, positive = compressing)

**Bump Stop Modeling:**

Quadratic bump stop at travel limits:
```cpp
// Near full extension (travel_min + 2cm)
if (position < travel_min + 0.02) {
    Real penetration = (travel_min + 0.02) - position;
    F -= 100000 × penetration² / 0.02;
}

// Near full compression (travel_max - 2cm)
if (position > travel_max - 0.02) {
    Real penetration = position - (travel_max - 0.02);
    F += 100000 × penetration² / 0.02;
}
```

**Usage:**
```cpp
SuspensionUnit wheel;
wheel.spring_k = 60000.0;   // 60 kN/m
wheel.damper_c = 4000.0;    // 4 kN·s/m
wheel.preload = 2000.0;     // 2 kN preload
wheel.travel_max = 0.25;    // 25cm travel
wheel.travel_min = 0.0;

// Update state (from terrain contact)
wheel.current_position = 0.1;   // 10cm compressed
wheel.current_velocity = 0.2;   // Compressing at 0.2 m/s

Real force = wheel.calculate_force();
// force = 60000*0.1 + 4000*0.2 + 2000 = 8800 N
```

### WheelSuspension (Terrain-Integrated)

Individual wheel suspension with full terrain integration (v0.6.0+):

```cpp
class WheelSuspension {
public:
    /**
     * Create wheel suspension at body-relative position
     * @param position Position in body frame (m)
     * @param wheel_radius Wheel radius (m)
     * @param unit Suspension unit parameters
     */
    WheelSuspension(const Vec3& position, Real wheel_radius, const SuspensionUnit& unit);

    /**
     * Compute suspension and contact forces
     * @param state Entity state (position, orientation, velocity)
     * @param env_service Environment service for terrain queries (can be nullptr)
     * @param dt Time step (s)
     * @return Force in world frame
     */
    Vec3 compute_forces(const physics::EntityState& state,
                       const environment::EnvironmentService* env_service,
                       Real dt);

    // Torque contribution about body center of mass
    Vec3 compute_torque(const physics::EntityState& state) const;

    // State queries
    Real get_compression() const;           // Current compression (m)
    Real get_contact_force() const;         // Force magnitude (N)
    bool is_grounded() const;               // Contact status
    Real get_friction_coefficient() const;  // From terrain material
};
```

**Key Features:**

- **Terrain-Aware Contact**: Queries terrain elevation and surface normal at wheel position
- **Surface Normal Forces**: Forces applied along terrain normal, not vertical (supports slopes and banking)
- **Material Friction**: Friction coefficient from terrain material (0.15 ice to 0.9 asphalt)
- **Slope Support**: Correctly handles hills, valleys, and uneven terrain
- **Graceful Degradation**: Falls back to flat ground if no environment service provided

**Force Calculation Pipeline:**

```
1. Query terrain at wheel position via EnvironmentService
   - Gets elevation, surface normal, material properties

2. Transform wheel position to world frame
   - wheel_pos_world = vehicle_pos + vehicle_orientation * body_frame_pos

3. Compute penetration relative to terrain
   - penetration = wheel_radius - distance_to_surface
   - distance measured perpendicular to terrain normal

4. Update suspension state
   - compression = clamp(penetration, travel_min, travel_max)
   - velocity = -wheel_velocity · terrain_normal

5. Calculate force magnitude
   - force = spring_force + damping_force + bump_stops

6. Apply force along terrain normal
   - force_world = terrain_normal * force_magnitude
```

**Terrain Integration Example:**

```cpp
// Create wheel with terrain awareness
SuspensionUnit unit;
unit.spring_k = 60000.0;   // N/m
unit.damper_c = 5000.0;    // N·s/m
unit.travel_max = 0.3;     // m

WheelSuspension wheel(Vec3{2.0, 1.0, -0.4}, 0.35, unit);

// In simulation loop
environment::EnvironmentService env_service;  // Populated with terrain data
auto state = entity.get_state();

Vec3 force = wheel.compute_forces(state, &env_service, dt);
Vec3 torque = wheel.compute_torque(state);
```

**Sloped Terrain Behavior:**

On a 30° hill, the suspension force is not vertical but follows the terrain normal:

```
Normal terrain (flat):      Normal terrain (30° slope):
force = (0, 0, +F)         force = (-sin(30°)*F, 0, cos(30°)*F)
                                  = (-0.5*F, 0, 0.866*F)
```

This correctly distributes forces perpendicular to the surface, improving realism for wheeled vehicles on rough terrain.

### SuspensionModel (Multi-Wheel Vehicle)

Complete vehicle suspension system with multiple wheels:

```cpp
class SuspensionModel {
public:
    /**
     * Add a wheel suspension unit at body-relative position
     */
    void add_wheel(const Vec3& position, Real wheel_radius, const SuspensionUnit& unit);

    /**
     * Update all wheels and compute total forces/torques
     * @param state Entity state
     * @param env_service Environment service (nullptr for flat terrain fallback)
     * @param dt Time step
     */
    void update(const physics::EntityState& state,
               const environment::EnvironmentService* env_service,
               Real dt);

    // Force and torque aggregation
    Vec3 get_total_force() const;   // Sum of all wheel forces (world frame)
    Vec3 get_total_torque() const;  // Sum of all wheel torques (world frame)

    // Queries
    SizeT wheel_count() const;                    // Total number of wheels
    const WheelSuspension& wheel(SizeT i) const; // Access wheel i
    SizeT grounded_wheel_count() const;           // Wheels in contact
};
```

**Force Aggregation:**

The model computes total force and torque by summing contributions from all wheels:

```
F_total = Σ F_wheel_i
τ_total = Σ (r_i × F_wheel_i)

where:
  F_wheel_i = force from wheel i (in world frame)
  r_i = position of wheel i relative to vehicle CoM (in world frame)
  × = cross product
```

**Usage:**
```cpp
SuspensionModel suspension;

// Configure 4-wheel vehicle
SuspensionUnit front, rear;
front.spring_k = 60000.0;
rear.spring_k = 70000.0;

// Add wheels at body-frame positions
suspension.add_wheel(Vec3{2.0, 1.0, -0.4}, 0.35, front);    // Front left
suspension.add_wheel(Vec3{2.0, -1.0, -0.4}, 0.35, front);   // Front right
suspension.add_wheel(Vec3{-2.0, 1.0, -0.4}, 0.35, rear);    // Rear left
suspension.add_wheel(Vec3{-2.0, -1.0, -0.4}, 0.35, rear);   // Rear right

// In simulation loop
environment::EnvironmentService env;
auto state = entity.get_state();

suspension.update(state, &env, dt);

Vec3 susp_force = suspension.get_total_force();
Vec3 susp_torque = suspension.get_total_torque();

forces.add_force(susp_force);
forces.add_torque(susp_torque);

// Monitor grounded wheels
SizeT grounded = suspension.grounded_wheel_count();
if (grounded == 0) {
    // Vehicle is airborne
}
```

**Multi-Wheel Terrain Effects:**

When driving over uneven terrain, different wheels experience different compressions and forces:

```
Vehicle on slope:
        Front wheels (higher):        Rear wheels (lower):
        lower compression             higher compression

Result: Vehicle pitches to follow terrain contour
Torque: τ = r_front × F_front + r_rear × F_rear
        (naturally models nose-down on downhill slopes)
```

### TrackedVehicleModel

Simplified tracked vehicle dynamics:

```cpp
class TrackedVehicleModel {
public:
    struct TrackState {
        Real tension{10000.0};  // Track tension (N)
        Real velocity{0.0};     // Track linear velocity (m/s)
        Real slip{0.0};         // Track slip ratio
    };

    // Configuration
    void set_sprocket(Real radius, Real max_torque);

    // Update track dynamics
    void update(Real drive_torque, Real load, const SoilProperties& soil, Real dt);

    // State queries
    const TrackState& get_left_track() const;
    const TrackState& get_right_track() const;
    Real get_propulsive_force() const;
};
```

**Track Dynamics:**

Drive force from torque:
```
F_drive = T_drive / r_sprocket
```

Slip-based traction model:
```cpp
if (traction_ratio < 0.8) {
    slip = 0.1 × traction_ratio;      // Linear region
} else {
    slip = 0.08 + 0.5 × (ratio - 0.8); // High-slip region
}
```

Propulsive force with efficiency:
```
F_prop = T_track × (1 - slip) × η
```
Where η ≈ 0.85 (track efficiency).

**Usage:**
```cpp
TrackedVehicleModel tracks;

// Configure sprocket
tracks.set_sprocket(0.35, 80000.0);  // 35cm radius, 80 kN·m max

// Get soil at current location
SoilProperties soil = SoilProperties::Clay();

// Update dynamics
Real engine_torque = 50000.0;  // N·m
Real vehicle_weight = 500000.0; // N
tracks.update(engine_torque, vehicle_weight, soil, dt);

// Check track states
auto& left = tracks.get_left_track();
if (left.slip > 0.3) {
    // High slip - reduce throttle
}

Real propulsion = tracks.get_propulsive_force();
```

## Complete Ground Vehicle Examples

### Tracked Vehicle with Terrain-Integrated Suspension

```cpp
#include <jaguar/jaguar.h>

class Tank {
public:
    Tank(const std::string& name) {
        entity_id_ = entities_.create_entity(name, Domain::Land);

        // Configure terramechanics
        terra_.set_contact_area(0.5, 4.0);      // Track dimensions (50cm wide, 4m long)
        terra_.set_vehicle_weight(500000.0);   // 50 tonnes

        // Configure tracks
        tracks_.set_sprocket(0.35, 80000.0);   // 35cm radius, 80 kN·m max

        // Configure suspension (6 road wheels per side)
        SuspensionUnit wheel;
        wheel.spring_k = 200000.0;  // 200 kN/m
        wheel.damper_c = 20000.0;   // 20 kN·s/m
        wheel.travel_max = 0.35;    // 35cm travel

        // Left side (x positions, y = 1.5m)
        for (Real x = -3.0; x <= 2.0; x += 1.0) {
            suspension_.add_wheel(Vec3{x, 1.5, -0.8}, 0.4, wheel);
        }
        // Right side (y = -1.5m)
        for (Real x = -3.0; x <= 2.0; x += 1.0) {
            suspension_.add_wheel(Vec3{x, -1.5, -0.8}, 0.4, wheel);
        }
    }

    void set_throttle(Real throttle) { throttle_ = throttle; }

    void update(Real dt) {
        auto state = entities_.get_state(entity_id_);

        // Get environment with terrain data
        environment::EnvironmentService& env = engine_.environment();

        forces_.clear();

        // Terramechanics forces (for track-terrain interaction)
        terra_.compute_forces(state, env, dt, forces_);

        // Track dynamics
        Real drive_torque = throttle_ * 80000.0;  // Max 80 kN·m
        SoilProperties soil = SoilProperties::DrySand();
        tracks_.update(drive_torque, state.mass * 9.81, soil, dt);

        // Suspension forces with TERRAIN INTEGRATION (v0.6.0+)
        // Each wheel queries terrain for elevation, normal, and material properties
        suspension_.update(state, &env, dt);
        forces_.add_force(suspension_.get_total_force());
        forces_.add_torque(suspension_.get_total_torque());

        // Gravity
        Vec3 gravity{0.0, 0.0, state.mass * 9.81};
        forces_.add_force(gravity);

        // ... integrate state ...
    }

    // Telemetry
    Real get_sinkage() const { return terra_.get_sinkage(); }
    Real get_slip() const { return tracks_.get_left_track().slip; }
    SizeT get_grounded_wheels() const { return suspension_.grounded_wheel_count(); }

private:
    physics::EntityManager entities_;
    EntityId entity_id_;

    TerramechanicsModel terra_;
    TrackedVehicleModel tracks_;
    SuspensionModel suspension_;
    physics::EntityForces forces_;

    Real throttle_{0.0};
};
```

### Wheeled Vehicle on Hilly Terrain

This example demonstrates terrain-aware suspension handling slopes:

```cpp
class HumveeOnHillyTerrain {
public:
    HumveeOnHillyTerrain() {
        // 4-wheel vehicle with 2.5m wheelbase, 1.5m track
        SuspensionUnit unit;
        unit.spring_k = 40000.0;
        unit.damper_c = 3000.0;
        unit.travel_max = 0.25;

        Real wheelbase = 2.5;
        Real track = 1.5;

        suspension_.add_wheel(Vec3{wheelbase/2, track/2, -0.4}, 0.35, unit);   // FR
        suspension_.add_wheel(Vec3{wheelbase/2, -track/2, -0.4}, 0.35, unit);  // FL
        suspension_.add_wheel(Vec3{-wheelbase/2, track/2, -0.4}, 0.35, unit);  // RR
        suspension_.add_wheel(Vec3{-wheelbase/2, -track/2, -0.4}, 0.35, unit); // RL
    }

    void update(const physics::EntityState& state,
               environment::EnvironmentService& env,
               Real dt) {
        // Update suspension with terrain queries
        suspension_.update(state, &env, dt);

        // Get forces
        Vec3 total_force = suspension_.get_total_force();
        Vec3 total_torque = suspension_.get_total_torque();

        // Monitor individual wheel loads for traction control
        for (size_t i = 0; i < suspension_.wheel_count(); ++i) {
            const auto& wheel = suspension_.wheel(i);

            if (wheel.is_grounded()) {
                // Wheel load determines available traction
                Real load = wheel.get_contact_force();
                Real friction = wheel.get_friction_coefficient();
                Real max_traction = load * friction;

                // Use for slip control, torque vectoring, etc.
                traction_available_[i] = max_traction;
            }
        }
    }

    // Traction control feedback
    std::array<Real, 4> get_traction_available() const {
        return traction_available_;
    }

private:
    SuspensionModel suspension_;
    std::array<Real, 4> traction_available_{};
};
```

### Terrain Response Comparison

**Flat terrain (EnvironmentService with flat terrain):**
```
All wheels compress equally → balanced vertical forces
No pitch/roll torque → vehicle maintains level attitude
```

**Hilly terrain (EnvironmentService with slope data):**
```
Front wheels on upslope:  lower compression, lower force
Rear wheels on downslope: higher compression, higher force
Suspension automatically creates pitch moment
Vehicle tilts naturally to follow slope
```

**Without terrain data (nullptr EnvironmentService):**
```
Suspensions fall back to flat ground model
No slope effects → less realistic on rough terrain
Still maintains physics correctness
```

## Coordinate Conventions

### Body-Axis System
- **X**: Forward (vehicle front)
- **Y**: Right (passenger side)
- **Z**: Down (into ground)

### Forces
- Traction: Positive X (forward acceleration)
- Resistance: Negative X (opposes motion)
- Normal: Positive Z (ground reaction upward in NED → force on vehicle is downward, but reaction is upward)

## Terrain Integration with EnvironmentService

Suspension systems integrate with `environment::EnvironmentService` for realistic terrain interaction (v0.6.0+).

### TerrainQuery Structure

Each terrain query returns elevation, surface properties, and geometry:

```cpp
struct TerrainQuery {
    Real elevation{0.0};           // Height above reference (m)
    Vec3 normal{0, 0, 1};          // Surface normal (world frame)
    Real slope_angle{0.0};         // Terrain slope (rad)
    TerrainMaterial material;      // Surface properties
    bool valid{false};             // Query success
};

struct TerrainMaterial {
    SurfaceType type;
    Real friction_coefficient;     // Typical range: 0.15-0.9
    Real rolling_resistance;       // Typical range: 0.01-0.05
    // Bekker-Wong parameters for soft-soil mobility
    Real k_c, k_phi, n;
};
```

### Integration Pattern

The suspension model queries terrain automatically:

```cpp
// 1. Wheel suspension queries terrain at its position
Vec3 wheel_pos_world = vehicle_state.position +
                       vehicle_state.orientation.rotate(body_frame_pos);

Environment env = env_service->query(wheel_pos_world, current_time);
TerrainQuery terrain = env.terrain;

// 2. Extract contact properties
Vec3 surface_normal = terrain.normal;
Real friction_mu = terrain.material.friction_coefficient;
Real elevation = terrain.elevation;

// 3. Compute penetration along surface normal
Real distance_to_surface = (wheel_pos_world - surface_point).dot(normal);
Real penetration = wheel_radius - distance_to_surface;

// 4. Apply force along surface normal (not vertical!)
Vec3 force = normal * force_magnitude;
```

### Fallback Behavior

If no environment service is provided, suspension gracefully defaults to flat terrain:

```cpp
// Without environment service
Vec3 force = wheel.compute_forces(state, nullptr, dt);

// Defaults to:
//   - Flat horizontal surface
//   - Surface normal = (0, 0, 1)
//   - Friction coefficient = 0.7
//   - No slope effects
```

This maintains backward compatibility with simulations that don't use terrain data.

### Complete Terrain-Aware Example

```cpp
#include <jaguar/jaguar.h>

class TerrainAwareVehicle {
public:
    void initialize() {
        // Setup suspension with terrain awareness
        SuspensionUnit wheel_unit;
        wheel_unit.spring_k = 50000.0;
        wheel_unit.damper_c = 5000.0;
        wheel_unit.travel_max = 0.3;

        // 4-wheel passenger vehicle
        suspension_.add_wheel(Vec3{1.2, 0.8, -0.4}, 0.33, wheel_unit);   // FL
        suspension_.add_wheel(Vec3{1.2, -0.8, -0.4}, 0.33, wheel_unit);  // FR
        suspension_.add_wheel(Vec3{-1.3, 0.8, -0.4}, 0.33, wheel_unit);  // RL
        suspension_.add_wheel(Vec3{-1.3, -0.8, -0.4}, 0.33, wheel_unit); // RR
    }

    void update(const physics::EntityState& state, Real dt) {
        // Get environment and terrain data at vehicle location
        environment::EnvironmentService& env = engine.environment();

        // Update suspension with terrain integration
        suspension_.update(state, &env, dt);

        // Get forces from all wheels
        Vec3 susp_force = suspension_.get_total_force();
        Vec3 susp_torque = suspension_.get_total_torque();

        // Apply to vehicle dynamics
        forces_.clear();
        forces_.add_force(susp_force);
        forces_.add_torque(susp_torque);

        // Log terrain interaction
        for (size_t i = 0; i < suspension_.wheel_count(); ++i) {
            const auto& wheel = suspension_.wheel(i);
            if (wheel.is_grounded()) {
                Real friction = wheel.get_friction_coefficient();
                Real compression = wheel.get_compression();
                // Use for traction/braking control
            }
        }
    }

    SuspensionModel suspension_;
    physics::EntityForces forces_;
};
```

### Advanced: Slope and Banking Effects

Suspension correctly handles non-horizontal terrain:

```
Driving uphill (30° slope):
┌─────────────────── Vehicle ──────────────────┐
│  Front wheel: less penetration, lower force  │
│  Rear wheel: more penetration, higher force  │
└─────────────────────────────────────────────┘
         ╱╱  Terrain surface normal points up-slope

Forces applied perpendicular to slope, naturally
creating pitching moment that keeps vehicle aligned
with terrain
```

For banked turns or sidehills, similar behavior applies in the lateral direction, supporting realistic vehicle dynamics on complex terrain.

## Performance Considerations

### Typical Parameters

| Vehicle Type | Weight (tonnes) | Track Width (m) | Track Length (m) |
|-------------|-----------------|-----------------|------------------|
| Light APC | 15 | 0.4 | 3.0 |
| Main Battle Tank | 50-70 | 0.5-0.6 | 4.0-5.0 |
| Heavy Tank | 70+ | 0.6 | 5.0 |

### Numerical Stability
- Sinkage limited to 0.5m maximum
- Contact dimensions clamped to minimum 1cm
- Slip ratio bounded to [-1, 1]

### Integration Considerations
- 50-100 Hz typical for ground vehicles
- Higher rates needed for rough terrain simulation
- RK4 provides good stability

## References

- **Terramechanics**: Wong - "Theory of Ground Vehicles"
- **Bekker Theory**: Bekker - "Introduction to Terrain-Vehicle Systems"
- **Suspension**: Jazar - "Vehicle Dynamics: Theory and Application"
- **Tracked Vehicles**: Ogorkiewicz - "Technology of Tanks"
