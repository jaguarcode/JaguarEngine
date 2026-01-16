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

### SuspensionModel

Complete vehicle suspension system:

```cpp
class SuspensionModel {
public:
    // Add suspension unit at body-frame position
    void add_unit(const Vec3& position, const SuspensionUnit& unit);

    // Update all units based on vehicle motion
    void update(const physics::EntityState& state, Real dt);

    // Get total force and torque from suspension
    Vec3 get_total_force() const;
    Vec3 get_total_torque() const;

    // Query
    SizeT unit_count() const;
};
```

**Usage:**
```cpp
SuspensionModel suspension;

// Configure 4-wheel vehicle
SuspensionUnit front, rear;
front.spring_k = 60000.0;
rear.spring_k = 70000.0;

// Add at body-frame positions
suspension.add_unit(Vec3{2.0, 1.0, -0.5}, front);   // Front left
suspension.add_unit(Vec3{2.0, -1.0, -0.5}, front);  // Front right
suspension.add_unit(Vec3{-2.0, 1.0, -0.5}, rear);   // Rear left
suspension.add_unit(Vec3{-2.0, -1.0, -0.5}, rear);  // Rear right

// In simulation loop
suspension.update(state, dt);

Vec3 susp_force = suspension.get_total_force();
Vec3 susp_torque = suspension.get_total_torque();

forces.add_force(susp_force);
forces.add_torque(susp_torque);
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

## Complete Ground Vehicle Example

```cpp
#include <jaguar/jaguar.h>

class Tank {
public:
    Tank(const std::string& name) {
        entity_id_ = entities_.create_entity(name, Domain::Land);

        // Configure terramechanics
        terra_.set_contact_area(0.5, 4.0);      // Track dimensions
        terra_.set_vehicle_weight(500000.0);   // 50 tonnes

        // Configure tracks
        tracks_.set_sprocket(0.35, 80000.0);

        // Configure suspension (6 road wheels per side)
        SuspensionUnit wheel;
        wheel.spring_k = 200000.0;  // 200 kN/m
        wheel.damper_c = 20000.0;   // 20 kN·s/m
        wheel.travel_max = 0.35;

        // Left side (x positions, y = 1.5m)
        for (Real x = -3.0; x <= 2.0; x += 1.0) {
            suspension_.add_unit(Vec3{x, 1.5, -0.8}, wheel);
        }
        // Right side (y = -1.5m)
        for (Real x = -3.0; x <= 2.0; x += 1.0) {
            suspension_.add_unit(Vec3{x, -1.5, -0.8}, wheel);
        }
    }

    void set_throttle(Real throttle) { throttle_ = throttle; }

    void update(Real dt) {
        auto state = entities_.get_state(entity_id_);

        environment::Environment env;
        // ... populate environment ...

        forces_.clear();

        // Terramechanics forces
        terra_.compute_forces(state, env, dt, forces_);

        // Track dynamics
        Real drive_torque = throttle_ * 80000.0;  // Max 80 kN·m
        SoilProperties soil = SoilProperties::DrySand();
        tracks_.update(drive_torque, state.mass * 9.81, soil, dt);

        // Suspension forces
        suspension_.update(state, dt);
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

## Coordinate Conventions

### Body-Axis System
- **X**: Forward (vehicle front)
- **Y**: Right (passenger side)
- **Z**: Down (into ground)

### Forces
- Traction: Positive X (forward acceleration)
- Resistance: Negative X (opposes motion)
- Normal: Positive Z (ground reaction upward in NED → force on vehicle is downward, but reaction is upward)

## Terrain Integration

The terramechanics model queries the environment for:
- `env.altitude` - Vehicle altitude (m)
- `env.terrain_elevation` - Ground elevation at vehicle position (m)

Height above terrain:
```cpp
Real height = env.altitude - env.terrain_elevation;
if (height < 0.1) {
    // Vehicle is on ground, compute terra forces
}
```

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
