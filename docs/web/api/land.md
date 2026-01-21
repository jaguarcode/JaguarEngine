# Land Domain API Reference

Terramechanics, suspension, and tracked/wheeled vehicle APIs.

**Header:** `jaguar/domain/land.h`

## SoilProperties

Bekker-Wong soil parameters for terramechanics.

```cpp
struct SoilProperties {
    Real k_c{0.0};      // Cohesive modulus (kN/m^(n+1))
    Real k_phi{0.0};    // Frictional modulus (kN/m^(n+2))
    Real n{1.0};        // Deformation exponent
    Real c{0.0};        // Cohesion (kPa)
    Real phi{0.0};      // Internal friction angle (rad)
    Real K{0.0};        // Shear deformation modulus (m)

    // Preset soil types
    static SoilProperties DrySand();
    static SoilProperties WetSand();
    static SoilProperties Clay();
    static SoilProperties Snow();
    static SoilProperties Asphalt();
    static SoilProperties Mud();
    static SoilProperties Gravel();
};
```

### Preset Values

| Soil Type | k_c | k_phi | n | c (kPa) | φ (deg) |
|-----------|-----|-------|---|---------|---------|
| Dry Sand | 0.99 | 1528 | 1.10 | 1.04 | 28 |
| Wet Sand | 5.27 | 1515 | 0.73 | 1.72 | 29 |
| Clay | 13.19 | 692 | 0.50 | 4.14 | 13 |
| Snow | 4.37 | 196 | 1.60 | 1.03 | 19.7 |
| Asphalt | 10⁶ | 10⁶ | 0.0 | 1000 | 45 |

### Example

```cpp
// Use preset
auto soil = domain::land::SoilProperties::DrySand();

// Or customize
domain::land::SoilProperties custom_soil;
custom_soil.k_c = 5.0;
custom_soil.k_phi = 1000.0;
custom_soil.n = 0.9;
custom_soil.c = 2.0;
custom_soil.phi = 30.0 * DEG_TO_RAD;
```

## TerramechanicsModel

Bekker-Wong pressure-sinkage and shear-displacement model.

```cpp
class TerramechanicsModel : public physics::ITerramechanicsModel {
public:
    // Configuration
    void set_contact_area(Real width, Real length);  // m
    void set_vehicle_weight(Real weight_n);          // N
    void set_soil(const SoilProperties& soil);

    // IForceGenerator
    void compute_forces(const physics::EntityState& state,
                        const environment::Environment& env,
                        Real dt,
                        physics::EntityForces& forces) override;

    // ITerramechanicsModel queries
    Real get_sinkage() const override;            // m
    Real get_motion_resistance() const override;  // N
    Real get_traction() const override;           // N
    Real get_slip_ratio() const override;         // 0-1

    // Additional queries
    Real get_ground_pressure() const;   // Pa
    Real get_bearing_capacity() const;  // Pa
    bool is_mobility_limited() const;   // True if sinking
};
```

### Example

```cpp
domain::land::TerramechanicsModel terra;

// Configure for tank tracks
terra.set_contact_area(0.63, 4.6);  // Track: 63cm x 4.6m
terra.set_vehicle_weight(62000.0 * constants::G0);  // 62 tonnes

// Set soil (can change dynamically based on terrain)
auto soil = domain::land::SoilProperties::DrySand();
terra.set_soil(soil);

// In simulation loop
physics::EntityForces forces;
terra.compute_forces(state, env, dt, forces);

// Check mobility
Real sinkage = terra.get_sinkage();
if (sinkage > 0.3) {
    std::cout << "Warning: Vehicle stuck! Sinkage: " << sinkage << " m\n";
}

std::cout << "Motion resistance: " << terra.get_motion_resistance() << " N\n";
std::cout << "Available traction: " << terra.get_traction() << " N\n";
```

## SuspensionUnit

Individual suspension unit (road wheel, spring, damper).

```cpp
struct SuspensionUnit {
    Real spring_k{50000.0};      // Spring stiffness (N/m)
    Real damper_c{5000.0};       // Damping coefficient (N·s/m)
    Real preload{0.0};           // Preload force (N)
    Real travel_max{0.3};        // Maximum travel (m)
    Real travel_min{0.0};        // Minimum travel (m)
    Real current_position{0.0};  // Current compression (m)
    Real current_velocity{0.0};  // Compression rate (m/s)

    // Calculate spring-damper force
    Real calculate_force() const;

    // Update state
    void update(Real ground_height, Real wheel_height, Real dt);

    // Check limits
    bool is_bottomed_out() const;
    bool is_topped_out() const;
};
```

### Example

```cpp
domain::land::SuspensionUnit wheel;
wheel.spring_k = 300000.0;    // 300 kN/m (heavy vehicle)
wheel.damper_c = 30000.0;     // 30 kN·s/m
wheel.travel_max = 0.40;      // 40 cm travel
wheel.preload = 10000.0;      // 10 kN preload

// Update based on terrain
Real terrain_height = env.terrain.elevation;
Real wheel_height = vehicle_height - wheel_offset;
wheel.update(terrain_height, wheel_height, dt);

// Get force
Real suspension_force = wheel.calculate_force();
```

## SuspensionModel

Complete vehicle suspension system.

```cpp
class SuspensionModel {
public:
    // Add suspension units
    void add_unit(const Vec3& position, const SuspensionUnit& unit);
    void clear_units();

    // Update all units
    void update(const physics::EntityState& state,
                const environment::Environment& env,
                Real dt);

    // Get total forces and torques
    Vec3 get_total_force() const;
    Vec3 get_total_torque() const;

    // Query individual units
    SizeT unit_count() const;
    const SuspensionUnit& get_unit(SizeT index) const;
    Real get_average_compression() const;
};
```

### Example

```cpp
domain::land::SuspensionModel suspension;

// Configure suspension unit template
domain::land::SuspensionUnit wheel;
wheel.spring_k = 300000.0;
wheel.damper_c = 30000.0;
wheel.travel_max = 0.40;

// Add 7 road wheels per side (tank)
Real wheel_spacing = 0.8;  // meters
for (int i = 0; i < 7; ++i) {
    Real x = -3.0 + i * wheel_spacing;

    // Left side
    suspension.add_unit(Vec3{x, 1.5, -0.8}, wheel);

    // Right side
    suspension.add_unit(Vec3{x, -1.5, -0.8}, wheel);
}

// In simulation loop
suspension.update(state, env, dt);

physics::EntityForces forces;
forces.add_force(suspension.get_total_force());
forces.add_torque(suspension.get_total_torque());
```

## TrackedVehicleModel

Track system dynamics.

```cpp
class TrackedVehicleModel {
public:
    struct TrackState {
        Real tension{10000.0};  // Track tension (N)
        Real velocity{0.0};     // Track linear velocity (m/s)
        Real slip{0.0};         // Track slip ratio
    };

    // Configuration
    void set_sprocket_radius(Real radius);  // m
    void set_sprocket_max_torque(Real torque);  // N·m
    void set_track_width(Real width);  // m
    void set_track_length(Real length);  // m
    void set_idler_radius(Real radius);  // m

    // Update tracks
    void update(Real left_torque, Real right_torque,
                Real load, const SoilProperties& soil, Real dt);

    // Queries
    const TrackState& get_left_track() const;
    const TrackState& get_right_track() const;
    Real get_propulsive_force() const;
    Real get_steering_moment() const;

    // Steering (differential)
    void set_steering(Real steer);  // -1 to +1 (left to right)
};
```

### Example

```cpp
domain::land::TrackedVehicleModel tracks;

// Configure
tracks.set_sprocket_radius(0.33);
tracks.set_sprocket_max_torque(100000.0);  // 100 kN·m
tracks.set_track_width(0.63);
tracks.set_track_length(4.6);

// In simulation loop
Real engine_torque = throttle * max_engine_torque;

// Apply steering (differential)
tracks.set_steering(steering_input);

// Update
auto soil = domain::land::SoilProperties::DrySand();
tracks.update(engine_torque, engine_torque, vehicle_weight, soil, dt);

// Get forces
Real propulsion = tracks.get_propulsive_force();
Real steering_moment = tracks.get_steering_moment();

physics::EntityForces forces;
Vec3 thrust_body{propulsion, 0.0, 0.0};
forces.add_force(state.orientation.rotate(thrust_body));
forces.add_torque(Vec3{0.0, 0.0, steering_moment});
```

## WheeledVehicleModel

Wheeled vehicle drivetrain and tire model.

```cpp
class WheeledVehicleModel {
public:
    struct WheelState {
        Real angular_velocity{0.0};  // rad/s
        Real slip_ratio{0.0};
        Real slip_angle{0.0};        // rad
        Real normal_load{0.0};       // N
        Vec3 force{0, 0, 0};         // Tire force
    };

    // Configuration
    void set_wheel_radius(Real radius);  // m
    void set_wheel_inertia(Real inertia);  // kg·m²

    // Add wheels (position relative to CG)
    void add_wheel(const Vec3& position, bool is_driven, bool is_steered);

    // Control
    void set_drive_torque(Real torque);  // N·m (to driven wheels)
    void set_brake_torque(Real torque);  // N·m
    void set_steering_angle(Real angle);  // rad

    // Update
    void update(const physics::EntityState& state,
                const SoilProperties& soil, Real dt);

    // Queries
    Vec3 get_total_force() const;
    Vec3 get_total_torque() const;
    const WheelState& get_wheel(SizeT index) const;
    SizeT wheel_count() const;
};
```

### Example

```cpp
domain::land::WheeledVehicleModel wheels;

// Configure wheels
wheels.set_wheel_radius(0.4);
wheels.set_wheel_inertia(5.0);

// 4x4 vehicle
wheels.add_wheel(Vec3{2.0, 1.0, -0.4}, true, true);   // FL: driven, steered
wheels.add_wheel(Vec3{2.0, -1.0, -0.4}, true, true);  // FR: driven, steered
wheels.add_wheel(Vec3{-2.0, 1.0, -0.4}, true, false); // RL: driven
wheels.add_wheel(Vec3{-2.0, -1.0, -0.4}, true, false); // RR: driven

// Control
wheels.set_drive_torque(throttle * max_torque);
wheels.set_brake_torque(brake * max_brake);
wheels.set_steering_angle(steer * max_steer_rad);

// Update
auto soil = domain::land::SoilProperties::Asphalt();
wheels.update(state, soil, dt);

// Apply forces
physics::EntityForces forces;
forces.add_force(wheels.get_total_force());
forces.add_torque(wheels.get_total_torque());
```

## Terrain Integration

```cpp
// Get terrain properties at vehicle location
auto env = engine.get_environment(vehicle_id);

// Terrain data available
Real elevation = env.terrain.elevation;
Vec3 normal = env.terrain.normal;
Real slope = env.terrain.slope_angle;

// Get soil type based on terrain material
SoilProperties soil;
switch (env.terrain.material.type) {
    case MaterialType::Sand:
        soil = SoilProperties::DrySand();
        break;
    case MaterialType::Clay:
        soil = SoilProperties::Clay();
        break;
    case MaterialType::Road:
        soil = SoilProperties::Asphalt();
        break;
    default:
        soil = SoilProperties::DrySand();
}

terra.set_soil(soil);
```

## See Also

- [Physics API](physics.md) - Force generator interfaces
- [Environment API](environment.md) - Terrain system
- [Land Domain Concepts](../domains/land.md) - Domain overview
- [Land Domain Tutorial](../tutorials/land-domain.md) - Step-by-step guide
- [Terrain Tutorial](../tutorials/terrain.md) - Working with terrain

