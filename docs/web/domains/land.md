# Land Domain

The Land domain module implements terramechanics, suspension systems, and tracked/wheeled vehicle dynamics for ground vehicles.

## Overview

JaguarEngine's Land domain provides:

- **Bekker-Wong Terramechanics**: Soil-vehicle interaction modeling
- **Suspension Dynamics**: Spring-damper systems with bump stops
- **Tracked Vehicle Models**: Continuous track approximation
- **Terrain Integration**: DTED/SRTM terrain queries

## Terramechanics Model

### Bekker-Wong Theory

The pressure-sinkage relationship:

```
p = (k_c/b + k_φ) × z^n
```

Where:
- p = ground pressure (Pa)
- k_c = cohesive modulus (kN/m^(n+1))
- k_φ = frictional modulus (kN/m^(n+2))
- b = contact width (m)
- z = sinkage (m)
- n = deformation exponent

### Soil Properties

| Soil Type | k_c | k_φ | n | c (kPa) | φ (deg) |
|-----------|-----|-----|---|---------|---------|
| Dry Sand | 0.99 | 1528 | 1.10 | 1.04 | 28° |
| Wet Sand | 5.27 | 1515 | 0.73 | 1.72 | 29° |
| Clay | 13.19 | 692 | 0.50 | 4.14 | 13° |
| Snow | 4.37 | 196 | 1.60 | 1.03 | 19.7° |
| Asphalt | 10⁶ | 10⁶ | 0 | 1000 | 45° |

### Motion Resistance

Compaction resistance from soil deformation:

```
R_c = b × (k_c/b + k_φ) × z^(n+1) / (n+1)
```

### Traction Model (Mohr-Coulomb)

Maximum shear stress:
```
τ_max = c + σ × tan(φ)
```

Maximum traction force:
```
T_max = A × τ_max
```

## Example: Creating a Tank

```cpp
#include <jaguar/jaguar.h>

using namespace jaguar;

int main() {
    Engine engine;
    engine.initialize();

    // Create ground vehicle entity
    EntityId tank = engine.create_entity("M1A2", Domain::Land);

    // Configure initial state
    physics::EntityState state;
    state.position = Vec3{0.0, 0.0, 0.0};
    state.velocity = Vec3{0.0, 0.0, 0.0};
    state.mass = 62000.0;  // 62 tonnes
    engine.set_entity_state(tank, state);

    // Configure terramechanics
    domain::land::TerramechanicsModel terra;
    terra.set_contact_area(0.63, 4.6);    // Track width, length
    terra.set_vehicle_weight(62000.0 * 9.81);

    // Run simulation
    for (int i = 0; i < 1000; ++i) {
        engine.step(0.02);  // 50 Hz

        // Check mobility
        Real sinkage = terra.get_sinkage();
        if (sinkage > 0.3) {
            std::cout << "Warning: High sinkage!\n";
        }
    }

    engine.shutdown();
    return 0;
}
```

## Suspension System

### Spring-Damper Model

Force calculation:
```
F = k × x + c × v + preload
```

Where:
- k = spring stiffness (N/m)
- c = damping coefficient (N·s/m)
- x = compression (m)
- v = compression rate (m/s)

### Bump Stop Modeling

Quadratic resistance at travel limits:
```cpp
if (position > travel_max - 0.02) {
    Real penetration = position - (travel_max - 0.02);
    F += 100000 × penetration² / 0.02;
}
```

### Configuration

```xml
<suspension type="torsion_bar">
    <road_wheels_per_side>7</road_wheels_per_side>
    <spring_rate unit="N/m">300000</spring_rate>
    <damping_rate unit="N*s/m">30000</damping_rate>
    <travel unit="m">0.40</travel>
</suspension>
```

## Tracked Vehicle Dynamics

### Track System

```cpp
TrackedVehicleModel tracks;

// Configure sprocket
tracks.set_sprocket(0.35, 80000.0);  // 35cm radius, 80 kN·m max torque

// Update dynamics
Real engine_torque = throttle * 80000.0;
tracks.update(engine_torque, vehicle_weight, soil, dt);

// Check track states
auto& left = tracks.get_left_track();
if (left.slip > 0.3) {
    // High slip - reduce throttle
}
```

### Drive Force

```
F_drive = T_drive / r_sprocket
```

### Propulsive Force with Efficiency

```
F_prop = T_track × (1 - slip) × η
```

Where η ≈ 0.85 (track efficiency).

## Coordinate Conventions

### Body-Axis System
- **X**: Forward (vehicle front)
- **Y**: Right (passenger side)
- **Z**: Down (into ground)

### Forces
- **Traction**: Positive X (forward acceleration)
- **Resistance**: Negative X (opposes motion)
- **Normal**: Ground reaction force

## Terrain Integration

The terramechanics model queries the environment:

```cpp
environment::Environment env;
// ... populated by engine ...

Real height = env.altitude - env.terrain_elevation;
if (height < 0.1) {
    // Vehicle is on ground, compute terra forces
    SoilProperties soil = env.terrain.material.get_soil_properties();
    terra.compute_forces(state, env, dt, forces);
}
```

## Vehicle Configuration

```xml
<entity type="ground_vehicle" name="M1A2">
    <metrics>
        <length unit="m">9.77</length>
        <width unit="m">3.66</width>
        <height unit="m">2.44</height>
    </metrics>

    <mass_balance>
        <combat_weight unit="kg">62000</combat_weight>
        <fuel_capacity unit="L">1900</fuel_capacity>
    </mass_balance>

    <tracks>
        <track_width unit="m">0.63</track_width>
        <track_length unit="m">4.6</track_length>
        <sprocket_radius unit="m">0.33</sprocket_radius>
        <max_torque unit="N*m">100000</max_torque>
    </tracks>
</entity>
```

## Performance Guidelines

| Parameter | Recommended Value |
|-----------|-------------------|
| Integration rate | 50-100 Hz |
| Sinkage limit | 0.5 m maximum |
| Contact minimum | 1 cm dimensions |
| Slip ratio bounds | [-1, 1] |

## See Also

- [Air Domain](air.md) - Aircraft simulation
- [Sea Domain](sea.md) - Naval vessel simulation
- [Space Domain](space.md) - Orbital mechanics
- [Examples](../tutorials/examples.md) - Complete code examples
