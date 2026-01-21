# Air Domain

The Air domain module implements aerodynamics, propulsion, and flight control systems for aircraft, missiles, and other airborne entities.

## Overview

JaguarEngine's Air domain provides:

- **6-DOF Flight Dynamics**: Full six degrees of freedom rigid body dynamics
- **Coefficient-Based Aerodynamics**: N-dimensional interpolation tables
- **Propulsion Models**: Turbofan/turbojet with altitude-Mach corrections
- **Flight Control Systems**: Rate limiting, autopilot, and stability augmentation

## Aerodynamic Model

### Coefficient Tables

Multi-dimensional lookup tables for aerodynamic coefficients:

| Coefficient | Dependencies | Description |
|-------------|--------------|-------------|
| C_L | α, M, δ_e | Lift coefficient |
| C_D | α, M, C_L | Drag coefficient |
| C_m | α, M, δ_e | Pitching moment |
| C_l | β, p̄, δ_a, δ_r | Rolling moment |
| C_n | β, r̄, δ_a, δ_r | Yawing moment |
| C_Y | β, δ_r | Side force |

### Equations of Motion

**Translational (ECEF frame):**
```
m·a = F_aero + F_thrust + F_gravity + F_ground
```

**Rotational (Body frame):**
```
I·ω̇ = M - ω × (I·ω)
```

**Quaternion Integration (Gimbal-lock free):**
```
q̇ = 0.5 · q ⊗ ω_body
```

## Example: Creating an Aircraft

```cpp
#include <jaguar/jaguar.h>

using namespace jaguar;

int main() {
    // Create engine
    Engine engine;
    engine.initialize();

    // Create aircraft entity
    EntityId aircraft = engine.create_entity("F16", Domain::Air);

    // Configure initial state
    physics::EntityState state;
    state.position = Vec3{0.0, 0.0, -10000.0};  // 10 km altitude (NED)
    state.velocity = Vec3{250.0, 0.0, 0.0};     // 250 m/s forward
    state.mass = 12000.0;                        // 12,000 kg
    engine.set_entity_state(aircraft, state);

    // Run simulation
    for (int i = 0; i < 1000; ++i) {
        engine.step(0.01);  // 100 Hz

        // Get current state
        auto current = engine.get_entity_state(aircraft);
        std::cout << "Altitude: " << -current.position.z << " m\n";
    }

    engine.shutdown();
    return 0;
}
```

## Propulsion

### Thrust Modeling

The propulsion model accounts for:

1. **Altitude Effect** (density ratio):
```cpp
σ = ρ / ρ_SL
altitude_factor = σ^0.7  // Typical turbofan
```

2. **Ram Effect** (Mach correction):
```cpp
if (M < 0.8)  ram = 1.0 + 0.15 × M
if (M < 1.2)  ram = 1.12 - 0.1 × (M - 0.8)
if (M ≥ 1.2)  ram = max(0.5, 1.08 - 0.3 × (M - 1.2))
```

3. **Net Thrust**:
```cpp
T = T_max × throttle × altitude_factor × ram_factor
```

### Configuration

```xml
<propulsion>
    <engine type="turbofan" name="F110-GE-129">
        <max_thrust unit="lbf">17000</max_thrust>
        <afterburner_thrust unit="lbf">29000</afterburner_thrust>
        <tsfc>0.76</tsfc>  <!-- lb/hr/lbf -->
    </engine>
</propulsion>
```

## Flight Control System

### Control Surface Mapping

```cpp
FlightControlSystem fcs;

// Configure limits
fcs.set_elevator_range(-25.0, 25.0);  // degrees
fcs.set_aileron_range(-20.0, 20.0);
fcs.set_rudder_range(-30.0, 30.0);

// Process pilot inputs
FlightControlSystem::ControlInputs inputs;
inputs.pitch_cmd = -0.3;   // Pull back
inputs.roll_cmd = 0.5;     // Roll right
inputs.yaw_cmd = 0.0;
inputs.throttle_cmd = 0.8;

auto outputs = fcs.process(inputs, dt);
// outputs.elevator_deg, outputs.aileron_deg, outputs.rudder_deg
```

## Coordinate Conventions

### Body-Axis System
- **X**: Forward (out the nose)
- **Y**: Right (out the right wing)
- **Z**: Down

### Sign Conventions

| Parameter | Positive Direction |
|-----------|-------------------|
| α (alpha) | Nose up relative to velocity |
| β (beta) | Nose left relative to velocity |
| Elevator | Trailing edge up (nose up) |
| Aileron | Right aileron trailing edge down |
| Rudder | Trailing edge left (nose left) |

## Missile-Specific Features

- **Seeker Model**: Configurable FOV, track rate, noise
- **Guidance Laws**: PNG, APN, optimal (configurable via XML)
- **Motor Profiles**: Thrust(t) curves with burn-out detection
- **Fin Actuators**: Rate limits, deflection limits, lag

## Performance Guidelines

| Parameter | Recommended Value |
|-----------|-------------------|
| Integration rate | 100 Hz (dt = 0.01s) |
| Alpha breakpoints | 5-7 minimum |
| Mach breakpoints | 0.0, 0.4, 0.8, 0.9, 1.0, 1.2, 1.5 |
| Minimum airspeed | 1 m/s threshold |

## See Also

- [Land Domain](land.md) - Ground vehicle simulation
- [Sea Domain](sea.md) - Naval vessel simulation
- [Space Domain](space.md) - Orbital mechanics
- [Examples](../tutorials/examples.md) - Complete code examples
