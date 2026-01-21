# Sea Domain

The Sea domain module implements hydrodynamics, buoyancy, wave modeling, and ship motion response for surface vessels and submarines.

## Overview

JaguarEngine's Sea domain provides:

- **Hydrostatic Buoyancy**: Archimedes' principle with metacentric stability
- **Wave Spectrum Models**: Pierson-Moskowitz, JONSWAP, Bretschneider
- **RAO-Based Motion**: Response Amplitude Operator seakeeping
- **MMG Maneuvering**: Maneuvering Mathematical Group hydrodynamics

## Sea State Conditions

### NATO Sea State Scale

| Code | Description | H_s (m) | T_p (s) |
|------|-------------|---------|---------|
| 0 | Calm (glassy) | 0.0 | 0.0 |
| 1 | Calm (rippled) | 0.05 | 2.0 |
| 2 | Smooth (wavelets) | 0.2 | 3.0 |
| 3 | Slight | 0.75 | 5.0 |
| 4 | Moderate | 1.5 | 6.5 |
| 5 | Rough | 2.75 | 8.0 |
| 6 | Very rough | 5.0 | 10.0 |
| 7 | High | 8.0 | 12.0 |
| 8 | Very high | 12.0 | 15.0 |

### Wave Spectrum Types

- **Pierson-Moskowitz**: Fully developed sea (open ocean)
- **JONSWAP**: Fetch-limited sea (coastal waters)
- **Bretschneider**: Parameterized spectrum

## Example: Creating a Ship

```cpp
#include <jaguar/jaguar.h>

using namespace jaguar;

int main() {
    Engine engine;
    engine.initialize();

    // Create surface vessel entity
    EntityId ship = engine.create_entity("DDG51", Domain::Sea);

    // Configure initial state
    physics::EntityState state;
    state.position = Vec3{0.0, 0.0, 0.0};
    state.velocity = Vec3{10.0, 0.0, 0.0};  // 10 m/s forward
    state.mass = 8600000.0;  // 8,600 tonnes
    engine.set_entity_state(ship, state);

    // Set sea conditions
    domain::sea::SeaState conditions = domain::sea::SeaState::FromNATOSeaState(4);

    // Run simulation
    for (int i = 0; i < 1000; ++i) {
        engine.step(0.05);  // 20 Hz

        auto current = engine.get_entity_state(ship);
        Real heel_deg = current.get_roll() * constants::RAD_TO_DEG;
        std::cout << "Heel angle: " << heel_deg << " deg\n";
    }

    engine.shutdown();
    return 0;
}
```

## Buoyancy Model

### Archimedes' Principle

```
F_b = ρ_water × g × V_displaced
```

### Static Stability - Righting Moment

```
M_roll = W × GM × sin(θ)
```

Where:
- W = vessel weight (N)
- GM = metacentric height (m)
- θ = heel angle (rad)

### Metacentric Height (GM)

| GM Value | Stability |
|----------|-----------|
| GM > 0 | Stable (positive righting moment) |
| GM = 0 | Neutral stability |
| GM < 0 | Unstable (capsizing tendency) |

### Typical GM Values

| Vessel Type | GM (m) |
|-------------|--------|
| Large tanker | 0.5-2.0 |
| Container ship | 1.0-3.0 |
| Naval vessel | 1.5-3.0 |
| Yacht | 0.5-1.5 |

## Wave Model

### Wave Superposition

```cpp
// Generate 30 wave components from spectrum
for each component i:
    amplitude[i] = sqrt(2 × S(ω[i]) × Δω)
    phase[i] = random

// Surface elevation at (x, y, t)
η = Σ a[i] × cos(k[i]×x + k[i]×y - ω[i]×t + φ[i])
```

### Usage

```cpp
WaveModel waves;
waves.set_sea_state(SeaState::FromNATOSeaState(5));  // Rough sea

// Surface elevation at ship position
Real elevation = waves.get_elevation(x, y, time);

// Particle velocity at keel depth
Vec3 orbital_velocity = waves.get_particle_velocity(x, y, -5.0, time);

// Surface slope for roll/pitch excitation
Vec3 slope = waves.get_slope(x, y, time);
```

## RAO Model

### Response Amplitude Operator

The RAO describes how a vessel responds to waves:

```
Response_amplitude = Wave_amplitude × RAO(ω)
Response_phase = Wave_phase + RAO_phase(ω)
```

### DOF Indices

| Index | Motion | Unit |
|-------|--------|------|
| 0 | Surge | m/m |
| 1 | Sway | m/m |
| 2 | Heave | m/m |
| 3 | Roll | rad/m |
| 4 | Pitch | rad/m |
| 5 | Yaw | rad/m |

### Configuration

```cpp
RAOModel rao;

// Set heave RAO data
std::vector<Real> freqs = {0.3, 0.5, 0.8, 1.0, 1.5};  // rad/s
std::vector<Real> amps = {0.8, 0.9, 1.0, 0.9, 0.6};   // RAO amplitude
std::vector<Real> phases = {0.0, 0.1, 0.2, 0.3, 0.5}; // RAO phase
rao.set_rao(2, freqs, amps, phases);  // DOF 2 = heave

// Calculate total response
Vec3 displacement, rotation;
rao.calculate_response(waves, time, displacement, rotation);
```

## MMG Maneuvering Model

### Force Decomposition

```
X_total = X_hull + X_propeller + X_rudder
Y_total = Y_hull + Y_rudder
N_total = N_hull + N_rudder
```

### Hull Coefficients

| Coefficient | Typical Value | Effect |
|-------------|---------------|--------|
| X_vv | -0.04 | Surge due to drift² |
| X_rr | -0.01 | Surge due to yaw rate² |
| Y_v | -0.4 | Sway due to drift |
| Y_r | 0.05 | Sway due to yaw rate |
| N_v | -0.1 | Yaw due to drift |
| N_r | -0.05 | Yaw damping |

### Configuration

```xml
<hydrodynamics>
    <hull_coefficients>
        <x_vv>-0.04</x_vv>
        <x_rr>-0.01</x_rr>
        <y_v>-0.4</y_v>
        <y_r>0.05</y_r>
        <n_v>-0.1</n_v>
        <n_r>-0.05</n_r>
    </hull_coefficients>

    <rudder>
        <area unit="m2">18.0</area>
        <aspect_ratio>1.6</aspect_ratio>
        <max_angle unit="deg">35.0</max_angle>
    </rudder>

    <propeller>
        <diameter unit="m">5.2</diameter>
        <pitch_ratio>1.0</pitch_ratio>
        <max_rpm>180</max_rpm>
    </propeller>
</hydrodynamics>
```

## Coordinate Conventions

### Ship Body-Axis System
- **X**: Forward (bow)
- **Y**: Starboard (right)
- **Z**: Down

### Motions

| Motion | Axis | Positive Direction |
|--------|------|-------------------|
| Surge | X | Forward |
| Sway | Y | Starboard |
| Heave | Z | Down |
| Roll | X | Starboard down |
| Pitch | Y | Bow up |
| Yaw | Z | Bow to starboard |

## Performance Guidelines

| Parameter | Recommended Value |
|-----------|-------------------|
| Integration rate | 20-50 Hz |
| Wave components | 30 for good spectrum |
| Frequency range | 0.3-2.5 rad/s |
| Minimum velocity | 0.01 m/s threshold |

## See Also

- [Air Domain](air.md) - Aircraft simulation
- [Land Domain](land.md) - Ground vehicle simulation
- [Space Domain](space.md) - Orbital mechanics
- [Examples](../tutorials/examples.md) - Complete code examples
