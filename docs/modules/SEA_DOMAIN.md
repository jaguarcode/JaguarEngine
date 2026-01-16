# Sea Domain Documentation

The Sea domain module implements hydrodynamics, buoyancy, wave modeling, and ship motion response for surface vessels and submarines.

## Headers

| Header | Purpose |
|--------|---------|
| `jaguar/domain/sea.h` | Buoyancy, waves, RAO, hydrodynamics |

## Components

### SeaState

Sea condition parameters:

```cpp
struct SeaState {
    Real significant_height{1.0};   // H_s (m)
    Real peak_period{6.0};          // T_p (s)
    Real direction{0.0};            // Primary wave direction (rad from N)
    WaveSpectrum spectrum{WaveSpectrum::PiersonMoskowitz};

    // NATO sea state presets (0-8)
    static SeaState FromNATOSeaState(int sea_state);
};
```

**NATO Sea State Scale:**

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

**Wave Spectrum Types:**
- `PiersonMoskowitz`: Fully developed sea (open ocean)
- `JONSWAP`: Fetch-limited sea (coastal waters)
- `Bretschneider`: Parameterized spectrum

**Usage:**
```cpp
// From NATO code
SeaState conditions = SeaState::FromNATOSeaState(4);  // Moderate

// Custom conditions
SeaState custom;
custom.significant_height = 3.0;
custom.peak_period = 9.0;
custom.direction = 45.0 * constants::DEG_TO_RAD;  // From NE
custom.spectrum = WaveSpectrum::JONSWAP;
```

### WaveModel

Irregular wave surface simulation using spectral superposition:

```cpp
class WaveModel {
public:
    // Set sea conditions
    void set_sea_state(const SeaState& state);

    // Wave properties at position and time
    Real get_elevation(Real x, Real y, Real time) const;
    Vec3 get_particle_velocity(Real x, Real y, Real z, Real time) const;
    Vec3 get_slope(Real x, Real y, Real time) const;
};
```

**Wave Spectrum Formulas:**

Pierson-Moskowitz spectrum (deep water, fully developed):
```
S(ω) = (α × g² / ω⁵) × exp(-β × (ωₚ/ω)⁴)
```
Where α = 0.0081 (Phillips constant), β = 1.25.

JONSWAP spectrum (fetch-limited):
```
S(ω) = S_PM(ω) × γ^r
```
Where γ = 3.3 (peak enhancement), r = exp(-(ω-ωₚ)²/(2σ²ωₚ²)).

**Wave Superposition:**
```cpp
// Generate 30 wave components from spectrum
for each component i:
    amplitude[i] = sqrt(2 × S(ω[i]) × Δω)
    phase[i] = random

// Surface elevation at (x, y, t)
η = Σ a[i] × cos(k[i]×x + k[i]×y - ω[i]×t + φ[i])
```

**Deep Water Dispersion Relation:**
```
ω² = g × k    →    k = ω²/g
```

**Particle Velocity (Linear Airy Theory):**
```cpp
// Horizontal velocity (decays with depth)
u = a × ω × exp(k×z) × cos(θ)  // z < 0

// Vertical velocity
w = a × ω × exp(k×z) × sin(θ)
```

**Usage:**
```cpp
WaveModel waves;
waves.set_sea_state(SeaState::FromNATOSeaState(5));  // Rough sea

Real t = simulation_time;
Real x = ship_position.x;
Real y = ship_position.y;

// Surface elevation at ship position
Real elevation = waves.get_elevation(x, y, t);

// Particle velocity at keel depth
Real z = -5.0;  // 5m below surface
Vec3 orbital_velocity = waves.get_particle_velocity(x, y, z, t);

// Surface slope (for roll/pitch excitation)
Vec3 slope = waves.get_slope(x, y, t);
```

### BuoyancyModel

Hydrostatic buoyancy implementing `IHydrodynamicsModel`:

```cpp
class BuoyancyModel : public physics::IHydrodynamicsModel {
public:
    // Configuration
    void set_displaced_volume(Real volume);      // Hull volume (m³)
    void set_metacentric_height(Real gm);        // GM (m)
    void set_center_of_buoyancy(const Vec3& cb); // CB relative to CG

    // State queries
    Real get_buoyancy() const;  // Buoyancy force (N)
    Real get_draft() const;     // Current draft (m)
    Real get_heel() const;      // Roll angle (rad)
    Real get_trim() const;      // Pitch angle (rad)
};
```

**Archimedes' Principle:**
```
F_b = ρ × g × V_displaced
```

**Static Stability - Righting Moment:**
```
M_roll = W × GM × sin(θ)
```
Where:
- W = vessel weight (N)
- GM = metacentric height (m)
- θ = heel angle (rad)

**Metacentric Height (GM):**
- GM > 0: Stable (positive righting moment)
- GM = 0: Neutral stability
- GM < 0: Unstable (capsizing tendency)

**Typical GM Values:**
| Vessel Type | GM (m) |
|-------------|--------|
| Large tanker | 0.5-2.0 |
| Container ship | 1.0-3.0 |
| Naval vessel | 1.5-3.0 |
| Yacht | 0.5-1.5 |

**Usage:**
```cpp
BuoyancyModel buoyancy;

// Configure hull characteristics
buoyancy.set_displaced_volume(10000.0);  // 10,000 m³
buoyancy.set_metacentric_height(2.5);    // 2.5m GM
buoyancy.set_center_of_buoyancy(Vec3{0.0, 0.0, -2.0});  // 2m below CG

// In simulation loop
physics::EntityForces forces;
forces.clear();
buoyancy.compute_forces(state, env, dt, forces);

// Check stability
Real heel_deg = buoyancy.get_heel() * constants::RAD_TO_DEG;
if (std::abs(heel_deg) > 30.0) {
    // Danger of capsizing
}
```

### RAOModel

Response Amplitude Operator for ship motion prediction:

```cpp
class RAOModel {
public:
    // Set RAO data for DOF (0-5: surge, sway, heave, roll, pitch, yaw)
    void set_rao(int dof,
                 const std::vector<Real>& frequencies,  // rad/s
                 const std::vector<Real>& amplitudes,   // RAO values
                 const std::vector<Real>& phases);      // rad

    // Query response at specific frequency
    void get_response(Real omega, Real wave_amp,
                      Real& out_amplitude, Real& out_phase, int dof) const;

    // Calculate total 6DOF response to sea state
    void calculate_response(const WaveModel& waves, Real time,
                            Vec3& out_disp, Vec3& out_rot);
};
```

**RAO Concept:**

The RAO describes how a vessel responds to waves of different frequencies:
```
Response_amplitude = Wave_amplitude × RAO(ω)
Response_phase = Wave_phase + RAO_phase(ω)
```

**DOF Indices:**
| Index | Motion | Unit |
|-------|--------|------|
| 0 | Surge | m/m |
| 1 | Sway | m/m |
| 2 | Heave | m/m |
| 3 | Roll | rad/m |
| 4 | Pitch | rad/m |
| 5 | Yaw | rad/m |

**Usage:**
```cpp
RAOModel rao;

// Set heave RAO data
std::vector<Real> freqs = {0.3, 0.5, 0.8, 1.0, 1.5};  // rad/s
std::vector<Real> amps = {0.8, 0.9, 1.0, 0.9, 0.6};   // RAO amplitude
std::vector<Real> phases = {0.0, 0.1, 0.2, 0.3, 0.5}; // RAO phase
rao.set_rao(2, freqs, amps, phases);  // DOF 2 = heave

// Query response
Real omega = 0.7;      // Wave frequency
Real wave_amp = 2.0;   // Wave amplitude (m)
Real resp_amp, resp_phase;
rao.get_response(omega, wave_amp, resp_amp, resp_phase, 2);

// Total response to irregular sea
Vec3 displacement, rotation;
rao.calculate_response(waves, time, displacement, rotation);
```

### HydrodynamicsModel

Maneuvering Mathematical Group (MMG) model implementing `IHydrodynamicsModel`:

```cpp
class HydrodynamicsModel : public physics::IHydrodynamicsModel {
public:
    // Configuration
    void set_hull_coefficients(Real x_vv, Real x_rr,
                               Real y_v, Real y_r,
                               Real n_v, Real n_r);
    void set_rudder_parameters(Real area, Real aspect_ratio);
    void set_propeller_parameters(Real diameter, Real pitch_ratio);

    // Control
    void set_rudder_angle(Real angle_rad);  // Positive = port
    void set_propeller_rpm(Real rpm);

    // State queries
    Real get_buoyancy() const;
    Real get_draft() const;
    Real get_heel() const;
    Real get_trim() const;
};
```

**MMG Force Decomposition:**
```
X_total = X_hull + X_propeller + X_rudder
Y_total = Y_hull + Y_rudder
N_total = N_hull + N_rudder
```

**Hull Forces:**

Non-dimensional velocities:
```
v' = v / U
r' = r × L / U
```

Hull forces (surge, sway, yaw):
```
X_H = -R_T + ½ρL²U²(X_vv×v'² + X_rr×r'²)
Y_H = Y_cross + ½ρL²U²(Y_v×v' + Y_r×r')
N_H = ½ρL³U²(N_v×v' + N_r×r')
```

**Propeller Thrust:**

Using actuator disk theory:
```
J = V_A / (n × D)           // Advance coefficient
K_T = a₀ + a₁J + a₂J²       // Thrust coefficient
T = K_T × ρ × n² × D⁴       // Thrust
T_net = T × (1 - t)         // With thrust deduction
```
Where t ≈ 0.2 (thrust deduction factor).

**Rudder Forces:**

Lift coefficient:
```
CL = CL_α × α_R    (for α < stall)
```

Lift curve slope:
```
CL_α = π × AR / (1 + AR)
```

Rudder forces:
```
L_R = ½ρU²A_rudder×CL
N_R = Y_R × x_R              // Yaw moment
```

**Typical Hull Coefficients:**

| Coefficient | Typical Value | Effect |
|-------------|---------------|--------|
| X_vv | -0.04 | Surge due to drift² |
| X_rr | -0.01 | Surge due to yaw rate² |
| Y_v | -0.4 | Sway due to drift |
| Y_r | 0.05 | Sway due to yaw rate |
| N_v | -0.1 | Yaw due to drift |
| N_r | -0.05 | Yaw damping |

**Usage:**
```cpp
HydrodynamicsModel hydro;

// Configure hull
hydro.set_hull_coefficients(-0.04, -0.01, -0.4, 0.05, -0.1, -0.05);

// Configure rudder
hydro.set_rudder_parameters(15.0, 1.8);  // 15m², AR=1.8

// Configure propeller
hydro.set_propeller_parameters(5.0, 1.0);  // 5m diameter

// In simulation loop
hydro.set_propeller_rpm(100.0);
hydro.set_rudder_angle(0.1);  // ~6 degrees

physics::EntityForces forces;
forces.clear();
hydro.compute_forces(state, env, dt, forces);
```

## Complete Surface Vessel Example

```cpp
#include <jaguar/jaguar.h>

class Ship {
public:
    Ship(const std::string& name) {
        entity_id_ = entities_.create_entity(name, Domain::Sea);

        // Configure buoyancy
        buoyancy_.set_displaced_volume(15000.0);  // 15,000 m³
        buoyancy_.set_metacentric_height(2.0);

        // Configure hydrodynamics
        hydro_.set_hull_coefficients(-0.04, -0.01, -0.4, 0.05, -0.1, -0.05);
        hydro_.set_rudder_parameters(12.0, 1.5);
        hydro_.set_propeller_parameters(4.5, 1.0);

        // Set sea state
        waves_.set_sea_state(SeaState::FromNATOSeaState(4));
    }

    void set_throttle(Real throttle) {
        hydro_.set_propeller_rpm(throttle * 120.0);  // Max 120 RPM
    }

    void set_rudder(Real command) {
        hydro_.set_rudder_angle(command * 0.52);  // Max 30°
    }

    void update(Real dt) {
        auto state = entities_.get_state(entity_id_);

        environment::Environment env;
        env.over_water = true;
        env.ocean.surface_elevation = waves_.get_elevation(
            state.position.x, state.position.y, sim_time_);
        // ... set other environment fields ...

        forces_.clear();

        // Buoyancy and stability
        buoyancy_.compute_forces(state, env, dt, forces_);

        // Maneuvering forces
        hydro_.compute_forces(state, env, dt, forces_);

        // Gravity
        Vec3 gravity{0.0, 0.0, state.mass * constants::G0};
        forces_.add_force(gravity);

        // Ship motion from waves (seakeeping)
        Vec3 wave_disp, wave_rot;
        rao_.calculate_response(waves_, sim_time_, wave_disp, wave_rot);

        sim_time_ += dt;
        // ... integrate state ...
    }

    // Telemetry
    Real get_draft() const { return buoyancy_.get_draft(); }
    Real get_heel_deg() const { return buoyancy_.get_heel() * constants::RAD_TO_DEG; }
    Real get_speed() const;

private:
    physics::EntityManager entities_;
    EntityId entity_id_;

    BuoyancyModel buoyancy_;
    HydrodynamicsModel hydro_;
    WaveModel waves_;
    RAOModel rao_;
    physics::EntityForces forces_;

    Real sim_time_{0.0};
};
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

### Control Surface Signs
- **Rudder**: Positive = trailing edge to port (ship turns starboard)
- **Propeller RPM**: Positive = ahead

## Performance Considerations

### Wave Model
- 30 wave components provides good spectral representation
- Frequency range: 0.3-2.5 rad/s covers most wave energy
- Deterministic phases for reproducibility

### Numerical Stability
- Minimum velocity threshold (0.01 m/s) prevents singularities
- Rudder stall model prevents unrealistic forces
- Roll damping essential for stability

### Integration
- 20-50 Hz typical for surface vessels
- Higher rates for high-speed craft
- Lower rates acceptable for large slow vessels

## References

- **Ship Stability**: Biran - "Ship Hydrostatics and Stability"
- **Seakeeping**: Lloyd - "Seakeeping: Ship Behaviour in Rough Weather"
- **Maneuvering**: Fossen - "Handbook of Marine Craft Hydrodynamics and Motion Control"
- **MMG Model**: Ogawa - "MMG Report I-IV" (JTTC)
- **Wave Spectra**: Ochi - "Ocean Waves: The Stochastic Approach"
