# Sea Domain API Reference

Hydrodynamics, buoyancy, waves, and ship maneuvering APIs.

**Header:** `jaguar/domain/sea.h`

## SeaState

Ocean wave conditions.

```cpp
enum class WaveSpectrum {
    PiersonMoskowitz,  // Fully developed sea
    JONSWAP,           // Fetch-limited sea
    Bretschneider      // Two-parameter spectrum
};

struct SeaState {
    Real significant_height{1.0};  // H_s (m)
    Real peak_period{6.0};         // T_p (s)
    Real direction{0.0};           // Primary wave direction (rad from N)
    WaveSpectrum spectrum{WaveSpectrum::PiersonMoskowitz};
    Real spreading{0.0};           // Directional spreading (rad)

    // Create from NATO sea state code (0-8)
    static SeaState FromNATOSeaState(int sea_state);

    // Derived quantities
    Real mean_wave_length() const;
    Real mean_wave_period() const;
};
```

### NATO Sea State Scale

| Code | Significant Height | Description |
|------|-------------------|-------------|
| 0 | 0 m | Calm (glassy) |
| 1 | 0 - 0.1 m | Calm (rippled) |
| 2 | 0.1 - 0.5 m | Smooth |
| 3 | 0.5 - 1.25 m | Slight |
| 4 | 1.25 - 2.5 m | Moderate |
| 5 | 2.5 - 4 m | Rough |
| 6 | 4 - 6 m | Very rough |
| 7 | 6 - 9 m | High |
| 8 | 9+ m | Very high |

### Example

```cpp
// From NATO sea state
auto sea = domain::sea::SeaState::FromNATOSeaState(4);  // Moderate

// Or specify directly
domain::sea::SeaState sea;
sea.significant_height = 2.0;
sea.peak_period = 8.0;
sea.direction = 45.0 * DEG_TO_RAD;  // NE swell
sea.spectrum = WaveSpectrum::JONSWAP;
```

## WaveModel

Wave surface simulation.

```cpp
class WaveModel {
public:
    // Configuration
    void set_sea_state(const SeaState& state);
    void set_component_count(int count);  // Number of wave components
    void seed_random(UInt32 seed);        // For reproducibility

    // Query wave surface
    Real get_elevation(Real x, Real y, Real time) const;
    Vec3 get_slope(Real x, Real y, Real time) const;

    // Subsurface kinematics
    Vec3 get_particle_velocity(Real x, Real y, Real z, Real time) const;
    Vec3 get_particle_acceleration(Real x, Real y, Real z, Real time) const;
    Real get_pressure(Real x, Real y, Real z, Real time) const;

    // Update internal state
    void update(Real dt);
};
```

### Example

```cpp
domain::sea::WaveModel waves;

// Set sea state
auto sea = domain::sea::SeaState::FromNATOSeaState(5);
waves.set_sea_state(sea);
waves.set_component_count(50);

// Query wave height at ship location
Real eta = waves.get_elevation(ship_x, ship_y, sim_time);

// Get wave slope for ship orientation
Vec3 slope = waves.get_slope(ship_x, ship_y, sim_time);

// Subsurface velocity (for hydrofoil or sonar)
Vec3 particle_vel = waves.get_particle_velocity(x, y, -10.0, sim_time);
```

## BuoyancyModel

Static and dynamic buoyancy forces.

```cpp
class BuoyancyModel : public physics::IHydrodynamicsModel {
public:
    // Configuration
    void set_displaced_volume(Real volume);       // m³
    void set_metacentric_height(Real gm);         // GM (m)
    void set_center_of_buoyancy(const Vec3& cb);  // Relative to CG
    void set_waterplane_area(Real area);          // m²

    // Hull shape (optional, for detailed calculation)
    void set_length(Real length);
    void set_beam(Real beam);
    void set_draft(Real draft);

    // IForceGenerator
    void compute_forces(const physics::EntityState& state,
                        const environment::Environment& env,
                        Real dt,
                        physics::EntityForces& forces) override;

    // IHydrodynamicsModel queries
    Real get_buoyancy() const override;   // N
    Real get_draft() const override;      // m
    Real get_heel() const override;       // rad
    Real get_trim() const override;       // rad

    // Additional queries
    Real get_reserve_buoyancy() const;    // N
    Real get_gm_transverse() const;       // m
    Real get_gm_longitudinal() const;     // m
};
```

### Example

```cpp
domain::sea::BuoyancyModel buoyancy;

// Configure for destroyer
buoyancy.set_displaced_volume(8390.0);      // m³
buoyancy.set_metacentric_height(2.5);       // GM = 2.5 m
buoyancy.set_center_of_buoyancy(Vec3{0.0, 0.0, -4.7});
buoyancy.set_waterplane_area(2500.0);

// In simulation loop
physics::EntityForces forces;
buoyancy.compute_forces(state, env, dt, forces);

// Check stability
Real heel = buoyancy.get_heel() * RAD_TO_DEG;
if (std::abs(heel) > 20.0) {
    std::cout << "Warning: Excessive heel angle!\n";
}
```

## HydrodynamicsModel

MMG (Maneuvering Modeling Group) ship maneuvering model.

```cpp
class HydrodynamicsModel : public physics::IHydrodynamicsModel {
public:
    // Hull coefficients (non-dimensional)
    void set_hull_coefficients(Real x_vv, Real x_rr,
                               Real y_v, Real y_r,
                               Real n_v, Real n_r);

    // Cross-coupling coefficients
    void set_cross_coefficients(Real y_vvr, Real y_vrr,
                                Real n_vvr, Real n_vrr);

    // Added mass coefficients
    void set_added_mass(Real x_udot, Real y_vdot, Real n_rdot);

    // Hull dimensions
    void set_length(Real lpp);        // Length between perpendiculars (m)
    void set_beam(Real beam);         // Beam (m)
    void set_draft(Real draft);       // Draft (m)
    void set_displacement(Real disp); // Displacement (kg)

    // Rudder parameters
    void set_rudder_area(Real area);           // m²
    void set_rudder_aspect_ratio(Real ar);
    void set_rudder_position(const Vec3& pos); // Relative to CG

    // Propeller parameters
    void set_propeller_diameter(Real d);       // m
    void set_propeller_pitch_ratio(Real pr);
    void set_propeller_position(const Vec3& pos);

    // Control
    void set_rudder_angle(Real angle_rad);
    void set_propeller_rpm(Real rpm);

    // IForceGenerator
    void compute_forces(const physics::EntityState& state,
                        const environment::Environment& env,
                        Real dt,
                        physics::EntityForces& forces) override;

    // Queries
    Real get_ship_speed() const;        // m/s
    Real get_drift_angle() const;       // rad
    Real get_turn_rate() const;         // rad/s
    Real get_advance() const;           // m (in turning)
    Real get_transfer() const;          // m (in turning)
    Real get_tactical_diameter() const; // m
};
```

### Example

```cpp
domain::sea::HydrodynamicsModel hydro;

// Set hull coefficients (from model tests or CFD)
hydro.set_hull_coefficients(
    -0.04,   // X_vv
    -0.01,   // X_rr
    -0.4,    // Y_v
    0.05,    // Y_r
    -0.1,    // N_v
    -0.05    // N_r
);

// Hull dimensions
hydro.set_length(154.0);
hydro.set_beam(20.0);
hydro.set_draft(9.4);
hydro.set_displacement(8600000.0);

// Rudder
hydro.set_rudder_area(18.0);
hydro.set_rudder_aspect_ratio(1.6);

// Propeller
hydro.set_propeller_diameter(5.2);
hydro.set_propeller_pitch_ratio(1.0);

// Control
hydro.set_propeller_rpm(120.0);
hydro.set_rudder_angle(rudder_cmd * 35.0 * DEG_TO_RAD);

// Compute forces
physics::EntityForces forces;
hydro.compute_forces(state, env, dt, forces);
```

## RAOModel

Response Amplitude Operators for seakeeping.

```cpp
class RAOModel {
public:
    // Set RAO for each DOF
    // DOF: 0=surge, 1=sway, 2=heave, 3=roll, 4=pitch, 5=yaw
    void set_rao(int dof,
                 const std::vector<Real>& frequencies,  // rad/s
                 const std::vector<Real>& amplitudes,   // m/m or rad/m
                 const std::vector<Real>& phases);      // rad

    // Load from file
    bool load_from_file(const std::string& path);

    // Query RAO at specific frequency
    void get_response(Real omega, Real wave_amp,
                      Real& out_amplitude, Real& out_phase, int dof) const;

    // Calculate total response from wave spectrum
    void calculate_response(const WaveModel& waves, Real time,
                            Vec3& out_displacement,   // surge, sway, heave
                            Vec3& out_rotation);      // roll, pitch, yaw

    // Statistical predictions
    Real get_significant_response(int dof, const SeaState& sea) const;
    Real get_rms_response(int dof, const SeaState& sea) const;
};
```

### Example

```cpp
domain::sea::RAOModel rao;

// Set heave RAO
std::vector<Real> freqs = {0.3, 0.5, 0.7, 1.0, 1.5};
std::vector<Real> heave_amp = {0.9, 1.0, 0.95, 0.7, 0.4};
std::vector<Real> heave_phase = {0.0, 0.1, 0.2, 0.4, 0.6};
rao.set_rao(2, freqs, heave_amp, heave_phase);  // DOF 2 = heave

// Set roll RAO
std::vector<Real> roll_amp = {0.02, 0.05, 0.08, 0.04, 0.02};
std::vector<Real> roll_phase = {0.0, 0.2, 0.5, 0.8, 1.0};
rao.set_rao(3, freqs, roll_amp, roll_phase);  // DOF 3 = roll

// Calculate response
Vec3 displacement, rotation;
rao.calculate_response(waves, sim_time, displacement, rotation);

// Apply to ship motion
state.position += displacement;
// Apply rotation to orientation...

// Predict performance
Real sig_roll = rao.get_significant_response(3, sea);
std::cout << "Significant roll: " << sig_roll * RAD_TO_DEG << " deg\n";
```

## Current Model

Ocean current effects.

```cpp
class CurrentModel {
public:
    // Set uniform current
    void set_surface_current(const Vec3& velocity);  // m/s

    // Set depth-varying current
    void set_current_profile(const std::vector<Real>& depths,
                             const std::vector<Vec3>& velocities);

    // Query
    Vec3 get_current(Real depth) const;

    // For relative velocity calculations
    Vec3 get_relative_velocity(const physics::EntityState& state,
                               Real depth) const;
};
```

## See Also

- [Physics API](physics.md) - Force generator interfaces
- [Environment API](environment.md) - Ocean system
- [Sea Domain Concepts](../domains/sea.md) - Domain overview
- [Sea Domain Tutorial](../tutorials/sea-domain.md) - Step-by-step guide

