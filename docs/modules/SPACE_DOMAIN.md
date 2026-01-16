# Space Domain Documentation

The Space domain module implements orbital mechanics, gravity models, and atmospheric drag for satellites and space vehicles.

## Headers

| Header | Purpose |
|--------|---------|
| `jaguar/domain/space.h` | Orbital elements, SGP4, gravity, drag |

## Components

### OrbitalElements

Classical Keplerian orbital elements:

```cpp
struct OrbitalElements {
    Real semi_major_axis{7000000.0};  // a (m)
    Real eccentricity{0.0};           // e
    Real inclination{0.0};            // i (rad)
    Real raan{0.0};                   // Right Ascension of Ascending Node (rad)
    Real arg_of_perigee{0.0};         // Argument of perigee (rad)
    Real true_anomaly{0.0};           // True anomaly (rad)

    void to_state_vector(Vec3& pos_eci, Vec3& vel_eci) const;
    static OrbitalElements from_state_vector(const Vec3& pos_eci, const Vec3& vel_eci);
    Real period() const;
    Real altitude() const;
};
```

**Element Definitions:**

| Element | Symbol | Range | Description |
|---------|--------|-------|-------------|
| Semi-major axis | a | > 0 | Size of orbit (m) |
| Eccentricity | e | [0, 1) | Shape (0=circular, 0.999=highly elliptical) |
| Inclination | i | [0, π] | Tilt relative to equator (rad) |
| RAAN | Ω | [0, 2π] | Orientation in equatorial plane (rad) |
| Arg. of perigee | ω | [0, 2π] | Orientation within orbital plane (rad) |
| True anomaly | ν | [0, 2π] | Current position in orbit (rad) |

**Orbital Period:**
```
T = 2π × √(a³/μ)
```
Where μ = 3.986004418 × 10¹⁴ m³/s² (Earth GM).

**Common Orbit Types:**

| Orbit Type | Altitude (km) | Period | Inclination |
|------------|---------------|--------|-------------|
| LEO | 160-2000 | 90-120 min | Various |
| MEO | 2000-35786 | 2-24 hr | Various |
| GEO | 35786 | 24 hr | 0° |
| SSO | 400-800 | ~98 min | 97-99° |
| Polar | Various | Various | ~90° |

**Usage:**
```cpp
OrbitalElements orbit;

// ISS-like orbit
orbit.semi_major_axis = constants::R_EARTH_EQUATOR + 420000.0;  // ~420 km altitude
orbit.eccentricity = 0.0001;           // Nearly circular
orbit.inclination = 51.6 * DEG_TO_RAD; // ISS inclination
orbit.raan = 0.0;
orbit.arg_of_perigee = 0.0;
orbit.true_anomaly = 0.0;

// Convert to state vectors
Vec3 pos_eci, vel_eci;
orbit.to_state_vector(pos_eci, vel_eci);

// Get orbital period
Real period = orbit.period();  // ~92 minutes

// Current altitude
Real alt = orbit.altitude();
```

### TLE (Two-Line Element)

Standard format for satellite orbital data:

```cpp
struct TLE {
    std::string name;
    std::string line1;
    std::string line2;

    // Parsed values
    int satellite_number{0};
    Real epoch_year{0.0};
    Real epoch_day{0.0};
    Real bstar{0.0};           // Drag term (1/Earth radii)
    Real inclination{0.0};     // rad
    Real raan{0.0};            // rad
    Real eccentricity{0.0};
    Real arg_of_perigee{0.0};  // rad
    Real mean_anomaly{0.0};    // rad
    Real mean_motion{0.0};     // rev/day

    static TLE parse(const std::string& line1, const std::string& line2);
};
```

**TLE Format Example:**
```
ISS (ZARYA)
1 25544U 98067A   21275.52422453  .00001234  00000-0  28888-4 0  9990
2 25544  51.6442  27.4345 0003542 211.7212 148.3510 15.48919914305213
```

**Line 1 Fields:**
- Satellite number (25544)
- Classification (U = Unclassified)
- Launch designator (98067A)
- Epoch (year + day fraction)
- B* drag term
- Checksum

**Line 2 Fields:**
- Inclination (degrees)
- RAAN (degrees)
- Eccentricity (decimal point assumed)
- Argument of perigee (degrees)
- Mean anomaly (degrees)
- Mean motion (rev/day)
- Revolution number

**Usage:**
```cpp
std::string line1 = "1 25544U 98067A   21275.52422453  .00001234  00000-0  28888-4 0  9990";
std::string line2 = "2 25544  51.6442  27.4345 0003542 211.7212 148.3510 15.48919914305213";

TLE tle = TLE::parse(line1, line2);

// Access parsed values
Real inc_deg = tle.inclination * RAD_TO_DEG;  // 51.6442°
Real rev_per_day = tle.mean_motion;           // 15.489 rev/day
```

### SGP4Propagator

SGP4/SDP4 orbital propagator for satellite tracking:

```cpp
class SGP4Propagator {
public:
    // Initialize from TLE
    bool initialize(const TLE& tle);

    // Propagate to time (minutes from epoch)
    bool propagate(Real minutes_since_epoch, Vec3& pos_eci, Vec3& vel_eci);

    // Get current orbital elements
    OrbitalElements get_elements() const;

    // Check propagation mode
    bool is_deep_space() const;
};
```

**SGP4 vs SDP4:**
- **SGP4**: Near-Earth satellites (period < 225 minutes)
- **SDP4**: Deep-space satellites (period ≥ 225 minutes)
- Selection is automatic based on mean motion

**Perturbations Modeled:**
- Earth oblateness (J2, J3, J4)
- Atmospheric drag (for LEO)
- Solar/lunar gravity (for deep-space)
- Solar radiation pressure (simplified)

**Usage:**
```cpp
SGP4Propagator sgp4;

// Initialize from TLE
TLE tle = TLE::parse(line1, line2);
if (!sgp4.initialize(tle)) {
    // Initialization failed
}

// Propagate forward 90 minutes
Vec3 pos_km, vel_kms;
if (sgp4.propagate(90.0, pos_km, vel_kms)) {
    // Convert km to m
    Vec3 pos_m = pos_km * 1000.0;
    Vec3 vel_ms = vel_kms * 1000.0;
}

// Check if deep-space mode
if (sgp4.is_deep_space()) {
    // Using SDP4 propagator
}
```

### GravityModel

Gravitational force model implementing `IGravityModel`:

```cpp
class GravityModel : public physics::IGravityModel {
public:
    // Set fidelity level (0-3)
    void set_fidelity(int level);
    int get_fidelity() const;

    // Get computed acceleration
    Vec3 get_gravity_acceleration() const;
};
```

**Fidelity Levels:**

| Level | Model | Accuracy | Performance |
|-------|-------|----------|-------------|
| 0 | Point mass | ~1 km at LEO | Fastest |
| 1 | J2 | ~100 m at LEO | Fast |
| 2 | J2-J4 | ~10 m at LEO | Moderate |
| 3 | EGM96/EGM2008 | ~1 m | Slowest |

**Point Mass Gravity:**
```
a = -μ/r³ × r
```

**J2 Perturbation:**
```
a_J2 = (3/2) × J2 × (μ/r²) × (R_E/r)² × f(latitude)
```
Where J2 = 1.08263 × 10⁻³.

**Zonal Harmonics:**
| Coefficient | Value | Effect |
|-------------|-------|--------|
| J2 | 1.08263 × 10⁻³ | Earth oblateness (dominant) |
| J3 | -2.532 × 10⁻⁶ | North-south asymmetry |
| J4 | -1.6109 × 10⁻⁶ | Higher-order oblateness |

**Usage:**
```cpp
GravityModel gravity;

// Set fidelity (1 = J2 perturbation)
gravity.set_fidelity(1);

// In simulation loop
physics::EntityForces forces;
forces.clear();
gravity.compute_forces(state, env, dt, forces);

// Get acceleration for analysis
Vec3 g = gravity.get_gravity_acceleration();
Real g_mag = g.length();  // m/s²
```

### JB08AtmosphereModel

Jacchia-Bowman 2008 thermospheric density model:

```cpp
class JB08AtmosphereModel {
public:
    // Get density at position
    Real get_density(Real altitude, Real latitude, Real longitude, Real time) const;

    // Set space weather indices
    void set_space_weather(Real f107, Real f107_avg, Real ap);
};
```

**Space Weather Parameters:**

| Parameter | Description | Typical Range | Effect |
|-----------|-------------|---------------|--------|
| F10.7 | Solar radio flux | 70-300 sfu | Higher = denser atmosphere |
| F10.7 avg | 81-day average | 70-250 sfu | Background activity |
| Ap | Geomagnetic index | 0-400 | Higher = more heating |

**Atmospheric Density vs Altitude:**

| Altitude (km) | Quiet Sun (kg/m³) | Active Sun (kg/m³) |
|---------------|-------------------|-------------------|
| 200 | ~3 × 10⁻¹⁰ | ~1 × 10⁻⁹ |
| 400 | ~3 × 10⁻¹² | ~3 × 10⁻¹¹ |
| 600 | ~1 × 10⁻¹³ | ~3 × 10⁻¹² |
| 800 | ~1 × 10⁻¹⁴ | ~5 × 10⁻¹³ |

**Usage:**
```cpp
JB08AtmosphereModel atm;

// Set current space weather
atm.set_space_weather(150.0, 145.0, 15.0);  // Moderate activity

// Get density at ISS altitude
Real density = atm.get_density(420000.0, 51.6 * DEG_TO_RAD, 0.0, sim_time);
```

### AtmosphericDragModel

Atmospheric drag force for satellites:

```cpp
class AtmosphericDragModel : public physics::IForceGenerator {
public:
    void set_ballistic_coefficient(Real bc);  // m²/kg
    void set_area(Real area);                 // m²
    void set_cd(Real cd);                     // Drag coefficient
};
```

**Drag Force:**
```
F_drag = -½ × ρ × Cd × A × V² × v̂
```

**Ballistic Coefficient:**
```
BC = Cd × A / m
```

**Typical Values:**

| Satellite Type | Cd | Area (m²) | Mass (kg) | BC (m²/kg) |
|---------------|-----|-----------|-----------|------------|
| Cubesat (1U) | 2.2 | 0.01 | 1.3 | 0.017 |
| Small sat | 2.2 | 1.0 | 100 | 0.022 |
| ISS | 2.0 | 2500 | 420000 | 0.012 |

**Usage:**
```cpp
AtmosphericDragModel drag;

// Configure for small satellite
drag.set_cd(2.2);
drag.set_area(1.0);      // 1 m²
// or set ballistic coefficient directly
drag.set_ballistic_coefficient(0.022);

// In simulation loop
physics::EntityForces forces;
forces.clear();
drag.compute_forces(state, env, dt, forces);
```

### Coordinate Transformations

```cpp
namespace transforms {
    // Frame conversions
    Vec3 ecef_to_eci(const Vec3& ecef, Real time);
    Vec3 eci_to_ecef(const Vec3& eci, Real time);

    // Geodetic conversions
    Vec3 geodetic_to_ecef(Real lat_rad, Real lon_rad, Real alt_m);
    void ecef_to_geodetic(const Vec3& ecef, Real& lat, Real& lon, Real& alt);

    // Time systems
    Real gmst(Real julian_date);  // Greenwich Mean Sidereal Time
}
```

**Coordinate Frames:**

| Frame | Origin | Axes | Use Case |
|-------|--------|------|----------|
| ECI | Earth center | J2000 inertial | Orbit propagation |
| ECEF | Earth center | Earth-fixed | Ground track |
| Geodetic | Earth surface | lat/lon/alt | Ground station |

**ECI to ECEF Transformation:**
```
R_ECEF = R_z(GMST) × R_ECI
```
Where GMST = Greenwich Mean Sidereal Time.

**Usage:**
```cpp
using namespace transforms;

// ECI position to ECEF
Vec3 pos_eci{6878000.0, 0.0, 0.0};  // On x-axis
Real time = simulation_time;
Vec3 pos_ecef = eci_to_ecef(pos_eci, time);

// ECEF to geodetic
Real lat, lon, alt;
ecef_to_geodetic(pos_ecef, lat, lon, alt);

// Ground station location to ECEF
Vec3 station = geodetic_to_ecef(
    37.0 * DEG_TO_RAD,   // Latitude
    -122.0 * DEG_TO_RAD, // Longitude
    100.0                // Altitude (m)
);
```

## Complete Satellite Example

```cpp
#include <jaguar/jaguar.h>

class Satellite {
public:
    Satellite(const std::string& name, const TLE& tle) {
        entity_id_ = entities_.create_entity(name, Domain::Space);

        // Initialize SGP4 propagator
        if (!sgp4_.initialize(tle)) {
            throw std::runtime_error("Failed to initialize SGP4");
        }

        // Configure gravity model (J2)
        gravity_.set_fidelity(1);

        // Configure drag model
        drag_.set_cd(2.2);
        drag_.set_area(1.0);

        tle_epoch_ = tle.epoch_year + tle.epoch_day / 365.25;
    }

    void update(Real sim_time_jd, Real dt) {
        // Minutes since TLE epoch
        Real minutes = (sim_time_jd - tle_epoch_) * 24.0 * 60.0;

        // Propagate with SGP4
        Vec3 pos_km, vel_kms;
        if (sgp4_.propagate(minutes, pos_km, vel_kms)) {
            // Convert to SI units
            Vec3 pos_m = pos_km * 1000.0;
            Vec3 vel_ms = vel_kms * 1000.0;

            // Update entity state
            physics::EntityState state;
            state.position = pos_m;
            state.velocity = vel_ms;
            state.mass = 100.0;  // 100 kg satellite
            entities_.set_state(entity_id_, state);
        }

        // For high-fidelity propagation, add perturbation forces
        auto state = entities_.get_state(entity_id_);
        forces_.clear();

        environment::Environment env;
        // Convert position to geodetic for atmosphere
        using namespace transforms;
        Real lat, lon, alt;
        Vec3 pos_ecef = eci_to_ecef(state.position, sim_time_jd);
        ecef_to_geodetic(pos_ecef, lat, lon, alt);
        env.altitude = alt;

        // Gravity perturbations beyond J2
        gravity_.compute_forces(state, env, dt, forces_);

        // Atmospheric drag (for LEO)
        if (alt < 800000.0) {  // Below 800 km
            drag_.compute_forces(state, env, dt, forces_);
        }
    }

    // Telemetry
    OrbitalElements get_orbit() const { return sgp4_.get_elements(); }
    Real get_altitude() const {
        auto state = entities_.get_state(entity_id_);
        return state.position.length() - constants::R_EARTH_EQUATOR;
    }

private:
    physics::EntityManager entities_;
    EntityId entity_id_;

    SGP4Propagator sgp4_;
    GravityModel gravity_;
    AtmosphericDragModel drag_;
    physics::EntityForces forces_;

    Real tle_epoch_{0.0};
};
```

## Orbit Determination Utilities

### Visibility Analysis

```cpp
// Check if satellite is visible from ground station
bool is_visible(const Vec3& sat_pos_eci,
                const Vec3& station_pos_ecef,
                Real time,
                Real min_elevation_rad) {
    using namespace transforms;

    // Convert satellite to ECEF
    Vec3 sat_ecef = eci_to_ecef(sat_pos_eci, time);

    // Vector from station to satellite
    Vec3 range = sat_ecef - station_pos_ecef;

    // Station location
    Real lat, lon, alt;
    ecef_to_geodetic(station_pos_ecef, lat, lon, alt);

    // Compute local vertical (up direction)
    Vec3 up = station_pos_ecef.normalized();

    // Elevation angle
    Real elevation = std::asin(range.dot(up) / range.length());

    return elevation > min_elevation_rad;
}
```

### Ground Track

```cpp
// Get satellite ground track (sub-satellite point)
void get_ground_track(const Vec3& sat_pos_eci, Real time,
                      Real& lat_out, Real& lon_out) {
    using namespace transforms;

    Vec3 pos_ecef = eci_to_ecef(sat_pos_eci, time);
    Real alt;
    ecef_to_geodetic(pos_ecef, lat_out, lon_out, alt);
}
```

## Performance Considerations

### SGP4 Propagation
- Very fast (~1 μs per propagation step)
- Accuracy degrades over time (days to weeks from epoch)
- Best for operational tracking, not precision applications

### Gravity Model Selection
- Level 0-1: Real-time simulation
- Level 2: Medium-fidelity analysis
- Level 3: Precision orbit determination

### Atmospheric Drag
- Only significant below ~800 km
- Varies with solar activity (factor of 10+)
- Largest source of LEO orbit uncertainty

### Integration
- SGP4: Use directly (pre-integrated analytical)
- Numerical integration: RK4 at 10-60 second steps for LEO
- Higher fidelity: Adaptive step-size integrators

## References

- **SGP4/SDP4**: Hoots & Roehrich - "Spacetrack Report #3"
- **Gravity Models**: Lemoine et al. - "EGM96 Gravity Model"
- **Atmospheric Models**: Bowman et al. - "JB2008 Atmospheric Model"
- **Astrodynamics**: Vallado - "Fundamentals of Astrodynamics and Applications"
- **Orbit Mechanics**: Bate, Mueller, White - "Fundamentals of Astrodynamics"
