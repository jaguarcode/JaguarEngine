# Space Domain

The Space domain module implements orbital mechanics, gravity models, atmospheric drag, and coordinate transformations for satellites and space vehicles.

## Overview

JaguarEngine's Space domain provides:

- **SGP4/SDP4 Propagation**: Analytical orbital propagation from TLE data
- **High-Fidelity Gravity**: Point mass to EGM2008 models
- **Atmospheric Drag**: JB08 thermospheric density model
- **Coordinate Transforms**: ECI, ECEF, geodetic conversions

## Orbital Elements

### Classical Keplerian Elements

| Element | Symbol | Range | Description |
|---------|--------|-------|-------------|
| Semi-major axis | a | > 0 | Size of orbit (m) |
| Eccentricity | e | [0, 1) | Shape (0=circular) |
| Inclination | i | [0, π] | Tilt relative to equator (rad) |
| RAAN | Ω | [0, 2π] | Orientation in equatorial plane (rad) |
| Arg. of perigee | ω | [0, 2π] | Orientation within orbital plane (rad) |
| True anomaly | ν | [0, 2π] | Current position in orbit (rad) |

### Common Orbit Types

| Orbit Type | Altitude (km) | Period | Inclination |
|------------|---------------|--------|-------------|
| LEO | 160-2000 | 90-120 min | Various |
| MEO | 2000-35786 | 2-24 hr | Various |
| GEO | 35786 | 24 hr | 0° |
| SSO | 400-800 | ~98 min | 97-99° |
| Polar | Various | Various | ~90° |

## Example: Creating a Satellite

```cpp
#include <jaguar/jaguar.h>

using namespace jaguar;

int main() {
    Engine engine;
    engine.initialize();

    // Create spacecraft entity
    EntityId satellite = engine.create_entity("GPS-IIR", Domain::Space);

    // Configure orbital elements (GPS constellation)
    domain::space::OrbitalElements orbit;
    orbit.semi_major_axis = 26560000.0;  // 26,560 km
    orbit.eccentricity = 0.0;
    orbit.inclination = 55.0 * constants::DEG_TO_RAD;
    orbit.raan = 0.0;
    orbit.arg_of_perigee = 0.0;
    orbit.true_anomaly = 0.0;

    // Convert to state vectors
    Vec3 pos_eci, vel_eci;
    orbit.to_state_vector(pos_eci, vel_eci);

    // Set initial state
    physics::EntityState state;
    state.position = pos_eci;
    state.velocity = vel_eci;
    state.mass = 1430.0;  // kg
    engine.set_entity_state(satellite, state);

    // Run simulation for one orbital period
    Real period = orbit.period();  // ~12 hours
    for (Real t = 0; t < period; t += 60.0) {
        engine.step(60.0);  // 1-minute steps

        auto current = engine.get_entity_state(satellite);
        Real alt = current.position.norm() - constants::EARTH_RADIUS;
        std::cout << "Altitude: " << alt / 1000.0 << " km\n";
    }

    engine.shutdown();
    return 0;
}
```

## SGP4 Propagator

### Two-Line Element (TLE) Format

```
ISS (ZARYA)
1 25544U 98067A   21275.52422453  .00001234  00000-0  28888-4 0  9990
2 25544  51.6442  27.4345 0003542 211.7212 148.3510 15.48919914305213
```

### Usage

```cpp
// Parse TLE
std::string line1 = "1 25544U 98067A   21275.52422453...";
std::string line2 = "2 25544  51.6442  27.4345 0003542...";
TLE tle = TLE::parse(line1, line2);

// Initialize propagator
SGP4Propagator sgp4;
sgp4.initialize(tle);

// Propagate forward 90 minutes
Vec3 pos_km, vel_kms;
sgp4.propagate(90.0, pos_km, vel_kms);

// Convert to SI units
Vec3 pos_m = pos_km * 1000.0;
Vec3 vel_ms = vel_kms * 1000.0;
```

### SGP4 vs SDP4

| Model | Period | Use Case |
|-------|--------|----------|
| SGP4 | < 225 min | Near-Earth satellites (LEO) |
| SDP4 | ≥ 225 min | Deep-space satellites (GEO, HEO) |

Selection is automatic based on mean motion.

## Gravity Model

### Fidelity Levels

| Level | Model | Accuracy | Performance |
|-------|-------|----------|-------------|
| 0 | Point mass | ~1 km at LEO | Fastest |
| 1 | J2 | ~100 m at LEO | Fast |
| 2 | J2-J4 | ~10 m at LEO | Moderate |
| 3 | EGM96/EGM2008 | ~1 m | Slowest |

### Zonal Harmonics

| Coefficient | Value | Effect |
|-------------|-------|--------|
| J2 | 1.08263 × 10⁻³ | Earth oblateness (dominant) |
| J3 | -2.532 × 10⁻⁶ | North-south asymmetry |
| J4 | -1.6109 × 10⁻⁶ | Higher-order oblateness |

### Usage

```cpp
GravityModel gravity;
gravity.set_fidelity(1);  // J2 perturbation

// Compute gravity acceleration
Vec3 g = gravity.compute_acceleration(position_ecef);
```

## Atmospheric Drag

### JB08 Atmosphere Model

Density depends on:
- Altitude
- Solar flux (F10.7)
- Geomagnetic activity (Ap)
- Day of year, local time

### Space Weather Parameters

| Parameter | Description | Typical Range |
|-----------|-------------|---------------|
| F10.7 | Solar radio flux | 70-300 sfu |
| F10.7 avg | 81-day average | 70-250 sfu |
| Ap | Geomagnetic index | 0-400 |

### Density vs Altitude

| Altitude (km) | Quiet Sun (kg/m³) | Active Sun (kg/m³) |
|---------------|-------------------|-------------------|
| 200 | ~3 × 10⁻¹⁰ | ~1 × 10⁻⁹ |
| 400 | ~3 × 10⁻¹² | ~3 × 10⁻¹¹ |
| 600 | ~1 × 10⁻¹³ | ~3 × 10⁻¹² |
| 800 | ~1 × 10⁻¹⁴ | ~5 × 10⁻¹³ |

### Drag Force

```
F_drag = -½ × ρ × Cd × A × V² × v̂
```

### Usage

```cpp
JB08AtmosphereModel atm;
atm.set_space_weather(150.0, 145.0, 15.0);  // Moderate activity

AtmosphericDragModel drag;
drag.set_cd(2.2);
drag.set_area(10.0);  // m²

// Compute drag force
drag.compute_forces(state, env, dt, forces);
```

## Coordinate Transformations

### Supported Frames

| Frame | Origin | Axes | Use Case |
|-------|--------|------|----------|
| ECI J2000 | Earth center | Inertial | Orbit propagation |
| ECEF | Earth center | Earth-fixed | Ground track |
| Geodetic | Earth surface | lat/lon/alt | Ground stations |

### Conversions

```cpp
using namespace transforms;

// ECI to ECEF
Vec3 pos_ecef = eci_to_ecef(pos_eci, julian_date);

// ECEF to geodetic
Real lat, lon, alt;
ecef_to_geodetic(pos_ecef, lat, lon, alt);

// Ground station to ECEF
Vec3 station = geodetic_to_ecef(
    37.0 * DEG_TO_RAD,   // Latitude
    -122.0 * DEG_TO_RAD, // Longitude
    100.0                // Altitude (m)
);
```

## Spacecraft Configuration

```xml
<entity type="spacecraft" name="GPS-IIR">
    <mass_balance>
        <dry_mass unit="kg">1080</dry_mass>
        <fuel_mass unit="kg">350</fuel_mass>
    </mass_balance>

    <aerodynamics>
        <drag_coefficient>2.2</drag_coefficient>
        <cross_section unit="m2">10.0</cross_section>
    </aerodynamics>

    <srp>
        <reflectivity>0.8</reflectivity>
        <area unit="m2">15.0</area>
    </srp>

    <orbit>
        <semi_major_axis unit="km">26560</semi_major_axis>
        <eccentricity>0.0</eccentricity>
        <inclination unit="deg">55.0</inclination>
    </orbit>
</entity>
```

## Performance Guidelines

| Application | Integration Step | Notes |
|-------------|------------------|-------|
| SGP4 | Direct use | Pre-integrated analytical |
| Numerical (LEO) | 10-60 seconds | RK4 or higher |
| High fidelity | Adaptive | Variable step-size |

## See Also

- [Air Domain](air.md) - Aircraft simulation
- [Land Domain](land.md) - Ground vehicle simulation
- [Sea Domain](sea.md) - Naval vessel simulation
- [Examples](../tutorials/examples.md) - Complete code examples
