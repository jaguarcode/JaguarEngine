# Space Domain Tutorial

This tutorial demonstrates how to simulate satellites using JaguarEngine's Space domain.

## Prerequisites

- JaguarEngine installed and configured
- Basic understanding of orbital mechanics
- C++ development environment

## Tutorial: Satellite Simulation

### Step 1: Create the Engine

```cpp
#include <jaguar/jaguar.h>

using namespace jaguar;

int main() {
    Engine engine;
    if (!engine.initialize()) {
        std::cerr << "Failed to initialize engine\n";
        return 1;
    }
```

### Step 2: Create a Satellite Entity

```cpp
    // Create satellite entity
    EntityId satellite = engine.create_entity("ISS", Domain::Space);
```

### Step 3: Define the Orbit

```cpp
    // Define orbital elements (ISS-like orbit)
    domain::space::OrbitalElements orbit;
    orbit.semi_major_axis = constants::EARTH_RADIUS + 420000.0; // 420 km altitude
    orbit.eccentricity = 0.0001;        // Nearly circular
    orbit.inclination = 51.6 * constants::DEG_TO_RAD;
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
    state.mass = 420000.0;  // ISS mass (kg)

    // Inertia tensor
    state.inertia = Mat3x3::identity();
    state.inertia.data[0][0] = 1e8;
    state.inertia.data[1][1] = 1e8;
    state.inertia.data[2][2] = 1e8;

    engine.set_entity_state(satellite, state);
```

### Step 4: Create Physics Models

```cpp
    // Create gravity model (J2 perturbation)
    domain::space::GravityModel gravity;
    gravity.set_fidelity(1);  // J2 level

    // Create atmospheric drag model
    domain::space::AtmosphericDragModel drag;
    drag.set_cd(2.2);
    drag.set_area(2500.0);  // ISS cross-section (m²)

    // Create atmosphere model
    domain::space::JB08AtmosphereModel atm;
    atm.set_space_weather(150.0, 145.0, 15.0);  // Moderate solar activity
```

### Step 5: Run the Simulation

```cpp
    // Simulation parameters
    Real dt = 60.0;                    // 1 minute steps
    Real duration = orbit.period();   // One orbital period

    std::cout << "Time(min), Altitude(km), Lat(deg), Lon(deg)\n";

    for (Real t = 0; t < duration; t += dt) {
        auto state = engine.get_entity_state(satellite);

        // Build environment
        environment::Environment env;
        env.altitude = state.position.norm() - constants::EARTH_RADIUS;

        // Compute forces
        physics::EntityForces forces;
        forces.clear();

        gravity.compute_forces(state, env, dt, forces);

        // Apply drag only in lower atmosphere
        if (env.altitude < 800000.0) {
            drag.compute_forces(state, env, dt, forces);
        }

        // Apply forces and step
        engine.apply_forces(satellite, forces);
        engine.step(dt);

        // Output telemetry every 5 minutes
        if (std::fmod(t, 300.0) < dt) {
            state = engine.get_entity_state(satellite);

            // Convert to geodetic
            using namespace transforms;
            Vec3 pos_ecef = eci_to_ecef(state.position, t);
            Real lat, lon, alt;
            ecef_to_geodetic(pos_ecef, lat, lon, alt);

            std::cout << t / 60.0 << ", "
                      << alt / 1000.0 << ", "
                      << lat * constants::RAD_TO_DEG << ", "
                      << lon * constants::RAD_TO_DEG << "\n";
        }
    }

    engine.shutdown();
    return 0;
}
```

## Tutorial: TLE Propagation

### Using SGP4 with TLE Data

```cpp
// Parse TLE
std::string line1 = "1 25544U 98067A   21275.52422453  .00001234  00000-0  28888-4 0  9990";
std::string line2 = "2 25544  51.6442  27.4345 0003542 211.7212 148.3510 15.48919914305213";

domain::space::TLE tle;
if (!tle.parse(line1, line2)) {
    std::cerr << "Failed to parse TLE\n";
    return 1;
}

// Initialize SGP4 propagator
domain::space::SGP4Propagator sgp4;
if (!sgp4.initialize(tle)) {
    std::cerr << "Failed to initialize SGP4\n";
    return 1;
}

// Propagate for 24 hours
for (Real minutes = 0; minutes < 24 * 60; minutes += 1.0) {
    Vec3 pos_km, vel_kms;
    sgp4.propagate(minutes, pos_km, vel_kms);

    // Convert to SI
    Vec3 pos_m = pos_km * 1000.0;
    Vec3 vel_ms = vel_kms * 1000.0;

    // Use position/velocity...
}
```

## Tutorial: Ground Station Visibility

### Computing Satellite Passes

```cpp
struct Pass {
    Real start_time;
    Real end_time;
    Real max_elevation;
};

std::vector<Pass> find_passes(
    domain::space::SGP4Propagator& sgp4,
    const Vec3& station_ecef,
    Real start_jd,
    Real duration_days,
    Real min_elevation_deg)
{
    std::vector<Pass> passes;
    Real min_elev_rad = min_elevation_deg * constants::DEG_TO_RAD;

    Real dt = 1.0;  // 1 minute steps
    bool in_pass = false;
    Pass current_pass;

    for (Real minutes = 0; minutes < duration_days * 24 * 60; minutes += dt) {
        // Get satellite position
        Vec3 pos_km, vel_kms;
        sgp4.propagate(minutes, pos_km, vel_kms);
        Vec3 pos_eci = pos_km * 1000.0;

        // Convert to ECEF
        Real jd = start_jd + minutes / (24 * 60);
        Vec3 sat_ecef = transforms::eci_to_ecef(pos_eci, jd);

        // Compute elevation angle
        Vec3 range = sat_ecef - station_ecef;
        Vec3 up = station_ecef.normalized();
        Real elevation = std::asin(range.dot(up) / range.norm());

        if (elevation > min_elev_rad) {
            if (!in_pass) {
                // Start of pass
                in_pass = true;
                current_pass.start_time = minutes;
                current_pass.max_elevation = elevation;
            } else {
                // During pass
                if (elevation > current_pass.max_elevation) {
                    current_pass.max_elevation = elevation;
                }
            }
        } else {
            if (in_pass) {
                // End of pass
                current_pass.end_time = minutes;
                passes.push_back(current_pass);
                in_pass = false;
            }
        }
    }

    return passes;
}
```

## Tutorial: Constellation Simulation

### GPS Constellation

```cpp
// Create GPS constellation (24 satellites, 6 orbital planes)
std::vector<EntityId> gps_satellites;

for (int plane = 0; plane < 6; ++plane) {
    for (int slot = 0; slot < 4; ++slot) {
        std::string name = "GPS-" + std::to_string(plane * 4 + slot + 1);
        EntityId sat = engine.create_entity(name, Domain::Space);

        // GPS orbit parameters
        domain::space::OrbitalElements orbit;
        orbit.semi_major_axis = 26560000.0;  // GPS altitude
        orbit.eccentricity = 0.0;
        orbit.inclination = 55.0 * DEG_TO_RAD;
        orbit.raan = plane * 60.0 * DEG_TO_RAD;  // 60° separation
        orbit.arg_of_perigee = 0.0;
        orbit.true_anomaly = slot * 90.0 * DEG_TO_RAD;  // 90° spacing

        Vec3 pos, vel;
        orbit.to_state_vector(pos, vel);

        physics::EntityState state;
        state.position = pos;
        state.velocity = vel;
        state.mass = 1080.0;  // GPS Block IIR mass
        engine.set_entity_state(sat, state);

        gps_satellites.push_back(sat);
    }
}
```

## XML Configuration

### Spacecraft Definition

```xml
<?xml version="1.0" encoding="UTF-8"?>
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

## Common Issues

### TLE Accuracy

**Problem:** Position errors grow over time

**Solution:** TLE accuracy degrades; use recent TLEs (< 1-2 weeks old)

### Atmospheric Drag Variations

**Problem:** Orbit decay rate varies unexpectedly

**Solution:** Update space weather parameters (F10.7, Ap)

## Next Steps

- [Air Domain Tutorial](air-domain.md) - Aircraft simulation
- [Land Domain Tutorial](land-domain.md) - Ground vehicle simulation
- [Sea Domain Tutorial](sea-domain.md) - Naval simulation
- [API Reference](../api/space.md) - Space domain API
