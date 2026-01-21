# Sea Domain Tutorial

This tutorial demonstrates how to simulate naval vessels using JaguarEngine's Sea domain.

## Prerequisites

- JaguarEngine installed and configured
- Basic understanding of ship dynamics
- C++ development environment

## Tutorial: Surface Ship Simulation

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

### Step 2: Create a Ship Entity

```cpp
    // Create destroyer entity
    EntityId ship = engine.create_entity("DDG51", Domain::Sea);
```

### Step 3: Set Initial State

```cpp
    // Configure initial state
    physics::EntityState state;

    // Position: At sea surface
    state.position = Vec3{0.0, 0.0, 0.0};

    // Velocity: 15 knots (7.7 m/s) heading North
    state.velocity = Vec3{7.7, 0.0, 0.0};

    // Orientation: Level, heading North
    state.orientation = Quaternion::identity();

    // Mass: 8,600 tonnes displacement
    state.mass = 8600000.0;

    // Inertia (simplified)
    state.inertia = Mat3x3::identity();
    state.inertia.data[0][0] = 1e10;   // Ixx (roll)
    state.inertia.data[1][1] = 5e10;   // Iyy (pitch)
    state.inertia.data[2][2] = 5e10;   // Izz (yaw)

    engine.set_entity_state(ship, state);
```

### Step 4: Create Physics Models

```cpp
    // Create buoyancy model
    domain::sea::BuoyancyModel buoyancy;
    buoyancy.set_displaced_volume(8390.0);      // m³ (at full load)
    buoyancy.set_metacentric_height(2.5);       // GM = 2.5 m
    buoyancy.set_center_of_buoyancy(Vec3{0.0, 0.0, -4.7});  // Below CG

    // Create hydrodynamics model (MMG)
    domain::sea::HydrodynamicsModel hydro;
    hydro.set_hull_coefficients(
        -0.04,  // X_vv
        -0.01,  // X_rr
        -0.4,   // Y_v
        0.05,   // Y_r
        -0.1,   // N_v
        -0.05   // N_r
    );
    hydro.set_rudder_parameters(18.0, 1.6);     // Area (m²), aspect ratio
    hydro.set_propeller_parameters(5.2, 1.0);   // Diameter (m), pitch ratio

    // Create wave model
    domain::sea::WaveModel waves;
    waves.set_sea_state(domain::sea::SeaState::FromNATOSeaState(4));

    // Create RAO model for ship motion response
    domain::sea::RAOModel rao;
    // Set heave RAO
    rao.set_rao(2,  // DOF 2 = heave
        {0.3, 0.5, 0.7, 1.0, 1.5},  // frequencies (rad/s)
        {0.9, 1.0, 0.95, 0.7, 0.4}, // amplitudes
        {0.0, 0.1, 0.2, 0.4, 0.6}); // phases (rad)
```

### Step 5: Run the Simulation

```cpp
    // Simulation parameters
    Real dt = 0.05;       // 20 Hz
    Real duration = 120.0; // 2 minutes
    Real rudder_cmd = 0.0;
    Real rpm_cmd = 100.0;

    std::cout << "Time(s), Speed(kts), Heel(deg), Draft(m)\n";

    for (Real t = 0; t < duration; t += dt) {
        auto state = engine.get_entity_state(ship);

        // Build environment with wave data
        environment::Environment env = engine.get_environment(ship);
        env.over_water = true;
        env.ocean.surface_elevation = waves.get_elevation(
            state.position.x, state.position.y, t);

        // Set controls
        hydro.set_rudder_angle(rudder_cmd * 0.52);  // Max 30°
        hydro.set_propeller_rpm(rpm_cmd);

        // Compute forces
        physics::EntityForces forces;
        forces.clear();

        buoyancy.compute_forces(state, env, dt, forces);
        hydro.compute_forces(state, env, dt, forces);

        // Add wave-induced motions via RAO
        Vec3 wave_disp, wave_rot;
        rao.calculate_response(waves, t, wave_disp, wave_rot);

        // Gravity
        forces.add_force(Vec3{0.0, 0.0, state.mass * constants::G0});

        // Apply forces and step
        engine.apply_forces(ship, forces);
        engine.step(dt);

        // Output telemetry every 5 seconds
        if (std::fmod(t, 5.0) < dt) {
            Real speed_kts = state.velocity.norm() * 1.944;
            Real heel_deg = buoyancy.get_heel() * constants::RAD_TO_DEG;
            Real draft = buoyancy.get_draft();

            std::cout << t << ", "
                      << speed_kts << ", "
                      << heel_deg << ", "
                      << draft << "\n";
        }
    }

    engine.shutdown();
    return 0;
}
```

## Tutorial: Maneuvering

### Turning Circle Test

```cpp
// Turning circle maneuver
void turning_circle_test(Engine& engine, EntityId ship,
                         domain::sea::HydrodynamicsModel& hydro) {
    Real dt = 0.05;
    Real duration = 600.0;  // 10 minutes

    // Initial straight run
    hydro.set_rudder_angle(0.0);
    for (Real t = 0; t < 30.0; t += dt) {
        run_step(engine, ship, hydro, dt);
    }

    // Apply rudder
    Real rudder_deg = 35.0;  // 35° rudder
    hydro.set_rudder_angle(rudder_deg * constants::DEG_TO_RAD);

    // Record trajectory
    std::vector<Vec3> trajectory;
    Vec3 initial_pos = engine.get_entity_state(ship).position;

    for (Real t = 0; t < duration; t += dt) {
        run_step(engine, ship, hydro, dt);

        if (std::fmod(t, 1.0) < dt) {
            Vec3 pos = engine.get_entity_state(ship).position;
            trajectory.push_back(pos - initial_pos);
        }
    }

    // Analyze turning circle
    // - Advance: Forward distance at 90° heading change
    // - Transfer: Lateral distance at 90° heading change
    // - Tactical diameter: Lateral distance at 180° heading change
    // - Turning radius: Radius of steady turn
}
```

### Zig-Zag Maneuver

```cpp
void zigzag_test(Engine& engine, EntityId ship,
                 domain::sea::HydrodynamicsModel& hydro,
                 Real rudder_angle_deg,
                 Real heading_change_deg) {
    Real dt = 0.05;
    Real initial_heading = get_heading(engine.get_entity_state(ship));
    Real target_heading = initial_heading;
    int direction = 1;  // 1 = starboard, -1 = port

    while (/* test not complete */) {
        auto state = engine.get_entity_state(ship);
        Real current_heading = get_heading(state);

        // Check if heading change threshold reached
        Real heading_error = current_heading - target_heading;
        normalize_angle(heading_error);

        if (direction * heading_error > heading_change_deg * DEG_TO_RAD) {
            // Reverse rudder
            direction *= -1;
            target_heading = initial_heading + direction * heading_change_deg * DEG_TO_RAD;
        }

        hydro.set_rudder_angle(direction * rudder_angle_deg * DEG_TO_RAD);
        run_step(engine, ship, hydro, dt);
    }
}
```

## Tutorial: Seakeeping

### Wave Response Analysis

```cpp
// Analyze ship motion in waves
void seakeeping_analysis(Engine& engine, EntityId ship,
                         domain::sea::WaveModel& waves,
                         domain::sea::RAOModel& rao) {
    Real dt = 0.05;
    Real duration = 300.0;  // 5 minutes

    // Data collection
    std::vector<Real> heave_data, roll_data, pitch_data;

    for (Real t = 0; t < duration; t += dt) {
        auto state = engine.get_entity_state(ship);

        // Get wave-induced motions
        Vec3 disp, rot;
        rao.calculate_response(waves, t, disp, rot);

        heave_data.push_back(disp.z);
        roll_data.push_back(rot.x);
        pitch_data.push_back(rot.y);

        engine.step(dt);
    }

    // Statistical analysis
    Real heave_rms = compute_rms(heave_data);
    Real roll_rms = compute_rms(roll_data);
    Real pitch_rms = compute_rms(pitch_data);

    std::cout << "Heave RMS: " << heave_rms << " m\n";
    std::cout << "Roll RMS: " << roll_rms * RAD_TO_DEG << " deg\n";
    std::cout << "Pitch RMS: " << pitch_rms * RAD_TO_DEG << " deg\n";
}
```

## XML Configuration

### Ship Definition

```xml
<?xml version="1.0" encoding="UTF-8"?>
<entity type="surface_ship" name="DDG51">
    <metrics>
        <length unit="m">154.0</length>
        <beam unit="m">20.0</beam>
        <draft unit="m">9.4</draft>
    </metrics>

    <mass_balance>
        <displacement unit="tonnes">8600</displacement>
        <metacentric_height unit="m">2.5</metacentric_height>
    </mass_balance>

    <hydrodynamics>
        <hull_coefficients>
            <x_vv>-0.04</x_vv>
            <y_v>-0.4</y_v>
            <n_v>-0.1</n_v>
            <n_r>-0.05</n_r>
        </hull_coefficients>

        <rudder>
            <area unit="m2">18.0</area>
            <max_angle unit="deg">35</max_angle>
        </rudder>

        <propeller>
            <diameter unit="m">5.2</diameter>
            <max_rpm>180</max_rpm>
        </propeller>
    </hydrodynamics>
</entity>
```

## Common Issues

### Capsizing

**Symptoms:** Ship rolls over

**Solutions:**
- Increase metacentric height (GM)
- Check displacement volume
- Verify mass distribution

### Unrealistic Speed

**Symptoms:** Ship accelerates too fast/slow

**Solutions:**
- Check propeller parameters
- Verify hull resistance coefficients
- Check thrust deduction factor

## Next Steps

- [Air Domain Tutorial](air-domain.md) - Aircraft simulation
- [Land Domain Tutorial](land-domain.md) - Ground vehicle simulation
- [Multi-Entity Tutorial](multi-entity.md) - Multiple ship simulation
- [API Reference](../api/sea.md) - Sea domain API
