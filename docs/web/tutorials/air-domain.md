# Air Domain Tutorial

This tutorial demonstrates how to simulate aircraft using JaguarEngine's Air domain.

## Prerequisites

- JaguarEngine installed and configured
- Basic understanding of flight dynamics
- C++ development environment

## Tutorial: Simple Aircraft Simulation

### Step 1: Create the Engine

```cpp
#include <jaguar/jaguar.h>

using namespace jaguar;

int main() {
    // Create and initialize engine
    Engine engine;
    if (!engine.initialize()) {
        std::cerr << "Failed to initialize engine\n";
        return 1;
    }
```

### Step 2: Create an Aircraft Entity

```cpp
    // Create aircraft entity
    EntityId aircraft = engine.create_entity("F16", Domain::Air);
```

### Step 3: Set Initial State

```cpp
    // Configure initial state
    physics::EntityState state;

    // Position: 10 km altitude (NED, so Z is negative up)
    state.position = Vec3{0.0, 0.0, -10000.0};

    // Velocity: 250 m/s forward (North in NED)
    state.velocity = Vec3{250.0, 0.0, 0.0};

    // Orientation: Level flight, heading North
    state.orientation = Quaternion::identity();

    // Mass properties
    state.mass = 12000.0;  // 12,000 kg

    // Inertia tensor (simplified)
    state.inertia = Mat3x3::identity();
    state.inertia.data[0][0] = 12875;   // Ixx
    state.inertia.data[1][1] = 75674;   // Iyy
    state.inertia.data[2][2] = 85552;   // Izz

    engine.set_entity_state(aircraft, state);
```

### Step 4: Create Physics Models

```cpp
    // Create aerodynamics model
    domain::air::AerodynamicsModel aero;
    aero.set_reference_area(27.87);    // Wing area (m²)
    aero.set_reference_chord(3.45);    // Mean chord (m)
    aero.set_reference_span(9.45);     // Wingspan (m)

    // Create propulsion model
    domain::air::PropulsionModel propulsion;
    propulsion.set_max_thrust(130000.0);  // 130 kN
    propulsion.set_fuel_capacity(3200.0); // 3200 kg
    propulsion.set_specific_fuel_consumption(2.5e-5);
    propulsion.start();
    propulsion.set_throttle(0.7);  // 70% throttle
```

### Step 5: Run the Simulation

```cpp
    // Simulation parameters
    Real dt = 0.01;       // 100 Hz
    Real duration = 60.0; // 1 minute

    std::cout << "Time(s), Altitude(m), Airspeed(m/s), Alpha(deg)\n";

    for (Real t = 0; t < duration; t += dt) {
        // Get current state and environment
        auto state = engine.get_entity_state(aircraft);
        auto env = engine.get_environment(aircraft);

        // Compute forces
        physics::EntityForces forces;
        forces.clear();

        aero.compute_forces(state, env, dt, forces);
        propulsion.compute_forces(state, env, dt, forces);

        // Add gravity
        forces.add_force(Vec3{0.0, 0.0, state.mass * constants::G0});

        // Apply forces and step
        engine.apply_forces(aircraft, forces);
        engine.step(dt);

        // Output telemetry every second
        if (std::fmod(t, 1.0) < dt) {
            Real altitude = -state.position.z;
            Real airspeed = state.velocity.norm();
            Real alpha = aero.get_alpha() * constants::RAD_TO_DEG;

            std::cout << t << ", "
                      << altitude << ", "
                      << airspeed << ", "
                      << alpha << "\n";
        }
    }

    engine.shutdown();
    return 0;
}
```

## Tutorial: Aircraft with Flight Controls

### Adding Control Inputs

```cpp
// Create flight control system
domain::air::FlightControlSystem fcs;
fcs.set_elevator_range(-25.0, 25.0);
fcs.set_aileron_range(-20.0, 20.0);
fcs.set_rudder_range(-30.0, 30.0);

// Define control inputs
domain::air::FlightControlSystem::ControlInputs inputs;
inputs.pitch_cmd = 0.0;      // Neutral pitch
inputs.roll_cmd = 0.0;       // Wings level
inputs.yaw_cmd = 0.0;        // No rudder
inputs.throttle_cmd = 0.7;   // 70% throttle

// In simulation loop
auto surfaces = fcs.process(inputs, dt);

// Apply control surface effects to aerodynamics
// (Control surfaces modify aerodynamic coefficients)
```

### Implementing a Simple Autopilot

```cpp
class SimpleAutopilot {
public:
    void set_altitude_target(Real altitude) {
        target_alt_ = altitude;
    }

    void set_heading_target(Real heading) {
        target_hdg_ = heading;
    }

    FlightControlSystem::ControlInputs compute(
        const physics::EntityState& state,
        Real dt)
    {
        FlightControlSystem::ControlInputs inputs;

        // Altitude hold (pitch control)
        Real alt = -state.position.z;
        Real alt_error = target_alt_ - alt;
        Real climb_rate = -state.velocity.z;

        // PI controller for altitude
        alt_integral_ += alt_error * dt;
        inputs.pitch_cmd = kp_alt_ * alt_error +
                           ki_alt_ * alt_integral_ -
                           kd_alt_ * climb_rate;
        inputs.pitch_cmd = std::clamp(inputs.pitch_cmd, -1.0, 1.0);

        // Heading hold (roll control)
        Real roll, pitch, yaw;
        state.orientation.to_euler(roll, pitch, yaw);

        Real hdg_error = target_hdg_ - yaw;
        // Normalize to [-π, π]
        while (hdg_error > M_PI) hdg_error -= 2 * M_PI;
        while (hdg_error < -M_PI) hdg_error += 2 * M_PI;

        // Bank angle command
        Real bank_cmd = kp_hdg_ * hdg_error;
        bank_cmd = std::clamp(bank_cmd, -0.5, 0.5);  // Max 30° bank

        // Roll to achieve bank
        Real bank_error = bank_cmd - roll;
        inputs.roll_cmd = kp_roll_ * bank_error;
        inputs.roll_cmd = std::clamp(inputs.roll_cmd, -1.0, 1.0);

        return inputs;
    }

private:
    Real target_alt_{10000.0};
    Real target_hdg_{0.0};
    Real alt_integral_{0.0};

    // Gains
    Real kp_alt_{0.01};
    Real ki_alt_{0.001};
    Real kd_alt_{0.05};
    Real kp_hdg_{1.0};
    Real kp_roll_{2.0};
};
```

## Tutorial: Missile Simulation

### Creating a Guided Missile

```cpp
// Create missile entity
EntityId missile = engine.create_entity("AIM120", Domain::Air);

// Initial state (launched from aircraft)
physics::EntityState missile_state;
missile_state.position = aircraft_state.position;
missile_state.velocity = aircraft_state.velocity +
    aircraft_state.orientation.rotate(Vec3{50.0, 0.0, 0.0});
missile_state.orientation = aircraft_state.orientation;
missile_state.mass = 150.0;

// Missile-specific models
domain::air::AerodynamicsModel missile_aero;
missile_aero.set_reference_area(0.03);  // Small frontal area
missile_aero.set_reference_chord(0.2);
missile_aero.set_reference_span(0.5);

domain::air::PropulsionModel rocket;
rocket.set_max_thrust(12000.0);
rocket.set_fuel_capacity(30.0);
rocket.set_specific_fuel_consumption(1e-4);
rocket.start();
rocket.set_throttle(1.0);  // Full throttle
```

### Proportional Navigation Guidance

```cpp
class ProNavGuidance {
public:
    Vec3 compute_acceleration(const Vec3& missile_pos,
                              const Vec3& missile_vel,
                              const Vec3& target_pos,
                              const Vec3& target_vel) {
        // Line-of-sight vector
        Vec3 los = target_pos - missile_pos;
        Real range = los.norm();
        Vec3 los_unit = los / range;

        // Closing velocity
        Vec3 rel_vel = target_vel - missile_vel;
        Real closing_speed = -rel_vel.dot(los_unit);

        // LOS rate
        Vec3 omega_los = los.cross(rel_vel) / (range * range);

        // Pro-nav acceleration
        Real N = 4.0;  // Navigation constant
        Vec3 accel_cmd = missile_vel.cross(omega_los) * N;

        return accel_cmd;
    }
};
```

## XML Configuration

### Aircraft Definition

```xml
<?xml version="1.0" encoding="UTF-8"?>
<entity type="aircraft" name="F16">
    <metrics>
        <wingspan unit="m">9.45</wingspan>
        <wing_area unit="m2">27.87</wing_area>
        <wing_chord unit="m">3.45</wing_chord>
    </metrics>

    <mass_balance>
        <empty_weight unit="kg">8936</empty_weight>
        <fuel_capacity unit="kg">3200</fuel_capacity>
    </mass_balance>

    <propulsion>
        <engine type="turbofan">
            <max_thrust unit="N">130000</max_thrust>
        </engine>
    </propulsion>
</entity>
```

## Common Issues

### Numerical Instability

**Symptoms:** Aircraft diverges, oscillates wildly, or crashes

**Solutions:**
- Reduce time step (try 0.005s instead of 0.01s)
- Check coefficient table bounds
- Verify mass and inertia are positive

### Unrealistic Flight

**Symptoms:** Aircraft doesn't behave physically

**Solutions:**
- Verify aerodynamic coefficients match aircraft type
- Check reference dimensions (area, chord, span)
- Ensure control surface limits are reasonable

## Next Steps

- [Land Domain Tutorial](land-domain.md) - Ground vehicle simulation
- [Sea Domain Tutorial](sea-domain.md) - Naval simulation
- [Multi-Entity Tutorial](multi-entity.md) - Multiple aircraft simulation
- [API Reference](../api/air.md) - Air domain API

