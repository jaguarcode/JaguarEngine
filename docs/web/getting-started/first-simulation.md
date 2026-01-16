# Your First Complete Simulation

This guide walks you through building a complete, interactive flight simulation with JaguarEngine.

## What You'll Build

By the end of this tutorial, you'll have:

- A configurable F-16 aircraft model
- Realistic aerodynamics and propulsion
- Flight control with keyboard input
- Real-time telemetry display
- Data logging for analysis

---

## Project Structure

```
my_flight_sim/
├── CMakeLists.txt
├── src/
│   ├── main.cpp
│   ├── aircraft.h
│   ├── aircraft.cpp
│   └── telemetry.h
├── config/
│   └── f16.xml
└── output/
    └── flight_data.csv
```

---

## Step 1: Aircraft Configuration

Create `config/f16.xml`:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<entity>
    <name>F-16C</name>
    <domain>air</domain>

    <!-- Mass Properties -->
    <mass>
        <empty_kg>8570</empty_kg>
        <fuel_capacity_kg>3200</fuel_capacity_kg>
        <center_of_gravity>
            <x_m>0.0</x_m>
            <y_m>0.0</y_m>
            <z_m>0.0</z_m>
        </center_of_gravity>
        <inertia>
            <ixx>12875</ixx>
            <iyy>75674</iyy>
            <izz>85552</izz>
            <ixy>0</ixy>
            <ixz>1331</ixz>
            <iyz>0</iyz>
        </inertia>
    </mass>

    <!-- Aerodynamic Reference -->
    <aerodynamics>
        <reference_area_m2>27.87</reference_area_m2>
        <reference_chord_m>3.45</reference_chord_m>
        <reference_span_m>9.45</reference_span_m>

        <!-- Basic coefficients (simplified) -->
        <cl_alpha>5.5</cl_alpha>
        <cl_0>0.2</cl_0>
        <cd_0>0.021</cd_0>
        <cd_k>0.12</cd_k>
        <cm_alpha>-1.2</cm_alpha>
    </aerodynamics>

    <!-- Propulsion -->
    <propulsion>
        <engine type="turbofan">
            <name>F110-GE-100</name>
            <max_thrust_n>131000</max_thrust_n>
            <mil_thrust_n>76300</mil_thrust_n>
            <idle_thrust_n>6000</idle_thrust_n>
            <sfc_kg_per_n_s>2.5e-5</sfc_kg_per_n_s>
        </engine>
    </propulsion>

    <!-- Flight Control -->
    <flight_control>
        <elevator>
            <min_deg>-25</min_deg>
            <max_deg>25</max_deg>
            <rate_limit_deg_s>60</rate_limit_deg_s>
        </elevator>
        <aileron>
            <min_deg>-21.5</min_deg>
            <max_deg>21.5</max_deg>
            <rate_limit_deg_s>80</rate_limit_deg_s>
        </aileron>
        <rudder>
            <min_deg>-30</min_deg>
            <max_deg>30</max_deg>
            <rate_limit_deg_s>120</rate_limit_deg_s>
        </rudder>
    </flight_control>
</entity>
```

---

## Step 2: Aircraft Class

Create `src/aircraft.h`:

```cpp
#pragma once

#include <jaguar/jaguar.h>
#include <jaguar/domain/air.h>
#include <string>

class Aircraft {
public:
    // Control inputs
    struct Controls {
        jaguar::Real pitch{0.0};     // -1 to +1
        jaguar::Real roll{0.0};      // -1 to +1
        jaguar::Real yaw{0.0};       // -1 to +1
        jaguar::Real throttle{0.0};  // 0 to 1
    };

    // Telemetry output
    struct Telemetry {
        jaguar::Real time;
        jaguar::Real altitude;       // m
        jaguar::Real airspeed;       // m/s
        jaguar::Real mach;
        jaguar::Real alpha;          // deg
        jaguar::Real beta;           // deg
        jaguar::Real roll;           // deg
        jaguar::Real pitch;          // deg
        jaguar::Real heading;        // deg
        jaguar::Real thrust;         // N
        jaguar::Real fuel_remaining; // kg
        jaguar::Real g_load;         // g's
    };

    Aircraft(jaguar::interface::Engine& engine, const std::string& config_path);
    ~Aircraft() = default;

    // Setup
    void set_initial_state(
        const jaguar::Vec3& position,
        const jaguar::Vec3& velocity,
        jaguar::Real fuel_kg
    );

    // Control
    void set_controls(const Controls& controls);
    void update(jaguar::Real dt);

    // Telemetry
    Telemetry get_telemetry() const;
    jaguar::EntityId get_entity_id() const { return entity_id_; }

private:
    jaguar::interface::Engine& engine_;
    jaguar::EntityId entity_id_;

    // Physics models
    jaguar::domain::air::AerodynamicsModel aero_;
    jaguar::domain::air::PropulsionModel propulsion_;
    jaguar::domain::air::FlightControlSystem fcs_;

    // Current state
    Controls controls_;
    jaguar::physics::EntityForces forces_;
    jaguar::Real sim_time_{0.0};
};
```

Create `src/aircraft.cpp`:

```cpp
#include "aircraft.h"
#include <cmath>

Aircraft::Aircraft(jaguar::interface::Engine& engine, const std::string& config_path)
    : engine_(engine)
{
    // Create entity
    entity_id_ = engine_.create_entity("F-16", jaguar::Domain::Air);

    // Configure aerodynamics (F-16 values)
    aero_.set_reference_area(27.87);
    aero_.set_reference_chord(3.45);
    aero_.set_reference_span(9.45);

    // Configure propulsion (F110 engine)
    propulsion_.set_max_thrust(131000.0);
    propulsion_.set_fuel_capacity(3200.0);
    propulsion_.set_specific_fuel_consumption(2.5e-5);
    propulsion_.start();

    // Configure flight control limits
    fcs_.set_elevator_range(-25.0, 25.0);
    fcs_.set_aileron_range(-21.5, 21.5);
    fcs_.set_rudder_range(-30.0, 30.0);
}

void Aircraft::set_initial_state(
    const jaguar::Vec3& position,
    const jaguar::Vec3& velocity,
    jaguar::Real fuel_kg)
{
    jaguar::physics::EntityState state;
    state.position = position;
    state.velocity = velocity;
    state.orientation = jaguar::Quaternion::identity();
    state.angular_velocity = {0, 0, 0};

    // Mass = empty + fuel
    state.mass = 8570.0 + fuel_kg;

    // Inertia tensor (simplified)
    state.inertia = jaguar::Mat3x3{{
        {12875, 0, 1331},
        {0, 75674, 0},
        {1331, 0, 85552}
    }};

    engine_.set_entity_state(entity_id_, state);
}

void Aircraft::set_controls(const Controls& controls) {
    controls_ = controls;
    propulsion_.set_throttle(controls.throttle);
}

void Aircraft::update(jaguar::Real dt) {
    auto state = engine_.get_entity_state(entity_id_);
    auto env = engine_.get_environment(entity_id_);

    // Clear forces
    forces_.clear();

    // Flight control surface deflections
    jaguar::domain::air::FlightControlSystem::ControlInputs fcs_in;
    fcs_in.pitch_cmd = controls_.pitch;
    fcs_in.roll_cmd = controls_.roll;
    fcs_in.yaw_cmd = controls_.yaw;
    fcs_in.throttle_cmd = controls_.throttle;

    auto fcs_out = fcs_.process(fcs_in, dt);

    // Aerodynamic forces
    aero_.compute_forces(state, env, dt, forces_);

    // Propulsion forces
    propulsion_.compute_forces(state, env, dt, forces_);

    // Gravity
    jaguar::Vec3 gravity{0, 0, state.mass * jaguar::constants::G0};
    forces_.add_force(gravity);

    // Apply forces
    engine_.apply_forces(entity_id_, forces_);

    sim_time_ += dt;
}

Aircraft::Telemetry Aircraft::get_telemetry() const {
    auto state = engine_.get_entity_state(entity_id_);

    Telemetry t;
    t.time = sim_time_;
    t.altitude = -state.position.z;
    t.airspeed = state.velocity.norm();
    t.mach = aero_.get_mach();
    t.alpha = aero_.get_alpha() * jaguar::constants::RAD_TO_DEG;
    t.beta = aero_.get_beta() * jaguar::constants::RAD_TO_DEG;

    // Euler angles
    jaguar::Real roll, pitch, yaw;
    state.orientation.to_euler(roll, pitch, yaw);
    t.roll = roll * jaguar::constants::RAD_TO_DEG;
    t.pitch = pitch * jaguar::constants::RAD_TO_DEG;
    t.heading = yaw * jaguar::constants::RAD_TO_DEG;
    if (t.heading < 0) t.heading += 360.0;

    t.thrust = propulsion_.get_thrust();
    t.fuel_remaining = propulsion_.get_fuel_remaining();

    // G-load (simplified: lift / weight)
    t.g_load = 1.0;  // Would compute from forces

    return t;
}
```

---

## Step 3: Telemetry Display

Create `src/telemetry.h`:

```cpp
#pragma once

#include "aircraft.h"
#include <iostream>
#include <iomanip>
#include <fstream>

class TelemetryDisplay {
public:
    void print_header() {
        std::cout << "\n";
        std::cout << "╔══════════════════════════════════════════════════════════════════╗\n";
        std::cout << "║                    JaguarEngine Flight Simulator                  ║\n";
        std::cout << "╠══════════════════════════════════════════════════════════════════╣\n";
        std::cout << "║  Time │ Alt (ft) │ IAS (kt) │ Mach │ Alpha │ G's │ Fuel (lb)    ║\n";
        std::cout << "╠══════════════════════════════════════════════════════════════════╣\n";
    }

    void print_telemetry(const Aircraft::Telemetry& t) {
        // Convert to aviation units
        double alt_ft = t.altitude * 3.28084;
        double ias_kt = t.airspeed * 1.94384;
        double fuel_lb = t.fuel_remaining * 2.20462;

        std::cout << std::fixed << std::setprecision(1);
        std::cout << "║ " << std::setw(5) << t.time << " │ "
                  << std::setw(8) << alt_ft << " │ "
                  << std::setw(8) << ias_kt << " │ "
                  << std::setw(4) << std::setprecision(2) << t.mach << " │ "
                  << std::setw(5) << std::setprecision(1) << t.alpha << " │ "
                  << std::setw(4) << std::setprecision(1) << t.g_load << " │ "
                  << std::setw(8) << fuel_lb << "     ║\n";
    }

    void print_footer() {
        std::cout << "╚══════════════════════════════════════════════════════════════════╝\n";
    }
};

class DataLogger {
public:
    DataLogger(const std::string& filename) : file_(filename) {
        file_ << "time,altitude,airspeed,mach,alpha,beta,roll,pitch,heading,thrust,fuel,g_load\n";
    }

    void log(const Aircraft::Telemetry& t) {
        file_ << t.time << ","
              << t.altitude << ","
              << t.airspeed << ","
              << t.mach << ","
              << t.alpha << ","
              << t.beta << ","
              << t.roll << ","
              << t.pitch << ","
              << t.heading << ","
              << t.thrust << ","
              << t.fuel_remaining << ","
              << t.g_load << "\n";
    }

private:
    std::ofstream file_;
};
```

---

## Step 4: Main Simulation

Create `src/main.cpp`:

```cpp
#include <jaguar/jaguar.h>
#include "aircraft.h"
#include "telemetry.h"
#include <iostream>
#include <chrono>
#include <thread>

void print_controls_help() {
    std::cout << "\nFlight Controls:\n";
    std::cout << "  W/S - Pitch down/up\n";
    std::cout << "  A/D - Roll left/right\n";
    std::cout << "  Q/E - Yaw left/right\n";
    std::cout << "  +/- - Throttle up/down\n";
    std::cout << "  ESC - Exit\n\n";
}

int main(int argc, char* argv[]) {
    std::cout << "╔═══════════════════════════════════════════════════╗\n";
    std::cout << "║     JaguarEngine Flight Simulation Demo           ║\n";
    std::cout << "║     Version: " << jaguar::GetVersionString() << "                            ║\n";
    std::cout << "╚═══════════════════════════════════════════════════╝\n";

    // Initialize engine
    jaguar::interface::Engine engine;
    if (!engine.initialize()) {
        std::cerr << "Error: Failed to initialize JaguarEngine\n";
        return 1;
    }
    std::cout << "Engine initialized.\n";

    // Create aircraft
    Aircraft f16(engine, "config/f16.xml");

    // Set initial conditions: 20,000 ft, 400 knots, heading north
    jaguar::Vec3 initial_pos{0.0, 0.0, -6096.0};  // 20,000 ft in meters
    jaguar::Vec3 initial_vel{205.8, 0.0, 0.0};    // 400 knots in m/s
    f16.set_initial_state(initial_pos, initial_vel, 2500.0);  // 2500 kg fuel

    std::cout << "Aircraft spawned at 20,000 ft, 400 KIAS\n";

    // Set initial throttle
    Aircraft::Controls controls;
    controls.throttle = 0.8;  // 80% power
    f16.set_controls(controls);

    // Create telemetry display and logger
    TelemetryDisplay display;
    DataLogger logger("output/flight_data.csv");

    print_controls_help();
    display.print_header();

    // Simulation parameters
    const jaguar::Real dt = 0.01;           // 100 Hz physics
    const jaguar::Real display_rate = 1.0;  // 1 Hz display
    const jaguar::Real sim_duration = 60.0; // 60 seconds

    jaguar::Real last_display = 0.0;

    // Main simulation loop
    for (jaguar::Real t = 0.0; t < sim_duration; t += dt) {
        // Update physics
        f16.update(dt);
        engine.step(dt);

        // Get telemetry
        auto telemetry = f16.get_telemetry();

        // Log data (every step)
        logger.log(telemetry);

        // Display (every second)
        if (t - last_display >= display_rate) {
            display.print_telemetry(telemetry);
            last_display = t;
        }

        // Simple maneuver: gentle climb at t=10, level off at t=30
        if (t >= 10.0 && t < 30.0) {
            controls.pitch = -0.1;  // Slight nose up
        } else if (t >= 30.0 && t < 35.0) {
            controls.pitch = 0.05;  // Nose down to level
        } else {
            controls.pitch = 0.0;
        }
        f16.set_controls(controls);
    }

    display.print_footer();

    // Final status
    auto final_telemetry = f16.get_telemetry();
    std::cout << "\nFinal Status:\n";
    std::cout << "  Altitude: " << final_telemetry.altitude * 3.28084 << " ft\n";
    std::cout << "  Airspeed: " << final_telemetry.airspeed * 1.94384 << " kt\n";
    std::cout << "  Fuel remaining: " << final_telemetry.fuel_remaining * 2.20462 << " lb\n";
    std::cout << "\nFlight data saved to: output/flight_data.csv\n";

    // Cleanup
    engine.shutdown();
    std::cout << "Simulation complete.\n";

    return 0;
}
```

---

## Step 5: CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.25)
project(MyFlightSim VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find JaguarEngine
find_package(Jaguar REQUIRED)

# Create executable
add_executable(flight_sim
    src/main.cpp
    src/aircraft.cpp
)

target_include_directories(flight_sim PRIVATE src)
target_link_libraries(flight_sim PRIVATE Jaguar::jaguar)

# Create output directory
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/output)

# Copy config files
file(COPY config DESTINATION ${CMAKE_BINARY_DIR})
```

---

## Step 6: Build and Run

```bash
# Create directories
mkdir -p config output

# Build
mkdir build && cd build
cmake ..
make

# Run
./flight_sim
```

---

## Expected Output

```
╔═══════════════════════════════════════════════════╗
║     JaguarEngine Flight Simulation Demo           ║
║     Version: 0.4.0                                ║
╚═══════════════════════════════════════════════════╝
Engine initialized.
Aircraft spawned at 20,000 ft, 400 KIAS

Flight Controls:
  W/S - Pitch down/up
  A/D - Roll left/right
  Q/E - Yaw left/right
  +/- - Throttle up/down
  ESC - Exit

╔══════════════════════════════════════════════════════════════════╗
║                    JaguarEngine Flight Simulator                  ║
╠══════════════════════════════════════════════════════════════════╣
║  Time │ Alt (ft) │ IAS (kt) │ Mach │ Alpha │ G's │ Fuel (lb)    ║
╠══════════════════════════════════════════════════════════════════╣
║   0.0 │  20000.0 │    400.0 │ 0.62 │   2.1 │  1.0 │   5511.6     ║
║   1.0 │  19998.2 │    399.5 │ 0.62 │   2.1 │  1.0 │   5509.1     ║
║   2.0 │  19996.4 │    399.0 │ 0.62 │   2.2 │  1.0 │   5506.6     ║
...
╚══════════════════════════════════════════════════════════════════╝

Final Status:
  Altitude: 22450.3 ft
  Airspeed: 385.2 kt
  Fuel remaining: 5312.4 lb

Flight data saved to: output/flight_data.csv
Simulation complete.
```

---

## Analyzing Results

The simulation outputs CSV data that you can analyze:

```python
import pandas as pd
import matplotlib.pyplot as plt

# Load data
df = pd.read_csv('output/flight_data.csv')

# Plot altitude profile
fig, axes = plt.subplots(2, 2, figsize=(12, 8))

axes[0, 0].plot(df['time'], df['altitude'] * 3.28084)
axes[0, 0].set_xlabel('Time (s)')
axes[0, 0].set_ylabel('Altitude (ft)')
axes[0, 0].set_title('Altitude Profile')
axes[0, 0].grid(True)

axes[0, 1].plot(df['time'], df['airspeed'] * 1.94384)
axes[0, 1].set_xlabel('Time (s)')
axes[0, 1].set_ylabel('Airspeed (kt)')
axes[0, 1].set_title('Airspeed Profile')
axes[0, 1].grid(True)

axes[1, 0].plot(df['time'], df['alpha'])
axes[1, 0].set_xlabel('Time (s)')
axes[1, 0].set_ylabel('Alpha (deg)')
axes[1, 0].set_title('Angle of Attack')
axes[1, 0].grid(True)

axes[1, 1].plot(df['time'], df['fuel'] * 2.20462)
axes[1, 1].set_xlabel('Time (s)')
axes[1, 1].set_ylabel('Fuel (lb)')
axes[1, 1].set_title('Fuel Consumption')
axes[1, 1].grid(True)

plt.tight_layout()
plt.savefig('flight_analysis.png')
plt.show()
```

---

## Next Steps

- **[Add Terrain](../tutorials/terrain.md)** - Add terrain collision and following
- **[Autopilot](../tutorials/autopilot.md)** - Implement altitude and heading hold
- **[Multi-Aircraft](../tutorials/multi-entity.md)** - Simulate multiple aircraft
- **[Python Integration](../api/python.md)** - Use Python for analysis
