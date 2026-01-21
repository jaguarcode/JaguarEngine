# Quick Start Guide

Get up and running with JaguarEngine in under 10 minutes.

## Prerequisites

Before starting, ensure you have:

- [x] JaguarEngine built and installed ([Installation Guide](installation.md))
- [x] C++ development environment (compiler, IDE)
- [x] Basic understanding of physics simulation concepts

---

## Your First Simulation

### Step 1: Create a New Project

Create a new directory for your simulation project:

```bash
mkdir my_first_simulation
cd my_first_simulation
```

Create a `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.25)
project(MyFirstSimulation)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find JaguarEngine
find_package(Jaguar REQUIRED)

# Create executable
add_executable(simulation main.cpp)
target_link_libraries(simulation PRIVATE Jaguar::jaguar)
```

### Step 2: Write Your Simulation

Create `main.cpp`:

```cpp
#include <jaguar/jaguar.h>
#include <iostream>
#include <iomanip>

int main() {
    std::cout << "JaguarEngine Quick Start Example\n";
    std::cout << "Version: " << jaguar::GetVersionString() << "\n\n";

    // Step 1: Create and initialize the engine
    jaguar::interface::Engine engine;

    if (!engine.initialize()) {
        std::cerr << "Failed to initialize engine\n";
        return 1;
    }
    std::cout << "Engine initialized successfully\n";

    // Step 2: Create an aircraft entity
    jaguar::EntityId aircraft = engine.create_entity("F-16", jaguar::Domain::Air);
    std::cout << "Created aircraft entity (ID: " << aircraft << ")\n";

    // Step 3: Set initial state
    jaguar::physics::EntityState initial_state;
    initial_state.position = jaguar::Vec3{0.0, 0.0, -10000.0};  // 10 km altitude (NED)
    initial_state.velocity = jaguar::Vec3{250.0, 0.0, 0.0};     // 250 m/s forward
    initial_state.orientation = jaguar::Quaternion::identity();  // Level flight
    initial_state.mass = 12000.0;                                // 12,000 kg

    engine.set_entity_state(aircraft, initial_state);
    std::cout << "Initial state set\n\n";

    // Step 4: Run simulation loop
    const jaguar::Real dt = 0.01;  // 100 Hz update rate
    const jaguar::Real duration = 10.0;  // 10 seconds

    std::cout << "Running simulation for " << duration << " seconds...\n";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "--------------------------------------------\n";
    std::cout << "  Time (s)  |  Altitude (m)  |  Speed (m/s)\n";
    std::cout << "--------------------------------------------\n";

    for (jaguar::Real t = 0; t <= duration; t += dt) {
        // Advance simulation
        engine.step(dt);

        // Print status every second
        if (static_cast<int>(t * 100) % 100 == 0) {
            auto state = engine.get_entity_state(aircraft);
            jaguar::Real altitude = -state.position.z;  // NED: negative z is up
            jaguar::Real speed = state.velocity.norm();

            std::cout << "    " << std::setw(6) << t
                      << "    |    " << std::setw(8) << altitude
                      << "    |    " << std::setw(6) << speed << "\n";
        }
    }

    std::cout << "--------------------------------------------\n";
    std::cout << "Simulation complete!\n";

    // Step 5: Get final state
    auto final_state = engine.get_entity_state(aircraft);
    std::cout << "\nFinal Position: ("
              << final_state.position.x << ", "
              << final_state.position.y << ", "
              << final_state.position.z << ") m\n";
    std::cout << "Final Velocity: ("
              << final_state.velocity.x << ", "
              << final_state.velocity.y << ", "
              << final_state.velocity.z << ") m/s\n";

    // Step 6: Clean up
    engine.shutdown();
    std::cout << "Engine shutdown complete\n";

    return 0;
}
```

### Step 3: Build and Run

```bash
# Configure
mkdir build && cd build
cmake ..

# Build
make

# Run
./simulation
```

**Expected Output:**

```
JaguarEngine Quick Start Example
Version: 0.4.0

Engine initialized successfully
Created aircraft entity (ID: 1)
Initial state set

Running simulation for 10.00 seconds...
--------------------------------------------
  Time (s)  |  Altitude (m)  |  Speed (m/s)
--------------------------------------------
      0.00    |    10000.00    |   250.00
      1.00    |     9951.23    |   249.85
      2.00    |     9902.46    |   249.70
      3.00    |     9853.69    |   249.55
      ...
     10.00    |     9512.30    |   248.50
--------------------------------------------
Simulation complete!

Final Position: (2485.00, 0.00, -9512.30) m
Final Velocity: (248.50, 0.00, 0.15) m/s
Engine shutdown complete
```

---

## Understanding the Code

### Engine Lifecycle

```cpp
// 1. Create engine instance
jaguar::interface::Engine engine;

// 2. Initialize (loads configuration, sets up subsystems)
engine.initialize();

// 3. Use the engine (create entities, run simulation)
// ...

// 4. Clean shutdown (releases resources)
engine.shutdown();
```

### Entity Management

```cpp
// Create entity with name and domain
jaguar::EntityId id = engine.create_entity("name", jaguar::Domain::Air);

// Set entity state
engine.set_entity_state(id, state);

// Get entity state
auto state = engine.get_entity_state(id);

// Destroy entity when done
engine.destroy_entity(id);
```

### Coordinate System (NED)

JaguarEngine uses the North-East-Down (NED) coordinate frame:

- **X**: North (positive forward)
- **Y**: East (positive right)
- **Z**: Down (positive into earth, negative is altitude)

```cpp
// 10 km altitude: z = -10000
state.position = {0.0, 0.0, -10000.0};

// Flying north at 250 m/s
state.velocity = {250.0, 0.0, 0.0};
```

### Simulation Loop

```cpp
const jaguar::Real dt = 0.01;  // Time step (seconds)

for (jaguar::Real t = 0; t < duration; t += dt) {
    // 1. Update entity controls (if needed)
    // 2. Advance physics
    engine.step(dt);
    // 3. Read results
    auto state = engine.get_entity_state(entity);
}
```

---

## Common Patterns

### Setting Up Multiple Entities

```cpp
// Create entities for different domains
auto aircraft = engine.create_entity("F-16", jaguar::Domain::Air);
auto tank = engine.create_entity("M1A2", jaguar::Domain::Land);
auto ship = engine.create_entity("DDG-51", jaguar::Domain::Sea);
auto satellite = engine.create_entity("GPS-IIR", jaguar::Domain::Space);
```

### Applying Forces

```cpp
jaguar::physics::EntityForces forces;

// Add force in body frame
forces.add_force({1000.0, 0.0, 0.0});  // 1000 N forward

// Add torque
forces.add_torque({0.0, 100.0, 0.0});  // 100 Nm pitch

// Apply to entity
engine.apply_forces(entity_id, forces);
```

### Querying Environment

```cpp
// Get environment at entity location
auto env = engine.get_environment(entity_id);

std::cout << "Altitude: " << env.altitude << " m\n";
std::cout << "Air density: " << env.atmosphere.density << " kg/m³\n";
std::cout << "Temperature: " << env.atmosphere.temperature << " K\n";
```

---

## Next Steps

Now that you've created your first simulation, explore these topics:

1. **[Air Domain Tutorial](../tutorials/air-domain.md)** - Detailed aircraft simulation
2. **[Land Domain Tutorial](../tutorials/land-domain.md)** - Ground vehicle physics
3. **[Sea Domain Tutorial](../tutorials/sea-domain.md)** - Ship simulation with waves
4. **[Space Domain Tutorial](../tutorials/space-domain.md)** - Orbital mechanics
5. **[Multi-Entity Example](../tutorials/multi-entity.md)** - Combined simulation
6. **[Configuration Guide](../api/configuration.md)** - XML configuration system
7. **[API Reference](../api/overview.md)** - Complete API documentation

---

## Troubleshooting

### Engine Fails to Initialize

```cpp
if (!engine.initialize()) {
    // Check for missing configuration files
    // Verify data paths are correct
}
```

### Entity State Not Changing

Make sure you're calling `engine.step(dt)` to advance the simulation:

```cpp
// Wrong: state won't change
engine.set_entity_state(id, state);
auto s = engine.get_entity_state(id);  // Same as input

// Correct: advance physics
engine.set_entity_state(id, state);
engine.step(0.01);  // Physics update
auto s = engine.get_entity_state(id);  // Updated state
```

### Unexpected Position Values

Remember NED coordinate conventions:
- Altitude is negative Z
- Velocity forward is positive X

```cpp
// Altitude check
jaguar::Real altitude = -state.position.z;  // Convert to positive altitude
```
