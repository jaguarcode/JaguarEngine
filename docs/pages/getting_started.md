# Getting Started {#getting_started}

This guide will help you get started with JaguarEngine quickly.

## Installation

### Prerequisites

- CMake 3.20 or higher
- C++17 compatible compiler (GCC 9+, Clang 10+, MSVC 2019+)
- Eigen 3.4+ (automatically downloaded)

### Optional Dependencies

- Python 3.8+ and pybind11 (for Python bindings)
- Lua 5.4+ and sol2 (for Lua bindings)
- Google Test (for unit tests)

### Building from Source

```bash
# Clone the repository
git clone https://github.com/jaguarcode/JaguarEngine.git
cd JaguarEngine

# Create build directory
mkdir build && cd build

# Configure (basic)
cmake ..

# Configure (with Python bindings)
cmake -DJAGUAR_BUILD_PYTHON=ON ..

# Configure (with all features)
cmake -DJAGUAR_BUILD_PYTHON=ON \
      -DJAGUAR_BUILD_LUA=ON \
      -DJAGUAR_BUILD_TESTS=ON \
      -DJAGUAR_BUILD_EXAMPLES=ON ..

# Build
cmake --build . --config Release -j$(nproc)

# Install
cmake --install . --prefix /usr/local
```

## Basic Usage

### C++ Example

```cpp
#include <jaguar/interface/api.h>
#include <iostream>

int main() {
    // Create and initialize engine
    jaguar::Engine engine;
    if (!engine.initialize()) {
        std::cerr << "Failed to initialize engine" << std::endl;
        return 1;
    }

    // Create entities
    auto aircraft = engine.create_entity("f16", jaguar::Domain::Air);
    auto tank = engine.create_entity("m1a2", jaguar::Domain::Land);
    auto ship = engine.create_entity("ddg", jaguar::Domain::Sea);
    auto satellite = engine.create_entity("gps", jaguar::Domain::Space);

    // Configure aircraft initial state
    auto state = engine.get_entity_state(aircraft);
    state.position = jaguar::Vec3(0, 0, -3000);     // 3km altitude
    state.velocity = jaguar::Vec3(250, 0, 0);       // 250 m/s
    state.mass = 12000;                              // 12 tons
    engine.set_entity_state(aircraft, state);

    // Simulation loop
    double dt = 0.01;  // 100 Hz
    while (engine.get_time() < 60.0) {  // 1 minute
        engine.step(dt);

        // Get updated state
        auto current = engine.get_entity_state(aircraft);
        std::cout << "t=" << engine.get_time()
                  << " pos=(" << current.position.x << ", "
                  << current.position.y << ", "
                  << current.position.z << ")" << std::endl;
    }

    engine.shutdown();
    return 0;
}
```

### Python Example

```python
import pyjaguar as jag
import numpy as np

# Create and initialize
engine = jag.Engine()
engine.initialize()

# Create aircraft
aircraft = engine.create_entity("f16", jag.Domain.Air)

# Set state
state = engine.get_entity_state(aircraft)
state.position = jag.Vec3(0, 0, -1000)
state.velocity = jag.Vec3(200, 0, 0)
engine.set_entity_state(aircraft, state)

# Simulate
for _ in range(1000):
    engine.step(0.01)

print(f"Final time: {engine.get_time():.2f} s")
engine.shutdown()
```

### Lua Example

```lua
-- Initialize engine
local engine = Engine()
engine:initialize()

-- Create aircraft
local aircraft = engine:create_entity("f16", Domain.Air)

-- Set state
local state = engine:get_entity_state(aircraft)
state.position = Vec3(0, 0, -1000)
state.velocity = Vec3(200, 0, 0)
engine:set_entity_state(aircraft, state)

-- Simulate
for i = 1, 1000 do
    engine:step(0.01)
end

print(string.format("Final time: %.2f s", engine:get_time()))
engine:shutdown()
```

## Domain Overview

### Air Domain

The Air domain provides 6-DOF flight dynamics simulation:

- Aerodynamic forces and moments (lift, drag, side force)
- Engine thrust modeling
- Atmospheric effects (ISA, wind, gusts)
- High-alpha post-stall aerodynamics
- Ground effect

### Land Domain

The Land domain provides ground vehicle dynamics:

- Tire/track-terrain interaction (Bekker-Wong model)
- Suspension dynamics
- Steering and drivetrain models
- Terrain slope and obstacle effects

### Sea Domain

The Sea domain provides ship and submarine dynamics:

- Hydrodynamic forces (ITTC compliant)
- Wave response and seakeeping
- Propulsion and maneuvering
- Stability and roll damping

### Space Domain

The Space domain provides orbital mechanics:

- SGP4/SDP4 propagation
- Gravity models (point mass to J2-J4)
- Atmospheric drag (JB08 model)
- Orbital element conversions
- LVLH frame transforms

## Next Steps

- See the @ref cpp_api "C++ API Reference" for detailed API documentation
- Explore the @ref physics_models "Physics Models" section for model details
- Check the @ref validation "Validation" section for V&V information
