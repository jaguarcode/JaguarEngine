# JaguarEngine

**Next-Generation Multi-Domain Physics Simulation Platform**

JaguarEngine is a high-performance physics simulation framework designed for defense modeling and simulation (M&S) applications. It supports Air, Land, Sea, and Space domains within a unified architecture, inspired by JSBSim's proven design patterns while leveraging modern C++20 capabilities.

## What's New in v0.6.0

- **Component Force Registry**: Per-entity force management for modular physics composition
- **GDAL Terrain Support**: Real-world elevation data integration with bilinear interpolation
- **Work-Stealing Thread Pool**: Parallel force computation for multi-physics scenarios
- **DIS/HLA Federation**: Networked multi-simulation support (IEEE 1278.1-2012 & IEEE 1516-2010)
- **Improved Constraint Solver**: Split impulse formulation with early exit optimization
- **Terrain-Aware Ground Contact**: Suspension systems adapting to terrain elevation and normal

## Features

### Core Physics

- **Multi-Domain Physics**: Unified framework supporting aircraft, ground vehicles, surface vessels, and spacecraft
- **High Performance**: Data-oriented design with Structure-of-Arrays (SoA) memory layout and SIMD optimization
- **Advanced Integrators**:
  - RK4, Adams-Bashforth-Moulton (ABM4)
  - Energy-conserving Symplectic Euler and Verlet integrators
  - Adaptive Dormand-Prince (RK45) with automatic error control
  - Boris integrator for charged particle dynamics
- **Constraint System**: Joint constraints, distance constraints, XPB solver

### Domain Models

- 6-DOF rigid body dynamics
- US Standard Atmosphere 1976 with Dryden turbulence
- Bekker-Wong terramechanics
- MMG ship maneuvering model
- SGP4/SDP4 orbital propagation

### GPU Acceleration

- CUDA/Metal/Vulkan compute backends
- GPU-accelerated collision detection
- Hybrid CPU-GPU physics pipeline

### Extended Reality

- **OpenXR Integration**: VR/AR headset support with full hand/eye tracking
- **Spatial Audio**: HRTF-based 3D audio with distance attenuation, room acoustics, occlusion, and Doppler effect
- **Haptic Feedback**: Controller vibration, haptic vest, motion platform, engine vibration, and G-force simulation
- **XR Session Management**: Multi-platform XR runtime abstraction

### Event System

- Thread-safe event bus with pub/sub pattern
- Type-safe event dispatching
- Async and deferred event processing

### Scripting Support

- **Python bindings** with NumPy interoperability
- **Lua bindings** with full operator support and table conversion

### Environment & Terrain

- **GDAL Terrain Integration**: Load real-world elevation data (GeoTIFF, HDF5, etc.)
- **Bilinear Interpolation**: Smooth terrain elevation queries
- **Terrain-Aware Physics**: Ground contact, suspension, and friction adapted to terrain

### Architecture & Networking

- **Component-Based Force Generators**: Modular per-entity force composition
- **Property System**: Reflection and introspection for runtime configuration
- **Event-Driven Design**: Asynchronous system communication
- **Parallel Computation**: Work-stealing thread pool for force calculation
- **Distributed Simulation**:
  - DIS (IEEE 1278.1-2012) for real-time federation
  - HLA (IEEE 1516-2010) for hierarchical simulation
  - Network transport with entity state synchronization

## Quick Start

### Prerequisites

- C++20 compatible compiler (GCC 11+, Clang 14+, MSVC 2022+)
- CMake 3.25+
- Eigen3 (auto-fetched if not found)

### Building

```bash
git clone https://github.com/jaguarcode/JaguarEngine.git
cd JaguarEngine
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### Running Tests

```bash
./jaguar_unit_tests
```

### Basic Usage

```cpp
#include <jaguar/jaguar.h>

int main() {
    using namespace jaguar;

    // Create and initialize engine
    interface::Engine engine;
    engine.initialize();

    // Create an aircraft entity
    EntityId aircraft = engine.create_entity("F16", Domain::Air);

    // Set initial state
    physics::EntityState state;
    state.position = Vec3{0.0, 0.0, -1000.0};  // 1000m altitude
    state.velocity = Vec3{200.0, 0.0, 0.0};    // 200 m/s forward
    state.orientation = Quat::Identity();
    state.mass = 12000.0;  // kg
    engine.set_entity_state(aircraft, state);

    // Run simulation loop
    Real dt = 0.01;  // 100 Hz
    for (int i = 0; i < 1000; ++i) {
        engine.step(dt);
    }

    engine.shutdown();
    return 0;
}
```

### Python Example

```python
import jaguar

# Create and initialize engine
engine = jaguar.Engine()
engine.initialize()

# Create an aircraft entity
aircraft = engine.create_entity("F16", jaguar.Domain.Air)

# Set initial state
state = jaguar.EntityState()
state.position = jaguar.Vec3(0, 0, -1000)  # 1000m altitude
state.velocity = jaguar.Vec3(200, 0, 0)    # 200 m/s
state.mass = 12000.0
engine.set_entity_state(aircraft, state)

# Run simulation
for _ in range(100):
    engine.step(0.01)

# Get final position
final = engine.get_entity_state(aircraft)
print(f"Final altitude: {-final.position.z:.1f} m")

engine.shutdown()
```

### Lua Example

```lua
local jag = require("jaguar")

-- Create and initialize engine
local engine = Engine()
engine:initialize()

-- Create an aircraft entity
local aircraft = engine:create_entity("F16", Domain.Air)

-- Set initial state
local state = EntityState()
state.position = Vec3(0, 0, -1000)  -- 1000m altitude
state.velocity = Vec3(200, 0, 0)    -- 200 m/s
state.mass = 12000.0
engine:set_entity_state(aircraft, state)

-- Run simulation
for i = 1, 100 do
    engine:step(0.01)
end

-- Get final position
local final = engine:get_entity_state(aircraft)
print(string.format("Final altitude: %.1f m", -final.position.z))

engine:shutdown()
```

### Terrain Integration Example (C++)

```cpp
#include <jaguar/jaguar.h>

int main() {
    using namespace jaguar;

    interface::Engine engine;
    engine.initialize();

    // Load terrain from GeoTIFF file
    auto terrain = environment::Terrain::from_gdal("data/terrain/dem.tif");
    engine.set_environment(terrain);

    // Create vehicle entity
    EntityId vehicle = engine.create_entity("Humvee", Domain::Land);

    // Set position - suspension will automatically adapt to terrain
    physics::EntityState state;
    state.position = Vec3{100.0, 50.0, 0.0};  // Z will be set by terrain elevation
    state.mass = 2500.0;
    engine.set_entity_state(vehicle, state);

    // Run simulation with terrain-aware physics
    for (int i = 0; i < 100; ++i) {
        engine.step(0.01);
    }

    engine.shutdown();
    return 0;
}
```

## Project Structure

```
JaguarEngine/
├── include/jaguar/          # Public headers
│   ├── core/                # Core types, math, memory, threading
│   ├── physics/             # Entity management, forces, integrators, constraints
│   │   ├── integrators/     # RK4, ABM4, Symplectic, Verlet, Dormand-Prince, Boris
│   │   └── constraints/     # Joint, distance, and angle constraints
│   ├── domain/              # Domain-specific physics (air, land, sea, space)
│   ├── environment/         # Terrain, atmosphere, ocean, turbulence
│   ├── events/              # Event bus and type-safe dispatching
│   ├── gpu/                 # GPU compute backends (CUDA, Metal, Vulkan)
│   ├── sensors/             # Sensor simulation (IMU, GPS, radar)
│   ├── xr/                  # XR integration (OpenXR, spatial audio)
│   └── interface/           # API facade, configuration, scripting
├── src/                     # Implementation
├── tests/                   # Unit, integration, and GPU tests
├── examples/                # Example applications
│   ├── frontend/            # React/Cesium visualization frontend
│   └── server/              # WebSocket server for real-time data
├── docs/                    # Documentation
└── data/                    # Configuration files and models
```

## Documentation

| Document                                          | Description                          |
| ------------------------------------------------- | ------------------------------------ |
| [Architecture](docs/architecture/ARCHITECTURE.md) | System design and component overview |
| [Installation](docs/INSTALLATION.md)              | Build instructions and dependencies  |
| [API Reference](docs/API_REFERENCE.md)            | Complete API documentation           |
| [Examples Guide](docs/EXAMPLES.md)                | Walkthrough of example code          |
| [Configuration](docs/CONFIGURATION.md)            | XML configuration reference          |

### Module Documentation

| Module                                       | Description                                    |
| -------------------------------------------- | ---------------------------------------------- |
| [Core](docs/modules/CORE.md)                 | Types, math, memory, property system           |
| [Physics](docs/modules/PHYSICS.md)           | Entity management, integrators, constraints    |
| [Air Domain](docs/modules/AIR_DOMAIN.md)     | Aerodynamics, propulsion, flight control       |
| [Land Domain](docs/modules/LAND_DOMAIN.md)   | Terramechanics, suspension                     |
| [Sea Domain](docs/modules/SEA_DOMAIN.md)     | Hydrodynamics, buoyancy, waves                 |
| [Space Domain](docs/modules/SPACE_DOMAIN.md) | Orbital mechanics, SGP4                        |
| [Environment](docs/modules/ENVIRONMENT.md)   | Terrain, atmosphere, ocean, turbulence         |
| [GPU Compute](docs/modules/GPU.md)           | CUDA/Metal/Vulkan backends, hybrid physics     |
| [XR Integration](docs/modules/XR.md)         | OpenXR, spatial audio, haptics, motion platform|
| [Sensors](docs/modules/SENSORS.md)           | IMU simulation, noise models, failure modes    |
| [Events](docs/modules/EVENTS.md)             | Event bus, dispatching, threshold monitoring   |
| [Cloud](docs/modules/CLOUD.md)               | Distributed simulation, auto-scaling           |
| [Federation](docs/modules/FEDERATION.md)     | DIS/HLA protocols, network transport           |
| [Digital Thread](docs/modules/THREAD.md)     | Lifecycle management, history, degradation     |
| [Machine Learning](docs/modules/ML.md)       | ONNX inference, neural autopilot, RL env       |

### Scripting APIs

| Language                         | Documentation      | Description                          |
| -------------------------------- | ------------------ | ------------------------------------ |
| [Python](docs/web/api/python.md) | Full API reference | pybind11 bindings with NumPy support |
| [Lua](docs/web/api/lua.md)       | Full API reference | sol2 bindings with table conversion  |

## Physics Domains

### Air Domain

- Coefficient-based aerodynamics with N-dimensional interpolation tables
- Turbofan/turbojet propulsion with altitude-Mach corrections
- Flight control system with rate limiting

### Land Domain

- Bekker-Wong terramechanics for soil-vehicle interaction
- Spring-damper suspension with bump stops
- Tracked vehicle dynamics

### Sea Domain

- Buoyancy with metacentric height stability
- MMG (Maneuvering Mathematical Group) hydrodynamics
- Pierson-Moskowitz and JONSWAP wave spectra
- RAO-based ship motion response

### Space Domain

- SGP4/SDP4 orbital propagation
- High-fidelity gravity models (J2, J4, EGM96)
- Atmospheric drag (JBH08 model)

## Test Coverage

```
1150+ tests across 100+ test suites
- Math: Vector, quaternion, matrix operations
- Physics: Entity state, force accumulation, integration
- Integrators: Symplectic Euler, Verlet, Dormand-Prince, Boris
- Constraints: Joint, distance, angle constraints
- Coordinates: ECEF, NED, ECI transformations
- Atmosphere: US Standard 1976 verification
- Air Domain: Aerodynamics, propulsion, flight control
- Land Domain: Terramechanics, suspension, tracked vehicles
- Sea Domain: Buoyancy, hydrodynamics, waves, RAO
- GPU: Compute backend validation, collision detection
- Events: Event bus, async dispatching
- XR: Session management, tracking, spatial audio
- Sensors: IMU, GPS, radar simulation
- Cloud: Partitioning, auto-scaling, state sync, distributed time
- Federation: DIS protocol, HLA RTI, network transport
- Digital Thread: Lifecycle, history store, degradation model
- Machine Learning: Inference, neural autopilot, RL environment
```

## Build Options

| Option                    | Default | Description                           |
| ------------------------- | ------- | ------------------------------------- |
| `JAGUAR_BUILD_TESTS`      | ON      | Build unit test suite                 |
| `JAGUAR_BUILD_BENCHMARKS` | ON      | Build performance benchmarks          |
| `JAGUAR_BUILD_EXAMPLES`   | ON      | Build example applications            |
| `JAGUAR_BUILD_PYTHON`     | OFF     | Build Python bindings                 |
| `JAGUAR_BUILD_LUA`        | OFF     | Build Lua bindings                    |
| `JAGUAR_ENABLE_DIS`       | OFF     | Enable DIS network support            |
| `JAGUAR_ENABLE_HLA`       | OFF     | Enable HLA network support            |
| `JAGUAR_ENABLE_SIMD`      | ON      | Enable SIMD optimizations             |
| `JAGUAR_ENABLE_CUDA`      | OFF     | Enable CUDA GPU acceleration          |
| `JAGUAR_ENABLE_METAL`     | OFF     | Enable Metal GPU acceleration (macOS) |
| `JAGUAR_ENABLE_VULKAN`    | OFF     | Enable Vulkan compute                 |
| `JAGUAR_ENABLE_XR`        | ON      | Enable XR (VR/AR) support             |
| `JAGUAR_ENABLE_OPENXR`    | OFF     | Enable OpenXR runtime                 |
| `JAGUAR_ENABLE_CLOUD`     | ON      | Enable cloud/distributed simulation   |
| `JAGUAR_ENABLE_THREAD`    | ON      | Enable Digital Thread support         |
| `JAGUAR_ENABLE_ML`        | ON      | Enable Machine Learning support       |

## Dependencies

| Library    | Version | Purpose            | Required                |
| ---------- | ------- | ------------------ | ----------------------- |
| Eigen3     | 3.4+    | Linear algebra     | Yes (auto-fetched)      |
| pugixml    | 1.14+   | XML parsing        | Yes (auto-fetched)      |
| GDAL       | 3.0+    | Geospatial terrain | Optional                |
| GoogleTest | 1.14+   | Testing            | Auto-fetched            |
| pybind11   | 2.11+   | Python bindings    | Optional (auto-fetched) |
| sol2       | develop | Lua bindings       | Optional (auto-fetched) |
| Lua        | 5.4+    | Lua runtime        | Optional (bundled)      |

## Contributing

See [CONTRIBUTING.md](docs/CONTRIBUTING.md) for development guidelines.

## License

Copyright (c) 2025 JaguarEngine Contributors. All rights reserved.

## Acknowledgments

- Inspired by [JSBSim](https://github.com/JSBSim-Team/jsbsim) flight dynamics model
- Uses the [Eigen](https://eigen.tuxfamily.org/) linear algebra library
