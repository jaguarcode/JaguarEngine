# JaguarEngine

**Next-Generation Multi-Domain Physics Simulation Platform**

JaguarEngine is a high-performance physics simulation framework designed for defense modeling and simulation (M&S) applications. It supports Air, Land, Sea, and Space domains within a unified architecture, inspired by JSBSim's proven design patterns while leveraging modern C++20 capabilities.

## Features

- **Multi-Domain Physics**: Unified framework supporting aircraft, ground vehicles, surface vessels, and spacecraft
- **High Performance**: Data-oriented design with Structure-of-Arrays (SoA) memory layout and SIMD optimization
- **Accurate Models**:
  - 6-DOF rigid body dynamics with multiple integration methods (RK4, Adams-Bashforth)
  - US Standard Atmosphere 1976
  - Bekker-Wong terramechanics
  - MMG ship maneuvering model
  - SGP4/SDP4 orbital propagation
- **Flexible Architecture**: Component-based force generators, property system, and event-driven design
- **Network Ready**: DIS and HLA protocol support for distributed simulation
- **Scriptable**: Python and Lua bindings for rapid prototyping

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

## Project Structure

```
JaguarEngine/
├── include/jaguar/          # Public headers
│   ├── core/                # Core types, math, memory
│   ├── physics/             # Entity management, forces, solvers
│   ├── domain/              # Domain-specific physics (air, land, sea, space)
│   ├── environment/         # Terrain, atmosphere, ocean
│   └── interface/           # API facade, configuration
├── src/                     # Implementation
├── tests/                   # Unit and integration tests
├── examples/                # Example applications
├── docs/                    # Documentation
└── data/                    # Configuration files and models
```

## Documentation

| Document | Description |
|----------|-------------|
| [Architecture](docs/ARCHITECTURE.md) | System design and component overview |
| [Installation](docs/INSTALLATION.md) | Build instructions and dependencies |
| [API Reference](docs/API_REFERENCE.md) | Complete API documentation |
| [Examples Guide](docs/EXAMPLES.md) | Walkthrough of example code |
| [Configuration](docs/CONFIGURATION.md) | XML configuration reference |

### Module Documentation

| Module | Description |
|--------|-------------|
| [Core](docs/modules/CORE.md) | Types, math, memory, property system |
| [Physics](docs/modules/PHYSICS.md) | Entity management, integration |
| [Air Domain](docs/modules/AIR_DOMAIN.md) | Aerodynamics, propulsion, flight control |
| [Land Domain](docs/modules/LAND_DOMAIN.md) | Terramechanics, suspension |
| [Sea Domain](docs/modules/SEA_DOMAIN.md) | Hydrodynamics, buoyancy, waves |
| [Space Domain](docs/modules/SPACE_DOMAIN.md) | Orbital mechanics, SGP4 |
| [Environment](docs/modules/ENVIRONMENT.md) | Terrain, atmosphere, ocean |

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
237 tests across 41 test suites
- Math: Vector, quaternion, matrix operations
- Physics: Entity state, force accumulation, integration
- Coordinates: ECEF, NED, ECI transformations
- Atmosphere: US Standard 1976 verification
- Air Domain: Aerodynamics, propulsion, flight control
- Land Domain: Terramechanics, suspension, tracked vehicles
- Sea Domain: Buoyancy, hydrodynamics, waves, RAO
```

## Build Options

| Option | Default | Description |
|--------|---------|-------------|
| `JAGUAR_BUILD_TESTS` | ON | Build unit test suite |
| `JAGUAR_BUILD_BENCHMARKS` | ON | Build performance benchmarks |
| `JAGUAR_BUILD_EXAMPLES` | ON | Build example applications |
| `JAGUAR_BUILD_PYTHON` | OFF | Build Python bindings |
| `JAGUAR_BUILD_LUA` | OFF | Build Lua bindings |
| `JAGUAR_ENABLE_DIS` | OFF | Enable DIS network support |
| `JAGUAR_ENABLE_HLA` | OFF | Enable HLA network support |
| `JAGUAR_ENABLE_SIMD` | ON | Enable SIMD optimizations |

## Dependencies

| Library | Version | Purpose | Required |
|---------|---------|---------|----------|
| Eigen3 | 3.4+ | Linear algebra | Yes (auto-fetched) |
| pugixml | 1.14+ | XML parsing | Yes (auto-fetched) |
| GDAL | 3.0+ | Geospatial terrain | Optional |
| GoogleTest | 1.14+ | Testing | Auto-fetched |
| pybind11 | 2.11+ | Python bindings | Optional |

## Contributing

See [CONTRIBUTING.md](docs/CONTRIBUTING.md) for development guidelines.

## License

Copyright (c) 2024 JaguarEngine Contributors. All rights reserved.

## Acknowledgments

- Inspired by [JSBSim](https://github.com/JSBSim-Team/jsbsim) flight dynamics model
- Uses the [Eigen](https://eigen.tuxfamily.org/) linear algebra library
