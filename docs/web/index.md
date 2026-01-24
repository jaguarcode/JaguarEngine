# JaguarEngine Documentation

<div class="hero">
  <h1>JaguarEngine</h1>
  <p class="tagline">Next-Generation Multi-Domain Physics Simulation Platform</p>
</div>

## Overview

JaguarEngine is a high-performance physics simulation framework designed for defense modeling and simulation (M&S) applications. It supports **Air**, **Land**, **Sea**, and **Space** domains within a unified architecture, inspired by JSBSim's proven design patterns while leveraging modern C++20 capabilities.

<div class="features">

### Multi-Domain Physics
Unified framework supporting aircraft, ground vehicles, surface vessels, and spacecraft in a single simulation.

### High Performance
Data-oriented design with Structure-of-Arrays (SoA) memory layout, SIMD optimization, and GPU compute acceleration (CUDA, OpenCL, Metal) for real-time execution.

### Accurate Models
6-DOF rigid body dynamics, US Standard Atmosphere 1976, Bekker-Wong terramechanics, MMG ship maneuvering, SGP4/SDP4 orbital propagation.

### Distributed Simulation
Complete IEEE 1278.1-2012 DIS and IEEE 1516-2010 HLA RTI federation support for Live-Virtual-Constructive (LVC) integration.

### Machine Learning
Neural autopilot systems, ONNX model inference, reinforcement learning environments with Gym-compatible API, and model repository management.

### Cloud Native
Auto-scaling cloud burst capability with distributed state synchronization, Raft consensus, and spatial partitioning for massively concurrent simulations.

### XR Ready
Full OpenXR integration with spatial audio, haptic feedback, and training scenario management for immersive simulation experiences.

### Digital Thread
Complete lifecycle tracking with history store, predictive degradation models, and physics-based failure prediction for digital twin applications.

### Scriptable
Python and Lua bindings for rapid prototyping and integration with data science workflows.

</div>

---

## Quick Links

| Getting Started | Reference | Advanced |
|-----------------|-----------|----------|
| [Installation Guide](getting-started/installation.md) | [API Reference](api/overview.md) | [Architecture](advanced/architecture.md) |
| [Quick Start Tutorial](getting-started/quickstart.md) | [Configuration](api/configuration.md) | [Custom Models](advanced/custom-models.md) |
| [First Simulation](getting-started/first-simulation.md) | [Python API](api/python.md) | [Network Integration](advanced/networking.md) |

| Phase 7 Modules | | |
|-----------------|-----------|----------|
| [Cloud Burst](api/cloud.md) | [Digital Thread](api/thread.md) | [Machine Learning](api/ml.md) |
| [Federation](api/federation.md) | [GPU Compute](api/gpu.md) | [Sensors](api/sensors.md) |

---

## Domain Overview

### Air Domain
- Coefficient-based aerodynamics with N-dimensional interpolation tables
- Turbofan/turbojet propulsion with altitude-Mach corrections
- Flight control system with rate limiting and autopilot

[Learn more about Air Domain &rarr;](domains/air.md)

### Land Domain
- Bekker-Wong terramechanics for soil-vehicle interaction
- Spring-damper suspension with bump stops
- Tracked and wheeled vehicle dynamics

[Learn more about Land Domain &rarr;](domains/land.md)

### Sea Domain
- Buoyancy with metacentric height stability
- MMG (Maneuvering Mathematical Group) hydrodynamics
- Pierson-Moskowitz and JONSWAP wave spectra
- RAO-based ship motion response

[Learn more about Sea Domain &rarr;](domains/sea.md)

### Space Domain
- SGP4/SDP4 orbital propagation
- High-fidelity gravity models (J2, J4, EGM96)
- Atmospheric drag (JBH08 model)

[Learn more about Space Domain &rarr;](domains/space.md)

---

## Installation

```bash
# Clone the repository
git clone https://github.com/jaguarcode/JaguarEngine.git
cd JaguarEngine

# Build
mkdir build && cd build
cmake ..
make -j$(nproc)

# Run tests
./jaguar_unit_tests
```

[Complete installation guide &rarr;](getting-started/installation.md)

---

## Basic Usage

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
    state.position = Vec3{0.0, 0.0, -10000.0};  // 10 km altitude
    state.velocity = Vec3{250.0, 0.0, 0.0};     // 250 m/s forward
    state.mass = 12000.0;
    engine.set_entity_state(aircraft, state);

    // Run simulation
    for (int i = 0; i < 1000; ++i) {
        engine.step(0.01);  // 100 Hz
    }

    engine.shutdown();
    return 0;
}
```

[View more examples &rarr;](tutorials/examples.md)

---

## Project Status

| Component | Status | Version |
|-----------|--------|---------|
| Core Engine | Stable | 0.5.0 |
| Air Domain | Stable | Complete |
| Land Domain | Stable | Complete |
| Sea Domain | Stable | Complete |
| Space Domain | Stable | Complete |
| XR Integration | Stable | Complete |
| Cloud Burst | Stable | Complete |
| Digital Thread | Stable | Complete |
| Machine Learning | Stable | Complete |
| GPU Compute | Stable | Complete |
| Sensors | Stable | Complete |
| DIS Protocol | Stable | Complete |
| HLA Protocol | Stable | Complete |
| Python Bindings | Stable | 0.5.0 |
| Lua Bindings | Stable | 0.5.0 |

**Test Coverage**: 1150+ unit tests across all modules

---

## Community & Support

- **GitHub Issues**: [Report bugs and request features](https://github.com/jaguarcode/JaguarEngine/issues)
- **Discussions**: [Ask questions and share ideas](https://github.com/jaguarcode/JaguarEngine/discussions)
- **Contributing**: [Development guidelines](contributing.md)

---

## License

Copyright (c) 2024 JaguarEngine Contributors. All rights reserved.

See [LICENSE](https://github.com/jaguarcode/JaguarEngine/blob/main/LICENSE) for details.
