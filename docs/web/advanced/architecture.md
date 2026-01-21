# Architecture Overview

This document provides a comprehensive overview of JaguarEngine's system architecture, design patterns, and internal structure.

## System Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         EXTERNAL INTERFACES                              │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐    │
│  │   Python    │  │    Lua      │  │  DIS/HLA    │  │     CIG     │    │
│  │  (pybind11) │  │   (sol2)    │  │  Network    │  │ (UE/Unity)  │    │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘    │
└─────────┼────────────────┼────────────────┼────────────────┼────────────┘
          │                │                │                │
          └────────────────┴────────┬───────┴────────────────┘
                                    │
┌───────────────────────────────────┴────────────────────────────────────┐
│                         PUBLIC API LAYER                                │
│                      jaguar::Engine (Facade)                           │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │
┌───────────────────────────────────┴────────────────────────────────────┐
│                    PHYSICS ENGINE EXECUTIVE                             │
│  ┌──────────────────────────────────────────────────────────────────┐  │
│  │                    PhysicsEngineExec                              │  │
│  │  ┌────────────┐ ┌────────────┐ ┌────────────┐ ┌────────────┐    │  │
│  │  │  Entity    │ │  Physics   │ │  Property  │ │   Time     │    │  │
│  │  │  Manager   │ │  System    │ │  Manager   │ │  Manager   │    │  │
│  │  └────────────┘ └────────────┘ └────────────┘ └────────────┘    │  │
│  └──────────────────────────────────────────────────────────────────┘  │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │
┌───────────────────────────────────┴────────────────────────────────────┐
│                        DOMAIN PHYSICS LAYER                             │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐      │
│  │     AIR     │ │    LAND     │ │     SEA     │ │    SPACE    │      │
│  │ Aerodynamics│ │Terramechanics│ │Hydrodynamics│ │  Astrodynamics│    │
│  │ Propulsion  │ │ Suspension  │ │  Buoyancy   │ │   Gravity   │      │
│  │ FCS/Autopilot│ │   MBS      │ │    RAO      │ │  SGP4/SDP4  │      │
│  └─────────────┘ └─────────────┘ └─────────────┘ └─────────────┘      │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │
┌───────────────────────────────────┴────────────────────────────────────┐
│                      ENVIRONMENT SERVICES                               │
│  ┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐          │
│  │     TERRAIN     │ │   ATMOSPHERE    │ │      OCEAN      │          │
│  │   Digital Twin  │ │  US Std 1976    │ │ Wave Spectrum   │          │
│  │   GDAL/GIS      │ │  JBH08 (Space)  │ │ PM/JONSWAP      │          │
│  │   Quadtree LOD  │ │  MODTRAN LUT    │ │ Sea State       │          │
│  └─────────────────┘ └─────────────────┘ └─────────────────┘          │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │
┌───────────────────────────────────┴────────────────────────────────────┐
│                         CORE SERVICES                                   │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐    │
│  │  Memory  │ │ Threading│ │   Math   │ │   I/O    │ │  Config  │    │
│  │  Pools   │ │Work-Steal│ │   SIMD   │ │  Async   │ │   XML    │    │
│  └──────────┘ └──────────┘ └──────────┘ └──────────┘ └──────────┘    │
└────────────────────────────────────────────────────────────────────────┘
```

## Design Principles

### Data-Oriented Design (DOD)

JaguarEngine uses Structure of Arrays (SoA) memory layout for optimal cache utilization:

```cpp
// Traditional OOP (Array of Structures) - poor cache utilization
struct Entity { Vec3 pos; Vec3 vel; Quat ori; Real mass; };
std::vector<Entity> entities;

// JaguarEngine (Structure of Arrays) - excellent cache utilization
struct EntityStorage {
    std::vector<Vec3> positions;    // Contiguous
    std::vector<Vec3> velocities;   // Contiguous
    std::vector<Quat> orientations; // Contiguous
    std::vector<Real> masses;       // Contiguous
};
```

**Performance Benefits:**

| Pattern | Cache Miss Rate | Relative Speed |
|---------|-----------------|----------------|
| AoS (random) | ~80% | 1.0x |
| AoS (sequential) | ~40% | 2.5x |
| SoA | ~5% | 8-15x |

### Component-Based Physics

Modular force generators are attached to entities:

- **Aerodynamics**: Coefficient-based lookup tables
- **Propulsion**: Thrust curves with altitude/Mach corrections
- **Terramechanics**: Bekker-Wong soil interaction
- **Hydrodynamics**: MMG maneuvering model
- **Gravity**: Point mass to EGM2008 fidelity levels

### Property System

Hierarchical key-value access for runtime configuration (JSBSim-inspired):

```
aircraft/f16/aero/cl_alpha     → Lift curve slope
vehicle/m1a2/terra/sinkage     → Terrain sinkage (m)
vessel/ddg51/hydro/draft       → Current draft (m)
satellite/gps01/orbit/altitude → Orbital altitude (km)
```

## Core Subsystems

### PhysicsEngineExec

The central coordinator responsible for simulation lifecycle management:

- Initialize and configure all subsystems
- Execute the main simulation loop
- Coordinate time advancement across all entities
- Manage module registration and discovery

### Entity Management

Entities are lightweight identifiers; data lives in contiguous arrays:

```cpp
struct Entity {
    EntityId id;              // Unique identifier
    ComponentMask components; // Bitfield of attached components
    Domain primary_domain;    // Primary physics domain
    PropertyNode* properties; // Property tree root
};
```

### Time Management

| Mode | Description | Use Case |
|------|-------------|----------|
| Real-Time | 1s sim = 1s wall | Training, HiL |
| Accelerated | N×s sim = 1s wall | Analysis, Monte Carlo |
| Stepped | Manual advance | Debugging, Scripting |

## Threading Model

```
┌─────────────────────────────────────────────────────────────┐
│                    Main Thread                               │
│  • Input processing                                         │
│  • Script execution                                         │
│  • Network I/O dispatch                                     │
│  • Frame synchronization                                    │
└───────────────────────────┬─────────────────────────────────┘
                            │ Job dispatch
┌───────────────────────────┴─────────────────────────────────┐
│                 Physics Thread Pool                          │
│  ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐           │
│  │Worker 0 │ │Worker 1 │ │Worker 2 │ │Worker N │           │
│  │   ←Work Stealing→   │ │   ←Work Stealing→   │           │
│  └─────────┘ └─────────┘ └─────────┘ └─────────┘           │
└───────────────────────────┬─────────────────────────────────┘
                            │
┌───────────────────────────┴─────────────────────────────────┐
│                 I/O Thread Pool                              │
│  • Terrain tile loading (GDAL)                              │
│  • Network PDU send/receive                                 │
│  • File I/O operations                                      │
└─────────────────────────────────────────────────────────────┘
```

## Coordinate Systems

### Supported Frames

| Frame | Origin | Use Case |
|-------|--------|----------|
| ECEF | Earth center | Global positioning, geodetic reference |
| ECI J2000 | Earth center | Orbital mechanics |
| NED | Local origin | Aircraft, ground vehicles |
| Body | Entity CG | Entity-specific calculations |

### 64-bit Precision

Double-precision floating point is used throughout for geodetic accuracy, ensuring sub-meter precision for global positioning.

## Network Interfaces

### DIS Protocol (IEEE 1278)

| PDU Type | Direction | Purpose |
|----------|-----------|---------|
| Entity State | TX/RX | Position, velocity, orientation |
| Fire | TX/RX | Weapon discharge |
| Detonation | TX/RX | Impact/explosion |

### HLA Interface (IEEE 1516)

Supports RPR FOM 2.0 with object classes:

- BaseEntity.PhysicalEntity
- Platform.Aircraft/GroundVehicle/SurfaceVessel/Spacecraft
- Munition

## Technology Stack

| Component | Library | Version | Purpose |
|-----------|---------|---------|---------|
| Build | CMake | 3.25+ | Cross-platform build |
| Compiler | C++20 | - | Language standard |
| Math | Eigen | 3.4+ | Linear algebra |
| GIS | GDAL | 3.6+ | Geospatial data |
| XML | pugixml | 1.13+ | Configuration parsing |
| Testing | GoogleTest | 1.14+ | Unit testing |

## See Also

- [Custom Models](custom-models.md) - Creating custom physics models
- [Network Integration](networking.md) - DIS/HLA networking
- [API Reference](../api/overview.md) - Complete API documentation
