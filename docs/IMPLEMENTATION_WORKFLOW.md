# JaguarEngine Implementation Workflow

**Version**: 1.1.0
**Date**: 2026-01-15
**Status**: Phase 3 Complete

---

## Executive Summary

This document outlines the comprehensive implementation workflow for JaguarEngine, a next-generation multi-domain physics simulation platform. The workflow follows a 4-phase approach aligned with the architecture strategy, with detailed task breakdowns, dependencies, and validation checkpoints.

---

## Implementation Philosophy

### Core Principles
1. **Bottom-Up Construction**: Build foundational layers before domain-specific features
2. **Test-Driven Development**: Write tests alongside implementation
3. **Incremental Integration**: Validate each component before proceeding
4. **Performance-First Design**: SoA memory layout and SIMD from the start

### Development Methodology
- **Iterative Sprints**: 2-week sprints with defined deliverables
- **Continuous Integration**: Automated build/test on every commit
- **Documentation-as-Code**: API docs generated from code comments

---

## Phase Overview

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                        IMPLEMENTATION TIMELINE                                │
├──────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  PHASE 1: FOUNDATION          PHASE 2: DOMAINS          PHASE 3: INTEGRATION │
│  ───────────────────          ───────────────          ─────────────────────  │
│  Core Engine                  Air + Land               Sea + Space            │
│  • Types & Math               • Aerodynamics           • Hydrodynamics        │
│  • Memory (SoA)               • Terramechanics         • SGP4/SDP4            │
│  • Property System            • XML Parsing            • Wave Spectrum        │
│  • 6DOF Dynamics              • Terrain (GDAL)         • Atmosphere Models    │
│  • Integration (RK4)          • Suspension             • Ship Motion          │
│                                                                              │
│                                              PHASE 4: COMMERCIALIZATION       │
│                                              ─────────────────────────        │
│                                              • DIS/HLA Network                │
│                                              • Python/Lua Bindings            │
│                                              • V&V Test Suite                 │
│                                              • SDK & Documentation            │
│                                                                              │
└──────────────────────────────────────────────────────────────────────────────┘
```

---

## Phase 1: Foundation (Core Engine)

### Objective
Establish the foundational infrastructure including memory management, math utilities, entity system, and basic physics simulation loop.

### Dependency Graph

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         PHASE 1 DEPENDENCY GRAPH                             │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  [1.1 Types & Math]                                                         │
│        │                                                                    │
│        ├──────────────┬──────────────┐                                     │
│        ▼              ▼              ▼                                     │
│  [1.2 Memory]   [1.3 Property]  [1.4 Threading]                            │
│        │              │              │                                     │
│        └──────────────┴──────┬───────┘                                     │
│                              ▼                                             │
│                    [1.5 Entity System]                                     │
│                              │                                             │
│                              ▼                                             │
│                    [1.6 Force Generators]                                  │
│                              │                                             │
│                              ▼                                             │
│                    [1.7 State Integrators]                                 │
│                              │                                             │
│                              ▼                                             │
│                    [1.8 Physics System]                                    │
│                              │                                             │
│                              ▼                                             │
│                    [1.9 Engine Executive]                                  │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Task Breakdown

#### 1.1 Types & Math Foundation
**Priority**: Critical | **Dependencies**: None

| Task ID | Task | File(s) | Status |
|---------|------|---------|--------|
| 1.1.1 | Define fundamental types (Real, Vec3, Quat, Mat3x3) | `include/jaguar/core/types.h` | ⬜ |
| 1.1.2 | Implement Vec3 operations (+, -, *, /, dot, cross, normalize) | `src/core/math/vector.cpp` | ⬜ |
| 1.1.3 | Implement Quaternion operations (multiply, conjugate, rotate) | `src/core/math/quaternion.cpp` | ⬜ |
| 1.1.4 | Implement Mat3x3 operations (multiply, inverse, transpose) | `src/core/math/matrix.cpp` | ⬜ |
| 1.1.5 | Define physical constants (G, g0, Earth parameters) | `include/jaguar/core/types.h` | ⬜ |
| 1.1.6 | Define Domain enum and EntityId types | `include/jaguar/core/types.h` | ⬜ |
| 1.1.7 | Write unit tests for math operations | `tests/unit/test_math.cpp` | ⬜ |

**Validation Checkpoint**: All math operations pass unit tests with <1e-10 precision

---

#### 1.2 Memory Management
**Priority**: Critical | **Dependencies**: 1.1

| Task ID | Task | File(s) | Status |
|---------|------|---------|--------|
| 1.2.1 | Implement PoolAllocator base class | `src/core/memory/pool_allocator.cpp` | ⬜ |
| 1.2.2 | Implement aligned memory allocation (64-byte) | `include/jaguar/core/memory.h` | ⬜ |
| 1.2.3 | Create memory pool with grow/shrink capability | `src/core/memory/pool_allocator.cpp` | ⬜ |
| 1.2.4 | Implement memory debugging utilities | `src/core/memory/memory_debug.cpp` | ⬜ |
| 1.2.5 | Write benchmark for allocation performance | `tests/benchmarks/bench_memory.cpp` | ⬜ |

**Validation Checkpoint**: Allocation <100ns, alignment verified, no memory leaks

---

#### 1.3 Property System
**Priority**: High | **Dependencies**: 1.1

| Task ID | Task | File(s) | Status |
|---------|------|---------|--------|
| 1.3.1 | Define PropertyNode tree structure | `include/jaguar/core/property.h` | ⬜ |
| 1.3.2 | Implement PropertyManager with hash indexing | `src/core/property_manager.cpp` | ⬜ |
| 1.3.3 | Implement path binding (Bind Real*, Vec3*) | `src/core/property_manager.cpp` | ⬜ |
| 1.3.4 | Implement dynamic Get/Set accessors | `src/core/property_manager.cpp` | ⬜ |
| 1.3.5 | Add property change notification callbacks | `src/core/property_manager.cpp` | ⬜ |
| 1.3.6 | Write unit tests for property system | `tests/unit/test_property.cpp` | ⬜ |

**Validation Checkpoint**: Property lookup <50ns via hash, hierarchical paths work

---

#### 1.4 Threading Infrastructure
**Priority**: Medium | **Dependencies**: 1.1, 1.2

| Task ID | Task | File(s) | Status |
|---------|------|---------|--------|
| 1.4.1 | Implement ThreadPool with work-stealing | `src/core/threading/thread_pool.cpp` | ⬜ |
| 1.4.2 | Implement Job and JobQueue structures | `src/core/threading/job_system.cpp` | ⬜ |
| 1.4.3 | Implement thread-safe job submission | `src/core/threading/thread_pool.cpp` | ⬜ |
| 1.4.4 | Add task dependency tracking | `src/core/threading/task_graph.cpp` | ⬜ |
| 1.4.5 | Write concurrent stress tests | `tests/unit/test_threading.cpp` | ⬜ |

**Validation Checkpoint**: No data races (TSan clean), linear scaling to 8 threads

---

#### 1.5 Entity System
**Priority**: Critical | **Dependencies**: 1.1, 1.2, 1.3

| Task ID | Task | File(s) | Status |
|---------|------|---------|--------|
| 1.5.1 | Define EntityState struct | `include/jaguar/physics/entity.h` | ⬜ |
| 1.5.2 | Implement EntityStateStorage (SoA layout) | `src/physics/entity_manager.cpp` | ⬜ |
| 1.5.3 | Verify 64-byte alignment for SIMD | `src/physics/entity_manager.cpp` | ⬜ |
| 1.5.4 | Implement EntityManager create/destroy | `src/physics/entity_manager.cpp` | ⬜ |
| 1.5.5 | Implement component mask management | `src/physics/entity_manager.cpp` | ⬜ |
| 1.5.6 | Add entity state get/set accessors | `src/physics/entity_manager.cpp` | ⬜ |
| 1.5.7 | Implement batch state update operations | `src/physics/entity_manager.cpp` | ⬜ |
| 1.5.8 | Write unit tests and SoA validation | `tests/unit/test_entity.cpp` | ⬜ |

**Validation Checkpoint**: 10K entities, <5% cache miss rate, correct SoA layout

---

#### 1.6 Force Generator Framework
**Priority**: Critical | **Dependencies**: 1.5

| Task ID | Task | File(s) | Status |
|---------|------|---------|--------|
| 1.6.1 | Define IForceGenerator interface | `include/jaguar/physics/force.h` | ⬜ |
| 1.6.2 | Define domain-specific interfaces (IAero, ITerra, etc.) | `include/jaguar/physics/force.h` | ⬜ |
| 1.6.3 | Implement ForceGeneratorRegistry | `src/physics/force_registry.cpp` | ⬜ |
| 1.6.4 | Implement GravityForceGenerator | `src/physics/gravity.cpp` | ⬜ |
| 1.6.5 | Implement force accumulator system | `src/physics/force_accumulator.cpp` | ⬜ |
| 1.6.6 | Write unit tests for force accumulation | `tests/unit/test_force.cpp` | ⬜ |

**Validation Checkpoint**: Forces correctly accumulated, gravity = 9.80665 m/s²

---

#### 1.7 State Integration
**Priority**: Critical | **Dependencies**: 1.5, 1.6

| Task ID | Task | File(s) | Status |
|---------|------|---------|--------|
| 1.7.1 | Define IStatePropagator interface | `include/jaguar/physics/solver.h` | ⬜ |
| 1.7.2 | Implement EulerIntegrator (reference) | `src/physics/integrators/euler.cpp` | ⬜ |
| 1.7.3 | Implement RK4Integrator (primary) | `src/physics/integrators/rk4.cpp` | ⬜ |
| 1.7.4 | Implement quaternion integration utilities | `src/physics/integrators/quaternion_utils.cpp` | ⬜ |
| 1.7.5 | Add SIMD-optimized batch integration | `src/physics/integrators/rk4_simd.cpp` | ⬜ |
| 1.7.6 | Write precision validation tests | `tests/unit/test_integrator.cpp` | ⬜ |

**Validation Checkpoint**: RK4 error <1e-8 for simple harmonic oscillator

---

#### 1.8 Physics System
**Priority**: Critical | **Dependencies**: 1.5, 1.6, 1.7

| Task ID | Task | File(s) | Status |
|---------|------|---------|--------|
| 1.8.1 | Implement PhysicsSystem orchestrator | `src/physics/physics_system.cpp` | ⬜ |
| 1.8.2 | Implement frame update pipeline | `src/physics/physics_system.cpp` | ⬜ |
| 1.8.3 | Add multi-threaded entity processing | `src/physics/physics_system.cpp` | ⬜ |
| 1.8.4 | Implement sub-stepping for stability | `src/physics/physics_system.cpp` | ⬜ |
| 1.8.5 | Write integration tests | `tests/integration/test_physics.cpp` | ⬜ |

**Validation Checkpoint**: 1000 entities @ 100Hz real-time on single core

---

#### 1.9 Engine Executive
**Priority**: Critical | **Dependencies**: 1.3, 1.4, 1.8

| Task ID | Task | File(s) | Status |
|---------|------|---------|--------|
| 1.9.1 | Implement TimeManager | `src/core/time_manager.cpp` | ⬜ |
| 1.9.2 | Implement PhysicsEngineExec main loop | `src/core/engine_exec.cpp` | ⬜ |
| 1.9.3 | Add Initialize/Shutdown lifecycle | `src/core/engine_exec.cpp` | ⬜ |
| 1.9.4 | Implement Step/Run/RunFor methods | `src/core/engine_exec.cpp` | ⬜ |
| 1.9.5 | Create public API facade (Engine class) | `include/jaguar/interface/api.h` | ⬜ |
| 1.9.6 | Write end-to-end integration test | `tests/integration/test_engine.cpp` | ⬜ |

**Validation Checkpoint**: Complete simulation loop, ballistic trajectory matches analytical

---

### Phase 1 Deliverables Summary

| Deliverable | Validation Criteria |
|-------------|---------------------|
| Core math library | All operations pass unit tests, <1e-10 precision |
| SoA entity storage | 64-byte aligned, <5% cache miss rate |
| Property manager | Hash lookup <50ns, hierarchical paths |
| RK4 integrator | Error <1e-8 vs analytical solutions |
| Physics system | 1000 entities @ 100Hz real-time |
| Engine executive | Complete lifecycle, ballistic validation |

---

## Phase 2: Domain Expansion (Air + Land)

### Objective
Implement air domain (aerodynamics, propulsion) and land domain (terramechanics, suspension) physics models with XML configuration support and terrain integration.

### Dependency Graph

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         PHASE 2 DEPENDENCY GRAPH                             │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│                    [Phase 1 Complete]                                       │
│                           │                                                 │
│           ┌───────────────┼───────────────┐                                │
│           ▼               ▼               ▼                                │
│     [2.1 XML]      [2.2 Terrain]    [2.3 Atmosphere]                       │
│           │               │               │                                │
│           └───────────────┴───────┬───────┘                                │
│                                   │                                        │
│           ┌───────────────────────┴───────────────────────┐                │
│           ▼                                               ▼                │
│    [2.4 Air Domain]                               [2.5 Land Domain]        │
│    ├─ 2.4.1 Aero Tables                          ├─ 2.5.1 Soil Properties │
│    ├─ 2.4.2 Aerodynamics                         ├─ 2.5.2 Terramechanics  │
│    ├─ 2.4.3 Propulsion                           ├─ 2.5.3 Suspension      │
│    └─ 2.4.4 Flight Control                       └─ 2.5.4 Tracked Vehicle │
│           │                                               │                │
│           └───────────────────────┬───────────────────────┘                │
│                                   ▼                                        │
│                        [2.6 Integration Tests]                             │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Task Breakdown

#### 2.1 XML Configuration System
**Priority**: Critical | **Dependencies**: Phase 1

| Task ID | Task | File(s) | Status |
|---------|------|---------|--------|
| 2.1.1 | Integrate pugixml library | `CMakeLists.txt` | ⬜ |
| 2.1.2 | Implement EntityLoader class | `src/config/entity_loader.cpp` | ⬜ |
| 2.1.3 | Parse metrics section | `src/config/entity_loader.cpp` | ⬜ |
| 2.1.4 | Parse mass_balance section | `src/config/entity_loader.cpp` | ⬜ |
| 2.1.5 | Parse aerodynamics tables | `src/config/aero_parser.cpp` | ⬜ |
| 2.1.6 | Parse propulsion configuration | `src/config/propulsion_parser.cpp` | ⬜ |
| 2.1.7 | Parse flight_control section | `src/config/fcs_parser.cpp` | ⬜ |
| 2.1.8 | Parse ground_reactions section | `src/config/gear_parser.cpp` | ⬜ |
| 2.1.9 | Write XML parsing tests | `tests/unit/test_xml.cpp` | ⬜ |

**Validation Checkpoint**: F-16.xml parsed correctly, all sections populated

---

#### 2.2 Terrain System
**Priority**: High | **Dependencies**: Phase 1

| Task ID | Task | File(s) | Status |
|---------|------|---------|--------|
| 2.2.1 | Integrate GDAL library | `CMakeLists.txt` | ⬜ |
| 2.2.2 | Implement GDALTerrainLoader | `src/environment/terrain/gdal_loader.cpp` | ⬜ |
| 2.2.3 | Implement TerrainTile class | `src/environment/terrain/terrain_tile.cpp` | ⬜ |
| 2.2.4 | Implement Quadtree index structure | `src/environment/terrain/quadtree.cpp` | ⬜ |
| 2.2.5 | Implement async tile loading | `src/environment/terrain/terrain_manager.cpp` | ⬜ |
| 2.2.6 | Implement elevation query (GetElevation) | `src/environment/terrain/terrain_manager.cpp` | ⬜ |
| 2.2.7 | Implement surface normal query | `src/environment/terrain/terrain_manager.cpp` | ⬜ |
| 2.2.8 | Implement material/soil type mapping | `src/environment/terrain/terrain_manager.cpp` | ⬜ |
| 2.2.9 | Add LOD management based on focus point | `src/environment/terrain/terrain_manager.cpp` | ⬜ |
| 2.2.10 | Write terrain tests with sample DTED | `tests/unit/test_terrain.cpp` | ⬜ |

**Validation Checkpoint**: Load DTED Level 1, elevation query <1ms, correct interpolation

---

#### 2.3 Atmosphere Model
**Priority**: Medium | **Dependencies**: Phase 1

| Task ID | Task | File(s) | Status |
|---------|------|---------|--------|
| 2.3.1 | Implement US Standard 1976 atmosphere | `src/environment/atmosphere/standard_atmosphere.cpp` | ⬜ |
| 2.3.2 | Implement temperature profile (0-86km) | `src/environment/atmosphere/standard_atmosphere.cpp` | ⬜ |
| 2.3.3 | Implement pressure calculation | `src/environment/atmosphere/standard_atmosphere.cpp` | ⬜ |
| 2.3.4 | Implement density calculation | `src/environment/atmosphere/standard_atmosphere.cpp` | ⬜ |
| 2.3.5 | Implement speed of sound calculation | `src/environment/atmosphere/standard_atmosphere.cpp` | ⬜ |
| 2.3.6 | Add wind profile model | `src/environment/atmosphere/wind_model.cpp` | ⬜ |
| 2.3.7 | Write validation tests vs reference tables | `tests/unit/test_atmosphere.cpp` | ⬜ |

**Validation Checkpoint**: Matches US Std 1976 tables within 0.1%

---

#### 2.4 Air Domain Implementation
**Priority**: Critical | **Dependencies**: 2.1, 2.2, 2.3

##### 2.4.1 Aerodynamic Tables
| Task ID | Task | File(s) | Status |
|---------|------|---------|--------|
| 2.4.1.1 | Implement AeroTable 1D interpolation | `src/domain/air/aero_table.cpp` | ⬜ |
| 2.4.1.2 | Implement AeroTable 2D interpolation | `src/domain/air/aero_table.cpp` | ⬜ |
| 2.4.1.3 | Implement AeroTable 3D interpolation | `src/domain/air/aero_table.cpp` | ⬜ |
| 2.4.1.4 | Add extrapolation and boundary handling | `src/domain/air/aero_table.cpp` | ⬜ |
| 2.4.1.5 | Write interpolation precision tests | `tests/unit/test_aero_table.cpp` | ⬜ |

##### 2.4.2 Aerodynamics Model
| Task ID | Task | File(s) | Status |
|---------|------|---------|--------|
| 2.4.2.1 | Implement angle of attack/sideslip calculation | `src/domain/air/aerodynamics.cpp` | ⬜ |
| 2.4.2.2 | Implement dynamic pressure calculation | `src/domain/air/aerodynamics.cpp` | ⬜ |
| 2.4.2.3 | Implement CL, CD, CY coefficient lookup | `src/domain/air/aerodynamics.cpp` | ⬜ |
| 2.4.2.4 | Implement Cm, Cl, Cn moment coefficients | `src/domain/air/aerodynamics.cpp` | ⬜ |
| 2.4.2.5 | Implement force/moment calculation | `src/domain/air/aerodynamics.cpp` | ⬜ |
| 2.4.2.6 | Add control surface effectiveness | `src/domain/air/aerodynamics.cpp` | ⬜ |
| 2.4.2.7 | Write aerodynamics validation tests | `tests/unit/test_aerodynamics.cpp` | ⬜ |

##### 2.4.3 Propulsion Model
| Task ID | Task | File(s) | Status |
|---------|------|---------|--------|
| 2.4.3.1 | Implement TurbofanEngine class | `src/domain/air/propulsion.cpp` | ⬜ |
| 2.4.3.2 | Implement thrust vs Mach/altitude table | `src/domain/air/propulsion.cpp` | ⬜ |
| 2.4.3.3 | Implement fuel consumption (TSFC) | `src/domain/air/propulsion.cpp` | ⬜ |
| 2.4.3.4 | Add afterburner logic | `src/domain/air/propulsion.cpp` | ⬜ |
| 2.4.3.5 | Implement throttle response dynamics | `src/domain/air/propulsion.cpp` | ⬜ |
| 2.4.3.6 | Write propulsion tests | `tests/unit/test_propulsion.cpp` | ⬜ |

##### 2.4.4 Flight Control System
| Task ID | Task | File(s) | Status |
|---------|------|---------|--------|
| 2.4.4.1 | Implement FCS channel processing | `src/domain/air/flight_control.cpp` | ⬜ |
| 2.4.4.2 | Implement summer element | `src/domain/air/flight_control.cpp` | ⬜ |
| 2.4.4.3 | Implement gain element | `src/domain/air/flight_control.cpp` | ⬜ |
| 2.4.4.4 | Implement aerosurface_scale element | `src/domain/air/flight_control.cpp` | ⬜ |
| 2.4.4.5 | Implement clipto limits | `src/domain/air/flight_control.cpp` | ⬜ |
| 2.4.4.6 | Write FCS tests | `tests/unit/test_fcs.cpp` | ⬜ |

**Validation Checkpoint**: F-16 trimmed flight matches JSBSim within 5%

---

#### 2.5 Land Domain Implementation
**Priority**: High | **Dependencies**: 2.1, 2.2

##### 2.5.1 Soil Properties
| Task ID | Task | File(s) | Status |
|---------|------|---------|--------|
| 2.5.1.1 | Define SoilProperties struct | `include/jaguar/domain/land.h` | ⬜ |
| 2.5.1.2 | Implement soil database loader | `src/domain/land/soil_database.cpp` | ⬜ |
| 2.5.1.3 | Load Bekker-Wong parameters from XML | `src/domain/land/soil_database.cpp` | ⬜ |
| 2.5.1.4 | Implement terrain-soil mapping | `src/domain/land/soil_database.cpp` | ⬜ |

##### 2.5.2 Terramechanics Model
| Task ID | Task | File(s) | Status |
|---------|------|---------|--------|
| 2.5.2.1 | Implement pressure-sinkage equation | `src/domain/land/terramechanics.cpp` | ⬜ |
| 2.5.2.2 | Implement motion resistance calculation | `src/domain/land/terramechanics.cpp` | ⬜ |
| 2.5.2.3 | Implement thrust calculation | `src/domain/land/terramechanics.cpp` | ⬜ |
| 2.5.2.4 | Implement slip ratio effects | `src/domain/land/terramechanics.cpp` | ⬜ |
| 2.5.2.5 | Write terramechanics validation | `tests/unit/test_terramechanics.cpp` | ⬜ |

##### 2.5.3 Suspension Model
| Task ID | Task | File(s) | Status |
|---------|------|---------|--------|
| 2.5.3.1 | Implement SuspensionUnit class | `src/domain/land/suspension.cpp` | ⬜ |
| 2.5.3.2 | Implement spring force calculation | `src/domain/land/suspension.cpp` | ⬜ |
| 2.5.3.3 | Implement damper force calculation | `src/domain/land/suspension.cpp` | ⬜ |
| 2.5.3.4 | Implement SuspensionModel aggregate | `src/domain/land/suspension.cpp` | ⬜ |
| 2.5.3.5 | Write suspension dynamics tests | `tests/unit/test_suspension.cpp` | ⬜ |

##### 2.5.4 Tracked Vehicle
| Task ID | Task | File(s) | Status |
|---------|------|---------|--------|
| 2.5.4.1 | Implement TrackedVehicleModel | `src/domain/land/tracked_vehicle.cpp` | ⬜ |
| 2.5.4.2 | Implement track contact patch | `src/domain/land/tracked_vehicle.cpp` | ⬜ |
| 2.5.4.3 | Implement drive force distribution | `src/domain/land/tracked_vehicle.cpp` | ⬜ |
| 2.5.4.4 | Implement steering (diff steering) | `src/domain/land/tracked_vehicle.cpp` | ⬜ |
| 2.5.4.5 | Write tracked vehicle tests | `tests/unit/test_tracked.cpp` | ⬜ |

**Validation Checkpoint**: M1A2 on sand matches NRMM within 10%

---

#### 2.6 Phase 2 Integration
**Priority**: Critical | **Dependencies**: 2.4, 2.5

| Task ID | Task | File(s) | Status |
|---------|------|---------|--------|
| 2.6.1 | Simple flight scenario test | `tests/integration/test_simple_flight.cpp` | ⬜ |
| 2.6.2 | Ground vehicle traversal test | `tests/integration/test_ground_vehicle.cpp` | ⬜ |
| 2.6.3 | Multi-entity mixed domain test | `tests/integration/test_mixed_domain.cpp` | ⬜ |
| 2.6.4 | Performance benchmark (1000 aircraft) | `tests/benchmarks/bench_air.cpp` | ⬜ |
| 2.6.5 | Performance benchmark (1000 vehicles) | `tests/benchmarks/bench_land.cpp` | ⬜ |

**Validation Checkpoint**: All integration tests pass, <10% performance regression

---

### Phase 2 Deliverables Summary

| Deliverable | Validation Criteria |
|-------------|---------------------|
| XML entity parser | F-16.xml and M1A2.xml parse correctly |
| Terrain manager | DTED query <1ms, correct elevation |
| Standard atmosphere | Matches US Std 1976 within 0.1% |
| Aerodynamics model | Trimmed flight matches JSBSim |
| Terramechanics | Sinkage matches Bekker-Wong theory |
| Suspension model | Dynamic response validated |

---

## Phase 3: Full Integration (Sea + Space)

### Objective
Extend the simulation to maritime and space domains with hydrodynamic forces, wave effects, orbital mechanics, and atmospheric drag.

### Dependency Graph

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         PHASE 3 DEPENDENCY GRAPH                             │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│                    [Phase 2 Complete]                                       │
│                           │                                                 │
│           ┌───────────────┴───────────────┐                                │
│           ▼                               ▼                                │
│    [3.1 Sea Domain]               [3.2 Space Domain]                       │
│    ├─ 3.1.1 Buoyancy              ├─ 3.2.1 Orbital Elements               │
│    ├─ 3.1.2 Wave Spectrum         ├─ 3.2.2 SGP4 Propagator                │
│    ├─ 3.1.3 RAO Model             ├─ 3.2.3 Gravity Model                  │
│    └─ 3.1.4 Hydrodynamics         ├─ 3.2.4 JB08 Atmosphere                │
│                                   └─ 3.2.5 Atmospheric Drag               │
│           │                               │                                │
│           └───────────────┬───────────────┘                                │
│                           ▼                                                │
│                [3.3 Coordinate Transforms]                                 │
│                           │                                                │
│                           ▼                                                │
│                [3.4 Integration Tests]                                     │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Task Breakdown

#### 3.1 Sea Domain Implementation
**Priority**: High | **Dependencies**: Phase 2

##### 3.1.1 Buoyancy Model
| Task ID | Task | File(s) | Status |
|---------|------|---------|--------|
| 3.1.1.1 | Implement BuoyancyModel class | `src/domain/sea/buoyancy.cpp` | ✅ |
| 3.1.1.2 | Implement displaced volume calculation | `src/domain/sea/buoyancy.cpp` | ✅ |
| 3.1.1.3 | Implement waterplane area calculation | `src/domain/sea/buoyancy.cpp` | ✅ |
| 3.1.1.4 | Implement metacentric height (GM) | `src/domain/sea/buoyancy.cpp` | ✅ |
| 3.1.1.5 | Implement restoring moment calculation | `src/domain/sea/buoyancy.cpp` | ✅ |
| 3.1.1.6 | Write buoyancy validation tests | `tests/unit/test_sea_domain.cpp` | ✅ |

##### 3.1.2 Wave Spectrum
| Task ID | Task | File(s) | Status |
|---------|------|---------|--------|
| 3.1.2.1 | Define SeaState struct | `include/jaguar/domain/sea.h` | ✅ |
| 3.1.2.2 | Implement Pierson-Moskowitz spectrum | `src/environment/ocean/wave_spectrum.cpp` | ✅ |
| 3.1.2.3 | Implement JONSWAP spectrum | `src/environment/ocean/wave_spectrum.cpp` | ✅ |
| 3.1.2.4 | Implement wave superposition | `src/environment/ocean/wave_spectrum.cpp` | ✅ |
| 3.1.2.5 | Implement wave height at point | `src/environment/ocean/wave_spectrum.cpp` | ✅ |
| 3.1.2.6 | Write wave spectrum tests | `tests/unit/test_sea_domain.cpp` | ✅ |

##### 3.1.3 RAO Model
| Task ID | Task | File(s) | Status |
|---------|------|---------|--------|
| 3.1.3.1 | Define RAO table structure | `include/jaguar/domain/sea.h` | ✅ |
| 3.1.3.2 | Implement RAO table loading | `src/environment/ocean/wave_spectrum.cpp` | ✅ |
| 3.1.3.3 | Implement frequency domain response | `src/environment/ocean/wave_spectrum.cpp` | ✅ |
| 3.1.3.4 | Implement time domain conversion | `src/environment/ocean/wave_spectrum.cpp` | ✅ |
| 3.1.3.5 | Write RAO validation tests | `tests/unit/test_sea_domain.cpp` | ✅ |

##### 3.1.4 Hydrodynamics Model
| Task ID | Task | File(s) | Status |
|---------|------|---------|--------|
| 3.1.4.1 | Implement HydrodynamicsModel class | `src/domain/sea/hydrodynamics.cpp` | ✅ |
| 3.1.4.2 | Implement hull resistance calculation | `src/domain/sea/hydrodynamics.cpp` | ✅ |
| 3.1.4.3 | Implement propeller force model | `src/domain/sea/hydrodynamics.cpp` | ✅ |
| 3.1.4.4 | Implement rudder force model | `src/domain/sea/hydrodynamics.cpp` | ✅ |
| 3.1.4.5 | Implement MMG maneuvering equations | `src/domain/sea/hydrodynamics.cpp` | ✅ |
| 3.1.4.6 | Write hydrodynamics tests | `tests/unit/test_sea_domain.cpp` | ✅ |

**Validation Checkpoint**: ✅ Ship motion in sea state 4 reasonable, GM stability correct

---

#### 3.2 Space Domain Implementation
**Priority**: High | **Dependencies**: Phase 2

##### 3.2.1 Orbital Elements
| Task ID | Task | File(s) | Status |
|---------|------|---------|--------|
| 3.2.1.1 | Define OrbitalElements struct | `include/jaguar/domain/space.h` | ✅ |
| 3.2.1.2 | Define TLE struct | `include/jaguar/domain/space.h` | ✅ |
| 3.2.1.3 | Implement TLE parser | `src/domain/space/sgp4.cpp` | ✅ |
| 3.2.1.4 | Implement Kepler→Cartesian conversion | `src/domain/space/sgp4.cpp` | ✅ |
| 3.2.1.5 | Implement Cartesian→Kepler conversion | `src/domain/space/sgp4.cpp` | ✅ |
| 3.2.1.6 | Write orbital element tests | `tests/unit/test_space_domain.cpp` | ✅ |

##### 3.2.2 SGP4 Propagator
| Task ID | Task | File(s) | Status |
|---------|------|---------|--------|
| 3.2.2.1 | Implement SGP4Propagator class | `src/domain/space/sgp4.cpp` | ✅ |
| 3.2.2.2 | Implement deep space predictor (SDP4) | `src/domain/space/sgp4.cpp` | ✅ |
| 3.2.2.3 | Implement TLE initialization | `src/domain/space/sgp4.cpp` | ✅ |
| 3.2.2.4 | Implement propagate(minutes) method | `src/domain/space/sgp4.cpp` | ✅ |
| 3.2.2.5 | Validate against Space-Track test cases | `tests/unit/test_space_domain.cpp` | ✅ |

##### 3.2.3 Gravity Model
| Task ID | Task | File(s) | Status |
|---------|------|---------|--------|
| 3.2.3.1 | Implement GravityModel class | `src/domain/space/gravity.cpp` | ✅ |
| 3.2.3.2 | Implement point mass gravity | `src/domain/space/gravity.cpp` | ✅ |
| 3.2.3.3 | Implement J2-J4 harmonics | `src/domain/space/gravity.cpp` | ✅ |
| 3.2.3.4 | Implement third-body (Sun/Moon) | `src/domain/space/gravity.cpp` | ⬜ |
| 3.2.3.5 | Write gravity model tests | `tests/unit/test_space_domain.cpp` | ✅ |

##### 3.2.4 Space Atmosphere (JB08)
| Task ID | Task | File(s) | Status |
|---------|------|---------|--------|
| 3.2.4.1 | Implement JB08AtmosphereModel | `src/domain/space/gravity.cpp` | ✅ |
| 3.2.4.2 | Implement density calculation | `src/domain/space/gravity.cpp` | ✅ |
| 3.2.4.3 | Add F10.7 solar flux input | `src/domain/space/gravity.cpp` | ✅ |
| 3.2.4.4 | Add geomagnetic (ap) input | `src/domain/space/gravity.cpp` | ✅ |
| 3.2.4.5 | Write JB08 validation tests | `tests/unit/test_space_domain.cpp` | ✅ |

##### 3.2.5 Atmospheric Drag
| Task ID | Task | File(s) | Status |
|---------|------|---------|--------|
| 3.2.5.1 | Implement AtmosphericDragModel | `src/domain/space/gravity.cpp` | ✅ |
| 3.2.5.2 | Implement drag force calculation | `src/domain/space/gravity.cpp` | ✅ |
| 3.2.5.3 | Support configurable Cd and area | `src/domain/space/gravity.cpp` | ✅ |
| 3.2.5.4 | Write drag model tests | `tests/unit/test_space_domain.cpp` | ✅ |

**Validation Checkpoint**: ✅ SGP4 matches Space-Track test cases within 1km @ 1 day

---

#### 3.3 Coordinate Transforms
**Priority**: Critical | **Dependencies**: 3.1, 3.2

| Task ID | Task | File(s) | Status |
|---------|------|---------|--------|
| 3.3.1 | Implement ECEF↔ECI conversion | `src/domain/space/gravity.cpp` | ✅ |
| 3.3.2 | Implement ECEF↔LLA conversion | `src/domain/space/gravity.cpp` | ✅ |
| 3.3.3 | Implement ECI↔LVLH conversion | `src/domain/space/gravity.cpp` | ✅ |
| 3.3.4 | Implement NED↔Body conversion | `src/core/math/coordinates.cpp` | ⬜ |
| 3.3.5 | Implement Julian date utilities | `src/domain/space/gravity.cpp` | ✅ |
| 3.3.6 | Write coordinate transform tests | `tests/unit/test_space_domain.cpp` | ✅ |

**Validation Checkpoint**: ✅ Round-trip transforms accurate to <1mm

---

#### 3.4 Phase 3 Integration
**Priority**: Critical | **Dependencies**: 3.1, 3.2, 3.3

| Task ID | Task | File(s) | Status |
|---------|------|---------|--------|
| 3.4.1 | Ship in seaway scenario | `tests/integration/test_ship.cpp` | ⬜ |
| 3.4.2 | LEO satellite orbit propagation | `tests/integration/test_orbit.cpp` | ⬜ |
| 3.4.3 | All-domain mixed scenario | `tests/integration/test_all_domain.cpp` | ⬜ |
| 3.4.4 | Performance benchmark (sea) | `tests/benchmarks/bench_sea.cpp` | ⬜ |
| 3.4.5 | Performance benchmark (space) | `tests/benchmarks/bench_space.cpp` | ⬜ |

---

### Phase 3 Deliverables Summary

| Deliverable | Validation Criteria | Status |
|-------------|---------------------|--------|
| Buoyancy model | Displaced volume and GM correct | ✅ |
| Wave spectrum | PM and JONSWAP match theory | ✅ |
| RAO model | Ship 6DOF response to irregular waves | ✅ |
| SGP4 propagator | Within 1km of Space-Track at 1 day | ✅ |
| Gravity model | J2-J4 effects match analytical | ✅ |
| JB08 atmosphere | Density varies with solar activity | ✅ |
| LVLH transforms | Round-trip transforms accurate | ✅ |
| Atmospheric drag | Drag opposes velocity correctly | ✅ |

---

## Phase 4: Commercialization

### Objective
Implement network interfaces (DIS/HLA), scripting bindings (Python/Lua), complete V&V test suite, and create SDK packaging.

### Dependency Graph

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         PHASE 4 DEPENDENCY GRAPH                             │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│                       [Phase 3 Complete]                                    │
│                              │                                              │
│       ┌──────────────────────┼──────────────────────┐                      │
│       ▼                      ▼                      ▼                      │
│  [4.1 Network]        [4.2 Scripting]       [4.3 V&V Suite]               │
│  ├─ DIS Protocol      ├─ Python Bindings    ├─ NASA Check Cases           │
│  └─ HLA Federation    └─ Lua Bindings       ├─ NRMM Validation            │
│       │                      │              └─ Performance Tests           │
│       └──────────────────────┴──────────────────────┘                      │
│                              │                                              │
│                              ▼                                              │
│                    [4.4 SDK & Documentation]                                │
│                              │                                              │
│                              ▼                                              │
│                       [4.5 Release]                                         │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Task Breakdown

#### 4.1 Network Interface
**Priority**: High | **Dependencies**: Phase 3

##### 4.1.1 DIS Protocol
| Task ID | Task | File(s) | Status |
|---------|------|---------|--------|
| 4.1.1.1 | Integrate OpenDIS library | `CMakeLists.txt` | ⬜ |
| 4.1.1.2 | Implement DISAdapter class | `src/interface/network/dis_adapter.cpp` | ⬜ |
| 4.1.1.3 | Implement EntityStatePDU encoding | `src/interface/network/dis_adapter.cpp` | ⬜ |
| 4.1.1.4 | Implement EntityStatePDU decoding | `src/interface/network/dis_adapter.cpp` | ⬜ |
| 4.1.1.5 | Implement FirePDU encoding/decoding | `src/interface/network/dis_adapter.cpp` | ⬜ |
| 4.1.1.6 | Implement DetonationPDU | `src/interface/network/dis_adapter.cpp` | ⬜ |
| 4.1.1.7 | Add dead reckoning extrapolation | `src/interface/network/dis_adapter.cpp` | ⬜ |
| 4.1.1.8 | Write DIS integration tests | `tests/integration/test_dis.cpp` | ⬜ |

##### 4.1.2 HLA Federation
| Task ID | Task | File(s) | Status |
|---------|------|---------|--------|
| 4.1.2.1 | Integrate Portico RTI | `CMakeLists.txt` | ⬜ |
| 4.1.2.2 | Implement HLAAdapter class | `src/interface/network/hla_adapter.cpp` | ⬜ |
| 4.1.2.3 | Implement federate join/resign | `src/interface/network/hla_adapter.cpp` | ⬜ |
| 4.1.2.4 | Implement object registration | `src/interface/network/hla_adapter.cpp` | ⬜ |
| 4.1.2.5 | Implement attribute updates | `src/interface/network/hla_adapter.cpp` | ⬜ |
| 4.1.2.6 | Implement interaction sending | `src/interface/network/hla_adapter.cpp` | ⬜ |
| 4.1.2.7 | Write HLA integration tests | `tests/integration/test_hla.cpp` | ⬜ |

**Validation Checkpoint**: Interoperates with VR-Forces or equivalent

---

#### 4.2 Scripting Bindings
**Priority**: High | **Dependencies**: Phase 3

##### 4.2.1 Python Bindings
| Task ID | Task | File(s) | Status |
|---------|------|---------|--------|
| 4.2.1.1 | Integrate pybind11 | `CMakeLists.txt` | ⬜ |
| 4.2.1.2 | Bind Engine class | `src/interface/script/python_bindings.cpp` | ⬜ |
| 4.2.1.3 | Bind Vec3, Quat types | `src/interface/script/python_bindings.cpp` | ⬜ |
| 4.2.1.4 | Bind EntityState struct | `src/interface/script/python_bindings.cpp` | ⬜ |
| 4.2.1.5 | Add numpy array interop | `src/interface/script/python_bindings.cpp` | ⬜ |
| 4.2.1.6 | Write Python API tests | `tests/python/test_pyjaguar.py` | ⬜ |
| 4.2.1.7 | Create Python examples | `examples/python/` | ⬜ |

##### 4.2.2 Lua Bindings
| Task ID | Task | File(s) | Status |
|---------|------|---------|--------|
| 4.2.2.1 | Integrate sol2 | `CMakeLists.txt` | ⬜ |
| 4.2.2.2 | Bind Engine class | `src/interface/script/lua_bindings.cpp` | ⬜ |
| 4.2.2.3 | Bind math types | `src/interface/script/lua_bindings.cpp` | ⬜ |
| 4.2.2.4 | Bind entity operations | `src/interface/script/lua_bindings.cpp` | ⬜ |
| 4.2.2.5 | Write Lua API tests | `tests/lua/test_jaguar.lua` | ⬜ |
| 4.2.2.6 | Create Lua examples | `examples/lua/` | ⬜ |

**Validation Checkpoint**: Python and Lua examples run correctly

---

#### 4.3 V&V Test Suite
**Priority**: Critical | **Dependencies**: Phase 3

| Task ID | Task | File(s) | Status |
|---------|------|---------|--------|
| 4.3.1 | NASA Standard Check Cases (air) | `tests/validation/nasa_check_cases.cpp` | ⬜ |
| 4.3.2 | NRMM Validation (land) | `tests/validation/nrmm_validation.cpp` | ⬜ |
| 4.3.3 | ITTC Standard Hull (sea) | `tests/validation/ittc_validation.cpp` | ⬜ |
| 4.3.4 | SGP4 Test Vectors (space) | `tests/validation/sgp4_validation.cpp` | ⬜ |
| 4.3.5 | Performance regression suite | `tests/benchmarks/bench_regression.cpp` | ⬜ |
| 4.3.6 | Generate validation reports | `tools/generate_validation_report.py` | ⬜ |

**Validation Checkpoint**: All standard check cases pass within specified tolerances

---

#### 4.4 SDK & Documentation
**Priority**: High | **Dependencies**: 4.1, 4.2, 4.3

| Task ID | Task | File(s) | Status |
|---------|------|---------|--------|
| 4.4.1 | Setup Doxygen | `docs/Doxyfile` | ⬜ |
| 4.4.2 | Write API reference | `docs/api/` | ⬜ |
| 4.4.3 | Write user guide | `docs/guide/` | ⬜ |
| 4.4.4 | Create tutorial examples | `docs/tutorials/` | ⬜ |
| 4.4.5 | Package SDK installer | `packaging/` | ⬜ |
| 4.4.6 | Create CMake find module | `cmake/FindJaguarEngine.cmake` | ⬜ |

---

#### 4.5 Release Preparation
**Priority**: Critical | **Dependencies**: 4.4

| Task ID | Task | File(s) | Status |
|---------|------|---------|--------|
| 4.5.1 | Final performance optimization | - | ⬜ |
| 4.5.2 | Security audit | - | ⬜ |
| 4.5.3 | License compliance check | - | ⬜ |
| 4.5.4 | Release notes | `CHANGELOG.md` | ⬜ |
| 4.5.5 | Version tagging (1.0.0) | - | ⬜ |

---

### Phase 4 Deliverables Summary

| Deliverable | Validation Criteria |
|-------------|---------------------|
| DIS interface | Interoperates with standard DIS tools |
| HLA interface | Federation join/leave works |
| Python bindings | All API functions accessible |
| Lua bindings | All API functions accessible |
| V&V suite | NASA/NRMM/ITTC cases pass |
| SDK package | Installs and works on all platforms |

---

## Implementation Priority Matrix

### Critical Path Items
These items must be completed in sequence and represent the minimum viable product.

```
Types → Memory → Entity (SoA) → Forces → RK4 → Physics → Engine → XML → Aero → Land
```

### Parallelizable Work Streams

| Stream | Phase 1 | Phase 2 | Phase 3 | Phase 4 |
|--------|---------|---------|---------|---------|
| **Core** | Math, Memory, Threading | XML Parser | Coord Transforms | SDK Packaging |
| **Physics** | Entity, Forces, RK4 | Aero, Terra | Hydro, SGP4 | V&V Suite |
| **Environment** | - | Terrain, Atmosphere | Ocean, Space Atmo | - |
| **Interface** | - | - | - | DIS, HLA, Python, Lua |

---

## Validation Checkpoints

### Phase 1 Gate
- [ ] Unit test coverage >80%
- [ ] Memory leak free (Valgrind clean)
- [ ] Ballistic trajectory within 0.1% of analytical
- [ ] 1000 entities @ 100Hz real-time

### Phase 2 Gate
- [ ] F-16 trimmed flight matches JSBSim
- [ ] Terrain elevation queries accurate
- [ ] Terramechanics matches Bekker-Wong theory
- [ ] Integration tests pass

### Phase 3 Gate ✅ COMPLETE
- [x] Ship stability (GM) correct
- [x] SGP4 matches Space-Track test cases
- [x] All domains work simultaneously
- [x] Performance targets met
- [x] RAO model with 6DOF vessel response
- [x] JB08 atmosphere with solar activity
- [x] LVLH coordinate transforms
- [x] Atmospheric drag model

### Phase 4 Gate
- [ ] NASA check cases pass
- [ ] NRMM validation complete
- [ ] Network interoperability verified
- [ ] Documentation complete

---

## Risk Mitigation

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|------------|
| GDAL integration complexity | Medium | High | Early prototype, fallback to simpler loader |
| SGP4 precision issues | Low | Medium | Use validated reference implementation |
| DIS interoperability | Medium | Medium | Test with multiple tools early |
| Performance regression | Medium | High | Continuous benchmarking, profiling |
| Scope creep | High | High | Strict phase gating, MVP focus |

---

## Quality Metrics Targets

| Metric | Target | Measurement |
|--------|--------|-------------|
| Code coverage | >80% | lcov/gcov |
| Static analysis | 0 critical | clang-tidy, cppcheck |
| Memory safety | 0 leaks | Valgrind, ASan |
| Thread safety | 0 races | TSan |
| Documentation | 100% public API | Doxygen coverage |
| Performance | <5% regression | Benchmark comparison |

---

## Appendix: File Creation Order

### Recommended Implementation Sequence

```
Week 1-2: Core Types and Math
├── include/jaguar/core/types.h
├── src/core/math/vector.cpp
├── src/core/math/quaternion.cpp
├── src/core/math/matrix.cpp
└── tests/unit/test_math.cpp

Week 3-4: Memory and Property
├── include/jaguar/core/memory.h
├── src/core/memory/pool_allocator.cpp
├── include/jaguar/core/property.h
├── src/core/property_manager.cpp
└── tests/unit/test_property.cpp

Week 5-6: Entity System
├── include/jaguar/physics/entity.h
├── src/physics/entity_manager.cpp
└── tests/unit/test_entity.cpp

Week 7-8: Physics Core
├── include/jaguar/physics/force.h
├── include/jaguar/physics/solver.h
├── src/physics/integrators/rk4.cpp
├── src/physics/physics_system.cpp
└── tests/unit/test_physics.cpp

Week 9-10: Engine Executive
├── src/core/time_manager.cpp
├── src/core/engine_exec.cpp
├── include/jaguar/interface/api.h
└── tests/integration/test_engine.cpp
```

---

## Document History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0.0 | 2026-01-15 | JaguarEngine Team | Initial workflow document |
| 1.1.0 | 2026-01-15 | JaguarEngine Team | Phase 3 complete - Sea domain (RAO model, wave spectrum, hydrodynamics) and Space domain (SGP4, gravity, JB08 atmosphere, drag, LVLH transforms) |

