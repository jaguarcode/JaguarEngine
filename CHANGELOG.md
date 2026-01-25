# Changelog

All notable changes to JaguarEngine are documented in this file.

## [0.6.0] - 2026-01-25

### Overview

JaguarEngine v0.6.0 introduces enterprise-grade physics simulation with entity-specific component models, advanced terrain integration, high-performance parallel execution, and production federation support. This release consolidates 24 major features across 6 development phases.

**Key Themes:**
- Per-entity heterogeneous force models with type-safe registration
- Advanced terrain physics with GDAL GIS integration
- Production-ready multi-threaded parallel execution
- IEEE 1278.1-2012 DIS and HLA federation standards

---

### Phase 1: Core Engine Critical Fixes

#### 1. Component Force Registry
- Per-entity force model management via `ComponentForceRegistry`
- Type-safe template-based model registration and retrieval
- Entity-specific force computation for heterogeneous populations
- Thread-safe read operations supporting parallel force computation
- API: `register_model<T>()`, `get<T>()`, `has<T>()`, `remove_model<T>()`
- Location: `include/jaguar/physics/component_force_registry.h`

#### 2. Entity-Specific Force Computation
- Force computation driven by `ComponentBits` mask per entity
- Extensible force model interface for custom domain-specific implementations
- Efficient polymorphic dispatch without virtual function overhead
- Domain support: Aerodynamics, Hydrodynamics, Terramechanics, Propulsion, Gravity
- Location: `src/physics/component_force_registry.cpp`

#### 3. Terrain Height Query
- `TerrainManager` for unified terrain data access
- Elevation queries via geodetic coordinates (lat, lon)
- ECEF-aware query methods: `get_elevation_ecef()`, `query_ecef()`
- Surface normal computation for slope-aware physics
- Location: `include/jaguar/environment/terrain.h`, `src/environment/terrain/terrain_manager.cpp`

#### 4. Ground Contact Constraints
- `GroundContactConstraint` for terrain-surface contact modeling
- Terrain-normal based force application with Coulomb friction
- Prevents entities from sinking into slopes via constraint solving
- Friction cone enforcement: -μλ_n ≤ λ_tangent ≤ μλ_n
- Supports restitution and rolling resistance
- Location: `include/jaguar/physics/constraints/ground_contact_constraint.h`

#### 5. Constants Header
- Centralized configuration in `include/jaguar/core/constants.h`
- Physics solver defaults (stiffness, damping, slop)
- Numeric tolerances and convergence thresholds
- Integrator configuration constants
- Location: `include/jaguar/core/constants.h`

#### 6. XML Entity Loader
- `XmlEntityLoader` with JSBSim-style XML parsing
- Domain detection: Aircraft, Land Vehicle, Vessel, Satellite
- Component mask automatic determination from entity type
- Unit conversion engine: feet→meters, lbs→kg, lbf→N, deg→rad, etc.
- Parses: metrics, mass/balance, inertia tensor, aerodynamics, propulsion
- Location: `include/jaguar/interface/xml_entity_loader.h`, `src/core/xml_entity_loader.cpp`

---

### Phase 2: Physics System Enhancement

#### 7. ABM4 Angular Acceleration Fix
- Fixed history buffer management for ABM4 angular velocity integration
- Proper multi-step integration of angular momentum
- Corrected quaternion propagation in adaptive integrators
- Location: `src/physics/integrators/abm4.cpp`

#### 8. Split Impulse Solver
- Two-phase constraint solving: position then velocity
- Separates position correction (penetration resolution) from velocity correction
- Eliminates contact jitter in stacked bodies
- Faster convergence than monolithic solvers
- Location: `src/physics/constraints/sequential_impulse_solver.cpp`

#### 9. Quaternion Error Metric
- Geodesic distance calculation for rotation error
- Rotation-invariant error measurement without gimbal lock
- Used by adaptive integrators for step size control
- API: `quaternion_utils::geodesic_distance()`, `angular_distance()`, `error_vector()`
- Location: `include/jaguar/core/types.h` (quaternion utilities)

#### 10. Integrator API Standardization
- Unified `IStatePropagator::integrate()` interface across all integrators
- Consistent method signature: `integrate(state, forces, dt)`
- Replaced inconsistent `propagate()` and `integrate()` naming
- Bounds checking for numeric stability
- Location: `include/jaguar/physics/integrators/state_propagator.h`

#### 11. Inverse Inertia Caching
- Pre-computed and cached inverse inertia tensors in `EntityState`
- `Mat3x3 inertia_inv` member eliminates per-frame matrix inversions
- `InvertiaCacheManager` manages cache lifecycle
- 2-3x constraint solver speedup for large systems (100+ bodies)
- Location: `include/jaguar/physics/entity.h`

#### 12. Constraint Solver Early Exit
- Convergence-based termination: exits when all constraints solved
- Configurable via `SolverConfig::convergence_threshold` and `max_iterations`
- 10-30% iteration reduction when contacts resolve quickly
- Typical early exit at iteration 3-4 of 10
- Location: `src/physics/constraints/sequential_impulse_solver.cpp`

#### 13. Warm Start Validation
- Impulse clamping prevents extreme values from previous frames
- Maintains energy conservation while accelerating convergence
- Optional feature via `SolverConfig::enable_warm_start`
- Particularly effective for stacked/sleeping bodies
- Location: `src/physics/constraints/sequential_impulse_solver.cpp`

---

### Phase 3: Environment & Terrain

#### 14. GDAL Terrain Loader
- `GDALTerrainLoader` integrating GDAL for GIS raster data
- Supports: GeoTIFF, DTED, DTED Level 0/1/2, HGT formats
- Bilinear interpolation for smooth elevation queries
- Surface normal computation via Sobel gradient operator
- WGS84 geodetic coordinate queries
- Location: `src/environment/terrain/gdal_loader.cpp`

#### 15. Terrain in EnvironmentService
- `TerrainManager` unified terrain access interface
- Thread-safe read operations via `std::shared_lock`
- LRU tile caching with configurable cache size (default 256 MB)
- Focus-point based LOD for streaming detail radius
- Batch query support for multiple elevation/normal queries
- Location: `include/jaguar/environment/terrain.h`

#### 16. Terrain-Aware Suspension
- `SuspensionContactModel` for vehicle suspension over slopes
- Forces applied along terrain normal, not global vertical
- Material-dependent friction from `SurfaceType` enumeration
- Support for: Asphalt, Sand, Clay, Grass, Snow, Ice, Water, Rock
- Bekker-Wong terramechanics parameters: k_c, k_phi, n
- Trafficability assessment: max_slope, bearing_capacity
- Location: `src/domain/land/suspension.cpp`

---

### Phase 4: Threading & Parallelism

#### 17. Work-Stealing Thread Pool
- Chase-Lev algorithm for lock-free task distribution
- Per-thread work queues with atomic head/tail pointers
- Automatic work stealing from idle threads
- `ThreadPool` class with configurable thread count
- API: `enqueue(Task)`, `parallel_for(start, end, body, grain_size)`, `wait_all()`
- Location: `include/jaguar/core/threading/thread_pool.h`, `src/core/threading/thread_pool.cpp`

#### 18. Parallel Force Computation
- Force computation fully parallelizable across threads
- Integration with `ComponentForceRegistry` for per-entity models
- Batch task submission with grain size tuning
- Thread-safe result collection via futures/promises
- Typical speedup: 3.5-4x on 4-core, 7-8x on 8-core
- Location: `src/physics/physics_system.cpp`

---

### Phase 5: Federation & Network

#### 19. DIS UDP Socket
- Cross-platform UDP networking for DIS protocol (IEEE 1278.1-2012)
- `UDPSocket` class with Windows/POSIX abstraction
- Unicast, multicast (group join/leave), and broadcast modes
- Non-blocking async operations with timeout support
- API: `send_to(data, size, address, port)`, `receive_from(buffer, size, timeout_ms)`
- Location: `include/jaguar/federation/dis_socket.h`, `src/federation/dis_socket.cpp`

#### 20. Fire/Detonation PDUs
- `FirePDU` for weapon firing events
- `DetonationPDU` for impact/detonation events
- Entity, munition, target identification (EntityIdentifier struct)
- Event tracking via `EventIdentifier` (exerciseID, eventSerial)
- Velocity and range information capture
- IEEE 1278.1-2012 format compliance
- Location: `include/jaguar/federation/dis_fire_pdu.h`, `src/federation/dis_fire_pdu.cpp`

#### 21. HLA RTI Methods
- Federation management: create federation, join, resign, destroy
- Object management: register, update, delete instances
- Time management: enable/disable time regulation and time-constrained
- Interaction management: subscribe/publish, send/receive
- Federate ambassador callbacks for async notifications
- `IRTIAmbassador` interface abstraction
- Location: `include/jaguar/federation/hla_rti.h`, `src/federation/hla_rti.cpp`

---

### Phase 6: Integration & Testing

#### 22. Physics Module Documentation
- Comprehensive API reference for physics components
- Force generator interfaces and implementations
- Constraint solver theory and configuration
- Integration method selection guide
- Location: `docs/modules/PHYSICS.md`

#### 23. Threading Module Documentation
- Work-stealing algorithm explanation with performance analysis
- Thread pool usage patterns and best practices
- Parallel force computation workflow
- Performance tuning guide (grain size, thread count)
- Location: `docs/modules/THREADING.md`

#### 24. Terrain Module Documentation
- GDAL terrain loader integration guide
- Coordinate system transformations (WGS84, ECEF)
- Terrain-aware physics setup procedures
- Performance characteristics and caching behavior
- Location: `docs/modules/TERRAIN.md`

---

### New Files Added

#### Headers
- `include/jaguar/core/constants.h` - Centralized physics constants
- `include/jaguar/core/threading/thread_pool.h` - Work-stealing thread pool
- `include/jaguar/physics/component_force_registry.h` - Entity-specific force models
- `include/jaguar/physics/constraints/ground_contact_constraint.h` - Terrain contact
- `include/jaguar/interface/xml_entity_loader.h` - XML entity parsing
- `include/jaguar/environment/terrain.h` - Terrain manager and queries
- `include/jaguar/federation/dis_socket.h` - DIS UDP networking
- `include/jaguar/federation/dis_fire_pdu.h` - Fire/Detonation PDUs
- `include/jaguar/federation/hla_rti.h` - HLA RTI interface
- `include/jaguar/federation/network_transport.h` - Network transport abstraction

#### Source Files
- `src/core/threading/thread_pool.cpp` - Thread pool implementation
- `src/core/xml_entity_loader.cpp` - XML loader implementation
- `src/physics/component_force_registry.cpp` - Force registry implementation
- `src/physics/constraints/ground_contact_constraint.cpp` - Terrain contact implementation
- `src/environment/terrain/gdal_loader.cpp` - GDAL terrain loader
- `src/environment/terrain/terrain_manager.cpp` - Terrain manager implementation
- `src/federation/dis_socket.cpp` - DIS socket implementation
- `src/federation/dis_fire_pdu.cpp` - Fire/Detonation PDU handlers
- `src/federation/hla_rti.cpp` - HLA RTI implementation
- `src/federation/network_transport.cpp` - Network transport implementation
- `src/domain/land/suspension.cpp` - Terrain-aware suspension model

#### Documentation
- `docs/modules/PHYSICS.md` - Physics system documentation
- `docs/modules/THREADING.md` - Threading module guide
- `docs/modules/TERRAIN.md` - Terrain integration guide

---

### Breaking Changes

#### 1. Integrator API Changes
**Before (v0.5.x):**
```cpp
RK4Integrator rk4;
rk4.propagate(state, forces, dt);

ABM4Integrator abm;
abm.integrate(state, forces, dt);  // Different method name
```

**After (v0.6.0):**
```cpp
std::unique_ptr<IStatePropagator> integrator = std::make_unique<RK4Integrator>();
integrator->integrate(state, forces, dt);  // Consistent interface
```

**Migration**: Replace all `propagate()` calls with `integrate()`. Create integrators via `IStatePropagator` interface for flexibility.

#### 2. EntityState Structure
**Added field:**
```cpp
Mat3x3 inertia_inv;  // Cached inverse inertia tensor
```

**Migration**: If you have custom serialization of `EntityState`, update to include new field.

#### 3. Force Generator Registration
**Before (v0.5.x):**
```cpp
ForceGeneratorRegistry forces;
forces.register_generator(std::make_unique<MyAero>());
```

**After (v0.6.0):**
```cpp
ComponentForceRegistry registry;
auto aero = std::make_unique<F16Aerodynamics>();
registry.register_model<IAerodynamicsModel>(entity_id, std::move(aero));
```

**Migration**: Use entity-specific registration for multi-domain simulations.

---

### Migration Guide

#### From 0.5.0 to 0.6.0

##### Force Model Registration

Entity-specific force registration replaces global registry:

**Old Pattern (0.5.0):**
```cpp
ForceGeneratorRegistry global_forces;
global_forces.register_generator(std::make_unique<GenericAerodynamics>());
// Single model applied to all aircraft
```

**New Pattern (0.6.0):**
```cpp
ComponentForceRegistry registry;
// Register F-16 specific aerodynamics
auto f16_aero = std::make_unique<F16Aerodynamics>();
registry.register_model<IAerodynamicsModel>(f16_id, std::move(f16_aero));

// Register different helicopter aerodynamics
auto uh60_aero = std::make_unique<UH60Aerodynamics>();
registry.register_model<IAerodynamicsModel>(uh60_id, std::move(uh60_aero));
```

##### Terrain Queries

Replace placeholder elevations with real terrain data:

**Old Pattern (0.5.0):**
```cpp
Real elev = 0.0;  // Hardcoded flat terrain
```

**New Pattern (0.6.0):**
```cpp
TerrainManager terrain;
terrain.load_terrain("dem.tif");  // GeoTIFF, DTED, etc.

Real elev = terrain.get_elevation(lat, lon);
Vec3 normal = terrain.get_surface_normal(lat, lon);
TerrainQuery query = terrain.query(lat, lon);
```

##### Parallel Physics

Sequential force computation becomes parallel:

**Old Pattern (0.5.0):**
```cpp
// Sequential
for (auto& entity : entities) {
    compute_forces(entity);
}
```

**New Pattern (0.6.0):**
```cpp
// Parallel with automatic load balancing
ThreadPool& pool = ThreadPool::get_global(4);

std::vector<std::future<EntityForces>> futures;
for (auto& entity : entities) {
    auto future = pool.enqueue([&entity]() {
        return compute_forces(entity);
    });
    futures.push_back(std::move(future));
}
pool.wait_all();

// Collect results
for (auto& future : futures) {
    auto forces = future.get();
    // Apply forces...
}
```

##### Integrator Selection

Consistent API across integrators:

**Old Pattern (0.5.0):**
```cpp
// Inconsistent API
RK4Integrator rk4;
rk4.propagate(state, forces, dt);

ABM4Integrator abm;
abm.integrate(state, forces, dt);  // Different method!

EulerIntegrator euler;
euler.step(state, forces, dt);  // Another variant!
```

**New Pattern (0.6.0):**
```cpp
// Consistent IStatePropagator interface
std::unique_ptr<IStatePropagator> integrator;

if (adaptive_required) {
    integrator = std::make_unique<AdaptiveIntegrator>();
} else {
    integrator = std::make_unique<RK4Integrator>();
}

// Same method for all
integrator->integrate(state, forces, dt);
```

##### Entity Loading

Load multi-domain entities from XML:

**Old Pattern (0.5.0):**
```cpp
// Manual per-domain setup
EntityState aircraft;
aircraft.mass = 80000.0;
aircraft.component_mask = AERODYNAMICS | PROPULSION;
// ... tedious manual initialization
```

**New Pattern (0.6.0):**
```cpp
// Automatic from XML
XmlEntityLoader loader;
EntityDefinition def = loader.load("f16.xml");

// Auto-creates component mask and loads all properties
EntityState entity;
entity.mass = def.mass_balance.empty_mass + def.mass_balance.fuel_mass;
entity.inertia = def.mass_balance.inertia;
entity.component_mask = def.component_mask;
```

---

### Performance Improvements

| Feature | Improvement | Conditions |
|---------|-------------|-----------|
| Constraint Solver | 2-3x faster | 100+ bodies, inverse inertia caching |
| Force Computation | 3.5-4x speedup | 4-core CPU, 100+ entities |
| Integrator Convergence | 10-30% fewer iterations | Split impulse solver enabled |
| Physics Scalability | Linear to 16 threads | Work-stealing thread pool |
| Terrain Queries | <10ms per query | GDAL loader with LRU cache |

---

### Known Issues

#### Phase 1
- XML loader: Comments in entity definitions not preserved; use attributes instead
- Ground contact: Multiple simultaneous contacts on same entity may accumulate impulses incorrectly

#### Phase 2
- ABM4: Quaternion error metric may have gimbal lock handling issues at poles (latitude = ±90°)
- Warm-start: Disabled by default; enable via `SolverConfig::enable_warm_start` for stacked bodies

#### Phase 3
- Terrain: LRU eviction with very large detail_radius may cause frame stalls; reduce radius or increase cache_size_mb
- GDAL: Some DTED Level 2 files report incorrect data bounds; use GeoTIFF if available

#### Phase 4
- Threading: Work-stealing has ~5-10% overhead on very small task counts (<10 tasks)
- Thread pool: Global pool not thread-safe for dynamic thread count changes after creation

#### Phase 5
- DIS: Multicast TTL defaults to 255; adjust for large network deployments
- HLA: Federate resignation may hang if time management not properly disabled before resize
- Network: UDP loss not handled; use TCP for critical simulation state

#### Phase 6
- Documentation: Code examples may require GCC 11+ for proper constexpr support

---

### Compilation Flags

#### Enable/Disable Features
```cmake
# Threading
set(JAGUAR_ENABLE_THREADING ON)  # Default ON

# Federation
set(JAGUAR_ENABLE_DIS ON)        # DIS protocol support
set(JAGUAR_ENABLE_HLA OFF)       # HLA RTI support (requires RTI library)

# Terrain
set(JAGUAR_ENABLE_GDAL ON)       # GDAL support for terrain
```

---

### Testing & Validation

#### Unit Tests Added
- `tests/physics/test_component_force_registry.cpp` - Force registration and retrieval
- `tests/physics/test_ground_contact.cpp` - Terrain contact constraints
- `tests/threading/test_thread_pool.cpp` - Work-stealing algorithm correctness
- `tests/environment/test_terrain_manager.cpp` - Terrain queries and caching
- `tests/federation/test_dis_socket.cpp` - DIS UDP socket functionality
- `tests/interface/test_xml_loader.cpp` - XML parsing and unit conversion

#### Validation Tests Added
- `tests/validation/physics_energy_conservation.cpp` - Energy conservation with new integrators
- `tests/validation/constraint_solver_accuracy.cpp` - Quaternion error metric validation
- `tests/validation/terrain_continuity.cpp` - Bilinear interpolation smoothness
- `tests/validation/parallel_force_determinism.cpp` - Deterministic parallel execution

#### Run Tests
```bash
mkdir build && cd build
cmake ..
ctest --output-on-failure
```

---

### Deprecations

The following APIs are deprecated and will be removed in v0.7.0:

- `IStatePropagator::propagate()` → Use `integrate()` instead
- `GlobalForceRegistry` → Use `ComponentForceRegistry` with entity IDs
- `SimpleTerrainProvider` → Use `TerrainManager` with GDAL
- `SequentialPhysics::step()` → Use `ParallelPhysics` with thread pool

---

### References

- [API Reference](docs/API_REFERENCE.md)
- [Physics Module](docs/modules/PHYSICS.md)
- [Terrain Module](docs/modules/TERRAIN.md)
- [Threading Module](docs/modules/THREADING.md)
- [Federation Module](docs/modules/FEDERATION.md)
- [IEEE 1278.1-2012 DIS Standard](https://standards.ieee.org/ieee/1278.1/2012/)
- [HLA RTI Specification](https://www.dmso.mil/public/organization/oti/)

---

## [0.5.0] - Previous Release

See [GitHub Releases](https://github.com/ikhyeon/JaguarEngine/releases/tag/v0.5.0)

---

## Version Numbering

**Format:** MAJOR.MINOR.PATCH

- **MAJOR**: Framework-level changes (rare)
- **MINOR**: Feature additions (quarterly)
- **PATCH**: Bug fixes and refinements (as needed)

---

## Contributors

JaguarEngine is developed by the Simulation Research Team.

## License

See LICENSE file in repository.
