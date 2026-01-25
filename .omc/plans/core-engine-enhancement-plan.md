# JaguarEngine Core Engine Enhancement Plan

**Version**: 1.0.0
**Date**: 2026-01-25
**Scope**: Large Initiative (1+ month)
**Status**: Planning

---

## Executive Summary

This plan outlines a comprehensive enhancement of the JaguarEngine core engine across all subsystems. Based on detailed code analysis, this initiative addresses:

- **Critical incomplete implementations** (threading, terrain loading, ML inference)
- **Physics system quality gaps** (ABM4 angular history, constraint solver improvements)
- **Core engine logic issues** (entity-specific forces, terrain queries, surface constraints)
- **Performance optimizations** (caching, convergence checks, SIMD opportunities)
- **API consistency** (standardized integrator behavior, configuration usage)

---

## Phase Summary

| Phase | Name | Duration | Priority | Risk |
|-------|------|----------|----------|------|
| 1 | Core Engine Critical Fixes | 2 weeks | P0 | Low |
| 2 | Physics System Enhancement | 3 weeks | P0 | Medium |
| 3 | Environment & Terrain | 2 weeks | P1 | Medium |
| 4 | Threading & Parallelism | 2 weeks | P1 | High |
| 5 | Federation & Network | 2 weeks | P2 | Medium |
| 6 | Integration & Testing | 1 week | P0 | Low |

**Total Estimated Duration**: 12 weeks

---

## Phase 1: Core Engine Critical Fixes (2 weeks)

### 1.1 Entity-Specific Force Computation

**File**: `src/core/engine_exec.cpp` (lines 343-386)

**Current State**: Completely unimplemented - all component checks exist but produce no forces.

**Implementation Tasks**:

1. **Create Component Force Registry** (`include/jaguar/physics/component_force_registry.h`)
   - Store per-entity force model instances keyed by ComponentBits
   - Factory pattern for creating force models from XML configuration
   - Thread-safe access for parallel force computation

2. **Implement force lookup and application**:
   ```cpp
   void compute_entity_specific_forces(...) {
       if (entity.components & Aerodynamics) {
           auto* aero_model = component_registry_.get<AerodynamicsModel>(entity.id);
           if (aero_model) {
               aero_model->compute_forces(state, env, dt, forces);
           }
       }
       // Similar for Propulsion, Terramechanics, Hydrodynamics, Buoyancy
   }
   ```

3. **Wire up existing domain force generators**:
   - Connect `AerodynamicsModel` from `src/domain/air.cpp`
   - Connect `TerramechanicsModel` from `src/domain/land/`
   - Connect `HydrodynamicsModel` from `src/environment/ocean/`

**Acceptance Criteria**:
- [ ] Entities with Aerodynamics component receive proper aero forces
- [ ] Unit tests verify force accumulation for each component type
- [ ] XML entity loading creates appropriate force model instances

---

### 1.2 Terrain Height Query Implementation

**File**: `src/core/engine_exec.cpp` (lines 277-282)

**Current State**: Hardcoded `target_altitude = 100.0` ignores actual terrain.

**Implementation Tasks**:

1. **Use environment service terrain query**:
   ```cpp
   if (entity.primary_domain == Domain::Land) {
       // Query terrain elevation from environment
       target_altitude = env.terrain_elevation;
       if (!std::isfinite(target_altitude)) {
           target_altitude = 0.0; // Fallback to sea level
       }
   }
   ```

2. **Extend Environment struct if needed** (`include/jaguar/environment/environment.h`):
   - Verify `terrain_elevation` field exists and is populated
   - Add `terrain_normal` for proper surface constraint direction

3. **Add configuration check**:
   ```cpp
   if (!property_manager_.get_bool("config/terrain_enabled")) {
       return; // Skip terrain constraints if disabled
   }
   ```

**Acceptance Criteria**:
- [ ] Land entities constrained to actual terrain elevation
- [ ] Tests verify varying terrain heights affect constraint
- [ ] config/terrain_enabled flag controls behavior

---

### 1.3 Surface Constraints Enhancement

**File**: `src/core/engine_exec.cpp` (lines 256-308)

**Current State**: Sea domain handling incomplete, no terrain normal consideration.

**Implementation Tasks**:

1. **Implement Sea domain constraints**:
   - Constrain to water surface (altitude = 0) for surface vessels
   - Allow submarines to go below surface with depth constraints
   - Use `env.over_water` flag to determine if over ocean

2. **Use terrain normal for Land domain**:
   ```cpp
   // Get surface normal for slope handling
   Vec3 surface_normal = env.terrain_normal;
   if (surface_normal.length_squared() < 0.01) {
       surface_normal = Vec3{0, 0, 1}; // Default up
   }
   // Project velocity onto surface plane instead of zeroing vertical
   Real penetration_velocity = state.velocity.dot(surface_normal);
   if (penetration_velocity > 0) {
       state.velocity -= penetration_velocity * surface_normal;
   }
   ```

3. **Add contact force generation** (not just constraint):
   - Calculate normal force based on penetration depth
   - Apply ground reaction force to EntityForces

**Acceptance Criteria**:
- [ ] Vehicles on slopes respond to terrain angle
- [ ] Ships constrained to water surface
- [ ] Ground reaction forces appear in force diagnostics

---

### 1.4 Hardcoded Value Cleanup

**Files**: `src/core/engine_exec.cpp` (lines 84, 94, 188)

**Implementation Tasks**:

1. **Create configuration constants** (`include/jaguar/core/constants.h`):
   ```cpp
   namespace jaguar::constants {
       constexpr Real DEFAULT_TIME_STEP = 0.01;  // 100 Hz
       constexpr Real DEFAULT_GROUND_LEVEL = 0.0;
       constexpr Real TIME_STEP_MIN = 1e-6;
       constexpr Real TIME_STEP_MAX = 0.1;
   }
   ```

2. **Replace hardcoded values**:
   - Line 84: Use `time_manager_.get_fixed_dt()` in run()
   - Line 94: Use `constants::DEFAULT_TIME_STEP` as fallback
   - Line 188: Use `constants::DEFAULT_TIME_STEP` in initialize()

**Acceptance Criteria**:
- [ ] No magic numbers in engine_exec.cpp
- [ ] All constants documented in constants.h
- [ ] Configuration overrides work correctly

---

### 1.5 XML Entity Loading Implementation

**File**: `src/core/engine_exec.cpp` (lines 486-489)

**Current State**: Returns `INVALID_ENTITY_ID` with TODO comment.

**Implementation Tasks**:

1. **Create XML entity parser** (`src/interface/xml_entity_loader.cpp`):
   - Parse entity definition XML (already documented in ARCHITECTURE.md)
   - Extract metrics, mass_balance, aerodynamics, propulsion sections
   - Create appropriate force model instances

2. **Integrate with Engine::create_entity**:
   ```cpp
   EntityId Engine::create_entity(const std::string& xml_path) {
       XmlEntityLoader loader;
       EntityDefinition def = loader.load(xml_path);
       EntityId id = impl_->get_entity_manager().create_entity(def.name, def.domain);
       // Attach force models based on definition
       register_force_models(id, def);
       return id;
   }
   ```

**Acceptance Criteria**:
- [ ] Sample F-16 XML loads successfully
- [ ] Aerodynamic coefficients parsed into lookup tables
- [ ] Mass/inertia properties set correctly

---

## Phase 2: Physics System Enhancement (3 weeks)

### 2.1 ABM4 Angular Acceleration History

**File**: `src/physics/integrators/abm4.cpp`

**Current State**: Angular acceleration history initialized but never populated or used.

**Implementation Tasks**:

1. **Track angular acceleration in ABM4**:
   ```cpp
   void integrate(EntityState& state, const EntityForces& forces, Real dt) {
       // Compute current angular acceleration
       Vec3 angular_accel = compute_angular_accel(state, forces);

       // Store in history (for multi-step method)
       angular_accel_history_[step_count_ % 4] = angular_accel;

       // Use ABM4 formula for angular velocity integration
       if (step_count_ >= 3) {
           // 4th-order Adams-Bashforth predictor for angular velocity
           state.angular_velocity += dt * (
               55.0 * angular_accel_history_[(step_count_) % 4]
             - 59.0 * angular_accel_history_[(step_count_ - 1) % 4]
             + 37.0 * angular_accel_history_[(step_count_ - 2) % 4]
             -  9.0 * angular_accel_history_[(step_count_ - 3) % 4]
           ) / 24.0;
       }
       // Use RK4 quaternion integration for orientation
       state.orientation = quaternion_utils::integrate_rk4(
           state.orientation, state.angular_velocity, dt);
   }
   ```

**Acceptance Criteria**:
- [ ] ABM4 rotational accuracy matches RK4 for test cases
- [ ] Energy conservation tests pass
- [ ] Gyroscopic precession computed correctly

---

### 2.2 Sequential Impulse Split Position Correction

**File**: `src/physics/constraints/sequential_impulse_solver.cpp` (line 232)

**Current State**: `solve_position_constraints()` is stubbed with `(void)data` cast.

**Implementation Tasks**:

1. **Implement split impulse position correction**:
   ```cpp
   void solve_position_constraints(ConstraintSolverData& data) {
       if (!config_.split_impulse) {
           return; // Use Baumgarte only
       }

       for (int iter = 0; iter < config_.position_iterations; ++iter) {
           for (auto& constraint : data.constraints) {
               Real error = constraint.compute_position_error();
               if (std::abs(error) < config_.slop) continue;

               Real correction = -config_.position_correction * error;
               Vec3 impulse = correction * constraint.jacobian.normal;

               // Apply position change directly (no velocity)
               data.storage.positions(constraint.body_a) +=
                   impulse * data.storage.inverse_mass(constraint.body_a);
               // ... similar for body_b with rotation
           }
       }
   }
   ```

**Acceptance Criteria**:
- [ ] Stacking stability test (10 boxes) passes
- [ ] No position drift after 1000 steps
- [ ] Split impulse configurable via ConstraintSolverConfig

---

### 2.3 Quaternion Error Metric in Adaptive Integrators

**File**: `src/physics/integrators/adaptive.cpp` (lines 56-64)

**Current State**: Uses Euclidean component difference (incorrect for quaternions).

**Implementation Tasks**:

1. **Implement proper quaternion distance**:
   ```cpp
   Real quaternion_error(const Quat& q1, const Quat& q2) {
       // Angular distance: arccos(|q1 · q2|)
       Real dot = std::abs(q1.w*q2.w + q1.x*q2.x + q1.y*q2.y + q1.z*q2.z);
       dot = std::min(dot, 1.0); // Clamp for numerical stability
       return 2.0 * std::acos(dot); // Returns angle in radians
   }
   ```

2. **Use in estimate_error()**:
   ```cpp
   Real estimate_error(...) {
       Real pos_error = (state_full.position - state_half.position).norm();
       Real vel_error = (state_full.velocity - state_half.velocity).norm();
       Real quat_error = quaternion_error(state_full.orientation, state_half.orientation);
       Real omega_error = (state_full.angular_velocity - state_half.angular_velocity).norm();

       return std::max({pos_error, vel_error, quat_error, omega_error});
   }
   ```

**Acceptance Criteria**:
- [ ] Tumbling rigid body test shows stable adaptive stepping
- [ ] Error estimates scale correctly with rotation rate
- [ ] No spurious step rejections for rotating objects

---

### 2.4 Integrator API Consistency

**Files**: All integrator source files

**Current State**: Inconsistent zero-mass handling and time-step bounds.

**Implementation Tasks**:

1. **Standardize zero-mass early return**:
   ```cpp
   // Add to all integrators at start of integrate()
   if (state.mass < constants::MASS_EPSILON) {
       return; // Skip integration for massless entities
   }
   ```

2. **Unify time-step bounds**:
   ```cpp
   namespace constants {
       constexpr Real ADAPTIVE_MIN_DT = 1e-9;
       constexpr Real ADAPTIVE_MAX_DT = 0.1;
   }
   ```

3. **Add boundary checks in integrate()**:
   ```cpp
   dt = std::clamp(dt, constants::ADAPTIVE_MIN_DT, constants::ADAPTIVE_MAX_DT);
   ```

**Acceptance Criteria**:
- [ ] All integrators handle zero-mass identically
- [ ] Switching integrators mid-simulation doesn't change behavior
- [ ] Time-step bounds documented in API

---

### 2.5 Inverse Inertia Caching

**Files**: `include/jaguar/physics/entity.h`, all integrators

**Current State**: `compute_inverse_inertia()` called every step.

**Implementation Tasks**:

1. **Add cached inverse inertia to EntityState**:
   ```cpp
   struct EntityState {
       // Existing fields...
       Mat3x3 inverse_inertia_cached;
       bool inverse_inertia_dirty{true};

       Mat3x3 get_inverse_inertia() {
           if (inverse_inertia_dirty) {
               inverse_inertia_cached = compute_inverse_inertia(inertia);
               inverse_inertia_dirty = false;
           }
           return inverse_inertia_cached;
       }
   };
   ```

2. **Mark dirty when inertia changes**:
   ```cpp
   void set_inertia(const Mat3x3& I) {
       inertia = I;
       inverse_inertia_dirty = true;
   }
   ```

**Acceptance Criteria**:
- [ ] Benchmark shows 5-10% improvement for 10K entities
- [ ] Inertia changes correctly invalidate cache
- [ ] No behavior change from caching

---

### 2.6 Constraint Solver Convergence Early-Exit

**File**: `src/physics/constraints/sequential_impulse_solver.cpp` (lines 205-214)

**Current State**: Always runs full iterations regardless of convergence.

**Implementation Tasks**:

1. **Track max impulse per iteration**:
   ```cpp
   for (int iter = 0; iter < config_.velocity_iterations; ++iter) {
       Real max_impulse = 0.0;

       for (auto& row : data.rows) {
           Real impulse = solve_row(row, data);
           max_impulse = std::max(max_impulse, std::abs(impulse));
       }

       // Early exit if converged
       if (max_impulse < config_.convergence_threshold) {
           break;
       }
   }
   ```

2. **Add convergence config**:
   ```cpp
   struct ConstraintSolverConfig {
       // Existing...
       Real convergence_threshold{1e-8};
   };
   ```

**Acceptance Criteria**:
- [ ] Stable scenes use 30-50% fewer iterations
- [ ] Active scenes still use full iterations
- [ ] Convergence stats available via get_stats()

---

### 2.7 Warm Start Validation

**File**: `src/physics/constraints/sequential_impulse_solver.cpp` (lines 117-119)

**Current State**: Warm start applied unconditionally, can amplify errors.

**Implementation Tasks**:

1. **Validate warm start impulses**:
   ```cpp
   void apply_warm_start(ConstraintSolverData& data) {
       for (auto& row : data.rows) {
           Real cached_impulse = row.accumulated_impulse * config_.warm_start_factor;

           // Validate magnitude
           Real max_reasonable = compute_max_impulse(row, data);
           if (std::abs(cached_impulse) > max_reasonable) {
               cached_impulse = std::copysign(max_reasonable, cached_impulse);
           }

           apply_impulse(row, cached_impulse, data);
       }
   }
   ```

**Acceptance Criteria**:
- [ ] Previously unstable stacking scenario now stable
- [ ] Warm start still provides speedup for normal cases
- [ ] No NaN propagation from bad warm starts

---

## Phase 3: Environment & Terrain (2 weeks)

### 3.1 GDAL Terrain Loader Implementation

**File**: `src/environment/terrain/gdal_loader.cpp`

**Current State**: Empty stub (237 bytes) with TODO comment.

**Implementation Tasks**:

1. **Implement GDAL data loading**:
   ```cpp
   class GDALTerrainLoader : public ITerrainLoader {
   public:
       bool load(const std::string& path) override {
           GDALDataset* dataset = (GDALDataset*)GDALOpen(path.c_str(), GA_ReadOnly);
           if (!dataset) return false;

           // Get raster band
           GDALRasterBand* band = dataset->GetRasterBand(1);

           // Get georeferencing
           double geo_transform[6];
           dataset->GetGeoTransform(geo_transform);

           // Load elevation data into grid
           // ...
       }

       Real query_elevation(Real lat, Real lon) const override;
       Vec3 query_normal(Real lat, Real lon) const override;
   };
   ```

2. **Implement tile caching**:
   - Quadtree-based tile management
   - LRU cache with configurable memory budget
   - Async tile loading via I/O thread pool

3. **Support multiple formats**:
   - DTED Level 0/1/2
   - GeoTIFF
   - Shapefile (for features)

**Acceptance Criteria**:
- [ ] DTED Level 1 file loads correctly
- [ ] Elevation queries return correct values
- [ ] Memory usage stays within configured budget

---

### 3.2 Terrain Query Integration in Environment Service

**File**: `src/environment/environment.cpp`

**Implementation Tasks**:

1. **Wire terrain loader to environment queries**:
   ```cpp
   Environment EnvironmentService::query(const Vec3& position, Real time) {
       Environment env;

       // Convert ECEF to geodetic
       GeodeticPosition lla = coord::ecef_to_lla(position);

       // Query terrain
       if (terrain_manager_) {
           env.terrain_elevation = terrain_manager_->get_elevation(lla.latitude, lla.longitude);
           env.terrain_normal = terrain_manager_->get_surface_normal(lla.latitude, lla.longitude);
           env.over_water = env.terrain_elevation < 0.0; // Simplified
       }

       // Query atmosphere, ocean, etc.
       // ...

       return env;
   }
   ```

**Acceptance Criteria**:
- [ ] Environment queries return real terrain elevation
- [ ] Surface normals computed from terrain gradient
- [ ] Performance: 50K queries/second target

---

### 3.3 Suspension Model Terrain Integration

**File**: `src/domain/land/suspension.cpp` (line 108)

**Current State**: Placeholder TODO for terrain query.

**Implementation Tasks**:

1. **Query terrain at wheel positions**:
   ```cpp
   void SuspensionModel::update(EntityState& state, const Environment& env, Real dt) {
       for (auto& wheel : wheels_) {
           // Get wheel position in world frame
           Vec3 wheel_world = transform_to_world(wheel.local_position, state);

           // Query terrain at wheel
           Real terrain_height = env.terrain_elevation; // Per-wheel query needed
           Real wheel_height = wheel_world.z;  // Simplified - needs geodetic

           // Compute suspension compression
           Real compression = terrain_height - wheel_height + wheel.radius;
           if (compression > 0) {
               wheel.force = compute_spring_damper_force(compression, wheel.velocity);
           }
       }
   }
   ```

**Acceptance Criteria**:
- [ ] Wheels respond to terrain undulation
- [ ] Suspension forces visible in diagnostics
- [ ] Vehicle stable on slopes

---

## Phase 4: Threading & Parallelism (2 weeks)

### 4.1 Work-Stealing Thread Pool Implementation

**File**: `src/core/threading/thread_pool.cpp`

**Current State**: Empty stub (191 bytes).

**Implementation Tasks**:

1. **Implement lock-free work queue**:
   ```cpp
   class WorkStealingQueue {
   public:
       void push(Task&& task);      // Owner push
       std::optional<Task> pop();    // Owner pop (LIFO)
       std::optional<Task> steal();  // Thief steal (FIFO)

   private:
       std::deque<Task> tasks_;
       mutable std::mutex mutex_;
       // Or use lock-free deque for better performance
   };
   ```

2. **Implement thread pool**:
   ```cpp
   class ThreadPool {
   public:
       ThreadPool(size_t num_threads = std::thread::hardware_concurrency());

       template<typename F>
       std::future<std::invoke_result_t<F>> submit(F&& func);

       void parallel_for(size_t begin, size_t end,
                        std::function<void(size_t)> body);

       void shutdown();

   private:
       std::vector<std::thread> workers_;
       std::vector<WorkStealingQueue> queues_;
       std::atomic<bool> stop_{false};
   };
   ```

3. **Integrate with PhysicsSystem**:
   ```cpp
   void PhysicsSystem::update(EntityManager& em, Real dt) {
       if (parallel_enabled_) {
           thread_pool_.parallel_for(0, em.count(), [&](size_t i) {
               update_entity(em.get_entity(i), em.get_state_storage(), dt);
           });
       } else {
           // Sequential fallback
       }
   }
   ```

**Acceptance Criteria**:
- [ ] Thread pool creates N threads (configurable)
- [ ] Work stealing balances load across threads
- [ ] 4x speedup on 8-core system for 10K entities

---

### 4.2 Parallel Force Computation

**File**: `src/core/engine_exec.cpp`

**Implementation Tasks**:

1. **Parallelize compute_all_forces()**:
   ```cpp
   void compute_all_forces(Real dt) {
       auto& storage = entity_manager_.get_state_storage();
       auto enabled_generators = force_registry_.get_enabled_generators();

       // Parallel entity processing
       thread_pool_.parallel_for(0, entity_manager_.count(), [&](size_t i) {
           physics::Entity& entity = entity_manager_.get_entity(i);
           if (!entity.active) return;

           physics::EntityState state = storage.get_state(entity.state_index);
           environment::Environment env = environment_.query(state.position, time_manager_.get_time());

           // Thread-local force accumulation
           physics::EntityForces forces;
           for (auto* generator : enabled_generators) {
               if (generator->domain() == Domain::Generic ||
                   generator->domain() == entity.primary_domain) {
                   generator->compute_forces(state, env, dt, forces);
               }
           }

           // Write back (storage must be thread-safe for writes)
           storage.set_forces(entity.state_index, forces);
       });
   }
   ```

**Acceptance Criteria**:
- [ ] Force computation scales linearly with cores
- [ ] No race conditions in parallel writes
- [ ] Deterministic results (same as sequential)

---

## Phase 5: Federation & Network (2 weeks)

### 5.1 DIS Socket Implementation

**File**: `src/federation/dis_protocol.cpp` (line 794)

**Current State**: Uses in-memory queue instead of actual UDP sockets.

**Implementation Tasks**:

1. **Implement UDP socket layer**:
   ```cpp
   class DISSocket {
   public:
       bool bind(uint16_t port);
       bool join_multicast(const std::string& group);
       ssize_t send(const uint8_t* data, size_t len, const sockaddr_in& dest);
       ssize_t receive(uint8_t* buffer, size_t max_len, sockaddr_in* src);

   private:
       int socket_fd_{-1};
       uint16_t port_{0};
   };
   ```

2. **Wire into DISProtocol**:
   ```cpp
   void DISProtocol::initialize(uint16_t port) {
       socket_ = std::make_unique<DISSocket>();
       socket_->bind(port);
       socket_->join_multicast("239.1.2.3"); // Configurable
   }
   ```

**Acceptance Criteria**:
- [ ] Can send/receive Entity State PDUs over network
- [ ] Multicast group configurable via XML
- [ ] Throughput: 1000 PDUs/second

---

### 5.2 Fire/Detonation PDU Implementation

**File**: `src/interface/network/dis_adapter.cpp` (lines 902, 910)

**Implementation Tasks**:

1. **Implement Fire PDU deserialization**:
   ```cpp
   void deserialize_fire_pdu(const uint8_t* data, FireEvent& event) {
       // Parse header
       event.exercise_id = read_u8(data, 4);
       event.firing_entity_id = read_entity_id(data, 12);
       event.target_entity_id = read_entity_id(data, 18);
       event.munition_id = read_entity_id(data, 24);
       event.fire_location = read_world_coordinates(data, 30);
       // ...
   }
   ```

2. **Implement Detonation PDU**:
   - Similar structure with detonation location and result

**Acceptance Criteria**:
- [ ] Fire PDUs trigger weapon fire events in simulation
- [ ] Detonation PDUs trigger damage calculations
- [ ] Round-trip PDU test passes

---

### 5.3 HLA RTI Stub Replacement

**File**: `src/federation/hla_rti.cpp`

**Current State**: All 22+ RTI methods are no-op stubs.

**Implementation Tasks**:

1. **Add Portico RTI library dependency**:
   - CMake find module for Portico
   - Conditional compilation based on availability

2. **Implement core RTI operations**:
   - `createFederationExecution()`
   - `joinFederationExecution()`
   - `registerObjectInstance()`
   - `updateAttributeValues()`
   - `reflectAttributeValues()` callback

**Note**: This is a large task that may extend beyond 2 weeks depending on RTI library integration complexity.

**Acceptance Criteria**:
- [ ] Can join HLA federation with Portico RTI
- [ ] Entity State attributes published/reflected
- [ ] Time management (lookahead) works

---

## Phase 6: Integration & Testing (1 week)

### 6.1 End-to-End Integration Tests

**Implementation Tasks**:

1. **Create integration test scenarios**:
   - Multi-domain scenario (air + land + sea entities)
   - Long-running stability test (10,000 steps)
   - Network federation test (if HLA complete)

2. **Performance benchmarks**:
   - 1K, 10K, 100K entity scaling tests
   - Thread scaling tests (1, 2, 4, 8 cores)
   - Memory usage validation

---

### 6.2 Documentation Updates

**Implementation Tasks**:

1. **Update ARCHITECTURE.md** with:
   - New component force registry
   - Threading model
   - Enhanced constraint solver

2. **Update API documentation**:
   - New configuration options
   - Threading API
   - Error handling improvements

---

### 6.3 Validation Test Completion

**Files**: `tests/validation/test_*.cpp`

**Implementation Tasks**:

1. **Complete aerodynamics validation** (line 140, 163)
2. **Complete SGP4 gravity validation** (line 285)
3. **Complete terrain model validation** (line 162)

---

## Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Threading race conditions | Medium | High | Extensive testing, TSAN |
| GDAL library compatibility | Low | Medium | Version pinning, fallback |
| HLA RTI complexity | High | Medium | Make optional, stub fallback |
| Performance regression | Low | High | Continuous benchmarking |
| API breaking changes | Low | Medium | Deprecation warnings, migration guide |

---

## Success Metrics

| Metric | Current | Target | Phase |
|--------|---------|--------|-------|
| Entity-specific forces working | 0% | 100% | 1 |
| Terrain queries accurate | 0% | 100% | 3 |
| Thread pool implemented | 0% | 100% | 4 |
| Physics step (10K entities) | 80ms | 20ms | 2, 4 |
| DIS network functional | 70% | 100% | 5 |
| Test coverage | ~80% | 90% | 6 |

---

## Appendix: File Change Summary

### New Files
- `include/jaguar/physics/component_force_registry.h`
- `include/jaguar/core/constants.h`
- `src/physics/component_force_registry.cpp`
- `src/interface/xml_entity_loader.cpp`
- `src/environment/terrain/gdal_loader.cpp` (rewrite)
- `src/core/threading/thread_pool.cpp` (rewrite)

### Modified Files
- `src/core/engine_exec.cpp` (major changes)
- `src/physics/integrators/abm4.cpp`
- `src/physics/integrators/adaptive.cpp`
- `src/physics/constraints/sequential_impulse_solver.cpp`
- `include/jaguar/physics/entity.h`
- `src/domain/land/suspension.cpp`
- `src/federation/dis_protocol.cpp`
- `src/federation/hla_rti.cpp`

---

*Plan Version: 1.0.0*
*Created: 2026-01-25*
*Author: Claude (Planning Session)*
