# JaguarEngine Quick Start Implementation Guide

**Purpose**: Actionable sprint-by-sprint guide to begin implementation immediately.

---

## Sprint 0: Development Environment Setup (1 day)

### Prerequisites Checklist

```bash
# Required Tools
□ CMake 3.25+
□ C++20 compiler (GCC 12+, Clang 15+, or MSVC 2022)
□ Git
□ Python 3.10+ (for scripts)

# Optional but Recommended
□ clang-tidy
□ clang-format
□ Valgrind (Linux)
□ ccache
```

### Setup Commands

```bash
# Clone and setup
cd /Users/ikhyeon.kim/Workspace/JaguarEngine

# Create build directory
mkdir -p build && cd build

# Configure with CMake
cmake .. -DCMAKE_BUILD_TYPE=Debug \
         -DJAGUAR_BUILD_TESTS=ON \
         -DJAGUAR_BUILD_BENCHMARKS=ON

# Build (this will currently fail until implementation begins)
cmake --build . -j$(nproc)
```

### First Build Target
Get the project to compile with stub implementations:

```bash
# Expected first successful build
cmake --build . --target jaguar_core
```

---

## Sprint 1: Core Math Library (1 week)

### Goal
Complete `include/jaguar/core/types.h` and math implementations.

### Tasks

#### Day 1-2: Vec3 Implementation
```cpp
// File: src/core/math/vector.cpp
// Implement these operations:

Vec3 operator+(const Vec3& a, const Vec3& b);
Vec3 operator-(const Vec3& a, const Vec3& b);
Vec3 operator*(const Vec3& v, Real s);
Vec3 operator*(Real s, const Vec3& v);
Real dot(const Vec3& a, const Vec3& b);
Vec3 cross(const Vec3& a, const Vec3& b);
Real length(const Vec3& v);
Vec3 normalize(const Vec3& v);
```

#### Day 3-4: Quaternion Implementation
```cpp
// File: src/core/math/quaternion.cpp
// Implement these operations:

Quat operator*(const Quat& a, const Quat& b);
Quat conjugate(const Quat& q);
Quat normalize(const Quat& q);
Vec3 rotate(const Quat& q, const Vec3& v);
Quat from_axis_angle(const Vec3& axis, Real angle);
Mat3x3 to_rotation_matrix(const Quat& q);
```

#### Day 5: Unit Tests
```cpp
// File: tests/unit/test_math.cpp
TEST(Vec3Test, Addition) {
    Vec3 a{1, 2, 3};
    Vec3 b{4, 5, 6};
    Vec3 c = a + b;
    EXPECT_NEAR(c.x, 5.0, 1e-10);
    EXPECT_NEAR(c.y, 7.0, 1e-10);
    EXPECT_NEAR(c.z, 9.0, 1e-10);
}

TEST(Vec3Test, CrossProduct) {
    Vec3 i{1, 0, 0};
    Vec3 j{0, 1, 0};
    Vec3 k = cross(i, j);
    EXPECT_NEAR(k.x, 0.0, 1e-10);
    EXPECT_NEAR(k.y, 0.0, 1e-10);
    EXPECT_NEAR(k.z, 1.0, 1e-10);
}

TEST(QuatTest, Rotation) {
    Quat q = from_axis_angle({0, 0, 1}, M_PI / 2);  // 90° around Z
    Vec3 v{1, 0, 0};
    Vec3 rotated = rotate(q, v);
    EXPECT_NEAR(rotated.x, 0.0, 1e-10);
    EXPECT_NEAR(rotated.y, 1.0, 1e-10);
    EXPECT_NEAR(rotated.z, 0.0, 1e-10);
}
```

### Sprint 1 Definition of Done
- [ ] All math operations implemented
- [ ] Unit tests pass with <1e-10 precision
- [ ] No compiler warnings

---

## Sprint 2: Memory and Entity System (1 week)

### Goal
Implement SoA entity storage with proper alignment.

### Tasks

#### Day 1-2: Memory Alignment
```cpp
// File: include/jaguar/core/memory.h
// Implement aligned allocation

template<typename T, size_t Alignment = 64>
class AlignedVector {
public:
    T* data() { return data_; }
    size_t size() const { return size_; }
    void resize(size_t n);
    void push_back(const T& value);
    T& operator[](size_t i);
private:
    T* data_ = nullptr;
    size_t size_ = 0;
    size_t capacity_ = 0;
};
```

#### Day 3-5: Entity Storage (SoA)
```cpp
// File: src/physics/entity_manager.cpp
// Key implementation

class EntityStateStorage {
public:
    EntityId create() {
        EntityId id = next_id_++;
        positions_.push_back({0, 0, 0});
        velocities_.push_back({0, 0, 0});
        orientations_.push_back({1, 0, 0, 0});
        angular_velocities_.push_back({0, 0, 0});
        masses_.push_back(1.0);
        return id;
    }

    void set_position(EntityId id, const Vec3& pos) {
        positions_[id] = pos;
    }

    Vec3 get_position(EntityId id) const {
        return positions_[id];
    }

private:
    // Verify 64-byte alignment
    static_assert(alignof(decltype(positions_)) >= 64);

    alignas(64) AlignedVector<Vec3> positions_;
    alignas(64) AlignedVector<Vec3> velocities_;
    alignas(64) AlignedVector<Quat> orientations_;
    alignas(64) AlignedVector<Vec3> angular_velocities_;
    alignas(64) AlignedVector<Real> masses_;
    EntityId next_id_ = 0;
};
```

### Sprint 2 Definition of Done
- [ ] EntityStateStorage compiles
- [ ] Alignment verified (check with debugger)
- [ ] Can create/destroy 10,000 entities
- [ ] Memory access tests pass

---

## Sprint 3: RK4 Integration (1 week)

### Goal
Implement RK4 integrator for position, velocity, and quaternion.

### Tasks

#### Day 1-3: RK4 Core
```cpp
// File: src/physics/integrators/rk4.cpp

void RK4Integrator::integrate(
    EntityStateStorage& storage,
    const std::vector<Vec3>& forces,
    Real dt)
{
    const size_t n = storage.size();

    for (size_t i = 0; i < n; ++i) {
        Vec3 pos = storage.get_position(i);
        Vec3 vel = storage.get_velocity(i);
        Real mass = storage.get_mass(i);
        Vec3 accel = forces[i] * (1.0 / mass);

        // RK4 for position
        Vec3 k1_v = vel;
        Vec3 k1_a = accel;

        Vec3 k2_v = vel + k1_a * (dt * 0.5);
        Vec3 k2_a = accel;  // Assuming constant force over timestep

        Vec3 k3_v = vel + k2_a * (dt * 0.5);
        Vec3 k3_a = accel;

        Vec3 k4_v = vel + k3_a * dt;
        Vec3 k4_a = accel;

        Vec3 new_pos = pos + (k1_v + k2_v * 2.0 + k3_v * 2.0 + k4_v) * (dt / 6.0);
        Vec3 new_vel = vel + (k1_a + k2_a * 2.0 + k3_a * 2.0 + k4_a) * (dt / 6.0);

        storage.set_position(i, new_pos);
        storage.set_velocity(i, new_vel);
    }
}
```

#### Day 4-5: Quaternion Integration
```cpp
// File: src/physics/integrators/quaternion_utils.cpp

Quat integrate_quaternion(const Quat& q, const Vec3& omega, Real dt) {
    // q_dot = 0.5 * q * omega_quat
    Quat omega_quat{0, omega.x, omega.y, omega.z};
    Quat q_dot = q * omega_quat * 0.5;

    // Euler step (for simplicity; RK4 can be added later)
    Quat new_q{
        q.w + q_dot.w * dt,
        q.x + q_dot.x * dt,
        q.y + q_dot.y * dt,
        q.z + q_dot.z * dt
    };

    return normalize(new_q);
}
```

### Validation Test
```cpp
// Test: Simple harmonic oscillator
// Spring: F = -k*x, k = 1, m = 1
// Analytical: x(t) = cos(t), should return to x=1 at t=2π

TEST(RK4Test, HarmonicOscillator) {
    EntityStateStorage storage;
    EntityId id = storage.create();
    storage.set_position(id, {1, 0, 0});
    storage.set_velocity(id, {0, 0, 0});
    storage.set_mass(id, 1.0);

    RK4Integrator integrator;
    const Real k = 1.0;
    const Real dt = 0.001;
    const int steps = static_cast<int>(2 * M_PI / dt);

    for (int i = 0; i < steps; ++i) {
        Vec3 pos = storage.get_position(id);
        Vec3 force = {-k * pos.x, 0, 0};
        integrator.integrate(storage, {force}, dt);
    }

    Vec3 final_pos = storage.get_position(id);
    EXPECT_NEAR(final_pos.x, 1.0, 1e-4);  // Should return near initial
}
```

### Sprint 3 Definition of Done
- [ ] RK4 integrator implemented
- [ ] Harmonic oscillator test passes (<1e-4 error)
- [ ] Quaternion integration works

---

## Sprint 4: Physics System and Engine (1 week)

### Goal
Create the main simulation loop with time management.

### Tasks

#### Day 1-2: TimeManager
```cpp
// File: src/core/time_manager.cpp

class TimeManager {
public:
    void start() { start_time_ = std::chrono::steady_clock::now(); }

    void advance(Real dt) {
        sim_time_ += dt;
        frame_count_++;
    }

    Real get_sim_time() const { return sim_time_; }
    Real get_delta_time() const { return delta_time_; }

    void set_fixed_timestep(Real dt) { delta_time_ = dt; }

private:
    Real sim_time_ = 0.0;
    Real delta_time_ = 0.01;  // 100 Hz default
    uint64_t frame_count_ = 0;
    std::chrono::steady_clock::time_point start_time_;
};
```

#### Day 3-4: PhysicsSystem
```cpp
// File: src/physics/physics_system.cpp

class PhysicsSystem {
public:
    void update(EntityStateStorage& storage, Real dt) {
        // 1. Accumulate forces
        std::vector<Vec3> forces(storage.size(), {0, 0, 0});

        for (auto& generator : force_generators_) {
            generator->apply(storage, forces);
        }

        // 2. Integrate
        integrator_.integrate(storage, forces, dt);
    }

    void add_force_generator(std::unique_ptr<IForceGenerator> gen) {
        force_generators_.push_back(std::move(gen));
    }

private:
    RK4Integrator integrator_;
    std::vector<std::unique_ptr<IForceGenerator>> force_generators_;
};
```

#### Day 5: Engine Executive
```cpp
// File: src/core/engine_exec.cpp

class Engine {
public:
    bool initialize(const std::string& config_path = "") {
        time_manager_.set_fixed_timestep(0.01);

        // Add gravity by default
        physics_.add_force_generator(
            std::make_unique<GravityGenerator>(9.80665));

        initialized_ = true;
        return true;
    }

    void step(Real dt) {
        physics_.update(entities_, dt);
        time_manager_.advance(dt);
    }

    void run_for(Real duration) {
        Real dt = time_manager_.get_delta_time();
        while (time_manager_.get_sim_time() < duration) {
            step(dt);
        }
    }

    EntityId create_entity(const std::string& name, Domain domain) {
        EntityId id = entities_.create();
        // Additional setup based on domain
        return id;
    }

    Real get_time() const { return time_manager_.get_sim_time(); }

private:
    TimeManager time_manager_;
    PhysicsSystem physics_;
    EntityStateStorage entities_;
    bool initialized_ = false;
};
```

### Integration Test: Ballistic Trajectory
```cpp
TEST(EngineTest, BallisticTrajectory) {
    Engine engine;
    engine.initialize();

    EntityId projectile = engine.create_entity("bullet", Domain::Air);
    engine.set_entity_state(projectile, {
        .position = {0, 0, 0},
        .velocity = {100, 0, 100},  // 45° angle, 141 m/s
        .mass = 1.0
    });

    // Run until it hits ground (z = 0 again)
    engine.run_for(20.4);  // Analytical time to land: 2*100/9.8 ≈ 20.4s

    auto state = engine.get_entity_state(projectile);

    // Should land at approximately x = 2040m (100 * 20.4)
    EXPECT_NEAR(state.position.x, 2040.8, 10.0);
    EXPECT_NEAR(state.position.z, 0.0, 1.0);
}
```

### Sprint 4 Definition of Done
- [ ] Engine lifecycle works (init, step, shutdown)
- [ ] Ballistic trajectory matches analytical solution
- [ ] Can run 10 seconds of simulation

---

## Sprint 5: First Working Example (1 week)

### Goal
Get `examples/simple_flight/main.cpp` to compile and run.

### Tasks

#### Day 1-3: Complete API Surface
Make sure all public API functions work:
- `Engine::initialize()`
- `Engine::create_entity()`
- `Engine::set_entity_state()`
- `Engine::get_entity_state()`
- `Engine::step()`
- `Engine::run_for()`
- `Engine::get_time()`
- `Engine::shutdown()`

#### Day 4-5: Run Simple Flight Example
```bash
# Build and run
cd build
cmake --build . --target simple_flight
./examples/simple_flight/simple_flight
```

Expected output:
```
JaguarEngine Simple Flight Example
Version: 0.1.0

Running simulation for 10.00 seconds...

Time(s)  X(m)       Y(m)       Z(m)       Vx(m/s)
-------  ---------  ---------  ---------  --------
   0.00       0.00       0.00  -10000.00    250.00
   1.00     250.00       0.00   -9995.10    250.00
   2.00     500.00       0.00   -9980.40    250.00
...
  10.00    2500.00       0.00   -9509.75    250.00

Simulation complete.
```

### Sprint 5 Definition of Done
- [ ] `simple_flight` example runs
- [ ] Output looks reasonable (altitude decreasing due to gravity)
- [ ] No crashes or memory errors

---

## Next Steps After Sprint 5

Once the foundation is complete, proceed to Phase 2:

1. **XML Parsing** (Week 6-7)
   - Integrate pugixml
   - Parse F-16.xml entity definition

2. **Aerodynamics** (Week 8-9)
   - Implement coefficient tables
   - Implement force calculation

3. **Terrain** (Week 10-11)
   - Integrate GDAL
   - Implement elevation queries

---

## Quick Reference: Build Commands

```bash
# Full rebuild
cd build && cmake --build . --clean-first

# Run all tests
ctest --output-on-failure

# Run specific test
./tests/unit/jaguar_unit_tests --gtest_filter=Vec3Test.*

# Run benchmarks
./tests/benchmarks/jaguar_bench

# Check for memory leaks (Linux)
valgrind --leak-check=full ./tests/unit/jaguar_unit_tests

# Generate compile_commands.json for IDE
cmake .. -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

---

## Troubleshooting

### Common Issues

| Issue | Solution |
|-------|----------|
| CMake not finding GTest | Run `cmake .. -DFETCHCONTENT_QUIET=OFF` to see download progress |
| Alignment errors | Check that AlignedVector uses `aligned_alloc` or `_aligned_malloc` |
| Precision failures | Use `EXPECT_NEAR` with appropriate epsilon (1e-10 for double) |
| Linker errors | Ensure all `.cpp` files are added to CMakeLists.txt |

### Debug Tips

```cpp
// Add to any function for debugging
#include <iostream>
std::cout << "Position: " << pos.x << ", " << pos.y << ", " << pos.z << "\n";

// Check alignment
std::cout << "Alignment: " << alignof(decltype(positions_)) << "\n";
std::cout << "Address: " << static_cast<void*>(positions_.data()) << "\n";
assert(reinterpret_cast<uintptr_t>(positions_.data()) % 64 == 0);
```

---

## Success Criteria Summary

| Sprint | Deliverable | Test |
|--------|-------------|------|
| 1 | Math library | Unit tests pass |
| 2 | Entity storage | 10K entities, alignment verified |
| 3 | RK4 integrator | Harmonic oscillator <1e-4 error |
| 4 | Engine executive | Ballistic trajectory correct |
| 5 | Working example | simple_flight runs |

After Sprint 5, you have a working physics simulation engine ready for domain-specific expansion.

