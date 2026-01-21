# Multi-Entity Tutorial

This tutorial demonstrates how to simulate multiple interacting entities using JaguarEngine.

## Overview

Multi-entity simulations involve:

- Managing multiple entities simultaneously
- Inter-entity interactions (weapons, sensors)
- Performance optimization for large scenarios
- Network distribution (DIS/HLA)

## Tutorial: Air Combat Scenario

### Creating Multiple Aircraft

```cpp
#include <jaguar/jaguar.h>

using namespace jaguar;

int main() {
    Engine engine;
    engine.initialize();

    // Create blue force (4 fighters)
    std::vector<EntityId> blue_aircraft;
    for (int i = 0; i < 4; ++i) {
        EntityId ac = engine.create_entity("Blue-" + std::to_string(i+1),
                                           Domain::Air);

        physics::EntityState state;
        state.position = Vec3{0.0, i * 500.0, -8000.0};  // Formation
        state.velocity = Vec3{250.0, 0.0, 0.0};
        state.mass = 12000.0;
        engine.set_entity_state(ac, state);

        blue_aircraft.push_back(ac);
    }

    // Create red force (4 fighters)
    std::vector<EntityId> red_aircraft;
    for (int i = 0; i < 4; ++i) {
        EntityId ac = engine.create_entity("Red-" + std::to_string(i+1),
                                           Domain::Air);

        physics::EntityState state;
        state.position = Vec3{50000.0, i * 500.0, -8000.0};  // Opposing
        state.velocity = Vec3{-250.0, 0.0, 0.0};
        state.mass = 12000.0;
        engine.set_entity_state(ac, state);

        red_aircraft.push_back(ac);
    }

    // Simulation loop
    Real dt = 0.01;
    Real duration = 120.0;

    for (Real t = 0; t < duration; t += dt) {
        // Update all entities
        for (EntityId ac : blue_aircraft) {
            update_aircraft(engine, ac, dt);
        }
        for (EntityId ac : red_aircraft) {
            update_aircraft(engine, ac, dt);
        }

        engine.step(dt);
    }

    engine.shutdown();
    return 0;
}
```

### Entity Interaction: Detection

```cpp
struct DetectionResult {
    EntityId target;
    Real range;
    Real bearing;
    Real elevation;
};

std::vector<DetectionResult> detect_targets(
    Engine& engine,
    EntityId sensor_entity,
    const std::vector<EntityId>& potential_targets,
    Real max_range,
    Real fov_deg)
{
    std::vector<DetectionResult> detections;
    auto sensor_state = engine.get_entity_state(sensor_entity);

    for (EntityId target : potential_targets) {
        auto target_state = engine.get_entity_state(target);

        // Range
        Vec3 los = target_state.position - sensor_state.position;
        Real range = los.norm();

        if (range > max_range) continue;

        // Convert to body frame
        Vec3 los_body = sensor_state.orientation.conjugate().rotate(los);

        // Bearing and elevation
        Real bearing = std::atan2(los_body.y, los_body.x);
        Real elevation = std::asin(-los_body.z / range);

        // FOV check
        Real off_boresight = std::acos(los_body.x / range);
        if (off_boresight * RAD_TO_DEG > fov_deg / 2) continue;

        // Detection
        DetectionResult det;
        det.target = target;
        det.range = range;
        det.bearing = bearing;
        det.elevation = elevation;
        detections.push_back(det);
    }

    return detections;
}
```

### Entity Interaction: Weapons

```cpp
EntityId launch_missile(Engine& engine, EntityId launcher, EntityId target) {
    auto launcher_state = engine.get_entity_state(launcher);

    // Create missile
    EntityId missile = engine.create_entity("AIM-120", Domain::Air);

    // Initial state from launcher
    physics::EntityState state;
    state.position = launcher_state.position;
    state.velocity = launcher_state.velocity +
        launcher_state.orientation.rotate(Vec3{50.0, 0.0, 0.0});
    state.orientation = launcher_state.orientation;
    state.mass = 150.0;
    engine.set_entity_state(missile, state);

    // Store target association (for guidance)
    // ... implementation-specific ...

    return missile;
}

void update_missile_guidance(Engine& engine, EntityId missile, EntityId target) {
    auto missile_state = engine.get_entity_state(missile);
    auto target_state = engine.get_entity_state(target);

    // Proportional navigation
    Vec3 los = target_state.position - missile_state.position;
    Vec3 rel_vel = target_state.velocity - missile_state.velocity;

    Real range = los.norm();
    Vec3 los_rate = los.cross(rel_vel) / (range * range);

    // Acceleration command
    Real N = 4.0;  // Navigation constant
    Vec3 accel_cmd = missile_state.velocity.cross(los_rate) * N;

    // Apply guidance command as force
    physics::EntityForces forces;
    forces.add_force(accel_cmd * missile_state.mass);
    engine.apply_forces(missile, forces);
}
```

## Tutorial: Combined Arms Scenario

### Mixed Domain Entities

```cpp
// Create ground forces
EntityId tank1 = engine.create_entity("Tank-1", Domain::Land);
EntityId tank2 = engine.create_entity("Tank-2", Domain::Land);
EntityId apc1 = engine.create_entity("APC-1", Domain::Land);

// Create air support
EntityId helo = engine.create_entity("Apache", Domain::Air);

// Create naval support
EntityId ship = engine.create_entity("DDG-51", Domain::Sea);

// Simulation with different time steps
Real fast_dt = 0.01;  // Aircraft
Real slow_dt = 0.02;  // Ground/sea

for (Real t = 0; t < duration; t += fast_dt) {
    // Always update aircraft
    update_aircraft(engine, helo, fast_dt);

    // Update ground/sea at lower rate
    if (std::fmod(t, slow_dt) < fast_dt) {
        update_ground_vehicle(engine, tank1, slow_dt);
        update_ground_vehicle(engine, tank2, slow_dt);
        update_ground_vehicle(engine, apc1, slow_dt);
        update_ship(engine, ship, slow_dt);
    }

    engine.step(fast_dt);
}
```

## Performance Optimization

### Spatial Partitioning

```cpp
class SpatialGrid {
public:
    SpatialGrid(Real cell_size) : cell_size_(cell_size) {}

    void update(Engine& engine, const std::vector<EntityId>& entities) {
        cells_.clear();
        for (EntityId id : entities) {
            auto state = engine.get_entity_state(id);
            CellKey key = get_cell(state.position);
            cells_[key].push_back(id);
        }
    }

    std::vector<EntityId> get_nearby(const Vec3& position, Real radius) {
        std::vector<EntityId> result;
        int cells_to_check = static_cast<int>(radius / cell_size_) + 1;

        CellKey center = get_cell(position);
        for (int dx = -cells_to_check; dx <= cells_to_check; ++dx) {
            for (int dy = -cells_to_check; dy <= cells_to_check; ++dy) {
                CellKey key{center.x + dx, center.y + dy};
                if (cells_.count(key)) {
                    for (EntityId id : cells_[key]) {
                        result.push_back(id);
                    }
                }
            }
        }
        return result;
    }

private:
    struct CellKey {
        int x, y;
        bool operator<(const CellKey& other) const {
            return std::tie(x, y) < std::tie(other.x, other.y);
        }
    };

    CellKey get_cell(const Vec3& pos) {
        return {static_cast<int>(pos.x / cell_size_),
                static_cast<int>(pos.y / cell_size_)};
    }

    Real cell_size_;
    std::map<CellKey, std::vector<EntityId>> cells_;
};
```

### Parallel Processing

```cpp
#include <thread>
#include <future>

void update_entities_parallel(Engine& engine,
                              std::vector<EntityId>& entities,
                              Real dt) {
    // Split entities among threads
    size_t num_threads = std::thread::hardware_concurrency();
    size_t chunk_size = entities.size() / num_threads;

    std::vector<std::future<void>> futures;
    for (size_t i = 0; i < num_threads; ++i) {
        size_t start = i * chunk_size;
        size_t end = (i == num_threads - 1) ? entities.size()
                                            : (i + 1) * chunk_size;

        futures.push_back(std::async(std::launch::async, [&, start, end]() {
            for (size_t j = start; j < end; ++j) {
                compute_forces(engine, entities[j], dt);
            }
        }));
    }

    // Wait for all threads
    for (auto& f : futures) {
        f.get();
    }

    // Step engine (single-threaded)
    engine.step(dt);
}
```

## Network Distribution

### DIS Integration

```cpp
#include <jaguar/network/dis.h>

// Configure DIS
network::DISConfig dis_config;
dis_config.port = 3000;
dis_config.site_id = 1;
dis_config.app_id = 1;

network::DISInterface dis;
dis.initialize(dis_config);

// Register entities for DIS
for (EntityId id : local_entities) {
    dis.register_entity(id, get_dis_type(id));
}

// Simulation loop
while (running) {
    // Receive remote entity updates
    dis.process_received_pdus();

    // Update local entities
    for (EntityId id : local_entities) {
        update_entity(engine, id, dt);
    }

    // Publish local entity states
    dis.publish_entity_states();

    engine.step(dt);
}
```

## Best Practices

### Entity Management

1. **Group by domain**: Process similar entities together
2. **Use spatial partitioning**: For interaction queries
3. **Cull distant entities**: Skip detailed physics for far entities

### Performance

1. **Profile first**: Identify actual bottlenecks
2. **Batch operations**: Minimize per-entity overhead
3. **Consider multi-rate**: Different update rates for different fidelity needs

### Scalability

- 100s of entities: Single-threaded fine
- 1000s of entities: Consider parallel processing
- 10000+ entities: Use spatial partitioning, LOD, network distribution

## See Also

- [Architecture](../advanced/architecture.md) - System architecture
- [Network Integration](../advanced/networking.md) - DIS/HLA
- [API Reference](../api/overview.md) - Complete API
