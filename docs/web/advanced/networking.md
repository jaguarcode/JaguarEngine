# Network Integration

This guide covers JaguarEngine's network interfaces for distributed simulation, including DIS (Distributed Interactive Simulation) and HLA (High Level Architecture).

## Overview

JaguarEngine supports industry-standard networking protocols for Live-Virtual-Constructive (LVC) integration:

| Protocol | Standard | Use Case |
|----------|----------|----------|
| DIS | IEEE 1278 | Real-time entity state exchange |
| HLA | IEEE 1516 | Federation-based distributed simulation |

## DIS Protocol

### Configuration

Enable DIS in the engine configuration:

```xml
<engine_config>
    <network>
        <dis enabled="true">
            <port>3000</port>
            <site_id>1</site_id>
            <app_id>1</app_id>
            <exercise_id>1</exercise_id>
        </dis>
    </network>
</engine_config>
```

### Supported PDUs

| PDU Type | Direction | Purpose |
|----------|-----------|---------|
| Entity State | TX/RX | Position, velocity, orientation |
| Fire | TX/RX | Weapon discharge events |
| Detonation | TX/RX | Impact and explosion events |
| Collision | TX | Physical contact detection |
| Start/Resume | RX | Simulation control |
| Stop/Freeze | RX | Simulation control |

### C++ API

```cpp
#include <jaguar/network/dis.h>

// Configure DIS
jaguar::network::DISConfig dis_cfg;
dis_cfg.enabled = true;
dis_cfg.port = 3000;
dis_cfg.site_id = 1;
dis_cfg.app_id = 1;

// Initialize DIS interface
jaguar::network::DISInterface dis;
dis.initialize(dis_cfg);

// Register entity for DIS publication
EntityId aircraft = engine.create_entity("F16", Domain::Air);
dis.register_entity(aircraft, DISEntityType::Aircraft);

// In simulation loop
while (running) {
    engine.step(dt);

    // Publish entity states
    dis.publish_entity_states();

    // Process incoming PDUs
    dis.process_received_pdus();

    // Handle fire events
    if (dis.has_fire_events()) {
        for (const auto& fire : dis.get_fire_events()) {
            handle_fire_event(fire);
        }
    }
}

dis.shutdown();
```

### Entity Type Mapping

```cpp
// Configure entity type for DIS
DISEntityType entity_type;
entity_type.kind = 1;        // Platform
entity_type.domain = 2;      // Air
entity_type.country = 225;   // USA
entity_type.category = 1;    // Fighter
entity_type.subcategory = 2; // F-16

dis.set_entity_type(aircraft, entity_type);
```

## HLA Interface

### Configuration

```xml
<engine_config>
    <network>
        <hla enabled="true">
            <federation>ExampleFederation</federation>
            <federate>JaguarEngine</federate>
            <fom>RPR_FOM_v2.0.xml</fom>
            <rti>Portico</rti>
        </hla>
    </network>
</engine_config>
```

### RPR FOM Object Classes

JaguarEngine supports RPR FOM 2.0 object classes:

- `BaseEntity.PhysicalEntity`
  - `Platform.Aircraft`
  - `Platform.GroundVehicle`
  - `Platform.SurfaceVessel`
  - `Platform.Spacecraft`
  - `Munition`

### Interaction Classes

- `WeaponFire` - Weapon discharge events
- `MunitionDetonation` - Impact and explosion events
- `RadioSignal` - Communication events

### C++ API

```cpp
#include <jaguar/network/hla.h>

// Configure HLA
jaguar::network::HLAConfig hla_cfg;
hla_cfg.enabled = true;
hla_cfg.federation = "ExampleFederation";
hla_cfg.federate = "JaguarEngine";
hla_cfg.fom_path = "RPR_FOM_v2.0.xml";

// Initialize HLA federate
jaguar::network::HLAFederate federate;
federate.initialize(hla_cfg);

// Join federation
federate.join_federation();

// Register object classes
federate.subscribe_object_class("Platform.Aircraft");
federate.publish_object_class("Platform.Aircraft");

// Register entity as HLA object
EntityId aircraft = engine.create_entity("F16", Domain::Air);
auto hla_handle = federate.register_object(aircraft, "Platform.Aircraft");

// In simulation loop
while (running) {
    engine.step(dt);

    // Update HLA attributes
    federate.update_attributes();

    // Process reflections
    federate.tick();

    // Handle discovered objects
    for (const auto& obj : federate.get_discovered_objects()) {
        create_remote_entity(obj);
    }
}

// Leave federation
federate.resign();
federate.shutdown();
```

### Time Management

```cpp
// Enable time management
federate.enable_time_regulation(lookahead);
federate.enable_time_constrained();

// Advance time
while (running) {
    federate.request_time_advance(current_time + dt);

    // Wait for grant
    while (!federate.is_time_granted()) {
        federate.tick();
    }

    engine.step(dt);
}
```

## Dead Reckoning

JaguarEngine implements dead reckoning algorithms for bandwidth optimization:

```cpp
// Configure dead reckoning
DeadReckoningConfig dr_cfg;
dr_cfg.algorithm = DeadReckoningAlgorithm::DRM_FPW;  // Fixed velocity
dr_cfg.position_threshold = 1.0;   // meters
dr_cfg.orientation_threshold = 0.05;  // radians

dis.set_dead_reckoning(aircraft, dr_cfg);

// Entity state PDUs are only sent when thresholds are exceeded
```

### Supported Algorithms

| Algorithm | Description |
|-----------|-------------|
| DRM_STATIC | No dead reckoning |
| DRM_FPW | Fixed position, world coordinates |
| DRM_RPW | Rotating position, world coordinates |
| DRM_RVW | Rotating velocity, world coordinates |
| DRM_FVW | Fixed velocity, world coordinates |
| DRM_FPB | Fixed position, body coordinates |
| DRM_RPB | Rotating position, body coordinates |

## Python API

```python
import pyjaguar as jag

# Create engine with DIS
config = jag.EngineConfig()
config.dis_enabled = True
config.dis_port = 3000
config.dis_site_id = 1
config.dis_app_id = 1

engine = jag.Engine()
engine.initialize(config)

# Create entity
aircraft = engine.create_entity("F16", jag.Domain.Air)

# Run with network
while running:
    engine.step(0.01)
    # DIS PDUs are automatically exchanged

engine.shutdown()
```

## Best Practices

### Performance

1. **Use dead reckoning**: Reduces network traffic by 80-90%
2. **Batch updates**: Group attribute updates where possible
3. **Filter subscriptions**: Only subscribe to needed object classes

### Reliability

1. **Handle disconnections**: Implement reconnection logic
2. **Validate incoming data**: Check for invalid states
3. **Log network events**: Enable debug logging for troubleshooting

### Interoperability

1. **Follow FOM conventions**: Use standard attribute names and types
2. **Test with multiple RTIs**: Verify compatibility with different RTI implementations
3. **Document custom extensions**: Clearly document any FOM extensions

## Example: Multi-Federate Exercise

```cpp
// Federate 1: Aircraft simulation
jaguar::Engine aircraft_sim;
aircraft_sim.initialize(aircraft_config);

jaguar::network::HLAFederate fed1;
fed1.initialize(hla_config);
fed1.join_federation();
fed1.publish_object_class("Platform.Aircraft");
fed1.subscribe_object_class("Platform.GroundVehicle");

// Federate 2: Ground vehicle simulation
jaguar::Engine ground_sim;
ground_sim.initialize(ground_config);

jaguar::network::HLAFederate fed2;
fed2.initialize(hla_config);
fed2.join_federation();
fed2.publish_object_class("Platform.GroundVehicle");
fed2.subscribe_object_class("Platform.Aircraft");

// Both federates now exchange entity states automatically
```

## See Also

- [Architecture](architecture.md) - System architecture
- [Configuration](../api/configuration.md) - Configuration options
- [Examples](../tutorials/examples.md) - Complete examples
