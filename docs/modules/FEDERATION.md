# Federation Module - Distributed Simulation Interoperability

## Overview

The Federation module provides standards-compliant interoperability capabilities for JaguarEngine, enabling distributed simulation across heterogeneous systems. It implements two major simulation interoperability standards:

- **DIS (Distributed Interactive Simulation)** - IEEE 1278.1-2012
- **HLA (High Level Architecture)** - IEEE 1516-2010 (Evolved)

These protocols enable JaguarEngine simulations to participate in larger distributed exercises with other simulation systems, live training systems, and operational systems.

## Key Features

### DIS Protocol (IEEE 1278.1)
- Complete PDU type definitions (Entity State, Fire, Detonation, Collision)
- Dead reckoning algorithms for bandwidth optimization
- Geocentric coordinate transformations (ECEF ↔ LLA)
- Entity type enumeration system (SISO-REF-010)
- Big-endian encoding/decoding for network transmission
- UDP multicast networking support

### HLA RTI (IEEE 1516-2010)
- Complete RTI Ambassador interface
- Federate Ambassador callbacks
- Time management (time-constrained and time-regulating)
- Object management (publish, subscribe, update)
- Interaction management
- Ownership management
- Data Distribution Management (DDM)
- Federation synchronization points
- Save/Restore capabilities

## Architecture

### DIS Protocol Stack

```
┌─────────────────────────────────────────────────────────────────┐
│                     DIS Protocol Layer                          │
├─────────────────────────────────────────────────────────────────┤
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐  │
│  │   PDU Types  │  │ Dead Reckon. │  │  Coordinate Xform    │  │
│  │ Entity State │  │   FPW/RPW    │  │   LLA ↔ ECEF         │  │
│  │ Fire/Deton.  │  │   FVW/RVW    │  │   Euler ↔ Quat       │  │
│  └──────────────┘  └──────────────┘  └──────────────────────┘  │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │                      IDISCodec                            │  │
│  │          Big-endian encoding/decoding                     │  │
│  └──────────────────────────────────────────────────────────┘  │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │                     IDISNetwork                           │  │
│  │          UDP multicast send/receive                       │  │
│  └──────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

### HLA RTI Stack

```
┌─────────────────────────────────────────────────────────────────┐
│                         Application                              │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────────────────┐  ┌────────────────────────────┐   │
│  │    IRTIAmbassador       │  │   IFederateAmbassador      │   │
│  │  (Federate → RTI)       │  │     (RTI → Federate)       │   │
│  │                         │  │                            │   │
│  │  - Federation Mgmt      │  │  - Object Callbacks        │   │
│  │  - Object Mgmt          │  │  - Interaction Callbacks   │   │
│  │  - Interaction Mgmt     │  │  - Time Callbacks          │   │
│  │  - Time Mgmt            │  │  - Ownership Callbacks     │   │
│  │  - Ownership Mgmt       │  │                            │   │
│  └─────────────────────────┘  └────────────────────────────┘   │
├─────────────────────────────────────────────────────────────────┤
│                        RTI Backend                               │
│         (HLA Evolver, Pitch, Portico, MAK, etc.)                │
└─────────────────────────────────────────────────────────────────┘
```

## API Reference

### DIS Protocol

#### Entity Identifier

```cpp
#include <jaguar/federation/dis_protocol.h>

using namespace jaguar::federation;

// Create entity identifier
EntityIdentifier entity_id(1, 1, 100);  // site, app, entity
ASSERT(entity_id.is_valid());

// Invalid identifier constant
EntityIdentifier invalid = INVALID_ENTITY_IDENTIFIER;
ASSERT(!invalid.is_valid());
```

#### Entity Type (SISO-REF-010)

```cpp
// Create F-16 fighter aircraft
auto f16 = EntityType::create_platform(
    PlatformDomain::Air,
    225,    // USA
    1,      // Fighter category
    3,      // Subcategory
    0       // Specific
);

// Create AIM-120 missile
auto aim120 = EntityType::create_munition(
    225,    // USA
    1,      // Guided category
    2,      // Subcategory
    3       // Specific
);
```

#### Entity State PDU

```cpp
EntityStatePDU pdu;

// Entity identification
pdu.entity_id = EntityIdentifier(1, 1, 100);
pdu.force_id = ForceId::Friendly;
pdu.entity_type = EntityType::create_platform(PlatformDomain::Air, 225, 1, 3, 0);

// Position (ECEF coordinates)
auto location_lla = GeodeticCoordinates::from_degrees(37.7749, -122.4194, 10000.0);
pdu.entity_location = lla_to_geocentric(location_lla);

// Velocity (m/s)
pdu.entity_linear_velocity = Vec3{250.0, 0.0, 0.0};

// Orientation (Euler angles)
pdu.entity_orientation = EulerAngles::from_degrees(90.0, 5.0, 0.0);

// Dead reckoning
pdu.dead_reckoning_params.algorithm = DeadReckoningAlgorithm::DRM_FVW;

// Marking/callsign
pdu.entity_marking = EntityMarking("VIPER01");

// Appearance
pdu.appearance.set_damage(0);  // No damage
pdu.appearance.set_lights(true);
```

#### Fire and Detonation PDUs

```cpp
// Fire event
FirePDU fire;
fire.firing_entity_id = EntityIdentifier(1, 1, 100);
fire.target_entity_id = EntityIdentifier(1, 1, 200);
fire.munition_id = EntityIdentifier(1, 1, 1000);
fire.event_id = EventIdentifier(1, 1, 1);
fire.velocity = Vec3{500.0, 0.0, -10.0};
fire.range = 5000.0;

// Detonation event
DetonationPDU detonation;
detonation.firing_entity_id = fire.firing_entity_id;
detonation.target_entity_id = fire.target_entity_id;
detonation.munition_id = fire.munition_id;
detonation.event_id = fire.event_id;
detonation.detonation_result = DetonationResult::EntityImpact;
detonation.location_in_world = target_position;
```

#### Dead Reckoning

```cpp
DeadReckoningCalculator calc;

// Extrapolate position
EntityStatePDU current_state = /* ... */;
Real elapsed_time = 0.5;  // seconds
EntityStatePDU predicted = calc.extrapolate_position(current_state, elapsed_time);

// Check if update is needed
bool needs_update = calc.should_send_update(
    actual_state,
    predicted_state,
    DIS_DR_THRESHOLD_POSITION,      // 1.0 meter
    DIS_DR_THRESHOLD_ORIENTATION    // 3.0 degrees
);
```

#### Coordinate Transformations

```cpp
// LLA to ECEF
GeodeticCoordinates lla = GeodeticCoordinates::from_degrees(37.7749, -122.4194, 100.0);
Vec3 ecef = lla_to_geocentric(lla);

// ECEF to LLA
GeodeticCoordinates lla_back = geocentric_to_lla(ecef);

// Euler to Quaternion
EulerAngles euler = EulerAngles::from_degrees(90.0, 45.0, 0.0);
Quat orientation = euler_to_orientation(euler);

// Quaternion to Euler
EulerAngles euler_back = orientation_to_euler(orientation);
```

#### Network Interface

```cpp
// Create codec and network
auto codec = create_dis_codec();
auto network = create_dis_network();

// Initialize network
network->initialize("239.1.2.3", 3000);  // Multicast group, port
network->join_exercise(1, 1);            // Site ID, Application ID

// Send PDU
EntityStatePDU pdu = /* ... */;
network->send_pdu(pdu);

// Receive PDU
UInt8 buffer[DIS_MAX_PDU_SIZE];
SizeT bytes = network->receive_pdu(buffer, sizeof(buffer));
if (bytes > 0) {
    auto header = codec->decode_header(buffer, bytes);
    if (header && header->pdu_type == PDUType::EntityState) {
        auto entity_state = codec->decode_entity_state(buffer, bytes);
        // Process entity state...
    }
}

// Cleanup
network->leave_exercise();
network->shutdown();
```

### HLA RTI

#### Configuration

```cpp
#include <jaguar/federation/hla_rti.h>

using namespace jaguar::federation::hla;

// Default configuration
auto config = HLAConfiguration::default_config();

// Custom configuration
HLAConfiguration config;
config.federation_name = "TrainingExercise";
config.federate_name = "JaguarSim01";
config.federate_type = "Simulator";
config.fom_module_paths = {"RPR_FOM_v2.xml"};
config.connection.rti_host = "rti.example.com";
config.connection.rti_port = 8989;
config.time_management.enable_time_regulation = true;
config.time_management.enable_time_constrained = true;
config.time_management.lookahead = LogicalTimeInterval(0.1);
```

#### Federation Management

```cpp
auto rti = create_rti_ambassador(config);

// Create or join federation
HLAResult result = rti->create_federation_execution(
    "TrainingExercise",
    {"RPR_FOM_v2.xml"}
);
if (result == HLAResult::FederationExecutionAlreadyExists) {
    // Federation exists, just join
}

result = rti->join_federation_execution(
    "JaguarSim01",
    "Simulator",
    "TrainingExercise"
);

// Synchronization point
rti->register_federation_synchronization_point("ReadyToRun", {});
rti->synchronization_point_achieved("ReadyToRun");

// Resign and destroy
rti->resign_federation_execution(ResignAction::DeleteObjectsThenDivest);
rti->destroy_federation_execution("TrainingExercise");
```

#### Object Management

```cpp
// Get handles
ObjectClassHandle aircraft_class;
rti->get_object_class_handle("HLAobjectRoot.BaseEntity.PhysicalEntity.Platform.Aircraft", aircraft_class);

AttributeHandle position_attr;
rti->get_attribute_handle(aircraft_class, "SpatialStatic", position_attr);

// Publish and subscribe
rti->publish_object_class_attributes(aircraft_class, {position_attr});
rti->subscribe_object_class_attributes(aircraft_class, {position_attr});

// Register object instance
ObjectInstanceHandle my_aircraft;
rti->register_object_instance(aircraft_class, my_aircraft, "Viper01");

// Update attributes
AttributeValueSet attributes;
attributes.set_attribute(position_attr, encode_position(position));
rti->update_attribute_values(my_aircraft, attributes, {}, LogicalTime(current_time));

// Delete object
rti->delete_object_instance(my_aircraft, {});
```

#### Interaction Management

```cpp
// Get handles
InteractionClassHandle fire_interaction;
rti->get_interaction_class_handle("HLAinteractionRoot.WeaponFire", fire_interaction);

ParameterHandle shooter_param;
rti->get_parameter_handle(fire_interaction, "FiringObjectIdentifier", shooter_param);

// Publish and subscribe
rti->publish_interaction_class(fire_interaction);
rti->subscribe_interaction_class(fire_interaction);

// Send interaction
ParameterValueSet params;
params.set_parameter(shooter_param, encode_entity_id(shooter_id));
rti->send_interaction(fire_interaction, params, {}, LogicalTime(current_time));
```

#### Time Management

```cpp
// Enable time management
rti->enable_time_regulation(LogicalTimeInterval(0.1));  // 100ms lookahead
rti->enable_time_constrained();

// Time advance
rti->time_advance_request(LogicalTime(target_time));

// Query time
LogicalTime galt;
rti->query_galt(galt);  // Greatest Available Logical Time
```

#### Federate Ambassador Callbacks

```cpp
class MyFederateAmbassador : public IFederateAmbassador {
public:
    // Object discovery
    void discover_object_instance(
        ObjectInstanceHandle object,
        ObjectClassHandle object_class,
        std::string_view instance_name) override {
        // New object discovered
    }

    // Attribute reflection
    void reflect_attribute_values(
        ObjectInstanceHandle object,
        const AttributeValueSet& attributes,
        const std::vector<UInt8>& tag) override {
        // Process attribute update
    }

    // Interaction reception
    void receive_interaction(
        InteractionClassHandle interaction,
        const ParameterValueSet& parameters,
        const std::vector<UInt8>& tag) override {
        // Process interaction
    }

    // Time advance grant
    void time_advance_grant(const LogicalTime& time) override {
        current_time = time;
    }
};
```

## Dead Reckoning Algorithms

| Algorithm | Description | Use Case |
|-----------|-------------|----------|
| DRM_Static | No extrapolation | Static objects (buildings) |
| DRM_FPW | Fixed Position, World | Slow-moving ground vehicles |
| DRM_RPW | Rotating Position, World | Rotating objects |
| DRM_FVW | Fixed Velocity, World | Aircraft, missiles |
| DRM_RVW | Rotating Velocity, World | Maneuvering aircraft |
| DRM_FPB | Fixed Position, Body | Body-frame motion |
| DRM_RPB | Rotating Position, Body | Rotating in body frame |
| DRM_FVB | Fixed Velocity, Body | Body-frame velocity |
| DRM_RVB | Rotating Velocity, Body | Full 6DOF body frame |

## Entity Types (SISO-REF-010)

### Entity Kinds
| Kind | Description |
|------|-------------|
| Platform | Aircraft, vehicles, ships |
| Munition | Missiles, bombs, bullets |
| Lifeform | Personnel |
| Environmental | Weather, obstacles |
| CulturalFeature | Buildings, bridges |
| Supply | Cargo, supplies |
| Radio | Radio equipment |
| Expendable | Chaff, flares |
| SensorEmitter | Radar, sensors |

### Platform Domains
| Domain | Description |
|--------|-------------|
| Land | Ground vehicles, tanks |
| Air | Aircraft, helicopters |
| Surface | Ships, boats |
| Subsurface | Submarines |
| Space | Satellites, spacecraft |

## Performance Considerations

### DIS
- Default heartbeat: 5 seconds
- Dead reckoning threshold: 1 meter position, 3 degrees orientation
- Maximum PDU size: 8192 bytes
- Typical port: UDP 3000
- Multicast groups: 239.x.x.x range

### HLA
- Lookahead: Trade-off between latency and synchronization
- Tick frequency: Balance RTI callbacks with simulation loop
- Publication scope: Only publish what other federates need
- Passive subscription: Reduce unnecessary updates

## DIS UDP Socket

The DIS UDP Socket provides cross-platform, thread-safe UDP communication for DIS protocol traffic.

### Socket Features

- **Cross-platform support**: POSIX sockets (Linux/macOS) and Winsock (Windows)
- **Multicast support**: Join/leave multicast groups, multicast TTL configuration
- **Broadcast support**: Enable/disable broadcast with SO_BROADCAST
- **MTU-aware fragmentation**: Automatic PDU fragmentation for large payloads (>1472 bytes)
- **Non-blocking I/O**: Configurable receive timeout (0ms = non-blocking)
- **Socket statistics**: Track packets/bytes sent/received, errors, fragments
- **Thread-safe**: Mutex-protected concurrent send/receive operations

### Socket Configuration

```cpp
#include <jaguar/federation/dis_socket.h>

using namespace jaguar::federation;

// Create default DIS socket configuration
DisSocketConfig config = DisSocketConfig::dis_default();
config.receive_buffer_size = 65536;  // 64KB socket buffer
config.send_buffer_size = 65536;
config.receive_timeout = std::chrono::milliseconds(100);  // 100ms timeout
config.multicast_ttl = 32;           // Hop count for multicast
config.multicast_loopback = true;    // Receive own multicast packets

// Create and initialize socket
DisSocket socket;
DisSocketError error = socket.initialize(config);
if (error != DisSocketError::Success) {
    std::cerr << "Failed to initialize socket: "
              << dis_socket_error_to_string(error) << std::endl;
}
```

### Multicast Group Management

```cpp
// Join multicast group
DisEndpoint multicast_group(DIS_DEFAULT_MULTICAST, DIS_DEFAULT_PORT);
error = socket.join_multicast_group(multicast_group);

// Send to multicast group
std::vector<UInt8> pdu_data = /* ... */;
socket.send_to(pdu_data.data(), pdu_data.size(), multicast_group);

// Leave multicast group
socket.leave_multicast_group(multicast_group);
```

### Receive Operations

```cpp
// Receive with timeout (non-blocking)
std::vector<UInt8> buffer(DIS_MAX_PAYLOAD_SIZE);
DisReceiveResult result;

DisSocketError error = socket.receive_from(buffer.data(), buffer.size(), result);
if (error == DisSocketError::Success) {
    std::cout << "Received " << result.bytes_received
              << " bytes from " << result.source.to_string() << std::endl;
    std::cout << "Timestamp: " << result.timestamp.time_since_epoch().count() << std::endl;
} else if (error == DisSocketError::Timeout) {
    // No data available within timeout
} else if (error == DisSocketError::WouldBlock) {
    // Non-blocking socket - no data available
}

// Poll for data availability
if (socket.poll_readable(std::chrono::milliseconds(500))) {
    // Data is available
}
```

### Multi-Endpoint Send

```cpp
// Send same PDU to multiple recipients
std::vector<DisEndpoint> endpoints = {
    DisEndpoint("192.168.1.100", 3000),
    DisEndpoint("192.168.1.101", 3000),
    DisEndpoint("192.168.1.102", 3000)
};

error = socket.send_to_multiple(
    pdu_data.data(),
    pdu_data.size(),
    endpoints.data(),
    endpoints.size()
);
```

### Fragmentation Handling

```cpp
// Large PDU (> MTU) with automatic fragmentation
std::vector<UInt8> large_pdu = /* ... (5000 bytes) */;

error = socket.send_fragmented(
    large_pdu.data(),
    large_pdu.size(),
    DisEndpoint("239.1.2.3", 3000)
);

// Note: DIS does not define a standard defragmentation protocol.
// Fragmentation should be avoided when possible by optimizing PDU size.
```

### Socket Statistics

```cpp
// Get statistics
const DisSocketStats& stats = socket.get_stats();

std::cout << "Packets sent: " << stats.packets_sent << std::endl;
std::cout << "Packets received: " << stats.packets_received << std::endl;
std::cout << "Bytes sent: " << stats.bytes_sent << std::endl;
std::cout << "Bytes received: " << stats.bytes_received << std::endl;
std::cout << "Send errors: " << stats.send_errors << std::endl;
std::cout << "Receive errors: " << stats.receive_errors << std::endl;
std::cout << "Fragmented sends: " << stats.fragments_sent << std::endl;

// Reset statistics
socket.reset_stats();
```

### Error Handling

```cpp
// Check and log errors
DisSocketError error = socket.send_to(buffer, size, endpoint);
if (error != DisSocketError::Success) {
    const char* error_str = dis_socket_error_to_string(error);
    std::string last_error = socket.get_last_error_message();
    std::cerr << "Socket error: " << error_str << " (" << last_error << ")" << std::endl;
}

// Common error codes:
// - BufferOverflow: PDU larger than configured MTU
// - InvalidMulticastAddress: Multicast group address not in 224.0.0.0/4 range
// - AddressInUse: Port already in use
// - PermissionDenied: Insufficient privileges for multicast
// - NetworkUnreachable: Destination network unreachable
```

### Utility Functions

```cpp
// Get local network interfaces
auto interfaces = get_network_interfaces();
for (const auto& [ifname, address] : interfaces) {
    std::cout << ifname << ": " << address << std::endl;
}

// Get default interface address
if (auto default_addr = get_default_interface_address()) {
    std::cout << "Default interface: " << *default_addr << std::endl;
}

// Validate IPv4 address
if (is_valid_ipv4("192.168.1.100")) {
    std::cout << "Valid IPv4 address" << std::endl;
}

// Check if port is available
if (is_port_available(3000)) {
    std::cout << "Port 3000 is available" << std::endl;
}

// Parse endpoint from string
auto endpoint = DisEndpoint::parse("239.1.2.3:3000");
if (endpoint) {
    std::cout << "Parsed: " << endpoint->to_string() << std::endl;
}
```

## Fire PDU Handler

The Fire PDU handler provides serialization, deserialization, and validation of Fire PDUs (Type 2).

### Fire PDU Structure

Fire PDUs are 96 bytes fixed size and include:

| Field | Size | Description |
|-------|------|-------------|
| Header | 12 bytes | PDU header (type, length, timestamp) |
| Firing Entity ID | 6 bytes | Entity that fired the weapon |
| Target Entity ID | 6 bytes | Intended target (may be invalid) |
| Munition ID | 6 bytes | Munition entity identifier |
| Event ID | 6 bytes | Unique event identifier |
| Fire Mission Index | 4 bytes | Fire mission reference |
| Location | 24 bytes | Fire location (3 x float64, ECEF meters) |
| Burst Descriptor | 16 bytes | Munition type, warhead, fuse |
| Velocity | 12 bytes | Initial velocity (3 x float32, m/s) |
| Range | 4 bytes | Range to target (float32, meters) |
| **Total** | **96 bytes** | |

### Serialization and Deserialization

```cpp
#include <jaguar/federation/dis_fire_pdu.h>

using namespace jaguar::federation;

// Create handler
FirePDUHandler handler;

// Create Fire PDU
FirePDU fire_pdu = handler.create_fire_pdu(
    EntityIdentifier(1, 1, 100),    // Firing entity (F-16)
    EntityIdentifier(1, 1, 200),    // Target entity (SAM site)
    EntityType::create_munition(225, 1, 2, 3),  // AIM-120 missile
    Vec3{-2476471.0, -4480342.0, 3555850.0},   // Fire location (ECEF)
    Vec3{-100.0, 20.0, 50.0},                   // Initial velocity (m/s)
    45000.0                                      // Range (meters)
);

// Set burst descriptor
handler.set_burst_descriptor(
    fire_pdu,
    EntityType::create_munition(225, 1, 2, 3),
    1,      // Warhead type (1 = High Explosive)
    2,      // Fuse type
    1,      // Quantity (1 round)
    0       // Rate of fire (0 = not specified)
);

// Serialize to network byte order (big-endian)
std::vector<UInt8> buffer(FirePDUHandler::get_fire_pdu_size());
SizeT bytes_written = handler.serialize(fire_pdu, buffer.data(), buffer.size());

if (bytes_written == 0) {
    std::cerr << "Serialization failed" << std::endl;
}

// Deserialize from network bytes
std::optional<FirePDU> received_pdu = handler.deserialize(buffer.data(), bytes_written);

if (received_pdu) {
    std::cout << "Firing entity: (" << received_pdu->firing_entity_id.site << ", "
              << received_pdu->firing_entity_id.application << ", "
              << received_pdu->firing_entity_id.entity << ")" << std::endl;
}
```

### Validation

```cpp
// Validate Fire PDU
if (handler.validate(fire_pdu)) {
    std::cout << "Fire PDU is valid" << std::endl;
} else {
    std::cerr << "Fire PDU validation failed" << std::endl;
}

// Validation checks:
// - Firing entity identifier is non-zero
// - Event identifier is valid
// - Range is non-negative
// - Velocity magnitude is reasonable (<1000 m/s)
```

## Detonation PDU Handler

The Detonation PDU handler provides serialization, deserialization, and validation of Detonation PDUs (Type 3).

### Detonation PDU Structure

Detonation PDUs have variable size based on articulation parameters:

| Field | Size | Description |
|-------|------|-------------|
| Header | 12 bytes | PDU header |
| Firing Entity ID | 6 bytes | Entity that fired the munition |
| Target Entity ID | 6 bytes | Impact target (may be invalid) |
| Munition ID | 6 bytes | Munition entity identifier |
| Event ID | 6 bytes | Unique event identifier (matches Fire PDU) |
| Velocity | 12 bytes | Velocity at detonation (3 x float32, m/s) |
| Location | 24 bytes | Detonation location (3 x float64, ECEF meters) |
| Burst Descriptor | 16 bytes | Munition type, warhead, fuse |
| Location in Entity | 12 bytes | Impact point in target frame (3 x float32) |
| Result | 1 byte | Detonation result code |
| Num Articulation | 1 byte | Number of articulation parameters |
| Padding | 2 bytes | Alignment padding |
| Articulation Params | Variable | 16 bytes each (max 255) |
| **Total Base** | **104 bytes** | Without articulation parameters |

### Detonation Result Enumeration

Detonation Results describe the outcome (30+ types):

```cpp
enum class DetonationResult : UInt8 {
    Other = 0,
    EntityImpact = 1,              // Direct hit on entity
    EntityProximateDetonation = 2, // Near-miss on entity
    GroundImpact = 3,              // Direct ground impact
    GroundProximateDetonation = 4, // Near-miss ground impact
    Detonation = 5,                // In-flight detonation
    None = 6,                       // No detonation (dud)
    HE_Hit_Small = 7,              // High explosive (small)
    HE_Hit_Medium = 8,             // High explosive (medium)
    HE_Hit_Large = 9,              // High explosive (large)
    ArmorPiercingHit = 10,         // Armor piercing round
    DirtBlastSmall = 11,           // Dirt blast (small)
    DirtBlastMedium = 12,          // Dirt blast (medium)
    DirtBlastLarge = 13,           // Dirt blast (large)
    WaterBlastSmall = 14,          // Water blast (small)
    WaterBlastMedium = 15,         // Water blast (medium)
    WaterBlastLarge = 16,          // Water blast (large)
    AirHit = 17,                   // Hit in air
    BuildingHitSmall = 18,         // Building impact (small)
    BuildingHitMedium = 19,        // Building impact (medium)
    BuildingHitLarge = 20,         // Building impact (large)
    MineClearingLineCharge = 21,   // Mine clearing charge
    EnvironmentObjectImpact = 22,  // Environmental object hit
    EnvironmentObjectProximateDetonation = 23,  // Env object near-miss
    WaterImpact = 24,              // Water surface impact
    AirBurst = 25,                 // Airburst detonation
    KillWithFragmentType1 = 26,    // Kill with fragmentation (type 1)
    KillWithFragmentType2 = 27,    // Kill with fragmentation (type 2)
    KillWithFragmentType3 = 28,    // Kill with fragmentation (type 3)
    KillWithoutFragment = 29,      // Kill without fragmentation
    Miss = 30                       // Complete miss
};
```

### Creation and Serialization

```cpp
#include <jaguar/federation/dis_fire_pdu.h>

using namespace jaguar::federation;

DetonationPDUHandler handler;

// Create Detonation PDU
DetonationPDU detonation = handler.create_detonation_pdu(
    EntityIdentifier(1, 1, 100),                        // Firing entity
    EntityIdentifier(1, 1, 200),                        // Target entity
    EntityIdentifier(1, 1, 1000),                       // Munition ID
    EventIdentifier(1, 1, 1),                          // Event ID (matches Fire)
    Vec3{-2475000.0, -4479000.0, 3557000.0},          // Detonation location
    Vec3{-85.0, 15.0, -40.0},                          // Velocity at impact
    DetonationResult::EntityImpact                      // Direct hit
);

// Set burst descriptor (must match Fire PDU)
handler.set_burst_descriptor(
    detonation,
    EntityType::create_munition(225, 1, 2, 3),
    1,      // Warhead (High Explosive)
    2,      // Fuse type
    1,      // Quantity
    0       // Rate
);

// Set impact location relative to target
handler.set_location_in_entity(detonation, Vec3{2.5, -1.3, 0.8});

// Add articulation parameter (e.g., turret destroyed)
ArticulationParameter damage_param;
damage_param.parameter_type = 1;      // Example type code
damage_param.parameter_value = 1.0;   // Damaged/destroyed
handler.add_articulation_parameter(detonation, damage_param);

// Serialize (104 + 16*num_params bytes)
std::vector<UInt8> buffer(DetonationPDUHandler::get_detonation_pdu_size(1));
SizeT bytes_written = handler.serialize(detonation, buffer.data(), buffer.size());

// Deserialize
std::optional<DetonationPDU> received = handler.deserialize(buffer.data(), bytes_written);
if (received) {
    std::cout << "Impact result: "
              << detonation_result_to_string(received->detonation_result) << std::endl;
    std::cout << "Articulation params: " << static_cast<int>(received->num_articulation_params)
              << std::endl;
}
```

### Utility Functions

```cpp
// Match Fire to Detonation PDU
FirePDU fire = /* ... */;
DetonationPDU detonation = /* ... */;

if (match_fire_to_detonation(fire, detonation)) {
    std::cout << "Detonation matches fire event" << std::endl;
}

// Get detonation result description
const char* result_str = detonation_result_to_string(DetonationResult::EntityImpact);
std::cout << "Result: " << result_str << std::endl;

// Check hit type
if (is_entity_hit(detonation.detonation_result)) {
    std::cout << "Entity impact confirmed" << std::endl;
}

if (is_ground_impact(detonation.detonation_result)) {
    std::cout << "Ground impact confirmed" << std::endl;
}

// Calculate damage assessment
Real damage_score = calculate_damage_score(detonation.detonation_result);
std::cout << "Damage score: " << damage_score << " (0.0 = no damage, 1.0 = maximum)" << std::endl;
```

## HLA RTI Integration

High Level Architecture (HLA) integration is available with conditional compilation.

### Configuration

HLA RTI support is optional and enabled via CMake:

```bash
cmake .. -DJAGUAR_ENABLE_HLA=ON
```

### RTI Configuration

```cpp
#include <jaguar/federation/hla_rti.h>

using namespace jaguar::federation::hla;

// Create HLA configuration
HLAConfiguration config;
config.federation_name = "TrainingExercise";
config.federate_name = "JaguarSim01";
config.federate_type = "Simulator";

// FOM (Federated Object Model) modules
config.fom_module_paths = {"RPR_FOM_v2.xml"};

// RTI connection settings
config.connection.rti_host = "rti.example.com";
config.connection.rti_port = 8989;
config.connection.rti_type = "HLA_EVOLVER";  // or Portico, MAK, Pitch
config.connection.connect_timeout = std::chrono::milliseconds(30000);
config.connection.operation_timeout = std::chrono::milliseconds(10000);

// Time management
config.time_management.enable_time_regulation = true;
config.time_management.enable_time_constrained = true;
config.time_management.lookahead = LogicalTimeInterval(0.1);  // 100ms

// Publication/subscription
config.publication.published_object_classes.insert(aircraft_class_handle);
config.subscription.subscribed_object_classes.insert(aircraft_class_handle);

// Create RTI Ambassador
auto rti = create_rti_ambassador(config);
```

### Federation Management

```cpp
// Create federation (fails if already exists)
HLAResult result = rti->create_federation_execution(
    "TrainingExercise",
    {"RPR_FOM_v2.xml"}
);

if (result == HLAResult::FederationExecutionAlreadyExists) {
    // Federation already exists - continue to join
}

// Join federation
result = rti->join_federation_execution(
    "JaguarSim01",
    "Simulator",
    "TrainingExercise"
);

if (result != HLAResult::Success) {
    std::cerr << "Failed to join: " << hla_result_to_string(result) << std::endl;
}

// Synchronization point
rti->register_federation_synchronization_point("ReadyToRun", {});
rti->synchronization_point_achieved("ReadyToRun");

// Eventually: Resign and destroy
rti->resign_federation_execution(ResignAction::DeleteObjectsThenDivest);
rti->destroy_federation_execution("TrainingExercise");
```

### Object Management

```cpp
// Get class handles from FOM
ObjectClassHandle aircraft_class;
rti->get_object_class_handle("HLAobjectRoot.BaseEntity.PhysicalEntity.Platform.Aircraft",
                             aircraft_class);

AttributeHandle position_attr, velocity_attr;
rti->get_attribute_handle(aircraft_class, "SpatialStatic", position_attr);
rti->get_attribute_handle(aircraft_class, "VelocityVector", velocity_attr);

// Publish attributes
rti->publish_object_class_attributes(aircraft_class, {position_attr, velocity_attr});

// Register object instance
ObjectInstanceHandle my_aircraft;
rti->register_object_instance(aircraft_class, my_aircraft, "Viper01");

// Update attributes
AttributeValueSet attributes;
std::vector<UInt8> pos_data = encode_position({37.7749, -122.4194, 10000.0});
attributes.set_attribute(position_attr, pos_data);

rti->update_attribute_values(my_aircraft, attributes, {}, LogicalTime(current_time));

// Subscribe to other objects
rti->subscribe_object_class_attributes(aircraft_class, {position_attr, velocity_attr});

// Delete object
rti->delete_object_instance(my_aircraft, {}, LogicalTime(current_time));
```

### Interaction Management

```cpp
// Get interaction handles
InteractionClassHandle fire_interaction;
rti->get_interaction_class_handle("HLAinteractionRoot.WeaponFire", fire_interaction);

ParameterHandle shooter_param, target_param;
rti->get_parameter_handle(fire_interaction, "FiringObjectIdentifier", shooter_param);
rti->get_parameter_handle(fire_interaction, "TargetObjectIdentifier", target_param);

// Publish and subscribe
rti->publish_interaction_class(fire_interaction);
rti->subscribe_interaction_class(fire_interaction);

// Send interaction
ParameterValueSet params;
params.set_parameter(shooter_param, encode_entity_id(shooter_id));
params.set_parameter(target_param, encode_entity_id(target_id));

rti->send_interaction(fire_interaction, params, {}, LogicalTime(current_time));
```

### Time Management

```cpp
// Enable time regulation (this federate advances federation time)
rti->enable_time_regulation(LogicalTimeInterval(0.1));

// Enable time constrained (this federate receives time-stamped updates)
rti->enable_time_constrained();

// Advance time
rti->time_advance_request(LogicalTime(target_time));

// Query greatest available logical time
LogicalTime galt;
rti->query_galt(galt);
std::cout << "GALT: " << galt.value() << std::endl;

// Query time of next interaction
LogicalTime lits;
rti->query_lits(lits);
std::cout << "LITS: " << lits.value() << std::endl;

// Modify lookahead
rti->modify_lookahead(LogicalTimeInterval(0.2));
```

### Federate Ambassador Callbacks

```cpp
// Implement IFederateAmbassador interface
class MyFederateAmbassador : public IFederateAmbassador {
public:
    // Object discovery
    void discover_object_instance(
        ObjectInstanceHandle object,
        ObjectClassHandle object_class,
        std::string_view instance_name) override {
        std::cout << "Discovered object: " << instance_name << std::endl;
    }

    // Attribute reflection (receive order)
    void reflect_attribute_values(
        ObjectInstanceHandle object,
        const AttributeValueSet& attributes,
        const std::vector<UInt8>& tag) override {
        // Process updates for known objects
    }

    // Attribute reflection (timestamp order)
    void reflect_attribute_values_with_time(
        ObjectInstanceHandle object,
        const AttributeValueSet& attributes,
        const std::vector<UInt8>& tag,
        const LogicalTime& time) override {
        // Timestamped updates (useful for dead reckoning)
    }

    // Object removal
    void remove_object_instance(
        ObjectInstanceHandle object,
        const std::vector<UInt8>& tag) override {
        std::cout << "Object removed" << std::endl;
    }

    // Interaction reception
    void receive_interaction(
        InteractionClassHandle interaction,
        const ParameterValueSet& parameters,
        const std::vector<UInt8>& tag) override {
        // Process interaction
    }

    // Time advance grant
    void time_advance_grant(const LogicalTime& time) override {
        current_simulation_time = time.value();
    }

    // All other required callbacks...
};

// Create and use ambassador
auto ambassador = create_federate_ambassador();
```

## Network Byte Order (Big-Endian) Conversion

DIS protocol requires big-endian (network byte order) for all multi-byte values to ensure cross-platform compatibility.

### Safe Byte Order Conversion

```cpp
#include <jaguar/federation/dis_socket.h>

using namespace jaguar::federation;

// 16-bit conversions
UInt16 host_value16 = 0x1234;
UInt16 network_value16 = htons_safe(host_value16);  // Host to network
UInt16 back_to_host16 = ntohs_safe(network_value16);  // Network to host

// 32-bit conversions
UInt32 host_value32 = 0x12345678;
UInt32 network_value32 = htonl_safe(host_value32);
UInt32 back_to_host32 = ntohl_safe(network_value32);

// Equivalent to standard htons/htonl but always safe
// (standard functions may not be available on all platforms)
```

### Floating-Point Conversion

For IEEE 754 single-precision (float) and double-precision (double) values:

```cpp
// Floats and doubles are converted using their binary representation
UInt32 float_bits = *reinterpret_cast<const UInt32*>(&float_value);
UInt32 network_bits = htonl_safe(float_bits);

// Reconstruct
float reconstructed = *reinterpret_cast<const float*>(&network_bits);
```

### Codec Usage

The DIS codec automatically handles byte order conversion:

```cpp
// Encoding (host byte order -> network byte order)
auto codec = create_dis_codec();
std::vector<UInt8> buffer(DIS_MAX_PDU_SIZE);
SizeT bytes = codec->encode_entity_state(entity_state_pdu, buffer.data());

// Decoding (network byte order -> host byte order)
std::optional<EntityStatePDU> decoded = codec->decode_entity_state(buffer.data(), bytes);
```

## Network Transport Layer

The federation module includes a unified network transport layer supporting UDP and TCP.

### TCP Client/Server

```cpp
// Server
auto server = create_tcp_server();
TCPServerConfig server_config;
server_config.bind_address = SocketAddress("0.0.0.0", 8989);
server->listen(server_config);

auto conn = server->accept(std::chrono::milliseconds(5000));
if (conn && conn->is_connected()) {
    UInt8 buffer[1024];
    Int64 received = conn->receive(buffer, sizeof(buffer));
    // Process data
}

// Client
auto client = create_tcp_connection();
TCPConnectionConfig client_config;
client_config.remote_address = SocketAddress("192.168.1.100", 8989);
client->connect(client_config);
client->send_all(data.data(), data.size());
```

### Message Framing

For TCP streams, use `MessageFramer` for length-prefixed messages:

```cpp
MessageFramer framer;

// Sender side
std::vector<UInt8> message = /* data */;
std::vector<UInt8> framed(message.size() + 4);
framer.frame_message(message.data(), message.size(), framed.data());
connection->send_all(framed.data(), framed.size());

// Receiver side
std::vector<std::vector<UInt8>> messages;
framer.process_received_data(buffer, bytes_received, messages);
for (const auto& msg : messages) {
    // Process complete message
}
```

## Testing

Federation module includes comprehensive unit tests:

```bash
# Build tests
cmake .. -DJAGUAR_BUILD_TESTS=ON -DJAGUAR_ENABLE_DIS=ON -DJAGUAR_ENABLE_HLA=ON
cmake --build . --target jaguar_dis_protocol_tests jaguar_hla_rti_tests jaguar_network_transport_tests

# Run tests
./jaguar_dis_protocol_tests        # 80 tests
./jaguar_hla_rti_tests             # 86 tests
./jaguar_network_transport_tests   # 46 tests
# Total: 212 tests
```

## Standards References

- **IEEE 1278.1-2012**: DIS Protocol Standard
- **IEEE 1516-2010**: HLA Framework and Rules
- **IEEE 1516.1-2010**: HLA Federate Interface Specification
- **IEEE 1516.2-2010**: HLA Object Model Template (OMT)
- **SISO-REF-010**: DIS Enumerations
- **SISO-REF-015**: Dead Reckoning Algorithms
- **RPR FOM v2.0**: Real-time Platform Reference FOM

## Future Enhancements

- [x] Network Transport layer (UDP/TCP) - Implemented
- [ ] TENA gateway integration
- [ ] Link 16/VMF tactical data link support
- [ ] WebLVC gateway for browser-based federation
- [ ] Docker-based RTI deployment
- [ ] Prometheus metrics for federation monitoring
