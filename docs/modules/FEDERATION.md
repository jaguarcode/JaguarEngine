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

## Network Transport Layer

The federation module includes a unified network transport layer supporting UDP and TCP.

### UDP Socket

```cpp
#include <jaguar/federation/network_transport.h>

using namespace jaguar::federation::network;

// Create UDP socket for DIS
auto socket = create_udp_socket();

// Configure for DIS
auto config = UDPSocketConfig::dis_default(3000);
socket->initialize(config);

// Join multicast group
socket->join_multicast_group(SocketAddress("239.1.2.3", 3000));

// Send data
std::vector<UInt8> message = /* PDU data */;
socket->send_to(message.data(), message.size(), SocketAddress("239.1.2.3", 3000));

// Receive data
UInt8 buffer[8192];
ReceiveResult result;
if (socket->receive_from(buffer, sizeof(buffer), result) == NetworkResult::Success) {
    // Process result.bytes_received bytes from result.source
}

socket->leave_multicast_group(SocketAddress("239.1.2.3", 3000));
socket->close();
```

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
