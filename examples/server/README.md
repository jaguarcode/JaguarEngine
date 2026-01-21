# JaguarEngine WebSocket Server

Real-time simulation server that bridges JaguarEngine with web clients via WebSocket.

## Features

- Real-time entity state broadcasting at 20 Hz
- Simulation control (start/pause/stop/reset)
- Entity spawning and destruction
- Time scale adjustment
- Flight/vehicle/ship control commands
- Multi-client support

## Requirements

- JaguarEngine library
- libwebsockets
- CMake 3.16+
- C++17 compiler

## Building

```bash
# From JaguarEngine build directory
mkdir -p build && cd build
cmake .. -DBUILD_EXAMPLES=ON
cmake --build . --target jaguar_server
```

Or standalone:

```bash
cd examples/server
mkdir build && cd build
cmake .. -DJaguarEngine_DIR=/path/to/jaguar/lib/cmake/JaguarEngine
make
```

## Usage

```bash
# Default settings (port 8081, real-time)
./jaguar_server

# Custom port
./jaguar_server --port 8082

# Fast-forward simulation (5x speed)
./jaguar_server --time-scale 5.0

# Higher tick rate (50 Hz)
./jaguar_server --tick-rate 50

# Show help
./jaguar_server --help
```

## WebSocket Protocol

Connect to: `ws://localhost:8081`

### Message Format

All messages are JSON with format:
```json
{
  "type": "message_type",
  "timestamp": 1234567890,
  "data": { ... }
}
```

### Message Types

#### World State (Server → Client)

Broadcast at 20 Hz with all entity states:

```json
{
  "type": "world_state",
  "timestamp": 1234567890,
  "data": {
    "entities": [...],
    "stats": {
      "simulationTime": 123.45,
      "wallClockTime": 123.45,
      "deltaTime": 0.05,
      "realtimeRatio": 1.0,
      "frameRate": 60.0,
      "physicsTime": 2.5,
      "totalEntities": 8,
      "activeEntities": 8,
      "entitiesByDomain": {
        "air": 3,
        "land": 2,
        "sea": 2,
        "space": 1
      }
    },
    "status": "running"
  }
}
```

#### Command (Client → Server)

```json
{
  "type": "command",
  "data": {
    "command": "start|pause|stop|reset|set_time_scale",
    "params": { "scale": 2.0 }
  }
}
```

#### Entity Spawn (Client → Server)

```json
{
  "type": "entity_spawn",
  "data": {
    "domain": "air",
    "name": "F-16 Fighter",
    "position": {
      "latitude": 37.5665,
      "longitude": 126.978,
      "altitude": 5000
    }
  }
}
```

#### Entity Destroy (Client → Server)

```json
{
  "type": "entity_destroy",
  "data": {
    "entityId": "entity_5"
  }
}
```

#### Flight Controls (Client → Server)

```json
{
  "type": "command",
  "data": {
    "command": "set_flight_controls",
    "entityId": "entity_1",
    "params": {
      "elevator": 0.1,
      "aileron": -0.05,
      "rudder": 0.0,
      "throttle": 0.8
    }
  }
}
```

## Entity Data Format

Each entity includes:

```json
{
  "id": "entity_1",
  "name": "F-16 Eagle #1",
  "domain": "air",
  "kind": "platform",
  "isActive": true,
  "health": 100,
  "damage": 0,
  "position": {
    "latitude": 37.5665,
    "longitude": 126.978,
    "altitude": 10000
  },
  "velocity": {
    "north": 150.0,
    "east": 50.0,
    "down": -5.0
  },
  "orientation": {
    "roll": 0,
    "pitch": 0,
    "yaw": 0
  }
}
```

Domain-specific fields are included based on entity type (air, land, sea, space).

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                   Web Browser                            │
│  ┌─────────────────────────────────────────────────┐   │
│  │          JaguarEngine Frontend (React)           │   │
│  │    - 3D Globe (Cesium)                          │   │
│  │    - Entity List                                 │   │
│  │    - Telemetry Panel                            │   │
│  │    - Controls                                    │   │
│  └─────────────────────────────────────────────────┘   │
└────────────────────────┬────────────────────────────────┘
                         │ WebSocket
                         ▼
┌─────────────────────────────────────────────────────────┐
│              WebSocket Server (C++)                      │
│  ┌─────────────────────────────────────────────────┐   │
│  │  - Message handling                              │   │
│  │  - JSON serialization                           │   │
│  │  - Client management                            │   │
│  │  - State broadcasting                           │   │
│  └─────────────────────────────────────────────────┘   │
│                         │                               │
│                         ▼                               │
│  ┌─────────────────────────────────────────────────┐   │
│  │            JaguarEngine Core                     │   │
│  │  - Physics simulation                           │   │
│  │  - Entity management                            │   │
│  │  - Domain models (air/land/sea/space)          │   │
│  │  - Time management                              │   │
│  └─────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────┘
```

## License

Part of JaguarEngine project.
