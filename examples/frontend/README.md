# JaguarEngine Test Frontend

A React-based frontend application for testing and visualizing JaguarEngine multi-domain simulation features.

## Features

- **3D Visualization**: Cesium-based globe with entity rendering across Air, Land, Sea, and Space domains
- **Real-time Telemetry**: Performance metrics, timing statistics, and entity counts
- **Entity Management**: Spawn, select, and control entities with domain-specific interfaces
- **Domain Controls**: Flight controls for aircraft, vehicle controls for ground vehicles, helm controls for ships
- **WebSocket Integration**: Real-time communication with the JaguarEngine simulation server

## Prerequisites

- Node.js 18+ and npm/pnpm
- JaguarEngine simulation server running on `ws://localhost:8765`
- (Optional) Cesium Ion token for 3D photorealistic tiles

## Quick Start

```bash
# Install dependencies
npm install

# Copy environment configuration
cp .env.example .env

# (Optional) Add your Cesium Ion token to .env
# VITE_CESIUM_TOKEN=your_token_here

# Start development server
npm run dev
```

Open http://localhost:3000 in your browser.

## Project Structure

```
src/
├── components/
│   ├── viewer/          # Cesium 3D visualization
│   │   ├── CesiumViewer.tsx
│   │   └── EntityLayer.tsx
│   ├── monitor/         # Telemetry and entity monitoring
│   │   ├── TelemetryPanel.tsx
│   │   ├── EntityList.tsx
│   │   └── EntityDetailPanel.tsx
│   ├── controls/        # Simulation controls
│   │   ├── SimControls.tsx
│   │   └── EntitySpawner.tsx
│   ├── editor/          # Domain-specific controls
│   │   └── DomainTestPanel.tsx
│   └── common/          # Shared components
│       ├── ConnectionStatus.tsx
│       ├── HelpModal.tsx
│       └── PerformanceOverlay.tsx
├── stores/              # Zustand state management
│   ├── simulationStore.ts
│   ├── entityStore.ts
│   └── uiStore.ts
├── hooks/               # Custom React hooks
│   ├── useWebSocket.ts
│   └── useKeyboardShortcuts.ts
├── services/            # External services
│   └── websocket.ts
├── types/               # TypeScript type definitions
│   └── index.ts
├── App.tsx              # Main application component
└── main.tsx             # Entry point
```

## Keyboard Shortcuts

| Key | Action |
|-----|--------|
| `Space` | Play/Pause simulation |
| `R` | Reset simulation |
| `P` | Toggle performance overlay |
| `?` | Show help modal |
| `[` | Toggle left panel |
| `]` | Toggle right panel |

## WebSocket Protocol

The frontend communicates with the simulation server using JSON messages:

### Outbound Messages

```typescript
// Start simulation
{ type: 'command', data: { command: 'start' } }

// Spawn entity
{ type: 'entity_spawn', data: { domain, kind, name, position, template } }

// Set flight controls
{ type: 'command', data: { command: 'set_flight_controls', entityId, params: { elevator, aileron, rudder, throttle } } }
```

### Inbound Messages

```typescript
// World state update
{ type: 'world_state', data: { entities: [...], stats: {...}, status: 'running' } }

// Entity update
{ type: 'entity_update', data: { id, position, velocity, ... } }

// Telemetry
{ type: 'telemetry', data: { stats: { frameRate, physicsTime, ... } } }
```

## Domain Support

### Air Domain
- Aircraft control surfaces (elevator, aileron, rudder, throttle)
- Autopilot modes (altitude hold, heading hold, speed hold)
- Flight parameters (airspeed, Mach, AoA, G-load)

### Land Domain
- Vehicle controls (throttle, steering, brake)
- Engine parameters (RPM, gear)
- Terrain interaction (ground contact, sinkage)

### Sea Domain
- Helm controls (rudder, throttle)
- Ship motion (roll, pitch, heave)
- Sea state information

### Space Domain
- Orbital elements display
- Orbital maneuver commands
- Ground track visualization

## Environment Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `VITE_WS_URL` | WebSocket server URL | `ws://localhost:8765` |
| `VITE_CESIUM_TOKEN` | Cesium Ion access token | (none) |

## Building for Production

```bash
npm run build
```

The build output will be in the `dist/` directory.

## Technology Stack

- **React 18** - UI framework
- **TypeScript 5** - Type safety
- **Vite 5** - Build tool
- **Cesium + Resium** - 3D globe visualization
- **Zustand** - State management
- **Tailwind CSS** - Styling
- **Recharts** - Data visualization

## License

Part of the JaguarEngine project. See main repository for license information.
