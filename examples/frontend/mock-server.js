/**
 * Mock WebSocket server for testing JaguarEngine frontend
 * Run with: node mock-server.js
 */

import { WebSocketServer } from 'ws';

const PORT = 8081;
const wss = new WebSocketServer({ port: PORT });

console.log(`Mock JaguarEngine server running on ws://localhost:${PORT}`);

// Simulation state
let simulationStatus = 'idle';
let simulationTime = 0;
let timeScale = 1.0;
let entityIdCounter = 0;

// Mock entities
const entities = new Map();

// Generate random entity
function createMockEntity(domain, position) {
  const id = `entity_${++entityIdCounter}`;
  const names = {
    air: ['F-16 Eagle', 'C-130 Hercules', 'UH-60 Blackhawk', 'Predator UAV', 'AH-64 Apache'],
    land: ['M1 Abrams', 'HMMWV', 'Bradley IFV', 'M777 Howitzer', 'Patriot Launcher'],
    sea: ['DDG-51 Burke', 'LCS Freedom', 'Virginia SSN', 'Container Ship', 'Cargo Vessel'],
    space: ['GPS-IIR', 'ISS', 'Starlink-1', 'Hubble', 'GOES-16'],
  };

  const name = names[domain][Math.floor(Math.random() * names[domain].length)] + ` #${entityIdCounter}`;

  return {
    id,
    identifier: {
      siteId: 1,
      applicationId: 1,
      entityNumber: entityIdCounter,
    },
    name,
    domain,
    kind: 'platform',
    position: position || {
      latitude: 37.5665 + (Math.random() - 0.5) * 2,
      longitude: 126.9780 + (Math.random() - 0.5) * 2,
      altitude: domain === 'air' ? 5000 + Math.random() * 10000 :
                domain === 'space' ? 400000 + Math.random() * 100000 :
                domain === 'sea' ? 0 : 100,
    },
    velocity: {
      north: (Math.random() - 0.5) * 200,
      east: (Math.random() - 0.5) * 200,
      down: domain === 'air' ? (Math.random() - 0.5) * 20 : 0,
    },
    orientation: {
      roll: 0,
      pitch: 0,
      yaw: Math.random() * Math.PI * 2,
    },
    angularVelocity: { roll: 0, pitch: 0, yaw: 0 },
    health: 80 + Math.random() * 20,
    damage: Math.random() * 20,
    isActive: true,
    properties: {},
    lastUpdate: Date.now(),
    createdAt: Date.now(),

    // Domain-specific properties
    ...(domain === 'air' && {
      airspeed: 150 + Math.random() * 200,
      machNumber: 0.3 + Math.random() * 0.5,
      altitude: 5000 + Math.random() * 10000,
      verticalSpeed: (Math.random() - 0.5) * 20,
      heading: Math.random() * 360,
      elevator: 0,
      aileron: 0,
      rudder: 0,
      throttle: 0.6 + Math.random() * 0.3,
      fuel: 50 + Math.random() * 50,
      gearDown: false,
      flapsPosition: 0,
      aoa: Math.random() * 10,
      sideslip: (Math.random() - 0.5) * 5,
      gForce: 1 + Math.random() * 0.5,
    }),
    ...(domain === 'land' && {
      speed: Math.random() * 30,
      heading: Math.random() * 360,
      throttle: Math.random(),
      steering: 0,
      brake: 0,
      fuel: 50 + Math.random() * 50,
      engineRPM: 1000 + Math.random() * 2000,
      gear: Math.floor(Math.random() * 5) + 1,
      groundContact: true,
      sinkage: Math.random() * 0.1,
    }),
    ...(domain === 'sea' && {
      speed: Math.random() * 15,
      heading: Math.random() * 360,
      roll: (Math.random() - 0.5) * 10,
      pitch: (Math.random() - 0.5) * 5,
      heave: (Math.random() - 0.5) * 2,
      rudder: 0,
      throttle: 0.5 + Math.random() * 0.5,
      draft: 5 + Math.random() * 10,
      displacement: 5000 + Math.random() * 10000,
      waveHeight: 1 + Math.random() * 3,
      seaState: Math.floor(Math.random() * 5) + 1,
    }),
    ...(domain === 'space' && {
      semiMajorAxis: 6778 + Math.random() * 1000,
      eccentricity: Math.random() * 0.1,
      inclination: Math.random() * 90,
      raan: Math.random() * 360,
      argOfPerigee: Math.random() * 360,
      trueAnomaly: Math.random() * 360,
      altitude: 400 + Math.random() * 200,
      velocity: 7.5 + Math.random() * 0.5,
      period: 90 + Math.random() * 10,
      power: 80 + Math.random() * 20,
      fuel: 50 + Math.random() * 50,
    }),
  };
}

// Create initial entities
function initializeEntities() {
  entities.clear();

  // Create some entities for each domain
  for (let i = 0; i < 3; i++) {
    const air = createMockEntity('air');
    entities.set(air.id, air);
  }
  for (let i = 0; i < 2; i++) {
    const land = createMockEntity('land');
    entities.set(land.id, land);
  }
  for (let i = 0; i < 2; i++) {
    const sea = createMockEntity('sea');
    entities.set(sea.id, sea);
  }
  const space = createMockEntity('space');
  entities.set(space.id, space);

  console.log(`Initialized ${entities.size} entities`);
}

// Update entities (simulate movement)
function updateEntities(dt) {
  entities.forEach((entity) => {
    if (!entity.isActive) return;

    // Update position based on velocity
    const R = 6371000; // Earth radius in meters
    const lat = entity.position.latitude * Math.PI / 180;

    entity.position.latitude += (entity.velocity.north * dt) / R * (180 / Math.PI);
    entity.position.longitude += (entity.velocity.east * dt) / (R * Math.cos(lat)) * (180 / Math.PI);
    entity.position.altitude -= entity.velocity.down * dt;

    // Update heading based on velocity
    if (entity.velocity.north !== 0 || entity.velocity.east !== 0) {
      entity.orientation.yaw = Math.atan2(entity.velocity.east, entity.velocity.north);
    }

    entity.lastUpdate = Date.now();

    // Domain-specific updates
    if (entity.domain === 'air' && entity.heading !== undefined) {
      entity.heading = (entity.orientation.yaw * 180 / Math.PI + 360) % 360;
      entity.altitude = entity.position.altitude;
    }
  });
}

// Get simulation stats
function getStats() {
  const domainCounts = { air: 0, land: 0, sea: 0, space: 0 };
  entities.forEach((e) => domainCounts[e.domain]++);

  return {
    simulationTime,
    wallClockTime: simulationTime,
    deltaTime: 0.016 * timeScale,
    realtimeRatio: timeScale,
    frameRate: 60,
    physicsTime: 2 + Math.random() * 3,
    renderTime: 5 + Math.random() * 5,
    totalEntities: entities.size,
    entitiesByDomain: domainCounts,
    activeEntities: entities.size,
    collisionChecks: entities.size * (entities.size - 1) / 2,
    activeCollisions: 0,
  };
}

// Send world state to all clients
function broadcastWorldState() {
  const message = JSON.stringify({
    type: 'world_state',
    timestamp: Date.now(),
    data: {
      entities: Array.from(entities.values()),
      stats: getStats(),
      status: simulationStatus,
    },
  });

  wss.clients.forEach((client) => {
    if (client.readyState === 1) { // WebSocket.OPEN
      client.send(message);
    }
  });
}

// Simulation loop
let simulationInterval = null;

function startSimulation() {
  if (simulationInterval) return;

  simulationStatus = 'running';
  console.log('Simulation started');

  simulationInterval = setInterval(() => {
    const dt = 0.05 * timeScale; // 50ms tick * time scale
    simulationTime += dt;
    updateEntities(dt);
    broadcastWorldState();
  }, 50);
}

function pauseSimulation() {
  if (!simulationInterval) return;

  clearInterval(simulationInterval);
  simulationInterval = null;
  simulationStatus = 'paused';
  console.log('Simulation paused');
  broadcastWorldState();
}

function stopSimulation() {
  pauseSimulation();
  simulationStatus = 'stopped';
  console.log('Simulation stopped');
  broadcastWorldState();
}

function resetSimulation() {
  stopSimulation();
  simulationTime = 0;
  simulationStatus = 'idle';
  initializeEntities();
  console.log('Simulation reset');
  broadcastWorldState();
}

// Handle WebSocket connections
wss.on('connection', (ws) => {
  console.log('Client connected');

  // Send initial state
  ws.send(JSON.stringify({
    type: 'world_state',
    timestamp: Date.now(),
    data: {
      entities: Array.from(entities.values()),
      stats: getStats(),
      status: simulationStatus,
    },
  }));

  // Handle messages from client
  ws.on('message', (data) => {
    try {
      const message = JSON.parse(data.toString());
      console.log('Received:', message.type, message.data?.command || '');

      switch (message.type) {
        case 'command':
          handleCommand(message.data, ws);
          break;
        case 'entity_spawn':
          handleSpawn(message.data, ws);
          break;
        case 'entity_destroy':
          handleDestroy(message.data, ws);
          break;
      }
    } catch (err) {
      console.error('Error handling message:', err);
    }
  });

  ws.on('close', () => {
    console.log('Client disconnected');
  });
});

function handleCommand(data, ws) {
  switch (data.command) {
    case 'start':
      startSimulation();
      break;
    case 'pause':
      pauseSimulation();
      break;
    case 'stop':
      stopSimulation();
      break;
    case 'reset':
      resetSimulation();
      break;
    case 'set_time_scale':
      timeScale = data.params?.scale || 1.0;
      console.log('Time scale:', timeScale);
      break;
    case 'set_flight_controls':
      const aircraft = entities.get(data.entityId);
      if (aircraft && aircraft.domain === 'air') {
        Object.assign(aircraft, data.params);
        console.log('Updated flight controls for', aircraft.name);
      }
      break;
    case 'set_vehicle_controls':
      const vehicle = entities.get(data.entityId);
      if (vehicle && vehicle.domain === 'land') {
        Object.assign(vehicle, data.params);
        console.log('Updated vehicle controls for', vehicle.name);
      }
      break;
    case 'set_ship_controls':
      const ship = entities.get(data.entityId);
      if (ship && ship.domain === 'sea') {
        Object.assign(ship, data.params);
        console.log('Updated ship controls for', ship.name);
      }
      break;
  }
}

function handleSpawn(data, ws) {
  const entity = createMockEntity(data.domain, data.position);
  if (data.name) entity.name = data.name;
  entities.set(entity.id, entity);

  console.log('Spawned:', entity.name, 'at', entity.position);

  ws.send(JSON.stringify({
    type: 'entity_spawn',
    timestamp: Date.now(),
    data: entity,
    requestId: data.requestId,
  }));

  broadcastWorldState();
}

function handleDestroy(data, ws) {
  const entity = entities.get(data.entityId);
  if (entity) {
    entities.delete(data.entityId);
    console.log('Destroyed:', entity.name);
    broadcastWorldState();
  }
}

// Initialize
initializeEntities();
console.log('Ready for connections...');
