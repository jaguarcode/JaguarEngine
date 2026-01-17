// JaguarEngine Frontend Types

//==============================================================================
// Domain Types
//==============================================================================

export type Domain = 'air' | 'land' | 'sea' | 'space';

export type EntityKind =
  | 'platform'      // Vehicles, vessels, spacecraft
  | 'munition'      // Missiles, bombs, torpedoes
  | 'sensor'        // Radars, cameras
  | 'emitter'       // Radio, jamming
  | 'cultural'      // Buildings, structures
  | 'environmental';// Weather, terrain features

export type SimulationStatus = 'idle' | 'running' | 'paused' | 'stopped' | 'error';

//==============================================================================
// Coordinate Types
//==============================================================================

export interface GeodeticPosition {
  latitude: number;   // degrees
  longitude: number;  // degrees
  altitude: number;   // meters MSL
}

export interface ECEFPosition {
  x: number;  // meters
  y: number;  // meters
  z: number;  // meters
}

export interface NEDVelocity {
  north: number;  // m/s
  east: number;   // m/s
  down: number;   // m/s
}

export interface EulerAngles {
  roll: number;   // radians
  pitch: number;  // radians
  yaw: number;    // radians
}

export interface Quaternion {
  w: number;
  x: number;
  y: number;
  z: number;
}

//==============================================================================
// Entity Types
//==============================================================================

export interface EntityIdentifier {
  siteId: number;
  applicationId: number;
  entityNumber: number;
}

export interface EntityState {
  id: string;
  identifier: EntityIdentifier;
  name: string;
  domain: Domain;
  kind: EntityKind;

  // Transform
  position: GeodeticPosition;
  velocity: NEDVelocity;
  orientation: EulerAngles;
  angularVelocity: EulerAngles;

  // Status
  health: number;       // 0-100%
  damage: number;       // 0-100%
  isActive: boolean;

  // Domain-specific properties
  properties: Record<string, number | string | boolean>;

  // Timestamps
  lastUpdate: number;
  createdAt: number;
}

//==============================================================================
// Air Domain Types
//==============================================================================

export interface AircraftState extends EntityState {
  domain: 'air';

  // Flight parameters
  airspeed: number;         // m/s TAS
  machNumber: number;
  altitude: number;         // meters MSL
  verticalSpeed: number;    // m/s
  heading: number;          // degrees

  // Control surfaces
  elevator: number;         // -1 to 1
  aileron: number;          // -1 to 1
  rudder: number;           // -1 to 1
  throttle: number;         // 0 to 1

  // Systems
  fuel: number;             // percentage
  gearDown: boolean;
  flapsPosition: number;    // degrees

  // Performance
  aoa: number;              // angle of attack (degrees)
  sideslip: number;         // beta (degrees)
  gForce: number;           // G-load
}

export interface MissileState extends EntityState {
  domain: 'air';
  kind: 'munition';

  // Guidance
  guidanceMode: 'active' | 'semi-active' | 'passive' | 'inertial' | 'gps';
  targetId?: string;
  seekerLocked: boolean;

  // Performance
  machNumber: number;
  fuel: number;
  timeToImpact?: number;
}

//==============================================================================
// Land Domain Types
//==============================================================================

export interface GroundVehicleState extends EntityState {
  domain: 'land';

  // Movement
  speed: number;            // m/s
  heading: number;          // degrees

  // Controls
  throttle: number;         // -1 to 1
  steering: number;         // -1 to 1
  brake: number;            // 0 to 1

  // Status
  fuel: number;             // percentage
  engineRPM: number;
  gear: number;

  // Terrain interaction
  groundContact: boolean;
  sinkage: number;          // meters
}

//==============================================================================
// Sea Domain Types
//==============================================================================

export interface ShipState extends EntityState {
  domain: 'sea';

  // Navigation
  speed: number;            // m/s
  heading: number;          // degrees

  // Motion
  roll: number;             // degrees
  pitch: number;            // degrees
  heave: number;            // meters

  // Controls
  rudder: number;           // -1 to 1
  throttle: number;         // 0 to 1

  // Status
  draft: number;            // meters
  displacement: number;     // tonnes

  // Sea state
  waveHeight: number;       // meters
  seaState: number;         // 0-9 Douglas scale
}

//==============================================================================
// Space Domain Types
//==============================================================================

export interface SpacecraftState extends Omit<EntityState, 'velocity'> {
  domain: 'space';

  // Orbital elements
  semiMajorAxis: number;    // km
  eccentricity: number;
  inclination: number;      // degrees
  raan: number;             // degrees (Right Ascension of Ascending Node)
  argOfPerigee: number;     // degrees
  trueAnomaly: number;      // degrees

  // Derived
  altitude: number;         // km
  velocity: number;         // km/s (scalar orbital velocity)
  period: number;           // minutes

  // Status
  power: number;            // percentage
  fuel: number;             // percentage (for maneuvering)
}

//==============================================================================
// Simulation Types
//==============================================================================

export interface SimulationConfig {
  timeScale: number;        // 1.0 = real-time
  maxEntities: number;
  enableCollisions: boolean;
  enableDamage: boolean;

  // Domain-specific
  atmosphereModel: 'standard' | 'jbh08' | 'modtran';
  terrainEnabled: boolean;
  oceanEnabled: boolean;
  gravityModel: 'wgs84' | 'egm96' | 'spherical';
}

export interface SimulationStats {
  // Timing
  simulationTime: number;   // seconds
  wallClockTime: number;    // seconds
  deltaTime: number;        // seconds
  realtimeRatio: number;    // sim_time / wall_time

  // Performance
  frameRate: number;        // Hz
  physicsTime: number;      // ms per step
  renderTime: number;       // ms per frame

  // Entities
  totalEntities: number;
  entitiesByDomain: Record<Domain, number>;
  activeEntities: number;

  // Physics
  collisionChecks: number;
  activeCollisions: number;
}

export interface TelemetryData {
  timestamp: number;
  stats: SimulationStats;
  entities: EntityState[];
}

//==============================================================================
// WebSocket Messages
//==============================================================================

export type MessageType =
  | 'connect'
  | 'disconnect'
  | 'world_state'
  | 'entity_update'
  | 'entity_spawn'
  | 'entity_destroy'
  | 'command'
  | 'response'
  | 'error'
  | 'telemetry';

export interface WebSocketMessage<T = unknown> {
  type: MessageType;
  timestamp: number;
  data: T;
  requestId?: string;
}

export interface SpawnEntityRequest {
  domain: Domain;
  kind: EntityKind;
  name: string;
  position: GeodeticPosition;
  velocity?: NEDVelocity;
  template?: string;
  properties?: Record<string, number | string | boolean>;
}

export interface CommandRequest {
  command: string;
  entityId?: string;
  params?: Record<string, unknown>;
}

//==============================================================================
// UI Types
//==============================================================================

export interface SelectionState {
  selectedEntityId: string | null;
  hoveredEntityId: string | null;
}

export interface ViewState {
  mode: '2d' | '3d' | '3d-photo';
  cameraPosition: GeodeticPosition;
  cameraHeading: number;
  cameraPitch: number;
  zoom: number;
}

export interface FilterState {
  domains: Domain[];
  kinds: EntityKind[];
  searchQuery: string;
  showInactive: boolean;
}

export interface PanelState {
  leftPanelOpen: boolean;
  rightPanelOpen: boolean;
  bottomPanelOpen: boolean;
  activeTabs: {
    left: string;
    right: string;
    bottom: string;
  };
}
