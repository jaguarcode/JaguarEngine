// JaguarEngine Physics Debug Types
// Types for real-time physics visualization and debugging

//==============================================================================
// Vector Types (Local Coordinate System)
//==============================================================================

export interface Vec3 {
  x: number;
  y: number;
  z: number;
}

export interface Quaternion {
  w: number;
  x: number;
  y: number;
  z: number;
}

//==============================================================================
// Collision Shape Types
//==============================================================================

export type ShapeType =
  | 'sphere'
  | 'box'
  | 'capsule'
  | 'cylinder'
  | 'convex_hull'
  | 'mesh'
  | 'compound';

export interface BaseShape {
  type: ShapeType;
  position: Vec3;  // Local offset from entity center
  rotation: Quaternion;
}

export interface SphereShape extends BaseShape {
  type: 'sphere';
  radius: number;
}

export interface BoxShape extends BaseShape {
  type: 'box';
  halfExtents: Vec3;
}

export interface CapsuleShape extends BaseShape {
  type: 'capsule';
  radius: number;
  height: number;
}

export interface CylinderShape extends BaseShape {
  type: 'cylinder';
  radius: number;
  height: number;
}

export interface ConvexHullShape extends BaseShape {
  type: 'convex_hull';
  vertices: Vec3[];
}

export interface MeshShape extends BaseShape {
  type: 'mesh';
  vertices: Vec3[];
  indices: number[];
}

export interface CompoundShape extends BaseShape {
  type: 'compound';
  children: CollisionShape[];
}

export type CollisionShape =
  | SphereShape
  | BoxShape
  | CapsuleShape
  | CylinderShape
  | ConvexHullShape
  | MeshShape
  | CompoundShape;

//==============================================================================
// AABB Tree Types
//==============================================================================

export interface AABB {
  min: Vec3;
  max: Vec3;
}

export interface AABBNode {
  id: number;
  aabb: AABB;
  entityId?: string;        // Leaf node has entity
  left?: number;            // Child node IDs
  right?: number;
  isLeaf: boolean;
  depth: number;
}

export interface AABBTree {
  nodes: AABBNode[];
  root: number;
  totalNodes: number;
  maxDepth: number;
}

//==============================================================================
// Collision Detection Types
//==============================================================================

export interface ContactPoint {
  position: Vec3;           // World space
  normal: Vec3;             // Points from B to A
  penetration: number;      // Positive = overlapping
  entityA: string;
  entityB: string;
}

export interface CollisionPair {
  entityA: string;
  entityB: string;
  isColliding: boolean;
  contacts: ContactPoint[];
}

export type CollisionLayer =
  | 'default'
  | 'aircraft'
  | 'ground_vehicle'
  | 'ship'
  | 'submarine'
  | 'spacecraft'
  | 'missile'
  | 'projectile'
  | 'terrain'
  | 'ocean'
  | 'structure'
  | 'sensor';

export interface CollisionLayerMask {
  layers: CollisionLayer[];
  mask: number;  // Bitmask representation
}

//==============================================================================
// Constraint Types
//==============================================================================

export type ConstraintType =
  | 'distance'
  | 'ball_socket'
  | 'hinge'
  | 'slider'
  | 'fixed'
  | 'contact'
  | 'friction'
  | 'distance_limit'
  | 'angle_limit'
  | 'universal'
  | 'cone'
  | 'motor'
  | 'custom';

export interface ConstraintBody {
  entityId: string;
  localAnchor: Vec3;
  localAxis?: Vec3;
}

export interface BaseConstraint {
  id: string;
  type: ConstraintType;
  bodyA: ConstraintBody;
  bodyB: ConstraintBody | null;  // null for world-fixed
  enabled: boolean;
  breakForce?: number;
  breakTorque?: number;
}

export interface DistanceConstraint extends BaseConstraint {
  type: 'distance';
  minDistance: number;
  maxDistance: number;
  currentDistance: number;
  stiffness: number;
  damping: number;
}

export interface HingeConstraint extends BaseConstraint {
  type: 'hinge';
  axis: Vec3;
  currentAngle: number;
  minAngle?: number;
  maxAngle?: number;
  motorEnabled: boolean;
  motorTargetVelocity?: number;
  motorMaxForce?: number;
}

export interface BallSocketConstraint extends BaseConstraint {
  type: 'ball_socket';
  swingLimit?: number;
  twistLimit?: number;
}

export interface SliderConstraint extends BaseConstraint {
  type: 'slider';
  axis: Vec3;
  currentPosition: number;
  minPosition?: number;
  maxPosition?: number;
  motorEnabled: boolean;
  motorTargetVelocity?: number;
  motorMaxForce?: number;
}

export interface FixedConstraint extends BaseConstraint {
  type: 'fixed';
}

export type Constraint =
  | DistanceConstraint
  | HingeConstraint
  | BallSocketConstraint
  | SliderConstraint
  | FixedConstraint
  | BaseConstraint;

//==============================================================================
// Force Visualization Types
//==============================================================================

export type ForceType =
  | 'gravity'
  | 'aerodynamic_lift'
  | 'aerodynamic_drag'
  | 'thrust'
  | 'buoyancy'
  | 'hydrodynamic'
  | 'ground_contact'
  | 'constraint'
  | 'external'
  | 'total';

export interface ForceVector {
  entityId: string;
  type: ForceType;
  force: Vec3;            // World space force vector
  applicationPoint: Vec3;  // World space point
  torque?: Vec3;          // Resulting torque
  magnitude: number;
  color?: string;         // Suggested visualization color
}

export interface EntityForces {
  entityId: string;
  forces: ForceVector[];
  totalForce: Vec3;
  totalTorque: Vec3;
}

//==============================================================================
// Integrator Types
//==============================================================================

export type IntegratorType =
  | 'euler'
  | 'symplectic_euler'
  | 'verlet'
  | 'rk4'
  | 'abm4'
  | 'dormand_prince'
  | 'boris';

export interface IntegratorMetrics {
  name: IntegratorType;
  order: number;
  currentTimestep: number;
  adaptiveTimestep: boolean;
  errorEstimate?: number;
  rejectedSteps?: number;
  acceptedSteps?: number;
}

//==============================================================================
// Energy & Momentum Types
//==============================================================================

export interface EnergyMetrics {
  kineticEnergy: number;
  potentialEnergy: number;
  totalEnergy: number;
  energyDrift: number;        // Percentage drift from initial
  initialEnergy: number;
}

export interface MomentumMetrics {
  linearMomentum: Vec3;
  angularMomentum: Vec3;
  centerOfMass: Vec3;
  totalMass: number;
}

//==============================================================================
// Physics Debug State
//==============================================================================

export interface PhysicsDebugState {
  // Timestamp
  timestamp: number;
  simulationTime: number;

  // Collision System
  collisionShapes: Map<string, CollisionShape>;  // entityId -> shape
  aabbTree: AABBTree | null;
  contacts: ContactPoint[];
  collisionPairs: CollisionPair[];
  broadPhasePairsChecked: number;
  narrowPhasePairsChecked: number;

  // Constraint System
  constraints: Constraint[];
  constraintIterations: number;
  constraintError: number;

  // Force Visualization
  entityForces: EntityForces[];

  // Energy & Conservation
  energy: EnergyMetrics;
  momentum: MomentumMetrics;

  // Integrator
  integrator: IntegratorMetrics;
}

//==============================================================================
// Visualization Settings
//==============================================================================

export interface PhysicsVisualizationSettings {
  // Visibility toggles
  showCollisionShapes: boolean;
  showAABBTree: boolean;
  showContactPoints: boolean;
  showContactNormals: boolean;
  showConstraints: boolean;
  showForceVectors: boolean;
  showVelocityVectors: boolean;
  showAngularVelocity: boolean;
  showCenterOfMass: boolean;

  // AABB Tree settings
  aabbTreeMaxDepth: number;
  aabbTreeShowOnlyLeaves: boolean;

  // Force visualization settings
  forceScale: number;           // Visual scale multiplier
  forceThreshold: number;       // Minimum force to display
  showForceLabels: boolean;
  forceTypes: ForceType[];      // Which force types to show

  // Constraint visualization
  constraintScale: number;
  showConstraintLimits: boolean;
  showConstraintForces: boolean;

  // Colors
  colors: {
    collisionShape: string;
    collisionShapeColliding: string;
    aabbNode: string;
    aabbLeaf: string;
    contactPoint: string;
    contactNormal: string;
    constraintActive: string;
    constraintBroken: string;
    forceGravity: string;
    forceLift: string;
    forceDrag: string;
    forceThrust: string;
    forceBuoyancy: string;
    forceTotal: string;
    velocity: string;
    angularVelocity: string;
    centerOfMass: string;
  };

  // Opacity
  shapeOpacity: number;
  aabbOpacity: number;
}

export const defaultPhysicsVisualizationSettings: PhysicsVisualizationSettings = {
  // Visibility
  showCollisionShapes: true,
  showAABBTree: false,
  showContactPoints: true,
  showContactNormals: true,
  showConstraints: true,
  showForceVectors: false,
  showVelocityVectors: false,
  showAngularVelocity: false,
  showCenterOfMass: false,

  // AABB Tree
  aabbTreeMaxDepth: 4,
  aabbTreeShowOnlyLeaves: false,

  // Forces
  forceScale: 0.001,
  forceThreshold: 1.0,
  showForceLabels: false,
  forceTypes: ['total', 'gravity', 'aerodynamic_lift', 'aerodynamic_drag', 'thrust'],

  // Constraints
  constraintScale: 1.0,
  showConstraintLimits: true,
  showConstraintForces: false,

  // Colors
  colors: {
    collisionShape: '#00ff00',
    collisionShapeColliding: '#ff0000',
    aabbNode: '#ffff00',
    aabbLeaf: '#00ffff',
    contactPoint: '#ff00ff',
    contactNormal: '#ffffff',
    constraintActive: '#00ff88',
    constraintBroken: '#ff4444',
    forceGravity: '#8888ff',
    forceLift: '#00ff00',
    forceDrag: '#ff8800',
    forceThrust: '#ffff00',
    forceBuoyancy: '#00ffff',
    forceTotal: '#ffffff',
    velocity: '#00ff00',
    angularVelocity: '#ff00ff',
    centerOfMass: '#ffff00',
  },

  // Opacity
  shapeOpacity: 0.3,
  aabbOpacity: 0.2,
};

//==============================================================================
// Event Log Types
//==============================================================================

export type EventCategory =
  | 'system'
  | 'entity'
  | 'physics'
  | 'domain'
  | 'sensor'
  | 'threshold'
  | 'user';

export type EventSeverity = 'info' | 'warning' | 'error' | 'debug';

export interface PhysicsEvent {
  id: string;
  timestamp: number;
  simulationTime: number;
  category: EventCategory;
  type: string;
  severity: EventSeverity;
  message: string;
  entityId?: string;
  data?: Record<string, unknown>;
}

//==============================================================================
// WebSocket Message Types for Physics Debug
//==============================================================================

export interface PhysicsDebugMessage {
  type: 'physics_debug';
  timestamp: number;
  data: {
    shapes?: Array<{ entityId: string; shape: CollisionShape }>;
    aabbTree?: AABBTree;
    contacts?: ContactPoint[];
    constraints?: Constraint[];
    forces?: EntityForces[];
    energy?: EnergyMetrics;
    momentum?: MomentumMetrics;
    integrator?: IntegratorMetrics;
  };
}

export interface PhysicsEventMessage {
  type: 'physics_event';
  timestamp: number;
  data: PhysicsEvent;
}

export interface PhysicsMetricsMessage {
  type: 'physics_metrics';
  timestamp: number;
  data: {
    energy: EnergyMetrics;
    momentum: MomentumMetrics;
    integrator: IntegratorMetrics;
    collisionStats: {
      broadPhasePairs: number;
      narrowPhasePairs: number;
      activeContacts: number;
    };
    constraintStats: {
      activeConstraints: number;
      iterations: number;
      residualError: number;
    };
  };
}
