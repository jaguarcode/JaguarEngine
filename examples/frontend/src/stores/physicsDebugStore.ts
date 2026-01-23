import { create } from 'zustand';
import { subscribeWithSelector } from 'zustand/middleware';
import type {
  CollisionShape,
  AABBTree,
  ContactPoint,
  CollisionPair,
  Constraint,
  EntityForces,
  EnergyMetrics,
  MomentumMetrics,
  IntegratorMetrics,
  PhysicsVisualizationSettings,
  PhysicsEvent,
  Vec3,
} from '@/types/physics';
import { defaultPhysicsVisualizationSettings } from '@/types/physics';

// Throttle updates to prevent performance issues
let lastDebugUpdateTime = 0;
const DEBUG_UPDATE_INTERVAL_MS = 50; // ~20fps for debug data

// History buffer size for energy/momentum graphs
const HISTORY_BUFFER_SIZE = 300;

//==============================================================================
// History Data Types
//==============================================================================

interface HistoryPoint {
  time: number;
  value: number;
}

interface EnergyHistory {
  kinetic: HistoryPoint[];
  potential: HistoryPoint[];
  total: HistoryPoint[];
}

interface MomentumHistory {
  linearMagnitude: HistoryPoint[];
  angularMagnitude: HistoryPoint[];
}

//==============================================================================
// Store Interface
//==============================================================================

interface PhysicsDebugState {
  // Debug data enabled
  debugEnabled: boolean;
  debugStreamConnected: boolean;

  // Collision System
  collisionShapes: Map<string, CollisionShape>;
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
  entityForces: Map<string, EntityForces>;

  // Energy & Conservation
  energy: EnergyMetrics;
  momentum: MomentumMetrics;

  // History for graphs
  energyHistory: EnergyHistory;
  momentumHistory: MomentumHistory;

  // Integrator
  integrator: IntegratorMetrics;

  // Event Log
  events: PhysicsEvent[];
  maxEventLogSize: number;

  // Visualization Settings
  visualizationSettings: PhysicsVisualizationSettings;

  // Selected entity for detail view
  focusedEntityId: string | null;

  // Simulation time reference
  lastSimulationTime: number;

  // Actions
  setDebugEnabled: (enabled: boolean) => void;
  setDebugStreamConnected: (connected: boolean) => void;

  // Update actions
  updateCollisionShapes: (shapes: Array<{ entityId: string; shape: CollisionShape }>) => void;
  updateAABBTree: (tree: AABBTree) => void;
  updateContacts: (contacts: ContactPoint[]) => void;
  updateConstraints: (constraints: Constraint[]) => void;
  updateEntityForces: (forces: EntityForces[]) => void;
  updateEnergy: (energy: EnergyMetrics, simTime: number) => void;
  updateMomentum: (momentum: MomentumMetrics, simTime: number) => void;
  updateIntegrator: (integrator: IntegratorMetrics) => void;
  updateCollisionStats: (broadPhase: number, narrowPhase: number) => void;
  updateConstraintStats: (iterations: number, error: number) => void;

  // Event log actions
  addEvent: (event: PhysicsEvent) => void;
  clearEvents: () => void;

  // Visualization settings actions
  updateVisualizationSettings: (settings: Partial<PhysicsVisualizationSettings>) => void;
  resetVisualizationSettings: () => void;

  // Focus actions
  setFocusedEntity: (entityId: string | null) => void;

  // Reset
  reset: () => void;
}

//==============================================================================
// Initial State
//==============================================================================

const defaultEnergy: EnergyMetrics = {
  kineticEnergy: 0,
  potentialEnergy: 0,
  totalEnergy: 0,
  energyDrift: 0,
  initialEnergy: 0,
};

const defaultMomentum: MomentumMetrics = {
  linearMomentum: { x: 0, y: 0, z: 0 },
  angularMomentum: { x: 0, y: 0, z: 0 },
  centerOfMass: { x: 0, y: 0, z: 0 },
  totalMass: 0,
};

const defaultIntegrator: IntegratorMetrics = {
  name: 'rk4',
  order: 4,
  currentTimestep: 0.01,
  adaptiveTimestep: false,
};

const defaultEnergyHistory: EnergyHistory = {
  kinetic: [],
  potential: [],
  total: [],
};

const defaultMomentumHistory: MomentumHistory = {
  linearMagnitude: [],
  angularMagnitude: [],
};

//==============================================================================
// Helper Functions
//==============================================================================

function vec3Magnitude(v: Vec3): number {
  return Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

function addToHistory(
  history: HistoryPoint[],
  time: number,
  value: number,
  maxSize: number
): HistoryPoint[] {
  const newHistory = [...history, { time, value }];
  if (newHistory.length > maxSize) {
    return newHistory.slice(-maxSize);
  }
  return newHistory;
}

//==============================================================================
// Store Implementation
//==============================================================================

export const usePhysicsDebugStore = create<PhysicsDebugState>()(
  subscribeWithSelector((set, get) => ({
    // Initial state
    debugEnabled: false,
    debugStreamConnected: false,

    collisionShapes: new Map(),
    aabbTree: null,
    contacts: [],
    collisionPairs: [],
    broadPhasePairsChecked: 0,
    narrowPhasePairsChecked: 0,

    constraints: [],
    constraintIterations: 0,
    constraintError: 0,

    entityForces: new Map(),

    energy: defaultEnergy,
    momentum: defaultMomentum,

    energyHistory: defaultEnergyHistory,
    momentumHistory: defaultMomentumHistory,

    integrator: defaultIntegrator,

    events: [],
    maxEventLogSize: 500,

    visualizationSettings: { ...defaultPhysicsVisualizationSettings },

    focusedEntityId: null,
    lastSimulationTime: 0,

    // Actions
    setDebugEnabled: (enabled) => set({ debugEnabled: enabled }),

    setDebugStreamConnected: (connected) => set({ debugStreamConnected: connected }),

    updateCollisionShapes: (shapes) => {
      const now = performance.now();
      if (now - lastDebugUpdateTime < DEBUG_UPDATE_INTERVAL_MS) return;
      lastDebugUpdateTime = now;

      const newShapes = new Map(get().collisionShapes);
      shapes.forEach(({ entityId, shape }) => {
        newShapes.set(entityId, shape);
      });
      set({ collisionShapes: newShapes });
    },

    updateAABBTree: (tree) => {
      const now = performance.now();
      if (now - lastDebugUpdateTime < DEBUG_UPDATE_INTERVAL_MS) return;
      set({ aabbTree: tree });
    },

    updateContacts: (contacts) => {
      const now = performance.now();
      if (now - lastDebugUpdateTime < DEBUG_UPDATE_INTERVAL_MS) return;
      set({ contacts });
    },

    updateConstraints: (constraints) => {
      const now = performance.now();
      if (now - lastDebugUpdateTime < DEBUG_UPDATE_INTERVAL_MS) return;
      set({ constraints });
    },

    updateEntityForces: (forces) => {
      const now = performance.now();
      if (now - lastDebugUpdateTime < DEBUG_UPDATE_INTERVAL_MS) return;

      const newForces = new Map<string, EntityForces>();
      forces.forEach((f) => {
        newForces.set(f.entityId, f);
      });
      set({ entityForces: newForces });
    },

    updateEnergy: (energy, simTime) => {
      const state = get();
      const newHistory: EnergyHistory = {
        kinetic: addToHistory(
          state.energyHistory.kinetic,
          simTime,
          energy.kineticEnergy,
          HISTORY_BUFFER_SIZE
        ),
        potential: addToHistory(
          state.energyHistory.potential,
          simTime,
          energy.potentialEnergy,
          HISTORY_BUFFER_SIZE
        ),
        total: addToHistory(
          state.energyHistory.total,
          simTime,
          energy.totalEnergy,
          HISTORY_BUFFER_SIZE
        ),
      };
      set({
        energy,
        energyHistory: newHistory,
        lastSimulationTime: simTime,
      });
    },

    updateMomentum: (momentum, simTime) => {
      const state = get();
      const linearMag = vec3Magnitude(momentum.linearMomentum);
      const angularMag = vec3Magnitude(momentum.angularMomentum);

      const newHistory: MomentumHistory = {
        linearMagnitude: addToHistory(
          state.momentumHistory.linearMagnitude,
          simTime,
          linearMag,
          HISTORY_BUFFER_SIZE
        ),
        angularMagnitude: addToHistory(
          state.momentumHistory.angularMagnitude,
          simTime,
          angularMag,
          HISTORY_BUFFER_SIZE
        ),
      };
      set({
        momentum,
        momentumHistory: newHistory,
      });
    },

    updateIntegrator: (integrator) => {
      set({ integrator });
    },

    updateCollisionStats: (broadPhase, narrowPhase) => {
      set({
        broadPhasePairsChecked: broadPhase,
        narrowPhasePairsChecked: narrowPhase,
      });
    },

    updateConstraintStats: (iterations, error) => {
      set({
        constraintIterations: iterations,
        constraintError: error,
      });
    },

    addEvent: (event) => {
      const state = get();
      const newEvents = [event, ...state.events];
      if (newEvents.length > state.maxEventLogSize) {
        newEvents.pop();
      }
      set({ events: newEvents });
    },

    clearEvents: () => set({ events: [] }),

    updateVisualizationSettings: (settings) => {
      set((state) => ({
        visualizationSettings: { ...state.visualizationSettings, ...settings },
      }));
    },

    resetVisualizationSettings: () => {
      set({ visualizationSettings: { ...defaultPhysicsVisualizationSettings } });
    },

    setFocusedEntity: (entityId) => set({ focusedEntityId: entityId }),

    reset: () =>
      set({
        collisionShapes: new Map(),
        aabbTree: null,
        contacts: [],
        collisionPairs: [],
        broadPhasePairsChecked: 0,
        narrowPhasePairsChecked: 0,
        constraints: [],
        constraintIterations: 0,
        constraintError: 0,
        entityForces: new Map(),
        energy: defaultEnergy,
        momentum: defaultMomentum,
        energyHistory: defaultEnergyHistory,
        momentumHistory: defaultMomentumHistory,
        integrator: defaultIntegrator,
        events: [],
        focusedEntityId: null,
        lastSimulationTime: 0,
      }),
  }))
);

//==============================================================================
// Selectors
//==============================================================================

export const selectDebugEnabled = (state: PhysicsDebugState) => state.debugEnabled;

export const selectContacts = (state: PhysicsDebugState) => state.contacts;

export const selectConstraints = (state: PhysicsDebugState) => state.constraints;

export const selectEnergy = (state: PhysicsDebugState) => state.energy;

export const selectMomentum = (state: PhysicsDebugState) => state.momentum;

export const selectIntegrator = (state: PhysicsDebugState) => state.integrator;

export const selectVisualizationSettings = (state: PhysicsDebugState) =>
  state.visualizationSettings;

export const selectEnergyHistory = (state: PhysicsDebugState) => state.energyHistory;

export const selectMomentumHistory = (state: PhysicsDebugState) => state.momentumHistory;

export const selectEvents = (state: PhysicsDebugState) => state.events;

export const selectCollisionStats = (state: PhysicsDebugState) => ({
  contacts: state.contacts.length,
  broadPhase: state.broadPhasePairsChecked,
  narrowPhase: state.narrowPhasePairsChecked,
});

export const selectConstraintStats = (state: PhysicsDebugState) => ({
  count: state.constraints.length,
  iterations: state.constraintIterations,
  error: state.constraintError,
});

export const selectEntityForces = (entityId: string) => (state: PhysicsDebugState) =>
  state.entityForces.get(entityId);

export const selectCollisionShape = (entityId: string) => (state: PhysicsDebugState) =>
  state.collisionShapes.get(entityId);
