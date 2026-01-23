import { useEffect, useRef, useMemo } from 'react';
import { websocketService } from '@/services/websocket';
import { useSimulationStore } from '@/stores/simulationStore';
import { useEntityStore } from '@/stores/entityStore';
import { useAIAgentStore } from '@/stores/aiAgentStore';
import { usePhysicsDebugStore } from '@/stores/physicsDebugStore';
import { entityPositionBuffer } from '@/stores/entityPositionBuffer';
import type { WebSocketMessage, EntityState, SimulationStats } from '@/types';
import type {
  CollisionShape,
  AABBTree,
  ContactPoint,
  Constraint,
  EntityForces,
  EnergyMetrics,
  MomentumMetrics,
  IntegratorMetrics,
  PhysicsEvent,
} from '@/types/physics';

export function useWebSocket() {
  // Track if we've already initialized
  const initializedRef = useRef(false);

  // Setup WebSocket connection and handlers - only once
  useEffect(() => {
    // Prevent double initialization
    if (initializedRef.current) {
      return;
    }
    initializedRef.current = true;

    // Handle world state updates - use store.getState() to avoid dependency issues
    const handleWorldState = (message: WebSocketMessage) => {
      const data = message.data as {
        entities?: EntityState[];
        stats?: SimulationStats;
        status?: string;
      };

      if (data.entities) {
        // FAST PATH: Update position buffer directly for smooth rendering
        // This bypasses React state and feeds directly into the RAF render loop
        entityPositionBuffer.updatePositions(data.entities.map((e) => {
          // Calculate speed from NED velocity components
          const vel = e.velocity;
          const speed = vel ? Math.sqrt(vel.north * vel.north + vel.east * vel.east + vel.down * vel.down) : 0;

          return {
            id: e.id,
            name: e.name,
            domain: e.domain,
            kind: e.kind,
            isActive: e.isActive,
            position: e.position,
            orientation: e.orientation, // EulerAngles: roll, pitch, yaw
            velocity: { speed },
          };
        }));

        // SLOW PATH: Update React state for UI components (entity list, filters, etc.)
        // This can be throttled more aggressively
        useEntityStore.getState().setEntities(data.entities);
      }

      if (data.stats) {
        useSimulationStore.getState().updateStats(data.stats);
      }

      if (data.status) {
        useSimulationStore.getState().setStatus(
          data.status as 'idle' | 'running' | 'paused' | 'stopped' | 'error'
        );
      }
    };

    // Handle entity updates
    const handleEntityUpdate = (message: WebSocketMessage) => {
      const entity = message.data as EntityState;
      useEntityStore.getState().updateEntity(entity.id, entity);
    };

    // Handle entity spawned confirmation
    const handleEntitySpawned = (_message: WebSocketMessage) => {
      // Server sends: { id: "entity_X", name: "...", domain: "..." }
      // The full entity will be included in the next world_state update
      // No logging needed - entity will appear in the list automatically
    };

    // Handle entity destroyed confirmation
    const handleEntityDestroyed = (message: WebSocketMessage) => {
      const data = message.data as { id: string };
      // Remove entity from entity store
      useEntityStore.getState().removeEntity(data.id);
      // Also remove any AI agent associated with this entity
      useAIAgentStore.getState().removeAgent(data.id);
    };

    // Handle telemetry
    const handleTelemetry = (message: WebSocketMessage) => {
      const data = message.data as { stats: SimulationStats };
      if (data.stats) {
        useSimulationStore.getState().updateStats(data.stats);
      }
    };

    //==========================================================================
    // Physics Debug Message Handlers
    //==========================================================================

    // Handle physics debug state updates (main debug data stream)
    const handlePhysicsDebug = (message: WebSocketMessage) => {
      const store = usePhysicsDebugStore.getState();
      const data = message.data as {
        simulationTime?: number;
        collisionShapes?: Array<{ entityId: string; shape: CollisionShape }>;
        aabbTree?: AABBTree;
        contacts?: ContactPoint[];
        constraints?: Constraint[];
        forces?: EntityForces[];
        energy?: EnergyMetrics;
        momentum?: MomentumMetrics;
        integrator?: IntegratorMetrics;
        collisionStats?: { broadPhase: number; narrowPhase: number };
        constraintStats?: { iterations: number; error: number };
      };

      const simTime = data.simulationTime ?? store.lastSimulationTime;

      // Update collision shapes
      if (data.collisionShapes) {
        store.updateCollisionShapes(data.collisionShapes);
      }

      // Update AABB tree
      if (data.aabbTree) {
        store.updateAABBTree(data.aabbTree);
      }

      // Update contacts
      if (data.contacts) {
        store.updateContacts(data.contacts);
      }

      // Update constraints
      if (data.constraints) {
        store.updateConstraints(data.constraints);
      }

      // Update forces
      if (data.forces) {
        store.updateEntityForces(data.forces);
      }

      // Update energy metrics
      if (data.energy) {
        store.updateEnergy(data.energy, simTime);
      }

      // Update momentum metrics
      if (data.momentum) {
        store.updateMomentum(data.momentum, simTime);
      }

      // Update integrator metrics
      if (data.integrator) {
        store.updateIntegrator(data.integrator);
      }

      // Update collision stats
      if (data.collisionStats) {
        store.updateCollisionStats(
          data.collisionStats.broadPhase,
          data.collisionStats.narrowPhase
        );
      }

      // Update constraint stats
      if (data.constraintStats) {
        store.updateConstraintStats(
          data.constraintStats.iterations,
          data.constraintStats.error
        );
      }

      // Mark debug stream as connected
      if (!store.debugStreamConnected) {
        store.setDebugStreamConnected(true);
      }
    };

    // Handle physics events (collisions, constraint breaks, etc.)
    const handlePhysicsEvent = (message: WebSocketMessage) => {
      const event = message.data as PhysicsEvent;
      usePhysicsDebugStore.getState().addEvent(event);
    };

    // Handle physics metrics updates (periodic summary data)
    const handlePhysicsMetrics = (message: WebSocketMessage) => {
      const store = usePhysicsDebugStore.getState();
      const data = message.data as {
        simulationTime: number;
        energy?: EnergyMetrics;
        momentum?: MomentumMetrics;
        integrator?: IntegratorMetrics;
        collisionStats?: { broadPhase: number; narrowPhase: number };
        constraintStats?: { iterations: number; error: number };
      };

      if (data.energy) {
        store.updateEnergy(data.energy, data.simulationTime);
      }

      if (data.momentum) {
        store.updateMomentum(data.momentum, data.simulationTime);
      }

      if (data.integrator) {
        store.updateIntegrator(data.integrator);
      }

      if (data.collisionStats) {
        store.updateCollisionStats(
          data.collisionStats.broadPhase,
          data.collisionStats.narrowPhase
        );
      }

      if (data.constraintStats) {
        store.updateConstraintStats(
          data.constraintStats.iterations,
          data.constraintStats.error
        );
      }
    };

    // Connection handler
    const unsubConnection = websocketService.onConnection((connected, error) => {
      useSimulationStore.getState().setConnected(connected, error);
    });

    // Message handlers
    const unsubWorldState = websocketService.onMessage('world_state', handleWorldState);
    const unsubEntityUpdate = websocketService.onMessage('entity_update', handleEntityUpdate);
    const unsubEntitySpawned = websocketService.onMessage('entity_spawned', handleEntitySpawned);
    const unsubEntityDestroyed = websocketService.onMessage('entity_destroyed', handleEntityDestroyed);
    const unsubTelemetry = websocketService.onMessage('telemetry', handleTelemetry);

    // Physics debug message handlers
    const unsubPhysicsDebug = websocketService.onMessage('physics_debug', handlePhysicsDebug);
    const unsubPhysicsEvent = websocketService.onMessage('physics_event', handlePhysicsEvent);
    const unsubPhysicsMetrics = websocketService.onMessage('physics_metrics', handlePhysicsMetrics);

    // Connect
    websocketService.connect();

    // Cleanup
    return () => {
      unsubConnection();
      unsubWorldState();
      unsubEntityUpdate();
      unsubEntitySpawned();
      unsubEntityDestroyed();
      unsubTelemetry();
      // Physics debug cleanup
      unsubPhysicsDebug();
      unsubPhysicsEvent();
      unsubPhysicsMetrics();
      // Reset physics debug store connection status
      usePhysicsDebugStore.getState().setDebugStreamConnected(false);
      initializedRef.current = false;
    };
  }, []); // Empty dependency array - only run once

  // Memoize return object to prevent new function references on every render
  // This is critical to prevent re-render loops in components that depend on these functions
  return useMemo(() => ({
    isConnected: websocketService.isConnected,
    send: websocketService.send.bind(websocketService),
    sendCommand: websocketService.sendCommand.bind(websocketService),
    startSimulation: websocketService.startSimulation.bind(websocketService),
    pauseSimulation: websocketService.pauseSimulation.bind(websocketService),
    stopSimulation: websocketService.stopSimulation.bind(websocketService),
    resetSimulation: websocketService.resetSimulation.bind(websocketService),
    setTimeScale: websocketService.setTimeScale.bind(websocketService),
    spawnEntity: websocketService.spawnEntity.bind(websocketService),
    destroyEntity: websocketService.destroyEntity.bind(websocketService),
    // Air domain controls
    setFlightControls: websocketService.setFlightControls.bind(websocketService),
    setAutopilot: websocketService.setAutopilot.bind(websocketService),
    // Land domain controls
    setVehicleControls: websocketService.setVehicleControls.bind(websocketService),
    setVehicleAutopilot: websocketService.setVehicleAutopilot.bind(websocketService),
    // Sea domain controls
    setShipControls: websocketService.setShipControls.bind(websocketService),
    setShipAutopilot: websocketService.setShipAutopilot.bind(websocketService),
    // Space domain controls
    setSpaceControls: websocketService.setSpaceControls.bind(websocketService),
    setSpaceAutopilot: websocketService.setSpaceAutopilot.bind(websocketService),
    // Physics debug controls
    setPhysicsDebugEnabled: websocketService.setPhysicsDebugEnabled.bind(websocketService),
    configurePhysicsDebug: websocketService.configurePhysicsDebug.bind(websocketService),
    requestPhysicsSnapshot: websocketService.requestPhysicsSnapshot.bind(websocketService),
  }), []); // Empty deps - websocketService is a singleton, functions never change
}

export default useWebSocket;
