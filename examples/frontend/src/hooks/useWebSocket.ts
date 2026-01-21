import { useEffect, useRef, useMemo } from 'react';
import { websocketService } from '@/services/websocket';
import { useSimulationStore } from '@/stores/simulationStore';
import { useEntityStore } from '@/stores/entityStore';
import { useAIAgentStore } from '@/stores/aiAgentStore';
import { entityPositionBuffer } from '@/stores/entityPositionBuffer';
import type { WebSocketMessage, EntityState, SimulationStats } from '@/types';

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
  }), []); // Empty deps - websocketService is a singleton, functions never change
}

export default useWebSocket;
