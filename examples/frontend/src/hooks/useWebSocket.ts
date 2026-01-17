import { useEffect, useRef } from 'react';
import { websocketService } from '@/services/websocket';
import { useSimulationStore } from '@/stores/simulationStore';
import { useEntityStore } from '@/stores/entityStore';
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

    // Handle entity spawn
    const handleEntitySpawn = (message: WebSocketMessage) => {
      const entity = message.data as EntityState;
      useEntityStore.getState().addEntity(entity);
    };

    // Handle entity destroy
    const handleEntityDestroy = (message: WebSocketMessage) => {
      const data = message.data as { entityId: string };
      useEntityStore.getState().removeEntity(data.entityId);
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
    const unsubEntitySpawn = websocketService.onMessage('entity_spawn', handleEntitySpawn);
    const unsubEntityDestroy = websocketService.onMessage('entity_destroy', handleEntityDestroy);
    const unsubTelemetry = websocketService.onMessage('telemetry', handleTelemetry);

    // Connect
    websocketService.connect();

    // Cleanup
    return () => {
      unsubConnection();
      unsubWorldState();
      unsubEntityUpdate();
      unsubEntitySpawn();
      unsubEntityDestroy();
      unsubTelemetry();
      initializedRef.current = false;
    };
  }, []); // Empty dependency array - only run once

  return {
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
    setFlightControls: websocketService.setFlightControls.bind(websocketService),
    setVehicleControls: websocketService.setVehicleControls.bind(websocketService),
    setShipControls: websocketService.setShipControls.bind(websocketService),
    setAutopilot: websocketService.setAutopilot.bind(websocketService),
  };
}

export default useWebSocket;
