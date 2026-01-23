import { useEffect, useCallback, useMemo } from 'react';
import { websocketService } from '@/services/websocket';
import { usePhysicsDebugStore } from '@/stores/physicsDebugStore';
import { useSimulationStore } from '@/stores/simulationStore';
import type { PhysicsVisualizationSettings } from '@/types/physics';

/**
 * Hook to manage physics debug data streaming.
 * Automatically enables/disables debug data based on store state
 * and handles connection lifecycle.
 */
export function usePhysicsDebug() {
  const debugEnabled = usePhysicsDebugStore((s) => s.debugEnabled);
  const debugStreamConnected = usePhysicsDebugStore((s) => s.debugStreamConnected);
  const visualizationSettings = usePhysicsDebugStore((s) => s.visualizationSettings);
  const setDebugEnabled = usePhysicsDebugStore((s) => s.setDebugEnabled);
  const reset = usePhysicsDebugStore((s) => s.reset);
  const isConnected = useSimulationStore((s) => s.connected);

  // Sync debug enabled state with server
  useEffect(() => {
    if (isConnected) {
      websocketService.setPhysicsDebugEnabled(debugEnabled);
    }
  }, [debugEnabled, isConnected]);

  // Configure debug stream based on visualization settings
  useEffect(() => {
    if (isConnected && debugEnabled) {
      websocketService.configurePhysicsDebug({
        collisionShapes: visualizationSettings.showCollisionShapes,
        aabbTree: visualizationSettings.showAABBTree,
        contacts: visualizationSettings.showContactPoints,
        constraints: visualizationSettings.showConstraints,
        forces: visualizationSettings.showForceVectors,
        energy: true, // Always send energy for dashboard
        momentum: true, // Always send momentum for dashboard
        integrator: true, // Always send integrator info
        updateRate: 20, // 20 Hz default
      });
    }
  }, [
    isConnected,
    debugEnabled,
    visualizationSettings.showCollisionShapes,
    visualizationSettings.showAABBTree,
    visualizationSettings.showContactPoints,
    visualizationSettings.showConstraints,
    visualizationSettings.showForceVectors,
  ]);

  // Reset debug store when debug is disabled
  useEffect(() => {
    if (!debugEnabled) {
      reset();
    }
  }, [debugEnabled, reset]);

  // Toggle debug enabled state
  const toggleDebug = useCallback(() => {
    setDebugEnabled(!debugEnabled);
  }, [debugEnabled, setDebugEnabled]);

  // Request manual snapshot
  const requestSnapshot = useCallback(() => {
    if (isConnected) {
      websocketService.requestPhysicsSnapshot();
    }
  }, [isConnected]);

  // Update visualization settings
  const updateSettings = useCallback(
    (settings: Partial<PhysicsVisualizationSettings>) => {
      usePhysicsDebugStore.getState().updateVisualizationSettings(settings);
    },
    []
  );

  // Reset visualization settings to defaults
  const resetSettings = useCallback(() => {
    usePhysicsDebugStore.getState().resetVisualizationSettings();
  }, []);

  return useMemo(
    () => ({
      // State
      debugEnabled,
      debugStreamConnected,
      visualizationSettings,
      isConnected,

      // Actions
      toggleDebug,
      setDebugEnabled,
      requestSnapshot,
      updateSettings,
      resetSettings,
      reset,
    }),
    [
      debugEnabled,
      debugStreamConnected,
      visualizationSettings,
      isConnected,
      toggleDebug,
      setDebugEnabled,
      requestSnapshot,
      updateSettings,
      resetSettings,
      reset,
    ]
  );
}

export default usePhysicsDebug;
