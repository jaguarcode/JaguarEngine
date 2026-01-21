import { useEffect, useCallback } from 'react';
import { useWebSocket } from './useWebSocket';
import { useUIStore } from '@/stores/uiStore';
import { useSimulationStore } from '@/stores/simulationStore';
import { useEntityStore } from '@/stores/entityStore';

export function useKeyboardShortcuts() {
  const { startSimulation, pauseSimulation, resetSimulation } = useWebSocket();
  const { togglePerformance, toggleHelp, toggleLeftPanel, toggleRightPanel } = useUIStore();
  const status = useSimulationStore((s) => s.status);
  const resetSimStore = useSimulationStore((s) => s.reset);
  const clearEntities = useEntityStore((s) => s.clearEntities);
  const resetFilters = useEntityStore((s) => s.resetFilters);

  const handleKeyDown = useCallback(
    (event: KeyboardEvent) => {
      // Ignore if typing in an input
      if (
        event.target instanceof HTMLInputElement ||
        event.target instanceof HTMLTextAreaElement
      ) {
        return;
      }

      const key = event.key.toLowerCase();
      const ctrl = event.ctrlKey || event.metaKey;

      switch (key) {
        case ' ':
          // Space: Toggle play/pause
          event.preventDefault();
          if (status === 'running') {
            pauseSimulation();
          } else {
            startSimulation();
          }
          break;

        case 'r':
          // R: Reset simulation (server + frontend state)
          if (!ctrl) {
            event.preventDefault();
            resetSimulation();
            resetSimStore();
            clearEntities();
            resetFilters();
          }
          break;

        case 'p':
          // P: Toggle performance overlay
          event.preventDefault();
          togglePerformance();
          break;

        case '?':
        case '/':
          // ?: Show help
          if (event.shiftKey || key === '?') {
            event.preventDefault();
            toggleHelp();
          }
          break;

        case '[':
          // [: Toggle left panel
          event.preventDefault();
          toggleLeftPanel();
          break;

        case ']':
          // ]: Toggle right panel
          event.preventDefault();
          toggleRightPanel();
          break;

        case '1':
          // 1: Focus on Air domain
          if (ctrl) {
            event.preventDefault();
            // Could implement domain focus
          }
          break;

        case '2':
          // 2: Focus on Land domain
          if (ctrl) {
            event.preventDefault();
          }
          break;

        case '3':
          // 3: Focus on Sea domain
          if (ctrl) {
            event.preventDefault();
          }
          break;

        case '4':
          // 4: Focus on Space domain
          if (ctrl) {
            event.preventDefault();
          }
          break;

        case 'escape':
          // Escape: Close modals, deselect
          // This is handled by individual components
          break;
      }
    },
    [
      status,
      startSimulation,
      pauseSimulation,
      resetSimulation,
      resetSimStore,
      clearEntities,
      resetFilters,
      togglePerformance,
      toggleHelp,
      toggleLeftPanel,
      toggleRightPanel,
    ]
  );

  useEffect(() => {
    window.addEventListener('keydown', handleKeyDown);
    return () => {
      window.removeEventListener('keydown', handleKeyDown);
    };
  }, [handleKeyDown]);
}

export const KEYBOARD_SHORTCUTS = [
  { key: 'Space', description: 'Play / Pause simulation' },
  { key: 'R', description: 'Reset simulation' },
  { key: 'P', description: 'Toggle performance overlay' },
  { key: '?', description: 'Show help' },
  { key: '[', description: 'Toggle left panel' },
  { key: ']', description: 'Toggle right panel' },
  { key: 'Ctrl+1-4', description: 'Focus domain (Air/Land/Sea/Space)' },
  { key: 'Escape', description: 'Close modal / Deselect' },
];

export default useKeyboardShortcuts;
