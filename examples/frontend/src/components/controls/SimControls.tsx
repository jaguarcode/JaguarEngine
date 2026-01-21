import React, { useCallback } from 'react';
import clsx from 'clsx';
import { useWebSocket } from '@/hooks/useWebSocket';
import { useSimulationStore } from '@/stores/simulationStore';
import { useEntityStore } from '@/stores/entityStore';
import { useAIAgentStore } from '@/stores/aiAgentStore';

// Icon components
const PlayIcon = () => (
  <svg className="w-5 h-5" fill="currentColor" viewBox="0 0 20 20">
    <path
      fillRule="evenodd"
      d="M10 18a8 8 0 100-16 8 8 0 000 16zM9.555 7.168A1 1 0 008 8v4a1 1 0 001.555.832l3-2a1 1 0 000-1.664l-3-2z"
      clipRule="evenodd"
    />
  </svg>
);

const PauseIcon = () => (
  <svg className="w-5 h-5" fill="currentColor" viewBox="0 0 20 20">
    <path
      fillRule="evenodd"
      d="M18 10a8 8 0 11-16 0 8 8 0 0116 0zM7 8a1 1 0 012 0v4a1 1 0 11-2 0V8zm5-1a1 1 0 00-1 1v4a1 1 0 102 0V8a1 1 0 00-1-1z"
      clipRule="evenodd"
    />
  </svg>
);

const StopIcon = () => (
  <svg className="w-5 h-5" fill="currentColor" viewBox="0 0 20 20">
    <path
      fillRule="evenodd"
      d="M10 18a8 8 0 100-16 8 8 0 000 16zM8 7a1 1 0 00-1 1v4a1 1 0 001 1h4a1 1 0 001-1V8a1 1 0 00-1-1H8z"
      clipRule="evenodd"
    />
  </svg>
);

const ResetIcon = () => (
  <svg className="w-5 h-5" fill="currentColor" viewBox="0 0 20 20">
    <path
      fillRule="evenodd"
      d="M4 2a1 1 0 011 1v2.101a7.002 7.002 0 0111.601 2.566 1 1 0 11-1.885.666A5.002 5.002 0 005.999 7H9a1 1 0 010 2H4a1 1 0 01-1-1V3a1 1 0 011-1zm.008 9.057a1 1 0 011.276.61A5.002 5.002 0 0014.001 13H11a1 1 0 110-2h5a1 1 0 011 1v5a1 1 0 11-2 0v-2.101a7.002 7.002 0 01-11.601-2.566 1 1 0 01.61-1.276z"
      clipRule="evenodd"
    />
  </svg>
);

// Time scale presets
const TIME_SCALES = [
  { label: '0.1x', value: 0.1 },
  { label: '0.5x', value: 0.5 },
  { label: '1x', value: 1.0 },
  { label: '2x', value: 2.0 },
  { label: '5x', value: 5.0 },
  { label: '10x', value: 10.0 },
];

export const SimControls: React.FC = () => {
  const {
    startSimulation,
    pauseSimulation,
    stopSimulation,
    resetSimulation,
    setTimeScale,
  } = useWebSocket();

  const status = useSimulationStore((s) => s.status);
  const connected = useSimulationStore((s) => s.connected);
  const config = useSimulationStore((s) => s.config);
  const resetSimStore = useSimulationStore((s) => s.reset);
  const clearEntities = useEntityStore((s) => s.clearEntities);
  const resetFilters = useEntityStore((s) => s.resetFilters);
  const clearAllAgents = useAIAgentStore((s) => s.clearAllAgents);

  const isRunning = status === 'running';
  const isPaused = status === 'paused';
  const canControl = connected && status !== 'error';

  // Reset everything: server simulation + frontend state
  const handleReset = useCallback(() => {
    // Send reset command to server
    resetSimulation();
    // Reset frontend stores
    resetSimStore();
    clearEntities();
    resetFilters();
    clearAllAgents();
  }, [resetSimulation, resetSimStore, clearEntities, resetFilters, clearAllAgents]);

  // Handle time scale change without stealing focus
  const handleTimeScaleChange = useCallback(
    (e: React.MouseEvent<HTMLButtonElement>, scale: number) => {
      e.preventDefault();
      (e.target as HTMLButtonElement).blur();
      setTimeScale(scale);
    },
    [setTimeScale]
  );

  return (
    <div className="panel">
      <div className="panel-header">
        <h3 className="panel-title">Simulation Control</h3>
        <div className="flex items-center gap-2">
          <div
            className={clsx('status-dot', {
              'status-dot-active': connected && isRunning,
              'status-dot-warning': connected && isPaused,
              'status-dot-error': !connected || status === 'error',
              'status-dot-idle': connected && (status === 'idle' || status === 'stopped'),
            })}
          />
          <span className="text-xs text-gray-400">
            {!connected ? 'Disconnected' : status.toUpperCase()}
          </span>
        </div>
      </div>

      <div className="panel-content space-y-4">
        {/* Main controls */}
        <div className="flex items-center gap-2">
          {/* Play/Pause button */}
          <button
            className={clsx(
              'btn flex-1 flex items-center justify-center gap-2',
              isRunning ? 'btn-warning' : 'btn-success'
            )}
            onClick={isRunning ? pauseSimulation : startSimulation}
            disabled={!canControl}
          >
            {isRunning ? (
              <>
                <PauseIcon /> Pause
              </>
            ) : (
              <>
                <PlayIcon /> {isPaused ? 'Resume' : 'Start'}
              </>
            )}
          </button>

          {/* Stop button */}
          <button
            className="btn btn-danger flex items-center justify-center gap-2"
            onClick={stopSimulation}
            disabled={!canControl || status === 'idle' || status === 'stopped'}
          >
            <StopIcon /> Stop
          </button>

          {/* Reset button */}
          <button
            className="btn btn-secondary flex items-center justify-center gap-2"
            onClick={handleReset}
            disabled={!canControl}
            title="Reset simulation"
          >
            <ResetIcon />
          </button>
        </div>

        {/* Time scale */}
        <div>
          <label className="label">Time Scale</label>
          <div className="flex flex-wrap gap-1">
            {TIME_SCALES.map(({ label, value }) => (
              <button
                key={value}
                className={clsx(
                  'px-2 py-1 text-xs rounded transition-colors',
                  config.timeScale === value
                    ? 'bg-primary-600 text-white'
                    : 'bg-gray-700 text-gray-300 hover:bg-gray-600'
                )}
                onClick={(e) => handleTimeScaleChange(e, value)}
                disabled={!canControl}
              >
                {label}
              </button>
            ))}
          </div>
        </div>

        {/* Keyboard shortcuts hint */}
        <div className="text-xs text-gray-600 flex items-center gap-4">
          <span>
            <kbd className="px-1 py-0.5 bg-gray-800 rounded text-gray-400">Space</kbd> Play/Pause
          </span>
          <span>
            <kbd className="px-1 py-0.5 bg-gray-800 rounded text-gray-400">R</kbd> Reset
          </span>
        </div>
      </div>
    </div>
  );
};

export default SimControls;
