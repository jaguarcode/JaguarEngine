import React from 'react';
import clsx from 'clsx';
import { useUIStore } from '@/stores/uiStore';
import { useSimulationStore } from '@/stores/simulationStore';

export const PerformanceOverlay: React.FC = () => {
  const showPerformance = useUIStore((s) => s.showPerformance);
  const stats = useSimulationStore((s) => s.stats);

  if (!showPerformance) return null;

  const frameRateColor =
    stats.frameRate >= 55 ? 'text-green-400' : stats.frameRate >= 30 ? 'text-yellow-400' : 'text-red-400';

  const physicsColor =
    stats.physicsTime <= 10 ? 'text-green-400' : stats.physicsTime <= 20 ? 'text-yellow-400' : 'text-red-400';

  return (
    <div className="fixed top-4 left-1/2 -translate-x-1/2 z-40 panel px-4 py-2 animate-fadeIn">
      <div className="flex items-center gap-6 text-xs font-mono">
        <div className="flex items-center gap-2">
          <span className="text-gray-500">FPS:</span>
          <span className={clsx('font-bold', frameRateColor)}>
            {stats.frameRate.toFixed(1)}
          </span>
        </div>

        <div className="flex items-center gap-2">
          <span className="text-gray-500">Physics:</span>
          <span className={clsx('font-bold', physicsColor)}>
            {stats.physicsTime.toFixed(2)}ms
          </span>
        </div>

        <div className="flex items-center gap-2">
          <span className="text-gray-500">Entities:</span>
          <span className="font-bold text-gray-200">{stats.totalEntities}</span>
        </div>

        <div className="flex items-center gap-2">
          <span className="text-gray-500">RT Ratio:</span>
          <span className="font-bold text-gray-200">{stats.realtimeRatio.toFixed(2)}x</span>
        </div>

        <div className="flex items-center gap-2">
          <span className="text-gray-500">Sim Time:</span>
          <span className="font-bold text-gray-200">{stats.simulationTime.toFixed(1)}s</span>
        </div>
      </div>
    </div>
  );
};

export default PerformanceOverlay;
