import React from 'react';
import clsx from 'clsx';
import { useSimulationStore } from '@/stores/simulationStore';

const formatTime = (seconds: number): string => {
  const h = Math.floor(seconds / 3600);
  const m = Math.floor((seconds % 3600) / 60);
  const s = Math.floor(seconds % 60);
  const ms = Math.floor((seconds % 1) * 1000);

  if (h > 0) {
    return `${h}:${m.toString().padStart(2, '0')}:${s.toString().padStart(2, '0')}`;
  }
  return `${m}:${s.toString().padStart(2, '0')}.${ms.toString().padStart(3, '0')}`;
};

const MetricCard: React.FC<{
  label: string;
  value: string | number;
  unit?: string;
  variant?: 'default' | 'success' | 'warning' | 'danger';
}> = ({ label, value, unit, variant = 'default' }) => {
  const valueColor = {
    default: 'text-white',
    success: 'text-green-400',
    warning: 'text-yellow-400',
    danger: 'text-red-400',
  }[variant];

  return (
    <div className="metric-card">
      <div className="metric-label">{label}</div>
      <div className={clsx('metric-value', valueColor)}>
        {value}
        {unit && <span className="metric-unit">{unit}</span>}
      </div>
    </div>
  );
};

export const TelemetryPanel: React.FC = () => {
  const stats = useSimulationStore((s) => s.stats);
  const status = useSimulationStore((s) => s.status);
  const connected = useSimulationStore((s) => s.connected);

  // Determine performance health
  const frameRateHealth =
    stats.frameRate >= 55 ? 'success' : stats.frameRate >= 30 ? 'warning' : 'danger';

  const physicsTimeHealth =
    stats.physicsTime <= 10 ? 'success' : stats.physicsTime <= 20 ? 'warning' : 'danger';

  const realtimeHealth =
    stats.realtimeRatio >= 0.95
      ? 'success'
      : stats.realtimeRatio >= 0.8
      ? 'warning'
      : 'danger';

  return (
    <div className="panel h-full flex flex-col">
      <div className="panel-header">
        <h3 className="panel-title">Telemetry</h3>
        <div className="flex items-center gap-2">
          <div
            className={clsx('status-dot', {
              'status-dot-active': status === 'running',
              'status-dot-warning': status === 'paused',
              'status-dot-error': status === 'error' || !connected,
              'status-dot-idle': status === 'idle' || status === 'stopped',
            })}
          />
          <span className="text-xs text-gray-400 uppercase">{status}</span>
        </div>
      </div>

      <div className="panel-content flex-1 overflow-y-auto custom-scrollbar space-y-4">
        {/* Time Section */}
        <div>
          <h4 className="text-xs font-semibold text-gray-500 uppercase mb-2">Time</h4>
          <div className="grid grid-cols-2 gap-2">
            <MetricCard
              label="Simulation Time"
              value={formatTime(stats.simulationTime)}
            />
            <MetricCard
              label="Wall Clock"
              value={formatTime(stats.wallClockTime)}
            />
            <MetricCard
              label="Delta Time"
              value={stats.deltaTime.toFixed(4)}
              unit="s"
            />
            <MetricCard
              label="Realtime Ratio"
              value={stats.realtimeRatio.toFixed(2)}
              unit="x"
              variant={realtimeHealth}
            />
          </div>
        </div>

        {/* Performance Section */}
        <div>
          <h4 className="text-xs font-semibold text-gray-500 uppercase mb-2">
            Performance
          </h4>
          <div className="grid grid-cols-2 gap-2">
            <MetricCard
              label="Frame Rate"
              value={stats.frameRate.toFixed(1)}
              unit="Hz"
              variant={frameRateHealth}
            />
            <MetricCard
              label="Physics Time"
              value={stats.physicsTime.toFixed(2)}
              unit="ms"
              variant={physicsTimeHealth}
            />
            <MetricCard
              label="Render Time"
              value={stats.renderTime.toFixed(2)}
              unit="ms"
            />
            <MetricCard
              label="Collision Checks"
              value={stats.collisionChecks.toLocaleString()}
            />
          </div>
        </div>

        {/* Entities Section */}
        <div>
          <h4 className="text-xs font-semibold text-gray-500 uppercase mb-2">
            Entities
          </h4>
          <div className="grid grid-cols-2 gap-2">
            <MetricCard
              label="Total"
              value={stats.totalEntities.toLocaleString()}
            />
            <MetricCard
              label="Active"
              value={stats.activeEntities.toLocaleString()}
            />
          </div>

          {/* Domain breakdown */}
          <div className="mt-3 space-y-2">
            {Object.entries(stats.entitiesByDomain).map(([domain, count]) => (
              <div key={domain} className="flex items-center justify-between">
                <div className="flex items-center gap-2">
                  <div
                    className={clsx('w-3 h-3 rounded-full', {
                      'bg-domain-air': domain === 'air',
                      'bg-domain-land': domain === 'land',
                      'bg-domain-sea': domain === 'sea',
                      'bg-domain-space': domain === 'space',
                    })}
                  />
                  <span className="text-xs text-gray-400 uppercase">{domain}</span>
                </div>
                <span className="text-sm font-medium text-gray-200">
                  {count.toLocaleString()}
                </span>
              </div>
            ))}
          </div>
        </div>

        {/* Physics Section */}
        <div>
          <h4 className="text-xs font-semibold text-gray-500 uppercase mb-2">
            Physics
          </h4>
          <div className="grid grid-cols-2 gap-2">
            <MetricCard
              label="Active Collisions"
              value={stats.activeCollisions.toLocaleString()}
            />
          </div>
        </div>
      </div>
    </div>
  );
};

export default TelemetryPanel;
