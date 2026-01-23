import React, { useMemo } from 'react';
import {
  LineChart,
  Line,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  ResponsiveContainer,
  AreaChart,
  Area,
} from 'recharts';
import clsx from 'clsx';

// Stores
import { usePhysicsDebugStore } from '@/stores/physicsDebugStore';

// Hooks
import { usePhysicsDebug } from '@/hooks/usePhysicsDebug';

//==============================================================================
// Number Formatting Utilities
//==============================================================================

/**
 * Format large numbers in a human-readable way
 * Uses SI prefixes for very large/small numbers
 */
const formatNumber = (value: number, precision: number = 2): string => {
  if (!Number.isFinite(value)) return 'N/A';

  const absValue = Math.abs(value);

  // Very small numbers - use exponential
  if (absValue > 0 && absValue < 0.01) {
    return value.toExponential(precision);
  }

  // SI prefixes for large numbers
  const siPrefixes = [
    { threshold: 1e15, suffix: 'P' },  // Peta
    { threshold: 1e12, suffix: 'T' },  // Tera
    { threshold: 1e9, suffix: 'G' },   // Giga
    { threshold: 1e6, suffix: 'M' },   // Mega
    { threshold: 1e3, suffix: 'k' },   // Kilo
  ];

  for (const { threshold, suffix } of siPrefixes) {
    if (absValue >= threshold) {
      return (value / threshold).toFixed(precision) + suffix;
    }
  }

  // Normal range numbers
  return value.toFixed(precision);
};

/**
 * Format number for compact display (shorter width)
 */
const formatCompact = (value: number): string => {
  if (!Number.isFinite(value)) return 'N/A';

  const absValue = Math.abs(value);

  if (absValue >= 1e12) {
    return (value / 1e12).toFixed(1) + 'T';
  } else if (absValue >= 1e9) {
    return (value / 1e9).toFixed(1) + 'G';
  } else if (absValue >= 1e6) {
    return (value / 1e6).toFixed(1) + 'M';
  } else if (absValue >= 1e3) {
    return (value / 1e3).toFixed(1) + 'k';
  } else if (absValue > 0 && absValue < 0.01) {
    return value.toExponential(1);
  }

  return value.toFixed(2);
};

//==============================================================================
// Metric Card Component
//==============================================================================

interface MetricCardProps {
  title: string;
  value: string | number;
  unit?: string;
  trend?: 'up' | 'down' | 'stable';
  trendValue?: string;
  color?: string;
  compact?: boolean;
}

const MetricCard: React.FC<MetricCardProps> = ({
  title,
  value,
  unit,
  trend,
  trendValue,
  color = 'primary',
  compact = false,
}) => {
  const colorClasses: Record<string, string> = {
    primary: 'border-primary-500/30 bg-primary-500/5',
    green: 'border-green-500/30 bg-green-500/5',
    yellow: 'border-yellow-500/30 bg-yellow-500/5',
    red: 'border-red-500/30 bg-red-500/5',
    blue: 'border-blue-500/30 bg-blue-500/5',
    purple: 'border-purple-500/30 bg-purple-500/5',
  };

  // Format the display value
  const displayValue = typeof value === 'number'
    ? (compact ? formatCompact(value) : formatNumber(value))
    : value;

  // Full tooltip for the value
  const fullValue = typeof value === 'number' ? value.toLocaleString() : value;

  return (
    <div
      className={clsx('p-2 rounded-lg border', colorClasses[color] || colorClasses.primary)}
      title={`${title}: ${fullValue}${unit ? ' ' + unit : ''}`}
    >
      <div className="text-[10px] text-gray-400 mb-0.5 leading-tight">{title}</div>
      <div className="flex items-baseline gap-1">
        <span className="text-sm font-semibold text-white leading-tight">
          {displayValue}
        </span>
        {unit && <span className="text-[10px] text-gray-500">{unit}</span>}
      </div>
      {trend && trendValue && (
        <div
          className={clsx(
            'text-[10px] mt-0.5 leading-tight',
            trend === 'up' && 'text-green-400',
            trend === 'down' && 'text-red-400',
            trend === 'stable' && 'text-gray-400'
          )}
        >
          {trend === 'up' && '↑'}
          {trend === 'down' && '↓'}
          {trend === 'stable' && '→'} {trendValue}
        </div>
      )}
    </div>
  );
};

//==============================================================================
// Energy Panel
//==============================================================================

const EnergyPanel: React.FC = () => {
  const energy = usePhysicsDebugStore((s) => s.energy);
  const energyHistory = usePhysicsDebugStore((s) => s.energyHistory);

  const chartData = useMemo(() => {
    const maxPoints = Math.max(
      energyHistory.kinetic.length,
      energyHistory.potential.length,
      energyHistory.total.length
    );

    return Array.from({ length: maxPoints }, (_, i) => ({
      time: energyHistory.total[i]?.time || i,
      kinetic: energyHistory.kinetic[i]?.value || 0,
      potential: energyHistory.potential[i]?.value || 0,
      total: energyHistory.total[i]?.value || 0,
    }));
  }, [energyHistory]);

  const driftColor =
    Math.abs(energy.energyDrift) < 0.1
      ? 'green'
      : Math.abs(energy.energyDrift) < 1
      ? 'yellow'
      : 'red';

  return (
    <div className="bg-gray-900/80 rounded-lg p-4 border border-gray-700/50">
      <h3 className="text-sm font-semibold text-white mb-3 flex items-center gap-2">
        <span className="w-2 h-2 bg-green-500 rounded-full" />
        Energy Conservation
      </h3>

      {/* Metrics */}
      <div className="grid grid-cols-2 gap-2 mb-4">
        <MetricCard title="Kinetic Energy" value={energy.kineticEnergy} unit="J" color="blue" />
        <MetricCard
          title="Potential Energy"
          value={energy.potentialEnergy}
          unit="J"
          color="purple"
        />
        <MetricCard title="Total Energy" value={energy.totalEnergy} unit="J" color="green" />
        <MetricCard
          title="Energy Drift"
          value={energy.energyDrift}
          unit="%"
          color={driftColor}
          trend={energy.energyDrift > 0 ? 'up' : energy.energyDrift < 0 ? 'down' : 'stable'}
          trendValue="from initial"
        />
      </div>

      {/* Chart */}
      {chartData.length > 1 && (
        <div className="h-40">
          <ResponsiveContainer width="100%" height="100%">
            <AreaChart data={chartData}>
              <CartesianGrid strokeDasharray="3 3" stroke="#333" />
              <XAxis
                dataKey="time"
                stroke="#666"
                fontSize={10}
                tickFormatter={(v) => `${v.toFixed(1)}s`}
              />
              <YAxis
                stroke="#666"
                fontSize={10}
                tickFormatter={(v) => formatCompact(v)}
                width={50}
              />
              <Tooltip
                contentStyle={{ backgroundColor: '#1a1a2e', border: '1px solid #333' }}
                labelStyle={{ color: '#fff' }}
              />
              <Area
                type="monotone"
                dataKey="kinetic"
                stroke="#3b82f6"
                fill="#3b82f6"
                fillOpacity={0.2}
                name="Kinetic"
              />
              <Area
                type="monotone"
                dataKey="potential"
                stroke="#a855f7"
                fill="#a855f7"
                fillOpacity={0.2}
                name="Potential"
              />
              <Line
                type="monotone"
                dataKey="total"
                stroke="#22c55e"
                strokeWidth={2}
                dot={false}
                name="Total"
              />
            </AreaChart>
          </ResponsiveContainer>
        </div>
      )}
    </div>
  );
};

//==============================================================================
// Momentum Panel
//==============================================================================

const MomentumPanel: React.FC = () => {
  const momentum = usePhysicsDebugStore((s) => s.momentum);
  const momentumHistory = usePhysicsDebugStore((s) => s.momentumHistory);

  const linearMag = Math.sqrt(
    momentum.linearMomentum.x ** 2 +
      momentum.linearMomentum.y ** 2 +
      momentum.linearMomentum.z ** 2
  );

  const angularMag = Math.sqrt(
    momentum.angularMomentum.x ** 2 +
      momentum.angularMomentum.y ** 2 +
      momentum.angularMomentum.z ** 2
  );

  const chartData = useMemo(() => {
    const maxPoints = Math.max(
      momentumHistory.linearMagnitude.length,
      momentumHistory.angularMagnitude.length
    );

    return Array.from({ length: maxPoints }, (_, i) => ({
      time: momentumHistory.linearMagnitude[i]?.time || i,
      linear: momentumHistory.linearMagnitude[i]?.value || 0,
      angular: momentumHistory.angularMagnitude[i]?.value || 0,
    }));
  }, [momentumHistory]);

  return (
    <div className="bg-gray-900/80 rounded-lg p-4 border border-gray-700/50">
      <h3 className="text-sm font-semibold text-white mb-3 flex items-center gap-2">
        <span className="w-2 h-2 bg-blue-500 rounded-full" />
        Momentum
      </h3>

      {/* Metrics */}
      <div className="grid grid-cols-2 gap-2 mb-4">
        <MetricCard title="Linear Momentum" value={linearMag} unit="kg·m/s" color="blue" />
        <MetricCard title="Angular Momentum" value={angularMag} unit="kg·m²/s" color="purple" />
        <MetricCard title="Total Mass" value={momentum.totalMass} unit="kg" color="green" />
        <div className="p-3 rounded-lg border border-gray-700/30 bg-gray-800/50 overflow-hidden">
          <div className="text-xs text-gray-400 mb-1">Center of Mass</div>
          <div className="text-xs text-white font-mono truncate" title={`(${momentum.centerOfMass.x}, ${momentum.centerOfMass.y}, ${momentum.centerOfMass.z})`}>
            ({formatCompact(momentum.centerOfMass.x)}, {formatCompact(momentum.centerOfMass.y)}, {formatCompact(momentum.centerOfMass.z)})
          </div>
        </div>
      </div>

      {/* Chart */}
      {chartData.length > 1 && (
        <div className="h-32">
          <ResponsiveContainer width="100%" height="100%">
            <LineChart data={chartData}>
              <CartesianGrid strokeDasharray="3 3" stroke="#333" />
              <XAxis
                dataKey="time"
                stroke="#666"
                fontSize={10}
                tickFormatter={(v) => `${v.toFixed(1)}s`}
              />
              <YAxis
                stroke="#666"
                fontSize={10}
                tickFormatter={(v) => formatCompact(v)}
                width={50}
              />
              <Tooltip
                contentStyle={{ backgroundColor: '#1a1a2e', border: '1px solid #333' }}
              />
              <Line
                type="monotone"
                dataKey="linear"
                stroke="#3b82f6"
                strokeWidth={2}
                dot={false}
                name="Linear"
              />
              <Line
                type="monotone"
                dataKey="angular"
                stroke="#a855f7"
                strokeWidth={2}
                dot={false}
                name="Angular"
              />
            </LineChart>
          </ResponsiveContainer>
        </div>
      )}
    </div>
  );
};

//==============================================================================
// Integrator Panel
//==============================================================================

const IntegratorPanel: React.FC = () => {
  const integrator = usePhysicsDebugStore((s) => s.integrator);

  const integratorInfo: Record<string, { name: string; description: string }> = {
    euler: { name: 'Euler', description: '1st order, basic, energy drift' },
    symplectic_euler: { name: 'Symplectic Euler', description: '1st order, energy preserving' },
    verlet: { name: 'Verlet', description: '2nd order, time-reversible' },
    rk4: { name: 'RK4', description: '4th order, accurate' },
    abm4: { name: 'ABM4', description: '4th order, predictor-corrector' },
    dormand_prince: { name: 'Dormand-Prince', description: '5th order, adaptive' },
    boris: { name: 'Boris', description: 'EM fields, phase-preserving' },
  };

  const info = integratorInfo[integrator.name] || {
    name: integrator.name,
    description: 'Unknown integrator',
  };

  return (
    <div className="bg-gray-900/80 rounded-lg p-4 border border-gray-700/50">
      <h3 className="text-sm font-semibold text-white mb-3 flex items-center gap-2">
        <span className="w-2 h-2 bg-yellow-500 rounded-full" />
        Integrator
      </h3>

      <div className="space-y-3">
        {/* Current integrator */}
        <div className="flex items-center justify-between">
          <span className="text-xs text-gray-400">Method</span>
          <span className="text-sm font-medium text-white">{info.name}</span>
        </div>

        <div className="text-xs text-gray-500">{info.description}</div>

        <div className="grid grid-cols-2 gap-2">
          <div className="p-2 bg-gray-800/50 rounded">
            <div className="text-xs text-gray-400">Order</div>
            <div className="text-lg font-semibold text-white">{integrator.order}</div>
          </div>
          <div className="p-2 bg-gray-800/50 rounded">
            <div className="text-xs text-gray-400">Timestep</div>
            <div className="text-lg font-semibold text-white">
              {(integrator.currentTimestep * 1000).toFixed(1)}
              <span className="text-xs text-gray-500 ml-1">ms</span>
            </div>
          </div>
        </div>

        {/* Adaptive timestep info */}
        {integrator.adaptiveTimestep && (
          <div className="border-t border-gray-700/50 pt-3">
            <div className="flex items-center justify-between mb-2">
              <span className="text-xs text-gray-400">Adaptive Timestep</span>
              <span className="text-xs text-green-400">Active</span>
            </div>
            {integrator.errorEstimate !== undefined && (
              <div className="flex items-center justify-between">
                <span className="text-xs text-gray-400">Error Estimate</span>
                <span className="text-xs text-white">
                  {integrator.errorEstimate.toExponential(2)}
                </span>
              </div>
            )}
            {integrator.acceptedSteps !== undefined && integrator.rejectedSteps !== undefined && (
              <div className="flex items-center justify-between mt-1">
                <span className="text-xs text-gray-400">Accepted/Rejected</span>
                <span className="text-xs text-white">
                  {integrator.acceptedSteps}/{integrator.rejectedSteps}
                </span>
              </div>
            )}
          </div>
        )}
      </div>
    </div>
  );
};

//==============================================================================
// Collision Stats Panel
//==============================================================================

const CollisionStatsPanel: React.FC = () => {
  const contacts = usePhysicsDebugStore((s) => s.contacts);
  const broadPhase = usePhysicsDebugStore((s) => s.broadPhasePairsChecked);
  const narrowPhase = usePhysicsDebugStore((s) => s.narrowPhasePairsChecked);

  const efficiency =
    broadPhase > 0 ? ((1 - narrowPhase / broadPhase) * 100).toFixed(1) : '0';

  return (
    <div className="bg-gray-900/80 rounded-lg p-3 border border-gray-700/50">
      <h3 className="text-xs font-semibold text-white mb-2 flex items-center gap-2">
        <span className="w-2 h-2 bg-red-500 rounded-full" />
        Collision
      </h3>

      <div className="grid grid-cols-2 gap-1.5">
        <MetricCard title="Contacts" value={contacts.length} color="red" compact />
        <MetricCard title="Broad" value={broadPhase} color="yellow" compact />
        <MetricCard title="Narrow" value={narrowPhase} color="blue" compact />
        <MetricCard title="Cull %" value={efficiency} unit="%" color="green" compact />
      </div>
    </div>
  );
};

//==============================================================================
// Constraint Stats Panel
//==============================================================================

const ConstraintStatsPanel: React.FC = () => {
  const constraints = usePhysicsDebugStore((s) => s.constraints);
  const iterations = usePhysicsDebugStore((s) => s.constraintIterations);
  const error = usePhysicsDebugStore((s) => s.constraintError);

  const activeConstraints = constraints.filter((c) => c.enabled).length;

  return (
    <div className="bg-gray-900/80 rounded-lg p-3 border border-gray-700/50">
      <h3 className="text-xs font-semibold text-white mb-2 flex items-center gap-2">
        <span className="w-2 h-2 bg-purple-500 rounded-full" />
        Constraints
      </h3>

      <div className="grid grid-cols-2 gap-1.5">
        <MetricCard title="Active" value={activeConstraints} color="purple" compact />
        <MetricCard title="Total" value={constraints.length} color="blue" compact />
        <MetricCard title="Iters" value={iterations} color="yellow" compact />
        <MetricCard
          title="Error"
          value={error.toExponential(1)}
          color={error < 0.001 ? 'green' : error < 0.01 ? 'yellow' : 'red'}
          compact
        />
      </div>
    </div>
  );
};

//==============================================================================
// Main Physics Dashboard
//==============================================================================

interface PhysicsDashboardProps {
  className?: string;
  compact?: boolean;
}

export const PhysicsDashboard: React.FC<PhysicsDashboardProps> = ({
  className = '',
  compact = false,
}) => {
  const { debugEnabled, debugStreamConnected, toggleDebug, isConnected } = usePhysicsDebug();

  return (
    <div className={clsx('space-y-4', className)}>
      {/* Header with toggle */}
      <div className="flex items-center justify-between">
        <div className="flex items-center gap-2">
          <h2 className="text-lg font-semibold text-white">Physics Debug</h2>
          {debugEnabled && (
            <span
              className={clsx(
                'w-2 h-2 rounded-full',
                debugStreamConnected ? 'bg-green-500 animate-pulse' : 'bg-yellow-500'
              )}
              title={debugStreamConnected ? 'Stream connected' : 'Waiting for data...'}
            />
          )}
        </div>
        <button
          onClick={toggleDebug}
          disabled={!isConnected}
          className={clsx(
            'px-3 py-1.5 rounded-lg text-sm font-medium transition-colors',
            !isConnected && 'opacity-50 cursor-not-allowed',
            debugEnabled
              ? 'bg-green-500/20 text-green-400 border border-green-500/30'
              : 'bg-gray-700/50 text-gray-400 border border-gray-600/30'
          )}
          title={!isConnected ? 'Connect to server first' : undefined}
        >
          {debugEnabled ? 'Debug ON' : 'Debug OFF'}
        </button>
      </div>

      {debugEnabled && (
        <>
          {/* Energy & Momentum */}
          <EnergyPanel />
          {!compact && <MomentumPanel />}

          {/* Integrator */}
          <IntegratorPanel />

          {/* Collision & Constraints */}
          <div className={clsx(!compact && 'grid grid-cols-2 gap-4')}>
            <CollisionStatsPanel />
            {!compact && <ConstraintStatsPanel />}
          </div>
        </>
      )}
    </div>
  );
};

export default PhysicsDashboard;
