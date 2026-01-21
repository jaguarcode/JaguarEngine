import React, { useState, useCallback } from 'react';
import clsx from 'clsx';
import { useEntityStore, selectSelectedEntity } from '@/stores/entityStore';
import { useAIAgentStore, selectAgentByEntityId } from '@/stores/aiAgentStore';
import { commandService } from '@/services/commandService';
import type { AircraftState, GroundVehicleState, ShipState, SpacecraftState } from '@/types';
import type { AIAgentState } from '@/types/aiAgent';

// Helper to format numbers
const formatNum = (value: number, decimals = 2): string => {
  return value.toFixed(decimals);
};

// Property row component
const PropertyRow: React.FC<{
  label: string;
  value: string | number;
  unit?: string;
}> = ({ label, value, unit }) => (
  <div className="flex items-center justify-between py-1">
    <span className="text-xs text-gray-500">{label}</span>
    <span className="text-sm text-gray-200 font-mono">
      {typeof value === 'number' ? formatNum(value) : value}
      {unit && <span className="text-gray-500 ml-1">{unit}</span>}
    </span>
  </div>
);

// Section component
const Section: React.FC<{
  title: string;
  children: React.ReactNode;
}> = ({ title, children }) => (
  <div className="mb-4">
    <h4 className="text-xs font-semibold text-gray-500 uppercase mb-2">{title}</h4>
    <div className="space-y-0.5">{children}</div>
  </div>
);

// Aircraft details
const AircraftDetails: React.FC<{ entity: AircraftState }> = ({ entity }) => (
  <>
    <Section title="Flight Parameters">
      <PropertyRow label="Airspeed (TAS)" value={entity.airspeed} unit="m/s" />
      <PropertyRow label="Mach" value={entity.machNumber} />
      <PropertyRow label="Altitude" value={entity.altitude} unit="m MSL" />
      <PropertyRow label="Vertical Speed" value={entity.verticalSpeed} unit="m/s" />
      <PropertyRow label="Heading" value={entity.heading} unit="°" />
    </Section>

    <Section title="Flight Dynamics">
      <PropertyRow label="AoA" value={entity.aoa} unit="°" />
      <PropertyRow label="Sideslip" value={entity.sideslip} unit="°" />
      <PropertyRow label="G-Load" value={entity.gForce} unit="G" />
    </Section>

    <Section title="Controls">
      <PropertyRow label="Elevator" value={entity.elevator} />
      <PropertyRow label="Aileron" value={entity.aileron} />
      <PropertyRow label="Rudder" value={entity.rudder} />
      <PropertyRow label="Throttle" value={(entity.throttle * 100).toFixed(0)} unit="%" />
    </Section>

    <Section title="Systems">
      <PropertyRow label="Fuel" value={entity.fuel.toFixed(0)} unit="%" />
      <PropertyRow label="Gear" value={entity.gearDown ? 'DOWN' : 'UP'} />
      <PropertyRow label="Flaps" value={entity.flapsPosition} unit="°" />
    </Section>
  </>
);

// Ground vehicle details
const VehicleDetails: React.FC<{ entity: GroundVehicleState }> = ({ entity }) => (
  <>
    <Section title="Movement">
      <PropertyRow label="Speed" value={entity.speed} unit="m/s" />
      <PropertyRow label="Heading" value={entity.heading} unit="°" />
    </Section>

    <Section title="Controls">
      <PropertyRow label="Throttle" value={(entity.throttle * 100).toFixed(0)} unit="%" />
      <PropertyRow label="Steering" value={entity.steering} />
      <PropertyRow label="Brake" value={(entity.brake * 100).toFixed(0)} unit="%" />
    </Section>

    <Section title="Engine">
      <PropertyRow label="RPM" value={entity.engineRPM.toFixed(0)} />
      <PropertyRow label="Gear" value={entity.gear} />
      <PropertyRow label="Fuel" value={entity.fuel.toFixed(0)} unit="%" />
    </Section>

    <Section title="Terrain">
      <PropertyRow label="Ground Contact" value={entity.groundContact ? 'Yes' : 'No'} />
      <PropertyRow label="Sinkage" value={entity.sinkage * 100} unit="cm" />
    </Section>
  </>
);

// Ship details
const ShipDetails: React.FC<{ entity: ShipState }> = ({ entity }) => (
  <>
    <Section title="Navigation">
      <PropertyRow label="Speed" value={entity.speed} unit="m/s" />
      <PropertyRow label="Heading" value={entity.heading} unit="°" />
    </Section>

    <Section title="Motion">
      <PropertyRow label="Roll" value={entity.roll} unit="°" />
      <PropertyRow label="Pitch" value={entity.pitch} unit="°" />
      <PropertyRow label="Heave" value={entity.heave} unit="m" />
    </Section>

    <Section title="Controls">
      <PropertyRow label="Rudder" value={entity.rudder} />
      <PropertyRow label="Throttle" value={(entity.throttle * 100).toFixed(0)} unit="%" />
    </Section>

    <Section title="Hull">
      <PropertyRow label="Draft" value={entity.draft} unit="m" />
      <PropertyRow label="Displacement" value={entity.displacement} unit="t" />
    </Section>

    <Section title="Sea State">
      <PropertyRow label="Wave Height" value={entity.waveHeight} unit="m" />
      <PropertyRow label="Sea State" value={entity.seaState} />
    </Section>
  </>
);

// AI Decision details
const AIDecisionDetails: React.FC<{ agent: AIAgentState }> = ({ agent }) => {
  const { lastDecision, status, config, stats, error } = agent;

  // Status indicator colors
  const statusColors: Record<AIAgentState['status'], string> = {
    idle: 'text-gray-400',
    thinking: 'text-yellow-400',
    executing: 'text-blue-400',
    error: 'text-red-400',
  };

  const statusLabels: Record<AIAgentState['status'], string> = {
    idle: 'Idle',
    thinking: 'Thinking...',
    executing: 'Executing',
    error: 'Error',
  };

  // Format action parameters for display
  const formatParams = (params: Record<string, unknown>): React.ReactNode => {
    return Object.entries(params).map(([key, value]) => (
      <div key={key} className="text-xs font-mono text-gray-400">
        {key}: {typeof value === 'number' ? value.toFixed(3) : String(value)}
      </div>
    ));
  };

  return (
    <>
      <Section title="AI Agent">
        <div className="flex items-center justify-between py-1">
          <span className="text-xs text-gray-500">Status</span>
          <span className={clsx('text-sm font-medium', statusColors[status])}>
            {status === 'thinking' && <span className="inline-block animate-pulse mr-1">●</span>}
            {statusLabels[status]}
          </span>
        </div>
        <PropertyRow label="Mode" value={config.behaviorMode.toUpperCase()} />
        <PropertyRow label="Active" value={config.isActive ? 'Yes' : 'No'} />
        <PropertyRow label="Update Rate" value={config.updateFrequency} unit="Hz" />
        {config.targetEntityId && (
          <PropertyRow label="Target" value={config.targetEntityId} />
        )}
      </Section>

      {error && (
        <Section title="Error">
          <div className="text-xs text-red-400 break-words">{error}</div>
        </Section>
      )}

      {lastDecision && (
        <Section title="Last Decision">
          <div className="space-y-2">
            <div className="flex items-center justify-between">
              <span className="text-xs text-gray-500">Action</span>
              <span className="text-sm text-cyan-400 font-mono">
                {lastDecision.actionType.replace(/_/g, ' ')}
              </span>
            </div>

            <div className="flex items-center justify-between">
              <span className="text-xs text-gray-500">Confidence</span>
              <div className="flex items-center gap-2">
                <div className="w-16 h-1.5 bg-gray-700 rounded-full overflow-hidden">
                  <div
                    className={clsx('h-full rounded-full', {
                      'bg-green-500': lastDecision.confidence >= 0.7,
                      'bg-yellow-500': lastDecision.confidence >= 0.4 && lastDecision.confidence < 0.7,
                      'bg-red-500': lastDecision.confidence < 0.4,
                    })}
                    style={{ width: `${lastDecision.confidence * 100}%` }}
                  />
                </div>
                <span className="text-sm font-mono text-gray-300">
                  {(lastDecision.confidence * 100).toFixed(0)}%
                </span>
              </div>
            </div>

            {lastDecision.actionType !== 'no_action' && Object.keys(lastDecision.params).length > 0 && (
              <div>
                <span className="text-xs text-gray-500 block mb-1">Parameters</span>
                <div className="bg-gray-800/50 rounded px-2 py-1.5">
                  {formatParams(lastDecision.params)}
                </div>
              </div>
            )}

            <div>
              <span className="text-xs text-gray-500 block mb-1">Reasoning</span>
              <div className="text-xs text-gray-300 bg-gray-800/50 rounded px-2 py-1.5 italic">
                "{lastDecision.reasoning}"
              </div>
            </div>

            <div className="flex items-center justify-between text-xs text-gray-500">
              <span>Time</span>
              <span className="font-mono">
                {new Date(lastDecision.timestamp).toLocaleTimeString()}
              </span>
            </div>
          </div>
        </Section>
      )}

      <Section title="AI Statistics">
        <PropertyRow label="Total Decisions" value={stats.totalDecisions} />
        <PropertyRow label="Successful" value={stats.successfulActions} />
        <PropertyRow label="Failed" value={stats.failedActions} />
        <PropertyRow label="Avg Response" value={stats.averageResponseTime.toFixed(0)} unit="ms" />
        <PropertyRow label="Tokens Used" value={stats.tokensUsed} />
      </Section>
    </>
  );
};

// AI Command Input component
const AICommandInput: React.FC<{ entityId: string }> = ({ entityId }) => {
  const [command, setCommand] = useState('');
  const [feedback, setFeedback] = useState<{ type: 'success' | 'error'; message: string } | null>(null);
  const [showExamples, setShowExamples] = useState(false);
  const updateAgentConfig = useAIAgentStore((s) => s.updateAgentConfig);
  const setAgentBehavior = useAIAgentStore((s) => s.setAgentBehavior);
  const entityList = useEntityStore((s) => s.entityList);

  const handleSubmit = useCallback((e: React.FormEvent) => {
    e.preventDefault();

    const parsed = commandService.parseCommand(command, entityList);

    if (parsed.success) {
      // Update the agent config
      if (parsed.behavior) {
        setAgentBehavior(entityId, parsed.behavior, parsed.targetEntityId);
      }

      if (parsed.waypoints || parsed.customPrompt) {
        updateAgentConfig(entityId, {
          waypoints: parsed.waypoints,
          customPrompt: parsed.customPrompt,
        });
      }

      setFeedback({ type: 'success', message: parsed.description });
      setCommand('');

      // Clear feedback after 3 seconds
      setTimeout(() => setFeedback(null), 3000);
    } else {
      setFeedback({ type: 'error', message: parsed.error || 'Unknown error' });
    }
  }, [command, entityId, entityList, setAgentBehavior, updateAgentConfig]);

  const handleExampleClick = useCallback((example: string) => {
    setCommand(example);
    setShowExamples(false);
  }, []);

  const examples = commandService.getExampleCommands();

  return (
    <Section title="Command">
      <form onSubmit={handleSubmit} className="space-y-2">
        <div className="relative">
          <input
            type="text"
            value={command}
            onChange={(e) => setCommand(e.target.value)}
            placeholder="e.g., fly to Seoul, patrol Busan..."
            className="w-full bg-gray-800 border border-gray-700 rounded px-3 py-2 text-sm text-gray-200 placeholder-gray-500 focus:outline-none focus:border-cyan-500"
          />
          <button
            type="button"
            onClick={() => setShowExamples(!showExamples)}
            className="absolute right-2 top-1/2 -translate-y-1/2 text-gray-500 hover:text-gray-300 text-xs"
            title="Show examples"
          >
            ?
          </button>
        </div>

        {showExamples && (
          <div className="bg-gray-800/80 rounded p-2 space-y-1">
            <div className="text-xs text-gray-500 mb-1">Examples:</div>
            {examples.map((ex, i) => (
              <button
                key={i}
                type="button"
                onClick={() => handleExampleClick(ex)}
                className="block w-full text-left text-xs text-cyan-400 hover:text-cyan-300 py-0.5"
              >
                {ex}
              </button>
            ))}
          </div>
        )}

        <button
          type="submit"
          disabled={!command.trim()}
          className={clsx(
            'w-full py-1.5 rounded text-sm font-medium transition-colors',
            command.trim()
              ? 'bg-cyan-600 hover:bg-cyan-500 text-white'
              : 'bg-gray-700 text-gray-500 cursor-not-allowed'
          )}
        >
          Send Command
        </button>

        {feedback && (
          <div
            className={clsx(
              'text-xs px-2 py-1.5 rounded',
              feedback.type === 'success'
                ? 'bg-green-900/50 text-green-400'
                : 'bg-red-900/50 text-red-400'
            )}
          >
            {feedback.message}
          </div>
        )}
      </form>
    </Section>
  );
};

// Spacecraft details
const SpacecraftDetails: React.FC<{ entity: SpacecraftState }> = ({ entity }) => (
  <>
    <Section title="Orbital Elements">
      <PropertyRow label="Semi-Major Axis" value={entity.semiMajorAxis} unit="km" />
      <PropertyRow label="Eccentricity" value={entity.eccentricity} />
      <PropertyRow label="Inclination" value={entity.inclination} unit="°" />
      <PropertyRow label="RAAN" value={entity.raan} unit="°" />
      <PropertyRow label="Arg. of Perigee" value={entity.argOfPerigee} unit="°" />
      <PropertyRow label="True Anomaly" value={entity.trueAnomaly} unit="°" />
    </Section>

    <Section title="Derived">
      <PropertyRow label="Altitude" value={entity.altitude} unit="km" />
      <PropertyRow label="Velocity" value={entity.velocity} unit="km/s" />
      <PropertyRow label="Period" value={entity.period} unit="min" />
    </Section>

    <Section title="Systems">
      <PropertyRow label="Power" value={entity.power.toFixed(0)} unit="%" />
      <PropertyRow label="Fuel" value={entity.fuel.toFixed(0)} unit="%" />
    </Section>
  </>
);

export const EntityDetailPanel: React.FC = () => {
  const entity = useEntityStore(selectSelectedEntity);
  const selectedEntityId = useEntityStore((s) => s.selection.selectedEntityId);
  const agent = useAIAgentStore(selectAgentByEntityId(selectedEntityId || ''));

  if (!entity) {
    return (
      <div className="panel h-full flex items-center justify-center">
        <div className="text-center text-gray-500">
          <div className="text-4xl mb-2">📍</div>
          <div className="text-sm">Select an entity to view details</div>
        </div>
      </div>
    );
  }

  return (
    <div className="panel h-full flex flex-col">
      <div className="panel-header">
        <div className="flex items-center gap-2">
          <div
            className={clsx('status-dot', {
              'status-dot-active': entity.isActive,
              'status-dot-idle': !entity.isActive,
            })}
          />
          <h3 className="panel-title truncate">{entity.name}</h3>
        </div>
        <span
          className={clsx('domain-badge', {
            'domain-badge-air': entity.domain === 'air',
            'domain-badge-land': entity.domain === 'land',
            'domain-badge-sea': entity.domain === 'sea',
            'domain-badge-space': entity.domain === 'space',
          })}
        >
          {entity.domain}
        </span>
      </div>

      <div className="panel-content flex-1 overflow-y-auto custom-scrollbar">
        {/* Common info */}
        <Section title="Identification">
          <PropertyRow label="ID" value={entity.id} />
          <PropertyRow label="Name" value={entity.name} />
          <PropertyRow label="Kind" value={entity.kind} />
          {entity.identifier && (
            <PropertyRow
              label="Entity ID"
              value={`${entity.identifier.siteId}:${entity.identifier.applicationId}:${entity.identifier.entityNumber}`}
            />
          )}
        </Section>

        <Section title="Position">
          <PropertyRow label="Latitude" value={entity.position.latitude} unit="°" />
          <PropertyRow label="Longitude" value={entity.position.longitude} unit="°" />
          <PropertyRow label="Altitude" value={entity.position.altitude} unit="m" />
        </Section>

        <Section title="Velocity">
          <PropertyRow label="North" value={entity.velocity.north} unit="m/s" />
          <PropertyRow label="East" value={entity.velocity.east} unit="m/s" />
          <PropertyRow label="Down" value={entity.velocity.down} unit="m/s" />
        </Section>

        <Section title="Status">
          <PropertyRow label="Health" value={entity.health.toFixed(0)} unit="%" />
          <PropertyRow label="Damage" value={entity.damage.toFixed(0)} unit="%" />
        </Section>

        {/* AI Agent decision and command (if this entity has an AI agent) */}
        {agent && (
          <>
            <AIDecisionDetails agent={agent} />
            <AICommandInput entityId={entity.id} />
          </>
        )}

        {/* Domain-specific details */}
        {entity.domain === 'air' && entity.kind === 'platform' && (
          <AircraftDetails entity={entity as AircraftState} />
        )}
        {entity.domain === 'land' && entity.kind === 'platform' && (
          <VehicleDetails entity={entity as GroundVehicleState} />
        )}
        {entity.domain === 'sea' && entity.kind === 'platform' && (
          <ShipDetails entity={entity as ShipState} />
        )}
        {entity.domain === 'space' && (
          <SpacecraftDetails entity={entity as unknown as SpacecraftState} />
        )}

        {/* Custom properties */}
        {entity.properties && Object.keys(entity.properties).length > 0 && (
          <Section title="Properties">
            {Object.entries(entity.properties).map(([key, value]) => (
              <PropertyRow key={key} label={key} value={String(value)} />
            ))}
          </Section>
        )}
      </div>
    </div>
  );
};

export default EntityDetailPanel;
