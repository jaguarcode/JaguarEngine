/**
 * AI Agent Control Panel
 *
 * Provides interface for configuring and controlling AI agents
 * that autonomously control simulation entities.
 */

import React, { useState } from 'react';
import clsx from 'clsx';
import { useAIAgentStore } from '@/stores/aiAgentStore';
import { useEntityStore, selectEntities } from '@/stores/entityStore';
import { useSimulationStore } from '@/stores/simulationStore';
import type { AIBehaviorMode, AIServiceConfig } from '@/types/aiAgent';
import { BEHAVIOR_MODE_INFO, DEFAULT_AI_SERVICE_CONFIG } from '@/types/aiAgent';

// ============================================================================
// Icons
// ============================================================================

const BrainIcon = () => (
  <svg className="w-4 h-4" fill="none" stroke="currentColor" viewBox="0 0 24 24">
    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2}
      d="M9.663 17h4.673M12 3v1m6.364 1.636l-.707.707M21 12h-1M4 12H3m3.343-5.657l-.707-.707m2.828 9.9a5 5 0 117.072 0l-.548.547A3.374 3.374 0 0014 18.469V19a2 2 0 11-4 0v-.531c0-.895-.356-1.754-.988-2.386l-.548-.547z" />
  </svg>
);

const PlayIcon = () => (
  <svg className="w-4 h-4" fill="currentColor" viewBox="0 0 20 20">
    <path fillRule="evenodd" d="M10 18a8 8 0 100-16 8 8 0 000 16zM9.555 7.168A1 1 0 008 8v4a1 1 0 001.555.832l3-2a1 1 0 000-1.664l-3-2z" clipRule="evenodd" />
  </svg>
);

const StopIcon = () => (
  <svg className="w-4 h-4" fill="currentColor" viewBox="0 0 20 20">
    <path fillRule="evenodd" d="M10 18a8 8 0 100-16 8 8 0 000 16zM8 7a1 1 0 00-1 1v4a1 1 0 001 1h4a1 1 0 001-1V8a1 1 0 00-1-1H8z" clipRule="evenodd" />
  </svg>
);

const SettingsIcon = () => (
  <svg className="w-4 h-4" fill="none" stroke="currentColor" viewBox="0 0 24 24">
    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2}
      d="M10.325 4.317c.426-1.756 2.924-1.756 3.35 0a1.724 1.724 0 002.573 1.066c1.543-.94 3.31.826 2.37 2.37a1.724 1.724 0 001.065 2.572c1.756.426 1.756 2.924 0 3.35a1.724 1.724 0 00-1.066 2.573c.94 1.543-.826 3.31-2.37 2.37a1.724 1.724 0 00-2.572 1.065c-.426 1.756-2.924 1.756-3.35 0a1.724 1.724 0 00-2.573-1.066c-1.543.94-3.31-.826-2.37-2.37a1.724 1.724 0 00-1.065-2.572c-1.756-.426-1.756-2.924 0-3.35a1.724 1.724 0 001.066-2.573c-.94-1.543.826-3.31 2.37-2.37.996.608 2.296.07 2.572-1.065z" />
    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M15 12a3 3 0 11-6 0 3 3 0 016 0z" />
  </svg>
);

const TrashIcon = () => (
  <svg className="w-4 h-4" fill="none" stroke="currentColor" viewBox="0 0 24 24">
    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2}
      d="M19 7l-.867 12.142A2 2 0 0116.138 21H7.862a2 2 0 01-1.995-1.858L5 7m5 4v6m4-6v6m1-10V4a1 1 0 00-1-1h-4a1 1 0 00-1 1v3M4 7h16" />
  </svg>
);

// TargetIcon reserved for future use with target selection

// ============================================================================
// API Configuration Modal
// ============================================================================

interface APIConfigModalProps {
  isOpen: boolean;
  onClose: () => void;
  onSave: (config: AIServiceConfig) => void;
  currentConfig: AIServiceConfig | null;
}

const APIConfigModal: React.FC<APIConfigModalProps> = ({
  isOpen,
  onClose,
  onSave,
  currentConfig,
}) => {
  const [apiKey, setApiKey] = useState(currentConfig?.apiKey || '');
  const [model, setModel] = useState(currentConfig?.model || DEFAULT_AI_SERVICE_CONFIG.model);
  const [showKey, setShowKey] = useState(false);

  if (!isOpen) return null;

  const handleSave = () => {
    if (!apiKey.trim()) {
      alert('API key is required');
      return;
    }
    onSave({
      apiKey: apiKey.trim(),
      model: model as AIServiceConfig['model'],
      maxTokens: DEFAULT_AI_SERVICE_CONFIG.maxTokens,
      temperature: DEFAULT_AI_SERVICE_CONFIG.temperature,
    });
    onClose();
  };

  return (
    <div className="fixed inset-0 bg-black/50 flex items-center justify-center z-50">
      <div className="bg-gray-800 rounded-lg p-6 w-full max-w-md mx-4">
        <h2 className="text-lg font-semibold text-white mb-4">AI Agent Configuration</h2>

        <div className="space-y-4">
          <div>
            <label className="block text-sm text-gray-400 mb-1">Anthropic API Key</label>
            <div className="relative">
              <input
                type={showKey ? 'text' : 'password'}
                value={apiKey}
                onChange={(e) => setApiKey(e.target.value)}
                placeholder="sk-ant-..."
                className="w-full bg-gray-700 border border-gray-600 rounded px-3 py-2 text-white text-sm pr-16"
              />
              <button
                type="button"
                onClick={() => setShowKey(!showKey)}
                className="absolute right-2 top-1/2 -translate-y-1/2 text-xs text-gray-400 hover:text-white"
              >
                {showKey ? 'Hide' : 'Show'}
              </button>
            </div>
            <p className="text-xs text-yellow-500 mt-1">
              Warning: API key will be used in browser. Use with caution.
            </p>
          </div>

          <div>
            <label className="block text-sm text-gray-400 mb-1">Model</label>
            <select
              value={model}
              onChange={(e) => setModel(e.target.value as AIServiceConfig['model'])}
              className="w-full bg-gray-700 border border-gray-600 rounded px-3 py-2 text-white text-sm"
            >
              <option value="claude-3-haiku-20240307">Claude 3 Haiku (Fast, Cheap)</option>
              <option value="claude-3-sonnet-20240229">Claude 3 Sonnet (Balanced)</option>
              <option value="claude-3-opus-20240229">Claude 3 Opus (Best)</option>
            </select>
          </div>
        </div>

        <div className="flex justify-end gap-3 mt-6 pt-4 border-t border-gray-700">
          <button
            onClick={onClose}
            className="px-4 py-2 text-sm text-gray-300 hover:text-white bg-gray-700 hover:bg-gray-600 rounded transition-colors"
          >
            Cancel
          </button>
          <button
            onClick={handleSave}
            className="px-4 py-2 bg-blue-600 hover:bg-blue-500 text-white text-sm rounded font-medium transition-colors"
          >
            Save Configuration
          </button>
        </div>
      </div>
    </div>
  );
};

// ============================================================================
// Agent Card Component
// ============================================================================

interface AgentCardProps {
  entityId: string;
  entityName: string;
  entityDomain: string;
  onRemove: () => void;
}

const AgentCard: React.FC<AgentCardProps> = ({
  entityId,
  entityName,
  entityDomain,
  onRemove,
}) => {
  const agent = useAIAgentStore((s) => s.agents.get(entityId));
  const entitiesMap = useEntityStore(selectEntities);
  const entities = Array.from(entitiesMap.values());
  const setAgentActive = useAIAgentStore((s) => s.setAgentActive);
  const setAgentBehavior = useAIAgentStore((s) => s.setAgentBehavior);
  const updateAgentConfig = useAIAgentStore((s) => s.updateAgentConfig);
  const globalEnabled = useAIAgentStore((s) => s.globalEnabled);

  const [showAdvanced, setShowAdvanced] = useState(false);

  if (!agent) return null;

  const { config, status, lastDecision, stats } = agent;
  const supportedModes = Object.entries(BEHAVIOR_MODE_INFO)
    .filter(([_, info]) => info.supportedDomains.includes(entityDomain as any))
    .map(([mode]) => mode as AIBehaviorMode);

  const targetableEntities = entities.filter((e) => e.id !== entityId);

  const statusColor = {
    idle: 'bg-gray-500',
    thinking: 'bg-yellow-500 animate-pulse',
    executing: 'bg-green-500',
    error: 'bg-red-500',
  }[status];

  return (
    <div className="bg-gray-800/50 rounded-lg p-3 border border-gray-700">
      {/* Header */}
      <div className="flex items-center justify-between mb-3">
        <div className="flex items-center gap-2">
          <div className={clsx('w-2 h-2 rounded-full', statusColor)} />
          <span className="text-sm font-medium text-white">{entityName}</span>
          <span className="text-xs text-gray-500">({entityDomain})</span>
        </div>
        <div className="flex items-center gap-1">
          <button
            onClick={() => setShowAdvanced(!showAdvanced)}
            className="p-1 text-gray-400 hover:text-white"
            title="Settings"
          >
            <SettingsIcon />
          </button>
          <button
            onClick={onRemove}
            className="p-1 text-gray-400 hover:text-red-400"
            title="Remove agent"
          >
            <TrashIcon />
          </button>
        </div>
      </div>

      {/* Behavior Mode */}
      <div className="mb-3">
        <label className="block text-xs text-gray-500 mb-1">Behavior Mode</label>
        <select
          value={config.behaviorMode}
          onChange={(e) => setAgentBehavior(entityId, e.target.value as AIBehaviorMode)}
          className="w-full bg-gray-700 border border-gray-600 rounded px-2 py-1 text-sm text-white"
        >
          {supportedModes.map((mode) => (
            <option key={mode} value={mode}>
              {BEHAVIOR_MODE_INFO[mode].name}
            </option>
          ))}
        </select>
        <p className="text-xs text-gray-500 mt-1">
          {BEHAVIOR_MODE_INFO[config.behaviorMode].description}
        </p>
      </div>

      {/* Target Selection (if mode requires it) */}
      {BEHAVIOR_MODE_INFO[config.behaviorMode].requiresTarget && (
        <div className="mb-3">
          <label className="block text-xs text-gray-500 mb-1">Target Entity</label>
          <select
            value={config.targetEntityId || ''}
            onChange={(e) => setAgentBehavior(entityId, config.behaviorMode, e.target.value || undefined)}
            className="w-full bg-gray-700 border border-gray-600 rounded px-2 py-1 text-sm text-white"
          >
            <option value="">Select target...</option>
            {targetableEntities.map((entity) => (
              <option key={entity.id} value={entity.id}>
                {entity.name} ({entity.domain})
              </option>
            ))}
          </select>
        </div>
      )}

      {/* Advanced Settings */}
      {showAdvanced && (
        <div className="mb-3 p-2 bg-gray-900/50 rounded space-y-2">
          <div className="flex items-center justify-between">
            <label className="text-xs text-gray-500">Update Rate (Hz)</label>
            <input
              type="number"
              min={0.1}
              max={5}
              step={0.1}
              value={config.updateFrequency}
              onChange={(e) => updateAgentConfig(entityId, { updateFrequency: parseFloat(e.target.value) })}
              className="w-16 bg-gray-700 border border-gray-600 rounded px-2 py-0.5 text-xs text-white"
            />
          </div>
          <div className="flex items-center justify-between">
            <label className="text-xs text-gray-500">Debug Mode</label>
            <input
              type="checkbox"
              checked={config.debugMode}
              onChange={(e) => updateAgentConfig(entityId, { debugMode: e.target.checked })}
              className="accent-primary-500"
            />
          </div>
        </div>
      )}

      {/* Activate/Deactivate */}
      <button
        onClick={() => setAgentActive(entityId, !config.isActive)}
        disabled={!globalEnabled}
        className={clsx(
          'w-full flex items-center justify-center gap-2 py-1.5 rounded text-sm font-medium transition-colors',
          config.isActive
            ? 'bg-red-600/20 text-red-400 hover:bg-red-600/30'
            : 'bg-green-600/20 text-green-400 hover:bg-green-600/30',
          !globalEnabled && 'opacity-50 cursor-not-allowed'
        )}
      >
        {config.isActive ? (
          <>
            <StopIcon /> Stop Agent
          </>
        ) : (
          <>
            <PlayIcon /> Start Agent
          </>
        )}
      </button>

      {/* Last Decision */}
      {lastDecision && config.debugMode && (
        <div className="mt-2 p-2 bg-gray-900/50 rounded text-xs">
          <div className="text-gray-500">Last Decision:</div>
          <div className="text-gray-300 truncate">{lastDecision.reasoning}</div>
          <div className="text-gray-500 mt-1">
            Confidence: {(lastDecision.confidence * 100).toFixed(0)}% |
            Action: {lastDecision.actionType}
          </div>
        </div>
      )}

      {/* Stats */}
      <div className="mt-2 flex justify-between text-xs text-gray-500">
        <span>Decisions: {stats.totalDecisions}</span>
        <span>Tokens: {stats.tokensUsed}</span>
      </div>
    </div>
  );
};

// ============================================================================
// Main Panel Component
// ============================================================================

export const AIAgentPanel: React.FC = () => {
  const [isConfigOpen, setIsConfigOpen] = useState(false);
  const [isExpanded, setIsExpanded] = useState(true);
  const [selectedEntityForAgent, setSelectedEntityForAgent] = useState('');

  // Store state
  const serviceConfig = useAIAgentStore((s) => s.serviceConfig);
  const isServiceConfigured = useAIAgentStore((s) => s.isServiceConfigured);
  const globalEnabled = useAIAgentStore((s) => s.globalEnabled);
  const agents = useAIAgentStore((s) => s.agents);
  const setServiceConfig = useAIAgentStore((s) => s.setServiceConfig);
  const setGlobalEnabled = useAIAgentStore((s) => s.setGlobalEnabled);
  const createAgent = useAIAgentStore((s) => s.createAgent);
  const removeAgent = useAIAgentStore((s) => s.removeAgent);

  // Entity state - use ALL entities, not filtered
  const entitiesMap = useEntityStore(selectEntities);
  const entities = Array.from(entitiesMap.values());
  const connected = useSimulationStore((s) => s.connected);

  // Entities without agents (show all active entities)
  const availableEntities = entities.filter((e) => e.isActive && !agents.has(e.id));

  const handleAddAgent = () => {
    if (selectedEntityForAgent) {
      createAgent(selectedEntityForAgent);
      setSelectedEntityForAgent('');
    }
  };

  const activeAgentCount = Array.from(agents.values()).filter(
    (a) => a.config.isActive
  ).length;

  return (
    <div className="panel">
      {/* Header */}
      <div
        className="panel-header cursor-pointer"
        onClick={() => setIsExpanded(!isExpanded)}
      >
        <div className="flex items-center gap-2">
          <BrainIcon />
          <h3 className="panel-title">AI Agents</h3>
          {activeAgentCount > 0 && (
            <span className="px-1.5 py-0.5 bg-green-600/20 text-green-400 text-xs rounded">
              {activeAgentCount} active
            </span>
          )}
        </div>
        <button
          onClick={(e) => {
            e.stopPropagation();
            setIsConfigOpen(true);
          }}
          className="p-1 text-gray-400 hover:text-white"
          title="API Configuration"
        >
          <SettingsIcon />
        </button>
      </div>

      {isExpanded && (
        <div className="p-3 space-y-3">
          {/* Configuration Status */}
          {!isServiceConfigured ? (
            <div className="text-center py-4">
              <p className="text-gray-400 text-sm mb-2">
                Configure API key to enable AI agents
              </p>
              <button
                onClick={() => setIsConfigOpen(true)}
                className="btn btn-primary text-sm"
              >
                Configure API
              </button>
            </div>
          ) : (
            <>
              {/* Global Enable/Disable */}
              <div className="flex items-center justify-between p-2 bg-gray-800/50 rounded">
                <span className="text-sm text-gray-300">AI Control</span>
                <button
                  onClick={() => setGlobalEnabled(!globalEnabled)}
                  disabled={!connected}
                  className={clsx(
                    'px-3 py-1 rounded text-sm font-medium transition-colors',
                    globalEnabled
                      ? 'bg-green-600 text-white'
                      : 'bg-gray-600 text-gray-300',
                    !connected && 'opacity-50 cursor-not-allowed'
                  )}
                >
                  {globalEnabled ? 'Enabled' : 'Disabled'}
                </button>
              </div>

              {/* Add Agent */}
              <div className="flex gap-2">
                <select
                  value={selectedEntityForAgent}
                  onChange={(e) => setSelectedEntityForAgent(e.target.value)}
                  className="flex-1 bg-gray-700 border border-gray-600 rounded px-2 py-1 text-sm text-white"
                >
                  <option value="">Select entity...</option>
                  {availableEntities.map((entity) => (
                    <option key={entity.id} value={entity.id}>
                      {entity.name} ({entity.domain})
                    </option>
                  ))}
                </select>
                <button
                  onClick={handleAddAgent}
                  disabled={!selectedEntityForAgent}
                  className="btn btn-secondary text-sm disabled:opacity-50"
                >
                  Add Agent
                </button>
              </div>

              {/* Agent List */}
              <div className="space-y-2 max-h-96 overflow-y-auto">
                {agents.size === 0 ? (
                  <p className="text-center text-gray-500 text-sm py-4">
                    No AI agents configured
                  </p>
                ) : (
                  Array.from(agents.entries()).map(([entityId, _agent]) => {
                    const entity = entities.find((e) => e.id === entityId);
                    return (
                      <AgentCard
                        key={entityId}
                        entityId={entityId}
                        entityName={entity?.name || entityId}
                        entityDomain={entity?.domain || 'unknown'}
                        onRemove={() => removeAgent(entityId)}
                      />
                    );
                  })
                )}
              </div>
            </>
          )}
        </div>
      )}

      {/* Config Modal */}
      <APIConfigModal
        isOpen={isConfigOpen}
        onClose={() => setIsConfigOpen(false)}
        onSave={setServiceConfig}
        currentConfig={serviceConfig}
      />
    </div>
  );
};

export default AIAgentPanel;
