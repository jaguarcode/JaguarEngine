/**
 * AI Agent Store
 *
 * Manages state for AI-controlled entities in the simulation.
 * Tracks agent configurations, decisions, and statistics.
 */

import { create } from 'zustand';
import { subscribeWithSelector } from 'zustand/middleware';
import type {
  AIAgentConfig,
  AIAgentState,
  AIDecision,
  AIBehaviorMode,
  AIServiceConfig,
} from '@/types/aiAgent';

// ============================================================================
// Store Interface
// ============================================================================

interface AIAgentStore {
  // Global AI settings
  serviceConfig: AIServiceConfig | null;
  isServiceConfigured: boolean;
  globalEnabled: boolean;

  // Agent states keyed by entity ID
  agents: Map<string, AIAgentState>;

  // Actions - Service Configuration
  setServiceConfig: (config: AIServiceConfig) => void;
  clearServiceConfig: () => void;
  setGlobalEnabled: (enabled: boolean) => void;

  // Actions - Agent Management
  createAgent: (entityId: string, config?: Partial<AIAgentConfig>) => void;
  removeAgent: (entityId: string) => void;
  clearAllAgents: () => void;
  updateAgentConfig: (entityId: string, config: Partial<AIAgentConfig>) => void;
  setAgentActive: (entityId: string, active: boolean) => void;
  setAgentBehavior: (entityId: string, mode: AIBehaviorMode, targetId?: string) => void;

  // Actions - Decision Recording
  recordDecision: (entityId: string, decision: AIDecision) => void;
  setAgentStatus: (entityId: string, status: AIAgentState['status'], error?: string) => void;
  updateAgentStats: (entityId: string, responseTime: number, tokensUsed: number, success: boolean) => void;
  clearDecisionHistory: (entityId: string) => void;

  // Selectors
  getAgent: (entityId: string) => AIAgentState | undefined;
  getActiveAgents: () => AIAgentState[];
  isEntityAIControlled: (entityId: string) => boolean;
}

// ============================================================================
// Default Values
// ============================================================================

const createDefaultAgentState = (entityId: string, config?: Partial<AIAgentConfig>): AIAgentState => ({
  config: {
    id: `ai-agent-${entityId}-${Date.now()}`,
    entityId,
    behaviorMode: 'idle',
    updateFrequency: 0.1, // 0.1 Hz = 1 decision per 10 seconds (to stay within rate limits)
    maxEngagementRange: 50000,
    minAltitude: 100,
    maxAltitude: 15000,
    isActive: false,
    debugMode: false,
    ...config,
  },
  status: 'idle',
  decisionHistory: [],
  stats: {
    totalDecisions: 0,
    successfulActions: 0,
    failedActions: 0,
    averageResponseTime: 0,
    tokensUsed: 0,
  },
});

// ============================================================================
// Store Implementation
// ============================================================================

export const useAIAgentStore = create<AIAgentStore>()(
  subscribeWithSelector((set, get) => ({
    // Initial state
    serviceConfig: null,
    isServiceConfigured: false,
    globalEnabled: false,
    agents: new Map(),

    // Service Configuration
    setServiceConfig: (config) => {
      set({
        serviceConfig: config,
        isServiceConfigured: true,
      });
    },

    clearServiceConfig: () => {
      set({
        serviceConfig: null,
        isServiceConfigured: false,
        globalEnabled: false,
      });
      // Deactivate all agents
      const agents = get().agents;
      agents.forEach((agent) => {
        agent.config.isActive = false;
        agent.status = 'idle';
      });
      set({ agents: new Map(agents) });
    },

    setGlobalEnabled: (enabled) => {
      set({ globalEnabled: enabled });
      if (!enabled) {
        // Pause all agents when globally disabled
        const agents = get().agents;
        agents.forEach((agent) => {
          agent.status = 'idle';
        });
        set({ agents: new Map(agents) });
      }
    },

    // Agent Management
    createAgent: (entityId, config) => {
      const agents = get().agents;
      if (agents.has(entityId)) {
        console.warn(`AI Agent already exists for entity ${entityId}`);
        return;
      }
      agents.set(entityId, createDefaultAgentState(entityId, config));
      set({ agents: new Map(agents) });
    },

    removeAgent: (entityId) => {
      const agents = get().agents;
      agents.delete(entityId);
      set({ agents: new Map(agents) });
    },

    clearAllAgents: () => {
      set({ agents: new Map(), globalEnabled: false });
    },

    updateAgentConfig: (entityId, config) => {
      const agents = get().agents;
      const agent = agents.get(entityId);
      if (!agent) {
        console.warn(`AI Agent not found for entity ${entityId}`);
        return;
      }
      agent.config = { ...agent.config, ...config };
      set({ agents: new Map(agents) });
    },

    setAgentActive: (entityId, active) => {
      const agents = get().agents;
      const agent = agents.get(entityId);
      if (!agent) {
        console.warn(`AI Agent not found for entity ${entityId}`);
        return;
      }
      agent.config.isActive = active;
      agent.status = active ? 'idle' : 'idle';
      set({ agents: new Map(agents) });
    },

    setAgentBehavior: (entityId, mode, targetId) => {
      const agents = get().agents;
      const agent = agents.get(entityId);
      if (!agent) {
        console.warn(`AI Agent not found for entity ${entityId}`);
        return;
      }
      agent.config.behaviorMode = mode;
      agent.config.targetEntityId = targetId;
      set({ agents: new Map(agents) });
    },

    // Decision Recording
    recordDecision: (entityId, decision) => {
      const agents = get().agents;
      const agent = agents.get(entityId);
      if (!agent) return;

      agent.lastDecision = decision;
      agent.decisionHistory.push(decision);

      // Keep only last 100 decisions
      if (agent.decisionHistory.length > 100) {
        agent.decisionHistory = agent.decisionHistory.slice(-100);
      }

      agent.stats.totalDecisions++;
      set({ agents: new Map(agents) });
    },

    setAgentStatus: (entityId, status, error) => {
      const agents = get().agents;
      const agent = agents.get(entityId);
      if (!agent) return;

      agent.status = status;
      agent.error = error;
      set({ agents: new Map(agents) });
    },

    updateAgentStats: (entityId, responseTime, tokensUsed, success) => {
      const agents = get().agents;
      const agent = agents.get(entityId);
      if (!agent) return;

      const stats = agent.stats;
      stats.tokensUsed += tokensUsed;

      if (success) {
        stats.successfulActions++;
      } else {
        stats.failedActions++;
      }

      // Running average for response time
      const totalActions = stats.successfulActions + stats.failedActions;
      stats.averageResponseTime =
        (stats.averageResponseTime * (totalActions - 1) + responseTime) / totalActions;

      set({ agents: new Map(agents) });
    },

    clearDecisionHistory: (entityId) => {
      const agents = get().agents;
      const agent = agents.get(entityId);
      if (!agent) return;

      agent.decisionHistory = [];
      set({ agents: new Map(agents) });
    },

    // Selectors
    getAgent: (entityId) => {
      return get().agents.get(entityId);
    },

    getActiveAgents: () => {
      const { agents, globalEnabled, isServiceConfigured } = get();
      if (!globalEnabled || !isServiceConfigured) return [];

      return Array.from(agents.values()).filter(
        (agent) => agent.config.isActive && agent.status !== 'error'
      );
    },

    isEntityAIControlled: (entityId) => {
      const { agents, globalEnabled, isServiceConfigured } = get();
      if (!globalEnabled || !isServiceConfigured) return false;

      const agent = agents.get(entityId);
      return agent?.config.isActive ?? false;
    },
  }))
);

// ============================================================================
// Selectors
// ============================================================================

export const selectServiceConfigured = (state: AIAgentStore) => state.isServiceConfigured;
export const selectGlobalEnabled = (state: AIAgentStore) => state.globalEnabled;
export const selectAgentCount = (state: AIAgentStore) => state.agents.size;
export const selectActiveAgentCount = (state: AIAgentStore) =>
  state.getActiveAgents().length;

export const selectAgentByEntityId = (entityId: string) => (state: AIAgentStore) =>
  state.agents.get(entityId);

export const selectIsEntityAIControlled = (entityId: string) => (state: AIAgentStore) =>
  state.isEntityAIControlled(entityId);
