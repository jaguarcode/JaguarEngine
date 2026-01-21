/**
 * AI Agent Types for Autonomous Entity Control
 *
 * Enables Claude/LLM to control simulation entities with
 * intelligent decision-making for pursuit, evasion, patrol, etc.
 */

import type { EntityState, EntityDomain } from './index';

// ============================================================================
// Behavior Modes
// ============================================================================

export type AIBehaviorMode =
  | 'idle'           // No active behavior, waiting for commands
  | 'pursuit'        // Track and intercept target entity
  | 'evasion'        // Avoid threats, maximize distance
  | 'patrol'         // Follow waypoint pattern
  | 'escort'         // Protect designated entity
  | 'intercept'      // Calculate optimal intercept course
  | 'defensive'      // React to incoming threats
  | 'reconnaissance' // Observe and report
  | 'custom';        // User-defined behavior via prompt

// ============================================================================
// AI Agent Configuration
// ============================================================================

export interface AIAgentConfig {
  /** Unique identifier for this AI agent instance */
  id: string;

  /** Entity ID being controlled by this agent */
  entityId: string;

  /** Current behavior mode */
  behaviorMode: AIBehaviorMode;

  /** Target entity ID (for pursuit, escort, etc.) */
  targetEntityId?: string;

  /** Patrol waypoints (for patrol mode) */
  waypoints?: Array<{
    latitude: number;
    longitude: number;
    altitude: number;
  }>;

  /** Custom behavior prompt (for custom mode) */
  customPrompt?: string;

  /** Decision update frequency in Hz (decisions per second) */
  updateFrequency: number;

  /** Maximum engagement range in meters */
  maxEngagementRange: number;

  /** Minimum safe altitude in meters */
  minAltitude: number;

  /** Maximum altitude in meters */
  maxAltitude: number;

  /** Whether agent is currently active */
  isActive: boolean;

  /** Whether to log decisions for debugging */
  debugMode: boolean;
}

// ============================================================================
// AI Decision Types
// ============================================================================

export type AIActionType =
  // Air domain
  | 'set_flight_controls'
  | 'set_autopilot'
  // Land domain
  | 'set_vehicle_controls'
  | 'set_vehicle_autopilot'
  // Sea domain
  | 'set_ship_controls'
  | 'set_ship_autopilot'
  // Space domain
  | 'set_space_controls'
  | 'set_space_autopilot'
  // No action
  | 'no_action';

export interface AIDecision {
  /** Timestamp of the decision */
  timestamp: number;

  /** Entity ID this decision applies to */
  entityId: string;

  /** Type of action to execute */
  actionType: AIActionType;

  /** Action parameters */
  params: Record<string, unknown>;

  /** AI's reasoning for this decision */
  reasoning: string;

  /** Confidence level (0-1) */
  confidence: number;

  /** Detected threats that influenced this decision */
  threats?: Array<{
    entityId: string;
    threatLevel: 'low' | 'medium' | 'high' | 'critical';
    distance: number;
  }>;
}

// ============================================================================
// AI Agent State
// ============================================================================

export interface AIAgentState {
  /** Agent configuration */
  config: AIAgentConfig;

  /** Current status */
  status: 'idle' | 'thinking' | 'executing' | 'error';

  /** Last decision made */
  lastDecision?: AIDecision;

  /** Decision history (for debugging) */
  decisionHistory: AIDecision[];

  /** Error message if status is 'error' */
  error?: string;

  /** Statistics */
  stats: {
    totalDecisions: number;
    successfulActions: number;
    failedActions: number;
    averageResponseTime: number;
    tokensUsed: number;
  };
}

// ============================================================================
// World Context for AI
// ============================================================================

export interface AIWorldContext {
  /** The entity being controlled */
  controlledEntity: EntityState;

  /** Target entity (if applicable) */
  targetEntity?: EntityState;

  /** Nearby entities within detection range */
  nearbyEntities: Array<EntityState & {
    distance: number;
    bearing: number;
    relativeAltitude: number;
    threatLevel?: 'low' | 'medium' | 'high' | 'critical';
  }>;

  /** Current simulation time */
  simulationTime: number;

  /** Time scale factor */
  timeScale: number;

  /** Agent's current mission/behavior */
  mission: {
    mode: AIBehaviorMode;
    objective: string;
    constraints: string[];
  };
}

// ============================================================================
// Claude API Types
// ============================================================================

export interface AIPromptMessage {
  role: 'user' | 'assistant' | 'system';
  content: string;
}

export interface AIServiceConfig {
  /** Anthropic API key */
  apiKey: string;

  /** Model to use (default: claude-3-haiku for speed) */
  model: 'claude-3-haiku-20240307' | 'claude-3-sonnet-20240229' | 'claude-3-opus-20240229';

  /** Maximum tokens for response */
  maxTokens: number;

  /** Temperature for response generation */
  temperature: number;

  /** Optional proxy URL to avoid CORS */
  proxyUrl?: string;
}

// ============================================================================
// Default Configurations
// ============================================================================

export const DEFAULT_AI_AGENT_CONFIG: Omit<AIAgentConfig, 'id' | 'entityId'> = {
  behaviorMode: 'idle',
  updateFrequency: 0.1, // 0.1 Hz = 1 decision per 10 seconds (to stay within rate limits)
  maxEngagementRange: 50000, // 50 km
  minAltitude: 100, // 100 m
  maxAltitude: 15000, // 15 km
  isActive: false,
  debugMode: false,
};

export const DEFAULT_AI_SERVICE_CONFIG: Omit<AIServiceConfig, 'apiKey'> = {
  model: 'claude-3-haiku-20240307',
  maxTokens: 256, // Reduced to minimize token usage and costs
  temperature: 0.3,
};

// ============================================================================
// Behavior Mode Descriptions
// ============================================================================

export const BEHAVIOR_MODE_INFO: Record<AIBehaviorMode, {
  name: string;
  description: string;
  requiresTarget: boolean;
  supportedDomains: EntityDomain[];
}> = {
  idle: {
    name: 'Idle',
    description: 'No active behavior, maintains current state',
    requiresTarget: false,
    supportedDomains: ['air', 'land', 'sea', 'space'],
  },
  pursuit: {
    name: 'Pursuit',
    description: 'Track and intercept designated target',
    requiresTarget: true,
    supportedDomains: ['air', 'land', 'sea'],
  },
  evasion: {
    name: 'Evasion',
    description: 'Avoid threats and maximize survival',
    requiresTarget: false,
    supportedDomains: ['air', 'land', 'sea'],
  },
  patrol: {
    name: 'Patrol',
    description: 'Follow designated waypoint pattern',
    requiresTarget: false,
    supportedDomains: ['air', 'land', 'sea'],
  },
  escort: {
    name: 'Escort',
    description: 'Protect designated friendly entity',
    requiresTarget: true,
    supportedDomains: ['air', 'land', 'sea'],
  },
  intercept: {
    name: 'Intercept',
    description: 'Calculate and execute optimal intercept course',
    requiresTarget: true,
    supportedDomains: ['air', 'sea'],
  },
  defensive: {
    name: 'Defensive',
    description: 'React to incoming threats automatically',
    requiresTarget: false,
    supportedDomains: ['air', 'land', 'sea'],
  },
  reconnaissance: {
    name: 'Reconnaissance',
    description: 'Observe area and report findings',
    requiresTarget: false,
    supportedDomains: ['air', 'space'],
  },
  custom: {
    name: 'Custom',
    description: 'User-defined behavior via custom prompt',
    requiresTarget: false,
    supportedDomains: ['air', 'land', 'sea', 'space'],
  },
};
