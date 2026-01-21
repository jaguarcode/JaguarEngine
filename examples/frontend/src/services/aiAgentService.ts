/**
 * AI Agent Service
 *
 * Integrates with Claude API to provide intelligent decision-making
 * for autonomous entity control in the simulation.
 */

import type {
  AIServiceConfig,
  AIAgentConfig,
  AIDecision,
  AIWorldContext,
  AIBehaviorMode,
  AIActionType,
} from '@/types/aiAgent';
import type { EntityState, EntityDomain } from '@/types';

// ============================================================================
// System Prompts by Behavior Mode
// ============================================================================

const SYSTEM_PROMPT_BASE = `You are an AI pilot/commander controlling an entity in a multi-domain physics simulation.
Your decisions directly affect the entity's controls. Respond ONLY with valid JSON.

CRITICAL RULES:
1. Always respond with a valid JSON object
2. Never exceed physical limits of the entity
3. Prioritize survival if under threat
4. Consider fuel/endurance limitations
5. Maintain situational awareness

Response format:
{
  "actionType": "set_flight_controls" | "set_autopilot" | "set_vehicle_controls" | "set_ship_controls" | "no_action",
  "params": { ... action-specific parameters ... },
  "reasoning": "Brief explanation of decision",
  "confidence": 0.0 to 1.0
}`;

const BEHAVIOR_PROMPTS: Record<AIBehaviorMode, string> = {
  idle: `MISSION: Idle
Maintain current position and heading. Take no aggressive actions.
Only respond to immediate threats if survival is at risk.`,

  pursuit: `MISSION: Pursuit
Objective: Intercept and close distance to target entity.
- Calculate intercept course considering target's velocity
- Manage energy (altitude/speed trade-off)
- Maintain weapons engagement parameters
- If target evades, predict and cut off escape routes`,

  evasion: `MISSION: Evasion
Objective: Maximize survival by avoiding threats.
- Identify highest threat and prioritize escape vector
- Use terrain masking if available
- Perform defensive maneuvers (break turns, altitude changes)
- Maximize distance from pursuers while conserving energy`,

  patrol: `MISSION: Patrol
Objective: Follow assigned patrol route while maintaining awareness.
- Navigate between waypoints efficiently
- Scan for and report contacts
- Maintain optimal patrol altitude and speed
- React to threats by transitioning to defensive mode`,

  escort: `MISSION: Escort
Objective: Protect the designated friendly entity.
- Maintain optimal escort position relative to protected entity
- Interpose between threats and protected entity
- Engage threats that endanger the protected entity
- Match speed and course with protected entity`,

  intercept: `MISSION: Intercept
Objective: Calculate and execute optimal intercept course.
- Compute intercept geometry based on target velocity
- Optimize time-to-intercept
- Manage energy for weapons employment
- Adjust for target maneuvers`,

  defensive: `MISSION: Defensive
Objective: React to incoming threats automatically.
- Continuously scan for threats
- Prioritize threats by danger level and proximity
- Execute appropriate countermeasures
- Transition to evasion if overwhelmed`,

  reconnaissance: `MISSION: Reconnaissance
Objective: Observe and gather information.
- Maintain safe distance from potential threats
- Optimize sensor coverage of area of interest
- Avoid detection when possible
- Report significant observations`,

  custom: `MISSION: Custom
Follow the user-provided mission instructions below.`,
};

// ============================================================================
// Action Parameters by Domain
// ============================================================================

const DOMAIN_ACTIONS: Record<EntityDomain, {
  controlAction: AIActionType;
  parameterGuide: string;
}> = {
  air: {
    controlAction: 'set_flight_controls',
    parameterGuide: `Parameters for aircraft:
- elevator: -1.0 (pitch down) to 1.0 (pitch up)
- aileron: -1.0 (roll left) to 1.0 (roll right)
- rudder: -1.0 (yaw left) to 1.0 (yaw right)
- throttle: 0.0 (idle) to 1.0 (full power)

Or use set_autopilot with:
- mode: "altitude_hold" | "heading_hold" | "speed_hold" | "off"
- altitude: target altitude in meters
- heading: target heading in degrees
- speed: target speed in m/s`,
  },
  land: {
    controlAction: 'set_vehicle_controls',
    parameterGuide: `Parameters for ground vehicles:
- throttle: 0.0 (stop) to 1.0 (full speed)
- steering: -1.0 (left) to 1.0 (right)
- brake: 0.0 (none) to 1.0 (full brake)

Or use set_vehicle_autopilot with:
- mode: "manual" | "cruise_control" | "waypoint_follow" | "formation"
- speed: target speed in m/s
- heading: target heading in degrees`,
  },
  sea: {
    controlAction: 'set_ship_controls',
    parameterGuide: `Parameters for ships:
- throttle: -1.0 (full reverse) to 1.0 (full ahead)
- rudder: -1.0 (hard port) to 1.0 (hard starboard)

Or use set_ship_autopilot with:
- mode: "manual" | "autopilot" | "waypoint_nav" | "station_keeping"
- speed: target speed in m/s
- heading: target heading in degrees`,
  },
  space: {
    controlAction: 'set_space_controls',
    parameterGuide: `Parameters for spacecraft:
- thrust_x: -1.0 (left) to 1.0 (right) - lateral thrust
- thrust_y: -1.0 (down) to 1.0 (up) - vertical thrust
- thrust_z: -1.0 (backward) to 1.0 (forward) - main thrust
- roll_rate: target roll rate in deg/s
- pitch_rate: target pitch rate in deg/s
- yaw_rate: target yaw rate in deg/s

Or use set_space_autopilot with:
- mode: "drift" | "attitude_hold" | "orbit_maintain" | "maneuver"
- target_roll, target_pitch, target_yaw: target attitude in degrees`,
  },
};

// ============================================================================
// AI Agent Service Class
// ============================================================================

// Response type from Claude API with token usage
interface ClaudeAPIResponse {
  text: string;
  tokensUsed: number;
  inputTokens: number;
  outputTokens: number;
}

// Extended decision with token info (exported)
export interface AIDecisionWithTokens extends AIDecision {
  tokensUsed: number;
}

class AIAgentService {
  private config: AIServiceConfig | null = null;
  private abortControllers: Map<string, AbortController> = new Map();

  // Rate limiting state
  private rateLimitedUntil: number = 0;
  private consecutiveRateLimits: number = 0;
  private readonly MAX_BACKOFF_MS = 60000; // Max 1 minute backoff

  // Request queue to serialize API calls (prevents concurrent 529 errors)
  private requestQueue: Array<{
    resolve: (value: ClaudeAPIResponse) => void;
    reject: (error: Error) => void;
    systemPrompt: string;
    userPrompt: string;
    signal?: AbortSignal;
  }> = [];
  private isProcessingQueue: boolean = false;
  private readonly MIN_REQUEST_INTERVAL_MS = 500; // Minimum 500ms between requests
  private lastRequestTime: number = 0;

  /**
   * Configure the AI service with API credentials
   */
  configure(config: AIServiceConfig): void {
    this.config = config;
    // Reset rate limit state on reconfigure
    this.rateLimitedUntil = 0;
    this.consecutiveRateLimits = 0;
    console.log('[AIAgent] Service configured with model:', config.model);
  }

  /**
   * Check if we're currently rate limited
   */
  isRateLimited(): boolean {
    return Date.now() < this.rateLimitedUntil;
  }

  /**
   * Get remaining rate limit wait time in seconds
   */
  getRateLimitWaitTime(): number {
    const remaining = this.rateLimitedUntil - Date.now();
    return remaining > 0 ? Math.ceil(remaining / 1000) : 0;
  }

  /**
   * Check if service is configured
   */
  isConfigured(): boolean {
    return this.config !== null && !!this.config.apiKey;
  }

  /**
   * Generate a decision for an AI-controlled entity
   */
  async generateDecision(
    agentConfig: AIAgentConfig,
    worldContext: AIWorldContext
  ): Promise<AIDecisionWithTokens> {
    if (!this.config) {
      throw new Error('AI Agent service not configured');
    }

    // Check if we're rate limited
    if (this.isRateLimited()) {
      const waitTime = this.getRateLimitWaitTime();
      throw new Error(`Rate limited. Please wait ${waitTime} seconds.`);
    }

    // Build the prompt
    const systemPrompt = this.buildSystemPrompt(agentConfig, worldContext);
    const userPrompt = this.buildUserPrompt(worldContext);

    // Cancel any existing request for this entity
    this.cancelRequest(agentConfig.entityId);

    // Create new abort controller
    const abortController = new AbortController();
    this.abortControllers.set(agentConfig.entityId, abortController);

    try {
      const response = await this.callClaudeAPI(
        systemPrompt,
        userPrompt,
        abortController.signal
      );

      // Success - reset rate limit counter
      this.consecutiveRateLimits = 0;

      const decision = this.parseDecision(
        response.text,
        agentConfig.entityId,
        worldContext.controlledEntity.domain
      );

      decision.timestamp = Date.now();

      // Create decision with token info
      const decisionWithTokens: AIDecisionWithTokens = {
        ...decision,
        tokensUsed: response.tokensUsed,
      };

      if (agentConfig.debugMode) {
        console.log('[AIAgent] Decision for', agentConfig.entityId, decisionWithTokens, `(${response.tokensUsed} tokens)`);
      }

      return decisionWithTokens;
    } catch (error) {
      if ((error as Error).name === 'AbortError') {
        throw new Error('Request cancelled');
      }
      throw error;
    } finally {
      this.abortControllers.delete(agentConfig.entityId);
    }
  }

  /**
   * Cancel an ongoing request for an entity
   */
  cancelRequest(entityId: string): void {
    const controller = this.abortControllers.get(entityId);
    if (controller) {
      controller.abort();
      this.abortControllers.delete(entityId);
    }
  }

  /**
   * Cancel all ongoing requests
   */
  cancelAllRequests(): void {
    this.abortControllers.forEach((controller) => controller.abort());
    this.abortControllers.clear();
  }

  // ============================================================================
  // Private Methods
  // ============================================================================

  private buildSystemPrompt(
    agentConfig: AIAgentConfig,
    worldContext: AIWorldContext
  ): string {
    const domainInfo = DOMAIN_ACTIONS[worldContext.controlledEntity.domain];
    const behaviorPrompt = BEHAVIOR_PROMPTS[agentConfig.behaviorMode];

    let prompt = SYSTEM_PROMPT_BASE + '\n\n';
    prompt += `DOMAIN: ${worldContext.controlledEntity.domain.toUpperCase()}\n`;
    prompt += domainInfo.parameterGuide + '\n\n';
    prompt += behaviorPrompt;

    if (agentConfig.behaviorMode === 'custom' && agentConfig.customPrompt) {
      prompt += '\n\nCUSTOM INSTRUCTIONS:\n' + agentConfig.customPrompt;
    }

    prompt += `\n\nCONSTRAINTS:
- Minimum altitude: ${agentConfig.minAltitude}m
- Maximum altitude: ${agentConfig.maxAltitude}m
- Maximum engagement range: ${agentConfig.maxEngagementRange}m`;

    return prompt;
  }

  private buildUserPrompt(worldContext: AIWorldContext): string {
    const { controlledEntity, targetEntity, nearbyEntities, simulationTime } = worldContext;

    let prompt = `CURRENT SITUATION (T=${simulationTime.toFixed(1)}s):\n\n`;

    // Controlled entity state
    prompt += `YOUR ENTITY (${controlledEntity.name}):\n`;
    prompt += `- Position: ${controlledEntity.position.latitude.toFixed(4)}°, ${controlledEntity.position.longitude.toFixed(4)}°, ${controlledEntity.position.altitude.toFixed(0)}m\n`;
    prompt += `- Velocity: N${controlledEntity.velocity.north?.toFixed(1) ?? 0} E${controlledEntity.velocity.east?.toFixed(1) ?? 0} D${controlledEntity.velocity.down?.toFixed(1) ?? 0} m/s\n`;
    prompt += `- Orientation: Roll${controlledEntity.orientation.roll?.toFixed(1) ?? 0}° Pitch${controlledEntity.orientation.pitch?.toFixed(1) ?? 0}° Yaw${controlledEntity.orientation.yaw?.toFixed(1) ?? 0}°\n`;
    prompt += `- Health: ${controlledEntity.health}%\n`;

    if (controlledEntity.domain === 'air') {
      const airspeed = controlledEntity.properties?.airspeed as number | undefined;
      const heading = controlledEntity.properties?.heading as number | undefined;
      prompt += `- Airspeed: ${airspeed?.toFixed(1) ?? 0} m/s\n`;
      prompt += `- Heading: ${heading?.toFixed(1) ?? 0}°\n`;
    }

    // Target entity (if applicable)
    if (targetEntity) {
      prompt += `\nTARGET (${targetEntity.name}):\n`;
      prompt += `- Position: ${targetEntity.position.latitude.toFixed(4)}°, ${targetEntity.position.longitude.toFixed(4)}°, ${targetEntity.position.altitude.toFixed(0)}m\n`;
      prompt += `- Velocity: N${targetEntity.velocity.north?.toFixed(1) ?? 0} E${targetEntity.velocity.east?.toFixed(1) ?? 0} m/s\n`;
    }

    // Nearby entities
    if (nearbyEntities.length > 0) {
      prompt += `\nNEARBY CONTACTS (${nearbyEntities.length}):\n`;
      nearbyEntities.slice(0, 5).forEach((entity, i) => {
        const threat = entity.threatLevel ? ` [${entity.threatLevel.toUpperCase()}]` : '';
        prompt += `${i + 1}. ${entity.name} (${entity.domain})${threat}\n`;
        prompt += `   - Distance: ${(entity.distance / 1000).toFixed(1)}km, Bearing: ${entity.bearing.toFixed(0)}°\n`;
        prompt += `   - Alt diff: ${entity.relativeAltitude > 0 ? '+' : ''}${entity.relativeAltitude.toFixed(0)}m\n`;
      });
    }

    prompt += `\nProvide your decision as JSON:`;

    return prompt;
  }

  /**
   * Queue a request and process sequentially to avoid concurrent API calls
   * This prevents 529 overload errors when multiple AI agents are active
   */
  private async callClaudeAPI(
    systemPrompt: string,
    userPrompt: string,
    signal: AbortSignal
  ): Promise<ClaudeAPIResponse> {
    if (!this.config) {
      throw new Error('Service not configured');
    }

    // Add request to queue and return a promise
    return new Promise((resolve, reject) => {
      // Check if already aborted
      if (signal.aborted) {
        reject(new Error('Request cancelled'));
        return;
      }

      // Add abort listener to reject if cancelled while in queue
      const abortHandler = () => {
        const index = this.requestQueue.findIndex(r => r.resolve === resolve);
        if (index !== -1) {
          this.requestQueue.splice(index, 1);
          reject(new Error('Request cancelled'));
        }
      };
      signal.addEventListener('abort', abortHandler, { once: true });

      this.requestQueue.push({
        resolve: (value) => {
          signal.removeEventListener('abort', abortHandler);
          resolve(value);
        },
        reject: (error) => {
          signal.removeEventListener('abort', abortHandler);
          reject(error);
        },
        systemPrompt,
        userPrompt,
        signal,
      });

      // Start processing if not already
      this.processQueue();
    });
  }

  /**
   * Process queued requests one at a time with minimum interval
   */
  private async processQueue(): Promise<void> {
    if (this.isProcessingQueue || this.requestQueue.length === 0) {
      return;
    }

    this.isProcessingQueue = true;

    while (this.requestQueue.length > 0) {
      const request = this.requestQueue.shift()!;

      // Skip if aborted
      if (request.signal?.aborted) {
        request.reject(new Error('Request cancelled'));
        continue;
      }

      // Wait for rate limit to clear
      if (this.isRateLimited()) {
        const waitTime = this.rateLimitedUntil - Date.now();
        console.log(`[AIAgent] Queue waiting ${Math.ceil(waitTime / 1000)}s for rate limit...`);
        await new Promise(r => setTimeout(r, waitTime));
      }

      // Enforce minimum interval between requests
      const timeSinceLastRequest = Date.now() - this.lastRequestTime;
      if (timeSinceLastRequest < this.MIN_REQUEST_INTERVAL_MS) {
        await new Promise(r => setTimeout(r, this.MIN_REQUEST_INTERVAL_MS - timeSinceLastRequest));
      }

      try {
        const result = await this.executeAPICall(
          request.systemPrompt,
          request.userPrompt,
          request.signal
        );
        this.lastRequestTime = Date.now();
        request.resolve(result);
      } catch (error) {
        this.lastRequestTime = Date.now();
        request.reject(error as Error);
      }
    }

    this.isProcessingQueue = false;
  }

  /**
   * Execute the actual API call (internal, called by queue processor)
   */
  private async executeAPICall(
    systemPrompt: string,
    userPrompt: string,
    signal?: AbortSignal
  ): Promise<ClaudeAPIResponse> {
    const endpoint = this.config!.proxyUrl || 'https://api.anthropic.com/v1/messages';

    const response = await fetch(endpoint, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'x-api-key': this.config!.apiKey,
        'anthropic-version': '2023-06-01',
        'anthropic-dangerous-direct-browser-access': 'true',
      },
      body: JSON.stringify({
        model: this.config!.model,
        max_tokens: this.config!.maxTokens,
        temperature: this.config!.temperature,
        system: systemPrompt,
        messages: [
          {
            role: 'user',
            content: userPrompt,
          },
        ],
      }),
      signal,
    });

    if (!response.ok) {
      const error = await response.text();

      // Handle rate limiting (429)
      if (response.status === 429) {
        this.consecutiveRateLimits++;
        // Exponential backoff: 15s, 30s, 60s (capped)
        const backoffMs = Math.min(
          15000 * Math.pow(2, this.consecutiveRateLimits - 1),
          this.MAX_BACKOFF_MS
        );
        this.rateLimitedUntil = Date.now() + backoffMs;
        console.warn(`[AIAgent] Rate limited. Backing off for ${backoffMs / 1000}s`);
        throw new Error(`Rate limited. Retrying in ${Math.ceil(backoffMs / 1000)} seconds.`);
      }

      // Handle overloaded error (529)
      if (response.status === 529) {
        this.consecutiveRateLimits++;
        // Exponential backoff: 30s, 60s (capped) - longer for overload
        const backoffMs = Math.min(
          30000 * Math.pow(2, this.consecutiveRateLimits - 1),
          this.MAX_BACKOFF_MS
        );
        this.rateLimitedUntil = Date.now() + backoffMs;
        console.warn(`[AIAgent] API overloaded. Backing off for ${backoffMs / 1000}s`);
        throw new Error(`API overloaded. Retrying in ${Math.ceil(backoffMs / 1000)} seconds.`);
      }

      throw new Error(`Claude API error: ${response.status} - ${error}`);
    }

    // Success - reset consecutive rate limit counter
    this.consecutiveRateLimits = 0;

    const data = await response.json();

    // Extract token usage from response
    const inputTokens = data.usage?.input_tokens || 0;
    const outputTokens = data.usage?.output_tokens || 0;
    const tokensUsed = inputTokens + outputTokens;

    return {
      text: data.content[0].text,
      tokensUsed,
      inputTokens,
      outputTokens,
    };
  }

  private parseDecision(
    response: string,
    entityId: string,
    _domain: EntityDomain
  ): AIDecision {
    // Try to extract JSON from response
    let jsonStr = response;

    // Handle markdown code blocks
    const jsonMatch = response.match(/```(?:json)?\s*([\s\S]*?)```/);
    if (jsonMatch) {
      jsonStr = jsonMatch[1];
    }

    // Try to find JSON object in response
    const objectMatch = jsonStr.match(/\{[\s\S]*\}/);
    if (objectMatch) {
      jsonStr = objectMatch[0];
    }

    try {
      const parsed = JSON.parse(jsonStr);

      // Validate action type
      const validActions: AIActionType[] = [
        'set_flight_controls',
        'set_autopilot',
        'set_vehicle_controls',
        'set_ship_controls',
        'no_action',
      ];

      const actionType = validActions.includes(parsed.actionType)
        ? parsed.actionType
        : 'no_action';

      // Clamp control values to valid ranges
      const params = this.sanitizeParams(parsed.params || {}, actionType);

      return {
        timestamp: Date.now(),
        entityId,
        actionType,
        params,
        reasoning: parsed.reasoning || 'No reasoning provided',
        confidence: Math.max(0, Math.min(1, parsed.confidence || 0.5)),
      };
    } catch (e) {
      console.error('[AIAgent] Failed to parse decision:', e, response);

      // Return safe default
      return {
        timestamp: Date.now(),
        entityId,
        actionType: 'no_action',
        params: {},
        reasoning: 'Failed to parse AI response',
        confidence: 0,
      };
    }
  }

  private sanitizeParams(
    params: Record<string, unknown>,
    actionType: AIActionType
  ): Record<string, unknown> {
    const clamp = (val: unknown, min: number, max: number): number => {
      const num = typeof val === 'number' ? val : 0;
      return Math.max(min, Math.min(max, num));
    };

    switch (actionType) {
      case 'set_flight_controls':
        return {
          elevator: clamp(params.elevator, -1, 1),
          aileron: clamp(params.aileron, -1, 1),
          rudder: clamp(params.rudder, -1, 1),
          throttle: clamp(params.throttle, 0, 1),
        };

      case 'set_vehicle_controls':
        return {
          throttle: clamp(params.throttle, 0, 1),
          steering: clamp(params.steering, -1, 1),
          brake: clamp(params.brake, 0, 1),
        };

      case 'set_ship_controls':
        return {
          throttle: clamp(params.throttle, -1, 1),
          rudder: clamp(params.rudder, -1, 1),
        };

      case 'set_autopilot':
        return {
          mode: params.mode || 'off',
          altitude: typeof params.altitude === 'number' ? params.altitude : undefined,
          heading: typeof params.heading === 'number' ? params.heading : undefined,
          speed: typeof params.speed === 'number' ? params.speed : undefined,
        };

      default:
        return {};
    }
  }
}

// ============================================================================
// Singleton Export
// ============================================================================

export const aiAgentService = new AIAgentService();

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Calculate distance between two entities in meters
 */
export function calculateDistance(
  entity1: EntityState,
  entity2: EntityState
): number {
  const R = 6371000; // Earth radius in meters
  const lat1 = (entity1.position.latitude * Math.PI) / 180;
  const lat2 = (entity2.position.latitude * Math.PI) / 180;
  const dLat = lat2 - lat1;
  const dLon = ((entity2.position.longitude - entity1.position.longitude) * Math.PI) / 180;

  const a =
    Math.sin(dLat / 2) * Math.sin(dLat / 2) +
    Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) * Math.sin(dLon / 2);

  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
  const horizontalDistance = R * c;

  const altDiff = entity2.position.altitude - entity1.position.altitude;
  return Math.sqrt(horizontalDistance * horizontalDistance + altDiff * altDiff);
}

/**
 * Calculate bearing from entity1 to entity2 in degrees
 */
export function calculateBearing(
  entity1: EntityState,
  entity2: EntityState
): number {
  const lat1 = (entity1.position.latitude * Math.PI) / 180;
  const lat2 = (entity2.position.latitude * Math.PI) / 180;
  const dLon = ((entity2.position.longitude - entity1.position.longitude) * Math.PI) / 180;

  const y = Math.sin(dLon) * Math.cos(lat2);
  const x =
    Math.cos(lat1) * Math.sin(lat2) -
    Math.sin(lat1) * Math.cos(lat2) * Math.cos(dLon);

  const bearing = (Math.atan2(y, x) * 180) / Math.PI;
  return (bearing + 360) % 360;
}

/**
 * Build world context for AI decision making
 */
export function buildWorldContext(
  controlledEntity: EntityState,
  allEntities: EntityState[],
  agentConfig: AIAgentConfig,
  simulationTime: number,
  timeScale: number
): AIWorldContext {
  // Find target entity if specified
  const targetEntity = agentConfig.targetEntityId
    ? allEntities.find((e) => e.id === agentConfig.targetEntityId)
    : undefined;

  // Find nearby entities with distance and bearing
  const nearbyEntities = allEntities
    .filter((e) => e.id !== controlledEntity.id)
    .map((entity) => {
      const distance = calculateDistance(controlledEntity, entity);
      const bearing = calculateBearing(controlledEntity, entity);
      const relativeAltitude = entity.position.altitude - controlledEntity.position.altitude;

      // Simple threat assessment
      let threatLevel: 'low' | 'medium' | 'high' | 'critical' | undefined;
      if (entity.domain === controlledEntity.domain && distance < 10000) {
        if (distance < 2000) threatLevel = 'critical';
        else if (distance < 5000) threatLevel = 'high';
        else threatLevel = 'medium';
      } else if (distance < 20000) {
        threatLevel = 'low';
      }

      return {
        ...entity,
        distance,
        bearing,
        relativeAltitude,
        threatLevel,
      };
    })
    .filter((e) => e.distance <= agentConfig.maxEngagementRange)
    .sort((a, b) => a.distance - b.distance);

  // Build mission description
  const modeInfo = {
    idle: { objective: 'Maintain position', constraints: [] },
    pursuit: { objective: `Intercept target: ${targetEntity?.name || 'Unknown'}`, constraints: ['Maintain energy', 'Do not overshoot'] },
    evasion: { objective: 'Evade all threats', constraints: ['Maximize distance', 'Conserve fuel'] },
    patrol: { objective: 'Follow patrol route', constraints: ['Maintain awareness', 'Report contacts'] },
    escort: { objective: `Protect: ${targetEntity?.name || 'Unknown'}`, constraints: ['Stay close to escort', 'Engage threats'] },
    intercept: { objective: `Intercept: ${targetEntity?.name || 'Unknown'}`, constraints: ['Optimize time', 'Manage energy'] },
    defensive: { objective: 'Defend against threats', constraints: ['Prioritize survival', 'Counter threats'] },
    reconnaissance: { objective: 'Observe and report', constraints: ['Avoid detection', 'Maximize coverage'] },
    custom: { objective: 'Follow custom instructions', constraints: [] },
  };

  const mission = modeInfo[agentConfig.behaviorMode] || modeInfo.idle;

  return {
    controlledEntity,
    targetEntity,
    nearbyEntities,
    simulationTime,
    timeScale,
    mission: {
      mode: agentConfig.behaviorMode,
      objective: mission.objective,
      constraints: [
        ...mission.constraints,
        `Min altitude: ${agentConfig.minAltitude}m`,
        `Max altitude: ${agentConfig.maxAltitude}m`,
      ],
    },
  };
}
