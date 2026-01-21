/**
 * AI Agent Loop Hook
 *
 * Manages the decision loop for AI-controlled entities.
 * Observes world state, generates decisions via Claude API,
 * and executes actions via WebSocket commands.
 */

import { useEffect, useRef, useCallback } from 'react';
import { useAIAgentStore } from '@/stores/aiAgentStore';
import { useEntityStore } from '@/stores/entityStore';
import { useSimulationStore } from '@/stores/simulationStore';
import { useWebSocket } from './useWebSocket';
import {
  aiAgentService,
  buildWorldContext,
} from '@/services/aiAgentService';
import type { AIAgentState, AIDecision } from '@/types/aiAgent';

// ============================================================================
// Hook Implementation
// ============================================================================

export function useAIAgentLoop() {
  const loopRef = useRef<number | null>(null);
  const lastUpdateRef = useRef<Map<string, number>>(new Map());

  // Store state - use getState() pattern to avoid re-render loops
  const serviceConfig = useAIAgentStore((s) => s.serviceConfig);
  const isServiceConfigured = useAIAgentStore((s) => s.isServiceConfigured);
  const globalEnabled = useAIAgentStore((s) => s.globalEnabled);
  const getActiveAgents = useAIAgentStore((s) => s.getActiveAgents);
  const setAgentStatus = useAIAgentStore((s) => s.setAgentStatus);
  const recordDecision = useAIAgentStore((s) => s.recordDecision);
  const updateAgentStats = useAIAgentStore((s) => s.updateAgentStats);

  // Simulation status for controlling the loop
  const simStatus = useSimulationStore((s) => s.status);

  // WebSocket commands
  const {
    setFlightControls,
    setAutopilot,
    setVehicleControls,
    setVehicleAutopilot,
    setShipControls,
    setShipAutopilot,
    setSpaceControls,
    setSpaceAutopilot,
  } = useWebSocket();

  /**
   * Execute a decision by sending appropriate command
   */
  const executeDecision = useCallback(
    (decision: AIDecision) => {
      const { entityId, actionType, params } = decision;

      switch (actionType) {
        case 'set_flight_controls':
          setFlightControls(entityId, {
            elevator: params.elevator as number,
            aileron: params.aileron as number,
            rudder: params.rudder as number,
            throttle: params.throttle as number,
          });
          break;

        case 'set_autopilot':
          setAutopilot(entityId, params.mode as string, {
            altitude: params.altitude,
            heading: params.heading,
            speed: params.speed,
          });
          break;

        case 'set_vehicle_controls':
          setVehicleControls(entityId, {
            throttle: params.throttle as number,
            steering: params.steering as number,
            brake: params.brake as number,
          });
          break;

        case 'set_vehicle_autopilot':
          setVehicleAutopilot(entityId, params.mode as string, {
            speed: params.speed,
            heading: params.heading,
          });
          break;

        case 'set_ship_controls':
          setShipControls(entityId, {
            throttle: params.throttle as number,
            rudder: params.rudder as number,
          });
          break;

        case 'set_ship_autopilot':
          setShipAutopilot(entityId, params.mode as string, {
            speed: params.speed,
            heading: params.heading,
          });
          break;

        case 'set_space_controls':
          setSpaceControls(entityId, {
            thrust_x: params.thrust_x as number,
            thrust_y: params.thrust_y as number,
            thrust_z: params.thrust_z as number,
            roll_rate: params.roll_rate as number,
            pitch_rate: params.pitch_rate as number,
            yaw_rate: params.yaw_rate as number,
          });
          break;

        case 'set_space_autopilot':
          setSpaceAutopilot(entityId, params.mode as string, {
            roll: params.roll,
            pitch: params.pitch,
            yaw: params.yaw,
          });
          break;

        case 'no_action':
        default:
          // No action needed
          break;
      }
    },
    [
      setFlightControls,
      setAutopilot,
      setVehicleControls,
      setVehicleAutopilot,
      setShipControls,
      setShipAutopilot,
      setSpaceControls,
      setSpaceAutopilot,
    ]
  );

  /**
   * Process a single agent's decision
   */
  const processAgent = useCallback(
    async (agent: AIAgentState) => {
      const { config, status } = agent;
      const entityId = config.entityId;

      // Skip if agent is already processing (thinking or executing)
      if (status === 'thinking' || status === 'executing') {
        return; // Already processing, don't start another request
      }

      // Skip if service is rate limited (429 or 529 backoff)
      if (aiAgentService.isRateLimited()) {
        return; // Wait for rate limit to clear
      }

      // Check update timing
      const lastUpdate = lastUpdateRef.current.get(entityId) || 0;
      const updateInterval = 1000 / config.updateFrequency; // ms
      const now = Date.now();

      if (now - lastUpdate < updateInterval) {
        return; // Not time to update yet
      }

      // Get current entities from store (avoid subscription re-render loop)
      const entitiesMap = useEntityStore.getState().entities;
      const entities = Array.from(entitiesMap.values());

      // Find the controlled entity
      const controlledEntity = entities.find((e) => e.id === entityId);
      if (!controlledEntity) {
        console.warn(`[AIAgent] Entity ${entityId} not found`);
        return;
      }

      // Get simulation state
      const simState = useSimulationStore.getState();
      const simulationTime = simState.stats.simulationTime;
      const timeScale = simState.config.timeScale;

      // Set status to thinking
      setAgentStatus(entityId, 'thinking');
      lastUpdateRef.current.set(entityId, now);

      try {
        // Build world context
        const worldContext = buildWorldContext(
          controlledEntity,
          entities,
          config,
          simulationTime,
          timeScale
        );

        // Generate decision via Claude
        const startTime = Date.now();
        const decision = await aiAgentService.generateDecision(config, worldContext);
        const responseTime = Date.now() - startTime;

        // Record the decision
        recordDecision(entityId, decision);

        // Execute the decision
        setAgentStatus(entityId, 'executing');
        executeDecision(decision);

        // Update stats with actual token count from API response
        updateAgentStats(entityId, responseTime, decision.tokensUsed, true);

        // Return to idle
        setAgentStatus(entityId, 'idle');

        if (config.debugMode) {
          console.log(`[AIAgent] ${entityId} decision:`, decision);
        }
      } catch (error) {
        const errorMessage = (error as Error).message;

        // Check for permanent errors that shouldn't be retried
        const isPermanentError =
          errorMessage.includes('credit balance') ||
          errorMessage.includes('invalid_api_key') ||
          errorMessage.includes('authentication') ||
          errorMessage.includes('401') ||
          errorMessage.includes('403');

        // Check for transient errors that will auto-retry after backoff
        const isTransientError =
          errorMessage.includes('Rate limited') ||
          errorMessage.includes('API overloaded') ||
          errorMessage.includes('429') ||
          errorMessage.includes('529');

        if (isPermanentError) {
          // Log once and deactivate the agent to prevent spam
          console.error(`[AIAgent] Permanent error for ${entityId}, deactivating agent:`, errorMessage);
          setAgentStatus(entityId, 'error', errorMessage);
          // Deactivate the agent to stop further retries
          useAIAgentStore.getState().setAgentActive(entityId, false);
        } else if (isTransientError) {
          // Transient error - show error but stay idle for auto-retry after backoff
          console.warn(`[AIAgent] Transient error for ${entityId}:`, errorMessage);
          setAgentStatus(entityId, 'idle', errorMessage);
        } else if (errorMessage !== 'Request cancelled') {
          // Other errors
          console.error(`[AIAgent] Error processing ${entityId}:`, errorMessage);
          setAgentStatus(entityId, 'error', errorMessage);
        } else {
          // Request was cancelled, just return to idle
          setAgentStatus(entityId, 'idle');
        }
        updateAgentStats(entityId, 0, 0, false);
      }
    },
    [
      setAgentStatus,
      recordDecision,
      executeDecision,
      updateAgentStats,
    ]
  );

  /**
   * Main loop iteration
   */
  const runLoop = useCallback(async () => {
    if (!globalEnabled) {
      return; // AI globally disabled
    }
    if (!isServiceConfigured) {
      return; // API not configured
    }
    if (simStatus !== 'running') {
      return; // Simulation not running
    }

    const activeAgents = getActiveAgents();

    if (activeAgents.length === 0) {
      return; // No active agents
    }

    // Process agents in parallel (with some limit to avoid overwhelming API)
    const batchSize = 3;
    for (let i = 0; i < activeAgents.length; i += batchSize) {
      const batch = activeAgents.slice(i, i + batchSize);
      await Promise.all(batch.map((agent) => processAgent(agent)));
    }
  }, [globalEnabled, isServiceConfigured, simStatus, getActiveAgents, processAgent]);

  // Configure service when config changes
  useEffect(() => {
    if (serviceConfig) {
      aiAgentService.configure(serviceConfig);
    }
  }, [serviceConfig]);

  // Main loop effect
  useEffect(() => {
    if (!globalEnabled || !isServiceConfigured) {
      if (loopRef.current) {
        clearInterval(loopRef.current);
        loopRef.current = null;
      }
      return;
    }

    // Run loop at 10 Hz (individual agents throttle themselves)
    const interval = 100; // 100ms = 10 Hz
    loopRef.current = window.setInterval(runLoop, interval);

    return () => {
      if (loopRef.current) {
        clearInterval(loopRef.current);
        loopRef.current = null;
      }
      aiAgentService.cancelAllRequests();
    };
  }, [globalEnabled, isServiceConfigured, runLoop]);

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      if (loopRef.current) {
        clearInterval(loopRef.current);
      }
      aiAgentService.cancelAllRequests();
    };
  }, []);

  return {
    isRunning: globalEnabled && isServiceConfigured && simStatus === 'running',
  };
}

export default useAIAgentLoop;
