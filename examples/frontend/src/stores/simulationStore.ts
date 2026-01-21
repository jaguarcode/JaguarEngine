import { create } from 'zustand';
import { subscribeWithSelector } from 'zustand/middleware';
import type { SimulationStatus, SimulationConfig, SimulationStats } from '@/types';

// Simple throttling for stats updates
let lastStatsUpdateTime = 0;
const STATS_UPDATE_INTERVAL_MS = 33; // ~30fps

interface SimulationState {
  // Connection
  connected: boolean;
  connectionError: string | null;

  // Simulation status
  status: SimulationStatus;
  config: SimulationConfig;
  stats: SimulationStats;

  // Actions
  setConnected: (connected: boolean, error?: string) => void;
  setStatus: (status: SimulationStatus) => void;
  setConfig: (config: Partial<SimulationConfig>) => void;
  updateStats: (stats: Partial<SimulationStats>) => void;
  reset: () => void;
}

const defaultConfig: SimulationConfig = {
  timeScale: 1.0,
  maxEntities: 10000,
  enableCollisions: true,
  enableDamage: true,
  atmosphereModel: 'standard',
  terrainEnabled: true,
  oceanEnabled: true,
  gravityModel: 'wgs84',
};

const defaultStats: SimulationStats = {
  simulationTime: 0,
  wallClockTime: 0,
  deltaTime: 0,
  realtimeRatio: 1.0,
  frameRate: 0,
  physicsTime: 0,
  renderTime: 0,
  totalEntities: 0,
  entitiesByDomain: {
    air: 0,
    land: 0,
    sea: 0,
    space: 0,
  },
  activeEntities: 0,
  collisionChecks: 0,
  activeCollisions: 0,
};

export const useSimulationStore = create<SimulationState>()(
  subscribeWithSelector((set) => ({
    // Initial state
    connected: false,
    connectionError: null,
    status: 'idle',
    config: defaultConfig,
    stats: defaultStats,

    // Actions
    setConnected: (connected, error) =>
      set({ connected, connectionError: error ?? null }),

    setStatus: (status) => set({ status }),

    setConfig: (config) =>
      set((state) => ({
        config: { ...state.config, ...config },
      })),

    updateStats: (stats) => {
      const now = performance.now();

      // Simple throttle: skip update if called too frequently
      if (now - lastStatsUpdateTime < STATS_UPDATE_INTERVAL_MS) {
        return;
      }
      lastStatsUpdateTime = now;

      set((state) => ({
        stats: { ...state.stats, ...stats },
      }));
    },

    reset: () =>
      set({
        status: 'idle',
        stats: defaultStats,
      }),
  }))
);

// Selectors
export const selectIsRunning = (state: SimulationState) =>
  state.status === 'running';

export const selectSimulationTime = (state: SimulationState) =>
  state.stats.simulationTime;

export const selectTotalEntityCount = (state: SimulationState) =>
  state.stats.totalEntities;

export const selectDomainCounts = (state: SimulationState) =>
  state.stats.entitiesByDomain;

export const selectPerformance = (state: SimulationState) => ({
  frameRate: state.stats.frameRate,
  physicsTime: state.stats.physicsTime,
  realtimeRatio: state.stats.realtimeRatio,
});
