import { create } from 'zustand';
import { persist } from 'zustand/middleware';
import type { ViewState, PanelState, GeodeticPosition } from '@/types';

// Viewer type: cesium (global), physics (Three.js), or split view
export type ViewerType = 'cesium' | 'physics' | 'split';

interface UIState {
  // View
  view: ViewState;

  // Viewer selection
  activeViewer: ViewerType;
  splitRatio: number; // 0.0 - 1.0, ratio of cesium viewer width

  // Panels
  panels: PanelState;

  // Theme
  theme: 'dark' | 'light';

  // Performance overlay
  showPerformance: boolean;

  // Help modal
  showHelp: boolean;

  // Bottom panel for logs/events
  showBottomPanel: boolean;

  // Actions - View
  setViewMode: (mode: ViewState['mode']) => void;
  setCameraPosition: (position: GeodeticPosition) => void;
  setCameraOrientation: (heading: number, pitch: number) => void;
  setZoom: (zoom: number) => void;

  // Actions - Viewer
  setActiveViewer: (viewer: ViewerType) => void;
  setSplitRatio: (ratio: number) => void;

  // Actions - Panels
  toggleLeftPanel: () => void;
  toggleRightPanel: () => void;
  toggleBottomPanel: () => void;
  setActiveTab: (panel: 'left' | 'right' | 'bottom', tab: string) => void;
  setShowBottomPanel: (show: boolean) => void;

  // Actions - UI
  togglePerformance: () => void;
  toggleHelp: () => void;
  setTheme: (theme: 'dark' | 'light') => void;
}

const defaultView: ViewState = {
  mode: '3d',
  cameraPosition: {
    latitude: 37.5665,   // Seoul, South Korea
    longitude: 126.9780,
    altitude: 10000,
  },
  cameraHeading: 0,
  cameraPitch: -45,
  zoom: 1.0,
};

const defaultPanels: PanelState = {
  leftPanelOpen: true,
  rightPanelOpen: true,
  bottomPanelOpen: false,
  activeTabs: {
    left: 'entities',
    right: 'telemetry',
    bottom: 'logs',
  },
};

export const useUIStore = create<UIState>()(
  persist(
    (set) => ({
      // Initial state
      view: defaultView,
      activeViewer: 'cesium' as ViewerType,
      splitRatio: 0.5,
      panels: defaultPanels,
      theme: 'dark',
      showPerformance: false,
      showHelp: false,
      showBottomPanel: false,

      // View actions
      setViewMode: (mode) =>
        set((state) => ({
          view: { ...state.view, mode },
        })),

      setCameraPosition: (position) =>
        set((state) => ({
          view: { ...state.view, cameraPosition: position },
        })),

      setCameraOrientation: (heading, pitch) =>
        set((state) => ({
          view: { ...state.view, cameraHeading: heading, cameraPitch: pitch },
        })),

      setZoom: (zoom) =>
        set((state) => ({
          view: { ...state.view, zoom },
        })),

      // Viewer actions
      setActiveViewer: (viewer) => set({ activeViewer: viewer }),

      setSplitRatio: (ratio) =>
        set({ splitRatio: Math.max(0.2, Math.min(0.8, ratio)) }),

      // Panel actions
      toggleLeftPanel: () =>
        set((state) => ({
          panels: { ...state.panels, leftPanelOpen: !state.panels.leftPanelOpen },
        })),

      toggleRightPanel: () =>
        set((state) => ({
          panels: { ...state.panels, rightPanelOpen: !state.panels.rightPanelOpen },
        })),

      toggleBottomPanel: () =>
        set((state) => ({
          panels: { ...state.panels, bottomPanelOpen: !state.panels.bottomPanelOpen },
        })),

      setActiveTab: (panel, tab) =>
        set((state) => ({
          panels: {
            ...state.panels,
            activeTabs: { ...state.panels.activeTabs, [panel]: tab },
          },
        })),

      setShowBottomPanel: (show) => set({ showBottomPanel: show }),

      // UI actions
      togglePerformance: () =>
        set((state) => ({ showPerformance: !state.showPerformance })),

      toggleHelp: () =>
        set((state) => ({ showHelp: !state.showHelp })),

      setTheme: (theme) => set({ theme }),
    }),
    {
      name: 'jaguar-ui-state',
      partialize: (state) => ({
        view: { mode: state.view.mode },
        activeViewer: state.activeViewer,
        splitRatio: state.splitRatio,
        panels: state.panels,
        theme: state.theme,
        showBottomPanel: state.showBottomPanel,
      }),
    }
  )
);

// Selectors
export const selectViewMode = (state: UIState) => state.view.mode;
export const selectCameraPosition = (state: UIState) => state.view.cameraPosition;
export const selectLeftPanelOpen = (state: UIState) => state.panels.leftPanelOpen;
export const selectRightPanelOpen = (state: UIState) => state.panels.rightPanelOpen;
export const selectActiveViewer = (state: UIState) => state.activeViewer;
export const selectSplitRatio = (state: UIState) => state.splitRatio;
export const selectShowBottomPanel = (state: UIState) => state.showBottomPanel;
