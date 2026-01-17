import { create } from 'zustand';
import { persist } from 'zustand/middleware';
import type { ViewState, PanelState, GeodeticPosition } from '@/types';

interface UIState {
  // View
  view: ViewState;

  // Panels
  panels: PanelState;

  // Theme
  theme: 'dark' | 'light';

  // Performance overlay
  showPerformance: boolean;

  // Help modal
  showHelp: boolean;

  // Actions - View
  setViewMode: (mode: ViewState['mode']) => void;
  setCameraPosition: (position: GeodeticPosition) => void;
  setCameraOrientation: (heading: number, pitch: number) => void;
  setZoom: (zoom: number) => void;

  // Actions - Panels
  toggleLeftPanel: () => void;
  toggleRightPanel: () => void;
  toggleBottomPanel: () => void;
  setActiveTab: (panel: 'left' | 'right' | 'bottom', tab: string) => void;

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
      panels: defaultPanels,
      theme: 'dark',
      showPerformance: false,
      showHelp: false,

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
        panels: state.panels,
        theme: state.theme,
      }),
    }
  )
);

// Selectors
export const selectViewMode = (state: UIState) => state.view.mode;
export const selectCameraPosition = (state: UIState) => state.view.cameraPosition;
export const selectLeftPanelOpen = (state: UIState) => state.panels.leftPanelOpen;
export const selectRightPanelOpen = (state: UIState) => state.panels.rightPanelOpen;
