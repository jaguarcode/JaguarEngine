import React from 'react';
import clsx from 'clsx';

// Hooks
import { useWebSocket } from '@/hooks/useWebSocket';
import { useKeyboardShortcuts } from '@/hooks/useKeyboardShortcuts';
import { useAIAgentLoop } from '@/hooks/useAIAgentLoop';

// Stores
import { useUIStore } from '@/stores/uiStore';

// Components
import { CesiumViewerComponent } from '@/components/viewer';
import { TelemetryPanel, EntityList, EntityDetailPanel } from '@/components/monitor';
import { SimControls, EntitySpawner } from '@/components/controls';
import { DomainTestPanel } from '@/components/editor';
import { ConnectionStatus, HelpModal, PerformanceOverlay } from '@/components/common';
import { AIAgentPanel } from '@/components/ai';

// Panel toggle button
const PanelToggle: React.FC<{
  direction: 'left' | 'right';
  isOpen: boolean;
  onClick: () => void;
}> = ({ direction, isOpen, onClick }) => (
  <button
    className={clsx(
      'absolute top-1/2 -translate-y-1/2 z-30',
      'w-6 h-16 bg-gray-800/80 border border-gray-700/50',
      'flex items-center justify-center',
      'hover:bg-gray-700/80 transition-colors',
      direction === 'left'
        ? 'left-0 rounded-r-lg border-l-0'
        : 'right-0 rounded-l-lg border-r-0'
    )}
    onClick={onClick}
  >
    <svg
      className={clsx(
        'w-4 h-4 text-gray-400 transition-transform',
        direction === 'left'
          ? isOpen
            ? 'rotate-180'
            : ''
          : isOpen
          ? ''
          : 'rotate-180'
      )}
      fill="none"
      stroke="currentColor"
      viewBox="0 0 24 24"
    >
      <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M15 19l-7-7 7-7" />
    </svg>
  </button>
);

// View mode selector
const ViewModeSelector: React.FC = () => {
  const viewMode = useUIStore((s) => s.view.mode);
  const setViewMode = useUIStore((s) => s.setViewMode);

  return (
    <div className="flex bg-gray-800/80 rounded-lg overflow-hidden">
      {[
        { mode: '2d' as const, label: '2D' },
        { mode: '3d' as const, label: '3D' },
        { mode: '3d-photo' as const, label: '3D Photo' },
      ].map(({ mode, label }) => (
        <button
          key={mode}
          className={clsx(
            'px-3 py-1.5 text-xs font-medium transition-colors',
            viewMode === mode
              ? 'bg-primary-600 text-white'
              : 'text-gray-400 hover:text-gray-200 hover:bg-gray-700/50'
          )}
          onClick={() => setViewMode(mode)}
        >
          {label}
        </button>
      ))}
    </div>
  );
};

function App() {
  // Initialize WebSocket connection
  useWebSocket();

  // Setup keyboard shortcuts
  useKeyboardShortcuts();

  // Initialize AI agent decision loop
  useAIAgentLoop();

  // Panel state
  const leftPanelOpen = useUIStore((s) => s.panels.leftPanelOpen);
  const rightPanelOpen = useUIStore((s) => s.panels.rightPanelOpen);
  const toggleLeftPanel = useUIStore((s) => s.toggleLeftPanel);
  const toggleRightPanel = useUIStore((s) => s.toggleRightPanel);

  return (
    <div className="w-full h-full bg-gray-950 flex flex-col overflow-hidden">
      {/* Header */}
      <header className="h-14 bg-gray-900/90 border-b border-gray-800 flex items-center justify-between px-4 z-20">
        {/* Logo */}
        <div className="flex items-center gap-3">
          <div className="w-8 h-8 bg-gradient-to-br from-primary-500 to-primary-700 rounded-lg flex items-center justify-center">
            <span className="text-white font-bold text-lg">J</span>
          </div>
          <div>
            <h1 className="text-sm font-bold text-white">JaguarEngine</h1>
            <p className="text-[10px] text-gray-500">Multi-Domain Simulation</p>
          </div>
        </div>

        {/* Center controls */}
        <div className="flex items-center gap-4">
          <ViewModeSelector />
        </div>

        {/* Right side */}
        <div className="flex items-center gap-4">
          <ConnectionStatus />
          <button
            className="btn-icon text-gray-400 hover:text-gray-200"
            onClick={() => useUIStore.getState().toggleHelp()}
            title="Help (?)"
          >
            <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path
                strokeLinecap="round"
                strokeLinejoin="round"
                strokeWidth={2}
                d="M8.228 9c.549-1.165 2.03-2 3.772-2 2.21 0 4 1.343 4 3 0 1.4-1.278 2.575-3.006 2.907-.542.104-.994.54-.994 1.093m0 3h.01M21 12a9 9 0 11-18 0 9 9 0 0118 0z"
              />
            </svg>
          </button>
        </div>
      </header>

      {/* Main content */}
      <main className="flex-1 relative overflow-hidden">
        {/* 3D Viewer */}
        <CesiumViewerComponent className="absolute inset-0" />

        {/* Left Panel */}
        <div
          className={clsx(
            'absolute top-0 left-0 h-full z-20 transition-transform duration-300',
            leftPanelOpen ? 'translate-x-0' : '-translate-x-full'
          )}
          style={{ width: '320px' }}
        >
          <div className="h-full p-4 flex flex-col gap-4">
            {/* Simulation Controls */}
            <SimControls />

            {/* Entity List */}
            <div className="flex-1 min-h-0">
              <EntityList />
            </div>
          </div>
        </div>

        {/* Left panel toggle */}
        <div
          className={clsx(
            'absolute top-0 h-full z-10 transition-all duration-300',
            leftPanelOpen ? 'left-[320px]' : 'left-0'
          )}
        >
          <PanelToggle direction="left" isOpen={leftPanelOpen} onClick={toggleLeftPanel} />
        </div>

        {/* Right Panel */}
        <div
          className={clsx(
            'absolute top-0 right-0 h-full z-20 transition-transform duration-300',
            rightPanelOpen ? 'translate-x-0' : 'translate-x-full'
          )}
          style={{ width: '360px' }}
        >
          <div className="h-full p-4 flex flex-col gap-3 overflow-y-auto">
            {/* Telemetry */}
            <div className="shrink-0">
              <TelemetryPanel />
            </div>

            {/* Entity Details */}
            <div className="shrink-0">
              <EntityDetailPanel />
            </div>

            {/* Domain Controls */}
            <div className="shrink-0">
              <DomainTestPanel />
            </div>

            {/* AI Agent Controls */}
            <div className="shrink-0">
              <AIAgentPanel />
            </div>
          </div>
        </div>

        {/* Right panel toggle */}
        <div
          className={clsx(
            'absolute top-0 h-full z-10 transition-all duration-300',
            rightPanelOpen ? 'right-[360px]' : 'right-0'
          )}
        >
          <PanelToggle direction="right" isOpen={rightPanelOpen} onClick={toggleRightPanel} />
        </div>

        {/* Bottom spawner (floating) */}
        <div className="absolute bottom-4 left-1/2 -translate-x-1/2 z-20 w-[400px]">
          <EntitySpawner />
        </div>

        {/* Performance overlay */}
        <PerformanceOverlay />
      </main>

      {/* Help modal */}
      <HelpModal />
    </div>
  );
}

export default App;
