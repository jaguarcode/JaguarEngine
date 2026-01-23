import React, { Suspense, lazy } from 'react';
import clsx from 'clsx';

// Hooks
import { useWebSocket } from '@/hooks/useWebSocket';
import { useKeyboardShortcuts } from '@/hooks/useKeyboardShortcuts';
import { useAIAgentLoop } from '@/hooks/useAIAgentLoop';

// Stores
import { useUIStore, type ViewerType } from '@/stores/uiStore';

// Components
import { CesiumViewerComponent } from '@/components/viewer';
import { TelemetryPanel, EntityList, EntityDetailPanel } from '@/components/monitor';
import { SimControls, EntitySpawner } from '@/components/controls';
import { DomainTestPanel } from '@/components/editor';
import { ConnectionStatus, HelpModal, PerformanceOverlay } from '@/components/common';
import { AIAgentPanel } from '@/components/ai';

// Lazy load Physics components (Three.js is heavy)
const PhysicsViewer = lazy(() => import('@/components/physics/PhysicsViewer'));
const PhysicsDashboard = lazy(() => import('@/components/physics/PhysicsDashboard'));
const EventLogPanel = lazy(() => import('@/components/physics/EventLogPanel'));

// Loading fallback
const ViewerLoading: React.FC = () => (
  <div className="w-full h-full flex items-center justify-center bg-gray-950">
    <div className="text-center">
      <div className="w-8 h-8 border-2 border-primary-500 border-t-transparent rounded-full animate-spin mx-auto mb-2" />
      <div className="text-sm text-gray-400">Loading 3D Viewer...</div>
    </div>
  </div>
);

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

// Viewer mode selector
const ViewerModeSelector: React.FC = () => {
  const activeViewer = useUIStore((s) => s.activeViewer);
  const setActiveViewer = useUIStore((s) => s.setActiveViewer);

  const viewerModes: { mode: ViewerType; label: string; icon: string }[] = [
    { mode: 'cesium', label: 'Globe', icon: '🌍' },
    { mode: 'physics', label: 'Physics', icon: '⚛️' },
    { mode: 'split', label: 'Split', icon: '▣' },
  ];

  return (
    <div className="flex bg-gray-800/80 rounded-lg overflow-hidden">
      {viewerModes.map(({ mode, label, icon }) => (
        <button
          key={mode}
          className={clsx(
            'px-3 py-1.5 text-xs font-medium transition-colors flex items-center gap-1.5',
            activeViewer === mode
              ? 'bg-primary-600 text-white'
              : 'text-gray-400 hover:text-gray-200 hover:bg-gray-700/50'
          )}
          onClick={() => setActiveViewer(mode)}
          title={`${label} View`}
        >
          <span>{icon}</span>
          <span>{label}</span>
        </button>
      ))}
    </div>
  );
};

// View mode selector (for Cesium)
const ViewModeSelector: React.FC = () => {
  const viewMode = useUIStore((s) => s.view.mode);
  const setViewMode = useUIStore((s) => s.setViewMode);
  const activeViewer = useUIStore((s) => s.activeViewer);

  // Only show for Cesium viewer
  if (activeViewer === 'physics') return null;

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

// Dual Viewer Container
const DualViewerContainer: React.FC = () => {
  const activeViewer = useUIStore((s) => s.activeViewer);
  const splitRatio = useUIStore((s) => s.splitRatio);
  const setSplitRatio = useUIStore((s) => s.setSplitRatio);

  // Handle split divider drag
  const handleDividerDrag = React.useCallback(
    (e: React.MouseEvent) => {
      e.preventDefault();
      const container = e.currentTarget.parentElement;
      if (!container) return;

      const startX = e.clientX;
      const startRatio = splitRatio;
      const containerRect = container.getBoundingClientRect();

      const handleMouseMove = (moveEvent: MouseEvent) => {
        const deltaX = moveEvent.clientX - startX;
        const deltaRatio = deltaX / containerRect.width;
        setSplitRatio(startRatio + deltaRatio);
      };

      const handleMouseUp = () => {
        document.removeEventListener('mousemove', handleMouseMove);
        document.removeEventListener('mouseup', handleMouseUp);
      };

      document.addEventListener('mousemove', handleMouseMove);
      document.addEventListener('mouseup', handleMouseUp);
    },
    [splitRatio, setSplitRatio]
  );

  return (
    <div className="absolute inset-0 flex">
      {/* Cesium Viewer */}
      {(activeViewer === 'cesium' || activeViewer === 'split') && (
        <div
          className="relative h-full"
          style={{
            width: activeViewer === 'split' ? `${splitRatio * 100}%` : '100%',
          }}
        >
          <CesiumViewerComponent className="absolute inset-0" />
          {activeViewer === 'split' && (
            <div className="absolute top-2 left-2 bg-gray-900/80 px-2 py-1 rounded text-xs text-gray-300">
              🌍 Globe View
            </div>
          )}
        </div>
      )}

      {/* Split Divider */}
      {activeViewer === 'split' && (
        <div
          className="w-1 bg-gray-700 hover:bg-primary-500 cursor-col-resize flex-shrink-0 transition-colors"
          onMouseDown={handleDividerDrag}
        />
      )}

      {/* Physics Viewer */}
      {(activeViewer === 'physics' || activeViewer === 'split') && (
        <div
          className="relative h-full"
          style={{
            width: activeViewer === 'split' ? `${(1 - splitRatio) * 100}%` : '100%',
          }}
        >
          <Suspense fallback={<ViewerLoading />}>
            <PhysicsViewer showStats={false} showGrid={true} showGizmo={true} />
          </Suspense>
          {activeViewer === 'split' && (
            <div className="absolute top-2 left-2 bg-gray-900/80 px-2 py-1 rounded text-xs text-gray-300">
              ⚛️ Physics Debug
            </div>
          )}
        </div>
      )}
    </div>
  );
};

// Bottom Panel (Event Log)
const BottomPanel: React.FC = () => {
  const showBottomPanel = useUIStore((s) => s.showBottomPanel);
  const setShowBottomPanel = useUIStore((s) => s.setShowBottomPanel);

  if (!showBottomPanel) return null;

  return (
    <div className="absolute bottom-0 left-0 right-0 h-64 bg-gray-900/95 border-t border-gray-700/50 z-20">
      {/* Header */}
      <div className="flex items-center justify-between px-4 py-2 border-b border-gray-700/50">
        <h3 className="text-sm font-semibold text-white">Event Log & Debug</h3>
        <button
          onClick={() => setShowBottomPanel(false)}
          className="text-gray-400 hover:text-white transition-colors"
        >
          <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
          </svg>
        </button>
      </div>

      {/* Content */}
      <div className="h-[calc(100%-40px)] overflow-auto">
        <Suspense fallback={<div className="p-4 text-gray-400">Loading...</div>}>
          <EventLogPanel className="h-full border-0 rounded-none" maxHeight="100%" />
        </Suspense>
      </div>
    </div>
  );
};

// Bottom Panel Toggle
const BottomPanelToggle: React.FC = () => {
  const showBottomPanel = useUIStore((s) => s.showBottomPanel);
  const setShowBottomPanel = useUIStore((s) => s.setShowBottomPanel);

  return (
    <button
      onClick={() => setShowBottomPanel(!showBottomPanel)}
      className={clsx(
        'px-3 py-1.5 rounded-lg text-xs font-medium transition-colors flex items-center gap-1.5',
        showBottomPanel
          ? 'bg-cyan-500/20 text-cyan-400 border border-cyan-500/30'
          : 'bg-gray-700/50 text-gray-400 border border-gray-600/30 hover:bg-gray-700'
      )}
      title="Toggle Event Log"
    >
      <span>📋</span>
      <span>Events</span>
    </button>
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
  const activeViewer = useUIStore((s) => s.activeViewer);
  const showBottomPanel = useUIStore((s) => s.showBottomPanel);

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
            <p className="text-[10px] text-gray-500">Integrated Test Platform</p>
          </div>
        </div>

        {/* Center controls */}
        <div className="flex items-center gap-3">
          <ViewerModeSelector />
          <div className="w-px h-6 bg-gray-700" />
          <ViewModeSelector />
          <div className="w-px h-6 bg-gray-700" />
          <EntitySpawner variant="header" />
          <div className="w-px h-6 bg-gray-700" />
          <BottomPanelToggle />
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
        {/* Dual Viewer */}
        <DualViewerContainer />

        {/* Left Panel */}
        <div
          className={clsx(
            'absolute top-0 left-0 z-20 transition-transform duration-300',
            leftPanelOpen ? 'translate-x-0' : '-translate-x-full',
            showBottomPanel ? 'h-[calc(100%-256px)]' : 'h-full'
          )}
          style={{ width: '320px' }}
        >
          <div className="h-full p-4 flex flex-col gap-4 overflow-y-auto">
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
            'absolute top-0 z-10 transition-all duration-300',
            leftPanelOpen ? 'left-[320px]' : 'left-0',
            showBottomPanel ? 'h-[calc(100%-256px)]' : 'h-full'
          )}
        >
          <PanelToggle direction="left" isOpen={leftPanelOpen} onClick={toggleLeftPanel} />
        </div>

        {/* Right Panel */}
        <div
          className={clsx(
            'absolute top-0 right-0 z-20 transition-transform duration-300',
            rightPanelOpen ? 'translate-x-0' : 'translate-x-full',
            showBottomPanel ? 'h-[calc(100%-256px)]' : 'h-full'
          )}
          style={{ width: '380px' }}
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

            {/* Physics Dashboard (when in physics view) */}
            {(activeViewer === 'physics' || activeViewer === 'split') && (
              <div className="shrink-0">
                <Suspense fallback={<div className="h-40 bg-gray-900/80 rounded-lg animate-pulse" />}>
                  <PhysicsDashboard compact={activeViewer === 'split'} />
                </Suspense>
              </div>
            )}

            {/* AI Agent Controls */}
            <div className="shrink-0">
              <AIAgentPanel />
            </div>
          </div>
        </div>

        {/* Right panel toggle */}
        <div
          className={clsx(
            'absolute top-0 z-10 transition-all duration-300',
            rightPanelOpen ? 'right-[380px]' : 'right-0',
            showBottomPanel ? 'h-[calc(100%-256px)]' : 'h-full'
          )}
        >
          <PanelToggle direction="right" isOpen={rightPanelOpen} onClick={toggleRightPanel} />
        </div>

        {/* Bottom spawner removed - now in header */}

        {/* Performance overlay */}
        <PerformanceOverlay />

        {/* Bottom Panel */}
        <BottomPanel />
      </main>

      {/* Help modal */}
      <HelpModal />
    </div>
  );
}

export default App;
