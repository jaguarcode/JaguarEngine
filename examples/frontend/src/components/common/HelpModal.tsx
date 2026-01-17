import React from 'react';
import { useUIStore } from '@/stores/uiStore';
import { KEYBOARD_SHORTCUTS } from '@/hooks/useKeyboardShortcuts';

export const HelpModal: React.FC = () => {
  const showHelp = useUIStore((s) => s.showHelp);
  const toggleHelp = useUIStore((s) => s.toggleHelp);

  if (!showHelp) return null;

  return (
    <div
      className="fixed inset-0 z-50 flex items-center justify-center bg-black/50 backdrop-blur-sm"
      onClick={toggleHelp}
    >
      <div
        className="panel w-full max-w-md mx-4 animate-fadeIn"
        onClick={(e) => e.stopPropagation()}
      >
        <div className="panel-header">
          <h3 className="panel-title">Keyboard Shortcuts</h3>
          <button
            className="text-gray-400 hover:text-gray-200"
            onClick={toggleHelp}
          >
            <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
              <path
                strokeLinecap="round"
                strokeLinejoin="round"
                strokeWidth={2}
                d="M6 18L18 6M6 6l12 12"
              />
            </svg>
          </button>
        </div>

        <div className="panel-content">
          <div className="space-y-2">
            {KEYBOARD_SHORTCUTS.map(({ key, description }) => (
              <div key={key} className="flex items-center justify-between py-1">
                <span className="text-sm text-gray-400">{description}</span>
                <kbd className="px-2 py-1 bg-gray-800 border border-gray-700 rounded text-xs text-gray-300 font-mono">
                  {key}
                </kbd>
              </div>
            ))}
          </div>

          <div className="mt-6 pt-4 border-t border-gray-700">
            <h4 className="text-xs font-semibold text-gray-500 uppercase mb-3">
              About JaguarEngine
            </h4>
            <p className="text-sm text-gray-400">
              JaguarEngine is a multi-domain physics simulation platform supporting
              Air, Land, Sea, and Space entities. This frontend provides real-time
              visualization and control for testing simulation features.
            </p>
            <div className="mt-3 flex items-center gap-4 text-xs text-gray-500">
              <span>Version 1.5.0</span>
              <span>•</span>
              <a
                href="https://github.com/jaguar-engine/jaguar"
                className="text-primary-400 hover:text-primary-300"
                target="_blank"
                rel="noopener noreferrer"
              >
                GitHub
              </a>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default HelpModal;
