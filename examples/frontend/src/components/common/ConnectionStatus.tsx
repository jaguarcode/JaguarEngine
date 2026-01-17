import React from 'react';
import clsx from 'clsx';
import { useSimulationStore } from '@/stores/simulationStore';

export const ConnectionStatus: React.FC = () => {
  const connected = useSimulationStore((s) => s.connected);
  const connectionError = useSimulationStore((s) => s.connectionError);

  return (
    <div
      className={clsx(
        'flex items-center gap-2 px-3 py-1.5 rounded-full text-xs font-medium',
        connected
          ? 'bg-green-900/50 text-green-400 border border-green-700/50'
          : 'bg-red-900/50 text-red-400 border border-red-700/50'
      )}
    >
      <div
        className={clsx('w-2 h-2 rounded-full', {
          'bg-green-500 animate-pulse': connected,
          'bg-red-500': !connected,
        })}
      />
      <span>{connected ? 'Connected' : connectionError || 'Disconnected'}</span>
    </div>
  );
};

export default ConnectionStatus;
