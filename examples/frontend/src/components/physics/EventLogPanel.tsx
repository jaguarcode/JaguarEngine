import React, { useMemo, useState } from 'react';
import clsx from 'clsx';

// Stores
import { usePhysicsDebugStore } from '@/stores/physicsDebugStore';

// Types
import type { PhysicsEvent, EventCategory, EventSeverity } from '@/types/physics';

//==============================================================================
// Constants
//==============================================================================

const CATEGORY_COLORS: Record<EventCategory, string> = {
  system: 'bg-blue-500',
  entity: 'bg-green-500',
  physics: 'bg-purple-500',
  domain: 'bg-yellow-500',
  sensor: 'bg-cyan-500',
  threshold: 'bg-orange-500',
  user: 'bg-pink-500',
};

const SEVERITY_COLORS: Record<EventSeverity, string> = {
  debug: 'text-gray-400',
  info: 'text-blue-400',
  warning: 'text-yellow-400',
  error: 'text-red-400',
};

const SEVERITY_ICONS: Record<EventSeverity, string> = {
  debug: '🔍',
  info: 'ℹ️',
  warning: '⚠️',
  error: '❌',
};

//==============================================================================
// Event Row Component
//==============================================================================

interface EventRowProps {
  event: PhysicsEvent;
  expanded: boolean;
  onToggle: () => void;
}

const EventRow: React.FC<EventRowProps> = ({ event, expanded, onToggle }) => {
  const formatTime = (timestamp: number) => {
    const date = new Date(timestamp);
    return date.toLocaleTimeString('en-US', {
      hour12: false,
      hour: '2-digit',
      minute: '2-digit',
      second: '2-digit',
      fractionalSecondDigits: 3,
    });
  };

  return (
    <div
      className={clsx(
        'border-b border-gray-800/50 hover:bg-gray-800/30 cursor-pointer transition-colors',
        expanded && 'bg-gray-800/20'
      )}
      onClick={onToggle}
    >
      {/* Main row */}
      <div className="px-3 py-2 flex items-start gap-2">
        {/* Severity icon */}
        <span className="text-xs">{SEVERITY_ICONS[event.severity]}</span>

        {/* Category badge */}
        <span
          className={clsx(
            'px-1.5 py-0.5 rounded text-[10px] font-medium text-white',
            CATEGORY_COLORS[event.category]
          )}
        >
          {event.category.toUpperCase()}
        </span>

        {/* Message */}
        <div className="flex-1 min-w-0">
          <div className={clsx('text-xs truncate', SEVERITY_COLORS[event.severity])}>
            {event.message}
          </div>
          {event.entityId && (
            <div className="text-[10px] text-gray-500 mt-0.5">Entity: {event.entityId}</div>
          )}
        </div>

        {/* Timestamp */}
        <div className="text-[10px] text-gray-500 whitespace-nowrap">
          {formatTime(event.timestamp)}
        </div>

        {/* Expand indicator */}
        {event.data && Object.keys(event.data).length > 0 && (
          <svg
            className={clsx(
              'w-4 h-4 text-gray-500 transition-transform',
              expanded && 'rotate-180'
            )}
            fill="none"
            stroke="currentColor"
            viewBox="0 0 24 24"
          >
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19 9l-7 7-7-7" />
          </svg>
        )}
      </div>

      {/* Expanded details */}
      {expanded && event.data && Object.keys(event.data).length > 0 && (
        <div className="px-3 pb-3 pt-1">
          <div className="bg-gray-900/50 rounded p-2 text-xs font-mono">
            <pre className="text-gray-300 whitespace-pre-wrap overflow-x-auto">
              {JSON.stringify(event.data, null, 2)}
            </pre>
          </div>
        </div>
      )}
    </div>
  );
};

//==============================================================================
// Filter Bar Component
//==============================================================================

interface FilterBarProps {
  categories: EventCategory[];
  selectedCategories: EventCategory[];
  onCategoryToggle: (category: EventCategory) => void;
  severities: EventSeverity[];
  selectedSeverities: EventSeverity[];
  onSeverityToggle: (severity: EventSeverity) => void;
  searchQuery: string;
  onSearchChange: (query: string) => void;
  onClear: () => void;
}

const FilterBar: React.FC<FilterBarProps> = ({
  categories,
  selectedCategories,
  onCategoryToggle,
  severities,
  selectedSeverities,
  onSeverityToggle,
  searchQuery,
  onSearchChange,
  onClear,
}) => {
  return (
    <div className="p-3 border-b border-gray-700/50 space-y-2">
      {/* Search */}
      <div className="relative">
        <input
          type="text"
          value={searchQuery}
          onChange={(e) => onSearchChange(e.target.value)}
          placeholder="Search events..."
          className="w-full bg-gray-800/50 border border-gray-700/50 rounded px-3 py-1.5 text-xs text-white placeholder-gray-500 focus:outline-none focus:border-primary-500/50"
        />
        {searchQuery && (
          <button
            onClick={() => onSearchChange('')}
            className="absolute right-2 top-1/2 -translate-y-1/2 text-gray-500 hover:text-gray-300"
          >
            ×
          </button>
        )}
      </div>

      {/* Category filters */}
      <div className="flex flex-wrap gap-1">
        {categories.map((category) => (
          <button
            key={category}
            onClick={() => onCategoryToggle(category)}
            className={clsx(
              'px-2 py-0.5 rounded text-[10px] font-medium transition-colors',
              selectedCategories.includes(category)
                ? clsx(CATEGORY_COLORS[category], 'text-white')
                : 'bg-gray-800/50 text-gray-400 hover:bg-gray-700/50'
            )}
          >
            {category}
          </button>
        ))}
      </div>

      {/* Severity filters */}
      <div className="flex items-center gap-2">
        <span className="text-[10px] text-gray-500">Severity:</span>
        <div className="flex gap-1">
          {severities.map((severity) => (
            <button
              key={severity}
              onClick={() => onSeverityToggle(severity)}
              className={clsx(
                'px-2 py-0.5 rounded text-[10px] transition-colors',
                selectedSeverities.includes(severity)
                  ? clsx(SEVERITY_COLORS[severity], 'bg-gray-700/50')
                  : 'text-gray-500 hover:text-gray-300'
              )}
            >
              {severity}
            </button>
          ))}
        </div>
        <div className="flex-1" />
        <button
          onClick={onClear}
          className="px-2 py-0.5 rounded text-[10px] text-red-400 hover:bg-red-500/10 transition-colors"
        >
          Clear All
        </button>
      </div>
    </div>
  );
};

//==============================================================================
// Main Event Log Panel
//==============================================================================

interface EventLogPanelProps {
  className?: string;
  maxHeight?: string;
}

export const EventLogPanel: React.FC<EventLogPanelProps> = ({
  className = '',
  maxHeight = '400px',
}) => {
  const events = usePhysicsDebugStore((s) => s.events);
  const clearEvents = usePhysicsDebugStore((s) => s.clearEvents);

  const [expandedId, setExpandedId] = useState<string | null>(null);
  const [searchQuery, setSearchQuery] = useState('');
  const [selectedCategories, setSelectedCategories] = useState<EventCategory[]>([
    'system',
    'entity',
    'physics',
    'domain',
    'sensor',
    'threshold',
    'user',
  ]);
  const [selectedSeverities, setSelectedSeverities] = useState<EventSeverity[]>([
    'debug',
    'info',
    'warning',
    'error',
  ]);

  const allCategories: EventCategory[] = [
    'system',
    'entity',
    'physics',
    'domain',
    'sensor',
    'threshold',
    'user',
  ];
  const allSeverities: EventSeverity[] = ['debug', 'info', 'warning', 'error'];

  const handleCategoryToggle = (category: EventCategory) => {
    setSelectedCategories((prev) =>
      prev.includes(category) ? prev.filter((c) => c !== category) : [...prev, category]
    );
  };

  const handleSeverityToggle = (severity: EventSeverity) => {
    setSelectedSeverities((prev) =>
      prev.includes(severity) ? prev.filter((s) => s !== severity) : [...prev, severity]
    );
  };

  const filteredEvents = useMemo(() => {
    return events.filter((event) => {
      // Category filter
      if (!selectedCategories.includes(event.category)) return false;

      // Severity filter
      if (!selectedSeverities.includes(event.severity)) return false;

      // Search filter
      if (searchQuery) {
        const query = searchQuery.toLowerCase();
        return (
          event.message.toLowerCase().includes(query) ||
          event.type.toLowerCase().includes(query) ||
          event.entityId?.toLowerCase().includes(query)
        );
      }

      return true;
    });
  }, [events, selectedCategories, selectedSeverities, searchQuery]);

  return (
    <div className={clsx('bg-gray-900/80 rounded-lg border border-gray-700/50', className)}>
      {/* Header */}
      <div className="px-4 py-3 border-b border-gray-700/50 flex items-center justify-between">
        <h3 className="text-sm font-semibold text-white flex items-center gap-2">
          <span className="w-2 h-2 bg-cyan-500 rounded-full" />
          Event Log
          <span className="text-xs font-normal text-gray-500">
            ({filteredEvents.length} / {events.length})
          </span>
        </h3>
        <div className="flex items-center gap-2">
          {/* Auto-scroll toggle could go here */}
        </div>
      </div>

      {/* Filters */}
      <FilterBar
        categories={allCategories}
        selectedCategories={selectedCategories}
        onCategoryToggle={handleCategoryToggle}
        severities={allSeverities}
        selectedSeverities={selectedSeverities}
        onSeverityToggle={handleSeverityToggle}
        searchQuery={searchQuery}
        onSearchChange={setSearchQuery}
        onClear={clearEvents}
      />

      {/* Event list */}
      <div className="overflow-y-auto" style={{ maxHeight }}>
        {filteredEvents.length === 0 ? (
          <div className="px-4 py-8 text-center text-gray-500 text-sm">
            {events.length === 0 ? 'No events recorded' : 'No events match filters'}
          </div>
        ) : (
          filteredEvents.map((event) => (
            <EventRow
              key={event.id}
              event={event}
              expanded={expandedId === event.id}
              onToggle={() => setExpandedId(expandedId === event.id ? null : event.id)}
            />
          ))
        )}
      </div>

      {/* Footer stats */}
      <div className="px-4 py-2 border-t border-gray-700/50 flex items-center justify-between text-[10px] text-gray-500">
        <span>
          Errors: {events.filter((e) => e.severity === 'error').length} | Warnings:{' '}
          {events.filter((e) => e.severity === 'warning').length}
        </span>
        <span>Max buffer: {usePhysicsDebugStore.getState().maxEventLogSize}</span>
      </div>
    </div>
  );
};

export default EventLogPanel;
