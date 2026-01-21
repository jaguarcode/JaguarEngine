import React, { useMemo, useState, useCallback, useRef, useEffect } from 'react';
import clsx from 'clsx';
import { useEntityStore, selectFilteredEntities } from '@/stores/entityStore';
import { useWebSocket } from '@/hooks/useWebSocket';
import type { Domain, EntityState } from '@/types';

// Trash icon for delete button
const TrashIcon = () => (
  <svg className="w-4 h-4" fill="none" stroke="currentColor" viewBox="0 0 24 24">
    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2}
      d="M19 7l-.867 12.142A2 2 0 0116.138 21H7.862a2 2 0 01-1.995-1.858L5 7m5 4v6m4-6v6m1-10V4a1 1 0 00-1-1h-4a1 1 0 00-1 1v3M4 7h16" />
  </svg>
);

// Constants for virtualization
const ITEM_HEIGHT = 56; // Height of each entity list item in pixels
const OVERSCAN = 5;     // Number of items to render outside visible area

// Domain badge component
const DomainBadge: React.FC<{ domain: Domain }> = ({ domain }) => (
  <span
    className={clsx('domain-badge', {
      'domain-badge-air': domain === 'air',
      'domain-badge-land': domain === 'land',
      'domain-badge-sea': domain === 'sea',
      'domain-badge-space': domain === 'space',
    })}
  >
    {domain}
  </span>
);

// Health bar component
const HealthBar: React.FC<{ health: number }> = ({ health }) => {
  const color =
    health >= 75 ? 'bg-green-500' : health >= 50 ? 'bg-yellow-500' : 'bg-red-500';

  return (
    <div className="w-16 h-1.5 bg-gray-700 rounded-full overflow-hidden">
      <div className={clsx('h-full transition-all', color)} style={{ width: `${health}%` }} />
    </div>
  );
};

// Entity list item
const EntityListItem: React.FC<{
  entity: EntityState;
  isSelected: boolean;
  isHovered: boolean;
  onSelect: (id: string) => void;
  onHover: (id: string | null) => void;
  onDelete: (id: string) => void;
}> = React.memo(({ entity, isSelected, isHovered, onSelect, onHover, onDelete }) => {
  const handleDelete = (e: React.MouseEvent) => {
    e.stopPropagation();
    onDelete(entity.id);
  };

  return (
    <div
      className={clsx(
        'px-3 py-2 cursor-pointer transition-colors border-l-2 group',
        {
          'bg-primary-900/50 border-primary-500': isSelected,
          'bg-gray-800/50 border-gray-600': isHovered && !isSelected,
          'border-transparent hover:bg-gray-800/30': !isSelected && !isHovered,
        }
      )}
      onClick={() => onSelect(entity.id)}
      onMouseEnter={() => onHover(entity.id)}
      onMouseLeave={() => onHover(null)}
    >
      <div className="flex items-center justify-between">
        <div className="flex items-center gap-2 min-w-0 flex-1">
          <div
            className={clsx('status-dot flex-shrink-0', {
              'status-dot-active': entity.isActive,
              'status-dot-idle': !entity.isActive,
            })}
          />
          <span className="text-sm text-gray-200 truncate">{entity.name}</span>
        </div>
        <div className="flex items-center gap-2">
          <DomainBadge domain={entity.domain} />
          <button
            onClick={handleDelete}
            className="p-1 text-gray-500 hover:text-red-400 opacity-0 group-hover:opacity-100 transition-opacity"
            title="Remove entity"
          >
            <TrashIcon />
          </button>
        </div>
      </div>

      <div className="mt-1 flex items-center justify-between text-xs text-gray-500">
        <span className="uppercase">{entity.kind}</span>
        <HealthBar health={entity.health} />
      </div>
    </div>
  );
});

EntityListItem.displayName = 'EntityListItem';

// Filter tabs
const DOMAIN_FILTERS: (Domain | 'all')[] = ['all', 'air', 'land', 'sea', 'space'];

// Virtualized list hook for efficient rendering of large lists
function useVirtualizedList<T>(
  items: T[],
  containerRef: React.RefObject<HTMLDivElement>,
  itemHeight: number,
  overscan: number = OVERSCAN
) {
  const [scrollTop, setScrollTop] = useState(0);
  const [containerHeight, setContainerHeight] = useState(0);

  useEffect(() => {
    const container = containerRef.current;
    if (!container) return;

    const handleScroll = () => {
      setScrollTop(container.scrollTop);
    };

    const handleResize = () => {
      setContainerHeight(container.clientHeight);
    };

    // Initial measurement
    handleResize();

    container.addEventListener('scroll', handleScroll, { passive: true });
    window.addEventListener('resize', handleResize);

    return () => {
      container.removeEventListener('scroll', handleScroll);
      window.removeEventListener('resize', handleResize);
    };
  }, [containerRef]);

  const totalHeight = items.length * itemHeight;
  const startIndex = Math.max(0, Math.floor(scrollTop / itemHeight) - overscan);
  const endIndex = Math.min(
    items.length,
    Math.ceil((scrollTop + containerHeight) / itemHeight) + overscan
  );

  const visibleItems = items.slice(startIndex, endIndex);
  const offsetY = startIndex * itemHeight;

  return {
    visibleItems,
    totalHeight,
    offsetY,
    startIndex,
  };
}

export const EntityList: React.FC = () => {
  const [searchQuery, setSearchQuery] = useState('');
  const [activeDomain, setActiveDomain] = useState<Domain | 'all'>('all');
  const listContainerRef = useRef<HTMLDivElement>(null);

  const { destroyEntity } = useWebSocket();

  const entities = useEntityStore(selectFilteredEntities);
  const selectedEntityId = useEntityStore((s) => s.selection.selectedEntityId);
  const hoveredEntityId = useEntityStore((s) => s.selection.hoveredEntityId);
  const selectEntity = useEntityStore((s) => s.selectEntity);
  const hoverEntity = useEntityStore((s) => s.hoverEntity);
  const setSearchQueryStore = useEntityStore((s) => s.setSearchQuery);

  const handleDeleteEntity = useCallback((entityId: string) => {
    destroyEntity(entityId);
  }, [destroyEntity]);

  // Filter entities by domain only (search is already applied in store)
  const filteredEntities = useMemo(() => {
    if (activeDomain === 'all') {
      return entities;
    }
    return entities.filter((e) => e.domain === activeDomain);
  }, [entities, activeDomain]);

  // Use virtualization for large lists
  const { visibleItems, totalHeight, offsetY } = useVirtualizedList(
    filteredEntities,
    listContainerRef,
    ITEM_HEIGHT
  );

  // Count by domain - optimized with single pass
  const domainCounts = useMemo(() => {
    const counts: Record<Domain | 'all', number> = {
      all: entities.length,
      air: 0,
      land: 0,
      sea: 0,
      space: 0,
    };

    for (const e of entities) {
      counts[e.domain]++;
    }

    return counts;
  }, [entities]);

  const handleSearchChange = useCallback(
    (e: React.ChangeEvent<HTMLInputElement>) => {
      const value = e.target.value;
      setSearchQuery(value);
      setSearchQueryStore(value);
    },
    [setSearchQueryStore]
  );

  return (
    <div className="panel h-full flex flex-col">
      <div className="panel-header">
        <h3 className="panel-title">Entities</h3>
        <span className="text-xs text-gray-500">
          {filteredEntities.length} / {entities.length}
        </span>
      </div>

      {/* Search */}
      <div className="px-3 py-2 border-b border-gray-700/50">
        <input
          type="text"
          className="input"
          placeholder="Search entities..."
          value={searchQuery}
          onChange={handleSearchChange}
        />
      </div>

      {/* Domain filter tabs */}
      <div className="flex border-b border-gray-700/50 overflow-x-auto">
        {DOMAIN_FILTERS.map((domain) => (
          <button
            key={domain}
            className={clsx(
              'flex-1 min-w-0 px-2 py-2 text-xs font-medium transition-colors',
              'border-b-2',
              {
                'border-primary-500 text-primary-400': activeDomain === domain,
                'border-transparent text-gray-500 hover:text-gray-400':
                  activeDomain !== domain,
              }
            )}
            onClick={() => setActiveDomain(domain)}
          >
            <span className="block truncate capitalize">{domain}</span>
            <span className="text-[10px] opacity-60">{domainCounts[domain]}</span>
          </button>
        ))}
      </div>

      {/* Virtualized entity list */}
      <div ref={listContainerRef} className="flex-1 overflow-y-auto custom-scrollbar">
        {filteredEntities.length === 0 ? (
          <div className="p-4 text-center text-sm text-gray-500">
            {searchQuery ? 'No matching entities' : 'No entities'}
          </div>
        ) : (
          <div style={{ height: totalHeight, position: 'relative' }}>
            <div style={{ transform: `translateY(${offsetY}px)` }}>
              {visibleItems.map((entity) => (
                <EntityListItem
                  key={entity.id}
                  entity={entity}
                  isSelected={entity.id === selectedEntityId}
                  isHovered={entity.id === hoveredEntityId}
                  onSelect={selectEntity}
                  onHover={hoverEntity}
                  onDelete={handleDeleteEntity}
                />
              ))}
            </div>
          </div>
        )}
      </div>
    </div>
  );
};

export default EntityList;
