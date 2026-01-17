import React, { useMemo, useState, useCallback } from 'react';
import clsx from 'clsx';
import { useEntityStore, selectFilteredEntities } from '@/stores/entityStore';
import type { Domain, EntityState } from '@/types';

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
}> = React.memo(({ entity, isSelected, isHovered, onSelect, onHover }) => {
  return (
    <div
      className={clsx(
        'px-3 py-2 cursor-pointer transition-colors border-l-2',
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
        <div className="flex items-center gap-2 min-w-0">
          <div
            className={clsx('status-dot flex-shrink-0', {
              'status-dot-active': entity.isActive,
              'status-dot-idle': !entity.isActive,
            })}
          />
          <span className="text-sm text-gray-200 truncate">{entity.name}</span>
        </div>
        <DomainBadge domain={entity.domain} />
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

export const EntityList: React.FC = () => {
  const [searchQuery, setSearchQuery] = useState('');
  const [activeDomain, setActiveDomain] = useState<Domain | 'all'>('all');

  const entities = useEntityStore(selectFilteredEntities);
  const selectedEntityId = useEntityStore((s) => s.selection.selectedEntityId);
  const hoveredEntityId = useEntityStore((s) => s.selection.hoveredEntityId);
  const selectEntity = useEntityStore((s) => s.selectEntity);
  const hoverEntity = useEntityStore((s) => s.hoverEntity);
  const setSearchQueryStore = useEntityStore((s) => s.setSearchQuery);

  // Filter entities by search and domain
  const filteredEntities = useMemo(() => {
    let result = entities;

    if (activeDomain !== 'all') {
      result = result.filter((e) => e.domain === activeDomain);
    }

    if (searchQuery) {
      const query = searchQuery.toLowerCase();
      result = result.filter(
        (e) =>
          e.name.toLowerCase().includes(query) ||
          e.id.toLowerCase().includes(query)
      );
    }

    return result;
  }, [entities, activeDomain, searchQuery]);

  // Count by domain
  const domainCounts = useMemo(() => {
    const counts: Record<Domain | 'all', number> = {
      all: entities.length,
      air: 0,
      land: 0,
      sea: 0,
      space: 0,
    };

    entities.forEach((e) => {
      counts[e.domain]++;
    });

    return counts;
  }, [entities]);

  const handleSearchChange = useCallback(
    (e: React.ChangeEvent<HTMLInputElement>) => {
      setSearchQuery(e.target.value);
      setSearchQueryStore(e.target.value);
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

      {/* Entity list */}
      <div className="flex-1 overflow-y-auto custom-scrollbar">
        {filteredEntities.length === 0 ? (
          <div className="p-4 text-center text-sm text-gray-500">
            {searchQuery ? 'No matching entities' : 'No entities'}
          </div>
        ) : (
          filteredEntities.map((entity) => (
            <EntityListItem
              key={entity.id}
              entity={entity}
              isSelected={entity.id === selectedEntityId}
              isHovered={entity.id === hoveredEntityId}
              onSelect={selectEntity}
              onHover={hoverEntity}
            />
          ))
        )}
      </div>
    </div>
  );
};

export default EntityList;
