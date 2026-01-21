import { create } from 'zustand';
import { subscribeWithSelector } from 'zustand/middleware';
import type { EntityState, Domain, EntityKind, FilterState, SelectionState } from '@/types';

interface EntityStore {
  // Entity data
  entities: Map<string, EntityState>;
  entityList: string[];  // Ordered list of entity IDs

  // Selection
  selection: SelectionState;

  // Filtering
  filters: FilterState;

  // Actions - Entity management
  setEntities: (entities: EntityState[]) => void;
  updateEntity: (id: string, update: Partial<EntityState>) => void;
  addEntity: (entity: EntityState) => void;
  removeEntity: (id: string) => void;
  clearEntities: () => void;

  // Actions - Selection
  selectEntity: (id: string | null) => void;
  hoverEntity: (id: string | null) => void;

  // Actions - Filtering
  setFilters: (filters: Partial<FilterState>) => void;
  toggleDomainFilter: (domain: Domain) => void;
  toggleKindFilter: (kind: EntityKind) => void;
  setSearchQuery: (query: string) => void;
  resetFilters: () => void;
}

const defaultFilters: FilterState = {
  domains: ['air', 'land', 'sea', 'space'],
  kinds: ['platform', 'munition', 'sensor', 'emitter', 'cultural', 'environmental'],
  searchQuery: '',
  showInactive: false,
};

// Simple throttling - just track last update time
let lastEntityUpdateTime = 0;
const ENTITY_UPDATE_INTERVAL_MS = 33; // ~30fps for entity updates

export const useEntityStore = create<EntityStore>()(
  subscribeWithSelector((set, get) => ({
    // Initial state
    entities: new Map(),
    entityList: [],
    selection: {
      selectedEntityId: null,
      hoveredEntityId: null,
    },
    filters: defaultFilters,

    // Entity management - simple and reliable
    setEntities: (entities) => {
      const now = performance.now();

      // Simple throttle: skip update if called too frequently
      if (now - lastEntityUpdateTime < ENTITY_UPDATE_INTERVAL_MS) {
        return;
      }
      lastEntityUpdateTime = now;

      // Create new Map with new entities (always create new to ensure React detects change)
      const newMap = new Map<string, EntityState>();
      const entityIds: string[] = [];

      for (const entity of entities) {
        newMap.set(entity.id, entity);
        entityIds.push(entity.id);
      }

      set({
        entities: newMap,
        entityList: entityIds,
      });
    },

    updateEntity: (id, update) => {
      const entities = new Map(get().entities);
      const existing = entities.get(id);

      if (existing) {
        entities.set(id, { ...existing, ...update, lastUpdate: Date.now() });
        set({ entities });
      }
    },

    addEntity: (entity) => {
      const entities = new Map(get().entities);
      const entityList = [...get().entityList];

      entities.set(entity.id, entity);
      if (!entityList.includes(entity.id)) {
        entityList.push(entity.id);
      }

      set({ entities, entityList });
    },

    removeEntity: (id) => {
      const entities = new Map(get().entities);
      entities.delete(id);

      const entityList = get().entityList.filter((eid) => eid !== id);

      // Clear selection if removed entity was selected
      const selection = { ...get().selection };
      if (selection.selectedEntityId === id) {
        selection.selectedEntityId = null;
      }
      if (selection.hoveredEntityId === id) {
        selection.hoveredEntityId = null;
      }

      set({ entities, entityList, selection });
    },

    clearEntities: () => {
      set({
        entities: new Map(),
        entityList: [],
        selection: { selectedEntityId: null, hoveredEntityId: null },
      });
    },

    // Selection
    selectEntity: (id) =>
      set((state) => ({
        selection: { ...state.selection, selectedEntityId: id },
      })),

    hoverEntity: (id) =>
      set((state) => ({
        selection: { ...state.selection, hoveredEntityId: id },
      })),

    // Filtering
    setFilters: (filters) =>
      set((state) => ({
        filters: { ...state.filters, ...filters },
      })),

    toggleDomainFilter: (domain) =>
      set((state) => {
        const domains = state.filters.domains.includes(domain)
          ? state.filters.domains.filter((d) => d !== domain)
          : [...state.filters.domains, domain];
        return { filters: { ...state.filters, domains } };
      }),

    toggleKindFilter: (kind) =>
      set((state) => {
        const kinds = state.filters.kinds.includes(kind)
          ? state.filters.kinds.filter((k) => k !== kind)
          : [...state.filters.kinds, kind];
        return { filters: { ...state.filters, kinds } };
      }),

    setSearchQuery: (query) =>
      set((state) => ({
        filters: { ...state.filters, searchQuery: query },
      })),

    resetFilters: () => set({ filters: defaultFilters }),
  }))
);

// Selectors
export const selectEntity = (id: string) => (state: EntityStore) =>
  state.entities.get(id);

export const selectSelectedEntity = (state: EntityStore) =>
  state.selection.selectedEntityId
    ? state.entities.get(state.selection.selectedEntityId)
    : null;

export const selectHoveredEntity = (state: EntityStore) =>
  state.selection.hoveredEntityId
    ? state.entities.get(state.selection.hoveredEntityId)
    : null;

// Simple selector for filtered entities - no external caching
// Let zustand handle the subscription and React handle re-renders
export const selectFilteredEntities = (state: EntityStore): EntityState[] => {
  const { filters, entities, entityList } = state;

  const result: EntityState[] = [];

  for (const id of entityList) {
    const entity = entities.get(id);
    if (!entity) continue;

    // Domain filter
    if (!filters.domains.includes(entity.domain)) continue;

    // Kind filter
    if (!filters.kinds.includes(entity.kind)) continue;

    // Active filter
    if (!filters.showInactive && !entity.isActive) continue;

    // Search query
    if (filters.searchQuery) {
      const query = filters.searchQuery.toLowerCase();
      if (
        !entity.name.toLowerCase().includes(query) &&
        !entity.id.toLowerCase().includes(query)
      ) {
        continue;
      }
    }

    result.push(entity);
  }

  return result;
};

// Fast selector for entity list - just returns the array directly
export const selectEntityList = (state: EntityStore) => state.entityList;

// Fast selector for entity map - for direct access
export const selectEntities = (state: EntityStore) => state.entities;

export const selectEntitiesByDomain = (domain: Domain) => (state: EntityStore) => {
  const result: EntityState[] = [];
  for (const id of state.entityList) {
    const entity = state.entities.get(id);
    if (entity?.domain === domain) {
      result.push(entity);
    }
  }
  return result;
};

export const selectEntityCount = (state: EntityStore) => state.entityList.length;
