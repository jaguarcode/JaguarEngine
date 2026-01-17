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

// Throttle helper for high-frequency updates
let updateTimeout: number | null = null;
const THROTTLE_MS = 100;

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

    // Entity management
    setEntities: (entities) => {
      // Throttle batch updates
      if (updateTimeout) {
        clearTimeout(updateTimeout);
      }

      updateTimeout = window.setTimeout(() => {
        const entityMap = new Map<string, EntityState>();
        const entityIds: string[] = [];

        for (const entity of entities) {
          entityMap.set(entity.id, entity);
          entityIds.push(entity.id);
        }

        set({
          entities: entityMap,
          entityList: entityIds,
        });

        updateTimeout = null;
      }, THROTTLE_MS);
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

export const selectFilteredEntities = (state: EntityStore): EntityState[] => {
  const { filters, entities, entityList } = state;

  return entityList
    .map((id) => entities.get(id))
    .filter((entity): entity is EntityState => {
      if (!entity) return false;

      // Domain filter
      if (!filters.domains.includes(entity.domain)) return false;

      // Kind filter
      if (!filters.kinds.includes(entity.kind)) return false;

      // Active filter
      if (!filters.showInactive && !entity.isActive) return false;

      // Search query
      if (filters.searchQuery) {
        const query = filters.searchQuery.toLowerCase();
        return (
          entity.name.toLowerCase().includes(query) ||
          entity.id.toLowerCase().includes(query)
        );
      }

      return true;
    });
};

export const selectEntitiesByDomain = (domain: Domain) => (state: EntityStore) =>
  state.entityList
    .map((id) => state.entities.get(id))
    .filter((e): e is EntityState => e?.domain === domain);

export const selectEntityCount = (state: EntityStore) => state.entityList.length;
