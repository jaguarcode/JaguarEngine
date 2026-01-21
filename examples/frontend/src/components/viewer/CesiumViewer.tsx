import React, { useRef, useEffect, useCallback } from 'react';
import {
  Viewer,
  Cartesian3,
  Math as CesiumMath,
  Ion,
  Color,
  ScreenSpaceEventType,
  ScreenSpaceEventHandler,
  defined,
  Entity,
  Cartesian2,
} from 'cesium';

import { useUIStore } from '@/stores/uiStore';
import { useEntityStore } from '@/stores/entityStore';
import { entityPositionBuffer, type EntityPosition } from '@/stores/entityPositionBuffer';
import type { Domain } from '@/types';

// Set Cesium Ion token
Ion.defaultAccessToken = import.meta.env.VITE_CESIUM_TOKEN || '';

// Default camera position (Seoul, South Korea)
const DEFAULT_CAMERA = {
  longitude: 126.9780,
  latitude: 37.5665,
  altitude: 10000,
  heading: 0,
  pitch: -45,
};

// Domain colors
const DOMAIN_COLORS: Record<Domain, Color> = {
  air: Color.fromCssColorString('#38bdf8'),
  land: Color.fromCssColorString('#84cc16'),
  sea: Color.fromCssColorString('#06b6d4'),
  space: Color.fromCssColorString('#a855f7'),
};

interface CesiumViewerComponentProps {
  className?: string;
}

export const CesiumViewerComponent: React.FC<CesiumViewerComponentProps> = ({
  className = '',
}) => {
  const containerRef = useRef<HTMLDivElement>(null);
  const viewerRef = useRef<Viewer | null>(null);
  const handlerRef = useRef<ScreenSpaceEventHandler | null>(null);
  const entityMapRef = useRef<Map<string, Entity>>(new Map());

  const view = useUIStore((s) => s.view);

  // Only use React state for selection (UI interaction)
  const selectedEntityId = useEntityStore((s) => s.selection.selectedEntityId);
  const filters = useEntityStore((s) => s.filters);

  // Initialize Cesium viewer
  useEffect(() => {
    if (!containerRef.current) return;

    // Prevent re-initialization if viewer already exists
    if (viewerRef.current) {
      try {
        // Check if viewer is still valid
        if (!viewerRef.current.isDestroyed()) {
          return;
        }
      } catch {
        // Viewer is invalid, will create new one
      }
      viewerRef.current = null;
    }

    let viewer: Viewer;
    try {
      viewer = new Viewer(containerRef.current, {
        animation: false,
        timeline: false,
        baseLayerPicker: false,
        fullscreenButton: false,
        geocoder: false,
        homeButton: false,
        infoBox: false,
        sceneModePicker: false,
        selectionIndicator: false,
        navigationHelpButton: false,
        creditContainer: document.createElement('div'), // Hide credits
      });
    } catch (error) {
      console.error('Failed to create Cesium viewer:', error);
      return;
    }

    // Configure scene
    viewer.scene.globe.enableLighting = true;
    viewer.scene.fog.enabled = true;
    if (viewer.scene.skyAtmosphere) {
      viewer.scene.skyAtmosphere.show = true;
    }

    // Set initial camera position with defaults
    const camPos = view?.cameraPosition || DEFAULT_CAMERA;
    const camHeading = view?.cameraHeading ?? DEFAULT_CAMERA.heading;
    const camPitch = view?.cameraPitch ?? DEFAULT_CAMERA.pitch;

    viewer.camera.flyTo({
      destination: Cartesian3.fromDegrees(
        camPos.longitude,
        camPos.latitude,
        camPos.altitude
      ),
      orientation: {
        heading: CesiumMath.toRadians(camHeading),
        pitch: CesiumMath.toRadians(camPitch),
        roll: 0,
      },
      duration: 0,
    });

    // Track camera movement
    viewer.camera.moveEnd.addEventListener(() => {
      if (viewer.isDestroyed()) return;
      const position = viewer.camera.positionCartographic;
      useUIStore.getState().setCameraPosition({
        latitude: CesiumMath.toDegrees(position.latitude),
        longitude: CesiumMath.toDegrees(position.longitude),
        altitude: position.height,
      });
    });

    // Setup click handler
    const handler = new ScreenSpaceEventHandler(viewer.scene.canvas);
    handler.setInputAction((movement: { position: Cartesian2 }) => {
      if (viewer.isDestroyed()) return;
      const pickedObject = viewer.scene.pick(movement.position);
      if (defined(pickedObject) && pickedObject.id) {
        const entityId = pickedObject.id.id;
        if (entityId) {
          useEntityStore.getState().selectEntity(entityId);
        }
      } else {
        useEntityStore.getState().selectEntity(null);
      }
    }, ScreenSpaceEventType.LEFT_CLICK);

    // Hover handler
    handler.setInputAction((movement: { endPosition: Cartesian2 }) => {
      if (viewer.isDestroyed()) return;
      const pickedObject = viewer.scene.pick(movement.endPosition);
      if (defined(pickedObject) && pickedObject.id) {
        const entityId = pickedObject.id.id;
        if (entityId) {
          useEntityStore.getState().hoverEntity(entityId);
        }
      } else {
        useEntityStore.getState().hoverEntity(null);
      }
    }, ScreenSpaceEventType.MOUSE_MOVE);

    viewerRef.current = viewer;
    handlerRef.current = handler;

    return () => {
      try {
        if (handlerRef.current && !handlerRef.current.isDestroyed()) {
          handlerRef.current.destroy();
        }
        if (viewerRef.current && !viewerRef.current.isDestroyed()) {
          viewerRef.current.destroy();
        }
      } catch (error) {
        console.warn('Error during Cesium cleanup:', error);
      }
      viewerRef.current = null;
      handlerRef.current = null;
      entityMapRef.current.clear();
    };
  }, []);

  // High-performance entity rendering using position buffer
  // This callback is invoked from requestAnimationFrame, bypassing React
  const updateEntitiesFromBuffer = useCallback((positions: Map<string, EntityPosition>) => {
    const viewer = viewerRef.current;
    if (!viewer || viewer.isDestroyed()) return;

    const currentEntityIds = new Set<string>();

    // Update or add entities from buffer
    positions.forEach((entityPos) => {
      // Apply filters
      if (!filters.domains.includes(entityPos.domain)) return;
      if (!filters.kinds.includes(entityPos.kind)) return;
      if (!filters.showInactive && !entityPos.isActive) return;
      if (filters.searchQuery) {
        const query = filters.searchQuery.toLowerCase();
        if (!entityPos.name.toLowerCase().includes(query) &&
            !entityPos.id.toLowerCase().includes(query)) {
          return;
        }
      }

      const lat = entityPos.latitude;
      const lon = entityPos.longitude;
      const alt = entityPos.altitude ?? 0;

      if (!Number.isFinite(lat) || !Number.isFinite(lon)) return;

      currentEntityIds.add(entityPos.id);
      const position = Cartesian3.fromDegrees(lon, lat, alt);
      const isSelected = entityPos.id === selectedEntityId;
      const color = DOMAIN_COLORS[entityPos.domain];
      const displayColor = isSelected ? Color.WHITE : color;
      const pixelSize = isSelected ? 16 : entityPos.kind === 'platform' ? 12 : 8;

      const existingEntity = entityMapRef.current.get(entityPos.id);

      if (existingEntity) {
        // Update existing entity - direct property assignment for performance
        existingEntity.position = position as unknown as typeof existingEntity.position;

        if (existingEntity.point) {
          existingEntity.point.pixelSize = pixelSize as unknown as typeof existingEntity.point.pixelSize;
          existingEntity.point.color = displayColor as unknown as typeof existingEntity.point.color;
          existingEntity.point.outlineColor = (isSelected ? Color.YELLOW : Color.BLACK) as unknown as typeof existingEntity.point.outlineColor;
          existingEntity.point.outlineWidth = (isSelected ? 2 : 1) as unknown as typeof existingEntity.point.outlineWidth;
        }
      } else {
        // Create new entity
        const newEntity = viewer.entities.add({
          id: entityPos.id,
          name: entityPos.name,
          position: position,
          point: {
            pixelSize: pixelSize,
            color: displayColor,
            outlineColor: isSelected ? Color.YELLOW : Color.BLACK,
            outlineWidth: isSelected ? 2 : 1,
          },
          label: {
            text: entityPos.name,
            font: '12px sans-serif',
            fillColor: Color.WHITE,
            outlineColor: Color.BLACK,
            outlineWidth: 2,
            pixelOffset: new Cartesian2(0, -15),
          },
        });
        entityMapRef.current.set(entityPos.id, newEntity);
      }
    });

    // Remove entities no longer in buffer
    entityMapRef.current.forEach((cesiumEntity, entityId) => {
      if (!currentEntityIds.has(entityId)) {
        viewer.entities.remove(cesiumEntity);
        entityMapRef.current.delete(entityId);
      }
    });
  }, [selectedEntityId, filters]);

  // Register buffer callback - this drives the render loop
  useEffect(() => {
    entityPositionBuffer.setUpdateCallback(updateEntitiesFromBuffer);

    return () => {
      entityPositionBuffer.setUpdateCallback(null);
    };
  }, [updateEntitiesFromBuffer]);

  // Note: Camera does NOT move when selecting an entity
  // The selected entity is highlighted visually but camera stays in place

  return (
    <div
      ref={containerRef}
      className={`cesium-container ${className}`}
      style={{ width: '100%', height: '100%' }}
    />
  );
};

export default CesiumViewerComponent;
