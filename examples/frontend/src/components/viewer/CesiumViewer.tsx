import React, { useRef, useEffect } from 'react';
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
import 'cesium/Build/Cesium/Widgets/widgets.css';

import { useUIStore } from '@/stores/uiStore';
import { useEntityStore, selectFilteredEntities } from '@/stores/entityStore';
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

  const entities = useEntityStore(selectFilteredEntities);
  const selectedEntityId = useEntityStore((s) => s.selection.selectedEntityId);

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

  // Update entities - clear all and recreate for simplicity
  useEffect(() => {
    const viewer = viewerRef.current;
    if (!viewer || viewer.isDestroyed()) return;

    // Clear all existing entities and recreate
    viewer.entities.removeAll();
    entityMapRef.current.clear();

    // Add all entities fresh
    entities.forEach((entityState) => {
      try {
        // Validate position data
        const lat = entityState.position?.latitude;
        const lon = entityState.position?.longitude;
        const alt = entityState.position?.altitude ?? 0;

        if (typeof lat !== 'number' || typeof lon !== 'number' || isNaN(lat) || isNaN(lon)) {
          return; // Skip entities with invalid position data
        }

        const isSelected = entityState.id === selectedEntityId;
        const color = DOMAIN_COLORS[entityState.domain];
        const displayColor = isSelected ? Color.WHITE : color;
        const pixelSize = isSelected ? 16 : entityState.kind === 'platform' ? 12 : 8;

        const position = Cartesian3.fromDegrees(lon, lat, alt);

        const newEntity = viewer.entities.add({
          id: entityState.id,
          name: entityState.name,
          position: position,
          point: {
            pixelSize: pixelSize,
            color: displayColor,
            outlineColor: isSelected ? Color.YELLOW : Color.BLACK,
            outlineWidth: isSelected ? 2 : 1,
          },
          label: {
            text: entityState.name,
            font: '12px sans-serif',
            fillColor: Color.WHITE,
            outlineColor: Color.BLACK,
            outlineWidth: 2,
            pixelOffset: new Cartesian2(0, -15),
          },
        });
        entityMapRef.current.set(entityState.id, newEntity);
      } catch (err: unknown) {
        // Silently skip entities that fail to create
      }
    });
  }, [entities, selectedEntityId]);

  // Fly to selected entity
  useEffect(() => {
    const viewer = viewerRef.current;
    if (!viewer || viewer.isDestroyed() || !selectedEntityId) return;

    const entityState = entities.find((e) => e.id === selectedEntityId);
    if (!entityState) return;

    const destination = Cartesian3.fromDegrees(
      entityState.position.longitude,
      entityState.position.latitude,
      entityState.position.altitude + 1000
    );

    viewer.camera.flyTo({
      destination,
      orientation: {
        heading: CesiumMath.toRadians(0),
        pitch: CesiumMath.toRadians(-45),
        roll: 0,
      },
      duration: 1.5,
    });
  }, [selectedEntityId]);

  return (
    <div
      ref={containerRef}
      className={`cesium-container ${className}`}
      style={{ width: '100%', height: '100%' }}
    />
  );
};

export default CesiumViewerComponent;
