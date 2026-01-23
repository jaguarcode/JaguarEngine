import React, { useRef, useMemo, useEffect } from 'react';
import { Canvas, useFrame, useThree } from '@react-three/fiber';
import {
  OrbitControls,
  Grid,
  GizmoHelper,
  GizmoViewport,
  PerspectiveCamera,
  Stats,
} from '@react-three/drei';
import * as THREE from 'three';
import { useControls, Leva, folder } from 'leva';

// Stores
import { usePhysicsDebugStore } from '@/stores/physicsDebugStore';
import { useEntityStore } from '@/stores/entityStore';

// Hooks
import { usePhysicsDebug } from '@/hooks/usePhysicsDebug';

// Components
import { CollisionShapeRenderer } from './CollisionShapeRenderer';
import { AABBTreeRenderer } from './AABBTreeRenderer';
import { ContactPointRenderer } from './ContactPointRenderer';
import { ConstraintRenderer } from './ConstraintRenderer';
import { ForceVectorRenderer } from './ForceVectorRenderer';

//==============================================================================
// Scene Setup Component
//==============================================================================

const SceneSetup: React.FC = () => {
  const { camera } = useThree();

  useEffect(() => {
    // Set initial camera position
    camera.position.set(50, 50, 50);
    camera.lookAt(0, 0, 0);
  }, [camera]);

  return null;
};

//==============================================================================
// Grid & Axes Component
//==============================================================================

const SceneGrid: React.FC<{ visible: boolean }> = ({ visible }) => {
  if (!visible) return null;

  return (
    <>
      {/* Ground grid */}
      <Grid
        args={[1000, 1000]}
        cellSize={10}
        cellThickness={0.5}
        cellColor="#404040"
        sectionSize={100}
        sectionThickness={1}
        sectionColor="#606060"
        fadeDistance={500}
        fadeStrength={1}
        followCamera={false}
        infiniteGrid
      />

      {/* Axis helper */}
      <axesHelper args={[50]} />
    </>
  );
};

//==============================================================================
// Entity Visualizer with Local Coordinate System
//==============================================================================

// Compute reference point (centroid of all entities) for local coordinates
const computeReferencePoint = (entities: Map<string, import('@/types').EntityState>) => {
  if (entities.size === 0) {
    return { lat: 0, lon: 0, alt: 0 };
  }

  let sumLat = 0;
  let sumLon = 0;
  let sumAlt = 0;
  let count = 0;

  entities.forEach((entity) => {
    const pos = entity.position;
    if (pos && isFinite(pos.latitude) && isFinite(pos.longitude)) {
      sumLat += pos.latitude;
      sumLon += pos.longitude;
      sumAlt += pos.altitude ?? 0;
      count++;
    }
  });

  if (count === 0) return { lat: 0, lon: 0, alt: 0 };

  return {
    lat: sumLat / count,
    lon: sumLon / count,
    alt: sumAlt / count,
  };
};

// Convert geodetic position to local coordinates (meters from reference point)
const toLocalCoords = (
  pos: { latitude?: number; longitude?: number; altitude?: number } | undefined,
  ref: { lat: number; lon: number; alt: number }
): THREE.Vector3 => {
  if (!pos) return new THREE.Vector3(0, 0, 0);

  const lat = pos.latitude ?? ref.lat;
  const lon = pos.longitude ?? ref.lon;
  const alt = pos.altitude ?? 0;

  // Convert delta lat/lon to meters (approximate for small distances)
  // 1 degree latitude ≈ 111,320 meters
  // 1 degree longitude ≈ 111,320 * cos(latitude) meters
  const metersPerDegreeLat = 111320;
  const metersPerDegreeLon = 111320 * Math.cos((ref.lat * Math.PI) / 180);

  const x = (lon - ref.lon) * metersPerDegreeLon;
  const z = (lat - ref.lat) * metersPerDegreeLat;
  const y = alt - ref.alt;

  // Scale down for better visualization (1 unit = 100 meters)
  const scale = 0.01;

  return new THREE.Vector3(
    isFinite(x) ? x * scale : 0,
    isFinite(y) ? y * scale : 0,
    isFinite(z) ? z * scale : 0
  );
};

// Local coordinate entity marker (simplified for physics view)
const LocalEntityMarker: React.FC<{
  entity: import('@/types').EntityState;
  position: THREE.Vector3;
  selected: boolean;
  onClick: () => void;
}> = ({ entity, position, selected, onClick }) => {
  // Domain colors
  const domainColors: Record<string, string> = {
    air: '#00aaff',
    land: '#88aa44',
    sea: '#0066cc',
    space: '#aa44ff',
  };
  const color = domainColors[entity.domain] || '#ffffff';

  return (
    <group position={position}>
      {/* Clickable mesh */}
      <mesh onClick={onClick}>
        <sphereGeometry args={[selected ? 1.5 : 1, 16, 16]} />
        <meshStandardMaterial
          color={color}
          emissive={selected ? '#ffff00' : color}
          emissiveIntensity={selected ? 0.5 : 0.2}
        />
      </mesh>

      {/* Label */}
      <sprite position={[0, 2, 0]} scale={[4, 1, 1]}>
        <spriteMaterial color={selected ? '#ffff00' : '#ffffff'} />
      </sprite>
    </group>
  );
};

const EntityVisualizer: React.FC = () => {
  const entities = useEntityStore((s) => s.entities);
  const selectedId = useEntityStore((s) => s.selection.selectedEntityId);
  const selectEntity = useEntityStore((s) => s.selectEntity);

  // Compute reference point for local coordinate system
  const referencePoint = useMemo(() => computeReferencePoint(entities), [entities]);

  // Convert entities to array with local positions
  const entitiesWithLocalPos = useMemo(() => {
    return Array.from(entities.values()).map((entity) => ({
      entity,
      localPos: toLocalCoords(entity.position, referencePoint),
    }));
  }, [entities, referencePoint]);

  return (
    <group name="entities">
      {entitiesWithLocalPos.map(({ entity, localPos }) => (
        <LocalEntityMarker
          key={entity.id}
          entity={entity}
          position={localPos}
          selected={entity.id === selectedId}
          onClick={() => selectEntity(entity.id)}
        />
      ))}
    </group>
  );
};

//==============================================================================
// Physics Debug Overlay
//==============================================================================

const PhysicsDebugOverlay: React.FC = () => {
  const settings = usePhysicsDebugStore((s) => s.visualizationSettings);
  const collisionShapes = usePhysicsDebugStore((s) => s.collisionShapes);
  const aabbTree = usePhysicsDebugStore((s) => s.aabbTree);
  const contacts = usePhysicsDebugStore((s) => s.contacts);
  const constraints = usePhysicsDebugStore((s) => s.constraints);
  const entityForces = usePhysicsDebugStore((s) => s.entityForces);
  const entities = useEntityStore((s) => s.entities);

  const shapesArray = useMemo(() => Array.from(collisionShapes.entries()), [collisionShapes]);
  const forcesArray = useMemo(() => Array.from(entityForces.values()), [entityForces]);

  // Use same local coordinate reference point as EntityVisualizer
  const referencePoint = useMemo(() => computeReferencePoint(entities), [entities]);

  return (
    <group name="physics-debug">
      {/* Collision Shapes */}
      {settings.showCollisionShapes && (
        <group name="collision-shapes">
          {shapesArray.map(([entityId, shape]) => {
            const entity = entities.get(entityId);
            const isColliding = contacts.some(
              (c) => c.entityA === entityId || c.entityB === entityId
            );
            return (
              <CollisionShapeRenderer
                key={entityId}
                shape={shape}
                entityPosition={toLocalCoords(entity?.position, referencePoint)}
                color={
                  isColliding
                    ? settings.colors.collisionShapeColliding
                    : settings.colors.collisionShape
                }
                opacity={settings.shapeOpacity}
              />
            );
          })}
        </group>
      )}

      {/* AABB Tree */}
      {settings.showAABBTree && aabbTree && (
        <AABBTreeRenderer
          tree={aabbTree}
          maxDepth={settings.aabbTreeMaxDepth}
          showOnlyLeaves={settings.aabbTreeShowOnlyLeaves}
          nodeColor={settings.colors.aabbNode}
          leafColor={settings.colors.aabbLeaf}
          opacity={settings.aabbOpacity}
        />
      )}

      {/* Contact Points */}
      {settings.showContactPoints && contacts.length > 0 && (
        <ContactPointRenderer
          contacts={contacts}
          showNormals={settings.showContactNormals}
          pointColor={settings.colors.contactPoint}
          normalColor={settings.colors.contactNormal}
        />
      )}

      {/* Constraints */}
      {settings.showConstraints && constraints.length > 0 && (
        <ConstraintRenderer
          constraints={constraints}
          entities={entities}
          activeColor={settings.colors.constraintActive}
          brokenColor={settings.colors.constraintBroken}
          scale={settings.constraintScale}
          showLimits={settings.showConstraintLimits}
        />
      )}

      {/* Force Vectors */}
      {settings.showForceVectors && forcesArray.length > 0 && (
        <group name="force-vectors">
          {forcesArray.map((entityForce) => (
            <ForceVectorRenderer
              key={entityForce.entityId}
              forces={entityForce}
              scale={settings.forceScale}
              threshold={settings.forceThreshold}
              showLabels={settings.showForceLabels}
              forceTypes={settings.forceTypes}
              colors={settings.colors}
            />
          ))}
        </group>
      )}
    </group>
  );
};

//==============================================================================
// Camera Controller with Keyboard
//==============================================================================

const CameraController: React.FC = () => {
  const { camera } = useThree();
  const keys = useRef<Set<string>>(new Set());
  const speed = 2;

  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => keys.current.add(e.key.toLowerCase());
    const handleKeyUp = (e: KeyboardEvent) => keys.current.delete(e.key.toLowerCase());

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);

    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, []);

  useFrame(() => {
    const direction = new THREE.Vector3();
    camera.getWorldDirection(direction);

    if (keys.current.has('w')) {
      camera.position.addScaledVector(direction, speed);
    }
    if (keys.current.has('s')) {
      camera.position.addScaledVector(direction, -speed);
    }
    if (keys.current.has('a')) {
      const left = new THREE.Vector3().crossVectors(camera.up, direction).normalize();
      camera.position.addScaledVector(left, speed);
    }
    if (keys.current.has('d')) {
      const right = new THREE.Vector3().crossVectors(direction, camera.up).normalize();
      camera.position.addScaledVector(right, speed);
    }
  });

  return null;
};

//==============================================================================
// Main Physics Viewer Component
//==============================================================================

interface PhysicsViewerProps {
  className?: string;
  showStats?: boolean;
  showGrid?: boolean;
  showGizmo?: boolean;
}

export const PhysicsViewer: React.FC<PhysicsViewerProps> = ({
  className = '',
  showStats = false,
  showGrid = true,
  showGizmo = true,
}) => {
  const debugEnabled = usePhysicsDebugStore((s) => s.debugEnabled);
  const updateSettings = usePhysicsDebugStore((s) => s.updateVisualizationSettings);

  // Use physics debug hook to manage server communication
  const { setDebugEnabled } = usePhysicsDebug();

  // Auto-enable physics debug when PhysicsViewer is mounted
  useEffect(() => {
    setDebugEnabled(true);
    return () => {
      // Optionally disable when unmounted (keep enabled for now for better UX)
      // setDebugEnabled(false);
    };
  }, [setDebugEnabled]);

  // Leva controls for visualization settings (called for side effects)
  useControls({
    'Collision': folder({
      showCollisionShapes: {
        value: true,
        onChange: (v) => updateSettings({ showCollisionShapes: v }),
      },
      showAABBTree: {
        value: false,
        onChange: (v) => updateSettings({ showAABBTree: v }),
      },
      showContactPoints: {
        value: true,
        onChange: (v) => updateSettings({ showContactPoints: v }),
      },
      showContactNormals: {
        value: true,
        onChange: (v) => updateSettings({ showContactNormals: v }),
      },
      shapeOpacity: {
        value: 0.3,
        min: 0,
        max: 1,
        step: 0.05,
        onChange: (v) => updateSettings({ shapeOpacity: v }),
      },
    }),
    'Constraints': folder({
      showConstraints: {
        value: true,
        onChange: (v) => updateSettings({ showConstraints: v }),
      },
      showConstraintLimits: {
        value: true,
        onChange: (v) => updateSettings({ showConstraintLimits: v }),
      },
      constraintScale: {
        value: 1.0,
        min: 0.1,
        max: 5,
        step: 0.1,
        onChange: (v) => updateSettings({ constraintScale: v }),
      },
    }),
    'Forces': folder({
      showForceVectors: {
        value: false,
        onChange: (v) => updateSettings({ showForceVectors: v }),
      },
      forceScale: {
        value: 0.001,
        min: 0.0001,
        max: 0.1,
        step: 0.0001,
        onChange: (v) => updateSettings({ forceScale: v }),
      },
      forceThreshold: {
        value: 1.0,
        min: 0,
        max: 100,
        step: 1,
        onChange: (v) => updateSettings({ forceThreshold: v }),
      },
      showForceLabels: {
        value: false,
        onChange: (v) => updateSettings({ showForceLabels: v }),
      },
    }),
    'Velocity': folder({
      showVelocityVectors: {
        value: false,
        onChange: (v) => updateSettings({ showVelocityVectors: v }),
      },
      showAngularVelocity: {
        value: false,
        onChange: (v) => updateSettings({ showAngularVelocity: v }),
      },
    }),
  });

  return (
    <div className={`relative w-full h-full ${className}`}>
      {/* Leva GUI */}
      <Leva
        collapsed
        titleBar={{ title: 'Physics Debug' }}
        theme={{
          sizes: {
            rootWidth: '320px',
            controlWidth: '160px',
          },
          colors: {
            elevation1: '#1a1a2e',
            elevation2: '#16213e',
            elevation3: '#0f3460',
            accent1: '#e94560',
            accent2: '#0f3460',
            accent3: '#533483',
            highlight1: '#ffffff',
            highlight2: '#e94560',
            highlight3: '#0f3460',
          },
        }}
      />

      <Canvas
        shadows
        dpr={[1, 2]}
        gl={{
          antialias: true,
          alpha: false,
          powerPreference: 'high-performance',
        }}
        style={{ background: '#0a0a0f' }}
      >
        <SceneSetup />

        {/* Camera */}
        <PerspectiveCamera makeDefault fov={60} near={0.1} far={10000} />

        {/* Lighting */}
        <ambientLight intensity={0.4} />
        <directionalLight
          position={[100, 100, 50]}
          intensity={1}
          castShadow
          shadow-mapSize={[2048, 2048]}
        />
        <directionalLight position={[-50, 50, -50]} intensity={0.3} />
        <hemisphereLight args={['#87ceeb', '#2a2a2a', 0.3]} />

        {/* Scene Elements */}
        <SceneGrid visible={showGrid} />

        {/* Entities */}
        <EntityVisualizer />

        {/* Physics Debug */}
        {debugEnabled && <PhysicsDebugOverlay />}

        {/* Controls */}
        <OrbitControls
          makeDefault
          enableDamping
          dampingFactor={0.05}
          minDistance={1}
          maxDistance={5000}
        />
        <CameraController />

        {/* Helpers */}
        {showGizmo && (
          <GizmoHelper alignment="bottom-right" margin={[80, 80]}>
            <GizmoViewport
              axisColors={['#ff3653', '#22e561', '#2d8cff']}
              labelColor="white"
            />
          </GizmoHelper>
        )}

        {/* Performance Stats */}
        {showStats && <Stats />}
      </Canvas>

      {/* Debug Status Indicator */}
      <div className="absolute top-4 left-4 flex items-center gap-2 bg-gray-900/80 px-3 py-2 rounded-lg">
        <div
          className={`w-2 h-2 rounded-full ${debugEnabled ? 'bg-green-500' : 'bg-gray-500'}`}
        />
        <span className="text-xs text-gray-300">
          {debugEnabled ? 'Physics Debug Active' : 'Physics Debug Off'}
        </span>
      </div>
    </div>
  );
};

export default PhysicsViewer;
