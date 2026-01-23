import React, { useMemo } from 'react';
import * as THREE from 'three';
import type {
  CollisionShape,
  SphereShape,
  BoxShape,
  CapsuleShape,
  CylinderShape,
  ConvexHullShape,
  Vec3,
  Quaternion,
} from '@/types/physics';

//==============================================================================
// Helper Functions
//==============================================================================

const DEFAULT_POSITION: Vec3 = { x: 0, y: 0, z: 0 };
const DEFAULT_ROTATION: Quaternion = { w: 1, x: 0, y: 0, z: 0 };

function safePosition(pos: Vec3 | undefined | null): [number, number, number] {
  const p = pos ?? DEFAULT_POSITION;
  return [
    isFinite(p.x) ? p.x : 0,
    isFinite(p.y) ? p.y : 0,
    isFinite(p.z) ? p.z : 0,
  ];
}

function quaternionToEuler(q: Quaternion | undefined | null): THREE.Euler {
  const r = q ?? DEFAULT_ROTATION;
  const quat = new THREE.Quaternion(
    isFinite(r.x) ? r.x : 0,
    isFinite(r.y) ? r.y : 0,
    isFinite(r.z) ? r.z : 0,
    isFinite(r.w) ? r.w : 1
  );
  return new THREE.Euler().setFromQuaternion(quat);
}

function isValidShape(shape: CollisionShape | undefined | null): shape is CollisionShape {
  return shape != null && typeof shape.type === 'string';
}

//==============================================================================
// Individual Shape Renderers
//==============================================================================

interface ShapeProps {
  color: string;
  opacity: number;
  wireframe?: boolean;
}

const SphereShapeRenderer: React.FC<{ shape: SphereShape } & ShapeProps> = ({
  shape,
  color,
  opacity,
  wireframe = false,
}) => {
  const radius = isFinite(shape.radius) ? shape.radius : 1;

  return (
    <mesh
      position={safePosition(shape.position)}
      rotation={quaternionToEuler(shape.rotation)}
    >
      <sphereGeometry args={[radius, 32, 32]} />
      <meshStandardMaterial
        color={color}
        transparent
        opacity={opacity}
        wireframe={wireframe}
        side={THREE.DoubleSide}
      />
    </mesh>
  );
};

const BoxShapeRenderer: React.FC<{ shape: BoxShape } & ShapeProps> = ({
  shape,
  color,
  opacity,
  wireframe = false,
}) => {
  const halfExtents = shape.halfExtents ?? DEFAULT_POSITION;
  const hx = isFinite(halfExtents.x) ? halfExtents.x : 1;
  const hy = isFinite(halfExtents.y) ? halfExtents.y : 1;
  const hz = isFinite(halfExtents.z) ? halfExtents.z : 1;

  return (
    <mesh
      position={safePosition(shape.position)}
      rotation={quaternionToEuler(shape.rotation)}
    >
      <boxGeometry args={[hx * 2, hy * 2, hz * 2]} />
      <meshStandardMaterial
        color={color}
        transparent
        opacity={opacity}
        wireframe={wireframe}
        side={THREE.DoubleSide}
      />
    </mesh>
  );
};

const CapsuleShapeRenderer: React.FC<{ shape: CapsuleShape } & ShapeProps> = ({
  shape,
  color,
  opacity,
  wireframe = false,
}) => {
  const radius = isFinite(shape.radius) ? shape.radius : 0.5;
  const height = isFinite(shape.height) ? shape.height : 1;

  return (
    <mesh
      position={safePosition(shape.position)}
      rotation={quaternionToEuler(shape.rotation)}
    >
      <capsuleGeometry args={[radius, height, 16, 32]} />
      <meshStandardMaterial
        color={color}
        transparent
        opacity={opacity}
        wireframe={wireframe}
        side={THREE.DoubleSide}
      />
    </mesh>
  );
};

const CylinderShapeRenderer: React.FC<{ shape: CylinderShape } & ShapeProps> = ({
  shape,
  color,
  opacity,
  wireframe = false,
}) => {
  const radius = isFinite(shape.radius) ? shape.radius : 0.5;
  const height = isFinite(shape.height) ? shape.height : 1;

  return (
    <mesh
      position={safePosition(shape.position)}
      rotation={quaternionToEuler(shape.rotation)}
    >
      <cylinderGeometry args={[radius, radius, height, 32]} />
      <meshStandardMaterial
        color={color}
        transparent
        opacity={opacity}
        wireframe={wireframe}
        side={THREE.DoubleSide}
      />
    </mesh>
  );
};

const ConvexHullShapeRenderer: React.FC<{ shape: ConvexHullShape } & ShapeProps> = ({
  shape,
  color,
  opacity,
  wireframe = false,
}) => {
  const geometry = useMemo(() => {
    const verts = shape.vertices ?? [];
    if (verts.length === 0) return null;

    const geo = new THREE.BufferGeometry();
    const vertices = new Float32Array(
      verts.flatMap((v) => [
        isFinite(v?.x) ? v.x : 0,
        isFinite(v?.y) ? v.y : 0,
        isFinite(v?.z) ? v.z : 0,
      ])
    );
    geo.setAttribute('position', new THREE.BufferAttribute(vertices, 3));
    geo.computeVertexNormals();
    return geo;
  }, [shape.vertices]);

  if (!geometry) return null;

  return (
    <mesh
      position={safePosition(shape.position)}
      rotation={quaternionToEuler(shape.rotation)}
      geometry={geometry}
    >
      <meshStandardMaterial
        color={color}
        transparent
        opacity={opacity}
        wireframe={wireframe}
        side={THREE.DoubleSide}
      />
    </mesh>
  );
};

//==============================================================================
// Main CollisionShapeRenderer
//==============================================================================

interface CollisionShapeRendererProps {
  shape: CollisionShape;
  entityPosition: THREE.Vector3;
  color: string;
  opacity: number;
  wireframe?: boolean;
}

export const CollisionShapeRenderer: React.FC<CollisionShapeRendererProps> = ({
  shape,
  entityPosition,
  color,
  opacity,
  wireframe = false,
}) => {
  // Early return if shape is invalid
  if (!isValidShape(shape)) {
    return null;
  }

  const props = { color, opacity, wireframe };

  // Safe entityPosition
  const safeEntityPosition = entityPosition ?? new THREE.Vector3();

  return (
    <group position={safeEntityPosition}>
      {shape.type === 'sphere' && (
        <SphereShapeRenderer shape={shape as SphereShape} {...props} />
      )}
      {shape.type === 'box' && (
        <BoxShapeRenderer shape={shape as BoxShape} {...props} />
      )}
      {shape.type === 'capsule' && (
        <CapsuleShapeRenderer shape={shape as CapsuleShape} {...props} />
      )}
      {shape.type === 'cylinder' && (
        <CylinderShapeRenderer shape={shape as CylinderShape} {...props} />
      )}
      {shape.type === 'convex_hull' && (
        <ConvexHullShapeRenderer shape={shape as ConvexHullShape} {...props} />
      )}
      {shape.type === 'compound' && (
        <group>
          {((shape as { children?: CollisionShape[] }).children ?? []).map((child, index) => (
            child && (
              <CollisionShapeRenderer
                key={index}
                shape={child}
                entityPosition={new THREE.Vector3()}
                {...props}
              />
            )
          ))}
        </group>
      )}

      {/* Wireframe outline for better visibility */}
      {!wireframe && (
        <group>
          {shape.type === 'sphere' && (
            <mesh
              position={safePosition(shape.position)}
              rotation={quaternionToEuler(shape.rotation)}
            >
              <sphereGeometry args={[isFinite((shape as SphereShape).radius) ? (shape as SphereShape).radius : 1, 16, 16]} />
              <meshBasicMaterial color={color} wireframe />
            </mesh>
          )}
          {shape.type === 'box' && (() => {
            const boxShape = shape as BoxShape;
            const he = boxShape.halfExtents ?? DEFAULT_POSITION;
            return (
              <mesh
                position={safePosition(shape.position)}
                rotation={quaternionToEuler(shape.rotation)}
              >
                <boxGeometry
                  args={[
                    (isFinite(he.x) ? he.x : 1) * 2,
                    (isFinite(he.y) ? he.y : 1) * 2,
                    (isFinite(he.z) ? he.z : 1) * 2,
                  ]}
                />
                <meshBasicMaterial color={color} wireframe />
              </mesh>
            );
          })()}
        </group>
      )}
    </group>
  );
};

export default CollisionShapeRenderer;
