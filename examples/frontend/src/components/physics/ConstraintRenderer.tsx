import React from 'react';
import * as THREE from 'three';
import { Line, Html } from '@react-three/drei';
import type { Constraint, DistanceConstraint, HingeConstraint, Vec3 } from '@/types/physics';
import type { EntityState } from '@/types';

//==============================================================================
// Helper Functions
//==============================================================================

function safeNumber(n: number | undefined | null): number {
  return isFinite(n as number) ? (n as number) : 0;
}

function safeVec3(v: Vec3 | { x: number; y: number; z: number } | undefined | null): THREE.Vector3 {
  if (!v) return new THREE.Vector3(0, 0, 0);
  return new THREE.Vector3(
    safeNumber(v.x),
    safeNumber(v.y),
    safeNumber(v.z)
  );
}

function isValidConstraint(constraint: Constraint | undefined | null): constraint is Constraint {
  return constraint != null && constraint.bodyA != null;
}

function getEntityWorldPosition(
  entityId: string | undefined,
  entities: Map<string, EntityState>,
  localAnchor: { x: number; y: number; z: number } | undefined | null
): THREE.Vector3 {
  const anchor = safeVec3(localAnchor);

  if (!entityId) {
    return anchor;
  }

  const entity = entities.get(entityId);
  if (!entity || !entity.position) {
    return anchor;
  }

  // Convert geodetic to local (simplified - actual implementation would use proper projection)
  const lon = safeNumber(entity.position.longitude);
  const lat = safeNumber(entity.position.latitude);
  const alt = safeNumber(entity.position.altitude);

  const basePos = new THREE.Vector3(
    lon * 111000,
    alt,
    lat * 111000
  );

  // Apply local anchor offset
  return basePos.add(anchor);
}

//==============================================================================
// Distance Constraint Renderer
//==============================================================================

interface DistanceConstraintRendererProps {
  constraint: DistanceConstraint;
  entities: Map<string, EntityState>;
  color: string;
  scale: number;
  showLimits: boolean;
}

const DistanceConstraintRenderer: React.FC<DistanceConstraintRendererProps> = ({
  constraint,
  entities,
  color,
  scale,
  showLimits,
}) => {
  const posA = getEntityWorldPosition(
    constraint.bodyA.entityId,
    entities,
    constraint.bodyA.localAnchor
  );

  const posB = constraint.bodyB
    ? getEntityWorldPosition(
        constraint.bodyB.entityId,
        entities,
        constraint.bodyB.localAnchor
      )
    : new THREE.Vector3(
        constraint.bodyA.localAnchor.x,
        constraint.bodyA.localAnchor.y,
        constraint.bodyA.localAnchor.z
      );

  const midpoint = new THREE.Vector3().addVectors(posA, posB).multiplyScalar(0.5);

  return (
    <group>
      {/* Main constraint line */}
      <Line
        points={[posA, posB]}
        color={color}
        lineWidth={2}
        dashed={!constraint.enabled}
        dashSize={0.5}
        gapSize={0.3}
      />

      {/* Anchor points */}
      <mesh position={posA}>
        <sphereGeometry args={[0.3 * scale, 8, 8]} />
        <meshBasicMaterial color={color} />
      </mesh>
      <mesh position={posB}>
        <sphereGeometry args={[0.3 * scale, 8, 8]} />
        <meshBasicMaterial color={color} />
      </mesh>

      {/* Distance limits visualization */}
      {showLimits && (
        <>
          {/* Min distance sphere */}
          <mesh position={posA}>
            <sphereGeometry args={[constraint.minDistance, 16, 16]} />
            <meshBasicMaterial
              color="#00ff00"
              transparent
              opacity={0.1}
              wireframe
            />
          </mesh>

          {/* Max distance sphere */}
          <mesh position={posA}>
            <sphereGeometry args={[constraint.maxDistance, 16, 16]} />
            <meshBasicMaterial
              color="#ff0000"
              transparent
              opacity={0.1}
              wireframe
            />
          </mesh>
        </>
      )}

      {/* Label */}
      <Html position={midpoint} center>
        <div className="bg-gray-900/80 px-2 py-1 rounded text-xs text-white whitespace-nowrap">
          Distance: {safeNumber(constraint.currentDistance).toFixed(2)}m
        </div>
      </Html>
    </group>
  );
};

//==============================================================================
// Hinge Constraint Renderer
//==============================================================================

interface HingeConstraintRendererProps {
  constraint: HingeConstraint;
  entities: Map<string, EntityState>;
  color: string;
  scale: number;
  showLimits: boolean;
}

const HingeConstraintRenderer: React.FC<HingeConstraintRendererProps> = ({
  constraint,
  entities,
  color,
  scale,
  showLimits,
}) => {
  if (!constraint.bodyA) return null;

  const posA = getEntityWorldPosition(
    constraint.bodyA.entityId,
    entities,
    constraint.bodyA.localAnchor
  );

  const axisVec = safeVec3(constraint.axis);
  const axis = axisVec.length() > 0.001 ? axisVec.normalize() : new THREE.Vector3(0, 1, 0);

  // Create rotation to align cylinder with axis
  const quaternion = new THREE.Quaternion().setFromUnitVectors(
    new THREE.Vector3(0, 1, 0),
    axis
  );

  return (
    <group position={posA}>
      {/* Hinge axis cylinder */}
      <mesh quaternion={quaternion}>
        <cylinderGeometry args={[0.2 * scale, 0.2 * scale, 2 * scale, 16]} />
        <meshBasicMaterial color={color} />
      </mesh>

      {/* Hinge disk */}
      <mesh quaternion={quaternion}>
        <torusGeometry args={[0.8 * scale, 0.1 * scale, 8, 32]} />
        <meshBasicMaterial color={color} transparent opacity={0.5} />
      </mesh>

      {/* Current angle indicator */}
      {showLimits && (
        <group quaternion={quaternion}>
          {/* Angle arc */}
          <mesh rotation={[Math.PI / 2, 0, constraint.currentAngle]}>
            <circleGeometry args={[0.6 * scale, 32, 0, Math.abs(constraint.currentAngle)]} />
            <meshBasicMaterial
              color="#ffff00"
              transparent
              opacity={0.3}
              side={THREE.DoubleSide}
            />
          </mesh>

          {/* Min/Max angle limits */}
          {constraint.minAngle !== undefined && (
            <Line
              points={[
                new THREE.Vector3(0, 0, 0),
                new THREE.Vector3(
                  Math.cos(constraint.minAngle) * scale,
                  0,
                  Math.sin(constraint.minAngle) * scale
                ),
              ]}
              color="#ff0000"
              lineWidth={2}
            />
          )}
          {constraint.maxAngle !== undefined && (
            <Line
              points={[
                new THREE.Vector3(0, 0, 0),
                new THREE.Vector3(
                  Math.cos(constraint.maxAngle) * scale,
                  0,
                  Math.sin(constraint.maxAngle) * scale
                ),
              ]}
              color="#00ff00"
              lineWidth={2}
            />
          )}
        </group>
      )}

      {/* Label */}
      <Html position={[0, 1.5 * scale, 0]} center>
        <div className="bg-gray-900/80 px-2 py-1 rounded text-xs text-white whitespace-nowrap">
          Hinge: {((safeNumber(constraint.currentAngle) * 180) / Math.PI).toFixed(1)}°
        </div>
      </Html>
    </group>
  );
};

//==============================================================================
// Ball Socket Constraint Renderer
//==============================================================================

interface BallSocketRendererProps {
  constraint: Constraint;
  entities: Map<string, EntityState>;
  color: string;
  scale: number;
}

const BallSocketRenderer: React.FC<BallSocketRendererProps> = ({
  constraint,
  entities,
  color,
  scale,
}) => {
  const posA = getEntityWorldPosition(
    constraint.bodyA.entityId,
    entities,
    constraint.bodyA.localAnchor
  );

  return (
    <group position={posA}>
      {/* Ball joint sphere */}
      <mesh>
        <sphereGeometry args={[0.5 * scale, 16, 16]} />
        <meshBasicMaterial color={color} transparent opacity={0.5} />
      </mesh>

      {/* Socket ring */}
      <mesh>
        <torusGeometry args={[0.6 * scale, 0.1 * scale, 8, 32]} />
        <meshBasicMaterial color={color} />
      </mesh>

      {/* Label */}
      <Html position={[0, 1 * scale, 0]} center>
        <div className="bg-gray-900/80 px-2 py-1 rounded text-xs text-white">
          Ball Socket
        </div>
      </Html>
    </group>
  );
};

//==============================================================================
// Fixed Constraint Renderer
//==============================================================================

interface FixedRendererProps {
  constraint: Constraint;
  entities: Map<string, EntityState>;
  color: string;
  scale: number;
}

const FixedRenderer: React.FC<FixedRendererProps> = ({
  constraint,
  entities,
  color,
  scale,
}) => {
  const posA = getEntityWorldPosition(
    constraint.bodyA.entityId,
    entities,
    constraint.bodyA.localAnchor
  );

  const posB = constraint.bodyB
    ? getEntityWorldPosition(
        constraint.bodyB.entityId,
        entities,
        constraint.bodyB.localAnchor
      )
    : posA;

  return (
    <group>
      {/* Rigid connection line */}
      <Line
        points={[posA, posB]}
        color={color}
        lineWidth={4}
      />

      {/* Anchor cubes */}
      <mesh position={posA}>
        <boxGeometry args={[0.4 * scale, 0.4 * scale, 0.4 * scale]} />
        <meshBasicMaterial color={color} />
      </mesh>
      {constraint.bodyB && (
        <mesh position={posB}>
          <boxGeometry args={[0.4 * scale, 0.4 * scale, 0.4 * scale]} />
          <meshBasicMaterial color={color} />
        </mesh>
      )}
    </group>
  );
};

//==============================================================================
// Main ConstraintRenderer
//==============================================================================

interface ConstraintRendererProps {
  constraints: Constraint[];
  entities: Map<string, EntityState>;
  activeColor: string;
  brokenColor: string;
  scale: number;
  showLimits: boolean;
}

export const ConstraintRenderer: React.FC<ConstraintRendererProps> = ({
  constraints,
  entities,
  activeColor,
  brokenColor,
  scale,
  showLimits,
}) => {
  if (!constraints || constraints.length === 0) return null;

  return (
    <group name="constraints">
      {constraints.map((constraint) => {
        if (!isValidConstraint(constraint)) return null;
        const color = constraint.enabled ? activeColor : brokenColor;

        switch (constraint.type) {
          case 'distance':
          case 'distance_limit':
            return (
              <DistanceConstraintRenderer
                key={constraint.id}
                constraint={constraint as DistanceConstraint}
                entities={entities}
                color={color}
                scale={scale}
                showLimits={showLimits}
              />
            );

          case 'hinge':
            return (
              <HingeConstraintRenderer
                key={constraint.id}
                constraint={constraint as HingeConstraint}
                entities={entities}
                color={color}
                scale={scale}
                showLimits={showLimits}
              />
            );

          case 'ball_socket':
            return (
              <BallSocketRenderer
                key={constraint.id}
                constraint={constraint}
                entities={entities}
                color={color}
                scale={scale}
              />
            );

          case 'fixed':
            return (
              <FixedRenderer
                key={constraint.id}
                constraint={constraint}
                entities={entities}
                color={color}
                scale={scale}
              />
            );

          default:
            // Generic constraint visualization
            return (
              <DistanceConstraintRenderer
                key={constraint.id}
                constraint={{
                  ...constraint,
                  type: 'distance',
                  minDistance: 0,
                  maxDistance: 10,
                  currentDistance: 5,
                  stiffness: 1,
                  damping: 0,
                } as DistanceConstraint}
                entities={entities}
                color={color}
                scale={scale}
                showLimits={false}
              />
            );
        }
      })}
    </group>
  );
};

export default ConstraintRenderer;
