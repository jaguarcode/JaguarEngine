import React, { useMemo } from 'react';
import * as THREE from 'three';
import { Line, Html, Cone } from '@react-three/drei';
import type { EntityForces, ForceType, Vec3 } from '@/types/physics';

//==============================================================================
// Helper Functions
//==============================================================================

function safeVec3(v: Vec3 | undefined | null): THREE.Vector3 {
  if (!v) return new THREE.Vector3(0, 0, 0);
  return new THREE.Vector3(
    isFinite(v.x) ? v.x : 0,
    isFinite(v.y) ? v.y : 0,
    isFinite(v.z) ? v.z : 0
  );
}

//==============================================================================
// Force Arrow Component
//==============================================================================

interface ForceArrowProps {
  origin: THREE.Vector3;
  direction: THREE.Vector3;
  magnitude: number;
  scale: number;
  color: string;
  label?: string;
  showLabel: boolean;
}

const ForceArrow: React.FC<ForceArrowProps> = ({
  origin,
  direction,
  magnitude,
  scale,
  color,
  label,
  showLabel,
}) => {
  const visualLength = magnitude * scale;

  // Don't render if too small
  if (visualLength < 0.1) return null;

  const end = useMemo(() => {
    return origin.clone().add(direction.clone().multiplyScalar(visualLength));
  }, [origin, direction, visualLength]);

  // Arrowhead position and orientation
  const arrowheadPosition = end;
  const arrowheadRotation = useMemo(() => {
    const matrix = new THREE.Matrix4();
    matrix.lookAt(end, origin, new THREE.Vector3(0, 1, 0));
    const euler = new THREE.Euler();
    euler.setFromRotationMatrix(matrix);
    euler.x += Math.PI / 2; // Cone points along Y by default
    return euler;
  }, [origin, end]);

  const arrowheadSize = Math.min(visualLength * 0.15, 1);

  return (
    <group>
      {/* Arrow shaft */}
      <Line
        points={[origin, end]}
        color={color}
        lineWidth={2}
      />

      {/* Arrowhead */}
      <Cone
        args={[arrowheadSize * 0.5, arrowheadSize, 8]}
        position={arrowheadPosition}
        rotation={arrowheadRotation}
      >
        <meshBasicMaterial color={color} />
      </Cone>

      {/* Label */}
      {showLabel && label && (
        <Html position={end.clone().add(direction.clone().multiplyScalar(1))} center>
          <div className="bg-gray-900/80 px-1.5 py-0.5 rounded text-[10px] text-white whitespace-nowrap">
            {label}: {magnitude.toFixed(1)} N
          </div>
        </Html>
      )}
    </group>
  );
};

//==============================================================================
// Force Type Color Mapping
//==============================================================================

function getForceColor(
  forceType: ForceType,
  colors: Record<string, string>
): string {
  const colorMap: Record<ForceType, string> = {
    gravity: colors.forceGravity || '#8888ff',
    aerodynamic_lift: colors.forceLift || '#00ff00',
    aerodynamic_drag: colors.forceDrag || '#ff8800',
    thrust: colors.forceThrust || '#ffff00',
    buoyancy: colors.forceBuoyancy || '#00ffff',
    hydrodynamic: '#00aaff',
    ground_contact: '#aa8844',
    constraint: '#ff00ff',
    external: '#ffffff',
    total: colors.forceTotal || '#ffffff',
  };

  return colorMap[forceType] || '#ffffff';
}

function getForceLabel(forceType: ForceType): string {
  const labelMap: Record<ForceType, string> = {
    gravity: 'Gravity',
    aerodynamic_lift: 'Lift',
    aerodynamic_drag: 'Drag',
    thrust: 'Thrust',
    buoyancy: 'Buoyancy',
    hydrodynamic: 'Hydro',
    ground_contact: 'Ground',
    constraint: 'Constraint',
    external: 'External',
    total: 'Total',
  };

  return labelMap[forceType] || forceType;
}

//==============================================================================
// Torque Visualization
//==============================================================================

interface TorqueVisualizationProps {
  origin: THREE.Vector3;
  torque: THREE.Vector3;
  scale: number;
  color: string;
}

const TorqueVisualization: React.FC<TorqueVisualizationProps> = ({
  origin,
  torque,
  scale,
  color,
}) => {
  const magnitude = torque.length();
  if (magnitude < 0.1) return null;

  const visualRadius = Math.min(magnitude * scale * 0.1, 5);
  const axis = torque.clone().normalize();

  // Create rotation to align torus with torque axis
  const quaternion = new THREE.Quaternion().setFromUnitVectors(
    new THREE.Vector3(0, 0, 1),
    axis
  );

  return (
    <group position={origin} quaternion={quaternion}>
      {/* Torque ring */}
      <mesh>
        <torusGeometry args={[visualRadius, visualRadius * 0.1, 8, 32, Math.PI * 1.5]} />
        <meshBasicMaterial color={color} transparent opacity={0.7} />
      </mesh>

      {/* Direction arrow on ring */}
      <Cone
        args={[visualRadius * 0.15, visualRadius * 0.3, 8]}
        position={[visualRadius, 0, 0]}
        rotation={[0, 0, -Math.PI / 2]}
      >
        <meshBasicMaterial color={color} />
      </Cone>
    </group>
  );
};

//==============================================================================
// Main ForceVectorRenderer
//==============================================================================

interface ForceVectorRendererProps {
  forces: EntityForces;
  scale: number;
  threshold: number;
  showLabels: boolean;
  forceTypes: ForceType[];
  colors: Record<string, string>;
  showTorques?: boolean;
}

export const ForceVectorRenderer: React.FC<ForceVectorRendererProps> = ({
  forces,
  scale,
  threshold,
  showLabels,
  forceTypes,
  colors,
  showTorques = true,
}) => {
  // Early return if forces is invalid
  if (!forces || !forces.forces) return null;

  const filteredForces = useMemo(() => {
    return (forces.forces ?? []).filter(
      (f) => f && forceTypes.includes(f.type) && isFinite(f.magnitude) && f.magnitude > threshold
    );
  }, [forces.forces, forceTypes, threshold]);

  // Calculate total force if not provided separately
  const totalForce = useMemo(() => {
    const total = new THREE.Vector3();
    filteredForces.forEach((f) => {
      if (f.force) {
        total.add(safeVec3(f.force));
      }
    });
    return total;
  }, [filteredForces]);

  const totalTorque = useMemo(() => {
    return safeVec3(forces.totalTorque);
  }, [forces.totalTorque]);

  return (
    <group name={`forces-${forces.entityId}`}>
      {/* Individual force vectors */}
      {filteredForces.map((forceVec, index) => {
        if (!forceVec || !forceVec.applicationPoint || !forceVec.force) return null;

        const origin = safeVec3(forceVec.applicationPoint);
        const forceDir = safeVec3(forceVec.force);

        // Skip if force vector is zero length
        if (forceDir.length() < 0.001) return null;

        const direction = forceDir.normalize();

        const color = forceVec.color || getForceColor(forceVec.type, colors);
        const label = getForceLabel(forceVec.type);

        return (
          <ForceArrow
            key={`${forces.entityId}-${forceVec.type}-${index}`}
            origin={origin}
            direction={direction}
            magnitude={forceVec.magnitude}
            scale={scale}
            color={color}
            label={label}
            showLabel={showLabels}
          />
        );
      })}

      {/* Total force vector (if showing totals) */}
      {forceTypes.includes('total') && totalForce.length() > threshold && (
        <ForceArrow
          origin={
            filteredForces.length > 0 && filteredForces[0].applicationPoint
              ? safeVec3(filteredForces[0].applicationPoint)
              : new THREE.Vector3()
          }
          direction={totalForce.clone().normalize()}
          magnitude={totalForce.length()}
          scale={scale}
          color={colors.forceTotal || '#ffffff'}
          label="Total"
          showLabel={showLabels}
        />
      )}

      {/* Torque visualization */}
      {showTorques && totalTorque.length() > threshold && filteredForces.length > 0 && filteredForces[0].applicationPoint && (
        <TorqueVisualization
          origin={safeVec3(filteredForces[0].applicationPoint)}
          torque={totalTorque}
          scale={scale * 10}
          color="#ff88ff"
        />
      )}
    </group>
  );
};

export default ForceVectorRenderer;
