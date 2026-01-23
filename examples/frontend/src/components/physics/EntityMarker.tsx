import React, { useRef, useMemo } from 'react';
import * as THREE from 'three';
import { useFrame } from '@react-three/fiber';
import { Billboard, Text, Line } from '@react-three/drei';
import type { EntityState, Domain } from '@/types';

//==============================================================================
// Domain Colors & Shapes
//==============================================================================

const DOMAIN_COLORS: Record<Domain, string> = {
  air: '#00aaff',
  land: '#88aa44',
  sea: '#0066cc',
  space: '#aa44ff',
};

const DOMAIN_EMISSIVE: Record<Domain, string> = {
  air: '#004488',
  land: '#446622',
  sea: '#003366',
  space: '#662288',
};

//==============================================================================
// Aircraft Model
//==============================================================================

const AircraftModel: React.FC<{ color: string; emissive: string }> = ({ color, emissive }) => {
  return (
    <group>
      {/* Fuselage */}
      <mesh>
        <cylinderGeometry args={[0.5, 0.3, 4, 8]} />
        <meshStandardMaterial color={color} emissive={emissive} emissiveIntensity={0.2} />
      </mesh>

      {/* Wings */}
      <mesh position={[0, 0, 0]} rotation={[0, 0, Math.PI / 2]}>
        <boxGeometry args={[0.1, 6, 1]} />
        <meshStandardMaterial color={color} emissive={emissive} emissiveIntensity={0.2} />
      </mesh>

      {/* Tail */}
      <mesh position={[0, -1.5, 0]}>
        <boxGeometry args={[0.1, 2, 0.5]} />
        <meshStandardMaterial color={color} emissive={emissive} emissiveIntensity={0.2} />
      </mesh>

      {/* Vertical stabilizer */}
      <mesh position={[0, -1.5, 0.5]}>
        <boxGeometry args={[0.1, 1, 1]} />
        <meshStandardMaterial color={color} emissive={emissive} emissiveIntensity={0.2} />
      </mesh>
    </group>
  );
};

//==============================================================================
// Ground Vehicle Model
//==============================================================================

const VehicleModel: React.FC<{ color: string; emissive: string }> = ({ color, emissive }) => {
  return (
    <group>
      {/* Body */}
      <mesh position={[0, 0.5, 0]}>
        <boxGeometry args={[2, 1, 4]} />
        <meshStandardMaterial color={color} emissive={emissive} emissiveIntensity={0.2} />
      </mesh>

      {/* Wheels */}
      {[
        [-1, 0, 1.2],
        [1, 0, 1.2],
        [-1, 0, -1.2],
        [1, 0, -1.2],
      ].map((pos, i) => (
        <mesh key={i} position={pos as [number, number, number]} rotation={[0, 0, Math.PI / 2]}>
          <cylinderGeometry args={[0.4, 0.4, 0.3, 16]} />
          <meshStandardMaterial color="#333333" />
        </mesh>
      ))}
    </group>
  );
};

//==============================================================================
// Ship Model
//==============================================================================

const ShipModel: React.FC<{ color: string; emissive: string }> = ({ color, emissive }) => {
  return (
    <group>
      {/* Hull */}
      <mesh>
        <boxGeometry args={[3, 1, 8]} />
        <meshStandardMaterial color={color} emissive={emissive} emissiveIntensity={0.2} />
      </mesh>

      {/* Superstructure */}
      <mesh position={[0, 1, -1]}>
        <boxGeometry args={[2, 1.5, 3]} />
        <meshStandardMaterial color={color} emissive={emissive} emissiveIntensity={0.2} />
      </mesh>

      {/* Mast */}
      <mesh position={[0, 2.5, -1]}>
        <cylinderGeometry args={[0.1, 0.1, 2, 8]} />
        <meshStandardMaterial color="#888888" />
      </mesh>
    </group>
  );
};

//==============================================================================
// Spacecraft Model
//==============================================================================

const SpacecraftModel: React.FC<{ color: string; emissive: string }> = ({ color, emissive }) => {
  return (
    <group>
      {/* Body */}
      <mesh>
        <boxGeometry args={[2, 2, 4]} />
        <meshStandardMaterial color={color} emissive={emissive} emissiveIntensity={0.3} />
      </mesh>

      {/* Solar panels */}
      <mesh position={[4, 0, 0]}>
        <boxGeometry args={[4, 0.1, 2]} />
        <meshStandardMaterial color="#2244aa" emissive="#001144" emissiveIntensity={0.5} />
      </mesh>
      <mesh position={[-4, 0, 0]}>
        <boxGeometry args={[4, 0.1, 2]} />
        <meshStandardMaterial color="#2244aa" emissive="#001144" emissiveIntensity={0.5} />
      </mesh>

      {/* Antenna */}
      <mesh position={[0, 1.5, 0]}>
        <coneGeometry args={[0.8, 0.5, 16]} />
        <meshStandardMaterial color="#aaaaaa" />
      </mesh>
    </group>
  );
};

//==============================================================================
// Selection Ring
//==============================================================================

const SelectionRing: React.FC<{ radius: number }> = ({ radius }) => {
  const ringRef = useRef<THREE.Mesh>(null);

  useFrame((state) => {
    if (ringRef.current) {
      ringRef.current.rotation.z = state.clock.elapsedTime * 0.5;
    }
  });

  return (
    <mesh ref={ringRef} rotation={[Math.PI / 2, 0, 0]} position={[0, 0.1, 0]}>
      <ringGeometry args={[radius * 0.9, radius, 32]} />
      <meshBasicMaterial color="#ffff00" transparent opacity={0.6} side={THREE.DoubleSide} />
    </mesh>
  );
};

//==============================================================================
// Velocity Vector
//==============================================================================

interface VelocityVectorProps {
  velocity: { north: number; east: number; down: number };
  scale: number;
  color: string;
}

const VelocityVector: React.FC<VelocityVectorProps> = ({ velocity, scale, color }) => {
  // Validate velocity values - skip rendering if any are NaN or undefined
  const north = velocity?.north ?? 0;
  const east = velocity?.east ?? 0;
  const down = velocity?.down ?? 0;

  if (!isFinite(north) || !isFinite(east) || !isFinite(down)) {
    return null;
  }

  const speed = Math.sqrt(north ** 2 + east ** 2 + down ** 2);

  if (speed < 0.1 || !isFinite(speed)) return null;

  const direction = new THREE.Vector3(east, -down, north).normalize();

  // Check for NaN after normalization (can happen with zero vector)
  if (!isFinite(direction.x) || !isFinite(direction.y) || !isFinite(direction.z)) {
    return null;
  }

  const length = Math.min(speed * scale, 20);
  const end = direction.clone().multiplyScalar(length);

  return (
    <group>
      <Line points={[new THREE.Vector3(0, 0, 0), end]} color={color} lineWidth={2} />
      <mesh position={end}>
        <coneGeometry args={[0.3, 0.8, 8]} />
        <meshBasicMaterial color={color} />
      </mesh>
    </group>
  );
};

//==============================================================================
// Main EntityMarker Component
//==============================================================================

interface EntityMarkerProps {
  entity: EntityState;
  selected: boolean;
  onClick: () => void;
  showVelocity?: boolean;
  showLabel?: boolean;
  scale?: number;
}

export const EntityMarker: React.FC<EntityMarkerProps> = ({
  entity,
  selected,
  onClick,
  showVelocity = true,
  showLabel = true,
  scale = 1,
}) => {
  const groupRef = useRef<THREE.Group>(null);

  // Convert geodetic position to local coordinates (simplified)
  const position = useMemo(() => {
    const lon = entity.position?.longitude ?? 0;
    const lat = entity.position?.latitude ?? 0;
    const alt = entity.position?.altitude ?? 0;

    // Validate and use safe values
    const x = isFinite(lon) ? lon * 111000 : 0;
    const y = isFinite(alt) ? alt : 0;
    const z = isFinite(lat) ? lat * 111000 : 0;

    return new THREE.Vector3(x, y, z);
  }, [entity.position]);

  // Convert orientation
  const rotation = useMemo(() => {
    const pitch = entity.orientation?.pitch ?? 0;
    const yaw = entity.orientation?.yaw ?? 0;
    const roll = entity.orientation?.roll ?? 0;

    return new THREE.Euler(
      isFinite(pitch) ? pitch : 0,
      isFinite(yaw) ? yaw : 0,
      isFinite(roll) ? roll : 0,
      'YXZ'
    );
  }, [entity.orientation]);

  const color = DOMAIN_COLORS[entity.domain];
  const emissive = DOMAIN_EMISSIVE[entity.domain];

  // Determine model size for selection ring
  const modelSize = entity.domain === 'space' ? 5 : entity.domain === 'sea' ? 6 : 4;

  return (
    <group ref={groupRef} position={position}>
      {/* Clickable area */}
      <mesh onClick={onClick} visible={false}>
        <sphereGeometry args={[modelSize * scale, 8, 8]} />
        <meshBasicMaterial />
      </mesh>

      {/* Oriented model group */}
      <group rotation={rotation} scale={scale}>
        {/* Domain-specific model */}
        {entity.domain === 'air' && <AircraftModel color={color} emissive={emissive} />}
        {entity.domain === 'land' && <VehicleModel color={color} emissive={emissive} />}
        {entity.domain === 'sea' && <ShipModel color={color} emissive={emissive} />}
        {entity.domain === 'space' && <SpacecraftModel color={color} emissive={emissive} />}

        {/* Selection ring */}
        {selected && <SelectionRing radius={modelSize} />}
      </group>

      {/* Velocity vector (world-aligned) */}
      {showVelocity && (
        <VelocityVector
          velocity={entity.velocity}
          scale={0.05}
          color="#00ff88"
        />
      )}

      {/* Label billboard */}
      {showLabel && (
        <Billboard position={[0, modelSize * scale + 2, 0]}>
          <Text
            fontSize={1.5}
            color={selected ? '#ffff00' : '#ffffff'}
            anchorX="center"
            anchorY="bottom"
            outlineWidth={0.1}
            outlineColor="#000000"
          >
            {entity.name}
          </Text>
          <Text
            fontSize={0.8}
            color="#aaaaaa"
            anchorX="center"
            anchorY="top"
            position={[0, -0.3, 0]}
          >
            {entity.domain.toUpperCase()}
          </Text>
        </Billboard>
      )}

      {/* Inactive indicator */}
      {!entity.isActive && (
        <mesh position={[modelSize * scale + 1, 0, 0]}>
          <sphereGeometry args={[0.5, 8, 8]} />
          <meshBasicMaterial color="#ff4444" />
        </mesh>
      )}
    </group>
  );
};

export default EntityMarker;
