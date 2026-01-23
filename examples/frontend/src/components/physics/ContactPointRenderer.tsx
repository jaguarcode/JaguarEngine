import React from 'react';
import * as THREE from 'three';
import { Line } from '@react-three/drei';
import type { ContactPoint, Vec3 } from '@/types/physics';

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

function isValidContact(contact: ContactPoint | undefined | null): contact is ContactPoint {
  return contact != null && contact.position != null && contact.normal != null;
}

//==============================================================================
// Contact Point Marker
//==============================================================================

interface ContactMarkerProps {
  contact: ContactPoint;
  pointColor: string;
  normalColor: string;
  showNormal: boolean;
  normalScale?: number;
}

const ContactMarker: React.FC<ContactMarkerProps> = ({
  contact,
  pointColor,
  normalColor,
  showNormal,
  normalScale = 5,
}) => {
  // Validate contact data
  if (!isValidContact(contact)) return null;

  const position = safeVec3(contact.position);
  const normal = safeVec3(contact.normal);

  const normalEnd = position.clone().add(normal.clone().multiplyScalar(normalScale));

  // Penetration indicator - show as a sphere sized by penetration depth
  const penetration = isFinite(contact.penetration) ? contact.penetration : 0;
  const penetrationScale = Math.max(0.1, Math.min(penetration * 10, 2));

  return (
    <group>
      {/* Contact point sphere */}
      <mesh position={position}>
        <sphereGeometry args={[0.3, 16, 16]} />
        <meshBasicMaterial color={pointColor} />
      </mesh>

      {/* Penetration depth indicator (inner sphere) */}
      <mesh position={position}>
        <sphereGeometry args={[0.15 * penetrationScale, 8, 8]} />
        <meshBasicMaterial color="#ff0000" opacity={0.7} transparent />
      </mesh>

      {/* Contact normal arrow */}
      {showNormal && (
        <>
          {/* Normal line */}
          <Line
            points={[position, normalEnd]}
            color={normalColor}
            lineWidth={2}
          />

          {/* Arrowhead cone */}
          <mesh
            position={normalEnd}
            rotation={new THREE.Euler().setFromRotationMatrix(
              new THREE.Matrix4().lookAt(
                normalEnd,
                position,
                new THREE.Vector3(0, 1, 0)
              )
            )}
          >
            <coneGeometry args={[0.3, 0.8, 8]} />
            <meshBasicMaterial color={normalColor} />
          </mesh>
        </>
      )}

      {/* Ring at contact point to show contact plane */}
      <mesh
        position={position}
        rotation={new THREE.Euler().setFromRotationMatrix(
          new THREE.Matrix4().lookAt(
            position,
            position.clone().add(normal),
            new THREE.Vector3(0, 1, 0)
          )
        )}
      >
        <ringGeometry args={[0.8, 1.0, 32]} />
        <meshBasicMaterial
          color={pointColor}
          side={THREE.DoubleSide}
          transparent
          opacity={0.5}
        />
      </mesh>
    </group>
  );
};

//==============================================================================
// Contact Pair Lines
//==============================================================================

interface ContactPairLinesProps {
  contacts: ContactPoint[];
  color: string;
}

const ContactPairLines: React.FC<ContactPairLinesProps> = ({ contacts, color }) => {
  // Group contacts by entity pairs
  const pairMap = new Map<string, ContactPoint[]>();

  contacts.forEach((contact) => {
    if (!isValidContact(contact)) return;
    const key = [contact.entityA, contact.entityB].sort().join('-');
    const existing = pairMap.get(key) || [];
    existing.push(contact);
    pairMap.set(key, existing);
  });

  return (
    <group name="contact-pair-lines">
      {Array.from(pairMap.values()).map((pairContacts, index) => {
        if (pairContacts.length < 2) return null;

        // Connect all contact points in a pair with lines
        const points = pairContacts.map((c) => safeVec3(c.position));

        return (
          <Line
            key={index}
            points={points}
            color={color}
            lineWidth={1}
            dashed
            dashSize={0.5}
            gapSize={0.3}
            transparent
            opacity={0.5}
          />
        );
      })}
    </group>
  );
};

//==============================================================================
// Main ContactPointRenderer
//==============================================================================

interface ContactPointRendererProps {
  contacts: ContactPoint[];
  showNormals: boolean;
  pointColor: string;
  normalColor: string;
  normalScale?: number;
}

export const ContactPointRenderer: React.FC<ContactPointRendererProps> = ({
  contacts,
  showNormals,
  pointColor,
  normalColor,
  normalScale = 5,
}) => {
  if (contacts.length === 0) return null;

  return (
    <group name="contact-points">
      {/* Individual contact markers */}
      {contacts.map((contact, index) => (
        <ContactMarker
          key={`contact-${index}`}
          contact={contact}
          pointColor={pointColor}
          normalColor={normalColor}
          showNormal={showNormals}
          normalScale={normalScale}
        />
      ))}

      {/* Lines connecting contacts in same collision pair */}
      <ContactPairLines contacts={contacts} color={pointColor} />

      {/* Legend / Stats overlay would go in a separate HUD component */}
    </group>
  );
};

export default ContactPointRenderer;
