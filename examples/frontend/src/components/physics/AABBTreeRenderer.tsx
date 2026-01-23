import React, { useMemo } from 'react';
import * as THREE from 'three';
import { Line } from '@react-three/drei';
import type { AABBTree, AABBNode, AABB, Vec3 } from '@/types/physics';

//==============================================================================
// Helper Functions
//==============================================================================

function safeNumber(n: number | undefined | null): number {
  return isFinite(n as number) ? (n as number) : 0;
}

function safeVec3(v: Vec3 | undefined | null): THREE.Vector3 {
  if (!v) return new THREE.Vector3(0, 0, 0);
  return new THREE.Vector3(
    safeNumber(v.x),
    safeNumber(v.y),
    safeNumber(v.z)
  );
}

function isValidAABB(aabb: AABB | undefined | null): aabb is AABB {
  return aabb != null && aabb.min != null && aabb.max != null;
}

function getAABBEdges(aabb: AABB): [THREE.Vector3, THREE.Vector3][] {
  if (!isValidAABB(aabb)) {
    return [];
  }

  const min = safeVec3(aabb.min);
  const max = safeVec3(aabb.max);

  // 8 corners of the AABB
  const corners = [
    new THREE.Vector3(min.x, min.y, min.z), // 0: min corner
    new THREE.Vector3(max.x, min.y, min.z), // 1
    new THREE.Vector3(max.x, max.y, min.z), // 2
    new THREE.Vector3(min.x, max.y, min.z), // 3
    new THREE.Vector3(min.x, min.y, max.z), // 4
    new THREE.Vector3(max.x, min.y, max.z), // 5
    new THREE.Vector3(max.x, max.y, max.z), // 6: max corner
    new THREE.Vector3(min.x, max.y, max.z), // 7
  ].map(v => new THREE.Vector3(
    isFinite(v.x) ? v.x : 0,
    isFinite(v.y) ? v.y : 0,
    isFinite(v.z) ? v.z : 0
  ));

  // 12 edges of the AABB
  const edges: [THREE.Vector3, THREE.Vector3][] = [
    // Bottom face
    [corners[0], corners[1]],
    [corners[1], corners[2]],
    [corners[2], corners[3]],
    [corners[3], corners[0]],
    // Top face
    [corners[4], corners[5]],
    [corners[5], corners[6]],
    [corners[6], corners[7]],
    [corners[7], corners[4]],
    // Vertical edges
    [corners[0], corners[4]],
    [corners[1], corners[5]],
    [corners[2], corners[6]],
    [corners[3], corners[7]],
  ];

  return edges;
}

function interpolateColor(
  color1: string,
  color2: string,
  factor: number
): string {
  const c1 = new THREE.Color(color1);
  const c2 = new THREE.Color(color2);
  c1.lerp(c2, factor);
  return '#' + c1.getHexString();
}

//==============================================================================
// AABB Box Component
//==============================================================================

interface AABBBoxProps {
  aabb: AABB;
  color: string;
  opacity: number;
  showFaces: boolean;
}

const AABBBox: React.FC<AABBBoxProps> = ({ aabb, color, opacity, showFaces }) => {
  // Skip if AABB is invalid
  if (!isValidAABB(aabb)) return null;

  const edges = useMemo(() => getAABBEdges(aabb), [aabb]);

  const min = safeVec3(aabb.min);
  const max = safeVec3(aabb.max);

  const center = useMemo(() => {
    return new THREE.Vector3(
      (min.x + max.x) / 2,
      (min.y + max.y) / 2,
      (min.z + max.z) / 2
    );
  }, [min, max]);

  const size = useMemo(() => {
    return new THREE.Vector3(
      Math.abs(max.x - min.x),
      Math.abs(max.y - min.y),
      Math.abs(max.z - min.z)
    );
  }, [min, max]);

  return (
    <group>
      {/* Wireframe edges */}
      {edges.map((edge, index) => (
        <Line
          key={index}
          points={edge}
          color={color}
          lineWidth={1}
          transparent
          opacity={opacity + 0.3}
        />
      ))}

      {/* Semi-transparent faces */}
      {showFaces && (
        <mesh position={center}>
          <boxGeometry args={[size.x, size.y, size.z]} />
          <meshBasicMaterial
            color={color}
            transparent
            opacity={opacity}
            side={THREE.DoubleSide}
            depthWrite={false}
          />
        </mesh>
      )}
    </group>
  );
};

//==============================================================================
// Recursive Node Renderer
//==============================================================================

interface NodeRendererProps {
  nodes: AABBNode[];
  nodeId: number;
  currentDepth: number;
  maxDepth: number;
  showOnlyLeaves: boolean;
  nodeColor: string;
  leafColor: string;
  opacity: number;
}

const NodeRenderer: React.FC<NodeRendererProps> = ({
  nodes,
  nodeId,
  currentDepth,
  maxDepth,
  showOnlyLeaves,
  nodeColor,
  leafColor,
  opacity,
}) => {
  const node = nodes.find((n) => n.id === nodeId);
  if (!node || !isValidAABB(node.aabb)) return null;

  // Skip if we've exceeded max depth
  if (currentDepth > maxDepth) return null;

  const depthFactor = currentDepth / maxDepth;
  const color = node.isLeaf
    ? leafColor
    : interpolateColor(nodeColor, leafColor, depthFactor);

  const adjustedOpacity = opacity * (1 - depthFactor * 0.5);

  return (
    <group>
      {/* Render this node's AABB */}
      {(!showOnlyLeaves || node.isLeaf) && (
        <AABBBox
          aabb={node.aabb}
          color={color}
          opacity={adjustedOpacity}
          showFaces={node.isLeaf}
        />
      )}

      {/* Recursively render children */}
      {!node.isLeaf && node.left !== undefined && (
        <NodeRenderer
          nodes={nodes}
          nodeId={node.left}
          currentDepth={currentDepth + 1}
          maxDepth={maxDepth}
          showOnlyLeaves={showOnlyLeaves}
          nodeColor={nodeColor}
          leafColor={leafColor}
          opacity={opacity}
        />
      )}
      {!node.isLeaf && node.right !== undefined && (
        <NodeRenderer
          nodes={nodes}
          nodeId={node.right}
          currentDepth={currentDepth + 1}
          maxDepth={maxDepth}
          showOnlyLeaves={showOnlyLeaves}
          nodeColor={nodeColor}
          leafColor={leafColor}
          opacity={opacity}
        />
      )}
    </group>
  );
};

//==============================================================================
// Main AABBTreeRenderer
//==============================================================================

interface AABBTreeRendererProps {
  tree: AABBTree;
  maxDepth: number;
  showOnlyLeaves: boolean;
  nodeColor: string;
  leafColor: string;
  opacity: number;
}

export const AABBTreeRenderer: React.FC<AABBTreeRendererProps> = ({
  tree,
  maxDepth,
  showOnlyLeaves,
  nodeColor,
  leafColor,
  opacity,
}) => {
  if (!tree || tree.nodes.length === 0) return null;

  const effectiveMaxDepth = Math.min(maxDepth, tree.maxDepth);

  return (
    <group name="aabb-tree">
      <NodeRenderer
        nodes={tree.nodes}
        nodeId={tree.root}
        currentDepth={0}
        maxDepth={effectiveMaxDepth}
        showOnlyLeaves={showOnlyLeaves}
        nodeColor={nodeColor}
        leafColor={leafColor}
        opacity={opacity}
      />
    </group>
  );
};

export default AABBTreeRenderer;
