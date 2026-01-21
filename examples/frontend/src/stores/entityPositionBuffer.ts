/**
 * High-performance entity position buffer with smooth interpolation.
 *
 * This buffer stores entity position data from WebSocket messages and provides
 * smoothly interpolated positions to the rendering loop (Cesium).
 *
 * Key features:
 * - Client-side interpolation for smooth movement despite network jitter
 * - Predictive extrapolation when data is delayed
 * - No React state - pure JavaScript for minimal overhead
 * - RAF-driven rendering at display refresh rate
 *
 * Architecture:
 * - WebSocket → updatePositions() (fast path, no React)
 * - RAF loop → interpolateAll() → callback → Cesium update
 * - React state updated separately at lower frequency for UI components
 */

import type { Domain, EntityKind } from '@/types';

// Public interface for rendering (matches what CesiumViewer expects)
export interface EntityPosition {
  id: string;
  name: string;
  domain: Domain;
  kind: EntityKind;
  isActive: boolean;
  // Interpolated position
  latitude: number;
  longitude: number;
  altitude: number;
  // Orientation (using EulerAngles convention: roll, pitch, yaw)
  roll: number;
  pitch: number;
  yaw: number;  // heading
  // Velocity for display
  speed: number;
}

// Internal state for interpolation
interface InterpolatedEntity {
  id: string;
  name: string;
  domain: Domain;
  kind: EntityKind;
  isActive: boolean;
  // Target position (from server)
  targetLat: number;
  targetLon: number;
  targetAlt: number;
  // Previous position (interpolation start)
  prevLat: number;
  prevLon: number;
  prevAlt: number;
  // Current interpolated position (for rendering)
  lat: number;
  lon: number;
  alt: number;
  // Velocity estimation for extrapolation (degrees/second, meters/second)
  velLat: number;
  velLon: number;
  velAlt: number;
  // Timestamps
  lastUpdateTime: number;
  prevUpdateTime: number;
  // Actual measured update interval (adaptive)
  measuredInterval: number;
  // Orientation
  roll: number;
  pitch: number;
  yaw: number;
  // Speed
  speed: number;
}

// Validate that a number is finite (not NaN, not Infinity)
function isValidCoord(n: number): boolean {
  return Number.isFinite(n);
}

// Linear interpolation
function lerp(a: number, b: number, t: number): number {
  return a + (b - a) * t;
}

class EntityPositionBuffer {
  // All entity states with interpolation data
  private entities: Map<string, InterpolatedEntity> = new Map();

  // Output buffer for rendering (avoids creating new Map each frame)
  private outputBuffer: Map<string, EntityPosition> = new Map();

  // Interpolation configuration - tuned for ~30Hz server updates
  private readonly DEFAULT_LERP_DURATION = 40; // ms - slightly longer than one 30Hz frame (33ms)
  private readonly EXTRAPOLATE_MAX = 100; // ms - max time to extrapolate
  private readonly VELOCITY_SMOOTHING = 0.5; // higher = more responsive to velocity changes
  private readonly INTERVAL_SMOOTHING = 0.2; // smoothing for adaptive interval measurement

  // Callback for position updates (used by CesiumViewer)
  private updateCallback: ((positions: Map<string, EntityPosition>) => void) | null = null;

  // RAF handle
  private rafHandle: number | null = null;
  private isRunning: boolean = false;

  // Version counter for change detection
  private version: number = 0;

  /**
   * Update positions from WebSocket message.
   * Called from useWebSocket hook - this is the fast path.
   */
  updatePositions(incoming: Array<{
    id: string;
    name: string;
    domain: Domain;
    kind: EntityKind;
    isActive: boolean;
    position: { latitude: number; longitude: number; altitude: number };
    orientation?: { roll: number; pitch: number; yaw: number };
    velocity?: { speed: number };
  }>): void {
    if (!incoming || incoming.length === 0) {
      return;
    }

    const now = performance.now();
    const incomingIds = new Set<string>();
    this.version++;

    for (const entity of incoming) {
      const lat = entity.position?.latitude;
      const lon = entity.position?.longitude;
      const alt = entity.position?.altitude ?? 0;

      if (!isValidCoord(lat) || !isValidCoord(lon)) {
        continue;
      }

      incomingIds.add(entity.id);
      const existing = this.entities.get(entity.id);

      if (existing) {
        // Calculate time delta for velocity estimation
        const dtMs = now - existing.lastUpdateTime;
        const dt = dtMs / 1000; // seconds

        // Update adaptive interval measurement
        if (dtMs > 1 && dtMs < 500) {
          existing.measuredInterval = existing.measuredInterval * (1 - this.INTERVAL_SMOOTHING)
                                     + dtMs * this.INTERVAL_SMOOTHING;
        }

        if (dt > 0.001) {
          // Estimate velocity with smoothing
          const newVelLat = (lat - existing.targetLat) / dt;
          const newVelLon = (lon - existing.targetLon) / dt;
          const newVelAlt = (alt - existing.targetAlt) / dt;

          const alpha = this.VELOCITY_SMOOTHING;
          existing.velLat = existing.velLat * (1 - alpha) + newVelLat * alpha;
          existing.velLon = existing.velLon * (1 - alpha) + newVelLon * alpha;
          existing.velAlt = existing.velAlt * (1 - alpha) + newVelAlt * alpha;
        }

        // Store current target as previous
        existing.prevLat = existing.targetLat;
        existing.prevLon = existing.targetLon;
        existing.prevAlt = existing.targetAlt;
        existing.prevUpdateTime = existing.lastUpdateTime;

        // Set new target
        existing.targetLat = lat;
        existing.targetLon = lon;
        existing.targetAlt = alt;
        existing.lastUpdateTime = now;

        // Update metadata
        existing.name = entity.name;
        existing.domain = entity.domain;
        existing.kind = entity.kind;
        existing.isActive = entity.isActive;
        existing.roll = entity.orientation?.roll ?? existing.roll;
        existing.pitch = entity.orientation?.pitch ?? existing.pitch;
        existing.yaw = entity.orientation?.yaw ?? existing.yaw;
        existing.speed = entity.velocity?.speed ?? existing.speed;
      } else {
        // New entity - initialize at target position
        this.entities.set(entity.id, {
          id: entity.id,
          name: entity.name,
          domain: entity.domain,
          kind: entity.kind,
          isActive: entity.isActive,
          targetLat: lat,
          targetLon: lon,
          targetAlt: alt,
          prevLat: lat,
          prevLon: lon,
          prevAlt: alt,
          lat: lat,
          lon: lon,
          alt: alt,
          velLat: 0,
          velLon: 0,
          velAlt: 0,
          lastUpdateTime: now,
          prevUpdateTime: now,
          measuredInterval: this.DEFAULT_LERP_DURATION,
          roll: entity.orientation?.roll ?? 0,
          pitch: entity.orientation?.pitch ?? 0,
          yaw: entity.orientation?.yaw ?? 0,
          speed: entity.velocity?.speed ?? 0,
        });
      }
    }

    // Remove entities that are no longer in the update
    for (const id of this.entities.keys()) {
      if (!incomingIds.has(id)) {
        this.entities.delete(id);
      }
    }
  }

  /**
   * Perform interpolation and return positions for rendering.
   * Called every RAF frame.
   */
  private interpolateAll(): Map<string, EntityPosition> {
    const now = performance.now();

    this.outputBuffer.clear();

    for (const entity of this.entities.values()) {
      const elapsed = now - entity.lastUpdateTime;
      const lerpDuration = Math.max(this.DEFAULT_LERP_DURATION, entity.measuredInterval * 1.2);

      if (elapsed <= lerpDuration) {
        // Normal interpolation
        const t = Math.min(1, elapsed / lerpDuration);
        entity.lat = lerp(entity.prevLat, entity.targetLat, t);
        entity.lon = lerp(entity.prevLon, entity.targetLon, t);
        entity.alt = lerp(entity.prevAlt, entity.targetAlt, t);
      } else if (elapsed <= lerpDuration + this.EXTRAPOLATE_MAX) {
        // Extrapolation
        const extraTime = (elapsed - lerpDuration) / 1000;
        const dampFactor = 1 - (elapsed - lerpDuration) / this.EXTRAPOLATE_MAX;

        entity.lat = entity.targetLat + entity.velLat * extraTime * dampFactor;
        entity.lon = entity.targetLon + entity.velLon * extraTime * dampFactor;
        entity.alt = entity.targetAlt + entity.velAlt * extraTime * dampFactor;
      }
      // else: keep at last known position

      this.outputBuffer.set(entity.id, {
        id: entity.id,
        name: entity.name,
        domain: entity.domain,
        kind: entity.kind,
        isActive: entity.isActive,
        latitude: entity.lat,
        longitude: entity.lon,
        altitude: entity.alt,
        roll: entity.roll,
        pitch: entity.pitch,
        yaw: entity.yaw,
        speed: entity.speed,
      });
    }

    return this.outputBuffer;
  }

  /**
   * Clear all positions
   */
  clear(): void {
    this.entities.clear();
    this.outputBuffer.clear();
    this.version++;
  }

  /**
   * Get current positions (returns last interpolated state)
   */
  getPositions(): Map<string, EntityPosition> {
    return this.outputBuffer;
  }

  /**
   * Get the number of entities
   */
  get size(): number {
    return this.entities.size;
  }

  /**
   * Get version for change detection
   */
  getVersion(): number {
    return this.version;
  }

  /**
   * Set callback for position updates.
   * The callback is called from requestAnimationFrame.
   */
  setUpdateCallback(callback: ((positions: Map<string, EntityPosition>) => void) | null): void {
    this.updateCallback = callback;

    if (callback && !this.isRunning) {
      this.startRenderLoop();
    } else if (!callback && this.isRunning) {
      this.stopRenderLoop();
    }
  }

  /**
   * Start the render loop
   */
  private startRenderLoop(): void {
    if (this.isRunning) return;
    this.isRunning = true;
    this.renderLoop();
  }

  /**
   * Stop the render loop
   */
  private stopRenderLoop(): void {
    this.isRunning = false;
    if (this.rafHandle !== null) {
      cancelAnimationFrame(this.rafHandle);
      this.rafHandle = null;
    }
  }

  /**
   * The main render loop - runs at display refresh rate
   */
  private renderLoop = (): void => {
    if (!this.isRunning) return;

    // Schedule next frame FIRST
    this.rafHandle = requestAnimationFrame(this.renderLoop);

    // Always interpolate and call callback
    if (this.entities.size > 0 && this.updateCallback) {
      const positions = this.interpolateAll();
      if (positions.size > 0) {
        this.updateCallback(positions);
      }
    }
  };
}

// Singleton instance
export const entityPositionBuffer = new EntityPositionBuffer();
