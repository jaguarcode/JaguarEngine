/**
 * Command Service
 *
 * Parses natural language commands and translates them into
 * AI agent configurations (behavior mode, waypoints, targets).
 */

import type { AIBehaviorMode, AIAgentConfig } from '@/types/aiAgent';

// ============================================================================
// Korean Location Database
// ============================================================================

interface Location {
  name: string;
  nameKr: string;
  latitude: number;
  longitude: number;
  defaultAltitude: number; // meters
}

const LOCATIONS: Location[] = [
  // Major Cities
  { name: 'seoul', nameKr: '서울', latitude: 37.5665, longitude: 126.9780, defaultAltitude: 3000 },
  { name: 'busan', nameKr: '부산', latitude: 35.1796, longitude: 129.0756, defaultAltitude: 3000 },
  { name: 'pusan', nameKr: '부산', latitude: 35.1796, longitude: 129.0756, defaultAltitude: 3000 }, // alias
  { name: 'incheon', nameKr: '인천', latitude: 37.4563, longitude: 126.7052, defaultAltitude: 3000 },
  { name: 'daegu', nameKr: '대구', latitude: 35.8714, longitude: 128.6014, defaultAltitude: 3000 },
  { name: 'daejeon', nameKr: '대전', latitude: 36.3504, longitude: 127.3845, defaultAltitude: 3000 },
  { name: 'gwangju', nameKr: '광주', latitude: 35.1595, longitude: 126.8526, defaultAltitude: 3000 },
  { name: 'ulsan', nameKr: '울산', latitude: 35.5384, longitude: 129.3114, defaultAltitude: 3000 },
  { name: 'sejong', nameKr: '세종', latitude: 36.4800, longitude: 127.2890, defaultAltitude: 3000 },

  // Airports
  { name: 'gimpo', nameKr: '김포', latitude: 37.5585, longitude: 126.7906, defaultAltitude: 2000 },
  { name: 'incheon airport', nameKr: '인천공항', latitude: 37.4602, longitude: 126.4407, defaultAltitude: 2000 },
  { name: 'gimhae', nameKr: '김해', latitude: 35.1795, longitude: 128.9382, defaultAltitude: 2000 },
  { name: 'jeju', nameKr: '제주', latitude: 33.5104, longitude: 126.4914, defaultAltitude: 2000 },

  // Military Bases (approximate public locations)
  { name: 'osan', nameKr: '오산', latitude: 37.0905, longitude: 127.0304, defaultAltitude: 3000 },
  { name: 'kunsan', nameKr: '군산', latitude: 35.9039, longitude: 126.6158, defaultAltitude: 3000 },
  { name: 'pyeongtaek', nameKr: '평택', latitude: 36.9921, longitude: 127.0852, defaultAltitude: 3000 },

  // Landmarks
  { name: 'dmz', nameKr: 'DMZ', latitude: 37.9500, longitude: 127.0000, defaultAltitude: 5000 },
  { name: 'dokdo', nameKr: '독도', latitude: 37.2426, longitude: 131.8597, defaultAltitude: 3000 },
  { name: 'jeju island', nameKr: '제주도', latitude: 33.4890, longitude: 126.4983, defaultAltitude: 3000 },
];

// ============================================================================
// Command Patterns
// ============================================================================

interface CommandPattern {
  patterns: RegExp[];
  behavior: AIBehaviorMode;
  requiresLocation: boolean;
  requiresTarget: boolean;
  description: string;
}

const COMMAND_PATTERNS: CommandPattern[] = [
  {
    patterns: [
      /fly\s+to\s+(.+)/i,
      /go\s+to\s+(.+)/i,
      /navigate\s+to\s+(.+)/i,
      /head\s+to\s+(.+)/i,
      /move\s+to\s+(.+)/i,
      /(.+)(으)?로\s*(이동|비행|가)/i, // Korean: X로 이동/비행/가
    ],
    behavior: 'patrol',
    requiresLocation: true,
    requiresTarget: false,
    description: 'Navigate to location',
  },
  {
    patterns: [
      /patrol\s+(?:around\s+|across\s+|over\s+)?(.+)/i,
      /circle\s+(?:around\s+|over\s+)?(.+)/i,
      /orbit\s+(?:around\s+|over\s+)?(.+)/i,
      /(.+)\s*(?:지역|상공)?\s*(?:순찰|정찰)/i, // Korean: X 지역 순찰
    ],
    behavior: 'patrol',
    requiresLocation: true,
    requiresTarget: false,
    description: 'Patrol around location',
  },
  {
    patterns: [
      /intercept\s+(.+)/i,
      /chase\s+(.+)/i,
      /pursue\s+(.+)/i,
      /attack\s+(.+)/i,
      /engage\s+(.+)/i,
      /(.+)\s*(?:추적|요격|공격)/i, // Korean: X 추적/요격
    ],
    behavior: 'pursuit',
    requiresLocation: false,
    requiresTarget: true,
    description: 'Pursue target',
  },
  {
    patterns: [
      /escort\s+(.+)/i,
      /protect\s+(.+)/i,
      /guard\s+(.+)/i,
      /(.+)\s*(?:호위|보호|엄호)/i, // Korean: X 호위
    ],
    behavior: 'escort',
    requiresLocation: false,
    requiresTarget: true,
    description: 'Escort target',
  },
  {
    patterns: [
      /evade/i,
      /escape/i,
      /run\s+away/i,
      /flee/i,
      /회피/i,
      /도주/i,
    ],
    behavior: 'evasion',
    requiresLocation: false,
    requiresTarget: false,
    description: 'Evade threats',
  },
  {
    patterns: [
      /defend/i,
      /defensive/i,
      /방어/i,
    ],
    behavior: 'defensive',
    requiresLocation: false,
    requiresTarget: false,
    description: 'Defensive mode',
  },
  {
    patterns: [
      /recon(?:naissance)?\s+(?:around\s+|over\s+)?(.+)?/i,
      /scout\s+(?:around\s+|over\s+)?(.+)?/i,
      /observe\s+(.+)?/i,
      /(.+)?\s*정찰/i, // Korean: X 정찰
    ],
    behavior: 'reconnaissance',
    requiresLocation: true,
    requiresTarget: false,
    description: 'Reconnaissance',
  },
  {
    patterns: [
      /hold\s+position/i,
      /stay/i,
      /hover/i,
      /stop/i,
      /idle/i,
      /대기/i,
      /정지/i,
    ],
    behavior: 'idle',
    requiresLocation: false,
    requiresTarget: false,
    description: 'Hold position',
  },
];

// ============================================================================
// Command Parser Result
// ============================================================================

export interface ParsedCommand {
  success: boolean;
  behavior?: AIBehaviorMode;
  waypoints?: Array<{ latitude: number; longitude: number; altitude: number }>;
  targetEntityId?: string;
  customPrompt?: string;
  description: string;
  error?: string;
}

// ============================================================================
// Command Service
// ============================================================================

class CommandService {
  /**
   * Find a location by name (supports English and Korean)
   */
  findLocation(query: string): Location | null {
    const normalizedQuery = query.toLowerCase().trim();

    // Direct match
    const direct = LOCATIONS.find(
      loc => loc.name === normalizedQuery || loc.nameKr === query.trim()
    );
    if (direct) return direct;

    // Partial match
    const partial = LOCATIONS.find(
      loc => loc.name.includes(normalizedQuery) ||
             normalizedQuery.includes(loc.name) ||
             loc.nameKr.includes(query.trim())
    );
    if (partial) return partial;

    return null;
  }

  /**
   * Generate patrol waypoints around a location
   */
  generatePatrolWaypoints(
    center: Location,
    radiusKm: number = 20,
    numPoints: number = 4
  ): Array<{ latitude: number; longitude: number; altitude: number }> {
    const waypoints: Array<{ latitude: number; longitude: number; altitude: number }> = [];

    // Convert km to degrees (approximate)
    const latDelta = radiusKm / 111; // 1 degree ≈ 111 km
    const lonDelta = radiusKm / (111 * Math.cos(center.latitude * Math.PI / 180));

    for (let i = 0; i < numPoints; i++) {
      const angle = (2 * Math.PI * i) / numPoints;
      waypoints.push({
        latitude: center.latitude + latDelta * Math.sin(angle),
        longitude: center.longitude + lonDelta * Math.cos(angle),
        altitude: center.defaultAltitude,
      });
    }

    return waypoints;
  }

  /**
   * Parse a natural language command
   */
  parseCommand(command: string, availableEntityIds?: string[]): ParsedCommand {
    const trimmedCommand = command.trim();

    if (!trimmedCommand) {
      return {
        success: false,
        description: 'Empty command',
        error: 'Please enter a command',
      };
    }

    // Try each command pattern
    for (const pattern of COMMAND_PATTERNS) {
      for (const regex of pattern.patterns) {
        const match = trimmedCommand.match(regex);
        if (match) {
          const captured = match[1]?.trim();

          // Handle location-based commands
          if (pattern.requiresLocation && captured) {
            const location = this.findLocation(captured);
            if (location) {
              // For "fly to" commands, create single waypoint
              if (pattern.patterns[0].toString().includes('fly')) {
                return {
                  success: true,
                  behavior: pattern.behavior,
                  waypoints: [{
                    latitude: location.latitude,
                    longitude: location.longitude,
                    altitude: location.defaultAltitude,
                  }],
                  customPrompt: `Navigate to ${location.name} (${location.nameKr}). Maintain safe altitude and heading.`,
                  description: `Flying to ${location.name} (${location.nameKr})`,
                };
              }

              // For patrol commands, generate patrol pattern
              return {
                success: true,
                behavior: pattern.behavior,
                waypoints: this.generatePatrolWaypoints(location),
                customPrompt: `Patrol around ${location.name} (${location.nameKr}). Maintain awareness and report contacts.`,
                description: `Patrolling ${location.name} (${location.nameKr}) area`,
              };
            } else {
              return {
                success: false,
                description: 'Location not found',
                error: `Unknown location: "${captured}". Try: Seoul, Busan, Incheon, Daegu, etc.`,
              };
            }
          }

          // Handle target-based commands
          if (pattern.requiresTarget && captured) {
            // Try to find entity by name or ID
            const targetId = availableEntityIds?.find(
              id => id.toLowerCase().includes(captured.toLowerCase())
            );

            if (targetId) {
              return {
                success: true,
                behavior: pattern.behavior,
                targetEntityId: targetId,
                customPrompt: `${pattern.description} target: ${targetId}`,
                description: `${pattern.description}: ${targetId}`,
              };
            } else {
              return {
                success: false,
                description: 'Target not found',
                error: `Unknown target: "${captured}". Specify a valid entity ID.`,
              };
            }
          }

          // Handle simple commands (no location/target needed)
          if (!pattern.requiresLocation && !pattern.requiresTarget) {
            return {
              success: true,
              behavior: pattern.behavior,
              customPrompt: pattern.description,
              description: pattern.description,
            };
          }

          // Location/target required but not provided
          if (pattern.requiresLocation) {
            return {
              success: false,
              description: 'Location required',
              error: 'Please specify a location. Example: "fly to Seoul" or "patrol Busan"',
            };
          }
          if (pattern.requiresTarget) {
            return {
              success: false,
              description: 'Target required',
              error: 'Please specify a target entity.',
            };
          }
        }
      }
    }

    // No pattern matched - treat as custom command
    return {
      success: true,
      behavior: 'custom',
      customPrompt: trimmedCommand,
      description: `Custom command: ${trimmedCommand.substring(0, 50)}...`,
    };
  }

  /**
   * Apply parsed command to agent config
   */
  applyCommand(
    currentConfig: Partial<AIAgentConfig>,
    parsed: ParsedCommand
  ): Partial<AIAgentConfig> {
    if (!parsed.success) {
      return currentConfig;
    }

    const updates: Partial<AIAgentConfig> = {
      behaviorMode: parsed.behavior,
    };

    if (parsed.waypoints) {
      updates.waypoints = parsed.waypoints;
    }

    if (parsed.targetEntityId) {
      updates.targetEntityId = parsed.targetEntityId;
    }

    if (parsed.customPrompt) {
      updates.customPrompt = parsed.customPrompt;
    }

    return { ...currentConfig, ...updates };
  }

  /**
   * Get list of available locations for autocomplete
   */
  getAvailableLocations(): string[] {
    return LOCATIONS.map(loc => `${loc.name} (${loc.nameKr})`);
  }

  /**
   * Get example commands
   */
  getExampleCommands(): string[] {
    return [
      'fly to Seoul',
      'fly to Busan',
      'patrol Seoul area',
      'patrol around Incheon',
      'hold position',
      'evade',
      'defensive mode',
      '서울로 비행',
      '부산 지역 순찰',
    ];
  }
}

// ============================================================================
// Singleton Export
// ============================================================================

export const commandService = new CommandService();
