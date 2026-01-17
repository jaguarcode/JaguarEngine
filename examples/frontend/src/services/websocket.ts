import type {
  WebSocketMessage,
  CommandRequest,
  SpawnEntityRequest,
} from '@/types';

type MessageHandler = (message: WebSocketMessage) => void;
type ConnectionHandler = (connected: boolean, error?: string) => void;

interface PendingRequest {
  resolve: (data: unknown) => void;
  reject: (error: Error) => void;
  timeout: number;
}

class WebSocketService {
  private ws: WebSocket | null = null;
  private url: string;
  private reconnectAttempts = 0;
  private maxReconnectAttempts = 10;
  private reconnectDelay = 1000;
  private messageHandlers: Map<string, MessageHandler[]> = new Map();
  private connectionHandlers: ConnectionHandler[] = [];
  private pendingRequests: Map<string, PendingRequest> = new Map();
  private requestIdCounter = 0;

  constructor(url = 'ws://localhost:8081') {
    this.url = url;
  }

  connect(): void {
    if (this.ws?.readyState === WebSocket.OPEN) {
      return;
    }

    try {
      this.ws = new WebSocket(this.url);

      this.ws.onopen = () => {
        console.log('[WebSocket] Connected to', this.url);
        this.reconnectAttempts = 0;
        this.notifyConnectionHandlers(true);
      };

      this.ws.onclose = (event) => {
        console.log('[WebSocket] Disconnected:', event.code, event.reason);
        this.notifyConnectionHandlers(false, 'Connection closed');
        this.scheduleReconnect();
      };

      this.ws.onerror = (error) => {
        console.error('[WebSocket] Error:', error);
        this.notifyConnectionHandlers(false, 'Connection error');
      };

      this.ws.onmessage = (event) => {
        this.handleMessage(event.data);
      };
    } catch (error) {
      console.error('[WebSocket] Failed to connect:', error);
      this.notifyConnectionHandlers(false, 'Failed to connect');
      this.scheduleReconnect();
    }
  }

  disconnect(): void {
    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }
    this.reconnectAttempts = this.maxReconnectAttempts; // Prevent auto-reconnect
  }

  private scheduleReconnect(): void {
    if (this.reconnectAttempts >= this.maxReconnectAttempts) {
      console.log('[WebSocket] Max reconnect attempts reached');
      return;
    }

    const delay = this.reconnectDelay * Math.pow(2, this.reconnectAttempts);
    this.reconnectAttempts++;

    console.log(`[WebSocket] Reconnecting in ${delay}ms (attempt ${this.reconnectAttempts})`);

    setTimeout(() => {
      this.connect();
    }, delay);
  }

  private handleMessage(data: string | ArrayBuffer): void {
    try {
      const message: WebSocketMessage =
        typeof data === 'string' ? JSON.parse(data) : this.parseBinaryMessage(data);

      // Handle pending requests
      if (message.requestId && this.pendingRequests.has(message.requestId)) {
        const pending = this.pendingRequests.get(message.requestId)!;
        clearTimeout(pending.timeout);
        this.pendingRequests.delete(message.requestId);

        if (message.type === 'error') {
          pending.reject(new Error(String(message.data)));
        } else {
          pending.resolve(message.data);
        }
        return;
      }

      // Notify type-specific handlers
      const handlers = this.messageHandlers.get(message.type) || [];
      handlers.forEach((handler) => handler(message));

      // Notify wildcard handlers
      const wildcardHandlers = this.messageHandlers.get('*') || [];
      wildcardHandlers.forEach((handler) => handler(message));
    } catch (error) {
      console.error('[WebSocket] Failed to parse message:', error);
    }
  }

  private parseBinaryMessage(data: ArrayBuffer): WebSocketMessage {
    // TODO: Implement protobuf parsing if needed
    const decoder = new TextDecoder();
    return JSON.parse(decoder.decode(data));
  }

  // Message handlers
  onMessage(type: string, handler: MessageHandler): () => void {
    const handlers = this.messageHandlers.get(type) || [];
    handlers.push(handler);
    this.messageHandlers.set(type, handlers);

    // Return unsubscribe function
    return () => {
      const handlers = this.messageHandlers.get(type) || [];
      const index = handlers.indexOf(handler);
      if (index > -1) {
        handlers.splice(index, 1);
      }
    };
  }

  onConnection(handler: ConnectionHandler): () => void {
    this.connectionHandlers.push(handler);

    return () => {
      const index = this.connectionHandlers.indexOf(handler);
      if (index > -1) {
        this.connectionHandlers.splice(index, 1);
      }
    };
  }

  private notifyConnectionHandlers(connected: boolean, error?: string): void {
    this.connectionHandlers.forEach((handler) => handler(connected, error));
  }

  // Send methods
  send(message: WebSocketMessage): void {
    if (this.ws?.readyState !== WebSocket.OPEN) {
      console.warn('[WebSocket] Cannot send - not connected');
      return;
    }

    this.ws.send(JSON.stringify(message));
  }

  async sendAsync<T>(message: Omit<WebSocketMessage, 'requestId'>, timeout = 15000): Promise<T> {
    return new Promise((resolve, reject) => {
      const requestId = `req_${++this.requestIdCounter}_${Date.now()}`;

      const timeoutHandle = window.setTimeout(() => {
        this.pendingRequests.delete(requestId);
        reject(new Error('Request timeout'));
      }, timeout);

      this.pendingRequests.set(requestId, {
        resolve: resolve as (data: unknown) => void,
        reject,
        timeout: timeoutHandle,
      });

      this.send({ ...message, requestId } as WebSocketMessage);
    });
  }

  // Convenience methods for JaguarEngine
  startSimulation(): void {
    this.send({
      type: 'command',
      timestamp: Date.now(),
      data: { command: 'start' },
    });
  }

  pauseSimulation(): void {
    this.send({
      type: 'command',
      timestamp: Date.now(),
      data: { command: 'pause' },
    });
  }

  stopSimulation(): void {
    this.send({
      type: 'command',
      timestamp: Date.now(),
      data: { command: 'stop' },
    });
  }

  resetSimulation(): void {
    this.send({
      type: 'command',
      timestamp: Date.now(),
      data: { command: 'reset' },
    });
  }

  setTimeScale(scale: number): void {
    this.send({
      type: 'command',
      timestamp: Date.now(),
      data: { command: 'set_time_scale', params: { scale } },
    });
  }

  spawnEntity(request: SpawnEntityRequest): void {
    this.send({
      type: 'entity_spawn',
      timestamp: Date.now(),
      data: request,
    });
  }

  destroyEntity(entityId: string): void {
    this.send({
      type: 'entity_destroy',
      timestamp: Date.now(),
      data: { entityId },
    });
  }

  sendCommand(request: CommandRequest): void {
    this.send({
      type: 'command',
      timestamp: Date.now(),
      data: request,
    });
  }

  // Entity-specific commands
  setEntityPosition(entityId: string, latitude: number, longitude: number, altitude: number): void {
    this.sendCommand({
      command: 'set_position',
      entityId,
      params: { latitude, longitude, altitude },
    });
  }

  setEntityVelocity(entityId: string, north: number, east: number, down: number): void {
    this.sendCommand({
      command: 'set_velocity',
      entityId,
      params: { north, east, down },
    });
  }

  // Aircraft-specific commands
  setFlightControls(
    entityId: string,
    controls: { elevator?: number; aileron?: number; rudder?: number; throttle?: number }
  ): void {
    this.sendCommand({
      command: 'set_flight_controls',
      entityId,
      params: controls,
    });
  }

  setAutopilot(entityId: string, mode: string, params?: Record<string, unknown>): void {
    this.sendCommand({
      command: 'set_autopilot',
      entityId,
      params: { mode, ...params },
    });
  }

  // Ground vehicle commands
  setVehicleControls(
    entityId: string,
    controls: { throttle?: number; steering?: number; brake?: number }
  ): void {
    this.sendCommand({
      command: 'set_vehicle_controls',
      entityId,
      params: controls,
    });
  }

  // Ship commands
  setShipControls(entityId: string, controls: { rudder?: number; throttle?: number }): void {
    this.sendCommand({
      command: 'set_ship_controls',
      entityId,
      params: controls,
    });
  }

  // Connection state
  get isConnected(): boolean {
    return this.ws?.readyState === WebSocket.OPEN;
  }

  get readyState(): number {
    return this.ws?.readyState ?? WebSocket.CLOSED;
  }
}

// Singleton instance
export const websocketService = new WebSocketService(
  import.meta.env.VITE_WS_URL || 'ws://localhost:8081'
);

export default websocketService;
