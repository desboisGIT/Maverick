import type {
  ClientEventMap,
  ClientEventType,
  EventMap,
  GatewayEventType,
} from "./types";
import { isGatewayEvent } from "./types";

export type ConnState = "connecting" | "open" | "closed";
type Handler<K extends GatewayEventType> = (data: EventMap[K], t: number) => void;

/** Websocket client with auto-reconnect and typed subscriptions. */
export class GatewayClient {
  readonly url: string;
  state: ConnState = "connecting";
  private ws: WebSocket | null = null;
  private handlers = new Map<string, Set<(d: never, t: number) => void>>();
  private stateHandlers = new Set<(s: ConnState) => void>();
  private retry = 500;
  private closedByUser = false;

  constructor(url: string) {
    this.url = url;
  }

  connect(): void {
    this.setState("connecting");
    let ws: WebSocket;
    try {
      ws = new WebSocket(this.url);
    } catch {
      this.scheduleRetry();
      return;
    }
    this.ws = ws;
    ws.onopen = () => {
      this.retry = 500;
      this.setState("open");
    };
    ws.onmessage = (ev) => {
      if (typeof ev.data !== "string") return;
      let msg: unknown;
      try {
        msg = JSON.parse(ev.data);
      } catch {
        return;
      }
      if (!isGatewayEvent(msg)) return;
      const set = this.handlers.get(msg.type);
      if (set) for (const h of set) (h as (d: unknown, t: number) => void)(msg.data, msg.t);
    };
    ws.onclose = () => {
      this.ws = null;
      this.setState("closed");
      if (!this.closedByUser) this.scheduleRetry();
    };
    ws.onerror = () => ws.close();
  }

  private scheduleRetry(): void {
    setTimeout(() => this.connect(), this.retry);
    this.retry = Math.min(this.retry * 1.6, 5000);
  }

  private setState(s: ConnState): void {
    this.state = s;
    for (const h of this.stateHandlers) h(s);
  }

  on<K extends GatewayEventType>(type: K, cb: Handler<K>): () => void {
    let set = this.handlers.get(type);
    if (!set) this.handlers.set(type, (set = new Set()));
    set.add(cb as (d: never, t: number) => void);
    return () => set.delete(cb as (d: never, t: number) => void);
  }

  onState(cb: (s: ConnState) => void): void {
    this.stateHandlers.add(cb);
    cb(this.state);
  }

  send<K extends ClientEventType>(type: K, data: ClientEventMap[K]): boolean {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) return false;
    this.ws.send(JSON.stringify({ type, t: Date.now() / 1000, data }));
    return true;
  }

  close(): void {
    this.closedByUser = true;
    this.ws?.close();
  }
}
