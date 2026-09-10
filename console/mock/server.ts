// Mock Maverick gateway: replays a scripted exploration of the seed-7 apartment.
// Map frame is ENU (x east, y north, z up); manifest coordinates are used as is.
// Usage: tsx mock/server.ts [--port 8765] [--speed 1] [--no-auto]
import { WebSocketServer, type WebSocket } from "ws";
import type {
  AgentStep, ClientMessage, EventMap, Frontier, GatewayEventType, MapObject, Room, RoomType, StepStatus, TranscriptData, Vec3,
} from "../src/gateway/types.ts";
import { CEILING, FLOOR, NONE, RES, WALL, World } from "./world.ts";
import { CAM_Z, HFOV, projectBox, renderView, type Prop } from "./render.ts";

const arg = (name: string, dflt: string) => {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && process.argv[i + 1] ? process.argv[i + 1] : (process.env[`MOCK_${name.toUpperCase()}`] ?? dflt);
};
const PORT = Number(arg("port", "8765"));
const SPEED = Number(arg("speed", "1"));
const AUTO = !process.argv.includes("--no-auto");

const world = new World();
const TICK_MS = 100;
const CRUISE = 1.0; // m/s
const ALT = CAM_Z;
const SPAWN: [number, number] = [world.m.spawn.pose[0], world.m.spawn.pose[1]];

class Aborted extends Error {}

// ------------------------------------------------------------------ state

interface Det { id: string; label: string; room: string; lo: number[]; hi: number[] }
const detectables: Det[] = world.m.furniture.map((f) => ({ id: f.id, label: f.class.replace(/_/g, " "), room: f.room, lo: f.bbox[0], hi: f.bbox[1] }));
const BANANAS: Det[] = [0, 1, 2].map((i) => ({ id: `banana_${i}`, label: "banana", room: "kitchen_0", lo: [6.78, 4.25 + i * 0.17, 0.9], hi: [6.96, 4.4 + i * 0.17, 0.95] }));
const CUP: Det = { id: "cup_0", label: "cup", room: "living_0", lo: [1.7, 4.85, 0.44], hi: [1.8, 4.95, 0.52] };
const BOOK: Det = { id: "book_0", label: "book", room: "bedroom_0", lo: [0.9, 1.0, 0.97], hi: [1.2, 1.25, 1.0] };
const smallItems = (): Det[] => [...(bananasPresent ? BANANAS : []), CUP, BOOK];
let bananasPresent = true;

let epoch = 0;
let simT = 0;
let frozen = 0;
const drone = { x: SPAWN[0], y: SPAWN[1], yaw: world.m.spawn.pose[3] ?? 0, yawGoal: 0, vx: 0, vy: 0 };
let route: { pts: [number, number][]; i: number } | null = null;

let revealed = new Set<number>();
let roomFloor = new Map<string, number>();
let roomState = new Map<string, { n: number; typed: RoomType; conf: number; visited: boolean }>();
let objects = new Map<string, MapObject>();
let objectsDirty = false;
let transcripts = new Map<string, TranscriptData>();
let plan: { plan_id: string; goal: string; steps: AgentStep[]; started: number[] } | null = null;
let skill: { skill: string; msg: () => string; prog: () => number } | null = null;
let battery = 94;
let linkLost = false;
let mode = "idle";
let seq = 0;
let msgN = 0;
let detActive = false;
const latest = new Map<GatewayEventType, string>();
const clients = new Set<WebSocket>();

const totalFloor = (() => { let n = 0; for (let ix = 0; ix < world.nx; ix++) for (let iy = 0; iy < world.ny; iy++) if (world.kind[world.idx(ix, iy, 0)] === FLOOR) n++; return n; })();
let floorSeen = 0;

// ------------------------------------------------------------------ emit

const STATEFUL: GatewayEventType[] = ["pose", "rooms", "objects", "frontiers", "path", "agent_step", "skill_progress", "status"];
function emit<K extends GatewayEventType>(type: K, data: EventMap[K]): void {
  const s = JSON.stringify({ type, t: Date.now() / 1000, data });
  if (STATEFUL.includes(type)) latest.set(type, s);
  if (clients.size === 0) return;
  for (const c of clients) if (c.readyState === 1) c.send(s);
}

function voxelFlat(set: Iterable<number>): number[] {
  const out: number[] = [];
  for (const k of set) {
    const iz = k % world.nz, r = (k - iz) / world.nz, iy = r % world.ny, ix = (r - iy) / world.ny;
    out.push(ix, iy, iz);
  }
  return out;
}
const mapMsg = (add: number[], reset: boolean) => ({ resolution: RES, origin: [0, 0, 0] as Vec3, reset, add, remove: [] as number[] });

function say(role: TranscriptData["role"], text: string, extra: Partial<TranscriptData> = {}): string {
  const id = extra.id ?? `m${++msgN}`;
  const d: TranscriptData = { id, role, text, ...extra };
  transcripts.set(id, d);
  emit("transcript", d);
  return id;
}

// ------------------------------------------------------------------ control primitives

type Waiter = { check: () => boolean; resolve: () => void; reject: (e: Error) => void; epoch: number };
const waiters: Waiter[] = [];
function until(check: () => boolean): Promise<void> {
  const e = epoch;
  return new Promise((resolve, reject) => waiters.push({ check, resolve, reject, epoch: e }));
}
const sleep = (s: number) => { const t = simT + s; return until(() => simT >= t); };
const turnTo = (yaw: number) => { drone.yawGoal = yaw; return until(() => Math.abs(angDiff(yaw, drone.yaw)) < 0.05); };
function flyTo(pts: [number, number][]): Promise<void> {
  route = { pts, i: 0 };
  publishPath("executing");
  return until(() => route === null);
}
async function scan(): Promise<void> {
  for (let k = 1; k <= 4; k++) await turnTo(drone.yaw + Math.PI / 2);
}
function angDiff(a: number, b: number): number { return Math.atan2(Math.sin(a - b), Math.cos(a - b)); }

function publishPath(status: "executing" | "done"): void {
  if (!route || status === "done") {
    emit("path", { path_id: "p", status: "done", waypoints: [] });
    return;
  }
  const wp: Vec3[] = [[drone.x, drone.y, ALT], ...route.pts.slice(route.i).map(([x, y]): Vec3 => [x, y, ALT])];
  emit("path", { path_id: "p" + epoch, status, waypoints: wp, goal: wp[wp.length - 1] });
}

function setPlan(goal: string, labels: string[]): void {
  plan = { plan_id: `plan${++msgN}`, goal, steps: labels.map((label, i) => ({ id: `s${i + 1}`, label, status: "pending" as StepStatus })), started: labels.map(() => 0) };
  publishPlan();
}
function step(i: number, status: StepStatus, detail?: string): void {
  if (!plan) return;
  const s = plan.steps[i];
  if (status === "running") plan.started[i] = simT;
  s.status = status;
  if (detail !== undefined) s.detail = detail;
  if (status === "done" || status === "failed") s.elapsed_s = Math.round(simT - plan.started[i]);
  publishPlan();
}
function publishPlan(): void {
  if (!plan) return emit("agent_step", { plan_id: "none", goal: "", steps: [] });
  emit("agent_step", { plan_id: plan.plan_id, goal: plan.goal, steps: plan.steps.map((s, i) => ({ ...s, elapsed_s: s.status === "running" ? Math.round(simT - plan!.started[i]) : s.elapsed_s })) });
}
function setMode(m: string): void { mode = m; }

// ------------------------------------------------------------------ perception

function castReveal(): void {
  const ox = drone.x, oy = drone.y, oz = ALT;
  const added: number[] = [];
  const maxD = 4.6, stepD = 0.04;
  for (let az = 0; az < 360; az += 1.5) {
    const ca = Math.cos((az * Math.PI) / 180), sa = Math.sin((az * Math.PI) / 180);
    for (let pitch = -62; pitch <= 16; pitch += 6) {
      const cp = Math.cos((pitch * Math.PI) / 180), sp = Math.sin((pitch * Math.PI) / 180);
      for (let d = 0.15; d < maxD; d += stepD) {
        const x = ox + ca * cp * d, y = oy + sa * cp * d, z = oz + sp * d;
        if (z < 0 || z >= CEILING) break;
        const k = world.kindAt(x, y, z);
        if (k === -1) break;
        if (k === NONE) continue;
        const key = world.idx(Math.floor(x / RES), Math.floor(y / RES), Math.floor(z / RES));
        if (!revealed.has(key)) {
          revealed.add(key);
          added.push(key);
          if (k === FLOOR) {
            floorSeen++;
            const r = world.roomAt(x, y);
            if (r) roomFloor.set(r, (roomFloor.get(r) ?? 0) + 1);
          }
        }
        break;
      }
    }
  }
  if (added.length) emit("map_update", mapMsg(voxelFlat(added), false));
}

function hash01(s: string): number { let h = 7; for (const c of s) h = (h * 33 + c.charCodeAt(0)) >>> 0; return (h % 1000) / 1000; }

function visible(det: Det): { box: [number, number, number, number]; conf: number } | null {
  const pr = projectBox(drone.x, drone.y, drone.yaw, det.lo, det.hi);
  if (!pr || pr.fwd > 5.5) return null;
  const cx = (det.lo[0] + det.hi[0]) / 2, cy = (det.lo[1] + det.hi[1]) / 2, cz = (det.lo[2] + det.hi[2]) / 2;
  const n = Math.ceil(Math.hypot(cx - drone.x, cy - drone.y, cz - ALT) / 0.05);
  for (let i = 1; i < n - 3; i++) {
    const f = i / n;
    if (world.kindAt(drone.x + (cx - drone.x) * f, drone.y + (cy - drone.y) * f, ALT + (cz - ALT) * f) === WALL) return null;
  }
  const size = Math.max(det.hi[0] - det.lo[0], det.hi[1] - det.lo[1]);
  if (size < 0.3 && pr.fwd > 3.2) return null; // small things only up close
  const conf = Math.min(0.98, Math.max(0.5, 0.97 - 0.05 * pr.fwd + 0.06 * hash01(det.id) + 0.02 * Math.sin(simT * 3 + hash01(det.id) * 6)));
  return { box: pr.box, conf };
}

function currentProps(): Prop[] {
  const p: Prop[] = smallItems().map((d) => ({ x: (d.lo[0] + d.hi[0]) / 2, y: (d.lo[1] + d.hi[1]) / 2, z: (d.lo[2] + d.hi[2]) / 2, w: Math.max(d.hi[1] - d.lo[1], 0.08) * 1.3, h: Math.max(d.hi[2] - d.lo[2], 0.05) * 1.4 + 0.03, color: d.label === "banana" ? "#f6d330" : d.label === "cup" ? "#e5e7eb" : "#c2410c" }));
  return p;
}

function perceive(): void {
  const boxes: { label: string; confidence: number; box: [number, number, number, number]; object_id?: string }[] = [];
  const all = [...detectables, ...smallItems()];
  for (const d of all) {
    const v = visible(d);
    if (!v) continue;
    boxes.push({ label: d.label, confidence: v.conf, box: v.box, object_id: d.id });
    const prev = objects.get(d.id);
    if (!prev || prev.status !== "present" || simT - prev.last_seen > 3) {
      objects.set(d.id, { id: d.id, label: d.label, position: [(d.lo[0] + d.hi[0]) / 2, (d.lo[1] + d.hi[1]) / 2, (d.lo[2] + d.hi[2]) / 2], size: [d.hi[0] - d.lo[0], d.hi[1] - d.lo[1], d.hi[2] - d.lo[2]], room_id: d.room, confidence: Math.max(prev?.confidence ?? 0, v.conf), last_seen: Date.now() / 1000, status: "present" });
      objectsDirty = true;
    }
  }
  seq++;
  const svg = renderView(world, drone.x, drone.y, drone.yaw, currentProps());
  emit("camera_frame", { seq, width: 640, height: 480, mime: "image/svg+xml", data: Buffer.from(svg).toString("base64"), hfov_deg: (HFOV * 180) / Math.PI });
  emit("detections", { frame_seq: seq, boxes });
  detActive = boxes.length > 0;
}

function typeOf(t: string): RoomType { return (["living", "kitchen", "bedroom", "bathroom", "hallway"].includes(t) ? t : "other") as RoomType; }

function updateRooms(finalize = false): void {
  let changed = false;
  for (const m of world.m.rooms) {
    const seen = (roomFloor.get(m.id) ?? 0) >= 20;
    if (!seen) continue;
    let st = roomState.get(m.id);
    if (!st) { st = { n: roomState.size + 1, typed: "unknown", conf: 0, visited: false }; roomState.set(m.id, st); changed = true; }
    const rc = world.rects.get(m.id)!;
    const inside = drone.x >= rc.x0 && drone.x < rc.x1 && drone.y >= rc.y0 && drone.y < rc.y1;
    if (inside && !st.visited) { st.visited = true; changed = true; }
    const nObj = [...objects.values()].filter((o) => o.room_id === m.id && !["banana", "cup", "book"].includes(o.label)).length;
    if (nObj > 0 || finalize) {
      const conf = Math.min(0.97, 0.55 + 0.14 * nObj + (finalize ? 0.1 : 0));
      if (st.typed === "unknown" || Math.abs(conf - st.conf) > 0.01) { st.typed = typeOf(m.type); st.conf = conf; changed = true; }
    }
  }
  if (changed) publishRooms();
}
function publishRooms(): void {
  const rooms: Room[] = [];
  for (const m of world.m.rooms) {
    const st = roomState.get(m.id);
    if (!st) continue;
    rooms.push({ id: m.id, type: st.typed, label: st.typed === "unknown" ? `Room ${st.n}` : st.typed[0].toUpperCase() + st.typed.slice(1), polygon: m.polygon, confidence: st.conf, visited: st.visited });
  }
  emit("rooms", { rooms });
}
function publishObjects(): void { emit("objects", { objects: [...objects.values()] }); objectsDirty = false; }

// coarse frontier detection on the floor grid
const CELL = 0.5;
function computeFrontiers(): void {
  const gx = Math.ceil((world.nx * RES) / CELL), gy = Math.ceil((world.ny * RES) / CELL);
  const tot = new Int32Array(gx * gy), seen = new Int32Array(gx * gy);
  for (let ix = 0; ix < world.nx; ix++) for (let iy = 0; iy < world.ny; iy++) {
    const key = world.idx(ix, iy, 0);
    if (world.kind[key] !== FLOOR) continue;
    const c = Math.floor((ix * RES) / CELL) * gy + Math.floor((iy * RES) / CELL);
    tot[c]++;
    if (revealed.has(key)) seen[c]++;
  }
  const open = (c: number) => tot[c] >= 12;
  const known = (c: number) => open(c) && seen[c] / tot[c] >= 0.45;
  const out: Frontier[] = [];
  for (let a = 0; a < gx; a++) for (let b = 0; b < gy; b++) {
    const c = a * gy + b;
    if (!known(c)) continue;
    for (const [da, db] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
      const a2 = a + da, b2 = b + db;
      if (a2 < 0 || b2 < 0 || a2 >= gx || b2 >= gy) continue;
      const c2 = a2 * gy + b2;
      if (!open(c2) || known(c2)) continue;
      const x1 = (a + 0.5) * CELL, y1 = (b + 0.5) * CELL, x2 = (a2 + 0.5) * CELL, y2 = (b2 + 0.5) * CELL;
      let blocked = false;
      for (let f = 0; f <= 1.001; f += 0.05) if (world.kindAt(x1 + (x2 - x1) * f, y1 + (y2 - y1) * f, 0.15) === WALL) { blocked = true; break; }
      if (blocked) continue;
      const p: Vec3 = [(x1 + x2) / 2, (y1 + y2) / 2, 0.5];
      if (out.some((o) => Math.hypot(o.position[0] - p[0], o.position[1] - p[1]) < 0.9)) continue;
      out.push({ id: `f${a}_${b}_${a2}_${b2}`, position: p, size: tot[c2], gain: Math.round((tot[c2] - seen[c2]) / 2.5) / 10 });
    }
  }
  out.sort((p, q) => Math.hypot(p.position[0] - drone.x, p.position[1] - drone.y) - Math.hypot(q.position[0] - drone.x, q.position[1] - drone.y));
  emit("frontiers", { frontiers: out.slice(0, 14) });
}

// ------------------------------------------------------------------ simulation tick

let lastReal = Date.now(), tickN = 0, frozenUntilNote = false;
function tick(): void {
  const now = Date.now();
  const dt = Math.min(0.25, (now - lastReal) / 1000) * SPEED;
  lastReal = now;
  tickN++;
  simT += dt;

  if (frozen > 0) {
    frozen -= dt;
    if (frozen <= 0) { linkLost = false; frozenUntilNote = false; say("system", "Link restored. Resuming."); }
  }
  drone.vx = drone.vy = 0;
  const hold = frozen > 0;
  // yaw towards goal
  if (route && !hold) {
    const [tx, ty] = route.pts[route.i];
    drone.yawGoal = Math.atan2(ty - drone.y, tx - drone.x);
  }
  const maxTurn = 1.6 * dt;
  const err = angDiff(drone.yawGoal, drone.yaw);
  if (!hold || Math.abs(err) > 0) drone.yaw += Math.max(-maxTurn, Math.min(maxTurn, err));
  if (route && !hold && Math.abs(angDiff(drone.yawGoal, drone.yaw)) < 0.5) {
    let rem = CRUISE * dt;
    while (rem > 0 && route && route.i < route.pts.length) {
      const [tx, ty] = route.pts[route.i];
      const d = Math.hypot(tx - drone.x, ty - drone.y);
      if (d <= rem) { drone.x = tx; drone.y = ty; rem -= d; route.i++; }
      else { drone.vx = ((tx - drone.x) / d) * CRUISE; drone.vy = ((ty - drone.y) / d) * CRUISE; drone.x += (drone.vx * rem) / CRUISE; drone.y += (drone.vy * rem) / CRUISE; rem = 0; }
    }
    if (route && route.i >= route.pts.length) { route = null; publishPath("done"); }
  }
  const bob = Math.sin(simT * 1.7) * 0.015;
  emit("pose", { position: [drone.x, drone.y, ALT + bob], yaw: drone.yaw, pitch: 0.02 * Math.sin(simT), roll: -0.15 * Math.max(-1, Math.min(1, err)) * Math.min(1, Math.hypot(drone.vx, drone.vy)), velocity: [drone.vx, drone.vy, 0] });

  const every = (n: number) => tickN % Math.max(1, Math.round(n / SPEED)) === 0;
  if (tickN % 2 === 0) castReveal();
  if (tickN % 2 === 1) perceive();
  if (every(15)) updateRooms();
  if (objectsDirty || every(60)) publishObjects();
  if (every(15)) computeFrontiers();
  if (route && every(10)) publishPath("executing");
  if (every(5)) {
    battery = Math.max(3, battery - 0.02 * SPEED * (hold ? 0.5 : 1));
    emit("status", {
      battery_pct: battery, battery_v: 13.2 + battery * 0.034,
      link: linkLost ? { quality: "lost" } : { quality: "good", rtt_ms: 11 + Math.random() * 6, loss_pct: Math.random() < 0.1 ? 1 : 0 },
      mode: hold ? "hover (link lost)" : mode, armed: true,
      vram_used_gb: 8.4 + 0.25 * Math.sin(simT / 9) + (detActive ? 0.5 : 0) + (mode === "search" ? 0.4 : 0), vram_total_gb: 16,
    });
    if (skill) {
      const p = skill.prog();
      emit("skill_progress", { skill: skill.skill, state: "running", progress: p, message: skill.msg() });
    }
    if (plan?.steps.some((s) => s.status === "running")) publishPlan();
  }
  // resolve waiters
  for (let i = waiters.length - 1; i >= 0; i--) {
    const w = waiters[i];
    if (w.epoch !== epoch) { waiters.splice(i, 1); w.reject(new Aborted()); }
    else if (w.check()) { waiters.splice(i, 1); w.resolve(); }
  }
  void frozenUntilNote;
}

// ------------------------------------------------------------------ missions

const KITCHEN_VIEW: [number, number] = [5.8, 4.55];
const R = (pts: [number, number][]) => pts;
const EXPLORE_ROUTE: [number, number][] = R([
  [2.4, 4.6], [1.5, 3.6], [3.1, 3.4], [3.1, 2.7], [3.1, 2.1], [2.5, 1.4], [2.5, 0.7], [2.6, 2.3], [3.1, 3.3],
  [2.7, 4.3], [2.6, 5.8], [3.6, 5.5], [4.1, 5.0], [4.6, 5.0], [5.4, 4.8], [6.1, 4.4], [6.1, 5.8], [5.4, 5.9], [5.3, 3.5],
  [6.65, 3.3], [6.65, 2.4], [6.65, 1.1], [6.4, 2.1], [5.9, 2.1], [5.2, 1.6], [4.6, 1.2], [4.9, 2.4], [5.9, 2.1], [6.5, 2.1],
  [6.65, 2.6], [6.65, 3.3], [5.6, 4.6], [4.6, 5.0], [3.6, 4.7], [SPAWN[0], SPAWN[1]],
]);

async function exploreMission(): Promise<void> {
  say("maverick", "On it. I'll explore the flat and label the rooms.");
  setPlan("Explore the flat", ["Scan surroundings", "Explore frontiers", "Label rooms from objects", "Return to start"]);
  setMode("explore");
  step(0, "running");
  skill = { skill: "explore", msg: () => `${floorSeen} of ~${totalFloor} floor cells mapped`, prog: () => Math.min(0.99, floorSeen / (totalFloor * 0.8)) };
  await scan();
  step(0, "done", "in the living room");
  step(1, "running", "choosing frontiers by gain / path cost");
  const legs = EXPLORE_ROUTE.slice(0, -1);
  await flyTo(legs);
  step(1, "done", `${roomState.size} rooms, ${[...objects.values()].length} objects`);
  step(2, "running");
  skill = { skill: "inspect", msg: () => "scoring furniture per room", prog: () => -1 };
  await sleep(2.5);
  updateRooms(true);
  step(2, "done", [...roomState.values()].map((s) => s.typed).join(", "));
  step(3, "running");
  skill = { skill: "return", msg: () => "flying back to the start point", prog: () => -1 };
  setMode("return");
  await flyTo([EXPLORE_ROUTE[EXPLORE_ROUTE.length - 1]]);
  await turnTo(0);
  step(3, "done");
  skill = null;
  emit("skill_progress", { skill: "return", state: "succeeded", progress: 1, message: "done" });
  setMode("hover");
  say("maverick", `Done. I mapped ${roomState.size} rooms (${[...roomState.values()].map((s) => s.typed).join(", ")}) and found ${[...objects.values()].filter((o) => o.status === "present").length} objects.`);
}

async function bananaMission(): Promise<void> {
  const kitchen = [...roomState.entries()].find(([id]) => id === "kitchen_0");
  say("maverick", "On it.");
  setPlan("Check the kitchen for bananas", ["Memory: where is the kitchen", "Go to the kitchen", "Search for bananas", "Take a photo", "Come back to you", "Report"]);
  step(0, "running");
  await sleep(1.2);
  step(0, "done", kitchen ? `kitchen = ${kitchen[0]}, known since exploring` : "not mapped yet, will look");
  setMode("goto");
  step(1, "running");
  skill = { skill: "goto", msg: () => "to the kitchen, via the open doorway", prog: () => -1 };
  const here: [number, number] = [drone.x, drone.y];
  await flyTo(route2(here, KITCHEN_VIEW));
  step(1, "done");
  setMode("search");
  step(2, "running", "detector: banana");
  skill = { skill: "search", msg: () => "looking at the counters", prog: () => -1 };
  await turnTo(0);
  await sleep(3);
  const found = bananasPresent;
  // memory update: look at the spot, confirm or mark gone
  for (const b of BANANAS) {
    const o = objects.get(b.id);
    if (found) { if (!o) { objects.set(b.id, { id: b.id, label: "banana", position: [(b.lo[0] + b.hi[0]) / 2, (b.lo[1] + b.hi[1]) / 2, (b.lo[2] + b.hi[2]) / 2], size: [b.hi[0] - b.lo[0], b.hi[1] - b.lo[1], b.hi[2] - b.lo[2]], room_id: "kitchen_0", confidence: 0.9, last_seen: Date.now() / 1000, status: "present" }); } }
    else if (o) { o.status = "gone"; o.last_seen = Date.now() / 1000; }
  }
  publishObjects();
  step(2, "done", found ? "3 bananas on the counter" : "none on the counter");
  step(3, "running");
  const boxes = BANANAS.map((b) => { const v = found ? visible(b) : null; return v ? { label: "banana", confidence: v.conf, box: v.box } : null; }).filter((x): x is NonNullable<typeof x> => x !== null);
  const photo = Buffer.from(renderView(world, drone.x, drone.y, drone.yaw, currentProps(), boxes)).toString("base64");
  await sleep(1);
  step(3, "done");
  setMode("return");
  step(4, "running");
  skill = { skill: "return", msg: () => "back to you", prog: () => -1 };
  await flyTo(route2([drone.x, drone.y], SPAWN));
  await turnTo(0);
  step(4, "done");
  step(5, "running");
  skill = null;
  emit("skill_progress", { skill: "return", state: "succeeded", progress: 1, message: "arrived" });
  await sleep(0.8);
  step(5, "done");
  setMode("hover");
  if (found) say("maverick", "Three bananas on the kitchen counter.", { attachments: [{ kind: "photo", mime: "image/svg+xml", data: photo, caption: "Kitchen counter, a minute ago", object_id: "banana_0" }] });
  else say("maverick", "The bananas are gone. The counter is empty now.", { attachments: [{ kind: "photo", mime: "image/svg+xml", data: photo, caption: "Kitchen counter, just now" }] });
}

/** Route between two points through the known doorways (living <-> kitchen only; enough for the script). */
function route2(a: [number, number], b: [number, number]): [number, number][] {
  const door: [number, number] = [4.6, 5.0];
  const side = (p: [number, number]) => (p[0] < 4.6 ? 0 : p[0] >= 4.6 && p[1] > 2.9 ? 1 : 2);
  const sa = side(a), sb = side(b);
  if (sa === sb) return [b];
  if (sa === 0 && sb === 1) return [[4.0, 4.95], door, [5.3, 4.9], b];
  if (sa === 1 && sb === 0) return [[5.3, 4.9], door, [4.0, 4.95], b];
  return [door, b];
}

// ------------------------------------------------------------------ command handling

let busy = false;
const queue: (() => Promise<void>)[] = [];
async function runQueue(): Promise<void> {
  if (busy) return;
  busy = true;
  const e = epoch;
  try {
    while (queue.length && e === epoch) await queue.shift()!();
  } catch (err) {
    if (!(err instanceof Aborted)) console.error(err);
  }
  if (e === epoch) busy = false;
}
function enqueue(job: () => Promise<void>): void { queue.push(job); void runQueue(); }

function handleCommand(text: string): void {
  const low = text.toLowerCase();
  if (busy) say("maverick", "One moment, I'll do that next.");
  if (/banana|kitchen/.test(low)) enqueue(bananaMission);
  else if (/explore|map|flat/.test(low)) enqueue(exploreMission);
  else enqueue(async () => { say("maverick", "The mock gateway only knows two tricks: 'explore the flat' and 'check the kitchen for bananas'."); });
}

function resetDemo(): void {
  epoch++;
  queue.length = 0;
  busy = false;
  route = null; skill = null; plan = null; frozen = 0; linkLost = false;
  Object.assign(drone, { x: SPAWN[0], y: SPAWN[1], yaw: world.m.spawn.pose[3] ?? 0, yawGoal: 0 });
  revealed = new Set(); roomFloor = new Map(); roomState = new Map(); objects = new Map(); transcripts = new Map();
  floorSeen = 0; bananasPresent = true; battery = 94; mode = "idle"; simT = 0;
  emit("map_update", mapMsg([], true));
  emit("rooms", { rooms: [] }); emit("objects", { objects: [] }); emit("frontiers", { frontiers: [] });
  emit("path", { path_id: "p", status: "done", waypoints: [] });
  publishPlan();
  startScript();
}

function startScript(): void {
  const e = epoch;
  setTimeout(() => {
    if (e !== epoch) return;
    say("user", "Maverick, explore the flat.", { source: "voice" });
    enqueue(exploreMission);
    if (AUTO) enqueue(async () => {
      await sleep(5);
      say("user", "We're sitting down, check the kitchen for bananas.", { source: "voice" });
      await bananaMission();
    });
  }, 1500 / SPEED);
}

// ------------------------------------------------------------------ server

const wss = new WebSocketServer({ port: PORT });
wss.on("connection", (ws) => {
  clients.add(ws);
  const send = (type: GatewayEventType, data: unknown) => ws.send(JSON.stringify({ type, t: Date.now() / 1000, data }));
  send("map_update", mapMsg(voxelFlat(revealed), true));
  for (const t of ["rooms", "objects", "frontiers", "path", "agent_step", "skill_progress", "status", "pose"] as GatewayEventType[]) {
    const s = latest.get(t);
    if (s) ws.send(s);
  }
  for (const d of transcripts.values()) send("transcript", d);
  ws.on("message", (raw) => {
    let msg: ClientMessage;
    try { msg = JSON.parse(String(raw)) as ClientMessage; } catch { return; }
    onClient(msg);
  });
  ws.on("close", () => clients.delete(ws));
});

let pttId = "";
function onClient(msg: ClientMessage): void {
  switch (msg.type) {
    case "text_command": {
      const text = String(msg.data.text ?? "").trim();
      if (!text) return;
      say("user", text, { source: "text" });
      handleCommand(text);
      break;
    }
    case "ptt_start":
      pttId = `ptt${++msgN}`;
      say("user", "...", { id: pttId, partial: true, source: "voice" });
      break;
    case "ptt_stop": {
      const id = pttId;
      if (!id) break;
      pttId = "";
      setTimeout(() => {
        const text = "Check the kitchen for bananas.";
        say("user", text, { id, source: "voice" });
        handleCommand(text);
      }, 500);
      break;
    }
    case "director": {
      const { action, params } = msg.data;
      if (action === "remove_object") { bananasPresent = false; say("system", "Director: the bananas were taken away."); }
      else if (action === "cut_wifi") { frozen = Number(params?.seconds ?? 5); linkLost = true; say("system", "Director: Wi-Fi cut. Maverick holds position."); }
      else if (action === "drain_battery") { battery = Math.max(8, battery - Number(params?.pct ?? 40)); say("system", `Director: battery drained to ${Math.round(battery)}%.`); }
      else if (action === "add_person") say("system", `Director: ${String(params?.name ?? "someone")} joins (people are not simulated by the mock gateway).`);
      else if (action === "reset_demo") resetDemo();
      else say("system", `Director action '${action}' is not implemented in the mock.`);
      break;
    }
    default: break;
  }
}

setInterval(tick, TICK_MS);
startScript();
console.log(`mock gateway on ws://0.0.0.0:${PORT}  speed x${SPEED}  auto-script ${AUTO ? "on" : "off"}  (horizontal fov ${(HFOV * 180) / Math.PI} deg)`);
