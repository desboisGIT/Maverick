// Headless smoke test: connects to the mock gateway, checks every event shape against the contract.
// Usage: MOCK_URL=ws://localhost:8765 tsx mock/smoke.ts   (run the mock with MOCK_SPEED=8 to finish quickly)
import WebSocket from "ws";
import { GATEWAY_EVENT_TYPES, type GatewayEventType } from "../src/gateway/types.ts";

const url = process.env.MOCK_URL ?? "ws://localhost:8765";
const num = (v: unknown) => typeof v === "number" && Number.isFinite(v);
const vec3 = (v: unknown) => Array.isArray(v) && v.length === 3 && v.every(num);
const str = (v: unknown) => typeof v === "string";
const arr = (v: unknown) => Array.isArray(v);
type D = Record<string, any>; // eslint-disable-line @typescript-eslint/no-explicit-any

const checks: Record<GatewayEventType, (d: D) => boolean> = {
  pose: (d) => vec3(d.position) && num(d.yaw) && num(d.pitch) && num(d.roll) && vec3(d.velocity),
  camera_frame: (d) => num(d.seq) && num(d.width) && num(d.height) && str(d.mime) && str(d.data) && d.data.length > 100 && num(d.hfov_deg),
  detections: (d) => num(d.frame_seq) && arr(d.boxes) && d.boxes.every((b: D) => str(b.label) && num(b.confidence) && b.box.length === 4 && b.box.every(num)),
  map_update: (d) => num(d.resolution) && vec3(d.origin) && typeof d.reset === "boolean" && arr(d.add) && d.add.length % 3 === 0 && arr(d.remove) && d.remove.length % 3 === 0,
  frontiers: (d) => arr(d.frontiers) && d.frontiers.every((f: D) => str(f.id) && vec3(f.position) && num(f.size)),
  path: (d) => str(d.path_id) && str(d.status) && arr(d.waypoints) && d.waypoints.every(vec3),
  rooms: (d) => arr(d.rooms) && d.rooms.every((r: D) => str(r.id) && str(r.type) && str(r.label) && arr(r.polygon) && r.polygon.every((p: unknown) => arr(p) && p.length === 2) && num(r.confidence) && typeof r.visited === "boolean"),
  objects: (d) => arr(d.objects) && d.objects.every((o: D) => str(o.id) && str(o.label) && vec3(o.position) && vec3(o.size) && num(o.confidence) && num(o.last_seen) && str(o.status)),
  transcript: (d) => str(d.id) && ["user", "maverick", "system"].includes(d.role) && str(d.text),
  agent_step: (d) => str(d.plan_id) && str(d.goal) && arr(d.steps) && d.steps.every((s: D) => str(s.id) && str(s.label) && str(s.status)),
  skill_progress: (d) => str(d.skill) && str(d.state) && num(d.progress) && str(d.message),
  status: (d) => num(d.battery_pct) && typeof d.link === "object" && str(d.link.quality) && str(d.mode) && typeof d.armed === "boolean" && num(d.vram_used_gb) && num(d.vram_total_gb),
};

const seen = new Map<string, number>();
let bad = 0, photo = false, voxels = 0, maxBoxes = 0;
const ws = new WebSocket(url);
const timer = setTimeout(() => finish("timeout"), Number(process.env.SMOKE_TIMEOUT_S ?? 90) * 1000);
ws.on("message", (raw) => {
  const m = JSON.parse(String(raw));
  if (!(GATEWAY_EVENT_TYPES as string[]).includes(m.type) || !num(m.t) || typeof m.data !== "object") { bad++; console.error("bad envelope", String(raw).slice(0, 200)); return; }
  seen.set(m.type, (seen.get(m.type) ?? 0) + 1);
  if (!checks[m.type as GatewayEventType](m.data)) { bad++; console.error("bad data for", m.type, JSON.stringify(m.data).slice(0, 300)); }
  if (m.type === "map_update") voxels += m.data.add.length / 3;
  if (m.type === "detections") maxBoxes = Math.max(maxBoxes, m.data.boxes.length);
  if (m.type === "transcript" && m.data.role === "maverick" && m.data.attachments?.length) { photo = true; }
  if (photo && GATEWAY_EVENT_TYPES.every((t) => seen.has(t))) finish("all event types and a photo card received");
});
ws.on("error", (e) => { console.error("ws error", e.message); process.exit(2); });
function finish(why: string): void {
  clearTimeout(timer);
  const missing = GATEWAY_EVENT_TYPES.filter((t) => !seen.has(t));
  console.log(`smoke: ${why}; voxels ${voxels}; max boxes ${maxBoxes}; counts`, Object.fromEntries(seen));
  const ok = bad === 0 && missing.length === 0 && photo;
  console.log(ok ? "smoke: OK" : `smoke: FAIL bad=${bad} missing=${missing.join(",")} photo=${photo}`);
  ws.close();
  process.exit(ok ? 0 : 1);
}
