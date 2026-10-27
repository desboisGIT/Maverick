/**
 * Maverick gateway event contract (protocol v1).
 *
 * One websocket, JSON text frames, every frame is an envelope
 *   { "type": <string>, "t": <number>, "data": <object> }
 * `t` is the Unix time in seconds (float) at which the data was produced.
 *
 * COORDINATE FRAME. Every position in this file is in the MAP frame, ROS
 * REP 103 ENU: x = east, y = north, z = up, metres, right-handed (same as
 * Gazebo worlds and the worldgen manifests; the gateway passes it through
 * with no axis conversion). Yaw is radians counter-clockwise seen from
 * above, 0 = +x (east); turning left is positive. Image coordinates are normalised
 *   [0..1], origin top-left, x right, y down.
 *
 * Full documentation: docs/gateway-events.md. Keep the two in sync.
 */

export const PROTOCOL_VERSION = 1;

/** [x east, y north, z up] in metres, map frame. */
export type Vec3 = [number, number, number];
/** [x east, y north] in metres, map frame. */
export type Vec2 = [number, number];

export interface Envelope<K extends string, D> {
  type: K;
  t: number;
  data: D;
}

// ---------------------------------------------------------------- gateway -> console

/** Drone state estimate. Stream, 10 Hz suggested. */
export interface PoseData {
  position: Vec3; // m, map frame
  yaw: number; // rad, CCW from +x (east), see header
  pitch: number; // rad, positive nose up
  roll: number; // rad, positive right side down
  velocity: Vec3; // m/s, map frame
}

/** One camera image. Stream, 5 to 10 Hz suggested. */
export interface CameraFrameData {
  seq: number; // monotonically increasing frame counter, referenced by detections
  width: number; // px
  height: number; // px
  mime: "image/jpeg" | "image/png" | "image/svg+xml"; // real gateway: image/jpeg
  data: string; // base64 of the encoded image, no data: prefix
  hfov_deg: number; // horizontal field of view in degrees
}

export interface DetectionBox {
  label: string; // class name, e.g. "banana"
  confidence: number; // 0..1
  /** [x, y, w, h], normalised to the image, top-left origin. */
  box: [number, number, number, number];
  track_id?: number;
  /** Id of the matching entry in `objects`, once associated. */
  object_id?: string;
}

/** Detector output for one frame. */
export interface DetectionsData {
  frame_seq: number; // CameraFrameData.seq these boxes belong to
  boxes: DetectionBox[];
}

/**
 * Voxel map delta. Voxels are addressed by integer index (ix, iy, iz); the
 * centre of voxel i in the map frame is origin + (i + 0.5) * resolution.
 * `add` and `remove` are flat int arrays [ix0, iy0, iz0, ix1, iy1, iz1, ...].
 */
export interface MapUpdateData {
  resolution: number; // m, edge of one voxel
  origin: Vec3; // m, map-frame position of the corner of voxel (0,0,0)
  reset: boolean; // true: drop the whole map first (also sent as first message to a new client)
  add: number[]; // newly occupied voxels
  remove: number[]; // voxels no longer occupied
}

export interface Frontier {
  id: string;
  position: Vec3; // m
  size: number; // number of free/unknown boundary cells in the cluster
  gain?: number; // expected information gain, unitless, optional
}
/** Full replacement of the current frontier set. */
export interface FrontiersData {
  frontiers: Frontier[];
}

export type PathStatus = "planned" | "executing" | "done" | "failed";
/** The path currently planned or flown. Empty `waypoints` clears it. Full replacement. */
export interface PathData {
  path_id: string;
  status: PathStatus;
  waypoints: Vec3[]; // m, in flight order, first is the current position
  goal?: Vec3;
}

export type RoomType =
  | "unknown"
  | "living"
  | "kitchen"
  | "bedroom"
  | "bathroom"
  | "hallway"
  | "office"
  | "other";

export interface Room {
  id: string;
  type: RoomType; // "unknown" until the memory has typed it
  label: string; // display name, e.g. "Kitchen" or "Room 3"
  polygon: Vec2[]; // floor outline, m, map frame
  confidence: number; // 0..1 for the type
  visited: boolean;
}
/** Full replacement of the known rooms. */
export interface RoomsData {
  rooms: Room[];
}

export type ObjectStatus = "present" | "gone" | "unverified";
export interface MapObject {
  id: string;
  label: string; // class name, e.g. "sofa"
  position: Vec3; // m, centre of the bounding box
  size: Vec3; // m, extent along x, y, z of the axis-aligned box
  room_id?: string;
  confidence: number; // 0..1
  last_seen: number; // Unix seconds
  status: ObjectStatus; // "gone": was here, drone looked and it is not
}
/** Full replacement of the object memory. */
export interface ObjectsData {
  objects: MapObject[];
}

export interface PhotoAttachment {
  kind: "photo";
  mime: CameraFrameData["mime"];
  data: string; // base64
  caption?: string;
  object_id?: string;
  /** Class the photo claims to show (agent photos), e.g. "banana". */
  label?: string;
}
export type TranscriptRole = "user" | "maverick" | "system";
/** A chat message. A message with the same `id` replaces the earlier one (partial speech to final). */
export interface TranscriptData {
  id: string;
  role: TranscriptRole;
  text: string;
  partial?: boolean; // true while speech recognition is still running
  source?: "voice" | "text";
  attachments?: PhotoAttachment[];
}

export type StepStatus = "pending" | "running" | "done" | "failed" | "skipped";
export interface AgentStep {
  id: string;
  label: string;
  status: StepStatus;
  detail?: string;
  /** Seconds the step has been running (or ran for), set by the gateway. */
  elapsed_s?: number;
}
/** The agent's current plan as a checklist. Full replacement. Empty `steps` clears it. */
export interface AgentStepData {
  plan_id: string;
  goal: string;
  steps: AgentStep[];
}

export type SkillState = "running" | "succeeded" | "failed" | "cancelled";
/** Feedback of the skill the plan is currently executing. */
export interface SkillProgressData {
  skill: string; // explore, goto, search, inspect, follow, monitor, return, report
  state: SkillState;
  progress: number; // 0..1, or -1 if unknown
  message: string;
  eta_s?: number;
}

export type LinkQuality = "good" | "degraded" | "lost";
export interface StatusData {
  battery_pct: number; // 0..100
  battery_v?: number;
  link: { quality: LinkQuality; rtt_ms?: number; loss_pct?: number };
  mode: string; // flight mode / current skill, free text: "explore", "hover", "land", ...
  armed: boolean;
  vram_used_gb: number;
  vram_total_gb: number;
}

export interface EventMap {
  pose: PoseData;
  camera_frame: CameraFrameData;
  detections: DetectionsData;
  map_update: MapUpdateData;
  frontiers: FrontiersData;
  path: PathData;
  rooms: RoomsData;
  objects: ObjectsData;
  transcript: TranscriptData;
  agent_step: AgentStepData;
  skill_progress: SkillProgressData;
  status: StatusData;
}
export type GatewayEventType = keyof EventMap;
export type GatewayEvent = { [K in GatewayEventType]: Envelope<K, EventMap[K]> }[GatewayEventType];

export const GATEWAY_EVENT_TYPES: GatewayEventType[] = [
  "pose",
  "camera_frame",
  "detections",
  "map_update",
  "frontiers",
  "path",
  "rooms",
  "objects",
  "transcript",
  "agent_step",
  "skill_progress",
  "status",
];

// ---------------------------------------------------------------- console -> gateway

export interface TextCommandData {
  text: string;
  speaker?: string; // avatar name, default "user"
}
export type PttData = Record<string, never>;

export type DirectorAction =
  | "remove_object" // params: { object_id?: string, label?: string }
  | "move_object" // params: { object_id: string, position: Vec3 }
  | "cut_wifi" // params: { seconds: number }
  | "drain_battery" // params: { pct: number }
  | "add_person" // params: { name: string }
  | "reset_demo"; // mock gateway only
export interface DirectorData {
  action: DirectorAction;
  params?: Record<string, unknown>;
}

export interface ClientEventMap {
  text_command: TextCommandData;
  ptt_start: PttData;
  ptt_stop: PttData;
  director: DirectorData;
}
export type ClientEventType = keyof ClientEventMap;
export type ClientMessage = { [K in ClientEventType]: Envelope<K, ClientEventMap[K]> }[ClientEventType];

export function isGatewayEvent(x: unknown): x is GatewayEvent {
  if (typeof x !== "object" || x === null) return false;
  const o = x as Record<string, unknown>;
  return (
    typeof o.type === "string" &&
    (GATEWAY_EVENT_TYPES as string[]).includes(o.type) &&
    typeof o.t === "number" &&
    typeof o.data === "object" &&
    o.data !== null
  );
}
