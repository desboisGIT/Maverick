# Gateway events (protocol v1)

The contract between the **gateway** (a ROS 2 node that bridges Maverick to the browser) and the **Mission Console** (`console/`). The source of truth for types is `console/src/gateway/types.ts`; this document describes the same fields. Change both together. The mock gateway (`console/mock/`) implements the gateway side and is the reference for what a real stream looks like.

## Transport

- One websocket, text frames, one JSON object per frame. The gateway listens (default port 8765); the console connects and reconnects on its own.
- Every frame is an envelope: `{"type": <string>, "t": <number>, "data": <object>}`.
  - `t`: Unix time in seconds (float) at which the data was produced.
- Gateway to console: the 12 event types below. Console to gateway: 4 input types (last section).
- Unknown `type` values are ignored by both sides, so new events can be added without breaking old peers.
- **State on connect.** When a console connects, the gateway must first send the current state: a `map_update` with `reset: true` containing all occupied voxels, then the latest `rooms`, `objects`, `frontiers`, `path`, `agent_step`, `skill_progress`, `status` and `pose`, then the recent `transcript` messages. After that it streams deltas.

## Coordinate frame and units (read this first)

All positions are in the **map frame**:

- ROS REP 103 **ENU**: **x = east**, **y = north**, **z = up**. Metres, right-handed. Same frame as Gazebo worlds and the worldgen manifests. Origin as defined by the stack (the mock uses the manifest origin).
- The **gateway passes ENU values through with no axis conversion**. The console converts to three.js axes in one place (`M2T` in `console/src/panels/mind.ts`).
- **yaw**: radians, counter-clockwise seen from above, 0 = +x (east), turning left is positive. Range any real, the console wraps it. **pitch** positive nose up, **roll** positive right side down, radians.
- Image coordinates are normalised to the image: `[0, 1]`, origin top-left, x right, y down.
- Velocities in m/s in the map frame. Times in seconds. Battery in percent.

## Gateway to console

Types: `Vec3 = [x, y, z]` metres, `Vec2 = [x, y]` metres. "Full replacement" events replace the console's whole copy of that thing each time (they are small); send them when something changes.

### `pose` (stream, 10 Hz)

| field | type | meaning |
|---|---|---|
| position | Vec3 | drone position, m |
| yaw, pitch, roll | number | rad, see above |
| velocity | Vec3 | m/s |

### `camera_frame` (stream, 5 to 10 Hz)

| field | type | meaning |
|---|---|---|
| seq | int | increasing frame counter; detections refer to it |
| width, height | int | pixels |
| mime | string | `image/jpeg` from the real gateway (v1). The mock uses `image/svg+xml`. `image/png` also works |
| data | string | base64 of the encoded image, no `data:` prefix. About 15 to 40 KB for 640x480 JPEG q70 |
| hfov_deg | number | horizontal field of view, degrees (the console draws the camera wedge from it) |

A URL-based transport (frames served over HTTP) may replace the base64 in a later version.

### `detections`

Boxes for one frame. Sent right after the matching `camera_frame`.

| field | type | meaning |
|---|---|---|
| frame_seq | int | `camera_frame.seq` these belong to |
| boxes[] | | one per detection |
| boxes[].label | string | class name |
| boxes[].confidence | number | 0..1 |
| boxes[].box | [x, y, w, h] | normalised to the image, top-left origin |
| boxes[].track_id | int, optional | tracker id |
| boxes[].object_id | string, optional | id of the matching `objects` entry |

### `map_update` (delta)

Voxel map as occupied voxels only (free and unknown are implicit).

| field | type | meaning |
|---|---|---|
| resolution | number | voxel edge, m (mock: 0.1) |
| origin | Vec3 | map-frame position of the corner of voxel (0,0,0) |
| reset | bool | true: drop the whole map before applying `add` |
| add | int[] | flat `[ix, iy, iz, ix, iy, iz, ...]` newly occupied voxels |
| remove | int[] | same layout, voxels that became free |

Voxel centre = `origin + (index + 0.5) * resolution` per axis. Indices may be negative. Send deltas at 2 to 5 Hz, batching changes; a full map is only sent on `reset`. About 20k voxels is comfortable; the console renders them as one instanced mesh.

### `frontiers` (full replacement)

`frontiers[]`: `{id: string, position: Vec3, size: int (cells in the cluster), gain?: number}`. The console shows an amber marker at each `position`.

### `path` (full replacement)

| field | type | meaning |
|---|---|---|
| path_id | string | |
| status | `planned` \| `executing` \| `done` \| `failed` | `failed` is drawn red |
| waypoints | Vec3[] | in flight order, first = where the drone is now. Empty clears the path |
| goal | Vec3, optional | drawn as a ring; defaults to the last waypoint |

### `rooms` (full replacement)

`rooms[]`: `{id, type, label, polygon: Vec2[], confidence, visited}`. `type` is one of `unknown, living, kitchen, bedroom, bathroom, hallway, office, other` (`unknown` until typed; the console tints the floor polygon by type and shows `label`). `polygon` is the floor outline in the map frame, any winding. `confidence` 0..1 is for the type.

### `objects` (full replacement)

`objects[]`: `{id, label, position: Vec3 (box centre), size: Vec3 (extent along x, y, z), room_id?, confidence, last_seen (Unix s), status}`; `status` is `present`, `gone` (was here, the drone looked and it is not) or `unverified`. The box is axis-aligned in the map frame.

### `transcript`

One chat message: `{id, role: user|maverick|system, text, partial?, source?: voice|text, attachments?}`. A message with an `id` already seen **replaces** that message (partial speech recognition, then the final text). `attachments[]`: `{kind: "photo", mime, data (base64), caption?, object_id?, label?}` (`label`: the class the photo claims to show). The gateway echoes the user's own input (typed or recognised) as a `user` message; the console does not add local copies.

### `agent_step` (full replacement)

The agent's plan: `{plan_id, goal, steps[]}`; step `{id, label, status: pending|running|done|failed|skipped, detail?, elapsed_s?}`. `elapsed_s` is the time the step has run (running) or took (done); the console ticks running timers locally between updates. An empty `steps` clears the panel.

### `skill_progress`

Feedback of the skill the plan is executing: `{skill: explore|goto|search|inspect|follow|monitor|return|report, state: running|succeeded|failed|cancelled, progress: 0..1 or -1 if unknown, message, eta_s?}`. The console shows a bar while `state` is `running`.

### `status` (1 to 2 Hz)

`{battery_pct, battery_v?, link: {quality: good|degraded|lost, rtt_ms?, loss_pct?}, mode: string, armed: bool, vram_used_gb, vram_total_gb}`. `mode` is free text for display (flight mode or current skill). VRAM is the shared GPU budget (16 GB, see the architecture study).

## Console to gateway

Same envelope. The gateway ignores what it does not understand.

| type | data | meaning |
|---|---|---|
| `text_command` | `{text: string, speaker?: string}` | a typed command |
| `ptt_start` | `{}` | push-to-talk pressed (Space held or button). v1: a stub, no audio is sent; the gateway may treat the pair as "listen now" |
| `ptt_stop` | `{}` | released |
| `director` | `{action, params?}` | simulation control, stub in v1 |

`director.action`: `remove_object {object_id? | label?}`, `move_object {object_id, position: Vec3}`, `cut_wifi {seconds}`, `drain_battery {pct}`, `add_person {name}`, and `reset_demo` (mock gateway only).

## Safety note

These events are for display and for natural-language input only. Nothing here carries flight setpoints, and the console must never be on the flight-critical path: losing the websocket must not change what the drone does (see the failsafe decisions in `docs/architecture-study.md`).

## Versioning

`PROTOCOL_VERSION = 1` in `types.ts`. Adding optional fields or new event types is compatible. Changing a field's meaning or units bumps the version.

## Agent node output (`maverick_agent`)

`agent_node` publishes `std_msgs/String` JSON on `/maverick/agent/event` in the envelope above (`{"type", "t", "data"}`) for the gateway to forward unchanged. No new event types: it produces `transcript` (role `maverick` for every spoken line, including the template acknowledgement "On it."; role `user` echoing each operator utterance) and `agent_step` (one plan per operator utterance, `plan_id` = `plan-<n>`, `goal` = the utterance, one step per tool call with label such as "Go to kitchen", status `pending|running|done|failed|skipped`, `detail` = the tool result). Spoken text is also published on `/maverick/agent/say` as `{"text": ...}` for TTS. `send_photo` emits a `transcript` message (role `maverick`, `text` = the caption) with the photo in `attachments[]`: the crop memory saved for the object (`object_id`, `label`), or the latest camera frame for `current_view`. A third, log-only envelope `mission_end` (`plan_id, ended, calls, seconds, tools, usage`) is also on the topic; the gateway does not forward it.

**Agent mode of the gateway** (S4c, parameter `agent:=true`, set by the `s4` host launch): a `text_command` is published as `{"text", "speaker"?}` on `/maverick/operator/text` and nothing else (the agent echoes it as the `user` transcript and answers), and the gateway forwards `transcript` and `agent_step` from `/maverick/agent/event` instead of running its keyword parser and its own one-step plan. "land" still also sends the flight mission "land".
