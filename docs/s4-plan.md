# S4 plan: agent, skills and voice ("bananas")

Stage S4 of `docs/simulation-plan.md` §9. Exit: **H1 and H2 pass 8/10 held-out seeds, by voice; demo acts 1–2 work live.** Claims are tagged [F] verified, [E] estimate, [H] hypothesis, [NV] not verified.

## 0. Where S3 leaves us

- Skills `explore` and `goto_room` exist in `maverick_skills` (JSON on `/maverick/skill/request`, progress on `/maverick/skill/progress`). The scene graph (`maverick_memory`) answers room and object queries. The Mission Console shows map, pose, path and skill progress over the gateway websocket [F].
- Closed-loop exploration in Gazebo needs accurate localisation. S4 development uses simulated motion capture (`MAVERICK_LOCALISATION=mocap`) and labels every result "with mocap" until Track B localisation lands (docs/s3-plan.md) [F].
- The owner provided DeepSeek and Anthropic API keys (git-ignored `docker/.env.secrets`, passed to the `host` container as an optional env file). Without the file the agent falls back to the scripted backend, so CI never needs a key or the network [F].

## 1. Sub-stages

| Sub-stage | What | Exit |
|---|---|---|
| **S4a Agent core** | `maverick_agent`: tool registry generated from typed skill specs, the agent loop, three backends behind one interface (`scripted`, `claude`, `local`), transcript events | Unit tests on the loop with the scripted backend; 20 typed commands mapped to the right tool calls with the cloud backend (≥ 19/20) |
| **S4b Skills and executive** | New skills `search(object, region)`, `come_to(person)`, `report(text, photo)`, `return_home`; the executive validates every call (room known, person distance ≥ 1.0 m, budget) and runs one motion skill at a time with cancel | `search` finds a placed object in 8/10 tuning seeds with mocap; the executive rejects all 12 hand-written unsafe calls |
| **S4c Typed missions end to end** | Console text input → gateway → agent → skills → spoken answer (TTS) + photo card | H2 by text passes 8/10 tuning seeds |
| **S4d Voice** | Simulated mic on the drone (referee renders the speaker's audio at the drone with distance attenuation + propeller noise), bridge streams raw audio, host runs VAD → wake word → ASR; TTS out through the console | Word error rate < 10 % on 50 scripted commands at 1.5–4 m with props on [E]; H1 and H2 by voice pass 8/10 **held-out** seeds |

S4a and the scripted-speech parts of S4d do not need the simulator and can run in parallel with S4b.

## 2. Agent design

**One loop, manual, owned by us** (not an SDK tool runner), because tools are long-running physical actions that the operator can interrupt:

1. Operator utterance arrives (text or ASR final). A fixed acknowledgement ("On it.") is spoken at once from a template, not from the model, so the reply starts in < 1 s [E].
2. The model is called with the stable system prompt + tool list (cached prefix) + the session history + a short **state block** (known rooms, objects seen, people, current skill, battery) appended as the last user content, so it never invalidates the cache.
3. Each `tool_use` goes to the executive, which validates it. Invalid → `tool_result` with `is_error: true` and the reason. Valid motion skills run to completion; the `tool_result` is sent when the skill ends (success, failure reason, or "cancelled: operator said …"). New speech during a skill cancels it only if the agent, given the new utterance, decides so; a "stop"/"land" keyword bypasses the model and goes straight to the executive.
4. Parallel tool calls are allowed only for non-motion tools (`memory_query`, `say`); the executive serialises motion.
5. The loop ends on `end_turn`. A per-mission cap (25 model calls, 15 min) ends runaway loops with a spoken apology.

**Tools exposed to the model** (strict JSON schemas, `additionalProperties: false`): `explore(budget_s)`, `goto_room(room)`, `search(object, region?, budget_s)`, `come_to(person)`, `return_home()`, `memory_query(question)`, `say(text)`, `send_photo(object_id | "current_view", caption)`. No tool takes coordinates or setpoints (CLAUDE.md decision) [F: by construction].

**Backends** (one interface, chosen by config `agent.backend`). **Default: `deepseek`** (owner decision 2026-11-11, from the measurements below); `claude` stays available behind the same interface and is re-compared on the end-to-end H2 runs.

20-command eval (`python3 -m maverick_agent.eval_commands`, fake executive and memory, rule-based scoring) [F]:

| Backend | Correct | Invalid calls | Model-call latency p50 / p95 | Cost per 20 commands |
|---|---|---|---|---|
| `deepseek` (`deepseek-flash`, thinking off) | 19/20 | 0 | 1.07 / 1.35 s | €0.006 |
| `claude` (`claude-opus-5-5`, effort low), after the history fix | 20/20 | 0 | 2.18 / 3.02 s | €0.17 |

DeepSeek's miss: "make me a coffee" made it query memory and start `goto_room kitchen` before declining. Claude's first runs lost 2–3 missions to a loop bug (the state block was appended to the sent message but not stored, so the replayed history differed and Claude rejected the thinking-block signatures); fixed in `loop.py`, verified 20/20 with 0 backend errors and 86 % of input tokens served from the prompt cache.


- `scripted`: maps utterance patterns to fixed tool-call sequences. Used by CI and the failsafe suite; no network.
- `claude`: Anthropic Python SDK, `client.messages.create`. Model `claude-opus-5-5` by default, `output_config.effort` = `low` for conversational turns (latency) and configurable per role; adaptive thinking is always on for this model. `tool_choice` stays `auto` (forced tool choice is rejected by this model). Prompt caching on the tools + system prefix (`cache_control` breakpoint after the system prompt); check `usage.cache_read_input_tokens` in the logs. Handle `stop_reason == "refusal"` with server-side fallbacks enabled and a spoken "I can't do that one." Typed SDK errors: rate limits and 5xx retry, then the drone says it lost its connection and keeps hovering (nothing flight-critical depends on the cloud).
- `deepseek`: DeepSeek chat completions through the `openai` SDK (`https://api.deepseek.com`, model `deepseek-flash`, thinking disabled for latency). Same error policy as `claude`.
- `local`: an on-host model for offline demos, added only after a VRAM check against the 16 GB budget (§10 of the simulation plan). Not needed for the S4 exit.

**Cassettes:** every cloud run records request/response pairs; CI replays them (`scripted` backend can read cassettes), so scenario tests are deterministic and free.

## 3. Executive and new skills

- The executive is the existing `skill_node` grown up: same topics, plus a validation step and an action-like request id so the agent can match results to calls. Pure-Python core (`executive.py`, unit tested) + thin rclpy node, as in S0–S3.
- `search(object, region)`: go to the region (room), fly a scan pattern that points the camera at every surface seen in the map within the room, stop when the detector reports the object with confidence ≥ threshold on ≥ 2 frames, store a photo crop in memory. Detector in sim: the segmentation-camera detector from S3, extended with a `banana`/`bread` class; real detector (YOLOE with text prompts) is S5 work.
- `come_to(person)`: go to the person's last known position from memory, stop at 1.5 m, face them. In S4 people are static actors with a known appearance class; re-identification is S6.
- Worldgen adds small objects (bananas, bread) on kitchen counters/tables with a presence probability (absent 30 % of runs) and records them in the manifest for the referee only.

## 4. Voice

- **Mic simulation (referee side, ground truth allowed there):** the referee takes the scripted utterance (TTS-generated speech with several voices, or a recorded file), attenuates it by distance from speaker to drone, adds propeller noise scaled with thrust, and streams 16 kHz mono frames into the sim container, which plays the role of the drone's mic. The drone stack only sees the audio stream [H: a real mic array needs beamforming; one channel is enough for S4].
- **Host:** VAD (Silero, CPU) → wake word "Maverick" (openWakeWord, CPU) → ASR (faster-whisper small on GPU, or a cloud ASR behind the same interface) → agent. TTS: Kokoro on CPU, audio to the console and to a simulated drone speaker [E: < 1 GB VRAM for ASR; check before adding].
- The console microphone path (owner talks live) feeds the same referee mixer, so live demo and tests share one pipeline.

## 5. Scoring

- H1/H2 scenario files as in simulation plan §8; the referee checks: spoken answer vs manifest (yes/no rule check; counts by a small LLM judge later), photo shows the object when present, ends within 1.5 m of the speaker, 0 collisions, time limit.
- Seeds: tune on 0–99999, report on held-out 1000000+ only. Results with mocap are labelled as such and do not count as the final exit until Track B localisation passes.

## 6. Owner decisions and open items

- Keys: DeepSeek (about €3 credit) and Anthropic (about €5 credit), in `docker/.env.secrets`. Every eval run takes a hard cost cap (`--cap-eur`).
- Default agent model: DeepSeek (cheaper by about 30×, faster by about 2× on the eval, one harmless miss). Re-compare with Claude on end-to-end H2 missions in the simulator.
- Vision (confirming objects in photos, roof damage, person appearance) needs an image-capable model; which provider is decided at S5.

## 7. Order of work

1. S4a with the scripted backend (no sim needed) — in parallel with the S3 closed-loop fixes.
2. Worldgen small objects + detector classes + `search` skill (needs the sim; after the S3 fixes land).
3. Executive validation + `come_to` with a static actor.
4. Console text path + TTS → H2 by text on tuning seeds.
5. Mic simulation + ASR → H1/H2 by voice on held-out seeds.

## 8. S4b built (branch of 2026-11-11; with mocap, tuning seeds only, not exit results)

**What exists** [F]
- Worldgen extras (`sim/worldgen/worldgen/extras.py`): classes `banana`, `bread`, `person` appended (ids 27-29, nothing renumbered). Each small object is present with probability 0.7 on a kitchen counter or table (absent 30 %); a mannequin (red shirt, collides) stands in a living room, bedroom or office at least 1.6 m from the take-off point (about 87 % of tuning seeds 0-99 get one). Own random stream, so layouts are unchanged; manifest keys `small_objects`, `presence`, `people` are referee-only. The S3 referee still generates worlds without extras (`make_world(extras=False)`).
- Detector and memory: the segmentation detector reports the new classes (small ones need 6 pixels instead of 32, same noise model); detection payloads carry the lifted map position; memory keeps the best RGB crop of small objects and people (`/tmp/maverick_photos/<obj>.jpg`, path in the `find` answer), `find` can return unconfirmed objects, `search_plan` lists rooms with a goal and the surfaces in them.
- Executive (`maverick_skills/executive.py`, pure; wired into `skill_node`): validates every request (grammar, budgets 10-900 s, room/person known, stand-off never under 1.0 m, one motion skill at a time, cancel always works), assigns a request id (the request's `id`, else `ex-n`) echoed as `request_id` in every progress message; structured outcomes under `result`; rejections come back as `failed` with `rejected: true` and a `code`. Run-time guard: a goto/search/come_to/return goal closer than 1.0 m to a known person ends the skill with "refused". The agent's `RosExecutive` now sends ids and ignores results of other requests; `return_home` sends `land: true`.
- Skills: `search.py` (plan from memory, fly to each room goal, 360 degree yaw sweep, viewpoints 1.3 m from each surface, stop on 2 frames at confidence 0.6 within 0.5 m of each other, confirm with memory, report position and photo; else "not found, searched ..."), `come_to.py` (person from memory, free stop point 1.35 m away on the approach side, face the person; `ReturnSkill` flies to the first-hover point, optionally lands). Built on the goto A* via `GotoXY`; planner and explore are untouched.
- Referee: `just test-s4 --seeds 10-19 --stacks 3` (`referee/s4.py`, verdicts in `s4_verdicts.py`): explore, search banana, search bread, come_to, return, all in one run.

**Tests** [F]: 12+ unsafe calls (24 cases) rejected in `test_executive.py`; `pytest` 363 tests pass except the unrelated `test_protocol.py` websocket test that times out under `--network none` (also before these changes).

**Results**, tuning seeds 10-19, small drone, mocap, explore 300 s, one run each, stack 3 [F]

| Check | Result |
|---|---|
| search correct (banana + bread) | 19/20: present 13/14 found within 0.3 m (error 0.01-0.13 m), absent 6/6 correctly not found, 0 false positives |
| search, per seed (both objects right) | 9/10 |
| come_to (7 seeds with a person; 1.35 m stand-off) | 6/7 ended 1.29-1.39 m away, never under 1.0 m; seed 15 ended 1.65 m (a fall-back ring point, ring now capped at +0.1 m, **not re-flown** [NV]) |
| return | 6/7; seed 12 refused because the path went 0.72 m from the person |

First dev run (seed 10) with stand-off 1.5 m ended 1.59 m away, so the default became 1.35 m.

**Open issues**
- [F] Seed 14: banana present, not found after searching kitchen, living, bathroom, hallway; not investigated.
- [F] The 1.0 m guard fails `return` when the shortest path passes a person (seed 12); the path planner does not know about the person clearance. Explore is not guarded at all (planner untouched by request).
- [F] Perception is the oracle segmentation detector with noise; a real detector will see bananas far worse (a banana is 6-40 pixels at 160x120).
- [NV] Person position is the visible-surface box centre (about 0.1 m bias); 3 of 10 seeds had no person so come_to ran on 7.
- [NV] One run per seed; no run-to-run noise measured. Held-out seeds untouched.
