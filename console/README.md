# Mission Console

Web app that shows what Maverick sees, knows and plans: drone camera with detection boxes, a 3D "Maverick's mind" view (voxel map, rooms, objects, path, frontiers), conversation, agent plan and status. TypeScript, Vite, three.js; everything is bundled (no CDN, no tracking).

Gateway protocol: [`../docs/gateway-events.md`](../docs/gateway-events.md), types in `src/gateway/types.ts`.

Node is not installed on the host; use the Node image. Set `D` once:

```sh
D="docker run --rm -v $PWD:/app -w /app -u $(id -u):$(id -g) -e HOME=/tmp -e npm_config_cache=/tmp/npm node:22"
$D npm install
```

## Run the demo without the simulator

The mock gateway replays a scripted exploration of the seed-7 apartment (explore the flat, then "check the kitchen for bananas" with a photo card):

```sh
docker run -d --rm --name maverick-console -v $PWD:/app -w /app -u $(id -u):$(id -g) -e HOME=/tmp \
  -p 5173:5173 -p 8765:8765 node:22 sh -c "npx tsx mock/server.ts & npx vite --host 0.0.0.0 --port 5173; wait"
# open http://localhost:5173   (stop with: docker stop maverick-console)
```

Mock options (flags or `MOCK_PORT`, `MOCK_SPEED` env): `--port 8765`, `--speed 1` (time scale, `4` for a fast run), `--no-auto` (do not auto-run the script; type commands instead). Typed commands the mock understands: anything with "explore"/"flat", anything with "banana"/"kitchen". Director menu: remove the bananas then ask again, cut Wi-Fi, drain the battery, "Restart demo". The mock keeps state, so a reloaded page picks up where the run is.

Point the console at another gateway with `?gateway=ws://host:8765`.

## Build, typecheck, smoke test

```sh
$D npm run typecheck     # tsc over src/ and mock/
$D npm run build         # typecheck + vite build into dist/
# smoke test: validates every event shape from a running mock (run the mock with MOCK_SPEED=8)
docker exec maverick-console npx tsx mock/smoke.ts
```

`npm run dev` / `npm run mock` / `npm run smoke` are the same commands for a machine with Node.

## Layout

- `src/gateway/` typed envelope (`types.ts`), reconnecting websocket client (`client.ts`)
- `src/panels/` camera, mind (three.js), conversation, plan, status
- `mock/` gateway mock: `world.ts` (voxel world from the manifest), `render.ts` (fake first-person camera), `server.ts` (script, ws server), `smoke.ts`; `apt_7.json` is the worldgen manifest for seed 7
- Not yet: the Gazebo world panel (placeholder), real push-to-talk audio, click-to-point.
