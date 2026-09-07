# worldgen

Seeded random apartment worlds for Gazebo Harmonic, plus a ground-truth manifest. Python 3.12+,
standard library only. Plan and exit test: [docs/s2-plan.md](../../docs/s2-plan.md).

## Usage

```
just worldgen seed="7"                      # sim/worlds/generated/apt_7.sdf + apt_7.json
just worldgen 7 --count 5 --no-ceiling      # five flats, no ceiling (top-down view)
PYTHONPATH=sim/worldgen python3 -m worldgen --seed 7 --out /tmp/w --lighting evening --low-texture
```

Options: `--no-ceiling`, `--low-texture` (plain single-colour floors, for the optical-flow risk
test), `--lighting day|evening|lamp_only`. Options never change the layout for a seed.
As a library: `from worldgen import generate; world = generate(7)` gives `world.sdf`,
`world.manifest`, `world.name`. Output is deterministic: the same seed gives byte-identical files.

Seed ranges: tuning 0-99 999, held-out 1 000 000-1 099 999 (`seed_set` in the manifest is
`tuning`, `held_out` or `other`).

## PX4 integration

- World name and file stem are `apt_<seed>`, so `PX4_GZ_WORLD=apt_<seed>` and the file must sit in
  the directory PX4 reads worlds from, as `textured.sdf` does in `docker/compose.yaml`.
- Spawn: manifest `spawn.pose = [x, y, z, yaw]` in the world frame (z = 0, the floor).
  `px4-rc.gzsim` reads `PX4_GZ_MODEL_POSE="x,y,z,roll,pitch,yaw"` [F], so use
  `"x,y,0,0,0,yaw"`.
- The world has no world-level `<plugin>` elements, like `textured.sdf`; PX4's server config
  supplies the systems.

## Algorithm

All draws come from one `random.Random(seed)`; decor, floors and lighting use separate
sub-streams so options do not shift the layout.

1. `programme.py`: flat type (studio, 1-bed, 2-bed, 3-bed), room list with target areas,
   minimum side, minimum area and aspect limits per type (living 3.2 m, hallway 1.2 m, others 2.0-2.4 m),
   room order so that list neighbours tend to be spatial neighbours.
2. `floorplan.py`: rectangular or L-shaped footprint (two wings) sized to the programme, then a
   binary space partition: the ordered room list is cut recursively, along the longer side
   mostly, at the area fraction, clamped so every part is thick enough. Leaves must satisfy the
   per-type limits, otherwise the attempt is redrawn.
3. `openings.py`: candidate doors on shared walls (at least 1.2 m long), filtered by room-type rules
   (bathroom only off hallway or bedroom, no bedroom-bedroom doors, kitchen next to living).
   A randomised Kruskal spanning tree guarantees connectivity, plus up to two extra loops between
   public rooms. Doors are 0.8, 0.9 or 1.0 m; an open kitchen gets a gap up to 2.4 m instead.
4. `furniture.py`, `assets.py`: per-room recipes place assets from the data library with
   rejection sampling: inside the room, clear of other furniture, clear of door zones (0.8 m deep),
   and the doors of the room must stay connected by a 0.8 m corridor (`clearance.py`). A spawn
   point with 1 m clear floor in all directions is searched after furnishing. If a required item
   does not fit, the furniture stage or the whole plan is redrawn.
5. `walls.py`, `floors.py`, `decor.py`, `sdf.py`: walls come from a decimetre grid (a wall exists
   where two cells have different owners), are merged into segments with door gaps cut out
   and lintels above doors. Floors are many slightly different coloured planks or tiles
   (no image files, so optical flow has texture). Windows and pictures are visual-only.
6. `manifest.py`, `validate.py`: manifest as in the plan, plus `flat_type`, `palette`, `windows`
   and per-door `kind` (`door` or `open`). `validate(manifest, sdf)` returns a list of problems.

Furniture ids are `<class>_<n>` and are also the SDF model names.

## Tests

`pytest sim/worldgen/tests` (also part of the repo-wide `pytest`): 100 tuning plus 100 held-out
seeds must all validate and each flat must generate in under 1 s; determinism; schema; asset
library consistency. Gazebo load check, in the tools container:
`gz sdf -k sim/worlds/generated/apt_7.sdf` and
`gz sim -s -r --iterations 500 sim/worlds/generated/apt_7.sdf`.
