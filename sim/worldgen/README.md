# worldgen

Seeded random apartment worlds for Gazebo Harmonic, plus a ground-truth manifest. Python 3.12+,
standard library only. Plan and exit test: [docs/s2-plan.md](../../docs/s2-plan.md).

## Usage

```
just worldgen seed="7"                      # sim/worlds/generated/apt_7.sdf + apt_7.json
just worldgen 7 --count 5 --no-ceiling      # five flats, no ceiling (top-down view)
PYTHONPATH=sim/worldgen python3 -m worldgen --seed 7 --out /tmp/w --lighting evening --low-texture
```

Drone profile `--drone x500|small` (default `x500`, recorded as `drone_profile` in the manifest):

| | x500 (circumscribed radius about 0.39 m) | small (3.5-5 inch) |
|---|---|---|
| door width | 1.1, 1.2, 1.3 m | 0.8, 0.9, 1.0 m |
| open-plan gap | at least 1.4 m | at least 1.2 m |
| corridor between furniture and door zones | 1.2 m, zones 1.0 m deep | 0.8 m, 0.8 m deep |
| spawn clearance | 1.2 m | 1.0 m |
| room minimum sides (living/kitchen/bath/bed/office/hall) | 3.4/2.4/2.2/2.8/2.4/1.5 m | 3.2/2.0/2.0/2.4/2.0/1.2 m |

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

## Semantic labels

Every model carries Gazebo's Label system, for the segmentation camera:
`<plugin filename="gz-sim-label-system" name="gz::sim::systems::Label"><label>ID</label></plugin>`
at model level. All instances of a class share one id. The table is in `worldgen/classes.py`
and in the manifest as `classes` (0 = background). Structural classes: floor 1 (also the ground
plane), wall 2, ceiling 3, door 4 (lintels), window 5, decor 6 (pictures). Furniture follows in
fixed order from 7: kitchen_counter, fridge, oven, dining_table, chair, sofa, coffee_table, tv_unit,
bed, nightstand, wardrobe, desk, office_chair, bathtub, shower, toilet, sink, shelves, plant,
rug (26 in total). Never reorder; append only.

## Algorithm

All draws come from one `random.Random(seed)`; decor, floors and lighting use separate
sub-streams so options do not shift the layout.

1. `programme.py`: flat type (studio, 1-bed, 2-bed, 3-bed), room list with target areas,
   minimum side, minimum area and aspect limits per type (per drone profile, see the table),
   room order so that list neighbours tend to be spatial neighbours.
2. `floorplan.py`: rectangular or L-shaped footprint (two wings) sized to the programme, then a
   binary space partition: the ordered room list is cut recursively, along the longer side
   mostly, at the area fraction, clamped so every part is thick enough. Leaves must satisfy the
   per-type limits, otherwise the attempt is redrawn.
3. `openings.py`: candidate doors on shared walls (at least 1.2 m long), filtered by room-type rules
   (bathroom only off hallway or bedroom, no bedroom-bedroom doors, kitchen next to living).
   A randomised Kruskal spanning tree guarantees connectivity, plus up to two extra loops between
   public rooms. Door widths come from the drone profile; an open kitchen gets a gap up to 2.4 m instead.
4. `furniture.py`, `assets.py`: per-room recipes place assets from the data library with
   rejection sampling: inside the room, clear of other furniture, clear of door zones (profile depth),
   and the doors of the room must stay connected by the profile's corridor (`clearance.py`). A spawn
   point with the profile's clear floor in all directions is searched after furnishing. If a required item
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
seeds must all validate for both profiles and each flat must generate in under 1 s; determinism; schema; asset
library consistency. Gazebo load check, in the tools container:
`gz sdf -k sim/worlds/generated/apt_7.sdf` and
`gz sim -s -r --iterations 500 sim/worlds/generated/apt_7.sdf`.
