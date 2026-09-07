# Stage S2: random apartment worlds

Status: plan, written 2026-11-11. Parent plan: [simulation-plan.md](simulation-plan.md) §3.1 and §9 (stage S2). Tags as in the study: [F] verified, [E] estimate, [H] hypothesis, [NV] not verified.

## Goal and exit test

A seeded generator that builds a random, furnished apartment as a Gazebo world plus a ground-truth manifest, so every later capability is developed and scored on flats the system has never seen.

**Exit test:**
1. 100 consecutive seeds from each seed range generate, and every one passes the validity checks below (unit tests, no simulator needed).
2. Every generated world loads in Gazebo Harmonic headless without errors.
3. The maverick drone takes off, hovers 10 s and lands, scored on ground truth, in 20 generated worlds (10 tuning seeds, 10 held-out seeds): 20/20.

## Design

Package `sim/worldgen/` (`worldgen`), Python 3.12 standard library only (no numpy, no external assets: everything is built from primitives so licensing is trivial and generation is fast).

Pipeline, all driven by one `random.Random(seed)`:

1. **Programme:** flat type (studio, 1-bed, 2-bed, 3-bed) → room list. Every flat has a kitchen (sometimes open to the living room), a living room, a bathroom; bedrooms by type; an optional hallway and office.
2. **Floor plan:** a rectangular or L-shaped footprint (about 30–110 m² [E]) split recursively into axis-aligned rectangular rooms. Adjacency rules: bathroom off the hallway or a bedroom, kitchen next to the living room, bedrooms not only reachable through another bedroom. Minimum room side 2.0 m, bathroom ≥ 3 m², living room ≥ 12 m².
3. **Openings:** doors 0.8–1.0 m wide (the study's ≥ 0.7 m rule, with margin for a 3.5–5" drone), open-plan gaps for open kitchens; every room reachable from the spawn room. Windows are visual only (S2).
4. **Geometry:** walls 0.1 m thick, ceiling height 2.4–2.7 m; floors, walls, ceiling as SDF boxes with collisions. Ceiling on by default, `--no-ceiling` for the demo's top-down view.
5. **Textures without image files:** optical flow needs floor texture (S1 finding), so floors are built as tiles or planks of slightly varied colours (like `sim/worlds/textured.sdf`), with per-flat palettes; walls get a palette colour and a few decor patches (pictures, skirting) so VIO in S3 has features. A low-texture floor option exists for testing the flow-quality risk, off by default.
6. **Furniture:** a small library of primitive-built assets (boxes and cylinders) with a semantic class, footprint, height and named support surfaces: kitchen counter, fridge, oven, dining table, chairs, sofa, coffee table, TV unit, bed, wardrobe, desk, office chair, bathtub/shower, toilet, sink, shelves, plants. Per-room rules place them against walls or in the centre, keeping a clear walking corridor (≥ 0.8 m) from every door into the room.
7. **Lighting:** day, evening, lamp-only presets (directional sun through windows is out of scope for S2).
8. **Spawn:** a free spot (≥ 1.0 m clearance in all directions, under open ceiling space) in the living room or hallway.
9. **Output:** `apt_<seed>.sdf` (the `<world name>` equals the file stem, as PX4 requires) with the world-level systems PX4 needs (copied from `sim/worlds/textured.sdf`), and `apt_<seed>.json` manifest.

### Manifest (ground truth, read only by the referee)

```json
{
  "seed": 7, "seed_set": "tuning", "generator_version": "1",
  "footprint": [[x, y], ...], "ceiling_height": 2.5,
  "rooms":  [{"id": "kitchen_0", "type": "kitchen", "polygon": [[x, y], ...], "area_m2": 9.1}],
  "doors":  [{"id": "d3", "rooms": ["hallway_0", "kitchen_0"], "center": [x, y], "width": 0.9}],
  "furniture": [{"id": "table_2", "class": "dining_table", "room": "kitchen_0",
                 "pose": [x, y, z, yaw], "bbox": [[xmin, ymin, zmin], [xmax, ymax, zmax]],
                 "surfaces": [{"name": "top", "height": 0.75, "polygon": [[x, y], ...]}]}],
  "spawn": {"room": "living_0", "pose": [x, y, z, yaw]},
  "lighting": "evening"
}
```

Support surfaces are where S3/S4 scenarios will place objects (bread on the table, bananas on the counter).

### Seed ranges

- Tuning: 0–99 999. Held-out: 1 000 000–1 099 999. `worldgen` records which set a seed belongs to; the referee refuses to score capability results on tuning seeds once S3 starts.

### Validity checks (unit tests)

Rooms inside the footprint and non-overlapping; total room area ≈ footprint area; every room reachable from the spawn room through doors or open-plan gaps (graph search); door widths in range; no furniture intersecting walls, doors' clearance zones or other furniture; spawn clearance; required room types present; manifest and SDF agree (same furniture ids); SDF is well-formed XML with the world name equal to the stem; generation is deterministic (same seed → byte-identical files); generation of one flat < 1 s.

### Integration with the stack

- Generated files go to `sim/worlds/generated/` (git-ignored); the sim container must find them by world name and spawn the drone at the manifest's spawn pose (PX4's Gazebo start script reads the world name from `PX4_GZ_WORLD` and a model pose variable [NV: check `px4-rc.gzsim` for the exact variables]).
- The referee's hover trial gains `--world apt_<seed>`.

## Work split

| Part | Owner |
|---|---|
| `worldgen` package, asset library, manifest, unit tests, Gazebo headless load check | agent D |
| PX4 world/spawn integration, flight check in 20 worlds | orchestrator, after the S1 exit test |
