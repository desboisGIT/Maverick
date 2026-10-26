# Seed ledger

Held-out apartments (seeds ≥ 1 000 000) are only clean while nobody has looked at them. Once a held-out seed's map, track, logs or failure is inspected to diagnose or tune anything, it is **retired**: it moves to the development pool and its results no longer count as held-out.

Rules:
- Diagnose on tuning seeds (0–99 999). If a held-out failure needs looking into, reproduce it on tuning seeds first.
- When a held-out seed is inspected anyway, add it below the same day.
- Report held-out results only on seeds not listed here.
- Pick fresh held-out seeds from ranges never used before; check `runs/*/results.jsonl` first.

## Retired held-out seeds

| Seeds | Retired | Why |
|---|---|---|
| 1000000–1000004 | 2026-11-11 | Used for S3 development: their failed runs were diagnosed (door-frame collisions, goto goals) and maps of 1000003/1000004 inspected while fixing open-plan kitchens |

## Fresh held-out sets in use

| Seeds | First used | For |
|---|---|---|
| 1000100–1000109 | 2026-11-11 | S3 explore and goto_room (small drone, mocap): goto 8/10, explore 4/10. Re-run 2026-11-12 with the explore changes: explore 6/10. Reporting only, never inspected |
| 1000200–1000219 | 2026-11-12 | S3 explore 9/20 and goto_room 14/20 (small drone, mocap, final code of docs/s3-plan.md "Explore robustness"). Reporting only, never inspected |
| 1000300–1000329 | 2026-11-12 | Final S3 report run after the noise / C6 rule / EV gate / collision guard work: explore and goto_room, one run each, small drone, mocap. Verified unused (no `runs/*/results.jsonl` contained them). Reporting only, never inspected |

## Tuning seeds in use

| Seeds | Used | For |
|---|---|---|
| 10–19 | 2026-11-11 | S3 goto and explore development |
| 20–59 | 2026-11-12 | S3 explore failure statistics and tuning (40 apartments), goto_room check on 20–29 |
| 20–29 | 2026-11-12 | Run-to-run noise (three runs each, before the guard), collision guard and EV-gate development, goto_room regression check |
| 30–79 | 2026-11-12 | Collision guard / EV-gate evaluation on 60 apartments in total (20–79), diagnosed freely (maps, tracks, skill logs) |
