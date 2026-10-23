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
| 1000100–1000109 | 2026-11-11 | S3 explore and goto_room (small drone, mocap): goto 8/10, explore 4/10. Reporting only, do not inspect |
