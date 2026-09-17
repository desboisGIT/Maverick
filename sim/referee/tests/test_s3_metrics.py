import numpy as np
from referee import s3_metrics
from referee.s3_frames import spawn_origin
from worldgen import generate


def test_shifted_scene_collisions_and_spawn_clear(tmp_path):
    w = generate(1000003)
    sdf, man = tmp_path / "w.sdf", tmp_path / "w.json"
    sdf.write_text(w.sdf)
    man.write_text(w.manifest_json())
    sc = s3_metrics.load_scene_map_frame(str(sdf), str(man))
    assert np.allclose(sc.spawn[:2], 0.0)
    # at the spawn (map origin), 1.2 m up, nothing is within the body radius
    track = np.array([[0.0, 0.0, 0.0, 1.2], [1.0, 0.0, 0.0, 1.2]])
    assert s3_metrics.collisions(sc, track)["collisions"] == 0
    # a drone pushed into the outer wall counts once, however many samples
    lo, _ = sc.bounds
    wall = np.array([[t, lo[0] + 0.05, lo[1] + 1.0, 1.2] for t in range(5)])
    r = s3_metrics.collisions(sc, wall)
    assert r["collisions"] == 1 and r["min_clearance_m"] < 0.39
    # ground samples are ignored
    ground = np.array([[0.0, lo[0] + 0.05, lo[1] + 1.0, 0.05]])
    assert s3_metrics.collisions(sc, ground)["collisions"] == 0
    assert spawn_origin(w.manifest)[:2] == tuple(w.manifest["spawn"]["pose"][:2])


def test_explore_metrics_end_to_end(tmp_path):
    """Offline exploration in the map frame, then scored through the same path as a real run."""
    from maverick_mapping.explore_offline import ExploreConfig, run_exploration
    from maverick_mapping.map_io import save_map
    from maverick_mapping.synthetic import Scene

    w = generate(1000002)
    sdf, man = tmp_path / "w.sdf", tmp_path / "w.json"
    sdf.write_text(w.sdf)
    man.write_text(w.manifest_json())
    origin = spawn_origin(w.manifest)
    scene_map = s3_metrics.shift_scene(Scene.from_world(w), origin)
    res = run_exploration(scene_map, ExploreConfig(budget=60.0, depth_noise=False))
    save_map(res.vmap, str(tmp_path / "final.npz"))
    track = [[0.5 * k, x + origin[0], y + origin[1], 1.2] for k, (x, y) in enumerate(res.trail)]
    raw = {
        "sdf": str(sdf),
        "manifest": str(man),
        "spawn_pose": w.manifest["spawn"]["pose"],
        "ground_z": 0.0,
        "track": track,
        "t_explore_start": 0.0,
        "t_explore_end": 1e9,
        "snapshots": [{"t": 10.0, "path": str(tmp_path / "final.npz")}],
        "final_map": str(tmp_path / "final.npz"),
    }
    m = s3_metrics.explore_metrics(raw)
    assert abs(m["coverage"] - res.metrics["coverage"]) < 0.02
    assert m["collisions"] == 0
    assert m["surface_err_mean_m"] < 0.1
    assert 0.0 <= m["c6_recall"] <= 1.0
