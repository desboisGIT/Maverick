from maverick_mapping.explore_offline import ExploreConfig, run_exploration
from maverick_mapping.synthetic import Scene


def test_explores_studio_offline():
    cfg = ExploreConfig(budget=120.0, depth_noise=False)
    res = run_exploration(Scene.from_seed(1000002), cfg)
    m = res.metrics
    assert m["coverage"] > 0.85
    assert m["time_s"] < 120.0
    assert m["wall_s"] < 60.0
