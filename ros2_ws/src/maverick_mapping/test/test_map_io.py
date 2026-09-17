import numpy as np
from maverick_mapping.map_io import load_map, save_map
from maverick_mapping.voxel_map import VoxelMap


def test_map_dump_roundtrip(tmp_path):
    v = VoxelMap(0.1)
    v.ensure(np.array([-1.0, -1.0, 0.0]), np.array([1.0, 1.0, 2.0]))
    v.logodds[3:6, 4, 5] = 1.5
    v.seen[2:8, 3:6, 4:7] = True
    v.frames = 7
    save_map(v, str(tmp_path / "m.npz"))
    w = load_map(str(tmp_path / "m.npz"))
    assert w.res == v.res and w.frames == 7
    assert np.array_equal(w.origin, v.origin) and np.array_equal(w.seen, v.seen)
    assert np.allclose(w.occupied_centers(), v.occupied_centers())
    assert np.array_equal(w.project_2d()[0], v.project_2d()[0])
