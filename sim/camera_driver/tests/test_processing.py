import numpy as np
from camera_driver.processing import (
    Pairer,
    depth_to_mm,
    downsample_depth,
    downsample_semantic,
)


def test_depth_to_mm_invalid_values_become_zero():
    d = np.array([[0.1, 0.2, 1.2345, 19.1], [19.2, np.inf, np.nan, -1.0]], dtype=np.float32)
    mm = depth_to_mm(d, 0.2, 19.1)
    assert mm.dtype == np.uint16
    assert [mm[0, 0], mm[0, 1], mm[0, 3]] == [0, 200, 19100]
    assert mm[1].tolist() == [0, 0, 0, 0]
    assert abs(int(mm[0, 2]) - 1234.5) <= 0.51


def test_downsample_depth_takes_nearest_valid():
    d = np.zeros((8, 8), dtype=np.uint16)
    d[0:4, 0:4] = 3000
    d[1, 1] = 1500  # nearest in block (0,0)
    d[0, 0] = 0  # invalid pixels ignored
    d[4:8, 4:8] = 0  # block with no valid pixel stays 0
    d[0:4, 4:8] = 5000
    d[4:8, 0:4] = 65534
    out = downsample_depth(d, 4)
    assert out.shape == (2, 2) and out.dtype == np.uint16
    assert out.tolist() == [[1500, 5000], [65534, 0]]


def test_downsample_depth_full_size():
    d = np.full((480, 640), 2500, dtype=np.uint16)
    assert downsample_depth(d, 4).shape == (120, 160)


def test_downsample_semantic_nearest_no_averaging():
    lab = np.zeros((8, 8), dtype=np.uint16)
    lab[:, :4] = 12
    lab[:, 4:] = 25
    lab[2, 2] = 7  # block centre sample (offset 2)
    out = downsample_semantic(lab, 4)
    assert out.dtype == np.uint8 and out.shape == (2, 2)
    assert out.tolist() == [[7, 25], [12, 25]]


def test_pairer_exact_stamps():
    p = Pairer()
    assert p.add("rgb", 1000, "r1") is None
    assert p.add("depth", 1000, "d1") is None
    t, parts = p.add("semantic", 1000, "s1")
    assert t == 1000 and parts == {"rgb": "r1", "depth": "d1", "semantic": "s1"}


def test_pairer_out_of_order_and_stale_dropped():
    p = Pairer(tolerance_ns=50)
    p.add("rgb", 100, "r100")
    p.add("depth", 300, "d300")  # no partner
    p.add("semantic", 300, "s300")
    t, parts = p.add("rgb", 300, "r300")
    assert t == 300 and parts["depth"] == "d300" and parts["rgb"] == "r300"
    # the stale rgb 100 was discarded, so a late partner for it never pairs
    assert p.add("depth", 100, "d100") is None
    assert p.add("semantic", 100, "s100") is None


def test_pairer_tolerance():
    p = Pairer(tolerance_ns=5_000_000)
    p.add("rgb", 1_000_000_000, "r")
    p.add("depth", 1_003_000_000, "d")
    t, parts = p.add("semantic", 1_000_000_000, "s")
    assert parts["depth"] == "d"
    p2 = Pairer(tolerance_ns=1_000_000)
    p2.add("rgb", 1_000_000_000, "r")
    p2.add("depth", 1_003_000_000, "d")
    assert p2.add("semantic", 1_000_000_000, "s") is None


def test_pairer_stream_of_sets():
    p = Pairer()
    out = []
    for k in range(10):
        t = k * 200_000_000
        for s in ("depth", "semantic", "rgb"):
            got = p.add(s, t, f"{s}{k}")
            if got:
                out.append(got[0])
    assert out == [k * 200_000_000 for k in range(10)]


def test_pairer_drops_one_missing_stream_without_blocking():
    p = Pairer()
    out = []
    for k in range(6):
        t = k * 200_000_000
        for s in ("rgb", "depth", "semantic"):
            if k == 2 and s == "semantic":
                continue  # one lost image
            got = p.add(s, t, f"{s}{k}")
            if got:
                out.append(got[0])
    assert out == [k * 200_000_000 for k in (0, 1, 3, 4, 5)]
