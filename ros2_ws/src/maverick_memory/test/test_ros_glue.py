"""Pure parts of the memory node: semantic detector, class table, query handling."""

import numpy as np
import pytest
from maverick_mapping import CameraIntrinsics, optical_rotation
from maverick_memory.lifting import Detection
from maverick_memory.queries import answer_query, detections_json
from maverick_memory.scene_graph import SceneGraph
from maverick_memory.semantic_detector import (
    CLASS_IDS,
    SemanticDetector,
    depth_edges,
)
from maverick_memory.sim_detector import DetectorNoise

CLEAN = DetectorNoise(miss_rate=0.0, fp_rate=0.0, box_jitter=0.0, mask_dropout=0.0)


def test_class_table_matches_worldgen():
    classes = pytest.importorskip("worldgen.classes")
    assert CLASS_IDS == classes.CLASSES


def scene(w=80, h=60):
    sem = np.zeros((h, w), np.uint8)
    depth = np.full((h, w), 4.0, np.float32)
    sem[10:30, 5:25] = CLASS_IDS["fridge"]
    depth[10:30, 5:25] = 2.0
    sem[20:50, 40:70] = CLASS_IDS["sofa"]
    depth[20:50, 40:70] = 3.0
    sem[0:5, :] = CLASS_IDS["wall"]  # structural: ignored
    return sem, depth


def test_semantic_detector_finds_each_furniture_component_with_clean_boxes():
    sem, depth = scene()
    dets = SemanticDetector(CLEAN).detect(sem, depth)
    by = {d.label: d for d in dets}
    assert set(by) == {"fridge", "sofa"}
    assert np.allclose(by["fridge"].box, (5.0, 10.0, 24.0, 29.0), atol=2.0)  # 0.5 px jitter
    assert 380 <= by["fridge"].mask.sum() <= 400  # rim corners may be lost


def test_same_class_objects_split_at_depth_jump():
    sem = np.zeros((60, 80), np.uint8)
    depth = np.full((60, 80), 5.0, np.float32)
    sem[10:40, 5:70] = CLASS_IDS["chair"]  # two chairs touching in the image
    depth[10:40, 5:30] = 2.0
    depth[10:40, 30:70] = 3.5
    assert depth_edges(depth).any()
    dets = SemanticDetector(CLEAN).detect(sem, depth)
    assert len(dets) == 2


def test_noise_misses_and_false_positives():
    sem, depth = scene()
    det = SemanticDetector(DetectorNoise(miss_rate=1.0, fp_rate=1.0), seed=1)
    dets = det.detect(sem, depth)
    assert len(dets) == 1 and dets[0].mask is None  # all real ones missed, one false positive
    kept = sum(
        len(SemanticDetector(DetectorNoise(miss_rate=0.1, fp_rate=0.0), seed=s).detect(sem, depth))
        for s in range(50)
    )
    assert 75 <= kept <= 100  # 2 objects x 50 frames, ~10 % missed


def test_detector_scales_with_resolution_and_lifts_to_3d():
    h, w = 120, 160
    sem = np.zeros((h, w), np.uint8)
    depth = np.full((h, w), 4.0, np.float32)
    sem[40:80, 60:100] = CLASS_IDS["fridge"]
    depth[40:80, 60:100] = 2.0
    intr = CameraIntrinsics.from_hfov(w, h, 73.0)
    sg = SceneGraph()
    t, r = np.array([0.0, 0.0, 1.0]), optical_rotation(0.0)
    dets = SemanticDetector(CLEAN).detect(sem, depth)
    for k in range(4):
        sg.add_detections(dets, depth, intr, t, r, now=float(k))
    (o,) = sg.objects()
    assert o.label == "fridge" and abs(o.position[0] - 2.0) < 0.1 and abs(o.position[1]) < 0.1


def test_detections_json_normalises_boxes():
    d = Detection("fridge", 0.9, (10.0, 20.0, 29.0, 59.0))
    out = detections_json([d, Detection("x", 0.5, None, None)], 80, 60, 12.5)
    (b,) = out["boxes"]
    assert b["box"] == [0.125, 0.3333, 0.25, 0.6667] and out["stamp"] == 12.5


def test_query_answers_without_rooms_and_unknown_query():
    sg = SceneGraph()
    a = answer_query(sg, {"id": "q1", "query": "goal_for_type", "room_type": "kitchen"}, (0.0, 0.0))
    assert a["id"] == "q1" and a["ok"] is False and "kitchen" in a["reason"]
    assert answer_query(sg, {"id": "q2", "query": "nope"}, None)["ok"] is False
    assert answer_query(sg, {"id": "q3", "query": "find", "label": "fridge"}, None)["ok"] is False
