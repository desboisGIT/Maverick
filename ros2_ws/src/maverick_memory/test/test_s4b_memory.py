"""S4b in memory: small-object classes in the detector, search plan and find queries, photos."""

import io

import numpy as np
from maverick_mapping import CameraIntrinsics, optical_rotation
from maverick_memory.lifting import Detection, lift_detection
from maverick_memory.photos import FrameBuffer, crop_jpeg
from maverick_memory.queries import answer_query, detections_json
from maverick_memory.scene_graph import SceneGraph
from maverick_memory.semantic_detector import CLASS_IDS, SemanticDetector
from maverick_memory.sim_detector import DetectorNoise
from maverick_memory.store import canonical_label

CLEAN = DetectorNoise(miss_rate=0.0, fp_rate=0.0, box_jitter=0.0, mask_dropout=0.0)


def test_small_classes_are_detected_with_few_pixels():
    sem = np.zeros((120, 160), np.uint8)
    depth = np.full((120, 160), 3.0, np.float32)
    sem[60:62, 70:77] = CLASS_IDS["banana"]  # 14 pixels: far below the furniture minimum (32)
    sem[30:60, 100:110] = CLASS_IDS["person"]
    sem[90:92, 20:23] = CLASS_IDS["bread"]  # 6 pixels
    sem[10:12, 10:12] = CLASS_IDS["fridge"]  # 4 pixels: still dropped
    labels = sorted(d.label for d in SemanticDetector(CLEAN).detect(sem, depth))
    assert labels == ["banana", "bread", "person"]


def test_new_class_ids_are_appended_after_the_furniture():
    assert CLASS_IDS["rug"] == 26 and CLASS_IDS["banana"] == 27
    assert (CLASS_IDS["bread"], CLASS_IDS["person"]) == (28, 29)


def test_aliases():
    assert canonical_label("Bananas") == "banana" and canonical_label("loaf") == "bread"
    assert canonical_label("human") == "person"


def _scene_with_banana():
    """A box-shaped counter and a banana lifted from a synthetic depth image."""
    intr = CameraIntrinsics(fx=100.0, fy=100.0, cx=80.0, cy=60.0, width=160, height=120)
    r_wc = optical_rotation(0.0)
    t_wc = np.array([0.0, 0.0, 1.2])
    depth = np.full((120, 160), 2.0, np.float32)
    box = (74.0, 62.0, 85.0, 66.0)
    det = Detection("banana", 0.8, box, None)
    obs = lift_detection(det, depth, intr, t_wc, r_wc, t=1.0)
    return intr, t_wc, r_wc, depth, det, obs


def test_find_query_reports_unconfirmed_objects_and_photo_reference():
    _, _, _, _, _, obs = _scene_with_banana()
    sg = SceneGraph()
    sg.add_observations([obs], 1.0)  # one observation: not confirmed yet (min_obs 3)
    assert not answer_query(sg, {"id": "a", "query": "find", "label": "banana"}, None)["ok"]
    ans = answer_query(
        sg, {"id": "b", "query": "find", "label": "banana", "confirmed": False}, None
    )
    assert ans["ok"] and ans["objects"][0]["n_obs"] == 1
    o = sg.store.match(obs)
    o.extra["photo"] = "/tmp/maverick_photos/obj_1.jpg"
    ans = answer_query(
        sg, {"id": "c", "query": "find", "label": "bananas", "confirmed": False}, None
    )
    assert ans["objects"][0]["photo"].endswith("obj_1.jpg") and "last_seen" in ans["objects"][0]


def test_detections_json_carries_lifted_positions():
    dets = [Detection("banana", 0.8, (10, 10, 20, 20)), Detection("fridge", 0.7, (30, 30, 50, 60))]
    j = detections_json(dets, 160, 120, 1.0, [np.array([1.0, 2.0, 0.9]), None])
    assert j["boxes"][0]["position"] == [1.0, 2.0, 0.9] and "position" not in j["boxes"][1]
    assert "position" not in detections_json(dets, 160, 120, 1.0)["boxes"][0]


def test_search_plan_without_rooms_and_with_unknown_region():
    sg = SceneGraph()
    ans = answer_query(sg, {"id": "p", "query": "search_plan", "label": "banana"}, None)
    assert ans["ok"] is False and "room" in ans["reason"]
    ans = answer_query(
        sg, {"id": "p", "query": "search_plan", "label": "banana", "region": "kitchen"}, None
    )
    assert ans["ok"] is False and "kitchen" in ans["reason"]


def test_crop_jpeg_and_frame_buffer(tmp_path):
    from PIL import Image

    buf = io.BytesIO()
    Image.fromarray(np.full((240, 320, 3), 128, np.uint8)).save(buf, "JPEG")
    fb = FrameBuffer()
    fb.add(1.0, buf.getvalue())
    assert fb.nearest(1.1) is not None and fb.nearest(2.0) is None
    out = tmp_path / "crops" / "obj_1.jpg"
    size = crop_jpeg(buf.getvalue(), (0.4, 0.4, 0.1, 0.1), out)
    assert out.exists() and size is not None and size[0] >= 32
    assert crop_jpeg(b"not a jpeg", (0.4, 0.4, 0.1, 0.1), tmp_path / "x.jpg") is None
