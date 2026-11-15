"""The memory node without ROS: `memory_node.MemoryNode._process/_update_rooms/_on_query`.

Real `SemanticDetector` (detector noise model), `lift_detection`, `SceneGraph` and `answer_query`;
only the message plumbing is replaced by direct calls.
"""

from __future__ import annotations

import numpy as np
from maverick_mapping.voxel_map import CameraIntrinsics
from maverick_memory.lifting import lift_detection
from maverick_memory.queries import answer_query, detections_json
from maverick_memory.scene_graph import SceneGraph, SceneGraphConfig
from maverick_memory.semantic_detector import SemanticDetector
from maverick_memory.sim_detector import DetectorNoise


class MemoryShim:
    def __init__(self, drone, seed: int, run: int, runner) -> None:
        # memory_node's own seed parameter is the launch `seed` (0 by default); the noise stream
        # per run is whatever the wall clock gave Gazebo; here it is deterministic per (seed, run)
        self.detector = SemanticDetector(
            DetectorNoise(miss_rate=0.10, fp_rate=0.02, box_jitter=0.05, mask_dropout=0.05),
            seed=int(np.random.SeedSequence([seed, run, 11]).generate_state(1)[0]),
        )
        self.sg = SceneGraph(
            SceneGraphConfig(
                robot_clear=drone.goal_clear,
                connect_clear=drone.connect_clear,
                wall_clear=drone.wall_clear,
            )
        )
        self.runner = runner
        self.robot_xy: tuple[float, float] | None = None
        self._grid = None
        self._grid_done = None
        self._intr: CameraIntrinsics | None = None

    def on_grid(self, grid) -> None:
        self._grid = grid

    def process(self, sem_depth, cam_t, cam_r, stamp: float) -> None:
        """memory_node._process on the 160x120 semantic image and the matching depth."""
        sem, depth = sem_depth
        intr = self._intr  # the node's CameraInfo scaled to the 160x120 image
        t_wc, r_wc = np.asarray(cam_t, float), np.asarray(cam_r, float)
        self.robot_xy = (float(t_wc[0]), float(t_wc[1]))
        dets = self.detector.detect(sem, depth)
        obs = [lift_detection(d, depth, intr, t_wc, r_wc, t=stamp) for d in dets]
        self.sg.add_observations([o for o in obs if o is not None], stamp)
        pos = [None if o is None else o.center for o in obs]
        payload = detections_json(dets, depth.shape[1], depth.shape[0], stamp, pos)
        self.runner.on_detections(payload)

    def set_intrinsics(self, intr_full: CameraIntrinsics) -> None:
        self._intr = intr_full.scaled(0.5)

    def update_rooms(self, now: float) -> None:
        """memory_node._update_rooms (1 Hz)."""
        if self._grid is not None and self._grid is not self._grid_done:
            grid, res, org = self._grid
            self._grid_done = self._grid
            if grid.size:
                self.sg.update_rooms(grid, res, org)
        else:
            self.sg.retype()
        if self.robot_xy is not None:
            self.sg.visit(self.robot_xy)
        self.runner.on_rooms(self.sg.gateway_rooms())
        self.runner.on_objects(self.sg.gateway_objects())

    def answer(self, q: dict) -> dict:
        try:
            return answer_query(self.sg, q, self.robot_xy)
        except (ValueError, TypeError, KeyError) as exc:
            return {"id": None, "ok": False, "reason": f"bad query: {exc}"}
