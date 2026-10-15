"""Camera driver ("the camera hardware"): Gazebo sensor topics -> TCP stream to the bridge.

Runs in the sim container. Subscribes (gz-transport, world network only) to the RGB, depth and
semantic topics of the drone, pairs them by sim time, downsamples semantic to 160x120, encodes
RGB (JPEG), depth (16-bit PNG, lossless) and semantic (PNG) and sends one frame per set over a
length-prefixed TCP connection (maverick_bridge.camera_protocol) to the bridge container on the
`drone` network.

  python3 -m camera_driver.driver [--host 10.231.1.3] [--port 5600] [--dump DIR]

--host - (or CAMERA_BRIDGE_ADDR=-) means no TCP: frames are only counted (and dumped as PNG with
--dump DIR), for standalone tests without a bridge.
"""

from __future__ import annotations

import argparse
import io
import os
import queue
import socket
import sys
import threading
import time
from pathlib import Path

import numpy as np
from PIL import Image as PILImage

# The wire format lives in the bridge package (pure Python); the repo is bind-mounted at /maverick.
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "ros2_ws/src/maverick_bridge"))

from maverick_bridge import camera_protocol as proto  # noqa: E402
from maverick_bridge.image_codec import encode_depth_png, encode_semantic_png  # noqa: E402

from camera_driver.processing import (  # noqa: E402
    Pairer,
    depth_to_mm,
    downsample_semantic,
)

_X500 = os.environ.get("MAVERICK_DRONE", "small").lower() == "x500"
_MODEL_DIR = "maverick" if _X500 else "maverick_small"
DEFAULT_MOUNT = Path(__file__).resolve().parents[3] / f"sim/models/{_MODEL_DIR}/camera_mount.json"
JPEG_QUALITY = 80


def decode_gz_image(msg) -> tuple[int, np.ndarray]:
    """gz.msgs.Image -> (sim time ns, array). RGB: HxWx3 u8, depth: HxW f32, labels: HxW int."""
    t_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nsec
    w, h = msg.width, msg.height
    fmt = msg.pixel_format_type
    data = msg.data
    if fmt == 13:  # R_FLOAT32
        arr = np.frombuffer(data, dtype="<f4").reshape(h, w)
    elif fmt == 3:  # RGB_INT8
        arr = np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3)
    elif fmt == 4:  # RGBA_INT8
        arr = np.frombuffer(data, dtype=np.uint8).reshape(h, w, 4)[:, :, :3]
    elif fmt == 1:  # L_INT8
        arr = np.frombuffer(data, dtype=np.uint8).reshape(h, w)
    elif fmt == 2:  # L_INT16
        arr = np.frombuffer(data, dtype="<u2").reshape(h, w)
    else:
        raise ValueError(f"unsupported pixel format {fmt}")
    return t_ns, arr


def semantic_labels(arr: np.ndarray) -> np.ndarray:
    """Label image as a 2D integer array. Gazebo's semantic map is L_INT16 (one id per pixel);
    an RGB-coded map carries the id in the first channel."""
    return arr if arr.ndim == 2 else arr[:, :, 0]


class Streamer:
    """Worker: downsample, encode, send. Drops frames while not connected (no buffering)."""

    def __init__(self, host: str | None, port: int, mount: dict, dump: Path | None) -> None:
        self.host, self.port, self.dump = host, port, dump
        self.near, self.far = mount["clip"]
        self.sem_w, self.sem_h = mount["semantic_size"]
        self.q: queue.Queue = queue.Queue(maxsize=2)
        self.seq = 0
        self.sock: socket.socket | None = None
        self.sent = self.dropped = self.bytes_sent = 0
        self.last_connect_try = 0.0
        self.t_first: float | None = None

    def offer(self, t_ns: int, parts: dict, t_wall: float) -> None:
        try:
            self.q.put_nowait((t_ns, parts, t_wall))
        except queue.Full:  # worker is behind: replace the oldest frame
            try:
                self.q.get_nowait()
            except queue.Empty:
                pass
            self.dropped += 1
            self.q.put_nowait((t_ns, parts, t_wall))

    def _connect(self) -> None:
        now = time.monotonic()
        if self.sock or self.host is None or now - self.last_connect_try < 1.0:
            return
        self.last_connect_try = now
        try:
            s = socket.create_connection((self.host, self.port), timeout=2.0)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            s.settimeout(5.0)
            self.sock = s
            print(f"[camera] connected to {self.host}:{self.port}", flush=True)
        except OSError:
            pass

    def build(self, t_ns: int, parts: dict, t_wall: float) -> proto.CameraFrame:
        rgb = parts["rgb"]
        buf = io.BytesIO()
        PILImage.fromarray(np.ascontiguousarray(rgb)).save(buf, "JPEG", quality=JPEG_QUALITY)
        depth_mm = depth_to_mm(parts["depth"], self.near, self.far)
        labels = semantic_labels(parts["semantic"])
        sem_small = downsample_semantic(labels, labels.shape[1] // self.sem_w)
        self.seq += 1
        return proto.CameraFrame(
            seq=self.seq,
            sim_ns=t_ns,
            t_wall=t_wall,
            rgb_size=(rgb.shape[1], rgb.shape[0]),
            depth_size=(depth_mm.shape[1], depth_mm.shape[0]),
            sem_size=(sem_small.shape[1], sem_small.shape[0]),
            jpeg=buf.getvalue(),
            depth_png=encode_depth_png(depth_mm),
            semantic_png=encode_semantic_png(sem_small),
        )

    def run(self) -> None:
        while True:
            t_ns, parts, t_wall = self.q.get()
            frame = self.build(t_ns, parts, t_wall)
            if self.dump is not None and self.seq % 5 == 1:
                dump_frame(self.dump, frame, parts)
            self._connect()
            if self.sock is None:
                self.dropped += 1
                continue
            try:
                data = proto.encode(frame)
                self.sock.sendall(data)
                self.sent += 1
                self.bytes_sent += len(data)
            except OSError as e:
                print(f"[camera] send failed ({e}); reconnecting", flush=True)
                self.sock.close()
                self.sock = None
                self.dropped += 1


def dump_frame(out: Path, frame: proto.CameraFrame, parts: dict) -> None:
    from maverick_bridge.image_codec import decode_png

    out.mkdir(parents=True, exist_ok=True)
    stem = f"{frame.seq:04d}"
    (out / f"{stem}_rgb.jpg").write_bytes(frame.jpeg)
    depth = decode_png(frame.depth_png, np.uint16)
    PILImage.fromarray(depth).save(out / f"{stem}_depth16.png")
    sem = decode_png(frame.semantic_png, np.uint8)
    PILImage.fromarray(sem).save(out / f"{stem}_semantic_ids.png")
    # colour versions for eyeballing
    scale = np.clip(depth.astype(np.float32) / 8000.0, 0, 1)
    PILImage.fromarray((255 * (1 - scale)).astype(np.uint8)).save(out / f"{stem}_depth_vis.png")
    pal = np.random.default_rng(1).integers(40, 255, size=(256, 3), dtype=np.uint8)
    pal[0] = 0
    PILImage.fromarray(pal[sem]).resize((320, 240), PILImage.NEAREST).save(
        out / f"{stem}_semantic_vis.png"
    )
    print(f"[camera] dumped {stem}: classes {sorted(set(sem.flatten().tolist()))}", flush=True)


def discover_topic(node, suffix: str) -> str:
    topics = node.topic_list()
    for t in topics:
        if t.endswith(suffix):
            return t
    return ""


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=os.environ.get("CAMERA_BRIDGE_ADDR", "10.231.1.3"))
    ap.add_argument("--port", type=int, default=proto.DEFAULT_PORT)
    ap.add_argument(
        "--model",
        default=os.environ.get("CAMERA_MODEL", "maverick_0" if _X500 else "maverick_small_0"),
    )
    ap.add_argument("--mount", type=Path, default=DEFAULT_MOUNT)
    ap.add_argument("--dump", type=Path, default=None)
    ap.add_argument("--stats-every", type=float, default=10.0)
    args = ap.parse_args(argv)

    import json

    from gz.msgs10.image_pb2 import Image as GzImage
    from gz.transport13 import Node

    mount = json.loads(args.mount.read_text())
    host = None if args.host == "-" else args.host
    streamer = Streamer(host, args.port, mount, args.dump)
    pairer = Pairer()
    lock = threading.Lock()
    counts = {"rgb": 0, "depth": 0, "semantic": 0, "paired": 0, "rejected": 0}

    def make_cb(stream: str):
        def cb(msg) -> None:
            try:
                t_ns, arr = decode_gz_image(msg)
            except ValueError as e:
                counts["rejected"] += 1
                print(f"[camera] {stream}: {e}", flush=True)
                return
            with lock:
                counts[stream] += 1
                got = pairer.add(stream, t_ns, arr)
                if got:
                    counts["paired"] += 1
            if got:
                streamer.offer(got[0], got[1], time.time())

        return cb

    node = Node()
    topics = {
        "rgb": "maverick/camera/rgb",
        "depth": "maverick/camera/depth",
        "semantic": "maverick/camera/semantic/labels_map",
    }
    # Wait for the model's sensors to appear (world loads, model spawns after PX4 starts).
    print(f"[camera] waiting for {topics}", flush=True)
    while True:
        have = set(node.topic_list())
        if all(any(t == "/" + n or t.endswith("/" + n) for t in have) for n in topics.values()):
            break
        time.sleep(1.0)
    real = {}
    for s, n in topics.items():
        real[s] = next(t for t in node.topic_list() if t == "/" + n or t.endswith("/" + n))
        if not node.subscribe(GzImage, real[s], make_cb(s)):
            print(f"[camera] subscribe to {real[s]} failed", file=sys.stderr)
            return 1
    print(f"[camera] subscribed: {real}", flush=True)
    threading.Thread(target=streamer.run, daemon=True).start()

    t0 = time.monotonic()
    last = (t0, 0, 0)
    try:
        while True:
            time.sleep(args.stats_every)
            now = time.monotonic()
            sent, nbytes = streamer.sent, streamer.bytes_sent
            dt = now - last[0]
            print(
                f"[camera] {counts} sent={sent} dropped={streamer.dropped} "
                f"{(sent - last[1]) / dt:.2f} fps {(nbytes - last[2]) / dt / 1e3:.0f} kB/s",
                flush=True,
            )
            last = (now, sent, nbytes)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
