"""Drive the Docker Compose stack and the processes in its containers (runs on the host machine).

The runner lives on the host machine, not in the referee container, so restarting the whole
stack (which includes the referee container) never kills the runner. Ground truth is read
inside the referee container (it is the only one on the `world` network) via `docker exec`.
"""

from __future__ import annotations

import json
import os
import subprocess
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
COMPOSE_FILE = ROOT / "docker" / "compose.yaml"
READY_TIMEOUT_S = 150


class Stream:
    """A long-running `docker exec` whose stdout is JSON lines, collected with a reader thread."""

    def __init__(self, name: str, argv: list[str]) -> None:
        self.name = name
        self.rows: list[dict] = []
        self.garbage: list[str] = []
        self._proc = subprocess.Popen(
            argv, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1
        )
        self._thread = threading.Thread(target=self._read, daemon=True)
        self._thread.start()

    def _read(self) -> None:
        assert self._proc.stdout is not None
        for line in self._proc.stdout:
            line = line.strip()
            if not line.startswith("{"):
                if line:
                    self.garbage.append(line)
                continue
            try:
                row = json.loads(line)
            except ValueError:
                self.garbage.append(line)
                continue
            self.rows.append(row)

    def stop(self) -> None:
        if self._proc.poll() is None:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._proc.kill()
        self._thread.join(timeout=2)


class Stack:
    def __init__(self, compose_file: Path = COMPOSE_FILE) -> None:
        self.compose_file = compose_file
        self.env = dict(os.environ)
        self.env["DOCKER_GID"] = str(os.stat("/var/run/docker.sock").st_gid)
        self.env["HEADLESS"] = "1"
        self.gz_partition: str | None = None

    # -- plumbing ----------------------------------------------------------------------------
    def compose(self, *args: str, timeout: float | None = 300, check: bool = False):
        return subprocess.run(
            ["docker", "compose", "-f", str(self.compose_file), *args],
            env=self.env,
            capture_output=True,
            text=True,
            timeout=timeout,
            check=check,
        )

    def exec_argv(self, service: str, cmd: str, env: dict[str, str] | None = None) -> list[str]:
        argv = ["docker", "compose", "-f", str(self.compose_file), "exec", "-T"]
        for k, v in (env or {}).items():
            argv += ["-e", f"{k}={v}"]
        return argv + [service, "bash", "-lc", cmd]

    def exec(self, service: str, cmd: str, env=None, timeout: float | None = 60):
        return subprocess.run(
            self.exec_argv(service, cmd, env),
            env=self.env,
            capture_output=True,
            text=True,
            timeout=timeout,
        )

    def stream(self, name: str, service: str, cmd: str, env=None) -> Stream:
        return Stream(name, self.exec_argv(service, cmd, env))

    # -- lifecycle ---------------------------------------------------------------------------
    def build_workspace(self) -> None:
        r = self.compose(
            "--profile",
            "tools",
            "run",
            "--rm",
            "-T",
            "tools",
            "bash",
            "-lc",
            "cd /maverick/ros2_ws && colcon build --symlink-install",
            timeout=900,
        )
        if r.returncode != 0:
            raise RuntimeError(f"colcon build failed:\n{r.stdout[-2000:]}\n{r.stderr[-2000:]}")

    def down(self) -> None:
        self.compose("down", "--timeout", "10", timeout=180)

    def restart(self, attempts: int = 3) -> float:
        """Fresh stack, retrying a start where PX4 never became ready (seen once in testing)."""
        t0 = time.monotonic()
        for n in range(1, attempts + 1):
            try:
                self._restart_once()
                return time.monotonic() - t0
            except RuntimeError as exc:
                tail = self.compose("logs", "--tail", "15", "sim", timeout=60).stdout[-1500:]
                print(f"  stack start attempt {n} failed: {exc}\n{tail}", flush=True)
                if n == attempts:
                    raise
        raise AssertionError("unreachable")

    def _restart_once(self) -> None:
        """Fresh stack: down, up, wait until PX4 is ready and the host sees the bridge."""
        self.down()
        r = self.compose("up", "-d", timeout=300)
        if r.returncode != 0:
            raise RuntimeError(f"compose up failed: {r.stderr[-1500:]}")
        deadline = time.monotonic() + READY_TIMEOUT_S
        while True:
            if self.exec("sim", "test -f /tmp/px4_ready", timeout=30).returncode == 0:
                break
            if time.monotonic() > deadline:
                raise RuntimeError("PX4 not ready in time")
            time.sleep(2)
        while True:
            r = self.exec(
                "host", "ros2 topic list 2>/dev/null | grep -q /maverick/bridge/status", timeout=30
            )
            if r.returncode == 0:
                break
            if time.monotonic() > deadline:
                raise RuntimeError("host does not see the bridge in time")
            time.sleep(2)
        host = self.exec("sim", "hostname", timeout=30).stdout.strip()
        user = self.exec("sim", "whoami", timeout=30).stdout.strip()
        self.gz_partition = f"{host}:{user}"
