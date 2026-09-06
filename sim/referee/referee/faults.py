"""Fault injection through the Docker CLI (runs on the host machine)."""

from __future__ import annotations

import json
import threading

from referee.stack import Stack

NETEM = "/maverick/docker/netem/apply.sh"


def _window_script(profile: str, duration_s: float) -> str:
    # One shell does apply -> sleep -> clear, so short outages are not stretched by docker
    # exec latency. Timestamps are taken in the container (same kernel clock as the host).
    return (
        f"bash {NETEM} {profile} >/dev/null && t0=$(date +%s.%N) && sleep {duration_s:.3f} && "
        f"bash {NETEM} clear >/dev/null && t1=$(date +%s.%N) && "
        'echo "{\\"t_start\\": $t0, \\"t_end\\": $t1}"'
    )


def netem_window(stack: Stack, profile: str, duration_s: float) -> dict:
    """Impair both ends of the wifi link for duration_s. Returns {t_start, t_end} (wall clock)."""
    out: dict[str, dict] = {}
    errors: list[str] = []

    def one(service: str) -> None:
        r = stack.exec(service, _window_script(profile, duration_s), timeout=duration_s + 60)
        try:
            out[service] = json.loads(r.stdout.strip().splitlines()[-1])
        except (ValueError, IndexError):
            errors.append(f"{service}: rc={r.returncode} {r.stdout[-200:]} {r.stderr[-200:]}")

    threads = [threading.Thread(target=one, args=(s,)) for s in ("bridge", "host")]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    if errors:
        raise RuntimeError("netem failed: " + "; ".join(errors))
    return {
        "t_start": min(v["t_start"] for v in out.values()),
        "t_end": max(v["t_end"] for v in out.values()),
    }


def netem_clear(stack: Stack) -> bool:
    ok = True
    for service in ("bridge", "host"):
        r = stack.exec(service, f"bash {NETEM} clear", timeout=30)
        ok = ok and r.returncode == 0
    return ok


def kill_process(stack: Stack, service: str, pattern: str) -> float:
    """SIGKILL processes whose command line matches `pattern` in `service`; returns wall time.

    `pattern` must be written like "[f]light_client" so that the shell running pkill does not
    match itself.
    """
    r = stack.exec(service, f"pkill -9 -f '{pattern}'; date +%s.%N", timeout=30)
    return float(r.stdout.strip().splitlines()[-1])
