"""Compact extract of the newest PX4 SITL ulog for estimator diagnosis (run in the sim container).

Prints one JSON object: PX4's position estimate against the simulator's own truth
(vehicle_local_position_groundtruth, referee-side diagnosis only), EKF horizontal EV innovations,
test ratio, reset counters and reset / fusion events. Times are PX4 time in seconds;
`wall_hint` (file modification time) allows matching with wall-clock logs.
"""

import glob
import json
import os
import sys

import numpy as np
from pyulog import ULog

LOGS = "/opt/PX4-Autopilot/build/px4_sitl_default/rootfs/log/*/*.ulg"
HZ = 20.0


def topic(u, name):
    for d in u.data_list:
        if d.name == name:
            return d.data
    return None


def thin(t, *cols, hz=HZ):
    keep = np.concatenate([[True], np.diff(np.floor(t * hz)) > 0])
    return [np.round(np.asarray(c, float)[keep], 3).tolist() for c in (t, *cols)]


def main() -> None:
    path = sorted(glob.glob(LOGS), key=os.path.getmtime)[-1]
    u = ULog(path)
    out: dict = {"path": path, "wall_hint": os.path.getmtime(path)}
    lp = topic(u, "vehicle_local_position")
    gt = topic(u, "vehicle_local_position_groundtruth")
    if lp is not None and gt is not None:
        t = lp["timestamp"] / 1e6
        tg = gt["timestamp"] / 1e6
        # the truth is in the simulator's frame; the estimator's frame starts at the spawn pose
        gx = np.interp(t, tg, gt["x"] - gt["x"][0])
        gy = np.interp(t, tg, gt["y"] - gt["y"][0])
        err = np.hypot(lp["x"] - gx, lp["y"] - gy)
        names = ["t", "x", "y", "z", "gx", "gy", "err", "xy_reset", "xy_valid"]
        cols = thin(
            t, lp["x"], lp["y"], lp["z"], gx, gy, err, lp["xy_reset_counter"], lp["xy_valid"]
        )
        out["pos"] = dict(zip(names, cols, strict=True))
    es = topic(u, "estimator_status")
    if es is not None:
        t = es["timestamp"] / 1e6
        names = ["t", "pos_test_ratio", "hdg_test_ratio", "time_slip", "reset_pos_ne"]
        cols = thin(
            t, es["pos_test_ratio"], es["hdg_test_ratio"], es["time_slip"], es["reset_count_pos_ne"]
        )
        out["status"] = dict(zip(names, cols, strict=True))
    inn = topic(u, "estimator_innovations")
    if inn is not None:
        t = inn["timestamp"] / 1e6
        cols = thin(t, inn["ev_hpos[0]"], inn["ev_hpos[1]"], inn["flow[0]"], inn["flow[1]"])
        out["innov"] = dict(zip(["t", "ev0", "ev1", "flow0", "flow1"], cols, strict=True))
    ts = topic(u, "timesync_status")
    if ts is not None:
        t = ts["timestamp"] / 1e6
        base = float(ts["estimated_offset"][0])
        names = ["t", "est_off_s", "obs_off_s", "rtt_ms"]
        cols = thin(
            t,
            (ts["estimated_offset"] - base) / 1e6,
            (ts["observed_offset"] - base) / 1e6,
            ts["round_trip_time"] / 1e3,
            hz=2.0,
        )
        out["timesync"] = dict(zip(names, cols, strict=True))
    aid = topic(u, "estimator_aid_src_ev_pos")
    if aid is not None:
        t = aid["timestamp"] / 1e6
        names = ["t", "fused", "rejected", "inn0", "inn1", "test_ratio"]
        cols = thin(
            t,
            aid["fused"],
            aid["innovation_rejected"],
            aid["innovation[0]"],
            aid["innovation[1]"],
            aid["test_ratio[0]"],
        )
        out["ev_aid"] = dict(zip(names, cols, strict=True))
    vo = topic(u, "vehicle_visual_odometry")
    if vo is not None:
        t = vo["timestamp"] / 1e6
        lag = (vo["timestamp"].astype(np.int64) - vo["timestamp_sample"].astype(np.int64)) / 1e3
        cols = thin(t, lag, hz=2.0)
        out["vo_lag_ms"] = dict(zip(["t", "lag_ms"], cols, strict=True))
    ev = topic(u, "estimator_event_flags")
    if ev is not None:
        names = [k for k in ev if k.startswith(("reset_", "starting_"))]
        evs = []
        for i, ts in enumerate(ev["timestamp"] / 1e6):
            for k in names:
                if ev[k][i] and (i == 0 or not ev[k][i - 1]):
                    evs.append([round(float(ts), 3), k])
        out["events"] = evs
    fl = topic(u, "estimator_status_flags")
    if fl is not None:
        t = fl["timestamp"] / 1e6
        out["cs_ev_pos_changes"] = [
            [round(float(t[i]), 3), int(fl["cs_ev_pos"][i])]
            for i in range(1, len(t))
            if fl["cs_ev_pos"][i] != fl["cs_ev_pos"][i - 1]
        ]
    sys.stdout.write(json.dumps(out))


if __name__ == "__main__":
    main()
