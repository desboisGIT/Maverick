"""Rate-loop check of a PX4 SITL log (run inside the sim container): angular-rate RMS and dominant
frequency while airborne, hover thrust estimate, mean motor commands.

Usage: python3 /maverick/scripts/ulog_rates.py [ulg-file]   (default: newest log)
"""

import glob
import sys

import numpy as np
from pyulog import ULog

LOGS = "/opt/PX4-Autopilot/build/px4_sitl_default/rootfs/log/*/*.ulg"


def main() -> None:
    path = sys.argv[1] if len(sys.argv) > 1 else sorted(glob.glob(LOGS))[-1]
    print(path)
    u = ULog(path)

    def topic(name):
        return next(x for x in u.data_list if x.name == name).data

    av = topic("vehicle_angular_velocity")
    t = av["timestamp"] / 1e6
    lp = topic("vehicle_local_position")
    tz = lp["timestamp"] / 1e6
    air = tz[lp["z"] < -0.5]
    t0, t1 = air.min() + 3, air.max()
    m = (t > t0) & (t < t1)
    print(f"airborne window {t1 - t0:.1f} s")
    for i, axis in enumerate("xyz"):
        w = av[f"xyz[{i}]"][m]
        w = w - np.mean(w)
        print(
            f"rate {axis} rms {w.std():.4f} rad/s ({np.degrees(w.std()):.2f} deg/s), "
            f"peak {np.abs(w).max():.3f}"
        )
        dt = np.median(np.diff(t[m]))
        win = np.hanning(len(w))
        sp = np.abs(np.fft.rfft(w * win))
        fr = np.fft.rfftfreq(len(w), dt)
        k = np.argmax(sp[3:]) + 3
        print(f"   dominant {fr[k]:.1f} Hz amp {sp[k] * 2 / win.sum():.4f}")
        hf = fr > 8.0  # above the position/attitude loops: rate-loop ringing would show here
        print(f"   8-100 Hz rms {np.sqrt(np.sum(sp[hf] ** 2) * 2) / win.sum() / np.sqrt(2):.4f}")
    try:
        h = topic("hover_thrust_estimate")["hover_thrust"]
        print("hover thrust estimate (last valid)", h[~np.isnan(h)][-1])
    except StopIteration:
        print("no hover_thrust_estimate")
    am = topic("actuator_motors")
    ta = am["timestamp"] / 1e6
    mm = (ta > t0) & (ta < t1)
    print(
        "motor cmd mean", [round(float(np.nanmean(am[f"control[{i}]"][mm])), 3) for i in range(4)]
    )


main()
