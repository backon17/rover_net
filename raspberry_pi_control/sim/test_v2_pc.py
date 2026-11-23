# -*- coding: utf-8 -*-
"""End-to-end test: unmodified wave_rover_traj.TrajController driving the fake
V2 rover (real firmware turn logic + plant). Checks L-shape and square paths.

Run: python3 sim/test_v2_pc.py
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)
import wave_rover_traj as w  # noqa: E402
from fake_v2 import FakeV2Rover  # noqa: E402


def run_path(name, wps, seed, speed=0.3):
    rover = FakeV2Rover(seed=seed)
    calib = dict(w.DEFAULT_CALIB)
    calib["speed_gain"] = rover.speed_gain
    logs = []
    ctrl = w.TrajController(rover, calib, log_fn=logs.append)
    assert ctrl.fw, "V2 firmware not detected"
    ctrl.run_waypoints(wps, speed_ms=speed)
    x, y, th = rover.true_pose()
    rover.close()
    ex, ey = wps[-1]
    # expected final heading = bearing of the last segment
    px, py = wps[-2] if len(wps) > 1 else (0.0, 0.0)
    exp_th = math.degrees(math.atan2(ey - py, ex - px))
    pos_err = math.hypot(x - ex, y - ey)
    th_err = w.wrap_deg(th - exp_th)
    turns = [l for l in logs if "[转向]" in l]
    print(f"{name:8s} seed={seed}: end=({x:+.3f},{y:+.3f}) heading={th:+6.1f}° "
          f"| pos err={pos_err * 100:5.1f} cm, heading err={th_err:+5.1f}°")
    for t in turns:
        print("        " + t.strip())
    return pos_err, th_err


def main():
    paths = {
        "L-shape": [(0, 0), (1.0, 0.0), (1.0, 1.0)],
        "L-right": [(0, 0), (0.5, 0.0), (0.5, -0.3)],
        "square": [(0, 0), (1, 0), (1, 1), (0, 1), (0, 0)],
    }
    worst_pos, worst_th = 0.0, 0.0
    for name, wps in paths.items():
        for seed in (1, 2):
            pe, te = run_path(name, wps, seed)
            worst_pos = max(worst_pos, pe)
            worst_th = max(worst_th, abs(te))
    ok = worst_pos < 0.10 and worst_th < 4.0
    print(f"\nworst position error {worst_pos * 100:.1f} cm, worst heading error {worst_th:.1f}°"
          f" -> {'PASS' if ok else 'FAIL'} (limits 10 cm / 4°)")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
