# -*- coding: utf-8 -*-
"""A/B: old PC-side straight heading hold (v2.2) vs on-board T:165 (v2.3)
under heavy-tailed WiFi latency. 1 m right-turn square on the fake rover.

Metrics per run: end position error, max cross-track deviation from the
planned square, stutter events (body speed collapses mid-segment)."""
import math, os, random, sys, threading, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE)); sys.path.insert(0, HERE)
from fake_v2 import FakeV2Rover  # noqa: E402
import wave_rover_traj as w  # noqa: E402

SQUARE = [(0, 0), (1, 0), (1, -1), (0, -1), (0, 0)]


class WifiLagRover(FakeV2Rover):
    def __init__(self, seed, onboard, lag):
        self.lag_rng = random.Random(seed * 7 + 1)
        self.lag = lag
        super().__init__(seed=seed)
        self.straight_fw = onboard

    def _request(self, obj, timeout=0.5):
        if self.lag:
            u = self.lag_rng.random()
            d = (self.lag_rng.uniform(0.03, 0.12) if u < 0.70 else
                 self.lag_rng.uniform(0.12, 0.30) if u < 0.95 else
                 self.lag_rng.uniform(0.30, 0.70))
            if d > timeout:              # the HTTP client gives up
                time.sleep(timeout)
                raise ConnectionError("timeout")
            time.sleep(d)
        return super()._request(obj, timeout)


def seg_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(px - ax - t * dx, py - ay - t * dy)


def run(seed, onboard, lag, speed=0.18):
    r = WifiLagRover(seed, onboard, lag)
    calib = dict(w.DEFAULT_CALIB)
    calib["speed_gain"] = r.speed_gain
    logs = []
    ctrl = w.TrajController(r, calib, log_fn=logs.append)
    assert ctrl.fw_straight == onboard
    samples, stop = [], threading.Event()

    def sampler():
        last = None
        while not stop.is_set():
            x, y, th = r.true_pose()
            t = time.monotonic()
            if last:
                samples.append((t, x, y, abs(getattr(r, "v_body", 0.0)), abs(r.w)))
            last = (x, y, t)
            time.sleep(0.02)
    th_ = threading.Thread(target=sampler, daemon=True)
    th_.start()
    err = ""
    try:
        ctrl.run_waypoints(SQUARE, speed_ms=speed)
    except Exception as e:
        err = str(e)
    stop.set(); th_.join()
    x, y, _ = r.true_pose()
    r.close()
    xt = max(min(seg_dist(sx, sy, *SQUARE[i], *SQUARE[i + 1]) for i in range(4))
             for _, sx, sy, _, _ in samples)
    # stutter: while translating (not pivoting), speed falls below 35% of the
    # target and comes back above 70%
    stut, state = 0, "idle"
    for _, _, _, v, wrate in samples:
        if wrate > 20:                 # pivoting: corner, not a stutter
            state = "idle"
        elif v > 0.7 * speed:
            if state == "dip":
                stut += 1
            state = "cruise"
        elif v < 0.35 * speed and state == "cruise":
            state = "dip"
    return math.hypot(x, y), xt, stut, err


def main():
    for lag in (False, True):
        for onboard in (False, True):
            res = [run(s, onboard, lag) for s in (1, 2, 3)]
            name = ("v2.3 on-board" if onboard else "v2.2 PC loop ") + (" | WiFi lag" if lag else " | low lag ")
            print(f"{name}: end err " + " ".join(f"{e*100:4.1f}" for e, *_ in res) + " cm"
                  f" | max off-line " + " ".join(f"{x*100:4.1f}" for _, x, *_ in res) + " cm"
                  f" | stutters " + " ".join(str(s) for _, _, s, _ in res)
                  + "".join(f" | ERR {e}" for *_, e in res if e))


if __name__ == "__main__":
    main()
