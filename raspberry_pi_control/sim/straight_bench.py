# -*- coding: utf-8 -*-
"""Deterministic bench for the on-board straight controller (turn_logic.h,
StraightCtrl, loaded from libturn.so): heading-hold gains, cruise speed,
integral carry-over (V2.4) and current-based stall detection (V2.4).

No threads: the firmware loop is stepped at ~5 ms with jitter. The plant is a
skid-steer body with per-side motor dead zones (left/right differ, which is
what makes low speeds drift), yaw stiction that only partly eases while
rolling, and optional surface patches that push the nose sideways.

Battery-current model (100 kHz PWM -> continuous motor current, so the
battery sees duty * motor current): board ~60 mA + per side
|d| * I_motor, with I_motor = rolling load while moving and |d| * Vb / R when
the wheels are blocked. Carpet raises the rolling load. All motor numbers
are guesses; the real thresholds must be checked on the car."""
import ctypes
import math
import os
import random
import statistics as st
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
L = ctypes.CDLL(os.path.join(HERE, "libturn.so"))
vp = ctypes.c_void_p
for name, res, args in [
    ("gh_new", vp, []), ("gh_update", None, [vp, ctypes.c_float, ctypes.c_float]),
    ("gh_heading", ctypes.c_double, [vp]), ("gh_rate", ctypes.c_float, [vp]),
    ("gh_calib", None, [vp, ctypes.c_float, ctypes.c_int]),
    ("gh_calibrating", ctypes.c_int, [vp]),
    ("sc_new", vp, []),
    ("sc_start", None, [vp, ctypes.c_double, ctypes.c_double, ctypes.c_float,
                        ctypes.c_float, ctypes.c_float]),
    ("sc_step", None, [vp, ctypes.c_double, ctypes.c_double, ctypes.c_float]),
    ("sc_state", ctypes.c_int, [vp]), ("sc_l", ctypes.c_float, [vp]), ("sc_r", ctypes.c_float, [vp]),
    ("sc_maxerr", ctypes.c_float, [vp]), ("sc_meanerr", ctypes.c_float, [vp]),
    ("sc_set", None, [vp] + [ctypes.c_float] * 5),
    ("sc_feed_current", None, [vp, ctypes.c_double, ctypes.c_float]),
    ("sc_base", ctypes.c_float, [vp]), ("sc_peak", ctypes.c_float, [vp]),
    ("sc_imem", ctypes.c_float, [vp, ctypes.c_int]),
    ("sc_set_v24", None, [vp, ctypes.c_int, ctypes.c_int] + [ctypes.c_float] * 4),
]:
    f = getattr(L, name)
    f.restype, f.argtypes = res, args

SPEED_GAIN = 1.11


class Car:
    def __init__(self, seed, imbalance=(0.90, 0.97), ease=(0.90, 0.97), ubias=(0.02, 0.035)):
        r = self.r = random.Random(seed)
        self.d0L = r.uniform(0.04, 0.08)
        self.d0R = self.d0L + r.uniform(-0.025, 0.025)     # sides differ
        self.kR = r.uniform(*imbalance)                     # right side weaker
        self.G = r.uniform(380, 600)
        self.tau = r.uniform(0.08, 0.25)
        self.u_kin0 = r.uniform(0.14, 0.18)
        self.u_static0 = self.u_kin0 + r.uniform(0.01, 0.05)
        self.ease = r.uniform(*ease)
        self.g_bias = r.uniform(-0.3, 0.3)
        # constant yaw torque (wheel misalignment / floor): fitted so that the
        # v2.3 gains reproduce the real car (0.6 m legs at 0.18 m/s: mean |err|
        # 1.1-1.3 deg, max 1.6-2.0 deg)
        self.u_bias = r.choice((-1, 1)) * r.uniform(*ubias)
        # electrical (per side = 2 motors in parallel)
        self.R = r.uniform(3.0, 6.0)
        self.iload = r.uniform(0.15, 0.30)
        self.vb = 11.2
        self.x = self.y = self.th = self.w = 0.0
        self.stuck = True
        self.blocked = False
        self.load_mult = 1.0
        self.patch_u = 0.0

    def eff(self, d, d0, k=1.0):
        return math.copysign(max(0.0, abs(d) - d0) * k / (1.0 - d0), d)

    def plant(self, ul, ur, dt):
        eL, eR = self.eff(ul, self.d0L), self.eff(ur, self.d0R, self.kR)
        v = 0.0 if self.blocked else SPEED_GAIN * 0.5 * (eL + eR)
        u = 0.5 * (eR - eL) + self.patch_u + self.u_bias
        moving = min(1.0, abs(v) / 0.05)   # rolling eases yaw quickly
        u_kin = self.u_kin0 * (1.0 - self.ease * moving)
        u_static = self.u_static0 * (1.0 - self.ease * moving)
        fr = self.G * u_kin / self.tau
        if self.blocked:
            u_static = self.u_static0 * 2       # nose against a wall: hard to yaw
        if self.stuck and abs(u) >= u_static:
            self.stuck = False
        if not self.stuck:
            dw = (self.G * u - self.w) / self.tau
            sgn = (self.w > 0) - (self.w < 0) or ((u > 0) - (u < 0))
            dw -= fr * sgn
            w_new = self.w + dw * dt
            if self.w != 0.0 and w_new * self.w <= 0 and abs(self.G * u) < fr * self.tau:
                w_new, self.stuck = 0.0, True
            self.th += math.radians(0.5 * (self.w + w_new) * dt)
            self.w = w_new
        self.x += v * math.cos(self.th) * dt
        self.y += v * math.sin(self.th) * dt
        self.v = v
        return ul, ur

    def current_ma(self, ul, ur):
        tot = 60.0
        for d, d0 in ((ul, self.d0L), (ur, self.d0R)):
            d = abs(d)
            stall = d * self.vb / self.R
            if self.blocked or d <= d0:
                im = stall
            else:
                im = min(stall, self.iload * self.load_mult)
            tot += d * im * 1000.0
        return tot + self.r.gauss(0, 12.0)


def straight(car, gh, sc, now, dist, speed, events=(), patch=None):
    """One on-board straight segment, like the PC does it. Returns stats."""
    pwm = speed / SPEED_GAIN
    ramp, r0 = 0.5, 0.4
    dur = dist / (pwm * SPEED_GAIN) + ramp * (1 - r0) / 2
    hd = L.gh_heading(gh)
    L.sc_start(sc, now, hd, pwm, dur, ramp)
    x0, y0, th0 = car.x, car.y, car.th
    ul = ur = 0.0
    t_end = now + dur + 0.5
    last_cur = -1.0
    t_start = now
    while now < t_end:
        dt = 0.005 + car.r.uniform(-0.001, 0.003)
        now += dt
        el = now - t_start
        for t_ev, kind, val in events:
            if el >= t_ev:
                setattr(car, kind, val)
        if patch and patch[0] <= el < patch[0] + patch[1]:
            car.patch_u = patch[2]
        else:
            car.patch_u = 0.0
        car.plant(ul, ur, dt)
        gz = car.w + car.g_bias + car.r.gauss(0, 0.5)
        L.gh_update(gh, round(gz * 16) / 16, dt)
        if L.sc_state(sc) == 1:
            if now - last_cur >= 0.025:
                last_cur = now
                L.sc_feed_current(sc, now, car.current_ma(ul, ur))
            L.sc_step(sc, now, L.gh_heading(gh), L.gh_rate(gh))
            ul, ur = L.sc_l(sc), L.sc_r(sc)
        else:
            ul = ur = 0.0
    # cross-track offset relative to the commanded line (start pose + target heading)
    dx, dy = car.x - x0, car.y - y0
    hd_rad = th0  # body heading at start == target heading (heading held from start)
    xt = -dx * math.sin(hd_rad) + dy * math.cos(hd_rad)
    return {"state": L.sc_state(sc), "mean": L.sc_meanerr(sc), "max": L.sc_maxerr(sc),
            "xt": xt, "base": L.sc_base(sc), "peak": L.sc_peak(sc), "now": now,
            "head_end": math.degrees(car.th - th0)}


def new_fw(kp, ki, keep, stall_on=1):
    gh, sc = L.gh_new(), L.sc_new()
    L.sc_set(sc, kp, ki, -1, -1, -1)
    L.sc_set_v24(sc, keep, stall_on, -1, -1, -1, -1)
    return gh, sc


def calib(car, gh, now):
    L.gh_calib(gh, 1.0, 1)
    while L.gh_calibrating(gh):
        now += 0.005
        L.gh_update(gh, round((car.g_bias + car.r.gauss(0, 0.5)) * 16) / 16, 0.005)
    return now


def run_lines(seed, kp, ki, keep, speed, dist=0.6, n=4, patch_seg=None):
    """n consecutive straight segments (the turns are irrelevant here)."""
    car = Car(seed)
    gh, sc = new_fw(kp, ki, keep)
    now = calib(car, gh, 0.0)
    out = []
    for i in range(n):
        patch = None
        if patch_seg == i:
            patch = (1.0, 0.8, car.r.choice((-1, 1)) * 0.05)
        s = straight(car, gh, sc, now, dist, speed, patch=patch)
        now = s["now"] + 0.3
        out.append(s)
    return out


def bench_gains():
    print("== heading hold: 0.6 m segments x4 (one with a 0.8 s sideways push), 40 cars")
    print("   cfg                         | mean err | max err | |cross-track| per seg (mean / p90)")
    cfgs = [("v2.3 kp.010 ki.006 noKeep", 0.010, 0.006, 0),
            ("kp.020 ki.012 noKeep", 0.020, 0.012, 0),
            ("kp.020 ki.012 keep (v2.4)", 0.020, 0.012, 1),
            ("kp.030 ki.018 keep", 0.030, 0.018, 1)]
    for speed in (0.18, 0.25, 0.30):
        print(f"-- speed {speed} m/s")
        for name, kp, ki, keep in cfgs:
            mean_e, max_e, xts = [], [], []
            for seed in range(40):
                segs = run_lines(seed, kp, ki, keep, speed, patch_seg=2)
                for s in segs:
                    mean_e.append(s["mean"]); max_e.append(s["max"]); xts.append(abs(s["xt"]) * 100)
            xts.sort()
            print(f"   {name:28s}| {st.mean(mean_e):5.2f}°  | {st.mean(max_e):5.2f}° (p90 "
                  f"{sorted(max_e)[int(.9 * len(max_e))]:4.1f}°) | {st.mean(xts):4.2f} / "
                  f"{xts[int(.9 * len(xts))]:4.2f} cm")


def bench_stall():
    print("== stall detection (V2.4 defaults), 0.25 m/s, 1.0 m segment, 200 cars each")
    cases = [
        ("normal run", []),
        ("hits a wall at 1.5 s", [(1.5, "blocked", True)]),
        ("blocked from the start", [(0.0, "blocked", True)]),
        ("onto carpet x1.6 at 1.5 s", [(1.5, "load_mult", 1.6)]),
        ("onto carpet x2.0 at 1.5 s", [(1.5, "load_mult", 2.0)]),
    ]
    for speed in (0.18, 0.25, 0.30):
        print(f"-- speed {speed} m/s")
        for name, ev in cases:
            hits, ratios = 0, []
            for seed in range(200):
                car = Car(1000 + seed)
                gh, sc = new_fw(0.02, 0.012, 1)
                now = calib(car, gh, 0.0)
                s = straight(car, gh, sc, now, 1.0, speed, events=ev)
                hits += s["state"] == 5
                if s["base"] > 0:
                    ratios.append(s["peak"] / s["base"])
            r = f"peak/base median {st.median(ratios):.2f}" if ratios else "no baseline"
            print(f"   {name:28s} -> stall flagged {hits:3d}/200   ({r})")


if __name__ == "__main__":
    which = sys.argv[1] if len(sys.argv) > 1 else "all"
    if which in ("all", "gains"):
        bench_gains()
    if which in ("all", "stall"):
        bench_stall()
