# -*- coding: utf-8 -*-
"""Virtual-time bench of the firmware turn controller (turn_logic.h via
libturn.so) on the fake_v2 skid-steer plant, with cars whose dead zone sits
close to the slow-down PWM (they crawl). Compares crawl boost on/off.

Run: python3 sim/turn_crawl_bench.py
"""
import ctypes
import math
import os
import random
import statistics as stt

HERE = os.path.dirname(os.path.abspath(__file__))
L = ctypes.CDLL(os.path.join(HERE, "libturn.so"))
vp = ctypes.c_void_p
for name, res, args in [
    ("tc_new", vp, []), ("tc_start", None, [vp, ctypes.c_double, ctypes.c_double, ctypes.c_float, ctypes.c_float, ctypes.c_float]),
    ("tc_step", None, [vp, ctypes.c_double, ctypes.c_double, ctypes.c_float]),
    ("tc_l", ctypes.c_float, [vp]), ("tc_r", ctypes.c_float, [vp]), ("tc_state", ctypes.c_int, [vp]),
    ("tc_rekicks", ctypes.c_int, [vp]), ("tc_boost", ctypes.c_float, [vp]),
    ("tc_set_boost", None, [vp, ctypes.c_float, ctypes.c_float, ctypes.c_float]),
]:
    f = getattr(L, name); f.restype = res; f.argtypes = args


class Plant:
    def __init__(self, rng, u_kin):
        self.G = rng.uniform(380, 600)
        self.tau = rng.uniform(0.08, 0.25)
        self.u_kin = u_kin
        self.u_static = u_kin + rng.uniform(0.01, 0.05)
        self.w = 0.0
        self.th = 0.0
        self.stuck = True

    def step(self, u, dt):
        f = self.G * self.u_kin / self.tau
        if self.stuck and abs(u) >= self.u_static:
            self.stuck = False
        if not self.stuck:
            dw = (self.G * u - self.w) / self.tau
            sgn = (self.w > 0) - (self.w < 0) or ((u > 0) - (u < 0))
            dw -= f * sgn
            w_new = self.w + dw * dt
            if self.w != 0.0 and w_new * self.w <= 0 and abs(self.G * u) < f * self.tau:
                w_new = 0.0
                self.stuck = True
            self.th += 0.5 * (self.w + w_new) * dt
            self.w = w_new


def run(seed, u_kin, deg, boost_on, hiccup):
    rng = random.Random(seed)
    pl = Plant(rng, u_kin)
    tc = L.tc_new()
    if not boost_on:
        L.tc_set_boost(tc, -1, -1, 0.0)
    t = 0.0
    L.tc_start(tc, t, 0.0, deg, 0.25, 0.0)
    ul = ur = 0.0
    while L.tc_state(tc) == 1 and t < 30:
        dt = 0.005 + (rng.uniform(0.05, 0.3) if hiccup and rng.random() < 0.004 else 0.0)
        n = max(1, int(dt / 0.001))
        for _ in range(n):
            pl.step(0.5 * (ur - ul), dt / n)
        t += dt
        rate = pl.w + rng.gauss(0, 0.4)
        L.tc_step(tc, t, pl.th, rate)
        ul, ur = L.tc_l(tc), L.tc_r(tc)
    # let it coast to rest
    for _ in range(400):
        pl.step(0.0, 0.005)
    return t, pl.th - deg, L.tc_state(tc), L.tc_rekicks(tc), L.tc_boost(tc)


def main():
    for label, ukr in [("normal cars (dead zone 0.14-0.18)", (0.14, 0.18)),
                       ("weak cars (dead zone 0.19-0.215)", (0.19, 0.215))]:
        for deg in (90, -90, 30, 180):
            for boost_on in (False, True):
                ts, errs, ok = [], [], 0
                for s in range(200):
                    rng = random.Random(1000 + s)
                    t, e, st, rk, b = run(s, rng.uniform(*ukr), deg, boost_on, hiccup=True)
                    ts.append(t); errs.append(abs(e)); ok += (st == 2 and abs(e) <= 3.0 + 0.005 * abs(deg))
                ts.sort(); errs.sort()
                print(f"{label:36s} {deg:+4d}deg boost={'on ' if boost_on else 'off'} "
                      f"time mean {stt.mean(ts):.2f}s p95 {ts[189]:.2f}s max {ts[-1]:.2f}s | "
                      f"err mean {stt.mean(errs):.2f} p95 {errs[189]:.2f} | ok {ok / 2:.0f}%")


if __name__ == "__main__":
    main()
