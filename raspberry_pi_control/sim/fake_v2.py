# -*- coding: utf-8 -*-
"""Fake V2-firmware WAVE ROVER for end-to-end tests of wave_rover_traj.py.

* The on-board heading/turn logic is the REAL firmware code (turn_logic.h,
  loaded from libturn.so through ctypes), driven by a 5 ms "loop()" thread.
* Plant: 2-D skid-steer body with dead zone, static friction for pivoting,
  short-brake damping, one side slightly weaker (straight-line drift),
  gyro scale error / bias / noise.
* Speaks the same HTTP-style protocol as the firmware through
  WifiRover._request(), with random request latency.
"""
import ctypes
import math
import os
import random
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import wave_rover_traj as w  # noqa: E402

# This Mac coalesces short timers (time.sleep(0.005) really sleeps ~200 ms in a
# background process), which would make the fake 5 ms firmware loop and the
# PC-side 50 ms loop unrealistically slow. Use a GIL-yielding spin sleep.
_orig_sleep = time.sleep


def _spin_sleep(d):
    end = time.monotonic() + d
    while end - time.monotonic() > 0:
        _orig_sleep(0)


time.sleep = _spin_sleep

_lib = ctypes.CDLL(os.path.join(HERE, "libturn.so"))
_vp = ctypes.c_void_p
for name, res, args in [
    ("gh_new", _vp, []),
    ("gh_update", None, [_vp, ctypes.c_float, ctypes.c_float]),
    ("gh_heading", ctypes.c_double, [_vp]),
    ("gh_rate", ctypes.c_float, [_vp]),
    ("gh_bias", ctypes.c_float, [_vp]),
    ("gh_set_scale", None, [_vp, ctypes.c_float]),
    ("gh_calib", None, [_vp, ctypes.c_float, ctypes.c_int]),
    ("gh_calibrating", ctypes.c_int, [_vp]),
    ("tc_new", _vp, []),
    ("tc_start", None, [_vp, ctypes.c_double, ctypes.c_double, ctypes.c_float,
                        ctypes.c_float, ctypes.c_float]),
    ("tc_step", None, [_vp, ctypes.c_double, ctypes.c_double, ctypes.c_float]),
    ("tc_abort", None, [_vp]),
    ("tc_l", ctypes.c_float, [_vp]),
    ("tc_r", ctypes.c_float, [_vp]),
    ("tc_state", ctypes.c_int, [_vp]),
    ("tc_phase", ctypes.c_int, [_vp]),
    ("tc_err", ctypes.c_float, [_vp]),
    ("tc_segs", ctypes.c_int, [_vp]),
    ("tc_rekicks", ctypes.c_int, [_vp]),
    ("tc_ck", ctypes.c_float, [_vp]),
    ("tc_set", None, [_vp, ctypes.c_float, ctypes.c_float]),
    ("tc_boost", ctypes.c_float, [_vp]),
    ("tc_set_boost", None, [_vp, ctypes.c_float, ctypes.c_float, ctypes.c_float]),
    ("gh_noise", ctypes.c_float, [_vp]),
    ("sc_new", _vp, []),
    ("sc_start", None, [_vp, ctypes.c_double, ctypes.c_double, ctypes.c_float,
                        ctypes.c_float, ctypes.c_float]),
    ("sc_step", None, [_vp, ctypes.c_double, ctypes.c_double, ctypes.c_float]),
    ("sc_abort", None, [_vp]),
    ("sc_state", ctypes.c_int, [_vp]),
    ("sc_l", ctypes.c_float, [_vp]),
    ("sc_r", ctypes.c_float, [_vp]),
    ("sc_progress", ctypes.c_float, [_vp, ctypes.c_double]),
    ("sc_err", ctypes.c_float, [_vp]),
    ("sc_maxerr", ctypes.c_float, [_vp]),
    ("sc_meanerr", ctypes.c_float, [_vp]),
    ("sc_set", None, [_vp] + [ctypes.c_float] * 5),
]:
    f = getattr(_lib, name)
    f.restype = res
    f.argtypes = args


class FakeV2Rover(w.WifiRover):
    LOOP_S = 0.005

    def __init__(self, seed=0, speed_gain=1.11, latency=(0.004, 0.03)):
        rng = random.Random(seed)
        self.rng = rng
        self.lock = threading.Lock()
        self.feedback = {}
        self._fail_streak = 0
        self._fast_stop_on = False
        self.latency = latency
        # plant
        self.x = self.y = 0.0
        self.th = math.radians(rng.uniform(-180, 180))   # arbitrary world heading
        self.th0 = self.th
        self.w = 0.0                 # yaw rate, dps
        self.stuck = True
        self.G = rng.uniform(380, 600)
        self.tau = rng.uniform(0.08, 0.25)
        self.u_kin = rng.uniform(0.14, 0.18)
        self.u_static = self.u_kin + rng.uniform(0.01, 0.05)
        self.speed_gain = speed_gain
        self.right_weak = rng.uniform(0.95, 0.99)   # right side slightly weaker
        self.g_scale = rng.uniform(0.995, 1.005)
        self.g_bias = rng.uniform(-0.6, 0.6)
        # firmware state
        self.gh = _lib.gh_new()
        self.tc = _lib.tc_new()
        self.sc = _lib.sc_new()
        self.straight_fw = True       # emulate v2.3 (T:165); False = v2.2
        self.cmd_l = self.cmd_r = 0.0
        self.deadline = 0.0
        self.heartbeat = 3.0
        self.last_cmd = 0.0
        self.t = time.monotonic()
        _lib.gh_calib(self.gh, 1.0, 1)          # boot-time calibration
        self._alive = True
        self._thr = threading.Thread(target=self._loop, daemon=True)
        self._thr.start()
        time.sleep(1.2)
        self._request({"T": 136, "cmd": 400})

    # ---------------- plant + firmware loop ----------------
    def _plant(self, ul, ur, dt):
        ur_eff = ur * self.right_weak
        v = self.speed_gain * 0.5 * (ul + ur_eff)
        if abs(ul + ur_eff) < 2 * 0.06:
            v = 0.0
        u = 0.5 * (ur_eff - ul)
        moving = min(1.0, abs(v) / 0.2)
        u_kin = self.u_kin * (1.0 - 0.85 * moving)      # rolling eases turning
        u_static = self.u_static * (1.0 - 0.85 * moving)
        f = self.G * u_kin / self.tau
        if self.stuck:
            if abs(u) >= u_static:
                self.stuck = False
        if not self.stuck:
            dw = (self.G * u - self.w) / self.tau
            sgn = (self.w > 0) - (self.w < 0) or ((u > 0) - (u < 0))
            dw -= f * sgn
            w_new = self.w + dw * dt
            if self.w != 0.0 and w_new * self.w <= 0 and abs(self.G * u) < f * self.tau:
                w_new = 0.0
                self.stuck = True
            self.th += math.radians(0.5 * (self.w + w_new) * dt)
            self.w = w_new
        self.v_body = v
        self.x += v * math.cos(self.th) * dt
        self.y += v * math.sin(self.th) * dt

    def _loop(self):
        last = time.monotonic()
        while self._alive:
            time.sleep(self.LOOP_S)
            now = time.monotonic()
            dt = now - last
            last = now
            with self.lock:
                # motor deadman (lease / heartbeat), as in the patched firmware
                if self.deadline and now >= self.deadline:
                    self.cmd_l = self.cmd_r = 0.0
                    self.deadline = 0.0
                if (now - self.last_cmd > self.heartbeat and not _lib.tc_state(self.tc) == 1
                        and not _lib.sc_state(self.sc) == 1):
                    self.cmd_l = self.cmd_r = 0.0
                sub = max(1, int(dt / 0.001))
                for _ in range(sub):
                    self._plant(self.cmd_l, self.cmd_r, dt / sub)
                gz = self.w * self.g_scale + self.g_bias + self.rng.gauss(0, 0.5)
                gz = round(gz * 16) / 16
                _lib.gh_update(self.gh, gz, dt)
                if _lib.tc_state(self.tc) == 1:
                    _lib.tc_step(self.tc, now, _lib.gh_heading(self.gh), _lib.gh_rate(self.gh))
                    self.cmd_l, self.cmd_r = _lib.tc_l(self.tc), _lib.tc_r(self.tc)
                    self.last_cmd = now
                if _lib.sc_state(self.sc) == 1:
                    _lib.sc_step(self.sc, now, _lib.gh_heading(self.gh), _lib.gh_rate(self.gh))
                    self.cmd_l, self.cmd_r = _lib.sc_l(self.sc), _lib.sc_r(self.sc)
                    self.last_cmd = now

    # ---------------- protocol ----------------
    def _status(self):
        return {"T": 1010, "gy": round(_lib.gh_heading(self.gh), 2),
                "gr": round(_lib.gh_rate(self.gh), 1), "ts": _lib.tc_state(self.tc),
                "tp": _lib.tc_phase(self.tc), "te": round(_lib.tc_err(self.tc), 1),
                "tn": _lib.tc_segs(self.tc), "tk": _lib.tc_rekicks(self.tc),
                "tb": round(_lib.tc_boost(self.tc), 3),
                "ck": round(_lib.tc_ck(self.tc), 3), "gb": round(_lib.gh_bias(self.gh), 3),
                "cal": _lib.gh_calibrating(self.gh), "lp": 6.0, "v": 12.1,
                **({"gn": round(_lib.gh_noise(self.gh), 3), "ss": _lib.sc_state(self.sc),
                    "sp": round(_lib.sc_progress(self.sc, time.monotonic()), 3),
                    "se": round(_lib.sc_err(self.sc), 1), "sm": round(_lib.sc_maxerr(self.sc), 1),
                    "sa": round(_lib.sc_meanerr(self.sc), 2), "si": 0, "sib": 0, "sip": 0,
                    "stf": 0.0, "stb": 0.0} if self.straight_fw else {})}

    def _request(self, obj, timeout=0.5):
        time.sleep(self.rng.uniform(*self.latency))
        now = time.monotonic()
        t = obj.get("T")
        with self.lock:
            if t == 1:
                _lib.tc_abort(self.tc)
                _lib.sc_abort(self.sc)
                self.cmd_l, self.cmd_r = float(obj["L"]), float(obj["R"])
                self.last_cmd = now
                ms = obj.get("ms")
                self.deadline = now + ms / 1000.0 if ms else 0.0
                return self._status() if obj.get("fb") == 1 else None
            if t == 136:
                self.heartbeat = obj["cmd"] / 1000.0
                return None
            if t == 130:
                st = {"T": 1001, "y": round(-_lib.gh_heading(self.gh) % 360, 2), "v": 12.1}
                self.feedback = st
                return st
            if t == 126:
                return {"T": 1002, "y": 0.0, "gz": self.w}
            if t == 165 and self.straight_fw:
                if not _lib.gh_calibrating(self.gh):
                    _lib.tc_abort(self.tc)
                    hd = float(obj["hd"]) if "hd" in obj else _lib.gh_heading(self.gh)
                    _lib.sc_start(self.sc, now, hd, float(obj.get("spd", 0.2)),
                                  float(obj.get("ms", 0)) / 1000.0, float(obj.get("ramp", 500)) / 1000.0)
                    self.last_cmd = now
                return self._status()
            if t == 160:
                if not _lib.gh_calibrating(self.gh):
                    _lib.sc_abort(self.sc)
                    _lib.tc_start(self.tc, now, _lib.gh_heading(self.gh), float(obj.get("deg", 0)),
                                  float(obj.get("spd", 0.3)), float(obj.get("tmo", 0)))
                    self.last_cmd = now
                return self._status()
            if t == 161:
                _lib.tc_abort(self.tc)
                _lib.sc_abort(self.sc)
                self.cmd_l = self.cmd_r = 0.0
                return self._status()
            if t == 162:
                _lib.tc_abort(self.tc)
                _lib.sc_abort(self.sc)
                self.cmd_l = self.cmd_r = 0.0
                _lib.gh_calib(self.gh, obj.get("ms", 1000) / 1000.0, obj.get("zero", 1))
                return self._status()
            if t == 163:
                if "gs" in obj:
                    _lib.gh_set_scale(self.gh, float(obj["gs"]))
                if any(k in obj for k in ("skp", "ski", "skd", "scf", "sr0")):
                    _lib.sc_set(self.sc, *[float(obj.get(k, -1)) for k in ("skp", "ski", "skd", "scf", "sr0")])
                if "trc" in obj and not getattr(self, "old_turn_fw", False):
                    _lib.tc_set_boost(self.tc, float(obj["trc"]), -1.0, -1.0)
                if "kick" in obj or "minp" in obj:
                    _lib.tc_set(self.tc, float(obj.get("kick", 0.4)), float(obj.get("minp", 0.22)))
                return self._status()
            if t == 164:
                return self._status()
        return None

    def close(self):
        self.stop()
        self._alive = False

    # ground truth in the start frame (start pose = origin, +x = initial heading)
    def true_pose(self):
        dx, dy = self.x, self.y
        c, s = math.cos(-self.th0), math.sin(-self.th0)
        return (dx * c - dy * s, dx * s + dy * c,
                w.wrap_deg(math.degrees(self.th - self.th0)))
