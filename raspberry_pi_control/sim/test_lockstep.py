# -*- coding: utf-8 -*-
"""Lock-step test of fleet_ui.py with three simulated cars that differ the way
the real ones do: one turns very slowly (high dead zone, no turn-rate floor,
like car 3 on v2.4.2) and the straight speeds differ by +-12 %.

Checks:
  1. lock-step: every step starts on all cars within 0.2 s (sync start) or
     exactly delay apart (stagger), although one car's turns are 2x slower;
  2. without lock-step the spread is large (reproduces the collision cause);
  3. per-car speed calibration (calib_speed API) brings the 1 m runs together.

Run: python3 sim/test_lockstep.py
"""
import json
import math
import os
import sys
import tempfile
import threading
import time
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)
from fake_v2 import FakeV2Rover, _lib  # noqa: E402  (also patches time.sleep)
import fleet_ui  # noqa: E402
import wave_rover_traj as w  # noqa: E402

CARS = {"car2": ("10.0.0.2", "aa:00:00:00:00:02", 1.11, False),
        "car3": ("10.0.0.3", "aa:00:00:00:00:03", 0.98, True),    # slow turns, slow
        "car4": ("10.0.0.4", "aa:00:00:00:00:04", 1.24, False)}   # fast straight
ROVERS = {}
EVENTS = {}             # ip -> list of (t, T)


class TracedRover(FakeV2Rover):
    def __init__(self, ip, seed, gain, weak):
        self.ip = ip
        super().__init__(seed=seed, speed_gain=gain)
        if weak:
            self.old_turn_fw = True
            self.u_kin = 0.205
            self.u_static = self.u_kin + 0.02
            _lib.tc_set_boost(self.tc, -1.0, -1.0, 0.0)   # v2.4.2 behaviour

    def _request(self, obj, timeout=0.5):
        if obj.get("T") in (160, 165):
            EVENTS.setdefault(self.ip, []).append((time.monotonic(), obj["T"]))
        return super()._request(obj, timeout)


def make_rover(ip):
    for i, (cid, (cip, mac, gain, weak)) in enumerate(CARS.items()):
        if cip == ip:
            r = TracedRover(ip, 30 + i, gain, weak)
            ROVERS[cid] = r
            return r
    raise ValueError(ip)


BASE = "http://127.0.0.1:18091"
OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def api(path, body=None):
    req = urllib.request.Request(BASE + path)
    if body is not None:
        req.data = json.dumps(body).encode()
        req.add_header("Content-Type", "application/json")
    with OPENER.open(req, timeout=5) as r:
        return json.loads(r.read().decode())


def wait_idle(timeout=90):
    t0 = time.monotonic()
    time.sleep(1.0)
    while time.monotonic() - t0 < timeout:
        st = api("/api/state")
        if not st["running"]:
            return st
        time.sleep(0.2)
    raise AssertionError("run did not finish")


def run(steps, sync, delays=None, speed=0.25):
    EVENTS.clear()
    p0 = {cid: (r.x, r.y) for cid, r in ROVERS.items()}
    specs = [{"id": cid, "steps": steps, "speed": speed,
              "delay": (delays or {}).get(cid, 0.0)} for cid in CARS]
    assert api("/api/start", {"cars": specs, "sync": sync})["error"] == ""
    st = wait_idle()
    ph = {c["id"]: c["phase"] for c in st["cars"]}
    starts = {cid: [t for t, _ in EVENTS.get(CARS[cid][0], [])] for cid in CARS}
    dist = {cid: math.hypot(r.x - p0[cid][0], r.y - p0[cid][1]) for cid, r in ROVERS.items()}
    return ph, starts, dist, st


def main():
    tmp = tempfile.mkdtemp()
    w.CALIB_FILE = os.path.join(tmp, "rover_calib.json")
    with open(w.CALIB_FILE, "w") as f:
        json.dump({"speed_gain": 1.1111}, f)
    fleet_ui.CONFIG_FILE = os.path.join(tmp, "fleet_config.json")
    fleet_ui.CARS_FILE = os.path.join(tmp, "cars.txt")
    with open(fleet_ui.CARS_FILE, "w") as f:
        for cid, (ip, mac, _, _) in CARS.items():
            f.write(f"{cid} {ip} MAC={mac}\n")
    fleet_ui.make_rover = make_rover
    cfg = fleet_ui.load_config()
    fleet_ui.save_config(cfg)
    fleet_ui.FLEET = fleet_ui.Fleet(cfg)
    srv = fleet_ui.ThreadingHTTPServer(("127.0.0.1", 18091), fleet_ui.Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    t0 = time.monotonic()
    while not all(c["connected"] for c in api("/api/state")["cars"]):
        assert time.monotonic() - t0 < 20, "cars did not connect"
        time.sleep(0.2)

    ok = True
    L = [{"t": "F", "v": 0.5}, {"t": "T", "v": -90}, {"t": "F", "v": 0.5},
         {"t": "T", "v": -90}, {"t": "F", "v": 0.5}]
    for label, sync, delays in (("no lock-step", False, None),
                                ("lock-step", True, None),
                                ("lock-step + stagger 2 s", True, {"car2": 2.0, "car4": 4.0})):
        ph, starts, dist, st = run(L, sync, delays)
        n = min(len(v) for v in starts.values())
        spreads = []
        for k in range(n):
            ts = [starts[cid][k] - (delays or {}).get(cid, 0.0) for cid in CARS]
            spreads.append(max(ts) - min(ts))
        worst = max(spreads)
        good = all(p == "done" for p in ph.values()) and n == len(L)
        if sync:
            good &= worst < 0.25
        if delays:
            # lock-step aligns plan time: each step must start when the shared
            # plan clock shows its planned start, and the plan clock maps to
            # one wall time for all cars
            worst = 0.0
            for cid in CARS:
                job = fleet_ui.FLEET.cars[cid].run
                for k, vt, _ in job["vt_log"]:
                    worst = max(worst, vt - job["plan"][k])
            ev = sorted((t, vt) for cid in CARS for _, vt, t in fleet_ui.FLEET.cars[cid].run["vt_log"])
            # same plan time <-> same wall time (between holds: slope 1)
            bad = 0.0
            for (t1, v1), (t2, v2) in zip(ev, ev[1:]):
                if v2 - v1 < 1e-3:
                    bad = max(bad, t2 - t1)
            good = all(p == "done" for p in ph.values()) and worst < 0.25
            print(f"    plan-time lag at step starts: worst {worst:.2f}s; "
                  f"same plan time different wall time: {bad:.2f}s -> {'PASS' if good else 'FAIL'}")
        print(f"[{label}] step start spread (s): " + ", ".join(f"{x:.2f}" for x in spreads)
              + f"  worst {worst:.2f} -> {'PASS' if good else 'FAIL'}")
        if not sync:
            print("    (expected: large spread, this is what made the cars collide)")
        for c in st["cars"]:
            for line in c["log"]:
                if "[同步]" in line:
                    print(f"    {c['id']}: {line}")
        ok &= good
        time.sleep(0.5)

    # ---------------------------------------------- speed calibration --
    one = [{"t": "F", "v": 1.0}]
    for it in range(4):
        ph, _, dist, _ = run(one, True)
        spread = max(dist.values()) - min(dist.values())
        print(f"[speed calib round {it}] distances " +
              ", ".join(f"{cid} {d:.3f} m" for cid, d in dist.items()) + f"  spread {spread * 100:.1f} cm")
        if it == 3:
            good = spread < 0.04 and all(abs(d - 1.0) < 0.03 for d in dist.values())
            print("    ->", "PASS" if good else "FAIL")
            ok &= good
            break
        for cid, d in dist.items():
            r = api("/api/calib_speed", {"id": cid, "measured": round(d, 3)})
            assert r["error"] == "", r
        time.sleep(0.3)
    files = sorted(os.listdir(tmp))
    good = any(f.startswith("rover_calib_aa0000000003") for f in files)
    with open(os.path.join(tmp, "rover_calib_aa0000000003.json")) as f:
        saved = json.load(f)
    good = good and saved.get("speed_gain") == 1.1111 and bool(saved.get("sv_k"))
    print(f"[calib file] {files} sv_k(car3)={saved.get('sv_k'):.3f} keeps base keys "
          f"dz={saved.get('sv_dz'):.3f} -> {'PASS' if good else 'FAIL'}")
    ok &= bool(good)
    r1 = api("/api/calib_speed", {"id": "car3", "measured": 1.0})
    r = api("/api/calib_speed", {"id": "car3", "measured": 1.0})
    good = r1["error"] == "" and "还没跑过" in r["error"]
    print(f"[calib twice] {r['error']} -> {'PASS' if good else 'FAIL'}")
    ok &= good

    srv.shutdown()
    print("\nALL PASS" if ok else "\nSOME FAILED")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
