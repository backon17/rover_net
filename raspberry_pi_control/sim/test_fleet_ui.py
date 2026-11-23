# -*- coding: utf-8 -*-
"""End-to-end test of fleet_ui.py with two simulated V2 rovers (real firmware
turn logic, see fake_v2.py), driven through the real HTTP API.

Checks: sync start, staggered start timing, path end poses, all-stop latency,
per-car stop before departure, calibration failure cancels the other car.

Run: python3 sim/test_fleet_ui.py
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
from fake_v2 import FakeV2Rover  # noqa: E402  (also patches time.sleep)
import fleet_ui  # noqa: E402
import wave_rover_traj as w  # noqa: E402

IPS = {"car1": "10.0.0.1", "car2": "10.0.0.2"}
ROVERS = {}
EVENTS = {}             # ip -> list of (t, "cal" | "move")


class TracedRover(FakeV2Rover):
    def __init__(self, ip, seed):
        self.ip = ip
        self.fail_calib = False
        super().__init__(seed=seed)
        self.speed_gain_true = self.speed_gain

    def _request(self, obj, timeout=0.5):
        t = obj.get("T")
        ev = EVENTS.setdefault(self.ip, [])
        if t == 162:
            ev.append((time.monotonic(), "cal"))
            if self.fail_calib:
                # firmware never finishes calibrating -> fw_calibrate times out
                with self.lock:
                    self._stuck = True
        if t == 1 and (abs(obj.get("L", 0)) > 0 or abs(obj.get("R", 0)) > 0):
            if not ev or ev[-1][1] != "move":
                ev.append((time.monotonic(), "move"))
        if t == 160:
            if not ev or ev[-1][1] != "move":
                ev.append((time.monotonic(), "move"))
        msg = super()._request(obj, timeout)
        if getattr(self, "_stuck", False) and isinstance(msg, dict) and msg.get("T") == 1010:
            msg = dict(msg, cal=1)
        return msg


def make_rover(ip):
    seed = 11 if ip.endswith(".1") else 22
    r = TracedRover(ip, seed)
    ROVERS[ip] = r
    return r


BASE = "http://127.0.0.1:18090"


def api(path, body=None):
    req = urllib.request.Request(BASE + path)
    if body is not None:
        req.data = json.dumps(body).encode()
        req.add_header("Content-Type", "application/json")
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open(req, timeout=5) as r:
        return json.loads(r.read().decode())


def wait_for(pred, timeout, what):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        st = api("/api/state")
        if pred(st):
            return st
        time.sleep(0.2)
    raise AssertionError(f"timeout waiting for {what}: {json.dumps(api('/api/state'), ensure_ascii=False)[:800]}")


def phases(st):
    return {c["id"]: c["phase"] for c in st["cars"]}


def reset_pose():
    for r in ROVERS.values():
        with r.lock:
            r.th0 = r.th
            r.x0, r.y0 = r.x, r.y


def pose_from_start(r, x0, y0, th0):
    dx, dy = r.x - x0, r.y - y0
    c, s = math.cos(-th0), math.sin(-th0)
    return dx * c - dy * s, dx * s + dy * c, w.wrap_deg(math.degrees(r.th - th0))


def main():
    tmp = tempfile.mkdtemp()
    fleet_ui.CONFIG_FILE = os.path.join(tmp, "fleet_config.json")
    fleet_ui.CARS_FILE = os.path.join(tmp, "cars.txt")
    with open(fleet_ui.CARS_FILE, "w") as f:
        f.write(f"car1 {IPS['car1']}\ncar2 {IPS['car2']}\n")
    fleet_ui.make_rover = make_rover
    cfg = fleet_ui.load_config()
    fleet_ui.save_config(cfg)
    fleet_ui.FLEET = fleet_ui.Fleet(cfg)
    srv = fleet_ui.ThreadingHTTPServer(("127.0.0.1", 18090), fleet_ui.Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()

    ok = True
    st = wait_for(lambda s: all(c["connected"] for c in s["cars"]), 15, "both connected")
    print("connected:", [(c["name"], c["fw"], c["bat"]) for c in st["cars"]])
    html = urllib.request.build_opener(urllib.request.ProxyHandler({})).open(BASE + "/").read()
    assert b"fleet" in html.lower() or "多车".encode() in html
    plans = api("/api/plans")
    print("plans:", [p["name"] for p in plans])

    # ---------------------------------------------------------- 1. stagger --
    paths = {"car1": [{"t": "F", "v": 1.0}, {"t": "T", "v": -90}, {"t": "F", "v": 0.5}],
             "car2": [{"t": "F", "v": 1.0}, {"t": "T", "v": 90}, {"t": "F", "v": 0.5}]}
    ends = {"car1": (1.0, -0.5), "car2": (1.0, 0.5)}
    for label, delays in (("sync", {"car1": 0.0, "car2": 0.0}),
                          ("stagger", {"car1": 0.0, "car2": 2.0})):
        EVENTS.clear()
        starts = {ip: (r.x, r.y, r.th) for ip, r in ROVERS.items()}
        specs = [{"id": cid, "steps": paths[cid], "speed": 0.25, "delay": delays[cid],
                  "start": {"x": 0, "y": 0, "th": 0}} for cid in ("car1", "car2")]
        r = api("/api/start", {"cars": specs})
        assert r["error"] == "", r
        st = wait_for(lambda s: not s["running"], 60, f"{label} run to finish")
        ph = phases(st)
        moves = {}
        for cid, ip in IPS.items():
            ev = EVENTS[ip]
            moves[cid] = next(t for t, k in ev if k == "move")
        gap = moves["car2"] - moves["car1"]
        line = f"[{label}] phases={ph} departure gap car2-car1 = {gap:+.3f}s (want {delays['car2']:+.1f})"
        good = all(p == "done" for p in ph.values()) and abs(gap - delays["car2"]) < 0.15
        for cid, ip in IPS.items():
            x, y, th = pose_from_start(ROVERS[ip], *starts[ip])
            ex, ey = ends[cid]
            perr = math.hypot(x - ex, y - ey)
            line += f"\n    {cid}: end ({x:+.3f},{y:+.3f}) heading {th:+.1f}°, pos err {perr * 100:.1f} cm"
            good &= perr < 0.10
        print(line, "->", "PASS" if good else "FAIL")
        ok &= good
        time.sleep(1.0)

    # --------------------------------------------------------- 2. all-stop --
    specs = [{"id": cid, "steps": [{"t": "F", "v": 3.0}], "speed": 0.25, "delay": 0.0,
              "start": {"x": 0, "y": 0, "th": 0}} for cid in ("car1", "car2")]
    assert api("/api/start", {"cars": specs})["error"] == ""
    wait_for(lambda s: all(c["phase"] == "straight" for c in s["cars"]), 20, "both driving")
    time.sleep(1.5)
    t0 = time.monotonic()
    api("/api/stop", {})
    stop_lat = time.monotonic() - t0
    time.sleep(0.6)
    speeds = [abs(r.v) for r in ROVERS.values()] if hasattr(next(iter(ROVERS.values())), "v") else []
    cmds = [(r.cmd_l, r.cmd_r) for r in ROVERS.values()]
    st = wait_for(lambda s: not s["running"], 5, "stop to take effect")
    good = all(p == "stopped" for p in phases(st).values()) and all(c == (0.0, 0.0) for c in cmds)
    print(f"[all-stop] api returned in {stop_lat:.2f}s, wheel cmds after 0.6s = {cmds}, "
          f"phases={phases(st)} -> {'PASS' if good else 'FAIL'}")
    ok &= good

    # ---------------------------------------- 3. per-car stop before going --
    specs = [{"id": "car1", "steps": [{"t": "F", "v": 0.5}], "speed": 0.25, "delay": 0.0},
             {"id": "car2", "steps": [{"t": "F", "v": 0.5}], "speed": 0.25, "delay": 5.0}]
    assert api("/api/start", {"cars": specs})["error"] == ""
    wait_for(lambda s: phases(s)["car2"] == "waiting", 20, "car2 waiting")
    api("/api/stop", {"ids": ["car2"]})
    st = wait_for(lambda s: not s["running"], 30, "car1 finish")
    good = phases(st) == {"car1": "done", "car2": "stopped"}
    print(f"[solo-stop] {phases(st)} -> {'PASS' if good else 'FAIL'}")
    ok &= good

    # ------------------------------- 4. calibration failure cancels others --
    EVENTS.clear()
    ROVERS[IPS["car2"]].fail_calib = True
    specs = [{"id": cid, "steps": [{"t": "F", "v": 0.5}], "speed": 0.25, "delay": 0.0}
             for cid in ("car1", "car2")]
    assert api("/api/start", {"cars": specs})["error"] == ""
    st = wait_for(lambda s: not s["running"], 30, "cancel")
    car1_moved = any(k == "move" for _, k in EVENTS.get(IPS["car1"], []))
    good = phases(st) == {"car1": "stopped", "car2": "error"} and not car1_moved
    print(f"[cal-fail] {phases(st)} car1 moved={car1_moved} -> {'PASS' if good else 'FAIL'}")
    print("    car1 log tail:", st["cars"][0]["log"][-1])
    print("    car2 error:", st["cars"][1]["error"])
    ok &= good
    ROVERS[IPS["car2"]].fail_calib = False
    ROVERS[IPS["car2"]]._stuck = False

    # ---------------------------------------------- 5. busy / bad requests --
    r = api("/api/start", {"cars": [{"id": "car1", "steps": [], "speed": 0.2}]})
    good = "没有任何动作" in r["error"]
    r2 = api("/api/start", {"cars": [{"id": "car1", "steps": [{"t": "F", "v": 1}], "speed": 2}]})
    good &= "速度" in r2["error"]
    print(f"[validate] {r['error']} / {r2['error']} -> {'PASS' if good else 'FAIL'}")
    ok &= good

    srv.shutdown()
    print("\nALL PASS" if ok else "\nSOME FAILED")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
