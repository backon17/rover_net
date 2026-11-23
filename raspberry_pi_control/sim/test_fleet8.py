# -*- coding: utf-8 -*-
"""8-car console test: 7 simulated V2 rovers + 1 offline car, driven through
the real web page (Chrome DevTools Protocol): select all, L template on one
car, copy to all, line up, one-click sync start. Foreground, bounded.

Run: python3 sim/test_fleet8.py [out_dir]"""
import json, math, os, subprocess, sys, tempfile, threading, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE)); sys.path.insert(0, HERE)
import test_fleet_ui as t  # noqa: E402  (fake rovers, api helpers)
import fleet_ui  # noqa: E402

CHROME = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
OUT = sys.argv[1] if len(sys.argv) > 1 else "/tmp/wr8"
os.makedirs(OUT, exist_ok=True)
SEEDS = {f"10.0.8.{i}": 100 + 7 * i for i in range(1, 9)}
START = {}


class Offline:
    def __init__(self, ip):
        raise ConnectionError("simulated offline car")


def make_rover(ip):
    if ip == "10.0.8.1":
        raise ConnectionError("simulated offline car")
    r = t.TracedRover(ip, SEEDS[ip])
    t.ROVERS[ip] = r
    return r


tmp = tempfile.mkdtemp()
fleet_ui.CONFIG_FILE = os.path.join(tmp, "fleet_config.json")
fleet_ui.CARS_FILE = os.path.join(tmp, "cars.txt")
open(fleet_ui.CARS_FILE, "w").write("".join(f"car{i} 10.0.8.{i}\n" for i in range(1, 9)))
fleet_ui.make_rover = make_rover
fleet_ui.scan_subnets = lambda prefixes: []          # no real network scan
cfg = fleet_ui.load_config()
fleet_ui.save_config(cfg)
fleet_ui.FLEET = fleet_ui.Fleet(cfg)
srv = fleet_ui.ThreadingHTTPServer(("127.0.0.1", 18090), fleet_ui.Handler)
threading.Thread(target=srv.serve_forever, daemon=True).start()
t.wait_for(lambda s: sum(c["connected"] for c in s["cars"]) == 7, 40, "7 cars connected")
for ip, r in t.ROVERS.items():
    START[ip] = (r.x, r.y, r.th)

chrome = subprocess.Popen([CHROME, "--headless=new", "--disable-gpu", "--hide-scrollbars",
                           "--remote-debugging-port=9333", f"--user-data-dir={tmp}/chrome",
                           t.BASE + "/"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
try:
    res = subprocess.run(["node", os.path.join(HERE, "cdp_drive.mjs"), "9333", OUT, "setup"],
                         capture_output=True, text=True, timeout=240)
finally:
    chrome.terminate()
    chrome.wait(timeout=10)
print(res.stdout[-6000:]); print(res.stderr[-2000:])

st = t.api("/api/state")
ok = True
cfg_now = json.load(open(fleet_ui.CONFIG_FILE, encoding="utf-8"))
want = [{"t": "F", "v": 1.0}, {"t": "T", "v": 90.0}, {"t": "F", "v": 0.3}]
for c in cfg_now["cars"]:
    good = c["enabled"] and [{"t": s["t"], "v": float(s["v"])} for s in c["steps"]] == want
    ok &= good
    print(f"config {c['id']}: enabled={c['enabled']} steps={c['steps']} start={c['start']} {'OK' if good else 'BAD'}")
# every online car finished; car1 skipped
starts_by_step = {}
for c in st["cars"]:
    if c["id"] == "car1":
        good = not c["connected"] and c["phase"] in ("offline", "idle")
        print(f"car1 (offline): phase {c['phase']} {'OK' if good else 'BAD'}")
        ok &= good
        continue
    good = c["phase"] == "done" and not c["error"]
    ok &= good
    ip = c["ip"]
    r = t.ROVERS[ip]
    x0, y0, th0 = START[ip]
    lx, ly, lh = t.pose_from_start(r, x0, y0, th0)
    err = math.hypot(lx - 1.0, ly - 0.3)
    print(f"{c['id']}: {c['phase']} end local ({lx:+.3f},{ly:+.3f}) head {lh:+.1f}  err {err*100:.1f} cm {'OK' if good else 'BAD'}")
    for line in c["log"]:
        if "[步 " in line:
            k = line.split("[步 ")[1].split("/")[0]
            hh, mm, ss = line[1:9].split(":")
            starts_by_step.setdefault(k, []).append(int(hh) * 3600 + int(mm) * 60 + int(ss))
for k, ts in sorted(starts_by_step.items()):
    spread = max(ts) - min(ts)
    print(f"step {k} start spread (1 s log resolution): {spread} s over {len(ts)} cars")
    ok &= spread <= 1 and len(ts) == 7
for line in res.stdout.splitlines():
    if line.startswith("JSERRORS") and line.strip() != "JSERRORS []":
        ok = False
print("RESULT", "PASS" if ok else "FAIL")
srv.shutdown()
