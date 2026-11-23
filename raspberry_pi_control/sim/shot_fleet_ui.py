# -*- coding: utf-8 -*-
"""Serve fleet_ui with two simulated rovers and take headless-Chrome
screenshots (idle, mid-run). Foreground, bounded."""
import json, os, subprocess, sys, tempfile, threading, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE)); sys.path.insert(0, HERE)
import test_fleet_ui as t  # noqa: E402
import fleet_ui  # noqa: E402

CHROME = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
OUT = sys.argv[1] if len(sys.argv) > 1 else "/tmp/wr/shots"
os.makedirs(OUT, exist_ok=True)


def shot(name):
    subprocess.run([CHROME, "--headless=new", "--disable-gpu", "--hide-scrollbars",
                    "--window-size=1880,1180", "--virtual-time-budget=2500",
                    f"--screenshot={OUT}/{name}.png", t.BASE + "/"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=60)
    print("shot", name)


tmp = tempfile.mkdtemp()
fleet_ui.CONFIG_FILE = os.path.join(tmp, "fleet_config.json")
fleet_ui.CARS_FILE = os.path.join(tmp, "cars.txt")
open(fleet_ui.CARS_FILE, "w").write("car1 10.0.0.1\ncar2 10.0.0.2\n")
fleet_ui.make_rover = t.make_rover
cfg = fleet_ui.load_config()
F = lambda v: {"t": "F", "v": v}
T = lambda v: {"t": "T", "v": v}
cfg["cars"][0].update(steps=[F(1), T(-90), F(1), T(-90), F(1), T(-90), F(1), T(-90)], speed=0.25, delay=0)
cfg["cars"][0]["start"] = {"x": 0, "y": 1.0, "th": 0}
cfg["cars"][1].update(steps=[F(1.5), T(90), F(0.8), T(90), F(1.5), T(90), F(0.8), T(90)], speed=0.25, delay=2.0)
cfg["cars"][1]["start"] = {"x": 0, "y": -1.6, "th": 0}
fleet_ui.save_config(cfg)
fleet_ui.FLEET = fleet_ui.Fleet(cfg)
srv = fleet_ui.ThreadingHTTPServer(("127.0.0.1", 18090), fleet_ui.Handler)
threading.Thread(target=srv.serve_forever, daemon=True).start()
t.wait_for(lambda s: all(c["connected"] for c in s["cars"]), 15, "connected")
shot("idle")
specs = [{"id": c["id"], "steps": c["steps"], "speed": c["speed"], "delay": c["delay"],
          "start": c["start"]} for c in cfg["cars"]]
print(t.api("/api/start", {"cars": specs}))
t.wait_for(lambda s: any(c["phase"] == "straight" for c in s["cars"]), 20, "moving")
time.sleep(9)
shot("running")
t.api("/api/stop", {})
srv.shutdown()
