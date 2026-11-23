# Read-only check for fleet_ui: scan the hotspot subnet and connect to every
# car in the config. Sends no motion commands.
import time
import fleet_ui
cfg = fleet_ui.load_config()
if not __import__("os").path.exists(fleet_ui.CONFIG_FILE):
    fleet_ui.save_config(cfg)
fleet = fleet_ui.Fleet(cfg)
t0 = time.monotonic()
res = fleet.scan()
print(f"scan {time.monotonic() - t0:.1f}s:", res)
t0 = time.monotonic()
while time.monotonic() - t0 < 12:
    if all(c["connected"] for c in fleet.state()["cars"]):
        break
    time.sleep(0.5)
for c in fleet.state()["cars"]:
    print(c["name"], c["ip"], c["mac"], "connected" if c["connected"] else "OFFLINE",
          c["conn_err"], "bat", c["bat"], "fw", c["fw"])
for car in fleet.cars.values():
    car.stop_poll.set()
