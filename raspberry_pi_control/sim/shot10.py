# Screenshot the console with 10 simulated cars (layout check). Foreground, bounded.
import os, sys, tempfile, threading
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE)); sys.path.insert(0, HERE)
import test_fleet_ui as t  # noqa: E402
import fleet_ui  # noqa: E402
import shot_fleet_ui as s  # noqa: E402  (runs its own 2-car demo first; harmless)
tmp = tempfile.mkdtemp()
fleet_ui.CONFIG_FILE = os.path.join(tmp, "fleet_config.json")
fleet_ui.CARS_FILE = os.path.join(tmp, "cars.txt")
open(fleet_ui.CARS_FILE, "w").write("".join(f"car{i} 10.0.1.{i}\n" for i in range(1, 11)))
cfg = fleet_ui.load_config()
fleet_ui.save_config(cfg)
fleet_ui.FLEET = fleet_ui.Fleet(cfg)
srv = fleet_ui.ThreadingHTTPServer(("127.0.0.1", 18090), fleet_ui.Handler)
threading.Thread(target=srv.serve_forever, daemon=True).start()
t.wait_for(lambda st: all(c["connected"] for c in st["cars"]), 30, "connected")
s.shot("ten_cars")
srv.shutdown()
