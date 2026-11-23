import sys
IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.137.73"
import urllib.request, json, time, urllib.parse
def u(o):
    r = urllib.request.urlopen(f"http://{IP}/js?json=" + urllib.parse.quote(json.dumps(o)), timeout=3).read()
    return json.loads(r) if r.strip() else None
for i in range(3):
    u({"T": 162, "ms": 1000, "zero": 0})
    time.sleep(1.6)
    d = u({"T": 164})
    print("gb", d["gb"], "gn", d["gn"], "cal", d["cal"], "lp", d["lp"], "gr", d["gr"], flush=True)
