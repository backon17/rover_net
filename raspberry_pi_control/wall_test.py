# Wall test: straight 1 m toward a wall, log current/state every 50 ms.
import json, sys, time, urllib.parse, urllib.request
IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.137.73"
DIST = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
SPEED = float(sys.argv[3]) if len(sys.argv) > 3 else 0.25
SIGN = 1.0 if DIST >= 0 else -1.0

def u(o, timeout=0.8):
    try:
        r = urllib.request.urlopen(f"http://{IP}/js?json=" + urllib.parse.quote(json.dumps(o)), timeout=timeout).read()
        return json.loads(r) if r.strip() else None
    except Exception as e:
        return None

for attempt in range(3):
    u({"T": 162, "ms": 1000, "zero": 1})
    time.sleep(1.5)
    st = u({"T": 164})
    print(f"calib: gb={st['gb']} gn={st['gn']}", flush=True)
    if st["gn"] < 2.5:
        break
pwm = SPEED / 1.11
dur = abs(DIST) / (pwm * 1.11) + 0.5 * 0.6 / 2
st = u({"T": 165, "ms": int(dur * 1000), "spd": round(SIGN * pwm, 4), "ramp": 500}, timeout=2)
t0 = time.monotonic()
print(f"start: ss={st and st.get('ss')} dur={dur:.2f}s pwm={pwm:.3f}", flush=True)
print("  t     ss   si   sib  sip   se", flush=True)
try:
    while True:
        st = u({"T": 164})
        t = time.monotonic() - t0
        if st:
            print(f"{t:5.2f}  {st['ss']}  {st['si']:4d} {st['sib']:4d} {st['sip']:4d} {st['se']:5.1f}", flush=True)
            if st["ss"] != 1 and t > 0.3:
                break
        if t > dur + 2.0:
            print("timeout -> stop", flush=True)
            break
        time.sleep(0.05)
finally:
    u({"T": 161})
st = u({"T": 164})
print("final:", {k: st.get(k) for k in ("ss", "si", "sib", "sip", "sm", "sa", "stf", "gy")}, flush=True)
