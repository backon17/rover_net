# -*- coding: utf-8 -*-
"""Run one step list on several cars with a single command, through the
running web console (fleet_ui.py, http://localhost:8090). Same code path as
the console's "sync start": every car calibrates, then all leave together.

Usage:
  python fleet_go.py                                  # all online cars, F0.5,T-90,F0.5
  python fleet_go.py --cars car2,car3,car4 --steps "F0.5,T-90,F0.5" --speed 0.25
  python fleet_go.py --delay 2                        # car k leaves k*2 s after the first
  python fleet_go.py --stop                           # E-stop every car
  python fleet_go.py --no-sync                        # don't align the steps

Lock-step (default): every turn / straight starts on all cars at the same
moment; a car that is slower on a step makes the others wait for it.

Per-car speed calibration (cars differ in motor dead zone, so the same speed
command gives different real speeds):
  python fleet_go.py --calib-speed --cars car2,car3,car4      # each drives 1 m
  (measure how far each car really went, then)
  python fleet_go.py --set-speed car2=0.93,car3=0.81,car4=1.02
Repeat once to check: all cars should then cover ~1 m.

Steps: F<metres> = forward (negative = back), T<deg> = turn (positive = left).
Ctrl+C while running = E-stop all cars.
"""
import argparse
import json
import sys
import time
import urllib.request

URL = "http://localhost:8090"
OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def api(path, body=None, timeout=5.0):
    data = None if body is None else json.dumps(body).encode("utf-8")
    req = urllib.request.Request(URL + path, data=data,
                                 headers={"Content-Type": "application/json"})
    with OPENER.open(req, timeout=timeout) as r:
        return json.loads(r.read().decode("utf-8") or "{}")


def parse_steps(text):
    steps = []
    for tok in text.replace(" ", "").split(","):
        if not tok:
            continue
        kind, val = tok[0].upper(), float(tok[1:])
        if kind not in ("F", "T"):
            raise ValueError(f"bad step {tok!r} (use F<m> or T<deg>)")
        steps.append({"t": kind, "v": val})
    if not steps:
        raise ValueError("no steps")
    return steps


def stop_all():
    print("E-stop:", api("/api/stop", {}).get("error") or "sent")


def main():
    # a GBK console (or a redirected file) cannot print every log symbol
    for stream in (sys.stdout, sys.stderr):
        stream.reconfigure(errors="replace")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cars", help="comma list of car ids (default: every connected car)")
    ap.add_argument("--steps", default="F0.5,T-90,F0.5")
    ap.add_argument("--speed", type=float, default=0.25, help="m/s")
    ap.add_argument("--delay", type=float, default=0.0, help="stagger: car k leaves k*delay s later")
    ap.add_argument("--stop", action="store_true", help="E-stop every car and exit")
    ap.add_argument("--no-sync", action="store_true", help="don't align the cars at every step")
    ap.add_argument("--calib-speed", action="store_true",
                    help="drive --dist straight on every car, for --set-speed")
    ap.add_argument("--dist", type=float, default=1.0, help="metres for --calib-speed")
    ap.add_argument("--set-speed", metavar="car=m,...",
                    help="measured distances of the last --calib-speed run")
    args = ap.parse_args()
    if args.set_speed:
        for tok in args.set_speed.replace(" ", "").split(","):
            if not tok:
                continue
            cid, _, m = tok.partition("=")
            r = api("/api/calib_speed", {"id": cid, "measured": m})
            if r.get("error"):
                print(f"{cid}: {r['error']}")
            else:
                print(f"{cid}: real speed {r['v_meas']:.3f} m/s at PWM {r['old_pwm']:.3f}; "
                      f"from now on PWM {r['new_pwm']:.3f} (saved to {r['file']})")
        return
    if args.calib_speed:
        args.steps = f"F{args.dist}"

    try:
        st = api("/api/state")
    except Exception as e:
        sys.exit(f"console not reachable at {URL} ({e}); start it with fleet_ui.bat")
    if args.stop:
        stop_all()
        return

    cars = {c["id"]: c for c in st["cars"]}
    ids = [s.strip() for s in args.cars.split(",")] if args.cars else \
        [c["id"] for c in st["cars"] if c["connected"]]
    for cid in ids:
        if cid not in cars:
            sys.exit(f"unknown car {cid}; known: {', '.join(cars)}")
        if not cars[cid]["connected"]:
            sys.exit(f"{cid} is offline ({cars[cid]['conn_err']})")
    if not ids:
        sys.exit("no connected car")
    steps = parse_steps(args.steps)
    specs = [{"id": cid, "steps": steps, "speed": args.speed, "delay": k * args.delay}
             for k, cid in enumerate(ids)]
    print(f"cars {', '.join(ids)} | steps {args.steps} | {args.speed} m/s | stagger {args.delay} s"
          f" | {'no step alignment' if args.no_sync else 'steps aligned'}")
    err = api("/api/start", {"cars": specs, "sync": not args.no_sync}).get("error")
    if err:
        sys.exit("start refused: " + err)

    # the console keeps only the last N log lines per car, so track the last
    # printed line rather than a count (a count stops advancing once the
    # buffer is full)
    last = {cid: (cars[cid]["log"][-1] if cars[cid]["log"] else None) for cid in ids}
    t0 = time.monotonic()
    try:
        while True:
            time.sleep(0.5)
            try:
                st = api("/api/state")
            except OSError:
                print('控制台不可达；已提交任务由树莓派继续，实时状态未知。', flush=True)
                continue
            now = {c["id"]: c for c in st["cars"]}
            for cid in ids:
                log = now[cid]["log"]
                start = 0
                if last[cid] is not None:
                    for k in range(len(log) - 1, -1, -1):
                        if log[k] == last[cid]:
                            start = k + 1
                            break
                for line in log[start:]:
                    print(f"{cid}: {line}", flush=True)
                if log:
                    last[cid] = log[-1]
            if time.monotonic() - t0 > 3 and not st["running"]:
                break
            if time.monotonic() - t0 > 300:
                print('监控超过 300 秒，任务可能仍在节点执行；请查询状态或明确发送 --stop。')
                sys.exit(2)
    except KeyboardInterrupt:
        stop_all()
        sys.exit(1)

    print()
    ok = True
    for cid in ids:
        c = now[cid]
        bad = c["phase"] != "done" or c["error"] or not c.get('node_online', True)
        ok &= not bad
        print(f"{cid}: {c['phase_text']}" + (f"  ERROR {c['error']}" if c["error"] else "")
              + f"  heading {c['gy']}  battery {c['bat']}")
    if args.calib_speed and ok:
        print(f"\nmeasure how far each car really went (planned {args.dist} m), then run:\n"
              f"  python fleet_go.py --set-speed " + ",".join(f"{cid}=<m>" for cid in ids))
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
