# -*- coding: utf-8 -*-
"""Straight-line diagnostic for v2.3 (T:165): out 1 m, turn 180, back 1 m.
Logs heading error over the segment and the yaw after the stop (braking).
Usage: python diag_straight.py <ip> [speed] [dist]"""
import sys
import time
import wave_rover_traj as w

ip = sys.argv[1]
speed = float(sys.argv[2]) if len(sys.argv) > 2 else 0.18
dist = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
rover = w.WifiRover(ip)
ctrl = w.TrajController(rover, w.load_calib(w.per_car_calib_file(ip)), log_fn=print)
assert ctrl.fw_straight, "car is not on v2.3"
ctrl.fw_calibrate()
c = ctrl.c


def leg(target_hd):
    hr = ctrl.fw_heading()
    hd = hr[0] + w.wrap_deg(target_hd - hr[0])
    base = max(speed / c["speed_gain"], c["min_pwm"])
    dur = dist / (base * c["speed_gain"]) + c["fw_ramp"] * (1 - c["fw_ramp0"]) / 2
    st = ctrl._fw_cmd({"T": 165, "ms": int(dur * 1000), "spd": round(base, 4),
                       "hd": round(hd, 2), "ramp": int(c["fw_ramp"] * 1000)})
    t0 = time.monotonic()
    rows = []
    while True:
        st = ctrl.fw_status()
        if st:
            rows.append((time.monotonic() - t0, st.get("sp"), st.get("se"), st.get("gy"),
                         st.get("gr"), st.get("ss")))
            if st.get("ss") != 1:
                break
        time.sleep(0.04)
    t_stop = time.monotonic()
    post = []
    while time.monotonic() - t_stop < 1.0:
        st = ctrl.fw_status()
        if st:
            post.append((time.monotonic() - t0, st.get("gy"), st.get("gr")))
        time.sleep(0.05)
    print(f"\n-- leg hd={hd:+.1f} pwm={base:.3f} dur={dur:.2f}s  end: ss={st.get('ss')} "
          f"mean|err|={st.get('sa')} max|err|={st.get('sm')}")
    print("   t(s)  prog   err(deg)  gyro(deg)  rate(dps)")
    step = max(1, len(rows) // 25)
    for r in rows[::step] + rows[-1:]:
        print(f"  {r[0]:5.2f}  {r[1] or 0:5.2f}  {r[2] or 0:+7.1f}  {r[3]:+8.2f}  {r[4]:+7.1f}")
    gy_stop = rows[-1][3]
    gy_end = post[-1][1] if post else gy_stop
    print(f"   heading at stop {gy_stop:+.2f}, 1 s later {gy_end:+.2f} -> "
          f"post-stop yaw {gy_end - gy_stop:+.2f} deg; final error vs target {w.wrap_deg(hd - gy_end):+.2f}")
    return hd


hd = leg(0.0)
hr = ctrl.fw_heading()
ctrl.fw_turn(w.wrap_deg(180.0 - hr[0]) if False else (180.0 + 0.0) - (hr[0] - 0.0))
time.sleep(0.2)
leg(180.0)
rover.stop()
