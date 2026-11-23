#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WAVE ROVER multi-car trajectory console (fleet_ui)
==================================================

Browser page to give each car its own path, then start them together or
with per-car delays.

    python fleet_ui.py                    # http://localhost:8090
    python fleet_ui.py --host 0.0.0.0     # also reachable from a phone

Architecture: browser <-HTTP-> this program (PC) <-WiFi HTTP-> each car.
One control thread per car (same TrajController as wave_rover_traj.py,
V2 firmware required: turns run on the ESP32).

Start sequence:
  1. every selected car calibrates its gyro bias while standing still (~1 s),
  2. a start gate waits until all of them are ready,
  3. common time zero T0 = gate open + 0.3 s; car i departs at T0 + delay_i.
So the cars depart together (or exactly delay_i apart) regardless of how
long each calibration took.

Lock-step (default, "sync"): every step gets a fixed time slot (straight =
its on-board duration + margin, turn = budget by angle), so each car has a
planned start time for every step. A car that finishes early waits for its
next planned start; a car that overruns its slot (e.g. a slow turn) holds the
plan clock, so the others wait at their next step boundary. Every turn and
every straight therefore starts at the same moment on all cars (or exactly
delay_i apart), however long each car's turn really took. Without lock-step
each car runs its steps back to back and differences pile up.

Paths are step lists, which is exactly what the car executes:
    {"t": "F", "v": 1.0}    drive 1.0 m (negative = backwards)
    {"t": "T", "v": -90}    turn in place, degrees, left positive
Each car's steps live in its own start frame (+x = initial heading, +y = left);
the page places every start frame on a shared map (start x, y, heading).
"""

import argparse
import glob
import json
import math
import os
import re
import subprocess
import threading
import time
import webbrowser
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import urllib.parse
import urllib.request
from urllib.parse import urlparse

import wave_rover_traj as w

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG_FILE = os.path.join(HERE, "fleet_config.json")
HTML_FILE = os.path.join(HERE, "fleet_ui.html")
CARS_FILE = os.path.join(HERE, "cars.txt")

POLL_S = 0.7            # idle status poll period per car
RECONNECT_S = 3.0       # retry period for an offline car
OFFLINE_AFTER = 3       # consecutive failed polls -> offline
GO_MARGIN_S = 0.3       # gate open -> common time zero
GATE_TIMEOUT_S = 20.0   # max wait for the other cars to finish calibrating
# Lock-step slot budgets (keep in sync with slotOf() in fleet_ui.html)
SLOT_STRAIGHT_PAD_S = 0.6               # on-board duration + this
SLOT_TURN_BASE_S = 1.35                 # turn slot = base + per_deg * |deg|
SLOT_TURN_PER_DEG = 0.015               # 90 deg -> 2.7 s
MAX_STEPS = 100
MAX_SEG_M = 20.0
MAX_TURN_DEG = 720.0

PHASE_TEXT = {
    "preparing": "节点准备中", "interrupted": "节点重启，任务中断",
    "offline": "离线", "idle": "空闲", "calib": "测零偏（别碰车）",
    "ready": "已就绪，等其他车", "waiting": "等待出发", "sync": "对齐等待", "turn": "转向中",
    "straight": "直行中", "done": "完成", "stopped": "已停止", "error": "出错",
}


def default_cars():
    """Car list from cars.txt ('car1 192.168.137.229 ...'), else built-in."""
    cars = []
    if os.path.exists(CARS_FILE):
        with open(CARS_FILE, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                parts = line.split()
                if len(parts) >= 2 and parts[1].count(".") == 3:
                    mac = next((p[4:].lower() for p in parts if p.upper().startswith("MAC=")), "")
                    cars.append((parts[0], parts[1], mac))
    if not cars:
        cars = [("car1", "192.168.137.137", "88:f1:55:9c:45:f0"),
                ("car2", "192.168.137.12", "88:f1:55:8f:90:8c")]
    out = []
    for i, (cid, ip, mac) in enumerate(cars):
        out.append({
            "id": cid, "name": car_name(cid, i), "ip": ip, "mac": mac,
            "start": {"x": 0.0, "y": 0.6 - 1.2 * i, "th": 0.0},
            "steps": [{"t": "F", "v": 1.0}], "speed": 0.25, "delay": 0.0,
            "enabled": True,
        })
    return {"cars": out}


def car_name(cid, i):
    """'car7' -> '车7' (falls back to the list position)."""
    digits = "".join(ch for ch in cid if ch.isdigit())
    return f"车{digits or i + 1}"


def load_config():
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r", encoding="utf-8") as f:
                cfg = json.load(f)
            if isinstance(cfg.get("cars"), list) and cfg["cars"]:
                if cfg.get('version') == 2:
                    return cfg
                # Cars registered in cars.txt after the config was created
                # (provision_car.py) are appended with default settings.
                have_ids = {c.get("id") for c in cfg["cars"]}
                have_macs = {norm_mac(c.get("mac")) for c in cfg["cars"] if c.get("mac")}
                n = len(cfg["cars"])
                for c in default_cars()["cars"]:
                    if c["id"] in have_ids or (c["mac"] and norm_mac(c["mac"]) in have_macs):
                        continue
                    c["start"] = {"x": 0.0, "y": round(0.6 - 1.2 * n, 2), "th": 0.0}
                    c["enabled"] = False      # new cars start unticked
                    cfg["cars"].append(c)
                    n += 1
                return cfg
        except (OSError, ValueError) as e:
            print(f"[warn] bad {CONFIG_FILE}, using defaults: {e}")
    return default_cars()


def save_config(cfg):
    tmp = CONFIG_FILE + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(cfg, f, ensure_ascii=False, indent=2)
    os.replace(tmp, CONFIG_FILE)


def make_rover(ip):
    """Factory (tests replace it with a simulated rover)."""
    return w.WifiRover(ip)


def norm_mac(mac):
    return (mac or "").strip().lower().replace("-", ":")


def arp_table():
    """{ip: mac} from the OS ARP cache (Windows/macOS/Linux 'arp -a')."""
    try:
        out = subprocess.run(["arp", "-a"], capture_output=True, text=True,
                             timeout=5, errors="ignore").stdout
    except Exception:
        return {}
    table = {}
    for line in out.splitlines():
        m_ip = re.search(r"(\d+\.\d+\.\d+\.\d+)", line)
        m_mac = re.search(r"([0-9a-fA-F]{2}(?:[-:][0-9a-fA-F]{2}){5})", line)
        if m_ip and m_mac:
            table[m_ip.group(1)] = norm_mac(m_mac.group(1))
    return table


_OPENER = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def probe(ip, timeout=0.8):
    """Read-only V2 status query; returns the T:1010 frame or None."""
    url = f"http://{ip}/js?json=" + urllib.parse.quote('{"T":164}')
    try:
        with _OPENER.open(url, timeout=timeout) as r:
            msg = json.loads(r.read().decode("utf-8", "ignore").strip() or "null")
    except Exception:
        return None
    return msg if isinstance(msg, dict) and msg.get("T") == w.FEEDBACK_TURN else None


def scan_subnets(prefixes):
    """Probe x.y.z.2..254 of every prefix in parallel (about 2-3 s)."""
    ips = [f"{p}.{i}" for p in prefixes for i in range(2, 255)]
    with ThreadPoolExecutor(max_workers=96) as ex:
        res = list(ex.map(probe, ips))
    arp = arp_table()
    return [{"ip": ip, "mac": arp.get(ip, ""), "fw": st.get("fw"), "v": st.get("v")}
            for ip, st in zip(ips, res) if st]


class GateCancelled(Exception):
    """Another car failed before departure; nobody leaves."""


class StartGate:
    """Waits until all cars are calibrated, then fixes the common time zero."""

    def __init__(self, n):
        self.n = n
        self.ready = 0
        self.t_go = None
        self.cancelled = ""
        self.cv = threading.Condition()

    def _maybe_open(self):
        if self.t_go is None and self.n > 0 and self.ready >= self.n:
            self.t_go = time.monotonic() + GO_MARGIN_S
            self.cv.notify_all()

    def arrive_and_wait(self, abort_evt):
        deadline = time.monotonic() + GATE_TIMEOUT_S
        with self.cv:
            self.ready += 1
            self._maybe_open()
            while self.t_go is None and not self.cancelled:
                if abort_evt.is_set():
                    self.ready -= 1
                    self.n -= 1
                    self._maybe_open()
                    raise w.Aborted()
                if time.monotonic() > deadline:
                    self.cancelled = "等待其他车准备超时"
                    self.cv.notify_all()
                    break
                self.cv.wait(0.05)
            if self.cancelled:
                raise GateCancelled(self.cancelled)
            return self.t_go

    def withdraw(self):
        """A car was stopped by the user before arriving: don't wait for it."""
        with self.cv:
            self.n -= 1
            self._maybe_open()

    def cancel(self, reason):
        with self.cv:
            if self.t_go is None and not self.cancelled:
                self.cancelled = reason
                self.cv.notify_all()


class SyncClock:
    """Plan clock of one run (lock-step at step boundaries).

    Plan time starts at 0 at the common time zero T0 and runs with the wall
    clock, but never passes the planned end ("due") of a step that some car
    is still executing. So a car that overruns its slot freezes the plan and
    every other car waits at its next step boundary; when the slow car is
    done, the plan resumes and they all start their next steps together.
    With enabled=False the clock is never held (cars run back to back)."""

    def __init__(self, enabled=True):
        self.enabled = enabled
        self.cv = threading.Condition()
        self.t_last = None           # wall time of the last advance
        self.vt = 0.0                # plan time
        self.busy = {}               # car id -> plan time its current step is due

    def set_t0(self, t0):
        with self.cv:
            if self.t_last is None:
                self.t_last = t0

    def _advance(self, now):
        if self.t_last is None or now <= self.t_last:
            return
        vt = self.vt + (now - self.t_last)
        if self.enabled and self.busy:
            vt = min(vt, max(self.vt, min(self.busy.values())))
        self.vt, self.t_last = vt, now

    def wait_until(self, p, abort_evt):
        """Block until plan time >= p; returns the wall seconds waited."""
        t_start = time.monotonic()
        with self.cv:
            while True:
                self._advance(time.monotonic())
                # held = some car is still on a step that was due by now; the
                # plan clock stops exactly at that due time, which may equal
                # p, so "vt >= p" alone would let this car leave early
                held = self.enabled and any(d <= self.vt + 1e-9 for d in self.busy.values())
                if self.t_last is not None and self.vt >= p - 1e-6 and not held:
                    return time.monotonic() - t_start
                if abort_evt.is_set():
                    raise w.Aborted()
                gap = p - self.vt if self.t_last is not None else 0.02
                self.cv.wait(min(0.02, max(0.002, gap)))

    def begin(self, cid, due):
        with self.cv:
            self._advance(time.monotonic())
            self.busy[cid] = due

    def end(self, cid):
        with self.cv:
            self._advance(time.monotonic())
            self.busy.pop(cid, None)
            self.cv.notify_all()

    def now(self):
        with self.cv:
            self._advance(time.monotonic())
            return self.vt


def fit_speed_model(pts, dz_default):
    """Straight speed model v = k * (pwm - dz) from measured [pwm, v] points.
    Two or more points far enough apart in PWM: the dead zone comes from a
    least-squares line (clamped to 0..0.25); k always goes through the latest
    point, so the speed the car is used at is matched exactly."""
    if len(pts) >= 2:
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        if max(xs) - min(xs) >= 0.005:
            mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
            sxx = sum((x - mx) ** 2 for x in xs)
            a = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
            if a > 0.3:
                dz_default = max(0.0, min(0.25, mx - my / a))
    pwm, v = pts[-1]
    if pwm - dz_default < 0.02:
        return None, dz_default
    return max(0.3, min(30.0, v / (pwm - dz_default))), dz_default


def slot_s(ctrl, step, speed):
    """Lock-step time slot of one step (s)."""
    if step["t"] == "F":
        return ctrl.straight_plan(step["v"], speed)[1] + SLOT_STRAIGHT_PAD_S
    return SLOT_TURN_BASE_S + SLOT_TURN_PER_DEG * abs(step["v"])


class Car:
    def __init__(self, cid, name, ip, mac=""):
        self.id, self.name, self.ip, self.mac = cid, name, ip, norm_mac(mac)
        self.rover = None
        self.ctrl = None
        self.status = {}
        self.conn_err = ""
        self.fail_streak = 0
        self.next_connect = 0.0
        self.log = deque(maxlen=300)
        self.phase = "offline"
        self.error = ""
        self.run = None              # info about the current / last run
        self.last_straight = None    # single-straight run, for --calib-speed
        self.thread = None
        self.lock = threading.Lock()  # guards rover/ctrl replacement
        self.stop_poll = threading.Event()
        self.poller = threading.Thread(target=self._poll_loop, daemon=True)
        self.poller.start()

    # ------------------------------------------------------------ logging --
    def add_log(self, text):
        self.log.append(f"[{time.strftime('%H:%M:%S')}] {text}")

    @property
    def running(self):
        return self.thread is not None and self.thread.is_alive()

    @property
    def connected(self):
        return self.ctrl is not None

    # --------------------------------------------------------- connection --
    def _connect(self):
        try:
            rover = make_rover(self.ip)
        except Exception as e:
            self.conn_err = f"连不上 {self.ip}: {e}"
            return
        try:
            ctrl = w.TrajController(rover, w.load_calib(w.per_car_calib_file(self.ip, self.mac or None)),
                                    log_fn=self.add_log)
        except Exception as e:
            self.conn_err = f"初始化失败: {e}"
            return
        if not ctrl.fw:
            self.conn_err = "车上不是 V2 固件（没有板载转向），本页面不支持"
            try:
                rover.close()
            except Exception:
                pass
            return
        with self.lock:
            self.rover, self.ctrl = rover, ctrl
            self.status = ctrl.fw_last or {}
            self.fail_streak = 0
            self.conn_err = ""
            if self.phase == "offline":
                self.phase = "idle"
        self.add_log(f"已连接 {self.ip}（固件 {self.status.get('fw', '?')}）")

    def _drop(self, reason):
        with self.lock:
            rover = self.rover
            self.rover = self.ctrl = None
            self.status = {}
            self.phase = "offline"
            self.conn_err = reason
        self.add_log(f"断开: {reason}")
        if rover is not None:
            try:
                rover.close()
            except Exception:
                pass

    def set_ip(self, ip):
        if self.running:
            return "这辆车正在跑，先停止"
        if ip != self.ip or not self.connected:
            self.ip = ip
            if self.connected:
                self._drop("改了 IP，重新连接")
            self.next_connect = 0.0
        return ""

    def _poll_loop(self):
        while not self.stop_poll.is_set():
            time.sleep(POLL_S)
            if self.running:
                continue        # the control thread owns the link; use fw_last
            if not self.connected:
                if time.monotonic() >= self.next_connect:
                    self._connect()
                    self.next_connect = time.monotonic() + RECONNECT_S
                continue
            ctrl = self.ctrl
            st = ctrl.fw_status() if ctrl else None
            if st:
                self.status = st
                self.fail_streak = 0
            else:
                self.fail_streak += 1
                if self.fail_streak >= OFFLINE_AFTER and not self.running:
                    self._drop("连续几次没有响应")

    # --------------------------------------------------------------- stop --
    def hard_stop(self):
        """Stop from the HTTP handler thread (control thread may be busy)."""
        ctrl, rover = self.ctrl, self.rover
        if ctrl is not None:
            ctrl.abort.set()
        if rover is None:
            return
        for _ in range(2):
            try:
                rover._request({"T": 161}, timeout=0.3)   # abort on-board turn
            except Exception:
                pass
            try:
                rover.stop()
            except Exception:
                pass
            time.sleep(0.15)

    # --------------------------------------------------------------- view --
    def est_local(self):
        """Estimated pose in the start frame: [x, y, heading_deg].
        Position = plan + progress (on-board straight reports its own
        progress 'sp'); heading = measured gyro heading."""
        r = self.run
        if not r:
            return None
        ctrl = self.ctrl
        st = (ctrl.fw_last if ctrl is not None and ctrl.fw_last else self.status) or {}
        gy = st.get("gy")
        i = r.get("step")
        if i is None:
            return [0.0, 0.0, 0.0 if gy is None else float(gy)]
        fx, fy = r["from"]
        tx, ty = r["to"]
        k = 0.0
        if r.get("kind") == "F":
            sp = st.get("sp") if st.get("ss") == 1 else None
            if r.get("seg_done"):
                k = 1.0
            elif sp is not None:
                k = float(sp)
            else:
                dist = math.hypot(tx - fx, ty - fy)
                t = time.monotonic() - r["seg_t0"]
                v, ramp = r["v"], r["ramp"]
                d = v * t * t / (2 * ramp) if t < ramp else v * (t - ramp / 2)
                k = 0.0 if dist < 1e-6 else d / dist
            k = max(0.0, min(1.0, k))
        elif r.get("kind") == "end":
            k = 1.0
        h = r["hd"] if gy is None else float(gy)
        return [fx + (tx - fx) * k, fy + (ty - fy) * k, h]

    def view(self, now):
        st = self.status
        if self.running and self.ctrl is not None and self.ctrl.fw_last:
            st = self.ctrl.fw_last
        r = self.run
        run = None
        if r:
            run = {
                "start": r["start"], "steps": r["steps"], "step": r.get("step"),
                "nstep": len(r["steps"]), "kind": r.get("kind"),
                "est": self.est_local(), "delay": r["delay"], "trail": r.get("trail", []),
                "t_dep": None if r.get("t_dep") is None else round(r["t_dep"] - now, 2),
                "elapsed": None if r.get("t_dep") is None or now < r["t_dep"]
                else round((r.get("t_end") or now) - r["t_dep"], 1),
            }
        fw_straight = bool(self.ctrl is not None and getattr(self.ctrl, "fw_straight", False))
        return {
            "id": self.id, "name": self.name, "ip": self.ip, "mac": self.mac,
            "connected": self.connected, "conn_err": self.conn_err,
            "phase": self.phase, "phase_text": PHASE_TEXT.get(self.phase, self.phase),
            "error": self.error, "running": self.running,
            "bat": st.get("v"), "gy": st.get("gy"), "gb": st.get("gb"), "gn": st.get("gn"),
            "lp": st.get("lp"), "fw": st.get("fw"), "cal": st.get("cal"),
            "onboard_straight": fw_straight,
            "run": run, "log": list(self.log)[-40:],
        }


class Fleet:
    def __init__(self, cfg):
        self.lock = threading.RLock()
        self.cars = {}
        for c in cfg["cars"]:
            self.cars[c["id"]] = Car(c["id"], c.get("name", c["id"]), c["ip"],
                                     c.get("mac", ""))
        self.gate = None
        self.t_go = None
        self.choreography = None
        self.choreography_abort = threading.Event()
        self.choreography_ids = set()

    def state(self):
        now = time.monotonic()
        gate = self.gate
        go_in = None
        if gate is not None and gate.t_go is not None:
            go_in = round(gate.t_go - now, 2)
        return {
            "running": self.choreography_running or any(c.running for c in self.cars.values()),
            "go_in": go_in,
            "cars": [c.view(now) for c in self.cars.values()],
            "choreography": self.choreography,
        }

    # -------------------------------------------------------------- start --
    @property
    def choreography_running(self):
        return self.choreography is not None and self.choreography["phase"] == "running"

    def start(self, specs, sync=True, choreography=False):
        with self.lock:
            if self.choreography_running and not choreography:
                return "编排运动正在执行"
            if any(c.running for c in self.cars.values()):
                return "还有车在跑，先急停或等它跑完"
            if not specs:
                return "没有选中任何车"
            jobs = []
            for s in specs:
                car = self.cars.get(s.get("id"))
                if car is None:
                    return f"未知车辆 {s.get('id')}"
                if not car.connected:
                    return f"{car.name} 没连上（{car.conn_err or '离线'}）"
                err, job = self._validate(car, s)
                if err:
                    return f"{car.name}: {err}"
                jobs.append((car, job))
            gate = StartGate(len(jobs))
            gate.clock = SyncClock(enabled=sync)
            self.gate = gate
            for car, job in jobs:
                car.error = ""
                car.run = job
                car.phase = "calib"
                car.ctrl.abort.clear()
                car.thread = threading.Thread(target=self._run_car,
                                              args=(car, job, gate), daemon=True)
            for car, _ in jobs:
                car.thread.start()
            names = "、".join(f"{c.name}(+{j['delay']:.1f}s)" for c, j in jobs)
            names += "，每一步对齐" if sync else "，各走各的（不对齐）"
            for car, _ in jobs:
                if not getattr(car.ctrl, "fw_straight", False):
                    car.add_log("注意：车上是旧固件，直行仍由电脑经 WiFi 控制（可能断续）")
            for car, _ in jobs:
                car.add_log(f"=== 启动: {names} ===")
            return ""

    @staticmethod
    def _motion(s):
        try:
            steps = []
            for st in s.get("steps", []):
                t = str(st["t"]).upper()
                v = float(st["v"])
                if t not in ("F", "T") or not math.isfinite(v):
                    raise ValueError
                steps.append({"t": t, "v": v})
            speed = float(s.get("speed", 0.25))
            delay = float(s.get("delay", 0.0))
            st = s.get("start") or {}
            start = {"x": float(st.get("x", 0)), "y": float(st.get("y", 0)),
                     "th": float(st.get("th", 0))}
        except (AttributeError, TypeError, ValueError, KeyError, OverflowError):
            return "步骤/速度/延时格式不对", None
        if not all(math.isfinite(value) for value in start.values()):
            return "起点坐标或朝向无效", None
        steps = [x for x in steps if abs(x["v"]) >= (0.01 if x["t"] == "F" else 0.5)]
        if not steps:
            return "没有任何动作", None
        if len(steps) > MAX_STEPS:
            return f"步骤太多（最多 {MAX_STEPS} 步）", None
        for x in steps:
            if x["t"] == "F" and abs(x["v"]) > MAX_SEG_M:
                return f"单步前进不能超过 {MAX_SEG_M:.0f} m", None
            if x["t"] == "T" and abs(x["v"]) > MAX_TURN_DEG:
                return f"单步转向不能超过 {MAX_TURN_DEG:.0f}°", None
        if not 0.08 <= speed <= 0.45:
            return "速度要在 0.08~0.45 m/s 之间", None
        if not 0.0 <= delay <= 600.0:
            return "延时要在 0~600 s 之间", None
        return "", {"steps": steps, "speed": speed, "delay": delay, "start": start}

    @staticmethod
    def _validate(car, s):
        err, motion = Fleet._motion(s)
        if err:
            return err, None
        steps, speed, delay = (motion[k] for k in ("steps", "speed", "delay"))
        ctrl = car.ctrl
        plan = [delay]
        for x in steps:
            plan.append(plan[-1] + slot_s(ctrl, x, speed))
        return "", {**motion,
                    "v": ctrl.straight_pwm(speed)[1], "plan": plan,
                    "ramp": max(w.LOOP_DT, ctrl.c["fw_ramp"]),
                    "step": None, "kind": None, "from": [0.0, 0.0], "to": [0.0, 0.0],
                    "hd": 0.0, "t_dep": None, "t_end": None, "trail": []}

    def parse_choreography(self, data):
        if not isinstance(data, dict) or data.get("version") != 1:
            raise ValueError("编排文件需要 version: 1")
        starts, stages = data.get("starts"), data.get("stages")
        if not isinstance(starts, dict) or not starts or not isinstance(stages, list) or not stages:
            raise ValueError("编排文件缺少 starts 或 stages")
        poses = {}
        for cid, pose in starts.items():
            if cid not in self.cars:
                raise ValueError(f"未登记的车辆 {cid}")
            if not isinstance(pose, dict) or any(type(pose.get(k)) not in (int, float)
                    or not math.isfinite(pose[k]) for k in ("x", "y", "th")):
                raise ValueError(f"{cid} 的起点需要有限数值 x、y、th")
            if max(abs(pose["x"]), abs(pose["y"])) > MAX_SEG_M * MAX_STEPS:
                raise ValueError(f"{cid} 的起点超出地图坐标范围")
            poses[cid] = {"x": pose["x"], "y": pose["y"], "th": pose["th"] % 360}
        initial = poses.copy()
        result = []
        for index, stage in enumerate(stages, 1):
            if not isinstance(stage, dict) or not isinstance(stage.get("cars"), list) or not stage["cars"]:
                raise ValueError(f"第 {index} 阶段缺少车辆动作")
            jobs = {}
            for spec in stage["cars"]:
                cid = spec.get("id") if isinstance(spec, dict) else None
                if not isinstance(cid, str) or cid not in poses or cid in jobs:
                    raise ValueError(f"第 {index} 阶段存在重复车辆或缺少起点：{cid}")
                err, motion = self._motion({**spec, "start": poses[cid]})
                if err:
                    raise ValueError(f"第 {index} 阶段 {cid}：{err}")
                jobs[cid] = {"id": cid, **motion}
                x, y, th = (poses[cid][key] for key in ("x", "y", "th"))
                for step in motion["steps"]:
                    value = step["v"]
                    if step["t"] == "T":
                        th += value
                    else:
                        x += value * math.cos(math.radians(th))
                        y += value * math.sin(math.radians(th))
                poses[cid] = {"x": x, "y": y, "th": th}
            result.append({"name": str(stage.get("name") or f"阶段 {index}"), "cars": list(jobs.values())})
        return {"version": 1, "name": str(data.get("name") or "编排运动"), "starts": initial, "stages": result}

    def start_choreography(self, data):
        plan = self.parse_choreography(data)
        with self.lock:
            if self.choreography_running or any(c.running for c in self.cars.values()):
                return "已有车辆在运行"
            offline = [cid for cid in plan["starts"] if not self.cars[cid].connected]
            if offline:
                return "以下车辆离线：" + "、".join(offline)
            self.choreography_ids = set(plan["starts"])
            self.choreography_abort.clear()
            run_id = self.choreography["id"] + 1 if self.choreography else 1
            self.choreography = {"id": run_id, "name": plan["name"], "total": len(plan["stages"]),
                                 "stage": 0, "phase": "running", "error": ""}
            threading.Thread(target=self._run_choreography, args=(plan,), daemon=True).start()
        return ""

    def _run_choreography(self, plan):
        abort = self.choreography_abort
        error = ""
        try:
            for index, stage in enumerate(plan["stages"], 1):
                with self.lock:
                    if abort.is_set():
                        break
                    self.choreography = {**self.choreography, "stage": index, "stage_name": stage["name"]}
                    error = self.start(stage["cars"], sync=True, choreography=True)
                    if error:
                        raise RuntimeError(error)
                    cars = [self.cars[job["id"]] for job in stage["cars"]]
                deadline = time.monotonic() + 60 + 2 * max(c.run["plan"][-1] for c in cars)
                while not abort.is_set():
                    failed = [c.name for c in cars if c.phase in ("error", "stopped") or c.error]
                    offline = [cid for cid in self.choreography_ids if not self.cars[cid].connected]
                    if failed or offline:
                        raise RuntimeError("车辆中断或离线：" + "、".join(sorted(set(failed + offline))))
                    if all(c.phase == "done" and not c.running for c in cars):
                        break
                    if time.monotonic() >= deadline:
                        raise RuntimeError("阶段执行超时")
                    abort.wait(0.2)
        except Exception as exc:
            error = str(exc)
        finally:
            with self.lock:
                if error and not abort.is_set():
                    self._stop_cars(self.choreography_ids)
                phase = "stopped" if abort.is_set() else "error" if error else "done"
                self.choreography = {**self.choreography, "phase": phase, "error": error if phase == "error" else ""}

    @staticmethod
    def _trail_loop(car, job):
        """Record the estimated path (start frame) while the car runs, so the
        page can draw the travelled line even after a reload."""
        trail = job["trail"]
        while car.run is job and car.thread is not None and car.thread.is_alive():
            if job.get("step") is not None:
                p = car.est_local()
                if p and (not trail or math.hypot(p[0] - trail[-1][0], p[1] - trail[-1][1]) > 0.01):
                    trail.append([round(p[0], 3), round(p[1], 3)])
            time.sleep(0.1)
        p = car.est_local() if car.run is job else None
        if p and job.get("step") is not None:
            trail.append([round(p[0], 3), round(p[1], 3)])

    def _run_car(self, car, job, gate):
        ctrl = car.ctrl
        arrived = False
        threading.Thread(target=self._trail_loop, args=(car, job), daemon=True).start()
        try:
            car.add_log("测陀螺零偏，车要保持静止…")
            ctrl.fw_calibrate()
            car.phase = "ready"
            arrived = True
            t_go = gate.arrive_and_wait(ctrl.abort)
            clock = gate.clock
            clock.set_t0(t_go)
            job["t_dep"] = t_go + job["delay"]
            car.phase = "waiting"
            clock.wait_until(job["plan"][0], ctrl.abort)
            car.add_log(f"出发（T0+{job['delay']:.1f}s）")
            self._drive(car, ctrl, job, clock)
            car.phase = "done"
            hr = ctrl.fw_heading()
            car.add_log("轨迹跑完 ✔" + (f"（终点陀螺航向 {hr[0]:+.1f}°）" if hr else ""))
        except w.Aborted:
            if not arrived:
                gate.withdraw()
            car.phase = "stopped"
            car.add_log("已急停 ✋")
        except GateCancelled as e:
            car.phase = "stopped"
            car.add_log(f"未出发：{e}")
        except Exception as e:
            gate.cancel(f"{car.name} 出错：{e}")
            car.phase = "error"
            car.error = str(e)
            car.add_log(f"出错: {e}")
            try:
                car.rover.stop()
            except Exception:
                pass
        finally:
            job["t_end"] = time.monotonic()
            gate.clock.end(car.id)          # never hold the others once out

    @staticmethod
    def _drive(car, ctrl, job, clock):
        """Execute the step list. Heading targets are absolute in the gyro
        frame zeroed at calibration, so each turn also absorbs the residual
        error of the previous segment (no accumulation)."""
        x = y = 0.0
        hd = 0.0
        steps = job["steps"]
        plan = job["plan"]
        for i, stp in enumerate(steps):
            ctrl._check_abort()
            if i > 0 and clock.enabled:
                car.phase = "sync"
                waited = clock.wait_until(plan[i], ctrl.abort)
                if waited > 1.0:
                    ctrl.log(f"    [同步] 等其他车 {waited:.1f}s")
            job["step_t0"] = time.monotonic()
            job.setdefault("vt_log", []).append((i, round(clock.now(), 3), job["step_t0"]))
            clock.begin(car.id, plan[i + 1])
            if stp["t"] == "T":
                hd += stp["v"]
                job.update(step=i, kind="T", hd=hd, **{"from": [x, y], "to": [x, y]})
                car.phase = "turn"
                hr = ctrl.fw_heading()
                if hr is None:
                    raise RuntimeError("没有收到陀螺航向，中止")
                delta = hd - hr[0]
                ctrl.log(f"[步 {i + 1}/{len(steps)}] {'左' if stp['v'] > 0 else '右'}转 "
                         f"{abs(stp['v']):.0f}°（实际下发 {delta:+.1f}°）")
                if abs(delta) >= 1.0:
                    ctrl.fw_turn(delta)
                time.sleep(0.15)
            else:
                d = stp["v"]
                nx = x + d * math.cos(math.radians(hd))
                ny = y + d * math.sin(math.radians(hd))
                job.update(step=i, kind="F", hd=hd, seg_t0=time.monotonic(), seg_done=False,
                           **{"from": [x, y], "to": [nx, ny]})
                car.phase = "straight"
                ctrl.log(f"[步 {i + 1}/{len(steps)}] {'前进' if d > 0 else '后退'} {abs(d):.2f} m")
                ctrl.drive_straight(d, speed_ms=job["speed"], heading_deg=hd)
                job["seg_done"] = True
                time.sleep(0.15)
                x, y = nx, ny
                if len(steps) == 1:
                    pwm, v = ctrl.straight_pwm(job["speed"])
                    car.last_straight = {"pwm": pwm, "v": v, "dist": abs(d),
                                         "speed": job["speed"], "t": time.time()}
            used = time.monotonic() - job["step_t0"]
            over = used - (plan[i + 1] - plan[i])
            clock.end(car.id)
            if clock.enabled and over > 0.3:
                ctrl.log(f"    [同步] 这一步比计划多用 {over:.1f}s，其他车会等它")
        job.update(step=len(steps), kind="end", hd=hd, **{"from": [x, y], "to": [x, y]})

    # --------------------------------------------------------------- scan --
    def scan(self):
        """Find V2 cars on the car subnets; re-point cars whose MAC moved."""
        prefixes = sorted({c.ip.rsplit(".", 1)[0] for c in self.cars.values()}
                          | {"192.168.137"})
        found = scan_subnets(prefixes)
        changes = []
        for car in self.cars.values():
            if not car.mac:
                continue
            hit = next((f for f in found if f["mac"] == car.mac), None)
            if hit and hit["ip"] != car.ip and not car.running:
                old = car.ip
                car.set_ip(hit["ip"])
                car.add_log(f"扫描：按 MAC 找到本车新 IP {old} → {hit['ip']}")
                changes.append({"id": car.id, "name": car.name, "old": old, "ip": hit["ip"]})
        if changes:
            with CONFIG_LOCK:
                cfg = load_config()
                for c in cfg["cars"]:
                    for ch in changes:
                        if c.get("id") == ch["id"]:
                            c["ip"] = ch["ip"]
                save_config(cfg)
        known = {c.mac: c.name for c in self.cars.values() if c.mac}
        for f in found:
            f["car"] = known.get(f["mac"], "")
        return {"found": found, "changes": changes}

    def auto_scan_later(self, delay=5.0, period=15.0):
        """Keep re-scanning while some car is offline (it may have rejoined the
        hotspot with a new IP); skipped while any car is running."""
        def job():
            time.sleep(delay)
            while True:
                try:
                    if (any(not c.connected for c in self.cars.values())
                            and not any(c.running for c in self.cars.values())):
                        res = self.scan()
                        if res["changes"]:
                            print(f"[fleet_ui] auto scan: re-pointed {res['changes']}")
                except Exception as e:  # never let the scanner thread die
                    print(f"[fleet_ui] auto scan failed: {e}")
                time.sleep(period)
        threading.Thread(target=job, daemon=True).start()

    # ------------------------------------------------------ speed calib --
    def calib_speed(self, cid, measured):
        """Fit this car's straight speed model from the distance it really
        covered in the last single-straight run (fleet_go.py --calib-speed)."""
        car = self.cars.get(cid)
        if car is None:
            return {"error": f"未知车辆 {cid}"}
        ls, ctrl = car.last_straight, car.ctrl
        if not ls:
            return {"error": f"{car.name} 还没跑过单段直行（先运行 --calib-speed）"}
        if ctrl is None:
            return {"error": f"{car.name} 没连上"}
        try:
            measured = float(measured)
        except (TypeError, ValueError):
            return {"error": "距离格式不对"}
        if not 0.2 * ls["dist"] <= measured <= 3.0 * ls["dist"]:
            return {"error": f"{car.name} 量到 {measured} m，和计划 {ls['dist']} m 差太多，请检查"}
        # the cruise lasted dist / v (by construction of the duration)
        v_meas = ls["v"] * measured / ls["dist"]
        pts = [p for p in (ctrl.c.get("sv_pts") or []) if abs(p[0] - ls["pwm"]) > 1e-4]
        pts = (pts + [[round(ls["pwm"], 4), round(v_meas, 4)]])[-4:]
        k, dz = fit_speed_model(pts, float(ctrl.c.get("sv_dz") or 0.15))
        if k is None:
            return {"error": "这次的 PWM 太接近死区，速度调高一点再标定"}
        ctrl.c["sv_k"], ctrl.c["sv_dz"], ctrl.c["sv_pts"] = k, dz, pts
        path = w.per_car_calib_file(car.ip, car.mac or None)
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(ctrl.c, f, ensure_ascii=False, indent=2)
        except OSError as e:
            return {"error": f"保存失败: {e}"}
        new_pwm = ctrl.straight_pwm(ls["speed"])[0]
        car.last_straight = None             # one measurement per run
        car.add_log(f"[速度标定] 计划 {ls['dist']:.2f} m 实际 {measured:.2f} m → "
                    f"PWM {ls['pwm']:.3f} 时 {v_meas:.3f} m/s；以后 {ls['speed']} m/s 用 PWM {new_pwm:.3f}")
        return {"error": "", "v_meas": round(v_meas, 4), "sv_k": round(k, 4),
                "old_pwm": round(ls["pwm"], 4), "new_pwm": round(new_pwm, 4),
                "file": os.path.basename(path)}

    # --------------------------------------------------------------- stop --
    def stop(self, ids=None):
        with self.lock:
            if self.choreography_running and (ids is None or self.choreography_ids.intersection(ids)):
                self.choreography_abort.set()
                ids = None if ids is None else self.choreography_ids
            return self._stop_cars(ids)

    def _stop_cars(self, ids):
        cars = [c for c in self.cars.values() if ids is None or c.id in ids]
        for c in cars:
            if c.ctrl is not None:
                c.ctrl.abort.set()
        threads = [threading.Thread(target=c.hard_stop, daemon=True) for c in cars]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=1.5)
        return ""


FLEET = None
CONFIG_LOCK = threading.Lock()


def list_plans():
    out = []
    for path in sorted(glob.glob(os.path.join(HERE, "plan_*.json"))):
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            wps = data.get("waypoints") if isinstance(data, dict) else data
            pts = [[float(p[0]), float(p[1])] for p in wps]
        except (OSError, ValueError, TypeError, IndexError):
            continue
        if pts and math.hypot(*pts[0]) < 1e-6:
            pts = pts[1:]
        out.append({"name": os.path.basename(path), "points": pts})
    return out


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def _send(self, code, body, ctype="application/json; charset=utf-8"):
        if isinstance(body, (dict, list)):
            body = json.dumps(body, ensure_ascii=False).encode("utf-8")
        elif isinstance(body, str):
            body = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _body(self):
        n = int(self.headers.get("Content-Length") or 0)
        if n <= 0:
            return {}
        try:
            return json.loads(self.rfile.read(n).decode("utf-8"))
        except ValueError:
            return {}

    def do_GET(self):
        path = urlparse(self.path).path
        if path in ("/", "/index.html"):
            with open(HTML_FILE, "rb") as f:
                self._send(200, f.read(), "text/html; charset=utf-8")
        elif path == "/api/state":
            self._send(200, FLEET.state())
        elif path == "/api/config":
            with CONFIG_LOCK:
                self._send(200, load_config())
        elif path == "/api/plans":
            self._send(200, list_plans())
        else:
            self._send(404, {"error": "not found"})

    def do_POST(self):
        path = urlparse(self.path).path
        data = self._body()
        if path == "/api/stop":
            ids = data.get("ids")
            self._send(200, {"error": FLEET.stop(ids)})
        elif path == "/api/start":
            self._send(200, {"error": FLEET.start(data.get("cars") or [],
                                                  sync=bool(data.get("sync", True)))})
        elif path in ("/api/choreography/validate", "/api/choreography/start"):
            try:
                if path.endswith("/validate"):
                    result = {"plan": FLEET.parse_choreography(data)}
                else:
                    result = {"error": FLEET.start_choreography(data)}
            except (ValueError, OverflowError) as exc:
                result = {"error": str(exc)}
            self._send(200, result)
        elif path == "/api/calib_speed":
            self._send(200, FLEET.calib_speed(data.get("id"), data.get("measured")))
        elif path == "/api/config":
            cars = data.get("cars")
            if not isinstance(cars, list) or not cars:
                self._send(400, {"error": "bad config"})
                return
            with CONFIG_LOCK:
                save_config({"version": 2, "cars": cars})
            self._send(200, {"error": ""})
        elif path == "/api/scan":
            self._send(200, FLEET.scan())
        elif path == "/api/car":
            car = FLEET.cars.get(data.get("id"))
            ip = str(data.get("ip", "")).strip()
            if car is None or not ip or ip.startswith('-') or any(x.isspace() for x in ip):
                self._send(400, {"error": "车编号或 IP 不对"})
                return
            self._send(200, {"error": car.set_ip(ip)})
        else:
            self._send(404, {"error": "not found"})


def main():
    global FLEET, CONFIG_FILE
    ap = argparse.ArgumentParser(description="WAVE ROVER multi-car trajectory console")
    ap.add_argument("--host", default="127.0.0.1",
                    help="listen address (0.0.0.0 = also from phone/LAN)")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--no-browser", action="store_true", help="don't open a browser")
    ap.add_argument('--config', default=os.path.join(HERE, 'fleet_nodes.json'))
    args = ap.parse_args()
    CONFIG_FILE = args.config
    if not os.path.exists(CONFIG_FILE):
        ap.error('node config missing; copy fleet_nodes.json and register Pi nodes')
    cfg = load_config()
    if not os.path.exists(CONFIG_FILE):
        save_config(cfg)
    from node_fleet import NodeFleet
    FLEET = NodeFleet(cfg)
    FLEET.auto_scan_later()
    srv = ThreadingHTTPServer((args.host, args.port), Handler)
    url = f"http://localhost:{args.port}/"
    print(f"[fleet_ui] serving {url}  (cars: "
          + ", ".join(f"{c.name}={c.ip}" for c in FLEET.cars.values()) + ")")
    print("[fleet_ui] 退出控制台后，已启动任务仍由树莓派继续；停车请使用明确的停止命令")
    if not args.no_browser:
        threading.Timer(1.0, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        FLEET.close()
        srv.server_close()


if __name__ == "__main__":
    main()
