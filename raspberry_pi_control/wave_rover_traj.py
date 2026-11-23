#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WAVE ROVER (微雪) 轨迹自动行驶程序
================================

原理
----
WAVE ROVER 的下位机是一块 ESP32 驱动板，JSON 指令可以通过 USB 串口或 WiFi(HTTP) 下达：
    {"T":1,"L":0.3,"R":0.3}      左/右轮速度，范围 -0.5~+0.5，正值前进
    {"T":130}                    请求底盘反馈（HTTP 模式下响应体直接返回该帧）
    {"T":1001,...,"y":航向角,...}  反馈帧：y=航向角(度), v=电池电压

本车的电机没有编码器，左右轮指令本质是开环 PWM，所以：
  * 距离  → 用标定出的速度(m/s) × 时间 推算（航位推算，误差约 5~10%）
  * 航向  → 用板载 IMU 的航向角做闭环（这是走直线不歪的关键）
  * 固件有 3 秒心跳超时，本程序以 20 Hz 持续发指令，兼作安全保底

两种连接方式（二选一）
----
  USB 串口（默认，需要拆车接驱动板中间的 USB-C 口）:
      python wave_rover_traj.py COM5 demo square
  WiFi（不用拆车！）—— 连上小车开机自动创建的热点（名称/密码见 OLED 或说明书），
  热点模式下小车 IP 固定为 192.168.4.1；若已配置连家里路由器(STA)，用 OLED 上显示的 IP:
      python wave_rover_traj.py --ip 192.168.4.1 demo square
  让小车连家里路由器（电脑和小车同局域网，电脑不用断外网）:
      python wave_rover_traj.py --ip 192.168.4.1 wifi-setup --ssid WiFi名 --password 密码

标定流程
----
  python wave_rover_traj.py [--ip IP | COM5] status         # 检查通信、看电池/航向
  python wave_rover_traj.py [--ip IP | COM5] calib-yaw       # 自动判断航向角方向（一次）
  python wave_rover_traj.py [--ip IP | COM5] calib-speed     # 标定直线速度（量一次距离）
  python wave_rover_traj.py [--ip IP | COM5] calib-turn      # 标定原地转向角速度（自动用IMU）

轨迹
----
  python wave_rover_traj.py [--ip IP | COM5] waypoints plan.json
  python wave_rover_traj.py [--ip IP | COM5] demo square|circle|figure8
  python wave_rover_traj.py [--ip IP | COM5] stop            # 紧急停车

标定结果保存在同目录 rover_calib.json，路径点文件格式见 README。
"""

import argparse
import json
import math
import os
import sys
import threading
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

try:
    import urllib.request
    import urllib.parse
except ImportError:   # pragma: no cover
    urllib = None

# ----------------------------------------------------------------------------
# 常量 / 默认参数
# ----------------------------------------------------------------------------
BAUD              = 115200
LOOP_DT           = 0.05     # 控制周期 20 Hz（> 心跳 3s 要求）
FEEDBACK_BASE     = 1001     # 底盘反馈帧类型（固件 FEEDBACK_BASE_INFO）
FEEDBACK_IMU      = 1002     # IMU 反馈帧类型（固件 FEEDBACK_IMU_DATA）
FEEDBACK_TURN     = 1010     # V2 固件：陀螺航向 + 板载转向状态帧（T:164 / T:1 带 fb）

# V2 固件板载转向状态（turn_logic.h TurnState）
TS_IDLE, TS_RUNNING, TS_DONE, TS_ABORTED, TS_TIMEOUT, TS_STALLED = range(6)
TS_NAMES = {TS_IDLE: "idle", TS_RUNNING: "running", TS_DONE: "done",
            TS_ABORTED: "aborted", TS_TIMEOUT: "timeout", TS_STALLED: "stalled"}

CALIB_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "rover_calib.json")

# 未标定时的缺省值（标定后会覆盖）
DEFAULT_CALIB = {
    "speed_gain": 2.2,       # (m/s) / 轮指令单位 —— 0.5(满PWM) 时约 1.1 m/s
    "turn_gain": 45.0,       # (度/s) / 单位差速(R-L) —— 实车标定值（静止净位移法）
    "yaw_sign": 1,          # +1: 左转时 IMU 航向角增大；-1: 相反（calib-yaw 自动判定）
    "gyro_scale": 1.11,     # 陀螺仪积分缩放（实测: 程序积分360°时实车转约400°）
    "wheel_track": 0.172,    # 左右轮距 (m)，默认取固件值，建议实测
    "min_pwm": 0.08,         # 低于该 PWM 电机可能不转（死区补偿，直行修正用）
    "turn_min_pwm": 0.21,    # 原地转向的 PWM 底线。实测死区≈0.19：低于此值
                             # 转不动；0.20~0.21 实测净转速 15~25 度/s
    "kp_turn": 1.2,          # 转向环 P 增益 (deg 误差 → deg/s 速度指令)
    "kd_turn": 1.2,          # 角速度阻尼 (deg/s 反馈 → deg/s 指令)
    "max_pwm": 0.5,          # 指令上限
    "kp_heading": 3.0,       # 直行/曲线航向环 P 增益
    "tol_heading": 4.0,      # 转向到位判定阈值 (度)
    "settle_time": 0.3,      # 转向到位后保持时间 (s)
    "stall_watchdog": 4.0,   # 转向误差连续 N 秒无改善 → 判定异常并报错 (s)
    "ramp_time": 0.6,        # 启动/停止 加减速时间 (s)
    "resync_settle": 1.2,    # 停车后等 AHRS 磁修正收敛的时间 (s)——电机电流会
                             # 干扰磁力计，转动中的 yaw 不可信，必须停车静止后再读
    # ---- V2 固件（板载陀螺航向 + 板载闭环转向）参数 ----
    "fw_gyro_scale": 1.0,    # 板载陀螺刻度修正（calib-gs 标定后写入）
    "fw_turn_pwm": 0.25,     # 板载转向巡航 PWM（实测 0.35 即达 ~190°/s，不宜过大）
    "fw_kick": 0.40,         # 板载转向起步冲击 PWM（克服静摩擦）
    "fw_min_pwm": 0.22,      # 板载转向接近目标时的最低 PWM（略高于死区）
    # ---- V2.3 固件（板载直行航向保持 T:165）----
    "fw_ramp": 0.3,          # straight soft-start time (s); v2.4.2 used 0.5
    "fw_ramp0": 0.8,         # soft-start begins at this fraction of cruise PWM; v2.4.2
                             # used 0.4, i.e. below the dead zone: each car started
                             # moving at a different moment of the ramp
    "fw_still_noise": 2.5,   # dps; gyro std above this during bias calib = car was moving
    "fw_skp": 0.020,         # on-board straight heading P gain (PWM/deg); v2.3 used 0.010
    "fw_ski": 0.012,         # on-board straight heading I gain (PWM/deg/s); v2.3 used 0.006
    # ---- V2.4.3: turn-rate floor (cars with a high dead zone used to crawl) ----
    "fw_turn_rate": 60.0,    # dps the on-board turn tries to keep at least
    "fw_turn_rate_min": 20.0,  # dps floor near the target
    # ---- per-car straight speed model (fleet_go.py --calib-speed) ----
    # Without the model: pwm = speed / speed_gain (proportional, shared by all
    # cars). With it: pwm = sv_dz + speed / sv_k, fitted from one measured run
    # per car, because the speed is roughly proportional to (pwm - dead zone)
    # and the dead zone differs between cars.
    "sv_dz": 0.15,           # assumed straight dead zone (PWM)
    "sv_k": None,            # m/s per PWM above sv_dz (None = not calibrated)
}


# Per-car calibration: rover_calib_<mac>.json (MAC without colons, lower case;
# the hotspot hands out a different IP after reboots). Falls back to the shared
# rover_calib.json for reading; saves always go to the per-car file.
CALIB_SAVE_FILE = CALIB_FILE
CARS_FILE = os.path.join(os.path.dirname(CALIB_FILE), "cars.txt")


def mac_key(mac: str) -> str:
    return "".join(c for c in (mac or "").lower() if c in "0123456789abcdef")


def car_mac_for_ip(ip: str) -> str:
    """MAC of the car at ip: from its status frame (fw v2.4.2+), else cars.txt."""
    try:
        url = f"http://{ip}/js?json=" + urllib.parse.quote(json.dumps({"T": 164}))
        with urllib.request.build_opener(urllib.request.ProxyHandler({})).open(url, timeout=1.5) as r:
            mac = json.loads(r.read() or b"{}").get("mac", "")
        if len(mac_key(mac)) == 12:
            return mac
    except Exception:
        pass
    if os.path.exists(CARS_FILE):
        with open(CARS_FILE, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                parts = line.split()
                if len(parts) >= 2 and parts[1] == ip:
                    for p in parts:
                        if p.upper().startswith("MAC="):
                            return p[4:]
    return ""


def per_car_calib_file(ip: str, mac: str = None) -> str:
    """Calibration file for a car. Keyed by MAC when known; an older
    IP-keyed file (rover_calib_<ip>.json) is migrated to the MAC name once."""
    d = os.path.dirname(CALIB_FILE)
    ip_file = os.path.join(d, f"rover_calib_{ip}.json")
    if mac is None:
        mac = car_mac_for_ip(ip)
    key = mac_key(mac)
    if len(key) != 12:
        return ip_file
    mac_file = os.path.join(d, f"rover_calib_{key}.json")
    if not os.path.exists(mac_file) and os.path.exists(ip_file):
        try:
            with open(ip_file, "r", encoding="utf-8") as f:
                data = f.read()
            json.loads(data)
            with open(mac_file, "w", encoding="utf-8") as f:
                f.write(data)
            print(f"[标定] 已把 {os.path.basename(ip_file)} 迁移为 {os.path.basename(mac_file)}")
        except (OSError, ValueError):
            pass
    return mac_file


def load_calib(path: str = None) -> dict:
    global CALIB_SAVE_FILE
    if path:
        CALIB_SAVE_FILE = path
    calib = dict(DEFAULT_CALIB)
    src = CALIB_SAVE_FILE if os.path.exists(CALIB_SAVE_FILE) else CALIB_FILE
    if os.path.exists(src):
        try:
            with open(src, "r", encoding="utf-8") as f:
                calib.update(json.load(f))
        except (OSError, ValueError) as e:
            print(f"[警告] 标定文件损坏，使用默认值: {e}")
    print(f"[标定] 读取 {os.path.basename(src)}"
          + ("" if src == CALIB_SAVE_FILE else f"（保存时写入 {os.path.basename(CALIB_SAVE_FILE)}）"))
    return calib


def save_calib(calib: dict):
    with open(CALIB_SAVE_FILE, "w", encoding="utf-8") as f:
        json.dump(calib, f, ensure_ascii=False, indent=2)
    print(f"[标定] 已保存到 {CALIB_SAVE_FILE}")


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Aborted(Exception):
    """用户请求中止（网页 Stop 键 / Ctrl+C）"""


def wrap_deg(a: float) -> float:
    """把角度差归一化到 [-180, 180)"""
    return (a + 180.0) % 360.0 - 180.0


# ----------------------------------------------------------------------------
# 小车通信客户端
# ----------------------------------------------------------------------------
class WaveRover:
    """封装与 ESP32 下位机的串口 JSON 通信 + 最新反馈缓存"""

    def __init__(self, port: str):
        # 与官方 serial_simple_ctrl.py 保持一致：不拉 RTS/DTR，避免复位 ESP32
        self.ser = serial.Serial(port, baudrate=BAUD, dsrdtr=None, timeout=0.05)
        self.ser.setRTS(False)
        self.ser.setDTR(False)
        self.lock = threading.Lock()
        self.feedback = {}          # 最新一帧 {"T":1001,...}
        self.feedback_time = 0.0
        self.imu_frame = {}         # 最新一帧 {"T":1002,...}（含原始陀螺仪 gz）
        self.imu_time = 0.0
        self.frames = {}            # 任意类型的最新反馈帧 {T: frame}
        self.frame_time = {}        # {T: 收到时刻}
        self.rx_count = 0
        self._alive = True
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        # 连续反馈限速到 50 ms/帧：不限速时固件每圈 loop() 都打印一帧，
        # 115200 波特率下会把主循环拖慢到 ~20 ms（板载转向控制环也跟着变慢）
        self.send({"T": 142, "cmd": 50})
        # 开启连续反馈：底盘会持续回传 T=1001 帧（含航向角）
        self.send({"T": 131, "cmd": 1})
        time.sleep(0.2)
        if self.rx_count == 0:
            print(f"[警告] 已连接 {port}，但 0.2 秒内没收到任何反馈帧。"
                  f"请确认下位机已开机、端口正确。")

    # ---------- 底层 ----------
    def send(self, obj: dict):
        data = (json.dumps(obj, separators=(",", ":")) + "\n").encode("utf-8")
        with self.lock:
            self.ser.write(data)

    def _read_loop(self):
        buf = b""
        while self._alive:
            try:
                chunk = self.ser.read(256)
            except (serial.SerialException, OSError):
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                line = line.strip()
                if not line:
                    continue
                try:
                    msg = json.loads(line.decode("utf-8"))
                except (ValueError, UnicodeDecodeError):
                    continue
                if isinstance(msg, dict) and "T" in msg:
                    self.frames[msg["T"]] = msg
                    self.frame_time[msg["T"]] = time.monotonic()
                if isinstance(msg, dict) and msg.get("T") in (FEEDBACK_BASE, FEEDBACK_IMU):
                    # 底盘帧含 y(航向)+v(电压)；IMU 帧额外提供原始陀螺仪 gz
                    # （电机电流会干扰磁力计，转动中的 y 不可信，gz 才可信）
                    if msg.get("T") == FEEDBACK_BASE:
                        self.feedback = msg
                        self.feedback_time = time.monotonic()
                    else:
                        self.imu_frame = msg
                        self.imu_time = time.monotonic()
                self.rx_count += 1

    # ---------- 状态 ----------
    def wait_feedback(self, timeout=2.0) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if self.feedback:
                return True
            time.sleep(0.05)
        return False

    @property
    def yaw_deg(self):
        """IMU 航向角（原始，未乘 yaw_sign）"""
        return self.feedback.get("y")

    @property
    def battery_v(self):
        return self.feedback.get("v")

    def imu(self):
        """主动要一帧 IMU 数据 {"T":1002, "y":.., "gz":..}（gz 为原始陀螺仪，
        单位 度/s，不受电机磁场干扰；y 仅在静止时可信）"""
        t_req = time.monotonic()
        self.send({"T": 126})
        t0 = time.monotonic()
        while time.monotonic() - t0 < 0.2:
            if self.imu_time > t_req and self.imu_frame:
                return self.imu_frame
            time.sleep(0.01)
        return self.imu_frame or None

    def query(self, obj: dict, want_t: int, timeout: float = 0.3):
        """发一条指令并等待指定类型的回帧（串口回帧是异步的）"""
        t_req = time.monotonic()
        self.send(obj)
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if self.frame_time.get(want_t, 0.0) > t_req:
                return self.frames[want_t]
            time.sleep(0.005)
        return None

    def fw_status(self):
        """V2 固件状态帧 {"T":1010,"gy":陀螺航向,"gr":角速度,"ts":转向状态,...}"""
        return self.query({"T": 164}, FEEDBACK_TURN)

    # ---------- 运动 ----------
    def set_wheels(self, left: float, right: float, fb: bool = False):
        """左右轮速度指令，范围 [-0.5, 0.5]，正值前进。
        每条指令自带 ms=600 的"租期"：补丁固件到点自动刹车（独立看门狗
        执行，主循环冻结也生效）；原版固件不认识 ms 字段，直接忽略、无副作用。
        fb=True（V2 固件）：同时请求状态帧；串口模式下返回最近一帧（异步）。"""
        left = clamp(left, -DEFAULT_CALIB["max_pwm"], DEFAULT_CALIB["max_pwm"])
        right = clamp(right, -DEFAULT_CALIB["max_pwm"], DEFAULT_CALIB["max_pwm"])
        cmd = {"T": 1, "L": round(left, 4), "R": round(right, 4), "ms": 600}
        if fb:
            cmd["fb"] = 1
        self.send(cmd)
        return self.frames.get(FEEDBACK_TURN) if fb else None

    def stop(self):
        self.set_wheels(0.0, 0.0)

    def close(self):
        try:
            self.stop()
            self.send({"T": 131, "cmd": 0})   # 关闭连续反馈
        except Exception:
            pass
        self._alive = False
        time.sleep(0.1)
        try:
            self.ser.close()
        except Exception:
            pass


# ----------------------------------------------------------------------------
# WiFi 通信客户端（与 WaveRover 同接口，可互换）
# ----------------------------------------------------------------------------
class WifiRover:
    """
    通过 ESP32 的 HTTP 接口通信（无需拆车接 USB 线）：
        GET http://<ip>/js?json=<URL编码的JSON指令>
    固件源码 http_server.h: 指令会被执行，且响应体返回该指令产生的反馈帧。
    因此取航向角 = 发一条 {"T":130}，读响应里的 "y" 字段。
    """

    def __init__(self, ip: str):
        self.base = f"http://{ip}/js?json="
        self.lock = threading.Lock()
        self.feedback = {}
        self._fail_streak = 0
        self._fast_stop_on = False
        # 强制直连，无视系统代理（电脑开代理时，局域网请求不能被代理劫走）
        self._opener = urllib.request.build_opener(
            urllib.request.ProxyHandler({}))
        # 连接自检
        if not self._request({"T": 130}):
            raise ConnectionError(
                f"无法连接小车 {ip}。请确认：1) 电脑已连上小车的 WiFi 热点 "
                f"(名称见 OLED 屏幕)；2) 热点模式 IP 是 192.168.4.1，"
                f"路由器模式用 OLED 上显示的 IP。")
        print(f"[WiFi] 已连接小车 http://{ip}/")
        # 收紧固件心跳 (T:136)：400ms 内没有新运动指令就自动停车。
        # - 原版固件：跑在主循环里，仅防 PC 端程序崩溃（主循环卡死时无效）
        # - 补丁固件：跑在独立看门狗任务里，主循环卡死时电机最多失控 0.4s
        # - 每条 T:1 还自带 ms=600 租期（见 set_wheels），双重保险
        # 心跳是 RAM 变量，小车断电重启后自动恢复官方默认 3000ms（不影响官方网页遥控）
        try:
            self._request({"T": 136, "cmd": 400})
        except Exception:
            pass

    def fast_stop(self):
        """固件卡死时的急停：无间隔连发停车指令（最长 ~60s）。
        固件恢复循环的第一瞬间就会被停车——电机干扰冻结固件时，
        心跳保护也一起冻结，电机保持最后指令狂转，这是唯一补救。"""
        if self._fast_stop_on:
            return False
        self._fast_stop_on = True
        try:
            url = self.base + urllib.parse.quote(
                json.dumps({"T": 1, "L": 0, "R": 0}, separators=(",", ":")))
            for _ in range(200):
                try:
                    with self.lock:
                        with self._opener.open(url, timeout=0.3) as resp:
                            resp.read()
                    return True
                except Exception:
                    time.sleep(0.05)
            return False
        finally:
            self._fast_stop_on = False

    def _request(self, obj: dict, timeout=0.5):
        """发一条 JSON 指令，返回响应体解析出的 dict（可能为空）。
        指令都是幂等 GET。快速失败（2次即抛 ConnectionError）：
        固件 HTTP 会被电机 I2C 干扰间歇性冻结（ping 仍通），快速失败
        让上层能尽早触发 fast_stop 连发急停。"""
        url = self.base + urllib.parse.quote(
            json.dumps(obj, separators=(",", ":")))
        last_err = None
        for attempt in range(2):
            try:
                with self.lock:
                    with self._opener.open(url,
                                            timeout=timeout + 0.5 * attempt) as resp:
                        body = resp.read().decode("utf-8").strip()
                self._fail_streak = 0
                break
            except Exception as e:
                last_err = e
                self._fail_streak += 1
                if attempt < 1:
                    time.sleep(0.3)
        else:
            raise ConnectionError(f"请求失败: {last_err}")
        if body:
            try:
                msg = json.loads(body)
                if isinstance(msg, dict) and msg.get("T") in (FEEDBACK_BASE,
                                                              FEEDBACK_IMU):
                    self.feedback = msg
                return msg
            except ValueError:
                pass
        return None

    # ---------- 与 WaveRover 相同的接口 ----------
    def send(self, obj: dict):
        self._request(obj)

    def query(self, obj: dict, want_t: int = None, timeout: float = 0.5):
        """发一条指令，返回响应体里的回帧（HTTP 模式是同步的）"""
        msg = self._request(obj, timeout=timeout)
        if want_t is not None and (not isinstance(msg, dict) or msg.get("T") != want_t):
            return None
        return msg

    def fw_status(self):
        """V2 固件状态帧 {"T":1010,"gy":陀螺航向,"gr":角速度,"ts":转向状态,...}"""
        return self.query({"T": 164}, FEEDBACK_TURN)

    def set_wheels(self, left: float, right: float, fb: bool = False):
        # ms=600: 指令自带租期，到点自动刹车（见 WaveRover.set_wheels 注释）
        # fb=True（V2 固件）：响应体直接带回状态帧，省一次 HTTP 往返
        left = clamp(left, -DEFAULT_CALIB["max_pwm"], DEFAULT_CALIB["max_pwm"])
        right = clamp(right, -DEFAULT_CALIB["max_pwm"], DEFAULT_CALIB["max_pwm"])
        cmd = {"T": 1, "L": round(left, 4), "R": round(right, 4), "ms": 600}
        if fb:
            cmd["fb"] = 1
        msg = self._request(cmd)
        if fb and isinstance(msg, dict) and msg.get("T") == FEEDBACK_TURN:
            return msg
        return None

    def stop(self):
        try:
            self.set_wheels(0.0, 0.0)
        except ConnectionError:
            pass

    def wait_feedback(self, timeout=2.0) -> bool:
        return self._request({"T": 130}) is not None

    def refresh_feedback(self):
        """主动拉一帧底盘反馈（更新 yaw_deg / battery_v）"""
        self._request({"T": 130})

    def imu(self):
        """拉一帧 IMU 数据 {"T":1002, "y":.., "gz":..}（HTTP 模式下响应体即反馈帧）。
        gz 为原始陀螺仪 (度/s)，不受电机磁场干扰；y 仅在静止时可信"""
        return self._request({"T": 126})

    @property
    def yaw_deg(self):
        # 每次取用时都拉最新一帧（HTTP 往返仅几毫秒，20 Hz 控制环内可直接用）
        self.refresh_feedback()
        return self.feedback.get("y")

    @property
    def battery_v(self):
        return self.feedback.get("v")

    def close(self):
        try:
            self.stop()
        except Exception:
            pass


# ----------------------------------------------------------------------------
# 轨迹控制器
# ----------------------------------------------------------------------------
class TrajController:
    """航向闭环 + 里程开环的轨迹执行器（针对无编码器的 WAVE ROVER）"""

    def __init__(self, rover: WaveRover, calib: dict, verbose=True, log_fn=None):
        self.rover = rover
        self.c = calib
        self.verbose = verbose
        self.log_fn = log_fn          # web_ui 传入的日志回调（None 时用 print）
        self.abort = threading.Event()
        # --- 混合航向状态 ---
        # 背景：电机电流会干扰板载磁力计，导致 Mahony 航向 y 在转动中虚高 10 倍；
        # 但原始陀螺仪 gz 干净。因此：转动中用 gz 积分推算航向，
        # 停车静止后读 y 重新对准（并顺带测陀螺零偏）。
        self._gyro_h = None           # 陀螺积分航向（内部约定：左转为正）
        self._gyro_bias = 0.0         # 静止时测得的陀螺零偏 (度/s)
        self._gyro_t = 0.0            # 上次积分时刻
        self._h_rate = 0.0            # 低通后的角速度 (度/s)
        self._last_raw_h = None       # 回退路径（无陀螺数据时）的差分测速
        self._last_raw_t = 0.0
        self._turn_rate_learned = None    # 转向实测转速 (度/s)，turn_to 自适应
        self._coast_deg = None            # 断电后滑行角幅度（turn_by 学习值）
        self._fast_stop_on = False    # 固件急停线程去重标记
        self._imu_fail_streak = 0     # 连续 IMU 请求失败计数（触发急停用）
        # V2 固件：板载陀螺航向 + 板载闭环转向（探测不到则走旧的 PC 端逻辑）
        self.fw = False
        self.fw_straight = False
        self.fw_last = None           # 最近一帧 T:1010 状态
        if rover is not None:
            self._probe_fw()

    # ------------------------------------------------------------ V2 固件 --
    def _probe_fw(self):
        st = None
        if hasattr(self.rover, "fw_status"):
            try:
                st = self.rover.fw_status()
            except Exception:
                st = None
        if not (isinstance(st, dict) and st.get("T") == FEEDBACK_TURN):
            self.log("[固件] 未检测到 V2 板载转向（旧固件），使用 PC 端转向逻辑")
            return
        self.fw = True
        self.fw_last = st
        self.fw_straight = "ss" in st     # v2.3+: on-board straight (T:165)
        self._fw_cmd({"T": 163, "gs": self.c["fw_gyro_scale"],
                      "kick": self.c["fw_kick"], "minp": self.c["fw_min_pwm"]})
        if self.fw_straight:
            self._fw_cmd({"T": 163, "skp": self.c["fw_skp"], "ski": self.c["fw_ski"],
                          "sr0": self.c["fw_ramp0"]})
        # v2.4.3+ (older firmware ignores unknown keys)
        self._fw_cmd({"T": 163, "trc": self.c["fw_turn_rate"], "trm": self.c["fw_turn_rate_min"]})
        self.log(f"[固件] V2 板载转向已启用 (陀螺航向 {st.get('gy')}°, "
                 f"零偏 {st.get('gb')}°/s, 主循环最大 {st.get('lp')} ms)")

    def _fw_cmd(self, obj):
        """发一条 V2 指令，返回状态帧（失败返回 None，不抛异常）"""
        try:
            if isinstance(self.rover, WifiRover):
                msg = self.rover.query(obj, FEEDBACK_TURN)
            else:
                msg = self.rover.query(obj, FEEDBACK_TURN, timeout=0.3)
        except ConnectionError:
            return None
        if msg:
            self.fw_last = msg
        return msg

    def fw_status(self):
        return self._fw_cmd({"T": 164})

    def fw_calibrate(self, ms: int = 1000, retries: int = 2):
        """静止测陀螺零偏并把陀螺航向清零（调用前车必须停稳）。
        v2.3 固件同时回报测量期间的陀螺噪声 gn：车在晃就重测，
        避免拿一个错的零偏去跑（直行会慢慢偏）。"""
        self.rover.stop()
        time.sleep(0.3)
        for attempt in range(retries + 1):
            self._fw_cmd({"T": 162, "ms": int(ms), "zero": 1})
            t0 = time.monotonic()
            st = None
            while time.monotonic() - t0 < ms / 1000.0 + 3.0:
                self._check_abort()
                time.sleep(0.1)
                st = self.fw_status()
                if st and not st.get("cal"):
                    break
            else:
                raise RuntimeError("陀螺标定超时（固件无响应）")
            gn = st.get("gn")
            if gn is None or gn <= self.c["fw_still_noise"]:
                self.log(f"    [陀螺] 零偏 {st.get('gb')}°/s"
                         + (f"，噪声 {gn}°/s" if gn is not None else "") + "，航向清零")
                return st
            self.log(f"    [陀螺] 测零偏时车在晃（噪声 {gn}°/s > {self.c['fw_still_noise']}），重测")
            time.sleep(0.5)
        raise RuntimeError("测零偏时车一直在晃：把车放稳、别碰它，再启动")

    def fw_heading(self):
        """(陀螺航向°, 角速度°/s)，内部约定左转为正；失败返回 None"""
        st = self.fw_status()
        if not st:
            return None
        return float(st["gy"]), float(st.get("gr", 0.0))

    def fw_turn(self, delta_deg: float, spd: float = None, timeout: float = None):
        """板载闭环原地转向：PC 只下发目标并轮询状态，控制环在 ESP32 上跑，
        不受 WiFi 延迟影响。"""
        if spd is None:
            spd = self.c["fw_turn_pwm"]
        if timeout is None:
            timeout = 3.0 + abs(delta_deg) / 15.0
        t0 = time.monotonic()
        st = self._fw_cmd({"T": 160, "deg": round(float(delta_deg), 2),
                           "spd": round(float(spd), 3), "tmo": round(timeout, 1)})
        if st is None:
            raise RuntimeError("板载转向指令发送失败（固件无响应）")
        last_ok = time.monotonic()
        try:
            while True:
                self._check_abort()
                time.sleep(0.05)
                st = self.fw_status()
                now = time.monotonic()
                if st is None:
                    # HTTP 一时没响应不影响板载控制环；长时间无响应 = 主循环卡死
                    if now - last_ok > timeout + 3.0:
                        raise RuntimeError("板载转向期间固件长时间无响应")
                    continue
                last_ok = now
                if st.get("ts") != TS_RUNNING:
                    break
                if now - t0 > timeout + 3.0:
                    raise RuntimeError("板载转向超时（状态未结束）")
        except Aborted:
            self._fw_cmd({"T": 161})
            raise
        state = st.get("ts")
        if state != TS_DONE and self.abort.is_set():
            raise Aborted()        # stopped by the user (T:161 raced the poll)
        msg = (f"    [转向] {delta_deg:+.1f}° → {TS_NAMES.get(state, state)}，"
               f"剩余误差 {st.get('te')}°，分段 {st.get('tn')}，"
               f"重冲 {st.get('tk')}，滑行系数 {st.get('ck')}，"
               f"用时 {time.monotonic() - t0:.1f}s")
        self.log(msg)
        if state == TS_DONE:
            return st
        if state == TS_ABORTED:
            raise RuntimeError("板载转向被中止（安全看门狗或其他运动指令打断）")
        if state == TS_STALLED:
            raise RuntimeError("板载转向卡住：多次重冲仍转不动（电量低/卡住/kick 太小）")
        raise RuntimeError("板载转向超时：可能电量不足或地面打滑")

    # -- 内部航向：约定左转(逆时针)为正，消除固件 yaw 方向的不确定性 --
    def heading_deg(self):
        raw = self.rover.yaw_deg
        if raw is None:
            raise RuntimeError("没有收到 IMU 反馈，无法闭环控制")
        return (raw * self.c["yaw_sign"]) % 360.0

    def log(self, *args):
        text = " ".join(str(a) for a in args)
        if self.log_fn:
            self.log_fn(text)
        elif self.verbose:
            print(text)

    def _check_abort(self):
        """网页 Stop 键置位 abort 后，在任意控制循环中安全退出"""
        if self.abort.is_set():
            try:
                self.rover.stop()
            except Exception:
                pass
            raise Aborted()

    def _heading_and_rate(self):
        """读航向 + 角速度。优先用混合航向（陀螺积分）：
        转动中 Mahony yaw 被电机磁场污染（实测虚报 10 倍），gz 才可信；
        停车静止时由 resync_heading() 用干净的 yaw 重新对准。"""
        frame = None
        if hasattr(self.rover, "imu"):
            try:
                frame = self.rover.imu()
            except Exception:
                frame = None
        now = time.monotonic()
        if frame and frame.get("gz") is not None and frame.get("y") is not None:
            gz = float(frame["gz"]) * self.c["yaw_sign"]     # 度/s，内部约定左转为正
            if self._gyro_h is None:
                self._gyro_h = float(frame["y"]) * self.c["yaw_sign"]
                self._gyro_t = now
                self._h_rate = gz - self._gyro_bias
            else:
                dt = max(1e-3, now - self._gyro_t)
                self._gyro_h += (gz - self._gyro_bias) * dt
                self._gyro_t = now
                self._h_rate = 0.7 * self._h_rate + 0.3 * (gz - self._gyro_bias)
            return self._gyro_h % 360.0, self._h_rate
        # 回退路径（无 IMU 帧可用）：直接用 Mahony yaw（静止时干净）
        h = self.heading_deg()
        if h is None:
            raise RuntimeError("没有收到 IMU 反馈，无法闭环控制")
        if self._gyro_h is not None:
            self._gyro_h = h * self.c["yaw_sign"]
            self._gyro_t = now
        raw_rate = 0.0
        if hasattr(self, "_last_raw_h") and self._last_raw_h is not None:
            raw_rate = wrap_deg(h - self._last_raw_h) / max(1e-3, now - self._last_raw_t)
        self._last_raw_h, self._last_raw_t = h, now
        self._h_rate = 0.7 * self._h_rate + 0.3 * raw_rate
        return h, self._h_rate

    def _imu_or_yaw(self):
        """取一帧 {'y':.., 'gz':..}；优先 IMU 帧，回退用 yaw 反馈。
        固件卡死时返回 None（不抛异常，让轮询继续等它恢复）。"""
        if hasattr(self.rover, "imu"):
            try:
                f = self.rover.imu()
                self._imu_fail_streak = 0
                if f and f.get("y") is not None:
                    return f
            except ConnectionError:
                self._imu_fail_streak += 1
                if self._imu_fail_streak >= 2:
                    self._kick_fast_stop()
                return None
            except Exception:
                return None
        try:
            y = getattr(self.rover, "yaw_deg", None)
        except (ConnectionError, RuntimeError):
            self._kick_fast_stop()
            return None
        return {"y": y, "gz": 0.0} if y is not None else None

    def _imu_frame(self):
        """取一帧 IMU（含原始 gz 度/s）；无 IMU 或固件卡死时返回 None"""
        if hasattr(self.rover, "imu"):
            try:
                f = self.rover.imu()
                self._imu_fail_streak = 0
                return f if f and f.get("gz") is not None else None
            except ConnectionError:
                # 连续 2 次失败（~3.5s）才连发急停——单次超时可能只是
                # WiFi 延迟毛刺，误触发会打断正在执行的脉冲
                self._imu_fail_streak += 1
                if self._imu_fail_streak >= 2:
                    self._kick_fast_stop()
                return None
            except Exception:
                return None
        return None

    def _kick_fast_stop(self):
        """后台线程连发急停指令（固件恢复瞬间即停车），不阻塞控制循环"""
        if self._fast_stop_on:
            return
        rover = self.rover
        if not hasattr(rover, "fast_stop"):
            return

        def _run():
            self._fast_stop_on = True
            try:
                self.log("!! 固件疑似卡死（电机干扰），后台连发急停...")
                rover.fast_stop()
            finally:
                self._fast_stop_on = False

        threading.Thread(target=_run, daemon=True).start()

    def _soft_stop(self):
        try:
            self.rover.stop()
        except Exception:
            pass

    def _wait_recover(self, deadline=90.0):
        """固件卡死后等它自愈（电机由后台急停/固件心跳保护，车是停着的）。
        用于直行/读数环节：恢复后继续执行，任务不中止。"""
        self.log("   [等待] 固件卡死，等它自愈（急停已在后台连发）...")
        t0 = time.monotonic()
        while time.monotonic() - t0 < deadline:
            self._check_abort()
            try:
                if hasattr(self.rover, "imu"):
                    if self.rover.imu() is not None:
                        self._imu_fail_streak = 0
                        self.log("   [等待] 固件已恢复，继续")
                        return True
                else:
                    self.heading_deg()
                    return True
            except Exception:
                pass
            time.sleep(1.0)
        return False

    def _sample_bias(self, n: int = 6, settle: float = 0.3):
        """静止时采样陀螺零偏（内部约定符号）。调用前车必须已停稳。
        陀螺零偏漂移很慢（热漂移 ~0.01°/s/min），一次转向内测一次足够。"""
        time.sleep(settle)
        gzs = []
        for _ in range(n):
            f = self._imu_frame()
            if f is not None:
                gzs.append(float(f["gz"]) * self.c["yaw_sign"])
            time.sleep(0.05)
        if gzs:
            self._gyro_bias = sum(gzs) / len(gzs)
        return self._gyro_bias

    def _yaw_and_rate(self):
        """直行段用的航向反馈：直接用 Mahony yaw 的差分。
        直行时电机电流恒定 → 磁偏移恒定 → yaw 的【变化】是真实转动，
        这是直行唯一可靠的反馈（陀螺在持续通电时虚报 ~1.5 倍，不能用）。"""
        h = self.heading_deg()          # 已是内部约定（乘过 yaw_sign）
        if h is None:
            raise RuntimeError("没有收到 IMU 反馈，无法闭环控制")
        now = time.monotonic()
        raw_rate = 0.0
        if self._last_raw_h is not None:
            raw_rate = wrap_deg(h - self._last_raw_h) / max(1e-3, now - self._last_raw_t)
        self._last_raw_h, self._last_raw_t = h, now
        self._h_rate = 0.7 * self._h_rate + 0.3 * raw_rate
        return h, self._h_rate

    def resync_heading(self, settle=None, stable_wait=6.0):
        """停车静止后重新对准航向。
        电机电流会污染 Mahony 航向解算（转动中虚报可达十倍），断电后读数以
        ~1.2s 时间常数回弹向真值——因此轮询直到读数稳定（连续两个 0.5s 窗口
        变化 < 1.2°）才采信，同时用稳定段的 gz 均值做陀螺零偏。
        必须在电机停止后调用。返回对准后的航向（内部约定），失败返回 None。"""
        if settle is None:
            settle = self.c["resync_settle"]
        self.rover.stop()
        time.sleep(settle)
        readings = []                      # [(t, yaw, gz)]
        t0 = time.monotonic()
        next_t = t0
        while time.monotonic() - t0 < stable_wait:
            now = time.monotonic()
            if now >= next_t:
                f = self._imu_or_yaw()
                if f:
                    s = self.c["yaw_sign"]
                    readings.append((now, float(f["y"]) * s,
                                     float(f.get("gz", 0.0)) * s))
                next_t = now + 0.5
            if len(readings) >= 3:
                d1 = wrap_deg(readings[-1][1] - readings[-2][1])
                d2 = wrap_deg(readings[-2][1] - readings[-3][1])
                if abs(d1) < 1.2 and abs(d2) < 1.2:
                    break                  # 读数已稳定（磁修正回弹结束）
            time.sleep(0.03)
        if not readings:
            return None
        tail = readings[-3:]
        self._gyro_h = sum(r[1] for r in tail) / len(tail)
        self._gyro_bias = sum(r[2] for r in tail) / len(tail)
        self._gyro_t = time.monotonic()
        self._h_rate = 0.0
        return self._gyro_h % 360.0

    # ------------------------------------------------------------------ 转向 --
    def turn_by(self, delta_deg: float, max_pwm: float = 0.26,
                timeout: float = None):
        """原地相对转角（对外接口）。包一层固件卡死保护。"""
        if self.fw:
            return self.fw_turn(delta_deg, timeout=timeout)
        try:
            return self._turn_by_impl(delta_deg, max_pwm, timeout)
        except ConnectionError:
            self._kick_fast_stop()
            raise RuntimeError(
                "固件 HTTP 卡死（电机干扰 I2C）。后台急停指令已连发；"
                "请等约半分钟或重启小车电源后重试")

    def _turn_by_impl(self, delta_deg: float, max_pwm: float = 0.26,
                      timeout: float = None):
        """
        原地相对转角 v6（连续转，内部约定：左转为正）。不依赖磁力计。
        实测教训（diag_gyro_scale2.py / diag_lshape.py）：
        - 指令的 启动/停止 跳变疑似触发固件卡死；连续匀速转一圈 0 卡死
          → 一个指令连续转到底，只在到位时停一次（中途仅 1s 保活重发同值）
        - 固件卡死期间电机带着旧指令失控狂转，陀螺盲区（积分数不到）
          → 连续模式下卡死极少发生，从源头规避
        - 反向刹车在滑地面上把车拧回去（来回摇摆）→ 不刹车，滑行计入积分
        - gyro_scale：陀螺读数整体偏小 ~10%（积分360° 实车转400°）
        结构：阶段1 连续转(提前留滑行量) → 阶段2 滑行计入+学习滑行角
              → 阶段3 微脉冲修正（至多3次）
        """
        tol = max(2.0, self.c["tol_heading"] * 0.75)
        gyro_scale = self.c.get("gyro_scale", 1.0)
        pwm = clamp(max_pwm, self.c["turn_min_pwm"], 0.30)
        if timeout is None:
            timeout = 12.0 + abs(delta_deg) / 4.0
        self._sample_bias()
        direction = 1.0 if delta_deg > 0 else -1.0
        t0 = time.monotonic()
        last_t = None
        last_cmd = t0                # 上次轮速指令时刻（保活）
        last_integ_abs = 0.0         # 无进展看门狗用
        progress_t = t0
        imu_ok_t = t0
        integ = 0.0
        coast_est = getattr(self, "_coast_deg", None)
        if coast_est is None:
            coast_est = 0.0

        # --- 阶段1: 连续转（提前切断，剩余量交给滑行）---
        try:
            self.rover.set_wheels(-pwm * direction, pwm * direction)
            last_cmd = time.monotonic()
            no_progress_t = t0
            while True:
                self._check_abort()
                now = time.monotonic()
                f = self._imu_frame()
                if f is None or f.get("gz") is None:
                    if now - imu_ok_t > 8.0:
                        raise RuntimeError(
                            "固件 HTTP 卡死超过 8s。后台急停指令已连发；"
                            "请等约半分钟或重启小车电源后重试")
                    # 卡死期间仍重发同值指令：恢复后车继续转（方向没变）
                    if now - last_cmd > 0.25:
                        try:
                            self.rover.set_wheels(-pwm * direction, pwm * direction)
                        except Exception:
                            pass
                        last_cmd = now
                    time.sleep(LOOP_DT)
                    continue
                imu_ok_t = now
                gz = float(f["gz"]) * self.c["yaw_sign"]
                rate = (gz - self._gyro_bias) * gyro_scale
                if last_t is not None:
                    # 卡死空窗的时长也按当前转速近似补算（车在卡死中仍在转）
                    integ += rate * (now - last_t)
                last_t = now
                # 无进展看门狗（电机指令在发但积分不动：卡住/死区/yaw_sign 反）
                if abs(integ) - last_integ_abs > 1.0:
                    last_integ_abs = abs(integ)
                    progress_t = now
                elif now - progress_t > 3.0:
                    raise RuntimeError(
                        f"转向无进展 (已转 {integ:+.1f}°)。常见原因: "
                        f"1) 未做方向自检, yaw_sign 反了; "
                        f"2) turn_min_pwm({pwm:.2f}) 不足以原地转向; "
                        f"3) 地面太滑/卡住")
                remaining = delta_deg - integ
                # --- 看门狗 ---
                if now - t0 > timeout:
                    raise RuntimeError(
                        f"转向超时: 已转 {integ:+.1f}° / 目标 {delta_deg:+.0f}°。"
                        f"常见原因: 1) 电池电压不足; 2) turn_min_pwm 太小转不动 "
                        f"(当前 {pwm:.2f}); 3) 地面太滑")
                if abs(integ) > abs(delta_deg) + 200.0:
                    raise RuntimeError(
                        f"转向严重过冲 (已转 {integ:+.1f}°, 目标 {delta_deg:+.0f}°)。"
                        f"疑似 yaw_sign 反了或陀螺异常（请先运行方向自检）")
                if abs(remaining) <= tol and abs(rate) < 8.0:
                    break                    # 已到位
                if abs(remaining) <= coast_est + max(4.0, abs(rate) * 0.15):
                    break                    # 提前切断，滑行接管
                # 保活：固件心跳已收紧为 400ms（T:136），保活节奏必须更快
                if now - last_cmd > 0.25:
                    try:
                        self.rover.set_wheels(-pwm * direction, pwm * direction)
                    except Exception:
                        pass
                    last_cmd = now
                time.sleep(0.04)
        finally:
            try:
                self.rover.stop()
            except Exception:
                pass

        # --- 阶段2: 滑行计入（等到停稳），学习滑行角 ---
        stop_integ = integ
        t_end = time.monotonic() + 3.0
        still_t = None
        while time.monotonic() < t_end:
            now = time.monotonic()
            f = self._imu_frame()
            if f is None or f.get("gz") is None:
                time.sleep(LOOP_DT)
                continue
            gz = float(f["gz"]) * self.c["yaw_sign"]
            rate = (gz - self._gyro_bias) * gyro_scale
            if last_t is not None:
                integ += rate * (now - last_t)
            last_t = now
            if abs(rate) < 3.0:
                if still_t is None:
                    still_t = now
                elif now - still_t > 0.5:
                    break
            else:
                still_t = None
            time.sleep(0.04)
        coast_actual = integ - stop_integ
        # 学习滑行角（带符号转成幅度，EMA）
        self._coast_deg = 0.5 * coast_est + 0.5 * abs(coast_actual)

        # --- 阶段3: 微脉冲修正（至多 3 次）---
        for i in range(3):
            remaining = delta_deg - integ
            if abs(remaining) <= tol:
                break
            d = 1.0 if remaining > 0 else -1.0
            try:
                self.rover.set_wheels(-pwm * d, pwm * d)
            except Exception:
                pass
            time.sleep(0.12)
            try:
                self.rover.stop()
            except Exception:
                pass
            # 滑行计入（至多 1.5s）
            t_end = time.monotonic() + 1.5
            still_t = None
            while time.monotonic() < t_end:
                now = time.monotonic()
                f = self._imu_frame()
                if f is None or f.get("gz") is None:
                    time.sleep(LOOP_DT)
                    continue
                gz = float(f["gz"]) * self.c["yaw_sign"]
                rate = (gz - self._gyro_bias) * gyro_scale
                if last_t is not None:
                    integ += rate * (now - last_t)
                last_t = now
                if abs(rate) < 3.0:
                    if still_t is None:
                        still_t = now
                    elif now - still_t > 0.4:
                        break
                else:
                    still_t = None
                time.sleep(0.04)
            self.log(f"      [修正#{i+1}] 剩余 {delta_deg - integ:+.1f}°")

        remaining = delta_deg - integ
        if abs(remaining) > tol * 2.5:
            self.log(f"    [转向] {delta_deg:+.0f}° 完成但偏差 {remaining:+.1f}° "
                     f"(积分 {integ:+.1f}°, 用时 {time.monotonic() - t0:.1f}s)")
        else:
            self.log(f"    [转向] {delta_deg:+.0f}° 完成 (积分 {integ:+.1f}°, "
                     f"滑行 {coast_actual:+.1f}°, 用时 {time.monotonic() - t0:.1f}s)")


    def turn_to(self, target_deg: float, max_pwm: float = 0.26,
                timeout: float = None):
        """转到绝对目标航向（内部约定：左转为正）。
        注意：绝对航向来自磁力计，本车磁场环境差时误差可达 ±90°（双稳态）；
        轨迹执行请用 turn_by()（相对转角，纯陀螺，不受影响）。"""
        h = self.resync_heading()
        if h is None:
            raise RuntimeError("没有收到 IMU 反馈，无法闭环转向")
        self.turn_by(wrap_deg(target_deg - h), max_pwm=max_pwm, timeout=timeout)

    # ------------------------------------------------------------------ 直线 --
    def drive_straight(self, dist_m: float, speed_ms: float = 0.25,
                       heading_deg: float = None):
        """沿当前（或指定）航向闭环节向直线行驶 dist_m 米（对外接口）。"""
        try:
            return self._drive_straight_impl(dist_m, speed_ms, heading_deg)
        except ConnectionError:
            self._kick_fast_stop()
            raise RuntimeError(
                "固件 HTTP 卡死（电机干扰 I2C）。后台急停指令已连发；"
                "请等约半分钟或重启小车电源后重试")

    def _drive_straight_impl(self, dist_m: float, speed_ms: float = 0.25,
                             heading_deg: float = None):
        if self.fw:
            return self._drive_straight_fw(dist_m, speed_ms, heading_deg)
        if heading_deg is None:
            # 直行反馈基准 = 转弯/停车后稳定下来的 yaw 读数（差分）
            heading_deg = self.resync_heading(settle=0.3, stable_wait=4.0)
            if heading_deg is None and self._wait_recover():
                heading_deg = self.resync_heading(settle=0.3, stable_wait=6.0)
            if heading_deg is None:
                raise RuntimeError("没有收到 IMU 反馈，无法直行")
        base_pwm = speed_ms / self.c["speed_gain"]
        base_pwm = max(base_pwm, self.c["min_pwm"])   # 直行至少越过死区
        # 加减速斜坡
        ramp_steps = max(1, int(self.c["ramp_time"] / LOOP_DT))
        # 按实际指令速度积分里程终止（斜坡期速度只有一半，名义时间会少走
        # speed*ramp/2 的距离，短距离段尤其明显）
        v_cmd = base_pwm * self.c["speed_gain"]       # 满速时的指令速度 m/s
        sign = 1.0 if dist_m >= 0 else -1.0
        dist_done = 0.0
        t0 = time.monotonic()
        t_last = t0
        while True:
            self._check_abort()
            now = time.monotonic()
            dt = now - t_last
            t_last = now
            t = now - t0
            # 斜坡：前 ramp_time 渐升（收尾靠电机断电惯性即可，避免过冲）
            scale = min(1.0, t / (ramp_steps * LOOP_DT))
            dist_done += v_cmd * scale * dt
            if dist_done >= abs(dist_m):
                break
            # 航向闭环（带角速度阻尼）：误差 → 差速修正
            # （直行段用 yaw 差分反馈——电机电流恒定时磁偏移恒定，可信）
            try:
                h, rate = self._yaw_and_rate()
            except (ConnectionError, RuntimeError):
                # 固件卡死：车停下等自愈，恢复后接着走（不中止任务）
                self._kick_fast_stop()
                self._soft_stop()
                if not self._wait_recover():
                    raise RuntimeError(
                        "固件 HTTP 卡死超过 90s，中止任务（车已停）")
                t_last = time.monotonic()
                continue
            err = wrap_deg(heading_deg - h)         # >0 车头偏右，需左修
            if abs(err) > 45.0:
                self.rover.stop()
                raise RuntimeError(
                    f"直行时航向偏差过大 ({err:+.0f}°)。常见原因: yaw_sign 反了"
                    f"（请先运行方向自检）、地面打滑或一侧电机异常")
            omega_deg = self.c["kp_heading"] * err - self.c["kd_turn"] * rate
            dv = math.radians(omega_deg) * self.c["wheel_track"] / 2.0
            corr_pwm = dv / self.c["speed_gain"]
            corr_pwm = clamp(corr_pwm, -0.15, 0.15)             # 修正量限幅
            # err>0 需左转(右轮加速/左轮减速)：vR - vL = 2*corr > 0
            try:
                self.rover.set_wheels(sign * base_pwm * scale - corr_pwm,
                                      sign * base_pwm * scale + corr_pwm)
            except ConnectionError:
                self._kick_fast_stop()
                if not self._wait_recover():
                    raise RuntimeError(
                        "固件 HTTP 卡死超过 90s，中止任务（车已停）")
                t_last = time.monotonic()
                continue
            time.sleep(LOOP_DT)
        self.rover.stop()
        self.log(f"    [直行] {dist_m:+.2f} m 完成 (用时 {time.monotonic() - t0:.1f}s)")

    def straight_pwm(self, speed_ms: float):
        """(cruise PWM, expected speed m/s) for a straight at speed_ms."""
        c = self.c
        if c.get("sv_k"):
            pwm = clamp(c["sv_dz"] + abs(speed_ms) / c["sv_k"], c["min_pwm"], 0.5)
            return pwm, max(1e-3, (pwm - c["sv_dz"]) * c["sv_k"])
        pwm = max(abs(speed_ms) / c["speed_gain"], c["min_pwm"])
        return pwm, pwm * c["speed_gain"]

    def straight_plan(self, dist_m: float, speed_ms: float):
        """(cruise PWM, on-board duration s) for an on-board straight."""
        pwm, v = self.straight_pwm(speed_ms)
        ramp, r0 = max(0.0, self.c["fw_ramp"]), self.c["fw_ramp0"]
        return pwm, abs(dist_m) / v + ramp * (1.0 - r0) / 2.0

    def _drive_straight_onboard(self, dist_m: float, speed_ms: float,
                                heading_deg: float = None):
        """V2.3 firmware: the whole straight segment runs on the ESP32 (T:165).
        The PC converts metres to a duration and only polls the status, so WiFi
        latency can neither bend the line nor make the motors stutter."""
        hr = self.fw_heading()
        if hr is None:
            raise RuntimeError("没有收到陀螺航向，无法直行")
        # the on-board heading is unwrapped (e.g. -180 after a right-turn
        # square's 2nd corner): send the target closest to the current heading
        heading_deg = hr[0] if heading_deg is None else hr[0] + wrap_deg(heading_deg - hr[0])
        base_pwm, dur = self.straight_plan(dist_m, speed_ms)
        ramp = max(0.0, self.c["fw_ramp"])
        sign = 1.0 if dist_m >= 0 else -1.0
        cmd = {"T": 165, "ms": int(round(dur * 1000)), "spd": round(sign * base_pwm, 4),
               "hd": round(heading_deg, 2), "ramp": int(round(ramp * 1000))}
        t0 = time.monotonic()
        st = None
        for _ in range(4):
            st = self._fw_cmd(cmd)
            if st is not None:
                break
            # The request may have reached the car even though the reply was
            # lost: re-sending would restart the segment (too long a line).
            st = self.fw_status()
            if st is not None and st.get("ss") == 1:
                break
        if st is None or st.get("ss") != 1:
            raise RuntimeError("板载直行指令发送失败（固件无响应或正在测零偏）")
        last_ok = t0
        try:
            while True:
                self._check_abort()
                time.sleep(0.05)
                st = self.fw_status()
                now = time.monotonic()
                if st is None:
                    # the car finishes the segment on its own; only give up
                    # if it stays unreachable well past the planned end
                    if now - last_ok > dur + 3.0:
                        raise RuntimeError("板载直行期间固件长时间无响应")
                    continue
                last_ok = now
                if st.get("ss") != 1:
                    break
                if now - t0 > dur + 3.0:
                    raise RuntimeError("板载直行超时（状态未结束）")
        except Aborted:
            self._fw_cmd({"T": 161})
            raise
        ss = st.get("ss")
        if ss != 2 and self.abort.is_set():
            raise Aborted()        # stopped by the user (T:161 raced the poll)
        cur = ""
        if "sib" in st:            # v2.4: battery current baseline / peak, motor trim
            cur = (f"，电流 基线 {st.get('sib')} / 峰值 {st.get('sip')} mA，"
                   f"配平 {st.get('stf')}")
        self.log(f"    [直行] {dist_m:+.2f} m（板载）用时 {time.monotonic() - t0:.1f}s，"
                 f"航向误差 平均 {st.get('sa')}° / 最大 {st.get('sm')}°{cur}")
        if ss == 2:
            return st
        if ss == 4:
            raise RuntimeError(f"直行时航向偏差超过 30°（打滑/卡住/撞到东西），已停车")
        if ss == 5:
            raise RuntimeError(f"直行时电流持续升高（{st.get('si')} mA，基线 {st.get('sib')} mA），"
                               f"判定为堵转（顶到墙或被卡住），已停车")
        raise RuntimeError("板载直行被中止（其他运动指令或安全看门狗打断）")

    def _drive_straight_fw(self, dist_m: float, speed_ms: float,
                           heading_deg: float = None):
        if self.fw_straight:
            return self._drive_straight_onboard(dist_m, speed_ms, heading_deg)
        """V2 固件直行：航向反馈 = 板载陀螺航向 gy（与转向同一参考系，
        不用磁力计、不用停车等回弹）。heading_deg 为陀螺系下的目标航向。"""
        if heading_deg is None:
            hr = self.fw_heading()
            if hr is None:
                raise RuntimeError("没有收到陀螺航向，无法直行")
            heading_deg = hr[0]
        base_pwm = max(speed_ms / self.c["speed_gain"], self.c["min_pwm"])
        ramp_t = max(LOOP_DT, self.c["ramp_time"])
        v_cmd = base_pwm * self.c["speed_gain"]
        sign = 1.0 if dist_m >= 0 else -1.0
        dist_done = 0.0
        t0 = time.monotonic()
        t_last = t0
        st = self.fw_last
        fail_t = None
        applied = 0.0                 # ramp factor of the command sent last cycle
        while True:
            self._check_abort()
            now = time.monotonic()
            dt = now - t_last
            t_last = now
            # the wheels ran at the PREVIOUS command during [t_last, now]
            dist_done += v_cmd * applied * dt
            if dist_done >= abs(dist_m):
                break
            scale = min(1.0, (now - t0) / ramp_t)
            corr_pwm = 0.0
            if st and "gy" in st:
                err = wrap_deg(heading_deg - float(st["gy"]))
                if abs(err) > 45.0:
                    self.rover.stop()
                    raise RuntimeError(
                        f"直行时航向偏差过大 ({err:+.0f}°)：地面打滑或一侧电机异常")
                omega_deg = (self.c["kp_heading"] * err
                             - self.c["kd_turn"] * float(st.get("gr", 0.0)))
                dv = math.radians(omega_deg) * self.c["wheel_track"] / 2.0
                corr_pwm = clamp(dv / self.c["speed_gain"], -0.15, 0.15)
            try:
                fb = self.rover.set_wheels(sign * base_pwm * scale - corr_pwm,
                                           sign * base_pwm * scale + corr_pwm,
                                           fb=True)
                applied = scale
                fail_t = None
            except ConnectionError:
                fb = None
                applied = 0.0         # unknown: assume the lease stopped the wheels
                if fail_t is None:
                    fail_t = now
                elif now - fail_t > 1.0:
                    self._kick_fast_stop()
                    self._soft_stop()
                    if not self._wait_recover():
                        raise RuntimeError("固件 HTTP 卡死超过 90s，中止任务（车已停）")
                    fail_t = None
                    t_last = time.monotonic()
                    continue
            if fb:
                st = self.fw_last = fb
            elif not isinstance(self.rover, WifiRover):
                st = self.rover.frames.get(FEEDBACK_TURN, st)
            time.sleep(LOOP_DT)
        self.rover.stop()
        self.log(f"    [直行] {dist_m:+.2f} m 完成 (用时 {time.monotonic() - t0:.1f}s)")

    # ------------------------------------------------------------------ 曲线 --
    def drive_curve(self, path_func, total_len_m: float, speed_ms: float = 0.25):
        """
        连续曲线跟踪（圆 / 8 字等）。
        path_func(t01) -> (x, y)   t01 ∈ [0,1]，世界坐标（起点车头朝 x+）
        原理：每周期求轨迹切线方向作为目标航向，航向环输出差速。
        """
        t0 = time.monotonic()
        # + ramp/2 补偿起步斜坡期的亏距（与 drive_straight 同理）
        total_t = total_len_m / speed_ms + 0.5 * self.c["ramp_time"]
        if self.fw:
            hr = self.fw_heading()
            if hr is None:
                raise RuntimeError("没有收到陀螺航向，无法闭环跟踪曲线")
            h0 = hr[0] % 360.0
        else:
            if self._gyro_h is None:      # 没有对准过的陀螺航向 → 静止读一次
                if self.resync_heading(settle=0.6) is None:
                    raise RuntimeError("没有收到 IMU 反馈，无法闭环跟踪曲线")
            h0 = self._gyro_h % 360.0
        last_xy = path_func(0.0)
        last_t01 = 0.0
        target_h = h0                      # 初始目标航向 = 当前航向
        ramp_steps = max(1, int(self.c["ramp_time"] / LOOP_DT))
        while True:
            self._check_abort()
            t = time.monotonic() - t0
            if t >= total_t:
                break
            t01 = min(1.0, t / total_t)
            x, y = path_func(t01)
            dx, dy = x - last_xy[0], y - last_xy[1]
            ds = math.hypot(dx, dy)
            if ds > 1e-6 and t01 > last_t01:
                tangent_deg = math.degrees(math.atan2(dy, dx))
                target_h = (h0 + tangent_deg) % 360.0
            last_xy = (x, y)
            last_t01 = t01
            # 航向环（带阻尼）
            if self.fw:
                hr = self.fw_heading()
                if hr is None:
                    time.sleep(LOOP_DT)
                    continue
                h, rate = hr[0] % 360.0, hr[1]
            else:
                h, rate = self._heading_and_rate()
            err = wrap_deg(target_h - h)
            omega_deg = self.c["kp_heading"] * err - self.c["kd_turn"] * rate
            omega_rad = math.radians(omega_deg)
            # 差速运动学：vL = v - ω*W/2, vR = v + ω*W/2
            v_l = speed_ms - omega_rad * self.c["wheel_track"] / 2.0
            v_r = speed_ms + omega_rad * self.c["wheel_track"] / 2.0
            scale = min(1.0, t / (ramp_steps * LOOP_DT))
            self.rover.set_wheels(v_l / self.c["speed_gain"] * scale,
                                 v_r / self.c["speed_gain"] * scale)
            time.sleep(LOOP_DT)
        self.rover.stop()
        self.log(f"    [曲线] 完成 (预计 {total_t:.1f}s)")

    # -------------------------------------------------------------- 路径点 --
    def run_waypoints(self, waypoints, speed_ms: float = 0.25,
                      turn_pwm: float = 0.26):
        """
        按路径点序列行驶。waypoints = [(x, y), ...] 单位米，
        坐标系：起点为原点，初始车头方向为 +x，左侧为 +y（常规右手系）。
        每段：转向到该段方位角 → 保持该方位角直行。
        方位角 atan2(dy,dx) 是相对初始车头的【绝对】角度，转向量必须减去
        当前朝向（旧版直接把绝对方位角当相对转角，正方形第 3/4 个弯会转错）。"""
        self.log(f"[轨迹] 共 {len(waypoints) - 1} 段")
        if self.fw:
            return self._run_waypoints_fw(waypoints, speed_ms)
        cur_x, cur_y = 0.0, 0.0
        cur_bearing = 0.0                  # 计划中的当前车头方位角（相对初始车头）
        for i, (wx, wy) in enumerate(waypoints):
            self._check_abort()
            dx, dy = wx - cur_x, wy - cur_y
            dist = math.hypot(dx, dy)
            if dist < 0.02:
                continue
            bearing_deg = math.degrees(math.atan2(dy, dx))
            delta = wrap_deg(bearing_deg - cur_bearing)
            self.log(f"[轨迹] 第 {i + 1}/{len(waypoints)} 点 ({wx:.2f},{wy:.2f})"
                     f" → 转向 {delta:+.0f}°，前进 {dist:.2f} m")
            if abs(delta) >= 1.0:
                self.turn_by(delta, max_pwm=turn_pwm)
            cur_bearing = bearing_deg
            # 直行的参考航向 = 转弯后稳定的 yaw 读数（等磁回弹稳定，取差分）
            h_ref = self.resync_heading(settle=0.6, stable_wait=5.0)
            if h_ref is None and self._wait_recover():
                h_ref = self.resync_heading(settle=0.6, stable_wait=8.0)
            if h_ref is None:
                raise RuntimeError("没有收到 IMU 反馈，中止任务")
            self.drive_straight(dist, speed_ms=speed_ms, heading_deg=h_ref)
            time.sleep(0.2)
            cur_x, cur_y = wx, wy
        self.log("[轨迹] 全部路径点执行完毕")

    def _run_waypoints_fw(self, waypoints, speed_ms: float):
        """V2 固件：起点静止标定陀螺并清零 → 航向 0 = 初始车头。
        每段目标航向 = 该段方位角（陀螺系绝对值），转向量按【实测】航向算，
        所以上一段的残余误差会在下一次转向时被吃掉，不会累积。"""
        self.fw_calibrate()
        cur_x, cur_y = 0.0, 0.0
        for i, (wx, wy) in enumerate(waypoints):
            self._check_abort()
            dx, dy = wx - cur_x, wy - cur_y
            dist = math.hypot(dx, dy)
            if dist < 0.02:
                continue
            bearing_deg = math.degrees(math.atan2(dy, dx))
            hr = self.fw_heading()
            if hr is None:
                raise RuntimeError("没有收到陀螺航向，中止任务")
            delta = wrap_deg(bearing_deg - hr[0])
            self.log(f"[轨迹] 第 {i + 1}/{len(waypoints)} 点 ({wx:.2f},{wy:.2f})"
                     f" → 方位 {bearing_deg:+.0f}°（转 {delta:+.1f}°），前进 {dist:.2f} m")
            if abs(delta) >= 1.0:
                self.fw_turn(delta)
                time.sleep(0.15)
            self.drive_straight(dist, speed_ms=speed_ms, heading_deg=bearing_deg)
            time.sleep(0.15)
            cur_x, cur_y = wx, wy
        hr = self.fw_heading()
        self.log("[轨迹] 全部路径点执行完毕"
                 + (f"（终点陀螺航向 {hr[0]:+.1f}°）" if hr else ""))


# ----------------------------------------------------------------------------
# 标定核心（命令行 / 网页共用）
# ----------------------------------------------------------------------------
def calib_yaw_sign(rover, calib, log=print, pwm=0.3, secs=2.0) -> str:
    """原地左转 secs 秒 → 看 IMU 航向角增减 → 自动写 yaw_sign。返回 '' 或错误
    注意：起止读数都必须在电机停止 ≥1.5s 后取（电机电流会污染航向解算）"""
    rover.stop()
    time.sleep(1.5)
    y0 = rover.yaw_deg
    if y0 is None:
        return "无 IMU 反馈，无法标定"
    log(f"方向自检: 小车原地向左转 {secs:.0f}s（注意留出空间）...")
    rover.set_wheels(-pwm, pwm)          # 左轮退、右轮进 = 逆时针(左转)
    time.sleep(secs)
    rover.stop()
    time.sleep(1.5)                      # 等磁修正收敛，读数才干净
    y1 = rover.yaw_deg
    delta = wrap_deg(y1 - y0)
    log(f"自检结果: 航向 {y0:.1f}° → {y1:.1f}°, 变化 {delta:+.1f}°")
    if abs(delta) < 10:
        return "航向角几乎没有变化：小车没转（电量/卡住）或 IMU 异常"
    calib["yaw_sign"] = 1 if delta > 0 else -1
    save_calib(calib)
    log(f"[标定] yaw_sign = {calib['yaw_sign']}"
        f"（{'左转时航向角增大' if calib['yaw_sign'] > 0 else '左转时航向角减小'}）")
    return ""


def calib_turn_gain(rover, calib, log=print, pwm=0.21, secs=4.0) -> str:
    """原地左转 secs 秒 → 静止读净角位移 → 算 turn_gain（度/s 每单位差速）。
    起止都停车静止 ≥1.8s 后读数：转动中的航向反馈被电机磁场污染（虚报~10倍），
    只有静止净位移可信。"""
    rover.stop()
    time.sleep(1.8)
    y0 = rover.yaw_deg
    if y0 is None:
        return "无 IMU 反馈，无法标定"
    log(f"转向标定: 小车原地向左转 {secs:.0f}s (PWM={pwm})...")
    t0 = time.monotonic()
    while time.monotonic() - t0 < secs:   # 持续发指令防 3s 心跳自动停车
        rover.set_wheels(-pwm, pwm)
        time.sleep(0.1)
    rover.stop()
    time.sleep(1.8)
    y1 = rover.yaw_deg
    delta = wrap_deg(y1 - y0)
    log(f"自检结果: 静止净角位移 {delta:+.1f}° (折合 {delta / secs:+.1f}°/s)")
    if abs(delta) < 15:
        return (f"只测到 {abs(delta):.0f}°（PWM={pwm} 可能低于转向死区），"
                f"请把 turn_min_pwm 调大后重试")
    calib["turn_gain"] = abs(delta) / secs / (2.0 * pwm)   # 每单位差速 (R-L)
    calib["yaw_sign"] = 1 if delta > 0 else -1
    save_calib(calib)
    log(f"[标定] 共转 {abs(delta):.0f}°, turn_gain = {calib['turn_gain']:.0f} "
        f"(度/s 每单位差速), yaw_sign = {calib['yaw_sign']}")
    return ""


def calib_speed_drive(rover, calib, log=print, pwm=0.3, secs=3.0) -> str:
    """速度标定的行驶部分：直线行驶 secs 秒（结束后用户量距离再调 calib_speed_save）"""
    log(f"速度标定: 小车直行 {secs:.0f}s (PWM={pwm})，请准备好量距离...")
    t0 = time.monotonic()
    while True:
        t = time.monotonic() - t0
        if t >= secs:
            break
        s = pwm * min(1.0, t / calib["ramp_time"])   # 软启动
        rover.set_wheels(s, s)
        time.sleep(LOOP_DT)
    rover.stop()
    return ""


def calib_speed_save(calib, pwm, secs, dist_m, log=print) -> str:
    """速度标定的计算部分：用户量出实际距离后调用"""
    if dist_m <= 0.1:
        return "距离太小，无效"
    calib["speed_gain"] = dist_m / secs / pwm
    save_calib(calib)
    log(f"[标定] speed_gain = {calib['speed_gain']:.3f} (m/s 每 PWM 单位)，"
        f"满 PWM 0.5 时约 {calib['speed_gain'] * 0.5:.2f} m/s")
    return ""


# ----------------------------------------------------------------------------
# 命令行版标定（交互提示 + 调用上面的核心）
# ----------------------------------------------------------------------------
def cmd_status(rover: WaveRover):
    if not rover.wait_feedback():
        print("[错误] 2 秒内未收到下位机反馈，请检查端口/电源。")
        return
    for _ in range(5):
        y, v = rover.yaw_deg, rover.battery_v
        print(f"航向角 y = {y}°   电池 v = {v} V")
        time.sleep(0.3)


def cmd_calib_yaw(rover: WaveRover, calib: dict):
    input("小车将原地向左转约 2 秒 —— 请放到开阔地面后回车: ")
    err = calib_yaw_sign(rover, calib)
    if err:
        print(f"[错误] {err}")


def cmd_calib_speed(rover: WaveRover, calib: dict, args):
    input(f"小车将以 PWM={args.pwm} 直行 {args.seconds}s。"
          f"在地上放好卷尺后回车: ")
    calib_speed_drive(rover, calib, pwm=args.pwm, secs=args.seconds)
    ans = input("小车实际走了多少米？(输入数字，单位 m): ").strip()
    try:
        dist = float(ans)
    except ValueError:
        print("[取消] 未输入有效数字")
        return
    err = calib_speed_save(calib, args.pwm, args.seconds, dist)
    if err:
        print(f"[错误] {err}")


def cmd_calib_turn(rover: WaveRover, calib: dict, args):
    input(f"小车将原地左转 {args.seconds}s (PWM={args.pwm})，请放到开阔地面后回车: ")
    err = calib_turn_gain(rover, calib, pwm=args.pwm, secs=args.seconds)
    if err:
        print(f"[错误] {err}")


def cmd_fw_status(ctrl):
    if not ctrl.fw:
        print("[提示] 当前固件不是 V2（无 T:164 状态帧）")
        return
    for _ in range(5):
        st = ctrl.fw_status()
        print(json.dumps(st, ensure_ascii=False))
        time.sleep(0.3)


def cmd_fw_turn(ctrl, deg_text):
    if not ctrl.fw:
        print("[错误] turn 命令需要 V2 固件")
        return
    deg = float(deg_text)
    ctrl.fw_calibrate()
    ctrl.fw_turn(deg)
    hr = ctrl.fw_heading()
    if hr:
        print(f"陀螺航向 {hr[0]:+.1f}°（目标 {deg:+.1f}°）")


def cmd_calib_gs(ctrl, calib):
    """陀螺刻度标定：板载转向连转 720°，用户目测多转/少转了多少度。"""
    if not ctrl.fw:
        print("[错误] calib-gs 需要 V2 固件")
        return
    input("把车头对准地上一条直线（如地砖缝），留出原地转向空间后回车: ")
    ctrl.fw_calibrate()
    ctrl.fw_turn(720.0)
    ans = input("车头相对那条线多转了多少度？多转为正、少转为负（如 +8 / -5）: ").strip()
    try:
        extra = float(ans)
    except ValueError:
        print("[取消] 未输入有效数字")
        return
    old = calib.get("fw_gyro_scale", 1.0)
    calib["fw_gyro_scale"] = round(old * (720.0 + extra) / 720.0, 4)
    save_calib(calib)
    ctrl._fw_cmd({"T": 163, "gs": calib["fw_gyro_scale"]})
    print(f"[标定] fw_gyro_scale: {old} → {calib['fw_gyro_scale']}")


# ----------------------------------------------------------------------------
# WiFi 配置（让小车连上家里路由器，STA 模式）
# ----------------------------------------------------------------------------
def _wifi_req(rover, obj, timeout=0.5):
    """WiFi 指令请求：返回响应 dict；串口模式不支持（响应帧会被过滤）"""
    if not isinstance(rover, WifiRover):
        print("[提示] wifi-info / wifi-setup 需要 WiFi 模式运行"
              "（--ip 192.168.4.1，电脑先连上小车热点）")
        return None
    return rover._request(obj, timeout=timeout)


def cmd_wifi_info(rover: WaveRover):
    """查询当前 WiFi 状态（固件 T:405）：IP / 模式 / SSID"""
    resp = _wifi_req(rover, {"T": 405})
    if not resp:
        return
    print(f"模式 wifi_mode_on_boot = {resp.get('wifi_mode_on_boot')}"
          f"（1=热点AP, 2=路由器STA, 3=AP+STA 同时开）")
    print(f"当前 IP = {resp.get('ip')}   信号 RSSI = {resp.get('rssi')} dBm")
    print(f"热点 AP: {resp.get('ap_ssid')} / {resp.get('ap_password')}")
    print(f"路由器 STA: {resp.get('sta_ssid')} / {resp.get('sta_password')}")


def cmd_wifi_setup(rover: WaveRover, args):
    """
    把小车配置成 AP+STA（T:407）：保留原热点，同时连家里路由器，
    配置写入 flash，重启后自动恢复。固件源码 createWifiConfigFileByInput()。
    """
    if not args.ssid or not args.password:
        print("用法: wifi-setup --ssid 家里WiFi名 --password 密码")
        print("注意: ESP32 只支持 2.4GHz WiFi，5G 频段的 WiFi 连不上")
        return
    # 1) 先读当前热点配置，保持热点名/密码不变
    cur = _wifi_req(rover, {"T": 405})
    if not cur:
        return
    ap_ssid = args.ap_ssid or cur.get("ap_ssid") or "WAVE_ROVER"
    ap_pwd = args.ap_password or cur.get("ap_password") or "12345678"
    print(f"[1/3] 当前热点: {ap_ssid} (将保持不变)")
    print(f"[2/3] 发送 STA 配置，正在连接路由器 {args.ssid!r} ...")
    # T:407 会阻塞到连接成功/失败（固件内嵌最长 15s 的等待循环），加大超时
    rover._request(
        {"T": 407, "mode": 3,
         "ap_ssid": ap_ssid, "ap_password": ap_pwd,
         "sta_ssid": args.ssid, "sta_password": args.password},
        timeout=25.0)
    # 2) 轮询查询结果
    new_ip = None
    for _ in range(8):
        time.sleep(1.5)
        try:
            resp = rover._request({"T": 405}, timeout=1.0) or {}
        except ConnectionError:
            resp = {}
        if resp.get("sta_ssid") == args.ssid and resp.get("ip") not in \
                (None, "0.0.0.0", "192.168.4.1"):
            new_ip = resp.get("ip")
            break
    if new_ip:
        print(f"[3/3] 连接成功！小车的局域网 IP = {new_ip}")
        print(f"  电脑连回家里路由器后，这样使用：")
        print(f"  python wave_rover_traj.py --ip {new_ip} status")
        print(f"  热点 {ap_ssid} 也仍然可用（IP 固定 192.168.4.1，作为备用通道）")
    else:
        print("[!] 未能确认连接成功。请看小车 OLED 屏幕：")
        print("    第二行 'ST:x.x.x.x' = 成功（也可用上面 wifi-info 查询）")
        print("    若显示 'ST: OFF'，常见原因: 密码错 / WiFi 是 5GHz 频段"
              " / 路由器开了 MAC 过滤")
        print("    连不上时会自动退回纯热点模式，不会失联。")


# ----------------------------------------------------------------------------
# 内置演示轨迹
# ----------------------------------------------------------------------------
def demo_waypoints(kind: str, size: float = 1.0):
    if kind == "square":                       # 正方形回到起点
        return [(0, 0), (size, 0), (size, size), (0, size), (0, 0)]
    raise ValueError(kind)


def demo_curve(kind: str, radius: float = 0.6):
    if kind == "circle":                      # 半径 R 的整圆
        def f(t01):
            a = 2 * math.pi * t01
            return radius * math.sin(a), radius * (1 - math.cos(a))
        return f, 2 * math.pi * radius
    if kind == "figure8":                     # 两个相切圆的 8 字
        def f(t01):
            a = 2 * math.pi * t01
            return radius * math.sin(a), radius * math.sin(a) * math.cos(a) / 2
        return f, 2 * math.pi * radius * 1.5   # 近似弧长
    raise ValueError(kind)


# ----------------------------------------------------------------------------
# 主入口
# ----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description="WAVE ROVER 轨迹自动行驶程序（航向闭环 + 里程推算）")
    ap.add_argument("port", nargs="?", help="串口，如 COM5 或 /dev/ttyUSB0")
    ap.add_argument("command", nargs="?", default=None,
                    help="status/calib-yaw/calib-speed/calib-turn/waypoints/demo/stop/"
                         "fw-status/turn/calib-gs/wifi-info/wifi-setup")
    ap.add_argument("target", nargs="?", help="plan.json 或 square/circle/figure8")
    ap.add_argument("--list", action="store_true", help="列出可用串口后退出")
    ap.add_argument("--ip", metavar="IP", default=None,
                    help="WiFi 模式：小车 IP（热点模式固定 192.168.4.1；"
                         "路由器模式见 OLED 显示），指定后无需串口")
    ap.add_argument("--ssid", default=None, help="wifi-setup: 家里路由器的 WiFi 名")
    ap.add_argument("--password", default=None, help="wifi-setup: 家里路由器的 WiFi 密码")
    ap.add_argument("--ap-ssid", default=None, help="wifi-setup: 可选，新热点名")
    ap.add_argument("--ap-password", default=None, help="wifi-setup: 可选，新热点密码")
    ap.add_argument("--speed", type=float, default=0.25, help="直行速度 m/s")
    ap.add_argument("--pwm", type=float, default=0.3, help="标定用 PWM")
    ap.add_argument("--seconds", type=float, default=3.0, help="标定用时长 s")
    ap.add_argument("--calib", default=None,
                    help="标定文件；默认 WiFi 模式按车 IP 用 rover_calib_<IP>.json")
    ap.add_argument("--dry-run", action="store_true",
                    help="不连接小车，只打印将要执行的轨迹段")
    args = ap.parse_args()

    if args.list:
        if list_ports is None:
            print("缺少 pyserial，请先运行: pip install pyserial")
            return
        for p in list_ports.comports():
            print(f"{p.device}  {p.description}")
        return

    # 允许省略串口号直接写命令，如 "demo square" / "waypoints plan.json"
    commands = ["status", "calib-yaw", "calib-speed", "calib-turn",
                "waypoints", "demo", "stop", "wifi-info", "wifi-setup",
                "fw-status", "turn", "calib-gs"]
    if args.port in commands:
        args.target = args.command
        args.command = args.port
        args.port = None
    if args.command is None:
        args.command = "status"
    if args.command not in commands:
        ap.error(f"未知命令: {args.command}（可选: {'/'.join(commands)}）")

    if not args.port and not args.ip and not args.dry_run:
        ap.error("需要连接方式：串口号（用 --list 查看）、--ip 192.168.4.1（WiFi），"
                 "或加 --dry-run 干跑")

    calib = load_calib(args.calib or (per_car_calib_file(args.ip) if args.ip else None))

    # ---------- 干跑模式：只打印计划 ----------
    if args.dry_run:
        ctrl = TrajController(None, calib)
        if args.command == "waypoints":
            with open(args.target, encoding="utf-8") as f:
                wps = json.load(f)
            if isinstance(wps, dict):
                wps = wps["waypoints"]
            print("[干跑] 路径点:")
        elif args.command == "demo":
            if args.target in ("circle", "figure8"):
                f, length = demo_curve(args.target)
                print(f"[干跑] 曲线 {args.target}，弧长约 {length:.1f} m")
                return
            wps = demo_waypoints(args.target)
        else:
            print("[干跑] 仅 waypoints/demo 支持干跑");  return
        cur = (0.0, 0.0)
        cur_b = 0.0
        for i, (wx, wy) in enumerate(wps):
            dx, dy = wx - cur[0], wy - cur[1]
            d = math.hypot(dx, dy)
            if d < 0.02:
                continue
            b = math.degrees(math.atan2(dy, dx))
            print(f"  段{i}: 转 {wrap_deg(b - cur_b):+7.1f}° (方位 {b:+6.1f}°) → 直行 {d:.2f} m"
                  f"  (目标点 {wx:.2f},{wy:.2f})  预计用时 {d / args.speed:.1f}s")
            cur = (wx, wy)
            cur_b = b
        return

    # ---------- 连接小车 ----------
    if args.ip:
        rover = WifiRover(args.ip)
    else:
        if serial is None:
            print("USB 串口模式缺少 pyserial（pip install pyserial），"
                  "或改用 WiFi 模式 --ip 192.168.4.1")
            return
        rover = WaveRover(args.port)
    ctrl = TrajController(rover, calib)
    try:
        if args.command == "status":
            cmd_status(rover)
        elif args.command == "calib-yaw":
            cmd_calib_yaw(rover, calib)
        elif args.command == "calib-speed":
            cmd_calib_speed(rover, calib, args)
        elif args.command == "calib-turn":
            cmd_calib_turn(rover, calib, args)
        elif args.command == "stop":
            rover.stop()
            print("已发送停车指令。")
        elif args.command == "wifi-info":
            cmd_wifi_info(rover)
        elif args.command == "fw-status":
            cmd_fw_status(ctrl)
        elif args.command == "turn":
            if not args.target:
                ap.error("turn 需要角度，如 turn 90（左转为正）")
            cmd_fw_turn(ctrl, args.target)
        elif args.command == "calib-gs":
            cmd_calib_gs(ctrl, calib)
        elif args.command == "wifi-setup":
            cmd_wifi_setup(rover, args)
        elif args.command == "waypoints":
            if not args.target:
                ap.error("waypoints 需要一个 JSON 路径文件")
            with open(args.target, encoding="utf-8") as f:
                wps = json.load(f)
            if isinstance(wps, dict):
                wps = wps["waypoints"]
            wps = [(float(x), float(y)) for x, y in wps]
            ctrl.run_waypoints(wps, speed_ms=args.speed)
        elif args.command == "demo":
            if not args.target:
                ap.error("demo 需要 square/circle/figure8")
            if args.target == "square":
                ctrl.run_waypoints(demo_waypoints("square"), speed_ms=args.speed)
            elif args.target in ("circle", "figure8"):
                f, length = demo_curve(args.target)
                ctrl.drive_curve(f, length, speed_ms=args.speed)
            else:
                ap.error("demo 只支持 square/circle/figure8")
    except KeyboardInterrupt:
        print("\n[中断] 用户停止")
    except Aborted:
        print("\n[中止] 已停止")
    finally:
        rover.close()
        print("[安全] 已停车并关闭串口")


if __name__ == "__main__":
    main()
