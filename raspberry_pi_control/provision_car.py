# -*- coding: utf-8 -*-
"""One-shot setup of a WAVE ROVER on USB: flash firmware V2, join the car
WiFi, register it in cars.txt and check it over WiFi.

Usage (one car at a time, plugged into the driver board's "USB" port):
  python provision_car.py                 # auto-detect the COM port, next car number
  python provision_car.py --port COM7 --car 5
  python provision_car.py --check car5     # WiFi check only (no USB needed)
  python provision_car.py --check car5 --turn   # ... plus in-place turns +-90 deg

Steps: find the CP210x port -> read MAC -> pick the car number (same MAC
keeps its number) -> verify the partition table matches ours -> (optional
backup) -> reset otadata + write the app -> send the WiFi config over serial
(T:407) -> read the STA IP (T:405) -> update cars.txt -> WiFi check
(firmware version, MAC, gyro noise, battery) -> restart the idle console.
"""
import argparse
import datetime
import json
import os
import re
import subprocess
import sys
import time
import urllib.parse
import urllib.request

import serial
from serial.tools import list_ports

ROOT = os.path.dirname(os.path.abspath(__file__))
CARS_FILE = os.path.join(ROOT, "cars.txt")
APP_BIN = os.path.join(ROOT, "firmware_v2", "out", "firmware_v2.ino.bin")
PART_BIN = os.path.join(ROOT, "firmware_v2", "out", "firmware_v2.ino.partitions.bin")
FW_SRC = os.path.join(ROOT, "firmware_v2", "turn_ctrl.h")
BACKUP_DIR = os.path.join(ROOT, "backups")
ESPTOOL = os.path.join(os.environ.get("LOCALAPPDATA", ""), "Arduino15", "packages", "esp32",
                       "tools", "esptool_py", "4.5.1", "esptool.exe")
STA_SSID = os.environ.get("ROVER_WIFI_SSID", "")
STA_PASS = os.environ.get("ROVER_WIFI_PASSWORD", "")
AP_PASS = "12345678"
CP210X_VID = 0x10C4
GN_OK = 0.6          # dps; gyro noise while still (v2.4.1 LPF: ~0.2)
HOTSPOT_MAX = 8      # Windows Mobile Hotspot client limit


def log(msg):
    print(f"[{datetime.datetime.now():%H:%M:%S}] {msg}", flush=True)


def fail(msg):
    log("FAILED: " + msg)
    sys.exit(1)


def expected_fw():
    with open(FW_SRC, "r", encoding="utf-8", errors="ignore") as f:
        m = re.search(r'#define FW_VERSION "([^"]+)"', f.read())
    return m.group(1) if m else ""


# ------------------------------------------------------------- cars.txt --
def read_cars():
    cars = []
    if os.path.exists(CARS_FILE):
        with open(CARS_FILE, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                parts = line.split()
                if len(parts) >= 2 and parts[0].startswith("car"):
                    mac = next((p[4:].lower() for p in parts if p.upper().startswith("MAC=")), "")
                    cars.append({"id": parts[0], "ip": parts[1], "mac": mac, "line": line.rstrip("\n")})
    return cars


def write_car(cid, ip, mac):
    cars = read_cars()
    n = int(cid[3:])
    line = f"{cid} {ip} AP=UGV{n} MAC={mac.lower()}"
    out, done = [], False
    for c in cars:
        if c["id"] == cid or (mac and c["mac"] == mac.lower()):
            if not done:
                out.append(line)
                done = True
        else:
            out.append(c["line"])
    if not done:
        out.append(line)
    out.sort(key=lambda l: int(re.match(r"car(\d+)", l).group(1)))
    with open(CARS_FILE, "w", encoding="utf-8") as f:
        f.write("\n".join(out) + "\n")


def find_car(key):
    for c in read_cars():
        if key in (c["id"], c["ip"], c["mac"]):
            return c
    return None


# ------------------------------------------------------------- esptool --
def esptool(port, *args, timeout=180):
    cmd = [ESPTOOL, "--chip", "esp32", "--port", port, "--baud", "460800", *args]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    if r.returncode != 0:
        tail = "\n".join((r.stdout + r.stderr).strip().splitlines()[-6:])
        fail(f"esptool {args[0]} failed:\n{tail}\n"
             "If the port cannot be opened: hold BOOT, tap RESET, release BOOT, run again.")
    return r.stdout


def find_port(want):
    if want:
        return want
    ports = [p for p in list_ports.comports() if p.vid == CP210X_VID]
    if not ports:
        fail("no CP210x serial port. Plug the USB-C cable into the driver board port "
             "labelled USB (not LIDAR). If Device Manager shows an unknown device, "
             r"install the driver from tools\cp210x.")
    if len(ports) > 1:
        fail("several CP210x ports: " + ", ".join(p.device for p in ports) +
             " -> plug in one car only, or pass --port COMx")
    return ports[0].device


def partition_entries(blob):
    out = []
    for i in range(0, len(blob) - 31, 32):
        e = blob[i:i + 32]
        if e[:2] != b"\xaa\x50":
            break
        out.append(e)
    return out


# --------------------------------------------------------------- serial --
def serial_wifi(port, cid, ssid, password, wait_ip=30.0):
    """Send T:407 (STA + own AP) and wait for the STA IP via T:405."""
    n = int(cid[3:])
    cfg = {"T": 407, "mode": 3, "ap_ssid": f"UGV{n}", "ap_password": AP_PASS,
           "sta_ssid": ssid, "sta_password": password}
    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = port, 115200, 0.05
    ser.dtr = ser.rts = False          # do not reset the board
    ser.open()
    buf = b""

    def pump(sec):
        nonlocal buf
        end = time.monotonic() + sec
        while time.monotonic() < end:
            buf += ser.read(1024)

    try:
        pump(1.0)
        ser.write((json.dumps(cfg) + "\n").encode())
        pump(1.0)
        t0 = time.monotonic()
        while time.monotonic() - t0 < wait_ip:
            buf = b""
            ser.write(b'{"T":405}\n')
            pump(1.5)
            for m in re.finditer(rb'"ip"\s*:\s*"(\d+\.\d+\.\d+\.\d+)"', buf):
                ip = m.group(1).decode()
                if not ip.startswith("0.") and not ip.startswith("192.168.4."):
                    return ip
            pump(0.5)
    finally:
        ser.close()
    return ""


# ----------------------------------------------------------------- http --
def http(ip, obj, timeout=2.0):
    url = f"http://{ip}/js?json=" + urllib.parse.quote(json.dumps(obj))
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open(url, timeout=timeout) as r:
        data = r.read()
    return json.loads(data) if data.strip() else None


def wifi_check(ip, mac, want_fw, turn=False):
    """Returns (ok, summary)."""
    st = None
    for _ in range(20):
        try:
            st = http(ip, {"T": 164})
            break
        except Exception:
            time.sleep(1.0)
    if not st:
        return False, f"no answer over WiFi at {ip}"
    probs = []
    if want_fw and st.get("fw") != want_fw:
        probs.append(f"firmware {st.get('fw')} (want {want_fw})")
    if mac and st.get("mac") and st["mac"].lower() != mac.lower():
        probs.append(f"MAC {st['mac']} != {mac}")
    gns, s2 = [], st
    for _ in range(2):
        try:
            http(ip, {"T": 162, "ms": 1000, "zero": 1})
            time.sleep(1.5)
            s2 = http(ip, {"T": 164}) or st
            gns.append(s2.get("gn", 99.0))
        except Exception:
            time.sleep(1.0)
    gn = min(gns) if gns else 99.0
    if gn > GN_OK:
        probs.append(f"gyro noise {gn:.2f} dps (want < {GN_OK}; car moved, or a noisy IMU)")
    v = st.get("v", 0.0)
    power = "battery on" if v > 9.0 else "battery switch OFF (USB power only)"
    summary = (f"fw {st.get('fw')}, MAC {st.get('mac', '?')}, gyro noise {gn:.2f} dps, "
               f"bias {s2.get('gb')} dps, {v:.2f} V ({power}), loop max {st.get('lp')} ms")
    if turn:
        if v <= 9.0:
            probs.append("turn test skipped: battery switch is off")
        else:
            res = []
            for deg in (90, -90):
                r = subprocess.run([sys.executable, os.path.join(ROOT, "wave_rover_traj.py"),
                                    "--ip", ip, "turn", str(deg)],
                                   capture_output=True, text=True, encoding="utf-8",
                                   errors="replace", timeout=60,
                                   env=dict(os.environ, PYTHONIOENCODING="utf-8"))
                m = re.search(r"\[转向\][^\n]*", r.stdout)
                line = m.group(0).strip() if m else "no turn result"
                res.append(line)
                if "done" not in line:
                    probs.append(f"turn {deg}: {line}")
            summary += "\n    " + "\n    ".join(res)
    return not probs, summary + ("" if not probs else "\n    PROBLEMS: " + "; ".join(probs))


def restart_console_if_idle():
    try:
        st = json.loads(urllib.request.build_opener(urllib.request.ProxyHandler({}))
                        .open("http://localhost:8090/api/state", timeout=3).read())
    except Exception:
        return "console not running (start it with fleet_ui.bat)"
    if st.get("running"):
        return "console is running a job: restart it later to show the new car"
    ps = ("Get-CimInstance Win32_Process -Filter \"Name='python.exe'\" | Where-Object "
          "{ $_.CommandLine -like '*fleet_ui.py*' } | ForEach-Object { Stop-Process -Id $_.ProcessId -Force }")
    subprocess.run(["schtasks", "/end", "/tn", "WaveRoverFleetUI"], capture_output=True)
    subprocess.run(["powershell", "-NoProfile", "-Command", ps], capture_output=True)
    r = subprocess.run(["schtasks", "/run", "/tn", "WaveRoverFleetUI"], capture_output=True)
    return "console restarted" if r.returncode == 0 else "console restart failed (run fleet_ui.bat)"


# ----------------------------------------------------------------- main --
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="COMx (default: the only CP210x port)")
    ap.add_argument("--car", type=int, help="car number (default: same MAC keeps its number, else next free)")
    ap.add_argument("--backup", action="store_true", help="read the stock app slots first (~1 min)")
    ap.add_argument("--ssid", default=STA_SSID)
    ap.add_argument("--password", default=STA_PASS)
    ap.add_argument("--check", metavar="CAR", help="WiFi check only: car id / IP / MAC")
    ap.add_argument("--turn", action="store_true", help="with the check: turn +90 / -90 in place")
    args = ap.parse_args()
    want_fw = expected_fw()

    if args.check:
        c = find_car(args.check)
        ip = c["ip"] if c else args.check
        ok, summary = wifi_check(ip, c["mac"] if c else "", want_fw, args.turn)
        log(f"{c['id'] if c else ip} @ {ip}: " + summary)
        sys.exit(0 if ok else 1)

    if not args.ssid or not args.password:
        ap.error("Set ROVER_WIFI_SSID/ROVER_WIFI_PASSWORD or pass --ssid/--password before provisioning")

    if not os.path.exists(ESPTOOL):
        fail(f"esptool not found: {ESPTOOL}")
    for p in (APP_BIN, PART_BIN):
        if not os.path.exists(p):
            fail(f"missing {p} (run build_v2.bat first)")
    port = find_port(args.port)
    log(f"port {port}, firmware {want_fw}")

    out = esptool(port, "read_mac", timeout=60)
    m = re.search(r"MAC:\s*([0-9a-fA-F:]{17})", out)
    if not m:
        fail("could not read the MAC address")
    mac = m.group(1).lower()
    known = next((c for c in read_cars() if c["mac"] == mac), None)
    if args.car:
        cid = f"car{args.car}"
    elif known:
        cid = known["id"]
    else:
        used = [int(c["id"][3:]) for c in read_cars() if c["id"][3:].isdigit()]
        cid = f"car{max(used, default=0) + 1}"
    log(f"MAC {mac} -> {cid}" + (" (already registered)" if known else " (new car)"))

    tmp = os.path.join(ROOT, f"_part_{mac.replace(':', '')}.bin")
    esptool(port, "read_flash", "0x8000", "0xC00", tmp, timeout=60)
    with open(tmp, "rb") as f:
        on_car = partition_entries(f.read())
    os.remove(tmp)
    with open(PART_BIN, "rb") as f:
        ours = partition_entries(f.read())
    if not ours or on_car != ours:
        fail("partition table on this car differs from ours: not flashing. "
             "Send me the log; this board needs a full image (bootloader + partitions).")
    log("partition table OK")

    if args.backup:
        os.makedirs(BACKUP_DIR, exist_ok=True)
        bk = os.path.join(BACKUP_DIR, f"{cid}_{mac.replace(':', '')}_{datetime.datetime.now():%Y%m%d_%H%M%S}.bin")
        log(f"backup otadata + app0 + app1 -> {bk}")
        esptool(port, "read_flash", "0xE000", "0x282000", bk, timeout=300)

    log("flashing (otadata reset + app0)")
    esptool(port, "--after", "no_reset", "erase_region", "0xE000", "0x2000", timeout=60)
    esptool(port, "write_flash", "0x10000", APP_BIN, timeout=300)
    log("flash OK, waiting for boot")
    time.sleep(4.0)

    ip = serial_wifi(port, cid, args.ssid, args.password)
    if not ip:
        fail(f"the car did not join {args.ssid} within 30 s. Is the PC hotspot on "
             f"(tools\\hotspot_on.ps1)? The hotspot takes at most {HOTSPOT_MAX} cars at once.")
    log(f"joined {args.ssid}: {ip}, own AP UGV{cid[3:]}")
    write_car(cid, ip, mac)
    log(f"cars.txt updated: {cid} {ip}")

    ok, summary = wifi_check(ip, mac, want_fw, turn=False)
    log("check: " + summary)
    log(restart_console_if_idle())
    print()
    print(("OK   " if ok else "CHECK ") + f"{cid}  MAC {mac}  IP {ip}  AP UGV{cid[3:]}")
    print(f"Next: unplug USB, close the car, switch the battery on, put it on the floor, then\n"
          f"      python provision_car.py --check {cid} --turn")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
