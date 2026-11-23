# -*- coding: utf-8 -*-
"""OTA every car in cars.txt that is online and not on the current build.

Usage:
  python ota_all.py            # upgrade cars whose firmware differs from the build
  python ota_all.py --force    # re-flash every online car
  python ota_all.py --list     # only show where each car is and what it runs

Cars are found by MAC (the hotspot hands out new IPs after reboots); cars.txt
is updated with the IPs found. Cars are flashed one after another.
"""
import argparse
import os
import subprocess
import sys
import time

import fleet_ui
import provision_car as pc


def locate():
    """{mac: {"ip", "fw", "v"}} for every V2 car answering on the hotspot subnet."""
    prefixes = sorted({c["ip"].rsplit(".", 1)[0] for c in pc.read_cars()} | {"192.168.137"})
    found = {}
    for f in fleet_ui.scan_subnets(prefixes):
        mac = f.get("mac", "")
        try:
            st = pc.http(f["ip"], {"T": 164}, timeout=1.5) or {}
        except Exception:
            st = {}
        mac = (st.get("mac") or mac or "").lower()
        if mac:
            found[mac] = {"ip": f["ip"], "fw": st.get("fw", f.get("fw")), "v": st.get("v", f.get("v"))}
    return found


def ota(ip):
    r = subprocess.run(["curl.exe", "--noproxy", "*", "-sS", "--fail", "--max-time", "180",
                        "-F", f"firmware=@{pc.APP_BIN}", f"http://{ip}/update"],
                       capture_output=True, text=True)
    if r.returncode != 0:
        return False, (r.stderr or r.stdout).strip()[-200:]
    time.sleep(8)
    for _ in range(30):
        try:
            st = pc.http(ip, {"T": 164}, timeout=2)
            if st:
                return True, f"{st.get('fw')} slot={st.get('part')}"
        except Exception:
            time.sleep(1)
    return False, "did not come back within 40 s"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--force", action="store_true")
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args()
    want = pc.expected_fw()
    cars = pc.read_cars()
    print(f"build: {want}; cars in cars.txt: {len(cars)}; scanning ...", flush=True)
    found = locate()
    rows, bad = [], 0
    for c in cars:
        hit = found.get(c["mac"])
        if not hit:
            rows.append(f"{c['id']:6s} offline")
            continue
        if hit["ip"] != c["ip"]:
            pc.write_car(c["id"], hit["ip"], c["mac"])
        line = f"{c['id']:6s} {hit['ip']:16s} {hit['fw']}  {hit['v'] or 0:.1f} V"
        if args.list or (hit["fw"] == want and not args.force):
            rows.append(line + ("" if args.list or hit["fw"] != want else "  (up to date)"))
            continue
        print(f"{c['id']}: OTA {hit['fw']} -> {want} at {hit['ip']} ...", flush=True)
        ok, msg = ota(hit["ip"])
        bad += not ok
        rows.append(f"{c['id']:6s} {hit['ip']:16s} " + (f"OK  {msg}" if ok else f"FAILED: {msg}"))
    unknown = [f"{m} @ {h['ip']} ({h['fw']})" for m, h in found.items()
               if m not in {c["mac"] for c in cars}]
    print("\n".join(rows))
    if unknown:
        print("not in cars.txt: " + ", ".join(unknown))
    online = len(found)
    if online >= pc.HOTSPOT_MAX:
        print(f"note: {online} cars online; the Windows hotspot takes at most {pc.HOTSPOT_MAX}.")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
