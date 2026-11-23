# WAVE ROVER 固件 V2 + PC 端配套改动

## 改了什么

固件（`firmware_v2/`，基于 `firmware_patched/`，原目录未动）：

1. **修 AHRS 采样周期 bug**（`IMU.cpp`）：原代码写死 `halfT = 0.024f`（假设 48 ms 解算一次），
   实际每圈 `loop()`（约 5 ms）都解算，转动时航向 `y` 被放大约 10 倍、停车后被磁力计拉回。
   改为用 `micros()` 实测间隔。
2. **板载陀螺航向 `gy`**（`turn_logic.h` / `turn_ctrl.h`）：每圈 `loop()` 用真实间隔梯形积分 z 轴陀螺，
   不依赖磁力计；开机自动测 1 s 零偏，也可用 `T:162` 手动标定。
3. **板载闭环原地转向 `T:160`**：控制环在 ESP32 上以 loop 频率运行，不受 WiFi 延迟影响。
   起步冲击（克服静摩擦）→ 巡航 → 接近目标减速 → 提前按预测滑行量短路制动 →
   停稳后小角度定时脉冲修正；滑行系数和脉冲增益每次自动学习。
4. **安全任务改为短路制动**（原来 `switchEmergencyStop()` 实际是滑行），冻结重启阈值 15 s → 30 s。
5. 反馈帧 `T:1001` 增加 `gy`、`ts`；新状态帧 `T:1010` 含主循环最大耗时 `lp`（诊断卡顿用）。

新指令：

| 指令 | 作用 |
|---|---|
| `{"T":160,"deg":90,"spd":0.30,"tmo":8}` | 原地转 90°（左正右负），`spd` 巡航 PWM，`tmo` 超时秒 |
| `{"T":161}` | 取消转向并刹车 |
| `{"T":162,"ms":1000,"zero":1}` | 静止标定陀螺零偏并航向清零 |
| `{"T":163,"gs":1.0,"kick":0.4,"minp":0.22}` | 设参数（一次最多约 6 个字段） |
| `{"T":164}` | 返回状态帧 `T:1010` |
| `{"T":1,...,"fb":1}` | 轮速指令同时返回状态帧 |

任何 `T:1 / T:11 / T:13` 运动指令都会打断正在进行的板载转向。

PC 端（`wave_rover_traj.py`，旧版备份为 `wave_rover_traj.v1.py`）：

- 自动探测 V2 固件；是 V2 就把转向交给固件、直行用 `gy` 保持航向，去掉每个拐角 3~12 s 的等待
- 修 `run_waypoints` 把绝对方位角当相对转角的 bug（正方形第 3、4 个弯原来会转 180° / -90°）
- 修直行里程积分（用上一周期实际下发的指令积分，原来加速段会多算、提前停车）
- 串口模式把连续反馈限速到 50 ms/帧（不限速会拖慢固件主循环）
- 新命令：`fw-status`、`turn <角度>`、`calib-gs`

## 刷机

```bat
cd /d D:\PHD\wave_rover
flash_v2.bat COM5            :: 先自动备份当前 app 分区，再刷 V2
flash_v2_rollback.bat COM5 backup_app_xxx.bin   :: 回滚
```

只写 app 分区（0x10000），WiFi 配置 / LittleFS 不动；分区表与官方 V0.9 逐字节一致。

## 上车测试顺序（建议先 USB 串口，排除 WiFi 变量）

```bat
python wave_rover_traj.py COM5 fw-status        :: 看 gy 静止时是否基本不漂、lp（主循环最大 ms）
python wave_rover_traj.py COM5 turn 90          :: 单独测左转 90°
python wave_rover_traj.py COM5 turn -90
python wave_rover_traj.py COM5 calib-gs         :: 连转 720°，目测多/少转多少度，自动算陀螺刻度
python wave_rover_traj.py COM5 waypoints plan_L.json --speed 0.25
```

WiFi 模式把 `COM5` 换成 `--ip 小车IP` 即可。

## 仿真（在 Mac 上跑过，见 sim/）

- `turn_sim.cpp`：固件同一份 `turn_logic.h` + 滑移转向物理模型（死区、静摩擦、短路制动、陀螺噪声/零偏/刻度误差、
  主循环随机卡顿），每种角度 300 次蒙特卡洛
- `ahrs_sim.cpp` + `extract_ahrs.py`：从新旧 `IMU.cpp` 原样抽出 AHRS 函数对比
- `fake_v2.py` + `test_v2_pc.py`：用真实 `turn_logic.h`（ctypes）做的假 V2 小车，端到端跑 PC 端 L 形 / 正方形

编译（需要 C++ 编译器）：
`c++ -O2 -std=c++17 -I../firmware_v2 turn_sim.cpp -o turn_sim`，
`c++ -O2 -std=c++17 -shared -fPIC -I../firmware_v2 turn_capi.cpp -o libturn.so`

## 无线刷机（OTA，v2.1 起）

固件内置 `/update` 上传入口：新固件写入另一个程序分区（app0/app1），完整写入并校验通过后才切换启动，
中途断开不影响当前固件。上传期间电机自动停止。

```bat
cd /d D:\PHD\wave_rover
build_v2.bat                 :: 编译 firmware_v2 -> firmware_v2\out
ota_flash.bat                :: 用 car_ip.txt 里的 IP 无线刷机；或 ota_flash.bat 192.168.1.6
```

也可以浏览器打开 `http://<小车IP>/update` 选文件上传。刷完用 `T:164` 状态帧里的
`fw`（版本）、`bd`（编译时间）、`part`（当前运行分区）确认。

前提：电脑能访问到小车 IP（同一局域网、路由器未开无线客户端隔离）。

USB 刷机（`flash_v2.bat COMx`）现在会备份 otadata+app0+app1（`backup_ota_*.bin`），
并清除 otadata，保证 USB 刷入的 app0 一定被启动（否则 OTA 后从 app1 启动，USB 刷 app0 不生效）。

## WiFi（电脑热点方案）

酒店/公共路由器通常开了无线客户端隔离，电脑访问不到小车，因此改用 **Windows 移动热点**：
电脑继续连外网 WiFi，同时开 2.4 GHz 热点 `YOUR_WIFI_SSID` / `YOUR_WIFI_PASSWORD`，小车连这个热点。

- 小车 IP：`192.168.137.229`（写在 `car_ip.txt`，`ota_flash.bat` 默认读取）
- 小车配置为 AP+STA：STA 连 `YOUR_WIFI_SSID`，自身热点 `UGV` / `12345678`（192.168.4.1）保留作后备
- 电脑重启后若热点没开：`powershell -ExecutionPolicy Bypass -File tools\hotspot_on.ps1`
- 热点开了但小车连不上、热点网卡（WLAN 3）没有 192.168.137.1：通常是旧的"网络共享"占用了该地址，
  运行 `tools\ics_list.ps1` 查看、`tools\ics_disable.ps1` 清除后再 `tools\hotspot_restart.ps1`
- 串口改小车 WiFi：`python tools\serial_cmd.py COM3 0.5 22 @tools\wifi_pc.json`

## V2.3 firmware: on-board straight driving (T:165)

Why: in v2.2 the straight-line heading hold ran on the PC over WiFi HTTP. Every
command waited for the car's reply (100-300 ms, sometimes more); a gap longer than
the 400 ms heartbeat braked the motors (stutter), and the slow, delayed correction
plus P-only control near the motor dead zone let the line drift.

Now:
- `{"T":165,"ms":5500,"spd":0.16,"hd":0,"ramp":500}` drives straight for `ms` with a
  PID on the gyro heading at loop rate (~200 Hz); both wheels always keep at least
  50 % of the base PWM, soft start from 40 %. Ends by itself even if WiFi drops.
  `ss` state (1 running, 2 done, 3 aborted, 4 fault >30 deg), `sp` progress,
  `sa`/`sm` mean/max heading error are in the T:1010 status frame.
- `WiFi.setSleep(false)` (no modem power-save latency).
- Gyro bias calibration also reports `gn` (gyro noise); the PC re-calibrates if the
  car was moving (`fw_still_noise`, default 2.5 dps).
- Straight gains can be changed live: `{"T":163,"skp":0.010,"ski":0.006,"skd":0.0008,"scf":0.5,"sr0":0.4}`.
- PC (`wave_rover_traj.py`) uses T:165 automatically when the car reports `ss`;
  v2.2 cars keep the old PC loop.
- Sim A/B (`sim/ab_straight.py`, 1 m square, heavy-tailed WiFi lag): v2.2 stutters
  3-6x per square and drifts up to 21 cm off the line; v2.3: no stutter, <= 2.4 cm.

## Multi-car console (fleet_ui)

Double-click `fleet_ui.bat` (or `python fleet_ui.py`) -> browser opens http://localhost:8090.

- Left "场地实况": shared map with each car's start box, planned route (dashed),
  travelled route (solid, estimated) and live pose (gyro heading); per-car progress
  bars below, then the launch buttons and a collision check for both start modes.
  Drag a start box to move it, drag the small circle in front of it to rotate.
- Right "轨迹设置": per-car tabs. 1) template (line / L / square / rectangle /
  out-and-back / Z, or import a plan_*.json), 2) editable step list
  (前进/后退 m, 左转/右转 deg), 3) speed + start delay, 4) start pose.
- Steps are what the car executes: `{"t":"F","v":1.0}` drive, `{"t":"T","v":-90}` turn
  (left positive). Turn targets are absolute in the gyro frame zeroed at start, so
  turns absorb the previous segment's residual error.
- "同时出发" ignores delays; "按各车延时错开出发" departs car i at T0 + delay_i.
  All selected cars first calibrate the gyro (stand still ~1 s), then a start gate
  fixes T0, so departures line up within ~0.1 s.
- Stop: red button, Space or Esc.
- "扫描小车" probes 192.168.137.x and re-points cars by MAC (hotspot DHCP IPs change).
- State is saved in `fleet_config.json`; first run seeds it from `cars.txt`.
- Tests: `sim/test_fleet_ui.py` (two simulated cars through the HTTP API),
  `sim/ab_straight.py` (straight-control A/B), `check_fleet.py` (read-only scan +
  connect against the real cars). Sim needs `sim/libturn.so` built from
  `sim/turn_capi.cpp` + `turn_logic.h` (macOS/Linux).

## V2.4 (v2.4-stall)

* Straight heading hold gains doubled: `skp` 0.010 -> 0.020, `ski` 0.006 -> 0.012
  (pushed by the PC on connect from `fw_skp` / `fw_ski`; change live with
  `{"T":163,"skp":..,"ski":..}`).
* The straight-line integral (left/right motor trim) is kept between segments
  (per direction, normalised by cruise PWM). Status: `stf` / `stb`.
  Disable with `{"T":163,"skeep":0}`. Lost on reboot (re-learned in one segment).
* Stall detection from the INA219 battery current, read every 25 ms while
  driving straight. Baseline = mean current 0.25-0.75 s after the soft start;
  stall = current > 1.5 x baseline and > baseline + 120 mA for 0.4 s
  -> `ss=5`, motors braked, the PC aborts the plan with a "堵转" error.
  Status: `si` (current), `sib` (baseline), `sip` (peak), all mA.
  Tune: `{"T":163,"ssr":1.5,"ssa":120,"ssms":400,"ssabs":0,"sst":1}`
  (`ssabs` = absolute limit in mA, 0 = off; `sst=0` disables detection).
  Bench (`sim/straight_bench.py`, motor numbers are guesses): detects ~80 % of
  wall hits at 0.25 m/s, ~100 % at 0.30 m/s, only ~20 % at 0.18 m/s; no false
  stops on normal runs; may false-stop when rolling onto very thick carpet.
  A car that is already blocked when the segment starts is not detected.
* Default speed in the console: 0.25 m/s (presets 0.18 / 0.25 / 0.30).
* Backups on the PC: `firmware_v23_src\`, `backup_v23_app.bin`,
  `wave_rover_traj.v23.py`, `fleet_ui.v2.*`.

## V2.4.1 (v2.4.1-glpf)

* QMI8658 on-chip gyro LPF enabled (mode 0, ~27 Hz at 1 kHz ODR). With the
  LPF off, all noise up to the ODR aliased into the ~200 Hz loop: car 2 had
  2.1 dps std on z while still (bias calib `gn` 2.7-4.3, above the 2.5 restart
  threshold). After: `gn` 0.19 dps on car 2. Group delay ~6 ms (negligible).
* Car 2 (192.168.137.219) on v2.4.1, turns +-30/+-90 within 1.1 deg.
  Car 1 still on v2.4 (offline when built): `ota_flash.bat <car1 ip>`.
* Tools: `tools\gz_probe.py <ip>` raw IMU noise, `tools\gn_probe.py <ip>`
  repeated bias calibration noise, `tools\wall_test.py` current log.
* Stall detection result on the real car: battery current does NOT rise when
  blocked at 0.25 m/s (PWM ~0.22 is just above the motor dead zone, and the
  battery sees duty x motor current). Left enabled (no false stops) but
  effectively inactive; user decided to keep it as is.

## v2.4.3: step lock-step + turn-rate floor + per-car speed (2026-11-12)

Why cars collided in the 3-car L test (v2.4.2): departure and straights were
in sync, but the turns took 1.8 s / 4.2 s / 5.1 s on the three cars and each
car started its next straight as soon as its own turn ended.

* **Lock-step (fleet_ui / fleet_go default).** Every step has a time slot
  (straight = on-board duration + 0.6 s, turn = 1.35 s + 0.015 s/deg). A car
  that is early waits ("对齐等待"); a car that overruns holds the shared plan
  clock so the others wait at their next step boundary. Every turn and every
  straight starts on all cars together (or exactly delay apart). Page:
  checkbox "每一步对齐"; CLI: `fleet_go.py --no-sync` to switch it off.
  Limitation with staggered delays: a car that is *behind* in the stagger and
  overruns cannot delay steps that cars *ahead* of it already started.
* **Firmware v2.4.3 turn-rate floor.** In the run phase of an on-board turn
  the gyro rate is compared with a desired rate (60 dps, tapering to 20 dps in
  the slow-down zone); a deficit adds PWM (<= 0.15). Cars with a high dead zone
  no longer crawl. Params via T:163 `trc`, `trm`, `bk`, `bmax`; status `tb` =
  peak boost of the last turn.
* **Straight soft start** now 0.3 s from 80 % of cruise PWM (was 0.5 s from
  40 %, below the dead zone, so each car started moving at a different time).
* **Per-car speed model** `pwm = sv_dz + v / sv_k`, fitted from measured runs:
  `python fleet_go.py --calib-speed --cars car2,car3,car4` (each drives 1 m),
  measure, then `python fleet_go.py --set-speed car2=0.93,car3=0.85,car4=1.04`.
  Repeat once or twice; the 2nd point also fits the dead zone. Stored in
  `rover_calib_<mac>.json` (sv_k, sv_dz, sv_pts).

Sim: `sim/test_lockstep.py` (3 cars, one with 7.5 s turns): step-start spread
without lock-step up to 10 s, with lock-step <= 0.06 s; speed calibration
24 cm -> 1 cm spread after two measurements. `sim/turn_crawl_bench.py`: weak
cars' 90 deg turn 5.1 s -> 2.6 s, same accuracy.
