# Wave Rover 多车平台 · 交接说明（2026-11-16）

先读这一份，技术细节和每个固件版本的改动记录见同目录 `README_V2.md`。

## 1. 项目是什么

总目标（原始要求见 `docs/项目要求.docx`）：用多辆小车做"无中心、多跳"的无线自组网实验平台。
每辆车是一个会动的网络节点，按信号强度阈值 x 决定谁和谁算邻居，演示组网、拓扑更新、路由恢复，
网页上显示车的位置、网络拓扑和性能曲线。

本包交接的是其中的**小车运动部分**：8 辆微雪 WAVE ROVER，用 Windows 电脑一键让多辆车同时按预定轨迹运行。
组网协议、定位（UWB）、树莓派上车这几块**还没开始**。

## 2. 现在能做什么

- 8 辆车都刷了自研固件 **v2.4.3-sync**，支持 WiFi 无线刷机（OTA），日常升级不用拆车。
- 固件在车上闭环：原地转向（陀螺仪，90° 约 1.2~2.5 s，误差 ≤1.5°）、直行航向保持（平均误差 < 1°）。
- 网页控制台 `fleet_ui`：右边编辑每辆车的步骤（前进 x m / 左右转 x°），左边看计划路线和估计位置，
  一键"同时出发"或"按延时错开出发"，**每一步对齐**（所有车同时开始每个转弯和每段直行），空格/Esc 急停。
- 新车一键上车脚本 `provision_car.bat`（USB 插上运行一次，约 2 分钟），批量升级 `ota_all.bat`。
- 已验证：8 车并排同时跑 0.6 m / 0.8 m 左转正方形、5 车 "直行 0.5 m → 右转 → 直行 2 m"。

## 3. 硬件和网络

| 项目 | 说明 |
|---|---|
| 车 | 微雪 WAVE ROVER：ESP32 驱动板（General Driver for Robots，TB6612FNG 电机驱动，QMI8658 IMU，INA219），4× N20 12V 电机（**无编码器**），3×18650 |
| 刷机口 | 车顶黑色小盖板（带 40 针排母）下面驱动板上标 `USB` 的 Type-C（不是 `LIDAR`，也不是电池仓）。CP2102 自动进下载模式，不用按键。驱动：微雪 CP210x |
| 网络 | Windows"移动热点" `YOUR_WIFI_SSID` / `YOUR_WIFI_PASSWORD`，必须 2.4 GHz（ESP32 不支持 5G），网段 192.168.137.x。车同时开自己的后备热点 `UGVn` / `12345678`（192.168.4.1） |
| 热点限制 | Windows 热点**最多 8 台设备**，现在正好 8 辆车。超过 8 辆必须换路由器（建议 Cudy TR3000，Type-C 供电；关双频合一、固定信道、按 MAC 绑定 IP） |

车辆登记（`cars.txt`，IP 由热点 DHCP 分配，**会变**；控制台按 MAC 自动找回）：

| 车 | MAC | 后备热点 |
|---|---|---|
| 车 1 | 88:f1:55:9c:45:f0 | UGV1 |
| 车 2 | 88:f1:55:8f:90:8c | UGV2 |
| 车 3 | ec:c9:ff:f1:3e:94 | UGV3 |
| 车 4 | 88:f1:55:8f:89:b8 | UGV4 |
| 车 5 | ec:c9:ff:f0:d7:30 | UGV5 |
| 车 6 | 30:76:f5:7d:e3:34 | UGV6 |
| 车 7 | ec:c9:ff:f0:e6:cc | UGV7 |
| 车 8 | ec:c9:ff:ef:f5:ec | UGV8 |

建议每辆车贴标签"车号 · MAC 后 4 位"，例如"2 · 908c"。

## 4. 快速上手（Windows）

运行目录是 `D:\PHD\wave_rover\`（本包的目录结构和它一致，代码文件平铺，固件在 `firmware_v2\`）。
需要 Python 3 + `pip install pyserial`，以及 Arduino ESP32 工具链（`arduino-cli` + esptool，只在编译/USB 刷机时用）。

1. **开热点**：`powershell -ExecutionPolicy Bypass -File tools\hotspot_on.ps1`（或系统设置里打开移动热点，频段选 2.4 GHz）。
   **先开热点，再给车开机**；车开机时连不上热点会停在离线状态，需要重新开机。
2. **开控制台**：双击 `fleet_ui.bat`，浏览器打开 http://localhost:8090 。车离线时控制台每 15 s 自动按 MAC 扫描一次，也可点"扫描小车"。
3. **设轨迹**：右侧每辆车一个标签页：选模板（直线 / L / 正方形 / 往返…）→ 改步骤 → 速度（默认 0.25 m/s）和延时 → 起点。
   左侧可以拖动起点。当前保存的配置是 8 车 0.6 m 左转正方形（在 `fleet_config.json`，见第 6 节）。
4. **出发**：左侧"同时出发（N 辆）"。车先静止约 1 s 测陀螺零偏，**这段时间别碰车**。勾选了但离线的车会弹窗提醒先拿出场地。
5. **急停**：空格 / Esc / 右上角红色按钮。

命令行等价操作：

```bat
python fleet_go.py --cars car1,car2,car3 --steps "F0.6,T90,F0.6,T90,F0.6,T90,F0.6,T90" --speed 0.25
python fleet_go.py --stop
python wave_rover_traj.py turn 90 --ip 192.168.137.91          :: 单车原地转
python provision_car.py --check car3 --turn                     :: 单车 WiFi 自检 + ±90° 转向
```

步骤格式：`F1.0` 前进 1 m，`T-90` 右转 90°（左正右负，每步是相对转角）。
程序内部把转角累加成相对出发时车头的绝对目标，所以上一段直行的残余偏差会在下一个弯里自动补掉，不会越积越多。

## 5. 刷固件

| 场景 | 做法 |
|---|---|
| 改了固件，升级所有在线车（最常用） | `build_v2.bat`（编译到 `firmware_v2\out\`）→ `ota_all.bat`（按 MAC 逐台 OTA，只升版本不对的） |
| 升级单辆 | `ota_flash.bat <车IP>`，或浏览器打开 `http://<车IP>/update` 上传 bin |
| 新车第一次（必须 USB） | 拆盖插驱动板 `USB` 口 → 双击 `provision_car.bat`：自动找 COM 口、检查分区表、备份出厂固件到 `backups\`、刷入、配 WiFi、分配车号、登记、自检 |
| WiFi 起不来的救车 | `flash_v2.bat COMx`（USB，会重置 otadata 保证从 app0 启动）；回滚 `flash_v2_rollback.bat COMx <备份bin>` |

刷完用 `T:164` 状态帧（控制台卡片也会显示）确认 `fw` 版本、`part` 运行分区（OTA 后在 app0/app1 之间切换）。
固件版本号在 `firmware_v2\turn_ctrl.h` 的 `FW_VERSION`，改了代码请同步改它，`ota_all` 靠它判断要不要升级。

## 6. 本包里有什么 / 没有什么

| 路径 | 内容 |
|---|---|
| `firmware_v2/` | 固件源码（基于微雪官方 WAVE ROVER 固件改）。自研部分主要在 `turn_logic.h`（纯控制逻辑：转向、直行、零偏，固件和仿真共用）、`turn_ctrl.h`（接硬件、指令 T:160~165、状态帧）、`http_server.h`（OTA `/update`）、`IMU.cpp`（AHRS 采样周期 bug 修复）、`QMI8658.cpp`（开片上低通） |
| `fleet_ui.py` / `fleet_ui.html` / `fleet_ui.bat` | 多车网页控制台（后端 + 页面） |
| `fleet_go.py` | 命令行多车下发、急停、速度标定 |
| `wave_rover_traj.py` | 单车底层驱动库 + 命令行（`fw-status`、`turn`、`calib-gs`、`waypoints` 等），控制台也 import 它 |
| `provision_car.py/.bat`、`ota_all.py/.bat` | 新车上车、批量 OTA |
| `cars.txt` | 车号 / 最近 IP / MAC 登记表 |
| `rover_calib.json` | 默认标定参数（每辆车的个体标定在 `rover_calib_<mac>.json`，Windows 上生成） |
| `plan_*.json` | 旧版路径点格式的示例轨迹 |
| `check_fleet.py`、`lat_probe.py`、`gz_probe.py`、`gn_probe.py`、`wall_test.py`、`diag_straight.py`、`probe_ip.py` | 诊断小工具（扫描、延迟、陀螺噪声、堵转电流等） |
| `sim/` | Mac/Linux 仿真：`fake_v2.py` 假小车（用真实 `turn_logic.h`），`test_fleet_ui.py`、`test_fleet8.py`、`test_lockstep.py`、`test_v2_pc.py` 回归测试，`*_bench.py` 控制器调参。先编译：`c++ -O2 -std=c++17 -shared -fPIC -I../firmware_v2 turn_capi.cpp -o libturn.so` |
| `docs/` | 项目原始要求、控制台截图 |

**只在 Windows `D:\PHD\wave_rover\` 上、本包里没有的**（打包时那台电脑离线，拿不到；交接时请一起拷过去，或用第 8 节的命令在 Windows 上重新打一个完整包）：

- 刷机/编译脚本：`build_v2.bat`、`ota_flash.bat`、`flash_v2.bat`、`flash_v2_rollback.bat`
- `tools\`：`hotspot_on.ps1`、`hotspot_restart.ps1`、`ics_list.ps1`、`ics_disable.ps1`、`serial_cmd.py`、`wifi_*.json`
- 当前配置：`fleet_config.json`（8 车 0.6 m 正方形；`fleet_config_square.json`/`fleet_config_square06.json` 是另存）、`rover_calib_<mac>.json`（每车标定）、最新 `cars.txt`
- 编译产物 `firmware_v2\out\`、出厂固件备份 `backups\`、各版本备份 `bak_*`/`firmware_v2*_src\`/`backup_*.bin`

## 7. 已知问题和下一步

1. **各车直行速度不一致**（最明显的问题）。原因：PWM 开环、0.25 m/s 只比电机死区高一点，电池电压差（10.8~12.4 V）和死区差被放大。
   - 速度标定功能已写好但**还没在真车上做**：车并排 → `python fleet_go.py --calib-speed --cars car1,...,car8` 各走 1 m → 量实际距离 → `python fleet_go.py --set-speed car1=0.93,car2=...`，0.25 和 0.35 m/s 各做一次效果最好。
   - 还没做：固件电池电压补偿（PWM × 12 V / 实际电压）。
   - 根治：换带霍尔编码器的 N20 电机，驱动板和官方固件本来就有编码器接口和轮速 PID（`movtion_module.h`）。
2. **跑前充电**。低于 9.9 V 不要跑；电压低的车更慢，排成一列时会被追尾。车 1、车 4 掉电相对快。
3. **没有位置反馈**。位置靠"时间 × 速度"推算，长距离会累积误差；50 m 室内场地需要 UWB 定位（GPS 室内不可用）。
4. **陀螺零偏个体差异大**：车 2 约 -5°/s、车 5 约 -38°/s、车 8 约 +8°/s，扣除后都稳定（残余漂移 ≤0.14°/s），能用，但出发前测零偏时必须静止。
5. **直行堵转检测（INA219 电流）实测无效**：低速顶墙时电池电流不升高，保持开启但基本不触发；撞墙后车会顶到该段时间结束。
6. **陀螺刻度标定 `calib-gs` 没做过**（需要人目测转两圈后的误差），转角大时可能有 1~2% 误差。
7. 不要加"电脑定时续约、车收不到就停"的机制：原负责人明确否决过（WiFi 一抖全车停）。
8. 超过 8 辆车需要路由器，见第 3 节。

## 8. 在 Windows 上打完整包

那台电脑上线后，在 PowerShell 里运行下面的命令，得到包含第 6 节全部内容（不含体积大的备份）的压缩包：

```powershell
cd D:\PHD
$items = Get-ChildItem wave_rover | Where-Object { $_.Name -notmatch '^(backups|bak_|firmware_v2\d|backup_)' }
Compress-Archive -Path $items.FullName -DestinationPath D:\PHD\wave_rover_handoff_full.zip -Force
```
