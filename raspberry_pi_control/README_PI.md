# 树莓派节点控制版（2026-11-23）

默认链路：电脑网页/命令行 → SSH → 树莓派常驻节点服务 → 本地数据串口 → ESP32。
每次先预装完整任务；在线协调多车步骤，断线3秒后本车自主继续整条任务。
断线续跑不保证队形、步骤同步或避碰。重连不重放，服务重启不自动恢复运动。

## 当前硬件与启动

rover01 的唯一 CH340 串口是用户确认的 TF 模块，禁止配置为底盘端口。
`node_config.example.json` 默认 serial_port/expected_mac 均为空，服务可查询但不能运动。
树莓派系统仍从联想 USB U盘运行，运行时不能拔系统盘。
必须另接树莓派到驱动板的数据线，确认串口、底盘MAC及本车标定后再配置。
首版支持已有V2.3+板载直行固件，不自动刷机或退回底盘Wi-Fi控制。

电脑：

```powershell
cd D:\hebang_all\小车组网\raspberry_pi_control
python fleet_ui.py
# 浏览器 http://localhost:8090
python fleet_go.py --cars car1 --steps "F0.3,T90,F0.3" --speed 0.18
python fleet_go.py --stop
```

上面的运动命令只在底盘接线、身份和场地确认完成后使用。日常启动只运行 fleet_ui.py。
登记文件为 fleet_nodes.json；每辆车设置不同 car_id、node_id、ssh_host。
当前 car1 只是 rover01 的控制台占位名称，不表示已确认它接到了原车1。
新增车辆时复制记录并配置相应树莓派，不能复制相同 node_id。
修改连接地址保存到 ssh_host；服务端口默认8765，电脑端监听默认8090。
浏览器运行状态会显示底盘未配置、节点失联、停车未知和同步降级。

树莓派安装（部署路径 /home/rover/rover_control）：

```sh
sh install_node.sh
systemctl status rover-node.service
journalctl -u rover-node.service -n 80 --no-pager
.venv/bin/python pi_control.py status
.venv/bin/python pi_control.py devices
.venv/bin/python pi_control.py run pi_plan_example.json --dry-run
```

安装脚本需要sudo；首次创建 node_config.json，之后不覆盖已有配置。
虚拟环境使用系统已装pyserial；未安装时在虚拟环境内运行 pip install -r requirements.txt。
服务以rover运行、开机自启、仅监听127.0.0.1。启动服务本身不打开底盘或TF串口。
配置 serial_port、expected_mac、calibration 后重启服务，先读状态，再现场验证短距离动作。
同型号USB串口可能没有唯一序列号，应使用物理by-path固定接线并核对MAC。

CLI status/stop/run 均通过服务，不再支持 --port 直接抢占底盘串口。
CLI run 提交后退出，任务由服务继续；停车使用 pi_control.py stop。
电脑到树莓派已有专用免密密钥；请先验证 ssh rover01，不要把私钥复制到项目或树莓派。

## 验证

```powershell
python sim/test_nodes.py
python -m compileall -q node_service.py node_client.py node_fleet.py node_protocol.py node_executor.py pi_control.py
```

集成测试使用假底盘与真实HTTP，不打开任何硬件，覆盖多节点同步、延时、完整编排、
丢失启动确认、单向断线、重连、重启、去重、标定与停车。模拟结果不等于实车运动精度。

协议与状态定义见 docs/NODE_PROTOCOL.md；架构与时序见 docs/node_architecture.drawio。
现场部署记录保留在本地，不随公开源码上传。

## 回滚与边界

旧电脑直连版保存在 fleet_ui_legacy.py；原始包 wave_rover_handoff_20261116 保留。
回滚前确认全部车辆已停，执行 sudo systemctl disable --now rover-node.service，
再使用原始包中的脚本与配置。不要同时启用两条底盘控制路径。
源码升级先备份旧代码，勿覆盖 node_config.json、state/、各车标定或.venv。
TF协议、电脑TF节点接入、定位与避碰不包含在首版；TF适配待型号与说明书明确后开发。
