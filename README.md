# rover_net

WAVE ROVER 多车控制与树莓派节点执行系统。

当前架构：电脑控制台 → SSH/IP → 每车树莓派常驻服务 → 有线串口 → ESP32 底盘。
在线协调多车步骤；电脑失联后，节点继续执行已预装的完整任务，并明确标记同步降级。

## 开始使用

```sh
cd raspberry_pi_control
python -m pip install -r requirements.txt
python fleet_ui.py
```

控制台默认地址为 http://localhost:8090。需要先部署树莓派节点服务，并确认本车底盘接线与MAC。
默认示例未配置底盘串口，不会自动启动运动；TF串口不能作为底盘端口。

- [安装与使用](raspberry_pi_control/README_PI.md)
- [节点协议](raspberry_pi_control/docs/NODE_PROTOCOL.md)
- [系统架构与任务时序（draw.io）](raspberry_pi_control/docs/node_architecture.drawio)
- [节点服务](raspberry_pi_control/node_service.py)
- [电脑端协调器](raspberry_pi_control/node_fleet.py)

## 测试

```sh
cd raspberry_pi_control
python sim/test_nodes.py
```

14项测试覆盖八节点仿真、同步与延时、完整编排、确认丢失、断线续跑、重连去重、重启中断与停车。
这些测试不打开实际串口，不代表实车运动精度。

## 配置与版本边界

- `fleet_nodes.json`、`node_config.example.json` 是初始配置示例，部署前按现场修改。
- 旧固件、标定与控制脚本保留，旧电脑直连版本入口为 `fleet_ui_legacy.py`。
- TF实际通信适配、定位和避碰尚未实现；需要TF模块型号与协议后进一步开发。
- 历史实验热点凭据已从此公开副本移除。旧固件上车工具改用环境变量
  `ROVER_WIFI_SSID`、`ROVER_WIFI_PASSWORD` 或显式命令行参数。
- 私钥、现场部署日志、原始文档附件、运行状态、测试缓存和部署压缩包未纳入仓库。
- 本次公开副本未修改原工作目录和树莓派的运行配置。
