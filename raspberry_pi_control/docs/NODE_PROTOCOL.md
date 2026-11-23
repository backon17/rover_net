# 节点协议 v1

电脑控制台 → SSH 本地端口转发 → 树莓派 `127.0.0.1:8765` → 单一执行器 → 底盘串口。
仅使用 Python 标准库和已有 pyserial；不需要 MQTT、云服务或外部数据库。

## 请求与身份

POST `/v1/{method}`，UTF-8 JSON，最多 2 MB。所有请求带 `version: 1`、
`request_id`。除 status/prepare 外均带当前 `run_id`。成功回复含 `version`、
`node_id`；错误为 HTTP 400/500 与 `error`。电脑核对登记的 node_id。
身份来自 SSH 目标和节点配置；底盘 MAC 则在串口握手时独立核对。

| method | 参数与作用 |
|---|---|
| status | 无任务参数。返回 boot_id、monotonic、phase、run_id、底盘状态、步骤、阶段、due、降级标志与日志 |
| prepare | run_id、stages、sync。stages 为本车每阶段 motion 或 null，立即返回 accepted；异步静止标定后进入 ready |
| commit | run_id、start_at、stage_bases。start_at 是该节点本次启动的单调时钟时间，stage_bases 是全队计算的阶段累计预算 |
| heartbeat | run_id、seq、vt、limit、degraded。更新虚拟计划时钟；乱序 seq 不被接受 |
| stop | run_id。置取消事件、立即发送 T:161 和零速度，回复 stop=confirmed/unknown |
| calibrate | run_id、measured。使用最后一次单段直行和实测距离拟合本车速度模型，仅空闲时允许 |

motion 保留 `steps:[{t:"F"或"T",v:数值}]`、speed、delay、start{x,y,th}。
speed=0.08..0.45 m/s，delay=0..600 s，每阶段最多100步，F绝对值≤20m、T绝对值≤720°。
每任务最多100阶段。起点仅用于展示，不提供定位或避碰。

## 生命周期与去重

idle → preparing → ready → waiting/sync/straight/turn → done。
失败进入 error，明确取消进入 stopped。重启把未完成任务标为 interrupted，绝不自动运行。
任务和进度原子保存到 state_file；启动 ID、历史任务 ID 用于防止重放。

同一 request_id 和同一内容返回原结果；ID复用但内容不同被拒绝。
已 commit 的任务不能再 commit，即使改用新 request_id。历史 run_id 不能重新 prepare。
请求结果缓存4096条，持久化最近10000个 run_id；请求重放保护不是无限期审计存储。

prepare、commit 分离。全部 ready 后电脑取5个时钟样本，以最小RTT样本估计时钟偏移；
RTT>100ms或样本抖动>50ms时拒绝启动。共同起点留出2秒。
commit 部分确认缺失时标为 partial，并尽力取消；分区网络不具备原子广播保证。

## 同步与离线

直行时隙=本车标定预计时间+0.6秒；转向=1.35+0.015×角度绝对值。
阶段长度取各参与车计划结束时间的最大值。完整编排先预装，再共同启动。
电脑取每车下一未完成动作的 end 最小值限制虚拟时钟，因此快车等待慢车。
控制更新间隔约50毫秒加一次并行网络往返；状态后台约0.5秒采样。

3秒未收到有效协调更新，本车锁存 degraded，自主继续剩余任务。
电脑发现任意节点失联或降级后，通知其余可达节点降级。
降级保留最后确认的计划时钟偏移，不跳步、不重放；重连不恢复本次任务的严格同步。
不新增电脑失联即停车的运动租约。底盘现有有限时长动作与故障机制保留。

关闭网页或控制台不停止任务；显式 stop 才取消。离线节点的停车结果显示 unknown。
本地底盘故障会尝试停车并终止，在线协调器也会尝试停止其他参与车；
失联车辆无法保证收到这一停止请求。

## TF 接入边界

NodeClient.request 是传输适配边界。首版适配器为 SSH/IP；回环 HTTP 用于本机 CLI 和仿真。
TF 若提供IP接入，保持协议并更换SSH节点地址；若为串口，需要根据厂商协议开发
寻址、分帧与收发适配器，并验证本协议所需的消息尺寸、时延与重试行为。
当前仅确认TF是CH340串口，尚未实现TF组网、路由或命令适配。
