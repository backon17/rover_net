"""将两排八辆 Wave Rover 并成向下的一列纵队，通过现有 fleet_ui 控制台执行。"""

import argparse
import math
import time

from fleet_go import api
from fleet_ui import MAX_SEG_M


# 图中向下为 +x，向右为车头左侧（+y）。所有车的初始航向为 0°。
REAR_ROW = (1, 2, 3, 4)  # 从图中右侧往左
FRONT_ROW = (5, 6, 7, 8)
QUEUE = FRONT_ROW + REAR_ROW  # 从队首到队尾
CAR_IDS = [f"car{n}" for n in QUEUE]


def build_plan(column_gap, row_gap, queue_gap, speed):
    start = {n: (x, -i * column_gap)
             for row, x in ((REAR_ROW, 0.0), (FRONT_ROW, row_gap))
             for i, n in enumerate(row)}
    # 前排先前移至少一车位，后排最后一辆留在原纵向位置。
    head_x = max(7 * queue_gap, row_gap + 4 * queue_gap)
    end_x = {n: head_x - i * queue_gap for i, n in enumerate(QUEUE)}

    def job(n, x, y, steps):
        return {"id": f"car{n}", "start": {"x": x, "y": y, "th": 0},
                "steps": [{"t": kind, "v": value} for kind, value in steps],
                "speed": speed, "delay": 0}

    stages = []
    for row in (FRONT_ROW, REAR_ROW):
        straight = []
        for n in row:
            x, y = start[n]
            distance = end_x[n] - x
            if distance >= 0.01:
                straight.append(job(n, x, y, [("F", distance)]))
            else:
                # 控制台会滤掉小于 1 cm 的直行段。
                end_x[n] = x
        stages.append(straight)
    stages.append([job(n, end_x[n], start[n][1],
                       [("T", 90), ("F", -start[n][1]), ("T", -90)])
                   for n in QUEUE if start[n][1] < 0])
    return start, end_x, stages


def checked_state():
    state = api("/api/state")
    cars = {car["id"]: car for car in state["cars"]}
    offline = [cid for cid in CAR_IDS if not cars.get(cid, {}).get("connected")]
    if offline:
        raise RuntimeError(f"未连上控制台的车辆：{', '.join(offline)}")
    return state, cars


def run_plan(stages):
    """Preload the complete choreography instead of uploading one stage at a time."""
    state, _ = checked_state()
    if state['running']:
        raise RuntimeError('控制台已有任务')
    starts = {}
    for stage in stages:
        for job in stage:
            starts.setdefault(job['id'], job['start'])
    plan = dict(version=1, name='八车合并纵队', starts=starts,
                stages=[dict(name=f'阶段 {i+1}', cars=jobs) for i, jobs in enumerate(stages) if jobs])
    result = api('/api/choreography/start', plan)
    if result.get('error'):
        raise RuntimeError(result['error'])
    try:
        while True:
            try:
                state = api('/api/state')
            except OSError:
                print('电脑控制台不可达；已提交的任务由树莓派继续。停车结果未知。', flush=True)
                time.sleep(2)
                continue
            group = state.get('group') or {}
            if group.get('phase') in ('error', 'partial', 'stopped'):
                raise RuntimeError(group.get('error') or group['phase'])
            if group.get('phase') == 'done':
                return
            time.sleep(.5)
    except KeyboardInterrupt:
        try:
            result = api('/api/stop', dict(ids=CAR_IDS), timeout=8)
            print(result.get('error') or '节点已确认停车')
        except OSError:
            print('停车结果未知，请现场检查')
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--column-gap", type=float, default=0.50, help="初始横向中心距（米）")
    parser.add_argument("--row-gap", type=float, default=0.50, help="初始两排中心距（米）")
    parser.add_argument("--queue-gap", type=float, default=0.50, help="成队后前后中心距（米）")
    parser.add_argument("--speed", type=float, default=0.18, help="直行速度（米/秒）")
    parser.add_argument("--run", action="store_true", help="向正在运行的控制台发送轨迹")
    args = parser.parse_args()
    gaps = (args.column_gap, args.row_gap, args.queue_gap)
    if any(not math.isfinite(gap) or gap < 0.35 for gap in gaps):
        parser.error("中心距须至少为 0.35 m（控制台的碰撞预警距离）")
    if not 0.08 <= args.speed <= 0.45:
        parser.error("速度须为 0.08–0.45 m/s")
    if max(gaps) > MAX_SEG_M:
        parser.error("中心距超出当前控制台轨迹范围")
    start, end_x, stages = build_plan(*gaps, args.speed)
    if any(step["t"] == "F" and step["v"] > MAX_SEG_M for stage in stages
           for job in stage for step in job["steps"]):
        parser.error(f"单段直行距离超过控制台上限 {MAX_SEG_M:g} m")

    print("初始排列（车头向下）\n4 3 2 1\n8 7 6 5")
    print("成队顺序（队首到队尾）：5 → 6 → 7 → 8 → 1 → 2 → 3 → 4")
    print("坐标单位 m，x 向下，y 向右，1 号车起点为原点")
    for n in QUEUE:
        x, y = start[n]
        print(f"car{n}: ({x:.2f}, {y:.2f}) → ({end_x[n]:.2f}, 0.00)")
    for index, jobs in enumerate(stages, 1):
        commands = [f"{job['id']} " + ",".join(f"{s['t']}{s['v']:g}" for s in job["steps"])
                    for job in jobs]
        print(f"第 {index} 段：" + " | ".join(commands))
    if not args.run:
        print("预览完成。实车执行请加 --run。")
        return 0
    try:
        run_plan(stages)
    except KeyboardInterrupt:
        return 130
    except (OSError, RuntimeError) as exc:
        print(f"编队中止：{exc}")
        return 1
    print("三段轨迹均已执行完成。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
