"""Transport-independent task validation shared by PC, node and CLI."""
import json
import math
import os
from pathlib import Path

VERSION = 1
ACTIVE = {'preparing', 'ready', 'waiting', 'sync', 'straight', 'turn'}
TERMINAL = {'done', 'stopped', 'error', 'interrupted'}


def number(value, low, high, name):
    if isinstance(value, bool):
        raise ValueError(f'{name}: expected number')
    value = float(value)
    if not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'{name}: expected {low}..{high}')
    return value


def motion(spec):
    if not isinstance(spec, dict):
        raise ValueError('motion must be an object')
    speed = number(spec.get('speed', .25), .08, .45, 'speed')
    delay = number(spec.get('delay', 0), 0, 600, 'delay')
    raw = spec.get('steps')
    if not isinstance(raw, list) or not 1 <= len(raw) <= 100:
        raise ValueError('steps must contain 1..100 actions')
    steps = []
    for step in raw:
        if not isinstance(step, dict) or step.get('t') not in ('F', 'T'):
            raise ValueError('action requires t=F or T')
        kind = step['t']
        limit = 20 if kind == 'F' else 720
        v = number(step['v'], -limit, limit, 'action')
        if abs(v) >= (.01 if kind == 'F' else .5):
            steps.append({'t': kind, 'v': v})
    if not steps:
        raise ValueError('no non-zero actions')
    pose = spec.get('start') or {}
    start = {k: number(pose.get(k, 0), -1e6, 1e6, k) for k in ('x', 'y', 'th')}
    return dict(speed=speed, delay=delay, steps=steps, start=start)


def slot(ctrl, step, speed):
    if step['t'] == 'F':
        return ctrl.straight_plan(step['v'], speed)[1] + .6
    return 1.35 + .015 * abs(step['v'])


def atomic_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + '.tmp')
    with temp.open('w', encoding='utf-8') as stream:
        json.dump(data, stream, ensure_ascii=False, allow_nan=False, indent=2)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temp, path)
