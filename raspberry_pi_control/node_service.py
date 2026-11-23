#!/usr/bin/env python3
"""Persistent per-car executor. No serial port is opened without explicit config."""
import argparse
from collections import OrderedDict, deque
import copy
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import signal
import threading
import time
import uuid

import wave_rover_traj as w
from node_protocol import ACTIVE, VERSION, atomic_json, motion, number, slot


class Hardware:
    def __init__(self, config, log):
        port, expected = config.get('serial_port'), config.get('expected_mac')
        if not port or not expected:
            raise ValueError('底盘未配置：需要 serial_port 和 expected_mac；TF 串口不能用作底盘')
        tf = config.get('tf_port')
        if tf and os.path.realpath(port) == os.path.realpath(tf):
            raise ValueError('TF 与底盘不能共用串口')
        # Enforce Linux TIOCEXCL without replacing the global pyserial factory.
        import serial
        original = w.serial
        class SerialModule:
            def __getattr__(self, key):
                return getattr(original, key)
            def Serial(self, *args, **kwargs):
                return serial.Serial(*args, **kwargs, exclusive=True)
        w.serial = SerialModule()
        self.rover = None
        try:
            self.rover = w.WaveRover(port)
            state = self.rover.fw_status()
            if not state or str(state.get('mac', '')).lower() != expected.lower():
                raise ValueError('底盘身份不符或无 V2 状态回复')
            self.ctrl = w.TrajController(self.rover, w.load_calib(config['calibration']), log_fn=log)
            if not self.ctrl.fw or not self.ctrl.fw_straight:
                raise ValueError('requires V2.3+ onboard straight firmware')
        except Exception:
            if self.rover:
                self.rover.close()
            raise
        finally:
            w.serial = original
        self.motor_lock = threading.RLock()
        self.blocked = True
        self.raw_send = self.rover.send
        self.rover.send = self._send

    def _send(self, command):
        moving = command.get('T') in (160, 165) or (
            command.get('T') in (1, 11) and (command.get('L', 0) or command.get('R', 0)))
        with self.motor_lock:
            if self.blocked and moving:
                raise w.Aborted()
            self.raw_send(command)

    def arm(self):
        with self.motor_lock:
            self.blocked = False
            self.ctrl.abort.clear()

    def state(self):
        state = dict(self.rover.frames.get(w.FEEDBACK_TURN) or self.ctrl.fw_last or {})
        base = self.rover.feedback
        if 'v' in base:
            state['v'] = base['v']
        latest = max(self.rover.frame_time.values(), default=0)
        state['_age'] = time.monotonic() - latest
        return state

    def stop(self):
        with self.motor_lock:
            self.blocked = True
            self.ctrl.abort.set()
            self.rover.send({'T': 161})
            self.rover.stop()
        deadline = time.monotonic() + .8
        while time.monotonic() < deadline:
            state = self.rover.fw_status()
            if state and state.get('ts') != 1 and state.get('ss') != 1:
                return True
            time.sleep(.02)
        return False

    def close(self):
        self.rover.close()


class Node:
    def __init__(self, config, backend_factory=Hardware):
        self.config = config
        self.node_id = config['node_id']
        self.boot_id = str(uuid.uuid4())
        self.lock = threading.RLock()
        self.backend_factory = backend_factory
        self.backend = None
        self.abort = threading.Event()
        self.thread = None
        self.log = deque(maxlen=300)
        self.requests = OrderedDict()
        self.run_id = None
        self.phase = 'idle'
        self.error = ''
        self.task = None
        self.actions = []
        self.cursor = 0
        self.executing = False
        self.degraded = False
        self.start_at = None
        self.heartbeat_at = 0
        self.heartbeat_seq = -1
        self.clock_vt = 0
        self.clock_limit = 0
        self.last_straight = None
        self.trail = []
        self.x = self.y = self.heading = 0.
        self.stage_origin = (0., 0., 0.)
        self.journal = Path(config.get('state_file', 'node_state.json'))
        self.seen_runs = []
        if self.journal.exists():
            previous = json.loads(self.journal.read_text(encoding='utf-8'))
            self.run_id = previous.get('run_id')
            self.seen_runs = previous.get('seen_runs', [])
            self.task = previous.get('task')
            self.actions = previous.get('actions') or []
            self.cursor = previous.get('cursor', 0)
            self.degraded = previous.get('degraded', False)
            self.phase = 'interrupted' if previous.get('phase') in ACTIVE else previous.get('phase', 'idle')
            self.error = '服务重启，未自动恢复运动' if self.phase == 'interrupted' else ''
            self._save()

    def add_log(self, message):
        with self.lock:
            self.log.append(f'[{time.strftime("%H:%M:%S")}] {message}')

    def _save(self):
        atomic_json(self.journal, {'run_id': self.run_id, 'phase': self.phase,
                                  'seen_runs': self.seen_runs[-10000:],
                                  'task': self.task, 'actions': self.actions,
                                  'cursor': self.cursor, 'degraded': self.degraded,
                                  'error': self.error, 'log': list(self.log)[-40:]})

    def _phase(self, phase):
        with self.lock:
            self.phase = phase
            self._save()

    def status(self):
        with self.lock:
            st = self.backend.state() if self.backend else {}
            action = self.actions[self.cursor] if self.cursor < len(self.actions) else None
            display = action or (self.actions[-1] if self.actions else None)
            x, y = self.x, self.y
            if self.executing and action and action['step']['t'] == 'F' and st.get('ss') == 1:
                progress = max(0., min(1., float(st.get('sp', 0))))
                dist = action['step']['v'] * progress
                x += dist * math.cos(math.radians(self.heading))
                y += dist * math.sin(math.radians(self.heading))
            ox, oy, oh = self.stage_origin
            angle = math.radians(oh)
            local = [(x-ox)*math.cos(angle)+(y-oy)*math.sin(angle),
                     -(x-ox)*math.sin(angle)+(y-oy)*math.cos(angle),
                     float(st.get('gy', self.heading))-oh]
            preparing = self.phase == 'preparing'
            return dict(version=VERSION, node_id=self.node_id, boot_id=self.boot_id,
                        monotonic=time.monotonic(), run_id=self.run_id, phase=self.phase,
                        error=self.error, configured=bool(self.config.get('serial_port')),
                        chassis_connected=bool(self.backend and st and st.get('_age', 0) < 2), firmware=st,
                        degraded=self.degraded, actions=copy.deepcopy(self.actions) if self.phase == 'ready' else None,
                        cursor=self.cursor, executing=self.executing,
                        due=action['end'] if action else None,
                        stage=display['stage'] if display else None,
                        stages=len(self.task['stages']) if self.task else 0,
                        step=display['index'] if display else None,
                        motion=self.task['stages'][display['stage']] if display else None,
                        depart_in=max(0., self.start_at-time.monotonic()) if self.start_at else None,
                        elapsed=max(0., time.monotonic()-self.start_at) if self.start_at else None,
                        local_est=local, trail=self.trail[-1000:],
                        log=list(self.log)[-40:], preparing=preparing,
                        last_straight=self.last_straight,
                        calibration=hashlib.sha256(json.dumps(self.backend.ctrl.c, sort_keys=True).encode()).hexdigest() if self.backend else None)

    def dispatch(self, method, data):
        if data.get('version') != VERSION:
            raise ValueError('unsupported protocol version')
        if method == 'status':
            return self.status()
        rid = data.get('request_id')
        if not isinstance(rid, str) or not 1 <= len(rid) <= 100:
            raise ValueError('request_id required')
        digest = json.dumps([method, data], sort_keys=True, allow_nan=False)
        # Serialize mutations. Slow work is done in the single execution thread.
        with self.lock:
            if rid in self.requests:
                old_digest, result = self.requests[rid]
                if old_digest != digest:
                    raise ValueError('request_id reused with different payload')
                return copy.deepcopy(result)
            if method == 'prepare':
                result = self.prepare(data)
            else:
                if data.get('run_id') != self.run_id or not self.run_id:
                    raise ValueError('stale or unknown run_id')
                if method == 'commit':
                    result = self.commit(data)
                elif method == 'heartbeat':
                    result = self.heartbeat(data)
                elif method == 'stop':
                    result = self.stop()
                elif method == 'calibrate':
                    result = self.calibrate(data)
                else:
                    raise ValueError('unknown method')
            result = dict(version=VERSION, node_id=self.node_id, **result)
            self.requests[rid] = (digest, copy.deepcopy(result))
            while len(self.requests) > 4096:
                self.requests.popitem(last=False)
            return result

    def prepare(self, data):
        run_id = data.get('run_id')
        if not isinstance(run_id, str) or not 1 <= len(run_id) <= 100:
            raise ValueError('run_id required')
        if run_id in self.seen_runs:
            raise ValueError('run_id already used; query status instead of replaying')
        if self.thread and self.thread.is_alive() or self.phase in ACTIVE:
            raise ValueError('node busy')
        stages = data.get('stages')
        if not isinstance(stages, list) or not 1 <= len(stages) <= 100:
            raise ValueError('stages must contain 1..100 stages')
        clean = [None if stage is None else motion(stage) for stage in stages]
        if not any(clean):
            raise ValueError('node has no actions')
        self.task = {'stages': clean, 'sync': bool(data.get('sync', True))}
        self.run_id = run_id
        self.seen_runs.append(run_id)
        self.abort.clear()
        self.actions, self.trail = [], []
        self.cursor = 0
        self.executing = self.degraded = False
        self.error = ''
        self.start_at = None
        self.last_straight = None
        self.x = self.y = self.heading = 0.
        self.stage_origin = (0., 0., 0.)
        self._phase('preparing')
        self.thread = threading.Thread(target=self._prepare_worker, daemon=True)
        self.thread.start()
        return {'accepted': True, 'run_id': run_id}

    def _prepare_worker(self):
        try:
            if self.backend is None:
                backend = self.backend_factory(self.config, self.add_log)
                with self.lock:
                    self.backend = backend
            ctrl = self.backend.ctrl
            if hasattr(self.backend, 'arm'):
                self.backend.arm()
            else:
                ctrl.abort.clear()
            if self.abort.is_set():
                raise w.Aborted()
            ctrl.fw_calibrate()
            actions = []
            for stage_index, stage in enumerate(self.task['stages']):
                if stage is None:
                    continue
                at = stage['delay']
                for index, step in enumerate(stage['steps']):
                    end = at + slot(ctrl, step, stage['speed'])
                    actions.append(dict(stage=stage_index, index=index, at=at, end=end,
                                        step=step, speed=stage['speed']))
                    at = end
            with self.lock:
                if self.abort.is_set():
                    raise w.Aborted()
                self.actions = actions
                self._phase('ready')
            self.add_log('完整任务已预装，等待启动')
        except w.Aborted:
            self._phase('stopped')
        except Exception as exc:
            self._failure(exc)

    def commit(self, data):
        # A new request ID must never start an already committed task again.
        if self.start_at is not None:
            raise ValueError('task already committed')
        if self.phase != 'ready' or self.abort.is_set():
            raise ValueError('task not ready')
        start_at = number(data.get('start_at'), time.monotonic() + .15,
                          time.monotonic() + 60, 'start_at')
        bases = data.get('stage_bases')
        if not isinstance(bases, list) or len(bases) != len(self.task['stages']):
            raise ValueError('stage_bases length mismatch')
        bases = [number(b, 0, 1e7, 'stage base') for b in bases]
        if bases[0] != 0 or any(b < a for a, b in zip(bases, bases[1:])):
            raise ValueError('stage bases must start at zero and be monotonic')
        for a, b in zip(self.actions, self.actions[1:]):
            if a['stage'] != b['stage'] and a['end'] + bases[a['stage']] > b['at'] + bases[b['stage']]:
                raise ValueError('overlapping stage schedule')
        for action in self.actions:
            base = bases[action['stage']]
            action['at'] += base
            action['end'] += base
        self.start_at = start_at
        self.heartbeat_at = time.monotonic()
        self.heartbeat_seq = -1
        self.clock_vt = self.heartbeat_at - start_at
        self.clock_limit = min((a['end'] for a in self.actions), default=1e7)
        self._phase('waiting')
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        return {'accepted': True, 'run_id': self.run_id}

    def heartbeat(self, data):
        if self.start_at is None or self.phase not in ACTIVE:
            return {'accepted': False}
        seq = data.get('seq')
        if type(seq) is not int or seq <= self.heartbeat_seq:
            return {'accepted': False}
        vt = number(data['vt'], -60, 1e7, 'vt')
        limit = number(data['limit'], 0, 1e7, 'limit')
        if data.get('degraded'):
            if not self.degraded:
                now = time.monotonic()
                self.clock_vt = min(self.clock_limit, self.clock_vt + now-self.heartbeat_at)
                self.heartbeat_at = now
            self.degraded = True
        if not self.degraded:
            self.clock_vt, self.clock_limit = vt, limit
            self.heartbeat_at = time.monotonic()
        self.heartbeat_seq = seq
        return {'accepted': True}

    def _virtual_now(self):
        now = time.monotonic()
        if not self.degraded and now - self.heartbeat_at > 3:
            # Preserve the last confirmed virtual-clock offset, once per run.
            self.clock_vt = min(self.clock_limit, self.clock_vt + 3)
            self.heartbeat_at += 3
            self.degraded = True
            self.add_log('协调通信失联：同步降级，按本地完整任务继续')
        if self.degraded:
            return self.clock_vt + now - self.heartbeat_at
        return min(self.clock_limit, self.clock_vt + now - self.heartbeat_at)

    def _wait(self, due, stage_start=False):
        while True:
            if self.abort.is_set():
                raise w.Aborted()
            with self.lock:
                vt = self._virtual_now()
                permitted = self.degraded or not self.task['sync'] or due < self.clock_limit - 1e-6
                scheduled = self.task['sync'] or stage_start
                if time.monotonic() >= self.start_at and (not scheduled or vt >= due - 1e-6) and permitted:
                    return
            self.abort.wait(.01)

    def _run(self):
        ctrl = self.backend.ctrl
        finished = False
        try:
            for cursor, action in enumerate(self.actions):
                with self.lock:
                    if action['index'] == 0:
                        self.stage_origin = (self.x, self.y, self.heading)
                        self.trail = [[0., 0.]]
                    self.cursor = cursor
                    self._phase('sync' if cursor else 'waiting')
                self._wait(action['at'], action['index'] == 0)
                with self.lock:
                    self.executing = True
                    self._phase('straight' if action['step']['t'] == 'F' else 'turn')
                # Every local movement is executed by the shared executor.
                from node_executor import execute_action
                self.heading = execute_action(ctrl, action['step'], action['speed'], self.heading)
                with self.lock:
                    if action['step']['t'] == 'F':
                        distance = action['step']['v']
                        self.x += distance * math.cos(math.radians(self.heading))
                        self.y += distance * math.sin(math.radians(self.heading))
                        stage = self.task['stages'][action['stage']]
                        if len(stage['steps']) == 1:
                            pwm, speed = ctrl.straight_pwm(action['speed'])
                            self.last_straight = dict(dist=abs(distance), pwm=pwm, v=speed, speed=action['speed'])
                    ox, oy, oh = self.stage_origin
                    angle = math.radians(oh)
                    self.trail.append([(self.x-ox)*math.cos(angle)+(self.y-oy)*math.sin(angle),
                                       -(self.x-ox)*math.sin(angle)+(self.y-oy)*math.cos(angle)])
                    self.executing = False
                    self.cursor = cursor + 1
                self.add_log(f'阶段 {action["stage"]+1}，步骤 {action["index"]+1} 完成')
            finished = True
        except w.Aborted:
            self._phase('stopped')
        except Exception as exc:
            self._failure(exc)
        finally:
            with self.lock:
                self.executing = False
            try:
                if not self.backend.stop():
                    raise RuntimeError('底盘未确认停车')
                if finished:
                    self._phase('done')
            except Exception as exc:
                with self.lock:
                    self.error = f'停车未确认: {exc}'
                    self._phase('error')

    def _failure(self, exc):
        self.error = str(exc)
        if self.backend:
            try:
                self.backend.stop()
            except Exception:
                pass
        self._phase('error')
        self.add_log(self.error)

    def stop(self):
        self.abort.set()
        confirmed = False
        if self.backend:
            confirmed = self.backend.stop()
        if not self.thread or not self.thread.is_alive():
            self._phase('stopped')
        return {'stop': 'confirmed' if confirmed else 'unknown', 'accepted': True}

    def calibrate(self, data):
        if self.phase in ACTIVE or self.thread and self.thread.is_alive():
            raise ValueError('cannot change calibration during active task')
        if not self.last_straight or self.backend is None:
            raise ValueError('no single-straight measurement available')
        ls = self.last_straight
        measured = number(data.get('measured'), .2*ls['dist'], 3*ls['dist'], 'measured')
        from fleet_ui import fit_speed_model
        ctrl = self.backend.ctrl
        v = ls['v'] * measured / ls['dist']
        points = [p for p in ctrl.c.get('sv_pts', []) if abs(p[0]-ls['pwm']) > 1e-4]
        points = (points + [[ls['pwm'], v]])[-4:]
        k, dz = fit_speed_model(points, float(ctrl.c.get('sv_dz') or .15))
        if k is None:
            raise ValueError('PWM too close to dead zone')
        calib = dict(ctrl.c, sv_k=k, sv_dz=dz, sv_pts=points)
        atomic_json(self.config['calibration'], calib)
        ctrl.c = calib
        self.last_straight = None
        return {'error': '', 'v_meas': v, 'sv_k': k, 'old_pwm': ls['pwm'],
                'new_pwm': ctrl.straight_pwm(ls['speed'])[0], 'file': self.config['calibration']}

    def close(self):
        with self.lock:
            self.abort.set()
            if self.backend:
                try:
                    self.backend.stop()
                except Exception:
                    pass
        if self.thread:
            self.thread.join(timeout=5)
        if self.backend:
            self.backend.close()


def make_server(node, port=8765):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def do_POST(self):
            try:
                if self.headers.get_content_type() != 'application/json':
                    raise ValueError('Content-Type must be application/json')
                size = int(self.headers.get('Content-Length', 0))
                if not 0 < size <= 2_000_000:
                    raise ValueError('invalid body size')
                self.connection.settimeout(5)
                data = json.loads(self.rfile.read(size))
                if not isinstance(data, dict):
                    raise ValueError('request must be object')
                if not self.path.startswith('/v1/'):
                    raise ValueError('unknown path')
                result = node.dispatch(self.path[4:], data)
                status = 200
            except (ValueError, KeyError, TypeError, OverflowError) as exc:
                result, status = {'error': str(exc)}, 400
            except Exception as exc:
                result, status = {'error': str(exc)}, 500
            body = json.dumps(result, ensure_ascii=False, allow_nan=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass
    server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    server.daemon_threads = True
    return server


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default='node_config.json')
    args = parser.parse_args()
    config = json.loads(Path(args.config).read_text(encoding='utf-8'))
    # Process lock also prevents a second service on a different TCP port.
    import fcntl
    state_path = Path(config.get('state_file', 'node_state.json')).resolve()
    state_path.parent.mkdir(parents=True, exist_ok=True)
    lockfile = open(str(state_path) + '.lock', 'a')
    fcntl.flock(lockfile, fcntl.LOCK_EX | fcntl.LOCK_NB)
    node = Node(config)
    server = make_server(node, int(config.get('port', 8765)))
    def shutdown(*_):
        threading.Thread(target=server.shutdown, daemon=True).start()
    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    try:
        server.serve_forever()
    finally:
        server.server_close()
        node.close()
        lockfile.close()


if __name__ == '__main__':
    main()
