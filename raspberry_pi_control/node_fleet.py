"""PC orchestration only. All motor operations run inside the Pi service."""
from concurrent.futures import ThreadPoolExecutor
import copy
import threading
import time
import uuid

from node_client import NodeClient
from node_protocol import ACTIVE, TERMINAL, motion
from fleet_ui import Fleet as LegacyFleet, PHASE_TEXT


class NodeCar:
    def __init__(self, config, client_factory=NodeClient):
        self.config = config
        self.id = config['id']
        self.name = config.get('name', self.id)
        self.ip = config.get('ssh_host', '')
        self.mac = ''
        self.client = client_factory(config)
        self.snapshot = {}
        self.online = False
        self.conn_err = ''
        self.last_seen = None
        self.pending = False
        self.status_lock = threading.Lock()
        self.snapshot_started = 0.
        self.quit = threading.Event()
        self.poller = threading.Thread(target=self._poll, daemon=True)
        self.poller.start()

    @property
    def connected(self):
        return self.online and self.snapshot.get('configured', False)

    @property
    def running(self):
        return self.pending or self.snapshot.get('phase') in ACTIVE

    def refresh(self):
        started = time.monotonic()
        try:
            state = self.client.request('status', timeout=1)
            with self.status_lock:
                if started >= self.snapshot_started:
                    self.snapshot_started = started
                    self.snapshot = state
                    self.online = True
                    self.last_seen = time.time()
                    self.conn_err = '' if state.get('configured') else '底盘未配置；需要单独的数据串口和底盘 MAC'
                return self.snapshot
        except Exception as exc:
            with self.status_lock:
                if started >= self.snapshot_started:
                    self.snapshot_started = started
                    self.online = False
                    self.conn_err = str(exc)
            raise

    def _poll(self):
        while not self.quit.is_set():
            try:
                self.refresh()
            except Exception:
                pass
            self.quit.wait(.5 if self.online else 3)

    def set_ip(self, address):
        if self.running:
            return '车辆正在运行，不能更改节点地址'
        self.client.close()
        self.ip = address
        self.config['ssh_host'] = address
        self.client = NodeClient(self.config)
        self.online = False
        return ''

    def view(self, _now):
        state = self.snapshot
        phase = 'preparing' if self.pending else state.get('phase', 'offline')
        fw = state.get('firmware') or {}
        current = state.get('motion')
        run = None
        if current:
            step = state.get('step')
            done = phase == 'done'
            run = dict(start=current['start'], steps=current['steps'],
                       step=len(current['steps']) if done else step,
                       nstep=len(current['steps']), kind='end' if done else None,
                       est=state.get('local_est'), delay=current['delay'],
                       trail=state.get('trail', []), t_dep=state.get('depart_in'),
                       elapsed=state.get('elapsed'))
        return dict(id=self.id, name=self.name, ip=self.ip, mac='', node_id=self.config.get('node_id'),
                    connected=self.connected, node_online=self.online,
                    chassis_connected=state.get('chassis_connected', False),
                    conn_err=self.conn_err, last_seen=self.last_seen,
                    phase=phase, phase_text=PHASE_TEXT.get(phase, phase),
                    running=self.running, error=state.get('error', ''),
                    degraded=state.get('degraded', False),
                    bat=fw.get('v'), gy=fw.get('gy'), gb=fw.get('gb'), gn=fw.get('gn'),
                    lp=fw.get('lp'), fw=fw.get('fw'), cal=fw.get('cal'),
                    onboard_straight='ss' in fw, run=run,
                    log=state.get('log', []), stop_result=getattr(self, 'stop_result', None))

    def close(self):
        self.quit.set()
        self.client.close()


class NodeFleet:
    parse_choreography = LegacyFleet.parse_choreography
    _motion = staticmethod(LegacyFleet._motion)

    def __init__(self, cfg, client_factory=NodeClient):
        self.lock = threading.RLock()
        self.cars = {c['id']: NodeCar(c, client_factory) for c in cfg['cars']}
        if len(self.cars) != len(cfg['cars']):
            raise ValueError('duplicate car IDs')
        node_ids = [c['node_id'] for c in cfg['cars']]
        if len(set(node_ids)) != len(node_ids):
            raise ValueError('each car needs a distinct node_id')
        self.choreography = None
        self.group = None
        self.quit = threading.Event()
        self.pool = ThreadPoolExecutor(max_workers=max(4, len(self.cars)))
        self.coordinator = None

    @property
    def choreography_running(self):
        return bool(self.group and self.group['phase'] in ('preparing', 'running', 'partial'))

    def state(self):
        group = copy.deepcopy(self.group)
        choreo = copy.deepcopy(self.choreography)
        if choreo and group:
            choreo['phase'] = 'running' if group['phase'] in ('preparing', 'partial') else group['phase']
            choreo['error'] = group.get('error', '')
            stages = [c.snapshot.get('stage') for c in self.cars.values()
                      if c.id in group['ids'] and c.snapshot.get('run_id') == group['run_id']]
            choreo['stage'] = 1 + max((s for s in stages if s is not None), default=0)
        return dict(running=self.choreography_running or any(c.running for c in self.cars.values()),
                    go_in=None, cars=[c.view(time.monotonic()) for c in self.cars.values()],
                    choreography=choreo, group=group)

    def start(self, specs, sync=True, choreography=False):
        try:
            if not isinstance(specs, list) or not specs:
                raise ValueError('没有选中任何车')
            ids = [s['id'] for s in specs]
            if len(set(ids)) != len(ids):
                raise ValueError('重复车辆')
            jobs = {s['id']: [motion(s)] for s in specs}
            return self._launch(jobs, sync)
        except (ValueError, TypeError, KeyError) as exc:
            return str(exc)

    def start_choreography(self, data):
        plan = self.parse_choreography(data)
        jobs = {cid: [None] * len(plan['stages']) for cid in plan['starts']}
        for index, stage in enumerate(plan['stages']):
            for spec in stage['cars']:
                jobs[spec['id']][index] = motion(spec)
        # Stationary-only participants do not need a motor task or calibration.
        jobs = {cid: stages for cid, stages in jobs.items() if any(stages)}
        error = self._launch(jobs, True)
        if not error:
            self.choreography = dict(id=self.group['run_id'], name=plan['name'],
                                     total=len(plan['stages']), stage=0, phase='running', error='')
        return error

    def _launch(self, jobs, sync):
        with self.lock:
            if self.choreography_running or any(c.running for c in self.cars.values()):
                return '已有任务或执行状态尚未确认，先查询或明确停止'
            for cid in jobs:
                if cid not in self.cars:
                    return f'未知车辆 {cid}'
                if not self.cars[cid].connected:
                    return f'{cid} 节点离线或底盘未配置'
            if not jobs:
                return '没有动作'
            self.choreography = None
            self.group = dict(run_id=str(uuid.uuid4()), ids=list(jobs), phase='preparing',
                              degraded=False, error='', stop_results={})
            group = self.group
            group['cancel'] = False
            for cid in jobs:
                self.cars[cid].pending = True
            self.coordinator = threading.Thread(target=self._coordinate, args=(group, jobs, sync), daemon=True)
            self.coordinator.start()
        return ''

    def _parallel(self, ids, func):
        futures = {cid: self.pool.submit(func, self.cars[cid]) for cid in ids}
        results, errors = {}, {}
        for cid, future in futures.items():
            try:
                results[cid] = future.result()
            except Exception as exc:
                errors[cid] = str(exc)
        return results, errors

    def _coordinate(self, group, jobs, sync):
        ids, run_id = list(jobs), group['run_id']
        committed = False
        try:
            _, errors = self._parallel(ids, lambda c: c.client.request('prepare',
                dict(run_id=run_id, stages=jobs[c.id], sync=sync), timeout=3))
            if errors:
                raise RuntimeError(f'准备请求未全部确认: {errors}')
            deadline = time.monotonic() + 30
            prepared = {}
            while len(prepared) != len(ids):
                if group['cancel'] or self.quit.is_set():
                    raise RuntimeError('启动已取消')
                states, errors = self._parallel(ids, lambda c: c.refresh())
                if errors:
                    raise RuntimeError(f'准备阶段节点失联: {errors}')
                for cid, state in states.items():
                    if state.get('run_id') != run_id:
                        raise RuntimeError('任务身份不一致')
                    if state['phase'] in TERMINAL:
                        raise RuntimeError(f'{cid}: {state.get("error") or state["phase"]}')
                    if state['phase'] == 'ready':
                        prepared[cid] = state
                if time.monotonic() > deadline:
                    raise TimeoutError('节点准备超时')
                time.sleep(.05)
            bases = [0.]
            for stage in range(len(next(iter(jobs.values()))) - 1):
                budget = max((a['end'] for state in prepared.values()
                              for a in state['actions'] if a['stage'] == stage), default=0.)
                bases.append(bases[-1] + budget)
            offsets, errors = self._parallel(ids, lambda c: c.client.clock_sample())
            if errors:
                raise RuntimeError(str(errors))
            if group['cancel'] or self.quit.is_set():
                raise RuntimeError('启动已取消')
            start = time.monotonic() + 2
            # These virtual-clock limits include not-yet-started actions, which
            # prevents a fast car escaping before the PC observes a slow car.
            initial = {cid: min(a['end'] + bases[a['stage']] for a in state['actions'])
                       for cid, state in prepared.items()}
            committed = True  # An unacknowledged commit may already have executed.
            _, errors = self._parallel(ids, lambda c: c.client.request('commit',
                dict(run_id=run_id, start_at=start+offsets[c.id], stage_bases=bases), timeout=1))
            if errors:
                group['phase'] = 'partial'
                raise RuntimeError(f'部分启动确认缺失，执行结果未知: {errors}')
            group['phase'] = 'running'
            for cid in ids:
                self.cars[cid].pending = False
            last = time.monotonic()
            vt = last - start
            seq = 0
            boot_ids = {cid: prepared[cid]['boot_id'] for cid in ids}
            while not self.quit.is_set() and not group['cancel']:
                states, errors = self._parallel(ids, lambda c: c.refresh())
                valid = {cid: s for cid, s in states.items()
                         if s.get('run_id') == run_id and s.get('boot_id') == boot_ids[cid]}
                if errors or len(valid) != len(ids) or any(s['degraded'] for s in valid.values()):
                    group['degraded'] = True
                failed = {cid: s for cid, s in valid.items() if s['phase'] in ('error', 'stopped', 'interrupted')}
                if failed:
                    raise RuntimeError(f'节点任务中断: {[(cid,s["error"] or s["phase"]) for cid,s in failed.items()]}')
                if len(valid) == len(ids) and all(s['phase'] == 'done' for s in valid.values()):
                    group['phase'] = 'done'
                    return
                # Freeze at the earliest unfinished action's planned end.
                limits = [(s.get('due') if s.get('due') is not None else 1e7)
                          for s in valid.values()]
                if len(valid) != len(ids):
                    limits.extend(initial[cid] for cid in ids if cid not in valid)
                limit = min(limits, default=1e7) if sync else 1e7
                now = time.monotonic()
                vt = min(limit, vt + now-last)
                last = now
                seq += 1
                active = [cid for cid, s in valid.items() if s['phase'] in ACTIVE]
                _, heartbeat_errors = self._parallel(active, lambda c: c.client.request('heartbeat',
                    dict(run_id=run_id, seq=seq, vt=vt, limit=limit,
                         degraded=group['degraded']), timeout=.5))
                if heartbeat_errors:
                    group['degraded'] = True
                self.quit.wait(.05)
        except Exception as exc:
            group['error'] = str(exc)
            group['phase'] = 'partial' if committed and '确认' in str(exc) else 'error'
            results, errors = self._parallel(ids, lambda c: c.client.request('stop', dict(run_id=run_id), timeout=2))
            group['stop_results'] = {cid: results.get(cid, {}).get('stop', 'unknown') for cid in ids}
            if errors:
                group['error'] += f'; 停车结果未知: {errors}'
        finally:
            for cid in ids:
                self.cars[cid].pending = False
            if group['cancel']:
                group['phase'] = 'stopped'

    def stop(self, ids=None):
        group = self.group
        if group and group['phase'] in ('preparing', 'running', 'partial'):
            # A group stop cancels all its members, preserving choreography semantics.
            if ids is None or set(ids).intersection(group['ids']):
                group['cancel'] = True
                ids = group['ids']
        selected = [cid for cid in self.cars if ids is None or cid in ids]
        results, errors = self._parallel(selected, lambda c: c.client.request('stop',
            dict(run_id=c.snapshot.get('run_id')), timeout=2))
        for cid in selected:
            self.cars[cid].stop_result = results.get(cid, {}).get('stop', 'unknown')
        if group and group.get('cancel') and (not self.coordinator or not self.coordinator.is_alive()):
            group['phase'] = 'stopped'
        unknown = [cid for cid in selected if self.cars[cid].stop_result != 'confirmed']
        return ('停车未确认: ' + ', '.join(unknown)) if unknown else ''

    def calib_speed(self, cid, measured):
        try:
            car = self.cars[cid]
            return car.client.request('calibrate', dict(run_id=car.snapshot.get('run_id'), measured=measured))
        except Exception as exc:
            return {'error': str(exc)}

    def scan(self):
        states, errors = self._parallel(list(self.cars), lambda c: c.refresh())
        return {'error': '', 'found': [dict(car=cid, ip=self.cars[cid].ip) for cid in states],
                'changes': [], 'errors': errors}

    def auto_scan_later(self, **_):
        pass  # NodeCar polls registered endpoints; no ESP32 subnet scanning.

    def close(self):
        # Explicitly do not stop already committed node jobs.
        self.quit.set()
        if self.coordinator and self.coordinator is not threading.current_thread():
            self.coordinator.join(timeout=15)
        for car in self.cars.values():
            car.close()
        if not self.coordinator or not self.coordinator.is_alive():
            self.pool.shutdown(wait=False, cancel_futures=True)
