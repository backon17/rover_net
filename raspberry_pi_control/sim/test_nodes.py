"""Distributed-node integration tests; no hardware, Wi-Fi or firmware compiler."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
import uuid
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import wave_rover_traj as w
from node_client import NodeClient
from node_fleet import NodeFleet
from node_protocol import ACTIVE, motion
from node_service import Node, make_server


class FakeCtrl:
    fw = fw_straight = True
    def __init__(self, backend):
        self.backend = backend
        self.abort = threading.Event()
        self.c = dict(w.DEFAULT_CALIB)
        self.angle = 0

    def _check_abort(self):
        if self.abort.is_set():
            raise w.Aborted()

    def fw_calibrate(self):
        self._check_abort()
        self.angle = 0

    def fw_heading(self):
        return (self.angle, 0)

    def straight_plan(self, distance, speed):
        return (.3, abs(distance)/speed)

    def straight_pwm(self, speed):
        return (.3, speed)

    def drive_straight(self, value, speed_ms, heading_deg):
        self._move('F', value)

    def fw_turn(self, value):
        self._move('T', value)
        self.angle += value

    def _move(self, kind, value):
        self._check_abort()
        self.backend.events.append((kind, value, time.monotonic()))
        deadline = time.monotonic() + self.backend.duration
        while time.monotonic() < deadline:
            self._check_abort()
            time.sleep(.003)


class FakeBackend:
    def __init__(self, config, log):
        self.events = []
        self.duration = config.get('duration', .02)
        self.ctrl = FakeCtrl(self)
        self.closed = False

    def state(self):
        return {'T': 1010, 'fw': 'fake-v2.4.3', 'ss': 0, 'ts': 0, 'gy': self.ctrl.angle, 'v': 12}

    def stop(self):
        self.ctrl.abort.set()
        return True

    def close(self):
        self.closed = True


def wait_until(predicate, timeout=8):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(.01)
    raise AssertionError('timed out waiting for condition')


PLAN = dict(speed=.18, steps=[{'t': 'F', 'v': .1}, {'t': 'T', 'v': 90}, {'t': 'F', 'v': .1}])


class NodeTests(unittest.TestCase):
    def setUp(self):
        artifacts = Path(__file__).resolve().parents[1] / '.test-artifacts'
        artifacts.mkdir(exist_ok=True)
        self.temp = tempfile.TemporaryDirectory(dir=artifacts)
        self.nodes, self.servers, self.fleets = [], [], []
        self.slot_patch = patch('node_service.slot', lambda *_: .07)
        self.slot_patch.start()

    def tearDown(self):
        for fleet in self.fleets:
            fleet.close()
        for node in self.nodes:
            node.close()
        for server in self.servers:
            server.shutdown()
            server.server_close()
        self.slot_patch.stop()
        self.temp.cleanup()

    def node(self, name='r1', duration=.02):
        config = dict(node_id=name, serial_port='fake-chassis', duration=duration,
                      calibration=str(Path(self.temp.name)/f'{name}_calib.json'),
                      state_file=str(Path(self.temp.name)/f'{name}_state.json'))
        node = Node(config, FakeBackend)
        self.nodes.append(node)
        server = make_server(node, 0)
        self.servers.append(server)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        return node, NodeClient(dict(url=f'http://127.0.0.1:{server.server_port}', node_id=name))

    def prepare(self, node, client, stages=None, sync=True):
        rid = str(uuid.uuid4())
        client.request('prepare', dict(run_id=rid, stages=stages or [PLAN], sync=sync))
        wait_until(lambda: node.phase == 'ready' and not node.thread.is_alive())
        return rid

    def commit(self, client, rid, bases=None):
        start = time.monotonic()+.2
        client.request('commit', dict(run_id=rid, start_at=start, stage_bases=bases or [0]))
        return start

    def test_service_is_passive_until_prepare(self):
        node = Node(dict(node_id='passive', state_file=str(Path(self.temp.name)/'passive.json')))
        self.nodes.append(node)
        self.assertFalse(node.status()['chassis_connected'])
        self.assertIsNone(node.backend)
        with self.assertRaises(ValueError):
            motion(dict(PLAN, speed=float('nan')))

    def test_duplicate_commit_and_request_payload_mismatch(self):
        node, client = self.node()
        rid = self.prepare(node, client, sync=False)
        request_id = 'same-commit'
        data = dict(run_id=rid, start_at=time.monotonic()+.2, stage_bases=[0])
        first = client.request('commit', data, request_id=request_id)
        self.assertEqual(first, client.request('commit', data, request_id=request_id))
        with self.assertRaises(ValueError):
            client.request('commit', data)
        with self.assertRaises(ValueError):
            client.request('stop', dict(run_id=rid), request_id=request_id)
        wait_until(lambda: node.phase == 'done')
        self.assertEqual(len(node.backend.events), 3)
        with self.assertRaises(ValueError):
            client.request('prepare', dict(run_id=rid, stages=[PLAN]))

    def test_disconnect_runs_all_stages_without_new_messages(self):
        node, client = self.node()
        rid = self.prepare(node, client, [PLAN, None, PLAN])
        self.commit(client, rid, [0, .21, .3])
        client.close()
        wait_until(lambda: node.phase == 'done', 6)
        self.assertTrue(node.degraded)
        self.assertEqual(len(node.backend.events), 6)

    def test_stop_priority_and_stale_stop(self):
        node, client = self.node(duration=.5)
        rid = self.prepare(node, client, sync=False)
        self.commit(client, rid)
        wait_until(lambda: node.executing)
        with self.assertRaises(ValueError):
            client.request('stop', dict(run_id='old-task'))
        self.assertFalse(node.abort.is_set())
        result = client.request('stop', dict(run_id=rid))
        self.assertEqual(result['stop'], 'confirmed')
        wait_until(lambda: node.phase == 'stopped')
        self.assertEqual(len(node.backend.events), 1)

    def test_restart_marks_interrupted_and_refuses_replay(self):
        node, client = self.node()
        rid = self.prepare(node, client)
        restarted = Node(node.config, FakeBackend)
        self.nodes.append(restarted)
        self.assertEqual(restarted.phase, 'interrupted')
        self.assertIsNone(restarted.backend)
        with self.assertRaises(ValueError):
            restarted.dispatch('prepare', dict(version=1, request_id='new', run_id=rid, stages=[PLAN]))

    def test_calibration_idle_only_persists_on_node(self):
        node, client = self.node()
        single = dict(speed=.18, steps=[dict(t='F', v=.1)])
        rid = self.prepare(node, client, [single], sync=False)
        self.commit(client, rid)
        with self.assertRaises(ValueError):
            client.request('calibrate', dict(run_id=rid, measured=.1))
        wait_until(lambda: node.phase == 'done' and not node.thread.is_alive())
        result = client.request('calibrate', dict(run_id=rid, measured=.11))
        self.assertEqual(result['error'], '')
        self.assertTrue(Path(node.config['calibration']).exists())

    def test_clock_update_reordering_and_degrade_latches(self):
        node, client = self.node()
        rid = self.prepare(node, client)
        self.commit(client, rid)
        client.request('heartbeat', dict(run_id=rid, seq=2, vt=0, limit=.07, degraded=True))
        result = client.request('heartbeat', dict(run_id=rid, seq=1, vt=100, limit=100))
        self.assertFalse(result['accepted'])
        client.request('heartbeat', dict(run_id=rid, seq=3, vt=0, limit=100))
        self.assertTrue(node.degraded)
        wait_until(lambda: node.phase == 'done')

    def fleet(self, count=3):
        configs = []
        for i in range(count):
            node, client = self.node(f'r{i}', duration=.18 if i == 1 else .02)
            configs.append(dict(id=f'car{i}', **client.config))
        fleet = NodeFleet({'cars': configs})
        self.fleets.append(fleet)
        wait_until(lambda: all(c.connected for c in fleet.cars.values()))
        return fleet

    def test_multinode_sync_slow_car_and_choreography(self):
        fleet = self.fleet()
        specs = [dict(id=cid, **PLAN) for cid in fleet.cars]
        self.assertEqual(fleet.start(specs), '')
        wait_until(lambda: fleet.group['phase'] == 'done', 8)
        self.assertFalse(fleet.group['degraded'])
        events = [n.backend.events for n in self.nodes]
        for index in range(3):
            starts = [e[index][2] for e in events]
            self.assertLess(max(starts)-min(starts), .25)
        # Fast car waits for the slow car before its next step.
        self.assertGreater(events[0][1][2]-events[0][0][2], .15)
        plan = dict(version=1, name='stages', starts={c: dict(x=0,y=0,th=0) for c in fleet.cars},
                    stages=[dict(cars=[dict(id='car0', **PLAN)]), dict(cars=[dict(id='car1', **PLAN)])])
        wait_until(lambda: all(not c.running for c in fleet.cars.values()))
        self.assertEqual(fleet.start_choreography(plan), '')
        wait_until(lambda: fleet.group['phase'] == 'done', 8)
        self.assertEqual(len(self.nodes[0].backend.events), 6)
        self.assertEqual(len(self.nodes[1].backend.events), 6)

    def test_pc_close_keeps_committed_task_running(self):
        fleet = self.fleet(1)
        self.assertEqual(fleet.start([dict(id='car0', **PLAN)]), '')
        wait_until(lambda: fleet.group['phase'] == 'running')
        fleet.close()
        wait_until(lambda: self.nodes[0].phase == 'done', 7)
        self.assertEqual(len(self.nodes[0].backend.events), 3)

    def test_eight_nodes_delays_and_live_http_api(self):
        fleet = self.fleet(8)
        import fleet_ui
        original = fleet_ui.FLEET
        fleet_ui.FLEET = fleet
        server = fleet_ui.ThreadingHTTPServer(('127.0.0.1', 0), fleet_ui.Handler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        import urllib.request
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        def api(path, body=None):
            req = urllib.request.Request(f'http://127.0.0.1:{server.server_port}'+path,
                data=None if body is None else json.dumps(body).encode(),
                headers={'Content-Type': 'application/json'})
            with opener.open(req, timeout=5) as response:
                return json.load(response)
        try:
            scan = api('/api/scan', {})
            self.assertEqual(len(scan['found']), 8)
            self.assertEqual(scan['changes'], [])
            specs = [dict(id=cid, delay=i*.04, **PLAN) for i, cid in enumerate(fleet.cars)]
            self.assertEqual(api('/api/start', dict(cars=specs, sync=True))['error'], '')
            wait_until(lambda: fleet.group['phase'] == 'done', 10)
            self.assertFalse(fleet.group['degraded'], fleet.group)
            starts = [node.backend.events[0][2] for node in self.nodes]
            # Logical delays can stretch when a deliberately slow car holds
            # the virtual clock, but later slots must never start early.
            self.assertGreater(starts[-1]-starts[0], .15)
            state = api('/api/state')
            self.assertFalse(state['running'])
            self.assertTrue(all(c['phase'] == 'done' for c in state['cars']))
        finally:
            server.shutdown()
            server.server_close()
            fleet_ui.FLEET = original

    def test_lost_commit_ack_is_partial_and_never_replayed(self):
        fleet = self.fleet(1)
        client = fleet.cars['car0'].client
        request = client.request
        def lose_ack(method, *args, **kwargs):
            result = request(method, *args, **kwargs)
            if method == 'commit':
                raise TimeoutError('simulated lost commit ACK')
            return result
        client.request = lose_ack
        self.assertEqual(fleet.start([dict(id='car0', **PLAN)]), '')
        wait_until(lambda: fleet.group['phase'] == 'partial')
        wait_until(lambda: self.nodes[0].phase == 'stopped')
        self.assertFalse(self.nodes[0].backend.events)

    def test_one_way_heartbeat_loss_and_reconnect(self):
        node, client = self.node()
        rid = self.prepare(node, client)
        self.commit(client, rid)
        # Status remains reachable, but no control messages arrive.
        wait_until(lambda: node.degraded, 5)
        client.request('heartbeat', dict(run_id=rid, seq=10, vt=0, limit=0))
        wait_until(lambda: node.phase == 'done')
        self.assertTrue(node.degraded)
        self.assertEqual(len(node.backend.events), 3)

    def test_no_serial_open_on_unconfigured_prepare(self):
        from node_service import Hardware
        with self.assertRaises(ValueError):
            Hardware({'serial_port': None}, lambda _: None)
        with self.assertRaises(ValueError):
            Hardware(dict(serial_port='same', expected_mac='a', tf_port='same'), lambda _: None)

    def test_motor_gate_blocks_late_motion_after_stop(self):
        from node_service import Hardware
        hardware = Hardware.__new__(Hardware)
        hardware.motor_lock = threading.RLock()
        hardware.blocked = True
        sent = []
        hardware.raw_send = sent.append
        for command in ({'T':160,'deg':90}, {'T':165}, {'T':1,'L':.2,'R':.2}):
            with self.assertRaises(w.Aborted):
                hardware._send(command)
        hardware._send({'T':161})
        hardware._send({'T':1,'L':0,'R':0})
        self.assertEqual(len(sent), 2)


if __name__ == '__main__':
    unittest.main(verbosity=2)
