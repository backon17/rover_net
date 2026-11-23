#!/usr/bin/env python3
"""Local node-service CLI. Never opens the chassis serial port itself."""
import argparse
import json
from pathlib import Path
import time
import uuid

from node_client import NodeClient
from node_protocol import motion, TERMINAL
from node_executor import execute_action


def read_plan(path):
    spec = motion(json.loads(Path(path).read_text(encoding='utf-8')))
    return spec['speed'], spec['steps']


def execute(ctrl, speed, steps):
    """Shared executor entry retained for callers without a network transport."""
    ctrl.fw_calibrate()
    heading = 0.
    for step in steps:
        heading = execute_action(ctrl, step, speed, heading)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:8765')
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('devices', help='Enumerate only; do not open serial devices')
    sub.add_parser('status')
    sub.add_parser('stop')
    run = sub.add_parser('run')
    run.add_argument('plan')
    run.add_argument('--dry-run', action='store_true')
    args = parser.parse_args(argv)
    if args.command == 'devices':
        from serial.tools import list_ports
        for port in list_ports.comports():
            print(f'{port.device}\t{port.description}\t{port.hwid}')
        return 0
    if args.command == 'run':
        spec = motion(json.loads(Path(args.plan).read_text(encoding='utf-8')))
        if args.dry_run:
            print(json.dumps(spec, indent=2))
            return 0
    client = NodeClient({'url': args.url})
    if args.command == 'status':
        print(json.dumps(client.request('status'), ensure_ascii=False, indent=2))
        return 0
    state = client.request('status')
    if args.command == 'stop':
        result = client.request('stop', {'run_id': state['run_id']})
        print(json.dumps(result, ensure_ascii=False))
        return 0 if result.get('stop') == 'confirmed' else 1
    run_id = str(uuid.uuid4())
    client.request('prepare', dict(run_id=run_id, stages=[spec], sync=False))
    deadline = time.monotonic() + 30
    while True:
        state = client.request('status')
        if state['run_id'] != run_id or state['phase'] in TERMINAL:
            raise RuntimeError(state.get('error') or 'task interrupted')
        if state['phase'] == 'ready':
            break
        if time.monotonic() > deadline:
            raise TimeoutError('prepare timeout; query status before retrying')
        time.sleep(.1)
    offset = client.clock_sample()
    client.request('commit', dict(run_id=run_id, start_at=time.monotonic()+offset+1, stage_bases=[0]))
    print(f'Task {run_id} committed. Disconnecting this CLI does not cancel it.')
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except Exception as exc:
        raise SystemExit(str(exc))
