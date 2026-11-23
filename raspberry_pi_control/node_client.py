"""Node RPC client. SSH/IP is one adapter; TF adapters implement request()."""
import json
import socket
import subprocess
import threading
import time
import urllib.error
import urllib.request
import uuid

from node_protocol import VERSION


class NodeClient:
    def __init__(self, config):
        self.config = dict(config)
        self.process = None
        self.port = None
        self.lock = threading.Lock()
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def _url(self):
        # Explicit loopback-only HTTP adapter is useful for simulation/local CLI.
        if self.config.get('url'):
            url = self.config['url'].rstrip('/')
            from urllib.parse import urlparse
            if urlparse(url).hostname not in ('localhost', '127.0.0.1', '::1'):
                raise ValueError('direct HTTP is restricted to loopback; use SSH')
            return url
        with self.lock:
            if self.process is None or self.process.poll() is not None:
                with socket.socket() as sock:
                    sock.bind(('127.0.0.1', 0))
                    self.port = sock.getsockname()[1]
                host = self.config['ssh_host']
                if not isinstance(host, str) or host.startswith('-') or any(c.isspace() for c in host):
                    raise ValueError('invalid ssh_host')
                remote = int(self.config.get('remote_port', 8765))
                if not 1 <= remote <= 65535:
                    raise ValueError('invalid remote_port')
                self.process = subprocess.Popen([
                    'ssh', '-N', '-T', '-o', 'BatchMode=yes',
                    '-o', 'ExitOnForwardFailure=yes', '-o', 'ConnectTimeout=5',
                    '-o', 'ServerAliveInterval=2', '-o', 'ServerAliveCountMax=2',
                    '-L', f'127.0.0.1:{self.port}:127.0.0.1:{remote}', host,
                ], stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
                deadline = time.monotonic() + 6
                while time.monotonic() < deadline:
                    if self.process.poll() is not None:
                        raise ConnectionError('SSH tunnel failed; check ssh alias/key and node service')
                    try:
                        with socket.create_connection(('127.0.0.1', self.port), .1):
                            break
                    except OSError:
                        time.sleep(.05)
                else:
                    self.process.terminate()
                    raise TimeoutError('SSH tunnel timeout')
            return f'http://127.0.0.1:{self.port}'

    def request(self, method, body=None, timeout=3, request_id=None):
        envelope = {'version': VERSION, 'request_id': request_id or str(uuid.uuid4()),
                    **(body or {})}
        req = urllib.request.Request(self._url() + '/v1/' + method,
                                     data=json.dumps(envelope, allow_nan=False).encode(),
                                     headers={'Content-Type': 'application/json'})
        try:
            with self.opener.open(req, timeout=timeout) as response:
                result = json.load(response)
        except urllib.error.HTTPError as exc:
            try:
                message = json.load(exc).get('error', str(exc))
            except (ValueError, OSError):
                message = str(exc)
            raise ValueError(message) from exc
        if result.get('version') != VERSION:
            raise ValueError('unsupported node protocol')
        expected = self.config.get('node_id')
        if expected and result.get('node_id') != expected:
            raise ValueError('node identity mismatch')
        return result

    def clock_sample(self):
        samples = []
        for _ in range(5):
            before = time.monotonic()
            state = self.request('status')
            after = time.monotonic()
            samples.append((after - before, state['monotonic'] - (before + after) / 2))
        rtt, offset = min(samples)
        if rtt > .1 or max(s[0] for s in samples) - rtt > .05:
            raise ValueError('network timing unstable; RTT must be <=100ms, jitter <=50ms')
        return offset

    def close(self):
        if self.process and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
