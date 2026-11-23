#!/bin/sh
set -eu
cd "$(dirname "$0")"
test "$(id -un)" = rover || { echo 'Run as rover, not root'; exit 1; }
test "$PWD" = /home/rover/rover_control || { echo 'Install path must be /home/rover/rover_control'; exit 1; }
if ! test -f node_config.json; then
    cp node_config.example.json node_config.json
fi
mkdir -p state
if ! test -x .venv/bin/python; then
    python3 -m venv --system-site-packages .venv
fi
.venv/bin/python -c 'import serial; assert serial.VERSION.startswith("3.")'
.venv/bin/python -m compileall -q node_service.py node_protocol.py node_executor.py node_client.py
sudo install -m 644 rover-node.service /etc/systemd/system/rover-node.service
sudo systemctl daemon-reload
sudo systemctl enable rover-node.service
sudo systemctl restart rover-node.service
systemctl --no-pager status rover-node.service
