@echo off
rem WAVE ROVER multi-car trajectory console: opens http://localhost:8090
cd /d %~dp0
python fleet_ui.py %*
pause
