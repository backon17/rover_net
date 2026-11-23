@echo off
rem OTA-update every online car listed in cars.txt that is not on the current firmware.
cd /d D:\PHD\wave_rover
set PYTHONIOENCODING=utf-8
python ota_all.py %*
pause
