@echo off
rem Set up one new car on USB: flash V2, join YOUR_WIFI_SSID, register in cars.txt, self-check.
cd /d D:\PHD\wave_rover
set PYTHONIOENCODING=utf-8
python provision_car.py --backup %*
pause
