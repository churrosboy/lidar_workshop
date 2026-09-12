@echo off
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0start-lidar.ps1"
if errorlevel 1 pause
