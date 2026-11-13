@echo off
chcp 65001 >nul
title Rotation Monitor (auto-detect board)
cd /d "%~dp0"
set "UV=C:\Users\Qi\AppData\Local\hermes\bin\uv.exe"
if not exist "%UV%" set "UV=uv"
echo ============================================================
echo  Auto-scan LAN for board, login, then stream rotation log
echo  Press Ctrl+C to quit
echo ============================================================
echo.
"%UV%" run --with paramiko python "%~dp0watch_rotation.py" %*
echo.
pause
