@echo off
chcp 65001 >nul
title 降落 rotation 实时监控（自动找板子 / Ctrl+C 退出）
cd /d "%~dp0"

set "UV=C:\Users\Qi\AppData\Local\hermes\bin\uv.exe"
if not exist "%UV%" set "UV=uv"

echo ============================================================
echo  自动扫描板子 IP（hostname=lubancat）并登录，实时显示：
echo    角度对比 rect / geo4 / pca4 / send
echo    检测结果 四孔坐标
echo  判断：geo4 跟随实际转角；正常关系 rect ^> geo4 ^> pca4
echo  Ctrl+C 退出
echo ============================================================
echo.

"%UV%" run --with paramiko python "%~dp0watch_rotation.py" %*
echo.
echo === 已退出 ===
pause
