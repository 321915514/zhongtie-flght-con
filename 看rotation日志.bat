@echo off
chcp 65001 >nul
title 降落 rotation 实时监控 (Ctrl+C 退出)
echo ============================================================
echo  实时看板端 rotation：rect / geo4 / pca4 / send
echo  板子: 192.168.0.199   密码: 见项目约定
echo  判断标准：send 应等于 geo4；geo4 与 pca4/rect 三者接近=稳定
echo ============================================================
echo.
ssh -o StrictHostKeyChecking=no -o PreferredAuthentications=password -o PubkeyAuthentication=no cat@192.168.0.199 "tail -n 20 -f /home/cat/Flght_con/flght_control/install/rk3588_linux/log/rotating_log.log" | findstr /C:"角度对比" /C:"检测结果"
