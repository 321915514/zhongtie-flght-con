#!/bin/bash
# ============================================================
# 一键回退 rotation 到 2026-11-12 版本（rect 角度，无 PCA）
# 用法（板子上）：sudo bash /home/cat/rollback_rotation.sh        # 回到 11-12 版(推荐)
#                 sudo bash /home/cat/rollback_rotation.sh keep-log # 只退 PCA，保留对比日志
# ============================================================
set -e
SRC=/home/cat/Flght_con/flght_control/yolov8seg_videocapture_demo.cc
TS=$(date +%Y%m%d_%H%M)

if [ "$1" = "keep-log" ]; then
    BAK=/home/cat/backups/yolov8seg_videocapture_demo.cc.bak_pca4_20261113_1014
    EXPECT=6db5121d2ed9e9e7fc018c5dcf128bd1
    echo ">>> 回退到【rect 角度 + 角度对比日志】(11-13 10:11 版)"
else
    BAK=/home/cat/backups/yolov8seg_videocapture_demo.cc.bak_anglog_20261113_1011
    EXPECT=c27a8d8168d858a6208e27c3f31ea6aa
    echo ">>> 回退到【11-12 17:05 版：限幅30°+死区3°+rect 角度，无 PCA/无对比日志】"
fi

[ -f "$BAK" ] || { echo "!! 备份不存在: $BAK"; exit 1; }
GOT=$(md5sum "$BAK" | cut -d' ' -f1)
[ "$GOT" = "$EXPECT" ] || { echo "!! 备份 md5 不匹配 期望=$EXPECT 实际=$GOT"; exit 1; }

echo "--- 先备份当前版本以便反悔"
cp "$SRC" "/home/cat/backups/yolov8seg_videocapture_demo.cc.bak_before_rollback_$TS"
echo "    当前已备份: bak_before_rollback_$TS"

echo "--- 覆盖并编译"
cp "$BAK" "$SRC"
cd /home/cat/Flght_con/flght_control
rm -rf build
./build-linux.sh -t rk3588 > /tmp/rollback_${TS}.log 2>&1 || { echo "!! 编译失败，见 /tmp/rollback_${TS}.log"; exit 1; }
grep -c "error:" /tmp/rollback_${TS}.log || true

echo "--- 重启服务"
sudo systemctl restart yolov8seg
sleep 8
echo "服务状态: $(systemctl is-active yolov8seg)"
echo "线上 md5: $(md5sum $SRC | cut -d' ' -f1)"
echo ">>> 完成。日志: /tmp/rollback_${TS}.log"
