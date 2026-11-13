#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""aim_trace.py —— 落点跟踪记录/分析（板端运行，只读，不碰服务）

配合 yolov8seg 里 2026-11-11 加的“只读调试轨迹”：
存在 /tmp/aim_trace_on 时，程序把每个有效帧追加一行到 /tmp/aim_trace.csv
列：t,aimx,aimy,k,cull,state,scale,rot,n_hole,dx,dy

用法（板上）：
  python3 aim_trace.py rec 40          # 录 40 秒后自动 stop 并出报告（最省事）
  python3 aim_trace.py start           # 开始录（truncate）
  python3 aim_trace.py stop            # 停止录
  python3 aim_trace.py report          # 分析当前 CSV
  python3 aim_trace.py report --tail 30

判定口径（“跳变”= 相邻有效帧落点位移 > 0.5 个孔距）：
  旧逻辑在上板漏孔时会跳到下面那块板 -> 单帧 100 px 级；新逻辑应 < 3 px。
"""
import os
import sys
import statistics

FLAG = os.environ.get("TRACE_FLAG", "/tmp/aim_trace_on")
CSV = os.environ.get("TRACE_CSV", "/tmp/aim_trace.csv")
COLS = ["t", "aimx", "aimy", "k", "cull", "state", "scale", "rot", "nhole", "dx", "dy"]


def cmd_start():
    with open(CSV, "w") as f:
        f.write(",".join(COLS) + "\n")
    open(FLAG, "w").close()
    print("录制已开启（服务会在约 0.6 s 内检测到标记文件）")
    print("现在开始移动下面的纸；结束后执行: python3 %s stop" % os.path.basename(__file__))


def cmd_stop():
    if os.path.exists(FLAG):
        os.remove(FLAG)
        print("录制已停止（服务约 0.6 s 内关闭文件）")
    else:
        print("本来就没有在录")


def load():
    if not os.path.exists(CSV):
        return []
    rows = []
    with open(CSV) as f:
        for line in f:
            p = line.strip().split(",")
            if len(p) < len(COLS):
                continue
            try:
                d = dict(zip(COLS, p))
                rows.append({k: float(v) for k, v in d.items()})
            except ValueError:
                continue          # 表头
    return rows


def pct(v, q):
    if not v:
        return 0.0
    v = sorted(v)
    i = min(len(v) - 1, max(0, int(round(q * (len(v) - 1)))))
    return v[i]


def report(tail=0):
    rows = load()
    name = os.path.basename(__file__)
    if not rows:
        print("CSV 里没有有效帧。检查三点：")
        print("  1) 画面里是否真的有孔板（/tmp/aim_trace_on 存在时才记录）")
        print("  2) 日志里有没有 “上板锁定” 行：tail -20 rotating_log.log")
        print("  3) 录制时长是否够（有效帧约 3 Hz）")
        return 2

    dur = rows[-1]["t"] - rows[0]["t"]
    fps = (len(rows) - 1) / dur if dur > 0 else 0
    st1 = sum(1 for r in rows if int(r["state"]) == 1)
    st2 = sum(1 for r in rows if int(r["state"]) == 2)

    # 相邻帧位移（按该帧 scale 归一化成“孔距倍数”）
    steps, norm = [], []
    for a, b in zip(rows, rows[1:]):
        d = ((b["aimx"] - a["aimx"]) ** 2 + (b["aimy"] - a["aimy"]) ** 2) ** 0.5
        steps.append(d)
        norm.append(d / a["scale"] if a["scale"] > 0 else 0)

    jumps = [i for i, n in enumerate(norm) if n > 0.5]
    path = sum(steps)
    kk = [int(r["k"]) for r in rows]

    print("=" * 62)
    print("落点跟踪报告  %s" % CSV)
    print("=" * 62)
    print("有效帧        : %d  (state=1 求解 %d / state=2 保持 %d)" % (len(rows), st1, st2))
    print("录制时长/帧率 : %.1f s / %.2f Hz" % (dur, fps))
    print("参与求解孔数  : 2 孔 %d 帧, 3 孔 %d 帧, 4 孔 %d 帧"
          % (kk.count(2), kk.count(3), kk.count(4)))
    print("落点覆盖范围  : x [%.1f, %.1f]  y [%.1f, %.1f]"
          % (min(r["aimx"] for r in rows), max(r["aimx"] for r in rows),
             min(r["aimy"] for r in rows), max(r["aimy"] for r in rows)))
    print("累计移动距离  : %.1f px" % path)
    print("-" * 62)
    print("单帧位移(px)  : p50 %.2f  p95 %.2f  max %.2f" % (pct(steps, .5), pct(steps, .95), max(steps)))
    print("单帧位移(孔距): p50 %.3f  p95 %.3f  max %.3f"
          % (pct(norm, .5), pct(norm, .95), max(norm)))
    print("跳变帧(>0.5孔距): %d" % len(jumps))
    if jumps:
        for i in jumps[:10]:
            a, b = rows[i], rows[i + 1]
            print("   !! t=%.2f  k=%d  (%.1f,%.1f)->(%.1f,%.1f)  %.1f px"
                  % (b["t"] - rows[0]["t"], int(b["k"]), a["aimx"], a["aimy"],
                     b["aimx"], b["aimy"], steps[i]))

    # 命令档位分布（控制侧实际吃到的值）
    def dist(key):
        c = {}
        for r in rows:
            v = round(r[key], 4)
            c[v] = c.get(v, 0) + 1
        return sorted(c.items(), key=lambda kv: -kv[1])[:5]
    print("-" * 62)
    print("dx 档位       : %s" % ", ".join("%.4f×%d" % kv for kv in dist("dx")))
    print("dy 档位       : %s" % ", ".join("%.4f×%d" % kv for kv in dist("dy")))

    if tail:
        print("-" * 62)
        print("最后 %d 帧：" % tail)
        print("   t     k cull st   aimx    aimy   scale   dx      dy")
        for r in rows[-tail:]:
            print("  %6.2f  %d  %3d  %d  %7.1f %7.1f  %6.1f  %+.4f %+.4f"
                  % (r["t"] - rows[0]["t"], int(r["k"]), int(r["cull"]), int(r["state"]),
                     r["aimx"], r["aimy"], r["scale"], r["dx"], r["dy"]))

    print("=" * 62)
    ok = len(jumps) == 0 and fps > 1.5
    print("结论: %s  （跳变 %d 帧，帧率 %.2f Hz）"
          % ("PASS 落点连续、无身份错位跳变" if ok else "FAIL 见上面的 !! 行", len(jumps), fps))
    return 0 if ok else 1


def main():
    a = sys.argv[1:] or ["report"]
    if a[0] == "start":
        cmd_start()
    elif a[0] == "stop":
        cmd_stop()
    elif a[0] == "rec":
        import time
        sec = int(a[1]) if len(a) > 1 else 40
        cmd_start()
        try:
            for i in range(sec, 0, -1):
                sys.stdout.write("\r录制中… 剩 %2d s " % i)
                sys.stdout.flush()
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        print("")
        cmd_stop()
        import time as _t
        _t.sleep(1.0)          # 等服务把缓冲刷完
        return report(tail=10)
    elif a[0] == "report":
        tail = int(a[a.index("--tail") + 1]) if "--tail" in a else 0
        return report(tail)
    else:
        print(__doc__)
    return 0


if __name__ == "__main__":
    sys.exit(main())
