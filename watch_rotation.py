#!/usr/bin/env python3
"""自动定位板子 + 自动登录 + 实时看 rotation 日志。
用法：双击 看rotation日志.bat，或  uv run --with paramiko python watch_rotation.py
按 Ctrl+C 退出。
"""
import concurrent.futures
import socket
import sys
import time

import paramiko

USER = "cat"
PASS = "temppwd"
HOSTNAME = "lubancat"
LOG = "/home/cat/Flght_con/flght_control/install/rk3588_linux/log/rotating_log.log"
KEEP = ("角度对比", "检测结果", "纠偏")   # 兼容两版日志格式


def probe(ip, timeout=0.7):
    s = socket.socket()
    s.settimeout(timeout)
    try:
        s.connect((ip, 22))
        return ip
    except Exception:
        return None
    finally:
        s.close()


def local_prefixes():
    """取本机所有 IPv4 的前三段，用于决定扫哪些网段"""
    import subprocess
    out = subprocess.run(["ipconfig"], capture_output=True, text=True, errors="ignore").stdout
    pre = set()
    for ln in out.splitlines():
        if "IPv4" in ln or "IPv4 地址" in ln:
            for tok in ln.replace(":", " ").split():
                p = tok.split(".")
                if len(p) == 4 and all(x.isdigit() for x in p[:3]) and tok.count(".") == 3:
                    ip = tok.strip()
                    if ip.startswith(("192.168.", "10.", "172.")) and not ip.startswith("127."):
                        pre.add(".".join(ip.split(".")[:3]))
    return sorted(pre)


def find_board():
    prefixes = local_prefixes() or ["192.168.0"]
    print("扫描网段:", ", ".join(p + ".0/24" for p in prefixes))
    cands = []
    for pre in prefixes:
        ips = [f"{pre}.{i}" for i in range(1, 255)]
        with concurrent.futures.ThreadPoolExecutor(max_workers=200) as ex:
            for r in ex.map(probe, ips):
                if r:
                    cands.append(r)
    print(f"22端口开放: {cands if cands else '无'}")

    # 优先用上次成功的 IP，再逐个验证 hostname
    for ip in cands:
        try:
            c = paramiko.SSHClient()
            c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            c.connect(ip, username=USER, password=PASS, timeout=6, banner_timeout=6, auth_timeout=6)
            hn = c.exec_command("hostname", timeout=6)[1].read().decode(errors="ignore").strip()
            if HOSTNAME in hn:
                c.close()
                return ip
            c.close()
        except Exception:
            continue

    # 都不匹配时，退化为"能登上就提示"
    for ip in cands:
        try:
            c = paramiko.SSHClient()
            c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            c.connect(ip, username=USER, password=PASS, timeout=6, banner_timeout=6, auth_timeout=6)
            c.close()
            print(f"!! 未找到 hostname={HOSTNAME}，但 {ip} 可登录，暂用它")
            return ip
        except Exception:
            continue
    return None


def tail(ip):
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(ip, username=USER, password=PASS, timeout=10, banner_timeout=10, auth_timeout=10)
    # -n 30：先补最近 30 行历史，之后持续跟随
    cmd = f"tail -n 30 -f '{LOG}'"
    stdin, stdout, stderr = c.exec_command(cmd, get_pty=False)
    print(f"\n=== 已连接 {ip}，实时日志中（Ctrl+C 退出）===\n")
    try:
        for raw in iter(stdout.readline, ""):
            line = raw.rstrip("\r\n")
            if any(k in line for k in KEEP):
                print(line, flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            stdin.channel.close()
        except Exception:
            pass
        c.close()
    print("\n已断开")


def main():
    ip = sys.argv[1] if len(sys.argv) > 1 else None
    if ip:
        print("使用指定 IP:", ip)
    else:
        while True:
            ip = find_board()
            if ip:
                break
            print("未找到板子，10 秒后重试（Ctrl+C 退出）...")
            try:
                time.sleep(10)
            except KeyboardInterrupt:
                return
    while True:
        try:
            tail(ip)
        except KeyboardInterrupt:
            return
        except Exception as e:
            print(f"连接中断: {e}，3 秒后重连（Ctrl+C 退出）")
            try:
                time.sleep(3)
                ip = ip or find_board()
            except KeyboardInterrupt:
                return


if __name__ == "__main__":
    main()
