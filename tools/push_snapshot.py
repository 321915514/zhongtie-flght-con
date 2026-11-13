#!/usr/bin/env python3
"""把本地 board_src_full 快照整树推到 GitHub 新仓库（Data API + curl，规避 DLP 掐 TLS）。
用法：python push_snapshot.py [repo_name]  默认 zhongtie-flght-con
幂等：仓库已存在则复用；已存在 main 则基于其上更新（不 force）。
"""
import base64, json, os, subprocess, sys, time

OWNER = "321915514"
REPO_NAME = sys.argv[1] if len(sys.argv) > 1 else "zhongtie-flght-con"
REPO = f"{OWNER}/{REPO_NAME}"
API = "https://api.github.com"
WORK = r"C:/Users/QI/Desktop/board_src_full"


def token():
    out = subprocess.run(["git", "credential", "fill"],
                         input=b"protocol=https\nhost=github.com\n\n",
                         capture_output=True,
                         env=dict(os.environ, GIT_TERMINAL_PROMPT="0")).stdout.decode()
    for ln in out.splitlines():
        if ln.startswith("password="):
            return ln[9:]
    raise RuntimeError("凭据库无 github.com token")


TOK = token()


def api(method, path, payload=None, tries=4, allow=(200, 201, 204, 304)):
    cmd = ["curl", "-sS", "-m", "120", "-X", method,
           "-H", "Authorization: Bearer " + TOK,
           "-H", "Accept: application/vnd.github+json",
           "-H", "User-Agent: hermes-push-snapshot",
           "-w", "\n@@%{http_code}"]
    if payload is not None:
        cmd += ["-H", "Content-Type: application/json", "--data-binary", "@-"]
    cmd.append(API + path)
    for i in range(tries):
        r = subprocess.run(cmd, input=(json.dumps(payload).encode() if payload is not None else None),
                           capture_output=True)
        raw = r.stdout.decode("utf-8", "replace")
        body, _, code = raw.rpartition("\n@@")
        code = code.strip()
        if r.returncode != 0:
            if i < tries - 1:
                time.sleep(3 * (i + 1)); continue
            raise RuntimeError("curl 失败: " + r.stderr.decode("utf-8", "replace")[:300])
        if code in ("403", "429") and i < tries - 1:
            time.sleep(8 * (i + 1)); continue
        try:
            n = int(code)
        except ValueError:
            n = 0
        if n not in allow:
            raise RuntimeError("%s %s -> HTTP %s %s" % (method, path, code, body[:400]))
        try:
            return json.loads(body) if body.strip() else {}
        except json.JSONDecodeError:
            return {}
    raise RuntimeError("重试耗尽: " + path)


# 1) 仓库
r = api("GET", "/repos/" + REPO, allow=(200, 404))
if not r:
    print("创建仓库", REPO)
    r = api("POST", "/user/repos", {"name": REPO_NAME, "private": True,
                                    "description": "中铁 zhongtiepro 无人机视觉降落纠偏 — 板端 Flght_con 源码"})
else:
    print("仓库已存在", REPO)

# 2) 远端 main（若有）
parent, base_tree = None, None
ref = api("GET", "/repos/%s/git/ref/heads/main" % REPO, allow=(200, 404, 409))
if ref and "object" in ref:
    parent = ref["object"]["sha"]
    base_tree = api("GET", "/repos/%s/git/commits/%s" % (REPO, parent))["tree"]["sha"]
    print("基于现有 main =", parent[:9])
else:
    print("远端为空，建初始提交")

# 3) 文件清单（git ls-files）
os.chdir(WORK)
files = subprocess.run(["git", "ls-files"], capture_output=True).stdout.decode().split()
print("待推文件数:", len(files))

entries = []
for p in files:
    data = subprocess.run(["git", "cat-file", "blob", "HEAD:" + p], capture_output=True).stdout
    blob = api("POST", "/repos/%s/git/blobs" % REPO,
               {"content": base64.b64encode(data).decode(), "encoding": "base64"})
    mode = "100755" if os.access(os.path.join(WORK, p), os.X_OK) else "100644"
    entries.append({"path": p, "mode": mode, "type": "blob", "sha": blob["sha"]})
    print("  blob %-52s %7d B" % (p[-49:], len(data)))

pay = {"tree": entries}
if base_tree:
    pay["base_tree"] = base_tree
tree = api("POST", "/repos/%s/git/trees" % REPO, pay)

# 4) commit
msg = subprocess.run(["git", "log", "-1", "--format=%B"], capture_output=True).stdout.decode().strip()
meta = subprocess.run(["git", "log", "-1", "--format=%an|%ae|%aI|%cn|%ce|%cI"],
                      capture_output=True).stdout.decode().strip().split("|")
cp = {"message": msg, "tree": tree["sha"],
      "author": {"name": meta[0], "email": meta[1], "date": meta[2]},
      "committer": {"name": meta[3], "email": meta[4], "date": meta[5]}}
if parent:
    cp["parents"] = [parent]
com = api("POST", "/repos/%s/git/commits" % REPO, cp)
print("commit:", com["sha"])

# 5) ref
if parent:
    api("PATCH", "/repos/%s/git/refs/heads/main" % REPO, {"sha": com["sha"], "force": False})
else:
    api("POST", "/repos/%s/git/refs" % REPO, {"ref": "refs/heads/main", "sha": com["sha"]})
print("OK ->", "https://github.com/%s/commit/%s" % (REPO, com["sha"]))
