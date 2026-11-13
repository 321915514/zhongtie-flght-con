// test_hole_track.cc  ——  hole_track.h 离线验证（不依赖 opencv，g++ 直接编）
//
// 基准数据：板端模型在 .JPG 上实测的 8 个孔心（1536x1280 原图坐标）。
//   上板 = 前 4 个（2 行 x 2 列，下标 0/1/2/3），落点 = 上板四孔对角线交点（外心）。
//   下板 = 后 4 个。
// 渲染：model(JPG px) --相似变换--> 1280x720 帧（缩放按 1/h = 下降放大），加 0.8px 噪声后送模块。
//
// 编译：g++ -O2 -std=c++17 -o /tmp/tht test_hole_track.cc
#include "hole_track.h"
#include <cstdio>
#include <cmath>
#include <random>
#include <vector>
#include <algorithm>

using holetrack::Vec2;
using holetrack::Obs;
using holetrack::mk;
using holetrack::vsub;
using holetrack::vadd;
using holetrack::vmul;
using holetrack::vnorm;

static const float FRAME_W = 1280.0f, FRAME_H = 720.0f;

// p = s*m + t
static Vec2 tr(Vec2 m, float s, Vec2 t) { return vadd(vmul(m, s), t); }

// .JPG 实测 8 孔心：前 4 = 上板（0=左上 1=右上 2=左下 3=右下），后 4 = 下板
static const Vec2 MODEL8[8] = {
    {490.5f, 281.0f}, {585.5f, 288.0f}, {489.0f, 376.5f}, {580.5f, 385.0f},
    {482.0f, 511.0f}, {575.0f, 519.0f}, {479.5f, 603.5f}, {571.5f, 613.0f}};

// 对角线交点（外心）：三种配对里取落在包围盒内的那个
static bool quadCenter(const std::vector<Vec2> &p, Vec2 &out)
{
    static const int PR[3][4] = {{0, 1, 2, 3}, {0, 2, 1, 3}, {0, 3, 1, 2}};
    if ((int)p.size() < 4) return false;
    for (int k = 0; k < 3; ++k)
    {
        Vec2 a = p[PR[k][0]], b = p[PR[k][1]], c = p[PR[k][2]], d = p[PR[k][3]];
        float den = (b.x - a.x) * (d.y - c.y) - (b.y - a.y) * (d.x - c.x);
        if (std::fabs(den) < 1e-6f) continue;
        float u = ((c.x - a.x) * (d.y - c.y) - (c.y - a.y) * (d.x - c.x)) / den;
        Vec2 o = mk(a.x + u * (b.x - a.x), a.y + u * (b.y - a.y));
        float xmin = std::min(std::min(a.x, b.x), std::min(c.x, d.x));
        float xmax = std::max(std::max(a.x, b.x), std::max(c.x, d.x));
        float ymin = std::min(std::min(a.y, b.y), std::min(c.y, d.y));
        float ymax = std::max(std::max(a.y, b.y), std::max(c.y, d.y));
        if (o.x < xmin || o.x > xmax || o.y < ymin || o.y > ymax) continue;
        out = o;
        return true;
    }
    return false;
}

// 旧逻辑：当前检出的孔按 y 升序取最上 4 个 -> 外心
static bool oldAim(std::vector<Vec2> det, Vec2 &out)
{
    if (det.size() < 4) return false;
    std::sort(det.begin(), det.end(), [](const Vec2 &a, const Vec2 &b) { return a.y < b.y; });
    std::vector<Vec2> t4(det.begin(), det.begin() + 4);
    return quadCenter(t4, out);
}

struct Frame
{
    std::vector<Obs> obs;
    Vec2 expected;
    int  visible;
};

// 渲染一帧：上板外心放在 aim_px，drop 里的孔不参与（模拟漏检）
static Frame render(float h, Vec2 aim_px, const std::vector<int> &drop, std::mt19937 &rng)
{
    Frame f;
    f.visible = 0;
    const float s = (66.0f / 96.25f) * (1.5f / h); // 1.5m 时上板行距 96.25 模型px -> 66px
    std::vector<Vec2> upm = {MODEL8[0], MODEL8[1], MODEL8[2], MODEL8[3]};
    Vec2 m_aim;
    if (!quadCenter(upm, m_aim)) { f.expected = aim_px; return f; }
    f.expected = aim_px;
    std::normal_distribution<float> nz(0.0f, 0.8f);
    for (int i = 0; i < 8; ++i)
    {
        Vec2 p = tr(vsub(MODEL8[i], m_aim), s, aim_px);
        const bool inside = (p.x > 0 && p.x < FRAME_W && p.y > 0 && p.y < FRAME_H);
        bool dropped = false;
        for (size_t k = 0; k < drop.size(); ++k) if (drop[k] == i) dropped = true;
        if (dropped || !inside) continue;
        Obs o;
        o.c = mk(p.x + nz(rng), p.y + nz(rng));
        o.r = 20.0f * s;
        o.conf = 0.9f;
        f.obs.push_back(o);
        f.visible++;
    }
    return f;
}

static float p95(std::vector<float> v)
{
    if (v.empty()) return -1;
    std::sort(v.begin(), v.end());
    return v[std::min<size_t>(v.size() - 1, (size_t)(v.size() * 0.95))];
}

int main()
{
    std::mt19937 rng(20261111);

    // ---------- A. 下降序列：8 孔 -> 6 孔 -> 4 孔，落点连续 ----------
    printf("== A 下降序列（每档冷启动 200 帧，噪声 0.8px，落点基准 (640,360)）==\n");
    printf("%7s %6s %6s %9s %9s\n", "h(m)", "可见", "有效", "p50(px)", "p95(px)");
    float hs[] = {3.0f, 1.5f, 1.0f, 0.85f, 0.72f, 0.60f, 0.45f, 0.38f};
    for (int hi = 0; hi < 8; ++hi)
    {
        std::vector<float> errs;
        int ok = 0, vis = 0;
        for (int it = 0; it < 200; ++it)
        {
            holetrack::UpperPlateLock lk;
            Frame f = render(hs[hi], mk(640.0f, 360.0f), {}, rng);
            holetrack::Result r = lk.update(f.obs, FRAME_W, FRAME_H);
            vis = f.visible;
            if (r.state == 1) { ++ok; errs.push_back(vnorm(vsub(r.aim, f.expected))); }
        }
        std::sort(errs.begin(), errs.end());
        printf("%7.2f %6d %6d %9.2f %9.2f\n", hs[hi], vis, ok,
               errs.empty() ? -1.0f : errs[errs.size() / 2], p95(errs));
    }

    // ---------- B. 已锁定后缺孔（先 8 孔锁住，再喂缺孔帧）----------
    printf("\n== B 已锁定后缺孔（h=1.5m，帧1=8孔锁定，帧2=缺孔，300 次取 p95）==\n");
    struct Case { const char *name; std::vector<int> drop; };
    std::vector<Case> cases;
    cases.push_back({"上板满 4 孔",           std::vector<int>()});
    cases.push_back({"缺 1（右上 R1R）",      {1}});
    cases.push_back({"缺 2（右列 R1R+R2R）",  {1, 3}});
    cases.push_back({"缺 2（下排 R2L+R2R）",  {2, 3}});
    cases.push_back({"缺 2（上排 R1L+R1R）",  {0, 1}});
    cases.push_back({"剩对角 2（R1L+R2R）",   {1, 2}});
    cases.push_back({"上板剩 2 + 下板 4",     {1, 3}});
    cases.push_back({"上板 4 全丢（只剩下板）", {0, 1, 2, 3}});
    printf("%-24s %6s %9s %9s %9s %8s\n", "情形", "上板k", "旧 p95", "新 p95", "新 state", "冷启动");
    for (size_t ci = 0; ci < cases.size(); ++ci)
    {
        std::vector<float> e_old, e_new;
        int n_up = 0, st = -1, cold_ok = 0;
        std::vector<float> e_cold;
        for (int it = 0; it < 300; ++it)
        {
            holetrack::UpperPlateLock lk;
            Frame f0 = render(1.5f, mk(640.0f, 360.0f), {}, rng);
            lk.update(f0.obs, FRAME_W, FRAME_H);                       // 帧1：8 孔建身份
            Frame f1 = render(1.5f, mk(640.0f, 360.0f), cases[ci].drop, rng);
            holetrack::Result r = lk.update(f1.obs, FRAME_W, FRAME_H);  // 帧2：缺孔
            n_up = r.n_used;
            st = r.state;
            if (r.state == 1) e_new.push_back(vnorm(vsub(r.aim, f1.expected)));

            std::vector<Vec2> det;
            for (size_t k = 0; k < f1.obs.size(); ++k) det.push_back(f1.obs[k].c);
            Vec2 oa;
            if (oldAim(det, oa)) e_old.push_back(vnorm(vsub(oa, f1.expected)));

            holetrack::UpperPlateLock lk2;                              // 对照：冷启动
            holetrack::Result r2 = lk2.update(f1.obs, FRAME_W, FRAME_H);
            if (r2.state == 1) { cold_ok++; e_cold.push_back(vnorm(vsub(r2.aim, f1.expected))); }
        }
        printf("%-24s %6d %9.2f %9.2f %9d %8d\n", cases[ci].name, n_up,
               p95(e_old), p95(e_new), st, cold_ok);
        if (cold_ok > 0 && cold_ok < 300)
            printf("    （冷启动 %d/300 帧成功，其误差 p95=%.1fpx）\n", cold_ok, p95(e_cold));
    }

    // ---------- C. 连续帧：上板逐个丢孔，落点是否跳变 ----------
    printf("\n== C 连续帧：上板逐个丢孔，落点跳变检查（h=1.5m）==\n");
    {
        holetrack::UpperPlateLock lk;
        Vec2 prev = mk(0, 0);
        std::vector<std::vector<int> > seq;
        const char *names[] = {"8孔全", "8孔全", "缺右上", "缺右上", "缺右列", "缺右列",
                              "剩对角", "剩对角", "0孔", "0孔", "8孔全"};
        std::vector<int> s0;                    seq.push_back(s0);
        std::vector<int> s1;                    seq.push_back(s1);
        std::vector<int> s2; s2.push_back(1);    seq.push_back(s2);
        seq.push_back(s2);
        std::vector<int> s3; s3.push_back(1); s3.push_back(3); seq.push_back(s3);
        seq.push_back(s3);
        std::vector<int> s4; s4.push_back(1); s4.push_back(2); seq.push_back(s4);
        seq.push_back(s4);
        std::vector<int> s5; s5.push_back(-1);   seq.push_back(s5);
        seq.push_back(s5);
        seq.push_back(s0);
        for (int i = 0; i < 11; ++i)
        {
            Frame f;
            if (seq[i].size() == 1 && seq[i][0] == -1) { f.expected = mk(640.0f, 360.0f); }
            else f = render(1.5f, mk(640.0f, 360.0f), seq[i], rng);
            holetrack::Result r = lk.update(f.obs, FRAME_W, FRAME_H);
            printf("  #%02d %-7s state=%d k=%d 剔除=%d aim=(%.1f,%.1f) 位移=%5.1fpx 误差=%.2fpx\n",
                   i, names[i], r.state, r.n_used, r.n_cull, r.aim.x, r.aim.y,
                   i > 0 ? vnorm(vsub(r.aim, prev)) : 0.0f,
                   vnorm(vsub(r.aim, f.expected)));
            prev = r.aim;
        }
    }

    // ---------- D. 静止抖动 ----------
    printf("\n== D 静止抖动（h=1.5m，300 帧，8 孔）==\n");
    {
        holetrack::UpperPlateLock lk;
        std::vector<float> xs, ys;
        for (int it = 0; it < 300; ++it)
        {
            Frame f = render(1.5f, mk(640.0f, 360.0f), {}, rng);
            holetrack::Result r = lk.update(f.obs, FRAME_W, FRAME_H);
            if (r.state == 1) { xs.push_back(r.aim.x); ys.push_back(r.aim.y); }
        }
        float mx = 0, my = 0;
        for (size_t i = 0; i < xs.size(); ++i) { mx += xs[i]; my += ys[i]; }
        mx /= (xs.empty() ? 1 : xs.size()); my /= (ys.empty() ? 1 : ys.size());
        float sx = 0, sy = 0;
        for (size_t i = 0; i < xs.size(); ++i) { sx += (xs[i] - mx) * (xs[i] - mx); sy += (ys[i] - my) * (ys[i] - my); }
        sx = std::sqrt(sx / std::max<size_t>(1, xs.size() - 1));
        sy = std::sqrt(sy / std::max<size_t>(1, ys.size() - 1));
        printf("  n=%zu  均值=(%.2f,%.2f)  std=(%.2f,%.2f)px\n", xs.size(), mx, my, sx, sy);
    }

    // ---------- E. 上板彻底丢失后，会不会锁到下面那块板 ----------
    printf("\n== E 上板 4 孔全丢后能否拒绝重锁到下板（h=1.5m）==\n");
    {
        holetrack::UpperPlateLock lk;
        std::vector<int> none, low;
        low.push_back(0); low.push_back(1); low.push_back(2); low.push_back(3);
        for (int i = 0; i < 2; ++i)
        {
            Frame f = render(1.5f, mk(640.0f, 360.0f), none, rng);
            holetrack::Result r = lk.update(f.obs, FRAME_W, FRAME_H);
            printf("  #%d 8孔      state=%d k=%d aim=(%.1f,%.1f) 误差=%.2f\n", i, r.state, r.n_used, r.aim.x, r.aim.y,
                   vnorm(vsub(r.aim, f.expected)));
        }
        for (int i = 0; i < 8; ++i)
        {
            Frame f = render(1.5f, mk(640.0f, 360.0f), low, rng); // 只剩下面 4 孔
            holetrack::Result r = lk.update(f.obs, FRAME_W, FRAME_H);
            printf("  #%d 只剩下板  state=%d k=%d aim=(%.1f,%.1f) 误差=%.2f %s\n", i, r.state, r.n_used,
                   r.aim.x, r.aim.y, vnorm(vsub(r.aim, f.expected)),
                   (r.state == 1 && vnorm(vsub(r.aim, f.expected)) > 30) ? "<< 锁错板!" : "");
        }
        for (int i = 0; i < 2; ++i)
        {
            Frame f = render(1.5f, mk(640.0f, 360.0f), none, rng); // 上板回来
            holetrack::Result r = lk.update(f.obs, FRAME_W, FRAME_H);
            printf("  #%d 8孔回归   state=%d k=%d aim=(%.1f,%.1f) 误差=%.2f\n", i, r.state, r.n_used, r.aim.x, r.aim.y,
                   vnorm(vsub(r.aim, f.expected)));
        }
    }
    return 0;
}
