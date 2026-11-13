// dbg_rows.cc —— 打印 tryLock 前的行聚类与 aspect，定位冷启动为何仍能锁
#include "hole_track.h"
#include <cstdio>
#include <cmath>
#include <random>
#include <vector>
#include <algorithm>
using holetrack::Vec2; using holetrack::Obs; using holetrack::mk;

static const Vec2 MODEL8[8] = {
    {490.5f, 281.0f}, {585.5f, 288.0f}, {489.0f, 376.5f}, {580.5f, 385.0f},
    {482.0f, 511.0f}, {575.0f, 519.0f}, {479.5f, 603.5f}, {571.5f, 613.0f}};

int main()
{
    const float s = 66.0f / 96.25f * 1.0f; // h=1.5
    std::vector<int> drops[3] = {{2, 3}, {0, 1}, {1, 3}};
    const char *nm[3] = {"缺下排 2,3", "缺上排 0,1", "缺右列 1,3"};
    for (int c = 0; c < 3; ++c)
    {
        std::vector<Obs> pool;
        for (int i = 0; i < 8; ++i)
        {
            bool d = false;
            for (size_t k = 0; k < drops[c].size(); ++k) if (drops[c][k] == i) d = true;
            if (d) continue;
            Obs o; o.c = mk(640.0f + (MODEL8[i].x - 536.0f) * s, 360.0f + (MODEL8[i].y - 333.0f) * s);
            o.r = 20.0f * s; o.conf = 0.9f; pool.push_back(o);
        }
        float rmean = 0; for (size_t i = 0; i < pool.size(); ++i) rmean += pool[i].r; rmean /= pool.size();
        float tol = std::max(4.0f, 0.8f * rmean);
        std::sort(pool.begin(), pool.end(), [](const Obs &a, const Obs &b) { return a.c.y < b.c.y; });
        printf("\n[%s] n=%zu rmean=%.1f tol=%.1f\n", nm[c], pool.size(), rmean, tol);
        std::vector<std::vector<Obs> > rows;
        for (size_t i = 0; i < pool.size(); ++i)
        {
            if (rows.empty() || pool[i].c.y - rows.back().back().c.y > tol) rows.push_back(std::vector<Obs>());
            rows.back().push_back(pool[i]);
        }
        for (size_t r = 0; r < rows.size(); ++r)
        {
            printf("  row%zu y=", r);
            for (size_t k = 0; k < rows[r].size(); ++k) printf("%.1f ", rows[r][k].c.y);
            printf(" x=");
            for (size_t k = 0; k < rows[r].size(); ++k) printf("%.1f ", rows[r][k].c.x);
            printf("\n");
        }
        if (rows.size() >= 2)
        {
            std::vector<Obs> a = rows[0], b = rows[1];
            std::sort(a.begin(), a.end(), [](const Obs &x, const Obs &y) { return x.c.x < y.c.x; });
            std::sort(b.begin(), b.end(), [](const Obs &x, const Obs &y) { return x.c.x < y.c.x; });
            if (a.size() >= 2 && b.size() >= 2)
            {
                float rowgap = 0.5f * ((b.front().c.y + b.back().c.y) - (a.front().c.y + a.back().c.y));
                float colgap = 0.5f * ((a.back().c.x + b.back().c.x) - (a.front().c.x + b.front().c.x));
                printf("  rowgap=%.1f colgap=%.1f aspect=%.2f -> %s\n", rowgap, colgap, rowgap / colgap,
                       (rowgap / colgap > 1.90f || rowgap / colgap < 0.40f) ? "REJECT" : "PASS");
            }
        }
        holetrack::UpperPlateLock lk;
        holetrack::Result r = lk.update(pool, 1280.0f, 720.0f);
        printf("  lock: state=%d k=%d aim=(%.1f,%.1f) 期望=(640,360) 误差=%.1f\n", r.state, r.n_used,
               r.aim.x, r.aim.y, std::hypot(r.aim.x - 640, r.aim.y - 360));
    }
    return 0;
}
