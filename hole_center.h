// ============================================================================
//  hole_center.h  —— 孔板"中心"的统一实现（外心 / 对角线交点，不是质心）
//  ---------------------------------------------------------------------------
//  背景：yolov8seg_videocapture_demo.cc:1603 用 calculateMeanPoint() 取四点
//        算术平均（质心）当 aim。质心 != 孔板中心：
//          · 4 孔规整成矩形时 质心 ≡ 对角线交点，差别只有几个像素；
//          · 丢 1 孔（只剩 3 孔）时 质心朝"缺的那颗孔的反侧"偏 √2·s/6
//            （s=孔间距），实测偏移 = 对角线长度 / 6，会把本来在 10 px 死区
//            内的那根轴顶到 0.0555 档，多下 5.5 cm 且方向错 90°。
//        外心（三条中垂线交点 = 矩形对角线交点）在丢孔时偏 0。
//
//  用法（替换 :1603 一行）：
//      #include "hole_center.h"
//      cv::Point2f aim = holecenter::centerOfHoles(top_centers);
//      // 需要时同时取稳健性信息：
//      // holecenter::Result r = holecenter::analyze(top_centers);
//      // r.aim / r.mode / r.spread_px
//
//  设计约束：
//    · 纯头文件，无第三方依赖（不 include opencv，模板只要求 .x/.y 与构造）；
//    · 不改动主程序其它逻辑，可单独编译单测；
//    · 退化为均值时与旧行为完全一致（<3 点 / 四点严重不共面 / 对角线不相交）。
// ============================================================================
#ifndef HOLE_CENTER_H
#define HOLE_CENTER_H

#include <vector>
#include <cmath>
#include <algorithm>

namespace holecenter {

// 与单测/日志对齐的说明性枚举
enum Mode {
    MODE_MEAN       = 0,   // 退化回落：算术平均（旧行为）
    MODE_QUAD_CROSS = 1,   // 4 孔：四条边择优后取两条对角线交点
    MODE_TRIPLE_MID = 2    // 3 孔：最远两点连线的中点（= 直角三角形的外心）
};

struct Result {
    double x = 0.0, y = 0.0;   // 中心（像素）
    int    mode = MODE_MEAN;
    int    n = 0;              // 参与计算的孔数
    double spread_px = 0.0;    // 交点到四点均值距离（评估"是否真的像矩形"）
};

namespace detail {

template <typename Pt>
inline double d2(const Pt& a, const Pt& b) {
    const double dx = double(a.x) - double(b.x);
    const double dy = double(a.y) - double(b.y);
    return dx * dx + dy * dy;
}

template <typename Pt>
inline Pt meanPoint(const std::vector<Pt>& p) {
    double sx = 0.0, sy = 0.0;
    for (const auto& q : p) { sx += double(q.x); sy += double(q.y); }
    const double n = p.empty() ? 1.0 : double(p.size());
    return Pt(float(sx / n), float(sy / n));
}

// 线段 ab 与 cd 的交点；不相交（含平行）返回 false
template <typename Pt>
inline bool segIntersect(const Pt& a, const Pt& b, const Pt& c, const Pt& d,
                         double& ox, double& oy) {
    const double x1 = a.x, y1 = a.y, x2 = b.x, y2 = b.y;
    const double x3 = c.x, y3 = c.y, x4 = d.x, y4 = d.y;
    const double den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if (std::fabs(den) < 1e-9) return false;
    const double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den;
    const double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / den;
    if (t < 0.0 || t > 1.0 || u < 0.0 || u > 1.0) return false;  // 延长线交叉不算
    ox = x1 + t * (x2 - x1);
    oy = y1 + t * (y2 - y1);
    return true;
}

// 4 个孔 → 择优配对（把 4 点分成两条对角线），取更像矩形的那一组交点
template <typename Pt>
inline bool quadCenter(const Pt& p0, const Pt& p1, const Pt& p2, const Pt& p3,
                       double& ox, double& oy) {
    const Pt* p[4] = { &p0, &p1, &p2, &p3 };
    // 三种分法：(01,23) (02,13) (03,12)
    static const int pairings[3][4] = { {0,1,2,3}, {0,2,1,3}, {0,3,1,2} };

    double mx = 0.0, my = 0.0;
    for (int i = 0; i < 4; ++i) { mx += p[i]->x; my += p[i]->y; }
    mx *= 0.25; my *= 0.25;

    bool   ok = false;
    double bestScore = 1e18, bx = 0.0, by = 0.0;
    for (int k = 0; k < 3; ++k) {
        const Pt& a = *p[pairings[k][0]];
        const Pt& b = *p[pairings[k][1]];
        const Pt& c = *p[pairings[k][2]];
        const Pt& d = *p[pairings[k][3]];
        double ix, iy;
        if (!segIntersect(a, b, c, d, ix, iy)) continue;

        const double L1 = std::sqrt(d2(a, b));
        const double L2 = std::sqrt(d2(c, d));
        if (L1 < 1e-6 || L2 < 1e-6) continue;

        // 越像矩形：两条对角线越长越接近，且交点越贴近四点均值
        const double ratio  = std::fabs(L1 - L2) / (L1 + L2);          // 0 = 等长
        const double offset = std::sqrt((ix - mx) * (ix - mx) + (iy - my) * (iy - my))
                              / (0.5 * (L1 + L2));                     // 0 = 交点居中
        const double score  = ratio + offset;
        if (score < bestScore) { bestScore = score; bx = ix; by = iy; ok = true; }
    }
    if (ok) { ox = bx; oy = by; }
    return ok;
}

} // namespace detail

// ---------------------------------------------------------------------------
//  主入口：n>=4 取最优四点的对角线交点；n==3 取最远两点中点；否则回落均值
// ---------------------------------------------------------------------------
template <typename Pt>
inline Result analyze(const std::vector<Pt>& pts) {
    Result r;
    r.n = int(pts.size());
    if (pts.size() < 3) {                       // 孔数不足：旧行为
        Pt m = detail::meanPoint(pts);
        // 仅用于日志：空输入时给出 NaN 语义的点
        if (pts.empty()) { r.x = r.y = 0.0; }
        else             { r.x = m.x; r.y = m.y; }
        r.mode = MODE_MEAN;
        return r;
    }

    if (pts.size() == 3) {
        const Pt& a = pts[0]; const Pt& b = pts[1]; const Pt& c = pts[2];
        const double dab = detail::d2(a, b);
        const double dbc = detail::d2(b, c);
        const double dac = detail::d2(a, c);
        // 斜边 = 最远两点；矩形中心 = 斜边中点
        if (dab >= dbc && dab >= dac) {
            r.x = 0.5 * (double(a.x) + double(b.x)); r.y = 0.5 * (double(a.y) + double(b.y));
        } else if (dbc >= dac) {
            r.x = 0.5 * (double(b.x) + double(c.x)); r.y = 0.5 * (double(b.y) + double(c.y));
        } else {
            r.x = 0.5 * (double(a.x) + double(c.x)); r.y = 0.5 * (double(a.y) + double(c.y));
        }
        r.mode = MODE_TRIPLE_MID;
        return r;
    }

    // n>=4：从所有四点组合里挑分数最低的一组（n 很小，组合数 ≤ 70）
    double bestScore = 1e18, bx = 0.0, by = 0.0;
    bool found = false;
    const int n = int(pts.size());
    const Pt mAll = detail::meanPoint(pts);
    for (int i = 0; i < n; ++i)
    for (int j = i + 1; j < n; ++j)
    for (int k = j + 1; k < n; ++k)
    for (int l = k + 1; l < n; ++l) {
        double ix, iy;
        if (!detail::quadCenter(pts[i], pts[j], pts[k], pts[l], ix, iy)) continue;
        const double L1 = std::sqrt(detail::d2(pts[i], pts[j]));
        const double L2 = std::sqrt(detail::d2(pts[k], pts[l]));
        if (L1 < 1e-6 || L2 < 1e-6) continue;
        const double ratio  = std::fabs(L1 - L2) / (L1 + L2);
        const double offset = std::sqrt((ix - mAll.x) * (ix - mAll.x) + (iy - mAll.y) * (iy - mAll.y))
                              / (0.5 * (L1 + L2));
        const double score  = ratio + offset;
        if (score < bestScore) { bestScore = score; bx = ix; by = iy; found = true; }
    }

    Pt mean = mAll;
    if (found) {
        r.x = bx; r.y = by; r.mode = MODE_QUAD_CROSS;
        r.spread_px = std::sqrt((bx - mean.x) * (bx - mean.x) + (by - mean.y) * (by - mean.y));
    } else {
        r.x = mean.x; r.y = mean.y; r.mode = MODE_MEAN;
    }
    return r;
}

// 只要点：与旧调用一行对换
template <typename Pt>
inline Pt centerOfHoles(const std::vector<Pt>& pts) {
    Result r = analyze(pts);
    return Pt(float(r.x), float(r.y));
}

} // namespace holecenter

#endif // HOLE_CENTER_H
