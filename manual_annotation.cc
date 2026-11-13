// manual_annotation.cc —— 实现见 manual_annotation.h
#include "manual_annotation.h"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <utility>

namespace manual_annotation {

using json = nlohmann::json;

namespace {

struct Circle { cv::Point2f c; float r; };

// 板面标准几何（1536×1280 图）：列间距 ~816、行间距 ~540、对角 ~979px
static const float BOARD_W_PX    = 816.0f;
static const float BOARD_H_PX    = 540.0f;
static const float BOARD_DIAG_PX = 979.6f;


// ===== 中心计算 =====
// 【保持同步】与 hole_hybrid.cc 的 5.3 节逻辑一致：
//   4 圆 → 对角线交点；3 圆 → 最远点对中点；2 圆 → 对角中点（相邻则失败）；<2 失败
bool compute_center(const std::vector<Circle>& cs, cv::Point2f& cen) {
    const int n = (int)cs.size();

    if (n >= 4) {
        std::vector<Circle> g(cs.begin(), cs.begin() + 4);
        std::sort(g.begin(), g.end(), [](const Circle& a, const Circle& b) { return a.c.y < b.c.y; });
        cv::Point2f TL = (g[0].c.x < g[1].c.x) ? g[0].c : g[1].c;
        cv::Point2f TR = (g[0].c.x < g[1].c.x) ? g[1].c : g[0].c;
        cv::Point2f BL = (g[2].c.x < g[3].c.x) ? g[2].c : g[3].c;
        cv::Point2f BR = (g[2].c.x < g[3].c.x) ? g[3].c : g[2].c;
        float d = (BR.x - TL.x) * (BL.y - TR.y) - (BR.y - TL.y) * (BL.x - TR.x);
        if (std::fabs(d) < 1e-6f) {
            cen = (TL + TR + BL + BR) * 0.25f;   // 退化：平行 → 四圆心平均
        } else {
            float num = (TR.x - TL.x) * (BL.y - TR.y) - (TR.y - TL.y) * (BL.x - TR.x);
            cen = TL + (num / d) * (BR - TL);
        }
        return true;
    }

    if (n == 3) {
        float best = -1.0f; int bi = 0, bj = 1;
        for (int i = 0; i < n; ++i)
            for (int j = i + 1; j < n; ++j) {
                float dx = cs[i].c.x - cs[j].c.x, dy = cs[i].c.y - cs[j].c.y;
                float dd = dx * dx + dy * dy;
                if (dd > best) { best = dd; bi = i; bj = j; }
            }
        cen = (cs[bi].c + cs[bj].c) * 0.5f;      // 最远两点(对角线)中点
        return true;
    }

    if (n == 2) {
        float dx = std::fabs(cs[0].c.x - cs[1].c.x);
        float dy = std::fabs(cs[0].c.y - cs[1].c.y);
        if (dx > DIAGONAL_THRESHOLD_PX && dy > DIAGONAL_THRESHOLD_PX) {  // 对角
            cen = (cs[0].c + cs[1].c) * 0.5f;
            return true;
        }
        return false;   // 相邻，定不了中心
    }

    return false;       // 0/1 圆
}

// 几何校验：仅用于"提交回来的点阵"，不影响吸附
bool geom_check(const std::vector<Circle>& cs) {
    const int n = (int)cs.size();
    if (n < 2) return false;
    float far = 0.0f;
    for (int i = 0; i < n; ++i)
        for (int j = i + 1; j < n; ++j)
            far = std::max(far, std::hypot(cs[i].c.x - cs[j].c.x, cs[i].c.y - cs[j].c.y));
    if (n == 2) {
        float dx = std::fabs(cs[0].c.x - cs[1].c.x), dy = std::fabs(cs[0].c.y - cs[1].c.y);
        if (!(dx > DIAGONAL_THRESHOLD_PX && dy > DIAGONAL_THRESHOLD_PX)) return false;
        return far >= BOARD_DIAG_PX * 0.60f && far <= BOARD_DIAG_PX * 1.35f;
    }
    if (far < BOARD_DIAG_PX * 0.60f) return false;
    if (n >= 4) {
        std::vector<Circle> g(cs.begin(), cs.begin() + 4);
        std::sort(g.begin(), g.end(), [](const Circle& a, const Circle& b) { return a.c.y < b.c.y; });
        cv::Point2f TL = (g[0].c.x < g[1].c.x) ? g[0].c : g[1].c;
        cv::Point2f TR = (g[0].c.x < g[1].c.x) ? g[1].c : g[0].c;
        cv::Point2f BL = (g[2].c.x < g[3].c.x) ? g[2].c : g[3].c;
        cv::Point2f BR = (g[2].c.x < g[3].c.x) ? g[3].c : g[2].c;
        float dT = cv::norm(TL - TR), dB = cv::norm(BL - BR);
        float dL = cv::norm(TL - BL), dR = cv::norm(TR - BR);
        float d1 = cv::norm(TL - BR), d2 = cv::norm(TR - BL);
        float mx = std::max(std::max(dT, dB), std::max(dL, dR));
        if (mx < 1.0f) return false;
        if (std::fabs(dT - dB) / mx > 0.22f) return false;
        if (std::fabs(dL - dR) / mx > 0.22f) return false;
        if (std::fabs(d1 - d2) / mx > 0.22f) return false;
        float w = (dT + dB) * 0.5f, h = (dL + dR) * 0.5f;
        if (w > BOARD_W_PX * 1.20f || w < BOARD_W_PX * 0.78f) return false;
        if (h > BOARD_H_PX * 1.20f || h < BOARD_H_PX * 0.78f) return false;
        float side2 = w * w + h * h;
        if (std::fabs(d1 * d1 - side2) / side2 > 0.25f) return false;
    }
    return true;
}

// 60px 去重（与 hole_hybrid.cc 一致：半径大者优先）
void dedup(std::vector<Circle>& cs, float tol = 60.0f) {
    std::vector<Circle> uniq;
    for (auto& c : cs) {
        bool merged = false;
        for (auto& k : uniq) {
            if (std::hypot(c.c.x - k.c.x, c.c.y - k.c.y) < tol) {
                if (c.r > k.r) k = c;
                merged = true;
                break;
            }
        }
        if (!merged) uniq.push_back(c);
    }
    cs.swap(uniq);
}

// 候选 >4 时选"最像矩形"的 4 个（对齐误差 + 半径一致最小），与 hole_hybrid.cc 一致
void pick_rect4(std::vector<Circle>& cs) {
    const int n = (int)cs.size();
    if (n <= 4) return;
    int ai = 0, bi = 1, ci = 2, di = 3; float best = 1e18f;
    for (int a = 0; a < n; ++a) for (int b = a + 1; b < n; ++b)
    for (int c = b + 1; c < n; ++c) for (int d = c + 1; d < n; ++d) {
        std::vector<Circle> q = { cs[a], cs[b], cs[c], cs[d] };
        std::sort(q.begin(), q.end(), [](const Circle& A, const Circle& B) { return A.c.y < B.c.y; });
        const Circle& TL = q[0].c.x < q[1].c.x ? q[0] : q[1];
        const Circle& TR = q[0].c.x < q[1].c.x ? q[1] : q[0];
        const Circle& BL = q[2].c.x < q[3].c.x ? q[2] : q[3];
        const Circle& BR = q[2].c.x < q[3].c.x ? q[3] : q[2];
        float err = std::fabs(TL.c.x - BL.c.x) + std::fabs(TR.c.x - BR.c.x)
                  + std::fabs(TL.c.y - TR.c.y) + std::fabs(BL.c.y - BR.c.y)
                  + std::fabs(TL.r - TR.r) + std::fabs(TL.r - BL.r) + std::fabs(TL.r - BR.r);
        if (err < best) { best = err; ai = a; bi = b; ci = c; di = d; }
    }
    std::vector<Circle> g = { cs[ai], cs[bi], cs[ci], cs[di] };
    cs.swap(g);
}

} // namespace

bool merge_and_compute(const DepthDetectResult& auto_res,
                       const std::vector<ManualPoint>& pts,
                       float fx, float fy, float Z,
                       float ref_x, float ref_y,
                       cv::Point2f& out_center, float& out_dx, float& out_dy, bool& out_valid)
{
    out_valid = false;

    std::vector<Circle> cs;
    cs.reserve(auto_res.centers.size() + pts.size());
    for (size_t i = 0; i < auto_res.centers.size(); ++i)
        cs.push_back({ auto_res.centers[i], auto_res.radii.empty() ? 90.0f : auto_res.radii[i] });
    for (auto& p : pts)
        cs.push_back({ cv::Point2f((float)p.x, (float)p.y), (p.r > 0.0 ? (float)p.r : 90.0f) });

    dedup(cs);
    if (!geom_check(cs)) return false;   // 几何不符 → 判无效（主程序会记录/兜底）
    pick_rect4(cs);
    if (!geom_check(cs)) return false;

    cv::Point2f cen;
    if (!compute_center(cs, cen)) return false;   // <3 圆 或 2 相邻

    if (fx <= 0.0f || fy <= 0.0f) return false;

    out_center = cen;
    out_dx = (ref_x - cen.x) * Z / fx;
    out_dy = (ref_y - cen.y) * Z / fy;
    out_valid = true;
    return true;
}

std::string save_failure_frame(const cv::Mat& rgb_bgr, const std::string& dir) {
    if (rgb_bgr.empty()) return "";
    std::time_t t = std::time(nullptr);
    std::tm tm_now{};
#if defined(_WIN32)
    localtime_s(&tm_now, &t);
#else
    localtime_r(&t, &tm_now);   // 板子 Linux
#endif
    char name[96];
    std::snprintf(name, sizeof(name), "manual_%04d%02d%02d_%02d%02d%02d.png",
                  tm_now.tm_year + 1900, tm_now.tm_mon + 1, tm_now.tm_mday,
                  tm_now.tm_hour, tm_now.tm_min, tm_now.tm_sec);
    std::string path = dir;
    if (!path.empty() && (path.back() == '/' || path.back() == '\\')) path.pop_back();
    path += "/" + std::string(name);
    if (cv::imwrite(path, rgb_bgr)) return path;
    return "";
}

bool parse_annotation_json(const std::string& text, std::vector<ManualPoint>& pts, std::string& err) {
    pts.clear();
    err.clear();
    try {
        json j = json::parse(text);
        if (!j.contains("points") || !j["points"].is_array()) { err = "缺少 points 数组"; return false; }
        for (auto& p : j["points"]) {
            ManualPoint m;
            if (p.contains("x") && p["x"].is_number()) m.x = p["x"].get<double>();
            else { err = "point 缺少数值 x"; return false; }
            if (p.contains("y") && p["y"].is_number()) m.y = p["y"].get<double>();
            else { err = "point 缺少数值 y"; return false; }
            if (p.contains("r") && p["r"].is_number()) m.r = p["r"].get<double>();
            pts.push_back(m);
        }
        if (pts.empty()) { err = "points 为空"; return false; }
        return true;
    } catch (const std::exception& e) {
        err = e.what();
        return false;
    }
}

void prune_archive(const std::string& dir, size_t keep_n)
{
    namespace fs = std::filesystem;
    std::error_code ec;
    std::vector<std::pair<fs::file_time_type, fs::path>> files;
    for (const auto& entry : fs::directory_iterator(dir, ec))
    {
        if (ec) break;
        if (!entry.is_regular_file(ec)) continue;
        const fs::path& p = entry.path();
        std::string name = p.filename().string();
        if (name.rfind("manual_", 0) == 0 && p.extension() == ".png")
            files.emplace_back(fs::last_write_time(p, ec), p);
    }
    if (files.size() <= keep_n) return;
    std::sort(files.begin(), files.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
    // 按时间从旧到新，删除最旧的 (size - keep_n) 张
    for (size_t i = 0; i + keep_n < files.size(); ++i)
        fs::remove(files[i].second, ec);
}

} // namespace manual_annotation
