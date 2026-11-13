// hole_track.h  ——  上板四孔身份锁定 + 相似变换求落点（2026-11-11）
//
// 背景：旧逻辑每帧把“当前检出的孔”按 y 升序取最上 4 个。上板（目标 2x2）漏检 1~2 个时，
//       下板的孔会顶上来，落点相对上板外心偏 61.5 / 93.3 / 115.3 px（1536x1280 实测）。
//
// 本模块把“选孔”从“每帧按几何排序”改成“首帧建立身份 + 每帧匹配”：
//   R1 建身份：首帧按两列 + 列内取最上 2 个，认定为上板四孔，记录其【归一化编号】model(uv)，
//              外心为原点、单位 = 上板平均孔距。不存像素坐标，所以画面放大后不用重认。
//   R2 每帧匹配：用上一帧姿态预测四个孔的位置，最近邻（窗口 0.45×预测孔距）确认身份，
//              下板孔永远不会顶替上板孔（对不上就空着）。
//   R3 缺孔降级：k>=2 解相似变换(s,theta,t)，aim=t（因为 model 原点=外心）；
//              k==1 沿用上帧 s/theta 只解平移；k==0 保持输出，连续 4 帧判无效并允许重锁。
//   R4 出画剔除：edge = min(x, W-x, y, H-y) < 半径 的孔丢弃（贴边孔框心被拉向画面内侧，
//              0.68~0.75 m 会出现几十 px 假偏移）。
//
// 依赖：仅 <vector>/<algorithm>/<cmath>，不依赖 opencv、spdlog，可单独编译单测。
#ifndef HOLE_TRACK_H_
#define HOLE_TRACK_H_

#include <algorithm>
#include <cmath>
#include <vector>

namespace holetrack {

struct Vec2
{
    float x;
    float y;
};

inline Vec2 mk(float x, float y) { Vec2 v; v.x = x; v.y = y; return v; }
inline Vec2 vsub(Vec2 a, Vec2 b) { return mk(a.x - b.x, a.y - b.y); }
inline Vec2 vadd(Vec2 a, Vec2 b) { return mk(a.x + b.x, a.y + b.y); }
inline Vec2 vmul(Vec2 a, float k) { return mk(a.x * k, a.y * k); }
inline float vnorm(Vec2 a) { return std::sqrt(a.x * a.x + a.y * a.y); }
inline Vec2 vrot(Vec2 a, float th)
{
    const float c = std::cos(th), s = std::sin(th);
    return mk(c * a.x - s * a.y, s * a.x + c * a.y);
}

// 一帧里的一个孔（像素）
struct Obs
{
    Vec2  c;            // 中心
    float r;            // 等效半径 = 0.5*max(w,h)
    float conf;
    Obs() : c(mk(0, 0)), r(0), conf(0) {}
};

// 一帧的求解结果
struct Result
{
    int   state;        // 0=未锁定 1=有效 2=保持(本帧 0 孔,<=3 帧) 3=无效(已解锁,下帧重锁)
    int   n_raw;        // 原始检出孔数
    int   n_cull;       // 贴边被剔除数
    int   n_used;       // 本帧确认身份的上板孔数
    int   init_kind;    // 1=多于 4 孔首帧锁定 2=仅 4 孔首帧锁定
    Vec2  aim;          // 上板四孔外心（像素）
    Vec2  used[4];      // 参与求解的上板孔像素位置
    float scale;        // 像素 / 模型单位（1 单位 = 上板平均孔距）
    float rot;          // 弧度，图像坐标系（y 向下）
    float res;          // 拟合残差 RMS（像素）
    bool  low_conf;     // 仅 4 孔锁定且落在画面下半部（有可能锁到下面那块板）
    Result()
        : state(0), n_raw(0), n_cull(0), n_used(0), init_kind(0),
          aim(mk(0, 0)), scale(0), rot(0), res(0), low_conf(false) {}
};

class UpperPlateLock
{
public:
    UpperPlateLock() { reset(); }

    void reset()
    {
        locked_ = false;
        miss_ = 0;
        s1_ = 0;
        s2_ = 0;
        rot_ = 0;
        aim_ = mk(0, 0);
        kind_ = 0;
        have_ever_ = false;
        for (int i = 0; i < 4; ++i) { model_[i] = mk(0, 0); prev_[i] = mk(0, 0); }
    }

    bool locked() const { return locked_; }

    Result update(const std::vector<Obs> &raw, float W, float H)
    {
        Result out;
        out.n_raw = (int)raw.size();

        // ---- R4 贴边剔除 ----
        std::vector<Obs> pool;
        pool.reserve(raw.size());
        for (size_t i = 0; i < raw.size(); ++i)
        {
            const Obs &o = raw[i];
            const float edge = std::min(std::min(o.c.x, W - o.c.x), std::min(o.c.y, H - o.c.y));
            if (edge < o.r * 1.05f + 1.0f) { out.n_cull++; continue; }
            pool.push_back(o);
        }

        // ---- R1 建身份 ----
        if (!locked_)
        {
            if (!tryLock(pool, out, H)) { out.state = 0; return out; }
        }

        // ---- R2 预测 ----
        float s_pred = s1_;
        if (s1_ > 1e-6f && s2_ > 1e-6f)
        {
            float ratio = s1_ / s2_;                       // 上帧相对上上帧的放大比（下降）
            ratio = std::max(0.65f, std::min(1.60f, ratio)); // 单帧物理上限，防跑飞
            s_pred = s1_ * ratio;
        }
        if (s_pred < 1e-3f) s_pred = 1.0f;

        Vec2 pred[4];
        for (int i = 0; i < 4; ++i) pred[i] = vadd(aim_, vmul(vrot(model_[i], rot_), s_pred));

        const float win = std::max(10.0f, 0.45f * s_pred);

        // ---- R2 最近邻匹配（贪心，每个检测孔只用一次）----
        bool used_obs[64] = {false};
        int  hit[4] = {-1, -1, -1, -1};
        for (int i = 0; i < 4; ++i)
        {
            float best = win;
            int bi = -1;
            const int n = (int)pool.size() < 64 ? (int)pool.size() : 64;
            for (int j = 0; j < n; ++j)
            {
                if (used_obs[j]) continue;
                const float d = vnorm(vsub(pool[j].c, pred[i]));
                if (d < best) { best = d; bi = j; }
            }
            if (bi >= 0) { hit[i] = bi; used_obs[bi] = true; }
        }

        Vec2 mp[4];
        Vec2 pp[4];
        int  k = 0;
        for (int i = 0; i < 4; ++i)
            if (hit[i] >= 0) { mp[k] = model_[i]; pp[k] = pool[hit[i]].c; ++k; }

        // ---- R3 求解 ----
        out.n_used = k;
        float s = s1_, th = rot_;
        Vec2 t = aim_;
        float res = 0;
        if (k >= 2)
        {
            if (!fitSim(mp, pp, k, s, th, t, res))
            {
                out.state = 2; out.aim = aim_; out.scale = s1_; out.rot = rot_;
                return out;
            }
            // 残差门限：k>=3 时若拟合偏差超过 0.30 孔距，判身份错位，本帧不发
            if (k >= 3 && res > 0.30f * s)
            {
                if (++miss_ >= 4) { locked_ = false; out.state = 3; }
                else              out.state = 2;
                out.aim = aim_; out.scale = s1_; out.rot = rot_;
                return out;
            }
            miss_ = 0;
        }
        else if (k == 1)
        {
            s = s_pred;
            th = rot_;
            t = vsub(pp[0], vmul(vrot(mp[0], th), s));
            res = 0;
            miss_ = 0;
        }
        else
        {
            miss_++;
            out.aim = aim_;
            out.scale = s1_;
            out.rot = rot_;
            out.n_used = 0;
            if (miss_ >= 4) { locked_ = false; out.state = 3; }
            else            out.state = 2;
            return out;
        }

        // ---- 提交状态 ----
        aim_ = t;
        s2_ = s1_;
        s1_ = s;
        rot_ = th;
        for (int i = 0; i < 4; ++i)
            prev_[i] = (hit[i] >= 0) ? pool[hit[i]].c
                                     : vadd(vadd(t, vmul(vrot(model_[i], th), s)), mk(0, 0));

        out.state = 1;
        out.aim = aim_;
        out.scale = s;
        out.rot = th;
        out.res = res;
        out.init_kind = kind_;
        {
            int w = 0;
            for (int i = 0; i < 4; ++i)
                if (hit[i] >= 0) out.used[w++] = pool[hit[i]].c;
        }
        return out;
    }

    // 上板四孔的“理想像素位置”（诊断/验证用，不论本帧是否检出）
    void projected(Vec2 out_px[4]) const
    {
        for (int i = 0; i < 4; ++i) out_px[i] = vadd(aim_, vmul(vrot(model_[i], rot_), s1_));
    }

private:
    bool  locked_;
    bool  have_ever_;   // 曾经锁定过（防止解锁后锁到下面那块板）
    int   miss_;
    int   kind_;
    float s1_, s2_;
    float rot_;
    Vec2  aim_;
    Vec2  model_[4];   // 归一化编号：外心=原点，单位=上板平均孔距
    Vec2  prev_[4];

    // 2D 相似变换最小二乘（Umeyama）：p = s*R(th)*m + t
    static bool fitSim(const Vec2 *m, const Vec2 *p, int n, float &s, float &th, Vec2 &t, float &res)
    {
        if (n < 2) return false;
        Vec2 cm = mk(0, 0), cp = mk(0, 0);
        for (int i = 0; i < n; ++i) { cm = vadd(cm, m[i]); cp = vadd(cp, p[i]); }
        cm = vmul(cm, 1.0f / n);
        cp = vmul(cp, 1.0f / n);
        float Sxx = 0, Sxy = 0, var = 0;
        for (int i = 0; i < n; ++i)
        {
            const Vec2 a = vsub(m[i], cm), b = vsub(p[i], cp);
            Sxx += a.x * b.x + a.y * b.y;
            Sxy += a.x * b.y - a.y * b.x;
            var += a.x * a.x + a.y * a.y;
        }
        if (var < 1e-9f) return false;
        s  = std::sqrt(Sxx * Sxx + Sxy * Sxy) / var;
        th = std::atan2(Sxy, Sxx);
        t  = vsub(cp, vmul(vrot(cm, th), s));
        float e = 0;
        for (int i = 0; i < n; ++i)
        {
            const Vec2 q = vadd(vmul(vrot(m[i], th), s), t);
            e += (q.x - p[i].x) * (q.x - p[i].x) + (q.y - p[i].y) * (q.y - p[i].y);
        }
        res = std::sqrt(e / n);
        return true;
    }

    // 对角线交点，要求交点落在四点的包围盒内
    static bool diagCenter(Vec2 a, Vec2 b, Vec2 c, Vec2 d, Vec2 &out)
    {
        const float x1 = a.x, y1 = a.y, x2 = b.x, y2 = b.y;
        const float x3 = c.x, y3 = c.y, x4 = d.x, y4 = d.y;
        // 直线 a-b 与 c-d 的交点参数 u（P = a + u*(b-a)），再要求 P 落在四点包围盒内
        const float den = (x2 - x1) * (y4 - y3) - (y2 - y1) * (x4 - x3);
        if (std::fabs(den) < 1e-6f) return false;
        const float u = ((x3 - x1) * (y4 - y3) - (y3 - y1) * (x4 - x3)) / den;
        const float X = x1 + u * (x2 - x1), Y = y1 + u * (y2 - y1);
        const float xmin = std::min(std::min(x1, x2), std::min(x3, x4));
        const float xmax = std::max(std::max(x1, x2), std::max(x3, x4));
        const float ymin = std::min(std::min(y1, y2), std::min(y3, y4));
        const float ymax = std::max(std::max(y1, y2), std::max(y3, y4));
        if (X < xmin - 1 || X > xmax + 1 || Y < ymin - 1 || Y > ymax + 1) return false;
        out = mk(X, Y);
        return true;
    }

    // 四孔构成的“2x2 矩形”几何校验（全部用比值，与分辨率/高度无关）
    static bool checkQuad(const Vec2 q[4], Vec2 &aim0, float &pitch)
    {
        const float rowgap = 0.5f * ((q[2].y + q[3].y) - (q[0].y + q[1].y)); // 上下两行间距
        const float colgap = 0.5f * ((q[1].x + q[3].x) - (q[0].x + q[2].x)); // 左右两列间距
        if (rowgap <= 1.0f || colgap <= 1.0f) return false;
        if (std::fabs(q[0].y - q[1].y) > 0.45f * rowgap) return false; // 同一行两点等高
        if (std::fabs(q[2].y - q[3].y) > 0.45f * rowgap) return false;
        if (std::fabs(q[0].x - q[2].x) > 0.45f * colgap) return false; // 同一列两点同竖
        if (std::fabs(q[1].x - q[3].x) > 0.45f * colgap) return false;
        if (rowgap * colgap < 400.0f) return false;                    // 太小不可信
        // 行距/列距比：真 2x2 板接近 1（实测 1.02）；上板缺一排、拿下面那块板的行来凑时
        // 比值变成 1.46（跨块行距 134 vs 块内 96）或 2.45（跨两块），一律拒绝冷启动。
        // 若现场板子的行距/列距确实差很多，改这两个常数即可。
        const float aspect = rowgap / colgap;
        // 2026-11-11 修复：原门限 [0.68,1.32] 假设孔距近似 1:1；实际板子行距/列距≈0.62（长方形排布），
        // checkQuad 恒失败 -> tryLock 失败 -> state=0 -> n_used=0 -> dx/dy 恒 0。放宽到 [0.45,2.2] 覆盖长方形板。
        if (aspect < 0.45f || aspect > 2.2f) return false;
        Vec2 c;
        if (!diagCenter(q[0], q[3], q[1], q[2], c)) return false;      // 对角线交点
        aim0 = c;
        pitch = 0.25f * (vnorm(vsub(q[1], q[0])) + vnorm(vsub(q[3], q[2])) +
                         vnorm(vsub(q[2], q[0])) + vnorm(vsub(q[3], q[1])));
        return pitch > 1.0f;
    }

    // 首帧锁定：两列 × 列内最上 2 个 = 上板四孔
    bool tryLock(const std::vector<Obs> &pool, Result &out, float H)
    {
        if (pool.size() < 4) return false;

        float rmean = 0;
        for (size_t i = 0; i < pool.size(); ++i) rmean += pool[i].r;
        rmean /= (float)pool.size();
        // 行内孔的 y 差远小于行间距（实测 行距/孔半径 ≈ 4.8），0.8r 足够分干净
        const float row_tol = std::max(4.0f, 0.8f * rmean);

        std::vector<Obs> byy = pool;
        std::sort(byy.begin(), byy.end(), [](const Obs &a, const Obs &b) { return a.c.y < b.c.y; });

        std::vector<std::vector<Obs> > rows;
        for (size_t i = 0; i < byy.size(); ++i)
        {
            if (rows.empty() || byy[i].c.y - rows.back().back().c.y > row_tol)
                rows.push_back(std::vector<Obs>());
            rows.back().push_back(byy[i]);
        }
        // 只认最上面两行（= 上板）；第二行不足 2 个就不锁定，绝不往下找
        if (rows.size() < 2) return false;
        if (rows[0].size() < 2 || rows[1].size() < 2) return false;

        std::vector<Obs> La = rows[0], Lb = rows[1];
        std::sort(La.begin(), La.end(), [](const Obs &a, const Obs &b) { return a.c.x < b.c.x; });
        std::sort(Lb.begin(), Lb.end(), [](const Obs &a, const Obs &b) { return a.c.x < b.c.x; });

        Vec2 q[4]; // 0=左上 1=右上 2=左下 3=右下
        q[0] = La.front().c; q[1] = La.back().c;
        q[2] = Lb.front().c; q[3] = Lb.back().c;

        Vec2  aim0;
        float pitch = 0;
        if (!checkQuad(q, aim0, pitch)) return false;

        // 重锁连续性门限：距上一次有效落点超过 1.2 个孔距就不认（下面那块板在 2.4 孔距外），
        // 宁可不发也不发一个偏一整块板的值
        if (have_ever_ && vnorm(vsub(aim0, aim_)) > 1.2f * s1_) return false;

        for (int i = 0; i < 4; ++i)
        {
            model_[i] = vmul(vsub(q[i], aim0), 1.0f / pitch);
            prev_[i]  = q[i];
        }
        aim_    = aim0;
        s1_     = pitch;
        s2_     = pitch;
        rot_    = 0;
        locked_ = true;
        miss_   = 0;
        have_ever_ = true;
        kind_   = (pool.size() >= 6) ? 1 : 2;

        out.state     = 1;
        out.n_used    = 4;
        out.aim       = aim0;
        out.scale     = pitch;
        out.rot       = 0;
        out.res       = 0;
        out.init_kind = kind_;
        for (int i = 0; i < 4; ++i) out.used[i] = q[i];
        // 仅 4 孔锁定且落在画面下半部：可能锁到下面那块板，标低置信
        out.low_conf = (kind_ == 2) && (aim0.y > 0.55f * H);
        return true;
    }
};

} // namespace holetrack

#endif // HOLE_TRACK_H_
