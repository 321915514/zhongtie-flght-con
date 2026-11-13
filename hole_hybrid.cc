// hole_hybrid.cc —— 混合检测：完整圆走外接矩形(快)，碎块/遮挡走 RANSAC(准)
#include "hole_detect.h"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <random>

void rgb8_planar_to_bgr(const unsigned char* src, int w, int h, unsigned char* dst) {
    int step = w * h;
    for (int i = 0; i < step; ++i) {
        dst[i * 3 + 0] = src[i + step * 2];
        dst[i * 3 + 1] = src[i + step * 1];
        dst[i * 3 + 2] = src[i + step * 0];
    }
}

static bool circle_from_3(cv::Point2f a, cv::Point2f b, cv::Point2f c, cv::Point2f& center, float& r) {
    double x1=a.x,y1=a.y,x2=b.x,y2=b.y,x3=c.x,y3=c.y;
    double A=x2-x1,B=y2-y1,C=x3-x1,D=y3-y1;
    double e=A*(x1+x2)+B*(y1+y2), f=C*(x1+x3)+D*(y1+y3);
    double det=2*(A*(y3-y2)-B*(x3-x2));
    if (std::abs(det)<1e-9) return false;
    center.x=(float)((D*e-B*f)/det); center.y=(float)((A*f-C*e)/det);
    r=(float)std::hypot(x1-center.x,y1-center.y);
    return true;
}

static bool kasa_fit(const std::vector<cv::Point2f>& p, double& cx, double& cy, double& r) {
    int n=(int)p.size(); if (n<3) return false;
    double Sx=0,Sy=0,Sx2=0,Sy2=0,Sxy=0,Sx3=0,Sy3=0,Sxy2=0,Sx2y=0;
    for (auto& q:p){double x=q.x,y=q.y,x2=x*x,y2=y*y;
        Sx+=x;Sy+=y;Sx2+=x2;Sy2+=y2;Sxy+=x*y;Sx3+=x*x2;Sy3+=y*y2;Sxy2+=x*y2;Sx2y+=x2*y;}
    double A[3][3]={{Sx2,Sxy,Sx},{Sxy,Sy2,Sy},{Sx,Sy,(double)n}};
    double b[3]={-(Sx3+Sxy2),-(Sx2y+Sy3),-(Sx2+Sy2)};
    for(int i=0;i<3;++i){
        int piv=i; for(int k=i+1;k<3;++k) if(std::abs(A[k][i])>std::abs(A[piv][i])) piv=k;
        if(std::abs(A[piv][i])<1e-12) return false;
        if(piv!=i){std::swap(A[piv],A[i]);std::swap(b[piv],b[i]);}
        for(int k=i+1;k<3;++k){double m=A[k][i]/A[i][i];
            for(int j=i;j<3;++j) A[k][j]-=m*A[i][j]; b[k]-=m*b[i];}
    }
    double sol[3];
    for(int i=2;i>=0;--i){double s=b[i];for(int j=i+1;j<3;++j)s-=A[i][j]*sol[j];sol[i]=s/A[i][i];}
    cx=-sol[0]/2.0; cy=-sol[1]/2.0; r=std::sqrt(std::max(0.0,cx*cx+cy*cy-sol[2]));
    return true;
}

static bool ransac_fit(const std::vector<cv::Point2f>& pts, float tol, int iters,
                       float rmin, float rmax, cv::Point2f& center, float& r,
                       int& nin, float& coverage) {
    int N=(int)pts.size(); if(N<15) return false;
    std::mt19937 rng(12345); std::uniform_int_distribution<int> dist(0,N-1);
    int best=-1; cv::Point2f bc; float br=0;
    for(int it=0;it<iters;++it){
        int i0=dist(rng),i1=dist(rng),i2=dist(rng);
        if(i0==i1||i0==i2||i1==i2) continue;
        cv::Point2f c; float rr;
        if(!circle_from_3(pts[i0],pts[i1],pts[i2],c,rr)) continue;
        if(rr<rmin||rr>rmax) continue;
        int cnt=0;
        for(auto&q:pts) if(std::abs((float)std::hypot(q.x-c.x,q.y-c.y)-rr)<tol) ++cnt;
        if(cnt>best){best=cnt;bc=c;br=rr;}
    }
    if(best<20) return false;
    std::vector<cv::Point2f> inl;
    for(auto&q:pts) if(std::abs((float)std::hypot(q.x-bc.x,q.y-bc.y)-br)<tol) inl.push_back(q);
    double cx,cy,rr; if(!kasa_fit(inl,cx,cy,rr)) return false;
    center=cv::Point2f((float)cx,(float)cy); r=(float)rr;
    std::vector<float> ang;
    for(auto&q:pts) if(std::abs((float)std::hypot(q.x-center.x,q.y-center.y)-r)<tol)
        ang.push_back(std::atan2(q.y-center.y,q.x-center.x));
    nin=(int)ang.size(); if(ang.size()<5){coverage=0;return false;}
    std::sort(ang.begin(),ang.end());
    float mg=0; for(size_t i=1;i<ang.size();++i) mg=std::max(mg,ang[i]-ang[i-1]);
    mg=std::max(mg,(float)(ang[0]+2*M_PI-ang.back()));
    coverage=1.0f-mg/(float)(2*M_PI);
    return true;
}

DepthDetectResult detect_4holes(const cv::Mat& rgb_bgr, const cv::Mat& depth16,
                                float fx, float fy, int img_w, int img_h, float cov_thresh) {
    DepthDetectResult out;
    if (rgb_bgr.empty() || depth16.empty()) return out;
    cv::Mat gray; cv::cvtColor(rgb_bgr, gray, cv::COLOR_BGR2GRAY);
    cv::Mat g; cv::GaussianBlur(gray, g, cv::Size(5,5), 0);

    cv::Mat binv;
    cv::threshold(g, binv, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
    cv::Mat merged;
    cv::morphologyEx(binv, merged, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(41,41)));
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(merged, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

    // Canny 边缘点：懒加载（只在有碎块需要 RANSAC 时才计算）
    std::vector<cv::Point2f> ep;
    bool edges_ready = false;
    auto ensure_edges = [&]() {
        if (edges_ready) return;
        cv::Mat edges; cv::Canny(g, edges, 40, 120);
        for (int r=0;r<edges.rows;++r) for(int c=0;c<edges.cols;++c)
            if (edges.at<uchar>(r,c)) ep.push_back(cv::Point2f((float)c,(float)r));
        edges_ready = true;
    };

    struct C { cv::Point2f c; float r; float cov; int nin; int clean; };
    std::vector<C> cands;
    out.n_bbox = 0; out.n_ransac = 0; out.n_depth = 0;

    for (auto& ct : contours) {
        double a = cv::contourArea(ct);
        if (a < M_PI*50*50 || a > M_PI*130*130) continue;   // 孔半径随距离变(实测~73)
        double p = cv::arcLength(ct, true);
        double circ = (p>1e-9) ? 4*M_PI*a/(p*p) : 0.0;
        cv::Rect bb = cv::boundingRect(ct);
        float ar = bb.height>0 ? (float)bb.width/bb.height : 0.f;

        if (circ >= 0.80 && ar >= 0.8f && ar <= 1.28f) {
            C c; c.c = cv::Point2f(bb.x+bb.width/2.0f, bb.y+bb.height/2.0f);
            c.r = (bb.width+bb.height)/4.0f; c.cov = 1.0f; c.nin = 999; c.clean = 1;
            cands.push_back(c); out.n_bbox++;
        } else {
            ensure_edges();   // 懒加载边缘点
            cv::Point2f ctr; float rad; cv::minEnclosingCircle(ct, ctr, rad);
            std::vector<cv::Point2f> sel;
            for (auto& q : ep) {
                float d = (float)std::hypot(q.x-ctr.x, q.y-ctr.y);
                if (d > rad*0.75f && d < rad*1.35f) sel.push_back(q);
            }
            if (sel.size() < 20) continue;
            cv::Point2f c2; float r2; int nin; float cov;
            if (ransac_fit(sel, 2.5f, 1200, 55.0f, 125.0f, c2, r2, nin, cov)) {   // 收紧半径到65~115
                C c; c.c=c2; c.r=r2; c.cov=cov; c.nin=nin; c.clean=0;
                cands.push_back(c); out.n_ransac++;
            }
        }
    }

    // Hough 圆补充：应对阴影/低对比度导致阈值漏检的孔（如右上孔）
    {
        std::vector<cv::Vec3f> hough;
        cv::HoughCircles(g, hough, cv::HOUGH_GRADIENT, 1.5, 180, 60, 40, 60, 130);
        for (auto& h : hough) {
            float hx=h[0], hy=h[1], hr=h[2];
            if (hr < 55.f || hr > 115.f) continue;   // 孔半径随距离变(实测~73)，放宽
            C c; c.c = cv::Point2f(hx,hy); c.r = hr; c.cov = 0.90f; c.nin = 999; c.clean = 2;
            cands.push_back(c);
        }
    }
    // ===== 黑帽变换补充 =====
    // 提取"亮背景上的暗斑"，专治暗背景/大块阴影场景。
    // 全局 Otsu 在暗背景图上会把孔和背景混为一类（实测现场图 30.8% 是暗背景），
    // 黑帽按局部对比度提取，比核更大的暗区天然被排除。
    // 降采样到 1/2 再算：形态学对核尺寸是平方级开销，实测 870ms → 90ms，检出不变。
    {
        const int BH_SCALE = 2;
        cv::Mat hg;
        cv::resize(g, hg, cv::Size(), 1.0 / BH_SCALE, 1.0 / BH_SCALE, cv::INTER_AREA);
        const int ks_list[] = {61, 101};   // 1/2 尺度下覆盖孔直径 70~90px（原图 140~180）
        for (int ki = 0; ki < 2; ++ki) {
            cv::Mat bh;
            cv::morphologyEx(hg, bh, cv::MORPH_BLACKHAT,
                             cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                       cv::Size(ks_list[ki], ks_list[ki])));
            cv::Mat bt;
            cv::threshold(bh, bt, 40, 255, cv::THRESH_BINARY);
            cv::morphologyEx(bt, bt, cv::MORPH_OPEN,
                             cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
            std::vector<std::vector<cv::Point>> bcont;
            cv::findContours(bt, bcont, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
            for (auto& ct : bcont) {
                double a = cv::contourArea(ct);
                if (a < M_PI * 22 * 22 || a > M_PI * 68 * 68) continue;   // 1/2 尺度（原 45~135）
                double pp = cv::arcLength(ct, true);
                double circ = pp > 1e-9 ? 4 * M_PI * a / (pp * pp) : 0.0;
                if (circ < 0.55) continue;
                cv::Point2f ctr; float rad;
                cv::minEnclosingCircle(ct, ctr, rad);
                if (rad < 25.f || rad > 68.f) continue;                  // 1/2 尺度（原 50~136）
                C c;
                c.c = cv::Point2f(ctr.x * BH_SCALE, ctr.y * BH_SCALE);
                c.r = rad * BH_SCALE; c.cov = 0.85f; c.nin = 999; c.clean = 1;
                cands.push_back(c); out.n_bbox++;
            }
        }
    }

    // 深度图二值化 + 分割圆（补充）：孔在深度上表现为比板面更深或无返回值
    // 注意：深度为原生深度系(1408×1024)，与RGB有基线视差(约52px)；这里 resize 到RGB尺寸近似对齐
    cv::Mat dres;
    cv::resize(depth16, dres, cv::Size(img_w, img_h), 0, 0, cv::INTER_NEAREST);
    float plateZ = 0.f;
    const bool rgb_only = std::getenv("HOLE_RGB_ONLY") != nullptr;   // 调试:只跑RGB路径
    if (!rgb_only) {
        std::vector<uint16_t> dv; dv.reserve(dres.rows*dres.cols/8);
        for(int r=0;r<dres.rows;r+=2){const uint16_t* row=dres.ptr<uint16_t>(r);
            for(int c=0;c<dres.cols;c+=2) if(row[c]>0) dv.push_back(row[c]);}
        if(!dv.empty()){
            std::sort(dv.begin(),dv.end());
            plateZ=(float)dv[dv.size()/2];   // 板面深度中位数(≈731)
            cv::Mat dmask(dres.size(),CV_8UC1,cv::Scalar(0));
            for(int r=0;r<dres.rows;++r){const uint16_t* row=dres.ptr<uint16_t>(r); uchar* m=dmask.ptr<uchar>(r);
                for(int c=0;c<dres.cols;++c){uint16_t v=row[c];
                    if(v==0 || (float)v>plateZ+10.0f) m[c]=255;}}   // 无效 或 比板面深>10mm(=孔)
            cv::morphologyEx(dmask,dmask,cv::MORPH_CLOSE,cv::getStructuringElement(cv::MORPH_RECT,cv::Size(15,15)));
            std::vector<std::vector<cv::Point>> dcont;
            cv::findContours(dmask,dcont,cv::RETR_LIST,cv::CHAIN_APPROX_NONE);
            for(auto& ct:dcont){
                double a=cv::contourArea(ct);
                if(a<3000.0 || a>60000.0) continue;              // 孔面积(深度resize后约6k~12k)
                double p=cv::arcLength(ct,true);
                double circ=p>1e-9?4*M_PI*a/(p*p):0.0;
                cv::Rect bb=cv::boundingRect(ct);
                float ar=bb.height>0?(float)bb.width/bb.height:0.f;
                if(circ<0.12f || ar<0.6f || ar>1.6f) continue;   // 深度孔边缘毛糙，圆度放宽到0.12
                cv::Point2f ctr; float rad; cv::minEnclosingCircle(ct,ctr,rad);
                if(rad<45.f||rad>125.f) continue;
                C c; c.c=ctr; c.r=rad; c.cov=0.80f; c.nin=999; c.clean=3;   // 3=深度分割
                cands.push_back(c); out.n_depth++;
            }
        }
    }
    // 深度确认：剔除圆心处不深(表面脏点/纹理/误检)的候选，只保留真孔
    if (!rgb_only && plateZ > 0.f) {
        std::vector<C> confirmed; confirmed.reserve(cands.size());
        for (auto& c : cands) {
            int cx=(int)c.c.x, cy=(int)c.c.y;
            bool deep=false;
            for (int dy=-35; dy<=35 && !deep; dy+=7)
                for (int dx=-35; dx<=35 && !deep; dx+=7) {
                    int x=cx+dx, y=cy+dy;
                    if (x<0||y<0||x>=dres.cols||y>=dres.rows) continue;
                    if ((float)dres.at<uint16_t>(y,x) > plateZ+10.0f) deep=true;
                }
            if (deep) confirmed.push_back(c);
        }
        cands = confirmed;
    }
    if (cands.empty()) return out;

    std::vector<C> uniq;
    for (auto& c : cands) {
        bool m = false;
        for (auto& k : uniq) {
            if (std::hypot(c.c.x-k.c.x, c.c.y-k.c.y) < 60) {
                // 可靠性优先：depth(3) > hough(2) > bbox(1) > ransac(0)；同级比覆盖度
                bool take = (c.clean > k.clean) || (c.clean == k.clean && c.cov > k.cov);
                if (take) k = c;
                m = true; break;
            }
        }
        if (!m) uniq.push_back(c);
    }

    std::vector<C> good;
    for (auto& c : uniq) if (c.cov >= cov_thresh) good.push_back(c);

    // ===== [遮挡补齐] =====
    // 约束（现场确认）：孔半径随距离变（用实测）、4 孔必成矩形。
    if ((good.size() == 2 || good.size() == 3) && plateZ > 0.f && !dres.empty()) {
        const float R = (good[0].r + good[1].r) * 0.5f;
        auto isDeep = [&](int x, int y) {
            return (x >= 0 && y >= 0 && x < dres.cols && y < dres.rows &&
                    (float)dres.at<uint16_t>(y, x) > plateZ + 10.0f);
        };
        auto deepWithin = [&](cv::Point2f pt, float rad) {
            int cnt = 0;
            for (int y = (int)(pt.y - rad); y <= (int)(pt.y + rad); y += 4)
                for (int x = (int)(pt.x - rad); x <= (int)(pt.x + rad); x += 4) {
                    if (x < 0 || y < 0 || x >= dres.cols || y >= dres.rows) continue;
                    float ex = x - pt.x, ey = y - pt.y;
                    if (ex * ex + ey * ey > rad * rad) continue;
                    if (isDeep(x, y)) { if (++cnt >= 40) return true; }
                }
            return false;
        };
        auto tryAdd = [&](float x, float y) {
            C c; c.c = cv::Point2f(x, y); c.r = R; c.cov = 0.80f; c.nin = 999; c.clean = 3;
            for (auto& g : good) if (std::hypot(c.c.x - g.c.x, c.c.y - g.c.y) < R * 0.8f) return;
            if (good.size() < 4) good.push_back(c);
        };
        // (1) 已检出 3 孔 → 平行四边形（对角线中点相等）推第 4 个角，深度确认后补上
        if (good.size() == 3) {
            int di = 0, dj = 1, dk = 2; float best = -1.f;
            for (int i = 0; i < 3; ++i) for (int j = i + 1; j < 3; ++j) {
                float d = std::hypot(good[i].c.x - good[j].c.x, good[i].c.y - good[j].c.y);
                if (d > best) { best = d; di = i; dj = j; dk = 3 - i - j; }
            }
            cv::Point2f p4 = good[di].c + good[dj].c - good[dk].c;
            if (deepWithin(p4, R * 1.3f)) tryAdd(p4.x, p4.y);
        }
        // (2) 只检出同一排/列的 2 孔 → 沿另一维扫深度找另一排（不假设长宽比）
        if (good.size() == 2) {
            const cv::Point2f A = good[0].c, Bp = good[1].c;
            const bool sameRow = std::fabs(A.y - Bp.y) < R * 1.6f && std::fabs(A.x - Bp.x) > R * 2.f;
            const bool sameCol = std::fabs(A.x - Bp.x) < R * 1.6f && std::fabs(A.y - Bp.y) > R * 2.f;
            auto scanDeep = [&](float fixedCoord, float excludeCenter, bool vertical) -> std::vector<float> {
                std::vector<float> res;
                const int N = vertical ? dres.rows : dres.cols;
                std::vector<char> deep(N, 0);
                for (int i = 0; i < N; ++i) {
                    int cnt = 0, tot = 0;
                    for (int o = -15; o <= 15; o += 3) {
                        int xx = vertical ? (int)fixedCoord + o : i;
                        int yy = vertical ? i : (int)fixedCoord + o;
                        if (xx < 0 || yy < 0 || xx >= dres.cols || yy >= dres.rows) continue;
                        ++tot; if (isDeep(xx, yy)) ++cnt;
                    }
                    deep[i] = (tot > 0 && cnt * 2 >= tot) ? 1 : 0;
                }
                auto widthAt = [&](int k) {
                    int c2 = 0;
                    for (int o = -(int)R; o <= (int)R; ++o) {
                        int xx = vertical ? (int)fixedCoord + o : k;
                        int yy = vertical ? k : (int)fixedCoord + o;
                        if (isDeep(xx, yy)) ++c2;
                    }
                    return c2;
                };
                int i = 0;
                while (i < N) {
                    if (!deep[i]) { ++i; continue; }
                    int j = i; while (j < N && deep[j]) ++j;
                    int ya = i, yb = j - 1;
                    float mid = (ya + yb) * 0.5f;
                    if (std::fabs(mid - excludeCenter) > R * 1.7f && (yb - ya) >= (int)(R * 0.25f)) {
                        if (yb - ya >= (int)(R * 1.6f)) res.push_back(mid);
                        else res.push_back(widthAt(ya + 3) >= widthAt(yb - 3) ? (yb - R) : (ya + R));
                    }
                    i = j;
                }
                return res;
            };
            if (sameRow) {
                for (int qi = 0; qi < 2; ++qi)
                    for (float y : scanDeep(good[qi].c.x, good[qi].c.y, true)) tryAdd(good[qi].c.x, y);
            } else if (sameCol) {
                for (int qi = 0; qi < 2; ++qi)
                    for (float x : scanDeep(good[qi].c.y, good[qi].c.x, false)) tryAdd(x, good[qi].c.y);
            }
        }
    }

    if (good.size() > 4) {
        // 选最像矩形的 4 个（对齐误差最小 + 半径一致），排除 Hough 误检
        int n=(int)good.size(), ai=0,bi=1,ci=2,di=3; float best=1e18f;
        for(int a=0;a<n;++a)for(int b=a+1;b<n;++b)for(int c2=b+1;c2<n;++c2)for(int d=c2+1;d<n;++d){
            std::vector<C> q={good[a],good[b],good[c2],good[d]};
            std::sort(q.begin(),q.end(),[](const C&A,const C&B){return A.c.y<B.c.y;});
            const C&TL=q[0].c.x<q[1].c.x?q[0]:q[1]; const C&TR=q[0].c.x<q[1].c.x?q[1]:q[0];
            const C&BL=q[2].c.x<q[3].c.x?q[2]:q[3]; const C&BR=q[2].c.x<q[3].c.x?q[3]:q[2];
            float err=std::fabs(TL.c.x-BL.c.x)+std::fabs(TR.c.x-BR.c.x)+std::fabs(TL.c.y-TR.c.y)+std::fabs(BL.c.y-BR.c.y)
                     +std::fabs(TL.r-TR.r)+std::fabs(TL.r-BL.r)+std::fabs(TL.r-BR.r);
            if(err<best){best=err;ai=a;bi=b;ci=c2;di=d;}
        }
        std::vector<C> g2={good[ai],good[bi],good[ci],good[di]};
        good=g2;
    }
    // [人工回流] 无论孔数多少，先把已检出的圆心暴露给调用方（manual_annotation 合并补点用）
    for (auto& c : good) { out.centers.push_back(c.c); out.radii.push_back(c.r); out.methods.push_back(c.clean); }
    out.n_holes = (int)good.size();

    // 深度中位数 Z：即使孔不足也先算出来，供人工合并换算 mm 用
    std::vector<uint16_t> vals; vals.reserve(depth16.rows*depth16.cols/8);
    for(int r=0;r<depth16.rows;r+=2){const uint16_t* row=depth16.ptr<uint16_t>(r);
        for(int c=0;c<depth16.cols;c+=2) if(row[c]>0) vals.push_back(row[c]);}
    if(!vals.empty()){ std::sort(vals.begin(),vals.end()); out.z=(float)vals[vals.size()/2]; }

    if (good.size() < 3) return out;   // 至少 3 个圆心才能自动定中心

    // 中心 = 四个圆心矩形的两条对角线交点；3 个圆时取最远两点(对角线)中点
    cv::Point2f cen;
    if (good.size() >= 4) {
        std::vector<C> g(good.begin(), good.begin()+4);
        std::sort(g.begin(), g.end(), [](const C&a,const C&b){return a.c.y < b.c.y;});
        cv::Point2f TL = (g[0].c.x < g[1].c.x) ? g[0].c : g[1].c;
        cv::Point2f TR = (g[0].c.x < g[1].c.x) ? g[1].c : g[0].c;
        cv::Point2f BL = (g[2].c.x < g[3].c.x) ? g[2].c : g[3].c;
        cv::Point2f BR = (g[2].c.x < g[3].c.x) ? g[3].c : g[2].c;
        float d = (BR.x-TL.x)*(BL.y-TR.y) - (BR.y-TL.y)*(BL.x-TR.x);
        if (std::fabs(d) < 1e-6f) cen = (TL+TR+BL+BR)*0.25f;   // 退化：平行则用平均
        else {
            float num = (TR.x-TL.x)*(BL.y-TR.y) - (TR.y-TL.y)*(BL.x-TR.x);
            cen = TL + (num/d)*(BR-TL);
        }
    } else {
        float best=-1; int bi=0, bj=1;
        for (size_t i=0;i<good.size();++i)
            for (size_t j=i+1;j<good.size();++j) {
                float ddx=good[i].c.x-good[j].c.x, ddy=good[i].c.y-good[j].c.y;
                float dd=ddx*ddx+ddy*ddy;
                if (dd>best){best=dd;bi=(int)i;bj=(int)j;}
            }
        cen = (good[bi].c + good[bj].c)*0.5f;
    }
    float mcov=1e9f;
    for (auto& c:good) mcov=std::min(mcov,c.cov);
    out.min_cov=mcov; out.center_px=cen;

    out.dx=(786.0f-cen.x)*out.z/fx;   // 零位基准(786.0,560.0) 零位_20261106_1203 八帧对角线交点均值
    out.dy=(560.0f-cen.y)*out.z/fy;   // 零位基准(786.0,560.0) 零位_20261106_1203 八帧对角线交点均值
    out.valid=true;
    return out;
}
