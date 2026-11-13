// test_pipeline.cc —— 端到端离线验证：真模型 + hole_track.h
// 用法: ./test_pipeline <model.rknn> <truth.json 或 "aimx,aimy"> <img1> [img2 ...]
// 按顺序对同一块 UpperPlateLock 喂图，模拟一次下降；打印每帧的选孔/剔除/状态/落点/跳变。
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <vector>
#include <cmath>
#include <string>

#include "yolov8_seg.h"
#include "image_utils.h"
#include "hole_track.h"
#include <opencv2/opencv.hpp>


// 旧逻辑（上线前）：本帧检出的孔按 y 升序取最上 4 个，取其对角线交点
static bool quadCenterOld(const std::vector<holetrack::Obs> &obs, float &ox, float &oy)
{
    if (obs.size() < 4) return false;
    std::vector<holetrack::Obs> s = obs;
    std::sort(s.begin(), s.end(), [](const holetrack::Obs &a, const holetrack::Obs &b) { return a.c.y < b.c.y; });
    const holetrack::Vec2 p[4] = { s[0].c, s[1].c, s[2].c, s[3].c };
    int pr[3][4] = { {0,1,2,3}, {0,2,1,3}, {0,3,1,2} };
    for (int k = 0; k < 3; ++k)
    {
        const holetrack::Vec2 &a = p[pr[k][0]], &b = p[pr[k][1]], &c = p[pr[k][2]], &d = p[pr[k][3]];
        float den = (b.x - a.x) * (d.y - c.y) - (b.y - a.y) * (d.x - c.x);
        if (std::fabs(den) < 1e-6f) continue;
        float u = ((c.x - a.x) * (d.y - c.y) - (c.y - a.y) * (d.x - c.x)) / den;
        float X = a.x + u * (b.x - a.x), Y = a.y + u * (b.y - a.y);
        float x0 = std::min(std::min(a.x,b.x),std::min(c.x,d.x)), x1 = std::max(std::max(a.x,b.x),std::max(c.x,d.x));
        float y0 = std::min(std::min(a.y,b.y),std::min(c.y,d.y)), y1 = std::max(std::max(a.y,b.y),std::max(c.y,d.y));
        if (X < x0 - 1 || X > x1 + 1 || Y < y0 - 1 || Y > y1 + 1) continue;
        ox = X; oy = Y; return true;
    }
    return false;
}

int main(int argc, char **argv)
{
    if (argc < 4) { printf("用法: %s <model> <aimx,aimy> <img1> [img2 ...]\n", argv[0]); return -1; }
    const char *model_path = argv[1];
    float aimx = 0, aimy = 0;
    sscanf(argv[2], "%f,%f", &aimx, &aimy);

    rknn_app_context_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    init_post_process();
    if (init_yolov8_seg_model(model_path, &ctx) != 0) { printf("init model fail\n"); return -1; }

    holetrack::UpperPlateLock lock;
    float prev_ax = 0, prev_ay = 0;
    bool  have_prev = false;

    printf("%-26s %4s %4s %4s %3s %-18s %8s %8s %8s | %8s %8s\n",
           "image", "hole", "k", "cull", "st", "aim", "err(px)", "jump", "scale", "旧Err", "旧jump");
    float oax = 0, oay = 0; bool o_have = false;
    for (int a = 3; a < argc; ++a)
    {
        cv::Mat orig = cv::imread(argv[a]);
        if (orig.empty()) { printf("%-26s imread fail\n", argv[a]); continue; }
        cv::Mat rgb;
        cv::cvtColor(orig, rgb, cv::COLOR_BGR2RGB);
        image_buffer_t src;
        memset(&src, 0, sizeof(src));
        src.width = rgb.cols; src.height = rgb.rows;
        src.format = IMAGE_FORMAT_RGB888;
        src.virt_addr = (unsigned char *)rgb.data;

        object_detect_result_list od;
        if (inference_yolov8_seg_model(&ctx, &src, &od) != 0) { printf("infer fail\n"); continue; }

        std::vector<holetrack::Obs> obs;
        for (int i = 0; i < od.count; ++i)
        {
            object_detect_result *r = &od.results[i];
            if (strcmp(coco_cls_to_name(r->cls_id), "hole") != 0) continue;
            holetrack::Obs o;
            o.c.x = (r->box.left + r->box.right) * 0.5f;
            o.c.y = (r->box.top + r->box.bottom) * 0.5f;
            o.r   = 0.5f * std::max(r->box.right - r->box.left, r->box.bottom - r->box.top);
            o.conf = r->prop;
            obs.push_back(o);
        }
        if (getenv("TP_VERBOSE"))
        {
            printf("  [%s] %dx%d 检出:\n", argv[a], rgb.cols, rgb.rows);
            for (int i = 0; i < od.count; ++i)
                printf("    %-6s (%d,%d)-(%d,%d) conf=%.3f\n", coco_cls_to_name(od.results[i].cls_id),
                       od.results[i].box.left, od.results[i].box.top,
                       od.results[i].box.right, od.results[i].box.bottom, od.results[i].prop);
        }
        holetrack::Result res = lock.update(obs, (float)rgb.cols, (float)rgb.rows);

        char aimbuf[40] = "-";
        float err = -1, jump = -1;
        if (res.state == 1 || res.state == 2)
        {
            snprintf(aimbuf, sizeof(aimbuf), "(%.1f,%.1f)", res.aim.x, res.aim.y);
            err = std::sqrt((res.aim.x - aimx) * (res.aim.x - aimx) + (res.aim.y - aimy) * (res.aim.y - aimy));
            if (have_prev)
                jump = std::sqrt((res.aim.x - prev_ax) * (res.aim.x - prev_ax) +
                                 (res.aim.y - prev_ay) * (res.aim.y - prev_ay));
            prev_ax = res.aim.x; prev_ay = res.aim.y; have_prev = true;
        }
        float oerr = -1, ojump = -1, ox = 0, oy = 0;
        if (quadCenterOld(obs, ox, oy))
        {
            oerr = std::sqrt((ox - aimx) * (ox - aimx) + (oy - aimy) * (oy - aimy));
            if (o_have) ojump = std::sqrt((ox - oax) * (ox - oax) + (oy - oay) * (oy - oay));
            oax = ox; oay = oy; o_have = true;
        }
        const char *fn = strrchr(argv[a], '/'); fn = fn ? fn + 1 : argv[a];
        printf("%-26s %4d %4d %4d %3d %-18s %8.2f %8.2f %8.2f | %8.2f %8.2f\n",
               fn, (int)obs.size(), res.n_used, res.n_cull, res.state, aimbuf, err, jump, res.scale, oerr, ojump);
        fflush(stdout);
    }
    release_yolov8_seg_model(&ctx);
    return 0;
}
