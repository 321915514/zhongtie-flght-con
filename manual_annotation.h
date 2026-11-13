// manual_annotation.h —— 四孔检测"人工回流标注"模块（板子端）
//
// 职责（与主程序/检测算法分离，独立可测）：
//   1. save_failure_frame     保存检测失败的帧（供 Android 遥控器人工标注）
//   2. parse_annotation_json  解析 Android 回传的标注 JSON
//   3. merge_and_compute      合并"自动检出圆心"+"人工补点"→ 最终中心 + dx/dy(mm)
//
// 依赖：OpenCV + hole_detect.h(DepthDetectResult)；JSON 用 nlohmann/json（项目已有）。
// 重要：中心算法(对角线交点/最远点对/对角中点)与 hole_hybrid.cc 内联逻辑保持一致，
//       若改动需两边同步（本文件内 compute_center 已标注"保持同步"）。
//
// 集成方式：本模块不与主程序耦合。主程序调用点（将来）：
//   detect_4holes 失败(valid=false 或 2 相邻) → save_failure_frame 存帧
//   → 等 Android 回传 JSON → parse_annotation_json → merge_and_compute → dx/dy

#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include "hole_detect.h"

namespace manual_annotation {

// 人工标注的一个点（原图像素坐标，1536x1280）
struct ManualPoint {
    double x = 0.0;   // 圆心像素 x
    double y = 0.0;   // 圆心像素 y
    double r = 0.0;   // 半径；<=0 表示未提供（合并时按默认 90px）
};

// 对角判定阈值（px）：两个圆心 |Δx| 与 |Δy| 都大于此值 → 视为对角，中点=中心
constexpr double DIAGONAL_THRESHOLD_PX = 150.0;

// 合并 + 计算中心与偏移
//   auto_res: detect_4holes 的结果（即使 valid=false，其 centers/radii 也含已检出的部分圆心，
//             依赖 hole_hybrid.cc 的"暴露部分圆心"改动）
//   pts:      人工标注点（Android 吸附后的圆心）
//   fx,fy:    内参；Z: 板面距离(mm)
//   ref_x,ref_y: 基准点(787,535)
// 输出：out_center(像素)、out_dx/out_dy(mm)、out_valid
// 返回：false 表示输入无效或中心无法确定（<3 圆 或 2 相邻）
bool merge_and_compute(const DepthDetectResult& auto_res,
                       const std::vector<ManualPoint>& pts,
                       float fx, float fy, float Z,
                       float ref_x, float ref_y,
                       cv::Point2f& out_center, float& out_dx, float& out_dy, bool& out_valid);

// 保存失败帧到 dir/manual_<时间戳>.png；成功返回完整路径，失败返回空串
std::string save_failure_frame(const cv::Mat& rgb_bgr, const std::string& dir);

// 清理归档帧：只保留最近 keep_n 张 manual_*.png（按修改时间），删除更早的
void prune_archive(const std::string& dir, size_t keep_n);

// 解析标注 JSON：
//   {"image_id":"...","points":[{"x":..,"y":..,"r":..}, ...]}
// 成功返回 true 并把点写入 pts；失败返回 false，err 填原因
bool parse_annotation_json(const std::string& text, std::vector<ManualPoint>& pts, std::string& err);

} // namespace manual_annotation
