// hole_detect.h —— 位置无关的四孔检测（RANSAC 圆拟合，不依赖固定参考位）
#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

struct DepthDetectResult {
    bool valid = false;
    float dx = 0.0f;      // mm，横向偏移（符号已取反：>0 质心偏左）
    float dy = 0.0f;      // mm，纵向偏移（符号已取反：>0 质心偏上）
    float z = 0.0f;       // mm，质心处深度
    float min_cov = 0.0f; // 四孔最小角度覆盖率（可信度，<0.6 建议判无效）
    int n_holes = 0;
    int n_bbox = 0;       // 走外接矩形(快路径)的孔数
    int n_ransac = 0;     // 走 RANSAC(遮挡路径)的孔数
    int n_depth = 0;      // 走深度二值化分割的孔数
    cv::Point2f center_px;   // 四个圆心的中心（圆心坐标平均，非面积质心）
    std::vector<cv::Point2f> centers;  // 每个孔的圆心（像素），用于标注
    std::vector<float> radii;          // 每个孔的半径
    std::vector<int> methods;          // 1=外接矩形, 0=RANSAC, 2=Hough, 3=深度分割
};

void rgb8_planar_to_bgr(const unsigned char* src, int w, int h, unsigned char* dst);

// 位置无关检测：全图找 4 个暗圆（RANSAC），算质心 + dx/dy(mm)
// 输入 rgb_bgr(1536x1280)、depth16(1408x1024)、fx/fy、图像宽高
DepthDetectResult detect_4holes(const cv::Mat& rgb_bgr, const cv::Mat& depth16,
                                float fx, float fy, int img_w, int img_h,
                                float cov_thresh = 0.55f);
