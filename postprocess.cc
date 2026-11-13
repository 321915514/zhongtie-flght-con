// Copyright (c) 2021 by Rockchip Electronics Co., Ltd. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// YOLOv8-OBB 后处理（由 yolov8-seg 后处理改造而来）。
// 输出：3 个检测头特征图（P3/P4/P5，各 [1, 4*reg_max + nc, h, w]）+ 1 个角度特征（outputs[3]）。
// 每个目标输出 AABB（box）+ 旋转角（angle）+ 4 角点（corners），供上层绘制旋转框 / 计算中心点。

#include "yolov8_seg.h"
#include "postprocess.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <float.h>

#include <algorithm>
#include <set>
#include <vector>

#define LABEL_NALE_TXT_PATH "/home/cat/Flght_con/flght_control/model/coco_80_labels_list.txt"

static char* labels[OBJ_CLASS_NUM];

const int anchor[3][6] = {{10, 13, 16, 30, 33, 23},
                          {30, 61, 62, 45, 59, 119},
                          {116, 90, 156, 198, 373, 326}};

int clamp(float val, int min, int max) {
    return val > min ? (val < max ? val : max) : min;
}

static float clampf(float val, float min, float max) {
    return val < min ? min : (val > max ? max : val);
}

static char* readLine(FILE* fp, char* buffer, int* len) {
    int ch;
    int i = 0;
    size_t buff_len = 0;

    buffer = (char*)malloc(buff_len + 1);
    if (!buffer)
        return NULL; // Out of memory

    while ((ch = fgetc(fp)) != '\n' && ch != EOF) {
        buff_len++;
        void* tmp = realloc(buffer, buff_len + 1);
        if (tmp == NULL) {
            free(buffer);
            return NULL; // Out of memory
        }
        buffer = (char*)tmp;

        buffer[i] = (char)ch;
        i++;
    }
    buffer[i] = '\0';

    *len = buff_len;

    // Detect end
    if (ch == EOF && (i == 0 || ferror(fp))) {
        free(buffer);
        return NULL;
    }
    return buffer;
}

static int readLines(const char* fileName, char* lines[], int max_line) {
    FILE* file = fopen(fileName, "r");
    char* s;
    int i = 0;
    int n = 0;

    if (file == NULL) {
        printf("Open %s fail!\n", fileName);
        return -1;
    }

    while ((s = readLine(file, s, &n)) != NULL) {
        lines[i++] = s;
        if (i >= max_line)
            break;
    }
    fclose(file);
    return i;
}

static int loadLabelName(const char* locationFilename, char* label[]) {
    // printf("load lable %s\n", locationFilename);
    readLines(locationFilename, label, OBJ_CLASS_NUM);
    return 0;
}

// ---------- 旋转框几何计算（NMS 用） ----------

// xywh + angle -> 4 角点（顺时针），cx/cy 为框中心
static std::vector<float> rbbox_to_corners(const std::vector<float>& rbbox) {
    float cx = rbbox[0] + rbbox[2] / 2;
    float cy = rbbox[1] + rbbox[3] / 2;
    float x_d = rbbox[2];
    float y_d = rbbox[3];
    float angle = rbbox[4];
    float a_cos = std::cos(angle);
    float a_sin = std::sin(angle);
    std::vector<float> corners(8, 0.0);
    float corners_x[4] = {-x_d / 2, -x_d / 2, x_d / 2, x_d / 2};
    float corners_y[4] = {-y_d / 2, y_d / 2, y_d / 2, -y_d / 2};
    for (int i = 0; i < 4; ++i) {
        corners[2 * i] = a_cos * corners_x[i] - a_sin * corners_y[i] + cx;
        corners[2 * i + 1] = a_sin * corners_x[i] + a_cos * corners_y[i] + cy;
    }
    return corners;
}

// 检测点是否在四边形内
static bool point_in_quadrilateral(float pt_x, float pt_y, const std::vector<float>& corners) {
    float ab0 = corners[2] - corners[0];
    float ab1 = corners[3] - corners[1];
    float ad0 = corners[6] - corners[0];
    float ad1 = corners[7] - corners[1];
    float ap0 = pt_x - corners[0];
    float ap1 = pt_y - corners[1];
    float abab = ab0 * ab0 + ab1 * ab1;
    float abap = ab0 * ap0 + ab1 * ap1;
    float adad = ad0 * ad0 + ad1 * ad1;
    float adap = ad0 * ap0 + ad1 * ap1;
    return abab >= abap && abap >= 0 && adad >= adap && adap >= 0;
}

// 线段相交
static int line_segment_intersection(const std::vector<float>& pts1, const std::vector<float>& pts2, int i, int j,
                                     bool& ret1, float& point_x, float& point_y) {
    std::vector<float> A(2), B(2), C(2), D(2), ret(2);
    A[0] = pts1[2 * i];
    A[1] = pts1[2 * i + 1];
    B[0] = pts1[2 * ((i + 1) % 4)];
    B[1] = pts1[2 * ((i + 1) % 4) + 1];
    C[0] = pts2[2 * j];
    C[1] = pts2[2 * j + 1];
    D[0] = pts2[2 * ((j + 1) % 4)];
    D[1] = pts2[2 * ((j + 1) % 4) + 1];

    float BA0 = B[0] - A[0];
    float BA1 = B[1] - A[1];
    float DA0 = D[0] - A[0];
    float CA0 = C[0] - A[0];
    float DA1 = D[1] - A[1];
    float CA1 = C[1] - A[1];

    bool acd = DA1 * CA0 > CA1 * DA0;
    bool bcd = (D[1] - B[1]) * (C[0] - B[0]) > (C[1] - B[1]) * (D[0] - B[0]);

    if (acd != bcd) {
        bool abc = CA1 * BA0 > BA1 * CA0;
        bool abd = DA1 * BA0 > BA1 * DA0;
        if (abc != abd) {
            float DC0 = D[0] - C[0];
            float DC1 = D[1] - C[1];
            float ABBA = A[0] * B[1] - B[0] * A[1];
            float CDDC = C[0] * D[1] - D[0] * C[1];
            float DH = BA1 * DC0 - BA0 * DC1;
            float Dx = ABBA * DC0 - BA0 * CDDC;
            float Dy = ABBA * DC1 - BA1 * CDDC;
            ret[0] = Dx / DH;
            ret[1] = Dy / DH;
            ret1 = true;
            point_x = ret[0];
            point_y = ret[1];
            return 0;
        }
    }
    ret1 = false;
    point_x = 0;
    point_y = 0;
    return 0;
}

// 凸多边形顶点按极角排序
static bool compare_points(const std::vector<float>& pt1, const std::vector<float>& pt2, const std::vector<float>& center) {
    float vx1 = pt1[0] - center[0];
    float vy1 = pt1[1] - center[1];
    float vx2 = pt2[0] - center[0];
    float vy2 = pt2[1] - center[1];
    float d1 = std::sqrt(vx1 * vx1 + vy1 * vy1);
    float d2 = std::sqrt(vx2 * vx2 + vy2 * vy2);
    vx1 /= d1;
    vy1 /= d1;
    vx2 /= d2;
    vy2 /= d2;
    if (vy1 < 0) vx1 = -2 - vx1;
    if (vy2 < 0) vx2 = -2 - vx2;
    return vx1 < vx2;
}

static void sort_vertex_in_convex_polygon(std::vector<std::vector<float>>& int_pts, int num_of_inter) {
    if (num_of_inter > 0) {
        std::vector<float> center(2, 0);
        for (int i = 0; i < num_of_inter; ++i) {
            center[0] += int_pts[i][0];
            center[1] += int_pts[i][1];
        }
        center[0] /= num_of_inter;
        center[1] /= num_of_inter;
        std::sort(int_pts.begin(), int_pts.end(), [&center](const std::vector<float>& pt1, const std::vector<float>& pt2) {
            return compare_points(pt1, pt2, center);
        });
    }
}

static float triangle_area(const std::vector<float>& a, const std::vector<float>& b, const std::vector<float>& c) {
    return std::abs((a[0] - c[0]) * (b[1] - c[1]) - (a[1] - c[1]) * (b[0] - c[0])) / 2.0;
}

static float polygon_area(const std::vector<std::vector<float>>& int_pts, int num_of_inter) {
    float area_val = 0.0;
    for (int i = 1; i < num_of_inter - 1; ++i) {
        area_val += triangle_area(int_pts[0], int_pts[i], int_pts[i + 1]);
    }
    return area_val;
}

// 旋转框 IoU（凸多边形求交）
static float Cal_IOU(float x1, float y1, float w1, float h1, float angle1, float x2, float y2, float w2, float h2, float angle2) {
    std::vector<float> rbbox1 = {x1, y1, w1, h1, angle1};
    std::vector<float> rbbox2 = {x2, y2, w2, h2, angle2};
    std::vector<float> corners1 = rbbox_to_corners(rbbox1);
    std::vector<float> corners2 = rbbox_to_corners(rbbox2);

    std::vector<std::vector<float>> pts;
    int num_pts = 0;
    for (int i = 0; i < 4; ++i) {
        float point_x = corners1[2 * i];
        float point_y = corners1[2 * i + 1];
        if (point_in_quadrilateral(point_x, point_y, corners2)) {
            num_pts++;
            pts.push_back({point_x, point_y});
        }
    }
    for (int i = 0; i < 4; ++i) {
        float point_x = corners2[2 * i];
        float point_y = corners2[2 * i + 1];
        if (point_in_quadrilateral(point_x, point_y, corners1)) {
            num_pts++;
            pts.push_back({point_x, point_y});
        }
    }
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            float point_x, point_y;
            bool ret;
            line_segment_intersection(corners1, corners2, i, j, ret, point_x, point_y);
            if (ret) {
                num_pts++;
                pts.push_back({point_x, point_y});
            }
        }
    }
    sort_vertex_in_convex_polygon(pts, num_pts);
    float polygon_area_val = polygon_area(pts, num_pts);
    float area_union = rbbox1[2] * rbbox1[3] + rbbox2[2] * rbbox2[3] - polygon_area_val;
    if (area_union <= 0) return 0;
    return polygon_area_val / area_union;
}

// 旋转框 NMS（修正官方 angle1/类别判断的小 bug）
static int nms(int validCount, std::vector<float>& outputLocations, std::vector<int>& classIds, std::vector<int>& order,
               int filterId, float threshold) {
    // 注意：objProbs 已排序，order[i] 才是"第 i 高置信度框"在原始数组中的下标。
    // 因此类别必须用 classIds[order[i]] 取，不能直接用 classIds[i]（官方代码此处有 bug，会导致 NMS 失效）。
    for (int i = 0; i < validCount; ++i) {
        if (order[i] == -1) {
            continue;
        }
        int n = order[i];
        if (classIds[n] != filterId) {
            continue;
        }
        for (int j = i + 1; j < validCount; ++j) {
            int m = order[j];
            if (m == -1 || classIds[m] != filterId) {
                continue;
            }
            float xmin0 = outputLocations[n * 5 + 0];
            float ymin0 = outputLocations[n * 5 + 1];
            float w0 = outputLocations[n * 5 + 2];
            float h0 = outputLocations[n * 5 + 3];
            float angle0 = outputLocations[n * 5 + 4];

            float xmin1 = outputLocations[m * 5 + 0];
            float ymin1 = outputLocations[m * 5 + 1];
            float w1 = outputLocations[m * 5 + 2];
            float h1 = outputLocations[m * 5 + 3];
            float angle1 = outputLocations[m * 5 + 4];

            float iou = Cal_IOU(xmin0, ymin0, w0, h0, angle0, xmin1, ymin1, w1, h1, angle1);
            if (iou > threshold) {
                order[j] = -1;
            }
        }
    }
    return 0;
}

static int quick_sort_indice_inverse(std::vector<float>& input, int left, int right, std::vector<int>& indices) {
    float key;
    int key_index;
    int low = left;
    int high = right;
    if (left < right) {
        key_index = indices[left];
        key = input[left];
        while (low < high) {
            while (low < high && input[high] <= key) {
                high--;
            }
            input[low] = input[high];
            indices[low] = indices[high];
            while (low < high && input[low] >= key) {
                low++;
            }
            input[high] = input[low];
            indices[high] = indices[low];
        }
        input[low] = key;
        indices[low] = key_index;
        quick_sort_indice_inverse(input, left, low - 1, indices);
        quick_sort_indice_inverse(input, low + 1, right, indices);
    }
    return low;
}

static float sigmoid(float x) {
    return 1.0 / (1.0 + expf(-x));
}

static float unsigmoid(float y) {
    return -1.0 * logf((1.0 / y) - 1.0);
}

inline static int32_t __clip(float val, float min, float max) {
    float f = val <= min ? min : (val >= max ? max : val);
    return f;
}

static int8_t qnt_f32_to_affine(float f32, int32_t zp, float scale) {
    float dst_val = (f32 / scale) + zp;
    int8_t res = (int8_t)__clip(dst_val, -128, 127);
    return res;
}

static float deqnt_affine_to_f32(int8_t qnt, int32_t zp, float scale) {
    return ((float)qnt - (float)zp) * scale;
}

static void softmax(float* input, int size) {
    float max_val = input[0];
    for (int i = 1; i < size; ++i) {
        if (input[i] > max_val) max_val = input[i];
    }
    float sum_exp = 0.0;
    for (int i = 0; i < size; ++i) {
        sum_exp += expf(input[i] - max_val);
    }
    for (int i = 0; i < size; ++i) {
        input[i] = expf(input[i] - max_val) / sum_exp;
    }
}

// 单分支处理（INT8 量化版）。输出 boxes 为 5 元组 (x, y, w, h, angle)。
static int process_i8(int8_t* input, int8_t* angle_feature, int grid_h, int grid_w, int stride, int dfl_len,
                      std::vector<float>& boxes, std::vector<float>& boxScores, std::vector<int>& classId, float threshold,
                      int32_t zp, float scale, int32_t angle_zp, float angle_scale, int index) {
    int input_loc_len = 4 * dfl_len;
    int tensor_len = input_loc_len + OBJ_CLASS_NUM;
    int validCount = 0;

    int8_t thres_i8 = qnt_f32_to_affine(unsigmoid(threshold), zp, scale);
    for (int h = 0; h < grid_h; h++) {
        for (int w = 0; w < grid_w; w++) {
            for (int a = 0; a < OBJ_CLASS_NUM; a++) {
                if (input[(input_loc_len + a) * grid_w * grid_h + h * grid_w + w] >= thres_i8) {
                    float box_conf_f32 = sigmoid(deqnt_affine_to_f32(input[(input_loc_len + a) * grid_w * grid_h + h * grid_w + w], zp, scale));
                    float loc[input_loc_len];
                    for (int i = 0; i < input_loc_len; ++i) {
                        loc[i] = deqnt_affine_to_f32(input[i * grid_w * grid_h + h * grid_w + w], zp, scale);
                    }
                    for (int i = 0; i < 4; ++i) {
                        softmax(&loc[i * dfl_len], dfl_len);
                    }
                    float xywh_[4] = {0, 0, 0, 0};
                    for (int dfl = 0; dfl < dfl_len; ++dfl) {
                        xywh_[0] += loc[0 * dfl_len + dfl] * dfl;
                        xywh_[1] += loc[1 * dfl_len + dfl] * dfl;
                        xywh_[2] += loc[2 * dfl_len + dfl] * dfl;
                        xywh_[3] += loc[3 * dfl_len + dfl] * dfl;
                    }

                    float xywh_add[2], xywh_sub[2];
                    xywh_add[0] = xywh_[0] + xywh_[2];
                    xywh_add[1] = xywh_[1] + xywh_[3];
                    xywh_sub[0] = (xywh_[2] - xywh_[0]) / 2;
                    xywh_sub[1] = (xywh_[3] - xywh_[1]) / 2;

                    float angle_feature_ = deqnt_affine_to_f32(angle_feature[index + h * grid_w + w], angle_zp, angle_scale);
                    angle_feature_ = (angle_feature_ - 0.25) * 3.1415927410125732;
                    float a_cos = cos(angle_feature_);
                    float a_sin = sin(angle_feature_);

                    // 中心点（含旋转偏移）
                    float xy_mul1 = xywh_sub[0] * a_cos;
                    float xy_mul2 = xywh_sub[1] * a_sin;
                    float xy_mul3 = xywh_sub[0] * a_sin;
                    float xy_mul4 = xywh_sub[1] * a_cos;
                    float cx = ((xy_mul1 - xy_mul2) + w + 0.5) * stride;
                    float cy = ((xy_mul3 + xy_mul4) + h + 0.5) * stride;
                    float bw = xywh_add[0] * stride;
                    float bh = xywh_add[1] * stride;

                    boxes.push_back(cx - bw / 2); // x
                    boxes.push_back(cy - bh / 2); // y
                    boxes.push_back(bw);          // w
                    boxes.push_back(bh);          // h
                    boxes.push_back(angle_feature_); // angle
                    boxScores.push_back(box_conf_f32);
                    classId.push_back(a);
                    validCount++;
                }
            }
        }
    }
    return validCount;
}

// 单分支处理（FP32 非量化版）
static int process_fp32(float* input, float* angle_feature, int grid_h, int grid_w, int stride, int dfl_len,
                        std::vector<float>& boxes, std::vector<float>& boxScores, std::vector<int>& classId, float threshold,
                        int index) {
    int input_loc_len = 4 * dfl_len;
    int validCount = 0;

    float thres_fp32 = unsigmoid(threshold);
    for (int h = 0; h < grid_h; h++) {
        for (int w = 0; w < grid_w; w++) {
            for (int a = 0; a < OBJ_CLASS_NUM; a++) {
                if (input[(input_loc_len + a) * grid_w * grid_h + h * grid_w + w] >= thres_fp32) {
                    float box_conf_f32 = sigmoid(input[(input_loc_len + a) * grid_w * grid_h + h * grid_w + w]);
                    float loc[input_loc_len];
                    for (int i = 0; i < input_loc_len; ++i) {
                        loc[i] = input[i * grid_w * grid_h + h * grid_w + w];
                    }
                    for (int i = 0; i < 4; ++i) {
                        softmax(&loc[i * dfl_len], dfl_len);
                    }
                    float xywh_[4] = {0, 0, 0, 0};
                    for (int dfl = 0; dfl < dfl_len; ++dfl) {
                        xywh_[0] += loc[0 * dfl_len + dfl] * dfl;
                        xywh_[1] += loc[1 * dfl_len + dfl] * dfl;
                        xywh_[2] += loc[2 * dfl_len + dfl] * dfl;
                        xywh_[3] += loc[3 * dfl_len + dfl] * dfl;
                    }

                    float xywh_add[2], xywh_sub[2];
                    xywh_add[0] = xywh_[0] + xywh_[2];
                    xywh_add[1] = xywh_[1] + xywh_[3];
                    xywh_sub[0] = (xywh_[2] - xywh_[0]) / 2;
                    xywh_sub[1] = (xywh_[3] - xywh_[1]) / 2;

                    float angle_feature_ = angle_feature[index + h * grid_w + w];
                    angle_feature_ = (angle_feature_ - 0.25) * 3.1415927410125732;
                    float a_cos = cos(angle_feature_);
                    float a_sin = sin(angle_feature_);

                    float xy_mul1 = xywh_sub[0] * a_cos;
                    float xy_mul2 = xywh_sub[1] * a_sin;
                    float xy_mul3 = xywh_sub[0] * a_sin;
                    float xy_mul4 = xywh_sub[1] * a_cos;
                    float cx = ((xy_mul1 - xy_mul2) + w + 0.5) * stride;
                    float cy = ((xy_mul3 + xy_mul4) + h + 0.5) * stride;
                    float bw = xywh_add[0] * stride;
                    float bh = xywh_add[1] * stride;

                    boxes.push_back(cx - bw / 2);
                    boxes.push_back(cy - bh / 2);
                    boxes.push_back(bw);
                    boxes.push_back(bh);
                    boxes.push_back(angle_feature_);
                    boxScores.push_back(box_conf_f32);
                    classId.push_back(a);
                    validCount++;
                }
            }
        }
    }
    return validCount;
}

int post_process(rknn_app_context_t* app_ctx, rknn_output* outputs, letterbox_t* letter_box, float conf_threshold, float nms_threshold, object_detect_result_list* od_results) {
    std::vector<float> filterBoxes;
    std::vector<float> objProbs;
    std::vector<int> classId;

    int model_in_w = app_ctx->model_width;
    int model_in_h = app_ctx->model_height;

    int validCount = 0;
    int stride = 0;
    int grid_h = 0;
    int grid_w = 0;

    memset(od_results, 0, sizeof(object_detect_result_list));

    // dfl_len = reg_max：由输出通道数反推（通道 = 4*reg_max + nc）
    int dfl_len = (app_ctx->output_attrs[0].dims[1] - OBJ_CLASS_NUM) / 4;

    // 3 个检测头（outputs[0..2]）+ 1 个角度特征（outputs[3]）
    int index = 0;
    for (int i = 0; i < 3; i++) {
        grid_h = app_ctx->output_attrs[i].dims[2];
        grid_w = app_ctx->output_attrs[i].dims[3];
        stride = model_in_h / grid_h;

        if (app_ctx->is_quant) {
            validCount += process_i8((int8_t*)outputs[i].buf, (int8_t*)outputs[3].buf, grid_h, grid_w, stride, dfl_len,
                                     filterBoxes, objProbs, classId, conf_threshold,
                                     app_ctx->output_attrs[i].zp, app_ctx->output_attrs[i].scale,
                                     app_ctx->output_attrs[3].zp, app_ctx->output_attrs[3].scale, index);
        } else {
            validCount += process_fp32((float*)outputs[i].buf, (float*)outputs[3].buf, grid_h, grid_w, stride, dfl_len,
                                       filterBoxes, objProbs, classId, conf_threshold, index);
        }
        index += grid_h * grid_w;
    }

    // 无目标
    if (validCount <= 0) {
        return 0;
    }

    // NMS
    std::vector<int> indexArray;
    for (int i = 0; i < validCount; ++i) {
        indexArray.push_back(i);
    }
    quick_sort_indice_inverse(objProbs, 0, validCount - 1, indexArray);

    std::set<int> class_set(std::begin(classId), std::end(classId));
    for (auto c : class_set) {
        nms(validCount, filterBoxes, classId, indexArray, c, nms_threshold);
    }

    int last_count = 0;
    od_results->count = 0;

    for (int i = 0; i < validCount; ++i) {
        if (indexArray[i] == -1 || last_count >= OBJ_NUMB_MAX_SIZE) {
            continue;
        }
        int n = indexArray[i];

        float x = filterBoxes[n * 5 + 0];
        float y = filterBoxes[n * 5 + 1];
        float w = filterBoxes[n * 5 + 2];
        float h = filterBoxes[n * 5 + 3];
        float angle = filterBoxes[n * 5 + 4];
        int id = classId[n];
        float obj_conf = objProbs[i];

        // 4 角点（model 空间）-> 反变换到原图坐标
        std::vector<float> rb = {x, y, w, h, angle};
        std::vector<float> corners = rbbox_to_corners(rb);

        float minx = FLT_MAX, miny = FLT_MAX, maxx = -FLT_MAX, maxy = -FLT_MAX;
        for (int c = 0; c < 4; c++) {
            float px = clampf(corners[2 * c] - letter_box->x_pad, 0.0f, (float)model_in_w) / letter_box->scale;
            float py = clampf(corners[2 * c + 1] - letter_box->y_pad, 0.0f, (float)model_in_h) / letter_box->scale;
            od_results->results[last_count].corners[2 * c] = px;
            od_results->results[last_count].corners[2 * c + 1] = py;
            if (px < minx) minx = px;
            if (px > maxx) maxx = px;
            if (py < miny) miny = py;
            if (py > maxy) maxy = py;
        }

        od_results->results[last_count].box.left = (int)minx;
        od_results->results[last_count].box.top = (int)miny;
        od_results->results[last_count].box.right = (int)maxx;
        od_results->results[last_count].box.bottom = (int)maxy;
        od_results->results[last_count].angle = angle;
        od_results->results[last_count].prop = obj_conf;
        od_results->results[last_count].cls_id = id;
        last_count++;
    }
    od_results->count = last_count;

    return 0;
}

int init_post_process() {
    int ret = 0;
    ret = loadLabelName(LABEL_NALE_TXT_PATH, labels);
    if (ret < 0) {
        printf("Load %s failed!\n", LABEL_NALE_TXT_PATH);
        return -1;
    }
    return 0;
}

char* coco_cls_to_name(int cls_id) {
    if (cls_id >= OBJ_CLASS_NUM) {
        return "null";
    }
    if (labels[cls_id]) {
        return labels[cls_id];
    }
    return "null";
}

void deinit_post_process() {
    for (int i = 0; i < OBJ_CLASS_NUM; i++) {
        {
            free(labels[i]);
            labels[i] = nullptr;
        }
    }
}
