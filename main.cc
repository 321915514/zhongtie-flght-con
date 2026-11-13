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

/*-------------------------------------------
                Includes
-------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <vector>

#include "yolov8_seg.h"
#include "image_utils.h"
#include <opencv2/opencv.hpp>

/*-------------------------------------------
                  Functions
-------------------------------------------*/

double __get_us(struct timeval t) { return (t.tv_sec * 1000000 + t.tv_usec); }

/*-------------------------------------------
                  Main Function
-------------------------------------------*/
int main(int argc, char **argv)
{
    if (argc != 3)
    {
        printf("%s <model_path> <image_path>\n", argv[0]);
        return -1;
    }

    const char *model_path = argv[1];
    const char *image_path = argv[2];

    unsigned char class_colors[][3] = {
        {255, 56, 56},   // 'FF3838'
        {255, 157, 151}, // 'FF9D97'
        {255, 112, 31},  // 'FF701F'
        {255, 178, 29},  // 'FFB21D'
        {207, 210, 49},  // 'CFD231'
        {72, 249, 10},   // '48F90A'
        {146, 204, 23},  // '92CC17'
        {61, 219, 134},  // '3DDB86'
        {26, 147, 52},   // '1A9334'
        {0, 212, 187},   // '00D4BB'
        {44, 153, 168},  // '2C99A8'
        {0, 194, 255},   // '00C2FF'
        {52, 69, 147},   // '344593'
        {100, 115, 255}, // '6473FF'
        {0, 24, 236},    // '0018EC'
        {132, 56, 255},  // '8438FF'
        {82, 0, 133},    // '520085'
        {203, 56, 255},  // 'CB38FF'
        {255, 149, 200}, // 'FF95C8'
        {255, 55, 199}   // 'FF37C7'
    };

    int ret;
    cv::Mat orig_img, image;
    struct timeval start_time, stop_time;
    rknn_app_context_t rknn_app_ctx;
    memset(&rknn_app_ctx, 0, sizeof(rknn_app_context_t));

    // 使用OpenCV读取图片
    image_buffer_t src_image;
    memset(&src_image, 0, sizeof(image_buffer_t));
    // gettimeofday(&start_time, NULL);
    orig_img = cv::imread(image_path);
    if (!orig_img.data) {
        printf("cv::imread %s fail!\n", image_path);
        return -1;
    }
    // gettimeofday(&stop_time, NULL);
    // printf("OpenCV read an image using %f ms\n", (__get_us(stop_time) - __get_us(start_time)) / 1000);
    cv::cvtColor(orig_img, image, cv::COLOR_BGR2RGB);
    src_image.width  = image.cols;
    src_image.height = image.rows;
    src_image.format = IMAGE_FORMAT_RGB888;
    src_image.virt_addr = (unsigned char*)image.data;

    // 初始化
    init_post_process();

    ret = init_yolov8_seg_model(model_path, &rknn_app_ctx);
    if (ret != 0)
    {
        printf("init_yolov8_seg_model fail! ret=%d model_path=%s\n", ret, model_path);
        goto out;
    }

    // rknn推理和处理
    gettimeofday(&start_time, NULL);
    object_detect_result_list od_results;
    ret = inference_yolov8_seg_model(&rknn_app_ctx, &src_image, &od_results);
    if (ret != 0)
    {
        printf("init_yolov8_seg_model fail! ret=%d\n", ret);
        goto out;
    }

    // draw boxes
    char text[256];
    for (int i = 0; i < od_results.count; i++)
    {
        object_detect_result *det_result = &(od_results.results[i]);
        printf("%s @ (%d %d %d %d) %.3f\n", coco_cls_to_name(det_result->cls_id),
               det_result->box.left, det_result->box.top,
               det_result->box.right, det_result->box.bottom,
               det_result->prop);
        int x1 = det_result->box.left;
        int y1 = det_result->box.top;
        int x2 = det_result->box.right;
        int y2 = det_result->box.bottom;

        sprintf(text, "%s %.1f%%", coco_cls_to_name(det_result->cls_id), det_result->prop * 100);

        // 绘制旋转框（OBB 4 角点）
        const float *crn = det_result->corners;
        std::vector<cv::Point> rot_pts;
        for (int c = 0; c < 4; c++)
            rot_pts.push_back(cv::Point((int)crn[2 * c], (int)crn[2 * c + 1]));
        cv::polylines(orig_img, rot_pts, true, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

        putText(orig_img, text, cv::Point(x1, y1 - 6), cv::FONT_HERSHEY_DUPLEX, 0.7, cv::Scalar(255,255,255), 1, cv::LINE_AA);
    }

    gettimeofday(&stop_time, NULL);
    printf("rknn run and process use %f ms\n", (__get_us(stop_time) - __get_us(start_time)) / 1000);

    // 保存结果
    cv::imwrite("./out.jpg", orig_img);

out:
    deinit_post_process();

    ret = release_yolov8_seg_model(&rknn_app_ctx);
    if (ret != 0)
    {
        printf("release_yolov8_seg_model fail! ret=%d\n", ret);
    }

    return 0;
}
