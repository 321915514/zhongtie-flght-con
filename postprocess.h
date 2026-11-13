#ifndef _RKNN_YOLOV8_SEG_DEMO_POSTPROCESS_H_
#define _RKNN_YOLOV8_SEG_DEMO_POSTPROCESS_H_

#include <stdint.h>
#include <vector>
#include "rknn_api.h"
#include "image_utils.h"

#define OBJ_NAME_MAX_SIZE 64
#define OBJ_NUMB_MAX_SIZE 128
#define OBJ_CLASS_NUM 2
#define NMS_THRESH 0.50
#define BOX_THRESH 0.40
#define PROP_BOX_SIZE (5 + OBJ_CLASS_NUM)

#define N_CLASS_COLORS 20

// class rknn_app_context_t;

typedef struct
{
    image_rect_t box;   // AABB（4 角点外接正框，left/top/right/bottom，原图像素坐标）
    float prop;         // 置信度
    int cls_id;         // 类别 id（0=hole, 1=rect）
    float angle;        // 旋转角（弧度，[-π/4, 3π/4)）
    float corners[8];   // 4 个角点，顺时针，x0 y0 x1 y1 x2 y2 x3 y3（原图像素坐标）
} object_detect_result;

typedef struct
{
    int id;
    int count;
    object_detect_result results[OBJ_NUMB_MAX_SIZE];
} object_detect_result_list;

int init_post_process();
void deinit_post_process();
char *coco_cls_to_name(int cls_id);
int post_process(rknn_app_context_t *app_ctx, rknn_output *outputs, letterbox_t *letter_box, float conf_threshold, float nms_threshold, object_detect_result_list *od_results);
int clamp(float val, int min, int max);

#endif //_RKNN_YOLOV8_SEG_DEMO_POSTPROCESS_H_
