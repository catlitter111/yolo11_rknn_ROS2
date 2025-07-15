// Copyright (c) 2024 by Rockchip Electronics Co., Ltd. All Rights Reserved.
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

#ifndef _RKNN_DEMO_YOLOV8_POSE_H_
#define _RKNN_DEMO_YOLOV8_POSE_H_

#include "rknn_api.h"
#include "common.h"

// 包含yolo11.h以使用其rknn_app_context_t定义
#include "yolo11.h"

#include "postprocess_pose.h"

int init_yolov8_pose_model(const char* model_path, rknn_app_context_t* app_ctx);

int release_yolov8_pose_model(rknn_app_context_t* app_ctx);

int inference_yolov8_pose_model(rknn_app_context_t* app_ctx, image_buffer_t* img, pose_object_detect_result_list* od_results);

#endif //_RKNN_DEMO_YOLOV8_POSE_H_ 