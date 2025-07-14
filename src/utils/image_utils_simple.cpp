#include "image_utils.h"
#include <opencv2/opencv.hpp>
#include <cstring>
#include <algorithm>

int read_image(const char* image_path, image_buffer_t* image_buffer)
{
    cv::Mat img = cv::imread(image_path);
    if (img.empty()) {
        printf("Failed to read image: %s\n", image_path);
        return -1;
    }
    
    // 转换为RGB格式
    cv::Mat rgb_img;
    cv::cvtColor(img, rgb_img, cv::COLOR_BGR2RGB);
    
    image_buffer->width = rgb_img.cols;
    image_buffer->height = rgb_img.rows;
    image_buffer->format = IMAGE_FORMAT_RGB888;
    image_buffer->size = rgb_img.cols * rgb_img.rows * 3;
    
    image_buffer->virt_addr = (unsigned char*)malloc(image_buffer->size);
    if (image_buffer->virt_addr == NULL) {
        printf("Failed to allocate memory for image\n");
        return -1;
    }
    
    memcpy(image_buffer->virt_addr, rgb_img.data, image_buffer->size);
    image_buffer->fd = -1;
    
    return 0;
}

int write_image(const char* image_path, image_buffer_t* image_buffer)
{
    if (image_buffer == NULL || image_buffer->virt_addr == NULL) {
        printf("Invalid image buffer\n");
        return -1;
    }
    
    cv::Mat img;
    if (image_buffer->format == IMAGE_FORMAT_RGB888) {
        img = cv::Mat(image_buffer->height, image_buffer->width, CV_8UC3, image_buffer->virt_addr);
        cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
    } else if (image_buffer->format == IMAGE_FORMAT_BGR888) {
        img = cv::Mat(image_buffer->height, image_buffer->width, CV_8UC3, image_buffer->virt_addr);
    } else {
        printf("Unsupported image format\n");
        return -1;
    }
    
    bool success = cv::imwrite(image_path, img);
    return success ? 0 : -1;
}

int get_image_size(image_buffer_t* image_buffer)
{
    if (image_buffer == NULL) {
        return -1;
    }
    
    int pixel_size = 0;
    switch (image_buffer->format) {
        case IMAGE_FORMAT_RGB888:
        case IMAGE_FORMAT_BGR888:
            pixel_size = 3;
            break;
        case IMAGE_FORMAT_RGBA8888:
            pixel_size = 4;
            break;
        case IMAGE_FORMAT_GRAY8:
            pixel_size = 1;
            break;
        default:
            pixel_size = 3;
            break;
    }
    
    return image_buffer->width * image_buffer->height * pixel_size;
}

int convert_image_with_letterbox(image_buffer_t* src, image_buffer_t* dst, letterbox_t* letterbox, int bg_color)
{
    if (src == NULL || dst == NULL || letterbox == NULL) {
        return -1;
    }
    
    cv::Mat src_img;
    if (src->format == IMAGE_FORMAT_RGB888) {
        src_img = cv::Mat(src->height, src->width, CV_8UC3, src->virt_addr);
    } else if (src->format == IMAGE_FORMAT_BGR888) {
        src_img = cv::Mat(src->height, src->width, CV_8UC3, src->virt_addr);
        cv::cvtColor(src_img, src_img, cv::COLOR_BGR2RGB);
    } else {
        printf("Unsupported source image format\n");
        return -1;
    }
    
    // 计算缩放比例
    float scale_x = (float)dst->width / src->width;
    float scale_y = (float)dst->height / src->height;
    float scale = std::min(scale_x, scale_y);
    
    // 计算缩放后的尺寸
    int new_width = (int)(src->width * scale);
    int new_height = (int)(src->height * scale);
    
    // 计算padding
    int pad_x = (dst->width - new_width) / 2;
    int pad_y = (dst->height - new_height) / 2;
    
    letterbox->scale = scale;
    letterbox->x_pad = pad_x;
    letterbox->y_pad = pad_y;
    
    // 创建目标图像
    cv::Mat dst_img = cv::Mat::zeros(dst->height, dst->width, CV_8UC3);
    if (bg_color == 0) {
        dst_img.setTo(cv::Scalar(114, 114, 114)); // 默认灰色背景
    }
    
    // 缩放源图像
    cv::Mat resized_img;
    cv::resize(src_img, resized_img, cv::Size(new_width, new_height));
    
    // 将缩放后的图像放置在目标图像中心
    cv::Rect roi(pad_x, pad_y, new_width, new_height);
    resized_img.copyTo(dst_img(roi));
    
    // 复制到目标缓冲区
    memcpy(dst->virt_addr, dst_img.data, dst->size);
    
    return 0;
}