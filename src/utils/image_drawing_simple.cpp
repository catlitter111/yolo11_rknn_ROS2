#include "image_drawing.h"
#include <opencv2/opencv.hpp>
#include <algorithm>

int draw_rectangle(image_buffer_t* image_buffer, int x, int y, int width, int height, 
                   COLOR_INDEX color, int thickness)
{
    if (image_buffer == NULL || image_buffer->virt_addr == NULL) {
        return -1;
    }
    
    cv::Mat img;
    if (image_buffer->format == IMAGE_FORMAT_RGB888) {
        img = cv::Mat(image_buffer->height, image_buffer->width, CV_8UC3, image_buffer->virt_addr);
    } else if (image_buffer->format == IMAGE_FORMAT_BGR888) {
        img = cv::Mat(image_buffer->height, image_buffer->width, CV_8UC3, image_buffer->virt_addr);
    } else {
        return -1;
    }
    
    cv::Scalar cv_color;
    switch (color) {
        case COLOR_RED:
            cv_color = (image_buffer->format == IMAGE_FORMAT_RGB888) ? 
                       cv::Scalar(255, 0, 0) : cv::Scalar(0, 0, 255);
            break;
        case COLOR_GREEN:
            cv_color = cv::Scalar(0, 255, 0);
            break;
        case COLOR_BLUE:
            cv_color = (image_buffer->format == IMAGE_FORMAT_RGB888) ? 
                       cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
            break;
        default:
            cv_color = cv::Scalar(255, 255, 255);
            break;
    }
    
    cv::rectangle(img, cv::Point(x, y), cv::Point(x + width, y + height), cv_color, thickness);
    
    return 0;
}

int draw_text(image_buffer_t* image_buffer, const char* text, int x, int y, 
              COLOR_INDEX color, int font_size)
{
    if (image_buffer == NULL || image_buffer->virt_addr == NULL || text == NULL) {
        return -1;
    }
    
    cv::Mat img;
    if (image_buffer->format == IMAGE_FORMAT_RGB888) {
        img = cv::Mat(image_buffer->height, image_buffer->width, CV_8UC3, image_buffer->virt_addr);
    } else if (image_buffer->format == IMAGE_FORMAT_BGR888) {
        img = cv::Mat(image_buffer->height, image_buffer->width, CV_8UC3, image_buffer->virt_addr);
    } else {
        return -1;
    }
    
    cv::Scalar cv_color;
    switch (color) {
        case COLOR_RED:
            cv_color = (image_buffer->format == IMAGE_FORMAT_RGB888) ? 
                       cv::Scalar(255, 0, 0) : cv::Scalar(0, 0, 255);
            break;
        case COLOR_GREEN:
            cv_color = cv::Scalar(0, 255, 0);
            break;
        case COLOR_BLUE:
            cv_color = (image_buffer->format == IMAGE_FORMAT_RGB888) ? 
                       cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
            break;
        default:
            cv_color = cv::Scalar(255, 255, 255);
            break;
    }
    
    double font_scale = font_size / 10.0;
    cv::putText(img, text, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 
                font_scale, cv_color, 2);
    
    return 0;
}