#ifndef _COMMON_H_
#define _COMMON_H_

#include <stdint.h>

typedef enum {
    IMAGE_FORMAT_GRAY8,
    IMAGE_FORMAT_RGB888,
    IMAGE_FORMAT_BGR888,
    IMAGE_FORMAT_RGBA8888,
    IMAGE_FORMAT_YUV420SP_NV21,
    IMAGE_FORMAT_YUV420SP_NV12,
    IMAGE_FORMAT_YUV420P_YU12,
    IMAGE_FORMAT_YUV420P_YV12,
    IMAGE_FORMAT_YUV422SP_NV16,
    IMAGE_FORMAT_YUV422P_YU16,
    IMAGE_FORMAT_YUV422SP_NV61,
    IMAGE_FORMAT_YUV422P_YV16,
    IMAGE_FORMAT_UNDEFINED
} image_format_t;

typedef struct {
    int left;
    int top;
    int right;
    int bottom;
} image_rect_t;

typedef struct {
    int width;
    int height;
    int format;
    unsigned char* virt_addr;
    int fd;
    int size;
} image_buffer_t;

typedef struct {
    float x_pad;
    float y_pad;
    float scale;
} letterbox_t;

typedef enum {
    COLOR_RED,
    COLOR_GREEN,
    COLOR_BLUE,
    COLOR_WHITE,
    COLOR_BLACK
} COLOR_INDEX;

// Format and type conversion functions
const char* get_format_string(int format);
const char* get_type_string(int type);
const char* get_qnt_type_string(int qnt_type);

#endif // _COMMON_H_