#ifndef _IMAGE_UTILS_H_
#define _IMAGE_UTILS_H_

#include "common.h"

#ifdef __cplusplus
extern "C" {
#endif

int read_image(const char* image_path, image_buffer_t* image_buffer);
int write_image(const char* image_path, image_buffer_t* image_buffer);
int get_image_size(image_buffer_t* image_buffer);
int convert_image_with_letterbox(image_buffer_t* src, image_buffer_t* dst, letterbox_t* letterbox, int bg_color);

#ifdef __cplusplus
}
#endif

#endif // _IMAGE_UTILS_H_