#ifndef _IMAGE_DRAWING_H_
#define _IMAGE_DRAWING_H_

#include "common.h"

#ifdef __cplusplus
extern "C" {
#endif

int draw_rectangle(image_buffer_t* image_buffer, int x, int y, int width, int height, 
                   COLOR_INDEX color, int thickness);
int draw_text(image_buffer_t* image_buffer, const char* text, int x, int y, 
              COLOR_INDEX color, int font_size);

#ifdef __cplusplus
}
#endif

#endif // _IMAGE_DRAWING_H_