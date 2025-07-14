#include "common.h"
#include <stdio.h>

const char* get_format_string(int format)
{
    switch(format) {
        case 0: return "NCHW";
        case 1: return "NHWC";
        case 2: return "NC1HWC0";
        default: return "UNKNOWN";
    }
}

const char* get_type_string(int type)
{
    switch(type) {
        case 0: return "FLOAT32";
        case 1: return "FLOAT16";
        case 2: return "INT8";
        case 3: return "UINT8";
        case 4: return "INT16";
        case 5: return "UINT16";
        case 6: return "INT32";
        case 7: return "UINT32";
        case 8: return "INT64";
        case 9: return "BOOL";
        default: return "UNKNOWN";
    }
}

const char* get_qnt_type_string(int qnt_type)
{
    switch(qnt_type) {
        case 0: return "NONE";
        case 1: return "DFP";
        case 2: return "AFFINE_ASYMMETRIC";
        default: return "UNKNOWN";
    }
}