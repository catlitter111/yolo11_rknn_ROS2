#ifndef _FILE_UTILS_H_
#define _FILE_UTILS_H_

#ifdef __cplusplus
extern "C" {
#endif

int read_data_from_file(const char* filename, char** data);

#ifdef __cplusplus
}
#endif

#endif // _FILE_UTILS_H_