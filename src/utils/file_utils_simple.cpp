#include "file_utils.h"
#include <fstream>
#include <cstring>

int read_data_from_file(const char* filename, char** data)
{
    std::ifstream file(filename, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        printf("Failed to open file: %s\n", filename);
        return -1;
    }
    
    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);
    
    *data = (char*)malloc(size);
    if (*data == NULL) {
        printf("Failed to allocate memory for file data\n");
        file.close();
        return -1;
    }
    
    if (!file.read(*data, size)) {
        printf("Failed to read file data\n");
        free(*data);
        *data = NULL;
        file.close();
        return -1;
    }
    
    file.close();
    return static_cast<int>(size);
}