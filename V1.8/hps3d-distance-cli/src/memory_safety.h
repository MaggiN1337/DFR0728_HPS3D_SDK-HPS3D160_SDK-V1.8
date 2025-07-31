#ifndef MEMORY_SAFETY_H
#define MEMORY_SAFETY_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>

// Memory safety macros for defensive programming
#define SAFE_FREE(ptr) do { \
    if (ptr) { \
        free(ptr); \
        ptr = NULL; \
    } \
} while(0)

#define SAFE_MEMSET(ptr, value, size) do { \
    if (ptr && size > 0) { \
        memset(ptr, value, size); \
    } \
} while(0)

#define CHECK_NULL_RETURN(ptr, retval) do { \
    if (!(ptr)) { \
        fprintf(stderr, "NULL pointer detected at %s:%d\n", __FILE__, __LINE__); \
        return retval; \
    } \
} while(0)

#define CHECK_NULL_RETURN_VOID(ptr) do { \
    if (!(ptr)) { \
        fprintf(stderr, "NULL pointer detected at %s:%d\n", __FILE__, __LINE__); \
        return; \
    } \
} while(0)

#define BOUNDS_CHECK(index, max_size) do { \
    if ((index) < 0 || (index) >= (max_size)) { \
        fprintf(stderr, "Bounds check failed: index %d, max %d at %s:%d\n", \
                (int)(index), (int)(max_size), __FILE__, __LINE__); \
        return -1; \
    } \
} while(0)

#define SAFE_ARRAY_ACCESS(array, index, max_size, default_val) \
    (((array) && (index) >= 0 && (index) < (max_size)) ? (array)[index] : (default_val))

// Memory validation functions
static inline int validate_hps3d_measure_data(const HPS3D_MeasureData_t* data) {
    if (!data) {
        fprintf(stderr, "HPS3D_MeasureData_t is NULL\n");
        return -1;
    }
    
    if (!data->full_depth_data.distance) {
        fprintf(stderr, "HPS3D distance array is NULL\n");
        return -1;
    }
    
    if (!data->full_depth_data.point_cloud_data.point_data) {
        fprintf(stderr, "HPS3D point cloud data is NULL\n");
        return -1;
    }
    
    if (!data->full_roi_data) {
        fprintf(stderr, "HPS3D full ROI data is NULL\n");
        return -1;
    }
    
    if (!data->simple_roi_data) {
        fprintf(stderr, "HPS3D simple ROI data is NULL\n");
        return -1;
    }
    
    return 0;
}

// Thread-safe memory tracking (optional debug feature)
#ifdef MEMORY_DEBUG
typedef struct memory_block {
    void* ptr;
    size_t size;
    const char* file;
    int line;
    struct memory_block* next;
} memory_block_t;

extern memory_block_t* memory_head;
extern pthread_mutex_t memory_debug_mutex;
extern size_t total_memory_allocated;

void* debug_malloc(size_t size, const char* file, int line);
void debug_free(void* ptr, const char* file, int line);
void print_memory_leaks(void);

#define DEBUG_MALLOC(size) debug_malloc(size, __FILE__, __LINE__)
#define DEBUG_FREE(ptr) debug_free(ptr, __FILE__, __LINE__)

#else
#define DEBUG_MALLOC(size) malloc(size)
#define DEBUG_FREE(ptr) free(ptr)
#endif

// Critical error handling
#define CRITICAL_ERROR(msg) do { \
    fprintf(stderr, "CRITICAL ERROR at %s:%d: %s\n", __FILE__, __LINE__, msg); \
    fprintf(stderr, "errno: %s\n", strerror(errno)); \
    abort(); \
} while(0)

#define CRITICAL_ERROR_IF(condition, msg) do { \
    if (condition) { \
        CRITICAL_ERROR(msg); \
    } \
} while(0)

// Safe string operations
#define SAFE_STRNCPY(dest, src, size) do { \
    if (dest && src && size > 0) { \
        strncpy(dest, src, size - 1); \
        dest[size - 1] = '\0'; \
    } \
} while(0)

#define SAFE_SNPRINTF(dest, size, format, ...) do { \
    if (dest && size > 0) { \
        int result = snprintf(dest, size, format, __VA_ARGS__); \
        if (result < 0 || result >= (int)size) { \
            fprintf(stderr, "snprintf overflow/error at %s:%d\n", __FILE__, __LINE__); \
        } \
    } \
} while(0)

#endif // MEMORY_SAFETY_H