/*
 * Unit tests for memory management in HPS3D LIDAR service
 * 
 * Tests include:
 * - Memory initialization and cleanup
 * - Mutex operations
 * - Data structure validation
 * - Memory leaks detection
 * - Thread-safe memory access
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>
#include <pthread.h>
#include <stdatomic.h>
#include <time.h>
#include <sys/resource.h>
#include <errno.h>

// Test framework macros
#define TEST_ASSERT(condition, message) \
    do { \
        if (!(condition)) { \
            printf("FAIL: %s - %s\n", __func__, message); \
            return 0; \
        } \
    } while(0)

#define TEST_SUCCESS() \
    do { \
        printf("PASS: %s\n", __func__); \
        return 1; \
    } while(0)

// Memory tracking structure
typedef struct {
    void *ptr;
    size_t size;
    const char *file;
    int line;
    struct memory_track *next;
} memory_track_t;

static memory_track_t *memory_head = NULL;
static pthread_mutex_t memory_track_mutex = PTHREAD_MUTEX_INITIALIZER;
static size_t total_allocated = 0;
static size_t peak_allocated = 0;

// Memory tracking functions
void* tracked_malloc(size_t size, const char* file, int line) {
    void *ptr = malloc(size);
    if (!ptr) return NULL;
    
    pthread_mutex_lock(&memory_track_mutex);
    
    memory_track_t *track = malloc(sizeof(memory_track_t));
    if (track) {
        track->ptr = ptr;
        track->size = size;
        track->file = file;
        track->line = line;
        track->next = memory_head;
        memory_head = track;
        
        total_allocated += size;
        if (total_allocated > peak_allocated) {
            peak_allocated = total_allocated;
        }
    }
    
    pthread_mutex_unlock(&memory_track_mutex);
    return ptr;
}

void tracked_free(void *ptr) {
    if (!ptr) return;
    
    pthread_mutex_lock(&memory_track_mutex);
    
    memory_track_t **current = &memory_head;
    while (*current) {
        if ((*current)->ptr == ptr) {
            memory_track_t *to_free = *current;
            total_allocated -= to_free->size;
            *current = to_free->next;
            free(to_free);
            break;
        }
        current = &(*current)->next;
    }
    
    pthread_mutex_unlock(&memory_track_mutex);
    free(ptr);
}

int get_memory_leaks(void) {
    pthread_mutex_lock(&memory_track_mutex);
    
    int leak_count = 0;
    memory_track_t *current = memory_head;
    while (current) {
        printf("LEAK: %zu bytes at %p (allocated at %s:%d)\n", 
               current->size, current->ptr, current->file, current->line);
        leak_count++;
        current = current->next;
    }
    
    pthread_mutex_unlock(&memory_track_mutex);
    return leak_count;
}

void reset_memory_tracking(void) {
    pthread_mutex_lock(&memory_track_mutex);
    
    while (memory_head) {
        memory_track_t *to_free = memory_head;
        memory_head = memory_head->next;
        free(to_free->ptr);
        free(to_free);
    }
    
    total_allocated = 0;
    peak_allocated = 0;
    
    pthread_mutex_unlock(&memory_track_mutex);
}

size_t get_current_memory_usage(void) {
    pthread_mutex_lock(&memory_track_mutex);
    size_t usage = total_allocated;
    pthread_mutex_unlock(&memory_track_mutex);
    return usage;
}

size_t get_peak_memory_usage(void) {
    pthread_mutex_lock(&memory_track_mutex);
    size_t usage = peak_allocated;
    pthread_mutex_unlock(&memory_track_mutex);
    return usage;
}

#define TRACKED_MALLOC(size) tracked_malloc(size, __FILE__, __LINE__)
#define TRACKED_FREE(ptr) tracked_free(ptr)

// Mock measurement point structure (from main.c)
typedef struct {
    int x, y;
    float distance;
    float min_distance;
    float max_distance;
    int valid_pixels;
    uint32_t timestamp;
    char name[16];
    struct {
        unsigned int valid : 1;
    } flags;
} MeasurePoint;

// Mock global variables for testing
static pthread_mutex_t test_data_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t test_debug_mutex = PTHREAD_MUTEX_INITIALIZER;
static volatile _Atomic int test_measurement_active = 0;
static MeasurePoint test_points[4];

// Test functions

// Test 1: Basic memory allocation and deallocation
int test_basic_memory_operations(void) {
    reset_memory_tracking();
    
    // Test malloc/free
    void *ptr1 = TRACKED_MALLOC(1024);
    TEST_ASSERT(ptr1 != NULL, "Memory allocation failed");
    TEST_ASSERT(get_current_memory_usage() == 1024, "Memory usage not tracked correctly");
    
    void *ptr2 = TRACKED_MALLOC(2048);
    TEST_ASSERT(ptr2 != NULL, "Second memory allocation failed");
    TEST_ASSERT(get_current_memory_usage() == 3072, "Memory usage not updated correctly");
    
    TRACKED_FREE(ptr1);
    TEST_ASSERT(get_current_memory_usage() == 2048, "Memory not freed correctly");
    
    TRACKED_FREE(ptr2);
    TEST_ASSERT(get_current_memory_usage() == 0, "All memory not freed");
    
    TEST_ASSERT(get_memory_leaks() == 0, "Memory leaks detected");
    
    TEST_SUCCESS();
}

// Test 2: Memory leak detection
int test_memory_leak_detection(void) {
    reset_memory_tracking();
    
    // Allocate memory without freeing
    void *ptr1 = TRACKED_MALLOC(512);
    void *ptr2 = TRACKED_MALLOC(1024);
    (void)ptr1; // Suppress unused variable warning
    (void)ptr2;
    
    TEST_ASSERT(get_current_memory_usage() == 1536, "Memory usage incorrect");
    TEST_ASSERT(get_memory_leaks() == 2, "Memory leaks not detected correctly");
    
    reset_memory_tracking(); // Clean up for next tests
    TEST_SUCCESS();
}

// Test 3: Peak memory usage tracking
int test_peak_memory_tracking(void) {
    reset_memory_tracking();
    
    void *ptr1 = TRACKED_MALLOC(1000);
    size_t peak1 = get_peak_memory_usage();
    TEST_ASSERT(peak1 == 1000, "Peak memory not tracked correctly");
    
    void *ptr2 = TRACKED_MALLOC(2000);
    size_t peak2 = get_peak_memory_usage();
    TEST_ASSERT(peak2 == 3000, "Peak memory not updated correctly");
    
    TRACKED_FREE(ptr1);
    size_t peak3 = get_peak_memory_usage();
    TEST_ASSERT(peak3 == 3000, "Peak memory should not decrease after free");
    
    void *ptr3 = TRACKED_MALLOC(500);
    size_t peak4 = get_peak_memory_usage();
    TEST_ASSERT(peak4 == 3000, "Peak memory should remain same");
    
    TRACKED_FREE(ptr2);
    TRACKED_FREE(ptr3);
    
    TEST_ASSERT(get_memory_leaks() == 0, "Memory leaks detected");
    
    TEST_SUCCESS();
}

// Test 4: Mutex initialization and operations
int test_mutex_operations(void) {
    pthread_mutex_t test_mutex = PTHREAD_MUTEX_INITIALIZER;
    
    // Test lock/unlock
    int result = pthread_mutex_lock(&test_mutex);
    TEST_ASSERT(result == 0, "Mutex lock failed");
    
    result = pthread_mutex_unlock(&test_mutex);
    TEST_ASSERT(result == 0, "Mutex unlock failed");
    
    // Test trylock
    result = pthread_mutex_trylock(&test_mutex);
    TEST_ASSERT(result == 0, "Mutex trylock failed");
    
    result = pthread_mutex_unlock(&test_mutex);
    TEST_ASSERT(result == 0, "Mutex unlock after trylock failed");
    
    // Test destroy
    result = pthread_mutex_destroy(&test_mutex);
    TEST_ASSERT(result == 0, "Mutex destroy failed");
    
    TEST_SUCCESS();
}

// Test 5: Data structure initialization
int test_data_structure_initialization(void) {
    // Initialize test points
    for (int i = 0; i < 4; i++) {
        test_points[i].x = 40 + i * 20;
        test_points[i].y = 30;
        test_points[i].distance = 0.0;
        test_points[i].min_distance = 0.0;
        test_points[i].max_distance = 0.0;
        test_points[i].valid_pixels = 0;
        test_points[i].timestamp = 0;
        snprintf(test_points[i].name, sizeof(test_points[i].name), "point_%d", i + 1);
        test_points[i].flags.valid = 0;
    }
    
    // Verify initialization
    for (int i = 0; i < 4; i++) {
        TEST_ASSERT(test_points[i].x >= 40 && test_points[i].x <= 100, "X coordinate not initialized correctly");
        TEST_ASSERT(test_points[i].y == 30, "Y coordinate not initialized correctly");
        TEST_ASSERT(test_points[i].distance == 0.0, "Distance not initialized to zero");
        TEST_ASSERT(test_points[i].flags.valid == 0, "Valid flag not initialized to false");
        TEST_ASSERT(strlen(test_points[i].name) > 0, "Name not initialized");
    }
    
    TEST_SUCCESS();
}

// Test 6: Atomic operations
int test_atomic_operations(void) {
    // Test atomic store/load
    atomic_store(&test_measurement_active, 0);
    TEST_ASSERT(atomic_load(&test_measurement_active) == 0, "Atomic store/load failed");
    
    atomic_store(&test_measurement_active, 1);
    TEST_ASSERT(atomic_load(&test_measurement_active) == 1, "Atomic store/load failed");
    
    // Test atomic exchange
    int old_value = atomic_exchange(&test_measurement_active, 0);
    TEST_ASSERT(old_value == 1, "Atomic exchange returned wrong old value");
    TEST_ASSERT(atomic_load(&test_measurement_active) == 0, "Atomic exchange didn't set new value");
    
    // Test atomic compare and swap
    int expected = 0;
    bool success = atomic_compare_exchange_strong(&test_measurement_active, &expected, 1);
    TEST_ASSERT(success == true, "Atomic compare and swap failed");
    TEST_ASSERT(atomic_load(&test_measurement_active) == 1, "Atomic compare and swap didn't set value");
    
    TEST_SUCCESS();
}

// Thread function for concurrent memory access test
void* memory_access_thread(void* arg) {
    int thread_id = *(int*)arg;
    
    for (int i = 0; i < 100; i++) {
        pthread_mutex_lock(&test_data_mutex);
        
        // Simulate data access
        test_points[thread_id % 4].distance = (float)(thread_id * 100 + i);
        test_points[thread_id % 4].timestamp = (uint32_t)time(NULL);
        test_points[thread_id % 4].flags.valid = (i % 2 == 0) ? 1 : 0;
        
        pthread_mutex_unlock(&test_data_mutex);
        
        usleep(1000); // 1ms
    }
    
    return NULL;
}

// Test 7: Thread-safe memory access
int test_thread_safe_memory_access(void) {
    const int num_threads = 4;
    pthread_t threads[num_threads];
    int thread_ids[num_threads];
    
    // Initialize test data
    for (int i = 0; i < 4; i++) {
        test_points[i].distance = 0.0;
        test_points[i].timestamp = 0;
        test_points[i].flags.valid = 0;
    }
    
    // Create threads
    for (int i = 0; i < num_threads; i++) {
        thread_ids[i] = i;
        int result = pthread_create(&threads[i], NULL, memory_access_thread, &thread_ids[i]);
        TEST_ASSERT(result == 0, "Thread creation failed");
    }
    
    // Wait for threads to complete
    for (int i = 0; i < num_threads; i++) {
        int result = pthread_join(threads[i], NULL);
        TEST_ASSERT(result == 0, "Thread join failed");
    }
    
    // Verify data integrity
    pthread_mutex_lock(&test_data_mutex);
    for (int i = 0; i < 4; i++) {
        TEST_ASSERT(test_points[i].timestamp > 0, "Timestamp not updated by threads");
        // Distance should be updated by at least one thread
        TEST_ASSERT(test_points[i].distance >= 0.0, "Distance corrupted by concurrent access");
    }
    pthread_mutex_unlock(&test_data_mutex);
    
    TEST_SUCCESS();
}

// Test 8: Memory stress test
int test_memory_stress(void) {
    reset_memory_tracking();
    
    const int num_allocations = 1000;
    void **ptrs = malloc(num_allocations * sizeof(void*));
    TEST_ASSERT(ptrs != NULL, "Failed to allocate pointer array");
    
    // Allocate many small blocks
    for (int i = 0; i < num_allocations; i++) {
        size_t size = 64 + (i % 256); // Variable sizes 64-319 bytes
        ptrs[i] = TRACKED_MALLOC(size);
        TEST_ASSERT(ptrs[i] != NULL, "Memory allocation failed in stress test");
    }
    
    size_t stress_peak = get_peak_memory_usage();
    TEST_ASSERT(stress_peak > 64000, "Peak memory usage too low"); // At least 64KB
    
    // Free every other block
    for (int i = 0; i < num_allocations; i += 2) {
        TRACKED_FREE(ptrs[i]);
        ptrs[i] = NULL;
    }
    
    size_t after_partial_free = get_current_memory_usage();
    TEST_ASSERT(after_partial_free < stress_peak, "Memory not freed correctly");
    TEST_ASSERT(after_partial_free > 0, "Too much memory freed");
    
    // Free remaining blocks
    for (int i = 1; i < num_allocations; i += 2) {
        TRACKED_FREE(ptrs[i]);
        ptrs[i] = NULL;
    }
    
    TEST_ASSERT(get_current_memory_usage() == 0, "Memory not completely freed");
    TEST_ASSERT(get_memory_leaks() == 0, "Memory leaks detected in stress test");
    
    free(ptrs);
    TEST_SUCCESS();
}

// Thread function for concurrent allocation/deallocation
void* allocation_thread(void* arg) {
    int thread_id = *(int*)arg;
    const int allocations_per_thread = 100;
    
    for (int i = 0; i < allocations_per_thread; i++) {
        size_t size = 128 + (thread_id * 64) + (i % 128);
        void *ptr = TRACKED_MALLOC(size);
        if (ptr) {
            // Write some data to ensure memory is accessible
            memset(ptr, thread_id, size);
            
            // Sleep briefly to simulate work
            usleep(100);
            
            // Verify data integrity
            unsigned char *data = (unsigned char*)ptr;
            for (size_t j = 0; j < size; j++) {
                if (data[j] != (unsigned char)thread_id) {
                    printf("ERROR: Data corruption in thread %d at offset %zu\n", thread_id, j);
                    break;
                }
            }
            
            TRACKED_FREE(ptr);
        }
    }
    
    return NULL;
}

// Test 9: Concurrent memory allocation
int test_concurrent_memory_allocation(void) {
    reset_memory_tracking();
    
    const int num_threads = 8;
    pthread_t threads[num_threads];
    int thread_ids[num_threads];
    
    // Create threads for concurrent allocation/deallocation
    for (int i = 0; i < num_threads; i++) {
        thread_ids[i] = i;
        int result = pthread_create(&threads[i], NULL, allocation_thread, &thread_ids[i]);
        TEST_ASSERT(result == 0, "Thread creation failed");
    }
    
    // Wait for all threads to complete
    for (int i = 0; i < num_threads; i++) {
        int result = pthread_join(threads[i], NULL);
        TEST_ASSERT(result == 0, "Thread join failed");
    }
    
    // Verify no memory leaks
    TEST_ASSERT(get_current_memory_usage() == 0, "Memory not completely freed after concurrent test");
    TEST_ASSERT(get_memory_leaks() == 0, "Memory leaks detected in concurrent test");
    
    TEST_SUCCESS();
}

// Test 10: Resource limit handling
int test_resource_limits(void) {
    struct rlimit limit;
    
    // Get current memory limit
    int result = getrlimit(RLIMIT_AS, &limit);
    TEST_ASSERT(result == 0, "Failed to get resource limits");
    
    printf("Current memory limit: %ld bytes (soft), %ld bytes (hard)\n", 
           (long)limit.rlim_cur, (long)limit.rlim_max);
    
    // Test memory allocation within reasonable bounds
    reset_memory_tracking();
    
    // Try to allocate 1MB - should succeed on most systems
    void *large_ptr = TRACKED_MALLOC(1024 * 1024);
    if (large_ptr) {
        // Write to the memory to ensure it's actually allocated
        memset(large_ptr, 0xAA, 1024 * 1024);
        TRACKED_FREE(large_ptr);
        TEST_ASSERT(get_memory_leaks() == 0, "Memory leak in large allocation test");
    } else {
        printf("WARNING: Could not allocate 1MB - system may be low on memory\n");
    }
    
    TEST_SUCCESS();
}

// Test runner
int main(void) {
    printf("=== Memory Management Tests ===\n");
    
    int total_tests = 0;
    int passed_tests = 0;
    
    // Run tests
    total_tests++; passed_tests += test_basic_memory_operations();
    total_tests++; passed_tests += test_memory_leak_detection();
    total_tests++; passed_tests += test_peak_memory_tracking();
    total_tests++; passed_tests += test_mutex_operations();
    total_tests++; passed_tests += test_data_structure_initialization();
    total_tests++; passed_tests += test_atomic_operations();
    total_tests++; passed_tests += test_thread_safe_memory_access();
    total_tests++; passed_tests += test_memory_stress();
    total_tests++; passed_tests += test_concurrent_memory_allocation();
    total_tests++; passed_tests += test_resource_limits();
    
    printf("\n=== Test Results ===\n");
    printf("Total tests: %d\n", total_tests);
    printf("Passed: %d\n", passed_tests);
    printf("Failed: %d\n", total_tests - passed_tests);
    printf("Success rate: %.1f%%\n", (float)passed_tests / total_tests * 100);
    printf("Peak memory usage during tests: %zu bytes\n", get_peak_memory_usage());
    
    // Final cleanup
    reset_memory_tracking();
    
    return (passed_tests == total_tests) ? 0 : 1;
}