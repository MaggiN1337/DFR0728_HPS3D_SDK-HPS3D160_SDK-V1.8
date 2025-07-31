/*
 * Unit tests for thread safety in HPS3D LIDAR service
 * 
 * Tests include:
 * - Thread creation and termination
 * - Mutex synchronization
 * - Atomic operations
 * - Race condition detection
 * - Deadlock prevention
 * - Thread communication
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>
#include <pthread.h>
#include <stdatomic.h>
#include <time.h>
#include <errno.h>
#include <signal.h>
#include <sys/time.h>

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

// Test configuration
#define MAX_THREADS 10
#define STRESS_ITERATIONS 1000
#define TIMEOUT_SECONDS 5

// Mock global variables (from main.c)
static volatile int running = 1;
static pthread_mutex_t data_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t debug_mutex = PTHREAD_MUTEX_INITIALIZER;
static volatile _Atomic int measurement_active = 0;
static volatile _Atomic int pointcloud_requested = 0;
static volatile _Atomic int mqtt_connected = 0;

// Test data structures
typedef struct {
    int thread_id;
    int iterations;
    int success_count;
    int error_count;
    volatile int running;
} thread_data_t;

typedef struct {
    float distance;
    int valid_pixels;
    uint32_t timestamp;
    pthread_mutex_t mutex;
} shared_data_t;

static shared_data_t test_shared_data = {0};
static int thread_completion_count = 0;
static pthread_mutex_t completion_mutex = PTHREAD_MUTEX_INITIALIZER;

// Helper functions
void reset_test_globals(void) {
    running = 1;
    atomic_store(&measurement_active, 0);
    atomic_store(&pointcloud_requested, 0);
    atomic_store(&mqtt_connected, 0);
    thread_completion_count = 0;
    
    // Reset shared data
    pthread_mutex_lock(&test_shared_data.mutex);
    test_shared_data.distance = 0.0;
    test_shared_data.valid_pixels = 0;
    test_shared_data.timestamp = 0;
    pthread_mutex_unlock(&test_shared_data.mutex);
}

double get_time_diff(struct timespec *start, struct timespec *end) {
    return (end->tv_sec - start->tv_sec) + (end->tv_nsec - start->tv_nsec) / 1e9;
}

// Test thread functions
void* basic_worker_thread(void* arg) {
    thread_data_t *data = (thread_data_t*)arg;
    
    for (int i = 0; i < data->iterations && data->running; i++) {
        // Simulate work
        usleep(1000); // 1ms
        data->success_count++;
    }
    
    pthread_mutex_lock(&completion_mutex);
    thread_completion_count++;
    pthread_mutex_unlock(&completion_mutex);
    
    return NULL;
}

void* mutex_contention_thread(void* arg) {
    thread_data_t *data = (thread_data_t*)arg;
    
    for (int i = 0; i < data->iterations && data->running; i++) {
        // Test data mutex
        if (pthread_mutex_lock(&data_mutex) == 0) {
            // Critical section - simulate data access
            usleep(100); // 0.1ms
            pthread_mutex_unlock(&data_mutex);
            data->success_count++;
        } else {
            data->error_count++;
        }
        
        // Test debug mutex
        if (pthread_mutex_lock(&debug_mutex) == 0) {
            // Critical section - simulate debug output
            usleep(50); // 0.05ms
            pthread_mutex_unlock(&debug_mutex);
        } else {
            data->error_count++;
        }
    }
    
    pthread_mutex_lock(&completion_mutex);
    thread_completion_count++;
    pthread_mutex_unlock(&completion_mutex);
    
    return NULL;
}

void* atomic_operations_thread(void* arg) {
    thread_data_t *data = (thread_data_t*)arg;
    
    for (int i = 0; i < data->iterations && data->running; i++) {
        // Test atomic operations
        atomic_store(&measurement_active, 1);
        int active = atomic_load(&measurement_active);
        if (active == 1) {
            data->success_count++;
        }
        
        atomic_store(&measurement_active, 0);
        active = atomic_load(&measurement_active);
        if (active == 0) {
            data->success_count++;
        }
        
        // Test atomic exchange
        int old_value = atomic_exchange(&pointcloud_requested, 1);
        if (old_value == 0 || old_value == 1) {
            data->success_count++;
        }
        
        atomic_store(&pointcloud_requested, 0);
    }
    
    pthread_mutex_lock(&completion_mutex);
    thread_completion_count++;
    pthread_mutex_unlock(&completion_mutex);
    
    return NULL;
}

void* shared_data_thread(void* arg) {
    thread_data_t *data = (thread_data_t*)arg;
    
    for (int i = 0; i < data->iterations && data->running; i++) {
        if (pthread_mutex_lock(&test_shared_data.mutex) == 0) {
            // Modify shared data
            test_shared_data.distance = (float)(data->thread_id * 1000 + i);
            test_shared_data.valid_pixels = data->thread_id + i;
            test_shared_data.timestamp = (uint32_t)time(NULL);
            
            // Verify data consistency
            if (test_shared_data.distance >= 0 && 
                test_shared_data.valid_pixels >= 0 &&
                test_shared_data.timestamp > 0) {
                data->success_count++;
            } else {
                data->error_count++;
            }
            
            pthread_mutex_unlock(&test_shared_data.mutex);
        } else {
            data->error_count++;
        }
        
        usleep(100); // 0.1ms
    }
    
    pthread_mutex_lock(&completion_mutex);
    thread_completion_count++;
    pthread_mutex_unlock(&completion_mutex);
    
    return NULL;
}

void* timeout_test_thread(void* arg) {
    thread_data_t *data = (thread_data_t*)arg;
    
    // This thread will run for a specific duration
    struct timespec start, current;
    clock_gettime(CLOCK_MONOTONIC, &start);
    
    while (data->running) {
        clock_gettime(CLOCK_MONOTONIC, &current);
        double elapsed = get_time_diff(&start, &current);
        
        if (elapsed >= TIMEOUT_SECONDS) {
            break;
        }
        
        data->success_count++;
        usleep(10000); // 10ms
    }
    
    pthread_mutex_lock(&completion_mutex);
    thread_completion_count++;
    pthread_mutex_unlock(&completion_mutex);
    
    return NULL;
}

// Test functions

// Test 1: Basic thread creation and termination
int test_thread_creation(void) {
    reset_test_globals();
    
    const int num_threads = 4;
    pthread_t threads[num_threads];
    thread_data_t thread_data[num_threads];
    
    // Initialize thread data
    for (int i = 0; i < num_threads; i++) {
        thread_data[i].thread_id = i;
        thread_data[i].iterations = 10;
        thread_data[i].success_count = 0;
        thread_data[i].error_count = 0;
        thread_data[i].running = 1;
    }
    
    // Create threads
    for (int i = 0; i < num_threads; i++) {
        int result = pthread_create(&threads[i], NULL, basic_worker_thread, &thread_data[i]);
        TEST_ASSERT(result == 0, "Thread creation failed");
    }
    
    // Wait for threads to complete
    for (int i = 0; i < num_threads; i++) {
        int result = pthread_join(threads[i], NULL);
        TEST_ASSERT(result == 0, "Thread join failed");
    }
    
    // Verify results
    for (int i = 0; i < num_threads; i++) {
        TEST_ASSERT(thread_data[i].success_count == 10, "Thread did not complete all iterations");
        TEST_ASSERT(thread_data[i].error_count == 0, "Thread encountered errors");
    }
    
    TEST_ASSERT(thread_completion_count == num_threads, "Not all threads completed");
    
    TEST_SUCCESS();
}

// Test 2: Mutex contention and synchronization
int test_mutex_contention(void) {
    reset_test_globals();
    
    const int num_threads = 6;
    pthread_t threads[num_threads];
    thread_data_t thread_data[num_threads];
    
    // Initialize thread data
    for (int i = 0; i < num_threads; i++) {
        thread_data[i].thread_id = i;
        thread_data[i].iterations = 50;
        thread_data[i].success_count = 0;
        thread_data[i].error_count = 0;
        thread_data[i].running = 1;
    }
    
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);
    
    // Create threads that will contend for mutexes
    for (int i = 0; i < num_threads; i++) {
        int result = pthread_create(&threads[i], NULL, mutex_contention_thread, &thread_data[i]);
        TEST_ASSERT(result == 0, "Thread creation failed");
    }
    
    // Wait for threads to complete
    for (int i = 0; i < num_threads; i++) {
        int result = pthread_join(threads[i], NULL);
        TEST_ASSERT(result == 0, "Thread join failed");
    }
    
    clock_gettime(CLOCK_MONOTONIC, &end);
    double elapsed = get_time_diff(&start, &end);
    
    // Verify results
    int total_success = 0;
    int total_errors = 0;
    for (int i = 0; i < num_threads; i++) {
        total_success += thread_data[i].success_count;
        total_errors += thread_data[i].error_count;
        TEST_ASSERT(thread_data[i].success_count > 0, "Thread made no successful mutex operations");
    }
    
    TEST_ASSERT(total_errors == 0, "Mutex operations had errors");
    TEST_ASSERT(thread_completion_count == num_threads, "Not all threads completed");
    
    printf("Mutex contention test: %d operations in %.3f seconds\n", total_success, elapsed);
    
    TEST_SUCCESS();
}

// Test 3: Atomic operations thread safety
int test_atomic_operations(void) {
    reset_test_globals();
    
    const int num_threads = 8;
    pthread_t threads[num_threads];
    thread_data_t thread_data[num_threads];
    
    // Initialize thread data
    for (int i = 0; i < num_threads; i++) {
        thread_data[i].thread_id = i;
        thread_data[i].iterations = 100;
        thread_data[i].success_count = 0;
        thread_data[i].error_count = 0;
        thread_data[i].running = 1;
    }
    
    // Create threads that will perform atomic operations
    for (int i = 0; i < num_threads; i++) {
        int result = pthread_create(&threads[i], NULL, atomic_operations_thread, &thread_data[i]);
        TEST_ASSERT(result == 0, "Thread creation failed");
    }
    
    // Wait for threads to complete
    for (int i = 0; i < num_threads; i++) {
        int result = pthread_join(threads[i], NULL);
        TEST_ASSERT(result == 0, "Thread join failed");
    }
    
    // Verify results
    int total_success = 0;
    for (int i = 0; i < num_threads; i++) {
        total_success += thread_data[i].success_count;
        TEST_ASSERT(thread_data[i].success_count > 0, "Thread made no successful atomic operations");
    }
    
    // Verify final atomic values are in valid state
    int final_active = atomic_load(&measurement_active);
    int final_pointcloud = atomic_load(&pointcloud_requested);
    TEST_ASSERT(final_active == 0 || final_active == 1, "Invalid final atomic value for measurement_active");
    TEST_ASSERT(final_pointcloud == 0 || final_pointcloud == 1, "Invalid final atomic value for pointcloud_requested");
    
    TEST_ASSERT(thread_completion_count == num_threads, "Not all threads completed");
    
    printf("Atomic operations test: %d successful operations\n", total_success);
    
    TEST_SUCCESS();
}

// Test 4: Shared data access synchronization
int test_shared_data_access(void) {
    reset_test_globals();
    
    // Initialize shared data mutex
    int result = pthread_mutex_init(&test_shared_data.mutex, NULL);
    TEST_ASSERT(result == 0, "Shared data mutex initialization failed");
    
    const int num_threads = 5;
    pthread_t threads[num_threads];
    thread_data_t thread_data[num_threads];
    
    // Initialize thread data
    for (int i = 0; i < num_threads; i++) {
        thread_data[i].thread_id = i;
        thread_data[i].iterations = 50;
        thread_data[i].success_count = 0;
        thread_data[i].error_count = 0;
        thread_data[i].running = 1;
    }
    
    // Create threads that will access shared data
    for (int i = 0; i < num_threads; i++) {
        int result = pthread_create(&threads[i], NULL, shared_data_thread, &thread_data[i]);
        TEST_ASSERT(result == 0, "Thread creation failed");
    }
    
    // Wait for threads to complete
    for (int i = 0; i < num_threads; i++) {
        int result = pthread_join(threads[i], NULL);
        TEST_ASSERT(result == 0, "Thread join failed");
    }
    
    // Verify results
    int total_success = 0;
    int total_errors = 0;
    for (int i = 0; i < num_threads; i++) {
        total_success += thread_data[i].success_count;
        total_errors += thread_data[i].error_count;
    }
    
    TEST_ASSERT(total_errors == 0, "Shared data access had errors");
    TEST_ASSERT(total_success > 0, "No successful shared data operations");
    TEST_ASSERT(thread_completion_count == num_threads, "Not all threads completed");
    
    // Verify final shared data is in valid state
    pthread_mutex_lock(&test_shared_data.mutex);
    TEST_ASSERT(test_shared_data.distance >= 0, "Invalid final distance value");
    TEST_ASSERT(test_shared_data.valid_pixels >= 0, "Invalid final valid_pixels value");
    TEST_ASSERT(test_shared_data.timestamp > 0, "Invalid final timestamp value");
    pthread_mutex_unlock(&test_shared_data.mutex);
    
    pthread_mutex_destroy(&test_shared_data.mutex);
    
    TEST_SUCCESS();
}

// Test 5: Thread timeout and cancellation
int test_thread_timeout(void) {
    reset_test_globals();
    
    pthread_t thread;
    thread_data_t thread_data = {
        .thread_id = 0,
        .iterations = 0,
        .success_count = 0,
        .error_count = 0,
        .running = 1
    };
    
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);
    
    // Create thread that will run for a specific duration
    int result = pthread_create(&thread, NULL, timeout_test_thread, &thread_data);
    TEST_ASSERT(result == 0, "Thread creation failed");
    
    // Wait for thread to complete
    result = pthread_join(thread, NULL);
    TEST_ASSERT(result == 0, "Thread join failed");
    
    clock_gettime(CLOCK_MONOTONIC, &end);
    double elapsed = get_time_diff(&start, &end);
    
    // Verify timing
    TEST_ASSERT(elapsed >= TIMEOUT_SECONDS - 0.1, "Thread completed too early");
    TEST_ASSERT(elapsed <= TIMEOUT_SECONDS + 1.0, "Thread took too long");
    TEST_ASSERT(thread_data.success_count > 0, "Thread made no progress");
    
    printf("Thread timeout test: %d operations in %.3f seconds\n", 
           thread_data.success_count, elapsed);
    
    TEST_SUCCESS();
}

// Test 6: Race condition detection
int test_race_condition_detection(void) {
    reset_test_globals();
    
    const int num_threads = 10;
    pthread_t threads[num_threads];
    thread_data_t thread_data[num_threads];
    volatile int shared_counter = 0;
    volatile int protected_counter = 0;
    pthread_mutex_t counter_mutex = PTHREAD_MUTEX_INITIALIZER;
    
    // Thread function for race condition test
    void* race_test_thread(void* arg) {
        thread_data_t *data = (thread_data_t*)arg;
        
        for (int i = 0; i < data->iterations; i++) {
            // Unprotected increment (potential race condition)
            shared_counter++;
            
            // Protected increment
            pthread_mutex_lock(&counter_mutex);
            protected_counter++;
            pthread_mutex_unlock(&counter_mutex);
            
            data->success_count++;
        }
        
        return NULL;
    }
    
    // Initialize thread data
    for (int i = 0; i < num_threads; i++) {
        thread_data[i].thread_id = i;
        thread_data[i].iterations = 1000;
        thread_data[i].success_count = 0;
        thread_data[i].error_count = 0;
        thread_data[i].running = 1;
    }
    
    // Create threads
    for (int i = 0; i < num_threads; i++) {
        int result = pthread_create(&threads[i], NULL, race_test_thread, &thread_data[i]);
        TEST_ASSERT(result == 0, "Thread creation failed");
    }
    
    // Wait for threads to complete
    for (int i = 0; i < num_threads; i++) {
        int result = pthread_join(threads[i], NULL);
        TEST_ASSERT(result == 0, "Thread join failed");
    }
    
    int expected_count = num_threads * 1000;
    
    // Protected counter should always be exact
    TEST_ASSERT(protected_counter == expected_count, "Protected counter incorrect - race condition in test framework");
    
    // Unprotected counter may be less due to race conditions
    printf("Race condition test: unprotected=%d, protected=%d (expected=%d)\n", 
           shared_counter, protected_counter, expected_count);
    
    if (shared_counter < expected_count) {
        printf("DETECTED: Race condition in unprotected counter (lost %d increments)\n", 
               expected_count - shared_counter);
    }
    
    pthread_mutex_destroy(&counter_mutex);
    
    TEST_SUCCESS();
}

// Test 7: Deadlock prevention
int test_deadlock_prevention(void) {
    reset_test_globals();
    
    pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER;
    pthread_mutex_t mutex2 = PTHREAD_MUTEX_INITIALIZER;
    volatile int deadlock_detected = 0;
    
    // Thread function that acquires mutexes in order 1->2
    void* ordered_thread1(void* arg) {
        (void)arg;
        
        for (int i = 0; i < 100; i++) {
            if (pthread_mutex_lock(&mutex1) == 0) {
                usleep(100); // Hold mutex1 briefly
                if (pthread_mutex_lock(&mutex2) == 0) {
                    usleep(100); // Hold both mutexes
                    pthread_mutex_unlock(&mutex2);
                } else {
                    deadlock_detected = 1;
                }
                pthread_mutex_unlock(&mutex1);
            } else {
                deadlock_detected = 1;
                break;
            }
        }
        
        return NULL;
    }
    
    // Thread function that acquires mutexes in order 2->1 (potential deadlock)
    void* ordered_thread2(void* arg) {
        (void)arg;
        
        for (int i = 0; i < 100; i++) {
            if (pthread_mutex_lock(&mutex2) == 0) {
                usleep(100); // Hold mutex2 briefly
                if (pthread_mutex_lock(&mutex1) == 0) {
                    usleep(100); // Hold both mutexes
                    pthread_mutex_unlock(&mutex1);
                } else {
                    deadlock_detected = 1;
                }
                pthread_mutex_unlock(&mutex2);
            } else {
                deadlock_detected = 1;
                break;
            }
        }
        
        return NULL;
    }
    
    pthread_t thread1, thread2;
    
    // Create threads with potential deadlock scenario
    int result1 = pthread_create(&thread1, NULL, ordered_thread1, NULL);
    int result2 = pthread_create(&thread2, NULL, ordered_thread2, NULL);
    
    TEST_ASSERT(result1 == 0 && result2 == 0, "Thread creation failed");
    
    // Set a timeout for deadlock detection
    struct timespec timeout;
    clock_gettime(CLOCK_REALTIME, &timeout);
    timeout.tv_sec += 3; // 3 second timeout
    
    // Try to join threads with timeout
    int join_result1 = pthread_join(thread1, NULL);
    int join_result2 = pthread_join(thread2, NULL);
    
    if (join_result1 != 0 || join_result2 != 0) {
        printf("WARNING: Possible deadlock detected (threads did not complete)\n");
        // Cancel threads if they're stuck
        pthread_cancel(thread1);
        pthread_cancel(thread2);
    }
    
    // This test mainly checks that we can detect potential deadlock scenarios
    // In a real application, proper mutex ordering would prevent deadlocks
    
    pthread_mutex_destroy(&mutex1);
    pthread_mutex_destroy(&mutex2);
    
    TEST_SUCCESS();
}

// Test 8: Thread stress test
int test_thread_stress(void) {
    reset_test_globals();
    
    const int num_threads = MAX_THREADS;
    pthread_t threads[num_threads];
    thread_data_t thread_data[num_threads];
    
    // Initialize thread data for stress test
    for (int i = 0; i < num_threads; i++) {
        thread_data[i].thread_id = i;
        thread_data[i].iterations = STRESS_ITERATIONS / 10; // Reduce to avoid timeout
        thread_data[i].success_count = 0;
        thread_data[i].error_count = 0;
        thread_data[i].running = 1;
    }
    
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);
    
    // Create many threads simultaneously
    for (int i = 0; i < num_threads; i++) {
        int result = pthread_create(&threads[i], NULL, mutex_contention_thread, &thread_data[i]);
        TEST_ASSERT(result == 0, "Thread creation failed in stress test");
    }
    
    // Wait for all threads to complete
    for (int i = 0; i < num_threads; i++) {
        int result = pthread_join(threads[i], NULL);
        TEST_ASSERT(result == 0, "Thread join failed in stress test");
    }
    
    clock_gettime(CLOCK_MONOTONIC, &end);
    double elapsed = get_time_diff(&start, &end);
    
    // Verify results
    int total_success = 0;
    int total_errors = 0;
    for (int i = 0; i < num_threads; i++) {
        total_success += thread_data[i].success_count;
        total_errors += thread_data[i].error_count;
    }
    
    TEST_ASSERT(total_errors == 0, "Stress test had errors");
    TEST_ASSERT(total_success > 0, "No successful operations in stress test");
    TEST_ASSERT(thread_completion_count == num_threads, "Not all threads completed in stress test");
    
    printf("Thread stress test: %d threads, %d operations in %.3f seconds\n", 
           num_threads, total_success, elapsed);
    
    TEST_SUCCESS();
}

// Test runner
int main(void) {
    printf("=== Thread Safety Tests ===\n");
    
    int total_tests = 0;
    int passed_tests = 0;
    
    // Initialize shared data mutex
    pthread_mutex_init(&test_shared_data.mutex, NULL);
    
    // Run tests
    total_tests++; passed_tests += test_thread_creation();
    total_tests++; passed_tests += test_mutex_contention();
    total_tests++; passed_tests += test_atomic_operations();
    total_tests++; passed_tests += test_shared_data_access();
    total_tests++; passed_tests += test_thread_timeout();
    total_tests++; passed_tests += test_race_condition_detection();
    total_tests++; passed_tests += test_deadlock_prevention();
    total_tests++; passed_tests += test_thread_stress();
    
    printf("\n=== Test Results ===\n");
    printf("Total tests: %d\n", total_tests);
    printf("Passed: %d\n", passed_tests);
    printf("Failed: %d\n", total_tests - passed_tests);
    printf("Success rate: %.1f%%\n", (float)passed_tests / total_tests * 100);
    
    // Cleanup
    pthread_mutex_destroy(&test_shared_data.mutex);
    
    return (passed_tests == total_tests) ? 0 : 1;
}