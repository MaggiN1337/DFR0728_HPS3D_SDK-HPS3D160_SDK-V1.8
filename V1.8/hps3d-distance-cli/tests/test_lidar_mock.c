/*
 * Unit tests for LIDAR interface with mock HPS3D library
 * 
 * Tests include:
 * - LIDAR initialization and connection
 * - Measurement data acquisition
 * - Point measurement with various scenarios
 * - Error handling and recovery
 * - Memory management
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>
#include <time.h>
#include <stdint.h>

// Mock HPS3D library definitions
#define HPS3D_RET_OK 0
#define HPS3D_RET_ERROR -1
#define HPS3D_RET_CONNECT_FAILED -2
#define HPS3D_RET_READ_ERR -3
#define HPS3D_RET_WRITE_ERR -4

#define HPS3D_LOW_AMPLITUDE 65001
#define HPS3D_SATURATION 65002
#define HPS3D_ADC_OVERFLOW 65003
#define HPS3D_INVALID_DATA 65004

typedef enum {
    HPS3D_DISCONNECT_EVEN = 0,
    HPS3D_SYS_EXCEPTION_EVEN = 1,
    HPS3D_FULL_DEPTH_EVEN = 2
} HPS3D_EventType_t;

typedef enum {
    HPS3D_SMOOTH_FILTER_DISABLE = 0
} HPS3D_SmoothFilterType_t;

typedef int HPS3D_StatusTypeDef;

// Mock measurement data structure
typedef struct {
    struct {
        uint16_t *distance;
    } full_depth_data;
} HPS3D_MeasureData_t;

// Mock global state
static int mock_handle = -1;
static int mock_connected = 0;
static int mock_capture_active = 0;
static uint16_t mock_distance_data[160 * 60];
static int mock_should_fail = 0;
static int mock_fail_count = 0;
static HPS3D_MeasureData_t mock_measure_data = {0};

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

// Mock HPS3D functions
HPS3D_StatusTypeDef HPS3D_MeasureDataInit(HPS3D_MeasureData_t *data) {
    if (!data) return HPS3D_RET_ERROR;
    
    data->full_depth_data.distance = mock_distance_data;
    return HPS3D_RET_OK;
}

HPS3D_StatusTypeDef HPS3D_MeasureDataFree(HPS3D_MeasureData_t *data) {
    if (!data) return HPS3D_RET_ERROR;
    
    data->full_depth_data.distance = NULL;
    return HPS3D_RET_OK;
}

HPS3D_StatusTypeDef HPS3D_RegisterEventCallback(void (*callback)(int, int, uint8_t*, int, void*), void *userdata) {
    (void)callback;
    (void)userdata;
    // Mock always succeeds
    return HPS3D_RET_OK;
}

HPS3D_StatusTypeDef HPS3D_UnregisterEventCallback(void) {
    // Mock always succeeds
    return HPS3D_RET_OK;
}

HPS3D_StatusTypeDef HPS3D_USBConnectDevice(const char *port, int *handle) {
    (void)port;
    
    if (mock_should_fail && mock_fail_count > 0) {
        mock_fail_count--;
        return HPS3D_RET_CONNECT_FAILED;
    }
    
    mock_handle = 1;
    mock_connected = 1;
    *handle = mock_handle;
    return HPS3D_RET_OK;
}

HPS3D_StatusTypeDef HPS3D_CloseDevice(int handle) {
    if (handle != mock_handle) return HPS3D_RET_ERROR;
    
    mock_connected = 0;
    mock_capture_active = 0;
    mock_handle = -1;
    return HPS3D_RET_OK;
}

int HPS3D_IsConnect(int handle) {
    return (handle == mock_handle && mock_connected);
}

const char* HPS3D_GetDeviceVersion(int handle) {
    if (handle != mock_handle) return "UNKNOWN";
    return "HPS3D-160 Mock v1.0";
}

HPS3D_StatusTypeDef HPS3D_SetDistanceFilterConf(int handle, int enable, float threshold) {
    (void)enable;
    (void)threshold;
    return (handle == mock_handle) ? HPS3D_RET_OK : HPS3D_RET_ERROR;
}

HPS3D_StatusTypeDef HPS3D_SetSmoothFilterConf(int handle, HPS3D_SmoothFilterType_t type, int param) {
    (void)type;
    (void)param;
    return (handle == mock_handle) ? HPS3D_RET_OK : HPS3D_RET_ERROR;
}

HPS3D_StatusTypeDef HPS3D_SetEdgeFilterEnable(int handle, int enable) {
    (void)enable;
    return (handle == mock_handle) ? HPS3D_RET_OK : HPS3D_RET_ERROR;
}

HPS3D_StatusTypeDef HPS3D_SetOpticalPathCalibration(int handle, int enable) {
    (void)enable;
    return (handle == mock_handle) ? HPS3D_RET_OK : HPS3D_RET_ERROR;
}

HPS3D_StatusTypeDef HPS3D_StartCapture(int handle) {
    if (handle != mock_handle || !mock_connected) return HPS3D_RET_ERROR;
    
    mock_capture_active = 1;
    return HPS3D_RET_OK;
}

HPS3D_StatusTypeDef HPS3D_StopCapture(int handle) {
    if (handle != mock_handle) return HPS3D_RET_ERROR;
    
    mock_capture_active = 0;
    return HPS3D_RET_OK;
}

HPS3D_StatusTypeDef HPS3D_SingleCapture(int handle, HPS3D_EventType_t *event_type, HPS3D_MeasureData_t *data) {
    if (handle != mock_handle || !mock_connected || !mock_capture_active) {
        return HPS3D_RET_ERROR;
    }
    
    if (mock_should_fail && mock_fail_count > 0) {
        mock_fail_count--;
        return HPS3D_RET_READ_ERR;
    }
    
    *event_type = HPS3D_FULL_DEPTH_EVEN;
    *data = mock_measure_data;
    return HPS3D_RET_OK;
}

// Mock data generation functions
void mock_generate_valid_data(int x, int y, uint16_t distance) {
    for (int dy = -2; dy <= 2; dy++) {
        for (int dx = -2; dx <= 2; dx++) {
            int px = x + dx;
            int py = y + dy;
            if (px >= 0 && px < 160 && py >= 0 && py < 60) {
                int idx = py * 160 + px;
                // Add some noise
                mock_distance_data[idx] = distance + (rand() % 20 - 10);
            }
        }
    }
}

void mock_generate_invalid_data(int x, int y) {
    for (int dy = -2; dy <= 2; dy++) {
        for (int dx = -2; dx <= 2; dx++) {
            int px = x + dx;
            int py = y + dy;
            if (px >= 0 && px < 160 && py >= 0 && py < 60) {
                int idx = py * 160 + px;
                mock_distance_data[idx] = HPS3D_INVALID_DATA;
            }
        }
    }
}

void mock_clear_data(void) {
    memset(mock_distance_data, 0, sizeof(mock_distance_data));
}

void mock_reset_state(void) {
    mock_handle = -1;
    mock_connected = 0;
    mock_capture_active = 0;
    mock_should_fail = 0;
    mock_fail_count = 0;
    mock_clear_data();
}

// Test functions

// Test 1: LIDAR initialization
int test_lidar_init(void) {
    mock_reset_state();
    
    HPS3D_MeasureData_t data;
    HPS3D_StatusTypeDef ret = HPS3D_MeasureDataInit(&data);
    TEST_ASSERT(ret == HPS3D_RET_OK, "Measure data init failed");
    TEST_ASSERT(data.full_depth_data.distance != NULL, "Distance data not initialized");
    
    ret = HPS3D_RegisterEventCallback(NULL, NULL);
    TEST_ASSERT(ret == HPS3D_RET_OK, "Event callback registration failed");
    
    HPS3D_MeasureDataFree(&data);
    TEST_SUCCESS();
}

// Test 2: USB connection
int test_usb_connection(void) {
    mock_reset_state();
    
    int handle;
    HPS3D_StatusTypeDef ret = HPS3D_USBConnectDevice("/dev/ttyACM0", &handle);
    TEST_ASSERT(ret == HPS3D_RET_OK, "USB connection failed");
    TEST_ASSERT(handle > 0, "Invalid handle returned");
    TEST_ASSERT(HPS3D_IsConnect(handle) == 1, "Connection not established");
    
    const char* version = HPS3D_GetDeviceVersion(handle);
    TEST_ASSERT(version != NULL, "Version string is NULL");
    TEST_ASSERT(strlen(version) > 0, "Version string is empty");
    
    ret = HPS3D_CloseDevice(handle);
    TEST_ASSERT(ret == HPS3D_RET_OK, "Device close failed");
    TEST_ASSERT(HPS3D_IsConnect(handle) == 0, "Connection not closed");
    
    TEST_SUCCESS();
}

// Test 3: Connection failure handling
int test_connection_failure(void) {
    mock_reset_state();
    mock_should_fail = 1;
    mock_fail_count = 1;
    
    int handle;
    HPS3D_StatusTypeDef ret = HPS3D_USBConnectDevice("/dev/ttyACM0", &handle);
    TEST_ASSERT(ret == HPS3D_RET_CONNECT_FAILED, "Expected connection failure");
    TEST_ASSERT(HPS3D_IsConnect(handle) == 0, "Should not be connected");
    
    TEST_SUCCESS();
}

// Test 4: Filter configuration
int test_filter_configuration(void) {
    mock_reset_state();
    
    int handle;
    HPS3D_USBConnectDevice("/dev/ttyACM0", &handle);
    
    HPS3D_StatusTypeDef ret = HPS3D_SetDistanceFilterConf(handle, 0, 0.1f);
    TEST_ASSERT(ret == HPS3D_RET_OK, "Distance filter config failed");
    
    ret = HPS3D_SetSmoothFilterConf(handle, HPS3D_SMOOTH_FILTER_DISABLE, 0);
    TEST_ASSERT(ret == HPS3D_RET_OK, "Smooth filter config failed");
    
    ret = HPS3D_SetEdgeFilterEnable(handle, 0);
    TEST_ASSERT(ret == HPS3D_RET_OK, "Edge filter config failed");
    
    ret = HPS3D_SetOpticalPathCalibration(handle, 1);
    TEST_ASSERT(ret == HPS3D_RET_OK, "Optical path calibration failed");
    
    HPS3D_CloseDevice(handle);
    TEST_SUCCESS();
}

// Test 5: Capture start/stop
int test_capture_control(void) {
    mock_reset_state();
    
    int handle;
    HPS3D_USBConnectDevice("/dev/ttyACM0", &handle);
    
    HPS3D_StatusTypeDef ret = HPS3D_StartCapture(handle);
    TEST_ASSERT(ret == HPS3D_RET_OK, "Start capture failed");
    TEST_ASSERT(mock_capture_active == 1, "Capture not active");
    
    ret = HPS3D_StopCapture(handle);
    TEST_ASSERT(ret == HPS3D_RET_OK, "Stop capture failed");
    TEST_ASSERT(mock_capture_active == 0, "Capture still active");
    
    HPS3D_CloseDevice(handle);
    TEST_SUCCESS();
}

// Test 6: Single measurement - valid data
int test_single_measurement_valid(void) {
    mock_reset_state();
    
    int handle;
    HPS3D_USBConnectDevice("/dev/ttyACM0", &handle);
    HPS3D_StartCapture(handle);
    
    // Generate valid test data for a point at (80, 30) with distance 1000mm
    mock_generate_valid_data(80, 30, 1000);
    
    HPS3D_EventType_t event_type;
    HPS3D_MeasureData_t data;
    HPS3D_MeasureDataInit(&data);
    
    HPS3D_StatusTypeDef ret = HPS3D_SingleCapture(handle, &event_type, &data);
    TEST_ASSERT(ret == HPS3D_RET_OK, "Single capture failed");
    TEST_ASSERT(event_type == HPS3D_FULL_DEPTH_EVEN, "Wrong event type");
    
    // Verify data at test point
    int idx = 30 * 160 + 80;
    uint16_t distance = data.full_depth_data.distance[idx];
    TEST_ASSERT(distance > 990 && distance < 1010, "Distance out of expected range");
    TEST_ASSERT(distance != HPS3D_INVALID_DATA, "Invalid data returned");
    TEST_ASSERT(distance != HPS3D_LOW_AMPLITUDE, "Low amplitude error");
    
    HPS3D_MeasureDataFree(&data);
    HPS3D_CloseDevice(handle);
    TEST_SUCCESS();
}

// Test 7: Single measurement - invalid data
int test_single_measurement_invalid(void) {
    mock_reset_state();
    
    int handle;
    HPS3D_USBConnectDevice("/dev/ttyACM0", &handle);
    HPS3D_StartCapture(handle);
    
    // Generate invalid test data for a point at (80, 30)
    mock_generate_invalid_data(80, 30);
    
    HPS3D_EventType_t event_type;
    HPS3D_MeasureData_t data;
    HPS3D_MeasureDataInit(&data);
    
    HPS3D_StatusTypeDef ret = HPS3D_SingleCapture(handle, &event_type, &data);
    TEST_ASSERT(ret == HPS3D_RET_OK, "Single capture failed");
    
    // Verify invalid data at test point
    int idx = 30 * 160 + 80;
    uint16_t distance = data.full_depth_data.distance[idx];
    TEST_ASSERT(distance == HPS3D_INVALID_DATA, "Expected invalid data");
    
    HPS3D_MeasureDataFree(&data);
    HPS3D_CloseDevice(handle);
    TEST_SUCCESS();
}

// Test 8: Measurement failure and retry
int test_measurement_failure_retry(void) {
    mock_reset_state();
    
    int handle;
    HPS3D_USBConnectDevice("/dev/ttyACM0", &handle);
    HPS3D_StartCapture(handle);
    
    // Setup to fail first 2 attempts, then succeed
    mock_should_fail = 1;
    mock_fail_count = 2;
    
    HPS3D_EventType_t event_type;
    HPS3D_MeasureData_t data;
    HPS3D_MeasureDataInit(&data);
    
    // First attempt should fail
    HPS3D_StatusTypeDef ret = HPS3D_SingleCapture(handle, &event_type, &data);
    TEST_ASSERT(ret == HPS3D_RET_READ_ERR, "Expected read error");
    
    // Second attempt should fail
    ret = HPS3D_SingleCapture(handle, &event_type, &data);
    TEST_ASSERT(ret == HPS3D_RET_READ_ERR, "Expected read error");
    
    // Third attempt should succeed
    ret = HPS3D_SingleCapture(handle, &event_type, &data);
    TEST_ASSERT(ret == HPS3D_RET_OK, "Third attempt should succeed");
    
    HPS3D_MeasureDataFree(&data);
    HPS3D_CloseDevice(handle);
    TEST_SUCCESS();
}

// Test 9: Point measurement algorithm
int test_point_measurement_algorithm(void) {
    mock_reset_state();
    
    int handle;
    HPS3D_USBConnectDevice("/dev/ttyACM0", &handle);
    HPS3D_StartCapture(handle);
    
    // Generate test data with known pattern
    // Point at (40, 30) with distance 2000mm
    mock_generate_valid_data(40, 30, 2000);
    
    HPS3D_EventType_t event_type;
    HPS3D_MeasureData_t data;
    HPS3D_MeasureDataInit(&data);
    
    HPS3D_StatusTypeDef ret = HPS3D_SingleCapture(handle, &event_type, &data);
    TEST_ASSERT(ret == HPS3D_RET_OK, "Capture failed");
    
    // Simulate point measurement algorithm (5x5 area around center)
    int center_x = 40, center_y = 30;
    float sum_distance = 0;
    int valid_count = 0;
    float min_distance = 65000;
    float max_distance = 0;
    
    // 5x5 area measurement
    for (int dy = -2; dy <= 2; dy++) {
        for (int dx = -2; dx <= 2; dx++) {
            int x = center_x + dx;
            int y = center_y + dy;
            int pixel_index = y * 160 + x;
            
            uint16_t distance_raw = data.full_depth_data.distance[pixel_index];
            
            if (distance_raw > 0 && distance_raw < 65000 && 
                distance_raw != HPS3D_LOW_AMPLITUDE && 
                distance_raw != HPS3D_SATURATION && 
                distance_raw != HPS3D_ADC_OVERFLOW && 
                distance_raw != HPS3D_INVALID_DATA) {
                
                sum_distance += distance_raw;
                valid_count++;
                
                if (distance_raw < min_distance) min_distance = distance_raw;
                if (distance_raw > max_distance) max_distance = distance_raw;
            }
        }
    }
    
    TEST_ASSERT(valid_count >= 6, "Not enough valid pixels"); // At least 25% of 25 pixels
    TEST_ASSERT(valid_count <= 25, "Too many valid pixels");
    
    float avg_distance = sum_distance / valid_count;
    TEST_ASSERT(avg_distance > 1980 && avg_distance < 2020, "Average distance out of range");
    TEST_ASSERT(min_distance > 1980, "Min distance too low");
    TEST_ASSERT(max_distance < 2020, "Max distance too high");
    
    HPS3D_MeasureDataFree(&data);
    HPS3D_CloseDevice(handle);
    TEST_SUCCESS();
}

// Test 10: Memory management
int test_memory_management(void) {
    mock_reset_state();
    
    // Test multiple init/free cycles
    for (int i = 0; i < 10; i++) {
        HPS3D_MeasureData_t data;
        HPS3D_StatusTypeDef ret = HPS3D_MeasureDataInit(&data);
        TEST_ASSERT(ret == HPS3D_RET_OK, "Measure data init failed in loop");
        TEST_ASSERT(data.full_depth_data.distance != NULL, "Distance data not initialized in loop");
        
        ret = HPS3D_MeasureDataFree(&data);
        TEST_ASSERT(ret == HPS3D_RET_OK, "Measure data free failed in loop");
        TEST_ASSERT(data.full_depth_data.distance == NULL, "Distance data not freed in loop");
    }
    
    TEST_SUCCESS();
}

// Test runner
int main(void) {
    printf("=== LIDAR Mock Interface Tests ===\n");
    
    int total_tests = 0;
    int passed_tests = 0;
    
    // Initialize random seed for mock data
    srand(time(NULL));
    
    // Initialize mock measurement data
    mock_measure_data.full_depth_data.distance = mock_distance_data;
    
    // Run tests
    total_tests++; passed_tests += test_lidar_init();
    total_tests++; passed_tests += test_usb_connection();
    total_tests++; passed_tests += test_connection_failure();
    total_tests++; passed_tests += test_filter_configuration();
    total_tests++; passed_tests += test_capture_control();
    total_tests++; passed_tests += test_single_measurement_valid();
    total_tests++; passed_tests += test_single_measurement_invalid();
    total_tests++; passed_tests += test_measurement_failure_retry();
    total_tests++; passed_tests += test_point_measurement_algorithm();
    total_tests++; passed_tests += test_memory_management();
    
    printf("\n=== Test Results ===\n");
    printf("Total tests: %d\n", total_tests);
    printf("Passed: %d\n", passed_tests);
    printf("Failed: %d\n", total_tests - passed_tests);
    printf("Success rate: %.1f%%\n", (float)passed_tests / total_tests * 100);
    
    return (passed_tests == total_tests) ? 0 : 1;
}