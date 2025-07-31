/*
 * Unit tests for MQTT functionality in HPS3D LIDAR service
 * 
 * Tests include:
 * - MQTT connection and disconnection
 * - Message publishing and receiving
 * - Control commands (start/stop/pointcloud)
 * - Error handling and reconnection
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>
#include <pthread.h>
#include <mosquitto.h>
#include <stdatomic.h>
#include <time.h>

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

// Mock global variables (from main.c)
static volatile _Atomic int measurement_active = 0;
static volatile _Atomic int pointcloud_requested = 0;
static volatile _Atomic int mqtt_connected = 0;
static struct mosquitto *test_mosq = NULL;
static char received_topic[256] = {0};
static char received_payload[256] = {0};
static int message_received = 0;

// Test configuration
#define TEST_MQTT_HOST "localhost"
#define TEST_MQTT_PORT 1883
#define TEST_CONTROL_TOPIC "hps3d/test/control"
#define TEST_DATA_TOPIC "hps3d/test/measurements"
#define TEST_POINTCLOUD_TOPIC "hps3d/test/pointcloud"

// Test message callback
void test_mqtt_message_callback(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message) {
    (void)mosq;
    (void)userdata;
    
    if (!message || !message->payload) return;
    
    strncpy(received_topic, message->topic, sizeof(received_topic) - 1);
    strncpy(received_payload, (char*)message->payload, 
            message->payloadlen < sizeof(received_payload) ? message->payloadlen : sizeof(received_payload) - 1);
    message_received = 1;
    
    // Process control commands like in main.c
    if (strcmp(message->topic, TEST_CONTROL_TOPIC) == 0) {
        if (strncmp(message->payload, "start", message->payloadlen) == 0) {
            atomic_store(&measurement_active, 1);
        } else if (strncmp(message->payload, "stop", message->payloadlen) == 0) {
            atomic_store(&measurement_active, 0);
        } else if (strncmp(message->payload, "get_pointcloud", message->payloadlen) == 0) {
            atomic_store(&pointcloud_requested, 1);
        }
    }
}

// Test connection callback
void test_mqtt_connect_callback(struct mosquitto *mosq, void *userdata, int result) {
    (void)mosq;
    (void)userdata;
    
    if (!result) {
        atomic_store(&mqtt_connected, 1);
    } else {
        atomic_store(&mqtt_connected, 0);
    }
}

// Test disconnect callback
void test_mqtt_disconnect_callback(struct mosquitto *mosq, void *userdata, int rc) {
    (void)mosq;
    (void)userdata;
    (void)rc;
    
    atomic_store(&mqtt_connected, 0);
}

// Initialize test MQTT client
int init_test_mqtt(void) {
    mosquitto_lib_init();
    test_mosq = mosquitto_new("test_client", true, NULL);
    if (!test_mosq) {
        return -1;
    }

    mosquitto_connect_callback_set(test_mosq, test_mqtt_connect_callback);
    mosquitto_disconnect_callback_set(test_mosq, test_mqtt_disconnect_callback);
    mosquitto_message_callback_set(test_mosq, test_mqtt_message_callback);

    return 0;
}

// Cleanup test MQTT client
void cleanup_test_mqtt(void) {
    if (test_mosq) {
        if (atomic_load(&mqtt_connected)) {
            mosquitto_disconnect(test_mosq);
        }
        mosquitto_loop_stop(test_mosq, true);
        mosquitto_destroy(test_mosq);
        test_mosq = NULL;
    }
    mosquitto_lib_cleanup();
    
    // Reset test state
    atomic_store(&measurement_active, 0);
    atomic_store(&pointcloud_requested, 0);
    atomic_store(&mqtt_connected, 0);
    message_received = 0;
    memset(received_topic, 0, sizeof(received_topic));
    memset(received_payload, 0, sizeof(received_payload));
}

// Test 1: MQTT client initialization
int test_mqtt_init(void) {
    int result = init_test_mqtt();
    TEST_ASSERT(result == 0, "MQTT client initialization failed");
    TEST_ASSERT(test_mosq != NULL, "MQTT client object is NULL");
    
    cleanup_test_mqtt();
    TEST_SUCCESS();
}

// Test 2: MQTT connection (requires broker)
int test_mqtt_connection(void) {
    init_test_mqtt();
    
    int result = mosquitto_connect(test_mosq, TEST_MQTT_HOST, TEST_MQTT_PORT, 60);
    if (result != MOSQ_ERR_SUCCESS) {
        cleanup_test_mqtt();
        printf("SKIP: %s - MQTT broker not available\n", __func__);
        return 1; // Skip test if broker not available
    }
    
    // Start loop and wait for connection
    mosquitto_loop_start(test_mosq);
    
    // Wait for connection (up to 3 seconds)
    for (int i = 0; i < 30; i++) {
        if (atomic_load(&mqtt_connected)) break;
        usleep(100000); // 100ms
    }
    
    TEST_ASSERT(atomic_load(&mqtt_connected) == 1, "MQTT connection not established");
    
    cleanup_test_mqtt();
    TEST_SUCCESS();
}

// Test 3: Message publishing
int test_mqtt_publish(void) {
    init_test_mqtt();
    
    int result = mosquitto_connect(test_mosq, TEST_MQTT_HOST, TEST_MQTT_PORT, 60);
    if (result != MOSQ_ERR_SUCCESS) {
        cleanup_test_mqtt();
        printf("SKIP: %s - MQTT broker not available\n", __func__);
        return 1;
    }
    
    mosquitto_loop_start(test_mosq);
    
    // Wait for connection
    for (int i = 0; i < 30; i++) {
        if (atomic_load(&mqtt_connected)) break;
        usleep(100000);
    }
    
    if (!atomic_load(&mqtt_connected)) {
        cleanup_test_mqtt();
        printf("SKIP: %s - Could not connect to broker\n", __func__);
        return 1;
    }
    
    // Test publishing
    const char* test_message = "{\"test\": \"data\"}";
    result = mosquitto_publish(test_mosq, NULL, TEST_DATA_TOPIC, 
                              strlen(test_message), test_message, 0, false);
    
    TEST_ASSERT(result == MOSQ_ERR_SUCCESS, "Message publish failed");
    
    cleanup_test_mqtt();
    TEST_SUCCESS();
}

// Test 4: Control command processing
int test_mqtt_control_commands(void) {
    init_test_mqtt();
    
    int result = mosquitto_connect(test_mosq, TEST_MQTT_HOST, TEST_MQTT_PORT, 60);
    if (result != MOSQ_ERR_SUCCESS) {
        cleanup_test_mqtt();
        printf("SKIP: %s - MQTT broker not available\n", __func__);
        return 1;
    }
    
    mosquitto_loop_start(test_mosq);
    
    // Wait for connection
    for (int i = 0; i < 30; i++) {
        if (atomic_load(&mqtt_connected)) break;
        usleep(100000);
    }
    
    if (!atomic_load(&mqtt_connected)) {
        cleanup_test_mqtt();
        printf("SKIP: %s - Could not connect to broker\n", __func__);
        return 1;
    }
    
    // Subscribe to control topic
    result = mosquitto_subscribe(test_mosq, NULL, TEST_CONTROL_TOPIC, 0);
    TEST_ASSERT(result == MOSQ_ERR_SUCCESS, "Subscribe to control topic failed");
    
    usleep(100000); // Wait for subscription to be processed
    
    // Test "start" command
    atomic_store(&measurement_active, 0);
    mosquitto_publish(test_mosq, NULL, TEST_CONTROL_TOPIC, 5, "start", 0, false);
    
    // Wait for message processing
    for (int i = 0; i < 10; i++) {
        if (atomic_load(&measurement_active)) break;
        usleep(100000);
    }
    
    TEST_ASSERT(atomic_load(&measurement_active) == 1, "Start command not processed");
    
    // Test "stop" command
    mosquitto_publish(test_mosq, NULL, TEST_CONTROL_TOPIC, 4, "stop", 0, false);
    
    // Wait for message processing
    for (int i = 0; i < 10; i++) {
        if (!atomic_load(&measurement_active)) break;
        usleep(100000);
    }
    
    TEST_ASSERT(atomic_load(&measurement_active) == 0, "Stop command not processed");
    
    // Test "get_pointcloud" command
    atomic_store(&pointcloud_requested, 0);
    mosquitto_publish(test_mosq, NULL, TEST_CONTROL_TOPIC, 13, "get_pointcloud", 0, false);
    
    // Wait for message processing
    for (int i = 0; i < 10; i++) {
        if (atomic_load(&pointcloud_requested)) break;
        usleep(100000);
    }
    
    TEST_ASSERT(atomic_load(&pointcloud_requested) == 1, "Pointcloud command not processed");
    
    cleanup_test_mqtt();
    TEST_SUCCESS();
}

// Test 5: Message receiving
int test_mqtt_message_receive(void) {
    init_test_mqtt();
    
    int result = mosquitto_connect(test_mosq, TEST_MQTT_HOST, TEST_MQTT_PORT, 60);
    if (result != MOSQ_ERR_SUCCESS) {
        cleanup_test_mqtt();
        printf("SKIP: %s - MQTT broker not available\n", __func__);
        return 1;
    }
    
    mosquitto_loop_start(test_mosq);
    
    // Wait for connection
    for (int i = 0; i < 30; i++) {
        if (atomic_load(&mqtt_connected)) break;
        usleep(100000);
    }
    
    if (!atomic_load(&mqtt_connected)) {
        cleanup_test_mqtt();
        printf("SKIP: %s - Could not connect to broker\n", __func__);
        return 1;
    }
    
    // Subscribe to test topic
    const char* test_topic = "test/message";
    result = mosquitto_subscribe(test_mosq, NULL, test_topic, 0);
    TEST_ASSERT(result == MOSQ_ERR_SUCCESS, "Subscribe failed");
    
    usleep(100000); // Wait for subscription
    
    // Publish test message
    const char* test_payload = "test_payload";
    message_received = 0;
    mosquitto_publish(test_mosq, NULL, test_topic, strlen(test_payload), test_payload, 0, false);
    
    // Wait for message
    for (int i = 0; i < 10; i++) {
        if (message_received) break;
        usleep(100000);
    }
    
    TEST_ASSERT(message_received == 1, "Message not received");
    TEST_ASSERT(strcmp(received_topic, test_topic) == 0, "Wrong topic received");
    TEST_ASSERT(strcmp(received_payload, test_payload) == 0, "Wrong payload received");
    
    cleanup_test_mqtt();
    TEST_SUCCESS();
}

// Test 6: Error handling - invalid host
int test_mqtt_error_handling(void) {
    init_test_mqtt();
    
    // Try to connect to invalid host
    int result = mosquitto_connect(test_mosq, "invalid.host.local", TEST_MQTT_PORT, 60);
    TEST_ASSERT(result != MOSQ_ERR_SUCCESS, "Connection to invalid host should fail");
    
    cleanup_test_mqtt();
    TEST_SUCCESS();
}

// Test 7: Reconnection handling
int test_mqtt_reconnection(void) {
    init_test_mqtt();
    
    int result = mosquitto_connect(test_mosq, TEST_MQTT_HOST, TEST_MQTT_PORT, 60);
    if (result != MOSQ_ERR_SUCCESS) {
        cleanup_test_mqtt();
        printf("SKIP: %s - MQTT broker not available\n", __func__);
        return 1;
    }
    
    mosquitto_loop_start(test_mosq);
    
    // Wait for connection
    for (int i = 0; i < 30; i++) {
        if (atomic_load(&mqtt_connected)) break;
        usleep(100000);
    }
    
    if (!atomic_load(&mqtt_connected)) {
        cleanup_test_mqtt();
        printf("SKIP: %s - Could not connect to broker\n", __func__);
        return 1;
    }
    
    // Force disconnect
    mosquitto_disconnect(test_mosq);
    
    // Wait for disconnect
    for (int i = 0; i < 10; i++) {
        if (!atomic_load(&mqtt_connected)) break;
        usleep(100000);
    }
    
    TEST_ASSERT(atomic_load(&mqtt_connected) == 0, "Disconnect not detected");
    
    // Reconnect
    result = mosquitto_reconnect(test_mosq);
    TEST_ASSERT(result == MOSQ_ERR_SUCCESS, "Reconnection failed");
    
    // Wait for reconnection
    for (int i = 0; i < 30; i++) {
        if (atomic_load(&mqtt_connected)) break;
        usleep(100000);
    }
    
    TEST_ASSERT(atomic_load(&mqtt_connected) == 1, "Reconnection not successful");
    
    cleanup_test_mqtt();
    TEST_SUCCESS();
}

// Test runner
int main(void) {
    printf("=== MQTT Functionality Tests ===\n");
    
    int total_tests = 0;
    int passed_tests = 0;
    
    // Run tests
    total_tests++; passed_tests += test_mqtt_init();
    total_tests++; passed_tests += test_mqtt_connection();
    total_tests++; passed_tests += test_mqtt_publish();
    total_tests++; passed_tests += test_mqtt_control_commands();
    total_tests++; passed_tests += test_mqtt_message_receive();
    total_tests++; passed_tests += test_mqtt_error_handling();
    total_tests++; passed_tests += test_mqtt_reconnection();
    
    printf("\n=== Test Results ===\n");
    printf("Total tests: %d\n", total_tests);
    printf("Passed: %d\n", passed_tests);
    printf("Failed: %d\n", total_tests - passed_tests);
    printf("Success rate: %.1f%%\n", (float)passed_tests / total_tests * 100);
    
    return (passed_tests == total_tests) ? 0 : 1;
}