#ifndef HPS3D_USER_IF_H
#define HPS3D_USER_IF_H

#include <stdint.h>
#include <stdbool.h>

// Define the maximum number of pixels
#define MAX_PIXELS 640

// Define event types for the HPS3D sensor
typedef enum {
    HPS3D_NULL_EVEN,
    HPS3D_SIMPLE_ROI_EVEN,
    HPS3D_FULL_ROI_EVEN,
    HPS3D_SIMPLE_DEPTH_EVEN,
    HPS3D_FULL_DEPTH_EVEN,
    HPS3D_SYS_EXCEPTION_EVEN,
    HPS3D_DISCONNECT_EVEN
} HPS3D_EventType_t;

// Structure to hold measurement data from the HPS3D sensor
typedef struct {
    int distance_average;
    int distance_min;
    int saturation_count;
    // Additional fields can be added as needed
} HPS3D_MeasureData_t;

// Function prototypes
HPS3D_StatusTypeDef HPS3D_USBConnectDevice(const char *devicePath, int *handle);
HPS3D_StatusTypeDef HPS3D_EthernetConnectDevice(const char *ipAddress, int port, int *handle);
HPS3D_StatusTypeDef HPS3D_StartCapture(int handle);
HPS3D_StatusTypeDef HPS3D_StopCapture(int handle);
HPS3D_StatusTypeDef HPS3D_CloseDevice(int handle);
HPS3D_StatusTypeDef HPS3D_RegisterEventCallback(void (*callback)(int, int, uint8_t*, int, void*), void *userPara);
HPS3D_StatusTypeDef HPS3D_ConvertToMeasureData(uint8_t *data, HPS3D_MeasureData_t *measureData, HPS3D_EventType_t eventType);
HPS3D_StatusTypeDef HPS3D_MeasureDataInit(HPS3D_MeasureData_t *measureData);
void HPS3D_MeasureDataFree(HPS3D_MeasureData_t *measureData);
const char* HPS3D_GetSDKVersion(void);
const char* HPS3D_GetDeviceVersion(int handle);
const char* HPS3D_GetSerialNumber(int handle);
HPS3D_StatusTypeDef HPS3D_ExportSettings(int handle, HPS3D_DeviceSettings_t *settings);

#endif // HPS3D_USER_IF_H