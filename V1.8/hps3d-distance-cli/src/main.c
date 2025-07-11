#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include "HPS3DUser_IF.h"

static int g_handle = -1;
static HPS3D_MeasureData_t g_measureData;

void signal_handler(int sig) {
    (void)sig; // suppress unused parameter warning
    HPS3D_StopCapture(g_handle);
    HPS3D_CloseDevice(g_handle);
    exit(0);
}

void query_distance(int pixel_x, int pixel_y) {
    HPS3D_EventType_t event_type;
    HPS3D_StatusTypeDef ret;

    ret = HPS3D_SingleCapture(g_handle, &event_type, &g_measureData);
    if (ret != HPS3D_RET_OK) {
        printf("SingleCapture failed, Err:%d\n", ret);
        return;
    }

    if (event_type == HPS3D_FULL_DEPTH_EVEN) {
        int width = g_measureData.full_depth_data.point_cloud_data.width;
        int height = g_measureData.full_depth_data.point_cloud_data.height;
        if (pixel_x < 0 || pixel_x >= width || pixel_y < 0 || pixel_y >= height) {
            printf("Pixel coordinates out of range! (width: %d, height: %d)\n", width, height);
            return;
        }
        int distance = g_measureData.full_depth_data.distance[pixel_y * width + pixel_x];
        printf("Distance at pixel (%d, %d): %d\n", pixel_x, pixel_y, distance);
    } else {
        printf("No valid depth data available.\n");
    }
}

int main(int argc, char *argv[]) {
    if (argc != 3) {
        printf("Usage: %s <pixel_x> <pixel_y>\n", argv[0]);
        return 1;
    }

    int pixel_x = atoi(argv[1]);
    int pixel_y = atoi(argv[2]);

    signal(SIGINT, signal_handler);

    // Initialisiere Messdatenstruktur, falls nötig
    HPS3D_MeasureDataInit(&g_measureData);

    HPS3D_StatusTypeDef ret = HPS3D_USBConnectDevice((char *)"/dev/ttyACM0", &g_handle);
    if (ret != HPS3D_RET_OK) {
        printf("Device connection failed, Err:%d\n", ret);
        return -1;
    }

    query_distance(pixel_x, pixel_y);

    HPS3D_StopCapture(g_handle);
    HPS3D_CloseDevice(g_handle);
    return 0;
}