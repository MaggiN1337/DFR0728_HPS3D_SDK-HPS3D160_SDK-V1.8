#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <stdbool.h>
#include <string.h>
#include "HPS3DUser_IF.h"

static int g_handle = -1;
static HPS3D_MeasureData_t g_measureData;

static bool isReconnectEnable = true;
static bool isReconnect = false;
static void EventCallBackFunc(int handle, int eventType, uint8_t *data,int dataLen, void *userPara)
{
    switch ((HPS3D_EventType_t)eventType)
    {
        // Measurement data notification event
        case HPS3D_SIMPLE_ROI_EVEN:	
        case HPS3D_FULL_ROI_EVEN:	
        case HPS3D_FULL_DEPTH_EVEN:
        case HPS3D_SIMPLE_DEPTH_EVEN:
            printf("handle:%d!\n", handle);
            HPS3D_ConvertToMeasureData(data,&g_measureData, (HPS3D_EventType_t)eventType);
            break;
        case HPS3D_SYS_EXCEPTION_EVEN: /* System exception notification event */
            printf("SYS ERR :%s\n", data);

            break;
        case HPS3D_DISCONNECT_EVEN: /* Connection abnormal disconnection notification event */
            printf("Device disconnected!\n");
            //sleep(10);
            //HPS3D_StopCapture(handle);
            //sleep(10);
            if(isReconnectEnable && isReconnect == false)
            {
                isReconnect = true;
            }
            break;
        case HPS3D_NULL_EVEN:  // Null event notification
        default:
            break;	
    }
}

void signal_handler(int sig)
{
    char c;
    signal(sig, SIG_IGN);
    HPS3D_StopCapture(g_handle);
    printf("Caught Ctrl-C\nAre you sure you want to exit? [y/n] ");	
    c = getchar();
        if (c == 'y' || c == 'Y')
    {
        HPS3D_CloseDevice(g_handle);
            exit(0);
    }
        else
    {
        signal(SIGINT, signal_handler);
        HPS3D_StartCapture(g_handle);
    }
        getchar();

    //HPS3D_StopCapture(g_handle);
    //HPS3D_CloseDevice(g_handle);
    //printf("Device disconnected!\n\n");
    //exit(0);
}

void query_distance(int pixel_x, int pixel_y) {
    HPS3D_EventType_t event_type;
    HPS3D_StatusTypeDef ret;

    ret = HPS3D_SingleCapture(g_handle, &event_type, &g_measureData);
    printf("SingleCapture ret=%d, event_type=%d\n", ret, event_type);

    if (ret != HPS3D_RET_OK) {
        printf("SingleCapture failed, Err:%d\n", ret);
        return;
    }

    if (event_type == HPS3D_FULL_DEPTH_EVEN) {
        int width = g_measureData.full_depth_data.point_cloud_data.width;
        int height = g_measureData.full_depth_data.point_cloud_data.height;
        //debug
        printf("width=%d, height=%d\n", width, height);
        if (pixel_x < 0 || pixel_x >= width || pixel_y < 0 || pixel_y >= height) {
            printf("Pixel coordinates out of range! (width: %d, height: %d)\n", width, height);
            return;
        }
        int distance = g_measureData.full_depth_data.distance[pixel_y * width + pixel_x];
        printf("Distance at pixel (%d, %d): %d\n", pixel_x, pixel_y, distance);
    } else {
        printf("No valid depth data available. Event type: %d\n", event_type);
    }
}

int main(int argc, char *argv[]) {
    if (argc != 3) {
        printf("Usage: %s <pixel_x> <pixel_y>\n", argv[0]);
        return 1;
    }

    int pixel_x = atoi(argv[1]);
    int pixel_y = atoi(argv[2]);

    //debug
    printf("pixel_x=%d, pixel_y=%d\n", pixel_x, pixel_y);

    HPS3D_StatusTypeDef ret = HPS3D_RET_OK;

    signal(SIGINT, signal_handler);

    // Initialisiere Messdatenstruktur
    HPS3D_MeasureDataInit(&g_measureData);

    ret = HPS3D_USBConnectDevice((char *)"/dev/ttyACM0", &g_handle);
    if (ret != HPS3D_RET_OK) {
        printf("Device connection failed, Err:%d\n", ret);
        return -1;
    }

    ret = HPS3D_RegisterEventCallback(EventCallBackFunc, NULL);
    if (ret != HPS3D_RET_OK)
    {
        printf("Failed to register callback function, Err:%d\n", ret);
        HPS3D_CloseDevice(g_handle);
        return -1;
    }

    query_distance(pixel_x, pixel_y);

    HPS3D_StopCapture(g_handle);
    HPS3D_CloseDevice(g_handle);
    //HPS3D_MeasureDataFree(&g_measureData);
    printf("Device disconnected!\n\n");
    return 0;
}