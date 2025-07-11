#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdarg.h>
#include <math.h>

#include "HPS3DUser_IF.h"


int g_handle = -1;
int m_handle[8] = {-1};
static HPS3D_MeasureData_t g_measureData;
static int FPS[8] = {0};

static void PrintResultFPS(int handle,HPS3D_EventType_t type, HPS3D_MeasureData_t data)
{
	switch (type)
	{
		case HPS3D_FULL_DEPTH_EVEN:
			FPS[handle]++;
			break;
		default:
			break;
	}
}

static bool PrintResultData(HPS3D_EventType_t type, HPS3D_MeasureData_t data)
{
	int num = 0;
	int i = 0;
	switch (type)
	{
	case HPS3D_SIMPLE_ROI_EVEN: //ROI data packet (does not include depth data for each point)
		printf("*************  HPS3D_SIMPLE_ROI_EVEN  ********************\n");
		num = data.simple_roi_data[0].roi_num;
		int i = 0;
		for (i = 0; i < num; i++)
		{
			printf("  ********GroupID:%d  ROIID:%d  *******\n", data.simple_roi_data[i].group_id, data.simple_roi_data[i].roi_id);
			printf("    distance_average:%d\n", data.simple_roi_data[i].distance_average);
			printf("    distance_min    :%d\n", data.simple_roi_data[i].distance_min);
			printf("    saturation_count:%d\n", data.simple_roi_data[i].saturation_count);
			printf("    threshold_state :%d\n", data.simple_roi_data[i].threshold_state);
			printf("    =====================================\n\n");
		}
		break;
	case HPS3D_FULL_ROI_EVEN: //Complete ROI data packet
		printf("*************  HPS3D_FULL_ROI_EVEN  ********************\n");
		num = data.full_roi_data[0].roi_num;
		for (i = 0; i < num; i++)
		{
			printf("  ********GroupID:%d  ROIID:%d  *******\n", data.full_roi_data[i].group_id, data.full_roi_data[i].roi_id);
			printf("    ROI Left Top    :(%d,%d)\n", data.full_roi_data[i].left_top_x, data.full_roi_data[i].left_top_y);
			printf("    ROI Right Bottom:(%d,%d)\n", data.full_roi_data[i].right_bottom_x, data.full_roi_data[i].right_bottom_y);
			printf("    ROI Pixel Number:%d\n", data.full_roi_data[i].pixel_number);
			printf("    distance_average:%d\n", data.full_roi_data[i].distance_average);
			printf("    distance_min    :%d\n", data.full_roi_data[i].distance_min);
			printf("    saturation_count:%d\n", data.full_roi_data[i].saturation_count);
			printf("    threshold_state :%d\n", data.full_roi_data[i].threshold_state);
			printf("    =====================================\n\n");
		}
		break;
	case HPS3D_SIMPLE_DEPTH_EVEN: //Simple depth data packet, does not include distance for each point or point cloud data
		printf("*************  HPS3D_SIMPLE_DEPTH_EVEN  ********************\n");
		printf(" distance_average:%d\n", data.simple_depth_data.distance_average);
		printf(" distance_min    :%d\n", data.simple_depth_data.distance_min);
		printf(" saturation_count:%d\n", data.simple_depth_data.saturation_count);
		printf("==========================================================\n\n");
		break;
	case HPS3D_FULL_DEPTH_EVEN: //Complete depth map data packet, includes point cloud data
		printf("*************  HPS3D_FULL_DEPTH_EVEN    ********************\n");
		printf("distance_average:%d\n", data.full_depth_data.distance_average);
		printf("distance_min    :%d\n", data.full_depth_data.distance_min);
		printf("saturation_count:%d\n", data.full_depth_data.saturation_count);
		printf("points_count:%d\n", data.full_depth_data.point_cloud_data.points);
		printf("width:%d\n", data.full_depth_data.point_cloud_data.width);
		printf("height:%d\n", data.full_depth_data.point_cloud_data.height);

		printf("distance[0]     :%d\n", data.full_depth_data.distance[0]);
		printf("pointCloud[0]   :(%f,%f.%f)\n", data.full_depth_data.point_cloud_data.point_data[0].x,
			data.full_depth_data.point_cloud_data.point_data[0].y, data.full_depth_data.point_cloud_data.point_data[0].z);

		printf("distance[1]     :%d\n", data.full_depth_data.distance[1]);
		printf("pointCloud[1]   :(%f,%f.%f)\n", data.full_depth_data.point_cloud_data.point_data[1].x,
			data.full_depth_data.point_cloud_data.point_data[1].y, data.full_depth_data.point_cloud_data.point_data[1].z);

		printf("==========================================================\n\n");
		break;
	default:
		break;
	}

	return true;
}

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
            PrintResultData((HPS3D_EventType_t)eventType, g_measureData);
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

int main()
{
printf("HPS3D160 C/C++ Demo (Visual Statudio 2017)\n\n");

printf("SDK Ver:%s\n", HPS3D_GetSDKVersion());

int handle = -1;
HPS3D_StatusTypeDef ret = HPS3D_RET_OK;
//Capture Terminal \"Ctrl+C\" Signal (capture terminal signal)
signal(SIGINT, signal_handler); 
do
{
    //Initialize memory
    ret = HPS3D_MeasureDataInit(&g_measureData);
    if (ret != HPS3D_RET_OK)
    {
        printf("MeasureDataInit failed,Err:%d\n", ret);
        break;
    }
/*     printf("Select device type: HPS3D160-U(1)  HPS3D160-L(2)  Exit(other)\n");
    char sel = getchar();
    getchar(); //filter enter
    if (sel == '1')
    { */
        ret = HPS3D_USBConnectDevice((char *)"/dev/ttyACM0",&g_handle);
   /*  }
    else if (sel == '2')
    {
        ret = HPS3D_EthernetConnectDevice((char *)"192.168.0.10", 12345, &g_handle);
    }
    else
    {
        return 0;
    } */
    if (ret != HPS3D_RET_OK)
    {
        printf("Device connection failed,Err:%d\n", ret);
        break;
    }
    printf("Device version: %s\n", HPS3D_GetDeviceVersion(0));
    printf("Device serial number: %s\n\n", HPS3D_GetSerialNumber(0));
    if (g_handle == 1)
    {
        printf("Device version: %s\n", HPS3D_GetDeviceVersion(1));
        printf("Device serial number: %s\n\n", HPS3D_GetSerialNumber(1));
    }

		// Register event callback function to receive continuous return packets and handle exceptions;
    ret= HPS3D_RegisterEventCallback(EventCallBackFunc, NULL);
    if (ret != HPS3D_RET_OK)
    {
        printf("Failed to register callback function, Err:%d\n", ret);
        break;

    }

    HPS3D_DeviceSettings_t settings;
    ret = HPS3D_ExportSettings(g_handle, &settings);
    if (ret != HPS3D_RET_OK)
    {
        printf("Failed to export device parameters, Err:%d\n", ret);
        break;
    }
    printf("Resolution: %d X %d\n", settings.max_resolution_X, settings.max_resolution_Y);
    printf("Max supported ROI group number: %d  Current ROI group: %d\n", settings.max_roi_group_number, settings.cur_group_id);
    printf("Max supported ROI number: %d\n", settings.max_roi_number);
    printf("Max supported multi-device code: %d, Current device multi-device code: %d\n", settings.max_multiCamera_code, settings.cur_multiCamera_code);
    printf("Current device user ID: %d\n", settings.user_id);
    printf("Optical path compensation enabled: %d\n\n", settings.optical_path_calibration);

    bool isContinuous = false;
    int count = 0;

    do
    {
        printf("select capture mode: SingleCapture(1)  ContinuousCapture(2)  Exit(...)\n");
        int sel = 0;
        scanf("%d", &sel);
        if (sel == 1)
        {
            HPS3D_EventType_t type = HPS3D_NULL_EVEN;
            ret = HPS3D_SingleCapture(g_handle, &type, &g_measureData);
            if (ret != HPS3D_RET_OK)
            {
                printf("SingleCapture failed,Err:%d\n", ret);
                break;
            }
            PrintResultData(type, g_measureData);
        }
        else if (sel == 2)
        {
            isContinuous = true;
            ret = HPS3D_StartCapture(g_handle);
            if (ret != HPS3D_RET_OK)
            {
                printf("Continuous Capture failed,Err:%d\n", ret);
                break;
            }
        }
        else if (sel == 3)
        {
            ret = HPS3D_EthernetConnectDevice((char *)"192.168.0.10", 12345, &g_handle);
        }
        else
        {
            HPS3D_CloseDevice(g_handle);
        }


        printf("handle: %d\n", g_handle);
    } while (1);

//Test the Reconnect
//		HPS3D_StartCapture(g_handle);

//		do
//		{
//			if (isReconnect)
//			{
//				//sleep(5);
//				ret = (HPS3D_StatusTypeDef)HPS3D_EthternetReconnection(g_handle);
//				if (ret == HPS3D_RET_OK)
//				{
//					HPS3D_StartCapture(g_handle);
//					printf("Reconnect successful: device %d\n", g_handle);
//					isReconnect = false;
//				}
//			}
//			sleep(1);
                        
//		} while (1);


} while (0);

HPS3D_StopCapture(g_handle);
HPS3D_CloseDevice(g_handle);
HPS3D_MeasureDataFree(&g_measureData);
system("pause");
}



