/*****************************************************************************
 * @file   taskDataAcquisition.c
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * sensor data acquisition task runs at 100Hz, gets the data for each sensor
 * and applies available calibration
 ******************************************************************************/
/*******************************************************************************
Copyright 2018 ACEINNA, INC

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*******************************************************************************/
#include "stm32f4xx_hal.h"
#include "osapi.h"
#include "osresources.h"

#include "userAPI.h"
// #include "sensorsAPI.h"
#include "gpsAPI.h"
#include "Indices.h" // For X_AXIS and Z_AXIS
// #include "platformAPI.h"
#include "osapi.h"
#include "GlobalConstants.h"
#include <string.h>
#include "ppprtk.h"
#include "EKF_Algorithm.h"
#include "TransformationMath.h"
// Local-function prototypes
extern GpsData_t *gCloudGpsDataPtr;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern void RTKAlgorithm();
extern void copy_rtk_pvt_to_ins(rcv_rtk_t *rtk);

gpsDataStruct_t gGPS;
rcv_rtk_t gRTK; /* store the RTK solution */

void RTKTask(void const *argument)
{

    static int xx = 0;
    while (1)
    {
        xx++;
        if (xx == 5)
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
            //HAL_UART_Transmit_DMA(&huart2, bt_trans_test0, strlen(bt_trans_test0));
        }
        else if (xx == 10)
        {
            xx = 0;
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
            //HAL_UART_Transmit_DMA(&huart2, bt_trans_test1, strlen(bt_trans_test1));
        }
        RTKAlgorithm();
        OS_Delay(10);
    }
}

static char ggaBuff[120] = {0};
void RTKAlgorithm(void)
{
    if (is_gnss_data_available())
    {
        reset_gnss_data_available();
        get_gnss_rtcm_data(&gGPS);
        obs_t *rov = (obs_t *)&gGPS.gnss_rtcm->obs;
#ifndef _USE_PPP_
        // obs_t *ref = &gGPS.gnss_rtcm->obs_ref;
        obs_t *ref = (obs_t *)&gCloudGpsDataPtr->rtcm.obs_ref;
#endif
        nav_t *nav = &gGPS.gnss_rtcm->nav;

#ifndef _USE_PPP_
        {
            //char ggaBuff[120] = {0};
            //memset(ggaBuff,0,sizeof ggaBuff);
            if (rtk_processor(rov, ref, nav, &gRTK, ggaBuff, NULL, 0) > 0)
            {
                copy_rtk_pvt_to_ins(&gRTK);
                HAL_UART_Transmit_DMA(&huart5, (uint8_t *)ggaBuff, strlen((const char *)ggaBuff));
#ifndef RTCM_UART2
#ifndef COLLECT_DATA
                // obs_t *rov = (obs_t *)&gGPS.gnss_rtcm->obs;
                // static char ggaBuff[120] = {0};
                // double ecef_m[3];
                // double llaDeg[3];
                // int num = 0;
                // if(gEKFOutputData.llaDeg[0] != 0)
                // {
                //     llaDeg[0] = gEKFOutputData.llaDeg[0]*D2R;
                //     llaDeg[1] = gEKFOutputData.llaDeg[1]*D2R;
                //     llaDeg[2] = gEKFOutputData.llaDeg[2];
                //     LLA_To_ECEF(llaDeg,ecef_m);
                //     num = print_nmea_gga(rov->time, ecef_m, rov->n, 5, 1.0, 0.0, (char *)ggaBuff);
                //     HAL_UART_Transmit_DMA(&huart1, (uint8_t *)ggaBuff, num);
                // }

                HAL_UART_Transmit_DMA(&huart2, (uint8_t *)ggaBuff, strlen((const char *)ggaBuff));
#endif
#endif
                OS_Delay(4);
            }
            else
            {
                print_nmea_gga(rov->time, rov->pos, rov->n, 1, 1.0, 0.0, (char *)ggaBuff);
                HAL_UART_Transmit_DMA(&huart5, (uint8_t *)ggaBuff, strlen((const char *)ggaBuff));
#ifndef RTCM_UART2
#ifndef COLLECT_DATA
                HAL_UART_Transmit_DMA(&huart2, (uint8_t *)ggaBuff, strlen((const char *)ggaBuff));
#endif
#endif
                OS_Delay(4);
            }
        }
#else
        {
            if (ppp_processor(rov, nav, &gPPP, 0) > 0)
            {
                copy_ppp_pvt_to_ins(&gPPP);
            }
        }
#endif
    }
}

// static void set_gnss_pvt_for_ins(obs_t *obs)
// {
//     /* need to get the solution from a solution data struct instead of directly from the observation */
//     /* to do list */
//     int w = 0;
//     double tow = time2gpst(obs->time, &w);
//     gGPS.gpsValid = 1;
//     gGPS.updateFlag = 1;
//     gGPS.latitude = obs->pos[0];
//     gGPS.longitude = obs->pos[1];
//     gGPS.altitude = obs->pos[2];
// 	gGPS.vNed[0] = 0.01;
//     gGPS.vNed[1] = -0.02;
//     gGPS.vNed[2] = 0.03;
//     gGPS.itow = (uint32_t)(tow*1000);

//     gGPS.HDOP = 1.5;
//     gGPS.GPSHorizAcc = 1.8;
//     gGPS.GPSVertAcc = 3.2;
// }

void copy_rtk_pvt_to_ins(rcv_rtk_t *rtk)
{
    /* need to get the solution from a solution data struct instead of directly from the observation */
    /* to do list */
    double blh[3] = {0.0};
    ecef2pos(rtk->x, blh);
    gGPS.gpsValid = 1;
    gGPS.updateFlag = 1;
    gGPS.latitude = blh[0] * 180.0 / PI;
    gGPS.longitude = blh[1] * 180.0 / PI;
    gGPS.altitude = blh[2];
    gGPS.vNed[0] = 0.01;
    gGPS.vNed[1] = -0.02;
    gGPS.vNed[2] = 0.03;
    gGPS.itow = (uint32_t)(rtk->time * 1000);

    gGPS.HDOP = 1.5;
    gGPS.GPSHorizAcc = 1.8;
    gGPS.GPSVertAcc = 3.2;
}

// static void copy_ppp_pvt_to_ins(rcv_ppp_t *ppp)
// {
//     /* need to get the solution from a solution data struct instead of directly from the observation */
//     /* to do list */
//     double blh[3] = { 0.0 };
//     ecef2pos(ppp->x, blh);
//     gGPS.gpsValid = 1;
//     gGPS.updateFlag = 1;
//     gGPS.latitude = blh[0]*180.0/PI;
//     gGPS.longitude = blh[1]*180.0/PI;
//     gGPS.altitude = blh[2];
// 	gGPS.vNed[0] = 0.01;
//     gGPS.vNed[1] = -0.02;
//     gGPS.vNed[2] = 0.03;
//     gGPS.itow = (uint32_t)(ppp->time*1000);

//     gGPS.HDOP = 1.5;
//     gGPS.GPSHorizAcc = 1.8;
//     gGPS.GPSVertAcc = 3.2;
// }