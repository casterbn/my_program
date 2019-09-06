/** ***************************************************************************
 * @file taskGps.c handle GPS data, make sure the GPS handling function gets
 *  called on a regular basis
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *****************************************************************************/
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

#include <stdint.h>
#include <stddef.h>
#include <string.h>

//#include "driverGPS.h"
#include "stm32f4xx_hal.h"
#include "osapi.h"
// #include "platformAPI.h"
// #include "algorithmAPI.h"
#include "gpsAPI.h"
#include "RingBuffer.h"


int gpsIdleCnt = 0;

extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart5_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
// extern uint8_t GpsRxBuff[GPS_BUFF_SIZE * 4];
// extern RingBuffer GpsRing;

// extern uint8_t CloudRxBuff[GPS_BUFF_SIZE * 4];
// extern RingBuffer CloudRing;

uint8_t gpsUartBuf[GPS_BUFF_SIZE];
uint8_t CloudUartBuf[GPS_BUFF_SIZE];
uint8_t BlueTooth[GPS_BUFF_SIZE];
uint8_t IMUUartBuf[IMU_BUFF_SIZE];

uint16_t GpsRxLen;
uint16_t CloudRxLen;


extern FIFO_Type Uart3RxFifo;
extern FIFO_Type Uart5RxFifo;
extern FIFO_Type Uart2RxFifo;
extern FIFO_Type Uart1RxFifo;
extern GpsData_t *gCloudGpsDataPtr;

//extern BOOL _handleGpsMessages(u8 *RtcmBuff,int length,GpsData_t *GPSData);
extern uint8_t stnID;
extern osMutexId uart2_mutex;

GpsData_t gGpsData = {
    .GPSbaudRate = 4800,
    .GPSProtocol = RTCM3,
    //.sirfInitialized = FALSE,
};

GpsData_t *gGpsDataPtr = &gGpsData;
GpsData_t *gCloudGpsDataPtr = (GpsData_t *)0x10000000;

// uint8_t GpsRxBuff[GPS_BUFF_SIZE * 2];
// RingBuffer GpsRing;

// uint8_t CloudRxBuff[GPS_BUFF_SIZE * 2];
// RingBuffer CloudRing;

uint32_t totalBytesRec = 0;
uint32_t maxBytesRec = 0;
uint32_t maxCycles = 0;
uint8_t stnID = 0;
/* output GGA to COM2 to request RTK data */
//uint8_t ggaBuff[256] = {0};
static void _handleGpsMessages(u8 *RtcmBuff, int length, GpsData_t *GPSData)
{
    int pos = 0;

    gnss_rtcm_t *rtcm = &GPSData->rtcm;
    
    obs_t *obs = &rtcm->obs[0];

    while (length)
    {
        length--;
        int ret_val = input_rtcm3(RtcmBuff[pos++], stnID, rtcm);
        if (ret_val == 1)
        {
            if (obs->pos[0] == 0.0 || obs->pos[1] == 0.0 || obs->pos[2] == 0.0)
            {
                /* do not output */
            }
            else if(stnID == 0)
            {
                rtcm->gnss_data_flag = 1;
                OS_Delay(5);
            }
        }
        else
        {
            //  HAL_UART_Transmit_DMA(&huart5, "no gga", 6);
        }
    }
}
/* end _handleGpsMessages */


/** ****************************************************************************
 * @name TaskGps
 * @brief task callback with the main loop for handle GPS data, make sure the
 *        GPS handling function gets called on a regular basis;.
 * gCalibration.productConfiguration.bit.hasGps = 1; by setting:
 * <hasGps>true</hasGps> and <useGps>true</useGps> in name_IMU380.xml file
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void GnssDataAcqTask(void const *argument)
{
    static uint32_t updateHDOP, pollSirfCnt;
    uint8_t Gpsbuf[GPS_BUFF_SIZE];
    // int pos = 0;

    // gnss_rtcm_t *rtcm = &gGpsDataPtr->rtcm;
    /* output GGA to COM2 to request RTK data */
    // uint8_t ggaBuff[256] = { 0 };
    // obs_t *obs = &rtcm->obs[0];

    // start out with the DOP high
    // gGpsDataPtr->HDOP = 50.0;

    FifoInit(&Uart3RxFifo,gpsUartBuf,GPS_BUFF_SIZE);
    FifoInit(&Uart5RxFifo,CloudUartBuf,GPS_BUFF_SIZE);
    FifoInit(&Uart2RxFifo,BlueTooth,BT_BUFF_SIZE);
    FifoInit(&Uart1RxFifo,IMUUartBuf,IMU_BUFF_SIZE);

    HAL_UART_Receive_DMA(&huart3,  gpsUartBuf, GPS_BUFF_SIZE);
    HAL_UART_Receive_DMA(&huart5, CloudUartBuf, GPS_BUFF_SIZE);
    __HAL_DMA_ENABLE(&hdma_usart3_rx);
    __HAL_DMA_ENABLE(&hdma_uart5_rx);

    __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    HAL_UART_Receive_DMA(&huart2, BlueTooth, GPS_BUFF_SIZE);
    __HAL_DMA_ENABLE(&hdma_usart2_rx);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);

    // __HAL_DMA_ENABLE(&hdma_usart1_rx);

    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart1, IMUUartBuf, IMU_BUFF_SIZE);
    while (1)
    {
        uint8_t bt_buff[GPS_BUFF_SIZE];
        int BtLen = FifoGet(&Uart2RxFifo,bt_buff, GPS_BUFF_SIZE); 
        if(BtLen)
        {
            stnID = 1;
            _handleGpsMessages(bt_buff,BtLen,gCloudGpsDataPtr);
        }
        // Uart3RxFifo.in = GPS_BUFF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);
        GpsRxLen=FifoGet(&Uart3RxFifo,Gpsbuf, GPS_BUFF_SIZE); 
#ifdef RTCM_UART2
        HAL_UART_Transmit_DMA(&huart2, Gpsbuf, GpsRxLen);   
#endif
        if( GpsRxLen)
        {
            stnID = 0;
            _handleGpsMessages(Gpsbuf,GpsRxLen,gGpsDataPtr);
        }

        CloudRxLen=FifoGet(&Uart5RxFifo,Gpsbuf, GPS_BUFF_SIZE); 
        if( CloudRxLen)
        {
            stnID = 1;
            _handleGpsMessages(Gpsbuf,CloudRxLen,gCloudGpsDataPtr);
        }

        OS_Delay(20);

    }
}

BOOL is_gnss_data_available()
{
    return gGpsDataPtr->rtcm.gnss_data_flag;
}
void  reset_gnss_data_available()
{
     gGpsDataPtr->rtcm.gnss_data_flag=0;
}

void get_gnss_rtcm_data(gpsDataStruct_t *gnss_data)
{

  #ifndef _USE_PPP_
    //  memcpy(&gGpsDataPtr->rtcm.obs_ref, &gCloudGpsDataPtr->rtcm.obs_ref,sizeof (obs_t));
  #endif 

    gnss_data->gnss_rtcm = &gGpsDataPtr->rtcm;
}


void GetGPSData(gpsDataStruct_t *data)
{
    data->gpsValid          = gGpsDataPtr->gpsValid;
    data->updateFlag        =  ( gGpsDataPtr->updateFlagForEachCall >> GOT_VTG_MSG ) & 0x00000001 &&
                               ( gGpsDataPtr->updateFlagForEachCall >> GOT_GGA_MSG ) & 0x00000001;
    // gGpsDataPtr->updateFlagForEachCall &= 0xFFFFFFFD;

    data->latitude          = (double)gGpsDataPtr->latSign * gGpsDataPtr->lat;
    data->longitude         = (double)gGpsDataPtr->lonSign * gGpsDataPtr->lon;
    data->altitude          = gGpsDataPtr->alt;

    data->vNed[0]           = gGpsDataPtr->vNed[0];
    data->vNed[1]           = gGpsDataPtr->vNed[1];
    data->vNed[2]           = gGpsDataPtr->vNed[2];

    data->trueCourse        = gGpsDataPtr->trueCourse;
    data->rawGroundSpeed    = gGpsDataPtr->rawGroundSpeed;

    //data->GPSSecondFraction = gGpsDataPtr->GPSSecondFraction; 
    data->altEllipsoid      = gGpsDataPtr->altEllipsoid;

    // data->itow              = gGpsDataPtr->itow;       
    // data->GPSmonth          = gGpsDataPtr->GPSmonth;
    // data->GPSday            = gGpsDataPtr->GPSday;
    // data->GPSyear           = gGpsDataPtr->GPSyear;  
    // data->GPSHour           = gGpsDataPtr->GPSHour;
    // data->GPSMinute         = gGpsDataPtr->GPSMinute; 
    // data->GPSSecond         = gGpsDataPtr->GPSSecond; 

}

void initGPSDataStruct(void)
{
    memset(gGpsDataPtr, 0, sizeof(GpsData_t));
    memset(gCloudGpsDataPtr, 0, sizeof(GpsData_t));
    gGpsData.GPSbaudRate = 115200;
    gGpsData.GPSProtocol = RTCM3;
    gCloudGpsDataPtr->GPSbaudRate = 115200;
    gCloudGpsDataPtr->GPSProtocol = RTCM3;
    // InitRingBuffer(&GpsRing,GpsRxBuff,GPS_BUFF_SIZE*4);
    // InitRingBuffer(&CloudRing,CloudRxBuff,GPS_BUFF_SIZE*4);
}
