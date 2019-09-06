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

// #include "algorithmAPI.h"
// #include "bitAPI.h"
// #include "boardAPI.h"
// #include "magAPI.h"
// #include "platformAPI.h"

// #include "taskDataAcquisition.h"
// // #include "taskUserCommunication.h"

// #include "osapi.h"
#include "osresources.h"
// #include "configureGPIO.h"

// #include "stm32f4xx.h"
// // #include "port_def.h"
// // #include "uart.h"
// // #include "comm_buffers.h"
// #include "osapi.h"
// #include "boardDefinition.h"
// // #include "BITStatus.h"
// #include "osresources.h"
// // #include "platformAPI.h"
// // #include "asm330lhh_reg.h"
// #include "gyroscope.h"
// #include "configurationAPI.h"
#include "taskDataAcquisition.h"
// #include "UserCommunicationSPI.h"
#include "osapi.h"
#include "sensors_data.h"
#include "board.h"
#include "ucb_packet.h"
#include "userAPI.h"
#include "hwAPI.h"
#include "sensorsAPI.h"
#include "calibrationAPI.h"
#include "commAPI.h"
#include "configurationAPI.h"

extern osMutexId uart_mutex;
extern osMutexId sensor_mutex;
/** ***************************************************************************
 * @name TaskDataAcquisition() CALLBACK main loop
 * @brief Get the sensor data at the specified frequency (based on the
 *        configuration of the accelerometer rate-sensor). Process and provide
 *        information to the user via the UART or SPI.
 * @param N/A
 * @retval N/A
 ******************************************************************************/

void SensorDataAcqINSTask(void const *argument)
{
    int res;
    // uint16_t dacqRate;

#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    BOOL overRange = FALSE; //uncomment this line if overrange processing required
#pragma GCC diagnostic warning "-Wunused-but-set-variable"
    InitFactoryCalibration();
    ApplyFactoryConfiguration();

    res = InitSensors();
    InitSensorsData();
    initUserDataProcessingEngine();

    while (1)
    {
        res = osSemaphoreWait(dataAcqSem, 1000);
#ifdef SENSOR_MUTEX
        osMutexWait(sensor_mutex,osWaitForever);
#endif
        SampleSensorsData();
#ifdef SENSOR_MUTEX
        osMutexRelease(sensor_mutex);
#endif
        ApplyFactoryCalibration();
        inertialAndPositionDataProcessing(200); 
#ifdef CALIBRATION 
#ifdef UART_MUTEX
        osMutexWait(uart_mutex,osWaitForever);
#endif
        ProcessUserCommands (0);
        SendContinuousPacket(200);
#ifdef UART_MUTEX
        osMutexRelease(uart_mutex);
#endif
        OS_Delay(6);
#endif    
    }
} 
