/** ***************************************************************************
* @file gyroscope.c Generic functions for gyroscope
* @author
* @date   March, 2017
* @copyright (c) 2017 All Rights Reserved.
* @section LICENSE
* THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
* KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
* PARTICULAR PURPOSE.
*
* Note, the gyro should implement the interface described in
* gyroscope.h. This file just provides the specifics, for use by the
* associated C file only.
*****************************************************************************/

//*****************************
#include <stdint.h>
#include <string.h>
#include "gyroscope.h"
#include "SpiBitBang.h"
#include "Indices.h"
#include "sensor.h"
#include "lowpass_filter.h"
#include "gyroASM330LHH.h"
#include "osapi.h"
// #include "configurationAPI.h"
extern UART_HandleTypeDef huart5;

uint16_t sensorChipsMask = 0x0007;
uint16_t activeChipsMask = 0x0007;

gRate_t gRate;
static int32_t gyroFilteredVal[NUM_SENSOR_CHIPS][NUM_AXIS];
static int32_t accelFilteredVal[NUM_SENSOR_CHIPS][NUM_AXIS];

/** ***************************************************************************
 * @name LimitInt16Value() API utility function to saturate an input value
 * @brief saturate
 * @param [in] value - input value
 * @param [in] limit - limit value
 * @retval limited input value
 ******************************************************************************/
int16_t LimitInt16Value(int16_t value,
                        int16_t limit)
{
    if (value > limit)
    {
        return limit;
    }
    else if (value < -limit)
    {
        return -limit;
    }
    else
    {
        return value;
    }
}

/** ****************************************************************************
* @name: _ReadGyroRegister LOCAL
* @param [in] address - SPI adddress
* @retval rx - data read at the address
******************************************************************************/
uint8_t _ReadGyroRegister(uint8_t address, uint8_t *reg1, uint8_t *reg2, uint8_t *reg3)
{
    address |= READ_INDICATION;
    SpiBitBangReadTransaction(address, reg1, reg2, reg3, 1);
    return 0;
}

/** ****************************************************************************
* @name: _ReadGyroData LOCAL
* @param [in] address - SPI adddress
* @retval rx - data read at the address
******************************************************************************/
uint8_t _ReadGyroData(uint8_t address, uint8_t *data1, uint8_t *data2, uint8_t *data3, int len)
{
    address |= READ_INDICATION;
    SpiBitBangReadTransaction(address, data1, data2, data3, len);
    return 0;
}

/** ****************************************************************************
* @name: _WriteGyroRegister LOCAL
* @param [in] address - SPI address
* @param [in] data - data to write out
* @retval always return 0
******************************************************************************/
uint8_t _WriteGyroRegister(uint8_t address, uint8_t data)
{
    SpiBitBangWriteTransaction(address, &data, 1);
    return 0;
}

void FilterAccelSensorsData(int chipNum, int16_t *inputData, int32_t *outputData)
{
    static uint8_t input_data_rate = BWF_LOWPASS_DATA_RATE_800;

    // Filter the data (or not)
    switch (configGetPrefilterFreq())
    {
    case 0:
    case 8:
    case 26785:
        // Do not prefilter the sensor data
        accelFilteredVal[chipNum][X_AXIS] = inputData[X_AXIS];
        accelFilteredVal[chipNum][Y_AXIS] = inputData[Y_AXIS];
        accelFilteredVal[chipNum][Z_AXIS] = inputData[Z_AXIS];
        break;

    case 6: // 3rd-order, 100 Hz BWF (cascaded 1st-order filters)
        _accelFilt_3rdOrderBWF_LowPass_Axis_cascaded1st(chipNum, X_AXIS, inputData[X_AXIS],
                                                        &accelFilteredVal[chipNum][X_AXIS],
                                                        LPF_100HZ, input_data_rate);
        _accelFilt_3rdOrderBWF_LowPass_Axis_cascaded1st(chipNum, Y_AXIS, inputData[Y_AXIS],
                                                        &accelFilteredVal[chipNum][Y_AXIS],
                                                        LPF_100HZ, input_data_rate);
        _accelFilt_3rdOrderBWF_LowPass_Axis_cascaded1st(chipNum, Z_AXIS, inputData[Z_AXIS],
                                                        &accelFilteredVal[chipNum][Z_AXIS],
                                                        LPF_100HZ, input_data_rate);
        break;
    case 2142:
    case 4: // 3rd-order, 25 Hz BWF (cascaded 1st-order filters)
        _accelFilt_3rdOrderBWF_LowPass_Axis_cascaded1st(chipNum, X_AXIS, inputData[X_AXIS],
                                                        &accelFilteredVal[chipNum][X_AXIS],
                                                        LPF_25HZ, input_data_rate);
        _accelFilt_3rdOrderBWF_LowPass_Axis_cascaded1st(chipNum, Y_AXIS, inputData[Y_AXIS],
                                                        &accelFilteredVal[chipNum][Y_AXIS],
                                                        LPF_25HZ, input_data_rate);
        _accelFilt_3rdOrderBWF_LowPass_Axis_cascaded1st(chipNum, Z_AXIS, inputData[Z_AXIS],
                                                        &accelFilteredVal[chipNum][Z_AXIS],
                                                        LPF_25HZ, input_data_rate);
        break;

    case 0x9: // Future values for 2nd-order filters from 0x9 to 0xF (9 to 15)
        // 4th-order, 50 Hz BWF (cascaded 2nd-order filters)
        _accelFilt_4thOrderBWF_LowPass_Axis_cascaded2nd(chipNum, X_AXIS, inputData[X_AXIS],
                                                        &accelFilteredVal[chipNum][X_AXIS],
                                                        LPF_50HZ, input_data_rate);
        _accelFilt_4thOrderBWF_LowPass_Axis_cascaded2nd(chipNum, Y_AXIS, inputData[Y_AXIS],
                                                        &accelFilteredVal[chipNum][Y_AXIS],
                                                        LPF_50HZ, input_data_rate);
        _accelFilt_4thOrderBWF_LowPass_Axis_cascaded2nd(chipNum, Z_AXIS, inputData[Z_AXIS],
                                                        &accelFilteredVal[chipNum][Z_AXIS],
                                                        LPF_50HZ, input_data_rate);
        break;

    default:
        _accelFilt_3rdOrderBWF_LowPass_Axis_cascaded1st(chipNum, X_AXIS, inputData[X_AXIS],
                                                        &accelFilteredVal[chipNum][X_AXIS],
                                                        LPF_50HZ, input_data_rate);
        _accelFilt_3rdOrderBWF_LowPass_Axis_cascaded1st(chipNum, Y_AXIS, inputData[Y_AXIS],
                                                        &accelFilteredVal[chipNum][Y_AXIS],
                                                        LPF_50HZ, input_data_rate);
        _accelFilt_3rdOrderBWF_LowPass_Axis_cascaded1st(chipNum, Z_AXIS, inputData[Z_AXIS],
                                                        &accelFilteredVal[chipNum][Z_AXIS],
                                                        LPF_50HZ, input_data_rate);
        break;
    }
    outputData[0] = LimitInt16Value(accelFilteredVal[chipNum][X_AXIS], INT16_LIMIT);
    outputData[1] = -LimitInt16Value(accelFilteredVal[chipNum][Y_AXIS], INT16_LIMIT);
    outputData[2] = -LimitInt16Value(accelFilteredVal[chipNum][Z_AXIS], INT16_LIMIT);
}

void FilterRateSensorsData(int chipNum, int16_t *inputData, int32_t *outputData)
{
    static uint8_t input_data_rate = BWF_LOWPASS_DATA_RATE_800;

    // Filter the data (or not)
    switch (configGetPrefilterFreq())
    {
    case 0:
    case 8:
    case 26785:
        // Do not prefilter the sensor data
        gyroFilteredVal[chipNum][X_AXIS] = inputData[X_AXIS];
        gyroFilteredVal[chipNum][Y_AXIS] = inputData[Y_AXIS];
        gyroFilteredVal[chipNum][Z_AXIS] = inputData[Z_AXIS];
        break;

    case 6: // 3rd-order, 100 Hz BWF (cascaded 1st-order filters)
        _rateFilt_3rdOrderBWF_LowPass_Axis_cascaded1st(chipNum, X_AXIS, inputData[X_AXIS],
                                                       &gyroFilteredVal[chipNum][X_AXIS],
                                                       LPF_100HZ, input_data_rate);
        _rateFilt_3rdOrderBWF_LowPass_Axis_cascaded1st(chipNum, Y_AXIS, inputData[Y_AXIS],
                                                       &gyroFilteredVal[chipNum][Y_AXIS],
                                                       LPF_100HZ, input_data_rate);
        _rateFilt_3rdOrderBWF_LowPass_Axis_cascaded1st(chipNum, Z_AXIS, inputData[Z_AXIS],
                                                       &gyroFilteredVal[chipNum][Z_AXIS],
                                                       LPF_100HZ, input_data_rate);
        break;
    case 2142:
    case 4: // 3rd-order, 25 Hz BWF (cascaded 1st-order filters)
        _rateFilt_3rdOrderBWF_LowPass_Axis_cascaded1st(chipNum, X_AXIS, inputData[X_AXIS],
                                                       &gyroFilteredVal[chipNum][X_AXIS],
                                                       LPF_25HZ, input_data_rate);
        _rateFilt_3rdOrderBWF_LowPass_Axis_cascaded1st(chipNum, Y_AXIS, inputData[Y_AXIS],
                                                       &gyroFilteredVal[chipNum][Y_AXIS],
                                                       LPF_25HZ, input_data_rate);
        _rateFilt_3rdOrderBWF_LowPass_Axis_cascaded1st(chipNum, Z_AXIS, inputData[Z_AXIS],
                                                       &gyroFilteredVal[chipNum][Z_AXIS],
                                                       LPF_25HZ, input_data_rate);
        break;

    case 0x9: // Future values for 2nd-order filters from 0x9 to 0xF (9 to 15)
        // 4th-order, 50 Hz BWF (cascaded 2nd-order filters)
        _rateFilt_4thOrderBWF_LowPass_Axis_cascaded2nd(chipNum, X_AXIS, inputData[X_AXIS],
                                                       &gyroFilteredVal[chipNum][X_AXIS],
                                                       LPF_50HZ, input_data_rate);
        _rateFilt_4thOrderBWF_LowPass_Axis_cascaded2nd(chipNum, Y_AXIS, inputData[Y_AXIS],
                                                       &gyroFilteredVal[chipNum][Y_AXIS],
                                                       LPF_50HZ, input_data_rate);
        _rateFilt_4thOrderBWF_LowPass_Axis_cascaded2nd(chipNum, Z_AXIS, inputData[Z_AXIS],
                                                       &gyroFilteredVal[chipNum][Z_AXIS],
                                                       LPF_50HZ, input_data_rate);
        break;

    default:
        _rateFilt_3rdOrderBWF_LowPass_Axis_cascaded1st(chipNum, X_AXIS, inputData[X_AXIS],
                                                       &gyroFilteredVal[chipNum][X_AXIS],
                                                       LPF_50HZ, input_data_rate);
        _rateFilt_3rdOrderBWF_LowPass_Axis_cascaded1st(chipNum, Y_AXIS, inputData[Y_AXIS],
                                                       &gyroFilteredVal[chipNum][Y_AXIS],
                                                       LPF_50HZ, input_data_rate);
        _rateFilt_3rdOrderBWF_LowPass_Axis_cascaded1st(chipNum, Z_AXIS, inputData[Z_AXIS],
                                                       &gyroFilteredVal[chipNum][Z_AXIS],
                                                       LPF_50HZ, input_data_rate);
        break;
    }
    outputData[0] = LimitInt16Value(gyroFilteredVal[chipNum][X_AXIS], INT16_LIMIT);
    outputData[1] = -LimitInt16Value(gyroFilteredVal[chipNum][Y_AXIS], INT16_LIMIT);
    outputData[2] = -LimitInt16Value(gyroFilteredVal[chipNum][Z_AXIS], INT16_LIMIT);
}

uint8_t FifoConfig()
{
    uint8_t res = ASM330LHHDisableFifo();

    if (!res)
    {
        return res;
    }

    res = ASM330LHHSetInertialSensorsBDR();

    if (!res)
    {
        return res;
    }

    return ASM330LHHEnableFifo();
}

uint8_t ActivateSensors()
{
    int result;

    result = ASM330LHHConfig();

    if (!result)
    {
        return 0;
    }

    OS_Delay(10);

    result = powerOnASM330LHHAccel();

    if (!result)
    {
        return 0;
    }

    OS_Delay(20);

    result = powerOnASM330LHHGyro();

    if (!result)
    {
        return 0;
    }

    OS_Delay(100);

    result = ASM330LHHSetAddrIncMode();

    if (!result)
    {
        return 0;
    }

    result = FifoConfig();

    if (!result)
    {
        return 0;
    }

    return result;
}

/** ****************************************************************************
* @name InitGyro set up the SPI bus and synch pin C5
* @param [in] rangeInDps - max Degrees per Second
* @param [in] outputDataRate - hz
* @retval status
******************************************************************************/
uint8_t InitSensors()
{
    int status;
    uint8_t isASM = FALSE;

    // Return the SPI clock frequency to nominal frequency (~7.5  Mhz)
    SpiBitBangSetBaud(SPI_MIN_SPEED);

    gRate.status = GYRO_BUFFER_IDLE;
    gRate.cached_num = 0;
    gRate.temp_buff_num = 0;

    status = ASM330LHHSoftReset();

    if (!status)
    {
        return FALSE;
    }

    // activeChipsMask   = configGetActiveChips();
    // globalSensorsMask = activeChipsMask;
    // SpiBitBangSelectActiveSensors(activeChipsMask);

    isASM = isASM330LHH();

    if (!isASM)
    {
        return FALSE;
    }

    return ActivateSensors();
}

// read set of sensors from FIFO
uint8_t GyroGetDataFromFifo(int16_t *readings1, int16_t *readings2, int16_t *readings3, uint16_t activeSensors)
{
    uint16_t mask;
    int16_t *readings;
    static uint8_t tmp[NUM_SENSOR_CHIPS][10];

    SpiBitBangSelectSaveActiveSensors(activeSensors);

    _ReadGyroData(ASM330LHH_FIFO_DATA_OUT_TAG, &tmp[0][0], &tmp[1][0], &tmp[2][0], 7);

    for (int i = 0; i < NUM_SENSOR_CHIPS; i++)
    {
        mask = 1 << i;
        if (mask & activeSensors)
        {
            switch (i)
            {
            case 0:
                readings = readings1;
                break;
            case 1:
                readings = readings2;
                break;
            case 2:
                readings = readings3;
                break;
            default:
                while (1)
                    ;
            }
            readings[0] = tmp[i][0];
            readings[1] = (tmp[i][2] << 8) | tmp[i][1];
            readings[2] = (tmp[i][4] << 8) | tmp[i][3];
            readings[3] = (tmp[i][6] << 8) | tmp[i][5];
        }
    }

    SpiBitBangRestoreActiveSensors();

    return 0;
}

int GyroGetNumSamplesInFifo(uint16_t *num)
{
    int max = 0;
    uint8_t regs[NUM_SENSOR_CHIPS];
    uint8_t regs1[NUM_SENSOR_CHIPS];
    uint8_t mask = activeChipsMask;
    _ReadGyroRegister(ASM330LHH_FIFO_STATUS1, &regs[0], &regs[1], &regs[2]);
    _ReadGyroRegister(ASM330LHH_FIFO_STATUS2, &regs1[0], &regs1[1], &regs1[2]);
    for (int i = 0; i < NUM_SENSOR_CHIPS; i++)
    {
        if (mask & 0x01)
        {
            num[i] = ((regs1[i] & 0x03) << 8) | regs[i];
            if (num[i] > max)
            {
                max = num[i];
            }
        }
        else
        {
            num[i] = 0;
        }
        mask >>= 1;
    }

    return max;
}

uint16_t numSamples[NUM_SENSOR_CHIPS + 1];
int16_t sensorData[NUM_SENSOR_CHIPS][5];
void SampleSensorsData()
{
    uint16_t   activeSensors, mask;
    int        maxNum = 0;
    int32_t    outData[4];
    int        temp;
 
    sensorsSamplingData_t *data = &_SensorsData;

    maxNum = GyroGetNumSamplesInFifo(numSamples);
    //sprintf(buf,"maxNum = %d\r\n",maxNum);

    //HAL_UART_Transmit_DMA(&huart5,buf,strlen(buf));
    for (int i = 0; i < maxNum; i++)
    {
        activeSensors = 0;
        if (numSamples[0])
        {
            activeSensors |= 0x01;
            numSamples[0]--;
        }
        if (numSamples[1])
        {
            activeSensors |= 0x02;
            numSamples[1]--;
        }
        if (numSamples[2])
        {
            activeSensors |= 0x04;
            numSamples[2]--;
        }

        GyroGetDataFromFifo(&sensorData[0][0], &sensorData[1][0], &sensorData[2][0], activeSensors);

        for (int j = 0; j < NUM_SENSOR_CHIPS; j++)
        {
            mask = 1 << j;
            if (activeSensors & mask)
            {
                switch (sensorData[j][0])
                {
                case ASM330LHH_FIFO_TAG_RATE_1:
                case ASM330LHH_FIFO_TAG_RATE_2:
                case ASM330LHH_FIFO_TAG_RATE_3:
                case ASM330LHH_FIFO_TAG_RATE_4:
                    FilterRateSensorsData(j, &sensorData[j][1], outData);
                    data->sensorData[j].gyroData[0] = outData[0];
                    data->sensorData[j].gyroData[1] = outData[1];
                    data->sensorData[j].gyroData[2] = outData[2];
                    break;
                case ASM330LHH_FIFO_TAG_ACCEL_1:
                case ASM330LHH_FIFO_TAG_ACCEL_2:
                case ASM330LHH_FIFO_TAG_ACCEL_3:
                case ASM330LHH_FIFO_TAG_ACCEL_4:
                    FilterAccelSensorsData(j, &sensorData[j][1], outData);
                    data->sensorData[j].accelData[0] = outData[0];
                    data->sensorData[j].accelData[1] = outData[1];
                    data->sensorData[j].accelData[2] = outData[2];
                    break;
                case ASM330LHH_FIFO_TAG_TEMP_1:
                case ASM330LHH_FIFO_TAG_TEMP_2:
                case ASM330LHH_FIFO_TAG_TEMP_3:
                case ASM330LHH_FIFO_TAG_TEMP_4:
                    temp = sensorData[j][1];
                    temp = (temp >> 1) + 5888; // scale to MAX temperature
                    data->sensorData[j].tempData = temp;
                    break;
                default:
                    break;
                }
            }
        }
    }
}
