/**
******************************************************************************
* @file    sensors.c 
******************************************************************************
*/

//**************************
#include "string.h"
#include "sensor.h"
#include "configurationAPI.h"
#include "sensors_data.h"
#include "sensorsAPI.h"

int gIsrDisableCount = 0;
uint16_t globalSensorsMask = 0x0007;

sensorsSamplingData_t _SensorsData;
sensorsDataPacket_t _SensorsDataPkt;
sensors_data_t gSensorsData;

void InitSensorsData()
{
    memset(&_SensorsData, 0, sizeof(sensorsSamplingData_t));
}

/** ***************************************************************************
 * @name _ConvertToXBowScaling() LOCAL
 * @brief XBOW packet scaling. Output data is placed in 'gSensorsData.rawSensors'
 *
 * Adjust the values in gSensorsData.rawSensors to work with the scaling that
 *  Nav-View uses to display counts and convert into (virtual) voltages (0-5V)
 *   -------- Rate sensor, accelerometer, and magnetometer readings --------
 *
 * NOTE: To make the factory packets work with Nav-View, it is necessary to
 *       shift the word(data contained in rawSensors) to the right by 9 bits.
 *       This limits the maximum value in 'rawSensors' to 2^23 (the scaling
 *       factor in Nav-View)
 * @param [in] reading - input raw data
 * @retval N/A
 ******************************************************************************/
void ConvertToXBowScaling()
{
    uint32_t boardTemp = 0;
    uint32_t accelTemp = 0;
    uint32_t gyroTemp = 0;

    /// Convert from int16 to uint32 and shift results to uint23_t (for proper
    ///   sensor-data scaling by Nav-View)
    gSensorsData.rawSensorsCombined[XACCEL] = CONVERT_380_TO_XBOW(gSensorsData.rawSensorsCombined[XACCEL]) >> 9;
    gSensorsData.rawSensorsCombined[YACCEL] = CONVERT_380_TO_XBOW(gSensorsData.rawSensorsCombined[YACCEL]) >> 9;
    gSensorsData.rawSensorsCombined[ZACCEL] = CONVERT_380_TO_XBOW(gSensorsData.rawSensorsCombined[ZACCEL]) >> 9;
    gSensorsData.rawSensorsCombined[XRATE] = CONVERT_380_TO_XBOW(gSensorsData.rawSensorsCombined[XRATE]) >> 9;
    gSensorsData.rawSensorsCombined[YRATE] = CONVERT_380_TO_XBOW(gSensorsData.rawSensorsCombined[YRATE]) >> 9;
    gSensorsData.rawSensorsCombined[ZRATE] = CONVERT_380_TO_XBOW(gSensorsData.rawSensorsCombined[ZRATE]) >> 9;

    /// -------- Board and accelerometer temperature readings --------
    /// Board temperature scaling factor (Nav-View) = 2^16. << 16 data short
    ///   to remove the scaling applied above.  NOTE: Shifting by
    ///   17 bits seems to place it in a more realistic voltage range (FIXME: verify this)
    boardTemp = CONVERT_380_TO_XBOW(gSensorsData.rawSensorsCombined[XATEMP]);
    gSensorsData.rawSensorsCombined[BTEMP] = boardTemp >> 16;

    /// The rate sensor and accelerometer temperatures are scaled by 2^30.
    /// Nav-View may not recognize all three axis.

    /// NOTE: all temperatures for board and sensors are gyro temp for
    ///       compatibility with Nav View and the Calibration Applications
    /// ----- Accelerometer temperature -----
    accelTemp = boardTemp >> 2;
    gSensorsData.rawSensorsCombined[XATEMP] = accelTemp;
    gSensorsData.rawSensorsCombined[YATEMP] = accelTemp;
    gSensorsData.rawSensorsCombined[ZATEMP] = accelTemp;

    /// ----- Rate sensor temperature -----
    /// 30-bit unsigned-integer
    gyroTemp = boardTemp >> 2;
    gSensorsData.rawSensorsCombined[XRTEMP] = gyroTemp;
    gSensorsData.rawSensorsCombined[YRTEMP] = gyroTemp;
    gSensorsData.rawSensorsCombined[ZRTEMP] = gyroTemp;
}

void AllignSensors()
{
    int tmpX, tmpY, tmpZ;

    //**********************************************************
    // sensor chip 0 accels
    tmpX = -_SensorsData.sensorData[0].accelData[0];
    tmpY = _SensorsData.sensorData[0].accelData[1];
    tmpZ = _SensorsData.sensorData[0].accelData[2];
    _SensorsData.sensorData[0].accelData[0] = tmpX;
    _SensorsData.sensorData[0].accelData[1] = tmpY;
    _SensorsData.sensorData[0].accelData[2] = tmpZ;
    // sensor chip 0 rates
    tmpX = -_SensorsData.sensorData[0].gyroData[0];
    tmpY = _SensorsData.sensorData[0].gyroData[1];
    tmpZ = _SensorsData.sensorData[0].gyroData[2];
    _SensorsData.sensorData[0].gyroData[0] = tmpX;
    _SensorsData.sensorData[0].gyroData[1] = tmpY;
    _SensorsData.sensorData[0].gyroData[2] = tmpZ;
     
    //**********************************************************
    // sensor chip 1 accels
    tmpX = _SensorsData.sensorData[1].accelData[1];
    tmpY = _SensorsData.sensorData[1].accelData[0];
    tmpZ = _SensorsData.sensorData[1].accelData[2];
    _SensorsData.sensorData[1].accelData[0] = tmpX;
    _SensorsData.sensorData[1].accelData[1] = tmpY;
    _SensorsData.sensorData[1].accelData[2] = tmpZ;
    // sensor chip 1 accels
    tmpX = _SensorsData.sensorData[1].gyroData[1];
    tmpY = -_SensorsData.sensorData[1].gyroData[0];
    tmpZ = _SensorsData.sensorData[1].gyroData[2];
    _SensorsData.sensorData[1].gyroData[0] = tmpX;
    _SensorsData.sensorData[1].gyroData[1] = tmpY;
    _SensorsData.sensorData[1].gyroData[2] = tmpZ;
    //**********************************************************
    // sensor chip 2 accels
    tmpX = -_SensorsData.sensorData[2].accelData[0];
    tmpY = _SensorsData.sensorData[2].accelData[1];
    tmpZ = _SensorsData.sensorData[2].accelData[2];
    _SensorsData.sensorData[2].accelData[0] = tmpX;
    _SensorsData.sensorData[2].accelData[1] = tmpY;
    _SensorsData.sensorData[2].accelData[2] = tmpZ;
    // sensor chip 2 rates
    tmpX = -_SensorsData.sensorData[2].gyroData[0];
    tmpY = _SensorsData.sensorData[2].gyroData[1];
    tmpZ = _SensorsData.sensorData[2].gyroData[2];
    _SensorsData.sensorData[2].gyroData[0] = tmpX;
    _SensorsData.sensorData[2].gyroData[1] = tmpY;
    _SensorsData.sensorData[2].gyroData[2] = tmpZ;

}

void FillRawSensorsData()
{
    uint16_t usedSensorChips = configGetUsedChips();
    uint16_t activeSensorChips = configGetActiveChips();
    usedSensorChips &= activeSensorChips;
    int divider = 0;

    AllignSensors();

    memset(&gSensorsData.scaledSensorsCombined, 0, sizeof(gSensorsData.scaledSensorsCombined));

    memset(&gSensorsData.rawSensors, 0, sizeof(gSensorsData.rawSensors));
    memset(&gSensorsData.rawSensorsCombined, 0, sizeof(gSensorsData.rawSensorsCombined));

    for (int i = 0; i < NUM_SENSOR_CHIPS; i++)
    {

        gSensorsData.rawSensors[i][XACCEL] = _SensorsData.sensorData[i].accelData[0];
        gSensorsData.rawSensors[i][YACCEL] = _SensorsData.sensorData[i].accelData[1];
        gSensorsData.rawSensors[i][ZACCEL] = _SensorsData.sensorData[i].accelData[2];
        gSensorsData.rawSensors[i][XRATE] = _SensorsData.sensorData[i].gyroData[0];
        gSensorsData.rawSensors[i][YRATE] = _SensorsData.sensorData[i].gyroData[1];
        gSensorsData.rawSensors[i][ZRATE] = _SensorsData.sensorData[i].gyroData[2];
        gSensorsData.rawSensors[i][XATEMP] = _SensorsData.sensorData[i].tempData;
        gSensorsData.rawSensors[i][YATEMP] = gSensorsData.rawSensors[i][XATEMP];
        gSensorsData.rawSensors[i][ZATEMP] = gSensorsData.rawSensors[i][XATEMP];
        gSensorsData.rawSensors[i][XRTEMP] = gSensorsData.rawSensors[i][XATEMP];
        gSensorsData.rawSensors[i][YRTEMP] = gSensorsData.rawSensors[i][XATEMP];
        gSensorsData.rawSensors[i][ZRTEMP] = gSensorsData.rawSensors[i][XATEMP];
        gSensorsData.rawSensors[i][BTEMP] = gSensorsData.rawSensors[i][XATEMP];
    }

    for (int i = 0; i < NUM_SENSOR_CHIPS; i++)
    {
        if (usedSensorChips & 0x0001)
        {
            for (int j = 0; j < N_RAW_SENS; j++)
            {
                gSensorsData.rawSensorsCombined[j] += gSensorsData.rawSensors[i][j];
            }
            divider++;
        }
        usedSensorChips >>= 1;
    }

    if (divider)
    {
        for (int j = 0; j < N_RAW_SENS; j++)
        {
            gSensorsData.rawSensorsCombined[j] /= divider;
        }
    }
}

void CombineSensorsData()
{
    uint16_t usedSensors;
    int divider[N_RAW_SENS];
    uint16_t usedSensorChips = configGetUsedChips();
    uint16_t activeSensorChips = configGetActiveChips();
    int divT = 0;
    usedSensorChips &= activeSensorChips;

    memset(&gSensorsData.scaledSensorsCombined, 0, sizeof(gSensorsData.scaledSensorsCombined));
    gSensorsData.rawTempSensorsCombined = 0;

    memset(divider, 0, sizeof(divider));

    for (int i = 0; i < NUM_SENSOR_CHIPS; i++)
    {
        if (usedSensorChips & 0x0001)
        {
            usedSensors = configGetUsedSensors(i);
            for (int j = 0; j < N_RAW_SENS; j++)
            {
                if (usedSensors & 0x0001)
                {
                    divider[j] += 1;
                    gSensorsData.scaledSensorsCombined[j] += gSensorsData.scaledSensors[i][j];
                }
                usedSensors >>= 1;
            }
            gSensorsData.rawTempSensorsCombined += gSensorsData.rawSensors[i][XATEMP];
            divT++;
        }
        usedSensorChips >>= 1;
    }

    for (int j = 0; j < N_RAW_SENS; j++)
    {
        if (divider[j])
        {
            gSensorsData.scaledSensorsCombined[j] /= divider[j];
        }
    }
    if (divT)
    {
        gSensorsData.rawTempSensorsCombined /= divT;
    }
}

void GetAccelData_g(float *data)
{
    for (int i = 0; i < 3; i++)
    {
        data[i] = gSensorsData.scaledSensorsCombined[i + XACCEL];
    }
}

void GetAccelData_g_AsDouble(double *data)
{
    for (int i = 0; i < 3; i++)
    {
        data[i] = gSensorsData.scaledSensorsCombined[i + XACCEL];
    }
}

void GetAccelData_mPerSecSq(float *data)
{
    for (int i = 0; i < 3; i++)
    {
        data[i] = gSensorsData.scaledSensorsCombined[i + XACCEL] * GRAVITY;
    }
}

void GetRateData_radPerSec(float *data)
{
    for (int i = 0; i < 3; i++)
    {
        data[i] = gSensorsData.scaledSensorsCombined[i + XRATE];
    }
}

void GetRateData_radPerSec_AsDouble(double *data)
{
    for (int i = 0; i < 3; i++)
    {
        data[i] = gSensorsData.scaledSensorsCombined[i + XRATE];
    }
}

void GetRateData_degPerSec(float *data)
{
    for (int i = 0; i < 3; i++)
    {
        data[i] = gSensorsData.scaledSensorsCombined[i + XRATE] * R2D;
    }
}

void GetRateData_degPerSec_AsDouble(double *data)
{
    for (int i = 0; i < 3; i++)
    {
        data[i] = gSensorsData.scaledSensorsCombined[i + XRATE] * R2D;
    }
}

void GetMagData_G(float *data)
{
    for (int i = 0; i < 3; i++)
    {
        data[i] = 0.0; // no mags on the platform so far
    }
}

void GetMagData_G_AsDouble(double *data)
{
    for (int i = 0; i < 3; i++)
    {
        //data[i] = 0.0;  // no mags on the platform so far
        data[i] = gSensorsData.scaledSensorsCombined[i + XRATE];
    }
}

void GetAccelsTempData(float *data)
{
    for (int i = 0; i < 3; i++)
    {
        data[i] = gSensorsData.scaledSensorsCombined[i + 9];
    }
}

void GetRatesTempData(float *data)
{
    for (int i = 0; i < 3; i++)
    {
        data[i] = gSensorsData.scaledSensorsCombined[i + 12];
    }
}

void GetBoardTempData(float *data)
{
    *data = gSensorsData.scaledSensorsCombined[15];
}

void GetBoardTempData_AsDouble(double *data)
{
    *data = gSensorsData.scaledSensorsCombined[15];
}

float GetUnitTemp()
{
    float ftmp;

    ftmp = gSensorsData.rawTempSensorsCombined - 6400;
    // temp in degrees C
    ftmp = ftmp / 256 + 25;
    return ftmp;
}
