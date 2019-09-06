/** ***************************************************************************
 * @file calibrate.c
 * @Author
 * @date   September, 2008
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *  Sensor Calibration Algorithms.
 ******************************************************************************/
#include <stdint.h>
#include <string.h> // memcpy

//****************************
#include "calibration.h"
#include "configurationAPI.h"
#include "scaling.h"
#include "filter.h"
#include "lowpass_filter.h"
#include "Indices.h"
#include "GlobalConstants.h"
//// For filter testing
#include "math.h"
#include "platformAPI.h"
#include "configurationAPI.h"
#include "eepromAPI.h"
#include "sensors_data.h"
#include "crc.h"
#include "sensorsAPI.h"
#include "hwAPI.h"

void     HW_SetTestMode(BOOL fOn);

CalibrationStruct   gCalibration[NUM_SENSOR_CHIPS];
int                 calibrateTableValid[NUM_SENSOR_CHIPS];

//#include "CompilerFlags.h"   // for GYRO_MAXIM21000 and GYRO_BMI160

static void   _ApplySensorCalibration(int idx,  int sensor, float *scaledSensors, int tempSensor );
static void   _AdjustForSensorsMisalignment(int idx,  int  sensor, int *rawSensors);
void          _orientSensors( float *scaledSensors );
void          _LimitSensorValues( float *scaledSensors );


/// Compensation lookup index... first six values are for temperature bias
///   (accelerometer and rate-sensor), next nine are for scale factor
///   (accelerometer, rate-sensor, and magnetometer)
int16_t gCurrentTableLookupIndex[NUM_SENSOR_CHIPS][15] = {0};
int16_t gBeginTableLookupIndex[NUM_SENSOR_CHIPS][15]   = {0};
int16_t gEndTableLookupIndex[NUM_SENSOR_CHIPS][15]     = {0};


/** ****************************************************************************
 * @name: _readCalIntoMem LOCAL Read configuration from EEPROM into RAM
 * TRACE:
 * [SDD_INIT_CONFIGURATION_ADAHRS <-- SRC_READ_CONFIGURATION_AND_CALIBRATION_INTO_MEMORY]
 * [SDD_EEPROM_INIT_READ <-- SRC_READ_CONFIGURATION_AND_CALIBRATION_INTO_MEMORY]
 * [SDD_INIT_MISALIGN_ADAHRS <-- SRC_READ_CONFIGURATION_AND_CALIBRATION_INTO_MEMORY]
 * @param N/A
 * @retval N/A
 ******************************************************************************/
static void _readCalIntoMem (int idx)
{
    readEEPROMCalibration(idx, &gCalibration[idx]); // s_eeprom.c
}

/** ****************************************************************************
 * @name _CheckCalibrationCrc LOCAL Verifies the stored calibration CRC matches
 *       the computed CRC
 * TRACE: [SDD_EEPROM_CRC_METHOD <-- SRC_CHECK_CALIBRATION_CRC]
 *        [SDD_EEPROM_CRC_DATA <-- SRC_CHECK_CALIBRATION_CRC]
 * @param N/A
 * @retval boolean indicating success or failure of CRC of calibration area
 ******************************************************************************/
static BOOL _CheckCalibrationCrc (int idx)
{
    unsigned int i;
    uint16_t     eepromWord;
    uint16_t     testCRC = CRC_CCITT_INITIAL_SEED;
    uint8_t      *ptr;
                
    ptr = getEEPROMCalTabPtr(idx);

    /// CRC valid EEPROM range to check
    for (i = 0; i < (CAL_PARTITION_SIZE-4); i+=2, ptr+=2) {
        /// read word, swap the bytes, add it to the CRC
        eepromWord = (ptr[1] << 8) | ptr[0];
        testCRC    = initCRC_16bit(eepromWord, testCRC);
    }

    if (testCRC ==  *(uint16_t *)ptr) {
        return TRUE;
    } else {
        return FALSE;
    }

}

/** ***************************************************************************
 * @name _searchTempTable() LOCAL linear interpolation
 * @brief linear interpolation based on a look-up table to estimate the
 *        temperature bias-compensation for the rate sensors and accelerometers.
 *
 * given inertial sensor index 0:5 (accels,gyros), this function looks at temp
 * sensor and searches the inertial sensor's temp comp table to find the correct
 * table entry. The search begins from where the last search ended.
 * Note: The "temperatures" are in counts
 *
 * @param [in] sensor - inertial sensor index 0:5 (accels, gyros)
 * @param [in] tempReading - index
 * @retval The interpolated temp bias is calculated and returned.
 ******************************************************************************/
static int _searchTempTable(int idx, int sensor, int32_t tempReading)
{
    int16_t  B_Index, E_Index;
    int      BCount,  ECount;
    float    BValue,  EValue;
    double   valOverCount;
    volatile BOOL     extrapolate = FALSE;
    int      eSign    = 1; 
    int      temp;
// FIXME: this only needs to be done at one hz and only needs to be re-calulated
// if the slope changes
    /// linear interpolation with the independent value (x) being temperature
    ///   dependent value (y) the bias.  The equation used is y = y1 + m( x - x1 ),
    ///   where x1 is equivalent to BCount.  ECount ~= to the x2 variable used to
    ///   compute the slope, m = ( y2 - y1 )/( x2 - x1 ).
    ///
    ///    |        /
    ///    |       o ( x2, y2 ) = ( ECount, EValue )
    ///  y -------+
    ///    |     /.
    ///    |    / .
    ///  --|---/--|----------
    ///    |  /   x
    ///    | /
    ///    |o ( x1, y1 ) = ( BCount, BValue )
    ///    /
    ///   /|

    /// Begin search at the last point. The temperature does not change rapidly;
    ///   reduces steps in the search. B_Index - location of count and value in
    ///   the calibration table.
    if(gBeginTableLookupIndex[idx][sensor] == 0xffff){
        return 0.0;     // no values defined
    }
    
    B_Index = gCurrentTableLookupIndex[idx][sensor];
    E_Index = B_Index + 1;

    /// "counts" are the independent variables (x) in the calibration tables
    BCount = gCalibration[idx].calibrationTablesA[B_Index].counts;
    ECount = gCalibration[idx].calibrationTablesA[E_Index].counts;

    if (tempReading < BCount)
    {   /// drop to a lower index and recheck
        do {
            /// If the program cannot go to a lower index, drop out of the do-loop
            if (B_Index <= gCalibration[idx].calibrationTableIndexA[sensor]) {
                B_Index  = gCalibration[idx].calibrationTableIndexA[sensor];  //first index
                BCount   = gCalibration[idx].calibrationTablesA[B_Index].counts;
                if(BCount > tempReading){
                    extrapolate = TRUE;
                    eSign       = -1;   // extrapolate below
                }
                break;
            }
            B_Index--;
            BCount = gCalibration[idx].calibrationTablesA[B_Index].counts;
            if (tempReading >= BCount) {
                break;
            }
        } while (1);

        /// Save off the beginning index for the next time through the table
        gCurrentTableLookupIndex[idx][sensor] = B_Index;

        E_Index = B_Index + 1; /// end-index
        ECount  = gCalibration[idx].calibrationTablesA[E_Index].counts;
    }
    else if (tempReading > ECount)
    {   // increase the index and recheck
        do {
            // If the program cannot go to a higher index, drop out of the do-loop
            if( E_Index >= gEndTableLookupIndex[idx][sensor]) {
                E_Index  = gEndTableLookupIndex[idx][sensor] - 1;
                ECount   = gCalibration[idx].calibrationTablesA[E_Index].counts;
                if(ECount < tempReading){
                    extrapolate = TRUE;
                    eSign       = 1;     // extrapolate above
                }
                break;
            }
            E_Index++;
            ECount = gCalibration[idx].calibrationTablesA[E_Index].counts;
            if (tempReading <= ECount) {
                break; 
            }
        } while (1);

        /// Save off the index for the next time through the table, and determine
        ///   BCount (equivalent to x1 in the interpolation scheme)
        B_Index = E_Index - 1;
        gCurrentTableLookupIndex[idx][sensor] = B_Index;
        BCount  = gCalibration[idx].calibrationTablesA[B_Index].counts;
    }

    // 1) y1 and y2
    BValue = gCalibration[idx].calibrationTablesA[B_Index].value;
    EValue = gCalibration[idx].calibrationTablesA[E_Index].value;

    // 2) m = ( y2 - y1 )/( x2 - x1 )
    valOverCount = ((double)(EValue - BValue) / (double)(ECount - BCount));

    if(extrapolate == 0){
    // ------ linear interpolation ------
        temp = (int32_t)(BValue + ((int32_t)(tempReading - BCount)) * valOverCount);
    }else {
    // ------ linear extrapolation ------
        if(eSign == 1){ // extrapolate up
            temp = (int32_t)(EValue + ((int32_t)(tempReading - ECount)) * valOverCount);
        }else{          // extrapolate dn
            temp = (int32_t)(BValue - ((int32_t)(BCount - tempReading)) * valOverCount);
        }
    }

    return temp;
}

/** ***************************************************************************
 * @name _searchScaleTable() LOCAL table search
 * @brief given sensor index 0:8 (accels,gyros,mags), and the temp compensated
 *        sensor counts, this function searches the sensor's scale table to find
 *        the correct table entry.  The search begins from where the last search
 *        ended.
 *
 * @param [in] sensor sensor index 0:8 (accels,gyros,mags)
 * @param [in] sensorCounts - temp compensated sensor counts
 * @retval The interpolated engineering unit value is calculated and returned.
 ******************************************************************************/
double _searchScaleTable(int idx, int     sensor, int32_t sensorCounts)
{
    int16_t B_Index;
    int16_t E_Index;
    int32_t BCount;
    int32_t ECount;
    float   BValue;
    float   EValue;
    double  valOverCount;
    double  num;
    double  den;
    double  temp;

    if(gBeginTableLookupIndex[idx][6+sensor] == 0xFFFF){
        return 0.0;
    }
    B_Index = gCurrentTableLookupIndex[idx][6 + sensor];
    E_Index = B_Index + 1;
    BCount = gCalibration[idx].calibrationTablesA[B_Index].counts;
    ECount = gCalibration[idx].calibrationTablesA[E_Index].counts;

    if (sensorCounts < BCount)  {
        do {
            if( B_Index <= gCalibration[idx].calibrationTableIndexA[sensor + 6] ) {
                B_Index  =  gCalibration[idx].calibrationTableIndexA[sensor + 6];
                BCount   = gCalibration[idx].calibrationTablesA[B_Index].counts;
                break;
            }
            B_Index--;
            BCount = gCalibration[idx].calibrationTablesA[B_Index].counts;
        } while (sensorCounts < BCount);

        gCurrentTableLookupIndex[idx][6 + sensor] = B_Index;
        E_Index = B_Index + 1;
        ECount  = gCalibration[idx].calibrationTablesA[E_Index].counts;

    } else if (sensorCounts > ECount)  {
        do {
            if( E_Index >= ( gEndTableLookupIndex[idx][6+sensor]) ) {
                E_Index =   gEndTableLookupIndex[idx][6+sensor] - 1;
                break; 
            }
            E_Index++;
            ECount = gCalibration[idx].calibrationTablesA[E_Index].counts;
        } while (sensorCounts > ECount);

        B_Index                            = E_Index - 1;
        gCurrentTableLookupIndex[idx][6+sensor] = B_Index;
        BCount                             = gCalibration[idx].calibrationTablesA[B_Index].counts;
    }

    /// force the bits into a double!
    BValue = gCalibration[idx].calibrationTablesA[B_Index].value;
    EValue = gCalibration[idx].calibrationTablesA[E_Index].value;
    num    = EValue - BValue;
    den    = ECount - BCount;
    valOverCount = num / den;
    temp         = (double)BValue + ((double)(sensorCounts - BCount)) * valOverCount;
    return temp;
}

/** ***************************************************************************
 * @name CalibrateInit() set up defaults in calibration
 * @brief Initialize the calibration global variables in case the eeprom version
 *  isn't any good
 *
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
void CalibrateInit( void )
{
    int       i;

    gSensorsData.rateAlarm  = (float)(450.0 * DEGREES_TO_RADS);
    gSensorsData.accelAlarm = (float)(0.95 * 8.0);

    if (gCalibration[0].serialNumber == 0 || gCalibration[0].serialNumber == 0xFFFFFFFF || !calibrateTableValid[0])
    { /// unit not calibrated, use default settings
        for (int idx = 0; idx < NUM_SENSOR_CHIPS; idx++)
        {
            for (i = 0; i < N_TABLES_A; i++)
            {
                gCalibration[idx].calibrationTableIndexA[i] = 0xffff; // invalid
            }

            for (i = 0; i < N_ROWS_TABLE_A; i++)
            {
                gCalibration[idx].calibrationTablesA[i].counts = 0;
                gCalibration[idx].calibrationTablesA[i].value = 0.0;
            }
            /// misalignment table
            for (i = 0; i < N_MISALIGN; i++)
            {
                gCalibration[idx].misalign[i] = 0.0; /// no off axis influences
            }
            for (i = 0; i < 15; i++)
            {
                gBeginTableLookupIndex[idx][i]   = 0xFFFF;
            }
        } // end if no serial number so fake a calibration table

    }else{
        for (int idx = 0; idx < NUM_SENSOR_CHIPS; idx++)
        {
            for (i = 0; i < 15; i++){
                gBeginTableLookupIndex[idx][i]   = gCalibration[idx].calibrationTableIndexA[i];
                gCurrentTableLookupIndex[idx][i] = gCalibration[idx].calibrationTableIndexA[i];
                for(int j = gCalibration[idx].calibrationTableIndexA[i]; j < gCalibration[idx].calibrationTableIndexA[i+1]; j++){
                    if(gCalibration[idx].calibrationTablesA[j].counts == 0x7fffffff){
                        gEndTableLookupIndex[idx][i] = j;
                        break;
                    }
                }
            }
        }
    }

    // FIXME: This is placed here to take the place of the original definition of
    //        gCalibration.misalign, which was originally defined as a float.  Now it is redefined
    //        as an int32_t so we could implement this as fixed-point math.  REMOVE AFTER IMPLEMENTATION!
    // 7.450580596923828e-09 = 1/2^27
//    // Compute the misalignment from the values loaded into EEPROM
//    int32_t misalign_hex[18];
//    for( i = 0; i < 18; i++ )
//    {
//        // Convert the data stored in memory into floats
//        memcpy( &misalign_hex[i], &gCalibration.misalign[i], sizeof( uint32_t ) );
//        gCalibration.misalign[i] = INT32_TO_MISALIGN_SCALING * (float)misalign_hex[i];
//    }
}

// borrowing the AnalogFilterClocks from the configuration to allow
// the filtering to be changed on the fly.
uint32_t _GetFilter(uint32_t counts)
{
    if (counts > 18749 ) {
        return IIR_02HZ_LPF;
    } else if ( (counts <= 18749) && (counts > 8034) ) {
        return IIR_05HZ_LPF;
    } else if ( (counts <= 8034) && (counts > 4017) ) {
        return IIR_10HZ_LPF;
    } else if ( (counts <= 4017) && (counts > 2410) ) {
        return IIR_20HZ_LPF;
    } else if ( (counts <= 2410) && (counts > 1740) ) {
        return IIR_25HZ_LPF;
    } else if ( (counts <= 1740) && (counts > 1204) ) {
        return IIR_40HZ_LPF;
    } else if ( (counts <= 1204) && (counts > 0) ) {
        return IIR_50HZ_LPF;
    } else if (counts == 0) {
        return UNFILTERED;
    } else {
        return 0;  // never hit this, just here to remove compiler warning
    }
}


/** ***************************************************************************
 * @name CalibrateFilter() apply selected DSP filter to raw sensor counts.
 * Moved to just after data collection to avoid applying calibration to
            unfiltered data.
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
void CalibrateFilter(int idx)
{
    uint8_t sensor = 0;
    static int firstTime = 1;
    
    if(firstTime){
        firstTime = 0;
        FilterInit(200);
    }



    // gSensorsData.rawSensors is uint32_t (the data stored in x)
    static int32_t  fir_5Hz_x [NUM_SENSOR_CHIPS][NUM_SENSOR_READINGS][24]; // [11][24] raw + delay data
    static int32_t  fir_10Hz_x[NUM_SENSOR_CHIPS][NUM_SENSOR_READINGS][12]; // [11][12]
    static int32_t  fir_20Hz_x[NUM_SENSOR_CHIPS][NUM_SENSOR_READINGS] [6]; // [11][6]
    static int32_t  fir_40Hz_x[NUM_SENSOR_CHIPS][NUM_SENSOR_READINGS] [3]; // [11][4]

    if( fSPI ) {
        /// Filter the data based on the system configuration
        switch(configGetSensorFilterTypeForSPI())
        {   /// Filter selection based on SPI data-register
            case UNFILTERED:
              // Do nothing
                break;
            case FIR_40HZ_LPF: // Bartlett
                for( sensor = XACCEL; sensor <= ZMAG; sensor++ ) {
                    Apply_Bartlett_Q27_Filter(idx, &firTaps_40_Hz, sensor,
                                               fir_40Hz_x[idx][sensor] );
                }
                break;
            case FIR_20HZ_LPF: // Bartlett
                for( sensor = XACCEL; sensor <= ZMAG; sensor++ ) {
                    Apply_Bartlett_Q27_Filter( idx,  &firTaps_20_Hz,
                                               sensor,
                                               fir_20Hz_x[idx][sensor] );
                }
                break;
            case FIR_10HZ_LPF: // Bartlett
                for( sensor = XACCEL; sensor <= ZMAG; sensor++ ) {
                    Apply_Bartlett_Q27_Filter(idx, &firTaps_10_Hz,
                                               sensor,
                                               fir_10Hz_x[idx][sensor] );
                }
                break;
            case FIR_05HZ_LPF: // Bartlett
                for( sensor = XACCEL; sensor <= ZMAG; sensor++ ) {
                    Apply_Bartlett_Q27_Filter(idx, &firTaps_5_Hz,
                                               sensor,
                                               fir_5Hz_x[idx][sensor] );
                }
                break;
            case IIR_50HZ_LPF: // Butterworth
                for( sensor = XACCEL; sensor <= ZMAG; sensor++ ) {
                    Apply_Butterworth_Q27_Filter(idx, &iirTaps_50_Hz,
                                                 sensor);
                }
                break;
            case IIR_20HZ_LPF: // Butterworth
                for( sensor = XACCEL; sensor <= ZMAG; sensor++ ) {
                    Apply_Butterworth_Q27_Filter(idx, &iirTaps_20_Hz,
                             sensor);
                }
                break;
            case IIR_10HZ_LPF: // Butterworth
                for( sensor = XACCEL; sensor <= ZMAG; sensor++ ) {
                    Apply_Butterworth_Q27_Filter(idx, &iirTaps_10_Hz,
                             sensor);
                }
                break;
            case IIR_05HZ_LPF: // Butterworth
                for( sensor = XACCEL; sensor <= ZMAG; sensor++ ) {
                    Apply_Butterworth_Q27_Filter(idx, &iirTaps_5_Hz,
                                                 sensor);
                }
                break;
            default: /// 50 Hz LPF (Butterworth)
                for( sensor = XACCEL; sensor <= ZMAG; sensor++ ) {
                    Apply_Butterworth_Q27_Filter(idx, &iirTaps_50_Hz,
                                                 sensor);
                }
                break;
        }
    } else
        { /// UART - 50 Hz LPF (Butterworth)
        // Filter accelerometer signal
        switch (_GetFilter(configGetAccelLfpFreq())) {
            case IIR_50HZ_LPF: // Butterworth
                for( sensor = XACCEL; sensor <= ZACCEL; sensor++ )
                    Apply_Butterworth_Q27_Filter(idx, &iirTaps_50_Hz, sensor);
                break;
            case IIR_40HZ_LPF: // Butterworth
                for( sensor = XACCEL; sensor <= ZACCEL; sensor++ )
                    Apply_Butterworth_Q27_Filter(idx, &iirTaps_40_Hz, sensor);
                break;
            case IIR_25HZ_LPF: // Butterworth
                for( sensor = XACCEL; sensor <= ZACCEL; sensor++ )
                    Apply_Butterworth_Q27_Filter(idx, &iirTaps_25_Hz, sensor);
                break;
            case IIR_20HZ_LPF: // Butterworth
                for( sensor = XACCEL; sensor <= ZACCEL; sensor++ )
                    Apply_Butterworth_Q27_Filter(idx, &iirTaps_20_Hz, sensor);
                break;
            case IIR_10HZ_LPF: // Butterworth
                for( sensor = XACCEL; sensor <= ZACCEL; sensor++ )
                    Apply_Butterworth_Q27_Filter(idx, &iirTaps_10_Hz, sensor);
                break;
            case IIR_05HZ_LPF: // Butterworth
                for( sensor = XACCEL; sensor <= ZACCEL; sensor++ )
                    Apply_Butterworth_Q27_Filter(idx, &iirTaps_5_Hz, sensor);
                break;
            case IIR_02HZ_LPF:
                for( sensor = XACCEL; sensor <= ZACCEL; sensor++ )
                    Apply_Butterworth_Q27_Filter(idx, &iirTaps_2_Hz, sensor);
                break;
            case UNFILTERED:
                break;
            default:    // Butterworth 25 Hz
                for( sensor = XACCEL; sensor <= ZACCEL; sensor++ )
                    Apply_Butterworth_Q27_Filter(idx, &iirTaps_25_Hz, sensor);
                break;
      }

      // Filter rate-sensor signal
      switch (_GetFilter(configGetRateLfpFreq())) {
          case IIR_50HZ_LPF: // Butterworth
              for( sensor = XRATE; sensor <= ZRATE; sensor++ )
                  Apply_Butterworth_Q27_Filter(idx, &iirTaps_50_Hz, sensor);
              break;
          case IIR_40HZ_LPF: // Butterworth
              for( sensor = XRATE; sensor <= ZRATE; sensor++ )
                  Apply_Butterworth_Q27_Filter(idx, &iirTaps_40_Hz, sensor);
              break;
          case IIR_25HZ_LPF: // Butterworth
              for( sensor = XRATE; sensor <= ZRATE; sensor++ )
                  Apply_Butterworth_Q27_Filter(idx, &iirTaps_25_Hz, sensor);
              break;
          case IIR_20HZ_LPF: // Butterworth
              for( sensor = XRATE; sensor <= ZRATE; sensor++ )
                  Apply_Butterworth_Q27_Filter(idx, &iirTaps_20_Hz, sensor);
              break;
          case IIR_10HZ_LPF: // Butterworth
              for( sensor = XRATE; sensor <= ZRATE; sensor++ )
                  Apply_Butterworth_Q27_Filter(idx, &iirTaps_10_Hz, sensor);
              break;
          case IIR_05HZ_LPF: // Butterworth
              for( sensor = XRATE; sensor <= ZRATE; sensor++ )
                  Apply_Butterworth_Q27_Filter(idx, &iirTaps_5_Hz, sensor);
              break;
            case IIR_02HZ_LPF:
                for( sensor = XRATE; sensor <= ZRATE; sensor++ )
                    Apply_Butterworth_Q27_Filter(idx, &iirTaps_2_Hz, sensor);
                break;
          case UNFILTERED:
              break;
          default:
              // Butterworth 25 HZ
              for( sensor = XRATE; sensor <= ZRATE; sensor++ )
                  Apply_Butterworth_Q27_Filter(idx, &iirTaps_25_Hz, sensor);
              break;
        }

        // Filter magnetometer signal
//        for( sensor = XMAG; sensor <= ZMAG; sensor++ ) {
//            Apply_Butterworth_Q27_Filter(idx, &iirTaps_5_Hz,
//                                         sensor);
//        }
    }
}

/// @brief
/// ============ DMU380: Coordinate-frame definition ============
///
/// Memsic refers to the heritage (Nav-View) definition
/// JD refers to the OEM coordinate frame (preferred by John Deere)
///   (Verified: February 26, 2013)
///                                          _
///            -z-axis (Memsic)  ^           /| -y-axis (Memsic)
///                +z-axis (JD)  |          /   +x-axis (JD)
///                              |         /
///                              |        /
///                              |       /
///                        ______|______/_________
///                      /       |               /|
///                     /        |              / |
///                    /         |             /  |
///                   /          |            /   |
///  -x-axis (Memsic)/           |           /    /
///  +y-axis (JD)   /                       /    /
///   <------------/                       /    /
///               /     _____________     /    /
///              /    /|            /    /    /
///             /____/ |___________/____/    /
///             |    | /           |    |   /
///             |    |/____________|    |  /
///             |                       | /
///             |_______________________|/
///

/** ***************************************************************************
 * @name CalibrateApply() calculates calibrated sensor variables
 * @brief The input is the raw readings (accels, gyros, mags, temp as described
 *        in eSensorOrder. The gain for each sensor is also passed in.
 *
 * The output is in scaledSensors, the results are in g's, rad/s, G, deg C,
 *  (rotated to user body frame)
 *
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
void CalibrateApply(int idx)
{
    float  scaledSensors[NUM_SENSOR_READINGS]     = { 0.0 }; // [11]

    memset(gSensorsData.scaledSensors[idx], 0, sizeof(gSensorsData.scaledSensors[0]));
    memset(scaledSensors, 0, sizeof(scaledSensors));

    // scaled board and accelerometer temperature
    // Tmax = ( X [cnts] - 6400 ) * ( 1/256 [degC/cnt] ) + 25 [degC]
//    gSensorsData.scaledSensors[idx][BTEMP]  = BOARD_TEMP_SCALE_FACTOR * (float)( CONVERT_XBOW_TO_380( gSensorsData.rawSensors[idx][BTEMP] << 16 ) );
    gSensorsData.scaledSensors[idx][BTEMP]  = BOARD_TEMP_SCALE_FACTOR * (float)(gSensorsData.rawSensors[idx][BTEMP]);
    gSensorsData.scaledSensors[idx][XATEMP] = gSensorsData.scaledSensors[idx][BTEMP];
    gSensorsData.scaledSensors[idx][YATEMP] = gSensorsData.scaledSensors[idx][BTEMP];
    gSensorsData.scaledSensors[idx][ZATEMP] = gSensorsData.scaledSensors[idx][BTEMP];
    // rate-sensor temperature
//    gSensorsData.scaledSensors[idx][XRTEMP] = MAXIM21000_TEMP_SCALE_FACTOR * (float)( CONVERT_XBOW_TO_380( gSensorsData.rawSensors[idx][ XRTEMP ] << 2 ) );
    gSensorsData.scaledSensors[idx][XRTEMP] = MAXIM21000_TEMP_SCALE_FACTOR * (float)(gSensorsData.rawSensors[idx][ XRTEMP ]);
    gSensorsData.scaledSensors[idx][YRTEMP] = gSensorsData.scaledSensors[idx][XRTEMP];
    gSensorsData.scaledSensors[idx][ZRTEMP] = gSensorsData.scaledSensors[idx][XRTEMP];

    /// temperature-dependent bias correction to rate-sensor and accelerometer
    /// scale factor correction to rate-sensors and  acclerometers

    /// -- Accelerometer ( negative sign to get the correct value) --
    _ApplySensorCalibration(idx, XACCEL, &scaledSensors[XACCEL], XATEMP );

    _ApplySensorCalibration (idx,XRATE,  &scaledSensors[XRATE], XATEMP);

    _orientSensors(scaledSensors); ///< based on faces
    _LimitSensorValues( scaledSensors ); ///< based on the range of the sensors

    // Populate the structure used in the program (by EKF and output functions)
    //   (Error in previous implementation.  Overwrote data with zeros.  The
    //   following fixes that.)
    for(int i = 0; i < 9; i++){
        gSensorsData.scaledSensors[idx][i] = scaledSensors[i];
    }

}

/** ***************************************************************************
 * @name _ApplySensorCalibration() LOCAL
 * @brief apply temperature dependant bias and scale factor corrections
 *
 * @param [in] sensor - index
 * @param [in] tempSensor - sensor index
 * @retval N/A
 ******************************************************************************/
static void _ApplySensorCalibration( int idx, int sensor, float *scaledData, int tempSensor )
{
    volatile static int32_t tempCompBias[3];
    int      rawReadingMinusBias[3];

    if( sensor < XMAG ) { // accel and rate
        /// Temperature-dependent bias correction (in raw sensor counts)
        for(int i = 0; i < 3; i++){
            tempCompBias[i]        = _searchTempTable(idx, sensor + i, gSensorsData.rawSensors[idx][tempSensor+i] );
            rawReadingMinusBias[i] = gSensorsData.rawSensors[idx][sensor + i] - tempCompBias[i];
        }
        _AdjustForSensorsMisalignment(idx, sensor, rawReadingMinusBias);
        for(int i = 0; i < 3; i++){
            scaledData[i] = _searchScaleTable(idx,  sensor + i, rawReadingMinusBias[i]);
        }
    }
}

/** ***************************************************************************
 * @name _AdjustForSensorsMisalignment() LOCAL
 * @brief apply coodinate misalignment to ortho sensor coordinates
 *
 * @param [in] sensor - index
 * @param [in] scaledSensors - sensor data
 * @retval N/A
 ******************************************************************************/
static void _AdjustForSensorsMisalignment( int idx, int  sensor, int *rawSensors)
{
    volatile float tempX;
    volatile float tempY;
    volatile float tempZ;
    volatile float misalign[6] = { 0.0 };

    misalign[0] = gCalibration[idx].misalign[2 * sensor + 0];
    misalign[1] = gCalibration[idx].misalign[2 * sensor + 1];
    misalign[2] = gCalibration[idx].misalign[2 * sensor + 2];
    misalign[3] = gCalibration[idx].misalign[2 * sensor + 3];
    misalign[4] = gCalibration[idx].misalign[2 * sensor + 4];
    misalign[5] = gCalibration[idx].misalign[2 * sensor + 5];
    
    tempX       = rawSensors[0];
    tempY       = rawSensors[1];
    tempZ       = rawSensors[2];

    rawSensors[0] = round(              tempX + misalign[0] * tempY + misalign[1]  * tempZ);
    rawSensors[1] = round(misalign[2] * tempX +               tempY + misalign[3]  * tempZ);
    rawSensors[2] = round(misalign[4] * tempX + misalign[5] * tempY +                tempZ);

}

// FIXME: remove mod operator since it has been shown to work
#define NO_MOD_OPER 1

/** ***************************************************************************
 * @name _orientSensors() LOCAL rotates the accels, rates, mags, and temps to the
 *       user defined coordinate frame
 * @brief (does not preserve right-handed frame; responsibility of the user?
 *         or is this done in Nav-View or an packet-handler?)
 * @author Darren Liccardo, Aug. 2005
 * @param [in] scaledSensors - sensor data
 * @retval N/A
 ******************************************************************************/
void _orientSensors( float *scaledSensors )
{
   int i;
   int j;
   float temp[NUM_AXIS];

   uint16_t orientation = configGetOrientation();
   int fwdSign, rightSign, downSign;
   int fwdAxis, rightAxis, downAxis;

   fwdSign   = orientation & 0x0001;
   fwdAxis   = (orientation >> 1) & 0x0003;

   rightSign = (orientation >> 3) & 0x0001;
   rightAxis = (orientation >> 4) & 0x0003;

   downSign  = (orientation >> 6) & 0x0001;
   downAxis  = (orientation >> 7) & 0x0003;

   for( i = 0; i < NUM_SENSOR_IN_AXIS; i += NUM_SENSORS )
   {
      // Store sensor data for next interation of swapping
      for( j = 0; j < NUM_AXIS; j++ ) {
         temp[j] = scaledSensors[i + j];
      }

      // Adjust the sign of the sensor data if the configuration bit is set
      if( fwdSign ) {
         scaledSensors[i] = -temp[fwdAxis];
      } else {
         scaledSensors[i] =  temp[fwdAxis];
      }

      // Adjust the sign of the sensor data if the configuration bit is set
      uint8_t index;
      // Add switch to get rid of mod-operation (this is run 3x per each pass
      //   through the calibrate routine)
      switch( rightAxis ) {
           case 0:
               index = 1;
               break;
           case 1:
               index = 2;
               break;
           case 2:
               index = 0;
               break;
           default:
                while(1){};
               // error if configuration is other than 0,1, or 2
               ;
      }

      if(rightSign) {
         scaledSensors[i + 1] = -temp[index];
      } else {
         scaledSensors[i + 1] =  temp[index];
      }

      // Add switch to get rid of mod-operation
      switch( downAxis ) {
           case 0:
               index = 2;
               break;
           case 1:
               index = 0;
               break;
           case 2:
               index = 1;
               break;
           default:
               // error if configuration is other than 0,1, or 2
               ;
      }

      // Adjust the sign of the sensor data if the configuration bit is set
      if(downSign) {
         scaledSensors[i + 2] = -temp[index];
      } else {
         scaledSensors[i + 2] =  temp[index];
      }
   }
}


/** ***************************************************************************
 * @name _LimitSensorValues()
 * @brief set a flag for SPI data and limit the sensor values based
*         upon the database value - rate limited to 400dps for ITAR
 *
 * @param [in] scaledSensors - sensor data
 * @retval N/A
 ******************************************************************************/
void _LimitSensorValues( float *scaledSensors )
{
    uint8_t temp = 0;

    /// limit rates for ITAR
    /// ---------- Rate Sensor Values ----------
    /// X-Axis
    if ( scaledSensors[ XRATE ] > gSensorsData.rateAlarm ) {
        if ( scaledSensors[ XRATE ] > ITAR_RATE_LIMIT ) {
            scaledSensors[ XRATE ] = ITAR_RATE_LIMIT;
        }
        temp = temp || 1;
    } else if ( scaledSensors[ XRATE ] < -gSensorsData.rateAlarm ) {
        if ( scaledSensors[ XRATE ] < -ITAR_RATE_LIMIT ) {
            scaledSensors[ XRATE ] = -ITAR_RATE_LIMIT;
        }
        temp = temp || 1;
    }

    /// Y-Axis
    if ( scaledSensors[ YRATE ] > gSensorsData.rateAlarm ) {
        if ( scaledSensors[ YRATE ] > ITAR_RATE_LIMIT ) {
            scaledSensors[ YRATE ] =  ITAR_RATE_LIMIT;
        }
        temp = temp || 1;
    } else if ( scaledSensors[ YRATE ] < -gSensorsData.rateAlarm )  {
        if ( scaledSensors[ YRATE ] < -ITAR_RATE_LIMIT ) {
            scaledSensors[ YRATE ] = -ITAR_RATE_LIMIT;
        }
        temp = temp || 1;
    }

    /// Z-Axis
    if ( scaledSensors[ ZRATE ] > gSensorsData.rateAlarm ) {
        if ( scaledSensors[ ZRATE ] > ITAR_RATE_LIMIT ) {
            scaledSensors[ ZRATE ] =   ITAR_RATE_LIMIT;
        }
        temp = temp || 1;
    } else if ( scaledSensors[ ZRATE ] < -gSensorsData.rateAlarm ) {
        if ( scaledSensors[ ZRATE ] < -ITAR_RATE_LIMIT ) {
            scaledSensors[ ZRATE ] =  -ITAR_RATE_LIMIT;
        }
        temp = temp || 1;
    }

/*
    if( platformGetUnitCommunicationType() == SPI_COMM )
    {    /// limits apply for the SPI implementation of the software
        ///   - FLAG the data
        /// ---------- Accelerometer Values ----------
        /// X-Axis
        if( ( scaledSensors[ XACCEL ] > gSensorsData.Limit.accelAlarm ) ||
            ( scaledSensors[ XACCEL ] < -gSensorsData.Limit.accelAlarm ) ) {
            temp = temp || 1;
        }
        /// Y-Axis
        if( ( scaledSensors[ YACCEL ] > gSensorsData.Limit.accelAlarm ) ||
            ( scaledSensors[ YACCEL ] < -gSensorsData.Limit.accelAlarm ) ) {
            temp = temp || 1;
        }
        /// Z-Axis
        if( ( scaledSensors[ ZACCEL ] > gSensorsData.Limit.accelAlarm ) ||
            ( scaledSensors[ ZACCEL ] < -gSensorsData.Limit.accelAlarm ) ) {
            temp = temp || 1;
        }

        /// If a rate-sensor or accelerometer exceed the defined limit then set
        ///   the over-range bit in the diagnostic register.
        if( temp != 0 ) {
            gUserSpi.DataRegister[ 0x3D ] = gUserSpi.DataRegister[ 0x3D ] | 0x10;
            gSensorsData.bitStatus.swAlgBIT.bit.overRange = 1;
        } else {
            // Clear the bit
            gUserSpi.DataRegister[ 0x3D ] = gUserSpi.DataRegister[ 0x3D ] & 0xEF;
            gSensorsData.bitStatus.swAlgBIT.bit.overRange = 0;
        }
        /// No saturation for Magnetometers
    }
    else
*/    
    {
        // UART implementation (Nav-View) of the software
    }
}



void InitFactoryCalibration(void)
{
    
    for(int i = 0; i < NUM_SENSOR_CHIPS; i++){
        /// CRC the calibration area and set BIT appropriately
        _readCalIntoMem(i);
        calibrateTableValid[i] = _CheckCalibrationCrc(i);
    }

    if(!calibrateTableValid[0] && !fSPI && !fUART){
        HW_SetTestMode(TRUE);
    }
    
    gCalibration[0].productConfiguration.bit.hasGps = FALSE;
    
    gCalibration[0].RollIncidenceLimit  = ROLL_INCIDENCE_LIMIT; // 0x1000
    gCalibration[0].PitchIncidenceLimit = PITCH_INCIDENCE_LIMIT; // 0x1000
    gCalibration[0].HardIronLimit       = HARD_IRON_LIMIT; // 0x6666
    gCalibration[0].SoftIronLimit       = SOFT_IRON_LIMIT; // 0x4000

    CalibrateInit();        /// calibrate.c  Copy cal data into the table
}

uint32_t GetUnitSerialNum()
{
    return gCalibration[0].serialNumber; 
}

int  CalibrationTableValid(int idx)
{
    return calibrateTableValid[idx];
}

void  ApplyFactoryCalibration(void)
{
    FillRawSensorsData();
    
    for(int i = 0; i < NUM_SENSOR_CHIPS; i++){
        if(CalibrationTableValid(i)) {
            CalibrateFilter(i);
            CalibrateApply(i);
        }
    }
    
    CombineSensorsData();
}

uint16_t GetProductConfiguration()
{
    return gCalibration[0].productConfiguration.all;
}

char *GetUnitVersion()
{
    return gCalibration[0].versionString; 
}
