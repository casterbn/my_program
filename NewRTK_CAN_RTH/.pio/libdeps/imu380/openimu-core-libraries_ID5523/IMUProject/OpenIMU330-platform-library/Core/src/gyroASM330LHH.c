/** ***************************************************************************
* @file gyroBMI16.c Gyroscope functions for the ASM330LHH gyro
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

//*************************
#include <stdint.h>
#include <string.h>
#include "GlobalConstants.h"
#include "gyroscope.h"
#include "gyroASM330LHH.h"
#include "SpiBitBang.h"
#include "Indices.h"
#include "sensor.h"
#include "osapi.h"

static uint8_t testPassed = 0;

uint8_t isGyroPowerOn(int chip)
{
    uint8_t regs[NUM_SENSOR_CHIPS];
    
    SpiBitBangSetBaud(SPI_MIN_SPEED);
    
    _ReadGyroRegister(ASM330LHH_CTRL2_G, &regs[0], &regs[1], &regs[2]);
    
    SpiBitBangSetBaud(SPI_MAX_SPEED);
    
    if ((regs[chip] & 0xF0) != 0)
        return true;
    
    return false;
}

uint8_t powerOnASM330LHHGyro(void)
{
    uint8_t regs[NUM_SENSOR_CHIPS];
    uint8_t cmd = ASM330LHH_gyro_normal_mode;
    uint8_t res  [NUM_SENSOR_CHIPS];
    int     gcnt = 0, cnt = 0;
    uint16_t mask = 0x0001;
    
    SpiBitBangSetBaud( SPI_MIN_SPEED );

    _WriteGyroRegister(ASM330LHH_CTRL2_G, cmd);
    OS_Delay(1);
    _ReadGyroRegister(ASM330LHH_CTRL2_G, &regs[0], &regs[1], &regs[2]);
    for(int i = 0; i < NUM_SENSOR_CHIPS; i++){
        if(globalSensorsMask & mask){
            gcnt++;
            if(regs[i] == cmd){
                res[i] = cmd;
            }
            if(res[i] == cmd){
                cnt++;
            }
        }
        mask <<= 1;
    }
    
    SpiBitBangSetBaud( SPI_MAX_SPEED );
    
    return cnt == gcnt;
}

uint8_t powerOffSM330LHHGyro(void)
{
    uint8_t regs[NUM_SENSOR_CHIPS];
    uint8_t cmd = ASM330LHH_gyro_suspend_mode;
    uint8_t res  [NUM_SENSOR_CHIPS];
    int     gcnt = 0, cnt = 0;
    uint16_t mask = 0x0001;
    
    SpiBitBangSetBaud( SPI_MIN_SPEED );

    _WriteGyroRegister(ASM330LHH_CTRL2_G, cmd);
    OS_Delay(1);
    _ReadGyroRegister(ASM330LHH_CTRL2_G, &regs[0], &regs[1], &regs[2]);
    for(int i = 0; i < NUM_SENSOR_CHIPS; i++){
        if(globalSensorsMask & mask){
            gcnt++;
            if(regs[i] == cmd){
                res[i] = cmd;
            }
            if(res[i] == cmd){
                cnt++;
            }
        }
        mask <<= 1;
    }
    
    SpiBitBangSetBaud( SPI_MAX_SPEED );
    
    return cnt == gcnt;
}

uint8_t isAccelPowerOn(int chip)
{
    uint8_t regs[NUM_SENSOR_CHIPS];
    
    SpiBitBangSetBaud( SPI_MIN_SPEED); // go slow  ~0.5  Mhz
    
    _ReadGyroRegister(ASM330LHH_CTRL1_XL, &regs[0], &regs[1], &regs[2]);
    
    SpiBitBangSetBaud(SPI_MAX_SPEED); // go fast ~7.5 MHz
    
    if ( (regs[chip] & 0xF0) != 0)
        return true;
    
    return false;
}

uint8_t powerOnASM330LHHAccel(void)
{
    uint8_t regs[NUM_SENSOR_CHIPS];
    uint8_t res [NUM_SENSOR_CHIPS];
    uint8_t  cmd = ASM330LHH_accel_normal_mode;
    int      gcnt = 0, cnt = 0;
    uint16_t mask = 0x0001;
    
    SpiBitBangSetBaud( SPI_MIN_SPEED );
    
    _WriteGyroRegister(ASM330LHH_CTRL1_XL, cmd);
    OS_Delay(1);
    _ReadGyroRegister(ASM330LHH_CTRL1_XL, &regs[0], &regs[1], &regs[2]);

    for(int i = 0; i < NUM_SENSOR_CHIPS; i++){
        if(globalSensorsMask & mask){
            gcnt++;
            if(regs[i] == cmd){
                res[i] = cmd;
            }
            if(res[i] == cmd){
                cnt++;
            }
        }
        mask <<= 1;
    }
    
    SpiBitBangSetBaud( SPI_MAX_SPEED );
    
    return cnt == gcnt;
}

uint8_t powerOffASM330LHHAccel(void)
{
    uint8_t regs[NUM_SENSOR_CHIPS];
    uint8_t cmd = ASM330LHH_accel_suspend_mode;
    uint8_t res  [NUM_SENSOR_CHIPS];
    int     gcnt = 0, cnt = 0;
    uint16_t mask = 0x0001;
    
    SpiBitBangSetBaud( SPI_MIN_SPEED );
    
    _WriteGyroRegister(ASM330LHH_CTRL1_XL, cmd);
    OS_Delay(1);
    _ReadGyroRegister(ASM330LHH_CTRL1_XL, &regs[0], &regs[1], &regs[2]);
    for(int i = 0; i < NUM_SENSOR_CHIPS; i++){
        if(globalSensorsMask & mask){
            gcnt++;
            if(regs[i] == cmd){
                res[i] = cmd;
            }
            if(res[i] == cmd){
                cnt++;
            }
        }
        mask <<= 1;
    }
    
    SpiBitBangSetBaud( SPI_MAX_SPEED );

    return cnt == gcnt;
}


/*
uint8_t isErrASM330LHH(void)
{
    uint8_t regs[NUM_SENSOR_CHIPS];
    
    SpiSetBaud(SPI_MIN_SPEED);
    
    _ReadGyroRegister(ASM330LHH_CTRL1_XL, &regs[0], &regs[1], &regs[2]);
    
    SpiSetBaud(SPI_MAX_SPEED);
    
    for(int i = 0; i < NUM_SENSOR_CHIPS; i++){
        if((regs[i] & 0x1f) != 0){
            return true; 
        }
    }
    
    return false;
}
*/

uint8_t isASM330LHH(void)
{
    uint8_t regs[NUM_SENSOR_CHIPS];
    uint8_t res  [NUM_SENSOR_CHIPS];
    int     gcnt = 0, cnt = 0;
    uint8_t retries = 3;
    uint16_t mask = 0x0001;
    
    memset(res, 0, sizeof(res));
    
    SpiBitBangSetBaud(SPI_MIN_SPEED);
    
    while (retries--) {
        cnt = 0;
        _ReadGyroRegister(ASM330LHH_WHO_AM_I, &regs[0], &regs[1], &regs[2]);
        for(int i = 0; i < NUM_SENSOR_CHIPS; i++){
            if(globalSensorsMask & mask){
                gcnt++;
                if(regs[i] == ASM330LHH_CHIP_ID){
                    res[i] = ASM330LHH_CHIP_ID;
                }
                if(res[i] == ASM330LHH_CHIP_ID){
                    cnt++;
                }
            }
            mask <<= 1;
        }
        if(cnt == gcnt){
            break;
        }
    }
    
    return cnt == gcnt;
}


int ASM330LHHConfig()
{
    uint8_t regs[NUM_SENSOR_CHIPS];
    uint8_t res  [NUM_SENSOR_CHIPS];
    int     gcnt, cnt;
    uint8_t cmd;
    uint16_t mask;

    // set low speed on SPI
    SpiBitBangSetBaud( SPI_MIN_SPEED);

    cmd  = 0x20;     // Accel Band = ODR/10 -> ~80 Hz
    cnt  = 0;
    gcnt = 0;
    mask = 0x0001;
    _WriteGyroRegister(ASM330LHH_CTRL8_XL, cmd);
    OS_Delay(1);
    _ReadGyroRegister(ASM330LHH_CTRL8_XL, &regs[0], &regs[1], &regs[2]);

    for(int i = 0; i < NUM_SENSOR_CHIPS; i++){
        if(globalSensorsMask & mask){
            gcnt++;
            if(regs[i] == cmd){
                res[i] = cmd;
            }
            if(res[i] == cmd){
                cnt++;
            }
        }
        mask <<= 1;
    }
    
    if (cnt != gcnt){
        return 0;
    }

    cmd  = 0x05;     // Gyro LPF1 band   - 50 Hz 1670 ODR
    cnt  = 0;
    gcnt = 0;
    mask = 0x0001;

    _WriteGyroRegister(ASM330LHH_CTRL6_G, cmd);
    OS_Delay(1);
    _ReadGyroRegister(ASM330LHH_CTRL6_G, &regs[0], &regs[1], &regs[2]);

    for(int i = 0; i < NUM_SENSOR_CHIPS; i++){
        if(globalSensorsMask & mask){
            gcnt++;
            if(regs[i] == cmd){
                res[i] = cmd;
            }
            if(res[i] == cmd){
                cnt++;
            }
        }
        mask <<= 1;
    }
    
    if (cnt != gcnt){
        return 0;
    }

    cmd  = 0x02;     // Enable Gyro LPF1
    cnt  = 0;
    gcnt = 0;
    mask = 0x0001;
    _WriteGyroRegister(ASM330LHH_CTRL4_C, cmd);
    OS_Delay(1);
    _ReadGyroRegister(ASM330LHH_CTRL4_C, &regs[0], &regs[1], &regs[2]);

    for(int i = 0; i < NUM_SENSOR_CHIPS; i++){
        if(globalSensorsMask & mask){
            gcnt++;
            if(regs[i] == cmd){
                res[i] = cmd;
            }
            if(res[i] == cmd){
                cnt++;
            }
        }
        mask <<= 1;
    }
    
    if (cnt != gcnt){
        return 0;
    }
    
    return 1;
}



/** ****************************************************************************
* @name GyroSelfTest_Passed This function retrieves gyro test results
*
* @param [in] apply - flag to set or unset the bias
* @retval N/A
******************************************************************************/
uint8_t GyroSelfTest_Passed()
{
    return testPassed != 0;
}

uint8_t ASM330LHHSetAddrIncMode(void)
{
    uint8_t regs[NUM_SENSOR_CHIPS];
    uint8_t cmd   = 0x44;    // BDU do not update, INT open drain addr inc enabled
    uint16_t mask = 0x0001;
    int cnt = 0, gcnt = 0;
    uint8_t res[NUM_SENSOR_CHIPS];
    
    SpiBitBangSetBaud( SPI_MIN_SPEED );
 
    _WriteGyroRegister(ASM330LHH_CTRL3_C, cmd);
    OS_Delay(1);
    _ReadGyroRegister(ASM330LHH_CTRL3_C, &regs[0], &regs[1], &regs[2]);
    for(int i = 0; i < NUM_SENSOR_CHIPS; i++){
        if(globalSensorsMask & mask){
            gcnt++;
            if(regs[i] == cmd){
                res[i] = cmd;
            }
            if(res[i] == cmd){
                cnt++;
            }
        }
        mask <<= 1;
    }
    
    SpiBitBangSetBaud( SPI_MAX_SPEED );
    return cnt == gcnt;
}

void ASM330LHHSync()
{
    uint8_t regs[NUM_SENSOR_CHIPS];
    
    _ReadGyroRegister(0x7F, &regs[0], &regs[1], &regs[2]);
}


int ASM330LHHSoftReset()
{
    uint8_t cmd = 0x01;
    
    SpiBitBangSetBaud( SPI_MIN_SPEED );

    OS_Delay(10);

    _WriteGyroRegister(ASM330LHH_CTRL3_C, cmd);

    OS_Delay(10);

    ASM330LHHSync();

    return true;

}


uint8_t ASM330LHHSetInertialSensorsBDR(void)
{
    uint8_t regs[NUM_SENSOR_CHIPS];
    uint8_t cmd   = 0x77;      // batching rate at 833Hz
    uint16_t mask = 0x0001;
    int cnt = 0, gcnt = 0;
    uint8_t res[NUM_SENSOR_CHIPS];
    
    SpiBitBangSetBaud( SPI_MIN_SPEED );
 
    _WriteGyroRegister(ASM330LHH_FIFO_CTRL3, cmd);
    OS_Delay(1);
    _ReadGyroRegister(ASM330LHH_FIFO_CTRL3, &regs[0], &regs[1], &regs[2]);
    for(int i = 0; i < NUM_SENSOR_CHIPS; i++){
        if(globalSensorsMask & mask){
            gcnt++;
            if(regs[i] == cmd){
                res[i] = cmd;
            }
            if(res[i] == cmd){
                cnt++;
            }
        }
        mask <<= 1;
    }
    
    SpiBitBangSetBaud( SPI_MAX_SPEED );
    return cnt == gcnt;
}

uint8_t ASM330LHHEnableFifo(void)
{
    uint8_t regs[NUM_SENSOR_CHIPS];
    uint8_t cmd   = 0x36;           // fifo in continious mode, temp sampled at 52 Hz
    uint16_t mask = 0x0001;
    int cnt = 0, gcnt = 0;
    uint8_t res[NUM_SENSOR_CHIPS];
    
    SpiBitBangSetBaud( SPI_MIN_SPEED );
 
    _WriteGyroRegister(ASM330LHH_FIFO_CTRL4, cmd);
    OS_Delay(1);
    _ReadGyroRegister(ASM330LHH_FIFO_CTRL4, &regs[0], &regs[1], &regs[2]);
    for(int i = 0; i < NUM_SENSOR_CHIPS; i++){
        if(globalSensorsMask & mask){
            gcnt++;
            if(regs[i] == cmd){
                res[i] = cmd;
            }
            if(res[i] == cmd){
                cnt++;
            }
        }
        mask <<= 1;
    }
    
    SpiBitBangSetBaud( SPI_MAX_SPEED );
    return cnt == gcnt;
}


uint8_t ASM330LHHDisableFifo(void)
{
    uint8_t regs[NUM_SENSOR_CHIPS];
    uint8_t cmd   = 0x00;           // fifo in continious mode, temp sampled at 52 Hz
    uint16_t mask = 0x0001;
    int cnt = 0, gcnt = 0;
    uint8_t res[NUM_SENSOR_CHIPS];
    
    SpiBitBangSetBaud( SPI_MIN_SPEED );
 
    _WriteGyroRegister(ASM330LHH_FIFO_CTRL4, cmd);
    OS_Delay(1);
    _ReadGyroRegister(ASM330LHH_FIFO_CTRL4, &regs[0], &regs[1], &regs[2]);
    for(int i = 0; i < NUM_SENSOR_CHIPS; i++){
        if(globalSensorsMask & mask){
            gcnt++;
            if(regs[i] == cmd){
                res[i] = cmd;
            }
            if(res[i] == cmd){
                cnt++;
            }
        }
        mask <<= 1;
    }
    
    SpiBitBangSetBaud( SPI_MAX_SPEED );
    return cnt == gcnt;
}




