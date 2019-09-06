/**
******************************************************************************
* @file    hwAPI.h 
******************************************************************************
*/
#ifndef __HW_API_H
#define __HW_API_H

#include "stdint.h"
#include "GlobalConstants.h"

void    HW_Init();

// GPIO - related fucntions
void    HW_IO2_Init();
void    HW_IO3_Init();
void    HW_IO3_Toggle();
void    HW_IO2_Toggle();
void    HW_IO2_On();
void    HW_IO3_On();
void    HW_IO2_Off();
void    HW_IO3_Off();
void    HW_DRDY_Off();
void    HW_DRDY_On();
uint8_t HW_ReadConfig();
BOOL    HW_IsTestOK();

// SPI - related functions
void    SPI_ProcessCommand(uint8_t cmd);
void    SPI_ProcessData(uint8_t* in);
int     SPI_PrepareForDataTransmit(uint8_t *out, int outLen);
void    SPI_ActivateInterface();
void    SPI_ProcessBootData(uint8_t* in, int len, uint8_t *statusAddr);
void    SPI_ActivateInterfaceForBootLoading(uint16_t statusWord);
void    SPI_DeactivateInterface();

// UART - related functions
BOOL    UART_Init(int port, int baudrate);
int     UART_Read(int port, uint8_t *buffer, uint32_t size);
int     UART_Write(int port, uint8_t *buffer, uint32_t size);

// CAN - related functions
void    CAN_Init();
BOOL    CAN_Transmit(uint32_t id, uint8_t *data, int len); 
void    CAN_ConfigureCallback(void (*callback)(void));

// Timers -related functions

int     StartDacqTimer(int freq);
int     TIMER_IsDacqOverrun();
int     TIMER_WaitForNewDacqTick();
int     TIMER_GetSensToPpsDelay();
int     TIMER_GetPpsToDrdyDelay();
void    StartReferenceTimer(int precisionUs);

// system related functions
void    HW_SystemReset(void);
void    HW_JumpToApp();
void    HW_EnforceBootMode();


#define NUM_SERIAL_PORTS  2

#define USER_SERIAL_PORT   0
#define DEBUG_SERIAL_PORT  1

extern BOOL fSPI;
extern BOOL fUART;


#endif //__UART_H