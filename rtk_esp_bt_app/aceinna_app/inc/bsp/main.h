/*******************************************************************************
* File Name          : rtk_bt.h
* Author             : Daich
* Revision           : 1.0
* Date               : 13/05/2019
* Description        : bluetooth
*
* HISTORY***********************************************************************
* 13/05/2019  |                                             | Daich
*
*******************************************************************************/
#ifndef _MAIN_H
#define _MAIN_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"


#define ESP32_KIT
#ifdef ESP32_KIT
#define DEVICE_NAME "ESP32_BT_ACEINNA"
//#define UART_CHANNEL UART_NUM_1                    //16 tx   17 rx
#define UART_CHANNEL UART_NUM_0


#if UART_CHANNEL == UART_NUM_0
#define ECHO_TXD  UART_PIN_NO_CHANGE
#define ECHO_RXD  UART_PIN_NO_CHANGE
#else
UART_PIN_NO_CHANGE
#define ECHO_TXD  (16)
#define ECHO_RXD  (17)
//#define ECHO_TEST_TXD  (16)
//#define ECHO_TEST_RXD  (17)
#endif

#else
#define DEVICE_NAME "ESP32_BT"
#define UART_CHANNEL UART_NUM_0
#endif
#define SPP_TAG "SPP_ACCEPTOR_DEMO"
#define TAG "TEST"
#define SPP_SERVER_NAME "SPP_SERVER"


void take_bt_mutex(void);
void release_bt_mutex(void);
void set_bt_name();

#endif
