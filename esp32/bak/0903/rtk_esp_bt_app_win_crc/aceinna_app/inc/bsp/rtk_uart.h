/*******************************************************************************
* File Name          : rtk_uart.h
* Author             : Daich
* Revision           : 1.0
* Date               : 29/05/2019
* Description        : bluetooth
*
* HISTORY***********************************************************************
* 29/05/2019  |                                             | Daich
*
*******************************************************************************/
#ifndef _RTK_UART_H
#define _RTK_UART_H
#include "driver/uart.h"


void uart_task(void *pvParameters);
void uart_recevive_task_create();
void uart_receive_task(void *pvParameters);
void uart_write_data(uart_port_t uart_num, const char* src, size_t size);
char* get_device_name();
typedef enum bt_uart_baud
{
    baud_115200 = 0,
    baud_460800,
    baud_921600
}bt_uart_baud_e;

#endif
