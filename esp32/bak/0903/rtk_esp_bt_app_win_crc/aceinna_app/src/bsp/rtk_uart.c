/*******************************************************************************
* File Name          : rtk_uart.c
* Author             : Daich
* Revision           : 1.0
* Date               : 28/05/2019
* Description        : bluetooth
*
* HISTORY***********************************************************************
* 28/05/2019  |                                             | Daich
*
*******************************************************************************/

#include "main.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "rtk_stdint.h"
#include "rtk_bt.h"
#include "rtk_uart.h"
#include "tool.h"
#include <string.h>
#include <stdio.h>
#include "json_parse.h"
QueueHandle_t uart0_queue;

#define BUF_SIZE (1024 * 1)

bt_uart_baud_e BAUD_RATE = baud_460800;

static void uart_init(int channel);
void uart_task(void *pvParameters);
void uart_recevive_task_create();
void uart_receive_task(void *pvParameters);
char device_name[40];
int config_json_received = 0;


static void uart_init(int channel)
{
    //int uart_num = channel;
    uart_config_t uart_config = {

       .baud_rate = 460800,
       //.baud_rate = 921600,
       //.baud_rate = 115200,
       .data_bits = UART_DATA_8_BITS,
       .parity = UART_PARITY_DISABLE,
       .stop_bits = UART_STOP_BITS_1,
       .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
       .rx_flow_ctrl_thresh = 122,
    };
    switch(BAUD_RATE)
    {
        case baud_115200:
            uart_config.baud_rate = 115200;
            break;
        case baud_460800:
            uart_config.baud_rate = 460800;
            break;
        case baud_921600:
            uart_config.baud_rate = 921600;
            break;
        default:
            break;
        
    }
    //Set UART parameters
    uart_param_config(channel, &uart_config);
    //uart_set_pin(channel, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, 18, 19);  
    uart_set_pin(channel, ECHO_TXD, ECHO_RXD, 18, 19);
    uart_driver_install(channel, BUF_SIZE * 2, BUF_SIZE * 2, 4, &uart0_queue, 0);
}



void uart_task(void *pvParameters)
{
    int uart_num = (int) pvParameters;
    uart_event_t event;
    size_t buffered_size;
    //uint8_t* dtmp = (uint8_t*) malloc(BUF_SIZE);
	uint8_t* uart_data = (uint8_t*) malloc(BUF_SIZE * 2);
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart0_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            ESP_LOGI(SPP_TAG, "uart[%d] event:", uart_num);
            switch(event.type) {
                case UART_DATA:
                    uart_get_buffered_data_len(uart_num, &buffered_size);
                    ESP_LOGI(SPP_TAG, "data, len: %d; buffered len: %d", event.size, buffered_size);
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(SPP_TAG, "hw fifo overflow\n");
                    uart_flush(uart_num);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(SPP_TAG, "ring buffer full\n");
                    uart_flush(uart_num);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(SPP_TAG, "uart rx break\n");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(SPP_TAG, "uart parity error\n");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(SPP_TAG, "uart frame error\n");
                    break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    ESP_LOGI(SPP_TAG, "uart pattern detected\n");
                    break;
                //Others
                default:
                    ESP_LOGI(SPP_TAG, "uart event type: %d\n", event.type);
                    break;
            }
        }

    }
    free(uart_data);
    uart_data = NULL;
    vTaskDelete(NULL);
}


void uart_receive_task(void *pvParameters)  // TODO: uart接收
{
    int uart_num = (int) pvParameters;
	uint8_t* uart_data = (uint8_t*) malloc(BUF_SIZE * 2);
	s32 Write_handle = 0;
	for(;;)
	{
#if 1
        //int len = uart_read_bytes(UART_NUM_0,temp,BUF_SIZE,10/ portTICK_RATE_MS);
		int len = uart_read_bytes(uart_num, uart_data, BUF_SIZE, 10/ portTICK_RATE_MS);
		if(len > 0)
		{
			ESP_LOGI(SPP_TAG, "uart read num : %d", len);
            //uart_write_bytes(uart_num, (const char*)uart_data, len);
            if(config_json_received == 0)
            {
                config_json_received = 1;
                if(doit((char*)uart_data) == json_true)
                {
                    printf("receive json_ini\r\n");
                    config_json_received = 2;
                }
            }
            else if(strstr((const char*)uart_data,"name") != NULL)
            {
                memset(device_name,0,40);
                strtrimall(uart_data,len);
                //uart_write_bytes(uart_num, (const char*)uart_data, len);
                sscanf((const char*)uart_data,"%*[^=]=%s",device_name);
                printf("device_name = %s\n",device_name);
                set_bt_name();
            }
#if 0            
            else if(strstr((const char*)uart_data,"devicetype=") != NULL) //从rtk得到设备类型
            {
                int bt_num = get_ble_num();
                for(int i = 0;i < bt_num;i++)
                {
                    //bt_write(i, sizeof(data), data);
                    bt_write(i, len, uart_data);
                }
            }     
#endif       
            else
            {
                int bt_num = get_ble_num();
                for(int i = 0;i < bt_num;i++)
                {
                    //bt_write(i, sizeof(data), data);
                    bt_write(i, len, uart_data);
                }
                //bt_write(Write_handle, len, uart_data);
            }
            //uart_flush(uart_num);
		}
		//uart_write_bytes(uart_num, (const char*)(data), sizeof(data));
#endif
	}
	free(uart_data);
	uart_data = NULL;
	vTaskDelete(NULL);

}


void uart_recevive_task_create()
{
    int uart_num = UART_CHANNEL;
	uart_init(uart_num);
    xTaskCreate(uart_task, "uart_task", 2048 * 5, (void*)uart_num, 5, NULL);
    xTaskCreate(uart_receive_task, "uart_receive_task", 2048 * 5, (void*)uart_num, 10, NULL);
    //process data
}

void uart_write_data(uart_port_t uart_num, const char* src, size_t size)
{
	uart_write_bytes(uart_num, src,size);
}

char* get_device_name()
{
    return device_name;
}
