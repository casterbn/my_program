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
#include "RingBuffer.h"

typedef SemaphoreHandle_t osSemaphoreId;
QueueHandle_t uart0_queue;
osSemaphoreId dataAcqSem;   //信号量


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

       //.baud_rate = 460800,
       .baud_rate = 921600,
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

extern RingBuffer  UartRxRing;
extern u8          UartRxBuff[RX_SIZE];
void uart_receive_task(void *pvParameters)  // TODO: uart接收
{
    int uart_num = (int) pvParameters;
	//uint8_t* uart_data = (uint8_t*) malloc(BUF_SIZE * 2);
    uint8_t* uart_data = (uint8_t*) malloc(BUF_SIZE);
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
                if(add_esp32_json((char*)uart_data) == json_true)
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
                //printf("len = %d\n",len);
#ifdef RTCM_FILTER                
                WriteRingBuffer(&UartRxRing,uart_data,len); //写入buffer
                xSemaphoreGive(dataAcqSem);
#else
                //count = uxSemaphoreGetCount(semaphore);
                //printf("write_index = %d\n",UartRxRing.write_index);
                //printf("read_index = %d\n",UartRxRing.read_index);
                int bt_num = get_ble_num();
                for(int i = 0;i < bt_num;i++)
                {
                    bt_write(i, len, uart_data);
                }
#endif
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


//int   TotalRxCnt=0;
u8    longBuff[4096]={0};
int   longIndex = 0;
rtcm_t rtcm_to_save_data ;

void uart_parse_task(void *pvParameters)  // TODO: uart接收
{
    int uart_num = (int) pvParameters;
	uint8_t* uart_data = (uint8_t*) malloc(BUF_SIZE * 2);
	s32 Write_handle = 0;
    printf("uart_parse_task start!!!!!!!!!!!!!!!!!!!!\n");
	for(;;)
	{
        int pos = 0;
        int bt_num = 0;
        int i = 0;
        xSemaphoreTake(dataAcqSem, portMAX_DELAY);
        int TotalRxCnt =  ReadAllDataNoDeel(&UartRxRing,&longBuff[longIndex]);  //解析uart数据
        if(TotalRxCnt == 0)
        {
            //vTaskDelay(10 / portTICK_PERIOD_MS);
            //vTaskDelay(6 / portTICK_PERIOD_MS);
        }
        while (TotalRxCnt)
        {
            TotalRxCnt--;
            
            //int ret_val = input_rtcm3(RtcmBuff[pos++], stnID, rtcm);
            int ret_val = input_rtcm3_data(&rtcm_to_save_data, longBuff[pos++]);
            if (ret_val == 1)    //完整数据
            {
                //printf("rtcm_to_save_data.len_to_send = %d,***************\n",rtcm_to_save_data.len_to_send);
                //for(int len = rtcm_to_save_data.len_to_send/2;len < rtcm_to_save_data.len_to_send;len++)
                {
                    //printf("%02x ",rtcm_to_save_data.buff[len]);
                }
                switch (rtcm_to_save_data.type)
                {
                    case 1001:
                    case 1002:
                    case 1003:
                    case 1004:
                    case 1005:
                    case 1006:
                    case 1007:
                    case 1008:
                    case 1009:
                    case 1010:
                    case 1011:
                    case 1012:
                    case 1013:
                    case 1019:
                    case 1020:
                    case 1021:
                    case 1022:
                    case 1023:
                    case 1024:
                    case 1025:
                    case 1026:
                    case 1027:
                    case 1029:
                    case 1030:
                    case 1031:
                    case 1032:
                    case 1033:
                    case 1034:
                    case 1035:
                    case 1037:
                    case 1038:
                    case 1039:
                    case 1042:
                    case 1044:
                    case 1045:
                    case 1046:
                    case 63:
                    case 4001:
                    case 1071:
                    case 1072:
                    case 1073:
                    case 1074:      //时间
                    case 1075:
                    case 1076:
                    case 1077:
                    case 1081:
                    case 1082:
                    case 1083:
                    case 1084:
                    case 1085:
                    case 1086:
                    case 1091:
                    case 1092:
                    case 1093:
                    case 1094:
                    case 1095:
                    case 1096:
                    case 1097:
                    case 1101:
                    case 1102:
                    case 1103:
                    case 1104:
                    case 1105:
                    case 1106:
                    case 1107:
                    case 1111:
                    case 1112:
                    case 1113:
                    case 1114:
                    case 1115:
                    case 1116:
                    case 1117:
                    case 1121:
                    case 1122:
                    case 1123:
                    case 1124:
                    case 1125:
                    case 1126:
                    case 1127:
                    case 1230:
                    case 1087:
                        bt_num = get_ble_num();
                        for(i = 0;i < bt_num;i++)
                        {
                            bt_write(i, rtcm_to_save_data.len_to_send, rtcm_to_save_data.buff);
                        }
                        break;
                    default:
                        break;
                }
            }
        }
    }
    free(uart_data);
    uart_data = NULL;
    vTaskDelete(NULL);
}

void uart_recevive_task_create()
{
    //dataAcqSem = osSemaphoreCreate(osSemaphore(DATA_ACQ_SEM), 1);
    dataAcqSem = xSemaphoreCreateCounting(10, 0);
    int uart_num = UART_CHANNEL;
	uart_init(uart_num);
    xTaskCreate(uart_task, "uart_task", 2048 * 5, (void*)uart_num, tskIDLE_PRIORITY + 10, NULL);
    xTaskCreate(uart_receive_task, "uart_receive_task", 2048 * 5, (void*)uart_num, tskIDLE_PRIORITY + 10, NULL);
#ifdef RTCM_FILTER    
    //xTaskCreate(uart_parse_task, "uart_parse_task", 2048 * 5, (void*)uart_num, tskIDLE_PRIORITY + 4, NULL); 
    xTaskCreate(uart_parse_task, "uart_parse_task", 2048 * 5, (void*)uart_num, tskIDLE_PRIORITY + 11, NULL);      
    //process data
#endif
}

void uart_write_data(uart_port_t uart_num, const char* src, size_t size)
{
	uart_write_bytes(uart_num, src,size);
}

char* get_device_name()
{
    return device_name;
}
