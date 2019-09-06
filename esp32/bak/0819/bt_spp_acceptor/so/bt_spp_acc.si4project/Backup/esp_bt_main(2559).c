/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#include "time.h"
#include "sys/time.h"

#include "driver/uart.h"
//#include "rtk_stdint.h"
#include "rtk_bt.h"
#include "tool.h"
#include "main.h"

typedef unsigned int u32;

/*
define
*/

#define ESP32_KIT


#define SPP_TAG "SPP_ACCEPTOR_DEMO"
#define SPP_SERVER_NAME "SPP_SERVER"

#ifndef size_t
#define size_t unsigned int
#endif
#define BUF_SIZE (1024 * 1)
#ifdef ESP32_KIT
#define EXCAMPLE_DEVICE_NAME "ESP32_BT_KIT"
#define UART_CHANNEL UART_NUM_1                    //16 tx   17 rx
#define ECHO_TEST_TXD  (16)
#define ECHO_TEST_RXD  (17)
//#define ECHO_TEST_TXD  (16)
//#define ECHO_TEST_RXD  (17)

#else
#define EXCAMPLE_DEVICE_NAME "ESP32_BT"
#define UART_CHANNEL UART_NUM_0
#endif
#define SPP_SHOW_DATA 0
#define SPP_SHOW_SPEED 1
//#define SPP_SHOW_MODE SPP_SHOW_SPEED    /*Choose show mode: show data or speed*/
#define SPP_SHOW_MODE SPP_SHOW_DATA    /*Choose show mode: show data or speed*/

static int connect_flag = 0;
static u32 write_handle;
static SemaphoreHandle_t w_task_xSemaphore;

static SemaphoreHandle_t bt_write_xSemaphore;        //写蓝牙锁
 
//static esp_spp_cb_param_t bt_spp_s;
esp_spp_cb_param_t bt_spp_s;

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
static struct timeval time_new, time_old;
static long data_num = 0;
static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_NONE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;



esp_spp_cb_param_t* bt_write_handle = NULL;
static const char remote_device_name[] = "DCH";
static char peer_bdname[ESP_BT_GAP_MAX_BDNAME_LEN + 1];
static uint8_t peer_bdname_len;
unsigned char data[] = {"$GPGGA,000000.00,3130.4988413,N,12024.0834646,E,1,00,1.0,40.647,M,0.000,M,0.0,*46"};


QueueHandle_t uart0_queue;

static void BT_write_task_crate();
static void bt_wrtie_task(void *pvParameters);
static void uart_init(int channel);
static void uart_task(void *pvParameters);
static void uart_recevive_task_create();
static void uart_receive_task(void *pvParameters);
void mutex_init();
void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);




void app_main()
{

	//esp_log_level_set(SPP_TAG, ESP_LOG_INFO);            //打印等级 ESP_LOG_NONE
	mutex_init();
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
	uart_recevive_task_create();
	Dbgprintf("aceinna test!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
	BT_write_task_crate();
    /*
     * Set default parameters for Legacy Pairing
     * Use variable pin, input pin code when pairing
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);
}


static void print_speed(void)
{
    float time_old_s = time_old.tv_sec + time_old.tv_usec / 1000000.0;
    float time_new_s = time_new.tv_sec + time_new.tv_usec / 1000000.0;
    float time_interval = time_new_s - time_old_s;
    float speed = data_num * 8 / time_interval / 1000.0;
    ESP_LOGI(SPP_TAG, "speed(%fs ~ %fs): %f kbit/s" , time_old_s, time_new_s, speed);
    data_num = 0;
    time_old.tv_sec = time_new.tv_sec;
    time_old.tv_usec = time_new.tv_usec;
}
//unsigned char data[7] = {0x64,0x61,0x69,0x63,0x68,0x06,0x07};

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event) {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
        esp_bt_dev_set_device_name(EXCAMPLE_DEVICE_NAME);                   //设置蓝牙名称
        esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);//设置可发现性及可连接性模式
        esp_spp_start_srv(sec_mask,role_slave, 0, SPP_SERVER_NAME);         //开始侦听蓝牙设备连接请求 
        //服务启动后回调 ESP_SPP_START_EVT，连接后 ESP_SPP_SRV_OPEN_EVT回调
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:                                  
        ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
        break;
    case ESP_SPP_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
        break;
    case ESP_SPP_CLOSE_EVT:                                                 //断开连接
        ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT");                             //如何判断哪个蓝牙退出连接
        
		set_bt_state(param,BT_DISCONNECT,ESP_SPP_CLOSE_EVT);
		esp_spp_start_srv(sec_mask,role_slave, 0, SPP_SERVER_NAME); 		//重新开始侦听蓝牙设备连接请求 
        break;
    case ESP_SPP_START_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT");
        break;
    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
        break;
    case ESP_SPP_DATA_IND_EVT:                                             //读事件
#if (SPP_SHOW_MODE == SPP_SHOW_DATA)
        ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT len=%d handle=%d",
                 param->data_ind.len, param->data_ind.handle);
        esp_log_buffer_hex("",param->data_ind.data,param->data_ind.len);
		
        Dbgprintf("reveive cloud bt data \n");
		unsigned char data_start[] = {"start:\r\n"};
		unsigned char data_end[] = {"end:\r\n"};
		uart_write_bytes(UART_CHANNEL, (const char*)(param->data_ind.data),param->data_ind.len);
		//bt_write(param->srv_open.handle, sizeof(data_start),data_start);
        //bt_write(param->srv_open.handle, param->data_ind.len, param->data_ind.data);
		//bt_write(param->srv_open.handle, sizeof(data_end),data_end);
		//Dbgprintf("param->srv_open.handle = %d\n",param->srv_open.handle);

#else
        gettimeofday(&time_new, NULL);
        data_num += param->data_ind.len;
        if (time_new.tv_sec - time_old.tv_sec >= 3) {
            print_speed();
        }
#endif
        break;
    case ESP_SPP_CONG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT");

        break;
    case ESP_SPP_WRITE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT");         //接收到写入事件
        break;
    case ESP_SPP_SRV_OPEN_EVT:                          //连接成功时回调
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT");
		//set_ble_write_handle(param);                  //连接成功，开始写入
		//connect_flag = 1;
	    add_bt(BT_CONNECT,param);
		set_bt_state(param,BT_CONNECT,ESP_SPP_SRV_OPEN_EVT);

		xSemaphoreGive(w_task_xSemaphore);
        gettimeofday(&time_old, NULL);
        break;
    default:
        break;
    }
/*
	printf("param->srv_open.status = %d\r\n  "
	"param->srv_open.handle = %d\r\n "         
	"param->srv_open.new_listen_handle = %d   "
	"param->srv_open.fd = %d\n",param->srv_open.status, "
	"param->srv_open.handle,param->srv_open.new_listen_handle"
	"param->srv_open.fd);
*/

	//printf("param->srv_open.handle = %d\r\n",param->srv_open.handle);
	//printf("param->srv_open.new_listen_handle = %d\r\n",param->srv_open.new_listen_handle);

	set_ble_new_write_handle(&bt_spp_s,param);

	//memcpy(&bt_spp_s,param,sizeof(esp_spp_cb_param_t));
}

#if 0
if(ESP_OK == esp_spp_connect(sec_mask, role_master, param->disc_comp.scn[0], peer_bd_addr)) 	//连接远程设备	 
{
	Dbgprintf("connect suc 111111111111\n");
}
#endif

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:{
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(SPP_TAG, "authentication success: %s", param->auth_cmpl.device_name);   //验证成功
            esp_log_buffer_hex(SPP_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        } else {
            ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT:{
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
            ESP_LOGI(SPP_TAG, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
		    pin_code[0] = '0';
            pin_code[1] = '0';
            pin_code[2] = '0';
            pin_code[3] = '0';
#if 0
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
#endif
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }
	case ESP_BT_GAP_READ_RSSI_DELTA_EVT:
		Dbgprintf("read rssi event \n");
		break;
	case ESP_BT_GAP_DISC_RES_EVT:
		Dbgprintf("device discovery result event \n");
		break;
	case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
		Dbgprintf("discovery state changed event  \n");
		break;
	case ESP_BT_GAP_RMT_SRVCS_EVT:
		Dbgprintf("get remote services event  \n");
		break;
	case ESP_BT_GAP_RMT_SRVC_REC_EVT:
		Dbgprintf("get remote service record event \n");
		break;
#if 0
	case ESP_BT_GAP_CFM_REQ_EVT:
		Dbgprintf("Simple Pairing User Confirmation request \n");
		break;	

	case ESP_BT_GAP_KEY_NOTIF_EVT:
		Dbgprintf("Simple Pairing Passkey Notification  \n");
		break;	

	case ESP_BT_GAP_KEY_REQ_EVT:
		Dbgprintf("Simple Pairing Passkey request \n");
		break;	
#endif

    default: {
        ESP_LOGI(SPP_TAG, "event: %d", event);
        break;
    }
    }
	
	set_ble_new_write_handle(&bt_spp_s,param);
    return;
}






static void bt_wrtie_task(void *pvParameters)
{ 	
	//while(!connect_flag);
	xSemaphoreTake(w_task_xSemaphore,portMAX_DELAY);
	//vTaskDelay(5000 / portTICK_PERIOD_MS);
	//esp_spp_cb_param_t* bt_write_handle;
	s32 w_task_handle;
	//get_ble_write_handle(&w_task_handle);
	for( ;; )	
	{		
		//esp_spp_write((bt_write_handle)->srv_open.handle, 7, data);
		get_ble_new_write_handle(&w_task_handle);
		//bt_write(w_task_handle, sizeof(data), data);
		for(int i=0;i<sizeof(data);i++)
		{
			//data[i]++;
		}
		vTaskDelay(5000 / portTICK_PERIOD_MS);
		Dbgprintf("sleep 5s\n");
	}
}

static void BT_write_task_crate()
{	
	xTaskHandle write_handle;
	w_task_xSemaphore = xSemaphoreCreateBinary();	 
	if( w_task_xSemaphore == NULL )   
	{	
		Dbgprintf("crate fail\n");   
		return;
	}
    else
    {
    	Dbgprintf("crate w_task_xSemaphore suc\n");   
   	}
	//xTaskCreate(bt_wrtie_task, "bt write task", 2048, (void *)pvParameters, 10, &write_handle);
	xTaskCreate(bt_wrtie_task, "bt write task", 2048*2, NULL, tskIDLE_PRIORITY, NULL);
	Dbgprintf("test4\n");
}



#if 0
static void set_ble_write_handle(esp_spp_cb_param_t* ble_write_handle)
{
	write_handle = ble_write_handle->srv_open.handle;
	Dbgprintf("write_handle = %d\n",write_handle);
}


static void get_ble_write_handle(u32* w_handle)
{
	*w_handle = write_handle;
}
#endif






#if 1
static void uart_init(int channel)
{
    //int uart_num = channel;
    uart_config_t uart_config = {
       .baud_rate = 460800,
       //.baud_rate = 115200,
       .data_bits = UART_DATA_8_BITS,
       .parity = UART_PARITY_DISABLE,
       .stop_bits = UART_STOP_BITS_1,
       .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
       .rx_flow_ctrl_thresh = 122,
    };
    //Set UART parameters
    uart_param_config(channel, &uart_config);
    //uart_set_pin(channel, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, 18, 19);  
    uart_set_pin(channel, ECHO_TEST_TXD, ECHO_TEST_RXD, 18, 19);
    uart_driver_install(channel, BUF_SIZE * 2, BUF_SIZE * 2, 10, &uart0_queue, 0);
    //xTaskCreate(uart_task, "uart_task", 2048 * 5, (void*)channel, 12, NULL);

}



static void uart_task(void *pvParameters)
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
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.
                in this example, we don't process data in event, but read data outside.*/
                case UART_DATA:
                    uart_get_buffered_data_len(uart_num, &buffered_size);
                    ESP_LOGI(SPP_TAG, "data, len: %d; buffered len: %d", event.size, buffered_size);
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(SPP_TAG, "hw fifo overflow\n");
                    //If fifo overflow happened, you should consider adding flow control for your application.
                    //We can read data out out the buffer, or directly flush the rx buffer.
                    uart_flush(uart_num);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(SPP_TAG, "ring buffer full\n");
                    //If buffer full happened, you should consider encreasing your buffer size
                    //We can read data out out the buffer, or directly flush the rx buffer.
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

		u32 Write_handle;
    }
    free(uart_data);
    uart_data = NULL;
    vTaskDelete(NULL);
}


static void uart_receive_task(void *pvParameters)
{
    int uart_num = (int) pvParameters;
	uint8_t* uart_data = (uint8_t*) malloc(BUF_SIZE * 2);
	s32 Write_handle = 0;
	for(;;)
	{
#if 1
		int len = uart_read_bytes(uart_num, uart_data, BUF_SIZE, 100/ portTICK_RATE_MS);
		if(len > 0)
		{
		
			ESP_LOGI(SPP_TAG, "uart read num : %d", len);
			//uart_write_bytes(uart_num, (const char*)uart_data, len);
			//get_ble_write_handle(&Write_handle);
			//get_ble_new_write_handle(&Write_handle);

			bt_write(Write_handle, len, uart_data);
			//uart_write_bytes(uart_num, (const char*)uart_data, len);
		}
		uart_write_bytes(uart_num, (const char*)(data), sizeof(data));
		sleep_s(2);

#endif
	}
	free(uart_data);
	uart_data = NULL;
	vTaskDelete(NULL);

}


static void uart_recevive_task_create()
{
    int uart_num = UART_CHANNEL;
	uart_init(uart_num);
    xTaskCreate(uart_receive_task, "uart_receive_task", 2048 * 5, (void*)uart_num, 12, NULL);
    //process data
}

esp_spp_cb_param_t* get_esp_spp_cb_param_t()
{
	return &bt_spp_s;
}


void mutex_init()
{
	bt_write_xSemaphore = xSemaphoreCreateBinary();	 
	if( bt_write_xSemaphore == NULL )   
	{	
		Dbgprintf("crate fail\n");   
		return;
	}
    else
    {
    	Dbgprintf("crate bt_write_xSemaphore suc\n");   
		xSemaphoreGive(bt_write_xSemaphore);
   	}
}

void take_bt_mutex(void)
{
	xSemaphoreTake(bt_write_xSemaphore,portMAX_DELAY);
}

void release_bt_mutex(void)
{
	xSemaphoreGive(bt_write_xSemaphore);
}


#endif
