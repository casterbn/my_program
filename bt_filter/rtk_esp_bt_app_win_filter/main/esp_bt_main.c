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

//#include "driver/uart.h"
#include "rtk_bt.h"
#include "tool.h"
#include "main.h"
#include "rtk_uart.h"
#include "esp_system.h"
#include "esp_wifi_types.h"
#include "json_parse.h"
#include "crc.h"
#include "RingBuffer.h"
/*

define

*/


RingBuffer  UartRxRing;
u8          UartRxBuff[RX_SIZE];
char esp32_serial[20];



#ifndef size_t
#define size_t unsigned int
#endif

#define BT_MAC_LEN      6
#define SPP_SHOW_DATA   0
#define SPP_SHOW_SPEED  1
//#define SPP_SHOW_MODE SPP_SHOW_SPEED    /*Choose show mode: show data or speed*/
#define SPP_SHOW_MODE SPP_SHOW_DATA    /*Choose show mode: show data or speed*/

static SemaphoreHandle_t w_task_xSemaphore;
static SemaphoreHandle_t bt_write_xSemaphore;        //写蓝牙锁
 

extern bt_uart_baud_e BAUD_RATE; 
char* baud_rate_str[3] = {"115200","460800","921600"};

esp_spp_cb_param_t bt_spp_s;

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
static struct timeval time_new, time_old;
static long data_num = 0;
static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_NONE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;
static int sleep_count = 20;
extern char device_name[40];
esp_spp_cb_param_t* bt_write_handle = NULL;
static const char remote_device_name[] = "DCH";
//static char peer_bdname[ESP_BT_GAP_MAX_BDNAME_LEN + 1];
//static uint8_t peer_bdname_len;
unsigned char data[] = {"$GPGGA,000000.00,3130.4988413,N,12024.0834646,E,1,00,1.0,40.647,M,0.000,M,0.0,*46\r\n"};
unsigned char bt_mac[BT_MAC_LEN];
char* bt_device_name;
const char* VERSION = "aceinna_bt_v_1.2";
extern char config_json_from_rtk[500];
static void BT_write_task_crate();
static void bt_wrtie_task(void *pvParameters);
extern int config_json_received;

void mutex_init();
void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param);
void nvs_read_data_from_flash(char* str);
void nvs_write_data_to_flash(char*str);



#if 1
void app_main()
{    

#ifdef RTCM_FILTER
    InitRingBuffer(&UartRxRing,UartRxBuff,RX_SIZE);
#endif
    printf("version = %s\n",VERSION);
    printf("bt uart baud rate = %s\n",baud_rate_str[BAUD_RATE]);

    CRC_Table_Init();
	//esp_log_level_set(SPP_TAG, ESP_LOG_INFO);          //打印等级 ESP_LOG_NONE
    esp_log_level_set(SPP_TAG, ESP_LOG_NONE);            //打印等级 ESP_LOG_NONE
	mutex_init();



    esp_err_t ret = nvs_flash_init();                    //读写flash
    int nvs_init_max_num = 0;
    while (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        //ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
        vTaskDelay(100 / portTICK_PERIOD_MS);
        printf("re init****************************\n");
        nvs_init_max_num++;
        if(nvs_init_max_num > 4)
        {
            break;
        }
    }
    ESP_ERROR_CHECK( ret );
    esp_read_mac(bt_mac,ESP_MAC_BT);
    Hex2Str((char*)bt_mac,esp32_serial,BT_MAC_LEN);
    printf("BT_MAC: %s\n",esp32_serial);    
#if 0    
    printf("bt_mac = ");
    for(int i = 0;i < 6;i++)
    {
        printf("%x ",bt_mac[i]);
    }
#endif
    char device_name_read[40];
    nvs_read_data_from_flash(device_name_read);
    if(strlen(device_name_read) == 0)
    {
        bt_device_name = DEVICE_NAME;
    }
    else
    {
        bt_device_name = device_name_read;
    }
    printf("bt_device_name = %s\n",bt_device_name);
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
	DbgPrintf("aceinna test!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
	BT_write_task_crate();
    /*
     * Set default parameters for Legacy Pairing
     * Use variable pin, input pin code when pairing
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);

}
#endif


void uart_echo_test();
void uart_evt_test1();

#if 0
void app_main()
{
    //A uart read/write example without event queue;
    //xTaskCreate(uart_echo_test, "uart_echo_test", 1024, NULL, 10, NULL);
    uart_recevive_task_create();
    //A uart example with event queue.
    //uart_evt_test1();
}
#endif

#if (SPP_SHOW_MODE == SPP_SHOW_SPEED)
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
#endif

const char* bt_to_app = "receive data\r\n" ;
char int_to_s[2];
int bt_num;
char write_to_bt[512];
static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event) {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
        esp_bt_dev_set_device_name(bt_device_name);                   //设置蓝牙名称
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
		//esp_spp_start_srv(sec_mask,role_slave, 0, SPP_SERVER_NAME); 		//重新开始侦听蓝牙设备连接请求 
        break;
    case ESP_SPP_START_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT");
        break;
    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
        break;
    case ESP_SPP_DATA_IND_EVT:                                             // TODO: 读事件
#if (SPP_SHOW_MODE == SPP_SHOW_DATA)
        ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT len=%d handle=%d",
                 param->data_ind.len, param->data_ind.handle);
        DbgPrintf("reveive cloud bt data \n");
        //bt_write(0, strlen(bt_to_app), bt_to_app);
        if(strstr((const char*)param->data_ind.data,"name") != NULL)
        {
            //printf("data_ind = %s\n",param->data_ind.data);
            memset(device_name,0,40);
            char* bt_data = (char*)malloc(sizeof(char) * param->data_ind.len);
            memcpy(bt_data,param->data_ind.data,param->data_ind.len);
            //printf("bt_data1 = %s\n",bt_data);
            strtrimall((uint8_t*)bt_data,param->data_ind.len);
            //printf("bt_data2 = %s\n",bt_data);
            sscanf((const char*)bt_data,"%*[^=]=%s",device_name);
            printf("device_name = %s\n",device_name);
            set_bt_name();
        }
        else if(strstr((const char*)param->data_ind.data,"params=devicetype") != NULL) //从蓝牙处获取查询状态
        {
            uart_write_data(UART_CHANNEL, "params=devicetype",strlen("params=devicetype"));   
        }
        else
        {
            uart_write_data(UART_CHANNEL, (const char*)(param->data_ind.data),param->data_ind.len);
        }

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
    case ESP_SPP_SRV_OPEN_EVT:                          //TODO: 连接成功时回调
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT");
		//set_ble_write_handle(param);                  //连接成功，开始写入
        printf("connect suc\n");
	    add_bt(BT_CONNECT,param);
		set_bt_state(param,BT_CONNECT,ESP_SPP_SRV_OPEN_EVT);
#if 1
        bt_num = get_ble_num();        //添加json
        itoa(bt_num,int_to_s,2);  
        if(config_json_received == 2)
        {
            change_item(config_json_from_rtk,write_to_bt,"connect_device_num",int_to_s);       
            printf("ini = %s\n",write_to_bt);        
        }
        else
        {
            esp32_json_only(write_to_bt);
        }
        
        for(int i = 0;i < bt_num;i++)
        {
            bt_write(i, strlen(write_to_bt), (unsigned char *)write_to_bt);
        }
#endif
		xSemaphoreGive(w_task_xSemaphore);
        gettimeofday(&time_old, NULL);
        break;
    default:
        break;
    }
	set_ble_new_write_handle(&bt_spp_s,param);

}


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
		DbgPrintf("read rssi event \n");
		break;
	case ESP_BT_GAP_DISC_RES_EVT:
		DbgPrintf("device discovery result event \n");
		break;
	case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
		DbgPrintf("discovery state changed event  \n");
		break;
	case ESP_BT_GAP_RMT_SRVCS_EVT:
		DbgPrintf("get remote services event  \n");
		break;
	case ESP_BT_GAP_RMT_SRVC_REC_EVT:
		DbgPrintf("get remote service record event \n");
		break;
#if 0
	case ESP_BT_GAP_CFM_REQ_EVT:
		DbgPrintf("Simple Pairing User Confirmation request \n");
		break;	

	case ESP_BT_GAP_KEY_NOTIF_EVT:
		DbgPrintf("Simple Pairing Passkey Notification  \n");
		break;	

	case ESP_BT_GAP_KEY_REQ_EVT:
		DbgPrintf("Simple Pairing Passkey request \n");
		break;	
#endif

    default: {
        ESP_LOGI(SPP_TAG, "event: %d", event);
        break;
    }
    }
	
	//set_ble_new_write_handle(&bt_spp_s,param);
    return;
}



static void bt_wrtie_task(void *pvParameters)
{ 	
	xSemaphoreTake(w_task_xSemaphore,portMAX_DELAY);
	//vTaskDelay(5000 / portTICK_PERIOD_MS);
	s32 w_task_handle;
	for( ;; )	
	{		
		//esp_spp_write((bt_write_handle)->srv_open.handle, 7, data);
		get_ble_new_write_handle(&w_task_handle);
		//bt_write(0, sizeof(data), data);
		for(int i=0;i<sizeof(data);i++)
		{
			//data[i]++;
		}
		vTaskDelay(sleep_count / portTICK_PERIOD_MS);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        int bt_num = get_ble_num();
        for(int i = 0;i < bt_num;i++)
        {
            //bt_write(i, sizeof(data), data);
        }
		//DbgPrintf("sleep 5s\n");
        //printf("sleep 2s\n");
	}
}

static void BT_write_task_crate()
{	
	//xTaskHandle write_handle;
	w_task_xSemaphore = xSemaphoreCreateBinary();	 
	if( w_task_xSemaphore == NULL )   
	{	
		DbgPrintf("crate fail\n");   
		return;
	}
    else
    {
    	DbgPrintf("crate w_task_xSemaphore suc\n");   
   	}
	//xTaskCreate(bt_wrtie_task, "bt write task", 2048*2, NULL, tskIDLE_PRIORITY + 2, NULL);    //10
	DbgPrintf("test4\n");
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
		DbgPrintf("crate fail\n");   
		return;
	}
    else
    {
    	DbgPrintf("crate bt_write_xSemaphore suc\n");   
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

void nvs_write_data_to_flash(char*str)
{
    static const char *BT_DEVICE_NAME_CUSTOMER = "bt customer";
    nvs_handle handle;
    printf("str: %s\r\n", str);    
    ESP_ERROR_CHECK( nvs_open( BT_DEVICE_NAME_CUSTOMER, NVS_READWRITE, &handle) );
    ESP_ERROR_CHECK( nvs_set_str( handle, "bt name", (const char*)str) );
    ESP_ERROR_CHECK( nvs_commit(handle) );
    nvs_close(handle);
    printf("write com\r\n");    
}

void nvs_read_data_from_flash(char* str)
{
    static const char *BT_DEVICE_NAME_CUSTOMER = "bt customer";
    printf("start read \r\n");    
    nvs_handle handle;
    uint32_t str_length = 40;
    ESP_ERROR_CHECK( nvs_open(BT_DEVICE_NAME_CUSTOMER, NVS_READWRITE, &handle) );
    //ESP_ERROR_CHECK ( nvs_get_str(handle, "bt name", (const char *)str, &str_length) );
    nvs_get_str(handle, "bt name", (const char *)str, &str_length);
    printf("str: %s\r\n", str);
    nvs_close(handle);
}

void set_bt_name()
{
	char* new_device_name = get_device_name();

	esp_bt_dev_set_device_name(new_device_name);                         //设置蓝牙名称
    esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE); //设置可发现性及可连接性模式
	esp_spp_start_srv(sec_mask,role_slave, 0, SPP_SERVER_NAME);          //开始侦听蓝牙设备连接请求 
    nvs_write_data_to_flash(new_device_name);
    printf("rename suc! please reconnect bt!!!\n");
}
