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
#ifndef _RTK_BT_H
#define _RTK_BT_H
#include "rtk_stdint.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

typedef int bt_handle_t;
#define MAX_BT_COUNT 7


typedef enum {
	BT_NONE = 0,
    BT_IDLE,
    BT_CONNECT,
    BT_DISCONNECT
}BT_STAT_E;

typedef struct _bt_manage
{
    struct _bt_info
    {
        BT_STAT_E state;      //蓝牙状态
        char sev_addr[6];     //蓝牙地址
        char bt_handle;       //true handle
    }bt_info[MAX_BT_COUNT];
    s32 write_pointer;        //读写指针
	s32 read_pointer;
	char addr_len;
	s32 bt_num;
}bt_manage_s;



void bt_write(u32 handle,int len, unsigned char *p_data);
int init_mul_bt(void);
int destroy_a_bt(void);
bt_handle_t add_bt(int state,esp_spp_cb_param_t* bt_info);
void set_bt_state(bt_handle_t bt_handle,BT_STAT_E bt_state);

BT_STAT_E get_this_bt_state(bt_handle_t bt_handle);

/*
typedef struct esp_bt_s_
{
	esp_spp_cb_param_t bt_spp_s;
	
}esp_bt_s;
*/
#endif
