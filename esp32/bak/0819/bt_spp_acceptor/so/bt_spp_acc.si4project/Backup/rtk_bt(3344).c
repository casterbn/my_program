/*******************************************************************************
* File Name          : rtk_bt.c
* Author             : Daich
* Revision           : 1.0
* Date               : 13/05/2019
* Description        : bluetooth
*
* HISTORY***********************************************************************
* 13/05/2019  |                                             | Daich
*
*******************************************************************************/
#include "rtk_stdint.h"
#include "rtk_bt.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

//extern esp_spp_cb_param_t bt_spp_s;

extern esp_spp_cb_param_t* get_esp_spp_cb_param_t();
bt_manage_s esp_bt_manage_s;



void bt_write(u32 handle,int len, unsigned char *p_data)
{
    if(handle > 0)
    {
    	esp_spp_cb_param_t* bt_spp_s = get_esp_spp_cb_param_t();
		printf("bt_spp_s->srv_open.handle = %d\n",bt_spp_s->srv_open.handle);
		if(bt_spp_s->srv_open.status == ESP_SPP_SUCCESS)
		{
	        printf("test20\n");
    		esp_spp_write(bt_spp_s->srv_open.handle, len, p_data);
		}
    }
}

int init_mul_bt(void)
{
	return 1;
}
int destroy_a_bt(void)
{
	return 1;
}

bt_handle_t  add_bt(int state,char addr[6])
{
	return 1;
}

BT_STAT_E is_this_bt_run(bt_handle_t bt_handle)
{
	return 1;
}


