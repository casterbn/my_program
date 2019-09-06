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

#include <string.h>

//extern esp_spp_cb_param_t bt_spp_s;

extern esp_spp_cb_param_t* get_esp_spp_cb_param_t();
bt_manage_s esp_bt_manage_s;



void bt_write(u32 handle,int len, unsigned char *p_data)
{
	printf("handle = %d\n",handle);
    if(get_this_bt_state(handle) == BT_CONNECT)
    {
    	esp_spp_cb_param_t* bt_spp_s = get_esp_spp_cb_param_t();
		printf("bt_spp_s->srv_open.handle = %d\n",bt_spp_s->srv_open.handle);
		if(bt_spp_s->srv_open.status == ESP_SPP_SUCCESS)
		{
	        printf("test20\n");
    		esp_spp_write(bt_spp_s->srv_open.handle, len, p_data);
		}
    }
    else
    {
    	printf("bt disconnect\n");
    }
}

int init_mul_bt(void)
{
	memset(&esp_bt_manage_s,0,sizeof(esp_bt_manage_s));
	esp_bt_manage_s.addr_len = ESP_BD_ADDR_LEN;
	return ESP_ERR_NONE;
}
int destroy_a_bt(void)
{
	return 1;
}

bt_handle_t add_bt(int state,esp_spp_cb_param_t* bt_info)
{
	int i;
	printf("add bt\n");

	for(i=0;i<MAX_BT_COUNT;i++)
	{
		for(int j=0;j<ESP_BD_ADDR_LEN;j++)
		{
			printf("bt_info->srv_open.rem_bda[%d] = %x\n",j,bt_info->srv_open.rem_bda[j]);
		}
		for(int j=0;j<ESP_BD_ADDR_LEN;j++)
		{
			printf("esp_bt_manage_s.sev_addr[%d] = %x\n",j,esp_bt_manage_s.bt_info[i].sev_addr[j]);
		}
		//for(int j=0;j<MAX_BT_COUNT;j++)
		{
			//if(strcmp(esp_bt_manage_s.bt_info[i].sev_addr,(char*)(bt_info->srv_open.rem_bda)) == 0) //是否可用strcmp比较
			for(int k=0;k<ESP_BD_ADDR_LEN;k++)
			{
				if(bt_info->srv_open.rem_bda[k] != esp_bt_manage_s.bt_info[i].sev_addr[k])
				break;
			}
			
			{
				esp_bt_manage_s.bt_info[i].bt_handle = bt_info->srv_open.handle;
				//memcpy(esp_bt_manage_s.bt_info[i].sev_addr,bt_info->srv_open.rem_bda,sizeof(esp_bt_manage_s.bt_info[i].sev_addr));
				esp_bt_manage_s.bt_info[i].state = BT_CONNECT;
				printf("111esp_bt_manage_s.bt_info[%d].state = %d\n",i,esp_bt_manage_s.bt_info[i].state);
				break;
			}
		}
	}
	if(i == MAX_BT_COUNT)
	{
		for(i=0;i<MAX_BT_COUNT;i++)
		{
			if(esp_bt_manage_s.bt_info[i].state == BT_NONE)
			{
				esp_bt_manage_s.bt_info[i].bt_handle = bt_info->srv_open.handle;
				memcpy(esp_bt_manage_s.bt_info[i].sev_addr,bt_info->srv_open.rem_bda,sizeof(esp_bt_manage_s.bt_info[i].sev_addr));
				esp_bt_manage_s.bt_info[i].state = BT_CONNECT;
				printf("222esp_bt_manage_s.bt_info[%d].state = %d\n",i,esp_bt_manage_s.bt_info[i].state);
				break;
			}
		}
	}
	esp_bt_manage_s.bt_num++;
	printf("esp_bt_manage_s.bt_num = %d\n",esp_bt_manage_s.bt_num);
	return i;             //虚拟handle
}

void set_bt_state(bt_handle_t bt_handle,BT_STAT_E bt_state)
{
	for(int i=0;i<MAX_BT_COUNT;i++)
	{
		if(esp_bt_manage_s.bt_info[i].bt_handle == bt_handle)
		{
			esp_bt_manage_s.bt_info[i].state = bt_state;
		}
	}
}

BT_STAT_E get_this_bt_state(bt_handle_t bt_handle)
{
	int get_handle = -1;
	for(int i=0;i<MAX_BT_COUNT;i++)
	{
		if(esp_bt_manage_s.bt_info[i].bt_handle == bt_handle)
		{
			get_handle = i;
			break;
		}
	}
	if(get_handle >= 0)
	{
		printf("find\n");
		return esp_bt_manage_s.bt_info[get_handle].state;
	}
	else
	{
		return BT_NONE;
	}
}


