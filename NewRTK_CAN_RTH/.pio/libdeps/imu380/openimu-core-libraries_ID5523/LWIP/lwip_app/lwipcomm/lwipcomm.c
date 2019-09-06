#include "lwipcomm.h"
#include "netif/etharp.h"
#include "lwip/dhcp.h"
#include "lwip/mem.h"
#include "lwip/memp.h"
#include "lwip/init.h"
#include "ethernetif.h"
#include "lwip/timers.h"
#include "lwip/tcp_impl.h"
#include "lwip/ip_frag.h"
#include "lwip/tcpip.h"
#include "lwip/timers.h"
#include "malloc.h"
#include "stm32f4xx_hal.h"
#include "osapi.h"
#include "osresources.h"
#include <stdio.h>
#include "KSZ8041NL.h"

__lwip_dev lwipdev;		 // lwip control struct
struct netif lwip_netif;

extern u32_t memp_get_memorysize(void);
extern u8_t *memp_memory;
extern u8_t *ram_heap;

#define LWIP_DHCP_TASK_PRIO 3
#define LWIP_DHCP_STK_SIZE 256
TaskHandle_t LWIP_DHCP_TaskHandler;

void lwip_dhcp_task(void *pdata);

void lwip_pkt_handle(void)
{
	ethernetif_input(&lwip_netif);
}

/** ***************************************************************************
 * @name lwip_comm_mem_malloc() malloc memory 
 * @brief malloc memory for mem and memp of lwip
 * 	if malloc failure, lwip_comm_mem_free() will be called
 * @param [in] N/A
 * @retval 0 or 1 indicating success or failure of malloc memory 
 ******************************************************************************/
uint8_t lwip_comm_mem_malloc(void)
{
	uint32_t mempsize;
	uint32_t ramheapsize;

	mempsize = memp_get_memorysize();		  
	memp_memory = mymalloc(SRAMIN, mempsize);
	ramheapsize = LWIP_MEM_ALIGN_SIZE(MEM_SIZE) + 2 * LWIP_MEM_ALIGN_SIZE(4 * 3) + MEM_ALIGNMENT;
	ram_heap = mymalloc(SRAMIN, ramheapsize); 

	if (!(uint32_t)&memp_memory || !(uint32_t)&ram_heap)
	{
		lwip_comm_mem_free();
		return 1;
	}
	return 0;
}

/** ***************************************************************************
 * @name lwip_comm_mem_free() free memory 
 * @brief free memory of lwip
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
void lwip_comm_mem_free(void)
{
	myfree(SRAMIN, memp_memory);
	myfree(SRAMIN, ram_heap);
}

/** ***************************************************************************
 * @name lwip_comm_default_ip_set() set default ip 
 * @brief set default remoteip, mac, ip, netmask, gateway
 *	dhcpstatus = 0
 *	in the future, these para can be load from flash 
 * @param [in] lwipx lwip control struct pointer
 * @retval N/A
 ******************************************************************************/
void lwip_comm_default_ip_set(__lwip_dev *lwipx)
{
	//uint32_t sn0 = 0x784502;
    uint32_t sn0 = 0x10006434;
	// sn0 = *(uint32_t*)(0x1FF0F420); // STM32 Unique DEVICE ID

	lwipx->remoteip[0] = 192;
	lwipx->remoteip[1] = 168;
	lwipx->remoteip[2] = 1;
	lwipx->remoteip[3] = 100;
	// high three bytes (2.0.0) , low three bytes (STM32 ID)
	lwipx->mac[0] = 2;
	lwipx->mac[1] = 0;
	lwipx->mac[2] = 0;
	lwipx->mac[3] = (sn0 >> 16) & 0XFF;
	lwipx->mac[4] = (sn0 >> 8) & 0XFFF;
	lwipx->mac[5] = sn0 & 0XFF;

	lwipx->ip[0] = 192;
	lwipx->ip[1] = 168;
	lwipx->ip[2] = 1;
	lwipx->ip[3] = 110;

	lwipx->netmask[0] = 255;
	lwipx->netmask[1] = 255;
	lwipx->netmask[2] = 255;
	lwipx->netmask[3] = 0;

	lwipx->gateway[0] = 192;
	lwipx->gateway[1] = 168;
	lwipx->gateway[2] = 1;
	lwipx->gateway[3] = 1;

	lwipx->dhcpstatus = 0; // no DHCP
}

// LWIP INIT
uint8_t lwip_comm_init(void)
{
	uint8_t retry = 0;
	struct netif *Netif_Init_Flag;
	struct ip_addr ipaddr;		 
	struct ip_addr netmask;		
	struct ip_addr gw;			 

	if (ETH_Mem_Malloc())
		return 1;

	if (lwip_comm_mem_malloc())     //分配内存
		return 2;

	lwip_comm_default_ip_set(&lwipdev); //设置ip等

	while (KSZ8041NL_Init())
	{
		retry++;
		if (retry > 5)
		{
			retry = 0;
			return 3;
		} 
	}

	tcpip_init(NULL, NULL);

#if LWIP_DHCP // DHCP
	ipaddr.addr = 0;
	netmask.addr = 0;
	gw.addr = 0;
#else // static ip
	IP4_ADDR(&ipaddr, lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3]);
	IP4_ADDR(&netmask, lwipdev.netmask[0], lwipdev.netmask[1], lwipdev.netmask[2], lwipdev.netmask[3]);
	IP4_ADDR(&gw, lwipdev.gateway[0], lwipdev.gateway[1], lwipdev.gateway[2], lwipdev.gateway[3]);
	printf("mac:%d.%d.%d.%d.%d.%d\r\n", lwipdev.mac[0], lwipdev.mac[1], lwipdev.mac[2], lwipdev.mac[3], lwipdev.mac[4], lwipdev.mac[5]);
	printf("static ip:%d.%d.%d.%d\r\n", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3]);
	printf("netmask:%d.%d.%d.%d\r\n", lwipdev.netmask[0], lwipdev.netmask[1], lwipdev.netmask[2], lwipdev.netmask[3]);
	printf("gateway:%d.%d.%d.%d\r\n", lwipdev.gateway[0], lwipdev.gateway[1], lwipdev.gateway[2], lwipdev.gateway[3]);
#endif
	Netif_Init_Flag = netif_add(&lwip_netif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input);

	if (Netif_Init_Flag == NULL)
		return 4;
	else
	{
		netif_set_default(&lwip_netif);
		netif_set_up(&lwip_netif);
	}
	return 0;
}

#if LWIP_DHCP

void lwip_comm_dhcp_creat(void)
{
	xTaskCreate((TaskFunction_t)lwip_dhcp_task,
				(const char *)"DHCP_TASK",
				(uint16_t)LWIP_DHCP_STK_SIZE,
				(void *)NULL,
				(UBaseType_t)LWIP_DHCP_TASK_PRIO,
				(TaskHandle_t *)&LWIP_DHCP_TaskHandler);
}

void lwip_comm_dhcp_delete(void)
{
	dhcp_stop(&lwip_netif);				
	vTaskDelete(LWIP_DHCP_TaskHandler); 
}

void lwip_dhcp_task(void *pdata)
{
	uint32_t ip = 0, netmask = 0, gw = 0;
	dhcp_start(&lwip_netif); 
	lwipdev.dhcpstatus = 0;  
	printf("searching dhcp server...\r\n");
	while (1)
	{
		printf("dhcp success\r\n");
		ip = lwip_netif.ip_addr.addr;	  
		netmask = lwip_netif.netmask.addr; 
		gw = lwip_netif.gw.addr;	
		if (ip != 0)				
		{
			lwipdev.dhcpstatus = 2; 
			printf("mac:%d.%d.%d.%d.%d.%d\r\n", lwipdev.mac[0], lwipdev.mac[1], lwipdev.mac[2], lwipdev.mac[3], lwipdev.mac[4], lwipdev.mac[5]);

			lwipdev.ip[3] = (uint8_t)(ip >> 24);
			lwipdev.ip[2] = (uint8_t)(ip >> 16);
			lwipdev.ip[1] = (uint8_t)(ip >> 8);
			lwipdev.ip[0] = (uint8_t)(ip);
			printf("IP:%d.%d.%d.%d\r\n", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3]);
			lwipdev.netmask[3] = (uint8_t)(netmask >> 24);
			lwipdev.netmask[2] = (uint8_t)(netmask >> 16);
			lwipdev.netmask[1] = (uint8_t)(netmask >> 8);
			lwipdev.netmask[0] = (uint8_t)(netmask);
			printf("netmask:%d.%d.%d.%d\r\n", lwipdev.netmask[0], lwipdev.netmask[1], lwipdev.netmask[2], lwipdev.netmask[3]);
			lwipdev.gateway[3] = (uint8_t)(gw >> 24);
			lwipdev.gateway[2] = (uint8_t)(gw >> 16);
			lwipdev.gateway[1] = (uint8_t)(gw >> 8);
			lwipdev.gateway[0] = (uint8_t)(gw);
			printf("gateway:%d.%d.%d.%d\r\n", lwipdev.gateway[0], lwipdev.gateway[1], lwipdev.gateway[2], lwipdev.gateway[3]);
			break;
		}
		else if (lwip_netif.dhcp->tries > LWIP_MAX_DHCP_TRIES)
		{
			lwipdev.dhcpstatus = 0xFF; 

			IP4_ADDR(&(lwip_netif.ip_addr), lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3]);
			IP4_ADDR(&(lwip_netif.netmask), lwipdev.netmask[0], lwipdev.netmask[1], lwipdev.netmask[2], lwipdev.netmask[3]);
			IP4_ADDR(&(lwip_netif.gw), lwipdev.gateway[0], lwipdev.gateway[1], lwipdev.gateway[2], lwipdev.gateway[3]);
			printf("DHCP timeout!\r\n");
			printf("mac:%d.%d.%d.%d.%d.%d\r\n", lwipdev.mac[0], lwipdev.mac[1], lwipdev.mac[2], lwipdev.mac[3], lwipdev.mac[4], lwipdev.mac[5]);
			printf("static ip:%d.%d.%d.%d\r\n", lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3]);
			printf("netmask:%d.%d.%d.%d\r\n", lwipdev.netmask[0], lwipdev.netmask[1], lwipdev.netmask[2], lwipdev.netmask[3]);
			printf("gateway:%d.%d.%d.%d\r\n", lwipdev.gateway[0], lwipdev.gateway[1], lwipdev.gateway[2], lwipdev.gateway[3]);
			break;
		}
		OS_Delay(250);
	}
	lwip_comm_dhcp_delete();
}
#endif
