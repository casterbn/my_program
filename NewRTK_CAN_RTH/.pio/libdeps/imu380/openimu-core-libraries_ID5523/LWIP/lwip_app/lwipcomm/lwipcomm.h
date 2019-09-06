#ifndef _LWIPCOMM_H
#define _LWIPCOMM_H 

#include <stdio.h>

#define LWIP_MAX_DHCP_TRIES		4  

typedef struct {
	uint8_t mac[6];			
	uint8_t remoteip[4];	
	uint8_t ip[4];			
	uint8_t netmask[4]; 	
	uint8_t gateway[4]; 	
	uint8_t dhcpstatus;	
} __lwip_dev;

extern __lwip_dev lwipdev;

void lwip_pkt_handle(void);
void lwip_comm_default_ip_set(__lwip_dev *lwipx);
uint8_t lwip_comm_mem_malloc(void);
void lwip_comm_mem_free(void);
uint8_t lwip_comm_init(void);

void lwip_comm_dhcp_creat(void);
void lwip_comm_dhcp_delete(void);

#endif

