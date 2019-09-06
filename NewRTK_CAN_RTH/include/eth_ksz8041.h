#ifndef _ETH_KSZ8041_H_
#define _ETH_KSZ8041_H_

#include <stdio.h>

#if 1
#define RTK_TCP_SERVER
#define TCP_CLIENT_RX_BUFSIZE	1500	
//#define REMOTE_PORT				8080	
#ifdef RTK_TCP_SERVER
#define REMOTE_PORT				80
#else
#define REMOTE_PORT				8080	
#endif
#define LOCAL_PORT				8080	
#define LWIP_SEND_DATA			0x80   

#define     MAX_NAME_SIZE       32
struct name
{
        int     length;
        char    bytes[MAX_NAME_SIZE];
};

extern uint8_t tcp_client_recvbuf[TCP_CLIENT_RX_BUFSIZE];	
extern uint8_t tcp_client_flag;	


void EthTask(void *arg);

#endif


#endif