#ifndef _TASK_CLIENT_H_
#define _TASK_CLIENT_H_

#include <stdio.h>


#define TCP_CLIENT_RX_BUFSIZE	1500	
#define REMOTE_PORT				8087	
#define LWIP_SEND_DATA			0x80   


extern uint8_t tcp_client_recvbuf[TCP_CLIENT_RX_BUFSIZE];	
extern uint8_t tcp_client_flag;	


void ClientTask(void *arg);

#endif