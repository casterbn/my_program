#include "stm32f4xx_hal.h"
#include "osapi.h"
#include "osresources.h"
#include "lwip/opt.h"
#include "lwipcomm.h"
#include "lwip/lwip_sys.h"
#include "lwip/api.h"
#include "FreeRTOS.h"
#include "task.h"

#include "stdio.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "eth_ksz8041.h"
#include "http_server.h"
#include "lwipcomm.h"
#include "malloc.h"



#include "lwip/stats.h"
#include "lwip/tcp.h"
#include "RingBuffer.h"

#if 1
static const char *const HTTP_POST_HEAD;
//#define RTK_TCP_SERVER
const char* cloud_post = "GET /rtk HTTP/1.0\r\nUser-Agent: NTRIP Aceinna CloudRTK 1.0\r\nAuthorization: Basic Chenghe:XHWDKBRBFDFMYQBP\r\n\r\n" ;
uint8_t NETBUS_txBuffer[1000] = {"eth test\n"};
uint32_t NETBUS_txLength = 100;
struct netconn *tcp_clientconn = NULL;
uint8_t tcp_client_recvbuf[TCP_CLIENT_RX_BUFSIZE];
uint8_t tcp_client_flag;
static ip_addr_t loca_ipaddr;

static err_t tcp_server_accept(void *arg, struct tcp_pcb *pcb, err_t err);
static err_t tcp_server_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *tcp_recv_pbuf, err_t err);
void Tcp_Server_Init(void);
void EthTask(void *arg)
{
	uint32_t data_len = 0;
	struct pbuf *q;
	err_t err, recv_err;
	static ip_addr_t server_ipaddr;
	static u16_t server_port, loca_port;
	static uint32_t num = 0;

	my_mem_init(SRAMIN);

	while (lwip_comm_init())
	{
		OS_Delay(500);
	}

	LWIP_UNUSED_ARG(arg);
	server_port = REMOTE_PORT;
    IP4_ADDR(&loca_ipaddr, lwipdev.ip[0], lwipdev.ip[1], lwipdev.ip[2], lwipdev.ip[3]);
	IP4_ADDR(&server_ipaddr, lwipdev.remoteip[0], lwipdev.remoteip[1], lwipdev.remoteip[2], lwipdev.remoteip[3]);
#ifdef RTK_TCP_SERVER    
    //Tcp_Server_Init();
    Http_Server_Init();
#endif
	while (1)
	{
        //lwip_periodic_handle();
#ifdef RTK_TCP_SERVER    
        OS_Delay(10);
#endif
#ifndef RTK_TCP_SERVER
		tcp_clientconn = netconn_new(NETCONN_TCP);	// create a tcp client
        //IP4_ADDR(&server_ipaddr, 104,42,214,164);   //test
		err = netconn_connect(tcp_clientconn, &server_ipaddr, server_port); // connect server
        
		if (err != ERR_OK)
		{
			netconn_delete(tcp_clientconn);
		}
		else if (err == ERR_OK)
		{
			struct netbuf *recvbuf;
			tcp_clientconn->recv_timeout = 10;
			netconn_getaddr(tcp_clientconn, &loca_ipaddr, &loca_port, 1);
			printf("connect server:%d.%d.%d.%d, local port:%d\r\n", lwipdev.remoteip[0], lwipdev.remoteip[1], lwipdev.remoteip[2], lwipdev.remoteip[3], loca_port);
	        netconn_write(tcp_clientconn, cloud_post, strlen(cloud_post), NETCONN_COPY); // send data
            while (1)
			{
				if ((tcp_client_flag & LWIP_SEND_DATA) == LWIP_SEND_DATA) 
				{
					err = netconn_write(tcp_clientconn, NETBUS_txBuffer, sizeof(NETBUS_txBuffer), NETCONN_COPY); // send data
					if (err != ERR_OK)
					{
						printf("send err\r\n");
					}
					tcp_client_flag &= ~LWIP_SEND_DATA;
				}

				if ((recv_err = netconn_recv(tcp_clientconn, &recvbuf)) == ERR_OK) // recieve data
				{
					taskENTER_CRITICAL();
					memset(tcp_client_recvbuf, 0, TCP_CLIENT_RX_BUFSIZE); 
					for (q = recvbuf->p; q != NULL; q = q->next)
					{
						if (q->len > (TCP_CLIENT_RX_BUFSIZE - data_len))
							memcpy(tcp_client_recvbuf + data_len, q->payload, (TCP_CLIENT_RX_BUFSIZE - data_len));
						else
							memcpy(tcp_client_recvbuf + data_len, q->payload, q->len);
						data_len += q->len;
						if (data_len > TCP_CLIENT_RX_BUFSIZE)
							break;
					}
					taskEXIT_CRITICAL();
					printf("%s\r\n", tcp_client_recvbuf);
					netconn_write(tcp_clientconn, tcp_client_recvbuf, data_len, NETCONN_COPY); // send data back
					data_len = 0;
					netbuf_delete(recvbuf);
				}
				else if (recv_err == ERR_CLSD)
				{
					netconn_close(tcp_clientconn);
					netconn_delete(tcp_clientconn);
					printf("server:%d.%d.%d.%d disconnect\r\n", lwipdev.remoteip[0], lwipdev.remoteip[1], lwipdev.remoteip[2], lwipdev.remoteip[3]);
					break;
				}

				num++;
				if (num == 100)
				{
					//printf("send data\r\n");
					tcp_client_flag |= LWIP_SEND_DATA;
					num = 0;
				}
				OS_Delay(10);
			}
		}
#endif
	}
}


void Tcp_Server_Init(void)
{
    struct tcp_pcb *tcp_server_pcb;

    /* 为tcp服务器分配一个tcp_pcb结构体    */
    tcp_server_pcb = tcp_new();

    /* 绑定本地端号和IP地址 */
    tcp_bind(tcp_server_pcb, &loca_ipaddr, LOCAL_PORT);
    //netconn_bind(tcp_server_pcb, &loca_ipaddr, LOCAL_PORT);

    /* 监听之前创建的结构体tcp_server_pcb */
    tcp_server_pcb = tcp_listen(tcp_server_pcb);

    /* 初始化结构体接收回调函数 */
    tcp_accept(tcp_server_pcb, tcp_server_accept);
}

static err_t tcp_server_accept(void *arg, struct tcp_pcb *pcb, err_t err)
{
    tcp_setprio(pcb, TCP_PRIO_MIN);
    /* 确认监听与连接 */
    tcp_arg(pcb, mem_calloc(sizeof(struct name), 1));

    /* 发送一个建立连接的字符串 */
    //tcp_write(pcb, "hello my dream \n\r",strlen("hello my dream \n\r  "), 1);

    /* 配置接收回调函数 */
    tcp_recv(pcb, tcp_server_recv);

    return ERR_OK;
}

static err_t tcp_server_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *tcp_recv_pbuf, err_t err)
{
    struct pbuf *tcp_send_pbuf;
    struct name *name = (struct name *)arg;

    if (tcp_recv_pbuf != NULL)
    {
            /* 扩大收发数据的窗口 */
            tcp_recved(pcb, tcp_recv_pbuf->tot_len);

            if (!name)
            {
                    pbuf_free(tcp_recv_pbuf);
                    return ERR_ARG;
            }

            /* 将接收的数据拷贝给发送结构体 */
            tcp_send_pbuf = tcp_recv_pbuf;

            /* 换行 */
            tcp_write(pcb, "rtk receive:", strlen("rtk receive:"), 1);
            /* 将接收到的数据再转发出去 */
            tcp_write(pcb, tcp_send_pbuf->payload, tcp_send_pbuf->len, 1);

            pbuf_free(tcp_recv_pbuf);
    }
    else if (err == ERR_OK)
    {
            /* 释放内存 */
            mem_free(name);
            return tcp_close(pcb);
    }

    return ERR_OK;
}

#endif
