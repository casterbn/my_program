#include "lwip/debug.h"
#include <stdlib.h>

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


#include "lwipcomm.h"
#include "malloc.h"


#include "lwip/stats.h"
#include "lwip/tcp.h"
#include "RingBuffer.h"
#include "eth_ksz8041.h"
#include "http_server.h"
#include "string.h"

#define RTK_TCP_SERVER


static err_t tcp_server_accept(void *arg, struct tcp_pcb *pcb, err_t err);
static err_t tcp_server_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *tcp_recv_pbuf, err_t err);
void Tcp_Server_Init(void);

static ip_addr_t loca_ipaddr;


struct http_state {
  struct fs_file *handle;
  char *file;       /* Pointer to first unsent byte in buf. */

#if 1
  struct pbuf *req;
#endif /* LWIP_HTTPD_SUPPORT_REQUESTLIST */

#if 1
  char *buf;        /* File read buffer. */
  int buf_len;      /* Size of file read buffer, buf. */
#endif /* LWIP_HTTPD_SSI || LWIP_HTTPD_DYNAMIC_HEADERS */
  u32_t left;       /* Number of unsent bytes in buf. */
  u8_t retries;
#if 0
  const char *parsed;     /* Pointer to the first unparsed byte in buf. */
#if 1
  const char *tag_started;/* Poitner to the first opening '<' of the tag. */
#endif /* !LWIP_HTTPD_SSI_INCLUDE_TAG */
  const char *tag_end;    /* Pointer to char after the closing '>' of the tag. */
  u32_t parse_left; /* Number of unparsed bytes in buf. */
  u16_t tag_index;   /* Counter used by tag parsing state machine */
  u16_t tag_insert_len; /* Length of insert in string tag_insert */
#if 0
  u16_t tag_part; /* Counter passed to and changed by tag insertion function to insert multiple times */
#endif /* LWIP_HTTPD_SSI_MULTIPART */
  u8_t tag_check;   /* true if we are processing a .shtml file else false */
  u8_t tag_name_len; /* Length of the tag name in string tag_name */
  char tag_name[LWIP_HTTPD_MAX_TAG_NAME_LEN + 1]; /* Last tag name extracted */
  char tag_insert[LWIP_HTTPD_MAX_TAG_INSERT_LEN + 1]; /* Insert string for tag_name */
  enum tag_check_state tag_state; /* State of the tag processor */
#endif /* LWIP_HTTPD_SSI */
#if 0
  char *params[LWIP_HTTPD_MAX_CGI_PARAMETERS]; /* Params extracted from the request URI */
  char *param_vals[LWIP_HTTPD_MAX_CGI_PARAMETERS]; /* Values for each extracted param */
#endif /* LWIP_HTTPD_CGI */
#if 0
  const char *hdrs[NUM_FILE_HDR_STRINGS]; /* HTTP headers to be sent. */
  u16_t hdr_pos;     /* The position of the first unsent header byte in the
                        current string */
  u16_t hdr_index;   /* The index of the hdr string currently being sent. */
#endif /* LWIP_HTTPD_DYNAMIC_HEADERS */
#if 0
  u32_t time_started;
#endif /* LWIP_HTTPD_TIMING */
#if 0
  u32_t post_content_len_left;
#if 0
  u32_t unrecved_bytes;
  struct tcp_pcb *pcb;
  u8_t no_auto_wnd;
#endif /* LWIP_HTTPD_POST_MANUAL_WND */
#endif /* LWIP_HTTPD_SUPPORT_POST*/
};





/*
*********************************************************************************************************
*                                            LOCAL TABLES
*********************************************************************************************************
*/
static err_t http_server_accept(void *arg, struct tcp_pcb *pcb, err_t err);
static err_t http_server_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *tcp_recv_pbuf, err_t err);
static err_t http_server_poll(void *arg, struct tcp_pcb *pcb);
static err_t http_init_file(struct http_state *hs, struct fs_file *file, int is_09, const char *uri);
static err_t http_find_file(struct http_state *hs, const char *uri, int is_09);
static err_t http_parse_request(struct pbuf **inp, struct http_state *hs, struct tcp_pcb *pcb);
static u8_t http_send_data(struct tcp_pcb *pcb, struct http_state *hs);
static err_t http_sent(void *arg, struct tcp_pcb *pcb, u16_t len);
static void close_conn(struct tcp_pcb *pcb, struct http_state *hs);

/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

/***
* 函数名称 : Http_Server_Init();
*
* 函数描述 : web服务器初始化;
*
* 传递值    : 无;
*
* 返回值   : 无;
*
**/
void Http_Server_Init(void)
{
    struct tcp_pcb *http_server_pcb;

    /* 为web服务器分配一个tcp_pcb结构体 */
    http_server_pcb = tcp_new();

    /* 绑定本地端号和IP地址 */
    //tcp_bind(http_server_pcb, IP_ADDR_ANY, 80);
    tcp_bind(http_server_pcb, &loca_ipaddr, 80);
    /* 监听之前创建的结构体http_server_pcb */
    http_server_pcb = tcp_listen(http_server_pcb);

    /* 初始化结构体接收回调函数 */
    tcp_accept(http_server_pcb, http_server_accept);
}

/***
* 函数名称 : http_server_accept();
*
* 函数描述 : lwip数据接收回调函数，包含对tcp连接的确认，接收回调函数的配置;
*
* 传递值    : *arg, *pcb, err ;
*
* 返回值   : ERR_OK 无错误;
*
**/
static err_t http_server_accept(void *arg, struct tcp_pcb *pcb, err_t err)
{
    struct http_state *hs;

    /* 分配内存空间 */
    hs = (struct http_state *)mem_malloc(sizeof(struct http_state));

    if (hs != NULL)
    {
        memset(hs, 0, sizeof(struct http_state));
    }

    /* 确认监听和连接 */
    tcp_arg(pcb, hs);

    /* 配置接收回调函数 */
    tcp_recv(pcb, http_server_recv);

    /* 配置轮询回调函数 */
    tcp_poll(pcb, http_server_poll, 4);

    /* 配置发送回调函数 */
    tcp_sent(pcb, http_sent);

    return ERR_OK;
}

/***
* 函数名称 : http_server_recv();
*
* 函数描述 : 接受到数据后，根据接收到数据的内容，返回网页;
*
* 传递值    : *arg, *pcb, *http_recv_pbuf, err;
*
* 返回值   : ERR_OK无错误;
*
**/
#if 0
SOURCETABLE 200 OK \
Server: NTRIP BKG Caster 2.0.31/2.0 \
Date: Wed, 31 Jul 2019 07:38:54 GMT \
Connection: close \
Content-Type: text/plain \
Content-Length: 137 \
\
STR;SF01;CA;RTCM 3;1005,1074,1084,1094,1124;2;GPS+GLO+GAL+BDS;ACEINNA;USA;37.00;-121.00;0;0;FEMTOMES;none;N;N;3600;none \
ENDSOURCETABLE \
" ;
#endif


#if 0
const char* data_test = " \
<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.01 Transitional//EN\"> \
<html><head><title>STM32F4xx</title></head> \
<body style=\"color: black; background-color: white;\"> \
<table width=\"100%\"> \
<tbody> \
<tr valign=\"top\"> \
<td width=\"80\"><br> \
<div style=\"text-align: center;\"><img style=\"width: 96px; height: 57px;\" alt=\"ST logo\" src=\"STM32F4xx_files/logo.jpg\"></div> \
</td> \
<td width=\"500\"> \
<h1><small><small><small><small><big><big><big style=\"font-weight: bold;\"><big><strong><span style=\"font-style: italic;\">STM32F4xx Webserver Demo</span></strong></big></big></big></big></small></small></small></small> \
<small style=\"font-family: Verdana;\"><small><big><big><big><big style=\"font-weight: bold; color: rgb(51, 51, 255);\"><big><strong><span style=\"font-style: italic;\"></span></strong></big><span style=\"color: rgb(51, 51, 255);\"><br> \
</span></big></big></big></big></small></small></h1> \
<h2>404 - Page not found</h2> \
<p><span style=\"font-family: Times New Roman,Times,serif;\"> Sorry, \
the page you are requesting was not found on this server.</span> </p> \
</td> \
</tr> \
</tbody> \
</table> \
</body></html> \
" ;
#endif


const char* data_test = " \
<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.01 Transitional//EN\"> \
<html><head><title>ACEINNA</title></head> \
<body style=\"color: black; background-color: rgb(100,100,100);\"> \
<table width=\"100%\"> \
<tbody> \
<tr valign=\"top\"> \
<td width=\"80\"><br> \
</td> \
<td width=\"500\"> \
<h1><small><small><small><small><big><big><big style=\"font-weight: bold;\"><big><strong><span style=\"font-style: italic;\">ACEINNA OpenRTK Webserver Demo</span></strong></big></big></big></big></small></small></small></small> \
<small style=\"font-family: Verdana;\"><small><big><big><big><big style=\"font-weight: bold; color: rgb(51, 51, 255);\"><big><strong><span style=\"font-style: italic;\"></span></strong></big><span style=\"color: rgb(51, 51, 255);\"><br> \
</span></big></big></big></big></small></small></h1> \
<p><span style=\"font-family: Times New Roman,Times,serif;\"> Hello, \
welcome to visit this page.</span> </p> \
</td> \
</tr> \
</tbody> \
</table> \
</body></html> \
" ;

static err_t http_server_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *http_recv_pbuf, err_t err)
{
    err_t parsed = ERR_ABRT;
    struct http_state *hs = (struct http_state *)arg;

    /* 告诉tcp已经接收到数据 */
    tcp_recved(pcb, http_recv_pbuf->tot_len);
    //tcp_write(pcb, "rtk_test receive:", strlen("rtk_test receive:"), 1);

    printf("test\n");
    if (hs->handle == NULL)
    {
        /* 解析接收到的浏览器请求数据 */
        parsed = http_parse_request(&http_recv_pbuf, hs, pcb);
    }

    /* 清空请求字符串 */
    if (parsed != ERR_INPROGRESS) 
    {       
        if (hs->req != NULL) 
        {
            pbuf_free(hs->req);
            hs->req = NULL;
        }
    }

    if (parsed == ERR_OK)
    {
        /* 发送网页数据 */
        http_send_data(pcb, hs);
    }
    else if (parsed == ERR_ARG)
    {
        /* 关闭连接 */
        close_conn(pcb, hs);
    }

    return ERR_OK;
}

/***
* 函数名称 : http_server_poll();
*
* 函数描述 : 轮询函数;
*
* 传递值    : *arg, *pcb;
*
* 返回值   : ERR_OK无错误;
*
**/
static err_t http_server_poll(void *arg, struct tcp_pcb *pcb)
{
    struct http_state *hs = arg;

    if (hs == NULL)
    {
        close_conn(pcb, hs);
        return ERR_OK;
    }
    else
    {
        hs->retries++;
        if (hs->retries == 4)
        {
            close_conn(pcb, hs);
            return ERR_OK;
        }

        /* 如果连接存在打开的文件，则将会发送剩下的数据；
         * 如果一直没有收到GET请求，那么连接将会立刻关闭 */
        if (hs && (hs->handle))
        {
            if (http_send_data(pcb, hs))
            {
                tcp_output(pcb);
            }
        }
    }

    return ERR_OK;
}

/***
* 函数名称 : http_parse_request();
*
* 函数描述 : 对接收到的数据进行解析，根据不同的浏览器请求，返回对应的网页数据;
*
* 传递值    : **inp, *hs, *pcb;
*
* 返回值   : ERR_OK无错误;
*
**/
static err_t http_parse_request(struct pbuf **inp, struct http_state *hs, struct tcp_pcb *pcb)
{
    char *data;
    char *crlf;
    u16_t data_len;
    struct pbuf *p = *inp;

    char *sp1, *sp2;
    u16_t uri_len;
    char *uri;

    /* 排列字符串 */
    if (hs->req == NULL)
    {
        hs->req = p;
    }
    else
    {
        /* 将多次的请求字符串进行连接排序 */
        pbuf_cat(hs->req, p);
    }

    /* 拷贝输入数据 */ 
    if (hs->req->next != NULL)
    {
        data_len = hs->req->tot_len;
        pbuf_copy_partial(hs->req, data, data_len, 0);
    }
    else
    {
        data = (char *)p->payload;
        data_len = p->len;
    }

    /* 提取接收到的浏览器字符串，浏览器请求示例："GET / HTTP/1.1" */
    if (data_len > 7) 
    {
        crlf = strstr(data, "\r\n");
        if (crlf != NULL) 
        {
            /* 比较前4个字符是否为 "GET " */
            if (strncmp(data, "GET ", 4) == 0) 
            {
                /* sp1指向字符串 "/ HTTP/1.1" */
                sp1 = (data + 4);
                tcp_write(pcb, data_test, strlen(data_test), 1);
            }
            /* 在sp1字符串中寻找字符" "，sp2指向字符串 " HTTP/1.1" */
            sp2 = strstr(sp1, " ");
            /* uri_len获取sp1字符串首地址到sp2字符串首地址的长度 */
            uri_len = sp2 - (sp1);

            if ((sp2 != 0) && (sp2 >= (sp1))) 
            {
                /* 将解析的字符串赋给uri，并在最后加上结束符\0，
                   uri指向字符串 "/\0" */
                uri = sp1;
                *(sp1 - 1) = 0;
                uri[uri_len] = 0;

                /* 根据字符串寻找对应网页数据 */
                return http_find_file(hs, uri, 0); 
                }
            }
    }

    return ERR_OK;
}

/***
* 函数名称 : http_find_file();
*
* 函数描述 : 对提取的数据进行判断，读取对应的网页数据;
*
* 传递值    : *hs, *uri, is_09;
*
* 返回值   : ERR_OK无错误;
*
**/
static err_t http_find_file(struct http_state *hs, const char *uri, int is_09)
{
#if 0
    struct fs_file *file = NULL;

    /* 如果字符串为 "/\0",则打开index网页 */
    if((uri[0] == '/') && (uri[1] == 0)) 
    {
        file = fs_open("/index.html");
        uri = "/index.html";
    } 
    else 
    {
        /* 如果为其他请求，则打开相应网页 */
        file = fs_open(uri);
    }

    /* 将网页文件数据赋值给http_state结构体，之后发送出去 */
    return http_init_file(hs, file, is_09, uri);
#endif
}

/***
* 函数名称 : http_init_file();
*
* 函数描述 : 将要发送的数据保存到http_state结构体当中;
*
* 传递值    : *hs, *file, is_09, *uri;
*
* 返回值   : ERR_OK无错误;
*
**/
static err_t http_init_file(struct http_state *hs, struct fs_file *file_send, int is_09, const char *uri)
{
#if 0
    if (file_send != NULL) 
    {
        hs->handle = file_send;
        /* 将网页数据赋值给http_state */
        //hs->file = (char*)file->data;
        hs->file = file_send->file;   //my_test
        /* 将网页长度赋值给http_state */
        hs->left = file->len;
        //hs->left = file_send->buf_len;   //my_test
        hs->retries = 0;
    } 
    else 
    {
        hs->handle = NULL;
        hs->file = NULL;
        hs->left = 0;
        hs->retries = 0;
    }
#endif
    return ERR_OK;
}

/***
* 函数名称 : http_send_data();
*
* 函数描述 : 数据发送函数;
*
* 传递值    : *pcb, *hs;
*
* 返回值   : ERR_OK无错误;
*
**/
static u8_t http_send_data(struct tcp_pcb *pcb, struct http_state *hs)
{
    err_t err = ERR_OK;
    u16_t len;
    u8_t data_to_send = 0;

    /* 配置发送数据长度，如果发送数据过长则分批发送 */
    if (tcp_sndbuf(pcb) < hs->left)
    {
        len = tcp_sndbuf(pcb);
    }
    else
    {
        len = (u16_t)hs->left;
    }       

    /* 发送网页数据 */
    err = tcp_write(pcb, hs->file, len, 1);

    if (err == ERR_OK)
    {
        data_to_send = 1;
        hs->file += len;
        hs->left -= len;
    }
#if 0
    if ((hs->left == 0) && (fs_bytes_left(hs->handle) <= 0))
    {
        /* 关闭连接 */
        close_conn(pcb, hs);
        return 0;
    }
#endif
    return data_to_send;
}


/***
* 函数名称 : http_sent();
*
* 函数描述 : 数据已经被发送，并且被远程主机确定;
*
* 传递值    : *arg, *pcb, len;
*
* 返回值   : ERR_OK无错误;
*
**/
static err_t http_sent(void *arg, struct tcp_pcb *pcb, u16_t len)
{
    struct http_state *hs = (struct http_state *)arg;

    if (hs == NULL)
    {
        return ERR_OK;
    }

    hs->retries = 0;

    http_send_data(pcb, hs);

    return ERR_OK;
}

/***
* 函数名称 : close_conn();
*  * 函数描述 : 关闭tcp连接;
*  * 传递值     : *pcb, *hs;
*  * 返回值   : 无;
*  **/
static void close_conn(struct tcp_pcb *pcb, struct http_state *hs)
{
#if 0
    tcp_arg(pcb, NULL);
    tcp_recv(pcb, NULL);
    tcp_err(pcb, NULL);
    tcp_poll(pcb, NULL, 0);
    tcp_sent(pcb, NULL);

    if (hs != NULL) 
    {
        if(hs->handle) 
        {
            fs_close(hs->handle);
            hs->handle = NULL;
        }
            mem_free(hs);
    }

    tcp_close(pcb);
#endif
}