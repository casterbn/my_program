#include "lwip/debug.h"
#include "httpd.h"
#include "lwip/tcp.h"
#include "fs.h"
#include "lwipcomm.h"

#include <string.h>
#include <stdlib.h>
#include "stm32f4xx_hal.h"

#define NUM_CONFIG_SSI_TAGS 2
#define NUM_CONFIG_CGI_URIS 2
#define NUM_CONFIG_JS_URIS  1

const char *LED_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);
const char *CONFIG_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);

const char *CONFIG_JS_Handler(void);

static const char *ssiTAGs[] =
	{
		"n",
		"t",
};

static const tCGI cgiURIs[] =
	{
		{"/led.cgi", LED_CGI_Handler},
		{"/config.cgi", CONFIG_CGI_Handler},
};

static const tJS jsURIs[] =
	{
		{"/config.js", CONFIG_JS_Handler},
};

static int FindCGIParameter(const char *pcToFind, char *pcParam[], int iNumParams)
{
	int iLoop;
	for (iLoop = 0; iLoop < iNumParams; iLoop++)
	{
		if (strcmp(pcToFind, pcParam[iLoop]) == 0)
		{
			return (iLoop);
		}
	}
	return (-1);
}

void Freshnum_Handler(char *pcInsert)
{
	static uint16_t fnum = 0;

	fnum++;

	if (fnum >= 1000)
	{
		fnum = 0;
	}

	if (fnum < 10)
	{
		*pcInsert = (char)((fnum % 10) + 0x30);
	}
	else if (fnum < 100)
	{
		*pcInsert = (char)(((fnum % 100) / 10) + 0x30);
		*(pcInsert + 1) = (char)((fnum % 10) + 0x30);
	}
	else if (fnum < 1000)
	{
		*pcInsert = (char)(((fnum % 1000) / 100) + 0x30);
		*(pcInsert + 1) = (char)(((fnum % 1000) / 100) + 0x30);
		*(pcInsert + 2) = (char)((fnum % 10) + 0x30);
	}
}

void Time_Handler(char *pcInsert)
{
	uint16_t time;

	*pcInsert = '2';
	*(pcInsert + 1) = '0';
}

static u16_t SSIHandler(int iIndex, char *pcInsert, int iInsertLen)
{
	switch (iIndex)
	{
	case 0:
		Freshnum_Handler(pcInsert);
		break;
	case 1:
		Time_Handler(pcInsert);
		break;
	}
	return strlen(pcInsert);
}

const char *LED_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
	iIndex = FindCGIParameter("LED", pcParam, iNumParams);

	if (iIndex != -1)
	{
		for (uint8_t i = 0; i < iNumParams; i++)
		{
			if (strcmp(pcParam[i], "LED") == 0)
			{
				if (strcmp(pcValue[i], "LEDON") == 0)
				{
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
				}
				else if (strcmp(pcValue[i], "LEDOFF") == 0)
				{
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
				}
			}
		}
	}

	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET)
		return "/LED_OFF.shtml";
	else
		return "/LED_ON.shtml";
}

typedef struct _CONFIG_MSG
{
	uint8_t mac[6];
	uint8_t ip[4];
	uint8_t sub[4];
	uint8_t gw[4];
	uint8_t dns[4];
	uint8_t rip[4];
	uint8_t sw_ver[2];
} CONFIG_MSG;

CONFIG_MSG ConfigMsg;

uint8_t http_response[1460];
uint8_t tx_buf[1460];

char c2d(uint8_t c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'a' && c <= 'f')
		return 10 + c - 'a';
	if (c >= 'A' && c <= 'F')
		return 10 + c - 'A';

	return (char)c;
}

uint16_t atoi16(char *str, uint16_t base)
{
	unsigned int num = 0;
	while (*str != 0)
		num = num * base + c2d(*str++);
	return num;
}

void inet_addr_(unsigned char *addr, unsigned char *ip)
{
	int i;
	char taddr[30];
	char *nexttok;
	char num;
	strcpy(taddr, (char *)addr);

	nexttok = taddr;
	for (i = 0; i < 4; i++)
	{
		nexttok = strtok(nexttok, ".");
		if (nexttok[0] == '0' && nexttok[1] == 'x')
			num = atoi16(nexttok + 2, 0x10);
		else
			num = atoi16(nexttok, 10);

		ip[i] = num;
		nexttok = NULL;
	}
}

const char *CONFIG_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
	iIndex = FindCGIParameter("ip", pcParam, iNumParams);
	if (iIndex != -1)
	{
		inet_addr_(pcValue[iIndex], ConfigMsg.ip);
	}
	
	iIndex = FindCGIParameter("sub", pcParam, iNumParams);
	if (iIndex != -1)
	{
		inet_addr_(pcValue[iIndex], ConfigMsg.sub);
	}
	
	iIndex = FindCGIParameter("gw", pcParam, iNumParams);
	if (iIndex != -1)
	{
		inet_addr_(pcValue[iIndex], ConfigMsg.gw);
	}

	return "/CONFIG.shtml";
}

const char *CONFIG_JS_Handler(void)
{
	memset(http_response, 0, 1460);
	memset(tx_buf, 0, 1460);

	sprintf((char *)tx_buf, "settingsCallback({\"ver\":\"%d.%d\",\"mac\":\"%02X:%02X:%02X:%02X:%02X:%02X\",\"ip\":\"%d.%d.%d.%d\",\"gw\":\"%d.%d.%d.%d\",\"sub\":\"%d.%d.%d.%d\",});",
			ConfigMsg.sw_ver[0], ConfigMsg.sw_ver[1],
			ConfigMsg.mac[0], ConfigMsg.mac[1], ConfigMsg.mac[2], ConfigMsg.mac[3], ConfigMsg.mac[4], ConfigMsg.mac[5],
			ConfigMsg.ip[0], ConfigMsg.ip[1], ConfigMsg.ip[2], ConfigMsg.ip[3],
			ConfigMsg.gw[0], ConfigMsg.gw[1], ConfigMsg.gw[2], ConfigMsg.gw[3],
			ConfigMsg.sub[0], ConfigMsg.sub[1], ConfigMsg.sub[2], ConfigMsg.sub[3]);

	sprintf((char *)http_response, "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length:%d\r\n\r\n%s", strlen(tx_buf), tx_buf);

	return (char *)http_response;
}

void httpd_ssi_init(void)
{
	http_set_ssi_handler(SSIHandler, ssiTAGs, NUM_CONFIG_SSI_TAGS);

	uint8_t mac[6] = {2, 0, 0, 0, 0x64, 0x34};
	uint8_t ip[4] = {192, 168, 1, 110};
	uint8_t sub[4] = {255, 255, 255, 0};
	uint8_t gw[4] = {192, 168, 1, 1};
	uint8_t sw_ver[2] = {0x31, 0x45};

	memcpy(ConfigMsg.mac, mac, 6);
	memcpy(ConfigMsg.ip, ip, 4);
	memcpy(ConfigMsg.sub, sub, 4);
	memcpy(ConfigMsg.gw, gw, 4);
	memcpy(ConfigMsg.sw_ver, sw_ver, 2);
}

void httpd_cgi_init(void)
{
	http_set_cgi_handlers(cgiURIs, NUM_CONFIG_CGI_URIS);
}

void httpd_js_init(void)
{
	http_set_js_handlers(jsURIs, NUM_CONFIG_JS_URIS);
}
