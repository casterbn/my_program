/*******************************************************************************
* File Name          : rtk_bt.h
* Author             : Daich
* Revision           : 1.0
* Date               : 16/05/2019
* Description        : bluetooth
*
* HISTORY***********************************************************************
* 13/05/2019  |                                             | Daich
*
*******************************************************************************/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include <string.h>
#include "tool.h"
/*
相等返回1，不等返回2
*/
int array_cmp(void* src1,void* src2,int arr_len,int type_len)
{
	char* src3 = (char*) src1;
	char* src4 = (char*) src2;
	for(int i = 0;i < (arr_len * type_len);i++)
	{
		if(*(src3+i) != *(src4+i))
		{
			return -1;
		}
	}
	return 1;
}

#if 1
void sleep_s(int idle_s)
{
	vTaskDelay(1000 * idle_s/ portTICK_PERIOD_MS);
}
#endif

char strtrimall(unsigned char *pstr,uint32_t len)  //p ' ' q vallue
{
    char cmp[2048];
    int j = 0;

    //int len1 = strlen((const char*)pstr);
	//printf("len1 = %d",len1);
	//printf("pstr = %s",pstr);
    //int len = 0;
    //for(int i = 0;i < strlen((const char*)pstr);i++)
	for(int i = 0;i < len;i++)
    {
        if(*(pstr + i) != ' ')
        {
            cmp[j++] = *(pstr + i);
        }
    }
    cmp[j] = '\0';
    //strncpy(pstr,cmp,j);
	strcpy((char *)pstr,(const char *)cmp);
	//printf("pstr = %s",pstr);
    int len1 = strlen((const char*)pstr);
    int count = len - len1;
    //printf("1111 pstr = %s",pstr);
    return count;
}


#define MAXROV 1

#define MAXREF 1

#define MAXSTN (MAXROV+MAXREF)
#define RTCM3PREAMB 0xD3 /* rtcm ver.3 frame preamble */

unsigned int rtcm_getbitu(const unsigned char *buff, int pos, int len)
{
    unsigned int bits = 0;
    int i;
    for (i = pos; i < pos + len; i++)
        bits = (bits << 1) + ((buff[i / 8] >> (7 - i % 8)) & 1u);
    return bits;
}

int input_rtcm3_data(rtcm_t *rtcm, unsigned char data)
{

    /* synchronize frame */
    int type = 0, ret = 0;
    if (rtcm->nbyte == 0)
    {
        /* key = 0 => RTCM, key = 1 => NMEA */
        if (data == RTCM3PREAMB /* RTCM data format */
                                //  || data == '$'      /* NMEA data format */
        )
        {
            rtcm->key = data;
            rtcm->buff[rtcm->nbyte++] = data;
        }
        return 0;       //接收到起始符d3 返回
    }

    /* RTCM decorder */
    rtcm->buff[rtcm->nbyte++] = data;

    if (rtcm->nbyte == 3)
    {
        rtcm->len = rtcm_getbitu(rtcm->buff, 14, 10) + 3; /* length without parity */
        //printf("rtcm_len = %d\n",rtcm->len);
    }
    if (rtcm->nbyte < 3 || rtcm->nbyte < rtcm->len + 3)         //没有接收完成返回0
        return 0;
    rtcm->len_to_send = rtcm->nbyte;    
    rtcm->nbyte = 0;                                    //一包数据结束
    type = rtcm_getbitu(rtcm->buff, 24, 12);
    rtcm->type = type;
    //printf("type!!!!!!!!!!! = %x\n",type);
    return 1;
    /* check parity */
#if 0
    if (rtk_crc24q(rtcm->buff, rtcm->len) != rtcm_getbitu(rtcm->buff, rtcm->len * 8, 24))
    {
        trace(2, "rtcm3 parity error: len=%d\n", rtcm->len);
        return 0;
    }
#endif
    /* decode rtcm3 message */
    //return decode_rtcm3(rtcm, obs, nav);
}
#if 0
int input_rtcm3(unsigned char data, unsigned int stnID, gnss_rtcm_t *gnss)
{
    //trace(5, "input_rtcm3: data=%02x\n", data);
    rtcm_t *rtcm = NULL;
    obs_t *obs = NULL;
    nav_t *nav = NULL;
    int ret = 0;

    if (stnID < MAXSTN)
    {
        rtcm = gnss->rcv + stnID;
        nav = &gnss->nav;
#ifndef _USE_PPP_
        obs = stnID < MAXROV ? gnss->obs + stnID : gnss->obs_ref + (stnID - MAXROV);
#else
        obs = stnID < MAXROV ? gnss->obs + stnID : NULL;
#endif
        ret = input_rtcm3_data(rtcm, data, obs, nav);
        if (0 == stnID)
        {
            if (1 == ret && obs != NULL) {
                // short week = 0;
                // double time = time2gpst(obs->time, &week);
                // if (fabs(time - floor(time + 0.5)) > 0.01) {
                //     ret = -1;
                // }
                if (rtcm->time.time <= 0 || rtcm->time.sec < 0.0) {
                    ret = 0;

                    // if (gnss->rcv[1].time.time > 0) {
                    //     rtcm->time.time = gnss->rcv[1].time.time;
                    // }
                } 
                //else {
                //     short week = 0;
                //     double time = time2gpst(rtcm->time, &week);
                //     week = 0;
                // }
            }
        }
    }

    return ret;
}
#endif

void Hex2Str(char *sSrc,  char *sDest, int nSrcLen )
{
    int  i;
    char szTmp[3];
    for( i = 0; i < nSrcLen; i++ )
    {
        sprintf(szTmp, "%02x", sSrc[i]);
        memcpy(&sDest[i * 2], szTmp, 2 );
    }
}
