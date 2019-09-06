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

