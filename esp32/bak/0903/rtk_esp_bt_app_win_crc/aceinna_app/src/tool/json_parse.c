/*
  Copyright (c) 2009 Dave Gamble
 
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:
 
  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.
 
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cJSON.h"
#include "json_parse.h"
#include "rtk_bt.h"
#include "rtk_uart.h"

extern bt_uart_baud_e BAUD_RATE; 
extern char* baud_rate_str[3];
char config_json_from_rtk[512];
extern const char* VERSION;
/* Parse text to JSON, then render back to text, and print! */

int change_item(char *text,char* json_to_write,char* key,char* value)
{
#if 1
	char *out;
    cJSON *root;
    cJSON *item;
	root = cJSON_Parse(text);

	if (!root)
    {
        //printf("Error before: [%s]\n",cJSON_GetErrorPtr());
        printf("not json format\r\n");
        return json_false;
    }
	else
	{
        item = cJSON_GetObjectItem(root,"esp32 config");
        memcpy(cJSON_GetObjectItem(item,key)->valuestring,value,strlen(value)); // how to modify value safety?
        //cJSON_GetObjectItem(item,key)->valuestring = value;
        out = cJSON_Print(root);
     
//        if(strlen(out) < 1024)
        {
            memcpy(json_to_write,out,strlen(out) + 1);
        }           
        //printf("after modify:%s\n",out);
        //printf("after json_to_write:%s\n",json_to_write);        
        cJSON_Delete(root);   
        free(out);  
        return json_true;
	}
#endif
}

int doit(char *text)    //添加esp32
{
#if 1
	char *out;
    cJSON *json;
	cJSON *fmt;
    int bt_num = get_ble_num();    
	json = cJSON_Parse(text);
    char int_to_s[2];
	if (!json)
    {
        //printf("Error before: [%s]\n",cJSON_GetErrorPtr());
        printf("not json format\r\n");
        return json_false;
    }
	else
	{
        cJSON_AddItemToObject(json, "esp32 config", fmt=cJSON_CreateObject());
        cJSON_AddItemToObject(fmt, "esp32_version", cJSON_CreateString(VERSION)); //TODO: 
        cJSON_AddItemToObject(fmt, "BaudRate", cJSON_CreateString(baud_rate_str[BAUD_RATE]));
        itoa(bt_num,int_to_s,2);
        cJSON_AddItemToObject(fmt, "connect_device_num", cJSON_CreateString(int_to_s));        
        out=cJSON_Print(json);
        cJSON_Delete(json);
        //printf("%s\n",out);
        if(strlen(out) < 512)
        {
            memcpy(config_json_from_rtk,out,strlen(out) + 1);
        }     
        //printf("%s\n",out);
        free(out);           
#if 0
		out = cJSON_Print(json);
		cJSON_Delete(json);
		printf("%s\n",out);
		free(out);
#endif
        return json_true;
	}
#endif
}

/* Read a file, parse, render back, etc. */
void dofile(char *filename)
{
	FILE *f;long len;char *data;
	
	f=fopen(filename,"rb");fseek(f,0,SEEK_END);len=ftell(f);fseek(f,0,SEEK_SET);
	data=(char*)malloc(len+1);fread(data,1,len,f);fclose(f);
	doit(data);
	free(data);
}

/* Used by some code below as an example datatype. */
struct record {const char *precision;double lat,lon;const char *address,*city,*state,*zip,*country; };

/* Create a bunch of objects as demonstration. */
void create_objects(char* json_to_str)
{
	cJSON *root,*fmt,*json;
    char *out;int i;	/* declare a few. */
	/* Our "Video" datatype: */
#if 0    
	root=cJSON_CreateObject();
	cJSON_AddItemToObject(root, "board", cJSON_CreateString("openrtk"));
   	cJSON_AddItemToObject(root, "version", cJSON_CreateString("v_1.0"));
	cJSON_AddItemToObject(root, "is_bt_use", cJSON_CreateString("yes"));
    cJSON_AddItemToObject(root, "bt_data_type", cJSON_CreateString("rtk"));

	out=cJSON_Print(root);
    cJSON_Delete(root);
    printf("%s\n",out);

    free(out);
#endif
    int bt_num = get_ble_num();    
    char int_to_s[2];

	json = cJSON_CreateObject();
    cJSON_AddItemToObject(json, "esp32 config", fmt=cJSON_CreateObject());
    cJSON_AddItemToObject(fmt, "esp32_version", cJSON_CreateString(VERSION)); //TODO: 
    cJSON_AddItemToObject(fmt, "BaudRate", cJSON_CreateString(baud_rate_str[BAUD_RATE]));
    itoa(bt_num,int_to_s,2);
    cJSON_AddItemToObject(fmt, "connect_device_num", cJSON_CreateString(int_to_s));        
    out=cJSON_Print(json);
    cJSON_Delete(json);
    //printf("%s\n",out);
    if(strlen(out) < 512)
    {
        memcpy(json_to_str,out,strlen(out) + 1);
    }     
    //printf("%s\n",out);
    free(out);           
    return json_true;

}

int json_create() { //ignore
	/* Now some samplecode for building objects concisely: */
	//create_objects();
	return 0;
}
