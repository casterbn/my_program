#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <linux/input.h>
#include <errno.h>
#include "ctr.h"
#include <dirent.h>
#include "main.h"


#define RTK_KEY_ESC (0x1B)
#define RTK_KEY_F2 (0x1B4F51)
#define RTK_KEY_F3 (0x1B4F52)
#define RTK_KEY_F4 (0x1B4F53)
#define RTK_KEY_F5 (0x5B31357F)
#define RTK_KEY_F6 (0x5B31377F)
#define RTK_KEY_F7 (0x5B31387F)
#define RTK_KEY_F8 (0x5B31397F)
#define RTK_KEY_F9 (0x5B32307F)
#define RTK_KEY_F10 (0x5B32317F)


#define RTK_KEY_1 (0x31)
#define RTK_KEY_2 (0x32)
#define RTK_KEY_3 (0x33)
#define RTK_KEY_4 (0x34)
#define RTK_KEY_5 (0x35)
#define RTK_KEY_6 (0x36)
#define RTK_KEY_7 (0x37)
#define RTK_KEY_8 (0x38)
#define RTK_KEY_9 (0x39)
#define RTK_KEY_0 (0x30)

#define RTK_KEY_M (0X6D)
#define RTK_KEY_C (0X63)
#define RTK_KEY_G (0X67)


char file_name[30][80];
int file_count;
char *file_read;

static int rtk_key_input_process(int keyvalue, char *input_buf, int input_len);
static void rtk_uart_input_process(char *input_buf, int intput_len);
static void *service_rtk_comm_thread(void *arg);

static void uart_open(int *fd);
static void uart_config(int fd);
static pthread_mutex_t rtk_ctrl_mutex;



static int match_tsenvno(const char *search)
{

}
void *rtk_key_input_monitor_thread(void *arg)
{
#ifndef DBUS_ENABLE

	char input_buf[20];
	int input_index = 0;
		while (1)
		{
			input_index = 0;
			while (1)
			{
				input_buf[input_index] = getchar();
				input_index++;
				if (input_index >= 20)
				{
					input_index = 0;
				}
				else if (input_buf[input_index - 1] == '\n')
				{
					break;
				}
			}
			printf("input_buf:%s  %d \n", input_buf, input_index - 1);
			rtk_key_input_process(0, input_buf, input_index - 1);
		}
#endif
}

extern int get_data;
static int rtk_key_input_process(int keyvalue, char *input_buf, int input_len)
{
	int key_value = 0;
	int i;
	int ret = 0;


	if (0 == keyvalue)
	{
		for (i = 0; i < input_len; i++)
		{
			key_value |= (input_buf[i] << (((input_len - i) - 1) * 8));
		}
	}
	else
	{
		//printf("keyvalue %d \n",keyvalue);
		key_value = keyvalue;
	}
	int file_index = key_value - 0x30;
	if((file_index > 0) && (file_index < 10))
	{
		file_read = file_name[file_index];
		printf("1111111111 index: %d\n",file_index);
		printf("1111111111 select file: %s\n",file_read);
		if(file_count > 0)
		{
			get_data = 1;
		}
	}
	pthread_mutex_lock(&rtk_ctrl_mutex);
	switch (key_value)
	{
		case RTK_KEY_F2:
			break;
		case RTK_KEY_F3:
			break;	
		case RTK_KEY_F4:
			break;
		case RTK_KEY_F5:
			break;
		case RTK_KEY_F7:
			break;
		case RTK_KEY_F8:
			break;
		case RTK_KEY_F10:
			break;
		case RTK_KEY_F9:
			break;
		case RTK_KEY_F6:
			break;
		case RTK_KEY_0:

		case RTK_KEY_1:
			//break;
		case RTK_KEY_2:
			//break;
		case RTK_KEY_3:
			//break;
		case RTK_KEY_4:

			//break;
		case RTK_KEY_5:
			//break;
		case RTK_KEY_6:
			break;
		case RTK_KEY_7:

			break;	
			//break;
		case RTK_KEY_8:
			//break;
		case RTK_KEY_9:
			break;			

		case RTK_KEY_M:

			break;
		case RTK_KEY_C:
			break;
		case RTK_KEY_G:
			printf("1111111111111\n");
			//get_data = 1;
			break;
	}
	pthread_mutex_unlock(&rtk_ctrl_mutex);
}

int rtk_input_monitor_thread_create(void)
{
	int rec = 0;

	pthread_t tid_rtk_key_input_monitor_thread;
	pthread_t tid_rtk_uart_input_monitor_thread;
	pthread_t tid_rtk_car_single_monitor_thread;
	pthread_mutex_init(&rtk_ctrl_mutex, NULL);
	for (int i = 0; i < file_count; i++)
	{
		printf("%d. %s\n", i, file_name[i]);
	}
	printf("please enter the num to select the file to decode:\n");
	if (0 != pthread_create(&tid_rtk_key_input_monitor_thread, NULL, rtk_key_input_monitor_thread, NULL))
	{
		rec = -1;
	}

}


int list_files(const char * dir_base)
{
    DIR *dir;
    int rec = -1;
    struct dirent *ptr;
    char base[1000];
    
    if ((dir=opendir(dir_base)) == NULL)
    {
        perror("Open dir error...");
        return rec;
    }
 
    while ((ptr=readdir(dir)) != NULL)
    {
        if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)    
            continue;
        else if(ptr->d_type == 8)    ///file
		{
            //DbgPrintf("1d_name:%s/%s\n",dir_base,ptr->d_name);
			if (strstr(ptr->d_name, "RTK_2019") != NULL)
			{
				memcpy(file_name[file_count++], ptr->d_name, strlen(ptr->d_name));
				//DbgPrintf("find txt\n");
			}
		}
        else if(ptr->d_type == 10)    ///link file
		{

		}
            //DbgPrintf("d_name:%s/%s\n",dir_base,ptr->d_name);
        else if(ptr->d_type == 4)    ///dir                 //目录
        {

        }
    }
    closedir(dir);
    return rec;
}

char* get_handle_file()
{
	return file_read;
}