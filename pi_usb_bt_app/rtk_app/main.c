#include "com.h" 
#include "bt.h"
#include <pthread.h>

#include <unistd.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <errno.h>

void *monitor_usb_pthread(void *arg);
//void *monitor_bt_pthread(void *arg);
#define BUFFER_SIZE 1200      /*最大缓存区*/

int fd;
int read_buffer_size;
unsigned char buf[1024] = {0};
extern int s,client, bytes_read,result,bt_state;
extern char bt_buf[1024];//,*addr;

int main(int argc, char *argv[])
{
    int nret;
    pthread_t usb_tid,bt_tid;
	char read_buffer[BUFFER_SIZE];

	//打开串口
	fd = open_port(0);
	//设置串口
    if(set_com_config(fd, 115200, 8, 'N', 1) < 0) /* 配置串口 */ 
    { 
        perror("set_com_config"); 
        return 1; 
    } 

#if 0
	//发送数据
	int rc,nread=0;
    fd_set rset;
    struct timeval tv;
    unsigned char buf[1024] = {0};
    int read_status = 0;
	int portfd = -1;
	portfd = fd;
	printf("fd = %d\n",fd);
#endif

    nret = pthread_create(&usb_tid,NULL,monitor_usb_pthread,NULL); 
    bt_config();
    nret = pthread_create(&bt_tid,NULL,monitor_bt_pthread,NULL); 
    if(nret != 0)
    {
        exit(-1);
    }

    nret = pthread_join(usb_tid,NULL);               
    if(nret != 0)
    {
        exit(-1);
    }

    if(nret != 0)
    {
        exit(-1);
    }

    nret = pthread_join(bt_tid,NULL);               
    if(nret != 0)
    {
        exit(-1);
    }    
    
#if 0
	do{
		do{
			memset(read_buffer,0, BUFFER_SIZE);
			read_buffer_size = read(fd, read_buffer, BUFFER_SIZE);				
		}while(!read_buffer_size);
		write(fd, read_buffer, read_buffer_size);
		printf("read[%d][%s]\n",  read_buffer_size,read_buffer);
	}while(1);
#endif
	close(fd);
	return 0;
}


void *monitor_usb_pthread(void *arg)
{
    int rc,i,nread=0;
    fd_set rset;
    struct timeval tv;

    int read_status = 0;

#if 1
    while(1)
    {
        FD_ZERO(&rset);
        FD_SET(fd,&rset);

        tv.tv_sec = 15;
        tv.tv_usec = 0;

        rc = select(fd+1,&rset,NULL,NULL,&tv);
        if(rc == -1)
        {
            continue;
        }
        if(rc == 0)
        {
            continue;
        }
        else
        {
			memset(buf,0, sizeof(buf));
			read_buffer_size = read(fd, buf, sizeof(buf));		
			//printf("read_buffer_size = %d\n",read_buffer_size);
			//printf("read[%d][%s]\n",  read_buffer_size,buf);
            //nread = read(fd,buf,sizeof(buf));   
            if(read_buffer_size == -1)
            {
                perror("read");
                usleep(10*1000);
                continue;
            }
            if(read_buffer_size == 0)
            {
                printf("nread==0\n");
                usleep(10*1000);
                continue;
            }
			//write(fd, buf, read_buffer_size);
            if(bt_state == 1)
            {
		        write(client,buf,read_buffer_size);
            }
        }
		//usleep(5*1000);
    }//END_while
#endif
}

