//#include <wiringSerial.h>
#include "wiringSerial.h"
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <pthread.h>
#include <sys/select.h>  
#include "rtk.h"


//#define F_PATH "/home/pi/rtk.csv"
char F_PATH[40];
#define F_FORMAT ".csv"
#define GPS_ONLY
u8          UartRxBuff[RX_SIZE];
RingBuffer  UartRxRing;
u8          ParseBuff[RX_SIZE];
RingBuffer  ParseRing;

int portfd = -1;
u8 dest[4096]={0};
int dest_cnt = 0;

FILE *f0;
char array[] = "timer,accX,accY,accZ,gyroX,gyroY,gyroZ,GPS\n";  //时间、三轴加速度、三轴角度、GPS

void print_frame(const char *desc,u8 *buf,int size);
int getCompleteFrame(u8 *inBuf,int inCnt,u8 *outBuf,int *destCnt,int *readStatus);
void *monitor_serial_readable(void *arg);
void *HandleSerialData(void *arg);
static void getTimeString(char *str, int len,char *file_format);

int main(int argc, int *argv[]) //int serialOpen (const char *device, const int baud)
{
    int fd;
    int nret;
    pthread_t wtid;
    pthread_t rtid;
    int i;
    if(argc > 1)
	{
		printf("argc = %d\n",argc);
		printf("argv = %s\n",argv[1]);
		
		if(strcmp(argv[1],"test") == 0)
		{
			
			for(i = 0;i<25;i++)
			{
				printf("dch test start\n");
				sleep(1);

			}
		}	
	}
    //char file_csv[32];
  
    getTimeString(F_PATH,32,F_FORMAT);           //创建以日期为命名的csv文件
	if(NULL == (f0=fopen(F_PATH,"a+")))
	{                                                 
		printf("can't  not open file\n");
	}
	else
	{
		printf("Create file succeed!\n");
		fwrite(array,sizeof(array),1,f0);
        fclose(f0);
	}


    InitRingBuffer(&UartRxRing,UartRxBuff,RX_SIZE);
    InitRingBuffer(&ParseRing,ParseBuff,RX_SIZE);
    /* open serial port */
    //if((fd = serialOpen("/dev/ttyAMA0", 460800)) < 0)
		
	if((fd = serialOpen("/dev/ttyAMA0", 460800)) < 0)
    {
        fprintf(stderr,"Unable to open serial device: %s\n", strerror(errno));
		printf("error code %s",stderr);
        return 1;
    }
 	printf("Usart send Test,Just by launcher!\n");
    portfd=fd;

    nret = pthread_create(&rtid,NULL,HandleSerialData,NULL);        //处理串口队列
    if(nret != 0)
    {
        exit(-1);
    }

    nret = pthread_create(&wtid,NULL,monitor_serial_readable,NULL);  //侦听串口
    if(nret != 0)
    {
        exit(-1);
    }

     nret = pthread_join(rtid,NULL);                //等待线程结束
    if(nret != 0)
    {
        exit(-1);
    }

    nret = pthread_join(wtid,NULL);
    if(nret != 0)
    {
        // DEBUG_ERR(join thread failed);
        exit(-1);
    }

    close(portfd);
//     for(;;)
//     {
//         if(serialDataAvail(fd) > 0)
//         {
// //           putchar(serialGetchar(fd));
// 			printf("char %x,\n",serialGetchar(fd));
//         }
//     }
    return 0;
}




void print_frame(const char *desc,u8 *buf,int size)
{
    int i;

    printf("[%s] [LEN=%d]",desc,size);
    for(i=0; i<size; i++)
    {
        printf("[%x]",buf[i]);
    }
    printf("\n");
}

void *monitor_serial_readable(void *arg)          //侦听串口数据并写入队列
{
    int rc,i,nread=0;
    fd_set rset;
    struct timeval tv;
    u8 buf[1024] = {0};
    int read_status = 0;

    while(1)
    {
        FD_ZERO(&rset);
        FD_SET(portfd,&rset);

        tv.tv_sec = 5;
        tv.tv_usec = 0;

        rc = select(portfd+1,&rset,NULL,NULL,&tv);
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
            nread = read(portfd,buf,sizeof(buf));   
            if(nread == -1)
            {
                perror("read");
                usleep(10*1000);
                continue;
            }
            if(nread == 0)
            {
                printf("nread==0\n");
                usleep(10*1000);
                continue;
            }
            // printf("nread = %d\n",nread);
            //  for(i=0; i<nread; i++)
            // {
            //     printf("[%x]",buf[i]);
            // }
            // printf("\n");
            WriteRingBuffer(&UartRxRing,buf,nread);
        }
        usleep(5*1000);
    }//END_while
}

float exchange_data(u8 *data)
{
 
	float float_data;
	float_data = *((float*)data);
	return float_data;
}

int   TotalRxCnt=0;
u8    longBuff[4096]={0};
int   longIndex=0;

// u8    frameBuff[4096]={0};
// int   frameIndex=0;

    int framIndexBuff[100];
    int framNum=0;
void *HandleSerialData(void *arg)
{
    int i;
    u8 buf[4096] = {0};
    int read_status = 0;
    int parse_size=0;
    int read_index=0;
    int valid_len=0;
    u8  write_buff[500];
    //int j;
    while(1)
    {
 
            TotalRxCnt =  ReadAllDataNoDeel(&UartRxRing,&longBuff[longIndex]);
            longIndex += TotalRxCnt;

            if(longIndex > 2* sizeof (UcbPacketStruct))
            {
                // for(i=0;i<longIndex;i++)
                // {
                //     printf("[%x]",longBuff[i]);
                // }
                findFrame(longBuff,longIndex,framIndexBuff,&framNum);
                longIndex=0;
            }
                if(framNum!=0)
                {
    
                    UcbPacketStruct *pPack;            //数据包
                    Data1Payload_t  *pData1Payload;

                    if(NULL == (f0=fopen(F_PATH,"a")))
                    {                                                 
                        printf("can't  not open file\n");
                    }
                    printf("frame %d \n",framNum);
                    for(i=0;i<framNum-1;i++)                    //读取所有有效数据
                    {
                        
                        pPack=&longBuff[framIndexBuff[i]];     //获取帧头地址
                        // printf("code_MSB= %x \n", pPack->code_MSB);
                        // printf("code_LSB= %x \n", pPack->code_LSB);
                        printf("payloadLength= %d \n", pPack->payloadLength);  //有效数据开始
                         pData1Payload=(Data1Payload_t  *)pPack->payload;
                        printf("pPayload= %ld \n", pData1Payload);
                        printf("timer= %ld ", pData1Payload->timer);
                        printf("accDataX= %f ",  exchange_data(&pData1Payload->accData[0]));
                        printf("accDataY= %f ",  exchange_data(&pData1Payload->accData[1]));
                        printf("accDataZ= %f ",  exchange_data(&pData1Payload->accData[2]));
                        printf("gyrDataX= %f ",  exchange_data(&pData1Payload->gyroData[0]));
                        printf("gyrDataY= %f ",  exchange_data(&pData1Payload->gyroData[1]));
                        printf("gyrDataZ= %f\n ",  exchange_data(&pData1Payload->gyroData[2]));
                        if(!memcmp(pData1Payload->gpsData,"GPSB",4))
                        {
                            printf("gpsDataLength= %d \n", pPack->payloadLength-40);
                            // printf("gpsBeginSymbol= %s  ",  pData1Payload->gpsData);
                            for(int j=0;j<pPack->payloadLength-40;j++)
                                printf("gpsData= %1x \n",  pData1Payload->gpsData[j]);
                            
                        }
                        else
                        {
                            printf(" \n",  pData1Payload->gpsData);
                        }
                        int len;
                        
#ifndef GPS_ONLY                        
                            len= sprintf(write_buff,"%ld,",pData1Payload->timer);
                            fwrite(write_buff,sizeof(char),len,f0);
                            len= sprintf(write_buff,"%f,",exchange_data(&pData1Payload->accData[0]));
                            fwrite(write_buff,sizeof(char),len,f0);
                            len= sprintf(write_buff,"%f, ",exchange_data(&pData1Payload->accData[1]));
                            fwrite(write_buff,sizeof(char),len,f0);
                            len= sprintf(write_buff,"%f, ",exchange_data(&pData1Payload->accData[2]));
                            fwrite(write_buff,sizeof(char),len,f0);

                            len= sprintf(write_buff,"%f, ",exchange_data(&pData1Payload->gyroData[0]));
                            fwrite(write_buff,sizeof(char),len,f0);
                            len= sprintf(write_buff,"%f, ",exchange_data(&pData1Payload->gyroData[1]));
                            fwrite(write_buff,sizeof(char),len,f0);
                            len= sprintf(write_buff,"%f, ",exchange_data(&pData1Payload->gyroData[2]));
                            fwrite(write_buff,sizeof(char),len,f0);
#endif
                            if(!memcmp(pData1Payload->gpsData,"GPSB",4))
                            {
                                // len= sprintf(write_buff,"%s, \n", pData1Payload->gpsData);
                                 for(int j=0;j<pPack->payloadLength-40-4;j++)   //写gps数据
                                 {
                                    len=sprintf(write_buff,"%1x ", pData1Payload->gpsData[j+4]);
                                    fwrite(write_buff,sizeof(char),len,f0);
                                 }
                                 len=sprintf(write_buff,",\n");
                                 fwrite(write_buff,sizeof(char),len,f0);
                            }
                            else
                            {
                                //len= sprintf(write_buff,"%s, \n","NULL");
								len= sprintf(write_buff," \n");
                                fwrite(write_buff,sizeof(char),len,f0);
                            }
                        

                    }
       
                   
                    fclose(f0);//鍏抽棴鏂囦欢

  
                    framNum=0;
                }

            // while(longIndex>sizeof (Data1Payload_t))
            // {
            //     if(findHead(longBuff,longIndex,&read_index))//header
            //     {
            //        valid_len=longIndex-read_index;
            //        memcpy(frameBuff,&longBuff[read_index],valid_len); 
            //        WriteRingBuffer(&ParseRing,frameBuff,valid_len);
            //        longIndex=0;                             //all useful data haven been copy,resume inde                   
            //     printf("read frame =%d \n",valid_len);
            //          for(i=0; i<valid_len; i++)
            //         {
            //             printf("[%x]",frameBuff[read_index+i]);
            //         }
            //         printf("\n");
            //     }
            //     else
            //     {
            //         /* code */
            //     }
                
            // }
            
            usleep(5*1000);
    }//END_while
}


static void getTimeString(char *str, int len,char *file_format)
{
    time_t timep;
    struct tm *p;
    int year, month, day, hour, min, sec;

    time(&timep);
    p = localtime(&timep);
    year = p->tm_year + 1900;
    month = p->tm_mon + 1;
    day = p->tm_mday;
    hour = p->tm_hour;
    min = p->tm_min;
    sec = p->tm_sec;
 	printf("year=%d,mouth=%02d,day=%02d\n", year, month, day);   
	snprintf(str, len, "%s_%d-%02d-%02d-%02d_%02d_%02d%s", "RTK",year, month, day, hour, min, sec,file_format);
    printf("file_name = %s\n",str);
}



