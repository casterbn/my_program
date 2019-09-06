//#include <wiringSerial.h>
#include "wiringSerial.h"
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <pthread.h>
#include <mntent.h>  
#include <sys/select.h>  
#include <unistd.h>
#include <dirent.h>
#include <sys/stat.h>
#include "rtk.h"
#include "main.h"
#include "handle_data.h"
//#define F_PATH "/home/pi/rtk.csv"
char F_PATH[80];
#define F_FORMAT ".csv"
//#define GPS_ONLY
u8          UartRxBuff[RX_SIZE];
RingBuffer  UartRxRing;
u8          ParseBuff[RX_SIZE];
RingBuffer  ParseRing;

int portfd = -1;
u8 dest[4096]={0};
int dest_cnt = 0;

FILE *f0;
char array[] = "timer,accX,accY,accZ,gyroX,gyroY,gyroZ,GPS\n";  //ʱ�䡢������ٶȡ�����Ƕȡ�GPS

void print_frame(const char *desc,u8 *buf,int size);
int getCompleteFrame(u8 *inBuf,int inCnt,u8 *outBuf,int *destCnt,int *readStatus);
void *monitor_serial_readable(void *arg);
void *HandleSerialData(void *arg);
static void get_time_crate_file(char* floder,char *str, int len,char *file_format);
static int check_usb_is_mount(void);
static void read_mount_stat();
int read_mount_node(char *basePath,char* node_path);
int main(int argc, int *argv[]) //int serialOpen (const char *device, const int baud)
{
    int fd;
    int nret;
    pthread_t wtid;
    pthread_t rtid;
    int i;
    //test();
    //return;
    if(argc > 1)
	{
		DbgPrintf("argc = %d\n",argc);
		DbgPrintf("argv = %s\n",argv[1]);
		
		if(strcmp(argv[1],"test") == 0)
		{
			
			for(i = 0;i<25;i++)
			{
				DbgPrintf("dch test start\n");
				sleep(1);

			}
		}	
	}
    //char file_csv[32];
	DbgPrintf("check_usb_is_mount = %d\n",check_usb_is_mount());
	char str[30] = {0};
	int rec = read_mount_node("/media/pi",str);
	DbgPrintf("str = %s\n",str);
	char floder[30] = {0};
	if(rec == 1)
	{
		sprintf(floder, "%s%s%s", "/media/pi/",str,"/");
		DbgPrintf("cat\n");
	}
    DbgPrintf("test1\n");
    get_time_crate_file(floder,F_PATH,80,F_FORMAT);           //����������Ϊ������csv�ļ�
    DbgPrintf("test2\n");
	if(NULL == (f0=fopen(F_PATH,"a+")))
	{
		DbgPrintf("can't  not open file\n");
	}
	else
	{
		DbgPrintf("Create file succeed!\n");
		//fwrite(array,sizeof(array),1,f0);
        fclose(f0);
	}


    InitRingBuffer(&UartRxRing,UartRxBuff,RX_SIZE);
    InitRingBuffer(&ParseRing,ParseBuff,RX_SIZE);
    /* open serial port */
    //if((fd = serialOpen("/dev/ttyAMA0", 460800)) < 0)
		
	if((fd = serialOpen("/dev/ttyAMA0", 460800)) < 0)
    {
        fprintf(stderr,"Unable to open serial device: %s\n", strerror(errno));
		DbgPrintf("error code %s",stderr);
        return 1;
    }
 	DbgPrintf("Usart send Test,Just by launcher!\n");
    portfd=fd;

    nret = pthread_create(&rtid,NULL,HandleSerialData,NULL);        //�������ڶ���
    if(nret != 0)
    {
        exit(-1);
    }

    nret = pthread_create(&wtid,NULL,monitor_serial_readable,NULL);  //��������
    if(nret != 0)
    {
        exit(-1);
    }

     nret = pthread_join(rtid,NULL);                //�ȴ��߳̽���
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
// 			DbgPrintf("char %x,\n",serialGetchar(fd));
//         }
//     }
    return 0;
}




void print_frame(const char *desc,u8 *buf,int size)
{
    int i;

    DbgPrintf("[%s] [LEN=%d]",desc,size);
    for(i=0; i<size; i++)
    {
        DbgPrintf("[%x]",buf[i]);
    }
    DbgPrintf("\n");
}

void *monitor_serial_readable(void *arg)          //�����������ݲ�д�����
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
                DbgPrintf("nread==0\n");
                usleep(10*1000);
                continue;
            }
            // DbgPrintf("nread = %d\n",nread);
            //  for(i=0; i<nread; i++)
            // {
            //     DbgPrintf("[%x]",buf[i]);
            // }
            // DbgPrintf("\n");
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
                //     DbgPrintf("[%x]",longBuff[i]);
                // }
                findFrame(longBuff,longIndex,framIndexBuff,&framNum);
                longIndex=0;
            }
                if(framNum!=0)
                {
    
                    UcbPacketStruct *pPack;            //���ݰ�
                    Data1Payload_t  *pData1Payload;

                    if(NULL == (f0=fopen(F_PATH,"a")))
                    {                                                 
                        DbgPrintf("can't  not open file\n");
                    }
                    DbgPrintf("frame %d \n",framNum);
                    for(i=0;i<framNum-1;i++)                    //��ȡ������Ч����
                    {
                        
                        pPack=&longBuff[framIndexBuff[i]];     //��ȡ֡ͷ��ַ
                        // DbgPrintf("code_MSB= %x \n", pPack->code_MSB);
                        // DbgPrintf("code_LSB= %x \n", pPack->code_LSB);
                        DbgPrintf("payloadLength= %d \n", pPack->payloadLength);  //��Ч���ݿ�ʼ
                        pData1Payload=(Data1Payload_t  *)pPack->payload;
                        //DbgPrintf("pPayload= %ld \n", pData1Payload);
                        DbgPrintf("timer= %ld ", pData1Payload->timer);
                        DbgPrintf("accDataX= %f ",  exchange_data(&pData1Payload->accData[0]));
#if 0
                        DbgPrintf("accDataY= %f ",  exchange_data(&pData1Payload->accData[1]));
                        DbgPrintf("accDataZ= %f ",  exchange_data(&pData1Payload->accData[2]));
                        DbgPrintf("gyrDataX= %f ",  exchange_data(&pData1Payload->gyroData[0]));
                        DbgPrintf("gyrDataY= %f ",  exchange_data(&pData1Payload->gyroData[1]));
                        DbgPrintf("gyrDataZ= %f\n ",  exchange_data(&pData1Payload->gyroData[2]));
#endif
                        if(!memcmp(pData1Payload->gpsData,"GPSB",4))
                        {
                            DbgPrintf("gpsDataLength= %d \n", pPack->payloadLength-40);
                            // DbgPrintf("gpsBeginSymbol= %s  ",  pData1Payload->gpsData);
                            for(int j=0;j<pPack->payloadLength-40;j++)
                                DbgPrintf("gpsData= %1x \n",  pData1Payload->gpsData[j]);
                        }
                        else
                        {
                            DbgPrintf(" \n",  pData1Payload->gpsData);
                        }
                        int len;
                        
#ifndef GPS_ONLY                        
                            len= sprintf(write_buff,"%s%ld\n","timer=",pData1Payload->timer);
                            fwrite(write_buff,sizeof(char),len,f0);
                            len= sprintf(write_buff,"%s%f\n","accData0=",exchange_data(&pData1Payload->accData[0]));
                            fwrite(write_buff,sizeof(char),len,f0);
                            len= sprintf(write_buff,"%s%f\n","accData1=",exchange_data(&pData1Payload->accData[1]));
                            fwrite(write_buff,sizeof(char),len,f0);
                            len= sprintf(write_buff,"%s%f\n","accData2=",exchange_data(&pData1Payload->accData[2]));
                            fwrite(write_buff,sizeof(char),len,f0);

                            len= sprintf(write_buff,"%s%f\n","gyroData0=",exchange_data(&pData1Payload->gyroData[0]));
                            fwrite(write_buff,sizeof(char),len,f0);
                            len= sprintf(write_buff,"%s%f\n","gyroData1=",exchange_data(&pData1Payload->gyroData[1]));
                            fwrite(write_buff,sizeof(char),len,f0);
                            len= sprintf(write_buff,"%s%f\n","gyroData2=",exchange_data(&pData1Payload->gyroData[2]));
                            fwrite(write_buff,sizeof(char),len,f0);
#endif
                            if(!memcmp(pData1Payload->gpsData,"GPSB",4))
                            {
                                // len= sprintf(write_buff,"%s, \n", pData1Payload->gpsData);
                                len=sprintf(write_buff,"%s","gps_data=");
                                fwrite(write_buff,sizeof(char),len,f0);
                                 for(int j=0;j<pPack->payloadLength-40-4;j++)   //дgps����
                                 {
                                    len=sprintf(write_buff,"%02x ", pData1Payload->gpsData[j+4]);
                                    fwrite(write_buff,sizeof(char),len,f0);
                                 }
                                 len=sprintf(write_buff,"\n");
                                 fwrite(write_buff,sizeof(char),len,f0);
                            }
                            else
                            {
                                //len= sprintf(write_buff,"%s, \n","NULL");
								len= sprintf(write_buff,"gps_data=null \n");
                                fwrite(write_buff,sizeof(char),len,f0);
                            }
                        

                    }
       
                   
                    fclose(f0);//关闭文件

  
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
            //     DbgPrintf("read frame =%d \n",valid_len);
            //          for(i=0; i<valid_len; i++)
            //         {
            //             DbgPrintf("[%x]",frameBuff[read_index+i]);
            //         }
            //         DbgPrintf("\n");
            //     }
            //     else
            //     {
            //         /* code */
            //     }
                
            // }
            
            usleep(5*1000);
    }//END_while
}


static void get_time_crate_file(char* floder,char *str, int len,char *file_format)
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
 	DbgPrintf("year=%d,mouth=%02d,day=%02d\n", year, month, day);   
	//snprintf(str, len, "%s_%d-%02d-%02d-%02d_%02d_%02d%s", "RTK",year, month, day, hour, min, sec,file_format);
	//snprintf(str, len, "%s%s_%d-%02d-%02d-%02d_%02d_%02d%s", floder,"RTK",year, month, day, hour, min, sec,file_format);
    snprintf(str, len, "%s%s_%d-%02d-%02d-%02d_%02d", floder,"RTK",year, month, day, hour, min);
    DbgPrintf("file_name = %s\n",str);
}



static int check_usb_is_mount(void)
{
	int state = system("mount|grep /media/pi/");
	if (state == 0)
	{
		 return 1;
	}
	else
	{
		return 0;
	}  
}


static int check_is_mount(char* node)
{
    char cmd[50];
    sprintf(cmd,"%s%s","mount|grep ",node);
	int state = system(cmd);
	if (state == 0)
	{
		 return 1;
	}
	else
	{
		return 0;
	}  
}


static void read_mount_stat()
{
	struct mntent *m;  
    FILE *f = NULL;  
	DbgPrintf("ls = %s\n",system("ls /media/pi"));
    //f = setmntent("/media/pi","r"); //open file for describing the mounted filesystems  
	f = setmntent("/dev/sda1","r"); 
	DbgPrintf("test2\n");
	
	
	m = getmntent(f);
	DbgPrintf("Drive %s, name %s,type  %s,opt  %s\n", m->mnt_dir, m->mnt_fsname,m->mnt_type,m->mnt_opts ); 
	endmntent(f);   //close file for describing the mounted filesystems  
/*	
    if(!f)  
       DbgPrintf("error:%s\n",strerror(errno));  
    while ((m = getmntent(f)))        //read next line  
    DbgPrintf("Drive %s, name %s,type  %s,opt  %s\n", m->mnt_dir, m->mnt_fsname,m->mnt_type,m->mnt_opts );  
    endmntent(f);   //close file for describing the mounted filesystems  
*/	
    return 0;  
}







int read_mount_node(char *basePath,char* node_path)
{
    DIR *dir;
    int rec = -1;
    struct dirent *ptr;
    char base[1000];
    
    if ((dir=opendir(basePath)) == NULL)
    {
        perror("Open dir error...");
        return rec;
    }
 
    while ((ptr=readdir(dir)) != NULL)
    {
        if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)    
            continue;
        else if(ptr->d_type == 8)    ///file
            DbgPrintf("d_name:%s/%s\n",basePath,ptr->d_name);
        else if(ptr->d_type == 10)    ///link file
            DbgPrintf("d_name:%s/%s\n",basePath,ptr->d_name);
        else if(ptr->d_type == 4)    ///dir                 //目录
        {
            if(check_is_mount(ptr->d_name) == 1)
            {
		        strcpy(node_path,ptr->d_name);
		        DbgPrintf("d_name:%s/%s\n",basePath,ptr->d_name);
                rec = 1;
            }
            else
            {
                DbgPrintf("not mount\n");
            }
            
             //memset(base,'\0',sizeof(base));
             //strcpy(base,basePath);
             //lsstrcat(base,"/");
             //strcat(base,ptr->d_name);
             //readFileList(base);
        }
    }
    closedir(dir);
    return rec;
}



void test()
{
	FILE * fp;
    char gps_data[200] = {0};
	DbgPrintf("test\n");
	fp = fopen("RTK_2019-06-03-11_48_41", "r");
	if (fp == NULL)
	{
		DbgPrintf("no file\n");
	}
    for(int i = 0;i < 10;i++)
    {
        int timer = read_timer_value("timer", fp);
        DbgPrintf("timer = %d\n",timer);
        float accData0 = read_ins_value("accData0", fp);
        float accData1 = read_ins_value("accData1", fp);
        float accData2 = read_ins_value("accData2", fp);
        DbgPrintf("accData0 = %f\n",accData0);
        DbgPrintf("accData1 = %f\n",accData1);
        DbgPrintf("accData2 = %f\n",accData2);

        float gyroData0 = read_ins_value("gyroData0", fp);
        float gyroData1 = read_ins_value("gyroData1", fp);
        float gyroData2 = read_ins_value("gyroData2", fp);
        DbgPrintf("gyroData0 = %f\n",gyroData0);
        DbgPrintf("gyroData1 = %f\n",gyroData1);
        DbgPrintf("gyroData2 = %f\n",gyroData2);
        read_gps_data("gps_data", fp,gps_data);
        DbgPrintf("gps = %s\n",gps_data);
        DbgPrintf("\n next \n\n");
    }
    fclose(fp);
}


 

