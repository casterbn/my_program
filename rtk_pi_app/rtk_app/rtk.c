#include "rtk.h"

u8 FrameAarray[500];
//if findhead, parseIndex= headindex,else paseIndex= inCnt-3;
int findHead(u8 *inBuf,int inCnt,int * parseIndex)
{
    int i=0;
    while(i <= inCnt-4)
    {
        if(inBuf[i] == 0x55 && inBuf[i+1] == 0x55 && inBuf[i+2] == 0x7a && inBuf[i+3] == 0x31)   //find header
        {
            *parseIndex= i;
            return YES;
        }
        i++;
    }
    // *parseIndex= inCnt-3;
    return NO;

}

//inCnt待处理数据量
//inBuf待处理buff
//parseIndex 存放各buff头的数组
//num表示共有多少组有效buff
int findFrame(u8 *inBuf,int inCnt,int * parseIndex,int * num)  
{
    int i=0,j=0;
    int index=0;
    while(index<inCnt)
    {
            if(findHead(&inBuf[index],inCnt-index,&i))   //i 为帧头位置
            {
                index+=i;
                parseIndex[j++] = index;
            }
            index++;

    }
    *num=j;
    return 0;

}

int getPayloadLength(u8 *inBuf,int inCnt,int * parseIndex)
{
    int i=0;
    while(i <= inCnt-4)
    {
        if(inBuf[i] == 0x55 && inBuf[i+1] == 0x55 && inBuf[i+2] == 0x7a && inBuf[i+3] == 0x31)   //find header
        {
            *parseIndex= i;
            return YES;
        }
        i++;
    }
    *parseIndex = inCnt-3;
    return NO;
}

int getCompleteFrame(u8 *inBuf,int inCnt,u8 *outBuf,int *destCnt,int *readStatus)
{
    int i=0,j=0;
    int rest_len=0;
    int pack_len=0;
    int head_index=0;
    static u8 FrameBegin=0;
 
        if(findHead(inBuf,inCnt,&head_index))//header
        {
            FrameBegin=1;      
        }

        rest_len-=2;                                                        //removed payload length filed
            if(rest_len<pack_len)                                               //if rest size  less than package size
        
        if(*readStatus == 1)//body
        {
            outBuf[(*destCnt)++] = inBuf[i];    
        }
        if(*destCnt == ((((int)outBuf[4]<<8)) + outBuf[5]))//tail
        {
            print_frame("tail",outBuf,*destCnt);
            *readStatus = 0;
            *destCnt = 0;
            memset(outBuf,-1,sizeof(outBuf));
            memset(inBuf,0,sizeof(inBuf));
            // continue;
        }
    
}
