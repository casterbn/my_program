// RingBuffer.c needed by park_algo_jack.c  writed by jacksun 2017.11.12
#include "RingBuffer.h"
#include <string.h>
#include "main.h"
int InitRingBuffer(RingBuffer *pRing,ELEMENT_TYPE * buff,u16 len)
{
    memset(pRing->pRing_buf,0,sizeof(ELEMENT_TYPE) * pRing->length);
    pRing->pRing_buf    =buff;
    pRing->write_index  =0;
    pRing->read_index   =0;
    pRing->length       =len;
    return 0;
}

int ReadRingBuffer(RingBuffer *pRing,ELEMENT_TYPE *pReadbuf,u16 rd_len)
{
    if(rd_len > pRing->length)
        return -1;
    
    
    if((pRing->read_index+rd_len) >= pRing->length)
    {
        memcpy(pReadbuf,&pRing->pRing_buf[pRing->read_index],sizeof(ELEMENT_TYPE) * (pRing->length - pRing->read_index));
        memcpy(&pReadbuf[pRing->length - pRing->read_index],&pRing->pRing_buf[0],sizeof(ELEMENT_TYPE) * (rd_len - (pRing->length - pRing->read_index)) );
        
        pRing->read_index = ((pRing->read_index+rd_len )- pRing->length)%pRing->length ;
    }
    else
    {
        memcpy(pReadbuf,&pRing->pRing_buf[pRing->read_index],sizeof(ELEMENT_TYPE) * rd_len);
        pRing->read_index= (pRing->read_index+rd_len)%pRing->length;
    }
    return 0;
}
int ReadRecentDataInRingBuffer(RingBuffer *pRing,ELEMENT_TYPE *pReadbuf,u16 rd_len)
{

     int num,rd_tail_num=0;

     if(rd_len > pRing->length)
        return -1;
     if(rd_len <= pRing->write_index)
     {
         memcpy(pReadbuf,&pRing->pRing_buf[pRing->write_index-rd_len],sizeof(ELEMENT_TYPE) *rd_len);
     }
     else
     {
        rd_tail_num=rd_len-pRing->write_index;
         
        memcpy(pReadbuf,&pRing->pRing_buf[pRing->length-rd_tail_num],sizeof(ELEMENT_TYPE) * rd_tail_num);
        memcpy(&pReadbuf[rd_tail_num],&pRing->pRing_buf[0],sizeof(ELEMENT_TYPE) * pRing->write_index);         
     }
    
      num = rd_len;

    return num;
}

int ReadAllDataNoDeel(RingBuffer *pRing,ELEMENT_TYPE *pReadbuf)
{
    int num;
    //DbgPrintf("pRing->write_index = %d,pRing->read_index = %d\n",pRing->write_index,pRing->read_index);
    if(pRing->write_index>=pRing->read_index)       //�Ƿ�����һ�ζ�ȡ���ࣿ
    {
        memcpy(pReadbuf,&pRing->pRing_buf[pRing->read_index],sizeof(ELEMENT_TYPE) * (pRing->write_index-pRing->read_index));
        num = pRing->write_index-pRing->read_index;
    }
    else
    {
        memcpy(pReadbuf,&pRing->pRing_buf[pRing->read_index],sizeof(ELEMENT_TYPE) * (pRing->length - pRing->read_index));
        memcpy(&pReadbuf[pRing->length - pRing->read_index],&pRing->pRing_buf[0],sizeof(ELEMENT_TYPE) * pRing->write_index);
        num = pRing->write_index+( pRing->length - pRing->read_index);
    }
    pRing->read_index = pRing->write_index;

    return num;
}

int WriteOneElementRingBuffer(RingBuffer *pRing,ELEMENT_TYPE element)
{
 
  if(pRing->write_index<pRing->length)    
    pRing->pRing_buf[pRing->write_index++]=element;
  else
     pRing->write_index=0; 
    
    return 0;
}

u16 GetLastWriteIndex(RingBuffer *pRing)
{
   u16 index=0;
  if(pRing->write_index==0)    
      index=pRing->length-1;
  else
     index=pRing->write_index-1; 
    
    return index;
    
}


int WriteRingBuffer(RingBuffer *pRing,ELEMENT_TYPE *pWrbuf,u16 wr_len)
{
    if(wr_len>pRing->length)
        return -1;
 
    if(pRing->write_index + wr_len >pRing->length)
    {
       memcpy(&pRing->pRing_buf[pRing->write_index], pWrbuf, sizeof(ELEMENT_TYPE) *(pRing->length - pRing->write_index));
       memcpy(pRing->pRing_buf,&pWrbuf[ pRing->length - pRing->write_index],sizeof(ELEMENT_TYPE) *(wr_len-(pRing->length - pRing->write_index)));
       pRing->write_index=((pRing->write_index + wr_len) -pRing->length)%pRing->length;
    }
    else
    {
       memcpy(&pRing->pRing_buf[pRing->write_index], pWrbuf, sizeof(ELEMENT_TYPE) *wr_len); 
       pRing->write_index = (pRing->write_index + wr_len)%pRing->length;
    }   
    
    return 0;
}
