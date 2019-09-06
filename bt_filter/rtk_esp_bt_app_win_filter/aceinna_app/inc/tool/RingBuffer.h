// RingBuffer.h writed by jacksun 2017.11.22
#ifndef _RingBuffer_H
#define _RingBuffer_H

#define RX_SIZE     4096

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;

typedef signed char s8;
typedef signed short s16;
typedef signed int s32;

typedef u8 ELEMENT_TYPE;
typedef struct
{
    ELEMENT_TYPE *pRing_buf;
    u16            length;
    u16            write_index;
    u16            read_index;
    
}RingBuffer;
int InitRingBuffer(RingBuffer *pRing,ELEMENT_TYPE * buff,u16 len);
int ReadRingBuffer(RingBuffer *pRing,ELEMENT_TYPE *pReadbuf,u16 rd_len);
int ReadRecentDataInRingBuffer(RingBuffer *pRing,ELEMENT_TYPE *pReadbuf,u16 rd_len);
int WriteRingBuffer(RingBuffer *pRing,ELEMENT_TYPE *pWrbuf,u16 wr_len);
int ReadAllDataNoDeel(RingBuffer *pRing,ELEMENT_TYPE *pReadbuf);
int WriteOneElementRingBuffer(RingBuffer *pRing,ELEMENT_TYPE element);
u16 GetLastWriteIndex(RingBuffer *pRing);
int GetDataLeng(RingBuffer *pRing);

extern void getTimeString(char *str, int len,char *file_format);



#endif
