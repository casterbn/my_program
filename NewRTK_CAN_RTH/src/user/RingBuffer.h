// RingBuffer.h writed by jacksun 2017.11.22
#include <stdint.h>
typedef unsigned char u8;
typedef unsigned short u16;
// typedef unsigned int u32;

typedef signed char s8;
typedef signed short s16;
// typedef signed int s32;

typedef u8 ELEMENT_TYPE;
typedef struct
{
    ELEMENT_TYPE *pRing_buf;
    u16            length;
    u16            write_index;
    u16            read_index;
    
}RingBuffer;

typedef struct
 {
	uint8_t* buffer;	//FIFO数据
	uint16_t in;			//入口下标
	uint16_t out;			//出口下标
	uint16_t size;		//FIFO大小
}FIFO_Type;

#define GPS_BUFF_SIZE (2000)
#define BT_BUFF_SIZE (512)
#define IMU_BUFF_SIZE 512

extern FIFO_Type Uart3RxFifo;
extern FIFO_Type Uart5RxFifo;
extern FIFO_Type Uart2RxFifo;
extern FIFO_Type Uart1RxFifo;

int InitRingBuffer(RingBuffer *pRing,ELEMENT_TYPE * buff,u16 len);
int ReadRingBuffer(RingBuffer *pRing,ELEMENT_TYPE *pReadbuf,u16 rd_len);
int ReadRecentDataInRingBuffer(RingBuffer *pRing,ELEMENT_TYPE *pReadbuf,u16 rd_len);
int WriteRingBuffer(RingBuffer *pRing,ELEMENT_TYPE *pWrbuf,u16 wr_len);
int ReadAllDataNoRead(RingBuffer *pRing,ELEMENT_TYPE *pReadbuf);
int ReadAllDataNoReadLength(RingBuffer *pRing);
int WriteOneElementRingBuffer(RingBuffer *pRing,ELEMENT_TYPE element);
u16 GetLastWriteIndex(RingBuffer *pRing);
void FifoInit(FIFO_Type* fifo, uint8_t* buffer, uint16_t size);
uint16_t FifoGet(FIFO_Type* fifo, uint8_t* buffer, uint16_t len);
uint16_t FifoStatus(FIFO_Type* fifo);





