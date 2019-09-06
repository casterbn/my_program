// rtk.h 
#include <stdint.h>
#include "RingBuffer.h"
#include <string.h>


#define RX_SIZE     8192
#define YES         1
#define NO          0

#define UCB_MAX_PAYLOAD_LENGTH		400
#define UCB_USER_IN                 200         
#define UCB_USER_OUT                201      
#define UCB_ERROR_INVALID_TYPE      202  

typedef struct 
{
    uint32_t timer;
    float accData[3];
    float gyroData[3];
    float magData[3];
    u8 gpsData[400];
} Data1Payload_t;

#pragma pack(1)
typedef struct
{
     uint8_t       sync_MSB;        // 3
     uint8_t       sync_LSB;        // 4
     uint8_t       code_MSB;        // 5
     uint8_t       code_LSB;        // 6
     uint16_t	   payloadLength;   // 7
     uint8_t       payload[UCB_MAX_PAYLOAD_LENGTH + 2]; // aligned to 4 bytes 
} UcbPacketStruct;
#pragma pack()




int findHead(u8 *inBuf,int inCnt,int * parseIndex);
int getCompleteFrame(u8 *inBuf,int inCnt,u8 *outBuf,int *destCnt,int *readStatus);
int findFrame(u8 *inBuf,int inCnt,int * parseIndex,int * num);
