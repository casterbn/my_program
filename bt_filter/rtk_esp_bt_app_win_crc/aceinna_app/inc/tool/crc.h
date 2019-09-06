#ifndef _CRC_H
#define _CRC_H

typedef unsigned int DWords;
typedef unsigned char Byte;
typedef unsigned char * BPoint;
typedef unsigned int uint;

unsigned short CRC16_Direct(int len,BPoint ptr, unsigned short CRC_Init);
unsigned short CRC16_DirectTable(int len,BPoint ptr, unsigned short CRC_Init);
void CRC_Table_Init();

#endif
