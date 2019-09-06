
#include <stdio.h>
#include "crc.h"

#define TABLE_SIZE 256
//#define CRC_POLY 0x04C11DB7  //CRC多项式
#define CRC_POLY 0x1021
//#define CRC_POLY 0xedb88320
#define LUT_LENGTH 8
#define CRC_LENGTH 32
#define DATA_SIZE 10



unsigned short CRC_TABLE[TABLE_SIZE]={0};
int RefCRC_TABLE[TABLE_SIZE]={0};
unsigned char Serial_Data[DATA_SIZE]={18,51,63,47,15,132,94,87,56,40};
unsigned char RefSerial_Data[DATA_SIZE];
static int crc_table_init_flag;

static unsigned short ccitt_table[256] = {
0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

#if 0
short CRC32_Direct(int len,BPoint ptr, int CRC_Init)
{
	int crc32;
	int i;
	crc32 = CRC_Init;
	while(len--!=0)
	{
		crc32 ^= ((*ptr++)<<24);
		//crc32 ^= ((*ptr++)<<8);
		for(i=0; i<8;i++)                //crc共移位32，刚好补了32位0进来
		{   
			if(crc32&0x80000000)
			//if(crc32&0x8000)
			{   
				crc32<<= 1;   
				crc32 ^=CRC_POLY;            
			}   
			else
				crc32 <<= 1;    
		}     
	}  
	return crc32;
}


int CRC32_DirectTable(int len,BPoint ptr, int CRC_Init)
{
	int CRC_Res = 0x00000000;
	int i;
	unsigned char CByte;
	for(i = 0; i < 4; i++)
	{
		CByte = (CRC_Init >> 24) & 0xff;
		CRC_Res = ((CRC_Res << 8)|(*ptr++^CByte))^CRC_TABLE[(CRC_Res >> 24) & 0xff];
		CRC_Init <<= 8;
	}
	len = len - 4;
	while(len--)
	{
		CRC_Res = ((CRC_Res << 8 ) | * ptr++) ^ CRC_TABLE[(CRC_Res >> 24) & 0xff];
	}
	for(i = 0; i < 4; i++)
	{
		CRC_Res = (CRC_Res << 8) ^ CRC_TABLE[(CRC_Res >> 24) & 0xff];
	}
	return CRC_Res;
}
#endif

unsigned short CRC16_Direct(int len,BPoint ptr, unsigned short CRC_Init)
{
	unsigned short crc16;
	int i;
	crc16 = CRC_Init;
	printf("crc16 = %d\n",crc16);
	while(len--!=0)
	{
		crc16 ^= ((*ptr++)<<8);
		//printf("crc32——0 = %x\n",crc16);
		for(i=0; i<8;i++)                //crc共移位16，刚好补了12位0进来
		{   
			if(crc16&0x8000)
			{   
				crc16<<= 1;   
				crc16 ^=CRC_POLY;            
				//printf("crc32____ = %x\n",crc16);
			}   
			else
				crc16 <<= 1;    
		}     
		//printf("crc16 = %x\n",crc16);
	}  
	
	//printf("crc16************** = %x\n",crc16);
	return crc16;
}
#if 0
unsigned short crc_ccitt(unsigned char *q, int len)
{
	unsigned short crc = 0;

	while (len-- > 0)
	crc = CRC_TABLE[(crc >> 8 ^ *q++) & 0xff] ^ (crc << 8);
	return crc;
}
#endif

unsigned short CRC16_DirectTable(int len,BPoint ptr, unsigned short CRC_Init)
{
	unsigned short CRC_Res = 0x0;
	int i;
	unsigned char CByte;
	while(len--)
	{
		CRC_Res = (CRC_Res << 8 ) ^ CRC_TABLE[(CRC_Res >> 8 ^ *ptr++) & 0xff];
	}
	return CRC_Res;
}




void CRC_Table_Init()	//生成CRC表
{
	uint i, j; 
	unsigned short nData16;  
	unsigned short CRC_Reg; 
	for ( i = 0; i < 256; i++ )  
	{ 
		nData16 = ( uint )( i << 8 );
		CRC_Reg = 0;   
		for ( j = 0; j < 8; j++ ) 
		{
			if ( ( nData16 ^ CRC_Reg ) & 0x8000 ) 
				CRC_Reg = ( CRC_Reg << 1 ) ^ CRC_POLY; 
			else                                       
				CRC_Reg <<= 1;                              
			nData16 <<= 1; 
		}
		CRC_TABLE[i] = CRC_Reg;	//crc表
	}
}

#if 0
int crc_test()
{  
	uint i, len;
	BPoint BP;
	unsigned short CRC_Res = 0x00;
	int CRC_Init = 0x00;	//CRC初始值
	Table_Init();
	RefTable_Init();
#ifdef REFDATA	
	RefData_Init();
#endif
	printf("Test CRC:\n");
	for(i = 0; i < 2; i++)
	{
		//CRC_Init = (i << 24) + (i <<16) + (i << 8) + i;
		printf("CRC Init Res; %x\n",CRC_Init);
		BP = Serial_Data;
		len = DATA_SIZE;
		CRC_Res = CRC32_Direct(len,BP,CRC_Init);
		printf("after direct crc  the result is： %x\n",CRC_Res);
		//Direct CRC TABLE
		CRC_Res = CRC32_DirectTable(len, BP, CRC_Init);
		printf("after direct-table the crc result is： %x\n",CRC_Res);

		//Drive CRC_TABLE
		CRC_Res = CRC32_DriveTable(len, BP, CRC_Init);
		printf("after drive-table the crc result is： %x\n\n",CRC_Res);
		
		CRC_Res = crc_ccitt(BP,len);
		
		printf("***************************： %x\n\n",CRC_Res);

	}
}
#endif
