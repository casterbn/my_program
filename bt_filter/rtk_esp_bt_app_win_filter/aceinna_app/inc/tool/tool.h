#ifndef _TOOL_H
#define _TOOL_H

#if 0
//#define DbgPrintf printf
#define DbgPrintf   //
#endif

//#define RTCM_FILTER

#ifdef DEBUG
        #define DbgPrintf(format,args...) \
        fprintf(stderr, format, ##args)
#else
        #define DbgPrintf(format,args...)
#endif

int array_cmp(void* src1,void* src2,int arr_len,int type_len);
#if	1
void sleep_s(int idle_s);
#endif

char strtrimall(unsigned char *pstr,uint32_t len);

typedef struct {        /* RTCM control struct type */
    //gtime_t time_s;     /* message start time */
    //obs_t obs;          /* observation data (uncorrected) */
    char msmtype[6][128]; /* msm signal types */
    /* YYD: need to save space for this */
    /* YYD: cp is only used in RTCM2 (adjcp), lock is temp disabled */
    //double cp[MAXSAT][NFREQ+NEXOBS]; /* carrier-phase measurement */
    //unsigned short lock[MAXSAT][NFREQ+NEXOBS]; /* lock time */
    //unsigned short loss[MAXSAT][NFREQ+NEXOBS]; /* loss of lock count */
    //gtime_t lltime[MAXSAT][NFREQ+NEXOBS]; /* last lock time */
    unsigned int nbyte;          /* number of bytes in message buffer */ 
    unsigned int nbit;           /* number of bits in word buffer */ 
    unsigned int len;            /* message length (bytes) */
    unsigned int len_to_send;            /* message length (bytes) */    
    unsigned int type; /* last rtcm type */
    unsigned char buff[1200]; /* message buffer */
	unsigned char key;
	unsigned char last_nmea_ts;
} rtcm_t;

int input_rtcm3_data(rtcm_t *rtcm, unsigned char data);
void Hex2Str(char *sSrc,  char *sDest, int nSrcLen);
#endif
