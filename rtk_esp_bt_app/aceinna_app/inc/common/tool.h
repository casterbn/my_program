#ifndef _TOOL_H
#define _TOOL_H

#if 0
//#define DbgPrintf printf
#define DbgPrintf   //
#endif

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

#endif
