#ifndef _TOOL_H
#define _TOOL_H

#define Dbgprintf printf
//#define Dbgprintf //

int array_cmp(void* src1,void* src2,int arr_len,int type_len);
#if	1
void sleep_s(int idle_s);
#endif

#endif
