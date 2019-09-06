#ifndef _MAIN_H_
#define _MAIN_H_
#include <stdio.h>
#include "rtcm.h"

void safe_free(void *ptr);
#define DbgPrintf printf
//#define DbgPrinitf //
int read_rtk_data(FILE *fp, rtcm_t *rtcm, obs_t *obs, nav_t *nav, int *numofread);


#endif