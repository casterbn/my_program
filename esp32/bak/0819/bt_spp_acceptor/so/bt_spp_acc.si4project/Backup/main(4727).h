/*******************************************************************************
* File Name          : rtk_bt.h
* Author             : Daich
* Revision           : 1.0
* Date               : 13/05/2019
* Description        : bluetooth
*
* HISTORY***********************************************************************
* 13/05/2019  |                                             | Daich
*
*******************************************************************************/
#ifndef _MAIN_H
#define _MAIN_H
#include "freertos/semphr.h"
void take_bt_mutex(void);
void release_bt_mutex(void);


#endif
