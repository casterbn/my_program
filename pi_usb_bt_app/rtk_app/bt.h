#ifndef _BT_H_
#define _BT_H_

#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>

#include <bluetooth/bluetooth.h>   //蓝牙的3个头文件.
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>

#include <stdio.h>
#include <unistd.h>
#include <bluetooth/rfcomm.h>
#include <bluetooth/l2cap.h>

int bt_config();
void *monitor_bt_pthread(void *arg);
#endif
