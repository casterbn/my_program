#ifndef _COM_H_
#define _COM_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>

int set_com_config(int fd,int baud_rate,int data_bits, char parity, int stop_bits);
int open_port(int com_port);


#endif
