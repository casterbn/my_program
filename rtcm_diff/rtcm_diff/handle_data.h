#ifndef _HANDLE_DATA_H_
#define _HANDLE_DATA_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <sys/types.h>
#include <time.h>
#include <sys/stat.h>
#include <stdarg.h>
#include <limits.h>

typedef struct item_t
{
    char *key;
    char *value;
} ITEM;

float read_conf_value(const char *key,  FILE *fp);
int write_conf_value(const char *key, float value, FILE *fp);

FILE * read_conf_pre(const char *file);
int read_conf_ok(FILE *fp);

int write_conf_ok(FILE *fp);
FILE * write_conf_pre(const char *file);
extern char *strtrimr(char *pstr);

extern char *strtrim(char *pstr);

int read_gps_data(const char *key,  FILE *fp,unsigned char** gps_data,int* len);
float read_ins_value(const char *key,  FILE *fp) ;
long read_timer_value(const char *key,  FILE *fp);
int line_read(FILE * fp, long line,int last_offset,unsigned char** gps_data,int* data_len);
void handle_gps_string(FILE * fp, char* pstr, int last_offset, unsigned char** gps_data, int* data_len);
void listFiles(const char * dir);


#ifdef __cplusplus
}
#endif

#endif //end of FILE_H_
