#ifndef __CTR_H_
#define __CTR_H_

void *rtk_key_input_monitor_thread(void *arg);
void *rtk_uart_input_monitor_thread(void *arg);
int rtk_input_monitor_thread_create(void);
int list_files(const char * dir_base);
char* get_handle_file();

#endif
