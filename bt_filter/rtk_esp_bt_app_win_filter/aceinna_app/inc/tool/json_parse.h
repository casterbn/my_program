#ifndef _JSON_PARSE_H_
#define _JSON_PARSE_H_
#include "cJSON.h"


int json_create();
void esp32_json_only(char* json_to_str);
int add_esp32_json(char *text);
int change_item(char *text,char* json_to_write,char* key,char* value);
#define json_true 1
#define json_false -1
void add_esp32_json_message(cJSON *json);

#endif

