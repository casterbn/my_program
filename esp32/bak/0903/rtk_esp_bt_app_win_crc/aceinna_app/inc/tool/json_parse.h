#ifndef _JSON_PARSE_H_
#define _JSON_PARSE_H_

int json_create();
void create_objects(char* json_to_str);
int doit(char *text);
int change_item(char *text,char* json_to_write,char* key,char* value);
#define json_true 1
#define json_false -1

#endif

