#include "handle_data.h"







/*
 *去除字符串右端空格
 */
char *strtrimr(char *pstr)
{
    int i;
    i = strlen(pstr) - 1;
    while(isspace(pstr[i]) && (i >= 0))
    { pstr[i--] = '\0'; }
    return pstr;
}
/*
 *去除字符串左端空格
 */
char *strtriml(char *pstr)
{
    int i = 0, j;
    j = strlen(pstr) - 1;
    while(isspace(pstr[i]) && (i <= j))
    { i++; }
    if(0 < i)
    { strcpy(pstr, &pstr[i]); }
    return pstr;
}
/*
 *去除字符串两端空格
 */
char *strtrim(char *pstr)
{
    char *p;
    p = strtrimr(pstr);
    return strtriml(p);
}


/*
 *从配置文件的一行读出key或value,返回item指针
 *line--从配置文件读出的一行
 */
int  get_item_from_line(char *line,  ITEM *item)  
{
    char *p = strtrim(line);
    int len = strlen(p);
    if(len <= 0)
    {
        return 1;//空行
    }
    else if(p[0] == '#')
    {
        return 2;
    }
    else
    {
        char *p2 = strchr(p, '=');
//		DbgPrintf("key = %s\n",p2);
//		DbgPrintf("key = %c\n",*p2);
        *p2++ = '\0';                //key��ֹ
       // p2++;
//		DbgPrintf("key = %c\n",*p2);

        item->key = (char *)malloc(strlen(p) + 1);
        item->value = (char *)malloc(strlen(p2) + 1);

        strcpy(item->key, p);
        strcpy(item->value, p2);
//		DbgPrintf("key = %s",item->key);
//		DbgPrintf("value = %s",item->value);

    }
    return 0;//查询成功
}

int file_to_items(const char *file,  ITEM *items,  int *num)
{
    char line[1024];
    FILE *fp;
    fp = fopen(file, "r");
    if(fp == NULL)
    { return 1; }
    int i = 0;
    while(fgets(line, 1023, fp))
    {
        char *p = strtrim(line);
        int len = strlen(p);
        if(len <= 0)
        {
            continue;
        }
        else if(p[0] == '#')
        {
            continue;
        }
        else
        {
            char *p2 = strchr(p, '=');
            /*这里认为只有key没什么意义*/
            if(p2 == NULL)
            { continue; }
            *p2++ = '\0';
            items[i].key = (char *)malloc(strlen(p) + 1);
            items[i].value = (char *)malloc(strlen(p2) + 1);
            strcpy(items[i].key, p);
            strcpy(items[i].value, p2);

            i++;
        }
    }
    (*num) = i;
    fclose(fp);
    return 0;
}

FILE *read_file_pre(const char *file)
{
    FILE *fp =NULL ;
    fp = fopen(file, "r");
    return fp;

}
int read_file_ok(FILE *fp)
{
    fclose(fp);
    fp= NULL;
}

float read_conf_value(const char *key,  FILE *fp)    
{
    char line[1024];
    float value = 0;
    if(fp == NULL)
    {
        return -1;
    }
    fgets(line, 1023, fp);
    {
        ITEM item;

        get_item_from_line(line, &item);
		
		DbgPrintf("item.key = %s\n",item.key);
		DbgPrintf("item.value = %s\n",item.value);
		DbgPrintf("key = %s\n",key);
        if(strcmp(item.key, key) == 0)
        {
			DbgPrintf("match\n");
            value =atof(item.value);
            free(item.key);
            free(item.value);
            //break;
        }
        else
        {
            DbgPrintf("not match\n");
        }
    }
    return value;
}

FILE *write_conf_pre(const char *file)
{
    FILE *fp;
    fp = fopen(file, "w");
    return fp;

}
int write_conf_value(const char *key, float value, FILE *fp)
{
    char a[30] = {'\0'};

    sprintf(a, "%.4f", value);
    fprintf(fp, "%s=%s\n", key, a);
    return 0;
}

int write_conf_ok(FILE *fp)
{
    fclose(fp);
}


void main() //RTK_2019-06-03-11_48_41
{
	FILE * fp;
	DbgPrintf("test\n");
	fp = fopen("RTK_2019-06-03-10_32_59", "r");
	if (fp == NULL)
	{
		DbgPrintf("no file\n");
	}
	DbgPrintf("test2\n");
	float timer =(char)read_conf_value("timer", fp);
	DbgPrintf("timer = %f\n",timer);
	DbgPrintf("test3\n");
	float accData0 = (char)read_conf_value("accData0", fp);
	
	DbgPrintf("accData0 = %f\n",accData0);
	fclose(fp);
}