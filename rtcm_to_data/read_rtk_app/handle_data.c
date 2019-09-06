#include "handle_data.h"
#include "main.h"

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
 *去除字符串空格
 */
char *strtrimall(char *pstr)  //p ' ' q vallue
{
    char cmp[2048];
    int j = 0;
    int len1 = strlen(pstr);
    //int len = 0;
    for(int i = 0;i < strlen(pstr) - 1;i++)
    {
        if(*(pstr + i) != ' ')
        {
            cmp[j++] = *(pstr + i);
        }
    }
    cmp[j] = '\0';
    strncpy(pstr,cmp,j);
    int len2 = strlen(pstr);
    int count = len1 - len2;
    printf("1111 pstr = %s",pstr);
    return count;
#if 0
	while (*pstr != ' ')
	{
		if (*pstr == '\0')
		{
			return;
		}
		pstr++;
	}
    int len1 = strlen(pstr);
    printf("len1 = %d\n",len1);
    char *p,*q;
    p = pstr;
    while(*p!=' ') p++;
    q = p;
    while(1)
    {
        while(*q == ' '||*q != '\0') q++;
        if(*q == '\0')
        {
            *p = *q;
            break;
        }
        else
        {
            *p = *q;
            *q = ' ';
            q++;
            p++;
        }
    }
    int len2 = strlen(pstr);
    int count = len1 - len2;
    printf("1111 pstr = %s",pstr);
    return count;
#endif
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
 *从文件的一行读出key或value,返回item指针
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
        *p2++ = '\0';
        item->key = (char *)malloc(strlen(p) + 1);
        item->value = (char *)malloc(strlen(p2) + 1);
        strcpy(item->key, p);
        strcpy(item->value, p2);

    }
    return 0;//查询成功
}


int file_to_items(const char *file,  ITEM *items,  int *num)
{
    char line[2048];
    FILE *fp;
    fp = fopen(file, "r");
    if(fp == NULL)
    { return 1; }
    int i = 0;
    while(fgets(line, 2047, fp))
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

FILE *read_conf_pre(const char *file)
{
    FILE *fp =NULL ;
    fp = fopen(file, "r");
    return fp;

}
int read_conf_ok(FILE *fp)
{
    fclose(fp);
    fp= NULL;
}

long  read_timer_value(const char *key,  FILE *fp)    
{
    char line[2048];
    int value = 0;
    if(fp == NULL)
    {
        return -1;
    }
    //fgets(line, 2047, fp);
    if(fgets(line, 2047, fp) != NULL)
    {
        ITEM item;

        get_item_from_line(line, &item);
		
		//printf("item.key = %s\n",item.key);
		//printf("item.value = %s\n",item.value);
		//printf("key = %s\n",key);
        if(strcmp(item.key, key) == 0)
        {
			//printf("match\n");
            value = atoi(item.value);
            //printf("value = %lf\n",value);
            safe_free(item.key);
            safe_free(item.value);
            //break;
        }
        else
        {
            //printf("not match\n");
        }
    }
    else
    {
        printf("time file end\n");
    }
    
    return value;
}


float read_ins_value(const char *key,  FILE *fp)    
{
    char line[2048];
    float value = 0;
    int rec = -1;
    if(fp == NULL)
    {
        return -1;
    }
    if(fgets(line, 2047, fp) != NULL)
    {
        ITEM item;

        get_item_from_line(line, &item);
		
		//printf("item.key = %s\n",item.key);
		//printf("item.value = %s\n",item.value);
		//printf("key = %s\n",key);
        if(strcmp(item.key, key) == 0)
        {
            value = atof(item.value);
            safe_free(item.key);
            safe_free(item.value);
            //break;
        }
        else
        {
            printf("not match\n");
        }
    }
    else
    {
        printf("gns file end\n");
    }
    
    return value;
}




int StringToHex(char *str, unsigned char *out, unsigned int *outlen)
{
    char *p = str;
    char high = 0, low = 0;
    int tmplen = strlen(p), cnt = 0;
    tmplen = strlen(p);
    while(cnt < (tmplen / 2))
    {
        high = ((*p > '9') && ((*p <= 'F') || (*p <= 'f'))) ? *p - 48 - 7 : *p - 48;
		low = (*(++ p) > '9' && ((*p <= 'F') || (*p <= 'f'))) ? *(p) - 48 - 7 : *(p) - 48;
        out[cnt] = ((high & 0x0f) << 4 | (low & 0x0f));
        p ++;
        cnt ++;
    }
    if(tmplen % 2 != 0) out[cnt] = ((*p > '9') && ((*p <= 'F') || (*p <= 'f'))) ? *p - 48 - 7 : *p - 48;
    if(outlen != NULL) *outlen = tmplen / 2 + tmplen % 2;
    return tmplen / 2 + tmplen % 2;
}


int read_gps_data(const char *key,  FILE *fp, unsigned char** gps_data,int* len)  
{
    char line[2048];
    float value = 0;
    char* value_trim;
    //int valid_len = 0;
    unsigned char data[4096] = {0};
    if(fp == NULL)
    {
        return -1;
    }
    //fgets(line, 2047, fp);
    if(fgets(line, 2047, fp) != NULL)
    {
        ITEM item;
        get_item_from_line(line, &item);
		printf("item.value = %s\n",item.value);
        if(strcmp(item.key, key) == 0)
        {
            int j = 0;
            if(strcmp(item.value,"null") == 0)
            {
                return -1;
            }
            for(int i = 0;i < strlen(item.value) - 1;i++)
            {
                if(*(item.value + i) != ' ')
                {
                    int out_len = 0;
                    char cut[2];
                    strncpy(cut, item.value + i, 2);
                    StringToHex(cut, data + (*len), &out_len);
                    (*len)++;
                    i++;                                    //取两个字符
                }
            }
            //printf("111cur_len = %d\n",*len);
            //printf("valid_len = %d\n",(*len));
            *gps_data = (unsigned char*)malloc((*len) * sizeof(char));
            memcpy(*gps_data,data,(*len));
            safe_free(item.key);
            safe_free(item.value);
            //break;
        }
        else
        {
            printf("not match\n");
        }
    }
    else
    {
        printf("gps file end\n");
    }
    return 1;
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

void file_return(FILE * fp, int last_offset)
{
    int cur_offset = ftell(fp);
    fseek(fp,last_offset - cur_offset,SEEK_CUR);  //退出时返回入口文件偏移处
    //DbgPrintf("cur_offset000000000000000000 = %ld\n",cur_offset);
}

/*
读取某行 line为相对目前跳转行数
*/
//int line_read(char *filename, unsigned int line)
int line_read(FILE * fp, unsigned int line,int last_offset,unsigned char** gps_data,int* data_len)
{
    (*data_len) = 0;               //重新赋值为0
    if(fp == NULL)
        return -1;
    char buf[4096];
    int nread = 0, count = 0 ;
    //long lastseek = 0, size=0;
    char *pch = NULL, *pstr = NULL;
    memset(buf, 0, 4096);
    unsigned char data[2048] = {0};
    int len = 0;
    static int search_count = 0;
    while(1)
    {
        fgets(buf, 2047, fp);
        if(feof(fp))
        {
            printf("FEOF!\n");
            //file_return(fp,last_offset);
            //exit;
            return -1;
        }
        //if(nread >= 0)
        if(strstr(buf,"gps_data") == NULL)               //不是gps数据行
        {
            continue;
        }
        else
        {
            int i = 0;
            pstr = buf;
            //printf("pstr = %s\n",pstr);
            if(strstr(buf,"gps_data=null") == NULL) //gps数据非空
            {
                pstr = buf;
                int next_len;
                //char* gps_null_dec = (char*)malloc(sizeof(char) * 4);   //判断gps数据是否为空
                handle_gps_string(fp,pstr,last_offset,gps_data,data_len);
                printf("data_len = %d\n",*data_len);
                return 1;
            }
            else
            {

            }

        }
    }

}
#if 0
{

}
#endif

void handle_gps_string(FILE * fp, char* pstr,int last_offset,unsigned char** gps_data,int* data_len)
{
    ITEM item;
    unsigned char data[2048] = {0};
    int len = 0;
    get_item_from_line(pstr, &item);
    if(strcmp(item.key, "gps_data") == 0)
    {
        int j = 0;
        if(strcmp(item.value,"null") == 0)
        {
            return ;
        }
        //printf("111item.value = %s\n",item.value);
        for(int i = 0;i < strlen(item.value) - 1;i++)
        {
            if(*(item.value + i) != ' ')
            {
                char cut[2];
                int hex_len = 0;
                strncpy(cut, item.value + i, 2);
                //printf("cut , %d = %s\n",i,cut);
                StringToHex(cut, data + (*data_len), &hex_len);
                (*data_len)++;
                i++;                                    //取两个字符
            }
        }
        
        *gps_data = (unsigned char*)malloc((*data_len) * sizeof(char));
        memcpy(*gps_data,data,(*data_len));
        safe_free(item.key);
        safe_free(item.value);
        //break;
    }
    else
    {
        printf("not match\n");
    }
    file_return(fp,last_offset);
    //fclose(fp);
    //return 1;
}