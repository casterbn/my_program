#include "handle_data.h"
#include "main.h"
#include "rtcm.h"

typedef struct data_
{
	int time;
    float accData0;
    float accData1;
    float accData2;
    float gyroData0;
    float gyroData1;
    float gyroData2;
    unsigned char gps_data[600];
    int gps_data_len;
    int invalid_gps_len;       //已经被连接到上一包的数据本包不再继续使用
	unsigned char rtcm_data[600];
	int rtcm_data_len;
}DATA_S;

DATA_S cur_data,last_data;

int get_data = 0;

FILE * fp;
int line = 0;
int first_read = 0;
int read_rtk_data(FILE *fp, rtcm_t *rtcm, obs_t *obs, nav_t *nav, int *numofread) //RTK_2019-06-03-11_48_41
{
    //char gps_data[2048] = {0};

	//fp = fopen("RTK_2019-06-04-15_39", "r");

    //rtk_input_monitor_thread_create();
    //while(1)
    {
        //if(get_data)
		{
			get_data = 0;
			int excess_len;

			memcpy(&last_data, &cur_data, sizeof(DATA_S));
			//if()
			unsigned char* gps_data = NULL;
			unsigned char* gps_data_next = NULL;
			int timer = read_timer_value("timer", fp);
			float accData0 = read_ins_value("accData0", fp);
			float accData1 = read_ins_value("accData1", fp);
			float accData2 = read_ins_value("accData2", fp);
#if 1
			float gyroData0 = read_ins_value("gyroData0", fp);
			float gyroData1 = read_ins_value("gyroData1", fp);
			float gyroData2 = read_ins_value("gyroData2", fp);
			printf("timer = %d\n", timer);
			//printf("gyroData0 = %f\n",gyroData0);
			//printf("gyroData1 = %f\n",gyroData1);
			//printf("gyroData2 = %f\n",gyroData2);
			line += 7;
			int cur_data_len = 0;
			int stat = read_gps_data("gps_data", fp, &gps_data, &cur_data_len);
			cur_data.time = timer;
			cur_data.accData0 = accData0;
			cur_data.accData0 = accData1;
			cur_data.accData0 = accData2;
			cur_data.accData0 = gyroData0;
			cur_data.accData0 = gyroData1;
			cur_data.accData0 = gyroData2;
			if ((stat == -1) || (cur_data_len == cur_data.invalid_gps_len)) //本包无或者被读完
			{
				printf("stat = %d\n", stat);
				cur_data.gps_data_len = last_data.gps_data_len;
				cur_data.invalid_gps_len = 0;
				printf("last_data.gps_data_len = %d\n", last_data.gps_data_len);
				memcpy(cur_data.gps_data, last_data.gps_data, last_data.gps_data_len);
				printf("222222222222222222222\n");
			}
			else
			{
				int j = 0;
				for (j = 0; j < cur_data_len - 1; j++)
				{
					if ((gps_data[j] == 0xd3) && (gps_data[j + 1] == 0x00))
						break;
				}
				if (j == cur_data_len - 1)                        //当前帧无有效数据，使用上一帧
				{
					cur_data.gps_data_len = last_data.gps_data_len;
					cur_data.invalid_gps_len = 0;
					memcpy(cur_data.gps_data, last_data.gps_data, last_data.gps_data_len);
				}
				else
				{
					int next_gps_len;
					//memcpy(cur_data.gps_data,gps_data,cur_data_len);
					//printf("cur_data.invalid_gps_len = %d\n", cur_data.invalid_gps_len);
					//printf("cur_data_len = %d\n",cur_data_len);
					//memcpy(cur_data.gps_data, gps_data + cur_data.invalid_gps_len, cur_data_len - cur_data.invalid_gps_len);//去掉已连接上一包的数据
					memcpy(cur_data.gps_data, gps_data, cur_data_len);//去掉已连接上一包的数据
					safe_free(gps_data);
					long offset = ftell(fp);
					stat = line_read(fp, 8, offset, &gps_data_next, &next_gps_len);         //读取下一包gps
					if (stat == 1)
					{
						for (excess_len = 0; excess_len < next_gps_len - 1; excess_len++)
						{
							if ((gps_data_next[excess_len] == 0xd3) && (gps_data_next[excess_len + 1] == 0x00))
								break;
						}
						if (excess_len == next_gps_len - 1)        //下一包没有数据头，则全部连接到上一包
						{
							memcpy(cur_data.gps_data + cur_data_len - cur_data.invalid_gps_len, gps_data_next, next_gps_len);
							excess_len += 1;
						}
						else if (excess_len > 0)     //下一包有数据头，则连接首个数据头前数据到上一包
						{
							memcpy(cur_data.gps_data + cur_data_len - cur_data.invalid_gps_len, gps_data_next, excess_len);    //下一包有数据头，则断点之前连接到上一包
						}
						cur_data.invalid_gps_len = excess_len;//当前多余的
					}
					else
					{
						return -1;              //读完返回
					}
					//当前长度 - 传给上一包的 + 收取下一包的
					//cur_data.gps_data_len = cur_data_len - last_data.invalid_gps_len + excess_len;
					cur_data.gps_data_len = cur_data_len + excess_len;
				}
			}

			//printf("cur_data.gps_data_len = %d\n", cur_data.gps_data_len);
			//printf("***********************************************timer = %d\n",timer);
			//printf("gyroData0 = %f\n", gyroData0);
			//printf("gyroData1 = %f\n", gyroData1);
			//printf("gyroData2 = %f\n", gyroData2);
			printf("cur_data.gps_data_len = %d\n", cur_data.gps_data_len);
			printf("cur_data.gps_data =  ");
			for (int i = 0; i < cur_data.gps_data_len; i++)
			{
				printf(" %02x",cur_data.gps_data[i]);
			}
			printf("\n");
			for (int i = 0; i < cur_data.gps_data_len; i++)
			{
				int ret = input_rtcm3_data(rtcm, cur_data.gps_data[i], obs, nav);
			}
			line++;
			printf("rtcm.nbyte = %d\n", rtcm->nbyte);
			printf("rtcm.buff =  ");
			cur_data.rtcm_data_len = rtcm->nbyte;
			memcpy(cur_data.rtcm_data,rtcm->buff,rtcm->nbyte);
			for (int i = 0; i < rtcm->nbyte; i++)
			{
				printf( " %02x", rtcm->buff[i]);
			}
			printf("\n\n");
			if (feof(fp) != 0)
			{
				fclose(fp);
				return -1;
			}
				
        }
#endif
    }
    
}

void safe_free(void *ptr)
{
    if(ptr != NULL)
    {
        free(ptr);
        ptr == NULL;
    }
}