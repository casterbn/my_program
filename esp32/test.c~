#include <stdio.h>

int a[3] = {0x98,0x2c,0xbb,0x33,0x1d,0xe0};
int b[3] = {0x98,0x2c,0xbb,0x33,0x1d,0xe0};

int i;
int array_cmp(void* src1,void* src2,int arr_len,int type_len)
{
	char* src3 = (char*) src1;
	char* src4 = (char*) src2;
	for(i=0;i<(arr_len*type_len);i++)
	{
		//if((*(src1+type_len))!=(*(src2+type_len)))
		//if(((*(src3+i)) != (*(src4+i)))
		//if(1)
		if(*(src3+i) != *(src4+i))
		{
			return -1;
		}
	}
	return 1;
}


void main()
{
	if(array_cmp(a,b,3,sizeof(int)) == 1)
	{
		printf("ok\n");
	}
	else
	{
		printf("not\n");
	}	
}
