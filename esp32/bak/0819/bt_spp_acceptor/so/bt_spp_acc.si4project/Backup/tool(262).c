/*******************************************************************************
* File Name          : rtk_bt.h
* Author             : Daich
* Revision           : 1.0
* Date               : 16/05/2019
* Description        : bluetooth
*
* HISTORY***********************************************************************
* 13/05/2019  |                                             | Daich
*
*******************************************************************************/


/*
相等返回1，不等返回2
*/
int array_cmp(void* src1,void* src2,int arr_len,int type_len)
{
	char* src3 = (char*) src1;
	char* src4 = (char*) src2;
	for(int i = 0;i < (arr_len * type_len);i++)
	{
		if(*(src3+i) != *(src4+i))
		{
			return -1;
		}
	}
	return 1;
}

