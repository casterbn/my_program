// rtcm_diff.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <iostream>
#include "rtcm.h"
#include "main.h"
#include "handle_data.h"

#include "windows.h"  


//extern void listFiles(const char * dir);

int read_epoch_from_file(FILE *fRTCM, rtcm_t *rtcm, obs_t *obs, nav_t *nav, int *numofread)
{
	int ret = 0, readCount = 0;
	char buff = ' ';
	*numofread = 0;
	while (!feof(fRTCM))
	{
		memset(&buff, 0, sizeof(buff));
		readCount = fread(&buff, sizeof(char), 1, fRTCM);
		//printf("readCount = %d\n", readCount);
		if (readCount < 1)
		{
			/* file error or eof of file */
			break;
		}
		*numofread += readCount;
		ret = input_rtcm3_data(rtcm, buff, obs, nav);

		if (ret == 1)
		{
			break;
		}
	}
	return ret;
}


int rtcm_diff(const char *fname1, const char *fname2, int year, int doy)
{
	/* read two rtcm data stream and comparing two files */
	obs_t obs[2] = { 0 };
	obsd_t *obsd[2] = { NULL };
	nav_t nav = { 0 };
	rtcm_t rtcm[2] = { 0 };
	FILE *fRTCM[2] = { NULL };
	FILE *fDIFF[4] = { NULL };
	int num[2] = { 0 }, ret[2] = { 0 }, week, i, j, f, sys, prn;
	double time[2] = { 0 };
	char buffer[255];
	fRTCM[0] = fopen(fname1, "rb");
	fRTCM[1] = fopen(fname2, "rb");
	if (fRTCM[0] == NULL || fRTCM[1] == NULL) {
		if (fRTCM[0] != NULL) fclose(fRTCM[0]);
		if (fRTCM[1] != NULL) fclose(fRTCM[1]);
	}
	for (i = 0; i < 4; ++i)
	{
		sprintf(buffer, "diff_%i.csv", i);
		fDIFF[i] = fopen(buffer, "w");
	}
	/* read file */

	set_approximate_time(year, doy, rtcm + 0);
	set_approximate_time(year, doy, rtcm + 1);

	while (1)
	{
		/* read rover data one byte */
		ret[0] = read_epoch_from_file(fRTCM[0], rtcm+0, obs+0, &nav, num+0);
		if (ret[0] != 1) {
			/* can not find the complete epoch data */
			if (feof(fRTCM[0])) break;
			continue;
		}
		/* rover data is ready, find the base data */
		time[0] = time2gpst(obs[0].time, &week);
		/*if (time[0] >= 410257.2) {
			printf("something wrong this epoch\n");
		}*/
		time[0] += week * 7 * 24 * 3600.0;

		///* DW DEBUG*/
		//for (i = 0; i < obs[0].n; ++i) {
		//	//sys = satsys(obsd[0]->sat, &prn);
		//	obsd[0] = obs[0].data + i;
		//	int s = satidx(obsd[0]->sat, &prn);
		//	if (s >= 0 && s < 4 && fDIFF[s]!=NULL) {
		//		fprintf(fDIFF[s], "%14.3f,%02d,%14.4f,%14.4f,%14.4f,%14.4f,%14.4f,%14.4f,%02i,%02i,%02i,%02i,%02i,%02i\n", time2gpst(obs[0].time, &week), prn, 
		//			//obsd[0]->P[0], obsd[0]->P[1], 
		//			//obsd[0]->L[0], obsd[0]->L[1],
		//			//obsd[0]->D[0], obsd[0]->D[1],
		//			-1.0,-1.0, -1.0, -1.0, -1.0, -1.0,
		//			obsd[0]->SNR[0], obsd[0]->SNR[1], i, 0, obs[0].n, 0);
		//	}		
		//}

		while (1)
		{
			num[1] = 0;
			ret[1] = read_epoch_from_file(fRTCM[1], rtcm + 1, obs + 1, &nav, num + 1);
			if (ret[1] != 1) break;
			time[1] = time2gpst(obs[1].time, &week);
			time[1] += week * 7 * 24 * 3600.0;
			if (time[1] >= time[0]) // (fabs(time[0] - time[1]) < 0.01) //
			{
				/* time match */
				while (fabs(time[0] - time[1]) > 0.05) {
					ret[0] = read_epoch_from_file(fRTCM[0], rtcm + 0, obs + 0, &nav, num + 0);
					if (ret[0] != 1) {
						/* can not find the complete epoch data */
						if (feof(fRTCM[0])) break;
						continue;
					}
					/* rover data is ready, find the base data */
					time[0] = time2gpst(obs[0].time, &week);
					time[0] += week * 7 * 24 * 3600.0;
				}

				break;
			}
		}
		if (fabs(time[0] - time[1]) < 0.01)
		{
			double tow = time2gpst(obs[0].time, &week);
			printf("TOW: %f\n", tow);
			for (i = 0; i < obs[0].n; ++i)
			{
				for (j = 0; j < obs[1].n; ++j)
				{
					if (obs[0].data[i].sat == obs[1].data[j].sat) break;
				}
				if (j == obs[1].n)
				{
					continue;
				}
				obsd[0] = obs[0].data + i;
				obsd[1] = obs[1].data + j;
				sys = satsys(obsd[0]->sat, &prn);
				double diffP[2] = { 0 };
				double diffL[2] = { 0 };
				double diffD[2] = { 0 };
				double diffS[2] = { 0 };
				if (obsd[0]->P[0] != 0.0 && obsd[1]->P[0] != 0.0) diffP[0] = obsd[1]->P[0] - obsd[0]->P[0];
				if (obsd[0]->P[1] != 0.0 && obsd[1]->P[1] != 0.0) diffP[1] = obsd[1]->P[1] - obsd[0]->P[1];
				if (obsd[0]->L[0] != 0.0 && obsd[1]->L[0] != 0.0) diffL[0] = obsd[1]->L[0] - obsd[0]->L[0];
				if (obsd[0]->L[1] != 0.0 && obsd[1]->L[1] != 0.0) diffL[1] = obsd[1]->L[1] - obsd[0]->L[1];
				if (obsd[0]->D[0] != 0.0 && obsd[1]->D[0] != 0.0) diffD[0] = obsd[1]->D[0] - obsd[0]->D[0];
				if (obsd[0]->D[1] != 0.0 && obsd[1]->D[1] != 0.0) diffD[1] = obsd[1]->D[1] - obsd[0]->D[1];
				if (obsd[0]->SNR[0] != 0 && obsd[1]->SNR[0] != 0) diffS[0] = (obsd[1]->SNR[0] - obsd[0]->SNR[0]) / 4.0;
				if (obsd[0]->SNR[1] != 0 && obsd[1]->SNR[1] != 0) diffS[1] = (obsd[1]->SNR[1] - obsd[0]->SNR[1]) / 4.0;
				int s = satidx(obsd[0]->sat, &prn);
				if (s >= 0 && s < 4&&fDIFF[s]!=NULL)
				{
					fprintf(fDIFF[s], "%14.3f,%02d,%14.4f,%14.4f,%14.4f,%14.4f,%14.4f,%14.4f,%14.4f,%14.4f,%02i,%02i,%02i,%02i\n", tow, prn, diffP[0], diffP[1], diffL[0], diffL[1], diffD[0], diffD[1], diffS[0], diffS[1], i, j, obs[0].n, obs[1].n);
				}
			}
		}
	}

	if (fRTCM[0] != NULL) fclose(fRTCM[0]);
	if (fRTCM[1] != NULL) fclose(fRTCM[1]);
	for (i = 0; i < 4; ++i)
	{
		if (fDIFF[i] != NULL) fclose(fDIFF[i]);
	}

	return 1;
}

extern char file_name[30][80];
extern int file_count;
int test_nmea_decoder(const char *fname, int year, int doy)
{
	/* read two rtcm data stream and comparing two files */
	obs_t obs = { 0 };
	nav_t nav = { 0 };
	rtcm_t rtcm = { 0 };
	FILE *fp = NULL;
	int num, ret, week, i, j, f, sys, prn;
	double time[2] = { 0 };
	char buffer[255];
	/*
	fp= fopen(fname, "rb");
	if (fp == NULL) {
		printf("Invalid NMEA file\n");
		return 0;
	}
	*/
	char dir[1024] ;
	//listFiles("D:\\GitHub\\openrtk-gnss_dev_dma\\openrtk-gnss_dev_dma\\msvc\\*.*");
	listFiles("..\\rtcm_diff\\*.*");
	for (int i = 0; i < file_count; i++)
	{
		printf("%d. %s\n", i, file_name[i]);
	}
	char str[80];
	int file_num = 0;
	printf("please enter the num to select the file to decode:\n");
	scanf("%d", &file_num);
	printf("file_num = %d\n",file_num);

	fp = fopen(file_name[file_num], "r");
	if (fp == NULL)
	{
		printf("no file\n");
	}

	while (1) {
		//ret = read_epoch_from_file(fp, &rtcm, &obs, &nav, &num);
		int ret = 0;
		while (ret == 0)
		{
			ret = read_rtk_data(fp, &rtcm, &obs, &nav, &num);
			if (ret == -1)
			{
				printf("file end\n");
				return -1;
			}
		}
		
		if (ret != 1) {
			/* can not find the complete epoch data */
			if (feof(fp)) break;
			continue;
		}

	}

	return 1;
}

int main()
{
	//return rtcm_diff("C:\\data\\125_2019\\st10125a05.dat", "C:\\data\\125_2019\\st11125a00.dat", 2019, 125);
	//return rtcm_diff("129\\ST_openrtk_129r53.rtcm3","129\\ST_evk_129r53.rtcm3", 2019, 129);
	return test_nmea_decoder("ST_126d03.nmea", 2019, 126);
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
