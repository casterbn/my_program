// msvc.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <iostream>
#include "ppprtk.h"
#include "rinex_file.h"

int read_epoch_from_file(FILE *fRTCM, gnss_rtcm_t *rtcm, int stnID, int *numofread)
{
	int ret = 0;
    int sys;
	size_t readCount = 0;
	char buff = ' ';
	*numofread = 0;
	while (!feof(fRTCM))
	{
		memset(&buff, 0, sizeof(buff));
		readCount = fread(&buff, sizeof(char), 1, fRTCM);
		if (readCount < 1)
		{
			/* file error or eof of file */
			break;
		}
		*numofread += (int)readCount;
		ret = input_rtcm3(buff, stnID, rtcm);

		if (ret == 1)
		{
#ifdef _USE_PPP_
			double time_rov, time_ssr, timediff;
			int week = 0;
			time_rov = time2gpst(rtcm->obs[stnID].time, &week);
			time_rov += week * 7 * 24 * 3600.0;

			if (rtcm->nav.ns == 0) break;
			for (int i = 0; i < rtcm->nav.ns; ++i)
			{
				time_ssr = time2gpst(rtcm->nav.ssr[i].t0[0], &week);
				time_ssr += week * 7 * 24 * 3600.0;
				timediff = time_rov - time_ssr;
				if (timediff < 0.0)
					break;
			}
	
			if (timediff > 0.0)
#endif
				break;
		}
#ifdef _USE_PPP_
		if (ret == 10)
		{
			double time_rov, time_ssr, timediff;
			int week = 0;
			time_rov = time2gpst(rtcm->obs[stnID].time, &week);
			time_rov += week * 7 * 24 * 3600.0;  

            if (rtcm->nav.ns == 0) break;

			for (int i = 0; i < rtcm->nav.ns; ++i)
			{
				time_ssr = time2gpst(rtcm->nav.ssr[i].t0[0], &week);
				time_ssr += week * 7 * 24 * 3600.0;
				timediff = time_rov - time_ssr;
				if (timediff < 30.0)
					break;
			}

			if (timediff < 30.0)
				break;
		}

		if (ret == 2)
		{
			int satidx = -1;
			int neph = rtcm->nav.n;
            if (rtcm->nav.n == 0) return ret;

			for (int i = 0; i < rtcm->obs[stnID].n; ++i)
			{
				if (rtcm->nav.eph[neph - 1].sat == rtcm->obs[stnID].data[i].sat &&
					abs(rtcm->nav.eph[neph - 1].toe.time - rtcm->obs[stnID].time.time) < 7200)
				{
					satidx = i;
					break;
				}
			}
			if (satidx == -1)
			{
				rtcm->nav.n--;
                sys = satsys(rtcm->nav.eph[neph - 1].sat,NULL);
                if (sys==_SYS_GPS_ ) rtcm->nav.n_gps--;
                if (sys == _SYS_GLO_) rtcm->nav.ng--;
                if (sys == _SYS_GAL_) rtcm->nav.n_gal--;
                if (sys == _SYS_BDS_) rtcm->nav.n_bds--;
				rtcm->nav.eph[neph - 1].sat = 0;
			}

           
   //         break;
			int oldbrdc = 0;
			for (int i = 0; i < rtcm->nav.ns; i++)
			{
				for (int j = 0; j < rtcm->nav.n; j++)
				{
					if (rtcm->nav.ssr[i].iode != 0 && rtcm->nav.ssr[i].iode != rtcm->nav.eph[j].iode && (rtcm->nav.ssr[i].sat == rtcm->nav.eph[j].sat))
					{
						oldbrdc += 1;
						break;
					}
				}
			}
			/* find the complete epoch data */
			if (oldbrdc < 4)
				break;  
	}
#endif
}
	return ret;
}


int ppp_main1(const char *rovfname, const char *brdcfname, const char *ssrfname, int year, int doy)
{
#ifdef _USE_PPP_

    FILE *fRTCM[3] = { NULL };
    int num_rov = 0, num_eph, num_ssr = 0, ret_eph = 0, ret_ssr = 0, ret_rov = 0, week = 0;
    gnss_rtcm_t rtcm = { 0 };
    rcv_ppp_t rcv_ppp = { 0 };
    double time_ref = 0.0, time_rov = 0.0;
    nav_t *nav = &rtcm.nav;
    obs_t *rov =  rtcm.obs + 0;
    rcv_ppp_t *rcv = &rcv_ppp;

   	for (num_rov = 0; num_rov < MAXSTN; ++num_rov)
	{
		set_approximate_time(year, doy, rtcm.rcv + num_rov);
	} 

    fRTCM[0] = fopen(rovfname, "rb");    if (fRTCM[0] == NULL) return 0;
    fRTCM[1] = fopen(brdcfname, "rb");   if (fRTCM[1] == NULL) { fclose(fRTCM[0]); return 0; }
    fRTCM[2] = fopen(ssrfname, "rb");    if (fRTCM[2] == NULL) { fclose(fRTCM[0]); fclose(fRTCM[1]);  return 0; }

    while (1)
    {
        while (1)
        {
            /* read rover data one byte */
            num_rov = 0;
            ret_rov = read_epoch_from_file(fRTCM[0], &rtcm, 0, &num_rov);
            if (ret_rov == 1 ) //&& rov->pos[0] != 0.0
                break;
        }
        if (ret_rov != 1)
        {
            if (feof(fRTCM[0])) break;
            continue;
        } 

        while (1)
        {
            num_eph = 0;
            ret_eph = read_epoch_from_file(fRTCM[1], &rtcm, 0, &num_eph);
            if (ret_eph == 2)
                break;
        }
        if (ret_eph != 2)
        {
            if (feof(fRTCM[1])) break;
            continue;
        }

        while (1)
        {
            /* read ssr one byte */
            num_ssr = 0;
            ret_ssr = read_epoch_from_file(fRTCM[2], &rtcm, 0, &num_ssr);
            if (ret_ssr == 10)  
                break;
        }
        if (ret_ssr != 10)
        {
            if (feof(fRTCM[2])) break;
            continue;
        }  

       if (nav->n_gps < 8) continue;
//        printf("time=%10d obsnum=%2d brdcnum=%2d ssrnum=%2d\n", rov->time.time,rov->n, nav->n, nav->ns);

        ppp_processor(rov, nav, rcv, 0);

    }
    if (fRTCM[0]) fclose(fRTCM[0]);
    if (fRTCM[1]) fclose(fRTCM[1]);
    if (fRTCM[2]) fclose(fRTCM[2]);
#endif
    return 1;
}

typedef struct
{
	double time, blh[3], xyz[3], N, HDOP;
	int numOfSat, solType, lineIndex;
} GGA_t;

bool ParseGGA(const char *buffer, GGA_t *gga)
{
	memset(gga, 0, sizeof(GGA_t));

	std::string lineReadStr(buffer), curstr;

	std::string::size_type nLoc = lineReadStr.find("$G"), nPreLoc; if (nLoc == std::string::npos) return false;
	nLoc = lineReadStr.find("GGA"); if (nLoc == std::string::npos) return false;
	//$GPGGA,201615.60,3946.9431210,N,08404.9232523,W,5,07,1.19,255.0887,M,0.0,M,0.0,0000*6A
	nPreLoc = 0;
	nLoc = lineReadStr.find(',', nPreLoc); if (nLoc == std::string::npos) return false;
	// GPGGA
	nPreLoc = nLoc + 1;
	nLoc = lineReadStr.find(',', nPreLoc); if (nLoc == std::string::npos) return false;
	// UTC time
	curstr = lineReadStr.substr(nPreLoc, nLoc - nPreLoc); if (curstr.length() < 6) return false;
	double hh = atof(curstr.substr(0, 2).c_str());
	double mm = atof(curstr.substr(2, 2).c_str());
	double ss = atof(curstr.substr(4).c_str());
	gga->time = hh * 3600.0 + mm * 60.0 + ss;
	// latitude
	nPreLoc = nLoc + 1;
	nLoc = lineReadStr.find(',', nPreLoc); if (nLoc == std::string::npos) return false;
	curstr = lineReadStr.substr(nPreLoc, nLoc - nPreLoc); if (curstr.length() < 4) return false;
	double dd = atof(curstr.substr(0, 2).c_str());
	mm = atof(curstr.substr(2).c_str());
	gga->blh[0] = (dd + mm / 60.0)*PI / 180.0;
	// S/N for latitude
	nPreLoc = nLoc + 1;
	nLoc = lineReadStr.find(',', nPreLoc); if (nLoc == std::string::npos) return false;
	curstr = lineReadStr.substr(nPreLoc, nLoc - nPreLoc); if (curstr.length() < 1) return false;
	if (curstr[0] == 'S' || curstr[0] == 's') gga->blh[0] = -gga->blh[0];
	// longitude
	nPreLoc = nLoc + 1;
	nLoc = lineReadStr.find(',', nPreLoc); if (nLoc == std::string::npos) return false;
	curstr = lineReadStr.substr(nPreLoc, nLoc - nPreLoc); if (curstr.length() < 5) return false;
	dd = atof(curstr.substr(0, 3).c_str());
	mm = atof(curstr.substr(3).c_str());
	gga->blh[1] = (dd + mm / 60.0)*PI / 180.0;
	// E/W for longitude
	nPreLoc = nLoc + 1;
	nLoc = lineReadStr.find(',', nPreLoc); if (nLoc == std::string::npos) return false;
	curstr = lineReadStr.substr(nPreLoc, nLoc - nPreLoc); if (curstr.length() < 1) return false;
	if (curstr[0] == 'W' || curstr[0] == 'w') gga->blh[1] = -gga->blh[1];
	// solution type
	nPreLoc = nLoc + 1;
	nLoc = lineReadStr.find(',', nPreLoc); if (nLoc == std::string::npos) return false;
	curstr = lineReadStr.substr(nPreLoc, nLoc - nPreLoc); if (curstr.length() < 1) return false;
	gga->solType = atoi(curstr.c_str());
	// number of satellite
	nPreLoc = nLoc + 1;
	nLoc = lineReadStr.find(',', nPreLoc); if (nLoc == std::string::npos) return false;
	curstr = lineReadStr.substr(nPreLoc, nLoc - nPreLoc); if (curstr.length() < 1) return false;
	gga->numOfSat = atoi(curstr.c_str());
	// HDOP
	nPreLoc = nLoc + 1;
	nLoc = lineReadStr.find(',', nPreLoc); if (nLoc == std::string::npos) return false;
	curstr = lineReadStr.substr(nPreLoc, nLoc - nPreLoc); if (curstr.length() < 1) return false;
	gga->HDOP = atof(curstr.c_str());
	// altitude
	nPreLoc = nLoc + 1;
	nLoc = lineReadStr.find(',', nPreLoc); if (nLoc == std::string::npos) return false;
	curstr = lineReadStr.substr(nPreLoc, nLoc - nPreLoc); if (curstr.length() < 1) return false;
	gga->blh[2] = atof(curstr.c_str());
	// M/m
	nPreLoc = nLoc + 1;
	nLoc = lineReadStr.find(',', nPreLoc); if (nLoc == std::string::npos) return false;
	curstr = lineReadStr.substr(nPreLoc, nLoc - nPreLoc); if (curstr.length() < 1) return false;
	if (curstr[0] != 'M'&&curstr[0] != 'm') return false;
	// geo height N
	nPreLoc = nLoc + 1;
	nLoc = lineReadStr.find(',', nPreLoc); if (nLoc == std::string::npos) return false;
	curstr = lineReadStr.substr(nPreLoc, nLoc - nPreLoc); if (curstr.length() < 1) return false;
	gga->N = atof(curstr.c_str());
	// M/m
	nPreLoc = nLoc + 1;
	nLoc = lineReadStr.find(',', nPreLoc); if (nLoc == std::string::npos) return false;
	curstr = lineReadStr.substr(nPreLoc, nLoc - nPreLoc); if (curstr.length() < 1) return false;
	if (curstr[0] != 'M'&&curstr[0] != 'm') return false;
	gga->blh[2] += gga->N;
	if (gga->blh[0] == 0.0&&gga->blh[1] == 0.0&&gga->blh[2] == 0.0) return false;
	return true;
}

void print_kml_heder(FILE *fKML)
{
	// write header for KML 
	if (fKML) {
		fprintf(fKML, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
		fprintf(fKML, "<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n");
		fprintf(fKML, "<Document>\n");
		// fprintf(fKML, "<Placemark>\n");    
		// fprintf(fKML, "<name>extruded</name>\n");
		// fprintf(fKML, "<LineString>\n");
		// fprintf(fKML, "<extrude>1</extrude>\n");
		// fprintf(fKML, "<tessellate>1</tessellate>\n");
		// fprintf(fKML, "<altitudeMode>relativeToGround</altitudeMode>\n");
		// fprintf(fKML, "<coordinates>\n"); 
		fprintf(fKML, "<Style id=\"spp\">\n");
		fprintf(fKML, "<IconStyle>\n");
		fprintf(fKML, "<color>ffff00ff</color>\n");
		fprintf(fKML, "<scale>0.500</scale>\n");
		fprintf(fKML, "<Icon>\n");
		fprintf(fKML, "<href>http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png</href>\n");
		fprintf(fKML, "</Icon>\n");
		fprintf(fKML, "</IconStyle>\n");
		fprintf(fKML, "</Style>\n");
		fprintf(fKML, "<Style id=\"fix\">\n");
		fprintf(fKML, "<IconStyle>\n");
		fprintf(fKML, "<color>ff008800</color>\n");
		fprintf(fKML, "<scale>0.500</scale>\n");
		fprintf(fKML, "<Icon>\n");
		fprintf(fKML, "<href>http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png</href>\n");
		fprintf(fKML, "</Icon>\n");
		fprintf(fKML, "</IconStyle>\n");
		fprintf(fKML, "</Style>\n");
		fprintf(fKML, "<Style id=\"flt\">\n");
		fprintf(fKML, "<IconStyle>\n");
		fprintf(fKML, "<color>ff00aaff</color>\n");
		fprintf(fKML, "<scale>0.500</scale>\n");
		fprintf(fKML, "<Icon>\n");
		fprintf(fKML, "<href>http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png</href>\n");
		fprintf(fKML, "</Icon>\n");
		fprintf(fKML, "</IconStyle>\n");
		fprintf(fKML, "</Style>\n");
	}
	return;
}

void print_kml_gga(FILE *fKML, char *buffer)
{
	if (fKML == NULL) return;
	GGA_t gga = { 0 };
	if (ParseGGA(buffer, &gga))
	{
		double lat = gga.blh[0] * 180.0 / PI;
		double lon = gga.blh[1] * 180.0 / PI;
		double ht = gga.blh[2];
		if (fKML) {
			fprintf(fKML, "<Placemark>\n");
			if (gga.solType == 1)
			{
				fprintf(fKML, "<styleUrl>#spp</styleUrl>\n");
			}
			else if (gga.solType == 4)
			{
				fprintf(fKML, "<styleUrl>#fix</styleUrl>\n");
			}
			else if (gga.solType == 5)
			{
				fprintf(fKML, "<styleUrl>#flt</styleUrl>\n");
			}
			else
			{
				fprintf(fKML, "<styleUrl>#spp</styleUrl>\n");
			}
			fprintf(fKML, "<Point>\n");
			fprintf(fKML, "<coordinates>%14.9f,%14.9f,%14.4f</coordinates>\n", lon, lat, ht);
			fprintf(fKML, "</Point>\n");
			fprintf(fKML, "</Placemark>\n");
		}
	}
	return;
}

void print_kml_eof(FILE *fKML)
{
	if (fKML)
	{
		// fprintf(fKML, "</coordinates>\n");    
		// fprintf(fKML, "</LineString>\n");
		// fprintf(fKML, "</Placemark>\n");
		fprintf(fKML, "</Document>\n");
		fprintf(fKML, "</kml>\n");

	}
}


int ppp_main2(const char *rovfname, const char *ssrfname, int year, int doy)
{
#ifdef _USE_PPP_

    FILE *fRTCM[2] = { NULL };
    FILE *fSOL = NULL;
    FILE *fGGA = NULL;
    FILE *fKML = NULL;
    char buffer_rov = ' ', buffer_ssr = ' ';
    int num_rov = 0, num_ssr = 0, ret_ssr = 0, ret_rov = 0, week = 0;
    gnss_rtcm_t rtcm = { 0 };
    rcv_ppp_t rcv_ppp = { 0 };
    rcv_ppp_t *rcv = &rcv_ppp;
    double time_ref = 0.0, time_rov = 0.0;
    nav_t *nav = &rtcm.nav;
    obs_t *rov = rtcm.obs + 0;
    obs_t ref_cur = { 0 };
    char fileName[255] = { 0 };
    char outfilename[255] = { 0 };

    char gga[1024] = { 0 }, sol[1024] = { 0 };

    const char * result = strrchr(rovfname, '\\');
    if (result != NULL)
        strncpy(fileName, result + 1, strlen(result));
    else
        strncpy(fileName, rovfname, strlen(rovfname));
    char *result1 = strrchr(fileName, '.');
    if (result1 != NULL) result1[0] = '\0';


    fRTCM[0] = fopen(rovfname, "rb"); if (fRTCM[0] == NULL) return 0;
    fRTCM[1] = fopen(ssrfname, "rb"); if (fRTCM[1] == NULL) { fclose(fRTCM[0]); return 0; }
    sprintf(outfilename, "%s.gga", fileName); fGGA = fopen(outfilename, "w");
    sprintf(outfilename, "%s.csv", fileName); fSOL = fopen(outfilename, "w");
    sprintf(outfilename, "%s.kml", fileName); fKML = fopen(outfilename, "w");

    print_kml_heder(fKML);

    for (num_rov = 0; num_rov < MAXSTN; ++num_rov)
    {
        set_approximate_time(year, doy, rtcm.rcv + num_rov);
    }

    while (1)
    {
        num_rov = 0;
        ret_rov = read_epoch_from_file(fRTCM[0], &rtcm, 0, &num_rov);
        if (ret_rov != 1) 
        {
            /* can not find the complete epoch data */
            if (feof(fRTCM[0])) break;
            continue;
        }
        /* no brdc */
        if (nav->n == 0) continue;

        while (1)
        {
            /* read ssr one byte */
            num_ssr = 0;
            ret_ssr = read_epoch_from_file(fRTCM[2], &rtcm, 0, &num_ssr);
            if (ret_ssr == 10 && nav->ns > 10)
                break;
        }
        if (ret_ssr != 10)
        {
            if (feof(fRTCM[2])) break;
            continue;
        }

        memset(gga, 0, sizeof(gga)); /* gga solution */
        memset(sol, 0, sizeof(sol)); /* filter solution */

        ppp_processor(rov, nav, rcv, 0);

        if (fGGA != NULL)
        {
            fprintf(fGGA, "%s", gga);
        }
        if (fSOL != NULL)
        {
            fprintf(fSOL, "%s", sol);
        }
        if (fKML != NULL)
        {
            print_kml_gga(fKML, gga);
        }
        printf("%s", gga);
    }

    print_kml_eof(fKML);
    if (fRTCM[0]) fclose(fRTCM[0]);
    if (fRTCM[1]) fclose(fRTCM[1]);
    if (fKML) fclose(fKML);
    if (fGGA) fclose(fGGA);
    if (fSOL) fclose(fSOL);
#endif
    return 1;
}

int rtk_main(const char *rovfname, const char *reffname, int year, int doy, int isPrint)
{
#ifndef _USE_PPP_
	FILE *fRTCM[2] = { NULL };
	FILE *fSOL = NULL;
	FILE *fGGA = NULL;
	FILE *fKML = NULL;
	char buffer_rov = ' ', buffer_ref = ' ';
	int num_rov = 0, num_ref = 0, ret_ref = 0, ret_rov = 0, week = 0;
	gnss_rtcm_t rtcm = { 0 };
	rcv_rtk_t rtk = { 0 };
	double time_ref = 0.0, time_rov = 0.0;
	nav_t *nav = &rtcm.nav;
	obs_t *rov = rtcm.obs + 0;
	obs_t *ref = rtcm.obs_ref + 0;
	obs_t ref_cur = { 0 };
	char fileName[255] = { 0 };
	char outfilename[255] = { 0 };

	char gga[1024] = { 0 }, sol[1024] = { 0 };

	const char * result = strrchr(rovfname, '\\');
	if (result != NULL)
		strncpy(fileName, result + 1, strlen(result));
	else
		strncpy(fileName, rovfname, strlen(rovfname));
	char *result1 = strrchr(fileName, '.');
	if (result1 != NULL) result1[0] = '\0';


	fRTCM[0] = fopen(rovfname, "rb"); if (fRTCM[0] == NULL) return 0;
	fRTCM[1] = fopen(reffname, "rb"); if (fRTCM[1] == NULL) { fclose(fRTCM[0]); return 0; }
	sprintf(outfilename, "%s.gga", fileName); fGGA = fopen(outfilename, "w");
	sprintf(outfilename, "%s.csv", fileName); fSOL = fopen(outfilename, "w");
	sprintf(outfilename, "%s.kml", fileName); fKML = fopen(outfilename, "w");

	print_kml_heder(fKML);

	for (num_rov = 0; num_rov < MAXSTN; ++num_rov)
	{
		set_approximate_time(year, doy, rtcm.rcv + num_rov);
	}

	while (1)
	{
		/* read rover data one byte */
		num_rov = 0;
		ret_rov = read_epoch_from_file(fRTCM[0], &rtcm, 0, &num_rov);
		if (ret_rov != 1) {/* no chinese comments */
			/* can not find the complete epoch data */
			if (feof(fRTCM[0])) break;
			continue;
		}
		/* no brdc */
		if (nav->n == 0) continue;
		//printf("G: %i, R:%i, E:%i, C:%i, SSR: %i\n", nav->n_gps, nav->ng, nav->n_gal, nav->n_bds, nav->ns);
		/* rover data is ready, find the base data */
		time_rov = time2gpst(rov->time, &week);
		time_rov += week * 7 * 24 * 3600.0;
		while (1)
		{
			if (ref->time.time > 0.0)
			{
				time_ref = time2gpst(ref->time, &week);
				time_ref += week * 7 * 24 * 3600.0;
				if (time_ref > time_rov)
				{
					/* time match */
					break;
				}
				ref_cur = *ref;
			}
			num_ref = 0;
			ret_ref = read_epoch_from_file(fRTCM[1], &rtcm, 1, &num_ref);
			if (ret_ref != 1) break;
		}
		time_ref = time2gpst(ref_cur.time, &week);
		time_ref += week * 7 * 24 * 3600.0;
		if (time_rov < time_ref - 0.01) continue;
		/* baseline processor */
		memset(gga, 0, sizeof(gga)); /* gga solution */
		memset(sol, 0, sizeof(sol)); /* filter solution */
		rtk_processor(rov, &ref_cur, nav, &rtk, gga, sol, isPrint);
		if (fGGA != NULL)
		{
			fprintf(fGGA, "%s", gga);
		}
		if (fSOL != NULL)
		{
			fprintf(fSOL, "%s", sol);
		}
		if (fKML != NULL)
		{
			//if (fabs(time_rov - floor(time_rov + 0.5)) < 0.01)
			{
				print_kml_gga(fKML, gga);
			}
		}
		printf("%s", gga);
	}
	print_kml_eof(fKML);
	if (fRTCM[0]) fclose(fRTCM[0]);
	if (fRTCM[1]) fclose(fRTCM[1]);
	if (fKML) fclose(fKML);
	if (fGGA) fclose(fGGA);
	if (fSOL) fclose(fSOL);
#endif
	return 1;
}

int rtk_main_rinex(const char *rovfname, const char *reffname, const char *navfname, int year, int doy, int isPrint)
{
	/* add the rtk pp using rinex input (rov, ref, nav/brdc) */
	/* need to read the obs_t and nav_t from files, */
	/* need to consider the nav_t ephemeris switch */
	/* follow the workflow from rtk_main */
	/* the major difference is the ephemeris is from nav/brdc instead of the original rtcm file */
#ifndef _USE_PPP_

	//static obs_t obss = { 0 };          /* observation data */
	//static nav_t navs = { 0 };          /* navigation data */
	//sta_t *sta = NULL;
	FILE *fSOL = NULL;
	FILE *fGGA = NULL;
	FILE *fKML = NULL;
	char fileName[255] = { 0 };
	char outfilename[255] = { 0 };

	const char * result = strrchr(rovfname, '\\');
	if (result != NULL)
		strncpy(fileName, result + 1, strlen(result));
	else
		strncpy(fileName, rovfname, strlen(rovfname));
	char *result1 = strrchr(fileName, '.');
	if (result1 != NULL) result1[0] = '\0';

	sprintf(outfilename, "%s.gga", fileName); fGGA = fopen(outfilename, "w");
	sprintf(outfilename, "%s.csv", fileName); fSOL = fopen(outfilename, "w");
	sprintf(outfilename, "%s.kml", fileName); fKML = fopen(outfilename, "w");

    print_kml_heder(fKML);

	char gga[1024] = { 0 }, sol[1024] = { 0 };

	const char* files[2] = { rovfname,reffname };
	const char **infile = files;
    int n = 2;
	rcv_rtk_t rtk = { 0 };
	std::vector<obs_t*> matched_obs_list;

	rinex_file rinex_file_reader;
	rinex_file_reader.set_navigation_file_name(navfname);
	rinex_file_reader.set_observation_file_name(infile, n);
	rinex_file_reader.read_navigation_rinex_file();
	rinex_file_reader.open_observation_rinex_files();

	while (true)
	{
		if (rinex_file_reader.read_next_epoch() == false) break;

		if (rinex_file_reader.match_obs_list() < 2) continue;

		rinex_file_reader.match_nav_list();

		nav_t nav = { 0 };
		rinex_file_reader.get_matched_ephemerides(nav);

		matched_obs_list.clear();
		rinex_file_reader.get_matched_obs_list(matched_obs_list);

		if (matched_obs_list.size() < 2) continue;
       
		/* baseline processor */
		memset(gga, 0, sizeof(gga)); /* gga solution */
		memset(sol, 0, sizeof(sol)); /* filter solution */
		rtk_processor(matched_obs_list[0], matched_obs_list[1], &nav, &rtk, gga, sol, isPrint);

		if (fGGA != NULL)
		{
			fprintf(fGGA, "%s", gga);
		}
		if (fSOL != NULL)
		{
			fprintf(fSOL, "%s", sol);
		}
		if (fKML != NULL)
		{
			print_kml_gga(fKML, gga);
		}

		struct tm Tm = { 0 };
		gmtime_s(&Tm, &rinex_file_reader.getTime());
		trace(1, "%04d/%02d/%02d %02d:%02d:%02d, %f, %f, %f \n", Tm.tm_year + 1900, Tm.tm_mon + 1, Tm.tm_mday, Tm.tm_hour, Tm.tm_min, Tm.tm_sec, rtk.x[0], rtk.x[1], rtk.x[2]);
	}

	print_kml_eof(fKML);

	if (fKML) fclose(fKML);
	if (fGGA) fclose(fGGA);
	if (fSOL) fclose(fSOL);

	rinex_file_reader.close_observation_rinex_files();
#endif
	return 1;
}

void gga2kml(const char *ubloxfilename)
{
	//-------------------------------------------------------------------------------
	/* need ublox ubx file and extract NMEA GGA, and store to a new file and generate the kml file */
	FILE *fGGA = NULL;
	FILE *fKML = NULL;
	FILE *ftemp = NULL;
	char buffer[1024] = { 0 };
	int lineIndex = 0;
	int numofread = 0;
	GGA_t gga = { 0 };

	char fileName[255] = { 0 };
	char outfilename[255] = { 0 };

	char kmlfname[255] = { 0 };
	char *newggafname[255] = { 0 };

	const char * result = strrchr(ubloxfilename, '\\');
	if (result != NULL)
		strncpy(fileName, result + 1, strlen(result));
	else
		strncpy(fileName, ubloxfilename, strlen(ubloxfilename));
	char *result1 = strrchr(fileName, '.');
	if (result1 != NULL) result1[0] = '\0';


	fGGA = fopen(ubloxfilename, "rb"); if (fGGA == NULL) return;

	sprintf(outfilename, "%s_gga.gga", fileName); ftemp = fopen(outfilename, "w");
	sprintf(outfilename, "%s_kml.kml", fileName); fKML = fopen(outfilename, "w");

	print_kml_heder(fKML);

	numofread = 0;
	while (true)
	{
		char key = ' ';
		size_t n = fread(&key, sizeof(char), 1, fGGA); if (n != 1) break;
		if (numofread == 0)
		{
			if (key == '$')
			{
				memset(buffer, 0, sizeof(buffer));
				numofread = 0;
				buffer[numofread] = key;
				++numofread;
			}
		}
		else
		{
			if (key == '\n' || key == '\r' || key == '$')
			{
				if (strstr(buffer, "GGA") != NULL)
				{

					if (ftemp) {
						fprintf(ftemp, "%s\n", buffer);
					}
					print_kml_gga(fKML, buffer);
				}
				memset(buffer, 0, sizeof(buffer));
				numofread = 0;
				if (key == '$')
				{
					buffer[numofread] = key;
					++numofread;
				}
			}
			else
			{
				buffer[numofread] = key;
				++numofread;
			}
		}
	}
	print_kml_eof(fKML);
	if (ftemp) fclose(ftemp);
	if (fGGA) fclose(fGGA);
	if (fKML) fclose(fKML);
	return;
}

void gga_diff(const char *fname1, const char *fname2)
{
	FILE *fGGA[2] = { NULL };
	FILE *fDiff = NULL;
	fGGA[0] = fopen(fname1, "r");
	fGGA[1] = fopen(fname2, "r");
	GGA_t gga1 = { 0 };
	GGA_t gga2 = { 0 };
	char buffer1[255] = { 0 };
	char buffer2[255] = { 0 };
    double leap_second = 18.0;

	if (fGGA[0] == NULL || fGGA[1] == NULL)
	{
		if (fGGA[0] != NULL) fclose(fGGA[0]);
		if (fGGA[1] != NULL) fclose(fGGA[1]);
		return;
	}

	const char * result = strrchr(fname1, '\\');
	if (result != NULL)
		strncpy(buffer1, result + 1, strlen(result));
	else
		strncpy(buffer1, fname1, strlen(fname1));
	char *result1 = strrchr(buffer1, '.');
	if (result1 != NULL) result1[0] = '\0';

	result = strrchr(fname2, '\\');
	if (result != NULL)
		strncpy(buffer2, result + 1, strlen(result));
	else
		strncpy(buffer2, fname2, strlen(fname2));
	result1 = strrchr(buffer2, '.');
	if (result1 != NULL) result1[0] = '\0';

	char filename[255] = { 0 };

	sprintf(filename, "%s--%s.csv", buffer1, buffer2);

	fDiff = fopen(filename, "w");

	int numofread = 0;

	double C_en[3][3] = { 0 };
	double startXYZ[3] = { 0 };
	double startBLH[3] = { 0 };

	numofread = 0;
	while (!feof(fGGA[0]))
	{
		char key = ' ';
		size_t n = fread(&key, sizeof(char), 1, fGGA[0]); if (n != 1) break;
		if (numofread == 0)
		{
			if (key == '$')
			{
				memset(buffer1, 0, sizeof(buffer1));
				numofread = 0;
				buffer1[numofread] = key;
				++numofread;
			}
			continue;
		}
		else
		{
			if (key == '\n' || key == '\r' || key == '$')
			{
				if (strstr(buffer1, "GGA") != NULL)
				{
					gga1 = { 0 };
					ParseGGA(buffer1, &gga1);
                    // cobvert to UTC if ublox gga is used.for other gga, please confirm time type.
                    gga1.time = gga1.time + leap_second;
				}
				memset(buffer1, 0, sizeof(buffer1));
				numofread = 0;
				if (key == '$')
				{
					buffer1[numofread] = key;
					++numofread;
				}
			}
			else
			{
				buffer1[numofread] = key;
				++numofread;
				continue;
			}
		}
		if (gga1.time == 0.0) continue;
		if (startXYZ[0] == 0.0 || startXYZ[1] == 0.0 || startXYZ[2] == 0.0)
		{
			memcpy(startBLH, gga1.blh, sizeof(double) * 3);
			blh2C_en(startBLH, C_en);
			pos2ecef(startBLH, startXYZ);
		}
		while (!feof(fGGA[1]))
		{
			if (gga2.time < gga1.time - 0.01)
			{
				memset(buffer2, 0, sizeof(buffer2));
				fgets(buffer2, sizeof(buffer2), fGGA[1]);
				if (!ParseGGA(buffer2, &gga2)) continue;
			}
			if (gga2.time >= gga1.time - 0.01)
				break;
		}
		if (fabs(gga2.time - gga1.time) < 0.01)
		{
			/* print */
			pos2ecef(gga1.blh, gga1.xyz);
			pos2ecef(gga2.blh, gga2.xyz);
			double xyz1[3] = { gga1.xyz[0] - startXYZ[0], gga1.xyz[1] - startXYZ[1], gga1.xyz[2] - startXYZ[2] };
			double xyz2[3] = { gga2.xyz[0] - startXYZ[0], gga2.xyz[1] - startXYZ[1], gga2.xyz[2] - startXYZ[2] };
			double ned1[3] = { 0.0 };
			double ned2[3] = { 0.0 };
			xyz2ned(C_en, xyz1, NULL, ned1, NULL);
			xyz2ned(C_en, xyz2, NULL, ned2, NULL);
			if (fDiff != NULL)
				fprintf(fDiff, "%10.3f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%i,%i\n", gga1.time, ned1[0], ned1[1], ned1[2], ned2[0] - ned1[0], ned2[1] - ned1[1], ned2[2] - ned1[2], gga1.solType, gga2.solType);
		}
	}

	if (fGGA[0] != NULL) fclose(fGGA[0]);
	if (fGGA[1] != NULL) fclose(fGGA[1]);

	if (fDiff != NULL) fclose(fDiff);

	return;
}


int batch(const char *fname)
{
	FILE *fINI = fopen(fname, "r");
	char buffer[255];
	char fname1[255] = { 0 };
	char fname2[255] = { 0 };
	char fname3[255] = { 0 };
	char inp_dir[255] = { 0 };
	char out_dir[255] = { 0 };
	int year = 0, doy = 0, line = 0, type = 0, isPrint = 0;
	if (fINI == NULL) fclose(fINI);

	while (!feof(fINI))
	{
		memset(buffer, 0, sizeof(buffer));
		fgets(buffer, sizeof(buffer), fINI);
		if (strlen(buffer) < 2) continue;
		if (buffer[0] == '#') continue;
		memset(fname1, 0, sizeof(fname1));
		memset(fname2, 0, sizeof(fname2));
		memset(fname3, 0, sizeof(fname3));
		year = 0;
		doy = 0;
		type = 0;
		sscanf(buffer, "%i", &type);
		if (type==0) /* RTK data process */
		{
			strncpy(fname1, inp_dir, strlen(inp_dir));
			strncpy(fname2, inp_dir, strlen(inp_dir));
			sscanf(buffer, "%i,%[^\,],%[^\,],%i,%i,%i", &type, fname1+ strlen(inp_dir), fname2+ strlen(inp_dir), &year, &doy, &isPrint);
#ifndef _USE_PPP_

			rtk_main(fname1, fname2, year, doy, isPrint);
#endif
		}
		else if (type == 1) /* PPP data process */
		{
			strncpy(fname1, inp_dir, strlen(inp_dir));
			strncpy(fname2, inp_dir, strlen(inp_dir));
			strncpy(fname3, inp_dir, strlen(inp_dir));
			sscanf(buffer, "%i,%[^\,],%[^\,],%[^\,],%i,%i", &type, fname1+ strlen(inp_dir), fname2+ strlen(inp_dir), fname3+ strlen(inp_dir), &year, &doy);
#ifdef _USE_PPP_
			ppp_main1(fname1, fname2, fname3, year, doy);
//            ppp_main2(fname1, fname2, year, doy);
#endif
		}
		else if (type == 2)
		{
			/* read ublox log files */
			strncpy(fname1, inp_dir, strlen(inp_dir));
			sscanf(buffer, "%i,%[^\,]", &type, fname1 + strlen(inp_dir));
			char *temp = strchr(fname1, '\n');
			if (temp != NULL) temp[0] = '\0';
			gga2kml(fname1);
		}
		else if (type == 3)
		{
			/* compare two GGA files */
			strncpy(fname1, inp_dir, strlen(inp_dir));
			strncpy(fname2, inp_dir, strlen(inp_dir));
			sscanf(buffer, "%i,%[^\,],%[^\,]", &type, fname1 + strlen(inp_dir), fname2 + strlen(inp_dir));
			char *temp = strchr(fname1, '\n');
			if (temp != NULL) temp[0] = '\0';
			temp = strchr(fname2, '\n');
			if (temp != NULL) temp[0] = '\0';
			gga_diff(fname1, fname2);
		}
		else if (type == 4)
		{
			/* input directory, effective after this command */
			sscanf(buffer, "%i,%[^\,]", &type, inp_dir);
			char *temp = strchr(inp_dir, '\n');
			if (temp != NULL) temp[0] = '\0';
			if (strlen(inp_dir) > 0)
			{
				if (inp_dir[strlen(inp_dir) - 1] != '\\')
				{
					inp_dir[strlen(inp_dir)] = '\\';
				}
			}
		}
		else if (type == 6)
		{
			/* rtk data process with rinex input : rovfname, reffnane, navfname, year, doy, isPrint */
			strncpy(fname1, inp_dir, strlen(inp_dir));
			strncpy(fname2, inp_dir, strlen(inp_dir));
			strncpy(fname3, inp_dir, strlen(inp_dir));
			sscanf(buffer, "%i,%[^\,],%[^\,],%[^\,],%i,%i,%i", &type, fname1 + strlen(inp_dir), fname2 + strlen(inp_dir), fname3 + strlen(inp_dir), &year, &doy, &isPrint);
#ifndef _USE_PPP_

			rtk_main_rinex(fname1, fname2, fname3, year, doy, isPrint);
#endif
		}
		++line;
	}
	if (fINI != NULL) fclose(fINI);
	return 0;
}



int main()
{
	OpenLogFile();
	batch("data.ini");
	CloseLogFile();
	system("pause");
	return 0;
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
