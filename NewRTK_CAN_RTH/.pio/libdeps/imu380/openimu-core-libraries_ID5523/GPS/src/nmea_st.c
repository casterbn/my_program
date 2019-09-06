#include "rtcm.h"
#include <string.h>

typedef struct gps_ephemeris_s
{
	unsigned int week : 16;
	unsigned int toe : 16;
	unsigned int toc : 16;
	unsigned int iode1 : 8;
	unsigned int iode2 : 8;
	unsigned int iodc : 10;
	unsigned int i_dot : 14;
	unsigned int spare1 : 8;
	unsigned int omega_dot : 24;
	unsigned int reserved1 : 2; // must be 0
	unsigned int reserved2 : 6; // must be 0
	unsigned int crs : 16;
	unsigned int crc : 16;
	unsigned int cus : 16;
	unsigned int cuc : 16;
	unsigned int cis : 16;
	unsigned int cic : 16;
	unsigned int motion_difference : 16;
	unsigned int reserved3 : 10; // must be 0
	unsigned int spare3 : 6;
	unsigned int inclination : 32;
	unsigned int eccentricity : 32;
	unsigned int root_a : 32;
	unsigned int mean_anomaly : 32;
	unsigned int omega_zero : 32;
	unsigned int perigee : 32;
	unsigned int time_group_delay : 8;
	unsigned int af2 : 8;
	unsigned int af1 : 16;
	unsigned int af0 : 22;
	unsigned int reserved4 : 1; // must be 1
	unsigned int reserved5 : 1; // must be 1
	unsigned int reserved6 : 1; // must be 1
	unsigned int available : 1;
	unsigned int health : 1;
	unsigned int reserved7 : 1; // must be 0
	unsigned int accuracy : 4;
} gps_ephemeris_t;

typedef struct glonass_ephemris_s
{
	unsigned int week : 16;
	unsigned int toe : 16;
	unsigned int toe_lsb : 4;
	unsigned int NA : 11;
	unsigned int tb : 7;
	unsigned int M : 2;
	unsigned int P1 : 2;
	unsigned int P3 : 1;
	unsigned int P2 : 1;
	unsigned int P4 : 1;
	unsigned int KP : 2;
	unsigned int spare0 : 1;
	unsigned int xn : 27;
	unsigned int xn_dot_dot : 5;
	unsigned int xn_dot : 24;
	unsigned int n : 5;
	unsigned int Bn : 3;
	unsigned int yn : 27;
	unsigned int yn_dot_dot : 5;
	unsigned int yn_dot : 24;
	unsigned int delta_tau_n : 5;
	unsigned int spare4 : 3;
	unsigned int zn : 27;
	unsigned int zn_dot_dot : 5;
	unsigned int zn_dot : 24;
	unsigned int reserved1 : 2; // must be 0
	unsigned int reserved2 : 6; // must be 0
	unsigned int gamma_n : 11;
	unsigned int E_n : 5;
	unsigned int freq_id : 4;
	unsigned int spare3 : 12;
	unsigned int tau_n : 22;
	unsigned int reserved3 : 10; // must be 0
	unsigned int tau_c : 32;
	unsigned int tau_GPS : 22;
	unsigned int spare5 : 10;
	unsigned int NT : 11;
	unsigned int N4 : 5;
	unsigned int tk : 12;
	unsigned int FT : 4;
	unsigned int spare7 : 32;
	unsigned int m_available : 5;
	unsigned int nvm_reliable : 1;
	unsigned int spare8 : 26;
	unsigned int spare9 : 25;
	unsigned int available : 1;
	unsigned int health : 1;
	unsigned int reserved4 : 1; // must be 0
	unsigned int spare10 : 4;
} glonass_ephemeris_t;

typedef struct beidou_ephemeris_s
{
	unsigned int inclination : 32;
	unsigned int eccentricity : 32;
	unsigned int root_a : 32;
	unsigned int mean_anomaly : 32;
	unsigned int omega_zero : 32;
	unsigned int perigee : 32;
	unsigned int toe : 17;
	unsigned int time_group_delay : 10;
	unsigned int aode : 5;
	unsigned int omega_dot : 24;
	unsigned int A0 : 8;
	unsigned int af0 : 24;
	unsigned int A1 : 8;
	unsigned int sow : 20;
	unsigned int af2 : 11;
	unsigned int is_geo : 1;
	unsigned int af1 : 22;
	unsigned int subframe_avail : 10;
	unsigned int motion_difference : 16;
	unsigned int A2 : 8;
	unsigned int A3 : 8;
	unsigned int crs : 18;
	unsigned int B2 : 8;
	unsigned int urai : 4;
	unsigned int reserved1 : 2;
	unsigned int crc : 18;
	unsigned int B3 : 8;
	unsigned int aodc : 5;
	unsigned int spare0 : 1;
	unsigned int cus : 18;
	unsigned int i_dot : 14;
	unsigned int cuc : 18;
	unsigned int B0 : 8;
	unsigned int spare1 : 6;
	unsigned int cis : 18;
	unsigned int B1 : 8;
	unsigned int reserved2 : 6;
	unsigned int cic : 18;
	unsigned int nvm_reliable : 1;
	unsigned int reserved3 : 1;
	unsigned int reserved4 : 10;
	unsigned int spare4 : 2;
	unsigned int toc : 17;
	unsigned int week : 13;
	unsigned int available : 1;
	unsigned int health : 1;
} beidou_ephemeris_t;

typedef struct galileo_ephemeris_s
{
	unsigned int week : 16;
	unsigned int toe : 14;
	unsigned int reserved1 : 2; // must be 0
	unsigned int toc : 14;
	unsigned int iod_nav : 10;
	unsigned int SISA : 8;
	unsigned int reserved2 : 10; // must be 0
	unsigned int BGD_E1_E5a : 10;
	unsigned int BGD_E1_E5b : 10;
	unsigned int E1BHS : 2;
	unsigned int inclination : 32;
	unsigned int eccentricity : 32;
	unsigned int root_a : 32;
	unsigned int mean_anomaly : 32;
	unsigned int omega_zero : 32;
	unsigned int perigee : 32;
	unsigned int i_dot : 14;
	unsigned int available : 1;
	unsigned int health : 1;
	unsigned int motion_difference : 16;
	unsigned int crs : 16;
	unsigned int crc : 16;
	unsigned int cus : 16;
	unsigned int cuc : 16;
	unsigned int cis : 16;
	unsigned int cic : 16;
	unsigned int omega_dot : 24;
	unsigned int SVID : 6;
	unsigned int E1BDVS : 1;
	unsigned int reserved3 : 1; // must be 0
	unsigned int af2 : 6;
	unsigned int af1 : 21;
	unsigned int word_available : 5;
	unsigned int af0 : 31;
	unsigned int spare0 : 1;
	unsigned int reserved4 : 6; // must be 0
	unsigned int spare1 : 26;
} galileo_ephemeris_t;

typedef struct {
	unsigned char TG_TS_version;
	unsigned char TG_TS_front_end;
	unsigned char clock_steering;
	int week_n;
	double tow;
	int sats_tracked;
	unsigned int cpu_time;
	unsigned int time_validity;
	double clock_drift;

	/* V1 legacy */
	int year;
	int month;
	int day;
	int hour;
	int mins;
	double secs;
	double tow_delta;
	unsigned int cpu_time_p;
	double req_tow_delta;
	unsigned int req_cpu_time;
} TG_data_t;

typedef struct {
	int dsp_dat;
	int Sat_ID;
	double PsR;
	double Freq;
	double cp;
	int dsp_glags;
	int CN0;
	int Ttim;
	int codeNoise;
	int phaseNoise;
	int cycle_slip_cnt;
	int Glonass_slot;
	int Satdat;
	double Satx;
	double Saty;
	double Satz;
	double Velx;
	double Vely;
	double Velz;
	double Src;
	double tgd;
	double iono;
	double tropo;
	double rrc;
	int ura;
	int Difdat;
	double Drc;
	double Drrc;
	int dX;
	int dY;
	int dZ;
	int dC;
} TS_data_t;

typedef struct {
	// double TalkerID;
	double Timestamp;
	double Lat;
	char NS;
	double Lon;
	char EW;
	int GPSQual;
	int Sats;
	double HDOP;
	double Alt;
	char AltVal;
	double GeoSep;
	char GeoVal;

} GGA_data_t;


int nmea_read_TG_msg(char *line, TG_data_t *tg_data)
{
	int num_fields_tg;
	int kf_config;
	double clock_drift;
	if (strncmp(line, "$PSTMTG,", strlen("$PSTMTG,")))
	{
		return -1;
	}
	num_fields_tg = sscanf(line, "$PSTMTG,%d,%lf,%d,%u,%d,%lf,%x,",
		&tg_data->week_n, &tg_data->tow, &tg_data->sats_tracked, &tg_data->cpu_time, &tg_data->time_validity, &clock_drift, &kf_config);
	if (num_fields_tg != 7)
	{
		return -1;
	}
	tg_data->TG_TS_version = (kf_config >> 12) & 0x7;
	tg_data->TG_TS_front_end = (kf_config >> 8) & 0x7;
	tg_data->clock_steering = (kf_config >> 15) & 0x1;
	if (tg_data->TG_TS_version == 1)
	{
		num_fields_tg = sscanf(line,
			"$PSTMTG,%d,%lf,%d,%u,%d,%lf,%x,%lf,%lf,%u,%u,",
			&tg_data->week_n, &tg_data->tow, &tg_data->sats_tracked, &tg_data->cpu_time, &tg_data->time_validity, &tg_data->clock_drift, &kf_config,
			&tg_data->tow_delta, &tg_data->req_tow_delta, &tg_data->cpu_time_p,
			&tg_data->req_cpu_time);
	}
	else
	{
		return -1;
	}

	return 0;
}

int nmea_read_TS_msg(char *line, TS_data_t *ts_data, int version)
{
	int num_fields;
	int pred_available;
	int pred_age_h;

	if (strncmp(line, "$PSTMTS,", strlen("$PSTMTS,")))
	{
		return -1;
	}
	num_fields = sscanf(line, "$PSTMTS,%d,%d", &ts_data->dsp_dat, &ts_data->Sat_ID);
	if (num_fields != 2)
	{
		return -1;
	}
	// if (ts_data->dsp_avail == 9)
	{
		num_fields = sscanf(line,
			"$PSTMTS,%d,%d,%lf,%lf,%lf,%d,%d,%d,%d,%d,%d,%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%d,%lf,%lf,%d,%d,%d,%d",
			&ts_data->dsp_dat,
			&ts_data->Sat_ID,
			&ts_data->PsR,
			&ts_data->Freq,
			&ts_data->cp,
			&ts_data->dsp_glags,
			&ts_data->CN0,
			&ts_data->Ttim,
			&ts_data->codeNoise,
			&ts_data->phaseNoise,
			&ts_data->cycle_slip_cnt,
			&ts_data->Glonass_slot,
			&ts_data->Satdat,
			&ts_data->Satx,
			&ts_data->Saty,
			&ts_data->Satz,
			&ts_data->Velx,
			&ts_data->Vely,
			&ts_data->Velz,
			&ts_data->Src,
			&ts_data->tgd,
			&ts_data->iono,
			&ts_data->tropo,
			&ts_data->rrc,
			&ts_data->ura,
			&ts_data->Difdat,
			&ts_data->Drc,
			&ts_data->Drrc,
			&ts_data->dX,
			&ts_data->dY,
			&ts_data->dZ,
			&ts_data->dC
		);
		if ((num_fields != 32))
		{
			return -1;
		}
	}

	return 0;
}

int decode_nmea_GGA(char *buffer, obs_t *obs)
{
	int ret = 0;
	/* decode the GGA message */
	int num_fields;
	GGA_data_t gga_data;
//	             sscanf(buffer, "$PSTMEPHEM,%d,%d,%s", &sat, &len, hex_buf);
	num_fields = sscanf(buffer, "$GPGGA,%lf,%lf,%c,%lf,%c,%d,%d,%lf,%lf,%c,%lf,%c", &gga_data.Timestamp,&gga_data.Lat,&gga_data.NS,&gga_data.Lon,
	&gga_data.EW,&gga_data.GPSQual,&gga_data.Sats,&gga_data.HDOP,&gga_data.Alt,&gga_data.AltVal,
	&gga_data.GeoSep,&gga_data.GeoVal
	);

	if(num_fields != 13)
	{
		return -1;

	}

	return ret;
}

/* satellite ID for ST */
/* GPS L1, [  1, 35], GPS L2 , +400, GPS L5,+500 */
/* GLO L1, [ 65, 92], GLO L2 , +400 */
/* GAL L1, [ 65, 92], GAL E5a, +600, GAL E5b, +650 */

int decode_nmea_PSTMTG(unsigned char *buffer, obs_t *obs)
{
	int ret = 0;
	/* decode the PSTMTG message 
	int nmea_read_TG_msg(char *line, TG_data_t *tg_data)
	*/
	TG_data_t tg_data; // DW: TO remove later
	nmea_read_TG_msg(buffer,&tg_data);

	/* conver to obs */
	obs->n = obs->obsflag = 0;
	obs->time = gpst2time(tg_data.week_n, tg_data.tow);
	memset(&obs->data, 0, MAXOBS * sizeof(obsd_t));

	return ret;
}

int decode_nmea_PSTMTS(unsigned char *buffer, obs_t *obs)
{
	int ret = 0;
	/* decode the $PSTMTS message 
	nmea_read_TS_msg(char *line, TS_data_t *ts_data, int version)
	*/

	int is_dual_freq =-1;
	int pre_dspsat = 0;
	if (obs->n < MAXOBS) {
		TS_data_t data_TS; //DW: TO remove later
		int version = 0;

		nmea_read_TS_msg(buffer, &data_TS, version);

		/* convert to obs */
		unsigned char dspsat = data_TS.dsp_dat;
		obsd_t *ch = obs->data + obs->n;

		if (3 == dspsat) {
			is_dual_freq = 1;
		}
		else if (2 == dspsat) {
			is_dual_freq = 0;
		}

		if (pre_dspsat != dspsat && pre_dspsat>0)
		{
			obs->n++;
		}
		pre_dspsat = dspsat;
		
		if (is_dual_freq < NFREQ && is_dual_freq >= 0)
		{
			ch->P[is_dual_freq] = data_TS.PsR;
			ch->D[is_dual_freq] = data_TS.Freq;
			ch->L[is_dual_freq] = data_TS.cp;
			ch->SNR[is_dual_freq] = data_TS.CN0;
		}
		
		if (0 == is_dual_freq) {
			unsigned short satid = data_TS.Sat_ID;
			if (2 == dspsat) {
				if (satid > 300 && satid < 400) {
					satid -= 300;
				}
				else if (satid > 400 && satid < 500) {
					satid -= 400;
				}
				else if (satid > 500 && satid < 600) {
					satid -= 500;
				}
				else if (satid > 600 && satid < 700) {
					satid -= 600;
				}
				else if (satid > 140 && satid < 171) {
					satid -= 140;
				}
				
			}
			ch->sat = satid;

			ch->svh = data_TS.Satdat;
#if 0 /* do not store the satellite information */
			ch->vec.satpvt[0] = data_TS.Satx;
			ch->vec.satpvt[1] = data_TS.Saty;
			ch->vec.satpvt[2] = data_TS.Satz;
			ch->vec.satpvt[3] = data_TS.Velx;
			ch->vec.satpvt[4] = data_TS.Vely;
			ch->vec.satpvt[5] = data_TS.Velz;
#endif

		}


	}

	return ret;
}



int decode_nmea_PSTMEPHEM(unsigned char *buffer, nav_t *nav)
{
	int ret = 0, sat = 0, len = 0, i = 0, prn = 0;

	gps_ephemeris_t eph_gps = { 0 };
	galileo_ephemeris_t eph_gal = { 0 };
	glonass_ephemeris_t eph_glo = { 0 };
	beidou_ephemeris_t eph_bds = { 0 };

	unsigned char hex_buf[1024] = { 0 }, payload[1024] = { 0 };

	/* decode the $PSTMEPHEM message */
	sscanf(buffer, "$PSTMEPHEM,%d,%d,%s", &sat, &len, hex_buf);
	for (i = 0; i < len; i++)
	{
		sscanf(&hex_buf[i * 2], "%02x", &payload[i]);
	}

	if (65 <= sat && sat <= 92)
	{
		/* glonass */
		prn = sat - 64;
		memcpy(&eph_glo, payload, sizeof(glonass_ephemeris_t));
	}
	if (141 <= sat && sat <= 170)
	{
		prn = sat - 140;
		memcpy(&eph_bds, payload, sizeof(beidou_ephemeris_t));
	}
	
	memcpy(&eph_gps, payload, sizeof(gps_ephemeris_t));
	memcpy(&eph_gal, payload, sizeof(galileo_ephemeris_t));
	
	return ret;
}

/* decode NMEA  -------------------------------------------------*/

extern int decode_nmea(rtcm_t *rtcm, obs_t *obs, nav_t *nav)
{
	int ret = 0;

   	int i = 0;
	int result;
	int result1;	
	/*check code*/
    result = rtcm->buff[1];
    for(i = 2; rtcm->buff[i] != '*'; i++)
    {
        result ^= rtcm->buff[i];
    }
	i++;
	sscanf(&rtcm->buff[i], "%02x", &result1);
	if (result != result1)
	{
		return -1;
	}
	



	if (!strncmp(rtcm->buff, "$PSTMTG", 7))
	{
		ret = decode_nmea_PSTMTG(rtcm->buff, obs);
	}
	else if (!strncmp(rtcm->buff, "$PSTMTS", 7))
	{
		ret = decode_nmea_PSTMTS(rtcm->buff, obs);
		rtcm->last_nmea_ts = 1;
	}
	else if (strstr(rtcm->buff, "$G")!=NULL && strstr(rtcm->buff, "GGA") != NULL)
	{
		/* need to consider $GXGGA, GPGGA, GNGGA, etc. */
		ret = decode_nmea_GGA(rtcm->buff, obs);
	}
	else if (!strncmp(rtcm->buff, "$PSTMEPHEM", 10))
	{
		ret = decode_nmea_PSTMEPHEM(rtcm->buff, nav);
	}

	if (rtcm->last_nmea_ts) {
		if (strncmp(rtcm->buff, "$PSTMTS",7)) {
			rtcm->last_nmea_ts = 0;
			ret = 1;
		}
	}

	return ret;
}