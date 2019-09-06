/** ***************************************************************************
 * @file gps.h GPS Driver for Inertial/GPS NAV.
 * @author Dong An
 * @date   2009-04-10 23:20:59Z
 * @ver 8719
 * @copyright (c) 2013, 2014 All Rights Reserved.
 * @section LICENSE
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * @brief This is a generalized GPS interface, taken loosely from the DMU440
 * project possibly implemented using the Origin ORG4475 GPS module (or NovAtel
 * or uBlox) the GPS may communicated via SPI or UART, that is passed in on init
 *  03.2007 DA  Cleaned up, Doxygenized, and finalized for NAV440 release.
 *****************************************************************************/
#ifndef GPS_H
#define GPS_H

#include <stdint.h>

#include "gnss_nav_utils.h"

// FIXME ECW: implement GpsWhoAmI and GpsSelfTest
uint8_t  GpsWhoAmI(uint32_t *whoami); /// returns true if value is as expected
uint8_t  GpsSelfTest();

// debugging stream out the debug port
#define GPS_NO_STREAM         0
#define GPS_NMEA_DEBUG_STREAM 1

#ifndef STREAM_GPS
//#define STREAM_GPS GPS_NO_STREAM
#define STREAM_GPS GPS_NMEA_DEBUG_STREAM
#endif

#define NMEA_SYNC_1  0x00244750 // $GP
#define NMEA_SYNC_2  0x0024474E // $GN


#define MAX_GPS_NUMBER_SAT       (32)

typedef struct gps_eph_channel_tag {
	bool_t           is_updated;            // only for logging purpose
	int              prn;
	double           tow;
	int              week;
	int              code;
	float            ura;
	uint32_t         health;
	uint32_t         iodc;
	unsigned int     flag_l2p;
	double           fit_interval;
	double           tgd;
	double           toc;
	double           af2;
	double           af1;
	double           af0;
	uint32_t         iode;
	double           crs;
	double           deltan;
	double           m0;
	double           cuc;
	double           ecc;
	double           cus;
	double           sqrta;
	double           toe;
	double           cic;
	double           omega0;
	double           cis;
	double           i0;
	double           crc;
	double           omega;
	double           omegadot;
	double           idot;
} gps_eph_channel_t;


typedef struct gps_eph_tag {
	gps_eph_channel_t svid[MAX_GPS_NUMBER_SAT];
} gps_eph_t;


void gps_time_rollover_correct(double *tk);
bool_t get_gps_satclockcorr(const double tc, const gps_eph_channel_t *eph, double *clock_bias, double *clock_drift);
double get_gps_saterelativecorr(const double tk, const gps_eph_channel_t *eph);
double get_gps_groupdelay(const gps_eph_channel_t* eph);

bool_t 
get_gps_satposvel(const double time, const gps_eph_channel_t *eph, const double *user_pos, const double *user_vel, double *sat_pos, double *sat_vel);

#endif /* GPS_H */