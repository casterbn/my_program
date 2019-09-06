#ifndef _GLONASS_H_
#define _GLONASS_H_

#define MAX_GLO_NUMBER_SAT          (32)

/* GLONASS broadcast ephemeris type */
typedef struct {        
	bool              is_updated;    // only for logging purpose
	int               prn;           // satellite number 
	int               iode;          // IODE (0-6 bit of tb field) 
	int               freq;          // satellite frequency number 
	int               health;        // satellite health
	int               sva;           // accuracy
	int               age;           // age of operation 
	double            toe;           // epoch of ephemeris (gps time) 
	double            tof;           // message frame time (gps time) 
	double            pos[3];        // satellite position (ecef) (m) 
	double            vel[3];        // satellite velocity (ecef) (m/s) 
	double            acc[3];        // satellite acceleration (ecef) (m/s^2) 
	double            taun;          // SV clock bias (s)
	double            gamn;          // relative freq bias 
	double            dtaun;         // delay between L1 and L2 (s) 
} glo_eph_channel_t;


typedef struct {
	glo_eph_channel_t svid[MAX_GLO_NUMBER_SAT];
} glo_eph_t;



#endif