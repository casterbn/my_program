#ifndef _RTCM_H_
#define _RTCM_H_

#include <time.h>
#include <stdint.h>
#include "main.h"
#ifdef __cplusplus
extern "C"
{
#endif

/* by Dr. Yudan Yi */

/* turn on this define by only use PPP mode, this will not declare the observation memory for the base station */
//#define _USE_PPP_
/* by Yihe Li*/
#define _USE_UC_

#define ARM_MCU
#define SECONDS_IN_WEEK (604800)

#ifndef _DEBUG
//#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
//#pragma GCC diagnostic ignored "-Wunused-variable"
//#pragma GCC diagnostic ignored "-Wunused-function"
//#pragma GCC diagnostic ignored "-Wunused-const-variable="
#endif

/* disable the define for embeded */
//#define _TRACE_

/*-----------------------------------------------------------*/
/* from rtklib to decode RTCM3 */
#define CLIGHT 299792458.0 /* speed of light (m/s) */
#ifndef PI
#define PI 3.1415926535897932 /* pi */
#endif
#define OMGE 7.2921151467E-5           /* earth angular velocity (IS-GPS) (rad/s) */
#define RE_WGS84 6378137.0             /* earth semimajor axis (WGS84) (m) */
#define FE_WGS84 (1.0 / 298.257223563) /* earth flattening (WGS84) */

#define NFREQ 2
#define NEXOBS 0
#define ENAGLO

#define MINPRNGPS 1                         /* min satellite PRN number of GPS */
#define MAXPRNGPS 40                        /* max satellite PRN number of GPS */
#define NSATGPS (MAXPRNGPS - MINPRNGPS + 1) /* number of GPS satellites */
#define NSYSGPS 1

#ifdef ENAGLO
#define MINPRNGLO 1                         /* min satellite slot number of GLONASS */
#define MAXPRNGLO 30                        /* max satellite slot number of GLONASS */
#define NSATGLO (MAXPRNGLO - MINPRNGLO + 1) /* number of GLONASS satellites */
#define NSYSGLO 1
#else
#define MINPRNGLO 0
#define MAXPRNGLO 0
#define NSATGLO 0
#define NSYSGLO 0
#endif

#define MINPRNGAL 1                         /* min satellite PRN number of Galileo */
#define MAXPRNGAL 40                        /* max satellite PRN number of Galileo */
#define NSATGAL (MAXPRNGAL - MINPRNGAL + 1) /* number of Galileo satellites */
#define NSYSGAL 1

#ifdef ENAQZS
#define MINPRNQZS 193                       /* min satellite PRN number of QZSS */
#define MAXPRNQZS 199                       /* max satellite PRN number of QZSS */
#define MINPRNQZS_S 183                     /* min satellite PRN number of QZSS SAIF */
#define MAXPRNQZS_S 189                     /* max satellite PRN number of QZSS SAIF */
#define NSATQZS (MAXPRNQZS - MINPRNQZS + 1) /* number of QZSS satellites */
#define NSYSQZS 1
#else
#define MINPRNQZS 0
#define MAXPRNQZS 0
#define MINPRNQZS_S 0
#define MAXPRNQZS_S 0
#define NSATQZS 0
#define NSYSQZS 0
#endif

#define MINPRNCMP 1                         /* min satellite sat number of BeiDou */
#define MAXPRNCMP 40                        /* max satellite sat number of BeiDou */
#define NSATCMP (MAXPRNCMP - MINPRNCMP + 1) /* number of BeiDou satellites */
#define NSYSCMP 1

#ifdef ENALEO
#define MINPRNLEO 1                         /* min satellite sat number of LEO */
#define MAXPRNLEO 10                        /* max satellite sat number of LEO */
#define NSATLEO (MAXPRNLEO - MINPRNLEO + 1) /* number of LEO satellites */
#define NSYSLEO 1
#else
#define MINPRNLEO 0
#define MAXPRNLEO 0
#define NSATLEO 0
#define NSYSLEO 0
#endif

#ifdef ENASBSSSS
#define MINPRNSBS 120                       /* min satellite PRN number of SBAS */
#define MAXPRNSBS 142                       /* max satellite PRN number of SBAS */
#define NSATSBS (MAXPRNSBS - MINPRNSBS + 1) /* number of SBAS satellites */
#else
#define MINPRNSBS 0
#define MAXPRNSBS 0
#define NSATSBS 0
#endif

#define NSYS (NSYSGPS + NSYSGLO + NSYSGAL + NSYSCMP) /* only use GPS, GLO, GAL, BDS */

#define MAXSAT (NSATGPS + NSATGLO + NSATGAL + NSATCMP)

#define _SYS_NONE_ 0x00 /* navigation system: none */
#define _SYS_GPS_ 0x01  /* navigation system: GPS */
#define _SYS_SBS_ 0x02  /* navigation system: SBAS */
#define _SYS_GLO_ 0x04  /* navigation system: GLONASS */
#define _SYS_GAL_ 0x08  /* navigation system: Galileo */
#define _SYS_QZS_ 0x10  /* navigation system: QZSS */
#define _SYS_BDS_ 0x20  /* navigation system: BeiDou */
#define _SYS_IRN_ 0x40  /* navigation system: IRNSS */
#define _SYS_LEO_ 0x80  /* navigation system: LEO */
#define _SYS_ALL_ 0xFF  /* navigation system: all */

#define CODE_NONE 0 /* obs code: none or unknown */
#define CODE_L1C 1  /* obs code: L1C/A,G1C/A,E1C (GPS,GLO,GAL,QZS,SBS) */
#define CODE_L1P 2  /* obs code: L1P,G1P    (GPS,GLO) */
#define CODE_L1W 3  /* obs code: L1 Z-track (GPS) */
#define CODE_L1Y 4  /* obs code: L1Y        (GPS) */
#define CODE_L1M 5  /* obs code: L1M        (GPS) */
#define CODE_L1N 6  /* obs code: L1codeless (GPS) */
#define CODE_L1S 7  /* obs code: L1C(D)     (GPS,QZS) */
#define CODE_L1L 8  /* obs code: L1C(P)     (GPS,QZS) */
#define CODE_L1E 9  /* (not used) */
#define CODE_L1A 10 /* obs code: E1A        (GAL) */
#define CODE_L1B 11 /* obs code: E1B        (GAL) */
#define CODE_L1X 12 /* obs code: E1B+C,L1C(D+P) (GAL,QZS) */
#define CODE_L1Z 13 /* obs code: E1A+B+C,L1SAIF (GAL,QZS) */
#define CODE_L2C 14 /* obs code: L2C/A,G1C/A (GPS,GLO) */
#define CODE_L2D 15 /* obs code: L2 L1C/A-(P2-P1) (GPS) */
#define CODE_L2S 16 /* obs code: L2C(M)     (GPS,QZS) */
#define CODE_L2L 17 /* obs code: L2C(L)     (GPS,QZS) */
#define CODE_L2X 18 /* obs code: L2C(M+L),B1I+Q (GPS,QZS,CMP) */
#define CODE_L2P 19 /* obs code: L2P,G2P    (GPS,GLO) */
#define CODE_L2W 20 /* obs code: L2 Z-track (GPS) */
#define CODE_L2Y 21 /* obs code: L2Y        (GPS) */
#define CODE_L2M 22 /* obs code: L2M        (GPS) */
#define CODE_L2N 23 /* obs code: L2codeless (GPS) */
#define CODE_L5I 24 /* obs code: L5/E5aI    (GPS,GAL,QZS,SBS) */
#define CODE_L5Q 25 /* obs code: L5/E5aQ    (GPS,GAL,QZS,SBS) */
#define CODE_L5X 26 /* obs code: L5/E5aI+Q/L5B+C (GPS,GAL,QZS,IRN,SBS) */
#define CODE_L7I 27 /* obs code: E5bI,B2I   (GAL,CMP) */
#define CODE_L7Q 28 /* obs code: E5bQ,B2Q   (GAL,CMP) */
#define CODE_L7X 29 /* obs code: E5bI+Q,B2I+Q (GAL,CMP) */
#define CODE_L6A 30 /* obs code: E6A        (GAL) */
#define CODE_L6B 31 /* obs code: E6B        (GAL) */
#define CODE_L6C 32 /* obs code: E6C        (GAL) */
#define CODE_L6X 33 /* obs code: E6B+C,LEXS+L,B3I+Q (GAL,QZS,CMP) */
#define CODE_L6Z 34 /* obs code: E6A+B+C    (GAL) */
#define CODE_L6S 35 /* obs code: LEXS       (QZS) */
#define CODE_L6L 36 /* obs code: LEXL       (QZS) */
#define CODE_L8I 37 /* obs code: E5(a+b)I   (GAL) */
#define CODE_L8Q 38 /* obs code: E5(a+b)Q   (GAL) */
#define CODE_L8X 39 /* obs code: E5(a+b)I+Q (GAL) */
#define CODE_L2I 40 /* obs code: B1I        (BDS) */
#define CODE_L2Q 41 /* obs code: B1Q        (BDS) */
#define CODE_L6I 42 /* obs code: B3I        (BDS) */
#define CODE_L6Q 43 /* obs code: B3Q        (BDS) */
#define CODE_L3I 44 /* obs code: G3I        (GLO) */
#define CODE_L3Q 45 /* obs code: G3Q        (GLO) */
#define CODE_L3X 46 /* obs code: G3I+Q      (GLO) */
#define CODE_L1I 47 /* obs code: B1I        (BDS) */
#define CODE_L1Q 48 /* obs code: B1Q        (BDS) */
#define CODE_L5A 49 /* obs code: L5A SPS    (IRN) */
#define CODE_L5B 50 /* obs code: L5B RS(D)  (IRN) */
#define CODE_L5C 51 /* obs code: L5C RS(P)  (IRN) */
#define CODE_L9A 52 /* obs code: SA SPS     (IRN) */
#define CODE_L9B 53 /* obs code: SB RS(D)   (IRN) */
#define CODE_L9C 54 /* obs code: SC RS(P)   (IRN) */
#define CODE_L9X 55 /* obs code: SB+C       (IRN) */
#define MAXCODE 55  /* max number of obs code */

#ifdef _USE_PPP_
/* YYD: complier error, need to check, now just comment out */
#ifndef _USE_UC_
#define MAXOBS 35
#define MAXEPH 30
#define MAXSSR 35
#define MAXEPH_R 15
#else
#define MAXOBS 24
#define MAXEPH 18
#define MAXSSR 24
#define MAXEPH_R 6
#define MAXIONOORDER 16
#define MAXIONODEGREE 16
#define NUMIONOLAYERS 4
#endif
#else
#define MAXOBS 35
#define MAXEPH 35
#define MAXEPH_R 30
#endif

    typedef struct
    {                /* time struct */
        time_t time; /* time (s) expressed by standard time_t */
        double sec;  /* fraction of second under 1 s */
    } gtime_t;

    typedef struct
    {                          /* GPS/QZS/GAL broadcast ephemeris type */
        unsigned char sat;     /* satellite number */
        int iode, iodc;        /* IODE,IODC */
        int sva;               /* SV accuracy (URA index) */
        int svh;               /* SV health (0:ok) */
        int week;              /* GPS/QZS: gps week, GAL: galileo week */
        int code;              /* GPS/QZS: code on L2, GAL/CMP: data sources */
        int flag;              /* GPS/QZS: L2 P data flag, CMP: nav type */
        gtime_t toe, toc, ttr; /* Toe,Toc,T_trans */
                               /* SV orbit parameters */
        double A, e, i0, OMG0, omg, M0, deln, OMGd, idot;
        double crc, crs, cuc, cus, cic, cis;
        double toes;       /* Toe (s) in week */
        double fit;        /* fit interval (h) */
        double f0, f1, f2; /* SV clock parameters (af0,af1,af2) */
        double tgd[4];     /* group delay parameters */
                           /* GPS/QZS:tgd[0]=TGD */
                           /* GAL    :tgd[0]=BGD E5a/E1,tgd[1]=BGD E5b/E1 */
                           /* CMP    :tgd[0]=BGD1,tgd[1]=BGD2 */
        double Adot, ndot; /* Adot,ndot for CNAV */
    } eph_t;

    typedef struct
    {                      /* GLONASS broadcast ephemeris type */
        unsigned char sat; /* satellite number */
        int iode;          /* IODE (0-6 bit of tb field) */
        int frq;           /* satellite frequency number */
        int svh, sva, age; /* satellite health, accuracy, age of operation */
        gtime_t toe;       /* epoch of epherides (gpst) */
        gtime_t tof;       /* message frame time (gpst) */
        double pos[3];     /* satellite position (ecef) (m) */
        double vel[3];     /* satellite velocity (ecef) (m/s) */
        double acc[3];     /* satellite acceleration (ecef) (m/s^2) */
        double taun, gamn; /* SV clock bias (s)/relative freq bias */
        double dtaun;      /* delay between L1 and L2 (s) */
    } geph_t;

    typedef struct
    { /* SSR correction type */
        unsigned char sat;
        gtime_t t0[6];   /* epoch time (GPST) {eph,clk,hrclk,ura,bias,pbias} */
        double udi[6];   /* SSR update interval (s) */
        int iod[6];      /* iod ssr {eph,clk,hrclk,ura,bias,pbias} */
        int iode;        /* issue of data */
        int iodcrc;      /* issue of data crc for beidou/sbas */
        int ura;         /* URA indicator */
        int refd;        /* sat ref datum (0:ITRF,1:regional) */
        double deph[3];  /* delta orbit {radial,along,cross} (m) */
        double ddeph[3]; /* dot delta orbit {radial,along,cross} (m/s) */
        double dclk[3];  /* delta clock {c0,c1,c2} (m,m/s,m/s^2) */
        double hrclk;    /* high-rate clock corection (m) */
        /* YYD: need to check the following vectors to store the minimum*/
        //float  cbias[MAXCODE]; /* code biases (m) */
        //double pbias[MAXCODE]; /* phase biases (m) */
        //float  stdpb[MAXCODE]; /* std-dev of phase biases (m) */
        double yaw_ang, yaw_rate; /* yaw angle and yaw rate (deg,deg/s) */
        unsigned char update;     /* update flag (0:no update,1:update) */
    } ssr_t;

    typedef struct
    {                    /* navigation data type */
        unsigned int n;  /* number of broadcast ephemeris */
        unsigned int ng; /* number of glonass ephemeris */
        unsigned int n_gps;
        unsigned int n_gal;
        unsigned int n_bds;
        unsigned int n_qzs;
        unsigned int ns;
        eph_t eph[MAXEPH];     /* GPS/QZS/GAL ephemeris */
        geph_t geph[MAXEPH_R]; /* GLONASS ephemeris */
#ifdef _USE_PPP_
        ssr_t ssr[MAXSSR]; /* output of ssr corrections */
#endif
        unsigned char ephsat;
    } nav_t;

    typedef struct
    {                              /* observation data record */
        unsigned char sat;         /* satellite/receiver number */
        unsigned char SNR[NFREQ];  /* signal strength (0.25 dBHz) */
        unsigned char LLI[NFREQ];  /* loss of lock indicator */
        unsigned char code[NFREQ]; /* code indicator (CODE_???) */
        double L[NFREQ];           /* observation data carrier-phase (cycle) */
        double P[NFREQ];           /* observation data pseudorange (m) */
        float D[NFREQ];            /* observation data doppler frequency (Hz) */

        /*unsigned char is_dual_freq; */ /* should not add the  */
        unsigned char svh;
    } obsd_t;

    typedef struct
    {                        /* observation data */
        unsigned int n;      /* number of obervation data/allocated */
        obsd_t data[MAXOBS]; /* observation data records */
        gtime_t time;
        double pos[6];         /* station position (ecef) (m) */
        unsigned char obsflag; /* obs data complete flag (1:ok,0:not complete) */
        unsigned int staid;    /* station id */
    } obs_t;

    typedef struct
    {                 /* RTCM control struct type */
        gtime_t time; /* message time */
        //gtime_t time_s;     /* message start time */
        //obs_t obs;          /* observation data (uncorrected) */
        char msmtype[6][128]; /* msm signal types */
        /* YYD: need to save space for this */
        /* YYD: cp is only used in RTCM2 (adjcp), lock is temp disabled */
        //double cp[MAXSAT][NFREQ+NEXOBS]; /* carrier-phase measurement */
        //unsigned short lock[MAXSAT][NFREQ+NEXOBS]; /* lock time */
        //unsigned short loss[MAXSAT][NFREQ+NEXOBS]; /* loss of lock count */
        //gtime_t lltime[MAXSAT][NFREQ+NEXOBS]; /* last lock time */
        unsigned int nbyte;       /* number of bytes in message buffer */
        unsigned int nbit;        /* number of bits in word buffer */
        unsigned int len;         /* message length (bytes) */
        unsigned int type;        /* last rtcm type */
        unsigned char buff[1200]; /* message buffer */
        unsigned char key;
        unsigned char last_nmea_ts;
    } rtcm_t;

#define MAXROV 1

#define MAXREF 1

#define MAXSTN (MAXROV + MAXREF)

    typedef struct
    {
        /* move the observation data struct out of rtcm definiton, to save more memory for PPP only mode */
        obs_t obs[MAXROV];
#ifndef _USE_PPP_
        obs_t obs_ref[MAXREF];
#endif
        rtcm_t rcv[MAXSTN];
        nav_t nav;
        double time;
        unsigned char gnss_data_flag;
    } gnss_rtcm_t;

    void trace(int level, const char *format, ...);
#ifdef QT_QML_DEBUG
    extern void OpenLogFile();
    extern void CloseLogFile();
#endif

    int input_rtcm3_data(rtcm_t *rtcm, unsigned char data, obs_t *obs, nav_t *nav);

    /* interface to GNSS db */
    int input_rtcm3(unsigned char data, unsigned int stnID, gnss_rtcm_t *gnss);

    int decode_nmea(rtcm_t *rtcm, obs_t *obs, nav_t *nav);

    /* glo frquent number function */
    void set_glo_frq(unsigned char prn, int frq);
    int get_glo_frq(unsigned char prn);

    void set_week_number(int week);
    int get_week_number();

    /* time function */
    gtime_t timeadd(gtime_t t, double sec);
    double timediff(gtime_t t1, gtime_t t2);
    gtime_t epoch2time(const double *ep);
    void time2epoch(gtime_t t, double *ep);
    gtime_t bdt2time(int week, double sec);
    double time2bdt(gtime_t t, int *week);
    double time2gpst(gtime_t t, int *week);
    gtime_t utc2gpst(gtime_t t);
    gtime_t gpst2utc(gtime_t t);
    gtime_t gpst2time(int week, double sec);
    gtime_t gpst2bdt(gtime_t t);
    gtime_t bdt2gpst(gtime_t t);
    void time2str(gtime_t t, char *s, int n);
    char *time_str(gtime_t t, int n);
    gtime_t timeget();

    /* satellite function */
    int satsys(int sat, int *prn);
    int satidx(int sat, int *prn);
    char satid(int sat, int *prn);
    char sys2char(int sys);
    double satwavelen(int sat, int frq);

    void ecef2pos(const double *r, double *pos);
    void pos2ecef(const double *pos, double *r);

    void set_approximate_time(int year, int doy, rtcm_t *rtcm);

    /* output NMEA GGA */
    int print_nmea_gga(gtime_t time, double *xyz, int nsat, int type, double dop, double age, char *buff);
    void set_ms(uint32_t ms);
#ifdef INT_SEC_SEND
    TIME_S *get_time();
#endif
/*--------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
