#ifndef _PPP_RTK_H_
#define _PPP_RTK_H_

#include "rtcm.h"

#include "ephemeris.h"
#include "lambda.h"
#include "gnss_math.h"
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* by Dr. Yudan Yi */

/*-----------------------------------------------------------*/
/* from rtklib to decode RTCM3 */
#ifdef _USE_PPP_

#define PMODE_SINGLE 0                  /* positioning mode: single */
#define PMODE_DGPS   1                  /* positioning mode: DGPS/DGNSS */
#define PMODE_KINEMA 2                  /* positioning mode: kinematic */
#define PMODE_STATIC 3                  /* positioning mode: static */
#define PMODE_MOVEB  4                  /* positioning mode: moving-base */
#define PMODE_FIXED  5                  /* positioning mode: fixed */
#define PMODE_PPP_KINEMA 6              /* positioning mode: PPP-kinemaric */
#define PMODE_PPP_STATIC 7              /* positioning mode: PPP-static */
#define PMODE_PPP_FIXED  8              /* positioning mode: PPP-fixed */
#endif
#if 1
#define EFACT_GPS   1.0                 /* error factor: GPS */
#define EFACT_GLO   1.5                 /* error factor: GLONASS */
#define EFACT_GAL   1.0                 /* error factor: Galileo */
#define EFACT_QZS   1.0                 /* error factor: QZSS */
#define EFACT_BDS   1.0                 /* error factor: BeiDou */
#endif


#ifdef _USE_PPP_
#define SOLQ_NONE   0                   /* solution status: no solution */
#define SOLQ_FIX    1                   /* solution status: fix */
#define SOLQ_FLOAT  2                   /* solution status: float */
#define SOLQ_SBAS   3                   /* solution status: SBAS */
#define SOLQ_DGPS   4                   /* solution status: DGPS/DGNSS */
#define SOLQ_SINGLE 5                   /* solution status: single */
#define SOLQ_PPP    6                   /* solution status: PPP */
#define SOLQ_DR     7                   /* solution status: dead reconing */
#define MAXSOLQ     8                   /* max number of solution status */
#endif

#if _USE_PPP_
#define IONOOPT_OFF  0                  /* ionosphere option: correction off */
#define IONOOPT_BRDC 1                  /* ionosphere option: broadcast model */
#define IONOOPT_IF1  2                  /* ionosphere option: L1 iono-free GRAPHIC  */
#define IONOOPT_IF12 3                  /* ionosphere option: L1/L2 or L1/L5 iono-free LC */
#define IONOOPT_UC1  4                  /* ionosphere option: estimation */
#define IONOOPT_UC12 5                  /* ionosphere option: estimation */
#define IONOOPT_TEC  6                  /* ionosphere option: IONEX TEC model */
#define IONOOPT_QZS  7                  /* ionosphere option: QZSS broadcast model */
#define IONOOPT_LEX  8                  /* ionosphere option: QZSS LEX ionospehre */
#define IONOOPT_STEC 9                  /* ionosphere option: SLANT TEC model */

#define TROPOPT_OFF 0                   /* troposphere option: correction off */
#define TROPOPT_SAAS 1                  /* troposphere option: Saastamoinen model */
#define TROPOPT_SBAS 2                  /* troposphere option: SBAS model */
#define TROPOPT_EST 3                   /* troposphere option: ZTD estimation */
#define TROPOPT_ESTG 4                  /* troposphere option: ZTD+grad estimation */
#define TROPOPT_ZTD 5                   /* troposphere option: ZTD correction */

#endif

#define MI(i, j, n) ((i) * (n) + (j))

#define SMD(i) ((i)*((i)+1)/2)

#define SMI(i, j)	((i) > (j) ? (SMD(i) + (j)) : (SMD(j) + (i)))

#ifndef _USE_UC_
#define NX_PPP 40    /* maximum filter state vector */
#else
#define NX_PPP 85
#endif

#define NV_PPP MAXOBS*NFREQ*2 
#define NP_PPP (SMD(NX_PPP)) 

/* YYD: modify this to store less satellite to save space */

#define NX_SPP (3+NSYS*NFREQ)
#define NP_SPP (SMD(NX_SPP))

#define NX_RTK (30) //(40) /* p(3),v(3),a(3),cdt(1) */
#define NP_RTK (SMD(NX_RTK))

#define NX_RTD (9*MAXROV+1) /* p(3),v(3),a(3),cdt(1) */
#define NP_RTD (SMD(NX_RTD))

typedef struct {        /* observation data */
    double x[NX_SPP];
    double P[NP_SPP];
    double time;
    int n_used;
    int solType;
} rcv_spp_t;

typedef struct 
{
    unsigned int s1,s2,f;
    double time;
}state_tag_t;

typedef struct
{
	unsigned int s1, s2, f;
	double data;
}ambdata_t;
typedef struct
{
	unsigned int n;
	ambdata_t amb[MAXAMB];
	double ratio;
	unsigned int nsat;
}ambset_t;

/* use dimension of RTD+doppler to filter position, velocity, then use EBE AR, to save  */
#define _USE_RTD_ 
#ifndef _USE_RTD_
    #define _USE_SLIP_DETEC_
#endif

typedef struct
{
#ifdef _USE_RTD_
	double x[NX_RTD]; 
    double P[NP_RTD];
	/*ambset_t ambset;*/ /* do not store it */
#else
    double x[NX_RTK]; 
    double P[NP_RTK];
    state_tag_t tag[NX_RTK];
	double x_fixed[NX_RTK];    
#endif
    unsigned int count;
    int np;
    int ns;
    double time;
    double tt;
    double age;
#ifdef _DEBUG
	double refxyz[3];
	double C_en[3][3];
	int numofepoch;
#endif
}rcv_rtk_t;

typedef struct
{
    double x[NX_PPP];
    double P[NP_PPP];
    state_tag_t tag[NX_PPP];
    unsigned int count;
    unsigned char np;
    unsigned char ns;
    double time;
    double tt;
}rcv_ppp_t;

#if 0 /* remove this defintion */
typedef struct {
    /* do not run the spp to save space, use the ST solution */
	//rcv_t      rcv[MAXSTN];
    //nav_t  nav;
    rcv_rtk_t filter;
    int    nx;
    double tt;
	float rtk_sol_age;
} ppprtk_t;
#endif
/* algorithm */
int spp_processor(obs_t *obs, nav_t *nav, rcv_spp_t *rcv, int isPrint);
int rtk_processor(obs_t *rov, obs_t *ref, nav_t *nav, rcv_rtk_t *rcv, char *gga, char *sol, int isPrint);
int ppp_processor(obs_t *obs, nav_t *nav, rcv_ppp_t *rcv, int isPrint);

void blh2C_en(const double *blh, double C_en[3][3]);
void xyz2ned(double C_en[3][3], double *xyz, double *covXYZ, double *ned, double *covNED);
void blhdiff(double *blh, double *blh_ref, double *ned);
/*--------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
