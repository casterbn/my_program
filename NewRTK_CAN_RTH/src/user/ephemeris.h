#ifndef _EPHEMERIS_H_
#define _EPHEMERIS_H_

#include "rtcm.h"


#ifdef __cplusplus
extern "C" {
#endif

/* by Dr. Yudan Yi */

/*-----------------------------------------------------------*/

#define EPHOPT_BRDC 0                   /* ephemeris option: broadcast ephemeris */
#define EPHOPT_PREC 1                   /* ephemeris option: precise ephemeris */
#define EPHOPT_SBAS 2                   /* ephemeris option: broadcast + SBAS */
#define EPHOPT_SSRAPC 3                 /* ephemeris option: broadcast + SSR_APC */
#define EPHOPT_SSRCOM 4                 /* ephemeris option: broadcast + SSR_COM */
#define EPHOPT_LEX  5                   /* ephemeris option: QZSS LEX ephemeris */

typedef struct {
	int	   sat;        /*prn*/
    double satpvt[8];
    double var;
    int svh;    
	double azel[2];    /*azimuth,elevation*/
	double e[3];       /*partial deviation*/
	double tgd;        /* tgd*/
	double r;          /* vector */
	double rate;
	double tro;        /* tropospheric */
}vec_t;

/* compute satellit position */
int8_t satposs(obs_t *obs, vec_t *vec, nav_t *nav, int ephopt);

/*--------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
