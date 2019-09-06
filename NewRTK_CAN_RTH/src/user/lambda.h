#ifndef _LAMBDA_H_
#define _LAMBDA_H_

#ifdef __cplusplus
extern "C" {
#endif

/* by Dr. Yudan Yi & Dr. Da Wang */
#define _USE_AVE_AMBRES_

/*-----------------------------------------------------------*/
#define MAXAMB (15)//(40)
#define NUM_INT_AMB_CANDIDATE (2)
//#define MAXNUM_INT_AMB_CANDIDATE (2) //(100)
#define MAX_NON_AMB_STATE (15)

extern int lambda(const double *a, const double *Q,
	const int n, const int m,
	double *a_chk, double *ss_err);

/*--------------------------------------------------------------------*/
#ifdef __cplusplus
}
#endif
#endif
