#include <math.h>
#include <assert.h>
#include <string.h>

#include "gnss_math.h"
#include "gnss_nav_utils.h"

//#include "rtcm.h"
#include "lambda.h"

//#define _DEBUG_LAMBDA_

#define LOOPMAX     10000 
#define NUM_INT_AMB_CAND 2
#define MAXNUM_INT_AMB_CAND 100

#define SGN(x)   ((x)<=0.0?-1.0:1.0)
#define ROUND(a) (floor((a) + 0.5))
#define SWAP(a, b) do {double SWAP = a; a = b; b = SWAP;} while (0)

/** 
* LDU decomposition (Q=L'*diag(D)*L) 
*/
static bool_t lddecom(const double *Q, const int n, double *L, double *d)
{
    int i,j,k;
	// double val, maxdif;
	double dtmp, alpha, beta;

	assert(L != NULL);
	assert(d != NULL);

	// best to check symmetry first
    memset(L, 0, sizeof(double)*n*n);
    memset(d, 0, sizeof(double)*n);

    // perform UDUt decomposition without square roots
	memcpy(L, Q, sizeof(double) * n * n);
	for (j = n - 1; j >= 0; j--) {
		if ((d[j] = L[j + j*n]) <= 0.0) { 
			return FALSE; 
		}

		alpha = 1.0/d[j];
		for (k = 0; k <= j - 1; k++) {
			beta = L[j + k*n];
			L[j + k*n] = alpha * beta;
			for (i = 0; i <= k; i++) {
				L[k + i*n] -= beta * L[j + i*n];
			}
		}
	}
	d[0] = L[0];

	for (i = 0; i < n; i++) {
		L[i + i*n] = 1.0;
		for (j = 0; j < i; j++) {
			L[j + i*n] = 0.0;
		}
	}
   
	// L = L.T() (i.e. U.T())
	for (i = 0; i < n; i++) {
		for (j = 0; j < n; j++) {
			dtmp = L[i +j*n];
			L[i + j*n] = L[j + i*n];
			L[j + i*n] = dtmp;
		}
	}

    return TRUE;
}


static void gausstrans(double *L, double *Z, const int n, const int i, const int j)
{
	int k, mu;

	if ((mu = (int)ROUND(L[i + j*n])) != 0) {
		for (k = i; k < n; k++) L[k + n*j] -= (double)mu * L[k + i*n];
		for (k = 0; k < n; k++) Z[k + n*j] -= (double)mu * Z[k + i*n];
	}
}

/**
* permutations
*/
static void perm(int n, double *L, double *d, int j, double del, double *Z)
{
    int k;
    double eta,lam,a0,a1;

    eta = d[j] / del;
    lam = d[j+1] * L[j+1 + j*n] / del;
    d[j] = eta * d[j+1]; 
	d[j+1] = del;
    for (k = 0; k <= j - 1; k++) {
        a0 = L[j + k*n]; 
		a1 = L[j+1 + k*n];
        L[j + k*n] = -L[j+1 + j*n]*a0 + a1;
        L[j+1 + k*n] = eta*a0 + lam*a1;
    }

    L[j+1 + j*n] = lam;
    for (k = j+2; k < n; k++) SWAP(L[k+j*n], L[k+(j+1)*n]);
    for (k = 0; k < n; k++) SWAP(Z[k+j*n], Z[k+(j+1)*n]);
}


static void reduction(double *L, double *d, const int n, double *Z)
{
	int i, j, k;
	double del;

	j = n - 2; 
	k = n - 2;
    while (j >= 0) {
        if (j <= k) for (i = j + 1; i < n; i++) gausstrans(L, Z, n, i, j);
        del = d[j] + L[j+1 + j*n] * L[j+1 + j*n] * d[j+1];
        if (del+1E-6 < d[j+1]) { /* compared considering numerical error */
            perm(n, L, d, j, del, Z);
            k = j; 
			j = n-2;
        }
        else j--;
    }
}

#ifdef _DEBUG_LAMBDA_
static void matfprintf(const double *M, const uint8_t n, const uint8_t m, FILE *f)
{
	for (uint8_t i = 0; i < n; i++) {
		for (uint8_t j = 0; j < m; j++) fprintf(f, "%.9f\t", M[i + j*n]);
		fprintf(f, "\n");
	}
}
#endif

static bool_t decorrelation(const double *Q, double *L, double *d, const uint8_t n, double *Z)
{
#ifdef _DEBUG_LAMBDA_
	FILE *fp = fopen("Q.txt", "w");
	if (fp != NULL) {
		matfprintf(Q, n, n, fp);
		fclose(fp);
	}
#endif

	if (!lddecom(Q, n, L, d))
		return FALSE;
	else
		reduction(L, d, n, Z);

	
	//  int i, j, k;
	//  int sw = 1, i1 = n -1;
	//  double delta, lambda, eta, tmpd, mu;

	//  while (sw == 1) {
	// 	 i = n;
	// 	 sw = 0;
	// 	 while (!sw && i > 1) {
	// 		 i = i - 1;
	// 		 if (i <= i1) {
	// 			 for (j = i+1; j <= n; j++) {
	// 				 mu = L[j - 1][i - 1];
	// 				 mu = ROUND(mu);
	// 				 if (mu != 0) {
	// 					 for (k = j; k <= n; k++) {
	// 						 L[k - 1][i - 1] -= mu * L[k-1][j-1];
	// 					 }

	// 					 for (k = 1; k <= n; k++) {
							 
	// 					 }
	// 				 }
	// 			 }
	// 		 }
	// 	 }
	//  }

	return TRUE;
}

// static bool_t chistart()
// {

// 	return True;
// }

static bool_t msearch(const double *L, const double *d, const double *zs, double *za, int n, int m, int ncand, double *sw)
{
	int i, j, k, c, nn = 0, imax = 0;
	double newdist, maxdist = 1E99, y, sumweight;
	double S[MAXAMB*MAXAMB];
	double dist[MAXAMB];
	double zb[MAXAMB];
	double z[MAXAMB];
	double step[MAXAMB];

#ifndef _USE_AVE_AMBRES_
	double zcand[MAXAMB*NUM_INT_AMB_CAND];
	double scand[NUM_INT_AMB_CAND];
    double s[NUM_INT_AMB_CAND];
#else
    double zcand[MAXAMB*MAXNUM_INT_AMB_CAND];
    double scand[MAXNUM_INT_AMB_CAND];
    double s[MAXNUM_INT_AMB_CAND];
#endif 

	memset(S, 0, sizeof(double)*MAXAMB*MAXAMB);

	k = n - 1; dist[k] = 0.0;
	zb[k] = zs[k];
	z[k] = ROUND(zb[k]); y = zb[k] - z[k]; step[k] = SGN(y);
	for (c = 0; c < LOOPMAX; c++) 
    {
		newdist = dist[k] + y * y / (d[k] / d[0]);
		if (newdist < maxdist) 
        {
			if (k != 0) 
            {
				dist[--k] = newdist;
				for (i = 0; i <= k; i++)
					S[k + i * n] = S[k + 1 + i * n] + (z[k + 1] - zb[k + 1])*L[k + 1 + i * n];
				zb[k] = zs[k] + S[k + k * n];
				z[k] = ROUND(zb[k]); y = zb[k] - z[k]; step[k] = SGN(y);
			}
			else 
            {
				if (nn < ncand) 
                {
					if (nn == 0 || newdist > s[imax]) imax = nn;
					for (i = 0; i < n; i++) zcand[i + nn * n] = z[i];
					scand[nn++] = newdist;
				}
				else {
					if (newdist < s[imax]) 
                    {
						for (i = 0; i < n; i++) zcand[i + imax * n] = z[i];
						scand[imax] = newdist;
						for (i = imax = 0; i < m; i++) if (scand[imax] < scand[i]) imax = i;
					}
					maxdist = s[imax];
				}
				z[0] += step[0]; y = zb[0] - z[0]; step[0] = -step[0] - SGN(step[0]);
			}
		}
		else 
        {
			if (k == n - 1) break;
			else 
            {
				k++;
				z[k] += step[k]; y = zb[k] - z[k]; step[k] = -step[k] - SGN(step[k]);
			}
		}
	}
	for (i = 0; i < ncand - 1; i++) 
    { /* sort by s */
		for (j = i + 1; j < ncand; j++) 
        {
			if (scand[i] < scand[j]) continue;
			SWAP(scand[i], scand[j]);
			for (k = 0; k < n; k++) SWAP(zcand[k + i * n], zcand[k + j * n]);
		}
	}

    sw[0] = scand[0];
    sw[1] = scand[1];

    if (sw[1] / sw[0] < 3.0)
    {
        sumweight = 0.0;
        for (i = 0; i < n; i++)
        {
            za[i] = 0.0;
            for (j = 0; j < ncand; j++)
            {
                za[i] = za[i] + zcand[i + j * n] * exp(-0.5*scand[j] * d[0]);
                if (i == 0) sumweight = sumweight + exp(-0.5*scand[j] * d[0]);
            }
            za[i] = za[i] / sumweight;
        }
    }
    else
    {
        for (i = 0; i < 2*n; i++)
        {
               za[i] =  zcand[i];
        }
    }

	if (c >= LOOPMAX) 
    {
		return FALSE;
	}
	return TRUE;
}

static bool_t search(const double *L, const double *d,
					 const double *zs, int n, int m, double *zn, double *s)
{
	int i, j, k, c, nn = 0, imax = 0;
	double newdist, maxdist = 1E99, y;
	double S[MAXAMB*MAXAMB];
	double dist[MAXAMB];
	double zb[MAXAMB];
	double z[MAXAMB];
	double step[MAXAMB];
	
	memset(S, 0, sizeof(double)*MAXAMB*MAXAMB);

	k = n - 1; dist[k] = 0.0;
	zb[k] = zs[k];
	z[k] = ROUND(zb[k]); y = zb[k] - z[k]; step[k] = SGN(y);
	for (c = 0; c<LOOPMAX; c++) {
		newdist = dist[k] + y*y / d[k];
		if (newdist<maxdist) {
			if (k != 0) {
				dist[--k] = newdist;
				for (i = 0; i <= k; i++)
					S[k + i*n] = S[k + 1 + i*n] + (z[k + 1] - zb[k + 1])*L[k + 1 + i*n];
				zb[k] = zs[k] + S[k + k*n];
				z[k] = ROUND(zb[k]); y = zb[k] - z[k]; step[k] = SGN(y);
			}
			else {
				if (nn<m) {
					if (nn == 0 || newdist>s[imax]) imax = nn;
					for (i = 0; i<n; i++) zn[i + nn*n] = z[i];
					s[nn++] = newdist;
				}
				else {
					if (newdist<s[imax]) {
						for (i = 0; i<n; i++) zn[i + imax*n] = z[i];
						s[imax] = newdist;
						for (i = imax = 0; i<m; i++) if (s[imax]<s[i]) imax = i;
					}
					maxdist = s[imax];
				}
				z[0] += step[0]; y = zb[0] - z[0]; step[0] = -step[0] - SGN(step[0]);
			}
		}
		else {
			if (k == n - 1) break;
			else {
				k++;
				z[k] += step[k]; y = zb[k] - z[k]; step[k] = -step[k] - SGN(step[k]);
			}
		}
	}
	for (i = 0; i<m - 1; i++) 
    { /* sort by s */
		for (j = i + 1; j<m; j++) 
        {
			if (s[i]<s[j]) continue;
			SWAP(s[i], s[j]);
			for (k = 0; k<n; k++) SWAP(zn[k + i*n], zn[k + j*n]);
		}
	}

	if (c >= LOOPMAX) {
		//log_error("LAMBDA search loop count overflow");
		return FALSE;
	}
	return TRUE;
}


static bool_t solve(const char *tr, const double *A, const double *Y, int n,
                 	int m, double *X)
{
	double B[MAXAMB*MAXAMB] = {0.0};
    bool_t info;
    
    memcpy(B, A, sizeof(double)*n*n);
	info = matinv(B, n);
    if (info) 
		matmul(tr[0]=='N'?"NN":"TN",n,m,n,1.0,B,Y,0.0,X);
    return info;
}


extern int lambda(const double *a, const double *Q, 
			  const int n, const int m, 
			  double *a_chk, double *ss_err)
{
	// double incr[n] = {0.0};
	// double ah[n] = {0.0};

	// for (uint8_t i = 0; i < n; ++i) {
	// 	incr[i] = ROUND(a[i]);
	// }

	// matminus_fast(a, incr, n, 1, ah);
	assert(n > 0);
	assert(m > 0);

	double L[MAXAMB*MAXAMB] = { 0.0 }, d[MAXAMB] = { 0.0 };
	double Z[MAXAMB*MAXAMB] = { 0.0 }, z[MAXAMB] = { 0.0 };

    double RES[MAXAMB*NUM_INT_AMB_CAND] = { 0.0 };

#ifndef _USE_AVE_AMBRES_
	if (n > MAXAMB || m > NUM_INT_AMB_CAND)
		return FALSE;
#else
    if (n > MAXAMB || m > MAXNUM_INT_AMB_CAND)
        return FALSE;
#endif


	for (int i = 0; i < n; i++) Z[i + i*n] = 1.0;

	if (decorrelation(Q, L, d, n, Z)) 
    {
		/* z = Z' * a */
		matmul("TN", n, 1, n, 1.0, Z, a, 0.0, z);

#ifdef _DEBUG_LAMBDA_
		FILE *fpL = fopen("L.txt", "w");
		if (fpL != NULL) {
			matfprintf(L, n, n, fpL);
			fclose(fpL);
		}
#endif

#ifndef _USE_AVE_AMBRES_       
		if (search(L, d, z, n, m, RES, ss_err)) 
        {
			if(!solve("T", Z, RES, n, m, a_chk)) 
				return FALSE;
		}
		else 
        {
			return FALSE;
		} 
#else
        if (msearch(L, d, z, RES, n, m, MAXNUM_INT_AMB_CAND,ss_err))
        {
            if (!solve("T", Z, RES, n, 1, a_chk))
                return FALSE;
        }
        else
        {
            return FALSE;
        }
#endif

	} 
	else {
		return FALSE;
	}

	assert(ss_err[0] != 0.0);
	return TRUE;
}
