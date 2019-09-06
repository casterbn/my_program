#include <math.h>
#include <string.h>

#include "gnss_math.h"
#include "gnss_nav_proc.h"
#include "gnss_nav_utils.h"

void
eye(double *A, uint32_t n)
{
    uint32_t i;
    
    for (i = 0; i < n; i++) 
		A[i + i * n] = 1.0;
}

void 
matcpy(double *A, const double *B, uint32_t n, uint32_t m)
{
    nav_memcpy(A, B, sizeof(double) * n * m);
}

void
matdel_row(double *A, uint32_t n, uint32_t m, uint32_t row)
{
	double tmp[MAX_OBS_NUM * MAX_OBS_NUM];
	uint32_t i, j;

	for (i = 0; i < row; i++)
		for (j = 0; j < m; j++)
			tmp[i + j * (n - 1)] = A[i + j * n];

	for (i = row + 1; i < n; i++)
		for (j = 0; j < m; j++)
			tmp[i - 1 + j * (n - 1)] = A[i + j * n];


	matcpy(A, tmp, n, m);

}


void
matdel_col(double *A, uint32_t n, uint32_t m, uint32_t col)
{
	double tmp[MAX_OBS_NUM * MAX_OBS_NUM];

	if (col > 0)
		nav_memcpy(tmp, A, col * n * sizeof(double));

	if (col < m - 1)
		nav_memcpy(tmp + n * col, A + n * (1 + col),
		(m - col - 1) * n * sizeof(double));

	matcpy(A, tmp, n, m);

}


void
square_del(double *A, uint32_t n, uint32_t col)
{

	matdel_col(A, n, n, col);
	matdel_row(A, n, n - 1, col);

}


void square_append(double *A, double value, uint32_t n)
{
	uint32_t i, j;
	double temp[MAX_KF_STATE *  MAX_KF_STATE] = { 0 };

	for (i = 0; i < n - 1; i++) {
		for (j = 0; j < n - 1; j++) {
			temp[i + j * n] = A[i + j * (n - 1)];
		}
	}

	temp[n - 1 + (n - 1) * n] = value;

	nav_memcpy(A, temp, sizeof(double) * MAX_KF_STATE *  MAX_KF_STATE);

}

/* multiply matrix -----------------------------------------------------------*/
void matmul(const char *tr, uint32_t n, uint32_t k, uint32_t m, double alpha,
const double *A, const double *B, double beta, double *C)
{
	double d;
	uint32_t i, j, x;
	char f = tr[0] == 'N' ? (tr[1] == 'N' ? 1 : 2) : (tr[1] == 'N' ? 3 : 4);

	for (i = 0; i < n; i++) {
		for (j = 0; j < k; j++) {
			d = 0.0;
			switch (f) {
			case 1:
				for (x = 0; x < m; x++)
					d += A[i + x*n] * B[x + j*m];
				break;
			case 2:
				for (x = 0; x < m; x++)
					d += A[i + x*n] * B[j + x*k];
				break;
			case 3:
				for (x = 0; x < m; x++)
					d += A[x + i*m] * B[x + j*m];
				break;
			case 4:
				for (x = 0; x < m; x++)
					d += A[x + i*m] * B[j + x*k];
				break;
			}

			if (beta == 0.0)
				C[i + j*n] = alpha * d;
			else
				C[i + j*n] = alpha * d + beta * C[i + j*n];
		}
	}
}


void matadd(double alpha, const double *A, double beta, const double *B, uint32_t n, uint32_t m, double *C)
{
	uint32_t i, j;

	for (i = 0; i < n; i++) {
		for (j = 0; j < m; j++) {
			C[i + j*n] = alpha * A[i + j*n] + beta * B[i + j*n];
		}
	}

}

void matminus_fast(const double *A, const double *B, uint32_t n, uint32_t m, double *C)
{
	uint32_t i, j;

	for (i = 0; i < n; i++) {
		for (j = 0; j < m; j++) {
			C[i + j * n] = A[i + j * n] - B[i + j * n];
		}
	}
}


/* LU decomposition ----------------------------------------------------------*/
static bool_t
ludcmp(double *A, uint32_t n, int32_t *indx, double *d)
{
	double big, s, tmp;
	double vv[MAX_OBS_NUM] = { 0 };
	uint32_t i, imax = 0, j, k;

	*d = 1.0;

	for (i = 0; i < n; i++) {
		big = 0.0;
		for (j = 0; j < n; j++)
		if ((tmp = ABS(A[i + j * n])) > big)
			big = tmp;
		if (big > 0.0)
			vv[i] = 1.0 / big;
		else
			return FALSE;
	}

	for (j = 0; j < n; j++) {
		for (i = 0; i < j; i++) {
			s = A[i + j * n];
			for (k = 0; k < i; k++)
				s -= A[i + k * n] * A[k + j * n];
			A[i + j * n] = s;
		}

		big = 0.0;
		for (i = j; i < n; i++) {
			s = A[i + j*n];
			for (k = 0; k < j; k++)
				s -= A[i + k * n] * A[k + j * n];
			A[i + j*n] = s;
			if ((tmp = vv[i] * ABS(s)) >= big) {
				big = tmp;
				imax = i;
			}
		}

		if (j != imax) {
			for (k = 0; k < n; k++) {
				tmp = A[imax + k * n];
				A[imax + k * n] = A[j + k * n];
				A[j + k * n] = tmp;
			}
			*d = -(*d);
			vv[imax] = vv[j];
		}

		indx[j] = (int32_t)imax;

		if (A[j + j*n] == 0.0)
			return FALSE;

		if (j != n - 1) {
			tmp = 1.0 / A[j + j * n];
			for (i = j + 1; i < n; i++)
				A[i + j * n] *= tmp;
		}
	}

	return TRUE;
}


/* LU back-substitution ------------------------------------------------------*/
static void
lubksb(const double *A, uint32_t n, const int32_t *indx, double *b)
{
	double s;
	int32_t ii = -1, i, ip, j;

	for (i = 0; i < (int32_t)n; i++) {
		ip = indx[i];
		s = b[ip];
		b[ip] = b[i];

		if (ii >= 0)
			for (j = ii; j < i; j++)
				s -= A[i + j * n] * b[j];
		else if (s)
			ii = (int32_t)i;

		b[i] = s;
	}

	for (i = (int32_t)n - 1; i >= 0; i--) {
		s = b[i];
		
		for (j = i + 1; j < (int32_t)n; j++)
			s -= A[i + j * n] * b[j];
		
		b[i] = s / A[i + i * n];
	}
}

#define MATRIX_EPSILON (1e-60)
int inv4(const double *a, double *b) {
  double det =
      (((a[4 * 1 + 0] * -((a[4 * 2 + 1] * -(a[4 * 0 + 2] * a[4 * 3 + 3] -
                                            a[4 * 0 + 3] * a[4 * 3 + 2]) +
                           a[4 * 2 + 2] * (a[4 * 0 + 1] * a[4 * 3 + 3] -
                                           a[4 * 0 + 3] * a[4 * 3 + 1])) +
                          a[4 * 2 + 3] * -(a[4 * 0 + 1] * a[4 * 3 + 2] -
                                           a[4 * 0 + 2] * a[4 * 3 + 1])) +
         a[4 * 1 + 1] * ((a[4 * 2 + 0] * -(a[4 * 0 + 2] * a[4 * 3 + 3] -
                                           a[4 * 0 + 3] * a[4 * 3 + 2]) +
                          a[4 * 2 + 2] * (a[4 * 0 + 0] * a[4 * 3 + 3] -
                                          a[4 * 0 + 3] * a[4 * 3 + 0])) +
                         a[4 * 2 + 3] * -(a[4 * 0 + 0] * a[4 * 3 + 2] -
                                          a[4 * 0 + 2] * a[4 * 3 + 0]))) +
        a[4 * 1 + 2] * -((a[4 * 2 + 0] * -(a[4 * 0 + 1] * a[4 * 3 + 3] -
                                           a[4 * 0 + 3] * a[4 * 3 + 1]) +
                          a[4 * 2 + 1] * (a[4 * 0 + 0] * a[4 * 3 + 3] -
                                          a[4 * 0 + 3] * a[4 * 3 + 0])) +
                         a[4 * 2 + 3] * -(a[4 * 0 + 0] * a[4 * 3 + 1] -
                                          a[4 * 0 + 1] * a[4 * 3 + 0]))) +
       a[4 * 1 + 3] * ((a[4 * 2 + 0] * -(a[4 * 0 + 1] * a[4 * 3 + 2] -
                                         a[4 * 0 + 2] * a[4 * 3 + 1]) +
                        a[4 * 2 + 1] * (a[4 * 0 + 0] * a[4 * 3 + 2] -
                                        a[4 * 0 + 2] * a[4 * 3 + 0])) +
                       a[4 * 2 + 2] * -(a[4 * 0 + 0] * a[4 * 3 + 1] -
                                        a[4 * 0 + 1] * a[4 * 3 + 0])));

  if (fabs(det) < MATRIX_EPSILON) return -1;

  b[4 * 0 + 0] =
      ((a[4 * 2 + 1] *
            -(a[4 * 1 + 2] * a[4 * 3 + 3] - a[4 * 1 + 3] * a[4 * 3 + 2]) +
        a[4 * 2 + 2] *
            (a[4 * 1 + 1] * a[4 * 3 + 3] - a[4 * 1 + 3] * a[4 * 3 + 1])) +
       a[4 * 2 + 3] *
           -(a[4 * 1 + 1] * a[4 * 3 + 2] - a[4 * 1 + 2] * a[4 * 3 + 1])) /
      det;
  b[4 * 1 + 0] =
      -((a[4 * 2 + 0] *
             -(a[4 * 1 + 2] * a[4 * 3 + 3] - a[4 * 1 + 3] * a[4 * 3 + 2]) +
         a[4 * 2 + 2] *
             (a[4 * 1 + 0] * a[4 * 3 + 3] - a[4 * 1 + 3] * a[4 * 3 + 0])) +
        a[4 * 2 + 3] *
            -(a[4 * 1 + 0] * a[4 * 3 + 2] - a[4 * 1 + 2] * a[4 * 3 + 0])) /
      det;
  b[4 * 2 + 0] =
      ((a[4 * 2 + 0] *
            -(a[4 * 1 + 1] * a[4 * 3 + 3] - a[4 * 1 + 3] * a[4 * 3 + 1]) +
        a[4 * 2 + 1] *
            (a[4 * 1 + 0] * a[4 * 3 + 3] - a[4 * 1 + 3] * a[4 * 3 + 0])) +
       a[4 * 2 + 3] *
           -(a[4 * 1 + 0] * a[4 * 3 + 1] - a[4 * 1 + 1] * a[4 * 3 + 0])) /
      det;
  b[4 * 3 + 0] =
      -((a[4 * 2 + 0] *
             -(a[4 * 1 + 1] * a[4 * 3 + 2] - a[4 * 1 + 2] * a[4 * 3 + 1]) +
         a[4 * 2 + 1] *
             (a[4 * 1 + 0] * a[4 * 3 + 2] - a[4 * 1 + 2] * a[4 * 3 + 0])) +
        a[4 * 2 + 2] *
            -(a[4 * 1 + 0] * a[4 * 3 + 1] - a[4 * 1 + 1] * a[4 * 3 + 0])) /
      det;

  b[4 * 0 + 1] =
      -((a[4 * 2 + 1] *
             -(a[4 * 0 + 2] * a[4 * 3 + 3] - a[4 * 0 + 3] * a[4 * 3 + 2]) +
         a[4 * 2 + 2] *
             (a[4 * 0 + 1] * a[4 * 3 + 3] - a[4 * 0 + 3] * a[4 * 3 + 1])) +
        a[4 * 2 + 3] *
            -(a[4 * 0 + 1] * a[4 * 3 + 2] - a[4 * 0 + 2] * a[4 * 3 + 1])) /
      det;
  b[4 * 1 + 1] =
      ((a[4 * 2 + 0] *
            -(a[4 * 0 + 2] * a[4 * 3 + 3] - a[4 * 0 + 3] * a[4 * 3 + 2]) +
        a[4 * 2 + 2] *
            (a[4 * 0 + 0] * a[4 * 3 + 3] - a[4 * 0 + 3] * a[4 * 3 + 0])) +
       a[4 * 2 + 3] *
           -(a[4 * 0 + 0] * a[4 * 3 + 2] - a[4 * 0 + 2] * a[4 * 3 + 0])) /
      det;
  b[4 * 2 + 1] =
      -((a[4 * 2 + 0] *
             -(a[4 * 0 + 1] * a[4 * 3 + 3] - a[4 * 0 + 3] * a[4 * 3 + 1]) +
         a[4 * 2 + 1] *
             (a[4 * 0 + 0] * a[4 * 3 + 3] - a[4 * 0 + 3] * a[4 * 3 + 0])) +
        a[4 * 2 + 3] *
            -(a[4 * 0 + 0] * a[4 * 3 + 1] - a[4 * 0 + 1] * a[4 * 3 + 0])) /
      det;
  b[4 * 3 + 1] =
      ((a[4 * 2 + 0] *
            -(a[4 * 0 + 1] * a[4 * 3 + 2] - a[4 * 0 + 2] * a[4 * 3 + 1]) +
        a[4 * 2 + 1] *
            (a[4 * 0 + 0] * a[4 * 3 + 2] - a[4 * 0 + 2] * a[4 * 3 + 0])) +
       a[4 * 2 + 2] *
           -(a[4 * 0 + 0] * a[4 * 3 + 1] - a[4 * 0 + 1] * a[4 * 3 + 0])) /
      det;

  b[4 * 0 + 2] =
      ((a[4 * 1 + 1] *
            -(a[4 * 0 + 2] * a[4 * 3 + 3] - a[4 * 0 + 3] * a[4 * 3 + 2]) +
        a[4 * 1 + 2] *
            (a[4 * 0 + 1] * a[4 * 3 + 3] - a[4 * 0 + 3] * a[4 * 3 + 1])) +
       a[4 * 1 + 3] *
           -(a[4 * 0 + 1] * a[4 * 3 + 2] - a[4 * 0 + 2] * a[4 * 3 + 1])) /
      det;
  b[4 * 1 + 2] =
      -((a[4 * 1 + 0] *
             -(a[4 * 0 + 2] * a[4 * 3 + 3] - a[4 * 0 + 3] * a[4 * 3 + 2]) +
         a[4 * 1 + 2] *
             (a[4 * 0 + 0] * a[4 * 3 + 3] - a[4 * 0 + 3] * a[4 * 3 + 0])) +
        a[4 * 1 + 3] *
            -(a[4 * 0 + 0] * a[4 * 3 + 2] - a[4 * 0 + 2] * a[4 * 3 + 0])) /
      det;
  b[4 * 2 + 2] =
      ((a[4 * 1 + 0] *
            -(a[4 * 0 + 1] * a[4 * 3 + 3] - a[4 * 0 + 3] * a[4 * 3 + 1]) +
        a[4 * 1 + 1] *
            (a[4 * 0 + 0] * a[4 * 3 + 3] - a[4 * 0 + 3] * a[4 * 3 + 0])) +
       a[4 * 1 + 3] *
           -(a[4 * 0 + 0] * a[4 * 3 + 1] - a[4 * 0 + 1] * a[4 * 3 + 0])) /
      det;
  b[4 * 3 + 2] =
      -((a[4 * 1 + 0] *
             -(a[4 * 0 + 1] * a[4 * 3 + 2] - a[4 * 0 + 2] * a[4 * 3 + 1]) +
         a[4 * 1 + 1] *
             (a[4 * 0 + 0] * a[4 * 3 + 2] - a[4 * 0 + 2] * a[4 * 3 + 0])) +
        a[4 * 1 + 2] *
            -(a[4 * 0 + 0] * a[4 * 3 + 1] - a[4 * 0 + 1] * a[4 * 3 + 0])) /
      det;

  b[4 * 0 + 3] =
      -((a[4 * 1 + 1] *
             -(a[4 * 0 + 2] * a[4 * 2 + 3] - a[4 * 0 + 3] * a[4 * 2 + 2]) +
         a[4 * 1 + 2] *
             (a[4 * 0 + 1] * a[4 * 2 + 3] - a[4 * 0 + 3] * a[4 * 2 + 1])) +
        a[4 * 1 + 3] *
            -(a[4 * 0 + 1] * a[4 * 2 + 2] - a[4 * 0 + 2] * a[4 * 2 + 1])) /
      det;
  b[4 * 1 + 3] =
      ((a[4 * 1 + 0] *
            -(a[4 * 0 + 2] * a[4 * 2 + 3] - a[4 * 0 + 3] * a[4 * 2 + 2]) +
        a[4 * 1 + 2] *
            (a[4 * 0 + 0] * a[4 * 2 + 3] - a[4 * 0 + 3] * a[4 * 2 + 0])) +
       a[4 * 1 + 3] *
           -(a[4 * 0 + 0] * a[4 * 2 + 2] - a[4 * 0 + 2] * a[4 * 2 + 0])) /
      det;
  b[4 * 2 + 3] =
      -((a[4 * 1 + 0] *
             -(a[4 * 0 + 1] * a[4 * 2 + 3] - a[4 * 0 + 3] * a[4 * 2 + 1]) +
         a[4 * 1 + 1] *
             (a[4 * 0 + 0] * a[4 * 2 + 3] - a[4 * 0 + 3] * a[4 * 2 + 0])) +
        a[4 * 1 + 3] *
            -(a[4 * 0 + 0] * a[4 * 2 + 1] - a[4 * 0 + 1] * a[4 * 2 + 0])) /
      det;
  b[4 * 3 + 3] =
      ((a[4 * 1 + 0] *
            -(a[4 * 0 + 1] * a[4 * 2 + 2] - a[4 * 0 + 2] * a[4 * 2 + 1]) +
        a[4 * 1 + 1] *
            (a[4 * 0 + 0] * a[4 * 2 + 2] - a[4 * 0 + 2] * a[4 * 2 + 0])) +
       a[4 * 1 + 2] *
           -(a[4 * 0 + 0] * a[4 * 2 + 1] - a[4 * 0 + 1] * a[4 * 2 + 0])) /
      det;

  return 0;
}

/* inverse of matrix ---------------------------------------------------------*/
bool_t
matinv(double *A, uint32_t n)
{
	double d;
	double B[MAX_OBS_NUM * MAX_OBS_NUM] ;
	uint32_t i, j;
	int32_t indx[MAX_OBS_NUM * MAX_OBS_NUM];

	matcpy(B, A, n, n);

	if (!ludcmp(B, n, indx, &d)) {
		return FALSE;
	}

	for (j = 0; j < n; j++) {
		for (i = 0; i < n; i++)
			A[i + j*n] = 0.0;

		A[j + j*n] = 1.0;
		lubksb(B, n, indx, A + j * n);
	}

	return TRUE;
}


void
least_squares(double *H, double *Z, double *W, double *dX, uint8_t nrow, uint8_t state_num)
{
	double Ht_Z[MAX_KF_STATE] = { 0 };
	double Ht_H1[MAX_GNSS_STATE * MAX_GNSS_STATE] = { 0 };
	double Ht_H[MAX_GNSS_STATE * MAX_GNSS_STATE] = { 0 };

	// matmul("TN", state_num, 1, nrow, 1.0, H, w, 0.0, Ht_w);
	matmul("TN", state_num, state_num, nrow, 1.0, H, H, 0.0, Ht_H1);
	// matmul("TN", 1, state_num, nrow, 1.0, w, H, 0.0, wt_H);
	// matmul("NN", state_num, state_num, 1, 1.0, Ht_w, wt_H, 0.0, Ht_H);
	// matinv(Ht_H, state_num);
	inv4(Ht_H1, Ht_H);

	matmul("TN", state_num, 1, nrow, 1.0, H, Z, 0.0, Ht_Z);
	
	matmul("NN", state_num, 1, state_num, 1.0, Ht_H, Ht_Z, 0.0, dX);

}

void kalman_filter_predition(double *T, double *Q, double *P, uint8_t state_num)
{
	double tmpP1[MAX_KF_STATE * MAX_KF_STATE] = { 0 };
	double tmpP2[MAX_KF_STATE * MAX_KF_STATE] = { 0 };

	matmul("NN", state_num, state_num, state_num, 1.0, T, P, 0.0, tmpP1);
	matmul("NT", state_num, state_num, state_num, 1.0, tmpP1, T, 0.0, tmpP2);
	matadd(1.0, tmpP2, 1.0, Q, state_num, state_num, P);

}

/* new integer matrix ----------------------------------------------------------
* allocate memory of integer matrix
* args   : int    n,m       I   number of rows and columns of matrix
* return : matrix pointer (if n<=0 or m<=0, return NULL)
*-----------------------------------------------------------------------------*/
extern int *imat(int n, int m)
{
    int *p;

    if (n <= 0 || m <= 0) return NULL;
    if (!(p = (int *)malloc(sizeof(int)*n*m))) {
        //  fatalerr("integer matrix memory allocation error: n=%d,m=%d\n",n,m);
    }
    return p;
}

/* identity matrix -------------------------------------------------------------
* generate new identity matrix
* args   : int    n         I   number of rows and columns of matrix
* return : matrix pointer (if n<=0, return NULL)

/* inner product ---------------------------------------------------------------
* inner product of vectors
* args   : double *a,*b     I   vector a,b (n x 1)
*          int    n         I   size of vector a,b
* return : a'*b
*-----------------------------------------------------------------------------*/
extern double dot(const double *a, const double *b, int n)
{
    double c = 0.0;

    while (--n >= 0) c += a[n] * b[n];
    return c;
}