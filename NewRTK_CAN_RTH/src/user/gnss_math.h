#ifndef _GNSS_MATH_H_
#define _GNSS_MATH_H_

#include <stdint.h>
#include "gnss_nav_utils.h"

#define matrix_at(A, x, y, rownum)    ((A)[(x) + (rownum) * (y)])

// set square matrix A to identity matrix with the rows of n
void eye(double *A, uint32_t n);
void matcpy(double *A, const double *B, uint32_t n, uint32_t m);

void matdel_row(double *A, uint32_t n, uint32_t m, uint32_t row);
void matdel_col(double *A, uint32_t n, uint32_t m, uint32_t col);
void square_del(double *A, uint32_t n, uint32_t col);
void square_append(double *A, double value, uint32_t n);
// matrix multiplication: C = alpha .*(A * B) + beta .* C 
void matmul(const char *tr, uint32_t n, uint32_t k, uint32_t m, double alpha,
	   const double *A, const double *B, double beta, double *C);

// matrix add: C = alpha * A + beta * B 
void matadd(double alpha, const double *A, double beta, const double *B, uint32_t n, uint32_t m, double *C);
void matminus_fast(const double *A, const double *B, uint32_t n, uint32_t m, double *C);
bool_t matinv(double *A, uint32_t n);
void least_squares(double *H, double *Z, double *R, double *dX, uint8_t nrow, uint8_t state_num);
void kalman_filter_predition(double *T, double *Q, double *P, uint8_t state_num);
int inv4(const double *a, double *b);

#endif