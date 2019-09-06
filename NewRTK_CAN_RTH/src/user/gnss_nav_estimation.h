#ifndef _GNSS_NAV_ESTIMATION_H_
#define _GNSS_NAV_ESTIMATION_H_


#include "gnss_nav_utils.h"
#include "gnss_nav_proc.h"

bool_t closedform_solution(gnss_nav_context_t *context);

void
build_gnssupdate_matrix(gnss_nav_context_t *context, double *H, double *R, double *Z);

bool_t ls_gnss_update(gnss_nav_context_t *context);

bool_t lsq_dgnss_update(gnss_nav_context_t *context);

void rtk_kalman_init(gnss_nav_context_t *context);

bool_t rtk_kalman_update(gnss_nav_context_t *context);
#endif