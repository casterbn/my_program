#include <math.h>

#include "gps.h"


void gps_time_rollover_correct(double *tk)
{
	if (*tk > 302400.0)
		*tk -= 604800.0;
	else if (*tk < -302400.0)
		*tk += 604800.0;
}

bool_t get_gps_satclockcorr(const double tc, const gps_eph_channel_t *eph, double *clock_bias, double *clock_drift) 
{
	*clock_bias  = (eph->af0 + eph->af1*tc + eph->af2*tc*tc) ;  
    *clock_drift = (eph->af1 + eph->af2*tc);     
	
	return TRUE;
}


double get_gps_saterelativecorr(const double tk, const gps_eph_channel_t *eph) 
{
    
    double f = -4.442807633e-10;
	double a,n,M,E;
	uint32_t i = 0;

    a = eph->sqrta * eph->sqrta;
    n = sqrt(GRAVITY_CONSTANT / (a * a * a)); 
    n = n + eph->deltan;
    M = eph->m0 + n * tk;
    E = M;

    for(i = 0; i < 10; i++) 
        E = M + eph->ecc * sin(E);

    return f * eph->ecc * eph->sqrta * sin(E); 
}


double get_gps_groupdelay(const gps_eph_channel_t* eph) 
{
	 return eph->tgd;   
}


static void 
iter_satposvel(const double tk, const gps_eph_channel_t *eph, const double range, const double rangerate, double *sat_pos, double *sat_vel) 
{
	double a, n, M, E, cosE, sinE, u, v, r, i, cos2u, sin2u, d_u, d_r, d_i;
	double cosu, sinu, x_op, y_op, omegak, cos_omegak, sin_omegak, cosi,sini;
	double edot, vdot, udot, rdot, idotdot, vx_op, vy_op, omegadotk, tmpa, tmpb;
	uint32_t k = 0;
		
	a = eph->sqrta * eph->sqrta;
	n = sqrt(GRAVITY_CONSTANT / (a * a * a) ); 
	n  = n + eph->deltan;
	M = eph->m0 + n * tk;
	E = M;

	for (k = 0; k < 6; k++) 
		E = M + eph->ecc * sin(E);
		
    cosE = cos(E);
    sinE = sin(E);

    v = atan2( (sqrt(1.0 - eph->ecc * eph->ecc) * sinE),  (cosE - eph->ecc) );
    u = v + eph->omega;

    r = a * (1.0 - eph->ecc * cos(E));
    i = eph->i0;
    
	cos2u = cos(2.0 * u);
    sin2u = sin(2.0 * u);
    
	d_u = eph->cuc * cos2u  +  eph->cus * sin2u; 
	d_r = eph->crc * cos2u  +  eph->crs * sin2u; 
	d_i = eph->cic * cos2u  +  eph->cis * sin2u;
    
    u = u + d_u;
    r = r + d_r;
    i = i + d_i + eph->idot * tk;
    
    cosu = cos(u);
    sinu = sin(u);
    x_op = r * cosu;
    y_op = r * sinu;
    
    omegak = eph->omega0 + (eph->omegadot - EARTH_ROTATION_RATE) * tk - 
		EARTH_ROTATION_RATE * (eph->toe + range / LIGHTSPEED );

	cos_omegak = cos(omegak);
	sin_omegak = sin(omegak);
	cosi = cos(i);
	sini = sin(i);
    
    sat_pos[0] = x_op * cos_omegak - y_op * sin_omegak * cosi;
    sat_pos[1] = x_op * sin_omegak + y_op * cos_omegak * cosi;
    sat_pos[2] = y_op * sini;

    // cos2u = cos(2.0 * u);
    // sin2u = sin(2.0 * u);
    
    edot = n / (1.0 - eph->ecc * cosE);
    vdot = sinE * edot * (1.0 + eph->ecc * cos(v)) / ( sin(v) * (1.0 - eph->ecc * cosE));  
    udot = vdot + 2.0 * (eph->cus * cos2u - eph->cuc * sin2u) * vdot;
    rdot = a * eph->ecc * sinE * n / (1.0 - eph->ecc * cosE) + 2.0 * (eph->crs * cos2u - eph->crc * sin2u) * vdot;
    idotdot = eph->idot + (eph->cis * cos2u - eph->cic * sin2u) * 2.0 * vdot;  

    vx_op   = rdot * cosu - y_op * udot;
    vy_op   = rdot * sinu + x_op * udot;

    omegadotk = eph->omegadot - EARTH_ROTATION_RATE * ( 1.0 + rangerate / LIGHTSPEED );
    
    tmpa  = vx_op - y_op * cosi * omegadotk;  
    tmpb  = x_op * omegadotk + vy_op * cosi - y_op * sini * idotdot;

    sat_vel[0] = tmpa * cos_omegak - tmpb * sin_omegak;  
    sat_vel[1] = tmpa * sin_omegak + tmpb * cos_omegak;  
    sat_vel[2] = vy_op * sini + y_op * cosi * idotdot;  
}


bool_t 
get_gps_satposvel(const double time, const gps_eph_channel_t *eph, const double *user_pos, const double *user_vel, double *sat_pos, double *sat_vel)
{	
	double range = 0.07 * LIGHTSPEED;
	double range_rate = 0.0;
	double tk = 0.0;
    uint32_t iter = 0;
	double dx, dy, dz;

	tk  = time - eph->toe;    

	gps_time_rollover_correct(&tk);

	if (user_pos == NULL && user_vel == NULL) {
		double prev_range = 0.0;
		const double kRangeThreshold = 1e-4 / 2000 * LIGHTSPEED;
		do {
			prev_range = range;
			iter_satposvel(tk, eph, range, range_rate, sat_pos, sat_vel);
			dx = sat_pos[0] ;
			dy = sat_pos[1] ;
			dz = sat_pos[2] ;

			range = sqrt(dx * dx + dy * dy + dz * dz);
			range_rate = (sat_vel[0] * dx + sat_vel[1] * dy + sat_vel[2] * dz) / range; 			
		} while(fabs(range - prev_range) > kRangeThreshold);
	}
	else { 
		for (iter = 0; iter < 2; iter++) {
			iter_satposvel(tk, eph, range, range_rate, sat_pos, sat_vel);
		  
			dx = sat_pos[0] - user_pos[0];
			dy = sat_pos[1] - user_pos[1];
			dz = sat_pos[2] - user_pos[2];

			range = sqrt(dx * dx + dy * dy + dz * dz);
			range_rate = ((sat_vel[0] - user_vel[0]) * dx + (sat_vel[1] - user_vel[1]) * dy + 
				(sat_vel[2] - user_vel[2]) * dz) / range;     
		}
	}

	return TRUE;
}
