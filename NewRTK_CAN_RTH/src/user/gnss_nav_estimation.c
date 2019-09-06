#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "gnss_math.h"
#include "gnss_nav_estimation.h"
#include "gnss_geodesy.h"



static uint8_t 
single_point_channel_add_gnss_obs(gnss_nav_context_t *context)
{
	uint8_t i;
	gnss_obs_t *gnss_data = &context->rover_obs;
	uint8_t *sat_idx = gnss_data->valid_obs_idx;
	solut_channel_t *solut_channel = &context->nav_sol.solut_channel;
	
	solut_channel->tow = gnss_data->tow;
	solut_channel->used_channels = 0;

	for (i = 0; i < gnss_data->num_obs_channel; i++) {
		gnss_obs_channel_t *obs_i = &gnss_data->channel[sat_idx[i]];

		if (obs_i->signal_type != 2) { // TODO
			continue;
		}

		uint8_t j = solut_channel->used_channels;
		solut_channel->channel[j].rover_obs = obs_i;
		solut_channel->channel[j].gps_eph = &context->gnss_eph.gps_eph.svid[obs_i->prn - 1];
		solut_channel->channel[j].is_blunder = FALSE;
		solut_channel->channel[j].base_obs = NULL;

		solut_channel->used_channels++;
	}

	return (solut_channel->used_channels);
}

static double
closedform_error(const double *A, const double *y, const uint32_t sat_num)
{
	double err = 0;
	uint32_t i = 0;
	// get the estimated receiver location
	double rX = y[0];
	double rY = y[1];
	double rZ = y[2];

	// get the estimated clock bias
	double bias = -1.0 * y[3];

	double pseudorange;
	double sX, sY, sZ;

	for (i = 0; i < sat_num; ++i) {
		sX = matrix_at(A, i, 0, sat_num);
		sY = matrix_at(A, i, 1, sat_num);
		sZ = matrix_at(A, i, 2, sat_num);
		pseudorange = matrix_at(A, i, 3, sat_num);
		err += ABS(sqrt(sX * sX + sY * sY + sZ * sZ - 2 * (rX * sX + rY * sY + rZ * sZ) + 
			rX * rX + rY * rY + rZ * rZ) + bias - pseudorange);
	}

	return err;
}


bool_t closedform_solution(gnss_nav_context_t *context)
{
	double A[4 * MAX_OBS_CHANNELS] = { 0 }, B[4 * MAX_OBS_CHANNELS] = { 0 };
	double i0[MAX_OBS_CHANNELS] = { 0 }, r[MAX_OBS_CHANNELS] = { 0 };
	double A_At[16], u[4], v[4], y1[4], y2[4];
	double transmit_time, tk, clock_bias, clock_drift, relative_bias, group_delay, psr;
	double E, F, G, discriminant, lambda1, lambda2, e1, e2;
	double sat_pos[3], sat_vel[3], user_pos[3];
	uint8_t i = 0, obs_num = 0;
		//obs_num = context->rover_obs.num_obs_channel;
	//uint8_t valid_rover_obs_idx[MAX_OBS_CHANNELS] = { 0 };

	context->nav_sol.time = context->rover_obs.tow;

	obs_num = single_point_channel_add_gnss_obs(context);

	if (obs_num < 4) {
		return FALSE;
	}

	// get the satellite positions, calculate pseudoranges and build the matrices required for the
	// closed-form solution
	for (i = 0; i < obs_num; ++i) {
		single_solut_channel_t *channel_i = &context->nav_sol.solut_channel.channel[i];

		transmit_time = context->nav_sol.time - channel_i->rover_obs->psr / LIGHTSPEED;
		
		if (channel_i->rover_obs->system == SYS_GPS) {
			// Get ephemeris for the particular satellite

			tk = transmit_time - channel_i->gps_eph->toe;

			gps_time_rollover_correct(&tk);

			// Get the ephemeris corrections
			get_gps_satclockcorr(tk, channel_i->gps_eph, &clock_bias, &clock_drift);
			relative_bias = get_gps_saterelativecorr(tk, channel_i->gps_eph);
			group_delay = get_gps_groupdelay(channel_i->gps_eph);

			// compute a refined version of the transmit time
			transmit_time = transmit_time - (clock_bias + relative_bias - group_delay);

			// compute the final pseudorange, corrected for all modeled errors
			psr = channel_i->rover_obs->psr + (clock_bias + relative_bias - group_delay) * LIGHTSPEED;

			// get the satellite position and velocity
			get_gps_satposvel(transmit_time, channel_i->gps_eph, NULL, NULL, sat_pos, sat_vel);

			// form the matrices
			matrix_at(A, i, 0, obs_num) = sat_pos[0];
			matrix_at(A, i, 1, obs_num) = sat_pos[1];
			matrix_at(A, i, 2, obs_num) = sat_pos[2];
			matrix_at(A, i, 3, obs_num) = psr;

			i0[i] = 1.0;

			r[i] = 0.5 * (sat_pos[0] * sat_pos[0] + sat_pos[1] * sat_pos[1] + sat_pos[2] * sat_pos[2] - psr * psr);
		}	
	}

	matmul("TN", 4, 4, obs_num, 1.0, A, A, 0.0, A_At);
	matinv(A_At, 4);
	matmul("NT", 4, obs_num, 4, 1.0, A_At, A, 0.0, B);

	matmul("NN", 4, 1, obs_num, 1.0, B, i0, 0.0, u);
	matmul("NN", 4, 1, obs_num, 1.0, B, r, 0.0, v);

	// coefficients of the quadratic equation
	E = u[0] * u[0] + u[1] * u[1] + u[2] * u[2] - u[3] * u[3];

	F = u[0] * v[0] + u[1] * v[1] + u[2] * v[2] - u[3] * v[3] - 1.0;

	G = v[0] * v[0] + v[1] * v[1] + v[2] * v[2] - v[3] * v[3];

	discriminant = sqrt(F * F - E * G); // check to make sure this is a real number?

	// calculate the two roots of the quadratic equation
	lambda1 = (-F + discriminant) / E;
	lambda2 = (-F - discriminant) / E;

	// solutions to the equations
	matadd(lambda1, u, 1.0, v, 4, 1, y1);
	matadd(lambda2, u, 1.0, v, 4, 1, y2);

	// check the two solutions      
	e1 = closedform_error(A, y1, obs_num);
	e2 = closedform_error(A, y2, obs_num);

	if (e1 < e2) {
		user_pos[0] = y1[0];
		user_pos[1] = y1[1];
		user_pos[2] = y1[2];
	}
	else {
		user_pos[0] = y2[0];
		user_pos[1] = y2[1];
		user_pos[2] = y2[2];
	}

	ecef2plh(user_pos, context->nav_sol.pos);

	compute_MN_radius(context->nav_sol.pos[0], &context->nav_sol.filter.M, &context->nav_sol.filter.N);

	//printf("lat = %.9f \nlon = %.9f\nheight = %.3f\n", 
	//	context->nav_sol.pos[0] * RAD2DEG, context->nav_sol.pos[1] * RAD2DEG, context->nav_sol.pos[2]);

	return TRUE;
}


static void calc_sat_pvt(const double t_trans,
	const gps_eph_channel_t *eph, 
	double *pos, double *vel, 
	double *clk_bias, double *clk_drft) 
{
	double tt = t_trans; // transmit time minus sat clock
	double tk = tt - eph->toc;
	gps_time_rollover_correct(&tk);
	*clk_bias = eph->af0 + tk * (eph->af1 + tk * eph->af2);
    *clk_drft = eph->af1 + tk * eph->af2;

	tt -= *clk_bias; 
	tk = tt - eph->toe;
	gps_time_rollover_correct(&tk);
    
	double M, E, Ek, a, ecc, tmp, tmp2;
	a = eph->sqrta * eph->sqrta;
	tmp2 = sqrt(GRAVITY_CONSTANT/(a*a*a)) + eph->deltan;
	M = eph->m0 + tmp2 * tk;
	E = M;
	ecc = eph->ecc;

	uint8_t cnt = 0;
	do {
		Ek = E; 
		tmp = 1.0 - ecc*cos(Ek);
		E += (M - Ek + ecc*sin(Ek)) / tmp;
		cnt++;
		if (cnt > 20) break;
	} while (fabs(E - Ek) > 1.0E-13);

	double cosE = cos(E);
    double sinE = sin(E);
	tmp = 1.0 - ecc*cosE;

	double v = atan2( (sqrt(1.0 - ecc * ecc) * sinE),  (cosE - ecc));
    double u = v + eph->omega;
	double cos2u = cos(2.0 * u);
    double sin2u = sin(2.0 * u);
	u += eph->cuc * cos2u  +  eph->cus * sin2u;

    double r = a * tmp + eph->crc * cos2u  +  eph->crs * sin2u;
    double i = eph->i0 + eph->cic * cos2u  +  eph->cis * sin2u + eph->idot * tk;
    
    double cosu = cos(u);
    double sinu = sin(u);
    double x_op = r * cosu;
    double y_op = r * sinu;

	double omegak = eph->omega0 + (eph->omegadot - EARTH_ROTATION_RATE) * tk - 
		EARTH_ROTATION_RATE * eph->toe;

	double cos_omegak = cos(omegak);
	double sin_omegak = sin(omegak);
	double cosi = cos(i);
	double sini = sin(i);
    
    pos[0] = x_op * cos_omegak - y_op * sin_omegak * cosi;
    pos[1] = x_op * sin_omegak + y_op * cos_omegak * cosi;
    pos[2] = y_op * sini;


	double edot = tmp2 / tmp;
    double vdot = sinE * edot * (1.0 + ecc*cos(v)) / ( sin(v) * tmp);  
    double udot = vdot + 2.0 * (eph->cus * cos2u - eph->cuc * sin2u) * vdot;
    double rdot = a * ecc * edot + 2.0 * (eph->crs * cos2u - eph->crc * sin2u) * vdot;
    double idotdot = eph->idot + (eph->cis * cos2u - eph->cic * sin2u) * 2.0 * vdot;  

    double vx_op   = rdot * cosu - y_op * udot;
    double vy_op   = rdot * sinu + x_op * udot;

    double omegadotk = eph->omegadot - EARTH_ROTATION_RATE;
    
    double tmpa  = vx_op - y_op * cosi * omegadotk;  
    double tmpb  = x_op * omegadotk + vy_op * cosi - y_op * sini * idotdot;

    vel[0] = tmpa * cos_omegak - tmpb * sin_omegak;
    vel[1] = tmpa * sin_omegak + tmpb * cos_omegak;  
    vel[2] = vy_op * sini + y_op * cosi * idotdot; 

	*clk_bias -= 2.0 * r * rdot / LIGHTSPEED / LIGHTSPEED; 
}


static void calculate_satellite_pvt(solut_channel_t *sol_channels)
{

	for (uint8_t i = 0; i < sol_channels->used_channels; i++) {
		single_solut_channel_t *channel_i = &sol_channels->channel[i];
		double transmit_time = sol_channels->tow - channel_i->rover_obs->psr / LIGHTSPEED;
		double clock_bias, clock_drift;

		if (channel_i->rover_obs->system == SYS_GPS) {
			calc_sat_pvt(transmit_time, channel_i->gps_eph, channel_i->rover_sat_pos, channel_i->rover_sat_vel, 
				&clock_bias, &clock_drift);

			channel_i->rover_psr = channel_i->rover_obs->psr + (clock_bias - channel_i->gps_eph->tgd) * LIGHTSPEED;
			channel_i->rover_prr = -1.0 * channel_i->rover_obs->doppler * L1_WAVELENGTH + clock_drift * LIGHTSPEED;

			// get_sataziele(user_pos, sat_pos, &sat_azimuth, &sat_elevation);
			// tropo_corr = get_tropocorr(sat_elevation, 1010.0, 15.0, 3.3645810873918274);
			// psr -= tropo_corr;
		}
	}
}


static void update_sat_ele_azi(solut_t *nav_sol)
{
	double user_pos[3];
	plh2ecef(nav_sol->pos, user_pos);

	for (uint8_t i = 0; i < nav_sol->solut_channel.used_channels; i++) {
		single_solut_channel_t *channel_i = &nav_sol->solut_channel.channel[i];
		double *sat_pos = channel_i->rover_sat_pos;

		get_sataziele(user_pos, sat_pos, &channel_i->rover_sat_azimuth, &channel_i->rover_sat_elevation);
	}
}

static void compute_psr_residual(solut_t *nav_sol, double *z)
{
	double user_pos[3];
	double tropo_corr = 0.0, range;

	plh2ecef(nav_sol->pos, user_pos);

	for (uint8_t i = 0; i < nav_sol->solut_channel.used_channels; i++) {
		single_solut_channel_t *channel_i = &nav_sol->solut_channel.channel[i];
		double *sat_pos = channel_i->rover_sat_pos;

		tropo_corr = get_tropocorr(channel_i->rover_sat_elevation, 1010.0, 15.0, 3.3645810873918274);
		//channel_i->rover_psr -= tropo_corr;

		range = compute_range(user_pos, sat_pos);
		range += nav_sol->clock_bias;

		z[i] = range - channel_i->rover_psr + tropo_corr;
	}
}

static void compute_prr_residual(solut_t *nav_sol, double *z)
{
	double user_pos[3];
	double range, range_rate;
	float *user_vel = nav_sol->vel;

	plh2ecef(nav_sol->pos, user_pos);

	for (uint8_t i = 0; i < nav_sol->solut_channel.used_channels; i++) {
		single_solut_channel_t *channel_i = &nav_sol->solut_channel.channel[i];
		double *sat_pos = channel_i->rover_sat_pos;
		double *sat_vel = channel_i->rover_sat_vel;

		range = compute_range(user_pos, sat_pos);
		range_rate = ((sat_vel[0] - user_vel[0]) * (sat_pos[0] - user_pos[0])
			+ (sat_vel[1] - user_vel[1]) * (sat_pos[1] - user_pos[1])
			+ (sat_vel[2] - user_vel[2]) * (sat_pos[2] - user_pos[2])) / range;

		range_rate += nav_sol->clock_drift;
		z[i] = range_rate - channel_i->rover_prr;
	}
}


static void
build_update_matrix(solut_channel_t *solut_channel, double *H)
{
	uint8_t obs_num = solut_channel->used_channels;
	uint8_t max_row_num = obs_num*NUM_OBSERVABLE;
	
	uint8_t i;
	
	for (i = 0; i < solut_channel->used_channels; i++) {
		single_solut_channel_t *channel_i = &solut_channel->channel[i];
		// double transmit_time = solut_channel->tow - channel_i->rover_obs->psr / LIGHTSPEED;
		// double clock_bias, clock_drift, sat_azimuth, sat_elevation, tropo_corr, psr, prr;
		// double sat_pos[3], sat_vel[3];
		double sat_elevation = channel_i->rover_sat_elevation; 
		double sat_azimuth = channel_i->rover_sat_azimuth;

		// Pseudorange
		matrix_at(H, i, 0, max_row_num) = -cos(sat_elevation) * sin(sat_azimuth);
		matrix_at(H, i, 1, max_row_num) = -cos(sat_elevation) * cos(sat_azimuth);
		matrix_at(H, i, 2, max_row_num) = -sin(sat_elevation);
		matrix_at(H, i, 3, max_row_num) = 1.0;

		// matrix_at(R, 2 * i, 2 * i, max_row_num) = 25.0 / sin(sat_elevation) / sin(sat_elevation);
		// matrix_at(R, 2 * i + 1, 2 * i + 1, max_row_num) = 0.04;
		//matrix_at(W, i, i, max_row_num) = sin(sat_elevation) * sin(sat_elevation) / 25.0;
		// matrix_at(W, 2 * i + 1, 2 * i + 1, max_row_num) = 1.0 / 0.04;

	}
}


static void calc_satellite_pos_vel_clk(gnss_nav_context_t *context)
{
	solut_channel_t *solut_channel = &context->nav_sol.solut_channel;
	uint8_t obs_num = solut_channel->used_channels;
	uint8_t max_row_num = obs_num * NUM_OBSERVABLE;
	double user_pos[3], user_vel[3];
	uint8_t i;
	
	plh2ecef(context->nav_sol.pos, user_pos);
	enu2xyz(context->nav_sol.pos[0], context->nav_sol.pos[1], context->nav_sol.vel, user_vel);

	for (i = 0; i < solut_channel->used_channels; i++) {
		single_solut_channel_t *channel_i = &context->nav_sol.solut_channel.channel[i];
		double transmit_time = solut_channel->tow - channel_i->rover_obs->psr / LIGHTSPEED;
		double tk;
		double clock_bias, clock_drift, relative_bias, group_delay, sat_azimuth, sat_elevation, tropo_corr, psr, prr;
		double sat_pos[3], sat_vel[3];
		double range, range_rate;

		if (channel_i->rover_obs->system == SYS_GPS) {
			tk = transmit_time - channel_i->gps_eph->toe;
			gps_time_rollover_correct(&tk);

			get_gps_satclockcorr(tk, channel_i->gps_eph, &clock_bias, &clock_drift);
			relative_bias = get_gps_saterelativecorr(tk, channel_i->gps_eph);
			group_delay = get_gps_groupdelay(channel_i->gps_eph);

			transmit_time = transmit_time - (clock_bias + relative_bias - group_delay);

			get_gps_satposvel(transmit_time, channel_i->gps_eph, user_pos, user_vel, 
				channel_i->rover_sat_pos, channel_i->rover_sat_vel);

			channel_i->rover_psr = channel_i->rover_obs->psr + (clock_bias + relative_bias - group_delay) * LIGHTSPEED;
			channel_i->rover_prr = channel_i->rover_obs->doppler + clock_drift * LIGHTSPEED;

			/*get_sataziele(user_pos, sat_pos, &sat_azimuth, &sat_elevation);
			tropo_corr = get_tropocorr(sat_elevation, 1010.0, 15.0, 3.3645810873918274);
			psr -= tropo_corr;*/
		}
	}
}


void
build_gnssupdate_matrix(gnss_nav_context_t *context, double *H, double *W, double *Z)
{
	solut_channel_t *solut_channel = &context->nav_sol.solut_channel;
	uint8_t obs_num = solut_channel->used_channels;
	uint8_t max_row_num = obs_num*NUM_OBSERVABLE;
	double user_pos[3], user_vel[3];
	uint8_t i;
	//gnss_nav_context_t *context = Gpsdata

	plh2ecef(context->nav_sol.pos, user_pos);
	enu2xyz(context->nav_sol.pos[0], context->nav_sol.pos[1], context->nav_sol.vel, user_vel);

	for (i = 0; i < solut_channel->used_channels; i++) {
		single_solut_channel_t *channel_i = &context->nav_sol.solut_channel.channel[i];
		double transmit_time = solut_channel->tow - channel_i->rover_obs->psr / LIGHTSPEED;
		double tk;
		double clock_bias, clock_drift, relative_bias, group_delay, sat_azimuth, sat_elevation, tropo_corr, psr, prr;
		double sat_pos[3], sat_vel[3];
		double range, range_rate;

		if (channel_i->rover_obs->system == SYS_GPS) {
			tk = transmit_time - channel_i->gps_eph->toe;
			gps_time_rollover_correct(&tk);

			get_gps_satclockcorr(tk, channel_i->gps_eph, &clock_bias, &clock_drift);
			relative_bias = get_gps_saterelativecorr(tk, channel_i->gps_eph);
			group_delay = get_gps_groupdelay(channel_i->gps_eph);

			transmit_time = transmit_time - (clock_bias + relative_bias - group_delay);

			get_gps_satposvel(transmit_time, channel_i->gps_eph, user_pos, user_vel, sat_pos, sat_vel);

			psr = channel_i->rover_obs->psr + (clock_bias + relative_bias - group_delay) * LIGHTSPEED;
			 prr = 1.0 * channel_i->rover_obs->doppler + clock_drift * LIGHTSPEED;

			get_sataziele(user_pos, sat_pos, &sat_azimuth, &sat_elevation);
			tropo_corr = get_tropocorr(sat_elevation, 1010.0, 15.0, 3.3645810873918274);
			psr -= tropo_corr;

			range = compute_range(user_pos, sat_pos);
			 range_rate = ((sat_vel[0] - user_vel[0]) * (sat_pos[0] - user_pos[0])
			 	+ (sat_vel[1] - user_vel[1]) * (sat_pos[1] - user_pos[1])
			 	+ (sat_vel[2] - user_vel[2]) * (sat_pos[2] - user_pos[2])) / range;

			// Pseudorange
			matrix_at(H, i, 0, max_row_num) = -cos(sat_elevation) * sin(sat_azimuth);
			matrix_at(H, i, 1, max_row_num) = -cos(sat_elevation) * cos(sat_azimuth);
			matrix_at(H, i, 2, max_row_num) = -sin(sat_elevation);
			matrix_at(H, i, 3, max_row_num) = 1.0;

			// Doppler
			 matrix_at(H, 2 * i + 1, 4, max_row_num) = -cos(sat_elevation) * sin(sat_azimuth);
			 matrix_at(H, 2 * i + 1, 5, max_row_num) = -cos(sat_elevation) * cos(sat_azimuth);
			 matrix_at(H, 2 * i + 1, 6, max_row_num) = -sin(sat_elevation);
			 matrix_at(H, 2 * i + 1, 7, max_row_num) = 1.0;

			// matrix_at(R, 2 * i, 2 * i, max_row_num) = 25.0 / sin(sat_elevation) / sin(sat_elevation);
			// matrix_at(R, 2 * i + 1, 2 * i + 1, max_row_num) = 0.04;
			matrix_at(W, i, i, max_row_num) = sin(sat_elevation) * sin(sat_elevation) / 25.0;
			// matrix_at(W, 2 * i + 1, 2 * i + 1, max_row_num) = 1.0 / 0.04;

			range += context->nav_sol.clock_bias;
			range_rate += context->nav_sol.clock_drift;

			Z[2*i] = range - psr;
			Z[2 * i + 1] = range_rate - prr;
		}
	}
}


static void compute_dop_and_cov(double *H, double *W, uint8_t nrow, uint8_t state_num, solut_t *sol)
{
	double Ht_H[MAX_GNSS_STATE * MAX_GNSS_STATE] = { 0 };
	double Ht_W[MAX_GNSS_STATE * MAX_OBS_NUM] = {0};
	double Ht_H1[MAX_GNSS_STATE * MAX_GNSS_STATE] = { 0 };

	matmul("TN", state_num, state_num, nrow, 1.0, H, H, 0.0, Ht_H1);
	// matinv(Ht_H, state_num);
	inv4(Ht_H1, Ht_H);

	sol->hdop = (float)sqrt(matrix_at(Ht_H, 0, 0, state_num) + matrix_at(Ht_H, 1, 1, state_num));
	sol->vdop = (float)sqrt(matrix_at(Ht_H, 2, 2, state_num));

	matmul("TN", state_num, nrow, nrow, 1.0, H, W, 0.0, Ht_W);
	matmul("NN", state_num, state_num, nrow, 1.0, Ht_W, H, 0.0, Ht_H1);
	// matinv(Ht_H, state_num);
	inv4(Ht_H1, Ht_H);

	sol->std_pos[0] = (float)sqrt(matrix_at(Ht_H, 0, 0, state_num));
	sol->std_pos[1] = (float)sqrt(matrix_at(Ht_H, 1, 1, state_num));
	sol->std_pos[2] = (float)sqrt(matrix_at(Ht_H, 2, 2, state_num));
}

bool_t ls_gnss_update(gnss_nav_context_t *context)
{
	uint8_t obs_num = 0, iter = 0;

	double H[MAX_OBS_NUM * MAX_GNSS_STATE] = { 0 };
	double Z[MAX_OBS_NUM] = { 0 };
	double W[MAX_OBS_NUM * MAX_OBS_NUM] = { 0 };
	double dX[MAX_GNSS_STATE] = { 0 };


	context->nav_sol.time = context->rover_obs.tow;

	// obs_num = gnss_data_prefilter(&context->rover_obs, valid_rover_obs_idx,
	// 	&context->gnss_eph.gps_eph, &context->base->gps_eph);
	//obs_num = context->rover_obs.num_obs_channel;

	obs_num = single_point_channel_add_gnss_obs(context);

	if (obs_num < 4) {
		return FALSE;
	}

	///calculate_satellite_pvt(&context->nav_sol.solut_channel);
	calc_satellite_pos_vel_clk(context);

	for (iter = 0; iter < 10; iter++) {
		solut_t *nav_sol = &context->nav_sol;

		 update_sat_ele_azi(nav_sol);
		 compute_psr_residual(nav_sol, Z);
		 build_update_matrix(&context->nav_sol.solut_channel, H);

		//build_gnssupdate_matrix(context, H, W, Z);
		
		least_squares(H, Z, W, dX, obs_num*NUM_OBSERVABLE, MAX_GNSS_STATE);

		context->nav_sol.pos[0] -= dX[1] / (context->nav_sol.filter.M + context->nav_sol.pos[2]);
		context->nav_sol.pos[1] -= dX[0] / (context->nav_sol.filter.N + context->nav_sol.pos[2]) / cos(context->nav_sol.pos[0]);
		context->nav_sol.pos[2] -= dX[2];
		context->nav_sol.clock_bias -= dX[3];
		
		/*context->nav_sol.vel[0] -= dX[4];
		context->nav_sol.vel[1] -= dX[5];
		context->nav_sol.vel[2] -= dX[6];
		context->nav_sol.clock_drift -= dX[7];*/

		if ((fabs(dX[0]) + fabs(dX[1]) + fabs(dX[2])) < 0.0001) {
			compute_dop_and_cov(H, W, obs_num*NUM_OBSERVABLE, MAX_GNSS_STATE, nav_sol);

			compute_MN_radius(nav_sol->pos[0], &nav_sol->filter.M, &nav_sol->filter.N);

			// *** Vel estimation
			update_sat_ele_azi(nav_sol);
			build_update_matrix(&context->nav_sol.solut_channel, H);
			compute_prr_residual(nav_sol, Z);

			least_squares(H, Z, W, dX, obs_num*NUM_OBSERVABLE, MAX_GNSS_STATE);

			context->nav_sol.vel[0] -= dX[0];
			context->nav_sol.vel[1] -= dX[1];
			context->nav_sol.vel[2] -= dX[2];
			context->nav_sol.clock_drift -= dX[3];

			break;
		}
	}

	return TRUE;
}


static void  channel_remove_idx(solut_channel_t *solut_channel, uint8_t idx)
{
	uint8_t i;

	solut_channel->prn_channel_idx[solut_channel->channel[idx].prn - 1] = 0xFF;

	//context.nav_sol.filter.dN[solut_channel->channel[idx].prn - 1] = 0.0;

	if (idx != solut_channel->used_channels - 1) {
		for (i = idx; i < solut_channel->used_channels - 1; i++) {
			uint8_t prn = solut_channel->channel[i + 1].prn;

			solut_channel->channel[i] = solut_channel->channel[i + 1];
			solut_channel->prn_channel_idx[prn - 1] = i;
		}
	}

	solut_channel->used_channels--;
}

static uint8_t dgnss_channel_add_gnss_obs(gnss_nav_context_t *context)
{
	uint8_t i, j;
	solut_channel_t *solut_channel = &context->rtk_sol.solut_channel;
	uint8_t cnt = 0;

	gnss_obs_t *rover_data = context->ptr_rover_obs;
	uint8_t rover_sat_num = context->ptr_rover_obs->num_obs_channel;
	uint8_t *rover_sat_idx = context->ptr_rover_obs->valid_obs_idx;

	gnss_obs_t *base_data = &context->base_obs;
	uint8_t base_sat_num = context->base_obs.num_obs_channel;
	uint8_t *base_sat_idx = context->base_obs.valid_obs_idx;

	solut_channel->tow = rover_data->tow;

	for (i = 0; i < rover_sat_num; i++) {
		gnss_obs_channel_t *rover_obs_i = &rover_data->channel[rover_sat_idx[i]];

		for (j = 0; j < base_sat_num; j++) {
			gnss_obs_channel_t *base_obs_j = &base_data->channel[base_sat_idx[j]];

			if (rover_obs_i->prn == base_obs_j->prn &&
				rover_obs_i->signal_type == base_obs_j->signal_type) {
				uint8_t *channel_idx = &solut_channel->prn_channel_idx[rover_obs_i->prn - 1];

				if (*channel_idx == 0xFF) {
					*channel_idx = solut_channel->used_channels;
					solut_channel->used_channels++;
					square_append(context->rtk_sol.filter.P, 1.0e6, solut_channel->used_channels + MAX_GNSS_STATE);
					solut_channel->channel[*channel_idx].prn = rover_obs_i->prn;
				}

				solut_channel->channel[*channel_idx].is_updated = TRUE;
				solut_channel->channel[*channel_idx].rover_obs = rover_obs_i;
				solut_channel->channel[*channel_idx].gps_eph = &context->gnss_eph.gps_eph.svid[rover_obs_i->prn - 1];
				solut_channel->channel[*channel_idx].is_blunder = FALSE;
				solut_channel->channel[*channel_idx].base_obs = base_obs_j;

				cnt++;
			}
		}
	}

	while (cnt < solut_channel->used_channels) {
		for (i = 0; i < solut_channel->used_channels; i++) {
			if (!solut_channel->channel[i].is_updated) {
				channel_remove_idx(solut_channel, i);
				square_del(context->rtk_sol.filter.P, solut_channel->used_channels + MAX_GNSS_STATE, MAX_GNSS_STATE + i);
				break;
			}
		}
	}

	for (i = 0; i < solut_channel->used_channels; i++) {
		solut_channel->channel[i].is_updated = FALSE;
	}

	solut_channel->used_channels = cnt;

	return cnt;
}

static void build_dgps_update_matrix(gnss_nav_context_t *context, double *H, double *R, double *Z)
{
	solut_channel_t *solut_channel = &context->rtk_sol.solut_channel;
	uint8_t obs_num = solut_channel->used_channels;
	uint8_t max_row_num = obs_num * NUM_OBSERVABLE;
	double user_pos[3], user_vel[3];
	double  base_pos[3], base_vel[3] = { 0 };
	uint8_t i;
	double transmit_time, tk;
	double clock_bias, clock_drift, relative_bias, group_delay, sat_azimuth, sat_elevation, tropo_corr;
	double psr_rover, prr_rover, psr_base, prr_base;
	double sat_pos[3];
	double sat_vel[3];
	double range_rover, range_rate_rover, range_base, range_rate_base;

	plh2ecef(context->rtk_sol.pos, user_pos);
	enu2xyz(context->rtk_sol.pos[0], context->rtk_sol.pos[1], context->rtk_sol.vel, user_vel);

	plh2ecef(context->rtk_sol.base_pos, base_pos);

	for (i = 0; i < obs_num; i++) {
		single_solut_channel_t *channel_i = &solut_channel->channel[i];

		if (channel_i->rover_obs->system == SYS_GPS) {

			transmit_time = solut_channel->tow - channel_i->rover_obs->psr / LIGHTSPEED;
			tk = transmit_time - channel_i->gps_eph->toe;

			gps_time_rollover_correct(&tk);

			get_gps_satclockcorr(tk, channel_i->gps_eph, &clock_bias, &clock_drift);
			relative_bias = get_gps_saterelativecorr(tk, channel_i->gps_eph);
			group_delay = get_gps_groupdelay(channel_i->gps_eph);

			transmit_time = transmit_time - (clock_bias + relative_bias - group_delay);

			get_gps_satposvel(transmit_time, channel_i->gps_eph, user_pos, user_vel, sat_pos, sat_vel);

			psr_rover = channel_i->rover_obs->psr + (clock_bias + relative_bias - group_delay) * LIGHTSPEED;
			/*prr_rover = -1.0 * channel_i->rover_obs->doppler * L1_WAVELENGTH + clock_drift * LIGHTSPEED;*/

			get_sataziele(user_pos, sat_pos, &sat_azimuth, &sat_elevation);
			tropo_corr = get_tropocorr(sat_elevation, 1010.0, 15.0, 3.3645810873918274);
			psr_rover -= tropo_corr;

			range_rover = compute_range(user_pos, sat_pos);
			/*range_rate_rover = ((sat_vel[0] - user_vel[0]) * (sat_pos[0] - user_pos[0])
				+ (sat_vel[1] - user_vel[1]) * (sat_pos[1] - user_pos[1])
				+ (sat_vel[2] - user_vel[2]) * (sat_pos[2] - user_pos[2])) / range_rover;*/

			range_rover += context->nav_sol.clock_bias;
			//range_rate_rover += context->nav_sol.clock_drift;

			// Pseudorange
			matrix_at(H, NUM_OBSERVABLE * i, 0, max_row_num) = -cos(sat_elevation) * sin(sat_azimuth);
			matrix_at(H, NUM_OBSERVABLE * i, 1, max_row_num) = -cos(sat_elevation) * cos(sat_azimuth);
			matrix_at(H, NUM_OBSERVABLE * i, 2, max_row_num) = -sin(sat_elevation);
			matrix_at(H, NUM_OBSERVABLE * i, 3, max_row_num) = 1.0;

			// Doppler
		/*	matrix_at(H, NUM_OBSERVABLE * i + 1, 3, max_row_num) = -cos(sat_elevation) * sin(sat_azimuth);
			matrix_at(H, NUM_OBSERVABLE * i + 1, 4, max_row_num) = -cos(sat_elevation) * cos(sat_azimuth);
			matrix_at(H, NUM_OBSERVABLE * i + 1, 5, max_row_num) = -sin(sat_elevation);
			matrix_at(H, NUM_OBSERVABLE * i + 1, 7, max_row_num) = 1.0;*/

			matrix_at(R, NUM_OBSERVABLE * i, NUM_OBSERVABLE * i, max_row_num) = 4.0 / sin(sat_elevation) / sin(sat_elevation);
			//matrix_at(R, NUM_OBSERVABLE * i + 1, NUM_OBSERVABLE * i + 1, max_row_num) = 0.04;

			if (channel_i->base_obs->system == SYS_GPS) {
				transmit_time = solut_channel->tow - channel_i->base_obs->psr / LIGHTSPEED;
				tk = transmit_time - channel_i->gps_eph->toe;

				gps_time_rollover_correct(&tk);

				get_gps_satclockcorr(tk, channel_i->gps_eph, &clock_bias, &clock_drift);
				relative_bias = get_gps_saterelativecorr(tk, channel_i->gps_eph);
				group_delay = get_gps_groupdelay(channel_i->gps_eph);

				transmit_time = transmit_time - (clock_bias + relative_bias - group_delay);

				get_gps_satposvel(transmit_time, channel_i->gps_eph, base_pos, base_vel, sat_pos, sat_vel);

				psr_base = channel_i->base_obs->psr + (clock_bias + relative_bias - group_delay) * LIGHTSPEED;
				//prr_base = -1.0 * channel_i->base_obs->doppler * L1_WAVELENGTH + clock_drift * LIGHTSPEED;

				get_sataziele(base_pos, sat_pos, &sat_azimuth, &sat_elevation);
				tropo_corr = get_tropocorr(sat_elevation, 1010.0, 15.0, 3.3645810873918274);
				psr_base -= tropo_corr;

				range_base = compute_range(base_pos, sat_pos);
			/*	range_rate_base = ((sat_vel[0]) * (sat_pos[0] - base_pos[0])
					+ (sat_vel[1]) * (sat_pos[1] - base_pos[1])
					+ (sat_vel[2]) * (sat_pos[2] - base_pos[2])) / range_base;*/

				Z[NUM_OBSERVABLE * i] = (range_rover - range_base) - (psr_rover - psr_base);
				//Z[2 * i + 1] = (range_rate_rover - range_rate_base) - (prr_rover - prr_base);
			}
		}
	}
}

bool_t lsq_dgnss_update(gnss_nav_context_t *context)
{
	uint8_t iter, rover_obs_num, base_obs_num, valid_obs_num;
	/*uint8_t valid_rover_obs_idx[MAX_OBS_CHANNELS] = { 0 };
	uint8_t valid_base_obs_idx[MAX_OBS_CHANNELS] = { 0 };*/

	double H[MAX_OBS_NUM * MAX_GNSS_STATE] = { 0 };
	double Z[MAX_OBS_NUM] = { 0 };
	double R[MAX_OBS_NUM * MAX_OBS_NUM] = { 0 };
	double dX[MAX_GNSS_STATE] = { 0 };

	context->rtk_sol.time = context->ptr_rover_obs->tow;

	if (!context->is_rover_updated) {
		context->ptr_rover_obs->num_obs_channel = gnss_data_prefilter(context->ptr_rover_obs, context->ptr_rover_obs->valid_obs_idx,
			&context->gnss_eph);
	}

	context->base_obs.num_obs_channel = gnss_data_prefilter(&context->base_obs, context->base_obs.valid_obs_idx,
		&context->gnss_eph);

	valid_obs_num = dgnss_channel_add_gnss_obs(context);

	if (valid_obs_num < 4) {
		return FALSE;
	}

	for (iter = 0; iter < 10; iter++) {

		build_dgps_update_matrix(context, H, R, Z);

		least_squares(H, Z, R, dX, valid_obs_num * NUM_OBSERVABLE, MAX_GNSS_STATE);

		context->rtk_sol.pos[0] -= dX[1] / (context->rtk_sol.filter.M + context->rtk_sol.pos[2]);
		context->rtk_sol.pos[1] -= dX[0] / (context->rtk_sol.filter.N + context->rtk_sol.pos[2]) / cos(context->rtk_sol.pos[0]);
		context->rtk_sol.pos[2] -= dX[2];
		context->rtk_sol.clock_bias -= dX[3];

		/*context->nav_sol.vel[0] -= dX[3];
		context->nav_sol.vel[1] -= dX[4];
		context->nav_sol.vel[2] -= dX[5];*/

		/*context->nav_sol.clock_bias -= dX[6];
		context->nav_sol.clock_drift -= dX[7];*/

		if ((fabs(dX[0]) + fabs(dX[1]) + fabs(dX[2])) < 0.0001) {
			compute_MN_radius(context->rtk_sol.pos[0], &context->rtk_sol.filter.M, &context->rtk_sol.filter.N);

			break;
		}
	}

	return TRUE;

}


static void gnss_kalman_build_prediction_matrix(double time_interval, double *T, double *Q, const uint8_t num_states)
{
	double t2 = time_interval * time_interval;
	double t3 = t2 * time_interval;
	double pi2 = PI * PI;
	double vel_psd = 0.4*0.4;

	double white_frquency_noise = 0.5 * 1.0E-14 * LIGHTSPEED * LIGHTSPEED;
	//double flicker_frequency_noise = 1.0E-10 * LIGHTSPEED;
	double rw_frequency_noise = 2.0 * pi2 * 1.0E-15 * LIGHTSPEED * LIGHTSPEED;

	eye(T, num_states);

	// relate position and velocity
	/*T[0 + 3 * num_states] = time_interval;
	T[1 + 4 * num_states] = time_interval;
	T[2 + 5 * num_states] = time_interval;*/

	// relate clock bias and drift
	//T[6 + 7 * num_states] = time_interval;

	// position states
	Q[0] = vel_psd * t3 / 3.0;
	Q[1 + 1 * num_states] = vel_psd * t3 / 3.0;
	Q[2 + 2 * num_states] = vel_psd * t3 / 3.0;

	// velocity states
	/*Q[3 + 3 * num_states] = vel_psd * time_interval;
	Q[4 + 4 * num_states] = vel_psd * time_interval;
	Q[5 + 5 * num_states] = vel_psd * time_interval;*/

	// position/velocity correlations
	/*Q[0 + 3 * num_states] = Q[3 + 0 * num_states] = vel_psd * t2 / 2.0;
	Q[1 + 4 * num_states] = Q[4 + 1 * num_states] = vel_psd * t2 / 2.0;
	Q[2 + 5 * num_states] = Q[5 + 2 * num_states] = vel_psd * t2 / 2.0;*/

	// clock bias
	Q[3 + 3 * num_states] = white_frquency_noise * time_interval + rw_frequency_noise * t3 / 3.0;

	// clock drift
	//Q[7 + 7 * num_states] = rw_frequency_noise * time_interval;

	// clock bias/drift correlation
	//Q[6 + 7 * num_states] = Q[7 + 6 * num_states] = rw_frequency_noise * t2 * 0.5;
}

static void gnss_kalman_propagagtion(double time_interval, double *T, double *Q, double *P, const uint8_t num_states)
{
	gnss_kalman_build_prediction_matrix(time_interval, T, Q, num_states);

	kalman_filter_predition(T, Q, P, num_states);
}

//static int
//obs_compare(const void * a, const void * b)
//{
//	single_solut_channel_t *channel_a = &nav_context.sol_channel.channel[*(uint8_t *)a];
//	single_solut_channel_t *channel_b = &sol_channel->channel[*(uint8_t *)b];
//
//	if (channel_a->rover_obs->cn0 <= channel_b->rover_obs->cn0)
//		return 1;
//	else
//		return -1;
//}

static bool_t
kalman_sequential_update(double *dX, double *P, double *H, double *R, double *Z, uint8_t state_num)
{
	double P_Ht[MAX_KF_STATE] = { 0 };
	double K[MAX_KF_STATE] = { 0 };
	double tmpP1[MAX_KF_STATE * MAX_KF_STATE] = { 0 };
	double tmpP2[MAX_KF_STATE * MAX_KF_STATE] = { 0 };
	double C;
	double test_vector;
	uint32_t i;

	matmul("NT", state_num, 1, state_num, 1.0, P, H, 0.0, P_Ht);
	matmul("NN", 1, 1, state_num, 1.0, H, P_Ht, 0.0, &C);

	C += (*R);

	test_vector = *Z / C;

	if (ABS(test_vector / sqrt(C)) > 330.0) {
		return FALSE;
	}

	for (i = 0; i < state_num; i++) {
		K[i] = P_Ht[i] / C;
	}

	for (i = 0; i < state_num; i++) {
		dX[i] = K[i] * (*Z);
	}

	matmul("NN", state_num, state_num, 1, 1.0, K, H, 0.0, tmpP1);
	matmul("NN", state_num, state_num, state_num, 1.0, tmpP1, P, 0.0, tmpP2);
	matadd(1.0, P, -1.0, tmpP2, state_num, state_num, P);

	return TRUE;
}

static void rtk_state_correction(rtk_solut_t *sol, double *dX, uint8_t valid_obs_num)
{
	uint8_t i;

	sol->pos[2] -= dX[2];
	sol->pos[0] -= dX[1] / (sol->filter.M + sol->pos[2]);
	sol->pos[1] -= dX[0] / (sol->filter.N + sol->pos[2]) / cos(sol->pos[0]);

	/*sol->vel[0] -= dX[3];
	sol->vel[1] -= dX[4];
	sol->vel[2] -= dX[5];*/

	sol->clock_bias -= dX[3];
	//sol->clock_drift -= dX[7];

	for (i = 0; i < valid_obs_num; i++) {
		single_solut_channel_t *channel_i = &sol->solut_channel.channel[i];
		sol->filter.dN[channel_i->prn - 1] -= dX[i + MAX_GNSS_STATE];
	}

}


void rtk_kalman_init(gnss_nav_context_t *context)
{
	rtk_filter_t *filter = &context->rtk_sol.filter;

	memset(filter->P, 0, MAX_KF_STATE * MAX_KF_STATE * sizeof(double));
	filter->P[0 + 0 * MAX_GNSS_STATE] = 0.25;
	filter->P[1 + 1 * MAX_GNSS_STATE] = 0.25;
	filter->P[2 + 2 * MAX_GNSS_STATE] = 0.25;

	//filter->P[3 + 3 * MAX_GNSS_STATE] = 0.04;//0.25;
	filter->P[4 + 4 * MAX_GNSS_STATE] = 10000.0; // 0.04;//0.25;
	//filter->P[5 + 5 * MAX_GNSS_STATE] = 0.04;//0.25;

	//filter->P[6 + 6 * MAX_GNSS_STATE] = 10000.0;//10000.0;
	//filter->P[7 + 7 * MAX_GNSS_STATE] = 1000.0;//10000.0;

	memset(filter->dN, 0, MAX_NUMBER_SAT * sizeof(double));

	memset(&context->rtk_sol.solut_channel, 0, sizeof(solut_channel_t));
	memset(&context->rtk_sol.solut_channel.prn_channel_idx, 0xFF, sizeof(uint8_t)* MAX_NUMBER_SAT);

}

static void build_satellite_pos_vel(rtk_solut_t *sol)
{
	solut_channel_t *solut_channel = &sol->solut_channel;
	uint8_t i;
	double user_pos[3], user_vel[3];
	double  base_pos[3], base_vel[3] = { 0 };
	uint8_t obs_num = solut_channel->used_channels;
	double transmit_time, tk;
	double clock_bias, clock_drift, relative_bias, group_delay, sat_azimuth, sat_elevation, tropo_corr;

	plh2ecef(sol->pos, user_pos);
	enu2xyz(sol->pos[0], sol->pos[1], sol->vel, user_vel);

	plh2ecef(sol->base_pos, base_pos);

	for (i = 0; i < obs_num; i++) {
		single_solut_channel_t *channel_i = &solut_channel->channel[i];

		channel_i->rover_phase_prev = channel_i->rover_phase;
		channel_i->rover_prr_prev = channel_i->rover_prr;

		transmit_time = solut_channel->tow - channel_i->rover_obs->psr / LIGHTSPEED;
		tk = transmit_time - channel_i->gps_eph->toe;

		gps_time_rollover_correct(&tk);

		get_gps_satclockcorr(tk, channel_i->gps_eph, &clock_bias, &clock_drift);
		relative_bias = get_gps_saterelativecorr(tk, channel_i->gps_eph);
		group_delay = get_gps_groupdelay(channel_i->gps_eph);

		transmit_time = transmit_time - (clock_bias + relative_bias - group_delay);

		get_gps_satposvel(transmit_time, channel_i->gps_eph, user_pos, user_vel, channel_i->rover_sat_pos, channel_i->rover_sat_vel);

		channel_i->rover_psr = channel_i->rover_obs->psr + (clock_bias + relative_bias - group_delay) * LIGHTSPEED;
		//channel_i->rover_prr = -1.0 * channel_i->rover_obs->doppler * L1_WAVELENGTH + clock_drift * LIGHTSPEED;
		channel_i->rover_phase = 1.0 * channel_i->rover_obs->phase + (clock_bias + relative_bias - group_delay) * LIGHTSPEED;

		get_sataziele(user_pos, channel_i->rover_sat_pos, &sat_azimuth, &sat_elevation);
		channel_i->rover_sat_azimuth = sat_azimuth;
		channel_i->rover_sat_elevation = sat_elevation;

		tropo_corr = get_tropocorr(sat_elevation, 1010.0, 15.0, 3.3645810873918274);
		channel_i->rover_psr -= tropo_corr;
		channel_i->rover_phase -= tropo_corr;


		if (channel_i->base_obs) {
			transmit_time = solut_channel->tow - channel_i->base_obs->psr / LIGHTSPEED;
			tk = transmit_time - channel_i->gps_eph->toe;

			gps_time_rollover_correct(&tk);

			get_gps_satclockcorr(tk, channel_i->gps_eph, &clock_bias, &clock_drift);
			relative_bias = get_gps_saterelativecorr(tk, channel_i->gps_eph);
			group_delay = get_gps_groupdelay(channel_i->gps_eph);

			transmit_time = transmit_time - (clock_bias + relative_bias - group_delay);

			get_gps_satposvel(transmit_time, channel_i->gps_eph, base_pos, base_vel, channel_i->base_sat_pos, channel_i->base_sat_vel);

			channel_i->base_psr = channel_i->base_obs->psr + (clock_bias + relative_bias - group_delay) * LIGHTSPEED;
			//channel_i->base_prr = -1.0 * channel_i->base_obs->doppler * L1_WAVELENGTH + clock_drift * LIGHTSPEED;
			channel_i->base_phase = 1.0 * channel_i->base_obs->phase + (clock_bias + relative_bias - group_delay) * LIGHTSPEED;

			get_sataziele(base_pos, channel_i->base_sat_pos, &sat_azimuth, &sat_elevation);
			tropo_corr = get_tropocorr(sat_elevation, 1010.0, 15.0, 3.3645810873918274);
			channel_i->base_psr -= tropo_corr;
			channel_i->base_phase -= tropo_corr;
		}
	}
}

static void
build_rtk_kalman_update_matrix(single_solut_channel_t *channel_i, rtk_solut_t *sol,
	double dt,
	double *H, double *R, double *Z, uint8_t type)
{
	double user_pos[3], base_pos[3];
	double user_vel[3], base_vel[3] = { 0 };
	double range_rover, range_rate_rover, range_base, range_rate_base;
	double *sat_pos = channel_i->rover_sat_pos;
	double *sat_vel = channel_i->rover_sat_vel;
	double sat_azimuth, sat_elevation;

	memset(H, 0, sizeof(double) * MAX_KF_STATE);

	plh2ecef(sol->pos, user_pos);
	enu2xyz(sol->pos[0], sol->pos[1], sol->vel, user_vel);

	plh2ecef(sol->base_pos, base_pos);


	range_rover = compute_range(user_pos, sat_pos);
	range_rate_rover = ((sat_vel[0] - user_vel[0]) * (sat_pos[0] - user_pos[0])
		+ (sat_vel[1] - user_vel[1]) * (sat_pos[1] - user_pos[1])
		+ (sat_vel[2] - user_vel[2]) * (sat_pos[2] - user_pos[2])) / range_rover;

	get_sataziele(user_pos, channel_i->rover_sat_pos, &sat_azimuth, &sat_elevation);
	channel_i->rover_sat_azimuth = sat_azimuth;
	channel_i->rover_sat_elevation = sat_elevation;

	sat_pos = channel_i->base_sat_pos;
	sat_vel = channel_i->base_sat_vel;
	range_base = compute_range(base_pos, sat_pos);
	range_rate_base = ((sat_vel[0] - base_vel[0]) * (sat_pos[0] - base_pos[0])
		+ (sat_vel[1] - base_vel[1]) * (sat_pos[1] - base_pos[1])
		+ (sat_vel[2] - base_vel[2]) * (sat_pos[2] - base_pos[2])) / range_base;


	range_rover += sol->clock_bias;
	range_rate_rover += sol->clock_drift;

	// pseudorange
	if (type == 0) {
		H[0] = -cos(sat_elevation) * sin(sat_azimuth);
		H[1] = -cos(sat_elevation) * cos(sat_azimuth);
		H[2] = -sin(sat_elevation);
		H[3] = 1.0;

		*R = 8.0 / sin(sat_elevation) / sin(sat_elevation);
		*Z = (range_rover - range_base) - (channel_i->rover_psr - channel_i->base_psr);
	}
	// Doppler
	else if (type == 1) {
		H[3] = -cos(sat_elevation) * sin(sat_azimuth);
		H[4] = -cos(sat_elevation) * cos(sat_azimuth);
		H[5] = -sin(sat_elevation);
		H[7] = 1.0;

		*R = 0.05 / sin(sat_elevation) / sin(sat_elevation);
		*Z = (range_rate_rover - range_rate_base) - (channel_i->rover_prr - channel_i->base_prr);
	}
	// phase
	else {
		double correction;

		H[0] = -cos(sat_elevation) * sin(sat_azimuth);
		H[1] = -cos(sat_elevation) * cos(sat_azimuth);
		H[2] = -sin(sat_elevation);
		H[3] = 1.0;
		H[MAX_GNSS_STATE + sol->solut_channel.prn_channel_idx[channel_i->prn - 1]] = 1.0;

		*R = 0.04 * L1_WAVELENGTH *L1_WAVELENGTH;

		if (sol->filter.dN[channel_i->prn - 1] == 0.0) {
			sol->filter.dN[channel_i->prn - 1] = (channel_i->rover_phase - channel_i->base_phase) -
				(channel_i->rover_psr - channel_i->base_psr);
		}

		correction = dt * channel_i->rover_prr;
		*Z = (range_rover - range_base) + sol->filter.dN[channel_i->prn - 1] -
			(channel_i->rover_phase - channel_i->base_phase) + correction;
	}
}

bool_t rtk_kalman_update(gnss_nav_context_t *context)
{
	uint8_t i, rover_obs_num, base_obs_num, valid_obs_num;
	uint8_t valid_rover_obs_idx[MAX_OBS_CHANNELS] = { 0 };
	uint8_t valid_base_obs_idx[MAX_OBS_CHANNELS] = { 0 };
	uint8_t channel_update_idx[MAX_OBS_CHANNELS];

	double dX[MAX_GNSS_STATE + MAX_OBS_CHANNELS] = { 0 };
	double T[MAX_KF_STATE * MAX_KF_STATE] = { 0 };
	double Q[MAX_KF_STATE * MAX_KF_STATE] = { 0 };
	double *P = context->rtk_sol.filter.P;
	double time_interval;

	if (!context->is_rover_updated) {
		context->ptr_rover_obs->num_obs_channel = gnss_data_prefilter(context->ptr_rover_obs, context->ptr_rover_obs->valid_obs_idx,
			&context->gnss_eph);
	}

	context->base_obs.num_obs_channel = gnss_data_prefilter(&context->base_obs, context->base_obs.valid_obs_idx,
		&context->gnss_eph);

	valid_obs_num = dgnss_channel_add_gnss_obs(context);


	time_interval = context->ptr_rover_obs->tow - context->rtk_sol.time;

	gps_time_rollover_correct(&time_interval);

	if (!NEAR_ZERO(time_interval)) {
		gnss_kalman_propagagtion(time_interval, T, Q, P, MAX_GNSS_STATE + valid_obs_num);

		context->rtk_sol.pos[0] += context->rtk_sol.vel[1] / (context->rtk_sol.filter.M + context->rtk_sol.pos[2]) * time_interval;
		context->rtk_sol.pos[1] += context->rtk_sol.vel[0] / (context->rtk_sol.filter.N + context->rtk_sol.pos[2])
			/ cos(context->rtk_sol.pos[0]) * time_interval;
		context->rtk_sol.pos[2] += context->rtk_sol.vel[2] * time_interval;
		context->rtk_sol.clock_bias += context->rtk_sol.clock_drift * time_interval;

		compute_MN_radius(context->rtk_sol.pos[0], &context->rtk_sol.filter.M, &context->rtk_sol.filter.N);
	}

	//matprint(context.nav_sol.filter.P, MAX_GNSS_STATE + valid_obs_num, valid_obs_num);

	context->rtk_sol.time = context->ptr_rover_obs->tow;

	build_satellite_pos_vel(&context->rtk_sol);

	//receiver_ms_jump_correction(&context->rtk_sol.solut_channel);

	for (i = 0; i < valid_obs_num; i++) {
		channel_update_idx[i] = i;
	}

	//qsort(channel_update_idx, valid_obs_num, sizeof(uint8_t), obs_compare);

	//valid_obs_number = kalman_blunder_dection(&Z, &H, &R, P, num_obs_type*sat_num, num_states);
	//valid_obs_number = sat_num * num_obs_type;
	double dt = (context->ptr_rover_obs->tow - context->base_obs.tow);

	for (i = 0; i < valid_obs_num; i++) {
		double R;
		double H[MAX_KF_STATE] = { 0 };
		double Z;

		if (context->rtk_sol.solut_channel.channel[i].rover_sat_elevation > 5.0 * DEG2RAD) {
			// Pseudorange update
			
			build_rtk_kalman_update_matrix(&context->rtk_sol.solut_channel.channel[i], &context->rtk_sol, dt, H, &R, &Z, 0);
			if (kalman_sequential_update(dX, P, H, &R, &Z, MAX_GNSS_STATE + valid_obs_num))
				rtk_state_correction(&context->rtk_sol, dX, valid_obs_num);
		}
	}

	//for (i = 0; i < valid_obs_num; i++) {
	//	double R;
	//	double H[MAX_KF_STATE] = { 0 };
	//	double Z;

	//	if (context.nav_sol.solut_channel.channel[i].rover_sat_elevation > 5.0 * DEG2RAD) {
	//		// Doppler update
	//		build_rtk_kalman_update_matrix(&context.nav_sol.solut_channel.channel[i], H, &R, &Z, 1);
	//		if (kalman_sequential_update(dX, P, H, &R, &Z, MAX_GNSS_STATE + valid_obs_num))
	//			error_state_correction(dX, valid_obs_num);
	//	}
	//}

	//if (ABS(context.rover->gnss_obs.tow - context.base->gnss_obs.tow) < 0.0005) {
		for (i = 0; i < valid_obs_num; i++) {
			double R;
			double H[MAX_KF_STATE] = { 0 };
			double Z;

			if (context->rtk_sol.solut_channel.channel[i].rover_sat_elevation > 5.0 * DEG2RAD) {
				// carrier phase update
				build_rtk_kalman_update_matrix(&context->rtk_sol.solut_channel.channel[i], &context->rtk_sol, dt, H, &R, &Z, 2);
				if (kalman_sequential_update(dX, P, H, &R, &Z, MAX_GNSS_STATE + valid_obs_num))
					rtk_state_correction(&context->rtk_sol, dX, valid_obs_num);
			}
		}
	//}


	//if (context.nav_sol.skip_cnt > 0) {
	//	double R;
	//	double H[MAX_KF_STATE] = { 0 };
	//	double Z;

	//	H[2] = 1.0;
	//    R = 0.0001;
	//    Z = context.nav_sol.pos[2] - context.nav_sol.prev_pos[2];

	//    if (kalman_sequential_update(dX, P, H, &R, &Z, MAX_GNSS_STATE + valid_obs_num))
	//	   error_state_correction(dX, valid_obs_num);
	//}


	//kalman_filter_update(dX, P, H, R, Z, valid_obs_num * 3, MAX_GNSS_STATE + valid_obs_num);
	//error_state_correction(dX, valid_obs_num);


	//matprint(P, MAX_GNSS_STATE + valid_obs_num, MAX_GNSS_STATE + valid_obs_num);


	compute_MN_radius(context->rtk_sol.pos[0], &context->rtk_sol.filter.M, &context->rtk_sol.filter.N);

	return TRUE;
}