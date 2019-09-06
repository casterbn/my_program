#include <stdlib.h>
#include <string.h>

#include "gnss_nav_utils.h"
#include "gnss_nav_proc.h"
#include "gnss_nav_estimation.h"


void gnss_obs_array_push(gnss_obs_array_t *obs_array, gnss_obs_t *next)
{
	memcpy(&obs_array->gnss_obs[obs_array->tail], next, sizeof(gnss_obs_t));

	if (obs_array->size <= MAX_OBS_ARRAY_LEN - 1)
		obs_array->size += 1;

	obs_array->tail = (obs_array->tail + 1) % MAX_OBS_ARRAY_LEN;
}

gnss_obs_t* gnss_obs_array_pop(gnss_obs_array_t *obs_array)
{
	gnss_obs_t *get = NULL;

	if (obs_array->size > 0) {
		get = &obs_array->gnss_obs[obs_array->head];
		obs_array->head = (obs_array->head + 1) % MAX_OBS_ARRAY_LEN;
		obs_array->size--;
	}

	return get;
}


bool update_gnss_obs(gnss_obs_t *gnss_obs, const gnss_obs_msg_t *gnss_obs_msg)
{
	if (!gnss_obs || !gnss_obs_msg) {
		return false;
	}

	gnss_obs->num_obs_channel = 0;
	uint8_t cnt = 0;
	for (uint8_t i = 0; i < gnss_obs_msg->num_obs_channel_msg && i < MAX_OBS_CHANNELS; i++) {
	
		if (gnss_obs_msg->channel[i].system == SYS_GPS) {
			// DW DEBUG todo
			if (1 == NUM_GNSS_FREQ && gnss_obs_msg->channel[i].signal_type != 2) {
				continue; // SINGLE FREQ processing
			}

			gnss_obs->channel[cnt].system = SYS_GPS;
			gnss_obs->sat_num[SYS_GPS]++;
		}
		else if (gnss_obs_msg->channel[i].system == SYS_BDS) {
			if (ENA_BDS) {
				gnss_obs->channel[cnt].system = SYS_BDS;
				gnss_obs->sat_num[SYS_BDS]++;
			}
			else {
				continue;
			}
		}
		else if (gnss_obs_msg->channel[i].system == SYS_GLO) {
			if (ENA_GLO) {
				gnss_obs->channel[cnt].system = SYS_GLO;
				gnss_obs->sat_num[SYS_GLO]++;
			}
			else {
				continue;
			}
		}
		else if (gnss_obs_msg->channel[i].system == SYS_GAL) {
			if (ENA_GAL) {
				gnss_obs->channel[cnt].system = SYS_GAL;
				gnss_obs->sat_num[SYS_GAL]++;
			}
			else {
				continue;
			}
		}

		gnss_obs->channel[cnt].prn = gnss_obs_msg->channel[i].prn;
		gnss_obs->channel[cnt].signal_type = gnss_obs_msg->channel[i].signal_type;
		gnss_obs->channel[cnt].phase_lock = gnss_obs_msg->channel[i].phase_lock;
		gnss_obs->channel[cnt].code_lock = gnss_obs_msg->channel[i].code_lock;
		gnss_obs->channel[cnt].lli = gnss_obs_msg->channel[i].lose_lock_indicator;
		gnss_obs->channel[cnt].cn0 = (float)gnss_obs_msg->channel[i].cn0;
		gnss_obs->channel[cnt].lock_time = (float)gnss_obs_msg->channel[i].lock_time;
		gnss_obs->channel[cnt].psr = gnss_obs_msg->channel[i].pseudorange;
		//context->rover_obs.channel[i].psr_std = gnss_obs_msg->channel[i].pseudorange_std;
		gnss_obs->channel[cnt].doppler = gnss_obs_msg->channel[i].doppler;
		//context->rover_obs.channel[i].doppler_std = gnss_obs_msg->channel[i].doppler_std;
		gnss_obs->channel[cnt].phase = gnss_obs_msg->channel[i].phase;
		//context->rover_obs.channel[i].phase_std = gnss_obs_msg->channel[i].phase_std;

		cnt++;
	}

	gnss_obs->num_obs_channel = cnt;
	gnss_obs->tow = gnss_obs_msg->tow;
	gnss_obs->week = gnss_obs_msg->week;
	gnss_obs->is_updated = true;


	return true;
}


bool update_gps_eph(gnss_nav_context_t *context, const gps_eph_msg_t *gps_eph_msg)
{
	int prn = gps_eph_msg->prn;
	int idx = prn - 1;

	if (context && idx >= 0 && idx < MAX_GPS_NUMBER_SAT) {
		context->gnss_eph.gps_eph.svid[idx].prn = gps_eph_msg->prn;
		context->gnss_eph.gps_eph.svid[idx].week = gps_eph_msg->week;
		context->gnss_eph.gps_eph.svid[idx].tow = gps_eph_msg->tow;
		context->gnss_eph.gps_eph.svid[idx].code = gps_eph_msg->code;
		context->gnss_eph.gps_eph.svid[idx].ura = gps_eph_msg->ura;
		context->gnss_eph.gps_eph.svid[idx].health = gps_eph_msg->health;
		context->gnss_eph.gps_eph.svid[idx].iodc = gps_eph_msg->iodc;
		context->gnss_eph.gps_eph.svid[idx].flag_l2p = gps_eph_msg->flag_l2p;
		context->gnss_eph.gps_eph.svid[idx].tgd = gps_eph_msg->tgd;
		context->gnss_eph.gps_eph.svid[idx].toc = gps_eph_msg->toc;
		context->gnss_eph.gps_eph.svid[idx].af2 = gps_eph_msg->af2;
		context->gnss_eph.gps_eph.svid[idx].af1 = gps_eph_msg->af1;
		context->gnss_eph.gps_eph.svid[idx].af0 = gps_eph_msg->af0;
		context->gnss_eph.gps_eph.svid[idx].iode = gps_eph_msg->iode;
		context->gnss_eph.gps_eph.svid[idx].crs = gps_eph_msg->crs;
		context->gnss_eph.gps_eph.svid[idx].deltan = gps_eph_msg->deltan;
		context->gnss_eph.gps_eph.svid[idx].m0 = gps_eph_msg->m0;
		context->gnss_eph.gps_eph.svid[idx].cuc = gps_eph_msg->cuc;
		context->gnss_eph.gps_eph.svid[idx].ecc = gps_eph_msg->ecc;
		context->gnss_eph.gps_eph.svid[idx].cus = gps_eph_msg->cus;
		context->gnss_eph.gps_eph.svid[idx].sqrta = gps_eph_msg->sqrta;
		context->gnss_eph.gps_eph.svid[idx].toe = gps_eph_msg->toe;
		context->gnss_eph.gps_eph.svid[idx].cic = gps_eph_msg->cic;
		context->gnss_eph.gps_eph.svid[idx].omega0 = gps_eph_msg->omega0;
		context->gnss_eph.gps_eph.svid[idx].cis = gps_eph_msg->cis;
		context->gnss_eph.gps_eph.svid[idx].i0 = gps_eph_msg->i0;
		context->gnss_eph.gps_eph.svid[idx].crc = gps_eph_msg->crc;
		context->gnss_eph.gps_eph.svid[idx].omega = gps_eph_msg->omega;
		context->gnss_eph.gps_eph.svid[idx].omegadot = gps_eph_msg->omegadot;
		context->gnss_eph.gps_eph.svid[idx].idot = gps_eph_msg->idot;

		context->gnss_eph.gps_eph.svid[idx].is_updated = true;
		return true;
	}
	else {
		return false;
	}
}

uint8_t gnss_data_prefilter(gnss_obs_t *input_data, uint8_t *valid_obs_idx, gnss_eph_t *eph)
{
	uint8_t i;
	uint8_t prn;
	uint8_t num_valid_obs = 0;

	for (i = 0; i < input_data->num_obs_channel; i++) {
		double tk;

		prn = input_data->channel[i].prn;

		if (prn == 0 || input_data->channel[i].lli != 0 ||
			input_data->channel[i].system == SYS_NONE || input_data->channel[i].phase_lock == 0 || input_data->channel[i].cn0 < 30.0f)
			continue;

		if (eph) {
			switch (input_data->channel[i].system) 
			{
			case SYS_GPS:
				//gps_eph_t *gpseph = &eph->gps_eph;
				if (eph->gps_eph.svid[prn - 1].prn == prn && eph->gps_eph.svid[prn - 1].health == 0) {
					tk = input_data->tow - eph->gps_eph.svid[prn - 1].toe;
					gps_time_rollover_correct(&tk);

					if (ABS(tk) <= 7200.0) {
						valid_obs_idx[num_valid_obs++] = i;
					}
				}
				break;
			case SYS_BDS:
				break;
			case SYS_GLO:
				break;
			default:
				break;
			}
		}
	}

	return num_valid_obs;
}


bool_t gnss_nav_proc_rover(gnss_nav_context_t *context)
{
	bool_t ret = FALSE;

	// context->nav_sol.prev_pos[0] = context->nav_sol.pos[0];
	// context->nav_sol.prev_pos[1] = context->nav_sol.pos[1];
	// context->nav_sol.prev_pos[2] = context->nav_sol.pos[2];
	context->rover_obs.num_obs_channel = gnss_data_prefilter(&context->rover_obs, context->rover_obs.valid_obs_idx,
		&context->gnss_eph);
	
	if (context->rover_obs.num_obs_channel < 4) {
		//context->nav_sol.nav_status = 0;
		return ret;
	}

	/*context->cnt_gnss_proc += 1;
	if (context->cnt_gnss_proc < 10) {
		return ret;
	}
	else {
		context->cnt_gnss_proc = 0;
	}*/

	switch (context->nav_status)
	{
		case 0:
			if (context->rover_obs.num_obs_channel < 4) {
				break;
			}

			if (!closedform_solution(context)) {
				break;
			}
			 
			//ret = TRUE;
			//break;

		case 1:
			if (ls_gnss_update(context)) {
				context->nav_status = 3;
				memset(&context->rtk_sol.solut_channel, 0, sizeof(solut_channel_t));
				memset(&context->rtk_sol.solut_channel.prn_channel_idx, 0xFF, sizeof(uint8_t) * MAX_NUMBER_SAT);

				
				context->is_rover_updated = TRUE;
            	ret = TRUE;
			}
			memset(&context->nav_sol.solut_channel, 0, sizeof(solut_channel_t));
			break;
		case 2:
		case 3: 
		case 4:// DW DEBUG, use LSQ as placeholder for now
			if (ls_gnss_update(context)) {
				
				/*memset(&context->nav_sol.solut_channel, 0, sizeof(solut_channel_t));*/
				memset(&context->nav_sol.solut_channel.prn_channel_idx, 0xFF, sizeof(uint8_t) * MAX_NUMBER_SAT);

				gnss_obs_array_push(&context->rover_obs_array, &context->rover_obs);
				context->is_rover_updated = TRUE;

				if (context->rover_obs_array.size == MAX_OBS_ARRAY_LEN) {
					context->nav_status = 1;
					context->is_base_init = FALSE;
					memset(&context->rover_obs_array, 0, sizeof(gnss_obs_array_t));
					ret = FALSE;

				}
				else {
					ret = TRUE;
				}
			}
			memset(&context->nav_sol.solut_channel, 0, sizeof(solut_channel_t));
			break;
		default:
			break;
	}

	return ret;
}


bool_t gnss_nav_proc_with_base(gnss_nav_context_t *context)
{
	bool_t ret = FALSE;
	gnss_obs_t *ptr_base_obs = &context->base_obs;
	context->ptr_rover_obs = &context->rover_obs;

	switch (context->nav_status) {
	case 0:
	case 1:
	case 2:
		while (context->rover_obs_array.size > 0) {
			context->ptr_rover_obs = gnss_obs_array_pop(&context->rover_obs_array);
		}
		break;
	case 3:
		// while (context->rover_obs_array.size > 0) {
		// 	context->ptr_rover_obs = gnss_obs_array_pop(&context->rover_obs_array);
		
			if (ABS(context->ptr_rover_obs->tow - ptr_base_obs->tow) <= 30.0) {
				memcpy(context->rtk_sol.pos, context->nav_sol.pos, 3 * sizeof(double));
				memcpy(context->rtk_sol.vel, context->nav_sol.vel, 3 * sizeof(float));
				context->rtk_sol.filter.M = context->nav_sol.filter.M;
				context->rtk_sol.filter.N = context->nav_sol.filter.N;

				if (lsq_dgnss_update(context)) {
					context->nav_status = 4;
					rtk_kalman_init(context);

					ret = TRUE;
					//break;
				}
			
			}
		//}

#ifdef POST_PROCESSING
		if (3 == context->nav_status) {
			context->ptr_rover_obs = &context->rover_obs;
			if (ABS(context->ptr_rover_obs->tow - ptr_base_obs->tow) < 0.1) {
				memcpy(context->rtk_sol.pos, context->nav_sol.pos, 3 * sizeof(double));
				memcpy(context->rtk_sol.vel, context->nav_sol.vel, 3 * sizeof(float));
				context->rtk_sol.filter.M = context->nav_sol.filter.M;
				context->rtk_sol.filter.N = context->nav_sol.filter.N;

				if (lsq_dgnss_update(context)) {
					context->nav_status = 4;
					rtk_kalman_init(context);

					ret = TRUE;
				}
			}
		}
#endif

		break;
	case 4:
		//while (context->rover_obs_array.size > 0) {
			//context->ptr_rover_obs = gnss_obs_array_pop(&context->rover_obs_array);
			if (ABS(context->ptr_rover_obs->tow - ptr_base_obs->tow) <= 30.0) {
				rtk_kalman_update(context);

				//break;
			}
		//}

#ifdef POST_PROCESSING
		if (4 == context->nav_status) {
		
			context->ptr_rover_obs = &context->rover_obs;
			if (ABS(context->ptr_rover_obs->tow - ptr_base_obs->tow) < 0.1) {
				rtk_kalman_update(context);
				ret = TRUE;
			
			}
		}
#endif

		break;

	default:
		break;
	}

	return ret;

}