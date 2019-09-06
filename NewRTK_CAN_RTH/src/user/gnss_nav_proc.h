
#ifndef _GNSS_NAV_PROC_H_
#define _GNSS_NAV_PROC_H_

#include <stdbool.h>
#include <stdint.h>


#include "gps.h"
#include "beidou.h"
#include "glonass.h"
#include "galileo.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ENA_BDS                     (0)
#define ENA_GLO                     (0)
#define ENA_GAL                     (0)

#define EST_IONO                    (0)
#define EST_DYN                     (0)

#define MAX_GNSS_OBS_MSG_CHANNELS           (64)
#define NUM_GNSS_FREQ                       (1)
//#define MAX_OBS_ARRAY_LEN                   (10)
#define NUM_GNSS_SYSTEM                     (1+ENA_GLO+ENA_BDS+ENA_GAL)
#define MAX_GPS_OBS_CHANNELS                (12)
#define MAX_GLO_OBS_CHANNELS                (10)
#define MAX_BDS_OBS_CHANNELS                (12)
#define MAX_GAL_OBS_CHANNELS                (10)
// #define MAX_OBS_CHANNELS                    (MAX_GPS_OBS_CHANNELS + (ENA_BDS==1?MAX_BDS_OBS_CHANNELS:0) \
//     + (ENA_GLO==1?MAX_GLO_OBS_CHANNELS:0) + (ENA_GAL==1?MAX_GAL_OBS_CHANNELS:0))
#define MAX_OBS_CHANNELS 					(12)
#define MAX_NUMBER_SAT 						(32)
#define MAX_GNSS_STATE 						((EST_DYN==0?4:8) + (NUM_GNSS_SYSTEM - 1))
#define NUM_OBSERVABLE						((EST_DYN==0?1:2))
#define MAX_OBS_NUM							(40)//MAX_OBS_CHANNELS*NUM_GNSS_FREQ*NUM_OBSERVABLE)
// #define MAX_KF_STATE                        ( + MAX_OBS_CHANNELS*NUM_GNSS_FREQ \
//     + (EST_IONO==0?0:MAX_OBS_CHANNELS*NUM_GNSS_FREQ))
                                            // PV + CLK + AMB + IONO + TROPO

#define MAX_KF_STATE 						(MAX_GNSS_STATE + MAX_OBS_CHANNELS*NUM_GNSS_FREQ)
// #define CN0_BUFFER_SIZE (5)

#undef MAX_OBS_ARRAY_LEN
#define MAX_OBS_ARRAY_LEN (10)


typedef struct gnss_obs_channel_msg_tag {
	uint16_t		     prn;
	uint8_t		         system;
	uint8_t       	     signal_type;
	bool		         phase_lock;
	bool		         code_lock;
	uint8_t		         lose_lock_indicator;
	float        	     cn0;
	float		         lock_time;
	double		         pseudorange;
	//float		         pseudorange_std;
	double		         doppler;
	//float 		         doppler_std;
	double        	     phase;
	//float                phase_std;
} gnss_obs_channel_msg_t;

typedef struct gnss_obs_msg_tag {
	int		             week;
	double		         tow;
	int		             num_obs_channel_msg;
	gnss_obs_channel_msg_t channel[MAX_GNSS_OBS_MSG_CHANNELS];
} gnss_obs_msg_t;

typedef struct gps_eph_msg_t {
	int		             prn;		    
	int		             week;		    
	double		         tow;
	int      		     code;		   
	float      		     ura;			
	uint32_t 	         health;
	uint32_t	         iodc;
	uint32_t	         flag_l2p;
	double               fit_interval;
	double		         tgd;
	double		         toc;
	double		         af2;
	double		         af1;
	double		         af0;
	uint32_t	         iode;
	double		         crs;
	double		         deltan;
	double		         m0;
	double		         cuc;
	double		         ecc;
	double		         cus;
	double		         sqrta;
	double		         toe;
	double		         cic;
	double               omega0;
	double               cis;
	double		         i0;
	double		         crc;
	double		         omega;
	double		         omegadot;
	double		         idot;
} gps_eph_msg_t;


typedef struct glonass_msg_tag {
	int		             prn;
	unsigned int	     health;
	int		             frequency;
	double		         toe;
	double		         toff;
	unsigned int	     iode;	
	double		         pos_x;
	double		         pos_y;
	double		         pos_z;
	double		         vel_x;
	double		         vel_y;
	double		         vel_z;
	double		         acc_x;
	double		         acc_y;
	double		         acc_z;
	double		         taun;
	double		         gamman;
	double		         tof;
	int 		         age;
} glo_eph_msg_t;


typedef struct bds_eph_msg_tag {
	int                  prn;
	uint32_t             health;
	int                  week;
	double               tow;
	float                ura;
	double               tgd[4];
	uint32_t             aodc;
	double               toc;
	double               ttr;
	double               af2;
	double               af1;
	double               af0;
	double               toes;
	uint32_t             aode;
	int                  sva;
	double               crs;
	double               deltan;
	double               m0;
	double               cuc;
	double               e;
	double               cus;
	double               sqrta;
	double               cic;
	double               omega0;
	double               cis;
	double               i0;
	double               crc;
	double               omega;
	double               omegadot;
	double               idot;
} bds_eph_msg_t;

union gnss_raw_msg_union {
	gnss_obs_msg_t  gnss_obs;
	gps_eph_msg_t   gps_eph;
	bds_eph_msg_t   bds_eph;
	glo_eph_msg_t   glo_eph;
};

typedef struct gnss_raw_msg_tag {
	int                  type;
	union gnss_raw_msg_union message;
} gnss_raw_msg_t;


typedef enum gnss_sys_type_tag {
	SYS_NONE = 0,
	SYS_GPS,
	SYS_BDS,
	SYS_GLO,
	SYS_GAL
} gnss_sys_type_t;

typedef struct gnss_obs_channel_tag {
    uint8_t                   prn;
    gnss_sys_type_t           system; 
    uint8_t                   signal_type;//signal_type[NUM_GNSS_FREQ];
    bool                      phase_lock;
    bool                      code_lock;
    uint8_t                   lli; //lli[NUM_GNSS_FREQ];
    float                     cn0;//cn0[NUM_GNSS_FREQ];
    float                     lock_time;
    double                    psr;//psr[NUM_GNSS_FREQ];
    //float                     psr_std;
    double                    doppler;//doppler[NUM_GNSS_FREQ];
    //float                     doppler_std;
    double                    phase; //phase[NUM_GNSS_FREQ];
	//float                     phase_std;
} gnss_obs_channel_t;

typedef struct gnss_obs_tag {
    int                       week;
    double                    tow;
    uint8_t                   sat_num[NUM_GNSS_SYSTEM+1];
    uint8_t                   num_obs_channel;
	//uint8_t					  valid_rover_obs_idx[MAX_OBS_CHANNELS];
	uint8_t					  valid_obs_idx[MAX_OBS_CHANNELS];
    gnss_obs_channel_t        channel[MAX_OBS_CHANNELS];
	bool                      is_updated;                   // Only for logging purpose.
} gnss_obs_t;


typedef struct gnss_eph_tag {
	gps_eph_t gps_eph;
	bds_eph_t bds_eph;
	//glo_eph_t glo_eph;
    //gal_eph_t gal_eph;
} gnss_eph_t;


typedef struct nav_filter_tag {
    double                 P[MAX_KF_STATE * MAX_KF_STATE];
    //bool                   is_initialized;
    //nav_filter_mode_t      mode;
	double M;
	double N;
} nav_filter_t;

typedef struct rtk_filter_tag {
	double                 P[MAX_KF_STATE * MAX_KF_STATE];
	//bool                   is_initialized;
	//nav_filter_mode_t      mode;
	//double                 x[MAX_KF_STATE];
	//double                 xa[MAX_OBS_CHANNELS*NUM_GNSS_FREQ];
	//int32_t                an[MAX_OBS_CHANNELS*NUM_GNSS_FREQ];
	double M;
	double N;
	//double dN[MAX_OBS_CHANNELS*NUM_GNSS_FREQ];
	double dN[MAX_NUMBER_SAT];
} rtk_filter_t;


typedef union gnss_eph_channel_tag {
    const gps_eph_channel_t   *gps_eph;
	const bds_eph_channel_t   *bds_eph;
    //const glo_eph_channel_t   *glo_eph;
    //const gal_eph_channel_t   *gal_eph;
} gnss_eph_channel_t;

// typedef struct nav_obs_channel_tag {
//     gnss_obs_channel_t        obs;
//     gnss_eph_channel_t        gnss_eph;
//     int                       low_power_cnt;
//     int                       phase_err_cnt;
// 	float                     cn0[CN0_BUFFER_SIZE];
//     float                     average_power;
// 	float                     lock_time;
// 	bool                      is_sat_pos_vel_valid;
//     double                    sat_pos[3];
//     double                    sat_vel[3];
//     double                    psr_corr;
//     double                    prr_corr;
//     double                    phase_corr;
//     double                    sat_azimuth;
//     double                    sat_elevation;
// } nav_obs_channel_t;

// typedef struct nav_obs_t {
//     int                       week;
//     double                    tow;
//     int                       obs_num_total;
//     int                       obs_num[NUM_GNSS_SYSTEM];
//     nav_obs_channel_t         channel[MAX_OBS_CHANNELS];
//     int8_t                    gps_channel_idx[MAX_GPS_NUMBER_SAT];
//     //int8_t                    glo_channel_idx[MAX_GLO_NUMBER_SAT];
// 	int8_t                    bds_channel_idx[MAX_GLO_NUMBER_SAT];
// } nav_obs_t;

typedef struct common_para_tag {
	double M;
	double N;
} common_para_t;

// typedef struct nav_channel_tag {
// 	int                    prn;
// 	gnss_sys_type_t        system;
// 	nav_obs_channel_t      *base_obs;
// 	nav_obs_channel_t      *rover_obs;
// 	bool                   is_active;
// 	bool                   is_psr_valid;
// 	bool                   is_prr_valid;
// 	bool                   is_phase_valid;
// 	double                 residual_psr;
// 	double                 residual_prr;
// 	double                 residual_phase;
// 	double                 sat_azimuth;
// 	double                 sat_elevation;
// 	double                 var_psr;
// 	double                 var_prr;
// } nav_channel_t;

// typedef struct nav_channel_info_tag {
// 	double                 tow;
// 	int                    channel_num_total;
// 	nav_channel_t          channel[MAX_OBS_CHANNELS];
// 	int                    used_channels[NUM_GNSS_SYSTEM];
// 	int8_t                 gps_channel_idx[MAX_GPS_NUMBER_SAT];
// 	int8_t                 bds_channel_idx[MAX_BDS_NUMBER_SAT];
// 	//int8_t                 glo_channel_idx[MAX_GLO_NUMBER_SAT];
// } nav_channel_info_t;


typedef struct gnss_obs_array_tag {
	gnss_obs_t gnss_obs[MAX_OBS_ARRAY_LEN];
	uint32_t   head;
	uint32_t   tail;
	uint32_t   size;
} gnss_obs_array_t;


typedef struct base_info_tag {
	uint32_t      station_id;
	bool_t        is_gps;
	bool_t        is_glonass;
	double        pos_ecef[3];
	double        pos_plh[3];
	float         height;
	char          antenna_descriptor[32];
} base_info_t;

typedef struct single_solut_channel_tag {
	uint8_t prn;
	gnss_obs_channel_t *base_obs;
	gnss_obs_channel_t *rover_obs;
	gps_eph_channel_t  *gps_eph;
	bds_eph_channel_t *bds_eph;
	bool_t is_blunder;
	bool_t is_updated;
	double rover_sat_elevation;
	double rover_sat_azimuth;
	double base_sat_pos[3];
	double base_sat_vel[3];
	double rover_sat_pos[3];
	double rover_sat_vel[3];
	double rover_psr;
	double rover_prr;
	double rover_phase;
	double base_psr;
	double base_prr;
	double base_phase;
	double tow_prev;
	double rover_prr_prev;
	double rover_phase_prev;
} single_solut_channel_t;


typedef struct solut_channel_tag {
	double tow;
	uint8_t used_channels;
	single_solut_channel_t channel[MAX_OBS_CHANNELS];
	uint8_t prn_channel_idx[MAX_NUMBER_SAT];
} solut_channel_t;


typedef struct {
	double time;
	solut_channel_t solut_channel;
	nav_filter_t filter;
	double pos[3];
	//double prev_pos[3];
	float vel[3];
	//double base_pos[3];
	//double base_vel[3];
	double clock_bias;
	double clock_drift;
	float std_pos[3];
	float hdop;
	float vdop;

	uint8_t mode;
	bool_t single;
} solut_t;

typedef struct {
	
	solut_channel_t        solut_channel;
	rtk_filter_t          filter;
	double time;
	double                 pos[3];
	float                 vel[3];
	double                 base_pos[3];
	float                 base_vel[3];
	double                 clock_bias;
	double                 clock_drift;
	double				   prev_pos[3];
	float				   prev_vel[3];
	double				   prev_clock_bias;
	double                 pos_amb_fixed[3];
	//double				   comple_gnss_clkbiases[4]; // 0: Beidou
	//double				   comple_gnss_clkdrifts[4]; // 0: Beidou
} rtk_solut_t;

typedef struct gnss_nav_context_tag {
	gnss_eph_t gnss_eph;
    gnss_obs_t rover_obs;
	gnss_obs_array_t rover_obs_array;
    gnss_obs_t base_obs;
	base_info_t base_info; 
	solut_t nav_sol;
	rtk_solut_t rtk_sol;
	gnss_obs_t *ptr_rover_obs;
	double tow_prev;
	//double start_bias;
	//uint8_t cnt_gnss_proc; // DW DEBUG, reduce GNSS rate
	uint8_t nav_status;
	bool_t is_base_init;
	bool_t is_rover_updated;
} gnss_nav_context_t;



typedef struct {
	bool is_obs_complete;
	uint8_t buff[1024];
	int station_id;
	int msg_len;
	int nbyte;
	base_info_t base_info;
	gnss_raw_msg_t gnss_raw_msg;
	// rtcm_msm_message msm;
	// rtcm_msg_eph eph;
} rtcm_data_t;


void gnss_obs_array_push(gnss_obs_array_t *obs_array, gnss_obs_t *next);

gnss_obs_t* gnss_obs_array_pop(gnss_obs_array_t *obs_array);

bool update_gnss_obs(gnss_obs_t *gnss_obs, const gnss_obs_msg_t *gnss_obs_msg);

bool update_gps_eph(gnss_nav_context_t *context, const gps_eph_msg_t *gps_eph_msg);

uint8_t gnss_data_prefilter(gnss_obs_t *input_data, uint8_t *valid_obs_idx, gnss_eph_t *gpseph);

bool_t gnss_nav_proc_rover(gnss_nav_context_t *context);

bool_t gnss_nav_proc_with_base(gnss_nav_context_t *context);

// Added by DW
uint8_t rtcm3_decode(const uint8_t in_byte, rtcm_data_t *rtcm_data);

//extern 
#ifdef __cplusplus
}
#endif


#endif
