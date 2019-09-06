// #ifndef _INS_ALGORITHM_
// #define _INS_ALGORITHM_

// #include <stdint.h>

// #include "ppprtk.h"

// #define IMU_RAW_DATA_RATE           (50)
// #define INS_NAV_UPDATE_RATE         (50)
// #define INS_UPDATE_INTERVAL         (1000/INS_NAV_UPDATE_RATE)

// #define NUM_GNSSINS_EKF_STATE       (15)

// typedef struct utc_time_tag
// {
//     uint16_t year;
//     int8_t month;
//     int8_t day;
//     int8_t hour;
//     int8_t minute;
//     int8_t millisecond;
// } utc_time_t, *ptr_utc_time_t;

// typedef struct gps_tow_tag
// {
//     uint16_t wn; // GPS week number
//     uint32_t tow_ms; // GPS time of week in milliseconds
// } gps_time_t;

// typedef struct gnss_pos_tag
// {
// 	double lat;
// 	double lon;
// 	double hgt;
// } pos_t, *ptr_pos_t;

// typedef struct gnss_vel_tag
// {
// 	float vn;
// 	float ve;
// 	float vu;
// } vel_t, *ptr_vel_t;

// typedef struct gnss_dop_tag
// {
// 	float hdop;
// 	float vdop;
// 	float pdop;
// } dop_t, *ptr_dop_t;

// typedef struct gnss_data_tag
// {
// 	utc_time_t utc_time;
//     gps_time_t gps_time;
// 	uint8_t nav_status;
// 	uint8_t nav_type;
// 	uint8_t nav_flag;
// 	pos_t pos;
// 	vel_t vel;
//     dop_t dops;			// hdop, vdop, pdop
// 	uint8_t num_sat_use;
//     float age;

// }gnss_data_t, *ptr_gnss_data_t;

// typedef struct imu_err_param_tag
// {
// 	uint8_t install_misalign_flag;
//     float install_mat[9];
//     float gyro_bias[3];
//     float acce_bias[3]; 
// } imu_err_param_t, *ptr_imu_err_param_t;

// typedef struct imu_data_tag
// {
//     utc_time_t utc_time;
//     uint32_t ms_interval;
//     float gyro[3];
//     float acce[3];
// } imu_data_t, *ptr_imu_data_t;

// typedef struct euler_angle_tag
// {
// 	float phi;
// 	float theta;
// 	float gamma;
// } euler_angle_t;

// typedef struct gnssins_nav_data_tag
// {
// 	utc_time_t utc_time;
// 	gps_time_t gps_time;
//     uint8_t nav_status;
// 	uint8_t frequency;
// 	uint32_t ins_status;

// 	uint8_t gst_status;

//     uint32_t imu_time_cnt;
// 	uint32_t cnt_static;
// 	uint32_t cnt_gnss_halt;
// 	uint32_t cnt_ms_ins_alone;
// 	uint32_t cnt_ekf;

//     imu_err_param_t imu_err_cfg;
	
// 	float gyro[3];
//     float prev_gyro[3];
//     float prev_acce[3];

// 	float gyro_drift[3];
// 	float gyro_rate;
// 	float gyro_diff;
// 	uint8_t flag_gyro_rate;
// 	uint8_t flag_line_motion;
	
// 	pos_t pos;
// 	vel_t vel;
// 	float gravity; // normal component
// 	double rm;
// 	double rn;
	
// 	float dcm_ne[9]; // direct cosine matrix, NE
// 	float q_ne[4]; // quaternion [q0 (const),q1,q2,q3]
// 	float wie[3];
// 	float wen[3];

// 	euler_angle_t euler_angles;
// 	float dcm_bn[9];
// 	vel_t dvel;
// 	float sf_n[3];

// 	uint8_t flag_heading;

// } gnssins_nav_data_t;

// typedef struct gnssins_nav_ekf_tag
// {
//     double x[NUM_GNSSINS_EKF_STATE];
// 	double Q[NUM_GNSSINS_EKF_STATE];
// 	double P[SMD(NUM_GNSSINS_EKF_STATE)];
// 	double Fpp[9];
// 	double Fpv[9];
// 	double Fvp[9];
// 	double Fvv[9];
// 	double Fva[9];
// 	double Faa[9];
// 	double Fvba[9];
// 	double Fabg[9];
// 	double Fvba_abg[9];
// } gnssins_nav_ekf_t;

// #endif