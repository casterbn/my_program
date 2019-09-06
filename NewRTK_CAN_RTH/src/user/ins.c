
// #include "gnss_nav_utils.h"
// #include "ins.h"

// #define matrix_at(A, x, y, rownum)    ((A)[(x) + (rownum) * (y)]) 
// #define matrix3x3_at(A, x, y)         ((A)[(x) + (3) * (y)]) 

// #define LEN_DYNAMIC_SMOOTHER        (8)
// #define LEN_INSTALL_SMOOTHER        (10)
// #define LEN_STATIC_ALIGN_SMOOTHER   (INS_NAV_UPDATE_RATE)

// #define INS_STATUS_INACTIVE         (0x00)
// #define INS_STATUS_ACTIVE           (0x01)
// #define INS_STATUS_LEVEL_ATT        (0x02)
// #define INS_STATUS_HEADING_INIT     (0x04)
// #define INS_STATUS_HEADING_GOOD     (0x08)
// #define INS_STATUS_POSVEL_GOOD      (0x10)

// //#define GRAVITY_CONSTANT            (9.7803267715) // m/s2
// #define WGS_E1_SQR		            (0.006694379990141317)	// (A/B)^2-1, 1st numerical eccentricity
// #define WGS_E2_SQR		            (0.006739496742276435)	// 1-(B/A)^2, 2nd numerical eccentricity

// #define IS_INS_ALIGNED(x)           ((x & INS_STATUS_LEVEL_ATT) == INS_STATUS_LEVEL_ATT \
//     && (x & INS_STATUS_HEADING_INIT) == INS_STATUS_HEADING_INIT && (x & INS_STATUS_POSVEL_GOOD) == INS_STATUS_POSVEL_GOOD)

// #define IS_GNSS_POS_VALID(x)        ((x & 0x03) == 0x03)
// #define IS_GNSS_HOR_VEL_VALID(x)    ((x & 0x50) == 0x50)
// #define IS_GNSS_VER_VEL_VALID(x)    ((x & 0x20) == 0x20)

// static imu_data_t imu_data;
// static gnss_data_t gnss_data;

// gnssins_nav_data_t gnssins_nav_data;
// gnssins_nav_ekf_t gnssins_nav_ekf;

// double *ptr_block[15][3];
// const uint8_t block_entry[] = { 0, 1, 3, 6, 10, 15, 21, 28, 36, 45, 55, 66, 78, 91, 105, 120 };

// static void float_mat_vec_mult3d(const float *M, const float *v, float *mv)
// {
//     for (int32_t i = 0; i < 3; i++) {
// 		mv[i] = 0.0f; 
// 		for (int32_t j = 0; j < 3; j++) {
// 			mv[i] += matrix3x3_at(M, i, j)*v[j];
// 		}
// 	}
// }

// static void pos2quat(const ptr_pos_t pos, float *q) 
// {
//     double s1 = 0.0, c1 = 0.0, s2 = 0.0, c2 = 0.0;

//     s1 = sin(pos->lon * 0.5);	
//     c1 = cos(pos->lon * 0.5);
// 	s2 = sin(-PI*0.25 - pos->lat * 0.5); 
//     c2 = cos(-PI*0.25 - pos->lat * 0.5);

// 	q[0] = (float)c1*c2;
// 	q[1] = (float)-s1*s2;
// 	q[2] = (float)c1*s2;
// 	q[3] = (float)c2*s1;
// }

// static void quat2dcm(const float *q, float *dcm)
// {
//     matrix3x3_at(dcm, 0, 0) = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
// 	matrix3x3_at(dcm, 0, 1) = 2.0f * ( q[1] * q[2] - q[0] * q[3] );
// 	matrix3x3_at(dcm, 0, 2) = 2.0f * ( q[1] * q[3] + q[0] * q[2] );

// 	matrix3x3_at(dcm, 1, 0) = 2.0f * ( q[1] * q[2] + q[0] * q[3] );
// 	matrix3x3_at(dcm, 1, 1) = q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3];
// 	matrix3x3_at(dcm, 1, 2) = 2.0f * ( q[2] * q[3] - q[0] * q[1] );

// 	matrix3x3_at(dcm, 2, 0) = 2.0f * ( q[1] * q[3] - q[0] * q[2] );
// 	matrix3x3_at(dcm, 2, 1) = 2.0f * ( q[2] * q[3] + q[0] * q[1] );
// 	matrix3x3_at(dcm, 2, 2) = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
// }

// static void eulerangles2dcm(ptr_euler_angles_t p_ea, float *dcm)
// {
//     float cr, cp, ch, sr, sp, sh;
//     cr = cos(p_ea->gamma); cp = cos(p_ea->theta); ch = cos(p_ea->phi);
//     sr = sin(p_ea->gamma); sp = sin(p_ea->theta); sh = sin(p_ea->phi);

//     matrix3x3_at(dcm, 0, 0) = cp*ch;
//     matrix3x3_at(dcm, 0, 1) = -cr*sh + sr*sp*ch;
//     matrix3x3_at(dcm, 0, 2) = sr*sh + cr*sp*ch;

// 	matrix3x3_at(dcm, 1, 0) = cp * sh;
// 	matrix3x3_at(dcm, 1, 1) = cr*ch + sr*sp*sh;
// 	matrix3x3_at(dcm, 1, 2) = -sr * ch + cr * sp * sh;

// 	matrix3x3_at(dcm, 2, 0) = - sp;
// 	matrix3x3_at(dcm, 2, 1) = sr * cp;
// 	matrix3x3_at(dcm, 2, 2) = cr * cp;
// }

// static double calc_normal_gravity(double lat, float hgt)
// {
//     double s2 = lat*lat;
//     double s4 = s2*s2;

//     /* gravity constant, 9.7803267715 */
//     return (9.7803267715*(1 + 0.0052790414*s2 + 0.0000232718*s4) + \
//         (-0.000003087691089 + 0.000000004397731*s2)*hgt + 0.000000000000721*hgt*hgt);
// }

// static void rearrange_ekf_cov_mat(double *block[][3], double *P)
// {
//     int32_t i, j, k;
//     for (i = 0; i < 5; i++) {
// 		for (j = block_entry[i]; j < block_entry[i + 1]; j++) {
// 			for (k = 0; k < 3; k++) {
// 				block[j][k] = &P[block_entry[3 * i + k] + 3 * (j - block_entry[i])];
//             }
//         }
// 	}
// }

// void gnssins_nav_init()
// {
//     memset(&gnssins_nav_data, 0, sizeof(gnssins_nav_data));
//     matrix3x3_at(gnssins_nav_data.imu_err_cfg.install_mat, 0, 0) = 1.0;
//     matrix3x3_at(gnssins_nav_data.imu_err_cfg.install_mat, 1, 1) = 1.0;
//     matrix3x3_at(gnssins_nav_data.imu_err_cfg.install_mat, 2, 2) = 1.0;

//     gnssins_nav_data.frequency = 20;
//     gnssins_nav_data.gyro_drift[0] = gnssins_nav_data.gyro_drift[1] = gnssins_nav_data.gyro_drift[2] = 0.0f;

//     memset(&gnssins_nav_ekf, 0, sizeof(gnssins_nav_ekf));
//     rearrange_ekf_cov_mat(ptr_block, gnssins_nav_ekf.P);
// }

// const static float thr_static[6] = {0.0001, 0.0001, 0.0001, 0.005, 0.005, 0.005};
// static void identify_dyn_mode(ptr_imu_data_t p_imu)
// {
//     static int32_t cnt_smt = 0;
//     static float gyro_smoother[3] = {0.0};
//     static float acce_smoother[3] = {0.0};
//     static float gyro_diff_smoother[3] = {0.0};
//     static float acce_diff_smoother[3] = {0.0};
//     float gyro_diff = 0.0, acce_diff = 0.0;
//     bool_t b_dyn = FALSE; 

//     cnt_smt++;
//     cnt_smt = MIN(cnt_smt, LEN_DYNAMIC_SMOOTHER);

   
//     for (int32_t j = 0; j < 3; j++) {
//         gyro_smoother[j] -= gyro_smoother[j] / cnt_smt;
//         gyro_smoother[j] += p_imu->gyro[j] / cnt_smt;
//         gyro_diff = ABS(p_imu->gyro[j] - gyro_smoother[j]);

//         acce_smoother[j] -= acce_smoother[j] / cnt_smt;
//         acce_smoother[j] += p_imu->acce[j] / cnt_smt;
//         acce_diff = ABS(p_imu->acce[j] - acce_smoother[j]);

//         gyro_diff_smoother[j] -= gyro_diff_smoother[j] / cnt_smt;
//         gyro_diff_smoother[j] += gyro_diff / cnt_smt;

//         acce_diff_smoother[j] -= acce_diff_smoother[j] / cnt_smt;
//         acce_diff_smoother[j] += acce_diff / cnt_smt;

//         b_dyn |= ((uint8_t)(gyro_diff_smoother[j] > thr_static[j])) << j;
//         b_dyn |= ((uint8_t)(acce_diff_smoother[j] > thr_static[j+3])) << (j+3);
//     }

//     gnssins_nav_data.cnt_static = (cnt_smt > LEN_DYNAMIC_SMOOTHER/2) ? 
//         ((b_dyn == TRUE) ? 0:(gnssins_nav_data.cnt_static+1)) : 0;

//     if (gnssins_nav_data.cnt_static > 0x00FFFFFF) {
//         gnssins_nav_data.cnt_static = 0x00FFFFFF;
//     }

// }

// static void compensate_imu_err(ptr_imu_data_t p_imu)
// {
//     float dt = p_imu->ms_interval*0.001;
//     float gyro[3] = {0.0}, acce[3] = {0.0};
//     imu_err_param_t *p_imu_cfg = &gnssins_nav_data.imu_err_cfg;

//     float_mat_vec_mult3d(p_imu_cfg->install_mat, p_imu->gyro, gyro);
//     float_mat_vec_mult3d(p_imu_cfg->install_mat, p_imu->acce, acce);

//     for (int32_t j = 0; j < 3; j++) {
//         p_imu->acce[j] = acce[j] - p_imu_cfg->acce_bias[j]*dt;

//         if (IS_INS_ALIGNED(gnssins_nav_data.ins_status)) {
//             p_imu->gyro[j] = gyro[j] - (p_imu_cfg->gyro_bias[j] + gnssins_nav_data.gyro_drift[j]*DEG2RAD)*dt;
//         }
//     }
// }

// static void determine_line_motion(ptr_imu_data_t p_imu)
// {
//     static float gyro[5];
//     static uint8_t num_gyro = 0, idx_gyro = 0;

//     gnssins_nav_data.gyro_rate = (p_imu->gyro[2]*RAD2DEG) / (p_imu->ms_interval*0.001);
//     if (ABS(gnssins_nav_data.gyro_rate) > 10) {
//         gnssins_nav_data.flag_gyro_rate = 1;
//     }
    
//     num_gyro = (ABS(gnssins_nav_data.gyro_rate) < 3) ? (num_gyro+1) : 0;
//     uint8_t thr = (gnssins_nav_data.frequency >> 1)*5;
//     if (num_gyro >= thr) {
//         num_gyro = thr;
//         gnssins_nav_data.flag_line_motion = 1;
//     } else {
//         gnssins_nav_data.flag_line_motion = 0;
//     }


//     if (idx_gyro < 5) {
//         gyro[idx_gyro] = gnssins_nav_data.gyro_rate;
//         idx_gyro++;
//     } else {
//         for (int i = 1; i < 5; i++) {
//             gyro[i-1] = gyro[i];
//         }
//         gyro[4] = gnssins_nav_data.gyro_rate;

//         gnssins_nav_data.gyro_diff = (gyro[4] - gyro[1]) / 0.3f;
//     }

// }

// static void init_gyro_bias(ptr_imu_data_t p_imu)
// {
//     static uint32_t cnt_smt = 0;
//     float gyro[3] = {0.0};
//     float dt = p_imu->ms_interval*0.001;

//     if (0 == gnssins_nav_data.cnt_static) {
//         return;
//     }

//     for (int j = 0; j < 3; j++) {
//         gyro[j] = p_imu->gyro[j] / dt;
//     }

//     cnt_smt++;
//     cnt_smt = MIN(cnt_smt, 30*INS_NAV_UPDATE_RATE);
//     ptr_imu_err_param_t p_imu_cfg = &gnssins_nav_data.imu_err_cfg;
//     for (int j = 0; j < 3; j++) {
//         p_imu_cfg->gyro_bias[j] -= p_imu_cfg->gyro_bias[j] / cnt_smt;
//         p_imu_cfg->gyro_bias[j] += gyro[j] / cnt_smt;
//     }

//     /* TODO if installation matrix is given? */
// }

// static void gnssins_init_ekf()
// {
//     gnssins_nav_ekf.P[0] = gnssins_nav_ekf.P[2] = 25.0;
//     gnssins_nav_ekf.P[5] = 64.0;
// 	gnssins_nav_ekf.P[9] = gnssins_nav_ekf.P[14] = 0.25;
// 	gnssins_nav_ekf.P[20] = 0.64;
// 	gnssins_nav_ekf.P[27] = gnssins_nav_ekf.P[35] = gnssins_nav_ekf.P[44] = 1.0*1.0*DEG2RAD*DEG2RAD;
// 	gnssins_nav_ekf.P[54] = gnssins_nav_ekf.P[65] = gnssins_nav_ekf.P[77] = 0.5*0.5; //ACC_CONST_BIAS*ACC_CONST_BIAS;//
// 	gnssins_nav_ekf.P[90] = gnssins_nav_ekf.P[104] = gnssins_nav_ekf.P[119] = 0.0005*0.0005; //GYRO_CONST_BIAS*GYRO_CONST_BIAS;//
// }

// static void init_install_misalign_mat()
// {
//     ;
// }

// static void ins_alignment(ptr_imu_data_t p_imu, ptr_gnss_data_t p_gnss)
// {
//     static float acce_smoother[3] = {0.0};
//     float sum_acce[3] = {0.0f};
//     float hdg = 0.0f, tmp = 0.0f;
//     int32_t cnt_smt = 0;
//     int8_t flag_att = 0, flag_run = 0;

//     if (gnssins_nav_data.cnt_static > 0 && (gnssins_nav_data.imu_err_cfg.install_misalign_flag >= 0x03)
//         && (gnssins_nav_data.ins_status & INS_STATUS_LEVEL_ATT == 0)) {

//         sum_acce[0] += p_imu->acce[0];
//         sum_acce[1] += p_imu->acce[1];
//         sum_acce[2] += p_imu->acce[2];
//         cnt_smt = MIN(gnssins_nav_data.cnt_static, LEN_STATIC_ALIGN_SMOOTHER);

//         acce_smoother[0] += (sum_acce[0] - acce_smoother[0]) / cnt_smt;
//         acce_smoother[1] += (sum_acce[1] - acce_smoother[1]) / cnt_smt;
//         acce_smoother[2] += (sum_acce[2] - acce_smoother[2]) / cnt_smt;

//         tmp = sqrtf(SQR(acce_smoother[0]) + SQR(acce_smoother[1]) + SQR(acce_smoother[2]));
//         gnssins_nav_data.euler_angles.theta = asin(acce_smoother[0]/tmp);
//         gnssins_nav_data.euler_angles.gamma = -asin(acce_smoother[1]/(cos(gnssins_nav_data.euler_angles.theta)*tmp));
//         eulerangles2dcm(&gnssins_nav_data.euler_angles, gnssins_nav_data.dcm_bn); // euler2quat, quat2dcm?
//         if (gnssins_nav_data.cnt_static > MAX_NUM_STATIC_ALIGN) {
//             gnssins_nav_data.ins_status |= INS_STATUS_LEVEL_ATT;
//             flag_att = 1;
//         }
//     } else {
//         acce_smoother[0] = acce_smoother[1] = acce_smoother[2] = 0.0f;
//     }


//     if (p_gnss_data && IS_GNSS_POS_VALID(p_gnss_data->nav_flag)) {
//         memcpy(&gnssins_nav_data.pos, &p_gnss_data->pos, sizeof(pos_t));
//         pos2quat(&gnssins_nav_data.pos, gnssins_nav_data.q_ne);
//         quat2dcm(gnssins_nav_data.q_ne, gnssins_nav_data.dcm);

//         gnssins_nav_data.gravity = calc_normal_gravity(gnssins_nav_data.pos.lat, gnssins_nav_data.pos.hgt);

//         float C31 = matrix3x3_at(gnssins_nav_data.dcm, 2, 0), C33 = matrix3x3_at(gnssins_nav_data.dcm, 2, 2);
//         float tmp = sqrt(1.0 - WGS_E1_SQR*C33*C33);
//         gnssins_nav_data.rm = WGS84_A*(1 - WGS_E1_SQR)/(tmp*tmp*tmp);
//         gnssins_nav_data.rn = WGS84_A/tmp;
//         gnssins_nav_data.wie[0] = EARTH_ROTATION_RATE*C31;
//         gnssins_nav_data.wie[1] = 0.0f;
//         gnssins_nav_data.wie[2] = EARTH_ROTATION_RATE*C33;

//         flag_run = (gnssins_nav_data.flag_reset) ? 1 : ((gnssins_nav_data.gst_status >= 1) ? 1 : 0);

//         if (IS_GNSS_HOR_VEL_VALID(p_gnss_data->nav_flag) && flag_run) {
//             memcpy(&gnssins_nav_data.vel, &p_gnss_data->vel, sizeof(vel_t));
//             gnssins_nav_data.cnt_ekf = 0;

//             tmp = sqrt(SQR(p_gnss->vel.ve) + SQR(p_gnss->vel.vn));

//             if (tmp > MAX_NUM_ALIGN_HEADING_VEL && 1 == flag_att && gnssins_nav_data.imu_err_cfg.install_misalign_flag >= 0x03) {
//                 // && gnssins)
//                 num_start++;
//             } else {
//                 num_start = 0;
//             }

//             if (num_start > 10) {
//                 num_start = 10;
                
//                 hdg = atan2(gnssins_nav_data.vel.ve, gnssins_nav_data.vel.vn);
//                 if (hdg*RAD2DEG <= 360.0f) {
//                     gnssins_nav_data.euler_angles.phi = hdg;

//                     if (gnssins_nav_data.ins_status & 0 == INS_STATUS_LEVEL_ATT) {
//                         gnssins_nav_data.euler_angles.gamma = 0;
//                         gnssins_nav_data.euler_angles.theta = atan(gnssins_nav_data.vel.vu / tmp);
//                         gnssins_nav_data.ins_status |= INS_STATUS_LEVEL_ATT;
//                     }

//                     eulerangles2dcm(&gnssins_nav_data.euler_angles, gnssins_nav_data.dcm_bn); // euler2quat, quat2dcm
//                     gnssins_nav_data.dvel.ve = gnssins_nav_data.dvel.vn = gnssins_nav_data.dvel.vu = 0.0f;
//                     gnssins_nav_data.sf_n[0] = gnssins_nav_data.sf_n[1] = gnssins_nav_data.sf_n[2] = 0.0f;

//                     gnssins_nav_data.wen[0] = gnssins_nav_data.vel.ve / (gnssins_nav_data.rn + gnssins_nav_data.pos.hgt);
//                     gnssins_nav_data.wen[1] = -gnssins_nav_data.vel.vn / (gnssins_nav_data.rm + gnssins_nav_data.pos.hgt);
//                     gnssins_nav_data.wen[2] = gnssins_nav_data.vel.ve*matrix3x3_at(gnssins_nav_data.dcm_ne, 2, 2) \
//                         / (matrix3x3_at(gnssins_nav_data.dcm_ne, 2, 0)*(gnssins_nav_data.rn + gnssins_nav_data.pos.hgt));
                
//                     gnssins_nav_data.ins_status |= INS_STATUS_HEADING_INIT;
//                     num_start = 0;
//                 }
//             }
                
//         }

//         if (gnssins_nav_data.flag_reset) {
//             if (gnssins_nav_data.gst_status >= 4 && gnssins_nav_data.flag_heading != 0) { 
//                 gnssins_nav_data.ins_status |= INS_STATUS_POSVEL_GOOD;
//                 // posquality = bad
//             }
//         } else {
//             if (gnssins_nav_data.flag_heading != 0) {
//                 gnssins_nav_data.ins_status |= INS_STATUS_POSVEL_GOOD;
//                 // posquality = bad
//             }
//         }

        
//     }


// }


// bool_t gnssins_nav_proc()
// {
//     int32_t i, j;

//     /* place holders */
//     ptr_imu_data_t p_imu_data = &imu_data;
//     ptr_gnss_data_t p_gnss_data = &gnss_data;

//     if (p_gnss_data) {

//     }

    
//     identify_dyn_mode(p_imu_data);

//     compensate_imu_err(p_imu_data);

//     determine_line_motion(p_imu_data);


//     if (!IS_INS_ALIGNED(gnssins_nav_data.ins_status)) {
//         if (0 == gnssins_nav_data.imu_err_cfg.install_misalign_flag) {
//             init_install_misalign_mat();
//         } else {
//             gnssins_nav_data.imu_err_cfg.install_misalign_flag = 0x03;
//         }

//         init_gyro_bias(p_imu_data);

//         // ins_alignment();

        

//         if (IS_INS_ALIGNED(gnssins_nav_data.ins_status)) {
//             gnssins_init_ekf();
//             gnssins_nav_data.cnt_ekf = 0;
//             gnssins_nav_data.cnt_gnss_halt = 0;
//             gnssins_nav_data.cnt_ms_ins_alone = 200000;
//         }
//     } else {


//         ins_mechanization();
//     }

// }