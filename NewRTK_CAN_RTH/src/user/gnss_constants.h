#ifndef _GNSS_CONSTANTS_H_
#define _GNSS_CONSTANTS_H_

#define CLIGHT                (299792458.0)            /* speed of light (m/s) */
#define RANGE_MS              (CLIGHT*0.001)           /* range in 1 ms */
#define SC2RAD                (3.1415926535898)        /* semi-circle to radian (IS-GPS) */

#define P2_5                  (0.03125)                /* 2^-5 */
#define P2_6                  (0.015625)               /* 2^-6 */
#define P2_10                 (0.0009765625)           /* 2^-10 */
#define P2_19                 (1.907348632812500E-06)  /* 2^-19 */
#define P2_24				  (5.960464477539063E-08) /* 2^-24 */
#define P2_29                 (1.862645149230957E-09)  /* 2^-29 */
#define P2_31                 (4.656612873077393E-10)  /* 2^-31 */
#define P2_33                 (1.164153218269348E-10)  /* 2^-33 */
#define P2_43                 (1.136868377216160E-13)  /* 2^-43 */
#define P2_50                 (8.881784197001252E-16)  /* 2^-50 */
#define P2_55                 (2.775557561562891E-17)  /* 2^-55 */
#define P2_66                 (1.355252715606880E-20)  /* 2^-66 */

static float ura_table[16] = { 2.4f, 3.4f, 4.85f, 6.85f, 9.65f, 13.65f, 24.0f, 48.0f,
96.0f, 192.0f, 384.0f, 768.0f, 1536.0f, 3072.0f, 6144.0f, 12288.0f };



#endif