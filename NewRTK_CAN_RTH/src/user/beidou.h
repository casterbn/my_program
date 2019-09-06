#ifndef _BEIDOU_H_
#define _BEIDOU_H_

#include <stdint.h>

#define MAX_BDS_NUMBER_SAT              (35)


typedef struct bds_eph_channel_tag {
	bool              is_update;
    int               prn;
    uint32_t          health;
	int               week;
    double            tow;
    double            tgd;
    uint32_t          aodc; 
    double            toc;
    double            ttr;
    double            af2;
    double            af1;
    double            af0;
    double            toe;
    uint32_t          aode;
    int               sva;
    double            crs;
    double            deltan;
    double            m0;
    double            cuc;
    double            e;
    double            cus;
    double            sqrta;
    double            cic;
    double            omega0;
    double            cis;
    double            i0;
    double            crc;
    double            omega;
    double            omegadot;
    double            idot;
} bds_eph_channel_t;


typedef struct bds_eph_tag {
	bds_eph_channel_t svid[MAX_BDS_NUMBER_SAT];
} bds_eph_t;

#endif