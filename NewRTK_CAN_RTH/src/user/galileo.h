#ifndef _GALILEO_H_
#define _GALILEO_H_

#include <stdint.h>

#define MAX_GAL_NUMBER_SAT              (32)


typedef struct gal_eph_channel_tag {
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
} gal_eph_channel_t;


typedef struct gal_eph_tag {
	gal_eph_channel_t svid[MAX_GAL_NUMBER_SAT];
} gal_eph_t;

#endif