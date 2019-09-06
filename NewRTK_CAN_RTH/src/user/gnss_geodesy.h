#ifndef _GNSS_GEODESY_H_
#define _GNSS_GEODESY_H_


bool_t 
ecef2plh(const double *from, double *to);

bool_t 
plh2ecef(const double *from, double *to);

bool_t 
xyz2enu(const double lat, const double lon, const double *dECEF, double *dENU);

bool_t 
enu2xyz(const double lat, const double lon, const double *dENU, double *dECEF);

double 
compute_range(double *from, double *to);

bool_t 
compute_MN_radius(const double latitude, double *M, double *N);

bool_t 
get_sataziele(const double *user_pos, const double *sat_pos, double *azimuth, double *elevation);

double 
get_tropocorr(const double sv_ele, const double pressure, const double temperature, const double part_press_H2O);


#endif
