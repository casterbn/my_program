#include <math.h>

#include "gnss_nav_utils.h"
#include "gnss_geodesy.h"


bool_t 
ecef2plh(const double *from, double *to) 
{
	double x = from[0];
	double y = from[1];
	double z = from[2];

	double lon,lat,hgt,p,dtmp,sinlat,N;

	if( x == 0.0 && y == 0.0 ) {    
		lon = 0.0; 
		if( z < 0 ){
			hgt = -z - WGS84_B;
			lat = -HALFPI;
        }
        else{
			hgt = z - WGS84_B;
			lat = HALFPI;
        }
    }
    else {
		p = sqrt(x * x + y * y);
        lon = 2.0 * atan2(y , (x + p));
     
        lat = atan(z / (p * (1.0 - WGS84_E2)));
        hgt = 0.0;
        do {  
			dtmp = hgt;
			sinlat = sin(lat);
			N   = WGS84_A / sqrt(1.0 - WGS84_E2 * sinlat * sinlat);
			hgt = p / cos(lat) - N;
			lat = atan(z/(p * (1.0 - WGS84_E2 * N / (N + hgt))));      
		} while(fabs(hgt - dtmp) > 0.00001);  // 0.1 mm convergence for height
    }
 
   to[0] = lat;
   to[1] = lon;  
   to[2] = hgt;

   return TRUE;
}


bool_t 
plh2ecef(const double *from, double *to) 
{
	double sinlat, N, dtmp;

	double latitude = from[0];
	double longitude = from[1];
	double height = from[2];

	sinlat = sin(latitude);             
    N = WGS84_A / sqrt(1.0 - WGS84_E2 * sinlat * sinlat);      
    dtmp = (N + height) * cos(latitude);

    to[0] = dtmp * cos(longitude);
    to[1] = dtmp * sin(longitude);
    to[2] = ((1.0 - WGS84_E2) * N + height) * sinlat;
 
    return TRUE;
}


bool_t 
xyz2enu(const double lat,  const double lon, const double *dECEF, double *ENU)  
{
   double sinlat;
   double coslat;
   double sinlon;
   double coslon;
 
   sinlat = sin(lat);
   coslat = cos(lat);
   sinlon = sin(lon);
   coslon = cos(lon);
   
   ENU[0] = -sinlon        * dECEF[0]  +  coslon        * dECEF[1];
   ENU[1] = -sinlat*coslon * dECEF[0]  -  sinlat*sinlon * dECEF[1]  +  coslat * dECEF[2];  
   ENU[2] =  coslat*coslon * dECEF[0]  +  coslat*sinlon * dECEF[1]  +  sinlat * dECEF[2];  
 
   return TRUE;
}


bool_t 
enu2xyz(const double lat, const double lon,  const double  *dENU, double *dECEF) 
{               
   double sinlat;
   double coslat;
   double sinlon;
   double coslon;

   sinlat = sin(lat);
   coslat = cos(lat);
   sinlon = sin(lon);
   coslon = cos(lon);
   
   dECEF[0] = -sinlat*coslon * dENU[1]  -  sinlon * dENU[0]  +  coslat * coslon * dENU[2];
   dECEF[1] = -sinlat*sinlon * dENU[1]  +  coslon * dENU[0]  +  coslat * sinlon * dENU[2];
   dECEF[2] =  coslat        * dENU[1]                       +  sinlat          * dENU[2];
   
   return TRUE;
}
 
bool_t 
compute_MN_radius(const double latitude,  double *M, double *N)
{
   double dtmp;
   double W;
    
   W = sin(latitude);
   W = sqrt(1.0 - WGS84_E2 * W * W );  
   dtmp = W * W * W;                 
   
   *M = WGS84_A * (1.0 - WGS84_E2) / dtmp;
   *N = WGS84_A / W;
   
   return TRUE;
}


double 
compute_range(double* from, double* to) 
{
	double dx, dy, dz;

	dx = from[0] - to[0];
	dy = from[1] - to[1];
	dz = from[2] - to[2];

	return sqrt(dx * dx + dy * dy + dz * dz);
}


bool_t 
get_sataziele(const double *user_pos, const double *sat_pos, double *azimuth, double *elevation) 
{
	double user_plh[3];
	double dECEF[3];
	double dENU[3];
	double tmp;
	
	ecef2plh(user_pos, user_plh);

    dECEF[0] = sat_pos[0] - user_pos[0];
	dECEF[1] = sat_pos[1] - user_pos[1];
	dECEF[2] = sat_pos[2] - user_pos[2];
    
    xyz2enu(user_plh[0], user_plh[1], dECEF, dENU);

    tmp  = sqrt(dENU[0] * dENU[0] + dENU[1] * dENU[1]);
    
    *elevation = atan(dENU[2] / tmp);
    *azimuth   = atan2(dENU[0], dENU[1]);

	return TRUE;
}


static double 
troposphere_correction(const double sv_ele, const double layer_height, const double refractivity) 
{
   double tropCorrection = 0.0;
   int32_t j;
   double elevation_angle = sv_ele < 0.0 ? 0.0 : sv_ele;
   const double earth_radius = 6371000.0;
   double r = sqrt(pow(earth_radius + layer_height, 2.0) - pow(earth_radius * cos(elevation_angle), 2))
			   - earth_radius * sin(elevation_angle);

	// compute coefficients
   double a = -sin(elevation_angle) / layer_height;
   double b = -pow(cos(elevation_angle), 2) / (2.0 * layer_height * earth_radius);

   // compute coefficients for a series expansion
   double alpha[9];

	alpha[0] = 1.0;
	alpha[1] = 4.0 *a;
	alpha[2] = 6.0 * a * a + 4.0 * b;
	alpha[3] = 4.0 * a * (a * a + 3.0 * b);
	alpha[4] = a * a * a * a + 12.0 * a * a * b + 6.0 * b * b;
	alpha[5] = 4.0 * a * b * (a * a + 3.0 * b);
	alpha[6] = b * b * (6.0 * a * a + 4.0 * b);
	alpha[7] = 4.0 * a * b * b * b;
	alpha[8] = b * b * b * b;

	// evaluate the series
	for (j = 0 ; j < 9 ; ++j)
		tropCorrection += alpha[j] * pow(r, j + 1) / (j + 1);

	// return the delay in units of metres
	return tropCorrection * 1.0e-6 * refractivity;
}


double 
get_tropocorr(const double sv_ele, const double pressure, const double temperature, const double part_press_H2O) 
{
	// compute refractivity
   double nDry = 77.64 * pressure / (temperature + 273.15);

   double hDry = 40136.0 + 148.72 * temperature;

   double tropDry = troposphere_correction(sv_ele, hDry, nDry);

   double nWet = -12.96 * part_press_H2O / (temperature + 273.16) 
               + 3.718e5 * part_press_H2O / pow(temperature + 273.16, 2);

   double hWet = 11000.0;

   double tropWet = troposphere_correction(sv_ele, hWet, nWet);

   return tropDry + tropWet;
}

