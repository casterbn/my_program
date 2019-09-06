#ifndef _GNSS_NAV_UTILS_H_
#define _GNSS_NAV_UTILS_H_

#include <stdint.h>

#ifndef NULL
	#ifdef __cplusplus
		#define NULL        (0L)
	#else /* !__cplusplus */
		#define NULL        ((void*) 0)
	#endif /* !__cplusplus */
#endif

typedef int8_t bool_t;

#ifndef	FALSE
	#define	FALSE	(0)
#endif

#ifndef	TRUE
	#define	TRUE	(!FALSE)
#endif

#undef	MAX
#define MAX(a, b)  (((a) > (b)) ? (a) : (b))

#undef	MIN
#define MIN(a, b)  (((a) < (b)) ? (a) : (b))

#undef	ABS
#define ABS(a)	   (((a) < 0) ? -(a) : (a))

#ifndef SIGN
#define SIGN(a)    ((a > 0) ? 1 : ((a < 0) ? -1 : 0))
#endif

#undef SQR
#define SQR(a)      ((a)*(a))

#define    LIGHTSPEED           (299792458.0)  

#define    L1_WAVELENGTH        (LIGHTSPEED/1575.42e6)

#define    PI_GPS               (3.1415926535898)
#ifndef PI
#define    PI                   (3.1415926535897932) //!< better value
#endif
#ifndef TWOPI
#define    TWOPI                (6.2831853071795865)  //!< 2.0*PI
#endif
#ifndef HALFPI
#define    HALFPI               (1.5707963267948966)    //!< PI/2.0
#endif
#undef RAD2DEG
#define    RAD2DEG              (57.295779513082320)
// #endif
#ifndef DEG2RAD
#define    DEG2RAD              (0.017453292519943)
#endif

#define    WGS84_A              (6378137.0)
#define    WGS84_F_INV          (298.257223563)       
#define    WGS84_B              (6356752.31424518)
#define    WGS84_E2             (0.00669437999014132)

#define    GRAVITY_CONSTANT     (3.986005e14)
#define    EARTH_ROTATION_RATE  (7.2921151467e-05)

#define FLT_EPSILON       1.192093e-007
#define DBL_EPSILON       2.2204460492503131e-016

#define NEAR_ZERO(val)    (((val) > -FLT_EPSILON) && ((val) < FLT_EPSILON))

uint32_t getbitu(const uint8_t *buff, int pos, int len);

int32_t getbits(const uint8_t *buff, int pos, int len);

void *nav_memcpy(void *dest, const void *src, uint32_t count);

#endif