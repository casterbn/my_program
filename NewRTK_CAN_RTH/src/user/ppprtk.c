#include "ppprtk.h"

/*--------------------------------------------------------*/
/* code from rtklib to decode RTCM3 */

#include <time.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#define AU 149597870691.0 /* 1 AU (m) */
#define PI 3.1415926535897932 /* pi */
#define D2R (PI / 180.0)      /* deg to rad */
#define R2D (180.0 / PI)      /* rad to deg */
#define DEG2RAD (0.017453292519943)
#define ROUND(x) ((int)floor((x) + 0.5))
#define MAX_ITER 8       /* max number of iterations */
#define MAX_STD_FIX 0.15 /* max std-dev (3d) to fix solution */
#define MIN_NSAT_SOL 4   /* min satellite number for solution */
#define THRES_REJECT 4.0 /* reject threshold of posfit-res (sigma) */
#define THRES_MW_JUMP 10.0
#define THRES_DA_JUMP 0.5
#define THRES_GF_JUMP 1.5
#define HION 350000.0 /* ionosphere height (m) */

#define VAR_POS SQR(60.0)    /* init variance receiver position (m^2) */
#define VAR_VEL SQR(10.0)    /* init variance of receiver vel ((m/s)^2) */
#define VAR_ACC SQR(10.0)    /* init variance of receiver acc ((m/ss)^2) */
#define VAR_CLK SQR(60.0)    /* init variance receiver clock (m^2) */
#define VAR_ZTD SQR(0.6)     /* init variance ztd (m^2) */
#define VAR_GRA SQR(0.01)    /* init variance gradient (m^2) */
#define VAR_DCB SQR(30.0)    /* init variance dcb (m^2) */
#define VAR_BIAS SQR(60.0)   /* init variance phase-bias (m^2) */
#define VAR_IONO SQR(60.0)   /* init variance iono-delay */
#define VAR_GLO_IFB SQR(0.6) /* variance of glonass ifb */

#define ERR_SAAS 0.3   /* saastamoinen model error std (m) */
#define ERR_BRDCI 0.5  /* broadcast iono model error factor */
#define ERR_CBIAS 0.3  /* code bias error std (m) */
#define REL_HUMI 0.7   /* relative humidity for saastamoinen model */
#define GAP_RESION 120 /* default gap to reset ionos parameters (ep) */

#define EFACT_GPS_L5 10.0 /* error factor of GPS/QZS L5 */

#define MUDOT_GPS (0.00836 * D2R) /* average angular velocity GPS (rad/s) */
#define MUDOT_GLO (0.00888 * D2R) /* average angular velocity GLO (rad/s) */
#define EPS0_GPS (13.5 * D2R)     /* max shadow crossing angle GPS (rad) */
#define EPS0_GLO (14.2 * D2R)     /* max shadow crossing angle GLO (rad) */
#define T_POSTSHADOW 1800.0       /* post-shadow recovery time (s) */
#define QZS_EC_BETA 20.0          /* max beta angle for qzss Ec (deg) */

#define AS2R (D2R / 3600.0) /* arc sec to radian */
#define GME 3.986004415E+14 /* earth gravitational constant */
#define GMS 1.327124E+20    /* sun gravitational constant */
#define GMM 4.902801E+12    /* moon gravitational constant */

#define FREQ1 1.57542E9      /* L1/E1  frequency (Hz) */
#define FREQ2 1.22760E9      /* L2     frequency (Hz) */
#define FREQ1_CMP 1.561098E9 /* BeiDou B1 frequency (Hz) */

/* number and index of states */
#define NF(opt) ((opt)->ionoopt == IONOOPT_IF12 ? 1 : (opt)->nf)                             /* number of frquencies */
#define NP(opt) ((opt)->dynamics ? 9 : 3)                                                    /* number of position solution */
#define NC(opt) (NSYS)                                                                       /* number of clock solution */
#define NT(opt) ((opt)->tropopt < TROPOPT_EST ? 0 : ((opt)->tropopt == TROPOPT_EST ? 1 : 3)) /* number of tropospheric parameters */
#define NR(opt) (NP(opt) + NC(opt) + NT(opt))
#define NB(opt) (MAXOBS)
#define IC(s, opt) (NP(opt) + (s))      /* state index of clocks (s=0:gps,1:glo,2:bds,3:gal) */
#define IT(opt) (NP(opt) + NC(opt))     /* state index of tropospheric parameters */
#define IB(s, f, opt) (NR(opt) + (s)-1) /* state index of phase ambiguity parameters */

typedef struct
{
    int mode;           /* positioning mode (PMODE_???) */
    int nf;             /* number of frequencies (1:L1,2:L1+L2,3:L1+L2+L5) */
    int navsys;         /* navigation system */
    int gnsisb;         /* stochastic modeling for ISBs in multi-GNSS processing */
    int gloicb;         /* considering GLONASS code inter-channel biases */
    double elmin;       /* elevation mask angle (rad) */
    int sateph;         /* satellite ephemeris/clock (EPHOPT_???) */
    int modear;         /* AR mode (0:off,1:continuous,2:instantaneous,3:fix and hold,4:ppp-ar) */
    int glomodear;      /* GLONASS AR mode (0:off,1:on,2:auto cal,3:ext cal) */
    int bdsmodear;      /* BeiDou AR mode (0:off,1:on) */
    int maxout;         /* obs outage count to reset bias */
    int ionoopt;        /* ionosphere option (IONOOPT_???) */
    int tropopt;        /* troposphere option (TROPOPT_???) */
    int ionopnoise;     /* ionosphere noise */
    int ion_const;      /*constraint of using extern ionospheric product, 0:off, 1:on */
    int ion_const_mode; /*constraint schemes of using extern ionospheric product, 1:constant constraint, 2:spatial-temporal constraint, 3:stepwise-relaxed constraint */
    int dynamics;       /* dynamics model (0:none,1:velociy,2:accel) */
    int tidecorr;       /* earth tide correction (0:off,1:solid,2:solid+otl+pole) */
    int niter;          /* number of filter iteration */
    double ru[3];       /* user position */
    double std[3];      /* initial-state std [0]bias,[1]iono [2]trop */
    double prn[6];      /* process-noise std [0]bias,[1]iono [2]trop [3]acch [4]accv [5] pos */
    double thresslip;   /* slip threshold of geometry-free phase (m) */
    double maxtdiff;    /* max difference of time (sec) */
    double maxinno;     /* reject threshold of innovation (m) */
    double maxgdop;     /* reject threshold of gdop */
    double err[5];
    double eratio[NFREQ];
    char anttype[20];     /* antenna types rover */
    double antdel[3];     /* antenna delta {{rov_e,rov_n,rov_u}} */
    double pco[NFREQ][3]; /* receiver antenna parameters {rov,base} */
    int posopt[6];        /* positioning options */
} prcopt_t;

prcopt_t prcopt = {7, 2, 5, 0, 0, 10 * D2R, 1, 0, 0, 0, 20, 3, 3, 1, 0, 3, 0, /* mode ~ dynamics     */
                   1,
                   1,
                   {0.0, 0.0, 0.0},
                   {30.0, 0.03, 0.3},
                   {1E-4, 1E-3, 1E-4, 1E-1, 1E-2, 0.0},
                   0.1, /* tidecorr ~ thresslip*/
                   30.0,
                   1000.0,
                   30.0,
                   {100.0, 0.003, 0.003, 0.0, 1.0},
                   {100.0, 100.0},
                   {"FemtomesAnt"},
                   {0.0, 0.0, 0.0},
                   {{0.0, 0.0, 0.0475}, {0.0, 0.0, 0.0425}},
                   {100.0, 0.003, 0.0, 0.0, 0.0, 0.0}};

typedef struct
{ /* satellite status type */
    int sat;
    int slip[NFREQ]; /* cycle-slip flag */
    double gf;       /* geometry-free phase L1-L2 (m) */
#ifdef _USE_PPP_
    unsigned char vs;          /* valid satellite flag single */
    double azel[2];            /* azimuth/elevation angles {az,el} (rad) */
    unsigned char vsat[NFREQ]; /* valid satellite flag */
    unsigned int rejc[NFREQ];  /* reject counter */
    unsigned int outc[NFREQ];
    double mw;  /* MW-LC (m) */
    double phw; /* phase windup (cycle) */
#else
    double L[NFREQ];
    double D[NFREQ];
    double dph[NFREQ];
    double rbias;
    double rstd;
#endif
} ssat_t;
ssat_t ssat[MAXOBS]; /* satellite status */

#ifdef _ON_LINE_
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

/* coordinate rotation matrix ------------------------------------------------*/
#define Rx(t, X)                                 \
    do                                           \
    {                                            \
        (X)[0] = 1.0;                            \
        (X)[1] = (X)[2] = (X)[3] = (X)[6] = 0.0; \
        (X)[4] = (X)[8] = cos(t);                \
        (X)[7] = sin(t);                         \
        (X)[5] = -(X)[7];                        \
    } while (0)

#define Ry(t, X)                                 \
    do                                           \
    {                                            \
        (X)[4] = 1.0;                            \
        (X)[1] = (X)[3] = (X)[5] = (X)[7] = 0.0; \
        (X)[0] = (X)[8] = cos(t);                \
        (X)[2] = sin(t);                         \
        (X)[6] = -(X)[2];                        \
    } while (0)

#define Rz(t, X)                                 \
    do                                           \
    {                                            \
        (X)[8] = 1.0;                            \
        (X)[2] = (X)[5] = (X)[6] = (X)[7] = 0.0; \
        (X)[0] = (X)[4] = cos(t);                \
        (X)[3] = sin(t);                         \
        (X)[1] = -(X)[3];                        \
    } while (0)

/* iau 1980 nutation ---------------------------------------------------------*/
static void nut_iau1980(double t, const double *f, double *dpsi, double *deps)
{
    static const double nut[106][10] = {
        {0, 0, 0, 0, 1, -6798.4, -171996, -174.2, 92025, 8.9},
        {0, 0, 2, -2, 2, 182.6, -13187, -1.6, 5736, -3.1},
        {0, 0, 2, 0, 2, 13.7, -2274, -0.2, 977, -0.5},
        {0, 0, 0, 0, 2, -3399.2, 2062, 0.2, -895, 0.5},
        {0, -1, 0, 0, 0, -365.3, -1426, 3.4, 54, -0.1},
        {1, 0, 0, 0, 0, 27.6, 712, 0.1, -7, 0.0},
        {0, 1, 2, -2, 2, 121.7, -517, 1.2, 224, -0.6},
        {0, 0, 2, 0, 1, 13.6, -386, -0.4, 200, 0.0},
        {1, 0, 2, 0, 2, 9.1, -301, 0.0, 129, -0.1},
        {0, -1, 2, -2, 2, 365.2, 217, -0.5, -95, 0.3},
        {-1, 0, 0, 2, 0, 31.8, 158, 0.0, -1, 0.0},
        {0, 0, 2, -2, 1, 177.8, 129, 0.1, -70, 0.0},
        {-1, 0, 2, 0, 2, 27.1, 123, 0.0, -53, 0.0},
        {1, 0, 0, 0, 1, 27.7, 63, 0.1, -33, 0.0},
        {0, 0, 0, 2, 0, 14.8, 63, 0.0, -2, 0.0},
        {-1, 0, 2, 2, 2, 9.6, -59, 0.0, 26, 0.0},
        {-1, 0, 0, 0, 1, -27.4, -58, -0.1, 32, 0.0},
        {1, 0, 2, 0, 1, 9.1, -51, 0.0, 27, 0.0},
        {-2, 0, 0, 2, 0, -205.9, -48, 0.0, 1, 0.0},
        {-2, 0, 2, 0, 1, 1305.5, 46, 0.0, -24, 0.0},
        {0, 0, 2, 2, 2, 7.1, -38, 0.0, 16, 0.0},
        {2, 0, 2, 0, 2, 6.9, -31, 0.0, 13, 0.0},
        {2, 0, 0, 0, 0, 13.8, 29, 0.0, -1, 0.0},
        {1, 0, 2, -2, 2, 23.9, 29, 0.0, -12, 0.0},
        {0, 0, 2, 0, 0, 13.6, 26, 0.0, -1, 0.0},
        {0, 0, 2, -2, 0, 173.3, -22, 0.0, 0, 0.0},
        {-1, 0, 2, 0, 1, 27.0, 21, 0.0, -10, 0.0},
        {0, 2, 0, 0, 0, 182.6, 17, -0.1, 0, 0.0},
        {0, 2, 2, -2, 2, 91.3, -16, 0.1, 7, 0.0},
        {-1, 0, 0, 2, 1, 32.0, 16, 0.0, -8, 0.0},
        {0, 1, 0, 0, 1, 386.0, -15, 0.0, 9, 0.0},
        {1, 0, 0, -2, 1, -31.7, -13, 0.0, 7, 0.0},
        {0, -1, 0, 0, 1, -346.6, -12, 0.0, 6, 0.0},
        {2, 0, -2, 0, 0, -1095.2, 11, 0.0, 0, 0.0},
        {-1, 0, 2, 2, 1, 9.5, -10, 0.0, 5, 0.0},
        {1, 0, 2, 2, 2, 5.6, -8, 0.0, 3, 0.0},
        {0, -1, 2, 0, 2, 14.2, -7, 0.0, 3, 0.0},
        {0, 0, 2, 2, 1, 7.1, -7, 0.0, 3, 0.0},
        {1, 1, 0, -2, 0, -34.8, -7, 0.0, 0, 0.0},
        {0, 1, 2, 0, 2, 13.2, 7, 0.0, -3, 0.0},
        {-2, 0, 0, 2, 1, -199.8, -6, 0.0, 3, 0.0},
        {0, 0, 0, 2, 1, 14.8, -6, 0.0, 3, 0.0},
        {2, 0, 2, -2, 2, 12.8, 6, 0.0, -3, 0.0},
        {1, 0, 0, 2, 0, 9.6, 6, 0.0, 0, 0.0},
        {1, 0, 2, -2, 1, 23.9, 6, 0.0, -3, 0.0},
        {0, 0, 0, -2, 1, -14.7, -5, 0.0, 3, 0.0},
        {0, -1, 2, -2, 1, 346.6, -5, 0.0, 3, 0.0},
        {2, 0, 2, 0, 1, 6.9, -5, 0.0, 3, 0.0},
        {1, -1, 0, 0, 0, 29.8, 5, 0.0, 0, 0.0},
        {1, 0, 0, -1, 0, 411.8, -4, 0.0, 0, 0.0},
        {0, 0, 0, 1, 0, 29.5, -4, 0.0, 0, 0.0},
        {0, 1, 0, -2, 0, -15.4, -4, 0.0, 0, 0.0},
        {1, 0, -2, 0, 0, -26.9, 4, 0.0, 0, 0.0},
        {2, 0, 0, -2, 1, 212.3, 4, 0.0, -2, 0.0},
        {0, 1, 2, -2, 1, 119.6, 4, 0.0, -2, 0.0},
        {1, 1, 0, 0, 0, 25.6, -3, 0.0, 0, 0.0},
        {1, -1, 0, -1, 0, -3232.9, -3, 0.0, 0, 0.0},
        {-1, -1, 2, 2, 2, 9.8, -3, 0.0, 1, 0.0},
        {0, -1, 2, 2, 2, 7.2, -3, 0.0, 1, 0.0},
        {1, -1, 2, 0, 2, 9.4, -3, 0.0, 1, 0.0},
        {3, 0, 2, 0, 2, 5.5, -3, 0.0, 1, 0.0},
        {-2, 0, 2, 0, 2, 1615.7, -3, 0.0, 1, 0.0},
        {1, 0, 2, 0, 0, 9.1, 3, 0.0, 0, 0.0},
        {-1, 0, 2, 4, 2, 5.8, -2, 0.0, 1, 0.0},
        {1, 0, 0, 0, 2, 27.8, -2, 0.0, 1, 0.0},
        {-1, 0, 2, -2, 1, -32.6, -2, 0.0, 1, 0.0},
        {0, -2, 2, -2, 1, 6786.3, -2, 0.0, 1, 0.0},
        {-2, 0, 0, 0, 1, -13.7, -2, 0.0, 1, 0.0},
        {2, 0, 0, 0, 1, 13.8, 2, 0.0, -1, 0.0},
        {3, 0, 0, 0, 0, 9.2, 2, 0.0, 0, 0.0},
        {1, 1, 2, 0, 2, 8.9, 2, 0.0, -1, 0.0},
        {0, 0, 2, 1, 2, 9.3, 2, 0.0, -1, 0.0},
        {1, 0, 0, 2, 1, 9.6, -1, 0.0, 0, 0.0},
        {1, 0, 2, 2, 1, 5.6, -1, 0.0, 1, 0.0},
        {1, 1, 0, -2, 1, -34.7, -1, 0.0, 0, 0.0},
        {0, 1, 0, 2, 0, 14.2, -1, 0.0, 0, 0.0},
        {0, 1, 2, -2, 0, 117.5, -1, 0.0, 0, 0.0},
        {0, 1, -2, 2, 0, -329.8, -1, 0.0, 0, 0.0},
        {1, 0, -2, 2, 0, 23.8, -1, 0.0, 0, 0.0},
        {1, 0, -2, -2, 0, -9.5, -1, 0.0, 0, 0.0},
        {1, 0, 2, -2, 0, 32.8, -1, 0.0, 0, 0.0},
        {1, 0, 0, -4, 0, -10.1, -1, 0.0, 0, 0.0},
        {2, 0, 0, -4, 0, -15.9, -1, 0.0, 0, 0.0},
        {0, 0, 2, 4, 2, 4.8, -1, 0.0, 0, 0.0},
        {0, 0, 2, -1, 2, 25.4, -1, 0.0, 0, 0.0},
        {-2, 0, 2, 4, 2, 7.3, -1, 0.0, 1, 0.0},
        {2, 0, 2, 2, 2, 4.7, -1, 0.0, 0, 0.0},
        {0, -1, 2, 0, 1, 14.2, -1, 0.0, 0, 0.0},
        {0, 0, -2, 0, 1, -13.6, -1, 0.0, 0, 0.0},
        {0, 0, 4, -2, 2, 12.7, 1, 0.0, 0, 0.0},
        {0, 1, 0, 0, 2, 409.2, 1, 0.0, 0, 0.0},
        {1, 1, 2, -2, 2, 22.5, 1, 0.0, -1, 0.0},
        {3, 0, 2, -2, 2, 8.7, 1, 0.0, 0, 0.0},
        {-2, 0, 2, 2, 2, 14.6, 1, 0.0, -1, 0.0},
        {-1, 0, 0, 0, 2, -27.3, 1, 0.0, -1, 0.0},
        {0, 0, -2, 2, 1, -169.0, 1, 0.0, 0, 0.0},
        {0, 1, 2, 0, 1, 13.1, 1, 0.0, 0, 0.0},
        {-1, 0, 4, 0, 2, 9.1, 1, 0.0, 0, 0.0},
        {2, 1, 0, -2, 0, 131.7, 1, 0.0, 0, 0.0},
        {2, 0, 0, 2, 0, 7.1, 1, 0.0, 0, 0.0},
        {2, 0, 2, -2, 1, 12.8, 1, 0.0, -1, 0.0},
        {2, 0, -2, 0, 1, -943.2, 1, 0.0, 0, 0.0},
        {1, -1, 0, -2, 0, -29.3, 1, 0.0, 0, 0.0},
        {-1, 0, 0, 1, 1, -388.3, 1, 0.0, 0, 0.0},
        {-1, -1, 0, 2, 1, 35.0, 1, 0.0, 0, 0.0},
        {0, 1, 0, 1, 0, 27.3, 1, 0.0, 0, 0.0}};
    double ang;
    int i, j;

    *dpsi = *deps = 0.0;

    for (i = 0; i < 106; i++)
    {
        ang = 0.0;
        for (j = 0; j < 5; j++)
            ang += nut[i][j] * f[j];
        *dpsi += (nut[i][6] + nut[i][7] * t) * sin(ang);
        *deps += (nut[i][8] + nut[i][9] * t) * cos(ang);
    }
    *dpsi *= 1E-4 * AS2R; /* 0.1 mas -> rad */
    *deps *= 1E-4 * AS2R;
}

/* multiply matrix -----------------------------------------------------------*/
extern void matmul_(const char *tr, int n, int k, int m, double alpha,
                    const double *A, const double *B, double beta, double *C)
{
    double d;
    int i, j, x, f = tr[0] == 'N' ? (tr[1] == 'N' ? 1 : 2) : (tr[1] == 'N' ? 3 : 4);

    for (i = 0; i < n; i++)
        for (j = 0; j < k; j++)
        {
            d = 0.0;
            switch (f)
            {
            case 1:
                for (x = 0; x < m; x++)
                    d += A[i + x * n] * B[x + j * m];
                break;
            case 2:
                for (x = 0; x < m; x++)
                    d += A[i + x * n] * B[j + x * k];
                break;
            case 3:
                for (x = 0; x < m; x++)
                    d += A[x + i * m] * B[x + j * m];
                break;
            case 4:
                for (x = 0; x < m; x++)
                    d += A[x + i * m] * B[j + x * k];
                break;
            }
            if (beta == 0.0)
                C[i + j * n] = alpha * d;
            else
                C[i + j * n] = alpha * d + beta * C[i + j * n];
        }
}

/* ecef to local coordinate transfromation matrix ------------------------------
* compute ecef to local coordinate transfromation matrix
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *E        O   ecef to local coord transformation matrix (3x3)
* return : none
* notes  : matirix stored by column-major order (fortran convention)
*-----------------------------------------------------------------------------*/
extern void xyz2enu_(const double *pos, double *E)
{
    double sinp = sin(pos[0]), cosp = cos(pos[0]), sinl = sin(pos[1]), cosl = cos(pos[1]);

    E[0] = -sinl;
    E[3] = cosl;
    E[6] = 0.0;
    E[1] = -sinp * cosl;
    E[4] = -sinp * sinl;
    E[7] = cosp;
    E[2] = cosp * cosl;
    E[5] = cosp * sinl;
    E[8] = sinp;
}
/* transform ecef vector to local tangental coordinate -------------------------
* transform ecef vector to local tangental coordinate
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *r        I   vector in ecef coordinate {x,y,z}
*          double *e        O   vector in local tangental coordinate {e,n,u}
* return : none
*-----------------------------------------------------------------------------*/
extern void ecef2enu(const double *pos, const double *r, double *e)
{
    double E[9];

    xyz2enu_(pos, E);
    matmul_("NN", 3, 1, 3, 1.0, E, r, 0.0, e);
}

/* time to day of year ---------------------------------------------------------
* convert time to day of year
* args   : gtime_t t        I   gtime_t struct
* return : day of year (days)
*-----------------------------------------------------------------------------*/
extern double time2doy(gtime_t t)
{
    double ep[6];

    time2epoch(t, ep);
    ep[1] = ep[2] = 1.0;
    ep[3] = ep[4] = ep[5] = 0.0;
    return timediff(t, epoch2time(ep)) / 86400.0 + 1.0;
}
/* inner product ---------------------------------------------------------------
* inner product of vectors
* args   : double *a,*b     I   vector a,b (n x 1)
*          int    n         I   size of vector a,b
* return : a'*b
*-----------------------------------------------------------------------------*/
extern double dot_(const double *a, const double *b, int n)
{
    double c = 0.0;

    while (--n >= 0)
        c += a[n] * b[n];
    return c;
}
/* euclid norm -----------------------------------------------------------------
* euclid norm of vector
* args   : double *a        I   vector a (n x 1)
*          int    n         I   size of vector a
* return : || a ||
*-----------------------------------------------------------------------------*/
double norm(const double *a, int n)
{
    return sqrt(dot_(a, a, n));
}
/* outer product of 3d vectors -------------------------------------------------
* outer product of 3d vectors 
* args   : double *a,*b     I   vector a,b (3 x 1)
*          double *c        O   outer product (a x b) (3 x 1)
* return : none
*-----------------------------------------------------------------------------*/
extern void cross3(const double *a, const double *b, double *c)
{
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
}
/* normalize 3d vector ---------------------------------------------------------
* normalize 3d vector
* args   : double *a        I   vector a (3 x 1)
*          double *b        O   normlized vector (3 x 1) || b || = 1
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
extern int normv3(const double *a, double *b)
{
    double r;
    if ((r = norm(a, 3)) <= 0.0)
        return 0;
    b[0] = a[0] / r;
    b[1] = a[1] / r;
    b[2] = a[2] / r;
    return 1;
}

/* ecef to local coordinate transfromation matrix ------------------------------
* compute ecef to local coordinate transfromation matrix
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *E        O   ecef to local coord transformation matrix (3x3)
* return : none
* notes  : matirix stored by column-major order (fortran convention)
*-----------------------------------------------------------------------------*/
static void xyz2enu(const double *pos, double *E)
{
    double sinp = sin(pos[0]), cosp = cos(pos[0]), sinl = sin(pos[1]), cosl = cos(pos[1]);

    E[0] = -sinl;
    E[3] = cosl;
    E[6] = 0.0;
    E[1] = -sinp * cosl;
    E[4] = -sinp * sinl;
    E[7] = cosp;
    E[2] = cosp * cosl;
    E[5] = cosp * sinl;
    E[8] = sinp;
}

/* transform local enu coordinate covariance to xyz-ecef -----------------------
* transform local enu covariance to xyz-ecef coordinate
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *Q        I   covariance in local enu coordinate
*          double *P        O   covariance in xyz-ecef coordinate
* return : none
*-----------------------------------------------------------------------------*/
extern void covecef(const double *pos, const double *Q, double *P)
{
    double E[9], EQ[9];

    xyz2enu(pos, E);
    matmul("TN", 3, 3, 3, 1.0, E, Q, 0.0, EQ);
    matmul("NN", 3, 3, 3, 1.0, EQ, E, 0.0, P);
}

/* time to day and sec -------------------------------------------------------*/
static double time2sec(gtime_t time, gtime_t *day)
{
    double ep[6], sec;
    time2epoch(time, ep);
    sec = ep[3] * 3600.0 + ep[4] * 60.0 + ep[5];
    ep[3] = ep[4] = ep[5] = 0.0;
    *day = epoch2time(ep);
    return sec;
}

/* utc to gmst -----------------------------------------------------------------
* convert utc to gmst (Greenwich mean sidereal time)
* args   : gtime_t t        I   time expressed in utc
*          double ut1_utc   I   UT1-UTC (s)
* return : gmst (rad)
*-----------------------------------------------------------------------------*/
extern double utc2gmst(gtime_t t, double ut1_utc)
{
    const double ep2000[] = {2000, 1, 1, 12, 0, 0};
    gtime_t tut, tut0;
    double ut, t1, t2, t3, gmst0, gmst;

    tut = timeadd(t, ut1_utc);
    ut = time2sec(tut, &tut0);
    t1 = timediff(tut0, epoch2time(ep2000)) / 86400.0 / 36525.0;
    t2 = t1 * t1;
    t3 = t2 * t1;
    gmst0 = 24110.54841 + 8640184.812866 * t1 + 0.093104 * t2 - 6.2E-6 * t3;
    gmst = gmst0 + 1.002737909350795 * ut;

    return fmod(gmst, 86400.0) * PI / 43200.0; /* 0 <= gmst <= 2*PI */
}

double satazel(const double *pos, const double *e, double *azel)
{
    double az = 0.0, el = PI / 2.0, enu[3];

    ecef2enu(pos, e, enu);
    az = dot_(enu, enu, 2) < 1E-12 ? 0.0 : atan2(enu[0], enu[1]);
    if (az < 0.0)
        az += 2 * PI;
    el = asin(enu[2]);

    azel[0] = az;
    azel[1] = el;

    return el;
}

double geodist(const double *rs, const double *rr, double *e)
{
    double r;
    int i;

    for (i = 0; i < 3; i++)
        e[i] = rs[i] - rr[i];
    r = norm(e, 3);
    for (i = 0; i < 3; i++)
        e[i] /= r;
    return r + OMGE * (rs[0] * rr[1] - rs[1] * rr[0]) / CLIGHT;
}
/* geometric distance ----------------------------------------------------------
* compute geometric distance and receiver-to-satellite unit vector
* args   : double *rs       I   satellilte position and vel(ecef at transmission) (m)
*          double *rv       I   receiver velocity (ecef at reception) (m)
*          double *e        O   line-of-sight vector (ecef)
* return : geometric distance (m) (0>:error/no satellite position)
* notes  : distance includes sagnac effect correction
*-----------------------------------------------------------------------------*/
double geovel(const double *rs, const double *rv, double *e)
{
    double rate = 0.0;
    int i;
    double vs[3] = {0.0};

    for (i = 0; i < 3; i++)
    {
        vs[i] = rs[3 + i] - rv[i];
    }
    rate = dot_(vs, e, 3);
    rate += OMGE / CLIGHT * (rs[4] * rv[0] + rs[1] * rv[0] - rs[3] * rv[1] - rs[0] * rv[1]);
    return rate;
}
static double tropmodel(const double *blh, const double *azel, double humi)
{
    const double temp0 = 15.0; /* temparature at sea level */
    double hgt, pres, temp, e, z, trph, trpw;

    if (blh[2] < -100.0 || 1E4 < blh[2] || azel[1] <= 0)
        return 0.0;

    /* standard atmosphere */
    hgt = blh[2] < 0.0 ? 0.0 : blh[2];

    pres = 1013.25 * pow(1.0 - 2.2557E-5 * hgt, 5.2568);
    temp = temp0 - 6.5E-3 * hgt + 273.16;
    e = 6.108 * humi * exp((17.15 * temp - 4684.0) / (temp - 38.45));

    /* saastamoninen model */
    z = PI / 2.0 - azel[1];
    trph = 0.0022768 * pres / (1.0 - 0.00266 * cos(2.0 * blh[0]) - 0.00028 * hgt / 1E3) / cos(z);
    trpw = 0.002277 * (1255.0 / temp + 0.05) * e / cos(z);
    return trph + trpw;
}

int compute_vector_data(obs_t *obs, vec_t *vec)
{
    int i;
    int n = 0;
    vec_t *vecd = NULL;
    double blh[3] = {0.0};

    ecef2pos(obs->pos, blh);

    for (i = 0; i < obs->n; ++i)
    {
        vecd = vec + i;
        if (norm(vecd->satpvt, 3) < 0.01)
            continue;
        /* compute geometric-range and azimuth/elevation angle */
        vecd->r = geodist(vecd->satpvt, obs->pos, vecd->e);
        vecd->rate = geovel(vecd->satpvt, obs->pos + 3, vecd->e);

        satazel(blh, vecd->e, vecd->azel);
        /* satellite clock-bias */

        vecd->tro = tropmodel(blh, vecd->azel, 0.7);

        ++n;
    }
    return n;
}

static int get_match_epoch(obs_t *obs_ref, obs_t *obs_rov, vec_t *vec_ref, vec_t *vec_rov, int *iref, int *irov)
{
    /* get the matched satellite index from the base and rover recivers, and sort by the elevation */
    int n = 0, i = 0, j = 0;
    double elev[MAXOBS] = {0.0};
    obsd_t *pObsRov = NULL;
    obsd_t *pObsRef = NULL;
    vec_t *pVecRov = NULL;
    vec_t *pVecRef = NULL;
    for (i = 0; i < obs_rov->n; ++i)
    {
        pObsRov = obs_rov->data + i;
        pVecRov = vec_rov + i;
        if (norm(pVecRov->satpvt, 3) < 0.1)
            continue;
        for (j = 0; j < obs_ref->n; ++j)
        {
            pObsRef = obs_ref->data + j;
            pVecRef = vec_ref + j;
            if (norm(pVecRef->satpvt, 3) < 0.1)
                continue;
            if (pObsRov->sat != pObsRef->sat)
                continue;
            iref[n] = j;
            irov[n] = i;
            elev[n] = pObsRov->SNR[0]; // pVecRef->azel[1];
            ++n;
        }
    }
    for (i = 0; i < n; ++i)
    {
        for (j = i + 1; j < n; ++j)
        {
            if (elev[i] < elev[j])
            {
                int temp1 = iref[i];
                int temp2 = irov[i];
                double v = elev[i];
                iref[i] = iref[j];
                irov[i] = irov[j];
                elev[i] = elev[j];
                iref[j] = temp1;
                irov[j] = temp2;
                elev[j] = v;
            }
        }
    }
    return n;
}

#ifdef _USE_PPP_
static int get_match_ssr(obs_t *obs_rov, vec_t *vec_rov, nav_t *nav)
{
    int n = 0, i = 0, j = 0;
    obsd_t *pObsRov = NULL;
    vec_t *pVecRov = NULL;
    ssr_t *pssr = NULL;

    for (i = 0; i < obs_rov->n; ++i)
    {
        pObsRov = obs_rov->data + i;
        pVecRov = vec_rov + i;

        if (norm(pVecRov->satpvt, 3) < 0.1)
            continue;
        for (j = 0; j < nav->ns; ++j)
        {
            pssr = nav->ssr + j;
            if (pssr->sat == pObsRov->sat)
            {
                n++;
                break;
            }
        }
    }

    return n;
}
#endif

/* single-differenced measurement error variance -----------------------------*/
static void sdvarerr(int sat, int sys, double el, double *codeCOV, double *phaseCOV)
{
    double a = 0.003, b = 0.003, fact = 150.0, sys_scale = 1.0;
    double sinel = sin(el);
    if (sys == _SYS_GLO_)
        sys_scale = 5.0;
    if (sys == _SYS_BDS_)
        sys_scale = 1.5;
    if (sys == _SYS_GAL_)
        sys_scale = 1.5;
    *phaseCOV = 2.0 * (a * a + b * b / sinel / sinel) * sys_scale * sys_scale;
    *codeCOV = fact * fact * (*phaseCOV);
    return;
}

/* measurement error variance ------------------------------------------------*/
double varerr(int sat, int sys, double el, int freq, int type)
{
    prcopt_t *opt = &prcopt;
    double fact = 1.0, sinel = sin(el);

    if (type == 1)
        fact *= opt->eratio[freq == 0 ? 0 : 1];
    fact *= sys == _SYS_GLO_ ? EFACT_GLO : EFACT_GPS;

    if (sys == _SYS_GPS_ || sys == _SYS_QZS_)
    {
        if (freq == 2)
            fact *= EFACT_GPS_L5; /* GPS/QZS L5 error factor */
    }
    //if (opt->ionoopt == IONOOPT_IF12) fact *= 3.0;

    return SQR(fact * opt->err[1]) + SQR(fact * opt->err[2] / sinel);
}

/* ionosphere model ------------------------------------------------------------
* compute ionospheric delay by broadcast ionosphere model (klobuchar model)
* args   : gtime_t t        I   time (gpst)
*          double *ion      I   iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3}
*          double *pos      I   receiver position {lat,lon,h} (rad,m)
*          double *azel     I   azimuth/elevation angle {az,el} (rad)
* return : ionospheric delay (L1) (m)
*-----------------------------------------------------------------------------*/
double ionmodel(gtime_t t, const double *ion, const double *pos,
                const double *azel)
{
    const double ion_default[] = {/* 2004/1/1 */
                                  0.1118E-07, -0.7451E-08, -0.5961E-07, 0.1192E-06,
                                  0.1167E+06, -0.2294E+06, -0.1311E+06, 0.1049E+07};
    double tt, f, psi, phi, lam, amp, per, x;
    int week;

    if (pos[2] < -1E3 || azel[1] <= 0)
        return 0.0;
    if (norm(ion, 8) <= 0.0)
        ion = ion_default;

    /* earth centered angle (semi-circle) */
    psi = 0.0137 / (azel[1] / PI + 0.11) - 0.022;

    /* subionospheric latitude/longitude (semi-circle) */
    phi = pos[0] / PI + psi * cos(azel[0]);
    if (phi > 0.416)
        phi = 0.416;
    else if (phi < -0.416)
        phi = -0.416;
    lam = pos[1] / PI + psi * sin(azel[0]) / cos(phi * PI);

    /* geomagnetic latitude (semi-circle) */
    phi += 0.064 * cos((lam - 1.617) * PI);

    /* local time (s) */
    tt = 43200.0 * lam + time2gpst(t, &week);
    tt -= floor(tt / 86400.0) * 86400.0; /* 0<=tt<86400 */

    /* slant factor */
    f = 1.0 + 16.0 * pow(0.53 - azel[1] / PI, 3.0);

    /* ionospheric delay */
    amp = ion[0] + phi * (ion[1] + phi * (ion[2] + phi * ion[3]));
    per = ion[4] + phi * (ion[5] + phi * (ion[6] + phi * ion[7]));
    amp = amp < 0.0 ? 0.0 : amp;
    per = per < 72000.0 ? 72000.0 : per;
    x = 2.0 * PI * (tt - 50400.0) / per;

    return CLIGHT * f * (fabs(x) < 1.57 ? 5E-9 + amp * (1.0 + x * x * (-0.5 + x * x / 24.0)) : 5E-9);
}
/* ionosphere mapping function -------------------------------------------------
* compute ionospheric delay mapping function by single layer model
* args   : double *pos      I   receiver position {lat,lon,h} (rad,m)
*          double *azel     I   azimuth/elevation angle {az,el} (rad)
* return : ionospheric mapping function
*-----------------------------------------------------------------------------*/
double ionmapf(const double *pos, const double *azel)
{
    if (pos[2] >= HION)
        return 1.0;
    return 1.0 / cos(asin((RE_WGS84 + pos[2]) / (RE_WGS84 + HION) * sin(PI / 2.0 - azel[1])));
}
/* ionospheric pierce point position -------------------------------------------
* compute ionospheric pierce point (ipp) position and slant factor
* args   : double *pos      I   receiver position {lat,lon,h} (rad,m)
*          double *azel     I   azimuth/elevation angle {az,el} (rad)
*          double re        I   earth radius (km)
*          double hion      I   altitude of ionosphere (km)
*          double *posp     O   pierce point position {lat,lon,h} (rad,m)
* return : slant factor
* notes  : see ref [2], only valid on the earth surface
*          fixing bug on ref [2] A.4.4.10.1 A-22,23
*-----------------------------------------------------------------------------*/
double ionppp(const double *pos, const double *azel, double re,
              double hion, double *posp)
{
    double cosaz, rp, ap, sinap, tanap;
    rp = re / (re + hion) * cos(azel[1]);
    ap = PI / 2.0 - azel[1] - asin(rp);
    sinap = sin(ap);
    tanap = tan(ap);
    cosaz = cos(azel[0]);
    posp[0] = asin(sin(pos[0]) * cos(ap) + cos(pos[0]) * sinap * cosaz);

    if ((pos[0] > 70.0 * D2R && tanap * cosaz > tan(PI / 2.0 - pos[0])) ||
        (pos[0] < -70.0 * D2R && -tanap * cosaz > tan(PI / 2.0 + pos[0])))
    {
        posp[1] = pos[1] + PI - asin(sinap * sin(azel[0]) / cos(posp[0]));
    }
    else
    {
        posp[1] = pos[1] + asin(sinap * sin(azel[0]) / cos(posp[0]));
    }
    return 1.0 / sqrt(1.0 - rp * rp);
}

double interpc(const double coef[], double lat)
{
    int i = (int)(lat / 15.0);
    if (i < 1)
        return coef[0];
    else if (i > 4)
        return coef[4];
    return coef[i - 1] * (1.0 - lat / 15.0 + i) + coef[i] * (lat / 15.0 - i);
}

double mapf(double el, double a, double b, double c)
{
    double sinel = sin(el);
    return (1.0 + a / (1.0 + b / (1.0 + c))) / (sinel + (a / (sinel + b / (sinel + c))));
}

double nmf(gtime_t time, const double pos[], const double azel[],
           double *mapfw)
{
    /* ref [5] table 3 */
    /* hydro-ave-a,b,c, hydro-amp-a,b,c, wet-a,b,c at latitude 15,30,45,60,75 */
    const double coef[][5] = {
        {1.2769934E-3, 1.2683230E-3, 1.2465397E-3, 1.2196049E-3, 1.2045996E-3},
        {2.9153695E-3, 2.9152299E-3, 2.9288445E-3, 2.9022565E-3, 2.9024912E-3},
        {62.610505E-3, 62.837393E-3, 63.721774E-3, 63.824265E-3, 64.258455E-3},

        {0.0000000E-0, 1.2709626E-5, 2.6523662E-5, 3.4000452E-5, 4.1202191E-5},
        {0.0000000E-0, 2.1414979E-5, 3.0160779E-5, 7.2562722E-5, 11.723375E-5},
        {0.0000000E-0, 9.0128400E-5, 4.3497037E-5, 84.795348E-5, 170.37206E-5},

        {5.8021897E-4, 5.6794847E-4, 5.8118019E-4, 5.9727542E-4, 6.1641693E-4},
        {1.4275268E-3, 1.5138625E-3, 1.4572752E-3, 1.5007428E-3, 1.7599082E-3},
        {4.3472961E-2, 4.6729510E-2, 4.3908931E-2, 4.4626982E-2, 5.4736038E-2}};
    const double aht[] = {2.53E-5, 5.49E-3, 1.14E-3}; /* height correction */

    double y, cosy, ah[3], aw[3], dm, el = azel[1], lat = pos[0] * R2D, hgt = pos[2];
    int i;

    if (el <= 0.0)
    {
        if (mapfw)
            *mapfw = 0.0;
        return 0.0;
    }
    /* year from doy 28, added half a year for southern latitudes */
    y = (time2doy(time) - 28.0) / 365.25 + (lat < 0.0 ? 0.5 : 0.0);

    cosy = cos(2.0 * PI * y);
    lat = fabs(lat);

    for (i = 0; i < 3; i++)
    {
        ah[i] = interpc(coef[i], lat) - interpc(coef[i + 3], lat) * cosy;
        aw[i] = interpc(coef[i + 6], lat);
    }
    /* ellipsoidal height is used instead of height above sea level */
    dm = (1.0 / sin(el) - mapf(el, aht[0], aht[1], aht[2])) * hgt / 1E3;

    if (mapfw)
        *mapfw = mapf(el, aw[0], aw[1], aw[2]);

    return mapf(el, ah[0], ah[1], ah[2]) + dm;
}

/* troposphere mapping function ------------------------------------------------
* compute tropospheric mapping function by NMF
* args   : gtime_t t        I   time
*          double *pos      I   receiver position {lat,lon,h} (rad,m)
*          double *azel     I   azimuth/elevation angle {az,el} (rad)
*          double *mapfw    IO  wet mapping function (NULL: not output)
* return : dry mapping function
* note   : see ref [5] (NMF) and [9] (GMF)
*          original JGR paper of [5] has bugs in eq.(4) and (5). the corrected
*          paper is obtained from:
*          ftp://web.haystack.edu/pub/aen/nmf/NMF_JGR.pdf
*-----------------------------------------------------------------------------*/
double tropmapf(gtime_t time, const double pos[], const double azel[],
                double *mapfw)
{
#ifdef IERS_MODEL
    const double ep[] = {2000, 1, 1, 12, 0, 0};
    double mjd, lat, lon, hgt, zd, gmfh, gmfw;
#endif
    // trace(4, "tropmapf: pos=%10.6f %11.6f %6.1f azel=%5.1f %4.1f\n",
    //     pos[0] * R2D, pos[1] * R2D, pos[2], azel[0] * R2D, azel[1] * R2D);

    if (pos[2] < -1000.0 || pos[2] > 20000.0)
    {
        if (mapfw)
            *mapfw = 0.0;
        return 0.0;
    }
#ifdef IERS_MODEL
    mjd = 51544.5 + (timediff(time, epoch2time(ep))) / 86400.0;
    lat = pos[0];
    lon = pos[1];
    hgt = pos[2] - geoidh(pos); /* height in m (mean sea level) */
    zd = PI / 2.0 - azel[1];

    /* call GMF */
    gmf_(&mjd, &lat, &lon, &hgt, &zd, &gmfh, &gmfw);

    if (mapfw)
        *mapfw = gmfw;
    return gmfh;
#else
    return nmf(time, pos, azel, mapfw); /* NMF */
#endif
}

/* receiver antenna model ------------------------------------------------------
* compute antenna offset by antenna phase center parameters
* args   : pcv_t *pcv       I   antenna phase center parameters
*          double *azel     I   azimuth/elevation for receiver {az,el} (rad)
*          int     opt      I   option (0:only offset,1:offset+pcv)
*          double *dant     O   range offsets for each frequency (m)
* return : none
* notes  : current version does not support azimuth dependent terms
*-----------------------------------------------------------------------------*/
void antmodel(const double *azel, double *dant)
{
    double e[3], off[3], cosel = cos(azel[1]);
    int i, j;

    prcopt_t *opt = &prcopt;

    e[0] = sin(azel[0]) * cosel;
    e[1] = cos(azel[0]) * cosel;
    e[2] = sin(azel[1]);

    for (i = 0; i < NFREQ; i++)
    {
        for (j = 0; j < 3; j++)
            off[j] = opt->pco[i][j] + opt->antdel[j];

        dant[i] = -(e[0] * off[0] + e[1] * off[1] + e[2] * off[2]);
    }
}

/* astronomical arguments: f={l,l',F,D,OMG} (rad) ----------------------------*/
static void ast_args(double t, double *f)
{
    static const double fc[][5] = {/* coefficients for iau 1980 nutation */
                                   {134.96340251, 1717915923.2178, 31.8792, 0.051635, -0.00024470},
                                   {357.52910918, 129596581.0481, -0.5532, 0.000136, -0.00001149},
                                   {93.27209062, 1739527262.8478, -12.7512, -0.001037, 0.00000417},
                                   {297.85019547, 1602961601.2090, -6.3706, 0.006593, -0.00003169},
                                   {125.04455501, -6962890.2665, 7.4722, 0.007702, -0.00005939}};
    double tt[4];
    int i, j;

    for (tt[0] = t, i = 1; i < 4; i++)
        tt[i] = tt[i - 1] * t;
    for (i = 0; i < 5; i++)
    {
        f[i] = fc[i][0] * 3600.0;
        for (j = 0; j < 4; j++)
            f[i] += fc[i][j + 1] * tt[j];
        f[i] = fmod(f[i] * AS2R, 2.0 * PI);
    }
}

/* eci to ecef transformation matrix -------------------------------------------
* compute eci to ecef transformation matrix
* args   : gtime_t tutc     I   time in utc
*          double *erpv     I   erp values {xp,yp,ut1_utc,lod} (rad,rad,s,s/d)
*          double *U        O   eci to ecef transformation matrix (3 x 3)
*          double *gmst     IO  greenwich mean sidereal time (rad)
*                               (NULL: no output)
* return : none
* note   : see ref [3] chap 5
*          not thread-safe
*-----------------------------------------------------------------------------*/
void eci2ecef(gtime_t tutc, const double *erpv, double *U, double *gmst)
{
    const double ep2000[] = {2000, 1, 1, 12, 0, 0};
    static gtime_t tutc_;
    static double U_[9], gmst_;
    gtime_t tgps;
    double eps, ze, th, z, t, t2, t3, dpsi, deps, gast, f[5];
    double R1[9], R2[9], R3[9], R[9], W[9], N[9], P[9], NP[9];
    int i;

    if (fabs(timediff(tutc, tutc_)) < 0.01)
    { /* read cache */
        for (i = 0; i < 9; i++)
            U[i] = U_[i];
        if (gmst)
            *gmst = gmst_;
        return;
    }
    tutc_ = tutc;

    /* terrestrial time */
    tgps = utc2gpst(tutc_);
    t = (timediff(tgps, epoch2time(ep2000)) + 19.0 + 32.184) / 86400.0 / 36525.0;
    t2 = t * t;
    t3 = t2 * t;

    /* astronomical arguments */
    ast_args(t, f);

    /* iau 1976 precession */
    ze = (2306.2181 * t + 0.30188 * t2 + 0.017998 * t3) * AS2R;
    th = (2004.3109 * t - 0.42665 * t2 - 0.041833 * t3) * AS2R;
    z = (2306.2181 * t + 1.09468 * t2 + 0.018203 * t3) * AS2R;
    eps = (84381.448 - 46.8150 * t - 0.00059 * t2 + 0.001813 * t3) * AS2R;
    Rz(-z, R1);
    Ry(th, R2);
    Rz(-ze, R3);
    matmul("NN", 3, 3, 3, 1.0, R1, R2, 0.0, R);
    matmul("NN", 3, 3, 3, 1.0, R, R3, 0.0, P); /* P=Rz(-z)*Ry(th)*Rz(-ze) */

    /* iau 1980 nutation */
    nut_iau1980(t, f, &dpsi, &deps);
    Rx(-eps - deps, R1);
    Rz(-dpsi, R2);
    Rx(eps, R3);
    matmul("NN", 3, 3, 3, 1.0, R1, R2, 0.0, R);
    matmul("NN", 3, 3, 3, 1.0, R, R3, 0.0, N); /* N=Rx(-eps)*Rz(-dspi)*Rx(eps) */

    /* greenwich aparent sidereal time (rad) */
    gmst_ = utc2gmst(tutc_, erpv[2]);
    gast = gmst_ + dpsi * cos(eps);
    gast += (0.00264 * sin(f[4]) + 0.000063 * sin(2.0 * f[4])) * AS2R;

    /* eci to ecef transformation matrix */
    Ry(-erpv[0], R1);
    Rx(-erpv[1], R2);
    Rz(gast, R3);
    matmul("NN", 3, 3, 3, 1.0, R1, R2, 0.0, W);
    matmul("NN", 3, 3, 3, 1.0, W, R3, 0.0, R); /* W=Ry(-xp)*Rx(-yp) */
    matmul("NN", 3, 3, 3, 1.0, N, P, 0.0, NP);
    matmul("NN", 3, 3, 3, 1.0, R, NP, 0.0, U_); /* U=W*Rz(gast)*N*P */

    for (i = 0; i < 9; i++)
        U[i] = U_[i];
    if (gmst)
        *gmst = gmst_;
}

/* sun and moon position in eci (ref [4] 5.1.1, 5.2.1) -----------------------*/
void sunmoonpos_eci(gtime_t tut, double *rsun, double *rmoon)
{
    const double ep2000[] = {2000, 1, 1, 12, 0, 0};
    double t, f[5], eps, Ms, ls, rs, lm, pm, rm, sine, cose, sinp, cosp, sinl, cosl;

    t = timediff(tut, epoch2time(ep2000)) / 86400.0 / 36525.0;

    /* astronomical arguments */
    ast_args(t, f);

    /* obliquity of the ecliptic */
    eps = 23.439291 - 0.0130042 * t;
    sine = sin(eps * D2R);
    cose = cos(eps * D2R);

    /* sun position in eci */
    if (rsun)
    {
        Ms = 357.5277233 + 35999.05034 * t;
        ls = 280.460 + 36000.770 * t + 1.914666471 * sin(Ms * D2R) + 0.019994643 * sin(2.0 * Ms * D2R);
        rs = AU * (1.000140612 - 0.016708617 * cos(Ms * D2R) - 0.000139589 * cos(2.0 * Ms * D2R));
        sinl = sin(ls * D2R);
        cosl = cos(ls * D2R);
        rsun[0] = rs * cosl;
        rsun[1] = rs * cose * sinl;
        rsun[2] = rs * sine * sinl;
    }
    /* moon position in eci */
    if (rmoon)
    {
        lm = 218.32 + 481267.883 * t + 6.29 * sin(f[0]) - 1.27 * sin(f[0] - 2.0 * f[3]) +
             0.66 * sin(2.0 * f[3]) + 0.21 * sin(2.0 * f[0]) - 0.19 * sin(f[1]) - 0.11 * sin(2.0 * f[2]);
        pm = 5.13 * sin(f[2]) + 0.28 * sin(f[0] + f[2]) - 0.28 * sin(f[2] - f[0]) -
             0.17 * sin(f[2] - 2.0 * f[3]);
        rm = RE_WGS84 / sin((0.9508 + 0.0518 * cos(f[0]) + 0.0095 * cos(f[0] - 2.0 * f[3]) +
                             0.0078 * cos(2.0 * f[3]) + 0.0028 * cos(2.0 * f[0])) *
                            D2R);
        sinl = sin(lm * D2R);
        cosl = cos(lm * D2R);
        sinp = sin(pm * D2R);
        cosp = cos(pm * D2R);
        rmoon[0] = rm * cosp * cosl;
        rmoon[1] = rm * (cose * cosp * sinl - sine * sinp);
        rmoon[2] = rm * (sine * cosp * sinl + cose * sinp);
    }
}

/* sun and moon position -------------------------------------------------------
* get sun and moon position in ecef
* args   : gtime_t tut      I   time in ut1
*          double *erpv     I   erp value {xp,yp,ut1_utc,lod} (rad,rad,s,s/d)
*          double *rsun     IO  sun position in ecef  (m) (NULL: not output)
*          double *rmoon    IO  moon position in ecef (m) (NULL: not output)
*          double *gmst     O   gmst (rad)
* return : none
*-----------------------------------------------------------------------------*/
void sunmoonpos(gtime_t tutc, const double *erpv, double *rsun,
                double *rmoon, double *gmst)
{
    gtime_t tut;
    double rs[3], rm[3], U[9], gmst_;

    // trace(4, "sunmoonpos: tutc=%s\n", time_str(tutc, 3));

    tut = timeadd(tutc, erpv[2]); /* utc -> ut1 */

    /* sun and moon position in eci */
    sunmoonpos_eci(tut, rsun ? rs : NULL, rmoon ? rm : NULL);

    /* eci to ecef transformation matrix */
    eci2ecef(tutc, erpv, U, &gmst_);

    /* sun and moon postion in ecef */
    if (rsun)
        matmul("NN", 3, 1, 3, 1.0, U, rs, 0.0, rsun);
    if (rmoon)
        matmul("NN", 3, 1, 3, 1.0, U, rm, 0.0, rmoon);
    if (gmst)
        *gmst = gmst_;
}

/* nominal yaw-angle ---------------------------------------------------------*/
double yaw_nominal(double beta, double mu)
{
    if (fabs(beta) < 1E-12 && fabs(mu) < 1E-12)
        return PI;
    return atan2(-tan(beta), sin(mu)) + PI;
}
/* yaw-angle of satellite ----------------------------------------------------*/
int yaw_angle(int sat, int opt, double beta, double mu, double *yaw)
{
    *yaw = yaw_nominal(beta, mu);
    return 1;
}

/* satellite attitude model --------------------------------------------------*/
int sat_yaw(gtime_t time, int sat, int opt,
            const double *rs, double *exs, double *eys)
{
    double rsun[3], ri[6], es[3], esun[3], n[3], p[3], en[3], ep[3], ex[3], E, beta, mu;
    double yaw, cosy, siny, erpv[5] = {0};
    int i;

    sunmoonpos(gpst2utc(time), erpv, rsun, NULL, NULL);

    /* beta and orbit angle */
    matcpy(ri, rs, 6, 1);
    ri[3] -= OMGE * ri[1];
    ri[4] += OMGE * ri[0];
    cross3(ri, ri + 3, n);
    cross3(rsun, n, p);
    if (!normv3(rs, es) || !normv3(rsun, esun) || !normv3(n, en) ||
        !normv3(p, ep))
        return 0;
    beta = PI / 2.0 - acos(dot(esun, en, 3));
    E = acos(dot(es, ep, 3));
    mu = PI / 2.0 + (dot(es, esun, 3) <= 0 ? -E : E);
    if (mu < -PI / 2.0)
        mu += 2.0 * PI;
    else if (mu >= PI / 2.0)
        mu -= 2.0 * PI;

    /* yaw-angle of satellite */
    if (!yaw_angle(sat, opt, beta, mu, &yaw))
        return 0;

    /* satellite fixed x,y-vector */
    cross3(en, es, ex);
    cosy = cos(yaw);
    siny = sin(yaw);
    for (i = 0; i < 3; i++)
    {
        exs[i] = -siny * en[i] + cosy * ex[i];
        eys[i] = -cosy * en[i] - siny * ex[i];
    }
    return 1;
}

/* phase windup model --------------------------------------------------------*/
int model_phw(gtime_t time, int sat, int opt,
              const double *rs, const double *rr, double *phw)
{
    double exs[3], eys[3], ek[3], exr[3], eyr[3], eks[3], ekr[3], E[9];
    double dr[3], ds[3], drs[3], r[3], pos[3], cosp, ph;
    int i;

    if (opt <= 0)
        return 1; /* no phase windup */

    /* satellite yaw attitude model */
    if (!sat_yaw(time, sat, opt, rs, exs, eys))
        return 0;

    /* unit vector satellite to receiver */
    for (i = 0; i < 3; i++)
        r[i] = rr[i] - rs[i];
    if (!normv3(r, ek))
        return 0;

    /* unit vectors of receiver antenna */
    ecef2pos(rr, pos);
    xyz2enu(pos, E);
    exr[0] = E[1];
    exr[1] = E[4];
    exr[2] = E[7]; /* x = north */
    eyr[0] = -E[0];
    eyr[1] = -E[3];
    eyr[2] = -E[6]; /* y = west  */

    /* phase windup effect */
    cross3(ek, eys, eks);
    cross3(ek, eyr, ekr);
    for (i = 0; i < 3; i++)
    {
        ds[i] = exs[i] - ek[i] * dot(ek, exs, 3) - eks[i];
        dr[i] = exr[i] - ek[i] * dot(ek, exr, 3) + ekr[i];
    }
    cosp = dot(ds, dr, 3) / norm(ds, 3) / norm(dr, 3);
    if (cosp < -1.0)
        cosp = -1.0;
    else if (cosp > 1.0)
        cosp = 1.0;
    ph = acos(cosp) / 2.0 / PI;
    cross3(ds, dr, drs);
    if (dot(ek, drs, 3) < 0.0)
        ph = -ph;

    *phw = ph + floor(*phw - ph + 0.5); /* in cycle */
    return 1;
}

/* initialize state and covariance -------------------------------------------*/
void initx(rcv_ppp_t *rcv, double xi, double var, int i)
{
    rcv->x[i] = xi;

    rcv->P[SMI(i, i)] = var;
}

/* geometry-free phase measurement -------------------------------------------*/
extern double gfmeas(const obsd_t *obs)
{
    double lam[2];
    lam[0] = satwavelen(obs->sat, 0);
    lam[1] = satwavelen(obs->sat, 1);

    int i = 1;

    if (lam[0] == 0.0 || lam[i] == 0.0 || obs->L[0] == 0.0 || obs->L[i] == 0.0)
        return 0.0;
    return lam[0] * obs->L[0] - lam[i] * obs->L[i];
}
/* Melbourne-Wubbena linear combination --------------------------------------*/
double mwmeas(const obsd_t *obs)
{
    double lam[2];
    lam[0] = satwavelen(obs->sat, 0);
    lam[1] = satwavelen(obs->sat, 1);
    int i = 1; //(satsys(obs->sat, NULL)&_SYS_GAL_) ? 2 :

    if (lam[0] == 0.0 || lam[i] == 0.0 || obs->L[0] == 0.0 || obs->L[i] == 0.0 ||
        obs->P[0] == 0.0 || obs->P[i] == 0.0)
        return 0.0;
    return lam[0] * lam[i] * (obs->L[0] - obs->L[i]) / (lam[i] - lam[0]) -
           (lam[i] * obs->P[0] + lam[0] * obs->P[i]) / (lam[i] + lam[0]);
}

/* antenna corrected measurements --------------------------------------------*/
void corr_meas(const obsd_t *obs, const double *azel,
               const double *dantr, double phw, double *L, double *P, double *Lc, double *Pc)
{
    double lam[2];
    double C1, C2;
    int i, sys;
    prcopt_t *opt = &prcopt;
    lam[0] = satwavelen(obs->sat, 0);
    lam[1] = satwavelen(obs->sat, 1);
    for (i = 0; i < NFREQ; i++)
    {
        L[i] = P[i] = 0.0;

        if (lam[i] == 0.0 || obs->L[i] == 0.0 || obs->P[i] == 0.0)
            continue;

        /* antenna phase center and phase windup correction */
        L[i] = obs->L[i] * lam[i] - dantr[i] - phw * lam[i];
        P[i] = obs->P[i] - dantr[i];

        /* P1-C1,P2-C2 dcb correction (C1->P1,C2->P2) */
        if (obs->code[i] == CODE_L1C)
        {
            P[i] += 0.0; /*nav->cbias[obs->sat-1][1];*/
        }
        else if (obs->code[i] == CODE_L2C || obs->code[i] == CODE_L2X ||
                 obs->code[i] == CODE_L2L || obs->code[i] == CODE_L2S)
        {
            P[i] += 0.0; /*nav->cbias[obs->sat-1][2];*/
        }
    }
    /* iono-free LC */
    *Lc = *Pc = 0.0;
    sys = satsys(obs->sat, NULL);
    i = (sys & _SYS_GAL_) ? 2 : 1; /* L1/L2 or L1/L5 */
    if (lam[0] == 0.0 || lam[i] == 0.0)
        return;

    C1 = SQR(lam[i]) / (SQR(lam[i]) - SQR(lam[0]));
    C2 = -SQR(lam[0]) / (SQR(lam[i]) - SQR(lam[0]));

    if (L[0] != 0.0 && L[i] != 0.0)
        *Lc = C1 * L[0] + C2 * L[i];
    if (P[0] != 0.0 && P[i] != 0.0)
        *Pc = C1 * P[0] + C2 * P[i];
}

static int find_sat_index(int sat)
{
    int i = 0;
    int satid = -1;
    int ns = -1;
    int nr = -1;
    for (i = 0; i < MAXOBS; ++i)
    {
        if (ssat[i].sat == 0)
        {
            ns = i;
            break;
        }

        if (ssat[i].sat == sat)
        {
            satid = i;
            break;
        }

        if (satsys(ssat[i].sat, NULL) == _SYS_GLO_)
            nr = i;
    }

    if (ns != -1)
        satid = ns;

    if (sat < MAXPRNGPS && ns == -1 && satid == -1)
    {
        satid = nr;
#ifdef _USE_PPP_
        ssat[nr].mw = 0.0;
        ssat[nr].gf = 0.0;
#else
        for (int f = 0; f < NFREQ; f++)
        {
            ssat[nr].gf = 0.0;
            ssat[nr].D[f] = 0.0;
            ssat[nr].L[f] = 0.0;
            ssat[nr].dph[f] = 0.0;
        }
#endif
    }

    return satid;
}

/* detect cycle slip by LLI --------------------------------------------------*/
static void detslp_ll(const obs_t *obs)
{
    int i, j, satid;

    prcopt_t *opt = &prcopt;
    obsd_t *pObs = NULL;

    for (i = 0; i < obs->n && i < MAXOBS; i++)
        for (j = 0; j < opt->nf; j++)
        {
            pObs = obs->data + i;
            if (pObs->L[j] == 0.0 || !(pObs->LLI[j] & 3))
                continue;

            satid = find_sat_index(pObs->sat);

            if (satid == -1)
                continue;

            trace(3, "detslp_ll: slip detected sat=%2d f=%d\n", pObs->sat, j + 1);

            ssat[satid].slip[j] = 1;
        }
}

#ifdef _USE_PPP_
/* detect cycle slip by geometry free phase jump -----------------------------*/
void detslp_gf(const obs_t *obs)
{
    double g0, g1;
    int i, j, satid;

    prcopt_t *opt = &prcopt;
    obsd_t *pObs = NULL;

    for (i = 0; i < obs->n && i < MAXOBS; i++)
    {
        pObs = obs->data + i;

        if ((g1 = gfmeas(pObs)) == 0.0)
            continue;

        satid = find_sat_index(pObs->sat);

        if (satid == -1)
            continue;

        ssat[satid].sat = pObs->sat;
        ssat[satid].vs = 1;

        g0 = ssat[satid].gf;
        ssat[satid].gf = g1;

        if (g0 != 0.0 && fabs(g1 - g0) > THRES_GF_JUMP)
        {
            /*   printf("detslip_gf: sat=%2d gf=%8.3f->%8.3f\n", pObs->sat, g0, g1); */
            for (j = 0; j < opt->nf; j++)
                ssat[satid].slip[j] |= 1;
        }
    }
}
#endif
#ifndef _USE_PPP_
/* detect cycle slip by delta phase + doppler -----------------------------*/
void rcvbias_estimate(const double *delph, const int nc, double *rcvbias, double *rcvstd)
{
    int i, ns = 0;
    int nmid = ROUND(nc / 2.0);
    int nbound = nc - ROUND(nc / 2.0);
    *rcvstd = 0.0;
    *rcvbias = delph[nmid];
    double v = 0.0;
    for (i = 1; i < nbound; i++)
    {
        if (nmid + i < nc)
        {
            v = fabs(delph[nmid] - delph[nmid + i]);
            if (v < 0.5)
            {
                *rcvbias += delph[nmid + i];
                *rcvstd += v * v;
                ns++;
            }
        }

        if (nmid - i < nc)
        {
            v = fabs(delph[nmid] - delph[nmid - i]);
            if (v < 0.5)
            {
                *rcvbias += delph[nmid - i];
                *rcvstd += v * v;
                ns++;
            }
        }
    }

    if (ns == 0)
        *rcvstd = 99.0;
    else
    {
        *rcvbias = *rcvbias / (ns + 1);
        *rcvstd = sqrt(*rcvstd / (ns + 1));
    }
}

double detslp_doppler(const obsd_t *pRovObs, const int f, const double tt)
{
    double d1, d0, l1, l0, dave, delph = 0.0;
    ;
    int i, j, satid;

    satid = find_sat_index(pRovObs->sat);

    l1 = pRovObs->L[f];
    d1 = pRovObs->D[f];

    if (l1 == ssat[satid].L[f])
        return 0.0;

    if (satid == -1)
        return 0.0;

    ssat[satid].sat = pRovObs->sat;

    d0 = ssat[satid].D[f];
    l0 = ssat[satid].L[f];
    ssat[satid].L[f] = l1;
    ssat[satid].D[f] = d1;

    if (l0 == 0.0 || d0 == 0.0)
        return 0.0;

    dave = (d0 + d1) / 2.0;

    ssat[satid].dph[f] = l1 - l0 + dave * tt;

    return ssat[satid].dph[f];
}

/* detect cycle slip by single-difference geometry free phase jump -----------------------------*/
void epoch_detslp_doppler(const obs_t *obs_rov, const double tt)
{
    int s, f, i, sat, sys, idx, satid, ncount = 0;
    obsd_t *pObsRov = NULL;
    double delph[NFREQ * MAXOBS], dph, rcvbias = 0.0, rcvstd = 0.0;
    double srate = 0.0;
    double nslp = 0;

    for (i = 0; i < MAXOBS; i++)
    {
        for (f = 0; f < NFREQ; ++f)
            ssat[i].slip[f] = -1;
    }

    for (f = 0; f < NFREQ; ++f)
    {
        for (i = 0; i < obs_rov->n; ++i)
        {
            pObsRov = obs_rov->data + i;
            sat = pObsRov->sat;
            sys = satsys(sat, NULL);
            if (sys != _SYS_GPS_ && sys != _SYS_GAL_)
                continue;
            dph = detslp_doppler(pObsRov, f, tt);

            idx = -1;
            if (dph != 0.0)
            {
                if (ncount > 0)
                {
                    for (int j = 0; j < ncount; j++)
                    {
                        if (dph < delph[j])
                        {
                            idx = j;
                            for (int k = ncount - 1; k >= j; k--)
                                delph[k + 1] = delph[k];

                            delph[j] = dph;
                            break;
                        }
                    }
                    if (idx == -1)
                        delph[ncount] = dph;
                }
                else
                    delph[ncount] = dph;

                ncount++;
            }
        }
    }

    if (ncount != 0)
    {
        rcvbias_estimate(delph, ncount, &rcvbias, &rcvstd);

        for (i = 0; i < obs_rov->n; i++)
        {
            pObsRov = obs_rov->data + i;
            sat = pObsRov->sat;
            sys = satsys(sat, NULL);
            if (sys != _SYS_GPS_ && sys != _SYS_GAL_)
                continue;
            satid = find_sat_index(sat);
            if (ssat[satid].sat == 0 || satid == -1)
                continue;

            if (rcvbias == 0.0 || (fabs(rcvbias - ssat[satid].rbias) > 10.0 && rcvstd > 0.5)) // && fabs(rcvbias - ssat[satid].rbias) > 10.0  ssat[satid].
            {
                memset(ssat, 0, sizeof(ssat));
            }
            else
            {
                if (rcvstd < 0.15)
                {
                    ssat[satid].rbias = rcvbias;
                    ssat[satid].rstd = rcvstd;
                }

                for (f = 0; f < NFREQ; f++)
                {
                    if (ssat[satid].dph[f] == 0.0)
                        continue;
                    ssat[satid].dph[f] -= ssat[satid].rbias;
                    if (fabs(ssat[satid].dph[f]) > THRES_DA_JUMP)
                    {
                        ssat[satid].slip[f] = 1;
                        nslp++;
                    }
                    else
                        ssat[satid].slip[f] = 0;

#ifdef _TRACE_
                    printf("detslip_doppler: time=%d sat=%3d freq=%2d dph=%14.3f rcvbias=%8.3f slip=%2d\n",
                           obs_rov->time.time, ssat[satid].sat, f, ssat[satid].dph[f], ssat[satid].rbias, ssat[satid].slip[f]);
#endif
                }
            }
        }

        srate = nslp / ncount;
        printf("detslip_doppler: time=%d cycleslip rate=%6.1f %2.0f %2d\n", obs_rov->time.time, srate * 100.0, nslp, ncount);

        if (srate > 0.4)
            memset(ssat, 0, sizeof(ssat));
    }
}
#endif

#ifdef _USE_PPP_
/* detect slip by Melbourne-Wubbena linear combination jump ------------------*/
void detslp_mw(const obs_t *obs)
{
    double w0, w1;
    int i, j, satid;

    prcopt_t *opt = &prcopt;
    obsd_t *pObs = NULL;

    for (i = 0; i < obs->n && i < MAXOBS; i++)
    {
        pObs = obs->data + i;

        if ((w1 = mwmeas(pObs)) == 0.0)
            continue;

        satid = find_sat_index(pObs->sat);

        if (satid == -1)
            continue;

        ssat[satid].sat = pObs->sat;
        ssat[satid].vs = 1;

        w0 = ssat[satid].mw;
        ssat[satid].mw = w1;

        if (w0 != 0.0 && fabs(w1 - w0) > THRES_MW_JUMP)
        {
            /*   printf("detslip_mw: sat=%2d mw=%8.3f->%8.3f\n", pObs->sat, w0, w1);*/
            for (j = 0; j < opt->nf; j++)
                ssat[satid].slip[j] |= 1;
        }
    }
}
#endif

#ifdef _USE_PPP_
void detecs(const obs_t *obs)
{
    int i, j;

    prcopt_t *opt = &prcopt;

    for (i = 0; i < MAXOBS; i++)
        for (j = 0; j < opt->nf; j++)
        {
            ssat[i].slip[j] = 0;
        }

    /* detect slip by Melbourne-Wubbena linear combination jump */
    detslp_mw(obs);

    /* detect cycle slip by geometry-free phase jump */
    detslp_gf(obs);
}
#endif

/* precise tropospheric model ------------------------------------------------*/
double trop_model_prec(gtime_t time, const double *pos,
                       const double *azel, const double *x, double *dtdx,
                       double *var)
{
    const double zazel[] = {0.0, PI / 2.0};
    double zhd, m_h, m_w, cotz, grad_n, grad_e;

    /* zenith hydrostatic delay */
    zhd = tropmodel(pos, zazel, 0.0);

    /* mapping function */
    m_h = tropmapf(time, pos, azel, &m_w);

    if (azel[1] > 0.0)
    {

        /* m_w=m_0+m_0*cot(el)*(Gn*cos(az)+Ge*sin(az)): ref [6] */
        cotz = 1.0 / tan(azel[1]);
        grad_n = m_w * cotz * cos(azel[0]);
        grad_e = m_w * cotz * sin(azel[0]);
        m_w += grad_n * x[1] + grad_e * x[2];
        dtdx[1] = grad_n * (x[0] - zhd);
        dtdx[2] = grad_e * (x[0] - zhd);
    }
    dtdx[0] = m_w;
    *var = SQR(0.01);
    return m_h * zhd + m_w * (x[0] - zhd);
}

#ifdef _USE_PPP_
/* tropospheric model ---------------------------------------------------------*/
int model_trop(gtime_t time, const double *pos, const double *azel,
               const double *x, double *dtdx, double *dtrp, double *var)
{
    double trp[3] = {0};
    prcopt_t *opt = &prcopt;

    if (opt->tropopt == TROPOPT_EST || opt->tropopt == TROPOPT_ESTG)
    {
        matcpy(trp, x + IT(opt), opt->tropopt == TROPOPT_EST ? 1 : 3, 1);
        *dtrp = trop_model_prec(time, pos, azel, trp, dtdx, var);
        return 1;
    }

    return 0;
}

/* ionospheric model ---------------------------------------------------------*/
int model_iono(const double *pos, const double *azel,
               int sat, const double *x, double *dion, double *var)
{
    prcopt_t *opt = &prcopt;

    if (opt->ionoopt == IONOOPT_IF12 || opt->ionoopt == IONOOPT_IF1)
    {
        *dion = *var = 0.0;
        return 1;
    }
    return 0;
}
#endif

/* solar/lunar tides (ref [2] 7) ---------------------------------------------*/
void tide_pl(const double *eu, const double *rp, double GMp,
             const double *pos, double *dr)
{
    const double H3 = 0.292, L3 = 0.015;
    double r, ep[3], latp, lonp, p, K2, K3, a, H2, L2, dp, du, cosp, sinl, cosl;
    int i;

    if ((r = norm(rp, 3)) <= 0.0)
        return;

    for (i = 0; i < 3; i++)
        ep[i] = rp[i] / r;

    K2 = GMp / GME * SQR(RE_WGS84) * SQR(RE_WGS84) / (r * r * r);
    K3 = K2 * RE_WGS84 / r;
    latp = asin(ep[2]);
    lonp = atan2(ep[1], ep[0]);
    cosp = cos(latp);
    sinl = sin(pos[0]);
    cosl = cos(pos[0]);

    /* step1 in phase (degree 2) */
    p = (3.0 * sinl * sinl - 1.0) / 2.0;
    H2 = 0.6078 - 0.0006 * p;
    L2 = 0.0847 + 0.0002 * p;
    a = dot(ep, eu, 3);
    dp = K2 * 3.0 * L2 * a;
    du = K2 * (H2 * (1.5 * a * a - 0.5) - 3.0 * L2 * a * a);

    /* step1 in phase (degree 3) */
    dp += K3 * L3 * (7.5 * a * a - 1.5);
    du += K3 * (H3 * (2.5 * a * a * a - 1.5 * a) - L3 * (7.5 * a * a - 1.5) * a);

    /* step1 out-of-phase (only radial) */
    du += 3.0 / 4.0 * 0.0025 * K2 * sin(2.0 * latp) * sin(2.0 * pos[0]) * sin(pos[1] - lonp);
    du += 3.0 / 4.0 * 0.0022 * K2 * cosp * cosp * cosl * cosl * sin(2.0 * (pos[1] - lonp));

    dr[0] = dp * ep[0] + du * eu[0];
    dr[1] = dp * ep[1] + du * eu[1];
    dr[2] = dp * ep[2] + du * eu[2];
}

/* displacement by solid earth tide (ref [2] 7) ------------------------------*/
void tide_solid(const double *rsun, const double *rmoon,
                const double *pos, const double *E, double gmst, int opt,
                double *dr)
{
    double dr1[3], dr2[3], eu[3], du, dn, sinl, sin2l;

    /* step1: time domain */
    eu[0] = E[2];
    eu[1] = E[5];
    eu[2] = E[8];
    tide_pl(eu, rsun, GMS, pos, dr1);
    tide_pl(eu, rmoon, GMM, pos, dr2);

    /* step2: frequency domain, only K1 radial */
    sin2l = sin(2.0 * pos[0]);
    du = -0.012 * sin2l * sin(gmst + pos[1]);

    dr[0] = dr1[0] + dr2[0] + du * E[2];
    dr[1] = dr1[1] + dr2[1] + du * E[5];
    dr[2] = dr1[2] + dr2[2] + du * E[8];

    /* eliminate permanent deformation */
    if (opt & 8)
    {
        sinl = sin(pos[0]);
        du = 0.1196 * (1.5 * sinl * sinl - 0.5);
        dn = 0.0247 * sin2l;
        dr[0] += du * E[2] + dn * E[1];
        dr[1] += du * E[5] + dn * E[4];
        dr[2] += du * E[8] + dn * E[7];
    }
}

/* tidal displacement ----------------------------------------------------------
* displacements by earth tides
* args   : gtime_t tutc     I   time in utc
*          double *rr       I   site position (ecef) (m)
*          int    opt       I   options (or of the followings)
*                                 1: solid earth tide
*                                 2: ocean tide loading
*                                 4: pole tide
*                                 8: elimate permanent deformation
*          double *erp      I   earth rotation parameters (NULL: not used)
*          double *odisp    I   ocean loading parameters  (NULL: not used)
*                                 odisp[0+i*6]: consituent i amplitude radial(m)
*                                 odisp[1+i*6]: consituent i amplitude west  (m)
*                                 odisp[2+i*6]: consituent i amplitude south (m)
*                                 odisp[3+i*6]: consituent i phase radial  (deg)
*                                 odisp[4+i*6]: consituent i phase west    (deg)
*                                 odisp[5+i*6]: consituent i phase south   (deg)
*                                (i=0:M2,1:S2,2:N2,3:K2,4:K1,5:O1,6:P1,7:Q1,
*                                   8:Mf,9:Mm,10:Ssa)
*          double *dr       O   displacement by earth tides (ecef) (m)
* return : none
* notes  : see ref [1], [2] chap 7
*          see ref [4] 5.2.1, 5.2.2, 5.2.3
*          ver.2.4.0 does not use ocean loading and pole tide corrections
*-----------------------------------------------------------------------------*/
extern void tidedisp(gtime_t tutc, const double *rr, int opt, double *dr)
{
    double pos[2], E[9], drt[3], rs[3], rm[3], gmst, erpv[5] = {0};
    int i;

    dr[0] = dr[1] = dr[2] = 0.0;

    if (norm(rr, 3) <= 0.0)
        return;

    pos[0] = asin(rr[2] / norm(rr, 3));
    pos[1] = atan2(rr[1], rr[0]);
    xyz2enu(pos, E);

    if (opt & 1)
    { /* solid earth tides */
        /* sun and moon position in ecef */
        sunmoonpos(tutc, erpv, rs, rm, &gmst);
        tide_solid(rs, rm, pos, E, gmst, opt, drt);
        for (i = 0; i < 3; i++)
            dr[i] += drt[i];
    }
}

#define MU_GPS 3.9860050E14   /* gravitational constant         ref [1] */
#define MU_GLO 3.9860044E14   /* gravitational constant         ref [2] */
#define MU_GAL 3.986004418E14 /* earth gravitational constant   ref [7] */
#define MU_BDS 3.986004418E14 /* earth gravitational constant   ref [9] */
//From GLAB
/*****************************************************************************
* Name        : gravitationalDelayCorrection
* Description : Obtains the gravitational delay correction for the effect of
*               general relativity (red shift) to the GPS signal
* Parameters  :
* Name                           |Da|Unit|Description
* double  *receiverPosition       I  m    Position of the receiver
* double  *satellitePosition      I  m    Position of the satellite
* Returned value (double)         O  m    Gravitational delay correction
*****************************************************************************/
extern double gravitationalDelayCorrection(const int sys, const double *receiverPosition,
                                           const double *satellitePosition)
{
    double receiverModule;
    double satelliteModule;
    double distance;
    double MU = MU_GPS;

    receiverModule = sqrt(receiverPosition[0] * receiverPosition[0] + receiverPosition[1] * receiverPosition[1] +
                          receiverPosition[2] * receiverPosition[2]);
    satelliteModule = sqrt(satellitePosition[0] * satellitePosition[0] + satellitePosition[1] * satellitePosition[1] +
                           satellitePosition[2] * satellitePosition[2]);
    distance = sqrt((satellitePosition[0] - receiverPosition[0]) * (satellitePosition[0] - receiverPosition[0]) +
                    (satellitePosition[1] - receiverPosition[1]) * (satellitePosition[1] - receiverPosition[1]) +
                    (satellitePosition[2] - receiverPosition[2]) * (satellitePosition[2] - receiverPosition[2]));

    switch (sys)
    {
    case _SYS_GPS_:
        MU = MU_GPS;
        break;
    case _SYS_GLO_:
        MU = MU_GLO;
        break;
    case _SYS_GAL_:
        MU = MU_GAL;
        break;
    case _SYS_BDS_:
        MU = MU_BDS;
        break;
    default:
        MU = MU_GPS;
        break;
    }
    return 2.0 * MU / (CLIGHT * CLIGHT) * log((satelliteModule + receiverModule + distance) / (satelliteModule + receiverModule - distance));
}

#ifdef _USE_PPP_
/* temporal update of position -----------------------------------------------*/
void udpos_ppp(rcv_ppp_t *rcv)
{
    double F[NP_PPP], P[NP_PPP], FP[NP_PPP], x[NX_PPP], xp[NX_PPP], pos[3], Q[9] = {0}, Qv[9];
    int i, j, *ix, nx;

    /* state transition of position/velocity/acceleration */
    for (i = 0; i < NP_PPP; i++)
        F[i] = 0.0;

    for (i = 0; i < NX_PPP; i++)
        F[SMI(i, i)] = 1.0;

    /* fixed mode */
    if (prcopt.mode == PMODE_PPP_FIXED)
    {
        for (i = 0; i < 3; i++)
            initx(rcv, prcopt.ru[i], 1E-8, i);
        return;
    }

    /* initialize position for first epoch */
    for (i = 0; i < 3; i++)
        initx(rcv, rcv->x[i], VAR_POS, i);
    if (prcopt.dynamics)
    {
        for (i = 3; i < 6; i++)
            initx(rcv, rcv->x[i], VAR_VEL, i);
        for (i = 6; i < 9; i++)
            initx(rcv, 1E-6, VAR_ACC, i);
    }

    /* kinmatic mode without dynamics */
    if (!prcopt.dynamics)
    {
        for (i = 0; i < 3; i++)
        {
            initx(rcv, rcv->x[i], VAR_POS, i);
        }
        return;
    }

    for (i = 0; i < 6; i++)
    {
        F[SMI(3 + i, i)] = rcv->tt; /* i + (i + 3)*nx */
    }
    for (i = 0; i < 3; i++)
    {
        F[SMI(6 + i, i)] = SQR(rcv->tt) / 2.0; /* i + (i + 6)*nx  */
    }

    for (i = 0; i < NX_PPP; i++)
    {
        x[i] = rcv->x[i];
        for (j = 0; j < NX_PPP; j++)
        {
            P[SMI(i, j)] = rcv->P[SMI(j, i)];
        }
    }

    /* x=F*x, P=F*P*F+Q */
    matmul("NN", NX_PPP, 1, NX_PPP, 1.0, F, x, 0.0, xp);
    matmul("NN", NX_PPP, NX_PPP, NX_PPP, 1.0, F, P, 0.0, FP);
    matmul("NT", NX_PPP, NX_PPP, NX_PPP, 1.0, FP, F, 0.0, P);

    for (i = 0; i < NX_PPP; i++)
    {
        rcv->x[i] = xp[i];
        for (j = 0; j < NX_PPP; j++)
        {
            rcv->P[SMI(j, i)] = P[SMI(j, i)]; /* i + j * nx */
        }
    }

    /* process noise added to only acceleration */
    Q[0] = Q[4] = SQR(prcopt.prn[3]) * fabs(rcv->tt);
    Q[8] = SQR(prcopt.prn[4]) * fabs(rcv->tt);
    ecef2pos(rcv->x, pos);
    covecef(pos, Q, Qv);
    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++)
        {
            rcv->P[SMI(j + 6, i + 6)] += Qv[i + j * 3]; /*  i + 6 + (j + 6)*ppprtk->nx */
        }
}
#endif

#ifdef _USE_PPP_
/* temporal update of clock --------------------------------------------------*/
void udclk_ppp(rcv_ppp_t *rcv)
{
    prcopt_t *opt = &prcopt;
    double dtr, tt = fabs(rcv->tt);
    int i, sat_r, frq, ic, sys = opt->navsys;

    /* initialize every epoch for clock (white noise) */
    if (sys & _SYS_GPS_)
    {
        dtr = rcv->x[3];
        if (fabs(dtr) < 1.0e-16)
            dtr = 1.0e-16;
        ic = IC(0, opt);
        initx(rcv, dtr, VAR_CLK, ic);
    }
    if (sys & _SYS_GLO_)
    {
        dtr = rcv->x[4];
        if (fabs(dtr) < 1.0e-16)
            dtr = 1.0e-16;
        ic = IC(1, opt);
        initx(rcv, dtr, VAR_CLK, ic);
    }
    if (sys & _SYS_BDS_)
    {
        dtr = rcv->x[5];
        if (fabs(dtr) < 1.0e-16)
            dtr = 1.0e-16;
        ic = IC(2, opt);
        initx(rcv, dtr, VAR_CLK, ic);
    }
    if (sys & _SYS_GAL_)
    {
        dtr = rcv->x[6];
        if (fabs(dtr) < 1.0e-16)
            dtr = 1.0e-16;
        ic = IC(3, opt);
        initx(rcv, dtr, VAR_CLK, ic);
    }

    return;
}
#endif

#ifdef _USE_PPP_
/* temporal update of tropospheric parameters --------------------------------*/
void udtrop_ppp(rcv_ppp_t *rcv)
{
    double pos[3], azel[] = {0.0, PI / 2.0}, ztd, var;
    prcopt_t *opt = &prcopt;
    int i = IT(opt), j;

    if (rcv->x[i] == 0.0)
    {
        var = SQR(0.3);
        initx(rcv, 0.15, var, i);

        if (opt->tropopt >= TROPOPT_ESTG)
        {
            for (j = i + 1; j < i + 3; j++)
                initx(rcv, 1E-6, VAR_GRA, j);
        }
    }
    else
    {
        rcv->P[SMI(i, i)] += SQR(opt->prn[2]) * fabs(rcv->tt);

        if (opt->tropopt >= TROPOPT_ESTG)
        {
            for (j = i + 1; j < i + 3; j++)
            {
                rcv->P[SMI(i, j)] += SQR(opt->prn[2] * 0.1) * fabs(rcv->tt);
            }
        }
    }
}
#endif

#ifdef _USE_PPP_
/* temporal update of phase biases -------------------------------------------*/
void udbias_ppp(rcv_ppp_t *rcv, const obs_t *obs, const nav_t *nav)
{
    double lam[2];
    double L[NFREQ], P[NFREQ], Lc, Pc, bias[MAXOBS], offset = 0.0, pos[3] = {0};
    double ion, dantr[NFREQ] = {0};
    int i, j, k, l, f, sat, idx, satid, slip[MAXOBS] = {0}, clk_jump = 0;

    prcopt_t *opt = &prcopt;
    obsd_t *pObs = NULL;

    ecef2pos(rcv->x, pos);

    for (f = 0; f < NF(opt); f++)
    {
        for (i = 0; i < obs->n && i < MAXOBS; i++)
        {
            pObs = obs->data + i;
            sat = pObs->sat;

            satid = find_sat_index(sat);

            if (satid == -1)
                continue;

            if (ssat[satid].sat == 0)
            {
                ssat[satid].sat = pObs->sat;
                ssat[satid].vs = 1;
            }

            j = IB(satid + 1, f, opt);
            corr_meas(pObs, ssat[satid].azel, dantr, 0.0, L, P, &Lc, &Pc);

            bias[i] = 0.0;

            if (opt->ionoopt == IONOOPT_IF12)
            {
                bias[i] = Lc - Pc;
                slip[i] = ssat[satid].slip[0] || ssat[satid].slip[1];
            }
            else if (L[f] != 0.0 && P[f] != 0.0)
            {
                slip[i] = ssat[satid].slip[f];
                l = satsys(sat, NULL) == _SYS_GAL_ ? 2 : 1;
                lam[0] = satwavelen(sat, 0);
                lam[1] = satwavelen(sat, 1);
                if (pObs->P[0] == 0.0 || pObs->P[l] == 0.0 ||
                    lam[0] == 0.0 || lam[l] == 0.0 || lam[f] == 0.0)
                    continue;

                ion = (pObs->P[0] - pObs->P[l]) / (1.0 - SQR(lam[l] / lam[0]));
                bias[i] = L[f] - P[f] + 2.0 * ion * SQR(lam[f] / lam[0]);
            }

            if (rcv->x[j] == 0.0 || slip[i] || bias[i] == 0.0)
                continue;
        }

        for (i = 0; i < obs->n && i < MAXOBS; i++)
        {
            pObs = obs->data + i;
            sat = pObs->sat;
            satid = find_sat_index(sat);

            if (satid == -1)
                continue;

            if (ssat[satid].sat == 0)
            {
                ssat[satid].sat = pObs->sat;
                ssat[satid].vs = 1;
            }

            j = IB(satid + 1, f, opt);

            rcv->P[SMI(j, j)] += SQR(opt->prn[0]) * fabs(rcv->tt); /* j + j * ppprtk->nx*/

            if (bias[i] == 0.0 || (rcv->x[j] != 0.0 && !slip[i]))
                continue;

            /* reinitialize phase-bias if detecting cycle slip */
            initx(rcv, bias[i], VAR_BIAS, IB(satid + 1, f, opt));
        }
    }
}
#endif

#ifdef _USE_PPP_
/* temporal update of states --------------------------------------------------*/
void udstate_ppp(rcv_ppp_t *rcv, const obsd_t *obs, const nav_t *nav)
{
    prcopt_t *opt = &prcopt;
    /* temporal update of position */
    udpos_ppp(rcv);

    /* temporal update of clock */
    udclk_ppp(rcv);

    /* temporal update of tropospheric parameters */
    if (opt->tropopt == TROPOPT_EST || opt->tropopt == TROPOPT_ESTG)
    {
        udtrop_ppp(rcv);
    }

    /* temporal update of phase-bias */
    udbias_ppp(rcv, obs, nav);
}
#endif

static void ekf_measurement_predict(double *x, double *P, const double *H, const int *L, double *z_, double *R_, int n, int m, double *PHt)
{
    /* kalman filter measurement update for single measurement */
    /* n: number of state vector */
    /* m: number of measurement index */
    int i, j;
    /* PHt = P*H' */
    if (PHt != NULL)
    {
        for (i = 0; i < n; ++i)
        {
            PHt[i] = 0.0;
            for (j = 0; j < m; ++j)
            {
                PHt[i] += P[SMI(i, L[j])] * H[j];
            }
        }
        /* R_ = H*P*H' */
        if (R_ != NULL)
        {
            *R_ = 0.0;
            for (i = 0; i < m; ++i)
            {
                *R_ += H[i] * PHt[L[i]];
            }
        }
    }
    /* z_ = H*x */
    *z_ = 0.0;
    for (i = 0; i < m; ++i)
    {
        *z_ += H[i] * x[L[i]];
    }
    return;
}

static void ekf_measurement_update(double *x, double *P, const double *H, const int *L, double inov, double P_inov, int n, int m, double *PHt)
{
    /* kalman filter measurement update for single measurement */
    /* n: number of state vector */
    /* m: number of measurement index */
    int i, j;
    /* P_inov = R + H*P*H' */
    /* inov = z-H*x */
    /* x = x + K * inov = x + P*H'*inv(P_inov)*inov */
    for (i = 0; i < n; ++i)
    {
        x[i] += PHt[i] * inov / P_inov;
    }
    /* P = P - (P*H')*inv(P_inov)*(H*P) */
    for (i = 0; i < n; ++i)
    {
        for (j = 0; j <= i; ++j)
        {
            P[SMI(i, j)] -= PHt[i] * PHt[j] / P_inov;
        }
    }
    return;
}

#ifdef _USE_PPP_
/* phase and code residuals --------------------------------------------------*/
int ppp_filter(int post, const obs_t *obs, vec_t *vec, const double *dr,
               double *x, double *Pp, rcv_ppp_t *rcv)
{
    double lam[2];
    prcopt_t *opt = &prcopt;
    double y, r, cdtr, bias, C, rr[3], pos[3], e[3], dtdx[3], L[NFREQ], P[NFREQ], Lc, Pc, frq;
    double v[MAXOBS * 4], var[MAXOBS * 4], dtrp = 0.0, dion = 0.0, vart = 0.0, vari = 0.0, dcb, gravitationalDelayModel = 0.0;
    double dantr[NFREQ] = {0.0};
    int num = 0;
    double ve[MAXOBS * 2 * NFREQ] = {0}, vmax = 0;
    char str[32];
    int i, j, k, sat, satid, sys, nv = 0, stat = 1;
    int ic = 0, id = 0;
    double z = 0.0, z_ = 0.0, R = 0.0, R_ = 0.0, inov = 0.0, P_inov = 0.0, wsf = 0.0, wave = 0.0, elev = 0.0;
    int num_of_out = 0;
    double PHt[NX_PPP] = {0.0};
    int Idx[NX_PPP] = {0};
    double H[NX_PPP] = {0.0};
    double dx[NX_PPP] = {0.0};
    int max_st = NX_PPP;

    time2str(obs[0].time, str, 2);

    obsd_t *pObs = NULL;
    vec_t *pVec = NULL;

    for (i = 0; i < MAXOBS; i++)
        for (j = 0; j < opt->nf; j++)
            ssat[i].vsat[j] = 0;

    if (norm(rcv->x, 3) <= 0.0)
        for (i = 0; i < 3; i++)
            rr[i] = obs->pos[i];
    else
        for (i = 0; i < 3; i++)
            rr[i] = rcv->x[i];

    ecef2pos(rr, pos);

    for (i = 0; i < obs->n && i < MAXOBS; i++)
    {
        pObs = obs->data + i;
        sat = pObs->sat;

        satid = find_sat_index(sat);

        ssat[satid].sat = pObs->sat;
        ssat[satid].vs = 1;

        lam[0] = satwavelen(sat, 0);
        lam[1] = satwavelen(sat, 1);

        pVec = vec + i;

        if (lam[j / 2] == 0.0 || lam[0] == 0.0)
            continue;

        r = geodist(pVec->satpvt, rr, e);

        if (pVec->satpvt[0] == 0.0 || pVec->satpvt[1] == 0.0 || pVec->satpvt[2] == 0.0)
            continue;

        if (pVec->r <= 0.0 || pVec->azel[1] < opt->elmin)
        {
            continue;
        }

        if (!(sys = satsys(sat, NULL)))
        {
            continue;
        }

        if (!(sys & opt->navsys))
            continue;

        if (!model_trop(obs->time, pos, pVec->azel, x, dtdx, &dtrp, &vart) ||
            !model_iono(pos, pVec->azel, sat, x, &dion, &vari))
        {
            continue;
        }

        antmodel(pVec->azel, dantr);

        /* phase windup model */
        if (!model_phw(obs->time, sat, 2, pVec->satpvt, rr, &ssat[satid].phw))
            continue;

        //Gravitational delay correction */
        gravitationalDelayModel = gravitationalDelayCorrection(sys, rr, pVec->satpvt);

        /* corrected phase and code measurements */
        corr_meas(pObs, pVec->azel, dantr, ssat[satid].phw, L, P, &Lc, &Pc);

        for (j = 0; j < 2 * NF(opt); j++)
        {
            dcb = bias = 0.0;

            if (opt->ionoopt == IONOOPT_IF12)
            {
                if ((y = j % 2 == 0 ? Lc : Pc) == 0.0)
                    continue;
            }
            else
            {
                if ((y = j % 2 == 0 ? L[j / 2] : P[j / 2]) == 0.0)
                    continue;
            }

            C = SQR(lam[j / 2] / lam[0]) * ionmapf(pos, pVec->azel) * (j % 2 == 0 ? -1.0 : 1.0);

            memset(Idx, 0, sizeof(Idx));
            memset(H, 0, sizeof(H));

            for (k = 0; k < NX_PPP; k++)
                H[k] = k < 3 ? -e[k] : 0.0;
            for (k = 0; k < 3; k++)
                Idx[k] = k;

            num = 3;
            /*receiver clock */
            cdtr = 0.0;
            if (sys == _SYS_GPS_)
            {
                ic = IC(0, opt);
                cdtr = x[ic];
                H[num] = 1.0;
                Idx[num] = ic;
                ++num;
            }

            if (sys == _SYS_GLO_)
            {
                ic = IC(1, opt);
                cdtr = x[ic];
                H[num] = 1.0;
                Idx[num] = ic;
                ++num;
            }

            if (sys == _SYS_GAL_)
            {
                ic = IC(2, opt);
                cdtr = x[ic];
                H[num] = 1.0;
                Idx[num] = ic;
                ++num;
            }

            if (sys == _SYS_BDS_)
            {
                ic = IC(3, opt);
                cdtr = x[ic];
                H[num] = 1.0;
                Idx[num] = ic;
                ++num;
            }

            if (opt->tropopt == TROPOPT_EST || opt->tropopt == TROPOPT_ESTG)
            {
                for (k = 0; k < (opt->tropopt >= TROPOPT_ESTG ? 3 : 1); k++)
                {
                    H[num] = dtdx[k];
                    Idx[num] = IT(opt) + k;
                    ++num;
                }
            }

            if (j % 2 == 0)
            { /* phase bias */

                if ((bias = x[IB(satid + 1, j / 2, opt)]) == 0.0)
                    continue;
                H[num] = 1.0;
                Idx[num] = IB(satid + 1, j / 2, opt);
                ++num;
            }

            v[nv] = y - (r + cdtr - CLIGHT * pVec->satpvt[6] + dtrp + C * dion + dcb + bias - gravitationalDelayModel);

            if (j % 2 == 0)
                ssat[satid].vsat[j / 2] = 1;

            var[nv] = varerr(sat, sys, pVec->azel[1], j / 2, j % 2) + vart + SQR(C) * vari; //+var_rs[i];

            printf("pppos iter=%2d %s sat=%2d r=%.4f cdtr=%.4f %s ambidx=%d bias=%8.4f res=%9.4f std=%.4f el=%4.1f\n", post, str, sat, pVec->r, cdtr, j % 2 ? "P" : "L", i, bias, v[nv], sqrt(var[nv]), pVec->azel[1] * R2D);

            if (j % 2 == 0)
                ssat[satid].vsat[j / 2] = 1;

            if (!post)
            {
                R = var[nv];
                z = v[nv];

                ekf_measurement_predict(dx, Pp, H, Idx, &z_, &R_, max_st, num, PHt);
                inov = z - z_;
                P_inov = R + R_;
                wsf = (inov * inov) / P_inov;

                if (wsf > 36.0)
                {
                    ++num_of_out;
                }
                else
                {
                    ekf_measurement_update(dx, Pp, H, Idx, inov, P_inov, max_st, num, PHt);
                }
            }

            /* reject satellite by pre-fit residuals*/
            if (!post && opt->maxinno > 0.0 && fabs(v[nv]) > opt->maxinno)
            {
                ssat[satid].rejc[j % 2]++;
                continue;
            }
            nv++;
        }
    }

    if (!post)
    {
        for (int i = 0; i < NX_PPP; ++i)
        {
            x[i] += dx[i];
            rcv->x[i] += dx[i];
        }
    }

    return nv;
}
#endif

#ifdef _USE_PPP_
/* precise point positioning -------------------------------------------------*/
void pppos(obs_t *obs, nav_t *nav, rcv_ppp_t *rcv, int isPrint)
{
    char str[32];
    double rr[3] = {0.0}, pos[3], dr[3] = {0};
    vec_t vec[MAXOBS] = {0};
    double xp[NX_PPP], Pp[NX_PPP * NX_PPP];
    int i, j, k, nv, nvec = 0, info, stat = SOLQ_SINGLE;

    time2str(obs[0].time, str, 2);

    if (norm(rcv->x, 3) == 0.0)
        for (i = 0; i < 3; i++)
            rcv->x[i] = obs->pos[i];

    if (norm(rr, 3) == 0.0)
        for (i = 0; i < 3; i++)
            rr[i] = rcv->x[i];

    ecef2pos(rr, pos);

    detecs(obs);

    /*temporal update of ekf states*/
    udstate_ppp(rcv, obs, nav);

    /*satellite positions and clocks*/
    satposs(obs, vec, nav, EPHOPT_SSRAPC);

    nvec = compute_vector_data(obs, vec);

    nvec = get_match_ssr(obs, vec, nav);

    /*earth tides correction*/
    tidedisp(gpst2utc(obs[0].time), rcv->x, 1, dr);

    for (i = 0; i < MAX_ITER; i++)
    {
        for (j = 0; j < NX_PPP; j++)
        {
            xp[j] = rcv->x[j];
            for (k = 0; k < NX_PPP; k++)
                Pp[SMI(j, k)] = rcv->P[SMI(j, k)];
        }

        /*prefit residuals*/
        if (!(nv = ppp_filter(0, obs, vec + 0, dr, xp, Pp, rcv)))
        {
            trace(2, "%s ppp (%d) no valid obs data\n", str, i + 1);
            break;
        }

        if (ppp_filter(i + 1, obs, vec + 0, dr, xp, Pp, rcv))
        {
            for (j = 0; j < NX_PPP; j++)
            {
                rcv->x[j] = xp[j];
                for (k = 0; k < NX_PPP; k++)
                    rcv->P[SMI(j, k)] = Pp[SMI(j, k)];
            }
            stat = SOLQ_PPP;
            break;
        }
    }

    for (j = 0; j < 3; ++j)
        obs->pos[j] = xp[j];

    printf("pppos: %d pos: %10.4f %10.4f %10.4f dtr=%8.4f %8.4f trop=%.4f\n", obs[0].time.time, rcv->x[0], rcv->x[1], rcv->x[2], rcv->x[3], rcv->x[4], rcv->x[7]);
}
#endif

static void filter_RTD_update(obs_t *rov, vec_t *vec, double *x, double *P, double *time, int max_st)
{
    int week = 0, i, j, k;

    double cur_time = time2gpst(rov->time, &week);

    cur_time += week * 7 * 24 * 3600.0;

    if (*time == 0.0 || fabs(cur_time - *time) > 10.0) /* init or large gap */
    {
        memset(x, 0, sizeof(double) * max_st);
        memset(P, 0, sizeof(double) * SMD(max_st));
        /* xyz */
        for (i = 0; i < 3; ++i)
        {
            P[SMI(i, i)] = 100.0 * 100.0;
        }
        /* vel */
        for (i = 0; i < 3; ++i)
        {
            P[SMI(i + 3, i + 3)] = 100.0 * 100.0;
        }
        /* acc */
        for (i = 0; i < 3; ++i)
        {
            P[SMI(i + 6, i + 6)] = 10.0 * 10.0;
        }
    }
    else if (*time != cur_time)
    {
        double F[9][9] = {0};
        double x_[9] = {0};
        double xp[9] = {0};
        double P_[9][9] = {0};
        double PP[9][9] = {0};
        double FP[9][9] = {0};
        double tt = cur_time - *time;

        for (i = 0; i < 9; ++i)
            F[i][i] = 1.0;
        for (i = 0; i < 6; i++)
        {
            F[i][i + 3] = tt;
        }
        for (i = 0; i < 3; i++)
        {
            F[i][i + 6] = tt * tt / 2.0;
        }
        for (i = 0; i < 9; i++)
        {
            x_[i] = x[i];
            for (j = 0; j < 9; j++)
            {
                P_[i][j] = P[SMI(i, j)];
            }
        }
        /* x=F*x, P=F*P*F+Q */
        for (i = 0; i < 9; ++i)
        {
            xp[i] = 0.0;
            for (j = 0; j < 9; ++j)
            {
                xp[i] += F[i][j] * x_[j];
            }
        }
        /* FP */
        for (i = 0; i < 9; ++i)
        {
            for (j = 0; j < 9; ++j)
            {
                FP[i][j] = 0.0;
                for (k = 0; k < 9; ++k)
                {
                    FP[i][j] += F[i][k] * P_[k][j];
                }
            }
        }
        /* F*P*F' */
        for (i = 0; i < 9; ++i)
        {
            for (j = 0; j < 9; ++j)
            {
                PP[i][j] = 0.0;
                for (k = 0; k < 9; ++k)
                {
                    PP[i][j] += FP[i][k] * F[j][k];
                }
            }
        }
        for (i = 0; i < 9; ++i)
        {
            x[i] = xp[i];
            for (j = i; j < 9; ++j)
            {
                P[SMI(i, j)] = PP[i][j];
            }
        }

        /* process noise added to only acceleration */
        double prnNoise[3] = {0.5, 0.5, 0.1};
        double Q[9] = {0.0}, Qv[9] = {0.0};
        Q[0] = prnNoise[0] * prnNoise[0] * fabs(tt);
        Q[4] = prnNoise[1] * prnNoise[1] * fabs(tt);
        Q[8] = prnNoise[2] * prnNoise[2] * fabs(tt);
        double pos[3] = {0.0};
        ecef2pos(x, pos);
        covecef(pos, Q, Qv);
        for (i = 0; i < 3; i++)
            for (j = i; j < 3; j++)
            {
                P[SMI(i + 6, j + 6)] += Qv[i + j * 3];
            }
        /* store the initial position and velocity, only estimate the difference w.r.t initial position and velocity */
        for (i = 0; i < 6; ++i)
        {
            rov->pos[i] = x[i];
            x[i] = 0.0;
        }
        compute_vector_data(rov, vec);
    }

    *time = cur_time;

    return;
}

static int filter_RTD(obs_t *obs_ref, obs_t *obs_rov, vec_t *vec_ref, vec_t *vec_rov, int *iref, int *irov, int nsd, double *x,
	double *P, int np, int ns, int *num_of_obs, int *num_of_out, float *dop, float maskElev, int isPrint)
{
    int i = 0, j = 0, k = 0, s = 0, f = 0, isd = 0, sat = 0, sys = 0, numl = 0;

    double z = 0.0, R = 0.0, z_ = 0.0, R_ = 0.0, inov = 0.0, P_inov = 0.0, wsf = 0.0, elev = 0.0;
    double PHt[NX_RTK] = {0.0};

    int L[NX_RTK] = {0};
    double H[NX_RTK] = {0};

    int max_st = np + ns;
    int week = 0;
    int prn = 0;
    int satIDs[MAXOBS] = {0};
    int nsat = 0;
    double cur_time = time2gpst(obs_rov->time, &week);

    obsd_t *pObsRef = NULL;
    obsd_t *pObsRov = NULL;
    vec_t *pVecRef = NULL;
    vec_t *pVecRov = NULL;

    double codeCOV = 0.0;
    double phaseCOV = 0.0;

    /* note: the state vector is P(3), V(3), A(3),bias(1) */
    /* the bias vector can be used as the clock for all the system, need to reset before use it */

    *num_of_obs = 0;
    *num_of_out = 0;

    for (s = 0; s < NSYS; ++s)
    {
        for (f = 0; f < NFREQ; ++f)
        {
            /* reset the clock state vector */
            P[SMI(9, 9)] += 10000.0 * 10000.0;
            for (isd = 0; isd < nsd; ++isd)
            {
                j = irov[isd];
                i = iref[isd];
                pObsRef = obs_ref->data + i;
                pObsRov = obs_rov->data + j;
                pVecRef = vec_ref + i;
                pVecRov = vec_rov + j;
                sat = pObsRov->sat;
                sys = satsys(sat, &prn);
                if (satidx(sat, &prn) != s)
                    continue;

                elev = pVecRov->azel[1];

                if (elev < (maskElev * DEG2RAD))
                    continue;

                /* measurement */

                sdvarerr(sat, satsys(pObsRov->sat, &prn), elev, &codeCOV, &phaseCOV);

                if (fabs(pObsRov->P[f]) < 0.001 || fabs(pObsRef->P[f]) < 0.001)
                    continue;

                z = (pObsRov->P[f] - pObsRef->P[f]) - ((pVecRov->r - pVecRef->r) + (pVecRov->tro - pVecRef->tro));
                R = codeCOV;

                ++(*num_of_obs);

                memset(L, 0, sizeof(L));
                memset(H, 0, sizeof(H));

                numl = 0;
                H[numl] = -pVecRov->e[0];
                L[numl] = 0;
                ++numl;
                H[numl] = -pVecRov->e[1];
                L[numl] = 1;
                ++numl;
                H[numl] = -pVecRov->e[2];
                L[numl] = 2;
                ++numl;
                H[numl] = 1.0;
                L[numl] = 9;
                ++numl;

                ekf_measurement_predict(x, P, H, L, &z_, &R_, max_st, numl, PHt);
                inov = z - z_;
                P_inov = R + R_;
                wsf = (inov * inov) / P_inov;
                if (isPrint)
                {
                    printf("%10.3f,%c,%2i,%7.3f,%10.3f,%10.3f,%10.3f,%10.3f,%10.3f,P%i,%c\n", cur_time, sys2char(sys), prn, elev * 180.0 / PI, z, sqrt(R), inov, sqrt(P_inov), wsf, f + 1, wsf > 36.0 ? '-' : '+');
                }
                if (wsf > 144.0)
                {
                    ++(*num_of_out);
                }
                else
                {
                    ekf_measurement_update(x, P, H, L, inov, P_inov, max_st, numl, PHt);
                    for (k = 0; k < nsat; ++k)
                    {
                        if (satIDs[k] == sat)
                        {
                            break;
                        }
                    }
                    if (k == nsat)
                    {
                        satIDs[nsat] = sat;
                        ++nsat;
                    }
                }
            }
        }
    }

    /* doppler measuremnt update, then the cdt state become as the cdt bias */
    for (s = 0; s < NSYS; ++s)
    {
        for (f = 0; f < NFREQ; ++f)
        {
            /* reset the clock state vector */
            P[SMI(9, 9)] += 10000.0 * 10000.0;
            for (j = 0; j < obs_rov->n; ++j)
            {
                pObsRov = obs_rov->data + j;
                pVecRov = vec_rov + j;
                sat = pObsRov->sat;
                sys = satsys(sat, &prn);
                if (satidx(sat, &prn) != s)
                    continue;

                elev = pVecRov->azel[1];

                if (elev < (maskElev * DEG2RAD))
                    continue;

                /* measurement */

                double w = satwavelen(sat, f);

                sdvarerr(sat, satsys(pObsRov->sat, &prn), elev, &codeCOV, &phaseCOV);

                if (fabs(pObsRov->D[f]) < 0.001 || fabs(pVecRov->rate) < 0.001)
                    continue;

                /* doppler residual */
                z = -w * pObsRov->D[f] - (pVecRov->rate - CLIGHT * pVecRov->satpvt[7]);

                R = phaseCOV * 5.0 * 5.0;

                ++(*num_of_obs);

                memset(L, 0, sizeof(L));
                memset(H, 0, sizeof(H));

                numl = 0;
                H[numl] = -pVecRov->e[0];
                L[numl] = 3;
                ++numl;
                H[numl] = -pVecRov->e[1];
                L[numl] = 4;
                ++numl;
                H[numl] = -pVecRov->e[2];
                L[numl] = 5;
                ++numl;
                H[numl] = 1.0;
                L[numl] = 9;
                ++numl;

                ekf_measurement_predict(x, P, H, L, &z_, &R_, max_st, numl, PHt);
                inov = z - z_;
                P_inov = R + R_;
                wsf = (inov * inov) / P_inov;
                if (isPrint)
                {
                    printf("%10.3f,%c,%2i,%7.3f,%10.3f,%10.3f,%10.3f,%10.3f,%10.3f,D%i,%c\n", cur_time, sys2char(sys), prn, elev * 180.0 / PI, z, sqrt(R), inov, sqrt(P_inov), wsf, f + 1, wsf > 36.0 ? '-' : '+');
                }
                if (wsf > 144.0)
                {
                    ++(*num_of_out);
                }
                else
                {
                    ekf_measurement_update(x, P, H, L, inov, P_inov, max_st, numl, PHt);
                }
            }
        }
    }

    /* restore the full position and velocity */
    for (i = 0; i < 6; ++i)
    {
        x[i] += obs_rov->pos[i];
    }

    /* add velocity constraints */
    double vxyz[3] = {x[3], x[4], x[5]}, vned[3] = {0.0};
    double C_en[3][3] = {0};
    double blh[3] = {0};
    double covXYZ[SMD(3)] = {0};
    double covNED[SMD(3)] = {0};
    for (i = 0; i < 3; ++i)
    {
        for (j = i; j < 3; ++j)
        {
            covXYZ[SMI(i, j)] = P[SMI(i + 3, j + 3)];
        }
    }
    ecef2pos(x, blh);
    blh2C_en(blh, C_en);
    xyz2ned(C_en, vxyz, covXYZ, vned, covNED);
    double vel2D = sqrt(vned[0] * vned[0] + vned[1] * vned[1]);
    if (vel2D < 0.05)
    {
        for (i = 0; i < 3; ++i)
        {
            memset(L, 0, sizeof(L));
            memset(H, 0, sizeof(H));

            z = 0.0;
            R = 0.001 * 0.001;
            numl = 0;
            H[numl] = 1.0;
            L[numl] = 3 + i;
            ++numl;

            ekf_measurement_predict(x, P, H, L, &z_, &R_, max_st, numl, PHt);
            inov = z - z_;
            P_inov = R + R_;
            wsf = (inov * inov) / P_inov;
            ekf_measurement_update(x, P, H, L, inov, P_inov, max_st, numl, PHt);
        }
    }

    /* save the full position and velocity */
    for (i = 0; i < 6; ++i)
    {
        obs_rov->pos[i] = x[i];
        x[i] = 0.0;
    }

    compute_vector_data(obs_rov, vec_rov);

    /* compute dops */
    double HH[4 * MAXOBS] = {0.0}, Q[16];
    int n = 0;

    for (isd = 0; isd < nsd; ++isd)
    {
        j = irov[isd];
        i = iref[isd];
        pObsRef = obs_ref->data + i;
        pObsRov = obs_rov->data + j;
        pVecRef = vec_ref + i;
        pVecRov = vec_rov + j;
        sat = pObsRov->sat;
        for (k = 0; k < nsat; ++k)
        {
            if (satIDs[k] == sat)
            {
                break;
            }
        }
        if (k == nsat)
            continue; /* did not use this satellite */

        HH[0 + 4 * n] = -pVecRov->e[0];
        HH[1 + 4 * n] = -pVecRov->e[1];
        HH[2 + 4 * n] = -pVecRov->e[2];
        HH[3 + 4 * n] = 1.0;
        ++n;
    }

    if (n >= 4)
    {
        matmul("NT", 4, 4, n, 1.0, HH, HH, 0.0, Q);
        double Q_[16] = {0.0};
        if (!inv4(Q, Q_))
        {
            dop[0] = sqrt(Q_[0] + Q_[5] + Q_[10] + Q_[15]); /* GDOP */
            dop[1] = sqrt(Q_[0] + Q_[5] + Q_[10]);          /* PDOP */
            dop[2] = sqrt(Q_[0] + Q_[5]);                   /* HDOP */
            dop[3] = sqrtf(Q_[10]);                          /* VDOP */
        }
    }

    return nsat;
}

static int find_ref_sat_amb(ambset_t *ambset, int *refsat)
{
    int i, s1, s2, f, s, prn, k = 0;
    for (i = 0; i < ambset->n; ++i)
    {
        s1 = ambset->amb[i].s1;
        s2 = ambset->amb[i].s2;
        f = ambset->amb[i].f;
        s = satidx(s1, &prn);
        refsat[MI(s, f, NFREQ)] = s1;
        ++k;
    }
    return k;
}

static int ambres_RTD(obs_t *obs_ref, obs_t *obs_rov, vec_t *vec_ref, vec_t *vec_rov, int *iref, int *irov, int nsd, double *xf, double *Pf, int np, int ns, int *num_of_obs, int *num_of_out, ambset_t *ambset, double maskElev, int isPrint)
{
    /* given the xyz and coverance, then estimate the float DD ambiguities, then search ambiguities */

    int i = 0, j = 0, k = 0, s = 0, f = 0, isd = 0, sat = 0, sys = 0, numl = 0;

    double z = 0.0, R = 0.0, z_ = 0.0, R_ = 0.0, inov = 0.0, P_inov = 0.0, wsf = 0.0, elev = 0.0;

    double x[NX_RTK] = {0};
    double P[SMD(NX_RTK)] = {0};
    double PHt[NX_RTK] = {0.0};

    int L[NX_RTK] = {0};
    double H[NX_RTK] = {0};

    int refsat[NSYS * NFREQ] = {0}; /* reference satellite */

    int max_st = np + ns;
    int week = 0;
    int prn = 0;

    int refloc = 0;
    double z_ref = 0.0; /* residuals */
    double R_ref = 0.0; /* covariance */
    double w_ref = 0.0; /* wave */
    double w = 0.0;     /* wave */

    double time = time2gpst(obs_rov->time, &week);

    obsd_t *pObsRef = NULL;
    obsd_t *pObsRov = NULL;
    vec_t *pVecRef = NULL;
    vec_t *pVecRov = NULL;

    double codeCOV = 0.0;
    double phaseCOV = 0.0;

    int fixID = 58;

    if (find_ref_sat_amb(ambset, refsat) == 0)
        return fixID;

    memcpy(x, xf, sizeof(double) * max_st);
    memcpy(P, Pf, sizeof(double) * SMD(max_st));

    *num_of_obs = 0;
    *num_of_out = 0;

    for (s = 0; s < NSYS; ++s)
    {
        for (f = 0; f < NFREQ; ++f)
        {
            /* reference satellite measurement update */
            refloc = -1;
            for (isd = 0; isd < nsd; ++isd)
            {
                j = irov[isd];
                i = iref[isd];
                pObsRef = obs_ref->data + i;
                pObsRov = obs_rov->data + j;
                pVecRef = vec_ref + i;
                pVecRov = vec_rov + j;

                if (pObsRov->sat != refsat[MI(s, f, NFREQ)])
                    continue;

                elev = pVecRov->azel[1];

                sat = pObsRov->sat;

                sys = satsys(sat, &prn);

                if (elev < (maskElev * DEG2RAD))
                    continue;

                w = satwavelen(sat, f);

                if (w <= 0.0)
                    continue;

                sdvarerr(sat, satsys(pObsRov->sat, &prn), elev, &codeCOV, &phaseCOV);

                if (fabs(pObsRov->L[f]) < 0.001 || fabs(pObsRef->L[f]) < 0.001)
                    continue;

                z = (pObsRov->L[f] - pObsRef->L[f]) - ((pVecRov->r - pVecRef->r) + (pVecRov->tro - pVecRef->tro)) / w;
                R = phaseCOV;
                x[9] = floor(z + 0.5);
                P[SMI(9, 9)] += 100000.0 * 100000.0;

                memset(L, 0, sizeof(L));
                memset(H, 0, sizeof(H));

                numl = 0;
                H[numl] = -pVecRov->e[0] / w;
                L[numl] = 0;
                ++numl;
                H[numl] = -pVecRov->e[1] / w;
                L[numl] = 1;
                ++numl;
                H[numl] = -pVecRov->e[2] / w;
                L[numl] = 2;
                ++numl;

                H[numl] = 1.0;
                L[numl] = 9;
                ++numl;

                ekf_measurement_predict(x, P, H, L, &z_, &R_, max_st, numl, PHt);
                inov = z - z_;
                P_inov = R + R_;
                wsf = (inov * inov) / P_inov;
                if (isPrint)
                {
                    printf("%10.3f,%c%02i,%7.3f,%10.3f,%10.3f,%10.3f,%10.3f,%10.3f,L%i,%c\n", time, sys2char(sys), prn, elev * 180.0 / PI, z, sqrt(R), inov, sqrt(P_inov), wsf, f + 1, wsf > 36.0 ? '-' : '+');
                }

                ekf_measurement_update(x, P, H, L, inov, P_inov, max_st, numl, PHt);

                refloc = isd;

                break;
            }
            if (refloc < 0)
                continue;

            /* rover satellites */
            for (isd = 0; isd < nsd; ++isd)
            {
                if (isd == refloc)
                    continue;

                j = irov[isd];
                i = iref[isd];
                pObsRef = obs_ref->data + i;
                pObsRov = obs_rov->data + j;
                pVecRef = vec_ref + i;
                pVecRov = vec_rov + j;
                sat = pObsRov->sat;
                if (satidx(sat, &prn) != s)
                    continue;

                for (k = 0; k < ambset->n; ++k)
                {
                    if (ambset->amb[k].s1 == refsat[MI(s, f, NFREQ)] && ambset->amb[k].s2 == sat && ambset->amb[k].f == f)
                    {
                        break;
                    }
                }
                if (k == ambset->n)
                    continue; /* can not find the fixed ambiguity */

                elev = pVecRov->azel[1];

                if (elev < (maskElev * DEG2RAD))
                    continue;

                w = satwavelen(sat, f);

                if (w <= 0.0)
                    continue;

                sdvarerr(sat, satsys(pObsRov->sat, &prn), elev, &codeCOV, &phaseCOV);

                if (fabs(pObsRov->L[f]) < 0.001 || fabs(pObsRef->L[f]) < 0.001)
                    continue;

                z = (pObsRov->L[f] - pObsRef->L[f]) - ((pVecRov->r - pVecRef->r) + (pVecRov->tro - pVecRef->tro)) / w - ambset->amb[k].data;
                R = phaseCOV;

                //++(*num_of_obs);

                memset(L, 0, sizeof(L));
                memset(H, 0, sizeof(H));

                numl = 0;
                H[numl] = -pVecRov->e[0] / w;
                L[numl] = 0;
                ++numl;
                H[numl] = -pVecRov->e[1] / w;
                L[numl] = 1;
                ++numl;
                H[numl] = -pVecRov->e[2] / w;
                L[numl] = 2;
                ++numl;

                H[numl] = 1.0;
                L[numl] = 9;
                ++numl;

                ekf_measurement_predict(x, P, H, L, &z_, &R_, max_st, numl, PHt);
                inov = z - z_;
                P_inov = R + R_;
                wsf = (inov * inov) / P_inov;
                if (isPrint)
                {
                    printf("%10.3f,%c%02i,%7.3f,%10.3f,%10.3f,%10.3f,%10.3f,%10.3f,L%i,%c\n", time, sys2char(sys), prn, elev * 180.0 / PI, z, sqrt(R), inov, sqrt(P_inov), wsf, f + 1, wsf > 36.0 ? '-' : '+');
                }
                if (wsf > 36.0)
                {
                    //++(*num_of_out);
                }
                //else
                {
                    ekf_measurement_update(x, P, H, L, inov, P_inov, max_st, numl, PHt);
                }
            }
        }
    }
#if 0
	/* compute residuals */
	for (s = 0; s < NSYS; ++s)
	{
		for (f = 0; f < NFREQ; ++f)
		{
			/* reference satellite measurement update */
			refloc = -1;
			for (isd = 0; isd < nsd; ++isd)
			{
				j = irov[isd];
				i = iref[isd];
				pObsRef = obs_ref->data + i;
				pObsRov = obs_rov->data + j;
				pVecRef = vec_ref + i;
				pVecRov = vec_rov + j;

				if (pObsRov->sat != refsat[MI(s, f, NFREQ)]) continue;

				elev = pVecRov->azel[1];

				sat = pObsRov->sat;

				sys = satsys(sat, &prn);

				if (elev < (maskElev*PI / 180.0)) continue;

				w = satwavelen(sat, f);

				if (w <= 0.0) continue;

				sdvarerr(sat, satsys(pObsRov->sat, &prn), elev, &codeCOV, &phaseCOV);

				if (fabs(pObsRov->L[f]) < 0.001 || fabs(pObsRef->L[f]) < 0.001) continue;

				z = (pObsRov->L[f] - pObsRef->L[f]) - ((pVecRov->r - pVecRef->r) + (pVecRov->tro - pVecRef->tro)) / w;
				R = phaseCOV;
				x[9] = floor(z + 0.5);
				P[SMI(9, 9)] += 100000.0*100000.0;

				memset(L, 0, sizeof(L));
				memset(H, 0, sizeof(H));

				numl = 0;
				H[numl] = -pVecRov->e[0] / w; L[numl] = 0; ++numl;
				H[numl] = -pVecRov->e[1] / w; L[numl] = 1; ++numl;
				H[numl] = -pVecRov->e[2] / w; L[numl] = 2; ++numl;

				H[numl] = 1.0; L[numl] = 9; ++numl;

				ekf_measurement_predict(x, P, H, L, &z_, &R_, max_st, numl, PHt);
				inov = z - z_;
				P_inov = R + R_;
				wsf = (inov*inov) / P_inov;
				if (isPrint)
				{
					printf("%10.3f,%c%02i,%7.3f,%10.3f,%10.3f,%10.3f,%10.3f,%10.3f,L%i,%c\n", time, sys2char(sys), prn, elev * 180.0 / PI, z, sqrt(R), inov, sqrt(P_inov), wsf, f + 1, wsf > 36.0 ? '-' : '+');
				}

				ekf_measurement_update(x, P, H, L, inov, P_inov, max_st, numl, PHt);

				refloc = isd;

				break;

			}
			if (refloc < 0) continue;

			/* rover satellites */
			for (isd = 0; isd < nsd; ++isd)
			{
				if (isd == refloc) continue;

				j = irov[isd];
				i = iref[isd];
				pObsRef = obs_ref->data + i;
				pObsRov = obs_rov->data + j;
				pVecRef = vec_ref + i;
				pVecRov = vec_rov + j;
				sat = pObsRov->sat;
				if (satidx(sat, &prn) != s) continue;

				for (k = 0; k < ambset->n; ++k)
				{
					if (ambset->amb[k].s1 == refsat[MI(s, f, NFREQ)] && ambset->amb[k].s2 == sat && ambset->amb[k].f == f)
					{
						break;
					}
				}
				if (k == ambset->n) continue; /* can not find the fixed ambiguity */

				elev = pVecRov->azel[1];

				if (elev < (maskElev*PI / 180.0)) continue;

				w = satwavelen(sat, f);

				if (w <= 0.0) continue;

				sdvarerr(sat, satsys(pObsRov->sat, &prn), elev, &codeCOV, &phaseCOV);

				if (fabs(pObsRov->L[f]) < 0.001 || fabs(pObsRef->L[f]) < 0.001) continue;

				z = (pObsRov->L[f] - pObsRef->L[f]) - ((pVecRov->r - pVecRef->r) + (pVecRov->tro - pVecRef->tro)) / w - ambset->amb[k].data;
				R = phaseCOV;

				++(*num_of_obs);

				memset(L, 0, sizeof(L));
				memset(H, 0, sizeof(H));

				numl = 0;
				H[numl] = -pVecRov->e[0] / w; L[numl] = 0; ++numl;
				H[numl] = -pVecRov->e[1] / w; L[numl] = 1; ++numl;
				H[numl] = -pVecRov->e[2] / w; L[numl] = 2; ++numl;

				H[numl] = 1.0; L[numl] = 9; ++numl;

				ekf_measurement_predict(x, P, H, L, &z_, &R_, max_st, numl, PHt);
				inov = z - z_;
				P_inov = R + R_;
				wsf = (inov*inov) / P_inov;
				if (isPrint)
				{
					printf("%10.3f,%c%02i,%7.3f,%10.3f,%10.3f,%10.3f,%10.3f,%10.3f,L%i,%c\n", time, sys2char(sys), prn, elev * 180.0 / PI, z, sqrt(R), inov, sqrt(P_inov), wsf, f + 1, fabs(inov) > 0.35 ? '-' : '+');
				}
				if (fabs(inov) > 0.35)//wsf > 36.0)
				{
					++(*num_of_out);
				}
				//else
				{
					//	ekf_measurement_update(x, P, H, L, inov, P_inov, max_st, numl, PHt);
				}
			}
		}
	}
	if (*num_of_obs >= 4)
	{
		double rate = (*num_of_out)*100.0 / (*num_of_obs);
		if (rate < 30.0)
		{
			memcpy(xf, x, sizeof(double)*max_st);
			memcpy(Pf, P, sizeof(double)*SMD(max_st));
			fixID = 4;
		}
	}
#endif
    fixID = 4;
    return fixID;
}

extern int spp_processor(obs_t *obs, nav_t *nav, rcv_spp_t *rcv, int isPrint)
{
    vec_t vec[MAXOBS] = {0};
    int sat_use[MAXOBS] = {0};

    int i = 0, s = 0, f = 0, sat = 0, sys = 0, numl = 0, locCLK = 0, locDCB = 0;

    double z = 0.0, R = 0.0, z_ = 0.0, R_ = 0.0, inov = 0.0, P_inov = 0.0, wsf = 0.0, elev = 0.0;
    double PHt[NX_SPP] = {0.0};

    int L[NX_SPP] = {0};
    double H[NX_SPP] = {0};

    double *x = rcv->x;
    double *P = rcv->P;

    int max_st = NX_SPP;
    int week = 0;
    int prn = 0;

    double cur_time = time2gpst(obs->time, &week);

    obsd_t *pObs = NULL;
    vec_t *pVec = NULL;

    double codeCOV = 0.0;
    double phaseCOV = 0.0;

    int num_of_obs = 0;
    int num_of_out = 0;

    double maskElev = 10.0;

    satposs(obs, vec + 0, nav, EPHOPT_BRDC);

    if (rcv->time == 0.0)
    {

        num_of_obs = compute_vector_data(obs, vec);
        if (num_of_obs < 5)
            return 0;

        memset(x, 0, sizeof(double) * max_st);
        memset(P, 0, sizeof(double) * SMD(max_st));
        /* xyz */
        for (i = 0; i < 3; ++i)
        {
            P[SMI(i, i)] = 1000.0 * 1000.0;
        }
        /* cdt */
        for (sys = 0; sys < NSYS; ++sys)
        {
            P[SMI(3 + sys, 3 + sys)] = 10000.0 * 10000.0;
        }
        /* dcb */
        for (f = 1; f < NFREQ; ++f)
        {
            for (sys = 0; sys < NSYS; ++sys)
            {
                P[SMI(3 + NSYS * f + sys, 3 + NSYS * f + sys)] = 1000.0 * 1000.0;
            }
        }
    }
    else if (rcv->time != cur_time)
    {
        for (i = 0; i < 3; ++i)
        {
            obs->pos[i] = x[i];
            x[i] = 0.0;
        }
        num_of_obs = compute_vector_data(obs, vec);
        /* xyz */
        for (i = 0; i < 3; ++i)
        {
            P[SMI(i, i)] += 1000.0 * 1000.0;
        }
        /* cdt */
        for (sys = 0; sys < NSYS; ++sys)
        {
            P[SMI(3 + sys, 3 + sys)] += 1000.0 * 1000.0;
        }
        /* dcb */
        for (f = 1; f < NFREQ; ++f)
        {
            for (sys = 0; sys < NSYS; ++sys)
            {
                P[SMI(3 + NSYS * f + sys, 3 + NSYS * f + sys)] += 10.0 * 10.0;
            }
        }
    }

    rcv->time = cur_time;

    num_of_obs = 0;

    for (s = 0; s < NSYS; ++s)
    {
        for (i = 0; i < obs->n; ++i)
        {
            pObs = obs->data + i;
            pVec = vec + i;
            sat = pObs->sat;
            sys = satidx(sat, &prn);
            if (sys != s)
                continue; /* process data by satellite system index */
            elev = pVec->azel[1];
            if (elev < (maskElev * DEG2RAD))
                continue;
            for (f = 0; f < NFREQ; ++f)
            {
                locCLK = 0;
                locDCB = 0;

                sdvarerr(sat, satsys(pObs->sat, &prn), elev, &codeCOV, &phaseCOV);

                if (pObs->P[f] == 0.0)
                    continue;
                z = pObs->P[f] - (pVec->r + pVec->tro - pVec->satpvt[6] * CLIGHT);
                R = codeCOV;
                locCLK = 3 + sys;
                if (f > 0)
                {
                    locDCB = 3 + NSYS * f + sys;
                }
                else
                {
                    locDCB = 0;
                }

                ++num_of_obs;

                memset(L, 0, sizeof(L));
                memset(H, 0, sizeof(H));

                numl = 0;
                H[numl] = -pVec->e[0];
                L[numl] = 0;
                ++numl;
                H[numl] = -pVec->e[1];
                L[numl] = 1;
                ++numl;
                H[numl] = -pVec->e[2];
                L[numl] = 2;
                ++numl;
                if (locCLK > 0)
                {
                    H[numl] = 1.0;
                    L[numl] = locCLK;
                    ++numl;
                }
                if (locDCB > 0)
                {
                    H[numl] = 1.0;
                    L[numl] = locDCB;
                    ++numl;
                }

                ekf_measurement_predict(x, P, H, L, &z_, &R_, max_st, numl, PHt);
                inov = z - z_;
                P_inov = R + R_;
                wsf = (inov * inov) / P_inov;
                if (isPrint)
                {
                    printf("%10.3f,%3i,%3i,%7.3f,%10.3f,%10.3f,%10.3f,%10.3f,%10.3f,P%i,%c\n", cur_time, sat, sys, elev * 180.0 / PI, z, R, inov, sqrt(P_inov), wsf, f + 1, wsf > 36.0 ? '-' : '+');
                }
                if (wsf > 144.0)
                {
                    ++num_of_out;
                }
                else
                {
                    ekf_measurement_update(x, P, H, L, inov, P_inov, max_st, numl, PHt);
                    sat_use[i] = 1;
                }
            }
        }
    }

    rcv->n_used = 0;
    for (i = 0; i < obs->n; ++i)
    {
        if (sat_use[i] == 1)
        {
            ++rcv->n_used;
        }
    }

    /* store the coordinate in the state vector, then use in the apprxinate coordinate for the next epoch */
    for (i = 0; i < 3; ++i)
    {
        obs->pos[i] += x[i];
        x[i] = obs->pos[i]; 
    }

    return rcv->n_used;
}

/* algorithm for ourlier detection to validate the ambiguity set searched from LAMBDA */
#define NX_AMB 4

static int ambres_pos(obs_t *obs_ref, obs_t *obs_rov, vec_t *vec_ref, vec_t *vec_rov, int *iref, int *irov, 
	int nsd, double *xyz, double *covXYZ, int *num_of_obs, int *num_of_out, ambset_t *ambset, 
	float maskElev, int isPrint)
{
    /* given the xyz and coverance, then estimate the float DD ambiguities, then search ambiguities */

    int i = 0, j = 0, k = 0, s = 0, f = 0, isd = 0, sat = 0, sys = 0, numl = 0;

    double z = 0.0, R = 0.0, z_ = 0.0, R_ = 0.0, inov = 0.0, P_inov = 0.0, wsf = 0.0, elev = 0.0;

    double x[NX_AMB] = {0.0};
    double P[SMD(NX_AMB)] = {0.0};

    double PHt[NX_AMB] = {0.0};

    int L[NX_AMB] = {0};
    double H[NX_AMB] = {0};

    int refsat[NSYS * NFREQ] = {0}; /* reference satellite */

    int max_st = NX_AMB;
    int week = 0;
    int prn = 0;

    int refloc = 0;
    double z_ref = 0.0; /* residuals */
    double R_ref = 0.0; /* covariance */
    double w_ref = 0.0; /* wave */
    double w = 0.0;     /* wave */

    double time = time2gpst(obs_rov->time, &week);

    obsd_t *pObsRef = NULL;
    obsd_t *pObsRov = NULL;
    vec_t *pVecRef = NULL;
    vec_t *pVecRov = NULL;

    double codeCOV = 0.0;
    double phaseCOV = 0.0;

    int np = 4; /* number of positioning state + single-difference AMB state */
    int ns = 0; /* number of ambiguity state */

    int fixID = 57;

    if (find_ref_sat_amb(ambset, refsat) == 0)
        return fixID;

    memset(x, 0, sizeof(double) * max_st);
    memset(P, 0, sizeof(double) * SMD(max_st));

    for (i = 0; i < 3; ++i)
    {
        x[i] = xyz[i];
        for (j = i; j < 3; ++j)
        {
            P[SMI(i, j)] = covXYZ[SMI(i, j)];
        }
    }

    *num_of_obs = 0;
    *num_of_out = 0;

    for (s = 0; s < NSYS; ++s)
    {
        for (f = 0; f < NFREQ; ++f)
        {
            /* reference satellite measurement update */
            refloc = -1;
            for (isd = 0; isd < nsd; ++isd)
            {
                j = irov[isd];
                i = iref[isd];
                pObsRef = obs_ref->data + i;
                pObsRov = obs_rov->data + j;
                pVecRef = vec_ref + i;
                pVecRov = vec_rov + j;

                if (pObsRov->sat != refsat[MI(s, f, NFREQ)])
                    continue;

                elev = pVecRov->azel[1];

                sat = pObsRov->sat;

                sys = satsys(sat, &prn);

                if (elev < (maskElev * DEG2RAD))
                    continue;

                w = satwavelen(sat, f);

                if (w <= 0.0)
                    continue;

                sdvarerr(sat, satsys(pObsRov->sat, &prn), elev, &codeCOV, &phaseCOV);

                if (fabs(pObsRov->L[f]) < 0.001 || fabs(pObsRef->L[f]) < 0.001)
                    continue;

                z = (pObsRov->L[f] - pObsRef->L[f]) - ((pVecRov->r - pVecRef->r) + (pVecRov->tro - pVecRef->tro)) / w;
                R = phaseCOV;
                x[3] = floor(z + 0.5);
                P[SMI(3, 3)] += 100000.0 * 100000.0;

                memset(L, 0, sizeof(L));
                memset(H, 0, sizeof(H));

                numl = 0;
                H[numl] = -pVecRov->e[0] / w;
                L[numl] = 0;
                ++numl;
                H[numl] = -pVecRov->e[1] / w;
                L[numl] = 1;
                ++numl;
                H[numl] = -pVecRov->e[2] / w;
                L[numl] = 2;
                ++numl;

                H[numl] = 1.0;
                L[numl] = 3;
                ++numl;

                ekf_measurement_predict(x, P, H, L, &z_, &R_, np + ns, numl, PHt);
                inov = z - z_;
                P_inov = R + R_;
                wsf = (inov * inov) / P_inov;
                if (isPrint)
                {
                    printf("%10.3f,%c,%02i,%7.3f,%10.3f,%10.3f,%10.3f,%10.3f,%10.3f,L%i,%c\n", time, sys2char(sys), prn, elev * 180.0 / PI, z, sqrt(R), inov, sqrt(P_inov), wsf, f + 1, wsf > 36.0 ? '-' : '+');
                }

                ekf_measurement_update(x, P, H, L, inov, P_inov, np + ns, numl, PHt);

                refloc = isd;

                break;
            }
            if (refloc < 0)
                continue;

            /* rover satellites */
            for (isd = 0; isd < nsd; ++isd)
            {
                if (isd == refloc)
                    continue;

                j = irov[isd];
                i = iref[isd];
                pObsRef = obs_ref->data + i;
                pObsRov = obs_rov->data + j;
                pVecRef = vec_ref + i;
                pVecRov = vec_rov + j;
                sat = pObsRov->sat;
                if (satidx(sat, &prn) != s)
                    continue;

                for (k = 0; k < ambset->n; ++k)
                {
                    if (ambset->amb[k].s1 == refsat[MI(s, f, NFREQ)] && ambset->amb[k].s2 == sat && ambset->amb[k].f == f)
                    {
                        break;
                    }
                }
                if (k == ambset->n)
                    continue; /* can not find the fixed ambiguity */

                elev = pVecRov->azel[1];

                if (elev < (maskElev * DEG2RAD))
                    continue;

                w = satwavelen(sat, f);

                sys = satsys(sat, &prn);

                if (w <= 0.0)
                    continue;

                sdvarerr(sat, satsys(pObsRov->sat, &prn), elev, &codeCOV, &phaseCOV);

                if (fabs(pObsRov->L[f]) < 0.001 || fabs(pObsRef->L[f]) < 0.001)
                    continue;

                z = (pObsRov->L[f] - pObsRef->L[f]) - ((pVecRov->r - pVecRef->r) + (pVecRov->tro - pVecRef->tro)) / w - ambset->amb[k].data;
                R = phaseCOV;

                ++(*num_of_obs);

                memset(L, 0, sizeof(L));
                memset(H, 0, sizeof(H));

                numl = 0;
                H[numl] = -pVecRov->e[0] / w;
                L[numl] = 0;
                ++numl;
                H[numl] = -pVecRov->e[1] / w;
                L[numl] = 1;
                ++numl;
                H[numl] = -pVecRov->e[2] / w;
                L[numl] = 2;
                ++numl;

                H[numl] = 1.0;
                L[numl] = 3;
                ++numl;

                ekf_measurement_predict(x, P, H, L, &z_, &R_, np + ns, numl, PHt);
                inov = z - z_;
                P_inov = R + R_;
                wsf = (inov * inov) / P_inov;
                if (isPrint)
                {
                    printf("%10.3f,%c,%02i,%7.3f,%10.3f,%10.3f,%10.3f,%10.3f,%10.3f,L%i,%c\n", time, sys2char(sys), prn, elev * 180.0 / PI, z, sqrt(R), inov, sqrt(P_inov), wsf, f + 1, wsf > 36.0 ? '-' : '+');
                }
                if (wsf > 36.0)
                {
                    ++(*num_of_out);
                }
                {
                    ekf_measurement_update(x, P, H, L, inov, P_inov, np + ns, numl, PHt);
                }
            }
        }
    }
    for (i = 0; i < 3; ++i)
    {
        xyz[i] = x[i];
        for (j = i; j < 3; ++j)
        {
            covXYZ[SMI(i, j)] = P[SMI(i, j)];
        }
    }
    if (*num_of_obs >= 4)
    {
#ifdef _USE_AVE_AMBRES_
        fixID = 4;
#else
        double rate = (*num_of_out) * 100.0 / (*num_of_obs);
        if (rate < 50.0)
        {
            fixID = 4;
        }
#endif
    }
    return fixID;
}

/* functions related to float ambiguity filter */
static int find_next_state_index(state_tag_t *tag, int np, int ns, int max_st)
{
    int i = 0;
    for (i = 0; i < ns; ++i)
    {
        if (tag[i].time == 0.0 || tag[i].s2 == 0)
        {
            break;
        }
    }
    if (i == ns)
    {
        if ((np + ns) >= max_st)
            i = -1;
    }
    return i;
}

static int find_state_index(state_tag_t *tag, int ns, int s1, int s2, int f)
{
    int i = 0;
    for (i = 0; i < ns; ++i)
    {
        if (tag[i].s1 == s1 && tag[i].s2 == s2 && tag[i].f == f)
        {
            break;
        }
    }
    if (i == ns)
        i = -1;
    return i;
}

static void find_ref_sat_from_state(state_tag_t *tag, int ns, int *refsat)
{
    int i = 0, f = 0, n = 0, s = 0, prn = 0;
    memset(refsat, 0, sizeof(int) * NSYS * NFREQ);
    for (i = 0; i < ns; ++i)
    {
        f = tag[i].f;
        s = satidx(tag[i].s1, &prn);
        refsat[MI(s, f, NFREQ)] = tag[i].s1;
    }
    return;
}

static void state_switch_ref_sat_convert(double *x, double *P, state_tag_t *tag, int np, int ns, int oldrefsat, int newrefsat)
{
    /* convert the state vector while switch reference satellites */
    /* currently,just reset */
    int i = 0, j = 0, loc = 0;
    for (i = 0; i < ns; ++i)
    {
        if (tag[i].s1 == oldrefsat)
        {
            loc = np + i;
            x[loc] = 0.0;
            for (j = 0; j < (np + ns); ++j)
            {
                P[SMI(loc, j)] = 0.0;
            }
            memset(tag + i, 0, sizeof(state_tag_t));
        }
    }
    return;
}

static void state_switch_ref_sat(double *x, double *P, state_tag_t *tag, int np, int ns, int *newrefsat)
{
    int oldrefsat[NSYS * NFREQ] = {0};
    int s = 0, f = 0;
    find_ref_sat_from_state(tag, ns, oldrefsat);
    for (s = 0; s < NSYS; ++s)
    {
        for (f = 0; f < NFREQ; ++f)
        {
            if (oldrefsat[MI(s, f, NFREQ)] != newrefsat[MI(s, f, NFREQ)])
            {
                /* need to work on it later */
                state_switch_ref_sat_convert(x, P, tag, np, ns, oldrefsat[MI(s, f, NFREQ)], newrefsat[MI(s, f, NFREQ)]);
            }
        }
    }
    return;
}

static int find_ref_sat(obs_t *obs_ref, obs_t *obs_rov, vec_t *vec_ref, vec_t *vec_rov, int *iref, int *irov, int nsd, int *refsat, double maskElev)
{
    obsd_t *pObsRef = NULL;
    obsd_t *pObsRov = NULL;
    vec_t *pVecRef = NULL;
    vec_t *pVecRov = NULL;
    int s, f, isd, refloc, i, j, sat, prn, satid;
    double w_ref;
    int num_valid_frq = 0;

    for (s = 0; s < NSYS; ++s)
    {
        for (f = 0; f < NFREQ; ++f)
        {
            refloc = -1;
            for (isd = 0; isd < nsd; ++isd)
            {
                j = irov[isd];
                i = iref[isd];
                pObsRef = obs_ref->data + i;
                pObsRov = obs_rov->data + j;
                pVecRef = vec_ref + i;
                pVecRov = vec_rov + j;
                sat = pObsRov->sat;
                if (satidx(sat, &prn) != s)
                    continue;

                if (pVecRov->azel[1] < (maskElev * DEG2RAD))
                    continue;

                w_ref = satwavelen(sat, f);

                if (w_ref <= 0.0)
                    continue;

#ifdef _USE_SLIP_DETEC_
                satid = find_sat_index(sat);

                if (satid == -1)
                    continue;

                if (pObsRov->L[f] != 0.0 && pObsRef->L[f] != 0.0 && ssat[satid].slip[f] != 1)
                {
                    refloc = isd;
                    refsat[MI(s, f, NFREQ)] = sat;
                    break;
                }
#else
                if (pObsRov->L[f] != 0.0 && pObsRef->L[f] != 0.0)
                {
                    refloc = isd;
                    refsat[MI(s, f, NFREQ)] = sat;
                    break;
                }
#endif
            }
            if (refloc < 0)
                continue; /* cannot find the reference satellite */
            ++num_valid_frq;
        }
    }
    return num_valid_frq;
}

/* ambiguity resolution using lambda */
#define _DEBUG_LAMBDA_
#ifdef _DEBUG_LAMBDA_
static void matfprintf(const double *M, const int n, const int m, FILE *f)
{
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < m; j++)
            fprintf(f, "%.9f\t", M[i + j * n]);
        fprintf(f, "\n");
    }
    fclose(f);
}
#endif

static int compute_rtk_fixed_solution(const double *dX, const double *P, const int nx, double *dx_chk)
{
    double a[MAXAMB] = {0.0};
    double Q[MAXAMB * MAXAMB] = {0.0};
    double x_DD[MAXAMB] = {0.0};
    double tmp[MAXAMB * MAXAMB] = {0.0};

    unsigned char i, j, n_valid_kf_states, nDD, n_non_amb_states, n_DD_states;
    n_valid_kf_states = nx;
    n_non_amb_states = 3;
    nDD = n_valid_kf_states - n_non_amb_states;
    n_DD_states = n_non_amb_states + nDD;

    /* extract sub-matrices a and Q */
    for (i = n_non_amb_states; i < n_DD_states; i++)
    {
        a[i - n_non_amb_states] = dX[i];
        for (j = n_non_amb_states; j < n_DD_states; j++)
            Q[(i - n_non_amb_states) + (j - n_non_amb_states) * nDD] = P[i + j * n_DD_states];
    }

    double a_chk[MAXAMB] = {0.0}; // fixed integer ambiguity
    double sum_square_err[NUM_INT_AMB_CANDIDATE] = {0.0};

    /* DEBUG with sample a and Q */
#ifdef _DEBUG_LAMBDA_
    /*double a0[5] = { -1.0197, -5.8352, -21.828, -12.052, -5.9338 };
	double Q0[25] = { 0.01925,  0.015165, -0.014545, 0.0086737,-0.011658,
		0.015165, 0.020188, -0.0052634,0.0050194,-0.0078477,
		-0.014545,-0.0052634, 0.02956, -0.0083501, 0.017837,
		0.0086737,0.0050194,-0.0083501,0.0044364,-0.0057653,
		-0.011658,-0.0078477, 0.017837,-0.0057653, 0.012023 };
	nDD = 5;*/
#endif

#ifndef _USE_AVE_AMBRES_
    if (!lambda(a, Q, nDD, NUM_INT_AMB_CANDIDATE, a_chk, sum_square_err))
        return 0;

    double amb_res_ratio = sum_square_err[1] / sum_square_err[0];
    if (amb_res_ratio >= 3.0)
    {
#ifdef _DEBUG
        printf("Ambiguity fixed correctly. Computing fixed position...\n");
#endif

        double Qba[MAX_NON_AMB_STATE * MAXAMB] = {0.0};
        double Qinv[MAXAMB * MAXAMB] = {0.0};
        /*double dx_chk[MAX_NON_AMB_STATE] = { 0.0 };*/
        double tmp_vec[MAXAMB] = {0.0};

        for (i = 0; i < n_non_amb_states; i++)
        {
            for (j = n_non_amb_states; j < n_DD_states; j++)
                Qba[i + (j - n_non_amb_states) * n_non_amb_states] = P[i + j * n_DD_states];
        }

#ifdef _DEBUG_GNSS_NAV_
        FILE *fp_qba = fopen("Qba.txt", "w");
        matfprintf(Qba, n_non_amb_states, nDD, fp_qba);
        fclose(fp_qba);
#endif

        memset(Qinv, 0, sizeof(double) * MAXAMB * MAXAMB);
        memcpy(Qinv, Q, sizeof(double) * MAXAMB * MAXAMB);
        matinv(Qinv, nDD);

        memset(tmp, 0, sizeof(double) * MAXAMB * MAXAMB);
        matminus_fast(a_chk, a, nDD, 1, tmp_vec);
        matmul("NN", n_non_amb_states, nDD, nDD, 1.0, Qba, Qinv, 0.0, tmp);
        matmul("NN", n_non_amb_states, 1, nDD, 1.0, tmp, tmp_vec, 0.0, *dx_chk);
    }
    else
    {
        return 0;
    }
#else
    if (!lambda(a, Q, nDD, NUM_INT_AMB_CANDIDATE, a_chk, sum_square_err))
        return 0;

    double Qba[MAX_NON_AMB_STATE * MAXAMB] = {0.0};
    double Qinv[MAXAMB * MAXAMB] = {0.0};
    /*double dx_chk[MAX_NON_AMB_STATE] = { 0.0 };*/
    double tmp_vec[MAXAMB] = {0.0};

    for (i = 0; i < n_non_amb_states; i++)
    {
        for (j = n_non_amb_states; j < n_DD_states; j++)
            Qba[i + (j - n_non_amb_states) * n_non_amb_states] = P[i + j * n_DD_states];
    }

    memset(Qinv, 0, sizeof(double) * MAXAMB * MAXAMB);
    memcpy(Qinv, Q, sizeof(double) * MAXAMB * MAXAMB);
    matinv(Qinv, nDD);

    memset(tmp, 0, sizeof(double) * MAXAMB * MAXAMB);
    matminus_fast(a_chk, a, nDD, 1, tmp_vec);
    matmul("NN", n_non_amb_states, nDD, nDD, 1.0, Qba, Qinv, 0.0, tmp);
    matmul("NN", n_non_amb_states, 1, nDD, 1.0, tmp, tmp_vec, 0.0, dx_chk);

    /*   static bool_t search(const double *L, const double *d,
        const double *zs, int n, int m, double *zn, double *s)*/
#endif

    return 1;
}

typedef struct
{
    int loc;
    int s1;
    int s2;
    double value;
} ambloc_t;

static double amb_ratio_table(int namb)
{
    double table[] = {10.00, 7.30, 3.40, 2.35, 1.93, 1.70, 1.56, 1.46, 1.39, 1.34, 1.29, 1.25, 1.23, 1.21, 1.20, 1.20};
    if (namb <= 1)
        return 16.0;
    else if (namb >= 16)
        return table[15];
    else
        return table[namb - 1];
}

#ifdef _USE_RTD_
static int ambres_state_vector_new(double *x, double *P, state_tag_t *tag, const uint8_t *np, 
    const uint8_t *ns, ambset_t *ambset, double *time)
{
	int f, sys, prn_ref, prn;
	int		 i, j, k;
	double	 ai[MAXAMB * 2] = { 0 };
	double	 af[MAXAMB] = { 0 };
	float   ratioTOL = 0.0f;
	double   z, R, z_, R_, inov, P_inov, wsf;
	double H[NX_RTK] = { 0.0 }, PHt[NX_RTK] = { 0.0 };
	int L[NX_RTK] = { 0 }, numl = 0;

	//memcpy(xa, x, sizeof(double)*(*np + *ns));
	//memcpy(Pa, P, sizeof(double)*SMD(*np + *ns));

	int numa = 0;
	int prefix_numa = 0;
	int prefix_sats[MAXOBS] = { 0 };
	int prefix_nsat = 0;

	double	 Qa[MAXAMB * MAXAMB] = { 0 }, s[2];

	ambloc_t ambloc[MAXAMB] = { 0 }, temp_ambloc = { 0 }, cur_ambloc = { 0 };

	// int week = 0;

	int fixID = 56;

	//double time = time2gpst(obs_rov->time, &week);

	double covP3 = sqrt(P[SMI(0, 0)] + P[SMI(1, 1)] + P[SMI(2, 2)]);

	if (sqrt(covP3) > 1.0) {
		return fixID;
	}

	memset(ambset, 0, sizeof(ambset_t));

	for (i = 0; i < *ns; ++i)
	{
		if (tag[i].s1 == 0 || fabs(tag[i].time - (*time)) > 0.1 || P[SMI(*np + i, *np + i)] > 25.0) continue;
		if (fabs(x[*np + i] - floor(x[*np + i] + 0.5)) < 1.0e-6)
		{
			ambset->amb[ambset->n].f = tag[i].f;
			ambset->amb[ambset->n].s1 = tag[i].s1;
			ambset->amb[ambset->n].s2 = tag[i].s2;
			ambset->amb[ambset->n].data = floor(x[*np + i] + 0.5);
			++ambset->n;

			for (j = 0; j < prefix_nsat; ++j)
			{
				if (prefix_sats[j] == tag[i].s2)
				{
					break;
				}
			}
			if (j == prefix_nsat)
			{
				prefix_sats[j] = tag[i].s2;
				++prefix_nsat;
			}
			++prefix_numa;
		}
		cur_ambloc.loc = i;
		cur_ambloc.s1 = tag[i].s1;
		cur_ambloc.s2 = tag[i].s2;
		double fAmbEst = x[*np + i];
		double iAmbEst = floor(fAmbEst + 0.5);
		double rAmbEst = fAmbEst - iAmbEst;
		double fAmbCov = P[SMI(*np + i, *np + i)];
		cur_ambloc.value = rAmbEst * rAmbEst + fAmbCov;

		//     printf("float ambs: sat=%2d %2d loc=%2d freq=%2d amb=%10.3f\n", tag[i].s2, tag[i].s1, i, tag[i].f,x[*np + i]);

		if (numa == MAXAMB)
		{
			for (k = 0; k < numa; ++k)
			{
				for (j = k + 1; j < numa; ++j)
				{
					if (ambloc[k].value > ambloc[j].value)
					{
						temp_ambloc = ambloc[k];
						ambloc[k] = ambloc[j];
						ambloc[j] = temp_ambloc;
					}
				}
			}
			if (ambloc[numa - 1].value < cur_ambloc.value)
			{
				continue;
			}
			ambloc[numa - 1] = cur_ambloc;
		}
		else
		{
			ambloc[numa] = cur_ambloc;
			++numa;
		}
	}

	if ( (numa+ prefix_numa) < 5)
	{
		return fixID;
	}

	for (i = 0; i < numa; ++i)
	{
		for (j = i + 1; j < numa; ++j)
		{
			if (ambloc[i].value > ambloc[j].value)
			{
				temp_ambloc = ambloc[i];
				ambloc[i] = ambloc[j];
				ambloc[j] = temp_ambloc;
			}
		}
	}

	fixID = 56;
	for (i = 0; i < numa; ++i)
	{
		af[i] = x[*np + ambloc[i].loc];
		for (j = 0; j < numa; j++)
			Qa[i + j * numa] = P[SMI(ambloc[i].loc + *np, ambloc[j].loc + *np)];
	}

	for (i = 0; i < numa; ++i)
		Qa[i + i * numa] += 0.01*0.01;

	/* lambda/mlambda integer least-square estimation */
	if (!lambda(af, Qa, numa, NUM_INT_AMB_CANDIDATE, ai, s))
		return fixID;

	ambset->ratio = (s[1] / s[0]);
	if (ambset->ratio > 999.9) ambset->ratio = 999.9f;

	if (ambset->ratio < 3.0) {
		return fixID;
	}

	for (i = 0; i < numa; ++i)
	{
		double ambv = fabs(ai[i] - ROUND(ai[i]));
		if (ambv > 1.0e-2)
		{
			continue;
		}

		sys = satsys(tag[ambloc[i].loc].s1, &prn_ref);
		sys = satsys(tag[ambloc[i].loc].s2, &prn);

		ambset->amb[ambset->n].f = tag[ambloc[i].loc].f;
		ambset->amb[ambset->n].s1 = tag[ambloc[i].loc].s1;
		ambset->amb[ambset->n].s2 = tag[ambloc[i].loc].s2;
		ambset->amb[ambset->n].data = ai[i];
		++ambset->n;

		for (j = 0; j < prefix_nsat; ++j)
		{
			if (prefix_sats[j] == tag[ambloc[i].loc].s2)
			{
				break;
			}
		}
		if (j == prefix_nsat)
		{
			prefix_sats[j] = tag[ambloc[i].loc].s2;
			++prefix_nsat;
		}

		z = ai[i];
		R = 0.001*0.001;
		numl = 0;
		H[numl] = +1.0; L[numl] = ambloc[i].loc + *np; ++numl;
		ekf_measurement_predict(x, P, H, L, &z_, &R_, *np + *ns, numl, PHt);
		inov = z - z_;
		P_inov = R + R_;
		wsf = (inov*inov) / P_inov;
		ekf_measurement_update(x, P, H, L, inov, P_inov, *np + *ns, numl, PHt);

		//   printf("fixed ambs: sat=%2d %2d loc=%2d freq=%2d amb=%10.3f\n", tag[ambloc[i].loc].s2, tag[ambloc[i].loc].s1, cur_ambloc.loc, f, ai[i]);
	}
	if (prefix_nsat >= 8) {
		//memcpy(x, xa, sizeof(double)*(*np + *ns));
		//memcpy(P, Pa, sizeof(double)*SMD(*np + *ns));
		fixID = 4;
	}
	else {
		fixID = 54;
    }
	ambset->nsat = prefix_nsat;

    return fixID;
}
#else
static int ambres_state_vector_new(obs_t *obs_ref, obs_t *obs_rov, int nsd, double *x, double *P, state_tag_t *tag, int *np, int *ns, double *xa, double *Pa, ambset_t *ambset, FILE *flog, int isPrint)
{
	int f, sys, prn_ref, prn;
	int		 i, j, k;
	double	 ai[MAXAMB * 2] = { 0 };
	double	 af[MAXAMB] = { 0 };
	double   ratioTOL = 0.0;
	double   z, R, z_, R_, inov, P_inov, wsf;
	double H[NX_RTK] = { 0.0 }, PHt[NX_RTK] = { 0.0 };
	int L[NX_RTK] = { 0 }, numl = 0;

	memcpy(xa, x, sizeof(double)*(*np + *ns));
	memcpy(Pa, P, sizeof(double)*SMD(*np + *ns));

	int numa = 0;
	int prefix_numa = 0;
	int prefix_sats[MAXOBS] = { 0 };
	int prefix_nsat = 0;

	double	 Qa[MAXAMB * MAXAMB] = { 0 }, s[2];

	ambloc_t ambloc[MAXAMB] = { 0 }, temp_ambloc = { 0 }, cur_ambloc = { 0 };

	int week = 0;

	int fixID = 5;

	double time = time2gpst(obs_rov->time, &week);

	double covP3 = sqrt(P[SMI(0, 0)] + P[SMI(1, 1)] + P[SMI(2, 2)]);

	if (sqrt(covP3) > 1.0) return fixID;

	memset(ambset, 0, sizeof(ambset_t));

	for (i = 0; i < *ns; ++i)
	{
		if (tag[i].s1 == 0 || fabs(tag[i].time - time) > 0.1 || P[SMI(*np + i, *np + i)] > 25.0) continue;
		if (fabs(x[*np + i] - floor(x[*np + i] + 0.5)) < 1.0e-6)
		{
			ambset->amb[ambset->n].f = tag[i].f;
			ambset->amb[ambset->n].s1 = tag[i].s1;
			ambset->amb[ambset->n].s2 = tag[i].s2;
			ambset->amb[ambset->n].data = floor(x[*np + i] + 0.5);
			++ambset->n;

			for (j = 0; j < prefix_nsat; ++j)
			{
				if (prefix_sats[j] == tag[i].s2)
				{
					break;
				}
			}
			if (j == prefix_nsat)
			{
				prefix_sats[j] = tag[i].s2;
				++prefix_nsat;
			}
			++prefix_numa;
		}
		cur_ambloc.loc = i;
		cur_ambloc.s1 = tag[i].s1;
		cur_ambloc.s2 = tag[i].s2;
		double fAmbEst = x[*np + i];
		double iAmbEst = floor(fAmbEst + 0.5);
		double rAmbEst = fAmbEst - iAmbEst;
		double fAmbCov = P[SMI(*np + i, *np + i)];
		cur_ambloc.value = rAmbEst * rAmbEst + fAmbCov;

		//     printf("float ambs: sat=%2d %2d loc=%2d freq=%2d amb=%10.3f\n", tag[i].s2, tag[i].s1, i, tag[i].f,x[*np + i]);

		if (numa == MAXAMB)
		{
			for (k = 0; k < numa; ++k)
			{
				for (j = k + 1; j < numa; ++j)
				{
					if (ambloc[k].value > ambloc[j].value)
					{
						temp_ambloc = ambloc[k];
						ambloc[k] = ambloc[j];
						ambloc[j] = temp_ambloc;
					}
				}
			}
			if (ambloc[numa - 1].value < cur_ambloc.value)
			{
				continue;
			}
			ambloc[numa - 1] = cur_ambloc;
		}
		else
		{
			ambloc[numa] = cur_ambloc;
			++numa;
		}
	}

	if ((numa + prefix_numa) < 5)
	{
		return fixID;
	}

	for (i = 0; i < numa; ++i)
	{
		for (j = i + 1; j < numa; ++j)
		{
			if (ambloc[i].value > ambloc[j].value)
			{
				temp_ambloc = ambloc[i];
				ambloc[i] = ambloc[j];
				ambloc[j] = temp_ambloc;
			}
		}
	}

	fixID = 5;
	for (i = 0; i < numa; ++i)
	{
		af[i] = x[*np + ambloc[i].loc];
		for (j = 0; j < numa; j++)
			Qa[i + j * numa] = P[SMI(ambloc[i].loc + *np, ambloc[j].loc + *np)];
	}

	for (i = 0; i < numa; ++i)
		Qa[i + i * numa] += 0.01*0.01;

	/* lambda/mlambda integer least-square estimation */
	if (!lambda(af, Qa, numa, NUM_INT_AMB_CANDIDATE, ai, s))
		return fixID;

	ambset->ratio = (s[1] / s[0]);
	if (ambset->ratio > 999.9) ambset->ratio = 999.9f;

	if (ambset->ratio < 3.0) return fixID;

	for (i = 0; i < numa; ++i)
	{
		double ambv = fabs(ai[i] - ROUND(ai[i]));
		if (ambv > 1.0e-2)
		{
			continue;
		}

		sys = satsys(tag[ambloc[i].loc].s1, &prn_ref);
		sys = satsys(tag[ambloc[i].loc].s2, &prn);

		ambset->amb[ambset->n].f = tag[ambloc[i].loc].f;
		ambset->amb[ambset->n].s1 = tag[ambloc[i].loc].s1;
		ambset->amb[ambset->n].s2 = tag[ambloc[i].loc].s2;
		ambset->amb[ambset->n].data = ai[i];
		++ambset->n;

		for (j = 0; j < prefix_nsat; ++j)
		{
			if (prefix_sats[j] == tag[ambloc[i].loc].s2)
			{
				break;
			}
		}
		if (j == prefix_nsat)
		{
			prefix_sats[j] = tag[ambloc[i].loc].s2;
			++prefix_nsat;
		}

		z = ai[i];
		R = 0.001*0.001;
		numl = 0;
		H[numl] = +1.0; L[numl] = ambloc[i].loc + *np; ++numl;
		ekf_measurement_predict(xa, Pa, H, L, &z_, &R_, *np + *ns, numl, PHt);
		inov = z - z_;
		P_inov = R + R_;
		wsf = (inov*inov) / P_inov;
		ekf_measurement_update(xa, Pa, H, L, inov, P_inov, *np + *ns, numl, PHt);

		//   printf("fixed ambs: sat=%2d %2d loc=%2d freq=%2d amb=%10.3f\n", tag[ambloc[i].loc].s2, tag[ambloc[i].loc].s1, cur_ambloc.loc, f, ai[i]);
	}
	if (prefix_nsat >= 10)
	{
		memcpy(x, xa, sizeof(double)*(*np + *ns));
		memcpy(P, Pa, sizeof(double)*SMD(*np + *ns));
		fixID = 4;
	}
	else
		fixID = 54;
	ambset->nsat = prefix_nsat;

	return fixID;
}
#endif

static int ambres_state_vector(obs_t *obs_ref, obs_t *obs_rov, int nsd, double *x, double *P, state_tag_t *tag, int *np, int *ns, double *xa, double *Pa, ambset_t *ambset, FILE *flog, int isPrint)
{
    int f, sys;
    int i, j, k;
    double ai[MAXAMB * 2] = {0};
    double af[MAXAMB] = {0};
    //int iv[NX] = { 0 };
    int numa = 0;
    int nums = 0;

    double Qa[MAXAMB * MAXAMB] = {0}, s[2], ratioTOL;
    int satIDs[MAXAMB] = {0};

    double z, R, z_, R_, inov, P_inov, wsf;
    double H[NX_RTK] = {0.0}, PHt[NX_RTK] = {0.0};
    int L[NX_RTK] = {0}, numl = 0;

    memcpy(xa, x, sizeof(double) * (*np + *ns));
    memcpy(Pa, P, sizeof(double) * SMD(*np + *ns));

    ambloc_t ambloc[MAXAMB] = {0}, temp_ambloc = {0}, cur_ambloc = {0};

    int fixID = 5;
    int week = 0;

    double time = time2gpst(obs_rov->time, &week);

    double covP3 = sqrt(P[SMI(0, 0)] + P[SMI(1, 1)] + P[SMI(2, 2)]);

    if (sqrt(covP3) > 1.0)
        return fixID;

    memset(ambset, 0, sizeof(ambset_t));

    for (i = 0; i < *ns; ++i)
    {
        if (tag[i].s1 == 0 || fabs(tag[i].time - time) > 0.1 || P[SMI(*np + i, *np + i)] > 25.0)
            continue;
        cur_ambloc.loc = i;
        cur_ambloc.s1 = tag[i].s1;
        cur_ambloc.s2 = tag[i].s2;
        double fAmbEst = x[*np + i];
        double iAmbEst = floor(fAmbEst + 0.5);
        double rAmbEst = fAmbEst - iAmbEst;
        double fAmbCov = P[SMI(*np + i, *np + i)];
        cur_ambloc.value = rAmbEst * rAmbEst + fAmbCov;
        if (numa == MAXAMB)
        {
            for (k = 0; k < numa; ++k)
            {
                for (j = k + 1; j < numa; ++j)
                {
                    if (ambloc[k].value > ambloc[j].value)
                    {
                        temp_ambloc = ambloc[k];
                        ambloc[k] = ambloc[j];
                        ambloc[j] = temp_ambloc;
                    }
                }
            }
            if (ambloc[numa - 1].value < cur_ambloc.value)
            {
                continue;
            }
            ambloc[numa - 1] = cur_ambloc;
        }
        else
        {
            ambloc[numa] = cur_ambloc;
            ++numa;
        }
        for (j = 0; j < nums; ++j)
        {
            if (satIDs[j] == cur_ambloc.s2)
                break;
        }
        if (j == nums)
        {
            satIDs[nums] = cur_ambloc.s2;
            ++nums;
        }
    }

    if (numa < 5 || nums < 5)
        return fixID;

    for (i = 0; i < numa; ++i)
    {
        for (j = i + 1; j < numa; ++j)
        {
            if (ambloc[i].value > ambloc[j].value)
            {
                temp_ambloc = ambloc[i];
                ambloc[i] = ambloc[j];
                ambloc[j] = temp_ambloc;
            }
        }
    }

    fixID = 5;
    int idxIter = 0;
    int prn = 0;
    int prn_ref = 0;
    while (numa > 0)
    {
        nums = 0;
        for (i = 0; i < numa; ++i)
        {
            af[i] = x[*np + ambloc[i].loc];
            for (j = 0; j < numa; j++)
                Qa[i + j * numa] = P[SMI(ambloc[i].loc + *np, ambloc[j].loc + *np)];

            for (j = 0; j < nums; ++j)
            {
                if (satIDs[j] == ambloc[i].s2)
                    break;
            }
            if (j == nums)
            {
                satIDs[nums] = ambloc[i].s2;
                ++nums;
            }
        }

        if (numa < 5 || nums < 5)
            return fixID;

        for (i = 0; i < numa; ++i)
            Qa[i + i * numa] += 0.01 * 0.01;

        /* lambda/mlambda integer least-square estimation */
        if (!lambda(af, Qa, numa, NUM_INT_AMB_CANDIDATE, ai, s) || s[0] <= 0.0)
            break;

        ambset->ratio = (s[1] / s[0]);
        if (ambset->ratio > 999.9)
            ambset->ratio = 999.9f;

        ratioTOL = amb_ratio_table(numa);

        if (isPrint)
        {
            printf("%10.3f,%3i,%3i,%10.3f,%10.3f\n", time, numa, nums, ambset->ratio, ratioTOL);
        }
        if (flog)
        {
            fprintf(flog, "%10.3f,%3i,%3i,%10.3f,%10.3f, AMB\n", time, numa, nums, ambset->ratio, ratioTOL);
            for (i = 0; i < numa; ++i)
            {
                fprintf(flog, "%3i,%3i,%3i,%10.3f,%10.3f\n", tag[ambloc[i].loc].s1, tag[ambloc[i].loc].s2, tag[ambloc[i].loc].f, af[i], ai[i + numa]);
            }
        }

        if (ambset->ratio >= 2 * ratioTOL)
        {
            for (i = 0; i < numa; ++i)
            {
                sys = satsys(tag[ambloc[i].loc].s1, &prn_ref);
                sys = satsys(tag[ambloc[i].loc].s2, &prn);
                f = tag[ambloc[i].loc].f;

                ambset->amb[ambset->n].f = f;
                ambset->amb[ambset->n].s1 = tag[ambloc[i].loc].s1;
                ambset->amb[ambset->n].s2 = tag[ambloc[i].loc].s2;
                ambset->amb[ambset->n].data = ai[i];
                ++ambset->n;

                z = ai[i];
                R = 0.001 * 0.001;
                numl = 0;
                H[numl] = +1.0;
                L[numl] = ambloc[i].loc + *np;
                ++numl;
                ekf_measurement_predict(xa, Pa, H, L, &z_, &R_, *np + *ns, numl, PHt);
                inov = z - z_;
                P_inov = R + R_;
                wsf = (inov * inov) / P_inov;
                if (isPrint)
                {
                    printf("%10.3f,%c,%3i,%3i,%10.3f,%10.3f,%10.3f,%10.3f,%10.3f,B%i,%c\n", time, sys2char(sys), prn_ref, prn, z, R, inov, sqrt(P_inov), wsf, f + 1, wsf > 9.0 ? '-' : '+');
                }
                if (flog)
                {
                    fprintf(flog, "%10.3f,%c,%3i,%3i,%10.3f,%10.3f,%10.3f,%10.3f,%10.3f,B%i,%c\n", time, sys2char(sys), prn_ref, prn, z, R, inov, sqrt(P_inov), wsf, f + 1, wsf > 9.0 ? '-' : '+');
                }
                ekf_measurement_update(xa, Pa, H, L, inov, P_inov, *np + *ns, numl, PHt);
            }
            fixID = 4;
            if (ambset->ratio >= (2.0 * ratioTOL))
            {
                /* fix the filter */
                memcpy(x, xa, sizeof(double) * (*np + *ns));
                memcpy(P, Pa, sizeof(double) * SMD(*np + *ns));
            }
            ambset->nsat = nums;
            break;
        }
        /* remove the worst one */
        ++idxIter;
        --numa;
    }

    return fixID;
}

/* float filter */
static int filter_RTK(obs_t *obs_ref, obs_t *obs_rov, vec_t *vec_ref, vec_t *vec_rov, int *iref, int *irov, int nsd, double *x, double *P, state_tag_t *tag, int *np, int *ns, int *num_of_obs, int *num_of_out, double maskElev, int isPrint)
{
    int i = 0, j = 0, k = 0, s = 0, f = 0, isd = 0, sat = 0, numl = 0, locAmb = 0;

    double z = 0.0, R = 0.0, z_ = 0.0, R_ = 0.0, inov = 0.0, P_inov = 0.0, wsf = 0.0, elev = 0.0;
    double PHt[NX_RTK] = {0.0};

    int L[NX_RTK] = {0};
    double H[NX_RTK] = {0};

    int refsat[NSYS * NFREQ] = {0}; /* reference satellite */

    int max_st = NX_RTK;
    int week = 0;
    int prn = 0, sys = 0;

    int refloc = 0;
    double z_ref = 0.0; /* residuals */
    double R_ref = 0.0; /* covariance */
    double w_ref = 0.0; /* wave */
    double w = 0.0;     /* wave */

    double cur_time = time2gpst(obs_rov->time, &week);

    obsd_t *pObsRef = NULL;
    obsd_t *pObsRov = NULL;
    vec_t *pVecRef = NULL;
    vec_t *pVecRov = NULL;

    obsd_t *pObsRef_0 = NULL;
    obsd_t *pObsRov_0 = NULL;
    vec_t *pVecRef_0 = NULL;
    vec_t *pVecRov_0 = NULL;

    double codeCOV = 0.0;
    double phaseCOV = 0.0;

    /* reset the ambiguity state if there is not reference satellites */
    if (find_ref_sat(obs_ref, obs_rov, vec_ref, vec_rov, iref, irov, nsd, refsat, maskElev) == 0)
    {
        memset(ssat, 0, sizeof(ssat));
        *ns = 0;
        for (i = 0; i < NX_RTK; i++)
        {
            memset(tag + i, 0, sizeof(state_tag_t));
        }
        for (i = *np; i < NX_RTK; ++i)
        {
            /* add jump to */
            P[SMI(i, i)] += 10000.0 * 10000.0;
        }
        return 0;
    }

    /* ambiguities */
    /* check the reference satellite or switch if the reference satellite changes */
    state_switch_ref_sat(x, P, tag, *np, *ns, refsat);

    *num_of_obs = 0;
    *num_of_out = 0;

    /* phase measurement update */
    for (s = 0; s < NSYS; ++s)
    {
        if (s == 3)
            continue; /* skip GLONASS AR */
        for (f = 0; f < NFREQ; ++f)
        {
            /* reference satellite measurement update */
            refloc = -1;
            for (isd = 0; isd < nsd; ++isd)
            {
                j = irov[isd];
                i = iref[isd];
                pObsRef = obs_ref->data + i;
                pObsRov = obs_rov->data + j;
                pVecRef = vec_ref + i;
                pVecRov = vec_rov + j;

                if (pObsRov->sat != refsat[MI(s, f, NFREQ)])
                    continue;

                elev = pVecRov->azel[1];

                sat = pObsRov->sat;

                sys = satsys(sat, &prn);

                if (elev < (maskElev * DEG2RAD))
                    continue;

                w = satwavelen(sat, f);

                if (w <= 0.0)
                    continue;

                sdvarerr(sat, satsys(pObsRov->sat, &prn), elev, &codeCOV, &phaseCOV);

                if (pObsRov->L[f] != 0.0 && pObsRef->L[f] != 0.0)
                {
                    z = (pObsRov->L[f] - pObsRef->L[f]) - ((pVecRov->r - pVecRef->r) + (pVecRov->tro - pVecRef->tro)) / w;
                    R = phaseCOV;
                    x[9] = floor(z + 0.5);

                    P[SMI(9, 9)] += 10000.0 * 10000.0; /* SD ambiguities for the reference satellites */
                }
                else
                    continue;

                ++(*num_of_obs);

                memset(L, 0, sizeof(L));
                memset(H, 0, sizeof(H));

                numl = 0;
                H[numl] = -pVecRov->e[0] / w;
                L[numl] = 0;
                ++numl;
                H[numl] = -pVecRov->e[1] / w;
                L[numl] = 1;
                ++numl;
                H[numl] = -pVecRov->e[2] / w;
                L[numl] = 2;
                ++numl;

                H[numl] = 1.0;
                L[numl] = 9;
                ++numl;

                ekf_measurement_predict(x, P, H, L, &z_, &R_, *np + *ns, numl, PHt);
                inov = z - z_;
                P_inov = R + R_;
                wsf = (inov * inov) / P_inov;
                if (isPrint)
                {
                    printf("%10.3f,%c%02i,%7.3f,%10.3f,%10.3f,%10.3f,%10.3f,%10.3f,L%i,%c\n", cur_time, sys2char(sys), prn, elev * 180.0 / PI, z, sqrt(R), inov, sqrt(P_inov), wsf, f + 1, wsf > 36.0 ? '-' : '+');
                }

                ekf_measurement_update(x, P, H, L, inov, P_inov, *np + *ns, numl, PHt);

                refloc = isd;

                break;
            }
            if (refloc < 0)
                continue;

            j = irov[refloc];
            i = iref[refloc];
            pObsRef_0 = obs_ref->data + i;
            pObsRov_0 = obs_rov->data + j;
            pVecRef_0 = vec_ref + i;
            pVecRov_0 = vec_rov + j;

            int prn_ref = 0;

            sys = satsys(pObsRov_0->sat, &prn_ref);

            /* rover satellites */
            for (isd = 0; isd < nsd; ++isd)
            {
                if (isd == refloc)
                    continue;

                j = irov[isd];
                i = iref[isd];
                pObsRef = obs_ref->data + i;
                pObsRov = obs_rov->data + j;
                pVecRef = vec_ref + i;
                pVecRov = vec_rov + j;
                sat = pObsRov->sat;

                if (satidx(sat, &prn) != s)
                    continue;

                elev = pVecRov->azel[1];

                if (elev < (maskElev * DEG2RAD))
                    continue;

                w = satwavelen(sat, f);

                sys = satsys(sat, &prn);

                if (w <= 0.0)
                    continue;

                sdvarerr(sat, satsys(pObsRov->sat, &prn), elev, &codeCOV, &phaseCOV);

                if (pObsRov->L[f] != 0.0 && pObsRef->L[f] != 0.0)
                {
                    z = (pObsRov->L[f] - pObsRef->L[f]) - ((pVecRov->r - pVecRef->r) + (pVecRov->tro - pVecRef->tro)) / w;
                    R = phaseCOV;

                    locAmb = find_state_index(tag, *ns, pObsRov_0->sat, pObsRov->sat, f);
                    if (locAmb < 0)
                    {
                        /* new state vector, need to add */
                        locAmb = find_next_state_index(tag, *np, *ns, max_st);
                        if (locAmb < 0)
                        {
                            /* cannot add ambiguity state vector */
                            continue;
                        }
                        tag[locAmb].s1 = pObsRov_0->sat;
                        tag[locAmb].s2 = pObsRov->sat;
                        tag[locAmb].f = f;
                        tag[locAmb].time = cur_time;
                        if (locAmb == *ns)
                        {
                            /* add new */
                            if (isPrint)
                            {
                                //printf("%10.3f,%c%02d-%02d,%i,%i,NEW\n", cur_time, sys2char(sys), prn_ref, prn, locAmb, *ns);
                            }
                            ++(*ns);
                        }
                        else
                        {
                            /* use old location */
                            if (isPrint)
                            {
                                //printf("%10.3f,%c%02d-%02d,%i,%i,OLD\n", cur_time, sys2char(sys), prn_ref, prn, locAmb, *ns);
                            }
                            k = 0;
                        }
                        locAmb += *np;
                        x[locAmb] = floor(z - x[9] + 0.5);
                        for (k = 0; k < (*np + *ns); ++k)
                        {
                            P[SMI(k, locAmb)] = 0.0;
                        }
                        P[SMI(locAmb, locAmb)] = 10000.0 * 10000.0;
                    }
                    else
                    {
                        /* old state vector, already exist */
                        /* check cycle slip */
                        /* to be done */
                        tag[locAmb].time = cur_time;
                        locAmb += *np;
#ifdef _USE_SLIP_DETEC_
                        /* need to do both reference and rover satellite */
                        int satid = find_sat_index(sat);
                        if (ssat[satid].slip[f] != 0)
                        {
                            for (k = 0; k < (*np + *ns); ++k)
                            {
                                P[SMI(k, locAmb)] = 0.0;
                            }
                            P[SMI(locAmb, locAmb)] += 1.0e8;
                            x[locAmb] = floor(z - x[9] + 0.5);
                        }
                        else
                        {
                            P[SMI(locAmb, locAmb)] += 1.0e-3;
                        }
#else
                        P[SMI(locAmb, locAmb)] += 10000.0 * 10000.0;
#endif
                    }
                }
                else
                    continue;

                ++(*num_of_obs);

                memset(L, 0, sizeof(L));
                memset(H, 0, sizeof(H));

                numl = 0;
                H[numl] = -pVecRov->e[0] / w;
                L[numl] = 0;
                ++numl;
                H[numl] = -pVecRov->e[1] / w;
                L[numl] = 1;
                ++numl;
                H[numl] = -pVecRov->e[2] / w;
                L[numl] = 2;
                ++numl;

                H[numl] = 1.0;
                L[numl] = locAmb;
                ++numl;
                H[numl] = 1.0;
                L[numl] = 9;
                ++numl;

                ekf_measurement_predict(x, P, H, L, &z_, &R_, *np + *ns, numl, PHt);
                inov = z - z_;
                P_inov = R + R_;
                wsf = (inov * inov) / P_inov;
                if (isPrint)
                {
                    printf("%10.3f,%c%02i,%7.3f,%10.3f,%10.3f,%10.3f,%10.3f,%10.3f,L%i,%c,%i,%i\n", cur_time, sys2char(sys), prn, elev * 180.0 / PI, z, sqrt(R), inov, sqrt(P_inov), wsf, f + 1, wsf > 36.0 ? '-' : '+', locAmb, *ns);
                }
                if (wsf > 36.0)
                {
                }
                else
                {
                    ekf_measurement_update(x, P, H, L, inov, P_inov, *np + *ns, numl, PHt);
                }
            }
        }
    }

    /* put ambiguity resolution here */

    return *num_of_obs;
}

/* epoch-by-epoch folat filter */
#define NX_MAX (50) // (MAXOBS + MAXOBS)

static int ambres_RTK(obs_t *obs_ref, obs_t *obs_rov, vec_t *vec_ref, vec_t *vec_rov, 
    int *iref, int *irov, int nsd, 
    const double *xyz, const double *covXYZ,
    double *x, double *P, state_tag_t *tag, uint8_t *num_s,
    int *num_of_obs, int *num_of_out, float maskElev)
{   
    /* given the xyz and coverance, then estimate the float DD ambiguities, then search ambiguities */
    int i = 0, j = 0, k = 0, s = 0, f = 0, isd = 0, sat = 0, sys = 0, numl = 0, locAmb = 0;
    int satid = -1;
    double z = 0.0, R = 0.0, z_ = 0.0, R_ = 0.0, inov = 0.0, P_inov = 0.0, wsf = 0.0, elev = 0.0;

    double PHt[NX_MAX] = { 0.0 };

    int L[NX_MAX] = { 0 };
    double H[NX_MAX] = { 0 };

    int refsat[NSYS*NFREQ] = { 0 }; /* reference satellite */

    int max_st = NX_MAX;
    int week = 0;
    int prn = 0;

    int refloc = 0;
    double z_ref = 0.0; /* residuals */
    double R_ref = 0.0; /* covariance */
    double w_ref = 0.0; /* wave */
    double w = 0.0; /* wave */
    double initCovAMB = 100000000.0;

    double time = time2gpst(obs_rov->time, &week);
    double sec = fabs(time - ROUND(time));

    obsd_t *pObsRef = NULL;
    obsd_t *pObsRov = NULL;
    vec_t *pVecRef = NULL;
    vec_t *pVecRov = NULL;

    double codeCOV = 0.0;
    double phaseCOV = 0.0;

    int np = 4; /* number of positioning state + single-difference AMB state */
    int ns = 0; /* number of ambiguity state */

    double rmsPOS3 = sqrt(covXYZ[SMI(0, 0)] + covXYZ[SMI(1, 1)] + covXYZ[SMI(2, 2)]);

    int8_t fixID = 5;

    if (rmsPOS3 > 1.2) return fixID; /* require filter convergence to a degree */

    //find_ref_sat(obs_ref, obs_rov, vec_ref, vec_rov, iref, irov, nsd, refsat, maskElev);

    if (find_ref_sat(obs_ref, obs_rov, vec_ref, vec_rov, iref, irov, nsd, refsat, maskElev) == 0) return fixID;

    memset(x, 0, sizeof(double)*max_st);
    memset(P, 0, sizeof(double)*SMD(max_st));

    for (i = 0; i < 3; ++i)
    {
        x[i] = xyz[i];
        for (j = i; j < 3; ++j)
        {
            P[SMI(i, j)] = covXYZ[SMI(i, j)];
        }
    }

    *num_of_obs = 0;
    *num_of_out = 0;

    for (s = 0; s < NSYS; ++s)
    {
        if (s == 3) continue; /* skip GLONASS AR */
        for (f = 0; f < NFREQ; ++f)
        {
            /* reference satellite measurement update */
            refloc = -1;
            for (isd = 0; isd < nsd; ++isd)
            {
                j = irov[isd];
                i = iref[isd];
                pObsRef = obs_ref->data + i;
                pObsRov = obs_rov->data + j;
                pVecRef = vec_ref + i;
                pVecRov = vec_rov + j;

                if (pObsRov->sat != refsat[MI(s, f, NFREQ)]) continue;

                elev = pVecRov->azel[1];

                sat = pObsRov->sat;

                sys = satsys(sat, &prn);

                if (elev < (maskElev*PI / 180.0)) continue;

                w = satwavelen(sat, f);

                if (w <= 0.0) continue;

                sdvarerr(sat, satsys(pObsRov->sat, &prn), elev, &codeCOV, &phaseCOV);

                if (pObsRov->L[f] != 0.0 && pObsRef->L[f] != 0.0)
                {
                    z = (pObsRov->L[f] - pObsRef->L[f]) - ((pVecRov->r - pVecRef->r) + (pVecRov->tro - pVecRef->tro)) / w;
                    R = phaseCOV;
                    x[3] = floor(z + 0.5);
					/* it is a epoch-by-epoch filter, no need for the cycle slip information here */
                    P[SMI(3, 3)] += initCovAMB;
                }
                else
                    continue;

                ++(*num_of_obs);

                memset(L, 0, sizeof(L));
                memset(H, 0, sizeof(H));

                numl = 0;
                H[numl] = -pVecRov->e[0] / w; L[numl] = 0; ++numl;
                H[numl] = -pVecRov->e[1] / w; L[numl] = 1; ++numl;
                H[numl] = -pVecRov->e[2] / w; L[numl] = 2; ++numl;

                H[numl] = 1.0; L[numl] = 3; ++numl;

                ekf_measurement_predict(x, P, H, L, &z_, &R_, np + ns, numl, PHt);
                inov = z - z_;
                P_inov = R + R_;
                wsf = (inov*inov) / P_inov;

                ekf_measurement_update(x, P, H, L, inov, P_inov, np + ns, numl, PHt);

                refloc = isd;

                break;

            }
            if (refloc < 0) continue;

            /* rover satellites */
            for (isd = 0; isd < nsd; ++isd)
            {
                if (isd == refloc) continue;

                j = irov[isd];
                i = iref[isd];
                pObsRef = obs_ref->data + i;
                pObsRov = obs_rov->data + j;
                pVecRef = vec_ref + i;
                pVecRov = vec_rov + j;
                sat = pObsRov->sat;

                if (satidx(sat, &prn) != s) continue;

                elev = pVecRov->azel[1];

                if (elev < (maskElev*PI / 180.0)) continue;

                w = satwavelen(sat, f);

                sys = satsys(sat, &prn);

                if (w <= 0.0) continue;

                sdvarerr(sat, satsys(pObsRov->sat, &prn), elev, &codeCOV, &phaseCOV);

                if (pObsRov->L[f] != 0.0 && pObsRef->L[f] != 0.0)
                {
                    z = (pObsRov->L[f] - pObsRef->L[f]) - ((pVecRov->r - pVecRef->r) + (pVecRov->tro - pVecRef->tro)) / w;
                    R = phaseCOV;
                    tag[ns].s1 = refsat[MI(s, f, NFREQ)];
                    tag[ns].s2 = pObsRov->sat;
                    tag[ns].f = f;
                    tag[ns].time = time;
                    locAmb = ns;
                    if ((np + ns) >= NX_MAX) continue;
                    ++ns;
                    x[locAmb + np] = floor(z - x[3] + 0.5);
					/* it is a epoch-by-epoch filter, no need for the cycle slip information here */
					P[SMI(locAmb + np, locAmb + np)] += initCovAMB;
	            }
                else
                    continue;

                ++(*num_of_obs);

                memset(L, 0, sizeof(L));
                memset(H, 0, sizeof(H));

                numl = 0;
                H[numl] = -pVecRov->e[0] / w; L[numl] = 0; ++numl;
                H[numl] = -pVecRov->e[1] / w; L[numl] = 1; ++numl;
                H[numl] = -pVecRov->e[2] / w; L[numl] = 2; ++numl;

                H[numl] = 1.0; L[numl] = locAmb + np; ++numl;
                H[numl] = 1.0; L[numl] = 3; ++numl;

                ekf_measurement_predict(x, P, H, L, &z_, &R_, np + ns, numl, PHt);
                inov = z - z_;
                P_inov = R + R_;
                double covAMB = sqrt(R_ - initCovAMB);
                wsf = (inov*inov) / P_inov;

                {
                    ekf_measurement_update(x, P, H, L, inov, P_inov, np + ns, numl, PHt);
                }
            }
        }
    }

    (*num_s) = ns;
    fixID = 55; // DW DEBUG
#ifndef _USE_AVE_AMBRES_
    /* search ambiguities */
    int namb = 0, nsat = 0;
    double xa[NX_MAX] = { 0.0 };
    double Pa[SMD(NX_MAX)] = { 0.0 };
    fixID = ambres_state_vector(obs_ref, obs_rov, vec_ref, vec_rov, iref, irov, nsd, x, P, tag, &np, &ns, xa, Pa, ambset, NULL, isPrint);

    if (fixID == 4)
    {
        /* re-check the fixed ambiguities using outlier detection */
        if (ambset->ratio < 3.0)
        {
            fixID = ambres_pos(obs_ref, obs_rov, vec_ref, vec_rov, iref, irov, nsd, xyz, covXYZ, num_of_obs, num_of_out, ambset, maskElev, isPrint);
        }
        else
        {
            for (i = 0; i < 3; ++i)
            {
                xyz[i] = xa[i];
                for (j = i; j < 3; ++j)
                {
                    covXYZ[SMI(i, j)] = P[SMI(i, j)];
                }
            }
        }
    }
#else
#ifdef _AMB_RES_
    int namb = 0, nsat = 0;
    double xa[NX_MAX] = { 0.0 };
    double Pa[SMD(NX_MAX)] = { 0.0 };

    fixID = ambres_state_vector_new(obs_ref, obs_rov, vec_ref, vec_rov, iref, irov, nsd, x, P, tag, &np, &ns, xa, Pa, ambset, NULL, isPrint);

    if (fixID == 4)
    {
        /* re-check the fixed ambiguities using outlier detection */
        if (ambset->ratio < 2.0)
        {
            fixID = ambres_pos(obs_ref, obs_rov, vec_ref, vec_rov, iref, irov, nsd, xyz, covXYZ, num_of_obs, num_of_out, ambset, maskElev, isPrint);
        }
        else
        {
            for (i = 0; i < 3; ++i)
            {
                xyz[i] = xa[i];
                for (j = i; j < 3; ++j)
                {
                    covXYZ[SMI(i, j)] = Pa[SMI(i, j)];
                }
            }
        }
        if (isPrint)
        {
            printf("rtk pos:time: %10.1f fixed pos:%14.4f %14.4f %14.4f float pos:%14.4f %14.4f %14.4f\n", time, xyz[0], xyz[1], xyz[2], x[0], x[1], x[2]);

        }
    }
#endif
#endif
    return fixID;
}

extern void blh2C_en(const double *blh, double C_en[3][3])
{
    // blh => C_en
    double lat = blh[0], lon = blh[1]; //, ht = blh[2];
    C_en[0][0] = -sin(lat) * cos(lon);
    C_en[1][0] = -sin(lat) * sin(lon);
    C_en[2][0] = cos(lat);
    C_en[0][1] = -sin(lon);
    C_en[1][1] = cos(lon);
    C_en[2][1] = 0.0;
    C_en[0][2] = -cos(lat) * cos(lon);
    C_en[1][2] = -cos(lat) * sin(lon);
    C_en[2][2] = -sin(lat);
    return;
}

extern void xyz2ned(double C_en[3][3], double *xyz, double *covXYZ, double *ned, double *covNED)
{
    double temp[3][3] = {0};
    int i, j, k;
    ned[0] = C_en[0][0] * xyz[0] + C_en[1][0] * xyz[1] + C_en[2][0] * xyz[2];
    ned[1] = C_en[0][1] * xyz[0] + C_en[1][1] * xyz[1] + C_en[2][1] * xyz[2];
    ned[2] = C_en[0][2] * xyz[0] + C_en[1][2] * xyz[1] + C_en[2][2] * xyz[2];
    if (covXYZ != NULL && covNED != NULL)
    {
        /* covNED = C_en'*covXYZ*C_en */
        for (i = 0; i < 3; ++i)
        {
            for (j = 0; j < 3; ++j)
            {
                temp[i][j] = 0.0;
                for (k = 0; k < 3; ++k)
                {
                    temp[i][j] += C_en[k][i] * covXYZ[SMI(k, j)];
                }
            }
        }

        for (i = 0; i < 3; ++i)
        {
            for (j = 0; j < 3; ++j)
            {
                covNED[SMI(i, j)] = 0.0;
                for (k = 0; k < 3; ++k)
                {
                    covNED[SMI(i, j)] += temp[i][k] * C_en[k][j];
                }
            }
        }
    }
    return;
}

extern void blhdiff(double *blh, double *blh_ref, double *ned)
{
    double C_en[3][3] = {0};
    double xyz[3] = {0};
    double xyz_ref[3] = {0};
    double dxyz[3] = {0};
    blh2C_en(blh_ref, C_en);
    pos2ecef(blh_ref, xyz_ref);
    pos2ecef(blh, xyz);
    dxyz[0] = xyz[0] - xyz_ref[0];
    dxyz[1] = xyz[1] - xyz_ref[1];
    dxyz[2] = xyz[2] - xyz_ref[2];
    ned[0] = C_en[0][0] * dxyz[0] + C_en[1][0] * dxyz[1] + C_en[2][0] * dxyz[2];
    ned[1] = C_en[0][1] * dxyz[0] + C_en[1][1] * dxyz[1] + C_en[2][1] * dxyz[2];
    ned[2] = C_en[0][2] * dxyz[0] + C_en[1][2] * dxyz[1] + C_en[2][2] * dxyz[2];
    return;
}

int rtd_filter(obs_t *rov, obs_t *ref, vec_t *vec_rov, vec_t *vec_ref, int *iref, int *irov, int nsd, 
	rcv_rtk_t *rcv, int *num_of_sat, float *dop, float maskElev, int isSearchAMB, int isPrint)
{
    int fixID = 5;
    *num_of_sat = nsd;
#ifdef _USE_RTD_
    int num_of_obs = 0;
    int num_of_out = 0;
    int wk = (int)floor(rcv->time / SECONDS_IN_WEEK);
    double ws = rcv->time - wk * SECONDS_IN_WEEK;

    filter_RTD_update(rov, vec_rov, rcv->x, rcv->P, &rcv->time, NX_RTD);

    *num_of_sat = filter_RTD(ref, rov, vec_ref, vec_rov, iref, irov, nsd, rcv->x, rcv->P, NX_RTD, 0, &num_of_obs, &num_of_out, dop, maskElev, isPrint);

    // if (isPrint)
    // {
    //     printf("RTD time: %10.1f pos: %14.4f %14.4f %14.4f\n", ws, rov->pos[0], rov->pos[1], rov->pos[2]);
    // }


    if (isSearchAMB == 1) {
        double x[NX_MAX] = { 0.0 };
        double P[SMD(NX_MAX)] = { 0.0 };
        state_tag_t tag[NX_MAX] = { 0 }; 
        ambset_t ambset = { 0 };
        int namb = 0, nsat = 0;
        uint8_t np = 4, ns = 0;
        //double xa[NX_MAX] = { 0.0 };
        //double Pa[SMD(NX_MAX)] = { 0.0 };

        fixID = ambres_RTK(ref, rov, vec_ref, vec_rov, iref, irov, nsd, rcv->x, rcv->P, x, P, tag, &ns, &num_of_obs, &num_of_out, 10.0);

         if (55 == fixID) {
             fixID = ambres_state_vector_new(x, P, tag, &np, &ns, &ambset, &ws);
         }

        if (fixID == 4) {
            /* re-check the fixed ambiguities using outlier detection */
            if (ambset.ratio < 2.0) {
                fixID = ambres_pos(ref, rov, vec_ref, vec_rov, iref, irov, nsd, rcv->x, rcv->P, num_of_obs, num_of_out, &ambset, maskElev, isPrint);
            }
            else {
                for (uint8_t i = 0; i < 3; ++i) {
                    rcv->x[i] = x[i];
                    for (uint8_t j = i; j < 3; ++j) {
                        rcv->P[SMI(i, j)] = P[SMI(i, j)];
                    }
                }
            }
            // if (isPrint)
            // {
            //     printf("rtk pos:time: %10.1f fixed pos:%14.4f %14.4f %14.4f float pos:%14.4f %14.4f %14.4f\n", time, xyz[0], xyz[1], xyz[2], x[0], x[1], x[2]);
            // }

			//rcv->ambset = ambset;
			ambres_RTD(ref, rov, vec_ref, vec_rov, iref, irov, nsd, rcv->x, rcv->P, NX_RTD, 0, &num_of_obs, &num_of_out, &ambset, 10.0, isPrint);
        }


			

    }
    else
    {
#if 0
		fixID = ambres_pos(ref, rov, vec_ref, vec_rov, iref, irov, nsd, xyz, covXYZ, &num_of_obs, &num_of_out, &rcv->ambset, 10.0, isPrint);

		if (fixID == 4)
		{
			rov->pos[0] += xyz[0];
			rov->pos[1] += xyz[1];
			rov->pos[2] += xyz[2];
			num_of_sat = rcv->ambset.nsat;
		}
		else
		{
			memset(&rcv->ambset, 0, sizeof(rcv->ambset));
		}
#endif
    }
#endif
    return fixID;
}

/* the filter for RTK */
int rtk_filter(obs_t *rov, obs_t *ref, vec_t *vec_rov, vec_t *vec_ref, int *iref, int *irov, int nsd, rcv_rtk_t *rcv, int *num_of_sat, double *dop, double maskElev, int isSearchAMB, int isPrint)
{
    /* Yihe, please work on RTK filter here */
    int fixID = 5;
    *num_of_sat = nsd;
#ifndef _USE_RTD_

    int num_of_obs = 0;
    int num_of_out = 0;

    filter_RTD_update(rov, vec_rov, rcv->x, rcv->P, &rcv->time, NX_RTK);

    if (rcv->np < NX_RTD)
        rcv->np = NX_RTD; /* make sure to hold at least RTD */

    *num_of_sat = filter_RTD(ref, rov, vec_ref, vec_rov, iref, irov, nsd, rcv->x, rcv->P, rcv->np, rcv->ns, &num_of_obs, &num_of_out, dop, maskElev, isPrint);
    //return 3;

    if (isPrint)
    {
        printf("RTD time: %10.1f pos: %14.4f %14.4f %14.4f\n", rcv->time, rov->pos[0], rov->pos[1], rov->pos[2]);
    }

    if (!filter_RTK(ref, rov, vec_ref, vec_rov, iref, irov, nsd, rcv->x, rcv->P, rcv->tag, &rcv->np, &rcv->ns, &num_of_obs, &num_of_out, maskElev, isPrint))
    {
        fixID = 50;
        return fixID;
    }

    if (isSearchAMB == 1)
    {
        ambset_t ambset = {0};

        int namb = 0, nsat = 0;
        double xa[NX_RTK] = {0.0};
        double Pa[SMD(NX_RTK)] = {0.0};

#ifdef _USE_AVE_AMBRES_
        fixID = ambres_state_vector_new(ref, rov, nsd, rcv->x, rcv->P, rcv->tag, &rcv->np, &rcv->ns, xa, Pa, &ambset, NULL, isPrint);
#else
        fixID = ambres_state_vector(ref, rov, nsd, rcv->x, rcv->P, rcv->tag, &rcv->np, &rcv->ns, xa, Pa, &ambset, NULL, isPrint);
#endif

        if (fixID == 4)
        {
            /* store the fixed solution */
            for (int i = 0; i < 6; ++i)
            {
                xa[i] += rov->pos[i];
            }
            memcpy(rcv->x_fixed, xa, sizeof(xa));

            //ambres_RTD(ref, rov, vec_ref, vec_rov, iref, irov, nsd, rcv->x, rcv->P, rcv->np, rcv->ns, &num_of_obs, &num_of_out, &ambset, 10.0, isPrint);
        }
    }

#endif
    return fixID;
}

extern int rtk_processor(obs_t *rov, obs_t *ref, nav_t *nav, rcv_rtk_t *rcv, char *gga, char *sol, int isPrint)
{
    vec_t vec_rov[MAXOBS] = {0}, vec_ref[MAXOBS] = {0};
    int week = 0;
    double time = 0.0;

    int iref[MAXOBS] = {0}, irov[MAXOBS] = {0}, nsd = 0, i = 0, j = 0;
    int num_of_sat = 0, fixID = 5;

    float dop[5] = {0.0};

    if (norm(ref->pos, 3) < 0.01)
    {
        /* can not process RTK if no base station coordinate */
        return 0;
    }

    if (norm(rov->pos, 3) < 0.01)
    {
        /* use base station coordinate as a initial */
        memcpy(rov->pos, ref->pos, sizeof(double) * 3);
    }
    if (rov->n < 4)
    {
        /* do not have enough satellites */
        return 0;
    }

    time = time2gpst(rov->time, &week);
    time += week * SECONDS_IN_WEEK;

    if (fabs(time - rcv->time) < 0.001) /* already processed */
        return 0;

    int8_t nsvr = satposs(rov, vec_rov + 0, nav, EPHOPT_BRDC);
    int8_t nsvb = satposs(ref, vec_ref + 0, nav, EPHOPT_BRDC);
    if (!nsvr || !nsvb) {
        return 0;
    }

    int8_t nrv = compute_vector_data(rov, vec_rov);
    int8_t nrb = compute_vector_data(ref, vec_ref);
    if (!nrv || !nrb) {
        return 0;
    }

    nsd = get_match_epoch(ref, rov, vec_ref, vec_rov, iref, irov);

    if (nsd <= 4)
    {
        return 0;
    }

    int isSearchAMB = 0;
    if (floor(time) != floor(rcv->time) || fabs(time - floor(time + 0.5)) < 0.01) /* need two conditions to check the integer second or cross the integer second */
    {
        isSearchAMB = 1;
    }

    rcv->age = (float)timediff(rov->time, ref->time);

#ifdef _USE_SLIP_DETEC_
    // if (rcv->time != 0.0)
    //     epoch_detslp_doppler(rov, time - rcv->time);
#endif

#ifdef _USE_RTD_
    fixID = rtd_filter(rov, ref, vec_rov, vec_ref, iref, irov, nsd, rcv, &num_of_sat, dop, 10.0f, isSearchAMB, isPrint);
#else
    fixID = rtk_filter(rov, ref, vec_rov, vec_ref, iref, irov, nsd, rcv, &num_of_sat, dop, 10.0f, isSearchAMB, isPrint);
#endif

    /* save the coordinate and velocity in the state vector */
    for (i = 0; i < 6; ++i)
    {
        rov->pos[i] += rcv->x[i];
        rcv->x[i] = rov->pos[i];
    }

    if (isPrint)
    {
        printf("rtk pos: %10.4f %10.4f %10.4f fixstus: %2d\n", rcv->x[0], rcv->x[1], rcv->x[2], fixID);
    }

    compute_vector_data(rov, vec_rov);

#ifdef _DEBUG
    if (rcv->numofepoch == 0)
    {
        memcpy(rcv->refxyz, ref->pos, sizeof(double) * 3);
        double refblh[3] = {0};
        ecef2pos(rcv->refxyz, refblh);
        blh2C_en(refblh, rcv->C_en);
    }
    else
    {
    }
    ++rcv->numofepoch;
    if (sol != NULL)
    {
        double blh[3] = {0};
        ecef2pos(rcv->x, blh);
        double dXYZ[3] = {0};
        double covXYZ[SMD(3)] = {0};
        double dNED[3] = {0}, covNED[SMD(3)] = {0};
        /* position in NED */
        for (i = 0; i < 3; ++i)
        {
            dXYZ[i] = rcv->x[i] - rcv->refxyz[i];
            for (j = i; j < 3; ++j)
            {
                covXYZ[SMI(i, j)] = rcv->P[SMI(i, j)];
            }
        }
        xyz2ned(rcv->C_en, dXYZ, covXYZ, dNED, covNED);
        int loc = 0;
		float badCov = sqrt(covNED[SMI(0, 0)]);
		if (isnan(badCov)) {
			printf("NaN number.\n");
			badCov = -0.001f;
		}
        loc += sprintf(sol + loc, "%10.3f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,", time, dNED[0], dNED[1], dNED[2], badCov, sqrt(covNED[SMI(1, 1)]), sqrt(covNED[SMI(2, 2)]));
        /* velocity in NED */
        for (i = 0; i < 3; ++i)
        {
            dXYZ[i] = rcv->x[i + 3];
            for (j = i; j < 3; ++j)
            {
                covXYZ[SMI(i, j)] = rcv->P[SMI(i + 3, j + 3)];
            }
        }
        xyz2ned(rcv->C_en, dXYZ, covXYZ, dNED, covNED);
        loc += sprintf(sol + loc, "%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,", dNED[0], dNED[1], dNED[2], sqrt(covNED[SMI(0, 0)]), sqrt(covNED[SMI(1, 1)]), sqrt(covNED[SMI(2, 2)]));
        /* acceleration in NED */
        for (i = 0; i < 3; ++i)
        {
            dXYZ[i] = rcv->x[i + 6];
            for (j = i; j < 3; ++j)
            {
                covXYZ[SMI(i, j)] = rcv->P[SMI(i + 6, j + 6)];
            }
        }
        xyz2ned(rcv->C_en, dXYZ, covXYZ, dNED, covNED);
		badCov = sqrt(covNED[SMI(2, 2)]);
		if (isnan(badCov)) {
			printf("NaN number.\n");
			badCov = -0.001f;
		}
        loc += sprintf(sol + loc, "%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%3i,%3i,%5.2f,%i\n", dNED[0], dNED[1], dNED[2], sqrt(covNED[SMI(0, 0)]), sqrt(covNED[SMI(1, 1)]), badCov, fixID, num_of_sat, rcv->age, rcv->numofepoch);
    }
#endif
    if (gga != NULL && num_of_sat > 0)
    {
        print_nmea_gga(rov->time, rov->pos, num_of_sat, fixID, dop[2], rcv->age, gga);
    }

    return nsd;
}

#ifdef _USE_PPP_
extern int ppp_processor(obs_t *obs, nav_t *nav, rcv_ppp_t *rcv, int isPrint)
{
    int week = 0;
    double time = 0.0;

    if (norm(obs->pos, 3) < 0.01)
    {
        return 0;
    }

    if (obs->n < 4)
    {
        /* do not have enough satellites */
        return 0;
    }

    time = time2gpst(obs->time, &week);

    pppos(obs, nav, rcv, 1);

    return 0;
}
#endif
