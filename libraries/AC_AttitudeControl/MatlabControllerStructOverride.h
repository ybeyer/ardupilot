#include <AP_Param/AP_Param.h>


#define DEFINED_TYPEDEF_FOR_struct_YGqhU08QAexFPzedAebZQE_

typedef struct {
  AP_Float yaw;
  AP_Float yawrate;
  AP_Float yawacc;
  AP_Float lean;
  AP_Float leanrate;
  AP_Float leanacc;
} struct_YGqhU08QAexFPzedAebZQE;


#define DEFINED_TYPEDEF_FOR_struct_lPIXEb6wJPkuu8Ltl008OH_

typedef struct {
  AP_Float leanmax;
  AP_Float yawratemax;
  AP_Float yawratetc;
  AP_Float leanfreq;
  AP_Float leandamp;
} struct_lPIXEb6wJPkuu8Ltl008OH;


#define DEFINED_TYPEDEF_FOR_struct_wtF2XFOtOELBemGDyRar7G_

typedef struct {
  AP_Float pos;
  AP_Float vel;
  AP_Float acc;
} struct_wtF2XFOtOELBemGDyRar7G;


#define DEFINED_TYPEDEF_FOR_struct_II9hVQ3mcxESIUwfbVrwcH_

typedef struct {
  AP_Float accumax;
  AP_Float accdmax;
  AP_Float veldmax;
  AP_Float velumax;
  AP_Float accxymax;
  AP_Float velxymax;
  AP_Float veltc;
} struct_II9hVQ3mcxESIUwfbVrwcH;


#define DEFINED_TYPEDEF_FOR_struct_b25WOgda3aujgqXgFZVduD_

typedef struct {
  AP_Float amax;
  AP_Float hend;
  AP_Float thrfall;
  AP_Float vloiter;
  AP_Float tslow;
  AP_Float ptchslow;
  AP_Float kacc;
} struct_b25WOgda3aujgqXgFZVduD;


#define DEFINED_TYPEDEF_FOR_struct_ZpRNaqoZiEb6XqsMOrzR0C_

typedef struct {
  AP_Float T;
  AP_Float wprad;
  AP_Float eposmax;
} struct_ZpRNaqoZiEb6XqsMOrzR0C;


#define DEFINED_TYPEDEF_FOR_struct_xa8fSDLeS0DsKnsQui8D6C_

typedef struct {
  AP_Float omega;
  AP_Float D;
} struct_xa8fSDLeS0DsKnsQui8D6C;


#define DEFINED_TYPEDEF_FOR_struct_bD75Rd31DgpMPGwc2srfDC_

typedef struct {
  struct_lPIXEb6wJPkuu8Ltl008OH rm;
  struct_YGqhU08QAexFPzedAebZQE k;
} struct_bD75Rd31DgpMPGwc2srfDC;


#define DEFINED_TYPEDEF_FOR_struct_wRd0ZmNiwsqTMzHFCs46JE_

typedef struct {
  struct_II9hVQ3mcxESIUwfbVrwcH rm;
  struct_wtF2XFOtOELBemGDyRar7G k;
} struct_wRd0ZmNiwsqTMzHFCs46JE;


#define DEFINED_TYPEDEF_FOR_struct_tuPgovHsB2lMDL0qC2CahD_

typedef struct {
  AP_Float m;
  AP_Float ixx;
  AP_Float iyy;
  AP_Float izz;
  AP_Float ixy;
  AP_Float ixz;
  AP_Float iyz;
} struct_tuPgovHsB2lMDL0qC2CahD;


#define DEFINED_TYPEDEF_FOR_struct_p46j2fCGmBuwCvCy3vmiNE_

typedef struct {
  AP_Float k;
  AP_Float d;
  AP_Float x[4];
  AP_Float y[4];
  AP_Float z[4];
  AP_Float a[4];
  AP_Float nx[4];
  AP_Float ny[4];
  AP_Float ip;
  AP_Float kt;
  AP_Float vb;
  AP_Float ri;
} struct_p46j2fCGmBuwCvCy3vmiNE;


#define DEFINED_TYPEDEF_FOR_struct_XfmEeQaS0sQCsf7bqZO5nH_

typedef struct {
  AP_Float min;
  AP_Float max;
} struct_XfmEeQaS0sQCsf7bqZO5nH;


#define DEFINED_TYPEDEF_FOR_struct_3Stra8KiLtcCGtAAmkH73D_

typedef struct {
  AP_Float u_min;
  AP_Float u_max;
  AP_Float u_d;
  AP_Float W_v[4];
  AP_Float W_u[4];
  AP_Float gamma;
  AP_Float i_max;
} struct_3Stra8KiLtcCGtAAmkH73D;


#define DEFINED_TYPEDEF_FOR_struct_CXcaB0melqVXr2gIbzeSsC_

typedef struct {
  struct_3Stra8KiLtcCGtAAmkH73D ca;
  struct_XfmEeQaS0sQCsf7bqZO5nH thr;
  AP_Float rllptch;
  struct_p46j2fCGmBuwCvCy3vmiNE cep;
  struct_tuPgovHsB2lMDL0qC2CahD ceb;
  struct_wRd0ZmNiwsqTMzHFCs46JE psc;
  struct_bD75Rd31DgpMPGwc2srfDC atc;
  AP_Float mtc;
  struct_xa8fSDLeS0DsKnsQui8D6C sflt;
  struct_ZpRNaqoZiEb6XqsMOrzR0C wpnav;
  struct_b25WOgda3aujgqXgFZVduD dive;
} struct_CXcaB0melqVXr2gIbzeSsC;
