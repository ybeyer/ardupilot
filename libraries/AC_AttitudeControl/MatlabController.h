//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: MatlabController.h
//
// Code generated for Simulink model 'ArduCopter_Fast_Descent'.
//
// Model version                  : 1.476
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Sun Jul  6 13:12:46 2025
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM 7
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#ifndef RTW_HEADER_MatlabController_h_
#define RTW_HEADER_MatlabController_h_
#include "rtwtypes.h"
#include <cmath>
#include <string.h>
#ifndef ArduCopter_Fast_Descent_COMMON_INCLUDES_
# define ArduCopter_Fast_Descent_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 // ArduCopter_Fast_Descent_COMMON_INCLUDES_ 

// Macros for accessing real-time model data structure

// user code (top of header file)
#include "MatlabControllerStructOverride.h"
#define MODE_CUSTOM_VAR_INFO
#ifndef DEFINED_TYPEDEF_FOR_cmdBus_
#define DEFINED_TYPEDEF_FOR_cmdBus_

typedef struct {
  real32_T roll;
  real32_T pitch;
  real32_T yaw;
  real32_T thr;
  real32_T s_Kg_init[3];
  real32_T yaw_init;
  uint16_T mission_change;
  real32_T waypoints[40];
  uint16_T num_waypoints;
  real32_T RC_pwm[16];
} cmdBus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_measureBus_
#define DEFINED_TYPEDEF_FOR_measureBus_

typedef struct {
  real32_T omega_Kb[3];
  real32_T EulerAngles[3];
  real32_T q_bg[4];
  real32_T a_Kg[3];
  real32_T a_Kb[3];
  real32_T V_Kg[3];
  real32_T s_Kg[3];
  real32_T s_Kg_origin[3];
  real32_T lla[3];
  real32_T rangefinder[6];
  real32_T V_bat;
  real32_T omega_mot[4];
  real32_T airspeed;
} measureBus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_lindiStateLogicBus_
#define DEFINED_TYPEDEF_FOR_lindiStateLogicBus_

typedef struct {
  boolean_T isPscEnabled;
  boolean_T isPosRmEnabled;
  boolean_T isVertPscEnabled;
  boolean_T isGdnceEnabled;
  boolean_T isAttiCmdEnabled;
  boolean_T isManThrEnabled;
  boolean_T isAutoTuneEnabled;
} lindiStateLogicBus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_logConfigBus_
#define DEFINED_TYPEDEF_FOR_logConfigBus_

typedef struct {
  uint8_T num_signals;
  uint8_T signal_names[42];
  uint8_T batch_name[4];
} logConfigBus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_3Stra8KiLtcCGtAAmkH73D_
#define DEFINED_TYPEDEF_FOR_struct_3Stra8KiLtcCGtAAmkH73D_

typedef struct {
  real32_T u_min;
  real32_T u_max;
  real32_T u_d;
  real32_T W_v[4];
  real32_T W_u[4];
  real32_T gamma;
  real32_T i_max;
} struct_3Stra8KiLtcCGtAAmkH73D;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_XfmEeQaS0sQCsf7bqZO5nH_
#define DEFINED_TYPEDEF_FOR_struct_XfmEeQaS0sQCsf7bqZO5nH_

typedef struct {
  real32_T min;
  real32_T max;
} struct_XfmEeQaS0sQCsf7bqZO5nH;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_p46j2fCGmBuwCvCy3vmiNE_
#define DEFINED_TYPEDEF_FOR_struct_p46j2fCGmBuwCvCy3vmiNE_

typedef struct {
  real32_T k;
  real32_T d;
  real32_T x[4];
  real32_T y[4];
  real32_T z[4];
  real32_T a[4];
  real32_T nx[4];
  real32_T ny[4];
  real32_T ip;
  real32_T kt;
  real32_T vb;
  real32_T ri;
} struct_p46j2fCGmBuwCvCy3vmiNE;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_tuPgovHsB2lMDL0qC2CahD_
#define DEFINED_TYPEDEF_FOR_struct_tuPgovHsB2lMDL0qC2CahD_

typedef struct {
  real32_T m;
  real32_T ixx;
  real32_T iyy;
  real32_T izz;
  real32_T ixy;
  real32_T ixz;
  real32_T iyz;
} struct_tuPgovHsB2lMDL0qC2CahD;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_II9hVQ3mcxESIUwfbVrwcH_
#define DEFINED_TYPEDEF_FOR_struct_II9hVQ3mcxESIUwfbVrwcH_

typedef struct {
  real32_T accumax;
  real32_T accdmax;
  real32_T veldmax;
  real32_T velumax;
  real32_T accxymax;
  real32_T velxymax;
  real32_T veltc;
} struct_II9hVQ3mcxESIUwfbVrwcH;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_wtF2XFOtOELBemGDyRar7G_
#define DEFINED_TYPEDEF_FOR_struct_wtF2XFOtOELBemGDyRar7G_

typedef struct {
  real32_T pos;
  real32_T vel;
  real32_T acc;
} struct_wtF2XFOtOELBemGDyRar7G;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_wRd0ZmNiwsqTMzHFCs46JE_
#define DEFINED_TYPEDEF_FOR_struct_wRd0ZmNiwsqTMzHFCs46JE_

typedef struct {
  struct_II9hVQ3mcxESIUwfbVrwcH rm;
  struct_wtF2XFOtOELBemGDyRar7G k;
} struct_wRd0ZmNiwsqTMzHFCs46JE;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_lPIXEb6wJPkuu8Ltl008OH_
#define DEFINED_TYPEDEF_FOR_struct_lPIXEb6wJPkuu8Ltl008OH_

typedef struct {
  real32_T leanmax;
  real32_T yawratemax;
  real32_T yawratetc;
  real32_T leanfreq;
  real32_T leandamp;
} struct_lPIXEb6wJPkuu8Ltl008OH;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_YGqhU08QAexFPzedAebZQE_
#define DEFINED_TYPEDEF_FOR_struct_YGqhU08QAexFPzedAebZQE_

typedef struct {
  real32_T yaw;
  real32_T yawrate;
  real32_T yawacc;
  real32_T lean;
  real32_T leanrate;
  real32_T leanacc;
} struct_YGqhU08QAexFPzedAebZQE;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_bD75Rd31DgpMPGwc2srfDC_
#define DEFINED_TYPEDEF_FOR_struct_bD75Rd31DgpMPGwc2srfDC_

typedef struct {
  struct_lPIXEb6wJPkuu8Ltl008OH rm;
  struct_YGqhU08QAexFPzedAebZQE k;
} struct_bD75Rd31DgpMPGwc2srfDC;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_xa8fSDLeS0DsKnsQui8D6C_
#define DEFINED_TYPEDEF_FOR_struct_xa8fSDLeS0DsKnsQui8D6C_

typedef struct {
  real32_T omega;
  real32_T D;
} struct_xa8fSDLeS0DsKnsQui8D6C;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_ZpRNaqoZiEb6XqsMOrzR0C_
#define DEFINED_TYPEDEF_FOR_struct_ZpRNaqoZiEb6XqsMOrzR0C_

typedef struct {
  real32_T T;
  real32_T wprad;
  real32_T eposmax;
} struct_ZpRNaqoZiEb6XqsMOrzR0C;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_b25WOgda3aujgqXgFZVduD_
#define DEFINED_TYPEDEF_FOR_struct_b25WOgda3aujgqXgFZVduD_

typedef struct {
  real32_T amax;
  real32_T hend;
  real32_T thrfall;
  real32_T vloiter;
  real32_T tslow;
  real32_T ptchslow;
  real32_T kacc;
} struct_b25WOgda3aujgqXgFZVduD;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_CXcaB0melqVXr2gIbzeSsC_
#define DEFINED_TYPEDEF_FOR_struct_CXcaB0melqVXr2gIbzeSsC_

typedef struct {
  struct_3Stra8KiLtcCGtAAmkH73D ca;
  struct_XfmEeQaS0sQCsf7bqZO5nH thr;
  real32_T rllptch;
  struct_p46j2fCGmBuwCvCy3vmiNE cep;
  struct_tuPgovHsB2lMDL0qC2CahD ceb;
  struct_wRd0ZmNiwsqTMzHFCs46JE psc;
  struct_bD75Rd31DgpMPGwc2srfDC atc;
  real32_T mtc;
  struct_xa8fSDLeS0DsKnsQui8D6C sflt;
  struct_ZpRNaqoZiEb6XqsMOrzR0C wpnav;
  struct_b25WOgda3aujgqXgFZVduD dive;
} struct_CXcaB0melqVXr2gIbzeSsC;

#endif

// Custom Type definition for MATLAB Function: '<S94>/DCM to quaternions'
#ifndef struct_tag_skA4KFEZ4HPkJJBOYCrevdH
#define struct_tag_skA4KFEZ4HPkJJBOYCrevdH

struct tag_skA4KFEZ4HPkJJBOYCrevdH
{
  uint32_T SafeEq;
  uint32_T Absolute;
  uint32_T NaNBias;
  uint32_T NaNWithFinite;
  uint32_T FiniteWithNaN;
  uint32_T NaNWithNaN;
};

#endif                                 //struct_tag_skA4KFEZ4HPkJJBOYCrevdH

#ifndef typedef_skA4KFEZ4HPkJJBOYCrevdH
#define typedef_skA4KFEZ4HPkJJBOYCrevdH

typedef struct tag_skA4KFEZ4HPkJJBOYCrevdH skA4KFEZ4HPkJJBOYCrevdH;

#endif                                 //typedef_skA4KFEZ4HPkJJBOYCrevdH

#ifndef struct_tag_sJCxfmxS8gBOONUZjbjUd9E
#define struct_tag_sJCxfmxS8gBOONUZjbjUd9E

struct tag_sJCxfmxS8gBOONUZjbjUd9E
{
  boolean_T CaseSensitivity;
  boolean_T StructExpand;
  char_T PartialMatching[6];
  boolean_T IgnoreNulls;
};

#endif                                 //struct_tag_sJCxfmxS8gBOONUZjbjUd9E

#ifndef typedef_sJCxfmxS8gBOONUZjbjUd9E
#define typedef_sJCxfmxS8gBOONUZjbjUd9E

typedef struct tag_sJCxfmxS8gBOONUZjbjUd9E sJCxfmxS8gBOONUZjbjUd9E;

#endif                                 //typedef_sJCxfmxS8gBOONUZjbjUd9E

// Block signals and states (default storage) for system '<Root>'
typedef struct {
  real_T NextOutput[4];                // '<S64>/White Noise'
  real32_T Merge[4];                   // '<S3>/Merge'
  real32_T s_g_ref[3];                 // '<S4>/Merge3'
  real32_T s_g_ref_dt[3];              // '<S4>/Merge3'
  real32_T s_g_ref_dt2[3];             // '<S4>/Merge3'
  real32_T n_g_des[3];                 // '<S4>/Merge'
  real32_T Merge2[3];                  // '<S151>/Merge2'
  real32_T Merge_b[3];                 // '<S151>/Merge'
  real32_T Merge1_p[3];                // '<S151>/Merge1'
  real32_T Add[9];                     // '<S135>/Add'
  real32_T nu[3];                      // '<S44>/Add1'
  real32_T s_g_ref_f[3];               // '<S44>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' 
  real32_T s_g[3];                     // '<S44>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' 
  real32_T s_g_dt[3];                  // '<S44>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' 
  real32_T s_g_dt2[3];                 // '<S44>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' 
  real32_T MatrixMultiply3[4];         // '<S66>/Matrix Multiply3'
  real32_T Add3[4];                    // '<S68>/Add3'
  real32_T MatrixMultiply2[4];         // '<S66>/Matrix Multiply2'
  real32_T Delta_nu_measure[4];        // '<S67>/Add2'
  real32_T Gain[3];                    // '<S54>/Gain'
  real32_T Delta_factors_G1[4];        // '<S53>/apply learn rate'
  real32_T Delay_DSTATE[4];            // '<S56>/Delay'
  real32_T DiscreteTimeIntegratory_DSTATE[3];// '<S92>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_DSTAT_a[3];// '<S91>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_DSTA_al[3];// '<S89>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DSTA[3];// '<S93>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTAT_p[9];// '<S90>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_DSTAT_m[3];// '<S123>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DS_j[3];// '<S123>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTA_pz[3];// '<S125>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DS_b[3];// '<S125>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTAT_k[3];// '<S93>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegrator_DSTATE_e[4];// '<S80>/Discrete-Time Integrator' 
  real32_T UnitDelay1_DSTATE_c[4];     // '<S72>/Unit Delay1'
  real32_T DiscreteTimeIntegratory_DSTAT_n[4];// '<S74>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DS_l[4];// '<S74>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_D_bx[9];// '<S90>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_DS_o[3];// '<S89>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_DS_n[3];// '<S91>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_D_nb[3];// '<S92>/Discrete-Time Integrator y_dt' 
  real32_T UnitDelay6_DSTATE[3];       // '<S49>/Unit Delay6'
  real32_T UnitDelay4_DSTATE[15];      // '<S49>/Unit Delay4'
  real32_T DiscreteTimeIntegratory_DSTAT_l[3];// '<S161>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_D_jp[3];// '<S161>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTA_l5[3];// '<S162>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_D_lk[3];// '<S162>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTA_ld[9];// '<S136>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DS_f[9];// '<S136>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegrator1_DSTATE[3];// '<S43>/Discrete-Time Integrator1' 
  real32_T DiscreteTimeIntegrator_DSTATE_c[2];// '<S127>/Discrete-Time Integrator' 
  real32_T DiscreteTimeIntegratory_DSTAT_c[3];// '<S86>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DS_e[3];// '<S86>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTA_lm[4];// '<S70>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DS_p[4];// '<S70>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTA_ak[4];// '<S69>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_D_n3[4];// '<S69>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTAT_d[4];// '<S65>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DS_h[4];// '<S65>/Discrete-Time Integrator y_dt' 
  real32_T UnitDelay_DSTATE[4];        // '<S62>/Unit Delay'
  real32_T Merge1;                     // '<S3>/Merge1'
  real32_T cmd_lean_angle_01;          // '<S4>/Merge'
  real32_T lean_dir_angle_des;         // '<S4>/Merge'
  real32_T Merge1_e;                   // '<S4>/Merge1'
  real32_T Delta_nu_a_T;               // '<S40>/incremental thrust atti correction' 
  real32_T a_T_meas;                   // '<S40>/desired and measured specific thrust' 
  real32_T Gain1;                      // '<S54>/Gain1'
  real32_T Delta_factors_G2;           // '<S53>/apply learn rate2'
  real32_T Merge_a;                    // '<S11>/Merge'
  real32_T Merge1_i;                   // '<S11>/Merge1'
  real32_T y_0;                        // '<S34>/y_0'
  real32_T h_abfang;                   // '<S21>/MATLAB Function'
  real32_T r_abfang;                   // '<S21>/MATLAB Function'
  real32_T uv_g;                       // '<S30>/MATLAB Function'
  real32_T DiscreteTimeIntegrator;     // '<S18>/Discrete-Time Integrator'
  real32_T Delay1_DSTATE;              // '<S57>/Delay1'
  real32_T UnitDelay1_DSTATE;          // '<S46>/Unit Delay1'
  real32_T Delay_DSTATE_b;             // '<S47>/Delay'
  real32_T DiscreteTimeIntegrator_DSTATE;// '<S122>/Discrete-Time Integrator'
  real32_T DiscreteTimeIntegrator2_DSTATE;// '<S103>/Discrete-Time Integrator2'
  real32_T DiscreteTimeIntegrator_DSTATE_n;// '<S128>/Discrete-Time Integrator'
  real32_T UnitDelay3_DSTATE;          // '<S11>/Unit Delay3'
  real32_T UnitDelay5_DSTATE;          // '<S11>/Unit Delay5'
  real32_T DiscreteTimeIntegrator_DSTAT_ng;// '<S12>/Discrete-Time Integrator'
  real32_T DiscreteTimeIntegratory_DSTAT_h;// '<S35>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegrator_DSTATE_m;// '<S34>/Discrete-Time Integrator'
  real32_T DiscreteTimeIntegratory_dt_D_fu;// '<S35>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegrator_DSTAT_mf;// '<S18>/Discrete-Time Integrator'
  int32_T UnitDelay_DSTATE_o;          // '<S49>/Unit Delay'
  int32_T UnitDelay1_DSTATE_f;         // '<S49>/Unit Delay1'
  int32_T UnitDelay2_DSTATE;           // '<S49>/Unit Delay2'
  int32_T UnitDelay3_DSTATE_m;         // '<S49>/Unit Delay3'
  uint32_T RandSeed[4];                // '<S64>/White Noise'
  struct {
    void *LoggedData[2];
  } Scope4_PWORK;                      // '<S11>/Scope4'

  struct {
    void *LoggedData[2];
  } Scope_PWORK;                       // '<S23>/Scope'

  struct {
    void *LoggedData;
  } Scope3_PWORK;                      // '<S11>/Scope3'

  struct {
    void *LoggedData;
  } Scope5_PWORK;                      // '<S11>/Scope5'

  int8_T DiscreteTimeIntegratory_PrevR_a;// '<S136>/Discrete-Time Integrator y'
  int8_T DiscreteTimeIntegratory_dt_P_mk;// '<S136>/Discrete-Time Integrator y_dt' 
  int8_T DiscreteTimeIntegrator1_PrevRes;// '<S43>/Discrete-Time Integrator1'
  int8_T DiscreteTimeIntegrator_PrevRe_m;// '<S127>/Discrete-Time Integrator'
  int8_T DiscreteTimeIntegrator_PrevRe_i;// '<S128>/Discrete-Time Integrator'
  int8_T DiscreteTimeIntegrator_PrevRe_g;// '<S12>/Discrete-Time Integrator'
  int8_T DiscreteTimeIntegrator_PrevRe_c;// '<S18>/Discrete-Time Integrator'
  uint8_T DiscreteTimeIntegratory_IC_LOAD;// '<S92>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_LO_p;// '<S91>/Discrete-Time Integrator y'
  uint8_T icLoad;                      // '<S47>/Delay'
  uint8_T DiscreteTimeIntegratory_IC_LO_a;// '<S90>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_LO_b;// '<S123>/Discrete-Time Integrator y' 
  uint8_T DiscreteTimeIntegratory_IC_LO_j;// '<S125>/Discrete-Time Integrator y' 
  uint8_T DiscreteTimeIntegratory_IC_L_jp;// '<S93>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegrator2_IC_LOAD;// '<S103>/Discrete-Time Integrator2'
  uint8_T DiscreteTimeIntegratory_IC_L_ai;// '<S161>/Discrete-Time Integrator y' 
  uint8_T DiscreteTimeIntegratory_IC_LO_d;// '<S162>/Discrete-Time Integrator y' 
  uint8_T DiscreteTimeIntegratory_IC_LO_h;// '<S136>/Discrete-Time Integrator y' 
  uint8_T DiscreteTimeIntegrator1_IC_LOAD;// '<S43>/Discrete-Time Integrator1'
  uint8_T DiscreteTimeIntegrator_IC_LOADI;// '<S127>/Discrete-Time Integrator'
  uint8_T DiscreteTimeIntegrator_IC_LOA_a;// '<S128>/Discrete-Time Integrator'
  uint8_T DiscreteTimeIntegratory_IC_LO_e;// '<S86>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_LO_f;// '<S70>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_L_p0;// '<S69>/Discrete-Time Integrator y'
  boolean_T UnitDelay5_DSTATE_n;       // '<S49>/Unit Delay5'
  boolean_T UnitDelay_DSTATE_b;        // '<S11>/Unit Delay'
  boolean_T UnitDelay4_DSTATE_d;       // '<S12>/Unit Delay4'
  boolean_T UnitDelay1_DSTATE_h;       // '<S11>/Unit Delay1'
  boolean_T UnitDelay2_DSTATE_f;       // '<S11>/Unit Delay2'
  boolean_T WaypointNavigation_MODE;   // '<S4>/Waypoint Navigation'
  boolean_T FlightPathSmoothing_MODE;  // '<S151>/Flight Path Smoothing'
  boolean_T NDIpositioncontrollerforcopters;// '<S4>/NDI position controller for copters with reference input' 
  boolean_T NDIpositioncontrollerforcopte_k;// '<S4>/NDI position controller for copters reference model' 
  boolean_T CopterRandomExcitation_MODE;// '<S52>/Copter Random Excitation'
  boolean_T AdaptiveINDIG1andG2correction_M;// '<S52>/Adaptive INDI G1 and G2 correction' 
  boolean_T fastdescentsequencer_MODE; // '<S3>/fast descent sequencer'
  boolean_T ThrottleInterceptArcLoadfactorc;// '<S11>/Throttle Intercept Arc (Load factor controller)' 
  boolean_T Subsystem2_MODE;           // '<S11>/Subsystem2'
  boolean_T Durationinslowing_MODE;    // '<S11>/Duration in slowing'
  boolean_T DesiredLeanInterceptArc_MODE;// '<S11>/Desired Lean Intercept Arc'
} DW;

// Constant parameters (default storage)
typedef struct {
  // Expression: G10
  //  Referenced by: '<S75>/MATLAB Function'

  real32_T MATLABFunction_G10[16];

  // Expression: G20
  //  Referenced by: '<S75>/MATLAB Function'

  real32_T MATLABFunction_G20[16];
} ConstP;

// External inputs (root inport signals with default storage)
typedef struct {
  cmdBus cmd;                          // '<Root>/cmd'
  measureBus measure;                  // '<Root>/measure'
} ExtU;

// External outputs (root outports fed by signals with default storage)
typedef struct {
  real32_T u[8];                       // '<Root>/u'
  real32_T logs[56];                   // '<Root>/logs'
} ExtY;

// Parameters (default storage)
struct P_ {
  struct_CXcaB0melqVXr2gIbzeSsC lindi; // Variable: lindi
                                       //  Referenced by:
                                       //    '<S11>/Constant'
                                       //    '<S11>/t_slowing_max'
                                       //    '<S11>/v_loiter'
                                       //    '<S37>/Gain1'
                                       //    '<S37>/Saturation1'
                                       //    '<S39>/Control Allocation Vertical Acc Weighting'
                                       //    '<S39>/Saturation3'
                                       //    '<S43>/velxymax'
                                       //    '<S45>/Gain1'
                                       //    '<S47>/throttle_-1_1 to throttle_0_1'
                                       //    '<S47>/Constant'
                                       //    '<S48>/Constant'
                                       //    '<S49>/Constant2'
                                       //    '<S49>/Constant3'
                                       //    '<S49>/Constant5'
                                       //    '<S13>/Gain2'
                                       //    '<S17>/Constant'
                                       //    '<S21>/Constant1'
                                       //    '<S21>/a_max'
                                       //    '<S22>/throttle decent'
                                       //    '<S23>/Gain4'
                                       //    '<S72>/Set Desired Motor Command'
                                       //    '<S72>/Set Vertical Acc Weight To Zero'
                                       //    '<S74>/d'
                                       //    '<S74>/omega'
                                       //    '<S75>/MATLAB Function'
                                       //    '<S75>/Constant1'
                                       //    '<S83>/d'
                                       //    '<S83>/omega'
                                       //    '<S89>/d'
                                       //    '<S89>/omega'
                                       //    '<S90>/d'
                                       //    '<S90>/omega'
                                       //    '<S91>/d'
                                       //    '<S91>/omega'
                                       //    '<S92>/d'
                                       //    '<S92>/omega'
                                       //    '<S93>/d'
                                       //    '<S93>/omega'
                                       //    '<S100>/Gain1'
                                       //    '<S100>/Gain2'
                                       //    '<S100>/Gain3'
                                       //    '<S100>/Gain4'
                                       //    '<S100>/Gain5'
                                       //    '<S100>/Gain6'
                                       //    '<S103>/r_max'
                                       //    '<S127>/T'
                                       //    '<S127>/Saturation'
                                       //    '<S128>/T'
                                       //    '<S128>/Saturation'
                                       //    '<S130>/MATLAB Function'
                                       //    '<S135>/acccntrlmax'
                                       //    '<S135>/eposmax'
                                       //    '<S135>/Constant'
                                       //    '<S135>/Gain'
                                       //    '<S135>/Gain3'
                                       //    '<S135>/Gain4'
                                       //    '<S151>/Constant'
                                       //    '<S155>/wp_rad_fix'
                                       //    '<S34>/T'
                                       //    '<S34>/y_0'
                                       //    '<S53>/G1 learn rate'
                                       //    '<S53>/G2 learn rate'
                                       //    '<S53>/Constant10'
                                       //    '<S54>/Constant3'
                                       //    '<S54>/Gain1'
                                       //    '<S76>/caIndiWls'
                                       //    '<S76>/Delta u_max'
                                       //    '<S80>/T'
                                       //    '<S111>/leanmax'
                                       //    '<S117>/d'
                                       //    '<S117>/omega'
                                       //    '<S118>/omega'
                                       //    '<S158>/Constant'
                                       //    '<S122>/T'
                                       //    '<S69>/omega'
                                       //    '<S70>/omega'

};

// Parameters (default storage)
typedef struct P_ P;

// Constant parameters (default storage)
extern const ConstP rtConstP;

// Exported data declaration

// Const memory section
// Declaration for custom storage class: Const
extern const logConfigBus log_config[5];

// Class declaration for model ArduCopter_Fast_Descent
class MatlabControllerClass {
  // public data and function members
 public:
  static const struct AP_Param::GroupInfo var_info_rtP_lindi_ca[];
  static const struct AP_Param::GroupInfo var_info_rtP_lindi_thr[];
  static const struct AP_Param::GroupInfo var_info_rtP_lindi_cep[];
  static const struct AP_Param::GroupInfo var_info_rtP_lindi_ceb[];
  static const struct AP_Param::GroupInfo var_info_rtP_lindi_psc[];
  static const struct AP_Param::GroupInfo var_info_rtP_lindi_atc[];
  static const struct AP_Param::GroupInfo var_info_rtP_lindi_sflt[];
  static const struct AP_Param::GroupInfo var_info_rtP_lindi_wpnav[];
  static const struct AP_Param::GroupInfo var_info_rtP_lindi_dive[];
  static const struct AP_Param::GroupInfo var_info[];

  // External inputs
  ExtU rtU;

  // External outputs
  ExtY rtY;

  // model initialize function
  void initialize();

  // model step function
  void step();

  // Constructor
  MatlabControllerClass();

  // Destructor
  ~MatlabControllerClass();

  // private data and function members
 private:
  // Tunable parameters
  P rtP;

  // Block signals and states
  DW rtDW;

  // private member function(s) for subsystem '<Root>'
  real32_T norm(const real32_T x[3]);
  void wpnavCircSeg(const real32_T waypoints3x3[9], real32_T wp_radius, real32_T
                    *circ_seg_r, real32_T circ_seg_center[3], real32_T
                    circ_seg_n[3], real32_T *circ_seg_angle, real32_T
                    circ_seg_start[3], real32_T circ_seg_end[3], real32_T
                    *circ_seg_wp_rad, real32_T circ_seg_wp[3]);
  void axisAngle(const real32_T v[3], real32_T axis[3], real32_T angle, real32_T
                 v_rot[3]);
  void wpnavMatchCircSeg(const real32_T circ_seg_center[3], const real32_T
    circ_seg_n[3], real32_T circ_seg_angle, const real32_T circ_seg_start[3],
    const real32_T circ_seg_wp[3], const real32_T p[3], real32_T p_match[3],
    real32_T *t, real32_T *d);
  void wpnavMatchLine(const real32_T p1[3], const real32_T p2[3], const real32_T
                      p[3], real32_T p_match[3], real32_T *t, real32_T *d);
  void wpnavMatch_b(const real32_T waypoints_data[], const int32_T
                    waypoints_size[2], real32_T wp_radius, int32_T *wp_idx,
                    int32_T *stage, const real32_T p[3], real32_T p_match[3],
                    real32_T *t, real32_T *d);
  void wpnavMatch(const real32_T waypoints[15], real32_T wp_radius, int32_T
                  *wp_idx, int32_T *stage, const real32_T p[3], real32_T
                  p_match[3], real32_T *t, real32_T *d);
  void LSQFromQR(const real32_T A_data[], const int32_T A_size[2], const
                 real32_T tau_data[], const int32_T jpvt_data[], real32_T B_3[8],
                 int32_T rankA, real32_T Y_data[], int32_T *Y_size);
  real32_T xnrm2(int32_T n, const real32_T x_data[], int32_T ix0);
  void xzlarf(int32_T m, int32_T n, int32_T iv0, real32_T tau, real32_T C_data[],
              int32_T ic0, real32_T work_data[]);
  void qrsolve(const real32_T A_data[], const int32_T A_size[2], const real32_T
               B_1[8], real32_T Y_data[], int32_T *Y_size);
  void mldivide(const real32_T A_data[], const int32_T A_size[2], const real32_T
                B_0[8], real32_T Y_data[], int32_T *Y_size);
  boolean_T any(const boolean_T x_data[], const int32_T *x_size);
  real32_T wls_alloc(const real32_T B_4[16], const real32_T v[4], const real32_T
                     umin[4], const real32_T umax[4], const real32_T Wv[16],
                     const real32_T Wu[16], const real32_T ud[4], real32_T gam,
                     real32_T u[4], real32_T W[4], real32_T imax);
  real32_T mean(const real32_T x[4]);
};

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<Root>/Constant2' : Unused code path elimination
//  Block '<S11>/Scope' : Unused code path elimination
//  Block '<S11>/Scope1' : Unused code path elimination
//  Block '<S11>/Scope2' : Unused code path elimination
//  Block '<S20>/Scope3' : Unused code path elimination
//  Block '<S23>/Scope1' : Unused code path elimination
//  Block '<S23>/Scope2' : Unused code path elimination
//  Block '<S11>/altitude' : Unused code path elimination
//  Block '<S11>/status' : Unused code path elimination
//  Block '<S66>/Add' : Unused code path elimination
//  Block '<S66>/Add1' : Unused code path elimination
//  Block '<S66>/Add2' : Unused code path elimination
//  Block '<S66>/Matrix Multiply' : Unused code path elimination
//  Block '<S66>/Matrix Multiply1' : Unused code path elimination
//  Block '<S66>/Scope' : Unused code path elimination
//  Block '<S66>/Scope1' : Unused code path elimination
//  Block '<S66>/Scope2' : Unused code path elimination
//  Block '<S66>/Unit Delay' : Unused code path elimination
//  Block '<S68>/Constant' : Unused code path elimination
//  Block '<S68>/Scope' : Unused code path elimination
//  Block '<S4>/Constant' : Unused code path elimination
//  Block '<S39>/Gain' : Unused code path elimination
//  Block '<S44>/Gain' : Unused code path elimination
//  Block '<S44>/Gain1' : Unused code path elimination
//  Block '<S137>/Data Type Duplicate' : Unused code path elimination
//  Block '<S137>/Data Type Propagation' : Unused code path elimination
//  Block '<S138>/Data Type Duplicate' : Unused code path elimination
//  Block '<S138>/Data Type Propagation' : Unused code path elimination
//  Block '<S49>/Data Type Conversion8' : Unused code path elimination
//  Block '<S49>/Data Type Conversion9' : Unused code path elimination
//  Block '<S160>/Discrete-Time Integrator y' : Unused code path elimination
//  Block '<S160>/Discrete-Time Integrator y_dt' : Unused code path elimination
//  Block '<S160>/Divide' : Unused code path elimination
//  Block '<S160>/Gain' : Unused code path elimination
//  Block '<S160>/Product1' : Unused code path elimination
//  Block '<S160>/Product2' : Unused code path elimination
//  Block '<S160>/Saturation' : Unused code path elimination
//  Block '<S160>/Sum2' : Unused code path elimination
//  Block '<S160>/Sum3' : Unused code path elimination
//  Block '<S160>/omega^2' : Unused code path elimination
//  Block '<S160>/y_dt_0' : Unused code path elimination
//  Block '<S158>/Scope' : Unused code path elimination
//  Block '<S158>/Scope1' : Unused code path elimination
//  Block '<S158>/Scope2' : Unused code path elimination
//  Block '<S158>/Scope3' : Unused code path elimination
//  Block '<S49>/Scope1' : Unused code path elimination
//  Block '<S34>/Saturation' : Eliminated Saturate block
//  Block '<S35>/Saturation' : Eliminated Saturate block
//  Block '<Root>/Gain7' : Eliminated nontunable gain of 1
//  Block '<S53>/Reshape3' : Reshape block reduction
//  Block '<S65>/Saturation' : Eliminated Saturate block
//  Block '<S69>/Saturation' : Eliminated Saturate block
//  Block '<S70>/Saturation' : Eliminated Saturate block
//  Block '<S80>/Saturation' : Eliminated Saturate block
//  Block '<S74>/Saturation' : Eliminated Saturate block
//  Block '<S86>/Saturation' : Eliminated Saturate block
//  Block '<S89>/Saturation' : Eliminated Saturate block
//  Block '<S90>/Saturation' : Eliminated Saturate block
//  Block '<S91>/Saturation' : Eliminated Saturate block
//  Block '<S92>/Saturation' : Eliminated Saturate block
//  Block '<S93>/Saturation' : Eliminated Saturate block
//  Block '<S42>/Reshape' : Reshape block reduction
//  Block '<S42>/Reshape1' : Reshape block reduction
//  Block '<S42>/Reshape2' : Reshape block reduction
//  Block '<S103>/Gain1' : Eliminated nontunable gain of 1
//  Block '<S103>/Gain2' : Eliminated nontunable gain of 1
//  Block '<S122>/Saturation' : Eliminated Saturate block
//  Block '<S123>/Saturation' : Eliminated Saturate block
//  Block '<S125>/Saturation' : Eliminated Saturate block
//  Block '<S103>/Reshape' : Reshape block reduction
//  Block '<S136>/Saturation' : Eliminated Saturate block
//  Block '<S49>/Data Type Conversion' : Eliminate redundant data type conversion
//  Block '<S49>/Data Type Conversion1' : Eliminate redundant data type conversion
//  Block '<S49>/Data Type Conversion3' : Eliminate redundant data type conversion
//  Block '<S49>/Data Type Conversion4' : Eliminate redundant data type conversion
//  Block '<S49>/Data Type Conversion5' : Eliminate redundant data type conversion
//  Block '<S161>/Saturation' : Eliminated Saturate block
//  Block '<S162>/Saturation' : Eliminated Saturate block
//  Block '<S39>/Constant' : Unused code path elimination
//  Block '<S155>/Constant' : Unused code path elimination
//  Block '<S155>/Constant1' : Unused code path elimination
//  Block '<S155>/Divide' : Unused code path elimination
//  Block '<S155>/Max' : Unused code path elimination
//  Block '<S155>/Square' : Unused code path elimination
//  Block '<S155>/wp_rad_min' : Unused code path elimination


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'ArduCopter_Fast_Descent'
//  '<S1>'   : 'ArduCopter_Fast_Descent/Actuator muxer'
//  '<S2>'   : 'ArduCopter_Fast_Descent/Compare To Constant'
//  '<S3>'   : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1'
//  '<S4>'   : 'ArduCopter_Fast_Descent/LindiCopter Autopilot'
//  '<S5>'   : 'ArduCopter_Fast_Descent/MATLAB Function'
//  '<S6>'   : 'ArduCopter_Fast_Descent/MATLAB Function1'
//  '<S7>'   : 'ArduCopter_Fast_Descent/Quaternions to Rotation Matrix'
//  '<S8>'   : 'ArduCopter_Fast_Descent/log muxer'
//  '<S9>'   : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/bypass flightmode'
//  '<S10>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/bypass stick commands'
//  '<S11>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer'
//  '<S12>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Arc Length'
//  '<S13>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Convert to rpyt'
//  '<S14>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Desired Lean Descent'
//  '<S15>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Desired Lean Intercept Arc'
//  '<S16>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Desired Lean finished'
//  '<S17>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Desired Lean slowing'
//  '<S18>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Duration in slowing'
//  '<S19>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Mission Status'
//  '<S20>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Speed'
//  '<S21>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Subsystem2'
//  '<S22>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Throttle Descent'
//  '<S23>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Throttle Intercept Arc (Load factor controller)'
//  '<S24>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Throttle finished'
//  '<S25>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Throttle slowing'
//  '<S26>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Convert to rpyt/MATLAB Function'
//  '<S27>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Desired Lean Intercept Arc/MATLAB Function6'
//  '<S28>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Speed/Compare To Constant'
//  '<S29>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Speed/MATLAB Function'
//  '<S30>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Speed/Subsystem'
//  '<S31>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Speed/Subsystem/MATLAB Function'
//  '<S32>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Subsystem2/MATLAB Function'
//  '<S33>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Throttle Intercept Arc (Load factor controller)/MATLAB Function'
//  '<S34>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Throttle Intercept Arc (Load factor controller)/PT1 discrete with saturations'
//  '<S35>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Throttle Intercept Arc (Load factor controller)/PT2 discrete with saturation'
//  '<S36>'  : 'ArduCopter_Fast_Descent/Fast Descent Sequencer1/fast descent sequencer/Throttle Intercept Arc (Load factor controller)/measured specific thrust'
//  '<S37>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Accelerations to Reduced Attitude and Thrust'
//  '<S38>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Autotune'
//  '<S39>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Copter Inner Loop INDI and Control Allocation'
//  '<S40>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Incremental specific thrust'
//  '<S41>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Measurement Filtering'
//  '<S42>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Multicopter (Reduced) Attitude INDI Controller'
//  '<S43>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/NDI position controller for copters reference model'
//  '<S44>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/NDI position controller for copters with reference input'
//  '<S45>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Pilot Reduced Attitude Commands'
//  '<S46>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/State Logic Bus'
//  '<S47>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Stick Command Bus'
//  '<S48>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Vertical Acc to Specific Thrust'
//  '<S49>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Waypoint Navigation'
//  '<S50>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Accelerations to Reduced Attitude and Thrust/INDI Copter Acc 2 Lean Vector'
//  '<S51>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Accelerations to Reduced Attitude and Thrust/MATLAB Function4'
//  '<S52>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Autotune/Copter INDI Autotune'
//  '<S53>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Autotune/Copter INDI Autotune/Adaptive INDI G1 and G2 correction'
//  '<S54>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Autotune/Copter INDI Autotune/Copter Random Excitation'
//  '<S55>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Autotune/Copter INDI Autotune/INDI Inversion Check'
//  '<S56>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Autotune/Copter INDI Autotune/accumulate G1 correction factors'
//  '<S57>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Autotune/Copter INDI Autotune/accumulate G2 correction factor'
//  '<S58>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Autotune/Copter INDI Autotune/create diag'
//  '<S59>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Autotune/Copter INDI Autotune/Adaptive INDI G1 and G2 correction/G1 learn rate'
//  '<S60>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Autotune/Copter INDI Autotune/Adaptive INDI G1 and G2 correction/G2 learn rate'
//  '<S61>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Autotune/Copter INDI Autotune/Adaptive INDI G1 and G2 correction/Gradient of Delta_nu w.r.t. G1 correction factors'
//  '<S62>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Autotune/Copter INDI Autotune/Adaptive INDI G1 and G2 correction/Gradient of Delta_nu w.r.t. G2 correction factor'
//  '<S63>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Autotune/Copter INDI Autotune/Adaptive INDI G1 and G2 correction/Inversion error'
//  '<S64>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Autotune/Copter INDI Autotune/Copter Random Excitation/Band-Limited White Noise'
//  '<S65>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Autotune/Copter INDI Autotune/Copter Random Excitation/PT2 discrete with saturation'
//  '<S66>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Autotune/Copter INDI Autotune/INDI Inversion Check/Inversion forward'
//  '<S67>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Autotune/Copter INDI Autotune/INDI Inversion Check/filtered Delta nu'
//  '<S68>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Autotune/Copter INDI Autotune/INDI Inversion Check/filtered Delta u'
//  '<S69>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Autotune/Copter INDI Autotune/INDI Inversion Check/filtered Delta nu/PT2 discrete with saturation1'
//  '<S70>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Autotune/Copter INDI Autotune/INDI Inversion Check/filtered Delta u/PT2 discrete with saturation'
//  '<S71>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Copter Inner Loop INDI and Control Allocation/Control Allocation Vertical Acc Weighting'
//  '<S72>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Copter Inner Loop INDI and Control Allocation/INDI high level wls control allocation'
//  '<S73>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Copter Inner Loop INDI and Control Allocation/Motor dynamics model'
//  '<S74>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Copter Inner Loop INDI and Control Allocation/Sensor filter model'
//  '<S75>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Copter Inner Loop INDI and Control Allocation/control effectiveness'
//  '<S76>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Copter Inner Loop INDI and Control Allocation/INDI high level wls control allocation/INDI control allocation'
//  '<S77>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Copter Inner Loop INDI and Control Allocation/INDI high level wls control allocation/Set Desired Motor Command'
//  '<S78>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Copter Inner Loop INDI and Control Allocation/INDI high level wls control allocation/Set Vertical Acc Weight To Zero'
//  '<S79>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Copter Inner Loop INDI and Control Allocation/INDI high level wls control allocation/INDI control allocation/caIndiWls'
//  '<S80>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Copter Inner Loop INDI and Control Allocation/Motor dynamics model/PT1 discrete with saturations'
//  '<S81>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Copter Inner Loop INDI and Control Allocation/control effectiveness/MATLAB Function'
//  '<S82>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Incremental specific thrust/DCM 2 Lean Vector'
//  '<S83>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Incremental specific thrust/PT2 Lean Vector'
//  '<S84>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Incremental specific thrust/desired and measured specific thrust'
//  '<S85>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Incremental specific thrust/incremental thrust atti correction'
//  '<S86>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Incremental specific thrust/PT2 Lean Vector/PT2 discrete with saturation'
//  '<S87>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Incremental specific thrust/PT2 Lean Vector/n ref norm'
//  '<S88>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Measurement Filtering/MATLAB Function'
//  '<S89>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Measurement Filtering/PT2 discrete with saturation'
//  '<S90>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Measurement Filtering/PT2 discrete with saturation1'
//  '<S91>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Measurement Filtering/PT2 discrete with saturation2'
//  '<S92>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Measurement Filtering/PT2 discrete with saturation3'
//  '<S93>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Measurement Filtering/PT2 discrete with saturation4'
//  '<S94>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Measurement Filtering/measured yaw'
//  '<S95>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Measurement Filtering/measured yaw/DCM to quaternions'
//  '<S96>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Measurement Filtering/measured yaw/Quaternion Reduced'
//  '<S97>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Multicopter (Reduced) Attitude INDI Controller/Reduced Attitude Scheduler'
//  '<S98>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Multicopter (Reduced) Attitude INDI Controller/error computation'
//  '<S99>'  : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Multicopter (Reduced) Attitude INDI Controller/measure'
//  '<S100>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Multicopter (Reduced) Attitude INDI Controller/ny control'
//  '<S101>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Multicopter (Reduced) Attitude INDI Controller/ny from reference'
//  '<S102>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Multicopter (Reduced) Attitude INDI Controller/ny measured'
//  '<S103>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Multicopter (Reduced) Attitude INDI Controller/reference model'
//  '<S104>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Multicopter (Reduced) Attitude INDI Controller/Reduced Attitude Scheduler/Reduced Attitude Weighting Factors'
//  '<S105>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Multicopter (Reduced) Attitude INDI Controller/error computation/angle error'
//  '<S106>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Multicopter (Reduced) Attitude INDI Controller/error computation/wrap angle'
//  '<S107>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Multicopter (Reduced) Attitude INDI Controller/error computation/wrap angle1'
//  '<S108>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Multicopter (Reduced) Attitude INDI Controller/measure/DCM to quaternions'
//  '<S109>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Multicopter (Reduced) Attitude INDI Controller/measure/Quaternion Reduced'
//  '<S110>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Multicopter (Reduced) Attitude INDI Controller/reference model/Desired Roll Pitch'
//  '<S111>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Multicopter (Reduced) Attitude INDI Controller/reference model/Lean Angle Limiter'
//  '<S112>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Multicopter (Reduced) Attitude INDI Controller/reference model/Lean Vector Derivative Trafo'
//  '<S113>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Multicopter (Reduced) Attitude INDI Controller/reference model/Lean Vector Derivative Trafo Delay'
//  '<S114>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Multicopter (Reduced) Attitude INDI Controller/reference model/Lean Vector Derivative to Omega'
//  '<S115>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Multicopter (Reduced) Attitude INDI Controller/reference model/MATLAB Function'
//  '<S116>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Multicopter (Reduced) Attitude INDI Controller/reference model/PT1 discrete reference model2'
//  '<S117>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Multicopter (Reduced) Attitude INDI Controller/reference model/PT2 Lean Vector'
//  '<S118>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Multicopter (Reduced) Attitude INDI Controller/reference model/PT2 Lean Vector1'
//  '<S119>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Multicopter (Reduced) Attitude INDI Controller/reference model/Pseudo-Control Roll Pitch'
//  '<S120>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Multicopter (Reduced) Attitude INDI Controller/reference model/Simulink Trickster'
//  '<S121>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Multicopter (Reduced) Attitude INDI Controller/reference model/lean angles 2 lean vector'
//  '<S122>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Multicopter (Reduced) Attitude INDI Controller/reference model/PT1 discrete reference model2/PT1 discrete with saturations'
//  '<S123>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Multicopter (Reduced) Attitude INDI Controller/reference model/PT2 Lean Vector/PT2 discrete with saturation'
//  '<S124>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Multicopter (Reduced) Attitude INDI Controller/reference model/PT2 Lean Vector/n ref norm'
//  '<S125>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Multicopter (Reduced) Attitude INDI Controller/reference model/PT2 Lean Vector1/PT2 discrete with saturation'
//  '<S126>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Multicopter (Reduced) Attitude INDI Controller/reference model/PT2 Lean Vector1/n ref norm'
//  '<S127>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/NDI position controller for copters reference model/PT1 discrete with saturation horizontal'
//  '<S128>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/NDI position controller for copters reference model/PT1 discrete with saturation vertical'
//  '<S129>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/NDI position controller for copters reference model/Stick RollPitch to limited RollPitch'
//  '<S130>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/NDI position controller for copters reference model/cmd_throttle to vertical_velocity'
//  '<S131>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/NDI position controller for copters reference model/Stick RollPitch to limited RollPitch/Stick RollPitch to Lean Command'
//  '<S132>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/NDI position controller for copters reference model/Stick RollPitch to limited RollPitch/Stick RollPitch to Lean Command/stickRP2LeanCmd'
//  '<S133>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/NDI position controller for copters reference model/cmd_throttle to vertical_velocity/MATLAB Function'
//  '<S134>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/NDI position controller for copters with reference input/measures'
//  '<S135>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/NDI position controller for copters with reference input/position controller'
//  '<S136>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/NDI position controller for copters with reference input/position controller/PT2 discrete with saturation1'
//  '<S137>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/NDI position controller for copters with reference input/position controller/Saturation Dynamic'
//  '<S138>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/NDI position controller for copters with reference input/position controller/Saturation Dynamic1'
//  '<S139>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/NDI position controller for copters with reference input/position controller/acccntrlmax'
//  '<S140>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/NDI position controller for copters with reference input/position controller/eposmax'
//  '<S141>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Pilot Reduced Attitude Commands/MATLAB Function'
//  '<S142>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Pilot Reduced Attitude Commands/MATLAB Function1'
//  '<S143>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/State Logic Bus/Compare To Zero'
//  '<S144>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/State Logic Bus/LindiCopter State Logic'
//  '<S145>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Stick Command Bus/MATLAB Function'
//  '<S146>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Stick Command Bus/MATLAB Function1'
//  '<S147>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Stick Command Bus/Rotations matrix to Euler angles'
//  '<S148>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Stick Command Bus/throttle_-1_1 to throttle_0_1'
//  '<S149>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Vertical Acc to Specific Thrust/MATLAB Function'
//  '<S150>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Waypoint Navigation/Avoid zero speed'
//  '<S151>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Waypoint Navigation/Flight Path Smoothing'
//  '<S152>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Waypoint Navigation/Look Ahead'
//  '<S153>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Waypoint Navigation/Look Ahead1'
//  '<S154>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Waypoint Navigation/Split waypoints and velocity'
//  '<S155>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Waypoint Navigation/Waypoint radius'
//  '<S156>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Waypoint Navigation/WpNav Matching'
//  '<S157>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Waypoint Navigation/Flight Path Smoothing/Compare To Constant'
//  '<S158>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Waypoint Navigation/Flight Path Smoothing/Flight Path Smoothing'
//  '<S159>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Waypoint Navigation/Flight Path Smoothing/Pass-through'
//  '<S160>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Waypoint Navigation/Flight Path Smoothing/Flight Path Smoothing/PT2 discrete with saturation3'
//  '<S161>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Waypoint Navigation/Flight Path Smoothing/Flight Path Smoothing/PT2 discrete with saturation4'
//  '<S162>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Waypoint Navigation/Flight Path Smoothing/Flight Path Smoothing/PT2 discrete with saturation5'
//  '<S163>' : 'ArduCopter_Fast_Descent/LindiCopter Autopilot/Waypoint Navigation/Waypoint radius/MATLAB Function'
//  '<S164>' : 'ArduCopter_Fast_Descent/log muxer/Auxiliary function to define log_config in generated C++ code'

#endif                                 // RTW_HEADER_MatlabController_h_

//
// File trailer for generated code.
//
// [EOF]
//
