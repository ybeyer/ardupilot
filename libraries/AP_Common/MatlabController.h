//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: MatlabController.h
//
// Code generated for Simulink model 'ArduPlane_LindiPlane'.
//
// Model version                  : 1.795
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Mon Feb 17 19:39:54 2025
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
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
#ifndef ArduPlane_LindiPlane_COMMON_INCLUDES_
# define ArduPlane_LindiPlane_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 // ArduPlane_LindiPlane_COMMON_INCLUDES_

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
  real32_T imu_p[8];
  real32_T imu_a_z[8];
} measureBus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_logConfigBus_
#define DEFINED_TYPEDEF_FOR_logConfigBus_

typedef struct {
  uint8_T num_signals;
  uint8_T signal_names[42];
  uint8_T batch_name[4];
} logConfigBus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_YFejTC3Mb5lyJ7XHyQfyqG_
#define DEFINED_TYPEDEF_FOR_struct_YFejTC3Mb5lyJ7XHyQfyqG_

typedef struct {
  real32_T clu[10];
  real32_T s[10];
  real32_T rotx[10];
  real32_T x[10];
  real32_T y[10];
  real32_T z[10];
} struct_YFejTC3Mb5lyJ7XHyQfyqG;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_NVgzPUNgrhoXyl8OD5Cw5F_
#define DEFINED_TYPEDEF_FOR_struct_NVgzPUNgrhoXyl8OD5Cw5F_

typedef struct {
  real32_T m;
  real32_T ixx;
  real32_T iyy;
  real32_T izz;
  real32_T ixy;
  real32_T ixz;
  real32_T iyz;
  real32_T scale;
} struct_NVgzPUNgrhoXyl8OD5Cw5F;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_D5pBfL2LPQt2lUKFw6GkNC_
#define DEFINED_TYPEDEF_FOR_struct_D5pBfL2LPQt2lUKFw6GkNC_

typedef struct {
  real32_T omega;
  real32_T d;
  real32_T delay;
  real32_T boost;
} struct_D5pBfL2LPQt2lUKFw6GkNC;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_XfqFc28dIjnSEgUD39JtWG_
#define DEFINED_TYPEDEF_FOR_struct_XfqFc28dIjnSEgUD39JtWG_

typedef struct {
  real32_T omega;
  real32_T d;
  real32_T numGyrFlt;
} struct_XfqFc28dIjnSEgUD39JtWG;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_sbKx6IyVlWG2aZGxKuo5AB_
#define DEFINED_TYPEDEF_FOR_struct_sbKx6IyVlWG2aZGxKuo5AB_

typedef struct {
  real32_T flttc;
  real32_T min;
} struct_sbKx6IyVlWG2aZGxKuo5AB;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_6BRWbv67APlqiRCPZEkE1C_
#define DEFINED_TYPEDEF_FOR_struct_6BRWbv67APlqiRCPZEkE1C_

typedef struct {
  real32_T rang;
  real32_T rrat;
  real32_T racc;
  real32_T pang;
  real32_T prat;
  real32_T pacc;
  real32_T yrat;
  real32_T yacc;
} struct_6BRWbv67APlqiRCPZEkE1C;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_iPhZVYNbhiJ7uk1NWTPeLC_
#define DEFINED_TYPEDEF_FOR_struct_iPhZVYNbhiJ7uk1NWTPeLC_

typedef struct {
  real32_T rfreq;
  real32_T rangmax;
  real32_T rratmax;
  real32_T pfreq;
  real32_T pangmax;
  real32_T pratmax;
  real32_T yfreq;
  real32_T yratmax;
  real32_T ydecaytc;
} struct_iPhZVYNbhiJ7uk1NWTPeLC;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_zFuKxOdy6xWCgrwijIt3EB_
#define DEFINED_TYPEDEF_FOR_struct_zFuKxOdy6xWCgrwijIt3EB_

typedef struct {
  struct_6BRWbv67APlqiRCPZEkE1C k;
  struct_iPhZVYNbhiJ7uk1NWTPeLC rm;
} struct_zFuKxOdy6xWCgrwijIt3EB;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_Byly2JHaAJfFWirb8CScwH_
#define DEFINED_TYPEDEF_FOR_struct_Byly2JHaAJfFWirb8CScwH_

typedef struct {
  real32_T clp;
  real32_T b;
  real32_T s;
  real32_T cla_h;
  real32_T x_h;
  real32_T s_h;
  real32_T cla;
  real32_T dahda;
  real32_T dahdu[8];
  real32_T xcg;
  real32_T xnp;
  real32_T xnp0;
} struct_Byly2JHaAJfFWirb8CScwH;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_wtF2XFOtOELBemGDyRar7G_
#define DEFINED_TYPEDEF_FOR_struct_wtF2XFOtOELBemGDyRar7G_

typedef struct {
  real32_T pos;
  real32_T vel;
  real32_T acc;
} struct_wtF2XFOtOELBemGDyRar7G;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_nUdewYMN9W2ArPMLvxYBeD_
#define DEFINED_TYPEDEF_FOR_struct_nUdewYMN9W2ArPMLvxYBeD_

typedef struct {
  struct_wtF2XFOtOELBemGDyRar7G k;
} struct_nUdewYMN9W2ArPMLvxYBeD;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_ZpRNaqoZiEb6XqsMOrzR0C_
#define DEFINED_TYPEDEF_FOR_struct_ZpRNaqoZiEb6XqsMOrzR0C_

typedef struct {
  real32_T T;
  real32_T wprad;
  real32_T eposmax;
} struct_ZpRNaqoZiEb6XqsMOrzR0C;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_t97zePNXqrora5Yj9ePAsF_
#define DEFINED_TYPEDEF_FOR_struct_t97zePNXqrora5Yj9ePAsF_

typedef struct {
  real32_T u_min[10];
  real32_T u_max[10];
  real32_T u_d[10];
  real32_T W_v[4];
  real32_T W_u[10];
  real32_T gamma;
  real32_T W[10];
  real32_T i_max;
} struct_t97zePNXqrora5Yj9ePAsF;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_9VdX5G3wISWD67sX4adsqF_
#define DEFINED_TYPEDEF_FOR_struct_9VdX5G3wISWD67sX4adsqF_

typedef struct {
  real32_T opt;
  real32_T flapdecay;
  real32_T maxptch;
} struct_9VdX5G3wISWD67sX4adsqF;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_pIgE2dka5KIuyGebGLL8dB_
#define DEFINED_TYPEDEF_FOR_struct_pIgE2dka5KIuyGebGLL8dB_

typedef struct {
  real32_T W_v[4];
  real32_T W_u[10];
  real32_T gamma;
  real32_T i_max;
} struct_pIgE2dka5KIuyGebGLL8dB;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_grmQpGhO6u50ZJYAi79l7E_
#define DEFINED_TYPEDEF_FOR_struct_grmQpGhO6u50ZJYAi79l7E_

typedef struct {
  real32_T use;
  real32_T eta_np[2];
  struct_pIgE2dka5KIuyGebGLL8dB ca;
} struct_grmQpGhO6u50ZJYAi79l7E;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_zTGPqEU9oPa0g1yI2jOg2F_
#define DEFINED_TYPEDEF_FOR_struct_zTGPqEU9oPa0g1yI2jOg2F_

typedef struct {
  real32_T len;
  real32_T mag;
} struct_zTGPqEU9oPa0g1yI2jOg2F;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_rGAsXWLaUsbxrXrQ2KHClH_
#define DEFINED_TYPEDEF_FOR_struct_rGAsXWLaUsbxrXrQ2KHClH_

typedef struct {
  struct_YFejTC3Mb5lyJ7XHyQfyqG cef;
  struct_NVgzPUNgrhoXyl8OD5Cw5F ceb;
  struct_D5pBfL2LPQt2lUKFw6GkNC servo;
  struct_XfqFc28dIjnSEgUD39JtWG sflt;
  struct_sbKx6IyVlWG2aZGxKuo5AB aspd;
  struct_zFuKxOdy6xWCgrwijIt3EB atc;
  struct_Byly2JHaAJfFWirb8CScwH eig;
  struct_nUdewYMN9W2ArPMLvxYBeD psc;
  struct_ZpRNaqoZiEb6XqsMOrzR0C wpnav;
  struct_t97zePNXqrora5Yj9ePAsF ca;
  struct_9VdX5G3wISWD67sX4adsqF dlc;
  struct_grmQpGhO6u50ZJYAi79l7E mla;
  struct_zTGPqEU9oPa0g1yI2jOg2F gust;
} struct_rGAsXWLaUsbxrXrQ2KHClH;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_8e4ElBjVTPe7Rw8vcm14eD_
#define DEFINED_TYPEDEF_FOR_struct_8e4ElBjVTPe7Rw8vcm14eD_

typedef struct {
  real32_T ts;
} struct_8e4ElBjVTPe7Rw8vcm14eD;

#endif

// Custom Type definition for MATLAB Function: '<S77>/MATLAB Function'
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
  real32_T Merge_m[10];                // '<S25>/Merge'
  real32_T u[11];
  real32_T Omega_Kb_dt_f[3];
  real32_T Euler_angles_f[3];
  real32_T Omega_Kb_f[3];
  real32_T e_s_g[3];
  real32_T e_s_g_dt[3];
  real32_T e_s_g_dt2[3];
  real32_T s_g_ref[3];                 // '<S9>/BusConversion_InsertedFor_ap_bus_at_inport_0' 
  real32_T s_g[3];                     // '<S9>/BusConversion_InsertedFor_ap_bus_at_inport_0' 
  real32_T s_g_match[3];
  real32_T s_g_match_i[3];
  real32_T Merge2_j[3];                // '<S140>/Merge2'
  real32_T Merge_l[3];                 // '<S140>/Merge'
  real32_T Merge1_p[3];                // '<S140>/Merge1'
  real32_T e_s_g_e[3];                 // '<S28>/Add'
  real32_T e_s_g_dt_i[3];              // '<S28>/Add1'
  real32_T e_s_g_dt2_k[3];             // '<S28>/Add2'
  real32_T nu[2];                      // '<S28>/Add5'
  real32_T s_g_ref_d[3];               // '<S28>/BusConversion_InsertedFor_pos_control_at_inport_0' 
  real32_T s_g_a[3];                   // '<S28>/BusConversion_InsertedFor_pos_control_at_inport_0' 
  real32_T DiscreteTimeIntegratory_DSTAT_b[9];// '<S48>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_DSTAT_n[3];// '<S49>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_DSTA_n2[3];// '<S50>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_DSTAT_j[3];// '<S51>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_DSTAT_l[10];// '<S106>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_DSTA_j2[10];// '<S105>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegrator1_DSTATE[10];// '<S109>/Discrete-Time Integrator1' 
  real32_T DiscreteTimeIntegrator2_DSTATE[10];// '<S109>/Discrete-Time Integrator2' 
  real32_T Delay_DSTATE[10];           // '<S107>/Delay'
  real32_T Delay1_DSTATE[10];          // '<S107>/Delay1'
  real32_T Delay2_DSTATE[10];          // '<S107>/Delay2'
  real32_T Delay3_DSTATE[10];          // '<S107>/Delay3'
  real32_T DiscreteTimeIntegratory_dt_DS_o[10];// '<S106>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_D_nz[10];// '<S105>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_D_ni[9];// '<S48>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_DS_e[3];// '<S51>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_DS_p[3];// '<S50>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_DS_i[3];// '<S49>/Discrete-Time Integrator y_dt' 
  real32_T UnitDelay6_DSTATE[3];       // '<S37>/Unit Delay6'
  real32_T UnitDelay4_DSTATE[15];      // '<S37>/Unit Delay4'
  real32_T DiscreteTimeIntegratory_DSTA_kr[3];// '<S150>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_D_iv[3];// '<S150>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTA_pq[3];// '<S151>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_D_ae[3];// '<S151>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTA_nm[3];// '<S65>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_D_jv[3];// '<S65>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTAT_e[3];// '<S64>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_D_ip[3];// '<S64>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTA_k4[3];// '<S66>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DS_c[3];// '<S66>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_D_pr[3];// '<S63>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTAT_m[3];// '<S63>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegrator1_DSTAT_j[6];// '<S56>/Discrete-Time Integrator1' 
  real32_T DiscreteTimeIntegrator1_DSTAT_n[6];// '<S55>/Discrete-Time Integrator1' 
  real32_T Merge2;                     // '<S9>/Merge2'
  real32_T Merge;                      // '<S9>/Merge'
  real32_T Merge1;                     // '<S9>/Merge1'
  real32_T iter;
  real32_T wp_idx;
  real32_T stage;
  real32_T t;
  real32_T V_A_f;
  real32_T wp_idx_n;
  real32_T stage_e;
  real32_T t_l;
  real32_T q;                          // '<S36>/Turn Coordination'
  real32_T e_Theta;                    // '<S30>/Add3'
  real32_T nu_q_dt_ptchcntrl;          // '<S30>/Gain'
  real32_T DiscreteTimeIntegratory_DSTATE;// '<S20>/Discrete-Time Integrator y'
  real32_T DiscreteTimeIntegrator_DSTATE;// '<S1>/Discrete-Time Integrator'
  real32_T DiscreteTimeIntegratory_dt_DSTA;// '<S20>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTAT_d;// '<S52>/Discrete-Time Integrator y' 
  real32_T UnitDelay_DSTATE;           // '<S33>/Unit Delay'
  real32_T DiscreteTimeIntegrator_DSTATE_e;// '<S80>/Discrete-Time Integrator'
  real32_T DiscreteTimeIntegratory_DSTAT_p;// '<S122>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DS_g;// '<S122>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTAT_c;// '<S134>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DS_f;// '<S134>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTAT_h;// '<S133>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_DSTA_jv;// '<S130>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_DSTAT_k;// '<S97>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DS_j;// '<S97>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTAT_f;// '<S93>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegrator_DSTATE_h;// '<S89>/Discrete-Time Integrator'
  real32_T DiscreteTimeIntegrator_DSTAT_en;// '<S91>/Discrete-Time Integrator'
  real32_T UnitDelay_DSTATE_f;         // '<S34>/Unit Delay'
  real32_T DiscreteTimeIntegrator_DSTATE_o;// '<S154>/Discrete-Time Integrator'
  real32_T DiscreteTimeIntegratory_DSTAT_g;// '<S157>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DS_n;// '<S157>/Discrete-Time Integrator y_dt' 
  real32_T UnitDelay_DSTATE_i;         // '<S100>/Unit Delay'
  real32_T DiscreteTimeIntegratory_DSTA_na;// '<S116>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_DSTAT_i;// '<S115>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_D_pj;// '<S115>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_D_ol;// '<S116>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTA_jg;// '<S156>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_D_jq;// '<S156>/Discrete-Time Integrator y_dt' 
  real32_T Delay_DSTATE_a;             // '<S158>/Delay'
  real32_T Delay1_DSTATE_g;            // '<S158>/Delay1'
  real32_T Delay2_DSTATE_n;            // '<S158>/Delay2'
  real32_T Delay3_DSTATE_a;            // '<S158>/Delay3'
  real32_T DiscreteTimeIntegrator_DSTATE_a;// '<S90>/Discrete-Time Integrator'
  real32_T DiscreteTimeIntegratory_dt_D_n5;// '<S93>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTA_hj;// '<S92>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_D_j0;// '<S92>/Discrete-Time Integrator y_dt' 
  real32_T Delay_DSTATE_k;             // '<S94>/Delay'
  real32_T Delay1_DSTATE_h;            // '<S94>/Delay1'
  real32_T Delay2_DSTATE_h;            // '<S94>/Delay2'
  real32_T Delay3_DSTATE_m;            // '<S94>/Delay3'
  real32_T DiscreteTimeIntegratory_DSTA_ny;// '<S96>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DS_a;// '<S96>/Discrete-Time Integrator y_dt' 
  real32_T Delay_DSTATE_n;             // '<S98>/Delay'
  real32_T Delay1_DSTATE_k;            // '<S98>/Delay1'
  real32_T Delay2_DSTATE_p;            // '<S98>/Delay2'
  real32_T Delay3_DSTATE_h;            // '<S98>/Delay3'
  real32_T DiscreteTimeIntegratory_dt_DS_m;// '<S130>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTAT_a;// '<S129>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DS_b;// '<S129>/Discrete-Time Integrator y_dt' 
  real32_T Delay_DSTATE_j;             // '<S131>/Delay'
  real32_T Delay1_DSTATE_ha;           // '<S131>/Delay1'
  real32_T Delay2_DSTATE_m;            // '<S131>/Delay2'
  real32_T Delay3_DSTATE_p;            // '<S131>/Delay3'
  real32_T DiscreteTimeIntegratory_dt_DS_k;// '<S133>/Discrete-Time Integrator y_dt' 
  real32_T Delay_DSTATE_jn;            // '<S135>/Delay'
  real32_T Delay1_DSTATE_d;            // '<S135>/Delay1'
  real32_T Delay2_DSTATE_j;            // '<S135>/Delay2'
  real32_T Delay3_DSTATE_g;            // '<S135>/Delay3'
  real32_T DiscreteTimeIntegratory_dt_D_gf;// '<S52>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegrator_DSTATE_p;// '<S72>/Discrete-Time Integrator'
  real32_T DiscreteTimeIntegrator_DSTAT_ar;// '<S73>/Discrete-Time Integrator'
  real32_T DiscreteTimeIntegratory_DSTAT_o;// '<S76>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_D_i2;// '<S76>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTA_bd;// '<S75>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_D_bo;// '<S75>/Discrete-Time Integrator y_dt' 
  real32_T Delay_DSTATE_b;             // '<S77>/Delay'
  real32_T Delay1_DSTATE_m;            // '<S77>/Delay1'
  real32_T Delay2_DSTATE_a;            // '<S77>/Delay2'
  real32_T Delay3_DSTATE_mn;           // '<S77>/Delay3'
  real32_T UnitDelay1_DSTATE;          // '<S29>/Unit Delay1'
  real32_T DiscreteTimeIntegrator_DSTATE_f;// '<S69>/Discrete-Time Integrator'
  real32_T DiscreteTimeIntegrator_DSTAT_o4;// '<S56>/Discrete-Time Integrator'
  real32_T UnitDelay_DSTATE_g;         // '<S56>/Unit Delay'
  real32_T DiscreteTimeIntegrator_DSTATE_b;// '<S55>/Discrete-Time Integrator'
  real32_T UnitDelay_DSTATE_l;         // '<S55>/Unit Delay'
  int32_T UnitDelay_DSTATE_b;          // '<S37>/Unit Delay'
  int32_T UnitDelay1_DSTATE_g;         // '<S37>/Unit Delay1'
  int32_T UnitDelay2_DSTATE;           // '<S37>/Unit Delay2'
  int32_T UnitDelay3_DSTATE;           // '<S37>/Unit Delay3'
  int8_T DiscreteTimeIntegrator_PrevRese;// '<S1>/Discrete-Time Integrator'
  uint8_T DiscreteTimeIntegratory_IC_LOAD;// '<S52>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_LO_a;// '<S48>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_LO_e;// '<S50>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_LO_i;// '<S51>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegrator_IC_LOADI;// '<S80>/Discrete-Time Integrator'
  uint8_T DiscreteTimeIntegratory_IC_LO_m;// '<S134>/Discrete-Time Integrator y' 
  uint8_T DiscreteTimeIntegratory_IC_LO_d;// '<S133>/Discrete-Time Integrator y' 
  uint8_T DiscreteTimeIntegratory_IC_LO_o;// '<S130>/Discrete-Time Integrator y' 
  uint8_T DiscreteTimeIntegratory_IC_L_eu;// '<S97>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_LO_k;// '<S93>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegrator_IC_LOA_g;// '<S154>/Discrete-Time Integrator'
  uint8_T DiscreteTimeIntegratory_IC_LO_f;// '<S157>/Discrete-Time Integrator y' 
  uint8_T icLoad;                      // '<S107>/Delay'
  uint8_T icLoad_i;                    // '<S107>/Delay1'
  uint8_T icLoad_o;                    // '<S107>/Delay2'
  uint8_T icLoad_k;                    // '<S107>/Delay3'
  uint8_T DiscreteTimeIntegratory_IC_L_km;// '<S156>/Discrete-Time Integrator y' 
  uint8_T icLoad_d;                    // '<S158>/Delay'
  uint8_T icLoad_b;                    // '<S158>/Delay1'
  uint8_T icLoad_l;                    // '<S158>/Delay2'
  uint8_T icLoad_b1;                   // '<S158>/Delay3'
  uint8_T DiscreteTimeIntegratory_IC_LO_g;// '<S92>/Discrete-Time Integrator y'
  uint8_T icLoad_dk;                   // '<S94>/Delay'
  uint8_T icLoad_j;                    // '<S94>/Delay1'
  uint8_T icLoad_ix;                   // '<S94>/Delay2'
  uint8_T icLoad_ow;                   // '<S94>/Delay3'
  uint8_T DiscreteTimeIntegratory_IC_L_el;// '<S96>/Discrete-Time Integrator y'
  uint8_T icLoad_dh;                   // '<S98>/Delay'
  uint8_T icLoad_f;                    // '<S98>/Delay1'
  uint8_T icLoad_fc;                   // '<S98>/Delay2'
  uint8_T icLoad_jp;                   // '<S98>/Delay3'
  uint8_T DiscreteTimeIntegratory_IC_LO_b;// '<S129>/Discrete-Time Integrator y' 
  uint8_T icLoad_dhb;                  // '<S131>/Delay'
  uint8_T icLoad_fx;                   // '<S131>/Delay1'
  uint8_T icLoad_m;                    // '<S131>/Delay2'
  uint8_T icLoad_dc;                   // '<S131>/Delay3'
  uint8_T icLoad_f1;                   // '<S135>/Delay'
  uint8_T icLoad_fu;                   // '<S135>/Delay1'
  uint8_T icLoad_ib;                   // '<S135>/Delay2'
  uint8_T icLoad_kd;                   // '<S135>/Delay3'
  uint8_T DiscreteTimeIntegratory_IC_L_io;// '<S150>/Discrete-Time Integrator y' 
  uint8_T DiscreteTimeIntegratory_IC_L_o2;// '<S151>/Discrete-Time Integrator y' 
  uint8_T DiscreteTimeIntegrator_IC_LOA_m;// '<S72>/Discrete-Time Integrator'
  uint8_T DiscreteTimeIntegrator_IC_LO_mz;// '<S73>/Discrete-Time Integrator'
  uint8_T DiscreteTimeIntegratory_IC_LO_l;// '<S76>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_L_id;// '<S75>/Discrete-Time Integrator y'
  uint8_T icLoad_fe;                   // '<S77>/Delay'
  uint8_T icLoad_lk;                   // '<S77>/Delay1'
  uint8_T icLoad_jd;                   // '<S77>/Delay2'
  uint8_T icLoad_f13;                  // '<S77>/Delay3'
  uint8_T DiscreteTimeIntegrator_IC_LOA_l;// '<S69>/Discrete-Time Integrator'
  uint8_T DiscreteTimeIntegratory_IC_L_d2;// '<S65>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_LO_p;// '<S64>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_L_ls;// '<S66>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegrator1_IC_LOAD;// '<S56>/Discrete-Time Integrator1'
  uint8_T DiscreteTimeIntegrator1_IC_LO_g;// '<S55>/Discrete-Time Integrator1'
  boolean_T UnitDelay5_DSTATE;         // '<S37>/Unit Delay5'
  boolean_T LindiPlaneAutopilot_MODE;  // '<Root>/LindiPlane Autopilot'
  boolean_T WaypointNavigation_MODE;   // '<S9>/Waypoint Navigation'
  boolean_T FlightPathSmoothing_MODE;  // '<S140>/Flight Path Smoothing'
  boolean_T PitchAngleController_MODE; // '<S9>/Pitch Angle Controller'
  boolean_T OuterLoopINDI_MODE;        // '<S9>/Outer Loop INDI'
  boolean_T NDIPositionController_MODE;// '<S9>/NDI Position Controller'
  boolean_T ManeuverLoadAlleviation_MODE;// '<S25>/Maneuver Load Alleviation'
} DW;

// Constant parameters (default storage)
typedef struct {
  // Computed Parameter: Gain_Gain_o
  //  Referenced by: '<S13>/Gain'

  real32_T Gain_Gain_o[8];

  // Expression: ch_fcn
  //  Referenced by: '<S2>/Constant'

  uint16_T Constant_Value_n[16];
} ConstP;

// External inputs (root inport signals with default storage)
typedef struct {
  cmdBus cmd;                          // '<Root>/cmd'
  measureBus measure;                  // '<Root>/measure'
} ExtU;

// External outputs (root outports fed by signals with default storage)
typedef struct {
  real32_T channels[16];               // '<Root>/channels'
  uint16_T function_channels[16];      // '<Root>/function_channels'
  real32_T logs[69];                   // '<Root>/logs'
} ExtY;

// Parameters (default storage)
struct P_ {
  struct_rGAsXWLaUsbxrXrQ2KHClH lindi; // Variable: lindi
                                       //  Referenced by:
                                       //    '<S1>/Constant1'
                                       //    '<S1>/Constant2'
                                       //    '<S7>/Flap deflections'
                                       //    '<S7>/Gain13'
                                       //    '<S8>/Constant'
                                       //    '<S9>/Cmd 2 Yaw Rate'
                                       //    '<S21>/Gain'
                                       //    '<S25>/Constant'
                                       //    '<S26>/Constant'
                                       //    '<S26>/Constant1'
                                       //    '<S28>/Gain'
                                       //    '<S28>/Gain1'
                                       //    '<S28>/Gain2'
                                       //    '<S30>/Gain'
                                       //    '<S30>/cmd 2 angle'
                                       //    '<S32>/Constant'
                                       //    '<S32>/Saturation3'
                                       //    '<S33>/Saturation'
                                       //    '<S34>/Gain5'
                                       //    '<S37>/Constant2'
                                       //    '<S37>/Constant3'
                                       //    '<S37>/Constant5'
                                       //    '<S41>/Maneuver Load Alleviation'
                                       //    '<S41>/Constant'
                                       //    '<S41>/Constant2'
                                       //    '<S42>/Constant'
                                       //    '<S47>/Constant1'
                                       //    '<S47>/Constant2'
                                       //    '<S48>/d'
                                       //    '<S48>/omega'
                                       //    '<S49>/d'
                                       //    '<S49>/omega'
                                       //    '<S50>/d'
                                       //    '<S50>/omega'
                                       //    '<S51>/d'
                                       //    '<S51>/omega'
                                       //    '<S52>/omega'
                                       //    '<S64>/omega'
                                       //    '<S65>/omega'
                                       //    '<S66>/omega'
                                       //    '<S69>/T'
                                       //    '<S72>/T'
                                       //    '<S73>/T'
                                       //    '<S79>/Gain3'
                                       //    '<S79>/Gain5'
                                       //    '<S80>/T'
                                       //    '<S81>/Gain8'
                                       //    '<S100>/Constant2'
                                       //    '<S100>/Gain'
                                       //    '<S101>/Constant1'
                                       //    '<S102>/Flap downwash compensation'
                                       //    '<S102>/Gain12'
                                       //    '<S103>/caIndiWls'
                                       //    '<S103>/Delta u_max'
                                       //    '<S105>/d'
                                       //    '<S105>/omega'
                                       //    '<S106>/d'
                                       //    '<S106>/omega'
                                       //    '<S107>/Constant'
                                       //    '<S109>/Gain4'
                                       //    '<S109>/Gain5'
                                       //    '<S109>/Gain6'
                                       //    '<S109>/Gain7'
                                       //    '<S109>/Gain8'
                                       //    '<S110>/indiCeFlapFix'
                                       //    '<S110>/Constant'
                                       //    '<S121>/Gain1'
                                       //    '<S121>/Gain3'
                                       //    '<S121>/Gain5'
                                       //    '<S122>/omega'
                                       //    '<S123>/Gain'
                                       //    '<S123>/Gain1'
                                       //    '<S123>/Gain3'
                                       //    '<S123>/Gain4'
                                       //    '<S140>/Constant'
                                       //    '<S144>/wp_rad_fix'
                                       //    '<S153>/Gain3'
                                       //    '<S153>/Gain5'
                                       //    '<S154>/T'
                                       //    '<S63>/d'
                                       //    '<S63>/omega'
                                       //    '<S75>/d'
                                       //    '<S75>/omega'
                                       //    '<S76>/d'
                                       //    '<S76>/omega'
                                       //    '<S77>/Constant'
                                       //    '<S83>/Gain5'
                                       //    '<S83>/Gain6'
                                       //    '<S83>/Gain7'
                                       //    '<S84>/Gain1'
                                       //    '<S84>/Gain12'
                                       //    '<S84>/Gain2'
                                       //    '<S84>/Gain3'
                                       //    '<S96>/d'
                                       //    '<S96>/omega'
                                       //    '<S97>/d'
                                       //    '<S97>/omega'
                                       //    '<S98>/Constant'
                                       //    '<S133>/d'
                                       //    '<S133>/omega'
                                       //    '<S134>/d'
                                       //    '<S134>/omega'
                                       //    '<S135>/Constant'
                                       //    '<S147>/Constant'
                                       //    '<S156>/d'
                                       //    '<S156>/omega'
                                       //    '<S157>/d'
                                       //    '<S157>/omega'
                                       //    '<S158>/Constant'
                                       //    '<S86>/Constant'
                                       //    '<S86>/Constant6'
                                       //    '<S86>/Constant7'
                                       //    '<S86>/Constant8'
                                       //    '<S92>/d'
                                       //    '<S92>/omega'
                                       //    '<S93>/d'
                                       //    '<S93>/omega'
                                       //    '<S94>/Constant'
                                       //    '<S129>/d'
                                       //    '<S129>/omega'
                                       //    '<S130>/d'
                                       //    '<S130>/omega'
                                       //    '<S131>/Constant'

};

// Parameters (default storage)
typedef struct P_ P;

// Constant parameters (default storage)
extern const ConstP rtConstP;

// Exported data declaration

// Const memory section
// Declaration for custom storage class: Const
extern const logConfigBus log_config[5];

// Class declaration for model ArduPlane_LindiPlane
class MatlabControllerClass {
  // public data and function members
 public:
  static const struct AP_Param::GroupInfo var_info_rtP_lindi_cef[];
  static const struct AP_Param::GroupInfo var_info_rtP_lindi_ceb[];
  static const struct AP_Param::GroupInfo var_info_rtP_lindi_servo[];
  static const struct AP_Param::GroupInfo var_info_rtP_lindi_sflt[];
  static const struct AP_Param::GroupInfo var_info_rtP_lindi_aspd[];
  static const struct AP_Param::GroupInfo var_info_rtP_lindi_atc[];
  static const struct AP_Param::GroupInfo var_info_rtP_lindi_eig[];
  static const struct AP_Param::GroupInfo var_info_rtP_lindi_psc[];
  static const struct AP_Param::GroupInfo var_info_rtP_lindi_wpnav[];
  static const struct AP_Param::GroupInfo var_info_rtP_lindi_ca[];
  static const struct AP_Param::GroupInfo var_info_rtP_lindi_dlc[];
  static const struct AP_Param::GroupInfo var_info_rtP_lindi_mla[];
  static const struct AP_Param::GroupInfo var_info_rtP_lindi_gust[];
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
  real32_T norm_c(const real32_T x[3]);
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
  void wpnavMatch_p(const real32_T waypoints_data[], const int32_T
                    waypoints_size[2], real32_T wp_radius, int32_T *wp_idx,
                    int32_T *stage, const real32_T p[3], real32_T p_match[3],
                    real32_T *t, real32_T *d);
  void wpnavMatch(const real32_T waypoints[15], real32_T wp_radius, int32_T
                  *wp_idx, int32_T *stage, const real32_T p[3], real32_T
                  p_match[3], real32_T *t, real32_T *d);
  real32_T norm(const real32_T x[2]);
  void LSQFromQR(const real32_T A_data[], const int32_T A_size[2], const
                 real32_T tau_data[], const int32_T jpvt_data[], real32_T B_3[13],
                 int32_T rankA, real32_T Y_data[], int32_T *Y_size);
  real32_T xnrm2(int32_T n, const real32_T x_data[], int32_T ix0);
  void xzlarf(int32_T m, int32_T n, int32_T iv0, real32_T tau, real32_T C_data[],
              int32_T ic0, real32_T work_data[]);
  void qrsolve(const real32_T A_data[], const int32_T A_size[2], const real32_T
               B_1[13], real32_T Y_data[], int32_T *Y_size);
  void mldivide(const real32_T A_data[], const int32_T A_size[2], const real32_T
                B_0[13], real32_T Y_data[], int32_T *Y_size);
  boolean_T any(const boolean_T x_data[], const int32_T *x_size);
  real32_T wls_alloc(const real32_T B_4[36], const real32_T v[4], const real32_T
                     umin[9], const real32_T umax[9], const real32_T Wv[16],
                     const real32_T Wu[81], const real32_T ud[9], real32_T gam,
                     real32_T u[9], real32_T W[9], real32_T imax);
  void LSQFromQR_e(const real32_T A_data[], const int32_T A_size[2], const
                   real32_T tau_data[], const int32_T jpvt_data[], real32_T B_8
                   [14], int32_T rankA, real32_T Y_data[], int32_T *Y_size);
  void xzlarf_h(int32_T m, int32_T n, int32_T iv0, real32_T tau, real32_T
                C_data[], int32_T ic0, real32_T work_data[]);
  void qrsolve_a(const real32_T A_data[], const int32_T A_size[2], const
                 real32_T B_6[14], real32_T Y_data[], int32_T *Y_size);
  void mldivide_k(const real32_T A_data[], const int32_T A_size[2], const
                  real32_T B_5[14], real32_T Y_data[], int32_T *Y_size);
  real32_T wls_alloc_c(const real32_T B_9[40], const real32_T v[4], const
                       real32_T umin[10], const real32_T umax[10], const
                       real32_T Wv[16], const real32_T Wu[100], const real32_T
                       ud[10], real32_T gam, real32_T u[10], real32_T W[10],
                       real32_T imax);
  real32_T sum(const real32_T x[8]);
};

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S28>/Scope' : Unused code path elimination
//  Block '<S31>/Scope' : Unused code path elimination
//  Block '<S110>/Display' : Unused code path elimination
//  Block '<S33>/Scope' : Unused code path elimination
//  Block '<S33>/Scope1' : Unused code path elimination
//  Block '<S149>/Discrete-Time Integrator y' : Unused code path elimination
//  Block '<S149>/Discrete-Time Integrator y_dt' : Unused code path elimination
//  Block '<S149>/Divide' : Unused code path elimination
//  Block '<S149>/Gain' : Unused code path elimination
//  Block '<S149>/Product1' : Unused code path elimination
//  Block '<S149>/Product2' : Unused code path elimination
//  Block '<S149>/Saturation' : Unused code path elimination
//  Block '<S149>/Sum2' : Unused code path elimination
//  Block '<S149>/Sum3' : Unused code path elimination
//  Block '<S149>/omega^2' : Unused code path elimination
//  Block '<S149>/y_dt_0' : Unused code path elimination
//  Block '<S147>/Scope' : Unused code path elimination
//  Block '<S147>/Scope1' : Unused code path elimination
//  Block '<S147>/Scope2' : Unused code path elimination
//  Block '<S147>/Scope3' : Unused code path elimination
//  Block '<S37>/Scope1' : Unused code path elimination
//  Block '<S13>/Scope' : Unused code path elimination
//  Block '<S2>/Data Type Conversion1' : Eliminate redundant data type conversion
//  Block '<Root>/Gain' : Eliminated nontunable gain of 1
//  Block '<S20>/Saturation' : Eliminated Saturate block
//  Block '<S41>/Reshape' : Reshape block reduction
//  Block '<S41>/Reshape1' : Reshape block reduction
//  Block '<S48>/Saturation' : Eliminated Saturate block
//  Block '<S49>/Saturation' : Eliminated Saturate block
//  Block '<S50>/Saturation' : Eliminated Saturate block
//  Block '<S51>/Saturation' : Eliminated Saturate block
//  Block '<S52>/Saturation' : Eliminated Saturate block
//  Block '<S26>/Reshape' : Reshape block reduction
//  Block '<S26>/Reshape1' : Reshape block reduction
//  Block '<S63>/Saturation' : Eliminated Saturate block
//  Block '<S64>/Saturation' : Eliminated Saturate block
//  Block '<S65>/Saturation' : Eliminated Saturate block
//  Block '<S66>/Saturation' : Eliminated Saturate block
//  Block '<S29>/Gain4' : Eliminated nontunable gain of 1
//  Block '<S69>/Saturation' : Eliminated Saturate block
//  Block '<S73>/Saturation' : Eliminated Saturate block
//  Block '<S75>/Saturation' : Eliminated Saturate block
//  Block '<S76>/Saturation' : Eliminated Saturate block
//  Block '<S80>/Saturation' : Eliminated Saturate block
//  Block '<S89>/Saturation' : Eliminated Saturate block
//  Block '<S90>/Saturation' : Eliminated Saturate block
//  Block '<S91>/Saturation' : Eliminated Saturate block
//  Block '<S92>/Saturation' : Eliminated Saturate block
//  Block '<S93>/Saturation' : Eliminated Saturate block
//  Block '<S96>/Saturation' : Eliminated Saturate block
//  Block '<S97>/Saturation' : Eliminated Saturate block
//  Block '<S115>/Saturation' : Eliminated Saturate block
//  Block '<S116>/Saturation' : Eliminated Saturate block
//  Block '<S105>/Saturation' : Eliminated Saturate block
//  Block '<S106>/Saturation' : Eliminated Saturate block
//  Block '<S122>/Saturation' : Eliminated Saturate block
//  Block '<S129>/Saturation' : Eliminated Saturate block
//  Block '<S130>/Saturation' : Eliminated Saturate block
//  Block '<S133>/Saturation' : Eliminated Saturate block
//  Block '<S134>/Saturation' : Eliminated Saturate block
//  Block '<S37>/Data Type Conversion' : Eliminate redundant data type conversion
//  Block '<S37>/Data Type Conversion1' : Eliminate redundant data type conversion
//  Block '<S37>/Data Type Conversion3' : Eliminate redundant data type conversion
//  Block '<S37>/Data Type Conversion4' : Eliminate redundant data type conversion
//  Block '<S37>/Data Type Conversion5' : Eliminate redundant data type conversion
//  Block '<S150>/Saturation' : Eliminated Saturate block
//  Block '<S151>/Saturation' : Eliminated Saturate block
//  Block '<S154>/Saturation' : Eliminated Saturate block
//  Block '<S38>/Reshape' : Reshape block reduction
//  Block '<S156>/Saturation' : Eliminated Saturate block
//  Block '<S157>/Saturation' : Eliminated Saturate block
//  Block '<S144>/Constant' : Unused code path elimination
//  Block '<S144>/Constant1' : Unused code path elimination
//  Block '<S144>/Divide' : Unused code path elimination
//  Block '<S144>/Max' : Unused code path elimination
//  Block '<S144>/Square' : Unused code path elimination
//  Block '<S144>/wp_rad_min' : Unused code path elimination


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
//  '<Root>' : 'ArduPlane_LindiPlane'
//  '<S1>'   : 'ArduPlane_LindiPlane/1-cos Gust'
//  '<S2>'   : 'ArduPlane_LindiPlane/Actuator Muxer ArduPlane'
//  '<S3>'   : 'ArduPlane_LindiPlane/Compare To Constant'
//  '<S4>'   : 'ArduPlane_LindiPlane/Compare To Constant1'
//  '<S5>'   : 'ArduPlane_LindiPlane/Compare To Constant2'
//  '<S6>'   : 'ArduPlane_LindiPlane/Compare To Constant3'
//  '<S7>'   : 'ArduPlane_LindiPlane/Gust Emulator'
//  '<S8>'   : 'ArduPlane_LindiPlane/Limit airspeed'
//  '<S9>'   : 'ArduPlane_LindiPlane/LindiPlane Autopilot'
//  '<S10>'  : 'ArduPlane_LindiPlane/PWM to -1_1'
//  '<S11>'  : 'ArduPlane_LindiPlane/Quaternions to Rotation Matrix'
//  '<S12>'  : 'ArduPlane_LindiPlane/Remove velocity'
//  '<S13>'  : 'ArduPlane_LindiPlane/Subsystem'
//  '<S14>'  : 'ArduPlane_LindiPlane/log muxer'
//  '<S15>'  : 'ArduPlane_LindiPlane/1-cos Gust/1-cos Gust'
//  '<S16>'  : 'ArduPlane_LindiPlane/1-cos Gust/Compare To Constant'
//  '<S17>'  : 'ArduPlane_LindiPlane/Gust Emulator/Angle of attack'
//  '<S18>'  : 'ArduPlane_LindiPlane/Gust Emulator/Elevator downwash compensation'
//  '<S19>'  : 'ArduPlane_LindiPlane/Gust Emulator/Flap deflections'
//  '<S20>'  : 'ArduPlane_LindiPlane/Gust Emulator/PT2 discrete with saturation2'
//  '<S21>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Cmd 2 Roll Angle'
//  '<S22>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Compare To Constant'
//  '<S23>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Demux'
//  '<S24>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Direct Lift Control'
//  '<S25>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Maneuver Load Alleviation'
//  '<S26>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Measurement Filtering'
//  '<S27>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Mux'
//  '<S28>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/NDI Position Controller'
//  '<S29>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Outer Loop INDI'
//  '<S30>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Angle Controller'
//  '<S31>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Rate Controller'
//  '<S32>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Plane Inner Loop INDI and Control Allocation'
//  '<S33>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Roll Angle Controller'
//  '<S34>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Rudder Decay Controller'
//  '<S35>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Sstick Command Bus'
//  '<S36>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Turn Coordination'
//  '<S37>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Waypoint Navigation'
//  '<S38>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Yaw Rate Controller'
//  '<S39>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/add'
//  '<S40>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Maneuver Load Alleviation/Compare To Constant'
//  '<S41>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Maneuver Load Alleviation/Maneuver Load Alleviation'
//  '<S42>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Maneuver Load Alleviation/Subsystem'
//  '<S43>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Maneuver Load Alleviation/Maneuver Load Alleviation/Desired Specific Lift'
//  '<S44>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Maneuver Load Alleviation/Maneuver Load Alleviation/Maneuver Load Alleviation'
//  '<S45>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Maneuver Load Alleviation/Maneuver Load Alleviation/Desired Specific Lift/MATLAB Function'
//  '<S46>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Measurement Filtering/Compare To Constant'
//  '<S47>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Measurement Filtering/Duplicate LPF'
//  '<S48>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Measurement Filtering/PT2 discrete with saturation1'
//  '<S49>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Measurement Filtering/PT2 discrete with saturation2'
//  '<S50>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Measurement Filtering/PT2 discrete with saturation3'
//  '<S51>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Measurement Filtering/PT2 discrete with saturation4'
//  '<S52>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Measurement Filtering/PT2 discrete with saturation5'
//  '<S53>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Measurement Filtering/Rotations matrix to Euler angles'
//  '<S54>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Measurement Filtering/Single LPF'
//  '<S55>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Measurement Filtering/Duplicate LPF/PT2 discrete ODE1'
//  '<S56>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Measurement Filtering/Duplicate LPF/PT2 discrete ODE4'
//  '<S57>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Measurement Filtering/Duplicate LPF/PT2 discrete ODE1/PT2 IC'
//  '<S58>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Measurement Filtering/Duplicate LPF/PT2 discrete ODE1/PT2 discrete ode4'
//  '<S59>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Measurement Filtering/Duplicate LPF/PT2 discrete ODE1/PT2 split'
//  '<S60>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Measurement Filtering/Duplicate LPF/PT2 discrete ODE4/PT2 IC'
//  '<S61>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Measurement Filtering/Duplicate LPF/PT2 discrete ODE4/PT2 discrete ode4'
//  '<S62>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Measurement Filtering/Duplicate LPF/PT2 discrete ODE4/PT2 split'
//  '<S63>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Measurement Filtering/Single LPF/PT2 discrete with saturation2'
//  '<S64>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/NDI Position Controller/PT2 discrete with saturation'
//  '<S65>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/NDI Position Controller/PT2 discrete with saturation1'
//  '<S66>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/NDI Position Controller/PT2 discrete with saturation2'
//  '<S67>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Outer Loop INDI/Bank angle'
//  '<S68>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Outer Loop INDI/Outer Loop INDI'
//  '<S69>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Outer Loop INDI/PT1 discrete with saturations1'
//  '<S70>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Outer Loop INDI/Bank angle/DCM to quaternions1'
//  '<S71>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Outer Loop INDI/Bank angle/Quaternions to Euler angles1'
//  '<S72>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Angle Controller/PT1 discrete with saturations'
//  '<S73>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Angle Controller/PT1 discrete with saturations1'
//  '<S74>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Angle Controller/Servo and Sensor Filter Delay'
//  '<S75>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Angle Controller/Servo and Sensor Filter Delay/PT2 discrete with saturation5'
//  '<S76>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Angle Controller/Servo and Sensor Filter Delay/PT2 discrete with saturation6'
//  '<S77>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Angle Controller/Servo and Sensor Filter Delay/Servo delay'
//  '<S78>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Angle Controller/Servo and Sensor Filter Delay/Servo delay/MATLAB Function'
//  '<S79>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Rate Controller/Feedback Gains'
//  '<S80>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Rate Controller/PT1 discrete with saturations'
//  '<S81>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Rate Controller/Pitch eigendynamics'
//  '<S82>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Rate Controller/Servo and Sensor Filter Delay'
//  '<S83>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Rate Controller/Pitch eigendynamics/Pitch damping'
//  '<S84>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Rate Controller/Pitch eigendynamics/Pitch stiffness'
//  '<S85>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Rate Controller/Pitch eigendynamics/Servo and Sensor Filter Delay'
//  '<S86>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Rate Controller/Pitch eigendynamics/Pitch stiffness/Delta angle of attack'
//  '<S87>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Rate Controller/Pitch eigendynamics/Pitch stiffness/Downwash transport delay'
//  '<S88>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Rate Controller/Pitch eigendynamics/Pitch stiffness/Delta angle of attack/Angle of attack time delay'
//  '<S89>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Rate Controller/Pitch eigendynamics/Pitch stiffness/Delta angle of attack/PT1 discrete with saturations'
//  '<S90>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Rate Controller/Pitch eigendynamics/Pitch stiffness/Downwash transport delay/PT1 discrete with saturations'
//  '<S91>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Rate Controller/Pitch eigendynamics/Pitch stiffness/Downwash transport delay/PT1 discrete with saturations1'
//  '<S92>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Rate Controller/Pitch eigendynamics/Servo and Sensor Filter Delay/PT2 discrete with saturation5'
//  '<S93>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Rate Controller/Pitch eigendynamics/Servo and Sensor Filter Delay/PT2 discrete with saturation6'
//  '<S94>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Rate Controller/Pitch eigendynamics/Servo and Sensor Filter Delay/Servo delay'
//  '<S95>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Rate Controller/Pitch eigendynamics/Servo and Sensor Filter Delay/Servo delay/MATLAB Function'
//  '<S96>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Rate Controller/Servo and Sensor Filter Delay/PT2 discrete with saturation5'
//  '<S97>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Rate Controller/Servo and Sensor Filter Delay/PT2 discrete with saturation6'
//  '<S98>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Rate Controller/Servo and Sensor Filter Delay/Servo delay'
//  '<S99>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Rate Controller/Servo and Sensor Filter Delay/Servo delay/MATLAB Function'
//  '<S100>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Plane Inner Loop INDI and Control Allocation/Collective ailerons decay'
//  '<S101>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Plane Inner Loop INDI and Control Allocation/De-activate DLC at high pitch angle errors'
//  '<S102>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Plane Inner Loop INDI and Control Allocation/Flap downwash compensation'
//  '<S103>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Plane Inner Loop INDI and Control Allocation/INDI control allocation'
//  '<S104>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Plane Inner Loop INDI and Control Allocation/Only rotational control effectiveness'
//  '<S105>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Plane Inner Loop INDI and Control Allocation/PT2 discrete with saturation'
//  '<S106>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Plane Inner Loop INDI and Control Allocation/PT2 discrete with saturation1'
//  '<S107>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Plane Inner Loop INDI and Control Allocation/Servo delay'
//  '<S108>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Plane Inner Loop INDI and Control Allocation/Set Delta_nu_acc_z'
//  '<S109>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Plane Inner Loop INDI and Control Allocation/T2 booster'
//  '<S110>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Plane Inner Loop INDI and Control Allocation/control effectiveness'
//  '<S111>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Plane Inner Loop INDI and Control Allocation/Collective ailerons decay/MATLAB Function2'
//  '<S112>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Plane Inner Loop INDI and Control Allocation/De-activate DLC at high pitch angle errors/MATLAB Function'
//  '<S113>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Plane Inner Loop INDI and Control Allocation/De-activate DLC at high pitch angle errors/rad2deg'
//  '<S114>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Plane Inner Loop INDI and Control Allocation/Flap downwash compensation/Flap downwash compensation'
//  '<S115>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Plane Inner Loop INDI and Control Allocation/Flap downwash compensation/PT2 discrete with saturation1'
//  '<S116>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Plane Inner Loop INDI and Control Allocation/Flap downwash compensation/PT2 discrete with saturation2'
//  '<S117>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Plane Inner Loop INDI and Control Allocation/INDI control allocation/caIndiWls'
//  '<S118>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Plane Inner Loop INDI and Control Allocation/Servo delay/MATLAB Function'
//  '<S119>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Plane Inner Loop INDI and Control Allocation/control effectiveness/indiCeFlapFix'
//  '<S120>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Roll Angle Controller/Avoid Angle Steps'
//  '<S121>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Roll Angle Controller/Feedback Gains'
//  '<S122>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Roll Angle Controller/PT2 discrete with saturation1'
//  '<S123>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Roll Angle Controller/Roll damping inversion'
//  '<S124>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Roll Angle Controller/Servo and Sensor Filter Delay'
//  '<S125>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Roll Angle Controller/Feedback Gains/angle error'
//  '<S126>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Roll Angle Controller/Feedback Gains/wrap angle'
//  '<S127>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Roll Angle Controller/Feedback Gains/wrap angle1'
//  '<S128>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Roll Angle Controller/Roll damping inversion/Servo and Sensor Filter Delay'
//  '<S129>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Roll Angle Controller/Roll damping inversion/Servo and Sensor Filter Delay/PT2 discrete with saturation5'
//  '<S130>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Roll Angle Controller/Roll damping inversion/Servo and Sensor Filter Delay/PT2 discrete with saturation6'
//  '<S131>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Roll Angle Controller/Roll damping inversion/Servo and Sensor Filter Delay/Servo delay'
//  '<S132>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Roll Angle Controller/Roll damping inversion/Servo and Sensor Filter Delay/Servo delay/MATLAB Function'
//  '<S133>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Roll Angle Controller/Servo and Sensor Filter Delay/PT2 discrete with saturation5'
//  '<S134>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Roll Angle Controller/Servo and Sensor Filter Delay/PT2 discrete with saturation6'
//  '<S135>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Roll Angle Controller/Servo and Sensor Filter Delay/Servo delay'
//  '<S136>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Roll Angle Controller/Servo and Sensor Filter Delay/Servo delay/MATLAB Function'
//  '<S137>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Rudder Decay Controller/Rudder command'
//  '<S138>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Turn Coordination/Turn Coordination'
//  '<S139>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Waypoint Navigation/Avoid zero speed'
//  '<S140>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Waypoint Navigation/Flight Path Smoothing'
//  '<S141>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Waypoint Navigation/Look Ahead'
//  '<S142>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Waypoint Navigation/Look Ahead1'
//  '<S143>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Waypoint Navigation/Split waypoints and velocity'
//  '<S144>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Waypoint Navigation/Waypoint radius'
//  '<S145>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Waypoint Navigation/WpNav Matching'
//  '<S146>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Waypoint Navigation/Flight Path Smoothing/Compare To Constant'
//  '<S147>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Waypoint Navigation/Flight Path Smoothing/Flight Path Smoothing'
//  '<S148>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Waypoint Navigation/Flight Path Smoothing/Pass-through'
//  '<S149>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Waypoint Navigation/Flight Path Smoothing/Flight Path Smoothing/PT2 discrete with saturation3'
//  '<S150>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Waypoint Navigation/Flight Path Smoothing/Flight Path Smoothing/PT2 discrete with saturation4'
//  '<S151>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Waypoint Navigation/Flight Path Smoothing/Flight Path Smoothing/PT2 discrete with saturation5'
//  '<S152>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Waypoint Navigation/Waypoint radius/MATLAB Function'
//  '<S153>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Yaw Rate Controller/Feedback Gains'
//  '<S154>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Yaw Rate Controller/PT1 discrete with saturations1'
//  '<S155>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Yaw Rate Controller/Servo and Sensor Filter Delay'
//  '<S156>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Yaw Rate Controller/Servo and Sensor Filter Delay/PT2 discrete with saturation5'
//  '<S157>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Yaw Rate Controller/Servo and Sensor Filter Delay/PT2 discrete with saturation6'
//  '<S158>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Yaw Rate Controller/Servo and Sensor Filter Delay/Servo delay'
//  '<S159>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Yaw Rate Controller/Servo and Sensor Filter Delay/Servo delay/MATLAB Function'
//  '<S160>' : 'ArduPlane_LindiPlane/Subsystem/MATLAB Function'
//  '<S161>' : 'ArduPlane_LindiPlane/Subsystem/Subsystem'
//  '<S162>' : 'ArduPlane_LindiPlane/Subsystem/Subsystem1'
//  '<S163>' : 'ArduPlane_LindiPlane/log muxer/Auxiliary function to define log_config in generated C++ code'

#endif                                 // RTW_HEADER_MatlabController_h_

//
// File trailer for generated code.
//
// [EOF]
//
