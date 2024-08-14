//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: MatlabController.h
//
// Code generated for Simulink model 'ArduPlane_LindiPlane'.
//
// Model version                  : 1.775
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Mon Aug 12 18:23:58 2024
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

#ifndef DEFINED_TYPEDEF_FOR_struct_MJUKigG23kBOsE76s4pAhB_
#define DEFINED_TYPEDEF_FOR_struct_MJUKigG23kBOsE76s4pAhB_

typedef struct {
  real32_T cla[10];
  real32_T dadf[10];
  real32_T dfdu[10];
  real32_T s[10];
  real32_T rotx[10];
  real32_T x[10];
  real32_T y[10];
  real32_T z[10];
  real32_T m[10];
  real32_T xm[10];
} struct_MJUKigG23kBOsE76s4pAhB;

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

#ifndef DEFINED_TYPEDEF_FOR_struct_gXBcg1A6eW6dnMqVoWYS8C_
#define DEFINED_TYPEDEF_FOR_struct_gXBcg1A6eW6dnMqVoWYS8C_

typedef struct {
  real32_T omega;
  real32_T boost;
} struct_gXBcg1A6eW6dnMqVoWYS8C;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_qZW4wEBszjZZRXHZEeXB7E_
#define DEFINED_TYPEDEF_FOR_struct_qZW4wEBszjZZRXHZEeXB7E_

typedef struct {
  real32_T omega;
  real32_T d;
} struct_qZW4wEBszjZZRXHZEeXB7E;

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

#ifndef DEFINED_TYPEDEF_FOR_struct_N3y5cN6hTzLfF948kCjh1G_
#define DEFINED_TYPEDEF_FOR_struct_N3y5cN6hTzLfF948kCjh1G_

typedef struct {
  real32_T clp;
  real32_T b;
  real32_T s;
  real32_T cla_h;
  real32_T x_h;
  real32_T s_h;
} struct_N3y5cN6hTzLfF948kCjh1G;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_wtF2XFOtOELBemGDyRar7G_
#define DEFINED_TYPEDEF_FOR_struct_wtF2XFOtOELBemGDyRar7G_

typedef struct {
  real32_T pos;
  real32_T vel;
  real32_T acc;
} struct_wtF2XFOtOELBemGDyRar7G;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_ZpRNaqoZiEb6XqsMOrzR0C_
#define DEFINED_TYPEDEF_FOR_struct_ZpRNaqoZiEb6XqsMOrzR0C_

typedef struct {
  real32_T T;
  real32_T wprad;
  real32_T eposmax;
} struct_ZpRNaqoZiEb6XqsMOrzR0C;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_pHL6i6dtl77VVkO57r2aiB_
#define DEFINED_TYPEDEF_FOR_struct_pHL6i6dtl77VVkO57r2aiB_

typedef struct {
  struct_wtF2XFOtOELBemGDyRar7G k;
  struct_ZpRNaqoZiEb6XqsMOrzR0C rm;
} struct_pHL6i6dtl77VVkO57r2aiB;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_2ZbzmTCPZOzo1j7KmkMmtF_
#define DEFINED_TYPEDEF_FOR_struct_2ZbzmTCPZOzo1j7KmkMmtF_

typedef struct {
  real32_T u_min[10];
  real32_T u_max[10];
  real32_T u_d[10];
  real32_T W_v[3];
  real32_T W_u[10];
  real32_T gamma;
  real32_T W[10];
  real32_T i_max;
} struct_2ZbzmTCPZOzo1j7KmkMmtF;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_rQ7NuXJonI6pqr2AXxgEAD_
#define DEFINED_TYPEDEF_FOR_struct_rQ7NuXJonI6pqr2AXxgEAD_

typedef struct {
  real32_T W_v[3];
  real32_T W_u[10];
  real32_T gamma;
  real32_T i_max;
} struct_rQ7NuXJonI6pqr2AXxgEAD;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_yVa4qmao2SCnxORA4Uk5SD_
#define DEFINED_TYPEDEF_FOR_struct_yVa4qmao2SCnxORA4Uk5SD_

typedef struct {
  real32_T use;
  real32_T eta_np[2];
  struct_rQ7NuXJonI6pqr2AXxgEAD ca;
} struct_yVa4qmao2SCnxORA4Uk5SD;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_JfqfdhZnKx0Xf9TZzodQYG_
#define DEFINED_TYPEDEF_FOR_struct_JfqfdhZnKx0Xf9TZzodQYG_

typedef struct {
  struct_MJUKigG23kBOsE76s4pAhB cef;
  struct_NVgzPUNgrhoXyl8OD5Cw5F ceb;
  struct_gXBcg1A6eW6dnMqVoWYS8C servo;
  struct_qZW4wEBszjZZRXHZEeXB7E sflt;
  struct_sbKx6IyVlWG2aZGxKuo5AB aspd;
  struct_zFuKxOdy6xWCgrwijIt3EB atc;
  struct_N3y5cN6hTzLfF948kCjh1G eig;
  struct_pHL6i6dtl77VVkO57r2aiB psc;
  struct_2ZbzmTCPZOzo1j7KmkMmtF ca;
  struct_yVa4qmao2SCnxORA4Uk5SD mla;
} struct_JfqfdhZnKx0Xf9TZzodQYG;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_8e4ElBjVTPe7Rw8vcm14eD_
#define DEFINED_TYPEDEF_FOR_struct_8e4ElBjVTPe7Rw8vcm14eD_

typedef struct {
  real32_T ts;
} struct_8e4ElBjVTPe7Rw8vcm14eD;

#endif

// Custom Type definition for MATLAB Function: '<Root>/Quaternions to Rotation Matrix' 
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
  real32_T Merge_m[10];                // '<S14>/Merge'
  real32_T u[11];
  real32_T Euler_angles_f[3];
  real32_T Omega_Kb_f[3];
  real32_T Omega_Kb_dt_f[3];
  real32_T e_s_g[3];
  real32_T e_s_g_dt[3];
  real32_T e_s_g_dt2[3];
  real32_T s_g_ref[3];                 // '<S5>/BusConversion_InsertedFor_ap_bus_at_inport_0' 
  real32_T s_g[3];                     // '<S5>/BusConversion_InsertedFor_ap_bus_at_inport_0' 
  real32_T s_g_match[3];
  real32_T p_match[3];                 // '<S25>/WpNav Matching'
  real32_T e_s_g_e[3];                 // '<S16>/Add'
  real32_T e_s_g_dt_i[3];              // '<S16>/Add1'
  real32_T e_s_g_dt2_k[3];             // '<S16>/Add2'
  real32_T nu[2];                      // '<S16>/Add5'
  real32_T s_g_ref_d[3];               // '<S16>/BusConversion_InsertedFor_pos_control_at_inport_0' 
  real32_T s_g_a[3];                   // '<S16>/BusConversion_InsertedFor_pos_control_at_inport_0' 
  real32_T DiscreteTimeIntegratory_DSTAT_b[9];// '<S39>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_DSTAT_n[3];// '<S40>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_DSTA_n2[3];// '<S41>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_DSTAT_j[3];// '<S42>/Discrete-Time Integrator y' 
  real32_T Delay_DSTATE[6];            // '<S38>/Delay'
  real32_T Delay_DSTATE_h[6];          // '<S37>/Delay'
  real32_T UnitDelay1_DSTATE[10];      // '<S69>/Unit Delay1'
  real32_T UnitDelay2_DSTATE[10];      // '<S69>/Unit Delay2'
  real32_T DiscreteTimeIntegratory_DSTAT_l[10];// '<S74>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_DSTA_j2[10];// '<S73>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegrator1_DSTATE[10];// '<S79>/Discrete-Time Integrator1' 
  real32_T DiscreteTimeIntegrator_DSTATE_l[10];// '<S79>/Discrete-Time Integrator' 
  real32_T DiscreteTimeIntegrator1_DSTAT_j[10];// '<S80>/Discrete-Time Integrator1' 
  real32_T DiscreteTimeIntegrator_DSTATE_g[10];// '<S80>/Discrete-Time Integrator' 
  real32_T DiscreteTimeIntegratory_dt_DS_o[10];// '<S74>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_DS_n[10];// '<S73>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_D_ni[9];// '<S39>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_DS_e[3];// '<S42>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_DS_p[3];// '<S41>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_DS_i[3];// '<S40>/Discrete-Time Integrator y_dt' 
  real32_T UnitDelay4_DSTATE[15];      // '<S25>/Unit Delay4'
  real32_T DiscreteTimeIntegratory_DSTA_nm[3];// '<S50>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_D_jv[3];// '<S50>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTA_e3[3];// '<S49>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_D_ip[3];// '<S49>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTAT_k[3];// '<S51>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DS_c[3];// '<S51>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTA_df[3];// '<S30>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_D_nq[3];// '<S30>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTA_jr[3];// '<S29>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_D_pb[3];// '<S29>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTA_ff[3];// '<S28>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_D_be[3];// '<S28>/Discrete-Time Integrator y_dt' 
  real32_T Merge;                      // '<S5>/Merge'
  real32_T Merge2;                     // '<S5>/Merge2'
  real32_T Merge1;                     // '<S5>/Merge1'
  real32_T iter;
  real32_T wp_idx;
  real32_T stage;
  real32_T t;
  real32_T V_A_f;
  real32_T stage_e;                    // '<S25>/Data Type Conversion8'
  real32_T wp_idx_j;                   // '<S25>/Data Type Conversion9'
  real32_T t_g;                        // '<S25>/WpNav Matching'
  real32_T q;                          // '<S24>/Turn Coordination'
  real32_T nu_q_dt_ptchcntrl;          // '<S18>/Gain'
  real32_T DiscreteTimeIntegratory_DSTATE;// '<S43>/Discrete-Time Integrator y'
  real32_T DiscreteTimeIntegrator_DSTATE;// '<S38>/Discrete-Time Integrator'
  real32_T UnitDelay_DSTATE;           // '<S38>/Unit Delay'
  real32_T DiscreteTimeIntegrator_DSTATE_c;// '<S37>/Discrete-Time Integrator'
  real32_T UnitDelay_DSTATE_a;         // '<S37>/Unit Delay'
  real32_T DiscreteTimeIntegratory_dt_DSTA;// '<S84>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTAT_h;// '<S92>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegrator_DSTATE_e;// '<S61>/Discrete-Time Integrator'
  real32_T DiscreteTimeIntegratory_DSTAT_i;// '<S63>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_DSTAT_e;// '<S62>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegrator_DSTATE_h;// '<S66>/Discrete-Time Integrator'
  real32_T DiscreteTimeIntegratory_DSTA_hy;// '<S67>/Discrete-Time Integrator y' 
  real32_T UnitDelay_DSTATE_f;         // '<S21>/Unit Delay'
  real32_T DiscreteTimeIntegratory_DSTAT_p;// '<S84>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_DSTA_bw;// '<S85>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_DSTAT_m;// '<S86>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_DSTAT_f;// '<S87>/Discrete-Time Integrator y' 
  real32_T UnitDelay_DSTATE_fx;        // '<S22>/Unit Delay'
  real32_T DiscreteTimeIntegrator_DSTATE_o;// '<S100>/Discrete-Time Integrator'
  real32_T DiscreteTimeIntegratory_DSTA_f0;// '<S102>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_DSTAT_o;// '<S101>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_D_oo;// '<S101>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_DS_j;// '<S102>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_DS_l;// '<S87>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_DS_f;// '<S86>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_D_pv;// '<S85>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_DS_b;// '<S67>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTAT_d;// '<S68>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_D_lv;// '<S68>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_D_eo;// '<S62>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_D_nr;// '<S63>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_DS_g;// '<S92>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTAT_g;// '<S93>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_D_ir;// '<S93>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_D_gf;// '<S43>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegrator_DSTATE_p;// '<S57>/Discrete-Time Integrator'
  real32_T DiscreteTimeIntegrator_DSTATE_a;// '<S58>/Discrete-Time Integrator'
  real32_T DiscreteTimeIntegratory_DSTA_pe;// '<S59>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DS_m;// '<S59>/Discrete-Time Integrator y_dt' 
  real32_T UnitDelay1_DSTATE_d;        // '<S17>/Unit Delay1'
  real32_T DiscreteTimeIntegrator_DSTATE_f;// '<S54>/Discrete-Time Integrator'
  int32_T UnitDelay_DSTATE_b;          // '<S25>/Unit Delay'
  int32_T UnitDelay1_DSTATE_g;         // '<S25>/Unit Delay1'
  int32_T UnitDelay2_DSTATE_i;         // '<S25>/Unit Delay2'
  int32_T UnitDelay3_DSTATE;           // '<S25>/Unit Delay3'
  uint8_T DiscreteTimeIntegratory_IC_LOAD;// '<S43>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_LO_a;// '<S39>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_LO_e;// '<S41>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_LO_i;// '<S42>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegrator_IC_LOADI;// '<S61>/Discrete-Time Integrator'
  uint8_T DiscreteTimeIntegratory_IC_LO_m;// '<S63>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_LO_j;// '<S62>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_LO_b;// '<S85>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_L_an;// '<S86>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_LO_c;// '<S87>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegrator_IC_LOA_g;// '<S100>/Discrete-Time Integrator'
  uint8_T DiscreteTimeIntegratory_IC_L_jp;// '<S102>/Discrete-Time Integrator y' 
  uint8_T DiscreteTimeIntegratory_IC_L_bc;// '<S101>/Discrete-Time Integrator y' 
  uint8_T DiscreteTimeIntegrator_IC_LOA_m;// '<S57>/Discrete-Time Integrator'
  uint8_T DiscreteTimeIntegrator_IC_LO_mz;// '<S58>/Discrete-Time Integrator'
  uint8_T DiscreteTimeIntegratory_IC_L_id;// '<S59>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegrator_IC_LOA_l;// '<S54>/Discrete-Time Integrator'
  uint8_T DiscreteTimeIntegratory_IC_LO_d;// '<S50>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_LO_p;// '<S49>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_LO_l;// '<S51>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_L_iy;// '<S30>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_L_ld;// '<S29>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_LO_h;// '<S28>/Discrete-Time Integrator y'
  boolean_T UnitDelay5_DSTATE;         // '<S25>/Unit Delay5'
  boolean_T LindiPlaneAutopilot_MODE;  // '<Root>/LindiPlane Autopilot'
  boolean_T PitchAngleController_MODE; // '<S5>/Pitch Angle Controller'
  boolean_T OuterLoopINDI_MODE;        // '<S5>/Outer Loop INDI'
  boolean_T NDIPositionController_MODE;// '<S5>/NDI Position Controller'
  boolean_T ManeuverLoadAlleviation_MODE;// '<S14>/Maneuver Load Alleviation'
  boolean_T FlightPathSmoothing_MODE;  // '<S5>/Flight Path Smoothing'
} DW;

// Constant parameters (default storage)
typedef struct {
  // Computed Parameter: Gain_Gain_o
  //  Referenced by: '<S8>/Gain'

  real32_T Gain_Gain_o[8];

  // Expression: ch_fcn
  //  Referenced by: '<S1>/Constant'

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
  real32_T logs[67];                   // '<Root>/logs'
} ExtY;

// Parameters (default storage)
struct P_ {
  struct_JfqfdhZnKx0Xf9TZzodQYG lindi; // Variable: lindi
                                       //  Referenced by:
                                       //    '<S5>/Cmd 2 Yaw Rate'
                                       //    '<S10>/Gain'
                                       //    '<S14>/Constant'
                                       //    '<S15>/Constant'
                                       //    '<S16>/Gain'
                                       //    '<S16>/Gain1'
                                       //    '<S16>/Gain2'
                                       //    '<S18>/Gain'
                                       //    '<S18>/cmd 2 angle'
                                       //    '<S20>/Saturation3'
                                       //    '<S21>/Saturation'
                                       //    '<S22>/Gain5'
                                       //    '<S25>/Constant1'
                                       //    '<S25>/Constant2'
                                       //    '<S25>/Constant3'
                                       //    '<S25>/Constant5'
                                       //    '<S28>/omega'
                                       //    '<S29>/omega'
                                       //    '<S30>/omega'
                                       //    '<S32>/Maneuver Load Alleviation'
                                       //    '<S32>/Constant'
                                       //    '<S32>/Constant2'
                                       //    '<S33>/Constant'
                                       //    '<S37>/Constant1'
                                       //    '<S37>/Constant2'
                                       //    '<S38>/Constant1'
                                       //    '<S38>/Constant2'
                                       //    '<S39>/d'
                                       //    '<S39>/omega'
                                       //    '<S40>/d'
                                       //    '<S40>/omega'
                                       //    '<S41>/d'
                                       //    '<S41>/omega'
                                       //    '<S42>/d'
                                       //    '<S42>/omega'
                                       //    '<S43>/omega'
                                       //    '<S49>/omega'
                                       //    '<S50>/omega'
                                       //    '<S51>/omega'
                                       //    '<S54>/T'
                                       //    '<S57>/T'
                                       //    '<S58>/T'
                                       //    '<S59>/omega'
                                       //    '<S60>/Gain3'
                                       //    '<S60>/Gain5'
                                       //    '<S61>/T'
                                       //    '<S62>/omega'
                                       //    '<S63>/omega'
                                       //    '<S64>/Constant6'
                                       //    '<S64>/Constant7'
                                       //    '<S64>/Constant8'
                                       //    '<S64>/Gain14'
                                       //    '<S64>/Gain15'
                                       //    '<S64>/Gain16'
                                       //    '<S64>/Gain5'
                                       //    '<S64>/Gain6'
                                       //    '<S64>/Gain7'
                                       //    '<S64>/Gain8'
                                       //    '<S73>/d'
                                       //    '<S73>/omega'
                                       //    '<S74>/omega'
                                       //    '<S76>/indiCeFlapFix'
                                       //    '<S76>/Constant'
                                       //    '<S83>/Gain1'
                                       //    '<S83>/Gain3'
                                       //    '<S83>/Gain5'
                                       //    '<S84>/omega'
                                       //    '<S85>/omega'
                                       //    '<S86>/omega'
                                       //    '<S87>/omega'
                                       //    '<S88>/Gain'
                                       //    '<S88>/Gain1'
                                       //    '<S88>/Gain3'
                                       //    '<S88>/Gain4'
                                       //    '<S99>/Gain3'
                                       //    '<S99>/Gain5'
                                       //    '<S100>/T'
                                       //    '<S101>/omega'
                                       //    '<S102>/omega'
                                       //    '<S67>/omega'
                                       //    '<S68>/omega'
                                       //    '<S77>/caIndiWls'
                                       //    '<S77>/Delta u_max'
                                       //    '<S79>/Gain'
                                       //    '<S79>/Gain2'
                                       //    '<S79>/Gain3'
                                       //    '<S80>/Gain'
                                       //    '<S80>/Gain2'
                                       //    '<S80>/Gain3'
                                       //    '<S92>/omega'
                                       //    '<S93>/omega'

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
  static const struct AP_Param::GroupInfo var_info_1[];
  static const struct AP_Param::GroupInfo var_info_2[];
  static const struct AP_Param::GroupInfo var_info_3[];
  static const struct AP_Param::GroupInfo var_info_4[];
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
  void cross(const real32_T a[30], const real32_T b[30], real32_T c[30]);
  real32_T xnrm2_f(const real32_T x[3]);
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
                 real32_T tau_data[], const int32_T jpvt_data[], real32_T B_3[11],
                 int32_T rankA, real32_T Y_data[], int32_T *Y_size);
  real32_T xnrm2(int32_T n, const real32_T x_data[], int32_T ix0);
  void xzlarf(int32_T m, int32_T n, int32_T iv0, real32_T tau, real32_T C_data[],
              int32_T ic0, real32_T work_data[]);
  void qrsolve(const real32_T A_data[], const int32_T A_size[2], const real32_T
               B_1[11], real32_T Y_data[], int32_T *Y_size);
  void mldivide(const real32_T A_data[], const int32_T A_size[2], const real32_T
                B_0[11], real32_T Y_data[], int32_T *Y_size);
  boolean_T any(const boolean_T x_data[], const int32_T *x_size);
  real32_T wls_alloc(const real32_T B_4[24], const real32_T v[3], const real32_T
                     umin[8], const real32_T umax[8], const real32_T Wv[9],
                     const real32_T Wu[64], const real32_T ud[8], real32_T gam,
                     real32_T u[8], real32_T W[8], real32_T imax);
  void LSQFromQR_a(const real32_T A_data[], const int32_T A_size[2], const
                   real32_T tau_data[], const int32_T jpvt_data[], real32_T B_8
                   [13], int32_T rankA, real32_T Y_data[], int32_T *Y_size);
  void xzlarf_f(int32_T m, int32_T n, int32_T iv0, real32_T tau, real32_T
                C_data[], int32_T ic0, real32_T work_data[]);
  void qrsolve_p(const real32_T A_data[], const int32_T A_size[2], const
                 real32_T B_6[13], real32_T Y_data[], int32_T *Y_size);
  void mldivide_i(const real32_T A_data[], const int32_T A_size[2], const
                  real32_T B_5[13], real32_T Y_data[], int32_T *Y_size);
  real32_T wls_alloc_f(const real32_T B_9[30], const real32_T v[3], const
                       real32_T umin[10], const real32_T umax[10], const
                       real32_T Wv[9], const real32_T Wu[100], const real32_T
                       ud[10], real32_T gam, real32_T u[10], real32_T W[10],
                       real32_T imax);
};

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S13>/Scope' : Unused code path elimination
//  Block '<S13>/Scope1' : Unused code path elimination
//  Block '<S13>/Scope2' : Unused code path elimination
//  Block '<S13>/Scope3' : Unused code path elimination
//  Block '<S16>/Scope' : Unused code path elimination
//  Block '<S76>/Display' : Unused code path elimination
//  Block '<S21>/Scope1' : Unused code path elimination
//  Block '<S1>/Data Type Conversion1' : Eliminate redundant data type conversion
//  Block '<S28>/Saturation' : Eliminated Saturate block
//  Block '<S29>/Saturation' : Eliminated Saturate block
//  Block '<S30>/Saturation' : Eliminated Saturate block
//  Block '<S32>/Reshape' : Reshape block reduction
//  Block '<S32>/Reshape1' : Reshape block reduction
//  Block '<S39>/Saturation' : Eliminated Saturate block
//  Block '<S40>/Saturation' : Eliminated Saturate block
//  Block '<S41>/Saturation' : Eliminated Saturate block
//  Block '<S42>/Saturation' : Eliminated Saturate block
//  Block '<S43>/Saturation' : Eliminated Saturate block
//  Block '<S15>/Reshape' : Reshape block reduction
//  Block '<S15>/Reshape1' : Reshape block reduction
//  Block '<S49>/Saturation' : Eliminated Saturate block
//  Block '<S50>/Saturation' : Eliminated Saturate block
//  Block '<S51>/Saturation' : Eliminated Saturate block
//  Block '<S17>/Gain4' : Eliminated nontunable gain of 1
//  Block '<S54>/Saturation' : Eliminated Saturate block
//  Block '<S58>/Saturation' : Eliminated Saturate block
//  Block '<S59>/Saturation' : Eliminated Saturate block
//  Block '<S61>/Saturation' : Eliminated Saturate block
//  Block '<S62>/Saturation' : Eliminated Saturate block
//  Block '<S63>/Saturation' : Eliminated Saturate block
//  Block '<S66>/Saturation' : Eliminated Saturate block
//  Block '<S67>/Saturation' : Eliminated Saturate block
//  Block '<S68>/Saturation' : Eliminated Saturate block
//  Block '<S73>/Saturation' : Eliminated Saturate block
//  Block '<S74>/Saturation' : Eliminated Saturate block
//  Block '<S79>/Gain1' : Eliminated nontunable gain of 1
//  Block '<S80>/Gain1' : Eliminated nontunable gain of 1
//  Block '<S84>/Saturation' : Eliminated Saturate block
//  Block '<S85>/Saturation' : Eliminated Saturate block
//  Block '<S86>/Saturation' : Eliminated Saturate block
//  Block '<S87>/Saturation' : Eliminated Saturate block
//  Block '<S92>/Saturation' : Eliminated Saturate block
//  Block '<S93>/Saturation' : Eliminated Saturate block
//  Block '<S25>/Data Type Conversion' : Eliminate redundant data type conversion
//  Block '<S25>/Data Type Conversion1' : Eliminate redundant data type conversion
//  Block '<S25>/Data Type Conversion3' : Eliminate redundant data type conversion
//  Block '<S25>/Data Type Conversion4' : Eliminate redundant data type conversion
//  Block '<S25>/Data Type Conversion5' : Eliminate redundant data type conversion
//  Block '<S100>/Saturation' : Eliminated Saturate block
//  Block '<S101>/Saturation' : Eliminated Saturate block
//  Block '<S102>/Saturation' : Eliminated Saturate block
//  Block '<S26>/Reshape' : Reshape block reduction


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
//  '<S1>'   : 'ArduPlane_LindiPlane/Actuator Muxer ArduPlane'
//  '<S2>'   : 'ArduPlane_LindiPlane/Compare To Constant'
//  '<S3>'   : 'ArduPlane_LindiPlane/Compare To Constant1'
//  '<S4>'   : 'ArduPlane_LindiPlane/Compare To Constant2'
//  '<S5>'   : 'ArduPlane_LindiPlane/LindiPlane Autopilot'
//  '<S6>'   : 'ArduPlane_LindiPlane/Quaternions to Rotation Matrix'
//  '<S7>'   : 'ArduPlane_LindiPlane/Remove velocity'
//  '<S8>'   : 'ArduPlane_LindiPlane/Subsystem'
//  '<S9>'   : 'ArduPlane_LindiPlane/log muxer'
//  '<S10>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Cmd 2 Roll Angle'
//  '<S11>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Compare To Constant'
//  '<S12>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Demux'
//  '<S13>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Flight Path Smoothing'
//  '<S14>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Maneuver Load Alleviation'
//  '<S15>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Measurement Filtering'
//  '<S16>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/NDI Position Controller'
//  '<S17>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Outer Loop INDI'
//  '<S18>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Angle Controller'
//  '<S19>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Rate Controller'
//  '<S20>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Plane Inner Loop INDI and Control Allocation'
//  '<S21>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Roll Angle Controller'
//  '<S22>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Rudder Decay Controller'
//  '<S23>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Sstick Command Bus'
//  '<S24>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Turn Coordination'
//  '<S25>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Waypoint Navigation'
//  '<S26>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Yaw Rate Controller'
//  '<S27>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/add'
//  '<S28>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Flight Path Smoothing/PT2 discrete with saturation3'
//  '<S29>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Flight Path Smoothing/PT2 discrete with saturation4'
//  '<S30>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Flight Path Smoothing/PT2 discrete with saturation5'
//  '<S31>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Maneuver Load Alleviation/Compare To Constant'
//  '<S32>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Maneuver Load Alleviation/Maneuver Load Alleviation'
//  '<S33>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Maneuver Load Alleviation/Subsystem'
//  '<S34>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Maneuver Load Alleviation/Maneuver Load Alleviation/Desired Specific Lift'
//  '<S35>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Maneuver Load Alleviation/Maneuver Load Alleviation/Maneuver Load Alleviation'
//  '<S36>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Maneuver Load Alleviation/Maneuver Load Alleviation/Desired Specific Lift/MATLAB Function'
//  '<S37>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Measurement Filtering/PT2 discrete ode1'
//  '<S38>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Measurement Filtering/PT2 discrete ode4'
//  '<S39>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Measurement Filtering/PT2 discrete with saturation1'
//  '<S40>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Measurement Filtering/PT2 discrete with saturation2'
//  '<S41>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Measurement Filtering/PT2 discrete with saturation3'
//  '<S42>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Measurement Filtering/PT2 discrete with saturation4'
//  '<S43>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Measurement Filtering/PT2 discrete with saturation5'
//  '<S44>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Measurement Filtering/Rotations matrix to Euler angles'
//  '<S45>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Measurement Filtering/PT2 discrete ode1/PT2 discrete ode4'
//  '<S46>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Measurement Filtering/PT2 discrete ode1/PT2 split'
//  '<S47>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Measurement Filtering/PT2 discrete ode4/PT2 discrete ode4'
//  '<S48>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Measurement Filtering/PT2 discrete ode4/PT2 split'
//  '<S49>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/NDI Position Controller/PT2 discrete with saturation'
//  '<S50>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/NDI Position Controller/PT2 discrete with saturation1'
//  '<S51>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/NDI Position Controller/PT2 discrete with saturation2'
//  '<S52>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Outer Loop INDI/Bank angle'
//  '<S53>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Outer Loop INDI/Outer Loop INDI'
//  '<S54>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Outer Loop INDI/PT1 discrete with saturations1'
//  '<S55>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Outer Loop INDI/Bank angle/DCM to quaternions1'
//  '<S56>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Outer Loop INDI/Bank angle/Quaternions to Euler angles1'
//  '<S57>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Angle Controller/PT1 discrete with saturations'
//  '<S58>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Angle Controller/PT1 discrete with saturations1'
//  '<S59>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Angle Controller/PT2 discrete with saturation1'
//  '<S60>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Rate Controller/Feedback Gains'
//  '<S61>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Rate Controller/PT1 discrete with saturations'
//  '<S62>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Rate Controller/PT2 discrete with saturation1'
//  '<S63>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Rate Controller/PT2 discrete with saturation2'
//  '<S64>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Rate Controller/Roll damping inversion'
//  '<S65>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Rate Controller/Roll damping inversion/MATLAB Function1'
//  '<S66>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Rate Controller/Roll damping inversion/PT1 discrete with saturations'
//  '<S67>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Rate Controller/Roll damping inversion/PT2 discrete with saturation1'
//  '<S68>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Pitch Rate Controller/Roll damping inversion/PT2 discrete with saturation3'
//  '<S69>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Plane Inner Loop INDI and Control Allocation/INDI wls control allocation'
//  '<S70>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Plane Inner Loop INDI and Control Allocation/Only rotational control effectiveness'
//  '<S71>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Plane Inner Loop INDI and Control Allocation/Only rotational control effectiveness1'
//  '<S72>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Plane Inner Loop INDI and Control Allocation/Only rotational control effectiveness2'
//  '<S73>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Plane Inner Loop INDI and Control Allocation/PT2 discrete with saturation'
//  '<S74>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Plane Inner Loop INDI and Control Allocation/PT2 discrete with saturation1'
//  '<S75>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Plane Inner Loop INDI and Control Allocation/booster'
//  '<S76>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Plane Inner Loop INDI and Control Allocation/control effectiveness'
//  '<S77>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Plane Inner Loop INDI and Control Allocation/INDI wls control allocation/INDI control allocation'
//  '<S78>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Plane Inner Loop INDI and Control Allocation/INDI wls control allocation/INDI control allocation/caIndiWls'
//  '<S79>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Plane Inner Loop INDI and Control Allocation/booster/booster1'
//  '<S80>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Plane Inner Loop INDI and Control Allocation/booster/booster2'
//  '<S81>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Plane Inner Loop INDI and Control Allocation/control effectiveness/indiCeFlapFix'
//  '<S82>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Roll Angle Controller/Avoid Angle Steps'
//  '<S83>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Roll Angle Controller/Feedback Gains'
//  '<S84>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Roll Angle Controller/PT2 discrete with saturation1'
//  '<S85>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Roll Angle Controller/PT2 discrete with saturation2'
//  '<S86>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Roll Angle Controller/PT2 discrete with saturation3'
//  '<S87>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Roll Angle Controller/PT2 discrete with saturation4'
//  '<S88>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Roll Angle Controller/Roll damping inversion'
//  '<S89>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Roll Angle Controller/Feedback Gains/angle error'
//  '<S90>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Roll Angle Controller/Feedback Gains/wrap angle'
//  '<S91>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Roll Angle Controller/Feedback Gains/wrap angle1'
//  '<S92>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Roll Angle Controller/Roll damping inversion/PT2 discrete with saturation1'
//  '<S93>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Roll Angle Controller/Roll damping inversion/PT2 discrete with saturation3'
//  '<S94>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Rudder Decay Controller/Rudder command'
//  '<S95>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Turn Coordination/Turn Coordination'
//  '<S96>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Waypoint Navigation/Look Ahead'
//  '<S97>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Waypoint Navigation/Look Ahead1'
//  '<S98>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Waypoint Navigation/WpNav Matching'
//  '<S99>'  : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Yaw Rate Controller/Feedback Gains'
//  '<S100>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Yaw Rate Controller/PT1 discrete with saturations1'
//  '<S101>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Yaw Rate Controller/PT2 discrete with saturation1'
//  '<S102>' : 'ArduPlane_LindiPlane/LindiPlane Autopilot/Yaw Rate Controller/PT2 discrete with saturation2'
//  '<S103>' : 'ArduPlane_LindiPlane/Subsystem/MATLAB Function'
//  '<S104>' : 'ArduPlane_LindiPlane/Subsystem/Subsystem'
//  '<S105>' : 'ArduPlane_LindiPlane/Subsystem/Subsystem1'
//  '<S106>' : 'ArduPlane_LindiPlane/log muxer/Auxiliary function to define log_config in generated C++ code'

#endif                                 // RTW_HEADER_MatlabController_h_

//
// File trailer for generated code.
//
// [EOF]
//
