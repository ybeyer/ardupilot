//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: MatlabController.h
//
// Code generated for Simulink model 'ArduCopter_MinnieLindiCopterFtc'.
//
// Model version                  : 1.445
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Tue Apr  4 18:44:03 2023
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
#ifndef ArduCopter_MinnieLindiCopterFtc_COMMON_INCLUDES_
# define ArduCopter_MinnieLindiCopterFtc_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 // ArduCopter_MinnieLindiCopterFtc_COMMON_INCLUDES_ 

// Macros for accessing real-time model data structure
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

#ifndef DEFINED_TYPEDEF_FOR_dtoSgl_sections_bus_
#define DEFINED_TYPEDEF_FOR_dtoSgl_sections_bus_

typedef struct {
  real32_T pos_x[6];
  real32_T pos_y[6];
  real32_T pos_z[6];
  real32_T vel[6];
  real32_T t;
  real32_T arc_length;
  real32_T distance;
  real32_T polynomial_degree;
} dtoSgl_sections_bus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_dtoSgl_trajectoryStructBus_
#define DEFINED_TYPEDEF_FOR_dtoSgl_trajectoryStructBus_

typedef struct {
  real32_T num_sections_max;
  real32_T num_sections_set;
  dtoSgl_sections_bus sections[4];
  real32_T active_section;
  real32_T current_time;
  real32_T arc_length;
  real32_T distance;
  boolean_T is_repeated_course;
  real32_T polynomial_degree;
} dtoSgl_trajectoryStructBus;

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

#ifndef DEFINED_TYPEDEF_FOR_struct_sP1q3HDJE1lywg1PS8jMsG_
#define DEFINED_TYPEDEF_FOR_struct_sP1q3HDJE1lywg1PS8jMsG_

typedef struct {
  real32_T leanmax;
  real32_T leandamp;
  real32_T yawratemax;
  real32_T leanfreq;
  real32_T yawratetc;
} struct_sP1q3HDJE1lywg1PS8jMsG;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_fmp4nqBWC4rULgUM9YPHLD_
#define DEFINED_TYPEDEF_FOR_struct_fmp4nqBWC4rULgUM9YPHLD_

typedef struct {
  real32_T u_min[4];
  real32_T u_max[4];
  real32_T u_d[4];
  real32_T W_v[16];
  real32_T W_u[16];
  real32_T gamma;
  real32_T W[4];
  real32_T i_max;
} struct_fmp4nqBWC4rULgUM9YPHLD;

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

#ifndef DEFINED_TYPEDEF_FOR_struct_VAW73qcsmorMGOuFyCcx3B_
#define DEFINED_TYPEDEF_FOR_struct_VAW73qcsmorMGOuFyCcx3B_

typedef struct {
  real_T pos_x[6];
  real_T pos_y[6];
  real_T pos_z[6];
  real_T vel[6];
  real_T t;
  real_T arc_length;
  real_T distance;
  real32_T polynomial_degree;
} struct_VAW73qcsmorMGOuFyCcx3B;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_GcSYgHEScYXcSSyZ6ZxhGE_
#define DEFINED_TYPEDEF_FOR_struct_GcSYgHEScYXcSSyZ6ZxhGE_

typedef struct {
  real32_T num_sections_max;
  real_T num_sections_set;
  struct_VAW73qcsmorMGOuFyCcx3B sections[4];
  real_T active_section;
  real_T current_time;
  real_T arc_length;
  real_T distance;
  boolean_T is_repeated_course;
  real32_T polynomial_degree;
} struct_GcSYgHEScYXcSSyZ6ZxhGE;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_hhWieQsLOg2nCzK7sanRFC_
#define DEFINED_TYPEDEF_FOR_struct_hhWieQsLOg2nCzK7sanRFC_

typedef struct {
  real32_T accumax;
  real32_T accdmax;
  real32_T veldmax;
  real32_T velumax;
  real32_T velxymax;
  real32_T accxymax;
  real32_T veltc;
} struct_hhWieQsLOg2nCzK7sanRFC;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_wtF2XFOtOELBemGDyRar7G_
#define DEFINED_TYPEDEF_FOR_struct_wtF2XFOtOELBemGDyRar7G_

typedef struct {
  real32_T pos;
  real32_T vel;
  real32_T acc;
} struct_wtF2XFOtOELBemGDyRar7G;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_U3vXfuAftwETLdXCHyHbn_
#define DEFINED_TYPEDEF_FOR_struct_U3vXfuAftwETLdXCHyHbn_

typedef struct {
  struct_hhWieQsLOg2nCzK7sanRFC rm;
  struct_wtF2XFOtOELBemGDyRar7G k;
} struct_U3vXfuAftwETLdXCHyHbn;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_t0r0dbw0Z6uQa2zAOkuWoH_
#define DEFINED_TYPEDEF_FOR_struct_t0r0dbw0Z6uQa2zAOkuWoH_

typedef struct {
  real32_T wpmax;
  boolean_T cycle;
  real32_T degree;
} struct_t0r0dbw0Z6uQa2zAOkuWoH;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_CVmGZLFPF0pRjVJ7mv3ro_
#define DEFINED_TYPEDEF_FOR_struct_CVmGZLFPF0pRjVJ7mv3ro_

typedef struct {
  real32_T lean;
  real32_T leanrate;
  real32_T leanacc;
  real32_T yaw;
  real32_T yawrate;
  real32_T yawacc;
} struct_CVmGZLFPF0pRjVJ7mv3ro;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_wPSyMEjWyBsuzVPD9drp3C_
#define DEFINED_TYPEDEF_FOR_struct_wPSyMEjWyBsuzVPD9drp3C_

typedef struct {
  struct_sP1q3HDJE1lywg1PS8jMsG rm;
  struct_CVmGZLFPF0pRjVJ7mv3ro k;
} struct_wPSyMEjWyBsuzVPD9drp3C;

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

#ifndef DEFINED_TYPEDEF_FOR_struct_AjMJOgu1Si8jWnlljeCYkD_
#define DEFINED_TYPEDEF_FOR_struct_AjMJOgu1Si8jWnlljeCYkD_

typedef struct {
  real32_T D;
  real32_T omega;
} struct_AjMJOgu1Si8jWnlljeCYkD;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_mMTmos5EX4scEbQKeeeQyG_
#define DEFINED_TYPEDEF_FOR_struct_mMTmos5EX4scEbQKeeeQyG_

typedef struct {
  struct_fmp4nqBWC4rULgUM9YPHLD ca;
  struct_t0r0dbw0Z6uQa2zAOkuWoH traj;
  struct_U3vXfuAftwETLdXCHyHbn psc;
  struct_wPSyMEjWyBsuzVPD9drp3C atc;
  struct_p46j2fCGmBuwCvCy3vmiNE cep;
  struct_tuPgovHsB2lMDL0qC2CahD ceb;
  real32_T mtc;
  real_T ts;
  struct_AjMJOgu1Si8jWnlljeCYkD sflt;
} struct_mMTmos5EX4scEbQKeeeQyG;

#endif

// Custom Type definition for MATLAB Function: '<S80>/DCM to quaternions'
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

#ifndef struct_emxArray_real32_T_1x5
#define struct_emxArray_real32_T_1x5

struct emxArray_real32_T_1x5
{
  real32_T data[5];
  int32_T size[2];
};

#endif                                 //struct_emxArray_real32_T_1x5

#ifndef typedef_emxArray_real32_T_1x5
#define typedef_emxArray_real32_T_1x5

typedef struct emxArray_real32_T_1x5 emxArray_real32_T_1x5;

#endif                                 //typedef_emxArray_real32_T_1x5

#ifndef struct_sKQLR94MqGzlZ6BjlviOSrB_tag
#define struct_sKQLR94MqGzlZ6BjlviOSrB_tag

struct sKQLR94MqGzlZ6BjlviOSrB_tag
{
  emxArray_real32_T_1x5 f1;
};

#endif                                 //struct_sKQLR94MqGzlZ6BjlviOSrB_tag

#ifndef typedef_b_cell_wrap_1
#define typedef_b_cell_wrap_1

typedef struct sKQLR94MqGzlZ6BjlviOSrB_tag b_cell_wrap_1;

#endif                                 //typedef_b_cell_wrap_1

// Block signals and states (default storage) for system '<Root>'
typedef struct {
  dtoSgl_trajectoryStructBus TrajMemory_PreviousInput;// '<S56>/TrajMemory'
  real_T NextOutput[4];                // '<S37>/White Noise'
  real32_T s_g_ref[3];                 // '<S2>/Merge3'
  real32_T s_g_ref_dt[3];              // '<S2>/Merge3'
  real32_T s_g_ref_dt2[3];             // '<S2>/Merge3'
  real32_T n_g_des[3];                 // '<S2>/Merge'
  real32_T Add[9];                     // '<S114>/Add'
  real32_T nu[3];                      // '<S18>/Add1'
  real32_T s_g_ref_f[3];               // '<S18>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' 
  real32_T s_g[3];                     // '<S18>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' 
  real32_T s_g_dt[3];                  // '<S18>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' 
  real32_T s_g_dt2[3];                 // '<S18>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' 
  real32_T Gain[3];                    // '<S27>/Gain'
  real32_T Delta_factors_G1[4];        // '<S26>/apply learn rate'
  real32_T Delay_DSTATE[4];            // '<S29>/Delay'
  real32_T DiscreteTimeIntegratory_DSTATE[3];// '<S78>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_DSTAT_a[3];// '<S77>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_DSTA_al[3];// '<S75>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DSTA[3];// '<S79>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTAT_p[9];// '<S76>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_DSTAT_m[3];// '<S102>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DS_o[3];// '<S102>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTAT_e[3];// '<S103>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DS_j[3];// '<S103>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTAT_k[3];// '<S79>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegrator_DSTATE_e[4];// '<S54>/Discrete-Time Integrator' 
  real32_T UnitDelay1_DSTATE[4];       // '<S45>/Unit Delay1'
  real32_T DiscreteTimeIntegratory_DSTAT_n[4];// '<S48>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DS_l[4];// '<S48>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_DS_b[9];// '<S76>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_D_o1[3];// '<S75>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_DS_n[3];// '<S77>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_D_nb[3];// '<S78>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTAT_l[9];// '<S115>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DS_e[9];// '<S115>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegrator_DSTATE_l[3];// '<S112>/Discrete-Time Integrator' 
  real32_T DiscreteTimeIntegrator_DSTAT_em[3];// '<S17>/Discrete-Time Integrator' 
  real32_T DiscreteTimeIntegratory_DSTA_lk[3];// '<S70>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DS_d[3];// '<S70>/Discrete-Time Integrator y_dt' 
  real32_T uvwxbMemory_PreviousInput[120];// '<S56>/uvwxbMemory'
  real32_T AnormAlfaRhoPhiMemory_PreviousI[4];// '<S56>/AnormAlfaRhoPhiMemory'
  real32_T StateVecMemory_PreviousInput[6];// '<S56>/StateVecMemory '
  real32_T cmd_lean_angle_01;          // '<S2>/Merge'
  real32_T lean_dir_angle_des;         // '<S2>/Merge'
  real32_T Merge1;                     // '<S2>/Merge1'
  real32_T Delta_nu_a_T;               // '<S14>/incremental thrust atti correction' 
  real32_T a_T_meas;                   // '<S14>/desired and measured specific thrust' 
  real32_T Gain1;                      // '<S27>/Gain1'
  real32_T Delta_factors_G2;           // '<S26>/apply learn rate2'
  real32_T Delay1_DSTATE;              // '<S30>/Delay1'
  real32_T UnitDelay_DSTATE;           // '<S20>/Unit Delay'
  real32_T DiscreteTimeIntegrator_DSTATE;// '<S7>/Discrete-Time Integrator'
  real32_T UnitDelay_DSTATE_c;         // '<S13>/Unit Delay'
  real32_T DiscreteTimeIntegrator_DSTATE_h;// '<S107>/Discrete-Time Integrator'
  real32_T DiscreteTimeIntegrator2_DSTATE;// '<S89>/Discrete-Time Integrator2'
  uint32_T RandSeed[4];                // '<S37>/White Noise'
  int8_T DiscreteTimeIntegrator_PrevRese;// '<S7>/Discrete-Time Integrator'
  uint8_T DiscreteTimeIntegratory_IC_LOAD;// '<S78>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_LO_n;// '<S77>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_LO_b;// '<S76>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_LO_i;// '<S102>/Discrete-Time Integrator y' 
  uint8_T DiscreteTimeIntegratory_IC_LO_a;// '<S103>/Discrete-Time Integrator y' 
  uint8_T DiscreteTimeIntegratory_IC_LO_d;// '<S79>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegrator2_IC_LOAD;// '<S89>/Discrete-Time Integrator2'
  uint8_T DiscreteTimeIntegratory_IC_L_b2;// '<S115>/Discrete-Time Integrator y' 
  uint8_T DiscreteTimeIntegrator_IC_LOADI;// '<S112>/Discrete-Time Integrator'
  uint8_T DiscreteTimeIntegrator_IC_LOA_o;// '<S17>/Discrete-Time Integrator'
  uint8_T DiscreteTimeIntegratory_IC_LO_e;// '<S70>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_L_eh;// '<S43>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_L_ir;// '<S42>/Discrete-Time Integrator y'
  boolean_T DelayInput1_DSTATE;        // '<S62>/Delay Input1'
  boolean_T NDIpositioncontrollerforcopters;// '<S2>/NDI position controller for copters reference model' 
  boolean_T CopterRandomExcitation_MODE;// '<S25>/Copter Random Excitation'
  boolean_T AdaptiveINDIG1andG2correction_M;// '<S25>/Adaptive INDI G1 and G2 correction' 
} DW;

// Constant parameters (default storage)
typedef struct {
  // Pooled Parameter (Mixed Expressions)
  //  Referenced by:
  //    '<S56>/Constant1'
  //    '<S56>/TrajMemory'

  dtoSgl_trajectoryStructBus pooled3;

  // Expression: ca
  //  Referenced by: '<S12>/Control Allocation Vertical Acc Weighting'

  struct_fmp4nqBWC4rULgUM9YPHLD ControlAllocationVerticalAccWei;

  // Expression: ca
  //  Referenced by: '<S50>/MATLAB Function2'

  struct_fmp4nqBWC4rULgUM9YPHLD MATLABFunction2_ca;

  // Expression: G10
  //  Referenced by: '<S49>/MATLAB Function'

  real32_T MATLABFunction_G10[16];

  // Expression: G20
  //  Referenced by: '<S49>/MATLAB Function'

  real32_T MATLABFunction_G20[16];

  // Expression: [psc.rm.veldmax,0,-psc.rm.velumax]
  //  Referenced by: '<S17>/1-D Lookup Table'

  real32_T uDLookupTable_tableData[3];

  // Computed Parameter: uDLookupTable_bp01Data
  //  Referenced by: '<S17>/1-D Lookup Table'

  real32_T uDLookupTable_bp01Data[3];

  // Computed Parameter: uDLookupTable_tableData_g
  //  Referenced by: '<Root>/1-D Lookup Table'

  real32_T uDLookupTable_tableData_g[2];

  // Pooled Parameter (Expression: [1000;2000])
  //  Referenced by:
  //    '<Root>/1-D Lookup Table'
  //    '<S8>/1-D Lookup Table'
  //    '<S8>/1-D Lookup Table1'

  real32_T pooled10[2];

  // Expression: simin.time
  //  Referenced by: '<S7>/Constant1'

  real32_T Constant1_Value[10];

  // Expression: simin.signals.values
  //  Referenced by: '<S7>/Constant2'

  real32_T Constant2_Value[40];

  // Pooled Parameter (Expression: [1;0])
  //  Referenced by:
  //    '<S8>/1-D Lookup Table'
  //    '<S8>/1-D Lookup Table1'

  real32_T pooled19[2];
} ConstP;

// External inputs (root inport signals with default storage)
typedef struct {
  cmdBus cmd;                          // '<Root>/cmd'
  measureBus measure;                  // '<Root>/measure'
} ExtU;

// External outputs (root outports fed by signals with default storage)
typedef struct {
  real32_T u[8];                       // '<Root>/u'
  real32_T logs[58];                   // '<Root>/logs'
} ExtY;

// Constant parameters (default storage)
extern const ConstP rtConstP;

// Exported data declaration

// Const memory section
// Declaration for custom storage class: Const
extern const logConfigBus log_config[5];

// Class declaration for model ArduCopter_MinnieLindiCopterFtc
class MatlabControllerClass {
  // public data and function members
 public:
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
  // Block signals and states
  DW rtDW;

  // private member function(s) for subsystem '<Root>'
  void trajSectionGetPos(const real32_T traj_section_pos_x[6], const real32_T
    traj_section_pos_y[6], const real32_T traj_section_pos_z[6], real_T
    varargin_1, real32_T pos[3]);
  void polyder_p(const real32_T u[6], real32_T a_data[], int32_T a_size[2]);
  real32_T polyVal(const real32_T p_data[], const int32_T p_size[2], real_T x);
  void trajSetArcLength(dtoSgl_trajectoryStructBus *traj);
  void polyder(const real_T u_data[], const int32_T u_size[2], real_T a_data[],
               int32_T a_size[2]);
  void polyInterpolationAx_f(real32_T num_of_splines, boolean_T cycle, const
    real32_T x_data[], const int32_T *x_size, real32_T b_data[], int32_T *b_size);
  real32_T norm(const real32_T x_data[], const int32_T *x_size);
  void polyInterpolationAx(real32_T num_of_splines, boolean_T cycle, const
    real32_T x_data[], const int32_T *x_size, real32_T b_data[], int32_T *b_size);
  real32_T norm_c(const real32_T x[2]);
  void ladacLsqrIterate(real32_T A_tunableEnvironment_f1, boolean_T
                        A_tunableEnvironment_f3, real32_T x_data[], int32_T
                        *x_size, real32_T w_data[], int32_T *w_size, real32_T
                        u_data[], int32_T *u_size, real32_T v_data[], int32_T
                        *v_size, real32_T *Anorm, real32_T *alfa, real32_T
                        *rhobar, real32_T *phibar);
  void polyInterpolationb(const real32_T points_data[], const int32_T
    points_size[2], boolean_T cycle, real32_T b_data[], int32_T *b_size,
    real32_T *num_of_splines);
  void ladacLsqrInit(real32_T A_tunableEnvironment_f1, boolean_T
                     A_tunableEnvironment_f3, const real32_T b_data[], const
                     int32_T *b_size, real32_T x_data[], int32_T *x_size,
                     real32_T w_data[], int32_T *w_size, real32_T u_data[],
                     int32_T *u_size, real32_T v_data[], int32_T *v_size,
                     real32_T *alfa, real32_T *rhobar, real32_T *phibar);
  boolean_T trajValidateWaypoints(uint16_T num_wp);
  void leanVectorNormDeriv2_c(const real32_T nn[3], const real32_T nn_dt[3],
    const real32_T nn_dt2[3], real_T n_dt2[3]);
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
};

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<Root>/Constant2' : Unused code path elimination
//  Block '<Root>/Constant3' : Unused code path elimination
//  Block '<S39>/Add' : Unused code path elimination
//  Block '<S39>/Add1' : Unused code path elimination
//  Block '<S39>/Add2' : Unused code path elimination
//  Block '<S39>/Matrix Multiply' : Unused code path elimination
//  Block '<S39>/Matrix Multiply1' : Unused code path elimination
//  Block '<S39>/Scope' : Unused code path elimination
//  Block '<S39>/Scope1' : Unused code path elimination
//  Block '<S39>/Scope2' : Unused code path elimination
//  Block '<S39>/Unit Delay' : Unused code path elimination
//  Block '<S41>/Constant' : Unused code path elimination
//  Block '<S41>/Scope' : Unused code path elimination
//  Block '<S2>/Constant' : Unused code path elimination
//  Block '<S12>/Gain' : Unused code path elimination
//  Block '<S57>/Add' : Unused code path elimination
//  Block '<S18>/Gain' : Unused code path elimination
//  Block '<S18>/Gain1' : Unused code path elimination
//  Block '<S116>/Data Type Duplicate' : Unused code path elimination
//  Block '<S116>/Data Type Propagation' : Unused code path elimination
//  Block '<S117>/Data Type Duplicate' : Unused code path elimination
//  Block '<S117>/Data Type Propagation' : Unused code path elimination
//  Block '<S5>/Data Type Duplicate' : Unused code path elimination
//  Block '<S5>/Data Type Propagation' : Unused code path elimination
//  Block '<S6>/Data Type Duplicate' : Unused code path elimination
//  Block '<S6>/Data Type Propagation' : Unused code path elimination
//  Block '<Root>/Data Type Conversion1' : Eliminate redundant data type conversion
//  Block '<Root>/Gain7' : Eliminated nontunable gain of 1
//  Block '<S26>/Reshape3' : Reshape block reduction
//  Block '<S38>/Saturation' : Eliminated Saturate block
//  Block '<S42>/Saturation' : Eliminated Saturate block
//  Block '<S43>/Saturation' : Eliminated Saturate block
//  Block '<S54>/Saturation' : Eliminated Saturate block
//  Block '<S48>/Saturation' : Eliminated Saturate block
//  Block '<S70>/Saturation' : Eliminated Saturate block
//  Block '<S75>/Saturation' : Eliminated Saturate block
//  Block '<S76>/Saturation' : Eliminated Saturate block
//  Block '<S77>/Saturation' : Eliminated Saturate block
//  Block '<S78>/Saturation' : Eliminated Saturate block
//  Block '<S79>/Saturation' : Eliminated Saturate block
//  Block '<S16>/Reshape' : Reshape block reduction
//  Block '<S16>/Reshape1' : Reshape block reduction
//  Block '<S16>/Reshape2' : Reshape block reduction
//  Block '<S89>/Gain1' : Eliminated nontunable gain of 1
//  Block '<S89>/Gain2' : Eliminated nontunable gain of 1
//  Block '<S107>/Saturation' : Eliminated Saturate block
//  Block '<S102>/Saturation' : Eliminated Saturate block
//  Block '<S103>/Saturation' : Eliminated Saturate block
//  Block '<S89>/Reshape' : Reshape block reduction
//  Block '<S115>/Saturation' : Eliminated Saturate block
//  Block '<S12>/Constant' : Unused code path elimination


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
//  '<Root>' : 'ArduCopter_MinnieLindiCopterFtc'
//  '<S1>'   : 'ArduCopter_MinnieLindiCopterFtc/Actuator muxer'
//  '<S2>'   : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1'
//  '<S3>'   : 'ArduCopter_MinnieLindiCopterFtc/MATLAB Function'
//  '<S4>'   : 'ArduCopter_MinnieLindiCopterFtc/Quaternions to Rotation Matrix'
//  '<S5>'   : 'ArduCopter_MinnieLindiCopterFtc/Saturation Dynamic'
//  '<S6>'   : 'ArduCopter_MinnieLindiCopterFtc/Saturation Dynamic1'
//  '<S7>'   : 'ArduCopter_MinnieLindiCopterFtc/automated stick inputs from workspace'
//  '<S8>'   : 'ArduCopter_MinnieLindiCopterFtc/kill motors'
//  '<S9>'   : 'ArduCopter_MinnieLindiCopterFtc/log muxer'
//  '<S10>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Accelerations to Reduced Attitude and Thrust'
//  '<S11>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Autotune'
//  '<S12>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Copter Inner Loop INDI and Control Allocation'
//  '<S13>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Flight path guidance with position controller reference'
//  '<S14>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Incremental specific thrust'
//  '<S15>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Measurement Filtering'
//  '<S16>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller'
//  '<S17>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/NDI position controller for copters reference model'
//  '<S18>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/NDI position controller for copters with reference input'
//  '<S19>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Pilot Reduced Attitude Commands'
//  '<S20>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/State Logic Bus'
//  '<S21>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Stick Command Bus'
//  '<S22>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Vertical Acc to Specific Thrust'
//  '<S23>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Accelerations to Reduced Attitude and Thrust/INDI Copter Acc 2 Lean Vector'
//  '<S24>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Accelerations to Reduced Attitude and Thrust/MATLAB Function4'
//  '<S25>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Autotune/Copter INDI Autotune'
//  '<S26>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Autotune/Copter INDI Autotune/Adaptive INDI G1 and G2 correction'
//  '<S27>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Autotune/Copter INDI Autotune/Copter Random Excitation'
//  '<S28>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Autotune/Copter INDI Autotune/INDI Inversion Check'
//  '<S29>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Autotune/Copter INDI Autotune/accumulate G1 correction factors'
//  '<S30>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Autotune/Copter INDI Autotune/accumulate G2 correction factor'
//  '<S31>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Autotune/Copter INDI Autotune/create diag'
//  '<S32>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Autotune/Copter INDI Autotune/Adaptive INDI G1 and G2 correction/G1 learn rate'
//  '<S33>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Autotune/Copter INDI Autotune/Adaptive INDI G1 and G2 correction/G2 learn rate'
//  '<S34>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Autotune/Copter INDI Autotune/Adaptive INDI G1 and G2 correction/Gradient of Delta_nu w.r.t. G1 correction factors'
//  '<S35>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Autotune/Copter INDI Autotune/Adaptive INDI G1 and G2 correction/Gradient of Delta_nu w.r.t. G2 correction factor'
//  '<S36>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Autotune/Copter INDI Autotune/Adaptive INDI G1 and G2 correction/Inversion error'
//  '<S37>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Autotune/Copter INDI Autotune/Copter Random Excitation/Band-Limited White Noise'
//  '<S38>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Autotune/Copter INDI Autotune/Copter Random Excitation/PT2 discrete with saturation'
//  '<S39>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Autotune/Copter INDI Autotune/INDI Inversion Check/Inversion forward'
//  '<S40>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Autotune/Copter INDI Autotune/INDI Inversion Check/filtered Delta nu'
//  '<S41>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Autotune/Copter INDI Autotune/INDI Inversion Check/filtered Delta u'
//  '<S42>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Autotune/Copter INDI Autotune/INDI Inversion Check/filtered Delta nu/PT2 discrete with saturation1'
//  '<S43>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Autotune/Copter INDI Autotune/INDI Inversion Check/filtered Delta u/PT2 discrete with saturation'
//  '<S44>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Copter Inner Loop INDI and Control Allocation/Control Allocation Vertical Acc Weighting'
//  '<S45>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Copter Inner Loop INDI and Control Allocation/INDI high level wls control allocation'
//  '<S46>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Copter Inner Loop INDI and Control Allocation/Is Manual Throttle Repmat'
//  '<S47>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Copter Inner Loop INDI and Control Allocation/Motor dynamics model'
//  '<S48>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Copter Inner Loop INDI and Control Allocation/Sensor filter model'
//  '<S49>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Copter Inner Loop INDI and Control Allocation/control effectiveness'
//  '<S50>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Copter Inner Loop INDI and Control Allocation/INDI high level wls control allocation/INDI control allocation'
//  '<S51>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Copter Inner Loop INDI and Control Allocation/INDI high level wls control allocation/Set Vertical Acc Weight To Zero'
//  '<S52>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Copter Inner Loop INDI and Control Allocation/INDI high level wls control allocation/Set Vertical Acc Weight To Zero1'
//  '<S53>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Copter Inner Loop INDI and Control Allocation/INDI high level wls control allocation/INDI control allocation/MATLAB Function2'
//  '<S54>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Copter Inner Loop INDI and Control Allocation/Motor dynamics model/PT1 discrete with saturations'
//  '<S55>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Copter Inner Loop INDI and Control Allocation/control effectiveness/MATLAB Function'
//  '<S56>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Flight path guidance with position controller reference/Trajectory from Waypoints'
//  '<S57>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Flight path guidance with position controller reference/flight-path velocity'
//  '<S58>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Flight path guidance with position controller reference/look ahead'
//  '<S59>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Flight path guidance with position controller reference/matching'
//  '<S60>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Flight path guidance with position controller reference/pick if traj is valid'
//  '<S61>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Flight path guidance with position controller reference/reference'
//  '<S62>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Flight path guidance with position controller reference/Trajectory from Waypoints/Detect Rise Positive'
//  '<S63>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Flight path guidance with position controller reference/Trajectory from Waypoints/trajFromWaypoints'
//  '<S64>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Flight path guidance with position controller reference/Trajectory from Waypoints/Detect Rise Positive/Positive'
//  '<S65>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Flight path guidance with position controller reference/flight-path velocity/absolute flight-path velocity'
//  '<S66>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Flight path guidance with position controller reference/look ahead/flight path look ahead'
//  '<S67>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Flight path guidance with position controller reference/matching/flight path matching'
//  '<S68>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Flight path guidance with position controller reference/reference/position controller reference from flight path'
//  '<S69>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Incremental specific thrust/DCM 2 Lean Vector'
//  '<S70>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Incremental specific thrust/PT2 Lean Vector'
//  '<S71>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Incremental specific thrust/desired and measured specific thrust'
//  '<S72>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Incremental specific thrust/incremental thrust atti correction'
//  '<S73>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Incremental specific thrust/PT2 Lean Vector/n ref norm'
//  '<S74>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Measurement Filtering/MATLAB Function'
//  '<S75>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Measurement Filtering/PT2 discrete with saturation'
//  '<S76>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Measurement Filtering/PT2 discrete with saturation1'
//  '<S77>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Measurement Filtering/PT2 discrete with saturation2'
//  '<S78>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Measurement Filtering/PT2 discrete with saturation3'
//  '<S79>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Measurement Filtering/PT2 discrete with saturation4'
//  '<S80>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Measurement Filtering/measured yaw'
//  '<S81>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Measurement Filtering/measured yaw/DCM to quaternions'
//  '<S82>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Measurement Filtering/measured yaw/Quaternion Reduced'
//  '<S83>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/Reduced Attitude Scheduler'
//  '<S84>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/error computation'
//  '<S85>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/measure'
//  '<S86>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/ny control'
//  '<S87>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/ny from reference'
//  '<S88>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/ny measured'
//  '<S89>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/reference model'
//  '<S90>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/Reduced Attitude Scheduler/Reduced Attitude Weighting Factors'
//  '<S91>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/error computation/angle error'
//  '<S92>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/error computation/wrap angle'
//  '<S93>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/error computation/wrap angle1'
//  '<S94>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/measure/DCM to quaternions'
//  '<S95>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/measure/Quaternion Reduced'
//  '<S96>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/reference model/Desired Roll Pitch'
//  '<S97>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/reference model/Lean Vector Derivative Trafo'
//  '<S98>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/reference model/Lean Vector Derivative Trafo Delay'
//  '<S99>'  : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/reference model/Lean Vector Derivative to Omega'
//  '<S100>' : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/reference model/MATLAB Function'
//  '<S101>' : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/reference model/PT1 discrete reference model2'
//  '<S102>' : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/reference model/PT2 Lean Vector'
//  '<S103>' : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/reference model/PT2 Lean Vector1'
//  '<S104>' : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/reference model/Pseudo-Control Roll Pitch'
//  '<S105>' : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/reference model/Simulink Trickster'
//  '<S106>' : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/reference model/lean angles 2 lean vector'
//  '<S107>' : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/reference model/PT1 discrete reference model2/PT1 discrete with saturations'
//  '<S108>' : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/reference model/PT2 Lean Vector/n ref norm'
//  '<S109>' : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Multicopter (Reduced) Attitude INDI Controller/reference model/PT2 Lean Vector1/n ref norm'
//  '<S110>' : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/NDI position controller for copters reference model/MATLAB Function1'
//  '<S111>' : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/NDI position controller for copters reference model/PT1 discrete reference model'
//  '<S112>' : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/NDI position controller for copters reference model/PT1 discrete reference model/PT1 discrete with saturations'
//  '<S113>' : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/NDI position controller for copters with reference input/measures'
//  '<S114>' : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/NDI position controller for copters with reference input/position controller'
//  '<S115>' : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/NDI position controller for copters with reference input/position controller/PT2 discrete with saturation'
//  '<S116>' : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/NDI position controller for copters with reference input/position controller/Saturation Dynamic'
//  '<S117>' : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/NDI position controller for copters with reference input/position controller/Saturation Dynamic1'
//  '<S118>' : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/NDI position controller for copters with reference input/position controller/acccntrlmax'
//  '<S119>' : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/NDI position controller for copters with reference input/position controller/eposmax'
//  '<S120>' : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Pilot Reduced Attitude Commands/MATLAB Function'
//  '<S121>' : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Pilot Reduced Attitude Commands/MATLAB Function1'
//  '<S122>' : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/State Logic Bus/LindiCopter State Logic'
//  '<S123>' : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Stick Command Bus/throttle_-1_1 to throttle_0_1'
//  '<S124>' : 'ArduCopter_MinnieLindiCopterFtc/LindiCopter Autopilot1/Vertical Acc to Specific Thrust/MATLAB Function'
//  '<S125>' : 'ArduCopter_MinnieLindiCopterFtc/automated stick inputs from workspace/interpHold'
//  '<S126>' : 'ArduCopter_MinnieLindiCopterFtc/log muxer/Auxiliary function to define log_config in generated C++ code'

#endif                                 // RTW_HEADER_MatlabController_h_

//
// File trailer for generated code.
//
// [EOF]
//
