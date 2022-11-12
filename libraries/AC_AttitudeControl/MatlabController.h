//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: MatlabController.h
//
// Code generated for Simulink model 'ArduCopter_MinnieLoiterFtc'.
//
// Model version                  : 1.410
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Sat Nov 12 15:27:04 2022
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
#ifndef ArduCopter_MinnieLoiterFtc_COMMON_INCLUDES_
# define ArduCopter_MinnieLoiterFtc_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 // ArduCopter_MinnieLoiterFtc_COMMON_INCLUDES_ 

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
  real32_T Omega_Kb_raw[3];
  real32_T omega_Kb[3];
  real32_T EulerAngles[3];
  real32_T q_bg[4];
  real32_T a_Kg[3];
  real32_T V_Kg[3];
  real32_T s_Kg[3];
  real32_T s_Kg_origin[3];
  real32_T lla[3];
  real32_T rangefinder[6];
  real32_T V_bat;
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

// Custom Type definition for MATLAB Function: '<S34>/DCM to quaternions'
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
  real32_T DiscreteTimeIntegratory_DSTATE[3];// '<S72>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegrator_DSTATE[3];// '<S55>/Discrete-Time Integrator'
  real32_T DiscreteTimeIntegratory_DSTAT_d[3];// '<S71>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_DSTAT_b[3];// '<S70>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegrator_DSTAT_my[3];// '<S59>/Discrete-Time Integrator' 
  real32_T DiscreteTimeIntegratory_DSTAT_l[9];// '<S62>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_DSTAT_k[9];// '<S69>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_DSTAT_j[3];// '<S49>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DSTA[3];// '<S49>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTAT_i[3];// '<S50>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DS_b[3];// '<S50>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTAT_g[3];// '<S68>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DS_m[3];// '<S68>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegrator_DSTATE_n[4];// '<S27>/Discrete-Time Integrator' 
  real32_T UnitDelay1_DSTATE[4];       // '<S20>/Unit Delay1'
  real32_T DiscreteTimeIntegratory_DSTA_j4[4];// '<S22>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DS_a[9];// '<S69>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_DS_h[9];// '<S62>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_DS_i[4];// '<S22>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_DS_f[3];// '<S70>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_D_b5[3];// '<S71>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_DS_n[3];// '<S72>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegrator_DSTATE_m;// '<S7>/Discrete-Time Integrator'
  real32_T DiscreteTimeIntegrator_DSTATE_b;// '<S52>/Discrete-Time Integrator'
  real32_T DiscreteTimeIntegrator2_DSTATE;// '<S38>/Discrete-Time Integrator2'
  real32_T DiscreteTimeIntegratory_DSTA_gx;// '<S31>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_D_mp;// '<S31>/Discrete-Time Integrator y_dt' 
  int8_T DiscreteTimeIntegrator_PrevRese;// '<S55>/Discrete-Time Integrator'
  int8_T DiscreteTimeIntegrator_PrevRe_h;// '<S7>/Discrete-Time Integrator'
  int8_T DiscreteTimeIntegrator_PrevRe_g;// '<S59>/Discrete-Time Integrator'
  uint8_T DiscreteTimeIntegratory_IC_LOAD;// '<S72>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegrator_IC_LOADI;// '<S55>/Discrete-Time Integrator'
  uint8_T DiscreteTimeIntegratory_IC_LO_f;// '<S71>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_LO_i;// '<S70>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegrator_IC_LOA_j;// '<S59>/Discrete-Time Integrator'
  uint8_T DiscreteTimeIntegratory_IC_LO_b;// '<S62>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_LO_g;// '<S69>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_LO_m;// '<S49>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_L_mh;// '<S50>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_LO_d;// '<S68>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegrator2_IC_LOAD;// '<S38>/Discrete-Time Integrator2'
  uint8_T DiscreteTimeIntegratory_IC_LO_j;// '<S31>/Discrete-Time Integrator y'
} DW;

// Constant parameters (default storage)
typedef struct {
  // Expression: ca
  //  Referenced by: '<S24>/MATLAB Function2'

  struct_fmp4nqBWC4rULgUM9YPHLD MATLABFunction2_ca;

  // Expression: G10
  //  Referenced by: '<S23>/MATLAB Function'

  real32_T MATLABFunction_G10[16];

  // Expression: G20
  //  Referenced by: '<S23>/MATLAB Function'

  real32_T MATLABFunction_G20[16];

  // Expression: simin.signals.values
  //  Referenced by: '<S7>/Constant2'

  real32_T Constant2_Value[40];

  // Pooled Parameter (Mixed Expressions)
  //  Referenced by:
  //    '<Root>/1-D Lookup Table'
  //    '<S20>/1-D Lookup Table2'

  real32_T pooled5[2];

  // Pooled Parameter (Expression: [1000;2000])
  //  Referenced by:
  //    '<Root>/1-D Lookup Table'
  //    '<S8>/1-D Lookup Table'
  //    '<S8>/1-D Lookup Table1'

  real32_T pooled6[2];

  // Expression: simin.time
  //  Referenced by: '<S7>/Constant1'

  real32_T Constant1_Value[10];

  // Expression: [psc.rm.veldmax,0,-psc.rm.velumax]
  //  Referenced by: '<S55>/1-D Lookup Table'

  real32_T uDLookupTable_tableData[3];

  // Computed Parameter: uDLookupTable_bp01Data
  //  Referenced by: '<S55>/1-D Lookup Table'

  real32_T uDLookupTable_bp01Data[3];

  // Expression: [-0.99999999*ca.gamma*[1,1],0,0]
  //  Referenced by: '<S20>/1-D Lookup Table1'

  real32_T uDLookupTable1_tableData[4];

  // Computed Parameter: uDLookupTable1_bp01Data
  //  Referenced by: '<S20>/1-D Lookup Table1'

  real32_T uDLookupTable1_bp01Data[4];

  // Pooled Parameter (Expression: [1;0.1])
  //  Referenced by:
  //    '<S8>/1-D Lookup Table'
  //    '<S8>/1-D Lookup Table1'

  real32_T pooled20[2];
} ConstP;

// External inputs (root inport signals with default storage)
typedef struct {
  cmdBus cmd;                          // '<Root>/cmd'
  measureBus measure;                  // '<Root>/measure'
} ExtU;

// External outputs (root outports fed by signals with default storage)
typedef struct {
  real32_T u[8];                       // '<Root>/u'
  real32_T logs[27];                   // '<Root>/logs'
} ExtY;

// Constant parameters (default storage)
extern const ConstP rtConstP;

// Exported data declaration

// Const memory section
// Declaration for custom storage class: Const
extern const logConfigBus log_config[5];

// Class declaration for model ArduCopter_MinnieLoiterFtc
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
  real32_T norm(const real32_T x[3]);
  real32_T divideFinite(real32_T B_5);
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
//  Block '<S2>/Constant' : Unused code path elimination
//  Block '<S10>/Gain' : Unused code path elimination
//  Block '<S56>/Gain' : Unused code path elimination
//  Block '<S56>/Gain1' : Unused code path elimination
//  Block '<S63>/Data Type Duplicate' : Unused code path elimination
//  Block '<S63>/Data Type Propagation' : Unused code path elimination
//  Block '<S64>/Data Type Duplicate' : Unused code path elimination
//  Block '<S64>/Data Type Propagation' : Unused code path elimination
//  Block '<S5>/Data Type Duplicate' : Unused code path elimination
//  Block '<S5>/Data Type Propagation' : Unused code path elimination
//  Block '<S6>/Data Type Duplicate' : Unused code path elimination
//  Block '<S6>/Data Type Propagation' : Unused code path elimination
//  Block '<S27>/Saturation' : Eliminated Saturate block
//  Block '<S22>/Saturation' : Eliminated Saturate block
//  Block '<S31>/Saturation' : Eliminated Saturate block
//  Block '<S14>/Reshape' : Reshape block reduction
//  Block '<S14>/Reshape1' : Reshape block reduction
//  Block '<S14>/Reshape2' : Reshape block reduction
//  Block '<S38>/Gain' : Eliminated nontunable gain of 1
//  Block '<S38>/Gain3' : Eliminated nontunable gain of 1
//  Block '<S52>/Saturation' : Eliminated Saturate block
//  Block '<S49>/Saturation' : Eliminated Saturate block
//  Block '<S50>/Saturation' : Eliminated Saturate block
//  Block '<S38>/Reshape' : Reshape block reduction
//  Block '<S55>/Gain' : Eliminated nontunable gain of 1
//  Block '<S55>/Gain1' : Eliminated nontunable gain of 1
//  Block '<S62>/Saturation' : Eliminated Saturate block
//  Block '<S68>/Saturation' : Eliminated Saturate block
//  Block '<S69>/Saturation' : Eliminated Saturate block
//  Block '<S70>/Saturation' : Eliminated Saturate block
//  Block '<S71>/Saturation' : Eliminated Saturate block
//  Block '<S72>/Saturation' : Eliminated Saturate block


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
//  '<Root>' : 'ArduCopter_MinnieLoiterFtc'
//  '<S1>'   : 'ArduCopter_MinnieLoiterFtc/Actuator muxer'
//  '<S2>'   : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode'
//  '<S3>'   : 'ArduCopter_MinnieLoiterFtc/Non-delayed position predictor'
//  '<S4>'   : 'ArduCopter_MinnieLoiterFtc/Quaternions to Rotation Matrix'
//  '<S5>'   : 'ArduCopter_MinnieLoiterFtc/Saturation Dynamic'
//  '<S6>'   : 'ArduCopter_MinnieLoiterFtc/Saturation Dynamic1'
//  '<S7>'   : 'ArduCopter_MinnieLoiterFtc/automated stick inputs from workspace'
//  '<S8>'   : 'ArduCopter_MinnieLoiterFtc/kill motors'
//  '<S9>'   : 'ArduCopter_MinnieLoiterFtc/log muxer'
//  '<S10>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Copter Inner Loop INDI and Control Allocation'
//  '<S11>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Delta attitude to stick commands'
//  '<S12>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Incremental specific thrust'
//  '<S13>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/MATLAB Function'
//  '<S14>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller'
//  '<S15>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/NDI position controller for copters with stick inputs'
//  '<S16>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/command direction'
//  '<S17>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/filtering'
//  '<S18>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/filtering1'
//  '<S19>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/initial yaw'
//  '<S20>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Copter Inner Loop INDI and Control Allocation/INDI high level wls control allocation'
//  '<S21>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Copter Inner Loop INDI and Control Allocation/Motor dynamics model'
//  '<S22>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Copter Inner Loop INDI and Control Allocation/Sensor filter model'
//  '<S23>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Copter Inner Loop INDI and Control Allocation/control effectiveness'
//  '<S24>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Copter Inner Loop INDI and Control Allocation/INDI high level wls control allocation/INDI control allocation'
//  '<S25>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Copter Inner Loop INDI and Control Allocation/INDI high level wls control allocation/MATLAB Function'
//  '<S26>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Copter Inner Loop INDI and Control Allocation/INDI high level wls control allocation/INDI control allocation/MATLAB Function2'
//  '<S27>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Copter Inner Loop INDI and Control Allocation/Motor dynamics model/PT1 discrete with saturations'
//  '<S28>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Copter Inner Loop INDI and Control Allocation/control effectiveness/MATLAB Function'
//  '<S29>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Delta attitude to stick commands/INDI Copter Acc 2 Lean Vector'
//  '<S30>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Delta attitude to stick commands/MATLAB Function4'
//  '<S31>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Incremental specific thrust/PT2 discrete with saturation Reduced attitude reference model'
//  '<S32>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Incremental specific thrust/desired and measured specific thrust'
//  '<S33>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/error computation'
//  '<S34>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/measure'
//  '<S35>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/ny control'
//  '<S36>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/ny from reference'
//  '<S37>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/ny measured'
//  '<S38>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/reference model'
//  '<S39>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/error computation/angle error'
//  '<S40>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/error computation/wrap angle'
//  '<S41>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/error computation/wrap angle1'
//  '<S42>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/measure/DCM to quaternions'
//  '<S43>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/measure/Quaternion Reduced'
//  '<S44>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/reference model/Lean Vector Derivative Trafo'
//  '<S45>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/reference model/Lean Vector Derivative Trafo Delay'
//  '<S46>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/reference model/Lean Vector Derivative to Omega'
//  '<S47>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/reference model/MATLAB Function'
//  '<S48>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/reference model/PT1 discrete reference model2'
//  '<S49>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/reference model/PT2 Lean Vector'
//  '<S50>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/reference model/PT2 Lean Vector1'
//  '<S51>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/reference model/lean angles 2 lean vector'
//  '<S52>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/reference model/PT1 discrete reference model2/PT1 discrete with saturations'
//  '<S53>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/reference model/PT2 Lean Vector/n ref norm'
//  '<S54>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/reference model/PT2 Lean Vector1/n ref norm'
//  '<S55>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/NDI position controller for copters with stick inputs/NDI position controller for copters reference model'
//  '<S56>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/NDI position controller for copters with stick inputs/NDI position controller for copters with reference input'
//  '<S57>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/NDI position controller for copters with stick inputs/NDI position controller for copters reference model/MATLAB Function1'
//  '<S58>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/NDI position controller for copters with stick inputs/NDI position controller for copters reference model/PT1 discrete reference model'
//  '<S59>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/NDI position controller for copters with stick inputs/NDI position controller for copters reference model/PT1 discrete reference model/PT1 discrete with saturations'
//  '<S60>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/NDI position controller for copters with stick inputs/NDI position controller for copters with reference input/measures'
//  '<S61>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/NDI position controller for copters with stick inputs/NDI position controller for copters with reference input/position controller'
//  '<S62>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/NDI position controller for copters with stick inputs/NDI position controller for copters with reference input/position controller/PT2 discrete with saturation'
//  '<S63>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/NDI position controller for copters with stick inputs/NDI position controller for copters with reference input/position controller/Saturation Dynamic'
//  '<S64>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/NDI position controller for copters with stick inputs/NDI position controller for copters with reference input/position controller/Saturation Dynamic1'
//  '<S65>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/NDI position controller for copters with stick inputs/NDI position controller for copters with reference input/position controller/acccntrlmax'
//  '<S66>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/NDI position controller for copters with stick inputs/NDI position controller for copters with reference input/position controller/eposmax'
//  '<S67>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/command direction/MATLAB Function'
//  '<S68>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/filtering/Sensor filter model'
//  '<S69>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/filtering/Sensor filter model1'
//  '<S70>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/filtering1/Sensor filter model'
//  '<S71>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/filtering1/Sensor filter model2'
//  '<S72>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/filtering1/Sensor filter model3'
//  '<S73>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/initial yaw/DCM to quaternions'
//  '<S74>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/initial yaw/Quaternion Reduced'
//  '<S75>'  : 'ArduCopter_MinnieLoiterFtc/automated stick inputs from workspace/interpHold'
//  '<S76>'  : 'ArduCopter_MinnieLoiterFtc/log muxer/Auxiliary function to define log_config in generated C++ code'

#endif                                 // RTW_HEADER_MatlabController_h_

//
// File trailer for generated code.
//
// [EOF]
//
