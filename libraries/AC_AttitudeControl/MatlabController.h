//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: MatlabController.h
//
// Code generated for Simulink model 'ArduCopter_MinnieLoiterFtc'.
//
// Model version                  : 1.395
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Fri Apr  8 17:15:03 2022
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
#ifndef DEFINED_TYPEDEF_FOR_dtoSglFlt_cmdBus_
#define DEFINED_TYPEDEF_FOR_dtoSglFlt_cmdBus_

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
} dtoSglFlt_cmdBus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_dtoSgl_measureBus_
#define DEFINED_TYPEDEF_FOR_dtoSgl_measureBus_

typedef struct {
  real32_T omega_Kb[3];
  real32_T EulerAngles[3];
  real32_T q_bg[4];
  real32_T a_Kg[3];
  real32_T V_Kg[3];
  real32_T s_Kg[3];
  real32_T lla[3];
  real32_T rangefinder[6];
} dtoSgl_measureBus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_uuVM5iCJRh0lWMg9ww4boE_
#define DEFINED_TYPEDEF_FOR_struct_uuVM5iCJRh0lWMg9ww4boE_

typedef struct {
  real_T u_min[4];
  real_T u_max[4];
  real_T u_d[4];
  real_T W_v[16];
  real_T W_u[16];
  real_T gamma;
  real_T W[4];
  real_T i_max;
} struct_uuVM5iCJRh0lWMg9ww4boE;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_bp3dTcfUy8YiLQ2C4HxdqF_
#define DEFINED_TYPEDEF_FOR_struct_bp3dTcfUy8YiLQ2C4HxdqF_

typedef struct {
  real_T veltc;
  real_T velxymax;
  real_T accxymax;
  real_T velumax;
  real_T veldmax;
  real_T accumax;
  real_T accdmax;
} struct_bp3dTcfUy8YiLQ2C4HxdqF;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_t1muYkFBpCsvhMydPIEJxF_
#define DEFINED_TYPEDEF_FOR_struct_t1muYkFBpCsvhMydPIEJxF_

typedef struct {
  real_T pos;
  real_T vel;
  real_T acc;
} struct_t1muYkFBpCsvhMydPIEJxF;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_9CO4a2DCsO6nqJ3du2LIdG_
#define DEFINED_TYPEDEF_FOR_struct_9CO4a2DCsO6nqJ3du2LIdG_

typedef struct {
  struct_bp3dTcfUy8YiLQ2C4HxdqF rm;
  struct_t1muYkFBpCsvhMydPIEJxF k;
} struct_9CO4a2DCsO6nqJ3du2LIdG;

#endif

// Custom Type definition for MATLAB Function: '<S37>/DCM to quaternions'
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
  real32_T DiscreteTimeIntegrator_DSTATE[4];// '<S32>/Discrete-Time Integrator'
  real32_T DiscreteTimeIntegratory_DSTATE[9];// '<S68>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DSTA[9];// '<S68>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegrator_DSTATE_m[3];// '<S65>/Discrete-Time Integrator' 
  real32_T DiscreteTimeIntegrator_DSTATE_c[3];// '<S61>/Discrete-Time Integrator' 
  real32_T UD_DSTATE[3];               // '<S11>/UD'
  real32_T UD_DSTATE_b[3];             // '<S12>/UD'
  real32_T DiscreteTimeIntegratory_DSTAT_h[3];// '<S51>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DS_m[3];// '<S51>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTAT_m[3];// '<S58>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_DSTAT_e[3];// '<S59>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_DSTAT_f[3];// '<S60>/Discrete-Time Integrator y' 
  real32_T UnitDelay1_DSTATE[4];       // '<S24>/Unit Delay1'
  real32_T DiscreteTimeIntegratory_DSTAT_a[4];// '<S26>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DS_n[4];// '<S26>/Discrete-Time Integrator y_dt' 
  real32_T Delay_DSTATE[4];            // '<S13>/Delay'
  real32_T DiscreteTimeIntegratory_dt_DS_f[3];// '<S60>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_DS_l[3];// '<S59>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_DS_i[3];// '<S58>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegrator_DSTAT_mg;// '<S7>/Discrete-Time Integrator'
  real32_T DiscreteTimeIntegrator_DSTATE_i;// '<S57>/Discrete-Time Integrator'
  real32_T DiscreteTimeIntegrator2_DSTATE;// '<S41>/Discrete-Time Integrator2'
  real32_T DiscreteTimeIntegratory_DSTAT_i;// '<S19>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_D_lz;// '<S19>/Discrete-Time Integrator y_dt' 
  real32_T PrevY;                      // '<S8>/Rate Limiter'
  int8_T DiscreteTimeIntegrator_PrevRese;// '<S7>/Discrete-Time Integrator'
  uint8_T DiscreteTimeIntegrator_IC_LOADI;// '<S61>/Discrete-Time Integrator'
  uint8_T DiscreteTimeIntegratory_IC_LOAD;// '<S51>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_LO_p;// '<S58>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_LO_n;// '<S59>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_LO_f;// '<S60>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegrator2_IC_LOAD;// '<S41>/Discrete-Time Integrator2'
  uint8_T DiscreteTimeIntegratory_IC_LO_b;// '<S19>/Discrete-Time Integrator y'
} DW;

// Constant parameters (default storage)
typedef struct {
  // Expression: ca
  //  Referenced by: '<S29>/MATLAB Function2'

  struct_uuVM5iCJRh0lWMg9ww4boE MATLABFunction2_ca;

  // Computed Parameter: Constant2_Value
  //  Referenced by: '<S7>/Constant2'

  real32_T Constant2_Value[40];

  // Computed Parameter: ny_du_red_trim_Value
  //  Referenced by: '<S28>/ny_du_red_trim'

  real32_T ny_du_red_trim_Value[16];

  // Computed Parameter: ny_du_dt_Value
  //  Referenced by: '<S28>/ny_du_dt'

  real32_T ny_du_dt_Value[16];

  // Computed Parameter: Constant1_Value
  //  Referenced by: '<S7>/Constant1'

  real32_T Constant1_Value[10];

  // Pooled Parameter (Mixed Expressions)
  //  Referenced by:
  //    '<Root>/1-D Lookup Table'
  //    '<S24>/1-D Lookup Table2'

  real32_T pooled6[2];

  // Pooled Parameter (Expression: [1000;2000])
  //  Referenced by:
  //    '<Root>/1-D Lookup Table'
  //    '<S8>/1-D Lookup Table'
  //    '<S8>/1-D Lookup Table1'

  real32_T pooled7[2];

  // Computed Parameter: uDLookupTable_tableData
  //  Referenced by: '<S61>/1-D Lookup Table'

  real32_T uDLookupTable_tableData[3];

  // Computed Parameter: uDLookupTable_bp01Data
  //  Referenced by: '<S61>/1-D Lookup Table'

  real32_T uDLookupTable_bp01Data[3];

  // Computed Parameter: uDLookupTable1_tableData
  //  Referenced by: '<S24>/1-D Lookup Table1'

  real32_T uDLookupTable1_tableData[4];

  // Computed Parameter: uDLookupTable1_bp01Data
  //  Referenced by: '<S24>/1-D Lookup Table1'

  real32_T uDLookupTable1_bp01Data[4];

  // Pooled Parameter (Expression: [1;0.1])
  //  Referenced by:
  //    '<S8>/1-D Lookup Table'
  //    '<S8>/1-D Lookup Table1'

  real32_T pooled16[2];
} ConstP;

// External inputs (root inport signals with default storage)
typedef struct {
  dtoSglFlt_cmdBus cmd;                // '<Root>/cmd'
  dtoSgl_measureBus measure;           // '<Root>/measure'
} ExtU;

// External outputs (root outports fed by signals with default storage)
typedef struct {
  real32_T u[8];                       // '<Root>/u'
  real32_T logs[15];                   // '<Root>/logs'
} ExtY;

// Constant parameters (default storage)
extern const ConstP rtConstP;

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
  real_T wls_alloc(const real32_T B_4[16], const real32_T v[4], const real32_T
                   umin[4], const real32_T umax[4], const real32_T Wv[16], const
                   real_T Wu[16], const real32_T ud[4], real32_T gam, real32_T
                   u[4], real_T W[4], real_T imax);
};

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S2>/Constant' : Unused code path elimination
//  Block '<S11>/Data Type Duplicate' : Unused code path elimination
//  Block '<S12>/Data Type Duplicate' : Unused code path elimination
//  Block '<S62>/Gain' : Unused code path elimination
//  Block '<S62>/Gain1' : Unused code path elimination
//  Block '<S69>/Data Type Duplicate' : Unused code path elimination
//  Block '<S69>/Data Type Propagation' : Unused code path elimination
//  Block '<S70>/Data Type Duplicate' : Unused code path elimination
//  Block '<S70>/Data Type Propagation' : Unused code path elimination
//  Block '<S4>/Data Type Duplicate' : Unused code path elimination
//  Block '<S4>/Data Type Propagation' : Unused code path elimination
//  Block '<S5>/Data Type Duplicate' : Unused code path elimination
//  Block '<S5>/Data Type Propagation' : Unused code path elimination
//  Block '<S6>/Data Type Duplicate' : Unused code path elimination
//  Block '<S6>/Data Type Propagation' : Unused code path elimination
//  Block '<S32>/Saturation' : Eliminated Saturate block
//  Block '<S26>/Saturation' : Eliminated Saturate block
//  Block '<S17>/Reshape' : Reshape block reduction
//  Block '<S17>/Reshape1' : Reshape block reduction
//  Block '<S17>/Reshape2' : Reshape block reduction
//  Block '<S41>/Gain' : Eliminated nontunable gain of 1
//  Block '<S41>/Gain3' : Eliminated nontunable gain of 1
//  Block '<S57>/Saturation' : Eliminated Saturate block
//  Block '<S51>/Saturation' : Eliminated Saturate block
//  Block '<S41>/Reshape' : Reshape block reduction
//  Block '<S58>/Saturation' : Eliminated Saturate block
//  Block '<S59>/Saturation' : Eliminated Saturate block
//  Block '<S60>/Saturation' : Eliminated Saturate block
//  Block '<S61>/Gain' : Eliminated nontunable gain of 1
//  Block '<S61>/Gain1' : Eliminated nontunable gain of 1
//  Block '<S68>/Saturation' : Eliminated Saturate block
//  Block '<S19>/Saturation' : Eliminated Saturate block


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
//  '<S3>'   : 'ArduCopter_MinnieLoiterFtc/Quaternions to Rotation Matrix'
//  '<S4>'   : 'ArduCopter_MinnieLoiterFtc/Saturation Dynamic'
//  '<S5>'   : 'ArduCopter_MinnieLoiterFtc/Saturation Dynamic1'
//  '<S6>'   : 'ArduCopter_MinnieLoiterFtc/Saturation Dynamic2'
//  '<S7>'   : 'ArduCopter_MinnieLoiterFtc/automated stick inputs from workspace'
//  '<S8>'   : 'ArduCopter_MinnieLoiterFtc/kill motors'
//  '<S9>'   : 'ArduCopter_MinnieLoiterFtc/log muxer'
//  '<S10>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Delta attitude to stick commands'
//  '<S11>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Discrete Derivative'
//  '<S12>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Discrete Derivative3'
//  '<S13>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/INDI + CA'
//  '<S14>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/MATLAB Function'
//  '<S15>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/MATLAB Function1'
//  '<S16>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/MATLAB Function2'
//  '<S17>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller'
//  '<S18>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/NDI position controller for copters with stick inputs'
//  '<S19>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/PT2 discrete with saturation Reduced attitude reference model'
//  '<S20>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/command direction'
//  '<S21>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/initial yaw'
//  '<S22>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Delta attitude to stick commands/INDI Copter Acc 2 Lean Vector'
//  '<S23>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Delta attitude to stick commands/MATLAB Function4'
//  '<S24>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/INDI + CA/INDI high level wls control allocation'
//  '<S25>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/INDI + CA/PT1 discrete reference model'
//  '<S26>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/INDI + CA/PT2 discrete with saturation1'
//  '<S27>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/INDI + CA/acceleration g to b frame'
//  '<S28>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/INDI + CA/control effectiveness'
//  '<S29>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/INDI + CA/INDI high level wls control allocation/INDI control allocation'
//  '<S30>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/INDI + CA/INDI high level wls control allocation/MATLAB Function'
//  '<S31>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/INDI + CA/INDI high level wls control allocation/INDI control allocation/MATLAB Function2'
//  '<S32>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/INDI + CA/PT1 discrete reference model/PT1 discrete with saturations'
//  '<S33>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/INDI + CA/acceleration g to b frame/MATLAB Function'
//  '<S34>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/INDI + CA/acceleration g to b frame/dcm2Lean'
//  '<S35>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/INDI + CA/control effectiveness/MATLAB Function'
//  '<S36>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/error computation'
//  '<S37>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/measure'
//  '<S38>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/ny control'
//  '<S39>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/ny from reference'
//  '<S40>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/ny measured'
//  '<S41>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/reference model'
//  '<S42>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/error computation/angle error'
//  '<S43>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/error computation/wrap angle'
//  '<S44>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/error computation/wrap angle1'
//  '<S45>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/measure/DCM to quaternions'
//  '<S46>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/measure/Quaternion Reduced'
//  '<S47>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/reference model/Lean Vector Derivative Trafo'
//  '<S48>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/reference model/Lean Vector Derivative Trafo Delay'
//  '<S49>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/reference model/MATLAB Function'
//  '<S50>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/reference model/PT1 discrete reference model2'
//  '<S51>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/reference model/PT2 discrete with saturation'
//  '<S52>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/reference model/Subsystem'
//  '<S53>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/reference model/Subsystem1'
//  '<S54>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/reference model/Subsystem2'
//  '<S55>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/reference model/lean angles 2 lean vector'
//  '<S56>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/reference model/n ref norm'
//  '<S57>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/reference model/PT1 discrete reference model2/PT1 discrete with saturations'
//  '<S58>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/reference model/Subsystem/PT2 discrete with saturation'
//  '<S59>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/reference model/Subsystem1/PT2 discrete with saturation'
//  '<S60>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/Multicopter (Reduced) Attitude INDI Controller/reference model/Subsystem2/PT2 discrete with saturation'
//  '<S61>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/NDI position controller for copters with stick inputs/NDI position controller for copters reference model'
//  '<S62>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/NDI position controller for copters with stick inputs/NDI position controller for copters with reference input'
//  '<S63>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/NDI position controller for copters with stick inputs/NDI position controller for copters reference model/MATLAB Function1'
//  '<S64>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/NDI position controller for copters with stick inputs/NDI position controller for copters reference model/PT1 discrete reference model'
//  '<S65>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/NDI position controller for copters with stick inputs/NDI position controller for copters reference model/PT1 discrete reference model/PT1 discrete with saturations'
//  '<S66>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/NDI position controller for copters with stick inputs/NDI position controller for copters with reference input/measures'
//  '<S67>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/NDI position controller for copters with stick inputs/NDI position controller for copters with reference input/position controller'
//  '<S68>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/NDI position controller for copters with stick inputs/NDI position controller for copters with reference input/position controller/PT2 discrete with saturation'
//  '<S69>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/NDI position controller for copters with stick inputs/NDI position controller for copters with reference input/position controller/Saturation Dynamic'
//  '<S70>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/NDI position controller for copters with stick inputs/NDI position controller for copters with reference input/position controller/Saturation Dynamic1'
//  '<S71>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/NDI position controller for copters with stick inputs/NDI position controller for copters with reference input/position controller/acccntrlmax'
//  '<S72>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/NDI position controller for copters with stick inputs/NDI position controller for copters with reference input/position controller/eposmax'
//  '<S73>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/command direction/MATLAB Function'
//  '<S74>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/initial yaw/DCM to quaternions'
//  '<S75>'  : 'ArduCopter_MinnieLoiterFtc/Multicopter FTC Loiter Flight Mode/initial yaw/Quaternion Reduced'
//  '<S76>'  : 'ArduCopter_MinnieLoiterFtc/automated stick inputs from workspace/interpHold'

#endif                                 // RTW_HEADER_MatlabController_h_

//
// File trailer for generated code.
//
// [EOF]
//
