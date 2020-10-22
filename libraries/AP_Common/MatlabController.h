//
// Sponsored License - for use in support of a program or activity
// sponsored by MathWorks.  Not for government, commercial or other
// non-sponsored organizational use.
//
// File: MatlabController.h
//
// Code generated for Simulink model 'MatlabController'.
//
// Model version                  : 1.385
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Thu Oct 22 16:10:19 2020
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
#ifndef MatlabController_COMMON_INCLUDES_
# define MatlabController_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 // MatlabController_COMMON_INCLUDES_

// Macros for accessing real-time model data structure
#ifndef DEFINED_TYPEDEF_FOR_cmdBus_
#define DEFINED_TYPEDEF_FOR_cmdBus_

typedef struct {
  real32_T roll;
  real32_T pitch;
  real32_T yaw;
  real32_T thr;
} cmdBus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_measureBus_
#define DEFINED_TYPEDEF_FOR_measureBus_

typedef struct {
  real32_T omega_Kb[3];
  real32_T EulerAngles[3];
  real32_T q_bg[4];
  real32_T a_Kg[3];
  real32_T V_Kg[3];
  real32_T s_Kg[3];
  real32_T lla[3];
} measureBus;

#endif

// Custom Type definition for MATLAB Function: '<S7>/Euler angles second derivate' 
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
  real32_T x_DSTATE[2];                // '<S24>/x'
  real32_T x_dt_DSTATE[2];             // '<S24>/x_dt'
  real32_T x_DSTATE_o[3];              // '<S14>/x'
  real32_T UnitDelay_DSTATE[4];        // '<S2>/Unit Delay'
  real32_T x_DSTATE_cg[4];             // '<S6>/x'
  real32_T A[4];                       // '<S5>/Discrete-Time Integrator'
  real32_T x_dt_DSTATE_m[4];           // '<S6>/x_dt'
  real32_T x_dt_DSTATE_h[3];           // '<S14>/x_dt'
  real32_T UD_DSTATE[3];               // '<S11>/UD'
  real32_T x_DSTATE_c;                 // '<S23>/x'
  real32_T DiscreteTimeIntegrator_DSTATE;// '<S10>/Discrete-Time Integrator'
} DW;

// Constant parameters (default storage)
typedef struct {
  // Computed Parameter: Constant6_Value
  //  Referenced by: '<S8>/Constant6'

  real32_T Constant6_Value[16];

  // Computed Parameter: Constant2_Value
  //  Referenced by: '<S2>/Constant2'

  real32_T Constant2_Value[12];

  // Computed Parameter: Constant1_Value
  //  Referenced by: '<S2>/Constant1'

  real32_T Constant1_Value[12];

  // Computed Parameter: Constant5_Value
  //  Referenced by: '<S8>/Constant5'

  real32_T Constant5_Value[9];

  // Expression: cntrl.K
  //  Referenced by: '<S4>/Gain'

  real32_T Gain_Gain[18];
} ConstP;

// External inputs (root inport signals with default storage)
typedef struct {
  cmdBus cmd;                          // '<Root>/cmd'
  measureBus measure;                  // '<Root>/measure'
} ExtU;

// External outputs (root outports fed by signals with default storage)
typedef struct {
  real32_T u[8];                       // '<Root>/u'
} ExtY;

// Constant parameters (default storage)
extern const ConstP rtConstP;

// Class declaration for model MatlabController
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
  void LSQFromQR(const real32_T A_data[], const int32_T A_size[2], const
                 real32_T tau_data[], const int32_T jpvt_data[], real32_T B_3[7],
                 int32_T rankA, real32_T Y_data[], int32_T *Y_size);
  real32_T xnrm2(int32_T n, const real32_T x_data[], int32_T ix0);
  void xzlarf(int32_T m, int32_T n, int32_T iv0, real32_T tau, real32_T C_data[],
              int32_T ic0, real32_T work_data[]);
  void qrsolve(const real32_T A_data[], const int32_T A_size[2], const real32_T
               B_1[7], real32_T Y_data[], int32_T *Y_size);
  void mldivide(const real32_T A_data[], const int32_T A_size[2], const real32_T
                B_0[7], real32_T Y_data[], int32_T *Y_size);
  boolean_T any(const boolean_T x_data[], const int32_T *x_size);
};

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S4>/Scope' : Unused code path elimination
//  Block '<S2>/Scope' : Unused code path elimination
//  Block '<S2>/Scope1' : Unused code path elimination
//  Block '<S2>/Scope2' : Unused code path elimination
//  Block '<S2>/Scope3' : Unused code path elimination
//  Block '<S2>/Scope4' : Unused code path elimination
//  Block '<S2>/Scope5' : Unused code path elimination
//  Block '<S2>/Scope6' : Unused code path elimination
//  Block '<S11>/Data Type Duplicate' : Unused code path elimination
//  Block '<S7>/Scope' : Unused code path elimination
//  Block '<S7>/Scope1' : Unused code path elimination
//  Block '<S7>/Scope2' : Unused code path elimination
//  Block '<S8>/Scope' : Unused code path elimination
//  Block '<S8>/Scope1' : Unused code path elimination
//  Block '<S8>/Scope2' : Unused code path elimination
//  Block '<S9>/Scope' : Unused code path elimination
//  Block '<S9>/Scope1' : Unused code path elimination
//  Block '<S9>/Scope14' : Unused code path elimination
//  Block '<S9>/Scope16' : Unused code path elimination
//  Block '<S2>/Data Type Conversion3' : Eliminate redundant data type conversion
//  Block '<S2>/Gain2' : Eliminated nontunable gain of 1
//  Block '<S5>/Saturation' : Eliminated Saturate block
//  Block '<S6>/Saturation' : Eliminated Saturate block
//  Block '<S14>/Saturation' : Eliminated Saturate block
//  Block '<S8>/Reshape' : Reshape block reduction
//  Block '<S8>/Reshape1' : Reshape block reduction
//  Block '<S8>/Reshape2' : Reshape block reduction
//  Block '<S8>/Reshape4' : Reshape block reduction
//  Block '<S8>/Reshape5' : Reshape block reduction
//  Block '<S23>/Saturation' : Eliminated Saturate block
//  Block '<S24>/Saturation' : Eliminated Saturate block


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
//  '<Root>' : 'MatlabController'
//  '<S1>'   : 'MatlabController/Actuator muxer'
//  '<S2>'   : 'MatlabController/Atti INDI simple1'
//  '<S3>'   : 'MatlabController/output norm'
//  '<S4>'   : 'MatlabController/Atti INDI simple1/LQR for error regulation'
//  '<S5>'   : 'MatlabController/Atti INDI simple1/PT1 reference model'
//  '<S6>'   : 'MatlabController/Atti INDI simple1/PT2 discrete reference model1'
//  '<S7>'   : 'MatlabController/Atti INDI simple1/additional outputs computation'
//  '<S8>'   : 'MatlabController/Atti INDI simple1/control allocation'
//  '<S9>'   : 'MatlabController/Atti INDI simple1/error'
//  '<S10>'  : 'MatlabController/Atti INDI simple1/reference trajectory'
//  '<S11>'  : 'MatlabController/Atti INDI simple1/additional outputs computation/Discrete Derivative'
//  '<S12>'  : 'MatlabController/Atti INDI simple1/additional outputs computation/Euler angles derivative'
//  '<S13>'  : 'MatlabController/Atti INDI simple1/additional outputs computation/Euler angles second derivate'
//  '<S14>'  : 'MatlabController/Atti INDI simple1/additional outputs computation/PT2 discrete reference model'
//  '<S15>'  : 'MatlabController/Atti INDI simple1/additional outputs computation/Quaternions to Rotation Matrix'
//  '<S16>'  : 'MatlabController/Atti INDI simple1/additional outputs computation/get attitude in FC frame'
//  '<S17>'  : 'MatlabController/Atti INDI simple1/additional outputs computation/get attitude in FC frame/Euler Angles to Rotation Matrix'
//  '<S18>'  : 'MatlabController/Atti INDI simple1/additional outputs computation/get attitude in FC frame/Rotations matrix to Euler angles'
//  '<S19>'  : 'MatlabController/Atti INDI simple1/control allocation/MATLAB Function'
//  '<S20>'  : 'MatlabController/Atti INDI simple1/control allocation/MATLAB Function1'
//  '<S21>'  : 'MatlabController/Atti INDI simple1/error/angle error'
//  '<S22>'  : 'MatlabController/Atti INDI simple1/error/wrap angle'
//  '<S23>'  : 'MatlabController/Atti INDI simple1/reference trajectory/PT1 discrete reference model'
//  '<S24>'  : 'MatlabController/Atti INDI simple1/reference trajectory/PT2 discrete reference model'
//  '<S25>'  : 'MatlabController/Atti INDI simple1/reference trajectory/wrap angle'
//  '<S26>'  : 'MatlabController/output norm/MATLAB Function'

#endif                                 // RTW_HEADER_MatlabController_h_

//
// File trailer for generated code.
//
// [EOF]
//
