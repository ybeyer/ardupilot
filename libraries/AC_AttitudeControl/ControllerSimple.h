//
// Sponsored License - for use in support of a program or activity
// sponsored by MathWorks.  Not for government, commercial or other
// non-sponsored organizational use.
//
// File: ControllerSimple.h
//
// Code generated for Simulink model 'ControllerSimple'.
//
// Model version                  : 1.343
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Sat Jan 18 12:43:32 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#ifndef RTW_HEADER_ControllerSimple_h_
#define RTW_HEADER_ControllerSimple_h_
#include <cmath>
#ifndef ControllerSimple_COMMON_INCLUDES_
# define ControllerSimple_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 // ControllerSimple_COMMON_INCLUDES_

// Macros for accessing real-time model data structure

// Block signals and states (default storage) for system '<Root>'
typedef struct {
  real32_T x_dt_DSTATE[2];             // '<S9>/x_dt'
  real32_T x_DSTATE_b[2];              // '<S9>/x'
  real32_T DiscreteTimeIntegrator1_DSTATE[4];// '<S1>/Discrete-Time Integrator1' 
  real32_T x_DSTATE;                   // '<S7>/x'
  real32_T DiscreteTimeIntegrator_DSTATE;// '<S4>/Discrete-Time Integrator'
  real32_T x_DSTATE_bz;                // '<S8>/x'
} DW;

// Constant parameters (default storage)
typedef struct {
  // Expression: cntrl.K(:,1:9)
  //  Referenced by: '<S1>/K_P'

  real32_T K_P_Gain[36];

  // Expression: cntrl.A_ff
  //  Referenced by: '<S3>/CA'

  real32_T CA_Gain[36];

  // Expression: cntrl.B_inv
  //  Referenced by: '<S3>/CB_inv'

  real32_T CB_inv_Gain[16];

  // Expression: cntrl.K(:,10:end)
  //  Referenced by: '<S1>/K_I'

  real32_T K_I_Gain[12];
} ConstP;

// External inputs (root inport signals with default storage)
typedef struct {
  real32_T ch18[8];                    // '<Root>/ch1-8'
  real32_T OutputMeasure[9];           // '<Root>/OutputMeasure'
} ExtU;

// External outputs (root outports fed by signals with default storage)
typedef struct {
  real32_T u[4];                       // '<Root>/u'
} ExtY;

// Constant parameters (default storage)
extern const ConstP rtConstP;

// Class declaration for model ControllerSimple
class ControllerSimpleModelClass {
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
  ControllerSimpleModelClass();

  // Destructor
  ~ControllerSimpleModelClass();

  // private data and function members
 private:
  // Block signals and states
  DW rtDW;
};

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<Root>/Scope' : Unused code path elimination
//  Block '<Root>/Scope1' : Unused code path elimination
//  Block '<Root>/Scope15' : Unused code path elimination
//  Block '<Root>/Scope8' : Unused code path elimination
//  Block '<S2>/Scope' : Unused code path elimination
//  Block '<S2>/Scope1' : Unused code path elimination
//  Block '<S2>/Scope14' : Unused code path elimination
//  Block '<S2>/Scope16' : Unused code path elimination
//  Block '<S3>/Scope' : Unused code path elimination
//  Block '<S3>/Gain' : Eliminated nontunable gain of 1
//  Block '<S7>/Saturation' : Eliminated Saturate block
//  Block '<S8>/Saturation' : Eliminated Saturate block
//  Block '<S9>/Saturation' : Eliminated Saturate block


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
//  '<Root>' : 'ControllerSimple'
//  '<S1>'   : 'ControllerSimple/PI gains'
//  '<S2>'   : 'ControllerSimple/error'
//  '<S3>'   : 'ControllerSimple/feed forward'
//  '<S4>'   : 'ControllerSimple/refernce model'
//  '<S5>'   : 'ControllerSimple/error/angle error'
//  '<S6>'   : 'ControllerSimple/error/wrap angle'
//  '<S7>'   : 'ControllerSimple/refernce model/PT1 discrete reference model'
//  '<S8>'   : 'ControllerSimple/refernce model/PT1 discrete reference model1'
//  '<S9>'   : 'ControllerSimple/refernce model/PT2 discrete reference model'
//  '<S10>'  : 'ControllerSimple/refernce model/wrap angle'

#endif                                 // RTW_HEADER_ControllerSimple_h_

//
// File trailer for generated code.
//
// [EOF]
//
