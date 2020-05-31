//
// Sponsored License - for use in support of a program or activity
// sponsored by MathWorks.  Not for government, commercial or other
// non-sponsored organizational use.
//
// File: ControllerSimple_data.cpp
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
#include "ControllerSimple.h"

// Constant parameters (default storage)
const ConstP rtConstP = {
  // Expression: cntrl.K(:,1:9)
  //  Referenced by: '<S1>/K_P'

  { 0.0685F, -0.0685F, -0.0626F, 0.0626F, 0.0699F, 0.0699F, -0.0699F, -0.0699F,
    -0.2858F, 0.2858F, -0.319F, 0.319F, 0.2895F, -0.2895F, -0.2616F, 0.2616F,
    0.2772F, 0.2772F, -0.2772F, -0.2772F, -0.4737F, 0.4737F, -0.5249F, 0.5249F,
    -0.0099F, -0.0099F, 0.0099F, 0.0099F, 0.0104F, -0.0104F, -0.0094F, 0.0094F,
    -0.0795F, -0.0795F, -0.0795F, -0.0795F },

  // Expression: cntrl.A_ff
  //  Referenced by: '<S3>/CA'

  { -0.0F, -0.8367F, -0.0F, -0.0002F, 0.0F, 0.0F, -0.5554F, -0.0F, 0.0F, -0.0F,
    -0.0F, -0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, -0.497F, -0.0F,
    0.0F, 0.0F },

  // Expression: cntrl.B_inv
  //  Referenced by: '<S3>/CB_inv'

  { -0.0316F, -0.0316F, -0.0316F, -0.0316F, 0.0041F, -0.0041F, -0.0041F, 0.0041F,
    0.005F, 0.005F, -0.005F, -0.005F, -0.0769F, 0.0769F, -0.0863F, 0.0863F },

  // Expression: cntrl.K(:,10:end)
  //  Referenced by: '<S1>/K_I'

  { 0.0011F, -0.0011F, -0.001F, 0.001F, 0.001F, 0.001F, -0.001F, -0.001F, -0.1F,
    -0.1F, -0.1F, -0.1F }
};

//
// File trailer for generated code.
//
// [EOF]
//
