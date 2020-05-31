//
// Sponsored License - for use in support of a program or activity
// sponsored by MathWorks.  Not for government, commercial or other
// non-sponsored organizational use.
//
// File: ControllerSimple.cpp
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

static void wrapangle(real32_T rtu_angle, real32_T *rty_angle_0_2pi);

//
// Output and update for atomic system:
//    '<S2>/wrap angle'
//    '<S4>/wrap angle'
//
static void wrapangle(real32_T rtu_angle, real32_T *rty_angle_0_2pi)
{
  real32_T x;
  x = std::abs(rtu_angle);
  x -= std::floor(x / 6.28318548F) * 6.28318548F;
  if (rtu_angle >= 0.0F) {
    *rty_angle_0_2pi = x;
  } else {
    *rty_angle_0_2pi = 6.28318548F - x;
  }
}

// Model step function
void ControllerSimpleModelClass::step()
{
  real32_T rtb_x_ref;
  real32_T rtb_r_max;
  real32_T rtb_uT;
  real32_T rtb_uT_k;
  real32_T rtb_angle_0_2pi;
  real32_T rtb_error1[5];
  real32_T rtb_uT_1[4];
  real32_T rtb_uT_2[4];
  real32_T rtb_error1_0[9];
  int32_T i;
  real32_T tmp;
  int32_T i_0;
  real32_T rtb_TmpSignalConversionAtK_II_0;
  real32_T rtb_x_dt_ref_idx_0;
  real32_T rtb_x_dt_ref_idx_1;
  real32_T rtb_omega2_idx_0;

  // DiscreteIntegrator: '<S9>/x_dt'
  rtb_x_dt_ref_idx_0 = rtDW.x_dt_DSTATE[0];
  rtb_x_dt_ref_idx_1 = rtDW.x_dt_DSTATE[1];

  // DiscreteIntegrator: '<S7>/x'
  rtb_x_ref = rtDW.x_DSTATE;

  // MATLAB Function: '<S4>/wrap angle' incorporates:
  //   DiscreteIntegrator: '<S4>/Discrete-Time Integrator'

  wrapangle(rtDW.DiscreteTimeIntegrator_DSTATE, &rtb_r_max);

  // Sum: '<S2>/error1 ' incorporates:
  //   DiscreteIntegrator: '<S7>/x'
  //   DiscreteIntegrator: '<S9>/x'
  //   DiscreteIntegrator: '<S9>/x_dt'
  //   Inport: '<Root>/OutputMeasure'

  rtb_error1[2] = rtDW.x_DSTATE - rtU.OutputMeasure[2];
  rtb_error1[0] = rtDW.x_dt_DSTATE[0] - rtU.OutputMeasure[0];
  rtb_error1[3] = rtDW.x_DSTATE_b[0] - rtU.OutputMeasure[3];
  rtb_error1[1] = rtDW.x_dt_DSTATE[1] - rtU.OutputMeasure[1];
  rtb_error1[4] = rtDW.x_DSTATE_b[1] - rtU.OutputMeasure[4];

  // MATLAB Function: '<S2>/wrap angle' incorporates:
  //   Inport: '<Root>/OutputMeasure'

  wrapangle(rtU.OutputMeasure[5], &rtb_angle_0_2pi);

  // MATLAB Function: '<S2>/angle error'
  rtb_angle_0_2pi = rtb_r_max - rtb_angle_0_2pi;
  if (rtb_angle_0_2pi > 3.1415926535897931) {
    rtb_angle_0_2pi -= 6.28318548F;
  } else {
    if (rtb_angle_0_2pi < -3.1415926535897931) {
      rtb_angle_0_2pi += 6.28318548F;
    }
  }

  // End of MATLAB Function: '<S2>/angle error'

  // Sum: '<S2>/error3' incorporates:
  //   DiscreteIntegrator: '<S8>/x'
  //   Inport: '<Root>/OutputMeasure'

  rtb_TmpSignalConversionAtK_II_0 = rtDW.x_DSTATE_bz - rtU.OutputMeasure[8];

  // Gain: '<S8>/1//T' incorporates:
  //   DiscreteIntegrator: '<S8>/x'
  //   Gain: '<S4>/w_max'
  //   Inport: '<Root>/ch1-8'
  //   Sum: '<S8>/Sum2'

  rtb_uT = (4.0F * rtU.ch18[1] - rtDW.x_DSTATE_bz) * 2.0F;

  // Gain: '<S7>/1//T' incorporates:
  //   DiscreteIntegrator: '<S7>/x'
  //   Gain: '<S4>/r_max'
  //   Inport: '<Root>/ch1-8'
  //   Sum: '<S7>/Sum2'

  rtb_uT_k = (1.04719758F * rtU.ch18[0] - rtDW.x_DSTATE) * 2.5F;

  // Sum: '<S3>/Add1'
  rtb_uT_1[0] = rtb_uT;
  rtb_uT_1[3] = rtb_uT_k;

  // SignalConversion: '<S3>/TmpSignal ConversionAtCAInport1' incorporates:
  //   DiscreteIntegrator: '<S7>/x'

  rtb_error1_0[2] = rtDW.x_DSTATE;
  rtb_error1_0[5] = rtb_r_max;

  // Gain: '<S9>/omega^2' incorporates:
  //   DiscreteIntegrator: '<S9>/x'
  //   DiscreteIntegrator: '<S9>/x_dt'
  //   Gain: '<S4>/angle_max'
  //   Gain: '<S9>/2*d//omega'
  //   Inport: '<Root>/ch1-8'
  //   Sum: '<S9>/Sum2'
  //   Sum: '<S9>/Sum3'

  rtb_r_max = (0.785398185F * rtU.ch18[2] - (0.5F * rtDW.x_dt_DSTATE[0] +
    rtDW.x_DSTATE_b[0])) * 16.0F;

  // Sum: '<S3>/Add1'
  rtb_uT_1[1] = rtb_r_max;

  // SignalConversion: '<S3>/TmpSignal ConversionAtCAInport1' incorporates:
  //   DiscreteIntegrator: '<S9>/x'
  //   DiscreteIntegrator: '<S9>/x_dt'
  //   Inport: '<Root>/OutputMeasure'

  rtb_error1_0[0] = rtDW.x_dt_DSTATE[0];
  rtb_error1_0[3] = rtDW.x_DSTATE_b[0];
  rtb_error1_0[6] = rtU.OutputMeasure[6];

  // Gain: '<S9>/omega^2' incorporates:
  //   DiscreteIntegrator: '<S9>/x'
  //   DiscreteIntegrator: '<S9>/x_dt'
  //   Gain: '<S4>/angle_max'
  //   Gain: '<S9>/2*d//omega'
  //   Inport: '<Root>/ch1-8'
  //   Sum: '<S9>/Sum2'
  //   Sum: '<S9>/Sum3'

  rtb_omega2_idx_0 = rtb_r_max;
  rtb_r_max = (0.785398185F * rtU.ch18[3] - (0.5F * rtDW.x_dt_DSTATE[1] +
    rtDW.x_DSTATE_b[1])) * 16.0F;

  // Sum: '<S3>/Add1'
  rtb_uT_1[2] = rtb_r_max;

  // SignalConversion: '<S3>/TmpSignal ConversionAtCAInport1' incorporates:
  //   DiscreteIntegrator: '<S8>/x'
  //   DiscreteIntegrator: '<S9>/x'
  //   DiscreteIntegrator: '<S9>/x_dt'
  //   Inport: '<Root>/OutputMeasure'

  rtb_error1_0[1] = rtDW.x_dt_DSTATE[1];
  rtb_error1_0[4] = rtDW.x_DSTATE_b[1];
  rtb_error1_0[7] = rtU.OutputMeasure[7];
  rtb_error1_0[8] = rtDW.x_DSTATE_bz;

  // Sum: '<S3>/Add1' incorporates:
  //   Gain: '<S3>/CA'
  //   SignalConversion: '<S3>/TmpSignal ConversionAtCAInport1'

  for (i = 0; i < 4; i++) {
    tmp = 0.0F;
    for (i_0 = 0; i_0 < 9; i_0++) {
      tmp += rtConstP.CA_Gain[(i_0 << 2) + i] * rtb_error1_0[i_0];
    }

    rtb_uT_2[i] = rtb_uT_1[i] - tmp;
  }

  // SignalConversion: '<S1>/TmpSignal ConversionAtK_PInport1' incorporates:
  //   Inport: '<Root>/OutputMeasure'
  //   Sum: '<S2>/error3'

  for (i = 0; i < 5; i++) {
    rtb_error1_0[i] = rtb_error1[i];
  }

  rtb_error1_0[5] = rtb_angle_0_2pi;
  rtb_error1_0[6] = rtU.OutputMeasure[6] - rtU.OutputMeasure[6];
  rtb_error1_0[7] = rtU.OutputMeasure[7] - rtU.OutputMeasure[7];
  rtb_error1_0[8] = rtb_TmpSignalConversionAtK_II_0;

  // Update for DiscreteIntegrator: '<S7>/x'
  rtDW.x_DSTATE += 0.0025F * rtb_uT_k;

  // Update for DiscreteIntegrator: '<S9>/x_dt'
  rtDW.x_dt_DSTATE[0] += 0.0025F * rtb_omega2_idx_0;

  // Update for DiscreteIntegrator: '<S9>/x'
  rtDW.x_DSTATE_b[0] += 0.0025F * rtb_x_dt_ref_idx_0;

  // Update for DiscreteIntegrator: '<S9>/x_dt'
  rtDW.x_dt_DSTATE[1] += 0.0025F * rtb_r_max;

  // Update for DiscreteIntegrator: '<S9>/x'
  rtDW.x_DSTATE_b[1] += 0.0025F * rtb_x_dt_ref_idx_1;

  // Update for DiscreteIntegrator: '<S4>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_DSTATE += 0.0025F * rtb_x_ref;

  // Update for DiscreteIntegrator: '<S8>/x'
  rtDW.x_DSTATE_bz += 0.0025F * rtb_uT;
  for (i_0 = 0; i_0 < 4; i_0++) {
    // Gain: '<S3>/CB_inv'
    tmp = rtConstP.CB_inv_Gain[i_0 + 12] * rtb_uT_2[3] +
      (rtConstP.CB_inv_Gain[i_0 + 8] * rtb_uT_2[2] + (rtConstP.CB_inv_Gain[i_0 +
        4] * rtb_uT_2[1] + rtConstP.CB_inv_Gain[i_0] * rtb_uT_2[0]));

    // Gain: '<S1>/K_P' incorporates:
    //   SignalConversion: '<S1>/TmpSignal ConversionAtK_PInport1'

    rtb_uT_1[i_0] = 0.0F;
    for (i = 0; i < 9; i++) {
      rtb_uT_1[i_0] += rtConstP.K_P_Gain[(i << 2) + i_0] * rtb_error1_0[i];
    }

    // End of Gain: '<S1>/K_P'

    // Saturate: '<Root>/Saturation1' incorporates:
    //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator1'
    //   Sum: '<Root>/u_PI1'

    rtb_x_dt_ref_idx_0 = (tmp + rtb_uT_1[i_0]) +
      rtDW.DiscreteTimeIntegrator1_DSTATE[i_0];
    if (rtb_x_dt_ref_idx_0 > 1.0F) {
      // Outport: '<Root>/u'
      rtY.u[i_0] = 1.0F;
    } else if (rtb_x_dt_ref_idx_0 < 0.0F) {
      // Outport: '<Root>/u'
      rtY.u[i_0] = 0.0F;
    } else {
      // Outport: '<Root>/u'
      rtY.u[i_0] = rtb_x_dt_ref_idx_0;
    }

    // End of Saturate: '<Root>/Saturation1'

    // Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator1' incorporates:
    //   Gain: '<S1>/K_I'
    //   SignalConversion: '<S1>/TmpSignal ConversionAtK_IInport1'

    rtDW.DiscreteTimeIntegrator1_DSTATE[i_0] += 0.0025F * (rtConstP.K_I_Gain[i_0
      + 8] * rtb_TmpSignalConversionAtK_II_0 + (rtConstP.K_I_Gain[i_0 + 4] *
      rtb_error1[4] + rtConstP.K_I_Gain[i_0] * rtb_error1[3]));
    if (rtDW.DiscreteTimeIntegrator1_DSTATE[i_0] >= 0.9F) {
      rtDW.DiscreteTimeIntegrator1_DSTATE[i_0] = 0.9F;
    } else {
      if (rtDW.DiscreteTimeIntegrator1_DSTATE[i_0] <= 0.0F) {
        rtDW.DiscreteTimeIntegrator1_DSTATE[i_0] = 0.0F;
      }
    }

    // End of Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator1'
  }
}

// Model initialize function
void ControllerSimpleModelClass::initialize()
{
  // (no initialization code required)
}

// Constructor
ControllerSimpleModelClass::ControllerSimpleModelClass()
{
  // Currently there is no constructor body generated.
}

// Destructor
ControllerSimpleModelClass::~ControllerSimpleModelClass()
{
  // Currently there is no destructor body generated.
}

//
// File trailer for generated code.
//
// [EOF]
//
