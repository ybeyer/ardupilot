//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: MatlabController.cpp
//
// Code generated for Simulink model 'ArduCopter_TemplateController'.
//
// Model version                  : 1.388
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Sat Oct  8 13:50:48 2022
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#include "MatlabController.h"

// Model step function
void MatlabControllerClass::step()
{
  int32_T i;

  // SignalConversion: '<S3>/Signal Conversion2' incorporates:
  //   Constant: '<S4>/num signals'
  //   Inport: '<Root>/cmd'
  //   Inport: '<Root>/measure'
  //   Outport: '<Root>/logs'

  rtY.logs[0].signals[0] = rtU.cmd.roll;
  rtY.logs[0].signals[1] = rtU.cmd.pitch;
  rtY.logs[0].signals[2] = rtU.cmd.yaw;
  rtY.logs[0].signals[3] = rtU.cmd.thr;
  rtY.logs[0].signals[4] = rtU.measure.omega_Kb[0];
  rtY.logs[0].signals[7] = rtU.measure.EulerAngles[0];
  rtY.logs[0].signals[10] = rtU.measure.V_Kg[0];
  rtY.logs[0].signals[5] = rtU.measure.omega_Kb[1];
  rtY.logs[0].signals[8] = rtU.measure.EulerAngles[1];
  rtY.logs[0].signals[11] = rtU.measure.V_Kg[1];
  rtY.logs[0].signals[6] = rtU.measure.omega_Kb[2];
  rtY.logs[0].signals[9] = rtU.measure.EulerAngles[2];
  rtY.logs[0].signals[12] = rtU.measure.V_Kg[2];
  rtY.logs[0].signals[13] = 0.0F;
  rtY.logs[0].num_signals = 14U;
  rtY.logs[0].batch_name[0] = 77U;
  rtY.logs[0].batch_name[1] = 76U;
  rtY.logs[0].batch_name[2] = 49U;
  rtY.logs[0].batch_name[3] = 1U;

  // SignalConversion: '<S3>/Signal Conversion1' incorporates:
  //   Constant: '<S5>/num signals'
  //   Outport: '<Root>/logs'

  rtY.logs[1].signals[0] = 0.0F;
  rtY.logs[1].signals[1] = 0.0F;
  rtY.logs[1].signals[2] = 0.0F;
  rtY.logs[1].signals[3] = 0.0F;
  rtY.logs[1].signals[4] = 0.0F;
  rtY.logs[1].signals[5] = 0.0F;
  rtY.logs[1].signals[6] = 0.0F;
  rtY.logs[1].signals[7] = 0.0F;
  rtY.logs[1].signals[8] = 0.0F;
  rtY.logs[1].signals[9] = 0.0F;
  rtY.logs[1].signals[10] = 0.0F;
  rtY.logs[1].signals[11] = 0.0F;
  rtY.logs[1].signals[12] = 0.0F;
  rtY.logs[1].signals[13] = 0.0F;
  rtY.logs[1].num_signals = 1U;
  rtY.logs[1].batch_name[0] = 77U;
  rtY.logs[1].batch_name[1] = 76U;
  rtY.logs[1].batch_name[2] = 50U;
  rtY.logs[1].batch_name[3] = 1U;

  // SignalConversion: '<S3>/Signal Conversion' incorporates:
  //   Constant: '<S6>/num signals'
  //   Outport: '<Root>/logs'

  rtY.logs[2].signals[0] = 0.0F;
  rtY.logs[2].signals[1] = 0.0F;
  rtY.logs[2].signals[2] = 0.0F;
  rtY.logs[2].signals[3] = 0.0F;
  rtY.logs[2].signals[4] = 0.0F;
  rtY.logs[2].signals[5] = 0.0F;
  rtY.logs[2].signals[6] = 0.0F;
  rtY.logs[2].signals[7] = 0.0F;
  rtY.logs[2].signals[8] = 0.0F;
  rtY.logs[2].signals[9] = 0.0F;
  rtY.logs[2].signals[10] = 0.0F;
  rtY.logs[2].signals[11] = 0.0F;
  rtY.logs[2].signals[12] = 0.0F;
  rtY.logs[2].signals[13] = 0.0F;
  rtY.logs[2].num_signals = 0U;
  rtY.logs[2].batch_name[0] = 77U;
  rtY.logs[2].batch_name[1] = 76U;
  rtY.logs[2].batch_name[2] = 51U;
  rtY.logs[2].batch_name[3] = 1U;

  // SignalConversion: '<S3>/Signal Conversion3' incorporates:
  //   Constant: '<S7>/num signals'
  //   Outport: '<Root>/logs'

  rtY.logs[3].signals[0] = 0.0F;
  rtY.logs[3].signals[1] = 0.0F;
  rtY.logs[3].signals[2] = 0.0F;
  rtY.logs[3].signals[3] = 0.0F;
  rtY.logs[3].signals[4] = 0.0F;
  rtY.logs[3].signals[5] = 0.0F;
  rtY.logs[3].signals[6] = 0.0F;
  rtY.logs[3].signals[7] = 0.0F;
  rtY.logs[3].signals[8] = 0.0F;
  rtY.logs[3].signals[9] = 0.0F;
  rtY.logs[3].signals[10] = 0.0F;
  rtY.logs[3].signals[11] = 0.0F;
  rtY.logs[3].signals[12] = 0.0F;
  rtY.logs[3].signals[13] = 0.0F;
  rtY.logs[3].num_signals = 0U;
  rtY.logs[3].batch_name[0] = 77U;
  rtY.logs[3].batch_name[1] = 76U;
  rtY.logs[3].batch_name[2] = 52U;
  rtY.logs[3].batch_name[3] = 1U;

  // SignalConversion: '<S3>/Signal Conversion4' incorporates:
  //   Constant: '<S8>/num signals'
  //   Outport: '<Root>/logs'

  rtY.logs[4].signals[0] = 0.0F;
  rtY.logs[4].signals[1] = 0.0F;
  rtY.logs[4].signals[2] = 0.0F;
  rtY.logs[4].signals[3] = 0.0F;
  rtY.logs[4].signals[4] = 0.0F;
  rtY.logs[4].signals[5] = 0.0F;
  rtY.logs[4].signals[6] = 0.0F;
  rtY.logs[4].signals[7] = 0.0F;
  rtY.logs[4].signals[8] = 0.0F;
  rtY.logs[4].signals[9] = 0.0F;
  rtY.logs[4].signals[10] = 0.0F;
  rtY.logs[4].signals[11] = 0.0F;
  rtY.logs[4].signals[12] = 0.0F;
  rtY.logs[4].signals[13] = 0.0F;
  rtY.logs[4].num_signals = 0U;
  for (i = 0; i < 42; i++) {
    // SignalConversion: '<S3>/Signal Conversion2' incorporates:
    //   Constant: '<S4>/signal names'
    //   Outport: '<Root>/logs'

    rtY.logs[0].signal_names[i] = rtConstP.pooled1[i];

    // SignalConversion: '<S3>/Signal Conversion1' incorporates:
    //   Constant: '<S5>/signal names'
    //   Outport: '<Root>/logs'

    rtY.logs[1].signal_names[i] = rtConstP.pooled1[i];

    // SignalConversion: '<S3>/Signal Conversion' incorporates:
    //   Constant: '<S6>/signal names'
    //   Outport: '<Root>/logs'

    rtY.logs[2].signal_names[i] = rtConstP.pooled1[i];

    // SignalConversion: '<S3>/Signal Conversion3' incorporates:
    //   Constant: '<S7>/signal names'
    //   Outport: '<Root>/logs'

    rtY.logs[3].signal_names[i] = rtConstP.pooled1[i];

    // SignalConversion: '<S3>/Signal Conversion4' incorporates:
    //   Constant: '<S8>/signal names'
    //   Outport: '<Root>/logs'

    rtY.logs[4].signal_names[i] = rtConstP.pooled1[i];
  }

  // SignalConversion: '<S3>/Signal Conversion4' incorporates:
  //   Outport: '<Root>/logs'

  rtY.logs[4].batch_name[0] = 77U;
  rtY.logs[4].batch_name[1] = 76U;
  rtY.logs[4].batch_name[2] = 53U;
  rtY.logs[4].batch_name[3] = 1U;
}

// Model initialize function
void MatlabControllerClass::initialize()
{
  {
    int32_T i;

    // ConstCode for Outport: '<Root>/u' incorporates:
    //   Constant: '<S2>/Constant'

    for (i = 0; i < 8; i++) {
      rtY.u[i] = rtConstP.Constant_Value[i];
    }

    // End of ConstCode for Outport: '<Root>/u'
  }
}

// Constructor
MatlabControllerClass::MatlabControllerClass()
{
  // Currently there is no constructor body generated.
}

// Destructor
MatlabControllerClass::~MatlabControllerClass()
{
  // Currently there is no destructor body generated.
}

//
// File trailer for generated code.
//
// [EOF]
//
