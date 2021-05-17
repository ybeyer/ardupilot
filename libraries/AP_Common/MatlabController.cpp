//
// Sponsored License - for use in support of a program or activity
// sponsored by MathWorks.  Not for government, commercial or other
// non-sponsored organizational use.
//
// File: MatlabController.cpp
//
// Code generated for Simulink model 'MatlabController'.
//
// Model version                  : 1.398
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Fri Apr 23 12:15:54 2021
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
  // MATLAB Function: '<S2>/MATLAB Function' incorporates:
  //   Inport: '<Root>/cmd'
  //   Inport: '<Root>/measure'
  //   Outport: '<Root>/channels'

  rtY.channels[0] = rtU.cmd.roll;
  rtY.channels[1] = rtU.cmd.pitch;
  rtY.channels[2] = rtU.cmd.thr;
  rtY.channels[3] = rtU.cmd.yaw;
  rtY.channels[4] = rtU.measure.rangefinder[0];
  rtY.channels[5] = rtU.measure.rangefinder[1];
  rtY.channels[6] = rtU.measure.rangefinder[2];
  rtY.channels[7] = rtU.measure.rangefinder[3];

  // Outport: '<Root>/logs' incorporates:
  //   Inport: '<Root>/cmd'
  //   Inport: '<Root>/measure'

  rtY.logs[0] = rtU.cmd.roll;
  rtY.logs[1] = rtU.cmd.pitch;
  rtY.logs[2] = rtU.cmd.yaw;
  rtY.logs[3] = rtU.cmd.thr;
  rtY.logs[4] = rtU.measure.omega_Kb[0];
  rtY.logs[7] = rtU.measure.EulerAngles[0];
  rtY.logs[10] = rtU.measure.V_Kg[0];
  rtY.logs[5] = rtU.measure.omega_Kb[1];
  rtY.logs[8] = rtU.measure.EulerAngles[1];
  rtY.logs[11] = rtU.measure.V_Kg[1];
  rtY.logs[6] = rtU.measure.omega_Kb[2];
  rtY.logs[9] = rtU.measure.EulerAngles[2];
  rtY.logs[12] = rtU.measure.V_Kg[2];
  rtY.logs[13] = 0.0F;
  rtY.logs[14] = 0.0F;
}

// Model initialize function
void MatlabControllerClass::initialize()
{
  {
    int32_T i;

    // ConstCode for Outport: '<Root>/function_channels'
    for (i = 0; i < 8; i++) {
      rtY.function_channels[i] = rtConstB.DataTypeConversion1[i];
    }

    // End of ConstCode for Outport: '<Root>/function_channels'
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
