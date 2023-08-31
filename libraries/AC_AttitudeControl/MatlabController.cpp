//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: MatlabController.cpp
//
// Code generated for Simulink model 'ArduCopter_TemplateController'.
//
// Model version                  : 1.390
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Tue Oct 18 18:48:13 2022
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#include "MatlabController.h"

// Exported data definition

// Const memory section
// Definition for custom storage class: Const
const logConfigBus log_config[5] = { {
    14U,

    { 115U, 49U, 1U, 115U, 50U, 1U, 115U, 51U, 1U, 115U, 52U, 1U, 115U, 53U, 1U,
      115U, 54U, 1U, 115U, 55U, 1U, 115U, 56U, 1U, 115U, 57U, 1U, 115U, 49U, 48U,
      115U, 49U, 49U, 115U, 49U, 50U, 115U, 49U, 51U, 115U, 49U, 52U },

    { 77U, 76U, 49U, 0U }
  }, { 1U,
    { 115U, 49U, 1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U },

    { 77U, 76U, 50U, 0U }
  }, { 0U,
    { 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U },

    { 77U, 76U, 51U, 0U }
  }, { 0U,
    { 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U },

    { 77U, 76U, 52U, 0U }
  }, { 0U,
    { 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U },

    { 77U, 76U, 53U, 0U }
  } } ;

// Model step function
void MatlabControllerClass::step()
{
  // Outport: '<Root>/logs' incorporates:
  //   Inport: '<Root>/cmd'
  //   Inport: '<Root>/measure'
  //   MATLAB Function: '<S3>/Auxiliary function to define log_config in generated C++ code'
  //   SignalConversion: '<S4>/TmpSignal ConversionAt SFunction Inport2'

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
