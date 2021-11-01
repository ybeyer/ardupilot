//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: MatlabController.cpp
//
// Code generated for Simulink model 'ArduCopter_TemplateController'.
//
// Model version                  : 1.395
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Mon Nov  1 17:23:44 2021
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
  uint16_T i;
  real32_T y;
  real32_T scale;
  real32_T absxk;
  real32_T t;
  uint32_T qY;
  int32_T absxk_tmp;
  int32_T absxk_tmp_0;

  // Outputs for Enabled SubSystem: '<S2>/Subsystem' incorporates:
  //   EnablePort: '<S5>/Enable'

  // Inport: '<Root>/cmd'
  if (rtU.cmd.mission_change > 0) {
    // MATLAB Function: '<S5>/MATLAB Function'
    rtDW.distance = 0.0F;
    for (i = 2U; i <= rtU.cmd.num_waypoints; i++) {
      qY = i - /*MW:OvSatOk*/ 1U;
      if (qY > i) {
        qY = 0U;
      }

      scale = 1.29246971E-26F;
      absxk_tmp = (i - 1) << 2;
      absxk_tmp_0 = ((int32_T)qY - 1) << 2;
      absxk = std::abs(rtU.cmd.waypoints[absxk_tmp] -
                       rtU.cmd.waypoints[absxk_tmp_0]);
      if (absxk > 1.29246971E-26F) {
        y = 1.0F;
        scale = absxk;
      } else {
        t = absxk / 1.29246971E-26F;
        y = t * t;
      }

      absxk = std::abs(rtU.cmd.waypoints[absxk_tmp + 1] -
                       rtU.cmd.waypoints[absxk_tmp_0 + 1]);
      if (absxk > scale) {
        t = scale / absxk;
        y = y * t * t + 1.0F;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }

      absxk = std::abs(rtU.cmd.waypoints[absxk_tmp + 2] -
                       rtU.cmd.waypoints[absxk_tmp_0 + 2]);
      if (absxk > scale) {
        t = scale / absxk;
        y = y * t * t + 1.0F;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }

      y = scale * std::sqrt(y);
      rtDW.distance += y;
    }

    // End of MATLAB Function: '<S5>/MATLAB Function'
  }

  // End of Outputs for SubSystem: '<S2>/Subsystem'

  // Outport: '<Root>/logs' incorporates:
  //   DataTypeConversion: '<Root>/Cast To Single'
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
  rtY.logs[13] = rtDW.distance;
  rtY.logs[14] = rtU.cmd.num_waypoints;
}

// Model initialize function
void MatlabControllerClass::initialize()
{
  {
    int32_T i;

    // ConstCode for Outport: '<Root>/u' incorporates:
    //   Constant: '<S3>/Constant'

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
