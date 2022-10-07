//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: MatlabController.h
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
#ifndef RTW_HEADER_MatlabController_h_
#define RTW_HEADER_MatlabController_h_
#include "rtwtypes.h"
#ifndef ArduCopter_TemplateController_COMMON_INCLUDES_
# define ArduCopter_TemplateController_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 // ArduCopter_TemplateController_COMMON_INCLUDES_ 

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
} measureBus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_logBus_
#define DEFINED_TYPEDEF_FOR_logBus_

typedef struct {
  real32_T signals[14];
  uint8_T num_signals;
  uint8_T signal_names[42];
  uint8_T batch_name[4];
} logBus;

#endif

// Constant parameters (default storage)
typedef struct {
  // Computed Parameter: Constant_Value
  //  Referenced by: '<S2>/Constant'

  real32_T Constant_Value[8];

  // Pooled Parameter (Expression: signal_names_1D)
  //  Referenced by:
  //    '<S4>/signal names'
  //    '<S5>/signal names'
  //    '<S6>/signal names'
  //    '<S7>/signal names'
  //    '<S8>/signal names'

  uint8_T pooled1[42];
} ConstP;

// External inputs (root inport signals with default storage)
typedef struct {
  cmdBus cmd;                          // '<Root>/cmd'
  measureBus measure;                  // '<Root>/measure'
} ExtU;

// External outputs (root outports fed by signals with default storage)
typedef struct {
  real32_T u[8];                       // '<Root>/u'
  logBus logs[5];                      // '<Root>/logs'
} ExtY;

// Constant parameters (default storage)
extern const ConstP rtConstP;

// Class declaration for model ArduCopter_TemplateController
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
};

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
//  '<Root>' : 'ArduCopter_TemplateController'
//  '<S1>'   : 'ArduCopter_TemplateController/Actuator muxer'
//  '<S2>'   : 'ArduCopter_TemplateController/dummy test controller'
//  '<S3>'   : 'ArduCopter_TemplateController/log muxer'
//  '<S4>'   : 'ArduCopter_TemplateController/log muxer/log muxer core1'
//  '<S5>'   : 'ArduCopter_TemplateController/log muxer/log muxer core2'
//  '<S6>'   : 'ArduCopter_TemplateController/log muxer/log muxer core3'
//  '<S7>'   : 'ArduCopter_TemplateController/log muxer/log muxer core4'
//  '<S8>'   : 'ArduCopter_TemplateController/log muxer/log muxer core5'

#endif                                 // RTW_HEADER_MatlabController_h_

//
// File trailer for generated code.
//
// [EOF]
//
