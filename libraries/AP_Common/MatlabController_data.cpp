//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: MatlabController_data.cpp
//
// Code generated for Simulink model 'EasyGlider_ManualMode_with_waypoints_and_sysID'.
//
// Model version                  : 1.471
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Wed Aug 24 12:11:04 2022
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#include "MatlabController.h"

// Invariant block signals (default storage)
const ConstB rtConstB = {
  {
    4U,
    19U,
    70U,
    21U,
    0U,
    0U,
    0U,
    0U
  }
  // '<S1>/Data Type Conversion1'
};

// Constant parameters (default storage)
const ConstP rtConstP = {
  // Pooled Parameter (Mixed Expressions)
  //  Referenced by:
  //    '<S8>/Constant1'
  //    '<S8>/Unit Delay'

  {
    10.0F,
    0.0F,

    {
      {
        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },
        0.0F,
        0.0F,
        0.0F,
        5.0F
      }, {
        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },
        0.0F,
        0.0F,
        0.0F,
        5.0F
      }, {
        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },
        0.0F,
        0.0F,
        0.0F,
        5.0F
      }, {
        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },
        0.0F,
        0.0F,
        0.0F,
        5.0F
      }, {
        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },
        0.0F,
        0.0F,
        0.0F,
        5.0F
      }, {
        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },
        0.0F,
        0.0F,
        0.0F,
        5.0F
      }, {
        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },
        0.0F,
        0.0F,
        0.0F,
        5.0F
      }, {
        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },
        0.0F,
        0.0F,
        0.0F,
        5.0F
      }, {
        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },
        0.0F,
        0.0F,
        0.0F,
        5.0F
      }, {
        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

        { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },
        0.0F,
        0.0F,
        0.0F,
        5.0F
      } }
    ,
    0.0F,
    0.0F,
    0.0F,
    0.0F,
    0,
    5.0F
  },

  // Expression: signals
  //  Referenced by: '<S6>/Constant2'

  { 0.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F,
    1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F,
    1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, -1.0F, -1.0F,
    -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F,
    -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F,
    -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F,
    1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, -1.0F, -1.0F, -1.0F, -1.0F,
    -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F, -1.0F,
    -1.0F, -1.0F, -1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

  // Computed Parameter: Constant1_Value
  //  Referenced by: '<S4>/Constant1'

  { 3.86370325F, 2.47593317E-32F, -6.33923825E-17F, -1.93185163F, 1.93185163F,
    0.517638087F, -1.93185163F, -1.93185163F, 0.517638087F },

  // Computed Parameter: Logic_table
  //  Referenced by: '<S16>/Logic'

  { 0, 1, 0, 0, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0 }
};

//
// File trailer for generated code.
//
// [EOF]
//
