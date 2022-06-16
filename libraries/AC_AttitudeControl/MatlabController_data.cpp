//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: MatlabController_data.cpp
//
// Code generated for Simulink model 'ArduCopter_MinnieDragonflyController'.
//
// Model version                  : 1.447
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Thu May 19 18:03:08 2022
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM 7
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#include "MatlabController.h"

// Constant parameters (default storage)
const ConstP rtConstP = {
  // Computed Parameter: Constant2_Value
  //  Referenced by: '<S23>/Constant2'

  {
    4.0F,
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
      } }
    ,
    0.0F,
    0.0F,
    0.0F,
    0.0F,
    0,
    5.0F
  },

  // Expression: ca
  //  Referenced by: '<S41>/MATLAB Function2'

  {
    { 0.1, 0.1, 0.1, 0.1 },

    { 1.0, 1.0, 1.0, 1.0 },

    { 0.1, 0.1, 0.1, 0.1 },

    { 10.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
      0.0, 1.0 },

    { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
      1.0 },
    1000.0,

    { 0.0, 0.0, 0.0, 0.0 },
    100.0
  },

  // Computed Parameter: ny_du_red_trim_Value
  //  Referenced by: '<S40>/ny_du_red_trim'

  { -552.858215F, 599.994324F, -42.9285F, -16.0513F, -552.86F, -599.994507F,
    42.9418F, -16.0513F, 552.859802F, -598.937683F, -31.9157F, -16.0513F,
    552.858521F, 598.937927F, 31.9025F, -16.0513F },

  // Computed Parameter: ny_du_dt_Value
  //  Referenced by: '<S40>/ny_du_dt'

  { 0.069F, -0.0001F, -4.8857F, 0.0F, -0.069F, 0.0001F, 4.8857F, 0.0F, 0.069F,
    -0.0001F, -4.8857F, 0.0F, -0.069F, 0.0001F, 4.8857F, 0.0F },

  // Computed Parameter: uDLookupTable1_tableData
  //  Referenced by: '<S37>/1-D Lookup Table1'

  { -1000.0F, -1000.0F, 0.0F, 0.0F },

  // Computed Parameter: uDLookupTable1_bp01Data
  //  Referenced by: '<S37>/1-D Lookup Table1'

  { 0.0F, 0.1F, 0.2F, 1.0F },

  // Computed Parameter: uDLookupTable_tableData
  //  Referenced by: '<S13>/1-D Lookup Table'

  { 4.0F, 0.0F, -10.0F },

  // Computed Parameter: uDLookupTable_bp01Data
  //  Referenced by: '<S13>/1-D Lookup Table'

  { -1.0F, 0.0F, 1.0F }
};

//
// File trailer for generated code.
//
// [EOF]
//
