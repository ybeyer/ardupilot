//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: MatlabController_data.cpp
//
// Code generated for Simulink model 'ArduCopter_MinnieLoiterFtc'.
//
// Model version                  : 1.393
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Fri Apr  8 14:03:02 2022
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
  // Expression: ca
  //  Referenced by: '<S29>/MATLAB Function2'

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

  // Computed Parameter: Constant2_Value
  //  Referenced by: '<S7>/Constant2'

  { 0.0F, 0.365F, 0.0F, 0.0F, 0.0F, -0.365F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, -0.365F, 0.0F, 0.0F, 0.0F, 0.365F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F },

  // Computed Parameter: ny_du_red_trim_Value
  //  Referenced by: '<S28>/ny_du_red_trim'

  { -552.858215F, 599.994324F, -42.9285F, -16.0513F, -552.86F, -599.994507F,
    42.9418F, -16.0513F, 552.859802F, -598.937683F, -31.9157F, -16.0513F,
    552.858521F, 598.937927F, 31.9025F, -16.0513F },

  // Computed Parameter: ny_du_dt_Value
  //  Referenced by: '<S28>/ny_du_dt'

  { 0.069F, -0.0001F, -4.8857F, 0.0F, -0.069F, 0.0001F, 4.8857F, 0.0F, 0.069F,
    -0.0001F, -4.8857F, 0.0F, -0.069F, 0.0001F, 4.8857F, 0.0F },

  // Computed Parameter: Constant1_Value
  //  Referenced by: '<S7>/Constant1'

  { 0.0F, 2.0F, 4.5F, 7.0F, 9.5F, 12.0F, 14.5F, 17.0F, 19.5F, 22.0F },

  // Pooled Parameter (Mixed Expressions)
  //  Referenced by:
  //    '<Root>/1-D Lookup Table'
  //    '<S24>/1-D Lookup Table2'

  { 0.0F, 1.0F },

  // Pooled Parameter (Expression: [1000;2000])
  //  Referenced by:
  //    '<Root>/1-D Lookup Table'
  //    '<S8>/1-D Lookup Table'
  //    '<S8>/1-D Lookup Table1'

  { 1000.0F, 2000.0F },

  // Computed Parameter: uDLookupTable_tableData
  //  Referenced by: '<S61>/1-D Lookup Table'

  { 4.0F, 0.0F, -10.0F },

  // Computed Parameter: uDLookupTable_bp01Data
  //  Referenced by: '<S61>/1-D Lookup Table'

  { -1.0F, 0.0F, 1.0F },

  // Computed Parameter: uDLookupTable1_tableData
  //  Referenced by: '<S24>/1-D Lookup Table1'

  { -1000.0F, -1000.0F, 0.0F, 0.0F },

  // Computed Parameter: uDLookupTable1_bp01Data
  //  Referenced by: '<S24>/1-D Lookup Table1'

  { 0.0F, 0.1F, 0.2F, 1.0F },

  // Pooled Parameter (Expression: [1;0.1])
  //  Referenced by:
  //    '<S8>/1-D Lookup Table'
  //    '<S8>/1-D Lookup Table1'

  { 1.0F, 0.1F }
};

//
// File trailer for generated code.
//
// [EOF]
//
