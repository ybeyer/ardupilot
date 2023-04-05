//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: MatlabController_data.cpp
//
// Code generated for Simulink model 'ArduCopter_MinnieLindiCopterFtc'.
//
// Model version                  : 1.445
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Tue Apr  4 18:44:03 2023
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
  // Pooled Parameter (Mixed Expressions)
  //  Referenced by:
  //    '<S56>/Constant1'
  //    '<S56>/TrajMemory'

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
  //  Referenced by: '<S12>/Control Allocation Vertical Acc Weighting'

  {
    { 0.1F, 0.1F, 0.1F, 0.1F },

    { 1.0F, 1.0F, 1.0F, 1.0F },

    { 0.1F, 0.1F, 0.1F, 0.1F },

    { 30.0F, 0.0F, 0.0F, 0.0F, 0.0F, 30.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.01F, 0.0F,
      0.0F, 0.0F, 0.0F, 300.0F },

    { 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 1.0F },
    1000.0F,

    { 0.0F, 0.0F, 0.0F, 0.0F },
    100.0F
  },

  // Expression: ca
  //  Referenced by: '<S50>/MATLAB Function2'

  {
    { 0.1F, 0.1F, 0.1F, 0.1F },

    { 1.0F, 1.0F, 1.0F, 1.0F },

    { 0.1F, 0.1F, 0.1F, 0.1F },

    { 30.0F, 0.0F, 0.0F, 0.0F, 0.0F, 30.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.01F, 0.0F,
      0.0F, 0.0F, 0.0F, 300.0F },

    { 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 1.0F },
    1000.0F,

    { 0.0F, 0.0F, 0.0F, 0.0F },
    100.0F
  },

  // Expression: G10
  //  Referenced by: '<S49>/MATLAB Function'

  { 0.000159011164F, 0.000174105327F, -1.02316853E-5F, -5.38287895E-6F,
    -0.000159011164F, 0.000174105327F, 1.02316853E-5F, -5.38287895E-6F,
    -0.000158978015F, -0.000174105327F, -9.88228931E-6F, -5.38287895E-6F,
    0.000158978015F, -0.000174105327F, 9.88228931E-6F, -5.38287895E-6F },

  // Expression: G20
  //  Referenced by: '<S49>/MATLAB Function'

  { 0.00130626222F, -0.0F, -0.792563558F, 0.0F, -0.00130626222F, 0.0F,
    0.792563558F, 0.0F, 0.00130626222F, -0.0F, -0.792563558F, 0.0F,
    -0.00130626222F, 0.0F, 0.792563558F, 0.0F },

  // Expression: [psc.rm.veldmax,0,-psc.rm.velumax]
  //  Referenced by: '<S17>/1-D Lookup Table'

  { 3.82967377F, 0.0F, -15.0265598F },

  // Computed Parameter: uDLookupTable_bp01Data
  //  Referenced by: '<S17>/1-D Lookup Table'

  { -1.0F, 0.0F, 1.0F },

  // Computed Parameter: uDLookupTable_tableData_g
  //  Referenced by: '<Root>/1-D Lookup Table'

  { 0.0F, 1.0F },

  // Pooled Parameter (Expression: [1000;2000])
  //  Referenced by:
  //    '<Root>/1-D Lookup Table'
  //    '<S8>/1-D Lookup Table'
  //    '<S8>/1-D Lookup Table1'

  { 1000.0F, 2000.0F },

  // Expression: simin.time
  //  Referenced by: '<S7>/Constant1'

  { 0.0F, 5.0F, 7.5F, 10.0F, 12.5F, 15.0F, 17.5F, 20.0F, 22.5F, 25.0F },

  // Expression: simin.signals.values
  //  Referenced by: '<S7>/Constant2'

  { 0.0F, 0.292F, 0.0F, 0.0F, 0.0F, -0.292F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, -0.292F, 0.0F, 0.0F, 0.0F, 0.292F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F },

  // Pooled Parameter (Expression: [1;0])
  //  Referenced by:
  //    '<S8>/1-D Lookup Table'
  //    '<S8>/1-D Lookup Table1'

  { 1.0F, 0.0F }
};

//
// File trailer for generated code.
//
// [EOF]
//
