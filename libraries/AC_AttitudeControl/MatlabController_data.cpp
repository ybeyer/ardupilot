//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: MatlabController_data.cpp
//
// Code generated for Simulink model 'ArduCopter_MinnieLoiterFtc'.
//
// Model version                  : 1.410
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Sat Nov 12 15:27:04 2022
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
  //  Referenced by: '<S24>/MATLAB Function2'

  {
    { 0.1F, 0.1F, 0.1F, 0.1F },

    { 1.0F, 1.0F, 1.0F, 1.0F },

    { 0.1F, 0.1F, 0.1F, 0.1F },

    { 10.0F, 0.0F, 0.0F, 0.0F, 0.0F, 10.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.01F, 0.0F,
      0.0F, 0.0F, 0.0F, 300.0F },

    { 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 1.0F },
    1000.0F,

    { 0.0F, 0.0F, 0.0F, 0.0F },
    100.0F
  },

  // Expression: G10
  //  Referenced by: '<S23>/MATLAB Function'

  { 0.000122825426F, 0.000113566784F, -8.07603737E-6F, -3.26728241E-6F,
    -0.000121244157F, 0.000111521353F, 7.43021747E-6F, -3.26728241E-6F,
    -0.000122785204F, -0.000113490474F, -7.0062556E-6F, -3.26728241E-6F,
    0.000121203935F, -0.000111597663F, 7.65207551E-6F, -3.26728241E-6F },

  // Expression: G20
  //  Referenced by: '<S23>/MATLAB Function'

  { 0.00158503931F, 0.00300671556F, -0.594297051F, 0.0F, -0.00158503931F,
    -0.00300671556F, 0.594297051F, 0.0F, 0.00158503931F, 0.00300671556F,
    -0.594297051F, 0.0F, -0.00158503931F, -0.00300671556F, 0.594297051F, 0.0F },

  // Expression: simin.signals.values
  //  Referenced by: '<S7>/Constant2'

  { 0.0F, 0.38028F, 0.0F, 0.0F, 0.0F, -0.38028F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, -0.38028F, 0.0F, 0.0F, 0.0F, 0.38028F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

  // Pooled Parameter (Mixed Expressions)
  //  Referenced by:
  //    '<Root>/1-D Lookup Table'
  //    '<S20>/1-D Lookup Table2'

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

  // Expression: [psc.rm.veldmax,0,-psc.rm.velumax]
  //  Referenced by: '<S55>/1-D Lookup Table'

  { 3.78896332F, 0.0F, -13.8162899F },

  // Computed Parameter: uDLookupTable_bp01Data
  //  Referenced by: '<S55>/1-D Lookup Table'

  { -1.0F, 0.0F, 1.0F },

  // Expression: [-0.99999999*ca.gamma*[1,1],0,0]
  //  Referenced by: '<S20>/1-D Lookup Table1'

  { -1000.0F, -1000.0F, 0.0F, 0.0F },

  // Computed Parameter: uDLookupTable1_bp01Data
  //  Referenced by: '<S20>/1-D Lookup Table1'

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
