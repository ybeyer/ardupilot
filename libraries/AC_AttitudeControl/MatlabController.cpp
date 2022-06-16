//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: MatlabController.cpp
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

extern real32_T rt_hypotf(real32_T u0, real32_T u1);
static real32_T look1_iflf_binlx(real32_T u0, const real32_T bp0[], const
  real32_T table[], uint32_T maxIndex);
static void wrapangle(real32_T rtu_angle, real32_T *rty_angle_0_2pi);
static void DCMtoquaternions(const real32_T rtu_M_bg[9], real32_T rty_q_bg[4]);
static void QuaternionReduced(const real32_T rtu_q_bg[4], real32_T rty_q_red[4],
  real32_T *rty_yaw);
static void LeanVectorDerivativeTrafo(const real32_T rtu_n_g[3], const real32_T
  rtu_n_g_dt[3], const real32_T rtu_n_g_dt2[3], const real32_T rtu_M_bg[9],
  const real32_T rtu_omega_Kb[3], const real32_T rtu_omega_Kb_dt[3], real32_T
  rty_n_b[3], real32_T rty_n_b_dt[3], real32_T rty_n_b_dt2[3]);

// Forward declaration for local functions
static void quatNormalize(const real32_T q[4], real32_T q_out[4]);
static real32_T acosReal(real32_T u);
static real32_T look1_iflf_binlx(real32_T u0, const real32_T bp0[], const
  real32_T table[], uint32_T maxIndex)
{
  real32_T frac;
  uint32_T iRght;
  uint32_T iLeft;
  uint32_T bpIdx;

  // Column-major Lookup 1-D
  // Search method: 'binary'
  // Use previous index: 'off'
  // Interpolation method: 'Linear point-slope'
  // Extrapolation method: 'Linear'
  // Use last breakpoint for index at or above upper limit: 'off'
  // Remove protection against out-of-range input in generated code: 'off'

  // Prelookup - Index and Fraction
  // Index Search method: 'binary'
  // Extrapolation method: 'Linear'
  // Use previous index: 'off'
  // Use last breakpoint for index at or above upper limit: 'off'
  // Remove protection against out-of-range input in generated code: 'off'

  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = (u0 - bp0[0U]) / (bp0[1U] - bp0[0U]);
  } else if (u0 < bp0[maxIndex]) {
    // Binary Search
    bpIdx = maxIndex >> 1U;
    iLeft = 0U;
    iRght = maxIndex;
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex - 1U;
    frac = (u0 - bp0[maxIndex - 1U]) / (bp0[maxIndex] - bp0[maxIndex - 1U]);
  }

  // Column-major Interpolation 1-D
  // Interpolation method: 'Linear point-slope'
  // Use last breakpoint for index at or above upper limit: 'off'
  // Overflow mode: 'wrapping'

  return (table[iLeft + 1U] - table[iLeft]) * frac + table[iLeft];
}

//
// Output and update for atomic system:
//    '<S46>/wrap angle'
//    '<S46>/wrap angle1'
//
static void wrapangle(real32_T rtu_angle, real32_T *rty_angle_0_2pi)
{
  real32_T x;
  x = std::abs(rtu_angle);
  x -= std::floor(x / 6.28318548F) * 6.28318548F;
  if (rtu_angle >= 0.0F) {
    *rty_angle_0_2pi = x;
  } else {
    *rty_angle_0_2pi = 6.28318548F - x;
  }
}

//
// Output and update for atomic system:
//    '<S47>/DCM to quaternions'
//    '<S17>/DCM to quaternions'
//
static void DCMtoquaternions(const real32_T rtu_M_bg[9], real32_T rty_q_bg[4])
{
  real32_T q_0;
  real32_T q_1;
  real32_T q_2;
  real32_T q_3;
  real32_T ex;
  int32_T idx;
  q_0 = ((1.0F + rtu_M_bg[0]) + rtu_M_bg[4]) + rtu_M_bg[8];
  if (0.0F >= q_0) {
    q_0 = 0.0F;
  }

  q_0 = 0.5F * std::sqrt(q_0);
  q_1 = ((1.0F + rtu_M_bg[0]) - rtu_M_bg[4]) - rtu_M_bg[8];
  if (0.0F >= q_1) {
    q_1 = 0.0F;
  }

  q_1 = 0.5F * std::sqrt(q_1);
  q_2 = ((1.0F - rtu_M_bg[0]) + rtu_M_bg[4]) - rtu_M_bg[8];
  if (0.0F >= q_2) {
    q_2 = 0.0F;
  }

  q_2 = 0.5F * std::sqrt(q_2);
  q_3 = ((1.0F - rtu_M_bg[0]) - rtu_M_bg[4]) + rtu_M_bg[8];
  if (0.0F >= q_3) {
    q_3 = 0.0F;
  }

  q_3 = 0.5F * std::sqrt(q_3);
  ex = q_0;
  idx = -1;
  if (q_0 < q_1) {
    ex = q_1;
    idx = 0;
  }

  if (ex < q_2) {
    ex = q_2;
    idx = 1;
  }

  if (ex < q_3) {
    idx = 2;
  }

  switch (idx + 1) {
   case 0:
    ex = rtu_M_bg[7] - rtu_M_bg[5];
    if (ex < 0.0F) {
      ex = -1.0F;
    } else {
      if (ex > 0.0F) {
        ex = 1.0F;
      }
    }

    q_1 *= ex;
    ex = rtu_M_bg[2] - rtu_M_bg[6];
    if (ex < 0.0F) {
      ex = -1.0F;
    } else {
      if (ex > 0.0F) {
        ex = 1.0F;
      }
    }

    q_2 *= ex;
    ex = rtu_M_bg[3] - rtu_M_bg[1];
    if (ex < 0.0F) {
      ex = -1.0F;
    } else {
      if (ex > 0.0F) {
        ex = 1.0F;
      }
    }

    q_3 *= ex;
    break;

   case 1:
    ex = rtu_M_bg[7] - rtu_M_bg[5];
    if (ex < 0.0F) {
      ex = -1.0F;
    } else {
      if (ex > 0.0F) {
        ex = 1.0F;
      }
    }

    q_0 *= ex;
    ex = rtu_M_bg[3] + rtu_M_bg[1];
    if (ex < 0.0F) {
      ex = -1.0F;
    } else {
      if (ex > 0.0F) {
        ex = 1.0F;
      }
    }

    q_2 *= ex;
    ex = rtu_M_bg[2] + rtu_M_bg[6];
    if (ex < 0.0F) {
      ex = -1.0F;
    } else {
      if (ex > 0.0F) {
        ex = 1.0F;
      }
    }

    q_3 *= ex;
    break;

   case 2:
    ex = rtu_M_bg[2] - rtu_M_bg[6];
    if (ex < 0.0F) {
      ex = -1.0F;
    } else {
      if (ex > 0.0F) {
        ex = 1.0F;
      }
    }

    q_0 *= ex;
    ex = rtu_M_bg[3] + rtu_M_bg[1];
    if (ex < 0.0F) {
      ex = -1.0F;
    } else {
      if (ex > 0.0F) {
        ex = 1.0F;
      }
    }

    q_1 *= ex;
    ex = rtu_M_bg[7] + rtu_M_bg[5];
    if (ex < 0.0F) {
      ex = -1.0F;
    } else {
      if (ex > 0.0F) {
        ex = 1.0F;
      }
    }

    q_3 *= ex;
    break;

   case 3:
    ex = rtu_M_bg[3] - rtu_M_bg[1];
    if (ex < 0.0F) {
      ex = -1.0F;
    } else {
      if (ex > 0.0F) {
        ex = 1.0F;
      }
    }

    q_0 *= ex;
    ex = rtu_M_bg[2] + rtu_M_bg[6];
    if (ex < 0.0F) {
      ex = -1.0F;
    } else {
      if (ex > 0.0F) {
        ex = 1.0F;
      }
    }

    q_1 *= ex;
    ex = rtu_M_bg[7] + rtu_M_bg[5];
    if (ex < 0.0F) {
      ex = -1.0F;
    } else {
      if (ex > 0.0F) {
        ex = 1.0F;
      }
    }

    q_2 *= ex;
    break;
  }

  rty_q_bg[0] = q_0;
  rty_q_bg[1] = q_1;
  rty_q_bg[2] = q_2;
  rty_q_bg[3] = q_3;
  q_1 = 1.29246971E-26F;
  q_2 = std::abs(rty_q_bg[0]);
  if (q_2 > 1.29246971E-26F) {
    q_0 = 1.0F;
    q_1 = q_2;
  } else {
    q_3 = q_2 / 1.29246971E-26F;
    q_0 = q_3 * q_3;
  }

  q_2 = std::abs(rty_q_bg[1]);
  if (q_2 > q_1) {
    q_3 = q_1 / q_2;
    q_0 = q_0 * q_3 * q_3 + 1.0F;
    q_1 = q_2;
  } else {
    q_3 = q_2 / q_1;
    q_0 += q_3 * q_3;
  }

  q_2 = std::abs(rty_q_bg[2]);
  if (q_2 > q_1) {
    q_3 = q_1 / q_2;
    q_0 = q_0 * q_3 * q_3 + 1.0F;
    q_1 = q_2;
  } else {
    q_3 = q_2 / q_1;
    q_0 += q_3 * q_3;
  }

  q_2 = std::abs(rty_q_bg[3]);
  if (q_2 > q_1) {
    q_3 = q_1 / q_2;
    q_0 = q_0 * q_3 * q_3 + 1.0F;
    q_1 = q_2;
  } else {
    q_3 = q_2 / q_1;
    q_0 += q_3 * q_3;
  }

  q_0 = q_1 * std::sqrt(q_0);
  if (2.22044605E-16F >= q_0) {
    q_0 = 2.22044605E-16F;
  }

  rty_q_bg[0] /= q_0;
  rty_q_bg[1] /= q_0;
  rty_q_bg[2] /= q_0;
  rty_q_bg[3] /= q_0;
}

// Function for MATLAB Function: '<S47>/Quaternion Reduced'
static void quatNormalize(const real32_T q[4], real32_T q_out[4])
{
  real32_T y;
  real32_T scale;
  real32_T absxk;
  real32_T t;
  scale = 1.29246971E-26F;
  absxk = std::abs(q[0]);
  if (absxk > 1.29246971E-26F) {
    y = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    y = t * t;
  }

  absxk = std::abs(q[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = std::abs(q[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = std::abs(q[3]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  y = scale * std::sqrt(y);
  if (2.22044605E-16F >= y) {
    y = 2.22044605E-16F;
  }

  q_out[0] = q[0] / y;
  q_out[1] = q[1] / y;
  q_out[2] = q[2] / y;
  q_out[3] = q[3] / y;
}

// Function for MATLAB Function: '<S47>/Quaternion Reduced'
static real32_T acosReal(real32_T u)
{
  real32_T b_idx_0;
  b_idx_0 = u;
  if (u > 1.0F) {
    b_idx_0 = 1.0F;
  }

  if (b_idx_0 < -1.0F) {
    b_idx_0 = -1.0F;
  }

  return std::acos(b_idx_0);
}

//
// Output and update for atomic system:
//    '<S47>/Quaternion Reduced'
//    '<S17>/Quaternion Reduced'
//
static void QuaternionReduced(const real32_T rtu_q_bg[4], real32_T rty_q_red[4],
  real32_T *rty_yaw)
{
  real32_T q_yaw[4];
  real32_T q1_q1;
  real32_T q_yaw_0[4];
  real32_T M_bg_idx_2;
  real32_T M_bg_idx_8;
  int32_T tmp;
  quatNormalize(rtu_q_bg, q_yaw);
  M_bg_idx_2 = (q_yaw[1] * q_yaw[3] + q_yaw[0] * q_yaw[2]) * 2.0F;
  M_bg_idx_8 = ((q_yaw[0] * q_yaw[0] - q_yaw[1] * q_yaw[1]) - q_yaw[2] * q_yaw[2])
    + q_yaw[3] * q_yaw[3];
  if (1.0F <= M_bg_idx_8) {
    M_bg_idx_8 = 1.0F;
  }

  if (-1.0F >= M_bg_idx_8) {
    M_bg_idx_8 = -1.0F;
  }

  M_bg_idx_8 = std::acos(M_bg_idx_8);
  q1_q1 = std::sin(M_bg_idx_8);
  q1_q1 = q1_q1 * q1_q1 - M_bg_idx_2 * M_bg_idx_2;
  if ((q_yaw[2] * q_yaw[3] - q_yaw[0] * q_yaw[1]) * 2.0F >= 0.0F) {
    tmp = -1;
  } else {
    tmp = 1;
  }

  if (0.0F >= q1_q1) {
    q1_q1 = 0.0F;
  }

  q1_q1 = std::atan2((real32_T)tmp * std::sqrt(q1_q1), -M_bg_idx_2);
  M_bg_idx_2 = std::sin(M_bg_idx_8 / 2.0F);
  rty_q_red[0] = std::cos(M_bg_idx_8 / 2.0F);
  rty_q_red[1] = std::sin(q1_q1) * M_bg_idx_2;
  rty_q_red[2] = -std::cos(q1_q1) * M_bg_idx_2;
  rty_q_red[3] = 0.0F;
  M_bg_idx_8 = ((rty_q_red[0] * rty_q_red[0] + rty_q_red[1] * rty_q_red[1]) +
                rty_q_red[2] * rty_q_red[2]) + rty_q_red[3] * rty_q_red[3];
  if (2.22044605E-16F >= M_bg_idx_8) {
    M_bg_idx_8 = 2.22044605E-16F;
  }

  q_yaw[0] = rty_q_red[0] / M_bg_idx_8;
  q_yaw[1] = -rty_q_red[1] / M_bg_idx_8;
  q_yaw[2] = -rty_q_red[2] / M_bg_idx_8;
  q_yaw[3] = -rty_q_red[3] / M_bg_idx_8;
  q_yaw_0[0] = ((q_yaw[0] * rtu_q_bg[0] - q_yaw[1] * rtu_q_bg[1]) - q_yaw[2] *
                rtu_q_bg[2]) - q_yaw[3] * rtu_q_bg[3];
  q_yaw_0[1] = (q_yaw[0] * rtu_q_bg[1] + rtu_q_bg[0] * q_yaw[1]) + (q_yaw[2] *
    rtu_q_bg[3] - q_yaw[3] * rtu_q_bg[2]);
  q_yaw_0[2] = (q_yaw[0] * rtu_q_bg[2] + rtu_q_bg[0] * q_yaw[2]) + (q_yaw[3] *
    rtu_q_bg[1] - q_yaw[1] * rtu_q_bg[3]);
  q_yaw_0[3] = (q_yaw[0] * rtu_q_bg[3] + rtu_q_bg[0] * q_yaw[3]) + (q_yaw[1] *
    rtu_q_bg[2] - q_yaw[2] * rtu_q_bg[1]);
  quatNormalize(q_yaw_0, q_yaw);
  if (q_yaw[3] < 0.0F) {
    M_bg_idx_8 = -1.0F;
  } else if (q_yaw[3] > 0.0F) {
    M_bg_idx_8 = 1.0F;
  } else {
    M_bg_idx_8 = q_yaw[3];
  }

  if (M_bg_idx_8 >= 0.0F) {
    *rty_yaw = 2.0F * acosReal(q_yaw[0]);
  } else {
    *rty_yaw = 2.0F * acosReal(-q_yaw[0]);
  }
}

//
// Output and update for atomic system:
//    '<S51>/Lean Vector Derivative Trafo'
//    '<S51>/Lean Vector Derivative Trafo Delay'
//
static void LeanVectorDerivativeTrafo(const real32_T rtu_n_g[3], const real32_T
  rtu_n_g_dt[3], const real32_T rtu_n_g_dt2[3], const real32_T rtu_M_bg[9],
  const real32_T rtu_omega_Kb[3], const real32_T rtu_omega_Kb_dt[3], real32_T
  rty_n_b[3], real32_T rty_n_b_dt[3], real32_T rty_n_b_dt2[3])
{
  real32_T rtu_omega_Kb_0[3];
  real32_T rtu_omega_Kb_dt_0[3];
  real32_T tmp[9];
  real32_T rtu_M_bg_0[9];
  int32_T i;
  int32_T i_0;
  real32_T tmp_0;
  int32_T rtu_M_bg_tmp;
  int32_T rtu_M_bg_tmp_0;
  for (i = 0; i < 3; i++) {
    rty_n_b[i] = 0.0F;
    rty_n_b[i] += rtu_M_bg[i] * rtu_n_g[0];
    rty_n_b[i] += rtu_M_bg[i + 3] * rtu_n_g[1];
    rty_n_b[i] += rtu_M_bg[i + 6] * rtu_n_g[2];
  }

  rtu_omega_Kb_0[0] = -(rtu_omega_Kb[1] * rty_n_b[2] - rtu_omega_Kb[2] *
                        rty_n_b[1]);
  rtu_omega_Kb_0[1] = -(rtu_omega_Kb[2] * rty_n_b[0] - rtu_omega_Kb[0] *
                        rty_n_b[2]);
  rtu_omega_Kb_0[2] = -(rtu_omega_Kb[0] * rty_n_b[1] - rtu_omega_Kb[1] *
                        rty_n_b[0]);
  for (i = 0; i < 3; i++) {
    rty_n_b_dt[i] = rtu_omega_Kb_0[i] + (rtu_M_bg[i + 6] * rtu_n_g_dt[2] +
      (rtu_M_bg[i + 3] * rtu_n_g_dt[1] + rtu_M_bg[i] * rtu_n_g_dt[0]));
  }

  rtu_omega_Kb_dt_0[0] = -(rtu_omega_Kb_dt[1] * rty_n_b[2] - rtu_omega_Kb_dt[2] *
    rty_n_b[1]);
  rtu_omega_Kb_dt_0[1] = -(rtu_omega_Kb_dt[2] * rty_n_b[0] - rtu_omega_Kb_dt[0] *
    rty_n_b[2]);
  rtu_omega_Kb_dt_0[2] = -(rtu_omega_Kb_dt[0] * rty_n_b[1] - rtu_omega_Kb_dt[1] *
    rty_n_b[0]);
  rtu_omega_Kb_0[0] = rtu_omega_Kb[1] * rty_n_b_dt[2] - rtu_omega_Kb[2] *
    rty_n_b_dt[1];
  rtu_omega_Kb_0[1] = rtu_omega_Kb[2] * rty_n_b_dt[0] - rtu_omega_Kb[0] *
    rty_n_b_dt[2];
  rtu_omega_Kb_0[2] = rtu_omega_Kb[0] * rty_n_b_dt[1] - rtu_omega_Kb[1] *
    rty_n_b_dt[0];
  tmp[0] = 0.0F;
  tmp[3] = -rtu_omega_Kb[2];
  tmp[6] = rtu_omega_Kb[1];
  tmp[1] = rtu_omega_Kb[2];
  tmp[4] = 0.0F;
  tmp[7] = -rtu_omega_Kb[0];
  tmp[2] = -rtu_omega_Kb[1];
  tmp[5] = rtu_omega_Kb[0];
  tmp[8] = 0.0F;
  for (i = 0; i < 3; i++) {
    tmp_0 = 0.0F;
    for (i_0 = 0; i_0 < 3; i_0++) {
      rtu_M_bg_tmp = i + 3 * i_0;
      rtu_M_bg_0[rtu_M_bg_tmp] = 0.0F;
      rtu_M_bg_tmp_0 = 3 * i_0 + i;
      rtu_M_bg_0[rtu_M_bg_tmp] = rtu_M_bg_0[rtu_M_bg_tmp_0] + rtu_M_bg[3 * i_0] *
        tmp[3 * i];
      rtu_M_bg_0[rtu_M_bg_tmp] = rtu_M_bg[3 * i_0 + 1] * tmp[3 * i + 1] +
        rtu_M_bg_0[rtu_M_bg_tmp_0];
      rtu_M_bg_0[rtu_M_bg_tmp] = rtu_M_bg[3 * i_0 + 2] * tmp[3 * i + 2] +
        rtu_M_bg_0[rtu_M_bg_tmp_0];
      tmp_0 += rtu_M_bg_0[rtu_M_bg_tmp_0] * rtu_n_g_dt[i_0];
    }

    rty_n_b_dt2[i] = ((rtu_omega_Kb_dt_0[i] - rtu_omega_Kb_0[i]) + tmp_0) +
      (rtu_M_bg[i + 6] * rtu_n_g_dt2[2] + (rtu_M_bg[i + 3] * rtu_n_g_dt2[1] +
        rtu_M_bg[i] * rtu_n_g_dt2[0]));
  }
}

void MatlabControllerClass::emxInit_real32_T(emxArray_real32_T **pEmxArray,
  int32_T numDimensions)
{
  emxArray_real32_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_real32_T *)malloc(sizeof(emxArray_real32_T));
  emxArray = *pEmxArray;
  emxArray->data = (real32_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

void MatlabControllerClass::emxInit_real_T(emxArray_real_T **pEmxArray, int32_T
  numDimensions)
{
  emxArray_real_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_real_T *)malloc(sizeof(emxArray_real_T));
  emxArray = *pEmxArray;
  emxArray->data = (real_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

void MatlabControllerClass::emxEnsureCapacity_real_T(emxArray_real_T *emxArray,
  int32_T oldNumel)
{
  int32_T newNumel;
  int32_T i;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = calloc((uint32_T)i, sizeof(real_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(real_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (real_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

void MatlabControllerClass::emxFree_real_T(emxArray_real_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_real_T *)NULL) {
    if (((*pEmxArray)->data != (real_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_real_T *)NULL;
  }
}

void MatlabControllerClass::emxEnsureCapacity_real32_T(emxArray_real32_T
  *emxArray, int32_T oldNumel)
{
  int32_T newNumel;
  int32_T i;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = calloc((uint32_T)i, sizeof(real32_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(real32_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (real32_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

void MatlabControllerClass::emxInit_int32_T(emxArray_int32_T **pEmxArray,
  int32_T numDimensions)
{
  emxArray_int32_T *emxArray;
  int32_T i;
  *pEmxArray = (emxArray_int32_T *)malloc(sizeof(emxArray_int32_T));
  emxArray = *pEmxArray;
  emxArray->data = (int32_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int32_T *)malloc(sizeof(int32_T) * numDimensions);
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = true;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

void MatlabControllerClass::emxEnsureCapacity_int32_T(emxArray_int32_T *emxArray,
  int32_T oldNumel)
{
  int32_T newNumel;
  int32_T i;
  void *newData;
  if (oldNumel < 0) {
    oldNumel = 0;
  }

  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      if (i > 1073741823) {
        i = MAX_int32_T;
      } else {
        i <<= 1;
      }
    }

    newData = calloc((uint32_T)i, sizeof(int32_T));
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, sizeof(int32_T) * oldNumel);
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = (int32_T *)newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = true;
  }
}

void MatlabControllerClass::emxFree_real32_T(emxArray_real32_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_real32_T *)NULL) {
    if (((*pEmxArray)->data != (real32_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_real32_T *)NULL;
  }
}

// Function for MATLAB Function: '<S29>/trajFromWaypoints'
real32_T MatlabControllerClass::xnrm2(int32_T n, const emxArray_real32_T *x,
  int32_T ix0)
{
  real32_T y;
  real32_T scale;
  int32_T kend;
  real32_T absxk;
  real32_T t;
  int32_T k;
  y = 0.0F;
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x->data[ix0 - 1]);
    } else {
      scale = 1.29246971E-26F;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        absxk = std::abs(x->data[k - 1]);
        if (absxk > scale) {
          t = scale / absxk;
          y = y * t * t + 1.0F;
          scale = absxk;
        } else {
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * std::sqrt(y);
    }
  }

  return y;
}

real32_T rt_hypotf(real32_T u0, real32_T u1)
{
  real32_T y;
  real32_T a;
  real32_T b;
  a = std::abs(u0);
  b = std::abs(u1);
  if (a < b) {
    a /= b;
    y = std::sqrt(a * a + 1.0F) * b;
  } else if (a > b) {
    b /= a;
    y = std::sqrt(b * b + 1.0F) * a;
  } else {
    y = a * 1.41421354F;
  }

  return y;
}

// Function for MATLAB Function: '<S29>/trajFromWaypoints'
void MatlabControllerClass::xscal(int32_T n, real32_T a, emxArray_real32_T *x,
  int32_T ix0)
{
  int32_T b;
  int32_T k;
  b = (ix0 + n) - 1;
  for (k = ix0; k <= b; k++) {
    x->data[k - 1] *= a;
  }
}

// Function for MATLAB Function: '<S29>/trajFromWaypoints'
void MatlabControllerClass::xgeqp3(emxArray_real32_T *A, emxArray_real32_T *tau,
  emxArray_int32_T *jpvt)
{
  int32_T m;
  int32_T n;
  int32_T mn;
  emxArray_real32_T *work;
  emxArray_real32_T *vn1;
  emxArray_real32_T *vn2;
  int32_T i_i;
  int32_T nmi;
  int32_T b_n;
  int32_T yk;
  int32_T ix;
  real32_T smax;
  real32_T s;
  int32_T b_ix;
  int32_T iy;
  int32_T knt;
  int32_T c_ix;
  int32_T e;
  int32_T b_ia;
  int32_T d_ix;
  int32_T exitg1;
  boolean_T exitg2;
  m = A->size[0];
  n = A->size[1];
  b_n = A->size[0];
  mn = A->size[1];
  if (b_n < mn) {
    mn = b_n;
  }

  yk = tau->size[0];
  tau->size[0] = mn;
  emxEnsureCapacity_real32_T(tau, yk);
  if (A->size[1] < 1) {
    b_n = 0;
  } else {
    b_n = A->size[1];
  }

  yk = jpvt->size[0] * jpvt->size[1];
  jpvt->size[0] = 1;
  jpvt->size[1] = b_n;
  emxEnsureCapacity_int32_T(jpvt, yk);
  if (b_n > 0) {
    jpvt->data[0] = 1;
    yk = 1;
    for (i_i = 2; i_i <= b_n; i_i++) {
      yk++;
      jpvt->data[i_i - 1] = yk;
    }
  }

  if ((A->size[0] != 0) && (A->size[1] != 0)) {
    emxInit_real32_T(&work, 1);
    b_n = A->size[1];
    yk = work->size[0];
    work->size[0] = b_n;
    emxEnsureCapacity_real32_T(work, yk);
    for (yk = 0; yk < b_n; yk++) {
      work->data[yk] = 0.0F;
    }

    emxInit_real32_T(&vn1, 1);
    emxInit_real32_T(&vn2, 1);
    b_n = A->size[1];
    yk = vn1->size[0];
    vn1->size[0] = b_n;
    emxEnsureCapacity_real32_T(vn1, yk);
    b_n = A->size[1];
    yk = vn2->size[0];
    vn2->size[0] = b_n;
    emxEnsureCapacity_real32_T(vn2, yk);
    b_n = 1;
    for (yk = 0; yk < n; yk++) {
      vn1->data[yk] = xnrm2(m, A, b_n);
      vn2->data[yk] = vn1->data[yk];
      b_n += m;
    }

    for (b_n = 0; b_n < mn; b_n++) {
      i_i = b_n * m + b_n;
      nmi = n - b_n;
      yk = (m - b_n) - 1;
      if (nmi < 1) {
        knt = 0;
      } else {
        knt = 1;
        if (nmi > 1) {
          ix = b_n;
          smax = std::abs(vn1->data[b_n]);
          for (b_ix = 2; b_ix <= nmi; b_ix++) {
            ix++;
            s = std::abs(vn1->data[ix]);
            if (s > smax) {
              knt = b_ix;
              smax = s;
            }
          }
        }
      }

      ix = (b_n + knt) - 1;
      if (ix + 1 != b_n + 1) {
        b_ix = m * ix;
        iy = m * b_n;
        for (knt = 0; knt < m; knt++) {
          smax = A->data[b_ix];
          A->data[b_ix] = A->data[iy];
          A->data[iy] = smax;
          b_ix++;
          iy++;
        }

        b_ix = jpvt->data[ix];
        jpvt->data[ix] = jpvt->data[b_n];
        jpvt->data[b_n] = b_ix;
        vn1->data[ix] = vn1->data[b_n];
        vn2->data[ix] = vn2->data[b_n];
      }

      if (b_n + 1 < m) {
        smax = A->data[i_i];
        tau->data[b_n] = 0.0F;
        if (1 + yk > 0) {
          s = xnrm2(yk, A, i_i + 2);
          if (s != 0.0F) {
            s = rt_hypotf(A->data[i_i], s);
            if (A->data[i_i] >= 0.0F) {
              s = -s;
            }

            if (std::abs(s) < 9.86076132E-32F) {
              knt = -1;
              do {
                knt++;
                xscal(yk, 1.01412048E+31F, A, i_i + 2);
                s *= 1.01412048E+31F;
                smax *= 1.01412048E+31F;
              } while (std::abs(s) < 9.86076132E-32F);

              s = rt_hypotf(smax, xnrm2(yk, A, i_i + 2));
              if (smax >= 0.0F) {
                s = -s;
              }

              tau->data[b_n] = (s - smax) / s;
              xscal(yk, 1.0F / (smax - s), A, i_i + 2);
              for (ix = 0; ix <= knt; ix++) {
                s *= 9.86076132E-32F;
              }

              smax = s;
            } else {
              tau->data[b_n] = (s - A->data[i_i]) / s;
              xscal(yk, 1.0F / (A->data[i_i] - s), A, i_i + 2);
              smax = s;
            }
          }
        }

        A->data[i_i] = smax;
      } else {
        tau->data[b_n] = 0.0F;
      }

      if (b_n + 1 < n) {
        smax = A->data[i_i];
        A->data[i_i] = 1.0F;
        b_ix = ((b_n + 1) * m + b_n) + 1;
        if (tau->data[b_n] != 0.0F) {
          ix = yk + 1;
          knt = i_i + yk;
          while ((ix > 0) && (A->data[knt] == 0.0F)) {
            ix--;
            knt--;
          }

          knt = nmi - 2;
          exitg2 = false;
          while ((!exitg2) && (knt + 1 > 0)) {
            nmi = knt * m + b_ix;
            iy = nmi;
            do {
              exitg1 = 0;
              if (iy <= (nmi + ix) - 1) {
                if (A->data[iy - 1] != 0.0F) {
                  exitg1 = 1;
                } else {
                  iy++;
                }
              } else {
                knt--;
                exitg1 = 2;
              }
            } while (exitg1 == 0);

            if (exitg1 == 1) {
              exitg2 = true;
            }
          }

          nmi = knt;
        } else {
          ix = 0;
          nmi = -1;
        }

        if (ix > 0) {
          if (nmi + 1 != 0) {
            for (knt = 0; knt <= nmi; knt++) {
              work->data[knt] = 0.0F;
            }

            knt = 0;
            iy = m * nmi + b_ix;
            d_ix = b_ix;
            while (((m > 0) && (d_ix <= iy)) || ((m < 0) && (d_ix >= iy))) {
              c_ix = i_i;
              s = 0.0F;
              e = (d_ix + ix) - 1;
              for (b_ia = d_ix; b_ia <= e; b_ia++) {
                s += A->data[b_ia - 1] * A->data[c_ix];
                c_ix++;
              }

              work->data[knt] += s;
              knt++;
              d_ix += m;
            }
          }

          if (-tau->data[b_n] != 0.0F) {
            b_ix--;
            knt = 0;
            for (iy = 0; iy <= nmi; iy++) {
              if (work->data[knt] != 0.0F) {
                s = work->data[knt] * -tau->data[b_n];
                d_ix = i_i;
                c_ix = ix + b_ix;
                for (e = b_ix; e < c_ix; e++) {
                  A->data[e] += A->data[d_ix] * s;
                  d_ix++;
                }
              }

              knt++;
              b_ix += m;
            }
          }
        }

        A->data[i_i] = smax;
      }

      for (i_i = b_n + 1; i_i < n; i_i++) {
        if (vn1->data[i_i] != 0.0F) {
          smax = std::abs(A->data[A->size[0] * i_i + b_n]) / vn1->data[i_i];
          smax = 1.0F - smax * smax;
          if (smax < 0.0F) {
            smax = 0.0F;
          }

          s = vn1->data[i_i] / vn2->data[i_i];
          s = s * s * smax;
          if (s <= 0.000345266977F) {
            if (b_n + 1 < m) {
              vn1->data[i_i] = xnrm2(yk, A, (m * i_i + b_n) + 2);
              vn2->data[i_i] = vn1->data[i_i];
            } else {
              vn1->data[i_i] = 0.0F;
              vn2->data[i_i] = 0.0F;
            }
          } else {
            vn1->data[i_i] *= std::sqrt(smax);
          }
        }
      }
    }

    emxFree_real32_T(&vn2);
    emxFree_real32_T(&vn1);
    emxFree_real32_T(&work);
  }
}

void MatlabControllerClass::emxFree_int32_T(emxArray_int32_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_int32_T *)NULL) {
    if (((*pEmxArray)->data != (int32_T *)NULL) && (*pEmxArray)->canFreeData) {
      free((*pEmxArray)->data);
    }

    free((*pEmxArray)->size);
    free(*pEmxArray);
    *pEmxArray = (emxArray_int32_T *)NULL;
  }
}

// Function for MATLAB Function: '<S29>/trajFromWaypoints'
void MatlabControllerClass::lusolve(const emxArray_real32_T *A,
  emxArray_real32_T *B_1)
{
  int32_T n;
  emxArray_real32_T *b_A;
  emxArray_int32_T *ipiv;
  int32_T mmj;
  int32_T b_n;
  int32_T yk;
  int32_T b_c;
  int32_T ix;
  real32_T smax;
  real32_T s;
  int32_T iy;
  int32_T c_ix;
  int32_T e;
  int32_T ijA;
  int32_T b_kAcol;
  emxInit_real32_T(&b_A, 2);
  n = A->size[1];
  yk = b_A->size[0] * b_A->size[1];
  b_A->size[0] = A->size[0];
  b_A->size[1] = A->size[1];
  emxEnsureCapacity_real32_T(b_A, yk);
  b_kAcol = A->size[0] * A->size[1] - 1;
  for (yk = 0; yk <= b_kAcol; yk++) {
    b_A->data[yk] = A->data[yk];
  }

  emxInit_int32_T(&ipiv, 2);
  b_kAcol = A->size[1];
  b_n = A->size[1];
  if (b_kAcol < b_n) {
    b_n = b_kAcol;
  }

  if (b_n < 1) {
    b_n = 0;
  }

  yk = ipiv->size[0] * ipiv->size[1];
  ipiv->size[0] = 1;
  ipiv->size[1] = b_n;
  emxEnsureCapacity_int32_T(ipiv, yk);
  if (b_n > 0) {
    ipiv->data[0] = 1;
    yk = 1;
    for (b_kAcol = 2; b_kAcol <= b_n; b_kAcol++) {
      yk++;
      ipiv->data[b_kAcol - 1] = yk;
    }
  }

  if (A->size[1] >= 1) {
    b_kAcol = A->size[1] - 1;
    yk = A->size[1];
    if (b_kAcol < yk) {
      yk = b_kAcol;
    }

    for (b_kAcol = 0; b_kAcol < yk; b_kAcol++) {
      mmj = n - b_kAcol;
      b_c = (n + 1) * b_kAcol;
      if (mmj < 1) {
        b_n = -1;
      } else {
        b_n = 0;
        if (mmj > 1) {
          ix = b_c;
          smax = std::abs(b_A->data[b_c]);
          for (iy = 2; iy <= mmj; iy++) {
            ix++;
            s = std::abs(b_A->data[ix]);
            if (s > smax) {
              b_n = iy - 1;
              smax = s;
            }
          }
        }
      }

      if (b_A->data[b_c + b_n] != 0.0F) {
        if (b_n != 0) {
          iy = b_kAcol + b_n;
          ipiv->data[b_kAcol] = iy + 1;
          ix = b_kAcol;
          for (b_n = 0; b_n < n; b_n++) {
            smax = b_A->data[ix];
            b_A->data[ix] = b_A->data[iy];
            b_A->data[iy] = smax;
            ix += n;
            iy += n;
          }
        }

        iy = b_c + mmj;
        for (b_n = b_c + 1; b_n < iy; b_n++) {
          b_A->data[b_n] /= b_A->data[b_c];
        }
      }

      ix = b_c + n;
      b_n = ix + 1;
      for (iy = 0; iy <= mmj - 2; iy++) {
        smax = b_A->data[ix];
        if (b_A->data[ix] != 0.0F) {
          c_ix = b_c + 1;
          e = mmj + b_n;
          for (ijA = b_n; ijA < e - 1; ijA++) {
            b_A->data[ijA] += b_A->data[c_ix] * -smax;
            c_ix++;
          }
        }

        ix += n;
        b_n += n;
      }
    }
  }

  for (yk = 0; yk <= n - 2; yk++) {
    if (yk + 1 != ipiv->data[yk]) {
      smax = B_1->data[yk];
      B_1->data[yk] = B_1->data[ipiv->data[yk] - 1];
      B_1->data[ipiv->data[yk] - 1] = smax;
    }
  }

  emxFree_int32_T(&ipiv);
  for (yk = 0; yk < n; yk++) {
    b_kAcol = n * yk;
    if (B_1->data[yk] != 0.0F) {
      for (mmj = yk + 1; mmj < n; mmj++) {
        B_1->data[mmj] -= b_A->data[mmj + b_kAcol] * B_1->data[yk];
      }
    }
  }

  for (yk = A->size[1] - 1; yk + 1 > 0; yk--) {
    b_kAcol = n * yk;
    if (B_1->data[yk] != 0.0F) {
      B_1->data[yk] /= b_A->data[yk + b_kAcol];
      for (mmj = 0; mmj < yk; mmj++) {
        B_1->data[mmj] -= b_A->data[mmj + b_kAcol] * B_1->data[yk];
      }
    }
  }

  emxFree_real32_T(&b_A);
}

// Function for MATLAB Function: '<S29>/trajFromWaypoints'
void MatlabControllerClass::mldivide(const emxArray_real32_T *A, const
  emxArray_real32_T *B_0, emxArray_real32_T *Y)
{
  emxArray_real32_T *b_A;
  emxArray_real32_T *tau;
  emxArray_int32_T *jpvt;
  int32_T rankR;
  int32_T minmn;
  int32_T maxmn;
  emxArray_real32_T *b_B;
  real32_T wj;
  int32_T c_i;
  uint32_T b_idx_0;
  int32_T u1;
  boolean_T exitg1;
  emxInit_real32_T(&b_A, 2);
  emxInit_real32_T(&tau, 1);
  emxInit_int32_T(&jpvt, 2);
  emxInit_real32_T(&b_B, 1);
  if ((A->size[0] == 0) || (A->size[1] == 0) || (B_0->size[0] == 0)) {
    b_idx_0 = (uint32_T)A->size[1];
    minmn = Y->size[0];
    Y->size[0] = (int32_T)b_idx_0;
    emxEnsureCapacity_real32_T(Y, minmn);
    maxmn = (int32_T)b_idx_0;
    for (minmn = 0; minmn < maxmn; minmn++) {
      Y->data[minmn] = 0.0F;
    }
  } else if (A->size[0] == A->size[1]) {
    minmn = Y->size[0];
    Y->size[0] = B_0->size[0];
    emxEnsureCapacity_real32_T(Y, minmn);
    maxmn = B_0->size[0];
    for (minmn = 0; minmn < maxmn; minmn++) {
      Y->data[minmn] = B_0->data[minmn];
    }

    lusolve(A, Y);
  } else {
    minmn = b_A->size[0] * b_A->size[1];
    b_A->size[0] = A->size[0];
    b_A->size[1] = A->size[1];
    emxEnsureCapacity_real32_T(b_A, minmn);
    maxmn = A->size[0] * A->size[1] - 1;
    for (minmn = 0; minmn <= maxmn; minmn++) {
      b_A->data[minmn] = A->data[minmn];
    }

    xgeqp3(b_A, tau, jpvt);
    rankR = 0;
    if (b_A->size[0] < b_A->size[1]) {
      minmn = b_A->size[0];
      maxmn = b_A->size[1];
    } else {
      minmn = b_A->size[1];
      maxmn = b_A->size[0];
    }

    if (minmn > 0) {
      exitg1 = false;
      while ((!exitg1) && (rankR < minmn)) {
        wj = 1.1920929E-6F * (real32_T)maxmn;
        if (0.000345266977F < wj) {
          wj = 0.000345266977F;
        }

        if (std::abs(b_A->data[b_A->size[0] * rankR + rankR]) > wj * std::abs
            (b_A->data[0])) {
          rankR++;
        } else {
          exitg1 = true;
        }
      }
    }

    maxmn = b_A->size[1];
    minmn = Y->size[0];
    Y->size[0] = maxmn;
    emxEnsureCapacity_real32_T(Y, minmn);
    for (minmn = 0; minmn < maxmn; minmn++) {
      Y->data[minmn] = 0.0F;
    }

    minmn = b_B->size[0];
    b_B->size[0] = B_0->size[0];
    emxEnsureCapacity_real32_T(b_B, minmn);
    maxmn = B_0->size[0];
    for (minmn = 0; minmn < maxmn; minmn++) {
      b_B->data[minmn] = B_0->data[minmn];
    }

    minmn = b_A->size[0];
    maxmn = b_A->size[0];
    u1 = b_A->size[1];
    if (maxmn < u1) {
      u1 = maxmn;
    }

    maxmn = u1 - 1;
    for (u1 = 0; u1 <= maxmn; u1++) {
      if (tau->data[u1] != 0.0F) {
        wj = b_B->data[u1];
        for (c_i = u1 + 1; c_i < minmn; c_i++) {
          wj += b_A->data[b_A->size[0] * u1 + c_i] * b_B->data[c_i];
        }

        wj *= tau->data[u1];
        if (wj != 0.0F) {
          b_B->data[u1] -= wj;
          for (c_i = u1 + 1; c_i < minmn; c_i++) {
            b_B->data[c_i] -= b_A->data[b_A->size[0] * u1 + c_i] * wj;
          }
        }
      }
    }

    for (maxmn = 0; maxmn < rankR; maxmn++) {
      Y->data[jpvt->data[maxmn] - 1] = b_B->data[maxmn];
    }

    for (rankR--; rankR + 1 > 0; rankR--) {
      Y->data[jpvt->data[rankR] - 1] /= b_A->data[b_A->size[0] * rankR + rankR];
      for (u1 = 0; u1 < rankR; u1++) {
        Y->data[jpvt->data[u1] - 1] -= b_A->data[b_A->size[0] * rankR + u1] *
          Y->data[jpvt->data[rankR] - 1];
      }
    }
  }

  emxFree_real32_T(&b_B);
  emxFree_int32_T(&jpvt);
  emxFree_real32_T(&tau);
  emxFree_real32_T(&b_A);
}

// Function for MATLAB Function: '<S29>/trajFromWaypoints'
void MatlabControllerClass::polyder(const emxArray_real_T *u, emxArray_real_T *a)
{
  int32_T nymax;
  int32_T nlead0;
  int32_T ny;
  int32_T tmp;
  if (u->size[1] < 2) {
    nymax = 1;
  } else {
    nymax = u->size[1] - 1;
  }

  tmp = a->size[0] * a->size[1];
  a->size[0] = 1;
  a->size[1] = nymax;
  emxEnsureCapacity_real_T(a, tmp);
  switch (u->size[1]) {
   case 0:
    a->data[0] = 0.0;
    break;

   case 1:
    a->data[0] = 0.0;
    break;

   default:
    nlead0 = 0;
    ny = 0;
    while ((ny <= nymax - 2) && (u->data[ny] == 0.0)) {
      nlead0++;
      ny++;
    }

    ny = nymax - nlead0;
    tmp = a->size[0] * a->size[1];
    a->size[0] = 1;
    a->size[1] = ny;
    emxEnsureCapacity_real_T(a, tmp);
    for (nymax = 0; nymax < ny; nymax++) {
      a->data[nymax] = u->data[nymax + nlead0];
    }
    break;
  }

  nlead0 = a->size[1] - 2;
  for (ny = 0; ny <= nlead0; ny++) {
    a->data[ny] *= (real_T)((nlead0 - ny) + 1) + 1.0;
  }
}

// Function for MATLAB Function: '<S29>/trajFromWaypoints'
void MatlabControllerClass::polyInterpolation(const real32_T points[4], real32_T
  degree, boolean_T cycle, emxArray_real32_T *coeffs, real_T *num_of_splines)
{
  real32_T points_new_data[5];
  emxArray_real_T *pp;
  emxArray_real_T *point_0;
  real32_T intermediate_size;
  real32_T size_A_mat;
  emxArray_real32_T *A;
  emxArray_real32_T *b;
  real32_T bnd_left;
  real32_T last_right;
  int32_T c;
  int32_T d;
  int32_T eb;
  int32_T fb;
  emxArray_real32_T *qb;
  emxArray_real_T *pp_0;
  emxArray_real_T *tmp;
  int32_T loop_ub;
  real32_T points_0[5];
  int32_T i;
  int32_T i_0;
  int32_T i_1;
  int32_T points_new_size_idx_1;
  int32_T degree_idx_1;
  real32_T intermediate_size_tmp;
  int32_T intermediate_size_tmp_0;
  int32_T intermediate_size_tmp_1;
  int32_T exitg1;
  if (cycle) {
    points_0[0] = points[0];
    points_0[1] = points[1];
    points_0[2] = points[2];
    points_0[3] = points[3];
    points_0[4] = points[0];
    points_new_size_idx_1 = 5;
    for (i = 0; i < 5; i++) {
      points_new_data[i] = points_0[i];
    }
  } else {
    points_new_size_idx_1 = 4;
    points_new_data[0] = points[0];
    points_new_data[1] = points[1];
    points_new_data[2] = points[2];
    points_new_data[3] = points[3];
  }

  emxInit_real_T(&pp, 2);
  i = pp->size[0] * pp->size[1];
  degree_idx_1 = (int32_T)(degree + 1.0F);
  pp->size[0] = degree_idx_1;
  pp->size[1] = degree_idx_1;
  emxEnsureCapacity_real_T(pp, i);
  loop_ub = degree_idx_1 * degree_idx_1 - 1;
  for (i = 0; i <= loop_ub; i++) {
    pp->data[i] = 1.0;
  }

  if (2 > degree_idx_1) {
    d = 1;
    c = 0;
  } else {
    d = 2;
    c = degree_idx_1;
  }

  c = (c - d) + 1;
  i = d - 1;
  for (i_1 = 0; i_1 < degree_idx_1; i_1++) {
    for (i_0 = 0; i_0 < c; i_0++) {
      pp->data[(i + i_0) + pp->size[0] * i_1] = 0.0;
    }
  }

  d = 0;
  emxInit_real_T(&pp_0, 2);
  emxInit_real_T(&tmp, 2);
  do {
    exitg1 = 0;
    i = (int32_T)degree;
    i_1 = i - 1;
    if (d <= i_1) {
      bnd_left = ((real32_T)pp->size[1] - (1.0F + (real32_T)d)) + 1.0F;
      if (1.0F > bnd_left) {
        loop_ub = 0;
      } else {
        loop_ub = (int32_T)bnd_left;
      }

      i = (int32_T)(1.0F + (real32_T)d);
      i_1 = pp_0->size[0] * pp_0->size[1];
      pp_0->size[0] = 1;
      pp_0->size[1] = loop_ub;
      emxEnsureCapacity_real_T(pp_0, i_1);
      for (i_1 = 0; i_1 < loop_ub; i_1++) {
        pp_0->data[i_1] = pp->data[(pp->size[0] * i_1 + i) - 1];
      }

      polyder(pp_0, tmp);
      i = (int32_T)((1.0F + (real32_T)d) + 1.0F);
      loop_ub = tmp->size[1];
      for (i_1 = 0; i_1 < loop_ub; i_1++) {
        pp->data[(i + pp->size[0] * i_1) - 1] = tmp->data[i_1];
      }

      d++;
    } else {
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  emxFree_real_T(&tmp);
  emxFree_real_T(&pp_0);
  emxInit_real_T(&point_0, 2);
  i_0 = point_0->size[0] * point_0->size[1];
  point_0->size[0] = pp->size[0];
  point_0->size[1] = pp->size[1];
  emxEnsureCapacity_real_T(point_0, i_0);
  loop_ub = pp->size[0] * pp->size[1] - 1;
  for (i_0 = 0; i_0 <= loop_ub; i_0++) {
    point_0->data[i_0] = pp->data[i_0];
  }

  for (d = 0; d <= i_1; d++) {
    bnd_left = (real32_T)point_0->size[1] - (1.0F + (real32_T)d);
    if (1.0F > bnd_left) {
      c = 0;
    } else {
      c = (int32_T)bnd_left;
    }

    i_0 = (int32_T)(1.0F + (real32_T)d);
    for (loop_ub = 0; loop_ub < c; loop_ub++) {
      point_0->data[(i_0 + point_0->size[0] * loop_ub) - 1] = 0.0;
    }
  }

  emxInit_real32_T(&A, 2);
  intermediate_size_tmp = (((real32_T)points_new_size_idx_1 - 1.0F) - 1.0F) *
    (degree + 1.0F);
  size_A_mat = (degree + 1.0F) * ((real32_T)points_new_size_idx_1 - 1.0F);
  i_1 = A->size[0] * A->size[1];
  loop_ub = (int32_T)size_A_mat;
  A->size[0] = loop_ub;
  A->size[1] = loop_ub;
  emxEnsureCapacity_real32_T(A, i_1);
  degree_idx_1 = loop_ub * loop_ub - 1;
  for (i_1 = 0; i_1 <= degree_idx_1; i_1++) {
    A->data[i_1] = 0.0F;
  }

  emxInit_real32_T(&b, 1);
  i_1 = b->size[0];
  b->size[0] = loop_ub;
  emxEnsureCapacity_real32_T(b, i_1);
  for (i_1 = 0; i_1 < loop_ub; i_1++) {
    b->data[i_1] = 0.0F;
  }

  if (!cycle) {
    last_right = (degree + 1.0F) / 2.0F;
    bnd_left = std::ceil(last_right);
    last_right = std::floor(last_right);
    if (1.0F > bnd_left) {
      i_1 = 0;
    } else {
      i_1 = (int32_T)bnd_left;
    }

    loop_ub = i_1 - 1;
    degree_idx_1 = point_0->size[1] - 1;
    for (i_1 = 0; i_1 <= degree_idx_1; i_1++) {
      for (i_0 = 0; i_0 <= loop_ub; i_0++) {
        A->data[i_0 + A->size[0] * i_1] = (real32_T)point_0->data[point_0->size
          [0] * i_1 + i_0];
      }
    }

    intermediate_size = (bnd_left + 1.0F) + intermediate_size_tmp;
    if (intermediate_size > (intermediate_size + last_right) - 1.0F) {
      d = 0;
    } else {
      d = (int32_T)intermediate_size - 1;
    }

    if (1.0F > last_right) {
      i_1 = 0;
    } else {
      i_1 = (int32_T)last_right;
    }

    loop_ub = i_1 - 1;
    degree_idx_1 = pp->size[1] - 1;
    for (i_1 = 0; i_1 <= degree_idx_1; i_1++) {
      for (i_0 = 0; i_0 <= loop_ub; i_0++) {
        A->data[(d + i_0) + A->size[0] * ((int32_T)((1.0F +
          intermediate_size_tmp) + (real32_T)i_1) - 1)] = (real32_T)pp->data
          [pp->size[0] * i_1 + i_0];
      }
    }

    b->data[0] = points_new_data[0];
    b->data[(int32_T)(((real32_T)(int32_T)size_A_mat - last_right) + 1.0F) - 1] =
      points_new_data[points_new_size_idx_1 - 1];
    if (degree > 1.0F) {
      b->data[1] = points_new_data[1] - points_new_data[0];
    }

    if (degree > 2.0F) {
      b->data[(int32_T)(((real32_T)b->size[0] - last_right) + 2.0F) - 1] =
        points_new_data[points_new_size_idx_1 - 1] -
        points_new_data[points_new_size_idx_1 - 2];
    }
  } else {
    bnd_left = 2.0F;
    loop_ub = point_0->size[1] - 1;
    for (i_1 = 0; i_1 <= loop_ub; i_1++) {
      A->data[A->size[0] * i_1] = (real32_T)point_0->data[point_0->size[0] * i_1];
    }

    loop_ub = pp->size[1] - 1;
    for (i_1 = 0; i_1 <= loop_ub; i_1++) {
      A->data[1 + A->size[0] * ((int32_T)((intermediate_size_tmp + 1.0F) +
        (real32_T)i_1) - 1)] = (real32_T)pp->data[pp->size[0] * i_1];
    }

    b->data[0] = points_new_data[0];
    b->data[1] = points_new_data[points_new_size_idx_1 - 1];
    if (2.0F > degree) {
      d = 0;
      c = 0;
    } else {
      d = 1;
      c = i;
    }

    last_right = (((intermediate_size_tmp + 2.0F) + 1.0F) + degree) - 2.0F;
    if ((intermediate_size_tmp + 2.0F) + 1.0F > last_right) {
      fb = 0;
    } else {
      fb = (int32_T)((intermediate_size_tmp + 2.0F) + 1.0F) - 1;
    }

    loop_ub = point_0->size[1] - 1;
    for (i_1 = 0; i_1 <= loop_ub; i_1++) {
      degree_idx_1 = c - d;
      for (i_0 = 0; i_0 < degree_idx_1; i_0++) {
        A->data[(fb + i_0) + A->size[0] * i_1] = (real32_T)point_0->data[(d +
          i_0) + point_0->size[0] * i_1];
      }
    }

    if (2.0F > degree) {
      d = 0;
      c = 0;
    } else {
      d = 1;
      c = i;
    }

    if ((intermediate_size_tmp + 2.0F) + 1.0F > last_right) {
      fb = 0;
    } else {
      fb = (int32_T)((intermediate_size_tmp + 2.0F) + 1.0F) - 1;
    }

    loop_ub = pp->size[1] - 1;
    for (i_1 = 0; i_1 <= loop_ub; i_1++) {
      degree_idx_1 = c - d;
      for (i_0 = 0; i_0 < degree_idx_1; i_0++) {
        A->data[(fb + i_0) + A->size[0] * ((int32_T)((intermediate_size_tmp +
          1.0F) + (real32_T)i_1) - 1)] = (real32_T)-pp->data[(d + i_0) +
          pp->size[0] * i_1];
      }
    }
  }

  for (d = 0; d <= points_new_size_idx_1 - 3; d++) {
    intermediate_size_tmp = ((1.0F + (real32_T)d) - 1.0F) * (degree + 1.0F);
    intermediate_size = intermediate_size_tmp + bnd_left;
    size_A_mat = (((1.0F + (real32_T)d) - 1.0F) * (degree + 1.0F) + 1.0F) +
      (degree + 1.0F);
    loop_ub = pp->size[1] - 1;
    intermediate_size_tmp_0 = (int32_T)(intermediate_size + 1.0F);
    for (i_1 = 0; i_1 <= loop_ub; i_1++) {
      A->data[(intermediate_size_tmp_0 + A->size[0] * ((int32_T)
                ((intermediate_size_tmp + 1.0F) + (real32_T)i_1) - 1)) - 1] =
        (real32_T)pp->data[pp->size[0] * i_1];
    }

    loop_ub = point_0->size[1] - 1;
    intermediate_size_tmp_1 = (int32_T)(intermediate_size + 2.0F);
    for (i_1 = 0; i_1 <= loop_ub; i_1++) {
      A->data[(intermediate_size_tmp_1 + A->size[0] * ((int32_T)(size_A_mat +
                 (real32_T)i_1) - 1)) - 1] = (real32_T)point_0->data
        [point_0->size[0] * i_1];
    }

    if (2.0F > degree) {
      c = 0;
      fb = 0;
    } else {
      c = 1;
      fb = i;
    }

    last_right = ((intermediate_size + 3.0F) + degree) - 2.0F;
    if (intermediate_size + 3.0F > last_right) {
      eb = 0;
    } else {
      eb = (int32_T)(intermediate_size + 3.0F) - 1;
    }

    loop_ub = pp->size[1] - 1;
    for (i_1 = 0; i_1 <= loop_ub; i_1++) {
      degree_idx_1 = fb - c;
      for (i_0 = 0; i_0 < degree_idx_1; i_0++) {
        A->data[(eb + i_0) + A->size[0] * ((int32_T)((intermediate_size_tmp +
          1.0F) + (real32_T)i_1) - 1)] = (real32_T)pp->data[(c + i_0) + pp->
          size[0] * i_1];
      }
    }

    if (2.0F > degree) {
      c = 0;
      fb = 0;
    } else {
      c = 1;
      fb = i;
    }

    if (intermediate_size + 3.0F > last_right) {
      eb = 0;
    } else {
      eb = (int32_T)(intermediate_size + 3.0F) - 1;
    }

    loop_ub = point_0->size[1] - 1;
    for (i_1 = 0; i_1 <= loop_ub; i_1++) {
      degree_idx_1 = fb - c;
      for (i_0 = 0; i_0 < degree_idx_1; i_0++) {
        A->data[(eb + i_0) + A->size[0] * ((int32_T)(size_A_mat + (real32_T)i_1)
          - 1)] = (real32_T)-point_0->data[(c + i_0) + point_0->size[0] * i_1];
      }
    }

    last_right = points_new_data[d + 1];
    b->data[intermediate_size_tmp_0 - 1] = last_right;
    b->data[intermediate_size_tmp_1 - 1] = last_right;
  }

  emxFree_real_T(&point_0);
  emxFree_real_T(&pp);
  emxInit_real32_T(&qb, 1);
  mldivide(A, b, qb);
  i = coeffs->size[0] * coeffs->size[1];
  coeffs->size[0] = 1;
  coeffs->size[1] = qb->size[0];
  emxEnsureCapacity_real32_T(coeffs, i);
  loop_ub = qb->size[0];
  emxFree_real32_T(&b);
  emxFree_real32_T(&A);
  for (i = 0; i < loop_ub; i++) {
    coeffs->data[i] = qb->data[i];
  }

  emxFree_real32_T(&qb);
  *num_of_splines = points_new_size_idx_1 - 1;
}

// Function for MATLAB Function: '<S29>/trajFromWaypoints'
void MatlabControllerClass::polyder_p(const real32_T u[6], real32_T a_data[],
  int32_T a_size[2])
{
  int32_T nlead0;
  int32_T b_k;
  nlead0 = 0;
  b_k = 0;
  while ((b_k < 4) && (u[b_k] == 0.0F)) {
    nlead0++;
    b_k++;
  }

  a_size[0] = 1;
  a_size[1] = 5 - nlead0;
  for (b_k = 0; b_k <= 4 - nlead0; b_k++) {
    a_data[b_k] = u[b_k + nlead0];
  }

  nlead0 = a_size[1] - 2;
  for (b_k = 0; b_k <= nlead0; b_k++) {
    a_data[b_k] *= (real32_T)((nlead0 - b_k) + 1) + 1.0F;
  }
}

// Function for MATLAB Function: '<S29>/trajFromWaypoints'
real32_T MatlabControllerClass::polyVal(const real32_T p_data[], const int32_T
  p_size[2], real_T x)
{
  real32_T y;
  int32_T i;
  y = 0.0F;
  if (p_size[1] > 0) {
    y = p_data[0];
  }

  for (i = 0; i <= p_size[1] - 2; i++) {
    y = (real32_T)x * y + p_data[i + 1];
  }

  return y;
}

// Function for MATLAB Function: '<S25>/flight path look ahead'
real32_T MatlabControllerClass::integralSimpson(const b_cell_wrap_1
  func_tunableEnvironment[3], real_T A, real_T B_2, real_T steps)
{
  real32_T Q;
  real_T t;
  real_T step;
  real_T step_1_2;
  int32_T i;
  real32_T a;
  real32_T b_a;
  real32_T c_a;
  real32_T d_a;
  real32_T e_a;
  real32_T f_a;
  real_T varargin_1;
  t = A;
  step = (B_2 - A) / steps;
  step_1_2 = 0.5 * step;
  a = polyVal(func_tunableEnvironment[0].f1.data, func_tunableEnvironment[0].
              f1.size, A);
  b_a = polyVal(func_tunableEnvironment[1].f1.data, func_tunableEnvironment[1].
                f1.size, A);
  c_a = polyVal(func_tunableEnvironment[2].f1.data, func_tunableEnvironment[2].
                f1.size, A);
  d_a = polyVal(func_tunableEnvironment[0].f1.data, func_tunableEnvironment[0].
                f1.size, B_2);
  e_a = polyVal(func_tunableEnvironment[1].f1.data, func_tunableEnvironment[1].
                f1.size, B_2);
  f_a = polyVal(func_tunableEnvironment[2].f1.data, func_tunableEnvironment[2].
                f1.size, B_2);
  Q = std::sqrt((a * a + b_a * b_a) + c_a * c_a) * 0.5F - std::sqrt((d_a * d_a +
    e_a * e_a) + f_a * f_a) * 0.5F;
  for (i = 0; i < (int32_T)steps; i++) {
    t += step;
    varargin_1 = t - step_1_2;
    a = polyVal(func_tunableEnvironment[0].f1.data, func_tunableEnvironment[0].
                f1.size, varargin_1);
    b_a = polyVal(func_tunableEnvironment[1].f1.data, func_tunableEnvironment[1]
                  .f1.size, varargin_1);
    c_a = polyVal(func_tunableEnvironment[2].f1.data, func_tunableEnvironment[2]
                  .f1.size, varargin_1);
    d_a = polyVal(func_tunableEnvironment[0].f1.data, func_tunableEnvironment[0]
                  .f1.size, t);
    e_a = polyVal(func_tunableEnvironment[1].f1.data, func_tunableEnvironment[1]
                  .f1.size, t);
    f_a = polyVal(func_tunableEnvironment[2].f1.data, func_tunableEnvironment[2]
                  .f1.size, t);
    Q = (std::sqrt((a * a + b_a * b_a) + c_a * c_a) * 2.0F + Q) + std::sqrt((d_a
      * d_a + e_a * e_a) + f_a * f_a);
  }

  Q *= (real32_T)(0.33333333333333331 * step);
  return Q;
}

// Function for MATLAB Function: '<S25>/flight path look ahead'
void MatlabControllerClass::trajSectionGetArcLength(const real32_T
  traj_section_pos_x[6], const real32_T traj_section_pos_y[6], const real32_T
  traj_section_pos_z[6], real32_T varargin_1, real32_T *arc_length, real32_T
  *arc_length_dt)
{
  real_T t;
  real32_T dx_data[5];
  real32_T dy_data[5];
  real32_T dz_data[5];
  b_cell_wrap_1 tunableEnvironment[3];
  real32_T a;
  real32_T b_a;
  real32_T c_a;
  int32_T loop_ub;
  int32_T i;
  int32_T dx_size[2];
  int32_T dy_size[2];
  int32_T dz_size[2];
  if (1.1 < varargin_1) {
    t = 1.1;
  } else {
    t = varargin_1;
  }

  if (t <= -0.1) {
    t = -0.1;
  }

  polyder_p(traj_section_pos_x, dx_data, dx_size);
  polyder_p(traj_section_pos_y, dy_data, dy_size);
  polyder_p(traj_section_pos_z, dz_data, dz_size);
  tunableEnvironment[0].f1.size[0] = 1;
  tunableEnvironment[0].f1.size[1] = dx_size[1];
  loop_ub = dx_size[0] * dx_size[1] - 1;
  for (i = 0; i <= loop_ub; i++) {
    tunableEnvironment[0].f1.data[i] = dx_data[i];
  }

  tunableEnvironment[1].f1.size[0] = 1;
  tunableEnvironment[1].f1.size[1] = dy_size[1];
  loop_ub = dy_size[0] * dy_size[1] - 1;
  for (i = 0; i <= loop_ub; i++) {
    tunableEnvironment[1].f1.data[i] = dy_data[i];
  }

  tunableEnvironment[2].f1.size[0] = 1;
  tunableEnvironment[2].f1.size[1] = dz_size[1];
  loop_ub = dz_size[0] * dz_size[1] - 1;
  for (i = 0; i <= loop_ub; i++) {
    tunableEnvironment[2].f1.data[i] = dz_data[i];
  }

  *arc_length = integralSimpson(tunableEnvironment, 0.0, t, 15.0);
  a = polyVal(dx_data, dx_size, t);
  b_a = polyVal(dy_data, dy_size, t);
  c_a = polyVal(dz_data, dz_size, t);
  *arc_length_dt = std::sqrt((a * a + b_a * b_a) + c_a * c_a);
}

// Function for MATLAB Function: '<S25>/flight path look ahead'
void MatlabControllerClass::trajSectionGetArcLength_l(const real32_T
  traj_section_pos_x[6], const real32_T traj_section_pos_y[6], const real32_T
  traj_section_pos_z[6], real_T varargin_1, real32_T *arc_length, real32_T
  *arc_length_dt)
{
  real_T t;
  real32_T dx_data[5];
  real32_T dy_data[5];
  real32_T dz_data[5];
  b_cell_wrap_1 tunableEnvironment[3];
  real32_T a;
  real32_T b_a;
  real32_T c_a;
  int32_T loop_ub;
  int32_T i;
  int32_T dx_size[2];
  int32_T dy_size[2];
  int32_T dz_size[2];
  if (1.1 < varargin_1) {
    t = 1.1;
  } else {
    t = varargin_1;
  }

  if (t <= -0.1) {
    t = -0.1;
  }

  polyder_p(traj_section_pos_x, dx_data, dx_size);
  polyder_p(traj_section_pos_y, dy_data, dy_size);
  polyder_p(traj_section_pos_z, dz_data, dz_size);
  tunableEnvironment[0].f1.size[0] = 1;
  tunableEnvironment[0].f1.size[1] = dx_size[1];
  loop_ub = dx_size[0] * dx_size[1] - 1;
  for (i = 0; i <= loop_ub; i++) {
    tunableEnvironment[0].f1.data[i] = dx_data[i];
  }

  tunableEnvironment[1].f1.size[0] = 1;
  tunableEnvironment[1].f1.size[1] = dy_size[1];
  loop_ub = dy_size[0] * dy_size[1] - 1;
  for (i = 0; i <= loop_ub; i++) {
    tunableEnvironment[1].f1.data[i] = dy_data[i];
  }

  tunableEnvironment[2].f1.size[0] = 1;
  tunableEnvironment[2].f1.size[1] = dz_size[1];
  loop_ub = dz_size[0] * dz_size[1] - 1;
  for (i = 0; i <= loop_ub; i++) {
    tunableEnvironment[2].f1.data[i] = dz_data[i];
  }

  *arc_length = integralSimpson(tunableEnvironment, 0.0, t, 15.0);
  a = polyVal(dx_data, dx_size, t);
  b_a = polyVal(dy_data, dy_size, t);
  c_a = polyVal(dz_data, dz_size, t);
  *arc_length_dt = std::sqrt((a * a + b_a * b_a) + c_a * c_a);
}

// Function for MATLAB Function: '<S6>/INDI Copter Acc 2 Lean Vector'
real32_T MatlabControllerClass::norm(const real32_T x[3])
{
  real32_T y;
  real32_T scale;
  real32_T absxk;
  real32_T t;
  scale = 1.29246971E-26F;
  absxk = std::abs(x[0]);
  if (absxk > 1.29246971E-26F) {
    y = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    y = t * t;
  }

  absxk = std::abs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = std::abs(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * std::sqrt(y);
}

// Function for MATLAB Function: '<S26>/flight path matching'
void MatlabControllerClass::polyder_fj(const real32_T u_data[], const int32_T
  u_size[2], real32_T a_data[], int32_T a_size[2])
{
  int32_T nymax;
  int32_T nlead0;
  int32_T ny;
  if (u_size[1] < 2) {
    nymax = 1;
  } else {
    nymax = u_size[1] - 1;
  }

  a_size[0] = 1;
  a_size[1] = nymax;
  switch (u_size[1]) {
   case 0:
    a_data[0] = 0.0F;
    break;

   case 1:
    a_data[0] = 0.0F;
    break;

   default:
    nlead0 = 0;
    ny = 0;
    while ((ny <= nymax - 2) && (u_data[ny] == 0.0F)) {
      nlead0++;
      ny++;
    }

    ny = nymax - nlead0;
    a_size[0] = 1;
    a_size[1] = ny;
    for (nymax = 0; nymax < ny; nymax++) {
      a_data[nymax] = u_data[nymax + nlead0];
    }
    break;
  }

  nlead0 = a_size[1] - 2;
  for (ny = 0; ny <= nlead0; ny++) {
    a_data[ny] *= (real32_T)((nlead0 - ny) + 1) + 1.0F;
  }
}

// Function for MATLAB Function: '<S27>/position controller reference from flight path'
void MatlabControllerClass::trajSectionGetFrenetSerretWit_k(const real32_T
  traj_section_pos_x[6], const real32_T traj_section_pos_y[6], const real32_T
  traj_section_pos_z[6], real32_T vel, real_T varargin_1, real32_T T[3],
  real32_T B_4[3], real32_T N[3], real32_T *kappa, real32_T *tau)
{
  real32_T ddot_r_g[3];
  real32_T dot_r[3];
  real32_T dx_data[5];
  real32_T dy_data[5];
  real32_T dz_data[5];
  real32_T ddx_data[4];
  real32_T ddy_data[4];
  real32_T ddz_data[4];
  real32_T a;
  boolean_T isodd;
  real32_T A[9];
  int8_T ipiv[3];
  int32_T j;
  int32_T c_c;
  int32_T ix;
  real32_T smax;
  real32_T s;
  int32_T c_k;
  int32_T iy;
  int32_T c_ix;
  int32_T d;
  int32_T ijA;
  real32_T tmp_data[4];
  int32_T dx_size[2];
  int32_T dy_size[2];
  int32_T dz_size[2];
  int32_T ddx_size[2];
  real32_T ddot_r_idx_0;
  real32_T ddot_r_idx_1;
  real32_T ddot_r_idx_2;
  real32_T tmp;
  real32_T ddot_r_g_tmp;
  real32_T ddot_r_g_tmp_tmp;
  real32_T a_tmp;
  polyder_p(traj_section_pos_x, dx_data, dx_size);
  polyder_p(traj_section_pos_y, dy_data, dy_size);
  polyder_p(traj_section_pos_z, dz_data, dz_size);
  smax = polyVal(dx_data, dx_size, varargin_1);
  s = polyVal(dy_data, dy_size, varargin_1);
  tmp = polyVal(dz_data, dz_size, varargin_1);
  dot_r[0] = smax;
  dot_r[1] = s;
  dot_r[2] = tmp;
  polyder_fj(dx_data, dx_size, ddx_data, ddx_size);
  polyder_fj(dy_data, dy_size, ddy_data, dx_size);
  polyder_fj(dz_data, dz_size, ddz_data, dy_size);
  ddot_r_idx_0 = polyVal(ddx_data, ddx_size, varargin_1);
  ddot_r_idx_1 = polyVal(ddy_data, dx_size, varargin_1);
  ddot_r_idx_2 = polyVal(ddz_data, dy_size, varargin_1);
  a_tmp = norm(dot_r);
  a = a_tmp * a_tmp;
  ddot_r_g_tmp_tmp = vel * vel;
  ddot_r_g_tmp = 0.0F / ddot_r_g_tmp_tmp * a;
  ddot_r_g[0] = ddot_r_g_tmp + ddot_r_idx_0;
  ddot_r_g[1] = ddot_r_g_tmp + ddot_r_idx_1;
  ddot_r_g[2] = -0.0F / ddot_r_g_tmp_tmp * a + ddot_r_idx_2;
  if (norm(ddot_r_g) > 2.22044605E-16F) {
    ddot_r_idx_0 = ddot_r_g[0];
    ddot_r_idx_1 = ddot_r_g[1];
    ddot_r_idx_2 = ddot_r_g[2];
  }

  ddot_r_g[0] = s * ddot_r_idx_2 - tmp * ddot_r_idx_1;
  ddot_r_g[1] = tmp * ddot_r_idx_0 - smax * ddot_r_idx_2;
  ddot_r_g[2] = smax * ddot_r_idx_1 - s * ddot_r_idx_0;
  if (norm(ddot_r_g) < 2.22044605E-16F) {
    ddot_r_g[0] = 0.0F;
    ddot_r_g[1] = 2.22044605E-16F;
    ddot_r_g[2] = 0.0F;
  }

  a = a_tmp;
  if (a_tmp < 2.22044605E-16F) {
    a = 2.22044605E-16F;
  }

  T[0] = smax / a;
  T[1] = s / a;
  T[2] = tmp / a;
  ddot_r_g_tmp_tmp = norm(ddot_r_g);
  a = ddot_r_g_tmp_tmp;
  if (ddot_r_g_tmp_tmp < 2.22044605E-16F) {
    a = 2.22044605E-16F;
  }

  B_4[0] = ddot_r_g[0] / a;
  A[0] = smax;
  A[3] = ddot_r_idx_0;
  ipiv[0] = 1;
  B_4[1] = ddot_r_g[1] / a;
  A[1] = s;
  A[4] = ddot_r_idx_1;
  ipiv[1] = 2;
  B_4[2] = ddot_r_g[2] / a;
  A[2] = tmp;
  A[5] = ddot_r_idx_2;
  a = std::pow(a_tmp, 3.0F);
  polyder_fj(ddx_data, ddx_size, tmp_data, dz_size);
  polyder_fj(ddy_data, dx_size, ddx_data, ddx_size);
  polyder_fj(ddz_data, dy_size, ddy_data, dx_size);
  A[6] = polyVal(tmp_data, dz_size, varargin_1);
  A[7] = polyVal(ddx_data, ddx_size, varargin_1);
  A[8] = polyVal(ddy_data, dx_size, varargin_1);
  for (j = 0; j < 2; j++) {
    c_c = j << 2;
    iy = 0;
    ix = c_c;
    smax = std::abs(A[c_c]);
    for (c_k = 2; c_k <= 3 - j; c_k++) {
      ix++;
      s = std::abs(A[ix]);
      if (s > smax) {
        iy = c_k - 1;
        smax = s;
      }
    }

    if (A[c_c + iy] != 0.0F) {
      if (iy != 0) {
        iy += j;
        ipiv[j] = (int8_T)(iy + 1);
        smax = A[j];
        A[j] = A[iy];
        A[iy] = smax;
        ix = j + 3;
        iy += 3;
        smax = A[ix];
        A[ix] = A[iy];
        A[iy] = smax;
        ix += 3;
        iy += 3;
        smax = A[ix];
        A[ix] = A[iy];
        A[iy] = smax;
      }

      iy = (c_c - j) + 3;
      for (ix = c_c + 1; ix < iy; ix++) {
        A[ix] /= A[c_c];
      }
    }

    iy = c_c + 4;
    ix = c_c + 3;
    for (c_k = 0; c_k <= 1 - j; c_k++) {
      smax = A[ix];
      if (A[ix] != 0.0F) {
        c_ix = c_c + 1;
        d = (iy - j) + 2;
        for (ijA = iy; ijA < d; ijA++) {
          A[ijA] += A[c_ix] * -smax;
          c_ix++;
        }
      }

      ix += 3;
      iy += 3;
    }
  }

  isodd = false;
  if (ipiv[0] > 1) {
    isodd = true;
  }

  smax = A[0] * A[4] * A[8];
  if (ipiv[1] > 2) {
    isodd = !isodd;
  }

  if (isodd) {
    smax = -smax;
  }

  s = ddot_r_g_tmp_tmp * ddot_r_g_tmp_tmp;
  N[0] = B_4[1] * T[2] - B_4[2] * T[1];
  N[1] = B_4[2] * T[0] - B_4[0] * T[2];
  N[2] = B_4[0] * T[1] - B_4[1] * T[0];
  if (a < 2.22044605E-16F) {
    a = 2.22044605E-16F;
  }

  *kappa = ddot_r_g_tmp_tmp / a;
  if (s < 2.22044605E-16F) {
    s = 2.22044605E-16F;
  }

  *tau = smax / s;
}

// Function for MATLAB Function: '<S41>/MATLAB Function2'
void MatlabControllerClass::LSQFromQR(const real32_T A_data[], const int32_T
  A_size[2], const real32_T tau_data[], const int32_T jpvt_data[], real32_T B_8
  [8], int32_T rankA, real32_T Y_data[], int32_T *Y_size)
{
  int32_T b_i;
  real32_T wj;
  int32_T b_j;
  int32_T loop_ub;
  int8_T b_idx_0;
  b_idx_0 = (int8_T)A_size[1];
  *Y_size = b_idx_0;
  if (0 <= b_idx_0 - 1) {
    memset(&Y_data[0], 0, b_idx_0 * sizeof(real32_T));
  }

  for (b_j = 0; b_j < A_size[1]; b_j++) {
    if (tau_data[b_j] != 0.0F) {
      wj = B_8[b_j];
      for (loop_ub = b_j + 1; loop_ub + 1 < 9; loop_ub++) {
        wj += A_data[(b_j << 3) + loop_ub] * B_8[loop_ub];
      }

      wj *= tau_data[b_j];
      if (wj != 0.0F) {
        B_8[b_j] -= wj;
        for (loop_ub = b_j + 1; loop_ub + 1 < 9; loop_ub++) {
          B_8[loop_ub] -= A_data[(b_j << 3) + loop_ub] * wj;
        }
      }
    }
  }

  for (loop_ub = 0; loop_ub < rankA; loop_ub++) {
    Y_data[jpvt_data[loop_ub] - 1] = B_8[loop_ub];
  }

  for (loop_ub = rankA - 1; loop_ub + 1 > 0; loop_ub--) {
    b_j = loop_ub << 3;
    Y_data[jpvt_data[loop_ub] - 1] /= A_data[b_j + loop_ub];
    for (b_i = 0; b_i < loop_ub; b_i++) {
      Y_data[jpvt_data[b_i] - 1] -= A_data[b_j + b_i] * Y_data[jpvt_data[loop_ub]
        - 1];
    }
  }
}

// Function for MATLAB Function: '<S41>/MATLAB Function2'
real32_T MatlabControllerClass::xnrm2_dg(int32_T n, const real32_T x_data[],
  int32_T ix0)
{
  real32_T y;
  real32_T scale;
  int32_T kend;
  real32_T absxk;
  real32_T t;
  int32_T k;
  y = 0.0F;
  scale = 1.29246971E-26F;
  kend = (ix0 + n) - 1;
  for (k = ix0; k <= kend; k++) {
    absxk = std::abs(x_data[k - 1]);
    if (absxk > scale) {
      t = scale / absxk;
      y = y * t * t + 1.0F;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * std::sqrt(y);
}

// Function for MATLAB Function: '<S41>/MATLAB Function2'
void MatlabControllerClass::xzlarf(int32_T m, int32_T n, int32_T iv0, real32_T
  tau, real32_T C_data[], int32_T ic0, real32_T work_data[])
{
  int32_T lastv;
  int32_T lastc;
  int32_T coltop;
  int32_T ix;
  real32_T c;
  int32_T iac;
  int32_T d;
  int32_T b_ia;
  int32_T jy;
  int32_T exitg1;
  boolean_T exitg2;
  if (tau != 0.0F) {
    lastv = m;
    lastc = iv0 + m;
    while ((lastv > 0) && (C_data[lastc - 2] == 0.0F)) {
      lastv--;
      lastc--;
    }

    lastc = n - 1;
    exitg2 = false;
    while ((!exitg2) && (lastc + 1 > 0)) {
      coltop = (lastc << 3) + ic0;
      jy = coltop;
      do {
        exitg1 = 0;
        if (jy <= (coltop + lastv) - 1) {
          if (C_data[jy - 1] != 0.0F) {
            exitg1 = 1;
          } else {
            jy++;
          }
        } else {
          lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    lastv = 0;
    lastc = -1;
  }

  if (lastv > 0) {
    if (lastc + 1 != 0) {
      for (coltop = 0; coltop <= lastc; coltop++) {
        work_data[coltop] = 0.0F;
      }

      coltop = 0;
      jy = (lastc << 3) + ic0;
      for (iac = ic0; iac <= jy; iac += 8) {
        ix = iv0;
        c = 0.0F;
        d = (iac + lastv) - 1;
        for (b_ia = iac; b_ia <= d; b_ia++) {
          c += C_data[b_ia - 1] * C_data[ix - 1];
          ix++;
        }

        work_data[coltop] += c;
        coltop++;
      }
    }

    if (-tau != 0.0F) {
      coltop = ic0 - 1;
      jy = 0;
      for (iac = 0; iac <= lastc; iac++) {
        if (work_data[jy] != 0.0F) {
          c = work_data[jy] * -tau;
          ix = iv0;
          d = lastv + coltop;
          for (b_ia = coltop; b_ia < d; b_ia++) {
            C_data[b_ia] += C_data[ix - 1] * c;
            ix++;
          }
        }

        jy++;
        coltop += 8;
      }
    }
  }
}

// Function for MATLAB Function: '<S41>/MATLAB Function2'
void MatlabControllerClass::qrsolve(const real32_T A_data[], const int32_T
  A_size[2], const real32_T B_6[8], real32_T Y_data[], int32_T *Y_size)
{
  real32_T b_A_data[32];
  real32_T tau_data[4];
  int32_T jpvt_data[4];
  int32_T n;
  real32_T work_data[4];
  real32_T vn1_data[4];
  real32_T vn2_data[4];
  int32_T nmi;
  int32_T b_n;
  int32_T yk;
  int32_T idxmax;
  int32_T ix;
  real32_T smax;
  real32_T s;
  int32_T b_ix;
  int32_T iy;
  real32_T absxk;
  real32_T t;
  real32_T B_7[8];
  int32_T b_A_size[2];
  int8_T c_idx_0;
  b_A_size[0] = 8;
  b_A_size[1] = A_size[1];
  b_n = A_size[0] * A_size[1] - 1;
  if (0 <= b_n) {
    memcpy(&b_A_data[0], &A_data[0], (b_n + 1) * sizeof(real32_T));
  }

  n = A_size[1];
  if (A_size[1] < 1) {
    b_n = 0;
  } else {
    b_n = A_size[1];
  }

  if (b_n > 0) {
    jpvt_data[0] = 1;
    yk = 1;
    for (nmi = 2; nmi <= b_n; nmi++) {
      yk++;
      jpvt_data[nmi - 1] = yk;
    }
  }

  if (A_size[1] != 0) {
    c_idx_0 = (int8_T)A_size[1];
    if (0 <= c_idx_0 - 1) {
      memset(&work_data[0], 0, c_idx_0 * sizeof(real32_T));
    }

    b_n = 1;
    for (yk = 0; yk < n; yk++) {
      smax = 0.0F;
      s = 1.29246971E-26F;
      for (nmi = b_n; nmi <= b_n + 7; nmi++) {
        absxk = std::abs(A_data[nmi - 1]);
        if (absxk > s) {
          t = s / absxk;
          smax = smax * t * t + 1.0F;
          s = absxk;
        } else {
          t = absxk / s;
          smax += t * t;
        }
      }

      vn1_data[yk] = s * std::sqrt(smax);
      vn2_data[yk] = vn1_data[yk];
      b_n += 8;
    }

    for (b_n = 0; b_n < n; b_n++) {
      iy = b_n << 3;
      yk = iy + b_n;
      nmi = n - b_n;
      if (nmi < 1) {
        idxmax = 0;
      } else {
        idxmax = 1;
        if (nmi > 1) {
          ix = b_n;
          smax = std::abs(vn1_data[b_n]);
          for (b_ix = 2; b_ix <= nmi; b_ix++) {
            ix++;
            s = std::abs(vn1_data[ix]);
            if (s > smax) {
              idxmax = b_ix;
              smax = s;
            }
          }
        }
      }

      ix = (b_n + idxmax) - 1;
      if (ix + 1 != b_n + 1) {
        b_ix = ix << 3;
        for (idxmax = 0; idxmax < 8; idxmax++) {
          smax = b_A_data[b_ix];
          b_A_data[b_ix] = b_A_data[iy];
          b_A_data[iy] = smax;
          b_ix++;
          iy++;
        }

        b_ix = jpvt_data[ix];
        jpvt_data[ix] = jpvt_data[b_n];
        jpvt_data[b_n] = b_ix;
        vn1_data[ix] = vn1_data[b_n];
        vn2_data[ix] = vn2_data[b_n];
      }

      smax = b_A_data[yk];
      tau_data[b_n] = 0.0F;
      s = xnrm2_dg(7 - b_n, b_A_data, yk + 2);
      if (s != 0.0F) {
        s = rt_hypotf(b_A_data[yk], s);
        if (b_A_data[yk] >= 0.0F) {
          s = -s;
        }

        if (std::abs(s) < 9.86076132E-32F) {
          ix = -1;
          b_ix = (yk - b_n) + 8;
          do {
            ix++;
            for (iy = yk + 1; iy < b_ix; iy++) {
              b_A_data[iy] *= 1.01412048E+31F;
            }

            s *= 1.01412048E+31F;
            smax *= 1.01412048E+31F;
          } while (std::abs(s) < 9.86076132E-32F);

          s = rt_hypotf(smax, xnrm2_dg(7 - b_n, b_A_data, yk + 2));
          if (smax >= 0.0F) {
            s = -s;
          }

          tau_data[b_n] = (s - smax) / s;
          smax = 1.0F / (smax - s);
          b_ix = (yk - b_n) + 8;
          for (iy = yk + 1; iy < b_ix; iy++) {
            b_A_data[iy] *= smax;
          }

          for (idxmax = 0; idxmax <= ix; idxmax++) {
            s *= 9.86076132E-32F;
          }

          smax = s;
        } else {
          tau_data[b_n] = (s - b_A_data[yk]) / s;
          smax = 1.0F / (b_A_data[yk] - s);
          ix = (yk - b_n) + 8;
          for (b_ix = yk + 1; b_ix < ix; b_ix++) {
            b_A_data[b_ix] *= smax;
          }

          smax = s;
        }
      }

      b_A_data[yk] = smax;
      if (b_n + 1 < n) {
        smax = b_A_data[yk];
        b_A_data[yk] = 1.0F;
        xzlarf(8 - b_n, nmi - 1, yk + 1, tau_data[b_n], b_A_data, (b_n + ((b_n +
                  1) << 3)) + 1, work_data);
        b_A_data[yk] = smax;
      }

      for (yk = b_n + 1; yk < n; yk++) {
        if (vn1_data[yk] != 0.0F) {
          nmi = (yk << 3) + b_n;
          smax = std::abs(b_A_data[nmi]) / vn1_data[yk];
          smax = 1.0F - smax * smax;
          if (smax < 0.0F) {
            smax = 0.0F;
          }

          s = vn1_data[yk] / vn2_data[yk];
          s = s * s * smax;
          if (s <= 0.000345266977F) {
            vn1_data[yk] = xnrm2_dg(7 - b_n, b_A_data, nmi + 2);
            vn2_data[yk] = vn1_data[yk];
          } else {
            vn1_data[yk] *= std::sqrt(smax);
          }
        }
      }
    }
  }

  n = 0;
  if (b_A_size[1] > 0) {
    while ((n < b_A_size[1]) && (std::abs(b_A_data[(n << 3) + n]) >
            9.53674316E-6F * std::abs(b_A_data[0]))) {
      n++;
    }
  }

  for (b_ix = 0; b_ix < 8; b_ix++) {
    B_7[b_ix] = B_6[b_ix];
  }

  LSQFromQR(b_A_data, b_A_size, tau_data, jpvt_data, B_7, n, Y_data, Y_size);
}

// Function for MATLAB Function: '<S41>/MATLAB Function2'
void MatlabControllerClass::mldivide_b(const real32_T A_data[], const int32_T
  A_size[2], const real32_T B_5[8], real32_T Y_data[], int32_T *Y_size)
{
  if (A_size[1] == 0) {
    *Y_size = 0;
  } else {
    qrsolve(A_data, A_size, B_5, Y_data, Y_size);
  }
}

// Function for MATLAB Function: '<S41>/MATLAB Function2'
boolean_T MatlabControllerClass::any(const boolean_T x_data[], const int32_T
  *x_size)
{
  boolean_T y;
  int32_T ix;
  boolean_T exitg1;
  y = false;
  ix = 1;
  exitg1 = false;
  while ((!exitg1) && (ix <= *x_size)) {
    if (!x_data[ix - 1]) {
      ix++;
    } else {
      y = true;
      exitg1 = true;
    }
  }

  return y;
}

// Function for MATLAB Function: '<S41>/MATLAB Function2'
real_T MatlabControllerClass::wls_alloc(const real32_T B_9[16], const real32_T
  v[4], const real32_T umin[4], const real32_T umax[4], const real32_T Wv[16],
  const real_T Wu[16], const real32_T ud[4], real32_T gam, real32_T u[4], real_T
  W[4], real_T imax)
{
  real_T iter;
  real32_T gam_sq;
  real32_T A[32];
  real32_T d[8];
  boolean_T i_free[4];
  real32_T A_free_data[32];
  real32_T p_free_data[4];
  real_T p[4];
  real32_T u_opt[4];
  real_T b_data[4];
  int8_T e_data[4];
  int8_T f_data[4];
  int8_T g_data[4];
  int8_T h_data[4];
  int32_T aoffset;
  int32_T b_k;
  int32_T b_aoffset;
  boolean_T x[4];
  real32_T A_tmp[16];
  boolean_T u_opt_data[4];
  real32_T A_tmp_0[16];
  real32_T A_tmp_1[8];
  boolean_T u_opt_0[4];
  int32_T A_free_size[2];
  real_T p_0;
  boolean_T x_0;
  real_T dist_idx_0;
  boolean_T c_idx_0;
  real_T dist_idx_1;
  boolean_T c_idx_1;
  real_T dist_idx_2;
  boolean_T c_idx_2;
  real_T dist_idx_3;
  boolean_T c_idx_3;
  real32_T Wu_0;
  int32_T A_tmp_tmp;
  int32_T A_tmp_tmp_0;
  int32_T A_tmp_tmp_1;
  boolean_T exitg1;
  boolean_T exitg2;
  gam_sq = std::sqrt(gam);
  for (b_k = 0; b_k < 16; b_k++) {
    A_tmp[b_k] = gam_sq * Wv[b_k];
  }

  for (b_k = 0; b_k < 4; b_k++) {
    for (aoffset = 0; aoffset < 4; aoffset++) {
      A_tmp_tmp = b_k << 2;
      A_tmp_tmp_0 = aoffset + A_tmp_tmp;
      A_tmp_0[A_tmp_tmp_0] = 0.0F;
      A_tmp_tmp_1 = A_tmp_tmp + aoffset;
      A_tmp_0[A_tmp_tmp_0] = A_tmp_0[A_tmp_tmp_1] + B_9[A_tmp_tmp] *
        A_tmp[aoffset];
      A_tmp_0[A_tmp_tmp_0] = B_9[A_tmp_tmp + 1] * A_tmp[aoffset + 4] +
        A_tmp_0[A_tmp_tmp_1];
      A_tmp_0[A_tmp_tmp_0] = B_9[A_tmp_tmp + 2] * A_tmp[aoffset + 8] +
        A_tmp_0[A_tmp_tmp_1];
      A_tmp_0[A_tmp_tmp_0] = B_9[A_tmp_tmp + 3] * A_tmp[aoffset + 12] +
        A_tmp_0[A_tmp_tmp_1];
    }
  }

  for (b_k = 0; b_k < 4; b_k++) {
    A_tmp_tmp = b_k << 2;
    A_tmp_tmp_0 = b_k << 3;
    A[A_tmp_tmp_0] = A_tmp_0[A_tmp_tmp];
    A[4 + A_tmp_tmp_0] = (real32_T)Wu[A_tmp_tmp];
    A_tmp_tmp_1 = A_tmp_tmp + 1;
    A[1 + A_tmp_tmp_0] = A_tmp_0[A_tmp_tmp_1];
    A[5 + A_tmp_tmp_0] = (real32_T)Wu[A_tmp_tmp_1];
    A_tmp_tmp_1 = A_tmp_tmp + 2;
    A[2 + A_tmp_tmp_0] = A_tmp_0[A_tmp_tmp_1];
    A[6 + A_tmp_tmp_0] = (real32_T)Wu[A_tmp_tmp_1];
    A_tmp_tmp += 3;
    A[3 + A_tmp_tmp_0] = A_tmp_0[A_tmp_tmp];
    A[7 + A_tmp_tmp_0] = (real32_T)Wu[A_tmp_tmp];
    gam_sq = A_tmp[b_k + 12] * v[3] + (A_tmp[b_k + 8] * v[2] + (A_tmp[b_k + 4] *
      v[1] + A_tmp[b_k] * v[0]));
    Wu_0 = (real32_T)Wu[b_k + 12] * ud[3] + ((real32_T)Wu[b_k + 8] * ud[2] +
      ((real32_T)Wu[b_k + 4] * ud[1] + (real32_T)Wu[b_k] * ud[0]));
    A_tmp_1[b_k] = gam_sq;
    A_tmp_1[b_k + 4] = Wu_0;
  }

  for (b_k = 0; b_k < 8; b_k++) {
    gam_sq = A[b_k + 24] * u[3] + (A[b_k + 16] * u[2] + (A[b_k + 8] * u[1] +
      A[b_k] * u[0]));
    d[b_k] = A_tmp_1[b_k] - gam_sq;
  }

  i_free[0] = (W[0] == 0.0);
  i_free[1] = (W[1] == 0.0);
  i_free[2] = (W[2] == 0.0);
  i_free[3] = (W[3] == 0.0);
  iter = 1.0;
  A_tmp_tmp = 0;
  exitg1 = false;
  while ((!exitg1) && (A_tmp_tmp <= (int32_T)imax - 1)) {
    iter = 1.0 + (real_T)A_tmp_tmp;
    A_tmp_tmp_1 = 0;
    if (i_free[0]) {
      A_tmp_tmp_1 = 1;
    }

    if (i_free[1]) {
      A_tmp_tmp_1++;
    }

    if (i_free[2]) {
      A_tmp_tmp_1++;
    }

    if (i_free[3]) {
      A_tmp_tmp_1++;
    }

    A_tmp_tmp_0 = A_tmp_tmp_1;
    A_tmp_tmp_1 = 0;
    if (i_free[0]) {
      e_data[0] = 1;
      A_tmp_tmp_1 = 1;
    }

    if (i_free[1]) {
      e_data[A_tmp_tmp_1] = 2;
      A_tmp_tmp_1++;
    }

    if (i_free[2]) {
      e_data[A_tmp_tmp_1] = 3;
      A_tmp_tmp_1++;
    }

    if (i_free[3]) {
      e_data[A_tmp_tmp_1] = 4;
    }

    A_free_size[0] = 8;
    A_free_size[1] = A_tmp_tmp_0;
    for (b_k = 0; b_k < A_tmp_tmp_0; b_k++) {
      for (aoffset = 0; aoffset < 8; aoffset++) {
        A_free_data[aoffset + (b_k << 3)] = A[((e_data[b_k] - 1) << 3) + aoffset];
      }
    }

    mldivide_b(A_free_data, A_free_size, d, p_free_data, &b_aoffset);
    A_tmp_tmp_1 = 0;
    p_0 = 0.0;
    if (i_free[0]) {
      p_0 = p_free_data[0];
      A_tmp_tmp_1 = 1;
    }

    u_opt[0] = u[0] + (real32_T)p_0;
    p[0] = p_0;
    p_0 = 0.0;
    if (i_free[1]) {
      p_0 = p_free_data[A_tmp_tmp_1];
      A_tmp_tmp_1++;
    }

    u_opt[1] = u[1] + (real32_T)p_0;
    p[1] = p_0;
    p_0 = 0.0;
    if (i_free[2]) {
      p_0 = p_free_data[A_tmp_tmp_1];
      A_tmp_tmp_1++;
    }

    u_opt[2] = u[2] + (real32_T)p_0;
    p[2] = p_0;
    p_0 = 0.0;
    if (i_free[3]) {
      p_0 = p_free_data[A_tmp_tmp_1];
    }

    u_opt[3] = u[3] + (real32_T)p_0;
    p[3] = p_0;
    A_tmp_tmp_1 = 0;
    if (i_free[0]) {
      A_tmp_tmp_1 = 1;
    }

    if (i_free[1]) {
      A_tmp_tmp_1++;
    }

    if (i_free[2]) {
      A_tmp_tmp_1++;
    }

    if (i_free[3]) {
      A_tmp_tmp_1++;
    }

    aoffset = A_tmp_tmp_1;
    A_tmp_tmp_1 = 0;
    if (i_free[0]) {
      f_data[0] = 1;
      A_tmp_tmp_1 = 1;
    }

    u_opt_0[0] = ((u_opt[0] < umin[0]) || (u_opt[0] > umax[0]));
    if (i_free[1]) {
      f_data[A_tmp_tmp_1] = 2;
      A_tmp_tmp_1++;
    }

    u_opt_0[1] = ((u_opt[1] < umin[1]) || (u_opt[1] > umax[1]));
    if (i_free[2]) {
      f_data[A_tmp_tmp_1] = 3;
      A_tmp_tmp_1++;
    }

    u_opt_0[2] = ((u_opt[2] < umin[2]) || (u_opt[2] > umax[2]));
    if (i_free[3]) {
      f_data[A_tmp_tmp_1] = 4;
    }

    u_opt_0[3] = ((u_opt[3] < umin[3]) || (u_opt[3] > umax[3]));
    for (b_k = 0; b_k < aoffset; b_k++) {
      u_opt_data[b_k] = u_opt_0[f_data[b_k] - 1];
    }

    if (!any(u_opt_data, &aoffset)) {
      u[0] = u_opt[0];
      u[1] = u_opt[1];
      u[2] = u_opt[2];
      u[3] = u_opt[3];
      if (A_tmp_tmp_0 == 1) {
        for (b_k = 0; b_k < 8; b_k++) {
          A_tmp_1[b_k] = 0.0F;
          for (aoffset = 0; aoffset < A_tmp_tmp_0; aoffset++) {
            A_tmp_1[b_k] += A_free_data[(aoffset << 3) + b_k] *
              p_free_data[aoffset];
          }
        }
      } else if (b_aoffset == 1) {
        for (b_k = 0; b_k < 8; b_k++) {
          A_tmp_1[b_k] = 0.0F;
          for (aoffset = 0; aoffset < A_tmp_tmp_0; aoffset++) {
            A_tmp_1[b_k] += A_free_data[(aoffset << 3) + b_k] *
              p_free_data[aoffset];
          }
        }
      } else {
        for (A_tmp_tmp_1 = 0; A_tmp_tmp_1 < 8; A_tmp_tmp_1++) {
          A_tmp_1[A_tmp_tmp_1] = 0.0F;
        }

        for (A_tmp_tmp_1 = 0; A_tmp_tmp_1 < A_tmp_tmp_0; A_tmp_tmp_1++) {
          b_aoffset = A_tmp_tmp_1 << 3;
          for (b_k = 0; b_k < 8; b_k++) {
            aoffset = b_aoffset + b_k;
            A_tmp_1[b_k] += A[((e_data[aoffset / 8] - 1) << 3) + aoffset % 8] *
              p_free_data[A_tmp_tmp_1];
          }
        }
      }

      for (b_k = 0; b_k < 8; b_k++) {
        d[b_k] -= A_tmp_1[b_k];
      }

      for (A_tmp_tmp_0 = 0; A_tmp_tmp_0 < 4; A_tmp_tmp_0++) {
        p_free_data[A_tmp_tmp_0] = 0.0F;
        for (b_k = 0; b_k < 8; b_k++) {
          p_free_data[A_tmp_tmp_0] += A[(A_tmp_tmp_0 << 3) + b_k] * d[b_k];
        }

        gam_sq = (real32_T)W[A_tmp_tmp_0] * p_free_data[A_tmp_tmp_0];
        x[A_tmp_tmp_0] = (gam_sq >= -2.22044605E-16F);
        u_opt[A_tmp_tmp_0] = gam_sq;
      }

      x_0 = true;
      A_tmp_tmp_1 = 0;
      exitg2 = false;
      while ((!exitg2) && (A_tmp_tmp_1 < 4)) {
        if (!x[A_tmp_tmp_1]) {
          x_0 = false;
          exitg2 = true;
        } else {
          A_tmp_tmp_1++;
        }
      }

      if (x_0) {
        exitg1 = true;
      } else {
        gam_sq = u_opt[0];
        A_tmp_tmp_1 = 0;
        if (u_opt[0] > u_opt[1]) {
          gam_sq = u_opt[1];
          A_tmp_tmp_1 = 1;
        }

        if (gam_sq > u_opt[2]) {
          gam_sq = u_opt[2];
          A_tmp_tmp_1 = 2;
        }

        if (gam_sq > u_opt[3]) {
          A_tmp_tmp_1 = 3;
        }

        W[A_tmp_tmp_1] = 0.0;
        i_free[A_tmp_tmp_1] = true;
        A_tmp_tmp++;
      }
    } else {
      A_tmp_tmp_1 = 0;
      dist_idx_0 = 1.0;
      x_0 = (p[0] < 0.0);
      c_idx_0 = (p[0] > 0.0);
      if (i_free[0] && x_0) {
        A_tmp_tmp_1 = 1;
      }

      x[0] = x_0;
      dist_idx_1 = 1.0;
      x_0 = (p[1] < 0.0);
      c_idx_1 = (p[1] > 0.0);
      if (i_free[1] && x_0) {
        A_tmp_tmp_1++;
      }

      x[1] = x_0;
      dist_idx_2 = 1.0;
      x_0 = (p[2] < 0.0);
      c_idx_2 = (p[2] > 0.0);
      if (i_free[2] && x_0) {
        A_tmp_tmp_1++;
      }

      x[2] = x_0;
      dist_idx_3 = 1.0;
      x_0 = (p_0 < 0.0);
      c_idx_3 = (p_0 > 0.0);
      if (i_free[3] && x_0) {
        A_tmp_tmp_1++;
      }

      aoffset = A_tmp_tmp_1;
      A_tmp_tmp_1 = 0;
      if (i_free[0] && x[0]) {
        g_data[0] = 1;
        A_tmp_tmp_1 = 1;
      }

      if (i_free[1] && x[1]) {
        g_data[A_tmp_tmp_1] = 2;
        A_tmp_tmp_1++;
      }

      if (i_free[2] && x[2]) {
        g_data[A_tmp_tmp_1] = 3;
        A_tmp_tmp_1++;
      }

      if (i_free[3] && x_0) {
        g_data[A_tmp_tmp_1] = 4;
      }

      for (b_k = 0; b_k < aoffset; b_k++) {
        A_tmp_tmp_1 = g_data[b_k] - 1;
        b_data[b_k] = (umin[A_tmp_tmp_1] - u[A_tmp_tmp_1]) / (real32_T)
          p[A_tmp_tmp_1];
      }

      A_tmp_tmp_1 = 0;
      if (i_free[0] && x[0]) {
        dist_idx_0 = b_data[0];
        A_tmp_tmp_1 = 1;
      }

      if (i_free[1] && x[1]) {
        dist_idx_1 = b_data[A_tmp_tmp_1];
        A_tmp_tmp_1++;
      }

      if (i_free[2] && x[2]) {
        dist_idx_2 = b_data[A_tmp_tmp_1];
        A_tmp_tmp_1++;
      }

      if (i_free[3] && x_0) {
        dist_idx_3 = b_data[A_tmp_tmp_1];
      }

      A_tmp_tmp_1 = 0;
      if (i_free[0] && c_idx_0) {
        A_tmp_tmp_1 = 1;
      }

      if (i_free[1] && c_idx_1) {
        A_tmp_tmp_1++;
      }

      if (i_free[2] && c_idx_2) {
        A_tmp_tmp_1++;
      }

      if (i_free[3] && c_idx_3) {
        A_tmp_tmp_1++;
      }

      aoffset = A_tmp_tmp_1;
      A_tmp_tmp_1 = 0;
      if (i_free[0] && c_idx_0) {
        h_data[0] = 1;
        A_tmp_tmp_1 = 1;
      }

      if (i_free[1] && c_idx_1) {
        h_data[A_tmp_tmp_1] = 2;
        A_tmp_tmp_1++;
      }

      if (i_free[2] && c_idx_2) {
        h_data[A_tmp_tmp_1] = 3;
        A_tmp_tmp_1++;
      }

      if (i_free[3] && c_idx_3) {
        h_data[A_tmp_tmp_1] = 4;
      }

      for (b_k = 0; b_k < aoffset; b_k++) {
        A_tmp_tmp_1 = h_data[b_k] - 1;
        b_data[b_k] = (umax[A_tmp_tmp_1] - u[A_tmp_tmp_1]) / (real32_T)
          p[A_tmp_tmp_1];
      }

      A_tmp_tmp_1 = 0;
      if (i_free[0] && c_idx_0) {
        dist_idx_0 = b_data[0];
        A_tmp_tmp_1 = 1;
      }

      if (i_free[1] && c_idx_1) {
        dist_idx_1 = b_data[A_tmp_tmp_1];
        A_tmp_tmp_1++;
      }

      if (i_free[2] && c_idx_2) {
        dist_idx_2 = b_data[A_tmp_tmp_1];
        A_tmp_tmp_1++;
      }

      if (i_free[3] && c_idx_3) {
        dist_idx_3 = b_data[A_tmp_tmp_1];
      }

      A_tmp_tmp_1 = 0;
      if (dist_idx_0 > dist_idx_1) {
        dist_idx_0 = dist_idx_1;
        A_tmp_tmp_1 = 1;
      }

      if (dist_idx_0 > dist_idx_2) {
        dist_idx_0 = dist_idx_2;
        A_tmp_tmp_1 = 2;
      }

      if (dist_idx_0 > dist_idx_3) {
        dist_idx_0 = dist_idx_3;
        A_tmp_tmp_1 = 3;
      }

      u[0] += (real32_T)(dist_idx_0 * p[0]);
      u[1] += (real32_T)(dist_idx_0 * p[1]);
      u[2] += (real32_T)(dist_idx_0 * p[2]);
      u[3] += (real32_T)(dist_idx_0 * p_0);
      aoffset = (A_tmp_tmp_0 << 3) - 1;
      for (b_k = 0; b_k <= aoffset; b_k++) {
        A_free_data[b_k] *= (real32_T)dist_idx_0;
      }

      if (A_tmp_tmp_0 == 1) {
        for (b_k = 0; b_k < 8; b_k++) {
          A_tmp_1[b_k] = 0.0F;
          for (aoffset = 0; aoffset < 1; aoffset++) {
            A_tmp_1[b_k] += A_free_data[b_k] * p_free_data[0];
          }
        }
      } else if (b_aoffset == 1) {
        for (b_k = 0; b_k < 8; b_k++) {
          A_tmp_1[b_k] = 0.0F;
          for (aoffset = 0; aoffset < A_tmp_tmp_0; aoffset++) {
            A_tmp_1[b_k] += A_free_data[(aoffset << 3) + b_k] *
              p_free_data[aoffset];
          }
        }
      } else {
        for (b_aoffset = 0; b_aoffset < 8; b_aoffset++) {
          A_tmp_1[b_aoffset] = 0.0F;
        }

        for (b_k = 0; b_k < A_tmp_tmp_0; b_k++) {
          aoffset = b_k << 3;
          for (b_aoffset = 0; b_aoffset < 8; b_aoffset++) {
            A_tmp_1[b_aoffset] += A_free_data[aoffset + b_aoffset] *
              p_free_data[b_k];
          }
        }
      }

      for (b_k = 0; b_k < 8; b_k++) {
        d[b_k] -= A_tmp_1[b_k];
      }

      if (p[A_tmp_tmp_1] < 0.0) {
        W[A_tmp_tmp_1] = -1.0;
      } else if (p[A_tmp_tmp_1] > 0.0) {
        W[A_tmp_tmp_1] = 1.0;
      } else {
        W[A_tmp_tmp_1] = p[A_tmp_tmp_1];
      }

      i_free[A_tmp_tmp_1] = false;
      A_tmp_tmp++;
    }
  }

  return iter;
}

// Function for MATLAB Function: '<S26>/flight path matching'
real32_T MatlabControllerClass::xnrm2_d(int32_T n, const real32_T x[81], int32_T
  ix0)
{
  real32_T y;
  real32_T scale;
  int32_T kend;
  real32_T absxk;
  real32_T t;
  int32_T k;
  y = 0.0F;
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x[ix0 - 1]);
    } else {
      scale = 1.29246971E-26F;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        absxk = std::abs(x[k - 1]);
        if (absxk > scale) {
          t = scale / absxk;
          y = y * t * t + 1.0F;
          scale = absxk;
        } else {
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * std::sqrt(y);
    }
  }

  return y;
}

// Function for MATLAB Function: '<S26>/flight path matching'
void MatlabControllerClass::xgehrd(real32_T a[81])
{
  real32_T tau[8];
  real32_T work[9];
  int32_T in;
  int32_T ia0;
  real32_T xnorm;
  int32_T knt;
  int32_T rowleft;
  int32_T iy;
  int32_T g;
  int32_T b_ia;
  int32_T jy;
  real32_T temp;
  int32_T b_ix;
  int32_T d_ix;
  int32_T i;
  real32_T alpha1_tmp;
  int32_T jy_tmp;
  int32_T tmp;
  int32_T exitg1;
  boolean_T exitg2;
  for (i = 0; i < 9; i++) {
    work[i] = 0.0F;
  }

  for (i = 0; i < 8; i++) {
    in = (i + 1) * 9;
    ia0 = i + 3;
    if (ia0 >= 9) {
      ia0 = 9;
    }

    ia0 += i * 9;
    alpha1_tmp = a[(9 * i + i) + 1];
    temp = 0.0F;
    xnorm = xnrm2_d(7 - i, a, ia0);
    if (xnorm != 0.0F) {
      xnorm = rt_hypotf(alpha1_tmp, xnorm);
      if (alpha1_tmp >= 0.0F) {
        xnorm = -xnorm;
      }

      if (std::abs(xnorm) < 9.86076132E-32F) {
        knt = -1;
        jy_tmp = (ia0 - i) + 6;
        do {
          knt++;
          for (rowleft = ia0; rowleft <= jy_tmp; rowleft++) {
            a[rowleft - 1] *= 1.01412048E+31F;
          }

          xnorm *= 1.01412048E+31F;
          alpha1_tmp *= 1.01412048E+31F;
        } while (std::abs(xnorm) < 9.86076132E-32F);

        xnorm = rt_hypotf(alpha1_tmp, xnrm2_d(7 - i, a, ia0));
        if (alpha1_tmp >= 0.0F) {
          xnorm = -xnorm;
        }

        temp = (xnorm - alpha1_tmp) / xnorm;
        alpha1_tmp = 1.0F / (alpha1_tmp - xnorm);
        while (ia0 <= jy_tmp) {
          a[ia0 - 1] *= alpha1_tmp;
          ia0++;
        }

        for (ia0 = 0; ia0 <= knt; ia0++) {
          xnorm *= 9.86076132E-32F;
        }

        alpha1_tmp = xnorm;
      } else {
        temp = (xnorm - alpha1_tmp) / xnorm;
        alpha1_tmp = 1.0F / (alpha1_tmp - xnorm);
        knt = (ia0 - i) + 6;
        while (ia0 <= knt) {
          a[ia0 - 1] *= alpha1_tmp;
          ia0++;
        }

        alpha1_tmp = xnorm;
      }
    }

    tau[i] = temp;
    tmp = (i + 9 * i) + 1;
    a[tmp] = 1.0F;
    jy_tmp = (i + i * 9) + 1;
    jy = jy_tmp;
    if (tau[i] != 0.0F) {
      knt = 7 - i;
      ia0 = (jy_tmp - i) + 7;
      while ((knt + 1 > 0) && (a[ia0] == 0.0F)) {
        knt--;
        ia0--;
      }

      ia0 = 9;
      exitg2 = false;
      while ((!exitg2) && (ia0 > 0)) {
        rowleft = in + ia0;
        d_ix = rowleft;
        do {
          exitg1 = 0;
          if (d_ix <= knt * 9 + rowleft) {
            if (a[d_ix - 1] != 0.0F) {
              exitg1 = 1;
            } else {
              d_ix += 9;
            }
          } else {
            ia0--;
            exitg1 = 2;
          }
        } while (exitg1 == 0);

        if (exitg1 == 1) {
          exitg2 = true;
        }
      }
    } else {
      knt = -1;
      ia0 = 0;
    }

    if (knt + 1 > 0) {
      if (ia0 != 0) {
        for (rowleft = 0; rowleft < ia0; rowleft++) {
          work[rowleft] = 0.0F;
        }

        rowleft = jy_tmp;
        d_ix = (9 * knt + in) + 1;
        for (b_ix = in + 1; b_ix <= d_ix; b_ix += 9) {
          iy = 0;
          g = (b_ix + ia0) - 1;
          for (b_ia = b_ix; b_ia <= g; b_ia++) {
            work[iy] += a[b_ia - 1] * a[rowleft];
            iy++;
          }

          rowleft++;
        }
      }

      if (-tau[i] != 0.0F) {
        rowleft = in;
        for (d_ix = 0; d_ix <= knt; d_ix++) {
          if (a[jy] != 0.0F) {
            temp = a[jy] * -tau[i];
            b_ix = 0;
            iy = ia0 + rowleft;
            for (g = rowleft; g < iy; g++) {
              a[g] += work[b_ix] * temp;
              b_ix++;
            }
          }

          jy++;
          rowleft += 9;
        }
      }
    }

    ia0 = (i + in) + 2;
    if (tau[i] != 0.0F) {
      in = 8 - i;
      knt = (jy_tmp - i) + 7;
      while ((in > 0) && (a[knt] == 0.0F)) {
        in--;
        knt--;
      }

      knt = 7 - i;
      exitg2 = false;
      while ((!exitg2) && (knt + 1 > 0)) {
        jy = knt * 9 + ia0;
        rowleft = jy;
        do {
          exitg1 = 0;
          if (rowleft <= (jy + in) - 1) {
            if (a[rowleft - 1] != 0.0F) {
              exitg1 = 1;
            } else {
              rowleft++;
            }
          } else {
            knt--;
            exitg1 = 2;
          }
        } while (exitg1 == 0);

        if (exitg1 == 1) {
          exitg2 = true;
        }
      }
    } else {
      in = 0;
      knt = -1;
    }

    if (in > 0) {
      if (knt + 1 != 0) {
        for (jy = 0; jy <= knt; jy++) {
          work[jy] = 0.0F;
        }

        jy = 0;
        rowleft = 9 * knt + ia0;
        for (d_ix = ia0; d_ix <= rowleft; d_ix += 9) {
          b_ix = jy_tmp;
          temp = 0.0F;
          iy = (d_ix + in) - 1;
          for (g = d_ix; g <= iy; g++) {
            temp += a[g - 1] * a[b_ix];
            b_ix++;
          }

          work[jy] += temp;
          jy++;
        }
      }

      if (-tau[i] != 0.0F) {
        ia0--;
        jy = 0;
        for (rowleft = 0; rowleft <= knt; rowleft++) {
          if (work[jy] != 0.0F) {
            temp = work[jy] * -tau[i];
            d_ix = jy_tmp;
            b_ix = in + ia0;
            for (iy = ia0; iy < b_ix; iy++) {
              a[iy] += a[d_ix] * temp;
              d_ix++;
            }
          }

          jy++;
          ia0 += 9;
        }
      }
    }

    a[tmp] = alpha1_tmp;
  }
}

// Function for MATLAB Function: '<S26>/flight path matching'
real32_T MatlabControllerClass::xnrm2_dl(int32_T n, const real32_T x[3])
{
  real32_T y;
  real32_T scale;
  real32_T absxk;
  real32_T t;
  y = 0.0F;
  if (n >= 1) {
    if (n == 1) {
      y = std::abs(x[1]);
    } else {
      scale = 1.29246971E-26F;
      t = std::abs(x[1]);
      if (t > 1.29246971E-26F) {
        y = 1.0F;
        scale = t;
      } else {
        t /= 1.29246971E-26F;
        y = t * t;
      }

      absxk = std::abs(x[2]);
      if (absxk > scale) {
        t = scale / absxk;
        y = y * t * t + 1.0F;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }

      y = scale * std::sqrt(y);
    }
  }

  return y;
}

// Function for MATLAB Function: '<S26>/flight path matching'
real32_T MatlabControllerClass::xzlarfg(int32_T n, real32_T *alpha1, real32_T x
  [3])
{
  real32_T tau;
  real32_T xnorm;
  int32_T knt;
  int32_T c_k;
  tau = 0.0F;
  if (n > 0) {
    xnorm = xnrm2_dl(n - 1, x);
    if (xnorm != 0.0F) {
      xnorm = rt_hypotf(*alpha1, xnorm);
      if (*alpha1 >= 0.0F) {
        xnorm = -xnorm;
      }

      if (std::abs(xnorm) < 9.86076132E-32F) {
        knt = -1;
        do {
          knt++;
          for (c_k = 1; c_k < n; c_k++) {
            x[c_k] *= 1.01412048E+31F;
          }

          xnorm *= 1.01412048E+31F;
          *alpha1 *= 1.01412048E+31F;
        } while (std::abs(xnorm) < 9.86076132E-32F);

        xnorm = rt_hypotf(*alpha1, xnrm2_dl(n - 1, x));
        if (*alpha1 >= 0.0F) {
          xnorm = -xnorm;
        }

        tau = (xnorm - *alpha1) / xnorm;
        *alpha1 = 1.0F / (*alpha1 - xnorm);
        for (c_k = 1; c_k < n; c_k++) {
          x[c_k] *= *alpha1;
        }

        for (c_k = 0; c_k <= knt; c_k++) {
          xnorm *= 9.86076132E-32F;
        }

        *alpha1 = xnorm;
      } else {
        tau = (xnorm - *alpha1) / xnorm;
        *alpha1 = 1.0F / (*alpha1 - xnorm);
        for (knt = 1; knt < n; knt++) {
          x[knt] *= *alpha1;
        }

        *alpha1 = xnorm;
      }
    }
  }

  return tau;
}

// Function for MATLAB Function: '<S26>/flight path matching'
void MatlabControllerClass::xdlanv2(real32_T *a, real32_T *b, real32_T *c,
  real32_T *d, real32_T *rt1r, real32_T *rt1i, real32_T *rt2r, real32_T *rt2i,
  real32_T *cs, real32_T *sn)
{
  real32_T temp;
  real32_T p;
  real32_T bcmax;
  real32_T bcmis;
  real32_T scale;
  real32_T z;
  int32_T b_0;
  int32_T c_0;
  if (*c == 0.0F) {
    *cs = 1.0F;
    *sn = 0.0F;
  } else if (*b == 0.0F) {
    *cs = 0.0F;
    *sn = 1.0F;
    temp = *d;
    *d = *a;
    *a = temp;
    *b = -*c;
    *c = 0.0F;
  } else {
    temp = *a - *d;
    if ((temp == 0.0F) && ((*b < 0.0F) != (*c < 0.0F))) {
      *cs = 1.0F;
      *sn = 0.0F;
    } else {
      p = 0.5F * temp;
      bcmis = std::abs(*b);
      z = std::abs(*c);
      if (bcmis > z) {
        bcmax = bcmis;
      } else {
        bcmax = z;
      }

      if (bcmis < z) {
        z = bcmis;
      }

      if (*b >= 0.0F) {
        b_0 = 1;
      } else {
        b_0 = -1;
      }

      if (*c >= 0.0F) {
        c_0 = 1;
      } else {
        c_0 = -1;
      }

      bcmis = z * (real32_T)b_0 * (real32_T)c_0;
      scale = std::abs(p);
      if (scale <= bcmax) {
        scale = bcmax;
      }

      z = p / scale * p + bcmax / scale * bcmis;
      if (z >= 8.8817842E-16F) {
        if (p >= 0.0F) {
          z = std::sqrt(scale) * std::sqrt(z);
        } else {
          z = -(std::sqrt(scale) * std::sqrt(z));
        }

        z += p;
        *a = *d + z;
        *d -= bcmax / z * bcmis;
        bcmax = rt_hypotf(*c, z);
        *cs = z / bcmax;
        *sn = *c / bcmax;
        *b -= *c;
        *c = 0.0F;
      } else {
        bcmis = *b + *c;
        bcmax = rt_hypotf(bcmis, temp);
        *cs = std::sqrt((std::abs(bcmis) / bcmax + 1.0F) * 0.5F);
        if (bcmis >= 0.0F) {
          b_0 = 1;
        } else {
          b_0 = -1;
        }

        *sn = -(p / (bcmax * *cs)) * (real32_T)b_0;
        temp = *a * *cs + *b * *sn;
        p = -*a * *sn + *b * *cs;
        bcmax = *c * *cs + *d * *sn;
        bcmis = -*c * *sn + *d * *cs;
        *b = p * *cs + bcmis * *sn;
        *c = -temp * *sn + bcmax * *cs;
        temp = ((temp * *cs + bcmax * *sn) + (-p * *sn + bcmis * *cs)) * 0.5F;
        *a = temp;
        *d = temp;
        if (*c != 0.0F) {
          if (*b != 0.0F) {
            if ((*b < 0.0F) == (*c < 0.0F)) {
              z = std::sqrt(std::abs(*b));
              bcmis = std::sqrt(std::abs(*c));
              p = z * bcmis;
              if (*c < 0.0F) {
                p = -p;
              }

              bcmax = 1.0F / std::sqrt(std::abs(*b + *c));
              *a = temp + p;
              *d = temp - p;
              *b -= *c;
              *c = 0.0F;
              p = z * bcmax;
              bcmax *= bcmis;
              temp = *cs * p - *sn * bcmax;
              *sn = *cs * bcmax + *sn * p;
              *cs = temp;
            }
          } else {
            *b = -*c;
            *c = 0.0F;
            temp = *cs;
            *cs = -*sn;
            *sn = temp;
          }
        }
      }
    }
  }

  *rt1r = *a;
  *rt2r = *d;
  if (*c == 0.0F) {
    *rt1i = 0.0F;
    *rt2i = 0.0F;
  } else {
    *rt1i = std::sqrt(std::abs(*b)) * std::sqrt(std::abs(*c));
    *rt2i = -*rt1i;
  }
}

// Function for MATLAB Function: '<S26>/flight path matching'
int32_T MatlabControllerClass::eml_dlahqr(real32_T h[81])
{
  int32_T info;
  real32_T v[3];
  int32_T i;
  int32_T L;
  boolean_T goto150;
  int32_T k;
  real32_T tst;
  real32_T htmp1;
  real32_T ab;
  real32_T ba;
  real32_T aa;
  real32_T h12;
  int32_T m;
  int32_T nr;
  int32_T hoffset;
  real32_T unusedU2;
  real32_T unusedU3;
  real32_T b_v[3];
  int32_T ix;
  int32_T tst_tmp;
  real32_T tst_tmp_0;
  int32_T h12_tmp;
  real32_T tst_tmp_tmp;
  int32_T h12_tmp_0;
  int32_T h12_tmp_1;
  boolean_T exitg1;
  boolean_T exitg2;
  boolean_T exitg3;
  info = 0;
  v[0] = 0.0F;
  v[1] = 0.0F;
  v[2] = 0.0F;
  for (i = 0; i < 6; i++) {
    hoffset = i + 9 * i;
    h[hoffset + 2] = 0.0F;
    h[hoffset + 3] = 0.0F;
  }

  h[62] = 0.0F;
  i = 8;
  exitg1 = false;
  while ((!exitg1) && (i + 1 >= 1)) {
    L = 1;
    goto150 = false;
    ix = 0;
    exitg2 = false;
    while ((!exitg2) && (ix < 301)) {
      k = i;
      exitg3 = false;
      while ((!exitg3) && (k + 1 > L)) {
        hoffset = (k - 1) * 9 + k;
        ba = std::abs(h[hoffset]);
        if (ba <= 8.87468518E-31F) {
          exitg3 = true;
        } else {
          tst_tmp = 9 * k + k;
          htmp1 = std::abs(h[tst_tmp]);
          tst_tmp_0 = h[hoffset - 1];
          tst = std::abs(tst_tmp_0) + htmp1;
          if (tst == 0.0F) {
            if (k - 1 >= 1) {
              tst = std::abs(h[((k - 2) * 9 + k) - 1]);
            }

            if (k + 2 <= 9) {
              tst += std::abs(h[tst_tmp + 1]);
            }
          }

          if (ba <= 1.1920929E-7F * tst) {
            tst = std::abs(h[tst_tmp - 1]);
            if (ba > tst) {
              ab = ba;
              ba = tst;
            } else {
              ab = tst;
            }

            tst = std::abs(tst_tmp_0 - h[9 * k + k]);
            if (htmp1 > tst) {
              aa = htmp1;
              htmp1 = tst;
            } else {
              aa = tst;
            }

            tst = aa + ab;
            htmp1 = aa / tst * htmp1 * 1.1920929E-7F;
            if (8.87468518E-31F > htmp1) {
              htmp1 = 8.87468518E-31F;
            }

            if (ab / tst * ba <= htmp1) {
              exitg3 = true;
            } else {
              k--;
            }
          } else {
            k--;
          }
        }
      }

      L = k + 1;
      if (k + 1 > 1) {
        h[k + 9 * (k - 1)] = 0.0F;
      }

      if (k + 1 >= i) {
        goto150 = true;
        exitg2 = true;
      } else {
        switch (ix) {
         case 10:
          tst = std::abs(h[((k + 1) * 9 + k) + 2]) + std::abs(h[(9 * k + k) + 1]);
          htmp1 = h[9 * k + k] + 0.75F * tst;
          h12 = -0.4375F * tst;
          aa = tst;
          ba = htmp1;
          break;

         case 20:
          tst = std::abs(h[((i - 2) * 9 + i) - 1]) + std::abs(h[(i - 1) * 9 + i]);
          htmp1 = h[9 * i + i] + 0.75F * tst;
          h12 = -0.4375F * tst;
          aa = tst;
          ba = htmp1;
          break;

         default:
          m = (i - 1) * 9 + i;
          htmp1 = h[m - 1];
          aa = h[m];
          h12_tmp = 9 * i + i;
          h12 = h[h12_tmp - 1];
          ba = h[h12_tmp];
          break;
        }

        tst = ((std::abs(htmp1) + std::abs(h12)) + std::abs(aa)) + std::abs(ba);
        if (tst == 0.0F) {
          htmp1 = 0.0F;
          ba = 0.0F;
          ab = 0.0F;
          aa = 0.0F;
        } else {
          htmp1 /= tst;
          aa /= tst;
          h12 /= tst;
          ba /= tst;
          ab = (htmp1 + ba) / 2.0F;
          htmp1 = (htmp1 - ab) * (ba - ab) - h12 * aa;
          aa = std::sqrt(std::abs(htmp1));
          if (htmp1 >= 0.0F) {
            htmp1 = ab * tst;
            ab = htmp1;
            ba = aa * tst;
            aa = -ba;
          } else {
            htmp1 = ab + aa;
            ab -= aa;
            if (std::abs(htmp1 - ba) <= std::abs(ab - ba)) {
              htmp1 *= tst;
              ab = htmp1;
            } else {
              ab *= tst;
              htmp1 = ab;
            }

            ba = 0.0F;
            aa = 0.0F;
          }
        }

        m = i - 1;
        exitg3 = false;
        while ((!exitg3) && (m >= k + 1)) {
          tst_tmp = (m - 1) * 9 + m;
          tst_tmp_tmp = h[tst_tmp - 1];
          tst_tmp_0 = tst_tmp_tmp - ab;
          tst = (std::abs(tst_tmp_0) + std::abs(aa)) + std::abs(h[tst_tmp]);
          h12 = h[(m - 1) * 9 + m] / tst;
          tst_tmp = 9 * m + m;
          v[0] = (tst_tmp_0 / tst * (tst_tmp_tmp - htmp1) + h[tst_tmp - 1] * h12)
            - aa / tst * ba;
          v[1] = (((tst_tmp_tmp + h[tst_tmp]) - htmp1) - ab) * h12;
          v[2] = h[tst_tmp + 1] * h12;
          tst = (std::abs(v[0]) + std::abs(v[1])) + std::abs(v[2]);
          tst_tmp_0 = v[0] / tst;
          v[0] = tst_tmp_0;
          h12 = v[1] / tst;
          v[1] = h12;
          tst = v[2] / tst;
          v[2] = tst;
          if (k + 1 == m) {
            exitg3 = true;
          } else {
            hoffset = (m - 2) * 9 + m;
            if (std::abs(h[hoffset - 1]) * (std::abs(h12) + std::abs(tst)) <=
                ((std::abs(h[hoffset - 2]) + std::abs(tst_tmp_tmp)) + std::abs
                 (h[9 * m + m])) * (1.1920929E-7F * std::abs(tst_tmp_0))) {
              exitg3 = true;
            } else {
              m--;
            }
          }
        }

        for (tst_tmp = m; tst_tmp <= i; tst_tmp++) {
          nr = (i - tst_tmp) + 2;
          if (3 < nr) {
            nr = 3;
          }

          if (tst_tmp > m) {
            hoffset = ((tst_tmp - 2) * 9 + tst_tmp) - 1;
            for (h12_tmp = 0; h12_tmp < nr; h12_tmp++) {
              v[h12_tmp] = h[h12_tmp + hoffset];
            }
          }

          htmp1 = v[0];
          b_v[0] = v[0];
          b_v[1] = v[1];
          b_v[2] = v[2];
          tst = xzlarfg(nr, &htmp1, b_v);
          v[1] = b_v[1];
          v[2] = b_v[2];
          v[0] = htmp1;
          if (tst_tmp > m) {
            h[(tst_tmp + 9 * (tst_tmp - 2)) - 1] = htmp1;
            h[tst_tmp + 9 * (tst_tmp - 2)] = 0.0F;
            if (tst_tmp < i) {
              h[(tst_tmp + 9 * (tst_tmp - 2)) + 1] = 0.0F;
            }
          } else {
            if (m > k + 1) {
              h[(tst_tmp + 9 * (tst_tmp - 2)) - 1] *= 1.0F - tst;
            }
          }

          htmp1 = b_v[1];
          ab = tst * b_v[1];
          switch (nr) {
           case 3:
            ba = b_v[2];
            aa = tst * b_v[2];
            for (nr = tst_tmp - 1; nr + 1 < 10; nr++) {
              h12_tmp = 9 * nr + tst_tmp;
              h12_tmp_0 = h12_tmp - 1;
              h12_tmp_1 = h12_tmp + 1;
              h12 = (h[h12_tmp_0] + h[h12_tmp] * htmp1) + h[h12_tmp_1] * ba;
              hoffset = tst_tmp + 9 * nr;
              h[hoffset - 1] = h[h12_tmp_0] - h12 * tst;
              h[hoffset] = h[h12_tmp] - h12 * ab;
              h[hoffset + 1] = h[h12_tmp_1] - h12 * aa;
            }

            hoffset = tst_tmp + 3;
            nr = i + 1;
            if (hoffset < nr) {
              nr = hoffset;
            }

            for (hoffset = 0; hoffset < nr; hoffset++) {
              tst_tmp_0 = h[(tst_tmp - 1) * 9 + hoffset];
              h12_tmp = 9 * tst_tmp + hoffset;
              h12_tmp_0 = (tst_tmp + 1) * 9 + hoffset;
              h12 = (tst_tmp_0 + h[h12_tmp] * htmp1) + h[h12_tmp_0] * ba;
              h[hoffset + 9 * (tst_tmp - 1)] = tst_tmp_0 - h12 * tst;
              h[hoffset + 9 * tst_tmp] = h[h12_tmp] - h12 * ab;
              h[hoffset + 9 * (tst_tmp + 1)] = h[h12_tmp_0] - h12 * aa;
            }
            break;

           case 2:
            for (nr = tst_tmp - 1; nr + 1 < 10; nr++) {
              h12_tmp = 9 * nr + tst_tmp;
              h12_tmp_0 = h12_tmp - 1;
              h12 = h[h12_tmp_0] + h[h12_tmp] * htmp1;
              hoffset = tst_tmp + 9 * nr;
              h[hoffset - 1] = h[h12_tmp_0] - h12 * tst;
              h[hoffset] = h[h12_tmp] - h12 * ab;
            }

            for (nr = 0; nr <= i; nr++) {
              h12_tmp = (tst_tmp - 1) * 9 + nr;
              h12_tmp_0 = 9 * tst_tmp + nr;
              h12 = h[h12_tmp] + h[h12_tmp_0] * htmp1;
              h[nr + 9 * (tst_tmp - 1)] = h[h12_tmp] - h12 * tst;
              h[nr + 9 * tst_tmp] = h[h12_tmp_0] - h12 * ab;
            }
            break;
          }
        }

        ix++;
      }
    }

    if (!goto150) {
      info = i + 1;
      exitg1 = true;
    } else {
      if ((i + 1 != L) && (L == i)) {
        ab = h[((i - 1) * 9 + i) - 1];
        ba = h[(9 * i + i) - 1];
        aa = h[(i - 1) * 9 + i];
        h12 = h[9 * i + i];
        xdlanv2(&ab, &ba, &aa, &h12, &tst_tmp_tmp, &tst_tmp_0, &unusedU2,
                &unusedU3, &tst, &htmp1);
        hoffset = i + 9 * (i - 1);
        h[hoffset - 1] = ab;
        ix = i + 9 * i;
        h[ix - 1] = ba;
        h[hoffset] = aa;
        h[ix] = h12;
        if (9 > i + 1) {
          k = (i + 1) * 9 + i;
          ix = k - 1;
          for (m = 0; m <= 7 - i; m++) {
            ab = tst * h[ix] + htmp1 * h[k];
            h[k] = tst * h[k] - htmp1 * h[ix];
            h[ix] = ab;
            k += 9;
            ix += 9;
          }
        }

        if (i - 1 >= 1) {
          ix = (i - 1) * 9;
          k = i * 9;
          for (m = 0; m <= i - 2; m++) {
            ab = tst * h[ix] + htmp1 * h[k];
            h[k] = tst * h[k] - htmp1 * h[ix];
            h[ix] = ab;
            k++;
            ix++;
          }
        }
      }

      i = L - 2;
    }
  }

  return info;
}

// Function for MATLAB Function: '<S29>/trajFromWaypoints'
void MatlabControllerClass::trajSectionGetPos(const real32_T traj_section_pos_x
  [6], const real32_T traj_section_pos_y[6], const real32_T traj_section_pos_z[6],
  real_T varargin_1, real32_T pos[3])
{
  real32_T px;
  real32_T py;
  real32_T pz;
  int32_T i;
  px = traj_section_pos_x[0];
  py = traj_section_pos_y[0];
  pz = traj_section_pos_z[0];
  for (i = 0; i < 5; i++) {
    px = (real32_T)varargin_1 * px + traj_section_pos_x[i + 1];
    py = (real32_T)varargin_1 * py + traj_section_pos_y[i + 1];
    pz = (real32_T)varargin_1 * pz + traj_section_pos_z[i + 1];
  }

  pos[0] = px;
  pos[1] = py;
  pos[2] = pz;
}

// Model step function
void MatlabControllerClass::step()
{
  real32_T d[16];
  real32_T q0_q3;
  real32_T q1_q2;
  real32_T q1_q3;
  real32_T q2_q3;
  real_T W[4];
  real32_T xf;
  real32_T yf;
  real32_T zf;
  real32_T P_0[10];
  real32_T A[81];
  real32_T T[11];
  real32_T t_values[6];
  int32_T exponent;
  int32_T b_exponent;
  int32_T istart;
  real_T b_section_idx;
  real32_T px;
  real32_T py;
  static const int8_T b[81] = { 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  real32_T dist_remaining;
  int32_T b_k;
  emxArray_real32_T *coeffs_x;
  emxArray_real32_T *coeffs_y;
  emxArray_real32_T *coeffs_z;
  real_T total_arc_length;
  real_T total_distance;
  int32_T k;
  real32_T dx_data[5];
  real32_T dy_data[5];
  real32_T dz_data[5];
  uint8_T rtb_Compare;
  real32_T rtb_uT;
  real32_T rtb_Sum2_bj[9];
  real32_T rtb_yaw;
  real32_T rtb_Diff[3];
  real32_T rtb_Delta_ny[3];
  real32_T rtb_y_f;
  real32_T rtb_y_a[4];
  boolean_T rtb_Compare_f;
  boolean_T rtb_FixPtRelationalOperator;
  boolean_T rtb_OR;
  real32_T rtb_Sqrt;
  real32_T rtb_Switch_k[3];
  real32_T rtb_n_g_des[3];
  real32_T rtb_n_g[3];
  real32_T rtb_n_b[3];
  real32_T rtb_n_b_dt[3];
  real32_T rtb_uDLookupTable2[4];
  real32_T rtb_wp_out[12];
  real32_T rtb_TSamp[3];
  real32_T rtb_G2[16];
  real32_T rtb_M_bg[9];
  real32_T rtb_TSamp_k[3];
  real32_T rtb_Diff_n[3];
  real32_T rtb_y_dt_c[3];
  real32_T rtb_n[3];
  real32_T rtb_y_jr[3];
  real32_T rtb_y_g[3];
  real32_T rtb_y_p[3];
  real32_T rtb_q_red[4];
  real32_T tmp[16];
  real32_T rtb_y_k[4];
  real32_T tmp_0[3];
  real32_T rtb_y_k_0[4];
  int32_T i;
  int32_T tmp_size[2];
  int32_T tmp_size_0[2];
  int32_T tmp_size_1[2];
  real32_T rtb_n_g_des_b;
  real32_T rtb_n_g_c;
  real32_T rtb_y_idx_3;
  real32_T rtb_y_idx_2;
  real32_T rtb_y_idx_1;
  real32_T rtb_y_idx_0;
  real32_T rtb_y_dt_idx_2;
  real32_T rtb_Switch2_idx_2;
  real32_T rtb_Switch2_idx_1;
  real32_T u;
  int32_T rtb_wp_out_tmp;
  const dtoSgl_sectionsBus *rtb_Switch_sections;
  boolean_T exitg1;
  boolean_T exitg2;

  // RelationalOperator: '<S2>/Compare' incorporates:
  //   Constant: '<S2>/Constant'
  //   Inport: '<Root>/cmd'

  rtb_Compare = (rtU.cmd.RC_pwm[6] >= 1500.0F);

  // Outputs for Enabled SubSystem: '<S3>/Flight path guidance with position controller reference' incorporates:
  //   EnablePort: '<S9>/Enable'

  if (rtb_Compare > 0) {
    // MATLAB Function: '<S23>/pad_waypoints' incorporates:
    //   Inport: '<Root>/cmd'

    for (i = 0; i < 4; i++) {
      rtb_wp_out_tmp = i << 2;
      rtb_wp_out[3 * i] = rtU.cmd.waypoints[rtb_wp_out_tmp];
      rtb_wp_out[1 + 3 * i] = rtU.cmd.waypoints[rtb_wp_out_tmp + 1];
      rtb_wp_out[2 + 3 * i] = rtU.cmd.waypoints[rtb_wp_out_tmp + 2];
    }

    // End of MATLAB Function: '<S23>/pad_waypoints'

    // RelationalOperator: '<S31>/Compare' incorporates:
    //   Constant: '<S31>/Constant'
    //   Inport: '<Root>/cmd'

    rtb_Compare_f = (rtU.cmd.mission_change > 0);

    // RelationalOperator: '<S28>/FixPt Relational Operator' incorporates:
    //   UnitDelay: '<S28>/Delay Input1'
    //
    //  Block description for '<S28>/Delay Input1':
    //
    //   Store in Global RAM

    rtb_FixPtRelationalOperator = ((int32_T)rtb_Compare_f > (int32_T)
      rtDW.DelayInput1_DSTATE);

    // Logic: '<S23>/OR' incorporates:
    //   Delay: '<S23>/Delay'

    rtb_OR = (rtDW.Delay_DSTATE_d || rtb_FixPtRelationalOperator);

    // Outputs for Enabled SubSystem: '<S23>/Subsystem2' incorporates:
    //   EnablePort: '<S29>/Enable'

    if (rtb_FixPtRelationalOperator) {
      // MATLAB Function: '<S29>/trajFromWaypoints' incorporates:
      //   Constant: '<S23>/Constant2'

      rtDW.traj = rtConstP.Constant2_Value;
      rtb_Sqrt = rtDW.traj.polynomial_degree;
      if (rtDW.traj.polynomial_degree == 0.0F) {
        rtb_uT = 0.0F;
      } else {
        rtb_uT = std::fmod(rtDW.traj.polynomial_degree, 2.0F);
        if (rtb_uT == 0.0F) {
          rtb_uT = 0.0F;
        } else {
          if (rtDW.traj.polynomial_degree < 0.0F) {
            rtb_uT += 2.0F;
          }
        }
      }

      if (rtb_uT == 0.0F) {
        rtb_Sqrt = rtDW.traj.polynomial_degree + 1.0F;
      }

      rtb_q_red[0] = rtb_wp_out[0];
      rtb_q_red[1] = rtb_wp_out[3];
      rtb_q_red[2] = rtb_wp_out[6];
      rtb_q_red[3] = rtb_wp_out[9];
      emxInit_real32_T(&coeffs_x, 2);

      // MATLAB Function: '<S29>/trajFromWaypoints' incorporates:
      //   Constant: '<S9>/Constant'

      polyInterpolation(rtb_q_red, rtb_Sqrt, true, coeffs_x, &b_section_idx);
      rtb_q_red[0] = rtb_wp_out[1];
      rtb_q_red[1] = rtb_wp_out[4];
      rtb_q_red[2] = rtb_wp_out[7];
      rtb_q_red[3] = rtb_wp_out[10];
      emxInit_real32_T(&coeffs_y, 2);

      // MATLAB Function: '<S29>/trajFromWaypoints' incorporates:
      //   Constant: '<S9>/Constant'

      polyInterpolation(rtb_q_red, rtb_Sqrt, true, coeffs_y, &total_arc_length);
      rtb_q_red[0] = rtb_wp_out[2];
      rtb_q_red[1] = rtb_wp_out[5];
      rtb_q_red[2] = rtb_wp_out[8];
      rtb_q_red[3] = rtb_wp_out[11];
      emxInit_real32_T(&coeffs_z, 2);

      // MATLAB Function: '<S29>/trajFromWaypoints' incorporates:
      //   Constant: '<S9>/Constant'

      polyInterpolation(rtb_q_red, rtb_Sqrt, true, coeffs_z, &total_arc_length);
      rtDW.traj.num_sections_set = (real32_T)b_section_idx;
      rtDW.traj.is_repeated_course = true;
      rtDW.traj.polynomial_degree = rtb_Sqrt;
      for (rtb_wp_out_tmp = 0; rtb_wp_out_tmp < (int32_T)b_section_idx;
           rtb_wp_out_tmp++) {
        xf = (real32_T)((1.0 + (real_T)rtb_wp_out_tmp) - 1.0) * (rtb_Sqrt + 1.0F);
        for (i = 0; i < 6; i++) {
          b_k = (int32_T)((xf + 1.0F) + (real32_T)i) - 1;
          rtDW.traj.sections[rtb_wp_out_tmp].pos_x[i] = coeffs_x->data[b_k];
          rtDW.traj.sections[rtb_wp_out_tmp].pos_y[i] = coeffs_y->data[b_k];
          rtDW.traj.sections[rtb_wp_out_tmp].pos_z[i] = coeffs_z->data[b_k];
        }
      }

      emxFree_real32_T(&coeffs_z);
      emxFree_real32_T(&coeffs_y);
      emxFree_real32_T(&coeffs_x);

      // MATLAB Function: '<S29>/trajFromWaypoints'
      total_arc_length = 0.0;
      total_distance = 0.0;
      for (k = 0; k < (int32_T)rtDW.traj.num_sections_set; k++) {
        b_section_idx = 1.0F + (real32_T)k;
        if (1.0F + (real32_T)k > rtDW.traj.num_sections_set) {
          b_section_idx = 1.0;
        }

        trajSectionGetPos(rtDW.traj.sections[(int32_T)b_section_idx - 1].pos_x,
                          rtDW.traj.sections[(int32_T)b_section_idx - 1].pos_y,
                          rtDW.traj.sections[(int32_T)b_section_idx - 1].pos_z,
                          1.0, rtb_n_g_des);
        trajSectionGetPos(rtDW.traj.sections[(int32_T)b_section_idx - 1].pos_x,
                          rtDW.traj.sections[(int32_T)b_section_idx - 1].pos_y,
                          rtDW.traj.sections[(int32_T)b_section_idx - 1].pos_z,
                          0.0, tmp_0);
        py = 1.29246971E-26F;
        px = std::abs(rtb_n_g_des[0] - tmp_0[0]);
        if (px > 1.29246971E-26F) {
          rtb_Sqrt = 1.0F;
          py = px;
        } else {
          rtb_yaw = px / 1.29246971E-26F;
          rtb_Sqrt = rtb_yaw * rtb_yaw;
        }

        px = std::abs(rtb_n_g_des[1] - tmp_0[1]);
        if (px > py) {
          rtb_yaw = py / px;
          rtb_Sqrt = rtb_Sqrt * rtb_yaw * rtb_yaw + 1.0F;
          py = px;
        } else {
          rtb_yaw = px / py;
          rtb_Sqrt += rtb_yaw * rtb_yaw;
        }

        px = std::abs(rtb_n_g_des[2] - tmp_0[2]);
        if (px > py) {
          rtb_yaw = py / px;
          rtb_Sqrt = rtb_Sqrt * rtb_yaw * rtb_yaw + 1.0F;
          py = px;
        } else {
          rtb_yaw = px / py;
          rtb_Sqrt += rtb_yaw * rtb_yaw;
        }

        rtb_Sqrt = py * std::sqrt(rtb_Sqrt);
        polyder_p(rtDW.traj.sections[(int32_T)b_section_idx - 1].pos_x, dx_data,
                  tmp_size);
        polyder_p(rtDW.traj.sections[(int32_T)b_section_idx - 1].pos_y, dy_data,
                  tmp_size_0);
        polyder_p(rtDW.traj.sections[(int32_T)b_section_idx - 1].pos_z, dz_data,
                  tmp_size_1);
        b_section_idx = 0.0;
        rtb_uT = polyVal(dx_data, tmp_size, 0.0);
        rtb_y_f = polyVal(dy_data, tmp_size_0, 0.0);
        rtb_yaw = polyVal(dz_data, tmp_size_1, 0.0);
        dist_remaining = polyVal(dx_data, tmp_size, 1.0);
        px = polyVal(dy_data, tmp_size_0, 1.0);
        py = polyVal(dz_data, tmp_size_1, 1.0);
        rtb_uT = std::sqrt((rtb_uT * rtb_uT + rtb_y_f * rtb_y_f) + rtb_yaw *
                           rtb_yaw) * 0.5F - std::sqrt((dist_remaining *
          dist_remaining + px * px) + py * py) * 0.5F;
        for (rtb_wp_out_tmp = 0; rtb_wp_out_tmp < 15; rtb_wp_out_tmp++) {
          b_section_idx += 0.066666666666666666;
          rtb_y_f = polyVal(dx_data, tmp_size, b_section_idx -
                            0.033333333333333333);
          rtb_yaw = polyVal(dy_data, tmp_size_0, b_section_idx -
                            0.033333333333333333);
          dist_remaining = polyVal(dz_data, tmp_size_1, b_section_idx -
            0.033333333333333333);
          px = polyVal(dx_data, tmp_size, b_section_idx);
          py = polyVal(dy_data, tmp_size_0, b_section_idx);
          xf = polyVal(dz_data, tmp_size_1, b_section_idx);
          rtb_uT = (std::sqrt((rtb_y_f * rtb_y_f + rtb_yaw * rtb_yaw) +
                              dist_remaining * dist_remaining) * 2.0F + rtb_uT)
            + std::sqrt((px * px + py * py) + xf * xf);
        }

        rtb_uT *= 0.0222222228F;
        i = (int32_T)(1.0F + (real32_T)k) - 1;
        rtDW.traj.sections[i].arc_length = rtb_uT;
        rtDW.traj.sections[i].distance = rtb_Sqrt;
        total_arc_length = (real32_T)total_arc_length + rtb_uT;
        total_distance = (real32_T)total_distance + rtb_Sqrt;
      }

      rtDW.traj.distance = (real32_T)total_distance;
      rtDW.traj.arc_length = (real32_T)total_arc_length;
    }

    // End of Outputs for SubSystem: '<S23>/Subsystem2'

    // Switch: '<S23>/Switch' incorporates:
    //   Constant: '<S23>/Constant2'

    if (rtb_OR) {
      rtb_uT = rtDW.traj.num_sections_set;
      rtb_Switch_sections = &rtDW.traj.sections[0];
      dist_remaining = rtDW.traj.arc_length;
    } else {
      rtb_uT = 0.0F;
      rtb_Switch_sections = (&rtConstP.Constant2_Value.sections[0]);
      dist_remaining = 0.0F;
    }

    // End of Switch: '<S23>/Switch'

    // Sqrt: '<S33>/Sqrt' incorporates:
    //   DotProduct: '<S33>/Dot Product'
    //   Inport: '<Root>/measure'

    rtb_Sqrt = std::sqrt((rtU.measure.V_Kg[0] * rtU.measure.V_Kg[0] +
                          rtU.measure.V_Kg[1] * rtU.measure.V_Kg[1]) +
                         rtU.measure.V_Kg[2] * rtU.measure.V_Kg[2]);

    // MATLAB Function: '<S26>/flight path matching' incorporates:
    //   Inport: '<Root>/measure'

    rtb_y_f = 1.0F;
    py = 3.402823466E+38F;
    rtb_yaw = 0.0F;
    xf = rtU.measure.s_Kg[0];
    yf = rtU.measure.s_Kg[1];
    zf = rtU.measure.s_Kg[2];
    i = (int32_T)rtb_uT - 1;
    if (0 <= i) {
      T[9] = 0.0F;
      T[10] = 1.0F;
    }

    for (k = 0; k <= i; k++) {
      P_0[0] = (rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1].pos_x[0] *
                rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1].pos_x[0] *
                5.0F + rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1].
                pos_y[0] * rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1]
                .pos_y[0] * 5.0F) + rtb_Switch_sections[(int32_T)(1.0F +
        (real32_T)k) - 1].pos_z[0] * rtb_Switch_sections[(int32_T)(1.0F +
        (real32_T)k) - 1].pos_z[0] * 5.0F;
      P_0[1] = (rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1].pos_x[0] *
                9.0F * rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1].
                pos_x[1] + rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1]
                .pos_y[0] * 9.0F * rtb_Switch_sections[(int32_T)(1.0F +
                 (real32_T)k) - 1].pos_y[1]) + rtb_Switch_sections[(int32_T)
        (1.0F + (real32_T)k) - 1].pos_z[0] * 9.0F * rtb_Switch_sections[(int32_T)
        (1.0F + (real32_T)k) - 1].pos_z[1];
      P_0[2] = ((((rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1].pos_x
                   [1] * rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1].
                   pos_x[1] * 4.0F + rtb_Switch_sections[(int32_T)(1.0F +
        (real32_T)k) - 1].pos_y[1] * rtb_Switch_sections[(int32_T)(1.0F +
        (real32_T)k) - 1].pos_y[1] * 4.0F) + rtb_Switch_sections[(int32_T)(1.0F
        + (real32_T)k) - 1].pos_z[1] * rtb_Switch_sections[(int32_T)(1.0F +
        (real32_T)k) - 1].pos_z[1] * 4.0F) + rtb_Switch_sections[(int32_T)(1.0F
                  + (real32_T)k) - 1].pos_x[0] * 8.0F * rtb_Switch_sections
                 [(int32_T)(1.0F + (real32_T)k) - 1].pos_x[2]) +
                rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1].pos_y[0] *
                8.0F * rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1].
                pos_y[2]) + rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) -
        1].pos_z[0] * 8.0F * rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) -
        1].pos_z[2];
      P_0[3] = ((((rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1].pos_x
                   [0] * 7.0F * rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k)
                   - 1].pos_x[3] + rtb_Switch_sections[(int32_T)(1.0F +
        (real32_T)k) - 1].pos_x[1] * 7.0F * rtb_Switch_sections[(int32_T)(1.0F +
        (real32_T)k) - 1].pos_x[2]) + rtb_Switch_sections[(int32_T)(1.0F +
        (real32_T)k) - 1].pos_y[0] * 7.0F * rtb_Switch_sections[(int32_T)(1.0F +
        (real32_T)k) - 1].pos_y[3]) + rtb_Switch_sections[(int32_T)(1.0F +
                  (real32_T)k) - 1].pos_y[1] * 7.0F * rtb_Switch_sections
                 [(int32_T)(1.0F + (real32_T)k) - 1].pos_y[2]) +
                rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1].pos_z[0] *
                7.0F * rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1].
                pos_z[3]) + rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) -
        1].pos_z[1] * 7.0F * rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) -
        1].pos_z[2];
      P_0[4] = (((((((rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1].
                      pos_x[2] * rtb_Switch_sections[(int32_T)(1.0F + (real32_T)
        k) - 1].pos_x[2] * 3.0F + rtb_Switch_sections[(int32_T)(1.0F + (real32_T)
        k) - 1].pos_y[2] * rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1]
                      .pos_y[2] * 3.0F) + rtb_Switch_sections[(int32_T)(1.0F +
        (real32_T)k) - 1].pos_z[2] * rtb_Switch_sections[(int32_T)(1.0F +
        (real32_T)k) - 1].pos_z[2] * 3.0F) + rtb_Switch_sections[(int32_T)(1.0F
        + (real32_T)k) - 1].pos_x[0] * 6.0F * rtb_Switch_sections[(int32_T)(1.0F
        + (real32_T)k) - 1].pos_x[4]) + rtb_Switch_sections[(int32_T)(1.0F +
        (real32_T)k) - 1].pos_x[1] * 6.0F * rtb_Switch_sections[(int32_T)(1.0F +
        (real32_T)k) - 1].pos_x[3]) + rtb_Switch_sections[(int32_T)(1.0F +
        (real32_T)k) - 1].pos_y[0] * 6.0F * rtb_Switch_sections[(int32_T)(1.0F +
        (real32_T)k) - 1].pos_y[4]) + rtb_Switch_sections[(int32_T)(1.0F +
                  (real32_T)k) - 1].pos_y[1] * 6.0F * rtb_Switch_sections
                 [(int32_T)(1.0F + (real32_T)k) - 1].pos_y[3]) +
                rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1].pos_z[0] *
                6.0F * rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1].
                pos_z[4]) + rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) -
        1].pos_z[1] * 6.0F * rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) -
        1].pos_z[3];
      P_0[5] = ((((((((((rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1].
                         pos_x[0] * 5.0F * rtb_Switch_sections[(int32_T)(1.0F +
        (real32_T)k) - 1].pos_x[5] + rtb_Switch_sections[(int32_T)(1.0F +
        (real32_T)k) - 1].pos_x[1] * 5.0F * rtb_Switch_sections[(int32_T)(1.0F +
        (real32_T)k) - 1].pos_x[4]) + rtb_Switch_sections[(int32_T)(1.0F +
        (real32_T)k) - 1].pos_x[2] * 5.0F * rtb_Switch_sections[(int32_T)(1.0F +
        (real32_T)k) - 1].pos_x[3]) - rtb_Switch_sections[(int32_T)(1.0F +
        (real32_T)k) - 1].pos_x[0] * 5.0F * xf) + rtb_Switch_sections[(int32_T)
                      (1.0F + (real32_T)k) - 1].pos_y[0] * 5.0F *
                      rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1].
                      pos_y[5]) + rtb_Switch_sections[(int32_T)(1.0F + (real32_T)
        k) - 1].pos_y[1] * 5.0F * rtb_Switch_sections[(int32_T)(1.0F + (real32_T)
        k) - 1].pos_y[4]) + rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) -
                    1].pos_y[2] * 5.0F * rtb_Switch_sections[(int32_T)(1.0F +
        (real32_T)k) - 1].pos_y[3]) - rtb_Switch_sections[(int32_T)(1.0F +
        (real32_T)k) - 1].pos_y[0] * 5.0F * yf) + rtb_Switch_sections[(int32_T)
                  (1.0F + (real32_T)k) - 1].pos_z[0] * 5.0F *
                  rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1].pos_z[5])
                 + rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1].pos_z
                 [1] * 5.0F * rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k)
                 - 1].pos_z[4]) + rtb_Switch_sections[(int32_T)(1.0F + (real32_T)
                 k) - 1].pos_z[2] * 5.0F * rtb_Switch_sections[(int32_T)(1.0F +
                 (real32_T)k) - 1].pos_z[3]) - rtb_Switch_sections[(int32_T)
        (1.0F + (real32_T)k) - 1].pos_z[0] * 5.0F * zf;
      P_0[6] = ((((((((((rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1].
                         pos_x[3] * rtb_Switch_sections[(int32_T)(1.0F +
        (real32_T)k) - 1].pos_x[3] * 2.0F + rtb_Switch_sections[(int32_T)(1.0F +
        (real32_T)k) - 1].pos_y[3] * rtb_Switch_sections[(int32_T)(1.0F +
        (real32_T)k) - 1].pos_y[3] * 2.0F) + rtb_Switch_sections[(int32_T)(1.0F
        + (real32_T)k) - 1].pos_z[3] * rtb_Switch_sections[(int32_T)(1.0F +
        (real32_T)k) - 1].pos_z[3] * 2.0F) + rtb_Switch_sections[(int32_T)(1.0F
        + (real32_T)k) - 1].pos_x[1] * 4.0F * rtb_Switch_sections[(int32_T)(1.0F
        + (real32_T)k) - 1].pos_x[5]) + rtb_Switch_sections[(int32_T)(1.0F +
        (real32_T)k) - 1].pos_x[2] * 4.0F * rtb_Switch_sections[(int32_T)(1.0F +
        (real32_T)k) - 1].pos_x[4]) - rtb_Switch_sections[(int32_T)(1.0F +
        (real32_T)k) - 1].pos_x[1] * 4.0F * xf) + rtb_Switch_sections[(int32_T)
                    (1.0F + (real32_T)k) - 1].pos_y[1] * 4.0F *
                    rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1]
                    .pos_y[5]) + rtb_Switch_sections[(int32_T)(1.0F + (real32_T)
        k) - 1].pos_y[2] * 4.0F * rtb_Switch_sections[(int32_T)(1.0F + (real32_T)
        k) - 1].pos_y[4]) - rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) -
                  1].pos_y[1] * 4.0F * yf) + rtb_Switch_sections[(int32_T)(1.0F
                  + (real32_T)k) - 1].pos_z[1] * 4.0F * rtb_Switch_sections
                 [(int32_T)(1.0F + (real32_T)k) - 1].pos_z[5]) +
                rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1].pos_z[2] *
                4.0F * rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1].
                pos_z[4]) - rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) -
        1].pos_z[1] * 4.0F * zf;
      P_0[7] = (((((((rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1].
                      pos_x[2] * 3.0F * rtb_Switch_sections[(int32_T)(1.0F +
        (real32_T)k) - 1].pos_x[5] + rtb_Switch_sections[(int32_T)(1.0F +
        (real32_T)k) - 1].pos_x[3] * 3.0F * rtb_Switch_sections[(int32_T)(1.0F +
        (real32_T)k) - 1].pos_x[4]) - rtb_Switch_sections[(int32_T)(1.0F +
        (real32_T)k) - 1].pos_x[2] * 3.0F * xf) + rtb_Switch_sections[(int32_T)
                    (1.0F + (real32_T)k) - 1].pos_y[2] * 3.0F *
                    rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1]
                    .pos_y[5]) + rtb_Switch_sections[(int32_T)(1.0F + (real32_T)
        k) - 1].pos_y[3] * 3.0F * rtb_Switch_sections[(int32_T)(1.0F + (real32_T)
        k) - 1].pos_y[4]) - rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) -
                  1].pos_y[2] * 3.0F * yf) + rtb_Switch_sections[(int32_T)(1.0F
                  + (real32_T)k) - 1].pos_z[2] * 3.0F * rtb_Switch_sections
                 [(int32_T)(1.0F + (real32_T)k) - 1].pos_z[5]) +
                rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1].pos_z[3] *
                3.0F * rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1].
                pos_z[4]) - rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) -
        1].pos_z[2] * 3.0F * zf;
      P_0[8] = (((((((rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1].
                      pos_x[4] * rtb_Switch_sections[(int32_T)(1.0F + (real32_T)
        k) - 1].pos_x[4] + rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1]
                      .pos_y[4] * rtb_Switch_sections[(int32_T)(1.0F + (real32_T)
        k) - 1].pos_y[4]) + rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) -
                     1].pos_z[4] * rtb_Switch_sections[(int32_T)(1.0F +
        (real32_T)k) - 1].pos_z[4]) + rtb_Switch_sections[(int32_T)(1.0F +
        (real32_T)k) - 1].pos_x[3] * 2.0F * rtb_Switch_sections[(int32_T)(1.0F +
        (real32_T)k) - 1].pos_x[5]) - rtb_Switch_sections[(int32_T)(1.0F +
        (real32_T)k) - 1].pos_x[3] * 2.0F * xf) + rtb_Switch_sections[(int32_T)
                  (1.0F + (real32_T)k) - 1].pos_y[3] * 2.0F *
                  rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1].pos_y[5])
                 - rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1].pos_y
                 [3] * 2.0F * yf) + rtb_Switch_sections[(int32_T)(1.0F +
                 (real32_T)k) - 1].pos_z[3] * 2.0F * rtb_Switch_sections
                [(int32_T)(1.0F + (real32_T)k) - 1].pos_z[5]) -
        rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1].pos_z[3] * 2.0F *
        zf;
      P_0[9] = ((((rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1].pos_x
                   [4] * rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1].
                   pos_x[5] - rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k)
                   - 1].pos_x[4] * xf) + rtb_Switch_sections[(int32_T)(1.0F +
        (real32_T)k) - 1].pos_y[4] * rtb_Switch_sections[(int32_T)(1.0F +
        (real32_T)k) - 1].pos_y[5]) - rtb_Switch_sections[(int32_T)(1.0F +
                  (real32_T)k) - 1].pos_y[4] * yf) + rtb_Switch_sections
                [(int32_T)(1.0F + (real32_T)k) - 1].pos_z[4] *
                rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1].pos_z[5])
        - rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1].pos_z[4] * zf;
      u = std::abs(P_0[0]);
      if (u <= 1.17549435E-38F) {
        px = 1.4013E-45F;
      } else {
        std::frexp(u, &exponent);
        px = std::ldexp(1.0F, exponent - 24);
      }

      if (u <= px) {
        if (u <= 1.17549435E-38F) {
          P_0[0] = 1.4013E-45F;
        } else {
          std::frexp(u, &b_exponent);
          P_0[0] = std::ldexp(1.0F, b_exponent - 24);
        }
      }

      for (b_k = 0; b_k < 81; b_k++) {
        A[b_k] = b[b_k];
      }

      for (b_k = 0; b_k < 9; b_k++) {
        A[9 * b_k] = -P_0[1 + b_k] / P_0[0];
      }

      xgehrd(A);
      eml_dlahqr(A);
      istart = 4;
      for (b_k = 0; b_k < 6; b_k++) {
        for (rtb_wp_out_tmp = istart; rtb_wp_out_tmp < 10; rtb_wp_out_tmp++) {
          A[(rtb_wp_out_tmp + 9 * b_k) - 1] = 0.0F;
        }

        istart++;
      }

      T[0] = A[0];
      T[1] = A[10];
      T[2] = A[20];
      T[3] = A[30];
      T[4] = A[40];
      T[5] = A[50];
      T[6] = A[60];
      T[7] = A[70];
      T[8] = A[80];
      for (rtb_wp_out_tmp = 0; rtb_wp_out_tmp < 11; rtb_wp_out_tmp++) {
        if ((T[rtb_wp_out_tmp] <= 1.0F) && (T[rtb_wp_out_tmp] >= 0.0F)) {
          t_values[5] = 1.0F;
          t_values[4] = T[rtb_wp_out_tmp];
          t_values[3] = T[rtb_wp_out_tmp] * T[rtb_wp_out_tmp];
          t_values[2] = t_values[3] * T[rtb_wp_out_tmp];
          t_values[1] = t_values[2] * T[rtb_wp_out_tmp];
          t_values[0] = t_values[1] * T[rtb_wp_out_tmp];
          u = 0.0F;
          for (b_k = 0; b_k < 6; b_k++) {
            u += rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1]
              .pos_x[b_k] * t_values[b_k];
          }

          rtb_TSamp[0] = u - xf;
          u = 0.0F;
          for (b_k = 0; b_k < 6; b_k++) {
            u += rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1]
              .pos_y[b_k] * t_values[b_k];
          }

          rtb_TSamp[1] = u - yf;
          u = 0.0F;
          for (b_k = 0; b_k < 6; b_k++) {
            u += rtb_Switch_sections[(int32_T)(1.0F + (real32_T)k) - 1]
              .pos_z[b_k] * t_values[b_k];
          }

          u -= zf;
          px = (rtb_TSamp[0] * rtb_TSamp[0] + rtb_TSamp[1] * rtb_TSamp[1]) + u *
            u;
          if (px < py) {
            rtb_yaw = T[rtb_wp_out_tmp];
            rtb_y_f = 1.0F + (real32_T)k;
            py = px;
          }
        }
      }
    }

    b_section_idx = rtb_y_f;
    if (rtb_y_f > rtb_uT) {
      b_section_idx = 1.0;
    }

    px = rtb_Switch_sections[(int32_T)b_section_idx - 1].pos_x[0];
    py = rtb_Switch_sections[(int32_T)b_section_idx - 1].pos_y[0];
    xf = rtb_Switch_sections[(int32_T)b_section_idx - 1].pos_z[0];
    for (k = 0; k < 5; k++) {
      px = rtb_yaw * px + rtb_Switch_sections[(int32_T)b_section_idx - 1]
        .pos_x[k + 1];
      py = rtb_yaw * py + rtb_Switch_sections[(int32_T)b_section_idx - 1]
        .pos_y[k + 1];
      xf = rtb_yaw * xf + rtb_Switch_sections[(int32_T)b_section_idx - 1]
        .pos_z[k + 1];
    }

    rtDW.pos_match_g[0] = px;
    rtDW.pos_match_g[1] = py;
    rtDW.pos_match_g[2] = xf;

    // Gain: '<S25>/look ahead distance'
    px = 0.181333333F * rtb_Sqrt;

    // MATLAB Function: '<S25>/flight path look ahead' incorporates:
    //   MATLAB Function: '<S26>/flight path matching'

    total_arc_length = -1.0;
    total_distance = -1.0;
    py = dist_remaining;
    k = 0;
    if (std::abs(dist_remaining) < 2.22044605E-16F) {
      k = 1;
    }

    if (0 <= k - 1) {
      py = 2.22044605E-16F;
    }

    u = px / py;
    if (u < 0.0F) {
      u = std::ceil(u);
    } else {
      u = std::floor(u);
    }

    dist_remaining = std::abs(px - u * dist_remaining);
    if (px < 0.0F) {
      px = -1.0F;
    } else {
      if (px > 0.0F) {
        px = 1.0F;
      }
    }

    if (px >= 0.0F) {
      k = 0;
      exitg2 = false;
      while ((!exitg2) && (k <= (int32_T)(rtb_uT + 1.0F) - 1)) {
        px = rtb_y_f + (real32_T)k;
        if (px > rtb_uT) {
          px -= rtb_uT;
        }

        trajSectionGetArcLength(rtb_Switch_sections[(int32_T)px - 1].pos_x,
          rtb_Switch_sections[(int32_T)px - 1].pos_y, rtb_Switch_sections
          [(int32_T)px - 1].pos_z, rtb_yaw, &py, &xf);
        yf = rtb_Switch_sections[(int32_T)px - 1].arc_length - py;
        if (dist_remaining <= yf) {
          total_distance = px;
          total_arc_length = rtb_yaw - ((py - dist_remaining) - py) / xf;
          trajSectionGetArcLength_l(rtb_Switch_sections[(int32_T)px - 1].pos_x,
            rtb_Switch_sections[(int32_T)px - 1].pos_y, rtb_Switch_sections
            [(int32_T)px - 1].pos_z, total_arc_length, &rtb_y_f, &rtb_yaw);
          total_arc_length = (real32_T)total_arc_length - ((rtb_y_f -
            dist_remaining) - py) / rtb_yaw;
          trajSectionGetArcLength_l(rtb_Switch_sections[(int32_T)px - 1].pos_x,
            rtb_Switch_sections[(int32_T)px - 1].pos_y, rtb_Switch_sections
            [(int32_T)px - 1].pos_z, total_arc_length, &rtb_y_f, &rtb_yaw);
          total_arc_length = (real32_T)total_arc_length - ((rtb_y_f -
            dist_remaining) - py) / rtb_yaw;
          exitg2 = true;
        } else {
          dist_remaining -= yf;
          rtb_yaw = 0.0F;
          k++;
        }
      }
    } else {
      b_k = 0;
      exitg1 = false;
      while ((!exitg1) && (b_k <= (int32_T)(rtb_uT + 1.0F) - 1)) {
        px = rtb_y_f - (real32_T)b_k;
        if (px < 1.0F) {
          px += rtb_uT;
        }

        trajSectionGetArcLength(rtb_Switch_sections[(int32_T)px - 1].pos_x,
          rtb_Switch_sections[(int32_T)px - 1].pos_y, rtb_Switch_sections
          [(int32_T)px - 1].pos_z, rtb_yaw, &py, &xf);
        if (dist_remaining <= py) {
          total_distance = px;
          total_arc_length = rtb_yaw - ((dist_remaining + py) - py) / xf;
          trajSectionGetArcLength_l(rtb_Switch_sections[(int32_T)px - 1].pos_x,
            rtb_Switch_sections[(int32_T)px - 1].pos_y, rtb_Switch_sections
            [(int32_T)px - 1].pos_z, total_arc_length, &rtb_y_f, &rtb_yaw);
          total_arc_length = (real32_T)total_arc_length - ((dist_remaining +
            rtb_y_f) - py) / rtb_yaw;
          trajSectionGetArcLength_l(rtb_Switch_sections[(int32_T)px - 1].pos_x,
            rtb_Switch_sections[(int32_T)px - 1].pos_y, rtb_Switch_sections
            [(int32_T)px - 1].pos_z, total_arc_length, &rtb_y_f, &rtb_yaw);
          total_arc_length = (real32_T)total_arc_length - ((dist_remaining +
            rtb_y_f) - py) / rtb_yaw;
          exitg1 = true;
        } else {
          dist_remaining -= py;
          rtb_yaw = 1.0F;
          b_k++;
        }
      }
    }

    // MATLAB Function: '<S27>/position controller reference from flight path' incorporates:
    //   Inport: '<Root>/cmd'
    //   MATLAB Function: '<S25>/flight path look ahead'
    //   Selector: '<S9>/Selector'

    b_section_idx = (real32_T)total_distance;
    if (((real32_T)total_distance > rtb_uT) || ((real32_T)total_distance < 1.0F))
    {
      b_section_idx = 1.0;
    }

    px = rtb_Switch_sections[(int32_T)b_section_idx - 1].pos_x[0];
    py = rtb_Switch_sections[(int32_T)b_section_idx - 1].pos_y[0];
    xf = rtb_Switch_sections[(int32_T)b_section_idx - 1].pos_z[0];
    for (rtb_wp_out_tmp = 0; rtb_wp_out_tmp < 5; rtb_wp_out_tmp++) {
      px = (real32_T)total_arc_length * px + rtb_Switch_sections[(int32_T)
        b_section_idx - 1].pos_x[rtb_wp_out_tmp + 1];
      py = (real32_T)total_arc_length * py + rtb_Switch_sections[(int32_T)
        b_section_idx - 1].pos_y[rtb_wp_out_tmp + 1];
      xf = (real32_T)total_arc_length * xf + rtb_Switch_sections[(int32_T)
        b_section_idx - 1].pos_z[rtb_wp_out_tmp + 1];
    }

    polyder_p(rtb_Switch_sections[(int32_T)b_section_idx - 1].pos_x, dx_data,
              tmp_size);
    polyder_p(rtb_Switch_sections[(int32_T)b_section_idx - 1].pos_y, dy_data,
              tmp_size_0);
    polyder_p(rtb_Switch_sections[(int32_T)b_section_idx - 1].pos_z, dz_data,
              tmp_size_1);
    rtDW.s_g_dt_ref[0] = polyVal(dx_data, tmp_size, (real_T)(real32_T)
      total_arc_length);
    rtDW.s_g_dt_ref[1] = polyVal(dy_data, tmp_size_0, (real_T)(real32_T)
      total_arc_length);
    rtDW.s_g_dt_ref[2] = polyVal(dz_data, tmp_size_1, (real_T)(real32_T)
      total_arc_length);
    rtb_uT = norm(rtDW.s_g_dt_ref);
    if (2.22044605E-16F < rtb_uT) {
      rtb_yaw = rtb_uT;
    } else {
      rtb_yaw = 2.22044605E-16F;
    }

    trajSectionGetFrenetSerretWit_k(rtb_Switch_sections[(int32_T)b_section_idx -
      1].pos_x, rtb_Switch_sections[(int32_T)b_section_idx - 1].pos_y,
      rtb_Switch_sections[(int32_T)b_section_idx - 1].pos_z, rtb_Sqrt, (real_T)
      (real32_T)total_arc_length, rtb_TSamp, rtb_TSamp_k, rtDW.s_g_dt2_ref,
      &rtb_uT, &rtb_y_f);
    rtb_Sqrt *= rtb_Sqrt;
    rtDW.s_g_ref[0] = px;
    rtDW.s_g_ref[1] = py;
    rtDW.s_g_ref[2] = xf;
    rtDW.s_g_dt_ref[0] = rtDW.s_g_dt_ref[0] / rtb_yaw * rtU.cmd.waypoints[3];
    rtDW.s_g_dt2_ref[0] = rtb_uT * rtDW.s_g_dt2_ref[0] * rtb_Sqrt;
    rtDW.s_g_dt_ref[1] = rtDW.s_g_dt_ref[1] / rtb_yaw * rtU.cmd.waypoints[3];
    rtDW.s_g_dt2_ref[1] = rtb_uT * rtDW.s_g_dt2_ref[1] * rtb_Sqrt;
    rtDW.s_g_dt_ref[2] = rtDW.s_g_dt_ref[2] / rtb_yaw * rtU.cmd.waypoints[3];
    rtDW.s_g_dt2_ref[2] = rtb_uT * rtDW.s_g_dt2_ref[2] * rtb_Sqrt;

    // End of MATLAB Function: '<S27>/position controller reference from flight path' 

    // Update for Delay: '<S23>/Delay'
    rtDW.Delay_DSTATE_d = rtb_OR;

    // Update for UnitDelay: '<S28>/Delay Input1'
    //
    //  Block description for '<S28>/Delay Input1':
    //
    //   Store in Global RAM

    rtDW.DelayInput1_DSTATE = rtb_Compare_f;
  }

  // End of Outputs for SubSystem: '<S3>/Flight path guidance with position controller reference' 

  // SampleTimeMath: '<S8>/TSamp' incorporates:
  //   Inport: '<Root>/measure'
  //
  //  About '<S8>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  u = rtU.measure.omega_Kb[0] * 400.0F;

  // Sum: '<S8>/Diff' incorporates:
  //   UnitDelay: '<S8>/UD'
  //
  //  Block description for '<S8>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S8>/UD':
  //
  //   Store in Global RAM

  rtb_Diff[0] = u - rtDW.UD_DSTATE[0];

  // SampleTimeMath: '<S8>/TSamp' incorporates:
  //   Inport: '<Root>/measure'
  //
  //  About '<S8>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  rtb_TSamp[0] = u;
  u = rtU.measure.omega_Kb[1] * 400.0F;

  // Sum: '<S8>/Diff' incorporates:
  //   UnitDelay: '<S8>/UD'
  //
  //  Block description for '<S8>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S8>/UD':
  //
  //   Store in Global RAM

  rtb_Diff[1] = u - rtDW.UD_DSTATE[1];

  // SampleTimeMath: '<S8>/TSamp' incorporates:
  //   Inport: '<Root>/measure'
  //
  //  About '<S8>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  rtb_TSamp[1] = u;
  u = rtU.measure.omega_Kb[2] * 400.0F;

  // Sum: '<S8>/Diff' incorporates:
  //   UnitDelay: '<S8>/UD'
  //
  //  Block description for '<S8>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S8>/UD':
  //
  //   Store in Global RAM

  rtb_Diff[2] = u - rtDW.UD_DSTATE[2];

  // DiscreteIntegrator: '<S44>/Discrete-Time Integrator'
  rtb_y_idx_0 = rtDW.DiscreteTimeIntegrator_DSTATE[0];
  rtb_y_idx_1 = rtDW.DiscreteTimeIntegrator_DSTATE[1];
  rtb_y_idx_2 = rtDW.DiscreteTimeIntegrator_DSTATE[2];
  rtb_y_idx_3 = rtDW.DiscreteTimeIntegrator_DSTATE[3];

  // MATLAB Function: '<S40>/MATLAB Function' incorporates:
  //   Constant: '<S40>/Constant4'
  //   Constant: '<S40>/ny_du_dt'

  for (rtb_wp_out_tmp = 0; rtb_wp_out_tmp < 16; rtb_wp_out_tmp++) {
    rtb_G2[rtb_wp_out_tmp] = rtConstP.ny_du_dt_Value[rtb_wp_out_tmp] / 0.0025F;
    d[rtb_wp_out_tmp] = 0.0F;
  }

  // DiscreteIntegrator: '<S67>/Discrete-Time Integrator'
  rtb_Sqrt = rtDW.DiscreteTimeIntegrator_DSTATE_n;

  // Gain: '<S67>/1//T' incorporates:
  //   DiscreteIntegrator: '<S67>/Discrete-Time Integrator'
  //   Gain: '<S51>/r_max'
  //   Inport: '<Root>/cmd'
  //   Sum: '<S67>/Sum2'

  rtb_uT = (5.23598766F * rtU.cmd.yaw - rtDW.DiscreteTimeIntegrator_DSTATE_n) *
    10.0F;

  // MATLAB Function: '<Root>/Quaternions to Rotation Matrix'
  py = 1.29246971E-26F;

  // MinMax: '<S40>/Max' incorporates:
  //   DiscreteIntegrator: '<S44>/Discrete-Time Integrator'

  if (rtDW.DiscreteTimeIntegrator_DSTATE[0] > 0.1F) {
    yf = rtDW.DiscreteTimeIntegrator_DSTATE[0];
  } else {
    yf = 0.1F;
  }

  // MATLAB Function: '<S40>/MATLAB Function'
  d[0] = yf / 0.283F;

  // MATLAB Function: '<Root>/Quaternions to Rotation Matrix' incorporates:
  //   Inport: '<Root>/measure'

  px = std::abs(rtU.measure.q_bg[0]);
  if (px > 1.29246971E-26F) {
    xf = 1.0F;
    py = px;
  } else {
    rtb_yaw = px / 1.29246971E-26F;
    xf = rtb_yaw * rtb_yaw;
  }

  // MinMax: '<S40>/Max' incorporates:
  //   DiscreteIntegrator: '<S44>/Discrete-Time Integrator'

  if (rtDW.DiscreteTimeIntegrator_DSTATE[1] > 0.1F) {
    yf = rtDW.DiscreteTimeIntegrator_DSTATE[1];
  } else {
    yf = 0.1F;
  }

  // MATLAB Function: '<S40>/MATLAB Function'
  d[5] = yf / 0.283F;

  // MATLAB Function: '<Root>/Quaternions to Rotation Matrix' incorporates:
  //   Inport: '<Root>/measure'

  px = std::abs(rtU.measure.q_bg[1]);
  if (px > py) {
    rtb_yaw = py / px;
    xf = xf * rtb_yaw * rtb_yaw + 1.0F;
    py = px;
  } else {
    rtb_yaw = px / py;
    xf += rtb_yaw * rtb_yaw;
  }

  // MinMax: '<S40>/Max' incorporates:
  //   DiscreteIntegrator: '<S44>/Discrete-Time Integrator'

  if (rtDW.DiscreteTimeIntegrator_DSTATE[2] > 0.1F) {
    yf = rtDW.DiscreteTimeIntegrator_DSTATE[2];
  } else {
    yf = 0.1F;
  }

  // MATLAB Function: '<S40>/MATLAB Function'
  d[10] = yf / 0.283F;

  // MATLAB Function: '<Root>/Quaternions to Rotation Matrix' incorporates:
  //   Inport: '<Root>/measure'

  px = std::abs(rtU.measure.q_bg[2]);
  if (px > py) {
    rtb_yaw = py / px;
    xf = xf * rtb_yaw * rtb_yaw + 1.0F;
    py = px;
  } else {
    rtb_yaw = px / py;
    xf += rtb_yaw * rtb_yaw;
  }

  // MinMax: '<S40>/Max' incorporates:
  //   DiscreteIntegrator: '<S44>/Discrete-Time Integrator'

  if (rtDW.DiscreteTimeIntegrator_DSTATE[3] > 0.1F) {
    yf = rtDW.DiscreteTimeIntegrator_DSTATE[3];
  } else {
    yf = 0.1F;
  }

  // MATLAB Function: '<S40>/MATLAB Function'
  d[15] = yf / 0.283F;

  // MATLAB Function: '<Root>/Quaternions to Rotation Matrix' incorporates:
  //   Inport: '<Root>/measure'

  px = std::abs(rtU.measure.q_bg[3]);
  if (px > py) {
    rtb_yaw = py / px;
    xf = xf * rtb_yaw * rtb_yaw + 1.0F;
    py = px;
  } else {
    rtb_yaw = px / py;
    xf += rtb_yaw * rtb_yaw;
  }

  xf = py * std::sqrt(xf);
  if (2.22044605E-16F < xf) {
    rtb_yaw = xf;
  } else {
    rtb_yaw = 2.22044605E-16F;
  }

  rtb_y_a[0] = rtU.measure.q_bg[0] / rtb_yaw;
  rtb_y_a[1] = rtU.measure.q_bg[1] / rtb_yaw;
  rtb_y_a[2] = rtU.measure.q_bg[2] / rtb_yaw;
  rtb_y_a[3] = rtU.measure.q_bg[3] / rtb_yaw;
  rtb_yaw = rtb_y_a[0] * rtb_y_a[0];
  px = rtb_y_a[1] * rtb_y_a[1];
  py = rtb_y_a[2] * rtb_y_a[2];
  xf = rtb_y_a[3] * rtb_y_a[3];
  yf = rtb_y_a[0] * rtb_y_a[1];
  zf = rtb_y_a[0] * rtb_y_a[2];
  q0_q3 = rtb_y_a[0] * rtb_y_a[3];
  q1_q2 = rtb_y_a[1] * rtb_y_a[2];
  q1_q3 = rtb_y_a[1] * rtb_y_a[3];
  q2_q3 = rtb_y_a[2] * rtb_y_a[3];
  rtb_M_bg[0] = ((rtb_yaw + px) - py) - xf;
  rtb_M_bg[3] = (q1_q2 + q0_q3) * 2.0F;
  rtb_M_bg[6] = (q1_q3 - zf) * 2.0F;
  rtb_M_bg[1] = (q1_q2 - q0_q3) * 2.0F;
  q0_q3 = rtb_yaw - px;
  rtb_M_bg[4] = (q0_q3 + py) - xf;
  rtb_M_bg[7] = (q2_q3 + yf) * 2.0F;
  rtb_M_bg[2] = (q1_q3 + zf) * 2.0F;
  rtb_M_bg[5] = (q2_q3 - yf) * 2.0F;
  rtb_M_bg[8] = (q0_q3 - py) + xf;

  // SampleTimeMath: '<S7>/TSamp' incorporates:
  //   Inport: '<Root>/measure'
  //
  //  About '<S7>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  yf = rtU.measure.V_Kg[0] * 400.0F;

  // Sum: '<S7>/Diff' incorporates:
  //   UnitDelay: '<S7>/UD'
  //
  //  Block description for '<S7>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S7>/UD':
  //
  //   Store in Global RAM

  rtb_Diff_n[0] = yf - rtDW.UD_DSTATE_o[0];

  // SampleTimeMath: '<S7>/TSamp' incorporates:
  //   Inport: '<Root>/measure'
  //
  //  About '<S7>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  rtb_TSamp_k[0] = yf;
  yf = rtU.measure.V_Kg[1] * 400.0F;

  // Sum: '<S7>/Diff' incorporates:
  //   UnitDelay: '<S7>/UD'
  //
  //  Block description for '<S7>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S7>/UD':
  //
  //   Store in Global RAM

  rtb_Diff_n[1] = yf - rtDW.UD_DSTATE_o[1];

  // SampleTimeMath: '<S7>/TSamp' incorporates:
  //   Inport: '<Root>/measure'
  //
  //  About '<S7>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  rtb_TSamp_k[1] = yf;
  yf = rtU.measure.V_Kg[2] * 400.0F;

  // Sum: '<S7>/Diff' incorporates:
  //   UnitDelay: '<S7>/UD'
  //
  //  Block description for '<S7>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S7>/UD':
  //
  //   Store in Global RAM

  rtb_Diff_n[2] = yf - rtDW.UD_DSTATE_o[2];

  // Gain: '<S13>/Gain3' incorporates:
  //   Gain: '<Root>/Gain'
  //   Inport: '<Root>/cmd'
  //   MATLAB Function: '<S13>/MATLAB Function1'

  rtb_yaw = std::sqrt(rtU.cmd.roll * rtU.cmd.roll + -rtU.cmd.pitch *
                      -rtU.cmd.pitch) * 15.0F;

  // Product: '<S13>/Product' incorporates:
  //   Gain: '<Root>/Gain'
  //   Inport: '<Root>/cmd'

  dist_remaining = -rtU.cmd.pitch * rtb_yaw;
  py = rtU.cmd.roll * rtb_yaw;

  // Lookup_n-D: '<S13>/1-D Lookup Table' incorporates:
  //   Gain: '<Root>/Gain5'
  //   Inport: '<Root>/cmd'

  rtb_yaw = look1_iflf_binlx(-rtU.cmd.thr, rtConstP.uDLookupTable_bp01Data,
    rtConstP.uDLookupTable_tableData, 2U);

  // RelationalOperator: '<S16>/Relational Operator' incorporates:
  //   Memory: '<S16>/Memory'

  rtb_Compare_f = (rtb_Compare != rtDW.Memory_PreviousInput);

  // DiscreteIntegrator: '<S73>/Discrete-Time Integrator' incorporates:
  //   Logic: '<S3>/Logical Operator'

  if (rtDW.DiscreteTimeIntegrator_IC_LOADI != 0) {
    rtDW.DiscreteTimeIntegrator_DSTATE_k[0] = dist_remaining;
    rtDW.DiscreteTimeIntegrator_DSTATE_k[1] = py;
    rtDW.DiscreteTimeIntegrator_DSTATE_k[2] = rtb_yaw;
  }

  if (rtb_Compare_f && (rtDW.DiscreteTimeIntegrator_PrevRese <= 0)) {
    rtDW.DiscreteTimeIntegrator_DSTATE_k[0] = dist_remaining;
    rtDW.DiscreteTimeIntegrator_DSTATE_k[1] = py;
    rtDW.DiscreteTimeIntegrator_DSTATE_k[2] = rtb_yaw;
  }

  zf = rtDW.DiscreteTimeIntegrator_DSTATE_k[0];
  q1_q3 = rtDW.DiscreteTimeIntegrator_DSTATE_k[1];
  q2_q3 = rtDW.DiscreteTimeIntegrator_DSTATE_k[2];

  // Gain: '<S73>/1//T' incorporates:
  //   DiscreteIntegrator: '<S73>/Discrete-Time Integrator'
  //   Sum: '<S73>/Sum2'

  q0_q3 = (dist_remaining - rtDW.DiscreteTimeIntegrator_DSTATE_k[0]) *
    1.66666663F;

  // Saturate: '<S73>/Saturation'
  if (q0_q3 > 12.0F) {
    q0_q3 = 12.0F;
  } else {
    if (q0_q3 < -12.0F) {
      q0_q3 = -12.0F;
    }
  }

  // Gain: '<S73>/1//T' incorporates:
  //   DiscreteIntegrator: '<S73>/Discrete-Time Integrator'
  //   Sum: '<S73>/Sum2'

  q1_q2 = (py - rtDW.DiscreteTimeIntegrator_DSTATE_k[1]) * 1.66666663F;

  // Saturate: '<S73>/Saturation'
  if (q1_q2 > 12.0F) {
    q1_q2 = 12.0F;
  } else {
    if (q1_q2 < -12.0F) {
      q1_q2 = -12.0F;
    }
  }

  // Gain: '<S73>/1//T' incorporates:
  //   DiscreteIntegrator: '<S73>/Discrete-Time Integrator'
  //   Sum: '<S73>/Sum2'

  rtb_y_dt_idx_2 = (rtb_yaw - rtDW.DiscreteTimeIntegrator_DSTATE_k[2]) *
    1.66666663F;

  // Saturate: '<S73>/Saturation'
  if (rtb_y_dt_idx_2 > 4.0F) {
    rtb_y_dt_idx_2 = 4.0F;
  } else {
    if (rtb_y_dt_idx_2 < -25.0F) {
      rtb_y_dt_idx_2 = -25.0F;
    }
  }

  // Switch: '<S18>/Switch2'
  if (rtb_Compare > 0) {
    rtb_y_f = rtDW.s_g_dt2_ref[0];
    rtb_Switch2_idx_1 = rtDW.s_g_dt2_ref[1];
    rtb_Switch2_idx_2 = rtDW.s_g_dt2_ref[2];
  } else {
    rtb_y_f = q0_q3;
    rtb_Switch2_idx_1 = q1_q2;
    rtb_Switch2_idx_2 = rtb_y_dt_idx_2;
  }

  // End of Switch: '<S18>/Switch2'

  // Sum: '<S75>/Add' incorporates:
  //   DiscreteIntegrator: '<S76>/Discrete-Time Integrator y'
  //   Inport: '<Root>/measure'

  rtb_Sum2_bj[0] = rtDW.DiscreteTimeIntegratory_DSTATE[0] - rtU.measure.s_Kg[0];

  // Switch: '<S77>/Switch2' incorporates:
  //   RelationalOperator: '<S77>/LowerRelop1'
  //   RelationalOperator: '<S77>/UpperRelop'
  //   Switch: '<S77>/Switch'

  if (rtb_Sum2_bj[0] > 12.2174301F) {
    rtb_yaw = 12.2174301F;
  } else if (rtb_Sum2_bj[0] < -12.2174301F) {
    // Switch: '<S77>/Switch'
    rtb_yaw = -12.2174301F;
  } else {
    rtb_yaw = rtb_Sum2_bj[0];
  }

  // Sum: '<S75>/Add1' incorporates:
  //   DiscreteIntegrator: '<S76>/Discrete-Time Integrator y'
  //   Gain: '<S75>/Gain'
  //   Gain: '<S75>/Gain3'
  //   Gain: '<S75>/Gain4'
  //   Inport: '<Root>/measure'
  //   Sum: '<S75>/Add'

  rtb_yaw = ((rtDW.DiscreteTimeIntegratory_DSTATE[3] - rtU.measure.V_Kg[0]) *
             3.9012F + 3.3528F * rtb_yaw) +
    (rtDW.DiscreteTimeIntegratory_DSTATE[6] - rtb_Diff_n[0]) * 0.7114F;

  // RelationalOperator: '<S78>/LowerRelop1'
  rtb_FixPtRelationalOperator = (rtb_yaw > 9.0F);

  // Sum: '<S75>/Add1'
  rtb_Delta_ny[0] = rtb_yaw;

  // Sum: '<S75>/Add' incorporates:
  //   DiscreteIntegrator: '<S76>/Discrete-Time Integrator y'
  //   Inport: '<Root>/measure'

  rtb_Sum2_bj[1] = rtDW.DiscreteTimeIntegratory_DSTATE[1] - rtU.measure.s_Kg[1];

  // Switch: '<S77>/Switch2' incorporates:
  //   RelationalOperator: '<S77>/LowerRelop1'
  //   RelationalOperator: '<S77>/UpperRelop'
  //   Switch: '<S77>/Switch'

  if (rtb_Sum2_bj[1] > 12.2174301F) {
    rtb_yaw = 12.2174301F;
  } else if (rtb_Sum2_bj[1] < -12.2174301F) {
    // Switch: '<S77>/Switch'
    rtb_yaw = -12.2174301F;
  } else {
    rtb_yaw = rtb_Sum2_bj[1];
  }

  // Sum: '<S75>/Add1' incorporates:
  //   DiscreteIntegrator: '<S76>/Discrete-Time Integrator y'
  //   Gain: '<S75>/Gain'
  //   Gain: '<S75>/Gain3'
  //   Gain: '<S75>/Gain4'
  //   Inport: '<Root>/measure'
  //   Sum: '<S75>/Add'

  rtb_yaw = ((rtDW.DiscreteTimeIntegratory_DSTATE[4] - rtU.measure.V_Kg[1]) *
             3.9012F + 3.3528F * rtb_yaw) +
    (rtDW.DiscreteTimeIntegratory_DSTATE[7] - rtb_Diff_n[1]) * 0.7114F;

  // RelationalOperator: '<S78>/LowerRelop1'
  rtb_OR = (rtb_yaw > 9.0F);

  // Sum: '<S75>/Add1'
  rtb_Delta_ny[1] = rtb_yaw;

  // Sum: '<S75>/Add' incorporates:
  //   DiscreteIntegrator: '<S76>/Discrete-Time Integrator y'
  //   Inport: '<Root>/measure'

  rtb_Sum2_bj[2] = rtDW.DiscreteTimeIntegratory_DSTATE[2] - rtU.measure.s_Kg[2];

  // Switch: '<S77>/Switch2' incorporates:
  //   RelationalOperator: '<S77>/LowerRelop1'
  //   RelationalOperator: '<S77>/UpperRelop'
  //   Switch: '<S77>/Switch'

  if (rtb_Sum2_bj[2] > 12.2174301F) {
    rtb_yaw = 12.2174301F;
  } else if (rtb_Sum2_bj[2] < -12.2174301F) {
    // Switch: '<S77>/Switch'
    rtb_yaw = -12.2174301F;
  } else {
    rtb_yaw = rtb_Sum2_bj[2];
  }

  // Sum: '<S75>/Add1' incorporates:
  //   DiscreteIntegrator: '<S76>/Discrete-Time Integrator y'
  //   Gain: '<S75>/Gain'
  //   Gain: '<S75>/Gain3'
  //   Gain: '<S75>/Gain4'
  //   Inport: '<Root>/measure'
  //   Sum: '<S75>/Add'

  rtb_yaw = ((rtDW.DiscreteTimeIntegratory_DSTATE[5] - rtU.measure.V_Kg[2]) *
             3.9012F + 3.3528F * rtb_yaw) +
    (rtDW.DiscreteTimeIntegratory_DSTATE[8] - rtb_Diff_n[2]) * 0.7114F;

  // RelationalOperator: '<S78>/UpperRelop'
  dist_remaining = rtb_Delta_ny[0];

  // Switch: '<S78>/Switch' incorporates:
  //   RelationalOperator: '<S78>/UpperRelop'

  if (rtb_Delta_ny[0] < -9.0F) {
    dist_remaining = -9.0F;
  }

  // Switch: '<S78>/Switch2'
  if (rtb_FixPtRelationalOperator) {
    dist_remaining = 9.0F;
  }

  // Switch: '<S78>/Switch' incorporates:
  //   RelationalOperator: '<S78>/UpperRelop'
  //   Sum: '<S14>/Add1'

  rtb_Delta_ny[0] = rtb_y_f + dist_remaining;

  // MATLAB Function: '<S6>/INDI Copter Acc 2 Lean Vector'
  rtb_n_b[0] = rtb_Diff_n[0];

  // RelationalOperator: '<S78>/UpperRelop'
  dist_remaining = rtb_Delta_ny[1];

  // Switch: '<S78>/Switch' incorporates:
  //   RelationalOperator: '<S78>/UpperRelop'

  if (rtb_Delta_ny[1] < -9.0F) {
    dist_remaining = -9.0F;
  }

  // Switch: '<S78>/Switch2'
  if (rtb_OR) {
    dist_remaining = 9.0F;
  }

  // Switch: '<S78>/Switch' incorporates:
  //   RelationalOperator: '<S78>/UpperRelop'
  //   Sum: '<S14>/Add1'

  rtb_Delta_ny[1] = rtb_Switch2_idx_1 + dist_remaining;

  // MATLAB Function: '<S6>/INDI Copter Acc 2 Lean Vector'
  rtb_n_b[1] = rtb_Diff_n[1];
  dist_remaining = rtb_yaw;

  // Switch: '<S78>/Switch' incorporates:
  //   RelationalOperator: '<S78>/UpperRelop'

  if (rtb_yaw < -9.0F) {
    dist_remaining = -9.0F;
  }

  // Switch: '<S78>/Switch2' incorporates:
  //   RelationalOperator: '<S78>/LowerRelop1'

  if (rtb_yaw > 9.0F) {
    dist_remaining = 9.0F;
  }

  // Switch: '<S78>/Switch' incorporates:
  //   RelationalOperator: '<S78>/UpperRelop'
  //   Sum: '<S14>/Add1'

  rtb_Delta_ny[2] = rtb_Switch2_idx_2 + dist_remaining;

  // MATLAB Function: '<S6>/INDI Copter Acc 2 Lean Vector'
  rtb_n_b[2] = rtb_Diff_n[2] - 9.81F;
  xf = norm(rtb_n_b);
  for (i = 0; i < 3; i++) {
    rtb_n_g_des[i] = -rtb_M_bg[3 * i + 2] + (rtb_Delta_ny[i] - rtb_Diff_n[i]) /
      xf;
  }

  rtb_yaw = norm(rtb_n_g_des);
  k = 0;
  if (std::abs(rtb_yaw) < 2.22044605E-16F) {
    k = 1;
  }

  if (0 <= k - 1) {
    rtb_yaw = 2.22044605E-16F;
  }

  rtb_n_g_des[0] /= rtb_yaw;
  rtb_n_g_des[1] /= rtb_yaw;
  rtb_n_g_des_b = rtb_n_g_des[2] / rtb_yaw;
  rtb_n_g_des[2] = rtb_n_g_des_b;

  // MATLAB Function: '<S6>/MATLAB Function4'
  rtb_yaw = -rtb_n_g_des_b;
  if (-rtb_n_g_des_b > 1.0F) {
    rtb_yaw = 1.0F;
  }

  if (rtb_yaw < -1.0F) {
    rtb_yaw = -1.0F;
  }

  rtb_yaw = std::acos(rtb_yaw);

  // Saturate: '<S6>/Saturation1'
  if (rtb_yaw > 6.28318548F) {
    rtb_yaw = 6.28318548F;
  } else {
    if (rtb_yaw < 0.0F) {
      rtb_yaw = 0.0F;
    }
  }

  // End of Saturate: '<S6>/Saturation1'

  // Gain: '<S51>/lean_angle_max' incorporates:
  //   Gain: '<S6>/Gain1'
  //   MATLAB Function: '<S6>/MATLAB Function4'

  dist_remaining = 0.159154937F * rtb_yaw * 6.28318548F;
  py = std::atan2(rtb_n_g_des[1], rtb_n_g_des[0]);

  // MATLAB Function: '<S51>/lean angles 2 lean vector'
  rtb_yaw = std::sin(dist_remaining);
  rtb_n_g[0] = rtb_yaw * std::cos(py);
  rtb_n_g[1] = rtb_yaw * std::sin(py);
  rtb_n_g[2] = -std::cos(dist_remaining);

  // DiscreteIntegrator: '<S61>/Discrete-Time Integrator y'
  if (rtDW.DiscreteTimeIntegratory_IC_LOAD != 0) {
    rtDW.DiscreteTimeIntegratory_DSTAT_a[0] = rtb_n_g[0];
    rtDW.DiscreteTimeIntegratory_DSTAT_a[1] = rtb_n_g[1];
    rtDW.DiscreteTimeIntegratory_DSTAT_a[2] = rtb_n_g[2];
  }

  // MATLAB Function: '<S51>/n ref norm'
  py = 1.29246971E-26F;

  // DiscreteIntegrator: '<S61>/Discrete-Time Integrator y_dt'
  rtb_y_dt_c[0] = rtDW.DiscreteTimeIntegratory_dt_DSTA[0];

  // MATLAB Function: '<S51>/n ref norm' incorporates:
  //   DiscreteIntegrator: '<S61>/Discrete-Time Integrator y'

  px = std::abs(rtDW.DiscreteTimeIntegratory_DSTAT_a[0]);
  if (px > 1.29246971E-26F) {
    dist_remaining = 1.0F;
    py = px;
  } else {
    rtb_yaw = px / 1.29246971E-26F;
    dist_remaining = rtb_yaw * rtb_yaw;
  }

  // DiscreteIntegrator: '<S61>/Discrete-Time Integrator y_dt'
  rtb_y_dt_c[1] = rtDW.DiscreteTimeIntegratory_dt_DSTA[1];

  // MATLAB Function: '<S51>/n ref norm' incorporates:
  //   DiscreteIntegrator: '<S61>/Discrete-Time Integrator y'

  px = std::abs(rtDW.DiscreteTimeIntegratory_DSTAT_a[1]);
  if (px > py) {
    rtb_yaw = py / px;
    dist_remaining = dist_remaining * rtb_yaw * rtb_yaw + 1.0F;
    py = px;
  } else {
    rtb_yaw = px / py;
    dist_remaining += rtb_yaw * rtb_yaw;
  }

  // DiscreteIntegrator: '<S61>/Discrete-Time Integrator y_dt'
  rtb_y_dt_c[2] = rtDW.DiscreteTimeIntegratory_dt_DSTA[2];

  // MATLAB Function: '<S51>/n ref norm' incorporates:
  //   DiscreteIntegrator: '<S61>/Discrete-Time Integrator y'

  px = std::abs(rtDW.DiscreteTimeIntegratory_DSTAT_a[2]);
  if (px > py) {
    rtb_yaw = py / px;
    dist_remaining = dist_remaining * rtb_yaw * rtb_yaw + 1.0F;
    py = px;
  } else {
    rtb_yaw = px / py;
    dist_remaining += rtb_yaw * rtb_yaw;
  }

  dist_remaining = py * std::sqrt(dist_remaining);
  k = 0;
  if (dist_remaining < 2.22044605E-16F) {
    k = 1;
  }

  if (0 <= k - 1) {
    dist_remaining = 2.22044605E-16F;
  }

  rtb_n[0] = rtDW.DiscreteTimeIntegratory_DSTAT_a[0] / dist_remaining;
  rtb_n[1] = rtDW.DiscreteTimeIntegratory_DSTAT_a[1] / dist_remaining;
  rtb_n[2] = rtDW.DiscreteTimeIntegratory_DSTAT_a[2] / dist_remaining;

  // DiscreteIntegrator: '<S68>/Discrete-Time Integrator y'
  if (rtDW.DiscreteTimeIntegratory_IC_LO_f != 0) {
    rtDW.DiscreteTimeIntegratory_DSTAT_f[0] = rtb_n[0];
    rtDW.DiscreteTimeIntegratory_DSTAT_f[1] = rtb_n[1];
    rtDW.DiscreteTimeIntegratory_DSTAT_f[2] = rtb_n[2];
  }

  rtb_y_jr[0] = rtDW.DiscreteTimeIntegratory_DSTAT_f[0];
  rtb_y_jr[1] = rtDW.DiscreteTimeIntegratory_DSTAT_f[1];
  rtb_y_jr[2] = rtDW.DiscreteTimeIntegratory_DSTAT_f[2];

  // DiscreteIntegrator: '<S69>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S61>/Discrete-Time Integrator y_dt'

  if (rtDW.DiscreteTimeIntegratory_IC_LO_b != 0) {
    rtDW.DiscreteTimeIntegratory_DSTAT_k[0] =
      rtDW.DiscreteTimeIntegratory_dt_DSTA[0];
    rtDW.DiscreteTimeIntegratory_DSTAT_k[1] =
      rtDW.DiscreteTimeIntegratory_dt_DSTA[1];
    rtDW.DiscreteTimeIntegratory_DSTAT_k[2] =
      rtDW.DiscreteTimeIntegratory_dt_DSTA[2];
  }

  rtb_y_g[0] = rtDW.DiscreteTimeIntegratory_DSTAT_k[0];

  // Gain: '<S61>/omega^2' incorporates:
  //   DiscreteIntegrator: '<S61>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S61>/Discrete-Time Integrator y_dt'
  //   Gain: '<S61>/2*d//omega'
  //   Sum: '<S61>/Sum2'
  //   Sum: '<S61>/Sum3'

  rtb_n_g[0] = (rtb_n_g[0] - (0.13333334F *
    rtDW.DiscreteTimeIntegratory_dt_DSTA[0] +
    rtDW.DiscreteTimeIntegratory_DSTAT_a[0])) * 225.0F;

  // DiscreteIntegrator: '<S69>/Discrete-Time Integrator y'
  rtb_y_g[1] = rtDW.DiscreteTimeIntegratory_DSTAT_k[1];

  // Gain: '<S61>/omega^2' incorporates:
  //   DiscreteIntegrator: '<S61>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S61>/Discrete-Time Integrator y_dt'
  //   Gain: '<S61>/2*d//omega'
  //   Sum: '<S61>/Sum2'
  //   Sum: '<S61>/Sum3'

  rtb_n_g[1] = (rtb_n_g[1] - (0.13333334F *
    rtDW.DiscreteTimeIntegratory_dt_DSTA[1] +
    rtDW.DiscreteTimeIntegratory_DSTAT_a[1])) * 225.0F;

  // DiscreteIntegrator: '<S69>/Discrete-Time Integrator y'
  rtb_y_g[2] = rtDW.DiscreteTimeIntegratory_DSTAT_k[2];

  // Gain: '<S61>/omega^2' incorporates:
  //   DiscreteIntegrator: '<S61>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S61>/Discrete-Time Integrator y_dt'
  //   Gain: '<S61>/2*d//omega'
  //   Sum: '<S61>/Sum2'
  //   Sum: '<S61>/Sum3'

  rtb_n_g_c = (rtb_n_g[2] - (0.13333334F * rtDW.DiscreteTimeIntegratory_dt_DSTA
    [2] + rtDW.DiscreteTimeIntegratory_DSTAT_a[2])) * 225.0F;
  rtb_n_g[2] = rtb_n_g_c;

  // DiscreteIntegrator: '<S70>/Discrete-Time Integrator y'
  if (rtDW.DiscreteTimeIntegratory_IC_LO_e != 0) {
    rtDW.DiscreteTimeIntegratory_DSTAT_p[0] = rtb_n_g[0];
    rtDW.DiscreteTimeIntegratory_DSTAT_p[1] = rtb_n_g[1];
    rtDW.DiscreteTimeIntegratory_DSTAT_p[2] = rtb_n_g_c;
  }

  rtb_y_p[0] = rtDW.DiscreteTimeIntegratory_DSTAT_p[0];
  rtb_y_p[1] = rtDW.DiscreteTimeIntegratory_DSTAT_p[1];
  rtb_y_p[2] = rtDW.DiscreteTimeIntegratory_DSTAT_p[2];

  // MATLAB Function: '<S51>/Lean Vector Derivative Trafo Delay' incorporates:
  //   Inport: '<Root>/measure'

  LeanVectorDerivativeTrafo(rtb_y_jr, rtb_y_g, rtb_y_p, rtb_M_bg,
    rtU.measure.omega_Kb, rtb_Diff, rtb_n_b, rtb_n_b_dt, rtb_Switch_k);

  // MATLAB Function: '<S51>/Lean Vector Derivative Trafo' incorporates:
  //   Inport: '<Root>/measure'

  LeanVectorDerivativeTrafo(rtb_n, rtb_y_dt_c, rtb_n_g, rtb_M_bg,
    rtU.measure.omega_Kb, rtb_Diff, rtb_y_g, rtb_y_p, rtb_y_jr);

  // MATLAB Function: '<S51>/MATLAB Function'
  if (rtb_n_b[2] > 0.0F) {
    py = 1.29246971E-26F;
    px = std::abs(rtb_n_b[0]);
    if (px > 1.29246971E-26F) {
      xf = 1.0F;
      py = px;
    } else {
      rtb_yaw = px / 1.29246971E-26F;
      xf = rtb_yaw * rtb_yaw;
    }

    px = std::abs(rtb_n_b[1]);
    if (px > py) {
      rtb_yaw = py / px;
      xf = xf * rtb_yaw * rtb_yaw + 1.0F;
      py = px;
    } else {
      rtb_yaw = px / py;
      xf += rtb_yaw * rtb_yaw;
    }

    xf = py * std::sqrt(xf);
    k = 0;
    if (xf < 2.22044605E-16F) {
      k = 1;
    }

    if (0 <= k - 1) {
      xf = 2.22044605E-16F;
    }

    rtb_n_b[0] = rtb_n_b[0] / xf * 2.0F - rtb_n_b[0];
    rtb_n_b_dt[0] = -rtb_n_b_dt[0];
    rtb_Switch_k[0] = -rtb_Switch_k[0];
    rtb_n_b[1] = rtb_n_b[1] / xf * 2.0F - rtb_n_b[1];
    rtb_n_b_dt[1] = -rtb_n_b_dt[1];
    rtb_Switch_k[1] = -rtb_Switch_k[1];
  }

  // MATLAB Function: '<S17>/DCM to quaternions'
  DCMtoquaternions(rtb_M_bg, rtb_y_a);

  // MATLAB Function: '<S17>/Quaternion Reduced'
  QuaternionReduced(rtb_y_a, rtb_q_red, &rtb_yaw);

  // DiscreteIntegrator: '<S51>/Discrete-Time Integrator2'
  if (rtDW.DiscreteTimeIntegrator2_IC_LOAD != 0) {
    rtDW.DiscreteTimeIntegrator2_DSTATE = rtb_yaw;
  }

  // MATLAB Function: '<S46>/wrap angle' incorporates:
  //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator2'

  wrapangle(rtDW.DiscreteTimeIntegrator2_DSTATE, &rtb_yaw);

  // MATLAB Function: '<S47>/DCM to quaternions'
  DCMtoquaternions(rtb_M_bg, rtb_y_a);

  // MATLAB Function: '<S47>/Quaternion Reduced'
  QuaternionReduced(rtb_y_a, rtb_q_red, &dist_remaining);

  // MATLAB Function: '<S46>/wrap angle1'
  wrapangle(dist_remaining, &px);

  // MATLAB Function: '<S46>/angle error'
  rtb_yaw -= px;
  if (rtb_yaw > 3.1415926535897931) {
    rtb_yaw -= 6.28318548F;
  } else {
    if (rtb_yaw < -3.1415926535897931) {
      rtb_yaw += 6.28318548F;
    }
  }

  // End of MATLAB Function: '<S46>/angle error'

  // DotProduct: '<S6>/Dot Product1'
  py = 0.0F;
  for (i = 0; i < 3; i++) {
    // DotProduct: '<S6>/Dot Product1' incorporates:
    //   Sum: '<S6>/Add2'

    py += rtb_n_g_des[i] * rtb_Delta_ny[i];

    // MATLAB Function: '<S6>/MATLAB Function'
    rtb_y_dt_c[i] = -rtb_M_bg[3 * i + 2];
  }

  // Gain: '<S6>/Gain' incorporates:
  //   DotProduct: '<S6>/Dot Product1'

  dist_remaining = -py;

  // MATLAB Function: '<S6>/no thrust if flipped' incorporates:
  //   DotProduct: '<S6>/Dot Product1'

  if ((rtb_y_dt_c[0] * rtb_n_g_des[0] + rtb_y_dt_c[1] * rtb_n_g_des[1]) +
      rtb_y_dt_c[2] * rtb_n_g_des_b < 0.0F) {
    dist_remaining = py;
  }

  // End of MATLAB Function: '<S6>/no thrust if flipped'

  // DiscreteIntegrator: '<S15>/Discrete-Time Integrator y'
  if (rtDW.DiscreteTimeIntegratory_IC_LO_g != 0) {
    rtDW.DiscreteTimeIntegratory_DSTAT_h = dist_remaining;
  }

  for (k = 0; k < 4; k++) {
    // MATLAB Function: '<S41>/MATLAB Function2'
    W[k] = 0.0;

    // MATLAB Function: '<S40>/MATLAB Function' incorporates:
    //   Constant: '<S40>/ny_du_red_trim'

    for (i = 0; i < 4; i++) {
      b_k = i << 2;
      tmp[k + b_k] = (((d[b_k + 1] * rtConstP.ny_du_red_trim_Value[k + 4] +
                        d[b_k] * rtConstP.ny_du_red_trim_Value[k]) + d[b_k + 2] *
                       rtConstP.ny_du_red_trim_Value[k + 8]) + d[b_k + 3] *
                      rtConstP.ny_du_red_trim_Value[k + 12]) + rtb_G2[b_k + k];
    }

    // Gain: '<S37>/Gain' incorporates:
    //   DiscreteIntegrator: '<S44>/Discrete-Time Integrator'
    //   MATLAB Function: '<S41>/MATLAB Function2'

    rtb_y_a[k] = 0.1F - rtDW.DiscreteTimeIntegrator_DSTATE[k];

    // Lookup_n-D: '<S37>/1-D Lookup Table2' incorporates:
    //   DiscreteIntegrator: '<S44>/Discrete-Time Integrator'
    //   MATLAB Function: '<S41>/MATLAB Function2'

    rtb_uDLookupTable2[k] = ((0.1F - rtDW.DiscreteTimeIntegrator_DSTATE[k]) +
      (1.0F - rtDW.DiscreteTimeIntegrator_DSTATE[k])) * 0.5F;

    // MATLAB Function: '<S41>/MATLAB Function2' incorporates:
    //   DiscreteIntegrator: '<S44>/Discrete-Time Integrator'

    rtb_q_red[k] = 1.0F - rtDW.DiscreteTimeIntegrator_DSTATE[k];
  }

  // Sum: '<S12>/Add1' incorporates:
  //   DiscreteIntegrator: '<S67>/Discrete-Time Integrator'
  //   Gain: '<S48>/Gain1'
  //   Gain: '<S48>/Gain2'
  //   Gain: '<S48>/Gain3'
  //   Gain: '<S48>/Gain4'
  //   Gain: '<S48>/Gain5'
  //   Gain: '<S48>/Gain6'
  //   Inport: '<Root>/measure'
  //   MATLAB Function: '<S51>/MATLAB Function'
  //   Sum: '<S46>/error1 4'
  //   Sum: '<S46>/error1 5'
  //   Sum: '<S46>/error1 6'
  //   Sum: '<S46>/error1 8'
  //   Sum: '<S46>/error1 9'
  //   Sum: '<S48>/Add'
  //   Sum: '<S48>/Add1'

  rtb_n_g_des[2] = ((rtDW.DiscreteTimeIntegrator_DSTATE_n -
                     rtU.measure.omega_Kb[2]) * 15.6068F + 50.0F * rtb_yaw) +
    (rtb_uT - rtb_Diff[2]) * 0.4357F;
  rtb_Delta_ny[2] = rtb_uT;
  rtb_n_g_des[0] = (200.0F * rtb_n_b[0] + 39.9604F * rtb_n_b_dt[0]) + 0.9513F *
    rtb_Switch_k[0];
  rtb_Delta_ny[0] = rtb_y_jr[0];

  // Sum: '<S12>/Add2' incorporates:
  //   Constant: '<S50>/Constant1'

  tmp_0[0] = 0.0F;

  // Sum: '<S12>/Add1' incorporates:
  //   Gain: '<S48>/Gain1'
  //   Gain: '<S48>/Gain3'
  //   Gain: '<S48>/Gain5'
  //   MATLAB Function: '<S51>/MATLAB Function'
  //   Sum: '<S46>/error1 4'
  //   Sum: '<S46>/error1 5'
  //   Sum: '<S46>/error1 6'
  //   Sum: '<S48>/Add'

  rtb_n_g_des[1] = (200.0F * rtb_n_b[1] + 39.9604F * rtb_n_b_dt[1]) + 0.9513F *
    rtb_Switch_k[1];
  rtb_Delta_ny[1] = rtb_y_jr[1];

  // Sum: '<S12>/Add2' incorporates:
  //   Constant: '<S50>/Constant1'

  tmp_0[1] = 0.0F;
  tmp_0[2] = rtb_Diff[2];
  for (i = 0; i < 3; i++) {
    // Sum: '<S37>/Add2' incorporates:
    //   Sum: '<S12>/Add1'
    //   Sum: '<S12>/Add2'

    rtb_y_k_0[i] = (rtb_Delta_ny[i] + rtb_n_g_des[i]) - tmp_0[i];

    // Product: '<S3>/Matrix Multiply1'
    rtb_y_jr[i] = rtb_M_bg[i + 6] * rtb_Diff_n[2] + (rtb_M_bg[i + 3] *
      rtb_Diff_n[1] + rtb_M_bg[i] * rtb_Diff_n[0]);
  }

  // Sum: '<S37>/Add2' incorporates:
  //   DiscreteIntegrator: '<S15>/Discrete-Time Integrator y'
  //   Product: '<S37>/MatrixMultiply2'
  //   Product: '<S3>/Matrix Multiply1'
  //   Sum: '<S3>/Add'
  //   UnitDelay: '<S37>/Unit Delay1'

  rtb_y_k_0[3] = rtDW.DiscreteTimeIntegratory_DSTAT_h - rtb_y_jr[2];
  for (i = 0; i < 4; i++) {
    rtb_y_k[i] = (((rtb_G2[i + 4] * rtDW.UnitDelay1_DSTATE[1] + rtb_G2[i] *
                    rtDW.UnitDelay1_DSTATE[0]) + rtb_G2[i + 8] *
                   rtDW.UnitDelay1_DSTATE[2]) + rtb_G2[i + 12] *
                  rtDW.UnitDelay1_DSTATE[3]) + rtb_y_k_0[i];
  }

  // MATLAB Function: '<S41>/MATLAB Function2' incorporates:
  //   DiscreteIntegrator: '<S44>/Discrete-Time Integrator'
  //   Gain: '<Root>/Gain5'
  //   Inport: '<Root>/cmd'
  //   Lookup_n-D: '<S37>/1-D Lookup Table1'
  //   MATLAB Function: '<S3>/MATLAB Function'

  for (i = 0; i < 16; i++) {
    rtb_G2[i] = (real32_T)rtConstP.MATLABFunction2_ca.W_v[i];
  }

  rtb_y_k_0[0] = 0.1F - rtDW.DiscreteTimeIntegrator_DSTATE[0];
  rtb_y_k_0[1] = 0.1F - rtDW.DiscreteTimeIntegrator_DSTATE[1];
  rtb_y_k_0[2] = 0.1F - rtDW.DiscreteTimeIntegrator_DSTATE[2];
  rtb_y_k_0[3] = 0.1F - rtDW.DiscreteTimeIntegrator_DSTATE[3];
  wls_alloc(tmp, rtb_y_k, rtb_y_a, rtb_q_red, rtb_G2,
            &rtConstP.MATLABFunction2_ca.W_u[0], rtb_y_k_0, 1000.0F +
            look1_iflf_binlx(0.5F * -rtU.cmd.thr + 0.5F,
             rtConstP.uDLookupTable1_bp01Data, rtConstP.uDLookupTable1_tableData,
             3U), rtb_uDLookupTable2, W, 100.0);

  // Sum: '<S10>/Add6' incorporates:
  //   DiscreteIntegrator: '<S39>/Discrete-Time Integrator y'

  rtb_yaw = rtb_uDLookupTable2[0] + rtDW.DiscreteTimeIntegratory_DSTAT_e[0];

  // Saturate: '<S10>/Saturation3'
  if (rtb_yaw > 1.0F) {
    rtb_yaw = 1.0F;
  } else {
    if (rtb_yaw < 0.1F) {
      rtb_yaw = 0.1F;
    }
  }

  // Sum: '<S10>/Add6' incorporates:
  //   DiscreteIntegrator: '<S39>/Discrete-Time Integrator y'

  py = rtb_uDLookupTable2[1] + rtDW.DiscreteTimeIntegratory_DSTAT_e[1];

  // Saturate: '<S10>/Saturation3'
  if (py > 1.0F) {
    py = 1.0F;
  } else {
    if (py < 0.1F) {
      py = 0.1F;
    }
  }

  // Sum: '<S10>/Add6' incorporates:
  //   DiscreteIntegrator: '<S39>/Discrete-Time Integrator y'

  xf = rtb_uDLookupTable2[2] + rtDW.DiscreteTimeIntegratory_DSTAT_e[2];

  // Saturate: '<S10>/Saturation3'
  if (xf > 1.0F) {
    xf = 1.0F;
  } else {
    if (xf < 0.1F) {
      xf = 0.1F;
    }
  }

  // Sum: '<S10>/Add6' incorporates:
  //   DiscreteIntegrator: '<S39>/Discrete-Time Integrator y'

  px = rtb_uDLookupTable2[3] + rtDW.DiscreteTimeIntegratory_DSTAT_e[3];

  // Saturate: '<S10>/Saturation3'
  if (px > 1.0F) {
    px = 1.0F;
  } else {
    if (px < 0.1F) {
      px = 0.1F;
    }
  }

  // Outport: '<Root>/logs' incorporates:
  //   Inport: '<Root>/measure'
  //   Saturate: '<S10>/Saturation3'

  rtY.logs[0] = py;
  rtY.logs[1] = px;
  rtY.logs[2] = rtb_yaw;
  rtY.logs[3] = xf;
  rtY.logs[4] = rtDW.pos_match_g[0];
  rtY.logs[7] = rtU.measure.s_Kg[0];
  rtY.logs[10] = rtU.measure.omega_Kb[0];
  rtY.logs[5] = rtDW.pos_match_g[1];
  rtY.logs[8] = rtU.measure.s_Kg[1];
  rtY.logs[11] = rtU.measure.omega_Kb[1];
  rtY.logs[6] = rtDW.pos_match_g[2];
  rtY.logs[9] = rtU.measure.s_Kg[2];
  rtY.logs[12] = rtU.measure.omega_Kb[2];
  rtY.logs[13] = rtb_Diff[0];
  rtY.logs[14] = rtb_Diff[1];

  // DiscreteIntegrator: '<S13>/Discrete-Time Integrator' incorporates:
  //   Inport: '<Root>/measure'
  //   Logic: '<S3>/Logical Operator'

  if (rtDW.DiscreteTimeIntegrator_IC_LOA_d != 0) {
    rtDW.DiscreteTimeIntegrator_DSTAT_n5[0] = rtU.measure.s_Kg[0];
    rtDW.DiscreteTimeIntegrator_DSTAT_n5[1] = rtU.measure.s_Kg[1];
    rtDW.DiscreteTimeIntegrator_DSTAT_n5[2] = rtU.measure.s_Kg[2];
  }

  if (rtb_Compare_f && (rtDW.DiscreteTimeIntegrator_PrevRe_g <= 0)) {
    rtDW.DiscreteTimeIntegrator_DSTAT_n5[0] = rtU.measure.s_Kg[0];
    rtDW.DiscreteTimeIntegrator_DSTAT_n5[1] = rtU.measure.s_Kg[1];
    rtDW.DiscreteTimeIntegrator_DSTAT_n5[2] = rtU.measure.s_Kg[2];
  }

  // Switch: '<S18>/Switch1' incorporates:
  //   DiscreteIntegrator: '<S13>/Discrete-Time Integrator'

  if (rtb_Compare > 0) {
    rtb_n_g_des_b = rtDW.s_g_ref[0];
  } else {
    rtb_n_g_des_b = rtDW.DiscreteTimeIntegrator_DSTAT_n5[0];
  }

  // Sum: '<S76>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S76>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S76>/Discrete-Time Integrator y_dt'
  //   Gain: '<S76>/2*d//omega'
  //   Sum: '<S76>/Sum3'

  rtb_Sum2_bj[0] = rtb_n_g_des_b - (0.181333333F *
    rtDW.DiscreteTimeIntegratory_dt_DS_j[0] +
    rtDW.DiscreteTimeIntegratory_DSTATE[0]);

  // Switch: '<S18>/Switch' incorporates:
  //   DiscreteIntegrator: '<S73>/Discrete-Time Integrator'

  if (rtb_Compare > 0) {
    rtb_n_g_des_b = rtDW.s_g_dt_ref[0];
  } else {
    rtb_n_g_des_b = rtDW.DiscreteTimeIntegrator_DSTATE_k[0];
  }

  // Sum: '<S76>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S76>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S76>/Discrete-Time Integrator y_dt'
  //   Gain: '<S76>/2*d//omega'
  //   Sum: '<S76>/Sum3'

  rtb_Sum2_bj[3] = rtb_n_g_des_b - (0.181333333F *
    rtDW.DiscreteTimeIntegratory_dt_DS_j[3] +
    rtDW.DiscreteTimeIntegratory_DSTATE[3]);
  rtb_Sum2_bj[6] = rtb_y_f - (0.181333333F *
    rtDW.DiscreteTimeIntegratory_dt_DS_j[6] +
    rtDW.DiscreteTimeIntegratory_DSTATE[6]);

  // Switch: '<S18>/Switch1' incorporates:
  //   DiscreteIntegrator: '<S13>/Discrete-Time Integrator'

  if (rtb_Compare > 0) {
    rtb_n_g_des_b = rtDW.s_g_ref[1];
  } else {
    rtb_n_g_des_b = rtDW.DiscreteTimeIntegrator_DSTAT_n5[1];
  }

  // Sum: '<S76>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S76>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S76>/Discrete-Time Integrator y_dt'
  //   Gain: '<S76>/2*d//omega'
  //   Sum: '<S76>/Sum3'

  rtb_Sum2_bj[1] = rtb_n_g_des_b - (0.181333333F *
    rtDW.DiscreteTimeIntegratory_dt_DS_j[1] +
    rtDW.DiscreteTimeIntegratory_DSTATE[1]);

  // Switch: '<S18>/Switch' incorporates:
  //   DiscreteIntegrator: '<S73>/Discrete-Time Integrator'

  if (rtb_Compare > 0) {
    rtb_n_g_des_b = rtDW.s_g_dt_ref[1];
  } else {
    rtb_n_g_des_b = rtDW.DiscreteTimeIntegrator_DSTATE_k[1];
  }

  // Sum: '<S76>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S76>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S76>/Discrete-Time Integrator y_dt'
  //   Gain: '<S76>/2*d//omega'
  //   Sum: '<S76>/Sum3'

  rtb_Sum2_bj[4] = rtb_n_g_des_b - (0.181333333F *
    rtDW.DiscreteTimeIntegratory_dt_DS_j[4] +
    rtDW.DiscreteTimeIntegratory_DSTATE[4]);
  rtb_Sum2_bj[7] = rtb_Switch2_idx_1 - (0.181333333F *
    rtDW.DiscreteTimeIntegratory_dt_DS_j[7] +
    rtDW.DiscreteTimeIntegratory_DSTATE[7]);

  // Switch: '<S18>/Switch1' incorporates:
  //   DiscreteIntegrator: '<S13>/Discrete-Time Integrator'

  if (rtb_Compare > 0) {
    rtb_n_g_des_b = rtDW.s_g_ref[2];
  } else {
    rtb_n_g_des_b = rtDW.DiscreteTimeIntegrator_DSTAT_n5[2];
  }

  // Sum: '<S76>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S76>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S76>/Discrete-Time Integrator y_dt'
  //   Gain: '<S76>/2*d//omega'
  //   Sum: '<S76>/Sum3'

  rtb_Sum2_bj[2] = rtb_n_g_des_b - (0.181333333F *
    rtDW.DiscreteTimeIntegratory_dt_DS_j[2] +
    rtDW.DiscreteTimeIntegratory_DSTATE[2]);

  // Switch: '<S18>/Switch' incorporates:
  //   DiscreteIntegrator: '<S73>/Discrete-Time Integrator'

  if (rtb_Compare > 0) {
    rtb_n_g_des_b = rtDW.s_g_dt_ref[2];
  } else {
    rtb_n_g_des_b = rtDW.DiscreteTimeIntegrator_DSTATE_k[2];
  }

  // Sum: '<S76>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S76>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S76>/Discrete-Time Integrator y_dt'
  //   Gain: '<S76>/2*d//omega'
  //   Sum: '<S76>/Sum3'

  rtb_Sum2_bj[5] = rtb_n_g_des_b - (0.181333333F *
    rtDW.DiscreteTimeIntegratory_dt_DS_j[5] +
    rtDW.DiscreteTimeIntegratory_DSTATE[5]);
  rtb_Sum2_bj[8] = rtb_Switch2_idx_2 - (0.181333333F *
    rtDW.DiscreteTimeIntegratory_dt_DS_j[8] +
    rtDW.DiscreteTimeIntegratory_DSTATE[8]);

  // Outport: '<Root>/u' incorporates:
  //   Gain: '<Root>/Gain1'
  //   Gain: '<Root>/Gain2'
  //   Gain: '<Root>/Gain3'
  //   Gain: '<Root>/Gain4'
  //   Saturate: '<S10>/Saturation3'

  rtY.u[0] = py;
  rtY.u[1] = px;
  rtY.u[2] = rtb_yaw;
  rtY.u[3] = xf;
  rtY.u[4] = 0.0F;
  rtY.u[5] = 0.0F;
  rtY.u[6] = 0.0F;
  rtY.u[7] = 0.0F;

  // Sum: '<S39>/Sum2' incorporates:
  //   Delay: '<S10>/Delay'
  //   DiscreteIntegrator: '<S39>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S39>/Discrete-Time Integrator y_dt'
  //   Gain: '<S39>/2*d//omega'
  //   Sum: '<S39>/Sum3'

  rtb_q_red[0] = rtDW.Delay_DSTATE[0] - (0.02F *
    rtDW.DiscreteTimeIntegratory_dt_DS_m[0] +
    rtDW.DiscreteTimeIntegratory_DSTAT_e[0]);
  rtb_q_red[1] = rtDW.Delay_DSTATE[1] - (0.02F *
    rtDW.DiscreteTimeIntegratory_dt_DS_m[1] +
    rtDW.DiscreteTimeIntegratory_DSTAT_e[1]);
  rtb_q_red[2] = rtDW.Delay_DSTATE[2] - (0.02F *
    rtDW.DiscreteTimeIntegratory_dt_DS_m[2] +
    rtDW.DiscreteTimeIntegratory_DSTAT_e[2]);
  rtb_q_red[3] = rtDW.Delay_DSTATE[3] - (0.02F *
    rtDW.DiscreteTimeIntegratory_dt_DS_m[3] +
    rtDW.DiscreteTimeIntegratory_DSTAT_e[3]);

  // Sum: '<S15>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S15>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S15>/Discrete-Time Integrator y_dt'
  //   Gain: '<S15>/2*d//omega'
  //   Sum: '<S15>/Sum3'

  rtb_y_f = dist_remaining - (0.13333334F * rtDW.DiscreteTimeIntegratory_dt_D_kp
    + rtDW.DiscreteTimeIntegratory_DSTAT_h);

  // Sum: '<S70>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y_dt'
  //   Gain: '<S70>/2*d//omega'
  //   Sum: '<S70>/Sum3'

  rtb_Diff[0] = rtb_n_g[0] - (0.048F * rtDW.DiscreteTimeIntegratory_dt_DS_l[0] +
    rtDW.DiscreteTimeIntegratory_DSTAT_p[0]);

  // Sum: '<S69>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S61>/Discrete-Time Integrator y_dt'
  //   DiscreteIntegrator: '<S69>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S69>/Discrete-Time Integrator y_dt'
  //   Gain: '<S69>/2*d//omega'
  //   Sum: '<S69>/Sum3'

  rtb_Diff_n[0] = rtDW.DiscreteTimeIntegratory_dt_DSTA[0] - (0.048F *
    rtDW.DiscreteTimeIntegratory_dt_DS_k[0] +
    rtDW.DiscreteTimeIntegratory_DSTAT_k[0]);

  // Update for UnitDelay: '<S8>/UD'
  //
  //  Block description for '<S8>/UD':
  //
  //   Store in Global RAM

  rtDW.UD_DSTATE[0] = rtb_TSamp[0];

  // Sum: '<S68>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S68>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S68>/Discrete-Time Integrator y_dt'
  //   Gain: '<S68>/2*d//omega'
  //   Sum: '<S68>/Sum3'

  rtb_n[0] -= 0.048F * rtDW.DiscreteTimeIntegratory_dt_D_kt[0] +
    rtDW.DiscreteTimeIntegratory_DSTAT_f[0];

  // Sum: '<S70>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y_dt'
  //   Gain: '<S70>/2*d//omega'
  //   Sum: '<S70>/Sum3'

  rtb_Diff[1] = rtb_n_g[1] - (0.048F * rtDW.DiscreteTimeIntegratory_dt_DS_l[1] +
    rtDW.DiscreteTimeIntegratory_DSTAT_p[1]);

  // Sum: '<S69>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S61>/Discrete-Time Integrator y_dt'
  //   DiscreteIntegrator: '<S69>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S69>/Discrete-Time Integrator y_dt'
  //   Gain: '<S69>/2*d//omega'
  //   Sum: '<S69>/Sum3'

  rtb_Diff_n[1] = rtDW.DiscreteTimeIntegratory_dt_DSTA[1] - (0.048F *
    rtDW.DiscreteTimeIntegratory_dt_DS_k[1] +
    rtDW.DiscreteTimeIntegratory_DSTAT_k[1]);

  // Update for UnitDelay: '<S8>/UD'
  //
  //  Block description for '<S8>/UD':
  //
  //   Store in Global RAM

  rtDW.UD_DSTATE[1] = rtb_TSamp[1];

  // Sum: '<S68>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S68>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S68>/Discrete-Time Integrator y_dt'
  //   Gain: '<S68>/2*d//omega'
  //   Sum: '<S68>/Sum3'

  rtb_n[1] -= 0.048F * rtDW.DiscreteTimeIntegratory_dt_D_kt[1] +
    rtDW.DiscreteTimeIntegratory_DSTAT_f[1];

  // Sum: '<S70>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y_dt'
  //   Gain: '<S70>/2*d//omega'
  //   Sum: '<S70>/Sum3'

  rtb_Diff[2] = rtb_n_g_c - (0.048F * rtDW.DiscreteTimeIntegratory_dt_DS_l[2] +
    rtDW.DiscreteTimeIntegratory_DSTAT_p[2]);

  // Sum: '<S69>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S61>/Discrete-Time Integrator y_dt'
  //   DiscreteIntegrator: '<S69>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S69>/Discrete-Time Integrator y_dt'
  //   Gain: '<S69>/2*d//omega'
  //   Sum: '<S69>/Sum3'

  rtb_Diff_n[2] = rtDW.DiscreteTimeIntegratory_dt_DSTA[2] - (0.048F *
    rtDW.DiscreteTimeIntegratory_dt_DS_k[2] +
    rtDW.DiscreteTimeIntegratory_DSTAT_k[2]);

  // Sum: '<S68>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S68>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S68>/Discrete-Time Integrator y_dt'
  //   Gain: '<S68>/2*d//omega'
  //   Sum: '<S68>/Sum3'

  dist_remaining = rtb_n[2] - (0.048F * rtDW.DiscreteTimeIntegratory_dt_D_kt[2]
    + rtDW.DiscreteTimeIntegratory_DSTAT_f[2]);

  // Update for UnitDelay: '<S8>/UD'
  //
  //  Block description for '<S8>/UD':
  //
  //   Store in Global RAM

  rtDW.UD_DSTATE[2] = u;

  // Update for DiscreteIntegrator: '<S44>/Discrete-Time Integrator' incorporates:
  //   Gain: '<S44>/1//T'
  //   Saturate: '<S10>/Saturation3'
  //   Sum: '<S44>/Sum2'

  rtDW.DiscreteTimeIntegrator_DSTATE[0] += (rtb_yaw -
    rtDW.DiscreteTimeIntegrator_DSTATE[0]) * 35.7142868F * 0.0025F;
  rtDW.DiscreteTimeIntegrator_DSTATE[1] += (py -
    rtDW.DiscreteTimeIntegrator_DSTATE[1]) * 35.7142868F * 0.0025F;
  rtDW.DiscreteTimeIntegrator_DSTATE[2] += (xf -
    rtDW.DiscreteTimeIntegrator_DSTATE[2]) * 35.7142868F * 0.0025F;
  rtDW.DiscreteTimeIntegrator_DSTATE[3] += (px -
    rtDW.DiscreteTimeIntegrator_DSTATE[3]) * 35.7142868F * 0.0025F;

  // Update for DiscreteIntegrator: '<S67>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_DSTATE_n += 0.0025F * rtb_uT;

  // Update for Memory: '<S16>/Memory'
  rtDW.Memory_PreviousInput = rtb_Compare_f;

  // Update for DiscreteIntegrator: '<S73>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_IC_LOADI = 0U;

  // Update for UnitDelay: '<S7>/UD'
  //
  //  Block description for '<S7>/UD':
  //
  //   Store in Global RAM

  rtDW.UD_DSTATE_o[0] = rtb_TSamp_k[0];

  // Update for DiscreteIntegrator: '<S73>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_DSTATE_k[0] += 0.0025F * q0_q3;

  // Update for UnitDelay: '<S7>/UD'
  //
  //  Block description for '<S7>/UD':
  //
  //   Store in Global RAM

  rtDW.UD_DSTATE_o[1] = rtb_TSamp_k[1];

  // Update for DiscreteIntegrator: '<S73>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_DSTATE_k[1] += 0.0025F * q1_q2;

  // Update for UnitDelay: '<S7>/UD'
  //
  //  Block description for '<S7>/UD':
  //
  //   Store in Global RAM

  rtDW.UD_DSTATE_o[2] = yf;

  // Update for DiscreteIntegrator: '<S73>/Discrete-Time Integrator' incorporates:
  //   Logic: '<S3>/Logical Operator'

  rtDW.DiscreteTimeIntegrator_DSTATE_k[2] += 0.0025F * rtb_y_dt_idx_2;
  rtDW.DiscreteTimeIntegrator_PrevRese = (int8_T)rtb_Compare_f;

  // Update for DiscreteIntegrator: '<S61>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LOAD = 0U;

  // Update for DiscreteIntegrator: '<S68>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LO_f = 0U;

  // Update for DiscreteIntegrator: '<S69>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LO_b = 0U;

  // Update for DiscreteIntegrator: '<S70>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LO_e = 0U;

  // Update for DiscreteIntegrator: '<S61>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S61>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_a[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DSTA[0];

  // Update for DiscreteIntegrator: '<S61>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DSTA[0] += 0.0025F * rtb_n_g[0];

  // Update for DiscreteIntegrator: '<S68>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S68>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_f[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_D_kt[0];

  // Update for DiscreteIntegrator: '<S69>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S69>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_k[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_k[0];

  // Update for DiscreteIntegrator: '<S70>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_p[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_l[0];

  // Update for DiscreteIntegrator: '<S61>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S61>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_a[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DSTA[1];

  // Update for DiscreteIntegrator: '<S61>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DSTA[1] += 0.0025F * rtb_n_g[1];

  // Update for DiscreteIntegrator: '<S68>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S68>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_f[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_D_kt[1];

  // Update for DiscreteIntegrator: '<S69>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S69>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_k[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_k[1];

  // Update for DiscreteIntegrator: '<S70>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_p[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_l[1];

  // Update for DiscreteIntegrator: '<S61>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S61>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_a[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DSTA[2];

  // Update for DiscreteIntegrator: '<S61>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DSTA[2] += 0.0025F * rtb_n_g_c;

  // Update for DiscreteIntegrator: '<S68>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S68>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_f[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_D_kt[2];

  // Update for DiscreteIntegrator: '<S69>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S69>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_k[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_k[2];

  // Update for DiscreteIntegrator: '<S70>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_p[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_l[2];

  // Update for DiscreteIntegrator: '<S51>/Discrete-Time Integrator2'
  rtDW.DiscreteTimeIntegrator2_IC_LOAD = 0U;
  rtDW.DiscreteTimeIntegrator2_DSTATE += 0.0025F * rtb_Sqrt;

  // Update for DiscreteIntegrator: '<S15>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S15>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_IC_LO_g = 0U;
  rtDW.DiscreteTimeIntegratory_DSTAT_h += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_D_kp;

  // Update for UnitDelay: '<S37>/Unit Delay1'
  rtDW.UnitDelay1_DSTATE[0] = rtb_uDLookupTable2[0];

  // Update for DiscreteIntegrator: '<S39>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S39>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_e[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_m[0];

  // Update for UnitDelay: '<S37>/Unit Delay1'
  rtDW.UnitDelay1_DSTATE[1] = rtb_uDLookupTable2[1];

  // Update for DiscreteIntegrator: '<S39>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S39>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_e[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_m[1];

  // Update for UnitDelay: '<S37>/Unit Delay1'
  rtDW.UnitDelay1_DSTATE[2] = rtb_uDLookupTable2[2];

  // Update for DiscreteIntegrator: '<S39>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S39>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_e[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_m[2];

  // Update for UnitDelay: '<S37>/Unit Delay1'
  rtDW.UnitDelay1_DSTATE[3] = rtb_uDLookupTable2[3];

  // Update for DiscreteIntegrator: '<S39>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S39>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_e[3] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_m[3];
  for (i = 0; i < 9; i++) {
    // Update for DiscreteIntegrator: '<S76>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S76>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTATE[i] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_j[i];

    // Update for DiscreteIntegrator: '<S76>/Discrete-Time Integrator y_dt' incorporates:
    //   Gain: '<S76>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_DS_j[i] += 121.647926F * rtb_Sum2_bj[i] *
      0.0025F;
  }

  // Update for DiscreteIntegrator: '<S13>/Discrete-Time Integrator' incorporates:
  //   Logic: '<S3>/Logical Operator'

  rtDW.DiscreteTimeIntegrator_IC_LOA_d = 0U;
  rtDW.DiscreteTimeIntegrator_DSTAT_n5[0] += 0.0025F * zf;
  rtDW.DiscreteTimeIntegrator_DSTAT_n5[1] += 0.0025F * q1_q3;
  rtDW.DiscreteTimeIntegrator_DSTAT_n5[2] += 0.0025F * q2_q3;
  rtDW.DiscreteTimeIntegrator_PrevRe_g = (int8_T)rtb_Compare_f;

  // Update for DiscreteIntegrator: '<S39>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S39>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_m[0] += 10000.0F * rtb_q_red[0] * 0.0025F;

  // Update for Delay: '<S10>/Delay'
  rtDW.Delay_DSTATE[0] = rtb_y_idx_0;

  // Update for DiscreteIntegrator: '<S39>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S39>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_m[1] += 10000.0F * rtb_q_red[1] * 0.0025F;

  // Update for Delay: '<S10>/Delay'
  rtDW.Delay_DSTATE[1] = rtb_y_idx_1;

  // Update for DiscreteIntegrator: '<S39>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S39>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_m[2] += 10000.0F * rtb_q_red[2] * 0.0025F;

  // Update for Delay: '<S10>/Delay'
  rtDW.Delay_DSTATE[2] = rtb_y_idx_2;

  // Update for DiscreteIntegrator: '<S39>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S39>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_m[3] += 10000.0F * rtb_q_red[3] * 0.0025F;

  // Update for Delay: '<S10>/Delay'
  rtDW.Delay_DSTATE[3] = rtb_y_idx_3;

  // Update for DiscreteIntegrator: '<S70>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S70>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_l[0] += 1736.11108F * rtb_Diff[0] * 0.0025F;

  // Update for DiscreteIntegrator: '<S69>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S69>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_k[0] += 1736.11108F * rtb_Diff_n[0] *
    0.0025F;

  // Update for DiscreteIntegrator: '<S68>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S68>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_D_kt[0] += 1736.11108F * rtb_n[0] * 0.0025F;

  // Update for DiscreteIntegrator: '<S70>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S70>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_l[1] += 1736.11108F * rtb_Diff[1] * 0.0025F;

  // Update for DiscreteIntegrator: '<S69>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S69>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_k[1] += 1736.11108F * rtb_Diff_n[1] *
    0.0025F;

  // Update for DiscreteIntegrator: '<S68>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S68>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_D_kt[1] += 1736.11108F * rtb_n[1] * 0.0025F;

  // Update for DiscreteIntegrator: '<S70>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S70>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_l[2] += 1736.11108F * rtb_Diff[2] * 0.0025F;

  // Update for DiscreteIntegrator: '<S69>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S69>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_k[2] += 1736.11108F * rtb_Diff_n[2] *
    0.0025F;

  // Update for DiscreteIntegrator: '<S68>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S68>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_D_kt[2] += 1736.11108F * dist_remaining *
    0.0025F;

  // Update for DiscreteIntegrator: '<S15>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S15>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_D_kp += 225.0F * rtb_y_f * 0.0025F;
}

// Model initialize function
void MatlabControllerClass::initialize()
{
  // InitializeConditions for DiscreteIntegrator: '<S73>/Discrete-Time Integrator' 
  rtDW.DiscreteTimeIntegrator_IC_LOADI = 1U;
  rtDW.DiscreteTimeIntegrator_PrevRese = 2;

  // InitializeConditions for DiscreteIntegrator: '<S61>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LOAD = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S68>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LO_f = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S69>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LO_b = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S70>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LO_e = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S51>/Discrete-Time Integrator2' 
  rtDW.DiscreteTimeIntegrator2_IC_LOAD = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S15>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LO_g = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S13>/Discrete-Time Integrator' 
  rtDW.DiscreteTimeIntegrator_IC_LOA_d = 1U;
  rtDW.DiscreteTimeIntegrator_PrevRe_g = 2;
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
