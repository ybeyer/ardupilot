//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: MatlabController.cpp
//
// Code generated for Simulink model 'ArduCopter_MinnieLoiterFtc'.
//
// Model version                  : 1.395
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Fri Apr  8 17:15:03 2022
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
//    '<S36>/wrap angle'
//    '<S36>/wrap angle1'
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
//    '<S37>/DCM to quaternions'
//    '<S21>/DCM to quaternions'
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

// Function for MATLAB Function: '<S37>/Quaternion Reduced'
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

// Function for MATLAB Function: '<S37>/Quaternion Reduced'
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
//    '<S37>/Quaternion Reduced'
//    '<S21>/Quaternion Reduced'
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
//    '<S41>/Lean Vector Derivative Trafo'
//    '<S41>/Lean Vector Derivative Trafo Delay'
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

// Function for MATLAB Function: '<S10>/INDI Copter Acc 2 Lean Vector'
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

// Function for MATLAB Function: '<S29>/MATLAB Function2'
void MatlabControllerClass::LSQFromQR(const real32_T A_data[], const int32_T
  A_size[2], const real32_T tau_data[], const int32_T jpvt_data[], real32_T B_3
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
      wj = B_3[b_j];
      for (loop_ub = b_j + 1; loop_ub + 1 < 9; loop_ub++) {
        wj += A_data[(b_j << 3) + loop_ub] * B_3[loop_ub];
      }

      wj *= tau_data[b_j];
      if (wj != 0.0F) {
        B_3[b_j] -= wj;
        for (loop_ub = b_j + 1; loop_ub + 1 < 9; loop_ub++) {
          B_3[loop_ub] -= A_data[(b_j << 3) + loop_ub] * wj;
        }
      }
    }
  }

  for (loop_ub = 0; loop_ub < rankA; loop_ub++) {
    Y_data[jpvt_data[loop_ub] - 1] = B_3[loop_ub];
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

// Function for MATLAB Function: '<S29>/MATLAB Function2'
real32_T MatlabControllerClass::xnrm2(int32_T n, const real32_T x_data[],
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

// Function for MATLAB Function: '<S29>/MATLAB Function2'
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

// Function for MATLAB Function: '<S29>/MATLAB Function2'
void MatlabControllerClass::qrsolve(const real32_T A_data[], const int32_T
  A_size[2], const real32_T B_1[8], real32_T Y_data[], int32_T *Y_size)
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
  real32_T B_2[8];
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
      s = xnrm2(7 - b_n, b_A_data, yk + 2);
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

          s = rt_hypotf(smax, xnrm2(7 - b_n, b_A_data, yk + 2));
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
            vn1_data[yk] = xnrm2(7 - b_n, b_A_data, nmi + 2);
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
    B_2[b_ix] = B_1[b_ix];
  }

  LSQFromQR(b_A_data, b_A_size, tau_data, jpvt_data, B_2, n, Y_data, Y_size);
}

// Function for MATLAB Function: '<S29>/MATLAB Function2'
void MatlabControllerClass::mldivide(const real32_T A_data[], const int32_T
  A_size[2], const real32_T B_0[8], real32_T Y_data[], int32_T *Y_size)
{
  if (A_size[1] == 0) {
    *Y_size = 0;
  } else {
    qrsolve(A_data, A_size, B_0, Y_data, Y_size);
  }
}

// Function for MATLAB Function: '<S29>/MATLAB Function2'
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

// Function for MATLAB Function: '<S29>/MATLAB Function2'
real_T MatlabControllerClass::wls_alloc(const real32_T B_4[16], const real32_T
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
      A_tmp_0[A_tmp_tmp_0] = A_tmp_0[A_tmp_tmp_1] + B_4[A_tmp_tmp] *
        A_tmp[aoffset];
      A_tmp_0[A_tmp_tmp_0] = B_4[A_tmp_tmp + 1] * A_tmp[aoffset + 4] +
        A_tmp_0[A_tmp_tmp_1];
      A_tmp_0[A_tmp_tmp_0] = B_4[A_tmp_tmp + 2] * A_tmp[aoffset + 8] +
        A_tmp_0[A_tmp_tmp_1];
      A_tmp_0[A_tmp_tmp_0] = B_4[A_tmp_tmp + 3] * A_tmp[aoffset + 12] +
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

    mldivide(A_free_data, A_free_size, d, p_free_data, &b_aoffset);
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

// Model step function
void MatlabControllerClass::step()
{
  real32_T d[16];
  real32_T q0_q0;
  real32_T q1_q1;
  real32_T q2_q2;
  real32_T q3_q3;
  real32_T q0_q1;
  real32_T q0_q2;
  real32_T q2_q3;
  real32_T scale;
  real32_T absxk;
  real32_T t;
  boolean_T varargin_1[10];
  int32_T idx;
  real_T W[4];
  real32_T d_0[16];
  real32_T rtb_Switch[4];
  real32_T rtb_Diff_c[3];
  real32_T rtb_Delta_ny[3];
  real32_T rtb_nu[3];
  real32_T rtb_Diff[3];
  real32_T rtb_y_b[4];
  real32_T rtb_n_g_des[3];
  real32_T rtb_n_b_dt[3];
  real32_T rtb_n_b_dt2[3];
  real32_T rtb_uDLookupTable2[4];
  real32_T rtb_G2[16];
  real32_T rtb_M_bg[9];
  real32_T rtb_omega2[9];
  real32_T rtb_y_dt_gf[3];
  real32_T rtb_n[3];
  real32_T rtb_y_j3[3];
  real32_T rtb_y_p[3];
  real32_T rtb_q_red_p[4];
  int32_T i;
  real32_T tmp[16];
  real32_T rtb_Delta_ny_0[4];
  real32_T rtb_TSamp;
  real32_T rtb_Diff_l;
  real32_T rtb_TSamp_h;
  real32_T rtb_y_idx_3;
  real32_T rtb_y_idx_2;
  real32_T rtb_y_idx_1;
  real32_T rtb_y_idx_0;
  real32_T rtb_y_c_idx_2;
  real32_T cmd_V_NED_idx_2;
  real32_T rtb_TSamp_k_idx_1;
  real32_T rtb_TSamp_idx_1;
  real32_T rtb_y_c_idx_1;
  real32_T cmd_V_NED_idx_1;
  real32_T rtb_TSamp_k_idx_0;
  real32_T rtb_TSamp_idx_0;
  real32_T rtb_y_c_idx_0;
  real32_T cmd_V_NED_idx_0;
  boolean_T rtb_LowerRelop1_idx_1;
  boolean_T rtb_LowerRelop1_idx_0;
  real32_T rtb_Add_i_idx_7;
  real32_T rtb_Add_i_idx_0;
  real_T y;
  int32_T tmp_0;

  // DiscreteIntegrator: '<S32>/Discrete-Time Integrator'
  rtb_y_idx_0 = rtDW.DiscreteTimeIntegrator_DSTATE[0];
  rtb_y_idx_1 = rtDW.DiscreteTimeIntegrator_DSTATE[1];
  rtb_y_idx_2 = rtDW.DiscreteTimeIntegrator_DSTATE[2];
  rtb_y_idx_3 = rtDW.DiscreteTimeIntegrator_DSTATE[3];

  // MATLAB Function: '<S28>/MATLAB Function' incorporates:
  //   Constant: '<S28>/Constant4'
  //   Constant: '<S28>/ny_du_dt'

  for (i = 0; i < 16; i++) {
    rtb_G2[i] = rtConstP.ny_du_dt_Value[i] / 0.0025F;
    d[i] = 0.0F;
  }

  // MATLAB Function: '<Root>/Quaternions to Rotation Matrix'
  scale = 1.29246971E-26F;

  // MinMax: '<S28>/Max' incorporates:
  //   DiscreteIntegrator: '<S32>/Discrete-Time Integrator'

  if (rtDW.DiscreteTimeIntegrator_DSTATE[0] > 0.1F) {
    q2_q2 = rtDW.DiscreteTimeIntegrator_DSTATE[0];
  } else {
    q2_q2 = 0.1F;
  }

  // MATLAB Function: '<S28>/MATLAB Function'
  d[0] = q2_q2 / 0.283F;

  // MATLAB Function: '<Root>/Quaternions to Rotation Matrix' incorporates:
  //   Inport: '<Root>/measure'

  absxk = std::abs(rtU.measure.q_bg[0]);
  if (absxk > 1.29246971E-26F) {
    q2_q3 = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    q2_q3 = t * t;
  }

  // MinMax: '<S28>/Max' incorporates:
  //   DiscreteIntegrator: '<S32>/Discrete-Time Integrator'

  if (rtDW.DiscreteTimeIntegrator_DSTATE[1] > 0.1F) {
    q2_q2 = rtDW.DiscreteTimeIntegrator_DSTATE[1];
  } else {
    q2_q2 = 0.1F;
  }

  // MATLAB Function: '<S28>/MATLAB Function'
  d[5] = q2_q2 / 0.283F;

  // MATLAB Function: '<Root>/Quaternions to Rotation Matrix' incorporates:
  //   Inport: '<Root>/measure'

  absxk = std::abs(rtU.measure.q_bg[1]);
  if (absxk > scale) {
    t = scale / absxk;
    q2_q3 = q2_q3 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q2_q3 += t * t;
  }

  // MinMax: '<S28>/Max' incorporates:
  //   DiscreteIntegrator: '<S32>/Discrete-Time Integrator'

  if (rtDW.DiscreteTimeIntegrator_DSTATE[2] > 0.1F) {
    q2_q2 = rtDW.DiscreteTimeIntegrator_DSTATE[2];
  } else {
    q2_q2 = 0.1F;
  }

  // MATLAB Function: '<S28>/MATLAB Function'
  d[10] = q2_q2 / 0.283F;

  // MATLAB Function: '<Root>/Quaternions to Rotation Matrix' incorporates:
  //   Inport: '<Root>/measure'

  absxk = std::abs(rtU.measure.q_bg[2]);
  if (absxk > scale) {
    t = scale / absxk;
    q2_q3 = q2_q3 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q2_q3 += t * t;
  }

  // MinMax: '<S28>/Max' incorporates:
  //   DiscreteIntegrator: '<S32>/Discrete-Time Integrator'

  if (rtDW.DiscreteTimeIntegrator_DSTATE[3] > 0.1F) {
    q2_q2 = rtDW.DiscreteTimeIntegrator_DSTATE[3];
  } else {
    q2_q2 = 0.1F;
  }

  // MATLAB Function: '<S28>/MATLAB Function'
  d[15] = q2_q2 / 0.283F;

  // MATLAB Function: '<Root>/Quaternions to Rotation Matrix' incorporates:
  //   Inport: '<Root>/measure'

  absxk = std::abs(rtU.measure.q_bg[3]);
  if (absxk > scale) {
    t = scale / absxk;
    q2_q3 = q2_q3 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q2_q3 += t * t;
  }

  q2_q3 = scale * std::sqrt(q2_q3);
  if (2.22044605E-16F < q2_q3) {
    q0_q0 = q2_q3;
  } else {
    q0_q0 = 2.22044605E-16F;
  }

  rtb_y_b[0] = rtU.measure.q_bg[0] / q0_q0;
  rtb_y_b[1] = rtU.measure.q_bg[1] / q0_q0;
  rtb_y_b[2] = rtU.measure.q_bg[2] / q0_q0;
  rtb_y_b[3] = rtU.measure.q_bg[3] / q0_q0;
  q0_q0 = rtb_y_b[0] * rtb_y_b[0];
  q1_q1 = rtb_y_b[1] * rtb_y_b[1];
  q2_q2 = rtb_y_b[2] * rtb_y_b[2];
  q3_q3 = rtb_y_b[3] * rtb_y_b[3];
  q0_q1 = rtb_y_b[0] * rtb_y_b[1];
  q0_q2 = rtb_y_b[0] * rtb_y_b[2];
  scale = rtb_y_b[0] * rtb_y_b[3];
  absxk = rtb_y_b[1] * rtb_y_b[2];
  t = rtb_y_b[1] * rtb_y_b[3];
  q2_q3 = rtb_y_b[2] * rtb_y_b[3];
  rtb_M_bg[0] = ((q0_q0 + q1_q1) - q2_q2) - q3_q3;
  rtb_M_bg[3] = (absxk + scale) * 2.0F;
  rtb_M_bg[6] = (t - q0_q2) * 2.0F;
  rtb_M_bg[1] = (absxk - scale) * 2.0F;
  q0_q0 -= q1_q1;
  rtb_M_bg[4] = (q0_q0 + q2_q2) - q3_q3;
  rtb_M_bg[7] = (q2_q3 + q0_q1) * 2.0F;
  rtb_M_bg[2] = (t + q0_q2) * 2.0F;
  rtb_M_bg[5] = (q2_q3 - q0_q1) * 2.0F;
  rtb_M_bg[8] = (q0_q0 - q2_q2) + q3_q3;

  // DiscreteIntegrator: '<S65>/Discrete-Time Integrator'
  rtb_y_c_idx_0 = rtDW.DiscreteTimeIntegrator_DSTATE_m[0];
  rtb_y_c_idx_1 = rtDW.DiscreteTimeIntegrator_DSTATE_m[1];
  rtb_y_c_idx_2 = rtDW.DiscreteTimeIntegrator_DSTATE_m[2];

  // Lookup_n-D: '<Root>/1-D Lookup Table' incorporates:
  //   Inport: '<Root>/cmd'

  q0_q0 = look1_iflf_binlx(rtU.cmd.RC_pwm[8], rtConstP.pooled7, rtConstP.pooled6,
    1U);

  // DiscreteIntegrator: '<S7>/Discrete-Time Integrator'
  if ((q0_q0 > 0.0F) && (rtDW.DiscreteTimeIntegrator_PrevRese <= 0)) {
    rtDW.DiscreteTimeIntegrator_DSTAT_mg = 0.0F;
  }

  // MATLAB Function: '<S7>/interpHold' incorporates:
  //   Constant: '<S7>/Constant1'
  //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator'

  for (i = 0; i < 10; i++) {
    varargin_1[i] = (rtConstP.Constant1_Value[i] >
                     rtDW.DiscreteTimeIntegrator_DSTAT_mg);
  }

  idx = -1;
  rtb_LowerRelop1_idx_0 = varargin_1[0];
  for (i = 0; i < 9; i++) {
    rtb_LowerRelop1_idx_1 = varargin_1[i + 1];
    if ((int32_T)rtb_LowerRelop1_idx_0 < (int32_T)rtb_LowerRelop1_idx_1) {
      rtb_LowerRelop1_idx_0 = rtb_LowerRelop1_idx_1;
      idx = i;
    }
  }

  // Switch: '<S7>/Switch' incorporates:
  //   Constant: '<S7>/Constant2'
  //   Inport: '<Root>/cmd'
  //   MATLAB Function: '<S7>/interpHold'

  if (q0_q0 > 0.5F) {
    // MATLAB Function: '<S7>/interpHold'
    if (1.0 > ((real_T)idx + 2.0) - 1.0) {
      y = 1.0;
    } else {
      y = ((real_T)idx + 2.0) - 1.0;
    }

    rtb_Switch[0] = rtConstP.Constant2_Value[(int32_T)y - 1];
    rtb_Switch[1] = rtConstP.Constant2_Value[(int32_T)y + 9];
    rtb_Switch[2] = rtConstP.Constant2_Value[(int32_T)y + 19];
    rtb_Switch[3] = rtConstP.Constant2_Value[(int32_T)y + 29];
  } else {
    rtb_Switch[0] = rtU.cmd.roll;
    rtb_Switch[1] = rtU.cmd.pitch;
    rtb_Switch[2] = rtU.cmd.yaw;
    rtb_Switch[3] = rtU.cmd.thr;
  }

  // End of Switch: '<S7>/Switch'

  // MATLAB Function: '<S20>/MATLAB Function'
  q3_q3 = rtb_Switch[0];

  // Gain: '<S61>/Gain3' incorporates:
  //   Gain: '<Root>/Gain'
  //   MATLAB Function: '<S20>/MATLAB Function'
  //   MATLAB Function: '<S61>/MATLAB Function1'

  q0_q2 = std::sqrt(rtb_Switch[0] * rtb_Switch[0] + -rtb_Switch[1] *
                    -rtb_Switch[1]) * 15.0F;

  // Saturate: '<S65>/Saturation' incorporates:
  //   DiscreteIntegrator: '<S65>/Discrete-Time Integrator'
  //   Gain: '<Root>/Gain'
  //   Gain: '<Root>/Gain5'
  //   Gain: '<S65>/1//T'
  //   Lookup_n-D: '<S61>/1-D Lookup Table'
  //   MATLAB Function: '<S20>/MATLAB Function'
  //   Product: '<S61>/Product'
  //   Sum: '<S65>/Sum2'

  cmd_V_NED_idx_0 = (-rtb_Switch[1] * q0_q2 -
                     rtDW.DiscreteTimeIntegrator_DSTATE_m[0]) * 1.66666663F;
  if (cmd_V_NED_idx_0 > 12.0F) {
    cmd_V_NED_idx_0 = 12.0F;
  } else {
    if (cmd_V_NED_idx_0 < -12.0F) {
      cmd_V_NED_idx_0 = -12.0F;
    }
  }

  cmd_V_NED_idx_1 = (rtb_Switch[0] * q0_q2 -
                     rtDW.DiscreteTimeIntegrator_DSTATE_m[1]) * 1.66666663F;
  if (cmd_V_NED_idx_1 > 12.0F) {
    cmd_V_NED_idx_1 = 12.0F;
  } else {
    if (cmd_V_NED_idx_1 < -12.0F) {
      cmd_V_NED_idx_1 = -12.0F;
    }
  }

  cmd_V_NED_idx_2 = (look1_iflf_binlx(-rtb_Switch[3],
    rtConstP.uDLookupTable_bp01Data, rtConstP.uDLookupTable_tableData, 2U) -
                     rtDW.DiscreteTimeIntegrator_DSTATE_m[2]) * 1.66666663F;
  if (cmd_V_NED_idx_2 > 4.0F) {
    cmd_V_NED_idx_2 = 4.0F;
  } else {
    if (cmd_V_NED_idx_2 < -25.0F) {
      cmd_V_NED_idx_2 = -25.0F;
    }
  }

  // End of Saturate: '<S65>/Saturation'

  // DiscreteIntegrator: '<S61>/Discrete-Time Integrator' incorporates:
  //   Inport: '<Root>/measure'

  if (rtDW.DiscreteTimeIntegrator_IC_LOADI != 0) {
    rtDW.DiscreteTimeIntegrator_DSTATE_c[0] = rtU.measure.s_Kg[0];
    rtDW.DiscreteTimeIntegrator_DSTATE_c[1] = rtU.measure.s_Kg[1];
    rtDW.DiscreteTimeIntegrator_DSTATE_c[2] = rtU.measure.s_Kg[2];
  }

  // Gain: '<S68>/omega^2' incorporates:
  //   DiscreteIntegrator: '<S61>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<S65>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<S68>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S68>/Discrete-Time Integrator y_dt'
  //   Gain: '<S68>/2*d//omega'
  //   Sum: '<S68>/Sum2'
  //   Sum: '<S68>/Sum3'

  rtb_omega2[0] = (rtDW.DiscreteTimeIntegrator_DSTATE_c[0] - (0.165312201F *
    rtDW.DiscreteTimeIntegratory_dt_DSTA[0] +
    rtDW.DiscreteTimeIntegratory_DSTATE[0])) * 146.369354F;
  rtb_omega2[3] = (rtDW.DiscreteTimeIntegrator_DSTATE_m[0] - (0.165312201F *
    rtDW.DiscreteTimeIntegratory_dt_DSTA[3] +
    rtDW.DiscreteTimeIntegratory_DSTATE[3])) * 146.369354F;
  rtb_omega2[6] = (cmd_V_NED_idx_0 - (0.165312201F *
    rtDW.DiscreteTimeIntegratory_dt_DSTA[6] +
    rtDW.DiscreteTimeIntegratory_DSTATE[6])) * 146.369354F;

  // SampleTimeMath: '<S11>/TSamp' incorporates:
  //   Inport: '<Root>/measure'
  //
  //  About '<S11>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  rtb_TSamp = rtU.measure.V_Kg[0] * 400.0F;

  // Sum: '<S11>/Diff' incorporates:
  //   UnitDelay: '<S11>/UD'
  //
  //  Block description for '<S11>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S11>/UD':
  //
  //   Store in Global RAM

  rtb_Diff_l = rtb_TSamp - rtDW.UD_DSTATE[0];

  // Sum: '<S67>/Add' incorporates:
  //   DiscreteIntegrator: '<S68>/Discrete-Time Integrator y'
  //   Inport: '<Root>/measure'

  rtb_Add_i_idx_0 = rtDW.DiscreteTimeIntegratory_DSTATE[0] - rtU.measure.s_Kg[0];
  q2_q3 = rtDW.DiscreteTimeIntegratory_DSTATE[6] - rtb_Diff_l;

  // SampleTimeMath: '<S11>/TSamp'
  //
  //  About '<S11>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  rtb_TSamp_idx_0 = rtb_TSamp;

  // Sum: '<S11>/Diff'
  //
  //  Block description for '<S11>/Diff':
  //
  //   Add in CPU

  rtb_Diff[0] = rtb_Diff_l;

  // Gain: '<S68>/omega^2' incorporates:
  //   DiscreteIntegrator: '<S61>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<S65>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<S68>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S68>/Discrete-Time Integrator y_dt'
  //   Gain: '<S68>/2*d//omega'
  //   Sum: '<S68>/Sum2'
  //   Sum: '<S68>/Sum3'

  rtb_omega2[1] = (rtDW.DiscreteTimeIntegrator_DSTATE_c[1] - (0.165312201F *
    rtDW.DiscreteTimeIntegratory_dt_DSTA[1] +
    rtDW.DiscreteTimeIntegratory_DSTATE[1])) * 146.369354F;
  rtb_omega2[4] = (rtDW.DiscreteTimeIntegrator_DSTATE_m[1] - (0.165312201F *
    rtDW.DiscreteTimeIntegratory_dt_DSTA[4] +
    rtDW.DiscreteTimeIntegratory_DSTATE[4])) * 146.369354F;
  rtb_omega2[7] = (cmd_V_NED_idx_1 - (0.165312201F *
    rtDW.DiscreteTimeIntegratory_dt_DSTA[7] +
    rtDW.DiscreteTimeIntegratory_DSTATE[7])) * 146.369354F;

  // SampleTimeMath: '<S11>/TSamp' incorporates:
  //   Inport: '<Root>/measure'
  //
  //  About '<S11>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  rtb_TSamp = rtU.measure.V_Kg[1] * 400.0F;

  // Sum: '<S11>/Diff' incorporates:
  //   UnitDelay: '<S11>/UD'
  //
  //  Block description for '<S11>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S11>/UD':
  //
  //   Store in Global RAM

  rtb_Diff_l = rtb_TSamp - rtDW.UD_DSTATE[1];

  // Sum: '<S67>/Add' incorporates:
  //   DiscreteIntegrator: '<S68>/Discrete-Time Integrator y'
  //   Inport: '<Root>/measure'

  q0_q2 = rtDW.DiscreteTimeIntegratory_DSTATE[1] - rtU.measure.s_Kg[1];
  rtb_Add_i_idx_7 = rtDW.DiscreteTimeIntegratory_DSTATE[7] - rtb_Diff_l;

  // SampleTimeMath: '<S11>/TSamp'
  //
  //  About '<S11>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  rtb_TSamp_idx_1 = rtb_TSamp;

  // Sum: '<S11>/Diff'
  //
  //  Block description for '<S11>/Diff':
  //
  //   Add in CPU

  rtb_Diff[1] = rtb_Diff_l;

  // Gain: '<S68>/omega^2' incorporates:
  //   DiscreteIntegrator: '<S61>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<S65>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<S68>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S68>/Discrete-Time Integrator y_dt'
  //   Gain: '<S68>/2*d//omega'
  //   Sum: '<S68>/Sum2'
  //   Sum: '<S68>/Sum3'

  rtb_omega2[2] = (rtDW.DiscreteTimeIntegrator_DSTATE_c[2] - (0.165312201F *
    rtDW.DiscreteTimeIntegratory_dt_DSTA[2] +
    rtDW.DiscreteTimeIntegratory_DSTATE[2])) * 146.369354F;
  rtb_omega2[5] = (rtDW.DiscreteTimeIntegrator_DSTATE_m[2] - (0.165312201F *
    rtDW.DiscreteTimeIntegratory_dt_DSTA[5] +
    rtDW.DiscreteTimeIntegratory_DSTATE[5])) * 146.369354F;
  rtb_omega2[8] = (cmd_V_NED_idx_2 - (0.165312201F *
    rtDW.DiscreteTimeIntegratory_dt_DSTA[8] +
    rtDW.DiscreteTimeIntegratory_DSTATE[8])) * 146.369354F;

  // SampleTimeMath: '<S11>/TSamp' incorporates:
  //   Inport: '<Root>/measure'
  //
  //  About '<S11>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  rtb_TSamp = rtU.measure.V_Kg[2] * 400.0F;

  // Sum: '<S11>/Diff' incorporates:
  //   UnitDelay: '<S11>/UD'
  //
  //  Block description for '<S11>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S11>/UD':
  //
  //   Store in Global RAM

  rtb_Diff_l = rtb_TSamp - rtDW.UD_DSTATE[2];

  // Sum: '<S67>/Add' incorporates:
  //   DiscreteIntegrator: '<S68>/Discrete-Time Integrator y'
  //   Inport: '<Root>/measure'

  scale = rtDW.DiscreteTimeIntegratory_DSTATE[2] - rtU.measure.s_Kg[2];

  // Sum: '<S11>/Diff'
  //
  //  Block description for '<S11>/Diff':
  //
  //   Add in CPU

  rtb_Diff[2] = rtb_Diff_l;

  // MATLAB Function: '<S37>/DCM to quaternions'
  DCMtoquaternions(rtb_M_bg, rtb_y_b);

  // MATLAB Function: '<S37>/Quaternion Reduced'
  QuaternionReduced(rtb_y_b, rtb_q_red_p, &q3_q3);

  // MATLAB Function: '<S2>/MATLAB Function' incorporates:
  //   Gain: '<Root>/Gain5'

  q0_q1 = 0.5F * -rtb_Switch[3] + 0.5F;

  // SampleTimeMath: '<S12>/TSamp' incorporates:
  //   Inport: '<Root>/measure'
  //
  //  About '<S12>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  rtb_TSamp_h = rtU.measure.omega_Kb[0] * 400.0F;

  // Sum: '<S12>/Diff' incorporates:
  //   UnitDelay: '<S12>/UD'
  //
  //  Block description for '<S12>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S12>/UD':
  //
  //   Store in Global RAM

  rtb_Diff_c[0] = rtb_TSamp_h - rtDW.UD_DSTATE_b[0];

  // SampleTimeMath: '<S12>/TSamp' incorporates:
  //   Inport: '<Root>/measure'
  //
  //  About '<S12>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  rtb_TSamp_k_idx_0 = rtb_TSamp_h;
  rtb_TSamp_h = rtU.measure.omega_Kb[1] * 400.0F;

  // Sum: '<S12>/Diff' incorporates:
  //   UnitDelay: '<S12>/UD'
  //
  //  Block description for '<S12>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S12>/UD':
  //
  //   Store in Global RAM

  rtb_Diff_c[1] = rtb_TSamp_h - rtDW.UD_DSTATE_b[1];

  // SampleTimeMath: '<S12>/TSamp' incorporates:
  //   Inport: '<Root>/measure'
  //
  //  About '<S12>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  rtb_TSamp_k_idx_1 = rtb_TSamp_h;
  rtb_TSamp_h = rtU.measure.omega_Kb[2] * 400.0F;

  // Sum: '<S12>/Diff' incorporates:
  //   UnitDelay: '<S12>/UD'
  //
  //  Block description for '<S12>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S12>/UD':
  //
  //   Store in Global RAM

  rtb_Diff_c[2] = rtb_TSamp_h - rtDW.UD_DSTATE_b[2];

  // DiscreteIntegrator: '<S57>/Discrete-Time Integrator'
  q1_q1 = rtDW.DiscreteTimeIntegrator_DSTATE_i;

  // Gain: '<S57>/1//T' incorporates:
  //   DiscreteIntegrator: '<S57>/Discrete-Time Integrator'
  //   Gain: '<S41>/r_max'
  //   Sum: '<S57>/Sum2'

  q2_q2 = (5.23598766F * rtb_Switch[2] - rtDW.DiscreteTimeIntegrator_DSTATE_i) *
    10.0F;

  // Switch: '<S69>/Switch2' incorporates:
  //   RelationalOperator: '<S69>/LowerRelop1'
  //   RelationalOperator: '<S69>/UpperRelop'
  //   Switch: '<S69>/Switch'

  if (rtb_Add_i_idx_0 > 12.2174301F) {
    rtb_Add_i_idx_0 = 12.2174301F;
  } else {
    if (rtb_Add_i_idx_0 < -12.2174301F) {
      // Switch: '<S69>/Switch'
      rtb_Add_i_idx_0 = -12.2174301F;
    }
  }

  // Sum: '<S67>/Add1' incorporates:
  //   DiscreteIntegrator: '<S68>/Discrete-Time Integrator y'
  //   Gain: '<S67>/Gain'
  //   Gain: '<S67>/Gain3'
  //   Gain: '<S67>/Gain4'
  //   Inport: '<Root>/measure'
  //   Sum: '<S67>/Add'

  rtb_Add_i_idx_0 = ((rtDW.DiscreteTimeIntegratory_DSTATE[3] - rtU.measure.V_Kg
                      [0]) * 3.9012F + 3.3528F * rtb_Add_i_idx_0) + 0.7114F *
    q2_q3;

  // RelationalOperator: '<S70>/LowerRelop1'
  rtb_LowerRelop1_idx_0 = (rtb_Add_i_idx_0 > 9.0F);

  // Sum: '<S67>/Add1'
  rtb_Delta_ny[0] = rtb_Add_i_idx_0;

  // Switch: '<S69>/Switch2' incorporates:
  //   RelationalOperator: '<S69>/LowerRelop1'
  //   RelationalOperator: '<S69>/UpperRelop'
  //   Switch: '<S69>/Switch'

  if (q0_q2 > 12.2174301F) {
    q0_q2 = 12.2174301F;
  } else {
    if (q0_q2 < -12.2174301F) {
      // Switch: '<S69>/Switch'
      q0_q2 = -12.2174301F;
    }
  }

  // Sum: '<S67>/Add1' incorporates:
  //   DiscreteIntegrator: '<S68>/Discrete-Time Integrator y'
  //   Gain: '<S67>/Gain'
  //   Gain: '<S67>/Gain3'
  //   Gain: '<S67>/Gain4'
  //   Inport: '<Root>/measure'
  //   Sum: '<S67>/Add'

  rtb_Add_i_idx_0 = ((rtDW.DiscreteTimeIntegratory_DSTATE[4] - rtU.measure.V_Kg
                      [1]) * 3.9012F + 3.3528F * q0_q2) + 0.7114F *
    rtb_Add_i_idx_7;

  // RelationalOperator: '<S70>/LowerRelop1'
  rtb_LowerRelop1_idx_1 = (rtb_Add_i_idx_0 > 9.0F);

  // Sum: '<S67>/Add1'
  rtb_Delta_ny[1] = rtb_Add_i_idx_0;

  // Switch: '<S69>/Switch2' incorporates:
  //   RelationalOperator: '<S69>/LowerRelop1'
  //   RelationalOperator: '<S69>/UpperRelop'
  //   Switch: '<S69>/Switch'

  if (scale > 12.2174301F) {
    scale = 12.2174301F;
  } else {
    if (scale < -12.2174301F) {
      // Switch: '<S69>/Switch'
      scale = -12.2174301F;
    }
  }

  // Sum: '<S67>/Add1' incorporates:
  //   DiscreteIntegrator: '<S68>/Discrete-Time Integrator y'
  //   Gain: '<S67>/Gain'
  //   Gain: '<S67>/Gain3'
  //   Gain: '<S67>/Gain4'
  //   Inport: '<Root>/measure'
  //   Sum: '<S67>/Add'

  rtb_Add_i_idx_0 = ((rtDW.DiscreteTimeIntegratory_DSTATE[5] - rtU.measure.V_Kg
                      [2]) * 3.9012F + 3.3528F * scale) +
    (rtDW.DiscreteTimeIntegratory_DSTATE[8] - rtb_Diff_l) * 0.7114F;

  // RelationalOperator: '<S70>/UpperRelop'
  q0_q2 = rtb_Delta_ny[0];

  // Switch: '<S70>/Switch' incorporates:
  //   RelationalOperator: '<S70>/UpperRelop'

  if (rtb_Delta_ny[0] < -9.0F) {
    q0_q2 = -9.0F;
  }

  // Switch: '<S70>/Switch2'
  if (rtb_LowerRelop1_idx_0) {
    q0_q2 = 9.0F;
  }

  // Sum: '<S62>/Add1'
  rtb_nu[0] = cmd_V_NED_idx_0 + q0_q2;

  // MATLAB Function: '<S10>/INDI Copter Acc 2 Lean Vector'
  rtb_n_g_des[0] = rtb_Diff[0];

  // RelationalOperator: '<S70>/UpperRelop'
  q0_q2 = rtb_Delta_ny[1];

  // Switch: '<S70>/Switch' incorporates:
  //   RelationalOperator: '<S70>/UpperRelop'

  if (rtb_Delta_ny[1] < -9.0F) {
    q0_q2 = -9.0F;
  }

  // Switch: '<S70>/Switch2'
  if (rtb_LowerRelop1_idx_1) {
    q0_q2 = 9.0F;
  }

  // Sum: '<S62>/Add1'
  rtb_nu[1] = cmd_V_NED_idx_1 + q0_q2;

  // MATLAB Function: '<S10>/INDI Copter Acc 2 Lean Vector'
  rtb_n_g_des[1] = rtb_Diff[1];
  q0_q2 = rtb_Add_i_idx_0;

  // Switch: '<S70>/Switch' incorporates:
  //   RelationalOperator: '<S70>/UpperRelop'

  if (rtb_Add_i_idx_0 < -9.0F) {
    q0_q2 = -9.0F;
  }

  // Switch: '<S70>/Switch2' incorporates:
  //   RelationalOperator: '<S70>/LowerRelop1'

  if (rtb_Add_i_idx_0 > 9.0F) {
    q0_q2 = 9.0F;
  }

  // Sum: '<S62>/Add1'
  rtb_nu[2] = cmd_V_NED_idx_2 + q0_q2;

  // MATLAB Function: '<S10>/INDI Copter Acc 2 Lean Vector'
  rtb_n_g_des[2] = rtb_Diff_l - 9.81F;
  q2_q3 = norm(rtb_n_g_des);
  for (idx = 0; idx < 3; idx++) {
    rtb_n_g_des[idx] = -rtb_M_bg[3 * idx + 2] + (rtb_nu[idx] - rtb_Diff[idx]) /
      q2_q3;
  }

  q0_q2 = norm(rtb_n_g_des);
  i = 0;
  if (std::abs(q0_q2) < 2.22044605E-16F) {
    i = 1;
  }

  if (0 <= i - 1) {
    q0_q2 = 2.22044605E-16F;
  }

  rtb_n_g_des[0] /= q0_q2;
  rtb_n_g_des[1] /= q0_q2;
  rtb_Add_i_idx_7 = rtb_n_g_des[2] / q0_q2;

  // MATLAB Function: '<S10>/MATLAB Function4'
  rtb_Add_i_idx_0 = -rtb_Add_i_idx_7;
  if (-rtb_Add_i_idx_7 > 1.0F) {
    rtb_Add_i_idx_0 = 1.0F;
  }

  if (rtb_Add_i_idx_0 < -1.0F) {
    rtb_Add_i_idx_0 = -1.0F;
  }

  rtb_Add_i_idx_0 = std::acos(rtb_Add_i_idx_0);

  // Saturate: '<S10>/Saturation1'
  if (rtb_Add_i_idx_0 > 1.04719758F) {
    rtb_Add_i_idx_0 = 1.04719758F;
  } else {
    if (rtb_Add_i_idx_0 < 0.0F) {
      rtb_Add_i_idx_0 = 0.0F;
    }
  }

  // End of Saturate: '<S10>/Saturation1'

  // Gain: '<S41>/lean_angle_max' incorporates:
  //   Gain: '<S10>/Gain1'
  //   MATLAB Function: '<S10>/MATLAB Function4'

  rtb_Add_i_idx_0 = 0.95492965F * rtb_Add_i_idx_0 * 1.04719758F;
  rtb_Add_i_idx_7 = std::atan2(rtb_n_g_des[1], rtb_n_g_des[0]);

  // MATLAB Function: '<S41>/lean angles 2 lean vector'
  q0_q2 = std::sin(rtb_Add_i_idx_0);
  rtb_n_g_des[0] = q0_q2 * std::cos(rtb_Add_i_idx_7);
  rtb_n_g_des[1] = q0_q2 * std::sin(rtb_Add_i_idx_7);
  rtb_n_g_des[2] = -std::cos(rtb_Add_i_idx_0);

  // DiscreteIntegrator: '<S51>/Discrete-Time Integrator y'
  if (rtDW.DiscreteTimeIntegratory_IC_LOAD != 0) {
    rtDW.DiscreteTimeIntegratory_DSTAT_h[0] = rtb_n_g_des[0];
    rtDW.DiscreteTimeIntegratory_DSTAT_h[1] = rtb_n_g_des[1];
    rtDW.DiscreteTimeIntegratory_DSTAT_h[2] = rtb_n_g_des[2];
  }

  // MATLAB Function: '<S41>/n ref norm'
  scale = 1.29246971E-26F;

  // DiscreteIntegrator: '<S51>/Discrete-Time Integrator y_dt'
  rtb_y_dt_gf[0] = rtDW.DiscreteTimeIntegratory_dt_DS_m[0];

  // MATLAB Function: '<S41>/n ref norm' incorporates:
  //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y'

  absxk = std::abs(rtDW.DiscreteTimeIntegratory_DSTAT_h[0]);
  if (absxk > 1.29246971E-26F) {
    q0_q2 = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    q0_q2 = t * t;
  }

  // DiscreteIntegrator: '<S51>/Discrete-Time Integrator y_dt'
  rtb_y_dt_gf[1] = rtDW.DiscreteTimeIntegratory_dt_DS_m[1];

  // MATLAB Function: '<S41>/n ref norm' incorporates:
  //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y'

  absxk = std::abs(rtDW.DiscreteTimeIntegratory_DSTAT_h[1]);
  if (absxk > scale) {
    t = scale / absxk;
    q0_q2 = q0_q2 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q0_q2 += t * t;
  }

  // DiscreteIntegrator: '<S51>/Discrete-Time Integrator y_dt'
  rtb_y_dt_gf[2] = rtDW.DiscreteTimeIntegratory_dt_DS_m[2];

  // MATLAB Function: '<S41>/n ref norm' incorporates:
  //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y'

  absxk = std::abs(rtDW.DiscreteTimeIntegratory_DSTAT_h[2]);
  if (absxk > scale) {
    t = scale / absxk;
    q0_q2 = q0_q2 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q0_q2 += t * t;
  }

  q0_q2 = scale * std::sqrt(q0_q2);
  i = 0;
  if (q0_q2 < 2.22044605E-16F) {
    i = 1;
  }

  if (0 <= i - 1) {
    q0_q2 = 2.22044605E-16F;
  }

  rtb_n[0] = rtDW.DiscreteTimeIntegratory_DSTAT_h[0] / q0_q2;
  rtb_n[1] = rtDW.DiscreteTimeIntegratory_DSTAT_h[1] / q0_q2;
  rtb_n[2] = rtDW.DiscreteTimeIntegratory_DSTAT_h[2] / q0_q2;

  // DiscreteIntegrator: '<S58>/Discrete-Time Integrator y'
  if (rtDW.DiscreteTimeIntegratory_IC_LO_p != 0) {
    rtDW.DiscreteTimeIntegratory_DSTAT_m[0] = rtb_n[0];
    rtDW.DiscreteTimeIntegratory_DSTAT_m[1] = rtb_n[1];
    rtDW.DiscreteTimeIntegratory_DSTAT_m[2] = rtb_n[2];
  }

  rtb_Delta_ny[0] = rtDW.DiscreteTimeIntegratory_DSTAT_m[0];
  rtb_Delta_ny[1] = rtDW.DiscreteTimeIntegratory_DSTAT_m[1];
  rtb_Delta_ny[2] = rtDW.DiscreteTimeIntegratory_DSTAT_m[2];

  // DiscreteIntegrator: '<S59>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y_dt'

  if (rtDW.DiscreteTimeIntegratory_IC_LO_n != 0) {
    rtDW.DiscreteTimeIntegratory_DSTAT_e[0] =
      rtDW.DiscreteTimeIntegratory_dt_DS_m[0];
    rtDW.DiscreteTimeIntegratory_DSTAT_e[1] =
      rtDW.DiscreteTimeIntegratory_dt_DS_m[1];
    rtDW.DiscreteTimeIntegratory_DSTAT_e[2] =
      rtDW.DiscreteTimeIntegratory_dt_DS_m[2];
  }

  rtb_y_j3[0] = rtDW.DiscreteTimeIntegratory_DSTAT_e[0];

  // Gain: '<S51>/omega^2' incorporates:
  //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y_dt'
  //   Gain: '<S51>/2*d//omega'
  //   Sum: '<S51>/Sum2'
  //   Sum: '<S51>/Sum3'

  rtb_n_g_des[0] = (rtb_n_g_des[0] - (0.13333334F *
    rtDW.DiscreteTimeIntegratory_dt_DS_m[0] +
    rtDW.DiscreteTimeIntegratory_DSTAT_h[0])) * 225.0F;

  // DiscreteIntegrator: '<S59>/Discrete-Time Integrator y'
  rtb_y_j3[1] = rtDW.DiscreteTimeIntegratory_DSTAT_e[1];

  // Gain: '<S51>/omega^2' incorporates:
  //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y_dt'
  //   Gain: '<S51>/2*d//omega'
  //   Sum: '<S51>/Sum2'
  //   Sum: '<S51>/Sum3'

  rtb_n_g_des[1] = (rtb_n_g_des[1] - (0.13333334F *
    rtDW.DiscreteTimeIntegratory_dt_DS_m[1] +
    rtDW.DiscreteTimeIntegratory_DSTAT_h[1])) * 225.0F;

  // DiscreteIntegrator: '<S59>/Discrete-Time Integrator y'
  rtb_y_j3[2] = rtDW.DiscreteTimeIntegratory_DSTAT_e[2];

  // Gain: '<S51>/omega^2' incorporates:
  //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y_dt'
  //   Gain: '<S51>/2*d//omega'
  //   Sum: '<S51>/Sum2'
  //   Sum: '<S51>/Sum3'

  rtb_Add_i_idx_7 = (rtb_n_g_des[2] - (0.13333334F *
    rtDW.DiscreteTimeIntegratory_dt_DS_m[2] +
    rtDW.DiscreteTimeIntegratory_DSTAT_h[2])) * 225.0F;
  rtb_n_g_des[2] = rtb_Add_i_idx_7;

  // DiscreteIntegrator: '<S60>/Discrete-Time Integrator y'
  if (rtDW.DiscreteTimeIntegratory_IC_LO_f != 0) {
    rtDW.DiscreteTimeIntegratory_DSTAT_f[0] = rtb_n_g_des[0];
    rtDW.DiscreteTimeIntegratory_DSTAT_f[1] = rtb_n_g_des[1];
    rtDW.DiscreteTimeIntegratory_DSTAT_f[2] = rtb_Add_i_idx_7;
  }

  rtb_y_p[0] = rtDW.DiscreteTimeIntegratory_DSTAT_f[0];
  rtb_y_p[1] = rtDW.DiscreteTimeIntegratory_DSTAT_f[1];
  rtb_y_p[2] = rtDW.DiscreteTimeIntegratory_DSTAT_f[2];

  // MATLAB Function: '<S41>/Lean Vector Derivative Trafo Delay' incorporates:
  //   Inport: '<Root>/measure'

  LeanVectorDerivativeTrafo(rtb_Delta_ny, rtb_y_j3, rtb_y_p, rtb_M_bg,
    rtU.measure.omega_Kb, rtb_Diff_c, rtb_Diff, rtb_n_b_dt, rtb_n_b_dt2);

  // MATLAB Function: '<S41>/Lean Vector Derivative Trafo' incorporates:
  //   Inport: '<Root>/measure'

  LeanVectorDerivativeTrafo(rtb_n, rtb_y_dt_gf, rtb_n_g_des, rtb_M_bg,
    rtU.measure.omega_Kb, rtb_Diff_c, rtb_y_j3, rtb_y_p, rtb_Delta_ny);

  // MATLAB Function: '<S41>/MATLAB Function'
  if (rtb_Diff[2] > 0.0F) {
    scale = 1.29246971E-26F;
    absxk = std::abs(rtb_Diff[0]);
    if (absxk > 1.29246971E-26F) {
      q2_q3 = 1.0F;
      scale = absxk;
    } else {
      t = absxk / 1.29246971E-26F;
      q2_q3 = t * t;
    }

    absxk = std::abs(rtb_Diff[1]);
    if (absxk > scale) {
      t = scale / absxk;
      q2_q3 = q2_q3 * t * t + 1.0F;
      scale = absxk;
    } else {
      t = absxk / scale;
      q2_q3 += t * t;
    }

    q2_q3 = scale * std::sqrt(q2_q3);
    i = 0;
    if (q2_q3 < 2.22044605E-16F) {
      i = 1;
    }

    if (0 <= i - 1) {
      q2_q3 = 2.22044605E-16F;
    }

    rtb_Diff[0] = rtb_Diff[0] / q2_q3 * 2.0F - rtb_Diff[0];
    rtb_n_b_dt[0] = -rtb_n_b_dt[0];
    rtb_n_b_dt2[0] = -rtb_n_b_dt2[0];
    rtb_Diff[1] = rtb_Diff[1] / q2_q3 * 2.0F - rtb_Diff[1];
    rtb_n_b_dt[1] = -rtb_n_b_dt[1];
    rtb_n_b_dt2[1] = -rtb_n_b_dt2[1];
  }

  // MATLAB Function: '<S21>/DCM to quaternions'
  DCMtoquaternions(rtb_M_bg, rtb_y_b);

  // MATLAB Function: '<S21>/Quaternion Reduced'
  QuaternionReduced(rtb_y_b, rtb_Switch, &q0_q2);

  // DiscreteIntegrator: '<S41>/Discrete-Time Integrator2'
  if (rtDW.DiscreteTimeIntegrator2_IC_LOAD != 0) {
    rtDW.DiscreteTimeIntegrator2_DSTATE = q0_q2;
  }

  // MATLAB Function: '<S36>/wrap angle' incorporates:
  //   DiscreteIntegrator: '<S41>/Discrete-Time Integrator2'

  wrapangle(rtDW.DiscreteTimeIntegrator2_DSTATE, &q0_q2);

  // MATLAB Function: '<S36>/wrap angle1'
  wrapangle(q3_q3, &scale);

  // MATLAB Function: '<S36>/angle error'
  q0_q2 -= scale;
  if (q0_q2 > 3.1415926535897931) {
    q0_q2 -= 6.28318548F;
  } else {
    if (q0_q2 < -3.1415926535897931) {
      q0_q2 += 6.28318548F;
    }
  }

  // End of MATLAB Function: '<S36>/angle error'

  // Sum: '<S17>/Add1' incorporates:
  //   Gain: '<S38>/Gain1'
  //   Gain: '<S38>/Gain3'
  //   Gain: '<S38>/Gain5'
  //   MATLAB Function: '<S41>/MATLAB Function'
  //   Sum: '<S36>/error1 4'
  //   Sum: '<S36>/error1 5'
  //   Sum: '<S36>/error1 6'
  //   Sum: '<S38>/Add'

  rtb_Delta_ny[0] += (200.0F * rtb_Diff[0] + 39.9604F * rtb_n_b_dt[0]) + 0.9513F
    * rtb_n_b_dt2[0];
  rtb_Delta_ny[1] += (200.0F * rtb_Diff[1] + 39.9604F * rtb_n_b_dt[1]) + 0.9513F
    * rtb_n_b_dt2[1];

  // Sum: '<S17>/Add2' incorporates:
  //   DiscreteIntegrator: '<S57>/Discrete-Time Integrator'
  //   Gain: '<S38>/Gain2'
  //   Gain: '<S38>/Gain4'
  //   Gain: '<S38>/Gain6'
  //   Inport: '<Root>/measure'
  //   Sum: '<S17>/Add1'
  //   Sum: '<S36>/error1 8'
  //   Sum: '<S36>/error1 9'
  //   Sum: '<S38>/Add1'

  rtb_Add_i_idx_0 = ((((rtDW.DiscreteTimeIntegrator_DSTATE_i -
                        rtU.measure.omega_Kb[2]) * 15.6068F + 50.0F * q0_q2) +
                      (q2_q2 - rtb_Diff_c[2]) * 0.4357F) + q2_q2) - rtb_Diff_c[2];

  // DiscreteIntegrator: '<S19>/Discrete-Time Integrator y'
  if (rtDW.DiscreteTimeIntegratory_IC_LO_b != 0) {
    rtDW.DiscreteTimeIntegratory_DSTAT_i = rtb_nu[2];
  }

  // Sum: '<S2>/Add' incorporates:
  //   DiscreteIntegrator: '<S19>/Discrete-Time Integrator y'

  q0_q2 = rtDW.DiscreteTimeIntegratory_DSTAT_i - rtb_Diff_l;

  // MATLAB Function: '<S2>/MATLAB Function1'
  if (rtb_Diff[2] >= 0.0F) {
    q0_q2 = 0.0F;
  }

  // End of MATLAB Function: '<S2>/MATLAB Function1'

  // MATLAB Function: '<S27>/dcm2Lean'
  if (1.0F > rtb_M_bg[8]) {
    scale = rtb_M_bg[8];
  } else {
    scale = 1.0F;
  }

  if (-1.0F >= scale) {
    scale = -1.0F;
  }

  // MATLAB Function: '<S27>/MATLAB Function' incorporates:
  //   MATLAB Function: '<S27>/dcm2Lean'

  scale = std::cos(std::acos(scale));
  if (scale > 0.1) {
    q0_q2 /= scale;
  }

  // End of MATLAB Function: '<S27>/MATLAB Function'

  // MATLAB Function: '<S2>/MATLAB Function2' incorporates:
  //   Constant: '<S2>/Constant3'
  //   Constant: '<S2>/Constant4'
  //   Constant: '<S2>/Constant5'
  //   Inport: '<Root>/measure'

  y = 0.0;
  for (idx = 0; idx < 3; idx++) {
    rtb_Diff[idx] = -rtb_M_bg[3 * idx + 2];
  }

  rtb_Diff_l = 0.0F;
  q2_q3 = 0.0F;
  scale = 1.29246971E-26F;
  for (i = 0; i < 3; i++) {
    rtb_Diff_l += (rtb_M_bg[i + 6] * rtb_Diff[2] + (rtb_M_bg[i + 3] * rtb_Diff[1]
      + rtb_M_bg[i] * rtb_Diff[0])) * rtU.measure.omega_Kb[i];
    absxk = std::abs(rtU.measure.omega_Kb[i]);
    if (absxk > scale) {
      t = scale / absxk;
      q2_q3 = q2_q3 * t * t + 1.0F;
      scale = absxk;
    } else {
      t = absxk / scale;
      q2_q3 += t * t;
    }
  }

  q2_q3 = scale * std::sqrt(q2_q3);
  i = 0;
  if (q2_q3 < 2.22044605E-16F) {
    i = 1;
  }

  if (0 <= i - 1) {
    q2_q3 = 2.22044605E-16F;
  }

  if (rtb_Diff[2] < 0.0F) {
    y = (std::tanh((std::abs(rtb_Diff_l) / q2_q3 * std::abs
                    (rtU.measure.omega_Kb[2]) - 8.0F) * 0.382833332F) + 1.0F) *
      150.0F;
  }

  // Gain: '<S13>/Gain'
  rtb_uDLookupTable2[0] = 0.0F;

  // MATLAB Function: '<S29>/MATLAB Function2' incorporates:
  //   DiscreteIntegrator: '<S32>/Discrete-Time Integrator'

  rtb_y_b[0] = 0.1F - rtDW.DiscreteTimeIntegrator_DSTATE[0];
  rtb_Switch[0] = 1.0F - rtDW.DiscreteTimeIntegrator_DSTATE[0];

  // Gain: '<S13>/Gain'
  rtb_uDLookupTable2[1] = 0.0F;

  // MATLAB Function: '<S29>/MATLAB Function2' incorporates:
  //   DiscreteIntegrator: '<S32>/Discrete-Time Integrator'

  rtb_y_b[1] = 0.1F - rtDW.DiscreteTimeIntegrator_DSTATE[1];
  rtb_Switch[1] = 1.0F - rtDW.DiscreteTimeIntegrator_DSTATE[1];

  // Gain: '<S13>/Gain'
  rtb_uDLookupTable2[2] = 0.0F;

  // MATLAB Function: '<S29>/MATLAB Function2' incorporates:
  //   DiscreteIntegrator: '<S32>/Discrete-Time Integrator'

  rtb_y_b[2] = 0.1F - rtDW.DiscreteTimeIntegrator_DSTATE[2];
  rtb_Switch[2] = 1.0F - rtDW.DiscreteTimeIntegrator_DSTATE[2];

  // Gain: '<S13>/Gain' incorporates:
  //   MATLAB Function: '<S2>/MATLAB Function2'

  rtb_uDLookupTable2[3] = (real32_T)y;

  // MATLAB Function: '<S29>/MATLAB Function2' incorporates:
  //   DiscreteIntegrator: '<S32>/Discrete-Time Integrator'

  rtb_y_b[3] = 0.1F - rtDW.DiscreteTimeIntegrator_DSTATE[3];
  rtb_Switch[3] = 1.0F - rtDW.DiscreteTimeIntegrator_DSTATE[3];
  memset(&d_0[0], 0, sizeof(real32_T) << 4U);
  for (i = 0; i < 4; i++) {
    d_0[i + (i << 2)] = rtb_uDLookupTable2[i];
    W[i] = 0.0;

    // MATLAB Function: '<S28>/MATLAB Function' incorporates:
    //   Constant: '<S28>/ny_du_red_trim'

    for (idx = 0; idx < 4; idx++) {
      tmp_0 = idx << 2;
      tmp[i + tmp_0] = (((d[tmp_0 + 1] * rtConstP.ny_du_red_trim_Value[i + 4] +
                          d[tmp_0] * rtConstP.ny_du_red_trim_Value[i]) + d[tmp_0
                         + 2] * rtConstP.ny_du_red_trim_Value[i + 8]) + d[tmp_0
                        + 3] * rtConstP.ny_du_red_trim_Value[i + 12]) +
        rtb_G2[tmp_0 + i];
    }

    rtb_uDLookupTable2[i] = (rtb_y_b[i] + rtb_Switch[i]) * 0.5F;
  }

  // Sum: '<S24>/Add2' incorporates:
  //   Product: '<S24>/MatrixMultiply2'
  //   UnitDelay: '<S24>/Unit Delay1'

  rtb_Delta_ny_0[0] = rtb_Delta_ny[0];
  rtb_Delta_ny_0[1] = rtb_Delta_ny[1];
  rtb_Delta_ny_0[2] = rtb_Add_i_idx_0;
  rtb_Delta_ny_0[3] = q0_q2;
  for (idx = 0; idx < 4; idx++) {
    rtb_q_red_p[idx] = (((rtb_G2[idx + 4] * rtDW.UnitDelay1_DSTATE[1] +
                          rtb_G2[idx] * rtDW.UnitDelay1_DSTATE[0]) + rtb_G2[idx
                         + 8] * rtDW.UnitDelay1_DSTATE[2]) + rtb_G2[idx + 12] *
                        rtDW.UnitDelay1_DSTATE[3]) + rtb_Delta_ny_0[idx];
  }

  // End of Sum: '<S24>/Add2'

  // MATLAB Function: '<S29>/MATLAB Function2' incorporates:
  //   DiscreteIntegrator: '<S32>/Discrete-Time Integrator'
  //   Lookup_n-D: '<S24>/1-D Lookup Table1'

  for (idx = 0; idx < 16; idx++) {
    rtb_G2[idx] = (real32_T)rtConstP.MATLABFunction2_ca.W_v[idx] + d_0[idx];
  }

  rtb_Delta_ny_0[0] = 0.1F - rtDW.DiscreteTimeIntegrator_DSTATE[0];
  rtb_Delta_ny_0[1] = 0.1F - rtDW.DiscreteTimeIntegrator_DSTATE[1];
  rtb_Delta_ny_0[2] = 0.1F - rtDW.DiscreteTimeIntegrator_DSTATE[2];
  rtb_Delta_ny_0[3] = 0.1F - rtDW.DiscreteTimeIntegrator_DSTATE[3];
  wls_alloc(tmp, rtb_q_red_p, rtb_y_b, rtb_Switch, rtb_G2,
            &rtConstP.MATLABFunction2_ca.W_u[0], rtb_Delta_ny_0, 1000.0F +
            look1_iflf_binlx(q0_q1, rtConstP.uDLookupTable1_bp01Data,
             rtConstP.uDLookupTable1_tableData, 3U), rtb_uDLookupTable2, W,
            100.0);

  // Sum: '<S13>/Add6' incorporates:
  //   DiscreteIntegrator: '<S26>/Discrete-Time Integrator y'

  rtb_Add_i_idx_0 = rtb_uDLookupTable2[0] +
    rtDW.DiscreteTimeIntegratory_DSTAT_a[0];

  // Saturate: '<S13>/Saturation3'
  if (rtb_Add_i_idx_0 > 1.0F) {
    rtb_Add_i_idx_0 = 1.0F;
  } else {
    if (rtb_Add_i_idx_0 < 0.1F) {
      rtb_Add_i_idx_0 = 0.1F;
    }
  }

  // Outport: '<Root>/logs'
  rtY.logs[0] = rtb_Add_i_idx_0;

  // Saturate: '<S13>/Saturation3'
  rtb_y_b[0] = rtb_Add_i_idx_0;

  // Sum: '<S13>/Add6' incorporates:
  //   DiscreteIntegrator: '<S26>/Discrete-Time Integrator y'

  rtb_Add_i_idx_0 = rtb_uDLookupTable2[1] +
    rtDW.DiscreteTimeIntegratory_DSTAT_a[1];

  // Saturate: '<S13>/Saturation3'
  if (rtb_Add_i_idx_0 > 1.0F) {
    rtb_Add_i_idx_0 = 1.0F;
  } else {
    if (rtb_Add_i_idx_0 < 0.1F) {
      rtb_Add_i_idx_0 = 0.1F;
    }
  }

  // Outport: '<Root>/logs'
  rtY.logs[1] = rtb_Add_i_idx_0;

  // Saturate: '<S13>/Saturation3'
  rtb_y_b[1] = rtb_Add_i_idx_0;

  // Sum: '<S13>/Add6' incorporates:
  //   DiscreteIntegrator: '<S26>/Discrete-Time Integrator y'

  rtb_Add_i_idx_0 = rtb_uDLookupTable2[2] +
    rtDW.DiscreteTimeIntegratory_DSTAT_a[2];

  // Saturate: '<S13>/Saturation3'
  if (rtb_Add_i_idx_0 > 1.0F) {
    rtb_Add_i_idx_0 = 1.0F;
  } else {
    if (rtb_Add_i_idx_0 < 0.1F) {
      rtb_Add_i_idx_0 = 0.1F;
    }
  }

  // Outport: '<Root>/logs'
  rtY.logs[2] = rtb_Add_i_idx_0;

  // Saturate: '<S13>/Saturation3'
  rtb_y_b[2] = rtb_Add_i_idx_0;

  // Sum: '<S13>/Add6' incorporates:
  //   DiscreteIntegrator: '<S26>/Discrete-Time Integrator y'

  rtb_Add_i_idx_0 = rtb_uDLookupTable2[3] +
    rtDW.DiscreteTimeIntegratory_DSTAT_a[3];

  // Saturate: '<S13>/Saturation3'
  if (rtb_Add_i_idx_0 > 1.0F) {
    rtb_Add_i_idx_0 = 1.0F;
  } else {
    if (rtb_Add_i_idx_0 < 0.1F) {
      rtb_Add_i_idx_0 = 0.1F;
    }
  }

  // Outport: '<Root>/logs' incorporates:
  //   DiscreteIntegrator: '<S61>/Discrete-Time Integrator'
  //   Inport: '<Root>/measure'

  rtY.logs[3] = rtb_Add_i_idx_0;
  rtY.logs[4] = rtDW.DiscreteTimeIntegrator_DSTATE_c[0];
  rtY.logs[7] = rtU.measure.s_Kg[0];
  rtY.logs[5] = rtDW.DiscreteTimeIntegrator_DSTATE_c[1];
  rtY.logs[8] = rtU.measure.s_Kg[1];
  rtY.logs[6] = rtDW.DiscreteTimeIntegrator_DSTATE_c[2];
  rtY.logs[9] = rtU.measure.s_Kg[2];
  rtY.logs[10] = q3_q3;
  rtY.logs[11] = 0.0F;
  rtY.logs[12] = 0.0F;
  rtY.logs[13] = 0.0F;
  rtY.logs[14] = 0.0F;

  // Lookup_n-D: '<S8>/1-D Lookup Table1' incorporates:
  //   Inport: '<Root>/cmd'

  q3_q3 = look1_iflf_binlx(rtU.cmd.RC_pwm[7], rtConstP.pooled7,
    rtConstP.pooled16, 1U);

  // Switch: '<S4>/Switch2' incorporates:
  //   RelationalOperator: '<S4>/LowerRelop1'

  if (rtb_y_b[1] > q3_q3) {
    // Outport: '<Root>/u'
    rtY.u[0] = q3_q3;
  } else {
    // Outport: '<Root>/u' incorporates:
    //   Switch: '<S4>/Switch'

    rtY.u[0] = rtb_y_b[1];
  }

  // End of Switch: '<S4>/Switch2'

  // Switch: '<S5>/Switch2' incorporates:
  //   RelationalOperator: '<S5>/LowerRelop1'

  if (rtb_Add_i_idx_0 <= q3_q3) {
    q3_q3 = rtb_Add_i_idx_0;
  }

  // End of Switch: '<S5>/Switch2'

  // Lookup_n-D: '<S8>/1-D Lookup Table' incorporates:
  //   Inport: '<Root>/cmd'

  scale = look1_iflf_binlx(rtU.cmd.RC_pwm[6], rtConstP.pooled7,
    rtConstP.pooled16, 1U);

  // RateLimiter: '<S8>/Rate Limiter'
  absxk = scale - rtDW.PrevY;
  if (absxk > 2.5F) {
    scale = rtDW.PrevY + 2.5F;
  } else {
    if (absxk < -0.00025F) {
      scale = rtDW.PrevY + -0.00025F;
    }
  }

  rtDW.PrevY = scale;

  // End of RateLimiter: '<S8>/Rate Limiter'

  // Math: '<S8>/Square'
  scale *= scale;

  // Switch: '<S6>/Switch2' incorporates:
  //   RelationalOperator: '<S6>/LowerRelop1'
  //   Switch: '<S6>/Switch'

  if (rtb_y_b[2] <= scale) {
    scale = rtb_y_b[2];
  }

  // End of Switch: '<S6>/Switch2'

  // Outport: '<Root>/u' incorporates:
  //   Gain: '<Root>/Gain1'
  //   Gain: '<Root>/Gain2'
  //   Gain: '<Root>/Gain3'
  //   Gain: '<Root>/Gain4'

  rtY.u[1] = q3_q3;
  rtY.u[2] = rtb_y_b[0];
  rtY.u[3] = scale;
  rtY.u[4] = 0.0F;
  rtY.u[5] = 0.0F;
  rtY.u[6] = 0.0F;
  rtY.u[7] = 0.0F;

  // Sum: '<S26>/Sum2' incorporates:
  //   Delay: '<S13>/Delay'
  //   DiscreteIntegrator: '<S26>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S26>/Discrete-Time Integrator y_dt'
  //   Gain: '<S26>/2*d//omega'
  //   Sum: '<S26>/Sum3'

  rtb_Switch[0] = rtDW.Delay_DSTATE[0] - (0.0039788736F *
    rtDW.DiscreteTimeIntegratory_dt_DS_n[0] +
    rtDW.DiscreteTimeIntegratory_DSTAT_a[0]);
  rtb_Switch[1] = rtDW.Delay_DSTATE[1] - (0.0039788736F *
    rtDW.DiscreteTimeIntegratory_dt_DS_n[1] +
    rtDW.DiscreteTimeIntegratory_DSTAT_a[1]);
  rtb_Switch[2] = rtDW.Delay_DSTATE[2] - (0.0039788736F *
    rtDW.DiscreteTimeIntegratory_dt_DS_n[2] +
    rtDW.DiscreteTimeIntegratory_DSTAT_a[2]);
  rtb_Switch[3] = rtDW.Delay_DSTATE[3] - (0.0039788736F *
    rtDW.DiscreteTimeIntegratory_dt_DS_n[3] +
    rtDW.DiscreteTimeIntegratory_DSTAT_a[3]);

  // Sum: '<S60>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S60>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S60>/Discrete-Time Integrator y_dt'
  //   Gain: '<S60>/2*d//omega'
  //   Sum: '<S60>/Sum3'

  rtb_Diff[0] = rtb_n_g_des[0] - (0.0319788754F *
    rtDW.DiscreteTimeIntegratory_dt_DS_f[0] +
    rtDW.DiscreteTimeIntegratory_DSTAT_f[0]);

  // Sum: '<S59>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y_dt'
  //   DiscreteIntegrator: '<S59>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S59>/Discrete-Time Integrator y_dt'
  //   Gain: '<S59>/2*d//omega'
  //   Sum: '<S59>/Sum3'

  rtb_Diff_c[0] = rtDW.DiscreteTimeIntegratory_dt_DS_m[0] - (0.0319788754F *
    rtDW.DiscreteTimeIntegratory_dt_DS_l[0] +
    rtDW.DiscreteTimeIntegratory_DSTAT_e[0]);

  // Sum: '<S58>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S58>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S58>/Discrete-Time Integrator y_dt'
  //   Gain: '<S58>/2*d//omega'
  //   Sum: '<S58>/Sum3'

  rtb_n[0] -= 0.0319788754F * rtDW.DiscreteTimeIntegratory_dt_DS_i[0] +
    rtDW.DiscreteTimeIntegratory_DSTAT_m[0];

  // Sum: '<S60>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S60>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S60>/Discrete-Time Integrator y_dt'
  //   Gain: '<S60>/2*d//omega'
  //   Sum: '<S60>/Sum3'

  rtb_Diff[1] = rtb_n_g_des[1] - (0.0319788754F *
    rtDW.DiscreteTimeIntegratory_dt_DS_f[1] +
    rtDW.DiscreteTimeIntegratory_DSTAT_f[1]);

  // Sum: '<S59>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y_dt'
  //   DiscreteIntegrator: '<S59>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S59>/Discrete-Time Integrator y_dt'
  //   Gain: '<S59>/2*d//omega'
  //   Sum: '<S59>/Sum3'

  rtb_Diff_c[1] = rtDW.DiscreteTimeIntegratory_dt_DS_m[1] - (0.0319788754F *
    rtDW.DiscreteTimeIntegratory_dt_DS_l[1] +
    rtDW.DiscreteTimeIntegratory_DSTAT_e[1]);

  // Sum: '<S58>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S58>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S58>/Discrete-Time Integrator y_dt'
  //   Gain: '<S58>/2*d//omega'
  //   Sum: '<S58>/Sum3'

  rtb_n[1] -= 0.0319788754F * rtDW.DiscreteTimeIntegratory_dt_DS_i[1] +
    rtDW.DiscreteTimeIntegratory_DSTAT_m[1];

  // Sum: '<S60>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S60>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S60>/Discrete-Time Integrator y_dt'
  //   Gain: '<S60>/2*d//omega'
  //   Sum: '<S60>/Sum3'

  rtb_Diff[2] = rtb_Add_i_idx_7 - (0.0319788754F *
    rtDW.DiscreteTimeIntegratory_dt_DS_f[2] +
    rtDW.DiscreteTimeIntegratory_DSTAT_f[2]);

  // Sum: '<S59>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y_dt'
  //   DiscreteIntegrator: '<S59>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S59>/Discrete-Time Integrator y_dt'
  //   Gain: '<S59>/2*d//omega'
  //   Sum: '<S59>/Sum3'

  rtb_Diff_c[2] = rtDW.DiscreteTimeIntegratory_dt_DS_m[2] - (0.0319788754F *
    rtDW.DiscreteTimeIntegratory_dt_DS_l[2] +
    rtDW.DiscreteTimeIntegratory_DSTAT_e[2]);

  // Sum: '<S58>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S58>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S58>/Discrete-Time Integrator y_dt'
  //   Gain: '<S58>/2*d//omega'
  //   Sum: '<S58>/Sum3'

  q0_q1 = rtb_n[2] - (0.0319788754F * rtDW.DiscreteTimeIntegratory_dt_DS_i[2] +
                      rtDW.DiscreteTimeIntegratory_DSTAT_m[2]);

  // Sum: '<S19>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S19>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S19>/Discrete-Time Integrator y_dt'
  //   Gain: '<S19>/2*d//omega'
  //   Sum: '<S19>/Sum3'

  q3_q3 = rtb_nu[2] - (0.13333334F * rtDW.DiscreteTimeIntegratory_dt_D_lz +
                       rtDW.DiscreteTimeIntegratory_DSTAT_i);

  // Update for DiscreteIntegrator: '<S32>/Discrete-Time Integrator' incorporates:
  //   Gain: '<S32>/1//T'
  //   Sum: '<S32>/Sum2'

  rtDW.DiscreteTimeIntegrator_DSTATE[0] += (rtb_y_b[0] -
    rtDW.DiscreteTimeIntegrator_DSTATE[0]) * 35.7142868F * 0.0025F;
  rtDW.DiscreteTimeIntegrator_DSTATE[1] += (rtb_y_b[1] -
    rtDW.DiscreteTimeIntegrator_DSTATE[1]) * 35.7142868F * 0.0025F;
  rtDW.DiscreteTimeIntegrator_DSTATE[2] += (rtb_y_b[2] -
    rtDW.DiscreteTimeIntegrator_DSTATE[2]) * 35.7142868F * 0.0025F;
  rtDW.DiscreteTimeIntegrator_DSTATE[3] += (rtb_Add_i_idx_0 -
    rtDW.DiscreteTimeIntegrator_DSTATE[3]) * 35.7142868F * 0.0025F;
  for (idx = 0; idx < 9; idx++) {
    // Update for DiscreteIntegrator: '<S68>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S68>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTATE[idx] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DSTA[idx];

    // Update for DiscreteIntegrator: '<S68>/Discrete-Time Integrator y_dt'
    rtDW.DiscreteTimeIntegratory_dt_DSTA[idx] += 0.0025F * rtb_omega2[idx];
  }

  // Update for DiscreteIntegrator: '<S7>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_DSTAT_mg += 0.0025F;
  if (q0_q0 > 0.0F) {
    rtDW.DiscreteTimeIntegrator_PrevRese = 1;
  } else if (q0_q0 < 0.0F) {
    rtDW.DiscreteTimeIntegrator_PrevRese = -1;
  } else if (q0_q0 == 0.0F) {
    rtDW.DiscreteTimeIntegrator_PrevRese = 0;
  } else {
    rtDW.DiscreteTimeIntegrator_PrevRese = 2;
  }

  // End of Update for DiscreteIntegrator: '<S7>/Discrete-Time Integrator'

  // Update for DiscreteIntegrator: '<S61>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_IC_LOADI = 0U;

  // Update for DiscreteIntegrator: '<S57>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_DSTATE_i += 0.0025F * q2_q2;

  // Update for DiscreteIntegrator: '<S51>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LOAD = 0U;

  // Update for DiscreteIntegrator: '<S58>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LO_p = 0U;

  // Update for DiscreteIntegrator: '<S59>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LO_n = 0U;

  // Update for DiscreteIntegrator: '<S60>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LO_f = 0U;

  // Update for DiscreteIntegrator: '<S65>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_DSTATE_m[0] += 0.0025F * cmd_V_NED_idx_0;

  // Update for DiscreteIntegrator: '<S61>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_DSTATE_c[0] += 0.0025F * rtb_y_c_idx_0;

  // Update for UnitDelay: '<S11>/UD'
  //
  //  Block description for '<S11>/UD':
  //
  //   Store in Global RAM

  rtDW.UD_DSTATE[0] = rtb_TSamp_idx_0;

  // Update for UnitDelay: '<S12>/UD'
  //
  //  Block description for '<S12>/UD':
  //
  //   Store in Global RAM

  rtDW.UD_DSTATE_b[0] = rtb_TSamp_k_idx_0;

  // Update for DiscreteIntegrator: '<S51>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_h[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_m[0];

  // Update for DiscreteIntegrator: '<S51>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DS_m[0] += 0.0025F * rtb_n_g_des[0];

  // Update for DiscreteIntegrator: '<S58>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S58>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_m[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_i[0];

  // Update for DiscreteIntegrator: '<S59>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S59>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_e[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_l[0];

  // Update for DiscreteIntegrator: '<S60>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S60>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_f[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_f[0];

  // Update for DiscreteIntegrator: '<S65>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_DSTATE_m[1] += 0.0025F * cmd_V_NED_idx_1;

  // Update for DiscreteIntegrator: '<S61>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_DSTATE_c[1] += 0.0025F * rtb_y_c_idx_1;

  // Update for UnitDelay: '<S11>/UD'
  //
  //  Block description for '<S11>/UD':
  //
  //   Store in Global RAM

  rtDW.UD_DSTATE[1] = rtb_TSamp_idx_1;

  // Update for UnitDelay: '<S12>/UD'
  //
  //  Block description for '<S12>/UD':
  //
  //   Store in Global RAM

  rtDW.UD_DSTATE_b[1] = rtb_TSamp_k_idx_1;

  // Update for DiscreteIntegrator: '<S51>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_h[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_m[1];

  // Update for DiscreteIntegrator: '<S51>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DS_m[1] += 0.0025F * rtb_n_g_des[1];

  // Update for DiscreteIntegrator: '<S58>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S58>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_m[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_i[1];

  // Update for DiscreteIntegrator: '<S59>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S59>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_e[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_l[1];

  // Update for DiscreteIntegrator: '<S60>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S60>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_f[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_f[1];

  // Update for DiscreteIntegrator: '<S65>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_DSTATE_m[2] += 0.0025F * cmd_V_NED_idx_2;

  // Update for DiscreteIntegrator: '<S61>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_DSTATE_c[2] += 0.0025F * rtb_y_c_idx_2;

  // Update for UnitDelay: '<S11>/UD'
  //
  //  Block description for '<S11>/UD':
  //
  //   Store in Global RAM

  rtDW.UD_DSTATE[2] = rtb_TSamp;

  // Update for UnitDelay: '<S12>/UD'
  //
  //  Block description for '<S12>/UD':
  //
  //   Store in Global RAM

  rtDW.UD_DSTATE_b[2] = rtb_TSamp_h;

  // Update for DiscreteIntegrator: '<S51>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_h[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_m[2];

  // Update for DiscreteIntegrator: '<S51>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DS_m[2] += 0.0025F * rtb_Add_i_idx_7;

  // Update for DiscreteIntegrator: '<S58>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S58>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_m[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_i[2];

  // Update for DiscreteIntegrator: '<S59>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S59>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_e[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_l[2];

  // Update for DiscreteIntegrator: '<S60>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S60>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_f[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_f[2];

  // Update for DiscreteIntegrator: '<S41>/Discrete-Time Integrator2'
  rtDW.DiscreteTimeIntegrator2_IC_LOAD = 0U;
  rtDW.DiscreteTimeIntegrator2_DSTATE += 0.0025F * q1_q1;

  // Update for DiscreteIntegrator: '<S19>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S19>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_IC_LO_b = 0U;
  rtDW.DiscreteTimeIntegratory_DSTAT_i += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_D_lz;

  // Update for UnitDelay: '<S24>/Unit Delay1'
  rtDW.UnitDelay1_DSTATE[0] = rtb_uDLookupTable2[0];

  // Update for DiscreteIntegrator: '<S26>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S26>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_a[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_n[0];

  // Update for DiscreteIntegrator: '<S26>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S26>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_n[0] += 252661.875F * rtb_Switch[0] *
    0.0025F;

  // Update for Delay: '<S13>/Delay'
  rtDW.Delay_DSTATE[0] = rtb_y_idx_0;

  // Update for UnitDelay: '<S24>/Unit Delay1'
  rtDW.UnitDelay1_DSTATE[1] = rtb_uDLookupTable2[1];

  // Update for DiscreteIntegrator: '<S26>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S26>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_a[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_n[1];

  // Update for DiscreteIntegrator: '<S26>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S26>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_n[1] += 252661.875F * rtb_Switch[1] *
    0.0025F;

  // Update for Delay: '<S13>/Delay'
  rtDW.Delay_DSTATE[1] = rtb_y_idx_1;

  // Update for UnitDelay: '<S24>/Unit Delay1'
  rtDW.UnitDelay1_DSTATE[2] = rtb_uDLookupTable2[2];

  // Update for DiscreteIntegrator: '<S26>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S26>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_a[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_n[2];

  // Update for DiscreteIntegrator: '<S26>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S26>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_n[2] += 252661.875F * rtb_Switch[2] *
    0.0025F;

  // Update for Delay: '<S13>/Delay'
  rtDW.Delay_DSTATE[2] = rtb_y_idx_2;

  // Update for UnitDelay: '<S24>/Unit Delay1'
  rtDW.UnitDelay1_DSTATE[3] = rtb_uDLookupTable2[3];

  // Update for DiscreteIntegrator: '<S26>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S26>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_a[3] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_n[3];

  // Update for DiscreteIntegrator: '<S26>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S26>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_n[3] += 252661.875F * rtb_Switch[3] *
    0.0025F;

  // Update for Delay: '<S13>/Delay'
  rtDW.Delay_DSTATE[3] = rtb_y_idx_3;

  // Update for DiscreteIntegrator: '<S60>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S60>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_f[0] += 3911.41284F * rtb_Diff[0] * 0.0025F;

  // Update for DiscreteIntegrator: '<S59>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S59>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_l[0] += 3911.41284F * rtb_Diff_c[0] *
    0.0025F;

  // Update for DiscreteIntegrator: '<S58>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S58>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_i[0] += 3911.41284F * rtb_n[0] * 0.0025F;

  // Update for DiscreteIntegrator: '<S60>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S60>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_f[1] += 3911.41284F * rtb_Diff[1] * 0.0025F;

  // Update for DiscreteIntegrator: '<S59>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S59>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_l[1] += 3911.41284F * rtb_Diff_c[1] *
    0.0025F;

  // Update for DiscreteIntegrator: '<S58>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S58>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_i[1] += 3911.41284F * rtb_n[1] * 0.0025F;

  // Update for DiscreteIntegrator: '<S60>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S60>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_f[2] += 3911.41284F * rtb_Diff[2] * 0.0025F;

  // Update for DiscreteIntegrator: '<S59>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S59>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_l[2] += 3911.41284F * rtb_Diff_c[2] *
    0.0025F;

  // Update for DiscreteIntegrator: '<S58>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S58>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_i[2] += 3911.41284F * q0_q1 * 0.0025F;

  // Update for DiscreteIntegrator: '<S19>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S19>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_D_lz += 225.0F * q3_q3 * 0.0025F;
}

// Model initialize function
void MatlabControllerClass::initialize()
{
  // InitializeConditions for DiscreteIntegrator: '<S7>/Discrete-Time Integrator' 
  rtDW.DiscreteTimeIntegrator_PrevRese = 2;

  // InitializeConditions for DiscreteIntegrator: '<S61>/Discrete-Time Integrator' 
  rtDW.DiscreteTimeIntegrator_IC_LOADI = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S51>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LOAD = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S58>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LO_p = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S59>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LO_n = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S60>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LO_f = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S41>/Discrete-Time Integrator2' 
  rtDW.DiscreteTimeIntegrator2_IC_LOAD = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S19>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LO_b = 1U;
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
