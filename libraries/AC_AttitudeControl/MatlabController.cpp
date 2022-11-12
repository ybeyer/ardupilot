//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: MatlabController.cpp
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

// Exported data definition

// Const memory section
// Definition for custom storage class: Const
const logConfigBus log_config[5] = { {
    14U,

    { 117U, 49U, 1U, 117U, 50U, 1U, 117U, 51U, 1U, 117U, 52U, 1U, 101U, 120U,
      103U, 101U, 121U, 103U, 101U, 122U, 103U, 101U, 117U, 103U, 101U, 118U,
      103U, 101U, 119U, 103U, 101U, 97U, 120U, 101U, 97U, 121U, 101U, 97U, 122U,
      104U, 49U, 1U },

    { 77U, 76U, 49U, 0U }
  }, { 13U,
    { 104U, 50U, 1U, 120U, 103U, 114U, 121U, 103U, 114U, 122U, 103U, 114U, 120U,
      103U, 109U, 121U, 103U, 109U, 122U, 103U, 109U, 120U, 100U, 49U, 121U,
      100U, 49U, 122U, 100U, 49U, 120U, 100U, 50U, 121U, 100U, 50U, 122U, 100U,
      50U, 0U, 0U, 0U },

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
//    '<S33>/wrap angle'
//    '<S33>/wrap angle1'
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
//    '<S34>/DCM to quaternions'
//    '<S19>/DCM to quaternions'
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

// Function for MATLAB Function: '<S34>/Quaternion Reduced'
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

//
// Output and update for atomic system:
//    '<S34>/Quaternion Reduced'
//    '<S19>/Quaternion Reduced'
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
    if (1.0F > q_yaw[0]) {
      M_bg_idx_8 = q_yaw[0];
    } else {
      M_bg_idx_8 = 1.0F;
    }

    if (-1.0F >= M_bg_idx_8) {
      M_bg_idx_8 = -1.0F;
    }

    *rty_yaw = 2.0F * std::acos(M_bg_idx_8);
  } else {
    if (1.0F > -q_yaw[0]) {
      M_bg_idx_8 = -q_yaw[0];
    } else {
      M_bg_idx_8 = 1.0F;
    }

    if (-1.0F >= M_bg_idx_8) {
      M_bg_idx_8 = -1.0F;
    }

    *rty_yaw = 2.0F * std::acos(M_bg_idx_8);
  }
}

//
// Output and update for atomic system:
//    '<S38>/Lean Vector Derivative Trafo'
//    '<S38>/Lean Vector Derivative Trafo Delay'
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

// Function for MATLAB Function: '<S11>/INDI Copter Acc 2 Lean Vector'
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

// Function for MATLAB Function: '<S49>/n ref norm'
real32_T MatlabControllerClass::divideFinite(real32_T B_5)
{
  real32_T b;
  int32_T trueCount;
  b = B_5;
  trueCount = 0;
  if (std::abs(B_5) < 2.22044605E-16F) {
    trueCount = 1;
  }

  if (0 <= trueCount - 1) {
    b = 2.22044605E-16F;
  }

  return 1.0F / b;
}

// Function for MATLAB Function: '<S24>/MATLAB Function2'
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

// Function for MATLAB Function: '<S24>/MATLAB Function2'
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

// Function for MATLAB Function: '<S24>/MATLAB Function2'
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

// Function for MATLAB Function: '<S24>/MATLAB Function2'
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

// Function for MATLAB Function: '<S24>/MATLAB Function2'
void MatlabControllerClass::mldivide(const real32_T A_data[], const int32_T
  A_size[2], const real32_T B_0[8], real32_T Y_data[], int32_T *Y_size)
{
  if (A_size[1] == 0) {
    *Y_size = 0;
  } else {
    qrsolve(A_data, A_size, B_0, Y_data, Y_size);
  }
}

// Function for MATLAB Function: '<S24>/MATLAB Function2'
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

// Function for MATLAB Function: '<S24>/MATLAB Function2'
real32_T MatlabControllerClass::wls_alloc(const real32_T B_4[16], const real32_T
  v[4], const real32_T umin[4], const real32_T umax[4], const real32_T Wv[16],
  const real32_T Wu[16], const real32_T ud[4], real32_T gam, real32_T u[4],
  real32_T W[4], real32_T imax)
{
  real32_T iter;
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
    A[4 + A_tmp_tmp_0] = Wu[A_tmp_tmp];
    A_tmp_tmp_1 = A_tmp_tmp + 1;
    A[1 + A_tmp_tmp_0] = A_tmp_0[A_tmp_tmp_1];
    A[5 + A_tmp_tmp_0] = Wu[A_tmp_tmp_1];
    A_tmp_tmp_1 = A_tmp_tmp + 2;
    A[2 + A_tmp_tmp_0] = A_tmp_0[A_tmp_tmp_1];
    A[6 + A_tmp_tmp_0] = Wu[A_tmp_tmp_1];
    A_tmp_tmp += 3;
    A[3 + A_tmp_tmp_0] = A_tmp_0[A_tmp_tmp];
    A[7 + A_tmp_tmp_0] = Wu[A_tmp_tmp];
    gam_sq = A_tmp[b_k + 12] * v[3] + (A_tmp[b_k + 8] * v[2] + (A_tmp[b_k + 4] *
      v[1] + A_tmp[b_k] * v[0]));
    Wu_0 = Wu[b_k + 12] * ud[3] + (Wu[b_k + 8] * ud[2] + (Wu[b_k + 4] * ud[1] +
      Wu[b_k] * ud[0]));
    A_tmp_1[b_k] = gam_sq;
    A_tmp_1[b_k + 4] = Wu_0;
  }

  for (b_k = 0; b_k < 8; b_k++) {
    gam_sq = A[b_k + 24] * u[3] + (A[b_k + 16] * u[2] + (A[b_k + 8] * u[1] +
      A[b_k] * u[0]));
    d[b_k] = A_tmp_1[b_k] - gam_sq;
  }

  i_free[0] = (W[0] == 0.0F);
  i_free[1] = (W[1] == 0.0F);
  i_free[2] = (W[2] == 0.0F);
  i_free[3] = (W[3] == 0.0F);
  iter = 1.0F;
  A_tmp_tmp = 0;
  exitg1 = false;
  while ((!exitg1) && (A_tmp_tmp <= (int32_T)imax - 1)) {
    iter = 1.0F + (real32_T)A_tmp_tmp;
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

        gam_sq = W[A_tmp_tmp_0] * p_free_data[A_tmp_tmp_0];
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

        W[A_tmp_tmp_1] = 0.0F;
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
        W[A_tmp_tmp_1] = -1.0F;
      } else if (p[A_tmp_tmp_1] > 0.0) {
        W[A_tmp_tmp_1] = 1.0F;
      } else {
        W[A_tmp_tmp_1] = (real32_T)p[A_tmp_tmp_1];
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
  boolean_T varargin_1[10];
  int32_T idx;
  int32_T k;
  real32_T q0_q0;
  real32_T q1_q1;
  real32_T q2_q2;
  real32_T q0_q3;
  real32_T q1_q2;
  real32_T q1_q3;
  real32_T q2_q3;
  real32_T scale;
  real32_T absxk;
  real32_T t;
  real32_T cmd_V_NED[3];
  real32_T b;
  real32_T omega[4];
  real32_T G_omega[16];
  real32_T b_d[16];
  real32_T rtb_uDLookupTable;
  real32_T rtb_Switch[4];
  real32_T rtb_Add_h[9];
  real32_T rtb_y_b[3];
  real32_T rtb_y_dt_mq[3];
  real32_T rtb_Sum2_c[3];
  real32_T rtb_Max[4];
  real32_T rtb_n_g_des[3];
  real32_T rtb_n_dt2_h[3];
  real32_T rtb_n_dt_d[3];
  real32_T rtb_n_dt2[3];
  real32_T rtb_n[3];
  real32_T rtb_n_dt[3];
  real32_T rtb_n_b[3];
  real32_T rtb_n_b_dt[3];
  real32_T rtb_uDLookupTable2[4];
  real32_T rtb_M_bg[9];
  real32_T rtb_y_dl[9];
  real32_T rtb_G2[16];
  int32_T i;
  real32_T tmp[16];
  real32_T tmp_0[4];
  real32_T q2_q2_0[9];
  real32_T tmp_1[16];
  real32_T tmp_2[4];
  real32_T rtb_n_g_des_a;
  real32_T rtb_omega2_e_idx_2;
  real32_T rtb_omega2_e_idx_1;
  real32_T rtb_omega2_e_idx_0;
  boolean_T rtb_LowerRelop1_idx_1;
  boolean_T rtb_LowerRelop1_idx_0;
  real_T y;
  real32_T q2_q2_tmp;
  real32_T q2_q2_tmp_tmp;
  real32_T q2_q2_tmp_0;
  real32_T q2_q2_tmp_tmp_0;
  real32_T n_dt2_idx_0_tmp;
  real32_T n_dt2_idx_0_tmp_0;
  real32_T n_dt2_idx_0_tmp_1;
  real32_T n_dt2_idx_0_tmp_2;
  real32_T n_dt2_idx_0_tmp_3;
  real32_T n_dt2_idx_1_tmp;
  int32_T rtb_G2_tmp;
  int32_T rtb_G2_tmp_0;

  // DiscreteIntegrator: '<S72>/Discrete-Time Integrator y' incorporates:
  //   Inport: '<Root>/measure'

  if (rtDW.DiscreteTimeIntegratory_IC_LOAD != 0) {
    rtDW.DiscreteTimeIntegratory_DSTATE[0] = rtU.measure.s_Kg[0];
    rtDW.DiscreteTimeIntegratory_DSTATE[1] = rtU.measure.s_Kg[1];
    rtDW.DiscreteTimeIntegratory_DSTATE[2] = rtU.measure.s_Kg[2];
  }

  // DiscreteIntegrator: '<S55>/Discrete-Time Integrator' incorporates:
  //   DiscreteIntegrator: '<S72>/Discrete-Time Integrator y'

  if (rtDW.DiscreteTimeIntegrator_IC_LOADI != 0) {
    rtDW.DiscreteTimeIntegrator_DSTATE[0] = rtDW.DiscreteTimeIntegratory_DSTATE
      [0];
    rtDW.DiscreteTimeIntegrator_DSTATE[1] = rtDW.DiscreteTimeIntegratory_DSTATE
      [1];
    rtDW.DiscreteTimeIntegrator_DSTATE[2] = rtDW.DiscreteTimeIntegratory_DSTATE
      [2];
  }

  // DiscreteIntegrator: '<S71>/Discrete-Time Integrator y' incorporates:
  //   Inport: '<Root>/measure'

  if (rtDW.DiscreteTimeIntegratory_IC_LO_f != 0) {
    rtDW.DiscreteTimeIntegratory_DSTAT_d[0] = rtU.measure.V_Kg[0];
    rtDW.DiscreteTimeIntegratory_DSTAT_d[1] = rtU.measure.V_Kg[1];
    rtDW.DiscreteTimeIntegratory_DSTAT_d[2] = rtU.measure.V_Kg[2];
  }

  // DiscreteIntegrator: '<S70>/Discrete-Time Integrator y' incorporates:
  //   Inport: '<Root>/measure'
  //   Sum: '<Root>/Add'

  if (rtDW.DiscreteTimeIntegratory_IC_LO_i != 0) {
    rtDW.DiscreteTimeIntegratory_DSTAT_b[0] = rtU.measure.a_Kg[0];
    rtDW.DiscreteTimeIntegratory_DSTAT_b[1] = rtU.measure.a_Kg[1];
    rtDW.DiscreteTimeIntegratory_DSTAT_b[2] = rtU.measure.a_Kg[2] + 9.81F;
  }

  // Lookup_n-D: '<Root>/1-D Lookup Table' incorporates:
  //   Inport: '<Root>/cmd'

  rtb_uDLookupTable = look1_iflf_binlx(rtU.cmd.RC_pwm[8], rtConstP.pooled6,
    rtConstP.pooled5, 1U);

  // DiscreteIntegrator: '<S7>/Discrete-Time Integrator'
  if ((rtb_uDLookupTable > 0.0F) && (rtDW.DiscreteTimeIntegrator_PrevRe_h <= 0))
  {
    rtDW.DiscreteTimeIntegrator_DSTATE_m = 0.0F;
  }

  // MATLAB Function: '<S7>/interpHold' incorporates:
  //   Constant: '<S7>/Constant1'
  //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator'

  for (i = 0; i < 10; i++) {
    varargin_1[i] = (rtConstP.Constant1_Value[i] >
                     rtDW.DiscreteTimeIntegrator_DSTATE_m);
  }

  idx = -1;
  rtb_LowerRelop1_idx_0 = varargin_1[0];
  for (k = 0; k < 9; k++) {
    rtb_LowerRelop1_idx_1 = varargin_1[k + 1];
    if ((int32_T)rtb_LowerRelop1_idx_0 < (int32_T)rtb_LowerRelop1_idx_1) {
      rtb_LowerRelop1_idx_0 = rtb_LowerRelop1_idx_1;
      idx = k;
    }
  }

  // Switch: '<S7>/Switch' incorporates:
  //   Constant: '<S7>/Constant2'
  //   Inport: '<Root>/cmd'
  //   MATLAB Function: '<S7>/interpHold'

  if (rtb_uDLookupTable > 0.5F) {
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

  // MATLAB Function: '<Root>/Quaternions to Rotation Matrix' incorporates:
  //   Inport: '<Root>/measure'

  scale = 1.29246971E-26F;
  absxk = std::abs(rtU.measure.q_bg[0]);
  if (absxk > 1.29246971E-26F) {
    q2_q2 = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    q2_q2 = t * t;
  }

  absxk = std::abs(rtU.measure.q_bg[1]);
  if (absxk > scale) {
    t = scale / absxk;
    q2_q2 = q2_q2 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q2_q2 += t * t;
  }

  absxk = std::abs(rtU.measure.q_bg[2]);
  if (absxk > scale) {
    t = scale / absxk;
    q2_q2 = q2_q2 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q2_q2 += t * t;
  }

  absxk = std::abs(rtU.measure.q_bg[3]);
  if (absxk > scale) {
    t = scale / absxk;
    q2_q2 = q2_q2 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q2_q2 += t * t;
  }

  q2_q2 = scale * std::sqrt(q2_q2);
  if (2.22044605E-16F < q2_q2) {
    q0_q0 = q2_q2;
  } else {
    q0_q0 = 2.22044605E-16F;
  }

  rtb_Max[0] = rtU.measure.q_bg[0] / q0_q0;
  rtb_Max[1] = rtU.measure.q_bg[1] / q0_q0;
  rtb_Max[2] = rtU.measure.q_bg[2] / q0_q0;
  rtb_Max[3] = rtU.measure.q_bg[3] / q0_q0;
  q0_q0 = rtb_Max[0] * rtb_Max[0];
  q1_q1 = rtb_Max[1] * rtb_Max[1];
  q2_q2 = rtb_Max[2] * rtb_Max[2];
  scale = rtb_Max[3] * rtb_Max[3];
  absxk = rtb_Max[0] * rtb_Max[1];
  t = rtb_Max[0] * rtb_Max[2];
  q0_q3 = rtb_Max[0] * rtb_Max[3];
  q1_q2 = rtb_Max[1] * rtb_Max[2];
  q1_q3 = rtb_Max[1] * rtb_Max[3];
  q2_q3 = rtb_Max[2] * rtb_Max[3];
  rtb_M_bg[0] = ((q0_q0 + q1_q1) - q2_q2) - scale;
  rtb_M_bg[3] = (q1_q2 + q0_q3) * 2.0F;
  rtb_M_bg[6] = (q1_q3 - t) * 2.0F;
  rtb_M_bg[1] = (q1_q2 - q0_q3) * 2.0F;
  q0_q0 -= q1_q1;
  rtb_M_bg[4] = (q0_q0 + q2_q2) - scale;
  rtb_M_bg[7] = (q2_q3 + absxk) * 2.0F;
  rtb_M_bg[2] = (q1_q3 + t) * 2.0F;
  rtb_M_bg[5] = (q2_q3 - absxk) * 2.0F;
  rtb_M_bg[8] = (q0_q0 - q2_q2) + scale;

  // End of MATLAB Function: '<Root>/Quaternions to Rotation Matrix'

  // Gain: '<S55>/Gain3' incorporates:
  //   Gain: '<Root>/Gain'
  //   MATLAB Function: '<S16>/MATLAB Function'
  //   MATLAB Function: '<S55>/MATLAB Function1'

  q0_q0 = std::sqrt(rtb_Switch[0] * rtb_Switch[0] + -rtb_Switch[1] *
                    -rtb_Switch[1]) * 13.8162899F;

  // Product: '<S55>/Product' incorporates:
  //   Gain: '<Root>/Gain'
  //   MATLAB Function: '<S16>/MATLAB Function'

  scale = -rtb_Switch[1] * q0_q0;
  q1_q1 = rtb_Switch[0] * q0_q0;

  // Lookup_n-D: '<S55>/1-D Lookup Table' incorporates:
  //   Gain: '<Root>/Gain5'

  q0_q0 = look1_iflf_binlx(-rtb_Switch[3], rtConstP.uDLookupTable_bp01Data,
    rtConstP.uDLookupTable_tableData, 2U);

  // DiscreteIntegrator: '<S59>/Discrete-Time Integrator'
  if (rtDW.DiscreteTimeIntegrator_IC_LOA_j != 0) {
    rtDW.DiscreteTimeIntegrator_DSTAT_my[0] = scale;
    rtDW.DiscreteTimeIntegrator_DSTAT_my[1] = q1_q1;
    rtDW.DiscreteTimeIntegrator_DSTAT_my[2] = q0_q0;
  }

  // Gain: '<S59>/1//T' incorporates:
  //   DiscreteIntegrator: '<S59>/Discrete-Time Integrator'
  //   Sum: '<S59>/Sum2'

  t = (scale - rtDW.DiscreteTimeIntegrator_DSTAT_my[0]) * 1.64418423F;

  // Saturate: '<S59>/Saturation'
  if (t > 43.685627F) {
    q1_q2 = 43.685627F;
    cmd_V_NED[0] = 43.685627F;
  } else if (t < -43.685627F) {
    q1_q2 = -43.685627F;
    cmd_V_NED[0] = -43.685627F;
  } else {
    q1_q2 = t;
    cmd_V_NED[0] = t;
  }

  // Gain: '<S59>/1//T' incorporates:
  //   DiscreteIntegrator: '<S59>/Discrete-Time Integrator'
  //   Sum: '<S59>/Sum2'

  t = (q1_q1 - rtDW.DiscreteTimeIntegrator_DSTAT_my[1]) * 1.64418423F;

  // Saturate: '<S59>/Saturation'
  if (t > 43.685627F) {
    q2_q3 = 43.685627F;
    cmd_V_NED[1] = 43.685627F;
  } else if (t < -43.685627F) {
    q2_q3 = -43.685627F;
    cmd_V_NED[1] = -43.685627F;
  } else {
    q2_q3 = t;
    cmd_V_NED[1] = t;
  }

  // Gain: '<S59>/1//T' incorporates:
  //   DiscreteIntegrator: '<S59>/Discrete-Time Integrator'
  //   Sum: '<S59>/Sum2'

  t = (q0_q0 - rtDW.DiscreteTimeIntegrator_DSTAT_my[2]) * 1.64418423F;

  // Saturate: '<S59>/Saturation'
  if (t > 5.74600601F) {
    q1_q3 = 5.74600601F;
    cmd_V_NED[2] = 5.74600601F;
  } else if (t < -38.3970413F) {
    q1_q3 = -38.3970413F;
    cmd_V_NED[2] = -38.3970413F;
  } else {
    q1_q3 = t;
    cmd_V_NED[2] = t;
  }

  // DiscreteIntegrator: '<S62>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S55>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<S59>/Discrete-Time Integrator'
  //   Saturate: '<S59>/Saturation'

  if (rtDW.DiscreteTimeIntegratory_IC_LO_b != 0) {
    rtDW.DiscreteTimeIntegratory_DSTAT_l[0] =
      rtDW.DiscreteTimeIntegrator_DSTATE[0];
    rtDW.DiscreteTimeIntegratory_DSTAT_l[3] =
      rtDW.DiscreteTimeIntegrator_DSTAT_my[0];
    rtDW.DiscreteTimeIntegratory_DSTAT_l[6] = q1_q2;
    rtDW.DiscreteTimeIntegratory_DSTAT_l[1] =
      rtDW.DiscreteTimeIntegrator_DSTATE[1];
    rtDW.DiscreteTimeIntegratory_DSTAT_l[4] =
      rtDW.DiscreteTimeIntegrator_DSTAT_my[1];
    rtDW.DiscreteTimeIntegratory_DSTAT_l[7] = q2_q3;
    rtDW.DiscreteTimeIntegratory_DSTAT_l[2] =
      rtDW.DiscreteTimeIntegrator_DSTATE[2];
    rtDW.DiscreteTimeIntegratory_DSTAT_l[5] =
      rtDW.DiscreteTimeIntegrator_DSTAT_my[2];
    rtDW.DiscreteTimeIntegratory_DSTAT_l[8] = q1_q3;
  }

  // Sum: '<S61>/Add' incorporates:
  //   DiscreteIntegrator: '<S62>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S71>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S72>/Discrete-Time Integrator y'

  rtb_Add_h[0] = rtDW.DiscreteTimeIntegratory_DSTAT_l[0] -
    rtDW.DiscreteTimeIntegratory_DSTATE[0];
  rtb_Add_h[3] = rtDW.DiscreteTimeIntegratory_DSTAT_l[3] -
    rtDW.DiscreteTimeIntegratory_DSTAT_d[0];
  rtb_Add_h[6] = rtDW.DiscreteTimeIntegratory_DSTAT_l[6] -
    rtDW.DiscreteTimeIntegratory_DSTAT_b[0];
  rtb_Add_h[1] = rtDW.DiscreteTimeIntegratory_DSTAT_l[1] -
    rtDW.DiscreteTimeIntegratory_DSTATE[1];
  rtb_Add_h[4] = rtDW.DiscreteTimeIntegratory_DSTAT_l[4] -
    rtDW.DiscreteTimeIntegratory_DSTAT_d[1];
  rtb_Add_h[7] = rtDW.DiscreteTimeIntegratory_DSTAT_l[7] -
    rtDW.DiscreteTimeIntegratory_DSTAT_b[1];
  rtb_Add_h[2] = rtDW.DiscreteTimeIntegratory_DSTAT_l[2] -
    rtDW.DiscreteTimeIntegratory_DSTATE[2];
  rtb_Add_h[5] = rtDW.DiscreteTimeIntegratory_DSTAT_l[5] -
    rtDW.DiscreteTimeIntegratory_DSTAT_d[2];
  rtb_Add_h[8] = rtDW.DiscreteTimeIntegratory_DSTAT_l[8] -
    rtDW.DiscreteTimeIntegratory_DSTAT_b[2];

  // DiscreteIntegrator: '<S69>/Discrete-Time Integrator y'
  if (rtDW.DiscreteTimeIntegratory_IC_LO_g != 0) {
    for (i = 0; i < 9; i++) {
      rtDW.DiscreteTimeIntegratory_DSTAT_k[i] = rtb_M_bg[i];
    }
  }

  for (i = 0; i < 9; i++) {
    rtb_y_dl[i] = rtDW.DiscreteTimeIntegratory_DSTAT_k[i];
  }

  // Switch: '<S63>/Switch2' incorporates:
  //   RelationalOperator: '<S63>/LowerRelop1'
  //   RelationalOperator: '<S63>/UpperRelop'
  //   Switch: '<S63>/Switch'

  if (rtb_Add_h[0] > 11.0167017F) {
    q0_q0 = 11.0167017F;
  } else if (rtb_Add_h[0] < -11.0167017F) {
    // Switch: '<S63>/Switch'
    q0_q0 = -11.0167017F;
  } else {
    q0_q0 = rtb_Add_h[0];
  }

  // Sum: '<S61>/Add1' incorporates:
  //   Gain: '<S61>/Gain'
  //   Gain: '<S61>/Gain3'
  //   Gain: '<S61>/Gain4'
  //   RelationalOperator: '<S63>/LowerRelop1'

  q0_q0 = (3.46807289F * q0_q0 + 3.95048451F * rtb_Add_h[3]) + 0.5F * rtb_Add_h
    [6];

  // RelationalOperator: '<S64>/LowerRelop1'
  rtb_LowerRelop1_idx_0 = (q0_q0 > 32.7642212F);

  // Gain: '<S61>/Gain4'
  rtb_Sum2_c[0] = q0_q0;

  // Switch: '<S63>/Switch2' incorporates:
  //   RelationalOperator: '<S63>/LowerRelop1'
  //   RelationalOperator: '<S63>/UpperRelop'
  //   Switch: '<S63>/Switch'

  if (rtb_Add_h[1] > 11.0167017F) {
    q0_q0 = 11.0167017F;
  } else if (rtb_Add_h[1] < -11.0167017F) {
    // Switch: '<S63>/Switch'
    q0_q0 = -11.0167017F;
  } else {
    q0_q0 = rtb_Add_h[1];
  }

  // Sum: '<S61>/Add1' incorporates:
  //   Gain: '<S61>/Gain'
  //   Gain: '<S61>/Gain3'
  //   Gain: '<S61>/Gain4'
  //   RelationalOperator: '<S63>/LowerRelop1'

  q0_q0 = (3.46807289F * q0_q0 + 3.95048451F * rtb_Add_h[4]) + 0.5F * rtb_Add_h
    [7];

  // RelationalOperator: '<S64>/LowerRelop1'
  rtb_LowerRelop1_idx_1 = (q0_q0 > 32.7642212F);

  // Gain: '<S61>/Gain4'
  rtb_Sum2_c[1] = q0_q0;

  // Switch: '<S63>/Switch2' incorporates:
  //   RelationalOperator: '<S63>/LowerRelop1'
  //   RelationalOperator: '<S63>/UpperRelop'
  //   Switch: '<S63>/Switch'

  if (rtb_Add_h[2] > 11.0167017F) {
    q0_q0 = 11.0167017F;
  } else if (rtb_Add_h[2] < -11.0167017F) {
    // Switch: '<S63>/Switch'
    q0_q0 = -11.0167017F;
  } else {
    q0_q0 = rtb_Add_h[2];
  }

  // Sum: '<S61>/Add1' incorporates:
  //   Gain: '<S61>/Gain'
  //   Gain: '<S61>/Gain3'
  //   Gain: '<S61>/Gain4'
  //   RelationalOperator: '<S63>/LowerRelop1'

  q0_q0 = (3.46807289F * q0_q0 + 3.95048451F * rtb_Add_h[5]) + 0.5F * rtb_Add_h
    [8];

  // RelationalOperator: '<S64>/UpperRelop'
  q2_q2 = rtb_Sum2_c[0];

  // Switch: '<S64>/Switch' incorporates:
  //   RelationalOperator: '<S64>/UpperRelop'

  if (rtb_Sum2_c[0] < -32.7642212F) {
    q2_q2 = -32.7642212F;
  }

  // Switch: '<S64>/Switch2'
  if (rtb_LowerRelop1_idx_0) {
    q2_q2 = 32.7642212F;
  }

  // MATLAB Function: '<S11>/INDI Copter Acc 2 Lean Vector' incorporates:
  //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y'

  rtb_n[0] = rtDW.DiscreteTimeIntegratory_DSTAT_b[0];

  // Switch: '<S64>/Switch' incorporates:
  //   RelationalOperator: '<S64>/UpperRelop'

  rtb_Sum2_c[0] = q2_q2;

  // RelationalOperator: '<S64>/UpperRelop'
  q2_q2 = rtb_Sum2_c[1];

  // Switch: '<S64>/Switch' incorporates:
  //   RelationalOperator: '<S64>/UpperRelop'

  if (rtb_Sum2_c[1] < -32.7642212F) {
    q2_q2 = -32.7642212F;
  }

  // Switch: '<S64>/Switch2'
  if (rtb_LowerRelop1_idx_1) {
    q2_q2 = 32.7642212F;
  }

  // MATLAB Function: '<S11>/INDI Copter Acc 2 Lean Vector' incorporates:
  //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y'

  rtb_n[1] = rtDW.DiscreteTimeIntegratory_DSTAT_b[1];

  // Switch: '<S64>/Switch' incorporates:
  //   RelationalOperator: '<S64>/UpperRelop'

  rtb_Sum2_c[1] = q2_q2;
  rtb_Sum2_c[2] = q0_q0;
  if (q0_q0 < -32.7642212F) {
    rtb_Sum2_c[2] = -32.7642212F;
  }

  // Switch: '<S64>/Switch2' incorporates:
  //   RelationalOperator: '<S64>/LowerRelop1'

  if (q0_q0 > 32.7642212F) {
    // Switch: '<S64>/Switch' incorporates:
    //   RelationalOperator: '<S64>/UpperRelop'

    rtb_Sum2_c[2] = 32.7642212F;
  }

  // MATLAB Function: '<S11>/INDI Copter Acc 2 Lean Vector' incorporates:
  //   DiscreteIntegrator: '<S69>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y'
  //   Sum: '<S56>/Add1'

  rtb_n[2] = rtDW.DiscreteTimeIntegratory_DSTAT_b[2] - 9.81F;
  q0_q0 = norm(rtb_n);
  for (i = 0; i < 3; i++) {
    rtb_n_g_des[i] = ((cmd_V_NED[i] + rtb_Sum2_c[i]) -
                      rtDW.DiscreteTimeIntegratory_DSTAT_b[i]) +
      -rtDW.DiscreteTimeIntegratory_DSTAT_k[3 * i + 2] * q0_q0;
  }

  q0_q0 = norm(rtb_n_g_des);
  scale = q0_q0;
  idx = 0;
  if (std::abs(q0_q0) < 2.22044605E-16F) {
    idx = 1;
  }

  if (0 <= idx - 1) {
    scale = 2.22044605E-16F;
  }

  q2_q2 = 1.0F / scale;
  rtb_n_g_des[0] *= q2_q2;
  rtb_n_g_des[1] *= q2_q2;
  rtb_n_g_des_a = q2_q2 * rtb_n_g_des[2];

  // MATLAB Function: '<S11>/MATLAB Function4'
  if (1.0F > -rtb_n_g_des_a) {
    q2_q2 = -rtb_n_g_des_a;
  } else {
    q2_q2 = 1.0F;
  }

  if (-1.0F >= q2_q2) {
    q2_q2 = -1.0F;
  }

  t = std::acos(q2_q2);

  // Saturate: '<S11>/Saturation1'
  if (t > 6.28318548F) {
    t = 6.28318548F;
  } else {
    if (t < 0.0F) {
      t = 0.0F;
    }
  }

  // End of Saturate: '<S11>/Saturation1'

  // Gain: '<S38>/lean_angle_max' incorporates:
  //   Gain: '<S11>/Gain1'
  //   MATLAB Function: '<S11>/MATLAB Function4'

  scale = 0.159154937F * t * 6.28318548F;
  q1_q1 = std::atan2(rtb_n_g_des[1], rtb_n_g_des[0]);

  // MATLAB Function: '<S38>/lean angles 2 lean vector'
  q2_q2 = std::sin(scale);
  rtb_n_g_des[0] = q2_q2 * std::cos(q1_q1);
  rtb_n_g_des[1] = q2_q2 * std::sin(q1_q1);
  rtb_n_g_des[2] = -std::cos(scale);

  // DiscreteIntegrator: '<S49>/Discrete-Time Integrator y'
  if (rtDW.DiscreteTimeIntegratory_IC_LO_m != 0) {
    rtDW.DiscreteTimeIntegratory_DSTAT_j[0] = rtb_n_g_des[0];
    rtDW.DiscreteTimeIntegratory_DSTAT_j[1] = rtb_n_g_des[1];
    rtDW.DiscreteTimeIntegratory_DSTAT_j[2] = rtb_n_g_des[2];
  }

  // MATLAB Function: '<S49>/n ref norm' incorporates:
  //   DiscreteIntegrator: '<S49>/Discrete-Time Integrator y'

  scale = 1.29246971E-26F;
  absxk = std::abs(rtDW.DiscreteTimeIntegratory_DSTAT_j[0]);
  if (absxk > 1.29246971E-26F) {
    q2_q2 = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    q2_q2 = t * t;
  }

  // Gain: '<S49>/omega^2' incorporates:
  //   DiscreteIntegrator: '<S49>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S49>/Discrete-Time Integrator y_dt'
  //   Gain: '<S49>/2*d//omega'
  //   Sum: '<S49>/Sum2'
  //   Sum: '<S49>/Sum3'

  rtb_n_g_des[0] = (rtb_n_g_des[0] - (0.137113988F *
    rtDW.DiscreteTimeIntegratory_dt_DSTA[0] +
    rtDW.DiscreteTimeIntegratory_DSTAT_j[0])) * 212.763184F;

  // MATLAB Function: '<S49>/n ref norm' incorporates:
  //   DiscreteIntegrator: '<S49>/Discrete-Time Integrator y'

  absxk = std::abs(rtDW.DiscreteTimeIntegratory_DSTAT_j[1]);
  if (absxk > scale) {
    t = scale / absxk;
    q2_q2 = q2_q2 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q2_q2 += t * t;
  }

  // Gain: '<S49>/omega^2' incorporates:
  //   DiscreteIntegrator: '<S49>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S49>/Discrete-Time Integrator y_dt'
  //   Gain: '<S49>/2*d//omega'
  //   Sum: '<S49>/Sum2'
  //   Sum: '<S49>/Sum3'

  rtb_n_g_des[1] = (rtb_n_g_des[1] - (0.137113988F *
    rtDW.DiscreteTimeIntegratory_dt_DSTA[1] +
    rtDW.DiscreteTimeIntegratory_DSTAT_j[1])) * 212.763184F;
  rtb_n_g_des_a = (rtb_n_g_des[2] - (0.137113988F *
    rtDW.DiscreteTimeIntegratory_dt_DSTA[2] +
    rtDW.DiscreteTimeIntegratory_DSTAT_j[2])) * 212.763184F;

  // MATLAB Function: '<S49>/n ref norm' incorporates:
  //   DiscreteIntegrator: '<S49>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S49>/Discrete-Time Integrator y_dt'

  absxk = std::abs(rtDW.DiscreteTimeIntegratory_DSTAT_j[2]);
  if (absxk > scale) {
    t = scale / absxk;
    q2_q2 = q2_q2 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q2_q2 += t * t;
  }

  q2_q2 = scale * std::sqrt(q2_q2);
  b = q2_q2;
  idx = 0;
  if (q2_q2 < 2.22044605E-16F) {
    idx = 1;
  }

  if (0 <= idx - 1) {
    b = 2.22044605E-16F;
  }

  q1_q1 = rtDW.DiscreteTimeIntegratory_DSTAT_j[1] *
    rtDW.DiscreteTimeIntegratory_DSTAT_j[1];
  scale = rtDW.DiscreteTimeIntegratory_DSTAT_j[2] *
    rtDW.DiscreteTimeIntegratory_DSTAT_j[2];
  absxk = rtDW.DiscreteTimeIntegratory_DSTAT_j[0] *
    rtDW.DiscreteTimeIntegratory_DSTAT_j[0];
  t = absxk + q1_q1;
  q0_q3 = t + scale;
  q2_q2_tmp_0 = divideFinite(std::pow(q0_q3, 1.5F));
  q2_q2_0[0] = (q1_q1 + scale) * q2_q2_tmp_0;
  q2_q2 = -rtDW.DiscreteTimeIntegratory_DSTAT_j[0] *
    rtDW.DiscreteTimeIntegratory_DSTAT_j[1] * q2_q2_tmp_0;
  q2_q2_0[3] = q2_q2;
  q2_q2_tmp_tmp = -rtDW.DiscreteTimeIntegratory_DSTAT_j[0] *
    rtDW.DiscreteTimeIntegratory_DSTAT_j[2];
  q2_q2_tmp = q2_q2_tmp_tmp * q2_q2_tmp_0;
  q2_q2_0[6] = q2_q2_tmp;
  q2_q2_0[1] = q2_q2;
  q2_q2_0[4] = (absxk + scale) * q2_q2_tmp_0;
  q2_q2 = -rtDW.DiscreteTimeIntegratory_DSTAT_j[1] *
    rtDW.DiscreteTimeIntegratory_DSTAT_j[2] * q2_q2_tmp_0;
  q2_q2_0[7] = q2_q2;
  q2_q2_0[2] = q2_q2_tmp;
  q2_q2_0[5] = q2_q2;
  q2_q2_0[8] = t * q2_q2_tmp_0;
  for (i = 0; i < 3; i++) {
    cmd_V_NED[i] = rtDW.DiscreteTimeIntegratory_DSTAT_j[i] / b;
    rtb_n_dt_d[i] = q2_q2_0[i + 6] * rtDW.DiscreteTimeIntegratory_dt_DSTA[2] +
      (q2_q2_0[i + 3] * rtDW.DiscreteTimeIntegratory_dt_DSTA[1] + q2_q2_0[i] *
       rtDW.DiscreteTimeIntegratory_dt_DSTA[0]);
  }

  rtb_n_dt_d[2] = ((q2_q2_tmp_tmp * rtDW.DiscreteTimeIntegratory_dt_DSTA[0] +
                    absxk * rtDW.DiscreteTimeIntegratory_dt_DSTA[2]) +
                   (rtDW.DiscreteTimeIntegratory_DSTAT_j[1] *
                    rtDW.DiscreteTimeIntegratory_dt_DSTA[2] -
                    rtDW.DiscreteTimeIntegratory_DSTAT_j[2] *
                    rtDW.DiscreteTimeIntegratory_dt_DSTA[1]) *
                   rtDW.DiscreteTimeIntegratory_DSTAT_j[1]) * q2_q2_tmp_0;
  q2_q2_tmp_tmp = 2.0F * rtDW.DiscreteTimeIntegratory_DSTAT_j[1] *
    rtDW.DiscreteTimeIntegratory_dt_DSTA[1];
  b = 2.0F * rtDW.DiscreteTimeIntegratory_DSTAT_j[0] *
    rtDW.DiscreteTimeIntegratory_dt_DSTA[0];
  q2_q2_tmp_tmp_0 = 2.0F * rtDW.DiscreteTimeIntegratory_DSTAT_j[2] *
    rtDW.DiscreteTimeIntegratory_dt_DSTA[2];
  q2_q2_tmp_0 = (b + q2_q2_tmp_tmp) + q2_q2_tmp_tmp_0;
  q2_q2_tmp = 2.0F * rtDW.DiscreteTimeIntegratory_DSTAT_j[1] * rtb_n_g_des[1];
  n_dt2_idx_0_tmp = rtDW.DiscreteTimeIntegratory_dt_DSTA[1] *
    rtDW.DiscreteTimeIntegratory_dt_DSTA[1] * 2.0F;
  n_dt2_idx_0_tmp_0 = 2.0F * rtDW.DiscreteTimeIntegratory_DSTAT_j[0] *
    rtb_n_g_des[0];
  n_dt2_idx_0_tmp_1 = rtDW.DiscreteTimeIntegratory_dt_DSTA[0] *
    rtDW.DiscreteTimeIntegratory_dt_DSTA[0] * 2.0F;
  n_dt2_idx_0_tmp_2 = 2.0F * rtDW.DiscreteTimeIntegratory_DSTAT_j[2] *
    rtb_n_g_des_a;
  n_dt2_idx_0_tmp_3 = rtDW.DiscreteTimeIntegratory_dt_DSTA[2] *
    rtDW.DiscreteTimeIntegratory_dt_DSTA[2] * 2.0F;
  q2_q2 = (q1_q1 + absxk) + scale;
  t = (q2_q2_tmp_tmp + b) + q2_q2_tmp_tmp_0;
  n_dt2_idx_1_tmp = std::pow(q2_q2, 1.5F);
  y = (t * t * 3.0F / (4.0F * std::pow(q2_q2, 2.5F)) - (((((q2_q2_tmp +
            n_dt2_idx_0_tmp) + n_dt2_idx_0_tmp_0) + n_dt2_idx_0_tmp_1) +
         n_dt2_idx_0_tmp_2) + n_dt2_idx_0_tmp_3) / (2.0F * n_dt2_idx_1_tmp)) *
    rtDW.DiscreteTimeIntegratory_DSTAT_j[1] + (rtb_n_g_des[1] / std::sqrt(q2_q2)
    - t * rtDW.DiscreteTimeIntegratory_dt_DSTA[1] / n_dt2_idx_1_tmp);
  q2_q2 = (scale + absxk) + q1_q1;
  q1_q1 = (q2_q2_tmp_tmp_0 + b) + q2_q2_tmp_tmp;
  rtb_n_dt2_h[0] = (q2_q2_tmp_0 * q2_q2_tmp_0 * 3.0F / (4.0F * std::pow(q0_q3,
    2.5F)) - (((((n_dt2_idx_0_tmp_0 + n_dt2_idx_0_tmp_1) + q2_q2_tmp) +
                n_dt2_idx_0_tmp) + n_dt2_idx_0_tmp_2) + n_dt2_idx_0_tmp_3) /
                    (2.0F * std::pow(q0_q3, 1.5F))) *
    rtDW.DiscreteTimeIntegratory_DSTAT_j[0] + (rtb_n_g_des[0] / std::sqrt(q0_q3)
    - q2_q2_tmp_0 * rtDW.DiscreteTimeIntegratory_dt_DSTA[0] / std::pow(q0_q3,
    1.5F));
  rtb_n_dt2_h[1] = (real32_T)y;
  rtb_omega2_e_idx_0 = std::pow(q2_q2, 1.5F);
  rtb_n_dt2_h[2] = (q1_q1 * q1_q1 * 3.0F / (4.0F * std::pow(q2_q2, 2.5F)) -
                    (((((n_dt2_idx_0_tmp_2 + n_dt2_idx_0_tmp_3) +
                        n_dt2_idx_0_tmp_0) + n_dt2_idx_0_tmp_1) + q2_q2_tmp) +
                     n_dt2_idx_0_tmp) / (2.0F * rtb_omega2_e_idx_0)) *
    rtDW.DiscreteTimeIntegratory_DSTAT_j[2] + (rtb_n_g_des_a / std::sqrt(q2_q2)
    - q1_q1 * rtDW.DiscreteTimeIntegratory_dt_DSTA[2] / rtb_omega2_e_idx_0);

  // DiscreteIntegrator: '<S50>/Discrete-Time Integrator y'
  if (rtDW.DiscreteTimeIntegratory_IC_L_mh != 0) {
    rtDW.DiscreteTimeIntegratory_DSTAT_i[0] = cmd_V_NED[0];
    rtDW.DiscreteTimeIntegratory_DSTAT_i[1] = cmd_V_NED[1];
    rtDW.DiscreteTimeIntegratory_DSTAT_i[2] = cmd_V_NED[2];
  }

  // MATLAB Function: '<S50>/n ref norm'
  scale = 1.29246971E-26F;

  // Gain: '<S50>/omega^2' incorporates:
  //   DiscreteIntegrator: '<S50>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S50>/Discrete-Time Integrator y_dt'
  //   Gain: '<S50>/2*d//omega'
  //   Sum: '<S50>/Sum2'
  //   Sum: '<S50>/Sum3'

  rtb_omega2_e_idx_0 = (cmd_V_NED[0] - (0.0527361445F *
    rtDW.DiscreteTimeIntegratory_dt_DS_b[0] +
    rtDW.DiscreteTimeIntegratory_DSTAT_i[0])) * 1438.2793F;

  // MATLAB Function: '<S50>/n ref norm' incorporates:
  //   DiscreteIntegrator: '<S50>/Discrete-Time Integrator y'

  absxk = std::abs(rtDW.DiscreteTimeIntegratory_DSTAT_i[0]);
  if (absxk > 1.29246971E-26F) {
    q2_q2 = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    q2_q2 = t * t;
  }

  // Gain: '<S50>/omega^2' incorporates:
  //   DiscreteIntegrator: '<S50>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S50>/Discrete-Time Integrator y_dt'
  //   Gain: '<S50>/2*d//omega'
  //   Sum: '<S50>/Sum2'
  //   Sum: '<S50>/Sum3'

  rtb_omega2_e_idx_1 = (cmd_V_NED[1] - (0.0527361445F *
    rtDW.DiscreteTimeIntegratory_dt_DS_b[1] +
    rtDW.DiscreteTimeIntegratory_DSTAT_i[1])) * 1438.2793F;

  // MATLAB Function: '<S50>/n ref norm' incorporates:
  //   DiscreteIntegrator: '<S50>/Discrete-Time Integrator y'

  absxk = std::abs(rtDW.DiscreteTimeIntegratory_DSTAT_i[1]);
  if (absxk > scale) {
    t = scale / absxk;
    q2_q2 = q2_q2 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q2_q2 += t * t;
  }

  // Gain: '<S50>/omega^2' incorporates:
  //   DiscreteIntegrator: '<S50>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S50>/Discrete-Time Integrator y_dt'
  //   Gain: '<S50>/2*d//omega'
  //   Sum: '<S50>/Sum2'
  //   Sum: '<S50>/Sum3'

  rtb_omega2_e_idx_2 = (cmd_V_NED[2] - (0.0527361445F *
    rtDW.DiscreteTimeIntegratory_dt_DS_b[2] +
    rtDW.DiscreteTimeIntegratory_DSTAT_i[2])) * 1438.2793F;

  // MATLAB Function: '<S50>/n ref norm' incorporates:
  //   DiscreteIntegrator: '<S50>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S50>/Discrete-Time Integrator y_dt'

  absxk = std::abs(rtDW.DiscreteTimeIntegratory_DSTAT_i[2]);
  if (absxk > scale) {
    t = scale / absxk;
    q2_q2 = q2_q2 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q2_q2 += t * t;
  }

  q2_q2 = scale * std::sqrt(q2_q2);
  b = q2_q2;
  idx = 0;
  if (q2_q2 < 2.22044605E-16F) {
    idx = 1;
  }

  if (0 <= idx - 1) {
    b = 2.22044605E-16F;
  }

  q1_q1 = rtDW.DiscreteTimeIntegratory_DSTAT_i[1] *
    rtDW.DiscreteTimeIntegratory_DSTAT_i[1];
  scale = rtDW.DiscreteTimeIntegratory_DSTAT_i[2] *
    rtDW.DiscreteTimeIntegratory_DSTAT_i[2];
  absxk = rtDW.DiscreteTimeIntegratory_DSTAT_i[0] *
    rtDW.DiscreteTimeIntegratory_DSTAT_i[0];
  t = absxk + q1_q1;
  q0_q3 = t + scale;
  q2_q2_tmp_0 = divideFinite(std::pow(q0_q3, 1.5F));
  q2_q2_0[0] = (q1_q1 + scale) * q2_q2_tmp_0;
  q2_q2 = -rtDW.DiscreteTimeIntegratory_DSTAT_i[0] *
    rtDW.DiscreteTimeIntegratory_DSTAT_i[1] * q2_q2_tmp_0;
  q2_q2_0[3] = q2_q2;
  q2_q2_tmp_tmp = -rtDW.DiscreteTimeIntegratory_DSTAT_i[0] *
    rtDW.DiscreteTimeIntegratory_DSTAT_i[2];
  q2_q2_tmp = q2_q2_tmp_tmp * q2_q2_tmp_0;
  q2_q2_0[6] = q2_q2_tmp;
  q2_q2_0[1] = q2_q2;
  q2_q2_0[4] = (absxk + scale) * q2_q2_tmp_0;
  q2_q2 = -rtDW.DiscreteTimeIntegratory_DSTAT_i[1] *
    rtDW.DiscreteTimeIntegratory_DSTAT_i[2] * q2_q2_tmp_0;
  q2_q2_0[7] = q2_q2;
  q2_q2_0[2] = q2_q2_tmp;
  q2_q2_0[5] = q2_q2;
  q2_q2_0[8] = t * q2_q2_tmp_0;
  for (i = 0; i < 3; i++) {
    rtb_n[i] = rtDW.DiscreteTimeIntegratory_DSTAT_i[i] / b;
    rtb_n_dt[i] = q2_q2_0[i + 6] * rtDW.DiscreteTimeIntegratory_dt_DS_b[2] +
      (q2_q2_0[i + 3] * rtDW.DiscreteTimeIntegratory_dt_DS_b[1] + q2_q2_0[i] *
       rtDW.DiscreteTimeIntegratory_dt_DS_b[0]);
  }

  rtb_n_dt[2] = ((q2_q2_tmp_tmp * rtDW.DiscreteTimeIntegratory_dt_DS_b[0] +
                  absxk * rtDW.DiscreteTimeIntegratory_dt_DS_b[2]) +
                 (rtDW.DiscreteTimeIntegratory_DSTAT_i[1] *
                  rtDW.DiscreteTimeIntegratory_dt_DS_b[2] -
                  rtDW.DiscreteTimeIntegratory_DSTAT_i[2] *
                  rtDW.DiscreteTimeIntegratory_dt_DS_b[1]) *
                 rtDW.DiscreteTimeIntegratory_DSTAT_i[1]) * q2_q2_tmp_0;
  q2_q2_tmp_tmp = 2.0F * rtDW.DiscreteTimeIntegratory_DSTAT_i[1] *
    rtDW.DiscreteTimeIntegratory_dt_DS_b[1];
  b = 2.0F * rtDW.DiscreteTimeIntegratory_DSTAT_i[0] *
    rtDW.DiscreteTimeIntegratory_dt_DS_b[0];
  q2_q2_tmp_tmp_0 = 2.0F * rtDW.DiscreteTimeIntegratory_DSTAT_i[2] *
    rtDW.DiscreteTimeIntegratory_dt_DS_b[2];
  q2_q2_tmp_0 = (b + q2_q2_tmp_tmp) + q2_q2_tmp_tmp_0;
  q2_q2_tmp = 2.0F * rtDW.DiscreteTimeIntegratory_DSTAT_i[1] *
    rtb_omega2_e_idx_1;
  n_dt2_idx_0_tmp = rtDW.DiscreteTimeIntegratory_dt_DS_b[1] *
    rtDW.DiscreteTimeIntegratory_dt_DS_b[1] * 2.0F;
  n_dt2_idx_0_tmp_0 = 2.0F * rtDW.DiscreteTimeIntegratory_DSTAT_i[0] *
    rtb_omega2_e_idx_0;
  n_dt2_idx_0_tmp_1 = rtDW.DiscreteTimeIntegratory_dt_DS_b[0] *
    rtDW.DiscreteTimeIntegratory_dt_DS_b[0] * 2.0F;
  n_dt2_idx_0_tmp_2 = 2.0F * rtDW.DiscreteTimeIntegratory_DSTAT_i[2] *
    rtb_omega2_e_idx_2;
  n_dt2_idx_0_tmp_3 = rtDW.DiscreteTimeIntegratory_dt_DS_b[2] *
    rtDW.DiscreteTimeIntegratory_dt_DS_b[2] * 2.0F;
  q2_q2 = (q1_q1 + absxk) + scale;
  t = (q2_q2_tmp_tmp + b) + q2_q2_tmp_tmp_0;
  n_dt2_idx_1_tmp = std::pow(q2_q2, 1.5F);
  y = (t * t * 3.0F / (4.0F * std::pow(q2_q2, 2.5F)) - (((((q2_q2_tmp +
            n_dt2_idx_0_tmp) + n_dt2_idx_0_tmp_0) + n_dt2_idx_0_tmp_1) +
         n_dt2_idx_0_tmp_2) + n_dt2_idx_0_tmp_3) / (2.0F * n_dt2_idx_1_tmp)) *
    rtDW.DiscreteTimeIntegratory_DSTAT_i[1] + (rtb_omega2_e_idx_1 / std::sqrt
    (q2_q2) - t * rtDW.DiscreteTimeIntegratory_dt_DS_b[1] / n_dt2_idx_1_tmp);
  q2_q2 = (scale + absxk) + q1_q1;
  q1_q1 = (q2_q2_tmp_tmp_0 + b) + q2_q2_tmp_tmp;
  rtb_n_dt2[0] = (q2_q2_tmp_0 * q2_q2_tmp_0 * 3.0F / (4.0F * std::pow(q0_q3,
    2.5F)) - (((((n_dt2_idx_0_tmp_0 + n_dt2_idx_0_tmp_1) + q2_q2_tmp) +
                n_dt2_idx_0_tmp) + n_dt2_idx_0_tmp_2) + n_dt2_idx_0_tmp_3) /
                  (2.0F * std::pow(q0_q3, 1.5F))) *
    rtDW.DiscreteTimeIntegratory_DSTAT_i[0] + (rtb_omega2_e_idx_0 / std::sqrt
    (q0_q3) - q2_q2_tmp_0 * rtDW.DiscreteTimeIntegratory_dt_DS_b[0] / std::pow
    (q0_q3, 1.5F));
  rtb_n_dt2[1] = (real32_T)y;
  scale = std::pow(q2_q2, 1.5F);
  rtb_n_dt2[2] = (q1_q1 * q1_q1 * 3.0F / (4.0F * std::pow(q2_q2, 2.5F)) -
                  (((((n_dt2_idx_0_tmp_2 + n_dt2_idx_0_tmp_3) +
                      n_dt2_idx_0_tmp_0) + n_dt2_idx_0_tmp_1) + q2_q2_tmp) +
                   n_dt2_idx_0_tmp) / (2.0F * scale)) *
    rtDW.DiscreteTimeIntegratory_DSTAT_i[2] + (rtb_omega2_e_idx_2 / std::sqrt
    (q2_q2) - q1_q1 * rtDW.DiscreteTimeIntegratory_dt_DS_b[2] / scale);

  // DiscreteIntegrator: '<S68>/Discrete-Time Integrator y' incorporates:
  //   Inport: '<Root>/measure'

  if (rtDW.DiscreteTimeIntegratory_IC_LO_d != 0) {
    rtDW.DiscreteTimeIntegratory_DSTAT_g[0] = rtU.measure.Omega_Kb_raw[0];
    rtDW.DiscreteTimeIntegratory_DSTAT_g[1] = rtU.measure.Omega_Kb_raw[1];
    rtDW.DiscreteTimeIntegratory_DSTAT_g[2] = rtU.measure.Omega_Kb_raw[2];
  }

  rtb_y_b[0] = rtDW.DiscreteTimeIntegratory_DSTAT_g[0];

  // DiscreteIntegrator: '<S68>/Discrete-Time Integrator y_dt'
  rtb_y_dt_mq[0] = rtDW.DiscreteTimeIntegratory_dt_DS_m[0];

  // DiscreteIntegrator: '<S68>/Discrete-Time Integrator y'
  rtb_y_b[1] = rtDW.DiscreteTimeIntegratory_DSTAT_g[1];

  // DiscreteIntegrator: '<S68>/Discrete-Time Integrator y_dt'
  rtb_y_dt_mq[1] = rtDW.DiscreteTimeIntegratory_dt_DS_m[1];

  // DiscreteIntegrator: '<S68>/Discrete-Time Integrator y'
  rtb_y_b[2] = rtDW.DiscreteTimeIntegratory_DSTAT_g[2];

  // DiscreteIntegrator: '<S68>/Discrete-Time Integrator y_dt'
  rtb_y_dt_mq[2] = rtDW.DiscreteTimeIntegratory_dt_DS_m[2];

  // MATLAB Function: '<S38>/Lean Vector Derivative Trafo Delay'
  LeanVectorDerivativeTrafo(rtb_n, rtb_n_dt, rtb_n_dt2, rtb_y_dl, rtb_y_b,
    rtb_y_dt_mq, rtb_n_b, rtb_n_b_dt, rtb_Sum2_c);

  // MATLAB Function: '<S38>/Lean Vector Derivative Trafo'
  LeanVectorDerivativeTrafo(cmd_V_NED, rtb_n_dt_d, rtb_n_dt2_h, rtb_y_dl,
    rtb_y_b, rtb_y_dt_mq, rtb_n, rtb_n_dt, rtb_n_dt2);

  // MATLAB Function: '<S38>/MATLAB Function'
  if (rtb_n_b[2] > 0.0F) {
    if (rtb_n_b[2] > 0.99) {
      scale = 0.0F;
      q1_q1 = 1.0F;
    } else {
      scale = 1.29246971E-26F;
      absxk = std::abs(rtb_n_b[0]);
      if (absxk > 1.29246971E-26F) {
        q2_q2 = 1.0F;
        scale = absxk;
      } else {
        t = absxk / 1.29246971E-26F;
        q2_q2 = t * t;
      }

      absxk = std::abs(rtb_n_b[1]);
      if (absxk > scale) {
        t = scale / absxk;
        q2_q2 = q2_q2 * t * t + 1.0F;
        scale = absxk;
      } else {
        t = absxk / scale;
        q2_q2 += t * t;
      }

      q2_q2 = scale * std::sqrt(q2_q2);
      idx = 0;
      if (q2_q2 < 2.22044605E-16F) {
        idx = 1;
      }

      if (0 <= idx - 1) {
        q2_q2 = 2.22044605E-16F;
      }

      scale = rtb_n_b[0] / q2_q2;
      q1_q1 = rtb_n_b[1] / q2_q2;
    }

    rtb_n_b[0] = 2.0F * scale - rtb_n_b[0];
    rtb_n_b_dt[0] = -rtb_n_b_dt[0];
    rtb_Sum2_c[0] = -rtb_Sum2_c[0];
    rtb_n_b[1] = 2.0F * q1_q1 - rtb_n_b[1];
    rtb_n_b_dt[1] = -rtb_n_b_dt[1];
    rtb_Sum2_c[1] = -rtb_Sum2_c[1];
  }

  // MinMax: '<S23>/Max' incorporates:
  //   DiscreteIntegrator: '<S27>/Discrete-Time Integrator'

  if (rtDW.DiscreteTimeIntegrator_DSTATE_n[0] > 0.1F) {
    rtb_Max[0] = rtDW.DiscreteTimeIntegrator_DSTATE_n[0];
  } else {
    rtb_Max[0] = 0.1F;
  }

  if (rtDW.DiscreteTimeIntegrator_DSTATE_n[1] > 0.1F) {
    rtb_Max[1] = rtDW.DiscreteTimeIntegrator_DSTATE_n[1];
  } else {
    rtb_Max[1] = 0.1F;
  }

  if (rtDW.DiscreteTimeIntegrator_DSTATE_n[2] > 0.1F) {
    rtb_Max[2] = rtDW.DiscreteTimeIntegrator_DSTATE_n[2];
  } else {
    rtb_Max[2] = 0.1F;
  }

  if (rtDW.DiscreteTimeIntegrator_DSTATE_n[3] > 0.1F) {
    rtb_Max[3] = rtDW.DiscreteTimeIntegrator_DSTATE_n[3];
  } else {
    rtb_Max[3] = 0.1F;
  }

  // End of MinMax: '<S23>/Max'

  // MATLAB Function: '<S23>/MATLAB Function'
  q2_q2 = std::sqrt(1.05322839E-9F * rtb_Max[0] + 9.95611482E-10F);
  omega[1] = std::sqrt(1.05322839E-9F * rtb_Max[1] + 9.95611482E-10F);
  omega[2] = std::sqrt(1.05322839E-9F * rtb_Max[2] + 9.95611482E-10F);
  omega[3] = std::sqrt(1.05322839E-9F * rtb_Max[3] + 9.95611482E-10F);
  omega[0] = (q2_q2 - 3.15533107E-5F) / 4.22296154E-9F;
  omega[1] = (omega[1] - 3.15533107E-5F) / 4.22296154E-9F;
  omega[2] = (omega[2] - 3.15533107E-5F) / 4.22296154E-9F;
  omega[3] = (omega[3] - 3.15533107E-5F) / 4.22296154E-9F;
  rtb_Max[1] = std::sqrt(1.05322839E-9F * rtb_Max[1] + 9.95611482E-10F);
  rtb_Max[2] = std::sqrt(1.05322839E-9F * rtb_Max[2] + 9.95611482E-10F);
  rtb_Max[0] = 0.124702588F / q2_q2;
  rtb_Max[1] = 0.124702588F / rtb_Max[1];
  rtb_Max[2] = 0.124702588F / rtb_Max[2];
  q2_q2 = 0.124702588F / std::sqrt(1.05322839E-9F * rtb_Max[3] + 9.95611482E-10F);
  rtb_Max[3] = q2_q2;
  memset(&G_omega[0], 0, sizeof(real32_T) << 4U);
  memset(&b_d[0], 0, sizeof(real32_T) << 4U);
  G_omega[0] = rtb_Max[0];
  G_omega[5] = rtb_Max[1];
  G_omega[10] = rtb_Max[2];
  G_omega[15] = q2_q2;
  for (idx = 0; idx < 4; idx++) {
    b_d[idx + (idx << 2)] = omega[idx];
    for (i = 0; i < 4; i++) {
      k = i << 2;
      rtb_G2_tmp = idx + k;
      rtb_G2[rtb_G2_tmp] = 0.0F;
      rtb_G2_tmp_0 = k + idx;
      rtb_G2[rtb_G2_tmp] = rtb_G2[rtb_G2_tmp_0] + G_omega[k] *
        rtConstP.MATLABFunction_G20[idx];
      rtb_G2[rtb_G2_tmp] = G_omega[k + 1] * rtConstP.MATLABFunction_G20[idx + 4]
        + rtb_G2[rtb_G2_tmp_0];
      rtb_G2[rtb_G2_tmp] = G_omega[k + 2] * rtConstP.MATLABFunction_G20[idx + 8]
        + rtb_G2[rtb_G2_tmp_0];
      rtb_G2[rtb_G2_tmp] = G_omega[k + 3] * rtConstP.MATLABFunction_G20[idx + 12]
        + rtb_G2[rtb_G2_tmp_0];
    }
  }

  // MATLAB Function: '<S2>/MATLAB Function' incorporates:
  //   Gain: '<Root>/Gain5'

  scale = 0.5F * -rtb_Switch[3] + 0.5F;

  // DiscreteIntegrator: '<S52>/Discrete-Time Integrator'
  q1_q1 = rtDW.DiscreteTimeIntegrator_DSTATE_b;

  // Gain: '<S52>/1//T' incorporates:
  //   DiscreteIntegrator: '<S52>/Discrete-Time Integrator'
  //   Gain: '<S38>/r_max'
  //   Sum: '<S52>/Sum2'

  q2_q2 = (6.28318548F * rtb_Switch[2] - rtDW.DiscreteTimeIntegrator_DSTATE_b) *
    4.99576426F;

  // MATLAB Function: '<S19>/DCM to quaternions'
  DCMtoquaternions(rtb_y_dl, rtb_Switch);

  // MATLAB Function: '<S19>/Quaternion Reduced'
  QuaternionReduced(rtb_Switch, rtb_Max, &absxk);

  // DiscreteIntegrator: '<S38>/Discrete-Time Integrator2'
  if (rtDW.DiscreteTimeIntegrator2_IC_LOAD != 0) {
    rtDW.DiscreteTimeIntegrator2_DSTATE = absxk;
  }

  // MATLAB Function: '<S33>/wrap angle' incorporates:
  //   DiscreteIntegrator: '<S38>/Discrete-Time Integrator2'

  wrapangle(rtDW.DiscreteTimeIntegrator2_DSTATE, &absxk);

  // MATLAB Function: '<S34>/DCM to quaternions'
  DCMtoquaternions(rtb_y_dl, rtb_Switch);

  // MATLAB Function: '<S34>/Quaternion Reduced'
  QuaternionReduced(rtb_Switch, rtb_Max, &t);

  // MATLAB Function: '<S33>/wrap angle1'
  wrapangle(t, &q0_q3);

  // MATLAB Function: '<S33>/angle error'
  absxk -= q0_q3;
  if (absxk > 3.1415926535897931) {
    absxk -= 6.28318548F;
  } else {
    if (absxk < -3.1415926535897931) {
      absxk += 6.28318548F;
    }
  }

  // End of MATLAB Function: '<S33>/angle error'

  // Gain: '<S12>/Gain' incorporates:
  //   MATLAB Function: '<S11>/INDI Copter Acc 2 Lean Vector'

  q0_q0 = -q0_q0;

  // DiscreteIntegrator: '<S31>/Discrete-Time Integrator y'
  if (rtDW.DiscreteTimeIntegratory_IC_LO_j != 0) {
    rtDW.DiscreteTimeIntegratory_DSTA_gx = q0_q0;
  }

  for (k = 0; k < 4; k++) {
    // MATLAB Function: '<S24>/MATLAB Function2'
    omega[k] = 0.0F;

    // MATLAB Function: '<S23>/MATLAB Function'
    for (i = 0; i < 4; i++) {
      idx = i << 2;
      rtb_G2_tmp = k + idx;
      tmp[rtb_G2_tmp] = 0.0F;
      rtb_G2_tmp_0 = idx + k;
      tmp[rtb_G2_tmp] = tmp[rtb_G2_tmp_0] + b_d[idx] *
        rtConstP.MATLABFunction_G10[k];
      tmp[rtb_G2_tmp] = b_d[idx + 1] * rtConstP.MATLABFunction_G10[k + 4] +
        tmp[rtb_G2_tmp_0];
      tmp[rtb_G2_tmp] = b_d[idx + 2] * rtConstP.MATLABFunction_G10[k + 8] +
        tmp[rtb_G2_tmp_0];
      tmp[rtb_G2_tmp] = b_d[idx + 3] * rtConstP.MATLABFunction_G10[k + 12] +
        tmp[rtb_G2_tmp_0];
    }

    // Sum: '<S20>/Add1' incorporates:
    //   MATLAB Function: '<S23>/MATLAB Function'

    for (i = 0; i < 4; i++) {
      // MATLAB Function: '<S23>/MATLAB Function'
      idx = i << 2;
      tmp_1[k + idx] = (((G_omega[idx + 1] * tmp[k + 4] + G_omega[idx] * tmp[k])
                         + G_omega[idx + 2] * tmp[k + 8]) + G_omega[idx + 3] *
                        tmp[k + 12]) + rtb_G2[idx + k];
    }

    // Gain: '<S20>/Gain' incorporates:
    //   DiscreteIntegrator: '<S27>/Discrete-Time Integrator'
    //   MATLAB Function: '<S24>/MATLAB Function2'

    rtb_Switch[k] = 0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_n[k];

    // Lookup_n-D: '<S20>/1-D Lookup Table2' incorporates:
    //   DiscreteIntegrator: '<S27>/Discrete-Time Integrator'
    //   MATLAB Function: '<S24>/MATLAB Function2'

    rtb_uDLookupTable2[k] = ((0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_n[k]) +
      (1.0F - rtDW.DiscreteTimeIntegrator_DSTATE_n[k])) * 0.5F;

    // MATLAB Function: '<S24>/MATLAB Function2' incorporates:
    //   DiscreteIntegrator: '<S27>/Discrete-Time Integrator'

    rtb_Max[k] = 1.0F - rtDW.DiscreteTimeIntegrator_DSTATE_n[k];
  }

  // Sum: '<S20>/Add1'
  memcpy(&tmp[0], &tmp_1[0], sizeof(real32_T) << 4U);

  // Sum: '<S14>/Add1' incorporates:
  //   DiscreteIntegrator: '<S52>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<S68>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S68>/Discrete-Time Integrator y_dt'
  //   Gain: '<S35>/Gain'
  //   Gain: '<S35>/Gain1'
  //   Gain: '<S35>/Gain2'
  //   Gain: '<S35>/Gain3'
  //   Gain: '<S35>/Gain4'
  //   Gain: '<S35>/Gain5'
  //   Gain: '<S35>/Gain6'
  //   Gain: '<S36>/Gain'
  //   MATLAB Function: '<S38>/MATLAB Function'
  //   Sum: '<S33>/error1 4'
  //   Sum: '<S33>/error1 5'
  //   Sum: '<S33>/error1 6'
  //   Sum: '<S33>/error1 8'
  //   Sum: '<S33>/error1 9'
  //   Sum: '<S35>/Add'
  //   Sum: '<S35>/Add1'

  rtb_n[2] = q2_q2;
  rtb_n_dt2_h[2] = ((rtDW.DiscreteTimeIntegrator_DSTATE_b -
                     rtDW.DiscreteTimeIntegratory_DSTAT_g[2]) * 3.94851255F +
                    6.57527876F * absxk) + (q2_q2 -
    rtDW.DiscreteTimeIntegratory_dt_DS_m[2]) * -0.209627926F;
  rtb_n[0] = rtb_n_dt2[1];
  rtb_n_dt2_h[0] = (163.663986F * rtb_n_b[1] + 33.6609344F * rtb_n_b_dt[1]) +
    1.30769229F * rtb_Sum2_c[1];

  // Sum: '<S14>/Add2' incorporates:
  //   Constant: '<S37>/Constant1'

  rtb_y_b[0] = 0.0F;

  // Sum: '<S14>/Add1' incorporates:
  //   Gain: '<S35>/Gain'
  //   Gain: '<S35>/Gain1'
  //   Gain: '<S35>/Gain3'
  //   Gain: '<S35>/Gain5'
  //   Gain: '<S36>/Gain'
  //   MATLAB Function: '<S38>/MATLAB Function'
  //   Sum: '<S33>/error1 4'
  //   Sum: '<S33>/error1 5'
  //   Sum: '<S33>/error1 6'
  //   Sum: '<S35>/Add'

  rtb_n[1] = -rtb_n_dt2[0];
  rtb_n_dt2_h[1] = -((163.663986F * rtb_n_b[0] + 33.6609344F * rtb_n_b_dt[0]) +
                     1.30769229F * rtb_Sum2_c[0]);

  // Sum: '<S14>/Add2' incorporates:
  //   Constant: '<S37>/Constant1'
  //   DiscreteIntegrator: '<S68>/Discrete-Time Integrator y_dt'

  rtb_y_b[1] = 0.0F;
  rtb_y_b[2] = rtDW.DiscreteTimeIntegratory_dt_DS_m[2];
  for (i = 0; i < 3; i++) {
    // MATLAB Function: '<S12>/desired and measured specific thrust' incorporates:
    //   DiscreteIntegrator: '<S69>/Discrete-Time Integrator y'

    absxk = rtDW.DiscreteTimeIntegratory_DSTAT_k[i + 6];

    // Sum: '<S20>/Add2' incorporates:
    //   Sum: '<S14>/Add1'
    //   Sum: '<S14>/Add2'

    tmp_2[i] = (rtb_n[i] + rtb_n_dt2_h[i]) - rtb_y_b[i];

    // MATLAB Function: '<S12>/desired and measured specific thrust' incorporates:
    //   DiscreteIntegrator: '<S69>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y'

    rtb_n_dt2[i] = absxk * rtDW.DiscreteTimeIntegratory_DSTAT_b[2] +
      (rtDW.DiscreteTimeIntegratory_DSTAT_k[i + 3] *
       rtDW.DiscreteTimeIntegratory_DSTAT_b[1] +
       rtDW.DiscreteTimeIntegratory_DSTAT_k[i] *
       rtDW.DiscreteTimeIntegratory_DSTAT_b[0]);
    rtb_n_b_dt[i] = absxk * 9.81F;
  }

  // Sum: '<S20>/Add2' incorporates:
  //   DiscreteIntegrator: '<S31>/Discrete-Time Integrator y'
  //   MATLAB Function: '<S12>/desired and measured specific thrust'
  //   Product: '<S20>/MatrixMultiply2'
  //   Sum: '<S12>/Add'
  //   UnitDelay: '<S20>/Unit Delay1'

  tmp_2[3] = rtDW.DiscreteTimeIntegratory_DSTA_gx - (rtb_n_dt2[2] - rtb_n_b_dt[2]);
  for (i = 0; i < 4; i++) {
    tmp_0[i] = (((rtb_G2[i + 4] * rtDW.UnitDelay1_DSTATE[1] + rtb_G2[i] *
                  rtDW.UnitDelay1_DSTATE[0]) + rtb_G2[i + 8] *
                 rtDW.UnitDelay1_DSTATE[2]) + rtb_G2[i + 12] *
                rtDW.UnitDelay1_DSTATE[3]) + tmp_2[i];
  }

  // MATLAB Function: '<S24>/MATLAB Function2' incorporates:
  //   DiscreteIntegrator: '<S27>/Discrete-Time Integrator'
  //   Lookup_n-D: '<S20>/1-D Lookup Table1'

  memcpy(&tmp_1[0], &rtConstP.MATLABFunction2_ca.W_v[0], sizeof(real32_T) << 4U);
  tmp_2[0] = 0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_n[0];
  tmp_2[1] = 0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_n[1];
  tmp_2[2] = 0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_n[2];
  tmp_2[3] = 0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_n[3];
  wls_alloc(tmp, tmp_0, rtb_Switch, rtb_Max, tmp_1,
            &rtConstP.MATLABFunction2_ca.W_u[0], tmp_2, 1000.0F +
            look1_iflf_binlx(scale, rtConstP.uDLookupTable1_bp01Data,
             rtConstP.uDLookupTable1_tableData, 3U), rtb_uDLookupTable2, omega,
            100.0F);

  // Sum: '<S10>/Add6' incorporates:
  //   DiscreteIntegrator: '<S22>/Discrete-Time Integrator y'

  t = rtb_uDLookupTable2[0] + rtDW.DiscreteTimeIntegratory_DSTA_j4[0];

  // Saturate: '<S10>/Saturation3'
  if (t > 1.0F) {
    t = 1.0F;
  } else {
    if (t < 0.1F) {
      t = 0.1F;
    }
  }

  // Outport: '<Root>/logs'
  rtY.logs[0] = t;

  // Saturate: '<S10>/Saturation3'
  rtb_Switch[0] = t;

  // Sum: '<S10>/Add6' incorporates:
  //   DiscreteIntegrator: '<S22>/Discrete-Time Integrator y'

  t = rtb_uDLookupTable2[1] + rtDW.DiscreteTimeIntegratory_DSTA_j4[1];

  // Saturate: '<S10>/Saturation3'
  if (t > 1.0F) {
    t = 1.0F;
  } else {
    if (t < 0.1F) {
      t = 0.1F;
    }
  }

  // Outport: '<Root>/logs'
  rtY.logs[1] = t;

  // Saturate: '<S10>/Saturation3'
  rtb_Switch[1] = t;

  // Sum: '<S10>/Add6' incorporates:
  //   DiscreteIntegrator: '<S22>/Discrete-Time Integrator y'

  t = rtb_uDLookupTable2[2] + rtDW.DiscreteTimeIntegratory_DSTA_j4[2];

  // Saturate: '<S10>/Saturation3'
  if (t > 1.0F) {
    t = 1.0F;
  } else {
    if (t < 0.1F) {
      t = 0.1F;
    }
  }

  // Outport: '<Root>/logs'
  rtY.logs[2] = t;

  // Saturate: '<S10>/Saturation3'
  rtb_Switch[2] = t;

  // Sum: '<S10>/Add6' incorporates:
  //   DiscreteIntegrator: '<S22>/Discrete-Time Integrator y'

  t = rtb_uDLookupTable2[3] + rtDW.DiscreteTimeIntegratory_DSTA_j4[3];

  // Saturate: '<S10>/Saturation3'
  if (t > 1.0F) {
    t = 1.0F;
  } else {
    if (t < 0.1F) {
      t = 0.1F;
    }
  }

  // Outport: '<Root>/logs' incorporates:
  //   DiscreteIntegrator: '<S55>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<S69>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S69>/Discrete-Time Integrator y_dt'
  //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S71>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S72>/Discrete-Time Integrator y'
  //   Gain: '<S69>/2*d//omega'
  //   MATLAB Function: '<S9>/Auxiliary function to define log_config in generated C++ code'
  //   SignalConversion: '<S76>/TmpSignal ConversionAt SFunction Inport2'
  //   Sum: '<S69>/Sum2'
  //   Sum: '<S69>/Sum3'

  rtY.logs[3] = t;
  rtY.logs[13] = rtb_n_b[0];
  rtY.logs[14] = rtb_n_b[1];
  rtY.logs[15] = rtDW.DiscreteTimeIntegrator_DSTATE[0];
  rtY.logs[18] = rtDW.DiscreteTimeIntegratory_DSTATE[0];
  rtY.logs[21] = rtDW.DiscreteTimeIntegratory_DSTAT_d[0];
  rtY.logs[24] = rtDW.DiscreteTimeIntegratory_DSTAT_b[0];
  rtY.logs[16] = rtDW.DiscreteTimeIntegrator_DSTATE[1];
  rtY.logs[19] = rtDW.DiscreteTimeIntegratory_DSTATE[1];
  rtY.logs[22] = rtDW.DiscreteTimeIntegratory_DSTAT_d[1];
  rtY.logs[25] = rtDW.DiscreteTimeIntegratory_DSTAT_b[1];
  rtY.logs[17] = rtDW.DiscreteTimeIntegrator_DSTATE[2];
  rtY.logs[20] = rtDW.DiscreteTimeIntegratory_DSTATE[2];
  rtY.logs[23] = rtDW.DiscreteTimeIntegratory_DSTAT_d[2];
  rtY.logs[26] = rtDW.DiscreteTimeIntegratory_DSTAT_b[2];
  for (i = 0; i < 9; i++) {
    rtY.logs[i + 4] = rtb_Add_h[i];
    rtb_M_bg[i] -= 0.0263680723F * rtDW.DiscreteTimeIntegratory_dt_DS_a[i] +
      rtDW.DiscreteTimeIntegratory_DSTAT_k[i];
  }

  // Sum: '<S62>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S55>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<S59>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<S62>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S62>/Discrete-Time Integrator y_dt'
  //   Gain: '<S62>/2*d//omega'
  //   Saturate: '<S59>/Saturation'
  //   Sum: '<S62>/Sum3'

  rtb_Add_h[0] = rtDW.DiscreteTimeIntegrator_DSTATE[0] - (0.189850137F *
    rtDW.DiscreteTimeIntegratory_dt_DS_h[0] +
    rtDW.DiscreteTimeIntegratory_DSTAT_l[0]);
  rtb_Add_h[3] = rtDW.DiscreteTimeIntegrator_DSTAT_my[0] - (0.189850137F *
    rtDW.DiscreteTimeIntegratory_dt_DS_h[3] +
    rtDW.DiscreteTimeIntegratory_DSTAT_l[3]);
  rtb_Add_h[6] = q1_q2 - (0.189850137F * rtDW.DiscreteTimeIntegratory_dt_DS_h[6]
    + rtDW.DiscreteTimeIntegratory_DSTAT_l[6]);
  rtb_Add_h[1] = rtDW.DiscreteTimeIntegrator_DSTATE[1] - (0.189850137F *
    rtDW.DiscreteTimeIntegratory_dt_DS_h[1] +
    rtDW.DiscreteTimeIntegratory_DSTAT_l[1]);
  rtb_Add_h[4] = rtDW.DiscreteTimeIntegrator_DSTAT_my[1] - (0.189850137F *
    rtDW.DiscreteTimeIntegratory_dt_DS_h[4] +
    rtDW.DiscreteTimeIntegratory_DSTAT_l[4]);
  rtb_Add_h[7] = q2_q3 - (0.189850137F * rtDW.DiscreteTimeIntegratory_dt_DS_h[7]
    + rtDW.DiscreteTimeIntegratory_DSTAT_l[7]);
  rtb_Add_h[2] = rtDW.DiscreteTimeIntegrator_DSTATE[2] - (0.189850137F *
    rtDW.DiscreteTimeIntegratory_dt_DS_h[2] +
    rtDW.DiscreteTimeIntegratory_DSTAT_l[2]);
  rtb_Add_h[5] = rtDW.DiscreteTimeIntegrator_DSTAT_my[2] - (0.189850137F *
    rtDW.DiscreteTimeIntegratory_dt_DS_h[5] +
    rtDW.DiscreteTimeIntegratory_DSTAT_l[5]);
  rtb_Add_h[8] = q1_q3 - (0.189850137F * rtDW.DiscreteTimeIntegratory_dt_DS_h[8]
    + rtDW.DiscreteTimeIntegratory_DSTAT_l[8]);

  // Lookup_n-D: '<S8>/1-D Lookup Table1' incorporates:
  //   Inport: '<Root>/cmd'

  scale = look1_iflf_binlx(rtU.cmd.RC_pwm[7], rtConstP.pooled6,
    rtConstP.pooled20, 1U);

  // Lookup_n-D: '<S8>/1-D Lookup Table' incorporates:
  //   Inport: '<Root>/cmd'

  absxk = look1_iflf_binlx(rtU.cmd.RC_pwm[6], rtConstP.pooled6,
    rtConstP.pooled20, 1U);

  // MinMax: '<S8>/Max'
  if (absxk >= scale) {
    absxk = scale;
  }

  // End of MinMax: '<S8>/Max'

  // Switch: '<S5>/Switch2' incorporates:
  //   RelationalOperator: '<S5>/LowerRelop1'
  //   Switch: '<S5>/Switch'

  if (rtb_Switch[1] <= absxk) {
    absxk = rtb_Switch[1];
  }

  // End of Switch: '<S5>/Switch2'

  // Switch: '<S6>/Switch2' incorporates:
  //   RelationalOperator: '<S6>/LowerRelop1'

  if (t <= scale) {
    scale = t;
  }

  // End of Switch: '<S6>/Switch2'

  // Outport: '<Root>/u' incorporates:
  //   Gain: '<Root>/Gain1'
  //   Gain: '<Root>/Gain2'
  //   Gain: '<Root>/Gain3'
  //   Gain: '<Root>/Gain4'

  rtY.u[0] = absxk;
  rtY.u[1] = scale;
  rtY.u[2] = rtb_Switch[0];
  rtY.u[3] = rtb_Switch[2];
  rtY.u[4] = 0.0F;
  rtY.u[5] = 0.0F;
  rtY.u[6] = 0.0F;
  rtY.u[7] = 0.0F;

  // Sum: '<S22>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S22>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S22>/Discrete-Time Integrator y_dt'
  //   DiscreteIntegrator: '<S27>/Discrete-Time Integrator'
  //   Product: '<S22>/Product2'
  //   Sum: '<S22>/Sum3'

  rtb_Max[0] = rtDW.DiscreteTimeIntegrator_DSTATE_n[0] -
    (rtDW.DiscreteTimeIntegratory_dt_DS_i[0] * 0.0263680723F +
     rtDW.DiscreteTimeIntegratory_DSTA_j4[0]);
  rtb_Max[1] = rtDW.DiscreteTimeIntegrator_DSTATE_n[1] -
    (rtDW.DiscreteTimeIntegratory_dt_DS_i[1] * 0.0263680723F +
     rtDW.DiscreteTimeIntegratory_DSTA_j4[1]);
  rtb_Max[2] = rtDW.DiscreteTimeIntegrator_DSTATE_n[2] -
    (rtDW.DiscreteTimeIntegratory_dt_DS_i[2] * 0.0263680723F +
     rtDW.DiscreteTimeIntegratory_DSTA_j4[2]);
  rtb_Max[3] = rtDW.DiscreteTimeIntegrator_DSTATE_n[3] -
    (rtDW.DiscreteTimeIntegratory_dt_DS_i[3] * 0.0263680723F +
     rtDW.DiscreteTimeIntegratory_DSTA_j4[3]);

  // Sum: '<S31>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S31>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S31>/Discrete-Time Integrator y_dt'
  //   Gain: '<S31>/2*d//omega'
  //   Sum: '<S31>/Sum3'

  q0_q0 -= 0.137113988F * rtDW.DiscreteTimeIntegratory_dt_D_mp +
    rtDW.DiscreteTimeIntegratory_DSTA_gx;

  // Update for DiscreteIntegrator: '<S72>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LOAD = 0U;

  // Update for DiscreteIntegrator: '<S55>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_IC_LOADI = 0U;
  rtDW.DiscreteTimeIntegrator_PrevRese = 0;

  // Update for DiscreteIntegrator: '<S71>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LO_f = 0U;

  // Update for DiscreteIntegrator: '<S70>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LO_i = 0U;

  // Update for DiscreteIntegrator: '<S7>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_DSTATE_m += 0.0025F;
  if (rtb_uDLookupTable > 0.0F) {
    rtDW.DiscreteTimeIntegrator_PrevRe_h = 1;
  } else if (rtb_uDLookupTable < 0.0F) {
    rtDW.DiscreteTimeIntegrator_PrevRe_h = -1;
  } else if (rtb_uDLookupTable == 0.0F) {
    rtDW.DiscreteTimeIntegrator_PrevRe_h = 0;
  } else {
    rtDW.DiscreteTimeIntegrator_PrevRe_h = 2;
  }

  // End of Update for DiscreteIntegrator: '<S7>/Discrete-Time Integrator'

  // Update for DiscreteIntegrator: '<S59>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_IC_LOA_j = 0U;

  // Gain: '<S68>/omega^2' incorporates:
  //   DiscreteIntegrator: '<S68>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S68>/Discrete-Time Integrator y_dt'
  //   Gain: '<S68>/2*d//omega'
  //   Inport: '<Root>/measure'
  //   Sum: '<S68>/Sum2'
  //   Sum: '<S68>/Sum3'

  cmd_V_NED[0] = (rtU.measure.Omega_Kb_raw[0] - (0.0263680723F *
    rtDW.DiscreteTimeIntegratory_dt_DS_m[0] +
    rtDW.DiscreteTimeIntegratory_DSTAT_g[0])) * 5753.11719F;

  // Sum: '<S70>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y_dt'
  //   Gain: '<S70>/2*d//omega'
  //   Inport: '<Root>/measure'
  //   Sum: '<Root>/Add'
  //   Sum: '<S70>/Sum3'

  rtb_uDLookupTable = rtU.measure.a_Kg[0] - (0.0263680723F *
    rtDW.DiscreteTimeIntegratory_dt_DS_f[0] +
    rtDW.DiscreteTimeIntegratory_DSTAT_b[0]);

  // Gain: '<S71>/omega^2' incorporates:
  //   DiscreteIntegrator: '<S71>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S71>/Discrete-Time Integrator y_dt'
  //   Gain: '<S71>/2*d//omega'
  //   Inport: '<Root>/measure'
  //   Sum: '<S71>/Sum2'
  //   Sum: '<S71>/Sum3'

  rtb_n_dt_d[0] = (rtU.measure.V_Kg[0] - (0.0263680723F *
    rtDW.DiscreteTimeIntegratory_dt_D_b5[0] +
    rtDW.DiscreteTimeIntegratory_DSTAT_d[0])) * 5753.11719F;

  // Sum: '<S72>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S72>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S72>/Discrete-Time Integrator y_dt'
  //   Gain: '<S72>/2*d//omega'
  //   Inport: '<Root>/measure'
  //   Sum: '<S72>/Sum3'

  rtb_Sum2_c[0] = rtU.measure.s_Kg[0] - (0.0263680723F *
    rtDW.DiscreteTimeIntegratory_dt_DS_n[0] +
    rtDW.DiscreteTimeIntegratory_DSTATE[0]);

  // Update for DiscreteIntegrator: '<S72>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S72>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTATE[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_n[0];

  // Update for DiscreteIntegrator: '<S55>/Discrete-Time Integrator' incorporates:
  //   DiscreteIntegrator: '<S59>/Discrete-Time Integrator'

  rtDW.DiscreteTimeIntegrator_DSTATE[0] += 0.0025F *
    rtDW.DiscreteTimeIntegrator_DSTAT_my[0];

  // Update for DiscreteIntegrator: '<S71>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S71>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_d[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_D_b5[0];

  // Update for DiscreteIntegrator: '<S70>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_b[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_f[0];

  // Update for DiscreteIntegrator: '<S59>/Discrete-Time Integrator' incorporates:
  //   Saturate: '<S59>/Saturation'

  rtDW.DiscreteTimeIntegrator_DSTAT_my[0] += 0.0025F * q1_q2;

  // Gain: '<S68>/omega^2' incorporates:
  //   DiscreteIntegrator: '<S68>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S68>/Discrete-Time Integrator y_dt'
  //   Gain: '<S68>/2*d//omega'
  //   Inport: '<Root>/measure'
  //   Sum: '<S68>/Sum2'
  //   Sum: '<S68>/Sum3'

  cmd_V_NED[1] = (rtU.measure.Omega_Kb_raw[1] - (0.0263680723F *
    rtDW.DiscreteTimeIntegratory_dt_DS_m[1] +
    rtDW.DiscreteTimeIntegratory_DSTAT_g[1])) * 5753.11719F;

  // Sum: '<S70>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y_dt'
  //   Gain: '<S70>/2*d//omega'
  //   Inport: '<Root>/measure'
  //   Sum: '<Root>/Add'
  //   Sum: '<S70>/Sum3'

  q1_q2 = rtU.measure.a_Kg[1] - (0.0263680723F *
    rtDW.DiscreteTimeIntegratory_dt_DS_f[1] +
    rtDW.DiscreteTimeIntegratory_DSTAT_b[1]);

  // Gain: '<S71>/omega^2' incorporates:
  //   DiscreteIntegrator: '<S71>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S71>/Discrete-Time Integrator y_dt'
  //   Gain: '<S71>/2*d//omega'
  //   Inport: '<Root>/measure'
  //   Sum: '<S71>/Sum2'
  //   Sum: '<S71>/Sum3'

  rtb_n_dt_d[1] = (rtU.measure.V_Kg[1] - (0.0263680723F *
    rtDW.DiscreteTimeIntegratory_dt_D_b5[1] +
    rtDW.DiscreteTimeIntegratory_DSTAT_d[1])) * 5753.11719F;

  // Sum: '<S72>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S72>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S72>/Discrete-Time Integrator y_dt'
  //   Gain: '<S72>/2*d//omega'
  //   Inport: '<Root>/measure'
  //   Sum: '<S72>/Sum3'

  rtb_Sum2_c[1] = rtU.measure.s_Kg[1] - (0.0263680723F *
    rtDW.DiscreteTimeIntegratory_dt_DS_n[1] +
    rtDW.DiscreteTimeIntegratory_DSTATE[1]);

  // Update for DiscreteIntegrator: '<S72>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S72>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTATE[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_n[1];

  // Update for DiscreteIntegrator: '<S55>/Discrete-Time Integrator' incorporates:
  //   DiscreteIntegrator: '<S59>/Discrete-Time Integrator'

  rtDW.DiscreteTimeIntegrator_DSTATE[1] += 0.0025F *
    rtDW.DiscreteTimeIntegrator_DSTAT_my[1];

  // Update for DiscreteIntegrator: '<S71>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S71>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_d[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_D_b5[1];

  // Update for DiscreteIntegrator: '<S70>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_b[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_f[1];

  // Update for DiscreteIntegrator: '<S59>/Discrete-Time Integrator' incorporates:
  //   Saturate: '<S59>/Saturation'

  rtDW.DiscreteTimeIntegrator_DSTAT_my[1] += 0.0025F * q2_q3;

  // Gain: '<S68>/omega^2' incorporates:
  //   DiscreteIntegrator: '<S68>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S68>/Discrete-Time Integrator y_dt'
  //   Gain: '<S68>/2*d//omega'
  //   Inport: '<Root>/measure'
  //   Sum: '<S68>/Sum2'
  //   Sum: '<S68>/Sum3'

  cmd_V_NED[2] = (rtU.measure.Omega_Kb_raw[2] - (0.0263680723F *
    rtDW.DiscreteTimeIntegratory_dt_DS_m[2] +
    rtDW.DiscreteTimeIntegratory_DSTAT_g[2])) * 5753.11719F;

  // Sum: '<S70>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y_dt'
  //   Gain: '<S70>/2*d//omega'
  //   Inport: '<Root>/measure'
  //   Sum: '<Root>/Add'
  //   Sum: '<S70>/Sum3'

  q2_q3 = (rtU.measure.a_Kg[2] + 9.81F) - (0.0263680723F *
    rtDW.DiscreteTimeIntegratory_dt_DS_f[2] +
    rtDW.DiscreteTimeIntegratory_DSTAT_b[2]);

  // Gain: '<S71>/omega^2' incorporates:
  //   DiscreteIntegrator: '<S71>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S71>/Discrete-Time Integrator y_dt'
  //   Gain: '<S71>/2*d//omega'
  //   Inport: '<Root>/measure'
  //   Sum: '<S71>/Sum2'
  //   Sum: '<S71>/Sum3'

  rtb_n_dt_d[2] = (rtU.measure.V_Kg[2] - (0.0263680723F *
    rtDW.DiscreteTimeIntegratory_dt_D_b5[2] +
    rtDW.DiscreteTimeIntegratory_DSTAT_d[2])) * 5753.11719F;

  // Sum: '<S72>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S72>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S72>/Discrete-Time Integrator y_dt'
  //   Gain: '<S72>/2*d//omega'
  //   Inport: '<Root>/measure'
  //   Sum: '<S72>/Sum3'

  rtb_Sum2_c[2] = rtU.measure.s_Kg[2] - (0.0263680723F *
    rtDW.DiscreteTimeIntegratory_dt_DS_n[2] +
    rtDW.DiscreteTimeIntegratory_DSTATE[2]);

  // Update for DiscreteIntegrator: '<S72>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S72>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTATE[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_n[2];

  // Update for DiscreteIntegrator: '<S55>/Discrete-Time Integrator' incorporates:
  //   DiscreteIntegrator: '<S59>/Discrete-Time Integrator'

  rtDW.DiscreteTimeIntegrator_DSTATE[2] += 0.0025F *
    rtDW.DiscreteTimeIntegrator_DSTAT_my[2];

  // Update for DiscreteIntegrator: '<S71>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S71>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_d[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_D_b5[2];

  // Update for DiscreteIntegrator: '<S70>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_b[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_f[2];

  // Update for DiscreteIntegrator: '<S59>/Discrete-Time Integrator' incorporates:
  //   Saturate: '<S59>/Saturation'

  rtDW.DiscreteTimeIntegrator_DSTAT_my[2] += 0.0025F * q1_q3;
  rtDW.DiscreteTimeIntegrator_PrevRe_g = 0;

  // Update for DiscreteIntegrator: '<S62>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LO_b = 0U;

  // Update for DiscreteIntegrator: '<S69>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LO_g = 0U;

  // Update for DiscreteIntegrator: '<S49>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LO_m = 0U;

  // Update for DiscreteIntegrator: '<S50>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_L_mh = 0U;

  // Update for DiscreteIntegrator: '<S68>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LO_d = 0U;

  // Update for DiscreteIntegrator: '<S49>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S49>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_j[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DSTA[0];

  // Update for DiscreteIntegrator: '<S49>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DSTA[0] += 0.0025F * rtb_n_g_des[0];

  // Update for DiscreteIntegrator: '<S50>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S50>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_i[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_b[0];

  // Update for DiscreteIntegrator: '<S50>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DS_b[0] += 0.0025F * rtb_omega2_e_idx_0;

  // Update for DiscreteIntegrator: '<S68>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S68>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_g[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_m[0];

  // Update for DiscreteIntegrator: '<S68>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DS_m[0] += 0.0025F * cmd_V_NED[0];

  // Update for DiscreteIntegrator: '<S49>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S49>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_j[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DSTA[1];

  // Update for DiscreteIntegrator: '<S49>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DSTA[1] += 0.0025F * rtb_n_g_des[1];

  // Update for DiscreteIntegrator: '<S50>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S50>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_i[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_b[1];

  // Update for DiscreteIntegrator: '<S50>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DS_b[1] += 0.0025F * rtb_omega2_e_idx_1;

  // Update for DiscreteIntegrator: '<S68>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S68>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_g[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_m[1];

  // Update for DiscreteIntegrator: '<S68>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DS_m[1] += 0.0025F * cmd_V_NED[1];

  // Update for DiscreteIntegrator: '<S49>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S49>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_j[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DSTA[2];

  // Update for DiscreteIntegrator: '<S49>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DSTA[2] += 0.0025F * rtb_n_g_des_a;

  // Update for DiscreteIntegrator: '<S50>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S50>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_i[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_b[2];

  // Update for DiscreteIntegrator: '<S50>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DS_b[2] += 0.0025F * rtb_omega2_e_idx_2;

  // Update for DiscreteIntegrator: '<S68>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S68>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_g[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_m[2];

  // Update for DiscreteIntegrator: '<S68>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DS_m[2] += 0.0025F * cmd_V_NED[2];

  // Update for DiscreteIntegrator: '<S52>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_DSTATE_b += 0.0025F * q2_q2;

  // Update for DiscreteIntegrator: '<S38>/Discrete-Time Integrator2'
  rtDW.DiscreteTimeIntegrator2_IC_LOAD = 0U;
  rtDW.DiscreteTimeIntegrator2_DSTATE += 0.0025F * q1_q1;

  // Update for DiscreteIntegrator: '<S31>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S31>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_IC_LO_j = 0U;
  rtDW.DiscreteTimeIntegratory_DSTA_gx += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_D_mp;

  // Update for DiscreteIntegrator: '<S27>/Discrete-Time Integrator' incorporates:
  //   Gain: '<S27>/1//T'
  //   Sum: '<S27>/Sum2'

  rtDW.DiscreteTimeIntegrator_DSTATE_n[0] += (rtb_Switch[0] -
    rtDW.DiscreteTimeIntegrator_DSTATE_n[0]) * 37.9246521F * 0.0025F;

  // Update for UnitDelay: '<S20>/Unit Delay1'
  rtDW.UnitDelay1_DSTATE[0] = rtb_uDLookupTable2[0];

  // Update for DiscreteIntegrator: '<S22>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S22>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTA_j4[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_i[0];

  // Update for DiscreteIntegrator: '<S27>/Discrete-Time Integrator' incorporates:
  //   Gain: '<S27>/1//T'
  //   Sum: '<S27>/Sum2'

  rtDW.DiscreteTimeIntegrator_DSTATE_n[1] += (rtb_Switch[1] -
    rtDW.DiscreteTimeIntegrator_DSTATE_n[1]) * 37.9246521F * 0.0025F;

  // Update for UnitDelay: '<S20>/Unit Delay1'
  rtDW.UnitDelay1_DSTATE[1] = rtb_uDLookupTable2[1];

  // Update for DiscreteIntegrator: '<S22>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S22>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTA_j4[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_i[1];

  // Update for DiscreteIntegrator: '<S27>/Discrete-Time Integrator' incorporates:
  //   Gain: '<S27>/1//T'
  //   Sum: '<S27>/Sum2'

  rtDW.DiscreteTimeIntegrator_DSTATE_n[2] += (rtb_Switch[2] -
    rtDW.DiscreteTimeIntegrator_DSTATE_n[2]) * 37.9246521F * 0.0025F;

  // Update for UnitDelay: '<S20>/Unit Delay1'
  rtDW.UnitDelay1_DSTATE[2] = rtb_uDLookupTable2[2];

  // Update for DiscreteIntegrator: '<S22>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S22>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTA_j4[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_i[2];

  // Update for DiscreteIntegrator: '<S27>/Discrete-Time Integrator' incorporates:
  //   Gain: '<S27>/1//T'
  //   Sum: '<S27>/Sum2'

  rtDW.DiscreteTimeIntegrator_DSTATE_n[3] += (t -
    rtDW.DiscreteTimeIntegrator_DSTATE_n[3]) * 37.9246521F * 0.0025F;

  // Update for UnitDelay: '<S20>/Unit Delay1'
  rtDW.UnitDelay1_DSTATE[3] = rtb_uDLookupTable2[3];

  // Update for DiscreteIntegrator: '<S22>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S22>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTA_j4[3] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_i[3];
  for (i = 0; i < 9; i++) {
    // Update for DiscreteIntegrator: '<S62>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S62>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTAT_l[i] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_h[i];

    // Update for DiscreteIntegrator: '<S69>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S69>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTAT_k[i] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_a[i];

    // Update for DiscreteIntegrator: '<S69>/Discrete-Time Integrator y_dt' incorporates:
    //   Gain: '<S69>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_DS_a[i] += 5753.11719F * rtb_M_bg[i] *
      0.0025F;

    // Update for DiscreteIntegrator: '<S62>/Discrete-Time Integrator y_dt' incorporates:
    //   Gain: '<S62>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_DS_h[i] += 110.978325F * rtb_Add_h[i] *
      0.0025F;
  }

  // Update for DiscreteIntegrator: '<S22>/Discrete-Time Integrator y_dt' incorporates:
  //   Product: '<S22>/Product1'

  rtDW.DiscreteTimeIntegratory_dt_DS_i[0] += rtb_Max[0] * 5753.11719F * 0.0025F;
  rtDW.DiscreteTimeIntegratory_dt_DS_i[1] += rtb_Max[1] * 5753.11719F * 0.0025F;
  rtDW.DiscreteTimeIntegratory_dt_DS_i[2] += rtb_Max[2] * 5753.11719F * 0.0025F;
  rtDW.DiscreteTimeIntegratory_dt_DS_i[3] += rtb_Max[3] * 5753.11719F * 0.0025F;

  // Update for DiscreteIntegrator: '<S70>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S70>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_f[0] += 5753.11719F * rtb_uDLookupTable *
    0.0025F;

  // Update for DiscreteIntegrator: '<S71>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_D_b5[0] += 0.0025F * rtb_n_dt_d[0];

  // Update for DiscreteIntegrator: '<S72>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S72>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_n[0] += 5753.11719F * rtb_Sum2_c[0] *
    0.0025F;

  // Update for DiscreteIntegrator: '<S70>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S70>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_f[1] += 5753.11719F * q1_q2 * 0.0025F;

  // Update for DiscreteIntegrator: '<S71>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_D_b5[1] += 0.0025F * rtb_n_dt_d[1];

  // Update for DiscreteIntegrator: '<S72>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S72>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_n[1] += 5753.11719F * rtb_Sum2_c[1] *
    0.0025F;

  // Update for DiscreteIntegrator: '<S70>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S70>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_f[2] += 5753.11719F * q2_q3 * 0.0025F;

  // Update for DiscreteIntegrator: '<S71>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_D_b5[2] += 0.0025F * rtb_n_dt_d[2];

  // Update for DiscreteIntegrator: '<S72>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S72>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_n[2] += 5753.11719F * rtb_Sum2_c[2] *
    0.0025F;

  // Update for DiscreteIntegrator: '<S31>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S31>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_D_mp += 212.763184F * q0_q0 * 0.0025F;
}

// Model initialize function
void MatlabControllerClass::initialize()
{
  // InitializeConditions for DiscreteIntegrator: '<S72>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LOAD = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S55>/Discrete-Time Integrator' 
  rtDW.DiscreteTimeIntegrator_IC_LOADI = 1U;
  rtDW.DiscreteTimeIntegrator_PrevRese = 2;

  // InitializeConditions for DiscreteIntegrator: '<S71>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LO_f = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S70>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LO_i = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S7>/Discrete-Time Integrator' 
  rtDW.DiscreteTimeIntegrator_PrevRe_h = 2;

  // InitializeConditions for DiscreteIntegrator: '<S59>/Discrete-Time Integrator' 
  rtDW.DiscreteTimeIntegrator_IC_LOA_j = 1U;
  rtDW.DiscreteTimeIntegrator_PrevRe_g = 2;

  // InitializeConditions for DiscreteIntegrator: '<S62>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LO_b = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S69>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LO_g = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S49>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LO_m = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S50>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_L_mh = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S68>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LO_d = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S38>/Discrete-Time Integrator2' 
  rtDW.DiscreteTimeIntegrator2_IC_LOAD = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S31>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LO_j = 1U;
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
