//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: MatlabController.cpp
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
  }, { 11U,
    { 68U, 118U, 49U, 68U, 118U, 50U, 68U, 118U, 51U, 68U, 118U, 52U, 112U, 1U,
      1U, 113U, 1U, 1U, 114U, 1U, 1U, 119U, 49U, 1U, 119U, 50U, 1U, 119U, 51U,
      1U, 119U, 52U, 1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U },

    { 77U, 76U, 51U, 0U }
  }, { 12U,
    { 113U, 49U, 1U, 113U, 50U, 1U, 113U, 51U, 1U, 113U, 52U, 1U, 112U, 100U,
      116U, 113U, 100U, 116U, 114U, 100U, 116U, 97U, 102U, 49U, 97U, 102U, 50U,
      97U, 102U, 51U, 97U, 102U, 52U, 97U, 102U, 53U, 0U, 0U, 0U, 0U, 0U, 0U },

    { 77U, 76U, 52U, 0U }
  }, { 8U,
    { 119U, 112U, 117U, 105U, 115U, 103U, 116U, 115U, 49U, 116U, 115U, 50U, 116U,
      115U, 51U, 116U, 115U, 52U, 116U, 115U, 53U, 116U, 115U, 54U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U },

    { 77U, 76U, 53U, 0U }
  } } ;

extern real32_T rt_roundf(real32_T u);
extern real32_T rt_hypotf(real32_T u0, real32_T u1);
extern real_T rt_urand_Upu32_Yd_f_pw(uint32_T *u);
extern real_T rt_nrand_Upu32_Yd_f_pw(uint32_T *u);
static real32_T look1_iflf_binlx(real32_T u0, const real32_T bp0[], const
  real32_T table[], uint32_T maxIndex);
static void DCM2LeanVector(const real32_T rtu_M_bg[9], real32_T rty_n_g[3]);
static void nrefnorm(const real32_T rtu_y_dt2[3], const real32_T rtu_y_dt[3],
                     const real32_T rtu_y[3], real32_T rty_n_dt2[3], real32_T
                     rty_n_dt[3], real32_T rty_n[3]);
static void DCMtoquaternions(const real32_T rtu_M_bg[9], real32_T rty_q_bg[4]);
static void QuaternionReduced(const real32_T rtu_q_bg[4], real32_T rty_q_red[4],
  real32_T *rty_yaw);
static void wrapangle(real32_T rtu_angle, real32_T *rty_angle_0_2pi);
static void LeanVectorDerivativeTrafo(const real32_T rtu_n_g[3], const real32_T
  rtu_n_g_dt[3], const real32_T rtu_n_g_dt2[3], const real32_T rtu_M_bg[9],
  const real32_T rtu_omega_Kb[3], const real32_T rtu_omega_Kb_dt[3], real32_T
  rty_n_b[3], real32_T rty_n_b_dt[3], real32_T rty_n_b_dt2[3]);

// Forward declaration for local functions
static void leanVectorNormDeriv2(const real32_T nn[3], const real32_T nn_dt[3],
  const real32_T nn_dt2[3], real_T n_dt2[3]);

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
//    '<S14>/DCM 2 Lean Vector'
//    '<S15>/MATLAB Function'
//
static void DCM2LeanVector(const real32_T rtu_M_bg[9], real32_T rty_n_g[3])
{
  int32_T i;
  for (i = 0; i < 3; i++) {
    rty_n_g[i] = 0.0F;
    rty_n_g[i] += -rtu_M_bg[3 * i + 2];
  }
}

// Function for MATLAB Function: '<S70>/n ref norm'
static void leanVectorNormDeriv2(const real32_T nn[3], const real32_T nn_dt[3],
  const real32_T nn_dt2[3], real_T n_dt2[3])
{
  real32_T xyz;
  real32_T a_tmp;
  real32_T n_dt2_tmp;
  real32_T xyz_tmp;
  real32_T xyz_tmp_0;
  real32_T xyz_tmp_1;
  real32_T a_tmp_tmp;
  real32_T a_tmp_tmp_0;
  real32_T a_tmp_tmp_1;
  real32_T n_dt2_tmp_0;
  real32_T n_dt2_tmp_1;
  real32_T n_dt2_tmp_2;
  real32_T n_dt2_tmp_3;
  real32_T n_dt2_tmp_4;
  real32_T n_dt2_tmp_5;
  xyz_tmp = nn[1] * nn[1];
  xyz_tmp_0 = nn[0] * nn[0];
  xyz_tmp_1 = nn[2] * nn[2];
  xyz = (xyz_tmp_0 + xyz_tmp) + xyz_tmp_1;
  a_tmp_tmp = 2.0F * nn[1] * nn_dt[1];
  a_tmp_tmp_0 = 2.0F * nn[0] * nn_dt[0];
  a_tmp_tmp_1 = 2.0F * nn[2] * nn_dt[2];
  a_tmp = (a_tmp_tmp_0 + a_tmp_tmp) + a_tmp_tmp_1;
  n_dt2_tmp = std::pow(xyz, 1.5F);
  n_dt2_tmp_0 = 2.0F * nn[1] * nn_dt2[1];
  n_dt2_tmp_1 = nn_dt[1] * nn_dt[1] * 2.0F;
  n_dt2_tmp_2 = 2.0F * nn[0] * nn_dt2[0];
  n_dt2_tmp_3 = nn_dt[0] * nn_dt[0] * 2.0F;
  n_dt2_tmp_4 = 2.0F * nn[2] * nn_dt2[2];
  n_dt2_tmp_5 = nn_dt[2] * nn_dt[2] * 2.0F;
  n_dt2[0] = (a_tmp * a_tmp * 3.0F / (4.0F * std::pow(xyz, 2.5F)) -
              (((((n_dt2_tmp_2 + n_dt2_tmp_3) + n_dt2_tmp_0) + n_dt2_tmp_1) +
                n_dt2_tmp_4) + n_dt2_tmp_5) / (2.0F * n_dt2_tmp)) * nn[0] +
    (nn_dt2[0] / std::sqrt(xyz) - a_tmp * nn_dt[0] / n_dt2_tmp);
  xyz = (xyz_tmp + xyz_tmp_0) + xyz_tmp_1;
  a_tmp = (a_tmp_tmp + a_tmp_tmp_0) + a_tmp_tmp_1;
  n_dt2_tmp = std::pow(xyz, 1.5F);
  n_dt2[1] = (a_tmp * a_tmp * 3.0F / (4.0F * std::pow(xyz, 2.5F)) -
              (((((n_dt2_tmp_0 + n_dt2_tmp_1) + n_dt2_tmp_2) + n_dt2_tmp_3) +
                n_dt2_tmp_4) + n_dt2_tmp_5) / (2.0F * n_dt2_tmp)) * nn[1] +
    (nn_dt2[1] / std::sqrt(xyz) - a_tmp * nn_dt[1] / n_dt2_tmp);
  xyz = (xyz_tmp_1 + xyz_tmp_0) + xyz_tmp;
  a_tmp = (a_tmp_tmp_1 + a_tmp_tmp_0) + a_tmp_tmp;
  n_dt2_tmp = std::pow(xyz, 1.5F);
  n_dt2[2] = (a_tmp * a_tmp * 3.0F / (4.0F * std::pow(xyz, 2.5F)) -
              (((((n_dt2_tmp_4 + n_dt2_tmp_5) + n_dt2_tmp_2) + n_dt2_tmp_3) +
                n_dt2_tmp_0) + n_dt2_tmp_1) / (2.0F * n_dt2_tmp)) * nn[2] +
    (nn_dt2[2] / std::sqrt(xyz) - a_tmp * nn_dt[2] / n_dt2_tmp);
}

//
// Output and update for atomic system:
//    '<S70>/n ref norm'
//    '<S103>/n ref norm'
//
static void nrefnorm(const real32_T rtu_y_dt2[3], const real32_T rtu_y_dt[3],
                     const real32_T rtu_y[3], real32_T rty_n_dt2[3], real32_T
                     rty_n_dt[3], real32_T rty_n[3])
{
  real32_T norm_n;
  real32_T scale;
  real32_T absxk;
  real32_T t;
  real_T tmp[3];
  real32_T scale_0[9];
  int32_T i;
  real32_T norm_n_tmp;
  real32_T scale_tmp;
  real32_T norm_n_tmp_0;
  real32_T tmp_0;
  real32_T scale_tmp_tmp;
  scale = 1.29246971E-26F;
  absxk = std::abs(rtu_y[0]);
  if (absxk > 1.29246971E-26F) {
    norm_n = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    norm_n = t * t;
  }

  absxk = std::abs(rtu_y[1]);
  if (absxk > scale) {
    t = scale / absxk;
    norm_n = norm_n * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    norm_n += t * t;
  }

  absxk = std::abs(rtu_y[2]);
  if (absxk > scale) {
    t = scale / absxk;
    norm_n = norm_n * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    norm_n += t * t;
  }

  norm_n = scale * std::sqrt(norm_n);
  if (norm_n < 2.22044605E-16F) {
    norm_n = 2.22044605E-16F;
  }

  rty_n[0] = rtu_y[0] / norm_n;
  rty_n[1] = rtu_y[1] / norm_n;
  rty_n[2] = rtu_y[2] / norm_n;
  norm_n_tmp = rtu_y[1] * rtu_y[1];
  absxk = rtu_y[2] * rtu_y[2];
  t = rtu_y[0] * rtu_y[0];
  norm_n_tmp_0 = t + norm_n_tmp;
  norm_n = std::pow(norm_n_tmp_0 + absxk, 1.5F);
  scale = norm_n;
  tmp_0 = std::abs(norm_n);
  if (tmp_0 < 2.22044605E-16F) {
    scale = 2.22044605E-16F;
  }

  scale = 1.0F / scale;
  scale_0[0] = (norm_n_tmp + absxk) * scale;
  norm_n_tmp = -rtu_y[0] * rtu_y[1] * scale;
  scale_0[3] = norm_n_tmp;
  scale_tmp_tmp = -rtu_y[0] * rtu_y[2];
  scale_tmp = scale_tmp_tmp * scale;
  scale_0[6] = scale_tmp;
  scale_0[1] = norm_n_tmp;
  scale_0[4] = (t + absxk) * scale;
  norm_n_tmp = -rtu_y[1] * rtu_y[2] * scale;
  scale_0[7] = norm_n_tmp;
  scale_0[2] = scale_tmp;
  scale_0[5] = norm_n_tmp;
  scale_0[8] = norm_n_tmp_0 * scale;
  for (i = 0; i < 3; i++) {
    rty_n_dt[i] = 0.0F;
    rty_n_dt[i] += scale_0[i] * rtu_y_dt[0];
    rty_n_dt[i] += scale_0[i + 3] * rtu_y_dt[1];
    rty_n_dt[i] += scale_0[i + 6] * rtu_y_dt[2];
  }

  if (tmp_0 < 2.22044605E-16F) {
    norm_n = 2.22044605E-16F;
  }

  rty_n_dt[2] = ((scale_tmp_tmp * rtu_y_dt[0] + t * rtu_y_dt[2]) + (rtu_y[1] *
    rtu_y_dt[2] - rtu_y[2] * rtu_y_dt[1]) * rtu_y[1]) * (1.0F / norm_n);
  leanVectorNormDeriv2(rtu_y, rtu_y_dt, rtu_y_dt2, tmp);
  rty_n_dt2[0] = (real32_T)tmp[0];
  rty_n_dt2[1] = (real32_T)tmp[1];
  rty_n_dt2[2] = (real32_T)tmp[2];
}

//
// Output and update for atomic system:
//    '<S80>/DCM to quaternions'
//    '<S85>/DCM to quaternions'
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

// Function for MATLAB Function: '<S80>/Quaternion Reduced'
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
//    '<S80>/Quaternion Reduced'
//    '<S85>/Quaternion Reduced'
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
//    '<S84>/wrap angle'
//    '<S84>/wrap angle1'
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
//    '<S89>/Lean Vector Derivative Trafo'
//    '<S89>/Lean Vector Derivative Trafo Delay'
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

real32_T rt_roundf(real32_T u)
{
  real32_T y;
  if (std::abs(u) < 8.388608E+6F) {
    if (u >= 0.5F) {
      y = std::floor(u + 0.5F);
    } else if (u > -0.5F) {
      y = 0.0F;
    } else {
      y = std::ceil(u - 0.5F);
    }
  } else {
    y = u;
  }

  return y;
}

// Function for MATLAB Function: '<S56>/trajFromWaypoints'
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

// Function for MATLAB Function: '<S56>/trajFromWaypoints'
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

// Function for MATLAB Function: '<S56>/trajFromWaypoints'
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

// Function for MATLAB Function: '<S56>/trajFromWaypoints'
void MatlabControllerClass::trajSetArcLength(dtoSgl_trajectoryStructBus *traj)
{
  real_T total_arc_length;
  real_T total_distance;
  real32_T distance;
  int32_T k;
  real_T section_idx;
  real32_T scale;
  real32_T absxk;
  real32_T t;
  int32_T b_k;
  real32_T dx_data[5];
  real32_T dy_data[5];
  real32_T dz_data[5];
  real32_T d_a;
  real32_T e_a;
  real32_T f_a;
  real32_T l_a;
  real32_T tmp[3];
  real32_T tmp_0[3];
  int32_T dx_size[2];
  int32_T dy_size[2];
  int32_T dz_size[2];
  total_arc_length = 0.0;
  total_distance = 0.0;
  for (k = 0; k < (int32_T)traj->num_sections_set; k++) {
    section_idx = 1.0F + (real32_T)k;
    if (1.0F + (real32_T)k > traj->num_sections_set) {
      section_idx = 1.0;
    }

    trajSectionGetPos(traj->sections[(int32_T)section_idx - 1].pos_x,
                      traj->sections[(int32_T)section_idx - 1].pos_y,
                      traj->sections[(int32_T)section_idx - 1].pos_z, 1.0, tmp);
    trajSectionGetPos(traj->sections[(int32_T)section_idx - 1].pos_x,
                      traj->sections[(int32_T)section_idx - 1].pos_y,
                      traj->sections[(int32_T)section_idx - 1].pos_z, 0.0, tmp_0);
    scale = 1.29246971E-26F;
    absxk = std::abs(tmp[0] - tmp_0[0]);
    if (absxk > 1.29246971E-26F) {
      distance = 1.0F;
      scale = absxk;
    } else {
      t = absxk / 1.29246971E-26F;
      distance = t * t;
    }

    absxk = std::abs(tmp[1] - tmp_0[1]);
    if (absxk > scale) {
      t = scale / absxk;
      distance = distance * t * t + 1.0F;
      scale = absxk;
    } else {
      t = absxk / scale;
      distance += t * t;
    }

    absxk = std::abs(tmp[2] - tmp_0[2]);
    if (absxk > scale) {
      t = scale / absxk;
      distance = distance * t * t + 1.0F;
      scale = absxk;
    } else {
      t = absxk / scale;
      distance += t * t;
    }

    distance = scale * std::sqrt(distance);
    polyder_p(traj->sections[(int32_T)section_idx - 1].pos_x, dx_data, dx_size);
    polyder_p(traj->sections[(int32_T)section_idx - 1].pos_y, dy_data, dy_size);
    polyder_p(traj->sections[(int32_T)section_idx - 1].pos_z, dz_data, dz_size);
    section_idx = 0.0;
    scale = polyVal(dx_data, dx_size, 0.0);
    absxk = polyVal(dy_data, dy_size, 0.0);
    t = polyVal(dz_data, dz_size, 0.0);
    d_a = polyVal(dx_data, dx_size, 1.0);
    e_a = polyVal(dy_data, dy_size, 1.0);
    f_a = polyVal(dz_data, dz_size, 1.0);
    scale = std::sqrt((scale * scale + absxk * absxk) + t * t) * 0.5F - std::
      sqrt((d_a * d_a + e_a * e_a) + f_a * f_a) * 0.5F;
    for (b_k = 0; b_k < 15; b_k++) {
      section_idx += 0.066666666666666666;
      absxk = polyVal(dx_data, dx_size, section_idx - 0.033333333333333333);
      t = polyVal(dy_data, dy_size, section_idx - 0.033333333333333333);
      d_a = polyVal(dz_data, dz_size, section_idx - 0.033333333333333333);
      e_a = polyVal(dx_data, dx_size, section_idx);
      f_a = polyVal(dy_data, dy_size, section_idx);
      l_a = polyVal(dz_data, dz_size, section_idx);
      scale = (std::sqrt((absxk * absxk + t * t) + d_a * d_a) * 2.0F + scale) +
        std::sqrt((e_a * e_a + f_a * f_a) + l_a * l_a);
    }

    scale *= 0.0222222228F;
    b_k = (int32_T)(1.0F + (real32_T)k) - 1;
    traj->sections[b_k].arc_length = scale;
    traj->sections[b_k].distance = distance;
    total_arc_length = (real32_T)total_arc_length + scale;
    total_distance = (real32_T)total_distance + distance;
  }

  traj->distance = (real32_T)total_distance;
  traj->arc_length = (real32_T)total_arc_length;
}

// Function for MATLAB Function: '<S56>/trajFromWaypoints'
void MatlabControllerClass::polyder(const real_T u_data[], const int32_T u_size
  [2], real_T a_data[], int32_T a_size[2])
{
  int32_T nlead0;
  int32_T ny;
  int32_T c_k;
  nlead0 = 0;
  ny = 0;
  while ((ny <= u_size[1] - 3) && (u_data[ny] == 0.0)) {
    nlead0++;
    ny++;
  }

  ny = (u_size[1] - nlead0) - 1;
  a_size[0] = 1;
  a_size[1] = ny;
  for (c_k = 0; c_k < ny; c_k++) {
    a_data[c_k] = u_data[c_k + nlead0];
  }

  nlead0 = ny - 2;
  for (ny = 0; ny <= nlead0; ny++) {
    a_data[ny] *= (real_T)((nlead0 - ny) + 1) + 1.0;
  }
}

// Function for MATLAB Function: '<S56>/trajFromWaypoints'
void MatlabControllerClass::polyInterpolationAx_f(real32_T num_of_splines,
  boolean_T cycle, const real32_T x_data[], const int32_T *x_size, real32_T
  b_data[], int32_T *b_size)
{
  real_T pp[36];
  real_T point_0[36];
  real32_T intermediate_size;
  int32_T bnd_left;
  int32_T l;
  int32_T n_data[60];
  int32_T w_data[57];
  real_T pp_data[6];
  real_T tmp_data[5];
  real32_T b[3];
  real32_T b_data_0[60];
  real32_T x[6];
  real32_T b_0[4];
  real32_T b_data_1[57];
  real32_T point_0_0;
  int32_T loop_ub;
  int32_T pp_size[2];
  int32_T tmp_size[2];
  int8_T c_x_idx_0;
  int32_T tmp;
  int8_T c_idx_0;
  real32_T intermediate_size_tmp;
  int32_T n_size_idx_1_tmp;
  int32_T b_data_tmp;
  int32_T j_tmp;
  c_x_idx_0 = (int8_T)*x_size;
  c_idx_0 = (int8_T)*x_size;
  *b_size = c_x_idx_0;
  if (0 <= c_x_idx_0 - 1) {
    memset(&b_data[0], 0, c_x_idx_0 * sizeof(real32_T));
  }

  for (b_data_tmp = 0; b_data_tmp < 36; b_data_tmp++) {
    pp[b_data_tmp] = 1.0;
  }

  for (b_data_tmp = 0; b_data_tmp < 6; b_data_tmp++) {
    for (j_tmp = 0; j_tmp < 5; j_tmp++) {
      pp[(j_tmp + 6 * b_data_tmp) + 1] = 0.0;
    }
  }

  for (bnd_left = 0; bnd_left < 5; bnd_left++) {
    loop_ub = 6 - bnd_left;
    pp_size[0] = 1;
    pp_size[1] = loop_ub;
    for (b_data_tmp = 0; b_data_tmp < loop_ub; b_data_tmp++) {
      pp_data[b_data_tmp] = pp[6 * b_data_tmp + bnd_left];
    }

    polyder(pp_data, pp_size, tmp_data, tmp_size);
    loop_ub = tmp_size[1];
    for (b_data_tmp = 0; b_data_tmp < loop_ub; b_data_tmp++) {
      pp[(bnd_left + 6 * b_data_tmp) + 1] = tmp_data[b_data_tmp];
    }
  }

  memcpy(&point_0[0], &pp[0], 36U * sizeof(real_T));
  for (bnd_left = 0; bnd_left < 5; bnd_left++) {
    tmp = 5 - bnd_left;
    for (b_data_tmp = 0; b_data_tmp < tmp; b_data_tmp++) {
      point_0[bnd_left + 6 * b_data_tmp] = 0.0;
    }
  }

  intermediate_size_tmp = (num_of_splines - 1.0F) * 6.0F;
  if (!cycle) {
    bnd_left = 3;
    *b_size = c_idx_0;
    if (0 <= c_idx_0 - 1) {
      memset(&b_data[0], 0, c_idx_0 * sizeof(real32_T));
    }

    for (b_data_tmp = 0; b_data_tmp < 6; b_data_tmp++) {
      x[b_data_tmp] = x_data[b_data_tmp];
    }

    for (b_data_tmp = 0; b_data_tmp < 3; b_data_tmp++) {
      point_0_0 = 0.0F;
      for (j_tmp = 0; j_tmp < 6; j_tmp++) {
        point_0_0 += (real32_T)point_0[6 * j_tmp + b_data_tmp] * x[j_tmp];
      }

      b_data[b_data_tmp] = point_0_0;
    }

    if (4.0F + intermediate_size_tmp > ((4.0F + intermediate_size_tmp) + 3.0F) -
        1.0F) {
      tmp = 0;
      j_tmp = 0;
      loop_ub = 0;
      l = 0;
    } else {
      tmp = (int32_T)(4.0F + intermediate_size_tmp) - 1;
      j_tmp = (int32_T)(((4.0F + intermediate_size_tmp) + 3.0F) - 1.0F);
      loop_ub = tmp;
      l = j_tmp;
    }

    n_size_idx_1_tmp = l - loop_ub;
    for (b_data_tmp = 0; b_data_tmp < n_size_idx_1_tmp; b_data_tmp++) {
      n_data[b_data_tmp] = loop_ub + b_data_tmp;
    }

    loop_ub = j_tmp - tmp;
    for (b_data_tmp = 0; b_data_tmp < loop_ub; b_data_tmp++) {
      b_data_0[b_data_tmp] = b_data[tmp + b_data_tmp];
    }

    for (b_data_tmp = 0; b_data_tmp < 6; b_data_tmp++) {
      x[b_data_tmp] = x_data[(int32_T)((1.0F + intermediate_size_tmp) +
        (real32_T)b_data_tmp) - 1];
    }

    for (b_data_tmp = 0; b_data_tmp < 3; b_data_tmp++) {
      point_0_0 = 0.0F;
      for (j_tmp = 0; j_tmp < 6; j_tmp++) {
        point_0_0 += (real32_T)pp[6 * j_tmp + b_data_tmp] * x[j_tmp];
      }

      b[b_data_tmp] = b_data_0[b_data_tmp] + point_0_0;
    }

    for (b_data_tmp = 0; b_data_tmp < n_size_idx_1_tmp; b_data_tmp++) {
      b_data[n_data[b_data_tmp]] = b[b_data_tmp];
    }
  } else {
    bnd_left = 2;
    point_0_0 = 0.0F;
    for (b_data_tmp = 0; b_data_tmp < 6; b_data_tmp++) {
      point_0_0 += (real32_T)point_0[6 * b_data_tmp] * x_data[b_data_tmp];
    }

    b_data[0] = point_0_0;
    point_0_0 = 0.0F;
    for (b_data_tmp = 0; b_data_tmp < 6; b_data_tmp++) {
      point_0_0 += x_data[(int32_T)((intermediate_size_tmp + 1.0F) + (real32_T)
        b_data_tmp) - 1] * (real32_T)pp[6 * b_data_tmp];
    }

    b_data[1] = point_0_0;
    if ((intermediate_size_tmp + 2.0F) + 1.0F > (((intermediate_size_tmp + 2.0F)
          + 1.0F) + 5.0F) - 2.0F) {
      tmp = 0;
      j_tmp = 0;
      loop_ub = 0;
      l = 0;
    } else {
      tmp = (int32_T)((intermediate_size_tmp + 2.0F) + 1.0F) - 1;
      j_tmp = (int32_T)((((intermediate_size_tmp + 2.0F) + 1.0F) + 5.0F) - 2.0F);
      loop_ub = tmp;
      l = j_tmp;
    }

    n_size_idx_1_tmp = l - loop_ub;
    for (b_data_tmp = 0; b_data_tmp < n_size_idx_1_tmp; b_data_tmp++) {
      n_data[b_data_tmp] = loop_ub + b_data_tmp;
    }

    loop_ub = j_tmp - tmp;
    for (b_data_tmp = 0; b_data_tmp < loop_ub; b_data_tmp++) {
      b_data_0[b_data_tmp] = b_data[tmp + b_data_tmp];
    }

    for (b_data_tmp = 0; b_data_tmp < 6; b_data_tmp++) {
      x[b_data_tmp] = x_data[b_data_tmp];
    }

    for (b_data_tmp = 0; b_data_tmp < 4; b_data_tmp++) {
      point_0_0 = 0.0F;
      for (j_tmp = 0; j_tmp < 6; j_tmp++) {
        point_0_0 += (real32_T)point_0[(6 * j_tmp + b_data_tmp) + 1] * x[j_tmp];
      }

      b_0[b_data_tmp] = b_data_0[b_data_tmp] + point_0_0;
    }

    for (b_data_tmp = 0; b_data_tmp < n_size_idx_1_tmp; b_data_tmp++) {
      b_data[n_data[b_data_tmp]] = b_0[b_data_tmp];
    }

    if ((intermediate_size_tmp + 2.0F) + 1.0F > (((intermediate_size_tmp + 2.0F)
          + 1.0F) + 5.0F) - 2.0F) {
      tmp = 0;
      j_tmp = 0;
      loop_ub = 0;
      l = 0;
    } else {
      tmp = (int32_T)((intermediate_size_tmp + 2.0F) + 1.0F) - 1;
      j_tmp = (int32_T)((((intermediate_size_tmp + 2.0F) + 1.0F) + 5.0F) - 2.0F);
      loop_ub = (int32_T)((intermediate_size_tmp + 2.0F) + 1.0F) - 1;
      l = (int32_T)((((intermediate_size_tmp + 2.0F) + 1.0F) + 5.0F) - 2.0F);
    }

    n_size_idx_1_tmp = l - loop_ub;
    for (b_data_tmp = 0; b_data_tmp < n_size_idx_1_tmp; b_data_tmp++) {
      n_data[b_data_tmp] = loop_ub + b_data_tmp;
    }

    loop_ub = j_tmp - tmp;
    for (b_data_tmp = 0; b_data_tmp < loop_ub; b_data_tmp++) {
      b_data_0[b_data_tmp] = b_data[tmp + b_data_tmp];
    }

    for (b_data_tmp = 0; b_data_tmp < 6; b_data_tmp++) {
      x[b_data_tmp] = x_data[(int32_T)((intermediate_size_tmp + 1.0F) +
        (real32_T)b_data_tmp) - 1];
    }

    for (b_data_tmp = 0; b_data_tmp < 4; b_data_tmp++) {
      point_0_0 = 0.0F;
      for (j_tmp = 0; j_tmp < 6; j_tmp++) {
        point_0_0 += (real32_T)pp[(6 * j_tmp + b_data_tmp) + 1] * x[j_tmp];
      }

      b_0[b_data_tmp] = b_data_0[b_data_tmp] - point_0_0;
    }

    for (b_data_tmp = 0; b_data_tmp < n_size_idx_1_tmp; b_data_tmp++) {
      b_data[n_data[b_data_tmp]] = b_0[b_data_tmp];
    }
  }

  for (tmp = 0; tmp < (int32_T)(num_of_splines - 1.0F); tmp++) {
    intermediate_size_tmp = ((1.0F + (real32_T)tmp) - 1.0F) * 6.0F;
    intermediate_size = intermediate_size_tmp + (real32_T)bnd_left;
    point_0_0 = 0.0F;
    for (b_data_tmp = 0; b_data_tmp < 6; b_data_tmp++) {
      point_0_0 += x_data[(int32_T)((intermediate_size_tmp + 1.0F) + (real32_T)
        b_data_tmp) - 1] * (real32_T)pp[6 * b_data_tmp];
    }

    b_data_tmp = (int32_T)(intermediate_size + 1.0F) - 1;
    b_data[b_data_tmp] += point_0_0;
    point_0_0 = 0.0F;
    for (b_data_tmp = 0; b_data_tmp < 6; b_data_tmp++) {
      point_0_0 += x_data[(int32_T)(((intermediate_size_tmp + 1.0F) + 6.0F) +
        (real32_T)b_data_tmp) - 1] * (real32_T)point_0[6 * b_data_tmp];
    }

    b_data_tmp = (int32_T)(intermediate_size + 2.0F) - 1;
    b_data[b_data_tmp] += point_0_0;
    if (intermediate_size + 3.0F > ((intermediate_size + 3.0F) + 5.0F) - 2.0F) {
      j_tmp = 0;
      loop_ub = 0;
      l = 0;
      b_data_tmp = 0;
    } else {
      j_tmp = (int32_T)(intermediate_size + 3.0F) - 1;
      loop_ub = (int32_T)(((intermediate_size + 3.0F) + 5.0F) - 2.0F);
      l = j_tmp;
      b_data_tmp = loop_ub;
    }

    n_size_idx_1_tmp = b_data_tmp - l;
    for (b_data_tmp = 0; b_data_tmp < n_size_idx_1_tmp; b_data_tmp++) {
      w_data[b_data_tmp] = l + b_data_tmp;
    }

    loop_ub -= j_tmp;
    for (b_data_tmp = 0; b_data_tmp < loop_ub; b_data_tmp++) {
      b_data_1[b_data_tmp] = b_data[j_tmp + b_data_tmp];
    }

    for (b_data_tmp = 0; b_data_tmp < 6; b_data_tmp++) {
      x[b_data_tmp] = x_data[(int32_T)((intermediate_size_tmp + 1.0F) +
        (real32_T)b_data_tmp) - 1];
    }

    for (b_data_tmp = 0; b_data_tmp < 4; b_data_tmp++) {
      point_0_0 = 0.0F;
      for (j_tmp = 0; j_tmp < 6; j_tmp++) {
        point_0_0 += (real32_T)pp[(6 * j_tmp + b_data_tmp) + 1] * x[j_tmp];
      }

      b_0[b_data_tmp] = b_data_1[b_data_tmp] + point_0_0;
    }

    for (b_data_tmp = 0; b_data_tmp < n_size_idx_1_tmp; b_data_tmp++) {
      b_data[w_data[b_data_tmp]] = b_0[b_data_tmp];
    }

    if (intermediate_size + 3.0F > ((intermediate_size + 3.0F) + 5.0F) - 2.0F) {
      j_tmp = 0;
      loop_ub = 0;
      l = 0;
      b_data_tmp = 0;
    } else {
      j_tmp = (int32_T)(intermediate_size + 3.0F) - 1;
      loop_ub = (int32_T)(((intermediate_size + 3.0F) + 5.0F) - 2.0F);
      l = (int32_T)(intermediate_size + 3.0F) - 1;
      b_data_tmp = (int32_T)(((intermediate_size + 3.0F) + 5.0F) - 2.0F);
    }

    n_size_idx_1_tmp = b_data_tmp - l;
    for (b_data_tmp = 0; b_data_tmp < n_size_idx_1_tmp; b_data_tmp++) {
      w_data[b_data_tmp] = l + b_data_tmp;
    }

    loop_ub -= j_tmp;
    for (b_data_tmp = 0; b_data_tmp < loop_ub; b_data_tmp++) {
      b_data_1[b_data_tmp] = b_data[j_tmp + b_data_tmp];
    }

    for (b_data_tmp = 0; b_data_tmp < 6; b_data_tmp++) {
      x[b_data_tmp] = x_data[(int32_T)(((intermediate_size_tmp + 1.0F) + 6.0F) +
        (real32_T)b_data_tmp) - 1];
    }

    for (b_data_tmp = 0; b_data_tmp < 4; b_data_tmp++) {
      point_0_0 = 0.0F;
      for (j_tmp = 0; j_tmp < 6; j_tmp++) {
        point_0_0 += (real32_T)point_0[(6 * j_tmp + b_data_tmp) + 1] * x[j_tmp];
      }

      b_0[b_data_tmp] = b_data_1[b_data_tmp] - point_0_0;
    }

    for (b_data_tmp = 0; b_data_tmp < n_size_idx_1_tmp; b_data_tmp++) {
      b_data[w_data[b_data_tmp]] = b_0[b_data_tmp];
    }
  }
}

// Function for MATLAB Function: '<S56>/trajFromWaypoints'
real32_T MatlabControllerClass::norm(const real32_T x_data[], const int32_T
  *x_size)
{
  real32_T y;
  real32_T scale;
  real32_T absxk;
  real32_T t;
  int32_T k;
  if (*x_size == 0) {
    y = 0.0F;
  } else {
    y = 0.0F;
    if (*x_size == 1) {
      y = std::abs(x_data[0]);
    } else {
      scale = 1.29246971E-26F;
      for (k = 0; k < *x_size; k++) {
        absxk = std::abs(x_data[k]);
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

// Function for MATLAB Function: '<S56>/trajFromWaypoints'
void MatlabControllerClass::polyInterpolationAx(real32_T num_of_splines,
  boolean_T cycle, const real32_T x_data[], const int32_T *x_size, real32_T
  b_data[], int32_T *b_size)
{
  real_T pp[36];
  real_T point_0[36];
  real32_T intermediate_size;
  int32_T bnd_left;
  int32_T g;
  int32_T l;
  int32_T b_tmp[6];
  real_T pp_data[6];
  real_T tmp_data[5];
  real32_T b[6];
  real32_T x_data_0[60];
  real32_T x;
  real32_T x_data_1[57];
  int32_T loop_ub;
  int32_T pp_size[2];
  int32_T tmp_size[2];
  int8_T b_x_idx_0;
  int32_T tmp;
  int8_T c_idx_0;
  real32_T intermediate_size_tmp;
  b_x_idx_0 = (int8_T)*x_size;
  c_idx_0 = (int8_T)*x_size;
  *b_size = b_x_idx_0;
  if (0 <= b_x_idx_0 - 1) {
    memset(&b_data[0], 0, b_x_idx_0 * sizeof(real32_T));
  }

  for (l = 0; l < 36; l++) {
    pp[l] = 1.0;
  }

  for (l = 0; l < 6; l++) {
    for (bnd_left = 0; bnd_left < 5; bnd_left++) {
      pp[(bnd_left + 6 * l) + 1] = 0.0;
    }
  }

  for (bnd_left = 0; bnd_left < 5; bnd_left++) {
    loop_ub = 6 - bnd_left;
    pp_size[0] = 1;
    pp_size[1] = loop_ub;
    for (l = 0; l < loop_ub; l++) {
      pp_data[l] = pp[6 * l + bnd_left];
    }

    polyder(pp_data, pp_size, tmp_data, tmp_size);
    loop_ub = tmp_size[1];
    for (l = 0; l < loop_ub; l++) {
      pp[(bnd_left + 6 * l) + 1] = tmp_data[l];
    }
  }

  memcpy(&point_0[0], &pp[0], 36U * sizeof(real_T));
  for (bnd_left = 0; bnd_left < 5; bnd_left++) {
    tmp = 5 - bnd_left;
    for (l = 0; l < tmp; l++) {
      point_0[bnd_left + 6 * l] = 0.0;
    }
  }

  intermediate_size_tmp = (num_of_splines - 1.0F) * 6.0F;
  if (!cycle) {
    bnd_left = 3;
    *b_size = c_idx_0;
    if (0 <= c_idx_0 - 1) {
      memset(&b_data[0], 0, c_idx_0 * sizeof(real32_T));
    }

    for (l = 0; l < 6; l++) {
      b_data[l] = ((real32_T)point_0[6 * l + 1] * x_data[1] + (real32_T)point_0
                   [6 * l] * x_data[0]) + (real32_T)point_0[6 * l + 2] * x_data
        [2];
    }

    if (4.0F + intermediate_size_tmp > ((4.0F + intermediate_size_tmp) + 3.0F) -
        1.0F) {
      tmp = 0;
      g = 0;
    } else {
      tmp = (int32_T)(4.0F + intermediate_size_tmp) - 1;
      g = (int32_T)(((4.0F + intermediate_size_tmp) + 3.0F) - 1.0F);
    }

    for (l = 0; l < 6; l++) {
      b_tmp[l] = (int32_T)((1.0F + intermediate_size_tmp) + (real32_T)l);
    }

    loop_ub = g - tmp;
    for (l = 0; l < loop_ub; l++) {
      x_data_0[l] = x_data[tmp + l];
    }

    for (l = 0; l < 6; l++) {
      b[l] = (((real32_T)pp[6 * l + 1] * x_data_0[1] + (real32_T)pp[6 * l] *
               x_data_0[0]) + (real32_T)pp[6 * l + 2] * x_data_0[2]) +
        b_data[b_tmp[l] - 1];
    }

    for (l = 0; l < 6; l++) {
      b_data[b_tmp[l] - 1] = b[l];
    }
  } else {
    bnd_left = 2;
    for (l = 0; l < 6; l++) {
      b_data[l] = (real32_T)point_0[6 * l] * x_data[0];
      b_tmp[l] = (int32_T)((intermediate_size_tmp + 1.0F) + (real32_T)l);
    }

    for (l = 0; l < 6; l++) {
      b[l] = (real32_T)pp[6 * l] * x_data[1] + b_data[b_tmp[l] - 1];
    }

    for (l = 0; l < 6; l++) {
      b_data[b_tmp[l] - 1] = b[l];
    }

    if ((intermediate_size_tmp + 2.0F) + 1.0F > (((intermediate_size_tmp + 2.0F)
          + 1.0F) + 5.0F) - 2.0F) {
      tmp = 0;
      g = 0;
    } else {
      tmp = (int32_T)((intermediate_size_tmp + 2.0F) + 1.0F) - 1;
      g = (int32_T)((((intermediate_size_tmp + 2.0F) + 1.0F) + 5.0F) - 2.0F);
    }

    loop_ub = g - tmp;
    for (l = 0; l < loop_ub; l++) {
      x_data_0[l] = x_data[tmp + l];
    }

    for (l = 0; l < 6; l++) {
      b_data[l] += (((real32_T)point_0[6 * l + 1] * x_data_0[0] + (real32_T)
                     point_0[6 * l + 2] * x_data_0[1]) + (real32_T)point_0[6 * l
                    + 3] * x_data_0[2]) + (real32_T)point_0[6 * l + 4] *
        x_data_0[3];
    }

    if ((intermediate_size_tmp + 2.0F) + 1.0F > (((intermediate_size_tmp + 2.0F)
          + 1.0F) + 5.0F) - 2.0F) {
      tmp = 0;
      g = 0;
    } else {
      tmp = (int32_T)((intermediate_size_tmp + 2.0F) + 1.0F) - 1;
      g = (int32_T)((((intermediate_size_tmp + 2.0F) + 1.0F) + 5.0F) - 2.0F);
    }

    loop_ub = g - tmp;
    for (l = 0; l < loop_ub; l++) {
      x_data_0[l] = x_data[tmp + l];
    }

    for (l = 0; l < 6; l++) {
      b[l] = b_data[(int32_T)((intermediate_size_tmp + 1.0F) + (real32_T)l) - 1]
        - ((((real32_T)pp[6 * l + 1] * x_data_0[0] + (real32_T)pp[6 * l + 2] *
             x_data_0[1]) + (real32_T)pp[6 * l + 3] * x_data_0[2]) + (real32_T)
           pp[6 * l + 4] * x_data_0[3]);
    }

    for (l = 0; l < 6; l++) {
      b_data[(int32_T)((intermediate_size_tmp + 1.0F) + (real32_T)l) - 1] = b[l];
    }
  }

  for (tmp = 0; tmp < (int32_T)(num_of_splines - 1.0F); tmp++) {
    intermediate_size_tmp = ((1.0F + (real32_T)tmp) - 1.0F) * 6.0F;
    intermediate_size = intermediate_size_tmp + (real32_T)bnd_left;
    for (l = 0; l < 6; l++) {
      b_tmp[l] = (int32_T)((intermediate_size_tmp + 1.0F) + (real32_T)l);
    }

    x = x_data[(int32_T)(intermediate_size + 1.0F) - 1];
    for (l = 0; l < 6; l++) {
      b[l] = (real32_T)pp[6 * l] * x + b_data[b_tmp[l] - 1];
    }

    for (l = 0; l < 6; l++) {
      b_data[b_tmp[l] - 1] = b[l];
    }

    for (l = 0; l < 6; l++) {
      b_tmp[l] = (int32_T)(((intermediate_size_tmp + 1.0F) + 6.0F) + (real32_T)l);
    }

    x = x_data[(int32_T)(intermediate_size + 2.0F) - 1];
    for (l = 0; l < 6; l++) {
      b[l] = (real32_T)point_0[6 * l] * x + b_data[b_tmp[l] - 1];
    }

    for (l = 0; l < 6; l++) {
      b_data[b_tmp[l] - 1] = b[l];
    }

    if (intermediate_size + 3.0F > ((intermediate_size + 3.0F) + 5.0F) - 2.0F) {
      g = 0;
      l = 0;
    } else {
      g = (int32_T)(intermediate_size + 3.0F) - 1;
      l = (int32_T)(((intermediate_size + 3.0F) + 5.0F) - 2.0F);
    }

    loop_ub = l - g;
    for (l = 0; l < loop_ub; l++) {
      x_data_1[l] = x_data[g + l];
    }

    for (l = 0; l < 6; l++) {
      b[l] = ((((real32_T)pp[6 * l + 1] * x_data_1[0] + (real32_T)pp[6 * l + 2] *
                x_data_1[1]) + (real32_T)pp[6 * l + 3] * x_data_1[2]) +
              (real32_T)pp[6 * l + 4] * x_data_1[3]) + b_data[(int32_T)
        ((intermediate_size_tmp + 1.0F) + (real32_T)l) - 1];
    }

    for (l = 0; l < 6; l++) {
      b_data[(int32_T)((intermediate_size_tmp + 1.0F) + (real32_T)l) - 1] = b[l];
    }

    if (intermediate_size + 3.0F > ((intermediate_size + 3.0F) + 5.0F) - 2.0F) {
      g = 0;
      l = 0;
    } else {
      g = (int32_T)(intermediate_size + 3.0F) - 1;
      l = (int32_T)(((intermediate_size + 3.0F) + 5.0F) - 2.0F);
    }

    loop_ub = l - g;
    for (l = 0; l < loop_ub; l++) {
      x_data_1[l] = x_data[g + l];
    }

    for (l = 0; l < 6; l++) {
      b[l] = b_data[(int32_T)(((intermediate_size_tmp + 1.0F) + 6.0F) +
        (real32_T)l) - 1] - ((((real32_T)point_0[6 * l + 1] * x_data_1[0] +
        (real32_T)point_0[6 * l + 2] * x_data_1[1]) + (real32_T)point_0[6 * l +
        3] * x_data_1[2]) + (real32_T)point_0[6 * l + 4] * x_data_1[3]);
    }

    for (l = 0; l < 6; l++) {
      b_data[(int32_T)(((intermediate_size_tmp + 1.0F) + 6.0F) + (real32_T)l) -
        1] = b[l];
    }
  }
}

// Function for MATLAB Function: '<S56>/trajFromWaypoints'
real32_T MatlabControllerClass::norm_c(const real32_T x[2])
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

  return scale * std::sqrt(y);
}

// Function for MATLAB Function: '<S56>/trajFromWaypoints'
void MatlabControllerClass::ladacLsqrIterate(real32_T A_tunableEnvironment_f1,
  boolean_T A_tunableEnvironment_f3, real32_T x_data[], int32_T *x_size,
  real32_T w_data[], int32_T *w_size, real32_T u_data[], int32_T *u_size,
  real32_T v_data[], int32_T *v_size, real32_T *Anorm, real32_T *alfa, real32_T *
  rhobar, real32_T *phibar)
{
  real32_T beta;
  real32_T scale;
  real32_T absxk;
  real32_T t;
  real32_T y_tmp_data[60];
  real32_T rhobar_0[2];
  real32_T tmp_data[60];
  int32_T i;
  int32_T tmp_size;
  polyInterpolationAx_f(A_tunableEnvironment_f1, A_tunableEnvironment_f3, v_data,
                        v_size, tmp_data, &tmp_size);
  *u_size = tmp_size;
  for (i = 0; i < tmp_size; i++) {
    u_data[i] = tmp_data[i] - *alfa * u_data[i];
  }

  beta = norm(u_data, u_size);
  if (beta > 0.0F) {
    scale = 1.0F / beta;
    tmp_size = *u_size;
    for (i = 0; i < tmp_size; i++) {
      y_tmp_data[i] = scale * u_data[i];
    }

    if (0 <= *u_size - 1) {
      memcpy(&u_data[0], &y_tmp_data[0], *u_size * sizeof(real32_T));
    }

    absxk = *Anorm;
    *Anorm = 0.0F;
    scale = 1.29246971E-26F;
    absxk = std::abs(absxk);
    if (absxk > 1.29246971E-26F) {
      t = 1.29246971E-26F / absxk;
      *Anorm = *Anorm * t * t + 1.0F;
      scale = absxk;
    } else {
      t = absxk / 1.29246971E-26F;
      *Anorm += t * t;
    }

    absxk = std::abs(*alfa);
    if (absxk > scale) {
      t = scale / absxk;
      *Anorm = *Anorm * t * t + 1.0F;
      scale = absxk;
    } else {
      t = absxk / scale;
      *Anorm += t * t;
    }

    if (beta > scale) {
      t = scale / beta;
      *Anorm = *Anorm * t * t + 1.0F;
      scale = beta;
    } else {
      t = beta / scale;
      *Anorm += t * t;
    }

    *Anorm = scale * std::sqrt(*Anorm);
    polyInterpolationAx(A_tunableEnvironment_f1, A_tunableEnvironment_f3,
                        y_tmp_data, u_size, tmp_data, &tmp_size);
    *v_size = tmp_size;
    for (i = 0; i < tmp_size; i++) {
      v_data[i] = tmp_data[i] - beta * v_data[i];
    }

    *alfa = norm(v_data, v_size);
    if (*alfa > 0.0F) {
      scale = 1.0F / *alfa;
      tmp_size = *v_size;
      for (i = 0; i < tmp_size; i++) {
        v_data[i] *= scale;
      }
    }

    rhobar_0[0] = *rhobar;
    rhobar_0[1] = 0.0F;
    absxk = norm_c(rhobar_0);
    *phibar *= *rhobar / absxk;
    rhobar_0[0] = absxk;
    rhobar_0[1] = beta;
    scale = norm_c(rhobar_0);
    absxk /= scale;
    beta /= scale;
    *rhobar = -absxk * *alfa;
    absxk *= *phibar;
    *phibar *= beta;
    absxk /= scale;
    beta = -(beta * *alfa) / scale;
    tmp_size = *x_size;
    for (i = 0; i < tmp_size; i++) {
      x_data[i] += absxk * w_data[i];
    }

    *w_size = *v_size;
    tmp_size = *v_size;
    for (i = 0; i < tmp_size; i++) {
      w_data[i] = beta * w_data[i] + v_data[i];
    }
  }
}

// Function for MATLAB Function: '<S56>/trajFromWaypoints'
void MatlabControllerClass::polyInterpolationb(const real32_T points_data[],
  const int32_T points_size[2], boolean_T cycle, real32_T b_data[], int32_T
  *b_size, real32_T *num_of_splines)
{
  real32_T points_new_data[11];
  int32_T size_A_mat;
  int32_T bnd_left;
  int32_T itm_row;
  int32_T points_new_size_idx_1;
  real32_T b_data_tmp;
  if (cycle) {
    points_new_size_idx_1 = points_size[1] + 1;
    size_A_mat = points_size[0] * points_size[1];
    if (0 <= size_A_mat - 1) {
      memcpy(&points_new_data[0], &points_data[0], size_A_mat * sizeof(real32_T));
    }

    points_new_data[size_A_mat] = points_data[0];
  } else {
    points_new_size_idx_1 = points_size[1];
    size_A_mat = points_size[0] * points_size[1] - 1;
    if (0 <= size_A_mat) {
      memcpy(&points_new_data[0], &points_data[0], (size_A_mat + 1) * sizeof
             (real32_T));
    }
  }

  *num_of_splines = (real32_T)points_new_size_idx_1 - 1.0F;
  size_A_mat = (points_new_size_idx_1 - 1) * 6;
  *b_size = size_A_mat;
  if (0 <= size_A_mat - 1) {
    memset(&b_data[0], 0, size_A_mat * sizeof(real32_T));
  }

  if (!cycle) {
    bnd_left = 3;
    b_data[0] = points_new_data[0];
    b_data[size_A_mat - 3] = points_new_data[points_new_size_idx_1 - 1];
    b_data[1] = points_new_data[1] - points_new_data[0];
    b_data[size_A_mat - 2] = points_new_data[points_new_size_idx_1 - 1] -
      points_new_data[points_new_size_idx_1 - 2];
  } else {
    bnd_left = 2;
    b_data[0] = points_new_data[0];
    b_data[1] = points_new_data[points_new_size_idx_1 - 1];
  }

  for (size_A_mat = 0; size_A_mat <= points_new_size_idx_1 - 3; size_A_mat++) {
    itm_row = 6 * size_A_mat + bnd_left;
    b_data_tmp = points_new_data[size_A_mat + 1];
    b_data[itm_row] = b_data_tmp;
    b_data[itm_row + 1] = b_data_tmp;
  }
}

// Function for MATLAB Function: '<S56>/trajFromWaypoints'
void MatlabControllerClass::ladacLsqrInit(real32_T A_tunableEnvironment_f1,
  boolean_T A_tunableEnvironment_f3, const real32_T b_data[], const int32_T
  *b_size, real32_T x_data[], int32_T *x_size, real32_T w_data[], int32_T
  *w_size, real32_T u_data[], int32_T *u_size, real32_T v_data[], int32_T
  *v_size, real32_T *alfa, real32_T *rhobar, real32_T *phibar)
{
  real32_T beta;
  real32_T z;
  int32_T loop_ub;
  int32_T i;
  *x_size = *b_size;
  if (0 <= *b_size - 1) {
    memset(&x_data[0], 0, *b_size * sizeof(real32_T));
  }

  *u_size = *b_size;
  if (0 <= *b_size - 1) {
    memcpy(&u_data[0], &b_data[0], *b_size * sizeof(real32_T));
  }

  *v_size = *b_size;
  if (0 <= *b_size - 1) {
    memset(&v_data[0], 0, *b_size * sizeof(real32_T));
  }

  *w_size = *b_size;
  if (0 <= *b_size - 1) {
    memset(&w_data[0], 0, *b_size * sizeof(real32_T));
  }

  *alfa = 0.0F;
  beta = norm(b_data, b_size);
  *rhobar = 0.0F;
  *phibar = 0.0F;
  if (beta > 0.0F) {
    z = 1.0F / beta;
    *u_size = *b_size;
    loop_ub = *b_size;
    for (i = 0; i < loop_ub; i++) {
      u_data[i] = z * b_data[i];
    }

    polyInterpolationAx(A_tunableEnvironment_f1, A_tunableEnvironment_f3, u_data,
                        u_size, v_data, v_size);
    *alfa = norm(v_data, v_size);
  }

  if (*alfa > 0.0F) {
    z = 1.0F / *alfa;
    loop_ub = *v_size;
    for (i = 0; i < loop_ub; i++) {
      v_data[i] *= z;
    }

    *w_size = *v_size;
    if (0 <= *v_size - 1) {
      memcpy(&w_data[0], &v_data[0], *v_size * sizeof(real32_T));
    }
  }

  if (*alfa * beta != 0.0F) {
    *rhobar = *alfa;
    *phibar = beta;
  }
}

// Function for MATLAB Function: '<S56>/trajFromWaypoints'
boolean_T MatlabControllerClass::trajValidateWaypoints(uint16_T num_wp)
{
  boolean_T is_valid;
  is_valid = false;
  if (num_wp >= 3) {
    is_valid = true;
  }

  return is_valid;
}

// Function for MATLAB Function: '<S102>/n ref norm'
void MatlabControllerClass::leanVectorNormDeriv2_c(const real32_T nn[3], const
  real32_T nn_dt[3], const real32_T nn_dt2[3], real_T n_dt2[3])
{
  real32_T xyz;
  real32_T a_tmp;
  real32_T n_dt2_tmp;
  real32_T xyz_tmp;
  real32_T xyz_tmp_0;
  real32_T xyz_tmp_1;
  real32_T a_tmp_tmp;
  real32_T a_tmp_tmp_0;
  real32_T a_tmp_tmp_1;
  real32_T n_dt2_tmp_0;
  real32_T n_dt2_tmp_1;
  real32_T n_dt2_tmp_2;
  real32_T n_dt2_tmp_3;
  real32_T n_dt2_tmp_4;
  real32_T n_dt2_tmp_5;
  xyz_tmp = nn[1] * nn[1];
  xyz_tmp_0 = nn[0] * nn[0];
  xyz_tmp_1 = nn[2] * nn[2];
  xyz = (xyz_tmp_0 + xyz_tmp) + xyz_tmp_1;
  a_tmp_tmp = 2.0F * nn[1] * nn_dt[1];
  a_tmp_tmp_0 = 2.0F * nn[0] * nn_dt[0];
  a_tmp_tmp_1 = 2.0F * nn[2] * nn_dt[2];
  a_tmp = (a_tmp_tmp_0 + a_tmp_tmp) + a_tmp_tmp_1;
  n_dt2_tmp = std::pow(xyz, 1.5F);
  n_dt2_tmp_0 = 2.0F * nn[1] * nn_dt2[1];
  n_dt2_tmp_1 = nn_dt[1] * nn_dt[1] * 2.0F;
  n_dt2_tmp_2 = 2.0F * nn[0] * nn_dt2[0];
  n_dt2_tmp_3 = nn_dt[0] * nn_dt[0] * 2.0F;
  n_dt2_tmp_4 = 2.0F * nn[2] * nn_dt2[2];
  n_dt2_tmp_5 = nn_dt[2] * nn_dt[2] * 2.0F;
  n_dt2[0] = (a_tmp * a_tmp * 3.0F / (4.0F * std::pow(xyz, 2.5F)) -
              (((((n_dt2_tmp_2 + n_dt2_tmp_3) + n_dt2_tmp_0) + n_dt2_tmp_1) +
                n_dt2_tmp_4) + n_dt2_tmp_5) / (2.0F * n_dt2_tmp)) * nn[0] +
    (nn_dt2[0] / std::sqrt(xyz) - a_tmp * nn_dt[0] / n_dt2_tmp);
  xyz = (xyz_tmp + xyz_tmp_0) + xyz_tmp_1;
  a_tmp = (a_tmp_tmp + a_tmp_tmp_0) + a_tmp_tmp_1;
  n_dt2_tmp = std::pow(xyz, 1.5F);
  n_dt2[1] = (a_tmp * a_tmp * 3.0F / (4.0F * std::pow(xyz, 2.5F)) -
              (((((n_dt2_tmp_0 + n_dt2_tmp_1) + n_dt2_tmp_2) + n_dt2_tmp_3) +
                n_dt2_tmp_4) + n_dt2_tmp_5) / (2.0F * n_dt2_tmp)) * nn[1] +
    (nn_dt2[1] / std::sqrt(xyz) - a_tmp * nn_dt[1] / n_dt2_tmp);
  xyz = (xyz_tmp_1 + xyz_tmp_0) + xyz_tmp;
  a_tmp = (a_tmp_tmp_1 + a_tmp_tmp_0) + a_tmp_tmp;
  n_dt2_tmp = std::pow(xyz, 1.5F);
  n_dt2[2] = (a_tmp * a_tmp * 3.0F / (4.0F * std::pow(xyz, 2.5F)) -
              (((((n_dt2_tmp_4 + n_dt2_tmp_5) + n_dt2_tmp_2) + n_dt2_tmp_3) +
                n_dt2_tmp_0) + n_dt2_tmp_1) / (2.0F * n_dt2_tmp)) * nn[2] +
    (nn_dt2[2] / std::sqrt(xyz) - a_tmp * nn_dt[2] / n_dt2_tmp);
}

// Function for MATLAB Function: '<S50>/MATLAB Function2'
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

// Function for MATLAB Function: '<S50>/MATLAB Function2'
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

// Function for MATLAB Function: '<S50>/MATLAB Function2'
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

// Function for MATLAB Function: '<S50>/MATLAB Function2'
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

// Function for MATLAB Function: '<S50>/MATLAB Function2'
void MatlabControllerClass::mldivide(const real32_T A_data[], const int32_T
  A_size[2], const real32_T B_0[8], real32_T Y_data[], int32_T *Y_size)
{
  if (A_size[1] == 0) {
    *Y_size = 0;
  } else {
    qrsolve(A_data, A_size, B_0, Y_data, Y_size);
  }
}

// Function for MATLAB Function: '<S50>/MATLAB Function2'
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

// Function for MATLAB Function: '<S50>/MATLAB Function2'
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

real_T rt_urand_Upu32_Yd_f_pw(uint32_T *u)
{
  uint32_T lo;
  uint32_T hi;

  // Uniform random number generator (random number between 0 and 1)

  // #define IA      16807                      magic multiplier = 7^5
  // #define IM      2147483647                 modulus = 2^31-1
  // #define IQ      127773                     IM div IA
  // #define IR      2836                       IM modulo IA
  // #define S       4.656612875245797e-10      reciprocal of 2^31-1
  // test = IA * (seed % IQ) - IR * (seed/IQ)
  // seed = test < 0 ? (test + IM) : test
  // return (seed*S)

  lo = *u % 127773U * 16807U;
  hi = *u / 127773U * 2836U;
  if (lo < hi) {
    *u = 2147483647U - (hi - lo);
  } else {
    *u = lo - hi;
  }

  return (real_T)*u * 4.6566128752457969E-10;
}

real_T rt_nrand_Upu32_Yd_f_pw(uint32_T *u)
{
  real_T y;
  real_T sr;
  real_T si;

  // Normal (Gaussian) random number generator
  do {
    sr = 2.0 * rt_urand_Upu32_Yd_f_pw(u) - 1.0;
    si = 2.0 * rt_urand_Upu32_Yd_f_pw(u) - 1.0;
    si = sr * sr + si * si;
  } while (si > 1.0);

  y = std::sqrt(-2.0 * std::log(si) / si) * sr;
  return y;
}

// Model step function
void MatlabControllerClass::step()
{
  boolean_T varargin_1[10];
  int32_T k;
  real32_T q0_q3;
  real32_T q1_q2;
  real32_T q1_q3;
  real32_T G_omega[16];
  real32_T umax[4];
  dtoSgl_trajectoryStructBus traj;
  uint16_T num_wp;
  real32_T b_data[60];
  real32_T x_data[60];
  real32_T u_data[60];
  real32_T v_data[60];
  real32_T b_w_data[24];
  real32_T b_u_data[24];
  real32_T b_v_data[24];
  int32_T db;
  real32_T dist_remaining;
  real32_T dist_available;
  real32_T rtb_Add_a[4];
  real32_T rtb_G2[16];
  real32_T rtb_G1[16];
  real32_T rtb_y_bn[16];
  real32_T rtb_n_b[3];
  real32_T rtb_Sqrt;
  real32_T rtb_y_i[9];
  real32_T rtb_Sum2_e[3];
  real32_T rtb_Gain3_b;
  real32_T rtb_n_b_dt2[3];
  real32_T rtb_n_b_dt_p[3];
  real32_T rtb_n_b_c[3];
  real32_T rtb_n_dt2_i[3];
  real32_T rtb_n_dt_o[3];
  real32_T rtb_n[3];
  real32_T rtb_n_b_dt[3];
  real32_T rtb_n_dt2[3];
  real32_T rtb_n_dt[3];
  real32_T rtb_y_dt_cs[3];
  real32_T rtb_omega2_f[3];
  real32_T rtb_s_g_dt2_ref[3];
  real32_T rtb_Add1[3];
  real32_T rtb_y_c[3];
  real32_T rtb_uvwxb[120];
  real32_T rtb_Sum2_p[9];
  real32_T rtb_y_dt_g[3];
  real32_T rtb_Switch[4];
  real32_T rtb_Gain;
  real32_T rtb_Gain5;
  real32_T rtb_uDLookupTable;
  real32_T rtb_distance;
  real32_T rtb_W[4];
  boolean_T rtb_Compare;
  int32_T i;
  real32_T rtb_n_dt_0[4];
  real_T tmp[3];
  real32_T tmp_data[60];
  real32_T tmp_data_0[10];
  real32_T x_data_0[24];
  real32_T dist_remaining_0[9];
  real32_T tmp_0[9];
  real32_T tmp_1[9];
  real32_T tmp_2[9];
  real32_T tmp_3[3];
  real32_T rtb_y_l[4];
  int32_T loop_ub;
  int32_T b_v_size;
  int32_T tmp_size[2];
  real32_T state_vec[6];
  uint16_T varargin_1_idx_1;
  real_T y;
  boolean_T tmp_4;
  int32_T tmp_5;
  real32_T tmp_6;

  // Outputs for Iterator SubSystem: '<S13>/Trajectory from Waypoints' incorporates:
  //   ForIterator: '<S56>/For Iterator'

  // Memory: '<S56>/uvwxbMemory'
  memcpy(&rtb_uvwxb[0], &rtDW.uvwxbMemory_PreviousInput[0], 120U * sizeof
         (real32_T));

  // Memory: '<S56>/AnormAlfaRhoPhiMemory'
  rtb_Switch[0] = rtDW.AnormAlfaRhoPhiMemory_PreviousI[0];
  rtb_Switch[1] = rtDW.AnormAlfaRhoPhiMemory_PreviousI[1];
  rtb_Switch[2] = rtDW.AnormAlfaRhoPhiMemory_PreviousI[2];
  rtb_Switch[3] = rtDW.AnormAlfaRhoPhiMemory_PreviousI[3];

  // RelationalOperator: '<S64>/Compare' incorporates:
  //   Constant: '<S64>/Constant'
  //   Inport: '<Root>/cmd'

  rtb_Compare = (rtU.cmd.mission_change > 0);

  // Memory: '<S56>/StateVecMemory '
  for (i = 0; i < 6; i++) {
    state_vec[i] = rtDW.StateVecMemory_PreviousInput[i];
  }

  // End of Memory: '<S56>/StateVecMemory '

  // MATLAB Function: '<S56>/trajFromWaypoints' incorporates:
  //   Constant: '<S13>/Constant'
  //   Constant: '<S56>/Constant1'
  //   Inport: '<Root>/cmd'
  //   Memory: '<S56>/AnormAlfaRhoPhiMemory'
  //   Memory: '<S56>/TrajMemory'
  //   Memory: '<S56>/uvwxbMemory'
  //   RelationalOperator: '<S62>/FixPt Relational Operator'
  //   UnitDelay: '<S62>/Delay Input1'
  //
  //  Block description for '<S62>/Delay Input1':
  //
  //   Store in Global RAM

  traj = rtDW.TrajMemory_PreviousInput;
  rtb_Sqrt = state_vec[0];
  rtb_uDLookupTable = state_vec[1];
  rtb_Gain5 = state_vec[2];
  rtb_Gain = state_vec[3];
  dist_remaining = state_vec[4];
  rtb_distance = state_vec[5];
  rtb_Gain3_b = rt_roundf(rtDW.TrajMemory_PreviousInput.num_sections_max);
  if (rtb_Gain3_b < 65536.0F) {
    if (rtb_Gain3_b >= 0.0F) {
      varargin_1_idx_1 = (uint16_T)rtb_Gain3_b;
    } else {
      varargin_1_idx_1 = 0U;
    }
  } else {
    varargin_1_idx_1 = MAX_uint16_T;
  }

  num_wp = rtU.cmd.num_waypoints;
  if (rtU.cmd.num_waypoints > varargin_1_idx_1) {
    num_wp = varargin_1_idx_1;
  }

  if (num_wp > 10) {
    num_wp = 10U;
  }

  if (((int32_T)rtb_Compare > (int32_T)rtDW.DelayInput1_DSTATE) && (state_vec[0]
       == 0.0F) && (state_vec[2] == 0.0F)) {
    dist_remaining = 0.0F;
    traj = rtConstP.pooled3;
    if (trajValidateWaypoints(num_wp)) {
      rtb_Gain5 = 1.0F;
      rtb_Sqrt = 0.0F;
    }
  } else if ((state_vec[2] >= 1.0F) && (state_vec[2] <= 3.0F)) {
    if (state_vec[0] == 0.0F) {
      if (1 > num_wp) {
        loop_ub = 0;
      } else {
        loop_ub = num_wp;
      }

      i = (int32_T)state_vec[2];
      tmp_size[0] = 1;
      tmp_size[1] = loop_ub;
      for (k = 0; k < loop_ub; k++) {
        tmp_data_0[k] = rtU.cmd.waypoints[((k << 2) + i) - 1];
      }

      polyInterpolationb(tmp_data_0, tmp_size, true, b_data, &db, &rtb_Gain);
      ladacLsqrInit(rtb_Gain, true, b_data, &db, x_data, &k, tmp_data, &i,
                    u_data, &b_v_size, v_data, &loop_ub, &rtb_Switch[1],
                    &rtb_Switch[2], &rtb_Switch[3]);
      rtb_Sqrt = 1.0F;
      rtb_uDLookupTable = (real32_T)db;
      if (0 <= db - 1) {
        memcpy(&rtb_uvwxb[0], &b_data[0], db * sizeof(real32_T));
      }

      if (0 <= b_v_size - 1) {
        memcpy(&rtb_uvwxb[24], &u_data[0], b_v_size * sizeof(real32_T));
      }

      if (0 <= loop_ub - 1) {
        memcpy(&rtb_uvwxb[48], &v_data[0], loop_ub * sizeof(real32_T));
      }

      if (0 <= i - 1) {
        memcpy(&rtb_uvwxb[72], &tmp_data[0], i * sizeof(real32_T));
      }

      if (0 <= k - 1) {
        memcpy(&rtb_uvwxb[96], &x_data[0], k * sizeof(real32_T));
      }

      rtb_Switch[0] = 0.0F;
      polyInterpolationAx_f(rtb_Gain, true, x_data, &k, tmp_data, &db);
      for (i = 0; i < db; i++) {
        x_data[i] = tmp_data[i] - b_data[i];
      }

      rtb_distance = norm(x_data, &db);
    } else if (state_vec[0] < 1000.0F) {
      if (1.0F > state_vec[1]) {
        loop_ub = 0;
        i = 0;
      } else {
        loop_ub = (int32_T)state_vec[1];
        i = (int32_T)state_vec[1];
      }

      if (0 <= loop_ub - 1) {
        memcpy(&b_data[0], &rtDW.uvwxbMemory_PreviousInput[0], loop_ub * sizeof
               (real32_T));
      }

      k = i;
      if (0 <= i - 1) {
        memcpy(&x_data_0[0], &rtDW.uvwxbMemory_PreviousInput[96], i * sizeof
               (real32_T));
      }

      if (1.0F > state_vec[1]) {
        i = 0;
      } else {
        i = (int32_T)state_vec[1];
      }

      db = i;
      if (0 <= i - 1) {
        memcpy(&b_w_data[0], &rtDW.uvwxbMemory_PreviousInput[72], i * sizeof
               (real32_T));
      }

      if (1.0F > state_vec[1]) {
        i = 0;
      } else {
        i = (int32_T)state_vec[1];
      }

      if (0 <= i - 1) {
        memcpy(&b_u_data[0], &rtDW.uvwxbMemory_PreviousInput[24], i * sizeof
               (real32_T));
      }

      if (1.0F > state_vec[1]) {
        i = 0;
      } else {
        i = (int32_T)state_vec[1];
      }

      b_v_size = i;
      if (0 <= i - 1) {
        memcpy(&b_v_data[0], &rtDW.uvwxbMemory_PreviousInput[48], i * sizeof
               (real32_T));
      }

      rtb_Switch[0] = rtDW.AnormAlfaRhoPhiMemory_PreviousI[0];
      rtb_Switch[1] = rtDW.AnormAlfaRhoPhiMemory_PreviousI[1];
      rtb_Switch[2] = rtDW.AnormAlfaRhoPhiMemory_PreviousI[2];
      rtb_Switch[3] = rtDW.AnormAlfaRhoPhiMemory_PreviousI[3];
      ladacLsqrIterate(state_vec[3], true, x_data_0, &k, b_w_data, &db, b_u_data,
                       &i, b_v_data, &b_v_size, &rtb_Switch[0], &rtb_Switch[1],
                       &rtb_Switch[2], &rtb_Switch[3]);
      if (0 <= loop_ub - 1) {
        memcpy(&rtb_uvwxb[0], &b_data[0], loop_ub * sizeof(real32_T));
      }

      if (0 <= i - 1) {
        memcpy(&rtb_uvwxb[24], &b_u_data[0], i * sizeof(real32_T));
      }

      if (0 <= b_v_size - 1) {
        memcpy(&rtb_uvwxb[48], &b_v_data[0], b_v_size * sizeof(real32_T));
      }

      if (0 <= db - 1) {
        memcpy(&rtb_uvwxb[72], &b_w_data[0], db * sizeof(real32_T));
      }

      if (0 <= k - 1) {
        memcpy(&rtb_uvwxb[96], &x_data_0[0], k * sizeof(real32_T));
      }

      rtb_Sqrt = state_vec[0] + 1.0F;
      polyInterpolationAx_f(state_vec[3], true, x_data_0, &k, tmp_data, &db);
      for (i = 0; i < db; i++) {
        x_data_0[i] = tmp_data[i] - b_data[i];
      }

      rtb_distance = norm(x_data_0, &db);
      if ((rtb_distance < 0.001) || (state_vec[5] < rtb_distance)) {
        rtb_Sqrt = 1000.0F;
      }
    } else {
      if (1.0F > state_vec[1]) {
        i = 0;
      } else {
        i = (int32_T)state_vec[1];
      }

      loop_ub = i - 1;
      if (0 <= loop_ub) {
        memcpy(&x_data[0], &rtDW.uvwxbMemory_PreviousInput[96], (loop_ub + 1) *
               sizeof(real32_T));
      }

      traj.num_sections_set = state_vec[3];
      traj.is_repeated_course = true;
      for (k = 0; k < (int32_T)rtb_Gain; k++) {
        rtb_Sqrt = ((1.0F + (real32_T)k) - 1.0F) * 6.0F + 1.0F;
        if (rtb_Gain5 == 1.0F) {
          if (rtb_Sqrt > rtb_Sqrt + 5.0F) {
            db = 1;
            i = 0;
          } else {
            db = (int32_T)rtb_Sqrt;
            i = (int32_T)(rtb_Sqrt + 5.0F);
          }

          loop_ub = i - db;
          for (i = 0; i <= loop_ub; i++) {
            x_data_0[i] = x_data[(db + i) - 1];
          }

          for (i = 0; i < 6; i++) {
            traj.sections[(int32_T)(1.0F + (real32_T)k) - 1].pos_x[i] =
              x_data_0[i];
          }
        } else if (rtb_Gain5 == 2.0F) {
          if (rtb_Sqrt > rtb_Sqrt + 5.0F) {
            db = 1;
            i = 0;
          } else {
            db = (int32_T)rtb_Sqrt;
            i = (int32_T)(rtb_Sqrt + 5.0F);
          }

          loop_ub = i - db;
          for (i = 0; i <= loop_ub; i++) {
            x_data_0[i] = x_data[(db + i) - 1];
          }

          for (i = 0; i < 6; i++) {
            traj.sections[(int32_T)(1.0F + (real32_T)k) - 1].pos_y[i] =
              x_data_0[i];
          }
        } else {
          if (rtb_Gain5 == 3.0F) {
            if (rtb_Sqrt > rtb_Sqrt + 5.0F) {
              db = 1;
              i = 0;
            } else {
              db = (int32_T)rtb_Sqrt;
              i = (int32_T)(rtb_Sqrt + 5.0F);
            }

            loop_ub = i - db;
            for (i = 0; i <= loop_ub; i++) {
              x_data_0[i] = x_data[(db + i) - 1];
            }

            for (i = 0; i < 6; i++) {
              traj.sections[(int32_T)(1.0F + (real32_T)k) - 1].pos_z[i] =
                x_data_0[i];
            }
          }
        }
      }

      rtb_Sqrt = 0.0F;
      rtb_Gain5 = state_vec[2] + 1.0F;
    }
  } else {
    if (state_vec[2] == 4.0F) {
      trajSetArcLength(&traj);
      dist_remaining = 1.0F;
      rtb_Gain5 = 0.0F;
      rtb_Sqrt = 0.0F;
    }
  }

  state_vec[0] = rtb_Sqrt;
  state_vec[1] = rtb_uDLookupTable;
  state_vec[2] = rtb_Gain5;
  state_vec[3] = rtb_Gain;
  state_vec[4] = dist_remaining;
  state_vec[5] = rtb_distance;

  // Update for Memory: '<S56>/uvwxbMemory'
  memcpy(&rtDW.uvwxbMemory_PreviousInput[0], &rtb_uvwxb[0], 120U * sizeof
         (real32_T));

  // Update for Memory: '<S56>/AnormAlfaRhoPhiMemory'
  rtDW.AnormAlfaRhoPhiMemory_PreviousI[0] = rtb_Switch[0];
  rtDW.AnormAlfaRhoPhiMemory_PreviousI[1] = rtb_Switch[1];
  rtDW.AnormAlfaRhoPhiMemory_PreviousI[2] = rtb_Switch[2];
  rtDW.AnormAlfaRhoPhiMemory_PreviousI[3] = rtb_Switch[3];

  // Update for UnitDelay: '<S62>/Delay Input1'
  //
  //  Block description for '<S62>/Delay Input1':
  //
  //   Store in Global RAM

  rtDW.DelayInput1_DSTATE = rtb_Compare;

  // Update for Memory: '<S56>/StateVecMemory '
  for (i = 0; i < 6; i++) {
    rtDW.StateVecMemory_PreviousInput[i] = state_vec[i];
  }

  // End of Update for Memory: '<S56>/StateVecMemory '

  // Update for Memory: '<S56>/TrajMemory' incorporates:
  //   MATLAB Function: '<S56>/trajFromWaypoints'

  rtDW.TrajMemory_PreviousInput = traj;

  // End of Outputs for SubSystem: '<S13>/Trajectory from Waypoints'

  // DiscreteIntegrator: '<S78>/Discrete-Time Integrator y' incorporates:
  //   Inport: '<Root>/measure'

  if (rtDW.DiscreteTimeIntegratory_IC_LOAD != 0) {
    rtDW.DiscreteTimeIntegratory_DSTATE[0] = rtU.measure.s_Kg[0];
    rtDW.DiscreteTimeIntegratory_DSTATE[1] = rtU.measure.s_Kg[1];
    rtDW.DiscreteTimeIntegratory_DSTATE[2] = rtU.measure.s_Kg[2];
  }

  // DiscreteIntegrator: '<S77>/Discrete-Time Integrator y' incorporates:
  //   Inport: '<Root>/measure'

  if (rtDW.DiscreteTimeIntegratory_IC_LO_n != 0) {
    rtDW.DiscreteTimeIntegratory_DSTAT_a[0] = rtU.measure.V_Kg[0];
    rtDW.DiscreteTimeIntegratory_DSTAT_a[1] = rtU.measure.V_Kg[1];
    rtDW.DiscreteTimeIntegratory_DSTAT_a[2] = rtU.measure.V_Kg[2];
  }

  // Lookup_n-D: '<Root>/1-D Lookup Table' incorporates:
  //   Inport: '<Root>/cmd'

  rtb_uDLookupTable = look1_iflf_binlx(rtU.cmd.RC_pwm[8], rtConstP.pooled10,
    rtConstP.uDLookupTable_tableData_g, 1U);

  // DiscreteIntegrator: '<S7>/Discrete-Time Integrator'
  if ((rtb_uDLookupTable > 0.0F) && (rtDW.DiscreteTimeIntegrator_PrevRese <= 0))
  {
    rtDW.DiscreteTimeIntegrator_DSTATE = 0.0F;
  }

  // MATLAB Function: '<S7>/interpHold' incorporates:
  //   Constant: '<S7>/Constant1'
  //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator'

  for (i = 0; i < 10; i++) {
    varargin_1[i] = (rtConstP.Constant1_Value[i] >
                     rtDW.DiscreteTimeIntegrator_DSTATE);
  }

  i = -1;
  rtb_Compare = varargin_1[0];
  for (k = 0; k < 9; k++) {
    tmp_4 = varargin_1[k + 1];
    if ((int32_T)rtb_Compare < (int32_T)tmp_4) {
      rtb_Compare = tmp_4;
      i = k;
    }
  }

  // Switch: '<S7>/Switch' incorporates:
  //   Constant: '<S7>/Constant2'
  //   Inport: '<Root>/cmd'
  //   MATLAB Function: '<S7>/interpHold'

  if (rtb_uDLookupTable > 0.5F) {
    // MATLAB Function: '<S7>/interpHold'
    if (1.0 > ((real_T)i + 2.0) - 1.0) {
      y = 1.0;
    } else {
      y = ((real_T)i + 2.0) - 1.0;
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

  // Outputs for Enabled SubSystem: '<S2>/NDI position controller for copters reference model' incorporates:
  //   EnablePort: '<S17>/Enable'

  if (!rtDW.NDIpositioncontrollerforcopters) {
    // InitializeConditions for DiscreteIntegrator: '<S112>/Discrete-Time Integrator' 
    rtDW.DiscreteTimeIntegrator_IC_LOADI = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S17>/Discrete-Time Integrator' 
    rtDW.DiscreteTimeIntegrator_IC_LOA_o = 1U;
    rtDW.NDIpositioncontrollerforcopters = true;
  }

  // DiscreteIntegrator: '<S112>/Discrete-Time Integrator' incorporates:
  //   DiscreteIntegrator: '<S77>/Discrete-Time Integrator y'

  if (rtDW.DiscreteTimeIntegrator_IC_LOADI != 0) {
    rtDW.DiscreteTimeIntegrator_DSTATE_l[0] =
      rtDW.DiscreteTimeIntegratory_DSTAT_a[0];
    rtDW.DiscreteTimeIntegrator_DSTATE_l[1] =
      rtDW.DiscreteTimeIntegratory_DSTAT_a[1];
    rtDW.DiscreteTimeIntegrator_DSTATE_l[2] =
      rtDW.DiscreteTimeIntegratory_DSTAT_a[2];
  }

  // Gain: '<S17>/Gain3' incorporates:
  //   Gain: '<Root>/Gain'
  //   MATLAB Function: '<S17>/MATLAB Function1'

  rtb_Gain3_b = std::sqrt(rtb_Switch[0] * rtb_Switch[0] + -rtb_Switch[1] *
    -rtb_Switch[1]) * 15.0265598F;

  // Gain: '<S112>/1//T' incorporates:
  //   DiscreteIntegrator: '<S112>/Discrete-Time Integrator'
  //   Gain: '<Root>/Gain'
  //   Gain: '<Root>/Gain5'
  //   Lookup_n-D: '<S17>/1-D Lookup Table'
  //   Product: '<S17>/Product'
  //   Sum: '<S112>/Sum2'

  rtb_Sum2_e[0] = (-rtb_Switch[1] * rtb_Gain3_b -
                   rtDW.DiscreteTimeIntegrator_DSTATE_l[0]) * 1.17755473F;
  rtb_Sum2_e[1] = (rtb_Switch[0] * rtb_Gain3_b -
                   rtDW.DiscreteTimeIntegrator_DSTATE_l[1]) * 1.17755473F;
  rtb_Sum2_e[2] = (look1_iflf_binlx(-rtb_Switch[3],
    rtConstP.uDLookupTable_bp01Data, rtConstP.uDLookupTable_tableData, 2U) -
                   rtDW.DiscreteTimeIntegrator_DSTATE_l[2]) * 1.17755473F;

  // DiscreteIntegrator: '<S112>/Discrete-Time Integrator'
  rtb_y_c[0] = rtDW.DiscreteTimeIntegrator_DSTATE_l[0];

  // Gain: '<S17>/Gain' incorporates:
  //   DiscreteIntegrator: '<S112>/Discrete-Time Integrator'

  rtDW.s_g_ref_dt[0] = rtDW.DiscreteTimeIntegrator_DSTATE_l[0];

  // Saturate: '<S112>/Saturation'
  if (rtb_Sum2_e[0] > 27.6478081F) {
    q1_q3 = 27.6478081F;
  } else if (rtb_Sum2_e[0] < -27.6478081F) {
    q1_q3 = -27.6478081F;
  } else {
    q1_q3 = rtb_Sum2_e[0];
  }

  // Gain: '<S17>/Gain1'
  rtDW.s_g_ref_dt2[0] = q1_q3;

  // Saturate: '<S112>/Saturation'
  rtb_y_dt_g[0] = q1_q3;

  // DiscreteIntegrator: '<S112>/Discrete-Time Integrator'
  rtb_y_c[1] = rtDW.DiscreteTimeIntegrator_DSTATE_l[1];

  // Gain: '<S17>/Gain' incorporates:
  //   DiscreteIntegrator: '<S112>/Discrete-Time Integrator'

  rtDW.s_g_ref_dt[1] = rtDW.DiscreteTimeIntegrator_DSTATE_l[1];

  // Saturate: '<S112>/Saturation'
  if (rtb_Sum2_e[1] > 27.6478081F) {
    q1_q3 = 27.6478081F;
  } else if (rtb_Sum2_e[1] < -27.6478081F) {
    q1_q3 = -27.6478081F;
  } else {
    q1_q3 = rtb_Sum2_e[1];
  }

  // Gain: '<S17>/Gain1'
  rtDW.s_g_ref_dt2[1] = q1_q3;

  // Saturate: '<S112>/Saturation'
  rtb_y_dt_g[1] = q1_q3;

  // DiscreteIntegrator: '<S112>/Discrete-Time Integrator'
  rtb_y_c[2] = rtDW.DiscreteTimeIntegrator_DSTATE_l[2];

  // Gain: '<S17>/Gain' incorporates:
  //   DiscreteIntegrator: '<S112>/Discrete-Time Integrator'

  rtDW.s_g_ref_dt[2] = rtDW.DiscreteTimeIntegrator_DSTATE_l[2];

  // Saturate: '<S112>/Saturation'
  if (rtb_Sum2_e[2] > 7.49204397F) {
    q1_q3 = 7.49204397F;
  } else if (rtb_Sum2_e[2] < -20.8920841F) {
    q1_q3 = -20.8920841F;
  } else {
    q1_q3 = rtb_Sum2_e[2];
  }

  // Gain: '<S17>/Gain1'
  rtDW.s_g_ref_dt2[2] = q1_q3;

  // DiscreteIntegrator: '<S17>/Discrete-Time Integrator' incorporates:
  //   DiscreteIntegrator: '<S78>/Discrete-Time Integrator y'

  if (rtDW.DiscreteTimeIntegrator_IC_LOA_o != 0) {
    rtDW.DiscreteTimeIntegrator_DSTAT_em[0] =
      rtDW.DiscreteTimeIntegratory_DSTATE[0];
    rtDW.DiscreteTimeIntegrator_DSTAT_em[1] =
      rtDW.DiscreteTimeIntegratory_DSTATE[1];
    rtDW.DiscreteTimeIntegrator_DSTAT_em[2] =
      rtDW.DiscreteTimeIntegratory_DSTATE[2];
  }

  // Update for DiscreteIntegrator: '<S112>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_IC_LOADI = 0U;

  // Update for DiscreteIntegrator: '<S17>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_IC_LOA_o = 0U;

  // DiscreteIntegrator: '<S17>/Discrete-Time Integrator'
  rtDW.s_g_ref[0] = rtDW.DiscreteTimeIntegrator_DSTAT_em[0];

  // Update for DiscreteIntegrator: '<S112>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_DSTATE_l[0] += 0.0025F * rtb_y_dt_g[0];

  // Update for DiscreteIntegrator: '<S17>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_DSTAT_em[0] += 0.0025F * rtb_y_c[0];

  // DiscreteIntegrator: '<S17>/Discrete-Time Integrator'
  rtDW.s_g_ref[1] = rtDW.DiscreteTimeIntegrator_DSTAT_em[1];

  // Update for DiscreteIntegrator: '<S112>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_DSTATE_l[1] += 0.0025F * rtb_y_dt_g[1];

  // Update for DiscreteIntegrator: '<S17>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_DSTAT_em[1] += 0.0025F * rtb_y_c[1];

  // DiscreteIntegrator: '<S17>/Discrete-Time Integrator'
  rtDW.s_g_ref[2] = rtDW.DiscreteTimeIntegrator_DSTAT_em[2];

  // Update for DiscreteIntegrator: '<S112>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_DSTATE_l[2] += 0.0025F * q1_q3;

  // Update for DiscreteIntegrator: '<S17>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_DSTAT_em[2] += 0.0025F * rtb_y_c[2];

  // End of Outputs for SubSystem: '<S2>/NDI position controller for copters reference model' 

  // Outputs for Enabled SubSystem: '<S2>/NDI position controller for copters with reference input' incorporates:
  //   EnablePort: '<S18>/Enable'

  // DiscreteIntegrator: '<S115>/Discrete-Time Integrator y'
  if (rtDW.DiscreteTimeIntegratory_IC_L_b2 != 0) {
    rtDW.DiscreteTimeIntegratory_DSTAT_l[0] = rtDW.s_g_ref[0];
    rtDW.DiscreteTimeIntegratory_DSTAT_l[3] = rtDW.s_g_ref_dt[0];
    rtDW.DiscreteTimeIntegratory_DSTAT_l[6] = rtDW.s_g_ref_dt2[0];
    rtDW.DiscreteTimeIntegratory_DSTAT_l[1] = rtDW.s_g_ref[1];
    rtDW.DiscreteTimeIntegratory_DSTAT_l[4] = rtDW.s_g_ref_dt[1];
    rtDW.DiscreteTimeIntegratory_DSTAT_l[7] = rtDW.s_g_ref_dt2[1];
    rtDW.DiscreteTimeIntegratory_DSTAT_l[2] = rtDW.s_g_ref[2];
    rtDW.DiscreteTimeIntegratory_DSTAT_l[5] = rtDW.s_g_ref_dt[2];
    rtDW.DiscreteTimeIntegratory_DSTAT_l[8] = rtDW.s_g_ref_dt2[2];
  }

  // Sum: '<S115>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S115>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S115>/Discrete-Time Integrator y_dt'
  //   Gain: '<S115>/2*d//omega'
  //   Sum: '<S115>/Sum3'

  rtb_Sum2_p[0] = rtDW.s_g_ref[0] - (0.208835155F *
    rtDW.DiscreteTimeIntegratory_dt_DS_e[0] +
    rtDW.DiscreteTimeIntegratory_DSTAT_l[0]);
  rtb_Sum2_p[3] = rtDW.s_g_ref_dt[0] - (0.208835155F *
    rtDW.DiscreteTimeIntegratory_dt_DS_e[3] +
    rtDW.DiscreteTimeIntegratory_DSTAT_l[3]);
  rtb_Sum2_p[6] = rtDW.s_g_ref_dt2[0] - (0.208835155F *
    rtDW.DiscreteTimeIntegratory_dt_DS_e[6] +
    rtDW.DiscreteTimeIntegratory_DSTAT_l[6]);

  // Sum: '<S114>/Add' incorporates:
  //   DiscreteIntegrator: '<S115>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S75>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S77>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S78>/Discrete-Time Integrator y'

  rtDW.Add[0] = rtDW.DiscreteTimeIntegratory_DSTAT_l[0] -
    rtDW.DiscreteTimeIntegratory_DSTATE[0];
  rtDW.Add[3] = rtDW.DiscreteTimeIntegratory_DSTAT_l[3] -
    rtDW.DiscreteTimeIntegratory_DSTAT_a[0];
  rtDW.Add[6] = rtDW.DiscreteTimeIntegratory_DSTAT_l[6] -
    rtDW.DiscreteTimeIntegratory_DSTA_al[0];

  // Sum: '<S115>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S115>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S115>/Discrete-Time Integrator y_dt'
  //   Gain: '<S115>/2*d//omega'
  //   Sum: '<S115>/Sum3'

  rtb_Sum2_p[1] = rtDW.s_g_ref[1] - (0.208835155F *
    rtDW.DiscreteTimeIntegratory_dt_DS_e[1] +
    rtDW.DiscreteTimeIntegratory_DSTAT_l[1]);
  rtb_Sum2_p[4] = rtDW.s_g_ref_dt[1] - (0.208835155F *
    rtDW.DiscreteTimeIntegratory_dt_DS_e[4] +
    rtDW.DiscreteTimeIntegratory_DSTAT_l[4]);
  rtb_Sum2_p[7] = rtDW.s_g_ref_dt2[1] - (0.208835155F *
    rtDW.DiscreteTimeIntegratory_dt_DS_e[7] +
    rtDW.DiscreteTimeIntegratory_DSTAT_l[7]);

  // Sum: '<S114>/Add' incorporates:
  //   DiscreteIntegrator: '<S115>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S75>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S77>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S78>/Discrete-Time Integrator y'

  rtDW.Add[1] = rtDW.DiscreteTimeIntegratory_DSTAT_l[1] -
    rtDW.DiscreteTimeIntegratory_DSTATE[1];
  rtDW.Add[4] = rtDW.DiscreteTimeIntegratory_DSTAT_l[4] -
    rtDW.DiscreteTimeIntegratory_DSTAT_a[1];
  rtDW.Add[7] = rtDW.DiscreteTimeIntegratory_DSTAT_l[7] -
    rtDW.DiscreteTimeIntegratory_DSTA_al[1];

  // Sum: '<S115>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S115>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S115>/Discrete-Time Integrator y_dt'
  //   Gain: '<S115>/2*d//omega'
  //   Sum: '<S115>/Sum3'

  rtb_Sum2_p[2] = rtDW.s_g_ref[2] - (0.208835155F *
    rtDW.DiscreteTimeIntegratory_dt_DS_e[2] +
    rtDW.DiscreteTimeIntegratory_DSTAT_l[2]);
  rtb_Sum2_p[5] = rtDW.s_g_ref_dt[2] - (0.208835155F *
    rtDW.DiscreteTimeIntegratory_dt_DS_e[5] +
    rtDW.DiscreteTimeIntegratory_DSTAT_l[5]);
  rtb_Sum2_p[8] = rtDW.s_g_ref_dt2[2] - (0.208835155F *
    rtDW.DiscreteTimeIntegratory_dt_DS_e[8] +
    rtDW.DiscreteTimeIntegratory_DSTAT_l[8]);

  // Sum: '<S114>/Add' incorporates:
  //   DiscreteIntegrator: '<S115>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S75>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S77>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S78>/Discrete-Time Integrator y'

  rtDW.Add[2] = rtDW.DiscreteTimeIntegratory_DSTAT_l[2] -
    rtDW.DiscreteTimeIntegratory_DSTATE[2];
  rtDW.Add[5] = rtDW.DiscreteTimeIntegratory_DSTAT_l[5] -
    rtDW.DiscreteTimeIntegratory_DSTAT_a[2];
  rtDW.Add[8] = rtDW.DiscreteTimeIntegratory_DSTAT_l[8] -
    rtDW.DiscreteTimeIntegratory_DSTA_al[2];

  // Switch: '<S116>/Switch2' incorporates:
  //   RelationalOperator: '<S116>/LowerRelop1'
  //   RelationalOperator: '<S116>/UpperRelop'
  //   Switch: '<S116>/Switch'

  if (rtDW.Add[0] > 10.7829809F) {
    rtb_Gain3_b = 10.7829809F;
  } else if (rtDW.Add[0] < -10.7829809F) {
    // Switch: '<S116>/Switch'
    rtb_Gain3_b = -10.7829809F;
  } else {
    rtb_Gain3_b = rtDW.Add[0];
  }

  // Sum: '<S114>/Add1' incorporates:
  //   Gain: '<S114>/Gain'
  //   Gain: '<S114>/Gain3'
  //   Gain: '<S114>/Gain4'

  rtb_n_b_dt2[0] = (3.71769834F * rtb_Gain3_b + 3.81114435F * rtDW.Add[3]) +
    0.503125F * rtDW.Add[6];

  // Switch: '<S116>/Switch2' incorporates:
  //   RelationalOperator: '<S116>/LowerRelop1'
  //   RelationalOperator: '<S116>/UpperRelop'
  //   Switch: '<S116>/Switch'

  if (rtDW.Add[1] > 10.7829809F) {
    rtb_Gain3_b = 10.7829809F;
  } else if (rtDW.Add[1] < -10.7829809F) {
    // Switch: '<S116>/Switch'
    rtb_Gain3_b = -10.7829809F;
  } else {
    rtb_Gain3_b = rtDW.Add[1];
  }

  // Sum: '<S114>/Add1' incorporates:
  //   Gain: '<S114>/Gain'
  //   Gain: '<S114>/Gain3'
  //   Gain: '<S114>/Gain4'

  rtb_n_b_dt2[1] = (3.71769834F * rtb_Gain3_b + 3.81114435F * rtDW.Add[4]) +
    0.503125F * rtDW.Add[7];

  // Switch: '<S116>/Switch2' incorporates:
  //   RelationalOperator: '<S116>/LowerRelop1'
  //   RelationalOperator: '<S116>/UpperRelop'
  //   Switch: '<S116>/Switch'

  if (rtDW.Add[2] > 10.7829809F) {
    rtb_Gain3_b = 10.7829809F;
  } else if (rtDW.Add[2] < -10.7829809F) {
    // Switch: '<S116>/Switch'
    rtb_Gain3_b = -10.7829809F;
  } else {
    rtb_Gain3_b = rtDW.Add[2];
  }

  // Sum: '<S114>/Add1' incorporates:
  //   Gain: '<S114>/Gain'
  //   Gain: '<S114>/Gain3'
  //   Gain: '<S114>/Gain4'

  rtb_n_b_dt2[2] = (3.71769834F * rtb_Gain3_b + 3.81114435F * rtDW.Add[5]) +
    0.503125F * rtDW.Add[8];

  // RelationalOperator: '<S117>/LowerRelop1'
  dist_remaining = rtb_n_b_dt2[0];

  // Switch: '<S117>/Switch' incorporates:
  //   RelationalOperator: '<S117>/LowerRelop1'
  //   RelationalOperator: '<S117>/UpperRelop'

  if (rtb_n_b_dt2[0] < -20.7358551F) {
    dist_remaining = -20.7358551F;
  }

  // Switch: '<S117>/Switch2' incorporates:
  //   RelationalOperator: '<S117>/LowerRelop1'

  if (rtb_n_b_dt2[0] > 20.7358551F) {
    dist_remaining = 20.7358551F;
  }

  // Sum: '<S18>/Add1'
  rtDW.nu[0] = rtDW.s_g_ref_dt2[0] + dist_remaining;

  // SignalConversion: '<S18>/BusConversion_InsertedFor_pos_cntrl_at_inport_0'
  rtDW.s_g_ref_f[0] = rtDW.s_g_ref[0];

  // SignalConversion: '<S18>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' incorporates:
  //   DiscreteIntegrator: '<S78>/Discrete-Time Integrator y'

  rtDW.s_g[0] = rtDW.DiscreteTimeIntegratory_DSTATE[0];

  // SignalConversion: '<S18>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' incorporates:
  //   DiscreteIntegrator: '<S77>/Discrete-Time Integrator y'

  rtDW.s_g_dt[0] = rtDW.DiscreteTimeIntegratory_DSTAT_a[0];

  // SignalConversion: '<S18>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' incorporates:
  //   DiscreteIntegrator: '<S75>/Discrete-Time Integrator y'

  rtDW.s_g_dt2[0] = rtDW.DiscreteTimeIntegratory_DSTA_al[0];

  // Switch: '<S117>/Switch' incorporates:
  //   RelationalOperator: '<S117>/UpperRelop'

  rtb_n_b_dt2[0] = dist_remaining;

  // RelationalOperator: '<S117>/LowerRelop1'
  dist_remaining = rtb_n_b_dt2[1];

  // Switch: '<S117>/Switch' incorporates:
  //   RelationalOperator: '<S117>/LowerRelop1'
  //   RelationalOperator: '<S117>/UpperRelop'

  if (rtb_n_b_dt2[1] < -20.7358551F) {
    dist_remaining = -20.7358551F;
  }

  // Switch: '<S117>/Switch2' incorporates:
  //   RelationalOperator: '<S117>/LowerRelop1'

  if (rtb_n_b_dt2[1] > 20.7358551F) {
    dist_remaining = 20.7358551F;
  }

  // Sum: '<S18>/Add1'
  rtDW.nu[1] = rtDW.s_g_ref_dt2[1] + dist_remaining;

  // SignalConversion: '<S18>/BusConversion_InsertedFor_pos_cntrl_at_inport_0'
  rtDW.s_g_ref_f[1] = rtDW.s_g_ref[1];

  // SignalConversion: '<S18>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' incorporates:
  //   DiscreteIntegrator: '<S78>/Discrete-Time Integrator y'

  rtDW.s_g[1] = rtDW.DiscreteTimeIntegratory_DSTATE[1];

  // SignalConversion: '<S18>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' incorporates:
  //   DiscreteIntegrator: '<S77>/Discrete-Time Integrator y'

  rtDW.s_g_dt[1] = rtDW.DiscreteTimeIntegratory_DSTAT_a[1];

  // SignalConversion: '<S18>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' incorporates:
  //   DiscreteIntegrator: '<S75>/Discrete-Time Integrator y'

  rtDW.s_g_dt2[1] = rtDW.DiscreteTimeIntegratory_DSTA_al[1];

  // Switch: '<S117>/Switch' incorporates:
  //   RelationalOperator: '<S117>/UpperRelop'

  rtb_n_b_dt2[1] = dist_remaining;

  // RelationalOperator: '<S117>/LowerRelop1'
  dist_remaining = rtb_n_b_dt2[2];

  // Switch: '<S117>/Switch' incorporates:
  //   RelationalOperator: '<S117>/LowerRelop1'
  //   RelationalOperator: '<S117>/UpperRelop'

  if (rtb_n_b_dt2[2] < -20.7358551F) {
    dist_remaining = -20.7358551F;
  }

  // Switch: '<S117>/Switch2' incorporates:
  //   RelationalOperator: '<S117>/LowerRelop1'

  if (rtb_n_b_dt2[2] > 20.7358551F) {
    dist_remaining = 20.7358551F;
  }

  // Sum: '<S18>/Add1'
  rtDW.nu[2] = rtDW.s_g_ref_dt2[2] + dist_remaining;

  // SignalConversion: '<S18>/BusConversion_InsertedFor_pos_cntrl_at_inport_0'
  rtDW.s_g_ref_f[2] = rtDW.s_g_ref[2];

  // SignalConversion: '<S18>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' incorporates:
  //   DiscreteIntegrator: '<S78>/Discrete-Time Integrator y'

  rtDW.s_g[2] = rtDW.DiscreteTimeIntegratory_DSTATE[2];

  // SignalConversion: '<S18>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' incorporates:
  //   DiscreteIntegrator: '<S77>/Discrete-Time Integrator y'

  rtDW.s_g_dt[2] = rtDW.DiscreteTimeIntegratory_DSTAT_a[2];

  // SignalConversion: '<S18>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' incorporates:
  //   DiscreteIntegrator: '<S75>/Discrete-Time Integrator y'

  rtDW.s_g_dt2[2] = rtDW.DiscreteTimeIntegratory_DSTA_al[2];

  // Switch: '<S117>/Switch' incorporates:
  //   RelationalOperator: '<S117>/UpperRelop'

  rtb_n_b_dt2[2] = dist_remaining;

  // Update for DiscreteIntegrator: '<S115>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S115>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_IC_L_b2 = 0U;
  for (i = 0; i < 9; i++) {
    rtDW.DiscreteTimeIntegratory_DSTAT_l[i] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_e[i];

    // Update for DiscreteIntegrator: '<S115>/Discrete-Time Integrator y_dt' incorporates:
    //   Gain: '<S115>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_DS_e[i] += 91.7176285F * rtb_Sum2_p[i] *
      0.0025F;
  }

  // End of Update for DiscreteIntegrator: '<S115>/Discrete-Time Integrator y'
  // End of Outputs for SubSystem: '<S2>/NDI position controller for copters with reference input' 

  // DiscreteIntegrator: '<S79>/Discrete-Time Integrator y_dt'
  rtb_y_c[0] = rtDW.DiscreteTimeIntegratory_dt_DSTA[0];
  rtb_y_c[1] = rtDW.DiscreteTimeIntegratory_dt_DSTA[1];
  rtb_y_c[2] = rtDW.DiscreteTimeIntegratory_dt_DSTA[2];

  // MATLAB Function: '<Root>/Quaternions to Rotation Matrix' incorporates:
  //   Inport: '<Root>/measure'

  rtb_Gain5 = 1.29246971E-26F;
  rtb_distance = std::abs(rtU.measure.q_bg[0]);
  if (rtb_distance > 1.29246971E-26F) {
    rtb_Sqrt = 1.0F;
    rtb_Gain5 = rtb_distance;
  } else {
    dist_remaining = rtb_distance / 1.29246971E-26F;
    rtb_Sqrt = dist_remaining * dist_remaining;
  }

  rtb_distance = std::abs(rtU.measure.q_bg[1]);
  if (rtb_distance > rtb_Gain5) {
    dist_remaining = rtb_Gain5 / rtb_distance;
    rtb_Sqrt = rtb_Sqrt * dist_remaining * dist_remaining + 1.0F;
    rtb_Gain5 = rtb_distance;
  } else {
    dist_remaining = rtb_distance / rtb_Gain5;
    rtb_Sqrt += dist_remaining * dist_remaining;
  }

  rtb_distance = std::abs(rtU.measure.q_bg[2]);
  if (rtb_distance > rtb_Gain5) {
    dist_remaining = rtb_Gain5 / rtb_distance;
    rtb_Sqrt = rtb_Sqrt * dist_remaining * dist_remaining + 1.0F;
    rtb_Gain5 = rtb_distance;
  } else {
    dist_remaining = rtb_distance / rtb_Gain5;
    rtb_Sqrt += dist_remaining * dist_remaining;
  }

  rtb_distance = std::abs(rtU.measure.q_bg[3]);
  if (rtb_distance > rtb_Gain5) {
    dist_remaining = rtb_Gain5 / rtb_distance;
    rtb_Sqrt = rtb_Sqrt * dist_remaining * dist_remaining + 1.0F;
    rtb_Gain5 = rtb_distance;
  } else {
    dist_remaining = rtb_distance / rtb_Gain5;
    rtb_Sqrt += dist_remaining * dist_remaining;
  }

  rtb_Sqrt = rtb_Gain5 * std::sqrt(rtb_Sqrt);
  if (2.22044605E-16F < rtb_Sqrt) {
    rtb_Gain3_b = rtb_Sqrt;
  } else {
    rtb_Gain3_b = 2.22044605E-16F;
  }

  rtb_Add_a[0] = rtU.measure.q_bg[0] / rtb_Gain3_b;
  rtb_Add_a[1] = rtU.measure.q_bg[1] / rtb_Gain3_b;
  rtb_Add_a[2] = rtU.measure.q_bg[2] / rtb_Gain3_b;
  rtb_Add_a[3] = rtU.measure.q_bg[3] / rtb_Gain3_b;
  dist_remaining = rtb_Add_a[0] * rtb_Add_a[0];
  rtb_Sqrt = rtb_Add_a[1] * rtb_Add_a[1];
  rtb_Gain5 = rtb_Add_a[2] * rtb_Add_a[2];
  rtb_distance = rtb_Add_a[3] * rtb_Add_a[3];
  rtb_Gain3_b = rtb_Add_a[0] * rtb_Add_a[1];
  dist_available = rtb_Add_a[0] * rtb_Add_a[2];
  q0_q3 = rtb_Add_a[0] * rtb_Add_a[3];
  q1_q2 = rtb_Add_a[1] * rtb_Add_a[2];
  q1_q3 = rtb_Add_a[1] * rtb_Add_a[3];
  rtb_Gain = rtb_Add_a[2] * rtb_Add_a[3];
  rtb_Sum2_p[0] = ((dist_remaining + rtb_Sqrt) - rtb_Gain5) - rtb_distance;
  rtb_Sum2_p[3] = (q1_q2 + q0_q3) * 2.0F;
  rtb_Sum2_p[6] = (q1_q3 - dist_available) * 2.0F;
  rtb_Sum2_p[1] = (q1_q2 - q0_q3) * 2.0F;
  dist_remaining -= rtb_Sqrt;
  rtb_Sum2_p[4] = (dist_remaining + rtb_Gain5) - rtb_distance;
  rtb_Sum2_p[7] = (rtb_Gain + rtb_Gain3_b) * 2.0F;
  rtb_Sum2_p[2] = (q1_q3 + dist_available) * 2.0F;
  rtb_Sum2_p[5] = (rtb_Gain - rtb_Gain3_b) * 2.0F;
  rtb_Sum2_p[8] = (dist_remaining - rtb_Gain5) + rtb_distance;

  // End of MATLAB Function: '<Root>/Quaternions to Rotation Matrix'

  // DiscreteIntegrator: '<S76>/Discrete-Time Integrator y'
  if (rtDW.DiscreteTimeIntegratory_IC_LO_b != 0) {
    for (i = 0; i < 9; i++) {
      rtDW.DiscreteTimeIntegratory_DSTAT_p[i] = rtb_Sum2_p[i];
    }
  }

  for (i = 0; i < 9; i++) {
    rtb_y_i[i] = rtDW.DiscreteTimeIntegratory_DSTAT_p[i];
  }

  // Outputs for Enabled SubSystem: '<S25>/Copter Random Excitation' incorporates:
  //   EnablePort: '<S27>/Enable'

  if (rtDW.CopterRandomExcitation_MODE) {
    // Disable for Outport: '<S27>/yaw_rate_excite'
    rtDW.Gain1 = 0.0F;

    // Disable for Outport: '<S27>/s_g_dt2_excite'
    rtDW.Gain[0] = 0.0F;
    rtDW.Gain[1] = 0.0F;
    rtDW.Gain[2] = 0.0F;
    rtDW.CopterRandomExcitation_MODE = false;
  }

  // End of Outputs for SubSystem: '<S25>/Copter Random Excitation'

  // Outputs for Enabled SubSystem: '<S2>/Accelerations to Reduced Attitude and Thrust' incorporates:
  //   EnablePort: '<S10>/Enable'

  // MATLAB Function: '<S10>/INDI Copter Acc 2 Lean Vector' incorporates:
  //   Constant: '<S2>/Constant2'
  //   DiscreteIntegrator: '<S75>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S76>/Discrete-Time Integrator y'
  //   Sum: '<S2>/Add1'

  rtb_omega2_f[0] = rtDW.DiscreteTimeIntegratory_DSTA_al[0];
  rtb_omega2_f[1] = rtDW.DiscreteTimeIntegratory_DSTA_al[1];
  rtb_omega2_f[2] = rtDW.DiscreteTimeIntegratory_DSTA_al[2] - 9.81F;
  rtb_Sqrt = 0.0F;
  for (k = 0; k < 3; k++) {
    q1_q3 = -rtDW.DiscreteTimeIntegratory_DSTAT_p[3 * k + 2];
    rtb_Sqrt += q1_q3 * rtb_omega2_f[k];
    rtb_y_dt_g[k] = q1_q3;
  }

  rtb_Gain5 = 1.29246971E-26F;
  q1_q3 = ((rtDW.Gain[0] + rtDW.nu[0]) - rtDW.DiscreteTimeIntegratory_DSTA_al[0])
    + rtb_y_dt_g[0] * rtb_Sqrt;
  rtb_distance = std::abs(q1_q3);
  if (rtb_distance > 1.29246971E-26F) {
    rtb_Gain = 1.0F;
    rtb_Gain5 = rtb_distance;
  } else {
    dist_remaining = rtb_distance / 1.29246971E-26F;
    rtb_Gain = dist_remaining * dist_remaining;
  }

  rtb_y_dt_g[0] = q1_q3;
  q1_q3 = ((rtDW.Gain[1] + rtDW.nu[1]) - rtDW.DiscreteTimeIntegratory_DSTA_al[1])
    + rtb_y_dt_g[1] * rtb_Sqrt;
  rtb_distance = std::abs(q1_q3);
  if (rtb_distance > rtb_Gain5) {
    dist_remaining = rtb_Gain5 / rtb_distance;
    rtb_Gain = rtb_Gain * dist_remaining * dist_remaining + 1.0F;
    rtb_Gain5 = rtb_distance;
  } else {
    dist_remaining = rtb_distance / rtb_Gain5;
    rtb_Gain += dist_remaining * dist_remaining;
  }

  rtb_y_dt_g[1] = q1_q3;
  q1_q3 = ((rtDW.Gain[2] + rtDW.nu[2]) - rtDW.DiscreteTimeIntegratory_DSTA_al[2])
    + rtb_y_dt_g[2] * rtb_Sqrt;
  rtb_distance = std::abs(q1_q3);
  if (rtb_distance > rtb_Gain5) {
    dist_remaining = rtb_Gain5 / rtb_distance;
    rtb_Gain = rtb_Gain * dist_remaining * dist_remaining + 1.0F;
    rtb_Gain5 = rtb_distance;
  } else {
    dist_remaining = rtb_distance / rtb_Gain5;
    rtb_Gain += dist_remaining * dist_remaining;
  }

  rtb_Gain = rtb_Gain5 * std::sqrt(rtb_Gain);
  rtb_Sqrt = rtb_Gain;
  if (rtb_Gain < 2.22044605E-16F) {
    rtb_Sqrt = 2.22044605E-16F;
  }

  dist_remaining = 1.0F / rtb_Sqrt;
  rtb_Gain3_b = dist_remaining * rtb_y_dt_g[0];

  // SignalConversion: '<S10>/BusConversion_InsertedFor_red_atti_des_at_inport_0' 
  rtDW.n_g_des[0] = rtb_Gain3_b;

  // MATLAB Function: '<S10>/INDI Copter Acc 2 Lean Vector'
  rtb_y_dt_g[0] = rtb_Gain3_b;
  rtb_Gain3_b = dist_remaining * rtb_y_dt_g[1];

  // SignalConversion: '<S10>/BusConversion_InsertedFor_red_atti_des_at_inport_0' 
  rtDW.n_g_des[1] = rtb_Gain3_b;

  // MATLAB Function: '<S10>/INDI Copter Acc 2 Lean Vector'
  rtb_y_dt_g[1] = rtb_Gain3_b;
  rtb_Gain3_b = dist_remaining * q1_q3;

  // SignalConversion: '<S10>/BusConversion_InsertedFor_red_atti_des_at_inport_0' 
  rtDW.n_g_des[2] = rtb_Gain3_b;

  // MATLAB Function: '<S10>/MATLAB Function4'
  if (1.0F > -rtb_Gain3_b) {
    dist_remaining = -rtb_Gain3_b;
  } else {
    dist_remaining = 1.0F;
  }

  // SignalConversion: '<S10>/BusConversion_InsertedFor_red_atti_des_at_inport_0' incorporates:
  //   MATLAB Function: '<S10>/MATLAB Function4'

  rtDW.lean_dir_angle_des = std::atan2(rtb_y_dt_g[1], rtb_y_dt_g[0]);

  // MATLAB Function: '<S10>/MATLAB Function4'
  if (-1.0F >= dist_remaining) {
    dist_remaining = -1.0F;
  }

  rtb_distance = std::acos(dist_remaining);

  // Saturate: '<S10>/Saturation1'
  if (rtb_distance > 3.14159274F) {
    rtb_distance = 3.14159274F;
  } else {
    if (rtb_distance < 0.0F) {
      rtb_distance = 0.0F;
    }
  }

  // End of Saturate: '<S10>/Saturation1'

  // SignalConversion: '<S10>/BusConversion_InsertedFor_red_atti_des_at_inport_0' incorporates:
  //   Gain: '<S10>/Gain1'

  rtDW.cmd_lean_angle_01 = 0.318309873F * rtb_distance;

  // SignalConversion: '<S10>/OutportBufferForT_spec_des' incorporates:
  //   MATLAB Function: '<S10>/INDI Copter Acc 2 Lean Vector'

  rtDW.Merge1 = rtb_Gain;

  // End of Outputs for SubSystem: '<S2>/Accelerations to Reduced Attitude and Thrust' 

  // Gain: '<S89>/lean_angle_max'
  dist_remaining = 3.14159274F * rtDW.cmd_lean_angle_01;

  // MATLAB Function: '<S89>/lean angles 2 lean vector' incorporates:
  //   Gain: '<S89>/lean_angle_max'

  rtb_Sqrt = std::sin(dist_remaining);
  rtb_y_dt_g[0] = rtb_Sqrt * std::cos(rtDW.lean_dir_angle_des);
  rtb_y_dt_g[1] = rtb_Sqrt * std::sin(rtDW.lean_dir_angle_des);
  rtb_y_dt_g[2] = -std::cos(dist_remaining);

  // DiscreteIntegrator: '<S102>/Discrete-Time Integrator y'
  if (rtDW.DiscreteTimeIntegratory_IC_LO_i != 0) {
    rtDW.DiscreteTimeIntegratory_DSTAT_m[0] = rtb_y_dt_g[0];
    rtDW.DiscreteTimeIntegratory_DSTAT_m[1] = rtb_y_dt_g[1];
    rtDW.DiscreteTimeIntegratory_DSTAT_m[2] = rtb_y_dt_g[2];
  }

  // MATLAB Function: '<S102>/n ref norm' incorporates:
  //   DiscreteIntegrator: '<S102>/Discrete-Time Integrator y'

  rtb_Gain5 = 1.29246971E-26F;
  rtb_distance = std::abs(rtDW.DiscreteTimeIntegratory_DSTAT_m[0]);
  if (rtb_distance > 1.29246971E-26F) {
    rtb_Gain = 1.0F;
    rtb_Gain5 = rtb_distance;
  } else {
    dist_remaining = rtb_distance / 1.29246971E-26F;
    rtb_Gain = dist_remaining * dist_remaining;
  }

  // Gain: '<S102>/omega^2' incorporates:
  //   DiscreteIntegrator: '<S102>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S102>/Discrete-Time Integrator y_dt'
  //   Gain: '<S102>/2*d//omega'
  //   Sum: '<S102>/Sum2'
  //   Sum: '<S102>/Sum3'

  rtb_y_dt_g[0] = (rtb_y_dt_g[0] - (0.150825381F *
    rtDW.DiscreteTimeIntegratory_dt_DS_o[0] +
    rtDW.DiscreteTimeIntegratory_DSTAT_m[0])) * 175.837341F;

  // MATLAB Function: '<S102>/n ref norm' incorporates:
  //   DiscreteIntegrator: '<S102>/Discrete-Time Integrator y'

  rtb_distance = std::abs(rtDW.DiscreteTimeIntegratory_DSTAT_m[1]);
  if (rtb_distance > rtb_Gain5) {
    dist_remaining = rtb_Gain5 / rtb_distance;
    rtb_Gain = rtb_Gain * dist_remaining * dist_remaining + 1.0F;
    rtb_Gain5 = rtb_distance;
  } else {
    dist_remaining = rtb_distance / rtb_Gain5;
    rtb_Gain += dist_remaining * dist_remaining;
  }

  // Gain: '<S102>/omega^2' incorporates:
  //   DiscreteIntegrator: '<S102>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S102>/Discrete-Time Integrator y_dt'
  //   Gain: '<S102>/2*d//omega'
  //   Sum: '<S102>/Sum2'
  //   Sum: '<S102>/Sum3'

  rtb_y_dt_g[1] = (rtb_y_dt_g[1] - (0.150825381F *
    rtDW.DiscreteTimeIntegratory_dt_DS_o[1] +
    rtDW.DiscreteTimeIntegratory_DSTAT_m[1])) * 175.837341F;
  q1_q3 = (rtb_y_dt_g[2] - (0.150825381F * rtDW.DiscreteTimeIntegratory_dt_DS_o
            [2] + rtDW.DiscreteTimeIntegratory_DSTAT_m[2])) * 175.837341F;

  // MATLAB Function: '<S102>/n ref norm' incorporates:
  //   DiscreteIntegrator: '<S102>/Discrete-Time Integrator y'

  rtb_distance = std::abs(rtDW.DiscreteTimeIntegratory_DSTAT_m[2]);
  if (rtb_distance > rtb_Gain5) {
    dist_remaining = rtb_Gain5 / rtb_distance;
    rtb_Gain = rtb_Gain * dist_remaining * dist_remaining + 1.0F;
    rtb_Gain5 = rtb_distance;
  } else {
    dist_remaining = rtb_distance / rtb_Gain5;
    rtb_Gain += dist_remaining * dist_remaining;
  }

  // Gain: '<S102>/omega^2' incorporates:
  //   Sum: '<S102>/Sum2'

  rtb_y_dt_g[2] = q1_q3;

  // MATLAB Function: '<S102>/n ref norm' incorporates:
  //   DiscreteIntegrator: '<S102>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S102>/Discrete-Time Integrator y_dt'

  rtb_Gain = rtb_Gain5 * std::sqrt(rtb_Gain);
  if (rtb_Gain < 2.22044605E-16F) {
    rtb_Gain = 2.22044605E-16F;
  }

  rtb_n[0] = rtDW.DiscreteTimeIntegratory_DSTAT_m[0] / rtb_Gain;
  rtb_n[1] = rtDW.DiscreteTimeIntegratory_DSTAT_m[1] / rtb_Gain;
  rtb_n[2] = rtDW.DiscreteTimeIntegratory_DSTAT_m[2] / rtb_Gain;
  dist_available = rtDW.DiscreteTimeIntegratory_DSTAT_m[1] *
    rtDW.DiscreteTimeIntegratory_DSTAT_m[1];
  rtb_distance = rtDW.DiscreteTimeIntegratory_DSTAT_m[2] *
    rtDW.DiscreteTimeIntegratory_DSTAT_m[2];
  rtb_Gain5 = rtDW.DiscreteTimeIntegratory_DSTAT_m[0] *
    rtDW.DiscreteTimeIntegratory_DSTAT_m[0];
  rtb_Sqrt = rtb_Gain5 + dist_available;
  rtb_Gain = std::pow(rtb_Sqrt + rtb_distance, 1.5F);
  dist_remaining = rtb_Gain;
  rtb_Gain3_b = std::abs(rtb_Gain);
  if (rtb_Gain3_b < 2.22044605E-16F) {
    dist_remaining = 2.22044605E-16F;
  }

  dist_remaining = 1.0F / dist_remaining;
  dist_remaining_0[0] = (dist_available + rtb_distance) * dist_remaining;
  dist_available = -rtDW.DiscreteTimeIntegratory_DSTAT_m[0] *
    rtDW.DiscreteTimeIntegratory_DSTAT_m[1] * dist_remaining;
  dist_remaining_0[3] = dist_available;
  q0_q3 = -rtDW.DiscreteTimeIntegratory_DSTAT_m[0] *
    rtDW.DiscreteTimeIntegratory_DSTAT_m[2];
  q1_q2 = q0_q3 * dist_remaining;
  dist_remaining_0[6] = q1_q2;
  dist_remaining_0[1] = dist_available;
  dist_remaining_0[4] = (rtb_Gain5 + rtb_distance) * dist_remaining;
  dist_available = -rtDW.DiscreteTimeIntegratory_DSTAT_m[1] *
    rtDW.DiscreteTimeIntegratory_DSTAT_m[2] * dist_remaining;
  dist_remaining_0[7] = dist_available;
  dist_remaining_0[2] = q1_q2;
  dist_remaining_0[5] = dist_available;
  dist_remaining_0[8] = rtb_Sqrt * dist_remaining;
  for (i = 0; i < 3; i++) {
    rtb_n_dt_o[i] = dist_remaining_0[i + 6] *
      rtDW.DiscreteTimeIntegratory_dt_DS_o[2] + (dist_remaining_0[i + 3] *
      rtDW.DiscreteTimeIntegratory_dt_DS_o[1] + dist_remaining_0[i] *
      rtDW.DiscreteTimeIntegratory_dt_DS_o[0]);
  }

  if (rtb_Gain3_b < 2.22044605E-16F) {
    rtb_Gain = 2.22044605E-16F;
  }

  rtb_n_dt_o[2] = ((q0_q3 * rtDW.DiscreteTimeIntegratory_dt_DS_o[0] + rtb_Gain5 *
                    rtDW.DiscreteTimeIntegratory_dt_DS_o[2]) +
                   (rtDW.DiscreteTimeIntegratory_DSTAT_m[1] *
                    rtDW.DiscreteTimeIntegratory_dt_DS_o[2] -
                    rtDW.DiscreteTimeIntegratory_DSTAT_m[2] *
                    rtDW.DiscreteTimeIntegratory_dt_DS_o[1]) *
                   rtDW.DiscreteTimeIntegratory_DSTAT_m[1]) * (1.0F / rtb_Gain);
  leanVectorNormDeriv2_c(rtDW.DiscreteTimeIntegratory_DSTAT_m,
    rtDW.DiscreteTimeIntegratory_dt_DS_o, rtb_y_dt_g, tmp);
  rtb_n_dt2_i[0] = (real32_T)tmp[0];
  rtb_n_dt2_i[1] = (real32_T)tmp[1];
  rtb_n_dt2_i[2] = (real32_T)tmp[2];

  // DiscreteIntegrator: '<S103>/Discrete-Time Integrator y'
  if (rtDW.DiscreteTimeIntegratory_IC_LO_a != 0) {
    rtDW.DiscreteTimeIntegratory_DSTAT_e[0] = rtb_n[0];
    rtDW.DiscreteTimeIntegratory_DSTAT_e[1] = rtb_n[1];
    rtDW.DiscreteTimeIntegratory_DSTAT_e[2] = rtb_n[2];
  }

  rtb_Sum2_e[0] = rtDW.DiscreteTimeIntegratory_DSTAT_e[0];

  // DiscreteIntegrator: '<S103>/Discrete-Time Integrator y_dt'
  rtb_y_dt_cs[0] = rtDW.DiscreteTimeIntegratory_dt_DS_j[0];

  // Gain: '<S103>/omega^2' incorporates:
  //   DiscreteIntegrator: '<S103>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S103>/Discrete-Time Integrator y_dt'
  //   Gain: '<S103>/2*d//omega'
  //   Sum: '<S103>/Sum2'
  //   Sum: '<S103>/Sum3'

  rtb_omega2_f[0] = (rtb_n[0] - (0.0580097549F *
    rtDW.DiscreteTimeIntegratory_dt_DS_j[0] +
    rtDW.DiscreteTimeIntegratory_DSTAT_e[0])) * 1188.66077F;

  // DiscreteIntegrator: '<S103>/Discrete-Time Integrator y'
  rtb_Sum2_e[1] = rtDW.DiscreteTimeIntegratory_DSTAT_e[1];

  // DiscreteIntegrator: '<S103>/Discrete-Time Integrator y_dt'
  rtb_y_dt_cs[1] = rtDW.DiscreteTimeIntegratory_dt_DS_j[1];

  // Gain: '<S103>/omega^2' incorporates:
  //   DiscreteIntegrator: '<S103>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S103>/Discrete-Time Integrator y_dt'
  //   Gain: '<S103>/2*d//omega'
  //   Sum: '<S103>/Sum2'
  //   Sum: '<S103>/Sum3'

  rtb_omega2_f[1] = (rtb_n[1] - (0.0580097549F *
    rtDW.DiscreteTimeIntegratory_dt_DS_j[1] +
    rtDW.DiscreteTimeIntegratory_DSTAT_e[1])) * 1188.66077F;

  // DiscreteIntegrator: '<S103>/Discrete-Time Integrator y'
  rtb_Sum2_e[2] = rtDW.DiscreteTimeIntegratory_DSTAT_e[2];

  // DiscreteIntegrator: '<S103>/Discrete-Time Integrator y_dt'
  rtb_y_dt_cs[2] = rtDW.DiscreteTimeIntegratory_dt_DS_j[2];

  // Gain: '<S103>/omega^2' incorporates:
  //   DiscreteIntegrator: '<S103>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S103>/Discrete-Time Integrator y_dt'
  //   Gain: '<S103>/2*d//omega'
  //   Sum: '<S103>/Sum2'
  //   Sum: '<S103>/Sum3'

  rtb_omega2_f[2] = (rtb_n[2] - (0.0580097549F *
    rtDW.DiscreteTimeIntegratory_dt_DS_j[2] +
    rtDW.DiscreteTimeIntegratory_DSTAT_e[2])) * 1188.66077F;

  // MATLAB Function: '<S103>/n ref norm'
  nrefnorm(rtb_omega2_f, rtb_y_dt_cs, rtb_Sum2_e, rtb_n_dt2, rtb_n_dt,
           rtb_s_g_dt2_ref);

  // DiscreteIntegrator: '<S79>/Discrete-Time Integrator y' incorporates:
  //   Inport: '<Root>/measure'

  if (rtDW.DiscreteTimeIntegratory_IC_LO_d != 0) {
    rtDW.DiscreteTimeIntegratory_DSTAT_k[0] = rtU.measure.omega_Kb[0];
    rtDW.DiscreteTimeIntegratory_DSTAT_k[1] = rtU.measure.omega_Kb[1];
    rtDW.DiscreteTimeIntegratory_DSTAT_k[2] = rtU.measure.omega_Kb[2];
  }

  rtb_y_dt_cs[0] = rtDW.DiscreteTimeIntegratory_DSTAT_k[0];
  rtb_y_dt_cs[1] = rtDW.DiscreteTimeIntegratory_DSTAT_k[1];
  rtb_y_dt_cs[2] = rtDW.DiscreteTimeIntegratory_DSTAT_k[2];

  // MATLAB Function: '<S89>/Lean Vector Derivative Trafo Delay'
  LeanVectorDerivativeTrafo(rtb_s_g_dt2_ref, rtb_n_dt, rtb_n_dt2, rtb_y_i,
    rtb_y_dt_cs, rtb_y_c, rtb_n_b, rtb_n_b_dt, rtb_Sum2_e);

  // MATLAB Function: '<S89>/Lean Vector Derivative Trafo'
  LeanVectorDerivativeTrafo(rtb_n, rtb_n_dt_o, rtb_n_dt2_i, rtb_y_i, rtb_y_dt_cs,
    rtb_y_c, rtb_n_b_c, rtb_n_b_dt_p, rtb_n_b_dt2);

  // MATLAB Function: '<S89>/MATLAB Function'
  rtb_y_dt_cs[0] = rtb_n_b[0];
  rtb_y_dt_cs[1] = rtb_n_b[1];
  if (rtb_n_b[2] > 0.0F) {
    if (rtb_n_b[2] > 0.999) {
      dist_remaining = 0.0F;
      rtb_n_b[0] = 0.0F;
      rtb_Gain = -1.0F;
      rtb_n_b[1] = 0.0F;
    } else {
      rtb_Gain5 = 1.29246971E-26F;
      rtb_distance = std::abs(rtb_n_b[0]);
      if (rtb_distance > 1.29246971E-26F) {
        rtb_Sqrt = 1.0F;
        rtb_Gain5 = rtb_distance;
      } else {
        dist_remaining = rtb_distance / 1.29246971E-26F;
        rtb_Sqrt = dist_remaining * dist_remaining;
      }

      rtb_distance = std::abs(rtb_n_b[1]);
      if (rtb_distance > rtb_Gain5) {
        dist_remaining = rtb_Gain5 / rtb_distance;
        rtb_Sqrt = rtb_Sqrt * dist_remaining * dist_remaining + 1.0F;
        rtb_Gain5 = rtb_distance;
      } else {
        dist_remaining = rtb_distance / rtb_Gain5;
        rtb_Sqrt += dist_remaining * dist_remaining;
      }

      rtb_Sqrt = rtb_Gain5 * std::sqrt(rtb_Sqrt);
      if (rtb_Sqrt < 2.22044605E-16F) {
        rtb_Sqrt = 2.22044605E-16F;
      }

      dist_remaining = rtb_n_b[0] / rtb_Sqrt;
      rtb_Gain = rtb_n_b[1] / rtb_Sqrt;
    }

    rtb_y_dt_cs[0] = 2.0F * dist_remaining - rtb_n_b[0];
    rtb_n_b_dt[0] = -rtb_n_b_dt[0];
    rtb_Sum2_e[0] = -rtb_Sum2_e[0];
    rtb_y_dt_cs[1] = 2.0F * rtb_Gain - rtb_n_b[1];
    rtb_n_b_dt[1] = -rtb_n_b_dt[1];
    rtb_Sum2_e[1] = -rtb_Sum2_e[1];
  }

  // MATLAB Function: '<S83>/Reduced Attitude Weighting Factors' incorporates:
  //   MATLAB Function: '<S89>/MATLAB Function'

  rtb_distance = -rtb_n_b[2];
  if (-rtb_n_b[2] < 0.0F) {
    rtb_distance = 0.0F;
  }

  // Gain: '<S107>/1//T' incorporates:
  //   DiscreteIntegrator: '<S107>/Discrete-Time Integrator'
  //   Gain: '<S89>/r_max'
  //   Sum: '<S107>/Sum2'
  //   Sum: '<S2>/Add'

  rtb_Gain = ((rtDW.Gain1 + rtb_Switch[2]) * 6.28318548F -
              rtDW.DiscreteTimeIntegrator_DSTATE_h) * 6.0716629F;

  // MATLAB Function: '<S89>/Simulink Trickster' incorporates:
  //   DiscreteIntegrator: '<S107>/Discrete-Time Integrator'

  dist_remaining = rtDW.DiscreteTimeIntegrator_DSTATE_h;

  // MATLAB Function: '<S89>/Desired Roll Pitch' incorporates:
  //   DiscreteIntegrator: '<S76>/Discrete-Time Integrator y'

  rtb_n_b[0] = rtb_s_g_dt2_ref[1] * rtb_n_dt[2] - rtb_s_g_dt2_ref[2] * rtb_n_dt
    [1];
  rtb_n_b[1] = rtb_s_g_dt2_ref[2] * rtb_n_dt[0] - rtb_s_g_dt2_ref[0] * rtb_n_dt
    [2];
  rtb_n_b[2] = rtb_s_g_dt2_ref[0] * rtb_n_dt[1] - rtb_s_g_dt2_ref[1] * rtb_n_dt
    [0];
  for (i = 0; i < 3; i++) {
    rtb_n_dt2_i[i] = rtDW.DiscreteTimeIntegratory_DSTAT_p[i + 6] * rtb_n_b[2] +
      (rtDW.DiscreteTimeIntegratory_DSTAT_p[i + 3] * rtb_n_b[1] +
       rtDW.DiscreteTimeIntegratory_DSTAT_p[i] * rtb_n_b[0]);
  }

  // MATLAB Function: '<S80>/DCM to quaternions'
  DCMtoquaternions(rtb_y_i, rtb_Add_a);

  // MATLAB Function: '<S80>/Quaternion Reduced'
  QuaternionReduced(rtb_Add_a, rtb_Switch, &rtb_Gain5);

  // DiscreteIntegrator: '<S89>/Discrete-Time Integrator2'
  if (rtDW.DiscreteTimeIntegrator2_IC_LOAD != 0) {
    rtDW.DiscreteTimeIntegrator2_DSTATE = rtb_Gain5;
  }

  // MATLAB Function: '<S84>/wrap angle' incorporates:
  //   DiscreteIntegrator: '<S89>/Discrete-Time Integrator2'

  wrapangle(rtDW.DiscreteTimeIntegrator2_DSTATE, &dist_available);

  // MATLAB Function: '<S85>/DCM to quaternions'
  DCMtoquaternions(rtb_y_i, rtb_Add_a);

  // MATLAB Function: '<S85>/Quaternion Reduced'
  QuaternionReduced(rtb_Add_a, rtb_Switch, &rtb_Gain5);

  // MATLAB Function: '<S84>/wrap angle1'
  wrapangle(rtb_Gain5, &rtb_Gain3_b);

  // MATLAB Function: '<S84>/angle error'
  rtb_Gain5 = dist_available - rtb_Gain3_b;
  if (rtb_Gain5 > 3.1415926535897931) {
    rtb_Gain5 -= 6.28318548F;
  } else {
    if (rtb_Gain5 < -3.1415926535897931) {
      rtb_Gain5 += 6.28318548F;
    }
  }

  // End of MATLAB Function: '<S84>/angle error'

  // MATLAB Function: '<S89>/Pseudo-Control Roll Pitch' incorporates:
  //   DiscreteIntegrator: '<S79>/Discrete-Time Integrator y'
  //   MATLAB Function: '<S102>/n ref norm'

  rtb_n_b_c[0] = rtb_n[1] * (real32_T)tmp[2] - rtb_n[2] * (real32_T)tmp[1];
  rtb_n_b_c[1] = rtb_n[2] * (real32_T)tmp[0] - rtb_n[0] * (real32_T)tmp[2];
  rtb_n_b_c[2] = rtb_n[0] * (real32_T)tmp[1] - rtb_n[1] * (real32_T)tmp[0];
  dist_remaining_0[0] = 0.0F;
  dist_remaining_0[3] = -rtDW.DiscreteTimeIntegratory_DSTAT_k[2];
  dist_remaining_0[6] = rtDW.DiscreteTimeIntegratory_DSTAT_k[1];
  dist_remaining_0[1] = rtDW.DiscreteTimeIntegratory_DSTAT_k[2];
  dist_remaining_0[4] = 0.0F;
  dist_remaining_0[7] = -rtDW.DiscreteTimeIntegratory_DSTAT_k[0];
  dist_remaining_0[2] = -rtDW.DiscreteTimeIntegratory_DSTAT_k[1];
  dist_remaining_0[5] = rtDW.DiscreteTimeIntegratory_DSTAT_k[0];
  dist_remaining_0[8] = 0.0F;
  rtb_n_b_dt_p[0] = rtb_n[1] * rtb_n_dt_o[2] - rtb_n[2] * rtb_n_dt_o[1];
  rtb_n_b_dt_p[1] = rtb_n[2] * rtb_n_dt_o[0] - rtb_n[0] * rtb_n_dt_o[2];
  rtb_n_b_dt_p[2] = rtb_n[0] * rtb_n_dt_o[1] - rtb_n[1] * rtb_n_dt_o[0];

  // MATLAB Function: '<S89>/Desired Roll Pitch' incorporates:
  //   DiscreteIntegrator: '<S79>/Discrete-Time Integrator y'

  rtb_Sqrt = rtb_s_g_dt2_ref[1] * rtb_n_dt2[2] - rtb_s_g_dt2_ref[2] * rtb_n_dt2
    [1];
  dist_available = rtb_s_g_dt2_ref[2] * rtb_n_dt2[0] - rtb_s_g_dt2_ref[0] *
    rtb_n_dt2[2];
  q0_q3 = rtb_s_g_dt2_ref[0] * rtb_n_dt2[1] - rtb_s_g_dt2_ref[1] * rtb_n_dt2[0];
  tmp_1[0] = 0.0F;
  tmp_1[3] = -rtDW.DiscreteTimeIntegratory_DSTAT_k[2];
  tmp_1[6] = rtDW.DiscreteTimeIntegratory_DSTAT_k[1];
  tmp_1[1] = rtDW.DiscreteTimeIntegratory_DSTAT_k[2];
  tmp_1[4] = 0.0F;
  tmp_1[7] = -rtDW.DiscreteTimeIntegratory_DSTAT_k[0];
  tmp_1[2] = -rtDW.DiscreteTimeIntegratory_DSTAT_k[1];
  tmp_1[5] = rtDW.DiscreteTimeIntegratory_DSTAT_k[0];
  tmp_1[8] = 0.0F;
  for (i = 0; i < 3; i++) {
    // MATLAB Function: '<S89>/Pseudo-Control Roll Pitch' incorporates:
    //   DiscreteIntegrator: '<S76>/Discrete-Time Integrator y'
    //   MATLAB Function: '<S89>/Desired Roll Pitch'

    rtb_n_dt_o[i] = 0.0F;
    rtb_s_g_dt2_ref[i] = 0.0F;
    for (k = 0; k < 3; k++) {
      // MATLAB Function: '<S89>/Desired Roll Pitch'
      loop_ub = i + 3 * k;
      tmp_0[loop_ub] = 0.0F;

      // MATLAB Function: '<S89>/Desired Roll Pitch' incorporates:
      //   DiscreteIntegrator: '<S76>/Discrete-Time Integrator y'

      db = 3 * k + i;
      rtb_Gain3_b = rtDW.DiscreteTimeIntegratory_DSTAT_p[3 * k];
      tmp_0[loop_ub] = tmp_0[db] + rtb_Gain3_b * dist_remaining_0[3 * i];

      // MATLAB Function: '<S89>/Desired Roll Pitch' incorporates:
      //   DiscreteIntegrator: '<S76>/Discrete-Time Integrator y'

      b_v_size = 3 * i + 1;
      q1_q2 = rtDW.DiscreteTimeIntegratory_DSTAT_p[3 * k + 1];
      tmp_0[loop_ub] = q1_q2 * dist_remaining_0[b_v_size] + tmp_0[db];

      // MATLAB Function: '<S89>/Desired Roll Pitch' incorporates:
      //   DiscreteIntegrator: '<S76>/Discrete-Time Integrator y'

      tmp_5 = 3 * i + 2;
      tmp_6 = rtDW.DiscreteTimeIntegratory_DSTAT_p[3 * k + 2];
      tmp_0[loop_ub] = tmp_6 * dist_remaining_0[tmp_5] + tmp_0[db];

      // MATLAB Function: '<S89>/Desired Roll Pitch'
      tmp_2[loop_ub] = 0.0F;
      tmp_2[loop_ub] = tmp_2[db] + rtb_Gain3_b * tmp_1[3 * i];
      tmp_2[loop_ub] = q1_q2 * tmp_1[b_v_size] + tmp_2[db];
      tmp_2[loop_ub] = tmp_6 * tmp_1[tmp_5] + tmp_2[db];
      rtb_n_dt_o[i] += rtDW.DiscreteTimeIntegratory_DSTAT_p[db] * rtb_n_b_c[k];
      rtb_s_g_dt2_ref[i] += tmp_0[db] * rtb_n_b_dt_p[k];
    }

    rtb_n_dt2[i] = rtb_n_dt_o[i] + rtb_s_g_dt2_ref[i];

    // MATLAB Function: '<S89>/Desired Roll Pitch' incorporates:
    //   DiscreteIntegrator: '<S76>/Discrete-Time Integrator y'

    tmp_3[i] = (rtDW.DiscreteTimeIntegratory_DSTAT_p[i + 6] * q0_q3 +
                (rtDW.DiscreteTimeIntegratory_DSTAT_p[i + 3] * dist_available +
                 rtDW.DiscreteTimeIntegratory_DSTAT_p[i] * rtb_Sqrt)) + (tmp_2[i
      + 6] * rtb_n_b[2] + (tmp_2[i + 3] * rtb_n_b[1] + tmp_2[i] * rtb_n_b[0]));
  }

  // Sum: '<S16>/Add2' incorporates:
  //   DiscreteIntegrator: '<S107>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<S79>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S79>/Discrete-Time Integrator y_dt'
  //   Gain: '<S86>/Gain'
  //   Gain: '<S86>/Gain1'
  //   Gain: '<S86>/Gain2'
  //   Gain: '<S86>/Gain3'
  //   Gain: '<S86>/Gain4'
  //   Gain: '<S86>/Gain5'
  //   Gain: '<S86>/Gain6'
  //   Gain: '<S86>/Gain7'
  //   Gain: '<S86>/Gain8'
  //   Gain: '<S87>/Gain'
  //   MATLAB Function: '<S83>/Reduced Attitude Weighting Factors'
  //   MATLAB Function: '<S89>/Desired Roll Pitch'
  //   MATLAB Function: '<S89>/MATLAB Function'
  //   MATLAB Function: '<S89>/Pseudo-Control Roll Pitch'
  //   MATLAB Function: '<S89>/Simulink Trickster'
  //   Product: '<S86>/Product'
  //   Product: '<S86>/Product1'
  //   Product: '<S86>/Product3'
  //   Product: '<S86>/Product4'
  //   Product: '<S87>/Product'
  //   Product: '<S87>/Product1'
  //   Product: '<S88>/Product'
  //   Sum: '<S16>/Add1'
  //   Sum: '<S84>/error1 1'
  //   Sum: '<S84>/error1 2'
  //   Sum: '<S84>/error1 4'
  //   Sum: '<S84>/error1 5'
  //   Sum: '<S84>/error1 6'
  //   Sum: '<S84>/error1 8'
  //   Sum: '<S84>/error1 9'
  //   Sum: '<S86>/Add'
  //   Sum: '<S86>/Add1'
  //   Sum: '<S86>/Add2'
  //   Sum: '<S86>/Add3'
  //   Sum: '<S87>/Add'
  //   Sum: '<S88>/Add'

  rtb_n_dt[0] = (((((1.0F - rtb_distance) * (rtb_n_dt2_i[0] -
    rtDW.DiscreteTimeIntegratory_DSTAT_k[0]) + rtb_distance * rtb_n_b_dt[1]) *
                   30.6008472F + 135.259491F * rtb_y_dt_cs[1]) + ((1.0F -
    rtb_distance) * (tmp_3[0] - rtDW.DiscreteTimeIntegratory_dt_DSTA[0]) +
    rtb_distance * rtb_Sum2_e[1]) * 1.30769229F) + ((1.0F - rtb_distance) *
    rtb_n_dt2[0] + rtb_distance * rtb_n_b_dt2[1])) - (1.0F - rtb_distance) *
    rtDW.DiscreteTimeIntegratory_dt_DSTA[0];
  rtb_n_dt[1] = (((((1.0F - rtb_distance) * (rtb_n_dt2_i[1] -
    rtDW.DiscreteTimeIntegratory_DSTAT_k[1]) + rtb_distance * -rtb_n_b_dt[0]) *
                   30.6008472F + 135.259491F * -rtb_y_dt_cs[0]) + ((1.0F -
    rtb_distance) * (tmp_3[1] - rtDW.DiscreteTimeIntegratory_dt_DSTA[1]) +
    rtb_distance * -rtb_Sum2_e[0]) * 1.30769229F) + ((1.0F - rtb_distance) *
    rtb_n_dt2[1] + rtb_distance * -rtb_n_b_dt2[0])) - (1.0F - rtb_distance) *
    rtDW.DiscreteTimeIntegratory_dt_DSTA[1];
  rtb_n_dt[2] = ((((rtDW.DiscreteTimeIntegrator_DSTATE_h -
                    rtDW.DiscreteTimeIntegratory_DSTAT_k[2]) * 6.41560555F +
                   12.9844656F * rtb_Gain5) + (rtb_Gain -
    rtDW.DiscreteTimeIntegratory_dt_DSTA[2]) * 0.056647189F) + rtb_Gain) -
    rtDW.DiscreteTimeIntegratory_dt_DSTA[2];

  // Outputs for Enabled SubSystem: '<S2>/Incremental specific thrust' incorporates:
  //   EnablePort: '<S14>/Enable'

  // DiscreteIntegrator: '<S70>/Discrete-Time Integrator y'
  if (rtDW.DiscreteTimeIntegratory_IC_LO_e != 0) {
    rtDW.DiscreteTimeIntegratory_DSTA_lk[0] = rtDW.n_g_des[0];
    rtDW.DiscreteTimeIntegratory_DSTA_lk[1] = rtDW.n_g_des[1];
    rtDW.DiscreteTimeIntegratory_DSTA_lk[2] = rtDW.n_g_des[2];
  }

  rtb_Sum2_e[0] = rtDW.DiscreteTimeIntegratory_DSTA_lk[0];

  // DiscreteIntegrator: '<S70>/Discrete-Time Integrator y_dt'
  rtb_n[0] = rtDW.DiscreteTimeIntegratory_dt_DS_d[0];

  // Gain: '<S70>/omega^2' incorporates:
  //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y_dt'
  //   Gain: '<S70>/2*d//omega'
  //   Sum: '<S70>/Sum2'
  //   Sum: '<S70>/Sum3'

  rtb_Add1[0] = (rtDW.n_g_des[0] - (0.150825381F *
    rtDW.DiscreteTimeIntegratory_dt_DS_d[0] +
    rtDW.DiscreteTimeIntegratory_DSTA_lk[0])) * 175.837341F;

  // DiscreteIntegrator: '<S70>/Discrete-Time Integrator y'
  rtb_Sum2_e[1] = rtDW.DiscreteTimeIntegratory_DSTA_lk[1];

  // DiscreteIntegrator: '<S70>/Discrete-Time Integrator y_dt'
  rtb_n[1] = rtDW.DiscreteTimeIntegratory_dt_DS_d[1];

  // Gain: '<S70>/omega^2' incorporates:
  //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y_dt'
  //   Gain: '<S70>/2*d//omega'
  //   Sum: '<S70>/Sum2'
  //   Sum: '<S70>/Sum3'

  rtb_Add1[1] = (rtDW.n_g_des[1] - (0.150825381F *
    rtDW.DiscreteTimeIntegratory_dt_DS_d[1] +
    rtDW.DiscreteTimeIntegratory_DSTA_lk[1])) * 175.837341F;

  // DiscreteIntegrator: '<S70>/Discrete-Time Integrator y'
  rtb_Sum2_e[2] = rtDW.DiscreteTimeIntegratory_DSTA_lk[2];

  // DiscreteIntegrator: '<S70>/Discrete-Time Integrator y_dt'
  rtb_n[2] = rtDW.DiscreteTimeIntegratory_dt_DS_d[2];

  // Gain: '<S70>/omega^2' incorporates:
  //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y_dt'
  //   Gain: '<S70>/2*d//omega'
  //   Sum: '<S70>/Sum2'
  //   Sum: '<S70>/Sum3'

  rtb_Add1[2] = (rtDW.n_g_des[2] - (0.150825381F *
    rtDW.DiscreteTimeIntegratory_dt_DS_d[2] +
    rtDW.DiscreteTimeIntegratory_DSTA_lk[2])) * 175.837341F;

  // MATLAB Function: '<S70>/n ref norm'
  nrefnorm(rtb_Add1, rtb_n, rtb_Sum2_e, rtb_n_dt_o, rtb_n_dt2_i, rtb_s_g_dt2_ref);

  // MATLAB Function: '<S14>/DCM 2 Lean Vector'
  DCM2LeanVector(rtb_y_i, rtb_Sum2_e);

  // MATLAB Function: '<S14>/desired and measured specific thrust' incorporates:
  //   DiscreteIntegrator: '<S75>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S76>/Discrete-Time Integrator y'

  for (i = 0; i < 3; i++) {
    rtb_Gain3_b = rtDW.DiscreteTimeIntegratory_DSTAT_p[i + 6];
    rtb_n_dt2_i[i] = rtb_Gain3_b * rtDW.DiscreteTimeIntegratory_DSTA_al[2] +
      (rtDW.DiscreteTimeIntegratory_DSTAT_p[i + 3] *
       rtDW.DiscreteTimeIntegratory_DSTA_al[1] +
       rtDW.DiscreteTimeIntegratory_DSTAT_p[i] *
       rtDW.DiscreteTimeIntegratory_DSTA_al[0]);
    rtb_n_dt_o[i] = rtb_Gain3_b * 9.81F;
  }

  rtDW.a_T_meas = rtb_n_dt2_i[2] - rtb_n_dt_o[2];

  // End of MATLAB Function: '<S14>/desired and measured specific thrust'

  // Sum: '<S14>/Add' incorporates:
  //   Abs: '<S14>/Abs'
  //   DotProduct: '<S14>/Dot Product'
  //   Gain: '<S14>/Gain'
  //   Product: '<S14>/Product'

  rtDW.Delta_nu_a_T = std::abs((rtDW.n_g_des[0] * rtb_s_g_dt2_ref[0] +
    rtDW.n_g_des[1] * rtb_s_g_dt2_ref[1]) + rtDW.n_g_des[2] * rtb_s_g_dt2_ref[2])
    * -rtDW.Merge1 - rtDW.a_T_meas;

  // Update for DiscreteIntegrator: '<S70>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTA_lk[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_d[0];

  // Update for DiscreteIntegrator: '<S70>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DS_d[0] += 0.0025F * rtb_Add1[0];

  // Update for DiscreteIntegrator: '<S70>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTA_lk[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_d[1];

  // Update for DiscreteIntegrator: '<S70>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DS_d[1] += 0.0025F * rtb_Add1[1];

  // MATLAB Function: '<S14>/incremental thrust atti correction'
  rtb_Sqrt = (rtb_Sum2_e[0] * rtb_s_g_dt2_ref[0] + rtb_Sum2_e[1] *
              rtb_s_g_dt2_ref[1]) + rtb_Sum2_e[2] * rtb_s_g_dt2_ref[2];

  // Update for DiscreteIntegrator: '<S70>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTA_lk[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_d[2];

  // Update for DiscreteIntegrator: '<S70>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DS_d[2] += 0.0025F * rtb_Add1[2];

  // MATLAB Function: '<S14>/incremental thrust atti correction'
  if (rtb_Sqrt < 0.0F) {
    rtDW.Delta_nu_a_T = 0.0F;
  } else {
    rtDW.Delta_nu_a_T *= rtb_Sqrt;
  }

  // Update for DiscreteIntegrator: '<S70>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LO_e = 0U;

  // End of Outputs for SubSystem: '<S2>/Incremental specific thrust'

  // MinMax: '<S49>/Max' incorporates:
  //   DiscreteIntegrator: '<S54>/Discrete-Time Integrator'

  if (rtDW.DiscreteTimeIntegrator_DSTATE_e[0] > 0.1F) {
    rtb_Switch[0] = rtDW.DiscreteTimeIntegrator_DSTATE_e[0];
  } else {
    rtb_Switch[0] = 0.1F;
  }

  if (rtDW.DiscreteTimeIntegrator_DSTATE_e[1] > 0.1F) {
    rtb_Switch[1] = rtDW.DiscreteTimeIntegrator_DSTATE_e[1];
  } else {
    rtb_Switch[1] = 0.1F;
  }

  if (rtDW.DiscreteTimeIntegrator_DSTATE_e[2] > 0.1F) {
    rtb_Switch[2] = rtDW.DiscreteTimeIntegrator_DSTATE_e[2];
  } else {
    rtb_Switch[2] = 0.1F;
  }

  if (rtDW.DiscreteTimeIntegrator_DSTATE_e[3] > 0.1F) {
    rtb_Switch[3] = rtDW.DiscreteTimeIntegrator_DSTATE_e[3];
  } else {
    rtb_Switch[3] = 0.1F;
  }

  // End of MinMax: '<S49>/Max'

  // MATLAB Function: '<S49>/MATLAB Function'
  rtb_Gain3_b = std::sqrt(5.79275627E-10F * rtb_Switch[0] + 9.95611482E-10F);
  rtb_Add_a[1] = (std::sqrt(5.79275627E-10F * rtb_Switch[1] + 9.95611482E-10F) -
                  3.15533107E-5F) / 4.22296154E-9F;
  rtb_Add_a[2] = (std::sqrt(5.79275627E-10F * rtb_Switch[2] + 9.95611482E-10F) -
                  3.15533107E-5F) / 4.22296154E-9F;
  rtb_Switch[1] = std::sqrt(5.79275627E-10F * rtb_Switch[1] + 9.95611482E-10F);
  rtb_Switch[2] = std::sqrt(5.79275627E-10F * rtb_Switch[2] + 9.95611482E-10F);
  rtb_Switch[1] = 0.068586424F / rtb_Switch[1];
  rtb_Switch[2] = 0.068586424F / rtb_Switch[2];
  memset(&G_omega[0], 0, sizeof(real32_T) << 4U);
  memset(&rtb_G2[0], 0, sizeof(real32_T) << 4U);
  G_omega[0] = 0.068586424F / rtb_Gain3_b;
  G_omega[5] = rtb_Switch[1];
  G_omega[10] = rtb_Switch[2];
  rtb_distance = std::sqrt(5.79275627E-10F * rtb_Switch[3] + 9.95611482E-10F);
  G_omega[15] = 0.068586424F / rtb_distance;
  rtb_G2[0] = (rtb_Gain3_b - 3.15533107E-5F) / 4.22296154E-9F;
  rtb_G2[5] = rtb_Add_a[1];
  rtb_G2[10] = rtb_Add_a[2];
  rtb_G2[15] = (rtb_distance - 3.15533107E-5F) / 4.22296154E-9F;
  for (i = 0; i < 4; i++) {
    for (k = 0; k < 4; k++) {
      loop_ub = k << 2;
      db = i + loop_ub;
      rtb_y_bn[db] = 0.0F;
      b_v_size = loop_ub + i;
      rtb_y_bn[db] = rtb_y_bn[b_v_size] + rtb_G2[loop_ub] *
        rtConstP.MATLABFunction_G10[i];
      rtb_y_bn[db] = rtb_G2[loop_ub + 1] * rtConstP.MATLABFunction_G10[i + 4] +
        rtb_y_bn[b_v_size];
      rtb_y_bn[db] = rtb_G2[loop_ub + 2] * rtConstP.MATLABFunction_G10[i + 8] +
        rtb_y_bn[b_v_size];
      rtb_y_bn[db] = rtb_G2[loop_ub + 3] * rtConstP.MATLABFunction_G10[i + 12] +
        rtb_y_bn[b_v_size];
    }

    for (k = 0; k < 4; k++) {
      loop_ub = k << 2;
      db = i + loop_ub;
      rtb_G1[db] = 0.0F;
      b_v_size = loop_ub + i;
      rtb_G1[db] = rtb_G1[b_v_size] + G_omega[loop_ub] * rtb_y_bn[i];
      rtb_G1[db] = G_omega[loop_ub + 1] * rtb_y_bn[i + 4] + rtb_G1[b_v_size];
      rtb_G1[db] = G_omega[loop_ub + 2] * rtb_y_bn[i + 8] + rtb_G1[b_v_size];
      rtb_G1[db] = G_omega[loop_ub + 3] * rtb_y_bn[i + 12] + rtb_G1[b_v_size];
    }
  }

  for (i = 0; i < 4; i++) {
    for (k = 0; k < 4; k++) {
      loop_ub = i << 2;
      db = k + loop_ub;
      rtb_G2[db] = 0.0F;
      b_v_size = loop_ub + k;
      rtb_G2[db] = rtb_G2[b_v_size] + G_omega[loop_ub] *
        rtConstP.MATLABFunction_G20[k];
      rtb_G2[db] = G_omega[loop_ub + 1] * rtConstP.MATLABFunction_G20[k + 4] +
        rtb_G2[b_v_size];
      rtb_G2[db] = G_omega[loop_ub + 2] * rtConstP.MATLABFunction_G20[k + 8] +
        rtb_G2[b_v_size];
      rtb_G2[db] = G_omega[loop_ub + 3] * rtConstP.MATLABFunction_G20[k + 12] +
        rtb_G2[b_v_size];
    }
  }

  // End of MATLAB Function: '<S49>/MATLAB Function'

  // MATLAB Function: '<S25>/create diag' incorporates:
  //   Delay: '<S29>/Delay'

  memset(&rtb_y_bn[0], 0, sizeof(real32_T) << 4U);
  rtb_y_bn[0] = rtDW.Delay_DSTATE[0];
  rtb_y_bn[5] = rtDW.Delay_DSTATE[1];
  rtb_y_bn[10] = rtDW.Delay_DSTATE[2];
  rtb_y_bn[15] = rtDW.Delay_DSTATE[3];

  // Product: '<S25>/correct G1'
  for (i = 0; i < 4; i++) {
    for (k = 0; k < 4; k++) {
      loop_ub = i << 2;
      db = k + loop_ub;
      G_omega[db] = 0.0F;
      b_v_size = loop_ub + k;
      G_omega[db] = G_omega[b_v_size] + rtb_G1[loop_ub] * rtb_y_bn[k];
      G_omega[db] = rtb_G1[loop_ub + 1] * rtb_y_bn[k + 4] + G_omega[b_v_size];
      G_omega[db] = rtb_G1[loop_ub + 2] * rtb_y_bn[k + 8] + G_omega[b_v_size];
      G_omega[db] = rtb_G1[loop_ub + 3] * rtb_y_bn[k + 12] + G_omega[b_v_size];
    }
  }

  // End of Product: '<S25>/correct G1'

  // Product: '<S25>/correct G2' incorporates:
  //   Delay: '<S30>/Delay1'

  for (i = 0; i < 16; i++) {
    rtb_y_bn[i] = rtDW.Delay1_DSTATE * rtb_G2[i];
  }

  // End of Product: '<S25>/correct G2'

  // MATLAB Function: '<S15>/MATLAB Function'
  DCM2LeanVector(rtb_y_i, rtb_s_g_dt2_ref);

  // MATLAB Function: '<S12>/Control Allocation Vertical Acc Weighting'
  rtb_Sqrt = (rtb_s_g_dt2_ref[0] * rtDW.n_g_des[0] + rtb_s_g_dt2_ref[1] *
              rtDW.n_g_des[1]) + rtb_s_g_dt2_ref[2] * rtDW.n_g_des[2];
  if (rtb_Sqrt < 0.0F) {
    rtb_Sqrt = 0.0F;
  } else {
    rtb_Sqrt = (rtConstP.ControlAllocationVerticalAccWei.W_v[15] - 10.0F) *
      (rtb_Sqrt * rtb_Sqrt) + 10.0F;
  }

  rtb_Switch[3] = rtb_Sqrt - rtConstP.ControlAllocationVerticalAccWei.W_v[15];

  // End of MATLAB Function: '<S12>/Control Allocation Vertical Acc Weighting'

  // MATLAB Function: '<S50>/MATLAB Function2' incorporates:
  //   DiscreteIntegrator: '<S54>/Discrete-Time Integrator'

  rtb_Add_a[0] = 0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_e[0];
  umax[0] = 1.0F - rtDW.DiscreteTimeIntegrator_DSTATE_e[0];
  rtb_Add_a[1] = 0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_e[1];
  umax[1] = 1.0F - rtDW.DiscreteTimeIntegrator_DSTATE_e[1];
  rtb_Add_a[2] = 0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_e[2];
  umax[2] = 1.0F - rtDW.DiscreteTimeIntegrator_DSTATE_e[2];
  rtb_Add_a[3] = 0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_e[3];
  umax[3] = 1.0F - rtDW.DiscreteTimeIntegrator_DSTATE_e[3];
  memset(&rtb_G2[0], 0, sizeof(real32_T) << 4U);
  rtb_G2[0] = 0.0F;
  rtb_W[0] = 0.0F;
  rtb_Switch[0] = ((0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_e[0]) + (1.0F -
    rtDW.DiscreteTimeIntegrator_DSTATE_e[0])) * 0.5F;
  rtb_G2[5] = 0.0F;
  rtb_W[1] = 0.0F;
  rtb_Switch[1] = ((0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_e[1]) + (1.0F -
    rtDW.DiscreteTimeIntegrator_DSTATE_e[1])) * 0.5F;
  rtb_G2[10] = 0.0F;
  rtb_W[2] = 0.0F;
  rtb_Switch[2] = ((0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_e[2]) + (1.0F -
    rtDW.DiscreteTimeIntegrator_DSTATE_e[2])) * 0.5F;
  rtb_G2[15] = rtb_Switch[3];
  rtb_W[3] = 0.0F;
  rtb_Switch[3] = ((0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_e[3]) + (1.0F -
    rtDW.DiscreteTimeIntegrator_DSTATE_e[3])) * 0.5F;

  // Sum: '<S45>/Add1' incorporates:
  //   Switch: '<S12>/Switch'
  //   Switch: '<S12>/Switch1'

  for (i = 0; i < 16; i++) {
    rtb_G1[i] = G_omega[i] + rtb_y_bn[i];
  }

  // End of Sum: '<S45>/Add1'

  // Product: '<S45>/MatrixMultiply2' incorporates:
  //   Switch: '<S12>/Switch1'
  //   UnitDelay: '<S45>/Unit Delay1'

  for (i = 0; i < 4; i++) {
    rtb_Gain3_b = rtb_y_bn[i + 12] * rtDW.UnitDelay1_DSTATE[3] + (rtb_y_bn[i + 8]
      * rtDW.UnitDelay1_DSTATE[2] + (rtb_y_bn[i + 4] * rtDW.UnitDelay1_DSTATE[1]
      + rtb_y_bn[i] * rtDW.UnitDelay1_DSTATE[0]));
    rtb_y_l[i] = rtb_Gain3_b;
  }

  // End of Product: '<S45>/MatrixMultiply2'

  // Sum: '<S45>/Add2'
  rtb_n_dt_0[0] = rtb_n_dt[0] + rtb_y_l[0];
  rtb_n_dt_0[1] = rtb_n_dt[1] + rtb_y_l[1];
  rtb_n_dt_0[2] = rtb_n_dt[2] + rtb_y_l[2];
  rtb_n_dt_0[3] = rtDW.Delta_nu_a_T + rtb_y_l[3];

  // MATLAB Function: '<S50>/MATLAB Function2' incorporates:
  //   DiscreteIntegrator: '<S54>/Discrete-Time Integrator'

  for (i = 0; i < 16; i++) {
    rtb_y_bn[i] = rtConstP.MATLABFunction2_ca.W_v[i] + rtb_G2[i];
  }

  rtb_y_l[0] = 0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_e[0];
  rtb_y_l[1] = 0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_e[1];
  rtb_y_l[2] = 0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_e[2];
  rtb_y_l[3] = 0.1F - rtDW.DiscreteTimeIntegrator_DSTATE_e[3];
  wls_alloc(rtb_G1, rtb_n_dt_0, rtb_Add_a, umax, rtb_y_bn,
            &rtConstP.MATLABFunction2_ca.W_u[0], rtb_y_l, 1000.0F, rtb_Switch,
            rtb_W, 100.0F);

  // Sum: '<S12>/Add6' incorporates:
  //   DiscreteIntegrator: '<S48>/Discrete-Time Integrator y'

  rtb_distance = rtb_Switch[0] + rtDW.DiscreteTimeIntegratory_DSTAT_n[0];

  // Saturate: '<S12>/Saturation3'
  if (rtb_distance > 1.0F) {
    rtb_distance = 1.0F;
  } else {
    if (rtb_distance < 0.1F) {
      rtb_distance = 0.1F;
    }
  }

  // Outport: '<Root>/logs'
  rtY.logs[0] = rtb_distance;

  // Saturate: '<S12>/Saturation3'
  rtb_Add_a[0] = rtb_distance;

  // Sum: '<S12>/Add6' incorporates:
  //   DiscreteIntegrator: '<S48>/Discrete-Time Integrator y'

  rtb_distance = rtb_Switch[1] + rtDW.DiscreteTimeIntegratory_DSTAT_n[1];

  // Saturate: '<S12>/Saturation3'
  if (rtb_distance > 1.0F) {
    rtb_distance = 1.0F;
  } else {
    if (rtb_distance < 0.1F) {
      rtb_distance = 0.1F;
    }
  }

  // Outport: '<Root>/logs'
  rtY.logs[1] = rtb_distance;

  // Saturate: '<S12>/Saturation3'
  rtb_Add_a[1] = rtb_distance;

  // Sum: '<S12>/Add6' incorporates:
  //   DiscreteIntegrator: '<S48>/Discrete-Time Integrator y'

  rtb_distance = rtb_Switch[2] + rtDW.DiscreteTimeIntegratory_DSTAT_n[2];

  // Saturate: '<S12>/Saturation3'
  if (rtb_distance > 1.0F) {
    rtb_distance = 1.0F;
  } else {
    if (rtb_distance < 0.1F) {
      rtb_distance = 0.1F;
    }
  }

  // Outport: '<Root>/logs'
  rtY.logs[2] = rtb_distance;

  // Saturate: '<S12>/Saturation3'
  rtb_Add_a[2] = rtb_distance;

  // Sum: '<S12>/Add6' incorporates:
  //   DiscreteIntegrator: '<S48>/Discrete-Time Integrator y'

  rtb_distance = rtb_Switch[3] + rtDW.DiscreteTimeIntegratory_DSTAT_n[3];

  // Saturate: '<S12>/Saturation3'
  if (rtb_distance > 1.0F) {
    rtb_distance = 1.0F;
  } else {
    if (rtb_distance < 0.1F) {
      rtb_distance = 0.1F;
    }
  }

  // Outport: '<Root>/logs' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion'
  //   DataTypeConversion: '<Root>/Data Type Conversion2'
  //   Delay: '<S29>/Delay'
  //   Delay: '<S30>/Delay1'
  //   DiscreteIntegrator: '<S79>/Discrete-Time Integrator y_dt'
  //   Inport: '<Root>/cmd'
  //   Inport: '<Root>/measure'
  //   MATLAB Function: '<S9>/Auxiliary function to define log_config in generated C++ code'
  //   SignalConversion: '<S126>/TmpSignal ConversionAt SFunction Inport2'

  rtY.logs[3] = rtb_distance;
  for (i = 0; i < 9; i++) {
    rtY.logs[i + 4] = rtDW.Add[i];
  }

  rtY.logs[13] = rtb_y_dt_cs[0];
  rtY.logs[14] = rtb_y_dt_cs[1];
  rtY.logs[30] = rtDW.Delta_nu_a_T;
  rtY.logs[15] = rtDW.s_g_ref_f[0];
  rtY.logs[18] = rtDW.s_g[0];
  rtY.logs[21] = rtDW.s_g_dt[0];
  rtY.logs[24] = rtDW.s_g_dt2[0];
  rtY.logs[27] = rtb_n_dt[0];
  rtY.logs[31] = rtU.measure.omega_Kb[0];
  rtY.logs[16] = rtDW.s_g_ref_f[1];
  rtY.logs[19] = rtDW.s_g[1];
  rtY.logs[22] = rtDW.s_g_dt[1];
  rtY.logs[25] = rtDW.s_g_dt2[1];
  rtY.logs[28] = rtb_n_dt[1];
  rtY.logs[32] = rtU.measure.omega_Kb[1];
  rtY.logs[17] = rtDW.s_g_ref_f[2];
  rtY.logs[20] = rtDW.s_g[2];
  rtY.logs[23] = rtDW.s_g_dt[2];
  rtY.logs[26] = rtDW.s_g_dt2[2];
  rtY.logs[29] = rtb_n_dt[2];
  rtY.logs[33] = rtU.measure.omega_Kb[2];
  rtY.logs[34] = rtU.measure.omega_mot[0];
  rtY.logs[38] = rtU.measure.q_bg[0];
  rtY.logs[35] = rtU.measure.omega_mot[1];
  rtY.logs[39] = rtU.measure.q_bg[1];
  rtY.logs[36] = rtU.measure.omega_mot[2];
  rtY.logs[40] = rtU.measure.q_bg[2];
  rtY.logs[37] = rtU.measure.omega_mot[3];
  rtY.logs[41] = rtU.measure.q_bg[3];
  rtY.logs[42] = rtDW.DiscreteTimeIntegratory_dt_DSTA[0];
  rtY.logs[43] = rtDW.DiscreteTimeIntegratory_dt_DSTA[1];
  rtY.logs[44] = rtDW.DiscreteTimeIntegratory_dt_DSTA[2];
  rtY.logs[45] = rtDW.Delay_DSTATE[0];
  rtY.logs[46] = rtDW.Delay_DSTATE[1];
  rtY.logs[47] = rtDW.Delay_DSTATE[2];
  rtY.logs[48] = rtDW.Delay_DSTATE[3];
  rtY.logs[49] = rtDW.Delay1_DSTATE;
  rtY.logs[50] = rtU.cmd.mission_change;
  rtY.logs[51] = 0.0F;
  for (i = 0; i < 6; i++) {
    rtY.logs[i + 52] = state_vec[i];
  }

  // Sum: '<S76>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S76>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S76>/Discrete-Time Integrator y_dt'
  //   Product: '<S76>/Product2'
  //   Sum: '<S76>/Sum3'

  for (i = 0; i < 9; i++) {
    rtb_Sum2_p[i] -= rtDW.DiscreteTimeIntegratory_dt_DS_b[i] * 0.0263680723F +
      rtDW.DiscreteTimeIntegratory_DSTAT_p[i];
  }

  // End of Sum: '<S76>/Sum2'

  // Outputs for Enabled SubSystem: '<S25>/Adaptive INDI G1 and G2 correction' incorporates:
  //   EnablePort: '<S26>/Enable'

  if (rtDW.AdaptiveINDIG1andG2correction_M) {
    // Disable for Outport: '<S26>/Delta_factors_G1'
    rtDW.Delta_factors_G1[0] = 0.0F;
    rtDW.Delta_factors_G1[1] = 0.0F;
    rtDW.Delta_factors_G1[2] = 0.0F;
    rtDW.Delta_factors_G1[3] = 0.0F;

    // Disable for Outport: '<S26>/Delta_factors_G2'
    rtDW.Delta_factors_G2 = 0.0F;
    rtDW.AdaptiveINDIG1andG2correction_M = false;
  }

  // End of Outputs for SubSystem: '<S25>/Adaptive INDI G1 and G2 correction'

  // Sum: '<S48>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S48>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S48>/Discrete-Time Integrator y_dt'
  //   DiscreteIntegrator: '<S54>/Discrete-Time Integrator'
  //   Product: '<S48>/Product2'
  //   Sum: '<S48>/Sum3'

  umax[0] = rtDW.DiscreteTimeIntegrator_DSTATE_e[0] -
    (rtDW.DiscreteTimeIntegratory_dt_DS_l[0] * 0.0263680723F +
     rtDW.DiscreteTimeIntegratory_DSTAT_n[0]);
  umax[1] = rtDW.DiscreteTimeIntegrator_DSTATE_e[1] -
    (rtDW.DiscreteTimeIntegratory_dt_DS_l[1] * 0.0263680723F +
     rtDW.DiscreteTimeIntegratory_DSTAT_n[1]);
  umax[2] = rtDW.DiscreteTimeIntegrator_DSTATE_e[2] -
    (rtDW.DiscreteTimeIntegratory_dt_DS_l[2] * 0.0263680723F +
     rtDW.DiscreteTimeIntegratory_DSTAT_n[2]);
  umax[3] = rtDW.DiscreteTimeIntegrator_DSTATE_e[3] -
    (rtDW.DiscreteTimeIntegratory_dt_DS_l[3] * 0.0263680723F +
     rtDW.DiscreteTimeIntegratory_DSTAT_n[3]);

  // Lookup_n-D: '<S8>/1-D Lookup Table1' incorporates:
  //   Inport: '<Root>/cmd'

  rtb_Sqrt = look1_iflf_binlx(rtU.cmd.RC_pwm[7], rtConstP.pooled10,
    rtConstP.pooled19, 1U);

  // Lookup_n-D: '<S8>/1-D Lookup Table' incorporates:
  //   Inport: '<Root>/cmd'

  rtb_Gain5 = look1_iflf_binlx(rtU.cmd.RC_pwm[6], rtConstP.pooled10,
    rtConstP.pooled19, 1U);

  // MinMax: '<S8>/Max'
  if (rtb_Gain5 >= rtb_Sqrt) {
    rtb_Gain5 = rtb_Sqrt;
  }

  // End of MinMax: '<S8>/Max'

  // Switch: '<S5>/Switch2' incorporates:
  //   RelationalOperator: '<S5>/LowerRelop1'
  //   Switch: '<S5>/Switch'

  if (rtb_Add_a[1] <= rtb_Gain5) {
    rtb_Gain5 = rtb_Add_a[1];
  }

  // End of Switch: '<S5>/Switch2'

  // Switch: '<S6>/Switch2' incorporates:
  //   RelationalOperator: '<S6>/LowerRelop1'

  if (rtb_distance <= rtb_Sqrt) {
    rtb_Sqrt = rtb_distance;
  }

  // End of Switch: '<S6>/Switch2'

  // Outport: '<Root>/u' incorporates:
  //   Gain: '<Root>/Gain1'
  //   Gain: '<Root>/Gain2'
  //   Gain: '<Root>/Gain3'
  //   Gain: '<Root>/Gain4'

  rtY.u[0] = rtb_Gain5;
  rtY.u[1] = rtb_Sqrt;
  rtY.u[2] = rtb_Add_a[0];
  rtY.u[3] = rtb_Add_a[2];
  rtY.u[4] = 0.0F;
  rtY.u[5] = 0.0F;
  rtY.u[6] = 0.0F;
  rtY.u[7] = 0.0F;

  // Sum: '<S75>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S75>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S75>/Discrete-Time Integrator y_dt'
  //   Inport: '<Root>/measure'
  //   Product: '<S75>/Product2'
  //   Sum: '<Root>/Add'
  //   Sum: '<S75>/Sum3'

  rtb_s_g_dt2_ref[0] = rtU.measure.a_Kg[0] -
    (rtDW.DiscreteTimeIntegratory_dt_D_o1[0] * 0.0263680723F +
     rtDW.DiscreteTimeIntegratory_DSTA_al[0]);

  // Product: '<S77>/Product1' incorporates:
  //   DiscreteIntegrator: '<S77>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S77>/Discrete-Time Integrator y_dt'
  //   Inport: '<Root>/measure'
  //   Product: '<S77>/Product2'
  //   Sum: '<S77>/Sum2'
  //   Sum: '<S77>/Sum3'

  rtb_Add1[0] = (rtU.measure.V_Kg[0] - (rtDW.DiscreteTimeIntegratory_dt_DS_n[0] *
    0.0263680723F + rtDW.DiscreteTimeIntegratory_DSTAT_a[0])) * 5753.11719F;

  // Sum: '<S78>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S78>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S78>/Discrete-Time Integrator y_dt'
  //   Inport: '<Root>/measure'
  //   Product: '<S78>/Product2'
  //   Sum: '<S78>/Sum3'

  rtb_Sum2_e[0] = rtU.measure.s_Kg[0] - (rtDW.DiscreteTimeIntegratory_dt_D_nb[0]
    * 0.0263680723F + rtDW.DiscreteTimeIntegratory_DSTATE[0]);

  // Sum: '<S75>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S75>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S75>/Discrete-Time Integrator y_dt'
  //   Inport: '<Root>/measure'
  //   Product: '<S75>/Product2'
  //   Sum: '<Root>/Add'
  //   Sum: '<S75>/Sum3'

  rtb_s_g_dt2_ref[1] = rtU.measure.a_Kg[1] -
    (rtDW.DiscreteTimeIntegratory_dt_D_o1[1] * 0.0263680723F +
     rtDW.DiscreteTimeIntegratory_DSTA_al[1]);

  // Product: '<S77>/Product1' incorporates:
  //   DiscreteIntegrator: '<S77>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S77>/Discrete-Time Integrator y_dt'
  //   Inport: '<Root>/measure'
  //   Product: '<S77>/Product2'
  //   Sum: '<S77>/Sum2'
  //   Sum: '<S77>/Sum3'

  rtb_Add1[1] = (rtU.measure.V_Kg[1] - (rtDW.DiscreteTimeIntegratory_dt_DS_n[1] *
    0.0263680723F + rtDW.DiscreteTimeIntegratory_DSTAT_a[1])) * 5753.11719F;

  // Sum: '<S78>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S78>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S78>/Discrete-Time Integrator y_dt'
  //   Inport: '<Root>/measure'
  //   Product: '<S78>/Product2'
  //   Sum: '<S78>/Sum3'

  rtb_Sum2_e[1] = rtU.measure.s_Kg[1] - (rtDW.DiscreteTimeIntegratory_dt_D_nb[1]
    * 0.0263680723F + rtDW.DiscreteTimeIntegratory_DSTATE[1]);

  // Sum: '<S75>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S75>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S75>/Discrete-Time Integrator y_dt'
  //   Inport: '<Root>/measure'
  //   Product: '<S75>/Product2'
  //   Sum: '<Root>/Add'
  //   Sum: '<S75>/Sum3'

  rtb_s_g_dt2_ref[2] = (rtU.measure.a_Kg[2] + 9.81F) -
    (rtDW.DiscreteTimeIntegratory_dt_D_o1[2] * 0.0263680723F +
     rtDW.DiscreteTimeIntegratory_DSTA_al[2]);

  // Product: '<S77>/Product1' incorporates:
  //   DiscreteIntegrator: '<S77>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S77>/Discrete-Time Integrator y_dt'
  //   Inport: '<Root>/measure'
  //   Product: '<S77>/Product2'
  //   Sum: '<S77>/Sum2'
  //   Sum: '<S77>/Sum3'

  rtb_Add1[2] = (rtU.measure.V_Kg[2] - (rtDW.DiscreteTimeIntegratory_dt_DS_n[2] *
    0.0263680723F + rtDW.DiscreteTimeIntegratory_DSTAT_a[2])) * 5753.11719F;

  // Sum: '<S78>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S78>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S78>/Discrete-Time Integrator y_dt'
  //   Inport: '<Root>/measure'
  //   Product: '<S78>/Product2'
  //   Sum: '<S78>/Sum3'

  rtb_Sum2_e[2] = rtU.measure.s_Kg[2] - (rtDW.DiscreteTimeIntegratory_dt_D_nb[2]
    * 0.0263680723F + rtDW.DiscreteTimeIntegratory_DSTATE[2]);

  // Saturate: '<S29>/Saturation1'
  if (rtDW.Delta_factors_G1[0] > 0.1F) {
    rtb_Gain3_b = 0.1F;
  } else if (rtDW.Delta_factors_G1[0] < -0.1F) {
    rtb_Gain3_b = -0.1F;
  } else {
    rtb_Gain3_b = rtDW.Delta_factors_G1[0];
  }

  // Saturate: '<S29>/Saturation2' incorporates:
  //   Delay: '<S29>/Delay'
  //   Saturate: '<S29>/Saturation1'
  //   Sum: '<S29>/Add'

  rtb_Gain3_b += rtDW.Delay_DSTATE[0];
  if (rtb_Gain3_b > 4.0F) {
    // Update for Delay: '<S29>/Delay'
    rtDW.Delay_DSTATE[0] = 4.0F;
  } else if (rtb_Gain3_b < 0.25F) {
    // Update for Delay: '<S29>/Delay'
    rtDW.Delay_DSTATE[0] = 0.25F;
  } else {
    // Update for Delay: '<S29>/Delay'
    rtDW.Delay_DSTATE[0] = rtb_Gain3_b;
  }

  // Saturate: '<S29>/Saturation1'
  if (rtDW.Delta_factors_G1[1] > 0.1F) {
    rtb_Gain3_b = 0.1F;
  } else if (rtDW.Delta_factors_G1[1] < -0.1F) {
    rtb_Gain3_b = -0.1F;
  } else {
    rtb_Gain3_b = rtDW.Delta_factors_G1[1];
  }

  // Saturate: '<S29>/Saturation2' incorporates:
  //   Delay: '<S29>/Delay'
  //   Saturate: '<S29>/Saturation1'
  //   Sum: '<S29>/Add'

  rtb_Gain3_b += rtDW.Delay_DSTATE[1];
  if (rtb_Gain3_b > 4.0F) {
    // Update for Delay: '<S29>/Delay'
    rtDW.Delay_DSTATE[1] = 4.0F;
  } else if (rtb_Gain3_b < 0.25F) {
    // Update for Delay: '<S29>/Delay'
    rtDW.Delay_DSTATE[1] = 0.25F;
  } else {
    // Update for Delay: '<S29>/Delay'
    rtDW.Delay_DSTATE[1] = rtb_Gain3_b;
  }

  // Saturate: '<S29>/Saturation1'
  if (rtDW.Delta_factors_G1[2] > 0.1F) {
    rtb_Gain3_b = 0.1F;
  } else if (rtDW.Delta_factors_G1[2] < -0.1F) {
    rtb_Gain3_b = -0.1F;
  } else {
    rtb_Gain3_b = rtDW.Delta_factors_G1[2];
  }

  // Saturate: '<S29>/Saturation2' incorporates:
  //   Delay: '<S29>/Delay'
  //   Saturate: '<S29>/Saturation1'
  //   Sum: '<S29>/Add'

  rtb_Gain3_b += rtDW.Delay_DSTATE[2];
  if (rtb_Gain3_b > 4.0F) {
    // Update for Delay: '<S29>/Delay'
    rtDW.Delay_DSTATE[2] = 4.0F;
  } else if (rtb_Gain3_b < 0.25F) {
    // Update for Delay: '<S29>/Delay'
    rtDW.Delay_DSTATE[2] = 0.25F;
  } else {
    // Update for Delay: '<S29>/Delay'
    rtDW.Delay_DSTATE[2] = rtb_Gain3_b;
  }

  // Saturate: '<S29>/Saturation1'
  if (rtDW.Delta_factors_G1[3] > 0.1F) {
    rtb_Gain3_b = 0.1F;
  } else if (rtDW.Delta_factors_G1[3] < -0.1F) {
    rtb_Gain3_b = -0.1F;
  } else {
    rtb_Gain3_b = rtDW.Delta_factors_G1[3];
  }

  // Saturate: '<S29>/Saturation2' incorporates:
  //   Delay: '<S29>/Delay'
  //   Saturate: '<S29>/Saturation1'
  //   Sum: '<S29>/Add'

  rtb_Gain3_b += rtDW.Delay_DSTATE[3];
  if (rtb_Gain3_b > 4.0F) {
    // Update for Delay: '<S29>/Delay'
    rtDW.Delay_DSTATE[3] = 4.0F;
  } else if (rtb_Gain3_b < 0.25F) {
    // Update for Delay: '<S29>/Delay'
    rtDW.Delay_DSTATE[3] = 0.25F;
  } else {
    // Update for Delay: '<S29>/Delay'
    rtDW.Delay_DSTATE[3] = rtb_Gain3_b;
  }

  // Saturate: '<S30>/Saturation1'
  if (rtDW.Delta_factors_G2 > 0.1F) {
    rtb_Gain3_b = 0.1F;
  } else if (rtDW.Delta_factors_G2 < -0.1F) {
    rtb_Gain3_b = -0.1F;
  } else {
    rtb_Gain3_b = rtDW.Delta_factors_G2;
  }

  // End of Saturate: '<S30>/Saturation1'

  // Sum: '<S30>/Add1' incorporates:
  //   Delay: '<S30>/Delay1'

  rtb_Gain3_b += rtDW.Delay1_DSTATE;

  // Saturate: '<S30>/Saturation3'
  if (rtb_Gain3_b > 4.0F) {
    // Update for Delay: '<S30>/Delay1'
    rtDW.Delay1_DSTATE = 4.0F;
  } else if (rtb_Gain3_b < 0.25F) {
    // Update for Delay: '<S30>/Delay1'
    rtDW.Delay1_DSTATE = 0.25F;
  } else {
    // Update for Delay: '<S30>/Delay1'
    rtDW.Delay1_DSTATE = rtb_Gain3_b;
  }

  // End of Saturate: '<S30>/Saturation3'

  // Update for UnitDelay: '<S20>/Unit Delay' incorporates:
  //   MATLAB Function: '<S13>/pick if traj is valid'

  rtDW.UnitDelay_DSTATE = state_vec[4];

  // Update for DiscreteIntegrator: '<S78>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LOAD = 0U;

  // Update for DiscreteIntegrator: '<S77>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LO_n = 0U;

  // Update for DiscreteIntegrator: '<S7>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_DSTATE += 0.0025F;
  if (rtb_uDLookupTable > 0.0F) {
    rtDW.DiscreteTimeIntegrator_PrevRese = 1;
  } else if (rtb_uDLookupTable < 0.0F) {
    rtDW.DiscreteTimeIntegrator_PrevRese = -1;
  } else if (rtb_uDLookupTable == 0.0F) {
    rtDW.DiscreteTimeIntegrator_PrevRese = 0;
  } else {
    rtDW.DiscreteTimeIntegrator_PrevRese = 2;
  }

  // End of Update for DiscreteIntegrator: '<S7>/Discrete-Time Integrator'

  // Update for UnitDelay: '<S13>/Unit Delay' incorporates:
  //   MATLAB Function: '<S13>/pick if traj is valid'

  rtDW.UnitDelay_DSTATE_c = state_vec[4];

  // Update for DiscreteIntegrator: '<S78>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S78>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTATE[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_D_nb[0];

  // Update for DiscreteIntegrator: '<S77>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S77>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_a[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_n[0];

  // Update for DiscreteIntegrator: '<S75>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S75>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTA_al[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_D_o1[0];

  // Update for DiscreteIntegrator: '<S79>/Discrete-Time Integrator y_dt' incorporates:
  //   DiscreteIntegrator: '<S79>/Discrete-Time Integrator y'
  //   Inport: '<Root>/measure'
  //   Product: '<S79>/Product1'
  //   Product: '<S79>/Product2'
  //   Sum: '<S79>/Sum2'
  //   Sum: '<S79>/Sum3'

  rtDW.DiscreteTimeIntegratory_dt_DSTA[0] += (rtU.measure.omega_Kb[0] -
    (rtDW.DiscreteTimeIntegratory_dt_DSTA[0] * 0.0263680723F +
     rtDW.DiscreteTimeIntegratory_DSTAT_k[0])) * 5753.11719F * 0.0025F;

  // Update for DiscreteIntegrator: '<S78>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S78>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTATE[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_D_nb[1];

  // Update for DiscreteIntegrator: '<S77>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S77>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_a[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_n[1];

  // Update for DiscreteIntegrator: '<S75>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S75>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTA_al[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_D_o1[1];

  // Update for DiscreteIntegrator: '<S79>/Discrete-Time Integrator y_dt' incorporates:
  //   DiscreteIntegrator: '<S79>/Discrete-Time Integrator y'
  //   Inport: '<Root>/measure'
  //   Product: '<S79>/Product1'
  //   Product: '<S79>/Product2'
  //   Sum: '<S79>/Sum2'
  //   Sum: '<S79>/Sum3'

  rtDW.DiscreteTimeIntegratory_dt_DSTA[1] += (rtU.measure.omega_Kb[1] -
    (rtDW.DiscreteTimeIntegratory_dt_DSTA[1] * 0.0263680723F +
     rtDW.DiscreteTimeIntegratory_DSTAT_k[1])) * 5753.11719F * 0.0025F;

  // Update for DiscreteIntegrator: '<S78>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S78>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTATE[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_D_nb[2];

  // Update for DiscreteIntegrator: '<S77>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S77>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_a[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_n[2];

  // Update for DiscreteIntegrator: '<S75>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S75>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTA_al[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_D_o1[2];

  // Update for DiscreteIntegrator: '<S79>/Discrete-Time Integrator y_dt' incorporates:
  //   DiscreteIntegrator: '<S79>/Discrete-Time Integrator y'
  //   Inport: '<Root>/measure'
  //   Product: '<S79>/Product1'
  //   Product: '<S79>/Product2'
  //   Sum: '<S79>/Sum2'
  //   Sum: '<S79>/Sum3'

  rtDW.DiscreteTimeIntegratory_dt_DSTA[2] += (rtU.measure.omega_Kb[2] -
    (rtDW.DiscreteTimeIntegratory_dt_DSTA[2] * 0.0263680723F +
     rtDW.DiscreteTimeIntegratory_DSTAT_k[2])) * 5753.11719F * 0.0025F;

  // Update for DiscreteIntegrator: '<S76>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LO_b = 0U;

  // Update for DiscreteIntegrator: '<S102>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LO_i = 0U;

  // Update for DiscreteIntegrator: '<S103>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LO_a = 0U;

  // Update for DiscreteIntegrator: '<S79>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LO_d = 0U;

  // Update for DiscreteIntegrator: '<S102>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S102>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_m[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_o[0];

  // Update for DiscreteIntegrator: '<S102>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DS_o[0] += 0.0025F * rtb_y_dt_g[0];

  // Update for DiscreteIntegrator: '<S103>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S103>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_e[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_j[0];

  // Update for DiscreteIntegrator: '<S103>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DS_j[0] += 0.0025F * rtb_omega2_f[0];

  // Update for DiscreteIntegrator: '<S79>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_DSTAT_k[0] += 0.0025F * rtb_y_c[0];

  // Update for DiscreteIntegrator: '<S102>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S102>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_m[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_o[1];

  // Update for DiscreteIntegrator: '<S102>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DS_o[1] += 0.0025F * rtb_y_dt_g[1];

  // Update for DiscreteIntegrator: '<S103>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S103>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_e[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_j[1];

  // Update for DiscreteIntegrator: '<S103>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DS_j[1] += 0.0025F * rtb_omega2_f[1];

  // Update for DiscreteIntegrator: '<S79>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_DSTAT_k[1] += 0.0025F * rtb_y_c[1];

  // Update for DiscreteIntegrator: '<S102>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S102>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_m[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_o[2];

  // Update for DiscreteIntegrator: '<S102>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DS_o[2] += 0.0025F * q1_q3;

  // Update for DiscreteIntegrator: '<S103>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S103>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_e[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_j[2];

  // Update for DiscreteIntegrator: '<S103>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DS_j[2] += 0.0025F * rtb_omega2_f[2];

  // Update for DiscreteIntegrator: '<S79>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_DSTAT_k[2] += 0.0025F * rtb_y_c[2];

  // Update for DiscreteIntegrator: '<S107>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_DSTATE_h += 0.0025F * rtb_Gain;

  // Update for DiscreteIntegrator: '<S89>/Discrete-Time Integrator2'
  rtDW.DiscreteTimeIntegrator2_IC_LOAD = 0U;
  rtDW.DiscreteTimeIntegrator2_DSTATE += 0.0025F * dist_remaining;

  // Update for DiscreteIntegrator: '<S54>/Discrete-Time Integrator' incorporates:
  //   Gain: '<S54>/1//T'
  //   Sum: '<S54>/Sum2'

  rtDW.DiscreteTimeIntegrator_DSTATE_e[0] += (rtb_Add_a[0] -
    rtDW.DiscreteTimeIntegrator_DSTATE_e[0]) * 31.6038761F * 0.0025F;

  // Update for UnitDelay: '<S45>/Unit Delay1'
  rtDW.UnitDelay1_DSTATE[0] = rtb_Switch[0];

  // Update for DiscreteIntegrator: '<S48>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S48>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_n[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_l[0];

  // Update for DiscreteIntegrator: '<S48>/Discrete-Time Integrator y_dt' incorporates:
  //   Product: '<S48>/Product1'

  rtDW.DiscreteTimeIntegratory_dt_DS_l[0] += umax[0] * 5753.11719F * 0.0025F;

  // Update for DiscreteIntegrator: '<S54>/Discrete-Time Integrator' incorporates:
  //   Gain: '<S54>/1//T'
  //   Sum: '<S54>/Sum2'

  rtDW.DiscreteTimeIntegrator_DSTATE_e[1] += (rtb_Add_a[1] -
    rtDW.DiscreteTimeIntegrator_DSTATE_e[1]) * 31.6038761F * 0.0025F;

  // Update for UnitDelay: '<S45>/Unit Delay1'
  rtDW.UnitDelay1_DSTATE[1] = rtb_Switch[1];

  // Update for DiscreteIntegrator: '<S48>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S48>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_n[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_l[1];

  // Update for DiscreteIntegrator: '<S48>/Discrete-Time Integrator y_dt' incorporates:
  //   Product: '<S48>/Product1'

  rtDW.DiscreteTimeIntegratory_dt_DS_l[1] += umax[1] * 5753.11719F * 0.0025F;

  // Update for DiscreteIntegrator: '<S54>/Discrete-Time Integrator' incorporates:
  //   Gain: '<S54>/1//T'
  //   Sum: '<S54>/Sum2'

  rtDW.DiscreteTimeIntegrator_DSTATE_e[2] += (rtb_Add_a[2] -
    rtDW.DiscreteTimeIntegrator_DSTATE_e[2]) * 31.6038761F * 0.0025F;

  // Update for UnitDelay: '<S45>/Unit Delay1'
  rtDW.UnitDelay1_DSTATE[2] = rtb_Switch[2];

  // Update for DiscreteIntegrator: '<S48>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S48>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_n[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_l[2];

  // Update for DiscreteIntegrator: '<S48>/Discrete-Time Integrator y_dt' incorporates:
  //   Product: '<S48>/Product1'

  rtDW.DiscreteTimeIntegratory_dt_DS_l[2] += umax[2] * 5753.11719F * 0.0025F;

  // Update for DiscreteIntegrator: '<S54>/Discrete-Time Integrator' incorporates:
  //   Gain: '<S54>/1//T'
  //   Sum: '<S54>/Sum2'

  rtDW.DiscreteTimeIntegrator_DSTATE_e[3] += (rtb_distance -
    rtDW.DiscreteTimeIntegrator_DSTATE_e[3]) * 31.6038761F * 0.0025F;

  // Update for UnitDelay: '<S45>/Unit Delay1'
  rtDW.UnitDelay1_DSTATE[3] = rtb_Switch[3];

  // Update for DiscreteIntegrator: '<S48>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S48>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_n[3] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_l[3];

  // Update for DiscreteIntegrator: '<S48>/Discrete-Time Integrator y_dt' incorporates:
  //   Product: '<S48>/Product1'

  rtDW.DiscreteTimeIntegratory_dt_DS_l[3] += umax[3] * 5753.11719F * 0.0025F;
  for (i = 0; i < 9; i++) {
    // Update for DiscreteIntegrator: '<S76>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S76>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTAT_p[i] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_b[i];

    // Update for DiscreteIntegrator: '<S76>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S76>/Product1'

    rtDW.DiscreteTimeIntegratory_dt_DS_b[i] += rtb_Sum2_p[i] * 5753.11719F *
      0.0025F;
  }

  // Update for DiscreteIntegrator: '<S75>/Discrete-Time Integrator y_dt' incorporates:
  //   Product: '<S75>/Product1'

  rtDW.DiscreteTimeIntegratory_dt_D_o1[0] += rtb_s_g_dt2_ref[0] * 5753.11719F *
    0.0025F;

  // Update for DiscreteIntegrator: '<S77>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DS_n[0] += 0.0025F * rtb_Add1[0];

  // Update for DiscreteIntegrator: '<S78>/Discrete-Time Integrator y_dt' incorporates:
  //   Product: '<S78>/Product1'

  rtDW.DiscreteTimeIntegratory_dt_D_nb[0] += rtb_Sum2_e[0] * 5753.11719F *
    0.0025F;

  // Update for DiscreteIntegrator: '<S75>/Discrete-Time Integrator y_dt' incorporates:
  //   Product: '<S75>/Product1'

  rtDW.DiscreteTimeIntegratory_dt_D_o1[1] += rtb_s_g_dt2_ref[1] * 5753.11719F *
    0.0025F;

  // Update for DiscreteIntegrator: '<S77>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DS_n[1] += 0.0025F * rtb_Add1[1];

  // Update for DiscreteIntegrator: '<S78>/Discrete-Time Integrator y_dt' incorporates:
  //   Product: '<S78>/Product1'

  rtDW.DiscreteTimeIntegratory_dt_D_nb[1] += rtb_Sum2_e[1] * 5753.11719F *
    0.0025F;

  // Update for DiscreteIntegrator: '<S75>/Discrete-Time Integrator y_dt' incorporates:
  //   Product: '<S75>/Product1'

  rtDW.DiscreteTimeIntegratory_dt_D_o1[2] += rtb_s_g_dt2_ref[2] * 5753.11719F *
    0.0025F;

  // Update for DiscreteIntegrator: '<S77>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DS_n[2] += 0.0025F * rtb_Add1[2];

  // Update for DiscreteIntegrator: '<S78>/Discrete-Time Integrator y_dt' incorporates:
  //   Product: '<S78>/Product1'

  rtDW.DiscreteTimeIntegratory_dt_D_nb[2] += rtb_Sum2_e[2] * 5753.11719F *
    0.0025F;
}

// Model initialize function
void MatlabControllerClass::initialize()
{
  // InitializeConditions for Delay: '<S29>/Delay'
  rtDW.Delay_DSTATE[0] = 1.0F;
  rtDW.Delay_DSTATE[1] = 1.0F;
  rtDW.Delay_DSTATE[2] = 1.0F;
  rtDW.Delay_DSTATE[3] = 1.0F;

  // InitializeConditions for Delay: '<S30>/Delay1'
  rtDW.Delay1_DSTATE = 1.0F;

  // InitializeConditions for DiscreteIntegrator: '<S78>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LOAD = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S77>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LO_n = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S7>/Discrete-Time Integrator' 
  rtDW.DiscreteTimeIntegrator_PrevRese = 2;

  // InitializeConditions for DiscreteIntegrator: '<S76>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LO_b = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S102>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LO_i = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S103>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LO_a = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S79>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LO_d = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S89>/Discrete-Time Integrator2' 
  rtDW.DiscreteTimeIntegrator2_IC_LOAD = 1U;

  // SystemInitialize for Iterator SubSystem: '<S13>/Trajectory from Waypoints'
  // InitializeConditions for Memory: '<S56>/TrajMemory'
  rtDW.TrajMemory_PreviousInput = rtConstP.pooled3;

  // End of SystemInitialize for SubSystem: '<S13>/Trajectory from Waypoints'

  // SystemInitialize for Enabled SubSystem: '<S2>/NDI position controller for copters reference model' 
  // InitializeConditions for DiscreteIntegrator: '<S112>/Discrete-Time Integrator' 
  rtDW.DiscreteTimeIntegrator_IC_LOADI = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S17>/Discrete-Time Integrator' 
  rtDW.DiscreteTimeIntegrator_IC_LOA_o = 1U;

  // End of SystemInitialize for SubSystem: '<S2>/NDI position controller for copters reference model' 

  // SystemInitialize for Enabled SubSystem: '<S2>/NDI position controller for copters with reference input' 
  // InitializeConditions for DiscreteIntegrator: '<S115>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_L_b2 = 1U;

  // End of SystemInitialize for SubSystem: '<S2>/NDI position controller for copters with reference input' 

  // SystemInitialize for Enabled SubSystem: '<S25>/Copter Random Excitation'
  // InitializeConditions for RandomNumber: '<S37>/White Noise'
  rtDW.RandSeed[0] = 1529675776U;
  rtDW.NextOutput[0] = rt_nrand_Upu32_Yd_f_pw(&rtDW.RandSeed[0]);
  rtDW.RandSeed[1] = 1529741312U;
  rtDW.NextOutput[1] = rt_nrand_Upu32_Yd_f_pw(&rtDW.RandSeed[1]);
  rtDW.RandSeed[2] = 1529806848U;
  rtDW.NextOutput[2] = rt_nrand_Upu32_Yd_f_pw(&rtDW.RandSeed[2]);
  rtDW.RandSeed[3] = 1529872384U;
  rtDW.NextOutput[3] = rt_nrand_Upu32_Yd_f_pw(&rtDW.RandSeed[3]);

  // End of SystemInitialize for SubSystem: '<S25>/Copter Random Excitation'

  // SystemInitialize for Enabled SubSystem: '<S2>/Incremental specific thrust'
  // InitializeConditions for DiscreteIntegrator: '<S70>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LO_e = 1U;

  // End of SystemInitialize for SubSystem: '<S2>/Incremental specific thrust'

  // SystemInitialize for Enabled SubSystem: '<S25>/INDI Inversion Check'
  // InitializeConditions for DiscreteIntegrator: '<S43>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_L_eh = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S42>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_L_ir = 1U;

  // End of SystemInitialize for SubSystem: '<S25>/INDI Inversion Check'
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
