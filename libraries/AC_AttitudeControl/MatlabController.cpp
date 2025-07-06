//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: MatlabController.cpp
//
// Code generated for Simulink model 'ArduCopter_Fast_Descent'.
//
// Model version                  : 1.476
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Sun Jul  6 13:12:46 2025
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM 7
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#include "MatlabController.h"

// user code (top of source file)
#include "MatlabControllerParams.cpp"

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
  }, { 6U,
    { 119U, 112U, 117U, 102U, 109U, 1U, 114U, 108U, 108U, 112U, 116U, 99U, 121U,
      97U, 119U, 116U, 104U, 114U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
      0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U },

    { 77U, 76U, 53U, 0U }
  } } ;

extern real_T rt_urand_Upu32_Yd_f_pw(uint32_T *u);
extern real_T rt_nrand_Upu32_Yd_f_pw(uint32_T *u);
extern real32_T rt_hypotf(real32_T u0, real32_T u1);
static void Throttlefinished(boolean_T rtu_Enable, real32_T *rty_throttle);
static void DCM2LeanVector(const real32_T rtu_M_bg[9], real32_T rty_n_g[3]);
static void nrefnorm(const real32_T rtu_y[3], const real32_T rtu_y_dt[3], const
                     real32_T rtu_y_dt2[3], real32_T rty_n[3], real32_T
                     rty_n_dt[3], real32_T rty_n_dt2[3]);
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

//
// Output and update for enable system:
//    '<S11>/Throttle finished'
//    '<S11>/Throttle slowing'
//
static void Throttlefinished(boolean_T rtu_Enable, real32_T *rty_throttle)
{
  // Outputs for Enabled SubSystem: '<S11>/Throttle finished' incorporates:
  //   EnablePort: '<S24>/Enable'

  if (rtu_Enable) {
    // SignalConversion: '<S24>/OutportBuffer_InsertedFor_throttle_at_inport_0' incorporates:
    //   Constant: '<S24>/throttle finished'

    *rty_throttle = 0.0F;
  }

  // End of Outputs for SubSystem: '<S11>/Throttle finished'
}

//
// Output and update for atomic system:
//    '<S40>/DCM 2 Lean Vector'
//    '<S41>/MATLAB Function'
//
static void DCM2LeanVector(const real32_T rtu_M_bg[9], real32_T rty_n_g[3])
{
  int32_T i;
  for (i = 0; i < 3; i++) {
    rty_n_g[i] = 0.0F;
    rty_n_g[i] += -rtu_M_bg[3 * i + 2];
  }
}

// Function for MATLAB Function: '<S83>/n ref norm'
static void leanVectorNormDeriv2(const real32_T nn[3], const real32_T nn_dt[3],
  const real32_T nn_dt2[3], real_T n_dt2[3])
{
  real32_T xyz;
  real32_T x;
  real32_T c;
  real32_T y;
  real32_T c_tmp;
  real32_T a_tmp;
  real32_T xyz_tmp;
  real32_T xyz_tmp_0;
  real32_T xyz_tmp_1;
  real32_T a_tmp_tmp;
  real32_T a_tmp_tmp_0;
  real32_T a_tmp_tmp_1;
  real32_T n_dt2_tmp;
  real32_T n_dt2_tmp_0;
  real32_T n_dt2_tmp_1;
  real32_T n_dt2_tmp_2;
  real32_T n_dt2_tmp_3;
  real32_T n_dt2_tmp_4;
  xyz_tmp = nn[1] * nn[1];
  xyz_tmp_0 = nn[0] * nn[0];
  xyz_tmp_1 = nn[2] * nn[2];
  xyz = (xyz_tmp_0 + xyz_tmp) + xyz_tmp_1;
  x = std::sqrt(xyz);
  if (std::abs(x) < 2.22044605E-16F) {
    x = 2.22044605E-16F;
  }

  c_tmp = std::pow(xyz, 1.5F);
  c = c_tmp;
  if (std::abs(c_tmp) < 2.22044605E-16F) {
    c = 2.22044605E-16F;
  }

  a_tmp_tmp = 2.0F * nn[1] * nn_dt[1];
  a_tmp_tmp_0 = 2.0F * nn[0] * nn_dt[0];
  a_tmp_tmp_1 = 2.0F * nn[2] * nn_dt[2];
  a_tmp = (a_tmp_tmp_0 + a_tmp_tmp) + a_tmp_tmp_1;
  y = 4.0F * std::pow(xyz, 2.5F);
  if (std::abs(y) < 2.22044605E-16F) {
    y = 2.22044605E-16F;
  }

  xyz = 2.0F * c_tmp;
  if (std::abs(xyz) < 2.22044605E-16F) {
    xyz = 2.22044605E-16F;
  }

  n_dt2_tmp = 2.0F * nn[1] * nn_dt2[1];
  n_dt2_tmp_0 = nn_dt[1] * nn_dt[1] * 2.0F;
  n_dt2_tmp_1 = 2.0F * nn[0] * nn_dt2[0];
  n_dt2_tmp_2 = nn_dt[0] * nn_dt[0] * 2.0F;
  n_dt2_tmp_3 = 2.0F * nn[2] * nn_dt2[2];
  n_dt2_tmp_4 = nn_dt[2] * nn_dt[2] * 2.0F;
  n_dt2[0] = (a_tmp * a_tmp * 3.0F / y - (((((n_dt2_tmp_1 + n_dt2_tmp_2) +
    n_dt2_tmp) + n_dt2_tmp_0) + n_dt2_tmp_3) + n_dt2_tmp_4) / xyz) * nn[0] +
    (nn_dt2[0] / x - a_tmp * nn_dt[0] / c);
  xyz = (xyz_tmp + xyz_tmp_0) + xyz_tmp_1;
  x = std::sqrt(xyz);
  if (std::abs(x) < 2.22044605E-16F) {
    x = 2.22044605E-16F;
  }

  c_tmp = std::pow(xyz, 1.5F);
  c = c_tmp;
  if (std::abs(c_tmp) < 2.22044605E-16F) {
    c = 2.22044605E-16F;
  }

  a_tmp = (a_tmp_tmp + a_tmp_tmp_0) + a_tmp_tmp_1;
  y = 4.0F * std::pow(xyz, 2.5F);
  if (std::abs(y) < 2.22044605E-16F) {
    y = 2.22044605E-16F;
  }

  xyz = 2.0F * c_tmp;
  if (std::abs(xyz) < 2.22044605E-16F) {
    xyz = 2.22044605E-16F;
  }

  n_dt2[1] = (a_tmp * a_tmp * 3.0F / y - (((((n_dt2_tmp + n_dt2_tmp_0) +
    n_dt2_tmp_1) + n_dt2_tmp_2) + n_dt2_tmp_3) + n_dt2_tmp_4) / xyz) * nn[1] +
    (nn_dt2[1] / x - a_tmp * nn_dt[1] / c);
  xyz = (xyz_tmp_1 + xyz_tmp_0) + xyz_tmp;
  x = std::sqrt(xyz);
  if (std::abs(x) < 2.22044605E-16F) {
    x = 2.22044605E-16F;
  }

  c_tmp = std::pow(xyz, 1.5F);
  c = c_tmp;
  if (std::abs(c_tmp) < 2.22044605E-16F) {
    c = 2.22044605E-16F;
  }

  a_tmp = (a_tmp_tmp_1 + a_tmp_tmp_0) + a_tmp_tmp;
  y = 4.0F * std::pow(xyz, 2.5F);
  if (std::abs(y) < 2.22044605E-16F) {
    y = 2.22044605E-16F;
  }

  xyz = 2.0F * c_tmp;
  if (std::abs(xyz) < 2.22044605E-16F) {
    xyz = 2.22044605E-16F;
  }

  n_dt2[2] = (a_tmp * a_tmp * 3.0F / y - (((((n_dt2_tmp_3 + n_dt2_tmp_4) +
    n_dt2_tmp_1) + n_dt2_tmp_2) + n_dt2_tmp) + n_dt2_tmp_0) / xyz) * nn[2] +
    (nn_dt2[2] / x - a_tmp * nn_dt[2] / c);
}

//
// Output and update for atomic system:
//    '<S83>/n ref norm'
//    '<S117>/n ref norm'
//    '<S118>/n ref norm'
//
static void nrefnorm(const real32_T rtu_y[3], const real32_T rtu_y_dt[3], const
                     real32_T rtu_y_dt2[3], real32_T rty_n[3], real32_T
                     rty_n_dt[3], real32_T rty_n_dt2[3])
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
//    '<S94>/DCM to quaternions'
//    '<S99>/DCM to quaternions'
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

// Function for MATLAB Function: '<S94>/Quaternion Reduced'
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
//    '<S94>/Quaternion Reduced'
//    '<S99>/Quaternion Reduced'
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
//    '<S98>/wrap angle'
//    '<S98>/wrap angle1'
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
//    '<S103>/Lean Vector Derivative Trafo'
//    '<S103>/Lean Vector Derivative Trafo Delay'
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

// Function for MATLAB Function: '<S49>/Avoid zero speed'
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

// Function for MATLAB Function: '<S49>/WpNav Matching'
void MatlabControllerClass::wpnavCircSeg(const real32_T waypoints3x3[9],
  real32_T wp_radius, real32_T *circ_seg_r, real32_T circ_seg_center[3],
  real32_T circ_seg_n[3], real32_T *circ_seg_angle, real32_T circ_seg_start[3],
  real32_T circ_seg_end[3], real32_T *circ_seg_wp_rad, real32_T circ_seg_wp[3])
{
  real32_T dist1;
  real32_T absxk;
  int32_T exponent;
  real32_T dist1_tmp;
  real32_T dist2_tmp;
  circ_seg_start[0] = waypoints3x3[3] - waypoints3x3[0];
  circ_seg_end[0] = waypoints3x3[6] - waypoints3x3[3];
  circ_seg_start[1] = waypoints3x3[4] - waypoints3x3[1];
  circ_seg_end[1] = waypoints3x3[7] - waypoints3x3[4];
  circ_seg_start[2] = waypoints3x3[5] - waypoints3x3[2];
  circ_seg_end[2] = waypoints3x3[8] - waypoints3x3[5];
  dist1_tmp = norm(circ_seg_start);
  dist2_tmp = norm(circ_seg_end);
  circ_seg_n[1] = dist1_tmp / 2.0F;
  circ_seg_n[2] = dist2_tmp / 2.0F;
  *circ_seg_wp_rad = wp_radius;
  if (wp_radius > circ_seg_n[1]) {
    *circ_seg_wp_rad = circ_seg_n[1];
  }

  if (*circ_seg_wp_rad > circ_seg_n[2]) {
    *circ_seg_wp_rad = circ_seg_n[2];
  }

  dist1 = dist1_tmp * dist2_tmp;
  if (std::abs(dist1) < 2.22044605E-16F) {
    dist1 = 2.22044605E-16F;
  }

  absxk = ((circ_seg_start[0] * circ_seg_end[0] + circ_seg_start[1] *
            circ_seg_end[1]) + circ_seg_start[2] * circ_seg_end[2]) / dist1;
  if (1.0F <= absxk) {
    absxk = 1.0F;
  }

  if (-1.0F >= absxk) {
    absxk = -1.0F;
  }

  absxk = std::acos(absxk);
  dist1 = std::abs(absxk);
  dist1 -= std::floor(dist1 / 6.28318548F) * 6.28318548F;
  if (absxk >= 0.0F) {
    *circ_seg_angle = dist1;
  } else {
    *circ_seg_angle = 6.28318548F - dist1;
  }

  absxk = std::abs(*circ_seg_angle);
  if (absxk <= 1.17549435E-38F) {
    absxk = 1.4013E-45F;
  } else {
    std::frexp(absxk, &exponent);
    absxk = std::ldexp(1.0F, exponent - 24);
  }

  if (*circ_seg_angle < 100.0F * absxk) {
    *circ_seg_wp_rad = 0.0F;
  }

  dist1 = std::cos(*circ_seg_angle / 2.0F);
  absxk = 2.0F * *circ_seg_wp_rad * dist1 / 2.0F;
  dist1 = 1.0F - dist1 * dist1;
  if (std::abs(dist1) < 2.22044605E-16F) {
    dist1 = 2.22044605E-16F;
  }

  absxk = absxk * absxk / dist1;
  if (0.0F >= absxk) {
    absxk = 0.0F;
  }

  *circ_seg_r = std::sqrt(absxk);
  circ_seg_n[0] = circ_seg_start[1] * circ_seg_end[2] - circ_seg_start[2] *
    circ_seg_end[1];
  circ_seg_n[1] = circ_seg_start[2] * circ_seg_end[0] - circ_seg_start[0] *
    circ_seg_end[2];
  circ_seg_n[2] = circ_seg_start[0] * circ_seg_end[1] - circ_seg_start[1] *
    circ_seg_end[0];
  absxk = norm(circ_seg_n);
  if (std::abs(absxk) < 2.22044605E-16F) {
    absxk = 2.22044605E-16F;
  }

  circ_seg_n[0] /= absxk;
  circ_seg_n[1] /= absxk;
  circ_seg_n[2] /= absxk;
  if (*circ_seg_r > 100000.0F) {
    *circ_seg_r = 100000.0F;
    circ_seg_n[0] = 0.0F;
    circ_seg_n[1] = 0.0F;
    circ_seg_n[2] = 1.0F;
  }

  absxk = dist1_tmp;
  if (std::abs(dist1_tmp) < 2.22044605E-16F) {
    absxk = 2.22044605E-16F;
  }

  circ_seg_start[0] /= absxk;
  circ_seg_start[1] /= absxk;
  dist1_tmp = circ_seg_start[2] / absxk;
  absxk = dist2_tmp;
  if (std::abs(dist2_tmp) < 2.22044605E-16F) {
    absxk = 2.22044605E-16F;
  }

  circ_seg_center[0] = circ_seg_n[1] * dist1_tmp - circ_seg_n[2] *
    circ_seg_start[1];
  circ_seg_center[1] = circ_seg_n[2] * circ_seg_start[0] - circ_seg_n[0] *
    dist1_tmp;
  circ_seg_center[2] = circ_seg_n[0] * circ_seg_start[1] - circ_seg_n[1] *
    circ_seg_start[0];
  dist1 = norm(circ_seg_center);
  if (std::abs(dist1) < 2.22044605E-16F) {
    dist1 = 2.22044605E-16F;
  }

  dist2_tmp = waypoints3x3[3] - *circ_seg_wp_rad * circ_seg_start[0];
  circ_seg_wp[0] = waypoints3x3[3];
  circ_seg_start[0] = dist2_tmp;
  circ_seg_center[0] = circ_seg_center[0] / dist1 * *circ_seg_r + dist2_tmp;
  circ_seg_end[0] = circ_seg_end[0] / absxk * *circ_seg_wp_rad + waypoints3x3[3];
  dist2_tmp = waypoints3x3[4] - *circ_seg_wp_rad * circ_seg_start[1];
  circ_seg_wp[1] = waypoints3x3[4];
  circ_seg_start[1] = dist2_tmp;
  circ_seg_center[1] = circ_seg_center[1] / dist1 * *circ_seg_r + dist2_tmp;
  circ_seg_end[1] = circ_seg_end[1] / absxk * *circ_seg_wp_rad + waypoints3x3[4];
  dist2_tmp = waypoints3x3[5] - *circ_seg_wp_rad * dist1_tmp;
  circ_seg_wp[2] = waypoints3x3[5];
  circ_seg_start[2] = dist2_tmp;
  circ_seg_center[2] = circ_seg_center[2] / dist1 * *circ_seg_r + dist2_tmp;
  circ_seg_end[2] = circ_seg_end[2] / absxk * *circ_seg_wp_rad + waypoints3x3[5];
}

// Function for MATLAB Function: '<S49>/WpNav Matching'
void MatlabControllerClass::axisAngle(const real32_T v[3], real32_T axis[3],
  real32_T angle, real32_T v_rot[3])
{
  real32_T axis_length;
  real32_T sin_angle;
  real32_T axis_0;
  real32_T v_rot_tmp;
  axis_length = norm(axis);
  sin_angle = axis_length;
  axis_0 = std::abs(axis_length);
  if (axis_0 < 2.22044605E-16F) {
    sin_angle = 2.22044605E-16F;
  }

  axis[0] /= sin_angle;
  sin_angle = axis_length;
  if (axis_0 < 2.22044605E-16F) {
    sin_angle = 2.22044605E-16F;
  }

  axis[1] /= sin_angle;
  sin_angle = axis_length;
  if (axis_0 < 2.22044605E-16F) {
    sin_angle = 2.22044605E-16F;
  }

  axis_0 = axis[2] / sin_angle;
  axis_length = std::cos(angle);
  sin_angle = std::sin(angle);
  v_rot_tmp = (1.0F - axis_length) * ((axis[0] * v[0] + axis[1] * v[1]) + axis_0
    * v[2]);
  v_rot[0] = ((axis[1] * v[2] - axis_0 * v[1]) * sin_angle + axis_length * v[0])
    + v_rot_tmp * axis[0];
  v_rot[1] = ((axis_0 * v[0] - axis[0] * v[2]) * sin_angle + axis_length * v[1])
    + v_rot_tmp * axis[1];
  v_rot[2] = ((axis[0] * v[1] - axis[1] * v[0]) * sin_angle + axis_length * v[2])
    + v_rot_tmp * axis_0;
}

// Function for MATLAB Function: '<S49>/WpNav Matching'
void MatlabControllerClass::wpnavMatchCircSeg(const real32_T circ_seg_center[3],
  const real32_T circ_seg_n[3], real32_T circ_seg_angle, const real32_T
  circ_seg_start[3], const real32_T circ_seg_wp[3], const real32_T p[3],
  real32_T p_match[3], real32_T *t, real32_T *d)
{
  real32_T H;
  real32_T r_p0[3];
  real32_T r_start[3];
  real32_T denom;
  real32_T circ_seg_n_0[3];
  real32_T tmp[3];
  H = ((p[0] - circ_seg_center[0]) * circ_seg_n[0] + (p[1] - circ_seg_center[1])
       * circ_seg_n[1]) + (p[2] - circ_seg_center[2]) * circ_seg_n[2];
  r_p0[0] = (p[0] - H * circ_seg_n[0]) - circ_seg_center[0];
  r_start[0] = circ_seg_start[0] - circ_seg_center[0];
  r_p0[1] = (p[1] - H * circ_seg_n[1]) - circ_seg_center[1];
  r_start[1] = circ_seg_start[1] - circ_seg_center[1];
  r_p0[2] = (p[2] - H * circ_seg_n[2]) - circ_seg_center[2];
  r_start[2] = circ_seg_start[2] - circ_seg_center[2];
  denom = norm(r_p0) * norm(r_start);
  if (std::abs(denom) < 2.22044605E-16F) {
    denom = 2.22044605E-16F;
  }

  H = ((r_p0[0] * r_start[0] + r_p0[1] * r_start[1]) + r_p0[2] * r_start[2]) /
    denom;
  if (1.0F <= H) {
    H = 1.0F;
  }

  denom = circ_seg_angle;
  if (std::abs(circ_seg_angle) < 2.22044605E-16F) {
    denom = 2.22044605E-16F;
  }

  if (-1.0F >= H) {
    H = -1.0F;
  }

  *t = std::acos(H) / denom;
  circ_seg_n_0[0] = circ_seg_n[0];
  circ_seg_n_0[1] = circ_seg_n[1];
  circ_seg_n_0[2] = circ_seg_n[2];
  axisAngle(r_start, circ_seg_n_0, *t * circ_seg_angle, tmp);
  H = circ_seg_center[0] + tmp[0];
  r_start[0] = H - p[0];
  r_p0[0] = circ_seg_center[0] - circ_seg_wp[0];
  p_match[0] = H;
  H = circ_seg_center[1] + tmp[1];
  r_start[1] = H - p[1];
  r_p0[1] = circ_seg_center[1] - circ_seg_wp[1];
  p_match[1] = H;
  H = circ_seg_center[2] + tmp[2];
  r_start[2] = H - p[2];
  r_p0[2] = circ_seg_center[2] - circ_seg_wp[2];
  p_match[2] = H;
  *d = norm(r_start);
  H = norm(r_p0);
  denom = H;
  if (std::abs(H) < 2.22044605E-16F) {
    denom = 2.22044605E-16F;
  }

  r_p0[0] /= denom;
  r_p0[1] /= denom;
  if (((p[0] - circ_seg_wp[0]) * r_p0[0] + (p[1] - circ_seg_wp[1]) * r_p0[1]) +
      (p[2] - circ_seg_wp[2]) * (r_p0[2] / denom) > H) {
    *t = 2.0F;
  }
}

// Function for MATLAB Function: '<S49>/WpNav Matching'
void MatlabControllerClass::wpnavMatchLine(const real32_T p1[3], const real32_T
  p2[3], const real32_T p[3], real32_T p_match[3], real32_T *t, real32_T *d)
{
  real32_T denom;
  real32_T p_match_0[3];
  real32_T p_match_1;
  p_match_1 = p2[0] - p1[0];
  denom = p_match_1 * p_match_1;
  p_match[0] = p_match_1;
  p_match_1 = p2[1] - p1[1];
  denom += p_match_1 * p_match_1;
  p_match[1] = p_match_1;
  p_match_1 = p2[2] - p1[2];
  denom += p_match_1 * p_match_1;
  if (denom < 1.0F) {
    denom = 1.0F;
  }

  *t = (((p[0] - p1[0]) * p_match[0] + (p[1] - p1[1]) * p_match[1]) + (p[2] -
         p1[2]) * p_match_1) / denom;
  denom = *t * p_match[0] + p1[0];
  p_match_0[0] = denom - p[0];
  p_match[0] = denom;
  denom = *t * p_match[1] + p1[1];
  p_match_0[1] = denom - p[1];
  p_match[1] = denom;
  denom = *t * p_match_1 + p1[2];
  p_match_0[2] = denom - p[2];
  p_match[2] = denom;
  *d = norm(p_match_0);
}

// Function for MATLAB Function: '<S49>/WpNav Matching'
void MatlabControllerClass::wpnavMatch_b(const real32_T waypoints_data[], const
  int32_T waypoints_size[2], real32_T wp_radius, int32_T *wp_idx, int32_T *stage,
  const real32_T p[3], real32_T p_match[3], real32_T *t, real32_T *d)
{
  int32_T num_wp;
  int32_T idx3[3];
  real32_T circ_seg_n[3];
  real32_T circ_seg_angle;
  real32_T circ_seg_start[3];
  real32_T circ_seg_end[3];
  real32_T circ_seg_wp[3];
  int32_T idx2[2];
  real32_T p2[3];
  real32_T expl_temp;
  real32_T expl_temp_0;
  real32_T expl_temp_1[3];
  real32_T waypoints[9];
  int32_T idx3_0;
  real_T tmp;
  int32_T waypoints_tmp;
  int32_T exitg1;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  num_wp = waypoints_size[1];
  if (*wp_idx > waypoints_size[1]) {
    *wp_idx = waypoints_size[1];
  }

  if (*wp_idx < 1) {
    *wp_idx = 1;
  }

  do {
    exitg1 = 0;
    guard1 = false;
    guard2 = false;
    if (*stage == 0) {
      if (*wp_idx < -2147483646) {
        idx3[0] = MIN_int32_T;
      } else {
        idx3[0] = *wp_idx - 2;
      }

      if (*wp_idx < -2147483647) {
        idx3[1] = MIN_int32_T;
      } else {
        idx3[1] = *wp_idx - 1;
      }

      if (*wp_idx < -2147483646) {
        idx3_0 = MIN_int32_T;
      } else {
        idx3_0 = *wp_idx - 2;
      }

      if (idx3[0] < 1) {
        tmp = (real_T)num_wp + (real_T)idx3[0];
        if (tmp >= -2.147483648E+9) {
          idx3_0 = (int32_T)tmp;
        } else {
          idx3_0 = MIN_int32_T;
        }
      }

      idx3[0] = idx3_0;
      if (*wp_idx < -2147483647) {
        idx3_0 = MIN_int32_T;
      } else {
        idx3_0 = *wp_idx - 1;
      }

      if (idx3[1] < 1) {
        tmp = (real_T)num_wp + (real_T)idx3[1];
        if (tmp >= -2.147483648E+9) {
          idx3_0 = (int32_T)tmp;
        } else {
          idx3_0 = MIN_int32_T;
        }
      }

      idx3[1] = idx3_0;
      idx3[2] = *wp_idx;
      if (*wp_idx < 1) {
        tmp = (real_T)num_wp + (real_T)*wp_idx;
        if (tmp >= -2.147483648E+9) {
          idx3[2] = (int32_T)tmp;
        } else {
          idx3[2] = MIN_int32_T;
        }
      }

      for (idx3_0 = 0; idx3_0 < 3; idx3_0++) {
        waypoints_tmp = (idx3[idx3_0] - 1) * 3;
        waypoints[3 * idx3_0] = waypoints_data[waypoints_tmp];
        waypoints[1 + 3 * idx3_0] = waypoints_data[waypoints_tmp + 1];
        waypoints[2 + 3 * idx3_0] = waypoints_data[waypoints_tmp + 2];
      }

      wpnavCircSeg(waypoints, wp_radius, &expl_temp, p2, circ_seg_n,
                   &circ_seg_angle, circ_seg_start, circ_seg_end, &expl_temp_0,
                   circ_seg_wp);
      wpnavMatchCircSeg(p2, circ_seg_n, circ_seg_angle, circ_seg_start,
                        circ_seg_wp, p, p_match, t, d);
      if (*t > 1.0F) {
        *stage = 1;
        guard2 = true;
      } else {
        exitg1 = 1;
      }
    } else {
      guard2 = true;
    }

    if (guard2) {
      if (*stage == 1) {
        if (*wp_idx < -2147483646) {
          idx3[0] = MIN_int32_T;
        } else {
          idx3[0] = *wp_idx - 2;
        }

        if (*wp_idx < -2147483647) {
          idx3[1] = MIN_int32_T;
        } else {
          idx3[1] = *wp_idx - 1;
        }

        if (*wp_idx < -2147483646) {
          idx3_0 = MIN_int32_T;
        } else {
          idx3_0 = *wp_idx - 2;
        }

        if (idx3[0] < 1) {
          tmp = (real_T)num_wp + (real_T)idx3[0];
          if (tmp >= -2.147483648E+9) {
            idx3_0 = (int32_T)tmp;
          } else {
            idx3_0 = MIN_int32_T;
          }
        }

        idx3[0] = idx3_0;
        if (*wp_idx < -2147483647) {
          idx3_0 = MIN_int32_T;
        } else {
          idx3_0 = *wp_idx - 1;
        }

        if (idx3[1] < 1) {
          tmp = (real_T)num_wp + (real_T)idx3[1];
          if (tmp >= -2.147483648E+9) {
            idx3_0 = (int32_T)tmp;
          } else {
            idx3_0 = MIN_int32_T;
          }
        }

        idx3[1] = idx3_0;
        idx3[2] = *wp_idx;
        if (*wp_idx < 1) {
          tmp = (real_T)num_wp + (real_T)*wp_idx;
          if (tmp >= -2.147483648E+9) {
            idx3[2] = (int32_T)tmp;
          } else {
            idx3[2] = MIN_int32_T;
          }
        }

        for (idx3_0 = 0; idx3_0 < 3; idx3_0++) {
          waypoints_tmp = (idx3[idx3_0] - 1) * 3;
          waypoints[3 * idx3_0] = waypoints_data[waypoints_tmp];
          waypoints[1 + 3 * idx3_0] = waypoints_data[waypoints_tmp + 1];
          waypoints[2 + 3 * idx3_0] = waypoints_data[waypoints_tmp + 2];
        }

        wpnavCircSeg(waypoints, wp_radius, &expl_temp, p2, circ_seg_n,
                     &circ_seg_angle, circ_seg_start, circ_seg_end, &expl_temp_0,
                     circ_seg_wp);
        if (*wp_idx == num_wp) {
          if (*wp_idx < -2147483647) {
            idx2[0] = MIN_int32_T;
          } else {
            idx2[0] = *wp_idx - 1;
          }

          if (*wp_idx < -2147483647) {
            idx3_0 = MIN_int32_T;
          } else {
            idx3_0 = *wp_idx - 1;
          }

          if (idx2[0] < 1) {
            tmp = (real_T)num_wp + (real_T)idx2[0];
            if (tmp >= -2.147483648E+9) {
              idx3_0 = (int32_T)tmp;
            } else {
              idx3_0 = MIN_int32_T;
            }
          }

          idx2[0] = idx3_0;
          idx2[1] = *wp_idx;
          if (*wp_idx < 1) {
            tmp = (real_T)num_wp + (real_T)*wp_idx;
            if (tmp >= -2.147483648E+9) {
              idx2[1] = (int32_T)tmp;
            } else {
              idx2[1] = MIN_int32_T;
            }
          }

          for (idx3_0 = 0; idx3_0 < 2; idx3_0++) {
            waypoints_tmp = (idx2[idx3_0] - 1) * 3;
            waypoints[3 * idx3_0] = waypoints_data[waypoints_tmp];
            waypoints[1 + 3 * idx3_0] = waypoints_data[waypoints_tmp + 1];
            waypoints[2 + 3 * idx3_0] = waypoints_data[waypoints_tmp + 2];
          }

          waypoints[6] = waypoints_data[0];
          waypoints[7] = waypoints_data[1];
          waypoints[8] = waypoints_data[2];
          wpnavCircSeg(waypoints, wp_radius, &expl_temp, circ_seg_n,
                       circ_seg_start, &expl_temp_0, p2, circ_seg_wp,
                       &circ_seg_angle, expl_temp_1);
        } else {
          if (*wp_idx < -2147483647) {
            idx3[0] = MIN_int32_T;
          } else {
            idx3[0] = *wp_idx - 1;
          }

          if (*wp_idx > 2147483646) {
            idx3[2] = MAX_int32_T;
          } else {
            idx3[2] = *wp_idx + 1;
          }

          if (*wp_idx < -2147483647) {
            idx3_0 = MIN_int32_T;
          } else {
            idx3_0 = *wp_idx - 1;
          }

          if (idx3[0] < 1) {
            tmp = (real_T)num_wp + (real_T)idx3[0];
            if (tmp >= -2.147483648E+9) {
              idx3_0 = (int32_T)tmp;
            } else {
              idx3_0 = MIN_int32_T;
            }
          }

          idx3[0] = idx3_0;
          idx3[1] = *wp_idx;
          if (*wp_idx < 1) {
            tmp = (real_T)num_wp + (real_T)*wp_idx;
            if (tmp >= -2.147483648E+9) {
              idx3[1] = (int32_T)tmp;
            } else {
              idx3[1] = MIN_int32_T;
            }
          }

          if (*wp_idx > 2147483646) {
            idx3_0 = MAX_int32_T;
          } else {
            idx3_0 = *wp_idx + 1;
          }

          if (idx3[2] < 1) {
            tmp = (real_T)num_wp + (real_T)idx3[2];
            if (tmp >= -2.147483648E+9) {
              idx3_0 = (int32_T)tmp;
            } else {
              idx3_0 = MIN_int32_T;
            }
          }

          idx3[2] = idx3_0;
          for (idx3_0 = 0; idx3_0 < 3; idx3_0++) {
            waypoints_tmp = (idx3[idx3_0] - 1) * 3;
            waypoints[3 * idx3_0] = waypoints_data[waypoints_tmp];
            waypoints[1 + 3 * idx3_0] = waypoints_data[waypoints_tmp + 1];
            waypoints[2 + 3 * idx3_0] = waypoints_data[waypoints_tmp + 2];
          }

          wpnavCircSeg(waypoints, wp_radius, &expl_temp, circ_seg_n,
                       circ_seg_start, &expl_temp_0, p2, circ_seg_wp,
                       &circ_seg_angle, expl_temp_1);
        }

        circ_seg_n[0] = circ_seg_end[0] - p2[0];
        circ_seg_n[1] = circ_seg_end[1] - p2[1];
        circ_seg_n[2] = circ_seg_end[2] - p2[2];
        if (norm(circ_seg_n) < 1.0F) {
          *stage = 0;
          if (*wp_idx > 2147483646) {
            *wp_idx = MAX_int32_T;
          } else {
            (*wp_idx)++;
          }

          guard1 = true;
        } else {
          wpnavMatchLine(circ_seg_end, p2, p, p_match, t, d);
          if (*t > 1.0F) {
            *stage = 0;
            if (*wp_idx > 2147483646) {
              *wp_idx = MAX_int32_T;
            } else {
              (*wp_idx)++;
            }

            guard1 = true;
          } else {
            exitg1 = 1;
          }
        }
      } else {
        guard1 = true;
      }
    }

    if (guard1) {
      if (*wp_idx > num_wp) {
        *wp_idx = 1;
      }
    }
  } while (exitg1 == 0);
}

// Function for MATLAB Function: '<S49>/WpNav Matching'
void MatlabControllerClass::wpnavMatch(const real32_T waypoints[15], real32_T
  wp_radius, int32_T *wp_idx, int32_T *stage, const real32_T p[3], real32_T
  p_match[3], real32_T *t, real32_T *d)
{
  int32_T idx3[3];
  real32_T circ_seg_n[3];
  real32_T circ_seg_angle;
  real32_T circ_seg_start[3];
  real32_T circ_seg_end[3];
  real32_T circ_seg_wp[3];
  int32_T idx2[2];
  real32_T p2[3];
  real32_T expl_temp;
  real32_T expl_temp_0;
  real32_T expl_temp_1[3];
  real32_T waypoints_0[9];
  int32_T idx3_0;
  int32_T waypoints_tmp;
  int32_T exitg1;
  boolean_T guard1 = false;
  boolean_T guard2 = false;
  if (*wp_idx > 5) {
    *wp_idx = 5;
  }

  if (*wp_idx < 1) {
    *wp_idx = 1;
  }

  do {
    exitg1 = 0;
    guard1 = false;
    guard2 = false;
    if (*stage == 0) {
      if (*wp_idx < -2147483646) {
        idx3[0] = MIN_int32_T;
      } else {
        idx3[0] = *wp_idx - 2;
      }

      if (*wp_idx < -2147483647) {
        idx3[1] = MIN_int32_T;
      } else {
        idx3[1] = *wp_idx - 1;
      }

      if (*wp_idx < -2147483646) {
        idx3_0 = MIN_int32_T;
      } else {
        idx3_0 = *wp_idx - 2;
      }

      if (idx3[0] < 1) {
        idx3_0 = 5 + idx3[0];
      }

      idx3[0] = idx3_0;
      if (*wp_idx < -2147483647) {
        idx3_0 = MIN_int32_T;
      } else {
        idx3_0 = *wp_idx - 1;
      }

      if (idx3[1] < 1) {
        idx3_0 = 5 + idx3[1];
      }

      idx3[1] = idx3_0;
      idx3[2] = *wp_idx;
      if (*wp_idx < 1) {
        idx3[2] = 5 + *wp_idx;
      }

      for (idx3_0 = 0; idx3_0 < 3; idx3_0++) {
        waypoints_tmp = (idx3[idx3_0] - 1) * 3;
        waypoints_0[3 * idx3_0] = waypoints[waypoints_tmp];
        waypoints_0[1 + 3 * idx3_0] = waypoints[waypoints_tmp + 1];
        waypoints_0[2 + 3 * idx3_0] = waypoints[waypoints_tmp + 2];
      }

      wpnavCircSeg(waypoints_0, wp_radius, &expl_temp, p2, circ_seg_n,
                   &circ_seg_angle, circ_seg_start, circ_seg_end, &expl_temp_0,
                   circ_seg_wp);
      wpnavMatchCircSeg(p2, circ_seg_n, circ_seg_angle, circ_seg_start,
                        circ_seg_wp, p, p_match, t, d);
      if (*t > 1.0F) {
        *stage = 1;
        guard2 = true;
      } else {
        exitg1 = 1;
      }
    } else {
      guard2 = true;
    }

    if (guard2) {
      if (*stage == 1) {
        if (*wp_idx < -2147483646) {
          idx3[0] = MIN_int32_T;
        } else {
          idx3[0] = *wp_idx - 2;
        }

        if (*wp_idx < -2147483647) {
          idx3[1] = MIN_int32_T;
        } else {
          idx3[1] = *wp_idx - 1;
        }

        if (*wp_idx < -2147483646) {
          idx3_0 = MIN_int32_T;
        } else {
          idx3_0 = *wp_idx - 2;
        }

        if (idx3[0] < 1) {
          idx3_0 = 5 + idx3[0];
        }

        idx3[0] = idx3_0;
        if (*wp_idx < -2147483647) {
          idx3_0 = MIN_int32_T;
        } else {
          idx3_0 = *wp_idx - 1;
        }

        if (idx3[1] < 1) {
          idx3_0 = 5 + idx3[1];
        }

        idx3[1] = idx3_0;
        idx3[2] = *wp_idx;
        if (*wp_idx < 1) {
          idx3[2] = 5 + *wp_idx;
        }

        for (idx3_0 = 0; idx3_0 < 3; idx3_0++) {
          waypoints_tmp = (idx3[idx3_0] - 1) * 3;
          waypoints_0[3 * idx3_0] = waypoints[waypoints_tmp];
          waypoints_0[1 + 3 * idx3_0] = waypoints[waypoints_tmp + 1];
          waypoints_0[2 + 3 * idx3_0] = waypoints[waypoints_tmp + 2];
        }

        wpnavCircSeg(waypoints_0, wp_radius, &expl_temp, p2, circ_seg_n,
                     &circ_seg_angle, circ_seg_start, circ_seg_end, &expl_temp_0,
                     circ_seg_wp);
        if (*wp_idx == 5) {
          if (*wp_idx < -2147483647) {
            idx2[0] = MIN_int32_T;
          } else {
            idx2[0] = *wp_idx - 1;
          }

          if (*wp_idx < -2147483647) {
            idx3_0 = MIN_int32_T;
          } else {
            idx3_0 = *wp_idx - 1;
          }

          if (idx2[0] < 1) {
            idx3_0 = 5 + idx2[0];
          }

          idx2[0] = idx3_0;
          idx2[1] = *wp_idx;
          if (*wp_idx < 1) {
            idx2[1] = 5 + *wp_idx;
          }

          for (idx3_0 = 0; idx3_0 < 2; idx3_0++) {
            waypoints_tmp = (idx2[idx3_0] - 1) * 3;
            waypoints_0[3 * idx3_0] = waypoints[waypoints_tmp];
            waypoints_0[1 + 3 * idx3_0] = waypoints[waypoints_tmp + 1];
            waypoints_0[2 + 3 * idx3_0] = waypoints[waypoints_tmp + 2];
          }

          waypoints_0[6] = waypoints[0];
          waypoints_0[7] = waypoints[1];
          waypoints_0[8] = waypoints[2];
          wpnavCircSeg(waypoints_0, wp_radius, &expl_temp, circ_seg_n,
                       circ_seg_start, &expl_temp_0, p2, circ_seg_wp,
                       &circ_seg_angle, expl_temp_1);
        } else {
          if (*wp_idx < -2147483647) {
            idx3[0] = MIN_int32_T;
          } else {
            idx3[0] = *wp_idx - 1;
          }

          if (*wp_idx > 2147483646) {
            idx3[2] = MAX_int32_T;
          } else {
            idx3[2] = *wp_idx + 1;
          }

          if (*wp_idx < -2147483647) {
            idx3_0 = MIN_int32_T;
          } else {
            idx3_0 = *wp_idx - 1;
          }

          if (idx3[0] < 1) {
            idx3_0 = 5 + idx3[0];
          }

          idx3[0] = idx3_0;
          idx3[1] = *wp_idx;
          if (*wp_idx < 1) {
            idx3[1] = 5 + *wp_idx;
          }

          if (*wp_idx > 2147483646) {
            idx3_0 = MAX_int32_T;
          } else {
            idx3_0 = *wp_idx + 1;
          }

          if (idx3[2] < 1) {
            idx3_0 = 5 + idx3[2];
          }

          idx3[2] = idx3_0;
          for (idx3_0 = 0; idx3_0 < 3; idx3_0++) {
            waypoints_tmp = (idx3[idx3_0] - 1) * 3;
            waypoints_0[3 * idx3_0] = waypoints[waypoints_tmp];
            waypoints_0[1 + 3 * idx3_0] = waypoints[waypoints_tmp + 1];
            waypoints_0[2 + 3 * idx3_0] = waypoints[waypoints_tmp + 2];
          }

          wpnavCircSeg(waypoints_0, wp_radius, &expl_temp, circ_seg_n,
                       circ_seg_start, &expl_temp_0, p2, circ_seg_wp,
                       &circ_seg_angle, expl_temp_1);
        }

        circ_seg_n[0] = circ_seg_end[0] - p2[0];
        circ_seg_n[1] = circ_seg_end[1] - p2[1];
        circ_seg_n[2] = circ_seg_end[2] - p2[2];
        if (norm(circ_seg_n) < 1.0F) {
          *stage = 0;
          if (*wp_idx > 2147483646) {
            *wp_idx = MAX_int32_T;
          } else {
            (*wp_idx)++;
          }

          guard1 = true;
        } else {
          wpnavMatchLine(circ_seg_end, p2, p, p_match, t, d);
          if (*t > 1.0F) {
            *stage = 0;
            if (*wp_idx > 2147483646) {
              *wp_idx = MAX_int32_T;
            } else {
              (*wp_idx)++;
            }

            guard1 = true;
          } else {
            exitg1 = 1;
          }
        }
      } else {
        guard1 = true;
      }
    }

    if (guard1) {
      if (*wp_idx > 5) {
        *wp_idx = 1;
      }
    }
  } while (exitg1 == 0);
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

// Function for MATLAB Function: '<S76>/caIndiWls'
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

// Function for MATLAB Function: '<S76>/caIndiWls'
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

// Function for MATLAB Function: '<S76>/caIndiWls'
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

// Function for MATLAB Function: '<S76>/caIndiWls'
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

// Function for MATLAB Function: '<S76>/caIndiWls'
void MatlabControllerClass::mldivide(const real32_T A_data[], const int32_T
  A_size[2], const real32_T B_0[8], real32_T Y_data[], int32_T *Y_size)
{
  if (A_size[1] == 0) {
    *Y_size = 0;
  } else {
    qrsolve(A_data, A_size, B_0, Y_data, Y_size);
  }
}

// Function for MATLAB Function: '<S76>/caIndiWls'
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

// Function for MATLAB Function: '<S76>/caIndiWls'
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

// Function for MATLAB Function: '<S53>/G1 learn rate'
real32_T MatlabControllerClass::mean(const real32_T x[4])
{
  return (((x[0] + x[1]) + x[2]) + x[3]) / 4.0F;
}

// Model step function
void MatlabControllerClass::step()
{
  // local block i/o variables
  real32_T rtb_a_T_meas;
  real32_T rtb_a_T;
  real32_T rtb_V_K_f;
  real32_T rtb_V_K_arc;
  real32_T rtb_status;
  real32_T scale;
  real32_T absxk;
  real32_T t;
  int32_T k;
  real32_T q0_q2;
  real32_T q1_q2;
  real32_T q1_q3;
  real32_T q2_q3;
  real32_T G_omega[16];
  int8_T b_data[4];
  real_T Delta_u_d;
  real32_T umin[4];
  real32_T umax[4];
  real32_T W_v[16];
  real32_T W_u[16];
  int32_T flight_mode;
  real32_T wp_approach_out[15];
  int32_T wp_idx_app_2;
  int32_T stage_app_2;
  int32_T stage_2;
  int32_T wp_idx_2;
  int32_T stage_tmp;
  int32_T wp_idx_tmp;
  int32_T b_stage_;
  int32_T c_stage_app;
  static const int8_T h[3] = { 3, 4, 0 };

  static const int8_T i[3] = { 4, 0, 1 };

  real_T learn_rate[4];
  real32_T a_Kb_meas[3];
  real32_T cos_Theta;
  real32_T rtb_Add_a[4];
  real32_T rtb_G2[16];
  real32_T rtb_G1[16];
  real32_T rtb_y_bn[16];
  real32_T rtb_n_b[3];
  real32_T rtb_wp[30];
  real32_T rtb_Sum2_e[3];
  real32_T rtb_q_red_l[4];
  real32_T rtb_n_b_dt2[3];
  real32_T rtb_n_b_dt_i[3];
  real32_T rtb_n_b_h[3];
  real32_T rtb_y_a[3];
  real32_T rtb_n_dt2_h[3];
  real32_T rtb_n_dt_j[3];
  real32_T rtb_n[3];
  real32_T rtb_n_b_dt[3];
  real32_T rtb_n_dt2[3];
  real32_T rtb_n_dt[3];
  real32_T rtb_y_o[3];
  boolean_T rtb_is_finished;
  boolean_T rtb_is_slowing;
  real32_T rtb_M_bg[9];
  real32_T rtb_Sum2_hq[3];
  real32_T rtb_Sum2_jx[9];
  real32_T rtb_Sum2_a[4];
  real32_T rtb_y_a0;
  real32_T rtb_n_g_des_n[3];
  int32_T rtb_DataTypeConversion2;
  real32_T rtb_p_ahead_l[3];
  real32_T rtb_Gain6;
  boolean_T rtb_is_intercept_arc;
  boolean_T rtb_is_descent;
  real32_T rtb_Delta_diag_W_v[4];
  boolean_T rtb_UnitDelay1_j;
  boolean_T rtb_UnitDelay4_n;
  boolean_T rtb_Compare_i;
  boolean_T rtb_Compare;
  real32_T G_omega_0[16];
  real32_T tmp[4];
  real32_T rtb_wp_data[30];
  real32_T cos_Theta_0[9];
  uint16_T rtb_DataTypeConversion2_0[3];
  uint16_T rtb_DataTypeConversion2_1[3];
  real32_T tmp_0[9];
  real32_T tmp_1[9];
  real32_T tmp_2[9];
  int32_T rtb_wp_size[2];
  int32_T rtb_wp_size_0[2];
  int32_T rtb_wp_size_1[2];
  int32_T rtb_wp_size_2[2];
  int32_T rtb_wp_size_3[2];
  real32_T rtb_p_ahead_c_idx_1;
  real32_T rtb_p_ahead_c_idx_2;
  real32_T wp_approach_out_tmp;
  real32_T tmp_3;
  real32_T tmp_4;
  real32_T scale_tmp;

  // MATLAB Function: '<Root>/MATLAB Function1' incorporates:
  //   Inport: '<Root>/measure'

  scale = 1.29246971E-26F;
  absxk = std::abs(rtU.measure.V_Kg[0]);
  if (absxk > 1.29246971E-26F) {
    rtb_y_a0 = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    rtb_y_a0 = t * t;
  }

  absxk = std::abs(rtU.measure.V_Kg[1]);
  if (absxk > scale) {
    t = scale / absxk;
    rtb_y_a0 = rtb_y_a0 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    rtb_y_a0 += t * t;
  }

  absxk = std::abs(rtU.measure.V_Kg[2]);
  if (absxk > scale) {
    t = scale / absxk;
    rtb_y_a0 = rtb_y_a0 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    rtb_y_a0 += t * t;
  }

  rtb_y_a0 = scale * std::sqrt(rtb_y_a0);

  // End of MATLAB Function: '<Root>/MATLAB Function1'

  // RelationalOperator: '<S2>/Compare' incorporates:
  //   Constant: '<S2>/Constant'
  //   Inport: '<Root>/cmd'
  //   SignalConversion: '<Root>/BusConversion_InsertedFor_cmd_at_outport_0'

  rtb_Compare = (rtU.cmd.RC_pwm[7] > 1500.0F);

  // Outputs for Enabled SubSystem: '<S3>/fast descent sequencer' incorporates:
  //   EnablePort: '<S11>/Enable'

  if (rtb_Compare) {
    if (!rtDW.fastdescentsequencer_MODE) {
      // InitializeConditions for UnitDelay: '<S11>/Unit Delay3'
      rtDW.UnitDelay3_DSTATE = 0.0F;

      // InitializeConditions for UnitDelay: '<S11>/Unit Delay5'
      rtDW.UnitDelay5_DSTATE = 0.0F;

      // InitializeConditions for UnitDelay: '<S11>/Unit Delay'
      rtDW.UnitDelay_DSTATE_b = false;

      // InitializeConditions for UnitDelay: '<S12>/Unit Delay4'
      rtDW.UnitDelay4_DSTATE_d = false;

      // InitializeConditions for DiscreteIntegrator: '<S12>/Discrete-Time Integrator' 
      rtDW.DiscreteTimeIntegrator_DSTAT_ng = 0.0F;
      rtDW.DiscreteTimeIntegrator_PrevRe_g = 2;

      // InitializeConditions for UnitDelay: '<S11>/Unit Delay1'
      rtDW.UnitDelay1_DSTATE_h = false;

      // InitializeConditions for UnitDelay: '<S11>/Unit Delay2'
      rtDW.UnitDelay2_DSTATE_f = false;
      rtDW.fastdescentsequencer_MODE = true;
    }

    // Outputs for Enabled SubSystem: '<S20>/Subsystem' incorporates:
    //   EnablePort: '<S30>/Enable'

    // RelationalOperator: '<S28>/Compare' incorporates:
    //   Constant: '<S28>/Constant'
    //   UnitDelay: '<S11>/Unit Delay3'

    if (rtDW.UnitDelay3_DSTATE == 0.0F) {
      // MATLAB Function: '<S30>/MATLAB Function'
      rtDW.uv_g = 0.0F;
      scale = 1.29246971E-26F;
      if (rtb_y_a0 > 1.29246971E-26F) {
        t = 1.29246971E-26F / rtb_y_a0;
        rtDW.uv_g = rtDW.uv_g * t * t + 1.0F;
        scale = rtb_y_a0;
      } else {
        t = rtb_y_a0 / 1.29246971E-26F;
        rtDW.uv_g += t * t;
      }

      if (rtb_y_a0 > scale) {
        t = scale / rtb_y_a0;
        rtDW.uv_g = rtDW.uv_g * t * t + 1.0F;
        scale = rtb_y_a0;
      } else {
        t = rtb_y_a0 / scale;
        rtDW.uv_g += t * t;
      }

      rtDW.uv_g = scale * std::sqrt(rtDW.uv_g);

      // End of MATLAB Function: '<S30>/MATLAB Function'
    }

    // End of RelationalOperator: '<S28>/Compare'
    // End of Outputs for SubSystem: '<S20>/Subsystem'

    // MATLAB Function: '<S20>/MATLAB Function' incorporates:
    //   UnitDelay: '<S11>/Unit Delay5'

    rtb_V_K_arc = rtb_y_a0 - std::cos(rtDW.UnitDelay5_DSTATE) * rtDW.uv_g;
    rtb_V_K_f = rtb_y_a0;

    // Outputs for Enabled SubSystem: '<S11>/Subsystem2' incorporates:
    //   EnablePort: '<S21>/Enable'

    // UnitDelay: '<S11>/Unit Delay'
    if (rtDW.UnitDelay_DSTATE_b) {
      if (!rtDW.Subsystem2_MODE) {
        rtDW.Subsystem2_MODE = true;
      }

      // MATLAB Function: '<S21>/MATLAB Function' incorporates:
      //   Constant: '<S21>/Constant'
      //   Constant: '<S21>/Constant1'
      //   Constant: '<S21>/a_max'

      if (2.0F < rtP.lindi.dive.amax - 9.81F) {
        q1_q3 = rtP.lindi.dive.amax - 9.81F;
      } else {
        q1_q3 = 2.0F;
      }

      rtb_y_a0 = rtb_V_K_arc * rtb_V_K_arc / q1_q3;
      rtDW.h_abfang = rtP.lindi.dive.hend + rtb_y_a0;
      rtDW.r_abfang = rtb_y_a0;

      // End of MATLAB Function: '<S21>/MATLAB Function'
    } else {
      if (rtDW.Subsystem2_MODE) {
        rtDW.Subsystem2_MODE = false;
      }
    }

    // End of UnitDelay: '<S11>/Unit Delay'
    // End of Outputs for SubSystem: '<S11>/Subsystem2'

    // UnitDelay: '<S12>/Unit Delay4'
    rtb_UnitDelay4_n = rtDW.UnitDelay4_DSTATE_d;

    // DiscreteIntegrator: '<S12>/Discrete-Time Integrator' incorporates:
    //   UnitDelay: '<S12>/Unit Delay4'

    if ((!rtDW.UnitDelay4_DSTATE_d) && (rtDW.DiscreteTimeIntegrator_PrevRe_g ==
         1)) {
      rtDW.DiscreteTimeIntegrator_DSTAT_ng = 0.0F;
    }

    // UnitDelay: '<S11>/Unit Delay1'
    rtb_UnitDelay1_j = rtDW.UnitDelay1_DSTATE_h;

    // Outputs for Enabled SubSystem: '<S11>/Duration in slowing' incorporates:
    //   EnablePort: '<S18>/Enable'

    // UnitDelay: '<S11>/Unit Delay2'
    if (rtDW.UnitDelay2_DSTATE_f) {
      if (!rtDW.Durationinslowing_MODE) {
        // InitializeConditions for DiscreteIntegrator: '<S18>/Discrete-Time Integrator' 
        rtDW.DiscreteTimeIntegrator_DSTAT_mf = 0.0F;
        rtDW.DiscreteTimeIntegrator_PrevRe_c = 2;
        rtDW.Durationinslowing_MODE = true;
      }

      // DiscreteIntegrator: '<S18>/Discrete-Time Integrator' incorporates:
      //   UnitDelay: '<S11>/Unit Delay1'

      if ((rtDW.UnitDelay1_DSTATE_h && (rtDW.DiscreteTimeIntegrator_PrevRe_c <=
            0)) || ((!rtDW.UnitDelay1_DSTATE_h) &&
                    (rtDW.DiscreteTimeIntegrator_PrevRe_c == 1))) {
        rtDW.DiscreteTimeIntegrator_DSTAT_mf = 0.0F;
      }

      rtDW.DiscreteTimeIntegrator = rtDW.DiscreteTimeIntegrator_DSTAT_mf;

      // End of DiscreteIntegrator: '<S18>/Discrete-Time Integrator'

      // Update for DiscreteIntegrator: '<S18>/Discrete-Time Integrator' incorporates:
      //   UnitDelay: '<S11>/Unit Delay1'

      rtDW.DiscreteTimeIntegrator_DSTAT_mf += 0.0025F;
      rtDW.DiscreteTimeIntegrator_PrevRe_c = (int8_T)rtDW.UnitDelay1_DSTATE_h;
    } else {
      if (rtDW.Durationinslowing_MODE) {
        rtDW.Durationinslowing_MODE = false;
      }
    }

    // End of UnitDelay: '<S11>/Unit Delay2'
    // End of Outputs for SubSystem: '<S11>/Duration in slowing'

    // MATLAB Function: '<S11>/Mission Status' incorporates:
    //   Constant: '<S11>/Constant'
    //   Constant: '<S11>/t_slowing_max'
    //   Constant: '<S11>/v_loiter'
    //   DiscreteIntegrator: '<S12>/Discrete-Time Integrator'
    //   Gain: '<Root>/Gain6'
    //   Inport: '<Root>/measure'
    //   UnitDelay: '<S11>/Unit Delay3'

    rtb_status = rtDW.UnitDelay3_DSTATE;
    rtb_y_a0 = rtb_status;
    flight_mode = 0;
    rtb_is_descent = false;
    rtb_is_intercept_arc = false;
    rtb_is_slowing = false;
    rtb_is_finished = false;
    if (rtb_status == 0.0F) {
      rtb_is_descent = true;
      flight_mode = 2;
      if (-rtU.measure.s_Kg[2] < rtDW.h_abfang) {
        rtb_y_a0 = rtb_status + 1.0F;
      }
    } else if (rtb_status == 1.0F) {
      rtb_is_intercept_arc = true;
      flight_mode = 2;
      if (-rtU.measure.s_Kg[2] < rtP.lindi.dive.hend) {
        rtb_y_a0 = rtb_status + 1.0F;
      } else {
        if (rtDW.DiscreteTimeIntegrator_DSTAT_ng > 1.57079637F * rtDW.r_abfang)
        {
          rtb_y_a0 = rtb_status + 1.0F;
        }
      }
    } else if (rtb_status == 2.0F) {
      rtb_is_slowing = true;
      flight_mode = 3;
      if (rtb_V_K_f < rtP.lindi.dive.vloiter) {
        rtb_y_a0 = rtb_status + 1.0F;
      } else {
        if (rtDW.DiscreteTimeIntegrator > rtP.lindi.dive.tslow) {
          rtb_y_a0 = rtb_status + 1.0F;
        }
      }
    } else {
      if (rtb_status == 3.0F) {
        rtb_is_finished = true;
      }
    }

    rtb_status = rtb_y_a0;

    // Outputs for Enabled SubSystem: '<S11>/Desired Lean Descent' incorporates:
    //   EnablePort: '<S14>/Enable'

    if (rtb_is_descent) {
      // SignalConversion: '<S14>/OutportBuffer_InsertedFor_lean_des_at_inport_0' incorporates:
      //   Constant: '<S14>/Constant'

      rtDW.Merge_a = -1.57079637F;
    }

    // End of Outputs for SubSystem: '<S11>/Desired Lean Descent'

    // Outputs for Enabled SubSystem: '<S11>/Desired Lean Intercept Arc' incorporates:
    //   EnablePort: '<S15>/Enable'

    if (rtb_is_intercept_arc) {
      if (!rtDW.DesiredLeanInterceptArc_MODE) {
        rtDW.DesiredLeanInterceptArc_MODE = true;
      }

      // MATLAB Function: '<S15>/MATLAB Function6'
      t = rtDW.r_abfang;
      if (std::abs(rtDW.r_abfang) < 2.22044605E-16F) {
        t = 2.22044605E-16F;
      }

      // SignalConversion: '<S15>/OutportBufferForlean_des' incorporates:
      //   DiscreteIntegrator: '<S12>/Discrete-Time Integrator'
      //   MATLAB Function: '<S15>/MATLAB Function6'

      rtDW.Merge_a = rtDW.DiscreteTimeIntegrator_DSTAT_ng / t + -1.57079637F;
    } else {
      if (rtDW.DesiredLeanInterceptArc_MODE) {
        rtDW.DesiredLeanInterceptArc_MODE = false;
      }
    }

    // End of Outputs for SubSystem: '<S11>/Desired Lean Intercept Arc'

    // Outputs for Enabled SubSystem: '<S11>/Desired Lean slowing' incorporates:
    //   EnablePort: '<S17>/Enable'

    if (rtb_is_slowing) {
      // SignalConversion: '<S17>/OutportBuffer_InsertedFor_lean_des_at_inport_0' incorporates:
      //   Constant: '<S17>/Constant'

      rtDW.Merge_a = rtP.lindi.dive.ptchslow * 3.14159274F / 180.0F;
    }

    // End of Outputs for SubSystem: '<S11>/Desired Lean slowing'

    // Outputs for Enabled SubSystem: '<S11>/Desired Lean finished' incorporates:
    //   EnablePort: '<S16>/Enable'

    if (rtb_is_finished) {
      // SignalConversion: '<S16>/OutportBuffer_InsertedFor_lean_des_at_inport_0' incorporates:
      //   Constant: '<S16>/Constant'

      rtDW.Merge_a = 0.0F;
    }

    // End of Outputs for SubSystem: '<S11>/Desired Lean finished'
    // Outputs for Enabled SubSystem: '<S11>/Throttle Descent' incorporates:
    //   EnablePort: '<S22>/Enable'

    if (rtb_is_descent) {
      // SignalConversion: '<S22>/OutportBuffer_InsertedFor_throttle_at_inport_0' incorporates:
      //   Constant: '<S22>/throttle decent'

      rtDW.Merge1_i = rtP.lindi.dive.thrfall;
    }

    // End of Outputs for SubSystem: '<S11>/Throttle Descent'

    // Outputs for Enabled SubSystem: '<S11>/Throttle Intercept Arc (Load factor controller)' incorporates:
    //   EnablePort: '<S23>/Enable'

    if (rtb_is_intercept_arc) {
      if (!rtDW.ThrottleInterceptArcLoadfactorc) {
        // InitializeConditions for DiscreteIntegrator: '<S35>/Discrete-Time Integrator y' 
        rtDW.DiscreteTimeIntegratory_DSTAT_h = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S34>/Discrete-Time Integrator' 
        rtDW.DiscreteTimeIntegrator_DSTATE_m = rtDW.y_0;

        // InitializeConditions for DiscreteIntegrator: '<S35>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_D_fu = 0.0F;
        rtDW.ThrottleInterceptArcLoadfactorc = true;
      }

      // MATLAB Function: '<S23>/MATLAB Function' incorporates:
      //   DiscreteIntegrator: '<S35>/Discrete-Time Integrator y'
      //   MATLAB Function: '<S23>/measured specific thrust'

      t = rtDW.r_abfang;
      if (std::abs(rtDW.r_abfang) < 2.22044605E-16F) {
        t = 2.22044605E-16F;
      }

      rtb_Gain6 = rtb_V_K_arc * rtb_V_K_arc / t;
      scale = 9.81F * std::cos(rtDW.DiscreteTimeIntegratory_DSTAT_h);
      rtb_a_T = scale + rtb_Gain6;

      // MATLAB Function: '<S23>/measured specific thrust' incorporates:
      //   DiscreteIntegrator: '<S35>/Discrete-Time Integrator y'
      //   Inport: '<Root>/measure'
      //   Sum: '<Root>/Add'

      rtb_y_a0 = std::sin(rtDW.DiscreteTimeIntegratory_DSTAT_h);
      cos_Theta = std::cos(rtDW.DiscreteTimeIntegratory_DSTAT_h);
      cos_Theta_0[0] = cos_Theta;
      cos_Theta_0[3] = 0.0F;
      cos_Theta_0[6] = -rtb_y_a0;
      cos_Theta_0[1] = 0.0F;
      cos_Theta_0[4] = 1.0F;
      cos_Theta_0[7] = 0.0F;
      cos_Theta_0[2] = rtb_y_a0;
      cos_Theta_0[5] = 0.0F;
      cos_Theta_0[8] = cos_Theta;
      for (wp_idx_app_2 = 0; wp_idx_app_2 < 3; wp_idx_app_2++) {
        a_Kb_meas[wp_idx_app_2] = cos_Theta_0[wp_idx_app_2 + 6] *
          (rtU.measure.a_Kg[2] + 9.81F) + (cos_Theta_0[wp_idx_app_2 + 3] *
          rtU.measure.a_Kg[1] + cos_Theta_0[wp_idx_app_2] * rtU.measure.a_Kg[0]);
      }

      rtb_a_T_meas = scale + -a_Kb_meas[2];

      // Constant: '<S34>/y_0'
      rtDW.y_0 = rtP.lindi.dive.thrfall;

      // Sum: '<S23>/Add1' incorporates:
      //   DiscreteIntegrator: '<S34>/Discrete-Time Integrator'
      //   Gain: '<S23>/Gain4'
      //   MATLAB Function: '<S23>/MATLAB Function'
      //   MATLAB Function: '<S23>/measured specific thrust'
      //   Sum: '<S23>/Add'

      t = (rtb_Gain6 - (-a_Kb_meas[2])) * rtP.lindi.dive.kacc +
        rtDW.DiscreteTimeIntegrator_DSTATE_m;

      // Saturate: '<S23>/Saturation'
      if (t > 1.0F) {
        t = 1.0F;
      } else {
        if (t < -1.0F) {
          t = -1.0F;
        }
      }

      // End of Saturate: '<S23>/Saturation'

      // SignalConversion: '<S23>/OutportBufferForthrottle'
      rtDW.Merge1_i = t;

      // Sum: '<S35>/Sum2' incorporates:
      //   DiscreteIntegrator: '<S35>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S35>/Discrete-Time Integrator y_dt'
      //   Product: '<S35>/Product2'
      //   Sum: '<S35>/Sum3'

      rtb_Gain6 = rtDW.Merge_a - (rtDW.DiscreteTimeIntegratory_dt_D_fu * 0.2F +
        rtDW.DiscreteTimeIntegratory_DSTAT_h);

      // Update for DiscreteIntegrator: '<S35>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S35>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_DSTAT_h += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_D_fu;

      // Update for DiscreteIntegrator: '<S34>/Discrete-Time Integrator' incorporates:
      //   Constant: '<S34>/T'
      //   Product: '<S34>/Divide'
      //   Sum: '<S34>/Sum2'

      rtDW.DiscreteTimeIntegrator_DSTATE_m += (t -
        rtDW.DiscreteTimeIntegrator_DSTATE_m) / rtP.lindi.mtc * 0.0025F;

      // Update for DiscreteIntegrator: '<S35>/Discrete-Time Integrator y_dt' incorporates:
      //   Product: '<S35>/Product1'

      rtDW.DiscreteTimeIntegratory_dt_D_fu += rtb_Gain6 * 100.0F * 0.0025F;
    } else {
      if (rtDW.ThrottleInterceptArcLoadfactorc) {
        rtDW.ThrottleInterceptArcLoadfactorc = false;
      }
    }

    // End of Outputs for SubSystem: '<S11>/Throttle Intercept Arc (Load factor controller)' 

    // Outputs for Enabled SubSystem: '<S11>/Throttle slowing'
    Throttlefinished(rtb_is_slowing, &rtDW.Merge1_i);

    // End of Outputs for SubSystem: '<S11>/Throttle slowing'

    // Outputs for Enabled SubSystem: '<S11>/Throttle finished'
    Throttlefinished(rtb_is_finished, &rtDW.Merge1_i);

    // End of Outputs for SubSystem: '<S11>/Throttle finished'

    // SignalConversion: '<S11>/OutportBufferForrpyt' incorporates:
    //   Constant: '<S13>/Constant3'
    //   Constant: '<S13>/Constant4'
    //   Gain: '<S13>/Gain2'

    rtDW.Merge[0] = 0.0F;
    rtDW.Merge[1] = 1.0F / rtP.lindi.atc.rm.leanmax * rtDW.Merge_a;
    rtDW.Merge[2] = 0.0F;
    rtDW.Merge[3] = rtDW.Merge1_i;

    // SignalConversion: '<S11>/OutportBufferForflight_mode' incorporates:
    //   MATLAB Function: '<S11>/Mission Status'

    rtDW.Merge1 = (real32_T)flight_mode;

    // Update for UnitDelay: '<S11>/Unit Delay3'
    rtDW.UnitDelay3_DSTATE = rtb_status;

    // Update for UnitDelay: '<S11>/Unit Delay5'
    rtDW.UnitDelay5_DSTATE = rtDW.Merge_a;

    // Update for UnitDelay: '<S11>/Unit Delay' incorporates:
    //   Logic: '<S11>/Logical Operator'

    rtDW.UnitDelay_DSTATE_b = !rtb_is_intercept_arc;

    // Update for UnitDelay: '<S12>/Unit Delay4'
    rtDW.UnitDelay4_DSTATE_d = rtb_is_intercept_arc;

    // Update for DiscreteIntegrator: '<S12>/Discrete-Time Integrator' incorporates:
    //   DataTypeConversion: '<S12>/Data Type Conversion'
    //   Product: '<S12>/Product'

    rtDW.DiscreteTimeIntegrator_DSTAT_ng += rtb_V_K_arc * (real32_T)
      rtb_is_intercept_arc * 0.0025F;
    rtDW.DiscreteTimeIntegrator_PrevRe_g = (int8_T)rtb_UnitDelay4_n;

    // Update for UnitDelay: '<S11>/Unit Delay1'
    rtDW.UnitDelay1_DSTATE_h = rtb_is_slowing;

    // Update for UnitDelay: '<S11>/Unit Delay2'
    rtDW.UnitDelay2_DSTATE_f = rtb_UnitDelay1_j;
  } else {
    if (rtDW.fastdescentsequencer_MODE) {
      // Disable for Enabled SubSystem: '<S11>/Subsystem2'
      if (rtDW.Subsystem2_MODE) {
        rtDW.Subsystem2_MODE = false;
      }

      // End of Disable for SubSystem: '<S11>/Subsystem2'

      // Disable for Enabled SubSystem: '<S11>/Duration in slowing'
      if (rtDW.Durationinslowing_MODE) {
        rtDW.Durationinslowing_MODE = false;
      }

      // End of Disable for SubSystem: '<S11>/Duration in slowing'

      // Disable for Enabled SubSystem: '<S11>/Desired Lean Intercept Arc'
      if (rtDW.DesiredLeanInterceptArc_MODE) {
        rtDW.DesiredLeanInterceptArc_MODE = false;
      }

      // End of Disable for SubSystem: '<S11>/Desired Lean Intercept Arc'

      // Disable for Enabled SubSystem: '<S11>/Throttle Intercept Arc (Load factor controller)' 
      if (rtDW.ThrottleInterceptArcLoadfactorc) {
        rtDW.ThrottleInterceptArcLoadfactorc = false;
      }

      // End of Disable for SubSystem: '<S11>/Throttle Intercept Arc (Load factor controller)' 
      rtDW.fastdescentsequencer_MODE = false;
    }
  }

  // End of Outputs for SubSystem: '<S3>/fast descent sequencer'

  // Logic: '<S3>/Logical Operator'
  rtb_Compare = !rtb_Compare;

  // Outputs for Enabled SubSystem: '<S3>/bypass stick commands' incorporates:
  //   EnablePort: '<S10>/Enable'

  if (rtb_Compare) {
    // Inport: '<S10>/rpyt_in' incorporates:
    //   Inport: '<Root>/cmd'
    //   SignalConversion: '<Root>/BusConversion_InsertedFor_cmd_at_outport_0'

    rtDW.Merge[0] = rtU.cmd.roll;
    rtDW.Merge[1] = rtU.cmd.pitch;
    rtDW.Merge[2] = rtU.cmd.yaw;
    rtDW.Merge[3] = rtU.cmd.thr;
  }

  // End of Outputs for SubSystem: '<S3>/bypass stick commands'

  // MATLAB Function: '<Root>/MATLAB Function' incorporates:
  //   Inport: '<Root>/cmd'
  //   SignalConversion: '<Root>/BusConversion_InsertedFor_cmd_at_outport_0'

  flight_mode = 0;
  if (rtU.cmd.RC_pwm[6] >= 1300.0F) {
    if ((rtU.cmd.RC_pwm[6] >= 1300.0F) && (rtU.cmd.RC_pwm[6] < 1700.0F)) {
      flight_mode = 2;
    } else {
      if (rtU.cmd.RC_pwm[6] >= 1700.0F) {
        flight_mode = 4;
      }
    }
  }

  // Outputs for Enabled SubSystem: '<S3>/bypass flightmode' incorporates:
  //   EnablePort: '<S9>/Enable'

  if (rtb_Compare) {
    // Inport: '<S9>/flightmode_in' incorporates:
    //   MATLAB Function: '<Root>/MATLAB Function'

    rtDW.Merge1 = (real32_T)flight_mode;
  }

  // End of Outputs for SubSystem: '<S3>/bypass flightmode'

  // MATLAB Function: '<S46>/LindiCopter State Logic'
  rtb_Compare = true;
  rtb_is_finished = true;
  rtb_UnitDelay4_n = true;
  rtb_Compare_i = true;
  rtb_UnitDelay1_j = true;
  rtb_is_descent = true;
  rtb_is_intercept_arc = true;
  switch ((int32_T)rtDW.Merge1) {
   case 0:
    rtb_UnitDelay4_n = false;
    rtb_Compare_i = false;
    rtb_UnitDelay1_j = false;
    rtb_is_descent = false;
    rtb_is_intercept_arc = false;
    break;

   case 1:
    rtb_is_finished = false;
    rtb_UnitDelay4_n = false;
    rtb_UnitDelay1_j = false;
    rtb_is_descent = false;
    rtb_is_intercept_arc = false;
    break;

   case 2:
    rtb_Compare = false;
    rtb_is_finished = false;
    rtb_UnitDelay4_n = false;
    rtb_Compare_i = false;
    rtb_is_intercept_arc = false;
    break;

   case 3:
    rtb_Compare_i = false;
    rtb_is_descent = false;
    rtb_is_intercept_arc = false;
    break;

   case 4:
    rtb_UnitDelay4_n = false;
    rtb_Compare_i = false;
    rtb_UnitDelay1_j = false;
    rtb_is_descent = false;
    break;
  }

  // End of MATLAB Function: '<S46>/LindiCopter State Logic'

  // DiscreteIntegrator: '<S92>/Discrete-Time Integrator y' incorporates:
  //   Inport: '<Root>/measure'

  if (rtDW.DiscreteTimeIntegratory_IC_LOAD != 0) {
    rtDW.DiscreteTimeIntegratory_DSTATE[0] = rtU.measure.s_Kg[0];
    rtDW.DiscreteTimeIntegratory_DSTATE[1] = rtU.measure.s_Kg[1];
    rtDW.DiscreteTimeIntegratory_DSTATE[2] = rtU.measure.s_Kg[2];
  }

  // DiscreteIntegrator: '<S91>/Discrete-Time Integrator y' incorporates:
  //   Inport: '<Root>/measure'

  if (rtDW.DiscreteTimeIntegratory_IC_LO_p != 0) {
    rtDW.DiscreteTimeIntegratory_DSTAT_a[0] = rtU.measure.V_Kg[0];
    rtDW.DiscreteTimeIntegratory_DSTAT_a[1] = rtU.measure.V_Kg[1];
    rtDW.DiscreteTimeIntegratory_DSTAT_a[2] = rtU.measure.V_Kg[2];
  }

  // RelationalOperator: '<S143>/Compare' incorporates:
  //   Constant: '<S143>/Constant'
  //   Sum: '<S46>/Add2'
  //   UnitDelay: '<S46>/Unit Delay1'

  rtb_is_slowing = (rtDW.Merge1 - rtDW.UnitDelay1_DSTATE != 0.0F);

  // MATLAB Function: '<Root>/Quaternions to Rotation Matrix' incorporates:
  //   Inport: '<Root>/measure'

  scale = 1.29246971E-26F;
  absxk = std::abs(rtU.measure.q_bg[0]);
  if (absxk > 1.29246971E-26F) {
    q0_q2 = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    q0_q2 = t * t;
  }

  absxk = std::abs(rtU.measure.q_bg[1]);
  if (absxk > scale) {
    t = scale / absxk;
    q0_q2 = q0_q2 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q0_q2 += t * t;
  }

  absxk = std::abs(rtU.measure.q_bg[2]);
  if (absxk > scale) {
    t = scale / absxk;
    q0_q2 = q0_q2 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q0_q2 += t * t;
  }

  absxk = std::abs(rtU.measure.q_bg[3]);
  if (absxk > scale) {
    t = scale / absxk;
    q0_q2 = q0_q2 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q0_q2 += t * t;
  }

  q0_q2 = scale * std::sqrt(q0_q2);
  if (2.22044605E-16F < q0_q2) {
    rtb_Gain6 = q0_q2;
  } else {
    rtb_Gain6 = 2.22044605E-16F;
  }

  rtb_Add_a[0] = rtU.measure.q_bg[0] / rtb_Gain6;
  rtb_Add_a[1] = rtU.measure.q_bg[1] / rtb_Gain6;
  rtb_Add_a[2] = rtU.measure.q_bg[2] / rtb_Gain6;
  rtb_Add_a[3] = rtU.measure.q_bg[3] / rtb_Gain6;
  rtb_Gain6 = rtb_Add_a[0] * rtb_Add_a[0];
  rtb_y_a0 = rtb_Add_a[1] * rtb_Add_a[1];
  cos_Theta = rtb_Add_a[2] * rtb_Add_a[2];
  scale = rtb_Add_a[3] * rtb_Add_a[3];
  t = rtb_Add_a[0] * rtb_Add_a[1];
  q0_q2 = rtb_Add_a[0] * rtb_Add_a[2];
  absxk = rtb_Add_a[0] * rtb_Add_a[3];
  q1_q2 = rtb_Add_a[1] * rtb_Add_a[2];
  q1_q3 = rtb_Add_a[1] * rtb_Add_a[3];
  q2_q3 = rtb_Add_a[2] * rtb_Add_a[3];
  rtb_M_bg[0] = ((rtb_Gain6 + rtb_y_a0) - cos_Theta) - scale;
  rtb_M_bg[3] = (q1_q2 + absxk) * 2.0F;
  rtb_M_bg[6] = (q1_q3 - q0_q2) * 2.0F;
  rtb_M_bg[1] = (q1_q2 - absxk) * 2.0F;
  rtb_Gain6 -= rtb_y_a0;
  rtb_M_bg[4] = (rtb_Gain6 + cos_Theta) - scale;
  rtb_M_bg[7] = (q2_q3 + t) * 2.0F;
  rtb_M_bg[2] = (q1_q3 + q0_q2) * 2.0F;
  rtb_M_bg[5] = (q2_q3 - t) * 2.0F;
  rtb_M_bg[8] = (rtb_Gain6 - cos_Theta) + scale;

  // End of MATLAB Function: '<Root>/Quaternions to Rotation Matrix'

  // Delay: '<S47>/Delay' incorporates:
  //   Inport: '<Root>/cmd'
  //   SignalConversion: '<Root>/BusConversion_InsertedFor_cmd_at_outport_0'

  if (rtDW.icLoad != 0) {
    rtDW.Delay_DSTATE_b = rtU.cmd.yaw_init;
  }

  // MATLAB Function: '<S47>/MATLAB Function1' incorporates:
  //   Delay: '<S47>/Delay'
  //   Inport: '<Root>/measure'

  q0_q2 = rtU.measure.s_Kg[0] * rtU.measure.s_Kg[0] + rtU.measure.s_Kg[1] *
    rtU.measure.s_Kg[1];
  if (0.0F >= q0_q2) {
    q0_q2 = 0.0F;
  }

  if (std::sqrt(q0_q2) > 10.0F) {
    rtb_Gain6 = std::atan2(rtU.measure.s_Kg[1], rtU.measure.s_Kg[0]);
  } else {
    rtb_Gain6 = rtDW.Delay_DSTATE_b;
  }

  // End of MATLAB Function: '<S47>/MATLAB Function1'

  // MATLAB Function: '<S47>/MATLAB Function' incorporates:
  //   Constant: '<S47>/Constant'
  //   Gain: '<Root>/Gain'
  //   Inport: '<Root>/cmd'
  //   MATLAB Function: '<S47>/Rotations matrix to Euler angles'
  //   SignalConversion: '<Root>/BusConversion_InsertedFor_cmd_at_outport_0'

  scale = std::atan2(rtb_M_bg[3], rtb_M_bg[0]);
  if (rtP.lindi.rllptch == 1.0F) {
    scale = rtU.cmd.yaw_init;
  } else {
    if (rtP.lindi.rllptch == 2.0F) {
      scale = rtb_Gain6;
    }
  }

  q1_q2 = std::cos(scale);
  scale = std::sin(scale);
  cos_Theta = rtDW.Merge[0] * q1_q2 + -rtDW.Merge[1] * scale;
  scale = -rtDW.Merge[1] * q1_q2 - rtDW.Merge[0] * scale;

  // End of MATLAB Function: '<S47>/MATLAB Function'

  // Outputs for Enabled SubSystem: '<S4>/NDI position controller for copters reference model' incorporates:
  //   EnablePort: '<S43>/Enable'

  if (rtb_is_finished) {
    if (!rtDW.NDIpositioncontrollerforcopte_k) {
      // InitializeConditions for DiscreteIntegrator: '<S43>/Discrete-Time Integrator1' 
      rtDW.DiscreteTimeIntegrator1_IC_LOAD = 1U;
      rtDW.DiscreteTimeIntegrator1_PrevRes = 2;

      // InitializeConditions for DiscreteIntegrator: '<S127>/Discrete-Time Integrator' 
      rtDW.DiscreteTimeIntegrator_IC_LOADI = 1U;
      rtDW.DiscreteTimeIntegrator_PrevRe_m = 2;

      // InitializeConditions for DiscreteIntegrator: '<S128>/Discrete-Time Integrator' 
      rtDW.DiscreteTimeIntegrator_IC_LOA_a = 1U;
      rtDW.DiscreteTimeIntegrator_PrevRe_i = 2;
      rtDW.NDIpositioncontrollerforcopte_k = true;
    }

    // DiscreteIntegrator: '<S43>/Discrete-Time Integrator1' incorporates:
    //   DiscreteIntegrator: '<S92>/Discrete-Time Integrator y'

    if (rtDW.DiscreteTimeIntegrator1_IC_LOAD != 0) {
      rtDW.DiscreteTimeIntegrator1_DSTATE[0] =
        rtDW.DiscreteTimeIntegratory_DSTATE[0];
      rtDW.DiscreteTimeIntegrator1_DSTATE[1] =
        rtDW.DiscreteTimeIntegratory_DSTATE[1];
      rtDW.DiscreteTimeIntegrator1_DSTATE[2] =
        rtDW.DiscreteTimeIntegratory_DSTATE[2];
    }

    if (rtb_is_slowing && (rtDW.DiscreteTimeIntegrator1_PrevRes <= 0)) {
      rtDW.DiscreteTimeIntegrator1_DSTATE[0] =
        rtDW.DiscreteTimeIntegratory_DSTATE[0];
      rtDW.DiscreteTimeIntegrator1_DSTATE[1] =
        rtDW.DiscreteTimeIntegratory_DSTATE[1];
      rtDW.DiscreteTimeIntegrator1_DSTATE[2] =
        rtDW.DiscreteTimeIntegratory_DSTATE[2];
    }

    // SignalConversion: '<S43>/BusConversion_InsertedFor_reference_at_inport_0' incorporates:
    //   DiscreteIntegrator: '<S43>/Discrete-Time Integrator1'

    rtDW.s_g_ref[0] = rtDW.DiscreteTimeIntegrator1_DSTATE[0];
    rtDW.s_g_ref[1] = rtDW.DiscreteTimeIntegrator1_DSTATE[1];
    rtDW.s_g_ref[2] = rtDW.DiscreteTimeIntegrator1_DSTATE[2];

    // DiscreteIntegrator: '<S127>/Discrete-Time Integrator' incorporates:
    //   DiscreteIntegrator: '<S91>/Discrete-Time Integrator y'

    if (rtDW.DiscreteTimeIntegrator_IC_LOADI != 0) {
      rtDW.DiscreteTimeIntegrator_DSTATE_c[0] =
        rtDW.DiscreteTimeIntegratory_DSTAT_a[0];
      rtDW.DiscreteTimeIntegrator_DSTATE_c[1] =
        rtDW.DiscreteTimeIntegratory_DSTAT_a[1];
    }

    if (rtb_is_slowing && (rtDW.DiscreteTimeIntegrator_PrevRe_m <= 0)) {
      rtDW.DiscreteTimeIntegrator_DSTATE_c[0] =
        rtDW.DiscreteTimeIntegratory_DSTAT_a[0];
      rtDW.DiscreteTimeIntegrator_DSTATE_c[1] =
        rtDW.DiscreteTimeIntegratory_DSTAT_a[1];
    }

    // DiscreteIntegrator: '<S128>/Discrete-Time Integrator' incorporates:
    //   DiscreteIntegrator: '<S91>/Discrete-Time Integrator y'

    if (rtDW.DiscreteTimeIntegrator_IC_LOA_a != 0) {
      rtDW.DiscreteTimeIntegrator_DSTATE_n =
        rtDW.DiscreteTimeIntegratory_DSTAT_a[2];
    }

    if (rtb_is_slowing && (rtDW.DiscreteTimeIntegrator_PrevRe_i <= 0)) {
      rtDW.DiscreteTimeIntegrator_DSTATE_n =
        rtDW.DiscreteTimeIntegratory_DSTAT_a[2];
    }

    // SignalConversion: '<S43>/BusConversion_InsertedFor_reference_at_inport_0' incorporates:
    //   DiscreteIntegrator: '<S127>/Discrete-Time Integrator'
    //   DiscreteIntegrator: '<S128>/Discrete-Time Integrator'

    rtDW.s_g_ref_dt[0] = rtDW.DiscreteTimeIntegrator_DSTATE_c[0];
    rtDW.s_g_ref_dt[1] = rtDW.DiscreteTimeIntegrator_DSTATE_c[1];
    rtDW.s_g_ref_dt[2] = rtDW.DiscreteTimeIntegrator_DSTATE_n;

    // MATLAB Function: '<S131>/stickRP2LeanCmd' incorporates:
    //   Gain: '<S129>/Gain'

    rtb_y_a0 = std::atan2(cos_Theta, scale);
    absxk = std::sqrt(cos_Theta * cos_Theta + -scale * -scale);
    if (absxk > 1.0F) {
      absxk = 1.0F;
    }

    // Product: '<S129>/Multiply1' incorporates:
    //   MATLAB Function: '<S131>/stickRP2LeanCmd'
    //   Trigonometry: '<S129>/Sin'

    q0_q2 = absxk * std::sin(rtb_y_a0);

    // Product: '<S127>/Divide' incorporates:
    //   Constant: '<S127>/T'
    //   DiscreteIntegrator: '<S127>/Discrete-Time Integrator'
    //   Gain: '<S43>/velxymax'
    //   MATLAB Function: '<S131>/stickRP2LeanCmd'
    //   Product: '<S129>/Multiply'
    //   Sum: '<S127>/Sum2'
    //   Trigonometry: '<S129>/Cos'

    absxk = (absxk * std::cos(rtb_y_a0) * rtP.lindi.psc.rm.velxymax -
             rtDW.DiscreteTimeIntegrator_DSTATE_c[0]) / rtP.lindi.psc.rm.veltc;

    // Saturate: '<S127>/Saturation'
    if (absxk > rtP.lindi.psc.rm.accxymax) {
      absxk = rtP.lindi.psc.rm.accxymax;
    } else {
      if (absxk < -rtP.lindi.psc.rm.accxymax) {
        absxk = -rtP.lindi.psc.rm.accxymax;
      }
    }

    // Product: '<S127>/Divide' incorporates:
    //   Constant: '<S127>/T'
    //   DiscreteIntegrator: '<S127>/Discrete-Time Integrator'
    //   Gain: '<S43>/velxymax'
    //   Sum: '<S127>/Sum2'

    q0_q2 = (rtP.lindi.psc.rm.velxymax * q0_q2 -
             rtDW.DiscreteTimeIntegrator_DSTATE_c[1]) / rtP.lindi.psc.rm.veltc;

    // Saturate: '<S127>/Saturation'
    if (q0_q2 > rtP.lindi.psc.rm.accxymax) {
      q0_q2 = rtP.lindi.psc.rm.accxymax;
    } else {
      if (q0_q2 < -rtP.lindi.psc.rm.accxymax) {
        q0_q2 = -rtP.lindi.psc.rm.accxymax;
      }
    }

    // MATLAB Function: '<S130>/MATLAB Function' incorporates:
    //   Gain: '<Root>/Gain5'

    if (-rtDW.Merge[3] < 0.0F) {
      q1_q3 = -rtP.lindi.psc.rm.veldmax * -rtDW.Merge[3];
    } else {
      q1_q3 = -rtP.lindi.psc.rm.velumax * -rtDW.Merge[3];
    }

    // End of MATLAB Function: '<S130>/MATLAB Function'

    // Product: '<S128>/Divide' incorporates:
    //   Constant: '<S128>/T'
    //   DiscreteIntegrator: '<S128>/Discrete-Time Integrator'
    //   Sum: '<S128>/Sum2'

    t = (q1_q3 - rtDW.DiscreteTimeIntegrator_DSTATE_n) / rtP.lindi.psc.rm.veltc;

    // Saturate: '<S128>/Saturation'
    if (t > rtP.lindi.psc.rm.accdmax) {
      t = rtP.lindi.psc.rm.accdmax;
    } else {
      if (t < -rtP.lindi.psc.rm.accumax) {
        t = -rtP.lindi.psc.rm.accumax;
      }
    }

    // End of Saturate: '<S128>/Saturation'

    // SignalConversion: '<S43>/BusConversion_InsertedFor_reference_at_inport_0' 
    rtDW.s_g_ref_dt2[2] = t;

    // Update for DiscreteIntegrator: '<S43>/Discrete-Time Integrator1'
    rtDW.DiscreteTimeIntegrator1_IC_LOAD = 0U;

    // SignalConversion: '<S43>/BusConversion_InsertedFor_reference_at_inport_0' 
    rtDW.s_g_ref_dt2[0] = absxk;
    rtDW.s_g_ref_dt2[1] = q0_q2;

    // Update for DiscreteIntegrator: '<S43>/Discrete-Time Integrator1' incorporates:
    //   DiscreteIntegrator: '<S127>/Discrete-Time Integrator'
    //   DiscreteIntegrator: '<S128>/Discrete-Time Integrator'

    rtDW.DiscreteTimeIntegrator1_DSTATE[0] += 0.0025F *
      rtDW.DiscreteTimeIntegrator_DSTATE_c[0];
    rtDW.DiscreteTimeIntegrator1_DSTATE[1] += 0.0025F *
      rtDW.DiscreteTimeIntegrator_DSTATE_c[1];
    rtDW.DiscreteTimeIntegrator1_DSTATE[2] += 0.0025F *
      rtDW.DiscreteTimeIntegrator_DSTATE_n;
    rtDW.DiscreteTimeIntegrator1_PrevRes = (int8_T)rtb_is_slowing;

    // Update for DiscreteIntegrator: '<S127>/Discrete-Time Integrator'
    rtDW.DiscreteTimeIntegrator_IC_LOADI = 0U;
    rtDW.DiscreteTimeIntegrator_DSTATE_c[0] += 0.0025F * absxk;
    rtDW.DiscreteTimeIntegrator_DSTATE_c[1] += 0.0025F * q0_q2;
    rtDW.DiscreteTimeIntegrator_PrevRe_m = (int8_T)rtb_is_slowing;

    // Update for DiscreteIntegrator: '<S128>/Discrete-Time Integrator'
    rtDW.DiscreteTimeIntegrator_IC_LOA_a = 0U;
    rtDW.DiscreteTimeIntegrator_DSTATE_n += 0.0025F * t;
    rtDW.DiscreteTimeIntegrator_PrevRe_i = (int8_T)rtb_is_slowing;
  } else {
    if (rtDW.NDIpositioncontrollerforcopte_k) {
      rtDW.NDIpositioncontrollerforcopte_k = false;
    }
  }

  // End of Outputs for SubSystem: '<S4>/NDI position controller for copters reference model' 

  // Outputs for Enabled SubSystem: '<S4>/Waypoint Navigation' incorporates:
  //   EnablePort: '<S49>/Enable'

  if (rtb_Compare_i) {
    if (!rtDW.WaypointNavigation_MODE) {
      rtDW.WaypointNavigation_MODE = true;
    }

    // MATLAB Function: '<S49>/Split waypoints and velocity' incorporates:
    //   Inport: '<Root>/cmd'
    //   SignalConversion: '<Root>/BusConversion_InsertedFor_cmd_at_outport_0'

    for (wp_idx_app_2 = 0; wp_idx_app_2 < 10; wp_idx_app_2++) {
      flight_mode = wp_idx_app_2 << 2;
      rtb_wp[3 * wp_idx_app_2] = rtU.cmd.waypoints[flight_mode];
      rtb_wp[1 + 3 * wp_idx_app_2] = rtU.cmd.waypoints[flight_mode + 1];
      rtb_wp[2 + 3 * wp_idx_app_2] = rtU.cmd.waypoints[flight_mode + 2];
    }

    // MATLAB Function: '<S49>/Avoid zero speed' incorporates:
    //   DiscreteIntegrator: '<S91>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S92>/Discrete-Time Integrator y'
    //   UnitDelay: '<S49>/Unit Delay'

    rtb_Sum2_e[0] = rtDW.DiscreteTimeIntegratory_DSTAT_a[0];
    rtb_Sum2_e[1] = rtDW.DiscreteTimeIntegratory_DSTAT_a[1];
    rtb_Sum2_e[2] = rtDW.DiscreteTimeIntegratory_DSTAT_a[2];
    if (norm(rtDW.DiscreteTimeIntegratory_DSTAT_a) < 0.3) {
      rtb_DataTypeConversion2 = (rtDW.UnitDelay_DSTATE_o - 1) * 3;
      rtb_Sum2_e[0] = rtb_wp[rtb_DataTypeConversion2] -
        rtDW.DiscreteTimeIntegratory_DSTATE[0];
      rtb_Sum2_e[1] = rtb_wp[rtb_DataTypeConversion2 + 1] -
        rtDW.DiscreteTimeIntegratory_DSTATE[1];
      rtb_Sum2_e[2] = rtb_wp[rtb_DataTypeConversion2 + 2] -
        rtDW.DiscreteTimeIntegratory_DSTATE[2];
      t = norm(rtb_Sum2_e);
      if (std::abs(t) < 2.22044605E-16F) {
        t = 2.22044605E-16F;
      }

      rtb_Sum2_e[0] = (rtb_wp[rtb_DataTypeConversion2] -
                       rtDW.DiscreteTimeIntegratory_DSTATE[0]) / t * 0.3F;
      rtb_Sum2_e[1] = (rtb_wp[rtb_DataTypeConversion2 + 1] -
                       rtDW.DiscreteTimeIntegratory_DSTATE[1]) / t * 0.3F;
      rtb_Sum2_e[2] = (rtb_wp[rtb_DataTypeConversion2 + 2] -
                       rtDW.DiscreteTimeIntegratory_DSTATE[2]) / t * 0.3F;
    }

    // End of MATLAB Function: '<S49>/Avoid zero speed'

    // DataTypeConversion: '<S49>/Data Type Conversion2' incorporates:
    //   Inport: '<Root>/cmd'
    //   SignalConversion: '<Root>/BusConversion_InsertedFor_cmd_at_outport_0'

    rtb_DataTypeConversion2 = rtU.cmd.num_waypoints;

    // MATLAB Function: '<S49>/Look Ahead' incorporates:
    //   Constant: '<S49>/Constant3'
    //   Constant: '<S49>/Constant5'
    //   MATLAB Function: '<S49>/Look Ahead1'

    q2_q3 = (2.0F / rtP.lindi.atc.rm.leanfreq + rtP.lindi.mtc) + 2.0F /
      rtP.lindi.sflt.omega;
    t = q2_q3 + rtP.lindi.wpnav.T;

    // MATLAB Function: '<S49>/WpNav Matching' incorporates:
    //   UnitDelay: '<S49>/Unit Delay'
    //   UnitDelay: '<S49>/Unit Delay1'
    //   UnitDelay: '<S49>/Unit Delay2'
    //   UnitDelay: '<S49>/Unit Delay3'
    //   UnitDelay: '<S49>/Unit Delay5'

    c_stage_app = rtDW.UnitDelay3_DSTATE_m;
    k = rtDW.UnitDelay2_DSTATE;
    rtb_is_finished = rtDW.UnitDelay5_DSTATE_n;
    b_stage_ = rtDW.UnitDelay1_DSTATE_f;
    flight_mode = rtDW.UnitDelay_DSTATE_o;

    // MATLAB Function: '<S49>/Look Ahead' incorporates:
    //   Constant: '<S49>/Constant5'
    //   DiscreteIntegrator: '<S89>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S91>/Discrete-Time Integrator y'
    //   UnitDelay: '<S49>/Unit Delay6'

    rtb_p_ahead_l[0] = 0.5F * rtDW.DiscreteTimeIntegratory_DSTA_al[0] * t * t +
      (rtDW.DiscreteTimeIntegratory_DSTAT_a[0] * t + rtDW.UnitDelay6_DSTATE[0]);

    // MATLAB Function: '<S49>/WpNav Matching'
    rtb_y_o[0] = 0.0F;

    // MATLAB Function: '<S49>/Look Ahead' incorporates:
    //   Constant: '<S49>/Constant5'
    //   DiscreteIntegrator: '<S89>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S91>/Discrete-Time Integrator y'
    //   UnitDelay: '<S49>/Unit Delay6'

    rtb_p_ahead_l[1] = 0.5F * rtDW.DiscreteTimeIntegratory_DSTA_al[1] * t * t +
      (rtDW.DiscreteTimeIntegratory_DSTAT_a[1] * t + rtDW.UnitDelay6_DSTATE[1]);

    // MATLAB Function: '<S49>/WpNav Matching'
    rtb_y_o[1] = 0.0F;

    // MATLAB Function: '<S49>/Look Ahead' incorporates:
    //   Constant: '<S49>/Constant5'
    //   DiscreteIntegrator: '<S89>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S91>/Discrete-Time Integrator y'
    //   UnitDelay: '<S49>/Unit Delay6'

    rtb_p_ahead_l[2] = 0.5F * rtDW.DiscreteTimeIntegratory_DSTA_al[2] * t * t +
      (rtDW.DiscreteTimeIntegratory_DSTAT_a[2] * t + rtDW.UnitDelay6_DSTATE[2]);

    // MATLAB Function: '<S49>/WpNav Matching' incorporates:
    //   Constant: '<S155>/wp_rad_fix'
    //   Constant: '<S49>/Constant2'
    //   DiscreteIntegrator: '<S92>/Discrete-Time Integrator y'
    //   Inport: '<Root>/cmd'
    //   MATLAB Function: '<S49>/Split waypoints and velocity'
    //   SignalConversion: '<Root>/BusConversion_InsertedFor_cmd_at_outport_0'
    //   Switch: '<S155>/Switch'
    //   UnitDelay: '<S49>/Unit Delay'
    //   UnitDelay: '<S49>/Unit Delay1'
    //   UnitDelay: '<S49>/Unit Delay2'
    //   UnitDelay: '<S49>/Unit Delay3'
    //   UnitDelay: '<S49>/Unit Delay4'
    //   UnitDelay: '<S49>/Unit Delay5'

    rtb_y_o[2] = 0.0F;
    for (wp_idx_app_2 = 0; wp_idx_app_2 < 15; wp_idx_app_2++) {
      wp_approach_out[wp_idx_app_2] = rtDW.UnitDelay4_DSTATE[wp_idx_app_2];
    }

    if (rtU.cmd.num_waypoints < 2) {
      rtb_DataTypeConversion2 = 2;
    }

    if (rtDW.UnitDelay5_DSTATE_n) {
      wpnavMatch(rtDW.UnitDelay4_DSTATE, rtP.lindi.wpnav.wprad, &k, &c_stage_app,
                 rtDW.DiscreteTimeIntegratory_DSTATE, a_Kb_meas, &t, &q0_q2);
      wp_idx_app_2 = k;
      stage_app_2 = c_stage_app;
      wpnavMatch(rtDW.UnitDelay4_DSTATE, rtP.lindi.wpnav.wprad, &wp_idx_app_2,
                 &stage_app_2, rtb_p_ahead_l, rtb_n_g_des_n, &q0_q2, &absxk);
      stage_2 = rtDW.UnitDelay1_DSTATE_f;
      wp_idx_2 = rtDW.UnitDelay_DSTATE_o;
      stage_tmp = rtDW.UnitDelay1_DSTATE_f;
      wp_idx_tmp = rtDW.UnitDelay_DSTATE_o;
    } else {
      wp_idx_tmp = rtDW.UnitDelay_DSTATE_o;
      stage_tmp = rtDW.UnitDelay1_DSTATE_f;
      rtb_wp_size_3[0] = 3;
      rtb_wp_size_3[1] = rtb_DataTypeConversion2;
      for (wp_idx_app_2 = 0; wp_idx_app_2 < rtb_DataTypeConversion2;
           wp_idx_app_2++) {
        rtb_wp_data[3 * wp_idx_app_2] = rtb_wp[3 * wp_idx_app_2];
        rtb_wp_data[1 + 3 * wp_idx_app_2] = rtb_wp[3 * wp_idx_app_2 + 1];
        rtb_wp_data[2 + 3 * wp_idx_app_2] = rtb_wp[3 * wp_idx_app_2 + 2];
      }

      wpnavMatch_b(rtb_wp_data, rtb_wp_size_3, rtP.lindi.wpnav.wprad,
                   &wp_idx_tmp, &stage_tmp, rtDW.DiscreteTimeIntegratory_DSTATE,
                   a_Kb_meas, &t, &q0_q2);
      wp_idx_2 = rtDW.UnitDelay_DSTATE_o;
      stage_2 = rtDW.UnitDelay1_DSTATE_f;
      rtb_wp_size_2[0] = 3;
      rtb_wp_size_2[1] = rtb_DataTypeConversion2;
      for (wp_idx_app_2 = 0; wp_idx_app_2 < rtb_DataTypeConversion2;
           wp_idx_app_2++) {
        rtb_wp_data[3 * wp_idx_app_2] = rtb_wp[3 * wp_idx_app_2];
        rtb_wp_data[1 + 3 * wp_idx_app_2] = rtb_wp[3 * wp_idx_app_2 + 1];
        rtb_wp_data[2 + 3 * wp_idx_app_2] = rtb_wp[3 * wp_idx_app_2 + 2];
      }

      wpnavMatch_b(rtb_wp_data, rtb_wp_size_2, rtP.lindi.wpnav.wprad, &wp_idx_2,
                   &stage_2, rtb_p_ahead_l, rtb_n_g_des_n, &q0_q2, &absxk);
      stage_app_2 = rtDW.UnitDelay3_DSTATE_m;
      wp_idx_app_2 = rtDW.UnitDelay2_DSTATE;
    }

    rtb_n_dt[0] = a_Kb_meas[0] - rtDW.DiscreteTimeIntegratory_DSTATE[0];
    rtb_n_dt[1] = a_Kb_meas[1] - rtDW.DiscreteTimeIntegratory_DSTATE[1];
    rtb_n_dt[2] = a_Kb_meas[2] - rtDW.DiscreteTimeIntegratory_DSTATE[2];
    if (norm(rtb_n_dt) > rtP.lindi.wpnav.eposmax) {
      a_Kb_meas[0] = rtb_wp[(rtDW.UnitDelay_DSTATE_o - 1) * 3] -
        rtDW.DiscreteTimeIntegratory_DSTATE[0];
      a_Kb_meas[1] = rtb_wp[(rtDW.UnitDelay_DSTATE_o - 1) * 3 + 1] -
        rtDW.DiscreteTimeIntegratory_DSTATE[1];
      a_Kb_meas[2] = rtb_wp[(rtDW.UnitDelay_DSTATE_o - 1) * 3 + 2] -
        rtDW.DiscreteTimeIntegratory_DSTATE[2];
      rtb_y_a0 = norm(a_Kb_meas);
      q0_q2 = norm(rtb_Sum2_e);
      if (std::abs(q0_q2) < 2.22044605E-16F) {
        q0_q2 = 2.22044605E-16F;
      }

      q1_q2 = rtb_Sum2_e[0] / q0_q2;
      absxk = (rtb_wp[(rtDW.UnitDelay_DSTATE_o - 1) * 3] -
               rtDW.DiscreteTimeIntegratory_DSTATE[0]) * q1_q2;
      rtb_Sum2_e[0] = q1_q2;
      q1_q2 = rtb_Sum2_e[1] / q0_q2;
      absxk += (rtb_wp[(rtDW.UnitDelay_DSTATE_o - 1) * 3 + 1] -
                rtDW.DiscreteTimeIntegratory_DSTATE[1]) * q1_q2;
      rtb_Sum2_e[1] = q1_q2;
      q1_q2 = rtb_Sum2_e[2] / q0_q2;
      absxk += (rtb_wp[(rtDW.UnitDelay_DSTATE_o - 1) * 3 + 2] -
                rtDW.DiscreteTimeIntegratory_DSTATE[2]) * q1_q2;
      rtb_Sum2_e[2] = q1_q2;
      q0_q2 = rtb_y_a0 * norm(rtb_Sum2_e);
      if (std::abs(q0_q2) < 2.22044605E-16F) {
        q0_q2 = 2.22044605E-16F;
      }

      q0_q2 = absxk / q0_q2;
      if (1.0F <= q0_q2) {
        q0_q2 = 1.0F;
      }

      if (-1.0F >= q0_q2) {
        q0_q2 = -1.0F;
      }

      if ((std::acos(q0_q2) < 0.5F) && (rtb_y_a0 < rtP.lindi.wpnav.wprad)) {
        t = 0.5F * rtb_y_a0;
      } else {
        t = 1.2F * rtP.lindi.wpnav.wprad;
      }

      wp_approach_out_tmp = rtb_Sum2_e[0] * t;
      wp_approach_out[0] = rtDW.DiscreteTimeIntegratory_DSTATE[0] -
        wp_approach_out_tmp;
      wp_approach_out[3] = wp_approach_out_tmp +
        rtDW.DiscreteTimeIntegratory_DSTATE[0];
      wp_approach_out[6] = rtb_wp[(rtDW.UnitDelay_DSTATE_o - 1) * 3];
      wp_approach_out_tmp = rtb_Sum2_e[1] * t;
      wp_approach_out[1] = rtDW.DiscreteTimeIntegratory_DSTATE[1] -
        wp_approach_out_tmp;
      wp_approach_out[4] = wp_approach_out_tmp +
        rtDW.DiscreteTimeIntegratory_DSTATE[1];
      wp_approach_out[7] = rtb_wp[(rtDW.UnitDelay_DSTATE_o - 1) * 3 + 1];
      wp_approach_out_tmp = q1_q2 * t;
      wp_approach_out[2] = rtDW.DiscreteTimeIntegratory_DSTATE[2] -
        wp_approach_out_tmp;
      wp_approach_out[5] = wp_approach_out_tmp +
        rtDW.DiscreteTimeIntegratory_DSTATE[2];
      wp_approach_out[8] = rtb_wp[(rtDW.UnitDelay_DSTATE_o - 1) * 3 + 2];
      if (rtb_DataTypeConversion2 - 1 == rtDW.UnitDelay_DSTATE_o) {
        wp_approach_out[9] = rtb_wp[(rtDW.UnitDelay_DSTATE_o - 1) * 3];
        wp_approach_out[12] = rtb_wp[0];
        wp_approach_out[10] = rtb_wp[(rtDW.UnitDelay_DSTATE_o - 1) * 3 + 1];
        wp_approach_out[13] = rtb_wp[1];
        wp_approach_out[11] = rtb_wp[(rtDW.UnitDelay_DSTATE_o - 1) * 3 + 2];
        wp_approach_out[14] = rtb_wp[2];
      } else if (rtDW.UnitDelay_DSTATE_o == rtb_DataTypeConversion2) {
        wp_approach_out[9] = rtb_wp[0];
        wp_approach_out[12] = rtb_wp[3];
        wp_approach_out[10] = rtb_wp[1];
        wp_approach_out[13] = rtb_wp[4];
        wp_approach_out[11] = rtb_wp[2];
        wp_approach_out[14] = rtb_wp[5];
      } else {
        if (rtDW.UnitDelay_DSTATE_o > 2147483646) {
          wp_idx_2 = MAX_int32_T;
        } else {
          wp_idx_2 = rtDW.UnitDelay_DSTATE_o + 1;
        }

        if (rtDW.UnitDelay_DSTATE_o > 2147483645) {
          wp_idx_app_2 = MAX_int32_T;
        } else {
          wp_idx_app_2 = rtDW.UnitDelay_DSTATE_o + 2;
        }

        c_stage_app = (wp_idx_2 - 1) * 3;
        wp_approach_out[9] = rtb_wp[c_stage_app];
        wp_idx_app_2 = (wp_idx_app_2 - 1) * 3;
        wp_approach_out[12] = rtb_wp[wp_idx_app_2];
        wp_approach_out[10] = rtb_wp[c_stage_app + 1];
        wp_approach_out[13] = rtb_wp[wp_idx_app_2 + 1];
        wp_approach_out[11] = rtb_wp[c_stage_app + 2];
        wp_approach_out[14] = rtb_wp[wp_idx_app_2 + 2];
      }

      rtb_is_finished = true;
      k = 2;
      c_stage_app = 1;
      wpnavMatch(wp_approach_out, rtP.lindi.wpnav.wprad, &k, &c_stage_app,
                 rtDW.DiscreteTimeIntegratory_DSTATE, a_Kb_meas, &t, &q0_q2);
      wp_idx_app_2 = k;
      stage_app_2 = c_stage_app;
      wpnavMatch(wp_approach_out, rtP.lindi.wpnav.wprad, &wp_idx_app_2,
                 &stage_app_2, rtb_p_ahead_l, rtb_n_g_des_n, &q0_q2, &absxk);
      stage_2 = rtDW.UnitDelay1_DSTATE_f;
      wp_idx_2 = rtDW.UnitDelay_DSTATE_o;
    } else {
      flight_mode = wp_idx_tmp;
      b_stage_ = stage_tmp;
    }

    if (((k == 4) && (c_stage_app == 1)) || (k == 5)) {
      k = 2;
      rtb_is_finished = false;
      if (flight_mode > 2147483646) {
        wp_idx_2 = MAX_int32_T;
      } else {
        wp_idx_2 = flight_mode + 1;
      }

      flight_mode = wp_idx_2;
      b_stage_ = 1;
      rtb_wp_size_1[0] = 3;
      rtb_wp_size_1[1] = rtb_DataTypeConversion2;
      for (wp_idx_app_2 = 0; wp_idx_app_2 < rtb_DataTypeConversion2;
           wp_idx_app_2++) {
        rtb_wp_data[3 * wp_idx_app_2] = rtb_wp[3 * wp_idx_app_2];
        rtb_wp_data[1 + 3 * wp_idx_app_2] = rtb_wp[3 * wp_idx_app_2 + 1];
        rtb_wp_data[2 + 3 * wp_idx_app_2] = rtb_wp[3 * wp_idx_app_2 + 2];
      }

      wpnavMatch_b(rtb_wp_data, rtb_wp_size_1, rtP.lindi.wpnav.wprad,
                   &flight_mode, &b_stage_, rtDW.DiscreteTimeIntegratory_DSTATE,
                   a_Kb_meas, &t, &q0_q2);
      if (flight_mode != wp_idx_2) {
        flight_mode = wp_idx_2 - 1;
        rtb_wp_size_0[0] = 3;
        rtb_wp_size_0[1] = rtb_DataTypeConversion2;
        for (wp_idx_app_2 = 0; wp_idx_app_2 < rtb_DataTypeConversion2;
             wp_idx_app_2++) {
          rtb_wp_data[3 * wp_idx_app_2] = rtb_wp[3 * wp_idx_app_2];
          rtb_wp_data[1 + 3 * wp_idx_app_2] = rtb_wp[3 * wp_idx_app_2 + 1];
          rtb_wp_data[2 + 3 * wp_idx_app_2] = rtb_wp[3 * wp_idx_app_2 + 2];
        }

        wpnavMatch_b(rtb_wp_data, rtb_wp_size_0, rtP.lindi.wpnav.wprad,
                     &flight_mode, &b_stage_,
                     rtDW.DiscreteTimeIntegratory_DSTATE, a_Kb_meas, &t, &q0_q2);
      }

      wp_idx_2 = flight_mode;
      stage_2 = b_stage_;
      rtb_wp_size[0] = 3;
      rtb_wp_size[1] = rtb_DataTypeConversion2;
      for (wp_idx_app_2 = 0; wp_idx_app_2 < rtb_DataTypeConversion2;
           wp_idx_app_2++) {
        rtb_wp_data[3 * wp_idx_app_2] = rtb_wp[3 * wp_idx_app_2];
        rtb_wp_data[1 + 3 * wp_idx_app_2] = rtb_wp[3 * wp_idx_app_2 + 1];
        rtb_wp_data[2 + 3 * wp_idx_app_2] = rtb_wp[3 * wp_idx_app_2 + 2];
      }

      wpnavMatch_b(rtb_wp_data, rtb_wp_size, rtP.lindi.wpnav.wprad, &wp_idx_2,
                   &stage_2, rtb_p_ahead_l, rtb_n_g_des_n, &q0_q2, &absxk);
      stage_app_2 = c_stage_app;
      wp_idx_app_2 = 2;
    }

    if (flight_mode < -2147483647) {
      wp_idx_tmp = MIN_int32_T;
    } else {
      wp_idx_tmp = flight_mode - 1;
    }

    wp_idx_tmp--;
    if (wp_idx_tmp + 1 <= 0) {
      wp_idx_tmp = 9;
    }

    if (b_stage_ == 0) {
      t = rtU.cmd.waypoints[(wp_idx_tmp << 2) + 3];
    } else {
      // MATLAB Function: '<S49>/Split waypoints and velocity'
      rtb_y_a0 = rtU.cmd.waypoints[(wp_idx_tmp << 2) + 3];
      t = (rtU.cmd.waypoints[((flight_mode - 1) << 2) + 3] - rtb_y_a0) * t +
        rtb_y_a0;
    }

    if (((stage_2 == 0) && (!rtb_is_finished)) || ((stage_app_2 == 0) &&
         rtb_is_finished)) {
      stage_2 = wp_idx_2 - 3;
      rtb_DataTypeConversion2_0[0] = (uint16_T)(rtb_DataTypeConversion2 - 1);
      rtb_DataTypeConversion2_0[1] = 0U;
      rtb_DataTypeConversion2_0[2] = 1U;
      rtb_DataTypeConversion2_1[0] = (uint16_T)(rtb_DataTypeConversion2 - 2);
      rtb_DataTypeConversion2_1[1] = (uint16_T)(rtb_DataTypeConversion2 - 1);
      rtb_DataTypeConversion2_1[2] = 0U;
      rtb_DataTypeConversion2 = wp_idx_app_2 - 3;
      if (!rtb_is_finished) {
        if (wp_idx_2 > 2) {
          for (wp_idx_app_2 = 0; wp_idx_app_2 < 3; wp_idx_app_2++) {
            wp_idx_2 = (wp_idx_app_2 + stage_2) * 3;
            cos_Theta_0[3 * wp_idx_app_2] = rtb_wp[wp_idx_2];
            cos_Theta_0[1 + 3 * wp_idx_app_2] = rtb_wp[wp_idx_2 + 1];
            cos_Theta_0[2 + 3 * wp_idx_app_2] = rtb_wp[wp_idx_2 + 2];
          }
        } else if (wp_idx_2 == 2) {
          for (wp_idx_app_2 = 0; wp_idx_app_2 < 3; wp_idx_app_2++) {
            wp_idx_2 = 3 * rtb_DataTypeConversion2_0[wp_idx_app_2];
            cos_Theta_0[3 * wp_idx_app_2] = rtb_wp[wp_idx_2];
            cos_Theta_0[1 + 3 * wp_idx_app_2] = rtb_wp[wp_idx_2 + 1];
            cos_Theta_0[2 + 3 * wp_idx_app_2] = rtb_wp[wp_idx_2 + 2];
          }
        } else {
          for (wp_idx_app_2 = 0; wp_idx_app_2 < 3; wp_idx_app_2++) {
            wp_idx_2 = 3 * rtb_DataTypeConversion2_1[wp_idx_app_2];
            cos_Theta_0[3 * wp_idx_app_2] = rtb_wp[wp_idx_2];
            cos_Theta_0[1 + 3 * wp_idx_app_2] = rtb_wp[wp_idx_2 + 1];
            cos_Theta_0[2 + 3 * wp_idx_app_2] = rtb_wp[wp_idx_2 + 2];
          }
        }
      } else if (wp_idx_app_2 > 2) {
        for (wp_idx_app_2 = 0; wp_idx_app_2 < 3; wp_idx_app_2++) {
          wp_idx_2 = (wp_idx_app_2 + rtb_DataTypeConversion2) * 3;
          cos_Theta_0[3 * wp_idx_app_2] = wp_approach_out[wp_idx_2];
          cos_Theta_0[1 + 3 * wp_idx_app_2] = wp_approach_out[wp_idx_2 + 1];
          cos_Theta_0[2 + 3 * wp_idx_app_2] = wp_approach_out[wp_idx_2 + 2];
        }
      } else if (wp_idx_app_2 == 2) {
        for (wp_idx_app_2 = 0; wp_idx_app_2 < 3; wp_idx_app_2++) {
          wp_idx_2 = 3 * i[wp_idx_app_2];
          cos_Theta_0[3 * wp_idx_app_2] = wp_approach_out[wp_idx_2];
          cos_Theta_0[1 + 3 * wp_idx_app_2] = wp_approach_out[wp_idx_2 + 1];
          cos_Theta_0[2 + 3 * wp_idx_app_2] = wp_approach_out[wp_idx_2 + 2];
        }
      } else {
        for (wp_idx_app_2 = 0; wp_idx_app_2 < 3; wp_idx_app_2++) {
          wp_idx_2 = 3 * h[wp_idx_app_2];
          cos_Theta_0[3 * wp_idx_app_2] = wp_approach_out[wp_idx_2];
          cos_Theta_0[1 + 3 * wp_idx_app_2] = wp_approach_out[wp_idx_2 + 1];
          cos_Theta_0[2 + 3 * wp_idx_app_2] = wp_approach_out[wp_idx_2 + 2];
        }
      }

      wpnavCircSeg(cos_Theta_0, rtP.lindi.wpnav.wprad, &absxk, rtb_Sum2_hq,
                   rtb_n_b_dt2, &q1_q2, rtb_n, rtb_y_o, &q1_q3, rtb_p_ahead_l);
      rtb_n[0] -= rtb_Sum2_hq[0];
      rtb_n[1] -= rtb_Sum2_hq[1];
      wp_approach_out_tmp = rtb_n[2] - rtb_Sum2_hq[2];
      rtb_n[2] = wp_approach_out_tmp;
      rtb_Sum2_e[0] = rtb_n_b_dt2[1] * wp_approach_out_tmp - rtb_n_b_dt2[2] *
        rtb_n[1];
      rtb_Sum2_e[1] = rtb_n_b_dt2[2] * rtb_n[0] - rtb_n_b_dt2[0] *
        wp_approach_out_tmp;
      rtb_Sum2_e[2] = rtb_n_b_dt2[0] * rtb_n[1] - rtb_n_b_dt2[1] * rtb_n[0];
      q1_q3 = norm(rtb_Sum2_e);
      if (std::abs(q1_q3) < 2.22044605E-16F) {
        q1_q3 = 2.22044605E-16F;
      }

      rtb_n_dt[0] = rtb_n_b_dt2[0];
      rtb_Sum2_e[0] /= q1_q3;
      rtb_n_dt[1] = rtb_n_b_dt2[1];
      rtb_Sum2_e[1] /= q1_q3;
      rtb_n_dt[2] = rtb_n_b_dt2[2];
      rtb_Sum2_e[2] /= q1_q3;
      q1_q3 = q0_q2 * q1_q2;
      axisAngle(rtb_Sum2_e, rtb_n_dt, q1_q3, rtb_n_b_h);
      rtb_p_ahead_l[0] = rtb_n_b_h[0] * t;
      rtb_n_dt[0] = rtb_n_b_dt2[0];
      rtb_p_ahead_l[1] = rtb_n_b_h[1] * t;
      rtb_n_dt[1] = rtb_n_b_dt2[1];
      rtb_p_ahead_l[2] = rtb_n_b_h[2] * t;
      rtb_n_dt[2] = rtb_n_b_dt2[2];
      axisAngle(rtb_n, rtb_n_dt, q1_q3, rtb_n_b_h);
      rtb_y_o[0] = rtb_Sum2_hq[0] - (rtb_Sum2_hq[0] + rtb_n_b_h[0]);
      rtb_y_o[1] = rtb_Sum2_hq[1] - (rtb_Sum2_hq[1] + rtb_n_b_h[1]);
      rtb_y_o[2] = rtb_Sum2_hq[2] - (rtb_Sum2_hq[2] + rtb_n_b_h[2]);
      q0_q2 = norm(rtb_y_o);
      if (std::abs(q0_q2) < 2.22044605E-16F) {
        q0_q2 = 2.22044605E-16F;
      }

      rtb_y_o[0] /= q0_q2;
      rtb_y_o[1] /= q0_q2;
      rtb_y_o[2] /= q0_q2;
      if (std::abs(absxk) < 2.22044605E-16F) {
        absxk = 2.22044605E-16F;
      }

      t = t * t / absxk;
      rtb_y_o[0] *= t;
      rtb_y_o[1] *= t;
      rtb_y_o[2] *= t;
    } else {
      if (!rtb_is_finished) {
        if (wp_idx_2 > 1) {
          rtb_DataTypeConversion2 = (wp_idx_2 - 2) * 3;
          rtb_Sum2_e[0] = rtb_wp[rtb_DataTypeConversion2];
          wp_idx_app_2 = (wp_idx_2 - 1) * 3;
          rtb_p_ahead_l[0] = rtb_wp[wp_idx_app_2];
          rtb_Sum2_e[1] = rtb_wp[rtb_DataTypeConversion2 + 1];
          rtb_p_ahead_l[1] = rtb_wp[wp_idx_app_2 + 1];
          rtb_Sum2_e[2] = rtb_wp[rtb_DataTypeConversion2 + 2];
          rtb_p_ahead_l[2] = rtb_wp[wp_idx_app_2 + 2];
        } else {
          rtb_DataTypeConversion2 = (rtb_DataTypeConversion2 - 1) * 3;
          rtb_Sum2_e[0] = rtb_wp[rtb_DataTypeConversion2];
          rtb_p_ahead_l[0] = rtb_wp[0];
          rtb_Sum2_e[1] = rtb_wp[rtb_DataTypeConversion2 + 1];
          rtb_p_ahead_l[1] = rtb_wp[1];
          rtb_Sum2_e[2] = rtb_wp[rtb_DataTypeConversion2 + 2];
          rtb_p_ahead_l[2] = rtb_wp[2];
        }
      } else {
        if (wp_idx_app_2 < -2147483647) {
          wp_idx_2 = MIN_int32_T;
        } else {
          wp_idx_2 = wp_idx_app_2 - 1;
        }

        rtb_DataTypeConversion2 = (wp_idx_2 - 1) * 3;
        rtb_Sum2_e[0] = wp_approach_out[rtb_DataTypeConversion2];
        wp_idx_app_2 = (wp_idx_app_2 - 1) * 3;
        rtb_p_ahead_l[0] = wp_approach_out[wp_idx_app_2];
        rtb_Sum2_e[1] = wp_approach_out[rtb_DataTypeConversion2 + 1];
        rtb_p_ahead_l[1] = wp_approach_out[wp_idx_app_2 + 1];
        rtb_Sum2_e[2] = wp_approach_out[rtb_DataTypeConversion2 + 2];
        rtb_p_ahead_l[2] = wp_approach_out[wp_idx_app_2 + 2];
      }

      rtb_p_ahead_l[0] -= rtb_Sum2_e[0];
      rtb_p_ahead_l[1] -= rtb_Sum2_e[1];
      rtb_p_ahead_l[2] -= rtb_Sum2_e[2];
      q0_q2 = norm(rtb_p_ahead_l);
      if (std::abs(q0_q2) < 2.22044605E-16F) {
        q0_q2 = 2.22044605E-16F;
      }

      rtb_p_ahead_l[0] = rtb_p_ahead_l[0] / q0_q2 * t;
      rtb_p_ahead_l[1] = rtb_p_ahead_l[1] / q0_q2 * t;
      rtb_p_ahead_l[2] = rtb_p_ahead_l[2] / q0_q2 * t;
    }

    // MATLAB Function: '<S49>/Look Ahead1' incorporates:
    //   Constant: '<S49>/Constant3'

    q0_q2 = q2_q3;
    q1_q3 = q2_q3;

    // Outputs for Enabled SubSystem: '<S151>/Flight Path Smoothing' incorporates:
    //   EnablePort: '<S158>/Enable'

    // RelationalOperator: '<S157>/Compare' incorporates:
    //   Constant: '<S151>/Constant'
    //   Constant: '<S157>/Constant'
    //   Inport: '<S159>/In1'
    //   Inport: '<S159>/In2'
    //   Inport: '<S159>/In3'
    //   MATLAB Function: '<S49>/WpNav Matching'

    if (rtP.lindi.wpnav.T >= 0.05F) {
      if (!rtDW.FlightPathSmoothing_MODE) {
        // InitializeConditions for DiscreteIntegrator: '<S161>/Discrete-Time Integrator y' 
        rtDW.DiscreteTimeIntegratory_IC_L_ai = 1U;

        // InitializeConditions for DiscreteIntegrator: '<S162>/Discrete-Time Integrator y' 
        rtDW.DiscreteTimeIntegratory_IC_LO_d = 1U;

        // InitializeConditions for DiscreteIntegrator: '<S161>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_D_jp[0] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S162>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_D_lk[0] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S161>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_D_jp[1] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S162>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_D_lk[1] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S161>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_D_jp[2] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S162>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_D_lk[2] = 0.0F;
        rtDW.FlightPathSmoothing_MODE = true;
      }

      // DiscreteIntegrator: '<S161>/Discrete-Time Integrator y' incorporates:
      //   MATLAB Function: '<S49>/WpNav Matching'

      if (rtDW.DiscreteTimeIntegratory_IC_L_ai != 0) {
        rtDW.DiscreteTimeIntegratory_DSTAT_l[0] = rtb_p_ahead_l[0];
        rtDW.DiscreteTimeIntegratory_DSTAT_l[1] = rtb_p_ahead_l[1];
        rtDW.DiscreteTimeIntegratory_DSTAT_l[2] = rtb_p_ahead_l[2];
      }

      // Product: '<S158>/Product1' incorporates:
      //   Constant: '<S158>/Constant'
      //   Constant: '<S158>/Constant1'

      t = 2.0F / rtP.lindi.wpnav.T * 0.7071F;

      // Gain: '<S161>/Gain' incorporates:
      //   Constant: '<S158>/Constant1'
      //   Gain: '<S162>/Gain'
      //   Product: '<S161>/Divide'

      q0_q2 = 0.7071F / t * 2.0F;

      // SignalConversion: '<S158>/OutportBufferFors_g_ref_dt_smooth' incorporates:
      //   DiscreteIntegrator: '<S161>/Discrete-Time Integrator y'

      rtDW.Merge1_p[0] = rtDW.DiscreteTimeIntegratory_DSTAT_l[0];

      // Sum: '<S161>/Sum2' incorporates:
      //   DiscreteIntegrator: '<S161>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S161>/Discrete-Time Integrator y_dt'
      //   Gain: '<S161>/Gain'
      //   MATLAB Function: '<S49>/WpNav Matching'
      //   Product: '<S161>/Product2'
      //   Sum: '<S161>/Sum3'

      rtb_Sum2_e[0] = rtb_p_ahead_l[0] - (rtDW.DiscreteTimeIntegratory_dt_D_jp[0]
        * q0_q2 + rtDW.DiscreteTimeIntegratory_DSTAT_l[0]);

      // SignalConversion: '<S158>/OutportBufferFors_g_ref_dt2_smooth' incorporates:
      //   DiscreteIntegrator: '<S161>/Discrete-Time Integrator y_dt'

      rtDW.Merge2[0] = rtDW.DiscreteTimeIntegratory_dt_D_jp[0];

      // SignalConversion: '<S158>/OutportBufferFors_g_ref_dt_smooth' incorporates:
      //   DiscreteIntegrator: '<S161>/Discrete-Time Integrator y'

      rtDW.Merge1_p[1] = rtDW.DiscreteTimeIntegratory_DSTAT_l[1];

      // Sum: '<S161>/Sum2' incorporates:
      //   DiscreteIntegrator: '<S161>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S161>/Discrete-Time Integrator y_dt'
      //   Gain: '<S161>/Gain'
      //   MATLAB Function: '<S49>/WpNav Matching'
      //   Product: '<S161>/Product2'
      //   Sum: '<S161>/Sum3'

      rtb_Sum2_e[1] = rtb_p_ahead_l[1] - (rtDW.DiscreteTimeIntegratory_dt_D_jp[1]
        * q0_q2 + rtDW.DiscreteTimeIntegratory_DSTAT_l[1]);

      // SignalConversion: '<S158>/OutportBufferFors_g_ref_dt2_smooth' incorporates:
      //   DiscreteIntegrator: '<S161>/Discrete-Time Integrator y_dt'

      rtDW.Merge2[1] = rtDW.DiscreteTimeIntegratory_dt_D_jp[1];

      // SignalConversion: '<S158>/OutportBufferFors_g_ref_dt_smooth' incorporates:
      //   DiscreteIntegrator: '<S161>/Discrete-Time Integrator y'

      rtDW.Merge1_p[2] = rtDW.DiscreteTimeIntegratory_DSTAT_l[2];

      // Sum: '<S161>/Sum2' incorporates:
      //   DiscreteIntegrator: '<S161>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S161>/Discrete-Time Integrator y_dt'
      //   Gain: '<S161>/Gain'
      //   MATLAB Function: '<S49>/WpNav Matching'
      //   Product: '<S161>/Product2'
      //   Sum: '<S161>/Sum3'

      rtb_Sum2_e[2] = rtb_p_ahead_l[2] - (rtDW.DiscreteTimeIntegratory_dt_D_jp[2]
        * q0_q2 + rtDW.DiscreteTimeIntegratory_DSTAT_l[2]);

      // SignalConversion: '<S158>/OutportBufferFors_g_ref_dt2_smooth' incorporates:
      //   DiscreteIntegrator: '<S161>/Discrete-Time Integrator y_dt'

      rtDW.Merge2[2] = rtDW.DiscreteTimeIntegratory_dt_D_jp[2];

      // Product: '<S161>/omega^2' incorporates:
      //   Product: '<S162>/omega^2'

      q1_q2 = t * t;

      // DiscreteIntegrator: '<S162>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S89>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S91>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S92>/Discrete-Time Integrator y'
      //   MATLAB Function: '<S49>/Look Ahead1'

      if (rtDW.DiscreteTimeIntegratory_IC_LO_d != 0) {
        rtDW.DiscreteTimeIntegratory_DSTA_l5[0] = 0.5F *
          rtDW.DiscreteTimeIntegratory_DSTA_al[0] * q2_q3 * q2_q3 +
          (rtDW.DiscreteTimeIntegratory_DSTAT_a[0] * q2_q3 +
           rtDW.DiscreteTimeIntegratory_DSTATE[0]);
        rtDW.DiscreteTimeIntegratory_DSTA_l5[1] = 0.5F *
          rtDW.DiscreteTimeIntegratory_DSTA_al[1] * q2_q3 * q2_q3 +
          (rtDW.DiscreteTimeIntegratory_DSTAT_a[1] * q2_q3 +
           rtDW.DiscreteTimeIntegratory_DSTATE[1]);
        rtDW.DiscreteTimeIntegratory_DSTA_l5[2] = 0.5F *
          rtDW.DiscreteTimeIntegratory_DSTA_al[2] * q2_q3 * q2_q3 +
          (rtDW.DiscreteTimeIntegratory_DSTAT_a[2] * q2_q3 +
           rtDW.DiscreteTimeIntegratory_DSTATE[2]);
      }

      // Update for DiscreteIntegrator: '<S161>/Discrete-Time Integrator y'
      rtDW.DiscreteTimeIntegratory_IC_L_ai = 0U;

      // Update for DiscreteIntegrator: '<S162>/Discrete-Time Integrator y'
      rtDW.DiscreteTimeIntegratory_IC_LO_d = 0U;

      // SignalConversion: '<S158>/OutportBufferFors_g_ref_smooth' incorporates:
      //   DiscreteIntegrator: '<S162>/Discrete-Time Integrator y'

      rtDW.Merge_b[0] = rtDW.DiscreteTimeIntegratory_DSTA_l5[0];

      // Sum: '<S162>/Sum2' incorporates:
      //   DiscreteIntegrator: '<S162>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S162>/Discrete-Time Integrator y_dt'
      //   MATLAB Function: '<S49>/WpNav Matching'
      //   Product: '<S162>/Product2'
      //   Sum: '<S162>/Sum3'

      q2_q3 = rtb_n_g_des_n[0] - (rtDW.DiscreteTimeIntegratory_dt_D_lk[0] *
        q0_q2 + rtDW.DiscreteTimeIntegratory_DSTA_l5[0]);

      // Update for DiscreteIntegrator: '<S161>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S161>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_DSTAT_l[0] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_D_jp[0];

      // Update for DiscreteIntegrator: '<S161>/Discrete-Time Integrator y_dt' incorporates:
      //   Product: '<S161>/Product1'
      //   Product: '<S161>/omega^2'

      rtDW.DiscreteTimeIntegratory_dt_D_jp[0] += rtb_Sum2_e[0] * q1_q2 * 0.0025F;

      // Update for DiscreteIntegrator: '<S162>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S162>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_DSTA_l5[0] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_D_lk[0];

      // Update for DiscreteIntegrator: '<S162>/Discrete-Time Integrator y_dt' incorporates:
      //   Product: '<S162>/Product1'

      rtDW.DiscreteTimeIntegratory_dt_D_lk[0] += q2_q3 * q1_q2 * 0.0025F;

      // SignalConversion: '<S158>/OutportBufferFors_g_ref_smooth' incorporates:
      //   DiscreteIntegrator: '<S162>/Discrete-Time Integrator y'

      rtDW.Merge_b[1] = rtDW.DiscreteTimeIntegratory_DSTA_l5[1];

      // Sum: '<S162>/Sum2' incorporates:
      //   DiscreteIntegrator: '<S162>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S162>/Discrete-Time Integrator y_dt'
      //   MATLAB Function: '<S49>/WpNav Matching'
      //   Product: '<S162>/Product2'
      //   Sum: '<S162>/Sum3'

      q2_q3 = rtb_n_g_des_n[1] - (rtDW.DiscreteTimeIntegratory_dt_D_lk[1] *
        q0_q2 + rtDW.DiscreteTimeIntegratory_DSTA_l5[1]);

      // Update for DiscreteIntegrator: '<S161>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S161>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_DSTAT_l[1] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_D_jp[1];

      // Update for DiscreteIntegrator: '<S161>/Discrete-Time Integrator y_dt' incorporates:
      //   Product: '<S161>/Product1'
      //   Product: '<S161>/omega^2'

      rtDW.DiscreteTimeIntegratory_dt_D_jp[1] += rtb_Sum2_e[1] * q1_q2 * 0.0025F;

      // Update for DiscreteIntegrator: '<S162>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S162>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_DSTA_l5[1] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_D_lk[1];

      // Update for DiscreteIntegrator: '<S162>/Discrete-Time Integrator y_dt' incorporates:
      //   Product: '<S162>/Product1'

      rtDW.DiscreteTimeIntegratory_dt_D_lk[1] += q2_q3 * q1_q2 * 0.0025F;

      // SignalConversion: '<S158>/OutportBufferFors_g_ref_smooth' incorporates:
      //   DiscreteIntegrator: '<S162>/Discrete-Time Integrator y'

      rtDW.Merge_b[2] = rtDW.DiscreteTimeIntegratory_DSTA_l5[2];

      // Sum: '<S162>/Sum2' incorporates:
      //   DiscreteIntegrator: '<S162>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S162>/Discrete-Time Integrator y_dt'
      //   MATLAB Function: '<S49>/WpNav Matching'
      //   Product: '<S162>/Product2'
      //   Sum: '<S162>/Sum3'

      q2_q3 = rtb_n_g_des_n[2] - (rtDW.DiscreteTimeIntegratory_dt_D_lk[2] *
        q0_q2 + rtDW.DiscreteTimeIntegratory_DSTA_l5[2]);

      // Update for DiscreteIntegrator: '<S161>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S161>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_DSTAT_l[2] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_D_jp[2];

      // Update for DiscreteIntegrator: '<S161>/Discrete-Time Integrator y_dt' incorporates:
      //   Product: '<S161>/Product1'
      //   Product: '<S161>/omega^2'

      rtDW.DiscreteTimeIntegratory_dt_D_jp[2] += rtb_Sum2_e[2] * q1_q2 * 0.0025F;

      // Update for DiscreteIntegrator: '<S162>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S162>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_DSTA_l5[2] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_D_lk[2];

      // Update for DiscreteIntegrator: '<S162>/Discrete-Time Integrator y_dt' incorporates:
      //   Product: '<S162>/Product1'

      rtDW.DiscreteTimeIntegratory_dt_D_lk[2] += q2_q3 * q1_q2 * 0.0025F;
    } else {
      if (rtDW.FlightPathSmoothing_MODE) {
        rtDW.FlightPathSmoothing_MODE = false;
      }

      // Outputs for Enabled SubSystem: '<S151>/Pass-through' incorporates:
      //   EnablePort: '<S159>/Enable'

      rtDW.Merge_b[0] = rtb_n_g_des_n[0];
      rtDW.Merge1_p[0] = rtb_p_ahead_l[0];
      rtDW.Merge2[0] = rtb_y_o[0];
      rtDW.Merge_b[1] = rtb_n_g_des_n[1];
      rtDW.Merge1_p[1] = rtb_p_ahead_l[1];
      rtDW.Merge2[1] = rtb_y_o[1];
      rtDW.Merge_b[2] = rtb_n_g_des_n[2];
      rtDW.Merge1_p[2] = rtb_p_ahead_l[2];
      rtDW.Merge2[2] = rtb_y_o[2];

      // End of Outputs for SubSystem: '<S151>/Pass-through'
    }

    // End of RelationalOperator: '<S157>/Compare'
    // End of Outputs for SubSystem: '<S151>/Flight Path Smoothing'

    // Update for UnitDelay: '<S49>/Unit Delay' incorporates:
    //   MATLAB Function: '<S49>/WpNav Matching'

    rtDW.UnitDelay_DSTATE_o = flight_mode;

    // Update for UnitDelay: '<S49>/Unit Delay1' incorporates:
    //   MATLAB Function: '<S49>/WpNav Matching'

    rtDW.UnitDelay1_DSTATE_f = b_stage_;

    // SignalConversion: '<S49>/BusConversion_InsertedFor_wpnav_at_inport_0'
    rtDW.s_g_ref_dt2[0] = rtDW.Merge2[0];

    // SignalConversion: '<S49>/BusConversion_InsertedFor_wpnav_at_inport_0'
    rtDW.s_g_ref[0] = rtDW.Merge_b[0];

    // SignalConversion: '<S49>/BusConversion_InsertedFor_wpnav_at_inport_0'
    rtDW.s_g_ref_dt[0] = rtDW.Merge1_p[0];

    // Update for UnitDelay: '<S49>/Unit Delay6' incorporates:
    //   MATLAB Function: '<S49>/WpNav Matching'

    rtDW.UnitDelay6_DSTATE[0] = a_Kb_meas[0];

    // SignalConversion: '<S49>/BusConversion_InsertedFor_wpnav_at_inport_0'
    rtDW.s_g_ref_dt2[1] = rtDW.Merge2[1];

    // SignalConversion: '<S49>/BusConversion_InsertedFor_wpnav_at_inport_0'
    rtDW.s_g_ref[1] = rtDW.Merge_b[1];

    // SignalConversion: '<S49>/BusConversion_InsertedFor_wpnav_at_inport_0'
    rtDW.s_g_ref_dt[1] = rtDW.Merge1_p[1];

    // Update for UnitDelay: '<S49>/Unit Delay6' incorporates:
    //   MATLAB Function: '<S49>/WpNav Matching'

    rtDW.UnitDelay6_DSTATE[1] = a_Kb_meas[1];

    // SignalConversion: '<S49>/BusConversion_InsertedFor_wpnav_at_inport_0'
    rtDW.s_g_ref_dt2[2] = rtDW.Merge2[2];

    // SignalConversion: '<S49>/BusConversion_InsertedFor_wpnav_at_inport_0'
    rtDW.s_g_ref[2] = rtDW.Merge_b[2];

    // SignalConversion: '<S49>/BusConversion_InsertedFor_wpnav_at_inport_0'
    rtDW.s_g_ref_dt[2] = rtDW.Merge1_p[2];

    // Update for UnitDelay: '<S49>/Unit Delay6' incorporates:
    //   MATLAB Function: '<S49>/WpNav Matching'

    rtDW.UnitDelay6_DSTATE[2] = a_Kb_meas[2];

    // Update for UnitDelay: '<S49>/Unit Delay5' incorporates:
    //   MATLAB Function: '<S49>/WpNav Matching'

    rtDW.UnitDelay5_DSTATE_n = rtb_is_finished;

    // Update for UnitDelay: '<S49>/Unit Delay2' incorporates:
    //   MATLAB Function: '<S49>/WpNav Matching'

    rtDW.UnitDelay2_DSTATE = k;

    // Update for UnitDelay: '<S49>/Unit Delay3' incorporates:
    //   MATLAB Function: '<S49>/WpNav Matching'

    rtDW.UnitDelay3_DSTATE_m = c_stage_app;

    // Update for UnitDelay: '<S49>/Unit Delay4' incorporates:
    //   MATLAB Function: '<S49>/WpNav Matching'

    for (wp_idx_app_2 = 0; wp_idx_app_2 < 15; wp_idx_app_2++) {
      rtDW.UnitDelay4_DSTATE[wp_idx_app_2] = wp_approach_out[wp_idx_app_2];
    }

    // End of Update for UnitDelay: '<S49>/Unit Delay4'
  } else {
    if (rtDW.WaypointNavigation_MODE) {
      // Disable for Enabled SubSystem: '<S151>/Flight Path Smoothing'
      if (rtDW.FlightPathSmoothing_MODE) {
        rtDW.FlightPathSmoothing_MODE = false;
      }

      // End of Disable for SubSystem: '<S151>/Flight Path Smoothing'
      rtDW.WaypointNavigation_MODE = false;
    }
  }

  // End of Outputs for SubSystem: '<S4>/Waypoint Navigation'

  // Outputs for Enabled SubSystem: '<S4>/NDI position controller for copters with reference input' incorporates:
  //   EnablePort: '<S44>/Enable'

  if (rtb_Compare) {
    if (!rtDW.NDIpositioncontrollerforcopters) {
      // InitializeConditions for DiscreteIntegrator: '<S136>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_IC_LO_h = 1U;
      rtDW.DiscreteTimeIntegratory_PrevR_a = 2;

      // InitializeConditions for DiscreteIntegrator: '<S136>/Discrete-Time Integrator y_dt' 
      for (wp_idx_app_2 = 0; wp_idx_app_2 < 9; wp_idx_app_2++) {
        rtDW.DiscreteTimeIntegratory_dt_DS_f[wp_idx_app_2] = 0.0F;
      }

      rtDW.DiscreteTimeIntegratory_dt_P_mk = 2;

      // End of InitializeConditions for DiscreteIntegrator: '<S136>/Discrete-Time Integrator y_dt' 
      rtDW.NDIpositioncontrollerforcopters = true;
    }

    // DiscreteIntegrator: '<S136>/Discrete-Time Integrator y'
    if (rtDW.DiscreteTimeIntegratory_IC_LO_h != 0) {
      rtDW.DiscreteTimeIntegratory_DSTA_ld[0] = rtDW.s_g_ref[0];
      rtDW.DiscreteTimeIntegratory_DSTA_ld[3] = rtDW.s_g_ref_dt[0];
      rtDW.DiscreteTimeIntegratory_DSTA_ld[6] = rtDW.s_g_ref_dt2[0];
      rtDW.DiscreteTimeIntegratory_DSTA_ld[1] = rtDW.s_g_ref[1];
      rtDW.DiscreteTimeIntegratory_DSTA_ld[4] = rtDW.s_g_ref_dt[1];
      rtDW.DiscreteTimeIntegratory_DSTA_ld[7] = rtDW.s_g_ref_dt2[1];
      rtDW.DiscreteTimeIntegratory_DSTA_ld[2] = rtDW.s_g_ref[2];
      rtDW.DiscreteTimeIntegratory_DSTA_ld[5] = rtDW.s_g_ref_dt[2];
      rtDW.DiscreteTimeIntegratory_DSTA_ld[8] = rtDW.s_g_ref_dt2[2];
    }

    if (rtb_is_slowing && (rtDW.DiscreteTimeIntegratory_PrevR_a <= 0)) {
      rtDW.DiscreteTimeIntegratory_DSTA_ld[0] = rtDW.s_g_ref[0];
      rtDW.DiscreteTimeIntegratory_DSTA_ld[3] = rtDW.s_g_ref_dt[0];
      rtDW.DiscreteTimeIntegratory_DSTA_ld[6] = rtDW.s_g_ref_dt2[0];
      rtDW.DiscreteTimeIntegratory_DSTA_ld[1] = rtDW.s_g_ref[1];
      rtDW.DiscreteTimeIntegratory_DSTA_ld[4] = rtDW.s_g_ref_dt[1];
      rtDW.DiscreteTimeIntegratory_DSTA_ld[7] = rtDW.s_g_ref_dt2[1];
      rtDW.DiscreteTimeIntegratory_DSTA_ld[2] = rtDW.s_g_ref[2];
      rtDW.DiscreteTimeIntegratory_DSTA_ld[5] = rtDW.s_g_ref_dt[2];
      rtDW.DiscreteTimeIntegratory_DSTA_ld[8] = rtDW.s_g_ref_dt2[2];
    }

    // Product: '<S135>/Product1' incorporates:
    //   Constant: '<S135>/Constant'
    //   Constant: '<S135>/Constant1'

    t = 2.0F / ((2.0F / rtP.lindi.atc.rm.leanfreq + rtP.lindi.mtc) + 2.0F /
                rtP.lindi.sflt.omega) * 0.7071F;

    // Gain: '<S136>/Gain' incorporates:
    //   Constant: '<S135>/Constant1'
    //   Product: '<S136>/Divide'

    q0_q2 = 0.7071F / t * 2.0F;

    // DiscreteIntegrator: '<S136>/Discrete-Time Integrator y_dt'
    if (rtb_is_slowing && (rtDW.DiscreteTimeIntegratory_dt_P_mk <= 0)) {
      for (wp_idx_app_2 = 0; wp_idx_app_2 < 9; wp_idx_app_2++) {
        rtDW.DiscreteTimeIntegratory_dt_DS_f[wp_idx_app_2] = 0.0F;
      }
    }

    // Sum: '<S136>/Sum2' incorporates:
    //   DiscreteIntegrator: '<S136>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S136>/Discrete-Time Integrator y_dt'
    //   Product: '<S136>/Product2'
    //   Sum: '<S136>/Sum3'

    rtb_Sum2_jx[0] = rtDW.s_g_ref[0] - (rtDW.DiscreteTimeIntegratory_dt_DS_f[0] *
      q0_q2 + rtDW.DiscreteTimeIntegratory_DSTA_ld[0]);
    rtb_Sum2_jx[3] = rtDW.s_g_ref_dt[0] - (rtDW.DiscreteTimeIntegratory_dt_DS_f
      [3] * q0_q2 + rtDW.DiscreteTimeIntegratory_DSTA_ld[3]);
    rtb_Sum2_jx[6] = rtDW.s_g_ref_dt2[0] -
      (rtDW.DiscreteTimeIntegratory_dt_DS_f[6] * q0_q2 +
       rtDW.DiscreteTimeIntegratory_DSTA_ld[6]);

    // Sum: '<S135>/Add' incorporates:
    //   DiscreteIntegrator: '<S136>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S89>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S91>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S92>/Discrete-Time Integrator y'

    rtDW.Add[0] = rtDW.DiscreteTimeIntegratory_DSTA_ld[0] -
      rtDW.DiscreteTimeIntegratory_DSTATE[0];
    rtDW.Add[3] = rtDW.DiscreteTimeIntegratory_DSTA_ld[3] -
      rtDW.DiscreteTimeIntegratory_DSTAT_a[0];
    rtDW.Add[6] = rtDW.DiscreteTimeIntegratory_DSTA_ld[6] -
      rtDW.DiscreteTimeIntegratory_DSTA_al[0];

    // Sum: '<S136>/Sum2' incorporates:
    //   DiscreteIntegrator: '<S136>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S136>/Discrete-Time Integrator y_dt'
    //   Product: '<S136>/Product2'
    //   Sum: '<S136>/Sum3'

    rtb_Sum2_jx[1] = rtDW.s_g_ref[1] - (rtDW.DiscreteTimeIntegratory_dt_DS_f[1] *
      q0_q2 + rtDW.DiscreteTimeIntegratory_DSTA_ld[1]);
    rtb_Sum2_jx[4] = rtDW.s_g_ref_dt[1] - (rtDW.DiscreteTimeIntegratory_dt_DS_f
      [4] * q0_q2 + rtDW.DiscreteTimeIntegratory_DSTA_ld[4]);
    rtb_Sum2_jx[7] = rtDW.s_g_ref_dt2[1] -
      (rtDW.DiscreteTimeIntegratory_dt_DS_f[7] * q0_q2 +
       rtDW.DiscreteTimeIntegratory_DSTA_ld[7]);

    // Sum: '<S135>/Add' incorporates:
    //   DiscreteIntegrator: '<S136>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S89>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S91>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S92>/Discrete-Time Integrator y'

    rtDW.Add[1] = rtDW.DiscreteTimeIntegratory_DSTA_ld[1] -
      rtDW.DiscreteTimeIntegratory_DSTATE[1];
    rtDW.Add[4] = rtDW.DiscreteTimeIntegratory_DSTA_ld[4] -
      rtDW.DiscreteTimeIntegratory_DSTAT_a[1];
    rtDW.Add[7] = rtDW.DiscreteTimeIntegratory_DSTA_ld[7] -
      rtDW.DiscreteTimeIntegratory_DSTA_al[1];

    // Sum: '<S136>/Sum2' incorporates:
    //   DiscreteIntegrator: '<S136>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S136>/Discrete-Time Integrator y_dt'
    //   Product: '<S136>/Product2'
    //   Sum: '<S136>/Sum3'

    rtb_Sum2_jx[2] = rtDW.s_g_ref[2] - (rtDW.DiscreteTimeIntegratory_dt_DS_f[2] *
      q0_q2 + rtDW.DiscreteTimeIntegratory_DSTA_ld[2]);
    rtb_Sum2_jx[5] = rtDW.s_g_ref_dt[2] - (rtDW.DiscreteTimeIntegratory_dt_DS_f
      [5] * q0_q2 + rtDW.DiscreteTimeIntegratory_DSTA_ld[5]);
    rtb_Sum2_jx[8] = rtDW.s_g_ref_dt2[2] -
      (rtDW.DiscreteTimeIntegratory_dt_DS_f[8] * q0_q2 +
       rtDW.DiscreteTimeIntegratory_DSTA_ld[8]);

    // Sum: '<S135>/Add' incorporates:
    //   DiscreteIntegrator: '<S136>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S89>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S91>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S92>/Discrete-Time Integrator y'

    rtDW.Add[2] = rtDW.DiscreteTimeIntegratory_DSTA_ld[2] -
      rtDW.DiscreteTimeIntegratory_DSTATE[2];
    rtDW.Add[5] = rtDW.DiscreteTimeIntegratory_DSTA_ld[5] -
      rtDW.DiscreteTimeIntegratory_DSTAT_a[2];
    rtDW.Add[8] = rtDW.DiscreteTimeIntegratory_DSTA_ld[8] -
      rtDW.DiscreteTimeIntegratory_DSTA_al[2];

    // Product: '<S136>/omega^2'
    q0_q2 = t * t;

    // MATLAB Function: '<S135>/eposmax'
    t = rtP.lindi.psc.k.pos;
    if (std::abs(rtP.lindi.psc.k.pos) < 2.22044605E-16F) {
      t = 2.22044605E-16F;
    }

    absxk = std::abs(rtP.lindi.psc.k.vel * rtP.lindi.psc.rm.velxymax * 0.7F / t);

    // End of MATLAB Function: '<S135>/eposmax'

    // Switch: '<S137>/Switch2' incorporates:
    //   Gain: '<S135>/Gain1'
    //   RelationalOperator: '<S137>/LowerRelop1'
    //   RelationalOperator: '<S137>/UpperRelop'
    //   Switch: '<S137>/Switch'

    if (rtDW.Add[0] > absxk) {
      q1_q3 = absxk;
    } else if (rtDW.Add[0] < -absxk) {
      // Switch: '<S137>/Switch' incorporates:
      //   Gain: '<S135>/Gain1'

      q1_q3 = -absxk;
    } else {
      q1_q3 = rtDW.Add[0];
    }

    // Sum: '<S135>/Add1' incorporates:
    //   Gain: '<S135>/Gain'
    //   Gain: '<S135>/Gain3'
    //   Gain: '<S135>/Gain4'

    rtb_n_b_dt2[0] = (rtP.lindi.psc.k.pos * q1_q3 + rtP.lindi.psc.k.vel *
                      rtDW.Add[3]) + rtP.lindi.psc.k.acc * rtDW.Add[6];

    // Switch: '<S137>/Switch2' incorporates:
    //   Gain: '<S135>/Gain1'
    //   RelationalOperator: '<S137>/LowerRelop1'
    //   RelationalOperator: '<S137>/UpperRelop'
    //   Switch: '<S137>/Switch'

    if (rtDW.Add[1] > absxk) {
      q1_q3 = absxk;
    } else if (rtDW.Add[1] < -absxk) {
      // Switch: '<S137>/Switch' incorporates:
      //   Gain: '<S135>/Gain1'

      q1_q3 = -absxk;
    } else {
      q1_q3 = rtDW.Add[1];
    }

    // Sum: '<S135>/Add1' incorporates:
    //   Gain: '<S135>/Gain'
    //   Gain: '<S135>/Gain3'
    //   Gain: '<S135>/Gain4'

    rtb_n_b_dt2[1] = (rtP.lindi.psc.k.pos * q1_q3 + rtP.lindi.psc.k.vel *
                      rtDW.Add[4]) + rtP.lindi.psc.k.acc * rtDW.Add[7];

    // Switch: '<S137>/Switch2' incorporates:
    //   Gain: '<S135>/Gain1'
    //   RelationalOperator: '<S137>/LowerRelop1'
    //   RelationalOperator: '<S137>/UpperRelop'
    //   Switch: '<S137>/Switch'

    if (rtDW.Add[2] > absxk) {
      q1_q3 = absxk;
    } else if (rtDW.Add[2] < -absxk) {
      // Switch: '<S137>/Switch' incorporates:
      //   Gain: '<S135>/Gain1'

      q1_q3 = -absxk;
    } else {
      q1_q3 = rtDW.Add[2];
    }

    if (rtDW.Add[2] <= absxk) {
      // Switch: '<S137>/Switch' incorporates:
      //   Gain: '<S135>/Gain1'
      //   RelationalOperator: '<S137>/UpperRelop'

      if (rtDW.Add[2] < -absxk) {
        absxk = -absxk;
      } else {
        absxk = rtDW.Add[2];
      }
    }

    // Gain: '<S135>/Gain3'
    q2_q3 = rtP.lindi.psc.k.vel * rtDW.Add[5];

    // Gain: '<S135>/Gain4'
    q1_q2 = rtP.lindi.psc.k.acc * rtDW.Add[8];

    // Sum: '<S135>/Add1' incorporates:
    //   Gain: '<S135>/Gain'
    //   Gain: '<S135>/Gain3'
    //   Gain: '<S135>/Gain4'

    rtb_n_b_dt2[2] = (rtP.lindi.psc.k.pos * absxk + q2_q3) + q1_q2;

    // MATLAB Function: '<S135>/acccntrlmax'
    absxk = std::abs(0.75F * rtP.lindi.psc.rm.accxymax);

    // RelationalOperator: '<S138>/LowerRelop1'
    wp_approach_out_tmp = rtb_n_b_dt2[0];

    // Switch: '<S138>/Switch' incorporates:
    //   Gain: '<S135>/Gain2'
    //   RelationalOperator: '<S138>/LowerRelop1'
    //   RelationalOperator: '<S138>/UpperRelop'

    if (rtb_n_b_dt2[0] < -absxk) {
      wp_approach_out_tmp = -absxk;
    }

    // Switch: '<S138>/Switch2' incorporates:
    //   RelationalOperator: '<S138>/LowerRelop1'

    if (rtb_n_b_dt2[0] > absxk) {
      wp_approach_out_tmp = absxk;
    }

    // Sum: '<S44>/Add1'
    rtDW.nu[0] = rtDW.s_g_ref_dt2[0] + wp_approach_out_tmp;

    // SignalConversion: '<S44>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' 
    rtDW.s_g_ref_f[0] = rtDW.s_g_ref[0];

    // SignalConversion: '<S44>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' incorporates:
    //   DiscreteIntegrator: '<S92>/Discrete-Time Integrator y'

    rtDW.s_g[0] = rtDW.DiscreteTimeIntegratory_DSTATE[0];

    // SignalConversion: '<S44>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' incorporates:
    //   DiscreteIntegrator: '<S91>/Discrete-Time Integrator y'

    rtDW.s_g_dt[0] = rtDW.DiscreteTimeIntegratory_DSTAT_a[0];

    // SignalConversion: '<S44>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' incorporates:
    //   DiscreteIntegrator: '<S89>/Discrete-Time Integrator y'

    rtDW.s_g_dt2[0] = rtDW.DiscreteTimeIntegratory_DSTA_al[0];

    // RelationalOperator: '<S138>/LowerRelop1'
    wp_approach_out_tmp = rtb_n_b_dt2[1];

    // Switch: '<S138>/Switch' incorporates:
    //   Gain: '<S135>/Gain2'
    //   RelationalOperator: '<S138>/LowerRelop1'
    //   RelationalOperator: '<S138>/UpperRelop'

    if (rtb_n_b_dt2[1] < -absxk) {
      wp_approach_out_tmp = -absxk;
    }

    // Switch: '<S138>/Switch2' incorporates:
    //   RelationalOperator: '<S138>/LowerRelop1'

    if (rtb_n_b_dt2[1] > absxk) {
      wp_approach_out_tmp = absxk;
    }

    // Sum: '<S44>/Add1'
    rtDW.nu[1] = rtDW.s_g_ref_dt2[1] + wp_approach_out_tmp;

    // SignalConversion: '<S44>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' 
    rtDW.s_g_ref_f[1] = rtDW.s_g_ref[1];

    // SignalConversion: '<S44>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' incorporates:
    //   DiscreteIntegrator: '<S92>/Discrete-Time Integrator y'

    rtDW.s_g[1] = rtDW.DiscreteTimeIntegratory_DSTATE[1];

    // SignalConversion: '<S44>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' incorporates:
    //   DiscreteIntegrator: '<S91>/Discrete-Time Integrator y'

    rtDW.s_g_dt[1] = rtDW.DiscreteTimeIntegratory_DSTAT_a[1];

    // SignalConversion: '<S44>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' incorporates:
    //   DiscreteIntegrator: '<S89>/Discrete-Time Integrator y'

    rtDW.s_g_dt2[1] = rtDW.DiscreteTimeIntegratory_DSTA_al[1];

    // Sum: '<S135>/Add1' incorporates:
    //   Gain: '<S135>/Gain'

    wp_approach_out_tmp = (rtP.lindi.psc.k.pos * q1_q3 + q2_q3) + q1_q2;

    // Switch: '<S138>/Switch' incorporates:
    //   Gain: '<S135>/Gain2'
    //   RelationalOperator: '<S138>/LowerRelop1'
    //   RelationalOperator: '<S138>/UpperRelop'

    if (rtb_n_b_dt2[2] < -absxk) {
      wp_approach_out_tmp = -absxk;
    }

    // Switch: '<S138>/Switch2' incorporates:
    //   RelationalOperator: '<S138>/LowerRelop1'

    if (rtb_n_b_dt2[2] > absxk) {
      wp_approach_out_tmp = absxk;
    }

    // Sum: '<S44>/Add1'
    rtDW.nu[2] = rtDW.s_g_ref_dt2[2] + wp_approach_out_tmp;

    // SignalConversion: '<S44>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' 
    rtDW.s_g_ref_f[2] = rtDW.s_g_ref[2];

    // SignalConversion: '<S44>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' incorporates:
    //   DiscreteIntegrator: '<S92>/Discrete-Time Integrator y'

    rtDW.s_g[2] = rtDW.DiscreteTimeIntegratory_DSTATE[2];

    // SignalConversion: '<S44>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' incorporates:
    //   DiscreteIntegrator: '<S91>/Discrete-Time Integrator y'

    rtDW.s_g_dt[2] = rtDW.DiscreteTimeIntegratory_DSTAT_a[2];

    // SignalConversion: '<S44>/BusConversion_InsertedFor_pos_cntrl_at_inport_0' incorporates:
    //   DiscreteIntegrator: '<S89>/Discrete-Time Integrator y'

    rtDW.s_g_dt2[2] = rtDW.DiscreteTimeIntegratory_DSTA_al[2];

    // Update for DiscreteIntegrator: '<S136>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S136>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_IC_LO_h = 0U;
    rtDW.DiscreteTimeIntegratory_PrevR_a = (int8_T)rtb_is_slowing;
    for (wp_idx_app_2 = 0; wp_idx_app_2 < 9; wp_idx_app_2++) {
      rtDW.DiscreteTimeIntegratory_DSTA_ld[wp_idx_app_2] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_DS_f[wp_idx_app_2];

      // Update for DiscreteIntegrator: '<S136>/Discrete-Time Integrator y_dt' incorporates:
      //   Product: '<S136>/Product1'

      rtDW.DiscreteTimeIntegratory_dt_DS_f[wp_idx_app_2] +=
        rtb_Sum2_jx[wp_idx_app_2] * q0_q2 * 0.0025F;
    }

    // End of Update for DiscreteIntegrator: '<S136>/Discrete-Time Integrator y' 

    // Update for DiscreteIntegrator: '<S136>/Discrete-Time Integrator y_dt'
    rtDW.DiscreteTimeIntegratory_dt_P_mk = (int8_T)rtb_is_slowing;
  } else {
    if (rtDW.NDIpositioncontrollerforcopters) {
      rtDW.NDIpositioncontrollerforcopters = false;
    }
  }

  // End of Outputs for SubSystem: '<S4>/NDI position controller for copters with reference input' 

  // DiscreteIntegrator: '<S93>/Discrete-Time Integrator y_dt'
  a_Kb_meas[0] = rtDW.DiscreteTimeIntegratory_dt_DSTA[0];
  a_Kb_meas[1] = rtDW.DiscreteTimeIntegratory_dt_DSTA[1];
  a_Kb_meas[2] = rtDW.DiscreteTimeIntegratory_dt_DSTA[2];

  // DiscreteIntegrator: '<S90>/Discrete-Time Integrator y'
  if (rtDW.DiscreteTimeIntegratory_IC_LO_a != 0) {
    for (wp_idx_app_2 = 0; wp_idx_app_2 < 9; wp_idx_app_2++) {
      rtDW.DiscreteTimeIntegratory_DSTAT_p[wp_idx_app_2] = rtb_M_bg[wp_idx_app_2];
    }
  }

  for (wp_idx_app_2 = 0; wp_idx_app_2 < 9; wp_idx_app_2++) {
    rtb_Sum2_jx[wp_idx_app_2] =
      rtDW.DiscreteTimeIntegratory_DSTAT_p[wp_idx_app_2];
  }

  // Outputs for Enabled SubSystem: '<S4>/Pilot Reduced Attitude Commands' incorporates:
  //   EnablePort: '<S45>/Enable'

  if (rtb_UnitDelay1_j) {
    // MATLAB Function: '<S45>/MATLAB Function' incorporates:
    //   Gain: '<S45>/Gain'

    q1_q3 = std::atan2(cos_Theta, scale);
    cos_Theta = std::sqrt(cos_Theta * cos_Theta + -scale * -scale);

    // MATLAB Function: '<S45>/MATLAB Function1' incorporates:
    //   Gain: '<S45>/Gain1'

    absxk = rtP.lindi.atc.rm.leanmax * cos_Theta;
    t = std::sin(absxk);

    // SignalConversion: '<S45>/BusConversion_InsertedFor_red_atti_des_at_inport_0' incorporates:
    //   MATLAB Function: '<S45>/MATLAB Function'
    //   MATLAB Function: '<S45>/MATLAB Function1'

    rtDW.n_g_des[0] = t * std::cos(q1_q3);
    rtDW.n_g_des[1] = t * std::sin(q1_q3);
    rtDW.n_g_des[2] = -std::cos(absxk);

    // SignalConversion: '<S45>/BusConversion_InsertedFor_red_atti_des_at_inport_0' incorporates:
    //   MATLAB Function: '<S45>/MATLAB Function'

    rtDW.lean_dir_angle_des = q1_q3;

    // SignalConversion: '<S45>/BusConversion_InsertedFor_red_atti_des_at_inport_0' 
    rtDW.cmd_lean_angle_01 = cos_Theta;
  }

  // End of Outputs for SubSystem: '<S4>/Pilot Reduced Attitude Commands'

  // Outputs for Enabled SubSystem: '<S52>/Copter Random Excitation' incorporates:
  //   EnablePort: '<S54>/Enable'

  if (rtb_is_intercept_arc) {
    if (!rtDW.CopterRandomExcitation_MODE) {
      // InitializeConditions for RandomNumber: '<S64>/White Noise'
      rtDW.RandSeed[0] = 1529675776U;
      rtDW.NextOutput[0] = rt_nrand_Upu32_Yd_f_pw(&rtDW.RandSeed[0]);

      // InitializeConditions for DiscreteIntegrator: '<S65>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_DSTAT_d[0] = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S65>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_DS_h[0] = 0.0F;

      // InitializeConditions for RandomNumber: '<S64>/White Noise'
      rtDW.RandSeed[1] = 1529741312U;
      rtDW.NextOutput[1] = rt_nrand_Upu32_Yd_f_pw(&rtDW.RandSeed[1]);

      // InitializeConditions for DiscreteIntegrator: '<S65>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_DSTAT_d[1] = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S65>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_DS_h[1] = 0.0F;

      // InitializeConditions for RandomNumber: '<S64>/White Noise'
      rtDW.RandSeed[2] = 1529806848U;
      rtDW.NextOutput[2] = rt_nrand_Upu32_Yd_f_pw(&rtDW.RandSeed[2]);

      // InitializeConditions for DiscreteIntegrator: '<S65>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_DSTAT_d[2] = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S65>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_DS_h[2] = 0.0F;

      // InitializeConditions for RandomNumber: '<S64>/White Noise'
      rtDW.RandSeed[3] = 1529872384U;
      rtDW.NextOutput[3] = rt_nrand_Upu32_Yd_f_pw(&rtDW.RandSeed[3]);

      // InitializeConditions for DiscreteIntegrator: '<S65>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_DSTAT_d[3] = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S65>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_DS_h[3] = 0.0F;
      rtDW.CopterRandomExcitation_MODE = true;
    }

    // Product: '<S54>/Divide' incorporates:
    //   Constant: '<S54>/Constant3'
    //   Constant: '<S54>/Constant4'
    //   Product: '<S65>/Divide'
    //   Product: '<S65>/omega^2'

    scale = 3.0F / (2.0F / rtP.lindi.sflt.omega + rtP.lindi.mtc);
    rtb_y_a0 = 1.0F / scale;

    // Sqrt: '<S54>/Sqrt' incorporates:
    //   Product: '<S54>/Divide'

    t = std::sqrt(rtb_y_a0);

    // Gain: '<S65>/Gain'
    cos_Theta = rtb_y_a0 * 2.0F;

    // Sum: '<S65>/Sum2' incorporates:
    //   DiscreteIntegrator: '<S65>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S65>/Discrete-Time Integrator y_dt'
    //   Gain: '<S54>/Gain4'
    //   Gain: '<S64>/Output'
    //   Product: '<S54>/Product'
    //   Product: '<S65>/Product2'
    //   RandomNumber: '<S64>/White Noise'
    //   Sum: '<S65>/Sum3'

    rtb_Add_a[0] = (real32_T)(3.1622776955095868 * rtDW.NextOutput[0] * 3.0) * t
      - (rtDW.DiscreteTimeIntegratory_dt_DS_h[0] * cos_Theta +
         rtDW.DiscreteTimeIntegratory_DSTAT_d[0]);
    rtb_Add_a[1] = (real32_T)(3.1622776955095868 * rtDW.NextOutput[1] * 3.0) * t
      - (rtDW.DiscreteTimeIntegratory_dt_DS_h[1] * cos_Theta +
         rtDW.DiscreteTimeIntegratory_DSTAT_d[1]);
    rtb_Add_a[2] = (real32_T)(3.1622776955095868 * rtDW.NextOutput[2] * 3.0) * t
      - (rtDW.DiscreteTimeIntegratory_dt_DS_h[2] * cos_Theta +
         rtDW.DiscreteTimeIntegratory_DSTAT_d[2]);
    rtb_Add_a[3] = (real32_T)(3.1622776955095868 * rtDW.NextOutput[3] * 3.0) * t
      - (rtDW.DiscreteTimeIntegratory_dt_DS_h[3] * cos_Theta +
         rtDW.DiscreteTimeIntegratory_DSTAT_d[3]);

    // Product: '<S65>/omega^2'
    cos_Theta = scale * scale;

    // Gain: '<S54>/Gain' incorporates:
    //   DiscreteIntegrator: '<S65>/Discrete-Time Integrator y'

    rtDW.Gain[0] = 3.5F * rtDW.DiscreteTimeIntegratory_DSTAT_d[0];
    rtDW.Gain[1] = 3.5F * rtDW.DiscreteTimeIntegratory_DSTAT_d[1];
    rtDW.Gain[2] = 3.5F * rtDW.DiscreteTimeIntegratory_DSTAT_d[2];

    // Gain: '<S54>/Gain1' incorporates:
    //   DiscreteIntegrator: '<S65>/Discrete-Time Integrator y_dt'
    //   Gain: '<S54>/Gain3'

    rtDW.Gain1 = 0.15F * rtDW.DiscreteTimeIntegratory_dt_DS_h[3] *
      rtP.lindi.atc.rm.yawratetc;

    // Update for RandomNumber: '<S64>/White Noise'
    rtDW.NextOutput[0] = rt_nrand_Upu32_Yd_f_pw(&rtDW.RandSeed[0]);

    // Update for DiscreteIntegrator: '<S65>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S65>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTAT_d[0] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_h[0];

    // Update for DiscreteIntegrator: '<S65>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S65>/Product1'

    rtDW.DiscreteTimeIntegratory_dt_DS_h[0] += rtb_Add_a[0] * cos_Theta *
      0.0025F;

    // Update for RandomNumber: '<S64>/White Noise'
    rtDW.NextOutput[1] = rt_nrand_Upu32_Yd_f_pw(&rtDW.RandSeed[1]);

    // Update for DiscreteIntegrator: '<S65>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S65>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTAT_d[1] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_h[1];

    // Update for DiscreteIntegrator: '<S65>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S65>/Product1'

    rtDW.DiscreteTimeIntegratory_dt_DS_h[1] += rtb_Add_a[1] * cos_Theta *
      0.0025F;

    // Update for RandomNumber: '<S64>/White Noise'
    rtDW.NextOutput[2] = rt_nrand_Upu32_Yd_f_pw(&rtDW.RandSeed[2]);

    // Update for DiscreteIntegrator: '<S65>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S65>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTAT_d[2] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_h[2];

    // Update for DiscreteIntegrator: '<S65>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S65>/Product1'

    rtDW.DiscreteTimeIntegratory_dt_DS_h[2] += rtb_Add_a[2] * cos_Theta *
      0.0025F;

    // Update for RandomNumber: '<S64>/White Noise'
    rtDW.NextOutput[3] = rt_nrand_Upu32_Yd_f_pw(&rtDW.RandSeed[3]);

    // Update for DiscreteIntegrator: '<S65>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S65>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTAT_d[3] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_h[3];

    // Update for DiscreteIntegrator: '<S65>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S65>/Product1'

    rtDW.DiscreteTimeIntegratory_dt_DS_h[3] += rtb_Add_a[3] * cos_Theta *
      0.0025F;
  } else {
    if (rtDW.CopterRandomExcitation_MODE) {
      // Disable for Outport: '<S54>/yaw_rate_excite'
      rtDW.Gain1 = 0.0F;

      // Disable for Outport: '<S54>/s_g_dt2_excite'
      rtDW.Gain[0] = 0.0F;
      rtDW.Gain[1] = 0.0F;
      rtDW.Gain[2] = 0.0F;
      rtDW.CopterRandomExcitation_MODE = false;
    }
  }

  // End of Outputs for SubSystem: '<S52>/Copter Random Excitation'

  // Sum: '<S4>/Add1'
  rtb_Sum2_hq[2] = rtDW.Gain[2] + rtDW.nu[2];

  // Outputs for Enabled SubSystem: '<S4>/Accelerations to Reduced Attitude and Thrust' incorporates:
  //   EnablePort: '<S37>/Enable'

  // Logic: '<S4>/Logical Operator' incorporates:
  //   DiscreteIntegrator: '<S89>/Discrete-Time Integrator y'
  //   Logic: '<S4>/Logical Operator1'
  //   Logic: '<S4>/Logical Operator2'
  //   MATLAB Function: '<S37>/INDI Copter Acc 2 Lean Vector'
  //   Sum: '<S4>/Add1'

  if ((!rtb_UnitDelay1_j) && (!rtb_UnitDelay4_n)) {
    // MATLAB Function: '<S37>/INDI Copter Acc 2 Lean Vector' incorporates:
    //   Constant: '<S4>/Constant2'
    //   DiscreteIntegrator: '<S89>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S90>/Discrete-Time Integrator y'

    rtb_y_o[0] = rtDW.DiscreteTimeIntegratory_DSTA_al[0];
    rtb_y_o[1] = rtDW.DiscreteTimeIntegratory_DSTA_al[1];
    rtb_y_o[2] = rtDW.DiscreteTimeIntegratory_DSTA_al[2] - 9.81F;
    t = 0.0F;
    for (k = 0; k < 3; k++) {
      q2_q3 = -rtDW.DiscreteTimeIntegratory_DSTAT_p[3 * k + 2];
      t += q2_q3 * rtb_y_o[k];
      rtb_n_g_des_n[k] = q2_q3;
    }

    rtb_n_g_des_n[0] = ((rtDW.Gain[0] + rtDW.nu[0]) -
                        rtDW.DiscreteTimeIntegratory_DSTA_al[0]) +
      rtb_n_g_des_n[0] * t;
    rtb_n_g_des_n[1] = ((rtDW.Gain[1] + rtDW.nu[1]) -
                        rtDW.DiscreteTimeIntegratory_DSTA_al[1]) +
      rtb_n_g_des_n[1] * t;

    // MATLAB Function: '<S37>/INDI Copter Acc 2 Lean Vector' incorporates:
    //   DiscreteIntegrator: '<S89>/Discrete-Time Integrator y'
    //   Sum: '<S4>/Add1'

    q2_q3 = rtb_n_g_des_n[2] * t + (rtb_Sum2_hq[2] -
      rtDW.DiscreteTimeIntegratory_DSTA_al[2]);
    scale = 1.29246971E-26F;
    absxk = std::abs(rtb_n_g_des_n[0]);
    if (absxk > 1.29246971E-26F) {
      cos_Theta = 1.0F;
      scale = absxk;
    } else {
      t = absxk / 1.29246971E-26F;
      cos_Theta = t * t;
    }

    absxk = std::abs(rtb_n_g_des_n[1]);
    if (absxk > scale) {
      t = scale / absxk;
      cos_Theta = cos_Theta * t * t + 1.0F;
      scale = absxk;
    } else {
      t = absxk / scale;
      cos_Theta += t * t;
    }

    absxk = std::abs(q2_q3);
    if (absxk > scale) {
      t = scale / absxk;
      cos_Theta = cos_Theta * t * t + 1.0F;
      scale = absxk;
    } else {
      t = absxk / scale;
      cos_Theta += t * t;
    }

    cos_Theta = scale * std::sqrt(cos_Theta);
    t = cos_Theta;
    if (cos_Theta < 2.22044605E-16F) {
      t = 2.22044605E-16F;
    }

    q0_q2 = 1.0F / t;
    scale = q0_q2 * rtb_n_g_des_n[0];

    // SignalConversion: '<S37>/BusConversion_InsertedFor_red_atti_des_at_inport_0' 
    rtDW.n_g_des[0] = scale;
    rtb_n_g_des_n[0] = scale;

    // MATLAB Function: '<S37>/INDI Copter Acc 2 Lean Vector'
    scale = q0_q2 * rtb_n_g_des_n[1];

    // SignalConversion: '<S37>/BusConversion_InsertedFor_red_atti_des_at_inport_0' 
    rtDW.n_g_des[1] = scale;
    rtb_n_g_des_n[1] = scale;

    // MATLAB Function: '<S37>/INDI Copter Acc 2 Lean Vector'
    scale = q0_q2 * q2_q3;

    // SignalConversion: '<S37>/BusConversion_InsertedFor_red_atti_des_at_inport_0' 
    rtDW.n_g_des[2] = scale;

    // MATLAB Function: '<S37>/MATLAB Function4'
    if (1.0F > -scale) {
      q0_q2 = -scale;
    } else {
      q0_q2 = 1.0F;
    }

    // SignalConversion: '<S37>/BusConversion_InsertedFor_red_atti_des_at_inport_0' incorporates:
    //   MATLAB Function: '<S37>/MATLAB Function4'

    rtDW.lean_dir_angle_des = std::atan2(rtb_n_g_des_n[1], rtb_n_g_des_n[0]);

    // MATLAB Function: '<S37>/MATLAB Function4'
    if (-1.0F >= q0_q2) {
      q0_q2 = -1.0F;
    }

    t = std::acos(q0_q2);

    // Saturate: '<S37>/Saturation1'
    if (t > rtP.lindi.atc.rm.leanmax) {
      t = rtP.lindi.atc.rm.leanmax;
    } else {
      if (t < 0.0F) {
        t = 0.0F;
      }
    }

    // End of Saturate: '<S37>/Saturation1'

    // SignalConversion: '<S37>/BusConversion_InsertedFor_red_atti_des_at_inport_0' incorporates:
    //   Gain: '<S37>/Gain1'

    rtDW.cmd_lean_angle_01 = 1.0F / rtP.lindi.atc.rm.leanmax * t;

    // SignalConversion: '<S37>/OutportBufferForT_spec_des' incorporates:
    //   MATLAB Function: '<S37>/INDI Copter Acc 2 Lean Vector'

    rtDW.Merge1_e = cos_Theta;
  }

  // End of Logic: '<S4>/Logical Operator'
  // End of Outputs for SubSystem: '<S4>/Accelerations to Reduced Attitude and Thrust' 

  // SignalConversion: '<S121>/TmpSignal ConversionAt SFunction Inport1' incorporates:
  //   Gain: '<S111>/leanmax'
  //   MATLAB Function: '<S103>/lean angles 2 lean vector'

  absxk = rtP.lindi.atc.rm.leanmax * rtDW.cmd_lean_angle_01;

  // MATLAB Function: '<S103>/lean angles 2 lean vector' incorporates:
  //   SignalConversion: '<S121>/TmpSignal ConversionAt SFunction Inport1'

  t = std::sin(absxk);
  rtb_n_g_des_n[0] = t * std::cos(rtDW.lean_dir_angle_des);
  rtb_n_g_des_n[1] = t * std::sin(rtDW.lean_dir_angle_des);
  rtb_n_g_des_n[2] = -std::cos(absxk);

  // DiscreteIntegrator: '<S123>/Discrete-Time Integrator y'
  if (rtDW.DiscreteTimeIntegratory_IC_LO_b != 0) {
    rtDW.DiscreteTimeIntegratory_DSTAT_m[0] = rtb_n_g_des_n[0];
    rtDW.DiscreteTimeIntegratory_DSTAT_m[1] = rtb_n_g_des_n[1];
    rtDW.DiscreteTimeIntegratory_DSTAT_m[2] = rtb_n_g_des_n[2];
  }

  // Outputs for Enabled SubSystem: '<S52>/Adaptive INDI G1 and G2 correction' incorporates:
  //   EnablePort: '<S53>/Enable'

  // Outputs for Enabled SubSystem: '<S4>/Incremental specific thrust' incorporates:
  //   EnablePort: '<S40>/Enable'

  // Product: '<S123>/omega^2' incorporates:
  //   Constant: '<S117>/omega'
  //   MATLAB Function: '<S53>/G1 learn rate'
  //   Product: '<S86>/omega^2'

  q1_q2 = rtP.lindi.atc.rm.leanfreq * rtP.lindi.atc.rm.leanfreq;

  // End of Outputs for SubSystem: '<S52>/Adaptive INDI G1 and G2 correction'

  // Gain: '<S123>/Gain' incorporates:
  //   Constant: '<S117>/d'
  //   Constant: '<S117>/omega'
  //   Gain: '<S86>/Gain'
  //   Product: '<S123>/Divide'

  rtb_y_a0 = rtP.lindi.atc.rm.leandamp / rtP.lindi.atc.rm.leanfreq * 2.0F;

  // End of Outputs for SubSystem: '<S4>/Incremental specific thrust'

  // DiscreteIntegrator: '<S123>/Discrete-Time Integrator y'
  rtb_y_o[0] = rtDW.DiscreteTimeIntegratory_DSTAT_m[0];

  // DiscreteIntegrator: '<S123>/Discrete-Time Integrator y_dt'
  rtb_p_ahead_l[0] = rtDW.DiscreteTimeIntegratory_dt_DS_j[0];

  // Product: '<S123>/Product1' incorporates:
  //   DiscreteIntegrator: '<S123>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S123>/Discrete-Time Integrator y_dt'
  //   Gain: '<S123>/Gain'
  //   Product: '<S123>/Product2'
  //   Product: '<S123>/omega^2'
  //   Sum: '<S123>/Sum2'
  //   Sum: '<S123>/Sum3'

  rtb_n_g_des_n[0] = (rtb_n_g_des_n[0] - (rtDW.DiscreteTimeIntegratory_dt_DS_j[0]
    * rtb_y_a0 + rtDW.DiscreteTimeIntegratory_DSTAT_m[0])) * q1_q2;

  // DiscreteIntegrator: '<S123>/Discrete-Time Integrator y'
  rtb_y_o[1] = rtDW.DiscreteTimeIntegratory_DSTAT_m[1];

  // DiscreteIntegrator: '<S123>/Discrete-Time Integrator y_dt'
  rtb_p_ahead_l[1] = rtDW.DiscreteTimeIntegratory_dt_DS_j[1];

  // Product: '<S123>/Product1' incorporates:
  //   DiscreteIntegrator: '<S123>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S123>/Discrete-Time Integrator y_dt'
  //   Gain: '<S123>/Gain'
  //   Product: '<S123>/Product2'
  //   Product: '<S123>/omega^2'
  //   Sum: '<S123>/Sum2'
  //   Sum: '<S123>/Sum3'

  rtb_n_g_des_n[1] = (rtb_n_g_des_n[1] - (rtDW.DiscreteTimeIntegratory_dt_DS_j[1]
    * rtb_y_a0 + rtDW.DiscreteTimeIntegratory_DSTAT_m[1])) * q1_q2;

  // DiscreteIntegrator: '<S123>/Discrete-Time Integrator y'
  rtb_y_o[2] = rtDW.DiscreteTimeIntegratory_DSTAT_m[2];

  // DiscreteIntegrator: '<S123>/Discrete-Time Integrator y_dt'
  rtb_p_ahead_l[2] = rtDW.DiscreteTimeIntegratory_dt_DS_j[2];

  // Product: '<S123>/Product1' incorporates:
  //   DiscreteIntegrator: '<S123>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S123>/Discrete-Time Integrator y_dt'
  //   Gain: '<S123>/Gain'
  //   Product: '<S123>/Product2'
  //   Product: '<S123>/omega^2'
  //   Sum: '<S123>/Sum2'
  //   Sum: '<S123>/Sum3'

  q2_q3 = (rtb_n_g_des_n[2] - (rtDW.DiscreteTimeIntegratory_dt_DS_j[2] *
            rtb_y_a0 + rtDW.DiscreteTimeIntegratory_DSTAT_m[2])) * q1_q2;
  rtb_n_g_des_n[2] = q2_q3;

  // MATLAB Function: '<S117>/n ref norm'
  nrefnorm(rtb_y_o, rtb_p_ahead_l, rtb_n_g_des_n, rtb_n, rtb_n_dt_j, rtb_n_dt2_h);

  // DiscreteIntegrator: '<S125>/Discrete-Time Integrator y'
  if (rtDW.DiscreteTimeIntegratory_IC_LO_j != 0) {
    rtDW.DiscreteTimeIntegratory_DSTA_pz[0] = rtb_n[0];
    rtDW.DiscreteTimeIntegratory_DSTA_pz[1] = rtb_n[1];
    rtDW.DiscreteTimeIntegratory_DSTA_pz[2] = rtb_n[2];
  }

  // Product: '<S125>/omega^2' incorporates:
  //   Constant: '<S118>/omega'
  //   Product: '<S125>/Divide'
  //   Product: '<S69>/omega^2'
  //   Product: '<S70>/omega^2'

  scale = 2.0F / (2.0F / rtP.lindi.sflt.omega + rtP.lindi.mtc);

  // Outputs for Enabled SubSystem: '<S52>/INDI Inversion Check' incorporates:
  //   EnablePort: '<S55>/Enable'

  wp_approach_out_tmp = scale * scale;

  // Gain: '<S125>/Gain' incorporates:
  //   Constant: '<S118>/d'
  //   Gain: '<S69>/Gain'
  //   Gain: '<S70>/Gain'
  //   Product: '<S125>/Divide'

  scale_tmp = 1.0F / scale * 2.0F;

  // End of Outputs for SubSystem: '<S52>/INDI Inversion Check'

  // DiscreteIntegrator: '<S125>/Discrete-Time Integrator y'
  rtb_Sum2_e[0] = rtDW.DiscreteTimeIntegratory_DSTA_pz[0];

  // DiscreteIntegrator: '<S125>/Discrete-Time Integrator y_dt'
  rtb_n_b_dt2[0] = rtDW.DiscreteTimeIntegratory_dt_DS_b[0];

  // Product: '<S125>/Product1' incorporates:
  //   DiscreteIntegrator: '<S125>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S125>/Discrete-Time Integrator y_dt'
  //   Gain: '<S125>/Gain'
  //   Product: '<S125>/Product2'
  //   Product: '<S125>/omega^2'
  //   Sum: '<S125>/Sum2'
  //   Sum: '<S125>/Sum3'

  rtb_y_o[0] = (rtb_n[0] - (rtDW.DiscreteTimeIntegratory_dt_DS_b[0] * scale_tmp
    + rtDW.DiscreteTimeIntegratory_DSTA_pz[0])) * wp_approach_out_tmp;

  // DiscreteIntegrator: '<S125>/Discrete-Time Integrator y'
  rtb_Sum2_e[1] = rtDW.DiscreteTimeIntegratory_DSTA_pz[1];

  // DiscreteIntegrator: '<S125>/Discrete-Time Integrator y_dt'
  rtb_n_b_dt2[1] = rtDW.DiscreteTimeIntegratory_dt_DS_b[1];

  // Product: '<S125>/Product1' incorporates:
  //   DiscreteIntegrator: '<S125>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S125>/Discrete-Time Integrator y_dt'
  //   Gain: '<S125>/Gain'
  //   Product: '<S125>/Product2'
  //   Product: '<S125>/omega^2'
  //   Sum: '<S125>/Sum2'
  //   Sum: '<S125>/Sum3'

  rtb_y_o[1] = (rtb_n[1] - (rtDW.DiscreteTimeIntegratory_dt_DS_b[1] * scale_tmp
    + rtDW.DiscreteTimeIntegratory_DSTA_pz[1])) * wp_approach_out_tmp;

  // DiscreteIntegrator: '<S125>/Discrete-Time Integrator y'
  rtb_Sum2_e[2] = rtDW.DiscreteTimeIntegratory_DSTA_pz[2];

  // DiscreteIntegrator: '<S125>/Discrete-Time Integrator y_dt'
  rtb_n_b_dt2[2] = rtDW.DiscreteTimeIntegratory_dt_DS_b[2];

  // Product: '<S125>/Product1' incorporates:
  //   DiscreteIntegrator: '<S125>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S125>/Discrete-Time Integrator y_dt'
  //   Gain: '<S125>/Gain'
  //   Product: '<S125>/Product2'
  //   Product: '<S125>/omega^2'
  //   Sum: '<S125>/Sum2'
  //   Sum: '<S125>/Sum3'

  rtb_y_o[2] = (rtb_n[2] - (rtDW.DiscreteTimeIntegratory_dt_DS_b[2] * scale_tmp
    + rtDW.DiscreteTimeIntegratory_DSTA_pz[2])) * wp_approach_out_tmp;

  // MATLAB Function: '<S118>/n ref norm'
  nrefnorm(rtb_Sum2_e, rtb_n_b_dt2, rtb_y_o, rtb_p_ahead_l, rtb_n_dt, rtb_n_dt2);

  // DiscreteIntegrator: '<S93>/Discrete-Time Integrator y' incorporates:
  //   Inport: '<Root>/measure'

  if (rtDW.DiscreteTimeIntegratory_IC_L_jp != 0) {
    rtDW.DiscreteTimeIntegratory_DSTAT_k[0] = rtU.measure.omega_Kb[0];
    rtDW.DiscreteTimeIntegratory_DSTAT_k[1] = rtU.measure.omega_Kb[1];
    rtDW.DiscreteTimeIntegratory_DSTAT_k[2] = rtU.measure.omega_Kb[2];
  }

  rtb_y_a[0] = rtDW.DiscreteTimeIntegratory_DSTAT_k[0];
  rtb_y_a[1] = rtDW.DiscreteTimeIntegratory_DSTAT_k[1];
  rtb_y_a[2] = rtDW.DiscreteTimeIntegratory_DSTAT_k[2];

  // MATLAB Function: '<S103>/Lean Vector Derivative Trafo Delay'
  LeanVectorDerivativeTrafo(rtb_p_ahead_l, rtb_n_dt, rtb_n_dt2, rtb_Sum2_jx,
    rtb_y_a, a_Kb_meas, rtb_n_b, rtb_n_b_dt, rtb_Sum2_e);

  // MATLAB Function: '<S103>/Lean Vector Derivative Trafo'
  LeanVectorDerivativeTrafo(rtb_n, rtb_n_dt_j, rtb_n_dt2_h, rtb_Sum2_jx, rtb_y_a,
    a_Kb_meas, rtb_n_b_h, rtb_n_b_dt_i, rtb_n_b_dt2);

  // MATLAB Function: '<S103>/MATLAB Function'
  rtb_y_a[0] = rtb_n_b[0];
  rtb_y_a[1] = rtb_n_b[1];
  if (rtb_n_b[2] > 0.0F) {
    if (rtb_n_b[2] > 0.999) {
      absxk = 0.0F;
      rtb_n_b[0] = 0.0F;
      q0_q2 = -1.0F;
      rtb_n_b[1] = 0.0F;
    } else {
      scale = 1.29246971E-26F;
      absxk = std::abs(rtb_n_b[0]);
      if (absxk > 1.29246971E-26F) {
        q0_q2 = 1.0F;
        scale = absxk;
      } else {
        t = absxk / 1.29246971E-26F;
        q0_q2 = t * t;
      }

      absxk = std::abs(rtb_n_b[1]);
      if (absxk > scale) {
        t = scale / absxk;
        q0_q2 = q0_q2 * t * t + 1.0F;
        scale = absxk;
      } else {
        t = absxk / scale;
        q0_q2 += t * t;
      }

      q0_q2 = scale * std::sqrt(q0_q2);
      if (q0_q2 < 2.22044605E-16F) {
        q0_q2 = 2.22044605E-16F;
      }

      absxk = rtb_n_b[0] / q0_q2;
      q0_q2 = rtb_n_b[1] / q0_q2;
    }

    rtb_y_a[0] = 2.0F * absxk - rtb_n_b[0];
    rtb_n_b_dt[0] = -rtb_n_b_dt[0];
    rtb_Sum2_e[0] = -rtb_Sum2_e[0];
    rtb_y_a[1] = 2.0F * q0_q2 - rtb_n_b[1];
    rtb_n_b_dt[1] = -rtb_n_b_dt[1];
    rtb_Sum2_e[1] = -rtb_Sum2_e[1];
  }

  // MATLAB Function: '<S97>/Reduced Attitude Weighting Factors' incorporates:
  //   MATLAB Function: '<S103>/MATLAB Function'

  absxk = -rtb_n_b[2];
  if (-rtb_n_b[2] < 0.0F) {
    absxk = 0.0F;
  }

  // Product: '<S122>/Divide' incorporates:
  //   Constant: '<S122>/T'
  //   DiscreteIntegrator: '<S122>/Discrete-Time Integrator'
  //   Gain: '<S103>/r_max'
  //   Sum: '<S122>/Sum2'
  //   Sum: '<S4>/Add'

  cos_Theta = ((rtDW.Gain1 + rtDW.Merge[2]) * rtP.lindi.atc.rm.yawratemax -
               rtDW.DiscreteTimeIntegrator_DSTATE) / rtP.lindi.atc.rm.yawratetc;

  // MATLAB Function: '<S103>/Simulink Trickster' incorporates:
  //   DiscreteIntegrator: '<S122>/Discrete-Time Integrator'

  scale = rtDW.DiscreteTimeIntegrator_DSTATE;

  // MATLAB Function: '<S103>/Desired Roll Pitch' incorporates:
  //   DiscreteIntegrator: '<S90>/Discrete-Time Integrator y'

  rtb_n_b[0] = rtb_p_ahead_l[1] * rtb_n_dt[2] - rtb_p_ahead_l[2] * rtb_n_dt[1];
  rtb_n_b[1] = rtb_p_ahead_l[2] * rtb_n_dt[0] - rtb_p_ahead_l[0] * rtb_n_dt[2];
  rtb_n_b[2] = rtb_p_ahead_l[0] * rtb_n_dt[1] - rtb_p_ahead_l[1] * rtb_n_dt[0];
  for (wp_idx_app_2 = 0; wp_idx_app_2 < 3; wp_idx_app_2++) {
    rtb_n_b_h[wp_idx_app_2] = rtDW.DiscreteTimeIntegratory_DSTAT_p[wp_idx_app_2
      + 6] * rtb_n_b[2] + (rtDW.DiscreteTimeIntegratory_DSTAT_p[wp_idx_app_2 + 3]
      * rtb_n_b[1] + rtDW.DiscreteTimeIntegratory_DSTAT_p[wp_idx_app_2] *
      rtb_n_b[0]);
  }

  // MATLAB Function: '<S94>/DCM to quaternions'
  DCMtoquaternions(rtb_Sum2_jx, rtb_Add_a);

  // MATLAB Function: '<S94>/Quaternion Reduced'
  QuaternionReduced(rtb_Add_a, rtb_q_red_l, &t);

  // DiscreteIntegrator: '<S103>/Discrete-Time Integrator2'
  if (rtDW.DiscreteTimeIntegrator2_IC_LOAD != 0) {
    rtDW.DiscreteTimeIntegrator2_DSTATE = t;
  }

  // MATLAB Function: '<S98>/wrap angle' incorporates:
  //   DiscreteIntegrator: '<S103>/Discrete-Time Integrator2'

  wrapangle(rtDW.DiscreteTimeIntegrator2_DSTATE, &q1_q3);

  // MATLAB Function: '<S99>/DCM to quaternions'
  DCMtoquaternions(rtb_Sum2_jx, rtb_Add_a);

  // MATLAB Function: '<S99>/Quaternion Reduced'
  QuaternionReduced(rtb_Add_a, rtb_q_red_l, &t);

  // MATLAB Function: '<S98>/wrap angle1'
  wrapangle(t, &q0_q2);

  // MATLAB Function: '<S98>/angle error'
  t = q1_q3 - q0_q2;
  if (t > 3.1415926535897931) {
    t -= 6.28318548F;
  } else {
    if (t < -3.1415926535897931) {
      t += 6.28318548F;
    }
  }

  // End of MATLAB Function: '<S98>/angle error'

  // MATLAB Function: '<S103>/Pseudo-Control Roll Pitch' incorporates:
  //   DiscreteIntegrator: '<S93>/Discrete-Time Integrator y'

  rtb_n_b_dt_i[0] = rtb_n[1] * rtb_n_dt2_h[2] - rtb_n[2] * rtb_n_dt2_h[1];
  rtb_n_b_dt_i[1] = rtb_n[2] * rtb_n_dt2_h[0] - rtb_n[0] * rtb_n_dt2_h[2];
  rtb_n_b_dt_i[2] = rtb_n[0] * rtb_n_dt2_h[1] - rtb_n[1] * rtb_n_dt2_h[0];
  cos_Theta_0[0] = 0.0F;
  cos_Theta_0[3] = -rtDW.DiscreteTimeIntegratory_DSTAT_k[2];
  cos_Theta_0[6] = rtDW.DiscreteTimeIntegratory_DSTAT_k[1];
  cos_Theta_0[1] = rtDW.DiscreteTimeIntegratory_DSTAT_k[2];
  cos_Theta_0[4] = 0.0F;
  cos_Theta_0[7] = -rtDW.DiscreteTimeIntegratory_DSTAT_k[0];
  cos_Theta_0[2] = -rtDW.DiscreteTimeIntegratory_DSTAT_k[1];
  cos_Theta_0[5] = rtDW.DiscreteTimeIntegratory_DSTAT_k[0];
  cos_Theta_0[8] = 0.0F;
  rtb_n_dt2_h[0] = rtb_n[1] * rtb_n_dt_j[2] - rtb_n[2] * rtb_n_dt_j[1];
  rtb_n_dt2_h[1] = rtb_n[2] * rtb_n_dt_j[0] - rtb_n[0] * rtb_n_dt_j[2];
  rtb_n_dt2_h[2] = rtb_n[0] * rtb_n_dt_j[1] - rtb_n[1] * rtb_n_dt_j[0];

  // MATLAB Function: '<S103>/Desired Roll Pitch' incorporates:
  //   DiscreteIntegrator: '<S93>/Discrete-Time Integrator y'

  q0_q2 = rtb_p_ahead_l[1] * rtb_n_dt2[2] - rtb_p_ahead_l[2] * rtb_n_dt2[1];
  rtb_p_ahead_c_idx_1 = rtb_p_ahead_l[2] * rtb_n_dt2[0] - rtb_p_ahead_l[0] *
    rtb_n_dt2[2];
  rtb_p_ahead_c_idx_2 = rtb_p_ahead_l[0] * rtb_n_dt2[1] - rtb_p_ahead_l[1] *
    rtb_n_dt2[0];
  tmp_1[0] = 0.0F;
  tmp_1[3] = -rtDW.DiscreteTimeIntegratory_DSTAT_k[2];
  tmp_1[6] = rtDW.DiscreteTimeIntegratory_DSTAT_k[1];
  tmp_1[1] = rtDW.DiscreteTimeIntegratory_DSTAT_k[2];
  tmp_1[4] = 0.0F;
  tmp_1[7] = -rtDW.DiscreteTimeIntegratory_DSTAT_k[0];
  tmp_1[2] = -rtDW.DiscreteTimeIntegratory_DSTAT_k[1];
  tmp_1[5] = rtDW.DiscreteTimeIntegratory_DSTAT_k[0];
  tmp_1[8] = 0.0F;
  for (wp_idx_app_2 = 0; wp_idx_app_2 < 3; wp_idx_app_2++) {
    // MATLAB Function: '<S103>/Pseudo-Control Roll Pitch' incorporates:
    //   DiscreteIntegrator: '<S90>/Discrete-Time Integrator y'
    //   MATLAB Function: '<S103>/Desired Roll Pitch'

    rtb_n[wp_idx_app_2] = 0.0F;
    rtb_p_ahead_l[wp_idx_app_2] = 0.0F;
    for (flight_mode = 0; flight_mode < 3; flight_mode++) {
      // MATLAB Function: '<S103>/Desired Roll Pitch'
      b_stage_ = wp_idx_app_2 + 3 * flight_mode;
      tmp_0[b_stage_] = 0.0F;

      // MATLAB Function: '<S103>/Desired Roll Pitch' incorporates:
      //   DiscreteIntegrator: '<S90>/Discrete-Time Integrator y'

      c_stage_app = 3 * flight_mode + wp_idx_app_2;
      q1_q3 = rtDW.DiscreteTimeIntegratory_DSTAT_p[3 * flight_mode];
      tmp_0[b_stage_] = tmp_0[c_stage_app] + q1_q3 * cos_Theta_0[3 *
        wp_idx_app_2];

      // MATLAB Function: '<S103>/Desired Roll Pitch' incorporates:
      //   DiscreteIntegrator: '<S90>/Discrete-Time Integrator y'

      k = 3 * wp_idx_app_2 + 1;
      tmp_3 = rtDW.DiscreteTimeIntegratory_DSTAT_p[3 * flight_mode + 1];
      tmp_0[b_stage_] = tmp_3 * cos_Theta_0[k] + tmp_0[c_stage_app];

      // MATLAB Function: '<S103>/Desired Roll Pitch' incorporates:
      //   DiscreteIntegrator: '<S90>/Discrete-Time Integrator y'

      rtb_DataTypeConversion2 = 3 * wp_idx_app_2 + 2;
      tmp_4 = rtDW.DiscreteTimeIntegratory_DSTAT_p[3 * flight_mode + 2];
      tmp_0[b_stage_] = tmp_4 * cos_Theta_0[rtb_DataTypeConversion2] +
        tmp_0[c_stage_app];

      // MATLAB Function: '<S103>/Desired Roll Pitch'
      tmp_2[b_stage_] = 0.0F;
      tmp_2[b_stage_] = tmp_2[c_stage_app] + q1_q3 * tmp_1[3 * wp_idx_app_2];
      tmp_2[b_stage_] = tmp_3 * tmp_1[k] + tmp_2[c_stage_app];
      tmp_2[b_stage_] = tmp_4 * tmp_1[rtb_DataTypeConversion2] +
        tmp_2[c_stage_app];
      rtb_n[wp_idx_app_2] += rtDW.DiscreteTimeIntegratory_DSTAT_p[c_stage_app] *
        rtb_n_b_dt_i[flight_mode];
      rtb_p_ahead_l[wp_idx_app_2] += tmp_0[c_stage_app] *
        rtb_n_dt2_h[flight_mode];
    }

    rtb_n_dt_j[wp_idx_app_2] = rtb_n[wp_idx_app_2] + rtb_p_ahead_l[wp_idx_app_2];

    // MATLAB Function: '<S103>/Desired Roll Pitch' incorporates:
    //   DiscreteIntegrator: '<S90>/Discrete-Time Integrator y'

    rtb_n_dt2[wp_idx_app_2] = (rtDW.DiscreteTimeIntegratory_DSTAT_p[wp_idx_app_2
      + 6] * rtb_p_ahead_c_idx_2 +
      (rtDW.DiscreteTimeIntegratory_DSTAT_p[wp_idx_app_2 + 3] *
       rtb_p_ahead_c_idx_1 + rtDW.DiscreteTimeIntegratory_DSTAT_p[wp_idx_app_2] *
       q0_q2)) + (tmp_2[wp_idx_app_2 + 6] * rtb_n_b[2] + (tmp_2[wp_idx_app_2 + 3]
      * rtb_n_b[1] + tmp_2[wp_idx_app_2] * rtb_n_b[0]));
  }

  // Sum: '<S42>/Add2' incorporates:
  //   DiscreteIntegrator: '<S122>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<S93>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S93>/Discrete-Time Integrator y_dt'
  //   Gain: '<S100>/Gain'
  //   Gain: '<S100>/Gain1'
  //   Gain: '<S100>/Gain2'
  //   Gain: '<S100>/Gain3'
  //   Gain: '<S100>/Gain4'
  //   Gain: '<S100>/Gain5'
  //   Gain: '<S100>/Gain6'
  //   Gain: '<S100>/Gain7'
  //   Gain: '<S100>/Gain8'
  //   Gain: '<S101>/Gain'
  //   MATLAB Function: '<S103>/Desired Roll Pitch'
  //   MATLAB Function: '<S103>/MATLAB Function'
  //   MATLAB Function: '<S103>/Pseudo-Control Roll Pitch'
  //   MATLAB Function: '<S103>/Simulink Trickster'
  //   MATLAB Function: '<S97>/Reduced Attitude Weighting Factors'
  //   Product: '<S100>/Product'
  //   Product: '<S100>/Product1'
  //   Product: '<S100>/Product3'
  //   Product: '<S100>/Product4'
  //   Product: '<S101>/Product'
  //   Product: '<S101>/Product1'
  //   Product: '<S102>/Product'
  //   Sum: '<S100>/Add'
  //   Sum: '<S100>/Add1'
  //   Sum: '<S100>/Add2'
  //   Sum: '<S100>/Add3'
  //   Sum: '<S101>/Add'
  //   Sum: '<S102>/Add'
  //   Sum: '<S42>/Add1'
  //   Sum: '<S98>/error1 1'
  //   Sum: '<S98>/error1 2'
  //   Sum: '<S98>/error1 4'
  //   Sum: '<S98>/error1 5'
  //   Sum: '<S98>/error1 6'
  //   Sum: '<S98>/error1 8'
  //   Sum: '<S98>/error1 9'

  rtb_n_dt[0] = (((((1.0F - absxk) * (rtb_n_b_h[0] -
    rtDW.DiscreteTimeIntegratory_DSTAT_k[0]) + absxk * rtb_n_b_dt[1]) *
                   rtP.lindi.atc.k.leanrate + rtP.lindi.atc.k.lean * rtb_y_a[1])
                  + ((1.0F - absxk) * (rtb_n_dt2[0] -
    rtDW.DiscreteTimeIntegratory_dt_DSTA[0]) + absxk * rtb_Sum2_e[1]) *
                  rtP.lindi.atc.k.leanacc) + ((1.0F - absxk) * rtb_n_dt_j[0] +
    absxk * rtb_n_b_dt2[1])) - (1.0F - absxk) *
    rtDW.DiscreteTimeIntegratory_dt_DSTA[0];
  rtb_n_dt[1] = (((((1.0F - absxk) * (rtb_n_b_h[1] -
    rtDW.DiscreteTimeIntegratory_DSTAT_k[1]) + absxk * -rtb_n_b_dt[0]) *
                   rtP.lindi.atc.k.leanrate + rtP.lindi.atc.k.lean * -rtb_y_a[0])
                  + ((1.0F - absxk) * (rtb_n_dt2[1] -
    rtDW.DiscreteTimeIntegratory_dt_DSTA[1]) + absxk * -rtb_Sum2_e[0]) *
                  rtP.lindi.atc.k.leanacc) + ((1.0F - absxk) * rtb_n_dt_j[1] +
    absxk * -rtb_n_b_dt2[0])) - (1.0F - absxk) *
    rtDW.DiscreteTimeIntegratory_dt_DSTA[1];
  rtb_n_dt[2] = ((((rtDW.DiscreteTimeIntegrator_DSTATE -
                    rtDW.DiscreteTimeIntegratory_DSTAT_k[2]) *
                   rtP.lindi.atc.k.yawrate + rtP.lindi.atc.k.yaw * t) +
                  (cos_Theta - rtDW.DiscreteTimeIntegratory_dt_DSTA[2]) *
                  rtP.lindi.atc.k.yawacc) + cos_Theta) -
    rtDW.DiscreteTimeIntegratory_dt_DSTA[2];

  // Outputs for Enabled SubSystem: '<S4>/Vertical Acc to Specific Thrust' incorporates:
  //   EnablePort: '<S48>/Enable'

  if (rtb_UnitDelay4_n) {
    // MATLAB Function: '<S48>/MATLAB Function' incorporates:
    //   Constant: '<S48>/Constant'
    //   Constant: '<S4>/Constant2'
    //   DiscreteIntegrator: '<S90>/Discrete-Time Integrator y'

    if (rtDW.DiscreteTimeIntegratory_DSTAT_p[8] < 9.81F /
        rtP.lindi.psc.rm.accumax) {
      // SignalConversion: '<S48>/OutportBufferForT_spec_des'
      rtDW.Merge1_e = 0.0F;
    } else {
      t = rtDW.DiscreteTimeIntegratory_DSTAT_p[8];
      if (std::abs(rtDW.DiscreteTimeIntegratory_DSTAT_p[8]) < 2.22044605E-16F) {
        t = 2.22044605E-16F;
      }

      // SignalConversion: '<S48>/OutportBufferForT_spec_des'
      rtDW.Merge1_e = (-rtb_Sum2_hq[2] + 9.81F) / t;
    }

    // End of MATLAB Function: '<S48>/MATLAB Function'
  }

  // End of Outputs for SubSystem: '<S4>/Vertical Acc to Specific Thrust'

  // Outputs for Enabled SubSystem: '<S4>/Incremental specific thrust' incorporates:
  //   EnablePort: '<S40>/Enable'

  if (rtb_Compare) {
    // DiscreteIntegrator: '<S86>/Discrete-Time Integrator y'
    if (rtDW.DiscreteTimeIntegratory_IC_LO_e != 0) {
      rtDW.DiscreteTimeIntegratory_DSTAT_c[0] = rtDW.n_g_des[0];
      rtDW.DiscreteTimeIntegratory_DSTAT_c[1] = rtDW.n_g_des[1];
      rtDW.DiscreteTimeIntegratory_DSTAT_c[2] = rtDW.n_g_des[2];
    }

    rtb_Sum2_e[0] = rtDW.DiscreteTimeIntegratory_DSTAT_c[0];

    // DiscreteIntegrator: '<S86>/Discrete-Time Integrator y_dt'
    rtb_n_b_dt2[0] = rtDW.DiscreteTimeIntegratory_dt_DS_e[0];

    // Product: '<S86>/Product1' incorporates:
    //   DiscreteIntegrator: '<S86>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S86>/Discrete-Time Integrator y_dt'
    //   Product: '<S86>/Product2'
    //   Sum: '<S86>/Sum2'
    //   Sum: '<S86>/Sum3'

    rtb_Sum2_hq[0] = (rtDW.n_g_des[0] - (rtDW.DiscreteTimeIntegratory_dt_DS_e[0]
      * rtb_y_a0 + rtDW.DiscreteTimeIntegratory_DSTAT_c[0])) * q1_q2;

    // DiscreteIntegrator: '<S86>/Discrete-Time Integrator y'
    rtb_Sum2_e[1] = rtDW.DiscreteTimeIntegratory_DSTAT_c[1];

    // DiscreteIntegrator: '<S86>/Discrete-Time Integrator y_dt'
    rtb_n_b_dt2[1] = rtDW.DiscreteTimeIntegratory_dt_DS_e[1];

    // Product: '<S86>/Product1' incorporates:
    //   DiscreteIntegrator: '<S86>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S86>/Discrete-Time Integrator y_dt'
    //   Product: '<S86>/Product2'
    //   Sum: '<S86>/Sum2'
    //   Sum: '<S86>/Sum3'

    rtb_Sum2_hq[1] = (rtDW.n_g_des[1] - (rtDW.DiscreteTimeIntegratory_dt_DS_e[1]
      * rtb_y_a0 + rtDW.DiscreteTimeIntegratory_DSTAT_c[1])) * q1_q2;

    // DiscreteIntegrator: '<S86>/Discrete-Time Integrator y'
    rtb_Sum2_e[2] = rtDW.DiscreteTimeIntegratory_DSTAT_c[2];

    // DiscreteIntegrator: '<S86>/Discrete-Time Integrator y_dt'
    rtb_n_b_dt2[2] = rtDW.DiscreteTimeIntegratory_dt_DS_e[2];

    // Product: '<S86>/Product1' incorporates:
    //   DiscreteIntegrator: '<S86>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S86>/Discrete-Time Integrator y_dt'
    //   Product: '<S86>/Product2'
    //   Sum: '<S86>/Sum2'
    //   Sum: '<S86>/Sum3'

    rtb_Sum2_hq[2] = (rtDW.n_g_des[2] - (rtDW.DiscreteTimeIntegratory_dt_DS_e[2]
      * rtb_y_a0 + rtDW.DiscreteTimeIntegratory_DSTAT_c[2])) * q1_q2;

    // MATLAB Function: '<S83>/n ref norm'
    nrefnorm(rtb_Sum2_e, rtb_n_b_dt2, rtb_Sum2_hq, rtb_p_ahead_l, rtb_n,
             rtb_n_dt_j);

    // MATLAB Function: '<S40>/DCM 2 Lean Vector'
    DCM2LeanVector(rtb_Sum2_jx, rtb_Sum2_e);

    // MATLAB Function: '<S40>/desired and measured specific thrust' incorporates:
    //   DiscreteIntegrator: '<S89>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S90>/Discrete-Time Integrator y'

    for (wp_idx_app_2 = 0; wp_idx_app_2 < 3; wp_idx_app_2++) {
      q1_q3 = rtDW.DiscreteTimeIntegratory_DSTAT_p[wp_idx_app_2 + 6];
      rtb_n_b_h[wp_idx_app_2] = q1_q3 * rtDW.DiscreteTimeIntegratory_DSTA_al[2]
        + (rtDW.DiscreteTimeIntegratory_DSTAT_p[wp_idx_app_2 + 3] *
           rtDW.DiscreteTimeIntegratory_DSTA_al[1] +
           rtDW.DiscreteTimeIntegratory_DSTAT_p[wp_idx_app_2] *
           rtDW.DiscreteTimeIntegratory_DSTA_al[0]);
      rtb_n[wp_idx_app_2] = q1_q3 * 9.81F;
    }

    rtDW.a_T_meas = rtb_n_b_h[2] - rtb_n[2];

    // End of MATLAB Function: '<S40>/desired and measured specific thrust'

    // Sum: '<S40>/Add' incorporates:
    //   Abs: '<S40>/Abs'
    //   DotProduct: '<S40>/Dot Product'
    //   Gain: '<S40>/Gain'
    //   Product: '<S40>/Product'

    rtDW.Delta_nu_a_T = std::abs((rtDW.n_g_des[0] * rtb_p_ahead_l[0] +
      rtDW.n_g_des[1] * rtb_p_ahead_l[1]) + rtDW.n_g_des[2] * rtb_p_ahead_l[2]) *
      -rtDW.Merge1_e - rtDW.a_T_meas;

    // Update for DiscreteIntegrator: '<S86>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S86>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTAT_c[0] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_e[0];

    // Update for DiscreteIntegrator: '<S86>/Discrete-Time Integrator y_dt'
    rtDW.DiscreteTimeIntegratory_dt_DS_e[0] += 0.0025F * rtb_Sum2_hq[0];

    // Update for DiscreteIntegrator: '<S86>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S86>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTAT_c[1] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_e[1];

    // Update for DiscreteIntegrator: '<S86>/Discrete-Time Integrator y_dt'
    rtDW.DiscreteTimeIntegratory_dt_DS_e[1] += 0.0025F * rtb_Sum2_hq[1];

    // MATLAB Function: '<S40>/incremental thrust atti correction'
    t = (rtb_Sum2_e[0] * rtb_p_ahead_l[0] + rtb_Sum2_e[1] * rtb_p_ahead_l[1]) +
      rtb_Sum2_e[2] * rtb_p_ahead_l[2];

    // Update for DiscreteIntegrator: '<S86>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S86>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTAT_c[2] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_e[2];

    // Update for DiscreteIntegrator: '<S86>/Discrete-Time Integrator y_dt'
    rtDW.DiscreteTimeIntegratory_dt_DS_e[2] += 0.0025F * rtb_Sum2_hq[2];

    // MATLAB Function: '<S40>/incremental thrust atti correction'
    if (t < 0.0F) {
      rtDW.Delta_nu_a_T = 0.0F;
    } else {
      rtDW.Delta_nu_a_T *= t;
    }

    // Update for DiscreteIntegrator: '<S86>/Discrete-Time Integrator y'
    rtDW.DiscreteTimeIntegratory_IC_LO_e = 0U;
  }

  // End of Outputs for SubSystem: '<S4>/Incremental specific thrust'

  // MinMax: '<S75>/Max' incorporates:
  //   Constant: '<S75>/Constant1'
  //   DiscreteIntegrator: '<S80>/Discrete-Time Integrator'

  if (rtDW.DiscreteTimeIntegrator_DSTATE_e[0] > rtP.lindi.ca.u_min) {
    rtb_Add_a[0] = rtDW.DiscreteTimeIntegrator_DSTATE_e[0];
  } else {
    rtb_Add_a[0] = rtP.lindi.ca.u_min;
  }

  if (rtDW.DiscreteTimeIntegrator_DSTATE_e[1] > rtP.lindi.ca.u_min) {
    rtb_Add_a[1] = rtDW.DiscreteTimeIntegrator_DSTATE_e[1];
  } else {
    rtb_Add_a[1] = rtP.lindi.ca.u_min;
  }

  if (rtDW.DiscreteTimeIntegrator_DSTATE_e[2] > rtP.lindi.ca.u_min) {
    rtb_Add_a[2] = rtDW.DiscreteTimeIntegrator_DSTATE_e[2];
  } else {
    rtb_Add_a[2] = rtP.lindi.ca.u_min;
  }

  if (rtDW.DiscreteTimeIntegrator_DSTATE_e[3] > rtP.lindi.ca.u_min) {
    rtb_Add_a[3] = rtDW.DiscreteTimeIntegrator_DSTATE_e[3];
  } else {
    rtb_Add_a[3] = rtP.lindi.ca.u_min;
  }

  // End of MinMax: '<S75>/Max'

  // MATLAB Function: '<S75>/MATLAB Function'
  if (rtP.lindi.cep.d != 0.0F) {
    t = 4.0F * rtP.lindi.cep.d * rtP.lindi.cep.kt * rtP.lindi.cep.ri *
      rtP.lindi.cep.vb;
    rtb_y_a0 = std::pow(rtP.lindi.cep.kt, 4.0F);
    absxk = rtP.lindi.cep.kt * rtP.lindi.cep.kt;
    q0_q2 = 2.0F * rtP.lindi.cep.d * rtP.lindi.cep.ri;
    rtb_q_red_l[0] = (std::sqrt(t * rtb_Add_a[0] + rtb_y_a0) - absxk) / q0_q2;
    rtb_q_red_l[1] = (std::sqrt(t * rtb_Add_a[1] + rtb_y_a0) - absxk) / q0_q2;
    rtb_q_red_l[2] = (std::sqrt(t * rtb_Add_a[2] + rtb_y_a0) - absxk) / q0_q2;
    rtb_q_red_l[3] = (std::sqrt(t * rtb_Add_a[3] + rtb_y_a0) - absxk) / q0_q2;
    q1_q3 = std::sqrt(t * rtb_Add_a[0] + rtb_y_a0);
    rtb_Add_a[0] = std::sqrt(t * rtb_Add_a[0] + rtb_y_a0);
    rtb_Add_a[1] = std::sqrt(t * rtb_Add_a[1] + rtb_y_a0);
    rtb_Add_a[2] = std::sqrt(t * rtb_Add_a[2] + rtb_y_a0);
    absxk = std::sqrt(t * rtb_Add_a[3] + rtb_y_a0);
    rtb_Add_a[3] = absxk;
    t = rtP.lindi.cep.kt * rtP.lindi.cep.vb;
    rtb_Compare = (std::abs(q1_q3) < 2.22044605E-16F);
    rtb_UnitDelay4_n = (std::abs(rtb_Add_a[1]) < 2.22044605E-16F);
    rtb_UnitDelay1_j = (std::abs(rtb_Add_a[2]) < 2.22044605E-16F);
    rtb_is_slowing = (std::abs(absxk) < 2.22044605E-16F);
    flight_mode = 0;
    if (rtb_Compare) {
      flight_mode = 1;
    }

    if (rtb_UnitDelay4_n) {
      flight_mode++;
    }

    if (rtb_UnitDelay1_j) {
      flight_mode++;
    }

    if (rtb_is_slowing) {
      flight_mode++;
    }

    b_stage_ = flight_mode;
    flight_mode = 0;
    if (rtb_Compare) {
      b_data[0] = 1;
      flight_mode = 1;
    }

    if (rtb_UnitDelay4_n) {
      b_data[flight_mode] = 2;
      flight_mode++;
    }

    if (rtb_UnitDelay1_j) {
      b_data[flight_mode] = 3;
      flight_mode++;
    }

    if (rtb_is_slowing) {
      b_data[flight_mode] = 4;
    }

    for (wp_idx_app_2 = 0; wp_idx_app_2 < b_stage_; wp_idx_app_2++) {
      rtb_Add_a[b_data[wp_idx_app_2] - 1] = 2.22044605E-16F;
    }

    rtb_Add_a[0] = t / rtb_Add_a[0];
    rtb_Add_a[1] = t / rtb_Add_a[1];
    rtb_Add_a[2] = t / rtb_Add_a[2];
    rtb_Add_a[3] = t / rtb_Add_a[3];
  } else {
    t = rtP.lindi.cep.vb / rtP.lindi.cep.kt;
    rtb_q_red_l[0] = t * rtb_Add_a[0];
    rtb_Add_a[0] = t;
    rtb_q_red_l[1] = t * rtb_Add_a[1];
    rtb_Add_a[1] = t;
    rtb_q_red_l[2] = t * rtb_Add_a[2];
    rtb_Add_a[2] = t;
    rtb_q_red_l[3] = t * rtb_Add_a[3];
    rtb_Add_a[3] = t;
  }

  memset(&G_omega[0], 0, sizeof(real32_T) << 4U);
  memset(&rtb_G2[0], 0, sizeof(real32_T) << 4U);
  G_omega[0] = rtb_Add_a[0];
  G_omega[5] = rtb_Add_a[1];
  G_omega[10] = rtb_Add_a[2];
  G_omega[15] = rtb_Add_a[3];
  rtb_G2[0] = rtb_q_red_l[0];
  rtb_G2[5] = rtb_q_red_l[1];
  rtb_G2[10] = rtb_q_red_l[2];
  rtb_G2[15] = rtb_q_red_l[3];
  for (wp_idx_app_2 = 0; wp_idx_app_2 < 4; wp_idx_app_2++) {
    for (flight_mode = 0; flight_mode < 4; flight_mode++) {
      b_stage_ = flight_mode << 2;
      c_stage_app = wp_idx_app_2 + b_stage_;
      rtb_y_bn[c_stage_app] = 0.0F;
      k = b_stage_ + wp_idx_app_2;
      rtb_y_bn[c_stage_app] = rtb_y_bn[k] + rtb_G2[b_stage_] *
        rtConstP.MATLABFunction_G10[wp_idx_app_2];
      rtb_y_bn[c_stage_app] = rtb_G2[b_stage_ + 1] *
        rtConstP.MATLABFunction_G10[wp_idx_app_2 + 4] + rtb_y_bn[k];
      rtb_y_bn[c_stage_app] = rtb_G2[b_stage_ + 2] *
        rtConstP.MATLABFunction_G10[wp_idx_app_2 + 8] + rtb_y_bn[k];
      rtb_y_bn[c_stage_app] = rtb_G2[b_stage_ + 3] *
        rtConstP.MATLABFunction_G10[wp_idx_app_2 + 12] + rtb_y_bn[k];
    }

    for (flight_mode = 0; flight_mode < 4; flight_mode++) {
      b_stage_ = flight_mode << 2;
      c_stage_app = wp_idx_app_2 + b_stage_;
      rtb_G1[c_stage_app] = 0.0F;
      k = b_stage_ + wp_idx_app_2;
      rtb_G1[c_stage_app] = rtb_G1[k] + G_omega[b_stage_] *
        rtb_y_bn[wp_idx_app_2];
      rtb_G1[c_stage_app] = G_omega[b_stage_ + 1] * rtb_y_bn[wp_idx_app_2 + 4] +
        rtb_G1[k];
      rtb_G1[c_stage_app] = G_omega[b_stage_ + 2] * rtb_y_bn[wp_idx_app_2 + 8] +
        rtb_G1[k];
      rtb_G1[c_stage_app] = G_omega[b_stage_ + 3] * rtb_y_bn[wp_idx_app_2 + 12]
        + rtb_G1[k];
    }
  }

  for (wp_idx_app_2 = 0; wp_idx_app_2 < 4; wp_idx_app_2++) {
    for (flight_mode = 0; flight_mode < 4; flight_mode++) {
      b_stage_ = wp_idx_app_2 << 2;
      c_stage_app = flight_mode + b_stage_;
      rtb_G2[c_stage_app] = 0.0F;
      k = b_stage_ + flight_mode;
      rtb_G2[c_stage_app] = rtb_G2[k] + G_omega[b_stage_] *
        rtConstP.MATLABFunction_G20[flight_mode];
      rtb_G2[c_stage_app] = G_omega[b_stage_ + 1] *
        rtConstP.MATLABFunction_G20[flight_mode + 4] + rtb_G2[k];
      rtb_G2[c_stage_app] = G_omega[b_stage_ + 2] *
        rtConstP.MATLABFunction_G20[flight_mode + 8] + rtb_G2[k];
      rtb_G2[c_stage_app] = G_omega[b_stage_ + 3] *
        rtConstP.MATLABFunction_G20[flight_mode + 12] + rtb_G2[k];
    }
  }

  // End of MATLAB Function: '<S75>/MATLAB Function'

  // MATLAB Function: '<S52>/create diag' incorporates:
  //   Delay: '<S56>/Delay'

  memset(&rtb_y_bn[0], 0, sizeof(real32_T) << 4U);
  rtb_y_bn[0] = rtDW.Delay_DSTATE[0];
  rtb_y_bn[5] = rtDW.Delay_DSTATE[1];
  rtb_y_bn[10] = rtDW.Delay_DSTATE[2];
  rtb_y_bn[15] = rtDW.Delay_DSTATE[3];

  // Product: '<S52>/correct G1'
  for (wp_idx_app_2 = 0; wp_idx_app_2 < 4; wp_idx_app_2++) {
    for (flight_mode = 0; flight_mode < 4; flight_mode++) {
      b_stage_ = wp_idx_app_2 << 2;
      c_stage_app = flight_mode + b_stage_;
      G_omega[c_stage_app] = 0.0F;
      k = b_stage_ + flight_mode;
      G_omega[c_stage_app] = G_omega[k] + rtb_G1[b_stage_] *
        rtb_y_bn[flight_mode];
      G_omega[c_stage_app] = rtb_G1[b_stage_ + 1] * rtb_y_bn[flight_mode + 4] +
        G_omega[k];
      G_omega[c_stage_app] = rtb_G1[b_stage_ + 2] * rtb_y_bn[flight_mode + 8] +
        G_omega[k];
      G_omega[c_stage_app] = rtb_G1[b_stage_ + 3] * rtb_y_bn[flight_mode + 12] +
        G_omega[k];
    }
  }

  // End of Product: '<S52>/correct G1'

  // Product: '<S52>/correct G2' incorporates:
  //   Delay: '<S57>/Delay1'

  for (wp_idx_app_2 = 0; wp_idx_app_2 < 16; wp_idx_app_2++) {
    rtb_y_bn[wp_idx_app_2] = rtDW.Delay1_DSTATE * rtb_G2[wp_idx_app_2];
  }

  // End of Product: '<S52>/correct G2'

  // MATLAB Function: '<S47>/throttle_-1_1 to throttle_0_1' incorporates:
  //   Gain: '<Root>/Gain5'

  q0_q2 = 0.5F * -rtDW.Merge[3] + 0.5F;
  if (1.0F <= q0_q2) {
    q0_q2 = 1.0F;
  }

  // MATLAB Function: '<S72>/Set Desired Motor Command'
  Delta_u_d = 0.0;
  if (rtb_is_descent) {
    // MATLAB Function: '<S47>/throttle_-1_1 to throttle_0_1'
    if (0.0F >= q0_q2) {
      q0_q2 = 0.0F;
    }

    q1_q3 = (rtP.lindi.thr.max - rtP.lindi.thr.min) * q0_q2 + rtP.lindi.thr.min;
    if (rtP.lindi.ca.u_max < q1_q3) {
      q1_q3 = rtP.lindi.ca.u_max;
    }

    if (rtP.lindi.ca.u_min > q1_q3) {
      q1_q3 = rtP.lindi.ca.u_min;
    }

    Delta_u_d = q1_q3 - rtP.lindi.ca.u_d;
  }

  // MATLAB Function: '<S41>/MATLAB Function'
  DCM2LeanVector(rtb_Sum2_jx, rtb_p_ahead_l);

  // MATLAB Function: '<S39>/Control Allocation Vertical Acc Weighting'
  rtb_Delta_diag_W_v[0] = 0.0F;
  rtb_Delta_diag_W_v[1] = 0.0F;
  if (rtP.lindi.ca.W_v[0] > rtP.lindi.ca.W_v[1]) {
    rtb_y_a0 = rtP.lindi.ca.W_v[1];
  } else {
    rtb_y_a0 = rtP.lindi.ca.W_v[0];
  }

  t = (rtb_p_ahead_l[0] * rtDW.n_g_des[0] + rtb_p_ahead_l[1] * rtDW.n_g_des[1])
    + rtb_p_ahead_l[2] * rtDW.n_g_des[2];
  if (t < 0.0F) {
    t = 0.0F;
  } else {
    t = (rtP.lindi.ca.W_v[3] - rtb_y_a0) * (t * t) + rtb_y_a0;
  }

  rtb_Delta_diag_W_v[3] = t - rtP.lindi.ca.W_v[3];

  // End of MATLAB Function: '<S39>/Control Allocation Vertical Acc Weighting'

  // MATLAB Function: '<S72>/Set Vertical Acc Weight To Zero'
  if (rtb_is_descent) {
    rtb_Delta_diag_W_v[3] = -rtP.lindi.ca.W_v[3];
    rtb_Delta_diag_W_v[0] = -rtP.lindi.ca.W_v[0] + 1.0F;
    rtb_Delta_diag_W_v[1] = -rtP.lindi.ca.W_v[1] + 1.0F;
  }

  // End of MATLAB Function: '<S72>/Set Vertical Acc Weight To Zero'

  // MATLAB Function: '<S76>/caIndiWls' incorporates:
  //   Constant: '<S76>/Delta u_max'
  //   DiscreteIntegrator: '<S80>/Discrete-Time Integrator'

  t = rtP.lindi.ca.u_min - rtDW.DiscreteTimeIntegrator_DSTATE_e[0];
  absxk = rtP.lindi.ca.u_max - rtDW.DiscreteTimeIntegrator_DSTATE_e[0];
  rtb_Add_a[0] = t + absxk;
  q0_q2 = std::abs(rtP.lindi.ca.u_max - rtP.lindi.ca.u_min);
  if (t > -q0_q2) {
    rtb_q_red_l[0] = t;
  } else {
    rtb_q_red_l[0] = -q0_q2;
  }

  if (absxk < q0_q2) {
    rtb_Sum2_a[0] = absxk;
  } else {
    rtb_Sum2_a[0] = q0_q2;
  }

  t = rtP.lindi.ca.u_min - rtDW.DiscreteTimeIntegrator_DSTATE_e[1];
  absxk = rtP.lindi.ca.u_max - rtDW.DiscreteTimeIntegrator_DSTATE_e[1];
  rtb_Add_a[1] = t + absxk;
  q1_q3 = -std::abs(rtP.lindi.ca.u_max - rtP.lindi.ca.u_min);
  if (t > q1_q3) {
    rtb_q_red_l[1] = t;
  } else {
    rtb_q_red_l[1] = q1_q3;
  }

  if (absxk < q0_q2) {
    rtb_Sum2_a[1] = absxk;
  } else {
    rtb_Sum2_a[1] = q0_q2;
  }

  t = rtP.lindi.ca.u_min - rtDW.DiscreteTimeIntegrator_DSTATE_e[2];
  absxk = rtP.lindi.ca.u_max - rtDW.DiscreteTimeIntegrator_DSTATE_e[2];
  rtb_Add_a[2] = t + absxk;
  q1_q3 = -std::abs(rtP.lindi.ca.u_max - rtP.lindi.ca.u_min);
  if (t > q1_q3) {
    rtb_q_red_l[2] = t;
  } else {
    rtb_q_red_l[2] = q1_q3;
  }

  if (absxk < q0_q2) {
    rtb_Sum2_a[2] = absxk;
  } else {
    rtb_Sum2_a[2] = q0_q2;
  }

  t = rtP.lindi.ca.u_min - rtDW.DiscreteTimeIntegrator_DSTATE_e[3];
  absxk = rtP.lindi.ca.u_max - rtDW.DiscreteTimeIntegrator_DSTATE_e[3];
  rtb_Add_a[3] = t + absxk;
  q1_q3 = -std::abs(rtP.lindi.ca.u_max - rtP.lindi.ca.u_min);
  if (t > q1_q3) {
    rtb_q_red_l[3] = t;
  } else {
    rtb_q_red_l[3] = q1_q3;
  }

  if (absxk < q0_q2) {
    rtb_Sum2_a[3] = absxk;
  } else {
    rtb_Sum2_a[3] = q0_q2;
  }

  memset(&W_v[0], 0, sizeof(real32_T) << 4U);
  memset(&W_u[0], 0, sizeof(real32_T) << 4U);
  W_v[0] = rtP.lindi.ca.W_v[0] + rtb_Delta_diag_W_v[0];
  W_v[5] = rtP.lindi.ca.W_v[1] + rtb_Delta_diag_W_v[1];
  W_v[10] = rtP.lindi.ca.W_v[2];
  W_v[15] = rtP.lindi.ca.W_v[3] + rtb_Delta_diag_W_v[3];
  W_u[0] = rtP.lindi.ca.W_u[0];
  rtb_Delta_diag_W_v[0] = 0.0F;
  rtb_Add_a[0] *= 0.5F;
  W_u[5] = rtP.lindi.ca.W_u[1];
  rtb_Delta_diag_W_v[1] = 0.0F;
  rtb_Add_a[1] *= 0.5F;
  W_u[10] = rtP.lindi.ca.W_u[2];
  rtb_Delta_diag_W_v[2] = 0.0F;
  rtb_Add_a[2] *= 0.5F;
  W_u[15] = rtP.lindi.ca.W_u[3];
  rtb_Delta_diag_W_v[3] = 0.0F;
  rtb_Add_a[3] *= 0.5F;

  // Sum: '<S72>/Add1' incorporates:
  //   Switch: '<S39>/Switch'
  //   Switch: '<S39>/Switch1'

  for (wp_idx_app_2 = 0; wp_idx_app_2 < 16; wp_idx_app_2++) {
    G_omega_0[wp_idx_app_2] = G_omega[wp_idx_app_2] + rtb_y_bn[wp_idx_app_2];
  }

  // End of Sum: '<S72>/Add1'

  // Product: '<S72>/MatrixMultiply2' incorporates:
  //   Switch: '<S39>/Switch1'
  //   UnitDelay: '<S72>/Unit Delay1'

  for (wp_idx_app_2 = 0; wp_idx_app_2 < 4; wp_idx_app_2++) {
    q1_q3 = rtb_y_bn[wp_idx_app_2 + 12] * rtDW.UnitDelay1_DSTATE_c[3] +
      (rtb_y_bn[wp_idx_app_2 + 8] * rtDW.UnitDelay1_DSTATE_c[2] +
       (rtb_y_bn[wp_idx_app_2 + 4] * rtDW.UnitDelay1_DSTATE_c[1] +
        rtb_y_bn[wp_idx_app_2] * rtDW.UnitDelay1_DSTATE_c[0]));
    umax[wp_idx_app_2] = q1_q3;
  }

  // End of Product: '<S72>/MatrixMultiply2'

  // Sum: '<S72>/Add2'
  umin[0] = rtb_n_dt[0] + umax[0];

  // MATLAB Function: '<S76>/caIndiWls' incorporates:
  //   DiscreteIntegrator: '<S80>/Discrete-Time Integrator'
  //   MATLAB Function: '<S72>/Set Desired Motor Command'

  tmp[0] = (rtP.lindi.ca.u_d - rtDW.DiscreteTimeIntegrator_DSTATE_e[0]) +
    (real32_T)Delta_u_d;

  // Sum: '<S72>/Add2'
  umin[1] = rtb_n_dt[1] + umax[1];

  // MATLAB Function: '<S76>/caIndiWls' incorporates:
  //   DiscreteIntegrator: '<S80>/Discrete-Time Integrator'
  //   MATLAB Function: '<S72>/Set Desired Motor Command'

  tmp[1] = (rtP.lindi.ca.u_d - rtDW.DiscreteTimeIntegrator_DSTATE_e[1]) +
    (real32_T)Delta_u_d;

  // Sum: '<S72>/Add2'
  umin[2] = rtb_n_dt[2] + umax[2];

  // MATLAB Function: '<S76>/caIndiWls' incorporates:
  //   DiscreteIntegrator: '<S80>/Discrete-Time Integrator'
  //   MATLAB Function: '<S72>/Set Desired Motor Command'

  tmp[2] = (rtP.lindi.ca.u_d - rtDW.DiscreteTimeIntegrator_DSTATE_e[2]) +
    (real32_T)Delta_u_d;

  // Sum: '<S72>/Add2'
  umin[3] = rtDW.Delta_nu_a_T + umax[3];

  // MATLAB Function: '<S76>/caIndiWls' incorporates:
  //   DiscreteIntegrator: '<S80>/Discrete-Time Integrator'
  //   MATLAB Function: '<S72>/Set Desired Motor Command'

  tmp[3] = (rtP.lindi.ca.u_d - rtDW.DiscreteTimeIntegrator_DSTATE_e[3]) +
    (real32_T)Delta_u_d;
  wls_alloc(G_omega_0, umin, rtb_q_red_l, rtb_Sum2_a, W_v, W_u, tmp,
            rtP.lindi.ca.gamma, rtb_Add_a, rtb_Delta_diag_W_v,
            rtP.lindi.ca.i_max);

  // Sum: '<S39>/Add6' incorporates:
  //   DiscreteIntegrator: '<S74>/Discrete-Time Integrator y'

  t = rtb_Add_a[0] + rtDW.DiscreteTimeIntegratory_DSTAT_n[0];

  // Saturate: '<S39>/Saturation3'
  if (t > rtP.lindi.ca.u_max) {
    t = rtP.lindi.ca.u_max;
  } else {
    if (t < rtP.lindi.ca.u_min) {
      t = rtP.lindi.ca.u_min;
    }
  }

  // Outport: '<Root>/logs'
  rtY.logs[0] = t;

  // Saturate: '<S39>/Saturation3'
  rtb_q_red_l[0] = t;

  // Sum: '<S39>/Add6' incorporates:
  //   DiscreteIntegrator: '<S74>/Discrete-Time Integrator y'

  t = rtb_Add_a[1] + rtDW.DiscreteTimeIntegratory_DSTAT_n[1];

  // Saturate: '<S39>/Saturation3'
  if (t > rtP.lindi.ca.u_max) {
    t = rtP.lindi.ca.u_max;
  } else {
    if (t < rtP.lindi.ca.u_min) {
      t = rtP.lindi.ca.u_min;
    }
  }

  // Outport: '<Root>/logs'
  rtY.logs[1] = t;

  // Saturate: '<S39>/Saturation3'
  rtb_q_red_l[1] = t;

  // Sum: '<S39>/Add6' incorporates:
  //   DiscreteIntegrator: '<S74>/Discrete-Time Integrator y'

  t = rtb_Add_a[2] + rtDW.DiscreteTimeIntegratory_DSTAT_n[2];

  // Saturate: '<S39>/Saturation3'
  if (t > rtP.lindi.ca.u_max) {
    t = rtP.lindi.ca.u_max;
  } else {
    if (t < rtP.lindi.ca.u_min) {
      t = rtP.lindi.ca.u_min;
    }
  }

  // Outport: '<Root>/logs'
  rtY.logs[2] = t;

  // Saturate: '<S39>/Saturation3'
  rtb_q_red_l[2] = t;

  // Sum: '<S39>/Add6' incorporates:
  //   DiscreteIntegrator: '<S74>/Discrete-Time Integrator y'

  t = rtb_Add_a[3] + rtDW.DiscreteTimeIntegratory_DSTAT_n[3];

  // Saturate: '<S39>/Saturation3'
  if (t > rtP.lindi.ca.u_max) {
    t = rtP.lindi.ca.u_max;
  } else {
    if (t < rtP.lindi.ca.u_min) {
      t = rtP.lindi.ca.u_min;
    }
  }

  // Outport: '<Root>/logs' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion2'
  //   Delay: '<S56>/Delay'
  //   Delay: '<S57>/Delay1'
  //   DiscreteIntegrator: '<S93>/Discrete-Time Integrator y_dt'
  //   Inport: '<Root>/cmd'
  //   Inport: '<Root>/measure'
  //   MATLAB Function: '<S8>/Auxiliary function to define log_config in generated C++ code'
  //   SignalConversion: '<Root>/BusConversion_InsertedFor_cmd_at_outport_0'
  //   SignalConversion: '<S164>/TmpSignal ConversionAt SFunction Inport2'

  rtY.logs[3] = t;
  for (wp_idx_app_2 = 0; wp_idx_app_2 < 9; wp_idx_app_2++) {
    rtY.logs[wp_idx_app_2 + 4] = rtDW.Add[wp_idx_app_2];
  }

  rtY.logs[13] = rtb_y_a[0];
  rtY.logs[14] = rtb_y_a[1];
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
  rtY.logs[49] = rtDW.Delay1_DSTATE;
  rtY.logs[50] = rtU.cmd.mission_change;
  rtY.logs[51] = rtDW.Merge1;
  rtY.logs[45] = rtDW.Delay_DSTATE[0];
  rtY.logs[52] = rtDW.Merge[0];
  rtY.logs[46] = rtDW.Delay_DSTATE[1];
  rtY.logs[53] = rtDW.Merge[1];
  rtY.logs[47] = rtDW.Delay_DSTATE[2];
  rtY.logs[54] = rtDW.Merge[2];
  rtY.logs[48] = rtDW.Delay_DSTATE[3];
  rtY.logs[55] = rtDW.Merge[3];

  // Outputs for Enabled SubSystem: '<S52>/INDI Inversion Check' incorporates:
  //   EnablePort: '<S55>/Enable'

  if (rtb_is_intercept_arc) {
    // DiscreteIntegrator: '<S70>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S74>/Discrete-Time Integrator y'

    if (rtDW.DiscreteTimeIntegratory_IC_LO_f != 0) {
      rtDW.DiscreteTimeIntegratory_DSTA_lm[0] =
        rtDW.DiscreteTimeIntegratory_DSTAT_n[0];
      rtDW.DiscreteTimeIntegratory_DSTA_lm[1] =
        rtDW.DiscreteTimeIntegratory_DSTAT_n[1];
      rtDW.DiscreteTimeIntegratory_DSTA_lm[2] =
        rtDW.DiscreteTimeIntegratory_DSTAT_n[2];
      rtDW.DiscreteTimeIntegratory_DSTA_lm[3] =
        rtDW.DiscreteTimeIntegratory_DSTAT_n[3];
    }

    // Sum: '<S70>/Sum2' incorporates:
    //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y_dt'
    //   DiscreteIntegrator: '<S74>/Discrete-Time Integrator y'
    //   Product: '<S70>/Product2'
    //   Sum: '<S70>/Sum3'

    rtb_Sum2_a[0] = rtDW.DiscreteTimeIntegratory_DSTAT_n[0] -
      (rtDW.DiscreteTimeIntegratory_dt_DS_p[0] * scale_tmp +
       rtDW.DiscreteTimeIntegratory_DSTA_lm[0]);
    rtb_Sum2_a[1] = rtDW.DiscreteTimeIntegratory_DSTAT_n[1] -
      (rtDW.DiscreteTimeIntegratory_dt_DS_p[1] * scale_tmp +
       rtDW.DiscreteTimeIntegratory_DSTA_lm[1]);
    rtb_Sum2_a[2] = rtDW.DiscreteTimeIntegratory_DSTAT_n[2] -
      (rtDW.DiscreteTimeIntegratory_dt_DS_p[2] * scale_tmp +
       rtDW.DiscreteTimeIntegratory_DSTA_lm[2]);
    rtb_Sum2_a[3] = rtDW.DiscreteTimeIntegratory_DSTAT_n[3] -
      (rtDW.DiscreteTimeIntegratory_dt_DS_p[3] * scale_tmp +
       rtDW.DiscreteTimeIntegratory_DSTA_lm[3]);

    // Gain: '<S66>/Gain' incorporates:
    //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y_dt'
    //   DiscreteIntegrator: '<S74>/Discrete-Time Integrator y_dt'
    //   Sum: '<S68>/Add1'

    q1_q3 = (rtDW.DiscreteTimeIntegratory_dt_DS_l[0] -
             rtDW.DiscreteTimeIntegratory_dt_DS_p[0]) * 0.0025F;
    absxk = (rtDW.DiscreteTimeIntegratory_dt_DS_l[1] -
             rtDW.DiscreteTimeIntegratory_dt_DS_p[1]) * 0.0025F;
    q0_q2 = (rtDW.DiscreteTimeIntegratory_dt_DS_l[2] -
             rtDW.DiscreteTimeIntegratory_dt_DS_p[2]) * 0.0025F;
    rtb_y_a0 = (rtDW.DiscreteTimeIntegratory_dt_DS_l[3] -
                rtDW.DiscreteTimeIntegratory_dt_DS_p[3]) * 0.0025F;
    for (wp_idx_app_2 = 0; wp_idx_app_2 < 4; wp_idx_app_2++) {
      // Product: '<S66>/Matrix Multiply3'
      rtDW.MatrixMultiply3[wp_idx_app_2] = 0.0F;
      rtDW.MatrixMultiply3[wp_idx_app_2] += rtb_y_bn[wp_idx_app_2] * q1_q3;
      rtDW.MatrixMultiply3[wp_idx_app_2] += rtb_y_bn[wp_idx_app_2 + 4] * absxk;
      rtDW.MatrixMultiply3[wp_idx_app_2] += rtb_y_bn[wp_idx_app_2 + 8] * q0_q2;
      rtDW.MatrixMultiply3[wp_idx_app_2] += rtb_y_bn[wp_idx_app_2 + 12] *
        rtb_y_a0;

      // Sum: '<S68>/Add3' incorporates:
      //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S74>/Discrete-Time Integrator y'

      rtDW.Add3[wp_idx_app_2] =
        rtDW.DiscreteTimeIntegratory_DSTAT_n[wp_idx_app_2] -
        rtDW.DiscreteTimeIntegratory_DSTA_lm[wp_idx_app_2];
    }

    // Product: '<S66>/Matrix Multiply2'
    for (wp_idx_app_2 = 0; wp_idx_app_2 < 4; wp_idx_app_2++) {
      rtDW.MatrixMultiply2[wp_idx_app_2] = 0.0F;
      rtDW.MatrixMultiply2[wp_idx_app_2] += G_omega[wp_idx_app_2] * rtDW.Add3[0];
      rtDW.MatrixMultiply2[wp_idx_app_2] += G_omega[wp_idx_app_2 + 4] *
        rtDW.Add3[1];
      rtDW.MatrixMultiply2[wp_idx_app_2] += G_omega[wp_idx_app_2 + 8] *
        rtDW.Add3[2];
      rtDW.MatrixMultiply2[wp_idx_app_2] += G_omega[wp_idx_app_2 + 12] *
        rtDW.Add3[3];
    }

    // End of Product: '<S66>/Matrix Multiply2'

    // DiscreteIntegrator: '<S69>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S93>/Discrete-Time Integrator y_dt'

    if (rtDW.DiscreteTimeIntegratory_IC_L_p0 != 0) {
      rtDW.DiscreteTimeIntegratory_DSTA_ak[0] =
        rtDW.DiscreteTimeIntegratory_dt_DSTA[0];
      rtDW.DiscreteTimeIntegratory_DSTA_ak[1] =
        rtDW.DiscreteTimeIntegratory_dt_DSTA[1];
      rtDW.DiscreteTimeIntegratory_DSTA_ak[2] =
        rtDW.DiscreteTimeIntegratory_dt_DSTA[2];
      rtDW.DiscreteTimeIntegratory_DSTA_ak[3] = rtDW.a_T_meas;
    }

    // Sum: '<S69>/Sum2' incorporates:
    //   DiscreteIntegrator: '<S69>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S69>/Discrete-Time Integrator y_dt'
    //   DiscreteIntegrator: '<S93>/Discrete-Time Integrator y_dt'
    //   Product: '<S69>/Product2'
    //   Sum: '<S69>/Sum3'

    rtb_Delta_diag_W_v[3] = rtDW.a_T_meas -
      (rtDW.DiscreteTimeIntegratory_dt_D_n3[3] * scale_tmp +
       rtDW.DiscreteTimeIntegratory_DSTA_ak[3]);
    rtb_Delta_diag_W_v[0] = rtDW.DiscreteTimeIntegratory_dt_DSTA[0] -
      (rtDW.DiscreteTimeIntegratory_dt_D_n3[0] * scale_tmp +
       rtDW.DiscreteTimeIntegratory_DSTA_ak[0]);

    // Sum: '<S67>/Add2' incorporates:
    //   DiscreteIntegrator: '<S69>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S93>/Discrete-Time Integrator y_dt'

    rtDW.Delta_nu_measure[0] = rtDW.DiscreteTimeIntegratory_dt_DSTA[0] -
      rtDW.DiscreteTimeIntegratory_DSTA_ak[0];

    // Sum: '<S69>/Sum2' incorporates:
    //   DiscreteIntegrator: '<S69>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S69>/Discrete-Time Integrator y_dt'
    //   DiscreteIntegrator: '<S93>/Discrete-Time Integrator y_dt'
    //   Product: '<S69>/Product2'
    //   Sum: '<S69>/Sum3'

    rtb_Delta_diag_W_v[1] = rtDW.DiscreteTimeIntegratory_dt_DSTA[1] -
      (rtDW.DiscreteTimeIntegratory_dt_D_n3[1] * scale_tmp +
       rtDW.DiscreteTimeIntegratory_DSTA_ak[1]);

    // Sum: '<S67>/Add2' incorporates:
    //   DiscreteIntegrator: '<S69>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S93>/Discrete-Time Integrator y_dt'

    rtDW.Delta_nu_measure[1] = rtDW.DiscreteTimeIntegratory_dt_DSTA[1] -
      rtDW.DiscreteTimeIntegratory_DSTA_ak[1];

    // Sum: '<S69>/Sum2' incorporates:
    //   DiscreteIntegrator: '<S69>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S69>/Discrete-Time Integrator y_dt'
    //   DiscreteIntegrator: '<S93>/Discrete-Time Integrator y_dt'
    //   Product: '<S69>/Product2'
    //   Sum: '<S69>/Sum3'

    rtb_Delta_diag_W_v[2] = rtDW.DiscreteTimeIntegratory_dt_DSTA[2] -
      (rtDW.DiscreteTimeIntegratory_dt_D_n3[2] * scale_tmp +
       rtDW.DiscreteTimeIntegratory_DSTA_ak[2]);

    // Sum: '<S67>/Add2' incorporates:
    //   DiscreteIntegrator: '<S69>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S93>/Discrete-Time Integrator y_dt'

    rtDW.Delta_nu_measure[2] = rtDW.DiscreteTimeIntegratory_dt_DSTA[2] -
      rtDW.DiscreteTimeIntegratory_DSTA_ak[2];
    rtDW.Delta_nu_measure[3] = rtDW.a_T_meas -
      rtDW.DiscreteTimeIntegratory_DSTA_ak[3];

    // Update for DiscreteIntegrator: '<S70>/Discrete-Time Integrator y'
    rtDW.DiscreteTimeIntegratory_IC_LO_f = 0U;

    // Update for DiscreteIntegrator: '<S69>/Discrete-Time Integrator y'
    rtDW.DiscreteTimeIntegratory_IC_L_p0 = 0U;

    // Update for DiscreteIntegrator: '<S70>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTA_lm[0] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_p[0];

    // Update for DiscreteIntegrator: '<S70>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S70>/Product1'

    rtDW.DiscreteTimeIntegratory_dt_DS_p[0] += rtb_Sum2_a[0] *
      wp_approach_out_tmp * 0.0025F;

    // Update for DiscreteIntegrator: '<S69>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S69>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTA_ak[0] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_D_n3[0];

    // Update for DiscreteIntegrator: '<S69>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S69>/Product1'

    rtDW.DiscreteTimeIntegratory_dt_D_n3[0] += rtb_Delta_diag_W_v[0] *
      wp_approach_out_tmp * 0.0025F;

    // Update for DiscreteIntegrator: '<S70>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTA_lm[1] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_p[1];

    // Update for DiscreteIntegrator: '<S70>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S70>/Product1'

    rtDW.DiscreteTimeIntegratory_dt_DS_p[1] += rtb_Sum2_a[1] *
      wp_approach_out_tmp * 0.0025F;

    // Update for DiscreteIntegrator: '<S69>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S69>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTA_ak[1] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_D_n3[1];

    // Update for DiscreteIntegrator: '<S69>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S69>/Product1'

    rtDW.DiscreteTimeIntegratory_dt_D_n3[1] += rtb_Delta_diag_W_v[1] *
      wp_approach_out_tmp * 0.0025F;

    // Update for DiscreteIntegrator: '<S70>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTA_lm[2] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_p[2];

    // Update for DiscreteIntegrator: '<S70>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S70>/Product1'

    rtDW.DiscreteTimeIntegratory_dt_DS_p[2] += rtb_Sum2_a[2] *
      wp_approach_out_tmp * 0.0025F;

    // Update for DiscreteIntegrator: '<S69>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S69>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTA_ak[2] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_D_n3[2];

    // Update for DiscreteIntegrator: '<S69>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S69>/Product1'

    rtDW.DiscreteTimeIntegratory_dt_D_n3[2] += rtb_Delta_diag_W_v[2] *
      wp_approach_out_tmp * 0.0025F;

    // Update for DiscreteIntegrator: '<S70>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S70>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTA_lm[3] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_p[3];

    // Update for DiscreteIntegrator: '<S70>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S70>/Product1'

    rtDW.DiscreteTimeIntegratory_dt_DS_p[3] += rtb_Sum2_a[3] *
      wp_approach_out_tmp * 0.0025F;

    // Update for DiscreteIntegrator: '<S69>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S69>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTA_ak[3] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_D_n3[3];

    // Update for DiscreteIntegrator: '<S69>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S69>/Product1'

    rtDW.DiscreteTimeIntegratory_dt_D_n3[3] += rtb_Delta_diag_W_v[3] *
      wp_approach_out_tmp * 0.0025F;
  }

  // End of Outputs for SubSystem: '<S52>/INDI Inversion Check'

  // Gain: '<S90>/Gain' incorporates:
  //   Constant: '<S90>/d'
  //   Constant: '<S90>/omega'
  //   Gain: '<S74>/Gain'
  //   Gain: '<S89>/Gain'
  //   Gain: '<S91>/Gain'
  //   Gain: '<S92>/Gain'
  //   Gain: '<S93>/Gain'
  //   Product: '<S90>/Divide'

  wp_approach_out_tmp = rtP.lindi.sflt.D / rtP.lindi.sflt.omega * 2.0F;

  // Sum: '<S90>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S90>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S90>/Discrete-Time Integrator y_dt'
  //   Gain: '<S90>/Gain'
  //   Product: '<S90>/Product2'
  //   Sum: '<S90>/Sum3'

  for (wp_idx_app_2 = 0; wp_idx_app_2 < 9; wp_idx_app_2++) {
    rtb_M_bg[wp_idx_app_2] -= rtDW.DiscreteTimeIntegratory_dt_D_bx[wp_idx_app_2]
      * wp_approach_out_tmp + rtDW.DiscreteTimeIntegratory_DSTAT_p[wp_idx_app_2];
  }

  // End of Sum: '<S90>/Sum2'

  // Product: '<S90>/omega^2' incorporates:
  //   Constant: '<S90>/omega'
  //   Product: '<S74>/omega^2'
  //   Product: '<S89>/omega^2'
  //   Product: '<S91>/omega^2'
  //   Product: '<S92>/omega^2'
  //   Product: '<S93>/omega^2'

  scale_tmp = rtP.lindi.sflt.omega * rtP.lindi.sflt.omega;

  // Outputs for Enabled SubSystem: '<S52>/Adaptive INDI G1 and G2 correction' incorporates:
  //   EnablePort: '<S53>/Enable'

  if (rtb_is_intercept_arc) {
    if (!rtDW.AdaptiveINDIG1andG2correction_M) {
      rtDW.AdaptiveINDIG1andG2correction_M = true;
    }

    // MATLAB Function: '<S53>/G1 learn rate' incorporates:
    //   Constant: '<S53>/Constant10'
    //   MATLAB Function: '<S53>/G2 learn rate'

    rtb_Delta_diag_W_v[0] = std::abs(rtb_G1[0]);
    rtb_Sum2_a[0] = std::abs(rtb_G1[1]);
    umin[0] = std::abs(rtb_G1[2]);
    umax[0] = std::abs(rtb_G1[3]);
    rtb_Delta_diag_W_v[1] = std::abs(rtb_G1[4]);
    rtb_Sum2_a[1] = std::abs(rtb_G1[5]);
    umin[1] = std::abs(rtb_G1[6]);
    umax[1] = std::abs(rtb_G1[7]);
    rtb_Delta_diag_W_v[2] = std::abs(rtb_G1[8]);
    rtb_Sum2_a[2] = std::abs(rtb_G1[9]);
    umin[2] = std::abs(rtb_G1[10]);
    umax[2] = std::abs(rtb_G1[11]);
    rtb_Delta_diag_W_v[3] = std::abs(rtb_G1[12]);
    rtb_Sum2_a[3] = std::abs(rtb_G1[13]);
    umin[3] = std::abs(rtb_G1[14]);
    umax[3] = std::abs(rtb_G1[15]);
    q1_q2 *= 0.122324161F;
    q1_q3 = 0.02F / (2.0F / rtP.lindi.sflt.omega + rtP.lindi.mtc);
    learn_rate[0] = q1_q3 / (q1_q2 * mean(rtb_Delta_diag_W_v));
    learn_rate[1] = q1_q3 / (q1_q2 * mean(rtb_Sum2_a));
    q1_q2 = rtP.lindi.atc.rm.yawratemax / rtP.lindi.atc.rm.yawratetc;
    learn_rate[2] = 0.0225F / (2.0F / rtP.lindi.sflt.omega + rtP.lindi.mtc) /
      (q1_q2 * mean(umin));
    learn_rate[3] = 0.01F / (2.0F / rtP.lindi.sflt.omega + rtP.lindi.mtc) /
      (1.2F * mean(umax));

    // Sum: '<S62>/Add' incorporates:
    //   UnitDelay: '<S62>/Unit Delay'

    q1_q3 = rtDW.Add3[0] - rtDW.UnitDelay_DSTATE[0];
    absxk = rtDW.Add3[1] - rtDW.UnitDelay_DSTATE[1];
    q0_q2 = rtDW.Add3[2] - rtDW.UnitDelay_DSTATE[2];
    rtb_y_a0 = rtDW.Add3[3] - rtDW.UnitDelay_DSTATE[3];
    for (k = 0; k < 4; k++) {
      // Product: '<S61>/Matrix Multiply5'
      rtb_p_ahead_c_idx_1 = rtb_G1[k + 12] * rtDW.Add3[3] + (rtb_G1[k + 8] *
        rtDW.Add3[2] + (rtb_G1[k + 4] * rtDW.Add3[1] + rtb_G1[k] * rtDW.Add3[0]));

      // Product: '<S53>/apply learn rate' incorporates:
      //   Gain: '<S53>/Gain'
      //   MATLAB Function: '<S53>/G1 learn rate'
      //   Product: '<S53>/Product'
      //   Sum: '<S63>/Add2'
      //   Sum: '<S63>/Add3'

      rtDW.Delta_factors_G1[k] = -((rtDW.MatrixMultiply2[k] -
        (rtDW.Delta_nu_measure[k] - rtDW.MatrixMultiply3[k])) *
        rtb_p_ahead_c_idx_1) * (real32_T)learn_rate[k];

      // MATLAB Function: '<S53>/G2 learn rate'
      rtb_Delta_diag_W_v[k] = std::abs(rtb_G2[(k << 2) + 2]);

      // Product: '<S62>/Matrix Multiply7'
      rtb_p_ahead_c_idx_1 = rtb_G2[k + 12] * rtb_y_a0 + (rtb_G2[k + 8] * q0_q2 +
        (rtb_G2[k + 4] * absxk + rtb_G2[k] * q1_q3));

      // Update for UnitDelay: '<S62>/Unit Delay'
      rtDW.UnitDelay_DSTATE[k] = rtDW.Add3[k];

      // Product: '<S62>/Matrix Multiply7'
      umin[k] = rtb_p_ahead_c_idx_1;
    }

    // Product: '<S53>/apply learn rate2' incorporates:
    //   Constant: '<S53>/Constant10'
    //   Gain: '<S53>/Gain1'
    //   MATLAB Function: '<S53>/G2 learn rate'
    //   Product: '<S53>/Product4'
    //   Sum: '<S63>/Add4'
    //   Sum: '<S63>/Add5'

    rtDW.Delta_factors_G2 = 0.015F / (2.0F / rtP.lindi.sflt.omega +
      rtP.lindi.mtc) / (q1_q2 / (2.0F / rtP.lindi.sflt.omega + rtP.lindi.mtc) *
                        0.0025F * ((((rtb_Delta_diag_W_v[0] +
      rtb_Delta_diag_W_v[1]) + rtb_Delta_diag_W_v[2]) + rtb_Delta_diag_W_v[3]) /
      4.0F)) * -((rtDW.MatrixMultiply3[2] - (rtDW.Delta_nu_measure[2] -
      rtDW.MatrixMultiply2[2])) * umin[2]);
  } else {
    if (rtDW.AdaptiveINDIG1andG2correction_M) {
      // Disable for Outport: '<S53>/Delta_factors_G1'
      rtDW.Delta_factors_G1[0] = 0.0F;
      rtDW.Delta_factors_G1[1] = 0.0F;
      rtDW.Delta_factors_G1[2] = 0.0F;
      rtDW.Delta_factors_G1[3] = 0.0F;

      // Disable for Outport: '<S53>/Delta_factors_G2'
      rtDW.Delta_factors_G2 = 0.0F;
      rtDW.AdaptiveINDIG1andG2correction_M = false;
    }
  }

  // End of Outputs for SubSystem: '<S52>/Adaptive INDI G1 and G2 correction'

  // Sum: '<S74>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S74>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S74>/Discrete-Time Integrator y_dt'
  //   DiscreteIntegrator: '<S80>/Discrete-Time Integrator'
  //   Product: '<S74>/Product2'
  //   Sum: '<S74>/Sum3'

  rtb_Sum2_a[0] = rtDW.DiscreteTimeIntegrator_DSTATE_e[0] -
    (rtDW.DiscreteTimeIntegratory_dt_DS_l[0] * wp_approach_out_tmp +
     rtDW.DiscreteTimeIntegratory_DSTAT_n[0]);
  rtb_Sum2_a[1] = rtDW.DiscreteTimeIntegrator_DSTATE_e[1] -
    (rtDW.DiscreteTimeIntegratory_dt_DS_l[1] * wp_approach_out_tmp +
     rtDW.DiscreteTimeIntegratory_DSTAT_n[1]);
  rtb_Sum2_a[2] = rtDW.DiscreteTimeIntegrator_DSTATE_e[2] -
    (rtDW.DiscreteTimeIntegratory_dt_DS_l[2] * wp_approach_out_tmp +
     rtDW.DiscreteTimeIntegratory_DSTAT_n[2]);
  rtb_Sum2_a[3] = rtDW.DiscreteTimeIntegrator_DSTATE_e[3] -
    (rtDW.DiscreteTimeIntegratory_dt_DS_l[3] * wp_approach_out_tmp +
     rtDW.DiscreteTimeIntegratory_DSTAT_n[3]);

  // Outport: '<Root>/u' incorporates:
  //   Gain: '<Root>/Gain1'
  //   Gain: '<Root>/Gain2'
  //   Gain: '<Root>/Gain3'
  //   Gain: '<Root>/Gain4'

  rtY.u[0] = rtb_q_red_l[1];
  rtY.u[1] = t;
  rtY.u[2] = rtb_q_red_l[0];
  rtY.u[3] = rtb_q_red_l[2];
  rtY.u[4] = 0.0F;
  rtY.u[5] = 0.0F;
  rtY.u[6] = 0.0F;
  rtY.u[7] = 0.0F;

  // Sum: '<S89>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S89>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S89>/Discrete-Time Integrator y_dt'
  //   Inport: '<Root>/measure'
  //   Product: '<S89>/Product2'
  //   Sum: '<Root>/Add'
  //   Sum: '<S89>/Sum3'

  q1_q2 = rtU.measure.a_Kg[0] - (rtDW.DiscreteTimeIntegratory_dt_DS_o[0] *
    wp_approach_out_tmp + rtDW.DiscreteTimeIntegratory_DSTA_al[0]);
  absxk = rtU.measure.a_Kg[1] - (rtDW.DiscreteTimeIntegratory_dt_DS_o[1] *
    wp_approach_out_tmp + rtDW.DiscreteTimeIntegratory_DSTA_al[1]);
  q0_q2 = (rtU.measure.a_Kg[2] + 9.81F) - (rtDW.DiscreteTimeIntegratory_dt_DS_o
    [2] * wp_approach_out_tmp + rtDW.DiscreteTimeIntegratory_DSTA_al[2]);

  // Product: '<S91>/Product1' incorporates:
  //   DiscreteIntegrator: '<S91>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S91>/Discrete-Time Integrator y_dt'
  //   Inport: '<Root>/measure'
  //   Product: '<S91>/Product2'
  //   Sum: '<S91>/Sum2'
  //   Sum: '<S91>/Sum3'

  rtb_p_ahead_l[0] = (rtU.measure.V_Kg[0] -
                      (rtDW.DiscreteTimeIntegratory_dt_DS_n[0] *
                       wp_approach_out_tmp +
                       rtDW.DiscreteTimeIntegratory_DSTAT_a[0])) * scale_tmp;
  rtb_p_ahead_l[1] = (rtU.measure.V_Kg[1] -
                      (rtDW.DiscreteTimeIntegratory_dt_DS_n[1] *
                       wp_approach_out_tmp +
                       rtDW.DiscreteTimeIntegratory_DSTAT_a[1])) * scale_tmp;
  rtb_p_ahead_l[2] = (rtU.measure.V_Kg[2] -
                      (rtDW.DiscreteTimeIntegratory_dt_DS_n[2] *
                       wp_approach_out_tmp +
                       rtDW.DiscreteTimeIntegratory_DSTAT_a[2])) * scale_tmp;

  // Sum: '<S92>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S92>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S92>/Discrete-Time Integrator y_dt'
  //   Inport: '<Root>/measure'
  //   Product: '<S92>/Product2'
  //   Sum: '<S92>/Sum3'

  rtb_Sum2_e[0] = rtU.measure.s_Kg[0] - (rtDW.DiscreteTimeIntegratory_dt_D_nb[0]
    * wp_approach_out_tmp + rtDW.DiscreteTimeIntegratory_DSTATE[0]);
  rtb_Sum2_e[1] = rtU.measure.s_Kg[1] - (rtDW.DiscreteTimeIntegratory_dt_D_nb[1]
    * wp_approach_out_tmp + rtDW.DiscreteTimeIntegratory_DSTATE[1]);
  rtb_Sum2_e[2] = rtU.measure.s_Kg[2] - (rtDW.DiscreteTimeIntegratory_dt_D_nb[2]
    * wp_approach_out_tmp + rtDW.DiscreteTimeIntegratory_DSTATE[2]);

  // Saturate: '<S56>/Saturation1'
  if (rtDW.Delta_factors_G1[0] > 0.1F) {
    q1_q3 = 0.1F;
  } else if (rtDW.Delta_factors_G1[0] < -0.1F) {
    q1_q3 = -0.1F;
  } else {
    q1_q3 = rtDW.Delta_factors_G1[0];
  }

  // Saturate: '<S56>/Saturation2' incorporates:
  //   Delay: '<S56>/Delay'
  //   Saturate: '<S56>/Saturation1'
  //   Sum: '<S56>/Add'

  q1_q3 += rtDW.Delay_DSTATE[0];
  if (q1_q3 > 4.0F) {
    // Update for Delay: '<S56>/Delay'
    rtDW.Delay_DSTATE[0] = 4.0F;
  } else if (q1_q3 < 0.25F) {
    // Update for Delay: '<S56>/Delay'
    rtDW.Delay_DSTATE[0] = 0.25F;
  } else {
    // Update for Delay: '<S56>/Delay'
    rtDW.Delay_DSTATE[0] = q1_q3;
  }

  // Saturate: '<S56>/Saturation1'
  if (rtDW.Delta_factors_G1[1] > 0.1F) {
    q1_q3 = 0.1F;
  } else if (rtDW.Delta_factors_G1[1] < -0.1F) {
    q1_q3 = -0.1F;
  } else {
    q1_q3 = rtDW.Delta_factors_G1[1];
  }

  // Saturate: '<S56>/Saturation2' incorporates:
  //   Delay: '<S56>/Delay'
  //   Saturate: '<S56>/Saturation1'
  //   Sum: '<S56>/Add'

  q1_q3 += rtDW.Delay_DSTATE[1];
  if (q1_q3 > 4.0F) {
    // Update for Delay: '<S56>/Delay'
    rtDW.Delay_DSTATE[1] = 4.0F;
  } else if (q1_q3 < 0.25F) {
    // Update for Delay: '<S56>/Delay'
    rtDW.Delay_DSTATE[1] = 0.25F;
  } else {
    // Update for Delay: '<S56>/Delay'
    rtDW.Delay_DSTATE[1] = q1_q3;
  }

  // Saturate: '<S56>/Saturation1'
  if (rtDW.Delta_factors_G1[2] > 0.1F) {
    q1_q3 = 0.1F;
  } else if (rtDW.Delta_factors_G1[2] < -0.1F) {
    q1_q3 = -0.1F;
  } else {
    q1_q3 = rtDW.Delta_factors_G1[2];
  }

  // Saturate: '<S56>/Saturation2' incorporates:
  //   Delay: '<S56>/Delay'
  //   Saturate: '<S56>/Saturation1'
  //   Sum: '<S56>/Add'

  q1_q3 += rtDW.Delay_DSTATE[2];
  if (q1_q3 > 4.0F) {
    // Update for Delay: '<S56>/Delay'
    rtDW.Delay_DSTATE[2] = 4.0F;
  } else if (q1_q3 < 0.25F) {
    // Update for Delay: '<S56>/Delay'
    rtDW.Delay_DSTATE[2] = 0.25F;
  } else {
    // Update for Delay: '<S56>/Delay'
    rtDW.Delay_DSTATE[2] = q1_q3;
  }

  // Saturate: '<S56>/Saturation1'
  if (rtDW.Delta_factors_G1[3] > 0.1F) {
    q1_q3 = 0.1F;
  } else if (rtDW.Delta_factors_G1[3] < -0.1F) {
    q1_q3 = -0.1F;
  } else {
    q1_q3 = rtDW.Delta_factors_G1[3];
  }

  // Saturate: '<S56>/Saturation2' incorporates:
  //   Delay: '<S56>/Delay'
  //   Saturate: '<S56>/Saturation1'
  //   Sum: '<S56>/Add'

  q1_q3 += rtDW.Delay_DSTATE[3];
  if (q1_q3 > 4.0F) {
    // Update for Delay: '<S56>/Delay'
    rtDW.Delay_DSTATE[3] = 4.0F;
  } else if (q1_q3 < 0.25F) {
    // Update for Delay: '<S56>/Delay'
    rtDW.Delay_DSTATE[3] = 0.25F;
  } else {
    // Update for Delay: '<S56>/Delay'
    rtDW.Delay_DSTATE[3] = q1_q3;
  }

  // Saturate: '<S57>/Saturation1'
  if (rtDW.Delta_factors_G2 > 0.1F) {
    q1_q3 = 0.1F;
  } else if (rtDW.Delta_factors_G2 < -0.1F) {
    q1_q3 = -0.1F;
  } else {
    q1_q3 = rtDW.Delta_factors_G2;
  }

  // End of Saturate: '<S57>/Saturation1'

  // Sum: '<S57>/Add1' incorporates:
  //   Delay: '<S57>/Delay1'

  q1_q3 += rtDW.Delay1_DSTATE;

  // Saturate: '<S57>/Saturation3'
  if (q1_q3 > 4.0F) {
    // Update for Delay: '<S57>/Delay1'
    rtDW.Delay1_DSTATE = 4.0F;
  } else if (q1_q3 < 0.25F) {
    // Update for Delay: '<S57>/Delay1'
    rtDW.Delay1_DSTATE = 0.25F;
  } else {
    // Update for Delay: '<S57>/Delay1'
    rtDW.Delay1_DSTATE = q1_q3;
  }

  // End of Saturate: '<S57>/Saturation3'

  // Update for DiscreteIntegrator: '<S92>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LOAD = 0U;

  // Update for DiscreteIntegrator: '<S91>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LO_p = 0U;

  // Update for UnitDelay: '<S46>/Unit Delay1'
  rtDW.UnitDelay1_DSTATE = rtDW.Merge1;

  // Update for Delay: '<S47>/Delay'
  rtDW.icLoad = 0U;
  rtDW.Delay_DSTATE_b = rtb_Gain6;

  // Update for DiscreteIntegrator: '<S92>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S92>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTATE[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_D_nb[0];

  // Update for DiscreteIntegrator: '<S91>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S91>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_a[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_n[0];

  // Update for DiscreteIntegrator: '<S89>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S89>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTA_al[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_o[0];

  // Update for DiscreteIntegrator: '<S93>/Discrete-Time Integrator y_dt' incorporates:
  //   DiscreteIntegrator: '<S93>/Discrete-Time Integrator y'
  //   Inport: '<Root>/measure'
  //   Product: '<S93>/Product1'
  //   Product: '<S93>/Product2'
  //   Sum: '<S93>/Sum2'
  //   Sum: '<S93>/Sum3'

  rtDW.DiscreteTimeIntegratory_dt_DSTA[0] += (rtU.measure.omega_Kb[0] -
    (rtDW.DiscreteTimeIntegratory_dt_DSTA[0] * wp_approach_out_tmp +
     rtDW.DiscreteTimeIntegratory_DSTAT_k[0])) * scale_tmp * 0.0025F;

  // Update for DiscreteIntegrator: '<S92>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S92>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTATE[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_D_nb[1];

  // Update for DiscreteIntegrator: '<S91>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S91>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_a[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_n[1];

  // Update for DiscreteIntegrator: '<S89>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S89>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTA_al[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_o[1];

  // Update for DiscreteIntegrator: '<S93>/Discrete-Time Integrator y_dt' incorporates:
  //   DiscreteIntegrator: '<S93>/Discrete-Time Integrator y'
  //   Inport: '<Root>/measure'
  //   Product: '<S93>/Product1'
  //   Product: '<S93>/Product2'
  //   Sum: '<S93>/Sum2'
  //   Sum: '<S93>/Sum3'

  rtDW.DiscreteTimeIntegratory_dt_DSTA[1] += (rtU.measure.omega_Kb[1] -
    (rtDW.DiscreteTimeIntegratory_dt_DSTA[1] * wp_approach_out_tmp +
     rtDW.DiscreteTimeIntegratory_DSTAT_k[1])) * scale_tmp * 0.0025F;

  // Update for DiscreteIntegrator: '<S92>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S92>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTATE[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_D_nb[2];

  // Update for DiscreteIntegrator: '<S91>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S91>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_a[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_n[2];

  // Update for DiscreteIntegrator: '<S89>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S89>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTA_al[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_o[2];

  // Update for DiscreteIntegrator: '<S93>/Discrete-Time Integrator y_dt' incorporates:
  //   DiscreteIntegrator: '<S93>/Discrete-Time Integrator y'
  //   Inport: '<Root>/measure'
  //   Product: '<S93>/Product1'
  //   Product: '<S93>/Product2'
  //   Sum: '<S93>/Sum2'
  //   Sum: '<S93>/Sum3'

  rtDW.DiscreteTimeIntegratory_dt_DSTA[2] += (rtU.measure.omega_Kb[2] -
    (rtDW.DiscreteTimeIntegratory_dt_DSTA[2] * wp_approach_out_tmp +
     rtDW.DiscreteTimeIntegratory_DSTAT_k[2])) * scale_tmp * 0.0025F;

  // Update for DiscreteIntegrator: '<S90>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LO_a = 0U;

  // Update for DiscreteIntegrator: '<S123>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LO_b = 0U;

  // Update for DiscreteIntegrator: '<S125>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LO_j = 0U;

  // Update for DiscreteIntegrator: '<S93>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_L_jp = 0U;

  // Update for DiscreteIntegrator: '<S123>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S123>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_m[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_j[0];

  // Update for DiscreteIntegrator: '<S123>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DS_j[0] += 0.0025F * rtb_n_g_des_n[0];

  // Update for DiscreteIntegrator: '<S125>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S125>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTA_pz[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_b[0];

  // Update for DiscreteIntegrator: '<S125>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DS_b[0] += 0.0025F * rtb_y_o[0];

  // Update for DiscreteIntegrator: '<S93>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_DSTAT_k[0] += 0.0025F * a_Kb_meas[0];

  // Update for DiscreteIntegrator: '<S123>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S123>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_m[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_j[1];

  // Update for DiscreteIntegrator: '<S123>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DS_j[1] += 0.0025F * rtb_n_g_des_n[1];

  // Update for DiscreteIntegrator: '<S125>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S125>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTA_pz[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_b[1];

  // Update for DiscreteIntegrator: '<S125>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DS_b[1] += 0.0025F * rtb_y_o[1];

  // Update for DiscreteIntegrator: '<S93>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_DSTAT_k[1] += 0.0025F * a_Kb_meas[1];

  // Update for DiscreteIntegrator: '<S123>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S123>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_m[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_j[2];

  // Update for DiscreteIntegrator: '<S123>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DS_j[2] += 0.0025F * q2_q3;

  // Update for DiscreteIntegrator: '<S125>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S125>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTA_pz[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_b[2];

  // Update for DiscreteIntegrator: '<S125>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DS_b[2] += 0.0025F * rtb_y_o[2];

  // Update for DiscreteIntegrator: '<S93>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_DSTAT_k[2] += 0.0025F * a_Kb_meas[2];

  // Update for DiscreteIntegrator: '<S122>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_DSTATE += 0.0025F * cos_Theta;

  // Update for DiscreteIntegrator: '<S103>/Discrete-Time Integrator2'
  rtDW.DiscreteTimeIntegrator2_IC_LOAD = 0U;
  rtDW.DiscreteTimeIntegrator2_DSTATE += 0.0025F * scale;

  // Update for DiscreteIntegrator: '<S80>/Discrete-Time Integrator' incorporates:
  //   Constant: '<S80>/T'
  //   Product: '<S80>/Divide'
  //   Sum: '<S80>/Sum2'

  rtDW.DiscreteTimeIntegrator_DSTATE_e[0] += (rtb_q_red_l[0] -
    rtDW.DiscreteTimeIntegrator_DSTATE_e[0]) / rtP.lindi.mtc * 0.0025F;

  // Update for UnitDelay: '<S72>/Unit Delay1'
  rtDW.UnitDelay1_DSTATE_c[0] = rtb_Add_a[0];

  // Update for DiscreteIntegrator: '<S74>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S74>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_n[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_l[0];

  // Update for DiscreteIntegrator: '<S74>/Discrete-Time Integrator y_dt' incorporates:
  //   Product: '<S74>/Product1'

  rtDW.DiscreteTimeIntegratory_dt_DS_l[0] += rtb_Sum2_a[0] * scale_tmp * 0.0025F;

  // Update for DiscreteIntegrator: '<S80>/Discrete-Time Integrator' incorporates:
  //   Constant: '<S80>/T'
  //   Product: '<S80>/Divide'
  //   Sum: '<S80>/Sum2'

  rtDW.DiscreteTimeIntegrator_DSTATE_e[1] += (rtb_q_red_l[1] -
    rtDW.DiscreteTimeIntegrator_DSTATE_e[1]) / rtP.lindi.mtc * 0.0025F;

  // Update for UnitDelay: '<S72>/Unit Delay1'
  rtDW.UnitDelay1_DSTATE_c[1] = rtb_Add_a[1];

  // Update for DiscreteIntegrator: '<S74>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S74>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_n[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_l[1];

  // Update for DiscreteIntegrator: '<S74>/Discrete-Time Integrator y_dt' incorporates:
  //   Product: '<S74>/Product1'

  rtDW.DiscreteTimeIntegratory_dt_DS_l[1] += rtb_Sum2_a[1] * scale_tmp * 0.0025F;

  // Update for DiscreteIntegrator: '<S80>/Discrete-Time Integrator' incorporates:
  //   Constant: '<S80>/T'
  //   Product: '<S80>/Divide'
  //   Sum: '<S80>/Sum2'

  rtDW.DiscreteTimeIntegrator_DSTATE_e[2] += (rtb_q_red_l[2] -
    rtDW.DiscreteTimeIntegrator_DSTATE_e[2]) / rtP.lindi.mtc * 0.0025F;

  // Update for UnitDelay: '<S72>/Unit Delay1'
  rtDW.UnitDelay1_DSTATE_c[2] = rtb_Add_a[2];

  // Update for DiscreteIntegrator: '<S74>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S74>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_n[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_l[2];

  // Update for DiscreteIntegrator: '<S74>/Discrete-Time Integrator y_dt' incorporates:
  //   Product: '<S74>/Product1'

  rtDW.DiscreteTimeIntegratory_dt_DS_l[2] += rtb_Sum2_a[2] * scale_tmp * 0.0025F;

  // Update for DiscreteIntegrator: '<S80>/Discrete-Time Integrator' incorporates:
  //   Constant: '<S80>/T'
  //   Product: '<S80>/Divide'
  //   Sum: '<S80>/Sum2'

  rtDW.DiscreteTimeIntegrator_DSTATE_e[3] += (t -
    rtDW.DiscreteTimeIntegrator_DSTATE_e[3]) / rtP.lindi.mtc * 0.0025F;

  // Update for UnitDelay: '<S72>/Unit Delay1'
  rtDW.UnitDelay1_DSTATE_c[3] = rtb_Add_a[3];

  // Update for DiscreteIntegrator: '<S74>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S74>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_n[3] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_l[3];

  // Update for DiscreteIntegrator: '<S74>/Discrete-Time Integrator y_dt' incorporates:
  //   Product: '<S74>/Product1'

  rtDW.DiscreteTimeIntegratory_dt_DS_l[3] += rtb_Sum2_a[3] * scale_tmp * 0.0025F;
  for (wp_idx_app_2 = 0; wp_idx_app_2 < 9; wp_idx_app_2++) {
    // Update for DiscreteIntegrator: '<S90>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S90>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTAT_p[wp_idx_app_2] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_D_bx[wp_idx_app_2];

    // Update for DiscreteIntegrator: '<S90>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S90>/Product1'
    //   Product: '<S90>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_D_bx[wp_idx_app_2] += rtb_M_bg[wp_idx_app_2]
      * scale_tmp * 0.0025F;
  }

  // Update for DiscreteIntegrator: '<S89>/Discrete-Time Integrator y_dt' incorporates:
  //   Product: '<S89>/Product1'

  rtDW.DiscreteTimeIntegratory_dt_DS_o[0] += q1_q2 * scale_tmp * 0.0025F;

  // Update for DiscreteIntegrator: '<S91>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DS_n[0] += 0.0025F * rtb_p_ahead_l[0];

  // Update for DiscreteIntegrator: '<S92>/Discrete-Time Integrator y_dt' incorporates:
  //   Product: '<S92>/Product1'

  rtDW.DiscreteTimeIntegratory_dt_D_nb[0] += rtb_Sum2_e[0] * scale_tmp * 0.0025F;

  // Update for DiscreteIntegrator: '<S89>/Discrete-Time Integrator y_dt' incorporates:
  //   Product: '<S89>/Product1'

  rtDW.DiscreteTimeIntegratory_dt_DS_o[1] += absxk * scale_tmp * 0.0025F;

  // Update for DiscreteIntegrator: '<S91>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DS_n[1] += 0.0025F * rtb_p_ahead_l[1];

  // Update for DiscreteIntegrator: '<S92>/Discrete-Time Integrator y_dt' incorporates:
  //   Product: '<S92>/Product1'

  rtDW.DiscreteTimeIntegratory_dt_D_nb[1] += rtb_Sum2_e[1] * scale_tmp * 0.0025F;

  // Update for DiscreteIntegrator: '<S89>/Discrete-Time Integrator y_dt' incorporates:
  //   Product: '<S89>/Product1'

  rtDW.DiscreteTimeIntegratory_dt_DS_o[2] += q0_q2 * scale_tmp * 0.0025F;

  // Update for DiscreteIntegrator: '<S91>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DS_n[2] += 0.0025F * rtb_p_ahead_l[2];

  // Update for DiscreteIntegrator: '<S92>/Discrete-Time Integrator y_dt' incorporates:
  //   Product: '<S92>/Product1'

  rtDW.DiscreteTimeIntegratory_dt_D_nb[2] += rtb_Sum2_e[2] * scale_tmp * 0.0025F;
}

// Model initialize function
void MatlabControllerClass::initialize()
{
  // Start for Constant: '<S34>/y_0'
  rtDW.y_0 = rtP.lindi.dive.thrfall;

  // End of Start for SubSystem: '<S11>/Throttle Intercept Arc (Load factor controller)' 

  // InitializeConditions for Delay: '<S56>/Delay'
  rtDW.Delay_DSTATE[0] = 1.0F;
  rtDW.Delay_DSTATE[1] = 1.0F;
  rtDW.Delay_DSTATE[2] = 1.0F;
  rtDW.Delay_DSTATE[3] = 1.0F;

  // InitializeConditions for Delay: '<S57>/Delay1'
  rtDW.Delay1_DSTATE = 1.0F;

  // InitializeConditions for DiscreteIntegrator: '<S92>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LOAD = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S91>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LO_p = 1U;

  // InitializeConditions for Delay: '<S47>/Delay'
  rtDW.icLoad = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S90>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LO_a = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S123>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LO_b = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S125>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LO_j = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S93>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_L_jp = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S103>/Discrete-Time Integrator2' 
  rtDW.DiscreteTimeIntegrator2_IC_LOAD = 1U;

  // SystemInitialize for Enabled SubSystem: '<S3>/fast descent sequencer'
  // InitializeConditions for DiscreteIntegrator: '<S12>/Discrete-Time Integrator' 
  rtDW.DiscreteTimeIntegrator_PrevRe_g = 2;

  // SystemInitialize for Enabled SubSystem: '<S11>/Duration in slowing'
  // InitializeConditions for DiscreteIntegrator: '<S18>/Discrete-Time Integrator' 
  rtDW.DiscreteTimeIntegrator_PrevRe_c = 2;

  // End of SystemInitialize for SubSystem: '<S11>/Duration in slowing'

  // SystemInitialize for Enabled SubSystem: '<S11>/Throttle Intercept Arc (Load factor controller)' 
  // InitializeConditions for DiscreteIntegrator: '<S34>/Discrete-Time Integrator' 
  rtDW.DiscreteTimeIntegrator_DSTATE_m = rtDW.y_0;

  // End of SystemInitialize for SubSystem: '<S11>/Throttle Intercept Arc (Load factor controller)' 
  // End of SystemInitialize for SubSystem: '<S3>/fast descent sequencer'

  // SystemInitialize for Enabled SubSystem: '<S4>/NDI position controller for copters reference model' 
  // InitializeConditions for DiscreteIntegrator: '<S43>/Discrete-Time Integrator1' 
  rtDW.DiscreteTimeIntegrator1_IC_LOAD = 1U;
  rtDW.DiscreteTimeIntegrator1_PrevRes = 2;

  // InitializeConditions for DiscreteIntegrator: '<S127>/Discrete-Time Integrator' 
  rtDW.DiscreteTimeIntegrator_IC_LOADI = 1U;
  rtDW.DiscreteTimeIntegrator_PrevRe_m = 2;

  // InitializeConditions for DiscreteIntegrator: '<S128>/Discrete-Time Integrator' 
  rtDW.DiscreteTimeIntegrator_IC_LOA_a = 1U;
  rtDW.DiscreteTimeIntegrator_PrevRe_i = 2;

  // End of SystemInitialize for SubSystem: '<S4>/NDI position controller for copters reference model' 

  // SystemInitialize for Enabled SubSystem: '<S4>/Waypoint Navigation'
  // InitializeConditions for UnitDelay: '<S49>/Unit Delay'
  rtDW.UnitDelay_DSTATE_o = 1;

  // InitializeConditions for UnitDelay: '<S49>/Unit Delay1'
  rtDW.UnitDelay1_DSTATE_f = 1;

  // InitializeConditions for UnitDelay: '<S49>/Unit Delay2'
  rtDW.UnitDelay2_DSTATE = 2;

  // SystemInitialize for Enabled SubSystem: '<S151>/Flight Path Smoothing'
  // InitializeConditions for DiscreteIntegrator: '<S161>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_L_ai = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S162>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LO_d = 1U;

  // End of SystemInitialize for SubSystem: '<S151>/Flight Path Smoothing'

  // InitializeConditions for UnitDelay: '<S49>/Unit Delay6'
  rtDW.UnitDelay6_DSTATE[0] = 1.0F;
  rtDW.UnitDelay6_DSTATE[1] = 1.0F;
  rtDW.UnitDelay6_DSTATE[2] = 1.0F;

  // End of SystemInitialize for SubSystem: '<S4>/Waypoint Navigation'

  // SystemInitialize for Enabled SubSystem: '<S4>/NDI position controller for copters with reference input' 
  // InitializeConditions for DiscreteIntegrator: '<S136>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LO_h = 1U;
  rtDW.DiscreteTimeIntegratory_PrevR_a = 2;

  // InitializeConditions for DiscreteIntegrator: '<S136>/Discrete-Time Integrator y_dt' 
  rtDW.DiscreteTimeIntegratory_dt_P_mk = 2;

  // End of SystemInitialize for SubSystem: '<S4>/NDI position controller for copters with reference input' 

  // SystemInitialize for Enabled SubSystem: '<S52>/Copter Random Excitation'
  // InitializeConditions for RandomNumber: '<S64>/White Noise'
  rtDW.RandSeed[0] = 1529675776U;
  rtDW.NextOutput[0] = rt_nrand_Upu32_Yd_f_pw(&rtDW.RandSeed[0]);
  rtDW.RandSeed[1] = 1529741312U;
  rtDW.NextOutput[1] = rt_nrand_Upu32_Yd_f_pw(&rtDW.RandSeed[1]);
  rtDW.RandSeed[2] = 1529806848U;
  rtDW.NextOutput[2] = rt_nrand_Upu32_Yd_f_pw(&rtDW.RandSeed[2]);
  rtDW.RandSeed[3] = 1529872384U;
  rtDW.NextOutput[3] = rt_nrand_Upu32_Yd_f_pw(&rtDW.RandSeed[3]);

  // End of SystemInitialize for SubSystem: '<S52>/Copter Random Excitation'

  // SystemInitialize for Enabled SubSystem: '<S4>/Incremental specific thrust'
  // InitializeConditions for DiscreteIntegrator: '<S86>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LO_e = 1U;

  // End of SystemInitialize for SubSystem: '<S4>/Incremental specific thrust'

  // SystemInitialize for Enabled SubSystem: '<S52>/INDI Inversion Check'
  // InitializeConditions for DiscreteIntegrator: '<S70>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LO_f = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S69>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_L_p0 = 1U;

  // End of SystemInitialize for SubSystem: '<S52>/INDI Inversion Check'
}

// Constructor
MatlabControllerClass::MatlabControllerClass()
{
  AP_Param::setup_object_defaults(this, var_info_rtP_lindi_ca);
  AP_Param::setup_object_defaults(this, var_info_rtP_lindi_thr);
  AP_Param::setup_object_defaults(this, var_info_rtP_lindi_cep);
  AP_Param::setup_object_defaults(this, var_info_rtP_lindi_ceb);
  AP_Param::setup_object_defaults(this, var_info_rtP_lindi_psc);
  AP_Param::setup_object_defaults(this, var_info_rtP_lindi_atc);
  AP_Param::setup_object_defaults(this, var_info_rtP_lindi_sflt);
  AP_Param::setup_object_defaults(this, var_info_rtP_lindi_wpnav);
  AP_Param::setup_object_defaults(this, var_info_rtP_lindi_dive);
  AP_Param::setup_object_defaults(this, var_info);

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
