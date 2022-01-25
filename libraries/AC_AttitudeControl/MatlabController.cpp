//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: MatlabController.cpp
//
// Code generated for Simulink model 'ArduCopter_MinnieLoiterController_3_mot_fails'.
//
// Model version                  : 1.402
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Thu Jan 20 16:38:28 2022
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#include "MatlabController.h"

extern real32_T rt_hypotf(real32_T u0, real32_T u1);
static real32_T look1_iflf_binlx(real32_T u0, const real32_T bp0[], const
  real32_T table[], uint32_T maxIndex);
static void DCMtoquaternions(const real32_T rtu_M_bg[9], real32_T rty_q_bg[4]);
static void MATLABFunction4(const real32_T rtu_n_g[3], real32_T *rty_phi,
  real32_T *rty_delta);
static void MATLABFunction3(real32_T rtu_phi, real32_T rtu_delta, real32_T
  rty_q_cmd_red[4]);
static void wrapangle(real32_T rtu_angle, real32_T *rty_angle_0_2pi);
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
//    '<S16>/DCM to quaternions'
//    '<S59>/DCM to quaternions'
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

//
// Output and update for atomic system:
//    '<S16>/MATLAB Function4'
//    '<S21>/MATLAB Function4'
//
static void MATLABFunction4(const real32_T rtu_n_g[3], real32_T *rty_phi,
  real32_T *rty_delta)
{
  real32_T minval;
  if (1.0F > -rtu_n_g[2]) {
    minval = -rtu_n_g[2];
  } else {
    minval = 1.0F;
  }

  *rty_delta = std::atan2(rtu_n_g[1], rtu_n_g[0]);
  if (-1.0F >= minval) {
    minval = -1.0F;
  }

  *rty_phi = std::acos(minval);
}

//
// Output and update for atomic system:
//    '<S25>/MATLAB Function3'
//    '<S25>/MATLAB Function8'
//
static void MATLABFunction3(real32_T rtu_phi, real32_T rtu_delta, real32_T
  rty_q_cmd_red[4])
{
  real32_T x;
  real32_T xy;
  real32_T b_x;
  real32_T b_x_tmp;
  x = std::sin(rtu_delta);
  b_x_tmp = std::cos(rtu_delta);
  xy = std::sqrt(x * x + -b_x_tmp * -b_x_tmp);
  if (xy <= 0.01) {
    x = 1.0F;
    xy = std::sqrt(-b_x_tmp * -b_x_tmp + 1.0F);
  }

  b_x = 1.0F / xy;
  xy = std::sin(rtu_phi / 2.0F);
  rty_q_cmd_red[0] = std::cos(rtu_phi / 2.0F);
  rty_q_cmd_red[1] = b_x * x * xy;
  rty_q_cmd_red[2] = b_x * -b_x_tmp * xy;
  rty_q_cmd_red[3] = 0.0F;
}

//
// Output and update for atomic system:
//    '<S58>/wrap angle'
//    '<S58>/wrap angle1'
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

// Function for MATLAB Function: '<S65>/MATLAB Function1'
real32_T MatlabControllerClass::norm_c(const real32_T x[3])
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

// Function for MATLAB Function: '<S59>/MATLAB Function1'
void MatlabControllerClass::quatNormalize(const real32_T q[4], real32_T q_out[4])
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

// Function for MATLAB Function: '<S52>/MATLAB Function'
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

// Function for MATLAB Function: '<S52>/MATLAB Function'
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

// Function for MATLAB Function: '<S52>/MATLAB Function'
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

// Function for MATLAB Function: '<S52>/MATLAB Function'
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

// Function for MATLAB Function: '<S52>/MATLAB Function'
void MatlabControllerClass::mldivide(const real32_T A_data[], const int32_T
  A_size[2], const real32_T B_0[8], real32_T Y_data[], int32_T *Y_size)
{
  if (A_size[1] == 0) {
    *Y_size = 0;
  } else {
    qrsolve(A_data, A_size, B_0, Y_data, Y_size);
  }
}

// Function for MATLAB Function: '<S52>/MATLAB Function'
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

// Model step function
void MatlabControllerClass::step()
{
  real32_T sin_Psi;
  real32_T cos_Psi;
  real32_T d[16];
  real32_T q0_q0;
  real32_T q0_q2;
  real32_T q0_q3;
  real32_T q1_q2;
  real32_T q1_q3;
  real32_T q2_q3;
  real32_T scale;
  real32_T absxk;
  real32_T t;
  int32_T k;
  int32_T exponent;
  real32_T q_yaw[4];
  real32_T A[32];
  real32_T d_0[8];
  boolean_T i_free[4];
  real32_T A_free_data[32];
  real32_T p_free_data[4];
  real_T p[4];
  real_T c_data[4];
  int8_T f_data[4];
  int8_T g_data[4];
  int8_T h_data[4];
  int8_T i_data[4];
  int32_T b_aoffset;
  boolean_T x[4];
  real32_T rtb_z_Kg;
  real32_T rtb_y_dt_m[2];
  real32_T rtb_phi_k;
  real32_T rtb_n_b_d_dt2[3];
  real32_T rtb_Diff[3];
  real32_T rtb_q_cmd_red_c[4];
  boolean_T rtb_Compare;
  real32_T rtb_Diff_j[3];
  real32_T rtb_Add[3];
  real32_T rtb_delta_f;
  real32_T rtb_n_b_d_dt_p[3];
  real32_T rtb_n_b_d_dt2_g[3];
  real32_T rtb_q_red[4];
  real32_T rtb_q_bg[4];
  real32_T rtb_uDLookupTable2[4];
  real32_T rtb_umax[4];
  real32_T rtb_ny_du_red_ff[16];
  real32_T rtb_G2[16];
  real32_T rtb_M_bg[9];
  real32_T rtb_Add_gb[6];
  real32_T rtb_Add1_m[2];
  real32_T rtb_TmpSignalConversionAtSFunct[3];
  real32_T rtb_Wv[16];
  int32_T i;
  boolean_T rtb_q_bg_data[4];
  int8_T cos_Psi_0[9];
  real32_T tmp[6];
  real32_T tmp_0[2];
  real32_T tmp_1[3];
  real32_T rtb_Diff_1[3];
  real32_T rtb_M_bg_0[9];
  real32_T rtb_ny_du_red_ff_0[16];
  real32_T rtb_lean_angle_max_0[9];
  real32_T rtb_Wv_0[8];
  boolean_T rtb_q_bg_0[4];
  int32_T A_free_size[2];
  real_T p_0;
  real32_T rtb_q_cmd;
  real32_T rtb_y_idx_3;
  real32_T rtb_y_idx_2;
  real32_T rtb_y_idx_1;
  real32_T rtb_y_idx_0;
  real32_T rtb_y_f_idx_1;
  real32_T rtb_y_f_idx_0;
  real32_T cmd_V_NED_idx_1;
  real32_T cmd_V_NED_idx_0;
  real32_T rtb_TSamp_idx_1;
  real32_T rtb_TSamp_idx_0;
  real_T dist_idx_3;
  real_T dist_idx_2;
  real_T dist_idx_1;
  boolean_T e_idx_3;
  boolean_T e_idx_2;
  boolean_T e_idx_1;
  real_T dist_idx_0;
  boolean_T e_idx_0;
  real32_T tmp_2;
  real32_T tmp_3;
  real32_T M_bg_idx_8;
  real32_T y;
  real32_T u0;
  real32_T y_0;
  real32_T y_1;
  real32_T rtb_Wv_1;
  int32_T rtb_ny_du_red_ff_tmp;
  int32_T rtb_ny_du_red_ff_tmp_0;
  int32_T rtb_ny_du_red_ff_tmp_1;
  real32_T rtb_M_bg_tmp;
  boolean_T exitg1;
  boolean_T exitg2;

  // MATLAB Function: '<S51>/MATLAB Function1' incorporates:
  //   Constant: '<S51>/ny_du_red_trim'

  memcpy(&rtb_ny_du_red_ff[0], &rtConstP.ny_du_red_trim_Value[0], sizeof
         (real32_T) << 4U);
  cos_Psi_0[0] = 1;
  cos_Psi_0[3] = 0;
  cos_Psi_0[6] = 0;
  cos_Psi_0[1] = 0;
  cos_Psi_0[4] = 1;
  cos_Psi_0[7] = 0;
  cos_Psi_0[2] = 0;
  cos_Psi_0[5] = 0;
  cos_Psi_0[8] = 1;
  for (k = 0; k < 4; k++) {
    for (i = 0; i < 3; i++) {
      rtb_ny_du_red_ff_tmp = k << 2;
      rtb_ny_du_red_ff_tmp_0 = i + rtb_ny_du_red_ff_tmp;
      rtb_ny_du_red_ff[rtb_ny_du_red_ff_tmp_0] = 0.0F;
      rtb_ny_du_red_ff_tmp_1 = rtb_ny_du_red_ff_tmp + i;
      rtb_ny_du_red_ff[rtb_ny_du_red_ff_tmp_0] =
        rtb_ny_du_red_ff[rtb_ny_du_red_ff_tmp_1] +
        rtConstP.ny_du_red_trim_Value[rtb_ny_du_red_ff_tmp] * (real32_T)
        cos_Psi_0[i];
      rtb_ny_du_red_ff[rtb_ny_du_red_ff_tmp_0] =
        rtConstP.ny_du_red_trim_Value[rtb_ny_du_red_ff_tmp + 1] * (real32_T)
        cos_Psi_0[i + 3] + rtb_ny_du_red_ff[rtb_ny_du_red_ff_tmp_1];
      rtb_ny_du_red_ff[rtb_ny_du_red_ff_tmp_0] =
        rtConstP.ny_du_red_trim_Value[rtb_ny_du_red_ff_tmp + 2] * (real32_T)
        cos_Psi_0[i + 6] + rtb_ny_du_red_ff[rtb_ny_du_red_ff_tmp_1];
    }
  }

  // End of MATLAB Function: '<S51>/MATLAB Function1'

  // DiscreteIntegrator: '<S93>/Discrete-Time Integrator'
  rtb_y_idx_0 = rtDW.DiscreteTimeIntegrator_DSTATE[0];
  rtb_y_idx_1 = rtDW.DiscreteTimeIntegrator_DSTATE[1];
  rtb_y_idx_2 = rtDW.DiscreteTimeIntegrator_DSTATE[2];
  rtb_y_idx_3 = rtDW.DiscreteTimeIntegrator_DSTATE[3];

  // MATLAB Function: '<S51>/MATLAB Function' incorporates:
  //   Constant: '<S51>/Constant4'
  //   Constant: '<S51>/ny_du_dt'

  for (i = 0; i < 16; i++) {
    rtb_G2[i] = rtConstP.ny_du_dt_Value[i] / 0.0025F;
    d[i] = 0.0F;
  }

  // MATLAB Function: '<Root>/Quaternions to Rotation Matrix'
  scale = 1.29246971E-26F;

  // MinMax: '<S51>/Max' incorporates:
  //   DiscreteIntegrator: '<S93>/Discrete-Time Integrator'

  if (rtDW.DiscreteTimeIntegrator_DSTATE[0] > 0.1F) {
    rtb_z_Kg = rtDW.DiscreteTimeIntegrator_DSTATE[0];
  } else {
    rtb_z_Kg = 0.1F;
  }

  // MATLAB Function: '<S51>/MATLAB Function'
  d[0] = rtb_z_Kg / 0.283F;

  // MATLAB Function: '<Root>/Quaternions to Rotation Matrix' incorporates:
  //   Inport: '<Root>/measure'

  absxk = std::abs(rtU.measure.q_bg[0]);
  if (absxk > 1.29246971E-26F) {
    q0_q0 = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    q0_q0 = t * t;
  }

  // MinMax: '<S51>/Max' incorporates:
  //   DiscreteIntegrator: '<S93>/Discrete-Time Integrator'

  if (rtDW.DiscreteTimeIntegrator_DSTATE[1] > 0.1F) {
    rtb_z_Kg = rtDW.DiscreteTimeIntegrator_DSTATE[1];
  } else {
    rtb_z_Kg = 0.1F;
  }

  // MATLAB Function: '<S51>/MATLAB Function'
  d[5] = rtb_z_Kg / 0.283F;

  // MATLAB Function: '<Root>/Quaternions to Rotation Matrix' incorporates:
  //   Inport: '<Root>/measure'

  absxk = std::abs(rtU.measure.q_bg[1]);
  if (absxk > scale) {
    t = scale / absxk;
    q0_q0 = q0_q0 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q0_q0 += t * t;
  }

  // MinMax: '<S51>/Max' incorporates:
  //   DiscreteIntegrator: '<S93>/Discrete-Time Integrator'

  if (rtDW.DiscreteTimeIntegrator_DSTATE[2] > 0.1F) {
    rtb_z_Kg = rtDW.DiscreteTimeIntegrator_DSTATE[2];
  } else {
    rtb_z_Kg = 0.1F;
  }

  // MATLAB Function: '<S51>/MATLAB Function'
  d[10] = rtb_z_Kg / 0.283F;

  // MATLAB Function: '<Root>/Quaternions to Rotation Matrix' incorporates:
  //   Inport: '<Root>/measure'

  absxk = std::abs(rtU.measure.q_bg[2]);
  if (absxk > scale) {
    t = scale / absxk;
    q0_q0 = q0_q0 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q0_q0 += t * t;
  }

  // MinMax: '<S51>/Max' incorporates:
  //   DiscreteIntegrator: '<S93>/Discrete-Time Integrator'

  if (rtDW.DiscreteTimeIntegrator_DSTATE[3] > 0.1F) {
    rtb_z_Kg = rtDW.DiscreteTimeIntegrator_DSTATE[3];
  } else {
    rtb_z_Kg = 0.1F;
  }

  // MATLAB Function: '<S51>/MATLAB Function'
  d[15] = rtb_z_Kg / 0.283F;

  // MATLAB Function: '<Root>/Quaternions to Rotation Matrix' incorporates:
  //   Inport: '<Root>/measure'

  absxk = std::abs(rtU.measure.q_bg[3]);
  if (absxk > scale) {
    t = scale / absxk;
    q0_q0 = q0_q0 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q0_q0 += t * t;
  }

  q0_q0 = scale * std::sqrt(q0_q0);
  if (2.22044605E-16F >= q0_q0) {
    q0_q0 = 2.22044605E-16F;
  }

  rtb_q_bg[0] = rtU.measure.q_bg[0] / q0_q0;
  rtb_q_bg[1] = rtU.measure.q_bg[1] / q0_q0;
  rtb_q_bg[2] = rtU.measure.q_bg[2] / q0_q0;
  rtb_q_bg[3] = rtU.measure.q_bg[3] / q0_q0;
  q0_q0 = rtb_q_bg[0] * rtb_q_bg[0];
  scale = rtb_q_bg[1] * rtb_q_bg[1];
  t = rtb_q_bg[2] * rtb_q_bg[2];
  absxk = rtb_q_bg[3] * rtb_q_bg[3];
  M_bg_idx_8 = rtb_q_bg[0] * rtb_q_bg[1];
  q0_q2 = rtb_q_bg[0] * rtb_q_bg[2];
  q0_q3 = rtb_q_bg[0] * rtb_q_bg[3];
  q1_q2 = rtb_q_bg[1] * rtb_q_bg[2];
  q1_q3 = rtb_q_bg[1] * rtb_q_bg[3];
  q2_q3 = rtb_q_bg[2] * rtb_q_bg[3];
  rtb_M_bg[0] = ((q0_q0 + scale) - t) - absxk;
  rtb_M_bg[3] = (q1_q2 + q0_q3) * 2.0F;
  rtb_M_bg[6] = (q1_q3 - q0_q2) * 2.0F;
  rtb_M_bg[1] = (q1_q2 - q0_q3) * 2.0F;
  rtb_M_bg_tmp = q0_q0 - scale;
  rtb_M_bg[4] = (rtb_M_bg_tmp + t) - absxk;
  rtb_M_bg[7] = (q2_q3 + M_bg_idx_8) * 2.0F;
  rtb_M_bg[2] = (q1_q3 + q0_q2) * 2.0F;
  rtb_M_bg[5] = (q2_q3 - M_bg_idx_8) * 2.0F;
  rtb_M_bg[8] = (rtb_M_bg_tmp - t) + absxk;

  // SampleTimeMath: '<S40>/TSamp' incorporates:
  //   Inport: '<Root>/measure'
  //
  //  About '<S40>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  rtb_M_bg_tmp = rtU.measure.omega_Kb[0] * 400.0F;

  // Sum: '<S40>/Diff' incorporates:
  //   UnitDelay: '<S40>/UD'
  //
  //  Block description for '<S40>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S40>/UD':
  //
  //   Store in Global RAM

  rtb_Diff[0] = rtb_M_bg_tmp - rtDW.UD_DSTATE[0];

  // SampleTimeMath: '<S40>/TSamp' incorporates:
  //   Inport: '<Root>/measure'
  //
  //  About '<S40>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  rtb_TSamp_idx_0 = rtb_M_bg_tmp;
  rtb_M_bg_tmp = rtU.measure.omega_Kb[1] * 400.0F;

  // Sum: '<S40>/Diff' incorporates:
  //   UnitDelay: '<S40>/UD'
  //
  //  Block description for '<S40>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S40>/UD':
  //
  //   Store in Global RAM

  rtb_Diff[1] = rtb_M_bg_tmp - rtDW.UD_DSTATE[1];

  // SampleTimeMath: '<S40>/TSamp' incorporates:
  //   Inport: '<Root>/measure'
  //
  //  About '<S40>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  rtb_TSamp_idx_1 = rtb_M_bg_tmp;
  rtb_M_bg_tmp = rtU.measure.omega_Kb[2] * 400.0F;

  // Sum: '<S40>/Diff' incorporates:
  //   UnitDelay: '<S40>/UD'
  //
  //  Block description for '<S40>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S40>/UD':
  //
  //   Store in Global RAM

  rtb_Diff[2] = rtb_M_bg_tmp - rtDW.UD_DSTATE[2];

  // Gain: '<S26>/Gain3' incorporates:
  //   Gain: '<Root>/Gain'
  //   Inport: '<Root>/cmd'
  //   MATLAB Function: '<S14>/MATLAB Function'
  //   MATLAB Function: '<S26>/MATLAB Function1'

  rtb_z_Kg = std::sqrt(rtU.cmd.roll * rtU.cmd.roll + -rtU.cmd.pitch *
                       -rtU.cmd.pitch) * 15.0F;

  // DiscreteIntegrator: '<S38>/Discrete-Time Integrator'
  rtb_y_f_idx_0 = rtDW.DiscreteTimeIntegrator_DSTATE_h[0];
  rtb_y_f_idx_1 = rtDW.DiscreteTimeIntegrator_DSTATE_h[1];

  // Gain: '<S38>/1//T' incorporates:
  //   DiscreteIntegrator: '<S38>/Discrete-Time Integrator'
  //   Gain: '<Root>/Gain'
  //   Inport: '<Root>/cmd'
  //   Product: '<S26>/Product'
  //   Sum: '<S38>/Sum2'

  u0 = (-rtU.cmd.pitch * rtb_z_Kg - rtDW.DiscreteTimeIntegrator_DSTATE_h[0]) *
    1.25F;

  // Saturate: '<S38>/Saturation'
  if (u0 > 12.0F) {
    y = 12.0F;
    rtb_y_dt_m[0] = 12.0F;
  } else if (u0 < -12.0F) {
    y = -12.0F;
    rtb_y_dt_m[0] = -12.0F;
  } else {
    y = u0;
    rtb_y_dt_m[0] = u0;
  }

  // Gain: '<S38>/1//T' incorporates:
  //   DiscreteIntegrator: '<S38>/Discrete-Time Integrator'
  //   Inport: '<Root>/cmd'
  //   MATLAB Function: '<S14>/MATLAB Function'
  //   Product: '<S26>/Product'
  //   Sum: '<S38>/Sum2'

  u0 = (rtU.cmd.roll * rtb_z_Kg - rtDW.DiscreteTimeIntegrator_DSTATE_h[1]) *
    1.25F;

  // Saturate: '<S38>/Saturation'
  if (u0 > 12.0F) {
    y_0 = 12.0F;
    rtb_y_dt_m[1] = 12.0F;
  } else if (u0 < -12.0F) {
    y_0 = -12.0F;
    rtb_y_dt_m[1] = -12.0F;
  } else {
    y_0 = u0;
    rtb_y_dt_m[1] = u0;
  }

  // SampleTimeMath: '<S11>/TSamp' incorporates:
  //   Inport: '<Root>/measure'
  //
  //  About '<S11>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  u0 = rtU.measure.V_Kg[0] * 400.0F;

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

  rtb_Diff_j[0] = u0 - rtDW.UD_DSTATE_k[0];

  // SampleTimeMath: '<S11>/TSamp' incorporates:
  //   Inport: '<Root>/measure'
  //
  //  About '<S11>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  cmd_V_NED_idx_0 = u0;
  u0 = rtU.measure.V_Kg[1] * 400.0F;

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

  rtb_Diff_j[1] = u0 - rtDW.UD_DSTATE_k[1];

  // SampleTimeMath: '<S11>/TSamp' incorporates:
  //   Inport: '<Root>/measure'
  //
  //  About '<S11>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  cmd_V_NED_idx_1 = u0;
  u0 = rtU.measure.V_Kg[2] * 400.0F;

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

  rtb_Diff_j[2] = u0 - rtDW.UD_DSTATE_k[2];

  // Sum: '<S28>/Add' incorporates:
  //   DiscreteIntegrator: '<S39>/Discrete-Time Integrator y'
  //   Inport: '<Root>/measure'

  rtb_Add_gb[0] = rtDW.DiscreteTimeIntegratory_DSTAT_j[0] - rtU.measure.s_Kg[0];
  rtb_Add_gb[2] = rtDW.DiscreteTimeIntegratory_DSTAT_j[2] - rtU.measure.V_Kg[0];
  rtb_Add_gb[4] = rtDW.DiscreteTimeIntegratory_DSTAT_j[4] - rtb_Diff_j[0];
  rtb_Add_gb[1] = rtDW.DiscreteTimeIntegratory_DSTAT_j[1] - rtU.measure.s_Kg[1];
  rtb_Add_gb[3] = rtDW.DiscreteTimeIntegratory_DSTAT_j[3] - rtU.measure.V_Kg[1];
  rtb_Add_gb[5] = rtDW.DiscreteTimeIntegratory_DSTAT_j[5] - rtb_Diff_j[1];

  // Gain: '<S28>/Gain'
  for (k = 0; k < 6; k++) {
    // Saturate: '<S28>/Saturation1'
    if (rtb_Add_gb[k] > rtConstP.Saturation1_UpperSat[k]) {
      tmp[k] = rtConstP.Saturation1_UpperSat[k];
    } else if (rtb_Add_gb[k] < rtConstP.Saturation1_LowerSat[k]) {
      tmp[k] = rtConstP.Saturation1_LowerSat[k];
    } else {
      tmp[k] = rtb_Add_gb[k];
    }

    // End of Saturate: '<S28>/Saturation1'
  }

  // DiscreteIntegrator: '<S92>/Discrete-Time Integrator'
  sin_Psi = rtDW.DiscreteTimeIntegrator_DSTATE_d;

  // Gain: '<S92>/1//T' incorporates:
  //   DiscreteIntegrator: '<S92>/Discrete-Time Integrator'
  //   Gain: '<Root>/Gain5'
  //   Inport: '<Root>/cmd'
  //   Lookup_n-D: '<S90>/1-D Lookup Table'
  //   Sum: '<S92>/Sum2'

  cos_Psi = (look1_iflf_binlx(-rtU.cmd.thr, rtConstP.uDLookupTable_bp01Data,
              rtConstP.uDLookupTable_tableData, 2U) -
             rtDW.DiscreteTimeIntegrator_DSTATE_d) * 3.33333325F;

  // Saturate: '<S92>/Saturation'
  if (cos_Psi > 4.0F) {
    cos_Psi = 4.0F;
  } else {
    if (cos_Psi < -25.0F) {
      cos_Psi = -25.0F;
    }
  }

  // End of Saturate: '<S92>/Saturation'
  for (i = 0; i < 2; i++) {
    // Gain: '<S28>/Gain'
    tmp_0[i] = 0.0F;
    for (k = 0; k < 6; k++) {
      tmp_0[i] += rtConstP.Gain_Gain[(k << 1) + i] * tmp[k];
    }

    // Saturate: '<S28>/Saturation' incorporates:
    //   Gain: '<S28>/Gain'
    //   Sum: '<S12>/Add1'

    if (tmp_0[i] > 9.0F) {
      rtb_z_Kg = 9.0F;
    } else if (tmp_0[i] < -9.0F) {
      rtb_z_Kg = -9.0F;
    } else {
      rtb_z_Kg = tmp_0[i];
    }

    // End of Saturate: '<S28>/Saturation'

    // Sum: '<S12>/Add1'
    rtb_delta_f = rtb_y_dt_m[i] + rtb_z_Kg;

    // SignalConversion: '<S29>/TmpSignal ConversionAt SFunction Inport1' incorporates:
    //   MATLAB Function: '<S25>/MATLAB Function1'

    rtb_TmpSignalConversionAtSFunct[i] = rtb_delta_f;

    // Sum: '<S12>/Add1'
    rtb_Add1_m[i] = rtb_delta_f;
  }

  // SignalConversion: '<S29>/TmpSignal ConversionAt SFunction Inport1' incorporates:
  //   MATLAB Function: '<S25>/MATLAB Function1'

  rtb_TmpSignalConversionAtSFunct[2] = cos_Psi;

  // MATLAB Function: '<S25>/MATLAB Function1' incorporates:
  //   MATLAB Function: '<S25>/MATLAB Function4'
  //   SignalConversion: '<S29>/TmpSignal ConversionAt SFunction Inport1'

  scale = 1.29246971E-26F;
  absxk = std::abs(rtb_TmpSignalConversionAtSFunct[0]);
  if (absxk > 1.29246971E-26F) {
    q0_q0 = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    q0_q0 = t * t;
  }

  rtb_n_b_d_dt2_g[0] = rtb_Diff_j[0];
  absxk = std::abs(rtb_TmpSignalConversionAtSFunct[1]);
  if (absxk > scale) {
    t = scale / absxk;
    q0_q0 = q0_q0 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q0_q0 += t * t;
  }

  rtb_n_b_d_dt2_g[1] = rtb_Diff_j[1];
  absxk = std::abs(cos_Psi - 9.81F);
  if (absxk > scale) {
    t = scale / absxk;
    q0_q0 = q0_q0 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q0_q0 += t * t;
  }

  rtb_n_b_d_dt2_g[2] = rtb_Diff_j[2] - 9.81F;
  q0_q0 = scale * std::sqrt(q0_q0);

  // MATLAB Function: '<S25>/MATLAB Function4'
  scale = 1.29246971E-26F;
  absxk = std::abs(rtb_Diff_j[0]);
  if (absxk > 1.29246971E-26F) {
    rtb_delta_f = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    rtb_delta_f = t * t;
  }

  absxk = std::abs(rtb_Diff_j[1]);
  if (absxk > scale) {
    t = scale / absxk;
    rtb_delta_f = rtb_delta_f * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    rtb_delta_f += t * t;
  }

  absxk = std::abs(rtb_Diff_j[2] - 9.81F);
  if (absxk > scale) {
    t = scale / absxk;
    rtb_delta_f = rtb_delta_f * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    rtb_delta_f += t * t;
  }

  rtb_delta_f = scale * std::sqrt(rtb_delta_f);

  // Sum: '<S16>/Add' incorporates:
  //   MATLAB Function: '<S16>/MATLAB Function1'
  //   MATLAB Function: '<S25>/MATLAB Function1'
  //   MATLAB Function: '<S25>/MATLAB Function4'
  //   Sum: '<S25>/Add2'

  for (k = 0; k < 3; k++) {
    rtb_Add[k] = ((rtb_TmpSignalConversionAtSFunct[k] - rtConstB.Gain2[k]) /
                  q0_q0 - rtb_n_b_d_dt2_g[k] / rtb_delta_f) + -rtb_M_bg[3 * k +
      2];
  }

  // End of Sum: '<S16>/Add'

  // MATLAB Function: '<S16>/MATLAB Function'
  scale = 1.29246971E-26F;
  absxk = std::abs(rtb_Add[0]);
  if (absxk > 1.29246971E-26F) {
    q0_q0 = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    q0_q0 = t * t;
  }

  absxk = std::abs(rtb_Add[1]);
  if (absxk > scale) {
    t = scale / absxk;
    q0_q0 = q0_q0 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q0_q0 += t * t;
  }

  absxk = std::abs(rtb_Add[2]);
  if (absxk > scale) {
    t = scale / absxk;
    q0_q0 = q0_q0 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q0_q0 += t * t;
  }

  q0_q0 = scale * std::sqrt(q0_q0);
  rtb_Add[0] /= q0_q0;
  rtb_Add[1] /= q0_q0;
  rtb_Add[2] /= q0_q0;

  // End of MATLAB Function: '<S16>/MATLAB Function'

  // MATLAB Function: '<S16>/MATLAB Function4'
  MATLABFunction4(rtb_Add, &rtb_phi_k, &rtb_delta_f);

  // Saturate: '<S10>/Saturation'
  if (rtb_phi_k > 1.04719758F) {
    rtb_phi_k = 1.04719758F;
  } else {
    if (rtb_phi_k < 0.0F) {
      rtb_phi_k = 0.0F;
    }
  }

  // End of Saturate: '<S10>/Saturation'

  // Gain: '<S10>/Gain'
  q0_q0 = 0.95492965F * rtb_phi_k;

  // MATLAB Function: '<S10>/MATLAB Function1'
  rtb_phi_k = q0_q0 * std::sin(rtb_delta_f);
  rtb_delta_f = -q0_q0 * std::cos(rtb_delta_f);

  // Gain: '<S65>/lean_angle_max' incorporates:
  //   MATLAB Function: '<S13>/MATLAB Function1'

  q0_q0 = std::sqrt(rtb_phi_k * rtb_phi_k + rtb_delta_f * rtb_delta_f) *
    1.04719758F;
  rtb_phi_k = std::atan2(rtb_phi_k, -rtb_delta_f);

  // MATLAB Function: '<S65>/lean angles 2 lean vector'
  rtb_delta_f = std::sin(q0_q0);

  // Gain: '<S81>/omega^2' incorporates:
  //   DiscreteIntegrator: '<S81>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S81>/Discrete-Time Integrator y_dt'
  //   Gain: '<S81>/2*d//omega'
  //   MATLAB Function: '<S65>/lean angles 2 lean vector'
  //   Sum: '<S81>/Sum2'
  //   Sum: '<S81>/Sum3'

  rtb_TmpSignalConversionAtSFunct[0] = (rtb_delta_f * std::cos(rtb_phi_k) -
    (0.166666672F * rtDW.DiscreteTimeIntegratory_dt_DSTA[0] +
     rtDW.DiscreteTimeIntegratory_DSTATE[0])) * 144.0F;
  rtb_TmpSignalConversionAtSFunct[1] = (rtb_delta_f * std::sin(rtb_phi_k) -
    (0.166666672F * rtDW.DiscreteTimeIntegratory_dt_DSTA[1] +
     rtDW.DiscreteTimeIntegratory_DSTATE[1])) * 144.0F;
  rtb_TmpSignalConversionAtSFunct[2] = (-std::cos(q0_q0) - (0.166666672F *
    rtDW.DiscreteTimeIntegratory_dt_DSTA[2] +
    rtDW.DiscreteTimeIntegratory_DSTATE[2])) * 144.0F;

  // MATLAB Function: '<S65>/MATLAB Function' incorporates:
  //   DiscreteIntegrator: '<S81>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S81>/Discrete-Time Integrator y_dt'
  //   Inport: '<Root>/measure'

  for (k = 0; k < 3; k++) {
    rtb_Add[k] = rtb_M_bg[k + 6] * rtDW.DiscreteTimeIntegratory_DSTATE[2] +
      (rtb_M_bg[k + 3] * rtDW.DiscreteTimeIntegratory_DSTATE[1] + rtb_M_bg[k] *
       rtDW.DiscreteTimeIntegratory_DSTATE[0]);
  }

  tmp_1[0] = -(rtU.measure.omega_Kb[1] * rtb_Add[2] - rtU.measure.omega_Kb[2] *
               rtb_Add[1]);
  tmp_1[1] = -(rtU.measure.omega_Kb[2] * rtb_Add[0] - rtU.measure.omega_Kb[0] *
               rtb_Add[2]);
  tmp_1[2] = -(rtU.measure.omega_Kb[0] * rtb_Add[1] - rtU.measure.omega_Kb[1] *
               rtb_Add[0]);
  for (k = 0; k < 3; k++) {
    rtb_n_b_d_dt_p[k] = tmp_1[k] + (rtb_M_bg[k + 6] *
      rtDW.DiscreteTimeIntegratory_dt_DSTA[2] + (rtb_M_bg[k + 3] *
      rtDW.DiscreteTimeIntegratory_dt_DSTA[1] + rtb_M_bg[k] *
      rtDW.DiscreteTimeIntegratory_dt_DSTA[0]));
  }

  rtb_Diff_1[0] = -(rtb_Diff[1] * rtb_Add[2] - rtb_Diff[2] * rtb_Add[1]);
  rtb_Diff_1[1] = -(rtb_Diff[2] * rtb_Add[0] - rtb_Diff[0] * rtb_Add[2]);
  rtb_Diff_1[2] = -(rtb_Diff[0] * rtb_Add[1] - rtb_Diff[1] * rtb_Add[0]);
  tmp_1[0] = rtU.measure.omega_Kb[1] * rtb_n_b_d_dt_p[2] - rtU.measure.omega_Kb
    [2] * rtb_n_b_d_dt_p[1];
  tmp_1[1] = rtU.measure.omega_Kb[2] * rtb_n_b_d_dt_p[0] - rtU.measure.omega_Kb
    [0] * rtb_n_b_d_dt_p[2];
  tmp_1[2] = rtU.measure.omega_Kb[0] * rtb_n_b_d_dt_p[1] - rtU.measure.omega_Kb
    [1] * rtb_n_b_d_dt_p[0];
  rtb_lean_angle_max_0[0] = 0.0F;
  rtb_lean_angle_max_0[3] = -rtU.measure.omega_Kb[2];
  rtb_lean_angle_max_0[6] = rtU.measure.omega_Kb[1];
  rtb_lean_angle_max_0[1] = rtU.measure.omega_Kb[2];
  rtb_lean_angle_max_0[4] = 0.0F;
  rtb_lean_angle_max_0[7] = -rtU.measure.omega_Kb[0];
  rtb_lean_angle_max_0[2] = -rtU.measure.omega_Kb[1];
  rtb_lean_angle_max_0[5] = rtU.measure.omega_Kb[0];
  rtb_lean_angle_max_0[8] = 0.0F;
  for (k = 0; k < 3; k++) {
    rtb_z_Kg = 0.0F;
    for (i = 0; i < 3; i++) {
      rtb_ny_du_red_ff_tmp = k + 3 * i;
      rtb_M_bg_0[rtb_ny_du_red_ff_tmp] = 0.0F;
      rtb_ny_du_red_ff_tmp_0 = 3 * i + k;
      rtb_M_bg_0[rtb_ny_du_red_ff_tmp] = rtb_M_bg_0[rtb_ny_du_red_ff_tmp_0] +
        rtb_M_bg[3 * i] * rtb_lean_angle_max_0[3 * k];
      rtb_M_bg_0[rtb_ny_du_red_ff_tmp] = rtb_M_bg[3 * i + 1] *
        rtb_lean_angle_max_0[3 * k + 1] + rtb_M_bg_0[rtb_ny_du_red_ff_tmp_0];
      rtb_M_bg_0[rtb_ny_du_red_ff_tmp] = rtb_M_bg[3 * i + 2] *
        rtb_lean_angle_max_0[3 * k + 2] + rtb_M_bg_0[rtb_ny_du_red_ff_tmp_0];
      rtb_z_Kg += rtb_M_bg_0[rtb_ny_du_red_ff_tmp_0] *
        rtDW.DiscreteTimeIntegratory_dt_DSTA[i];
    }

    rtb_n_b_d_dt2_g[k] = ((rtb_Diff_1[k] - tmp_1[k]) + rtb_z_Kg) + (rtb_M_bg[k +
      6] * rtb_TmpSignalConversionAtSFunct[2] + (rtb_M_bg[k + 3] *
      rtb_TmpSignalConversionAtSFunct[1] + rtb_M_bg[k] *
      rtb_TmpSignalConversionAtSFunct[0]));
  }

  if (rtb_Add[2] > 0.0F) {
    absxk = std::abs(rtb_Add[0]);
    if (absxk > 1.17549435E-38F) {
      std::frexp(absxk, &b_aoffset);
    }

    rtb_n_b_d_dt2_g[0] = -rtb_n_b_d_dt2_g[0] * 0.2F;
    rtb_n_b_d_dt2_g[1] = -rtb_n_b_d_dt2_g[1] * 0.2F;
  }

  // MATLAB Function: '<S50>/MATLAB Function4' incorporates:
  //   Inport: '<Root>/measure'
  //   MATLAB Function: '<S65>/MATLAB Function'

  if (rtb_Add[2] > 1.0F) {
    rtb_delta_f = 0.0F;
  } else {
    scale = 1.29246971E-26F;
    absxk = std::abs(rtU.measure.omega_Kb[0]);
    if (absxk > 1.29246971E-26F) {
      q0_q0 = 1.0F;
      scale = absxk;
    } else {
      t = absxk / 1.29246971E-26F;
      q0_q0 = t * t;
    }

    absxk = std::abs(rtU.measure.omega_Kb[1]);
    if (absxk > scale) {
      t = scale / absxk;
      q0_q0 = q0_q0 * t * t + 1.0F;
      scale = absxk;
    } else {
      t = absxk / scale;
      q0_q0 += t * t;
    }

    absxk = std::abs(rtU.measure.omega_Kb[2]);
    if (absxk > scale) {
      t = scale / absxk;
      q0_q0 = q0_q0 * t * t + 1.0F;
      scale = absxk;
    } else {
      t = absxk / scale;
      q0_q0 += t * t;
    }

    q0_q0 = scale * std::sqrt(q0_q0);
    rtb_delta_f = rtb_Add[2] * (rtU.measure.omega_Kb[2] * rtU.measure.omega_Kb[2]
      / (2.22044605E-16F + q0_q0)) + (rtb_Add[1] * (rtU.measure.omega_Kb[2] *
      rtU.measure.omega_Kb[1] / (2.22044605E-16F + q0_q0)) + rtb_Add[0] *
      (rtU.measure.omega_Kb[2] * rtU.measure.omega_Kb[0] / (2.22044605E-16F +
      q0_q0)));
  }

  // End of MATLAB Function: '<S50>/MATLAB Function4'

  // MATLAB Function: '<S52>/MATLAB Function2' incorporates:
  //   Abs: '<S50>/Abs1'
  //   Constant: '<S50>/Constant2'
  //   Constant: '<S52>/Constant5'
  //   Gain: '<S50>/Gain'
  //   MATLAB Function: '<S50>/MATLAB Function'
  //   Sum: '<S50>/Add1'

  memcpy(&rtb_Wv[0], &rtConstP.Constant5_Value[0], sizeof(real32_T) << 4U);
  rtb_Wv[15] = (real32_T)((std::tanh((std::abs(rtb_delta_f) - 8.0F) * 0.4F) *
    0.5F + 0.5F) * 300.0F + 5.0);

  // DiscreteIntegrator: '<S82>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S81>/Discrete-Time Integrator y'

  if (rtDW.DiscreteTimeIntegratory_IC_LOAD != 0) {
    rtDW.DiscreteTimeIntegratory_DSTAT_f[0] =
      rtDW.DiscreteTimeIntegratory_DSTATE[0];
    rtDW.DiscreteTimeIntegratory_DSTAT_f[1] =
      rtDW.DiscreteTimeIntegratory_DSTATE[1];
    rtDW.DiscreteTimeIntegratory_DSTAT_f[2] =
      rtDW.DiscreteTimeIntegratory_DSTATE[2];
  }

  // DiscreteIntegrator: '<S83>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S81>/Discrete-Time Integrator y_dt'

  if (rtDW.DiscreteTimeIntegratory_IC_LO_d != 0) {
    rtDW.DiscreteTimeIntegratory_DSTAT_d[0] =
      rtDW.DiscreteTimeIntegratory_dt_DSTA[0];
    rtDW.DiscreteTimeIntegratory_DSTAT_d[1] =
      rtDW.DiscreteTimeIntegratory_dt_DSTA[1];
    rtDW.DiscreteTimeIntegratory_DSTAT_d[2] =
      rtDW.DiscreteTimeIntegratory_dt_DSTA[2];
  }

  // DiscreteIntegrator: '<S84>/Discrete-Time Integrator y'
  if (rtDW.DiscreteTimeIntegratory_IC_LO_p != 0) {
    rtDW.DiscreteTimeIntegratory_DSTAT_c[0] = rtb_TmpSignalConversionAtSFunct[0];
    rtDW.DiscreteTimeIntegratory_DSTAT_c[1] = rtb_TmpSignalConversionAtSFunct[1];
    rtDW.DiscreteTimeIntegratory_DSTAT_c[2] = rtb_TmpSignalConversionAtSFunct[2];
  }

  // MATLAB Function: '<S65>/MATLAB Function1' incorporates:
  //   DiscreteIntegrator: '<S82>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S83>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S84>/Discrete-Time Integrator y'
  //   Inport: '<Root>/measure'

  for (k = 0; k < 3; k++) {
    rtb_Add[k] = rtb_M_bg[k + 6] * rtDW.DiscreteTimeIntegratory_DSTAT_f[2] +
      (rtb_M_bg[k + 3] * rtDW.DiscreteTimeIntegratory_DSTAT_f[1] + rtb_M_bg[k] *
       rtDW.DiscreteTimeIntegratory_DSTAT_f[0]);
  }

  tmp_1[0] = -(rtU.measure.omega_Kb[1] * rtb_Add[2] - rtU.measure.omega_Kb[2] *
               rtb_Add[1]);
  tmp_1[1] = -(rtU.measure.omega_Kb[2] * rtb_Add[0] - rtU.measure.omega_Kb[0] *
               rtb_Add[2]);
  tmp_1[2] = -(rtU.measure.omega_Kb[0] * rtb_Add[1] - rtU.measure.omega_Kb[1] *
               rtb_Add[0]);
  for (k = 0; k < 3; k++) {
    rtb_n_b_d_dt_p[k] = tmp_1[k] + (rtb_M_bg[k + 6] *
      rtDW.DiscreteTimeIntegratory_DSTAT_d[2] + (rtb_M_bg[k + 3] *
      rtDW.DiscreteTimeIntegratory_DSTAT_d[1] + rtb_M_bg[k] *
      rtDW.DiscreteTimeIntegratory_DSTAT_d[0]));
  }

  rtb_Diff_1[0] = -(rtb_Diff[1] * rtb_Add[2] - rtb_Diff[2] * rtb_Add[1]);
  rtb_Diff_1[1] = -(rtb_Diff[2] * rtb_Add[0] - rtb_Diff[0] * rtb_Add[2]);
  rtb_Diff_1[2] = -(rtb_Diff[0] * rtb_Add[1] - rtb_Diff[1] * rtb_Add[0]);
  tmp_1[0] = rtU.measure.omega_Kb[1] * rtb_n_b_d_dt_p[2] - rtU.measure.omega_Kb
    [2] * rtb_n_b_d_dt_p[1];
  tmp_1[1] = rtU.measure.omega_Kb[2] * rtb_n_b_d_dt_p[0] - rtU.measure.omega_Kb
    [0] * rtb_n_b_d_dt_p[2];
  tmp_1[2] = rtU.measure.omega_Kb[0] * rtb_n_b_d_dt_p[1] - rtU.measure.omega_Kb
    [1] * rtb_n_b_d_dt_p[0];
  rtb_lean_angle_max_0[0] = 0.0F;
  rtb_lean_angle_max_0[3] = -rtU.measure.omega_Kb[2];
  rtb_lean_angle_max_0[6] = rtU.measure.omega_Kb[1];
  rtb_lean_angle_max_0[1] = rtU.measure.omega_Kb[2];
  rtb_lean_angle_max_0[4] = 0.0F;
  rtb_lean_angle_max_0[7] = -rtU.measure.omega_Kb[0];
  rtb_lean_angle_max_0[2] = -rtU.measure.omega_Kb[1];
  rtb_lean_angle_max_0[5] = rtU.measure.omega_Kb[0];
  rtb_lean_angle_max_0[8] = 0.0F;
  for (k = 0; k < 3; k++) {
    rtb_z_Kg = 0.0F;
    for (i = 0; i < 3; i++) {
      rtb_ny_du_red_ff_tmp = k + 3 * i;
      rtb_M_bg_0[rtb_ny_du_red_ff_tmp] = 0.0F;
      rtb_ny_du_red_ff_tmp_0 = 3 * i + k;
      rtb_M_bg_0[rtb_ny_du_red_ff_tmp] = rtb_M_bg_0[rtb_ny_du_red_ff_tmp_0] +
        rtb_M_bg[3 * i] * rtb_lean_angle_max_0[3 * k];
      rtb_M_bg_0[rtb_ny_du_red_ff_tmp] = rtb_M_bg[3 * i + 1] *
        rtb_lean_angle_max_0[3 * k + 1] + rtb_M_bg_0[rtb_ny_du_red_ff_tmp_0];
      rtb_M_bg_0[rtb_ny_du_red_ff_tmp] = rtb_M_bg[3 * i + 2] *
        rtb_lean_angle_max_0[3 * k + 2] + rtb_M_bg_0[rtb_ny_du_red_ff_tmp_0];
      rtb_z_Kg += rtb_M_bg_0[rtb_ny_du_red_ff_tmp_0] *
        rtDW.DiscreteTimeIntegratory_DSTAT_d[i];
    }

    rtb_n_b_d_dt2[k] = ((rtb_Diff_1[k] - tmp_1[k]) + rtb_z_Kg) + (rtb_M_bg[k + 6]
      * rtDW.DiscreteTimeIntegratory_DSTAT_c[2] + (rtb_M_bg[k + 3] *
      rtDW.DiscreteTimeIntegratory_DSTAT_c[1] + rtb_M_bg[k] *
      rtDW.DiscreteTimeIntegratory_DSTAT_c[0]));
  }

  if (rtb_Add[2] > 0.0F) {
    rtb_Add[2] = 0.0F;
    rtb_phi_k = std::abs(rtb_Add[0]);
    if (rtb_phi_k <= 1.17549435E-38F) {
      rtb_delta_f = 1.4013E-45F;
    } else {
      std::frexp(rtb_phi_k, &exponent);
      rtb_delta_f = std::ldexp(1.0F, exponent - 24);
    }

    scale = 1.29246971E-26F;
    if (rtb_phi_k > 1.29246971E-26F) {
      q0_q0 = 1.0F;
      scale = rtb_phi_k;
    } else {
      t = rtb_phi_k / 1.29246971E-26F;
      q0_q0 = t * t;
    }

    rtb_phi_k = std::abs(rtb_Add[1]);
    if (rtb_phi_k > scale) {
      t = scale / rtb_phi_k;
      q0_q0 = q0_q0 * t * t + 1.0F;
      scale = rtb_phi_k;
    } else {
      t = rtb_phi_k / scale;
      q0_q0 += t * t;
    }

    q0_q0 = scale * std::sqrt(q0_q0);
    rtb_delta_f += q0_q0;
    rtb_Add[0] += (rtb_Add[0] / rtb_delta_f - rtb_Add[0]) * 2.0F;
    rtb_Add[1] += (rtb_Add[1] / rtb_delta_f - rtb_Add[1]) * 2.0F;
    if (norm_c(rtb_Add) < 0.9) {
      rtb_Add[0] = -1.0F;
      rtb_delta_f = norm_c(rtb_Add);
      rtb_Add[0] = -1.0F / rtb_delta_f;
      rtb_Add[1] /= rtb_delta_f;
    }

    rtb_n_b_d_dt_p[0] = -rtb_n_b_d_dt_p[0] * 0.2F;
    rtb_n_b_d_dt2[0] = -rtb_n_b_d_dt2[0] * 0.2F;
    rtb_n_b_d_dt_p[1] = -rtb_n_b_d_dt_p[1] * 0.2F;
    rtb_n_b_d_dt2[1] = -rtb_n_b_d_dt2[1] * 0.2F;
  }

  // End of MATLAB Function: '<S65>/MATLAB Function1'

  // DiscreteIntegrator: '<S65>/Discrete-Time Integrator2'
  if (rtDW.DiscreteTimeIntegrator2_IC_LOAD != 0) {
    // MATLAB Function: '<Root>/Rotations matrix to Euler angles'
    rtDW.DiscreteTimeIntegrator2_DSTATE = std::atan2(rtb_M_bg[3], rtb_M_bg[0]);
  }

  // MATLAB Function: '<S58>/wrap angle' incorporates:
  //   DiscreteIntegrator: '<S65>/Discrete-Time Integrator2'

  wrapangle(rtDW.DiscreteTimeIntegrator2_DSTATE, &rtb_delta_f);

  // MATLAB Function: '<S59>/DCM to quaternions'
  DCMtoquaternions(rtb_M_bg, rtb_q_bg);

  // MATLAB Function: '<S59>/MATLAB Function1'
  quatNormalize(rtb_q_bg, q_yaw);
  absxk = (q_yaw[1] * q_yaw[3] + q_yaw[0] * q_yaw[2]) * 2.0F;
  M_bg_idx_8 = ((q_yaw[0] * q_yaw[0] - q_yaw[1] * q_yaw[1]) - q_yaw[2] * q_yaw[2])
    + q_yaw[3] * q_yaw[3];
  if (1.0F <= M_bg_idx_8) {
    M_bg_idx_8 = 1.0F;
  }

  if (-1.0F >= M_bg_idx_8) {
    M_bg_idx_8 = -1.0F;
  }

  rtb_phi_k = std::acos(M_bg_idx_8);
  q0_q0 = std::sin(rtb_phi_k);
  q0_q0 = q0_q0 * q0_q0 - absxk * absxk;
  if ((q_yaw[2] * q_yaw[3] - q_yaw[0] * q_yaw[1]) * 2.0F >= 0.0F) {
    k = -1;
  } else {
    k = 1;
  }

  if (0.0F >= q0_q0) {
    q0_q0 = 0.0F;
  }

  q0_q0 = std::atan2((real32_T)k * std::sqrt(q0_q0), -absxk);
  scale = std::sin(rtb_phi_k / 2.0F);
  rtb_q_red[0] = std::cos(rtb_phi_k / 2.0F);
  rtb_q_red[1] = std::sin(q0_q0) * scale;
  rtb_q_red[2] = -std::cos(q0_q0) * scale;
  q0_q0 = (rtb_q_red[0] * rtb_q_red[0] + rtb_q_red[1] * rtb_q_red[1]) +
    rtb_q_red[2] * rtb_q_red[2];
  if (2.22044605E-16F < q0_q0) {
    rtb_phi_k = q0_q0;
  } else {
    rtb_phi_k = 2.22044605E-16F;
  }

  q_yaw[0] = rtb_q_red[0] / rtb_phi_k;
  q_yaw[1] = -rtb_q_red[1] / rtb_phi_k;
  q_yaw[2] = -rtb_q_red[2] / rtb_phi_k;
  rtb_q_red[0] = (q_yaw[0] * rtb_q_bg[0] - q_yaw[1] * rtb_q_bg[1]) - q_yaw[2] *
    rtb_q_bg[2];
  rtb_q_red[1] = (q_yaw[0] * rtb_q_bg[1] + rtb_q_bg[0] * q_yaw[1]) + q_yaw[2] *
    rtb_q_bg[3];
  rtb_q_red[2] = (q_yaw[0] * rtb_q_bg[2] + rtb_q_bg[0] * q_yaw[2]) + (0.0F -
    q_yaw[1] * rtb_q_bg[3]);
  rtb_q_red[3] = (q_yaw[1] * rtb_q_bg[2] - q_yaw[2] * rtb_q_bg[1]) + q_yaw[0] *
    rtb_q_bg[3];
  quatNormalize(rtb_q_red, q_yaw);
  if (q_yaw[3] < 0.0F) {
    rtb_phi_k = -1.0F;
  } else if (q_yaw[3] > 0.0F) {
    rtb_phi_k = 1.0F;
  } else {
    rtb_phi_k = q_yaw[3];
  }

  rtb_ny_du_red_ff_tmp_0 = 0;
  if (std::abs(rtb_phi_k) < 0.5F) {
    rtb_ny_du_red_ff_tmp_0 = 1;
  }

  if (0 <= rtb_ny_du_red_ff_tmp_0 - 1) {
    rtb_phi_k = 1.0F;
  }

  rtb_phi_k *= q_yaw[0];
  if (-1.0F >= rtb_phi_k) {
    rtb_phi_k = -1.0F;
  }

  if (1.0F <= rtb_phi_k) {
    rtb_phi_k = 1.0F;
  }

  // MATLAB Function: '<S58>/wrap angle1' incorporates:
  //   MATLAB Function: '<S59>/MATLAB Function1'

  wrapangle(2.0F * std::acos(rtb_phi_k), &q0_q0);

  // MATLAB Function: '<S58>/angle error'
  t = rtb_delta_f - q0_q0;
  if (t > 3.1415926535897931) {
    t -= 6.28318548F;
  } else {
    if (t < -3.1415926535897931) {
      t += 6.28318548F;
    }
  }

  // End of MATLAB Function: '<S58>/angle error'

  // Gain: '<S80>/1//T' incorporates:
  //   DiscreteIntegrator: '<S80>/Discrete-Time Integrator'
  //   Gain: '<S65>/r_max'
  //   Inport: '<Root>/cmd'
  //   Sum: '<S80>/Sum2'

  rtb_delta_f = (5.23598766F * rtU.cmd.yaw -
                 rtDW.DiscreteTimeIntegrator_DSTATE_o) * 10.0F;

  // Sum: '<S58>/error1 9'
  absxk = rtb_delta_f - rtb_Diff[2];

  // DiscreteIntegrator: '<S90>/Discrete-Time Integrator3' incorporates:
  //   Inport: '<Root>/measure'

  if (rtDW.DiscreteTimeIntegrator3_IC_LOAD != 0) {
    rtDW.DiscreteTimeIntegrator3_DSTATE = rtU.measure.s_Kg[2];
  }

  // Sum: '<S85>/error1 3' incorporates:
  //   DiscreteIntegrator: '<S90>/Discrete-Time Integrator3'
  //   Inport: '<Root>/measure'

  M_bg_idx_8 = rtDW.DiscreteTimeIntegrator3_DSTATE - rtU.measure.s_Kg[2];

  // Sum: '<S85>/error1 1' incorporates:
  //   DiscreteIntegrator: '<S92>/Discrete-Time Integrator'
  //   Inport: '<Root>/measure'

  q0_q2 = rtDW.DiscreteTimeIntegrator_DSTATE_d - rtU.measure.V_Kg[2];

  // SampleTimeMath: '<S41>/TSamp' incorporates:
  //   Inport: '<Root>/measure'
  //
  //  About '<S41>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  rtb_phi_k = rtU.measure.V_Kg[2] * 400.0F;

  // Sum: '<S41>/Diff' incorporates:
  //   UnitDelay: '<S41>/UD'
  //
  //  Block description for '<S41>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S41>/UD':
  //
  //   Store in Global RAM

  q0_q0 = rtb_phi_k - rtDW.UD_DSTATE_n;

  // Sum: '<S85>/error1 2'
  scale = cos_Psi - q0_q0;

  // Outport: '<Root>/logs' incorporates:
  //   Sum: '<S58>/error1 4'

  rtY.logs[0] = rtb_Add[0];
  rtY.logs[1] = rtb_Add[1];
  rtY.logs[2] = absxk;
  rtY.logs[3] = t;
  rtY.logs[4] = M_bg_idx_8;
  rtY.logs[5] = q0_q2;
  rtY.logs[6] = scale;
  for (i = 0; i < 6; i++) {
    rtY.logs[i + 7] = rtb_Add_gb[i];
  }

  rtY.logs[13] = 0.0F;
  rtY.logs[14] = 0.0F;

  // End of Outport: '<Root>/logs'

  // DiscreteIntegrator: '<S26>/Discrete-Time Integrator' incorporates:
  //   Inport: '<Root>/measure'

  if (rtDW.DiscreteTimeIntegrator_IC_LOADI != 0) {
    rtDW.DiscreteTimeIntegrator_DSTATE_a[0] = rtU.measure.s_Kg[0];
    rtDW.DiscreteTimeIntegrator_DSTATE_a[1] = rtU.measure.s_Kg[1];
  }

  // Sum: '<S39>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S26>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<S38>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<S39>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S39>/Discrete-Time Integrator y_dt'
  //   Gain: '<S39>/2*d//omega'
  //   Saturate: '<S38>/Saturation'
  //   Sum: '<S39>/Sum3'

  rtb_Add_gb[0] = rtDW.DiscreteTimeIntegrator_DSTATE_a[0] - (0.198645547F *
    rtDW.DiscreteTimeIntegratory_dt_DS_f[0] +
    rtDW.DiscreteTimeIntegratory_DSTAT_j[0]);
  rtb_Add_gb[2] = rtDW.DiscreteTimeIntegrator_DSTATE_h[0] - (0.198645547F *
    rtDW.DiscreteTimeIntegratory_dt_DS_f[2] +
    rtDW.DiscreteTimeIntegratory_DSTAT_j[2]);
  rtb_Add_gb[4] = y - (0.198645547F * rtDW.DiscreteTimeIntegratory_dt_DS_f[4] +
                       rtDW.DiscreteTimeIntegratory_DSTAT_j[4]);
  rtb_Add_gb[1] = rtDW.DiscreteTimeIntegrator_DSTATE_a[1] - (0.198645547F *
    rtDW.DiscreteTimeIntegratory_dt_DS_f[1] +
    rtDW.DiscreteTimeIntegratory_DSTAT_j[1]);
  rtb_Add_gb[3] = rtDW.DiscreteTimeIntegrator_DSTATE_h[1] - (0.198645547F *
    rtDW.DiscreteTimeIntegratory_dt_DS_f[3] +
    rtDW.DiscreteTimeIntegratory_DSTAT_j[3]);
  rtb_Add_gb[5] = y_0 - (0.198645547F * rtDW.DiscreteTimeIntegratory_dt_DS_f[5]
    + rtDW.DiscreteTimeIntegratory_DSTAT_j[5]);

  // Gain: '<S87>/Gain' incorporates:
  //   SignalConversion: '<S87>/TmpSignal ConversionAtGainInport1'

  rtb_z_Kg = (50.0F * M_bg_idx_8 + 28.1578F * q0_q2) + 0.6786F * scale;

  // MATLAB Function: '<S49>/DCM2lean'
  if (1.0F > rtb_M_bg[8]) {
    M_bg_idx_8 = rtb_M_bg[8];
  } else {
    M_bg_idx_8 = 1.0F;
  }

  if (-1.0F >= M_bg_idx_8) {
    M_bg_idx_8 = -1.0F;
  }

  // MATLAB Function: '<S49>/MATLAB Function' incorporates:
  //   Gain: '<S87>/Gain'
  //   MATLAB Function: '<S49>/DCM2lean'
  //   Sum: '<S46>/Add1'

  M_bg_idx_8 = std::cos(std::acos(M_bg_idx_8));
  if (M_bg_idx_8 > 0.1) {
    q0_q0 = ((cos_Psi + rtb_z_Kg) - q0_q0) / M_bg_idx_8;
  } else {
    q0_q0 = (cos_Psi + rtb_z_Kg) - q0_q0;
  }

  // End of MATLAB Function: '<S49>/MATLAB Function'
  for (exponent = 0; exponent < 4; exponent++) {
    // Product: '<S42>/MatrixMultiply2' incorporates:
    //   UnitDelay: '<S42>/Unit Delay1'

    M_bg_idx_8 = rtb_G2[exponent + 12] * rtDW.UnitDelay1_DSTATE[3] +
      (rtb_G2[exponent + 8] * rtDW.UnitDelay1_DSTATE[2] + (rtb_G2[exponent + 4] *
        rtDW.UnitDelay1_DSTATE[1] + rtb_G2[exponent] * rtDW.UnitDelay1_DSTATE[0]));

    // MATLAB Function: '<S52>/MATLAB Function'
    rtb_q_cmd_red_c[exponent] = 0.0F;

    // Gain: '<S42>/Gain'
    q_yaw[exponent] = 0.0F;

    // Lookup_n-D: '<S42>/1-D Lookup Table2' incorporates:
    //   DiscreteIntegrator: '<S93>/Discrete-Time Integrator'
    //   MATLAB Function: '<S52>/MATLAB Function1'

    rtb_uDLookupTable2[exponent] = ((0.1F -
      rtDW.DiscreteTimeIntegrator_DSTATE[exponent]) + (1.0F -
      rtDW.DiscreteTimeIntegrator_DSTATE[exponent])) * 0.5F;

    // MATLAB Function: '<S52>/MATLAB Function1' incorporates:
    //   DiscreteIntegrator: '<S93>/Discrete-Time Integrator'

    rtb_q_red[exponent] = 0.1F - rtDW.DiscreteTimeIntegrator_DSTATE[exponent];
    rtb_umax[exponent] = 1.0F - rtDW.DiscreteTimeIntegrator_DSTATE[exponent];

    // Product: '<S42>/MatrixMultiply2'
    rtb_q_bg[exponent] = M_bg_idx_8;
  }

  // MATLAB Function: '<S52>/MATLAB Function' incorporates:
  //   Constant: '<S52>/Constant1'
  //   Gain: '<Root>/Gain5'
  //   Inport: '<Root>/cmd'
  //   Lookup_n-D: '<S42>/1-D Lookup Table1'
  //   MATLAB Function: '<S13>/MATLAB Function'
  //   Sum: '<S52>/Add1'

  rtb_z_Kg = std::sqrt(look1_iflf_binlx(0.5F * -rtU.cmd.thr + 0.5F,
    rtConstP.uDLookupTable1_bp01Data, rtConstP.uDLookupTable1_tableData, 3U) +
                       1000.0F);
  for (k = 0; k < 16; k++) {
    rtb_Wv[k] *= rtb_z_Kg;
  }

  // MATLAB Function: '<S51>/MATLAB Function'
  for (k = 0; k < 4; k++) {
    for (i = 0; i < 4; i++) {
      rtb_ny_du_red_ff_tmp = k << 2;
      rtb_ny_du_red_ff_0[i + rtb_ny_du_red_ff_tmp] = (((d[rtb_ny_du_red_ff_tmp +
        1] * rtb_ny_du_red_ff[i + 4] + d[rtb_ny_du_red_ff_tmp] *
        rtb_ny_du_red_ff[i]) + d[rtb_ny_du_red_ff_tmp + 2] * rtb_ny_du_red_ff[i
        + 8]) + d[rtb_ny_du_red_ff_tmp + 3] * rtb_ny_du_red_ff[i + 12]) +
        rtb_G2[rtb_ny_du_red_ff_tmp + i];
    }
  }

  // MATLAB Function: '<S52>/MATLAB Function' incorporates:
  //   Constant: '<S52>/Constant6'

  for (k = 0; k < 4; k++) {
    for (i = 0; i < 4; i++) {
      exponent = k << 2;
      b_aoffset = i + exponent;
      rtb_ny_du_red_ff[b_aoffset] = 0.0F;
      rtb_ny_du_red_ff_tmp = exponent + i;
      rtb_ny_du_red_ff[b_aoffset] = rtb_ny_du_red_ff[rtb_ny_du_red_ff_tmp] +
        rtb_ny_du_red_ff_0[exponent] * rtb_Wv[i];
      rtb_ny_du_red_ff[b_aoffset] = rtb_ny_du_red_ff_0[exponent + 1] * rtb_Wv[i
        + 4] + rtb_ny_du_red_ff[rtb_ny_du_red_ff_tmp];
      rtb_ny_du_red_ff[b_aoffset] = rtb_ny_du_red_ff_0[exponent + 2] * rtb_Wv[i
        + 8] + rtb_ny_du_red_ff[rtb_ny_du_red_ff_tmp];
      rtb_ny_du_red_ff[b_aoffset] = rtb_ny_du_red_ff_0[exponent + 3] * rtb_Wv[i
        + 12] + rtb_ny_du_red_ff[rtb_ny_du_red_ff_tmp];
    }
  }

  for (k = 0; k < 4; k++) {
    exponent = k << 2;
    i = k << 3;
    A[i] = rtb_ny_du_red_ff[exponent];
    A[4 + i] = rtConstP.Constant6_Value[exponent];
    b_aoffset = exponent + 1;
    A[1 + i] = rtb_ny_du_red_ff[b_aoffset];
    A[5 + i] = rtConstP.Constant6_Value[b_aoffset];
    b_aoffset = exponent + 2;
    A[2 + i] = rtb_ny_du_red_ff[b_aoffset];
    A[6 + i] = rtConstP.Constant6_Value[b_aoffset];
    exponent += 3;
    A[3 + i] = rtb_ny_du_red_ff[exponent];
    A[7 + i] = rtConstP.Constant6_Value[exponent];
  }

  // SignalConversion: '<S61>/TmpSignal ConversionAtGainInport1' incorporates:
  //   DiscreteIntegrator: '<S80>/Discrete-Time Integrator'
  //   Inport: '<Root>/measure'
  //   Sum: '<S58>/error1 4'
  //   Sum: '<S58>/error1 5'
  //   Sum: '<S58>/error1 6'
  //   Sum: '<S58>/error1 8'

  rtb_lean_angle_max_0[2] = t;
  rtb_lean_angle_max_0[5] = rtDW.DiscreteTimeIntegrator_DSTATE_o -
    rtU.measure.omega_Kb[2];
  rtb_lean_angle_max_0[8] = absxk;
  rtb_lean_angle_max_0[0] = rtb_Add[0];
  rtb_lean_angle_max_0[3] = rtb_n_b_d_dt_p[0];
  rtb_lean_angle_max_0[6] = rtb_n_b_d_dt2[0];
  rtb_lean_angle_max_0[1] = rtb_Add[1];
  rtb_lean_angle_max_0[4] = rtb_n_b_d_dt_p[1];
  rtb_lean_angle_max_0[7] = rtb_n_b_d_dt2[1];

  // Gain: '<S61>/Gain' incorporates:
  //   SignalConversion: '<S61>/TmpSignal ConversionAtGainInport1'

  for (k = 0; k < 3; k++) {
    tmp_1[k] = 0.0F;
    for (i = 0; i < 9; i++) {
      tmp_1[k] += rtConstP.Gain_Gain_o0[3 * i + k] * rtb_lean_angle_max_0[i];
    }
  }

  // End of Gain: '<S61>/Gain'

  // Sum: '<S42>/Add2' incorporates:
  //   Sum: '<S45>/Add2'

  absxk = (rtb_n_b_d_dt2_g[0] + tmp_1[0]) + rtb_q_bg[0];

  // MATLAB Function: '<S52>/MATLAB Function1' incorporates:
  //   Constant: '<S52>/Constant7'
  //   DiscreteIntegrator: '<S93>/Discrete-Time Integrator'
  //   Sum: '<S52>/Add'

  t = (0.1F + q_yaw[0]) - rtDW.DiscreteTimeIntegrator_DSTATE[0];

  // Sum: '<S42>/Add2' incorporates:
  //   Sum: '<S45>/Add2'

  M_bg_idx_8 = (rtb_n_b_d_dt2_g[1] + tmp_1[1]) + rtb_q_bg[1];

  // MATLAB Function: '<S52>/MATLAB Function1' incorporates:
  //   Constant: '<S52>/Constant7'
  //   DiscreteIntegrator: '<S93>/Discrete-Time Integrator'
  //   Sum: '<S52>/Add'

  scale = (0.1F + q_yaw[1]) - rtDW.DiscreteTimeIntegrator_DSTATE[1];

  // Sum: '<S42>/Add2' incorporates:
  //   Sum: '<S45>/Add2'

  q0_q2 = ((rtb_delta_f + tmp_1[2]) - rtb_Diff[2]) + rtb_q_bg[2];

  // MATLAB Function: '<S52>/MATLAB Function1' incorporates:
  //   Constant: '<S52>/Constant7'
  //   DiscreteIntegrator: '<S93>/Discrete-Time Integrator'
  //   Sum: '<S52>/Add'

  tmp_3 = (0.1F + q_yaw[2]) - rtDW.DiscreteTimeIntegrator_DSTATE[2];

  // Sum: '<S42>/Add2'
  q0_q0 += rtb_q_bg[3];

  // MATLAB Function: '<S52>/MATLAB Function1' incorporates:
  //   Constant: '<S52>/Constant7'
  //   DiscreteIntegrator: '<S93>/Discrete-Time Integrator'
  //   Sum: '<S52>/Add'

  tmp_2 = (0.1F + q_yaw[3]) - rtDW.DiscreteTimeIntegrator_DSTATE[3];

  // MATLAB Function: '<S52>/MATLAB Function' incorporates:
  //   Constant: '<S52>/Constant6'

  for (k = 0; k < 4; k++) {
    rtb_Wv_1 = rtb_Wv[k + 12] * q0_q0 + (rtb_Wv[k + 8] * q0_q2 + (rtb_Wv[k + 4] *
      M_bg_idx_8 + rtb_Wv[k] * absxk));
    rtb_z_Kg = rtConstP.Constant6_Value[k + 12] * tmp_2 +
      (rtConstP.Constant6_Value[k + 8] * tmp_3 + (rtConstP.Constant6_Value[k + 4]
        * scale + rtConstP.Constant6_Value[k] * t));
    rtb_Wv_0[k] = rtb_Wv_1;
    rtb_Wv_0[k + 4] = rtb_z_Kg;
  }

  for (k = 0; k < 8; k++) {
    q0_q0 = A[k + 24] * rtb_uDLookupTable2[3] + (A[k + 16] * rtb_uDLookupTable2
      [2] + (A[k + 8] * rtb_uDLookupTable2[1] + A[k] * rtb_uDLookupTable2[0]));
    d_0[k] = rtb_Wv_0[k] - q0_q0;
  }

  i_free[0] = true;
  i_free[1] = true;
  i_free[2] = true;
  i_free[3] = true;
  exponent = 0;
  exitg1 = false;
  while ((!exitg1) && (exponent <= 99)) {
    rtb_ny_du_red_ff_tmp_0 = 0;
    if (i_free[0]) {
      rtb_ny_du_red_ff_tmp_0 = 1;
    }

    if (i_free[1]) {
      rtb_ny_du_red_ff_tmp_0++;
    }

    if (i_free[2]) {
      rtb_ny_du_red_ff_tmp_0++;
    }

    if (i_free[3]) {
      rtb_ny_du_red_ff_tmp_0++;
    }

    rtb_ny_du_red_ff_tmp = rtb_ny_du_red_ff_tmp_0;
    rtb_ny_du_red_ff_tmp_0 = 0;
    if (i_free[0]) {
      f_data[0] = 1;
      rtb_ny_du_red_ff_tmp_0 = 1;
    }

    if (i_free[1]) {
      f_data[rtb_ny_du_red_ff_tmp_0] = 2;
      rtb_ny_du_red_ff_tmp_0++;
    }

    if (i_free[2]) {
      f_data[rtb_ny_du_red_ff_tmp_0] = 3;
      rtb_ny_du_red_ff_tmp_0++;
    }

    if (i_free[3]) {
      f_data[rtb_ny_du_red_ff_tmp_0] = 4;
    }

    A_free_size[0] = 8;
    A_free_size[1] = rtb_ny_du_red_ff_tmp;
    for (k = 0; k < rtb_ny_du_red_ff_tmp; k++) {
      for (i = 0; i < 8; i++) {
        A_free_data[i + (k << 3)] = A[((f_data[k] - 1) << 3) + i];
      }
    }

    mldivide(A_free_data, A_free_size, d_0, p_free_data, &rtb_ny_du_red_ff_tmp_1);
    rtb_ny_du_red_ff_tmp_0 = 0;
    p_0 = 0.0;
    if (i_free[0]) {
      p_0 = p_free_data[0];
      rtb_ny_du_red_ff_tmp_0 = 1;
    }

    rtb_q_bg[0] = rtb_uDLookupTable2[0] + (real32_T)p_0;
    p[0] = p_0;
    p_0 = 0.0;
    if (i_free[1]) {
      p_0 = p_free_data[rtb_ny_du_red_ff_tmp_0];
      rtb_ny_du_red_ff_tmp_0++;
    }

    rtb_q_bg[1] = rtb_uDLookupTable2[1] + (real32_T)p_0;
    p[1] = p_0;
    p_0 = 0.0;
    if (i_free[2]) {
      p_0 = p_free_data[rtb_ny_du_red_ff_tmp_0];
      rtb_ny_du_red_ff_tmp_0++;
    }

    rtb_q_bg[2] = rtb_uDLookupTable2[2] + (real32_T)p_0;
    p[2] = p_0;
    p_0 = 0.0;
    if (i_free[3]) {
      p_0 = p_free_data[rtb_ny_du_red_ff_tmp_0];
    }

    rtb_q_bg[3] = rtb_uDLookupTable2[3] + (real32_T)p_0;
    p[3] = p_0;
    rtb_ny_du_red_ff_tmp_0 = 0;
    if (i_free[0]) {
      rtb_ny_du_red_ff_tmp_0 = 1;
    }

    if (i_free[1]) {
      rtb_ny_du_red_ff_tmp_0++;
    }

    if (i_free[2]) {
      rtb_ny_du_red_ff_tmp_0++;
    }

    if (i_free[3]) {
      rtb_ny_du_red_ff_tmp_0++;
    }

    i = rtb_ny_du_red_ff_tmp_0;
    rtb_ny_du_red_ff_tmp_0 = 0;
    if (i_free[0]) {
      g_data[0] = 1;
      rtb_ny_du_red_ff_tmp_0 = 1;
    }

    rtb_q_bg_0[0] = ((rtb_q_bg[0] < rtb_q_red[0]) || (rtb_q_bg[0] > rtb_umax[0]));
    if (i_free[1]) {
      g_data[rtb_ny_du_red_ff_tmp_0] = 2;
      rtb_ny_du_red_ff_tmp_0++;
    }

    rtb_q_bg_0[1] = ((rtb_q_bg[1] < rtb_q_red[1]) || (rtb_q_bg[1] > rtb_umax[1]));
    if (i_free[2]) {
      g_data[rtb_ny_du_red_ff_tmp_0] = 3;
      rtb_ny_du_red_ff_tmp_0++;
    }

    rtb_q_bg_0[2] = ((rtb_q_bg[2] < rtb_q_red[2]) || (rtb_q_bg[2] > rtb_umax[2]));
    if (i_free[3]) {
      g_data[rtb_ny_du_red_ff_tmp_0] = 4;
    }

    rtb_q_bg_0[3] = ((rtb_q_bg[3] < rtb_q_red[3]) || (rtb_q_bg[3] > rtb_umax[3]));
    for (k = 0; k < i; k++) {
      rtb_q_bg_data[k] = rtb_q_bg_0[g_data[k] - 1];
    }

    if (!any(rtb_q_bg_data, &i)) {
      rtb_uDLookupTable2[0] = rtb_q_bg[0];
      rtb_uDLookupTable2[1] = rtb_q_bg[1];
      rtb_uDLookupTable2[2] = rtb_q_bg[2];
      rtb_uDLookupTable2[3] = rtb_q_bg[3];
      if (rtb_ny_du_red_ff_tmp == 1) {
        for (k = 0; k < 8; k++) {
          rtb_Wv_0[k] = 0.0F;
          for (i = 0; i < rtb_ny_du_red_ff_tmp; i++) {
            rtb_Wv_0[k] += A_free_data[(i << 3) + k] * p_free_data[i];
          }
        }
      } else if (rtb_ny_du_red_ff_tmp_1 == 1) {
        for (k = 0; k < 8; k++) {
          rtb_Wv_0[k] = 0.0F;
          for (i = 0; i < rtb_ny_du_red_ff_tmp; i++) {
            rtb_Wv_0[k] += A_free_data[(i << 3) + k] * p_free_data[i];
          }
        }
      } else {
        for (rtb_ny_du_red_ff_tmp_0 = 0; rtb_ny_du_red_ff_tmp_0 < 8;
             rtb_ny_du_red_ff_tmp_0++) {
          rtb_Wv_0[rtb_ny_du_red_ff_tmp_0] = 0.0F;
        }

        for (k = 0; k < rtb_ny_du_red_ff_tmp; k++) {
          b_aoffset = k << 3;
          for (i = 0; i < 8; i++) {
            rtb_ny_du_red_ff_tmp_0 = b_aoffset + i;
            rtb_Wv_0[i] += A[((f_data[rtb_ny_du_red_ff_tmp_0 / 8] - 1) << 3) +
              rtb_ny_du_red_ff_tmp_0 % 8] * p_free_data[k];
          }
        }
      }

      for (k = 0; k < 8; k++) {
        d_0[k] -= rtb_Wv_0[k];
      }

      for (i = 0; i < 4; i++) {
        p_free_data[i] = 0.0F;
        for (k = 0; k < 8; k++) {
          p_free_data[i] += A[(i << 3) + k] * d_0[k];
        }

        M_bg_idx_8 = rtb_q_cmd_red_c[i] * p_free_data[i];
        x[i] = (M_bg_idx_8 >= -2.22044605E-16F);
        rtb_q_bg[i] = M_bg_idx_8;
      }

      rtb_Compare = true;
      rtb_ny_du_red_ff_tmp_0 = 0;
      exitg2 = false;
      while ((!exitg2) && (rtb_ny_du_red_ff_tmp_0 < 4)) {
        if (!x[rtb_ny_du_red_ff_tmp_0]) {
          rtb_Compare = false;
          exitg2 = true;
        } else {
          rtb_ny_du_red_ff_tmp_0++;
        }
      }

      if (rtb_Compare) {
        exitg1 = true;
      } else {
        rtb_z_Kg = rtb_q_bg[0];
        rtb_ny_du_red_ff_tmp_0 = 0;
        if (rtb_q_bg[0] > rtb_q_bg[1]) {
          rtb_z_Kg = rtb_q_bg[1];
          rtb_ny_du_red_ff_tmp_0 = 1;
        }

        if (rtb_z_Kg > rtb_q_bg[2]) {
          rtb_z_Kg = rtb_q_bg[2];
          rtb_ny_du_red_ff_tmp_0 = 2;
        }

        if (rtb_z_Kg > rtb_q_bg[3]) {
          rtb_ny_du_red_ff_tmp_0 = 3;
        }

        rtb_q_cmd_red_c[rtb_ny_du_red_ff_tmp_0] = 0.0F;
        i_free[rtb_ny_du_red_ff_tmp_0] = true;
        exponent++;
      }
    } else {
      rtb_ny_du_red_ff_tmp_0 = 0;
      dist_idx_0 = 1.0;
      rtb_Compare = (p[0] < 0.0);
      e_idx_0 = (p[0] > 0.0);
      if (i_free[0] && rtb_Compare) {
        rtb_ny_du_red_ff_tmp_0 = 1;
      }

      x[0] = rtb_Compare;
      dist_idx_1 = 1.0;
      rtb_Compare = (p[1] < 0.0);
      e_idx_1 = (p[1] > 0.0);
      if (i_free[1] && rtb_Compare) {
        rtb_ny_du_red_ff_tmp_0++;
      }

      x[1] = rtb_Compare;
      dist_idx_2 = 1.0;
      rtb_Compare = (p[2] < 0.0);
      e_idx_2 = (p[2] > 0.0);
      if (i_free[2] && rtb_Compare) {
        rtb_ny_du_red_ff_tmp_0++;
      }

      x[2] = rtb_Compare;
      dist_idx_3 = 1.0;
      rtb_Compare = (p_0 < 0.0);
      e_idx_3 = (p_0 > 0.0);
      if (i_free[3] && rtb_Compare) {
        rtb_ny_du_red_ff_tmp_0++;
      }

      i = rtb_ny_du_red_ff_tmp_0;
      rtb_ny_du_red_ff_tmp_0 = 0;
      if (i_free[0] && x[0]) {
        h_data[0] = 1;
        rtb_ny_du_red_ff_tmp_0 = 1;
      }

      if (i_free[1] && x[1]) {
        h_data[rtb_ny_du_red_ff_tmp_0] = 2;
        rtb_ny_du_red_ff_tmp_0++;
      }

      if (i_free[2] && x[2]) {
        h_data[rtb_ny_du_red_ff_tmp_0] = 3;
        rtb_ny_du_red_ff_tmp_0++;
      }

      if (i_free[3] && rtb_Compare) {
        h_data[rtb_ny_du_red_ff_tmp_0] = 4;
      }

      for (k = 0; k < i; k++) {
        b_aoffset = h_data[k] - 1;
        c_data[k] = (rtb_q_red[b_aoffset] - rtb_uDLookupTable2[b_aoffset]) /
          (real32_T)p[b_aoffset];
      }

      rtb_ny_du_red_ff_tmp_0 = 0;
      if (i_free[0] && x[0]) {
        dist_idx_0 = c_data[0];
        rtb_ny_du_red_ff_tmp_0 = 1;
      }

      if (i_free[1] && x[1]) {
        dist_idx_1 = c_data[rtb_ny_du_red_ff_tmp_0];
        rtb_ny_du_red_ff_tmp_0++;
      }

      if (i_free[2] && x[2]) {
        dist_idx_2 = c_data[rtb_ny_du_red_ff_tmp_0];
        rtb_ny_du_red_ff_tmp_0++;
      }

      if (i_free[3] && rtb_Compare) {
        dist_idx_3 = c_data[rtb_ny_du_red_ff_tmp_0];
      }

      rtb_ny_du_red_ff_tmp_0 = 0;
      if (i_free[0] && e_idx_0) {
        rtb_ny_du_red_ff_tmp_0 = 1;
      }

      if (i_free[1] && e_idx_1) {
        rtb_ny_du_red_ff_tmp_0++;
      }

      if (i_free[2] && e_idx_2) {
        rtb_ny_du_red_ff_tmp_0++;
      }

      if (i_free[3] && e_idx_3) {
        rtb_ny_du_red_ff_tmp_0++;
      }

      i = rtb_ny_du_red_ff_tmp_0;
      rtb_ny_du_red_ff_tmp_0 = 0;
      if (i_free[0] && e_idx_0) {
        i_data[0] = 1;
        rtb_ny_du_red_ff_tmp_0 = 1;
      }

      if (i_free[1] && e_idx_1) {
        i_data[rtb_ny_du_red_ff_tmp_0] = 2;
        rtb_ny_du_red_ff_tmp_0++;
      }

      if (i_free[2] && e_idx_2) {
        i_data[rtb_ny_du_red_ff_tmp_0] = 3;
        rtb_ny_du_red_ff_tmp_0++;
      }

      if (i_free[3] && e_idx_3) {
        i_data[rtb_ny_du_red_ff_tmp_0] = 4;
      }

      for (k = 0; k < i; k++) {
        b_aoffset = i_data[k] - 1;
        c_data[k] = (rtb_umax[b_aoffset] - rtb_uDLookupTable2[b_aoffset]) /
          (real32_T)p[b_aoffset];
      }

      rtb_ny_du_red_ff_tmp_0 = 0;
      if (i_free[0] && e_idx_0) {
        dist_idx_0 = c_data[0];
        rtb_ny_du_red_ff_tmp_0 = 1;
      }

      if (i_free[1] && e_idx_1) {
        dist_idx_1 = c_data[rtb_ny_du_red_ff_tmp_0];
        rtb_ny_du_red_ff_tmp_0++;
      }

      if (i_free[2] && e_idx_2) {
        dist_idx_2 = c_data[rtb_ny_du_red_ff_tmp_0];
        rtb_ny_du_red_ff_tmp_0++;
      }

      if (i_free[3] && e_idx_3) {
        dist_idx_3 = c_data[rtb_ny_du_red_ff_tmp_0];
      }

      rtb_ny_du_red_ff_tmp_0 = 0;
      if (dist_idx_0 > dist_idx_1) {
        dist_idx_0 = dist_idx_1;
        rtb_ny_du_red_ff_tmp_0 = 1;
      }

      if (dist_idx_0 > dist_idx_2) {
        dist_idx_0 = dist_idx_2;
        rtb_ny_du_red_ff_tmp_0 = 2;
      }

      if (dist_idx_0 > dist_idx_3) {
        dist_idx_0 = dist_idx_3;
        rtb_ny_du_red_ff_tmp_0 = 3;
      }

      rtb_uDLookupTable2[0] += (real32_T)(dist_idx_0 * p[0]);
      rtb_uDLookupTable2[1] += (real32_T)(dist_idx_0 * p[1]);
      rtb_uDLookupTable2[2] += (real32_T)(dist_idx_0 * p[2]);
      rtb_uDLookupTable2[3] += (real32_T)(dist_idx_0 * p_0);
      i = (rtb_ny_du_red_ff_tmp << 3) - 1;
      for (k = 0; k <= i; k++) {
        A_free_data[k] *= (real32_T)dist_idx_0;
      }

      if (rtb_ny_du_red_ff_tmp == 1) {
        for (k = 0; k < 8; k++) {
          rtb_Wv_0[k] = 0.0F;
          for (i = 0; i < 1; i++) {
            rtb_Wv_0[k] += A_free_data[k] * p_free_data[0];
          }
        }
      } else if (rtb_ny_du_red_ff_tmp_1 == 1) {
        for (k = 0; k < 8; k++) {
          rtb_Wv_0[k] = 0.0F;
          for (i = 0; i < rtb_ny_du_red_ff_tmp; i++) {
            rtb_Wv_0[k] += A_free_data[(i << 3) + k] * p_free_data[i];
          }
        }
      } else {
        for (k = 0; k < 8; k++) {
          rtb_Wv_0[k] = 0.0F;
        }

        for (k = 0; k < rtb_ny_du_red_ff_tmp; k++) {
          i = k << 3;
          for (b_aoffset = 0; b_aoffset < 8; b_aoffset++) {
            rtb_Wv_0[b_aoffset] += A_free_data[i + b_aoffset] * p_free_data[k];
          }
        }
      }

      for (k = 0; k < 8; k++) {
        d_0[k] -= rtb_Wv_0[k];
      }

      if (p[rtb_ny_du_red_ff_tmp_0] < 0.0) {
        rtb_q_cmd_red_c[rtb_ny_du_red_ff_tmp_0] = -1.0F;
      } else if (p[rtb_ny_du_red_ff_tmp_0] > 0.0) {
        rtb_q_cmd_red_c[rtb_ny_du_red_ff_tmp_0] = 1.0F;
      } else {
        rtb_q_cmd_red_c[rtb_ny_du_red_ff_tmp_0] = (real32_T)
          p[rtb_ny_du_red_ff_tmp_0];
      }

      i_free[rtb_ny_du_red_ff_tmp_0] = false;
      exponent++;
    }
  }

  // Sum: '<S13>/Add6' incorporates:
  //   DiscreteIntegrator: '<S48>/Discrete-Time Integrator y'

  tmp_3 = rtb_uDLookupTable2[0] + rtDW.DiscreteTimeIntegratory_DSTAT_n[0];

  // Saturate: '<S13>/Saturation3'
  if (tmp_3 > 1.0F) {
    tmp_2 = 1.0F;
  } else if (tmp_3 < 0.1F) {
    tmp_2 = 0.1F;
  } else {
    tmp_2 = tmp_3;
  }

  // Sum: '<S13>/Add6' incorporates:
  //   DiscreteIntegrator: '<S48>/Discrete-Time Integrator y'

  rtb_q_cmd_red_c[0] = tmp_3;
  tmp_3 = rtb_uDLookupTable2[1] + rtDW.DiscreteTimeIntegratory_DSTAT_n[1];

  // Saturate: '<S13>/Saturation3'
  if (tmp_3 > 1.0F) {
    rtb_Wv_1 = 1.0F;
  } else if (tmp_3 < 0.1F) {
    rtb_Wv_1 = 0.1F;
  } else {
    rtb_Wv_1 = tmp_3;
  }

  // Sum: '<S13>/Add6' incorporates:
  //   DiscreteIntegrator: '<S48>/Discrete-Time Integrator y'

  rtb_q_cmd_red_c[1] = tmp_3;
  tmp_3 = rtb_uDLookupTable2[2] + rtDW.DiscreteTimeIntegratory_DSTAT_n[2];

  // Saturate: '<S13>/Saturation3'
  if (tmp_3 > 1.0F) {
    y_1 = 1.0F;
  } else if (tmp_3 < 0.1F) {
    y_1 = 0.1F;
  } else {
    y_1 = tmp_3;
  }

  // Sum: '<S13>/Add6' incorporates:
  //   DiscreteIntegrator: '<S48>/Discrete-Time Integrator y'

  rtb_q_cmd_red_c[2] = tmp_3;
  tmp_3 = rtb_uDLookupTable2[3] + rtDW.DiscreteTimeIntegratory_DSTAT_n[3];

  // Saturate: '<S13>/Saturation3'
  if (tmp_3 > 1.0F) {
    q_yaw[3] = 1.0F;
  } else if (tmp_3 < 0.1F) {
    q_yaw[3] = 0.1F;
  } else {
    q_yaw[3] = tmp_3;
  }

  // Sum: '<S13>/Add6'
  rtb_q_cmd_red_c[3] = tmp_3;

  // Lookup_n-D: '<S8>/1-D Lookup Table1' incorporates:
  //   Inport: '<Root>/cmd'

  rtb_z_Kg = look1_iflf_binlx(rtU.cmd.RC_pwm[7], rtConstP.pooled14,
    rtConstP.pooled13, 1U);

  // Lookup_n-D: '<S8>/1-D Lookup Table' incorporates:
  //   Inport: '<Root>/cmd'

  scale = look1_iflf_binlx(rtU.cmd.RC_pwm[6], rtConstP.pooled14,
    rtConstP.pooled13, 1U);

  // MinMax: '<S8>/Min'
  if (scale >= rtb_z_Kg) {
    scale = rtb_z_Kg;
  }

  // End of MinMax: '<S8>/Min'

  // Switch: '<S5>/Switch2' incorporates:
  //   RelationalOperator: '<S5>/LowerRelop1'
  //   Saturate: '<S13>/Saturation3'

  if (rtb_Wv_1 <= scale) {
    scale = rtb_Wv_1;
  }

  // End of Switch: '<S5>/Switch2'

  // RelationalOperator: '<S102>/Compare' incorporates:
  //   Constant: '<S102>/Constant'
  //   UnitDelay: '<S8>/Unit Delay'

  rtb_Compare = (rtDW.UnitDelay_DSTATE > 0.5F);

  // DiscreteIntegrator: '<S8>/Discrete-Time Integrator'
  if (rtb_Compare && (rtDW.DiscreteTimeIntegrator_PrevRese <= 0)) {
    rtDW.DiscreteTimeIntegrator_DSTAT_ow = 0.0F;
  }

  // Switch: '<S8>/Switch1' incorporates:
  //   Constant: '<S101>/Constant'
  //   Constant: '<S8>/Constant3'
  //   Constant: '<S8>/Constant4'
  //   DataTypeConversion: '<S8>/Data Type Conversion'
  //   DiscreteIntegrator: '<S8>/Discrete-Time Integrator'
  //   RelationalOperator: '<S101>/Compare'

  if ((real32_T)(rtDW.DiscreteTimeIntegrator_DSTAT_ow >= 0.005F) >= 0.5F) {
    M_bg_idx_8 = 0.1F;
  } else {
    M_bg_idx_8 = 1.0F;
  }

  // End of Switch: '<S8>/Switch1'

  // RateLimiter: '<S8>/Rate Limiter'
  absxk = M_bg_idx_8 - rtDW.PrevY;
  if (absxk > 2.5F) {
    M_bg_idx_8 = rtDW.PrevY + 2.5F;
  } else {
    if (absxk < -0.0005F) {
      M_bg_idx_8 = rtDW.PrevY + -0.0005F;
    }
  }

  rtDW.PrevY = M_bg_idx_8;

  // End of RateLimiter: '<S8>/Rate Limiter'

  // Math: '<S8>/Square'
  M_bg_idx_8 *= M_bg_idx_8;

  // Switch: '<S7>/Switch2' incorporates:
  //   RelationalOperator: '<S7>/LowerRelop1'
  //   Saturate: '<S13>/Saturation3'

  if (y_1 <= M_bg_idx_8) {
    M_bg_idx_8 = y_1;
  }

  // End of Switch: '<S7>/Switch2'

  // Outport: '<Root>/u'
  rtY.u[0] = scale;

  // Switch: '<S6>/Switch2' incorporates:
  //   RelationalOperator: '<S6>/LowerRelop1'

  if (q_yaw[3] > rtb_z_Kg) {
    // Outport: '<Root>/u'
    rtY.u[1] = rtb_z_Kg;
  } else {
    // Outport: '<Root>/u' incorporates:
    //   Switch: '<S6>/Switch'

    rtY.u[1] = q_yaw[3];
  }

  // End of Switch: '<S6>/Switch2'

  // Outport: '<Root>/u' incorporates:
  //   Gain: '<Root>/Gain1'
  //   Gain: '<Root>/Gain2'
  //   Gain: '<Root>/Gain3'
  //   Gain: '<Root>/Gain4'
  //   Saturate: '<S13>/Saturation3'

  rtY.u[2] = tmp_2;
  rtY.u[3] = M_bg_idx_8;
  rtY.u[4] = 0.0F;
  rtY.u[5] = 0.0F;
  rtY.u[6] = 0.0F;
  rtY.u[7] = 0.0F;

  // Sum: '<S48>/Sum2' incorporates:
  //   Delay: '<S13>/Delay'
  //   DiscreteIntegrator: '<S48>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S48>/Discrete-Time Integrator y_dt'
  //   Gain: '<S48>/2*d//omega'
  //   Sum: '<S48>/Sum3'

  rtb_q_red[0] = rtDW.Delay_DSTATE[0] - (0.0039788736F *
    rtDW.DiscreteTimeIntegratory_dt_DS_m[0] +
    rtDW.DiscreteTimeIntegratory_DSTAT_n[0]);
  rtb_q_red[1] = rtDW.Delay_DSTATE[1] - (0.0039788736F *
    rtDW.DiscreteTimeIntegratory_dt_DS_m[1] +
    rtDW.DiscreteTimeIntegratory_DSTAT_n[1]);
  rtb_q_red[2] = rtDW.Delay_DSTATE[2] - (0.0039788736F *
    rtDW.DiscreteTimeIntegratory_dt_DS_m[2] +
    rtDW.DiscreteTimeIntegratory_DSTAT_n[2]);
  rtb_q_red[3] = rtDW.Delay_DSTATE[3] - (0.0039788736F *
    rtDW.DiscreteTimeIntegratory_dt_DS_m[3] +
    rtDW.DiscreteTimeIntegratory_DSTAT_n[3]);

  // MATLAB Function: '<S16>/DCM to quaternions'
  DCMtoquaternions(rtb_M_bg, rtb_umax);

  // MATLAB Function: '<S25>/MATLAB Function2' incorporates:
  //   Constant: '<S25>/Constant1'
  //   Sum: '<S25>/Add'

  rtb_n_b_d_dt2_g[2] = cos_Psi - -9.81F;
  if (std::abs(cos_Psi - -9.81F) < 1.0F) {
    if (cos_Psi - -9.81F > 0.0F) {
      rtb_n_b_d_dt2_g[2] = 1.0F;
    } else {
      rtb_n_b_d_dt2_g[2] = -1.0F;
    }
  }

  // MATLAB Function: '<S25>/MATLAB Function8' incorporates:
  //   MATLAB Function: '<S25>/MATLAB Function2'
  //   SignalConversion: '<S30>/TmpSignal ConversionAt SFunction Inport1'

  MATLABFunction3(std::atan2(std::sqrt(rtb_Add1_m[0] * rtb_Add1_m[0] +
    rtb_Add1_m[1] * rtb_Add1_m[1]), -rtb_n_b_d_dt2_g[2]), std::atan2(rtb_Add1_m
    [1], rtb_Add1_m[0]), rtb_q_bg);

  // MATLAB Function: '<S25>/MATLAB Function9' incorporates:
  //   Constant: '<S25>/Constant1'
  //   Sum: '<S25>/Add3'

  rtb_n_b_d_dt2_g[2] = rtb_Diff_j[2] - -9.81F;
  if (std::abs(rtb_Diff_j[2] - -9.81F) < 1.0F) {
    if (rtb_Diff_j[2] - -9.81F > 0.0F) {
      rtb_n_b_d_dt2_g[2] = 1.0F;
    } else {
      rtb_n_b_d_dt2_g[2] = -1.0F;
    }
  }

  // MATLAB Function: '<S25>/MATLAB Function3' incorporates:
  //   MATLAB Function: '<S25>/MATLAB Function9'
  //   SignalConversion: '<S35>/TmpSignal ConversionAt SFunction Inport1'

  MATLABFunction3(std::atan2(std::sqrt(rtb_Diff_j[0] * rtb_Diff_j[0] +
    rtb_Diff_j[1] * rtb_Diff_j[1]), -rtb_n_b_d_dt2_g[2]), std::atan2(rtb_Diff_j
    [1], rtb_Diff_j[0]), rtb_q_cmd_red_c);

  // MATLAB Function: '<S25>/MATLAB Function5'
  q0_q0 = ((rtb_q_cmd_red_c[0] * rtb_q_cmd_red_c[0] + rtb_q_cmd_red_c[1] *
            rtb_q_cmd_red_c[1]) + rtb_q_cmd_red_c[2] * rtb_q_cmd_red_c[2]) +
    rtb_q_cmd_red_c[3] * rtb_q_cmd_red_c[3];
  q0_q3 = rtb_q_cmd_red_c[0] / q0_q0;
  q1_q3 = -rtb_q_cmd_red_c[1] / q0_q0;
  q2_q3 = -rtb_q_cmd_red_c[2] / q0_q0;
  M_bg_idx_8 = -rtb_q_cmd_red_c[3] / q0_q0;
  rtb_q_cmd_red_c[0] = ((q0_q3 * rtb_q_bg[0] - q1_q3 * rtb_q_bg[1]) - q2_q3 *
                        rtb_q_bg[2]) - M_bg_idx_8 * rtb_q_bg[3];
  rtb_q_cmd_red_c[1] = (q0_q3 * rtb_q_bg[1] + rtb_q_bg[0] * q1_q3) + (q2_q3 *
    rtb_q_bg[3] - M_bg_idx_8 * rtb_q_bg[2]);
  rtb_q_cmd_red_c[2] = (q0_q3 * rtb_q_bg[2] + rtb_q_bg[0] * q2_q3) + (M_bg_idx_8
    * rtb_q_bg[1] - q1_q3 * rtb_q_bg[3]);
  rtb_q_cmd_red_c[3] = (q0_q3 * rtb_q_bg[3] + rtb_q_bg[0] * M_bg_idx_8) + (q1_q3
    * rtb_q_bg[2] - q2_q3 * rtb_q_bg[1]);

  // MATLAB Function: '<S16>/total reduced attitude command (quaternion)'
  q0_q3 = ((rtb_umax[0] * rtb_q_cmd_red_c[0] - rtb_umax[1] * rtb_q_cmd_red_c[1])
           - rtb_umax[2] * rtb_q_cmd_red_c[2]) - rtb_umax[3] * rtb_q_cmd_red_c[3];
  q1_q3 = (rtb_umax[0] * rtb_q_cmd_red_c[1] + rtb_q_cmd_red_c[0] * rtb_umax[1])
    + (rtb_umax[2] * rtb_q_cmd_red_c[3] - rtb_umax[3] * rtb_q_cmd_red_c[2]);
  q2_q3 = (rtb_umax[0] * rtb_q_cmd_red_c[2] + rtb_q_cmd_red_c[0] * rtb_umax[2])
    + (rtb_umax[3] * rtb_q_cmd_red_c[1] - rtb_umax[1] * rtb_q_cmd_red_c[3]);
  M_bg_idx_8 = (rtb_umax[0] * rtb_q_cmd_red_c[3] + rtb_q_cmd_red_c[0] *
                rtb_umax[3]) + (rtb_umax[1] * rtb_q_cmd_red_c[2] - rtb_umax[2] *
    rtb_q_cmd_red_c[1]);

  // MATLAB Function: '<S21>/quaternion to  reduced attitude unit vector'
  scale = 1.29246971E-26F;
  absxk = std::abs(q0_q3);
  if (absxk > 1.29246971E-26F) {
    q0_q0 = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    q0_q0 = t * t;
  }

  absxk = std::abs(q1_q3);
  if (absxk > scale) {
    t = scale / absxk;
    q0_q0 = q0_q0 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q0_q0 += t * t;
  }

  absxk = std::abs(q2_q3);
  if (absxk > scale) {
    t = scale / absxk;
    q0_q0 = q0_q0 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q0_q0 += t * t;
  }

  absxk = std::abs(M_bg_idx_8);
  if (absxk > scale) {
    t = scale / absxk;
    q0_q0 = q0_q0 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q0_q0 += t * t;
  }

  q0_q0 = scale * std::sqrt(q0_q0);
  if (2.22044605E-16F >= q0_q0) {
    q0_q0 = 2.22044605E-16F;
  }

  q0_q3 /= q0_q0;
  q1_q3 /= q0_q0;
  q2_q3 /= q0_q0;
  rtb_q_cmd = M_bg_idx_8 / q0_q0;
  q0_q0 = q0_q3 * q0_q3;
  scale = q1_q3 * q1_q3;
  t = q2_q3 * q2_q3;
  absxk = rtb_q_cmd * rtb_q_cmd;
  M_bg_idx_8 = q0_q3 * q1_q3;
  q0_q2 = q0_q3 * q2_q3;
  q0_q3 *= rtb_q_cmd;
  q1_q2 = q1_q3 * q2_q3;
  q1_q3 *= rtb_q_cmd;
  q2_q3 *= rtb_q_cmd;
  rtb_M_bg[0] = ((q0_q0 + scale) - t) - absxk;
  rtb_M_bg[1] = (q1_q2 + q0_q3) * 2.0F;
  rtb_M_bg[2] = (q1_q3 - q0_q2) * 2.0F;
  rtb_M_bg[3] = (q1_q2 - q0_q3) * 2.0F;
  q0_q0 -= scale;
  rtb_M_bg[4] = (q0_q0 + t) - absxk;
  rtb_M_bg[5] = (q2_q3 + M_bg_idx_8) * 2.0F;
  rtb_M_bg[6] = (q1_q3 + q0_q2) * 2.0F;
  rtb_M_bg[7] = (q2_q3 - M_bg_idx_8) * 2.0F;
  rtb_M_bg[8] = (q0_q0 - t) + absxk;
  for (i = 0; i < 3; i++) {
    // Sum: '<S84>/Sum2' incorporates:
    //   DiscreteIntegrator: '<S84>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S84>/Discrete-Time Integrator y_dt'
    //   Gain: '<S84>/2*d//omega'
    //   Sum: '<S84>/Sum3'

    rtb_Diff[i] = rtb_TmpSignalConversionAtSFunct[i] - (0.0319788754F *
      rtDW.DiscreteTimeIntegratory_dt_DS_d[i] +
      rtDW.DiscreteTimeIntegratory_DSTAT_c[i]);

    // Sum: '<S83>/Sum2' incorporates:
    //   DiscreteIntegrator: '<S81>/Discrete-Time Integrator y_dt'
    //   DiscreteIntegrator: '<S83>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S83>/Discrete-Time Integrator y_dt'
    //   Gain: '<S83>/2*d//omega'
    //   Sum: '<S83>/Sum3'

    rtb_n_b_d_dt2_g[i] = rtDW.DiscreteTimeIntegratory_dt_DSTA[i] -
      (0.0319788754F * rtDW.DiscreteTimeIntegratory_dt_DS_g[i] +
       rtDW.DiscreteTimeIntegratory_DSTAT_d[i]);

    // Sum: '<S82>/Sum2' incorporates:
    //   DiscreteIntegrator: '<S81>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S82>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S82>/Discrete-Time Integrator y_dt'
    //   Gain: '<S82>/2*d//omega'
    //   Sum: '<S82>/Sum3'

    rtb_Add[i] = rtDW.DiscreteTimeIntegratory_DSTATE[i] - (0.0319788754F *
      rtDW.DiscreteTimeIntegratory_dt_DS_i[i] +
      rtDW.DiscreteTimeIntegratory_DSTAT_f[i]);
    rtb_Diff_j[i] = -rtb_M_bg[i + 6];
  }

  // End of MATLAB Function: '<S21>/quaternion to  reduced attitude unit vector' 

  // MATLAB Function: '<S21>/MATLAB Function4'
  MATLABFunction4(rtb_Diff_j, &q0_q0, &scale);

  // Update for DiscreteIntegrator: '<S93>/Discrete-Time Integrator' incorporates:
  //   Gain: '<S93>/1//T'
  //   Saturate: '<S13>/Saturation3'
  //   Sum: '<S93>/Sum2'

  rtDW.DiscreteTimeIntegrator_DSTATE[0] += (tmp_2 -
    rtDW.DiscreteTimeIntegrator_DSTATE[0]) * 35.7142868F * 0.0025F;
  rtDW.DiscreteTimeIntegrator_DSTATE[1] += (rtb_Wv_1 -
    rtDW.DiscreteTimeIntegrator_DSTATE[1]) * 35.7142868F * 0.0025F;
  rtDW.DiscreteTimeIntegrator_DSTATE[2] += (y_1 -
    rtDW.DiscreteTimeIntegrator_DSTATE[2]) * 35.7142868F * 0.0025F;

  // Saturate: '<S13>/Saturation3'
  if (tmp_3 > 1.0F) {
    tmp_3 = 1.0F;
  } else {
    if (tmp_3 < 0.1F) {
      tmp_3 = 0.1F;
    }
  }

  // Update for DiscreteIntegrator: '<S93>/Discrete-Time Integrator' incorporates:
  //   Gain: '<S93>/1//T'
  //   Sum: '<S93>/Sum2'

  rtDW.DiscreteTimeIntegrator_DSTATE[3] += (tmp_3 -
    rtDW.DiscreteTimeIntegrator_DSTATE[3]) * 35.7142868F * 0.0025F;

  // Update for UnitDelay: '<S40>/UD'
  //
  //  Block description for '<S40>/UD':
  //
  //   Store in Global RAM

  rtDW.UD_DSTATE[0] = rtb_TSamp_idx_0;

  // Update for DiscreteIntegrator: '<S81>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S81>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTATE[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DSTA[0];

  // Update for DiscreteIntegrator: '<S81>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DSTA[0] += 0.0025F *
    rtb_TmpSignalConversionAtSFunct[0];

  // Update for UnitDelay: '<S40>/UD'
  //
  //  Block description for '<S40>/UD':
  //
  //   Store in Global RAM

  rtDW.UD_DSTATE[1] = rtb_TSamp_idx_1;

  // Update for DiscreteIntegrator: '<S81>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S81>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTATE[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DSTA[1];

  // Update for DiscreteIntegrator: '<S81>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DSTA[1] += 0.0025F *
    rtb_TmpSignalConversionAtSFunct[1];

  // Update for UnitDelay: '<S40>/UD'
  //
  //  Block description for '<S40>/UD':
  //
  //   Store in Global RAM

  rtDW.UD_DSTATE[2] = rtb_M_bg_tmp;

  // Update for DiscreteIntegrator: '<S81>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S81>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTATE[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DSTA[2];

  // Update for DiscreteIntegrator: '<S81>/Discrete-Time Integrator y_dt'
  rtDW.DiscreteTimeIntegratory_dt_DSTA[2] += 0.0025F *
    rtb_TmpSignalConversionAtSFunct[2];

  // Update for DiscreteIntegrator: '<S38>/Discrete-Time Integrator' incorporates:
  //   Saturate: '<S38>/Saturation'

  rtDW.DiscreteTimeIntegrator_DSTATE_h[0] += 0.0025F * y;
  rtDW.DiscreteTimeIntegrator_DSTATE_h[1] += 0.0025F * y_0;

  // Update for UnitDelay: '<S11>/UD'
  //
  //  Block description for '<S11>/UD':
  //
  //   Store in Global RAM

  rtDW.UD_DSTATE_k[0] = cmd_V_NED_idx_0;
  rtDW.UD_DSTATE_k[1] = cmd_V_NED_idx_1;
  rtDW.UD_DSTATE_k[2] = u0;

  // Update for DiscreteIntegrator: '<S92>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_DSTATE_d += 0.0025F * cos_Psi;

  // Update for DiscreteIntegrator: '<S82>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LOAD = 0U;

  // Update for DiscreteIntegrator: '<S83>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LO_d = 0U;

  // Update for DiscreteIntegrator: '<S84>/Discrete-Time Integrator y'
  rtDW.DiscreteTimeIntegratory_IC_LO_p = 0U;

  // Update for DiscreteIntegrator: '<S82>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S82>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_f[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_i[0];

  // Update for DiscreteIntegrator: '<S83>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S83>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_d[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_g[0];

  // Update for DiscreteIntegrator: '<S84>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S84>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_c[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_d[0];

  // Update for DiscreteIntegrator: '<S82>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S82>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_f[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_i[1];

  // Update for DiscreteIntegrator: '<S83>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S83>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_d[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_g[1];

  // Update for DiscreteIntegrator: '<S84>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S84>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_c[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_d[1];

  // Update for DiscreteIntegrator: '<S82>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S82>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_f[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_i[2];

  // Update for DiscreteIntegrator: '<S83>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S83>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_d[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_g[2];

  // Update for DiscreteIntegrator: '<S84>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S84>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_c[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_d[2];

  // Update for DiscreteIntegrator: '<S65>/Discrete-Time Integrator2' incorporates:
  //   DiscreteIntegrator: '<S80>/Discrete-Time Integrator'

  rtDW.DiscreteTimeIntegrator2_IC_LOAD = 0U;
  rtDW.DiscreteTimeIntegrator2_DSTATE += 0.0025F *
    rtDW.DiscreteTimeIntegrator_DSTATE_o;

  // Update for DiscreteIntegrator: '<S80>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_DSTATE_o += 0.0025F * rtb_delta_f;

  // Update for DiscreteIntegrator: '<S90>/Discrete-Time Integrator3'
  rtDW.DiscreteTimeIntegrator3_IC_LOAD = 0U;
  rtDW.DiscreteTimeIntegrator3_DSTATE += 0.0025F * sin_Psi;

  // Update for UnitDelay: '<S41>/UD'
  //
  //  Block description for '<S41>/UD':
  //
  //   Store in Global RAM

  rtDW.UD_DSTATE_n = rtb_phi_k;
  for (k = 0; k < 6; k++) {
    // Update for DiscreteIntegrator: '<S39>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S39>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTAT_j[k] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_f[k];

    // Update for DiscreteIntegrator: '<S39>/Discrete-Time Integrator y_dt' incorporates:
    //   Gain: '<S39>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_DS_f[k] += 101.368347F * rtb_Add_gb[k] *
      0.0025F;
  }

  // Update for DiscreteIntegrator: '<S26>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_IC_LOADI = 0U;
  rtDW.DiscreteTimeIntegrator_DSTATE_a[0] += 0.0025F * rtb_y_f_idx_0;
  rtDW.DiscreteTimeIntegrator_DSTATE_a[1] += 0.0025F * rtb_y_f_idx_1;

  // Update for UnitDelay: '<S8>/Unit Delay'
  rtDW.UnitDelay_DSTATE = rtb_z_Kg;

  // Update for DiscreteIntegrator: '<S8>/Discrete-Time Integrator' incorporates:
  //   Switch: '<S8>/Switch'

  rtDW.DiscreteTimeIntegrator_DSTAT_ow += (real32_T)(rtb_z_Kg <= 0.5F) * 0.0025F;
  rtDW.DiscreteTimeIntegrator_PrevRese = (int8_T)rtb_Compare;

  // Update for UnitDelay: '<S42>/Unit Delay1'
  rtDW.UnitDelay1_DSTATE[0] = rtb_uDLookupTable2[0];

  // Update for DiscreteIntegrator: '<S48>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S48>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_n[0] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_m[0];

  // Update for DiscreteIntegrator: '<S48>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S48>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_m[0] += 252661.875F * rtb_q_red[0] *
    0.0025F;

  // Update for Delay: '<S13>/Delay'
  rtDW.Delay_DSTATE[0] = rtb_y_idx_0;

  // Update for UnitDelay: '<S42>/Unit Delay1'
  rtDW.UnitDelay1_DSTATE[1] = rtb_uDLookupTable2[1];

  // Update for DiscreteIntegrator: '<S48>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S48>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_n[1] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_m[1];

  // Update for DiscreteIntegrator: '<S48>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S48>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_m[1] += 252661.875F * rtb_q_red[1] *
    0.0025F;

  // Update for Delay: '<S13>/Delay'
  rtDW.Delay_DSTATE[1] = rtb_y_idx_1;

  // Update for UnitDelay: '<S42>/Unit Delay1'
  rtDW.UnitDelay1_DSTATE[2] = rtb_uDLookupTable2[2];

  // Update for DiscreteIntegrator: '<S48>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S48>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_n[2] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_m[2];

  // Update for DiscreteIntegrator: '<S48>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S48>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_m[2] += 252661.875F * rtb_q_red[2] *
    0.0025F;

  // Update for Delay: '<S13>/Delay'
  rtDW.Delay_DSTATE[2] = rtb_y_idx_2;

  // Update for UnitDelay: '<S42>/Unit Delay1'
  rtDW.UnitDelay1_DSTATE[3] = rtb_uDLookupTable2[3];

  // Update for DiscreteIntegrator: '<S48>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S48>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTAT_n[3] += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DS_m[3];

  // Update for DiscreteIntegrator: '<S48>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S48>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_m[3] += 252661.875F * rtb_q_red[3] *
    0.0025F;

  // Update for Delay: '<S13>/Delay'
  rtDW.Delay_DSTATE[3] = rtb_y_idx_3;

  // Update for DiscreteIntegrator: '<S84>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S84>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_d[0] += 3911.41284F * rtb_Diff[0] * 0.0025F;

  // Update for DiscreteIntegrator: '<S83>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S83>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_g[0] += 3911.41284F * rtb_n_b_d_dt2_g[0] *
    0.0025F;

  // Update for DiscreteIntegrator: '<S82>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S82>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_i[0] += 3911.41284F * rtb_Add[0] * 0.0025F;

  // Update for DiscreteIntegrator: '<S84>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S84>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_d[1] += 3911.41284F * rtb_Diff[1] * 0.0025F;

  // Update for DiscreteIntegrator: '<S83>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S83>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_g[1] += 3911.41284F * rtb_n_b_d_dt2_g[1] *
    0.0025F;

  // Update for DiscreteIntegrator: '<S82>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S82>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_i[1] += 3911.41284F * rtb_Add[1] * 0.0025F;

  // Update for DiscreteIntegrator: '<S84>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S84>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_d[2] += 3911.41284F * rtb_Diff[2] * 0.0025F;

  // Update for DiscreteIntegrator: '<S83>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S83>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_g[2] += 3911.41284F * rtb_n_b_d_dt2_g[2] *
    0.0025F;

  // Update for DiscreteIntegrator: '<S82>/Discrete-Time Integrator y_dt' incorporates:
  //   Gain: '<S82>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DS_i[2] += 3911.41284F * rtb_Add[2] * 0.0025F;
}

// Model initialize function
void MatlabControllerClass::initialize()
{
  // InitializeConditions for DiscreteIntegrator: '<S81>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_DSTATE[0] = 0.0F;
  rtDW.DiscreteTimeIntegratory_DSTATE[1] = 0.0F;
  rtDW.DiscreteTimeIntegratory_DSTATE[2] = -1.0F;

  // InitializeConditions for DiscreteIntegrator: '<S82>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LOAD = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S83>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LO_d = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S84>/Discrete-Time Integrator y' 
  rtDW.DiscreteTimeIntegratory_IC_LO_p = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S65>/Discrete-Time Integrator2' 
  rtDW.DiscreteTimeIntegrator2_IC_LOAD = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S90>/Discrete-Time Integrator3' 
  rtDW.DiscreteTimeIntegrator3_IC_LOAD = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S26>/Discrete-Time Integrator' 
  rtDW.DiscreteTimeIntegrator_IC_LOADI = 1U;

  // InitializeConditions for DiscreteIntegrator: '<S8>/Discrete-Time Integrator' 
  rtDW.DiscreteTimeIntegrator_PrevRese = 2;
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
