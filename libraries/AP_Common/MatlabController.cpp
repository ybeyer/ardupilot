//
// Sponsored License - for use in support of a program or activity
// sponsored by MathWorks.  Not for government, commercial or other
// non-sponsored organizational use.
//
// File: MatlabController.cpp
//
// Code generated for Simulink model 'MatlabController'.
//
// Model version                  : 1.385
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Thu Oct 22 16:10:19 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#include "MatlabController.h"

extern real32_T rt_hypotf(real32_T u0, real32_T u1);
static void wrapangle(real32_T rtu_angle, real32_T *rty_angle_0_2pi);

//
// Output and update for atomic system:
//    '<S9>/wrap angle'
//    '<S10>/wrap angle'
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

// Function for MATLAB Function: '<S8>/MATLAB Function'
void MatlabControllerClass::LSQFromQR(const real32_T A_data[], const int32_T
  A_size[2], const real32_T tau_data[], const int32_T jpvt_data[], real32_T B_3
  [7], int32_T rankA, real32_T Y_data[], int32_T *Y_size)
{
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
      for (loop_ub = b_j + 1; loop_ub + 1 < 8; loop_ub++) {
        wj += A_data[7 * b_j + loop_ub] * B_3[loop_ub];
      }

      wj *= tau_data[b_j];
      if (wj != 0.0F) {
        B_3[b_j] -= wj;
        for (loop_ub = b_j + 1; loop_ub + 1 < 8; loop_ub++) {
          B_3[loop_ub] -= A_data[7 * b_j + loop_ub] * wj;
        }
      }
    }
  }

  for (loop_ub = 0; loop_ub < rankA; loop_ub++) {
    Y_data[jpvt_data[loop_ub] - 1] = B_3[loop_ub];
  }

  for (loop_ub = rankA - 1; loop_ub + 1 > 0; loop_ub--) {
    Y_data[jpvt_data[loop_ub] - 1] /= A_data[7 * loop_ub + loop_ub];
    for (b_j = 0; b_j < loop_ub; b_j++) {
      Y_data[jpvt_data[b_j] - 1] -= A_data[7 * loop_ub + b_j] *
        Y_data[jpvt_data[loop_ub] - 1];
    }
  }
}

// Function for MATLAB Function: '<S8>/MATLAB Function'
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

// Function for MATLAB Function: '<S8>/MATLAB Function'
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
      coltop = lastc * 7 + ic0;
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
      jy = 7 * lastc + ic0;
      for (iac = ic0; iac <= jy; iac += 7) {
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
        coltop += 7;
      }
    }
  }
}

// Function for MATLAB Function: '<S8>/MATLAB Function'
void MatlabControllerClass::qrsolve(const real32_T A_data[], const int32_T
  A_size[2], const real32_T B_1[7], real32_T Y_data[], int32_T *Y_size)
{
  real32_T b_A_data[28];
  real32_T tau_data[4];
  int32_T jpvt_data[4];
  int32_T n;
  real32_T work_data[4];
  real32_T vn1_data[4];
  real32_T vn2_data[4];
  int32_T nmi;
  int32_T b_n;
  int32_T yk;
  int32_T ix;
  real32_T smax;
  real32_T s;
  int32_T b_ix;
  int32_T iy;
  int32_T d_k;
  real32_T absxk;
  real32_T t;
  real32_T B_2[7];
  int32_T b_A_size[2];
  int8_T c_idx_0;
  b_A_size[0] = 7;
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
      for (nmi = b_n; nmi <= b_n + 6; nmi++) {
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
      b_n += 7;
    }

    for (b_n = 0; b_n < n; b_n++) {
      yk = b_n * 7 + b_n;
      nmi = n - b_n;
      if (nmi < 1) {
        iy = 0;
      } else {
        iy = 1;
        if (nmi > 1) {
          ix = b_n;
          smax = std::abs(vn1_data[b_n]);
          for (b_ix = 2; b_ix <= nmi; b_ix++) {
            ix++;
            s = std::abs(vn1_data[ix]);
            if (s > smax) {
              iy = b_ix;
              smax = s;
            }
          }
        }
      }

      ix = (b_n + iy) - 1;
      if (ix + 1 != b_n + 1) {
        b_ix = 7 * ix;
        iy = 7 * b_n;
        for (d_k = 0; d_k < 7; d_k++) {
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
      s = xnrm2(6 - b_n, b_A_data, yk + 2);
      if (s != 0.0F) {
        s = rt_hypotf(b_A_data[yk], s);
        if (b_A_data[yk] >= 0.0F) {
          s = -s;
        }

        if (std::abs(s) < 9.86076132E-32F) {
          ix = -1;
          b_ix = (yk - b_n) + 7;
          do {
            ix++;
            for (iy = yk + 1; iy < b_ix; iy++) {
              b_A_data[iy] *= 1.01412048E+31F;
            }

            s *= 1.01412048E+31F;
            smax *= 1.01412048E+31F;
          } while (std::abs(s) < 9.86076132E-32F);

          s = rt_hypotf(smax, xnrm2(6 - b_n, b_A_data, yk + 2));
          if (smax >= 0.0F) {
            s = -s;
          }

          tau_data[b_n] = (s - smax) / s;
          smax = 1.0F / (smax - s);
          b_ix = (yk - b_n) + 7;
          for (iy = yk + 1; iy < b_ix; iy++) {
            b_A_data[iy] *= smax;
          }

          for (iy = 0; iy <= ix; iy++) {
            s *= 9.86076132E-32F;
          }

          smax = s;
        } else {
          tau_data[b_n] = (s - b_A_data[yk]) / s;
          smax = 1.0F / (b_A_data[yk] - s);
          ix = (yk - b_n) + 7;
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
        xzlarf(7 - b_n, nmi - 1, yk + 1, tau_data[b_n], b_A_data, (b_n + (b_n +
                 1) * 7) + 1, work_data);
        b_A_data[yk] = smax;
      }

      for (yk = b_n + 1; yk < n; yk++) {
        if (vn1_data[yk] != 0.0F) {
          nmi = 7 * yk + b_n;
          smax = std::abs(b_A_data[nmi]) / vn1_data[yk];
          smax = 1.0F - smax * smax;
          if (smax < 0.0F) {
            smax = 0.0F;
          }

          s = vn1_data[yk] / vn2_data[yk];
          s = s * s * smax;
          if (s <= 0.000345266977F) {
            vn1_data[yk] = xnrm2(6 - b_n, b_A_data, nmi + 2);
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
    while ((n < b_A_size[1]) && (std::abs(b_A_data[7 * n + n]) > 8.34465E-6F *
            std::abs(b_A_data[0]))) {
      n++;
    }
  }

  for (b_ix = 0; b_ix < 7; b_ix++) {
    B_2[b_ix] = B_1[b_ix];
  }

  LSQFromQR(b_A_data, b_A_size, tau_data, jpvt_data, B_2, n, Y_data, Y_size);
}

// Function for MATLAB Function: '<S8>/MATLAB Function'
void MatlabControllerClass::mldivide(const real32_T A_data[], const int32_T
  A_size[2], const real32_T B_0[7], real32_T Y_data[], int32_T *Y_size)
{
  if (A_size[1] == 0) {
    *Y_size = 0;
  } else {
    qrsolve(A_data, A_size, B_0, Y_data, Y_size);
  }
}

// Function for MATLAB Function: '<S8>/MATLAB Function'
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
  real32_T sin_Phi;
  real32_T cos_Phi;
  real32_T sin_Theta;
  real32_T cos_Theta;
  real32_T sin_Psi;
  real32_T cos_Psi;
  real32_T q0_q3;
  real32_T q1_q2;
  real32_T q1_q3;
  real32_T q2_q3;
  real32_T unusedU0[4];
  real32_T A[28];
  real32_T d[7];
  boolean_T i_free[4];
  real32_T A_free_data[28];
  real32_T p_free_data[4];
  real_T p[4];
  real32_T u_opt[4];
  real_T c_data[4];
  int8_T f_data[4];
  int8_T g_data[4];
  int8_T h_data[4];
  int8_T i_data[4];
  int32_T trueCount;
  int32_T aoffset;
  int32_T b_k;
  int32_T b_aoffset;
  boolean_T x[4];
  real32_T rtb_eulerAngles_dt[3];
  real32_T rtb_umin[4];
  real32_T rtb_umax[4];
  real32_T rtb_u0[4];
  real32_T rtb_M_bg[9];
  real32_T rtb_MatrixMultiply[9];
  real32_T rtb_MatrixMultiply_l[3];
  int32_T i;
  boolean_T u_opt_data[4];
  real32_T sin_Phi_0[9];
  real32_T tmp;
  real32_T rtb_M_bg_0[12];
  real32_T rtb_omega2_0[3];
  real32_T tmp_0[5];
  real32_T rtb_eulerAngles_dt_0[5];
  real32_T tmp_1[6];
  real32_T tmp_2[3];
  real32_T rtb_eulerAngles_dt_1[9];
  real32_T rtb_omega2_1[3];
  real32_T rtb_M_bg_1[7];
  boolean_T u_opt_0[4];
  int32_T A_free_size[2];
  real_T p_0;
  boolean_T x_0;
  real32_T rtb_EulerAngles_idx_2;
  real_T dist_idx_0;
  boolean_T e_idx_0;
  real_T dist_idx_1;
  boolean_T e_idx_1;
  real_T dist_idx_2;
  boolean_T e_idx_2;
  real_T dist_idx_3;
  boolean_T e_idx_3;
  real32_T sin_Phi_tmp;
  int32_T rtb_MatrixMultiply_tmp;
  real32_T cos_Phi_tmp;
  boolean_T exitg1;
  boolean_T exitg2;

  // MATLAB Function: '<S16>/Euler Angles to Rotation Matrix'
  rtb_M_bg[0] = -4.37113883E-8F;
  rtb_M_bg[3] = -0.0F;
  rtb_M_bg[6] = 1.0F;
  rtb_M_bg[1] = -0.0F;
  rtb_M_bg[4] = 1.0F;
  rtb_M_bg[7] = -0.0F;
  rtb_M_bg[2] = -1.0F;
  rtb_M_bg[5] = -0.0F;
  rtb_M_bg[8] = -4.37113883E-8F;

  // MATLAB Function: '<S7>/Quaternions to Rotation Matrix' incorporates:
  //   Inport: '<Root>/measure'
  //   SignalConversion: '<Root>/BusConversion_InsertedFor_measure_at_outport_0'

  cos_Phi = 1.29246971E-26F;
  cos_Theta = std::abs(rtU.measure.q_bg[0]);
  if (cos_Theta > 1.29246971E-26F) {
    sin_Phi = 1.0F;
    cos_Phi = cos_Theta;
  } else {
    sin_Theta = cos_Theta / 1.29246971E-26F;
    sin_Phi = sin_Theta * sin_Theta;
  }

  cos_Theta = std::abs(rtU.measure.q_bg[1]);
  if (cos_Theta > cos_Phi) {
    sin_Theta = cos_Phi / cos_Theta;
    sin_Phi = sin_Phi * sin_Theta * sin_Theta + 1.0F;
    cos_Phi = cos_Theta;
  } else {
    sin_Theta = cos_Theta / cos_Phi;
    sin_Phi += sin_Theta * sin_Theta;
  }

  cos_Theta = std::abs(rtU.measure.q_bg[2]);
  if (cos_Theta > cos_Phi) {
    sin_Theta = cos_Phi / cos_Theta;
    sin_Phi = sin_Phi * sin_Theta * sin_Theta + 1.0F;
    cos_Phi = cos_Theta;
  } else {
    sin_Theta = cos_Theta / cos_Phi;
    sin_Phi += sin_Theta * sin_Theta;
  }

  cos_Theta = std::abs(rtU.measure.q_bg[3]);
  if (cos_Theta > cos_Phi) {
    sin_Theta = cos_Phi / cos_Theta;
    sin_Phi = sin_Phi * sin_Theta * sin_Theta + 1.0F;
    cos_Phi = cos_Theta;
  } else {
    sin_Theta = cos_Theta / cos_Phi;
    sin_Phi += sin_Theta * sin_Theta;
  }

  sin_Phi = cos_Phi * std::sqrt(sin_Phi);
  rtb_u0[0] = rtU.measure.q_bg[0] / sin_Phi;
  rtb_u0[1] = rtU.measure.q_bg[1] / sin_Phi;
  rtb_u0[2] = rtU.measure.q_bg[2] / sin_Phi;
  rtb_u0[3] = rtU.measure.q_bg[3] / sin_Phi;
  sin_Phi = rtb_u0[0] * rtb_u0[0];
  cos_Phi = rtb_u0[1] * rtb_u0[1];
  cos_Theta = rtb_u0[2] * rtb_u0[2];
  sin_Theta = rtb_u0[3] * rtb_u0[3];
  sin_Psi = rtb_u0[0] * rtb_u0[1];
  cos_Psi = rtb_u0[0] * rtb_u0[2];
  q0_q3 = rtb_u0[0] * rtb_u0[3];
  q1_q2 = rtb_u0[1] * rtb_u0[2];
  q1_q3 = rtb_u0[1] * rtb_u0[3];
  q2_q3 = rtb_u0[2] * rtb_u0[3];
  sin_Phi_0[0] = ((sin_Phi + cos_Phi) - cos_Theta) - sin_Theta;
  sin_Phi_0[3] = (q1_q2 + q0_q3) * 2.0F;
  sin_Phi_0[6] = (q1_q3 - cos_Psi) * 2.0F;
  sin_Phi_0[1] = (q1_q2 - q0_q3) * 2.0F;
  sin_Phi_tmp = sin_Phi - cos_Phi;
  sin_Phi_0[4] = (sin_Phi_tmp + cos_Theta) - sin_Theta;
  sin_Phi_0[7] = (q2_q3 + sin_Psi) * 2.0F;
  sin_Phi_0[2] = (q1_q3 + cos_Psi) * 2.0F;
  sin_Phi_0[5] = (q2_q3 - sin_Psi) * 2.0F;
  sin_Phi_0[8] = (sin_Phi_tmp - cos_Theta) + sin_Theta;

  // End of MATLAB Function: '<S7>/Quaternions to Rotation Matrix'

  // DiscreteIntegrator: '<S23>/x'
  sin_Theta = rtDW.x_DSTATE_c;
  for (b_k = 0; b_k < 3; b_k++) {
    // Product: '<S7>/Matrix Multiply'
    rtb_MatrixMultiply_l[b_k] = 0.0F;
    for (aoffset = 0; aoffset < 3; aoffset++) {
      // Product: '<S16>/Matrix Multiply' incorporates:
      //   Product: '<S7>/Matrix Multiply'

      rtb_MatrixMultiply_tmp = b_k + 3 * aoffset;
      rtb_MatrixMultiply[rtb_MatrixMultiply_tmp] = 0.0F;
      i = 3 * aoffset + b_k;
      rtb_MatrixMultiply[rtb_MatrixMultiply_tmp] = rtb_MatrixMultiply[i] +
        sin_Phi_0[3 * aoffset] * rtb_M_bg[b_k];
      rtb_MatrixMultiply[rtb_MatrixMultiply_tmp] = sin_Phi_0[3 * aoffset + 1] *
        rtb_M_bg[b_k + 3] + rtb_MatrixMultiply[i];
      rtb_MatrixMultiply[rtb_MatrixMultiply_tmp] = sin_Phi_0[3 * aoffset + 2] *
        rtb_M_bg[b_k + 6] + rtb_MatrixMultiply[i];

      // Product: '<S7>/Matrix Multiply' incorporates:
      //   Inport: '<Root>/measure'
      //   SignalConversion: '<Root>/BusConversion_InsertedFor_measure_at_outport_0'

      rtb_MatrixMultiply_l[b_k] += rtb_M_bg[i] * rtU.measure.omega_Kb[aoffset];
    }
  }

  // MATLAB Function: '<S16>/Rotations matrix to Euler angles'
  rtb_EulerAngles_idx_2 = std::atan2(rtb_MatrixMultiply[7], rtb_MatrixMultiply[8]);
  tmp = std::asin(rtb_MatrixMultiply[6]);

  // MATLAB Function: '<S7>/Euler angles derivative' incorporates:
  //   MATLAB Function: '<S16>/Rotations matrix to Euler angles'
  //   MATLAB Function: '<S7>/Euler angles second derivate'

  sin_Phi_tmp = std::sin(rtb_EulerAngles_idx_2);
  sin_Phi = sin_Phi_tmp;
  cos_Phi_tmp = std::cos(rtb_EulerAngles_idx_2);
  cos_Theta = std::cos(-tmp);
  q1_q2 = std::tan(-tmp);
  sin_Phi_0[0] = 1.0F;
  sin_Phi_0[3] = sin_Phi_tmp * q1_q2;
  sin_Phi_0[6] = cos_Phi_tmp * q1_q2;
  sin_Phi_0[1] = 0.0F;
  sin_Phi_0[4] = cos_Phi_tmp;
  sin_Phi_0[7] = -sin_Phi_tmp;
  sin_Phi_0[2] = 0.0F;
  sin_Phi_0[5] = sin_Phi_tmp / cos_Theta;
  sin_Phi_0[8] = cos_Phi_tmp / cos_Theta;
  for (b_k = 0; b_k < 3; b_k++) {
    rtb_eulerAngles_dt[b_k] = sin_Phi_0[b_k + 6] * rtb_MatrixMultiply_l[2] +
      (sin_Phi_0[b_k + 3] * rtb_MatrixMultiply_l[1] + sin_Phi_0[b_k] *
       rtb_MatrixMultiply_l[0]);
  }

  // End of MATLAB Function: '<S7>/Euler angles derivative'

  // MATLAB Function: '<S10>/wrap angle' incorporates:
  //   DiscreteIntegrator: '<S10>/Discrete-Time Integrator'

  wrapangle(rtDW.DiscreteTimeIntegrator_DSTATE, &q0_q3);

  // MATLAB Function: '<S9>/wrap angle' incorporates:
  //   MATLAB Function: '<S16>/Rotations matrix to Euler angles'

  wrapangle(std::atan2(rtb_MatrixMultiply[3], rtb_MatrixMultiply[0]), &sin_Phi);

  // MATLAB Function: '<S9>/angle error'
  cos_Psi = q0_q3 - sin_Phi;
  if (cos_Psi > 3.1415926535897931) {
    cos_Psi -= 6.28318548F;
  } else {
    if (cos_Psi < -3.1415926535897931) {
      cos_Psi += 6.28318548F;
    }
  }

  // End of MATLAB Function: '<S9>/angle error'

  // Gain: '<S24>/omega^2' incorporates:
  //   DiscreteIntegrator: '<S24>/x'
  //   DiscreteIntegrator: '<S24>/x_dt'
  //   Gain: '<S10>/angle_max'
  //   Gain: '<S24>/2*d//omega'
  //   Inport: '<Root>/cmd'
  //   Sum: '<S24>/Sum2'
  //   Sum: '<S24>/Sum3'

  q2_q3 = (0.785398185F * rtU.cmd.roll - (0.2F * rtDW.x_dt_DSTATE[0] +
            rtDW.x_DSTATE[0])) * 81.0F;
  cos_Phi = (0.785398185F * rtU.cmd.pitch - (0.2F * rtDW.x_dt_DSTATE[1] +
              rtDW.x_DSTATE[1])) * 81.0F;

  // Gain: '<S23>/1//T' incorporates:
  //   DiscreteIntegrator: '<S23>/x'
  //   Gain: '<S10>/r_max'
  //   Inport: '<Root>/cmd'
  //   Sum: '<S23>/Sum2'

  sin_Psi = (2.09439516F * rtU.cmd.yaw - rtDW.x_DSTATE_c) * 3.33333325F;

  // MATLAB Function: '<S7>/Euler angles second derivate' incorporates:
  //   MATLAB Function: '<S16>/Rotations matrix to Euler angles'

  if (-1.5550883635269477 < -tmp) {
    sin_Phi = -tmp;
  } else {
    sin_Phi = -1.5550884F;
  }

  if (1.5550883635269477 > sin_Phi) {
    q0_q3 = sin_Phi;
  } else {
    q0_q3 = 1.5550884F;
  }

  cos_Theta = std::cos(q0_q3);
  q1_q2 = std::tan(q0_q3);
  q1_q3 = cos_Theta * cos_Theta;

  // Fcn: '<S2>/Fcn' incorporates:
  //   Inport: '<Root>/cmd'

  q0_q3 = 1500.0F - 500.0F * rtU.cmd.thr;

  // MATLAB Function: '<S8>/MATLAB Function1' incorporates:
  //   DiscreteIntegrator: '<S6>/x'

  rtb_u0[0] = ((1100.0F - rtDW.x_DSTATE_cg[0]) + (1900.0F - rtDW.x_DSTATE_cg[0]))
    * 0.5F;

  // MATLAB Function: '<S8>/MATLAB Function'
  unusedU0[0] = 0.0F;

  // MATLAB Function: '<S8>/MATLAB Function1' incorporates:
  //   DiscreteIntegrator: '<S6>/x'

  rtb_umin[0] = 1100.0F - rtDW.x_DSTATE_cg[0];
  rtb_umax[0] = 1900.0F - rtDW.x_DSTATE_cg[0];
  rtb_u0[1] = ((1100.0F - rtDW.x_DSTATE_cg[1]) + (1900.0F - rtDW.x_DSTATE_cg[1]))
    * 0.5F;

  // MATLAB Function: '<S8>/MATLAB Function'
  unusedU0[1] = 0.0F;

  // MATLAB Function: '<S8>/MATLAB Function1' incorporates:
  //   DiscreteIntegrator: '<S6>/x'

  rtb_umin[1] = 1100.0F - rtDW.x_DSTATE_cg[1];
  rtb_umax[1] = 1900.0F - rtDW.x_DSTATE_cg[1];
  rtb_u0[2] = ((1000.0F - rtDW.x_DSTATE_cg[2]) + (2000.0F - rtDW.x_DSTATE_cg[2]))
    * 0.5F;

  // MATLAB Function: '<S8>/MATLAB Function'
  unusedU0[2] = 0.0F;

  // MATLAB Function: '<S8>/MATLAB Function1' incorporates:
  //   DiscreteIntegrator: '<S6>/x'

  rtb_umin[2] = 1000.0F - rtDW.x_DSTATE_cg[2];
  rtb_umax[2] = 2000.0F - rtDW.x_DSTATE_cg[2];
  rtb_u0[3] = ((1000.0F - rtDW.x_DSTATE_cg[3]) + (2000.0F - rtDW.x_DSTATE_cg[3]))
    * 0.5F;

  // MATLAB Function: '<S8>/MATLAB Function'
  unusedU0[3] = 0.0F;

  // MATLAB Function: '<S8>/MATLAB Function1' incorporates:
  //   DiscreteIntegrator: '<S6>/x'

  rtb_umin[3] = 1000.0F - rtDW.x_DSTATE_cg[3];
  rtb_umax[3] = 2000.0F - rtDW.x_DSTATE_cg[3];

  // MATLAB Function: '<S8>/MATLAB Function' incorporates:
  //   Constant: '<S2>/Constant2'
  //   Constant: '<S8>/Constant5'
  //   Constant: '<S8>/Constant6'

  for (i = 0; i < 9; i++) {
    rtb_M_bg[i] = 316.227753F * rtConstP.Constant5_Value[i];
  }

  for (b_k = 0; b_k < 4; b_k++) {
    for (aoffset = 0; aoffset < 3; aoffset++) {
      rtb_MatrixMultiply_tmp = aoffset + 3 * b_k;
      rtb_M_bg_0[rtb_MatrixMultiply_tmp] = 0.0F;
      i = 3 * b_k + aoffset;
      rtb_M_bg_0[rtb_MatrixMultiply_tmp] = rtb_M_bg_0[i] +
        rtConstP.Constant2_Value[3 * b_k] * rtb_M_bg[aoffset];
      rtb_M_bg_0[rtb_MatrixMultiply_tmp] = rtConstP.Constant2_Value[3 * b_k + 1]
        * rtb_M_bg[aoffset + 3] + rtb_M_bg_0[i];
      rtb_M_bg_0[rtb_MatrixMultiply_tmp] = rtConstP.Constant2_Value[3 * b_k + 2]
        * rtb_M_bg[aoffset + 6] + rtb_M_bg_0[i];
    }
  }

  for (b_k = 0; b_k < 4; b_k++) {
    A[7 * b_k] = rtb_M_bg_0[3 * b_k];
    A[1 + 7 * b_k] = rtb_M_bg_0[3 * b_k + 1];
    A[2 + 7 * b_k] = rtb_M_bg_0[3 * b_k + 2];
    rtb_MatrixMultiply_tmp = b_k << 2;
    A[3 + 7 * b_k] = rtConstP.Constant6_Value[rtb_MatrixMultiply_tmp];
    A[4 + 7 * b_k] = rtConstP.Constant6_Value[rtb_MatrixMultiply_tmp + 1];
    A[5 + 7 * b_k] = rtConstP.Constant6_Value[rtb_MatrixMultiply_tmp + 2];
    A[6 + 7 * b_k] = rtConstP.Constant6_Value[rtb_MatrixMultiply_tmp + 3];
  }

  // Sum: '<S2>/Add1'
  rtb_omega2_0[2] = sin_Psi;

  // Sum: '<S9>/error1 ' incorporates:
  //   DiscreteIntegrator: '<S23>/x'

  tmp_0[2] = rtDW.x_DSTATE_c;

  // Sum: '<S2>/Add1'
  rtb_omega2_0[0] = q2_q3;

  // Sum: '<S9>/error1 ' incorporates:
  //   DiscreteIntegrator: '<S24>/x'
  //   DiscreteIntegrator: '<S24>/x_dt'

  tmp_0[0] = rtDW.x_dt_DSTATE[0];
  tmp_0[3] = rtDW.x_DSTATE[0];

  // Sum: '<S2>/Add1'
  rtb_omega2_0[1] = cos_Phi;

  // Sum: '<S9>/error1 ' incorporates:
  //   DiscreteIntegrator: '<S24>/x'
  //   DiscreteIntegrator: '<S24>/x_dt'
  //   MATLAB Function: '<S16>/Rotations matrix to Euler angles'

  tmp_0[1] = rtDW.x_dt_DSTATE[1];
  tmp_0[4] = rtDW.x_DSTATE[1];
  rtb_eulerAngles_dt_0[0] = rtb_eulerAngles_dt[0];
  rtb_eulerAngles_dt_0[1] = rtb_eulerAngles_dt[1];
  rtb_eulerAngles_dt_0[2] = rtb_eulerAngles_dt[2];
  rtb_eulerAngles_dt_0[3] = rtb_EulerAngles_idx_2;
  rtb_eulerAngles_dt_0[4] = -tmp;

  // SignalConversion: '<S4>/TmpSignal ConversionAtGainInport1' incorporates:
  //   Sum: '<S9>/error1 '

  for (b_k = 0; b_k < 5; b_k++) {
    tmp_1[b_k] = tmp_0[b_k] - rtb_eulerAngles_dt_0[b_k];
  }

  tmp_1[5] = cos_Psi;

  // MATLAB Function: '<S7>/Euler angles second derivate' incorporates:
  //   DiscreteIntegrator: '<S14>/x'

  sin_Phi_0[0] = 1.0F;
  rtb_EulerAngles_idx_2 = sin_Phi_tmp * q1_q2;
  sin_Phi_0[3] = rtb_EulerAngles_idx_2;
  tmp = cos_Phi_tmp * q1_q2;
  sin_Phi_0[6] = tmp;
  sin_Phi_0[1] = 0.0F;
  sin_Phi_0[4] = cos_Phi_tmp;
  sin_Phi_0[7] = -sin_Phi_tmp;
  sin_Phi_0[2] = 0.0F;
  sin_Phi_0[5] = sin_Phi_tmp / cos_Theta;
  sin_Phi = cos_Phi_tmp / cos_Theta;
  sin_Phi_0[8] = sin_Phi;
  rtb_MatrixMultiply[0] = 0.0F;
  rtb_MatrixMultiply[3] = tmp * rtb_eulerAngles_dt[0];
  rtb_MatrixMultiply[6] = -sin_Phi_tmp * q1_q2 * rtb_eulerAngles_dt[0];
  rtb_MatrixMultiply[1] = 0.0F;
  rtb_MatrixMultiply[4] = rtb_eulerAngles_dt[0] * -sin_Phi_tmp;
  rtb_MatrixMultiply[7] = rtb_eulerAngles_dt[0] * -cos_Phi_tmp;
  rtb_MatrixMultiply[2] = 0.0F;
  rtb_MatrixMultiply[5] = sin_Phi * rtb_eulerAngles_dt[0];
  rtb_MatrixMultiply[8] = -sin_Phi_tmp / cos_Theta * rtb_eulerAngles_dt[0];
  rtb_eulerAngles_dt_1[0] = 0.0F;
  rtb_eulerAngles_dt_1[3] = sin_Phi_tmp / q1_q3 * rtb_eulerAngles_dt[1];
  rtb_eulerAngles_dt_1[6] = cos_Phi_tmp / q1_q3 * rtb_eulerAngles_dt[1];
  rtb_eulerAngles_dt_1[2] = 0.0F;
  rtb_eulerAngles_dt_1[5] = rtb_EulerAngles_idx_2 / cos_Theta *
    rtb_eulerAngles_dt[1];
  rtb_eulerAngles_dt_1[8] = tmp / cos_Theta * rtb_eulerAngles_dt[1];
  for (b_k = 0; b_k < 3; b_k++) {
    rtb_eulerAngles_dt_1[1 + 3 * b_k] = 0.0F;

    // Sum: '<S2>/Add1' incorporates:
    //   Gain: '<S4>/Gain'
    //   SignalConversion: '<S4>/TmpSignal ConversionAtGainInport1'

    rtb_EulerAngles_idx_2 = 0.0F;
    for (aoffset = 0; aoffset < 6; aoffset++) {
      rtb_EulerAngles_idx_2 += rtConstP.Gain_Gain[3 * aoffset + b_k] *
        tmp_1[aoffset];
    }

    rtb_omega2_1[b_k] = rtb_omega2_0[b_k] + rtb_EulerAngles_idx_2;
    tmp_2[b_k] = sin_Phi_0[b_k + 6] * rtDW.x_DSTATE_o[2] + (sin_Phi_0[b_k + 3] *
      rtDW.x_DSTATE_o[1] + sin_Phi_0[b_k] * rtDW.x_DSTATE_o[0]);
    rtb_eulerAngles_dt[b_k] = rtb_MatrixMultiply[b_k + 6] *
      rtb_MatrixMultiply_l[2] + rtb_MatrixMultiply[b_k + 3] *
      rtb_MatrixMultiply_l[1];
  }

  for (b_k = 0; b_k < 3; b_k++) {
    // Sum: '<S2>/Add' incorporates:
    //   Constant: '<S2>/Constant1'
    //   Gain: '<S2>/Gain'
    //   Product: '<S2>/MatrixMultiply1'
    //   Sum: '<S2>/Add1'
    //   UnitDelay: '<S2>/Unit Delay'

    rtb_omega2_0[b_k] = (rtb_omega2_1[b_k] - ((rtb_eulerAngles_dt_1[b_k + 3] *
      rtb_MatrixMultiply_l[1] + rtb_eulerAngles_dt_1[b_k + 6] *
      rtb_MatrixMultiply_l[2]) + (tmp_2[b_k] + rtb_eulerAngles_dt[b_k]))) +
      (((300.0F * rtDW.UnitDelay_DSTATE[0] * rtConstP.Constant1_Value[b_k] +
         rtConstP.Constant1_Value[b_k + 3] * (300.0F * rtDW.UnitDelay_DSTATE[1]))
        + rtConstP.Constant1_Value[b_k + 6] * (300.0F * rtDW.UnitDelay_DSTATE[2]))
       + rtConstP.Constant1_Value[b_k + 9] * (300.0F * rtDW.UnitDelay_DSTATE[3]));
  }

  // MATLAB Function: '<S8>/MATLAB Function1' incorporates:
  //   Constant: '<S8>/Constant7'
  //   DiscreteIntegrator: '<S6>/x'
  //   Gain: '<S2>/Gain1'
  //   Sum: '<S8>/Add'

  cos_Theta = q0_q3 - rtDW.x_DSTATE_cg[2];
  q0_q3 -= rtDW.x_DSTATE_cg[3];

  // MATLAB Function: '<S8>/MATLAB Function' incorporates:
  //   Constant: '<S8>/Constant6'
  //   Constant: '<S8>/Constant7'
  //   DiscreteIntegrator: '<S6>/x'
  //   MATLAB Function: '<S8>/MATLAB Function1'
  //   Sum: '<S8>/Add'

  for (b_k = 0; b_k < 3; b_k++) {
    rtb_eulerAngles_dt[b_k] = rtb_M_bg[b_k + 6] * rtb_omega2_0[2] +
      (rtb_M_bg[b_k + 3] * rtb_omega2_0[1] + rtb_M_bg[b_k] * rtb_omega2_0[0]);
  }

  for (b_k = 0; b_k < 4; b_k++) {
    rtb_EulerAngles_idx_2 = rtConstP.Constant6_Value[b_k + 12] * q0_q3 +
      (rtConstP.Constant6_Value[b_k + 8] * cos_Theta +
       (rtConstP.Constant6_Value[b_k + 4] * (1500.0F - rtDW.x_DSTATE_cg[1]) +
        (1500.0F - rtDW.x_DSTATE_cg[0]) * rtConstP.Constant6_Value[b_k]));
    p_free_data[b_k] = rtb_EulerAngles_idx_2;
  }

  rtb_M_bg_1[0] = rtb_eulerAngles_dt[0];
  rtb_M_bg_1[1] = rtb_eulerAngles_dt[1];
  rtb_M_bg_1[2] = rtb_eulerAngles_dt[2];
  rtb_M_bg_1[3] = p_free_data[0];
  rtb_M_bg_1[4] = p_free_data[1];
  rtb_M_bg_1[5] = p_free_data[2];
  rtb_M_bg_1[6] = p_free_data[3];
  for (b_k = 0; b_k < 7; b_k++) {
    q0_q3 = A[b_k + 21] * rtb_u0[3] + (A[b_k + 14] * rtb_u0[2] + (A[b_k + 7] *
      rtb_u0[1] + A[b_k] * rtb_u0[0]));
    d[b_k] = rtb_M_bg_1[b_k] - q0_q3;
  }

  i_free[0] = true;
  i_free[1] = true;
  i_free[2] = true;
  i_free[3] = true;
  rtb_MatrixMultiply_tmp = 0;
  exitg1 = false;
  while ((!exitg1) && (rtb_MatrixMultiply_tmp <= 99)) {
    trueCount = 0;
    if (i_free[0]) {
      trueCount = 1;
    }

    if (i_free[1]) {
      trueCount++;
    }

    if (i_free[2]) {
      trueCount++;
    }

    if (i_free[3]) {
      trueCount++;
    }

    i = trueCount;
    trueCount = 0;
    if (i_free[0]) {
      f_data[0] = 1;
      trueCount = 1;
    }

    if (i_free[1]) {
      f_data[trueCount] = 2;
      trueCount++;
    }

    if (i_free[2]) {
      f_data[trueCount] = 3;
      trueCount++;
    }

    if (i_free[3]) {
      f_data[trueCount] = 4;
    }

    A_free_size[0] = 7;
    A_free_size[1] = i;
    for (b_k = 0; b_k < i; b_k++) {
      for (aoffset = 0; aoffset < 7; aoffset++) {
        A_free_data[aoffset + 7 * b_k] = A[(f_data[b_k] - 1) * 7 + aoffset];
      }
    }

    mldivide(A_free_data, A_free_size, d, p_free_data, &b_aoffset);
    trueCount = 0;
    p_0 = 0.0;
    if (i_free[0]) {
      p_0 = p_free_data[0];
      trueCount = 1;
    }

    u_opt[0] = rtb_u0[0] + (real32_T)p_0;
    p[0] = p_0;
    p_0 = 0.0;
    if (i_free[1]) {
      p_0 = p_free_data[trueCount];
      trueCount++;
    }

    u_opt[1] = rtb_u0[1] + (real32_T)p_0;
    p[1] = p_0;
    p_0 = 0.0;
    if (i_free[2]) {
      p_0 = p_free_data[trueCount];
      trueCount++;
    }

    u_opt[2] = rtb_u0[2] + (real32_T)p_0;
    p[2] = p_0;
    p_0 = 0.0;
    if (i_free[3]) {
      p_0 = p_free_data[trueCount];
    }

    u_opt[3] = rtb_u0[3] + (real32_T)p_0;
    p[3] = p_0;
    trueCount = 0;
    if (i_free[0]) {
      trueCount = 1;
    }

    if (i_free[1]) {
      trueCount++;
    }

    if (i_free[2]) {
      trueCount++;
    }

    if (i_free[3]) {
      trueCount++;
    }

    aoffset = trueCount;
    trueCount = 0;
    if (i_free[0]) {
      g_data[0] = 1;
      trueCount = 1;
    }

    u_opt_0[0] = ((u_opt[0] < rtb_umin[0]) || (u_opt[0] > rtb_umax[0]));
    if (i_free[1]) {
      g_data[trueCount] = 2;
      trueCount++;
    }

    u_opt_0[1] = ((u_opt[1] < rtb_umin[1]) || (u_opt[1] > rtb_umax[1]));
    if (i_free[2]) {
      g_data[trueCount] = 3;
      trueCount++;
    }

    u_opt_0[2] = ((u_opt[2] < rtb_umin[2]) || (u_opt[2] > rtb_umax[2]));
    if (i_free[3]) {
      g_data[trueCount] = 4;
    }

    u_opt_0[3] = ((u_opt[3] < 1000.0F - rtDW.x_DSTATE_cg[3]) || (u_opt[3] >
      2000.0F - rtDW.x_DSTATE_cg[3]));
    for (b_k = 0; b_k < aoffset; b_k++) {
      u_opt_data[b_k] = u_opt_0[g_data[b_k] - 1];
    }

    if (!any(u_opt_data, &aoffset)) {
      rtb_u0[0] = u_opt[0];
      rtb_u0[1] = u_opt[1];
      rtb_u0[2] = u_opt[2];
      rtb_u0[3] = u_opt[3];
      if (i == 1) {
        for (b_k = 0; b_k < 7; b_k++) {
          rtb_M_bg_1[b_k] = 0.0F;
          for (aoffset = 0; aoffset < i; aoffset++) {
            rtb_M_bg_1[b_k] += A_free_data[7 * aoffset + b_k] *
              p_free_data[aoffset];
          }
        }
      } else if (b_aoffset == 1) {
        for (b_k = 0; b_k < 7; b_k++) {
          rtb_M_bg_1[b_k] = 0.0F;
          for (aoffset = 0; aoffset < i; aoffset++) {
            rtb_M_bg_1[b_k] += A_free_data[7 * aoffset + b_k] *
              p_free_data[aoffset];
          }
        }
      } else {
        for (trueCount = 0; trueCount < 7; trueCount++) {
          rtb_M_bg_1[trueCount] = 0.0F;
        }

        for (trueCount = 0; trueCount < i; trueCount++) {
          b_aoffset = trueCount * 7;
          for (b_k = 0; b_k < 7; b_k++) {
            aoffset = b_aoffset + b_k;
            rtb_M_bg_1[b_k] += A[(f_data[aoffset / 7] - 1) * 7 + aoffset % 7] *
              p_free_data[trueCount];
          }
        }
      }

      for (b_k = 0; b_k < 7; b_k++) {
        d[b_k] -= rtb_M_bg_1[b_k];
      }

      for (i = 0; i < 4; i++) {
        p_free_data[i] = 0.0F;
        for (b_k = 0; b_k < 7; b_k++) {
          p_free_data[i] += A[7 * i + b_k] * d[b_k];
        }

        q0_q3 = unusedU0[i] * p_free_data[i];
        x[i] = (q0_q3 >= -2.22044605E-16F);
        u_opt[i] = q0_q3;
      }

      x_0 = true;
      trueCount = 0;
      exitg2 = false;
      while ((!exitg2) && (trueCount < 4)) {
        if (!x[trueCount]) {
          x_0 = false;
          exitg2 = true;
        } else {
          trueCount++;
        }
      }

      if (x_0) {
        exitg1 = true;
      } else {
        sin_Phi = u_opt[0];
        trueCount = 0;
        if (u_opt[0] > u_opt[1]) {
          sin_Phi = u_opt[1];
          trueCount = 1;
        }

        if (sin_Phi > u_opt[2]) {
          sin_Phi = u_opt[2];
          trueCount = 2;
        }

        if (sin_Phi > u_opt[3]) {
          trueCount = 3;
        }

        unusedU0[trueCount] = 0.0F;
        i_free[trueCount] = true;
        rtb_MatrixMultiply_tmp++;
      }
    } else {
      trueCount = 0;
      dist_idx_0 = 1.0;
      x_0 = (p[0] < 0.0);
      e_idx_0 = (p[0] > 0.0);
      if (i_free[0] && x_0) {
        trueCount = 1;
      }

      x[0] = x_0;
      dist_idx_1 = 1.0;
      x_0 = (p[1] < 0.0);
      e_idx_1 = (p[1] > 0.0);
      if (i_free[1] && x_0) {
        trueCount++;
      }

      x[1] = x_0;
      dist_idx_2 = 1.0;
      x_0 = (p[2] < 0.0);
      e_idx_2 = (p[2] > 0.0);
      if (i_free[2] && x_0) {
        trueCount++;
      }

      x[2] = x_0;
      dist_idx_3 = 1.0;
      x_0 = (p_0 < 0.0);
      e_idx_3 = (p_0 > 0.0);
      if (i_free[3] && x_0) {
        trueCount++;
      }

      aoffset = trueCount;
      trueCount = 0;
      if (i_free[0] && x[0]) {
        h_data[0] = 1;
        trueCount = 1;
      }

      if (i_free[1] && x[1]) {
        h_data[trueCount] = 2;
        trueCount++;
      }

      if (i_free[2] && x[2]) {
        h_data[trueCount] = 3;
        trueCount++;
      }

      if (i_free[3] && x_0) {
        h_data[trueCount] = 4;
      }

      for (b_k = 0; b_k < aoffset; b_k++) {
        trueCount = h_data[b_k] - 1;
        c_data[b_k] = (rtb_umin[trueCount] - rtb_u0[trueCount]) / (real32_T)
          p[trueCount];
      }

      trueCount = 0;
      if (i_free[0] && x[0]) {
        dist_idx_0 = c_data[0];
        trueCount = 1;
      }

      if (i_free[1] && x[1]) {
        dist_idx_1 = c_data[trueCount];
        trueCount++;
      }

      if (i_free[2] && x[2]) {
        dist_idx_2 = c_data[trueCount];
        trueCount++;
      }

      if (i_free[3] && x_0) {
        dist_idx_3 = c_data[trueCount];
      }

      trueCount = 0;
      if (i_free[0] && e_idx_0) {
        trueCount = 1;
      }

      if (i_free[1] && e_idx_1) {
        trueCount++;
      }

      if (i_free[2] && e_idx_2) {
        trueCount++;
      }

      if (i_free[3] && e_idx_3) {
        trueCount++;
      }

      aoffset = trueCount;
      trueCount = 0;
      if (i_free[0] && e_idx_0) {
        i_data[0] = 1;
        trueCount = 1;
      }

      if (i_free[1] && e_idx_1) {
        i_data[trueCount] = 2;
        trueCount++;
      }

      if (i_free[2] && e_idx_2) {
        i_data[trueCount] = 3;
        trueCount++;
      }

      if (i_free[3] && e_idx_3) {
        i_data[trueCount] = 4;
      }

      for (b_k = 0; b_k < aoffset; b_k++) {
        trueCount = i_data[b_k] - 1;
        c_data[b_k] = (rtb_umax[trueCount] - rtb_u0[trueCount]) / (real32_T)
          p[trueCount];
      }

      trueCount = 0;
      if (i_free[0] && e_idx_0) {
        dist_idx_0 = c_data[0];
        trueCount = 1;
      }

      if (i_free[1] && e_idx_1) {
        dist_idx_1 = c_data[trueCount];
        trueCount++;
      }

      if (i_free[2] && e_idx_2) {
        dist_idx_2 = c_data[trueCount];
        trueCount++;
      }

      if (i_free[3] && e_idx_3) {
        dist_idx_3 = c_data[trueCount];
      }

      trueCount = 0;
      if (dist_idx_0 > dist_idx_1) {
        dist_idx_0 = dist_idx_1;
        trueCount = 1;
      }

      if (dist_idx_0 > dist_idx_2) {
        dist_idx_0 = dist_idx_2;
        trueCount = 2;
      }

      if (dist_idx_0 > dist_idx_3) {
        dist_idx_0 = dist_idx_3;
        trueCount = 3;
      }

      rtb_u0[0] += (real32_T)(dist_idx_0 * p[0]);
      rtb_u0[1] += (real32_T)(dist_idx_0 * p[1]);
      rtb_u0[2] += (real32_T)(dist_idx_0 * p[2]);
      rtb_u0[3] += (real32_T)(dist_idx_0 * p_0);
      aoffset = 7 * i - 1;
      for (b_k = 0; b_k <= aoffset; b_k++) {
        A_free_data[b_k] *= (real32_T)dist_idx_0;
      }

      if (i == 1) {
        for (b_k = 0; b_k < 7; b_k++) {
          rtb_M_bg_1[b_k] = 0.0F;
          for (aoffset = 0; aoffset < 1; aoffset++) {
            rtb_M_bg_1[b_k] += A_free_data[b_k] * p_free_data[0];
          }
        }
      } else if (b_aoffset == 1) {
        for (b_k = 0; b_k < 7; b_k++) {
          rtb_M_bg_1[b_k] = 0.0F;
          for (aoffset = 0; aoffset < i; aoffset++) {
            rtb_M_bg_1[b_k] += A_free_data[7 * aoffset + b_k] *
              p_free_data[aoffset];
          }
        }
      } else {
        for (b_aoffset = 0; b_aoffset < 7; b_aoffset++) {
          rtb_M_bg_1[b_aoffset] = 0.0F;
        }

        for (b_k = 0; b_k < i; b_k++) {
          aoffset = b_k * 7;
          for (b_aoffset = 0; b_aoffset < 7; b_aoffset++) {
            rtb_M_bg_1[b_aoffset] += A_free_data[aoffset + b_aoffset] *
              p_free_data[b_k];
          }
        }
      }

      for (b_k = 0; b_k < 7; b_k++) {
        d[b_k] -= rtb_M_bg_1[b_k];
      }

      if (p[trueCount] < 0.0) {
        unusedU0[trueCount] = -1.0F;
      } else if (p[trueCount] > 0.0) {
        unusedU0[trueCount] = 1.0F;
      } else {
        unusedU0[trueCount] = (real32_T)p[trueCount];
      }

      i_free[trueCount] = false;
      rtb_MatrixMultiply_tmp++;
    }
  }

  // Sum: '<S2>/Add3' incorporates:
  //   DiscreteIntegrator: '<S6>/x'

  q0_q3 = rtb_u0[0] + rtDW.x_DSTATE_cg[0];

  // Saturate: '<S2>/Saturation1'
  if (q0_q3 > 2000.0F) {
    cos_Theta = 2000.0F;
  } else if (q0_q3 < 1000.0F) {
    cos_Theta = 1000.0F;
  } else {
    cos_Theta = q0_q3;
  }

  // Sum: '<S2>/Add3' incorporates:
  //   DiscreteIntegrator: '<S6>/x'

  rtb_umin[0] = q0_q3;
  q0_q3 = rtb_u0[1] + rtDW.x_DSTATE_cg[1];

  // Saturate: '<S2>/Saturation1'
  if (q0_q3 > 2000.0F) {
    q1_q2 = 2000.0F;
  } else if (q0_q3 < 1000.0F) {
    q1_q2 = 1000.0F;
  } else {
    q1_q2 = q0_q3;
  }

  // Sum: '<S2>/Add3' incorporates:
  //   DiscreteIntegrator: '<S6>/x'

  rtb_umin[1] = q0_q3;
  q0_q3 = rtb_u0[2] + rtDW.x_DSTATE_cg[2];

  // Saturate: '<S2>/Saturation1'
  if (q0_q3 > 2000.0F) {
    q1_q3 = 2000.0F;
  } else if (q0_q3 < 1000.0F) {
    q1_q3 = 1000.0F;
  } else {
    q1_q3 = q0_q3;
  }

  // Sum: '<S2>/Add3' incorporates:
  //   DiscreteIntegrator: '<S6>/x'

  rtb_umin[2] = q0_q3;
  q0_q3 = rtb_u0[3] + rtDW.x_DSTATE_cg[3];

  // Saturate: '<S2>/Saturation1'
  if (q0_q3 > 2000.0F) {
    rtb_EulerAngles_idx_2 = 2000.0F;
  } else if (q0_q3 < 1000.0F) {
    rtb_EulerAngles_idx_2 = 1000.0F;
  } else {
    rtb_EulerAngles_idx_2 = q0_q3;
  }

  // Outport: '<Root>/u' incorporates:
  //   MATLAB Function: '<S3>/MATLAB Function'

  rtY.u[0] = (rtb_EulerAngles_idx_2 - 1500.0F) / 1000.0F + 0.5F;
  rtY.u[1] = (q1_q3 - 1500.0F) / 1000.0F + 0.5F;
  rtY.u[2] = 0.0F;
  rtY.u[3] = 0.0F;
  rtY.u[4] = -((q1_q2 - 1500.0F) / 1000.0F) + 0.5F;
  rtY.u[5] = (cos_Theta - 1500.0F) / 1000.0F + 0.5F;
  rtY.u[6] = 0.0F;
  rtY.u[7] = 0.0F;

  // Sum: '<S6>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<S6>/x'
  //   DiscreteIntegrator: '<S6>/x_dt'
  //   Gain: '<S6>/2*d//omega'
  //   Sum: '<S6>/Sum3'

  rtb_umax[0] = rtDW.A[0] - (0.02F * rtDW.x_dt_DSTATE_m[0] + rtDW.x_DSTATE_cg[0]);
  rtb_umax[1] = rtDW.A[1] - (0.02F * rtDW.x_dt_DSTATE_m[1] + rtDW.x_DSTATE_cg[1]);
  rtb_umax[2] = rtDW.A[2] - (0.02F * rtDW.x_dt_DSTATE_m[2] + rtDW.x_DSTATE_cg[2]);
  rtb_umax[3] = rtDW.A[3] - (0.02F * rtDW.x_dt_DSTATE_m[3] + rtDW.x_DSTATE_cg[3]);

  // SampleTimeMath: '<S11>/TSamp'
  //
  //  About '<S11>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  cos_Theta = rtb_MatrixMultiply_l[0] * 300.0F;

  // Sum: '<S14>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S14>/x'
  //   DiscreteIntegrator: '<S14>/x_dt'
  //   Gain: '<S14>/2*d//omega'
  //   Sum: '<S11>/Diff'
  //   Sum: '<S14>/Sum3'
  //   UnitDelay: '<S11>/UD'
  //
  //  Block description for '<S11>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S11>/UD':
  //
  //   Store in Global RAM

  q1_q2 = (cos_Theta - rtDW.UD_DSTATE[0]) - (0.02F * rtDW.x_dt_DSTATE_h[0] +
    rtDW.x_DSTATE_o[0]);

  // SampleTimeMath: '<S11>/TSamp'
  //
  //  About '<S11>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  rtb_MatrixMultiply_l[0] = cos_Theta;
  cos_Theta = rtb_MatrixMultiply_l[1] * 300.0F;

  // Sum: '<S14>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S14>/x'
  //   DiscreteIntegrator: '<S14>/x_dt'
  //   Gain: '<S14>/2*d//omega'
  //   Sum: '<S11>/Diff'
  //   Sum: '<S14>/Sum3'
  //   UnitDelay: '<S11>/UD'
  //
  //  Block description for '<S11>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S11>/UD':
  //
  //   Store in Global RAM

  q1_q3 = (cos_Theta - rtDW.UD_DSTATE[1]) - (0.02F * rtDW.x_dt_DSTATE_h[1] +
    rtDW.x_DSTATE_o[1]);

  // SampleTimeMath: '<S11>/TSamp'
  //
  //  About '<S11>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  rtb_MatrixMultiply_l[1] = cos_Theta;
  cos_Theta = rtb_MatrixMultiply_l[2] * 300.0F;

  // Sum: '<S14>/Sum2' incorporates:
  //   DiscreteIntegrator: '<S14>/x'
  //   DiscreteIntegrator: '<S14>/x_dt'
  //   Gain: '<S14>/2*d//omega'
  //   Sum: '<S11>/Diff'
  //   Sum: '<S14>/Sum3'
  //   UnitDelay: '<S11>/UD'
  //
  //  Block description for '<S11>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S11>/UD':
  //
  //   Store in Global RAM

  rtb_EulerAngles_idx_2 = (cos_Theta - rtDW.UD_DSTATE[2]) - (0.02F *
    rtDW.x_dt_DSTATE_h[2] + rtDW.x_DSTATE_o[2]);

  // Update for DiscreteIntegrator: '<S24>/x' incorporates:
  //   DiscreteIntegrator: '<S24>/x_dt'

  rtDW.x_DSTATE[0] += 0.00333333341F * rtDW.x_dt_DSTATE[0];

  // Update for DiscreteIntegrator: '<S24>/x_dt'
  rtDW.x_dt_DSTATE[0] += 0.00333333341F * q2_q3;

  // Update for DiscreteIntegrator: '<S24>/x' incorporates:
  //   DiscreteIntegrator: '<S24>/x_dt'

  rtDW.x_DSTATE[1] += 0.00333333341F * rtDW.x_dt_DSTATE[1];

  // Update for DiscreteIntegrator: '<S24>/x_dt'
  rtDW.x_dt_DSTATE[1] += 0.00333333341F * cos_Phi;

  // Update for DiscreteIntegrator: '<S23>/x'
  rtDW.x_DSTATE_c += 0.00333333341F * sin_Psi;

  // Update for DiscreteIntegrator: '<S10>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_DSTATE += 0.00333333341F * sin_Theta;

  // Update for DiscreteIntegrator: '<S14>/x' incorporates:
  //   DiscreteIntegrator: '<S14>/x_dt'

  rtDW.x_DSTATE_o[0] += 0.00333333341F * rtDW.x_dt_DSTATE_h[0];
  rtDW.x_DSTATE_o[1] += 0.00333333341F * rtDW.x_dt_DSTATE_h[1];
  rtDW.x_DSTATE_o[2] += 0.00333333341F * rtDW.x_dt_DSTATE_h[2];

  // Update for UnitDelay: '<S2>/Unit Delay'
  rtDW.UnitDelay_DSTATE[0] = rtb_u0[0];

  // Update for DiscreteIntegrator: '<S6>/x' incorporates:
  //   DiscreteIntegrator: '<S6>/x_dt'

  rtDW.x_DSTATE_cg[0] += 0.00333333341F * rtDW.x_dt_DSTATE_m[0];

  // Update for DiscreteIntegrator: '<S5>/Discrete-Time Integrator' incorporates:
  //   Gain: '<S5>/1//T'
  //   Sum: '<S5>/Sum2'

  rtDW.A[0] += (rtb_umin[0] - rtDW.A[0]) * 25.0F * 0.00333333341F;

  // Update for DiscreteIntegrator: '<S6>/x_dt' incorporates:
  //   Gain: '<S6>/omega^2'

  rtDW.x_dt_DSTATE_m[0] += 10000.0F * rtb_umax[0] * 0.00333333341F;

  // Update for UnitDelay: '<S2>/Unit Delay'
  rtDW.UnitDelay_DSTATE[1] = rtb_u0[1];

  // Update for DiscreteIntegrator: '<S6>/x' incorporates:
  //   DiscreteIntegrator: '<S6>/x_dt'

  rtDW.x_DSTATE_cg[1] += 0.00333333341F * rtDW.x_dt_DSTATE_m[1];

  // Update for DiscreteIntegrator: '<S5>/Discrete-Time Integrator' incorporates:
  //   Gain: '<S5>/1//T'
  //   Sum: '<S5>/Sum2'

  rtDW.A[1] += (rtb_umin[1] - rtDW.A[1]) * 25.0F * 0.00333333341F;

  // Update for DiscreteIntegrator: '<S6>/x_dt' incorporates:
  //   Gain: '<S6>/omega^2'

  rtDW.x_dt_DSTATE_m[1] += 10000.0F * rtb_umax[1] * 0.00333333341F;

  // Update for UnitDelay: '<S2>/Unit Delay'
  rtDW.UnitDelay_DSTATE[2] = rtb_u0[2];

  // Update for DiscreteIntegrator: '<S6>/x' incorporates:
  //   DiscreteIntegrator: '<S6>/x_dt'

  rtDW.x_DSTATE_cg[2] += 0.00333333341F * rtDW.x_dt_DSTATE_m[2];

  // Update for DiscreteIntegrator: '<S5>/Discrete-Time Integrator' incorporates:
  //   Gain: '<S5>/1//T'
  //   Sum: '<S5>/Sum2'

  rtDW.A[2] += (rtb_umin[2] - rtDW.A[2]) * 25.0F * 0.00333333341F;

  // Update for DiscreteIntegrator: '<S6>/x_dt' incorporates:
  //   Gain: '<S6>/omega^2'

  rtDW.x_dt_DSTATE_m[2] += 10000.0F * rtb_umax[2] * 0.00333333341F;

  // Update for UnitDelay: '<S2>/Unit Delay'
  rtDW.UnitDelay_DSTATE[3] = rtb_u0[3];

  // Update for DiscreteIntegrator: '<S6>/x' incorporates:
  //   DiscreteIntegrator: '<S6>/x_dt'

  rtDW.x_DSTATE_cg[3] += 0.00333333341F * rtDW.x_dt_DSTATE_m[3];

  // Update for DiscreteIntegrator: '<S5>/Discrete-Time Integrator' incorporates:
  //   Gain: '<S5>/1//T'
  //   Sum: '<S5>/Sum2'

  rtDW.A[3] += (q0_q3 - rtDW.A[3]) * 25.0F * 0.00333333341F;

  // Update for DiscreteIntegrator: '<S6>/x_dt' incorporates:
  //   Gain: '<S6>/omega^2'

  rtDW.x_dt_DSTATE_m[3] += 10000.0F * rtb_umax[3] * 0.00333333341F;

  // Update for DiscreteIntegrator: '<S14>/x_dt' incorporates:
  //   Gain: '<S14>/omega^2'

  rtDW.x_dt_DSTATE_h[0] += 10000.0F * q1_q2 * 0.00333333341F;

  // Update for UnitDelay: '<S11>/UD'
  //
  //  Block description for '<S11>/UD':
  //
  //   Store in Global RAM

  rtDW.UD_DSTATE[0] = rtb_MatrixMultiply_l[0];

  // Update for DiscreteIntegrator: '<S14>/x_dt' incorporates:
  //   Gain: '<S14>/omega^2'

  rtDW.x_dt_DSTATE_h[1] += 10000.0F * q1_q3 * 0.00333333341F;

  // Update for UnitDelay: '<S11>/UD'
  //
  //  Block description for '<S11>/UD':
  //
  //   Store in Global RAM

  rtDW.UD_DSTATE[1] = rtb_MatrixMultiply_l[1];

  // Update for DiscreteIntegrator: '<S14>/x_dt' incorporates:
  //   Gain: '<S14>/omega^2'

  rtDW.x_dt_DSTATE_h[2] += 10000.0F * rtb_EulerAngles_idx_2 * 0.00333333341F;

  // Update for UnitDelay: '<S11>/UD'
  //
  //  Block description for '<S11>/UD':
  //
  //   Store in Global RAM

  rtDW.UD_DSTATE[2] = cos_Theta;
}

// Model initialize function
void MatlabControllerClass::initialize()
{
  // InitializeConditions for DiscreteIntegrator: '<S6>/x'
  rtDW.x_DSTATE_cg[0] = 1500.0F;

  // InitializeConditions for DiscreteIntegrator: '<S5>/Discrete-Time Integrator' 
  rtDW.A[0] = 1500.0F;

  // InitializeConditions for DiscreteIntegrator: '<S6>/x'
  rtDW.x_DSTATE_cg[1] = 1500.0F;

  // InitializeConditions for DiscreteIntegrator: '<S5>/Discrete-Time Integrator' 
  rtDW.A[1] = 1500.0F;

  // InitializeConditions for DiscreteIntegrator: '<S6>/x'
  rtDW.x_DSTATE_cg[2] = 1000.0F;

  // InitializeConditions for DiscreteIntegrator: '<S5>/Discrete-Time Integrator' 
  rtDW.A[2] = 1000.0F;

  // InitializeConditions for DiscreteIntegrator: '<S6>/x'
  rtDW.x_DSTATE_cg[3] = 1000.0F;

  // InitializeConditions for DiscreteIntegrator: '<S5>/Discrete-Time Integrator' 
  rtDW.A[3] = 1000.0F;
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
