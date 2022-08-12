//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: MatlabController.cpp
//
// Code generated for Simulink model 'EasyGlider_ManualMode_with_waypoints_and_sysID'.
//
// Model version                  : 1.464
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Thu Aug 11 17:08:25 2022
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#include "MatlabController.h"

extern real32_T rt_roundf(real32_T u);
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

// Function for MATLAB Function: '<S9>/trajFromWaypoints'
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

// Function for MATLAB Function: '<S9>/trajFromWaypoints'
void MatlabControllerClass::polyder_a(const real32_T u[6], real32_T a_data[],
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

// Function for MATLAB Function: '<S9>/trajFromWaypoints'
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

// Function for MATLAB Function: '<S9>/trajFromWaypoints'
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
    if ((1.0F + (real32_T)k > traj->num_sections_set) || (1.0F + (real32_T)k <
         1.0F)) {
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
    polyder_a(traj->sections[(int32_T)section_idx - 1].pos_x, dx_data, dx_size);
    polyder_a(traj->sections[(int32_T)section_idx - 1].pos_y, dy_data, dy_size);
    polyder_a(traj->sections[(int32_T)section_idx - 1].pos_z, dz_data, dz_size);
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

// Function for MATLAB Function: '<S9>/trajFromWaypoints'
void MatlabControllerClass::polyder(const real_T u_data[], const int32_T u_size
  [2], real_T a_data[], int32_T a_size[2])
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
    a_data[0] = 0.0;
    break;

   case 1:
    a_data[0] = 0.0;
    break;

   default:
    nlead0 = 0;
    ny = 0;
    while ((ny <= nymax - 2) && (u_data[ny] == 0.0)) {
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
    a_data[ny] *= (real_T)((nlead0 - ny) + 1) + 1.0;
  }
}

// Function for MATLAB Function: '<S9>/trajFromWaypoints'
void MatlabControllerClass::multAx(const real32_T j[3], const real_T k[6], const
  real_T Ajk[18], const real32_T x_data[], real32_T b_data[], real_T dim)
{
  int32_T tmp[3];
  real32_T x[6];
  real32_T b[3];
  int32_T i;
  int32_T i_0;
  real32_T x_idx_0;
  real32_T x_idx_1;
  real32_T x_idx_2;
  if (dim == 2.0) {
    x_idx_0 = x_data[(int32_T)j[0] - 1];
    x_idx_1 = x_data[(int32_T)j[1] - 1];
    x_idx_2 = x_data[(int32_T)j[2] - 1];
    for (i = 0; i < 6; i++) {
      x[i] = (((real32_T)Ajk[3 * i + 1] * x_idx_1 + (real32_T)Ajk[3 * i] *
               x_idx_0) + (real32_T)Ajk[3 * i + 2] * x_idx_2) + b_data[(int32_T)
        k[i] - 1];
    }

    for (i = 0; i < 6; i++) {
      b_data[(int32_T)k[i] - 1] = x[i];
    }
  } else {
    if (dim == 1.0) {
      tmp[0] = (int32_T)j[0];
      tmp[1] = (int32_T)j[1];
      tmp[2] = (int32_T)j[2];
      for (i = 0; i < 6; i++) {
        x[i] = x_data[(int32_T)k[i] - 1];
      }

      for (i = 0; i < 3; i++) {
        x_idx_0 = 0.0F;
        for (i_0 = 0; i_0 < 6; i_0++) {
          x_idx_0 += (real32_T)Ajk[3 * i_0 + i] * x[i_0];
        }

        b[i] = b_data[tmp[i] - 1] + x_idx_0;
      }

      b_data[(int32_T)j[0] - 1] = b[0];
      b_data[(int32_T)j[1] - 1] = b[1];
      b_data[(int32_T)j[2] - 1] = b[2];
    }
  }
}

// Function for MATLAB Function: '<S9>/trajFromWaypoints'
void MatlabControllerClass::__anon_fcn_a(real32_T num_of_splines, real32_T cycle,
  const real32_T x_data[], const int32_T *x_size, real32_T varargout_1_data[],
  int32_T *varargout_1_size)
{
  real_T pp[36];
  real_T point_0[36];
  real32_T intermediate_size;
  int32_T bnd_left;
  int32_T m_data[60];
  real_T point_0_0[18];
  real_T pp_data[6];
  real_T tmp_data[5];
  real32_T varargout_1[3];
  real32_T varargout_1_data_0[60];
  real32_T x[6];
  real32_T varargout_1_0[4];
  real32_T point_0_1;
  int32_T loop_ub;
  int32_T pp_size[2];
  int32_T tmp_size[2];
  int32_T tmp;
  int8_T c_x_idx_0;
  real32_T intermediate_size_tmp;
  int32_T m_size_idx_1_tmp;
  int32_T varargout_1_data_tmp;
  int32_T h_tmp;
  int32_T l_tmp;
  c_x_idx_0 = (int8_T)*x_size;
  *varargout_1_size = c_x_idx_0;
  if (0 <= c_x_idx_0 - 1) {
    memset(&varargout_1_data[0], 0, c_x_idx_0 * sizeof(real32_T));
  }

  for (varargout_1_data_tmp = 0; varargout_1_data_tmp < 36; varargout_1_data_tmp
       ++) {
    pp[varargout_1_data_tmp] = 1.0;
  }

  for (varargout_1_data_tmp = 0; varargout_1_data_tmp < 6; varargout_1_data_tmp
       ++) {
    for (h_tmp = 0; h_tmp < 5; h_tmp++) {
      pp[(h_tmp + 6 * varargout_1_data_tmp) + 1] = 0.0;
    }
  }

  for (bnd_left = 0; bnd_left < 5; bnd_left++) {
    loop_ub = 6 - bnd_left;
    pp_size[0] = 1;
    pp_size[1] = loop_ub;
    for (varargout_1_data_tmp = 0; varargout_1_data_tmp < loop_ub;
         varargout_1_data_tmp++) {
      pp_data[varargout_1_data_tmp] = pp[6 * varargout_1_data_tmp + bnd_left];
    }

    polyder(pp_data, pp_size, tmp_data, tmp_size);
    loop_ub = tmp_size[1];
    for (varargout_1_data_tmp = 0; varargout_1_data_tmp < loop_ub;
         varargout_1_data_tmp++) {
      pp[(bnd_left + 6 * varargout_1_data_tmp) + 1] =
        tmp_data[varargout_1_data_tmp];
    }
  }

  memcpy(&point_0[0], &pp[0], 36U * sizeof(real_T));
  for (bnd_left = 0; bnd_left < 5; bnd_left++) {
    tmp = 5 - bnd_left;
    for (varargout_1_data_tmp = 0; varargout_1_data_tmp < tmp;
         varargout_1_data_tmp++) {
      point_0[bnd_left + 6 * varargout_1_data_tmp] = 0.0;
    }
  }

  intermediate_size_tmp = (num_of_splines - 1.0F) * 6.0F;
  if (cycle == 0.0F) {
    bnd_left = 3;
    varargout_1[0] = 1.0F;
    varargout_1[1] = 2.0F;
    varargout_1[2] = 3.0F;
    for (varargout_1_data_tmp = 0; varargout_1_data_tmp < 6;
         varargout_1_data_tmp++) {
      pp_data[varargout_1_data_tmp] = 1.0 + (real_T)varargout_1_data_tmp;
      point_0_0[3 * varargout_1_data_tmp] = point_0[6 * varargout_1_data_tmp];
      point_0_0[1 + 3 * varargout_1_data_tmp] = point_0[6 * varargout_1_data_tmp
        + 1];
      point_0_0[2 + 3 * varargout_1_data_tmp] = point_0[6 * varargout_1_data_tmp
        + 2];
    }

    multAx(varargout_1, pp_data, point_0_0, x_data, varargout_1_data, 1.0);
    if (4.0F + intermediate_size_tmp > ((4.0F + intermediate_size_tmp) + 3.0F) -
        1.0F) {
      tmp = 0;
      h_tmp = 0;
      l_tmp = 0;
      loop_ub = 0;
    } else {
      tmp = (int32_T)(4.0F + intermediate_size_tmp) - 1;
      h_tmp = (int32_T)(((4.0F + intermediate_size_tmp) + 3.0F) - 1.0F);
      l_tmp = tmp;
      loop_ub = h_tmp;
    }

    m_size_idx_1_tmp = loop_ub - l_tmp;
    for (varargout_1_data_tmp = 0; varargout_1_data_tmp < m_size_idx_1_tmp;
         varargout_1_data_tmp++) {
      m_data[varargout_1_data_tmp] = l_tmp + varargout_1_data_tmp;
    }

    loop_ub = h_tmp - tmp;
    for (varargout_1_data_tmp = 0; varargout_1_data_tmp < loop_ub;
         varargout_1_data_tmp++) {
      varargout_1_data_0[varargout_1_data_tmp] = varargout_1_data[tmp +
        varargout_1_data_tmp];
    }

    for (varargout_1_data_tmp = 0; varargout_1_data_tmp < 6;
         varargout_1_data_tmp++) {
      x[varargout_1_data_tmp] = x_data[(int32_T)((1.0F + intermediate_size_tmp)
        + (real32_T)varargout_1_data_tmp) - 1];
    }

    for (varargout_1_data_tmp = 0; varargout_1_data_tmp < 3;
         varargout_1_data_tmp++) {
      point_0_1 = 0.0F;
      for (h_tmp = 0; h_tmp < 6; h_tmp++) {
        point_0_1 += (real32_T)pp[6 * h_tmp + varargout_1_data_tmp] * x[h_tmp];
      }

      varargout_1[varargout_1_data_tmp] =
        varargout_1_data_0[varargout_1_data_tmp] + point_0_1;
    }

    for (varargout_1_data_tmp = 0; varargout_1_data_tmp < m_size_idx_1_tmp;
         varargout_1_data_tmp++) {
      varargout_1_data[m_data[varargout_1_data_tmp]] =
        varargout_1[varargout_1_data_tmp];
    }
  } else {
    bnd_left = 2;
    point_0_1 = 0.0F;
    for (varargout_1_data_tmp = 0; varargout_1_data_tmp < 6;
         varargout_1_data_tmp++) {
      point_0_1 += (real32_T)point_0[6 * varargout_1_data_tmp] *
        x_data[varargout_1_data_tmp];
    }

    varargout_1_data[0] = point_0_1;
    point_0_1 = 0.0F;
    for (varargout_1_data_tmp = 0; varargout_1_data_tmp < 6;
         varargout_1_data_tmp++) {
      point_0_1 += x_data[(int32_T)((intermediate_size_tmp + 1.0F) + (real32_T)
        varargout_1_data_tmp) - 1] * (real32_T)pp[6 * varargout_1_data_tmp];
    }

    varargout_1_data[1] = point_0_1;
    if ((intermediate_size_tmp + 2.0F) + 1.0F > (((intermediate_size_tmp + 2.0F)
          + 1.0F) + 5.0F) - 2.0F) {
      tmp = 0;
      h_tmp = 0;
      l_tmp = 0;
      loop_ub = 0;
    } else {
      tmp = (int32_T)((intermediate_size_tmp + 2.0F) + 1.0F) - 1;
      h_tmp = (int32_T)((((intermediate_size_tmp + 2.0F) + 1.0F) + 5.0F) - 2.0F);
      l_tmp = tmp;
      loop_ub = h_tmp;
    }

    m_size_idx_1_tmp = loop_ub - l_tmp;
    for (varargout_1_data_tmp = 0; varargout_1_data_tmp < m_size_idx_1_tmp;
         varargout_1_data_tmp++) {
      m_data[varargout_1_data_tmp] = l_tmp + varargout_1_data_tmp;
    }

    loop_ub = h_tmp - tmp;
    for (varargout_1_data_tmp = 0; varargout_1_data_tmp < loop_ub;
         varargout_1_data_tmp++) {
      varargout_1_data_0[varargout_1_data_tmp] = varargout_1_data[tmp +
        varargout_1_data_tmp];
    }

    for (varargout_1_data_tmp = 0; varargout_1_data_tmp < 6;
         varargout_1_data_tmp++) {
      x[varargout_1_data_tmp] = x_data[varargout_1_data_tmp];
    }

    for (varargout_1_data_tmp = 0; varargout_1_data_tmp < 4;
         varargout_1_data_tmp++) {
      point_0_1 = 0.0F;
      for (h_tmp = 0; h_tmp < 6; h_tmp++) {
        point_0_1 += (real32_T)point_0[(6 * h_tmp + varargout_1_data_tmp) + 1] *
          x[h_tmp];
      }

      varargout_1_0[varargout_1_data_tmp] =
        varargout_1_data_0[varargout_1_data_tmp] + point_0_1;
    }

    for (varargout_1_data_tmp = 0; varargout_1_data_tmp < m_size_idx_1_tmp;
         varargout_1_data_tmp++) {
      varargout_1_data[m_data[varargout_1_data_tmp]] =
        varargout_1_0[varargout_1_data_tmp];
    }

    if ((intermediate_size_tmp + 2.0F) + 1.0F > (((intermediate_size_tmp + 2.0F)
          + 1.0F) + 5.0F) - 2.0F) {
      tmp = 0;
      h_tmp = 0;
      l_tmp = 0;
      loop_ub = 0;
    } else {
      tmp = (int32_T)((intermediate_size_tmp + 2.0F) + 1.0F) - 1;
      h_tmp = (int32_T)((((intermediate_size_tmp + 2.0F) + 1.0F) + 5.0F) - 2.0F);
      l_tmp = (int32_T)((intermediate_size_tmp + 2.0F) + 1.0F) - 1;
      loop_ub = (int32_T)((((intermediate_size_tmp + 2.0F) + 1.0F) + 5.0F) -
                          2.0F);
    }

    m_size_idx_1_tmp = loop_ub - l_tmp;
    for (varargout_1_data_tmp = 0; varargout_1_data_tmp < m_size_idx_1_tmp;
         varargout_1_data_tmp++) {
      m_data[varargout_1_data_tmp] = l_tmp + varargout_1_data_tmp;
    }

    loop_ub = h_tmp - tmp;
    for (varargout_1_data_tmp = 0; varargout_1_data_tmp < loop_ub;
         varargout_1_data_tmp++) {
      varargout_1_data_0[varargout_1_data_tmp] = varargout_1_data[tmp +
        varargout_1_data_tmp];
    }

    for (varargout_1_data_tmp = 0; varargout_1_data_tmp < 6;
         varargout_1_data_tmp++) {
      x[varargout_1_data_tmp] = x_data[(int32_T)((intermediate_size_tmp + 1.0F)
        + (real32_T)varargout_1_data_tmp) - 1];
    }

    for (varargout_1_data_tmp = 0; varargout_1_data_tmp < 4;
         varargout_1_data_tmp++) {
      point_0_1 = 0.0F;
      for (h_tmp = 0; h_tmp < 6; h_tmp++) {
        point_0_1 += (real32_T)pp[(6 * h_tmp + varargout_1_data_tmp) + 1] *
          x[h_tmp];
      }

      varargout_1_0[varargout_1_data_tmp] =
        varargout_1_data_0[varargout_1_data_tmp] - point_0_1;
    }

    for (varargout_1_data_tmp = 0; varargout_1_data_tmp < m_size_idx_1_tmp;
         varargout_1_data_tmp++) {
      varargout_1_data[m_data[varargout_1_data_tmp]] =
        varargout_1_0[varargout_1_data_tmp];
    }
  }

  for (tmp = 0; tmp < (int32_T)(num_of_splines - 1.0F); tmp++) {
    intermediate_size_tmp = ((1.0F + (real32_T)tmp) - 1.0F) * 6.0F;
    intermediate_size = intermediate_size_tmp + (real32_T)bnd_left;
    point_0_1 = 0.0F;
    for (varargout_1_data_tmp = 0; varargout_1_data_tmp < 6;
         varargout_1_data_tmp++) {
      point_0_1 += x_data[(int32_T)((intermediate_size_tmp + 1.0F) + (real32_T)
        varargout_1_data_tmp) - 1] * (real32_T)pp[6 * varargout_1_data_tmp];
    }

    varargout_1_data_tmp = (int32_T)(intermediate_size + 1.0F) - 1;
    varargout_1_data[varargout_1_data_tmp] += point_0_1;
    point_0_1 = 0.0F;
    for (varargout_1_data_tmp = 0; varargout_1_data_tmp < 6;
         varargout_1_data_tmp++) {
      point_0_1 += x_data[(int32_T)(((intermediate_size_tmp + 1.0F) + 6.0F) +
        (real32_T)varargout_1_data_tmp) - 1] * (real32_T)point_0[6 *
        varargout_1_data_tmp];
    }

    varargout_1_data_tmp = (int32_T)(intermediate_size + 2.0F) - 1;
    varargout_1_data[varargout_1_data_tmp] += point_0_1;
    if (intermediate_size + 3.0F > ((intermediate_size + 3.0F) + 5.0F) - 2.0F) {
      h_tmp = 0;
      l_tmp = 0;
      loop_ub = 0;
      varargout_1_data_tmp = 0;
    } else {
      h_tmp = (int32_T)(intermediate_size + 3.0F) - 1;
      l_tmp = (int32_T)(((intermediate_size + 3.0F) + 5.0F) - 2.0F);
      loop_ub = h_tmp;
      varargout_1_data_tmp = l_tmp;
    }

    m_size_idx_1_tmp = varargout_1_data_tmp - loop_ub;
    for (varargout_1_data_tmp = 0; varargout_1_data_tmp < m_size_idx_1_tmp;
         varargout_1_data_tmp++) {
      m_data[varargout_1_data_tmp] = loop_ub + varargout_1_data_tmp;
    }

    loop_ub = l_tmp - h_tmp;
    for (varargout_1_data_tmp = 0; varargout_1_data_tmp < loop_ub;
         varargout_1_data_tmp++) {
      varargout_1_data_0[varargout_1_data_tmp] = varargout_1_data[h_tmp +
        varargout_1_data_tmp];
    }

    for (varargout_1_data_tmp = 0; varargout_1_data_tmp < 6;
         varargout_1_data_tmp++) {
      x[varargout_1_data_tmp] = x_data[(int32_T)((intermediate_size_tmp + 1.0F)
        + (real32_T)varargout_1_data_tmp) - 1];
    }

    for (varargout_1_data_tmp = 0; varargout_1_data_tmp < 4;
         varargout_1_data_tmp++) {
      point_0_1 = 0.0F;
      for (h_tmp = 0; h_tmp < 6; h_tmp++) {
        point_0_1 += (real32_T)pp[(6 * h_tmp + varargout_1_data_tmp) + 1] *
          x[h_tmp];
      }

      varargout_1_0[varargout_1_data_tmp] =
        varargout_1_data_0[varargout_1_data_tmp] + point_0_1;
    }

    for (varargout_1_data_tmp = 0; varargout_1_data_tmp < m_size_idx_1_tmp;
         varargout_1_data_tmp++) {
      varargout_1_data[m_data[varargout_1_data_tmp]] =
        varargout_1_0[varargout_1_data_tmp];
    }

    if (intermediate_size + 3.0F > ((intermediate_size + 3.0F) + 5.0F) - 2.0F) {
      h_tmp = 0;
      l_tmp = 0;
      loop_ub = 0;
      varargout_1_data_tmp = 0;
    } else {
      h_tmp = (int32_T)(intermediate_size + 3.0F) - 1;
      l_tmp = (int32_T)(((intermediate_size + 3.0F) + 5.0F) - 2.0F);
      loop_ub = (int32_T)(intermediate_size + 3.0F) - 1;
      varargout_1_data_tmp = (int32_T)(((intermediate_size + 3.0F) + 5.0F) -
        2.0F);
    }

    m_size_idx_1_tmp = varargout_1_data_tmp - loop_ub;
    for (varargout_1_data_tmp = 0; varargout_1_data_tmp < m_size_idx_1_tmp;
         varargout_1_data_tmp++) {
      m_data[varargout_1_data_tmp] = loop_ub + varargout_1_data_tmp;
    }

    loop_ub = l_tmp - h_tmp;
    for (varargout_1_data_tmp = 0; varargout_1_data_tmp < loop_ub;
         varargout_1_data_tmp++) {
      varargout_1_data_0[varargout_1_data_tmp] = varargout_1_data[h_tmp +
        varargout_1_data_tmp];
    }

    for (varargout_1_data_tmp = 0; varargout_1_data_tmp < 6;
         varargout_1_data_tmp++) {
      x[varargout_1_data_tmp] = x_data[(int32_T)(((intermediate_size_tmp + 1.0F)
        + 6.0F) + (real32_T)varargout_1_data_tmp) - 1];
    }

    for (varargout_1_data_tmp = 0; varargout_1_data_tmp < 4;
         varargout_1_data_tmp++) {
      point_0_1 = 0.0F;
      for (h_tmp = 0; h_tmp < 6; h_tmp++) {
        point_0_1 += (real32_T)point_0[(6 * h_tmp + varargout_1_data_tmp) + 1] *
          x[h_tmp];
      }

      varargout_1_0[varargout_1_data_tmp] =
        varargout_1_data_0[varargout_1_data_tmp] - point_0_1;
    }

    for (varargout_1_data_tmp = 0; varargout_1_data_tmp < m_size_idx_1_tmp;
         varargout_1_data_tmp++) {
      varargout_1_data[m_data[varargout_1_data_tmp]] =
        varargout_1_0[varargout_1_data_tmp];
    }
  }
}

// Function for MATLAB Function: '<S9>/trajFromWaypoints'
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

// Function for MATLAB Function: '<S9>/trajFromWaypoints'
void MatlabControllerClass::__anon_fcn(real32_T num_of_splines, real32_T cycle,
  const real32_T x_data[], const int32_T *x_size, real32_T varargout_1_data[],
  int32_T *varargout_1_size)
{
  real_T pp[36];
  real_T point_0[36];
  real32_T intermediate_size;
  int32_T bnd_left;
  int32_T g;
  int32_T l;
  int32_T varargout_1_tmp[6];
  real_T pp_data[6];
  real_T tmp_data[5];
  real32_T varargout_1[6];
  real32_T x_data_0[60];
  real32_T x;
  int32_T loop_ub;
  int32_T pp_size[2];
  int32_T tmp_size[2];
  int8_T b_x_idx_0;
  int32_T tmp;
  int8_T b_idx_0;
  real32_T intermediate_size_tmp;
  b_x_idx_0 = (int8_T)*x_size;
  b_idx_0 = (int8_T)*x_size;
  *varargout_1_size = b_x_idx_0;
  if (0 <= b_x_idx_0 - 1) {
    memset(&varargout_1_data[0], 0, b_x_idx_0 * sizeof(real32_T));
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
  if (cycle == 0.0F) {
    bnd_left = 3;
    *varargout_1_size = b_idx_0;
    if (0 <= b_idx_0 - 1) {
      memset(&varargout_1_data[0], 0, b_idx_0 * sizeof(real32_T));
    }

    for (l = 0; l < 6; l++) {
      varargout_1_data[l] = ((real32_T)point_0[6 * l + 1] * x_data[1] +
        (real32_T)point_0[6 * l] * x_data[0]) + (real32_T)point_0[6 * l + 2] *
        x_data[2];
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
      varargout_1_tmp[l] = (int32_T)((1.0F + intermediate_size_tmp) + (real32_T)
        l);
    }

    loop_ub = g - tmp;
    for (l = 0; l < loop_ub; l++) {
      x_data_0[l] = x_data[tmp + l];
    }

    for (l = 0; l < 6; l++) {
      varargout_1[l] = (((real32_T)pp[6 * l + 1] * x_data_0[1] + (real32_T)pp[6 *
                         l] * x_data_0[0]) + (real32_T)pp[6 * l + 2] * x_data_0
                        [2]) + varargout_1_data[varargout_1_tmp[l] - 1];
    }

    for (l = 0; l < 6; l++) {
      varargout_1_data[varargout_1_tmp[l] - 1] = varargout_1[l];
    }
  } else {
    bnd_left = 2;
    for (l = 0; l < 6; l++) {
      varargout_1_data[l] = (real32_T)point_0[6 * l] * x_data[0];
      varargout_1_tmp[l] = (int32_T)((intermediate_size_tmp + 1.0F) + (real32_T)
        l);
    }

    for (l = 0; l < 6; l++) {
      varargout_1[l] = (real32_T)pp[6 * l] * x_data[1] +
        varargout_1_data[varargout_1_tmp[l] - 1];
    }

    for (l = 0; l < 6; l++) {
      varargout_1_data[varargout_1_tmp[l] - 1] = varargout_1[l];
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
      varargout_1_data[l] += (((real32_T)point_0[6 * l + 1] * x_data_0[0] +
        (real32_T)point_0[6 * l + 2] * x_data_0[1]) + (real32_T)point_0[6 * l +
        3] * x_data_0[2]) + (real32_T)point_0[6 * l + 4] * x_data_0[3];
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
      varargout_1[l] = varargout_1_data[(int32_T)((intermediate_size_tmp + 1.0F)
        + (real32_T)l) - 1] - ((((real32_T)pp[6 * l + 1] * x_data_0[0] +
        (real32_T)pp[6 * l + 2] * x_data_0[1]) + (real32_T)pp[6 * l + 3] *
        x_data_0[2]) + (real32_T)pp[6 * l + 4] * x_data_0[3]);
    }

    for (l = 0; l < 6; l++) {
      varargout_1_data[(int32_T)((intermediate_size_tmp + 1.0F) + (real32_T)l) -
        1] = varargout_1[l];
    }
  }

  for (tmp = 0; tmp < (int32_T)(num_of_splines - 1.0F); tmp++) {
    intermediate_size_tmp = ((1.0F + (real32_T)tmp) - 1.0F) * 6.0F;
    intermediate_size = intermediate_size_tmp + (real32_T)bnd_left;
    for (l = 0; l < 6; l++) {
      varargout_1_tmp[l] = (int32_T)((intermediate_size_tmp + 1.0F) + (real32_T)
        l);
    }

    x = x_data[(int32_T)(intermediate_size + 1.0F) - 1];
    for (l = 0; l < 6; l++) {
      varargout_1[l] = (real32_T)pp[6 * l] * x +
        varargout_1_data[varargout_1_tmp[l] - 1];
    }

    for (l = 0; l < 6; l++) {
      varargout_1_data[varargout_1_tmp[l] - 1] = varargout_1[l];
    }

    for (l = 0; l < 6; l++) {
      varargout_1_tmp[l] = (int32_T)(((intermediate_size_tmp + 1.0F) + 6.0F) +
        (real32_T)l);
    }

    x = x_data[(int32_T)(intermediate_size + 2.0F) - 1];
    for (l = 0; l < 6; l++) {
      varargout_1[l] = (real32_T)point_0[6 * l] * x +
        varargout_1_data[varargout_1_tmp[l] - 1];
    }

    for (l = 0; l < 6; l++) {
      varargout_1_data[varargout_1_tmp[l] - 1] = varargout_1[l];
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
      x_data_0[l] = x_data[g + l];
    }

    for (l = 0; l < 6; l++) {
      varargout_1[l] = ((((real32_T)pp[6 * l + 1] * x_data_0[0] + (real32_T)pp[6
                          * l + 2] * x_data_0[1]) + (real32_T)pp[6 * l + 3] *
                         x_data_0[2]) + (real32_T)pp[6 * l + 4] * x_data_0[3]) +
        varargout_1_data[(int32_T)((intermediate_size_tmp + 1.0F) + (real32_T)l)
        - 1];
    }

    for (l = 0; l < 6; l++) {
      varargout_1_data[(int32_T)((intermediate_size_tmp + 1.0F) + (real32_T)l) -
        1] = varargout_1[l];
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
      x_data_0[l] = x_data[g + l];
    }

    for (l = 0; l < 6; l++) {
      varargout_1[l] = varargout_1_data[(int32_T)(((intermediate_size_tmp + 1.0F)
        + 6.0F) + (real32_T)l) - 1] - ((((real32_T)point_0[6 * l + 1] *
        x_data_0[0] + (real32_T)point_0[6 * l + 2] * x_data_0[1]) + (real32_T)
        point_0[6 * l + 3] * x_data_0[2]) + (real32_T)point_0[6 * l + 4] *
        x_data_0[3]);
    }

    for (l = 0; l < 6; l++) {
      varargout_1_data[(int32_T)(((intermediate_size_tmp + 1.0F) + 6.0F) +
        (real32_T)l) - 1] = varargout_1[l];
    }
  }
}

// Function for MATLAB Function: '<S9>/trajFromWaypoints'
real32_T MatlabControllerClass::norm_d(const real32_T x[2])
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

// Function for MATLAB Function: '<S9>/trajFromWaypoints'
void MatlabControllerClass::ladac_lsqr_iterate(const real32_T
  A_tunableEnvironment[3], real32_T x_data[], int32_T *x_size, real32_T w_data[],
  int32_T *w_size, real32_T u_data[], int32_T *u_size, real32_T v_data[],
  int32_T *v_size, real32_T *Anorm, real32_T *alfa, real32_T *rhobar, real32_T
  *phibar)
{
  real32_T beta;
  real32_T b_data[60];
  real32_T scale;
  real32_T absxk;
  real32_T t;
  real32_T rhobar_0[2];
  int32_T i;
  int32_T b_size;
  __anon_fcn_a(A_tunableEnvironment[0], A_tunableEnvironment[2], v_data, v_size,
               b_data, &b_size);
  *u_size = b_size;
  for (i = 0; i < b_size; i++) {
    u_data[i] = b_data[i] - *alfa * u_data[i];
  }

  beta = norm(u_data, u_size);
  if (beta > 0.0F) {
    scale = 1.0F / beta;
    b_size = *u_size;
    for (i = 0; i < b_size; i++) {
      u_data[i] *= scale;
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
    __anon_fcn(A_tunableEnvironment[0], A_tunableEnvironment[2], u_data, u_size,
               b_data, &b_size);
    *v_size = b_size;
    for (i = 0; i < b_size; i++) {
      v_data[i] = b_data[i] - beta * v_data[i];
    }

    *alfa = norm(v_data, v_size);
    if (*alfa > 0.0F) {
      scale = 1.0F / *alfa;
      b_size = *v_size;
      for (i = 0; i < b_size; i++) {
        v_data[i] *= scale;
      }
    }
  }

  rhobar_0[0] = *rhobar;
  rhobar_0[1] = 0.0F;
  absxk = norm_d(rhobar_0);
  *phibar *= *rhobar / absxk;
  rhobar_0[0] = absxk;
  rhobar_0[1] = beta;
  scale = norm_d(rhobar_0);
  absxk /= scale;
  beta /= scale;
  *rhobar = -absxk * *alfa;
  absxk *= *phibar;
  *phibar *= beta;
  absxk /= scale;
  beta = -(beta * *alfa) / scale;
  b_size = *x_size;
  for (i = 0; i < b_size; i++) {
    x_data[i] += absxk * w_data[i];
  }

  *w_size = *v_size;
  b_size = *v_size;
  for (i = 0; i < b_size; i++) {
    w_data[i] = beta * w_data[i] + v_data[i];
  }
}

// Function for MATLAB Function: '<S9>/trajFromWaypoints'
void MatlabControllerClass::polyInterpolationb(const real32_T points_data[],
  const int32_T points_size[2], real32_T cycle, real32_T b_data[], int32_T
  *b_size, real32_T *num_of_splines)
{
  real32_T points_new_data[11];
  int32_T size_A_mat;
  int32_T bnd_left;
  int32_T itm_row;
  int32_T loop_ub;
  real32_T b_data_tmp;
  if (cycle == 1.0F) {
    itm_row = points_size[1] + 1;
    loop_ub = points_size[0] * points_size[1];
    if (0 <= loop_ub - 1) {
      memcpy(&points_new_data[0], &points_data[0], loop_ub * sizeof(real32_T));
    }

    points_new_data[loop_ub] = points_data[0];
  } else {
    itm_row = points_size[1];
    loop_ub = points_size[0] * points_size[1] - 1;
    if (0 <= loop_ub) {
      memcpy(&points_new_data[0], &points_data[0], (loop_ub + 1) * sizeof
             (real32_T));
    }
  }

  *num_of_splines = (real32_T)itm_row - 1.0F;
  loop_ub = (int32_T)((real32_T)itm_row - 1.0F);
  size_A_mat = loop_ub * 6;
  *b_size = size_A_mat;
  if (0 <= size_A_mat - 1) {
    memset(&b_data[0], 0, size_A_mat * sizeof(real32_T));
  }

  if (cycle == 0.0F) {
    bnd_left = 3;
    b_data[0] = points_new_data[0];
    b_data[size_A_mat - 3] = points_new_data[itm_row - 1];
    b_data[1] = points_new_data[1] - points_new_data[0];
    b_data[size_A_mat - 2] = points_new_data[itm_row - 1] -
      points_new_data[itm_row - 2];
  } else {
    bnd_left = 2;
    b_data[0] = points_new_data[0];
    b_data[1] = points_new_data[itm_row - 1];
  }

  for (size_A_mat = 0; size_A_mat <= loop_ub - 2; size_A_mat++) {
    itm_row = 6 * size_A_mat + bnd_left;
    b_data_tmp = points_new_data[size_A_mat + 1];
    b_data[itm_row] = b_data_tmp;
    b_data[itm_row + 1] = b_data_tmp;
  }
}

// Function for MATLAB Function: '<S9>/trajFromWaypoints'
void MatlabControllerClass::ladac_lsqr_init(const real32_T A_tunableEnvironment
  [3], const real32_T b_data[], const int32_T *b_size, real32_T x_data[],
  int32_T *x_size, real32_T w_data[], int32_T *w_size, real32_T u_data[],
  int32_T *u_size, real32_T v_data[], int32_T *v_size, real32_T *alfa, real32_T *
  rhobar, real32_T *phibar)
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

    __anon_fcn(A_tunableEnvironment[0], A_tunableEnvironment[2], u_data, u_size,
               v_data, v_size);
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

// Function for MATLAB Function: '<S2>/trajGetMatch'
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

// Function for MATLAB Function: '<S2>/trajGetMatch'
void MatlabControllerClass::polyder_hy(const real32_T u_data[], const int32_T
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

// Function for MATLAB Function: '<S2>/trajGetMatch'
void MatlabControllerClass::trajGetMatchEnhanced(const
  dtoSgl_trajectoryStructBus *traj, const real32_T position[3], real32_T
  *section_idx, real32_T *error, real32_T *t)
{
  real32_T px[12];
  real32_T py[12];
  real32_T pz[12];
  real32_T curr_pos[36];
  real32_T trajSection_pos_x[6];
  real32_T trajSection_pos_y[6];
  real32_T trajSection_pos_z[6];
  real32_T b_px;
  real32_T b_py;
  real32_T b_pz;
  int32_T b_i;
  real_T b_section_idx;
  real32_T c_px;
  real32_T c_py;
  real32_T c_pz;
  int32_T e_i;
  real32_T dx_data[5];
  real32_T dy_data[5];
  real32_T dz_data[5];
  static const real32_T x[12] = { 0.0F, 0.0909090936F, 0.181818187F,
    0.272727281F, 0.363636374F, 0.454545468F, 0.545454562F, 0.636363626F,
    0.727272749F, 0.818181813F, 0.909090936F, 1.0F };

  static const real32_T b[12] = { 0.0F, 0.0909090936F, 0.181818187F,
    0.272727281F, 0.363636374F, 0.454545468F, 0.545454562F, 0.636363626F,
    0.727272749F, 0.818181813F, 0.909090936F, 1.0F };

  real32_T tmp_data[4];
  real32_T tmp_data_0[4];
  real32_T tmp_data_1[4];
  real32_T position_0[3];
  real32_T f_dt_idx_1;
  real32_T f_dt_idx_2;
  int32_T dx_size[2];
  int32_T dy_size[2];
  int32_T dz_size[2];
  int32_T tmp_size[2];
  int32_T curr_pos_tmp;
  int32_T curr_pos_tmp_0;
  boolean_T exitg1;
  *t = 0.0F;
  *section_idx = 1.0F;
  b_px = traj->sections[0].pos_x[0];
  b_py = traj->sections[0].pos_y[0];
  b_pz = traj->sections[0].pos_z[0];
  for (b_i = 0; b_i < 5; b_i++) {
    b_px = traj->sections[0].pos_x[b_i + 1];
    b_py = traj->sections[0].pos_y[b_i + 1];
    b_pz = traj->sections[0].pos_z[b_i + 1];
  }

  position_0[0] = position[0] - b_px;
  position_0[1] = position[1] - b_py;
  position_0[2] = position[2] - b_pz;
  *error = norm_c(position_0);
  for (b_i = 0; b_i < (int32_T)traj->num_sections_set; b_i++) {
    for (curr_pos_tmp = 0; curr_pos_tmp < 12; curr_pos_tmp++) {
      px[curr_pos_tmp] = traj->sections[(int32_T)(1.0F + (real32_T)b_i) - 1].
        pos_x[0];
    }

    for (e_i = 0; e_i < 5; e_i++) {
      b_px = traj->sections[(int32_T)(1.0F + (real32_T)b_i) - 1].pos_x[e_i + 1];
      for (curr_pos_tmp = 0; curr_pos_tmp < 12; curr_pos_tmp++) {
        px[curr_pos_tmp] = x[curr_pos_tmp] * px[curr_pos_tmp] + b_px;
      }
    }

    for (curr_pos_tmp = 0; curr_pos_tmp < 12; curr_pos_tmp++) {
      py[curr_pos_tmp] = traj->sections[(int32_T)(1.0F + (real32_T)b_i) - 1].
        pos_y[0];
    }

    for (e_i = 0; e_i < 5; e_i++) {
      b_px = traj->sections[(int32_T)(1.0F + (real32_T)b_i) - 1].pos_y[e_i + 1];
      for (curr_pos_tmp = 0; curr_pos_tmp < 12; curr_pos_tmp++) {
        py[curr_pos_tmp] = x[curr_pos_tmp] * py[curr_pos_tmp] + b_px;
      }
    }

    for (curr_pos_tmp = 0; curr_pos_tmp < 12; curr_pos_tmp++) {
      pz[curr_pos_tmp] = traj->sections[(int32_T)(1.0F + (real32_T)b_i) - 1].
        pos_z[0];
    }

    for (e_i = 0; e_i < 5; e_i++) {
      b_px = traj->sections[(int32_T)(1.0F + (real32_T)b_i) - 1].pos_z[e_i + 1];
      for (curr_pos_tmp = 0; curr_pos_tmp < 12; curr_pos_tmp++) {
        pz[curr_pos_tmp] = x[curr_pos_tmp] * pz[curr_pos_tmp] + b_px;
      }
    }

    for (e_i = 0; e_i < 12; e_i++) {
      curr_pos[3 * e_i] = px[e_i];
      curr_pos_tmp = 1 + 3 * e_i;
      curr_pos[curr_pos_tmp] = py[e_i];
      curr_pos_tmp_0 = 2 + 3 * e_i;
      curr_pos[curr_pos_tmp_0] = pz[e_i];
      curr_pos[3 * e_i] -= position[0];
      curr_pos[curr_pos_tmp] = curr_pos[3 * e_i + 1] - position[1];
      curr_pos[curr_pos_tmp_0] = curr_pos[3 * e_i + 2] - position[2];
    }

    for (e_i = 0; e_i < 12; e_i++) {
      curr_pos_tmp = e_i * 3 + 1;
      b_px = 0.0F;
      b_py = 1.29246971E-26F;
      for (curr_pos_tmp_0 = curr_pos_tmp; curr_pos_tmp_0 <= curr_pos_tmp + 2;
           curr_pos_tmp_0++) {
        b_pz = std::abs(curr_pos[curr_pos_tmp_0 - 1]);
        if (b_pz > b_py) {
          c_px = b_py / b_pz;
          b_px = b_px * c_px * c_px + 1.0F;
          b_py = b_pz;
        } else {
          c_px = b_pz / b_py;
          b_px += c_px * c_px;
        }
      }

      px[e_i] = b_py * std::sqrt(b_px);
    }

    b_px = px[0];
    e_i = -1;
    for (curr_pos_tmp = 0; curr_pos_tmp < 11; curr_pos_tmp++) {
      c_px = px[curr_pos_tmp + 1];
      if (b_px > c_px) {
        b_px = c_px;
        e_i = curr_pos_tmp;
      }
    }

    if (b_px < *error) {
      *error = b_px;
      *section_idx = 1.0F + (real32_T)b_i;
      *t = b[e_i + 1];
    }
  }

  b_section_idx = *section_idx;
  if ((*section_idx > traj->num_sections_set) || (*section_idx < 1.0F)) {
    b_section_idx = 1.0;
  }

  for (curr_pos_tmp = 0; curr_pos_tmp < 6; curr_pos_tmp++) {
    trajSection_pos_x[curr_pos_tmp] = traj->sections[(int32_T)b_section_idx - 1]
      .pos_x[curr_pos_tmp];
    trajSection_pos_y[curr_pos_tmp] = traj->sections[(int32_T)b_section_idx - 1]
      .pos_y[curr_pos_tmp];
    trajSection_pos_z[curr_pos_tmp] = traj->sections[(int32_T)b_section_idx - 1]
      .pos_z[curr_pos_tmp];
  }

  b_px = position[0];
  b_i = 0;
  exitg1 = false;
  while ((!exitg1) && (b_i < 100)) {
    c_px = trajSection_pos_x[0];
    c_py = trajSection_pos_y[0];
    c_pz = trajSection_pos_z[0];
    for (e_i = 0; e_i < 5; e_i++) {
      c_px = *t * c_px + trajSection_pos_x[e_i + 1];
      c_py = *t * c_py + trajSection_pos_y[e_i + 1];
      c_pz = *t * c_pz + trajSection_pos_z[e_i + 1];
    }

    b_py = c_px;
    polyder_a(trajSection_pos_x, dx_data, dx_size);
    polyder_a(trajSection_pos_y, dy_data, dy_size);
    polyder_a(trajSection_pos_z, dz_data, dz_size);
    b_pz = polyVal(dx_data, dx_size, (real_T)*t);
    f_dt_idx_1 = polyVal(dy_data, dy_size, (real_T)*t);
    f_dt_idx_2 = polyVal(dz_data, dz_size, (real_T)*t);
    polyder_hy(dx_data, dx_size, tmp_data, tmp_size);
    polyder_hy(dy_data, dy_size, tmp_data_0, dx_size);
    polyder_hy(dz_data, dz_size, tmp_data_1, dy_size);
    c_py -= position[1];
    c_pz -= position[2];
    c_px = (((c_px - b_px) * b_pz + c_py * f_dt_idx_1) + c_pz * f_dt_idx_2) *
      2.0F;
    c_py = ((((((b_py - b_px) * polyVal(tmp_data, tmp_size, (real_T)*t) + c_py *
                polyVal(tmp_data_0, dx_size, (real_T)*t)) + c_pz * polyVal
               (tmp_data_1, dy_size, (real_T)*t)) + b_pz * b_pz) + f_dt_idx_1 *
             f_dt_idx_1) + f_dt_idx_2 * f_dt_idx_2) * 2.0F;
    if (std::abs(c_py) < 1.0E-16) {
      c_py = 1.0E-8F;
    }

    *t -= c_px / c_py;
    if (std::abs(c_px) < 1.0E-12) {
      exitg1 = true;
    } else {
      if ((*t < 0.0F) || (*t > 1.0F)) {
        if (*t < 0.0F) {
          if (*t + 1.0F < 0.95) {
            *t = 0.95F;
          } else {
            (*t)++;
          }

          (*section_idx)--;
          if (*section_idx < 1.0F) {
            if (traj->is_repeated_course) {
              *section_idx = traj->num_sections_set;
            } else {
              *section_idx = 1.0F;
              *t = 0.0F;
            }
          }
        } else {
          if (*t - 1.0F > 0.05) {
            *t = 0.05F;
          } else {
            (*t)--;
          }

          (*section_idx)++;
          if (*section_idx > traj->num_sections_set) {
            if (traj->is_repeated_course) {
              *section_idx = 1.0F;
            } else {
              *section_idx = traj->num_sections_set;
              *t = 1.0F;
            }
          }
        }

        b_section_idx = *section_idx;
        if ((*section_idx > traj->num_sections_set) || (*section_idx < 1.0F)) {
          b_section_idx = 1.0;
        }

        for (curr_pos_tmp = 0; curr_pos_tmp < 6; curr_pos_tmp++) {
          trajSection_pos_x[curr_pos_tmp] = traj->sections[(int32_T)
            b_section_idx - 1].pos_x[curr_pos_tmp];
          trajSection_pos_y[curr_pos_tmp] = traj->sections[(int32_T)
            b_section_idx - 1].pos_y[curr_pos_tmp];
          trajSection_pos_z[curr_pos_tmp] = traj->sections[(int32_T)
            b_section_idx - 1].pos_z[curr_pos_tmp];
        }
      }

      b_i++;
    }
  }

  if (*t < 0.0F) {
    b_px = 0.0F;
  } else {
    b_px = *t;
  }

  if (b_px > 1.0F) {
    *t = 1.0F;
  } else {
    *t = b_px;
  }
}

// Model step function
void MatlabControllerClass::step()
{
  dtoSgl_trajectoryStructBus traj;
  uint16_T num_wp;
  real32_T state;
  real32_T b_size;
  real32_T axis_sel;
  real32_T num_of_splines;
  real32_T residuum;
  real32_T b_data[60];
  real32_T x_data[60];
  real32_T w_data[60];
  real32_T u_data[60];
  real32_T v_data[60];
  real32_T tunableEnvironment[3];
  real_T b_section_idx;
  boolean_T rtb_Compare;
  real32_T rtb_state_vec[6];
  real32_T rtb_Gain[16];
  boolean_T rtb_Compare_l;
  real32_T rtb_Add;
  real32_T rtb_uvwxb[300];
  real32_T rtb_MathFunction[3];
  int32_T i;
  real32_T tmp_data[10];
  int32_T loop_ub;
  int32_T x_size;
  int32_T w_size;
  int32_T v_size;
  int32_T tmp_size[2];
  real32_T rtb_AnormAlfaRhoPhi_idx_0;
  real32_T rtb_AnormAlfaRhoPhi_idx_1;
  real32_T rtb_AnormAlfaRhoPhi_idx_2;
  real32_T rtb_AnormAlfaRhoPhi_idx_3;
  real32_T rtb_Switch_i_idx_3;
  real32_T rtb_Switch_i_idx_0;
  real32_T rtb_Switch_i_idx_1;
  real32_T rtb_Switch_i_idx_2;
  uint16_T varargin_1_idx_1;
  const dtoSgl_trajectoryStructBus *rtb_Switch_0;

  // Memory: '<S9>/Memory2'
  memcpy(&rtb_uvwxb[0], &rtDW.Memory2_PreviousInput[0], 300U * sizeof(real32_T));

  // RelationalOperator: '<S29>/Compare' incorporates:
  //   Constant: '<S29>/Constant'
  //   Inport: '<Root>/cmd'

  rtb_Compare = (rtU.cmd.mission_change > 0);

  // Memory: '<S9>/Memory1'
  rtb_AnormAlfaRhoPhi_idx_0 = rtDW.Memory1_PreviousInput[0];
  rtb_AnormAlfaRhoPhi_idx_1 = rtDW.Memory1_PreviousInput[1];
  rtb_AnormAlfaRhoPhi_idx_2 = rtDW.Memory1_PreviousInput[2];
  rtb_AnormAlfaRhoPhi_idx_3 = rtDW.Memory1_PreviousInput[3];

  // MATLAB Function: '<S9>/trajFromWaypoints' incorporates:
  //   Constant: '<S2>/Constant2'
  //   Constant: '<S9>/Constant1'
  //   Inport: '<Root>/cmd'
  //   Memory: '<S9>/Memory'
  //   Memory: '<S9>/Memory1'
  //   Memory: '<S9>/Memory2'
  //   RelationalOperator: '<S26>/FixPt Relational Operator'
  //   UnitDelay: '<S26>/Delay Input1'
  //   UnitDelay: '<S9>/Unit Delay'
  //
  //  Block description for '<S26>/Delay Input1':
  //
  //   Store in Global RAM

  traj = rtDW.UnitDelay_DSTATE;
  rtb_state_vec[0] = rtDW.Memory_PreviousInput[0];
  rtb_state_vec[1] = rtDW.Memory_PreviousInput[1];
  axis_sel = rtDW.Memory_PreviousInput[2];
  num_of_splines = rtDW.Memory_PreviousInput[3];
  rtb_Add = rtDW.Memory_PreviousInput[4];
  residuum = rtDW.Memory_PreviousInput[5];
  b_size = rt_roundf(rtDW.UnitDelay_DSTATE.num_sections_max);
  if (b_size < 65536.0F) {
    if (b_size >= 0.0F) {
      varargin_1_idx_1 = (uint16_T)b_size;
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

  if (((int32_T)rtb_Compare > (int32_T)rtDW.DelayInput1_DSTATE) &&
      (rtDW.Memory_PreviousInput[0] == 0.0F) && (rtDW.Memory_PreviousInput[2] ==
       0.0F)) {
    rtb_Add = 0.0F;
    traj = rtConstP.pooled3;
    axis_sel = 1.0F;
    rtb_state_vec[0] = 0.0F;
  } else if ((rtDW.Memory_PreviousInput[2] >= 1.0F) &&
             (rtDW.Memory_PreviousInput[2] <= 3.0F)) {
    if (rtDW.Memory_PreviousInput[0] == 0.0F) {
      if (1 > num_wp) {
        loop_ub = 0;
      } else {
        loop_ub = num_wp;
      }

      i = (int32_T)rtDW.Memory_PreviousInput[2];
      tmp_size[0] = 1;
      tmp_size[1] = loop_ub;
      for (x_size = 0; x_size < loop_ub; x_size++) {
        tmp_data[x_size] = rtU.cmd.waypoints[((x_size << 2) + i) - 1];
      }

      polyInterpolationb(tmp_data, tmp_size, 1.0F, b_data, &loop_ub,
                         &num_of_splines);
      tunableEnvironment[0] = num_of_splines;
      tunableEnvironment[1] = 5.0F;
      tunableEnvironment[2] = 1.0F;
      ladac_lsqr_init(tunableEnvironment, b_data, &loop_ub, x_data, &x_size,
                      w_data, &w_size, u_data, &i, v_data, &v_size,
                      &rtb_AnormAlfaRhoPhi_idx_1, &rtb_AnormAlfaRhoPhi_idx_2,
                      &rtb_AnormAlfaRhoPhi_idx_3);
      rtb_state_vec[0] = 1.0F;
      rtb_state_vec[1] = (real32_T)loop_ub;
      if (0 <= loop_ub - 1) {
        memcpy(&rtb_uvwxb[0], &b_data[0], loop_ub * sizeof(real32_T));
      }

      if (0 <= i - 1) {
        memcpy(&rtb_uvwxb[60], &u_data[0], i * sizeof(real32_T));
      }

      if (0 <= v_size - 1) {
        memcpy(&rtb_uvwxb[120], &v_data[0], v_size * sizeof(real32_T));
      }

      if (0 <= w_size - 1) {
        memcpy(&rtb_uvwxb[180], &w_data[0], w_size * sizeof(real32_T));
      }

      if (0 <= x_size - 1) {
        memcpy(&rtb_uvwxb[240], &x_data[0], x_size * sizeof(real32_T));
      }

      rtb_AnormAlfaRhoPhi_idx_0 = 0.0F;
    } else if (rtDW.Memory_PreviousInput[0] < 1000.0F) {
      if (1.0F > rtDW.Memory_PreviousInput[1]) {
        loop_ub = 0;
      } else {
        loop_ub = (int32_T)rtDW.Memory_PreviousInput[1];
      }

      if (0 <= loop_ub - 1) {
        memcpy(&b_data[0], &rtDW.Memory2_PreviousInput[0], loop_ub * sizeof
               (real32_T));
      }

      tunableEnvironment[0] = rtDW.Memory_PreviousInput[3];
      tunableEnvironment[1] = 5.0F;
      tunableEnvironment[2] = 1.0F;
      if (1.0F > rtDW.Memory_PreviousInput[1]) {
        i = 0;
      } else {
        i = (int32_T)rtDW.Memory_PreviousInput[1];
      }

      x_size = i;
      if (0 <= i - 1) {
        memcpy(&x_data[0], &rtDW.Memory2_PreviousInput[240], i * sizeof(real32_T));
      }

      if (1.0F > rtDW.Memory_PreviousInput[1]) {
        i = 0;
      } else {
        i = (int32_T)rtDW.Memory_PreviousInput[1];
      }

      w_size = i;
      if (0 <= i - 1) {
        memcpy(&w_data[0], &rtDW.Memory2_PreviousInput[180], i * sizeof(real32_T));
      }

      if (1.0F > rtDW.Memory_PreviousInput[1]) {
        i = 0;
      } else {
        i = (int32_T)rtDW.Memory_PreviousInput[1];
      }

      if (0 <= i - 1) {
        memcpy(&u_data[0], &rtDW.Memory2_PreviousInput[60], i * sizeof(real32_T));
      }

      if (1.0F > rtDW.Memory_PreviousInput[1]) {
        i = 0;
      } else {
        i = (int32_T)rtDW.Memory_PreviousInput[1];
      }

      v_size = i;
      if (0 <= i - 1) {
        memcpy(&v_data[0], &rtDW.Memory2_PreviousInput[120], i * sizeof(real32_T));
      }

      rtb_AnormAlfaRhoPhi_idx_0 = rtDW.Memory1_PreviousInput[0];
      rtb_AnormAlfaRhoPhi_idx_1 = rtDW.Memory1_PreviousInput[1];
      rtb_AnormAlfaRhoPhi_idx_2 = rtDW.Memory1_PreviousInput[2];
      rtb_AnormAlfaRhoPhi_idx_3 = rtDW.Memory1_PreviousInput[3];
      ladac_lsqr_iterate(tunableEnvironment, x_data, &x_size, w_data, &w_size,
                         u_data, &i, v_data, &v_size, &rtb_AnormAlfaRhoPhi_idx_0,
                         &rtb_AnormAlfaRhoPhi_idx_1, &rtb_AnormAlfaRhoPhi_idx_2,
                         &rtb_AnormAlfaRhoPhi_idx_3);
      if (0 <= loop_ub - 1) {
        memcpy(&rtb_uvwxb[0], &b_data[0], loop_ub * sizeof(real32_T));
      }

      if (0 <= i - 1) {
        memcpy(&rtb_uvwxb[60], &u_data[0], i * sizeof(real32_T));
      }

      if (0 <= v_size - 1) {
        memcpy(&rtb_uvwxb[120], &v_data[0], v_size * sizeof(real32_T));
      }

      if (0 <= w_size - 1) {
        memcpy(&rtb_uvwxb[180], &w_data[0], w_size * sizeof(real32_T));
      }

      if (0 <= x_size - 1) {
        memcpy(&rtb_uvwxb[240], &x_data[0], x_size * sizeof(real32_T));
      }

      rtb_state_vec[0] = rtDW.Memory_PreviousInput[0] + 1.0F;
      __anon_fcn_a(rtDW.Memory_PreviousInput[3], 1.0F, x_data, &x_size, w_data,
                   &w_size);
      for (i = 0; i < w_size; i++) {
        x_data[i] = w_data[i] - b_data[i];
      }

      residuum = norm(x_data, &w_size);
      if (residuum < 0.001) {
        rtb_state_vec[0] = 1000.0F;
      }
    } else {
      if (1.0F > rtDW.Memory_PreviousInput[1]) {
        i = 0;
      } else {
        i = (int32_T)rtDW.Memory_PreviousInput[1];
      }

      loop_ub = i - 1;
      if (0 <= loop_ub) {
        memcpy(&x_data[0], &rtDW.Memory2_PreviousInput[240], (loop_ub + 1) *
               sizeof(real32_T));
      }

      traj.num_sections_set = rtDW.Memory_PreviousInput[3];
      traj.is_repeated_course = true;
      for (i = 0; i < (int32_T)num_of_splines; i++) {
        state = ((1.0F + (real32_T)i) - 1.0F) * 6.0F + 1.0F;
        if (axis_sel == 1.0F) {
          if (state > state + 5.0F) {
            x_size = 1;
            w_size = 0;
          } else {
            x_size = (int32_T)state;
            w_size = (int32_T)(state + 5.0F);
          }

          loop_ub = w_size - x_size;
          for (w_size = 0; w_size <= loop_ub; w_size++) {
            b_data[w_size] = x_data[(x_size + w_size) - 1];
          }

          for (x_size = 0; x_size < 6; x_size++) {
            traj.sections[(int32_T)(1.0F + (real32_T)i) - 1].pos_x[x_size] =
              b_data[x_size];
          }
        } else if (axis_sel == 2.0F) {
          if (state > state + 5.0F) {
            x_size = 1;
            w_size = 0;
          } else {
            x_size = (int32_T)state;
            w_size = (int32_T)(state + 5.0F);
          }

          loop_ub = w_size - x_size;
          for (w_size = 0; w_size <= loop_ub; w_size++) {
            b_data[w_size] = x_data[(x_size + w_size) - 1];
          }

          for (x_size = 0; x_size < 6; x_size++) {
            traj.sections[(int32_T)(1.0F + (real32_T)i) - 1].pos_y[x_size] =
              b_data[x_size];
          }
        } else {
          if (axis_sel == 3.0F) {
            if (state > state + 5.0F) {
              x_size = 1;
              w_size = 0;
            } else {
              x_size = (int32_T)state;
              w_size = (int32_T)(state + 5.0F);
            }

            loop_ub = w_size - x_size;
            for (w_size = 0; w_size <= loop_ub; w_size++) {
              b_data[w_size] = x_data[(x_size + w_size) - 1];
            }

            for (x_size = 0; x_size < 6; x_size++) {
              traj.sections[(int32_T)(1.0F + (real32_T)i) - 1].pos_z[x_size] =
                b_data[x_size];
            }
          }
        }
      }

      rtb_state_vec[0] = 0.0F;
      axis_sel = rtDW.Memory_PreviousInput[2] + 1.0F;
    }
  } else {
    if (rtDW.Memory_PreviousInput[2] == 4.0F) {
      trajSetArcLength(&traj);
      rtb_Add = 1.0F;
      axis_sel = 0.0F;
      rtb_state_vec[0] = 0.0F;
    }
  }

  rtb_state_vec[2] = axis_sel;
  rtb_state_vec[3] = num_of_splines;
  rtb_state_vec[4] = rtb_Add;
  rtb_state_vec[5] = residuum;

  // Gain: '<Root>/Gain' incorporates:
  //   Bias: '<Root>/Bias'
  //   Inport: '<Root>/cmd'

  for (i = 0; i < 16; i++) {
    rtb_Gain[i] = (rtU.cmd.RC_pwm[i] + -1000.0F) * 0.001F;
  }

  // End of Gain: '<Root>/Gain'

  // Switch: '<S9>/Switch' incorporates:
  //   Constant: '<S9>/Constant1'
  //   MATLAB Function: '<S9>/trajFromWaypoints'

  if (rtb_Add >= 1.0F) {
    rtb_Switch_0 = &traj;
  } else {
    rtb_Switch_0 = (&rtConstP.pooled3);
  }

  // End of Switch: '<S9>/Switch'

  // MATLAB Function: '<S2>/trajGetMatch' incorporates:
  //   Inport: '<Root>/measure'

  trajGetMatchEnhanced(rtb_Switch_0, rtU.measure.s_Kg, &b_size, &axis_sel,
                       &rtb_Add);
  b_section_idx = b_size;
  if ((b_size > rtb_Switch_0->num_sections_set) || (b_size < 1.0F)) {
    b_section_idx = 1.0;
  }

  b_size = rtb_Switch_0->sections[(int32_T)b_section_idx - 1].pos_x[0];
  axis_sel = rtb_Switch_0->sections[(int32_T)b_section_idx - 1].pos_y[0];
  num_of_splines = rtb_Switch_0->sections[(int32_T)b_section_idx - 1].pos_z[0];
  for (i = 0; i < 5; i++) {
    b_size = rtb_Add * b_size + rtb_Switch_0->sections[(int32_T)b_section_idx -
      1].pos_x[i + 1];
    axis_sel = rtb_Add * axis_sel + rtb_Switch_0->sections[(int32_T)
      b_section_idx - 1].pos_y[i + 1];
    num_of_splines = rtb_Add * num_of_splines + rtb_Switch_0->sections[(int32_T)
      b_section_idx - 1].pos_z[i + 1];
  }

  // Saturate: '<S2>/Saturation1'
  if (rtb_Gain[5] > 1.0F) {
    rtb_Add = 1.0F;
  } else if (rtb_Gain[5] < 0.0F) {
    rtb_Add = 0.0F;
  } else {
    rtb_Add = rtb_Gain[5];
  }

  // RelationalOperator: '<S18>/Compare' incorporates:
  //   Constant: '<S18>/Constant'
  //   DataTypeConversion: '<S6>/Data Type Conversion1'

  rtb_Compare_l = (rtb_Add != 0.0F);

  // Saturate: '<S2>/Saturation1'
  if (rtb_Gain[5] > 1.0F) {
    rtb_Add = 1.0F;
  } else if (rtb_Gain[5] < 0.0F) {
    rtb_Add = 0.0F;
  } else {
    rtb_Add = rtb_Gain[5];
  }

  // CombinatorialLogic: '<S17>/Logic' incorporates:
  //   DataTypeConversion: '<S6>/Data Type Conversion1'
  //   Logic: '<S6>/NOT'
  //   Logic: '<S6>/OR'
  //   Memory: '<S17>/Memory'
  //   RelationalOperator: '<S15>/FixPt Relational Operator'
  //   UnitDelay: '<S15>/Delay Input1'
  //   UnitDelay: '<S6>/Unit Delay'
  //
  //  Block description for '<S15>/Delay Input1':
  //
  //   Store in Global RAM

  loop_ub = (int32_T)(((((uint32_T)(rtDW.UnitDelay_DSTATE_l || (rtb_Add == 0.0F))
    << 1) + ((int32_T)rtb_Compare_l > (int32_T)rtDW.DelayInput1_DSTATE_e)) << 1)
                      + rtDW.Memory_PreviousInput_i);

  // Outputs for Enabled SubSystem: '<S2>/Subsystem2' incorporates:
  //   EnablePort: '<S7>/Enable'

  // Delay: '<S6>/Delay' incorporates:
  //   CombinatorialLogic: '<S17>/Logic'

  if (rtConstP.Logic_table[(uint32_T)loop_ub]) {
    rtDW.Delay_DSTATE = 1.0F;

    // Sum: '<S7>/Sum3' incorporates:
    //   Constant: '<S7>/Constant'
    //   DiscreteIntegrator: '<S19>/Discrete-Time Integrator y'

    residuum = 14.0F - rtDW.DiscreteTimeIntegratory_DSTATE;

    // Saturate: '<S7>/Saturation' incorporates:
    //   Gain: '<S7>/Gain6'
    //   Inport: '<Root>/cmd'

    state = 45.0F * rtU.cmd.roll;
    if (state > 45.0F) {
      state = 45.0F;
    } else {
      if (state < -45.0F) {
        state = -45.0F;
      }
    }

    // Sum: '<S7>/Sum4' incorporates:
    //   Gain: '<S20>/deg2rad'
    //   Inport: '<Root>/measure'

    state = rtU.measure.EulerAngles[0] - 0.0174532924F * state;

    // Sum: '<S7>/Sum' incorporates:
    //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator1'
    //   Gain: '<S7>/Gain'
    //   Inport: '<Root>/measure'

    rtDW.VectorConcatenate[0] = ((0.0F - 0.25F * rtU.measure.omega_Kb[0]) -
      state) - rtDW.DiscreteTimeIntegrator1_DSTATE[0];

    // Update for DiscreteIntegrator: '<S7>/Discrete-Time Integrator1' incorporates:
    //   Inport: '<Root>/measure'
    //   Sum: '<S7>/Sum1'

    rtDW.DiscreteTimeIntegrator1_DSTATE[0] += (rtU.measure.omega_Kb[0] + state) *
      0.0025F;

    // Saturate: '<S7>/Saturation' incorporates:
    //   Gain: '<S7>/Gain6'
    //   Inport: '<Root>/cmd'

    state = 45.0F * rtU.cmd.pitch;
    if (state > 45.0F) {
      state = 45.0F;
    } else {
      if (state < -45.0F) {
        state = -45.0F;
      }
    }

    // Sum: '<S7>/Sum4' incorporates:
    //   Gain: '<S20>/deg2rad'
    //   Inport: '<Root>/measure'

    state = rtU.measure.EulerAngles[1] - 0.0174532924F * state;

    // Sum: '<S7>/Sum' incorporates:
    //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator1'
    //   Gain: '<S7>/Gain'
    //   Inport: '<Root>/measure'

    rtDW.VectorConcatenate[1] = ((0.0F - 0.25F * rtU.measure.omega_Kb[1]) -
      state) - rtDW.DiscreteTimeIntegrator1_DSTATE[1];

    // Update for DiscreteIntegrator: '<S7>/Discrete-Time Integrator1' incorporates:
    //   Inport: '<Root>/measure'
    //   Sum: '<S7>/Sum1'

    rtDW.DiscreteTimeIntegrator1_DSTATE[1] += (rtU.measure.omega_Kb[1] + state) *
      0.0025F;

    // Sum: '<S7>/Sum' incorporates:
    //   Gain: '<S7>/Gain'
    //   Inport: '<Root>/measure'

    rtDW.VectorConcatenate[2] = 0.0F - 0.25F * rtU.measure.omega_Kb[2];

    // Update for DiscreteIntegrator: '<S7>/Discrete-Time Integrator1' incorporates:
    //   Inport: '<Root>/measure'
    //   Sum: '<S7>/Sum1'
    //   Sum: '<S7>/Sum4'

    rtDW.DiscreteTimeIntegrator1_DSTATE[2] += (rtU.measure.omega_Kb[2] +
      rtU.measure.EulerAngles[2]) * 0.0025F;

    // Sum: '<S19>/Sum2' incorporates:
    //   DiscreteIntegrator: '<S19>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S19>/Discrete-Time Integrator y_dt'
    //   DotProduct: '<S2>/Dot Product'
    //   Gain: '<S19>/2*d//omega'
    //   Inport: '<Root>/measure'
    //   Sqrt: '<S2>/Sqrt'
    //   Sum: '<S19>/Sum3'

    state = std::sqrt((rtU.measure.V_Kg[0] * rtU.measure.V_Kg[0] +
                       rtU.measure.V_Kg[1] * rtU.measure.V_Kg[1]) +
                      rtU.measure.V_Kg[2] * rtU.measure.V_Kg[2]) - (0.2F *
      rtDW.DiscreteTimeIntegratory_dt_DSTA + rtDW.DiscreteTimeIntegratory_DSTATE);

    // Sum: '<S7>/Sum2' incorporates:
    //   Constant: '<S7>/Constant'
    //   DiscreteIntegrator: '<S19>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S19>/Discrete-Time Integrator y_dt'
    //   DiscreteIntegrator: '<S7>/Discrete-Time Integrator'
    //   Gain: '<S7>/Gain5'

    rtDW.VectorConcatenate[3] = ((14.0F - rtDW.DiscreteTimeIntegratory_DSTATE) -
      0.25F * rtDW.DiscreteTimeIntegratory_dt_DSTA) +
      rtDW.DiscreteTimeIntegrator_DSTATE;

    // Update for DiscreteIntegrator: '<S19>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S19>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTATE += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DSTA;

    // Update for DiscreteIntegrator: '<S19>/Discrete-Time Integrator y_dt' incorporates:
    //   Gain: '<S19>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_DSTA += 100.0F * state * 0.0025F;

    // Update for DiscreteIntegrator: '<S7>/Discrete-Time Integrator'
    rtDW.DiscreteTimeIntegrator_DSTATE += 0.0025F * residuum;
    if (rtDW.DiscreteTimeIntegrator_DSTATE >= 0.8F) {
      rtDW.DiscreteTimeIntegrator_DSTATE = 0.8F;
    } else {
      if (rtDW.DiscreteTimeIntegrator_DSTATE <= 0.0F) {
        rtDW.DiscreteTimeIntegrator_DSTATE = 0.0F;
      }
    }

    // End of Update for DiscreteIntegrator: '<S7>/Discrete-Time Integrator'
  }

  // End of Outputs for SubSystem: '<S2>/Subsystem2'

  // Sum: '<S6>/Add' incorporates:
  //   Delay: '<S6>/Delay'
  //   Delay: '<S6>/Delay1'

  rtb_Add = rtDW.Delay1_DSTATE + rtDW.Delay_DSTATE;

  // Switch: '<S2>/Switch'
  if (rtb_Gain[7] > 0.5F) {
    // Saturate: '<S2>/Saturation2'
    if (rtb_Gain[6] > 1.0F) {
      residuum = 1.0F;
    } else if (rtb_Gain[6] < 0.0F) {
      residuum = 0.0F;
    } else {
      residuum = rtb_Gain[6];
    }

    // End of Saturate: '<S2>/Saturation2'

    // MATLAB Function: '<S6>/MATLAB Function'
    i = (int32_T)rtb_Add;

    // Outport: '<Root>/channels' incorporates:
    //   Constant: '<S6>/Constant2'
    //   MATLAB Function: '<S2>/MATLAB Function'
    //   MATLAB Function: '<S6>/MATLAB Function'
    //   Product: '<S2>/Multiply'
    //   Sum: '<S2>/Add'

    rtY.channels[0] = rtConstP.Constant2_Value[i - 1] * residuum +
      rtDW.VectorConcatenate[0];
    rtY.channels[1] = rtConstP.Constant2_Value[i + 2002] * residuum +
      rtDW.VectorConcatenate[1];
    rtY.channels[3] = rtConstP.Constant2_Value[i + 4005] * residuum +
      rtDW.VectorConcatenate[2];
    rtY.channels[2] = rtDW.VectorConcatenate[3];
  } else {
    // Outport: '<Root>/channels' incorporates:
    //   Inport: '<Root>/cmd'
    //   MATLAB Function: '<S2>/MATLAB Function'

    rtY.channels[0] = rtU.cmd.roll;
    rtY.channels[1] = rtU.cmd.pitch;
    rtY.channels[3] = rtU.cmd.yaw;
    rtY.channels[2] = rtU.cmd.thr;
  }

  // End of Switch: '<S2>/Switch'

  // Product: '<S4>/Divide' incorporates:
  //   Bias: '<S2>/Add Constant'
  //   Constant: '<S4>/Constant2'
  //   Inport: '<Root>/measure'

  residuum = 1.0F / (rtU.measure.rangefinder[0] + 1.0F);
  rtb_Switch_i_idx_0 = 1.0F / (rtU.measure.rangefinder[1] + 1.0F);
  rtb_Switch_i_idx_1 = 1.0F / (rtU.measure.rangefinder[2] + 1.0F);

  // DotProduct: '<S4>/Dot Product'
  state = 0.0F;
  for (i = 0; i < 3; i++) {
    // Product: '<S4>/Multiply' incorporates:
    //   Constant: '<S4>/Constant1'

    rtb_Switch_i_idx_2 = rtConstP.Constant1_Value[i + 6] * rtb_Switch_i_idx_1 +
      (rtConstP.Constant1_Value[i + 3] * rtb_Switch_i_idx_0 +
       rtConstP.Constant1_Value[i] * residuum);

    // DotProduct: '<S4>/Dot Product' incorporates:
    //   Math: '<S14>/Math Function'

    rtb_Switch_i_idx_3 = rtb_Switch_i_idx_2 * rtb_Switch_i_idx_2;
    state += rtb_Switch_i_idx_3;

    // Math: '<S14>/Math Function'
    rtb_MathFunction[i] = rtb_Switch_i_idx_3;

    // Product: '<S4>/Multiply' incorporates:
    //   Constant: '<S4>/Constant1'

    tunableEnvironment[i] = rtb_Switch_i_idx_2;
  }

  // Product: '<S4>/Divide1' incorporates:
  //   Constant: '<S4>/Constant'
  //   DotProduct: '<S4>/Dot Product'
  //   Sqrt: '<S4>/Sqrt'

  residuum = 1.0F / std::sqrt(state);

  // Sum: '<S14>/Sum of Elements'
  state = (rtb_MathFunction[0] + rtb_MathFunction[1]) + rtb_MathFunction[2];

  // Math: '<S14>/Math Function1'
  //
  //  About '<S14>/Math Function1':
  //   Operator: sqrt

  if (state < 0.0F) {
    state = -std::sqrt(std::abs(state));
  } else {
    state = std::sqrt(state);
  }

  // End of Math: '<S14>/Math Function1'

  // Switch: '<S14>/Switch' incorporates:
  //   Constant: '<S14>/Constant'
  //   Product: '<S14>/Product'

  if (state > 0.0F) {
    rtb_Switch_i_idx_0 = tunableEnvironment[0];
    rtb_Switch_i_idx_1 = tunableEnvironment[1];
    rtb_Switch_i_idx_2 = tunableEnvironment[2];
    rtb_Switch_i_idx_3 = state;
  } else {
    rtb_Switch_i_idx_0 = 0.0F;
    rtb_Switch_i_idx_1 = 0.0F;
    rtb_Switch_i_idx_2 = 0.0F;
    rtb_Switch_i_idx_3 = 1.0F;
  }

  // End of Switch: '<S14>/Switch'

  // Outport: '<Root>/logs' incorporates:
  //   Inport: '<Root>/measure'

  rtY.logs[3] = rtU.measure.s_Kg[0];

  // Product: '<S14>/Divide'
  tunableEnvironment[1] = rtb_Switch_i_idx_1 / rtb_Switch_i_idx_3;

  // Outport: '<Root>/logs' incorporates:
  //   Inport: '<Root>/measure'

  rtY.logs[4] = rtU.measure.s_Kg[1];

  // Product: '<S14>/Divide'
  tunableEnvironment[2] = rtb_Switch_i_idx_2 / rtb_Switch_i_idx_3;

  // Outport: '<Root>/logs' incorporates:
  //   Inport: '<Root>/measure'

  rtY.logs[5] = rtU.measure.s_Kg[2];

  // Gain: '<S11>/rad2deg' incorporates:
  //   MATLAB Function: '<S4>/MATLAB Function'

  state = 57.2957802F * std::atan2(tunableEnvironment[1], tunableEnvironment[2]);

  // Gain: '<S10>/rad2deg' incorporates:
  //   MATLAB Function: '<S4>/MATLAB Function'
  //   Product: '<S14>/Divide'

  rtb_Switch_i_idx_0 = std::atan2(-(rtb_Switch_i_idx_0 / rtb_Switch_i_idx_3),
    std::sqrt(tunableEnvironment[1] * tunableEnvironment[1] +
              tunableEnvironment[2] * tunableEnvironment[2])) * 57.2957802F;

  // MATLAB Function: '<S2>/MATLAB Function' incorporates:
  //   Outport: '<Root>/channels'

  rtY.channels[4] = residuum;
  rtY.channels[5] = state;
  rtY.channels[6] = rtb_Switch_i_idx_0;
  rtY.channels[7] = 0.0F;

  // Outport: '<Root>/logs' incorporates:
  //   MATLAB Function: '<S2>/trajGetMatch'

  rtY.logs[0] = residuum;
  rtY.logs[1] = state;
  rtY.logs[2] = rtb_Switch_i_idx_0;
  rtY.logs[6] = b_size;
  rtY.logs[7] = axis_sel;
  rtY.logs[8] = num_of_splines;
  for (i = 0; i < 6; i++) {
    rtY.logs[i + 9] = rtb_state_vec[i];
  }

  // Update for Memory: '<S9>/Memory2'
  memcpy(&rtDW.Memory2_PreviousInput[0], &rtb_uvwxb[0], 300U * sizeof(real32_T));

  // Update for UnitDelay: '<S9>/Unit Delay' incorporates:
  //   MATLAB Function: '<S9>/trajFromWaypoints'

  rtDW.UnitDelay_DSTATE = traj;

  // Update for UnitDelay: '<S26>/Delay Input1'
  //
  //  Block description for '<S26>/Delay Input1':
  //
  //   Store in Global RAM

  rtDW.DelayInput1_DSTATE = rtb_Compare;

  // Update for Memory: '<S9>/Memory1'
  rtDW.Memory1_PreviousInput[0] = rtb_AnormAlfaRhoPhi_idx_0;
  rtDW.Memory1_PreviousInput[1] = rtb_AnormAlfaRhoPhi_idx_1;
  rtDW.Memory1_PreviousInput[2] = rtb_AnormAlfaRhoPhi_idx_2;
  rtDW.Memory1_PreviousInput[3] = rtb_AnormAlfaRhoPhi_idx_3;

  // Update for Memory: '<S9>/Memory'
  for (i = 0; i < 6; i++) {
    rtDW.Memory_PreviousInput[i] = rtb_state_vec[i];
  }

  // End of Update for Memory: '<S9>/Memory'

  // Update for Delay: '<S6>/Delay1' incorporates:
  //   Constant: '<S6>/Constant'

  rtDW.Delay1_DSTATE = 1.0F;

  // Update for UnitDelay: '<S6>/Unit Delay' incorporates:
  //   Switch: '<S6>/Switch'

  rtDW.UnitDelay_DSTATE_l = (rtb_Add >= 2003.0F);

  // Update for UnitDelay: '<S15>/Delay Input1'
  //
  //  Block description for '<S15>/Delay Input1':
  //
  //   Store in Global RAM

  rtDW.DelayInput1_DSTATE_e = rtb_Compare_l;

  // Update for Memory: '<S17>/Memory' incorporates:
  //   CombinatorialLogic: '<S17>/Logic'

  rtDW.Memory_PreviousInput_i = rtConstP.Logic_table[(uint32_T)loop_ub];

  // Update for Delay: '<S6>/Delay'
  rtDW.Delay_DSTATE = rtb_Add;
}

// Model initialize function
void MatlabControllerClass::initialize()
{
  {
    int32_T i;

    // ConstCode for Outport: '<Root>/function_channels'
    for (i = 0; i < 8; i++) {
      rtY.function_channels[i] = rtConstB.DataTypeConversion1[i];
    }

    // End of ConstCode for Outport: '<Root>/function_channels'

    // InitializeConditions for UnitDelay: '<S9>/Unit Delay'
    rtDW.UnitDelay_DSTATE = rtConstP.pooled3;

    // InitializeConditions for Memory: '<S17>/Memory'
    rtDW.Memory_PreviousInput_i = true;

    // InitializeConditions for Delay: '<S6>/Delay'
    rtDW.Delay_DSTATE = 1.0F;

    // SystemInitialize for Enabled SubSystem: '<S2>/Subsystem2'
    // InitializeConditions for DiscreteIntegrator: '<S19>/Discrete-Time Integrator y' 
    rtDW.DiscreteTimeIntegratory_DSTATE = 14.0F;

    // End of SystemInitialize for SubSystem: '<S2>/Subsystem2'
  }
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
