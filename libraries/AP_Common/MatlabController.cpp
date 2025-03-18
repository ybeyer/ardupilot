//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: MatlabController.cpp
//
// Code generated for Simulink model 'ArduPlane_LindiPlane'.
//
// Model version                  : 1.795
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Mon Feb 17 19:39:54 2025
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
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

    { 117U, 49U, 1U, 117U, 50U, 1U, 117U, 51U, 1U, 117U, 52U, 1U, 117U, 53U, 1U,
      117U, 54U, 1U, 117U, 55U, 1U, 117U, 56U, 1U, 117U, 57U, 1U, 117U, 49U, 48U,
      117U, 49U, 49U, 80U, 104U, 105U, 84U, 104U, 101U, 80U, 115U, 105U },

    { 77U, 76U, 49U, 0U }
  }, { 14U,
    { 112U, 1U, 1U, 113U, 1U, 1U, 114U, 1U, 1U, 112U, 100U, 116U, 113U, 100U,
      116U, 114U, 100U, 116U, 101U, 120U, 103U, 101U, 121U, 103U, 101U, 122U,
      103U, 101U, 117U, 103U, 101U, 118U, 103U, 101U, 119U, 103U, 101U, 97U,
      120U, 101U, 97U, 121U },

    { 77U, 76U, 50U, 0U }
  }, { 14U,
    { 101U, 97U, 122U, 86U, 65U, 102U, 97U, 105U, 114U, 119U, 112U, 105U, 115U,
      116U, 97U, 116U, 1U, 1U, 120U, 103U, 114U, 121U, 103U, 114U, 122U, 103U,
      114U, 120U, 103U, 102U, 121U, 103U, 102U, 122U, 103U, 102U, 120U, 103U,
      109U, 121U, 103U, 109U },

    { 77U, 76U, 51U, 0U }
  }, { 14U,
    { 122U, 103U, 109U, 112U, 1U, 1U, 113U, 1U, 1U, 114U, 1U, 1U, 97U, 120U,
      103U, 97U, 121U, 103U, 97U, 122U, 103U, 102U, 109U, 1U, 105U, 116U, 101U,
      112U, 49U, 1U, 112U, 50U, 1U, 112U, 51U, 1U, 112U, 52U, 1U, 112U, 53U, 1U
    },

    { 77U, 76U, 52U, 0U }
  }, { 13U,
    { 112U, 54U, 1U, 112U, 55U, 1U, 112U, 56U, 1U, 97U, 122U, 49U, 97U, 122U,
      50U, 97U, 122U, 51U, 97U, 122U, 52U, 97U, 122U, 53U, 97U, 122U, 54U, 97U,
      122U, 55U, 97U, 122U, 56U, 103U, 117U, 119U, 103U, 117U, 104U, 0U, 0U, 0U
    },

    { 77U, 76U, 53U, 0U }
  } } ;

extern real32_T rt_hypotf(real32_T u0, real32_T u1);
static void PT2split(const real32_T rtu_y_dt_vec[6], const real32_T rtu_y_vec[6],
                     real32_T rty_y[3], real32_T rty_y_dt[3], real32_T
                     rty_y_dt2[3]);
static void MATLABFunction(real32_T rtu_u0, real32_T rtu_u1, real32_T rtu_u2,
  real32_T rtu_u3, real32_T rtu_u4, real32_T rtu_delay, real32_T rtu_ts,
  real32_T *rty_u_delay);
static void wrapangle(real32_T rtu_angle, real32_T *rty_angle_0_2pi);

//
// Output and update for atomic system:
//    '<S55>/PT2 split'
//    '<S56>/PT2 split'
//
static void PT2split(const real32_T rtu_y_dt_vec[6], const real32_T rtu_y_vec[6],
                     real32_T rty_y[3], real32_T rty_y_dt[3], real32_T
                     rty_y_dt2[3])
{
  // :  y = y_vec(2,:)';
  // :  y_dt = y_dt_vec(2,:)';
  // :  y_dt2 = y_dt_vec(1,:)';
  rty_y[0] = rtu_y_vec[1];
  rty_y_dt[0] = rtu_y_dt_vec[1];
  rty_y_dt2[0] = rtu_y_dt_vec[0];
  rty_y[1] = rtu_y_vec[3];
  rty_y_dt[1] = rtu_y_dt_vec[3];
  rty_y_dt2[1] = rtu_y_dt_vec[2];
  rty_y[2] = rtu_y_vec[5];
  rty_y_dt[2] = rtu_y_dt_vec[5];
  rty_y_dt2[2] = rtu_y_dt_vec[4];
}

//
// Output and update for atomic system:
//    '<S77>/MATLAB Function'
//    '<S94>/MATLAB Function'
//    '<S98>/MATLAB Function'
//    '<S131>/MATLAB Function'
//    '<S135>/MATLAB Function'
//    '<S158>/MATLAB Function'
//
static void MATLABFunction(real32_T rtu_u0, real32_T rtu_u1, real32_T rtu_u2,
  real32_T rtu_u3, real32_T rtu_u4, real32_T rtu_delay, real32_T rtu_ts,
  real32_T *rty_u_delay)
{
  real32_T num_delays;

  // :  num_delays = floor( divideFinite( delay, ts ) );
  num_delays = rtu_ts;

  // 'divideFinite:29' if numel(B)>1
  // 'divideFinite:31' else
  // 'divideFinite:32' if abs(B)<eps
  if (std::abs(rtu_ts) < 2.22044605E-16F) {
    // 'divideFinite:33' B(:) = eps;
    num_delays = 2.22044605E-16F;
  }

  // 'divideFinite:36' C = A ./ B;
  num_delays = std::floor(rtu_delay / num_delays);

  // :  num_delays(:) = min( num_delays, 4 );
  if (num_delays > 4.0F) {
    num_delays = 4.0F;
  }

  // :  num_delays(:) = max( num_delays, 0 );
  if (num_delays < 0.0F) {
    num_delays = 0.0F;
  }

  // :  if num_delays == 0
  if (num_delays == 0.0F) {
    // :  u_delay = u0;
    *rty_u_delay = rtu_u0;
  } else if (num_delays == 1.0F) {
    // :  elseif num_delays == 1
    // :  u_delay = u1;
    *rty_u_delay = rtu_u1;
  } else if (num_delays == 2.0F) {
    // :  elseif num_delays == 2
    // :  u_delay = u2;
    *rty_u_delay = rtu_u2;
  } else if (num_delays == 3.0F) {
    // :  elseif num_delays == 3
    // :  u_delay = u3;
    *rty_u_delay = rtu_u3;
  } else {
    // :  else
    // :  u_delay = u4;
    *rty_u_delay = rtu_u4;
  }
}

//
// Output and update for atomic system:
//    '<S121>/wrap angle'
//    '<S121>/wrap angle1'
//
static void wrapangle(real32_T rtu_angle, real32_T *rty_angle_0_2pi)
{
  real32_T x;

  // :  angle_0_2pi = wrapAngle(angle);
  // 'wrapAngle:20' x = abs( angle );
  x = std::abs(rtu_angle);

  // 'wrapAngle:21' y = 2*pi;
  // 'wrapAngle:22' n = floor(x./y);
  // 'wrapAngle:23' m = x - n.*y;
  // 'wrapAngle:24' mod_heading = m;
  // 'wrapAngle:27' if angle >= 0
  if (rtu_angle >= 0.0F) {
    // 'wrapAngle:28' angle_0_2pi = mod_heading;
    *rty_angle_0_2pi = x - std::floor(x / 6.28318548F) * 6.28318548F;
  } else {
    // 'wrapAngle:29' else
    // 'wrapAngle:30' angle_0_2pi = 2*pi - mod_heading;
    *rty_angle_0_2pi = 6.28318548F - (x - std::floor(x / 6.28318548F) *
      6.28318548F);
  }
}

// Function for MATLAB Function: '<S37>/Avoid zero speed'
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

//
// Function for MATLAB Function: '<S37>/WpNav Matching'
// function circ_seg = wpnavCircSeg( waypoints3x3, wp_radius )
//
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

  // 'wpnavCircSeg:29' diff1 = waypoints3x3(:,2)-waypoints3x3(:,1);
  // 'wpnavCircSeg:30' diff2 = waypoints3x3(:,3)-waypoints3x3(:,2);
  circ_seg_start[0] = waypoints3x3[3] - waypoints3x3[0];
  circ_seg_end[0] = waypoints3x3[6] - waypoints3x3[3];
  circ_seg_start[1] = waypoints3x3[4] - waypoints3x3[1];
  circ_seg_end[1] = waypoints3x3[7] - waypoints3x3[4];
  circ_seg_start[2] = waypoints3x3[5] - waypoints3x3[2];
  circ_seg_end[2] = waypoints3x3[8] - waypoints3x3[5];

  // 'wpnavCircSeg:31' dist1 = norm(diff1,2);
  dist1_tmp = norm_c(circ_seg_start);

  // 'wpnavCircSeg:32' dist2 = norm(diff2,2);
  dist2_tmp = norm_c(circ_seg_end);

  // 'wpnavCircSeg:33' wp_rad = min([wp_radius,dist1/2,dist2/2]);
  circ_seg_n[1] = dist1_tmp / 2.0F;
  circ_seg_n[2] = dist2_tmp / 2.0F;
  *circ_seg_wp_rad = wp_radius;
  if (wp_radius > circ_seg_n[1]) {
    *circ_seg_wp_rad = circ_seg_n[1];
  }

  if (*circ_seg_wp_rad > circ_seg_n[2]) {
    *circ_seg_wp_rad = circ_seg_n[2];
  }

  // 'wpnavCircSeg:35' alpha = wrapAngle( acosReal( divideFinite( dot(diff1,diff2), dist1*dist2 ) ) ); 
  dist1 = dist1_tmp * dist2_tmp;

  // 'divideFinite:29' if numel(B)>1
  // 'divideFinite:31' else
  // 'divideFinite:32' if abs(B)<eps
  if (std::abs(dist1) < 2.22044605E-16F) {
    // 'divideFinite:33' B(:) = eps;
    dist1 = 2.22044605E-16F;
  }

  // 'divideFinite:36' C = A ./ B;
  absxk = ((circ_seg_start[0] * circ_seg_end[0] + circ_seg_start[1] *
            circ_seg_end[1]) + circ_seg_start[2] * circ_seg_end[2]) / dist1;

  // 'acosReal:28' if numel(y) > 1
  // 'acosReal:31' else
  // 'acosReal:32' y = max(-1,min(1,y));
  if (1.0F <= absxk) {
    absxk = 1.0F;
  }

  // 'acosReal:34' y = acos(y);
  if (-1.0F >= absxk) {
    absxk = -1.0F;
  }

  absxk = std::acos(absxk);

  // 'wrapAngle:20' x = abs( angle );
  dist1 = std::abs(absxk);

  // 'wrapAngle:21' y = 2*pi;
  // 'wrapAngle:22' n = floor(x./y);
  // 'wrapAngle:23' m = x - n.*y;
  // 'wrapAngle:24' mod_heading = m;
  // 'wrapAngle:27' if angle >= 0
  if (absxk >= 0.0F) {
    // 'wrapAngle:28' angle_0_2pi = mod_heading;
    *circ_seg_angle = dist1 - std::floor(dist1 / 6.28318548F) * 6.28318548F;
  } else {
    // 'wrapAngle:29' else
    // 'wrapAngle:30' angle_0_2pi = 2*pi - mod_heading;
    *circ_seg_angle = 6.28318548F - (dist1 - std::floor(dist1 / 6.28318548F) *
      6.28318548F);
  }

  // 'wpnavCircSeg:37' if alpha < 100*eps(alpha)
  absxk = std::abs(*circ_seg_angle);
  if (absxk <= 1.17549435E-38F) {
    absxk = 1.4013E-45F;
  } else {
    std::frexp(absxk, &exponent);
    absxk = std::ldexp(1.0F, exponent - 24);
  }

  if (*circ_seg_angle < 100.0F * absxk) {
    // 'wpnavCircSeg:38' wp_rad(:) = 0;
    *circ_seg_wp_rad = 0.0F;
  }

  // 'wpnavCircSeg:40' d = 2*wp_rad*cos(alpha/2);
  // 'wpnavCircSeg:41' r = sqrtReal( divideFinite( (d/2)^2, (1-(cos(alpha/2))^2) ) ); 
  dist1 = std::cos(*circ_seg_angle / 2.0F);
  absxk = 2.0F * *circ_seg_wp_rad * dist1 / 2.0F;
  dist1 = 1.0F - dist1 * dist1;

  // 'divideFinite:29' if numel(B)>1
  // 'divideFinite:31' else
  // 'divideFinite:32' if abs(B)<eps
  if (std::abs(dist1) < 2.22044605E-16F) {
    // 'divideFinite:33' B(:) = eps;
    dist1 = 2.22044605E-16F;
  }

  // 'divideFinite:36' C = A ./ B;
  absxk = absxk * absxk / dist1;

  // 'sqrtReal:27' if numel(y) > 1
  // 'sqrtReal:29' else
  // 'sqrtReal:30' y = max(0,y);
  // 'sqrtReal:32' y = sqrt(y);
  if (0.0F >= absxk) {
    absxk = 0.0F;
  }

  *circ_seg_r = std::sqrt(absxk);

  // 'wpnavCircSeg:43' n = cross( diff1, diff2 );
  circ_seg_n[0] = circ_seg_start[1] * circ_seg_end[2] - circ_seg_start[2] *
    circ_seg_end[1];
  circ_seg_n[1] = circ_seg_start[2] * circ_seg_end[0] - circ_seg_start[0] *
    circ_seg_end[2];
  circ_seg_n[2] = circ_seg_start[0] * circ_seg_end[1] - circ_seg_start[1] *
    circ_seg_end[0];

  // 'wpnavCircSeg:44' n = divideFinite( n, norm(n,2) );
  absxk = norm_c(circ_seg_n);

  // 'divideFinite:29' if numel(B)>1
  // 'divideFinite:31' else
  // 'divideFinite:32' if abs(B)<eps
  if (std::abs(absxk) < 2.22044605E-16F) {
    // 'divideFinite:33' B(:) = eps;
    absxk = 2.22044605E-16F;
  }

  // 'divideFinite:36' C = A ./ B;
  circ_seg_n[0] /= absxk;
  circ_seg_n[1] /= absxk;
  circ_seg_n[2] /= absxk;

  // 'wpnavCircSeg:45' if r > 100000
  if (*circ_seg_r > 100000.0F) {
    // 'wpnavCircSeg:46' r(:) = 100000;
    *circ_seg_r = 100000.0F;

    // 'wpnavCircSeg:47' n(:) = [0;0;1];
    circ_seg_n[0] = 0.0F;
    circ_seg_n[1] = 0.0F;
    circ_seg_n[2] = 1.0F;
  }

  // 'wpnavCircSeg:49' diff1_unit = divideFinite( diff1, norm(diff1,2) );
  absxk = dist1_tmp;

  // 'divideFinite:29' if numel(B)>1
  // 'divideFinite:31' else
  // 'divideFinite:32' if abs(B)<eps
  if (std::abs(dist1_tmp) < 2.22044605E-16F) {
    // 'divideFinite:33' B(:) = eps;
    absxk = 2.22044605E-16F;
  }

  // 'divideFinite:36' C = A ./ B;
  circ_seg_start[0] /= absxk;
  circ_seg_start[1] /= absxk;
  dist1_tmp = circ_seg_start[2] / absxk;

  // 'wpnavCircSeg:50' diff2_unit = divideFinite( diff2, norm(diff2,2) );
  absxk = dist2_tmp;

  // 'divideFinite:29' if numel(B)>1
  // 'divideFinite:31' else
  // 'divideFinite:32' if abs(B)<eps
  if (std::abs(dist2_tmp) < 2.22044605E-16F) {
    // 'divideFinite:33' B(:) = eps;
    absxk = 2.22044605E-16F;
  }

  // 'divideFinite:36' C = A ./ B;
  // 'wpnavCircSeg:51' r_vec = cross(n,diff1_unit);
  circ_seg_center[0] = circ_seg_n[1] * dist1_tmp - circ_seg_n[2] *
    circ_seg_start[1];
  circ_seg_center[1] = circ_seg_n[2] * circ_seg_start[0] - circ_seg_n[0] *
    dist1_tmp;
  circ_seg_center[2] = circ_seg_n[0] * circ_seg_start[1] - circ_seg_n[1] *
    circ_seg_start[0];

  // 'wpnavCircSeg:52' r_vec = r * divideFinite( r_vec, norm(r_vec,2) );
  dist1 = norm_c(circ_seg_center);

  // 'divideFinite:29' if numel(B)>1
  // 'divideFinite:31' else
  // 'divideFinite:32' if abs(B)<eps
  if (std::abs(dist1) < 2.22044605E-16F) {
    // 'divideFinite:33' B(:) = eps;
    dist1 = 2.22044605E-16F;
  }

  // 'divideFinite:36' C = A ./ B;
  // 'wpnavCircSeg:54' seg_start = waypoints3x3(:,2) - wp_rad*diff1_unit;
  // 'wpnavCircSeg:55' seg_end = waypoints3x3(:,2) + wp_rad*diff2_unit;
  // 'wpnavCircSeg:57' center = waypoints3x3(:,2) - wp_rad*diff1_unit + r_vec;
  // 'wpnavCircSeg:60' circ_seg.r = r;
  // 'wpnavCircSeg:62' circ_seg.center = center;
  // 'wpnavCircSeg:64' circ_seg.n = n;
  // 'wpnavCircSeg:66' circ_seg.angle = alpha;
  // 'wpnavCircSeg:68' circ_seg.start = seg_start;
  // 'wpnavCircSeg:70' circ_seg.end = seg_end;
  // 'wpnavCircSeg:72' circ_seg.wp_rad = wp_rad;
  // 'wpnavCircSeg:74' circ_seg.wp = waypoints3x3(:,2);
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

//
// Function for MATLAB Function: '<S37>/WpNav Matching'
// function v_rot = axisAngle( v, axis, angle )
//
void MatlabControllerClass::axisAngle(const real32_T v[3], real32_T axis[3],
  real32_T angle, real32_T v_rot[3])
{
  real32_T axis_length;
  real32_T sin_angle;
  real32_T axis_0;
  real32_T v_rot_tmp;

  // 'axisAngle:23' v_rot       = zeros( size(v), class(v) );
  // 'axisAngle:24' axis_length = vecnorm( axis, 2, 1 );
  axis_length = norm_c(axis);

  // 'axisAngle:25' for i = 1:3
  // 'axisAngle:26' axis(i,:) = divideFinite( axis(i,:), axis_length );
  sin_angle = axis_length;

  // 'divideFinite:29' if numel(B)>1
  // 'divideFinite:31' else
  // 'divideFinite:32' if abs(B)<eps
  axis_0 = std::abs(axis_length);
  if (axis_0 < 2.22044605E-16F) {
    // 'divideFinite:33' B(:) = eps;
    sin_angle = 2.22044605E-16F;
  }

  // 'divideFinite:36' C = A ./ B;
  axis[0] /= sin_angle;

  // 'axisAngle:26' axis(i,:) = divideFinite( axis(i,:), axis_length );
  sin_angle = axis_length;

  // 'divideFinite:29' if numel(B)>1
  // 'divideFinite:31' else
  // 'divideFinite:32' if abs(B)<eps
  if (axis_0 < 2.22044605E-16F) {
    // 'divideFinite:33' B(:) = eps;
    sin_angle = 2.22044605E-16F;
  }

  // 'divideFinite:36' C = A ./ B;
  axis[1] /= sin_angle;

  // 'axisAngle:26' axis(i,:) = divideFinite( axis(i,:), axis_length );
  sin_angle = axis_length;

  // 'divideFinite:29' if numel(B)>1
  // 'divideFinite:31' else
  // 'divideFinite:32' if abs(B)<eps
  if (axis_0 < 2.22044605E-16F) {
    // 'divideFinite:33' B(:) = eps;
    sin_angle = 2.22044605E-16F;
  }

  // 'divideFinite:36' C = A ./ B;
  axis_0 = axis[2] / sin_angle;

  // 'axisAngle:28' cos_angle   = cos(angle);
  axis_length = std::cos(angle);

  // 'axisAngle:29' cos_angle_1 = 1-cos_angle;
  // 'axisAngle:30' sin_angle   = sin(angle);
  sin_angle = std::sin(angle);

  // 'axisAngle:31' cross_prod  = crossFast(axis,v);
  // 'crossFast:26' len = max(size(a,2),size(b,2));
  // 'crossFast:27' c = zeros(3,len);
  // 'crossFast:29' c(3,:) = a(1,:).*b(2,:)-a(2,:).*b(1,:);
  // 'crossFast:30' c(1,:) = a(2,:).*b(3,:)-a(3,:).*b(2,:);
  // 'crossFast:31' c(2,:) = a(3,:).*b(1,:)-a(1,:).*b(3,:);
  // 'axisAngle:32' dot_prod    = dot(axis,v);
  // 'axisAngle:33' for i = 1:3
  // 'axisAngle:34' v_rot(i,:) = cos_angle.*v(i,:) + sin_angle.*cross_prod(i,:) ... 
  // 'axisAngle:35'         + cos_angle_1 .* dot_prod .* axis(i,:);
  v_rot_tmp = (1.0F - axis_length) * ((axis[0] * v[0] + axis[1] * v[1]) + axis_0
    * v[2]);
  v_rot[0] = ((axis[1] * v[2] - axis_0 * v[1]) * sin_angle + axis_length * v[0])
    + v_rot_tmp * axis[0];

  // 'axisAngle:34' v_rot(i,:) = cos_angle.*v(i,:) + sin_angle.*cross_prod(i,:) ... 
  // 'axisAngle:35'         + cos_angle_1 .* dot_prod .* axis(i,:);
  v_rot[1] = ((axis_0 * v[0] - axis[0] * v[2]) * sin_angle + axis_length * v[1])
    + v_rot_tmp * axis[1];

  // 'axisAngle:34' v_rot(i,:) = cos_angle.*v(i,:) + sin_angle.*cross_prod(i,:) ... 
  // 'axisAngle:35'         + cos_angle_1 .* dot_prod .* axis(i,:);
  v_rot[2] = ((axis[0] * v[1] - axis[1] * v[0]) * sin_angle + axis_length * v[2])
    + v_rot_tmp * axis_0;
}

//
// Function for MATLAB Function: '<S37>/WpNav Matching'
// function [p_match,t,d] = wpnavMatchCircSeg( circ_seg, p )
//
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

  // 'wpnavMatchCircSeg:33' H = dot( p - circ_seg.center, circ_seg.n );
  H = ((p[0] - circ_seg_center[0]) * circ_seg_n[0] + (p[1] - circ_seg_center[1])
       * circ_seg_n[1]) + (p[2] - circ_seg_center[2]) * circ_seg_n[2];

  // 'wpnavMatchCircSeg:34' p0 = p - H*circ_seg.n;
  // 'wpnavMatchCircSeg:36' r_p0 = p0 - circ_seg.center;
  // 'wpnavMatchCircSeg:37' r_start = circ_seg.start - circ_seg.center;
  r_p0[0] = (p[0] - H * circ_seg_n[0]) - circ_seg_center[0];
  r_start[0] = circ_seg_start[0] - circ_seg_center[0];
  r_p0[1] = (p[1] - H * circ_seg_n[1]) - circ_seg_center[1];
  r_start[1] = circ_seg_start[1] - circ_seg_center[1];
  r_p0[2] = (p[2] - H * circ_seg_n[2]) - circ_seg_center[2];
  r_start[2] = circ_seg_start[2] - circ_seg_center[2];

  // 'wpnavMatchCircSeg:38' r_p0_abs = norm( r_p0, 2 );
  // 'wpnavMatchCircSeg:39' r_start_abs = norm( r_start, 2 );
  // 'wpnavMatchCircSeg:40' num = dot( r_p0 , r_start );
  // 'wpnavMatchCircSeg:41' denom = r_p0_abs*r_start_abs;
  denom = norm_c(r_p0) * norm_c(r_start);

  // 'wpnavMatchCircSeg:42' angle_match = acosReal( divideFinite( num, denom ) ); 
  // 'divideFinite:29' if numel(B)>1
  // 'divideFinite:31' else
  // 'divideFinite:32' if abs(B)<eps
  if (std::abs(denom) < 2.22044605E-16F) {
    // 'divideFinite:33' B(:) = eps;
    denom = 2.22044605E-16F;
  }

  // 'divideFinite:36' C = A ./ B;
  H = ((r_p0[0] * r_start[0] + r_p0[1] * r_start[1]) + r_p0[2] * r_start[2]) /
    denom;

  // 'acosReal:28' if numel(y) > 1
  // 'acosReal:31' else
  // 'acosReal:32' y = max(-1,min(1,y));
  if (1.0F <= H) {
    H = 1.0F;
  }

  // 'acosReal:34' y = acos(y);
  // 'wpnavMatchCircSeg:44' t = divideFinite( angle_match, circ_seg.angle );
  denom = circ_seg_angle;

  // 'divideFinite:29' if numel(B)>1
  // 'divideFinite:31' else
  // 'divideFinite:32' if abs(B)<eps
  if (std::abs(circ_seg_angle) < 2.22044605E-16F) {
    // 'divideFinite:33' B(:) = eps;
    denom = 2.22044605E-16F;
  }

  // 'divideFinite:36' C = A ./ B;
  if (-1.0F >= H) {
    H = -1.0F;
  }

  *t = std::acos(H) / denom;

  // 'wpnavMatchCircSeg:46' p_match = wpnavCircSegGetPos( circ_seg, t );
  // 'wpnavCircSegGetPos:26' angle = t * circ_seg.angle;
  // 'wpnavCircSegGetPos:27' s_g = circ_seg.center + axisAngle(circ_seg.start-circ_seg.center,circ_seg.n,angle); 
  circ_seg_n_0[0] = circ_seg_n[0];
  circ_seg_n_0[1] = circ_seg_n[1];
  circ_seg_n_0[2] = circ_seg_n[2];
  axisAngle(r_start, circ_seg_n_0, *t * circ_seg_angle, tmp);

  // 'wpnavMatchCircSeg:48' d = norm( p_match - p, 2 );
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
  *d = norm_c(r_start);

  // 'wpnavMatchCircSeg:53' R_center = circ_seg.center - circ_seg.wp;
  // 'wpnavMatchCircSeg:54' R_center_norm = norm(R_center,2);
  H = norm_c(r_p0);

  // 'wpnavMatchCircSeg:55' R_center_unit = divideFinite( R_center, R_center_norm ); 
  denom = H;

  // 'divideFinite:29' if numel(B)>1
  // 'divideFinite:31' else
  // 'divideFinite:32' if abs(B)<eps
  if (std::abs(H) < 2.22044605E-16F) {
    // 'divideFinite:33' B(:) = eps;
    denom = 2.22044605E-16F;
  }

  // 'divideFinite:36' C = A ./ B;
  // 'wpnavMatchCircSeg:56' R_p = p - circ_seg.wp;
  r_p0[0] /= denom;
  r_p0[1] /= denom;

  // 'wpnavMatchCircSeg:57' D = dot( R_p, R_center_unit );
  // 'wpnavMatchCircSeg:58' if D > R_center_norm
  if (((p[0] - circ_seg_wp[0]) * r_p0[0] + (p[1] - circ_seg_wp[1]) * r_p0[1]) +
      (p[2] - circ_seg_wp[2]) * (r_p0[2] / denom) > H) {
    // 'wpnavMatchCircSeg:59' t(:) = 2;
    *t = 2.0F;
  }
}

//
// Function for MATLAB Function: '<S37>/WpNav Matching'
// function [p_match,t,d] = wpnavMatchLine( p1, p2, p )
//
void MatlabControllerClass::wpnavMatchLine(const real32_T p1[3], const real32_T
  p2[3], const real32_T p[3], real32_T p_match[3], real32_T *t, real32_T *d)
{
  real32_T denom;
  real32_T p_match_0[3];
  real32_T p_match_1;

  // 'wpnavMatchLine:36' b = p2-p1;
  // 'wpnavMatchLine:38' denom = dot(b,b);
  p_match_1 = p2[0] - p1[0];
  denom = p_match_1 * p_match_1;
  p_match[0] = p_match_1;
  p_match_1 = p2[1] - p1[1];
  denom += p_match_1 * p_match_1;
  p_match[1] = p_match_1;
  p_match_1 = p2[2] - p1[2];
  denom += p_match_1 * p_match_1;

  // 'wpnavMatchLine:39' if denom < 1
  if (denom < 1.0F) {
    // 'wpnavMatchLine:40' denom(:) = 1;
    denom = 1.0F;
  }

  // 'wpnavMatchLine:43' t = divideFinite( dot(p-p1,b), denom );
  // 'divideFinite:29' if numel(B)>1
  // 'divideFinite:31' else
  // 'divideFinite:32' if abs(B)<eps
  // 'divideFinite:36' C = A ./ B;
  *t = (((p[0] - p1[0]) * p_match[0] + (p[1] - p1[1]) * p_match[1]) + (p[2] -
         p1[2]) * p_match_1) / denom;

  // 'wpnavMatchLine:45' p_match = wpnavLineGetPos(p1,p2,t);
  // 'wpnavLineGetPos:29' b = p2 - p1;
  // 'wpnavLineGetPos:30' s_g = p1 + t*b;
  // 'wpnavMatchLine:46' d = norm( p_match - p, 2);
  denom = *t * p_match[0] + p1[0];
  p_match_0[0] = denom - p[0];
  p_match[0] = denom;
  denom = *t * p_match[1] + p1[1];
  p_match_0[1] = denom - p[1];
  p_match[1] = denom;
  denom = *t * p_match_1 + p1[2];
  p_match_0[2] = denom - p[2];
  p_match[2] = denom;
  *d = norm_c(p_match_0);
}

//
// Function for MATLAB Function: '<S37>/WpNav Matching'
// function [p_match,wp_idx,stage,t,d] = wpnavMatch( waypoints, wp_radius, wp_idx, stage, p )
//
void MatlabControllerClass::wpnavMatch_p(const real32_T waypoints_data[], const
  int32_T waypoints_size[2], real32_T wp_radius, int32_T *wp_idx, int32_T *stage,
  const real32_T p[3], real32_T p_match[3], real32_T *t, real32_T *d)
{
  int32_T num_wp;
  boolean_T is_last_line;
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

  // 'wpnavMatch:47' num_wp = size(waypoints,2);
  num_wp = waypoints_size[1];

  // 'wpnavMatch:49' is_cycle = true;
  // 'wpnavMatch:51' p_match = zeros(3,1,superiorfloat(waypoints));
  // 'wpnavMatch:52' t = zeros(1,1,superiorfloat(waypoints));
  // 'wpnavMatch:53' d = zeros(1,1,superiorfloat(waypoints));
  // 'wpnavMatch:62' if wp_idx > num_wp
  if (*wp_idx > waypoints_size[1]) {
    // 'wpnavMatch:63' wp_idx(:) = num_wp;
    *wp_idx = waypoints_size[1];
  }

  // 'wpnavMatch:65' if wp_idx < 1
  if (*wp_idx < 1) {
    // 'wpnavMatch:66' wp_idx(:) = 1;
    *wp_idx = 1;
  }

  // 'wpnavMatch:69' while true
  do {
    exitg1 = 0;

    // 'wpnavMatch:71' if wp_idx == 2
    // 'wpnavMatch:76' if wp_idx == num_wp
    if (*wp_idx == num_wp) {
      // 'wpnavMatch:77' is_last_line = true;
      is_last_line = true;
    } else {
      // 'wpnavMatch:78' else
      // 'wpnavMatch:79' is_last_line = false;
      is_last_line = false;
    }

    // 'wpnavMatch:83' if stage == 0
    guard1 = false;
    guard2 = false;
    if (*stage == 0) {
      // 'wpnavMatch:84' idx3 = fixIdx([wp_idx-2,wp_idx-1,wp_idx],num_wp);
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

      // 'wpnavMatch:147' for i = 1:length(idx)
      if (*wp_idx < -2147483646) {
        idx3_0 = MIN_int32_T;
      } else {
        idx3_0 = *wp_idx - 2;
      }

      // 'wpnavMatch:148' if idx(i) < 1
      if (idx3[0] < 1) {
        // 'wpnavMatch:149' idx(i) = N + idx(i);
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

      // 'wpnavMatch:148' if idx(i) < 1
      if (idx3[1] < 1) {
        // 'wpnavMatch:149' idx(i) = N + idx(i);
        tmp = (real_T)num_wp + (real_T)idx3[1];
        if (tmp >= -2.147483648E+9) {
          idx3_0 = (int32_T)tmp;
        } else {
          idx3_0 = MIN_int32_T;
        }
      }

      idx3[1] = idx3_0;
      idx3[2] = *wp_idx;

      // 'wpnavMatch:148' if idx(i) < 1
      if (*wp_idx < 1) {
        // 'wpnavMatch:149' idx(i) = N + idx(i);
        tmp = (real_T)num_wp + (real_T)*wp_idx;
        if (tmp >= -2.147483648E+9) {
          idx3[2] = (int32_T)tmp;
        } else {
          idx3[2] = MIN_int32_T;
        }
      }

      // 'wpnavMatch:85' circ_seg = wpnavCircSeg(waypoints(:,idx3),wp_radius);
      for (idx3_0 = 0; idx3_0 < 3; idx3_0++) {
        waypoints_tmp = (idx3[idx3_0] - 1) * 3;
        waypoints[3 * idx3_0] = waypoints_data[waypoints_tmp];
        waypoints[1 + 3 * idx3_0] = waypoints_data[waypoints_tmp + 1];
        waypoints[2 + 3 * idx3_0] = waypoints_data[waypoints_tmp + 2];
      }

      wpnavCircSeg(waypoints, wp_radius, &expl_temp, p2, circ_seg_n,
                   &circ_seg_angle, circ_seg_start, circ_seg_end, &expl_temp_0,
                   circ_seg_wp);

      // 'wpnavMatch:86' [p_match(:),t(:),d(:)] = wpnavMatchCircSeg(circ_seg,p); 
      wpnavMatchCircSeg(p2, circ_seg_n, circ_seg_angle, circ_seg_start,
                        circ_seg_wp, p, p_match, t, d);

      // 'wpnavMatch:90' if t > 1
      if (*t > 1.0F) {
        // 'wpnavMatch:91' stage(:) = 1;
        *stage = 1;
        guard2 = true;
      } else {
        // 'wpnavMatch:92' else
        exitg1 = 1;
      }
    } else {
      guard2 = true;
    }

    if (guard2) {
      // 'wpnavMatch:98' if stage == 1
      if (*stage == 1) {
        // 'wpnavMatch:99' if is_initial_line
        // 'wpnavMatch:101' else
        // 'wpnavMatch:102' idx3 = fixIdx([wp_idx-2,wp_idx-1,wp_idx],num_wp);
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

        // 'wpnavMatch:147' for i = 1:length(idx)
        if (*wp_idx < -2147483646) {
          idx3_0 = MIN_int32_T;
        } else {
          idx3_0 = *wp_idx - 2;
        }

        // 'wpnavMatch:148' if idx(i) < 1
        if (idx3[0] < 1) {
          // 'wpnavMatch:149' idx(i) = N + idx(i);
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

        // 'wpnavMatch:148' if idx(i) < 1
        if (idx3[1] < 1) {
          // 'wpnavMatch:149' idx(i) = N + idx(i);
          tmp = (real_T)num_wp + (real_T)idx3[1];
          if (tmp >= -2.147483648E+9) {
            idx3_0 = (int32_T)tmp;
          } else {
            idx3_0 = MIN_int32_T;
          }
        }

        idx3[1] = idx3_0;
        idx3[2] = *wp_idx;

        // 'wpnavMatch:148' if idx(i) < 1
        if (*wp_idx < 1) {
          // 'wpnavMatch:149' idx(i) = N + idx(i);
          tmp = (real_T)num_wp + (real_T)*wp_idx;
          if (tmp >= -2.147483648E+9) {
            idx3[2] = (int32_T)tmp;
          } else {
            idx3[2] = MIN_int32_T;
          }
        }

        // 'wpnavMatch:103' circ_seg_1 = wpnavCircSeg(waypoints(:,idx3),wp_radius); 
        for (idx3_0 = 0; idx3_0 < 3; idx3_0++) {
          waypoints_tmp = (idx3[idx3_0] - 1) * 3;
          waypoints[3 * idx3_0] = waypoints_data[waypoints_tmp];
          waypoints[1 + 3 * idx3_0] = waypoints_data[waypoints_tmp + 1];
          waypoints[2 + 3 * idx3_0] = waypoints_data[waypoints_tmp + 2];
        }

        wpnavCircSeg(waypoints, wp_radius, &expl_temp, p2, circ_seg_n,
                     &circ_seg_angle, circ_seg_start, circ_seg_end, &expl_temp_0,
                     circ_seg_wp);

        // 'wpnavMatch:104' p1 = circ_seg_1.end;
        // 'wpnavMatch:106' if is_last_line
        if (is_last_line) {
          // 'wpnavMatch:107' if is_cycle
          // 'wpnavMatch:108' idx2 = fixIdx([wp_idx-1,wp_idx],num_wp);
          if (*wp_idx < -2147483647) {
            idx2[0] = MIN_int32_T;
          } else {
            idx2[0] = *wp_idx - 1;
          }

          // 'wpnavMatch:147' for i = 1:length(idx)
          if (*wp_idx < -2147483647) {
            idx3_0 = MIN_int32_T;
          } else {
            idx3_0 = *wp_idx - 1;
          }

          // 'wpnavMatch:148' if idx(i) < 1
          if (idx2[0] < 1) {
            // 'wpnavMatch:149' idx(i) = N + idx(i);
            tmp = (real_T)num_wp + (real_T)idx2[0];
            if (tmp >= -2.147483648E+9) {
              idx3_0 = (int32_T)tmp;
            } else {
              idx3_0 = MIN_int32_T;
            }
          }

          idx2[0] = idx3_0;
          idx2[1] = *wp_idx;

          // 'wpnavMatch:148' if idx(i) < 1
          if (*wp_idx < 1) {
            // 'wpnavMatch:149' idx(i) = N + idx(i);
            tmp = (real_T)num_wp + (real_T)*wp_idx;
            if (tmp >= -2.147483648E+9) {
              idx2[1] = (int32_T)tmp;
            } else {
              idx2[1] = MIN_int32_T;
            }
          }

          // 'wpnavMatch:109' circ_seg_2 = wpnavCircSeg([waypoints(:,idx2),waypoints(:,1)],wp_radius); 
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

          // 'wpnavMatch:110' p2 = circ_seg_2.start;
        } else {
          // 'wpnavMatch:114' else
          // 'wpnavMatch:115' idx3 = fixIdx([wp_idx-1,wp_idx,wp_idx+1],num_wp);
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

          // 'wpnavMatch:147' for i = 1:length(idx)
          if (*wp_idx < -2147483647) {
            idx3_0 = MIN_int32_T;
          } else {
            idx3_0 = *wp_idx - 1;
          }

          // 'wpnavMatch:148' if idx(i) < 1
          if (idx3[0] < 1) {
            // 'wpnavMatch:149' idx(i) = N + idx(i);
            tmp = (real_T)num_wp + (real_T)idx3[0];
            if (tmp >= -2.147483648E+9) {
              idx3_0 = (int32_T)tmp;
            } else {
              idx3_0 = MIN_int32_T;
            }
          }

          idx3[0] = idx3_0;
          idx3[1] = *wp_idx;

          // 'wpnavMatch:148' if idx(i) < 1
          if (*wp_idx < 1) {
            // 'wpnavMatch:149' idx(i) = N + idx(i);
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

          // 'wpnavMatch:148' if idx(i) < 1
          if (idx3[2] < 1) {
            // 'wpnavMatch:149' idx(i) = N + idx(i);
            tmp = (real_T)num_wp + (real_T)idx3[2];
            if (tmp >= -2.147483648E+9) {
              idx3_0 = (int32_T)tmp;
            } else {
              idx3_0 = MIN_int32_T;
            }
          }

          idx3[2] = idx3_0;

          // 'wpnavMatch:116' circ_seg_2 = wpnavCircSeg(waypoints(:,idx3),wp_radius); 
          for (idx3_0 = 0; idx3_0 < 3; idx3_0++) {
            waypoints_tmp = (idx3[idx3_0] - 1) * 3;
            waypoints[3 * idx3_0] = waypoints_data[waypoints_tmp];
            waypoints[1 + 3 * idx3_0] = waypoints_data[waypoints_tmp + 1];
            waypoints[2 + 3 * idx3_0] = waypoints_data[waypoints_tmp + 2];
          }

          wpnavCircSeg(waypoints, wp_radius, &expl_temp, circ_seg_n,
                       circ_seg_start, &expl_temp_0, p2, circ_seg_wp,
                       &circ_seg_angle, expl_temp_1);

          // 'wpnavMatch:117' p2 = circ_seg_2.start;
        }

        // 'wpnavMatch:119' line_length = norm( p1 - p2, 2);
        // 'wpnavMatch:120' if line_length < 1
        circ_seg_n[0] = circ_seg_end[0] - p2[0];
        circ_seg_n[1] = circ_seg_end[1] - p2[1];
        circ_seg_n[2] = circ_seg_end[2] - p2[2];
        if (norm_c(circ_seg_n) < 1.0F) {
          // 'wpnavMatch:121' stage(:) = 0;
          *stage = 0;

          // 'wpnavMatch:122' wp_idx(:) = wp_idx + 1;
          if (*wp_idx > 2147483646) {
            *wp_idx = MAX_int32_T;
          } else {
            (*wp_idx)++;
          }

          guard1 = true;
        } else {
          // 'wpnavMatch:123' else
          // 'wpnavMatch:124' [p_match(:),t(:),d(:)] = wpnavMatchLine(p1,p2,p);
          wpnavMatchLine(circ_seg_end, p2, p, p_match, t, d);

          // 'wpnavMatch:125' if t > 1
          if (*t > 1.0F) {
            // 'wpnavMatch:126' stage(:) = 0;
            *stage = 0;

            // 'wpnavMatch:127' wp_idx(:) = wp_idx + 1;
            if (*wp_idx > 2147483646) {
              *wp_idx = MAX_int32_T;
            } else {
              (*wp_idx)++;
            }

            guard1 = true;
          } else {
            // 'wpnavMatch:128' else
            exitg1 = 1;
          }
        }
      } else {
        guard1 = true;
      }
    }

    if (guard1) {
      // 'wpnavMatch:133' if wp_idx > num_wp
      if (*wp_idx > num_wp) {
        // 'wpnavMatch:134' if is_cycle
        // 'wpnavMatch:135' wp_idx(:) = 1;
        *wp_idx = 1;
      }
    }
  } while (exitg1 == 0);
}

//
// Function for MATLAB Function: '<S37>/WpNav Matching'
// function [p_match,wp_idx,stage,t,d] = wpnavMatch( waypoints, wp_radius, wp_idx, stage, p )
//
void MatlabControllerClass::wpnavMatch(const real32_T waypoints[15], real32_T
  wp_radius, int32_T *wp_idx, int32_T *stage, const real32_T p[3], real32_T
  p_match[3], real32_T *t, real32_T *d)
{
  boolean_T is_last_line;
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

  // 'wpnavMatch:47' num_wp = size(waypoints,2);
  // 'wpnavMatch:49' is_cycle = true;
  // 'wpnavMatch:51' p_match = zeros(3,1,superiorfloat(waypoints));
  // 'wpnavMatch:52' t = zeros(1,1,superiorfloat(waypoints));
  // 'wpnavMatch:53' d = zeros(1,1,superiorfloat(waypoints));
  // 'wpnavMatch:62' if wp_idx > num_wp
  if (*wp_idx > 5) {
    // 'wpnavMatch:63' wp_idx(:) = num_wp;
    *wp_idx = 5;
  }

  // 'wpnavMatch:65' if wp_idx < 1
  if (*wp_idx < 1) {
    // 'wpnavMatch:66' wp_idx(:) = 1;
    *wp_idx = 1;
  }

  // 'wpnavMatch:69' while true
  do {
    exitg1 = 0;

    // 'wpnavMatch:71' if wp_idx == 2
    // 'wpnavMatch:76' if wp_idx == num_wp
    if (*wp_idx == 5) {
      // 'wpnavMatch:77' is_last_line = true;
      is_last_line = true;
    } else {
      // 'wpnavMatch:78' else
      // 'wpnavMatch:79' is_last_line = false;
      is_last_line = false;
    }

    // 'wpnavMatch:83' if stage == 0
    guard1 = false;
    guard2 = false;
    if (*stage == 0) {
      // 'wpnavMatch:84' idx3 = fixIdx([wp_idx-2,wp_idx-1,wp_idx],num_wp);
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

      // 'wpnavMatch:147' for i = 1:length(idx)
      if (*wp_idx < -2147483646) {
        idx3_0 = MIN_int32_T;
      } else {
        idx3_0 = *wp_idx - 2;
      }

      // 'wpnavMatch:148' if idx(i) < 1
      if (idx3[0] < 1) {
        // 'wpnavMatch:149' idx(i) = N + idx(i);
        idx3_0 = 5 + idx3[0];
      }

      idx3[0] = idx3_0;
      if (*wp_idx < -2147483647) {
        idx3_0 = MIN_int32_T;
      } else {
        idx3_0 = *wp_idx - 1;
      }

      // 'wpnavMatch:148' if idx(i) < 1
      if (idx3[1] < 1) {
        // 'wpnavMatch:149' idx(i) = N + idx(i);
        idx3_0 = 5 + idx3[1];
      }

      idx3[1] = idx3_0;
      idx3[2] = *wp_idx;

      // 'wpnavMatch:148' if idx(i) < 1
      if (*wp_idx < 1) {
        // 'wpnavMatch:149' idx(i) = N + idx(i);
        idx3[2] = 5 + *wp_idx;
      }

      // 'wpnavMatch:85' circ_seg = wpnavCircSeg(waypoints(:,idx3),wp_radius);
      for (idx3_0 = 0; idx3_0 < 3; idx3_0++) {
        waypoints_tmp = (idx3[idx3_0] - 1) * 3;
        waypoints_0[3 * idx3_0] = waypoints[waypoints_tmp];
        waypoints_0[1 + 3 * idx3_0] = waypoints[waypoints_tmp + 1];
        waypoints_0[2 + 3 * idx3_0] = waypoints[waypoints_tmp + 2];
      }

      wpnavCircSeg(waypoints_0, wp_radius, &expl_temp, p2, circ_seg_n,
                   &circ_seg_angle, circ_seg_start, circ_seg_end, &expl_temp_0,
                   circ_seg_wp);

      // 'wpnavMatch:86' [p_match(:),t(:),d(:)] = wpnavMatchCircSeg(circ_seg,p); 
      wpnavMatchCircSeg(p2, circ_seg_n, circ_seg_angle, circ_seg_start,
                        circ_seg_wp, p, p_match, t, d);

      // 'wpnavMatch:90' if t > 1
      if (*t > 1.0F) {
        // 'wpnavMatch:91' stage(:) = 1;
        *stage = 1;
        guard2 = true;
      } else {
        // 'wpnavMatch:92' else
        exitg1 = 1;
      }
    } else {
      guard2 = true;
    }

    if (guard2) {
      // 'wpnavMatch:98' if stage == 1
      if (*stage == 1) {
        // 'wpnavMatch:99' if is_initial_line
        // 'wpnavMatch:101' else
        // 'wpnavMatch:102' idx3 = fixIdx([wp_idx-2,wp_idx-1,wp_idx],num_wp);
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

        // 'wpnavMatch:147' for i = 1:length(idx)
        if (*wp_idx < -2147483646) {
          idx3_0 = MIN_int32_T;
        } else {
          idx3_0 = *wp_idx - 2;
        }

        // 'wpnavMatch:148' if idx(i) < 1
        if (idx3[0] < 1) {
          // 'wpnavMatch:149' idx(i) = N + idx(i);
          idx3_0 = 5 + idx3[0];
        }

        idx3[0] = idx3_0;
        if (*wp_idx < -2147483647) {
          idx3_0 = MIN_int32_T;
        } else {
          idx3_0 = *wp_idx - 1;
        }

        // 'wpnavMatch:148' if idx(i) < 1
        if (idx3[1] < 1) {
          // 'wpnavMatch:149' idx(i) = N + idx(i);
          idx3_0 = 5 + idx3[1];
        }

        idx3[1] = idx3_0;
        idx3[2] = *wp_idx;

        // 'wpnavMatch:148' if idx(i) < 1
        if (*wp_idx < 1) {
          // 'wpnavMatch:149' idx(i) = N + idx(i);
          idx3[2] = 5 + *wp_idx;
        }

        // 'wpnavMatch:103' circ_seg_1 = wpnavCircSeg(waypoints(:,idx3),wp_radius); 
        for (idx3_0 = 0; idx3_0 < 3; idx3_0++) {
          waypoints_tmp = (idx3[idx3_0] - 1) * 3;
          waypoints_0[3 * idx3_0] = waypoints[waypoints_tmp];
          waypoints_0[1 + 3 * idx3_0] = waypoints[waypoints_tmp + 1];
          waypoints_0[2 + 3 * idx3_0] = waypoints[waypoints_tmp + 2];
        }

        wpnavCircSeg(waypoints_0, wp_radius, &expl_temp, p2, circ_seg_n,
                     &circ_seg_angle, circ_seg_start, circ_seg_end, &expl_temp_0,
                     circ_seg_wp);

        // 'wpnavMatch:104' p1 = circ_seg_1.end;
        // 'wpnavMatch:106' if is_last_line
        if (is_last_line) {
          // 'wpnavMatch:107' if is_cycle
          // 'wpnavMatch:108' idx2 = fixIdx([wp_idx-1,wp_idx],num_wp);
          if (*wp_idx < -2147483647) {
            idx2[0] = MIN_int32_T;
          } else {
            idx2[0] = *wp_idx - 1;
          }

          // 'wpnavMatch:147' for i = 1:length(idx)
          if (*wp_idx < -2147483647) {
            idx3_0 = MIN_int32_T;
          } else {
            idx3_0 = *wp_idx - 1;
          }

          // 'wpnavMatch:148' if idx(i) < 1
          if (idx2[0] < 1) {
            // 'wpnavMatch:149' idx(i) = N + idx(i);
            idx3_0 = 5 + idx2[0];
          }

          idx2[0] = idx3_0;
          idx2[1] = *wp_idx;

          // 'wpnavMatch:148' if idx(i) < 1
          if (*wp_idx < 1) {
            // 'wpnavMatch:149' idx(i) = N + idx(i);
            idx2[1] = 5 + *wp_idx;
          }

          // 'wpnavMatch:109' circ_seg_2 = wpnavCircSeg([waypoints(:,idx2),waypoints(:,1)],wp_radius); 
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

          // 'wpnavMatch:110' p2 = circ_seg_2.start;
        } else {
          // 'wpnavMatch:114' else
          // 'wpnavMatch:115' idx3 = fixIdx([wp_idx-1,wp_idx,wp_idx+1],num_wp);
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

          // 'wpnavMatch:147' for i = 1:length(idx)
          if (*wp_idx < -2147483647) {
            idx3_0 = MIN_int32_T;
          } else {
            idx3_0 = *wp_idx - 1;
          }

          // 'wpnavMatch:148' if idx(i) < 1
          if (idx3[0] < 1) {
            // 'wpnavMatch:149' idx(i) = N + idx(i);
            idx3_0 = 5 + idx3[0];
          }

          idx3[0] = idx3_0;
          idx3[1] = *wp_idx;

          // 'wpnavMatch:148' if idx(i) < 1
          if (*wp_idx < 1) {
            // 'wpnavMatch:149' idx(i) = N + idx(i);
            idx3[1] = 5 + *wp_idx;
          }

          if (*wp_idx > 2147483646) {
            idx3_0 = MAX_int32_T;
          } else {
            idx3_0 = *wp_idx + 1;
          }

          // 'wpnavMatch:148' if idx(i) < 1
          if (idx3[2] < 1) {
            // 'wpnavMatch:149' idx(i) = N + idx(i);
            idx3_0 = 5 + idx3[2];
          }

          idx3[2] = idx3_0;

          // 'wpnavMatch:116' circ_seg_2 = wpnavCircSeg(waypoints(:,idx3),wp_radius); 
          for (idx3_0 = 0; idx3_0 < 3; idx3_0++) {
            waypoints_tmp = (idx3[idx3_0] - 1) * 3;
            waypoints_0[3 * idx3_0] = waypoints[waypoints_tmp];
            waypoints_0[1 + 3 * idx3_0] = waypoints[waypoints_tmp + 1];
            waypoints_0[2 + 3 * idx3_0] = waypoints[waypoints_tmp + 2];
          }

          wpnavCircSeg(waypoints_0, wp_radius, &expl_temp, circ_seg_n,
                       circ_seg_start, &expl_temp_0, p2, circ_seg_wp,
                       &circ_seg_angle, expl_temp_1);

          // 'wpnavMatch:117' p2 = circ_seg_2.start;
        }

        // 'wpnavMatch:119' line_length = norm( p1 - p2, 2);
        // 'wpnavMatch:120' if line_length < 1
        circ_seg_n[0] = circ_seg_end[0] - p2[0];
        circ_seg_n[1] = circ_seg_end[1] - p2[1];
        circ_seg_n[2] = circ_seg_end[2] - p2[2];
        if (norm_c(circ_seg_n) < 1.0F) {
          // 'wpnavMatch:121' stage(:) = 0;
          *stage = 0;

          // 'wpnavMatch:122' wp_idx(:) = wp_idx + 1;
          if (*wp_idx > 2147483646) {
            *wp_idx = MAX_int32_T;
          } else {
            (*wp_idx)++;
          }

          guard1 = true;
        } else {
          // 'wpnavMatch:123' else
          // 'wpnavMatch:124' [p_match(:),t(:),d(:)] = wpnavMatchLine(p1,p2,p);
          wpnavMatchLine(circ_seg_end, p2, p, p_match, t, d);

          // 'wpnavMatch:125' if t > 1
          if (*t > 1.0F) {
            // 'wpnavMatch:126' stage(:) = 0;
            *stage = 0;

            // 'wpnavMatch:127' wp_idx(:) = wp_idx + 1;
            if (*wp_idx > 2147483646) {
              *wp_idx = MAX_int32_T;
            } else {
              (*wp_idx)++;
            }

            guard1 = true;
          } else {
            // 'wpnavMatch:128' else
            exitg1 = 1;
          }
        }
      } else {
        guard1 = true;
      }
    }

    if (guard1) {
      // 'wpnavMatch:133' if wp_idx > num_wp
      if (*wp_idx > 5) {
        // 'wpnavMatch:134' if is_cycle
        // 'wpnavMatch:135' wp_idx(:) = 1;
        *wp_idx = 1;
      }
    }
  } while (exitg1 == 0);
}

// Function for MATLAB Function: '<S29>/Outer Loop INDI'
real32_T MatlabControllerClass::norm(const real32_T x[2])
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

// Function for MATLAB Function: '<S41>/Maneuver Load Alleviation'
void MatlabControllerClass::LSQFromQR(const real32_T A_data[], const int32_T
  A_size[2], const real32_T tau_data[], const int32_T jpvt_data[], real32_T B_3
  [13], int32_T rankA, real32_T Y_data[], int32_T *Y_size)
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
      for (loop_ub = b_j + 1; loop_ub + 1 < 14; loop_ub++) {
        wj += A_data[13 * b_j + loop_ub] * B_3[loop_ub];
      }

      wj *= tau_data[b_j];
      if (wj != 0.0F) {
        B_3[b_j] -= wj;
        for (loop_ub = b_j + 1; loop_ub + 1 < 14; loop_ub++) {
          B_3[loop_ub] -= A_data[13 * b_j + loop_ub] * wj;
        }
      }
    }
  }

  for (loop_ub = 0; loop_ub < rankA; loop_ub++) {
    Y_data[jpvt_data[loop_ub] - 1] = B_3[loop_ub];
  }

  for (loop_ub = rankA - 1; loop_ub + 1 > 0; loop_ub--) {
    Y_data[jpvt_data[loop_ub] - 1] /= A_data[13 * loop_ub + loop_ub];
    for (b_j = 0; b_j < loop_ub; b_j++) {
      Y_data[jpvt_data[b_j] - 1] -= A_data[13 * loop_ub + b_j] *
        Y_data[jpvt_data[loop_ub] - 1];
    }
  }
}

// Function for MATLAB Function: '<S41>/Maneuver Load Alleviation'
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

// Function for MATLAB Function: '<S41>/Maneuver Load Alleviation'
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
      coltop = lastc * 13 + ic0;
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
      jy = 13 * lastc + ic0;
      for (iac = ic0; iac <= jy; iac += 13) {
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
        coltop += 13;
      }
    }
  }
}

// Function for MATLAB Function: '<S41>/Maneuver Load Alleviation'
void MatlabControllerClass::qrsolve(const real32_T A_data[], const int32_T
  A_size[2], const real32_T B_1[13], real32_T Y_data[], int32_T *Y_size)
{
  real32_T b_A_data[117];
  real32_T tau_data[9];
  int32_T jpvt_data[9];
  int32_T n;
  real32_T work_data[9];
  real32_T vn1_data[9];
  real32_T vn2_data[9];
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
  real32_T B_2[13];
  int32_T b_A_size[2];
  int8_T c_idx_0;
  b_A_size[0] = 13;
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
      for (nmi = b_n; nmi <= b_n + 12; nmi++) {
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
      b_n += 13;
    }

    for (b_n = 0; b_n < n; b_n++) {
      yk = b_n * 13 + b_n;
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
        b_ix = 13 * ix;
        iy = 13 * b_n;
        for (d_k = 0; d_k < 13; d_k++) {
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
      s = xnrm2(12 - b_n, b_A_data, yk + 2);
      if (s != 0.0F) {
        s = rt_hypotf(b_A_data[yk], s);
        if (b_A_data[yk] >= 0.0F) {
          s = -s;
        }

        if (std::abs(s) < 9.86076132E-32F) {
          ix = -1;
          b_ix = (yk - b_n) + 13;
          do {
            ix++;
            for (iy = yk + 1; iy < b_ix; iy++) {
              b_A_data[iy] *= 1.01412048E+31F;
            }

            s *= 1.01412048E+31F;
            smax *= 1.01412048E+31F;
          } while (std::abs(s) < 9.86076132E-32F);

          s = rt_hypotf(smax, xnrm2(12 - b_n, b_A_data, yk + 2));
          if (smax >= 0.0F) {
            s = -s;
          }

          tau_data[b_n] = (s - smax) / s;
          smax = 1.0F / (smax - s);
          b_ix = (yk - b_n) + 13;
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
          ix = (yk - b_n) + 13;
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
        xzlarf(13 - b_n, nmi - 1, yk + 1, tau_data[b_n], b_A_data, (b_n + (b_n +
                 1) * 13) + 1, work_data);
        b_A_data[yk] = smax;
      }

      for (yk = b_n + 1; yk < n; yk++) {
        if (vn1_data[yk] != 0.0F) {
          nmi = 13 * yk + b_n;
          smax = std::abs(b_A_data[nmi]) / vn1_data[yk];
          smax = 1.0F - smax * smax;
          if (smax < 0.0F) {
            smax = 0.0F;
          }

          s = vn1_data[yk] / vn2_data[yk];
          s = s * s * smax;
          if (s <= 0.000345266977F) {
            vn1_data[yk] = xnrm2(12 - b_n, b_A_data, nmi + 2);
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
    while ((n < b_A_size[1]) && (std::abs(b_A_data[13 * n + n]) > 1.54972076E-5F
            * std::abs(b_A_data[0]))) {
      n++;
    }
  }

  for (b_ix = 0; b_ix < 13; b_ix++) {
    B_2[b_ix] = B_1[b_ix];
  }

  LSQFromQR(b_A_data, b_A_size, tau_data, jpvt_data, B_2, n, Y_data, Y_size);
}

// Function for MATLAB Function: '<S41>/Maneuver Load Alleviation'
void MatlabControllerClass::mldivide(const real32_T A_data[], const int32_T
  A_size[2], const real32_T B_0[13], real32_T Y_data[], int32_T *Y_size)
{
  if (A_size[1] == 0) {
    *Y_size = 0;
  } else {
    qrsolve(A_data, A_size, B_0, Y_data, Y_size);
  }
}

// Function for MATLAB Function: '<S41>/Maneuver Load Alleviation'
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

//
// Function for MATLAB Function: '<S41>/Maneuver Load Alleviation'
// function [u,W,iter] = wls_alloc(B,v,umin,umax,Wv,Wu,ud,gam,u,W,imax)
//
real32_T MatlabControllerClass::wls_alloc(const real32_T B_4[36], const real32_T
  v[4], const real32_T umin[9], const real32_T umax[9], const real32_T Wv[16],
  const real32_T Wu[81], const real32_T ud[9], real32_T gam, real32_T u[9],
  real32_T W[9], real32_T imax)
{
  real32_T iter;
  real32_T gam_sq;
  real32_T A[117];
  real32_T d[13];
  boolean_T i_free[9];
  real32_T A_free_data[117];
  real32_T p_free_data[9];
  real_T p[9];
  real32_T u_opt[9];
  real_T dist[9];
  real_T b_data[9];
  int8_T e_data[9];
  int8_T f_data[9];
  int8_T g_data[9];
  int8_T h_data[9];
  int32_T aoffset;
  int32_T b_aoffset;
  real32_T A_tmp[16];
  int32_T i;
  boolean_T u_opt_data[9];
  real32_T A_tmp_0[36];
  real32_T A_tmp_1[4];
  real32_T A_tmp_2[13];
  real32_T A_0[13];
  boolean_T u_opt_0[9];
  int32_T A_free_size[2];
  real_T p_0;
  boolean_T x;
  int32_T f_size_idx_0;
  int32_T A_tmp_tmp;
  int32_T A_tmp_tmp_0;
  real_T tmp;
  real32_T tmp_0;
  boolean_T exitg1;
  boolean_T exitg2;

  // 'wls_alloc:42' m = length(umin);
  // 'wls_alloc:45' if nargin < 11
  // 'wls_alloc:55' gam_sq = sqrt(gam);
  gam_sq = std::sqrt(gam);

  // 'wls_alloc:56' A = [gam_sq*Wv*B ; Wu];
  for (b_aoffset = 0; b_aoffset < 16; b_aoffset++) {
    A_tmp[b_aoffset] = gam_sq * Wv[b_aoffset];
  }

  for (b_aoffset = 0; b_aoffset < 9; b_aoffset++) {
    for (f_size_idx_0 = 0; f_size_idx_0 < 4; f_size_idx_0++) {
      A_tmp_tmp = b_aoffset << 2;
      A_tmp_tmp_0 = f_size_idx_0 + A_tmp_tmp;
      A_tmp_0[A_tmp_tmp_0] = 0.0F;
      i = A_tmp_tmp + f_size_idx_0;
      A_tmp_0[A_tmp_tmp_0] = A_tmp_0[i] + B_4[A_tmp_tmp] * A_tmp[f_size_idx_0];
      A_tmp_0[A_tmp_tmp_0] = B_4[A_tmp_tmp + 1] * A_tmp[f_size_idx_0 + 4] +
        A_tmp_0[i];
      A_tmp_0[A_tmp_tmp_0] = B_4[A_tmp_tmp + 2] * A_tmp[f_size_idx_0 + 8] +
        A_tmp_0[i];
      A_tmp_0[A_tmp_tmp_0] = B_4[A_tmp_tmp + 3] * A_tmp[f_size_idx_0 + 12] +
        A_tmp_0[i];
    }
  }

  for (b_aoffset = 0; b_aoffset < 9; b_aoffset++) {
    A_tmp_tmp = b_aoffset << 2;
    A[13 * b_aoffset] = A_tmp_0[A_tmp_tmp];
    A[1 + 13 * b_aoffset] = A_tmp_0[A_tmp_tmp + 1];
    A[2 + 13 * b_aoffset] = A_tmp_0[A_tmp_tmp + 2];
    A[3 + 13 * b_aoffset] = A_tmp_0[A_tmp_tmp + 3];
    for (f_size_idx_0 = 0; f_size_idx_0 < 9; f_size_idx_0++) {
      A[(f_size_idx_0 + 13 * b_aoffset) + 4] = Wu[9 * b_aoffset + f_size_idx_0];
    }
  }

  // 'wls_alloc:57' b = [gam_sq*Wv*v ; Wu*ud];
  // 'wls_alloc:60' d = b - A*u;
  for (b_aoffset = 0; b_aoffset < 4; b_aoffset++) {
    gam_sq = A_tmp[b_aoffset + 12] * v[3] + (A_tmp[b_aoffset + 8] * v[2] +
      (A_tmp[b_aoffset + 4] * v[1] + A_tmp[b_aoffset] * v[0]));
    A_tmp_1[b_aoffset] = gam_sq;
  }

  A_tmp_2[0] = A_tmp_1[0];
  A_tmp_2[1] = A_tmp_1[1];
  A_tmp_2[2] = A_tmp_1[2];
  A_tmp_2[3] = A_tmp_1[3];
  for (b_aoffset = 0; b_aoffset < 9; b_aoffset++) {
    u_opt[b_aoffset] = 0.0F;
    for (f_size_idx_0 = 0; f_size_idx_0 < 9; f_size_idx_0++) {
      u_opt[b_aoffset] += Wu[9 * f_size_idx_0 + b_aoffset] * ud[f_size_idx_0];
    }

    A_tmp_2[b_aoffset + 4] = u_opt[b_aoffset];
  }

  for (b_aoffset = 0; b_aoffset < 13; b_aoffset++) {
    A_0[b_aoffset] = 0.0F;
    for (f_size_idx_0 = 0; f_size_idx_0 < 9; f_size_idx_0++) {
      A_0[b_aoffset] += A[13 * f_size_idx_0 + b_aoffset] * u[f_size_idx_0];
    }

    d[b_aoffset] = A_tmp_2[b_aoffset] - A_0[b_aoffset];
  }

  // 'wls_alloc:62' i_free = W==0;
  for (i = 0; i < 9; i++) {
    i_free[i] = (W[i] == 0.0F);
  }

  // 'wls_alloc:66' for iter = 1:imax
  iter = 1.0F;
  A_tmp_tmp = 0;
  exitg1 = false;
  while ((!exitg1) && (A_tmp_tmp <= (int32_T)imax - 1)) {
    iter = 1.0F + (real32_T)A_tmp_tmp;

    // 'wls_alloc:72' A_free = A(:,i_free);
    i = 0;
    for (b_aoffset = 0; b_aoffset < 9; b_aoffset++) {
      if (i_free[b_aoffset]) {
        i++;
      }
    }

    A_tmp_tmp_0 = i;
    i = 0;
    for (b_aoffset = 0; b_aoffset < 9; b_aoffset++) {
      if (i_free[b_aoffset]) {
        e_data[i] = (int8_T)(b_aoffset + 1);
        i++;
      }
    }

    A_free_size[0] = 13;
    A_free_size[1] = A_tmp_tmp_0;
    for (b_aoffset = 0; b_aoffset < A_tmp_tmp_0; b_aoffset++) {
      for (f_size_idx_0 = 0; f_size_idx_0 < 13; f_size_idx_0++) {
        A_free_data[f_size_idx_0 + 13 * b_aoffset] = A[(e_data[b_aoffset] - 1) *
          13 + f_size_idx_0];
      }
    }

    // 'wls_alloc:74' p_free = A_free\d;
    mldivide(A_free_data, A_free_size, d, p_free_data, &aoffset);

    // 'wls_alloc:76' p = zeros(m,1);
    // 'wls_alloc:78' p(i_free) = p_free;
    i = 0;

    // 'wls_alloc:84' u_opt = u + p;
    for (b_aoffset = 0; b_aoffset < 9; b_aoffset++) {
      p_0 = 0.0;
      if (i_free[b_aoffset]) {
        p_0 = p_free_data[i];
        i++;
      }

      u_opt[b_aoffset] = u[b_aoffset] + (real32_T)p_0;
      p[b_aoffset] = p_0;
    }

    // 'wls_alloc:85' infeasible = (u_opt < umin) | (u_opt > umax);
    // 'wls_alloc:87' if ~any(infeasible(i_free))
    i = 0;
    for (b_aoffset = 0; b_aoffset < 9; b_aoffset++) {
      if (i_free[b_aoffset]) {
        i++;
      }
    }

    f_size_idx_0 = i;
    i = 0;
    for (b_aoffset = 0; b_aoffset < 9; b_aoffset++) {
      if (i_free[b_aoffset]) {
        f_data[i] = (int8_T)(b_aoffset + 1);
        i++;
      }

      u_opt_0[b_aoffset] = ((u_opt[b_aoffset] < umin[b_aoffset]) ||
                            (u_opt[b_aoffset] > umax[b_aoffset]));
    }

    for (b_aoffset = 0; b_aoffset < f_size_idx_0; b_aoffset++) {
      u_opt_data[b_aoffset] = u_opt_0[f_data[b_aoffset] - 1];
    }

    if (!any(u_opt_data, &f_size_idx_0)) {
      // 'wls_alloc:94' u = u_opt;
      for (i = 0; i < 9; i++) {
        u[i] = u_opt[i];
      }

      // 'wls_alloc:95' d = d - A_free*p_free;
      if (A_tmp_tmp_0 == 1) {
        for (b_aoffset = 0; b_aoffset < 13; b_aoffset++) {
          A_tmp_2[b_aoffset] = 0.0F;
          for (f_size_idx_0 = 0; f_size_idx_0 < A_tmp_tmp_0; f_size_idx_0++) {
            A_tmp_2[b_aoffset] += A_free_data[13 * f_size_idx_0 + b_aoffset] *
              p_free_data[f_size_idx_0];
          }
        }
      } else if (aoffset == 1) {
        for (b_aoffset = 0; b_aoffset < 13; b_aoffset++) {
          A_tmp_2[b_aoffset] = 0.0F;
          for (f_size_idx_0 = 0; f_size_idx_0 < A_tmp_tmp_0; f_size_idx_0++) {
            A_tmp_2[b_aoffset] += A_free_data[13 * f_size_idx_0 + b_aoffset] *
              p_free_data[f_size_idx_0];
          }
        }
      } else {
        for (i = 0; i < 13; i++) {
          A_tmp_2[i] = 0.0F;
        }

        for (i = 0; i < A_tmp_tmp_0; i++) {
          b_aoffset = i * 13;
          for (f_size_idx_0 = 0; f_size_idx_0 < 13; f_size_idx_0++) {
            aoffset = b_aoffset + f_size_idx_0;
            A_tmp_2[f_size_idx_0] += A[(e_data[aoffset / 13] - 1) * 13 + aoffset
              % 13] * p_free_data[i];
          }
        }
      }

      for (b_aoffset = 0; b_aoffset < 13; b_aoffset++) {
        d[b_aoffset] -= A_tmp_2[b_aoffset];
      }

      // 'wls_alloc:97' lambda = W.*(A'*d);
      // 'wls_alloc:99' if lambda >= -eps
      for (i = 0; i < 9; i++) {
        p_free_data[i] = 0.0F;
        for (b_aoffset = 0; b_aoffset < 13; b_aoffset++) {
          p_free_data[i] += A[13 * i + b_aoffset] * d[b_aoffset];
        }

        gam_sq = W[i] * p_free_data[i];
        u_opt_0[i] = (gam_sq >= -2.22044605E-16F);
        u_opt[i] = gam_sq;
      }

      x = true;
      i = 0;
      exitg2 = false;
      while ((!exitg2) && (i < 9)) {
        if (!u_opt_0[i]) {
          x = false;
          exitg2 = true;
        } else {
          i++;
        }
      }

      if (x) {
        exitg1 = true;
      } else {
        // 'wls_alloc:112' [lambda_neg,i_neg] = min(lambda);
        gam_sq = u_opt[0];
        i = 0;
        for (b_aoffset = 0; b_aoffset < 8; b_aoffset++) {
          tmp_0 = u_opt[b_aoffset + 1];
          if (gam_sq > tmp_0) {
            gam_sq = tmp_0;
            i = b_aoffset + 1;
          }
        }

        // 'wls_alloc:113' W(i_neg) = 0;
        W[i] = 0.0F;

        // 'wls_alloc:114' i_free(i_neg) = 1;
        i_free[i] = true;
        A_tmp_tmp++;
      }
    } else {
      // 'wls_alloc:116' else
      // 'wls_alloc:124' dist = ones(m,1);
      // 'wls_alloc:125' i_min = i_free & p<0;
      // 'wls_alloc:126' i_max = i_free & p>0;
      // 'wls_alloc:128' dist(i_min) = (umin(i_min) - u(i_min)) ./ p(i_min);
      i = 0;
      for (b_aoffset = 0; b_aoffset < 9; b_aoffset++) {
        dist[b_aoffset] = 1.0;
        x = (p[b_aoffset] < 0.0);
        u_opt_data[b_aoffset] = (p[b_aoffset] > 0.0);
        if (i_free[b_aoffset] && x) {
          i++;
        }

        u_opt_0[b_aoffset] = x;
      }

      f_size_idx_0 = i;
      i = 0;
      for (b_aoffset = 0; b_aoffset < 9; b_aoffset++) {
        if (i_free[b_aoffset] && u_opt_0[b_aoffset]) {
          g_data[i] = (int8_T)(b_aoffset + 1);
          i++;
        }
      }

      for (b_aoffset = 0; b_aoffset < f_size_idx_0; b_aoffset++) {
        i = g_data[b_aoffset] - 1;
        b_data[b_aoffset] = (umin[i] - u[i]) / (real32_T)p[i];
      }

      i = 0;
      for (b_aoffset = 0; b_aoffset < 9; b_aoffset++) {
        if (i_free[b_aoffset] && u_opt_0[b_aoffset]) {
          dist[b_aoffset] = b_data[i];
          i++;
        }
      }

      // 'wls_alloc:129' dist(i_max) = (umax(i_max) - u(i_max)) ./ p(i_max);
      i = 0;
      for (b_aoffset = 0; b_aoffset < 9; b_aoffset++) {
        if (i_free[b_aoffset] && u_opt_data[b_aoffset]) {
          i++;
        }
      }

      f_size_idx_0 = i;
      i = 0;
      for (b_aoffset = 0; b_aoffset < 9; b_aoffset++) {
        if (i_free[b_aoffset] && u_opt_data[b_aoffset]) {
          h_data[i] = (int8_T)(b_aoffset + 1);
          i++;
        }
      }

      for (b_aoffset = 0; b_aoffset < f_size_idx_0; b_aoffset++) {
        i = h_data[b_aoffset] - 1;
        b_data[b_aoffset] = (umax[i] - u[i]) / (real32_T)p[i];
      }

      i = 0;
      for (b_aoffset = 0; b_aoffset < 9; b_aoffset++) {
        if (i_free[b_aoffset] && u_opt_data[b_aoffset]) {
          dist[b_aoffset] = b_data[i];
          i++;
        }
      }

      // 'wls_alloc:132' [alpha,i_alpha] = min(dist);
      p_0 = dist[0];
      i = 0;
      for (b_aoffset = 0; b_aoffset < 8; b_aoffset++) {
        tmp = dist[b_aoffset + 1];
        if (p_0 > tmp) {
          p_0 = tmp;
          i = b_aoffset + 1;
        }
      }

      // 'wls_alloc:134' u = u + alpha*p;
      for (b_aoffset = 0; b_aoffset < 9; b_aoffset++) {
        u[b_aoffset] += (real32_T)(p_0 * p[b_aoffset]);
      }

      // 'wls_alloc:135' d = d - A_free*alpha*p_free;
      f_size_idx_0 = 13 * A_tmp_tmp_0 - 1;
      for (b_aoffset = 0; b_aoffset <= f_size_idx_0; b_aoffset++) {
        A_free_data[b_aoffset] *= (real32_T)p_0;
      }

      if (A_tmp_tmp_0 == 1) {
        for (b_aoffset = 0; b_aoffset < 13; b_aoffset++) {
          A_tmp_2[b_aoffset] = 0.0F;
          for (f_size_idx_0 = 0; f_size_idx_0 < 1; f_size_idx_0++) {
            A_tmp_2[b_aoffset] += A_free_data[b_aoffset] * p_free_data[0];
          }
        }
      } else if (aoffset == 1) {
        for (b_aoffset = 0; b_aoffset < 13; b_aoffset++) {
          A_tmp_2[b_aoffset] = 0.0F;
          for (f_size_idx_0 = 0; f_size_idx_0 < A_tmp_tmp_0; f_size_idx_0++) {
            A_tmp_2[b_aoffset] += A_free_data[13 * f_size_idx_0 + b_aoffset] *
              p_free_data[f_size_idx_0];
          }
        }
      } else {
        for (b_aoffset = 0; b_aoffset < 13; b_aoffset++) {
          A_tmp_2[b_aoffset] = 0.0F;
        }

        for (f_size_idx_0 = 0; f_size_idx_0 < A_tmp_tmp_0; f_size_idx_0++) {
          aoffset = f_size_idx_0 * 13;
          for (b_aoffset = 0; b_aoffset < 13; b_aoffset++) {
            A_tmp_2[b_aoffset] += A_free_data[aoffset + b_aoffset] *
              p_free_data[f_size_idx_0];
          }
        }
      }

      for (b_aoffset = 0; b_aoffset < 13; b_aoffset++) {
        d[b_aoffset] -= A_tmp_2[b_aoffset];
      }

      // 'wls_alloc:138' W(i_alpha) = sign(p(i_alpha));
      if (p[i] < 0.0) {
        W[i] = -1.0F;
      } else if (p[i] > 0.0) {
        W[i] = 1.0F;
      } else {
        W[i] = (real32_T)p[i];
      }

      // 'wls_alloc:139' i_free(i_alpha) = 0;
      i_free[i] = false;
      A_tmp_tmp++;
    }
  }

  return iter;
}

// Function for MATLAB Function: '<S103>/caIndiWls'
void MatlabControllerClass::LSQFromQR_e(const real32_T A_data[], const int32_T
  A_size[2], const real32_T tau_data[], const int32_T jpvt_data[], real32_T B_8
  [14], int32_T rankA, real32_T Y_data[], int32_T *Y_size)
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
      wj = B_8[b_j];
      for (loop_ub = b_j + 1; loop_ub + 1 < 15; loop_ub++) {
        wj += A_data[14 * b_j + loop_ub] * B_8[loop_ub];
      }

      wj *= tau_data[b_j];
      if (wj != 0.0F) {
        B_8[b_j] -= wj;
        for (loop_ub = b_j + 1; loop_ub + 1 < 15; loop_ub++) {
          B_8[loop_ub] -= A_data[14 * b_j + loop_ub] * wj;
        }
      }
    }
  }

  for (loop_ub = 0; loop_ub < rankA; loop_ub++) {
    Y_data[jpvt_data[loop_ub] - 1] = B_8[loop_ub];
  }

  for (loop_ub = rankA - 1; loop_ub + 1 > 0; loop_ub--) {
    Y_data[jpvt_data[loop_ub] - 1] /= A_data[14 * loop_ub + loop_ub];
    for (b_j = 0; b_j < loop_ub; b_j++) {
      Y_data[jpvt_data[b_j] - 1] -= A_data[14 * loop_ub + b_j] *
        Y_data[jpvt_data[loop_ub] - 1];
    }
  }
}

// Function for MATLAB Function: '<S103>/caIndiWls'
void MatlabControllerClass::xzlarf_h(int32_T m, int32_T n, int32_T iv0, real32_T
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
      coltop = lastc * 14 + ic0;
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
      jy = 14 * lastc + ic0;
      for (iac = ic0; iac <= jy; iac += 14) {
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
        coltop += 14;
      }
    }
  }
}

// Function for MATLAB Function: '<S103>/caIndiWls'
void MatlabControllerClass::qrsolve_a(const real32_T A_data[], const int32_T
  A_size[2], const real32_T B_6[14], real32_T Y_data[], int32_T *Y_size)
{
  real32_T b_A_data[140];
  real32_T tau_data[10];
  int32_T jpvt_data[10];
  int32_T n;
  real32_T work_data[10];
  real32_T vn1_data[10];
  real32_T vn2_data[10];
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
  real32_T B_7[14];
  int32_T b_A_size[2];
  int8_T c_idx_0;
  b_A_size[0] = 14;
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
      for (nmi = b_n; nmi <= b_n + 13; nmi++) {
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
      b_n += 14;
    }

    for (b_n = 0; b_n < n; b_n++) {
      yk = b_n * 14 + b_n;
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
        b_ix = 14 * ix;
        iy = 14 * b_n;
        for (d_k = 0; d_k < 14; d_k++) {
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
      s = xnrm2(13 - b_n, b_A_data, yk + 2);
      if (s != 0.0F) {
        s = rt_hypotf(b_A_data[yk], s);
        if (b_A_data[yk] >= 0.0F) {
          s = -s;
        }

        if (std::abs(s) < 9.86076132E-32F) {
          ix = -1;
          b_ix = (yk - b_n) + 14;
          do {
            ix++;
            for (iy = yk + 1; iy < b_ix; iy++) {
              b_A_data[iy] *= 1.01412048E+31F;
            }

            s *= 1.01412048E+31F;
            smax *= 1.01412048E+31F;
          } while (std::abs(s) < 9.86076132E-32F);

          s = rt_hypotf(smax, xnrm2(13 - b_n, b_A_data, yk + 2));
          if (smax >= 0.0F) {
            s = -s;
          }

          tau_data[b_n] = (s - smax) / s;
          smax = 1.0F / (smax - s);
          b_ix = (yk - b_n) + 14;
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
          ix = (yk - b_n) + 14;
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
        xzlarf_h(14 - b_n, nmi - 1, yk + 1, tau_data[b_n], b_A_data, (b_n + (b_n
                   + 1) * 14) + 1, work_data);
        b_A_data[yk] = smax;
      }

      for (yk = b_n + 1; yk < n; yk++) {
        if (vn1_data[yk] != 0.0F) {
          nmi = 14 * yk + b_n;
          smax = std::abs(b_A_data[nmi]) / vn1_data[yk];
          smax = 1.0F - smax * smax;
          if (smax < 0.0F) {
            smax = 0.0F;
          }

          s = vn1_data[yk] / vn2_data[yk];
          s = s * s * smax;
          if (s <= 0.000345266977F) {
            vn1_data[yk] = xnrm2(13 - b_n, b_A_data, nmi + 2);
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
    while ((n < b_A_size[1]) && (std::abs(b_A_data[14 * n + n]) > 1.66893E-5F *
            std::abs(b_A_data[0]))) {
      n++;
    }
  }

  for (b_ix = 0; b_ix < 14; b_ix++) {
    B_7[b_ix] = B_6[b_ix];
  }

  LSQFromQR_e(b_A_data, b_A_size, tau_data, jpvt_data, B_7, n, Y_data, Y_size);
}

// Function for MATLAB Function: '<S103>/caIndiWls'
void MatlabControllerClass::mldivide_k(const real32_T A_data[], const int32_T
  A_size[2], const real32_T B_5[14], real32_T Y_data[], int32_T *Y_size)
{
  if (A_size[1] == 0) {
    *Y_size = 0;
  } else {
    qrsolve_a(A_data, A_size, B_5, Y_data, Y_size);
  }
}

//
// Function for MATLAB Function: '<S103>/caIndiWls'
// function [u,W,iter] = wls_alloc(B,v,umin,umax,Wv,Wu,ud,gam,u,W,imax)
//
real32_T MatlabControllerClass::wls_alloc_c(const real32_T B_9[40], const
  real32_T v[4], const real32_T umin[10], const real32_T umax[10], const
  real32_T Wv[16], const real32_T Wu[100], const real32_T ud[10], real32_T gam,
  real32_T u[10], real32_T W[10], real32_T imax)
{
  real32_T iter;
  real32_T gam_sq;
  real32_T A[140];
  real32_T d[14];
  boolean_T i_free[10];
  real32_T A_free_data[140];
  real32_T p_free_data[10];
  real_T p[10];
  real32_T u_opt[10];
  real_T dist[10];
  real_T b_data[10];
  int8_T e_data[10];
  int8_T f_data[10];
  int8_T g_data[10];
  int8_T h_data[10];
  int32_T aoffset;
  int32_T b_aoffset;
  real32_T A_tmp[16];
  int32_T i;
  boolean_T u_opt_data[10];
  real32_T A_tmp_0[40];
  real32_T A_tmp_1[4];
  real32_T A_tmp_2[14];
  real32_T A_0[14];
  boolean_T u_opt_0[10];
  int32_T A_free_size[2];
  real_T p_0;
  boolean_T x;
  int32_T f_size_idx_0;
  int32_T A_tmp_tmp;
  int32_T A_tmp_tmp_0;
  real_T tmp;
  real32_T tmp_0;
  boolean_T exitg1;
  boolean_T exitg2;

  // 'wls_alloc:42' m = length(umin);
  // 'wls_alloc:45' if nargin < 11
  // 'wls_alloc:55' gam_sq = sqrt(gam);
  gam_sq = std::sqrt(gam);

  // 'wls_alloc:56' A = [gam_sq*Wv*B ; Wu];
  for (b_aoffset = 0; b_aoffset < 16; b_aoffset++) {
    A_tmp[b_aoffset] = gam_sq * Wv[b_aoffset];
  }

  for (b_aoffset = 0; b_aoffset < 10; b_aoffset++) {
    for (f_size_idx_0 = 0; f_size_idx_0 < 4; f_size_idx_0++) {
      A_tmp_tmp = b_aoffset << 2;
      A_tmp_tmp_0 = f_size_idx_0 + A_tmp_tmp;
      A_tmp_0[A_tmp_tmp_0] = 0.0F;
      i = A_tmp_tmp + f_size_idx_0;
      A_tmp_0[A_tmp_tmp_0] = A_tmp_0[i] + B_9[A_tmp_tmp] * A_tmp[f_size_idx_0];
      A_tmp_0[A_tmp_tmp_0] = B_9[A_tmp_tmp + 1] * A_tmp[f_size_idx_0 + 4] +
        A_tmp_0[i];
      A_tmp_0[A_tmp_tmp_0] = B_9[A_tmp_tmp + 2] * A_tmp[f_size_idx_0 + 8] +
        A_tmp_0[i];
      A_tmp_0[A_tmp_tmp_0] = B_9[A_tmp_tmp + 3] * A_tmp[f_size_idx_0 + 12] +
        A_tmp_0[i];
    }
  }

  for (b_aoffset = 0; b_aoffset < 10; b_aoffset++) {
    A_tmp_tmp = b_aoffset << 2;
    A[14 * b_aoffset] = A_tmp_0[A_tmp_tmp];
    A[1 + 14 * b_aoffset] = A_tmp_0[A_tmp_tmp + 1];
    A[2 + 14 * b_aoffset] = A_tmp_0[A_tmp_tmp + 2];
    A[3 + 14 * b_aoffset] = A_tmp_0[A_tmp_tmp + 3];
    for (f_size_idx_0 = 0; f_size_idx_0 < 10; f_size_idx_0++) {
      A[(f_size_idx_0 + 14 * b_aoffset) + 4] = Wu[10 * b_aoffset + f_size_idx_0];
    }
  }

  // 'wls_alloc:57' b = [gam_sq*Wv*v ; Wu*ud];
  // 'wls_alloc:60' d = b - A*u;
  for (b_aoffset = 0; b_aoffset < 4; b_aoffset++) {
    gam_sq = A_tmp[b_aoffset + 12] * v[3] + (A_tmp[b_aoffset + 8] * v[2] +
      (A_tmp[b_aoffset + 4] * v[1] + A_tmp[b_aoffset] * v[0]));
    A_tmp_1[b_aoffset] = gam_sq;
  }

  A_tmp_2[0] = A_tmp_1[0];
  A_tmp_2[1] = A_tmp_1[1];
  A_tmp_2[2] = A_tmp_1[2];
  A_tmp_2[3] = A_tmp_1[3];
  for (b_aoffset = 0; b_aoffset < 10; b_aoffset++) {
    u_opt[b_aoffset] = 0.0F;
    for (f_size_idx_0 = 0; f_size_idx_0 < 10; f_size_idx_0++) {
      u_opt[b_aoffset] += Wu[10 * f_size_idx_0 + b_aoffset] * ud[f_size_idx_0];
    }

    A_tmp_2[b_aoffset + 4] = u_opt[b_aoffset];
  }

  for (b_aoffset = 0; b_aoffset < 14; b_aoffset++) {
    A_0[b_aoffset] = 0.0F;
    for (f_size_idx_0 = 0; f_size_idx_0 < 10; f_size_idx_0++) {
      A_0[b_aoffset] += A[14 * f_size_idx_0 + b_aoffset] * u[f_size_idx_0];
    }

    d[b_aoffset] = A_tmp_2[b_aoffset] - A_0[b_aoffset];
  }

  // 'wls_alloc:62' i_free = W==0;
  for (i = 0; i < 10; i++) {
    i_free[i] = (W[i] == 0.0F);
  }

  // 'wls_alloc:66' for iter = 1:imax
  iter = 1.0F;
  A_tmp_tmp = 0;
  exitg1 = false;
  while ((!exitg1) && (A_tmp_tmp <= (int32_T)imax - 1)) {
    iter = 1.0F + (real32_T)A_tmp_tmp;

    // 'wls_alloc:72' A_free = A(:,i_free);
    i = 0;
    for (b_aoffset = 0; b_aoffset < 10; b_aoffset++) {
      if (i_free[b_aoffset]) {
        i++;
      }
    }

    A_tmp_tmp_0 = i;
    i = 0;
    for (b_aoffset = 0; b_aoffset < 10; b_aoffset++) {
      if (i_free[b_aoffset]) {
        e_data[i] = (int8_T)(b_aoffset + 1);
        i++;
      }
    }

    A_free_size[0] = 14;
    A_free_size[1] = A_tmp_tmp_0;
    for (b_aoffset = 0; b_aoffset < A_tmp_tmp_0; b_aoffset++) {
      for (f_size_idx_0 = 0; f_size_idx_0 < 14; f_size_idx_0++) {
        A_free_data[f_size_idx_0 + 14 * b_aoffset] = A[(e_data[b_aoffset] - 1) *
          14 + f_size_idx_0];
      }
    }

    // 'wls_alloc:74' p_free = A_free\d;
    mldivide_k(A_free_data, A_free_size, d, p_free_data, &aoffset);

    // 'wls_alloc:76' p = zeros(m,1);
    // 'wls_alloc:78' p(i_free) = p_free;
    i = 0;

    // 'wls_alloc:84' u_opt = u + p;
    for (b_aoffset = 0; b_aoffset < 10; b_aoffset++) {
      p_0 = 0.0;
      if (i_free[b_aoffset]) {
        p_0 = p_free_data[i];
        i++;
      }

      u_opt[b_aoffset] = u[b_aoffset] + (real32_T)p_0;
      p[b_aoffset] = p_0;
    }

    // 'wls_alloc:85' infeasible = (u_opt < umin) | (u_opt > umax);
    // 'wls_alloc:87' if ~any(infeasible(i_free))
    i = 0;
    for (b_aoffset = 0; b_aoffset < 10; b_aoffset++) {
      if (i_free[b_aoffset]) {
        i++;
      }
    }

    f_size_idx_0 = i;
    i = 0;
    for (b_aoffset = 0; b_aoffset < 10; b_aoffset++) {
      if (i_free[b_aoffset]) {
        f_data[i] = (int8_T)(b_aoffset + 1);
        i++;
      }

      u_opt_0[b_aoffset] = ((u_opt[b_aoffset] < umin[b_aoffset]) ||
                            (u_opt[b_aoffset] > umax[b_aoffset]));
    }

    for (b_aoffset = 0; b_aoffset < f_size_idx_0; b_aoffset++) {
      u_opt_data[b_aoffset] = u_opt_0[f_data[b_aoffset] - 1];
    }

    if (!any(u_opt_data, &f_size_idx_0)) {
      // 'wls_alloc:94' u = u_opt;
      for (i = 0; i < 10; i++) {
        u[i] = u_opt[i];
      }

      // 'wls_alloc:95' d = d - A_free*p_free;
      if (A_tmp_tmp_0 == 1) {
        for (b_aoffset = 0; b_aoffset < 14; b_aoffset++) {
          A_tmp_2[b_aoffset] = 0.0F;
          for (f_size_idx_0 = 0; f_size_idx_0 < A_tmp_tmp_0; f_size_idx_0++) {
            A_tmp_2[b_aoffset] += A_free_data[14 * f_size_idx_0 + b_aoffset] *
              p_free_data[f_size_idx_0];
          }
        }
      } else if (aoffset == 1) {
        for (b_aoffset = 0; b_aoffset < 14; b_aoffset++) {
          A_tmp_2[b_aoffset] = 0.0F;
          for (f_size_idx_0 = 0; f_size_idx_0 < A_tmp_tmp_0; f_size_idx_0++) {
            A_tmp_2[b_aoffset] += A_free_data[14 * f_size_idx_0 + b_aoffset] *
              p_free_data[f_size_idx_0];
          }
        }
      } else {
        for (i = 0; i < 14; i++) {
          A_tmp_2[i] = 0.0F;
        }

        for (i = 0; i < A_tmp_tmp_0; i++) {
          b_aoffset = i * 14;
          for (f_size_idx_0 = 0; f_size_idx_0 < 14; f_size_idx_0++) {
            aoffset = b_aoffset + f_size_idx_0;
            A_tmp_2[f_size_idx_0] += A[(e_data[aoffset / 14] - 1) * 14 + aoffset
              % 14] * p_free_data[i];
          }
        }
      }

      for (b_aoffset = 0; b_aoffset < 14; b_aoffset++) {
        d[b_aoffset] -= A_tmp_2[b_aoffset];
      }

      // 'wls_alloc:97' lambda = W.*(A'*d);
      // 'wls_alloc:99' if lambda >= -eps
      for (i = 0; i < 10; i++) {
        p_free_data[i] = 0.0F;
        for (b_aoffset = 0; b_aoffset < 14; b_aoffset++) {
          p_free_data[i] += A[14 * i + b_aoffset] * d[b_aoffset];
        }

        gam_sq = W[i] * p_free_data[i];
        u_opt_0[i] = (gam_sq >= -2.22044605E-16F);
        u_opt[i] = gam_sq;
      }

      x = true;
      i = 0;
      exitg2 = false;
      while ((!exitg2) && (i < 10)) {
        if (!u_opt_0[i]) {
          x = false;
          exitg2 = true;
        } else {
          i++;
        }
      }

      if (x) {
        exitg1 = true;
      } else {
        // 'wls_alloc:112' [lambda_neg,i_neg] = min(lambda);
        gam_sq = u_opt[0];
        i = 0;
        for (b_aoffset = 0; b_aoffset < 9; b_aoffset++) {
          tmp_0 = u_opt[b_aoffset + 1];
          if (gam_sq > tmp_0) {
            gam_sq = tmp_0;
            i = b_aoffset + 1;
          }
        }

        // 'wls_alloc:113' W(i_neg) = 0;
        W[i] = 0.0F;

        // 'wls_alloc:114' i_free(i_neg) = 1;
        i_free[i] = true;
        A_tmp_tmp++;
      }
    } else {
      // 'wls_alloc:116' else
      // 'wls_alloc:124' dist = ones(m,1);
      // 'wls_alloc:125' i_min = i_free & p<0;
      // 'wls_alloc:126' i_max = i_free & p>0;
      // 'wls_alloc:128' dist(i_min) = (umin(i_min) - u(i_min)) ./ p(i_min);
      i = 0;
      for (b_aoffset = 0; b_aoffset < 10; b_aoffset++) {
        dist[b_aoffset] = 1.0;
        x = (p[b_aoffset] < 0.0);
        u_opt_data[b_aoffset] = (p[b_aoffset] > 0.0);
        if (i_free[b_aoffset] && x) {
          i++;
        }

        u_opt_0[b_aoffset] = x;
      }

      f_size_idx_0 = i;
      i = 0;
      for (b_aoffset = 0; b_aoffset < 10; b_aoffset++) {
        if (i_free[b_aoffset] && u_opt_0[b_aoffset]) {
          g_data[i] = (int8_T)(b_aoffset + 1);
          i++;
        }
      }

      for (b_aoffset = 0; b_aoffset < f_size_idx_0; b_aoffset++) {
        i = g_data[b_aoffset] - 1;
        b_data[b_aoffset] = (umin[i] - u[i]) / (real32_T)p[i];
      }

      i = 0;
      for (b_aoffset = 0; b_aoffset < 10; b_aoffset++) {
        if (i_free[b_aoffset] && u_opt_0[b_aoffset]) {
          dist[b_aoffset] = b_data[i];
          i++;
        }
      }

      // 'wls_alloc:129' dist(i_max) = (umax(i_max) - u(i_max)) ./ p(i_max);
      i = 0;
      for (b_aoffset = 0; b_aoffset < 10; b_aoffset++) {
        if (i_free[b_aoffset] && u_opt_data[b_aoffset]) {
          i++;
        }
      }

      f_size_idx_0 = i;
      i = 0;
      for (b_aoffset = 0; b_aoffset < 10; b_aoffset++) {
        if (i_free[b_aoffset] && u_opt_data[b_aoffset]) {
          h_data[i] = (int8_T)(b_aoffset + 1);
          i++;
        }
      }

      for (b_aoffset = 0; b_aoffset < f_size_idx_0; b_aoffset++) {
        i = h_data[b_aoffset] - 1;
        b_data[b_aoffset] = (umax[i] - u[i]) / (real32_T)p[i];
      }

      i = 0;
      for (b_aoffset = 0; b_aoffset < 10; b_aoffset++) {
        if (i_free[b_aoffset] && u_opt_data[b_aoffset]) {
          dist[b_aoffset] = b_data[i];
          i++;
        }
      }

      // 'wls_alloc:132' [alpha,i_alpha] = min(dist);
      p_0 = dist[0];
      i = 0;
      for (b_aoffset = 0; b_aoffset < 9; b_aoffset++) {
        tmp = dist[b_aoffset + 1];
        if (p_0 > tmp) {
          p_0 = tmp;
          i = b_aoffset + 1;
        }
      }

      // 'wls_alloc:134' u = u + alpha*p;
      for (b_aoffset = 0; b_aoffset < 10; b_aoffset++) {
        u[b_aoffset] += (real32_T)(p_0 * p[b_aoffset]);
      }

      // 'wls_alloc:135' d = d - A_free*alpha*p_free;
      f_size_idx_0 = 14 * A_tmp_tmp_0 - 1;
      for (b_aoffset = 0; b_aoffset <= f_size_idx_0; b_aoffset++) {
        A_free_data[b_aoffset] *= (real32_T)p_0;
      }

      if (A_tmp_tmp_0 == 1) {
        for (b_aoffset = 0; b_aoffset < 14; b_aoffset++) {
          A_tmp_2[b_aoffset] = 0.0F;
          for (f_size_idx_0 = 0; f_size_idx_0 < 1; f_size_idx_0++) {
            A_tmp_2[b_aoffset] += A_free_data[b_aoffset] * p_free_data[0];
          }
        }
      } else if (aoffset == 1) {
        for (b_aoffset = 0; b_aoffset < 14; b_aoffset++) {
          A_tmp_2[b_aoffset] = 0.0F;
          for (f_size_idx_0 = 0; f_size_idx_0 < A_tmp_tmp_0; f_size_idx_0++) {
            A_tmp_2[b_aoffset] += A_free_data[14 * f_size_idx_0 + b_aoffset] *
              p_free_data[f_size_idx_0];
          }
        }
      } else {
        for (b_aoffset = 0; b_aoffset < 14; b_aoffset++) {
          A_tmp_2[b_aoffset] = 0.0F;
        }

        for (f_size_idx_0 = 0; f_size_idx_0 < A_tmp_tmp_0; f_size_idx_0++) {
          aoffset = f_size_idx_0 * 14;
          for (b_aoffset = 0; b_aoffset < 14; b_aoffset++) {
            A_tmp_2[b_aoffset] += A_free_data[aoffset + b_aoffset] *
              p_free_data[f_size_idx_0];
          }
        }
      }

      for (b_aoffset = 0; b_aoffset < 14; b_aoffset++) {
        d[b_aoffset] -= A_tmp_2[b_aoffset];
      }

      // 'wls_alloc:138' W(i_alpha) = sign(p(i_alpha));
      if (p[i] < 0.0) {
        W[i] = -1.0F;
      } else if (p[i] > 0.0) {
        W[i] = 1.0F;
      } else {
        W[i] = (real32_T)p[i];
      }

      // 'wls_alloc:139' i_free(i_alpha) = 0;
      i_free[i] = false;
      A_tmp_tmp++;
    }
  }

  return iter;
}

// Function for MATLAB Function: '<S7>/Flap deflections'
real32_T MatlabControllerClass::sum(const real32_T x[8])
{
  real32_T y;
  int32_T k;
  y = x[0];
  for (k = 0; k < 7; k++) {
    y += x[k + 1];
  }

  return y;
}

// Model step function
void MatlabControllerClass::step()
{
  real32_T q0_q0;
  real32_T q1_q1;
  real32_T q2_q2;
  real32_T q0_q3;
  real32_T q1_q2;
  real32_T q1_q3;
  real32_T scale;
  real32_T absxk;
  real32_T t;
  real32_T I_b[9];
  real32_T force_dir[30];
  real32_T c_XYZ[30];
  int32_T p2;
  int32_T p3;
  int32_T itmp;
  real32_T c[30];
  real32_T umin[10];
  real32_T umax[10];
  real32_T W_u[100];
  real32_T x[7];
  real32_T wp_approach_out[15];
  boolean_T is_approach;
  int32_T wp_idx_app_2;
  int32_T stage_app_2;
  int32_T stage_2;
  int32_T wp_idx_2;
  int32_T stage_tmp;
  int32_T wp_idx_tmp;
  real32_T circ_seg_n[3];
  real32_T circ_seg_start[3];
  int32_T c_stage_app;
  static const int8_T h[3] = { 3, 4, 0 };

  static const int8_T i[3] = { 4, 0, 1 };

  real32_T k2[6];
  real32_T k3[6];
  real32_T k4[6];
  real32_T a_Kb_yz[2];
  real32_T g_b_yz[2];
  real32_T G11_2[9];
  real32_T Delta_u_max[9];
  real32_T umax_0[9];
  real32_T W_v[16];
  real32_T W_u_0[81];
  real32_T z1[9];
  real32_T rtb_Sum2_os;
  real32_T rtb_Delay2_k;
  real32_T rtb_Delay1_eh;
  real32_T rtb_Delay_i;
  real32_T rtb_Delay2_oz;
  real32_T rtb_Delay1_a;
  real32_T rtb_Delay_j;
  real32_T rtb_Delay2_o;
  real32_T rtb_Delay1_i;
  real32_T rtb_Delay_o;
  real32_T rtb_Delay2_b;
  real32_T rtb_Delay1_f;
  real32_T rtb_Delay_n;
  real32_T rtb_y_dm;
  real32_T rtb_Delay2_g;
  real32_T rtb_Delay1_k;
  real32_T rtb_Delay_m;
  real32_T rtb_y_dt_fn[6];
  real32_T rtb_y_e3;
  real32_T rtb_y_fg[3];
  real32_T rtb_DiscreteTimeIntegrator1_k[6];
  real32_T rtb_G4xX[40];
  real32_T rtb_M_bg[9];
  real32_T rtb_G11[36];
  real32_T rtb_Product2_ba[60];
  real32_T rtb_y0_k[6];
  real32_T rtb_Sum2_cx[3];
  real32_T rtb_y0[6];
  real32_T rtb_y_ku[30];
  real32_T rtb_Add_p[11];
  real32_T rtb_Sum2_p[3];
  real32_T rtb_Sum2_g[3];
  real32_T rtb_Sum2_gn;
  real32_T rtb_Sum2_mb;
  real32_T rtb_Sum2_eh;
  real32_T rtb_Sum2_dqn;
  real32_T rtb_Sum2_d0;
  real32_T rtb_Sum2_h0;
  real32_T rtb_Sum2_p1;
  real32_T rtb_Sum2_ms;
  real32_T rtb_Sum2_ig[10];
  real32_T rtb_V_Kg_n0[3];
  real32_T rtb_p_ahead_i[3];
  real32_T rtb_Delta_u[10];
  real32_T rtb_Delay[10];
  real32_T rtb_Delay1[10];
  real32_T rtb_Delay2[10];
  real32_T rtb_u_delay[10];
  real32_T rtb_y_if[11];
  boolean_T rtb_Compare_mi;
  real32_T rtb_Divide_l;
  real32_T rtb_DiscreteTimeIntegrator1[10];
  real32_T rtb_Divide_h;
  real32_T rtb_Divide_k;
  real32_T rtb_Divide_p;
  real32_T rtb_omega2_pb;
  uint8_T rtb_Compare;
  boolean_T rtb_Compare_j;
  int32_T i_0;
  real32_T q2_q2_0[8];
  real32_T a_Kb_yz_0[4];
  real32_T tmp[2];
  real32_T circ_seg_n_0[3];
  real32_T rtb_y_a5[3];
  uint16_T p3_0[3];
  int32_T rtb_y_g_size[2];
  int32_T rtb_y_g_size_0[2];
  int32_T rtb_y_g_size_1[2];
  int32_T rtb_y_g_size_2[2];
  int32_T rtb_y_g_size_3[2];
  real32_T s_g_ref_dt2[3];
  real32_T rtb_y_p3;
  real32_T q_bg_unsigned_idx_3;
  real32_T q_bg_unsigned_idx_2;
  real32_T q_bg_unsigned_idx_1;
  real32_T q_bg_unsigned_idx_0;
  real32_T umax_1;
  real32_T rtb_omega2_o_0;
  real32_T rtb_omega2_oa_0;
  real32_T rtb_omega2_d_0;
  real32_T rtb_omega2_e_0;

  // RelationalOperator: '<S3>/Compare' incorporates:
  //   Constant: '<S3>/Constant'
  //   Inport: '<Root>/cmd'

  rtb_Compare = (rtU.cmd.RC_pwm[7] < 1600.0F);

  // MATLAB Function: '<Root>/Remove velocity' incorporates:
  //   Inport: '<Root>/cmd'

  // :  y = u(1:3,:);
  for (stage_app_2 = 0; stage_app_2 < 10; stage_app_2++) {
    i_0 = stage_app_2 << 2;
    rtb_y_ku[3 * stage_app_2] = rtU.cmd.waypoints[i_0];
    rtb_y_ku[1 + 3 * stage_app_2] = rtU.cmd.waypoints[i_0 + 1];
    rtb_y_ku[2 + 3 * stage_app_2] = rtU.cmd.waypoints[i_0 + 2];
  }

  // End of MATLAB Function: '<Root>/Remove velocity'

  // MATLAB Function: '<Root>/Quaternions to Rotation Matrix' incorporates:
  //   Inport: '<Root>/measure'

  // :  M_bg = quat2Dcm(q_bg);
  // 'quat2Dcm:30' q_bg = quatNormalize( q_bg );
  // 'quatNormalize:31' q_out = q / max( eps, norm(q, 2) );
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
    q1_q1 = q2_q2;
  } else {
    q1_q1 = 2.22044605E-16F;
  }

  q_bg_unsigned_idx_0 = rtU.measure.q_bg[0] / q1_q1;
  q_bg_unsigned_idx_1 = rtU.measure.q_bg[1] / q1_q1;
  q_bg_unsigned_idx_2 = rtU.measure.q_bg[2] / q1_q1;
  q_bg_unsigned_idx_3 = rtU.measure.q_bg[3] / q1_q1;

  // 'quat2Dcm:33' q0_q0 = q_bg(1)^2;
  q0_q0 = q_bg_unsigned_idx_0 * q_bg_unsigned_idx_0;

  // 'quat2Dcm:34' q1_q1 = q_bg(2)^2;
  q1_q1 = q_bg_unsigned_idx_1 * q_bg_unsigned_idx_1;

  // 'quat2Dcm:35' q2_q2 = q_bg(3)^2;
  q2_q2 = q_bg_unsigned_idx_2 * q_bg_unsigned_idx_2;

  // 'quat2Dcm:36' q3_q3 = q_bg(4)^2;
  scale = q_bg_unsigned_idx_3 * q_bg_unsigned_idx_3;

  // 'quat2Dcm:37' q0_q1 = q_bg(1)*q_bg(2);
  absxk = q_bg_unsigned_idx_0 * q_bg_unsigned_idx_1;

  // 'quat2Dcm:38' q0_q2 = q_bg(1)*q_bg(3);
  t = q_bg_unsigned_idx_0 * q_bg_unsigned_idx_2;

  // 'quat2Dcm:39' q0_q3 = q_bg(1)*q_bg(4);
  q0_q3 = q_bg_unsigned_idx_0 * q_bg_unsigned_idx_3;

  // 'quat2Dcm:40' q1_q2 = q_bg(2)*q_bg(3);
  q1_q2 = q_bg_unsigned_idx_1 * q_bg_unsigned_idx_2;

  // 'quat2Dcm:41' q1_q3 = q_bg(2)*q_bg(4);
  q1_q3 = q_bg_unsigned_idx_1 * q_bg_unsigned_idx_3;

  // 'quat2Dcm:42' q2_q3 = q_bg(3)*q_bg(4);
  q_bg_unsigned_idx_3 *= q_bg_unsigned_idx_2;

  // 'quat2Dcm:45' M_bg = [
  // 'quat2Dcm:46'     q0_q0 + q1_q1 - q2_q2 - q3_q3,...
  // 'quat2Dcm:47'     2*(q1_q2 + q0_q3),...
  // 'quat2Dcm:48'     2*(q1_q3 - q0_q2); ...
  // 'quat2Dcm:49'     ...
  // 'quat2Dcm:50'     2*(q1_q2 - q0_q3),...
  // 'quat2Dcm:51'     q0_q0 - q1_q1 + q2_q2 - q3_q3,...
  // 'quat2Dcm:52'     2*(q2_q3 + q0_q1);
  // 'quat2Dcm:53'     ...
  // 'quat2Dcm:54'     2*(q1_q3 + q0_q2),...
  // 'quat2Dcm:55'     2*(q2_q3 - q0_q1),...
  // 'quat2Dcm:56'     q0_q0 - q1_q1 - q2_q2 + q3_q3 ];
  rtb_M_bg[0] = ((q0_q0 + q1_q1) - q2_q2) - scale;
  rtb_M_bg[3] = (q1_q2 + q0_q3) * 2.0F;
  rtb_M_bg[6] = (q1_q3 - t) * 2.0F;
  rtb_M_bg[1] = (q1_q2 - q0_q3) * 2.0F;
  q0_q0 -= q1_q1;
  rtb_M_bg[4] = (q0_q0 + q2_q2) - scale;
  rtb_M_bg[7] = (q_bg_unsigned_idx_3 + absxk) * 2.0F;
  rtb_M_bg[2] = (q1_q3 + t) * 2.0F;
  rtb_M_bg[5] = (q_bg_unsigned_idx_3 - absxk) * 2.0F;
  rtb_M_bg[8] = (q0_q0 - q2_q2) + scale;

  // End of MATLAB Function: '<Root>/Quaternions to Rotation Matrix'

  // RelationalOperator: '<S4>/Compare' incorporates:
  //   Constant: '<S4>/Constant'
  //   Inport: '<Root>/cmd'

  // :  y = (u-1500)/500;
  rtb_Compare_mi = (rtU.cmd.RC_pwm[7] < 1400.0F);

  // Outputs for Enabled SubSystem: '<Root>/LindiPlane Autopilot' incorporates:
  //   EnablePort: '<S9>/Enable'

  // RelationalOperator: '<S5>/Compare' incorporates:
  //   Constant: '<S140>/Constant'
  //   Constant: '<S146>/Constant'
  //   Constant: '<S26>/Constant1'
  //   Constant: '<S46>/Constant'
  //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y_dt'
  //   Gain: '<S51>/Gain'
  //   Inport: '<Root>/measure'
  //   Inport: '<S148>/In1'
  //   Inport: '<S148>/In2'
  //   Inport: '<S148>/In3'
  //   Logic: '<S25>/Logical Operator'
  //   MATLAB Function: '<S29>/Outer Loop INDI'
  //   MATLAB Function: '<S37>/WpNav Matching'
  //   MATLAB Function: '<S67>/DCM to quaternions1'
  //   MATLAB Function: '<S67>/Quaternions to Euler angles1'
  //   Product: '<S51>/Product1'
  //   Product: '<S51>/Product2'
  //   Product: '<S51>/omega^2'
  //   RelationalOperator: '<S146>/Compare'
  //   RelationalOperator: '<S46>/Compare'
  //   Sum: '<S51>/Sum2'
  //   Sum: '<S51>/Sum3'

  if (!rtb_Compare_mi) {
    if (!rtDW.LindiPlaneAutopilot_MODE) {
      // InitializeConditions for DiscreteIntegrator: '<S52>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_IC_LOAD = 1U;

      // InitializeConditions for DiscreteIntegrator: '<S48>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_IC_LO_a = 1U;

      // InitializeConditions for DiscreteIntegrator: '<S49>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_DSTAT_n[0] = 0.0F;
      rtDW.DiscreteTimeIntegratory_DSTAT_n[1] = 0.0F;
      rtDW.DiscreteTimeIntegratory_DSTAT_n[2] = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S50>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_IC_LO_e = 1U;

      // InitializeConditions for DiscreteIntegrator: '<S51>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_IC_LO_i = 1U;

      // InitializeConditions for UnitDelay: '<S33>/Unit Delay'
      rtDW.UnitDelay_DSTATE = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S80>/Discrete-Time Integrator' 
      rtDW.DiscreteTimeIntegrator_IC_LOADI = 1U;

      // InitializeConditions for DiscreteIntegrator: '<S122>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_DSTAT_p = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S122>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_DS_g = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S134>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_IC_LO_m = 1U;

      // InitializeConditions for DiscreteIntegrator: '<S134>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_DS_f = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S133>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_IC_LO_d = 1U;

      // InitializeConditions for DiscreteIntegrator: '<S130>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_IC_LO_o = 1U;

      // InitializeConditions for DiscreteIntegrator: '<S97>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_IC_L_eu = 1U;

      // InitializeConditions for DiscreteIntegrator: '<S97>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_DS_j = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S93>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_IC_LO_k = 1U;

      // InitializeConditions for DiscreteIntegrator: '<S89>/Discrete-Time Integrator' 
      rtDW.DiscreteTimeIntegrator_DSTATE_h = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S91>/Discrete-Time Integrator' 
      rtDW.DiscreteTimeIntegrator_DSTAT_en = 0.0F;

      // InitializeConditions for UnitDelay: '<S34>/Unit Delay'
      rtDW.UnitDelay_DSTATE_f = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S154>/Discrete-Time Integrator' 
      rtDW.DiscreteTimeIntegrator_IC_LOA_g = 1U;

      // InitializeConditions for DiscreteIntegrator: '<S157>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_IC_LO_f = 1U;

      // InitializeConditions for DiscreteIntegrator: '<S157>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_DS_n = 0.0F;

      // InitializeConditions for UnitDelay: '<S100>/Unit Delay'
      rtDW.UnitDelay_DSTATE_i = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S116>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_DSTA_na = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S115>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_DSTAT_i = 0.0F;

      // InitializeConditions for Delay: '<S107>/Delay'
      rtDW.icLoad = 1U;

      // InitializeConditions for Delay: '<S107>/Delay1'
      rtDW.icLoad_i = 1U;

      // InitializeConditions for Delay: '<S107>/Delay2'
      rtDW.icLoad_o = 1U;

      // InitializeConditions for Delay: '<S107>/Delay3'
      rtDW.icLoad_k = 1U;
      for (i_0 = 0; i_0 < 10; i_0++) {
        // InitializeConditions for DiscreteIntegrator: '<S106>/Discrete-Time Integrator y' 
        rtDW.DiscreteTimeIntegratory_DSTAT_l[i_0] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S105>/Discrete-Time Integrator y' 
        rtDW.DiscreteTimeIntegratory_DSTA_j2[i_0] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S109>/Discrete-Time Integrator1' 
        rtDW.DiscreteTimeIntegrator1_DSTATE[i_0] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S109>/Discrete-Time Integrator2' 
        rtDW.DiscreteTimeIntegrator2_DSTATE[i_0] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S106>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_DS_o[i_0] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S105>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_D_nz[i_0] = 0.0F;
      }

      // InitializeConditions for DiscreteIntegrator: '<S48>/Discrete-Time Integrator y_dt' 
      for (i_0 = 0; i_0 < 9; i_0++) {
        rtDW.DiscreteTimeIntegratory_dt_D_ni[i_0] = 0.0F;
      }

      // End of InitializeConditions for DiscreteIntegrator: '<S48>/Discrete-Time Integrator y_dt' 

      // InitializeConditions for DiscreteIntegrator: '<S51>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_DS_e[0] = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S50>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_DS_p[0] = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S49>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_DS_i[0] = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S51>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_DS_e[1] = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S50>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_DS_p[1] = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S49>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_DS_i[1] = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S51>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_DS_e[2] = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S50>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_DS_p[2] = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S49>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_DS_i[2] = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S115>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_D_pj = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S116>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_D_ol = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S156>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_IC_L_km = 1U;

      // InitializeConditions for DiscreteIntegrator: '<S156>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_D_jq = 0.0F;

      // InitializeConditions for Delay: '<S158>/Delay'
      rtDW.icLoad_d = 1U;

      // InitializeConditions for Delay: '<S158>/Delay1'
      rtDW.icLoad_b = 1U;

      // InitializeConditions for Delay: '<S158>/Delay2'
      rtDW.icLoad_l = 1U;

      // InitializeConditions for Delay: '<S158>/Delay3'
      rtDW.icLoad_b1 = 1U;

      // InitializeConditions for DiscreteIntegrator: '<S90>/Discrete-Time Integrator' 
      rtDW.DiscreteTimeIntegrator_DSTATE_a = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S93>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_D_n5 = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S92>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_IC_LO_g = 1U;

      // InitializeConditions for DiscreteIntegrator: '<S92>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_D_j0 = 0.0F;

      // InitializeConditions for Delay: '<S94>/Delay'
      rtDW.icLoad_dk = 1U;

      // InitializeConditions for Delay: '<S94>/Delay1'
      rtDW.icLoad_j = 1U;

      // InitializeConditions for Delay: '<S94>/Delay2'
      rtDW.icLoad_ix = 1U;

      // InitializeConditions for Delay: '<S94>/Delay3'
      rtDW.icLoad_ow = 1U;

      // InitializeConditions for DiscreteIntegrator: '<S96>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_IC_L_el = 1U;

      // InitializeConditions for DiscreteIntegrator: '<S96>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_DS_a = 0.0F;

      // InitializeConditions for Delay: '<S98>/Delay'
      rtDW.icLoad_dh = 1U;

      // InitializeConditions for Delay: '<S98>/Delay1'
      rtDW.icLoad_f = 1U;

      // InitializeConditions for Delay: '<S98>/Delay2'
      rtDW.icLoad_fc = 1U;

      // InitializeConditions for Delay: '<S98>/Delay3'
      rtDW.icLoad_jp = 1U;

      // InitializeConditions for DiscreteIntegrator: '<S130>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_DS_m = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S129>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_IC_LO_b = 1U;

      // InitializeConditions for DiscreteIntegrator: '<S129>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_DS_b = 0.0F;

      // InitializeConditions for Delay: '<S131>/Delay'
      rtDW.icLoad_dhb = 1U;

      // InitializeConditions for Delay: '<S131>/Delay1'
      rtDW.icLoad_fx = 1U;

      // InitializeConditions for Delay: '<S131>/Delay2'
      rtDW.icLoad_m = 1U;

      // InitializeConditions for Delay: '<S131>/Delay3'
      rtDW.icLoad_dc = 1U;

      // InitializeConditions for DiscreteIntegrator: '<S133>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_DS_k = 0.0F;

      // InitializeConditions for Delay: '<S135>/Delay'
      rtDW.icLoad_f1 = 1U;

      // InitializeConditions for Delay: '<S135>/Delay1'
      rtDW.icLoad_fu = 1U;

      // InitializeConditions for Delay: '<S135>/Delay2'
      rtDW.icLoad_ib = 1U;

      // InitializeConditions for Delay: '<S135>/Delay3'
      rtDW.icLoad_kd = 1U;

      // InitializeConditions for DiscreteIntegrator: '<S52>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_D_gf = 0.0F;
      rtDW.LindiPlaneAutopilot_MODE = true;
    }

    // MATLAB Function: '<S110>/indiCeFlapFix'
    // :  [G10] = indiCeFlapFix( cef, ceb );
    // 'indiCeFlapFix:3' I_b = [ ...
    // 'indiCeFlapFix:4'         ceb.ixx,    -ceb.ixy,	-ceb.ixz; ...
    // 'indiCeFlapFix:5'         -ceb.ixy,	ceb.iyy,	-ceb.iyz; ...
    // 'indiCeFlapFix:6'         -ceb.ixz,	-ceb.iyz,	ceb.izz ...
    // 'indiCeFlapFix:7'     ];
    I_b[0] = rtP.lindi.ceb.ixx;
    I_b[3] = -rtP.lindi.ceb.ixy;
    I_b[6] = -rtP.lindi.ceb.ixz;
    I_b[1] = -rtP.lindi.ceb.ixy;
    I_b[4] = rtP.lindi.ceb.iyy;
    I_b[7] = -rtP.lindi.ceb.iyz;
    I_b[2] = -rtP.lindi.ceb.ixz;
    I_b[5] = -rtP.lindi.ceb.iyz;
    I_b[8] = rtP.lindi.ceb.izz;

    // 'indiCeFlapFix:9' inv_I_b = inv(I_b);
    for (stage_app_2 = 0; stage_app_2 < 9; stage_app_2++) {
      G11_2[stage_app_2] = I_b[stage_app_2];
    }

    i_0 = 0;
    p2 = 3;
    p3 = 6;
    q0_q0 = std::abs(rtP.lindi.ceb.ixx);
    q1_q1 = std::abs(-rtP.lindi.ceb.ixy);
    q2_q2 = std::abs(-rtP.lindi.ceb.ixz);
    if ((q1_q1 > q0_q0) && (q1_q1 > q2_q2)) {
      i_0 = 3;
      p2 = 0;
      G11_2[0] = -rtP.lindi.ceb.ixy;
      G11_2[1] = rtP.lindi.ceb.ixx;
      G11_2[3] = rtP.lindi.ceb.iyy;
      G11_2[4] = -rtP.lindi.ceb.ixy;
      G11_2[6] = -rtP.lindi.ceb.iyz;
      G11_2[7] = -rtP.lindi.ceb.ixz;
    } else {
      if (q2_q2 > q0_q0) {
        i_0 = 6;
        p3 = 0;
        G11_2[0] = -rtP.lindi.ceb.ixz;
        G11_2[2] = rtP.lindi.ceb.ixx;
        G11_2[3] = -rtP.lindi.ceb.iyz;
        G11_2[5] = -rtP.lindi.ceb.ixy;
        G11_2[6] = rtP.lindi.ceb.izz;
        G11_2[8] = -rtP.lindi.ceb.ixz;
      }
    }

    scale = G11_2[1] / G11_2[0];
    G11_2[1] = scale;
    q1_q3 = G11_2[2] / G11_2[0];
    G11_2[2] = q1_q3;
    G11_2[4] -= scale * G11_2[3];
    G11_2[5] -= q1_q3 * G11_2[3];
    G11_2[7] -= scale * G11_2[6];
    G11_2[8] -= q1_q3 * G11_2[6];
    if (std::abs(G11_2[5]) > std::abs(G11_2[4])) {
      itmp = p2;
      p2 = p3;
      p3 = itmp;
      G11_2[1] = q1_q3;
      G11_2[2] = scale;
      q0_q0 = G11_2[4];
      G11_2[4] = G11_2[5];
      G11_2[5] = q0_q0;
      q0_q0 = G11_2[7];
      G11_2[7] = G11_2[8];
      G11_2[8] = q0_q0;
    }

    q1_q3 = G11_2[5] / G11_2[4];
    G11_2[8] -= q1_q3 * G11_2[7];
    q0_q0 = (q1_q3 * G11_2[1] - G11_2[2]) / G11_2[8];
    q1_q1 = -(G11_2[7] * q0_q0 + G11_2[1]) / G11_2[4];
    I_b[i_0] = ((1.0F - G11_2[3] * q1_q1) - G11_2[6] * q0_q0) / G11_2[0];
    I_b[i_0 + 1] = q1_q1;
    I_b[i_0 + 2] = q0_q0;
    q0_q0 = -q1_q3 / G11_2[8];
    q1_q1 = (1.0F - G11_2[7] * q0_q0) / G11_2[4];
    I_b[p2] = -(G11_2[3] * q1_q1 + G11_2[6] * q0_q0) / G11_2[0];
    I_b[p2 + 1] = q1_q1;
    I_b[p2 + 2] = q0_q0;
    q0_q0 = 1.0F / G11_2[8];
    q1_q1 = -G11_2[7] * q0_q0 / G11_2[4];
    I_b[p3] = -(G11_2[3] * q1_q1 + G11_2[6] * q0_q0) / G11_2[0];
    I_b[p3 + 1] = q1_q1;
    I_b[p3 + 2] = q0_q0;

    // 'indiCeFlapFix:11' force_dir = [ zeros(size(cef.rotx)); sin(cef.rotx); -cos(cef.rotx) ]; 
    for (itmp = 0; itmp < 10; itmp++) {
      q1_q3 = std::sin(rtP.lindi.cef.rotx[itmp]);
      force_dir[3 * itmp] = 0.0F;
      force_dir[1 + 3 * itmp] = q1_q3;
      force_dir[2 + 3 * itmp] = -std::cos(rtP.lindi.cef.rotx[itmp]);
      rtb_Delta_u[itmp] = q1_q3;
    }

    // 'indiCeFlapFix:12' c_XYZ = zeros(size(force_dir),class(cef.clu));
    // 'indiCeFlapFix:13' for i = 1:size(c_XYZ,1)
    for (i_0 = 0; i_0 < 3; i_0++) {
      // 'indiCeFlapFix:14' c_XYZ(i,:) = force_dir(i,:) .* cef.clu .* cef.s;
      for (stage_app_2 = 0; stage_app_2 < 10; stage_app_2++) {
        c_XYZ[i_0 + 3 * stage_app_2] = force_dir[3 * stage_app_2 + i_0] *
          rtP.lindi.cef.clu[stage_app_2] * rtP.lindi.cef.s[stage_app_2];
      }
    }

    // 'indiCeFlapFix:17' pos = [ cef.x; cef.y; cef.z ];
    for (stage_app_2 = 0; stage_app_2 < 10; stage_app_2++) {
      force_dir[3 * stage_app_2] = rtP.lindi.cef.x[stage_app_2];
      force_dir[1 + 3 * stage_app_2] = rtP.lindi.cef.y[stage_app_2];
      force_dir[2 + 3 * stage_app_2] = rtP.lindi.cef.z[stage_app_2];
    }

    // 'indiCeFlapFix:19' G10 = [ ...
    // 'indiCeFlapFix:20'         inv_I_b * cross( pos, c_XYZ ); ...
    // 'indiCeFlapFix:21'         c_XYZ / ceb.m ...
    // 'indiCeFlapFix:22'     ];
    for (i_0 = 0; i_0 <= 28; i_0 += 3) {
      for (p2 = i_0; p2 < i_0 + 1; p2++) {
        q0_q0 = force_dir[p2 + 2];
        q1_q1 = c_XYZ[p2 + 2];
        q1_q3 = c_XYZ[p2 + 1];
        scale = force_dir[p2 + 1];
        c[p2] = scale * q1_q1 - q0_q0 * q1_q3;
        c[p2 + 1] = q0_q0 * c_XYZ[p2] - q1_q1 * force_dir[p2];
        c[p2 + 2] = q1_q3 * force_dir[p2] - scale * c_XYZ[p2];
      }
    }

    // DiscreteIntegrator: '<S52>/Discrete-Time Integrator y' incorporates:
    //   Inport: '<Root>/measure'

    // 'indiCeFlapFix:24' if nargout > 1
    // 'indiCeFlapFix:30' if nargout > 2
    if (rtDW.DiscreteTimeIntegratory_IC_LOAD != 0) {
      rtDW.DiscreteTimeIntegratory_DSTAT_d = rtU.measure.airspeed;
    }

    // MinMax: '<S26>/Max' incorporates:
    //   Constant: '<S26>/Constant'
    //   DiscreteIntegrator: '<S52>/Discrete-Time Integrator y'

    if (rtDW.DiscreteTimeIntegratory_DSTAT_d > rtP.lindi.aspd.min) {
      q0_q0 = rtDW.DiscreteTimeIntegratory_DSTAT_d;
    } else {
      q0_q0 = rtP.lindi.aspd.min;
    }

    // End of MinMax: '<S26>/Max'

    // Product: '<S110>/Product5' incorporates:
    //   Math: '<S110>/Square'

    q1_q1 = q0_q0 * q0_q0 * 0.6125F;

    // MATLAB Function: '<S110>/indiCeFlapFix'
    for (stage_app_2 = 0; stage_app_2 < 10; stage_app_2++) {
      for (i_0 = 0; i_0 < 3; i_0++) {
        p3 = i_0 + 3 * stage_app_2;
        force_dir[p3] = 0.0F;
        p2 = 3 * stage_app_2 + i_0;
        force_dir[p3] = force_dir[p2] + c[3 * stage_app_2] * I_b[i_0];
        force_dir[p3] = c[3 * stage_app_2 + 1] * I_b[i_0 + 3] + force_dir[p2];
        force_dir[p3] = c[3 * stage_app_2 + 2] * I_b[i_0 + 6] + force_dir[p2];
      }
    }

    // Product: '<S110>/Product2' incorporates:
    //   Constant: '<S110>/Constant'
    //   MATLAB Function: '<S110>/indiCeFlapFix'
    //   Product: '<S110>/Product'

    for (stage_app_2 = 0; stage_app_2 < 10; stage_app_2++) {
      rtb_Product2_ba[6 * stage_app_2] = force_dir[3 * stage_app_2] * q1_q1 *
        rtP.lindi.ceb.scale;
      rtb_Product2_ba[3 + 6 * stage_app_2] = c_XYZ[3 * stage_app_2] /
        rtP.lindi.ceb.m * q1_q1 * rtP.lindi.ceb.scale;

      // MATLAB Function: '<S110>/indiCeFlapFix' incorporates:
      //   Constant: '<S110>/Constant'
      //   Product: '<S110>/Product'

      i_0 = 3 * stage_app_2 + 1;
      rtb_Product2_ba[1 + 6 * stage_app_2] = force_dir[i_0] * q1_q1 *
        rtP.lindi.ceb.scale;
      rtb_Product2_ba[4 + 6 * stage_app_2] = c_XYZ[i_0] / rtP.lindi.ceb.m *
        q1_q1 * rtP.lindi.ceb.scale;

      // MATLAB Function: '<S110>/indiCeFlapFix' incorporates:
      //   Constant: '<S110>/Constant'
      //   Product: '<S110>/Product'

      i_0 = 3 * stage_app_2 + 2;
      rtb_Product2_ba[2 + 6 * stage_app_2] = force_dir[i_0] * q1_q1 *
        rtP.lindi.ceb.scale;
      rtb_Product2_ba[5 + 6 * stage_app_2] = c_XYZ[i_0] / rtP.lindi.ceb.m *
        q1_q1 * rtP.lindi.ceb.scale;
    }

    // End of Product: '<S110>/Product2'

    // DiscreteIntegrator: '<S48>/Discrete-Time Integrator y'
    if (rtDW.DiscreteTimeIntegratory_IC_LO_a != 0) {
      for (i_0 = 0; i_0 < 9; i_0++) {
        rtDW.DiscreteTimeIntegratory_DSTAT_b[i_0] = rtb_M_bg[i_0];
      }
    }

    // RelationalOperator: '<S22>/Compare' incorporates:
    //   Constant: '<S22>/Constant'

    rtb_Compare_j = (rtb_Compare == 0);

    // DiscreteIntegrator: '<S50>/Discrete-Time Integrator y' incorporates:
    //   Inport: '<Root>/measure'

    if (rtDW.DiscreteTimeIntegratory_IC_LO_e != 0) {
      rtDW.DiscreteTimeIntegratory_DSTA_n2[0] = rtU.measure.s_Kg[0];
      rtDW.DiscreteTimeIntegratory_DSTA_n2[1] = rtU.measure.s_Kg[1];
      rtDW.DiscreteTimeIntegratory_DSTA_n2[2] = rtU.measure.s_Kg[2];
    }

    // DiscreteIntegrator: '<S51>/Discrete-Time Integrator y' incorporates:
    //   Inport: '<Root>/measure'

    if (rtDW.DiscreteTimeIntegratory_IC_LO_i != 0) {
      rtDW.DiscreteTimeIntegratory_DSTAT_j[0] = rtU.measure.V_Kg[0];
      rtDW.DiscreteTimeIntegratory_DSTAT_j[1] = rtU.measure.V_Kg[1];
      rtDW.DiscreteTimeIntegratory_DSTAT_j[2] = rtU.measure.V_Kg[2];
    }

    // Outputs for Enabled SubSystem: '<S9>/NDI Position Controller' incorporates:
    //   EnablePort: '<S28>/Enable'

    // Outputs for Enabled SubSystem: '<S9>/Waypoint Navigation' incorporates:
    //   EnablePort: '<S37>/Enable'

    if (rtb_Compare_j) {
      if (!rtDW.WaypointNavigation_MODE) {
        rtDW.WaypointNavigation_MODE = true;
      }

      // MATLAB Function: '<S37>/Split waypoints and velocity' incorporates:
      //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y'

      // :  if size(waypoints,2) < 1
      // :  else
      // :  wp = waypoints(1:3,:);
      // :  if size(waypoints,1)<4
      // :  vel_d = repmat(norm(V_Kg),1,size(waypoints,2));
      scale = 1.29246971E-26F;
      absxk = std::abs(rtDW.DiscreteTimeIntegratory_DSTAT_j[0]);
      if (absxk > 1.29246971E-26F) {
        q1_q1 = 1.0F;
        scale = absxk;
      } else {
        t = absxk / 1.29246971E-26F;
        q1_q1 = t * t;
      }

      // MATLAB Function: '<S37>/Avoid zero speed' incorporates:
      //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y'

      rtb_V_Kg_n0[0] = rtDW.DiscreteTimeIntegratory_DSTAT_j[0];

      // MATLAB Function: '<S37>/Split waypoints and velocity' incorporates:
      //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y'

      absxk = std::abs(rtDW.DiscreteTimeIntegratory_DSTAT_j[1]);
      if (absxk > scale) {
        t = scale / absxk;
        q1_q1 = q1_q1 * t * t + 1.0F;
        scale = absxk;
      } else {
        t = absxk / scale;
        q1_q1 += t * t;
      }

      // MATLAB Function: '<S37>/Avoid zero speed' incorporates:
      //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y'

      rtb_V_Kg_n0[1] = rtDW.DiscreteTimeIntegratory_DSTAT_j[1];

      // MATLAB Function: '<S37>/Split waypoints and velocity' incorporates:
      //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y'

      absxk = std::abs(rtDW.DiscreteTimeIntegratory_DSTAT_j[2]);
      if (absxk > scale) {
        t = scale / absxk;
        q1_q1 = q1_q1 * t * t + 1.0F;
        scale = absxk;
      } else {
        t = absxk / scale;
        q1_q1 += t * t;
      }

      // MATLAB Function: '<S37>/Avoid zero speed' incorporates:
      //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y'

      rtb_V_Kg_n0[2] = rtDW.DiscreteTimeIntegratory_DSTAT_j[2];

      // MATLAB Function: '<S37>/Split waypoints and velocity'
      q1_q1 = scale * std::sqrt(q1_q1);

      // MATLAB Function: '<S37>/Avoid zero speed' incorporates:
      //   DiscreteIntegrator: '<S50>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y'
      //   MATLAB Function: '<S37>/Split waypoints and velocity'
      //   UnitDelay: '<S37>/Unit Delay'

      // :  V_K = norm(V_Kg,2);
      // :  V_Kg_n0 = V_Kg;
      // :  if V_K < 0.3
      if (norm_c(rtDW.DiscreteTimeIntegratory_DSTAT_j) < 0.3) {
        // :  direction = waypoints(:,wp_idx) - p;
        stage_app_2 = (rtDW.UnitDelay_DSTATE_b - 1) * 3;
        rtb_V_Kg_n0[0] = rtb_y_ku[stage_app_2] -
          rtDW.DiscreteTimeIntegratory_DSTA_n2[0];
        rtb_V_Kg_n0[1] = rtb_y_ku[stage_app_2 + 1] -
          rtDW.DiscreteTimeIntegratory_DSTA_n2[1];
        rtb_V_Kg_n0[2] = rtb_y_ku[stage_app_2 + 2] -
          rtDW.DiscreteTimeIntegratory_DSTA_n2[2];

        // :  direction = divideFinite(direction,norm(direction,2));
        rtb_Sum2_os = norm_c(rtb_V_Kg_n0);

        // 'divideFinite:29' if numel(B)>1
        // 'divideFinite:31' else
        // 'divideFinite:32' if abs(B)<eps
        if (std::abs(rtb_Sum2_os) < 2.22044605E-16F) {
          // 'divideFinite:33' B(:) = eps;
          rtb_Sum2_os = 2.22044605E-16F;
        }

        // 'divideFinite:36' C = A ./ B;
        // :  V_Kg_n0(:) = 0.3 *  direction;
        rtb_V_Kg_n0[0] = (rtb_y_ku[stage_app_2] -
                          rtDW.DiscreteTimeIntegratory_DSTA_n2[0]) / rtb_Sum2_os
          * 0.3F;
        rtb_V_Kg_n0[1] = (rtb_y_ku[stage_app_2 + 1] -
                          rtDW.DiscreteTimeIntegratory_DSTA_n2[1]) / rtb_Sum2_os
          * 0.3F;
        rtb_V_Kg_n0[2] = (rtb_y_ku[stage_app_2 + 2] -
                          rtDW.DiscreteTimeIntegratory_DSTA_n2[2]) / rtb_Sum2_os
          * 0.3F;
      }

      // DataTypeConversion: '<S37>/Data Type Conversion2' incorporates:
      //   Inport: '<Root>/cmd'

      p3 = rtU.cmd.num_waypoints;

      // MATLAB Function: '<S37>/Look Ahead' incorporates:
      //   Constant: '<S37>/Constant3'
      //   Constant: '<S37>/Constant5'
      //   MATLAB Function: '<S37>/Look Ahead1'

      // :  p_ahead = p + v*Delta_t + 0.5*a*Delta_t*Delta_t;
      q1_q3 = (2.0F / (rtP.lindi.servo.boost * rtP.lindi.servo.omega) + 2.0F /
               rtP.lindi.atc.rm.rfreq) + 2.0F / rtP.lindi.sflt.omega;
      scale = q1_q3 + rtP.lindi.wpnav.T;

      // MATLAB Function: '<S37>/WpNav Matching' incorporates:
      //   UnitDelay: '<S37>/Unit Delay'
      //   UnitDelay: '<S37>/Unit Delay1'
      //   UnitDelay: '<S37>/Unit Delay2'
      //   UnitDelay: '<S37>/Unit Delay3'
      //   UnitDelay: '<S37>/Unit Delay5'

      // :  if wp_idx == 1
      // :  if stage == 0
      c_stage_app = rtDW.UnitDelay3_DSTATE;
      itmp = rtDW.UnitDelay2_DSTATE;
      is_approach = rtDW.UnitDelay5_DSTATE;
      p2 = rtDW.UnitDelay1_DSTATE_g;
      i_0 = rtDW.UnitDelay_DSTATE_b;

      // MATLAB Function: '<S37>/Look Ahead' incorporates:
      //   Constant: '<S37>/Constant5'
      //   DiscreteIntegrator: '<S49>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y'
      //   UnitDelay: '<S37>/Unit Delay6'

      // :  a = zeros(3,1,superiorfloat(waypoints));
      rtb_p_ahead_i[0] = 0.5F * rtDW.DiscreteTimeIntegratory_DSTAT_n[0] * scale *
        scale + (rtDW.DiscreteTimeIntegratory_DSTAT_j[0] * scale +
                 rtDW.UnitDelay6_DSTATE[0]);

      // MATLAB Function: '<S37>/WpNav Matching'
      rtb_Sum2_g[0] = 0.0F;

      // MATLAB Function: '<S37>/Look Ahead' incorporates:
      //   Constant: '<S37>/Constant5'
      //   DiscreteIntegrator: '<S49>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y'
      //   UnitDelay: '<S37>/Unit Delay6'

      rtb_p_ahead_i[1] = 0.5F * rtDW.DiscreteTimeIntegratory_DSTAT_n[1] * scale *
        scale + (rtDW.DiscreteTimeIntegratory_DSTAT_j[1] * scale +
                 rtDW.UnitDelay6_DSTATE[1]);

      // MATLAB Function: '<S37>/WpNav Matching'
      rtb_Sum2_g[1] = 0.0F;

      // MATLAB Function: '<S37>/Look Ahead' incorporates:
      //   Constant: '<S37>/Constant5'
      //   DiscreteIntegrator: '<S49>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y'
      //   UnitDelay: '<S37>/Unit Delay6'

      rtb_p_ahead_i[2] = 0.5F * rtDW.DiscreteTimeIntegratory_DSTAT_n[2] * scale *
        scale + (rtDW.DiscreteTimeIntegratory_DSTAT_j[2] * scale +
                 rtDW.UnitDelay6_DSTATE[2]);

      // MATLAB Function: '<S37>/WpNav Matching' incorporates:
      //   Constant: '<S144>/wp_rad_fix'
      //   Constant: '<S37>/Constant2'
      //   DataTypeConversion: '<S37>/Data Type Conversion2'
      //   DiscreteIntegrator: '<S50>/Discrete-Time Integrator y'
      //   Inport: '<Root>/cmd'
      //   MATLAB Function: '<S37>/Split waypoints and velocity'
      //   SignalConversion: '<S37>/BusConversion_InsertedFor_wpnav_at_inport_0'
      //   Switch: '<S144>/Switch'
      //   UnitDelay: '<S37>/Unit Delay'
      //   UnitDelay: '<S37>/Unit Delay1'
      //   UnitDelay: '<S37>/Unit Delay2'
      //   UnitDelay: '<S37>/Unit Delay3'
      //   UnitDelay: '<S37>/Unit Delay4'
      //   UnitDelay: '<S37>/Unit Delay5'

      rtb_Sum2_g[2] = 0.0F;

      // :  v = zeros(3,1,superiorfloat(waypoints));
      // :  wp_approach_out = zeros(3,5,superiorfloat(waypoints));
      // :  wp_approach_out(:) = wp_approach;
      for (stage_app_2 = 0; stage_app_2 < 15; stage_app_2++) {
        wp_approach_out[stage_app_2] = rtDW.UnitDelay4_DSTATE[stage_app_2];
      }

      // :  num_wp = max(num_wp,2);
      if (rtU.cmd.num_waypoints < 2) {
        p3 = 2;
      }

      // :  if is_approach
      if (rtDW.UnitDelay5_DSTATE) {
        // :  [p_match,wp_idx_app,stage_app,t,d] = wpnavMatch(wp_approach_out,wp_radius,wp_idx_app,stage_app,p); 
        itmp = rtDW.UnitDelay2_DSTATE;
        c_stage_app = rtDW.UnitDelay3_DSTATE;

        // SignalConversion: '<S37>/BusConversion_InsertedFor_wpnav_at_inport_0' incorporates:
        //   Constant: '<S144>/wp_rad_fix'
        //   Switch: '<S144>/Switch'
        //   UnitDelay: '<S37>/Unit Delay2'
        //   UnitDelay: '<S37>/Unit Delay3'
        //   UnitDelay: '<S37>/Unit Delay4'

        wpnavMatch(rtDW.UnitDelay4_DSTATE, rtP.lindi.wpnav.wprad, &itmp,
                   &c_stage_app, rtDW.DiscreteTimeIntegratory_DSTA_n2, rtb_y_fg,
                   &rtDW.t_l, &q2_q2);

        // :  [p_match_2,wp_idx_app_2,stage_app_2,t_2,d_2] = wpnavMatch(wp_approach_out,wp_radius,wp_idx_app,stage_app,p_ahead); 
        wp_idx_app_2 = itmp;
        stage_app_2 = c_stage_app;
        wpnavMatch(rtDW.UnitDelay4_DSTATE, rtP.lindi.wpnav.wprad, &wp_idx_app_2,
                   &stage_app_2, rtb_p_ahead_i, rtb_Sum2_cx, &q2_q2, &scale);

        // :  stage_2 = stage_;
        stage_2 = rtDW.UnitDelay1_DSTATE_g;

        // :  wp_idx_2 = wp_idx;
        wp_idx_2 = rtDW.UnitDelay_DSTATE_b;

        // :  stage_tmp = stage_;
        stage_tmp = rtDW.UnitDelay1_DSTATE_g;

        // :  wp_idx_tmp = wp_idx;
        wp_idx_tmp = rtDW.UnitDelay_DSTATE_b;
      } else {
        // :  else
        // :  [p_match,wp_idx_tmp,stage_tmp,t,d] = wpnavMatch(waypoints(:,1:num_wp),wp_radius,wp_idx,stage_,p); 
        wp_idx_tmp = rtDW.UnitDelay_DSTATE_b;
        stage_tmp = rtDW.UnitDelay1_DSTATE_g;
        rtb_y_g_size_3[0] = 3;
        rtb_y_g_size_3[1] = p3;
        for (stage_app_2 = 0; stage_app_2 < p3; stage_app_2++) {
          c_XYZ[3 * stage_app_2] = rtb_y_ku[3 * stage_app_2];
          c_XYZ[1 + 3 * stage_app_2] = rtb_y_ku[3 * stage_app_2 + 1];
          c_XYZ[2 + 3 * stage_app_2] = rtb_y_ku[3 * stage_app_2 + 2];
        }

        wpnavMatch_p(c_XYZ, rtb_y_g_size_3, rtP.lindi.wpnav.wprad, &wp_idx_tmp,
                     &stage_tmp, rtDW.DiscreteTimeIntegratory_DSTA_n2, rtb_y_fg,
                     &rtDW.t_l, &q2_q2);

        // :  [p_match_2,wp_idx_2,stage_2,t_2,d_2] = wpnavMatch(waypoints(:,1:num_wp),wp_radius,wp_idx,stage_,p_ahead); 
        wp_idx_2 = rtDW.UnitDelay_DSTATE_b;
        stage_2 = rtDW.UnitDelay1_DSTATE_g;
        rtb_y_g_size_2[0] = 3;
        rtb_y_g_size_2[1] = p3;
        for (stage_app_2 = 0; stage_app_2 < p3; stage_app_2++) {
          c_XYZ[3 * stage_app_2] = rtb_y_ku[3 * stage_app_2];
          c_XYZ[1 + 3 * stage_app_2] = rtb_y_ku[3 * stage_app_2 + 1];
          c_XYZ[2 + 3 * stage_app_2] = rtb_y_ku[3 * stage_app_2 + 2];
        }

        wpnavMatch_p(c_XYZ, rtb_y_g_size_2, rtP.lindi.wpnav.wprad, &wp_idx_2,
                     &stage_2, rtb_p_ahead_i, rtb_Sum2_cx, &q2_q2, &scale);

        // :  stage_app_2 = stage_app;
        stage_app_2 = rtDW.UnitDelay3_DSTATE;

        // :  wp_idx_app_2 = wp_idx_app;
        wp_idx_app_2 = rtDW.UnitDelay2_DSTATE;
      }

      // :  e_pos = norm(p_match-p,2);
      // :  if e_pos > e_pos_max
      rtb_y_a5[0] = rtb_y_fg[0] - rtDW.DiscreteTimeIntegratory_DSTA_n2[0];
      rtb_y_a5[1] = rtb_y_fg[1] - rtDW.DiscreteTimeIntegratory_DSTA_n2[1];
      rtb_y_a5[2] = rtb_y_fg[2] - rtDW.DiscreteTimeIntegratory_DSTA_n2[2];
      if (norm_c(rtb_y_a5) > rtP.lindi.wpnav.eposmax) {
        // :  dir_next_wp = waypoints(:,wp_idx) - p;
        rtb_y_fg[0] = rtb_y_ku[(rtDW.UnitDelay_DSTATE_b - 1) * 3] -
          rtDW.DiscreteTimeIntegratory_DSTA_n2[0];
        rtb_y_fg[1] = rtb_y_ku[(rtDW.UnitDelay_DSTATE_b - 1) * 3 + 1] -
          rtDW.DiscreteTimeIntegratory_DSTA_n2[1];
        rtb_y_fg[2] = rtb_y_ku[(rtDW.UnitDelay_DSTATE_b - 1) * 3 + 2] -
          rtDW.DiscreteTimeIntegratory_DSTA_n2[2];

        // :  dist_next_wp = norm( dir_next_wp );
        absxk = norm_c(rtb_y_fg);

        // :  flight_dir = divideFinite( V_Kg, norm(V_Kg,2) );
        q2_q2 = norm_c(rtb_V_Kg_n0);

        // 'divideFinite:29' if numel(B)>1
        // 'divideFinite:31' else
        // 'divideFinite:32' if abs(B)<eps
        if (std::abs(q2_q2) < 2.22044605E-16F) {
          // 'divideFinite:33' B(:) = eps;
          q2_q2 = 2.22044605E-16F;
        }

        // 'divideFinite:36' C = A ./ B;
        rtb_V_Kg_n0[0] /= q2_q2;
        rtb_V_Kg_n0[1] /= q2_q2;
        scale = rtb_V_Kg_n0[2] / q2_q2;
        rtb_V_Kg_n0[2] = scale;

        // :  angle_next_wp = acosReal( divideFinite( dot( dir_next_wp, flight_dir ), norm(dir_next_wp,2)*norm(flight_dir,2) ) ); 
        q1_q2 = absxk * norm_c(rtb_V_Kg_n0);

        // 'divideFinite:29' if numel(B)>1
        // 'divideFinite:31' else
        // 'divideFinite:32' if abs(B)<eps
        if (std::abs(q1_q2) < 2.22044605E-16F) {
          // 'divideFinite:33' B(:) = eps;
          q1_q2 = 2.22044605E-16F;
        }

        // 'divideFinite:36' C = A ./ B;
        q1_q2 = (((rtb_y_ku[(rtDW.UnitDelay_DSTATE_b - 1) * 3] -
                   rtDW.DiscreteTimeIntegratory_DSTA_n2[0]) * rtb_V_Kg_n0[0] +
                  (rtb_y_ku[(rtDW.UnitDelay_DSTATE_b - 1) * 3 + 1] -
                   rtDW.DiscreteTimeIntegratory_DSTA_n2[1]) * rtb_V_Kg_n0[1]) +
                 (rtb_y_ku[(rtDW.UnitDelay_DSTATE_b - 1) * 3 + 2] -
                  rtDW.DiscreteTimeIntegratory_DSTA_n2[2]) * scale) / q1_q2;

        // 'acosReal:28' if numel(y) > 1
        // 'acosReal:31' else
        // 'acosReal:32' y = max(-1,min(1,y));
        if (1.0F <= q1_q2) {
          q1_q2 = 1.0F;
        }

        // 'acosReal:34' y = acos(y);
        // :  if angle_next_wp < 0.5 && dist_next_wp < wp_radius
        if (-1.0F >= q1_q2) {
          q1_q2 = -1.0F;
        }

        if ((std::acos(q1_q2) < 0.5F) && (absxk < rtP.lindi.wpnav.wprad)) {
          // :  dist_app_wp = 0.5 * dist_next_wp;
          q2_q2 = 0.5F * absxk;
        } else {
          // :  else
          // :  dist_app_wp = 1.2*wp_radius;
          q2_q2 = 1.2F * rtP.lindi.wpnav.wprad;
        }

        // :  wp_approach_out(:,1) = p - flight_dir * 1 * dist_app_wp;
        // :  wp_approach_out(:,2) = p + flight_dir * 1 * dist_app_wp;
        // :  wp_approach_out(:,3) = waypoints(:,wp_idx);
        q1_q2 = rtb_V_Kg_n0[0] * q2_q2;
        wp_approach_out[0] = rtDW.DiscreteTimeIntegratory_DSTA_n2[0] - q1_q2;
        wp_approach_out[3] = rtDW.DiscreteTimeIntegratory_DSTA_n2[0] + q1_q2;
        wp_approach_out[6] = rtb_y_ku[(rtDW.UnitDelay_DSTATE_b - 1) * 3];
        q1_q2 = rtb_V_Kg_n0[1] * q2_q2;
        wp_approach_out[1] = rtDW.DiscreteTimeIntegratory_DSTA_n2[1] - q1_q2;
        wp_approach_out[4] = rtDW.DiscreteTimeIntegratory_DSTA_n2[1] + q1_q2;
        wp_approach_out[7] = rtb_y_ku[(rtDW.UnitDelay_DSTATE_b - 1) * 3 + 1];
        q1_q2 = scale * q2_q2;
        wp_approach_out[2] = rtDW.DiscreteTimeIntegratory_DSTA_n2[2] - q1_q2;
        wp_approach_out[5] = rtDW.DiscreteTimeIntegratory_DSTA_n2[2] + q1_q2;
        wp_approach_out[8] = rtb_y_ku[(rtDW.UnitDelay_DSTATE_b - 1) * 3 + 2];

        // :  if wp_idx == num_wp-1
        if (p3 - 1 == rtDW.UnitDelay_DSTATE_b) {
          // :  wp_approach_out(:,4) = waypoints(:,wp_idx);
          // :  wp_approach_out(:,5) = waypoints(:,1);
          wp_approach_out[9] = rtb_y_ku[(rtDW.UnitDelay_DSTATE_b - 1) * 3];
          wp_approach_out[12] = rtb_y_ku[0];
          wp_approach_out[10] = rtb_y_ku[(rtDW.UnitDelay_DSTATE_b - 1) * 3 + 1];
          wp_approach_out[13] = rtb_y_ku[1];
          wp_approach_out[11] = rtb_y_ku[(rtDW.UnitDelay_DSTATE_b - 1) * 3 + 2];
          wp_approach_out[14] = rtb_y_ku[2];
        } else if (rtDW.UnitDelay_DSTATE_b == p3) {
          // :  elseif wp_idx == num_wp
          // :  wp_approach_out(:,4) = waypoints(:,1);
          // :  wp_approach_out(:,5) = waypoints(:,2);
          wp_approach_out[9] = rtb_y_ku[0];
          wp_approach_out[12] = rtb_y_ku[3];
          wp_approach_out[10] = rtb_y_ku[1];
          wp_approach_out[13] = rtb_y_ku[4];
          wp_approach_out[11] = rtb_y_ku[2];
          wp_approach_out[14] = rtb_y_ku[5];
        } else {
          // :  else
          // :  wp_approach_out(:,4) = waypoints(:,wp_idx+1);
          if (rtDW.UnitDelay_DSTATE_b > 2147483646) {
            wp_idx_2 = MAX_int32_T;
          } else {
            wp_idx_2 = rtDW.UnitDelay_DSTATE_b + 1;
          }

          // :  wp_approach_out(:,5) = waypoints(:,wp_idx+2);
          if (rtDW.UnitDelay_DSTATE_b > 2147483645) {
            stage_app_2 = MAX_int32_T;
          } else {
            stage_app_2 = rtDW.UnitDelay_DSTATE_b + 2;
          }

          itmp = (wp_idx_2 - 1) * 3;
          wp_approach_out[9] = rtb_y_ku[itmp];
          stage_app_2 = (stage_app_2 - 1) * 3;
          wp_approach_out[12] = rtb_y_ku[stage_app_2];
          wp_approach_out[10] = rtb_y_ku[itmp + 1];
          wp_approach_out[13] = rtb_y_ku[stage_app_2 + 1];
          wp_approach_out[11] = rtb_y_ku[itmp + 2];
          wp_approach_out[14] = rtb_y_ku[stage_app_2 + 2];
        }

        // :  is_approach(:) = 1;
        is_approach = true;

        // :  wp_idx_app(:) = 2;
        // :  stage_app(:) = 1;
        // :  [p_match,wp_idx_app,stage_app,t,d] = wpnavMatch(wp_approach_out,wp_radius,wp_idx_app,stage_app,p); 
        itmp = 2;
        c_stage_app = 1;

        // SignalConversion: '<S37>/BusConversion_InsertedFor_wpnav_at_inport_0' incorporates:
        //   Constant: '<S144>/wp_rad_fix'
        //   MATLAB Function: '<S37>/Split waypoints and velocity'
        //   Switch: '<S144>/Switch'
        //   UnitDelay: '<S37>/Unit Delay'

        wpnavMatch(wp_approach_out, rtP.lindi.wpnav.wprad, &itmp, &c_stage_app,
                   rtDW.DiscreteTimeIntegratory_DSTA_n2, rtb_y_fg, &rtDW.t_l,
                   &q2_q2);

        // :  [p_match_2,wp_idx_app_2,stage_app_2,t_2,d_2] = wpnavMatch(wp_approach_out,wp_radius,wp_idx_app,stage_app,p_ahead); 
        wp_idx_app_2 = itmp;
        stage_app_2 = c_stage_app;
        wpnavMatch(wp_approach_out, rtP.lindi.wpnav.wprad, &wp_idx_app_2,
                   &stage_app_2, rtb_p_ahead_i, rtb_Sum2_cx, &q2_q2, &scale);

        // :  stage_2 = stage_;
        stage_2 = rtDW.UnitDelay1_DSTATE_g;

        // :  wp_idx_2 = wp_idx;
        wp_idx_2 = rtDW.UnitDelay_DSTATE_b;
      } else {
        // :  else
        // :  wp_idx = wp_idx_tmp;
        i_0 = wp_idx_tmp;

        // :  stage_ = stage_tmp;
        p2 = stage_tmp;
      }

      // :  if (wp_idx_app == 4 && stage_app == 1) || wp_idx_app == 5
      if (((itmp == 4) && (c_stage_app == 1)) || (itmp == 5)) {
        // :  wp_idx_app(:) = 2;
        itmp = 2;

        // :  is_approach(:) = 0;
        is_approach = false;

        // :  wp_idx_new = wp_idx + 1;
        if (i_0 > 2147483646) {
          wp_idx_2 = MAX_int32_T;
        } else {
          wp_idx_2 = i_0 + 1;
        }

        // :  stage_(:) = 1;
        // :  [p_match,wp_idx,stage_,t,d] = wpnavMatch(waypoints(:,1:num_wp),wp_radius,wp_idx_new,stage_,p); 
        i_0 = wp_idx_2;
        p2 = 1;
        rtb_y_g_size_1[0] = 3;
        rtb_y_g_size_1[1] = p3;
        for (stage_app_2 = 0; stage_app_2 < p3; stage_app_2++) {
          c_XYZ[3 * stage_app_2] = rtb_y_ku[3 * stage_app_2];
          c_XYZ[1 + 3 * stage_app_2] = rtb_y_ku[3 * stage_app_2 + 1];
          c_XYZ[2 + 3 * stage_app_2] = rtb_y_ku[3 * stage_app_2 + 2];
        }

        wpnavMatch_p(c_XYZ, rtb_y_g_size_1, rtP.lindi.wpnav.wprad, &i_0, &p2,
                     rtDW.DiscreteTimeIntegratory_DSTA_n2, rtb_y_fg, &rtDW.t_l,
                     &q2_q2);

        // :  if wp_idx ~= wp_idx_new
        if (i_0 != wp_idx_2) {
          // :  [p_match,wp_idx,stage_,t,d] = wpnavMatch(waypoints(:,1:num_wp),wp_radius,wp_idx_new-1,stage_,p); 
          i_0 = wp_idx_2 - 1;
          rtb_y_g_size_0[0] = 3;
          rtb_y_g_size_0[1] = p3;
          for (stage_app_2 = 0; stage_app_2 < p3; stage_app_2++) {
            c_XYZ[3 * stage_app_2] = rtb_y_ku[3 * stage_app_2];
            c_XYZ[1 + 3 * stage_app_2] = rtb_y_ku[3 * stage_app_2 + 1];
            c_XYZ[2 + 3 * stage_app_2] = rtb_y_ku[3 * stage_app_2 + 2];
          }

          wpnavMatch_p(c_XYZ, rtb_y_g_size_0, rtP.lindi.wpnav.wprad, &i_0, &p2,
                       rtDW.DiscreteTimeIntegratory_DSTA_n2, rtb_y_fg, &rtDW.t_l,
                       &q2_q2);
        }

        // :  [p_match_2,wp_idx_2,stage_2,t_2,d_2] = wpnavMatch(waypoints(:,1:num_wp),wp_radius,wp_idx,stage_,p_ahead); 
        wp_idx_2 = i_0;
        stage_2 = p2;
        rtb_y_g_size[0] = 3;
        rtb_y_g_size[1] = p3;
        for (stage_app_2 = 0; stage_app_2 < p3; stage_app_2++) {
          c_XYZ[3 * stage_app_2] = rtb_y_ku[3 * stage_app_2];
          c_XYZ[1 + 3 * stage_app_2] = rtb_y_ku[3 * stage_app_2 + 1];
          c_XYZ[2 + 3 * stage_app_2] = rtb_y_ku[3 * stage_app_2 + 2];
        }

        wpnavMatch_p(c_XYZ, rtb_y_g_size, rtP.lindi.wpnav.wprad, &wp_idx_2,
                     &stage_2, rtb_p_ahead_i, rtb_Sum2_cx, &q2_q2, &scale);

        // :  stage_app_2 = stage_app;
        stage_app_2 = c_stage_app;

        // :  wp_idx_app_2 = wp_idx_app;
        wp_idx_app_2 = 2;
      }

      // :  prev_wp_idx = wp_idx-1;
      // :  if prev_wp_idx <= 0
      // :  if stage_ == 0
      // :  if (stage_2 == 0 && ~is_approach) || (stage_app_2 == 0 && is_approach) 
      if (((stage_2 == 0) && (!is_approach)) || ((stage_app_2 == 0) &&
           is_approach)) {
        // :  if ~is_approach
        if (!is_approach) {
          // :  if wp_idx_2 > 2
          if (wp_idx_2 > 2) {
            // :  waypoints3x3 = waypoints(:,wp_idx_2-2:wp_idx_2);
            wp_idx_2 -= 3;
            for (stage_app_2 = 0; stage_app_2 < 3; stage_app_2++) {
              p3 = (stage_app_2 + wp_idx_2) * 3;
              I_b[3 * stage_app_2] = rtb_y_ku[p3];
              I_b[1 + 3 * stage_app_2] = rtb_y_ku[p3 + 1];
              I_b[2 + 3 * stage_app_2] = rtb_y_ku[p3 + 2];
            }
          } else if (wp_idx_2 == 2) {
            // :  elseif wp_idx_2 == 2
            // :  waypoints3x3 = waypoints(:,[num_wp,1,2]);
            p3_0[0] = (uint16_T)(p3 - 1);
            p3_0[1] = 0U;
            p3_0[2] = 1U;
            for (stage_app_2 = 0; stage_app_2 < 3; stage_app_2++) {
              p3 = 3 * p3_0[stage_app_2];
              I_b[3 * stage_app_2] = rtb_y_ku[p3];
              I_b[1 + 3 * stage_app_2] = rtb_y_ku[p3 + 1];
              I_b[2 + 3 * stage_app_2] = rtb_y_ku[p3 + 2];
            }
          } else {
            // :  else
            // :  waypoints3x3 = waypoints(:,[num_wp-1,num_wp,1]);
            p3_0[0] = (uint16_T)(p3 - 2);
            p3_0[1] = (uint16_T)(p3 - 1);
            p3_0[2] = 0U;
            for (stage_app_2 = 0; stage_app_2 < 3; stage_app_2++) {
              p3 = 3 * p3_0[stage_app_2];
              I_b[3 * stage_app_2] = rtb_y_ku[p3];
              I_b[1 + 3 * stage_app_2] = rtb_y_ku[p3 + 1];
              I_b[2 + 3 * stage_app_2] = rtb_y_ku[p3 + 2];
            }
          }
        } else {
          // :  else
          // :  if wp_idx_app_2 > 2
          if (wp_idx_app_2 > 2) {
            // :  waypoints3x3 = wp_approach_out(:,wp_idx_app_2-2:wp_idx_app_2); 
            wp_idx_app_2 -= 3;
            for (stage_app_2 = 0; stage_app_2 < 3; stage_app_2++) {
              p3 = (stage_app_2 + wp_idx_app_2) * 3;
              I_b[3 * stage_app_2] = wp_approach_out[p3];
              I_b[1 + 3 * stage_app_2] = wp_approach_out[p3 + 1];
              I_b[2 + 3 * stage_app_2] = wp_approach_out[p3 + 2];
            }
          } else if (wp_idx_app_2 == 2) {
            // :  elseif wp_idx_app_2 == 2
            // :  waypoints3x3 = wp_approach_out(:,[end,1,2]);
            for (stage_app_2 = 0; stage_app_2 < 3; stage_app_2++) {
              p3 = 3 * i[stage_app_2];
              I_b[3 * stage_app_2] = wp_approach_out[p3];
              I_b[1 + 3 * stage_app_2] = wp_approach_out[p3 + 1];
              I_b[2 + 3 * stage_app_2] = wp_approach_out[p3 + 2];
            }
          } else {
            // :  else
            // :  waypoints3x3 = wp_approach_out(:,[end-1,end,1]);
            for (stage_app_2 = 0; stage_app_2 < 3; stage_app_2++) {
              p3 = 3 * h[stage_app_2];
              I_b[3 * stage_app_2] = wp_approach_out[p3];
              I_b[1 + 3 * stage_app_2] = wp_approach_out[p3 + 1];
              I_b[2 + 3 * stage_app_2] = wp_approach_out[p3 + 2];
            }
          }
        }

        // :  circ_seg = wpnavCircSeg(waypoints3x3,wp_radius);
        wpnavCircSeg(I_b, rtP.lindi.wpnav.wprad, &scale, rtb_Sum2_p, circ_seg_n,
                     &absxk, circ_seg_start, rtb_Sum2_g, &t, rtb_p_ahead_i);

        // :  v(:) = wpnavCircSegGetVel(circ_seg,t_2,V_K);
        // 'wpnavCircSegGetVel:27' tangent_vec_p1_unit = cross( circ_seg.n, circ_seg.start-circ_seg.center ); 
        circ_seg_start[0] -= rtb_Sum2_p[0];
        circ_seg_start[1] -= rtb_Sum2_p[1];
        q1_q2 = circ_seg_start[2] - rtb_Sum2_p[2];
        circ_seg_start[2] = q1_q2;
        rtb_V_Kg_n0[0] = circ_seg_n[1] * q1_q2 - circ_seg_n[2] * circ_seg_start
          [1];
        rtb_V_Kg_n0[1] = circ_seg_n[2] * circ_seg_start[0] - circ_seg_n[0] *
          q1_q2;
        rtb_V_Kg_n0[2] = circ_seg_n[0] * circ_seg_start[1] - circ_seg_n[1] *
          circ_seg_start[0];

        // 'wpnavCircSegGetVel:28' tangent_vec_p1_unit = divideFinite( tangent_vec_p1_unit, norm( tangent_vec_p1_unit, 2 ) ); 
        t = norm_c(rtb_V_Kg_n0);

        // 'divideFinite:29' if numel(B)>1
        // 'divideFinite:31' else
        // 'divideFinite:32' if abs(B)<eps
        if (std::abs(t) < 2.22044605E-16F) {
          // 'divideFinite:33' B(:) = eps;
          t = 2.22044605E-16F;
        }

        // 'divideFinite:36' C = A ./ B;
        // 'wpnavCircSegGetVel:30' angle = t * circ_seg.angle;
        // 'wpnavCircSegGetVel:31' tangent_vec_t_unit = axisAngle(tangent_vec_p1_unit,circ_seg.n,angle); 
        // 'wpnavCircSegGetVel:33' V_Kg = tangent_vec_t_unit * V;
        circ_seg_n_0[0] = circ_seg_n[0];
        rtb_V_Kg_n0[0] /= t;
        circ_seg_n_0[1] = circ_seg_n[1];
        rtb_V_Kg_n0[1] /= t;
        circ_seg_n_0[2] = circ_seg_n[2];
        rtb_V_Kg_n0[2] /= t;
        rtb_Sum2_ms = q2_q2 * absxk;
        axisAngle(rtb_V_Kg_n0, circ_seg_n_0, rtb_Sum2_ms, rtb_y_a5);

        // :  a(:) = wpnavCircSegGetAcc(circ_seg,t_2,V_K);
        // 'wpnavCircSegGetAcc:29' p = wpnavCircSegGetPos( circ_seg, t );
        // 'wpnavCircSegGetPos:26' angle = t * circ_seg.angle;
        // 'wpnavCircSegGetPos:27' s_g = circ_seg.center + axisAngle(circ_seg.start-circ_seg.center,circ_seg.n,angle); 
        // 'wpnavCircSegGetAcc:30' dir_vec_unit = circ_seg.center - p;
        axisAngle(circ_seg_start, circ_seg_n, rtb_Sum2_ms, circ_seg_n_0);
        rtb_p_ahead_i[0] = rtb_y_a5[0] * q1_q1;
        rtb_Sum2_g[0] = rtb_Sum2_p[0] - (rtb_Sum2_p[0] + circ_seg_n_0[0]);
        rtb_p_ahead_i[1] = rtb_y_a5[1] * q1_q1;
        rtb_Sum2_g[1] = rtb_Sum2_p[1] - (rtb_Sum2_p[1] + circ_seg_n_0[1]);
        rtb_p_ahead_i[2] = rtb_y_a5[2] * q1_q1;
        rtb_Sum2_g[2] = rtb_Sum2_p[2] - (rtb_Sum2_p[2] + circ_seg_n_0[2]);

        // 'wpnavCircSegGetAcc:31' dir_vec_unit = divideFinite( dir_vec_unit, norm( dir_vec_unit, 2 ) ); 
        q2_q2 = norm_c(rtb_Sum2_g);

        // 'divideFinite:29' if numel(B)>1
        // 'divideFinite:31' else
        // 'divideFinite:32' if abs(B)<eps
        if (std::abs(q2_q2) < 2.22044605E-16F) {
          // 'divideFinite:33' B(:) = eps;
          q2_q2 = 2.22044605E-16F;
        }

        // 'divideFinite:36' C = A ./ B;
        rtb_Sum2_g[0] /= q2_q2;
        rtb_Sum2_g[1] /= q2_q2;
        rtb_Sum2_g[2] /= q2_q2;

        // 'wpnavCircSegGetAcc:32' a_Kg = divideFinite( V*V, circ_seg.r ) * dir_vec_unit; 
        // 'divideFinite:29' if numel(B)>1
        // 'divideFinite:31' else
        // 'divideFinite:32' if abs(B)<eps
        if (std::abs(scale) < 2.22044605E-16F) {
          // 'divideFinite:33' B(:) = eps;
          scale = 2.22044605E-16F;
        }

        // 'divideFinite:36' C = A ./ B;
        q1_q2 = q1_q1 * q1_q1 / scale;
        rtb_Sum2_g[0] *= q1_q2;
        rtb_Sum2_g[1] *= q1_q2;
        rtb_Sum2_g[2] *= q1_q2;
      } else {
        // :  else
        // :  if ~is_approach
        if (!is_approach) {
          // :  if wp_idx_2 > 1
          if (wp_idx_2 > 1) {
            // :  wp1 = waypoints(:,wp_idx_2-1);
            // :  wp2 = waypoints(:,wp_idx_2);
            stage_app_2 = (wp_idx_2 - 2) * 3;
            rtb_V_Kg_n0[0] = rtb_y_ku[stage_app_2];
            p3 = (wp_idx_2 - 1) * 3;
            rtb_p_ahead_i[0] = rtb_y_ku[p3];
            rtb_V_Kg_n0[1] = rtb_y_ku[stage_app_2 + 1];
            rtb_p_ahead_i[1] = rtb_y_ku[p3 + 1];
            rtb_V_Kg_n0[2] = rtb_y_ku[stage_app_2 + 2];
            rtb_p_ahead_i[2] = rtb_y_ku[p3 + 2];
          } else {
            // :  else
            // :  wp1 = waypoints(:,num_wp);
            // :  wp2 = waypoints(:,1);
            stage_app_2 = (p3 - 1) * 3;
            rtb_V_Kg_n0[0] = rtb_y_ku[stage_app_2];
            rtb_p_ahead_i[0] = rtb_y_ku[0];
            rtb_V_Kg_n0[1] = rtb_y_ku[stage_app_2 + 1];
            rtb_p_ahead_i[1] = rtb_y_ku[1];
            rtb_V_Kg_n0[2] = rtb_y_ku[stage_app_2 + 2];
            rtb_p_ahead_i[2] = rtb_y_ku[2];
          }
        } else {
          // :  else
          // :  wp1 = wp_approach_out(:,wp_idx_app_2-1);
          if (wp_idx_app_2 < -2147483647) {
            wp_idx_2 = MIN_int32_T;
          } else {
            wp_idx_2 = wp_idx_app_2 - 1;
          }

          // :  wp2 = wp_approach_out(:,wp_idx_app_2);
          stage_app_2 = (wp_idx_2 - 1) * 3;
          rtb_V_Kg_n0[0] = wp_approach_out[stage_app_2];
          p3 = (wp_idx_app_2 - 1) * 3;
          rtb_p_ahead_i[0] = wp_approach_out[p3];
          rtb_V_Kg_n0[1] = wp_approach_out[stage_app_2 + 1];
          rtb_p_ahead_i[1] = wp_approach_out[p3 + 1];
          rtb_V_Kg_n0[2] = wp_approach_out[stage_app_2 + 2];
          rtb_p_ahead_i[2] = wp_approach_out[p3 + 2];
        }

        // :  v(:) = wpnavLineGetVel(wp1,wp2,V_K);
        // 'wpnavLineGetVel:28' dir_vec_unit = p2-p1;
        rtb_p_ahead_i[0] -= rtb_V_Kg_n0[0];
        rtb_p_ahead_i[1] -= rtb_V_Kg_n0[1];
        rtb_p_ahead_i[2] -= rtb_V_Kg_n0[2];

        // 'wpnavLineGetVel:29' dir_vec_unit = divideFinite( dir_vec_unit, norm( dir_vec_unit, 2 ) ); 
        q2_q2 = norm_c(rtb_p_ahead_i);

        // 'divideFinite:29' if numel(B)>1
        // 'divideFinite:31' else
        // 'divideFinite:32' if abs(B)<eps
        if (std::abs(q2_q2) < 2.22044605E-16F) {
          // 'divideFinite:33' B(:) = eps;
          q2_q2 = 2.22044605E-16F;
        }

        // 'divideFinite:36' C = A ./ B;
        // 'wpnavLineGetVel:30' V_Kg = V * dir_vec_unit;
        rtb_p_ahead_i[0] = rtb_p_ahead_i[0] / q2_q2 * q1_q1;
        rtb_p_ahead_i[1] = rtb_p_ahead_i[1] / q2_q2 * q1_q1;
        rtb_p_ahead_i[2] = rtb_p_ahead_i[2] / q2_q2 * q1_q1;
      }

      // Outputs for Enabled SubSystem: '<S140>/Flight Path Smoothing' incorporates:
      //   EnablePort: '<S147>/Enable'

      // :  p_ahead = p + v*Delta_t + 0.5*a*Delta_t*Delta_t;
      if (rtP.lindi.wpnav.T >= 0.05F) {
        if (!rtDW.FlightPathSmoothing_MODE) {
          // InitializeConditions for DiscreteIntegrator: '<S150>/Discrete-Time Integrator y' 
          rtDW.DiscreteTimeIntegratory_IC_L_io = 1U;

          // InitializeConditions for DiscreteIntegrator: '<S151>/Discrete-Time Integrator y' 
          rtDW.DiscreteTimeIntegratory_IC_L_o2 = 1U;

          // InitializeConditions for DiscreteIntegrator: '<S150>/Discrete-Time Integrator y_dt' 
          rtDW.DiscreteTimeIntegratory_dt_D_iv[0] = 0.0F;

          // InitializeConditions for DiscreteIntegrator: '<S151>/Discrete-Time Integrator y_dt' 
          rtDW.DiscreteTimeIntegratory_dt_D_ae[0] = 0.0F;

          // InitializeConditions for DiscreteIntegrator: '<S150>/Discrete-Time Integrator y_dt' 
          rtDW.DiscreteTimeIntegratory_dt_D_iv[1] = 0.0F;

          // InitializeConditions for DiscreteIntegrator: '<S151>/Discrete-Time Integrator y_dt' 
          rtDW.DiscreteTimeIntegratory_dt_D_ae[1] = 0.0F;

          // InitializeConditions for DiscreteIntegrator: '<S150>/Discrete-Time Integrator y_dt' 
          rtDW.DiscreteTimeIntegratory_dt_D_iv[2] = 0.0F;

          // InitializeConditions for DiscreteIntegrator: '<S151>/Discrete-Time Integrator y_dt' 
          rtDW.DiscreteTimeIntegratory_dt_D_ae[2] = 0.0F;
          rtDW.FlightPathSmoothing_MODE = true;
        }

        // DiscreteIntegrator: '<S150>/Discrete-Time Integrator y' incorporates:
        //   MATLAB Function: '<S37>/WpNav Matching'

        if (rtDW.DiscreteTimeIntegratory_IC_L_io != 0) {
          rtDW.DiscreteTimeIntegratory_DSTA_kr[0] = rtb_p_ahead_i[0];
          rtDW.DiscreteTimeIntegratory_DSTA_kr[1] = rtb_p_ahead_i[1];
          rtDW.DiscreteTimeIntegratory_DSTA_kr[2] = rtb_p_ahead_i[2];
        }

        // Product: '<S147>/Product1' incorporates:
        //   Constant: '<S147>/Constant'
        //   Constant: '<S147>/Constant1'

        q1_q2 = 2.0F / rtP.lindi.wpnav.T * 0.7071F;

        // Gain: '<S150>/Gain' incorporates:
        //   Constant: '<S147>/Constant1'
        //   Gain: '<S151>/Gain'
        //   Product: '<S150>/Divide'

        q1_q1 = 0.7071F / q1_q2 * 2.0F;

        // SignalConversion: '<S147>/OutportBufferFors_g_ref_dt_smooth' incorporates:
        //   DiscreteIntegrator: '<S150>/Discrete-Time Integrator y'

        rtDW.Merge1_p[0] = rtDW.DiscreteTimeIntegratory_DSTA_kr[0];

        // Sum: '<S150>/Sum2' incorporates:
        //   DiscreteIntegrator: '<S150>/Discrete-Time Integrator y'
        //   DiscreteIntegrator: '<S150>/Discrete-Time Integrator y_dt'
        //   Gain: '<S150>/Gain'
        //   MATLAB Function: '<S37>/WpNav Matching'
        //   Product: '<S150>/Product2'
        //   Sum: '<S150>/Sum3'

        rtb_V_Kg_n0[0] = rtb_p_ahead_i[0] -
          (rtDW.DiscreteTimeIntegratory_dt_D_iv[0] * q1_q1 +
           rtDW.DiscreteTimeIntegratory_DSTA_kr[0]);

        // SignalConversion: '<S147>/OutportBufferFors_g_ref_dt2_smooth' incorporates:
        //   DiscreteIntegrator: '<S150>/Discrete-Time Integrator y_dt'

        rtDW.Merge2_j[0] = rtDW.DiscreteTimeIntegratory_dt_D_iv[0];

        // SignalConversion: '<S147>/OutportBufferFors_g_ref_dt_smooth' incorporates:
        //   DiscreteIntegrator: '<S150>/Discrete-Time Integrator y'

        rtDW.Merge1_p[1] = rtDW.DiscreteTimeIntegratory_DSTA_kr[1];

        // Sum: '<S150>/Sum2' incorporates:
        //   DiscreteIntegrator: '<S150>/Discrete-Time Integrator y'
        //   DiscreteIntegrator: '<S150>/Discrete-Time Integrator y_dt'
        //   Gain: '<S150>/Gain'
        //   MATLAB Function: '<S37>/WpNav Matching'
        //   Product: '<S150>/Product2'
        //   Sum: '<S150>/Sum3'

        rtb_V_Kg_n0[1] = rtb_p_ahead_i[1] -
          (rtDW.DiscreteTimeIntegratory_dt_D_iv[1] * q1_q1 +
           rtDW.DiscreteTimeIntegratory_DSTA_kr[1]);

        // SignalConversion: '<S147>/OutportBufferFors_g_ref_dt2_smooth' incorporates:
        //   DiscreteIntegrator: '<S150>/Discrete-Time Integrator y_dt'

        rtDW.Merge2_j[1] = rtDW.DiscreteTimeIntegratory_dt_D_iv[1];

        // SignalConversion: '<S147>/OutportBufferFors_g_ref_dt_smooth' incorporates:
        //   DiscreteIntegrator: '<S150>/Discrete-Time Integrator y'

        rtDW.Merge1_p[2] = rtDW.DiscreteTimeIntegratory_DSTA_kr[2];

        // Sum: '<S150>/Sum2' incorporates:
        //   DiscreteIntegrator: '<S150>/Discrete-Time Integrator y'
        //   DiscreteIntegrator: '<S150>/Discrete-Time Integrator y_dt'
        //   Gain: '<S150>/Gain'
        //   MATLAB Function: '<S37>/WpNav Matching'
        //   Product: '<S150>/Product2'
        //   Sum: '<S150>/Sum3'

        rtb_V_Kg_n0[2] = rtb_p_ahead_i[2] -
          (rtDW.DiscreteTimeIntegratory_dt_D_iv[2] * q1_q1 +
           rtDW.DiscreteTimeIntegratory_DSTA_kr[2]);

        // SignalConversion: '<S147>/OutportBufferFors_g_ref_dt2_smooth' incorporates:
        //   DiscreteIntegrator: '<S150>/Discrete-Time Integrator y_dt'

        rtDW.Merge2_j[2] = rtDW.DiscreteTimeIntegratory_dt_D_iv[2];

        // Product: '<S150>/omega^2' incorporates:
        //   Product: '<S151>/omega^2'

        scale = q1_q2 * q1_q2;

        // DiscreteIntegrator: '<S151>/Discrete-Time Integrator y' incorporates:
        //   DiscreteIntegrator: '<S49>/Discrete-Time Integrator y'
        //   DiscreteIntegrator: '<S50>/Discrete-Time Integrator y'
        //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y'
        //   MATLAB Function: '<S37>/Look Ahead1'

        if (rtDW.DiscreteTimeIntegratory_IC_L_o2 != 0) {
          rtDW.DiscreteTimeIntegratory_DSTA_pq[0] = 0.5F *
            rtDW.DiscreteTimeIntegratory_DSTAT_n[0] * q1_q3 * q1_q3 +
            (rtDW.DiscreteTimeIntegratory_DSTAT_j[0] * q1_q3 +
             rtDW.DiscreteTimeIntegratory_DSTA_n2[0]);
          rtDW.DiscreteTimeIntegratory_DSTA_pq[1] = 0.5F *
            rtDW.DiscreteTimeIntegratory_DSTAT_n[1] * q1_q3 * q1_q3 +
            (rtDW.DiscreteTimeIntegratory_DSTAT_j[1] * q1_q3 +
             rtDW.DiscreteTimeIntegratory_DSTA_n2[1]);
          rtDW.DiscreteTimeIntegratory_DSTA_pq[2] = 0.5F *
            rtDW.DiscreteTimeIntegratory_DSTAT_n[2] * q1_q3 * q1_q3 +
            (rtDW.DiscreteTimeIntegratory_DSTAT_j[2] * q1_q3 +
             rtDW.DiscreteTimeIntegratory_DSTA_n2[2]);
        }

        // SignalConversion: '<S147>/OutportBufferFors_g_ref_smooth' incorporates:
        //   DiscreteIntegrator: '<S151>/Discrete-Time Integrator y'

        rtDW.Merge_l[0] = rtDW.DiscreteTimeIntegratory_DSTA_pq[0];

        // Sum: '<S151>/Sum2' incorporates:
        //   DiscreteIntegrator: '<S151>/Discrete-Time Integrator y'
        //   DiscreteIntegrator: '<S151>/Discrete-Time Integrator y_dt'
        //   MATLAB Function: '<S37>/WpNav Matching'
        //   Product: '<S151>/Product2'
        //   Sum: '<S151>/Sum3'

        rtb_Sum2_p[0] = rtb_Sum2_cx[0] - (rtDW.DiscreteTimeIntegratory_dt_D_ae[0]
          * q1_q1 + rtDW.DiscreteTimeIntegratory_DSTA_pq[0]);

        // Update for DiscreteIntegrator: '<S150>/Discrete-Time Integrator y' incorporates:
        //   DiscreteIntegrator: '<S150>/Discrete-Time Integrator y_dt'

        rtDW.DiscreteTimeIntegratory_DSTA_kr[0] += 0.0025F *
          rtDW.DiscreteTimeIntegratory_dt_D_iv[0];

        // Update for DiscreteIntegrator: '<S150>/Discrete-Time Integrator y_dt' incorporates:
        //   Product: '<S150>/Product1'
        //   Product: '<S150>/omega^2'

        rtDW.DiscreteTimeIntegratory_dt_D_iv[0] += rtb_V_Kg_n0[0] * scale *
          0.0025F;

        // Update for DiscreteIntegrator: '<S151>/Discrete-Time Integrator y' incorporates:
        //   DiscreteIntegrator: '<S151>/Discrete-Time Integrator y_dt'

        rtDW.DiscreteTimeIntegratory_DSTA_pq[0] += 0.0025F *
          rtDW.DiscreteTimeIntegratory_dt_D_ae[0];

        // SignalConversion: '<S147>/OutportBufferFors_g_ref_smooth' incorporates:
        //   DiscreteIntegrator: '<S151>/Discrete-Time Integrator y'

        rtDW.Merge_l[1] = rtDW.DiscreteTimeIntegratory_DSTA_pq[1];

        // Sum: '<S151>/Sum2' incorporates:
        //   DiscreteIntegrator: '<S151>/Discrete-Time Integrator y'
        //   DiscreteIntegrator: '<S151>/Discrete-Time Integrator y_dt'
        //   MATLAB Function: '<S37>/WpNav Matching'
        //   Product: '<S151>/Product2'
        //   Sum: '<S151>/Sum3'

        rtb_Sum2_p[1] = rtb_Sum2_cx[1] - (rtDW.DiscreteTimeIntegratory_dt_D_ae[1]
          * q1_q1 + rtDW.DiscreteTimeIntegratory_DSTA_pq[1]);

        // Update for DiscreteIntegrator: '<S150>/Discrete-Time Integrator y' incorporates:
        //   DiscreteIntegrator: '<S150>/Discrete-Time Integrator y_dt'

        rtDW.DiscreteTimeIntegratory_DSTA_kr[1] += 0.0025F *
          rtDW.DiscreteTimeIntegratory_dt_D_iv[1];

        // Update for DiscreteIntegrator: '<S150>/Discrete-Time Integrator y_dt' incorporates:
        //   Product: '<S150>/Product1'
        //   Product: '<S150>/omega^2'

        rtDW.DiscreteTimeIntegratory_dt_D_iv[1] += rtb_V_Kg_n0[1] * scale *
          0.0025F;

        // Update for DiscreteIntegrator: '<S151>/Discrete-Time Integrator y' incorporates:
        //   DiscreteIntegrator: '<S151>/Discrete-Time Integrator y_dt'

        rtDW.DiscreteTimeIntegratory_DSTA_pq[1] += 0.0025F *
          rtDW.DiscreteTimeIntegratory_dt_D_ae[1];

        // SignalConversion: '<S147>/OutportBufferFors_g_ref_smooth' incorporates:
        //   DiscreteIntegrator: '<S151>/Discrete-Time Integrator y'

        rtDW.Merge_l[2] = rtDW.DiscreteTimeIntegratory_DSTA_pq[2];

        // Sum: '<S151>/Sum2' incorporates:
        //   DiscreteIntegrator: '<S151>/Discrete-Time Integrator y'
        //   DiscreteIntegrator: '<S151>/Discrete-Time Integrator y_dt'
        //   MATLAB Function: '<S37>/WpNav Matching'
        //   Product: '<S151>/Product2'
        //   Sum: '<S151>/Sum3'

        rtb_Sum2_p[2] = rtb_Sum2_cx[2] - (rtDW.DiscreteTimeIntegratory_dt_D_ae[2]
          * q1_q1 + rtDW.DiscreteTimeIntegratory_DSTA_pq[2]);

        // Update for DiscreteIntegrator: '<S150>/Discrete-Time Integrator y' incorporates:
        //   DiscreteIntegrator: '<S150>/Discrete-Time Integrator y_dt'

        rtDW.DiscreteTimeIntegratory_DSTA_kr[2] += 0.0025F *
          rtDW.DiscreteTimeIntegratory_dt_D_iv[2];

        // Update for DiscreteIntegrator: '<S150>/Discrete-Time Integrator y_dt' incorporates:
        //   Product: '<S150>/Product1'
        //   Product: '<S150>/omega^2'

        rtDW.DiscreteTimeIntegratory_dt_D_iv[2] += rtb_V_Kg_n0[2] * scale *
          0.0025F;

        // Update for DiscreteIntegrator: '<S151>/Discrete-Time Integrator y' incorporates:
        //   DiscreteIntegrator: '<S151>/Discrete-Time Integrator y_dt'

        rtDW.DiscreteTimeIntegratory_DSTA_pq[2] += 0.0025F *
          rtDW.DiscreteTimeIntegratory_dt_D_ae[2];

        // Update for DiscreteIntegrator: '<S150>/Discrete-Time Integrator y'
        rtDW.DiscreteTimeIntegratory_IC_L_io = 0U;

        // Update for DiscreteIntegrator: '<S151>/Discrete-Time Integrator y'
        rtDW.DiscreteTimeIntegratory_IC_L_o2 = 0U;

        // Update for DiscreteIntegrator: '<S151>/Discrete-Time Integrator y_dt' incorporates:
        //   Product: '<S151>/Product1'

        rtDW.DiscreteTimeIntegratory_dt_D_ae[0] += rtb_Sum2_p[0] * scale *
          0.0025F;
        rtDW.DiscreteTimeIntegratory_dt_D_ae[1] += rtb_Sum2_p[1] * scale *
          0.0025F;
        rtDW.DiscreteTimeIntegratory_dt_D_ae[2] += rtb_Sum2_p[2] * scale *
          0.0025F;
      } else {
        if (rtDW.FlightPathSmoothing_MODE) {
          rtDW.FlightPathSmoothing_MODE = false;
        }

        // Outputs for Enabled SubSystem: '<S140>/Pass-through' incorporates:
        //   EnablePort: '<S148>/Enable'

        rtDW.Merge_l[0] = rtb_Sum2_cx[0];
        rtDW.Merge1_p[0] = rtb_p_ahead_i[0];
        rtDW.Merge2_j[0] = rtb_Sum2_g[0];
        rtDW.Merge_l[1] = rtb_Sum2_cx[1];
        rtDW.Merge1_p[1] = rtb_p_ahead_i[1];
        rtDW.Merge2_j[1] = rtb_Sum2_g[1];
        rtDW.Merge_l[2] = rtb_Sum2_cx[2];
        rtDW.Merge1_p[2] = rtb_p_ahead_i[2];
        rtDW.Merge2_j[2] = rtb_Sum2_g[2];

        // End of Outputs for SubSystem: '<S140>/Pass-through'
      }

      // End of Outputs for SubSystem: '<S140>/Flight Path Smoothing'

      // SignalConversion: '<S37>/BusConversion_InsertedFor_wpnav_at_inport_0' incorporates:
      //   Constant: '<S140>/Constant'
      //   Constant: '<S146>/Constant'
      //   DataTypeConversion: '<S37>/Data Type Conversion9'
      //   Inport: '<S148>/In1'
      //   Inport: '<S148>/In2'
      //   Inport: '<S148>/In3'
      //   MATLAB Function: '<S37>/WpNav Matching'
      //   RelationalOperator: '<S146>/Compare'

      rtDW.wp_idx_n = (real32_T)i_0;

      // SignalConversion: '<S37>/BusConversion_InsertedFor_wpnav_at_inport_0' incorporates:
      //   DataTypeConversion: '<S37>/Data Type Conversion8'
      //   MATLAB Function: '<S37>/WpNav Matching'

      rtDW.stage_e = (real32_T)p2;

      // Update for UnitDelay: '<S37>/Unit Delay' incorporates:
      //   MATLAB Function: '<S37>/WpNav Matching'

      rtDW.UnitDelay_DSTATE_b = i_0;

      // Update for UnitDelay: '<S37>/Unit Delay1' incorporates:
      //   MATLAB Function: '<S37>/WpNav Matching'

      rtDW.UnitDelay1_DSTATE_g = p2;

      // SignalConversion: '<S37>/BusConversion_InsertedFor_wpnav_at_inport_0' incorporates:
      //   MATLAB Function: '<S37>/WpNav Matching'

      rtDW.s_g_match_i[0] = rtb_y_fg[0];

      // SignalConversion: '<S37>/BusConversion_InsertedFor_wpnav_at_inport_0'
      s_g_ref_dt2[0] = rtDW.Merge2_j[0];

      // Update for UnitDelay: '<S37>/Unit Delay6' incorporates:
      //   MATLAB Function: '<S37>/WpNav Matching'

      rtDW.UnitDelay6_DSTATE[0] = rtb_y_fg[0];

      // SignalConversion: '<S37>/BusConversion_InsertedFor_wpnav_at_inport_0' incorporates:
      //   MATLAB Function: '<S37>/WpNav Matching'

      rtDW.s_g_match_i[1] = rtb_y_fg[1];

      // SignalConversion: '<S37>/BusConversion_InsertedFor_wpnav_at_inport_0'
      s_g_ref_dt2[1] = rtDW.Merge2_j[1];

      // Update for UnitDelay: '<S37>/Unit Delay6' incorporates:
      //   MATLAB Function: '<S37>/WpNav Matching'

      rtDW.UnitDelay6_DSTATE[1] = rtb_y_fg[1];

      // SignalConversion: '<S37>/BusConversion_InsertedFor_wpnav_at_inport_0' incorporates:
      //   MATLAB Function: '<S37>/WpNav Matching'

      rtDW.s_g_match_i[2] = rtb_y_fg[2];

      // SignalConversion: '<S37>/BusConversion_InsertedFor_wpnav_at_inport_0'
      s_g_ref_dt2[2] = rtDW.Merge2_j[2];

      // Update for UnitDelay: '<S37>/Unit Delay6' incorporates:
      //   MATLAB Function: '<S37>/WpNav Matching'

      rtDW.UnitDelay6_DSTATE[2] = rtb_y_fg[2];

      // Update for UnitDelay: '<S37>/Unit Delay5' incorporates:
      //   MATLAB Function: '<S37>/WpNav Matching'

      rtDW.UnitDelay5_DSTATE = is_approach;

      // Update for UnitDelay: '<S37>/Unit Delay2' incorporates:
      //   MATLAB Function: '<S37>/WpNav Matching'

      rtDW.UnitDelay2_DSTATE = itmp;

      // Update for UnitDelay: '<S37>/Unit Delay3' incorporates:
      //   MATLAB Function: '<S37>/WpNav Matching'

      rtDW.UnitDelay3_DSTATE = c_stage_app;

      // Update for UnitDelay: '<S37>/Unit Delay4' incorporates:
      //   MATLAB Function: '<S37>/WpNav Matching'

      for (stage_app_2 = 0; stage_app_2 < 15; stage_app_2++) {
        rtDW.UnitDelay4_DSTATE[stage_app_2] = wp_approach_out[stage_app_2];
      }

      // End of Update for UnitDelay: '<S37>/Unit Delay4'
      if (!rtDW.NDIPositionController_MODE) {
        // InitializeConditions for DiscreteIntegrator: '<S65>/Discrete-Time Integrator y' 
        rtDW.DiscreteTimeIntegratory_IC_L_d2 = 1U;

        // InitializeConditions for DiscreteIntegrator: '<S64>/Discrete-Time Integrator y' 
        rtDW.DiscreteTimeIntegratory_IC_LO_p = 1U;

        // InitializeConditions for DiscreteIntegrator: '<S66>/Discrete-Time Integrator y' 
        rtDW.DiscreteTimeIntegratory_IC_L_ls = 1U;

        // InitializeConditions for DiscreteIntegrator: '<S65>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_D_jv[0] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S64>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_D_ip[0] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S66>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_DS_c[0] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S65>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_D_jv[1] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S64>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_D_ip[1] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S66>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_DS_c[1] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S65>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_D_jv[2] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S64>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_D_ip[2] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S66>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_DS_c[2] = 0.0F;
        rtDW.NDIPositionController_MODE = true;
      }

      // DiscreteIntegrator: '<S65>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S50>/Discrete-Time Integrator y'

      if (rtDW.DiscreteTimeIntegratory_IC_L_d2 != 0) {
        rtDW.DiscreteTimeIntegratory_DSTA_nm[0] =
          rtDW.DiscreteTimeIntegratory_DSTA_n2[0];
        rtDW.DiscreteTimeIntegratory_DSTA_nm[1] =
          rtDW.DiscreteTimeIntegratory_DSTA_n2[1];
        rtDW.DiscreteTimeIntegratory_DSTA_nm[2] =
          rtDW.DiscreteTimeIntegratory_DSTA_n2[2];
      }

      // Gain: '<S65>/Gain' incorporates:
      //   Constant: '<S65>/d'
      //   Constant: '<S65>/omega'
      //   Gain: '<S64>/Gain'
      //   Gain: '<S66>/Gain'
      //   Product: '<S65>/Divide'

      q1_q1 = 1.0F / rtP.lindi.atc.rm.rfreq * 2.0F;

      // Sum: '<S65>/Sum2' incorporates:
      //   DiscreteIntegrator: '<S65>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S65>/Discrete-Time Integrator y_dt'
      //   Gain: '<S65>/Gain'
      //   Product: '<S65>/Product2'
      //   SignalConversion: '<S37>/BusConversion_InsertedFor_wpnav_at_inport_0'
      //   Sum: '<S65>/Sum3'

      rtb_Sum2_cx[0] = rtDW.Merge_l[0] - (rtDW.DiscreteTimeIntegratory_dt_D_jv[0]
        * q1_q1 + rtDW.DiscreteTimeIntegratory_DSTA_nm[0]);

      // Sum: '<S28>/Add' incorporates:
      //   DiscreteIntegrator: '<S50>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S65>/Discrete-Time Integrator y'

      rtDW.e_s_g_e[0] = rtDW.DiscreteTimeIntegratory_DSTA_nm[0] -
        rtDW.DiscreteTimeIntegratory_DSTA_n2[0];

      // Sum: '<S65>/Sum2' incorporates:
      //   DiscreteIntegrator: '<S65>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S65>/Discrete-Time Integrator y_dt'
      //   Gain: '<S65>/Gain'
      //   Product: '<S65>/Product2'
      //   SignalConversion: '<S37>/BusConversion_InsertedFor_wpnav_at_inport_0'
      //   Sum: '<S65>/Sum3'

      rtb_Sum2_cx[1] = rtDW.Merge_l[1] - (rtDW.DiscreteTimeIntegratory_dt_D_jv[1]
        * q1_q1 + rtDW.DiscreteTimeIntegratory_DSTA_nm[1]);

      // Sum: '<S28>/Add' incorporates:
      //   DiscreteIntegrator: '<S50>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S65>/Discrete-Time Integrator y'

      rtDW.e_s_g_e[1] = rtDW.DiscreteTimeIntegratory_DSTA_nm[1] -
        rtDW.DiscreteTimeIntegratory_DSTA_n2[1];

      // Sum: '<S65>/Sum2' incorporates:
      //   DiscreteIntegrator: '<S65>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S65>/Discrete-Time Integrator y_dt'
      //   Gain: '<S65>/Gain'
      //   Product: '<S65>/Product2'
      //   SignalConversion: '<S37>/BusConversion_InsertedFor_wpnav_at_inport_0'
      //   Sum: '<S65>/Sum3'

      rtb_Sum2_cx[2] = rtDW.Merge_l[2] - (rtDW.DiscreteTimeIntegratory_dt_D_jv[2]
        * q1_q1 + rtDW.DiscreteTimeIntegratory_DSTA_nm[2]);

      // Sum: '<S28>/Add' incorporates:
      //   DiscreteIntegrator: '<S50>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S65>/Discrete-Time Integrator y'

      rtDW.e_s_g_e[2] = rtDW.DiscreteTimeIntegratory_DSTA_nm[2] -
        rtDW.DiscreteTimeIntegratory_DSTA_n2[2];

      // Product: '<S65>/omega^2' incorporates:
      //   Constant: '<S65>/omega'
      //   Product: '<S64>/omega^2'
      //   Product: '<S66>/omega^2'

      scale = rtP.lindi.atc.rm.rfreq * rtP.lindi.atc.rm.rfreq;

      // DiscreteIntegrator: '<S64>/Discrete-Time Integrator y' incorporates:
      //   SignalConversion: '<S37>/BusConversion_InsertedFor_wpnav_at_inport_0'

      if (rtDW.DiscreteTimeIntegratory_IC_LO_p != 0) {
        rtDW.DiscreteTimeIntegratory_DSTAT_e[0] = rtDW.Merge1_p[0];
        rtDW.DiscreteTimeIntegratory_DSTAT_e[1] = rtDW.Merge1_p[1];
        rtDW.DiscreteTimeIntegratory_DSTAT_e[2] = rtDW.Merge1_p[2];
      }

      // Sum: '<S64>/Sum2' incorporates:
      //   DiscreteIntegrator: '<S64>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S64>/Discrete-Time Integrator y_dt'
      //   Product: '<S64>/Product2'
      //   SignalConversion: '<S37>/BusConversion_InsertedFor_wpnav_at_inport_0'
      //   Sum: '<S64>/Sum3'

      rtb_Sum2_g[0] = rtDW.Merge1_p[0] - (rtDW.DiscreteTimeIntegratory_dt_D_ip[0]
        * q1_q1 + rtDW.DiscreteTimeIntegratory_DSTAT_e[0]);

      // Sum: '<S28>/Add1' incorporates:
      //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S64>/Discrete-Time Integrator y'

      rtDW.e_s_g_dt_i[0] = rtDW.DiscreteTimeIntegratory_DSTAT_e[0] -
        rtDW.DiscreteTimeIntegratory_DSTAT_j[0];

      // Sum: '<S64>/Sum2' incorporates:
      //   DiscreteIntegrator: '<S64>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S64>/Discrete-Time Integrator y_dt'
      //   Product: '<S64>/Product2'
      //   SignalConversion: '<S37>/BusConversion_InsertedFor_wpnav_at_inport_0'
      //   Sum: '<S64>/Sum3'

      rtb_Sum2_g[1] = rtDW.Merge1_p[1] - (rtDW.DiscreteTimeIntegratory_dt_D_ip[1]
        * q1_q1 + rtDW.DiscreteTimeIntegratory_DSTAT_e[1]);

      // Sum: '<S28>/Add1' incorporates:
      //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S64>/Discrete-Time Integrator y'

      rtDW.e_s_g_dt_i[1] = rtDW.DiscreteTimeIntegratory_DSTAT_e[1] -
        rtDW.DiscreteTimeIntegratory_DSTAT_j[1];

      // Sum: '<S64>/Sum2' incorporates:
      //   DiscreteIntegrator: '<S64>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S64>/Discrete-Time Integrator y_dt'
      //   Product: '<S64>/Product2'
      //   SignalConversion: '<S37>/BusConversion_InsertedFor_wpnav_at_inport_0'
      //   Sum: '<S64>/Sum3'

      rtb_Sum2_g[2] = rtDW.Merge1_p[2] - (rtDW.DiscreteTimeIntegratory_dt_D_ip[2]
        * q1_q1 + rtDW.DiscreteTimeIntegratory_DSTAT_e[2]);

      // Sum: '<S28>/Add1' incorporates:
      //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S64>/Discrete-Time Integrator y'

      rtDW.e_s_g_dt_i[2] = rtDW.DiscreteTimeIntegratory_DSTAT_e[2] -
        rtDW.DiscreteTimeIntegratory_DSTAT_j[2];

      // DiscreteIntegrator: '<S66>/Discrete-Time Integrator y' incorporates:
      //   SignalConversion: '<S37>/BusConversion_InsertedFor_wpnav_at_inport_0'

      if (rtDW.DiscreteTimeIntegratory_IC_L_ls != 0) {
        rtDW.DiscreteTimeIntegratory_DSTA_k4[0] = rtDW.Merge2_j[0];
        rtDW.DiscreteTimeIntegratory_DSTA_k4[1] = rtDW.Merge2_j[1];
        rtDW.DiscreteTimeIntegratory_DSTA_k4[2] = rtDW.Merge2_j[2];
      }

      for (i_0 = 0; i_0 < 3; i_0++) {
        // Sum: '<S66>/Sum2' incorporates:
        //   DiscreteIntegrator: '<S66>/Discrete-Time Integrator y'
        //   DiscreteIntegrator: '<S66>/Discrete-Time Integrator y_dt'
        //   Product: '<S66>/Product2'
        //   Sum: '<S66>/Sum3'

        rtb_p_ahead_i[i_0] = s_g_ref_dt2[i_0] -
          (rtDW.DiscreteTimeIntegratory_dt_DS_c[i_0] * q1_q1 +
           rtDW.DiscreteTimeIntegratory_DSTA_k4[i_0]);

        // Sum: '<S28>/Add2' incorporates:
        //   DiscreteIntegrator: '<S49>/Discrete-Time Integrator y'
        //   DiscreteIntegrator: '<S66>/Discrete-Time Integrator y'

        rtDW.e_s_g_dt2_k[i_0] = rtDW.DiscreteTimeIntegratory_DSTA_k4[i_0] -
          rtDW.DiscreteTimeIntegratory_DSTAT_n[i_0];
        rtb_y_fg[i_0] = rtDW.DiscreteTimeIntegratory_DSTAT_b[i_0 + 6] *
          s_g_ref_dt2[2] + (rtDW.DiscreteTimeIntegratory_DSTAT_b[i_0 + 3] *
                            s_g_ref_dt2[1] +
                            rtDW.DiscreteTimeIntegratory_DSTAT_b[i_0] *
                            s_g_ref_dt2[0]);
      }

      for (stage_app_2 = 0; stage_app_2 < 3; stage_app_2++) {
        // Product: '<S28>/Matrix Multiply' incorporates:
        //   DiscreteIntegrator: '<S48>/Discrete-Time Integrator y'
        //   Product: '<S28>/Matrix Multiply1'
        //   Product: '<S28>/Matrix Multiply2'

        rtb_Sum2_ms = rtDW.DiscreteTimeIntegratory_DSTAT_b[stage_app_2 + 3];
        rtb_y_p3 = rtDW.DiscreteTimeIntegratory_DSTAT_b[stage_app_2 + 6];
        rtb_y_a5[stage_app_2] = rtb_y_p3 * rtDW.e_s_g_e[2] + (rtb_Sum2_ms *
          rtDW.e_s_g_e[1] + rtDW.DiscreteTimeIntegratory_DSTAT_b[stage_app_2] *
          rtDW.e_s_g_e[0]);
        circ_seg_n_0[stage_app_2] = rtb_y_p3 * rtDW.e_s_g_dt_i[2] + (rtb_Sum2_ms
          * rtDW.e_s_g_dt_i[1] +
          rtDW.DiscreteTimeIntegratory_DSTAT_b[stage_app_2] * rtDW.e_s_g_dt_i[0]);
        rtb_V_Kg_n0[stage_app_2] = rtb_y_p3 * rtDW.e_s_g_dt2_k[2] + (rtb_Sum2_ms
          * rtDW.e_s_g_dt2_k[1] +
          rtDW.DiscreteTimeIntegratory_DSTAT_b[stage_app_2] * rtDW.e_s_g_dt2_k[0]);
      }

      // Sum: '<S28>/Add5' incorporates:
      //   DiscreteIntegrator: '<S48>/Discrete-Time Integrator y'
      //   Gain: '<S28>/Gain'
      //   Gain: '<S28>/Gain1'
      //   Gain: '<S28>/Gain2'
      //   Product: '<S28>/Matrix Multiply'
      //   Product: '<S28>/Matrix Multiply1'
      //   Product: '<S28>/Matrix Multiply2'
      //   Product: '<S28>/Matrix Multiply3'
      //   Sum: '<S28>/Add3'

      rtDW.nu[0] = ((rtP.lindi.psc.k.pos * rtb_y_a5[1] + rtP.lindi.psc.k.vel *
                     circ_seg_n_0[1]) + rtP.lindi.psc.k.acc * rtb_V_Kg_n0[1]) +
        rtb_y_fg[1];
      rtDW.nu[1] = ((rtP.lindi.psc.k.pos * rtb_y_a5[2] + rtP.lindi.psc.k.vel *
                     circ_seg_n_0[2]) + rtP.lindi.psc.k.acc * rtb_V_Kg_n0[2]) +
        rtb_y_fg[2];

      // Update for DiscreteIntegrator: '<S65>/Discrete-Time Integrator y'
      rtDW.DiscreteTimeIntegratory_IC_L_d2 = 0U;

      // Update for DiscreteIntegrator: '<S64>/Discrete-Time Integrator y'
      rtDW.DiscreteTimeIntegratory_IC_LO_p = 0U;

      // Update for DiscreteIntegrator: '<S66>/Discrete-Time Integrator y'
      rtDW.DiscreteTimeIntegratory_IC_L_ls = 0U;

      // SignalConversion: '<S28>/BusConversion_InsertedFor_pos_control_at_inport_0' incorporates:
      //   SignalConversion: '<S37>/BusConversion_InsertedFor_wpnav_at_inport_0'

      rtDW.s_g_ref_d[0] = rtDW.Merge_l[0];

      // SignalConversion: '<S28>/BusConversion_InsertedFor_pos_control_at_inport_0' incorporates:
      //   DiscreteIntegrator: '<S50>/Discrete-Time Integrator y'

      rtDW.s_g_a[0] = rtDW.DiscreteTimeIntegratory_DSTA_n2[0];

      // Update for DiscreteIntegrator: '<S65>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S65>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_DSTA_nm[0] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_D_jv[0];

      // Update for DiscreteIntegrator: '<S65>/Discrete-Time Integrator y_dt' incorporates:
      //   Product: '<S65>/Product1'
      //   Product: '<S65>/omega^2'

      rtDW.DiscreteTimeIntegratory_dt_D_jv[0] += rtb_Sum2_cx[0] * scale *
        0.0025F;

      // Update for DiscreteIntegrator: '<S64>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S64>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_DSTAT_e[0] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_D_ip[0];

      // Update for DiscreteIntegrator: '<S64>/Discrete-Time Integrator y_dt' incorporates:
      //   Product: '<S64>/Product1'

      rtDW.DiscreteTimeIntegratory_dt_D_ip[0] += rtb_Sum2_g[0] * scale * 0.0025F;

      // Update for DiscreteIntegrator: '<S66>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S66>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_DSTA_k4[0] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_DS_c[0];

      // Update for DiscreteIntegrator: '<S66>/Discrete-Time Integrator y_dt' incorporates:
      //   Product: '<S66>/Product1'

      rtDW.DiscreteTimeIntegratory_dt_DS_c[0] += rtb_p_ahead_i[0] * scale *
        0.0025F;

      // SignalConversion: '<S28>/BusConversion_InsertedFor_pos_control_at_inport_0' incorporates:
      //   SignalConversion: '<S37>/BusConversion_InsertedFor_wpnav_at_inport_0'

      rtDW.s_g_ref_d[1] = rtDW.Merge_l[1];

      // SignalConversion: '<S28>/BusConversion_InsertedFor_pos_control_at_inport_0' incorporates:
      //   DiscreteIntegrator: '<S50>/Discrete-Time Integrator y'

      rtDW.s_g_a[1] = rtDW.DiscreteTimeIntegratory_DSTA_n2[1];

      // Update for DiscreteIntegrator: '<S65>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S65>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_DSTA_nm[1] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_D_jv[1];

      // Update for DiscreteIntegrator: '<S65>/Discrete-Time Integrator y_dt' incorporates:
      //   Product: '<S65>/Product1'
      //   Product: '<S65>/omega^2'

      rtDW.DiscreteTimeIntegratory_dt_D_jv[1] += rtb_Sum2_cx[1] * scale *
        0.0025F;

      // Update for DiscreteIntegrator: '<S64>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S64>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_DSTAT_e[1] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_D_ip[1];

      // Update for DiscreteIntegrator: '<S64>/Discrete-Time Integrator y_dt' incorporates:
      //   Product: '<S64>/Product1'

      rtDW.DiscreteTimeIntegratory_dt_D_ip[1] += rtb_Sum2_g[1] * scale * 0.0025F;

      // Update for DiscreteIntegrator: '<S66>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S66>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_DSTA_k4[1] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_DS_c[1];

      // Update for DiscreteIntegrator: '<S66>/Discrete-Time Integrator y_dt' incorporates:
      //   Product: '<S66>/Product1'

      rtDW.DiscreteTimeIntegratory_dt_DS_c[1] += rtb_p_ahead_i[1] * scale *
        0.0025F;

      // SignalConversion: '<S28>/BusConversion_InsertedFor_pos_control_at_inport_0' incorporates:
      //   SignalConversion: '<S37>/BusConversion_InsertedFor_wpnav_at_inport_0'

      rtDW.s_g_ref_d[2] = rtDW.Merge_l[2];

      // SignalConversion: '<S28>/BusConversion_InsertedFor_pos_control_at_inport_0' incorporates:
      //   DiscreteIntegrator: '<S50>/Discrete-Time Integrator y'

      rtDW.s_g_a[2] = rtDW.DiscreteTimeIntegratory_DSTA_n2[2];

      // Update for DiscreteIntegrator: '<S65>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S65>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_DSTA_nm[2] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_D_jv[2];

      // Update for DiscreteIntegrator: '<S65>/Discrete-Time Integrator y_dt' incorporates:
      //   Product: '<S65>/Product1'
      //   Product: '<S65>/omega^2'

      rtDW.DiscreteTimeIntegratory_dt_D_jv[2] += rtb_Sum2_cx[2] * scale *
        0.0025F;

      // Update for DiscreteIntegrator: '<S64>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S64>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_DSTAT_e[2] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_D_ip[2];

      // Update for DiscreteIntegrator: '<S64>/Discrete-Time Integrator y_dt' incorporates:
      //   Product: '<S64>/Product1'

      rtDW.DiscreteTimeIntegratory_dt_D_ip[2] += rtb_Sum2_g[2] * scale * 0.0025F;

      // Update for DiscreteIntegrator: '<S66>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S66>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_DSTA_k4[2] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_DS_c[2];

      // Update for DiscreteIntegrator: '<S66>/Discrete-Time Integrator y_dt' incorporates:
      //   Product: '<S66>/Product1'

      rtDW.DiscreteTimeIntegratory_dt_DS_c[2] += rtb_p_ahead_i[2] * scale *
        0.0025F;
    } else {
      if (rtDW.WaypointNavigation_MODE) {
        // Disable for Enabled SubSystem: '<S140>/Flight Path Smoothing'
        if (rtDW.FlightPathSmoothing_MODE) {
          rtDW.FlightPathSmoothing_MODE = false;
        }

        // End of Disable for SubSystem: '<S140>/Flight Path Smoothing'
        rtDW.WaypointNavigation_MODE = false;
      }

      if (rtDW.NDIPositionController_MODE) {
        rtDW.NDIPositionController_MODE = false;
      }
    }

    // End of Outputs for SubSystem: '<S9>/Waypoint Navigation'
    // End of Outputs for SubSystem: '<S9>/NDI Position Controller'

    // Outputs for Enabled SubSystem: '<S26>/Single LPF' incorporates:
    //   EnablePort: '<S54>/Enable'

    // Outputs for Enabled SubSystem: '<S26>/Duplicate LPF' incorporates:
    //   EnablePort: '<S47>/Enable'

    if (rtP.lindi.sflt.numGyrFlt == 1.0F) {
      // Gain: '<S63>/Gain' incorporates:
      //   Constant: '<S63>/d'
      //   Constant: '<S63>/omega'
      //   Product: '<S63>/Divide'

      q1_q1 = rtP.lindi.sflt.d / rtP.lindi.sflt.omega * 2.0F;

      // Product: '<S63>/omega^2' incorporates:
      //   Constant: '<S63>/omega'

      scale = rtP.lindi.sflt.omega * rtP.lindi.sflt.omega;

      // DiscreteIntegrator: '<S63>/Discrete-Time Integrator y_dt'
      q1_q3 = rtDW.DiscreteTimeIntegratory_dt_D_pr[0];

      // SignalConversion: '<S54>/OutportBufferForOmega_Kb_dt_f' incorporates:
      //   DiscreteIntegrator: '<S63>/Discrete-Time Integrator y_dt'

      rtb_Sum2_cx[0] = rtDW.DiscreteTimeIntegratory_dt_D_pr[0];

      // SignalConversion: '<S54>/OutportBufferForOmega_Kb_f' incorporates:
      //   DiscreteIntegrator: '<S63>/Discrete-Time Integrator y'

      rtb_y_fg[0] = rtDW.DiscreteTimeIntegratory_DSTAT_m[0];

      // Update for DiscreteIntegrator: '<S63>/Discrete-Time Integrator y_dt' incorporates:
      //   DiscreteIntegrator: '<S63>/Discrete-Time Integrator y'
      //   Gain: '<S63>/Gain'
      //   Inport: '<Root>/measure'
      //   Product: '<S63>/Product1'
      //   Product: '<S63>/Product2'
      //   Product: '<S63>/omega^2'
      //   Sum: '<S63>/Sum2'
      //   Sum: '<S63>/Sum3'

      rtDW.DiscreteTimeIntegratory_dt_D_pr[0] += (rtU.measure.omega_Kb[0] -
        (rtDW.DiscreteTimeIntegratory_dt_D_pr[0] * q1_q1 +
         rtDW.DiscreteTimeIntegratory_DSTAT_m[0])) * scale * 0.0025F;

      // Update for DiscreteIntegrator: '<S63>/Discrete-Time Integrator y'
      rtDW.DiscreteTimeIntegratory_DSTAT_m[0] += 0.0025F * q1_q3;

      // DiscreteIntegrator: '<S63>/Discrete-Time Integrator y_dt'
      q1_q3 = rtDW.DiscreteTimeIntegratory_dt_D_pr[1];

      // SignalConversion: '<S54>/OutportBufferForOmega_Kb_dt_f' incorporates:
      //   DiscreteIntegrator: '<S63>/Discrete-Time Integrator y_dt'

      rtb_Sum2_cx[1] = rtDW.DiscreteTimeIntegratory_dt_D_pr[1];

      // SignalConversion: '<S54>/OutportBufferForOmega_Kb_f' incorporates:
      //   DiscreteIntegrator: '<S63>/Discrete-Time Integrator y'

      rtb_y_fg[1] = rtDW.DiscreteTimeIntegratory_DSTAT_m[1];

      // Update for DiscreteIntegrator: '<S63>/Discrete-Time Integrator y_dt' incorporates:
      //   DiscreteIntegrator: '<S63>/Discrete-Time Integrator y'
      //   Gain: '<S63>/Gain'
      //   Inport: '<Root>/measure'
      //   Product: '<S63>/Product1'
      //   Product: '<S63>/Product2'
      //   Product: '<S63>/omega^2'
      //   Sum: '<S63>/Sum2'
      //   Sum: '<S63>/Sum3'

      rtDW.DiscreteTimeIntegratory_dt_D_pr[1] += (rtU.measure.omega_Kb[1] -
        (rtDW.DiscreteTimeIntegratory_dt_D_pr[1] * q1_q1 +
         rtDW.DiscreteTimeIntegratory_DSTAT_m[1])) * scale * 0.0025F;

      // Update for DiscreteIntegrator: '<S63>/Discrete-Time Integrator y'
      rtDW.DiscreteTimeIntegratory_DSTAT_m[1] += 0.0025F * q1_q3;

      // DiscreteIntegrator: '<S63>/Discrete-Time Integrator y_dt'
      q1_q3 = rtDW.DiscreteTimeIntegratory_dt_D_pr[2];

      // SignalConversion: '<S54>/OutportBufferForOmega_Kb_dt_f' incorporates:
      //   DiscreteIntegrator: '<S63>/Discrete-Time Integrator y_dt'

      rtb_Sum2_cx[2] = rtDW.DiscreteTimeIntegratory_dt_D_pr[2];

      // SignalConversion: '<S54>/OutportBufferForOmega_Kb_f' incorporates:
      //   DiscreteIntegrator: '<S63>/Discrete-Time Integrator y'

      rtb_y_fg[2] = rtDW.DiscreteTimeIntegratory_DSTAT_m[2];

      // Update for DiscreteIntegrator: '<S63>/Discrete-Time Integrator y_dt' incorporates:
      //   DiscreteIntegrator: '<S63>/Discrete-Time Integrator y'
      //   Gain: '<S63>/Gain'
      //   Inport: '<Root>/measure'
      //   Product: '<S63>/Product1'
      //   Product: '<S63>/Product2'
      //   Product: '<S63>/omega^2'
      //   Sum: '<S63>/Sum2'
      //   Sum: '<S63>/Sum3'

      rtDW.DiscreteTimeIntegratory_dt_D_pr[2] += (rtU.measure.omega_Kb[2] -
        (rtDW.DiscreteTimeIntegratory_dt_D_pr[2] * q1_q1 +
         rtDW.DiscreteTimeIntegratory_DSTAT_m[2])) * scale * 0.0025F;

      // Update for DiscreteIntegrator: '<S63>/Discrete-Time Integrator y'
      rtDW.DiscreteTimeIntegratory_DSTAT_m[2] += 0.0025F * q1_q3;
    } else {
      // DiscreteIntegrator: '<S56>/Discrete-Time Integrator1'
      // :  y0 = zeros([2,size(u)],class(u));
      // :  y0(1,:) = y_dt_0;
      // :  y0(2,:) = y_0;
      if (rtDW.DiscreteTimeIntegrator1_IC_LOAD != 0) {
        for (i_0 = 0; i_0 < 6; i_0++) {
          rtDW.DiscreteTimeIntegrator1_DSTAT_j[i_0] = 0.0F;
        }
      }

      for (i_0 = 0; i_0 < 6; i_0++) {
        rtb_DiscreteTimeIntegrator1_k[i_0] =
          rtDW.DiscreteTimeIntegrator1_DSTAT_j[i_0];
      }

      // DiscreteIntegrator: '<S56>/Discrete-Time Integrator'
      q1_q1 = rtDW.DiscreteTimeIntegrator_DSTAT_o4;

      // Sum: '<S56>/Add1' incorporates:
      //   DiscreteIntegrator: '<S56>/Discrete-Time Integrator'
      //   UnitDelay: '<S56>/Unit Delay'

      absxk = rtDW.DiscreteTimeIntegrator_DSTAT_o4 - rtDW.UnitDelay_DSTATE_g;

      // Sqrt: '<S47>/Sqrt' incorporates:
      //   Constant: '<S47>/Constant2'

      scale = std::sqrt(rtP.lindi.sflt.d);

      // MATLAB Function: '<S56>/PT2 discrete ode4' incorporates:
      //   Constant: '<S47>/Constant1'
      //   DiscreteIntegrator: '<S56>/Discrete-Time Integrator1'
      //   Inport: '<Root>/measure'

      // :  y_dt = zeros([2,size(u)],class(u));
      // :  k1 = f(y_0,u,omega,d);
      // :  len_u = length(u);
      // :  y_vec_dt = zeros(2,len_u,superiorfloat(u));
      // :  for i = 1:len_u
      for (i_0 = 0; i_0 < 3; i_0++) {
        // :  y_vec_dt(:,i) = [ -2*d*omega, -omega^2; 1, 0 ] * y_0(:,i) + [omega^2;0] * u(i); 
        q1_q3 = 2.0F * rtP.lindi.sflt.omega;
        p2 = i_0 << 1;
        q1_q2 = q1_q3 * q1_q3;
        rtb_y0[p2] = (-2.0F * scale * q1_q3 *
                      rtDW.DiscreteTimeIntegrator1_DSTAT_j[p2] + -q1_q2 *
                      rtDW.DiscreteTimeIntegrator1_DSTAT_j[p2 + 1]) + q1_q2 *
          rtU.measure.omega_Kb[i_0];
        rtb_y0[1 + p2] = rtDW.DiscreteTimeIntegrator1_DSTAT_j[i_0 << 1];
      }

      // :  k2 = f(y_0+0.5*h*k1,u,omega,d);
      q2_q2 = 0.5F * absxk;
      for (i_0 = 0; i_0 < 6; i_0++) {
        rtb_y0_k[i_0] = q2_q2 * rtb_y0[i_0] +
          rtDW.DiscreteTimeIntegrator1_DSTAT_j[i_0];
      }

      // :  len_u = length(u);
      // :  y_vec_dt = zeros(2,len_u,superiorfloat(u));
      // :  for i = 1:len_u
      for (i_0 = 0; i_0 < 3; i_0++) {
        // :  y_vec_dt(:,i) = [ -2*d*omega, -omega^2; 1, 0 ] * y_0(:,i) + [omega^2;0] * u(i); 
        k2[i_0 << 1] = (-2.0F * scale * (2.0F * rtP.lindi.sflt.omega) *
                        rtb_y0_k[i_0 << 1] + -(2.0F * rtP.lindi.sflt.omega *
          (2.0F * rtP.lindi.sflt.omega)) * rtb_y0_k[(i_0 << 1) + 1]) + 2.0F *
          rtP.lindi.sflt.omega * (2.0F * rtP.lindi.sflt.omega) *
          rtU.measure.omega_Kb[i_0];
        k2[1 + (i_0 << 1)] = rtb_y0_k[i_0 << 1];
      }

      // :  k3 = f(y_0+0.5*h*k2,u,omega,d);
      q2_q2 = 0.5F * absxk;
      for (stage_app_2 = 0; stage_app_2 < 6; stage_app_2++) {
        rtb_y0_k[stage_app_2] = q2_q2 * k2[stage_app_2] +
          rtDW.DiscreteTimeIntegrator1_DSTAT_j[stage_app_2];
      }

      // :  len_u = length(u);
      // :  y_vec_dt = zeros(2,len_u,superiorfloat(u));
      // :  for i = 1:len_u
      for (i_0 = 0; i_0 < 3; i_0++) {
        // :  y_vec_dt(:,i) = [ -2*d*omega, -omega^2; 1, 0 ] * y_0(:,i) + [omega^2;0] * u(i); 
        k3[i_0 << 1] = (-2.0F * scale * (2.0F * rtP.lindi.sflt.omega) *
                        rtb_y0_k[i_0 << 1] + -(2.0F * rtP.lindi.sflt.omega *
          (2.0F * rtP.lindi.sflt.omega)) * rtb_y0_k[(i_0 << 1) + 1]) + 2.0F *
          rtP.lindi.sflt.omega * (2.0F * rtP.lindi.sflt.omega) *
          rtU.measure.omega_Kb[i_0];
        k3[1 + (i_0 << 1)] = rtb_y0_k[i_0 << 1];
      }

      // :  k4 = f(y_0+h*k3,u,omega,d);
      for (stage_app_2 = 0; stage_app_2 < 6; stage_app_2++) {
        rtb_y0_k[stage_app_2] = absxk * k3[stage_app_2] +
          rtDW.DiscreteTimeIntegrator1_DSTAT_j[stage_app_2];
      }

      // :  len_u = length(u);
      // :  y_vec_dt = zeros(2,len_u,superiorfloat(u));
      // :  for i = 1:len_u
      for (i_0 = 0; i_0 < 3; i_0++) {
        // :  y_vec_dt(:,i) = [ -2*d*omega, -omega^2; 1, 0 ] * y_0(:,i) + [omega^2;0] * u(i); 
        k4[i_0 << 1] = (-2.0F * scale * (2.0F * rtP.lindi.sflt.omega) *
                        rtb_y0_k[i_0 << 1] + -(2.0F * rtP.lindi.sflt.omega *
          (2.0F * rtP.lindi.sflt.omega)) * rtb_y0_k[(i_0 << 1) + 1]) + 2.0F *
          rtP.lindi.sflt.omega * (2.0F * rtP.lindi.sflt.omega) *
          rtU.measure.omega_Kb[i_0];
        k4[1 + (i_0 << 1)] = rtb_y0_k[i_0 << 1];
      }

      // :  y_dt(:) = 1/6*(k1+2*k2+2*k3+k4);
      for (stage_app_2 = 0; stage_app_2 < 6; stage_app_2++) {
        rtb_y0[stage_app_2] = (((2.0F * k2[stage_app_2] + rtb_y0[stage_app_2]) +
          2.0F * k3[stage_app_2]) + k4[stage_app_2]) * 0.166666672F;
      }

      // End of MATLAB Function: '<S56>/PT2 discrete ode4'

      // MATLAB Function: '<S56>/PT2 split'
      PT2split(rtb_y0, rtb_DiscreteTimeIntegrator1_k, rtb_y_fg, rtb_Sum2_cx,
               rtb_Sum2_g);

      // DiscreteIntegrator: '<S55>/Discrete-Time Integrator'
      absxk = rtDW.DiscreteTimeIntegrator_DSTATE_b;

      // Sum: '<S55>/Add1' incorporates:
      //   DiscreteIntegrator: '<S55>/Discrete-Time Integrator'
      //   UnitDelay: '<S55>/Unit Delay'

      t = rtDW.DiscreteTimeIntegrator_DSTATE_b - rtDW.UnitDelay_DSTATE_l;

      // DiscreteIntegrator: '<S55>/Discrete-Time Integrator1'
      // :  y0 = zeros([2,size(u)],class(u));
      // :  y0(1,:) = y_dt_0;
      // :  y0(2,:) = y_0;
      if (rtDW.DiscreteTimeIntegrator1_IC_LO_g != 0) {
        for (i_0 = 0; i_0 < 6; i_0++) {
          rtDW.DiscreteTimeIntegrator1_DSTAT_n[i_0] = 0.0F;
        }
      }

      for (i_0 = 0; i_0 < 6; i_0++) {
        rtb_DiscreteTimeIntegrator1_k[i_0] =
          rtDW.DiscreteTimeIntegrator1_DSTAT_n[i_0];
      }

      // MATLAB Function: '<S55>/PT2 discrete ode4' incorporates:
      //   Constant: '<S47>/Constant1'
      //   DiscreteIntegrator: '<S55>/Discrete-Time Integrator1'

      // :  y_dt = zeros([2,size(u)],class(u));
      // :  k1 = f(y_0,u,omega,d);
      // :  len_u = length(u);
      // :  y_vec_dt = zeros(2,len_u,superiorfloat(u));
      // :  for i = 1:len_u
      for (i_0 = 0; i_0 < 3; i_0++) {
        // :  y_vec_dt(:,i) = [ -2*d*omega, -omega^2; 1, 0 ] * y_0(:,i) + [omega^2;0] * u(i); 
        rtb_y_dt_fn[i_0 << 1] = (-2.0F * scale * (2.0F * rtP.lindi.sflt.omega) *
          rtDW.DiscreteTimeIntegrator1_DSTAT_n[i_0 << 1] + -(2.0F *
          rtP.lindi.sflt.omega * (2.0F * rtP.lindi.sflt.omega)) *
          rtDW.DiscreteTimeIntegrator1_DSTAT_n[(i_0 << 1) + 1]) + 2.0F *
          rtP.lindi.sflt.omega * (2.0F * rtP.lindi.sflt.omega) * rtb_y_fg[i_0];
        rtb_y_dt_fn[1 + (i_0 << 1)] = rtDW.DiscreteTimeIntegrator1_DSTAT_n[i_0 <<
          1];
      }

      // :  k2 = f(y_0+0.5*h*k1,u,omega,d);
      q2_q2 = 0.5F * t;
      for (i_0 = 0; i_0 < 6; i_0++) {
        rtb_y0_k[i_0] = q2_q2 * rtb_y_dt_fn[i_0] +
          rtDW.DiscreteTimeIntegrator1_DSTAT_n[i_0];
      }

      // :  len_u = length(u);
      // :  y_vec_dt = zeros(2,len_u,superiorfloat(u));
      // :  for i = 1:len_u
      for (i_0 = 0; i_0 < 3; i_0++) {
        // :  y_vec_dt(:,i) = [ -2*d*omega, -omega^2; 1, 0 ] * y_0(:,i) + [omega^2;0] * u(i); 
        k2[i_0 << 1] = (-2.0F * scale * (2.0F * rtP.lindi.sflt.omega) *
                        rtb_y0_k[i_0 << 1] + -(2.0F * rtP.lindi.sflt.omega *
          (2.0F * rtP.lindi.sflt.omega)) * rtb_y0_k[(i_0 << 1) + 1]) + 2.0F *
          rtP.lindi.sflt.omega * (2.0F * rtP.lindi.sflt.omega) * rtb_y_fg[i_0];
        k2[1 + (i_0 << 1)] = rtb_y0_k[i_0 << 1];
      }

      // :  k3 = f(y_0+0.5*h*k2,u,omega,d);
      q2_q2 = 0.5F * t;
      for (stage_app_2 = 0; stage_app_2 < 6; stage_app_2++) {
        rtb_y0_k[stage_app_2] = q2_q2 * k2[stage_app_2] +
          rtDW.DiscreteTimeIntegrator1_DSTAT_n[stage_app_2];
      }

      // :  len_u = length(u);
      // :  y_vec_dt = zeros(2,len_u,superiorfloat(u));
      // :  for i = 1:len_u
      for (i_0 = 0; i_0 < 3; i_0++) {
        // :  y_vec_dt(:,i) = [ -2*d*omega, -omega^2; 1, 0 ] * y_0(:,i) + [omega^2;0] * u(i); 
        k3[i_0 << 1] = (-2.0F * scale * (2.0F * rtP.lindi.sflt.omega) *
                        rtb_y0_k[i_0 << 1] + -(2.0F * rtP.lindi.sflt.omega *
          (2.0F * rtP.lindi.sflt.omega)) * rtb_y0_k[(i_0 << 1) + 1]) + 2.0F *
          rtP.lindi.sflt.omega * (2.0F * rtP.lindi.sflt.omega) * rtb_y_fg[i_0];
        k3[1 + (i_0 << 1)] = rtb_y0_k[i_0 << 1];
      }

      // :  k4 = f(y_0+h*k3,u,omega,d);
      for (stage_app_2 = 0; stage_app_2 < 6; stage_app_2++) {
        rtb_y0_k[stage_app_2] = t * k3[stage_app_2] +
          rtDW.DiscreteTimeIntegrator1_DSTAT_n[stage_app_2];
      }

      // :  len_u = length(u);
      // :  y_vec_dt = zeros(2,len_u,superiorfloat(u));
      // :  for i = 1:len_u
      for (i_0 = 0; i_0 < 3; i_0++) {
        // :  y_vec_dt(:,i) = [ -2*d*omega, -omega^2; 1, 0 ] * y_0(:,i) + [omega^2;0] * u(i); 
        k4[i_0 << 1] = (-2.0F * scale * (2.0F * rtP.lindi.sflt.omega) *
                        rtb_y0_k[i_0 << 1] + -(2.0F * rtP.lindi.sflt.omega *
          (2.0F * rtP.lindi.sflt.omega)) * rtb_y0_k[(i_0 << 1) + 1]) + 2.0F *
          rtP.lindi.sflt.omega * (2.0F * rtP.lindi.sflt.omega) * rtb_y_fg[i_0];
        k4[1 + (i_0 << 1)] = rtb_y0_k[i_0 << 1];
      }

      // :  y_dt(:) = 1/6*(k1+2*k2+2*k3+k4);
      for (stage_app_2 = 0; stage_app_2 < 6; stage_app_2++) {
        rtb_y_dt_fn[stage_app_2] = (((2.0F * k2[stage_app_2] +
          rtb_y_dt_fn[stage_app_2]) + 2.0F * k3[stage_app_2]) + k4[stage_app_2])
          * 0.166666672F;
      }

      // End of MATLAB Function: '<S55>/PT2 discrete ode4'

      // SignalConversion: '<S47>/OutportBufferForOmega_Kb_f' incorporates:
      //   MATLAB Function: '<S55>/PT2 split'
      //   SignalConversion: '<S47>/OutportBufferForOmega_Kb_dt_f'

      PT2split(rtb_y_dt_fn, rtb_DiscreteTimeIntegrator1_k, rtb_y_fg, rtb_Sum2_cx,
               rtb_Sum2_g);

      // Update for DiscreteIntegrator: '<S56>/Discrete-Time Integrator1'
      rtDW.DiscreteTimeIntegrator1_IC_LOAD = 0U;

      // Update for DiscreteIntegrator: '<S56>/Discrete-Time Integrator'
      rtDW.DiscreteTimeIntegrator_DSTAT_o4 += 0.0025F;

      // Update for UnitDelay: '<S56>/Unit Delay'
      rtDW.UnitDelay_DSTATE_g = q1_q1;

      // Update for DiscreteIntegrator: '<S55>/Discrete-Time Integrator'
      rtDW.DiscreteTimeIntegrator_DSTATE_b += 0.0025F;

      // Update for UnitDelay: '<S55>/Unit Delay'
      rtDW.UnitDelay_DSTATE_l = absxk;

      // Update for DiscreteIntegrator: '<S55>/Discrete-Time Integrator1'
      rtDW.DiscreteTimeIntegrator1_IC_LO_g = 0U;
      for (stage_app_2 = 0; stage_app_2 < 6; stage_app_2++) {
        // Update for DiscreteIntegrator: '<S56>/Discrete-Time Integrator1'
        rtDW.DiscreteTimeIntegrator1_DSTAT_j[stage_app_2] += 0.0025F *
          rtb_y0[stage_app_2];

        // Update for DiscreteIntegrator: '<S55>/Discrete-Time Integrator1'
        rtDW.DiscreteTimeIntegrator1_DSTAT_n[stage_app_2] += 0.0025F *
          rtb_y_dt_fn[stage_app_2];
      }
    }

    // End of Outputs for SubSystem: '<S26>/Duplicate LPF'
    // End of Outputs for SubSystem: '<S26>/Single LPF'

    // Outputs for Enabled SubSystem: '<S9>/Demux' incorporates:
    //   EnablePort: '<S23>/Enable'

    // Outputs for Enabled SubSystem: '<S9>/Outer Loop INDI' incorporates:
    //   EnablePort: '<S29>/Enable'

    if (rtb_Compare_j) {
      if (!rtDW.OuterLoopINDI_MODE) {
        // InitializeConditions for UnitDelay: '<S29>/Unit Delay1'
        rtDW.UnitDelay1_DSTATE = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S69>/Discrete-Time Integrator' 
        rtDW.DiscreteTimeIntegrator_IC_LOA_l = 1U;
        rtDW.OuterLoopINDI_MODE = true;
      }

      // MATLAB Function: '<S67>/DCM to quaternions1' incorporates:
      //   DiscreteIntegrator: '<S48>/Discrete-Time Integrator y'

      // :  q_bg  = dcm2Quat( M_bg );
      // 'dcm2Quat:32' m_11 = M_bg(1,1);
      // 'dcm2Quat:33' m_12 = M_bg(1,2);
      // 'dcm2Quat:34' m_13 = M_bg(1,3);
      // 'dcm2Quat:35' m_21 = M_bg(2,1);
      // 'dcm2Quat:36' m_22 = M_bg(2,2);
      // 'dcm2Quat:37' m_23 = M_bg(2,3);
      // 'dcm2Quat:38' m_31 = M_bg(3,1);
      // 'dcm2Quat:39' m_32 = M_bg(3,2);
      // 'dcm2Quat:40' m_33 = M_bg(3,3);
      // 'dcm2Quat:43' sign_m23_minus = sign( m_23 - m_32 );
      // 'dcm2Quat:44' sign_m13_minus = sign( m_31 - m_13 );
      // 'dcm2Quat:45' sign_m12_minus = sign( m_12 - m_21 );
      // 'dcm2Quat:46' sign_m12_plus = sign( m_12 + m_21 );
      // 'dcm2Quat:47' sign_m23_plus = sign( m_23 + m_32 );
      // 'dcm2Quat:48' sign_m13_plus = sign( m_31 + m_13 );
      // 'dcm2Quat:52' q_0 = 1/2 * sqrtReal( 1 + m_11 + m_22 + m_33 );
      q2_q2 = ((1.0F + rtDW.DiscreteTimeIntegratory_DSTAT_b[0]) +
               rtDW.DiscreteTimeIntegratory_DSTAT_b[4]) +
        rtDW.DiscreteTimeIntegratory_DSTAT_b[8];

      // 'sqrtReal:27' if numel(y) > 1
      // 'sqrtReal:29' else
      // 'sqrtReal:30' y = max(0,y);
      // 'sqrtReal:32' y = sqrt(y);
      if (0.0F >= q2_q2) {
        q2_q2 = 0.0F;
      }

      q1_q1 = 0.5F * std::sqrt(q2_q2);

      // 'dcm2Quat:53' q_1 = 1/2 * sqrtReal( 1 + m_11 - m_22 - m_33 );
      q2_q2 = ((1.0F + rtDW.DiscreteTimeIntegratory_DSTAT_b[0]) -
               rtDW.DiscreteTimeIntegratory_DSTAT_b[4]) -
        rtDW.DiscreteTimeIntegratory_DSTAT_b[8];

      // 'sqrtReal:27' if numel(y) > 1
      // 'sqrtReal:29' else
      // 'sqrtReal:30' y = max(0,y);
      // 'sqrtReal:32' y = sqrt(y);
      if (0.0F >= q2_q2) {
        q2_q2 = 0.0F;
      }

      q2_q2 = 0.5F * std::sqrt(q2_q2);

      // 'dcm2Quat:54' q_2 = 1/2 * sqrtReal( 1 - m_11 + m_22 - m_33 );
      scale = ((1.0F - rtDW.DiscreteTimeIntegratory_DSTAT_b[0]) +
               rtDW.DiscreteTimeIntegratory_DSTAT_b[4]) -
        rtDW.DiscreteTimeIntegratory_DSTAT_b[8];

      // 'sqrtReal:27' if numel(y) > 1
      // 'sqrtReal:29' else
      // 'sqrtReal:30' y = max(0,y);
      // 'sqrtReal:32' y = sqrt(y);
      if (0.0F >= scale) {
        scale = 0.0F;
      }

      scale = 0.5F * std::sqrt(scale);

      // 'dcm2Quat:55' q_3 = 1/2 * sqrtReal( 1 - m_11 - m_22 + m_33 );
      absxk = ((1.0F - rtDW.DiscreteTimeIntegratory_DSTAT_b[0]) -
               rtDW.DiscreteTimeIntegratory_DSTAT_b[4]) +
        rtDW.DiscreteTimeIntegratory_DSTAT_b[8];

      // 'sqrtReal:27' if numel(y) > 1
      // 'sqrtReal:29' else
      // 'sqrtReal:30' y = max(0,y);
      // 'sqrtReal:32' y = sqrt(y);
      if (0.0F >= absxk) {
        absxk = 0.0F;
      }

      absxk = 0.5F * std::sqrt(absxk);

      // 'dcm2Quat:58' q_bg_unsigned = [ q_0; q_1; q_2; q_3 ];
      // 'dcm2Quat:59' [~, idx] = max(q_bg_unsigned);
      t = q1_q1;
      i_0 = -1;
      if (q1_q1 < q2_q2) {
        t = q2_q2;
        i_0 = 0;
      }

      if (t < scale) {
        t = scale;
        i_0 = 1;
      }

      if (t < absxk) {
        i_0 = 2;
      }

      // 'dcm2Quat:59' ~
      // 'dcm2Quat:60' idx = idx - 1;
      // 'dcm2Quat:61' switch idx
      switch (i_0 + 1) {
       case 0:
        // 'dcm2Quat:62' case 0
        // 'dcm2Quat:63' q_1 = sign_m23_minus * q_1;
        q1_q3 = rtDW.DiscreteTimeIntegratory_DSTAT_b[7] -
          rtDW.DiscreteTimeIntegratory_DSTAT_b[5];
        if (q1_q3 < 0.0F) {
          q1_q3 = -1.0F;
        } else {
          if (q1_q3 > 0.0F) {
            q1_q3 = 1.0F;
          }
        }

        q2_q2 *= q1_q3;

        // 'dcm2Quat:64' q_2 = sign_m13_minus * q_2;
        q1_q3 = rtDW.DiscreteTimeIntegratory_DSTAT_b[2] -
          rtDW.DiscreteTimeIntegratory_DSTAT_b[6];
        if (q1_q3 < 0.0F) {
          q1_q3 = -1.0F;
        } else {
          if (q1_q3 > 0.0F) {
            q1_q3 = 1.0F;
          }
        }

        scale *= q1_q3;

        // 'dcm2Quat:65' q_3 = sign_m12_minus * q_3;
        q1_q3 = rtDW.DiscreteTimeIntegratory_DSTAT_b[3] -
          rtDW.DiscreteTimeIntegratory_DSTAT_b[1];
        if (q1_q3 < 0.0F) {
          q1_q3 = -1.0F;
        } else {
          if (q1_q3 > 0.0F) {
            q1_q3 = 1.0F;
          }
        }

        absxk *= q1_q3;
        break;

       case 1:
        // 'dcm2Quat:66' case 1
        // 'dcm2Quat:67' q_0 = sign_m23_minus * q_0;
        t = rtDW.DiscreteTimeIntegratory_DSTAT_b[7] -
          rtDW.DiscreteTimeIntegratory_DSTAT_b[5];
        if (t < 0.0F) {
          t = -1.0F;
        } else {
          if (t > 0.0F) {
            t = 1.0F;
          }
        }

        q1_q1 *= t;

        // 'dcm2Quat:68' q_2 = sign_m12_plus * q_2;
        q1_q3 = rtDW.DiscreteTimeIntegratory_DSTAT_b[3] +
          rtDW.DiscreteTimeIntegratory_DSTAT_b[1];
        if (q1_q3 < 0.0F) {
          q1_q3 = -1.0F;
        } else {
          if (q1_q3 > 0.0F) {
            q1_q3 = 1.0F;
          }
        }

        scale *= q1_q3;

        // 'dcm2Quat:69' q_3 = sign_m13_plus * q_3;
        q1_q3 = rtDW.DiscreteTimeIntegratory_DSTAT_b[2] +
          rtDW.DiscreteTimeIntegratory_DSTAT_b[6];
        if (q1_q3 < 0.0F) {
          q1_q3 = -1.0F;
        } else {
          if (q1_q3 > 0.0F) {
            q1_q3 = 1.0F;
          }
        }

        absxk *= q1_q3;
        break;

       case 2:
        // 'dcm2Quat:70' case 2
        // 'dcm2Quat:71' q_0 = sign_m13_minus * q_0;
        t = rtDW.DiscreteTimeIntegratory_DSTAT_b[2] -
          rtDW.DiscreteTimeIntegratory_DSTAT_b[6];
        if (t < 0.0F) {
          t = -1.0F;
        } else {
          if (t > 0.0F) {
            t = 1.0F;
          }
        }

        q1_q1 *= t;

        // 'dcm2Quat:72' q_1 = sign_m12_plus * q_1;
        t = rtDW.DiscreteTimeIntegratory_DSTAT_b[3] +
          rtDW.DiscreteTimeIntegratory_DSTAT_b[1];
        if (t < 0.0F) {
          t = -1.0F;
        } else {
          if (t > 0.0F) {
            t = 1.0F;
          }
        }

        q2_q2 *= t;

        // 'dcm2Quat:73' q_3 = sign_m23_plus * q_3;
        q1_q3 = rtDW.DiscreteTimeIntegratory_DSTAT_b[7] +
          rtDW.DiscreteTimeIntegratory_DSTAT_b[5];
        if (q1_q3 < 0.0F) {
          q1_q3 = -1.0F;
        } else {
          if (q1_q3 > 0.0F) {
            q1_q3 = 1.0F;
          }
        }

        absxk *= q1_q3;
        break;

       case 3:
        // 'dcm2Quat:74' case 3
        // 'dcm2Quat:75' q_0 = sign_m12_minus * q_0;
        t = rtDW.DiscreteTimeIntegratory_DSTAT_b[3] -
          rtDW.DiscreteTimeIntegratory_DSTAT_b[1];
        if (t < 0.0F) {
          t = -1.0F;
        } else {
          if (t > 0.0F) {
            t = 1.0F;
          }
        }

        q1_q1 *= t;

        // 'dcm2Quat:76' q_1 = sign_m13_plus * q_1;
        t = rtDW.DiscreteTimeIntegratory_DSTAT_b[2] +
          rtDW.DiscreteTimeIntegratory_DSTAT_b[6];
        if (t < 0.0F) {
          t = -1.0F;
        } else {
          if (t > 0.0F) {
            t = 1.0F;
          }
        }

        q2_q2 *= t;

        // 'dcm2Quat:77' q_2 = sign_m23_plus * q_2;
        t = rtDW.DiscreteTimeIntegratory_DSTAT_b[7] +
          rtDW.DiscreteTimeIntegratory_DSTAT_b[5];
        if (t < 0.0F) {
          t = -1.0F;
        } else {
          if (t > 0.0F) {
            t = 1.0F;
          }
        }

        scale *= t;
        break;
      }

      // 'dcm2Quat:81' q_bg = [ q_0; q_1; q_2; q_3 ];
      q_bg_unsigned_idx_0 = q1_q1;
      q_bg_unsigned_idx_1 = q2_q2;
      q_bg_unsigned_idx_2 = scale;
      q_bg_unsigned_idx_3 = absxk;

      // 'dcm2Quat:84' q_bg = quatNormalize( q_bg );
      // 'quatNormalize:31' q_out = q / max( eps, norm(q, 2) );
      scale = 1.29246971E-26F;
      absxk = std::abs(q1_q1);
      if (absxk > 1.29246971E-26F) {
        q1_q1 = 1.0F;
        scale = absxk;
      } else {
        t = absxk / 1.29246971E-26F;
        q1_q1 = t * t;
      }

      absxk = std::abs(q2_q2);
      if (absxk > scale) {
        t = scale / absxk;
        q1_q1 = q1_q1 * t * t + 1.0F;
        scale = absxk;
      } else {
        t = absxk / scale;
        q1_q1 += t * t;
      }

      absxk = std::abs(q_bg_unsigned_idx_2);
      if (absxk > scale) {
        t = scale / absxk;
        q1_q1 = q1_q1 * t * t + 1.0F;
        scale = absxk;
      } else {
        t = absxk / scale;
        q1_q1 += t * t;
      }

      absxk = std::abs(q_bg_unsigned_idx_3);
      if (absxk > scale) {
        t = scale / absxk;
        q1_q1 = q1_q1 * t * t + 1.0F;
        scale = absxk;
      } else {
        t = absxk / scale;
        q1_q1 += t * t;
      }

      q1_q1 = scale * std::sqrt(q1_q1);
      if (2.22044605E-16F >= q1_q1) {
        q1_q1 = 2.22044605E-16F;
      }

      // MATLAB Function: '<S67>/Quaternions to Euler angles1'
      // :  EulerAngles  = quat2Euler( q_bg );
      // 'quat2Euler:33' q_bg = quatNormalize( q_bg );
      // 'quatNormalize:31' q_out = q / max( eps, norm(q, 2) );
      scale = 1.29246971E-26F;

      // MATLAB Function: '<S67>/DCM to quaternions1'
      rtb_Divide_h = q_bg_unsigned_idx_0 / q1_q1;

      // MATLAB Function: '<S67>/Quaternions to Euler angles1'
      absxk = std::abs(rtb_Divide_h);
      if (absxk > 1.29246971E-26F) {
        q2_q2 = 1.0F;
        scale = absxk;
      } else {
        t = absxk / 1.29246971E-26F;
        q2_q2 = t * t;
      }

      q_bg_unsigned_idx_0 = rtb_Divide_h;

      // MATLAB Function: '<S67>/DCM to quaternions1'
      rtb_Divide_h = q_bg_unsigned_idx_1 / q1_q1;

      // MATLAB Function: '<S67>/Quaternions to Euler angles1'
      absxk = std::abs(rtb_Divide_h);
      if (absxk > scale) {
        t = scale / absxk;
        q2_q2 = q2_q2 * t * t + 1.0F;
        scale = absxk;
      } else {
        t = absxk / scale;
        q2_q2 += t * t;
      }

      q_bg_unsigned_idx_1 = rtb_Divide_h;

      // MATLAB Function: '<S67>/DCM to quaternions1'
      rtb_Divide_h = q_bg_unsigned_idx_2 / q1_q1;

      // MATLAB Function: '<S67>/Quaternions to Euler angles1'
      absxk = std::abs(rtb_Divide_h);
      if (absxk > scale) {
        t = scale / absxk;
        q2_q2 = q2_q2 * t * t + 1.0F;
        scale = absxk;
      } else {
        t = absxk / scale;
        q2_q2 += t * t;
      }

      q_bg_unsigned_idx_2 = rtb_Divide_h;

      // MATLAB Function: '<S67>/DCM to quaternions1'
      rtb_Divide_h = q_bg_unsigned_idx_3 / q1_q1;

      // MATLAB Function: '<S67>/Quaternions to Euler angles1'
      absxk = std::abs(rtb_Divide_h);
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
        q1_q1 = q2_q2;
      } else {
        q1_q1 = 2.22044605E-16F;
      }

      q_bg_unsigned_idx_0 /= q1_q1;
      q_bg_unsigned_idx_1 /= q1_q1;
      q_bg_unsigned_idx_2 /= q1_q1;

      // MATLAB Function: '<S67>/Quaternions to Euler angles1'
      rtb_Divide_h /= q1_q1;

      // MATLAB Function: '<S29>/Outer Loop INDI' incorporates:
      //   DiscreteIntegrator: '<S48>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S49>/Discrete-Time Integrator y'

      // 'quat2Euler:36' q0 = q_bg(1);
      // 'quat2Euler:36' q1 = q_bg(2);
      // 'quat2Euler:36' q2 = q_bg(3);
      // 'quat2Euler:36' q3 = q_bg(4);
      // 'quat2Euler:38' q02 = powerFast(q0,2);
      // 'powerFast:3' c = a;
      // 'powerFast:4' for i = 2:b
      // 'powerFast:5' c = c .* a;
      // 'quat2Euler:39' q12 = powerFast(q1,2);
      // 'powerFast:3' c = a;
      // 'powerFast:4' for i = 2:b
      // 'powerFast:5' c = c .* a;
      // 'quat2Euler:40' q22 = powerFast(q2,2);
      // 'powerFast:3' c = a;
      // 'powerFast:4' for i = 2:b
      // 'powerFast:5' c = c .* a;
      // 'quat2Euler:41' q32 = powerFast(q3,2);
      // 'powerFast:3' c = a;
      // 'powerFast:4' for i = 2:b
      // 'powerFast:5' c = c .* a;
      // 'quat2Euler:44' c_23 = 2 * ( q2 * q3 + q0 * q1 );
      // 'quat2Euler:45' c_33 = q02 - q12 - q22 + q32;
      // 'quat2Euler:46' c_13 = 2 * ( q1 * q3 - q0 * q2 );
      // 'quat2Euler:47' c_12 = 2 * ( q1 * q2 + q0 * q3 );
      // 'quat2Euler:48' c_11 = q02 + q12 - q22 - q32;
      // 'quat2Euler:51' Phi = atan2( c_23, c_33 );
      // 'quat2Euler:52' Theta = - asinReal(c_13);
      // 'asinReal:28' if numel(y)>1
      // 'asinReal:31' else
      // 'asinReal:32' y = max(-1,min(1,y));
      // 'asinReal:34' y = asin(y);
      // 'quat2Euler:53' Psi = atan2( c_12, c_11 );
      // 'quat2Euler:56' EulerAngles = [ Phi; Theta; Psi ];
      // :  [ Delta_Phi, Delta_q, Delta_r, a_des_abs, a_abs ] = ...
      // :      indiPlaneAcc2PhiQR( nu_a_Kb_yz, V, M_bg, a_Kg, Delta_Phi_f );
      // 'indiPlaneAcc2PhiQR:36' method = 1;
      // 'indiPlaneAcc2PhiQR:39' a_Kb = M_bg*a_Kg;
      // 'indiPlaneAcc2PhiQR:40' a_Kb_yz = a_Kb(2:3);
      for (stage_app_2 = 0; stage_app_2 < 3; stage_app_2++) {
        rtb_y_a5[stage_app_2] = rtDW.DiscreteTimeIntegratory_DSTAT_b[stage_app_2
          + 6] * rtDW.DiscreteTimeIntegratory_DSTAT_n[2] +
          (rtDW.DiscreteTimeIntegratory_DSTAT_b[stage_app_2 + 3] *
           rtDW.DiscreteTimeIntegratory_DSTAT_n[1] +
           rtDW.DiscreteTimeIntegratory_DSTAT_b[stage_app_2] *
           rtDW.DiscreteTimeIntegratory_DSTAT_n[0]);
      }

      a_Kb_yz[0] = rtb_y_a5[1];
      a_Kb_yz[1] = rtb_y_a5[2];

      // 'indiPlaneAcc2PhiQR:44' g_g = [0;0;9.81];
      // 'indiPlaneAcc2PhiQR:46' g_b = M_bg*g_g;
      // 'indiPlaneAcc2PhiQR:47' g_b_yz = g_b(2:3);
      for (stage_app_2 = 0; stage_app_2 < 3; stage_app_2++) {
        circ_seg_n_0[stage_app_2] =
          rtDW.DiscreteTimeIntegratory_DSTAT_b[stage_app_2 + 6] * 9.81F;
      }

      g_b_yz[0] = circ_seg_n_0[1];
      g_b_yz[1] = circ_seg_n_0[2];

      // 'indiPlaneAcc2PhiQR:49' a_g = a_Kg - g_g;
      // 'indiPlaneAcc2PhiQR:52' a_b = M_bg*a_g;
      // 'indiPlaneAcc2PhiQR:53' a_b_yz = a_b(2:3);
      // 'indiPlaneAcc2PhiQR:56' a_abs = norm(a_b_yz);
      for (stage_app_2 = 0; stage_app_2 < 3; stage_app_2++) {
        rtb_V_Kg_n0[stage_app_2] =
          rtDW.DiscreteTimeIntegratory_DSTAT_b[stage_app_2 + 6] *
          (rtDW.DiscreteTimeIntegratory_DSTAT_n[2] - 9.81F) +
          (rtDW.DiscreteTimeIntegratory_DSTAT_b[stage_app_2 + 3] *
           rtDW.DiscreteTimeIntegratory_DSTAT_n[1] +
           rtDW.DiscreteTimeIntegratory_DSTAT_b[stage_app_2] *
           rtDW.DiscreteTimeIntegratory_DSTAT_n[0]);
      }

      tmp[0] = rtb_V_Kg_n0[1];
      tmp[1] = rtb_V_Kg_n0[2];
      t = norm(tmp);

      // 'indiPlaneAcc2PhiQR:59' a_b_yz_des = nu_a_Kb_yz - g_b_yz;
      // 'indiPlaneAcc2PhiQR:61' a_des_abs = norm(a_b_yz_des);
      tmp[0] = rtDW.nu[0] - circ_seg_n_0[1];
      tmp[1] = rtDW.nu[1] - circ_seg_n_0[2];
      q0_q3 = norm(tmp);

      // 'indiPlaneAcc2PhiQR:64' if method == 1
      // 'indiPlaneAcc2PhiQR:66' g_abs = norm(g_b_yz);
      q1_q3 = norm(g_b_yz);

      // 'indiPlaneAcc2PhiQR:68' nu_abs = norm(nu_a_Kb_yz);
      q_bg_unsigned_idx_3 = norm(rtDW.nu);

      // 'indiPlaneAcc2PhiQR:70' a_K_abs = norm(a_Kb_yz);
      q1_q2 = norm(a_Kb_yz);

      // 'indiPlaneAcc2PhiQR:73' Phi_i = acosReal( divideFinite( g_abs^2 + a_abs^2 - a_K_abs^2, 2*g_abs*a_abs ) ); 
      rtb_Sum2_os = 2.0F * q1_q3 * t;

      // 'divideFinite:29' if numel(B)>1
      // 'divideFinite:31' else
      // 'divideFinite:32' if abs(B)<eps
      if (std::abs(rtb_Sum2_os) < 2.22044605E-16F) {
        // 'divideFinite:33' B(:) = eps;
        rtb_Sum2_os = 2.22044605E-16F;
      }

      // 'divideFinite:36' C = A ./ B;
      q1_q1 = q1_q3 * q1_q3;
      q1_q2 = ((q1_q1 + t * t) - q1_q2 * q1_q2) / rtb_Sum2_os;

      // 'acosReal:28' if numel(y) > 1
      // 'acosReal:31' else
      // 'acosReal:32' y = max(-1,min(1,y));
      if (1.0F <= q1_q2) {
        q1_q2 = 1.0F;
      }

      // 'acosReal:34' y = acos(y);
      // 'indiPlaneAcc2PhiQR:75' forward = M_bg'*[1;0;0];
      for (stage_app_2 = 0; stage_app_2 < 3; stage_app_2++) {
        rtb_Sum2_g[stage_app_2] = rtDW.DiscreteTimeIntegratory_DSTAT_b[3 *
          stage_app_2];
      }

      // 'indiPlaneAcc2PhiQR:76' cross_i_g = cross(forward,a_Kg);
      // 'indiPlaneAcc2PhiQR:77' Phi_i_sign = sign(cross_i_g(3));
      // 'indiPlaneAcc2PhiQR:78' Phi_i = Phi_i_sign*Phi_i;
      if (-1.0F < q1_q2) {
        rtb_Sum2_ms = q1_q2;
      } else {
        rtb_Sum2_ms = -1.0F;
      }

      t = rtb_Sum2_g[0] * rtDW.DiscreteTimeIntegratory_DSTAT_n[1] - rtb_Sum2_g[1]
        * rtDW.DiscreteTimeIntegratory_DSTAT_n[0];

      // 'indiPlaneAcc2PhiQR:81' Phi_des = acosReal( divideFinite( g_abs^2 + a_des_abs^2 - nu_abs^2, 2*g_abs*a_des_abs ) ); 
      q1_q2 = 2.0F * q1_q3 * q0_q3;

      // 'divideFinite:29' if numel(B)>1
      // 'divideFinite:31' else
      // 'divideFinite:32' if abs(B)<eps
      if (std::abs(q1_q2) < 2.22044605E-16F) {
        // 'divideFinite:33' B(:) = eps;
        q1_q2 = 2.22044605E-16F;
      }

      // 'divideFinite:36' C = A ./ B;
      q1_q2 = ((q1_q1 + q0_q3 * q0_q3) - q_bg_unsigned_idx_3 *
               q_bg_unsigned_idx_3) / q1_q2;

      // 'acosReal:28' if numel(y) > 1
      // 'acosReal:31' else
      // 'acosReal:32' y = max(-1,min(1,y));
      if (1.0F <= q1_q2) {
        q1_q2 = 1.0F;
      }

      // 'acosReal:34' y = acos(y);
      // 'indiPlaneAcc2PhiQR:83' cross_des_g = cross([0;g_b_yz],[0;nu_a_Kb_yz]); 
      // 'indiPlaneAcc2PhiQR:84' Phi_des_sign = -sign(cross_des_g(1));
      // 'indiPlaneAcc2PhiQR:85' Phi_des = Phi_des_sign*Phi_des;
      // 'indiPlaneAcc2PhiQR:88' Delta_Phi = Phi_des - Phi_i;
      // 'indiPlaneAcc2PhiQR:109' M_fb = [cos(Delta_Phi_f),sin(Delta_Phi_f);-sin(Delta_Phi_f),cos(Delta_Phi_f)]; 
      // 'indiPlaneAcc2PhiQR:112' nu_a_Kf_yz = M_fb*nu_a_Kb_yz;
      // 'indiPlaneAcc2PhiQR:114' if method == 1
      // 'indiPlaneAcc2PhiQR:116' omega_i = divideFinite( a_Kb_yz, V );
      q1_q3 = q0_q0;

      // 'divideFinite:29' if numel(B)>1
      // 'divideFinite:31' else
      // 'divideFinite:32' if abs(B)<eps
      rtb_y_p3 = std::abs(q0_q0);
      if (rtb_y_p3 < 2.22044605E-16F) {
        // 'divideFinite:33' B(:) = eps;
        q1_q3 = 2.22044605E-16F;
      }

      // 'divideFinite:36' C = A ./ B;
      a_Kb_yz[0] = rtb_y_a5[1] / q1_q3;

      // MATLAB Function: '<S29>/Outer Loop INDI' incorporates:
      //   UnitDelay: '<S29>/Unit Delay1'

      q1_q1 = rtb_y_a5[2] / q1_q3;

      // 'indiPlaneAcc2PhiQR:117' q_i = -omega_i(2);
      // 'indiPlaneAcc2PhiQR:118' r_i = omega_i(1);
      // 'indiPlaneAcc2PhiQR:121' omega_des = divideFinite( nu_a_Kf_yz, V );
      q1_q3 = q0_q0;

      // 'divideFinite:29' if numel(B)>1
      // 'divideFinite:31' else
      // 'divideFinite:32' if abs(B)<eps
      if (rtb_y_p3 < 2.22044605E-16F) {
        // 'divideFinite:33' B(:) = eps;
        q1_q3 = 2.22044605E-16F;
      }

      // 'divideFinite:36' C = A ./ B;
      scale = std::cos(rtDW.UnitDelay1_DSTATE);
      q2_q2 = std::sin(rtDW.UnitDelay1_DSTATE);

      // 'indiPlaneAcc2PhiQR:122' q_des = -omega_des(2);
      // 'indiPlaneAcc2PhiQR:123' r_des = omega_des(1);
      // 'indiPlaneAcc2PhiQR:126' Delta_q = q_des - q_i;
      // 'indiPlaneAcc2PhiQR:127' Delta_r = r_des - r_i;
      absxk = circ_seg_n_0[1] * rtDW.nu[1] - circ_seg_n_0[2] * rtDW.nu[0];
      if (absxk < 0.0F) {
        absxk = -1.0F;
      } else {
        if (absxk > 0.0F) {
          absxk = 1.0F;
        }
      }

      if (-1.0F >= q1_q2) {
        q1_q2 = -1.0F;
      }

      if (t < 0.0F) {
        t = -1.0F;
      } else {
        if (t > 0.0F) {
          t = 1.0F;
        }
      }

      q1_q2 = -absxk * std::acos(q1_q2) - t * std::acos(rtb_Sum2_ms);

      // DiscreteIntegrator: '<S69>/Discrete-Time Integrator'
      if (rtDW.DiscreteTimeIntegrator_IC_LOA_l != 0) {
        rtDW.DiscreteTimeIntegrator_DSTATE_f = q1_q2;
      }

      // Update for UnitDelay: '<S29>/Unit Delay1' incorporates:
      //   DiscreteIntegrator: '<S69>/Discrete-Time Integrator'

      rtDW.UnitDelay1_DSTATE = rtDW.DiscreteTimeIntegrator_DSTATE_f;

      // Update for DiscreteIntegrator: '<S69>/Discrete-Time Integrator' incorporates:
      //   Constant: '<S69>/T'
      //   Product: '<S69>/Divide'
      //   Sum: '<S69>/Sum2'

      rtDW.DiscreteTimeIntegrator_IC_LOA_l = 0U;
      rtDW.DiscreteTimeIntegrator_DSTATE_f += (q1_q2 -
        rtDW.DiscreteTimeIntegrator_DSTATE_f) / (1.0F / rtP.lindi.atc.rm.rfreq) *
        0.0025F;

      // SignalConversion: '<S23>/OutportBufferForPhi_d' incorporates:
      //   MATLAB Function: '<S67>/Quaternions to Euler angles1'
      //   Sum: '<S29>/Add7'

      rtDW.Merge2 = std::atan2((q_bg_unsigned_idx_2 * rtb_Divide_h +
        q_bg_unsigned_idx_0 * q_bg_unsigned_idx_1) * 2.0F, ((q_bg_unsigned_idx_0
        * q_bg_unsigned_idx_0 - q_bg_unsigned_idx_1 * q_bg_unsigned_idx_1) -
        q_bg_unsigned_idx_2 * q_bg_unsigned_idx_2) + rtb_Divide_h * rtb_Divide_h)
        + q1_q2;

      // SignalConversion: '<S23>/OutportBufferForq_d' incorporates:
      //   MATLAB Function: '<S29>/Outer Loop INDI'
      //   Sum: '<S29>/Add8'

      rtDW.Merge = (-((-q2_q2 * rtDW.nu[0] + scale * rtDW.nu[1]) / q1_q3) -
                    (-q1_q1)) + rtb_y_fg[1];

      // SignalConversion: '<S23>/OutportBufferForr_d' incorporates:
      //   MATLAB Function: '<S29>/Outer Loop INDI'
      //   Sum: '<S29>/Add9'

      rtDW.Merge1 = ((scale * rtDW.nu[0] + q2_q2 * rtDW.nu[1]) / q1_q3 -
                     a_Kb_yz[0]) + rtb_y_fg[2];
    } else {
      if (rtDW.OuterLoopINDI_MODE) {
        rtDW.OuterLoopINDI_MODE = false;
      }
    }

    // End of Outputs for SubSystem: '<S9>/Outer Loop INDI'
    // End of Outputs for SubSystem: '<S9>/Demux'

    // Outputs for Enabled SubSystem: '<S9>/Cmd 2 Roll Angle' incorporates:
    //   EnablePort: '<S21>/Enable'

    if (rtb_Compare > 0) {
      // Gain: '<S21>/Gain' incorporates:
      //   Inport: '<Root>/cmd'

      rtDW.Merge2 = rtP.lindi.atc.rm.rangmax * 3.14159274F / 180.0F *
        rtU.cmd.roll;
    }

    // End of Outputs for SubSystem: '<S9>/Cmd 2 Roll Angle'

    // Saturate: '<S33>/Saturation' incorporates:
    //   Constant: '<S140>/Constant'
    //   Constant: '<S146>/Constant'
    //   Constant: '<S26>/Constant1'
    //   Constant: '<S46>/Constant'
    //   Inport: '<S148>/In1'
    //   Inport: '<S148>/In2'
    //   Inport: '<S148>/In3'
    //   MATLAB Function: '<S29>/Outer Loop INDI'
    //   MATLAB Function: '<S37>/WpNav Matching'
    //   MATLAB Function: '<S67>/DCM to quaternions1'
    //   MATLAB Function: '<S67>/Quaternions to Euler angles1'
    //   RelationalOperator: '<S146>/Compare'
    //   RelationalOperator: '<S46>/Compare'

    q1_q1 = -rtP.lindi.atc.rm.rangmax * 3.14159274F / 180.0F;
    q1_q3 = rtP.lindi.atc.rm.rangmax * 3.14159274F / 180.0F;
    if (rtDW.Merge2 > q1_q3) {
      q1_q1 = q1_q3;
    } else {
      if (rtDW.Merge2 >= q1_q1) {
        q1_q1 = rtDW.Merge2;
      }
    }

    // End of Saturate: '<S33>/Saturation'

    // MATLAB Function: '<S33>/Avoid Angle Steps' incorporates:
    //   UnitDelay: '<S33>/Unit Delay'

    // :  Delta_angle = angle - angle_last;
    q2_q2 = q1_q1 - rtDW.UnitDelay_DSTATE;

    // :  if Delta_angle > deg2rad(180)
    // 'deg2rad:11' angle_rad = angle_deg * pi/180;
    if (q2_q2 > 3.1415926535897931) {
      // :  angle_cum = angle - deg2rad(360);
      // 'deg2rad:11' angle_rad = angle_deg * pi/180;
      q1_q1 -= 6.28318548F;
    } else {
      // 'deg2rad:11' angle_rad = angle_deg * pi/180;
      if (q2_q2 < -3.1415926535897931) {
        // :  elseif Delta_angle < -deg2rad(180)
        // :  angle_cum = angle + deg2rad(360);
        // 'deg2rad:11' angle_rad = angle_deg * pi/180;
        q1_q1 += 6.28318548F;
      } else {
        // :  else
        // :  angle_cum = angle_last + Delta_angle;
        q1_q1 = rtDW.UnitDelay_DSTATE + q2_q2;
      }
    }

    // End of MATLAB Function: '<S33>/Avoid Angle Steps'

    // Outputs for Enabled SubSystem: '<S9>/Turn Coordination' incorporates:
    //   EnablePort: '<S36>/Enable'

    if (rtb_Compare > 0) {
      // MATLAB Function: '<S36>/Turn Coordination'
      q2_q2 = q1_q1;
      scale = q0_q0;

      // :  if abs(Phi) > 0.8*pi/2
      if (std::abs(q1_q1) > 1.2566370614359172) {
        // :  Phi = sign(Phi)*0.8*pi/2;
        if (q1_q1 < 0.0F) {
          q1_q3 = -1.0F;
        } else if (q1_q1 > 0.0F) {
          q1_q3 = 1.0F;
        } else {
          q1_q3 = q1_q1;
        }

        q2_q2 = q1_q3 * 0.8F * 3.14159274F / 2.0F;
      }

      // :  a = 9.81 * tan(Phi);
      // :  if V < 1
      if (q0_q0 < 1.0F) {
        // :  V(:) = 1;
        scale = 1.0F;
      }

      // :  omega = a/V;
      scale = 9.81F * std::tan(q2_q2) / scale;

      // :  q = omega * sin(Phi);
      rtDW.q = scale * std::sin(q2_q2);

      // Sum: '<S36>/Add' incorporates:
      //   Gain: '<S9>/Cmd 2 Yaw Rate'
      //   Inport: '<Root>/cmd'
      //   MATLAB Function: '<S36>/Turn Coordination'

      // :  r = omega * cos(Phi);
      rtDW.Merge1 = scale * std::cos(q2_q2) + rtP.lindi.atc.rm.ydecaytc *
        rtU.cmd.yaw;
    }

    // End of Outputs for SubSystem: '<S9>/Turn Coordination'

    // MATLAB Function: '<S26>/Rotations matrix to Euler angles' incorporates:
    //   DiscreteIntegrator: '<S48>/Discrete-Time Integrator y'

    // :  EulerAngles  = dcm2Euler( M_bg );
    // 'dcm2Euler:30' Phi = atan2( M_bg(2,3), M_bg(3,3) );
    // 'dcm2Euler:31' Theta = -asinReal( M_bg(1,3) );
    // 'asinReal:28' if numel(y)>1
    // 'asinReal:31' else
    // 'asinReal:32' y = max(-1,min(1,y));
    if (1.0F > rtDW.DiscreteTimeIntegratory_DSTAT_b[6]) {
      q1_q2 = rtDW.DiscreteTimeIntegratory_DSTAT_b[6];
    } else {
      q1_q2 = 1.0F;
    }

    // 'asinReal:34' y = asin(y);
    // 'dcm2Euler:32' Psi = atan2( M_bg(1,2), M_bg(1,1) );
    // 'dcm2Euler:35' EulerAngles = [ Phi; Theta; Psi ];
    rtb_Sum2_ms = std::atan2(rtDW.DiscreteTimeIntegratory_DSTAT_b[7],
      rtDW.DiscreteTimeIntegratory_DSTAT_b[8]);
    if (-1.0F >= q1_q2) {
      q1_q2 = -1.0F;
    }

    rtb_y_p3 = std::asin(q1_q2);

    // Outputs for Enabled SubSystem: '<S9>/add' incorporates:
    //   EnablePort: '<S39>/Enable'

    // Outputs for Enabled SubSystem: '<S9>/Pitch Angle Controller' incorporates:
    //   EnablePort: '<S30>/Enable'

    if (rtb_Compare > 0) {
      if (!rtDW.PitchAngleController_MODE) {
        // InitializeConditions for DiscreteIntegrator: '<S72>/Discrete-Time Integrator' 
        rtDW.DiscreteTimeIntegrator_IC_LOA_m = 1U;

        // InitializeConditions for DiscreteIntegrator: '<S73>/Discrete-Time Integrator' 
        rtDW.DiscreteTimeIntegrator_IC_LO_mz = 1U;

        // InitializeConditions for DiscreteIntegrator: '<S76>/Discrete-Time Integrator y' 
        rtDW.DiscreteTimeIntegratory_IC_LO_l = 1U;

        // InitializeConditions for DiscreteIntegrator: '<S76>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_D_i2 = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S75>/Discrete-Time Integrator y' 
        rtDW.DiscreteTimeIntegratory_IC_L_id = 1U;

        // InitializeConditions for DiscreteIntegrator: '<S75>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_D_bo = 0.0F;

        // InitializeConditions for Delay: '<S77>/Delay'
        rtDW.icLoad_fe = 1U;

        // InitializeConditions for Delay: '<S77>/Delay1'
        rtDW.icLoad_lk = 1U;

        // InitializeConditions for Delay: '<S77>/Delay2'
        rtDW.icLoad_jd = 1U;

        // InitializeConditions for Delay: '<S77>/Delay3'
        rtDW.icLoad_f13 = 1U;
        rtDW.PitchAngleController_MODE = true;
      }

      // Gain: '<S30>/cmd 2 angle' incorporates:
      //   Inport: '<Root>/cmd'

      q2_q2 = rtP.lindi.atc.rm.pangmax * 3.14159274F / 180.0F * rtU.cmd.pitch;

      // DiscreteIntegrator: '<S72>/Discrete-Time Integrator'
      if (rtDW.DiscreteTimeIntegrator_IC_LOA_m != 0) {
        rtDW.DiscreteTimeIntegrator_DSTATE_p = q2_q2;
      }

      // Saturate: '<S72>/Saturation' incorporates:
      //   Constant: '<S72>/T'
      //   DiscreteIntegrator: '<S72>/Discrete-Time Integrator'
      //   Product: '<S72>/Divide'
      //   Sum: '<S72>/Sum2'

      q_bg_unsigned_idx_3 = (q2_q2 - rtDW.DiscreteTimeIntegrator_DSTATE_p) /
        (1.0F / rtP.lindi.atc.rm.pfreq);

      // DiscreteIntegrator: '<S73>/Discrete-Time Integrator' incorporates:
      //   DiscreteIntegrator: '<S72>/Discrete-Time Integrator'

      if (rtDW.DiscreteTimeIntegrator_IC_LO_mz != 0) {
        rtDW.DiscreteTimeIntegrator_DSTAT_ar =
          rtDW.DiscreteTimeIntegrator_DSTATE_p;
      }

      q2_q2 = rtDW.DiscreteTimeIntegrator_DSTAT_ar;

      // Sum: '<S73>/Sum2' incorporates:
      //   DiscreteIntegrator: '<S72>/Discrete-Time Integrator'
      //   DiscreteIntegrator: '<S73>/Discrete-Time Integrator'

      scale = rtDW.DiscreteTimeIntegrator_DSTATE_p -
        rtDW.DiscreteTimeIntegrator_DSTAT_ar;

      // DiscreteIntegrator: '<S76>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S73>/Discrete-Time Integrator'

      if (rtDW.DiscreteTimeIntegratory_IC_LO_l != 0) {
        rtDW.DiscreteTimeIntegratory_DSTAT_o =
          rtDW.DiscreteTimeIntegrator_DSTAT_ar;
      }

      // DiscreteIntegrator: '<S75>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S73>/Discrete-Time Integrator'

      if (rtDW.DiscreteTimeIntegratory_IC_L_id != 0) {
        rtDW.DiscreteTimeIntegratory_DSTA_bd =
          rtDW.DiscreteTimeIntegrator_DSTAT_ar;
      }

      // Sum: '<S76>/Sum2' incorporates:
      //   Constant: '<S76>/d'
      //   Constant: '<S76>/omega'
      //   DiscreteIntegrator: '<S75>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S76>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S76>/Discrete-Time Integrator y_dt'
      //   Gain: '<S76>/Gain'
      //   Product: '<S76>/Divide'
      //   Product: '<S76>/Product2'
      //   Sum: '<S76>/Sum3'

      absxk = rtDW.DiscreteTimeIntegratory_DSTA_bd - (rtP.lindi.sflt.d /
        rtP.lindi.sflt.omega * 2.0F * rtDW.DiscreteTimeIntegratory_dt_D_i2 +
        rtDW.DiscreteTimeIntegratory_DSTAT_o);

      // Delay: '<S77>/Delay' incorporates:
      //   DiscreteIntegrator: '<S73>/Discrete-Time Integrator'

      if (rtDW.icLoad_fe != 0) {
        rtDW.Delay_DSTATE_b = rtDW.DiscreteTimeIntegrator_DSTAT_ar;
      }

      t = rtDW.Delay_DSTATE_b;

      // Delay: '<S77>/Delay1' incorporates:
      //   DiscreteIntegrator: '<S73>/Discrete-Time Integrator'

      if (rtDW.icLoad_lk != 0) {
        rtDW.Delay1_DSTATE_m = rtDW.DiscreteTimeIntegrator_DSTAT_ar;
      }

      q0_q3 = rtDW.Delay1_DSTATE_m;

      // Delay: '<S77>/Delay2' incorporates:
      //   DiscreteIntegrator: '<S73>/Discrete-Time Integrator'

      if (rtDW.icLoad_jd != 0) {
        rtDW.Delay2_DSTATE_a = rtDW.DiscreteTimeIntegrator_DSTAT_ar;
      }

      q1_q2 = rtDW.Delay2_DSTATE_a;

      // Delay: '<S77>/Delay3' incorporates:
      //   DiscreteIntegrator: '<S73>/Discrete-Time Integrator'

      if (rtDW.icLoad_f13 != 0) {
        rtDW.Delay3_DSTATE_mn = rtDW.DiscreteTimeIntegrator_DSTAT_ar;
      }

      // MATLAB Function: '<S77>/MATLAB Function' incorporates:
      //   Constant: '<S77>/Constant'
      //   Constant: '<S77>/Constant1'
      //   Delay: '<S77>/Delay'
      //   Delay: '<S77>/Delay1'
      //   Delay: '<S77>/Delay2'
      //   Delay: '<S77>/Delay3'
      //   DiscreteIntegrator: '<S73>/Discrete-Time Integrator'

      MATLABFunction(rtDW.DiscreteTimeIntegrator_DSTAT_ar, rtDW.Delay_DSTATE_b,
                     rtDW.Delay1_DSTATE_m, rtDW.Delay2_DSTATE_a,
                     rtDW.Delay3_DSTATE_mn, rtP.lindi.servo.delay, 0.0025F,
                     &rtb_y_e3);

      // Product: '<S75>/Divide' incorporates:
      //   Constant: '<S75>/omega'
      //   Product: '<S75>/omega^2'

      rtb_Sum2_os = rtP.lindi.servo.omega * rtP.lindi.servo.boost;

      // Sum: '<S75>/Sum2' incorporates:
      //   Constant: '<S75>/d'
      //   Constant: '<S75>/omega'
      //   DiscreteIntegrator: '<S75>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S75>/Discrete-Time Integrator y_dt'
      //   Gain: '<S75>/Gain'
      //   Product: '<S75>/Divide'
      //   Product: '<S75>/Product2'
      //   Sum: '<S75>/Sum3'

      q1_q3 = rtb_y_e3 - (rtP.lindi.servo.d / rtb_Sum2_os * 2.0F *
                          rtDW.DiscreteTimeIntegratory_dt_D_bo +
                          rtDW.DiscreteTimeIntegratory_DSTA_bd);

      // Sum: '<S30>/Add3' incorporates:
      //   DiscreteIntegrator: '<S76>/Discrete-Time Integrator y'
      //   MATLAB Function: '<S26>/Rotations matrix to Euler angles'

      rtDW.e_Theta = rtDW.DiscreteTimeIntegratory_DSTAT_o - (-rtb_y_p3);

      // Gain: '<S30>/Gain'
      rtDW.nu_q_dt_ptchcntrl = rtP.lindi.atc.k.pang * rtDW.e_Theta;

      // Update for DiscreteIntegrator: '<S72>/Discrete-Time Integrator'
      rtDW.DiscreteTimeIntegrator_IC_LOA_m = 0U;
      rtDW.DiscreteTimeIntegrator_DSTATE_p += 0.0025F * q_bg_unsigned_idx_3;

      // Update for DiscreteIntegrator: '<S73>/Discrete-Time Integrator' incorporates:
      //   Constant: '<S73>/T'
      //   Product: '<S73>/Divide'

      rtDW.DiscreteTimeIntegrator_IC_LO_mz = 0U;
      rtDW.DiscreteTimeIntegrator_DSTAT_ar += scale / (1.0F /
        rtP.lindi.atc.rm.pfreq) * 0.0025F;

      // Update for DiscreteIntegrator: '<S76>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S76>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_IC_LO_l = 0U;
      rtDW.DiscreteTimeIntegratory_DSTAT_o += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_D_i2;

      // Update for DiscreteIntegrator: '<S76>/Discrete-Time Integrator y_dt' incorporates:
      //   Constant: '<S76>/omega'
      //   Product: '<S76>/Product1'
      //   Product: '<S76>/omega^2'

      rtDW.DiscreteTimeIntegratory_dt_D_i2 += rtP.lindi.sflt.omega *
        rtP.lindi.sflt.omega * absxk * 0.0025F;

      // Update for DiscreteIntegrator: '<S75>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S75>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_IC_L_id = 0U;
      rtDW.DiscreteTimeIntegratory_DSTA_bd += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_D_bo;

      // Update for DiscreteIntegrator: '<S75>/Discrete-Time Integrator y_dt' incorporates:
      //   Product: '<S75>/Product1'
      //   Product: '<S75>/omega^2'

      rtDW.DiscreteTimeIntegratory_dt_D_bo += rtb_Sum2_os * rtb_Sum2_os * q1_q3 *
        0.0025F;

      // Update for Delay: '<S77>/Delay'
      rtDW.icLoad_fe = 0U;
      rtDW.Delay_DSTATE_b = q2_q2;

      // Update for Delay: '<S77>/Delay1'
      rtDW.icLoad_lk = 0U;
      rtDW.Delay1_DSTATE_m = t;

      // Update for Delay: '<S77>/Delay2'
      rtDW.icLoad_jd = 0U;
      rtDW.Delay2_DSTATE_a = q0_q3;

      // Update for Delay: '<S77>/Delay3'
      rtDW.icLoad_f13 = 0U;
      rtDW.Delay3_DSTATE_mn = q1_q2;

      // Sum: '<S39>/Add2'
      rtDW.Merge = rtDW.q + q_bg_unsigned_idx_3;
    } else {
      if (rtDW.PitchAngleController_MODE) {
        rtDW.PitchAngleController_MODE = false;
      }
    }

    // End of Outputs for SubSystem: '<S9>/Pitch Angle Controller'
    // End of Outputs for SubSystem: '<S9>/add'

    // DiscreteIntegrator: '<S80>/Discrete-Time Integrator'
    if (rtDW.DiscreteTimeIntegrator_IC_LOADI != 0) {
      rtDW.DiscreteTimeIntegrator_DSTATE_e = rtDW.Merge;
    }

    scale = rtDW.DiscreteTimeIntegrator_DSTATE_e;

    // RelationalOperator: '<S40>/Compare' incorporates:
    //   Constant: '<S25>/Constant'
    //   Constant: '<S40>/Constant'

    rtb_Compare_j = (rtP.lindi.mla.use == 0.0F);

    // Outputs for Enabled SubSystem: '<S25>/Subsystem' incorporates:
    //   EnablePort: '<S42>/Enable'

    if (rtb_Compare_j) {
      // SignalConversion: '<S42>/OutportBuffer_InsertedFor_u_d_at_inport_0' incorporates:
      //   Constant: '<S42>/Constant'

      for (i_0 = 0; i_0 < 10; i_0++) {
        rtDW.Merge_m[i_0] = rtP.lindi.ca.u_d[i_0];
      }

      // End of SignalConversion: '<S42>/OutportBuffer_InsertedFor_u_d_at_inport_0' 
    }

    // End of Outputs for SubSystem: '<S25>/Subsystem'

    // Outputs for Enabled SubSystem: '<S25>/Maneuver Load Alleviation' incorporates:
    //   EnablePort: '<S41>/Enable'

    if (!rtb_Compare_j) {
      if (!rtDW.ManeuverLoadAlleviation_MODE) {
        rtDW.ManeuverLoadAlleviation_MODE = true;
      }

      // MATLAB Function: '<S43>/MATLAB Function' incorporates:
      //   DiscreteIntegrator: '<S48>/Discrete-Time Integrator y'

      // :  g_b = M_bg*[0;0;9.81];
      // :  a_des_abs = q_ref*V_A + g_b(3);
      for (stage_app_2 = 0; stage_app_2 < 3; stage_app_2++) {
        rtb_y_a5[stage_app_2] = rtDW.DiscreteTimeIntegratory_DSTAT_b[stage_app_2
          + 6] * 9.81F;
      }

      // Sum: '<S41>/Add1' incorporates:
      //   Constant: '<S41>/Constant1'
      //   DiscreteIntegrator: '<S80>/Discrete-Time Integrator'
      //   Gain: '<S41>/Gain'
      //   Gain: '<S41>/Gain1'
      //   Inport: '<Root>/cmd'
      //   MATLAB Function: '<Root>/PWM to -1_1'
      //   MATLAB Function: '<S43>/MATLAB Function'
      //   Sum: '<S41>/Add'

      q2_q2 = (rtU.cmd.RC_pwm[9] - 1500.0F) / 500.0F * 9.81F +
        -((rtDW.DiscreteTimeIntegrator_DSTATE_e * q0_q0 + rtb_y_a5[2]) - 9.81F);

      // :  num_flaps = int32(length(y_cp_flap)-2);
      // :  Delta_u_d = zeros(1,num_flaps+2,class(nu_a_Kz));
      for (i_0 = 0; i_0 < 10; i_0++) {
        // Reshape: '<S41>/Reshape4' incorporates:
        //   MATLAB Function: '<S41>/Maneuver Load Alleviation'

        rtDW.Merge_m[i_0] = 0.0F;
      }

      // MATLAB Function: '<S41>/Maneuver Load Alleviation' incorporates:
      //   Constant: '<S41>/Constant'
      //   Constant: '<S41>/Constant2'
      //   Reshape: '<S41>/Reshape4'

      // :  u = zeros(num_flaps+1,1,class(nu_a_Kz));
      // :  ca.u_min = ca_main.u_min(1:num_flaps+1);
      // :  ca.u_max = ca_main.u_max(1:num_flaps+1);
      // :  ca.u_d = ca_main.u_d(1:num_flaps+1);
      // :  ca.W_v = ca_mla.W_v;
      // :  ca.W_u = ca_mla.W_u(1:num_flaps+1);
      // :  ca.gamma = ca_mla.gamma;
      // :  ca.W = ca_main.W(1:num_flaps+1);
      // :  ca.i_max = ca_mla.i_max;
      // :  G11_1 = zeros(1,num_flaps+1,class(nu_a_Kz));
      // :  G11_2 = zeros(1,num_flaps+1,class(nu_a_Kz));
      for (stage_app_2 = 0; stage_app_2 < 9; stage_app_2++) {
        I_b[stage_app_2] = 0.0F;
        G11_2[stage_app_2] = 0.0F;
      }

      // :  G11_1(1:num_flaps/2) = -G10(6,1:num_flaps/2).*abs(y_cp_flap(1:num_flaps/2)); 
      I_b[0] = -rtb_Product2_ba[5] * std::abs(rtP.lindi.cef.y[0]);
      I_b[1] = -rtb_Product2_ba[11] * std::abs(rtP.lindi.cef.y[1]);
      I_b[2] = -rtb_Product2_ba[17] * std::abs(rtP.lindi.cef.y[2]);
      I_b[3] = -rtb_Product2_ba[23] * std::abs(rtP.lindi.cef.y[3]);

      // :  G11_2(num_flaps/2+1:num_flaps) = -G10(6,num_flaps/2+1:num_flaps).*abs(y_cp_flap(num_flaps/2+1:num_flaps)); 
      G11_2[4] = -rtb_Product2_ba[29] * std::abs(rtP.lindi.cef.y[4]);
      G11_2[5] = -rtb_Product2_ba[35] * std::abs(rtP.lindi.cef.y[5]);
      G11_2[6] = -rtb_Product2_ba[41] * std::abs(rtP.lindi.cef.y[6]);
      G11_2[7] = -rtb_Product2_ba[47] * std::abs(rtP.lindi.cef.y[7]);

      // :  G11 = [G11_1;G11_2;G10(2,1:num_flaps+1);G10(6,1:num_flaps+1)];
      // :  wrbm = abs(y_np_wing) * nu_a_Kz/2;
      // :  Delta_nu = [ wrbm(:); 0; 0 ];
      // :  Delta_u_d(1:num_flaps+1) = caIndiWls( ca, G11, Delta_nu, u );
      // 'caIndiWls:54' Delta_u_d       = zeros( size(ca.u_d) );
      // 'caIndiWls:55' Delta_gamma     = 0;
      // 'caIndiWls:56' Delta_diag_W_v  = zeros( size(ca.W_v) );
      // 'caIndiWls:57' Delta_u_max     = abs(ca.u_max-ca.u_min);
      // 'caIndiWls:58' for i = 1:length(varargin)
      // 'caIndiWls:71' umin    = ca.u_min - u;
      // 'caIndiWls:72' umax    = ca.u_max - u;
      // 'caIndiWls:73' ud      = ca.u_d - u;
      // 'caIndiWls:74' u0      = 0.5 * (umin+umax);
      // 'caIndiWls:77' umin    = max( umin, -Delta_u_max );
      for (i_0 = 0; i_0 < 9; i_0++) {
        p2 = i_0 << 2;
        rtb_G11[p2] = I_b[i_0];
        rtb_G11[1 + p2] = G11_2[i_0];
        rtb_G11[2 + p2] = rtb_Product2_ba[6 * i_0 + 1];
        rtb_G11[3 + p2] = rtb_Product2_ba[6 * i_0 + 5];
        q1_q3 = std::abs(rtP.lindi.ca.u_max[i_0] - rtP.lindi.ca.u_min[i_0]);
        umax_0[i_0] = rtP.lindi.ca.u_max[i_0];
        if (rtP.lindi.ca.u_min[i_0] > -q1_q3) {
          z1[i_0] = rtP.lindi.ca.u_min[i_0];
        } else {
          z1[i_0] = -q1_q3;
        }

        I_b[i_0] = rtP.lindi.ca.u_min[i_0];
        G11_2[i_0] = -q1_q3;
        Delta_u_max[i_0] = q1_q3;
      }

      // 'caIndiWls:78' umax    = min( umax, Delta_u_max );
      for (i_0 = 0; i_0 < 9; i_0++) {
        if (umax_0[i_0] < Delta_u_max[i_0]) {
          G11_2[i_0] = umax_0[i_0];
        } else {
          G11_2[i_0] = Delta_u_max[i_0];
        }
      }

      // 'caIndiWls:81' gamma   = ca.gamma + Delta_gamma;
      // 'caIndiWls:82' ud      = ud + Delta_u_d;
      // 'caIndiWls:83' W_v     = diag( ca.W_v + Delta_diag_W_v );
      memset(&W_v[0], 0, sizeof(real32_T) << 4U);
      W_v[0] = rtP.lindi.mla.ca.W_v[0];
      W_v[5] = rtP.lindi.mla.ca.W_v[1];
      W_v[10] = rtP.lindi.mla.ca.W_v[2];
      W_v[15] = rtP.lindi.mla.ca.W_v[3];

      // 'caIndiWls:85' W_u     = diag( ca.W_u );
      memset(&W_u_0[0], 0, 81U * sizeof(real32_T));

      // 'caIndiWls:88' W       = zeros( length(ca.W_u), 1, superiorfloat(ca.W_u) ); 
      // 'caIndiWls:90' [ Delta_u, W, iter ] = wls_alloc( B, Delta_nu, umin, umax, ... 
      // 'caIndiWls:91'     W_v, W_u, ud, gamma, u0, W, ca.i_max );
      a_Kb_yz_0[0] = std::abs(rtP.lindi.mla.eta_np[0] * rtP.lindi.eig.b / 2.0F) *
        q2_q2 / 2.0F;
      a_Kb_yz_0[1] = std::abs(rtP.lindi.mla.eta_np[1] * rtP.lindi.eig.b / 2.0F) *
        q2_q2 / 2.0F;
      a_Kb_yz_0[2] = 0.0F;
      a_Kb_yz_0[3] = 0.0F;
      for (i_0 = 0; i_0 < 9; i_0++) {
        W_u_0[i_0 + 9 * i_0] = rtP.lindi.mla.ca.W_u[i_0];
        umax_1 = (I_b[i_0] + umax_0[i_0]) * 0.5F;
        Delta_u_max[i_0] = rtP.lindi.ca.u_d[i_0];
        I_b[i_0] = 0.0F;
        umax_0[i_0] = umax_1;
      }

      wls_alloc(rtb_G11, a_Kb_yz_0, z1, G11_2, W_v, W_u_0, Delta_u_max,
                rtP.lindi.mla.ca.gamma, umax_0, I_b, rtP.lindi.mla.ca.i_max);
      for (stage_app_2 = 0; stage_app_2 < 9; stage_app_2++) {
        rtDW.Merge_m[stage_app_2] = umax_0[stage_app_2];
      }
    } else {
      if (rtDW.ManeuverLoadAlleviation_MODE) {
        rtDW.ManeuverLoadAlleviation_MODE = false;
      }
    }

    // End of Outputs for SubSystem: '<S25>/Maneuver Load Alleviation'

    // MATLAB Function: '<S32>/Only rotational control effectiveness' incorporates:
    //   Constant: '<S32>/Constant'
    //   Logic: '<S25>/Logical Operator'

    // :  G4xX = G([1:3,6],:);
    for (stage_app_2 = 0; stage_app_2 < 10; stage_app_2++) {
      i_0 = stage_app_2 << 2;
      rtb_G4xX[i_0] = rtb_Product2_ba[6 * stage_app_2];
      rtb_G4xX[1 + i_0] = rtb_Product2_ba[6 * stage_app_2 + 1];
      rtb_G4xX[2 + i_0] = rtb_Product2_ba[6 * stage_app_2 + 2];
      rtb_G4xX[3 + i_0] = rtb_Product2_ba[6 * stage_app_2 + 5];
    }

    // :  if DLC_opt == 0
    if (rtP.lindi.dlc.opt == 0.0F) {
      // :  G4xX(2,1:end-2) = 0;
      for (stage_app_2 = 0; stage_app_2 < 8; stage_app_2++) {
        rtb_G4xX[1 + (stage_app_2 << 2)] = 0.0F;
      }

      // :  G4xX(4,:) = 0;
      for (stage_app_2 = 0; stage_app_2 < 10; stage_app_2++) {
        rtb_G4xX[3 + (stage_app_2 << 2)] = 0.0F;
      }
    }

    // End of MATLAB Function: '<S32>/Only rotational control effectiveness'

    // DiscreteIntegrator: '<S122>/Discrete-Time Integrator y'
    absxk = rtDW.DiscreteTimeIntegratory_DSTAT_p;

    // DiscreteIntegrator: '<S122>/Discrete-Time Integrator y_dt'
    t = rtDW.DiscreteTimeIntegratory_dt_DS_g;

    // Product: '<S122>/Product1' incorporates:
    //   Constant: '<S122>/d'
    //   Constant: '<S122>/omega'
    //   DiscreteIntegrator: '<S122>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S122>/Discrete-Time Integrator y_dt'
    //   Gain: '<S122>/Gain'
    //   Product: '<S122>/Divide'
    //   Product: '<S122>/Product2'
    //   Product: '<S122>/omega^2'
    //   Sum: '<S122>/Sum2'
    //   Sum: '<S122>/Sum3'

    q0_q3 = (q1_q1 - (1.0F / rtP.lindi.atc.rm.rfreq * 2.0F *
                      rtDW.DiscreteTimeIntegratory_dt_DS_g +
                      rtDW.DiscreteTimeIntegratory_DSTAT_p)) *
      (rtP.lindi.atc.rm.rfreq * rtP.lindi.atc.rm.rfreq);

    // DiscreteIntegrator: '<S134>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S122>/Discrete-Time Integrator y'

    if (rtDW.DiscreteTimeIntegratory_IC_LO_m != 0) {
      rtDW.DiscreteTimeIntegratory_DSTAT_c =
        rtDW.DiscreteTimeIntegratory_DSTAT_p;
    }

    // MATLAB Function: '<S121>/wrap angle' incorporates:
    //   DiscreteIntegrator: '<S134>/Discrete-Time Integrator y'

    wrapangle(rtDW.DiscreteTimeIntegratory_DSTAT_c, &q1_q3);

    // MATLAB Function: '<S121>/wrap angle1' incorporates:
    //   MATLAB Function: '<S26>/Rotations matrix to Euler angles'

    wrapangle(rtb_Sum2_ms, &q1_q2);

    // MATLAB Function: '<S121>/angle error'
    // :  error = errorAngle(angle_ref,angle);
    // 'errorAngle:22' error = angle_ref - angle;
    q2_q2 = q1_q3 - q1_q2;

    // 'errorAngle:25' if error > pi
    if (q2_q2 > 3.1415926535897931) {
      // 'errorAngle:26' error = error - 2*pi;
      q2_q2 -= 6.28318548F;
    } else {
      if (q2_q2 < -3.1415926535897931) {
        // 'errorAngle:27' elseif error < -pi
        // 'errorAngle:28' error = error + 2*pi;
        q2_q2 += 6.28318548F;
      }
    }

    // End of MATLAB Function: '<S121>/angle error'

    // DiscreteIntegrator: '<S133>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S122>/Discrete-Time Integrator y'

    if (rtDW.DiscreteTimeIntegratory_IC_LO_d != 0) {
      rtDW.DiscreteTimeIntegratory_DSTAT_h =
        rtDW.DiscreteTimeIntegratory_DSTAT_p;
    }

    // Product: '<S134>/Product1' incorporates:
    //   Constant: '<S134>/d'
    //   Constant: '<S134>/omega'
    //   DiscreteIntegrator: '<S133>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S134>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S134>/Discrete-Time Integrator y_dt'
    //   Gain: '<S134>/Gain'
    //   Product: '<S134>/Divide'
    //   Product: '<S134>/Product2'
    //   Product: '<S134>/omega^2'
    //   Sum: '<S134>/Sum2'
    //   Sum: '<S134>/Sum3'

    q1_q2 = (rtDW.DiscreteTimeIntegratory_DSTAT_h - (rtP.lindi.sflt.d /
              rtP.lindi.sflt.omega * 2.0F * rtDW.DiscreteTimeIntegratory_dt_DS_f
              + rtDW.DiscreteTimeIntegratory_DSTAT_c)) * (rtP.lindi.sflt.omega *
      rtP.lindi.sflt.omega);

    // DiscreteIntegrator: '<S130>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S122>/Discrete-Time Integrator y_dt'

    if (rtDW.DiscreteTimeIntegratory_IC_LO_o != 0) {
      rtDW.DiscreteTimeIntegratory_DSTA_jv =
        rtDW.DiscreteTimeIntegratory_dt_DS_g;
    }

    // Sum: '<S33>/Add2' incorporates:
    //   DiscreteIntegrator: '<S122>/Discrete-Time Integrator y_dt'
    //   DiscreteIntegrator: '<S130>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S134>/Discrete-Time Integrator y_dt'
    //   Gain: '<S121>/Gain1'
    //   Gain: '<S121>/Gain3'
    //   Gain: '<S121>/Gain5'
    //   Gain: '<S123>/Gain'
    //   Gain: '<S123>/Gain1'
    //   Gain: '<S123>/Gain3'
    //   Gain: '<S123>/Gain4'
    //   Product: '<S123>/Product'
    //   Product: '<S123>/Product1'
    //   Sum: '<S121>/Add'
    //   Sum: '<S121>/Add1'
    //   Sum: '<S121>/Add2'
    //   Sum: '<S123>/Add2'
    //   Sum: '<S33>/Add1'
    //   Sum: '<S33>/Add3'

    rtb_Sum2_os = ((((rtDW.DiscreteTimeIntegratory_dt_DS_f - rtb_y_fg[0]) *
                     rtP.lindi.atc.k.rrat + rtP.lindi.atc.k.rang * q2_q2) +
                    (q1_q2 - rtb_Sum2_cx[0]) * rtP.lindi.atc.k.racc) + q0_q3) -
      (rtP.lindi.eig.b / 2.0F * (rtDW.DiscreteTimeIntegratory_dt_DS_g -
        rtDW.DiscreteTimeIntegratory_DSTA_jv) * rtP.lindi.eig.clp * (q0_q0 *
        0.6125F) * (rtP.lindi.eig.s * rtP.lindi.eig.b) * (1.0F /
        rtP.lindi.ceb.ixx) + rtb_Sum2_cx[0]);

    // Product: '<S80>/Divide' incorporates:
    //   Constant: '<S80>/T'
    //   DiscreteIntegrator: '<S80>/Discrete-Time Integrator'
    //   Sum: '<S80>/Sum2'

    q_bg_unsigned_idx_3 = (rtDW.Merge - rtDW.DiscreteTimeIntegrator_DSTATE_e) /
      (1.0F / rtP.lindi.atc.rm.pfreq);

    // DiscreteIntegrator: '<S97>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S80>/Discrete-Time Integrator'

    if (rtDW.DiscreteTimeIntegratory_IC_L_eu != 0) {
      rtDW.DiscreteTimeIntegratory_DSTAT_k =
        rtDW.DiscreteTimeIntegrator_DSTATE_e;
    }

    // DiscreteIntegrator: '<S93>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S80>/Discrete-Time Integrator'

    if (rtDW.DiscreteTimeIntegratory_IC_LO_k != 0) {
      rtDW.DiscreteTimeIntegratory_DSTAT_f =
        rtDW.DiscreteTimeIntegrator_DSTATE_e;
    }

    // Sum: '<S81>/Add2' incorporates:
    //   DiscreteIntegrator: '<S80>/Discrete-Time Integrator'
    //   DiscreteIntegrator: '<S93>/Discrete-Time Integrator y'

    q_bg_unsigned_idx_0 = rtDW.DiscreteTimeIntegrator_DSTATE_e -
      rtDW.DiscreteTimeIntegratory_DSTAT_f;

    // Gain: '<S84>/Gain3' incorporates:
    //   DiscreteIntegrator: '<S89>/Discrete-Time Integrator'
    //   Product: '<S84>/Product1'

    q_bg_unsigned_idx_1 = rtDW.DiscreteTimeIntegrator_DSTATE_h * q0_q0 *
      rtP.lindi.ceb.m;

    // Sum: '<S9>/Add1' incorporates:
    //   UnitDelay: '<S34>/Unit Delay'

    q2_q2 = rtDW.Merge1 + rtDW.UnitDelay_DSTATE_f;

    // DiscreteIntegrator: '<S154>/Discrete-Time Integrator'
    if (rtDW.DiscreteTimeIntegrator_IC_LOA_g != 0) {
      rtDW.DiscreteTimeIntegrator_DSTATE_o = q2_q2;
    }

    q_bg_unsigned_idx_2 = rtDW.DiscreteTimeIntegrator_DSTATE_o;

    // Product: '<S154>/Divide' incorporates:
    //   Constant: '<S154>/T'
    //   DiscreteIntegrator: '<S154>/Discrete-Time Integrator'
    //   Sum: '<S154>/Sum2'

    rtb_Divide_h = (q2_q2 - rtDW.DiscreteTimeIntegrator_DSTATE_o) / (1.0F /
      rtP.lindi.atc.rm.yfreq);

    // DiscreteIntegrator: '<S157>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S154>/Discrete-Time Integrator'

    if (rtDW.DiscreteTimeIntegratory_IC_LO_f != 0) {
      rtDW.DiscreteTimeIntegratory_DSTAT_g =
        rtDW.DiscreteTimeIntegrator_DSTATE_o;
    }

    // Sum: '<S38>/Add2' incorporates:
    //   DiscreteIntegrator: '<S157>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S157>/Discrete-Time Integrator y_dt'
    //   Gain: '<S153>/Gain3'
    //   Gain: '<S153>/Gain5'
    //   Sum: '<S153>/Add'
    //   Sum: '<S153>/Add1'
    //   Sum: '<S153>/Add2'
    //   Sum: '<S38>/Add1'

    rtb_y_dm = (((rtDW.DiscreteTimeIntegratory_DSTAT_g - rtb_y_fg[2]) *
                 rtP.lindi.atc.k.yrat + (rtDW.DiscreteTimeIntegratory_dt_DS_n -
      rtb_Sum2_cx[2]) * rtP.lindi.atc.k.yacc) + rtb_Divide_h) - rtb_Sum2_cx[2];

    // Product: '<S24>/Matrix Multiply' incorporates:
    //   DiscreteIntegrator: '<S48>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S49>/Discrete-Time Integrator y'

    // :  Delta_nu_acc_z(:) = exp(-(0.9124/e_Theta_cutoff*abs(e_Theta)).^4) * Delta_nu_acc_z; 
    for (stage_app_2 = 0; stage_app_2 < 3; stage_app_2++) {
      rtb_y_a5[stage_app_2] = rtDW.DiscreteTimeIntegratory_DSTAT_b[stage_app_2 +
        6] * rtDW.DiscreteTimeIntegratory_DSTAT_n[2] +
        (rtDW.DiscreteTimeIntegratory_DSTAT_b[stage_app_2 + 3] *
         rtDW.DiscreteTimeIntegratory_DSTAT_n[1] +
         rtDW.DiscreteTimeIntegratory_DSTAT_b[stage_app_2] *
         rtDW.DiscreteTimeIntegratory_DSTAT_n[0]);
    }

    // MATLAB Function: '<S101>/MATLAB Function' incorporates:
    //   Constant: '<S101>/Constant1'
    //   DiscreteIntegrator: '<S80>/Discrete-Time Integrator'
    //   Gain: '<S113>/rad2deg'
    //   Gain: '<S24>/Gain1'
    //   Product: '<S24>/Matrix Multiply'
    //   Product: '<S24>/Product'
    //   Sum: '<S24>/Add'

    q2_q2 = std::exp(-std::pow(0.9124F / rtP.lindi.dlc.maxptch * std::abs
      (57.2957802F * rtDW.e_Theta), 4.0F)) *
      (-(rtDW.DiscreteTimeIntegrator_DSTATE_e * q0_q0) - rtb_y_a5[2]);

    // MATLAB Function: '<S32>/Set Delta_nu_acc_z' incorporates:
    //   Constant: '<S32>/Constant'

    // :  if DLC_opt < 1.5
    if (rtP.lindi.dlc.opt < 1.5F) {
      // :  Delta_nu_acc_z(:) = 0;
      q2_q2 = 0.0F;
    }

    // End of MATLAB Function: '<S32>/Set Delta_nu_acc_z'
    // :  [ Delta_u, W, iter ] = caIndiWls( ca, ...
    // :      B, Delta_nu, u, ...
    // :      'DeltaUd', Delta_u_d, 'DeltaGamma', Delta_gamma, ...
    // :      'DeltaDiagWv', Delta_diag_W_v, 'DeltaUmax', Delta_u_max );
    // 'caIndiWls:54' Delta_u_d       = zeros( size(ca.u_d) );
    // 'caIndiWls:55' Delta_gamma     = 0;
    // 'caIndiWls:56' Delta_diag_W_v  = zeros( size(ca.W_v) );
    // 'caIndiWls:57' Delta_u_max     = abs(ca.u_max-ca.u_min);
    // 'caIndiWls:58' for i = 1:length(varargin)
    // 'caIndiWls:59' if strcmp(varargin{i},'DeltaUd')
    // 'caIndiWls:60' Delta_u_d(:) = varargin{i+1};
    // 'caIndiWls:61' elseif strcmp(varargin{i},'DeltaGamma')
    // 'caIndiWls:62' Delta_gamma(:) = varargin{i+1};
    // 'caIndiWls:63' elseif strcmp(varargin{i},'DeltaDiagWv')
    // 'caIndiWls:64' Delta_diag_W_v(:) = varargin{i+1};
    // 'caIndiWls:65' elseif strcmp(varargin{i},'DeltaUmax')
    // 'caIndiWls:66' Delta_u_max(:) = varargin{i+1};
    // 'caIndiWls:71' umin    = ca.u_min - u;
    // 'caIndiWls:72' umax    = ca.u_max - u;
    // 'caIndiWls:73' ud      = ca.u_d - u;
    // 'caIndiWls:74' u0      = 0.5 * (umin+umax);
    // 'caIndiWls:77' umin    = max( umin, -Delta_u_max );
    for (itmp = 0; itmp < 10; itmp++) {
      // MATLAB Function: '<S103>/caIndiWls' incorporates:
      //   Constant: '<S103>/Delta u_max'
      //   DiscreteIntegrator: '<S106>/Discrete-Time Integrator y'

      rtb_y_e3 = rtP.lindi.ca.u_min[itmp] -
        rtDW.DiscreteTimeIntegratory_DSTAT_l[itmp];
      umax_1 = rtP.lindi.ca.u_max[itmp] -
        rtDW.DiscreteTimeIntegratory_DSTAT_l[itmp];
      rtb_Delta_u[itmp] = rtb_y_e3 + umax_1;
      q1_q3 = rtP.lindi.ca.u_max[itmp] - rtP.lindi.ca.u_min[itmp];
      umax[itmp] = umax_1;

      // MATLAB Function: '<S103>/caIndiWls' incorporates:
      //   Constant: '<S103>/Delta u_max'

      umax_1 = std::abs(q1_q3);
      if (rtb_y_e3 > -umax_1) {
        rtb_DiscreteTimeIntegrator1[itmp] = rtb_y_e3;
      } else {
        rtb_DiscreteTimeIntegrator1[itmp] = -umax_1;
      }

      rtb_Delay1[itmp] = q1_q3;
    }

    // MATLAB Function: '<S103>/caIndiWls' incorporates:
    //   Constant: '<S103>/Delta u_max'

    // 'caIndiWls:78' umax    = min( umax, Delta_u_max );
    for (i_0 = 0; i_0 < 10; i_0++) {
      for (itmp = 0; itmp < 10; itmp++) {
        rtb_Delay[itmp] = std::abs(rtb_Delay1[itmp]);
      }

      if (umax[i_0] < rtb_Delay[i_0]) {
        umin[i_0] = umax[i_0];
      } else {
        umin[i_0] = rtb_Delay[i_0];
      }
    }

    // 'caIndiWls:81' gamma   = ca.gamma + Delta_gamma;
    // 'caIndiWls:82' ud      = ud + Delta_u_d;
    // 'caIndiWls:83' W_v     = diag( ca.W_v + Delta_diag_W_v );
    memset(&W_v[0], 0, sizeof(real32_T) << 4U);
    W_v[0] = rtP.lindi.ca.W_v[0];
    W_v[5] = rtP.lindi.ca.W_v[1];
    W_v[10] = rtP.lindi.ca.W_v[2];
    W_v[15] = rtP.lindi.ca.W_v[3];

    // 'caIndiWls:85' W_u     = diag( ca.W_u );
    memset(&W_u[0], 0, 100U * sizeof(real32_T));

    // Sum: '<S32>/Add1' incorporates:
    //   DiscreteIntegrator: '<S115>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S116>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S91>/Discrete-Time Integrator'
    //   DiscreteIntegrator: '<S97>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S97>/Discrete-Time Integrator y_dt'
    //   Gain: '<S100>/Gain'
    //   Gain: '<S79>/Gain3'
    //   Gain: '<S79>/Gain5'
    //   Gain: '<S81>/Gain8'
    //   Gain: '<S83>/Gain5'
    //   Gain: '<S83>/Gain6'
    //   Gain: '<S83>/Gain7'
    //   Gain: '<S84>/Gain1'
    //   Product: '<S83>/Product2'
    //   Product: '<S83>/Product3'
    //   Sum: '<S27>/Add'
    //   Sum: '<S31>/Add1'
    //   Sum: '<S31>/Add2'
    //   Sum: '<S31>/Add9'
    //   Sum: '<S32>/Add'
    //   Sum: '<S79>/Add'
    //   Sum: '<S79>/Add1'
    //   Sum: '<S79>/Add2'
    //   Sum: '<S81>/Add5'
    //   Sum: '<S84>/Add5'
    //   UnitDelay: '<S100>/Unit Delay'

    // 'caIndiWls:88' W       = zeros( length(ca.W_u), 1, superiorfloat(ca.W_u) ); 
    // 'caIndiWls:90' [ Delta_u, W, iter ] = wls_alloc( B, Delta_nu, umin, umax, ... 
    // 'caIndiWls:91'     W_v, W_u, ud, gamma, u0, W, ca.i_max );
    a_Kb_yz_0[0] = rtb_Sum2_os;
    a_Kb_yz_0[1] = (((((rtDW.DiscreteTimeIntegratory_DSTAT_k - rtb_y_fg[1]) *
                       rtP.lindi.atc.k.prat +
                       (rtDW.DiscreteTimeIntegratory_dt_DS_j - rtb_Sum2_cx[1]) *
                       rtP.lindi.atc.k.pacc) + q_bg_unsigned_idx_3) -
                     ((rtP.lindi.eig.x_h * q_bg_unsigned_idx_0 *
                       -rtP.lindi.eig.cla_h * (q0_q0 * 0.6125F) *
                       (rtP.lindi.eig.s_h * rtP.lindi.eig.x_h) +
                       ((rtP.lindi.eig.xnp0 - rtP.lindi.eig.xcg) *
                        q_bg_unsigned_idx_1 +
                        rtDW.DiscreteTimeIntegrator_DSTAT_en)) * (1.0F /
      rtP.lindi.ceb.iyy) + rtb_Sum2_cx[1])) + rtDW.nu_q_dt_ptchcntrl) +
      rtDW.DiscreteTimeIntegratory_DSTA_na;
    a_Kb_yz_0[2] = rtb_y_dm;
    a_Kb_yz_0[3] = (-rtP.lindi.dlc.flapdecay * rtDW.UnitDelay_DSTATE_i + q2_q2)
      + rtDW.DiscreteTimeIntegratory_DSTAT_i;

    // MATLAB Function: '<S103>/caIndiWls' incorporates:
    //   DiscreteIntegrator: '<S106>/Discrete-Time Integrator y'

    for (i_0 = 0; i_0 < 10; i_0++) {
      W_u[i_0 + 10 * i_0] = rtP.lindi.ca.W_u[i_0];
      umax[i_0] = 0.0F;
      rtb_Delay[i_0] = (rtP.lindi.ca.u_d[i_0] -
                        rtDW.DiscreteTimeIntegratory_DSTAT_l[i_0]) +
        rtDW.Merge_m[i_0];
      rtb_Delta_u[i_0] *= 0.5F;
    }

    // SignalConversion: '<S9>/BusConversion_InsertedFor_ap_bus_at_inport_0' incorporates:
    //   MATLAB Function: '<S103>/caIndiWls'

    rtDW.iter = wls_alloc_c(rtb_G4xX, a_Kb_yz_0, rtb_DiscreteTimeIntegrator1,
      umin, W_v, W_u, rtb_Delay, rtP.lindi.ca.gamma, rtb_Delta_u, umax,
      rtP.lindi.ca.i_max);

    // Gain: '<S109>/Gain7'
    q2_q2 = rtP.lindi.servo.boost * rtP.lindi.servo.boost;

    // Gain: '<S109>/Gain6' incorporates:
    //   Gain: '<S109>/Gain8'
    //   Product: '<S106>/omega^2'

    rtb_Sum2_os = 2.0F * rtP.lindi.servo.d;
    umax_1 = rtP.lindi.servo.omega * rtP.lindi.servo.omega;
    q1_q3 = (rtb_Sum2_os / rtP.lindi.servo.omega - rtb_Sum2_os /
             (rtP.lindi.servo.boost * rtP.lindi.servo.omega) / umax_1 *
             rtP.lindi.servo.boost * rtP.lindi.servo.omega *
             rtP.lindi.servo.boost * rtP.lindi.servo.omega) *
      rtP.lindi.servo.boost * rtP.lindi.servo.omega * rtP.lindi.servo.boost *
      rtP.lindi.servo.omega;

    // Gain: '<S109>/Gain8'
    rtb_omega2_pb = (1.0F - 1.0F / umax_1 * rtP.lindi.servo.boost *
                     rtP.lindi.servo.omega * rtP.lindi.servo.boost *
                     rtP.lindi.servo.omega) * rtP.lindi.servo.boost *
      rtP.lindi.servo.omega * rtP.lindi.servo.boost * rtP.lindi.servo.omega;

    // SignalConversion: '<S9>/BusConversion_InsertedFor_ap_bus_at_inport_0' incorporates:
    //   Inport: '<Root>/cmd'

    rtDW.u[10] = rtU.cmd.thr;
    for (i_0 = 0; i_0 < 10; i_0++) {
      // Sum: '<S32>/Add6' incorporates:
      //   DiscreteIntegrator: '<S105>/Discrete-Time Integrator y'

      rtb_y_e3 = rtb_Delta_u[i_0] + rtDW.DiscreteTimeIntegratory_DSTA_j2[i_0];

      // DiscreteIntegrator: '<S109>/Discrete-Time Integrator1'
      rtb_DiscreteTimeIntegrator1[i_0] = rtDW.DiscreteTimeIntegrator1_DSTATE[i_0];

      // Sum: '<S109>/Add3' incorporates:
      //   DiscreteIntegrator: '<S109>/Discrete-Time Integrator1'
      //   DiscreteIntegrator: '<S109>/Discrete-Time Integrator2'
      //   Gain: '<S109>/Gain6'
      //   Gain: '<S109>/Gain7'
      //   Gain: '<S109>/Gain8'

      rtb_Sum2_os = (q2_q2 * rtb_y_e3 + q1_q3 *
                     rtDW.DiscreteTimeIntegrator1_DSTATE[i_0]) + rtb_omega2_pb *
        rtDW.DiscreteTimeIntegrator2_DSTATE[i_0];

      // Saturate: '<S32>/Saturation3'
      if (rtb_Sum2_os > rtP.lindi.ca.u_max[i_0]) {
        rtb_Sum2_os = rtP.lindi.ca.u_max[i_0];
      } else {
        if (rtb_Sum2_os < rtP.lindi.ca.u_min[i_0]) {
          rtb_Sum2_os = rtP.lindi.ca.u_min[i_0];
        }
      }

      // SignalConversion: '<S9>/BusConversion_InsertedFor_ap_bus_at_inport_0'
      rtDW.u[i_0] = rtb_Sum2_os;

      // Delay: '<S107>/Delay'
      if (rtDW.icLoad != 0) {
        rtDW.Delay_DSTATE[i_0] = rtb_Sum2_os;
      }

      rtb_Delay[i_0] = rtDW.Delay_DSTATE[i_0];

      // Delay: '<S107>/Delay1'
      if (rtDW.icLoad_i != 0) {
        rtDW.Delay1_DSTATE[i_0] = rtb_Sum2_os;
      }

      rtb_Delay1[i_0] = rtDW.Delay1_DSTATE[i_0];

      // Delay: '<S107>/Delay2'
      if (rtDW.icLoad_o != 0) {
        rtDW.Delay2_DSTATE[i_0] = rtb_Sum2_os;
      }

      rtb_Delay2[i_0] = rtDW.Delay2_DSTATE[i_0];

      // Delay: '<S107>/Delay3'
      if (rtDW.icLoad_k != 0) {
        rtDW.Delay3_DSTATE[i_0] = rtb_Sum2_os;
      }

      umin[i_0] = rtb_y_e3;
      umax[i_0] = rtb_Sum2_os;
    }

    // MATLAB Function: '<S107>/MATLAB Function' incorporates:
    //   Constant: '<S107>/Constant'
    //   Delay: '<S107>/Delay'
    //   Delay: '<S107>/Delay1'
    //   Delay: '<S107>/Delay2'
    //   Delay: '<S107>/Delay3'
    //   Saturate: '<S32>/Saturation3'
    //   Sum: '<S32>/Add6'

    // :  num_delays = floor( divideFinite( delay, ts ) );
    // 'divideFinite:29' if numel(B)>1
    // 'divideFinite:31' else
    // 'divideFinite:32' if abs(B)<eps
    // 'divideFinite:36' C = A ./ B;
    q2_q2 = std::floor(rtP.lindi.servo.delay / 0.0025F);

    // :  num_delays(:) = min( num_delays, 4 );
    if (q2_q2 > 4.0F) {
      q2_q2 = 4.0F;
    }

    // :  num_delays(:) = max( num_delays, 0 );
    if (q2_q2 < 0.0F) {
      q2_q2 = 0.0F;
    }

    // :  if num_delays == 0
    if (q2_q2 == 0.0F) {
      // :  u_delay = u0;
      for (i_0 = 0; i_0 < 10; i_0++) {
        rtb_u_delay[i_0] = umax[i_0];
      }
    } else if (q2_q2 == 1.0F) {
      // :  elseif num_delays == 1
      // :  u_delay = u1;
      for (i_0 = 0; i_0 < 10; i_0++) {
        rtb_u_delay[i_0] = rtDW.Delay_DSTATE[i_0];
      }
    } else if (q2_q2 == 2.0F) {
      // :  elseif num_delays == 2
      // :  u_delay = u2;
      for (i_0 = 0; i_0 < 10; i_0++) {
        rtb_u_delay[i_0] = rtDW.Delay1_DSTATE[i_0];
      }
    } else if (q2_q2 == 3.0F) {
      // :  elseif num_delays == 3
      // :  u_delay = u3;
      for (i_0 = 0; i_0 < 10; i_0++) {
        rtb_u_delay[i_0] = rtDW.Delay2_DSTATE[i_0];
      }
    } else {
      // :  else
      // :  u_delay = u4;
      for (i_0 = 0; i_0 < 10; i_0++) {
        rtb_u_delay[i_0] = rtDW.Delay3_DSTATE[i_0];
      }
    }

    // End of MATLAB Function: '<S107>/MATLAB Function'

    // Gain: '<S106>/Gain' incorporates:
    //   Constant: '<S106>/d'
    //   Constant: '<S106>/omega'
    //   Product: '<S106>/Divide'

    q2_q2 = rtP.lindi.servo.d / rtP.lindi.servo.omega * 2.0F;

    // Sum: '<S106>/Sum2' incorporates:
    //   DiscreteIntegrator: '<S106>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S106>/Discrete-Time Integrator y_dt'
    //   Product: '<S106>/Product2'
    //   Sum: '<S106>/Sum3'

    for (stage_app_2 = 0; stage_app_2 < 10; stage_app_2++) {
      rtb_u_delay[stage_app_2] -=
        rtDW.DiscreteTimeIntegratory_dt_DS_o[stage_app_2] * q2_q2 +
        rtDW.DiscreteTimeIntegratory_DSTAT_l[stage_app_2];
    }

    // End of Sum: '<S106>/Sum2'

    // Gain: '<S109>/Gain4'
    q2_q2 = -rtP.lindi.servo.boost * rtP.lindi.servo.omega *
      rtP.lindi.servo.boost * rtP.lindi.servo.omega;

    // Gain: '<S109>/Gain5'
    q1_q3 = -2.0F * rtP.lindi.servo.d * (rtP.lindi.servo.boost *
      rtP.lindi.servo.omega);

    // Gain: '<S105>/Gain' incorporates:
    //   Constant: '<S105>/d'
    //   Constant: '<S105>/omega'
    //   Product: '<S105>/Divide'

    rtb_y_e3 = rtP.lindi.sflt.d / rtP.lindi.sflt.omega * 2.0F;

    // Sum: '<S105>/Sum2' incorporates:
    //   DiscreteIntegrator: '<S105>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S105>/Discrete-Time Integrator y_dt'
    //   DiscreteIntegrator: '<S106>/Discrete-Time Integrator y'
    //   Gain: '<S105>/Gain'
    //   Product: '<S105>/Product2'
    //   Sum: '<S105>/Sum3'

    for (i_0 = 0; i_0 < 10; i_0++) {
      rtb_Sum2_ig[i_0] = rtDW.DiscreteTimeIntegratory_DSTAT_l[i_0] -
        (rtDW.DiscreteTimeIntegratory_dt_D_nz[i_0] * rtb_y_e3 +
         rtDW.DiscreteTimeIntegratory_DSTA_j2[i_0]);
    }

    // End of Sum: '<S105>/Sum2'

    // Product: '<S105>/omega^2' incorporates:
    //   Constant: '<S105>/omega'

    rtb_omega2_o_0 = rtP.lindi.sflt.omega * rtP.lindi.sflt.omega;

    // Gain: '<S48>/Gain' incorporates:
    //   Constant: '<S48>/d'
    //   Constant: '<S48>/omega'
    //   Product: '<S48>/Divide'

    rtb_y_e3 = rtP.lindi.sflt.d / rtP.lindi.sflt.omega * 2.0F;

    // Sum: '<S48>/Sum2' incorporates:
    //   DiscreteIntegrator: '<S48>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S48>/Discrete-Time Integrator y_dt'
    //   Gain: '<S48>/Gain'
    //   Product: '<S48>/Product2'
    //   Sum: '<S48>/Sum3'

    for (stage_app_2 = 0; stage_app_2 < 9; stage_app_2++) {
      rtb_M_bg[stage_app_2] -= rtDW.DiscreteTimeIntegratory_dt_D_ni[stage_app_2]
        * rtb_y_e3 + rtDW.DiscreteTimeIntegratory_DSTAT_b[stage_app_2];
    }

    // End of Sum: '<S48>/Sum2'

    // Product: '<S48>/omega^2' incorporates:
    //   Constant: '<S48>/omega'

    rtb_omega2_oa_0 = rtP.lindi.sflt.omega * rtP.lindi.sflt.omega;

    // Gain: '<S51>/Gain' incorporates:
    //   Constant: '<S51>/d'
    //   Constant: '<S51>/omega'
    //   Product: '<S51>/Divide'

    rtb_y_e3 = rtP.lindi.sflt.d / rtP.lindi.sflt.omega * 2.0F;

    // Product: '<S51>/omega^2' incorporates:
    //   Constant: '<S51>/omega'

    rtb_omega2_pb = rtP.lindi.sflt.omega * rtP.lindi.sflt.omega;

    // SignalConversion: '<S9>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.Omega_Kb_dt_f[0] = rtb_Sum2_cx[0];

    // SignalConversion: '<S9>/BusConversion_InsertedFor_ap_bus_at_inport_0' incorporates:
    //   MATLAB Function: '<S26>/Rotations matrix to Euler angles'

    rtDW.Euler_angles_f[0] = rtb_Sum2_ms;

    // SignalConversion: '<S9>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.Omega_Kb_f[0] = rtb_y_fg[0];

    // SignalConversion: '<S9>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.e_s_g[0] = rtDW.e_s_g_e[0];

    // SignalConversion: '<S9>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.e_s_g_dt[0] = rtDW.e_s_g_dt_i[0];

    // SignalConversion: '<S9>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.e_s_g_dt2[0] = rtDW.e_s_g_dt2_k[0];

    // SignalConversion: '<S9>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.s_g_ref[0] = rtDW.s_g_ref_d[0];

    // SignalConversion: '<S9>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.s_g[0] = rtDW.s_g_a[0];

    // SignalConversion: '<S9>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.s_g_match[0] = rtDW.s_g_match_i[0];
    rtb_y_fg[0] = (rtU.measure.V_Kg[0] - (rtDW.DiscreteTimeIntegratory_dt_DS_e[0]
      * rtb_y_e3 + rtDW.DiscreteTimeIntegratory_DSTAT_j[0])) * rtb_omega2_pb;

    // SignalConversion: '<S9>/BusConversion_InsertedFor_ap_bus_at_inport_0' incorporates:
    //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y_dt'
    //   Gain: '<S51>/Gain'
    //   Inport: '<Root>/measure'
    //   Product: '<S51>/Product1'
    //   Product: '<S51>/Product2'
    //   Product: '<S51>/omega^2'
    //   Sum: '<S51>/Sum2'
    //   Sum: '<S51>/Sum3'

    rtDW.Omega_Kb_dt_f[1] = rtb_Sum2_cx[1];

    // SignalConversion: '<S9>/BusConversion_InsertedFor_ap_bus_at_inport_0' incorporates:
    //   MATLAB Function: '<S26>/Rotations matrix to Euler angles'

    rtDW.Euler_angles_f[1] = -rtb_y_p3;

    // SignalConversion: '<S9>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.Omega_Kb_f[1] = rtb_y_fg[1];

    // SignalConversion: '<S9>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.e_s_g[1] = rtDW.e_s_g_e[1];

    // SignalConversion: '<S9>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.e_s_g_dt[1] = rtDW.e_s_g_dt_i[1];

    // SignalConversion: '<S9>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.e_s_g_dt2[1] = rtDW.e_s_g_dt2_k[1];

    // SignalConversion: '<S9>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.s_g_ref[1] = rtDW.s_g_ref_d[1];

    // SignalConversion: '<S9>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.s_g[1] = rtDW.s_g_a[1];

    // SignalConversion: '<S9>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.s_g_match[1] = rtDW.s_g_match_i[1];
    rtb_y_fg[1] = (rtU.measure.V_Kg[1] - (rtDW.DiscreteTimeIntegratory_dt_DS_e[1]
      * rtb_y_e3 + rtDW.DiscreteTimeIntegratory_DSTAT_j[1])) * rtb_omega2_pb;

    // SignalConversion: '<S9>/BusConversion_InsertedFor_ap_bus_at_inport_0' incorporates:
    //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y_dt'
    //   Gain: '<S51>/Gain'
    //   Inport: '<Root>/measure'
    //   Product: '<S51>/Product1'
    //   Product: '<S51>/Product2'
    //   Product: '<S51>/omega^2'
    //   Sum: '<S51>/Sum2'
    //   Sum: '<S51>/Sum3'

    rtDW.Omega_Kb_dt_f[2] = rtb_Sum2_cx[2];

    // SignalConversion: '<S9>/BusConversion_InsertedFor_ap_bus_at_inport_0' incorporates:
    //   DiscreteIntegrator: '<S48>/Discrete-Time Integrator y'
    //   MATLAB Function: '<S26>/Rotations matrix to Euler angles'

    rtDW.Euler_angles_f[2] = std::atan2(rtDW.DiscreteTimeIntegratory_DSTAT_b[3],
      rtDW.DiscreteTimeIntegratory_DSTAT_b[0]);

    // SignalConversion: '<S9>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.Omega_Kb_f[2] = rtb_y_fg[2];

    // SignalConversion: '<S9>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.e_s_g[2] = rtDW.e_s_g_e[2];

    // SignalConversion: '<S9>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.e_s_g_dt[2] = rtDW.e_s_g_dt_i[2];

    // SignalConversion: '<S9>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.e_s_g_dt2[2] = rtDW.e_s_g_dt2_k[2];

    // SignalConversion: '<S9>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.s_g_ref[2] = rtDW.s_g_ref_d[2];

    // SignalConversion: '<S9>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.s_g[2] = rtDW.s_g_a[2];

    // SignalConversion: '<S9>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.s_g_match[2] = rtDW.s_g_match_i[2];

    // Product: '<S51>/Product1' incorporates:
    //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y_dt'
    //   Gain: '<S51>/Gain'
    //   Inport: '<Root>/measure'
    //   Product: '<S51>/Product2'
    //   Product: '<S51>/omega^2'
    //   Sum: '<S51>/Sum2'
    //   Sum: '<S51>/Sum3'

    rtb_y_p3 = (rtU.measure.V_Kg[2] - (rtDW.DiscreteTimeIntegratory_dt_DS_e[2] *
      rtb_y_e3 + rtDW.DiscreteTimeIntegratory_DSTAT_j[2])) * rtb_omega2_pb;

    // Gain: '<S50>/Gain' incorporates:
    //   Constant: '<S50>/d'
    //   Constant: '<S50>/omega'
    //   Product: '<S50>/Divide'

    rtb_y_e3 = rtP.lindi.sflt.d / rtP.lindi.sflt.omega * 2.0F;

    // Sum: '<S50>/Sum2' incorporates:
    //   DiscreteIntegrator: '<S50>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S50>/Discrete-Time Integrator y_dt'
    //   Gain: '<S50>/Gain'
    //   Inport: '<Root>/measure'
    //   Product: '<S50>/Product2'
    //   Sum: '<S50>/Sum3'

    rtb_Sum2_cx[0] = rtU.measure.s_Kg[0] -
      (rtDW.DiscreteTimeIntegratory_dt_DS_p[0] * rtb_y_e3 +
       rtDW.DiscreteTimeIntegratory_DSTA_n2[0]);
    rtb_Sum2_cx[1] = rtU.measure.s_Kg[1] -
      (rtDW.DiscreteTimeIntegratory_dt_DS_p[1] * rtb_y_e3 +
       rtDW.DiscreteTimeIntegratory_DSTA_n2[1]);
    rtb_Sum2_cx[2] = rtU.measure.s_Kg[2] -
      (rtDW.DiscreteTimeIntegratory_dt_DS_p[2] * rtb_y_e3 +
       rtDW.DiscreteTimeIntegratory_DSTA_n2[2]);

    // Product: '<S50>/omega^2' incorporates:
    //   Constant: '<S50>/omega'

    rtb_omega2_d_0 = rtP.lindi.sflt.omega * rtP.lindi.sflt.omega;

    // Gain: '<S49>/Gain' incorporates:
    //   Constant: '<S49>/d'
    //   Constant: '<S49>/omega'
    //   Product: '<S49>/Divide'

    rtb_y_e3 = rtP.lindi.sflt.d / rtP.lindi.sflt.omega * 2.0F;

    // Sum: '<S49>/Sum2' incorporates:
    //   DiscreteIntegrator: '<S49>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S49>/Discrete-Time Integrator y_dt'
    //   Gain: '<S49>/Gain'
    //   Inport: '<Root>/measure'
    //   Product: '<S49>/Product2'
    //   Sum: '<Root>/Add'
    //   Sum: '<S49>/Sum3'

    rtb_Sum2_g[0] = rtU.measure.a_Kg[0] - (rtDW.DiscreteTimeIntegratory_dt_DS_i
      [0] * rtb_y_e3 + rtDW.DiscreteTimeIntegratory_DSTAT_n[0]);
    rtb_Sum2_g[1] = rtU.measure.a_Kg[1] - (rtDW.DiscreteTimeIntegratory_dt_DS_i
      [1] * rtb_y_e3 + rtDW.DiscreteTimeIntegratory_DSTAT_n[1]);
    rtb_Sum2_g[2] = (rtU.measure.a_Kg[2] + 9.81F) -
      (rtDW.DiscreteTimeIntegratory_dt_DS_i[2] * rtb_y_e3 +
       rtDW.DiscreteTimeIntegratory_DSTAT_n[2]);

    // Product: '<S49>/omega^2' incorporates:
    //   Constant: '<S49>/omega'

    rtb_omega2_e_0 = rtP.lindi.sflt.omega * rtP.lindi.sflt.omega;

    // MATLAB Function: '<S100>/MATLAB Function2' incorporates:
    //   Constant: '<S100>/Constant2'
    //   Sum: '<S100>/Add1'

    // :  Delta_u_aileron_collective = mean(u_d(1:end-3)-u(1:end-3));
    for (i_0 = 0; i_0 < 7; i_0++) {
      x[i_0] = (rtDW.Merge_m[i_0] + rtP.lindi.ca.u_d[i_0]) - umax[i_0];
    }

    rtb_omega2_pb = x[0];
    for (itmp = 0; itmp < 6; itmp++) {
      rtb_omega2_pb += x[itmp + 1];
    }

    // MATLAB Function: '<S102>/Flap downwash compensation'
    // :  y = u(end-1);
    // :  Delta_a_z_dw = -eig.cla_h/cef.clu(end-1) * G(4,end-1) * eig.dahdu(:)' * u(1:end-2); 
    rtb_y_e3 = -rtP.lindi.eig.cla_h / rtP.lindi.cef.clu[8] * rtb_G4xX[35];
    rtb_Sum2_os = 0.0F;
    for (stage_app_2 = 0; stage_app_2 < 8; stage_app_2++) {
      rtb_Sum2_os += rtb_y_e3 * rtP.lindi.eig.dahdu[stage_app_2] *
        rtb_Delta_u[stage_app_2];
    }

    // Gain: '<S102>/Gain12'
    // :  Delta_q_dt_dw =  ceb.m/ceb.iyy*eig.x_h*Delta_a_z_dw;
    rtb_y_e3 = 2.0F / rtP.lindi.eig.x_h * 0.71F * q0_q0;

    // Sum: '<S115>/Sum2' incorporates:
    //   Constant: '<S102>/Constant'
    //   DiscreteIntegrator: '<S115>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S115>/Discrete-Time Integrator y_dt'
    //   Gain: '<S115>/Gain'
    //   MATLAB Function: '<S102>/Flap downwash compensation'
    //   Product: '<S115>/Divide'
    //   Product: '<S115>/Product2'
    //   Sum: '<S115>/Sum3'

    rtb_Sum2_ms = rtb_Sum2_os - (0.71F / rtb_y_e3 * 2.0F *
      rtDW.DiscreteTimeIntegratory_dt_D_pj +
      rtDW.DiscreteTimeIntegratory_DSTAT_i);

    // Sum: '<S116>/Sum2' incorporates:
    //   Constant: '<S102>/Constant'
    //   DiscreteIntegrator: '<S116>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S116>/Discrete-Time Integrator y_dt'
    //   Gain: '<S116>/Gain'
    //   MATLAB Function: '<S102>/Flap downwash compensation'
    //   Product: '<S116>/Divide'
    //   Product: '<S116>/Product2'
    //   Sum: '<S116>/Sum3'

    rtb_Sum2_p1 = rtP.lindi.ceb.m / rtP.lindi.ceb.iyy * rtP.lindi.eig.x_h *
      rtb_Sum2_os - (0.71F / rtb_y_e3 * 2.0F *
                     rtDW.DiscreteTimeIntegratory_dt_D_ol +
                     rtDW.DiscreteTimeIntegratory_DSTA_na);

    // DiscreteIntegrator: '<S156>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S154>/Discrete-Time Integrator'

    if (rtDW.DiscreteTimeIntegratory_IC_L_km != 0) {
      rtDW.DiscreteTimeIntegratory_DSTA_jg =
        rtDW.DiscreteTimeIntegrator_DSTATE_o;
    }

    // Sum: '<S157>/Sum2' incorporates:
    //   Constant: '<S157>/d'
    //   Constant: '<S157>/omega'
    //   DiscreteIntegrator: '<S156>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S157>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S157>/Discrete-Time Integrator y_dt'
    //   Gain: '<S157>/Gain'
    //   Product: '<S157>/Divide'
    //   Product: '<S157>/Product2'
    //   Sum: '<S157>/Sum3'

    rtb_Sum2_h0 = rtDW.DiscreteTimeIntegratory_DSTA_jg - (rtP.lindi.sflt.d /
      rtP.lindi.sflt.omega * 2.0F * rtDW.DiscreteTimeIntegratory_dt_DS_n +
      rtDW.DiscreteTimeIntegratory_DSTAT_g);

    // Delay: '<S158>/Delay' incorporates:
    //   DiscreteIntegrator: '<S154>/Discrete-Time Integrator'

    if (rtDW.icLoad_d != 0) {
      rtDW.Delay_DSTATE_a = rtDW.DiscreteTimeIntegrator_DSTATE_o;
    }

    rtb_Delay_m = rtDW.Delay_DSTATE_a;

    // Delay: '<S158>/Delay1' incorporates:
    //   DiscreteIntegrator: '<S154>/Discrete-Time Integrator'

    if (rtDW.icLoad_b != 0) {
      rtDW.Delay1_DSTATE_g = rtDW.DiscreteTimeIntegrator_DSTATE_o;
    }

    rtb_Delay1_k = rtDW.Delay1_DSTATE_g;

    // Delay: '<S158>/Delay2' incorporates:
    //   DiscreteIntegrator: '<S154>/Discrete-Time Integrator'

    if (rtDW.icLoad_l != 0) {
      rtDW.Delay2_DSTATE_n = rtDW.DiscreteTimeIntegrator_DSTATE_o;
    }

    rtb_Delay2_g = rtDW.Delay2_DSTATE_n;

    // Delay: '<S158>/Delay3' incorporates:
    //   DiscreteIntegrator: '<S154>/Discrete-Time Integrator'

    if (rtDW.icLoad_b1 != 0) {
      rtDW.Delay3_DSTATE_a = rtDW.DiscreteTimeIntegrator_DSTATE_o;
    }

    // MATLAB Function: '<S158>/MATLAB Function' incorporates:
    //   Constant: '<S158>/Constant'
    //   Constant: '<S158>/Constant1'
    //   Delay: '<S158>/Delay'
    //   Delay: '<S158>/Delay1'
    //   Delay: '<S158>/Delay2'
    //   Delay: '<S158>/Delay3'
    //   DiscreteIntegrator: '<S154>/Discrete-Time Integrator'

    MATLABFunction(rtDW.DiscreteTimeIntegrator_DSTATE_o, rtDW.Delay_DSTATE_a,
                   rtDW.Delay1_DSTATE_g, rtDW.Delay2_DSTATE_n,
                   rtDW.Delay3_DSTATE_a, rtP.lindi.servo.delay, 0.0025F,
                   &rtb_y_dm);

    // Sum: '<S156>/Sum2' incorporates:
    //   Constant: '<S156>/d'
    //   Constant: '<S156>/omega'
    //   DiscreteIntegrator: '<S156>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S156>/Discrete-Time Integrator y_dt'
    //   Gain: '<S156>/Gain'
    //   Product: '<S156>/Divide'
    //   Product: '<S156>/Product2'
    //   Sum: '<S156>/Sum3'

    rtb_y_dm -= rtP.lindi.servo.d / (rtP.lindi.servo.omega *
      rtP.lindi.servo.boost) * 2.0F * rtDW.DiscreteTimeIntegratory_dt_D_jq +
      rtDW.DiscreteTimeIntegratory_DSTA_jg;

    // Gain: '<S87>/Gain' incorporates:
    //   Constant: '<S84>/Constant1'
    //   Gain: '<S84>/Gain12'
    //   Product: '<S84>/Divide'

    rtb_Sum2_os = 1.0F / q0_q0 * rtP.lindi.eig.x_h * 0.5F;

    // Product: '<S91>/Divide' incorporates:
    //   DiscreteIntegrator: '<S90>/Discrete-Time Integrator'
    //   DiscreteIntegrator: '<S91>/Discrete-Time Integrator'
    //   Sum: '<S91>/Sum2'

    rtb_Divide_k = (rtDW.DiscreteTimeIntegrator_DSTATE_a -
                    rtDW.DiscreteTimeIntegrator_DSTAT_en) / rtb_Sum2_os;

    // Product: '<S90>/Divide' incorporates:
    //   DiscreteIntegrator: '<S90>/Discrete-Time Integrator'
    //   Gain: '<S84>/Gain2'
    //   Sum: '<S90>/Sum2'

    rtb_Divide_l = ((rtP.lindi.eig.xnp - rtP.lindi.eig.xnp0) *
                    q_bg_unsigned_idx_1 - rtDW.DiscreteTimeIntegrator_DSTATE_a) /
      rtb_Sum2_os;

    // MATLAB Function: '<S86>/Angle of attack time delay' incorporates:
    //   Constant: '<S86>/Constant6'
    //   Constant: '<S86>/Constant7'
    //   Constant: '<S86>/Constant8'

    // :  A_alpha = divideFinite(V,m)*rho/2*S*C_La;
    rtb_Sum2_os = rtP.lindi.ceb.m;

    // 'divideFinite:29' if numel(B)>1
    // 'divideFinite:31' else
    // 'divideFinite:32' if abs(B)<eps
    if (std::abs(rtP.lindi.ceb.m) < 2.22044605E-16F) {
      // 'divideFinite:33' B(:) = eps;
      rtb_Sum2_os = 2.22044605E-16F;
    }

    // 'divideFinite:36' C = A ./ B;
    rtb_Sum2_os = q0_q0 / rtb_Sum2_os * 1.225F / 2.0F * rtP.lindi.eig.s *
      rtP.lindi.eig.cla_h;

    // :  T = divideFinite( 1, A_alpha );
    // 'divideFinite:29' if numel(B)>1
    // 'divideFinite:31' else
    // 'divideFinite:32' if abs(B)<eps
    if (std::abs(rtb_Sum2_os) < 2.22044605E-16F) {
      // 'divideFinite:33' B(:) = eps;
      rtb_Sum2_os = 2.22044605E-16F;
    }

    // Switch: '<S86>/Switch' incorporates:
    //   Constant: '<S86>/Constant'
    //   Gain: '<S86>/Gain'
    //   MATLAB Function: '<S86>/Angle of attack time delay'

    // 'divideFinite:36' C = A ./ B;
    if (rtP.lindi.dlc.opt > 1.5F) {
      rtb_Sum2_os = 1.0F / rtb_Sum2_os * 0.5F;
    } else {
      rtb_Sum2_os = 1.0F / rtb_Sum2_os;
    }

    // End of Switch: '<S86>/Switch'

    // Product: '<S89>/Divide' incorporates:
    //   DiscreteIntegrator: '<S89>/Discrete-Time Integrator'
    //   Sum: '<S89>/Sum2'

    rtb_Divide_p = (q_bg_unsigned_idx_0 - rtDW.DiscreteTimeIntegrator_DSTATE_h) /
      rtb_Sum2_os;

    // DiscreteIntegrator: '<S92>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S80>/Discrete-Time Integrator'

    if (rtDW.DiscreteTimeIntegratory_IC_LO_g != 0) {
      rtDW.DiscreteTimeIntegratory_DSTA_hj =
        rtDW.DiscreteTimeIntegrator_DSTATE_e;
    }

    // Sum: '<S93>/Sum2' incorporates:
    //   Constant: '<S93>/d'
    //   Constant: '<S93>/omega'
    //   DiscreteIntegrator: '<S92>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S93>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S93>/Discrete-Time Integrator y_dt'
    //   Gain: '<S93>/Gain'
    //   Product: '<S93>/Divide'
    //   Product: '<S93>/Product2'
    //   Sum: '<S93>/Sum3'

    rtb_Sum2_d0 = rtDW.DiscreteTimeIntegratory_DSTA_hj - (rtP.lindi.sflt.d /
      rtP.lindi.sflt.omega * 2.0F * rtDW.DiscreteTimeIntegratory_dt_D_n5 +
      rtDW.DiscreteTimeIntegratory_DSTAT_f);

    // Delay: '<S94>/Delay' incorporates:
    //   DiscreteIntegrator: '<S80>/Discrete-Time Integrator'

    if (rtDW.icLoad_dk != 0) {
      rtDW.Delay_DSTATE_k = rtDW.DiscreteTimeIntegrator_DSTATE_e;
    }

    rtb_Delay_n = rtDW.Delay_DSTATE_k;

    // Delay: '<S94>/Delay1' incorporates:
    //   DiscreteIntegrator: '<S80>/Discrete-Time Integrator'

    if (rtDW.icLoad_j != 0) {
      rtDW.Delay1_DSTATE_h = rtDW.DiscreteTimeIntegrator_DSTATE_e;
    }

    rtb_Delay1_f = rtDW.Delay1_DSTATE_h;

    // Delay: '<S94>/Delay2' incorporates:
    //   DiscreteIntegrator: '<S80>/Discrete-Time Integrator'

    if (rtDW.icLoad_ix != 0) {
      rtDW.Delay2_DSTATE_h = rtDW.DiscreteTimeIntegrator_DSTATE_e;
    }

    rtb_Delay2_b = rtDW.Delay2_DSTATE_h;

    // Delay: '<S94>/Delay3' incorporates:
    //   DiscreteIntegrator: '<S80>/Discrete-Time Integrator'

    if (rtDW.icLoad_ow != 0) {
      rtDW.Delay3_DSTATE_m = rtDW.DiscreteTimeIntegrator_DSTATE_e;
    }

    // MATLAB Function: '<S94>/MATLAB Function' incorporates:
    //   Constant: '<S94>/Constant'
    //   Constant: '<S94>/Constant1'
    //   Delay: '<S94>/Delay'
    //   Delay: '<S94>/Delay1'
    //   Delay: '<S94>/Delay2'
    //   Delay: '<S94>/Delay3'
    //   DiscreteIntegrator: '<S80>/Discrete-Time Integrator'

    MATLABFunction(rtDW.DiscreteTimeIntegrator_DSTATE_e, rtDW.Delay_DSTATE_k,
                   rtDW.Delay1_DSTATE_h, rtDW.Delay2_DSTATE_h,
                   rtDW.Delay3_DSTATE_m, rtP.lindi.servo.delay, 0.0025F,
                   &q_bg_unsigned_idx_1);

    // Sum: '<S92>/Sum2' incorporates:
    //   Constant: '<S92>/d'
    //   Constant: '<S92>/omega'
    //   DiscreteIntegrator: '<S92>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S92>/Discrete-Time Integrator y_dt'
    //   Gain: '<S92>/Gain'
    //   Product: '<S92>/Divide'
    //   Product: '<S92>/Product2'
    //   Sum: '<S92>/Sum3'

    q_bg_unsigned_idx_1 -= rtP.lindi.servo.d / (rtP.lindi.servo.omega *
      rtP.lindi.servo.boost) * 2.0F * rtDW.DiscreteTimeIntegratory_dt_D_j0 +
      rtDW.DiscreteTimeIntegratory_DSTA_hj;

    // DiscreteIntegrator: '<S96>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S80>/Discrete-Time Integrator'

    if (rtDW.DiscreteTimeIntegratory_IC_L_el != 0) {
      rtDW.DiscreteTimeIntegratory_DSTA_ny =
        rtDW.DiscreteTimeIntegrator_DSTATE_e;
    }

    // Sum: '<S97>/Sum2' incorporates:
    //   Constant: '<S97>/d'
    //   Constant: '<S97>/omega'
    //   DiscreteIntegrator: '<S96>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S97>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S97>/Discrete-Time Integrator y_dt'
    //   Gain: '<S97>/Gain'
    //   Product: '<S97>/Divide'
    //   Product: '<S97>/Product2'
    //   Sum: '<S97>/Sum3'

    rtb_Sum2_dqn = rtDW.DiscreteTimeIntegratory_DSTA_ny - (rtP.lindi.sflt.d /
      rtP.lindi.sflt.omega * 2.0F * rtDW.DiscreteTimeIntegratory_dt_DS_j +
      rtDW.DiscreteTimeIntegratory_DSTAT_k);

    // Delay: '<S98>/Delay' incorporates:
    //   DiscreteIntegrator: '<S80>/Discrete-Time Integrator'

    if (rtDW.icLoad_dh != 0) {
      rtDW.Delay_DSTATE_n = rtDW.DiscreteTimeIntegrator_DSTATE_e;
    }

    rtb_Delay_o = rtDW.Delay_DSTATE_n;

    // Delay: '<S98>/Delay1' incorporates:
    //   DiscreteIntegrator: '<S80>/Discrete-Time Integrator'

    if (rtDW.icLoad_f != 0) {
      rtDW.Delay1_DSTATE_k = rtDW.DiscreteTimeIntegrator_DSTATE_e;
    }

    rtb_Delay1_i = rtDW.Delay1_DSTATE_k;

    // Delay: '<S98>/Delay2' incorporates:
    //   DiscreteIntegrator: '<S80>/Discrete-Time Integrator'

    if (rtDW.icLoad_fc != 0) {
      rtDW.Delay2_DSTATE_p = rtDW.DiscreteTimeIntegrator_DSTATE_e;
    }

    rtb_Delay2_o = rtDW.Delay2_DSTATE_p;

    // Delay: '<S98>/Delay3' incorporates:
    //   DiscreteIntegrator: '<S80>/Discrete-Time Integrator'

    if (rtDW.icLoad_jp != 0) {
      rtDW.Delay3_DSTATE_h = rtDW.DiscreteTimeIntegrator_DSTATE_e;
    }

    // MATLAB Function: '<S98>/MATLAB Function' incorporates:
    //   Constant: '<S98>/Constant'
    //   Constant: '<S98>/Constant1'
    //   Delay: '<S98>/Delay'
    //   Delay: '<S98>/Delay1'
    //   Delay: '<S98>/Delay2'
    //   Delay: '<S98>/Delay3'
    //   DiscreteIntegrator: '<S80>/Discrete-Time Integrator'

    MATLABFunction(rtDW.DiscreteTimeIntegrator_DSTATE_e, rtDW.Delay_DSTATE_n,
                   rtDW.Delay1_DSTATE_k, rtDW.Delay2_DSTATE_p,
                   rtDW.Delay3_DSTATE_h, rtP.lindi.servo.delay, 0.0025F,
                   &q_bg_unsigned_idx_0);

    // Sum: '<S96>/Sum2' incorporates:
    //   Constant: '<S96>/d'
    //   Constant: '<S96>/omega'
    //   DiscreteIntegrator: '<S96>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S96>/Discrete-Time Integrator y_dt'
    //   Gain: '<S96>/Gain'
    //   Product: '<S96>/Divide'
    //   Product: '<S96>/Product2'
    //   Sum: '<S96>/Sum3'

    q_bg_unsigned_idx_0 -= rtP.lindi.servo.d / (rtP.lindi.servo.omega *
      rtP.lindi.servo.boost) * 2.0F * rtDW.DiscreteTimeIntegratory_dt_DS_a +
      rtDW.DiscreteTimeIntegratory_DSTA_ny;

    // DiscreteIntegrator: '<S129>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S122>/Discrete-Time Integrator y_dt'

    if (rtDW.DiscreteTimeIntegratory_IC_LO_b != 0) {
      rtDW.DiscreteTimeIntegratory_DSTAT_a =
        rtDW.DiscreteTimeIntegratory_dt_DS_g;
    }

    // Sum: '<S130>/Sum2' incorporates:
    //   Constant: '<S130>/d'
    //   Constant: '<S130>/omega'
    //   DiscreteIntegrator: '<S129>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S130>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S130>/Discrete-Time Integrator y_dt'
    //   Gain: '<S130>/Gain'
    //   Product: '<S130>/Divide'
    //   Product: '<S130>/Product2'
    //   Sum: '<S130>/Sum3'

    rtb_Sum2_eh = rtDW.DiscreteTimeIntegratory_DSTAT_a - (rtP.lindi.sflt.d /
      rtP.lindi.sflt.omega * 2.0F * rtDW.DiscreteTimeIntegratory_dt_DS_m +
      rtDW.DiscreteTimeIntegratory_DSTA_jv);

    // Delay: '<S131>/Delay' incorporates:
    //   DiscreteIntegrator: '<S122>/Discrete-Time Integrator y_dt'

    if (rtDW.icLoad_dhb != 0) {
      rtDW.Delay_DSTATE_j = rtDW.DiscreteTimeIntegratory_dt_DS_g;
    }

    rtb_Delay_j = rtDW.Delay_DSTATE_j;

    // Delay: '<S131>/Delay1' incorporates:
    //   DiscreteIntegrator: '<S122>/Discrete-Time Integrator y_dt'

    if (rtDW.icLoad_fx != 0) {
      rtDW.Delay1_DSTATE_ha = rtDW.DiscreteTimeIntegratory_dt_DS_g;
    }

    rtb_Delay1_a = rtDW.Delay1_DSTATE_ha;

    // Delay: '<S131>/Delay2' incorporates:
    //   DiscreteIntegrator: '<S122>/Discrete-Time Integrator y_dt'

    if (rtDW.icLoad_m != 0) {
      rtDW.Delay2_DSTATE_m = rtDW.DiscreteTimeIntegratory_dt_DS_g;
    }

    rtb_Delay2_oz = rtDW.Delay2_DSTATE_m;

    // Delay: '<S131>/Delay3' incorporates:
    //   DiscreteIntegrator: '<S122>/Discrete-Time Integrator y_dt'

    if (rtDW.icLoad_dc != 0) {
      rtDW.Delay3_DSTATE_p = rtDW.DiscreteTimeIntegratory_dt_DS_g;
    }

    // MATLAB Function: '<S131>/MATLAB Function' incorporates:
    //   Constant: '<S131>/Constant'
    //   Constant: '<S131>/Constant1'
    //   Delay: '<S131>/Delay'
    //   Delay: '<S131>/Delay1'
    //   Delay: '<S131>/Delay2'
    //   Delay: '<S131>/Delay3'
    //   DiscreteIntegrator: '<S122>/Discrete-Time Integrator y_dt'

    MATLABFunction(rtDW.DiscreteTimeIntegratory_dt_DS_g, rtDW.Delay_DSTATE_j,
                   rtDW.Delay1_DSTATE_ha, rtDW.Delay2_DSTATE_m,
                   rtDW.Delay3_DSTATE_p, rtP.lindi.servo.delay, 0.0025F,
                   &rtb_Sum2_os);

    // Sum: '<S129>/Sum2' incorporates:
    //   Constant: '<S129>/d'
    //   Constant: '<S129>/omega'
    //   DiscreteIntegrator: '<S129>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S129>/Discrete-Time Integrator y_dt'
    //   Gain: '<S129>/Gain'
    //   Product: '<S129>/Divide'
    //   Product: '<S129>/Product2'
    //   Sum: '<S129>/Sum3'

    rtb_Sum2_mb = rtb_Sum2_os - (rtP.lindi.servo.d / (rtP.lindi.servo.omega *
      rtP.lindi.servo.boost) * 2.0F * rtDW.DiscreteTimeIntegratory_dt_DS_b +
      rtDW.DiscreteTimeIntegratory_DSTAT_a);

    // Delay: '<S135>/Delay' incorporates:
    //   DiscreteIntegrator: '<S122>/Discrete-Time Integrator y'

    if (rtDW.icLoad_f1 != 0) {
      rtDW.Delay_DSTATE_jn = rtDW.DiscreteTimeIntegratory_DSTAT_p;
    }

    rtb_Delay_i = rtDW.Delay_DSTATE_jn;

    // Delay: '<S135>/Delay1' incorporates:
    //   DiscreteIntegrator: '<S122>/Discrete-Time Integrator y'

    if (rtDW.icLoad_fu != 0) {
      rtDW.Delay1_DSTATE_d = rtDW.DiscreteTimeIntegratory_DSTAT_p;
    }

    rtb_Delay1_eh = rtDW.Delay1_DSTATE_d;

    // Delay: '<S135>/Delay2' incorporates:
    //   DiscreteIntegrator: '<S122>/Discrete-Time Integrator y'

    if (rtDW.icLoad_ib != 0) {
      rtDW.Delay2_DSTATE_j = rtDW.DiscreteTimeIntegratory_DSTAT_p;
    }

    rtb_Delay2_k = rtDW.Delay2_DSTATE_j;

    // Delay: '<S135>/Delay3' incorporates:
    //   DiscreteIntegrator: '<S122>/Discrete-Time Integrator y'

    if (rtDW.icLoad_kd != 0) {
      rtDW.Delay3_DSTATE_g = rtDW.DiscreteTimeIntegratory_DSTAT_p;
    }

    // MATLAB Function: '<S135>/MATLAB Function' incorporates:
    //   Constant: '<S135>/Constant'
    //   Constant: '<S135>/Constant1'
    //   Delay: '<S135>/Delay'
    //   Delay: '<S135>/Delay1'
    //   Delay: '<S135>/Delay2'
    //   Delay: '<S135>/Delay3'
    //   DiscreteIntegrator: '<S122>/Discrete-Time Integrator y'

    MATLABFunction(rtDW.DiscreteTimeIntegratory_DSTAT_p, rtDW.Delay_DSTATE_jn,
                   rtDW.Delay1_DSTATE_d, rtDW.Delay2_DSTATE_j,
                   rtDW.Delay3_DSTATE_g, rtP.lindi.servo.delay, 0.0025F,
                   &rtb_Sum2_os);

    // Sum: '<S133>/Sum2' incorporates:
    //   Constant: '<S133>/d'
    //   Constant: '<S133>/omega'
    //   DiscreteIntegrator: '<S133>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S133>/Discrete-Time Integrator y_dt'
    //   Gain: '<S133>/Gain'
    //   Product: '<S133>/Divide'
    //   Product: '<S133>/Product2'
    //   Sum: '<S133>/Sum3'

    rtb_Sum2_gn = rtb_Sum2_os - (rtP.lindi.servo.d / (rtP.lindi.servo.omega *
      rtP.lindi.servo.boost) * 2.0F * rtDW.DiscreteTimeIntegratory_dt_DS_k +
      rtDW.DiscreteTimeIntegratory_DSTAT_h);

    // SignalConversion: '<S9>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.wp_idx = rtDW.wp_idx_n;

    // SignalConversion: '<S9>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.stage = rtDW.stage_e;

    // SignalConversion: '<S9>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.t = rtDW.t_l;

    // SignalConversion: '<S9>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.V_A_f = q0_q0;

    // Product: '<S52>/Divide' incorporates:
    //   Constant: '<S52>/omega'
    //   Product: '<S52>/omega^2'

    q0_q0 = 2.0F / rtP.lindi.aspd.flttc;

    // Sum: '<S52>/Sum2' incorporates:
    //   Constant: '<S52>/d'
    //   Constant: '<S52>/omega'
    //   DiscreteIntegrator: '<S52>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S52>/Discrete-Time Integrator y_dt'
    //   Gain: '<S52>/Gain'
    //   Inport: '<Root>/measure'
    //   Product: '<S52>/Divide'
    //   Product: '<S52>/Product2'
    //   Sum: '<S52>/Sum3'

    rtb_Sum2_os = rtU.measure.airspeed - (1.0F / q0_q0 * 2.0F *
      rtDW.DiscreteTimeIntegratory_dt_D_gf +
      rtDW.DiscreteTimeIntegratory_DSTAT_d);

    // Update for DiscreteIntegrator: '<S52>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S52>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_IC_LOAD = 0U;
    rtDW.DiscreteTimeIntegratory_DSTAT_d += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_D_gf;

    // Update for DiscreteIntegrator: '<S48>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S48>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_IC_LO_a = 0U;
    for (stage_app_2 = 0; stage_app_2 < 9; stage_app_2++) {
      rtDW.DiscreteTimeIntegratory_DSTAT_b[stage_app_2] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_D_ni[stage_app_2];
    }

    // End of Update for DiscreteIntegrator: '<S48>/Discrete-Time Integrator y'

    // Update for DiscreteIntegrator: '<S50>/Discrete-Time Integrator y'
    rtDW.DiscreteTimeIntegratory_IC_LO_e = 0U;

    // Update for DiscreteIntegrator: '<S51>/Discrete-Time Integrator y'
    rtDW.DiscreteTimeIntegratory_IC_LO_i = 0U;

    // Update for DiscreteIntegrator: '<S49>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S49>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTAT_n[0] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_i[0];

    // Update for DiscreteIntegrator: '<S50>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S50>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTA_n2[0] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_p[0];

    // Update for DiscreteIntegrator: '<S51>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTAT_j[0] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_e[0];

    // Update for DiscreteIntegrator: '<S49>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S49>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTAT_n[1] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_i[1];

    // Update for DiscreteIntegrator: '<S50>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S50>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTA_n2[1] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_p[1];

    // Update for DiscreteIntegrator: '<S51>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTAT_j[1] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_e[1];

    // Update for DiscreteIntegrator: '<S49>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S49>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTAT_n[2] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_i[2];

    // Update for DiscreteIntegrator: '<S50>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S50>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTA_n2[2] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_p[2];

    // Update for DiscreteIntegrator: '<S51>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTAT_j[2] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_e[2];

    // Update for UnitDelay: '<S33>/Unit Delay'
    rtDW.UnitDelay_DSTATE = q1_q1;

    // Update for DiscreteIntegrator: '<S80>/Discrete-Time Integrator'
    rtDW.DiscreteTimeIntegrator_IC_LOADI = 0U;
    rtDW.DiscreteTimeIntegrator_DSTATE_e += 0.0025F * q_bg_unsigned_idx_3;

    // Update for DiscreteIntegrator: '<S122>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S122>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTAT_p += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_g;

    // Update for DiscreteIntegrator: '<S122>/Discrete-Time Integrator y_dt'
    rtDW.DiscreteTimeIntegratory_dt_DS_g += 0.0025F * q0_q3;

    // Update for DiscreteIntegrator: '<S134>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S134>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_IC_LO_m = 0U;
    rtDW.DiscreteTimeIntegratory_DSTAT_c += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_f;

    // Update for DiscreteIntegrator: '<S134>/Discrete-Time Integrator y_dt'
    rtDW.DiscreteTimeIntegratory_dt_DS_f += 0.0025F * q1_q2;

    // Update for DiscreteIntegrator: '<S133>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S133>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_IC_LO_d = 0U;
    rtDW.DiscreteTimeIntegratory_DSTAT_h += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_k;

    // Update for DiscreteIntegrator: '<S130>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S130>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_IC_LO_o = 0U;
    rtDW.DiscreteTimeIntegratory_DSTA_jv += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_m;

    // Update for DiscreteIntegrator: '<S97>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S97>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_IC_L_eu = 0U;
    rtDW.DiscreteTimeIntegratory_DSTAT_k += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_j;

    // Update for DiscreteIntegrator: '<S97>/Discrete-Time Integrator y_dt' incorporates:
    //   Constant: '<S97>/omega'
    //   Product: '<S97>/Product1'
    //   Product: '<S97>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_DS_j += rtP.lindi.sflt.omega *
      rtP.lindi.sflt.omega * rtb_Sum2_dqn * 0.0025F;

    // Update for DiscreteIntegrator: '<S93>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S93>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_IC_LO_k = 0U;
    rtDW.DiscreteTimeIntegratory_DSTAT_f += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_D_n5;

    // Update for DiscreteIntegrator: '<S89>/Discrete-Time Integrator'
    rtDW.DiscreteTimeIntegrator_DSTATE_h += 0.0025F * rtb_Divide_p;

    // Update for DiscreteIntegrator: '<S91>/Discrete-Time Integrator'
    rtDW.DiscreteTimeIntegrator_DSTAT_en += 0.0025F * rtb_Divide_k;

    // Update for UnitDelay: '<S34>/Unit Delay' incorporates:
    //   Gain: '<S34>/Gain5'
    //   MATLAB Function: '<S34>/Rudder command'
    //   SignalConversion: '<S137>/TmpSignal ConversionAt SFunction Inport1'

    rtDW.UnitDelay_DSTATE_f = rtP.lindi.atc.rm.ydecaytc * umax[9];

    // Update for DiscreteIntegrator: '<S154>/Discrete-Time Integrator'
    rtDW.DiscreteTimeIntegrator_IC_LOA_g = 0U;
    rtDW.DiscreteTimeIntegrator_DSTATE_o += 0.0025F * rtb_Divide_h;

    // Update for DiscreteIntegrator: '<S157>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S157>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_IC_LO_f = 0U;
    rtDW.DiscreteTimeIntegratory_DSTAT_g += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_n;

    // Update for DiscreteIntegrator: '<S157>/Discrete-Time Integrator y_dt' incorporates:
    //   Constant: '<S157>/omega'
    //   Product: '<S157>/Product1'
    //   Product: '<S157>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_DS_n += rtP.lindi.sflt.omega *
      rtP.lindi.sflt.omega * rtb_Sum2_h0 * 0.0025F;

    // Update for UnitDelay: '<S100>/Unit Delay' incorporates:
    //   MATLAB Function: '<S100>/MATLAB Function2'

    rtDW.UnitDelay_DSTATE_i = rtb_omega2_pb / 7.0F;

    // Update for DiscreteIntegrator: '<S116>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S116>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTA_na += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_D_ol;

    // Update for DiscreteIntegrator: '<S115>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S115>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTAT_i += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_D_pj;

    // Update for Delay: '<S107>/Delay'
    rtDW.icLoad = 0U;

    // Update for Delay: '<S107>/Delay1'
    rtDW.icLoad_i = 0U;

    // Update for Delay: '<S107>/Delay2'
    rtDW.icLoad_o = 0U;

    // Update for Delay: '<S107>/Delay3'
    rtDW.icLoad_k = 0U;
    for (i_0 = 0; i_0 < 10; i_0++) {
      // Update for DiscreteIntegrator: '<S106>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S106>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_DSTAT_l[i_0] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_DS_o[i_0];

      // Update for DiscreteIntegrator: '<S105>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S105>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_DSTA_j2[i_0] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_D_nz[i_0];

      // Update for DiscreteIntegrator: '<S109>/Discrete-Time Integrator1' incorporates:
      //   DiscreteIntegrator: '<S109>/Discrete-Time Integrator2'
      //   Gain: '<S109>/Gain4'
      //   Gain: '<S109>/Gain5'
      //   Sum: '<S109>/Add2'

      rtDW.DiscreteTimeIntegrator1_DSTATE[i_0] += ((q2_q2 *
        rtDW.DiscreteTimeIntegrator2_DSTATE[i_0] + umin[i_0]) + q1_q3 *
        rtDW.DiscreteTimeIntegrator1_DSTATE[i_0]) * 0.0025F;

      // Update for DiscreteIntegrator: '<S109>/Discrete-Time Integrator2'
      rtDW.DiscreteTimeIntegrator2_DSTATE[i_0] += 0.0025F *
        rtb_DiscreteTimeIntegrator1[i_0];

      // Update for Delay: '<S107>/Delay'
      rtDW.Delay_DSTATE[i_0] = umax[i_0];

      // Update for Delay: '<S107>/Delay1'
      rtDW.Delay1_DSTATE[i_0] = rtb_Delay[i_0];

      // Update for Delay: '<S107>/Delay2'
      rtDW.Delay2_DSTATE[i_0] = rtb_Delay1[i_0];

      // Update for Delay: '<S107>/Delay3'
      rtDW.Delay3_DSTATE[i_0] = rtb_Delay2[i_0];

      // Update for DiscreteIntegrator: '<S106>/Discrete-Time Integrator y_dt' incorporates:
      //   Product: '<S106>/Product1'

      rtDW.DiscreteTimeIntegratory_dt_DS_o[i_0] += rtb_u_delay[i_0] * umax_1 *
        0.0025F;

      // Update for DiscreteIntegrator: '<S105>/Discrete-Time Integrator y_dt' incorporates:
      //   Product: '<S105>/Product1'
      //   Product: '<S105>/omega^2'

      rtDW.DiscreteTimeIntegratory_dt_D_nz[i_0] += rtb_Sum2_ig[i_0] *
        rtb_omega2_o_0 * 0.0025F;
    }

    // Update for DiscreteIntegrator: '<S48>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S48>/Product1'
    //   Product: '<S48>/omega^2'

    for (stage_app_2 = 0; stage_app_2 < 9; stage_app_2++) {
      rtDW.DiscreteTimeIntegratory_dt_D_ni[stage_app_2] += rtb_M_bg[stage_app_2]
        * rtb_omega2_oa_0 * 0.0025F;
    }

    // End of Update for DiscreteIntegrator: '<S48>/Discrete-Time Integrator y_dt' 

    // Update for DiscreteIntegrator: '<S51>/Discrete-Time Integrator y_dt'
    rtDW.DiscreteTimeIntegratory_dt_DS_e[0] += 0.0025F * rtb_y_fg[0];

    // Update for DiscreteIntegrator: '<S50>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S50>/Product1'
    //   Product: '<S50>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_DS_p[0] += rtb_Sum2_cx[0] * rtb_omega2_d_0 *
      0.0025F;

    // Update for DiscreteIntegrator: '<S49>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S49>/Product1'
    //   Product: '<S49>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_DS_i[0] += rtb_Sum2_g[0] * rtb_omega2_e_0 *
      0.0025F;

    // Update for DiscreteIntegrator: '<S51>/Discrete-Time Integrator y_dt'
    rtDW.DiscreteTimeIntegratory_dt_DS_e[1] += 0.0025F * rtb_y_fg[1];

    // Update for DiscreteIntegrator: '<S50>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S50>/Product1'
    //   Product: '<S50>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_DS_p[1] += rtb_Sum2_cx[1] * rtb_omega2_d_0 *
      0.0025F;

    // Update for DiscreteIntegrator: '<S49>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S49>/Product1'
    //   Product: '<S49>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_DS_i[1] += rtb_Sum2_g[1] * rtb_omega2_e_0 *
      0.0025F;

    // Update for DiscreteIntegrator: '<S51>/Discrete-Time Integrator y_dt'
    rtDW.DiscreteTimeIntegratory_dt_DS_e[2] += 0.0025F * rtb_y_p3;

    // Update for DiscreteIntegrator: '<S50>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S50>/Product1'
    //   Product: '<S50>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_DS_p[2] += rtb_Sum2_cx[2] * rtb_omega2_d_0 *
      0.0025F;

    // Update for DiscreteIntegrator: '<S49>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S49>/Product1'
    //   Product: '<S49>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_DS_i[2] += rtb_Sum2_g[2] * rtb_omega2_e_0 *
      0.0025F;

    // Product: '<S115>/omega^2' incorporates:
    //   Product: '<S116>/omega^2'

    q1_q1 = rtb_y_e3 * rtb_y_e3;

    // Update for DiscreteIntegrator: '<S115>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S115>/Product1'
    //   Product: '<S115>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_D_pj += q1_q1 * rtb_Sum2_ms * 0.0025F;

    // Update for DiscreteIntegrator: '<S116>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S116>/Product1'

    rtDW.DiscreteTimeIntegratory_dt_D_ol += q1_q1 * rtb_Sum2_p1 * 0.0025F;

    // Update for DiscreteIntegrator: '<S156>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S156>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_IC_L_km = 0U;
    rtDW.DiscreteTimeIntegratory_DSTA_jg += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_D_jq;

    // Update for DiscreteIntegrator: '<S156>/Discrete-Time Integrator y_dt' incorporates:
    //   Constant: '<S156>/omega'
    //   Product: '<S156>/Product1'
    //   Product: '<S156>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_D_jq += rtP.lindi.servo.omega *
      rtP.lindi.servo.boost * (rtP.lindi.servo.omega * rtP.lindi.servo.boost) *
      rtb_y_dm * 0.0025F;

    // Update for Delay: '<S158>/Delay'
    rtDW.icLoad_d = 0U;
    rtDW.Delay_DSTATE_a = q_bg_unsigned_idx_2;

    // Update for Delay: '<S158>/Delay1'
    rtDW.icLoad_b = 0U;
    rtDW.Delay1_DSTATE_g = rtb_Delay_m;

    // Update for Delay: '<S158>/Delay2'
    rtDW.icLoad_l = 0U;
    rtDW.Delay2_DSTATE_n = rtb_Delay1_k;

    // Update for Delay: '<S158>/Delay3'
    rtDW.icLoad_b1 = 0U;
    rtDW.Delay3_DSTATE_a = rtb_Delay2_g;

    // Update for DiscreteIntegrator: '<S90>/Discrete-Time Integrator'
    rtDW.DiscreteTimeIntegrator_DSTATE_a += 0.0025F * rtb_Divide_l;

    // Update for DiscreteIntegrator: '<S93>/Discrete-Time Integrator y_dt' incorporates:
    //   Constant: '<S93>/omega'
    //   Product: '<S93>/Product1'
    //   Product: '<S93>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_D_n5 += rtP.lindi.sflt.omega *
      rtP.lindi.sflt.omega * rtb_Sum2_d0 * 0.0025F;

    // Update for DiscreteIntegrator: '<S92>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S92>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_IC_LO_g = 0U;
    rtDW.DiscreteTimeIntegratory_DSTA_hj += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_D_j0;

    // Update for DiscreteIntegrator: '<S92>/Discrete-Time Integrator y_dt' incorporates:
    //   Constant: '<S92>/omega'
    //   Product: '<S92>/Product1'
    //   Product: '<S92>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_D_j0 += rtP.lindi.servo.omega *
      rtP.lindi.servo.boost * (rtP.lindi.servo.omega * rtP.lindi.servo.boost) *
      q_bg_unsigned_idx_1 * 0.0025F;

    // Update for Delay: '<S94>/Delay'
    rtDW.icLoad_dk = 0U;
    rtDW.Delay_DSTATE_k = scale;

    // Update for Delay: '<S94>/Delay1'
    rtDW.icLoad_j = 0U;
    rtDW.Delay1_DSTATE_h = rtb_Delay_n;

    // Update for Delay: '<S94>/Delay2'
    rtDW.icLoad_ix = 0U;
    rtDW.Delay2_DSTATE_h = rtb_Delay1_f;

    // Update for Delay: '<S94>/Delay3'
    rtDW.icLoad_ow = 0U;
    rtDW.Delay3_DSTATE_m = rtb_Delay2_b;

    // Update for DiscreteIntegrator: '<S96>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S96>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_IC_L_el = 0U;
    rtDW.DiscreteTimeIntegratory_DSTA_ny += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_a;

    // Update for DiscreteIntegrator: '<S96>/Discrete-Time Integrator y_dt' incorporates:
    //   Constant: '<S96>/omega'
    //   Product: '<S96>/Product1'
    //   Product: '<S96>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_DS_a += rtP.lindi.servo.omega *
      rtP.lindi.servo.boost * (rtP.lindi.servo.omega * rtP.lindi.servo.boost) *
      q_bg_unsigned_idx_0 * 0.0025F;

    // Update for Delay: '<S98>/Delay'
    rtDW.icLoad_dh = 0U;
    rtDW.Delay_DSTATE_n = scale;

    // Update for Delay: '<S98>/Delay1'
    rtDW.icLoad_f = 0U;
    rtDW.Delay1_DSTATE_k = rtb_Delay_o;

    // Update for Delay: '<S98>/Delay2'
    rtDW.icLoad_fc = 0U;
    rtDW.Delay2_DSTATE_p = rtb_Delay1_i;

    // Update for Delay: '<S98>/Delay3'
    rtDW.icLoad_jp = 0U;
    rtDW.Delay3_DSTATE_h = rtb_Delay2_o;

    // Update for DiscreteIntegrator: '<S130>/Discrete-Time Integrator y_dt' incorporates:
    //   Constant: '<S130>/omega'
    //   Product: '<S130>/Product1'
    //   Product: '<S130>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_DS_m += rtP.lindi.sflt.omega *
      rtP.lindi.sflt.omega * rtb_Sum2_eh * 0.0025F;

    // Update for DiscreteIntegrator: '<S129>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S129>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_IC_LO_b = 0U;
    rtDW.DiscreteTimeIntegratory_DSTAT_a += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_b;

    // Update for DiscreteIntegrator: '<S129>/Discrete-Time Integrator y_dt' incorporates:
    //   Constant: '<S129>/omega'
    //   Product: '<S129>/Product1'
    //   Product: '<S129>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_DS_b += rtP.lindi.servo.omega *
      rtP.lindi.servo.boost * (rtP.lindi.servo.omega * rtP.lindi.servo.boost) *
      rtb_Sum2_mb * 0.0025F;

    // Update for Delay: '<S131>/Delay'
    rtDW.icLoad_dhb = 0U;
    rtDW.Delay_DSTATE_j = t;

    // Update for Delay: '<S131>/Delay1'
    rtDW.icLoad_fx = 0U;
    rtDW.Delay1_DSTATE_ha = rtb_Delay_j;

    // Update for Delay: '<S131>/Delay2'
    rtDW.icLoad_m = 0U;
    rtDW.Delay2_DSTATE_m = rtb_Delay1_a;

    // Update for Delay: '<S131>/Delay3'
    rtDW.icLoad_dc = 0U;
    rtDW.Delay3_DSTATE_p = rtb_Delay2_oz;

    // Update for DiscreteIntegrator: '<S133>/Discrete-Time Integrator y_dt' incorporates:
    //   Constant: '<S133>/omega'
    //   Product: '<S133>/Product1'
    //   Product: '<S133>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_DS_k += rtP.lindi.servo.omega *
      rtP.lindi.servo.boost * (rtP.lindi.servo.omega * rtP.lindi.servo.boost) *
      rtb_Sum2_gn * 0.0025F;

    // Update for Delay: '<S135>/Delay'
    rtDW.icLoad_f1 = 0U;
    rtDW.Delay_DSTATE_jn = absxk;

    // Update for Delay: '<S135>/Delay1'
    rtDW.icLoad_fu = 0U;
    rtDW.Delay1_DSTATE_d = rtb_Delay_i;

    // Update for Delay: '<S135>/Delay2'
    rtDW.icLoad_ib = 0U;
    rtDW.Delay2_DSTATE_j = rtb_Delay1_eh;

    // Update for Delay: '<S135>/Delay3'
    rtDW.icLoad_kd = 0U;
    rtDW.Delay3_DSTATE_g = rtb_Delay2_k;

    // Update for DiscreteIntegrator: '<S52>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S52>/Product1'
    //   Product: '<S52>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_D_gf += q0_q0 * q0_q0 * rtb_Sum2_os *
      0.0025F;
  } else {
    if (rtDW.LindiPlaneAutopilot_MODE) {
      // Disable for Enabled SubSystem: '<S9>/Waypoint Navigation'
      if (rtDW.WaypointNavigation_MODE) {
        // Disable for Enabled SubSystem: '<S140>/Flight Path Smoothing'
        if (rtDW.FlightPathSmoothing_MODE) {
          rtDW.FlightPathSmoothing_MODE = false;
        }

        // End of Disable for SubSystem: '<S140>/Flight Path Smoothing'
        rtDW.WaypointNavigation_MODE = false;
      }

      // End of Disable for SubSystem: '<S9>/Waypoint Navigation'

      // Disable for Enabled SubSystem: '<S9>/NDI Position Controller'
      if (rtDW.NDIPositionController_MODE) {
        rtDW.NDIPositionController_MODE = false;
      }

      // End of Disable for SubSystem: '<S9>/NDI Position Controller'

      // Disable for Enabled SubSystem: '<S9>/Outer Loop INDI'
      if (rtDW.OuterLoopINDI_MODE) {
        rtDW.OuterLoopINDI_MODE = false;
      }

      // End of Disable for SubSystem: '<S9>/Outer Loop INDI'

      // Disable for Enabled SubSystem: '<S9>/Pitch Angle Controller'
      if (rtDW.PitchAngleController_MODE) {
        rtDW.PitchAngleController_MODE = false;
      }

      // End of Disable for SubSystem: '<S9>/Pitch Angle Controller'

      // Disable for Enabled SubSystem: '<S25>/Maneuver Load Alleviation'
      if (rtDW.ManeuverLoadAlleviation_MODE) {
        rtDW.ManeuverLoadAlleviation_MODE = false;
      }

      // End of Disable for SubSystem: '<S25>/Maneuver Load Alleviation'
      rtDW.LindiPlaneAutopilot_MODE = false;
    }
  }

  // End of RelationalOperator: '<S5>/Compare'
  // End of Outputs for SubSystem: '<Root>/LindiPlane Autopilot'

  // RelationalOperator: '<S16>/Compare' incorporates:
  //   Constant: '<S16>/Constant'
  //   Constant: '<S6>/Constant'
  //   Inport: '<Root>/cmd'
  //   RelationalOperator: '<S6>/Compare'

  rtb_Compare_j = (rtU.cmd.RC_pwm[6] > 1500.0F);

  // DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
  if (rtb_Compare_j && (rtDW.DiscreteTimeIntegrator_PrevRese <= 0)) {
    rtDW.DiscreteTimeIntegrator_DSTATE = 0.0F;
  }

  // MATLAB Function: '<S1>/1-cos Gust'
  // :  U = zeros(1,class(s));
  q1_q1 = 0.0F;

  // Switch: '<S1>/Switch' incorporates:
  //   Constant: '<S1>/Constant'
  //   DiscreteIntegrator: '<S1>/Discrete-Time Integrator'

  // :  cos_arg = 2*pi*s/gust_length;
  if (rtb_Compare_j) {
    q0_q0 = rtDW.DiscreteTimeIntegrator_DSTATE;
  } else {
    q0_q0 = 0.0F;
  }

  // End of Switch: '<S1>/Switch'

  // MATLAB Function: '<S1>/1-cos Gust' incorporates:
  //   Constant: '<S1>/Constant1'
  //   Constant: '<S1>/Constant2'

  q0_q0 = 6.28318548F * q0_q0 / rtP.lindi.gust.len;

  // :  if cos_arg > 0 && cos_arg <= 2*pi
  if ((q0_q0 > 0.0F) && (q0_q0 <= 6.2831853071795862)) {
    // :  U(:) = gust_magnitude/2 * ( 1 - cos( cos_arg ) );
    q1_q1 = rtP.lindi.gust.mag / 2.0F * (1.0F - std::cos(q0_q0));
  }

  // MinMax: '<S8>/Max' incorporates:
  //   Constant: '<S8>/Constant'

  if (rtP.lindi.aspd.min > rtDW.V_A_f) {
    q0_q0 = rtP.lindi.aspd.min;
  } else {
    q0_q0 = rtDW.V_A_f;
  }

  // End of MinMax: '<S8>/Max'

  // MATLAB Function: '<S7>/Angle of attack'
  // :  alpha_W = divideFinite( -w_W, V );
  rtb_Sum2_os = q0_q0;

  // 'divideFinite:29' if numel(B)>1
  // 'divideFinite:31' else
  // 'divideFinite:32' if abs(B)<eps
  if (std::abs(q0_q0) < 2.22044605E-16F) {
    // 'divideFinite:33' B(:) = eps;
    rtb_Sum2_os = 2.22044605E-16F;
  }

  // 'divideFinite:36' C = A ./ B;
  q1_q1 = -q1_q1 / rtb_Sum2_os;

  // End of MATLAB Function: '<S7>/Angle of attack'

  // MATLAB Function: '<S7>/Flap deflections'
  // :  u = zeros(length(cef.clu),1,class(alpha_W));
  for (i_0 = 0; i_0 < 10; i_0++) {
    rtb_Delta_u[i_0] = 0.0F;
  }

  // :  clu_w = cef.clu(1:end-2);
  // :  dalpha_du = sum(1/eig_.cla*clu_w);
  q2_q2 = 1.0F / rtP.lindi.eig.cla;
  for (stage_app_2 = 0; stage_app_2 < 8; stage_app_2++) {
    q2_q2_0[stage_app_2] = q2_q2 * rtP.lindi.cef.clu[stage_app_2];
  }

  q2_q2 = sum(q2_q2_0);

  // :  u(1:end-2) = divideFinite(1,dalpha_du)*alpha_W;
  // 'divideFinite:29' if numel(B)>1
  // 'divideFinite:31' else
  // 'divideFinite:32' if abs(B)<eps
  if (std::abs(q2_q2) < 2.22044605E-16F) {
    // 'divideFinite:33' B(:) = eps;
    q2_q2 = 2.22044605E-16F;
  }

  // 'divideFinite:36' C = A ./ B;
  // :  u(end-1) = -sum(clu_w.*u(1:end-2)'.*cef.s(1:end-2).*(cef.x(1:end-2)+eig_.xcg-eig_.xnp)) ... 
  // :      ./ (cef.clu(end-1)*cef.s(end-1)*(cef.x(end-1)+eig_.xcg-eig_.xnp));
  q1_q3 = 1.0F / q2_q2 * q1_q1;
  for (i_0 = 0; i_0 < 8; i_0++) {
    q2_q2_0[i_0] = rtP.lindi.cef.clu[i_0] * q1_q3 * rtP.lindi.cef.s[i_0] *
      ((rtP.lindi.cef.x[i_0] + rtP.lindi.eig.xcg) - rtP.lindi.eig.xnp);
    rtb_Delta_u[i_0] = q1_q3;
  }

  rtb_Delta_u[8] = -sum(q2_q2_0) / (((rtP.lindi.cef.x[8] + rtP.lindi.eig.xcg) -
    rtP.lindi.eig.xnp) * (rtP.lindi.cef.clu[8] * rtP.lindi.cef.s[8]));

  // :  clu_h = cef.clu(end-1);
  // :  u_dw_a = divideFinite(1,clu_h)*eig_.cla_h*eig_.dahda*alpha_W;
  q1_q2 = rtP.lindi.cef.clu[8];

  // 'divideFinite:29' if numel(B)>1
  // 'divideFinite:31' else
  // 'divideFinite:32' if abs(B)<eps
  rtb_Sum2_ms = std::abs(rtP.lindi.cef.clu[8]);

  // 'divideFinite:36' C = A ./ B;
  // :  u_dw_u = -divideFinite(1,clu_h)*eig_.cla_h*eig_.dahdu*u(1:end-2);
  q1_q3 = rtP.lindi.cef.clu[8];

  // 'divideFinite:29' if numel(B)>1
  // 'divideFinite:31' else
  // 'divideFinite:32' if abs(B)<eps
  if (rtb_Sum2_ms < 2.22044605E-16F) {
    // 'divideFinite:33' B(:) = eps;
    q1_q2 = 2.22044605E-16F;

    // 'divideFinite:33' B(:) = eps;
    q1_q3 = 2.22044605E-16F;
  }

  // 'divideFinite:36' C = A ./ B;
  rtb_y_e3 = -(1.0F / q1_q3) * rtP.lindi.eig.cla_h;

  // :  u_dw = u_dw_a + u_dw_u;
  rtb_Sum2_os = 0.0F;
  for (stage_app_2 = 0; stage_app_2 < 8; stage_app_2++) {
    rtb_Sum2_os += rtb_y_e3 * rtP.lindi.eig.dahdu[stage_app_2] *
      rtb_Delta_u[stage_app_2];
  }

  q1_q1 = 1.0F / q1_q2 * rtP.lindi.eig.cla_h * rtP.lindi.eig.dahda * q1_q1 +
    rtb_Sum2_os;

  // End of MATLAB Function: '<S7>/Flap deflections'

  // MATLAB Function: '<S7>/Elevator downwash compensation' incorporates:
  //   DiscreteIntegrator: '<S20>/Discrete-Time Integrator y'

  // :  u(end-1) = u(end-1) + u_dw;
  rtb_Delta_u[8] += rtDW.DiscreteTimeIntegratory_DSTATE;

  // MATLAB Function: '<S13>/MATLAB Function'
  // :  y = zeros(11,1,superiorfloat(u));
  // :  num_flaps = (length(u)-3)/2;
  // :  y(1:num_flaps) = u(1:num_flaps);
  // :  y(5:4+num_flaps) = u(num_flaps+1:2*num_flaps);
  rtb_y_if[0] = rtDW.u[0];
  rtb_y_if[4] = rtDW.u[4];
  rtb_y_if[1] = rtDW.u[1];
  rtb_y_if[5] = rtDW.u[5];
  rtb_y_if[2] = rtDW.u[2];
  rtb_y_if[6] = rtDW.u[6];
  rtb_y_if[3] = rtDW.u[3];
  rtb_y_if[7] = rtDW.u[7];

  // :  y(end-2:end) = u(end-2:end);
  rtb_y_if[8] = rtDW.u[8];
  rtb_y_if[9] = rtDW.u[9];
  rtb_y_if[10] = rtDW.u[10];

  // Switch: '<S13>/Switch' incorporates:
  //   Gain: '<S13>/Gain'
  //   Gain: '<S13>/Gain7'
  //   Gain: '<S13>/Gain8'
  //   Inport: '<Root>/cmd'

  if (rtb_Compare_mi) {
    for (i_0 = 0; i_0 < 8; i_0++) {
      rtb_y_if[i_0] = rtConstP.Gain_Gain_o[i_0] * rtU.cmd.roll;
    }

    rtb_y_if[8] = -rtU.cmd.pitch;
    rtb_y_if[9] = -rtU.cmd.yaw;
    rtb_y_if[10] = rtU.cmd.thr;
  }

  // End of Switch: '<S13>/Switch'

  // Sum: '<S13>/Add'
  for (i_0 = 0; i_0 < 10; i_0++) {
    rtb_Add_p[i_0] = rtb_y_if[i_0] + rtb_Delta_u[i_0];
  }

  // Outport: '<Root>/logs' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion'
  //   Gain: '<S13>/Gain1'
  //   Gain: '<S13>/Gain2'
  //   Gain: '<S13>/Gain3'
  //   Gain: '<S13>/Gain4'
  //   Gain: '<S13>/Gain5'
  //   Gain: '<S13>/Gain6'
  //   Inport: '<Root>/measure'
  //   MATLAB Function: '<S14>/Auxiliary function to define log_config in generated C++ code'
  //   SignalConversion: '<S163>/TmpSignal ConversionAt SFunction Inport2'
  //   Sum: '<Root>/Add'
  //   Sum: '<S13>/Add'

  rtY.logs[0] = -rtb_Add_p[0];
  rtY.logs[1] = -rtb_Add_p[1];
  rtY.logs[2] = -rtb_Add_p[2];
  rtY.logs[3] = -rtb_Add_p[3];
  rtY.logs[4] = rtb_Add_p[4];
  rtY.logs[5] = rtb_Add_p[5];
  rtY.logs[6] = rtb_Add_p[6];
  rtY.logs[7] = rtb_Add_p[7];
  rtY.logs[8] = -rtb_Add_p[8];
  rtY.logs[9] = -rtb_Add_p[9];
  rtY.logs[10] = rtb_y_if[10];
  rtY.logs[29] = rtDW.V_A_f;
  rtY.logs[30] = rtU.measure.airspeed;
  rtY.logs[31] = rtDW.wp_idx;
  rtY.logs[32] = rtDW.stage;
  rtY.logs[33] = rtDW.t;
  rtY.logs[11] = rtDW.Euler_angles_f[0];
  rtY.logs[14] = rtDW.Omega_Kb_f[0];
  rtY.logs[17] = rtDW.Omega_Kb_dt_f[0];
  rtY.logs[20] = rtDW.e_s_g[0];
  rtY.logs[23] = rtDW.e_s_g_dt[0];
  rtY.logs[26] = rtDW.e_s_g_dt2[0];
  rtY.logs[34] = rtDW.s_g_ref[0];
  rtY.logs[37] = rtDW.s_g[0];
  rtY.logs[40] = rtDW.s_g_match[0];
  rtY.logs[43] = rtU.measure.omega_Kb[0];
  rtY.logs[46] = rtU.measure.a_Kg[0];
  rtY.logs[12] = rtDW.Euler_angles_f[1];
  rtY.logs[15] = rtDW.Omega_Kb_f[1];
  rtY.logs[18] = rtDW.Omega_Kb_dt_f[1];
  rtY.logs[21] = rtDW.e_s_g[1];
  rtY.logs[24] = rtDW.e_s_g_dt[1];
  rtY.logs[27] = rtDW.e_s_g_dt2[1];
  rtY.logs[35] = rtDW.s_g_ref[1];
  rtY.logs[38] = rtDW.s_g[1];
  rtY.logs[41] = rtDW.s_g_match[1];
  rtY.logs[44] = rtU.measure.omega_Kb[1];
  rtY.logs[47] = rtU.measure.a_Kg[1];
  rtY.logs[13] = rtDW.Euler_angles_f[2];
  rtY.logs[16] = rtDW.Omega_Kb_f[2];
  rtY.logs[19] = rtDW.Omega_Kb_dt_f[2];
  rtY.logs[22] = rtDW.e_s_g[2];
  rtY.logs[25] = rtDW.e_s_g_dt[2];
  rtY.logs[28] = rtDW.e_s_g_dt2[2];
  rtY.logs[36] = rtDW.s_g_ref[2];
  rtY.logs[39] = rtDW.s_g[2];
  rtY.logs[42] = rtDW.s_g_match[2];
  rtY.logs[45] = rtU.measure.omega_Kb[2];
  rtY.logs[48] = rtU.measure.a_Kg[2] + 9.81F;
  rtY.logs[49] = rtb_Compare;
  rtY.logs[50] = rtDW.iter;
  for (i_0 = 0; i_0 < 8; i_0++) {
    rtY.logs[i_0 + 51] = rtU.measure.imu_p[i_0];
    rtY.logs[i_0 + 59] = rtU.measure.imu_a_z[i_0];
  }

  rtY.logs[67] = rtb_Delta_u[0];
  rtY.logs[68] = rtb_Delta_u[8];

  // End of Outport: '<Root>/logs'

  // Outport: '<Root>/channels' incorporates:
  //   DataTypeConversion: '<S2>/Data Type Conversion'
  //   Gain: '<S13>/Gain1'
  //   Gain: '<S13>/Gain2'
  //   Gain: '<S13>/Gain3'
  //   Gain: '<S13>/Gain4'
  //   Gain: '<S13>/Gain5'
  //   Gain: '<S13>/Gain6'
  //   Sum: '<S13>/Add'

  rtY.channels[0] = -rtb_Add_p[0];
  rtY.channels[1] = -rtb_Add_p[1];
  rtY.channels[2] = -rtb_Add_p[2];
  rtY.channels[3] = -rtb_Add_p[3];
  rtY.channels[4] = rtb_Add_p[4];
  rtY.channels[5] = rtb_Add_p[5];
  rtY.channels[6] = rtb_Add_p[6];
  rtY.channels[7] = rtb_Add_p[7];
  rtY.channels[8] = -rtb_Add_p[8];
  rtY.channels[9] = -rtb_Add_p[9];
  rtY.channels[10] = rtb_y_if[10];
  rtY.channels[11] = 0.0F;
  rtY.channels[12] = 0.0F;
  rtY.channels[13] = 0.0F;
  rtY.channels[14] = 0.0F;
  rtY.channels[15] = 0.0F;

  // Gain: '<S7>/Gain13'
  q2_q2 = 1.42F / rtP.lindi.eig.x_h * q0_q0;

  // Sum: '<S20>/Sum2' incorporates:
  //   Constant: '<S7>/Constant8'
  //   DiscreteIntegrator: '<S20>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S20>/Discrete-Time Integrator y_dt'
  //   Gain: '<S20>/Gain'
  //   Product: '<S20>/Divide'
  //   Product: '<S20>/Product2'
  //   Sum: '<S20>/Sum3'

  q1_q1 -= 0.71F / q2_q2 * 2.0F * rtDW.DiscreteTimeIntegratory_dt_DSTA +
    rtDW.DiscreteTimeIntegratory_DSTATE;

  // Update for DiscreteIntegrator: '<S20>/Discrete-Time Integrator y' incorporates:
  //   DiscreteIntegrator: '<S20>/Discrete-Time Integrator y_dt'

  rtDW.DiscreteTimeIntegratory_DSTATE += 0.0025F *
    rtDW.DiscreteTimeIntegratory_dt_DSTA;

  // Update for DiscreteIntegrator: '<S1>/Discrete-Time Integrator'
  rtDW.DiscreteTimeIntegrator_DSTATE += 0.0025F * q0_q0;
  rtDW.DiscreteTimeIntegrator_PrevRese = (int8_T)rtb_Compare_j;

  // Update for DiscreteIntegrator: '<S20>/Discrete-Time Integrator y_dt' incorporates:
  //   Product: '<S20>/Product1'
  //   Product: '<S20>/omega^2'

  rtDW.DiscreteTimeIntegratory_dt_DSTA += q2_q2 * q2_q2 * q1_q1 * 0.0025F;
}

// Model initialize function
void MatlabControllerClass::initialize()
{
  {
    int32_T i;

    // ConstCode for Outport: '<Root>/function_channels' incorporates:
    //   Constant: '<S2>/Constant'

    for (i = 0; i < 16; i++) {
      rtY.function_channels[i] = rtConstP.Constant_Value_n[i];
    }

    // End of ConstCode for Outport: '<Root>/function_channels'

    // InitializeConditions for DiscreteIntegrator: '<S1>/Discrete-Time Integrator' 
    rtDW.DiscreteTimeIntegrator_PrevRese = 2;

    // SystemInitialize for Enabled SubSystem: '<Root>/LindiPlane Autopilot'
    // InitializeConditions for DiscreteIntegrator: '<S52>/Discrete-Time Integrator y' 
    rtDW.DiscreteTimeIntegratory_IC_LOAD = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S48>/Discrete-Time Integrator y' 
    rtDW.DiscreteTimeIntegratory_IC_LO_a = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S50>/Discrete-Time Integrator y' 
    rtDW.DiscreteTimeIntegratory_IC_LO_e = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S51>/Discrete-Time Integrator y' 
    rtDW.DiscreteTimeIntegratory_IC_LO_i = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S80>/Discrete-Time Integrator' 
    rtDW.DiscreteTimeIntegrator_IC_LOADI = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S134>/Discrete-Time Integrator y' 
    rtDW.DiscreteTimeIntegratory_IC_LO_m = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S133>/Discrete-Time Integrator y' 
    rtDW.DiscreteTimeIntegratory_IC_LO_d = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S130>/Discrete-Time Integrator y' 
    rtDW.DiscreteTimeIntegratory_IC_LO_o = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S97>/Discrete-Time Integrator y' 
    rtDW.DiscreteTimeIntegratory_IC_L_eu = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S93>/Discrete-Time Integrator y' 
    rtDW.DiscreteTimeIntegratory_IC_LO_k = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S154>/Discrete-Time Integrator' 
    rtDW.DiscreteTimeIntegrator_IC_LOA_g = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S157>/Discrete-Time Integrator y' 
    rtDW.DiscreteTimeIntegratory_IC_LO_f = 1U;

    // InitializeConditions for Delay: '<S107>/Delay'
    rtDW.icLoad = 1U;

    // InitializeConditions for Delay: '<S107>/Delay1'
    rtDW.icLoad_i = 1U;

    // InitializeConditions for Delay: '<S107>/Delay2'
    rtDW.icLoad_o = 1U;

    // InitializeConditions for Delay: '<S107>/Delay3'
    rtDW.icLoad_k = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S156>/Discrete-Time Integrator y' 
    rtDW.DiscreteTimeIntegratory_IC_L_km = 1U;

    // InitializeConditions for Delay: '<S158>/Delay'
    rtDW.icLoad_d = 1U;

    // InitializeConditions for Delay: '<S158>/Delay1'
    rtDW.icLoad_b = 1U;

    // InitializeConditions for Delay: '<S158>/Delay2'
    rtDW.icLoad_l = 1U;

    // InitializeConditions for Delay: '<S158>/Delay3'
    rtDW.icLoad_b1 = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S92>/Discrete-Time Integrator y' 
    rtDW.DiscreteTimeIntegratory_IC_LO_g = 1U;

    // InitializeConditions for Delay: '<S94>/Delay'
    rtDW.icLoad_dk = 1U;

    // InitializeConditions for Delay: '<S94>/Delay1'
    rtDW.icLoad_j = 1U;

    // InitializeConditions for Delay: '<S94>/Delay2'
    rtDW.icLoad_ix = 1U;

    // InitializeConditions for Delay: '<S94>/Delay3'
    rtDW.icLoad_ow = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S96>/Discrete-Time Integrator y' 
    rtDW.DiscreteTimeIntegratory_IC_L_el = 1U;

    // InitializeConditions for Delay: '<S98>/Delay'
    rtDW.icLoad_dh = 1U;

    // InitializeConditions for Delay: '<S98>/Delay1'
    rtDW.icLoad_f = 1U;

    // InitializeConditions for Delay: '<S98>/Delay2'
    rtDW.icLoad_fc = 1U;

    // InitializeConditions for Delay: '<S98>/Delay3'
    rtDW.icLoad_jp = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S129>/Discrete-Time Integrator y' 
    rtDW.DiscreteTimeIntegratory_IC_LO_b = 1U;

    // InitializeConditions for Delay: '<S131>/Delay'
    rtDW.icLoad_dhb = 1U;

    // InitializeConditions for Delay: '<S131>/Delay1'
    rtDW.icLoad_fx = 1U;

    // InitializeConditions for Delay: '<S131>/Delay2'
    rtDW.icLoad_m = 1U;

    // InitializeConditions for Delay: '<S131>/Delay3'
    rtDW.icLoad_dc = 1U;

    // InitializeConditions for Delay: '<S135>/Delay'
    rtDW.icLoad_f1 = 1U;

    // InitializeConditions for Delay: '<S135>/Delay1'
    rtDW.icLoad_fu = 1U;

    // InitializeConditions for Delay: '<S135>/Delay2'
    rtDW.icLoad_ib = 1U;

    // InitializeConditions for Delay: '<S135>/Delay3'
    rtDW.icLoad_kd = 1U;

    // SystemInitialize for Enabled SubSystem: '<S9>/Waypoint Navigation'
    // InitializeConditions for UnitDelay: '<S37>/Unit Delay'
    rtDW.UnitDelay_DSTATE_b = 1;

    // InitializeConditions for UnitDelay: '<S37>/Unit Delay1'
    rtDW.UnitDelay1_DSTATE_g = 1;

    // InitializeConditions for UnitDelay: '<S37>/Unit Delay2'
    rtDW.UnitDelay2_DSTATE = 2;

    // SystemInitialize for Enabled SubSystem: '<S140>/Flight Path Smoothing'
    // InitializeConditions for DiscreteIntegrator: '<S150>/Discrete-Time Integrator y' 
    rtDW.DiscreteTimeIntegratory_IC_L_io = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S151>/Discrete-Time Integrator y' 
    rtDW.DiscreteTimeIntegratory_IC_L_o2 = 1U;

    // End of SystemInitialize for SubSystem: '<S140>/Flight Path Smoothing'
    // End of SystemInitialize for SubSystem: '<S9>/Waypoint Navigation'

    // SystemInitialize for Enabled SubSystem: '<S9>/NDI Position Controller'
    // InitializeConditions for DiscreteIntegrator: '<S65>/Discrete-Time Integrator y' 
    rtDW.DiscreteTimeIntegratory_IC_L_d2 = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S64>/Discrete-Time Integrator y' 
    rtDW.DiscreteTimeIntegratory_IC_LO_p = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S66>/Discrete-Time Integrator y' 
    rtDW.DiscreteTimeIntegratory_IC_L_ls = 1U;

    // End of SystemInitialize for SubSystem: '<S9>/NDI Position Controller'

    // SystemInitialize for Enabled SubSystem: '<S9>/Waypoint Navigation'
    // InitializeConditions for UnitDelay: '<S37>/Unit Delay6'
    rtDW.UnitDelay6_DSTATE[0] = 1.0F;
    rtDW.UnitDelay6_DSTATE[1] = 1.0F;
    rtDW.UnitDelay6_DSTATE[2] = 1.0F;

    // End of SystemInitialize for SubSystem: '<S9>/Waypoint Navigation'

    // SystemInitialize for Enabled SubSystem: '<S26>/Duplicate LPF'
    // InitializeConditions for DiscreteIntegrator: '<S56>/Discrete-Time Integrator1' 
    rtDW.DiscreteTimeIntegrator1_IC_LOAD = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S55>/Discrete-Time Integrator1' 
    rtDW.DiscreteTimeIntegrator1_IC_LO_g = 1U;

    // End of SystemInitialize for SubSystem: '<S26>/Duplicate LPF'

    // SystemInitialize for Enabled SubSystem: '<S9>/Outer Loop INDI'
    // InitializeConditions for DiscreteIntegrator: '<S69>/Discrete-Time Integrator' 
    rtDW.DiscreteTimeIntegrator_IC_LOA_l = 1U;

    // End of SystemInitialize for SubSystem: '<S9>/Outer Loop INDI'

    // SystemInitialize for Enabled SubSystem: '<S9>/Pitch Angle Controller'
    // InitializeConditions for DiscreteIntegrator: '<S72>/Discrete-Time Integrator' 
    rtDW.DiscreteTimeIntegrator_IC_LOA_m = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S73>/Discrete-Time Integrator' 
    rtDW.DiscreteTimeIntegrator_IC_LO_mz = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S76>/Discrete-Time Integrator y' 
    rtDW.DiscreteTimeIntegratory_IC_LO_l = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S75>/Discrete-Time Integrator y' 
    rtDW.DiscreteTimeIntegratory_IC_L_id = 1U;

    // InitializeConditions for Delay: '<S77>/Delay'
    rtDW.icLoad_fe = 1U;

    // InitializeConditions for Delay: '<S77>/Delay1'
    rtDW.icLoad_lk = 1U;

    // InitializeConditions for Delay: '<S77>/Delay2'
    rtDW.icLoad_jd = 1U;

    // InitializeConditions for Delay: '<S77>/Delay3'
    rtDW.icLoad_f13 = 1U;

    // End of SystemInitialize for SubSystem: '<S9>/Pitch Angle Controller'
    // End of SystemInitialize for SubSystem: '<Root>/LindiPlane Autopilot'
  }
}

// Constructor
MatlabControllerClass::MatlabControllerClass()
{
  AP_Param::setup_object_defaults(this, var_info_rtP_lindi_cef);
  AP_Param::setup_object_defaults(this, var_info_rtP_lindi_ceb);
  AP_Param::setup_object_defaults(this, var_info_rtP_lindi_servo);
  AP_Param::setup_object_defaults(this, var_info_rtP_lindi_sflt);
  AP_Param::setup_object_defaults(this, var_info_rtP_lindi_aspd);
  AP_Param::setup_object_defaults(this, var_info_rtP_lindi_atc);
  AP_Param::setup_object_defaults(this, var_info_rtP_lindi_eig);
  AP_Param::setup_object_defaults(this, var_info_rtP_lindi_psc);
  AP_Param::setup_object_defaults(this, var_info_rtP_lindi_wpnav);
  AP_Param::setup_object_defaults(this, var_info_rtP_lindi_ca);
  AP_Param::setup_object_defaults(this, var_info_rtP_lindi_dlc);
  AP_Param::setup_object_defaults(this, var_info_rtP_lindi_mla);
  AP_Param::setup_object_defaults(this, var_info_rtP_lindi_gust);
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
