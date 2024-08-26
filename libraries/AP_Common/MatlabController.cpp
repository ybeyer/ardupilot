//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: MatlabController.cpp
//
// Code generated for Simulink model 'ArduPlane_LindiPlane'.
//
// Model version                  : 1.777
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Mon Aug 26 20:49:29 2024
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
  }, { 11U,
    { 112U, 54U, 1U, 112U, 55U, 1U, 112U, 56U, 1U, 97U, 122U, 49U, 97U, 122U,
      50U, 97U, 122U, 51U, 97U, 122U, 52U, 97U, 122U, 53U, 97U, 122U, 54U, 97U,
      122U, 55U, 97U, 122U, 56U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U },

    { 77U, 76U, 53U, 0U }
  } } ;

extern real32_T rt_hypotf(real32_T u0, real32_T u1);
static void PT2split(const real32_T rtu_y_n[6], real32_T rty_y[3], real32_T
                     rty_y_dt[3]);
static void Onlyrotationalcontroleffectiven(const real32_T rtu_G[60], real32_T
  rty_G3x4[30]);
static void wrapangle(real32_T rtu_angle, real32_T *rty_angle_0_2pi);
static void LookAhead(const real32_T rtu_p[3], const real32_T rtu_v[3], const
                      real32_T rtu_a[3], real32_T rtu_Delta_t, real32_T
                      rty_p_ahead[3]);

//
// Output and update for atomic system:
//    '<S37>/PT2 split'
//    '<S38>/PT2 split'
//
static void PT2split(const real32_T rtu_y_n[6], real32_T rty_y[3], real32_T
                     rty_y_dt[3])
{
  // :  y = y_n(2,:)';
  // :  y_dt = y_n(1,:)';
  rty_y[0] = rtu_y_n[1];
  rty_y_dt[0] = rtu_y_n[0];
  rty_y[1] = rtu_y_n[3];
  rty_y_dt[1] = rtu_y_n[2];
  rty_y[2] = rtu_y_n[5];
  rty_y_dt[2] = rtu_y_n[4];
}

//
// Output and update for atomic system:
//    '<S20>/Only rotational control effectiveness'
//    '<S20>/Only rotational control effectiveness1'
//    '<S20>/Only rotational control effectiveness2'
//
static void Onlyrotationalcontroleffectiven(const real32_T rtu_G[60], real32_T
  rty_G3x4[30])
{
  int32_T i;

  // :  G3x4 = G(1:3,:);
  for (i = 0; i < 10; i++) {
    rty_G3x4[3 * i] = rtu_G[6 * i];
    rty_G3x4[1 + 3 * i] = rtu_G[6 * i + 1];
    rty_G3x4[2 + 3 * i] = rtu_G[6 * i + 2];
  }

  // :  G3x4(2,1:end-2) = 0;
  for (i = 0; i < 8; i++) {
    rty_G3x4[1 + 3 * i] = 0.0F;
  }
}

//
// Output and update for atomic system:
//    '<S83>/wrap angle'
//    '<S83>/wrap angle1'
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

//
// Output and update for atomic system:
//    '<S25>/Look Ahead'
//    '<S25>/Look Ahead1'
//
static void LookAhead(const real32_T rtu_p[3], const real32_T rtu_v[3], const
                      real32_T rtu_a[3], real32_T rtu_Delta_t, real32_T
                      rty_p_ahead[3])
{
  // :  p_ahead = p + v*Delta_t + 0.5*a*Delta_t*Delta_t;
  rty_p_ahead[0] = 0.5F * rtu_a[0] * rtu_Delta_t * rtu_Delta_t + (rtu_v[0] *
    rtu_Delta_t + rtu_p[0]);
  rty_p_ahead[1] = 0.5F * rtu_a[1] * rtu_Delta_t * rtu_Delta_t + (rtu_v[1] *
    rtu_Delta_t + rtu_p[1]);
  rty_p_ahead[2] = 0.5F * rtu_a[2] * rtu_Delta_t * rtu_Delta_t + (rtu_v[2] *
    rtu_Delta_t + rtu_p[2]);
}

// Function for MATLAB Function: '<S76>/indiCeFlapFix'
void MatlabControllerClass::cross(const real32_T a[30], const real32_T b[30],
  real32_T c[30])
{
  int32_T iStart;
  int32_T i1;
  real32_T c_tmp;
  real32_T c_tmp_0;
  real32_T c_tmp_1;
  real32_T c_tmp_2;
  for (iStart = 0; iStart <= 28; iStart += 3) {
    for (i1 = iStart; i1 < iStart + 1; i1++) {
      c_tmp = a[i1 + 2];
      c_tmp_0 = b[i1 + 2];
      c_tmp_1 = b[i1 + 1];
      c_tmp_2 = a[i1 + 1];
      c[i1] = c_tmp_2 * c_tmp_0 - c_tmp * c_tmp_1;
      c[i1 + 1] = c_tmp * b[i1] - c_tmp_0 * a[i1];
      c[i1 + 2] = c_tmp_1 * a[i1] - c_tmp_2 * b[i1];
    }
  }
}

// Function for MATLAB Function: '<S25>/WpNav Matching'
real32_T MatlabControllerClass::xnrm2_f(const real32_T x[3])
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
// Function for MATLAB Function: '<S25>/WpNav Matching'
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
  dist1_tmp = xnrm2_f(circ_seg_start);

  // 'wpnavCircSeg:32' dist2 = norm(diff2,2);
  dist2_tmp = xnrm2_f(circ_seg_end);

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
  absxk = xnrm2_f(circ_seg_n);

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
  dist1 = xnrm2_f(circ_seg_center);

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
// Function for MATLAB Function: '<S25>/WpNav Matching'
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
  axis_length = xnrm2_f(axis);

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
// Function for MATLAB Function: '<S25>/WpNav Matching'
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
  denom = xnrm2_f(r_p0) * xnrm2_f(r_start);

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
  *d = xnrm2_f(r_start);

  // 'wpnavMatchCircSeg:53' R_center = circ_seg.center - circ_seg.wp;
  // 'wpnavMatchCircSeg:54' R_center_norm = norm(R_center,2);
  H = xnrm2_f(r_p0);

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
// Function for MATLAB Function: '<S25>/WpNav Matching'
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
  *d = xnrm2_f(p_match_0);
}

//
// Function for MATLAB Function: '<S25>/WpNav Matching'
// function [p_match,wp_idx,stage,t,d] = wpnavMatch( waypoints, wp_radius, wp_idx, stage, p )
//
void MatlabControllerClass::wpnavMatch_p(const real32_T waypoints_data[], const
  int32_T waypoints_size[2], real32_T wp_radius, int32_T *wp_idx, int32_T *stage,
  const real32_T p[3], real32_T p_match[3], real32_T *t, real32_T *d)
{
  int32_T num_wp;
  boolean_T is_initial_line;
  boolean_T is_last_line;
  int32_T idx3[3];
  real32_T circ_seg_center[3];
  real32_T circ_seg_n[3];
  real32_T circ_seg_angle;
  real32_T circ_seg_end[3];
  real32_T circ_seg_wp[3];
  real32_T p1[3];
  int32_T idx2[2];
  real32_T p2[3];
  real32_T expl_temp;
  real32_T expl_temp_0;
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
    if (*wp_idx == 2) {
      // 'wpnavMatch:72' is_initial_line = true;
      is_initial_line = true;
    } else {
      // 'wpnavMatch:73' else
      // 'wpnavMatch:74' is_initial_line = false;
      is_initial_line = false;
    }

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

      wpnavCircSeg(waypoints, wp_radius, &expl_temp, circ_seg_center, circ_seg_n,
                   &circ_seg_angle, p2, circ_seg_end, &expl_temp_0, circ_seg_wp);

      // 'wpnavMatch:86' [p_match(:),t(:),d(:)] = wpnavMatchCircSeg(circ_seg,p); 
      wpnavMatchCircSeg(circ_seg_center, circ_seg_n, circ_seg_angle, p2,
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
        if (is_initial_line) {
          // 'wpnavMatch:100' p1 = waypoints(:,1);
          p1[0] = waypoints_data[0];
          p1[1] = waypoints_data[1];
          p1[2] = waypoints_data[2];
        } else {
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

          wpnavCircSeg(waypoints, wp_radius, &expl_temp, circ_seg_center,
                       circ_seg_n, &circ_seg_angle, p2, p1, &expl_temp_0,
                       circ_seg_wp);

          // 'wpnavMatch:104' p1 = circ_seg_1.end;
        }

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
          wpnavCircSeg(waypoints, wp_radius, &expl_temp, circ_seg_center,
                       circ_seg_n, &circ_seg_angle, p2, circ_seg_end,
                       &expl_temp_0, circ_seg_wp);

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

          wpnavCircSeg(waypoints, wp_radius, &expl_temp, circ_seg_center,
                       circ_seg_n, &circ_seg_angle, p2, circ_seg_end,
                       &expl_temp_0, circ_seg_wp);

          // 'wpnavMatch:117' p2 = circ_seg_2.start;
        }

        // 'wpnavMatch:119' line_length = norm( p1 - p2, 2);
        // 'wpnavMatch:120' if line_length < 1
        circ_seg_center[0] = p1[0] - p2[0];
        circ_seg_center[1] = p1[1] - p2[1];
        circ_seg_center[2] = p1[2] - p2[2];
        if (xnrm2_f(circ_seg_center) < 1.0F) {
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
          wpnavMatchLine(p1, p2, p, p_match, t, d);

          // 'wpnavMatch:125' if t > 1 && norm(p-p2,2) < wp_radius
          if (*t > 1.0F) {
            circ_seg_center[0] = p[0] - p2[0];
            circ_seg_center[1] = p[1] - p2[1];
            circ_seg_center[2] = p[2] - p2[2];
            if (xnrm2_f(circ_seg_center) < wp_radius) {
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
// Function for MATLAB Function: '<S25>/WpNav Matching'
// function [p_match,wp_idx,stage,t,d] = wpnavMatch( waypoints, wp_radius, wp_idx, stage, p )
//
void MatlabControllerClass::wpnavMatch(const real32_T waypoints[15], real32_T
  wp_radius, int32_T *wp_idx, int32_T *stage, const real32_T p[3], real32_T
  p_match[3], real32_T *t, real32_T *d)
{
  boolean_T is_initial_line;
  boolean_T is_last_line;
  int32_T idx3[3];
  real32_T circ_seg_center[3];
  real32_T circ_seg_n[3];
  real32_T circ_seg_angle;
  real32_T circ_seg_end[3];
  real32_T circ_seg_wp[3];
  real32_T p1[3];
  int32_T idx2[2];
  real32_T p2[3];
  real32_T expl_temp;
  real32_T expl_temp_0;
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
    if (*wp_idx == 2) {
      // 'wpnavMatch:72' is_initial_line = true;
      is_initial_line = true;
    } else {
      // 'wpnavMatch:73' else
      // 'wpnavMatch:74' is_initial_line = false;
      is_initial_line = false;
    }

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

      wpnavCircSeg(waypoints_0, wp_radius, &expl_temp, circ_seg_center,
                   circ_seg_n, &circ_seg_angle, p2, circ_seg_end, &expl_temp_0,
                   circ_seg_wp);

      // 'wpnavMatch:86' [p_match(:),t(:),d(:)] = wpnavMatchCircSeg(circ_seg,p); 
      wpnavMatchCircSeg(circ_seg_center, circ_seg_n, circ_seg_angle, p2,
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
        if (is_initial_line) {
          // 'wpnavMatch:100' p1 = waypoints(:,1);
          p1[0] = waypoints[0];
          p1[1] = waypoints[1];
          p1[2] = waypoints[2];
        } else {
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

          wpnavCircSeg(waypoints_0, wp_radius, &expl_temp, circ_seg_center,
                       circ_seg_n, &circ_seg_angle, p2, p1, &expl_temp_0,
                       circ_seg_wp);

          // 'wpnavMatch:104' p1 = circ_seg_1.end;
        }

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
          wpnavCircSeg(waypoints_0, wp_radius, &expl_temp, circ_seg_center,
                       circ_seg_n, &circ_seg_angle, p2, circ_seg_end,
                       &expl_temp_0, circ_seg_wp);

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

          wpnavCircSeg(waypoints_0, wp_radius, &expl_temp, circ_seg_center,
                       circ_seg_n, &circ_seg_angle, p2, circ_seg_end,
                       &expl_temp_0, circ_seg_wp);

          // 'wpnavMatch:117' p2 = circ_seg_2.start;
        }

        // 'wpnavMatch:119' line_length = norm( p1 - p2, 2);
        // 'wpnavMatch:120' if line_length < 1
        circ_seg_center[0] = p1[0] - p2[0];
        circ_seg_center[1] = p1[1] - p2[1];
        circ_seg_center[2] = p1[2] - p2[2];
        if (xnrm2_f(circ_seg_center) < 1.0F) {
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
          wpnavMatchLine(p1, p2, p, p_match, t, d);

          // 'wpnavMatch:125' if t > 1 && norm(p-p2,2) < wp_radius
          if (*t > 1.0F) {
            circ_seg_center[0] = p[0] - p2[0];
            circ_seg_center[1] = p[1] - p2[1];
            circ_seg_center[2] = p[2] - p2[2];
            if (xnrm2_f(circ_seg_center) < wp_radius) {
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

// Function for MATLAB Function: '<S17>/Outer Loop INDI'
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

// Function for MATLAB Function: '<S32>/Maneuver Load Alleviation'
void MatlabControllerClass::LSQFromQR(const real32_T A_data[], const int32_T
  A_size[2], const real32_T tau_data[], const int32_T jpvt_data[], real32_T B_3
  [11], int32_T rankA, real32_T Y_data[], int32_T *Y_size)
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
      for (loop_ub = b_j + 1; loop_ub + 1 < 12; loop_ub++) {
        wj += A_data[11 * b_j + loop_ub] * B_3[loop_ub];
      }

      wj *= tau_data[b_j];
      if (wj != 0.0F) {
        B_3[b_j] -= wj;
        for (loop_ub = b_j + 1; loop_ub + 1 < 12; loop_ub++) {
          B_3[loop_ub] -= A_data[11 * b_j + loop_ub] * wj;
        }
      }
    }
  }

  for (loop_ub = 0; loop_ub < rankA; loop_ub++) {
    Y_data[jpvt_data[loop_ub] - 1] = B_3[loop_ub];
  }

  for (loop_ub = rankA - 1; loop_ub + 1 > 0; loop_ub--) {
    Y_data[jpvt_data[loop_ub] - 1] /= A_data[11 * loop_ub + loop_ub];
    for (b_j = 0; b_j < loop_ub; b_j++) {
      Y_data[jpvt_data[b_j] - 1] -= A_data[11 * loop_ub + b_j] *
        Y_data[jpvt_data[loop_ub] - 1];
    }
  }
}

// Function for MATLAB Function: '<S32>/Maneuver Load Alleviation'
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

// Function for MATLAB Function: '<S32>/Maneuver Load Alleviation'
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
      coltop = lastc * 11 + ic0;
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
      jy = 11 * lastc + ic0;
      for (iac = ic0; iac <= jy; iac += 11) {
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
        coltop += 11;
      }
    }
  }
}

// Function for MATLAB Function: '<S32>/Maneuver Load Alleviation'
void MatlabControllerClass::qrsolve(const real32_T A_data[], const int32_T
  A_size[2], const real32_T B_1[11], real32_T Y_data[], int32_T *Y_size)
{
  real32_T b_A_data[88];
  real32_T tau_data[8];
  int32_T jpvt_data[8];
  int32_T n;
  real32_T work_data[8];
  real32_T vn1_data[8];
  real32_T vn2_data[8];
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
  real32_T B_2[11];
  int32_T b_A_size[2];
  int8_T c_idx_0;
  b_A_size[0] = 11;
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
      for (nmi = b_n; nmi <= b_n + 10; nmi++) {
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
      b_n += 11;
    }

    for (b_n = 0; b_n < n; b_n++) {
      yk = b_n * 11 + b_n;
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
        b_ix = 11 * ix;
        iy = 11 * b_n;
        for (d_k = 0; d_k < 11; d_k++) {
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
      s = xnrm2(10 - b_n, b_A_data, yk + 2);
      if (s != 0.0F) {
        s = rt_hypotf(b_A_data[yk], s);
        if (b_A_data[yk] >= 0.0F) {
          s = -s;
        }

        if (std::abs(s) < 9.86076132E-32F) {
          ix = -1;
          b_ix = (yk - b_n) + 11;
          do {
            ix++;
            for (iy = yk + 1; iy < b_ix; iy++) {
              b_A_data[iy] *= 1.01412048E+31F;
            }

            s *= 1.01412048E+31F;
            smax *= 1.01412048E+31F;
          } while (std::abs(s) < 9.86076132E-32F);

          s = rt_hypotf(smax, xnrm2(10 - b_n, b_A_data, yk + 2));
          if (smax >= 0.0F) {
            s = -s;
          }

          tau_data[b_n] = (s - smax) / s;
          smax = 1.0F / (smax - s);
          b_ix = (yk - b_n) + 11;
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
          ix = (yk - b_n) + 11;
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
        xzlarf(11 - b_n, nmi - 1, yk + 1, tau_data[b_n], b_A_data, (b_n + (b_n +
                 1) * 11) + 1, work_data);
        b_A_data[yk] = smax;
      }

      for (yk = b_n + 1; yk < n; yk++) {
        if (vn1_data[yk] != 0.0F) {
          nmi = 11 * yk + b_n;
          smax = std::abs(b_A_data[nmi]) / vn1_data[yk];
          smax = 1.0F - smax * smax;
          if (smax < 0.0F) {
            smax = 0.0F;
          }

          s = vn1_data[yk] / vn2_data[yk];
          s = s * s * smax;
          if (s <= 0.000345266977F) {
            vn1_data[yk] = xnrm2(10 - b_n, b_A_data, nmi + 2);
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
    while ((n < b_A_size[1]) && (std::abs(b_A_data[11 * n + n]) > 1.31130219E-5F
            * std::abs(b_A_data[0]))) {
      n++;
    }
  }

  for (b_ix = 0; b_ix < 11; b_ix++) {
    B_2[b_ix] = B_1[b_ix];
  }

  LSQFromQR(b_A_data, b_A_size, tau_data, jpvt_data, B_2, n, Y_data, Y_size);
}

// Function for MATLAB Function: '<S32>/Maneuver Load Alleviation'
void MatlabControllerClass::mldivide(const real32_T A_data[], const int32_T
  A_size[2], const real32_T B_0[11], real32_T Y_data[], int32_T *Y_size)
{
  if (A_size[1] == 0) {
    *Y_size = 0;
  } else {
    qrsolve(A_data, A_size, B_0, Y_data, Y_size);
  }
}

// Function for MATLAB Function: '<S32>/Maneuver Load Alleviation'
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
// Function for MATLAB Function: '<S32>/Maneuver Load Alleviation'
// function [u,W,iter] = wls_alloc(B,v,umin,umax,Wv,Wu,ud,gam,u,W,imax)
//
real32_T MatlabControllerClass::wls_alloc(const real32_T B_4[24], const real32_T
  v[3], const real32_T umin[8], const real32_T umax[8], const real32_T Wv[9],
  const real32_T Wu[64], const real32_T ud[8], real32_T gam, real32_T u[8],
  real32_T W[8], real32_T imax)
{
  real32_T iter;
  real32_T gam_sq;
  real32_T A[88];
  real32_T d[11];
  boolean_T i_free[8];
  real32_T A_free_data[88];
  real32_T p_free_data[8];
  real_T p[8];
  real32_T u_opt[8];
  real_T dist[8];
  real_T b_data[8];
  int8_T e_data[8];
  int8_T f_data[8];
  int8_T g_data[8];
  int8_T h_data[8];
  int32_T aoffset;
  int32_T b_aoffset;
  real32_T A_tmp[9];
  int32_T i;
  boolean_T u_opt_data[8];
  real32_T A_tmp_0[24];
  real32_T A_tmp_1[3];
  real32_T A_tmp_2[11];
  real32_T A_0[11];
  boolean_T u_opt_0[8];
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
  for (b_aoffset = 0; b_aoffset < 9; b_aoffset++) {
    A_tmp[b_aoffset] = gam_sq * Wv[b_aoffset];
  }

  for (b_aoffset = 0; b_aoffset < 8; b_aoffset++) {
    for (f_size_idx_0 = 0; f_size_idx_0 < 3; f_size_idx_0++) {
      A_tmp_tmp = f_size_idx_0 + 3 * b_aoffset;
      A_tmp_0[A_tmp_tmp] = 0.0F;
      A_tmp_tmp_0 = 3 * b_aoffset + f_size_idx_0;
      A_tmp_0[A_tmp_tmp] = A_tmp_0[A_tmp_tmp_0] + B_4[3 * b_aoffset] *
        A_tmp[f_size_idx_0];
      A_tmp_0[A_tmp_tmp] = B_4[3 * b_aoffset + 1] * A_tmp[f_size_idx_0 + 3] +
        A_tmp_0[A_tmp_tmp_0];
      A_tmp_0[A_tmp_tmp] = B_4[3 * b_aoffset + 2] * A_tmp[f_size_idx_0 + 6] +
        A_tmp_0[A_tmp_tmp_0];
    }
  }

  for (b_aoffset = 0; b_aoffset < 8; b_aoffset++) {
    A[11 * b_aoffset] = A_tmp_0[3 * b_aoffset];
    A[1 + 11 * b_aoffset] = A_tmp_0[3 * b_aoffset + 1];
    A[2 + 11 * b_aoffset] = A_tmp_0[3 * b_aoffset + 2];
    for (f_size_idx_0 = 0; f_size_idx_0 < 8; f_size_idx_0++) {
      A[(f_size_idx_0 + 11 * b_aoffset) + 3] = Wu[(b_aoffset << 3) +
        f_size_idx_0];
    }
  }

  // 'wls_alloc:57' b = [gam_sq*Wv*v ; Wu*ud];
  // 'wls_alloc:60' d = b - A*u;
  for (b_aoffset = 0; b_aoffset < 3; b_aoffset++) {
    A_tmp_1[b_aoffset] = A_tmp[b_aoffset + 6] * v[2] + (A_tmp[b_aoffset + 3] *
      v[1] + A_tmp[b_aoffset] * v[0]);
  }

  A_tmp_2[0] = A_tmp_1[0];
  A_tmp_2[1] = A_tmp_1[1];
  A_tmp_2[2] = A_tmp_1[2];
  for (b_aoffset = 0; b_aoffset < 8; b_aoffset++) {
    u_opt[b_aoffset] = 0.0F;
    for (f_size_idx_0 = 0; f_size_idx_0 < 8; f_size_idx_0++) {
      u_opt[b_aoffset] += Wu[(f_size_idx_0 << 3) + b_aoffset] * ud[f_size_idx_0];
    }

    A_tmp_2[b_aoffset + 3] = u_opt[b_aoffset];
  }

  for (b_aoffset = 0; b_aoffset < 11; b_aoffset++) {
    A_0[b_aoffset] = 0.0F;
    for (f_size_idx_0 = 0; f_size_idx_0 < 8; f_size_idx_0++) {
      A_0[b_aoffset] += A[11 * f_size_idx_0 + b_aoffset] * u[f_size_idx_0];
    }

    d[b_aoffset] = A_tmp_2[b_aoffset] - A_0[b_aoffset];
  }

  // 'wls_alloc:62' i_free = W==0;
  for (i = 0; i < 8; i++) {
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
    for (b_aoffset = 0; b_aoffset < 8; b_aoffset++) {
      if (i_free[b_aoffset]) {
        i++;
      }
    }

    A_tmp_tmp_0 = i;
    i = 0;
    for (b_aoffset = 0; b_aoffset < 8; b_aoffset++) {
      if (i_free[b_aoffset]) {
        e_data[i] = (int8_T)(b_aoffset + 1);
        i++;
      }
    }

    A_free_size[0] = 11;
    A_free_size[1] = A_tmp_tmp_0;
    for (b_aoffset = 0; b_aoffset < A_tmp_tmp_0; b_aoffset++) {
      for (f_size_idx_0 = 0; f_size_idx_0 < 11; f_size_idx_0++) {
        A_free_data[f_size_idx_0 + 11 * b_aoffset] = A[(e_data[b_aoffset] - 1) *
          11 + f_size_idx_0];
      }
    }

    // 'wls_alloc:74' p_free = A_free\d;
    mldivide(A_free_data, A_free_size, d, p_free_data, &aoffset);

    // 'wls_alloc:76' p = zeros(m,1);
    // 'wls_alloc:78' p(i_free) = p_free;
    i = 0;

    // 'wls_alloc:84' u_opt = u + p;
    for (b_aoffset = 0; b_aoffset < 8; b_aoffset++) {
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
    for (b_aoffset = 0; b_aoffset < 8; b_aoffset++) {
      if (i_free[b_aoffset]) {
        i++;
      }
    }

    f_size_idx_0 = i;
    i = 0;
    for (b_aoffset = 0; b_aoffset < 8; b_aoffset++) {
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
      for (i = 0; i < 8; i++) {
        u[i] = u_opt[i];
      }

      // 'wls_alloc:95' d = d - A_free*p_free;
      if (A_tmp_tmp_0 == 1) {
        for (b_aoffset = 0; b_aoffset < 11; b_aoffset++) {
          A_tmp_2[b_aoffset] = 0.0F;
          for (f_size_idx_0 = 0; f_size_idx_0 < A_tmp_tmp_0; f_size_idx_0++) {
            A_tmp_2[b_aoffset] += A_free_data[11 * f_size_idx_0 + b_aoffset] *
              p_free_data[f_size_idx_0];
          }
        }
      } else if (aoffset == 1) {
        for (b_aoffset = 0; b_aoffset < 11; b_aoffset++) {
          A_tmp_2[b_aoffset] = 0.0F;
          for (f_size_idx_0 = 0; f_size_idx_0 < A_tmp_tmp_0; f_size_idx_0++) {
            A_tmp_2[b_aoffset] += A_free_data[11 * f_size_idx_0 + b_aoffset] *
              p_free_data[f_size_idx_0];
          }
        }
      } else {
        for (i = 0; i < 11; i++) {
          A_tmp_2[i] = 0.0F;
        }

        for (i = 0; i < A_tmp_tmp_0; i++) {
          b_aoffset = i * 11;
          for (f_size_idx_0 = 0; f_size_idx_0 < 11; f_size_idx_0++) {
            aoffset = b_aoffset + f_size_idx_0;
            A_tmp_2[f_size_idx_0] += A[(e_data[aoffset / 11] - 1) * 11 + aoffset
              % 11] * p_free_data[i];
          }
        }
      }

      for (b_aoffset = 0; b_aoffset < 11; b_aoffset++) {
        d[b_aoffset] -= A_tmp_2[b_aoffset];
      }

      // 'wls_alloc:97' lambda = W.*(A'*d);
      // 'wls_alloc:99' if lambda >= -eps
      for (i = 0; i < 8; i++) {
        p_free_data[i] = 0.0F;
        for (b_aoffset = 0; b_aoffset < 11; b_aoffset++) {
          p_free_data[i] += A[11 * i + b_aoffset] * d[b_aoffset];
        }

        gam_sq = W[i] * p_free_data[i];
        u_opt_0[i] = (gam_sq >= -2.22044605E-16F);
        u_opt[i] = gam_sq;
      }

      x = true;
      i = 0;
      exitg2 = false;
      while ((!exitg2) && (i < 8)) {
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
        for (b_aoffset = 0; b_aoffset < 7; b_aoffset++) {
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
      for (b_aoffset = 0; b_aoffset < 8; b_aoffset++) {
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
      for (b_aoffset = 0; b_aoffset < 8; b_aoffset++) {
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
      for (b_aoffset = 0; b_aoffset < 8; b_aoffset++) {
        if (i_free[b_aoffset] && u_opt_0[b_aoffset]) {
          dist[b_aoffset] = b_data[i];
          i++;
        }
      }

      // 'wls_alloc:129' dist(i_max) = (umax(i_max) - u(i_max)) ./ p(i_max);
      i = 0;
      for (b_aoffset = 0; b_aoffset < 8; b_aoffset++) {
        if (i_free[b_aoffset] && u_opt_data[b_aoffset]) {
          i++;
        }
      }

      f_size_idx_0 = i;
      i = 0;
      for (b_aoffset = 0; b_aoffset < 8; b_aoffset++) {
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
      for (b_aoffset = 0; b_aoffset < 8; b_aoffset++) {
        if (i_free[b_aoffset] && u_opt_data[b_aoffset]) {
          dist[b_aoffset] = b_data[i];
          i++;
        }
      }

      // 'wls_alloc:132' [alpha,i_alpha] = min(dist);
      p_0 = dist[0];
      i = 0;
      for (b_aoffset = 0; b_aoffset < 7; b_aoffset++) {
        tmp = dist[b_aoffset + 1];
        if (p_0 > tmp) {
          p_0 = tmp;
          i = b_aoffset + 1;
        }
      }

      // 'wls_alloc:134' u = u + alpha*p;
      for (b_aoffset = 0; b_aoffset < 8; b_aoffset++) {
        u[b_aoffset] += (real32_T)(p_0 * p[b_aoffset]);
      }

      // 'wls_alloc:135' d = d - A_free*alpha*p_free;
      f_size_idx_0 = 11 * A_tmp_tmp_0 - 1;
      for (b_aoffset = 0; b_aoffset <= f_size_idx_0; b_aoffset++) {
        A_free_data[b_aoffset] *= (real32_T)p_0;
      }

      if (A_tmp_tmp_0 == 1) {
        for (b_aoffset = 0; b_aoffset < 11; b_aoffset++) {
          A_tmp_2[b_aoffset] = 0.0F;
          for (f_size_idx_0 = 0; f_size_idx_0 < 1; f_size_idx_0++) {
            A_tmp_2[b_aoffset] += A_free_data[b_aoffset] * p_free_data[0];
          }
        }
      } else if (aoffset == 1) {
        for (b_aoffset = 0; b_aoffset < 11; b_aoffset++) {
          A_tmp_2[b_aoffset] = 0.0F;
          for (f_size_idx_0 = 0; f_size_idx_0 < A_tmp_tmp_0; f_size_idx_0++) {
            A_tmp_2[b_aoffset] += A_free_data[11 * f_size_idx_0 + b_aoffset] *
              p_free_data[f_size_idx_0];
          }
        }
      } else {
        for (b_aoffset = 0; b_aoffset < 11; b_aoffset++) {
          A_tmp_2[b_aoffset] = 0.0F;
        }

        for (f_size_idx_0 = 0; f_size_idx_0 < A_tmp_tmp_0; f_size_idx_0++) {
          aoffset = f_size_idx_0 * 11;
          for (b_aoffset = 0; b_aoffset < 11; b_aoffset++) {
            A_tmp_2[b_aoffset] += A_free_data[aoffset + b_aoffset] *
              p_free_data[f_size_idx_0];
          }
        }
      }

      for (b_aoffset = 0; b_aoffset < 11; b_aoffset++) {
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

// Function for MATLAB Function: '<S77>/caIndiWls'
void MatlabControllerClass::LSQFromQR_a(const real32_T A_data[], const int32_T
  A_size[2], const real32_T tau_data[], const int32_T jpvt_data[], real32_T B_8
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
      wj = B_8[b_j];
      for (loop_ub = b_j + 1; loop_ub + 1 < 14; loop_ub++) {
        wj += A_data[13 * b_j + loop_ub] * B_8[loop_ub];
      }

      wj *= tau_data[b_j];
      if (wj != 0.0F) {
        B_8[b_j] -= wj;
        for (loop_ub = b_j + 1; loop_ub + 1 < 14; loop_ub++) {
          B_8[loop_ub] -= A_data[13 * b_j + loop_ub] * wj;
        }
      }
    }
  }

  for (loop_ub = 0; loop_ub < rankA; loop_ub++) {
    Y_data[jpvt_data[loop_ub] - 1] = B_8[loop_ub];
  }

  for (loop_ub = rankA - 1; loop_ub + 1 > 0; loop_ub--) {
    Y_data[jpvt_data[loop_ub] - 1] /= A_data[13 * loop_ub + loop_ub];
    for (b_j = 0; b_j < loop_ub; b_j++) {
      Y_data[jpvt_data[b_j] - 1] -= A_data[13 * loop_ub + b_j] *
        Y_data[jpvt_data[loop_ub] - 1];
    }
  }
}

// Function for MATLAB Function: '<S77>/caIndiWls'
void MatlabControllerClass::xzlarf_f(int32_T m, int32_T n, int32_T iv0, real32_T
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

// Function for MATLAB Function: '<S77>/caIndiWls'
void MatlabControllerClass::qrsolve_p(const real32_T A_data[], const int32_T
  A_size[2], const real32_T B_6[13], real32_T Y_data[], int32_T *Y_size)
{
  real32_T b_A_data[130];
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
  real32_T B_7[13];
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
        xzlarf_f(13 - b_n, nmi - 1, yk + 1, tau_data[b_n], b_A_data, (b_n + (b_n
                   + 1) * 13) + 1, work_data);
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
    B_7[b_ix] = B_6[b_ix];
  }

  LSQFromQR_a(b_A_data, b_A_size, tau_data, jpvt_data, B_7, n, Y_data, Y_size);
}

// Function for MATLAB Function: '<S77>/caIndiWls'
void MatlabControllerClass::mldivide_i(const real32_T A_data[], const int32_T
  A_size[2], const real32_T B_5[13], real32_T Y_data[], int32_T *Y_size)
{
  if (A_size[1] == 0) {
    *Y_size = 0;
  } else {
    qrsolve_p(A_data, A_size, B_5, Y_data, Y_size);
  }
}

//
// Function for MATLAB Function: '<S77>/caIndiWls'
// function [u,W,iter] = wls_alloc(B,v,umin,umax,Wv,Wu,ud,gam,u,W,imax)
//
real32_T MatlabControllerClass::wls_alloc_f(const real32_T B_9[30], const
  real32_T v[3], const real32_T umin[10], const real32_T umax[10], const
  real32_T Wv[9], const real32_T Wu[100], const real32_T ud[10], real32_T gam,
  real32_T u[10], real32_T W[10], real32_T imax)
{
  real32_T iter;
  real32_T gam_sq;
  real32_T A[130];
  real32_T d[13];
  boolean_T i_free[10];
  real32_T A_free_data[130];
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
  real32_T A_tmp[9];
  int32_T i;
  boolean_T u_opt_data[10];
  real32_T A_tmp_0[30];
  real32_T A_tmp_1[3];
  real32_T A_tmp_2[13];
  real32_T A_0[13];
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
  for (b_aoffset = 0; b_aoffset < 9; b_aoffset++) {
    A_tmp[b_aoffset] = gam_sq * Wv[b_aoffset];
  }

  for (b_aoffset = 0; b_aoffset < 10; b_aoffset++) {
    for (f_size_idx_0 = 0; f_size_idx_0 < 3; f_size_idx_0++) {
      A_tmp_tmp = f_size_idx_0 + 3 * b_aoffset;
      A_tmp_0[A_tmp_tmp] = 0.0F;
      A_tmp_tmp_0 = 3 * b_aoffset + f_size_idx_0;
      A_tmp_0[A_tmp_tmp] = A_tmp_0[A_tmp_tmp_0] + B_9[3 * b_aoffset] *
        A_tmp[f_size_idx_0];
      A_tmp_0[A_tmp_tmp] = B_9[3 * b_aoffset + 1] * A_tmp[f_size_idx_0 + 3] +
        A_tmp_0[A_tmp_tmp_0];
      A_tmp_0[A_tmp_tmp] = B_9[3 * b_aoffset + 2] * A_tmp[f_size_idx_0 + 6] +
        A_tmp_0[A_tmp_tmp_0];
    }
  }

  for (b_aoffset = 0; b_aoffset < 10; b_aoffset++) {
    A[13 * b_aoffset] = A_tmp_0[3 * b_aoffset];
    A[1 + 13 * b_aoffset] = A_tmp_0[3 * b_aoffset + 1];
    A[2 + 13 * b_aoffset] = A_tmp_0[3 * b_aoffset + 2];
    for (f_size_idx_0 = 0; f_size_idx_0 < 10; f_size_idx_0++) {
      A[(f_size_idx_0 + 13 * b_aoffset) + 3] = Wu[10 * b_aoffset + f_size_idx_0];
    }
  }

  // 'wls_alloc:57' b = [gam_sq*Wv*v ; Wu*ud];
  // 'wls_alloc:60' d = b - A*u;
  for (b_aoffset = 0; b_aoffset < 3; b_aoffset++) {
    A_tmp_1[b_aoffset] = A_tmp[b_aoffset + 6] * v[2] + (A_tmp[b_aoffset + 3] *
      v[1] + A_tmp[b_aoffset] * v[0]);
  }

  A_tmp_2[0] = A_tmp_1[0];
  A_tmp_2[1] = A_tmp_1[1];
  A_tmp_2[2] = A_tmp_1[2];
  for (b_aoffset = 0; b_aoffset < 10; b_aoffset++) {
    u_opt[b_aoffset] = 0.0F;
    for (f_size_idx_0 = 0; f_size_idx_0 < 10; f_size_idx_0++) {
      u_opt[b_aoffset] += Wu[10 * f_size_idx_0 + b_aoffset] * ud[f_size_idx_0];
    }

    A_tmp_2[b_aoffset + 3] = u_opt[b_aoffset];
  }

  for (b_aoffset = 0; b_aoffset < 13; b_aoffset++) {
    A_0[b_aoffset] = 0.0F;
    for (f_size_idx_0 = 0; f_size_idx_0 < 10; f_size_idx_0++) {
      A_0[b_aoffset] += A[13 * f_size_idx_0 + b_aoffset] * u[f_size_idx_0];
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

    A_free_size[0] = 13;
    A_free_size[1] = A_tmp_tmp_0;
    for (b_aoffset = 0; b_aoffset < A_tmp_tmp_0; b_aoffset++) {
      for (f_size_idx_0 = 0; f_size_idx_0 < 13; f_size_idx_0++) {
        A_free_data[f_size_idx_0 + 13 * b_aoffset] = A[(e_data[b_aoffset] - 1) *
          13 + f_size_idx_0];
      }
    }

    // 'wls_alloc:74' p_free = A_free\d;
    mldivide_i(A_free_data, A_free_size, d, p_free_data, &aoffset);

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
      for (i = 0; i < 10; i++) {
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

// Model step function
void MatlabControllerClass::step()
{
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
  real32_T I_b[9];
  real32_T clu[10];
  real32_T force_dir[30];
  real32_T c_XYZ[30];
  real32_T pos[30];
  real32_T x[9];
  int32_T p2;
  int32_T p3;
  int32_T itmp;
  real32_T k2[6];
  real32_T k3[6];
  real32_T k4[6];
  real32_T y_0[6];
  real32_T umin[10];
  real32_T umax[10];
  real32_T W_u[100];
  real32_T wp_approach_out[15];
  boolean_T is_approach;
  int32_T wp_idx_app_2;
  int32_T stage_app_2;
  int32_T stage_2;
  int32_T wp_idx_2;
  int32_T stage_tmp;
  int32_T wp_idx_tmp;
  real32_T dir_next_wp[3];
  real32_T flight_dir[3];
  int32_T c_stage_app;
  static const int8_T g[3] = { 3, 4, 0 };

  static const int8_T h[3] = { 4, 0, 1 };

  real32_T a_Kb_yz[2];
  real32_T g_b_yz[2];
  real32_T Phi_i;
  real32_T G11_1[8];
  real32_T G11_2[8];
  real32_T umin_0[8];
  real32_T umax_0[8];
  real32_T W_u_0[64];
  real32_T varargin_2[8];
  real32_T z1[8];
  real32_T rtb_Product4[60];
  real32_T rtb_Product3[60];
  real32_T rtb_Product2[60];
  real32_T rtb_y_k[3];
  real32_T rtb_y_ai[3];
  real32_T rtb_y_pd[3];
  real32_T rtb_y_n_g[6];
  real32_T rtb_y_n[6];
  real32_T rtb_Sum2_ci[3];
  real32_T rtb_u[10];
  real32_T rtb_M_bg[9];
  real32_T rtb_y_j5[30];
  real32_T rtb_Sum2_ny[3];
  real32_T rtb_Sum2_g;
  real32_T rtb_Sum2_mn;
  real32_T rtb_Sum2_p;
  real32_T rtb_Sum2_dt;
  real32_T rtb_Sum2_ec;
  real32_T rtb_Sum2_po;
  real32_T rtb_Sum2_n;
  real32_T rtb_Sum2_o3;
  real32_T rtb_Sum2_lb;
  real32_T rtb_Sum2_b;
  real32_T rtb_Sum2_cx;
  real32_T rtb_Sum2_dx[10];
  real32_T rtb_Sum2_ha[10];
  real32_T rtb_Delta_u[10];
  real32_T rtb_y[11];
  real32_T rtb_DiscreteTimeIntegrator_l[10];
  boolean_T rtb_Compare_b;
  uint8_T rtb_Compare;
  boolean_T rtb_Compare_j;
  real32_T expl_temp[3];
  int32_T i;
  real32_T G11_1_0[24];
  real32_T tmp[8];
  real32_T tmp_0[2];
  real32_T dir_next_wp_0[3];
  real32_T rtb_Sum2_k[3];
  real32_T tmp_1[30];
  real32_T I_b_0[30];
  uint16_T p3_0[3];
  int32_T rtb_y_g_size[2];
  int32_T rtb_y_g_size_0[2];
  int32_T rtb_y_g_size_1[2];
  int32_T rtb_y_g_size_2[2];
  real32_T y_m[3];
  real32_T p_match_2[3];
  real32_T q_bg_unsigned_idx_0;
  real32_T q_bg_unsigned_idx_1;
  real32_T q_bg_unsigned_idx_2;
  real32_T q_bg_unsigned_idx_3;
  real32_T rtb_omega2_e_0;

  // RelationalOperator: '<S2>/Compare' incorporates:
  //   Constant: '<S2>/Constant'
  //   Inport: '<Root>/cmd'

  rtb_Compare = (rtU.cmd.RC_pwm[7] < 1600.0F);

  // MATLAB Function: '<Root>/Remove velocity' incorporates:
  //   Inport: '<Root>/cmd'

  // :  y = u(1:3,:);
  for (stage_app_2 = 0; stage_app_2 < 10; stage_app_2++) {
    i = stage_app_2 << 2;
    rtb_y_j5[3 * stage_app_2] = rtU.cmd.waypoints[i];
    rtb_y_j5[1 + 3 * stage_app_2] = rtU.cmd.waypoints[i + 1];
    rtb_y_j5[2 + 3 * stage_app_2] = rtU.cmd.waypoints[i + 2];
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
    q1_q3 = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.29246971E-26F;
    q1_q3 = t * t;
  }

  absxk = std::abs(rtU.measure.q_bg[1]);
  if (absxk > scale) {
    t = scale / absxk;
    q1_q3 = q1_q3 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q1_q3 += t * t;
  }

  absxk = std::abs(rtU.measure.q_bg[2]);
  if (absxk > scale) {
    t = scale / absxk;
    q1_q3 = q1_q3 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q1_q3 += t * t;
  }

  absxk = std::abs(rtU.measure.q_bg[3]);
  if (absxk > scale) {
    t = scale / absxk;
    q1_q3 = q1_q3 * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    q1_q3 += t * t;
  }

  q1_q3 = scale * std::sqrt(q1_q3);
  if (2.22044605E-16F < q1_q3) {
    scale = q1_q3;
  } else {
    scale = 2.22044605E-16F;
  }

  q_bg_unsigned_idx_0 = rtU.measure.q_bg[0] / scale;
  q_bg_unsigned_idx_1 = rtU.measure.q_bg[1] / scale;
  q_bg_unsigned_idx_2 = rtU.measure.q_bg[2] / scale;
  q_bg_unsigned_idx_3 = rtU.measure.q_bg[3] / scale;

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
  q2_q3 = q_bg_unsigned_idx_2 * q_bg_unsigned_idx_3;

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
  rtb_M_bg[7] = (q2_q3 + absxk) * 2.0F;
  rtb_M_bg[2] = (q1_q3 + t) * 2.0F;
  rtb_M_bg[5] = (q2_q3 - absxk) * 2.0F;
  rtb_M_bg[8] = (q0_q0 - q2_q2) + scale;

  // End of MATLAB Function: '<Root>/Quaternions to Rotation Matrix'

  // RelationalOperator: '<S3>/Compare' incorporates:
  //   Constant: '<S3>/Constant'
  //   Inport: '<Root>/cmd'

  rtb_Compare_b = (rtU.cmd.RC_pwm[7] < 1400.0F);

  // Outputs for Enabled SubSystem: '<Root>/LindiPlane Autopilot' incorporates:
  //   EnablePort: '<S5>/Enable'

  // RelationalOperator: '<S4>/Compare' incorporates:
  //   DiscreteIntegrator: '<S42>/Discrete-Time Integrator y'
  //   DiscreteIntegrator: '<S42>/Discrete-Time Integrator y_dt'
  //   Inport: '<Root>/measure'
  //   Logic: '<S14>/Logical Operator'
  //   Product: '<S42>/Product1'
  //   Product: '<S42>/Product2'
  //   Product: '<S42>/omega^2'
  //   Sum: '<S42>/Sum2'
  //   Sum: '<S42>/Sum3'

  if (!rtb_Compare_b) {
    if (!rtDW.LindiPlaneAutopilot_MODE) {
      // InitializeConditions for DiscreteIntegrator: '<S43>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_IC_LOAD = 1U;

      // InitializeConditions for DiscreteIntegrator: '<S39>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_IC_LO_a = 1U;

      // InitializeConditions for DiscreteIntegrator: '<S40>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_DSTAT_n[0] = 0.0F;
      rtDW.DiscreteTimeIntegratory_DSTAT_n[1] = 0.0F;
      rtDW.DiscreteTimeIntegratory_DSTAT_n[2] = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S41>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_IC_LO_e = 1U;

      // InitializeConditions for DiscreteIntegrator: '<S42>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_IC_LO_i = 1U;

      // InitializeConditions for DiscreteIntegrator: '<S38>/Discrete-Time Integrator' 
      rtDW.DiscreteTimeIntegrator_DSTATE = 0.0F;

      // InitializeConditions for UnitDelay: '<S38>/Unit Delay'
      rtDW.UnitDelay_DSTATE = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S37>/Discrete-Time Integrator' 
      rtDW.DiscreteTimeIntegrator_DSTATE_c = 0.0F;

      // InitializeConditions for UnitDelay: '<S37>/Unit Delay'
      rtDW.UnitDelay_DSTATE_a = 0.0F;
      for (i = 0; i < 6; i++) {
        // InitializeConditions for Delay: '<S38>/Delay'
        rtDW.Delay_DSTATE[i] = 0.0F;

        // InitializeConditions for Delay: '<S37>/Delay'
        rtDW.Delay_DSTATE_h[i] = 0.0F;
      }

      // InitializeConditions for UnitDelay: '<S21>/Unit Delay'
      rtDW.UnitDelay_DSTATE_f = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S61>/Discrete-Time Integrator' 
      rtDW.DiscreteTimeIntegrator_IC_LOADI = 1U;

      // InitializeConditions for DiscreteIntegrator: '<S63>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_IC_LO_m = 1U;

      // InitializeConditions for DiscreteIntegrator: '<S62>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_IC_LO_j = 1U;

      // InitializeConditions for DiscreteIntegrator: '<S66>/Discrete-Time Integrator' 
      rtDW.DiscreteTimeIntegrator_DSTATE_h = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S67>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_DSTAT_h = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S84>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_DSTAT_p = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S84>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_DSTA = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S85>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_IC_LO_b = 1U;

      // InitializeConditions for DiscreteIntegrator: '<S86>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_IC_L_an = 1U;

      // InitializeConditions for DiscreteIntegrator: '<S87>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_IC_LO_c = 1U;

      // InitializeConditions for DiscreteIntegrator: '<S92>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_DSTA_hc = 0.0F;

      // InitializeConditions for UnitDelay: '<S22>/Unit Delay'
      rtDW.UnitDelay_DSTATE_fx = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S100>/Discrete-Time Integrator' 
      rtDW.DiscreteTimeIntegrator_IC_LOA_g = 1U;

      // InitializeConditions for DiscreteIntegrator: '<S102>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_IC_L_jp = 1U;

      // InitializeConditions for DiscreteIntegrator: '<S101>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_IC_L_bc = 1U;
      for (i = 0; i < 10; i++) {
        // InitializeConditions for UnitDelay: '<S69>/Unit Delay1'
        rtDW.UnitDelay1_DSTATE[i] = 0.0F;

        // InitializeConditions for UnitDelay: '<S69>/Unit Delay2'
        rtDW.UnitDelay2_DSTATE[i] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S74>/Discrete-Time Integrator y' 
        rtDW.DiscreteTimeIntegratory_DSTAT_l[i] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S73>/Discrete-Time Integrator y' 
        rtDW.DiscreteTimeIntegratory_DSTA_j2[i] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S79>/Discrete-Time Integrator1' 
        rtDW.DiscreteTimeIntegrator1_DSTATE[i] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S79>/Discrete-Time Integrator' 
        rtDW.DiscreteTimeIntegrator_DSTATE_l[i] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S80>/Discrete-Time Integrator1' 
        rtDW.DiscreteTimeIntegrator1_DSTAT_j[i] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S80>/Discrete-Time Integrator' 
        rtDW.DiscreteTimeIntegrator_DSTATE_g[i] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S74>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_DS_o[i] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S73>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_DS_n[i] = 0.0F;
      }

      // InitializeConditions for DiscreteIntegrator: '<S39>/Discrete-Time Integrator y_dt' 
      for (i = 0; i < 9; i++) {
        rtDW.DiscreteTimeIntegratory_dt_D_ni[i] = 0.0F;
      }

      // End of InitializeConditions for DiscreteIntegrator: '<S39>/Discrete-Time Integrator y_dt' 

      // InitializeConditions for DiscreteIntegrator: '<S42>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_DS_e[0] = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S41>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_DS_p[0] = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S40>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_DS_i[0] = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S42>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_DS_e[1] = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S41>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_DS_p[1] = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S40>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_DS_i[1] = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S42>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_DS_e[2] = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S41>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_DS_p[2] = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S40>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_DS_i[2] = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S101>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_D_oo = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S102>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_DS_j = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S92>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_DS_g = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S93>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_DSTAT_g = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S93>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_D_ir = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S87>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_DS_l = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S86>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_DS_f = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S85>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_D_pv = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S67>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_DS_b = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S68>/Discrete-Time Integrator y' 
      rtDW.DiscreteTimeIntegratory_DSTAT_d = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S68>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_D_lv = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S62>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_D_eo = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S63>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_D_nr = 0.0F;

      // InitializeConditions for DiscreteIntegrator: '<S43>/Discrete-Time Integrator y_dt' 
      rtDW.DiscreteTimeIntegratory_dt_D_gf = 0.0F;
      rtDW.LindiPlaneAutopilot_MODE = true;
    }

    // MATLAB Function: '<S76>/indiCeFlapFix'
    // :  [G10,G20,G30] = indiCeFlapFix( cef, ceb );
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
      x[stage_app_2] = I_b[stage_app_2];
    }

    i = 0;
    p2 = 3;
    p3 = 6;
    q0_q0 = std::abs(rtP.lindi.ceb.ixx);
    q1_q1 = std::abs(-rtP.lindi.ceb.ixy);
    q2_q2 = std::abs(-rtP.lindi.ceb.ixz);
    if ((q1_q1 > q0_q0) && (q1_q1 > q2_q2)) {
      i = 3;
      p2 = 0;
      x[0] = -rtP.lindi.ceb.ixy;
      x[1] = rtP.lindi.ceb.ixx;
      x[3] = rtP.lindi.ceb.iyy;
      x[4] = -rtP.lindi.ceb.ixy;
      x[6] = -rtP.lindi.ceb.iyz;
      x[7] = -rtP.lindi.ceb.ixz;
    } else {
      if (q2_q2 > q0_q0) {
        i = 6;
        p3 = 0;
        x[0] = -rtP.lindi.ceb.ixz;
        x[2] = rtP.lindi.ceb.ixx;
        x[3] = -rtP.lindi.ceb.iyz;
        x[5] = -rtP.lindi.ceb.ixy;
        x[6] = rtP.lindi.ceb.izz;
        x[8] = -rtP.lindi.ceb.ixz;
      }
    }

    q_bg_unsigned_idx_1 = x[1] / x[0];
    x[1] = q_bg_unsigned_idx_1;
    q1_q1 = x[2] / x[0];
    x[2] = q1_q1;
    x[4] -= q_bg_unsigned_idx_1 * x[3];
    x[5] -= q1_q1 * x[3];
    x[7] -= q_bg_unsigned_idx_1 * x[6];
    x[8] -= q1_q1 * x[6];
    if (std::abs(x[5]) > std::abs(x[4])) {
      itmp = p2;
      p2 = p3;
      p3 = itmp;
      x[1] = q1_q1;
      x[2] = q_bg_unsigned_idx_1;
      q0_q0 = x[4];
      x[4] = x[5];
      x[5] = q0_q0;
      q0_q0 = x[7];
      x[7] = x[8];
      x[8] = q0_q0;
    }

    q_bg_unsigned_idx_1 = x[5] / x[4];
    x[8] -= q_bg_unsigned_idx_1 * x[7];
    q0_q0 = (q_bg_unsigned_idx_1 * x[1] - x[2]) / x[8];
    q1_q1 = -(x[7] * q0_q0 + x[1]) / x[4];
    I_b[i] = ((1.0F - x[3] * q1_q1) - x[6] * q0_q0) / x[0];
    I_b[i + 1] = q1_q1;
    I_b[i + 2] = q0_q0;
    q0_q0 = -q_bg_unsigned_idx_1 / x[8];
    q1_q1 = (1.0F - x[7] * q0_q0) / x[4];
    I_b[p2] = -(x[3] * q1_q1 + x[6] * q0_q0) / x[0];
    I_b[p2 + 1] = q1_q1;
    I_b[p2 + 2] = q0_q0;
    q0_q0 = 1.0F / x[8];
    q1_q1 = -x[7] * q0_q0 / x[4];
    I_b[p3] = -(x[3] * q1_q1 + x[6] * q0_q0) / x[0];
    I_b[p3 + 1] = q1_q1;
    I_b[p3 + 2] = q0_q0;

    // 'indiCeFlapFix:12' clu = cef.cla .* cef.dadf .* cef.dfdu;
    // 'indiCeFlapFix:15' force_dir = [ zeros(size(cef.rotx)); sin(cef.rotx); -cos(cef.rotx) ]; 
    for (itmp = 0; itmp < 10; itmp++) {
      clu[itmp] = rtP.lindi.cef.cla[itmp] * rtP.lindi.cef.dadf[itmp] *
        rtP.lindi.cef.dfdu[itmp];
      q0_q0 = std::sin(rtP.lindi.cef.rotx[itmp]);
      force_dir[3 * itmp] = 0.0F;
      force_dir[1 + 3 * itmp] = q0_q0;
      force_dir[2 + 3 * itmp] = -std::cos(rtP.lindi.cef.rotx[itmp]);
      rtb_Delta_u[itmp] = q0_q0;
    }

    // 'indiCeFlapFix:16' c_XYZ = zeros(size(force_dir),class(clu));
    // 'indiCeFlapFix:17' for i = 1:size(c_XYZ,1)
    for (i = 0; i < 3; i++) {
      // 'indiCeFlapFix:18' c_XYZ(i,:) = force_dir(i,:) .* clu .* cef.s;
      for (stage_app_2 = 0; stage_app_2 < 10; stage_app_2++) {
        c_XYZ[i + 3 * stage_app_2] = force_dir[3 * stage_app_2 + i] *
          clu[stage_app_2] * rtP.lindi.cef.s[stage_app_2];
      }
    }

    // 'indiCeFlapFix:21' pos = [ cef.x; cef.y; cef.z ];
    // 'indiCeFlapFix:23' G10 = [ ...
    // 'indiCeFlapFix:24'         inv_I_b * cross( pos, c_XYZ ); ...
    // 'indiCeFlapFix:25'         c_XYZ / ceb.m ...
    // 'indiCeFlapFix:26'     ];
    // 'indiCeFlapFix:28' G20 = [ ...
    // 'indiCeFlapFix:29'         zeros(size(G10),class(G10)) ...
    // 'indiCeFlapFix:30'     ];
    // 'indiCeFlapFix:32' G30 = [ ...
    // 'indiCeFlapFix:33'         inv_I_b * cross( pos, force_dir ) * diag( cef.m .* cef.dfdu .* cef.xm ); ... 
    // 'indiCeFlapFix:34'         zeros(3,size(G10,2),class(G10)) ...
    // 'indiCeFlapFix:35'     ];
    for (stage_app_2 = 0; stage_app_2 < 10; stage_app_2++) {
      pos[3 * stage_app_2] = rtP.lindi.cef.x[stage_app_2];
      pos[1 + 3 * stage_app_2] = rtP.lindi.cef.y[stage_app_2];
      pos[2 + 3 * stage_app_2] = rtP.lindi.cef.z[stage_app_2];
      clu[stage_app_2] = rtP.lindi.cef.m[stage_app_2] *
        rtP.lindi.cef.dfdu[stage_app_2] * rtP.lindi.cef.xm[stage_app_2];
    }

    memset(&W_u[0], 0, 100U * sizeof(real32_T));
    for (i = 0; i < 10; i++) {
      W_u[i + 10 * i] = clu[i];
    }

    cross(pos, force_dir, tmp_1);
    for (stage_app_2 = 0; stage_app_2 < 3; stage_app_2++) {
      for (i = 0; i < 10; i++) {
        p3 = stage_app_2 + 3 * i;
        force_dir[p3] = 0.0F;
        p2 = 3 * i + stage_app_2;
        force_dir[p3] = force_dir[p2] + tmp_1[3 * i] * I_b[stage_app_2];
        force_dir[p3] = tmp_1[3 * i + 1] * I_b[stage_app_2 + 3] + force_dir[p2];
        force_dir[p3] = tmp_1[3 * i + 2] * I_b[stage_app_2 + 6] + force_dir[p2];
      }

      for (i = 0; i < 10; i++) {
        p3 = stage_app_2 + 3 * i;
        I_b_0[p3] = 0.0F;
        for (p2 = 0; p2 < 10; p2++) {
          I_b_0[p3] = force_dir[3 * p2 + stage_app_2] * W_u[10 * i + p2] +
            I_b_0[3 * i + stage_app_2];
        }
      }
    }

    // Product: '<S76>/Product4' incorporates:
    //   Constant: '<S76>/Constant'
    //   MATLAB Function: '<S76>/indiCeFlapFix'

    for (stage_app_2 = 0; stage_app_2 < 10; stage_app_2++) {
      rtb_Product4[6 * stage_app_2] = I_b_0[3 * stage_app_2] *
        rtP.lindi.ceb.scale;
      rtb_Product4[3 + 6 * stage_app_2] = 0.0F;
      rtb_Product4[1 + 6 * stage_app_2] = I_b_0[3 * stage_app_2 + 1] *
        rtP.lindi.ceb.scale;
      rtb_Product4[4 + 6 * stage_app_2] = 0.0F;
      rtb_Product4[2 + 6 * stage_app_2] = I_b_0[3 * stage_app_2 + 2] *
        rtP.lindi.ceb.scale;
      rtb_Product4[5 + 6 * stage_app_2] = 0.0F;
    }

    // End of Product: '<S76>/Product4'

    // DiscreteIntegrator: '<S43>/Discrete-Time Integrator y' incorporates:
    //   Inport: '<Root>/measure'

    if (rtDW.DiscreteTimeIntegratory_IC_LOAD != 0) {
      rtDW.DiscreteTimeIntegratory_DSTATE = rtU.measure.airspeed;
    }

    // MinMax: '<S15>/Max' incorporates:
    //   Constant: '<S15>/Constant'
    //   DiscreteIntegrator: '<S43>/Discrete-Time Integrator y'

    if (rtDW.DiscreteTimeIntegratory_DSTATE > rtP.lindi.aspd.min) {
      q0_q0 = rtDW.DiscreteTimeIntegratory_DSTATE;
    } else {
      q0_q0 = rtP.lindi.aspd.min;
    }

    // End of MinMax: '<S15>/Max'

    // Product: '<S76>/Product5' incorporates:
    //   Math: '<S76>/Square'

    q1_q1 = q0_q0 * q0_q0 * 0.6125F;

    // Product: '<S76>/Product3' incorporates:
    //   MATLAB Function: '<S76>/indiCeFlapFix'

    memset(&rtb_Product3[0], 0, 60U * sizeof(real32_T));

    // MATLAB Function: '<S76>/indiCeFlapFix'
    cross(pos, c_XYZ, tmp_1);
    for (stage_app_2 = 0; stage_app_2 < 10; stage_app_2++) {
      for (i = 0; i < 3; i++) {
        p3 = i + 3 * stage_app_2;
        force_dir[p3] = 0.0F;
        p2 = 3 * stage_app_2 + i;
        force_dir[p3] = force_dir[p2] + tmp_1[3 * stage_app_2] * I_b[i];
        force_dir[p3] = tmp_1[3 * stage_app_2 + 1] * I_b[i + 3] + force_dir[p2];
        force_dir[p3] = tmp_1[3 * stage_app_2 + 2] * I_b[i + 6] + force_dir[p2];
      }
    }

    // Product: '<S76>/Product2' incorporates:
    //   Constant: '<S76>/Constant'
    //   MATLAB Function: '<S76>/indiCeFlapFix'
    //   Product: '<S76>/Product'

    for (stage_app_2 = 0; stage_app_2 < 10; stage_app_2++) {
      rtb_Product2[6 * stage_app_2] = force_dir[3 * stage_app_2] * q1_q1 *
        rtP.lindi.ceb.scale;
      rtb_Product2[3 + 6 * stage_app_2] = c_XYZ[3 * stage_app_2] /
        rtP.lindi.ceb.m * q1_q1 * rtP.lindi.ceb.scale;

      // MATLAB Function: '<S76>/indiCeFlapFix' incorporates:
      //   Constant: '<S76>/Constant'
      //   Product: '<S76>/Product'

      i = 3 * stage_app_2 + 1;
      rtb_Product2[1 + 6 * stage_app_2] = force_dir[i] * q1_q1 *
        rtP.lindi.ceb.scale;
      rtb_Product2[4 + 6 * stage_app_2] = c_XYZ[i] / rtP.lindi.ceb.m * q1_q1 *
        rtP.lindi.ceb.scale;

      // MATLAB Function: '<S76>/indiCeFlapFix' incorporates:
      //   Constant: '<S76>/Constant'
      //   Product: '<S76>/Product'

      i = 3 * stage_app_2 + 2;
      rtb_Product2[2 + 6 * stage_app_2] = force_dir[i] * q1_q1 *
        rtP.lindi.ceb.scale;
      rtb_Product2[5 + 6 * stage_app_2] = c_XYZ[i] / rtP.lindi.ceb.m * q1_q1 *
        rtP.lindi.ceb.scale;
    }

    // End of Product: '<S76>/Product2'

    // DiscreteIntegrator: '<S39>/Discrete-Time Integrator y'
    if (rtDW.DiscreteTimeIntegratory_IC_LO_a != 0) {
      for (i = 0; i < 9; i++) {
        rtDW.DiscreteTimeIntegratory_DSTAT_b[i] = rtb_M_bg[i];
      }
    }

    // RelationalOperator: '<S11>/Compare' incorporates:
    //   Constant: '<S11>/Constant'

    rtb_Compare_j = (rtb_Compare == 0);

    // DiscreteIntegrator: '<S40>/Discrete-Time Integrator y'
    rtb_y_k[0] = rtDW.DiscreteTimeIntegratory_DSTAT_n[0];
    rtb_y_k[1] = rtDW.DiscreteTimeIntegratory_DSTAT_n[1];
    rtb_y_k[2] = rtDW.DiscreteTimeIntegratory_DSTAT_n[2];

    // DiscreteIntegrator: '<S41>/Discrete-Time Integrator y' incorporates:
    //   Inport: '<Root>/measure'

    if (rtDW.DiscreteTimeIntegratory_IC_LO_e != 0) {
      rtDW.DiscreteTimeIntegratory_DSTA_n2[0] = rtU.measure.s_Kg[0];
      rtDW.DiscreteTimeIntegratory_DSTA_n2[1] = rtU.measure.s_Kg[1];
      rtDW.DiscreteTimeIntegratory_DSTA_n2[2] = rtU.measure.s_Kg[2];
    }

    rtb_y_pd[0] = rtDW.DiscreteTimeIntegratory_DSTA_n2[0];
    rtb_y_pd[1] = rtDW.DiscreteTimeIntegratory_DSTA_n2[1];
    rtb_y_pd[2] = rtDW.DiscreteTimeIntegratory_DSTA_n2[2];

    // DiscreteIntegrator: '<S42>/Discrete-Time Integrator y' incorporates:
    //   Inport: '<Root>/measure'

    if (rtDW.DiscreteTimeIntegratory_IC_LO_i != 0) {
      rtDW.DiscreteTimeIntegratory_DSTAT_j[0] = rtU.measure.V_Kg[0];
      rtDW.DiscreteTimeIntegratory_DSTAT_j[1] = rtU.measure.V_Kg[1];
      rtDW.DiscreteTimeIntegratory_DSTAT_j[2] = rtU.measure.V_Kg[2];
    }

    rtb_y_ai[0] = rtDW.DiscreteTimeIntegratory_DSTAT_j[0];
    rtb_y_ai[1] = rtDW.DiscreteTimeIntegratory_DSTAT_j[1];
    rtb_y_ai[2] = rtDW.DiscreteTimeIntegratory_DSTAT_j[2];

    // Outputs for Enabled SubSystem: '<S5>/NDI Position Controller' incorporates:
    //   EnablePort: '<S16>/Enable'

    // Outputs for Enabled SubSystem: '<S5>/Flight Path Smoothing' incorporates:
    //   EnablePort: '<S13>/Enable'

    // Outputs for Enabled SubSystem: '<S5>/Waypoint Navigation' incorporates:
    //   EnablePort: '<S25>/Enable'

    if (rtb_Compare_j) {
      // DataTypeConversion: '<S25>/Data Type Conversion2' incorporates:
      //   Inport: '<Root>/cmd'

      p3 = rtU.cmd.num_waypoints;

      // MATLAB Function: '<S25>/Look Ahead' incorporates:
      //   Constant: '<S25>/Constant5'

      LookAhead(rtb_y_pd, rtb_y_ai, rtb_y_k, rtP.lindi.psc.rm.T + 2.0F /
                rtP.lindi.atc.rm.rfreq, rtb_Sum2_ci);

      // MATLAB Function: '<S25>/WpNav Matching' incorporates:
      //   Constant: '<S25>/Constant1'
      //   Constant: '<S25>/Constant2'
      //   DataTypeConversion: '<S25>/Data Type Conversion2'
      //   DiscreteIntegrator: '<S41>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S42>/Discrete-Time Integrator y'
      //   Inport: '<Root>/cmd'
      //   UnitDelay: '<S25>/Unit Delay'
      //   UnitDelay: '<S25>/Unit Delay1'
      //   UnitDelay: '<S25>/Unit Delay2'
      //   UnitDelay: '<S25>/Unit Delay3'
      //   UnitDelay: '<S25>/Unit Delay4'
      //   UnitDelay: '<S25>/Unit Delay5'

      c_stage_app = rtDW.UnitDelay3_DSTATE;
      itmp = rtDW.UnitDelay2_DSTATE_i;
      is_approach = rtDW.UnitDelay5_DSTATE;
      p2 = rtDW.UnitDelay1_DSTATE_g;
      i = rtDW.UnitDelay_DSTATE_b;

      // :  a = zeros(3,1,superiorfloat(waypoints));
      t = 0.0F;
      q1_q2 = 0.0F;
      absxk = 0.0F;

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
        itmp = rtDW.UnitDelay2_DSTATE_i;
        c_stage_app = rtDW.UnitDelay3_DSTATE;
        wpnavMatch(rtDW.UnitDelay4_DSTATE, rtP.lindi.psc.rm.wprad, &itmp,
                   &c_stage_app, rtDW.DiscreteTimeIntegratory_DSTA_n2,
                   rtb_Sum2_ny, &rtDW.t_g, &q1_q1);

        // :  [p_match_2,wp_idx_app_2,stage_app_2,t_2,d_2] = wpnavMatch(wp_approach_out,wp_radius,wp_idx_app,stage_app,p_ahead); 
        wp_idx_app_2 = itmp;
        stage_app_2 = c_stage_app;
        wpnavMatch(rtDW.UnitDelay4_DSTATE, rtP.lindi.psc.rm.wprad, &wp_idx_app_2,
                   &stage_app_2, rtb_Sum2_ci, p_match_2, &q1_q1, &q2_q2);

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
        rtb_y_g_size_2[0] = 3;
        rtb_y_g_size_2[1] = p3;
        for (stage_app_2 = 0; stage_app_2 < p3; stage_app_2++) {
          c_XYZ[3 * stage_app_2] = rtb_y_j5[3 * stage_app_2];
          c_XYZ[1 + 3 * stage_app_2] = rtb_y_j5[3 * stage_app_2 + 1];
          c_XYZ[2 + 3 * stage_app_2] = rtb_y_j5[3 * stage_app_2 + 2];
        }

        wpnavMatch_p(c_XYZ, rtb_y_g_size_2, rtP.lindi.psc.rm.wprad, &wp_idx_tmp,
                     &stage_tmp, rtDW.DiscreteTimeIntegratory_DSTA_n2,
                     rtb_Sum2_ny, &rtDW.t_g, &q1_q1);

        // :  [p_match_2,wp_idx_2,stage_2,t_2,d_2] = wpnavMatch(waypoints(:,1:num_wp),wp_radius,wp_idx,stage_,p_ahead); 
        wp_idx_2 = rtDW.UnitDelay_DSTATE_b;
        stage_2 = rtDW.UnitDelay1_DSTATE_g;
        rtb_y_g_size_1[0] = 3;
        rtb_y_g_size_1[1] = p3;
        for (stage_app_2 = 0; stage_app_2 < p3; stage_app_2++) {
          c_XYZ[3 * stage_app_2] = rtb_y_j5[3 * stage_app_2];
          c_XYZ[1 + 3 * stage_app_2] = rtb_y_j5[3 * stage_app_2 + 1];
          c_XYZ[2 + 3 * stage_app_2] = rtb_y_j5[3 * stage_app_2 + 2];
        }

        wpnavMatch_p(c_XYZ, rtb_y_g_size_1, rtP.lindi.psc.rm.wprad, &wp_idx_2,
                     &stage_2, rtb_Sum2_ci, p_match_2, &q1_q1, &q2_q2);

        // :  stage_app_2 = stage_app;
        stage_app_2 = rtDW.UnitDelay3_DSTATE;

        // :  wp_idx_app_2 = wp_idx_app;
        wp_idx_app_2 = rtDW.UnitDelay2_DSTATE_i;
      }

      // :  e_pos = norm(p_match-p,2);
      // :  if e_pos > e_pos_max
      rtb_Sum2_k[0] = rtb_Sum2_ny[0] - rtDW.DiscreteTimeIntegratory_DSTA_n2[0];
      rtb_Sum2_k[1] = rtb_Sum2_ny[1] - rtDW.DiscreteTimeIntegratory_DSTA_n2[1];
      rtb_Sum2_k[2] = rtb_Sum2_ny[2] - rtDW.DiscreteTimeIntegratory_DSTA_n2[2];
      if (xnrm2_f(rtb_Sum2_k) > rtP.lindi.psc.rm.eposmax) {
        // :  dir_next_wp = waypoints(:,wp_idx) - p;
        stage_app_2 = (rtDW.UnitDelay_DSTATE_b - 1) * 3;
        dir_next_wp[0] = rtb_y_j5[stage_app_2] -
          rtDW.DiscreteTimeIntegratory_DSTA_n2[0];
        scale = rtb_y_j5[stage_app_2 + 1];
        dir_next_wp[1] = scale - rtDW.DiscreteTimeIntegratory_DSTA_n2[1];
        q0_q3 = rtb_y_j5[stage_app_2 + 2];
        dir_next_wp[2] = q0_q3 - rtDW.DiscreteTimeIntegratory_DSTA_n2[2];

        // :  dist_next_wp = norm( dir_next_wp );
        q1_q1 = xnrm2_f(dir_next_wp);

        // :  flight_dir = divideFinite( V_Kg, norm(V_Kg,2) );
        q1_q3 = xnrm2_f(rtDW.DiscreteTimeIntegratory_DSTAT_j);

        // 'divideFinite:29' if numel(B)>1
        // 'divideFinite:31' else
        // 'divideFinite:32' if abs(B)<eps
        if (std::abs(q1_q3) < 2.22044605E-16F) {
          // 'divideFinite:33' B(:) = eps;
          q1_q3 = 2.22044605E-16F;
        }

        // 'divideFinite:36' C = A ./ B;
        // :  angle_next_wp = acosReal( divideFinite( dot( dir_next_wp, flight_dir ), norm(dir_next_wp,2)*norm(flight_dir,2) ) ); 
        q2_q3 = rtDW.DiscreteTimeIntegratory_DSTAT_j[0] / q1_q3;
        q2_q2 = dir_next_wp[0] * q2_q3;
        flight_dir[0] = q2_q3;
        q2_q3 = rtDW.DiscreteTimeIntegratory_DSTAT_j[1] / q1_q3;
        q2_q2 += dir_next_wp[1] * q2_q3;
        flight_dir[1] = q2_q3;
        q2_q3 = rtDW.DiscreteTimeIntegratory_DSTAT_j[2] / q1_q3;
        q2_q2 += dir_next_wp[2] * q2_q3;
        flight_dir[2] = q2_q3;
        q_bg_unsigned_idx_3 = q1_q1 * xnrm2_f(flight_dir);

        // 'divideFinite:29' if numel(B)>1
        // 'divideFinite:31' else
        // 'divideFinite:32' if abs(B)<eps
        if (std::abs(q_bg_unsigned_idx_3) < 2.22044605E-16F) {
          // 'divideFinite:33' B(:) = eps;
          q_bg_unsigned_idx_3 = 2.22044605E-16F;
        }

        // 'divideFinite:36' C = A ./ B;
        Phi_i = q2_q2 / q_bg_unsigned_idx_3;

        // 'acosReal:28' if numel(y) > 1
        // 'acosReal:31' else
        // 'acosReal:32' y = max(-1,min(1,y));
        if (1.0F <= Phi_i) {
          Phi_i = 1.0F;
        }

        // 'acosReal:34' y = acos(y);
        // :  if angle_next_wp < 0.5 && dist_next_wp < wp_radius
        if (-1.0F >= Phi_i) {
          Phi_i = -1.0F;
        }

        if ((std::acos(Phi_i) < 0.5F) && (q1_q1 < rtP.lindi.psc.rm.wprad)) {
          // :  dist_app_wp = 0.5 * dist_next_wp;
          q1_q1 *= 0.5F;
        } else {
          // :  else
          // :  dist_app_wp = 1.2*wp_radius;
          q1_q1 = 1.2F * rtP.lindi.psc.rm.wprad;
        }

        // :  wp_approach_out(:,1) = p - flight_dir * 1 * dist_app_wp;
        // :  wp_approach_out(:,2) = p + flight_dir * 1 * dist_app_wp;
        // :  wp_approach_out(:,3) = waypoints(:,wp_idx);
        q2_q2 = flight_dir[0] * q1_q1;
        wp_approach_out[0] = rtDW.DiscreteTimeIntegratory_DSTA_n2[0] - q2_q2;
        wp_approach_out[3] = rtDW.DiscreteTimeIntegratory_DSTA_n2[0] + q2_q2;
        wp_approach_out[6] = rtb_y_j5[(rtDW.UnitDelay_DSTATE_b - 1) * 3];
        q2_q2 = flight_dir[1] * q1_q1;
        wp_approach_out[1] = rtDW.DiscreteTimeIntegratory_DSTA_n2[1] - q2_q2;
        wp_approach_out[4] = rtDW.DiscreteTimeIntegratory_DSTA_n2[1] + q2_q2;
        wp_approach_out[7] = scale;
        q2_q2 = q2_q3 * q1_q1;
        wp_approach_out[2] = rtDW.DiscreteTimeIntegratory_DSTA_n2[2] - q2_q2;
        wp_approach_out[5] = rtDW.DiscreteTimeIntegratory_DSTA_n2[2] + q2_q2;
        wp_approach_out[8] = q0_q3;

        // :  if wp_idx == num_wp-1
        if (p3 - 1 == rtDW.UnitDelay_DSTATE_b) {
          // :  wp_approach_out(:,4) = waypoints(:,wp_idx);
          // :  wp_approach_out(:,5) = waypoints(:,1);
          wp_approach_out[9] = rtb_y_j5[(rtDW.UnitDelay_DSTATE_b - 1) * 3];
          wp_approach_out[12] = rtb_y_j5[0];
          wp_approach_out[10] = scale;
          wp_approach_out[13] = rtb_y_j5[1];
          wp_approach_out[11] = q0_q3;
          wp_approach_out[14] = rtb_y_j5[2];
        } else if (rtDW.UnitDelay_DSTATE_b == p3) {
          // :  elseif wp_idx == num_wp
          // :  wp_approach_out(:,4) = waypoints(:,1);
          // :  wp_approach_out(:,5) = waypoints(:,2);
          wp_approach_out[9] = rtb_y_j5[0];
          wp_approach_out[12] = rtb_y_j5[3];
          wp_approach_out[10] = rtb_y_j5[1];
          wp_approach_out[13] = rtb_y_j5[4];
          wp_approach_out[11] = rtb_y_j5[2];
          wp_approach_out[14] = rtb_y_j5[5];
        } else {
          // :  else
          // :  wp_approach_out(:,4) = waypoints(:,wp_idx+1);
          if (rtDW.UnitDelay_DSTATE_b > 2147483646) {
            stage_app_2 = MAX_int32_T;
          } else {
            stage_app_2 = rtDW.UnitDelay_DSTATE_b + 1;
          }

          // :  wp_approach_out(:,5) = waypoints(:,wp_idx+2);
          if (rtDW.UnitDelay_DSTATE_b > 2147483645) {
            itmp = MAX_int32_T;
          } else {
            itmp = rtDW.UnitDelay_DSTATE_b + 2;
          }

          stage_app_2 = (stage_app_2 - 1) * 3;
          wp_approach_out[9] = rtb_y_j5[stage_app_2];
          itmp = (itmp - 1) * 3;
          wp_approach_out[12] = rtb_y_j5[itmp];
          wp_approach_out[10] = rtb_y_j5[stage_app_2 + 1];
          wp_approach_out[13] = rtb_y_j5[itmp + 1];
          wp_approach_out[11] = rtb_y_j5[stage_app_2 + 2];
          wp_approach_out[14] = rtb_y_j5[itmp + 2];
        }

        // :  is_approach(:) = 1;
        is_approach = true;

        // :  wp_idx_app(:) = 2;
        // :  stage_app(:) = 1;
        // :  [p_match,wp_idx_app,stage_app,t,d] = wpnavMatch(wp_approach_out,wp_radius,wp_idx_app,stage_app,p); 
        itmp = 2;
        c_stage_app = 1;
        wpnavMatch(wp_approach_out, rtP.lindi.psc.rm.wprad, &itmp, &c_stage_app,
                   rtDW.DiscreteTimeIntegratory_DSTA_n2, rtb_Sum2_ny, &rtDW.t_g,
                   &q1_q1);

        // :  [p_match_2,wp_idx_app_2,stage_app_2,t_2,d_2] = wpnavMatch(wp_approach_out,wp_radius,wp_idx_app,stage_app,p_ahead); 
        wp_idx_app_2 = itmp;
        stage_app_2 = c_stage_app;
        wpnavMatch(wp_approach_out, rtP.lindi.psc.rm.wprad, &wp_idx_app_2,
                   &stage_app_2, rtb_Sum2_ci, p_match_2, &q1_q1, &q2_q2);

        // :  stage_2 = stage_;
        stage_2 = rtDW.UnitDelay1_DSTATE_g;

        // :  wp_idx_2 = wp_idx;
        wp_idx_2 = rtDW.UnitDelay_DSTATE_b;
      } else {
        // :  else
        // :  wp_idx = wp_idx_tmp;
        i = wp_idx_tmp;

        // :  stage_ = stage_tmp;
        p2 = stage_tmp;
      }

      // :  if (wp_idx_app == 4 && stage_app == 1) || wp_idx_app == 5
      if (((itmp == 4) && (c_stage_app == 1)) || (itmp == 5)) {
        // :  wp_idx_app(:) = 2;
        itmp = 2;

        // :  is_approach(:) = 0;
        is_approach = false;

        // :  wp_idx = wp_idx + 1;
        if (i > 2147483646) {
          i = MAX_int32_T;
        } else {
          i++;
        }

        // :  stage_(:) = 1;
        // :  [p_match,wp_idx,stage_,t,d] = wpnavMatch(waypoints(:,1:num_wp),wp_radius,wp_idx,stage_,p); 
        p2 = 1;
        rtb_y_g_size_0[0] = 3;
        rtb_y_g_size_0[1] = p3;
        for (stage_app_2 = 0; stage_app_2 < p3; stage_app_2++) {
          c_XYZ[3 * stage_app_2] = rtb_y_j5[3 * stage_app_2];
          c_XYZ[1 + 3 * stage_app_2] = rtb_y_j5[3 * stage_app_2 + 1];
          c_XYZ[2 + 3 * stage_app_2] = rtb_y_j5[3 * stage_app_2 + 2];
        }

        wpnavMatch_p(c_XYZ, rtb_y_g_size_0, rtP.lindi.psc.rm.wprad, &i, &p2,
                     rtDW.DiscreteTimeIntegratory_DSTA_n2, rtb_Sum2_ny,
                     &rtDW.t_g, &q1_q1);

        // :  [p_match_2,wp_idx_2,stage_2,t_2,d_2] = wpnavMatch(waypoints(:,1:num_wp),wp_radius,wp_idx,stage_,p_ahead); 
        wp_idx_2 = i;
        stage_2 = p2;
        rtb_y_g_size[0] = 3;
        rtb_y_g_size[1] = p3;
        for (stage_app_2 = 0; stage_app_2 < p3; stage_app_2++) {
          c_XYZ[3 * stage_app_2] = rtb_y_j5[3 * stage_app_2];
          c_XYZ[1 + 3 * stage_app_2] = rtb_y_j5[3 * stage_app_2 + 1];
          c_XYZ[2 + 3 * stage_app_2] = rtb_y_j5[3 * stage_app_2 + 2];
        }

        wpnavMatch_p(c_XYZ, rtb_y_g_size, rtP.lindi.psc.rm.wprad, &wp_idx_2,
                     &stage_2, rtb_Sum2_ci, p_match_2, &q1_q1, &q2_q2);

        // :  stage_app_2 = stage_app;
        stage_app_2 = c_stage_app;

        // :  wp_idx_app_2 = wp_idx_app;
        wp_idx_app_2 = 2;
      }

      // :  V_K = norm(V_Kg,2);
      q_bg_unsigned_idx_3 = xnrm2_f(rtDW.DiscreteTimeIntegratory_DSTAT_j);

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
              I_b[3 * stage_app_2] = rtb_y_j5[p3];
              I_b[1 + 3 * stage_app_2] = rtb_y_j5[p3 + 1];
              I_b[2 + 3 * stage_app_2] = rtb_y_j5[p3 + 2];
            }
          } else if (wp_idx_2 == 2) {
            // :  elseif wp_idx_2 == 2
            // :  waypoints3x3 = waypoints(:,[num_wp,1,2]);
            p3_0[0] = (uint16_T)(p3 - 1);
            p3_0[1] = 0U;
            p3_0[2] = 1U;
            for (stage_app_2 = 0; stage_app_2 < 3; stage_app_2++) {
              p3 = 3 * p3_0[stage_app_2];
              I_b[3 * stage_app_2] = rtb_y_j5[p3];
              I_b[1 + 3 * stage_app_2] = rtb_y_j5[p3 + 1];
              I_b[2 + 3 * stage_app_2] = rtb_y_j5[p3 + 2];
            }
          } else {
            // :  else
            // :  waypoints3x3 = waypoints(:,[num_wp-1,num_wp,1]);
            p3_0[0] = (uint16_T)(p3 - 2);
            p3_0[1] = (uint16_T)(p3 - 1);
            p3_0[2] = 0U;
            for (stage_app_2 = 0; stage_app_2 < 3; stage_app_2++) {
              p3 = 3 * p3_0[stage_app_2];
              I_b[3 * stage_app_2] = rtb_y_j5[p3];
              I_b[1 + 3 * stage_app_2] = rtb_y_j5[p3 + 1];
              I_b[2 + 3 * stage_app_2] = rtb_y_j5[p3 + 2];
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
              p3 = 3 * h[stage_app_2];
              I_b[3 * stage_app_2] = wp_approach_out[p3];
              I_b[1 + 3 * stage_app_2] = wp_approach_out[p3 + 1];
              I_b[2 + 3 * stage_app_2] = wp_approach_out[p3 + 2];
            }
          } else {
            // :  else
            // :  waypoints3x3 = wp_approach_out(:,[end-1,end,1]);
            for (stage_app_2 = 0; stage_app_2 < 3; stage_app_2++) {
              p3 = 3 * g[stage_app_2];
              I_b[3 * stage_app_2] = wp_approach_out[p3];
              I_b[1 + 3 * stage_app_2] = wp_approach_out[p3 + 1];
              I_b[2 + 3 * stage_app_2] = wp_approach_out[p3 + 2];
            }
          }
        }

        // :  circ_seg = wpnavCircSeg(waypoints3x3,wp_radius);
        wpnavCircSeg(I_b, rtP.lindi.psc.rm.wprad, &scale, rtb_Sum2_ci,
                     dir_next_wp, &t, rtb_Sum2_k, flight_dir, &absxk, expl_temp);

        // :  v(:) = wpnavCircSegGetVel(circ_seg,t_2,V_K);
        // 'wpnavCircSegGetVel:27' tangent_vec_p1_unit = cross( circ_seg.n, circ_seg.start-circ_seg.center ); 
        rtb_Sum2_k[0] -= rtb_Sum2_ci[0];
        rtb_Sum2_k[1] -= rtb_Sum2_ci[1];
        q2_q2 = rtb_Sum2_k[2] - rtb_Sum2_ci[2];
        rtb_Sum2_k[2] = q2_q2;
        flight_dir[0] = dir_next_wp[1] * q2_q2 - dir_next_wp[2] * rtb_Sum2_k[1];
        flight_dir[1] = dir_next_wp[2] * rtb_Sum2_k[0] - dir_next_wp[0] * q2_q2;
        flight_dir[2] = dir_next_wp[0] * rtb_Sum2_k[1] - dir_next_wp[1] *
          rtb_Sum2_k[0];

        // 'wpnavCircSegGetVel:28' tangent_vec_p1_unit = divideFinite( tangent_vec_p1_unit, norm( tangent_vec_p1_unit, 2 ) ); 
        absxk = xnrm2_f(flight_dir);

        // 'divideFinite:29' if numel(B)>1
        // 'divideFinite:31' else
        // 'divideFinite:32' if abs(B)<eps
        if (std::abs(absxk) < 2.22044605E-16F) {
          // 'divideFinite:33' B(:) = eps;
          absxk = 2.22044605E-16F;
        }

        // 'divideFinite:36' C = A ./ B;
        // 'wpnavCircSegGetVel:30' angle = t * circ_seg.angle;
        // 'wpnavCircSegGetVel:31' tangent_vec_t_unit = axisAngle(tangent_vec_p1_unit,circ_seg.n,angle); 
        // 'wpnavCircSegGetVel:33' V_Kg = tangent_vec_t_unit * V;
        dir_next_wp_0[0] = dir_next_wp[0];
        flight_dir[0] /= absxk;
        dir_next_wp_0[1] = dir_next_wp[1];
        flight_dir[1] /= absxk;
        dir_next_wp_0[2] = dir_next_wp[2];
        flight_dir[2] /= absxk;
        Phi_i = q1_q1 * t;
        axisAngle(flight_dir, dir_next_wp_0, Phi_i, expl_temp);

        // :  a(:) = wpnavCircSegGetAcc(circ_seg,t_2,V_K);
        // 'wpnavCircSegGetAcc:29' p = wpnavCircSegGetPos( circ_seg, t );
        // 'wpnavCircSegGetPos:26' angle = t * circ_seg.angle;
        // 'wpnavCircSegGetPos:27' s_g = circ_seg.center + axisAngle(circ_seg.start-circ_seg.center,circ_seg.n,angle); 
        // 'wpnavCircSegGetAcc:30' dir_vec_unit = circ_seg.center - p;
        axisAngle(rtb_Sum2_k, dir_next_wp, Phi_i, dir_next_wp_0);
        q2_q2 = expl_temp[0] * q_bg_unsigned_idx_3;
        flight_dir[0] = rtb_Sum2_ci[0] - (rtb_Sum2_ci[0] + dir_next_wp_0[0]);
        q1_q3 = expl_temp[1] * q_bg_unsigned_idx_3;
        flight_dir[1] = rtb_Sum2_ci[1] - (rtb_Sum2_ci[1] + dir_next_wp_0[1]);
        Phi_i = expl_temp[2] * q_bg_unsigned_idx_3;
        flight_dir[2] = rtb_Sum2_ci[2] - (rtb_Sum2_ci[2] + dir_next_wp_0[2]);

        // 'wpnavCircSegGetAcc:31' dir_vec_unit = divideFinite( dir_vec_unit, norm( dir_vec_unit, 2 ) ); 
        absxk = xnrm2_f(flight_dir);

        // 'divideFinite:29' if numel(B)>1
        // 'divideFinite:31' else
        // 'divideFinite:32' if abs(B)<eps
        if (std::abs(absxk) < 2.22044605E-16F) {
          // 'divideFinite:33' B(:) = eps;
          absxk = 2.22044605E-16F;
        }

        // 'divideFinite:36' C = A ./ B;
        flight_dir[0] /= absxk;
        flight_dir[1] /= absxk;

        // 'wpnavCircSegGetAcc:32' a_Kg = divideFinite( V*V, circ_seg.r ) * dir_vec_unit; 
        // 'divideFinite:29' if numel(B)>1
        // 'divideFinite:31' else
        // 'divideFinite:32' if abs(B)<eps
        if (std::abs(scale) < 2.22044605E-16F) {
          // 'divideFinite:33' B(:) = eps;
          scale = 2.22044605E-16F;
        }

        // 'divideFinite:36' C = A ./ B;
        q2_q3 = q_bg_unsigned_idx_3 * q_bg_unsigned_idx_3 / scale;
        t = q2_q3 * flight_dir[0];
        q1_q2 = q2_q3 * flight_dir[1];
        absxk = flight_dir[2] / absxk * q2_q3;
      } else {
        // :  else
        // :  if ~is_approach
        if (!is_approach) {
          // :  if wp_idx_2 > 1
          if (wp_idx_2 > 1) {
            // :  wp1 = waypoints(:,wp_idx_2-1);
            // :  wp2 = waypoints(:,wp_idx_2);
            stage_app_2 = (wp_idx_2 - 2) * 3;
            dir_next_wp[0] = rtb_y_j5[stage_app_2];
            p3 = (wp_idx_2 - 1) * 3;
            rtb_Sum2_ci[0] = rtb_y_j5[p3];
            dir_next_wp[1] = rtb_y_j5[stage_app_2 + 1];
            rtb_Sum2_ci[1] = rtb_y_j5[p3 + 1];
            dir_next_wp[2] = rtb_y_j5[stage_app_2 + 2];
            rtb_Sum2_ci[2] = rtb_y_j5[p3 + 2];
          } else {
            // :  else
            // :  wp1 = waypoints(:,num_wp);
            // :  wp2 = waypoints(:,1);
            stage_app_2 = (p3 - 1) * 3;
            dir_next_wp[0] = rtb_y_j5[stage_app_2];
            rtb_Sum2_ci[0] = rtb_y_j5[0];
            dir_next_wp[1] = rtb_y_j5[stage_app_2 + 1];
            rtb_Sum2_ci[1] = rtb_y_j5[1];
            dir_next_wp[2] = rtb_y_j5[stage_app_2 + 2];
            rtb_Sum2_ci[2] = rtb_y_j5[2];
          }
        } else {
          // :  else
          // :  wp1 = wp_approach_out(:,wp_idx_app_2-1);
          if (wp_idx_app_2 < -2147483647) {
            stage_app_2 = MIN_int32_T;
          } else {
            stage_app_2 = wp_idx_app_2 - 1;
          }

          // :  wp2 = wp_approach_out(:,wp_idx_app_2);
          stage_app_2 = (stage_app_2 - 1) * 3;
          dir_next_wp[0] = wp_approach_out[stage_app_2];
          p3 = (wp_idx_app_2 - 1) * 3;
          rtb_Sum2_ci[0] = wp_approach_out[p3];
          dir_next_wp[1] = wp_approach_out[stage_app_2 + 1];
          rtb_Sum2_ci[1] = wp_approach_out[p3 + 1];
          dir_next_wp[2] = wp_approach_out[stage_app_2 + 2];
          rtb_Sum2_ci[2] = wp_approach_out[p3 + 2];
        }

        // :  v(:) = wpnavLineGetVel(wp1,wp2,V_K);
        // 'wpnavLineGetVel:28' dir_vec_unit = p2-p1;
        rtb_Sum2_ci[0] -= dir_next_wp[0];
        rtb_Sum2_ci[1] -= dir_next_wp[1];
        rtb_Sum2_ci[2] -= dir_next_wp[2];

        // 'wpnavLineGetVel:29' dir_vec_unit = divideFinite( dir_vec_unit, norm( dir_vec_unit, 2 ) ); 
        scale = xnrm2_f(rtb_Sum2_ci);

        // 'divideFinite:29' if numel(B)>1
        // 'divideFinite:31' else
        // 'divideFinite:32' if abs(B)<eps
        if (std::abs(scale) < 2.22044605E-16F) {
          // 'divideFinite:33' B(:) = eps;
          scale = 2.22044605E-16F;
        }

        // 'divideFinite:36' C = A ./ B;
        // 'wpnavLineGetVel:30' V_Kg = V * dir_vec_unit;
        q2_q2 = rtb_Sum2_ci[0] / scale * q_bg_unsigned_idx_3;
        q1_q3 = rtb_Sum2_ci[1] / scale * q_bg_unsigned_idx_3;
        Phi_i = rtb_Sum2_ci[2] / scale * q_bg_unsigned_idx_3;
      }

      rtDW.p_match[0] = rtb_Sum2_ny[0];
      rtDW.p_match[1] = rtb_Sum2_ny[1];
      rtDW.p_match[2] = rtb_Sum2_ny[2];

      // DataTypeConversion: '<S25>/Data Type Conversion8' incorporates:
      //   MATLAB Function: '<S25>/WpNav Matching'

      rtDW.stage_e = (real32_T)p2;

      // DataTypeConversion: '<S25>/Data Type Conversion9' incorporates:
      //   MATLAB Function: '<S25>/WpNav Matching'

      rtDW.wp_idx_j = (real32_T)i;

      // MATLAB Function: '<S25>/Look Ahead1' incorporates:
      //   Constant: '<S25>/Constant3'

      LookAhead(rtb_y_pd, rtb_y_ai, rtb_y_k, 2.0F / rtP.lindi.atc.rm.rfreq,
                rtb_Sum2_ny);

      // Update for UnitDelay: '<S25>/Unit Delay4' incorporates:
      //   MATLAB Function: '<S25>/WpNav Matching'

      for (stage_app_2 = 0; stage_app_2 < 15; stage_app_2++) {
        rtDW.UnitDelay4_DSTATE[stage_app_2] = wp_approach_out[stage_app_2];
      }

      // End of Update for UnitDelay: '<S25>/Unit Delay4'

      // Update for UnitDelay: '<S25>/Unit Delay' incorporates:
      //   MATLAB Function: '<S25>/WpNav Matching'

      rtDW.UnitDelay_DSTATE_b = i;

      // Update for UnitDelay: '<S25>/Unit Delay1' incorporates:
      //   MATLAB Function: '<S25>/WpNav Matching'

      rtDW.UnitDelay1_DSTATE_g = p2;

      // Update for UnitDelay: '<S25>/Unit Delay5' incorporates:
      //   MATLAB Function: '<S25>/WpNav Matching'

      rtDW.UnitDelay5_DSTATE = is_approach;

      // Update for UnitDelay: '<S25>/Unit Delay2' incorporates:
      //   MATLAB Function: '<S25>/WpNav Matching'

      rtDW.UnitDelay2_DSTATE_i = itmp;

      // Update for UnitDelay: '<S25>/Unit Delay3' incorporates:
      //   MATLAB Function: '<S25>/WpNav Matching'

      rtDW.UnitDelay3_DSTATE = c_stage_app;
      if (!rtDW.FlightPathSmoothing_MODE) {
        // InitializeConditions for DiscreteIntegrator: '<S30>/Discrete-Time Integrator y' 
        rtDW.DiscreteTimeIntegratory_IC_L_iy = 1U;

        // InitializeConditions for DiscreteIntegrator: '<S29>/Discrete-Time Integrator y' 
        rtDW.DiscreteTimeIntegratory_IC_L_ld = 1U;

        // InitializeConditions for DiscreteIntegrator: '<S28>/Discrete-Time Integrator y' 
        rtDW.DiscreteTimeIntegratory_IC_LO_h = 1U;

        // InitializeConditions for DiscreteIntegrator: '<S30>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_D_nq[0] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S29>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_D_pb[0] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S28>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_D_be[0] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S30>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_D_nq[1] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S29>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_D_pb[1] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S28>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_D_be[1] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S30>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_D_nq[2] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S29>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_D_pb[2] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S28>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_D_be[2] = 0.0F;
        rtDW.FlightPathSmoothing_MODE = true;
      }

      // DiscreteIntegrator: '<S30>/Discrete-Time Integrator y'
      if (rtDW.DiscreteTimeIntegratory_IC_L_iy != 0) {
        rtDW.DiscreteTimeIntegratory_DSTA_df[0] = rtb_Sum2_ny[0];
        rtDW.DiscreteTimeIntegratory_DSTA_df[1] = rtb_Sum2_ny[1];
        rtDW.DiscreteTimeIntegratory_DSTA_df[2] = rtb_Sum2_ny[2];
      }

      scale = rtDW.DiscreteTimeIntegratory_DSTA_df[0];
      q0_q3 = rtDW.DiscreteTimeIntegratory_DSTA_df[1];
      q2_q3 = rtDW.DiscreteTimeIntegratory_DSTA_df[2];

      // End of DiscreteIntegrator: '<S30>/Discrete-Time Integrator y'

      // Product: '<S30>/Divide' incorporates:
      //   Constant: '<S30>/omega'
      //   Product: '<S30>/omega^2'

      q1_q1 = 2.0F / rtP.lindi.psc.rm.T;

      // Gain: '<S30>/Gain' incorporates:
      //   Constant: '<S30>/d'
      //   Constant: '<S30>/omega'
      //   Gain: '<S28>/Gain'
      //   Gain: '<S29>/Gain'
      //   Product: '<S30>/Divide'

      q_bg_unsigned_idx_0 = 1.0F / q1_q1 * 2.0F;

      // Product: '<S30>/omega^2' incorporates:
      //   Product: '<S28>/omega^2'
      //   Product: '<S29>/omega^2'

      q1_q1 *= q1_q1;

      // DiscreteIntegrator: '<S29>/Discrete-Time Integrator y'
      if (rtDW.DiscreteTimeIntegratory_IC_L_ld != 0) {
        rtDW.DiscreteTimeIntegratory_DSTA_jr[0] = q2_q2;
        rtDW.DiscreteTimeIntegratory_DSTA_jr[1] = q1_q3;
        rtDW.DiscreteTimeIntegratory_DSTA_jr[2] = Phi_i;
      }

      q_bg_unsigned_idx_3 = rtDW.DiscreteTimeIntegratory_DSTA_jr[0];
      q_bg_unsigned_idx_1 = rtDW.DiscreteTimeIntegratory_DSTA_jr[1];
      q_bg_unsigned_idx_2 = rtDW.DiscreteTimeIntegratory_DSTA_jr[2];

      // End of DiscreteIntegrator: '<S29>/Discrete-Time Integrator y'

      // DiscreteIntegrator: '<S28>/Discrete-Time Integrator y'
      if (rtDW.DiscreteTimeIntegratory_IC_LO_h != 0) {
        rtDW.DiscreteTimeIntegratory_DSTA_ff[0] = t;
        rtDW.DiscreteTimeIntegratory_DSTA_ff[1] = q1_q2;
        rtDW.DiscreteTimeIntegratory_DSTA_ff[2] = absxk;
      }

      // Update for DiscreteIntegrator: '<S30>/Discrete-Time Integrator y'
      rtDW.DiscreteTimeIntegratory_IC_L_iy = 0U;

      // Update for DiscreteIntegrator: '<S29>/Discrete-Time Integrator y'
      rtDW.DiscreteTimeIntegratory_IC_L_ld = 0U;

      // Update for DiscreteIntegrator: '<S28>/Discrete-Time Integrator y'
      rtDW.DiscreteTimeIntegratory_IC_LO_h = 0U;

      // DiscreteIntegrator: '<S28>/Discrete-Time Integrator y'
      rtb_Sum2_o3 = rtDW.DiscreteTimeIntegratory_DSTA_ff[0];

      // Update for DiscreteIntegrator: '<S30>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S30>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_DSTA_df[0] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_D_nq[0];

      // Update for DiscreteIntegrator: '<S30>/Discrete-Time Integrator y_dt' incorporates:
      //   Gain: '<S30>/Gain'
      //   Product: '<S30>/Product1'
      //   Product: '<S30>/Product2'
      //   Product: '<S30>/omega^2'
      //   Sum: '<S30>/Sum2'
      //   Sum: '<S30>/Sum3'

      rtDW.DiscreteTimeIntegratory_dt_D_nq[0] += (p_match_2[0] -
        (rtDW.DiscreteTimeIntegratory_dt_D_nq[0] * q_bg_unsigned_idx_0 + scale))
        * q1_q1 * 0.0025F;

      // Update for DiscreteIntegrator: '<S29>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S29>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_DSTA_jr[0] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_D_pb[0];

      // Update for DiscreteIntegrator: '<S29>/Discrete-Time Integrator y_dt' incorporates:
      //   Product: '<S29>/Product1'
      //   Product: '<S29>/Product2'
      //   Sum: '<S29>/Sum2'
      //   Sum: '<S29>/Sum3'

      rtDW.DiscreteTimeIntegratory_dt_D_pb[0] += (q2_q2 -
        (rtDW.DiscreteTimeIntegratory_dt_D_pb[0] * q_bg_unsigned_idx_0 +
         q_bg_unsigned_idx_3)) * q1_q1 * 0.0025F;

      // Update for DiscreteIntegrator: '<S28>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S28>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_DSTA_ff[0] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_D_be[0];

      // Update for DiscreteIntegrator: '<S28>/Discrete-Time Integrator y_dt' incorporates:
      //   Product: '<S28>/Product1'
      //   Product: '<S28>/Product2'
      //   Sum: '<S28>/Sum2'
      //   Sum: '<S28>/Sum3'

      rtDW.DiscreteTimeIntegratory_dt_D_be[0] += (t -
        (rtDW.DiscreteTimeIntegratory_dt_D_be[0] * q_bg_unsigned_idx_0 +
         rtb_Sum2_o3)) * q1_q1 * 0.0025F;
      y_m[0] = rtb_Sum2_o3;

      // DiscreteIntegrator: '<S28>/Discrete-Time Integrator y'
      rtb_Sum2_o3 = rtDW.DiscreteTimeIntegratory_DSTA_ff[1];

      // Update for DiscreteIntegrator: '<S30>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S30>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_DSTA_df[1] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_D_nq[1];

      // Update for DiscreteIntegrator: '<S30>/Discrete-Time Integrator y_dt' incorporates:
      //   Gain: '<S30>/Gain'
      //   Product: '<S30>/Product1'
      //   Product: '<S30>/Product2'
      //   Product: '<S30>/omega^2'
      //   Sum: '<S30>/Sum2'
      //   Sum: '<S30>/Sum3'

      rtDW.DiscreteTimeIntegratory_dt_D_nq[1] += (p_match_2[1] -
        (rtDW.DiscreteTimeIntegratory_dt_D_nq[1] * q_bg_unsigned_idx_0 + q0_q3))
        * q1_q1 * 0.0025F;

      // Update for DiscreteIntegrator: '<S29>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S29>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_DSTA_jr[1] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_D_pb[1];

      // Update for DiscreteIntegrator: '<S29>/Discrete-Time Integrator y_dt' incorporates:
      //   Product: '<S29>/Product1'
      //   Product: '<S29>/Product2'
      //   Sum: '<S29>/Sum2'
      //   Sum: '<S29>/Sum3'

      rtDW.DiscreteTimeIntegratory_dt_D_pb[1] += (q1_q3 -
        (rtDW.DiscreteTimeIntegratory_dt_D_pb[1] * q_bg_unsigned_idx_0 +
         q_bg_unsigned_idx_1)) * q1_q1 * 0.0025F;

      // Update for DiscreteIntegrator: '<S28>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S28>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_DSTA_ff[1] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_D_be[1];

      // Update for DiscreteIntegrator: '<S28>/Discrete-Time Integrator y_dt' incorporates:
      //   Product: '<S28>/Product1'
      //   Product: '<S28>/Product2'
      //   Sum: '<S28>/Sum2'
      //   Sum: '<S28>/Sum3'

      rtDW.DiscreteTimeIntegratory_dt_D_be[1] += (q1_q2 -
        (rtDW.DiscreteTimeIntegratory_dt_D_be[1] * q_bg_unsigned_idx_0 +
         rtb_Sum2_o3)) * q1_q1 * 0.0025F;
      y_m[1] = rtb_Sum2_o3;

      // DiscreteIntegrator: '<S28>/Discrete-Time Integrator y'
      rtb_Sum2_o3 = rtDW.DiscreteTimeIntegratory_DSTA_ff[2];

      // Update for DiscreteIntegrator: '<S30>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S30>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_DSTA_df[2] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_D_nq[2];

      // Update for DiscreteIntegrator: '<S30>/Discrete-Time Integrator y_dt' incorporates:
      //   Gain: '<S30>/Gain'
      //   Product: '<S30>/Product1'
      //   Product: '<S30>/Product2'
      //   Product: '<S30>/omega^2'
      //   Sum: '<S30>/Sum2'
      //   Sum: '<S30>/Sum3'

      rtDW.DiscreteTimeIntegratory_dt_D_nq[2] += (p_match_2[2] -
        (rtDW.DiscreteTimeIntegratory_dt_D_nq[2] * q_bg_unsigned_idx_0 + q2_q3))
        * q1_q1 * 0.0025F;

      // Update for DiscreteIntegrator: '<S29>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S29>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_DSTA_jr[2] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_D_pb[2];

      // Update for DiscreteIntegrator: '<S29>/Discrete-Time Integrator y_dt' incorporates:
      //   Product: '<S29>/Product1'
      //   Product: '<S29>/Product2'
      //   Sum: '<S29>/Sum2'
      //   Sum: '<S29>/Sum3'

      rtDW.DiscreteTimeIntegratory_dt_D_pb[2] += (Phi_i -
        (rtDW.DiscreteTimeIntegratory_dt_D_pb[2] * q_bg_unsigned_idx_0 +
         q_bg_unsigned_idx_2)) * q1_q1 * 0.0025F;

      // Update for DiscreteIntegrator: '<S28>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S28>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_DSTA_ff[2] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_D_be[2];

      // Update for DiscreteIntegrator: '<S28>/Discrete-Time Integrator y_dt' incorporates:
      //   Product: '<S28>/Product1'
      //   Product: '<S28>/Product2'
      //   Sum: '<S28>/Sum2'
      //   Sum: '<S28>/Sum3'

      rtDW.DiscreteTimeIntegratory_dt_D_be[2] += (absxk -
        (rtDW.DiscreteTimeIntegratory_dt_D_be[2] * q_bg_unsigned_idx_0 +
         rtb_Sum2_o3)) * q1_q1 * 0.0025F;
      y_m[2] = rtb_Sum2_o3;
      if (!rtDW.NDIPositionController_MODE) {
        // InitializeConditions for DiscreteIntegrator: '<S50>/Discrete-Time Integrator y' 
        rtDW.DiscreteTimeIntegratory_IC_LO_d = 1U;

        // InitializeConditions for DiscreteIntegrator: '<S49>/Discrete-Time Integrator y' 
        rtDW.DiscreteTimeIntegratory_IC_LO_p = 1U;

        // InitializeConditions for DiscreteIntegrator: '<S51>/Discrete-Time Integrator y' 
        rtDW.DiscreteTimeIntegratory_IC_LO_l = 1U;

        // InitializeConditions for DiscreteIntegrator: '<S50>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_D_jv[0] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S49>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_D_ip[0] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S51>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_DS_c[0] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S50>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_D_jv[1] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S49>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_D_ip[1] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S51>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_DS_c[1] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S50>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_D_jv[2] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S49>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_D_ip[2] = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S51>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_DS_c[2] = 0.0F;
        rtDW.NDIPositionController_MODE = true;
      }

      // DiscreteIntegrator: '<S50>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S28>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S41>/Discrete-Time Integrator y'

      if (rtDW.DiscreteTimeIntegratory_IC_LO_d != 0) {
        rtDW.DiscreteTimeIntegratory_DSTA_nm[0] =
          rtDW.DiscreteTimeIntegratory_DSTA_n2[0];
        rtDW.DiscreteTimeIntegratory_DSTA_nm[1] =
          rtDW.DiscreteTimeIntegratory_DSTA_n2[1];
        rtDW.DiscreteTimeIntegratory_DSTA_nm[2] =
          rtDW.DiscreteTimeIntegratory_DSTA_n2[2];
      }

      // Gain: '<S50>/Gain' incorporates:
      //   Constant: '<S50>/d'
      //   Constant: '<S50>/omega'
      //   Gain: '<S49>/Gain'
      //   Gain: '<S51>/Gain'
      //   Product: '<S50>/Divide'

      q1_q1 = 1.0F / rtP.lindi.atc.rm.rfreq * 2.0F;

      // Sum: '<S50>/Sum2' incorporates:
      //   DiscreteIntegrator: '<S50>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S50>/Discrete-Time Integrator y_dt'
      //   Gain: '<S50>/Gain'
      //   Product: '<S50>/Product2'
      //   Sum: '<S50>/Sum3'

      rtb_y_pd[0] = scale - (rtDW.DiscreteTimeIntegratory_dt_D_jv[0] * q1_q1 +
        rtDW.DiscreteTimeIntegratory_DSTA_nm[0]);

      // Sum: '<S16>/Add' incorporates:
      //   DiscreteIntegrator: '<S41>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S50>/Discrete-Time Integrator y'

      rtDW.e_s_g_e[0] = rtDW.DiscreteTimeIntegratory_DSTA_nm[0] -
        rtDW.DiscreteTimeIntegratory_DSTA_n2[0];

      // Sum: '<S50>/Sum2' incorporates:
      //   DiscreteIntegrator: '<S50>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S50>/Discrete-Time Integrator y_dt'
      //   Gain: '<S50>/Gain'
      //   Product: '<S50>/Product2'
      //   Sum: '<S50>/Sum3'

      rtb_y_pd[1] = q0_q3 - (rtDW.DiscreteTimeIntegratory_dt_D_jv[1] * q1_q1 +
        rtDW.DiscreteTimeIntegratory_DSTA_nm[1]);

      // Sum: '<S16>/Add' incorporates:
      //   DiscreteIntegrator: '<S41>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S50>/Discrete-Time Integrator y'

      rtDW.e_s_g_e[1] = rtDW.DiscreteTimeIntegratory_DSTA_nm[1] -
        rtDW.DiscreteTimeIntegratory_DSTA_n2[1];

      // Sum: '<S50>/Sum2' incorporates:
      //   DiscreteIntegrator: '<S50>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S50>/Discrete-Time Integrator y_dt'
      //   Gain: '<S50>/Gain'
      //   Product: '<S50>/Product2'
      //   Sum: '<S50>/Sum3'

      rtb_y_pd[2] = q2_q3 - (rtDW.DiscreteTimeIntegratory_dt_D_jv[2] * q1_q1 +
        rtDW.DiscreteTimeIntegratory_DSTA_nm[2]);

      // Sum: '<S16>/Add' incorporates:
      //   DiscreteIntegrator: '<S41>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S50>/Discrete-Time Integrator y'

      rtDW.e_s_g_e[2] = rtDW.DiscreteTimeIntegratory_DSTA_nm[2] -
        rtDW.DiscreteTimeIntegratory_DSTA_n2[2];

      // Product: '<S50>/omega^2' incorporates:
      //   Constant: '<S50>/omega'
      //   Product: '<S49>/omega^2'
      //   Product: '<S51>/omega^2'

      q_bg_unsigned_idx_0 = rtP.lindi.atc.rm.rfreq * rtP.lindi.atc.rm.rfreq;

      // DiscreteIntegrator: '<S49>/Discrete-Time Integrator y'
      if (rtDW.DiscreteTimeIntegratory_IC_LO_p != 0) {
        rtDW.DiscreteTimeIntegratory_DSTA_e3[0] = q_bg_unsigned_idx_3;
        rtDW.DiscreteTimeIntegratory_DSTA_e3[1] = q_bg_unsigned_idx_1;
        rtDW.DiscreteTimeIntegratory_DSTA_e3[2] = q_bg_unsigned_idx_2;
      }

      // Sum: '<S49>/Sum2' incorporates:
      //   DiscreteIntegrator: '<S49>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S49>/Discrete-Time Integrator y_dt'
      //   Product: '<S49>/Product2'
      //   Sum: '<S49>/Sum3'

      rtb_y_ai[0] = q_bg_unsigned_idx_3 - (rtDW.DiscreteTimeIntegratory_dt_D_ip
        [0] * q1_q1 + rtDW.DiscreteTimeIntegratory_DSTA_e3[0]);

      // Sum: '<S16>/Add1' incorporates:
      //   DiscreteIntegrator: '<S42>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S49>/Discrete-Time Integrator y'

      rtDW.e_s_g_dt_i[0] = rtDW.DiscreteTimeIntegratory_DSTA_e3[0] -
        rtDW.DiscreteTimeIntegratory_DSTAT_j[0];

      // Sum: '<S49>/Sum2' incorporates:
      //   DiscreteIntegrator: '<S49>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S49>/Discrete-Time Integrator y_dt'
      //   Product: '<S49>/Product2'
      //   Sum: '<S49>/Sum3'

      rtb_y_ai[1] = q_bg_unsigned_idx_1 - (rtDW.DiscreteTimeIntegratory_dt_D_ip
        [1] * q1_q1 + rtDW.DiscreteTimeIntegratory_DSTA_e3[1]);

      // Sum: '<S16>/Add1' incorporates:
      //   DiscreteIntegrator: '<S42>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S49>/Discrete-Time Integrator y'

      rtDW.e_s_g_dt_i[1] = rtDW.DiscreteTimeIntegratory_DSTA_e3[1] -
        rtDW.DiscreteTimeIntegratory_DSTAT_j[1];

      // Sum: '<S49>/Sum2' incorporates:
      //   DiscreteIntegrator: '<S49>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S49>/Discrete-Time Integrator y_dt'
      //   Product: '<S49>/Product2'
      //   Sum: '<S49>/Sum3'

      rtb_y_ai[2] = q_bg_unsigned_idx_2 - (rtDW.DiscreteTimeIntegratory_dt_D_ip
        [2] * q1_q1 + rtDW.DiscreteTimeIntegratory_DSTA_e3[2]);

      // Sum: '<S16>/Add1' incorporates:
      //   DiscreteIntegrator: '<S42>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S49>/Discrete-Time Integrator y'

      rtDW.e_s_g_dt_i[2] = rtDW.DiscreteTimeIntegratory_DSTA_e3[2] -
        rtDW.DiscreteTimeIntegratory_DSTAT_j[2];

      // DiscreteIntegrator: '<S51>/Discrete-Time Integrator y'
      if (rtDW.DiscreteTimeIntegratory_IC_LO_l != 0) {
        rtDW.DiscreteTimeIntegratory_DSTAT_k[0] = y_m[0];
        rtDW.DiscreteTimeIntegratory_DSTAT_k[1] = y_m[1];
        rtDW.DiscreteTimeIntegratory_DSTAT_k[2] = rtb_Sum2_o3;
      }

      for (i = 0; i < 3; i++) {
        // Sum: '<S51>/Sum2' incorporates:
        //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y'
        //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y_dt'
        //   Product: '<S51>/Product2'
        //   Sum: '<S51>/Sum3'

        rtb_Sum2_ny[i] = y_m[i] - (rtDW.DiscreteTimeIntegratory_dt_DS_c[i] *
          q1_q1 + rtDW.DiscreteTimeIntegratory_DSTAT_k[i]);

        // Sum: '<S16>/Add2' incorporates:
        //   DiscreteIntegrator: '<S40>/Discrete-Time Integrator y'
        //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y'

        rtDW.e_s_g_dt2_k[i] = rtDW.DiscreteTimeIntegratory_DSTAT_k[i] -
          rtDW.DiscreteTimeIntegratory_DSTAT_n[i];
        rtb_y_k[i] = rtDW.DiscreteTimeIntegratory_DSTAT_b[i + 6] * rtb_Sum2_o3 +
          (rtDW.DiscreteTimeIntegratory_DSTAT_b[i + 3] * y_m[1] +
           rtDW.DiscreteTimeIntegratory_DSTAT_b[i] * y_m[0]);
      }

      for (stage_app_2 = 0; stage_app_2 < 3; stage_app_2++) {
        // Product: '<S16>/Matrix Multiply' incorporates:
        //   DiscreteIntegrator: '<S39>/Discrete-Time Integrator y'
        //   Product: '<S16>/Matrix Multiply1'
        //   Product: '<S16>/Matrix Multiply2'

        q_bg_unsigned_idx_3 = rtDW.DiscreteTimeIntegratory_DSTAT_b[stage_app_2 +
          3];
        Phi_i = rtDW.DiscreteTimeIntegratory_DSTAT_b[stage_app_2 + 6];
        expl_temp[stage_app_2] = Phi_i * rtDW.e_s_g_e[2] + (q_bg_unsigned_idx_3 *
          rtDW.e_s_g_e[1] + rtDW.DiscreteTimeIntegratory_DSTAT_b[stage_app_2] *
          rtDW.e_s_g_e[0]);
        dir_next_wp_0[stage_app_2] = Phi_i * rtDW.e_s_g_dt_i[2] +
          (q_bg_unsigned_idx_3 * rtDW.e_s_g_dt_i[1] +
           rtDW.DiscreteTimeIntegratory_DSTAT_b[stage_app_2] * rtDW.e_s_g_dt_i[0]);
        p_match_2[stage_app_2] = Phi_i * rtDW.e_s_g_dt2_k[2] +
          (q_bg_unsigned_idx_3 * rtDW.e_s_g_dt2_k[1] +
           rtDW.DiscreteTimeIntegratory_DSTAT_b[stage_app_2] * rtDW.e_s_g_dt2_k
           [0]);
      }

      // Sum: '<S16>/Add5' incorporates:
      //   DiscreteIntegrator: '<S39>/Discrete-Time Integrator y'
      //   Gain: '<S16>/Gain'
      //   Gain: '<S16>/Gain1'
      //   Gain: '<S16>/Gain2'
      //   Product: '<S16>/Matrix Multiply'
      //   Product: '<S16>/Matrix Multiply1'
      //   Product: '<S16>/Matrix Multiply2'
      //   Product: '<S16>/Matrix Multiply3'
      //   Sum: '<S16>/Add3'

      rtDW.nu[0] = ((rtP.lindi.psc.k.pos * expl_temp[1] + rtP.lindi.psc.k.vel *
                     dir_next_wp_0[1]) + rtP.lindi.psc.k.acc * p_match_2[1]) +
        rtb_y_k[1];
      rtDW.nu[1] = ((rtP.lindi.psc.k.pos * expl_temp[2] + rtP.lindi.psc.k.vel *
                     dir_next_wp_0[2]) + rtP.lindi.psc.k.acc * p_match_2[2]) +
        rtb_y_k[2];

      // Update for DiscreteIntegrator: '<S50>/Discrete-Time Integrator y'
      rtDW.DiscreteTimeIntegratory_IC_LO_d = 0U;

      // Update for DiscreteIntegrator: '<S49>/Discrete-Time Integrator y'
      rtDW.DiscreteTimeIntegratory_IC_LO_p = 0U;

      // Update for DiscreteIntegrator: '<S51>/Discrete-Time Integrator y'
      rtDW.DiscreteTimeIntegratory_IC_LO_l = 0U;

      // SignalConversion: '<S16>/BusConversion_InsertedFor_pos_control_at_inport_0' 
      rtDW.s_g_ref_d[0] = scale;

      // SignalConversion: '<S16>/BusConversion_InsertedFor_pos_control_at_inport_0' incorporates:
      //   DiscreteIntegrator: '<S41>/Discrete-Time Integrator y'

      rtDW.s_g_a[0] = rtDW.DiscreteTimeIntegratory_DSTA_n2[0];

      // Update for DiscreteIntegrator: '<S50>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S50>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_DSTA_nm[0] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_D_jv[0];

      // Update for DiscreteIntegrator: '<S50>/Discrete-Time Integrator y_dt' incorporates:
      //   Product: '<S50>/Product1'
      //   Product: '<S50>/omega^2'

      rtDW.DiscreteTimeIntegratory_dt_D_jv[0] += rtb_y_pd[0] *
        q_bg_unsigned_idx_0 * 0.0025F;

      // Update for DiscreteIntegrator: '<S49>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S49>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_DSTA_e3[0] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_D_ip[0];

      // Update for DiscreteIntegrator: '<S49>/Discrete-Time Integrator y_dt' incorporates:
      //   Product: '<S49>/Product1'

      rtDW.DiscreteTimeIntegratory_dt_D_ip[0] += rtb_y_ai[0] *
        q_bg_unsigned_idx_0 * 0.0025F;

      // Update for DiscreteIntegrator: '<S51>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_DSTAT_k[0] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_DS_c[0];

      // Update for DiscreteIntegrator: '<S51>/Discrete-Time Integrator y_dt' incorporates:
      //   Product: '<S51>/Product1'

      rtDW.DiscreteTimeIntegratory_dt_DS_c[0] += rtb_Sum2_ny[0] *
        q_bg_unsigned_idx_0 * 0.0025F;

      // SignalConversion: '<S16>/BusConversion_InsertedFor_pos_control_at_inport_0' 
      rtDW.s_g_ref_d[1] = q0_q3;

      // SignalConversion: '<S16>/BusConversion_InsertedFor_pos_control_at_inport_0' incorporates:
      //   DiscreteIntegrator: '<S41>/Discrete-Time Integrator y'

      rtDW.s_g_a[1] = rtDW.DiscreteTimeIntegratory_DSTA_n2[1];

      // Update for DiscreteIntegrator: '<S50>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S50>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_DSTA_nm[1] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_D_jv[1];

      // Update for DiscreteIntegrator: '<S50>/Discrete-Time Integrator y_dt' incorporates:
      //   Product: '<S50>/Product1'
      //   Product: '<S50>/omega^2'

      rtDW.DiscreteTimeIntegratory_dt_D_jv[1] += rtb_y_pd[1] *
        q_bg_unsigned_idx_0 * 0.0025F;

      // Update for DiscreteIntegrator: '<S49>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S49>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_DSTA_e3[1] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_D_ip[1];

      // Update for DiscreteIntegrator: '<S49>/Discrete-Time Integrator y_dt' incorporates:
      //   Product: '<S49>/Product1'

      rtDW.DiscreteTimeIntegratory_dt_D_ip[1] += rtb_y_ai[1] *
        q_bg_unsigned_idx_0 * 0.0025F;

      // Update for DiscreteIntegrator: '<S51>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_DSTAT_k[1] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_DS_c[1];

      // Update for DiscreteIntegrator: '<S51>/Discrete-Time Integrator y_dt' incorporates:
      //   Product: '<S51>/Product1'

      rtDW.DiscreteTimeIntegratory_dt_DS_c[1] += rtb_Sum2_ny[1] *
        q_bg_unsigned_idx_0 * 0.0025F;

      // SignalConversion: '<S16>/BusConversion_InsertedFor_pos_control_at_inport_0' 
      rtDW.s_g_ref_d[2] = q2_q3;

      // SignalConversion: '<S16>/BusConversion_InsertedFor_pos_control_at_inport_0' incorporates:
      //   DiscreteIntegrator: '<S41>/Discrete-Time Integrator y'

      rtDW.s_g_a[2] = rtDW.DiscreteTimeIntegratory_DSTA_n2[2];

      // Update for DiscreteIntegrator: '<S50>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S50>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_DSTA_nm[2] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_D_jv[2];

      // Update for DiscreteIntegrator: '<S50>/Discrete-Time Integrator y_dt' incorporates:
      //   Product: '<S50>/Product1'
      //   Product: '<S50>/omega^2'

      rtDW.DiscreteTimeIntegratory_dt_D_jv[2] += rtb_y_pd[2] *
        q_bg_unsigned_idx_0 * 0.0025F;

      // Update for DiscreteIntegrator: '<S49>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S49>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_DSTA_e3[2] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_D_ip[2];

      // Update for DiscreteIntegrator: '<S49>/Discrete-Time Integrator y_dt' incorporates:
      //   Product: '<S49>/Product1'

      rtDW.DiscreteTimeIntegratory_dt_D_ip[2] += rtb_y_ai[2] *
        q_bg_unsigned_idx_0 * 0.0025F;

      // Update for DiscreteIntegrator: '<S51>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S51>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_DSTAT_k[2] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_DS_c[2];

      // Update for DiscreteIntegrator: '<S51>/Discrete-Time Integrator y_dt' incorporates:
      //   Product: '<S51>/Product1'

      rtDW.DiscreteTimeIntegratory_dt_DS_c[2] += rtb_Sum2_ny[2] *
        q_bg_unsigned_idx_0 * 0.0025F;
    } else {
      if (rtDW.FlightPathSmoothing_MODE) {
        rtDW.FlightPathSmoothing_MODE = false;
      }

      if (rtDW.NDIPositionController_MODE) {
        rtDW.NDIPositionController_MODE = false;
      }
    }

    // End of Outputs for SubSystem: '<S5>/Waypoint Navigation'
    // End of Outputs for SubSystem: '<S5>/Flight Path Smoothing'
    // End of Outputs for SubSystem: '<S5>/NDI Position Controller'

    // DiscreteIntegrator: '<S38>/Discrete-Time Integrator' incorporates:
    //   DiscreteIntegrator: '<S28>/Discrete-Time Integrator y'

    q1_q1 = rtDW.DiscreteTimeIntegrator_DSTATE;

    // Sum: '<S38>/Add1' incorporates:
    //   DiscreteIntegrator: '<S38>/Discrete-Time Integrator'
    //   UnitDelay: '<S38>/Unit Delay'

    q2_q2 = rtDW.DiscreteTimeIntegrator_DSTATE - rtDW.UnitDelay_DSTATE;

    // MATLAB Function: '<S38>/PT2 discrete ode4' incorporates:
    //   Constant: '<S38>/Constant1'
    //   Constant: '<S38>/Constant2'
    //   Delay: '<S38>/Delay'
    //   Inport: '<Root>/measure'

    // :  k1 = f(y_0,u,omega,d);
    // :  len_u = length(u);
    // :  y_dt = zeros(2,len_u,superiorfloat(u));
    // :  for i = 1:len_u
    for (i = 0; i < 3; i++) {
      // :  y_dt(:,i) = [ -2*d*omega, -omega^2; 1, 0 ] * y_0(:,i) + [omega^2;0] * u(i); 
      p2 = i << 1;
      q_bg_unsigned_idx_3 = rtP.lindi.sflt.omega * rtP.lindi.sflt.omega;
      rtb_y_n[p2] = (-2.0F * rtP.lindi.sflt.d * rtP.lindi.sflt.omega *
                     rtDW.Delay_DSTATE[p2] + rtDW.Delay_DSTATE[p2 + 1] *
                     -q_bg_unsigned_idx_3) + q_bg_unsigned_idx_3 *
        rtU.measure.omega_Kb[i];
      rtb_y_n[1 + p2] = rtDW.Delay_DSTATE[i << 1];
    }

    // :  k2 = f(y_0+0.5*h*k1,u,omega,d);
    q1_q3 = 0.5F * q2_q2;
    for (stage_app_2 = 0; stage_app_2 < 6; stage_app_2++) {
      y_0[stage_app_2] = q1_q3 * rtb_y_n[stage_app_2] +
        rtDW.Delay_DSTATE[stage_app_2];
    }

    // :  len_u = length(u);
    // :  y_dt = zeros(2,len_u,superiorfloat(u));
    // :  for i = 1:len_u
    for (i = 0; i < 3; i++) {
      // :  y_dt(:,i) = [ -2*d*omega, -omega^2; 1, 0 ] * y_0(:,i) + [omega^2;0] * u(i); 
      k2[i << 1] = (-2.0F * rtP.lindi.sflt.d * rtP.lindi.sflt.omega * y_0[i << 1]
                    + y_0[(i << 1) + 1] * -(rtP.lindi.sflt.omega *
        rtP.lindi.sflt.omega)) + rtP.lindi.sflt.omega * rtP.lindi.sflt.omega *
        rtU.measure.omega_Kb[i];
      k2[1 + (i << 1)] = y_0[i << 1];
    }

    // :  k3 = f(y_0+0.5*h*k2,u,omega,d);
    scale = 0.5F * q2_q2;
    for (stage_app_2 = 0; stage_app_2 < 6; stage_app_2++) {
      y_0[stage_app_2] = scale * k2[stage_app_2] + rtDW.Delay_DSTATE[stage_app_2];
    }

    // :  len_u = length(u);
    // :  y_dt = zeros(2,len_u,superiorfloat(u));
    // :  for i = 1:len_u
    for (i = 0; i < 3; i++) {
      // :  y_dt(:,i) = [ -2*d*omega, -omega^2; 1, 0 ] * y_0(:,i) + [omega^2;0] * u(i); 
      k3[i << 1] = (-2.0F * rtP.lindi.sflt.d * rtP.lindi.sflt.omega * y_0[i << 1]
                    + y_0[(i << 1) + 1] * -(rtP.lindi.sflt.omega *
        rtP.lindi.sflt.omega)) + rtP.lindi.sflt.omega * rtP.lindi.sflt.omega *
        rtU.measure.omega_Kb[i];
      k3[1 + (i << 1)] = y_0[i << 1];
    }

    // :  k4 = f(y_0+h*k3,u,omega,d);
    for (stage_app_2 = 0; stage_app_2 < 6; stage_app_2++) {
      y_0[stage_app_2] = q2_q2 * k3[stage_app_2] + rtDW.Delay_DSTATE[stage_app_2];
    }

    // :  len_u = length(u);
    // :  y_dt = zeros(2,len_u,superiorfloat(u));
    // :  for i = 1:len_u
    for (i = 0; i < 3; i++) {
      // :  y_dt(:,i) = [ -2*d*omega, -omega^2; 1, 0 ] * y_0(:,i) + [omega^2;0] * u(i); 
      k4[i << 1] = (-2.0F * rtP.lindi.sflt.d * rtP.lindi.sflt.omega * y_0[i << 1]
                    + y_0[(i << 1) + 1] * -(rtP.lindi.sflt.omega *
        rtP.lindi.sflt.omega)) + rtP.lindi.sflt.omega * rtP.lindi.sflt.omega *
        rtU.measure.omega_Kb[i];
      k4[1 + (i << 1)] = y_0[i << 1];
    }

    // :  y_n = y_0 + 1/6*h*(k1+2*k2+2*k3+k4);
    absxk = 0.166666672F * q2_q2;
    for (stage_app_2 = 0; stage_app_2 < 6; stage_app_2++) {
      rtb_y_n[stage_app_2] = (((2.0F * k2[stage_app_2] + rtb_y_n[stage_app_2]) +
        2.0F * k3[stage_app_2]) + k4[stage_app_2]) * absxk +
        rtDW.Delay_DSTATE[stage_app_2];
    }

    // End of MATLAB Function: '<S38>/PT2 discrete ode4'

    // MATLAB Function: '<S38>/PT2 split'
    PT2split(rtb_y_n, rtb_y_k, rtb_y_pd);

    // DiscreteIntegrator: '<S37>/Discrete-Time Integrator'
    q2_q2 = rtDW.DiscreteTimeIntegrator_DSTATE_c;

    // Sum: '<S37>/Add1' incorporates:
    //   DiscreteIntegrator: '<S37>/Discrete-Time Integrator'
    //   UnitDelay: '<S37>/Unit Delay'

    absxk = rtDW.DiscreteTimeIntegrator_DSTATE_c - rtDW.UnitDelay_DSTATE_a;

    // MATLAB Function: '<S37>/PT2 discrete ode4' incorporates:
    //   Constant: '<S37>/Constant1'
    //   Constant: '<S37>/Constant2'
    //   Delay: '<S37>/Delay'

    // :  k1 = f(y_0,u,omega,d);
    // :  len_u = length(u);
    // :  y_dt = zeros(2,len_u,superiorfloat(u));
    // :  for i = 1:len_u
    for (i = 0; i < 3; i++) {
      // :  y_dt(:,i) = [ -2*d*omega, -omega^2; 1, 0 ] * y_0(:,i) + [omega^2;0] * u(i); 
      rtb_y_n_g[i << 1] = (-2.0F * rtP.lindi.sflt.d * rtP.lindi.sflt.omega *
                           rtDW.Delay_DSTATE_h[i << 1] + rtDW.Delay_DSTATE_h[(i <<
        1) + 1] * -(rtP.lindi.sflt.omega * rtP.lindi.sflt.omega)) +
        rtP.lindi.sflt.omega * rtP.lindi.sflt.omega * rtb_y_k[i];
      rtb_y_n_g[1 + (i << 1)] = rtDW.Delay_DSTATE_h[i << 1];
    }

    // :  k2 = f(y_0+0.5*h*k1,u,omega,d);
    q1_q3 = 0.5F * absxk;
    for (stage_app_2 = 0; stage_app_2 < 6; stage_app_2++) {
      y_0[stage_app_2] = q1_q3 * rtb_y_n_g[stage_app_2] +
        rtDW.Delay_DSTATE_h[stage_app_2];
    }

    // :  len_u = length(u);
    // :  y_dt = zeros(2,len_u,superiorfloat(u));
    // :  for i = 1:len_u
    for (i = 0; i < 3; i++) {
      // :  y_dt(:,i) = [ -2*d*omega, -omega^2; 1, 0 ] * y_0(:,i) + [omega^2;0] * u(i); 
      k2[i << 1] = (-2.0F * rtP.lindi.sflt.d * rtP.lindi.sflt.omega * y_0[i << 1]
                    + y_0[(i << 1) + 1] * -(rtP.lindi.sflt.omega *
        rtP.lindi.sflt.omega)) + rtP.lindi.sflt.omega * rtP.lindi.sflt.omega *
        rtb_y_k[i];
      k2[1 + (i << 1)] = y_0[i << 1];
    }

    // :  k3 = f(y_0+0.5*h*k2,u,omega,d);
    scale = 0.5F * absxk;
    for (stage_app_2 = 0; stage_app_2 < 6; stage_app_2++) {
      y_0[stage_app_2] = scale * k2[stage_app_2] +
        rtDW.Delay_DSTATE_h[stage_app_2];
    }

    // :  len_u = length(u);
    // :  y_dt = zeros(2,len_u,superiorfloat(u));
    // :  for i = 1:len_u
    for (i = 0; i < 3; i++) {
      // :  y_dt(:,i) = [ -2*d*omega, -omega^2; 1, 0 ] * y_0(:,i) + [omega^2;0] * u(i); 
      k3[i << 1] = (-2.0F * rtP.lindi.sflt.d * rtP.lindi.sflt.omega * y_0[i << 1]
                    + y_0[(i << 1) + 1] * -(rtP.lindi.sflt.omega *
        rtP.lindi.sflt.omega)) + rtP.lindi.sflt.omega * rtP.lindi.sflt.omega *
        rtb_y_k[i];
      k3[1 + (i << 1)] = y_0[i << 1];
    }

    // :  k4 = f(y_0+h*k3,u,omega,d);
    for (stage_app_2 = 0; stage_app_2 < 6; stage_app_2++) {
      y_0[stage_app_2] = absxk * k3[stage_app_2] +
        rtDW.Delay_DSTATE_h[stage_app_2];
    }

    // :  len_u = length(u);
    // :  y_dt = zeros(2,len_u,superiorfloat(u));
    // :  for i = 1:len_u
    for (i = 0; i < 3; i++) {
      // :  y_dt(:,i) = [ -2*d*omega, -omega^2; 1, 0 ] * y_0(:,i) + [omega^2;0] * u(i); 
      k4[i << 1] = (-2.0F * rtP.lindi.sflt.d * rtP.lindi.sflt.omega * y_0[i << 1]
                    + y_0[(i << 1) + 1] * -(rtP.lindi.sflt.omega *
        rtP.lindi.sflt.omega)) + rtP.lindi.sflt.omega * rtP.lindi.sflt.omega *
        rtb_y_k[i];
      k4[1 + (i << 1)] = y_0[i << 1];
    }

    // :  y_n = y_0 + 1/6*h*(k1+2*k2+2*k3+k4);
    absxk *= 0.166666672F;
    for (stage_app_2 = 0; stage_app_2 < 6; stage_app_2++) {
      rtb_y_n_g[stage_app_2] = (((2.0F * k2[stage_app_2] + rtb_y_n_g[stage_app_2])
        + 2.0F * k3[stage_app_2]) + k4[stage_app_2]) * absxk +
        rtDW.Delay_DSTATE_h[stage_app_2];
    }

    // End of MATLAB Function: '<S37>/PT2 discrete ode4'

    // MATLAB Function: '<S37>/PT2 split'
    PT2split(rtb_y_n_g, rtb_y_k, rtb_y_pd);

    // Outputs for Enabled SubSystem: '<S5>/Demux' incorporates:
    //   EnablePort: '<S12>/Enable'

    // Outputs for Enabled SubSystem: '<S5>/Outer Loop INDI' incorporates:
    //   EnablePort: '<S17>/Enable'

    if (rtb_Compare_j) {
      if (!rtDW.OuterLoopINDI_MODE) {
        // InitializeConditions for UnitDelay: '<S17>/Unit Delay1'
        rtDW.UnitDelay1_DSTATE_d = 0.0F;

        // InitializeConditions for DiscreteIntegrator: '<S54>/Discrete-Time Integrator' 
        rtDW.DiscreteTimeIntegrator_IC_LOA_l = 1U;
        rtDW.OuterLoopINDI_MODE = true;
      }

      // MATLAB Function: '<S52>/DCM to quaternions1' incorporates:
      //   DiscreteIntegrator: '<S39>/Discrete-Time Integrator y'

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
      scale = ((1.0F + rtDW.DiscreteTimeIntegratory_DSTAT_b[0]) +
               rtDW.DiscreteTimeIntegratory_DSTAT_b[4]) +
        rtDW.DiscreteTimeIntegratory_DSTAT_b[8];

      // 'sqrtReal:27' if numel(y) > 1
      // 'sqrtReal:29' else
      // 'sqrtReal:30' y = max(0,y);
      // 'sqrtReal:32' y = sqrt(y);
      if (0.0F >= scale) {
        scale = 0.0F;
      }

      scale = 0.5F * std::sqrt(scale);

      // 'dcm2Quat:53' q_1 = 1/2 * sqrtReal( 1 + m_11 - m_22 - m_33 );
      absxk = ((1.0F + rtDW.DiscreteTimeIntegratory_DSTAT_b[0]) -
               rtDW.DiscreteTimeIntegratory_DSTAT_b[4]) -
        rtDW.DiscreteTimeIntegratory_DSTAT_b[8];

      // 'sqrtReal:27' if numel(y) > 1
      // 'sqrtReal:29' else
      // 'sqrtReal:30' y = max(0,y);
      // 'sqrtReal:32' y = sqrt(y);
      if (0.0F >= absxk) {
        absxk = 0.0F;
      }

      absxk = 0.5F * std::sqrt(absxk);

      // 'dcm2Quat:54' q_2 = 1/2 * sqrtReal( 1 - m_11 + m_22 - m_33 );
      t = ((1.0F - rtDW.DiscreteTimeIntegratory_DSTAT_b[0]) +
           rtDW.DiscreteTimeIntegratory_DSTAT_b[4]) -
        rtDW.DiscreteTimeIntegratory_DSTAT_b[8];

      // 'sqrtReal:27' if numel(y) > 1
      // 'sqrtReal:29' else
      // 'sqrtReal:30' y = max(0,y);
      // 'sqrtReal:32' y = sqrt(y);
      if (0.0F >= t) {
        t = 0.0F;
      }

      t = 0.5F * std::sqrt(t);

      // 'dcm2Quat:55' q_3 = 1/2 * sqrtReal( 1 - m_11 - m_22 + m_33 );
      q0_q3 = ((1.0F - rtDW.DiscreteTimeIntegratory_DSTAT_b[0]) -
               rtDW.DiscreteTimeIntegratory_DSTAT_b[4]) +
        rtDW.DiscreteTimeIntegratory_DSTAT_b[8];

      // 'sqrtReal:27' if numel(y) > 1
      // 'sqrtReal:29' else
      // 'sqrtReal:30' y = max(0,y);
      // 'sqrtReal:32' y = sqrt(y);
      if (0.0F >= q0_q3) {
        q0_q3 = 0.0F;
      }

      q0_q3 = 0.5F * std::sqrt(q0_q3);

      // 'dcm2Quat:58' q_bg_unsigned = [ q_0; q_1; q_2; q_3 ];
      // 'dcm2Quat:59' [~, idx] = max(q_bg_unsigned);
      q1_q2 = scale;
      i = -1;
      if (scale < absxk) {
        q1_q2 = absxk;
        i = 0;
      }

      if (q1_q2 < t) {
        q1_q2 = t;
        i = 1;
      }

      if (q1_q2 < q0_q3) {
        i = 2;
      }

      // 'dcm2Quat:59' ~
      // 'dcm2Quat:60' idx = idx - 1;
      // 'dcm2Quat:61' switch idx
      switch (i + 1) {
       case 0:
        // 'dcm2Quat:62' case 0
        // 'dcm2Quat:63' q_1 = sign_m23_minus * q_1;
        q_bg_unsigned_idx_3 = rtDW.DiscreteTimeIntegratory_DSTAT_b[7] -
          rtDW.DiscreteTimeIntegratory_DSTAT_b[5];
        if (q_bg_unsigned_idx_3 < 0.0F) {
          q_bg_unsigned_idx_3 = -1.0F;
        } else {
          if (q_bg_unsigned_idx_3 > 0.0F) {
            q_bg_unsigned_idx_3 = 1.0F;
          }
        }

        absxk *= q_bg_unsigned_idx_3;

        // 'dcm2Quat:64' q_2 = sign_m13_minus * q_2;
        q_bg_unsigned_idx_3 = rtDW.DiscreteTimeIntegratory_DSTAT_b[2] -
          rtDW.DiscreteTimeIntegratory_DSTAT_b[6];
        if (q_bg_unsigned_idx_3 < 0.0F) {
          q_bg_unsigned_idx_3 = -1.0F;
        } else {
          if (q_bg_unsigned_idx_3 > 0.0F) {
            q_bg_unsigned_idx_3 = 1.0F;
          }
        }

        t *= q_bg_unsigned_idx_3;

        // 'dcm2Quat:65' q_3 = sign_m12_minus * q_3;
        q_bg_unsigned_idx_3 = rtDW.DiscreteTimeIntegratory_DSTAT_b[3] -
          rtDW.DiscreteTimeIntegratory_DSTAT_b[1];
        if (q_bg_unsigned_idx_3 < 0.0F) {
          q_bg_unsigned_idx_3 = -1.0F;
        } else {
          if (q_bg_unsigned_idx_3 > 0.0F) {
            q_bg_unsigned_idx_3 = 1.0F;
          }
        }

        q0_q3 *= q_bg_unsigned_idx_3;
        break;

       case 1:
        // 'dcm2Quat:66' case 1
        // 'dcm2Quat:67' q_0 = sign_m23_minus * q_0;
        q1_q2 = rtDW.DiscreteTimeIntegratory_DSTAT_b[7] -
          rtDW.DiscreteTimeIntegratory_DSTAT_b[5];
        if (q1_q2 < 0.0F) {
          q1_q2 = -1.0F;
        } else {
          if (q1_q2 > 0.0F) {
            q1_q2 = 1.0F;
          }
        }

        scale *= q1_q2;

        // 'dcm2Quat:68' q_2 = sign_m12_plus * q_2;
        q_bg_unsigned_idx_3 = rtDW.DiscreteTimeIntegratory_DSTAT_b[3] +
          rtDW.DiscreteTimeIntegratory_DSTAT_b[1];
        if (q_bg_unsigned_idx_3 < 0.0F) {
          q_bg_unsigned_idx_3 = -1.0F;
        } else {
          if (q_bg_unsigned_idx_3 > 0.0F) {
            q_bg_unsigned_idx_3 = 1.0F;
          }
        }

        t *= q_bg_unsigned_idx_3;

        // 'dcm2Quat:69' q_3 = sign_m13_plus * q_3;
        q_bg_unsigned_idx_3 = rtDW.DiscreteTimeIntegratory_DSTAT_b[2] +
          rtDW.DiscreteTimeIntegratory_DSTAT_b[6];
        if (q_bg_unsigned_idx_3 < 0.0F) {
          q_bg_unsigned_idx_3 = -1.0F;
        } else {
          if (q_bg_unsigned_idx_3 > 0.0F) {
            q_bg_unsigned_idx_3 = 1.0F;
          }
        }

        q0_q3 *= q_bg_unsigned_idx_3;
        break;

       case 2:
        // 'dcm2Quat:70' case 2
        // 'dcm2Quat:71' q_0 = sign_m13_minus * q_0;
        q1_q2 = rtDW.DiscreteTimeIntegratory_DSTAT_b[2] -
          rtDW.DiscreteTimeIntegratory_DSTAT_b[6];
        if (q1_q2 < 0.0F) {
          q1_q2 = -1.0F;
        } else {
          if (q1_q2 > 0.0F) {
            q1_q2 = 1.0F;
          }
        }

        scale *= q1_q2;

        // 'dcm2Quat:72' q_1 = sign_m12_plus * q_1;
        q1_q2 = rtDW.DiscreteTimeIntegratory_DSTAT_b[3] +
          rtDW.DiscreteTimeIntegratory_DSTAT_b[1];
        if (q1_q2 < 0.0F) {
          q1_q2 = -1.0F;
        } else {
          if (q1_q2 > 0.0F) {
            q1_q2 = 1.0F;
          }
        }

        absxk *= q1_q2;

        // 'dcm2Quat:73' q_3 = sign_m23_plus * q_3;
        q_bg_unsigned_idx_3 = rtDW.DiscreteTimeIntegratory_DSTAT_b[7] +
          rtDW.DiscreteTimeIntegratory_DSTAT_b[5];
        if (q_bg_unsigned_idx_3 < 0.0F) {
          q_bg_unsigned_idx_3 = -1.0F;
        } else {
          if (q_bg_unsigned_idx_3 > 0.0F) {
            q_bg_unsigned_idx_3 = 1.0F;
          }
        }

        q0_q3 *= q_bg_unsigned_idx_3;
        break;

       case 3:
        // 'dcm2Quat:74' case 3
        // 'dcm2Quat:75' q_0 = sign_m12_minus * q_0;
        q1_q2 = rtDW.DiscreteTimeIntegratory_DSTAT_b[3] -
          rtDW.DiscreteTimeIntegratory_DSTAT_b[1];
        if (q1_q2 < 0.0F) {
          q1_q2 = -1.0F;
        } else {
          if (q1_q2 > 0.0F) {
            q1_q2 = 1.0F;
          }
        }

        scale *= q1_q2;

        // 'dcm2Quat:76' q_1 = sign_m13_plus * q_1;
        q1_q2 = rtDW.DiscreteTimeIntegratory_DSTAT_b[2] +
          rtDW.DiscreteTimeIntegratory_DSTAT_b[6];
        if (q1_q2 < 0.0F) {
          q1_q2 = -1.0F;
        } else {
          if (q1_q2 > 0.0F) {
            q1_q2 = 1.0F;
          }
        }

        absxk *= q1_q2;

        // 'dcm2Quat:77' q_2 = sign_m23_plus * q_2;
        q1_q2 = rtDW.DiscreteTimeIntegratory_DSTAT_b[7] +
          rtDW.DiscreteTimeIntegratory_DSTAT_b[5];
        if (q1_q2 < 0.0F) {
          q1_q2 = -1.0F;
        } else {
          if (q1_q2 > 0.0F) {
            q1_q2 = 1.0F;
          }
        }

        t *= q1_q2;
        break;
      }

      // 'dcm2Quat:81' q_bg = [ q_0; q_1; q_2; q_3 ];
      q_bg_unsigned_idx_0 = scale;
      q_bg_unsigned_idx_1 = absxk;
      q_bg_unsigned_idx_2 = t;
      q_bg_unsigned_idx_3 = q0_q3;

      // 'dcm2Quat:84' q_bg = quatNormalize( q_bg );
      // 'quatNormalize:31' q_out = q / max( eps, norm(q, 2) );
      scale = 1.29246971E-26F;
      absxk = std::abs(q_bg_unsigned_idx_0);
      if (absxk > 1.29246971E-26F) {
        q0_q3 = 1.0F;
        scale = absxk;
      } else {
        t = absxk / 1.29246971E-26F;
        q0_q3 = t * t;
      }

      absxk = std::abs(q_bg_unsigned_idx_1);
      if (absxk > scale) {
        t = scale / absxk;
        q0_q3 = q0_q3 * t * t + 1.0F;
        scale = absxk;
      } else {
        t = absxk / scale;
        q0_q3 += t * t;
      }

      absxk = std::abs(q_bg_unsigned_idx_2);
      if (absxk > scale) {
        t = scale / absxk;
        q0_q3 = q0_q3 * t * t + 1.0F;
        scale = absxk;
      } else {
        t = absxk / scale;
        q0_q3 += t * t;
      }

      absxk = std::abs(q_bg_unsigned_idx_3);
      if (absxk > scale) {
        t = scale / absxk;
        q0_q3 = q0_q3 * t * t + 1.0F;
        scale = absxk;
      } else {
        t = absxk / scale;
        q0_q3 += t * t;
      }

      q0_q3 = scale * std::sqrt(q0_q3);
      if (2.22044605E-16F < q0_q3) {
        scale = q0_q3;
      } else {
        scale = 2.22044605E-16F;
      }

      q_bg_unsigned_idx_0 /= scale;
      q_bg_unsigned_idx_1 /= scale;
      q_bg_unsigned_idx_2 /= scale;

      // MATLAB Function: '<S52>/DCM to quaternions1'
      q0_q3 = q_bg_unsigned_idx_3 / scale;

      // MATLAB Function: '<S52>/Quaternions to Euler angles1'
      // :  EulerAngles  = quat2Euler( q_bg );
      // 'quat2Euler:33' q_bg = quatNormalize( q_bg );
      // 'quatNormalize:31' q_out = q / max( eps, norm(q, 2) );
      scale = 1.29246971E-26F;
      absxk = std::abs(q_bg_unsigned_idx_0);
      if (absxk > 1.29246971E-26F) {
        q1_q3 = 1.0F;
        scale = absxk;
      } else {
        t = absxk / 1.29246971E-26F;
        q1_q3 = t * t;
      }

      absxk = std::abs(q_bg_unsigned_idx_1);
      if (absxk > scale) {
        t = scale / absxk;
        q1_q3 = q1_q3 * t * t + 1.0F;
        scale = absxk;
      } else {
        t = absxk / scale;
        q1_q3 += t * t;
      }

      absxk = std::abs(q_bg_unsigned_idx_2);
      if (absxk > scale) {
        t = scale / absxk;
        q1_q3 = q1_q3 * t * t + 1.0F;
        scale = absxk;
      } else {
        t = absxk / scale;
        q1_q3 += t * t;
      }

      absxk = std::abs(q0_q3);
      if (absxk > scale) {
        t = scale / absxk;
        q1_q3 = q1_q3 * t * t + 1.0F;
        scale = absxk;
      } else {
        t = absxk / scale;
        q1_q3 += t * t;
      }

      q1_q3 = scale * std::sqrt(q1_q3);
      if (2.22044605E-16F < q1_q3) {
        scale = q1_q3;
      } else {
        scale = 2.22044605E-16F;
      }

      q_bg_unsigned_idx_0 /= scale;
      q_bg_unsigned_idx_1 /= scale;
      q_bg_unsigned_idx_2 /= scale;

      // MATLAB Function: '<S52>/Quaternions to Euler angles1'
      q0_q3 /= scale;

      // MATLAB Function: '<S17>/Outer Loop INDI' incorporates:
      //   DiscreteIntegrator: '<S39>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S40>/Discrete-Time Integrator y'

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
        expl_temp[stage_app_2] =
          rtDW.DiscreteTimeIntegratory_DSTAT_b[stage_app_2 + 6] *
          rtDW.DiscreteTimeIntegratory_DSTAT_n[2] +
          (rtDW.DiscreteTimeIntegratory_DSTAT_b[stage_app_2 + 3] *
           rtDW.DiscreteTimeIntegratory_DSTAT_n[1] +
           rtDW.DiscreteTimeIntegratory_DSTAT_b[stage_app_2] *
           rtDW.DiscreteTimeIntegratory_DSTAT_n[0]);
      }

      a_Kb_yz[0] = expl_temp[1];
      a_Kb_yz[1] = expl_temp[2];

      // 'indiPlaneAcc2PhiQR:44' g_g = [0;0;9.81];
      // 'indiPlaneAcc2PhiQR:46' g_b = M_bg*g_g;
      // 'indiPlaneAcc2PhiQR:47' g_b_yz = g_b(2:3);
      for (stage_app_2 = 0; stage_app_2 < 3; stage_app_2++) {
        dir_next_wp_0[stage_app_2] =
          rtDW.DiscreteTimeIntegratory_DSTAT_b[stage_app_2 + 6] * 9.81F;
      }

      g_b_yz[0] = dir_next_wp_0[1];
      g_b_yz[1] = dir_next_wp_0[2];

      // 'indiPlaneAcc2PhiQR:49' a_g = a_Kg - g_g;
      // 'indiPlaneAcc2PhiQR:52' a_b = M_bg*a_g;
      // 'indiPlaneAcc2PhiQR:53' a_b_yz = a_b(2:3);
      // 'indiPlaneAcc2PhiQR:56' a_abs = norm(a_b_yz);
      for (stage_app_2 = 0; stage_app_2 < 3; stage_app_2++) {
        p_match_2[stage_app_2] =
          rtDW.DiscreteTimeIntegratory_DSTAT_b[stage_app_2 + 6] *
          (rtDW.DiscreteTimeIntegratory_DSTAT_n[2] - 9.81F) +
          (rtDW.DiscreteTimeIntegratory_DSTAT_b[stage_app_2 + 3] *
           rtDW.DiscreteTimeIntegratory_DSTAT_n[1] +
           rtDW.DiscreteTimeIntegratory_DSTAT_b[stage_app_2] *
           rtDW.DiscreteTimeIntegratory_DSTAT_n[0]);
      }

      tmp_0[0] = p_match_2[1];
      tmp_0[1] = p_match_2[2];
      q1_q2 = norm(tmp_0);

      // 'indiPlaneAcc2PhiQR:59' a_b_yz_des = nu_a_Kb_yz - g_b_yz;
      // 'indiPlaneAcc2PhiQR:61' a_des_abs = norm(a_b_yz_des);
      tmp_0[0] = rtDW.nu[0] - dir_next_wp_0[1];
      tmp_0[1] = rtDW.nu[1] - dir_next_wp_0[2];
      q1_q3 = norm(tmp_0);

      // 'indiPlaneAcc2PhiQR:64' if method == 1
      // 'indiPlaneAcc2PhiQR:66' g_abs = norm(g_b_yz);
      q2_q3 = norm(g_b_yz);

      // 'indiPlaneAcc2PhiQR:68' nu_abs = norm(nu_a_Kb_yz);
      scale = norm(rtDW.nu);

      // 'indiPlaneAcc2PhiQR:70' a_K_abs = norm(a_Kb_yz);
      q_bg_unsigned_idx_3 = norm(a_Kb_yz);

      // 'indiPlaneAcc2PhiQR:73' Phi_i = acosReal( divideFinite( g_abs^2 + a_abs^2 - a_K_abs^2, 2*g_abs*a_abs ) ); 
      Phi_i = 2.0F * q2_q3 * q1_q2;

      // 'divideFinite:29' if numel(B)>1
      // 'divideFinite:31' else
      // 'divideFinite:32' if abs(B)<eps
      if (std::abs(Phi_i) < 2.22044605E-16F) {
        // 'divideFinite:33' B(:) = eps;
        Phi_i = 2.22044605E-16F;
      }

      // 'divideFinite:36' C = A ./ B;
      absxk = q2_q3 * q2_q3;
      Phi_i = ((absxk + q1_q2 * q1_q2) - q_bg_unsigned_idx_3 *
               q_bg_unsigned_idx_3) / Phi_i;

      // 'acosReal:28' if numel(y) > 1
      // 'acosReal:31' else
      // 'acosReal:32' y = max(-1,min(1,y));
      if (1.0F <= Phi_i) {
        Phi_i = 1.0F;
      }

      // 'acosReal:34' y = acos(y);
      // 'indiPlaneAcc2PhiQR:75' forward = M_bg'*[1;0;0];
      for (stage_app_2 = 0; stage_app_2 < 3; stage_app_2++) {
        rtb_y_ai[stage_app_2] = rtDW.DiscreteTimeIntegratory_DSTAT_b[3 *
          stage_app_2];
      }

      // 'indiPlaneAcc2PhiQR:76' cross_i_g = cross(forward,a_Kg);
      // 'indiPlaneAcc2PhiQR:77' Phi_i_sign = sign(cross_i_g(3));
      // 'indiPlaneAcc2PhiQR:78' Phi_i = Phi_i_sign*Phi_i;
      q1_q2 = rtb_y_ai[0] * rtDW.DiscreteTimeIntegratory_DSTAT_n[1] - rtb_y_ai[1]
        * rtDW.DiscreteTimeIntegratory_DSTAT_n[0];

      // 'indiPlaneAcc2PhiQR:81' Phi_des = acosReal( divideFinite( g_abs^2 + a_des_abs^2 - nu_abs^2, 2*g_abs*a_des_abs ) ); 
      q_bg_unsigned_idx_3 = 2.0F * q2_q3 * q1_q3;

      // 'divideFinite:29' if numel(B)>1
      // 'divideFinite:31' else
      // 'divideFinite:32' if abs(B)<eps
      if (std::abs(q_bg_unsigned_idx_3) < 2.22044605E-16F) {
        // 'divideFinite:33' B(:) = eps;
        q_bg_unsigned_idx_3 = 2.22044605E-16F;
      }

      // 'divideFinite:36' C = A ./ B;
      q2_q3 = ((absxk + q1_q3 * q1_q3) - scale * scale) / q_bg_unsigned_idx_3;

      // 'acosReal:28' if numel(y) > 1
      // 'acosReal:31' else
      // 'acosReal:32' y = max(-1,min(1,y));
      if (1.0F <= q2_q3) {
        q2_q3 = 1.0F;
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
      scale = q0_q0;

      // 'divideFinite:29' if numel(B)>1
      // 'divideFinite:31' else
      // 'divideFinite:32' if abs(B)<eps
      q_bg_unsigned_idx_3 = std::abs(q0_q0);
      if (q_bg_unsigned_idx_3 < 2.22044605E-16F) {
        // 'divideFinite:33' B(:) = eps;
        scale = 2.22044605E-16F;
      }

      // 'divideFinite:36' C = A ./ B;
      a_Kb_yz[0] = expl_temp[1] / scale;

      // MATLAB Function: '<S17>/Outer Loop INDI' incorporates:
      //   UnitDelay: '<S17>/Unit Delay1'

      q1_q3 = expl_temp[2] / scale;

      // 'indiPlaneAcc2PhiQR:117' q_i = -omega_i(2);
      // 'indiPlaneAcc2PhiQR:118' r_i = omega_i(1);
      // 'indiPlaneAcc2PhiQR:121' omega_des = divideFinite( nu_a_Kf_yz, V );
      scale = q0_q0;

      // 'divideFinite:29' if numel(B)>1
      // 'divideFinite:31' else
      // 'divideFinite:32' if abs(B)<eps
      if (q_bg_unsigned_idx_3 < 2.22044605E-16F) {
        // 'divideFinite:33' B(:) = eps;
        scale = 2.22044605E-16F;
      }

      // 'divideFinite:36' C = A ./ B;
      q_bg_unsigned_idx_3 = std::cos(rtDW.UnitDelay1_DSTATE_d);
      absxk = std::sin(rtDW.UnitDelay1_DSTATE_d);

      // 'indiPlaneAcc2PhiQR:122' q_des = -omega_des(2);
      // 'indiPlaneAcc2PhiQR:123' r_des = omega_des(1);
      // 'indiPlaneAcc2PhiQR:126' Delta_q = q_des - q_i;
      // 'indiPlaneAcc2PhiQR:127' Delta_r = r_des - r_i;
      t = dir_next_wp_0[1] * rtDW.nu[1] - dir_next_wp_0[2] * rtDW.nu[0];
      if (t < 0.0F) {
        t = -1.0F;
      } else {
        if (t > 0.0F) {
          t = 1.0F;
        }
      }

      if (-1.0F >= q2_q3) {
        q2_q3 = -1.0F;
      }

      if (q1_q2 < 0.0F) {
        q1_q2 = -1.0F;
      } else {
        if (q1_q2 > 0.0F) {
          q1_q2 = 1.0F;
        }
      }

      if (-1.0F >= Phi_i) {
        Phi_i = -1.0F;
      }

      q2_q3 = -t * std::acos(q2_q3) - q1_q2 * std::acos(Phi_i);

      // DiscreteIntegrator: '<S54>/Discrete-Time Integrator'
      if (rtDW.DiscreteTimeIntegrator_IC_LOA_l != 0) {
        rtDW.DiscreteTimeIntegrator_DSTATE_f = q2_q3;
      }

      // Update for UnitDelay: '<S17>/Unit Delay1' incorporates:
      //   DiscreteIntegrator: '<S54>/Discrete-Time Integrator'

      rtDW.UnitDelay1_DSTATE_d = rtDW.DiscreteTimeIntegrator_DSTATE_f;

      // Update for DiscreteIntegrator: '<S54>/Discrete-Time Integrator' incorporates:
      //   Constant: '<S54>/T'
      //   Product: '<S54>/Divide'
      //   Sum: '<S54>/Sum2'

      rtDW.DiscreteTimeIntegrator_IC_LOA_l = 0U;
      rtDW.DiscreteTimeIntegrator_DSTATE_f += (q2_q3 -
        rtDW.DiscreteTimeIntegrator_DSTATE_f) / (1.0F / rtP.lindi.atc.rm.rfreq) *
        0.0025F;

      // SignalConversion: '<S12>/OutportBufferForPhi_d' incorporates:
      //   MATLAB Function: '<S52>/Quaternions to Euler angles1'
      //   Sum: '<S17>/Add7'

      rtDW.Merge2 = std::atan2((q_bg_unsigned_idx_2 * q0_q3 +
        q_bg_unsigned_idx_0 * q_bg_unsigned_idx_1) * 2.0F, ((q_bg_unsigned_idx_0
        * q_bg_unsigned_idx_0 - q_bg_unsigned_idx_1 * q_bg_unsigned_idx_1) -
        q_bg_unsigned_idx_2 * q_bg_unsigned_idx_2) + q0_q3 * q0_q3) + q2_q3;

      // SignalConversion: '<S12>/OutportBufferForq_d' incorporates:
      //   MATLAB Function: '<S17>/Outer Loop INDI'
      //   Sum: '<S17>/Add8'

      rtDW.Merge = (-((-absxk * rtDW.nu[0] + q_bg_unsigned_idx_3 * rtDW.nu[1]) /
                      scale) - (-q1_q3)) + rtb_y_k[1];

      // SignalConversion: '<S12>/OutportBufferForr_d' incorporates:
      //   MATLAB Function: '<S17>/Outer Loop INDI'
      //   Sum: '<S17>/Add9'

      rtDW.Merge1 = ((q_bg_unsigned_idx_3 * rtDW.nu[0] + absxk * rtDW.nu[1]) /
                     scale - a_Kb_yz[0]) + rtb_y_k[2];
    } else {
      if (rtDW.OuterLoopINDI_MODE) {
        rtDW.OuterLoopINDI_MODE = false;
      }
    }

    // End of Outputs for SubSystem: '<S5>/Outer Loop INDI'
    // End of Outputs for SubSystem: '<S5>/Demux'

    // Outputs for Enabled SubSystem: '<S5>/Cmd 2 Roll Angle' incorporates:
    //   EnablePort: '<S10>/Enable'

    if (rtb_Compare > 0) {
      // Gain: '<S10>/Gain' incorporates:
      //   Inport: '<Root>/cmd'

      rtDW.Merge2 = rtP.lindi.atc.rm.rangmax * 3.14159274F / 180.0F *
        rtU.cmd.roll;
    }

    // End of Outputs for SubSystem: '<S5>/Cmd 2 Roll Angle'

    // Saturate: '<S21>/Saturation' incorporates:
    //   MATLAB Function: '<S17>/Outer Loop INDI'
    //   MATLAB Function: '<S52>/DCM to quaternions1'
    //   MATLAB Function: '<S52>/Quaternions to Euler angles1'

    scale = -rtP.lindi.atc.rm.rangmax * 3.14159274F / 180.0F;
    q_bg_unsigned_idx_3 = rtP.lindi.atc.rm.rangmax * 3.14159274F / 180.0F;
    if (rtDW.Merge2 > q_bg_unsigned_idx_3) {
      scale = q_bg_unsigned_idx_3;
    } else {
      if (rtDW.Merge2 >= scale) {
        scale = rtDW.Merge2;
      }
    }

    // End of Saturate: '<S21>/Saturation'

    // MATLAB Function: '<S21>/Avoid Angle Steps' incorporates:
    //   UnitDelay: '<S21>/Unit Delay'

    // :  Delta_angle = angle - angle_last;
    absxk = scale - rtDW.UnitDelay_DSTATE_f;

    // :  if Delta_angle > deg2rad(180)
    // 'deg2rad:11' angle_rad = angle_deg * pi/180;
    if (absxk > 3.1415926535897931) {
      // :  angle_cum = angle - deg2rad(360);
      // 'deg2rad:11' angle_rad = angle_deg * pi/180;
      scale -= 6.28318548F;
    } else {
      // 'deg2rad:11' angle_rad = angle_deg * pi/180;
      if (absxk < -3.1415926535897931) {
        // :  elseif Delta_angle < -deg2rad(180)
        // :  angle_cum = angle + deg2rad(360);
        // 'deg2rad:11' angle_rad = angle_deg * pi/180;
        scale += 6.28318548F;
      } else {
        // :  else
        // :  angle_cum = angle_last + Delta_angle;
        scale = rtDW.UnitDelay_DSTATE_f + absxk;
      }
    }

    // End of MATLAB Function: '<S21>/Avoid Angle Steps'

    // Outputs for Enabled SubSystem: '<S5>/Turn Coordination' incorporates:
    //   EnablePort: '<S24>/Enable'

    if (rtb_Compare > 0) {
      // MATLAB Function: '<S24>/Turn Coordination'
      absxk = scale;
      t = q0_q0;

      // :  if abs(Phi) > 0.8*pi/2
      if (std::abs(scale) > 1.2566370614359172) {
        // :  Phi = sign(Phi)*0.8*pi/2;
        if (scale < 0.0F) {
          q_bg_unsigned_idx_3 = -1.0F;
        } else if (scale > 0.0F) {
          q_bg_unsigned_idx_3 = 1.0F;
        } else {
          q_bg_unsigned_idx_3 = scale;
        }

        absxk = q_bg_unsigned_idx_3 * 0.8F * 3.14159274F / 2.0F;
      }

      // :  a = 9.81 * tan(Phi);
      // :  if V < 1
      if (q0_q0 < 1.0F) {
        // :  V(:) = 1;
        t = 1.0F;
      }

      // :  omega = a/V;
      t = 9.81F * std::tan(absxk) / t;

      // :  q = omega * sin(Phi);
      rtDW.q = t * std::sin(absxk);

      // Sum: '<S24>/Add' incorporates:
      //   Gain: '<S5>/Cmd 2 Yaw Rate'
      //   Inport: '<Root>/cmd'
      //   MATLAB Function: '<S24>/Turn Coordination'

      // :  r = omega * cos(Phi);
      rtDW.Merge1 = t * std::cos(absxk) + rtP.lindi.atc.rm.ydecaytc *
        rtU.cmd.yaw;
    }

    // End of Outputs for SubSystem: '<S5>/Turn Coordination'

    // MATLAB Function: '<S15>/Rotations matrix to Euler angles' incorporates:
    //   DiscreteIntegrator: '<S39>/Discrete-Time Integrator y'

    // :  EulerAngles  = dcm2Euler( M_bg );
    // 'dcm2Euler:30' Phi = atan2( M_bg(2,3), M_bg(3,3) );
    // 'dcm2Euler:31' Theta = -asinReal( M_bg(1,3) );
    // 'asinReal:28' if numel(y)>1
    // 'asinReal:31' else
    // 'asinReal:32' y = max(-1,min(1,y));
    if (1.0F > rtDW.DiscreteTimeIntegratory_DSTAT_b[6]) {
      Phi_i = rtDW.DiscreteTimeIntegratory_DSTAT_b[6];
    } else {
      Phi_i = 1.0F;
    }

    // 'asinReal:34' y = asin(y);
    // 'dcm2Euler:32' Psi = atan2( M_bg(1,2), M_bg(1,1) );
    // 'dcm2Euler:35' EulerAngles = [ Phi; Theta; Psi ];
    q_bg_unsigned_idx_3 = std::atan2(rtDW.DiscreteTimeIntegratory_DSTAT_b[7],
      rtDW.DiscreteTimeIntegratory_DSTAT_b[8]);
    if (-1.0F >= Phi_i) {
      Phi_i = -1.0F;
    }

    Phi_i = std::asin(Phi_i);

    // Outputs for Enabled SubSystem: '<S5>/add' incorporates:
    //   EnablePort: '<S27>/Enable'

    // Outputs for Enabled SubSystem: '<S5>/Pitch Angle Controller' incorporates:
    //   EnablePort: '<S18>/Enable'

    if (rtb_Compare > 0) {
      if (!rtDW.PitchAngleController_MODE) {
        // InitializeConditions for DiscreteIntegrator: '<S57>/Discrete-Time Integrator' 
        rtDW.DiscreteTimeIntegrator_IC_LOA_m = 1U;

        // InitializeConditions for DiscreteIntegrator: '<S58>/Discrete-Time Integrator' 
        rtDW.DiscreteTimeIntegrator_IC_LO_mz = 1U;

        // InitializeConditions for DiscreteIntegrator: '<S59>/Discrete-Time Integrator y' 
        rtDW.DiscreteTimeIntegratory_IC_L_id = 1U;

        // InitializeConditions for DiscreteIntegrator: '<S59>/Discrete-Time Integrator y_dt' 
        rtDW.DiscreteTimeIntegratory_dt_DS_m = 0.0F;
        rtDW.PitchAngleController_MODE = true;
      }

      // Gain: '<S18>/cmd 2 angle' incorporates:
      //   Inport: '<Root>/cmd'

      absxk = rtP.lindi.atc.rm.pangmax * 3.14159274F / 180.0F * rtU.cmd.pitch;

      // DiscreteIntegrator: '<S57>/Discrete-Time Integrator'
      if (rtDW.DiscreteTimeIntegrator_IC_LOA_m != 0) {
        rtDW.DiscreteTimeIntegrator_DSTATE_p = absxk;
      }

      // Saturate: '<S57>/Saturation' incorporates:
      //   Constant: '<S57>/T'
      //   DiscreteIntegrator: '<S57>/Discrete-Time Integrator'
      //   Product: '<S57>/Divide'
      //   Sum: '<S57>/Sum2'

      q0_q3 = (absxk - rtDW.DiscreteTimeIntegrator_DSTATE_p) / (1.0F /
        rtP.lindi.atc.rm.pfreq);

      // DiscreteIntegrator: '<S58>/Discrete-Time Integrator' incorporates:
      //   DiscreteIntegrator: '<S57>/Discrete-Time Integrator'

      if (rtDW.DiscreteTimeIntegrator_IC_LO_mz != 0) {
        rtDW.DiscreteTimeIntegrator_DSTATE_a =
          rtDW.DiscreteTimeIntegrator_DSTATE_p;
      }

      // Sum: '<S58>/Sum2' incorporates:
      //   DiscreteIntegrator: '<S57>/Discrete-Time Integrator'
      //   DiscreteIntegrator: '<S58>/Discrete-Time Integrator'

      absxk = rtDW.DiscreteTimeIntegrator_DSTATE_p -
        rtDW.DiscreteTimeIntegrator_DSTATE_a;

      // DiscreteIntegrator: '<S59>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S58>/Discrete-Time Integrator'

      if (rtDW.DiscreteTimeIntegratory_IC_L_id != 0) {
        rtDW.DiscreteTimeIntegratory_DSTA_pe =
          rtDW.DiscreteTimeIntegrator_DSTATE_a;
      }

      q1_q2 = rtDW.DiscreteTimeIntegratory_DSTA_pe;

      // Sum: '<S18>/Add3' incorporates:
      //   DiscreteIntegrator: '<S59>/Discrete-Time Integrator y'
      //   MATLAB Function: '<S15>/Rotations matrix to Euler angles'

      q1_q3 = rtDW.DiscreteTimeIntegratory_DSTA_pe - (-Phi_i);

      // Gain: '<S18>/Gain'
      rtDW.nu_q_dt_ptchcntrl = rtP.lindi.atc.k.pang * q1_q3;

      // Product: '<S59>/Divide' incorporates:
      //   Constant: '<S59>/omega'
      //   Product: '<S59>/omega^2'

      q1_q3 = 2.0F / (2.0F / (rtP.lindi.servo.omega * rtP.lindi.servo.boost) +
                      2.0F / rtP.lindi.sflt.omega);

      // Sum: '<S59>/Sum2' incorporates:
      //   Constant: '<S59>/d'
      //   Constant: '<S59>/omega'
      //   DiscreteIntegrator: '<S58>/Discrete-Time Integrator'
      //   DiscreteIntegrator: '<S59>/Discrete-Time Integrator y'
      //   DiscreteIntegrator: '<S59>/Discrete-Time Integrator y_dt'
      //   Gain: '<S59>/Gain'
      //   Product: '<S59>/Divide'
      //   Product: '<S59>/Product2'
      //   Sum: '<S59>/Sum3'

      t = rtDW.DiscreteTimeIntegrator_DSTATE_a - (1.0F / q1_q3 * 2.0F *
        rtDW.DiscreteTimeIntegratory_dt_DS_m +
        rtDW.DiscreteTimeIntegratory_DSTA_pe);

      // Update for DiscreteIntegrator: '<S57>/Discrete-Time Integrator'
      rtDW.DiscreteTimeIntegrator_IC_LOA_m = 0U;
      rtDW.DiscreteTimeIntegrator_DSTATE_p += 0.0025F * q0_q3;

      // Update for DiscreteIntegrator: '<S58>/Discrete-Time Integrator' incorporates:
      //   Constant: '<S58>/T'
      //   Product: '<S58>/Divide'

      rtDW.DiscreteTimeIntegrator_IC_LO_mz = 0U;
      rtDW.DiscreteTimeIntegrator_DSTATE_a += absxk / (1.0F /
        rtP.lindi.atc.rm.pfreq) * 0.0025F;

      // Update for DiscreteIntegrator: '<S59>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S59>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_IC_L_id = 0U;
      rtDW.DiscreteTimeIntegratory_DSTA_pe += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_DS_m;

      // Update for DiscreteIntegrator: '<S59>/Discrete-Time Integrator y_dt' incorporates:
      //   Product: '<S59>/Product1'
      //   Product: '<S59>/omega^2'

      rtDW.DiscreteTimeIntegratory_dt_DS_m += q1_q3 * q1_q3 * t * 0.0025F;

      // Sum: '<S27>/Add2'
      rtDW.Merge = rtDW.q + q0_q3;
    } else {
      if (rtDW.PitchAngleController_MODE) {
        rtDW.PitchAngleController_MODE = false;
      }
    }

    // End of Outputs for SubSystem: '<S5>/Pitch Angle Controller'
    // End of Outputs for SubSystem: '<S5>/add'

    // DiscreteIntegrator: '<S61>/Discrete-Time Integrator'
    if (rtDW.DiscreteTimeIntegrator_IC_LOADI != 0) {
      rtDW.DiscreteTimeIntegrator_DSTATE_e = rtDW.Merge;
    }

    // RelationalOperator: '<S31>/Compare' incorporates:
    //   Constant: '<S14>/Constant'
    //   Constant: '<S31>/Constant'

    rtb_Compare_j = (rtP.lindi.mla.use == 0.0F);

    // Outputs for Enabled SubSystem: '<S14>/Subsystem' incorporates:
    //   EnablePort: '<S33>/Enable'

    if (rtb_Compare_j) {
      // SignalConversion: '<S33>/OutportBuffer_InsertedFor_u_d_at_inport_0' incorporates:
      //   Constant: '<S33>/Constant'

      for (i = 0; i < 10; i++) {
        rtDW.Merge_m[i] = rtP.lindi.ca.u_d[i];
      }

      // End of SignalConversion: '<S33>/OutportBuffer_InsertedFor_u_d_at_inport_0' 
    }

    // End of Outputs for SubSystem: '<S14>/Subsystem'

    // Outputs for Enabled SubSystem: '<S14>/Maneuver Load Alleviation' incorporates:
    //   EnablePort: '<S32>/Enable'

    if (!rtb_Compare_j) {
      if (!rtDW.ManeuverLoadAlleviation_MODE) {
        rtDW.ManeuverLoadAlleviation_MODE = true;
      }

      // MATLAB Function: '<S34>/MATLAB Function' incorporates:
      //   DiscreteIntegrator: '<S39>/Discrete-Time Integrator y'

      // :  g_b = M_bg*[0;0;9.81];
      // :  a_des_abs = q_ref*V_A + g_b(3);
      for (stage_app_2 = 0; stage_app_2 < 3; stage_app_2++) {
        expl_temp[stage_app_2] =
          rtDW.DiscreteTimeIntegratory_DSTAT_b[stage_app_2 + 6] * 9.81F;
      }

      // Gain: '<S32>/Gain1' incorporates:
      //   Constant: '<S32>/Constant1'
      //   DiscreteIntegrator: '<S61>/Discrete-Time Integrator'
      //   MATLAB Function: '<S34>/MATLAB Function'
      //   Sum: '<S32>/Add'

      absxk = -((rtDW.DiscreteTimeIntegrator_DSTATE_e * q0_q0 + expl_temp[2]) -
                9.81F);

      // :  num_flaps = int32(length(y_cp_flap)-2);
      // :  Delta_u_d = zeros(1,num_flaps+2,class(nu_a_Kz));
      for (i = 0; i < 10; i++) {
        // Reshape: '<S32>/Reshape4' incorporates:
        //   MATLAB Function: '<S32>/Maneuver Load Alleviation'

        rtDW.Merge_m[i] = 0.0F;
      }

      // MATLAB Function: '<S32>/Maneuver Load Alleviation' incorporates:
      //   Constant: '<S32>/Constant'
      //   Constant: '<S32>/Constant2'
      //   Reshape: '<S32>/Reshape4'

      // :  u = zeros(num_flaps,1,class(nu_a_Kz));
      // :  ca.u_min = ca_main.u_min(1:num_flaps);
      // :  ca.u_max = ca_main.u_max(1:num_flaps);
      // :  ca.u_d = ca_main.u_d(1:num_flaps);
      // :  ca.W_v = ca_mla.W_v;
      // :  ca.W_u = ca_mla.W_u(1:num_flaps);
      // :  ca.gamma = ca_mla.gamma;
      // :  ca.W = ca_main.W(1:num_flaps);
      // :  ca.i_max = ca_mla.i_max;
      // :  G11_1 = zeros(1,num_flaps,class(nu_a_Kz));
      // :  G11_2 = zeros(1,num_flaps,class(nu_a_Kz));
      for (stage_app_2 = 0; stage_app_2 < 8; stage_app_2++) {
        G11_1[stage_app_2] = 0.0F;
        G11_2[stage_app_2] = 0.0F;
      }

      // :  G11_1(1:end/2) = -G10(6,1:num_flaps/2).*abs(y_cp_flap(1:num_flaps/2)); 
      G11_1[0] = -rtb_Product2[5] * std::abs(rtP.lindi.cef.y[0]);
      G11_1[1] = -rtb_Product2[11] * std::abs(rtP.lindi.cef.y[1]);
      G11_1[2] = -rtb_Product2[17] * std::abs(rtP.lindi.cef.y[2]);
      G11_1[3] = -rtb_Product2[23] * std::abs(rtP.lindi.cef.y[3]);

      // :  G11_2(end/2+1:end) = -G10(6,num_flaps/2+1:num_flaps).*abs(y_cp_flap(num_flaps/2+1:num_flaps)); 
      G11_2[4] = -rtb_Product2[29] * std::abs(rtP.lindi.cef.y[4]);
      G11_2[5] = -rtb_Product2[35] * std::abs(rtP.lindi.cef.y[5]);
      G11_2[6] = -rtb_Product2[41] * std::abs(rtP.lindi.cef.y[6]);
      G11_2[7] = -rtb_Product2[47] * std::abs(rtP.lindi.cef.y[7]);

      // :  G11 = [G11_1;G11_2;G10(6,1:8)];
      // :  wrbm = abs(y_np_wing) * nu_a_Kz/2;
      // :  Delta_nu = [ wrbm(:); 0 ];
      // :  Delta_u_d(1:8) = caIndiWls( ca, G11, Delta_nu, u );
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
      // 'caIndiWls:78' umax    = min( umax, Delta_u_max );
      for (i = 0; i < 8; i++) {
        q1_q3 = std::abs(rtP.lindi.ca.u_max[i] - rtP.lindi.ca.u_min[i]);
        if (rtP.lindi.ca.u_min[i] > -q1_q3) {
          z1[i] = rtP.lindi.ca.u_min[i];
        } else {
          z1[i] = -q1_q3;
        }

        umin_0[i] = rtP.lindi.ca.u_min[i];
        if (rtP.lindi.ca.u_max[i] < q1_q3) {
          varargin_2[i] = rtP.lindi.ca.u_max[i];
        } else {
          varargin_2[i] = q1_q3;
        }

        umax_0[i] = rtP.lindi.ca.u_max[i];
      }

      // 'caIndiWls:81' gamma   = ca.gamma + Delta_gamma;
      // 'caIndiWls:82' ud      = ud + Delta_u_d;
      // 'caIndiWls:83' W_v     = diag( ca.W_v + Delta_diag_W_v );
      for (stage_app_2 = 0; stage_app_2 < 9; stage_app_2++) {
        I_b[stage_app_2] = 0.0F;
      }

      I_b[0] = rtP.lindi.mla.ca.W_v[0];
      I_b[4] = rtP.lindi.mla.ca.W_v[1];
      I_b[8] = rtP.lindi.mla.ca.W_v[2];

      // 'caIndiWls:85' W_u     = diag( ca.W_u );
      memset(&W_u_0[0], 0, sizeof(real32_T) << 6U);

      // 'caIndiWls:88' W       = zeros( length(ca.W_u), 1, superiorfloat(ca.W_u) ); 
      // 'caIndiWls:90' [ Delta_u, W, iter ] = wls_alloc( B, Delta_nu, umin, umax, ... 
      // 'caIndiWls:91'     W_v, W_u, ud, gamma, u0, W, ca.i_max );
      rtb_y_ai[0] = std::abs(rtP.lindi.mla.eta_np[0] * rtP.lindi.eig.b / 2.0F) *
        absxk / 2.0F;
      rtb_y_ai[1] = std::abs(rtP.lindi.mla.eta_np[1] * rtP.lindi.eig.b / 2.0F) *
        absxk / 2.0F;
      rtb_y_ai[2] = 0.0F;
      for (i = 0; i < 8; i++) {
        W_u_0[i + (i << 3)] = rtP.lindi.mla.ca.W_u[i];
        q2_q3 = (umin_0[i] + umax_0[i]) * 0.5F;
        G11_1_0[3 * i] = G11_1[i];
        G11_1_0[1 + 3 * i] = G11_2[i];
        G11_1_0[2 + 3 * i] = rtb_Product2[6 * i + 5];
        tmp[i] = rtP.lindi.ca.u_d[i];
        umin_0[i] = 0.0F;
        umax_0[i] = q2_q3;
      }

      wls_alloc(G11_1_0, rtb_y_ai, z1, varargin_2, I_b, W_u_0, tmp,
                rtP.lindi.mla.ca.gamma, umax_0, umin_0, rtP.lindi.mla.ca.i_max);
      for (stage_app_2 = 0; stage_app_2 < 8; stage_app_2++) {
        rtDW.Merge_m[stage_app_2] = umax_0[stage_app_2];
      }
    } else {
      if (rtDW.ManeuverLoadAlleviation_MODE) {
        rtDW.ManeuverLoadAlleviation_MODE = false;
      }
    }

    // End of Outputs for SubSystem: '<S14>/Maneuver Load Alleviation'

    // MATLAB Function: '<S20>/Only rotational control effectiveness' incorporates:
    //   Logic: '<S14>/Logical Operator'

    Onlyrotationalcontroleffectiven(rtb_Product2, rtb_y_j5);

    // MATLAB Function: '<S20>/Only rotational control effectiveness1'
    Onlyrotationalcontroleffectiven(rtb_Product3, force_dir);

    // Gain: '<S69>/Gain'
    for (stage_app_2 = 0; stage_app_2 < 30; stage_app_2++) {
      c_XYZ[stage_app_2] = 400.0F * force_dir[stage_app_2];
    }

    // End of Gain: '<S69>/Gain'

    // MATLAB Function: '<S20>/Only rotational control effectiveness2'
    Onlyrotationalcontroleffectiven(rtb_Product4, force_dir);
    for (stage_app_2 = 0; stage_app_2 < 30; stage_app_2++) {
      // Gain: '<S69>/Gain1'
      q1_q3 = 160000.0F * force_dir[stage_app_2];
      force_dir[stage_app_2] = q1_q3;
      rtb_y_j5[stage_app_2] = (rtb_y_j5[stage_app_2] + c_XYZ[stage_app_2]) +
        q1_q3;
      c_XYZ[stage_app_2] += 2.0F * q1_q3;
    }

    // Product: '<S61>/Divide' incorporates:
    //   Constant: '<S61>/T'
    //   DiscreteIntegrator: '<S61>/Discrete-Time Integrator'
    //   Gain: '<S69>/Gain1'
    //   Gain: '<S69>/Gain2'
    //   Sum: '<S61>/Sum2'
    //   Sum: '<S69>/Add'
    //   Sum: '<S69>/Add1'

    absxk = (rtDW.Merge - rtDW.DiscreteTimeIntegrator_DSTATE_e) / (1.0F /
      rtP.lindi.atc.rm.pfreq);

    // DiscreteIntegrator: '<S63>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S61>/Discrete-Time Integrator'

    if (rtDW.DiscreteTimeIntegratory_IC_LO_m != 0) {
      rtDW.DiscreteTimeIntegratory_DSTAT_i =
        rtDW.DiscreteTimeIntegrator_DSTATE_e;
    }

    // DiscreteIntegrator: '<S62>/Discrete-Time Integrator y'
    if (rtDW.DiscreteTimeIntegratory_IC_LO_j != 0) {
      rtDW.DiscreteTimeIntegratory_DSTAT_e = absxk;
    }

    // Sum: '<S64>/Add2' incorporates:
    //   DiscreteIntegrator: '<S61>/Discrete-Time Integrator'
    //   DiscreteIntegrator: '<S67>/Discrete-Time Integrator y'

    t = rtDW.DiscreteTimeIntegrator_DSTATE_e -
      rtDW.DiscreteTimeIntegratory_DSTAT_h;

    // Product: '<S84>/Product1' incorporates:
    //   Constant: '<S84>/d'
    //   Constant: '<S84>/omega'
    //   DiscreteIntegrator: '<S84>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S84>/Discrete-Time Integrator y_dt'
    //   Gain: '<S84>/Gain'
    //   Product: '<S84>/Divide'
    //   Product: '<S84>/Product2'
    //   Product: '<S84>/omega^2'
    //   Sum: '<S84>/Sum2'
    //   Sum: '<S84>/Sum3'

    q0_q3 = (scale - (1.0F / rtP.lindi.atc.rm.rfreq * 2.0F *
                      rtDW.DiscreteTimeIntegratory_dt_DSTA +
                      rtDW.DiscreteTimeIntegratory_DSTAT_p)) *
      (rtP.lindi.atc.rm.rfreq * rtP.lindi.atc.rm.rfreq);

    // DiscreteIntegrator: '<S85>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S84>/Discrete-Time Integrator y'

    if (rtDW.DiscreteTimeIntegratory_IC_LO_b != 0) {
      rtDW.DiscreteTimeIntegratory_DSTA_bw =
        rtDW.DiscreteTimeIntegratory_DSTAT_p;
    }

    // MATLAB Function: '<S83>/wrap angle' incorporates:
    //   DiscreteIntegrator: '<S85>/Discrete-Time Integrator y'

    wrapangle(rtDW.DiscreteTimeIntegratory_DSTA_bw, &q1_q3);

    // MATLAB Function: '<S83>/wrap angle1' incorporates:
    //   MATLAB Function: '<S15>/Rotations matrix to Euler angles'

    wrapangle(q_bg_unsigned_idx_3, &q1_q2);

    // MATLAB Function: '<S83>/angle error'
    // :  error = errorAngle(angle_ref,angle);
    // 'errorAngle:22' error = angle_ref - angle;
    q1_q3 -= q1_q2;

    // 'errorAngle:25' if error > pi
    if (q1_q3 > 3.1415926535897931) {
      // 'errorAngle:26' error = error - 2*pi;
      q1_q3 -= 6.28318548F;
    } else {
      if (q1_q3 < -3.1415926535897931) {
        // 'errorAngle:27' elseif error < -pi
        // 'errorAngle:28' error = error + 2*pi;
        q1_q3 += 6.28318548F;
      }
    }

    // End of MATLAB Function: '<S83>/angle error'

    // DiscreteIntegrator: '<S86>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S84>/Discrete-Time Integrator y_dt'

    if (rtDW.DiscreteTimeIntegratory_IC_L_an != 0) {
      rtDW.DiscreteTimeIntegratory_DSTAT_m =
        rtDW.DiscreteTimeIntegratory_dt_DSTA;
    }

    // DiscreteIntegrator: '<S87>/Discrete-Time Integrator y'
    if (rtDW.DiscreteTimeIntegratory_IC_LO_c != 0) {
      rtDW.DiscreteTimeIntegratory_DSTAT_f = q0_q3;
    }

    // Sum: '<S5>/Add1' incorporates:
    //   UnitDelay: '<S22>/Unit Delay'

    q1_q2 = rtDW.Merge1 + rtDW.UnitDelay_DSTATE_fx;

    // DiscreteIntegrator: '<S100>/Discrete-Time Integrator'
    if (rtDW.DiscreteTimeIntegrator_IC_LOA_g != 0) {
      rtDW.DiscreteTimeIntegrator_DSTATE_o = q1_q2;
    }

    // Product: '<S100>/Divide' incorporates:
    //   Constant: '<S100>/T'
    //   DiscreteIntegrator: '<S100>/Discrete-Time Integrator'
    //   Sum: '<S100>/Sum2'

    q1_q2 = (q1_q2 - rtDW.DiscreteTimeIntegrator_DSTATE_o) / (1.0F /
      rtP.lindi.atc.rm.yfreq);

    // DiscreteIntegrator: '<S102>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S100>/Discrete-Time Integrator'

    if (rtDW.DiscreteTimeIntegratory_IC_L_jp != 0) {
      rtDW.DiscreteTimeIntegratory_DSTA_f0 =
        rtDW.DiscreteTimeIntegrator_DSTATE_o;
    }

    // DiscreteIntegrator: '<S101>/Discrete-Time Integrator y'
    if (rtDW.DiscreteTimeIntegratory_IC_L_bc != 0) {
      rtDW.DiscreteTimeIntegratory_DSTAT_o = q1_q2;
    }

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
      // UnitDelay: '<S69>/Unit Delay1'
      clu[itmp] = rtDW.UnitDelay1_DSTATE[itmp];

      // MATLAB Function: '<S77>/caIndiWls' incorporates:
      //   Constant: '<S77>/Delta u_max'
      //   DiscreteIntegrator: '<S74>/Discrete-Time Integrator y'

      q_bg_unsigned_idx_0 = rtP.lindi.ca.u_min[itmp] -
        rtDW.DiscreteTimeIntegratory_DSTAT_l[itmp];
      q2_q3 = rtP.lindi.ca.u_max[itmp] -
        rtDW.DiscreteTimeIntegratory_DSTAT_l[itmp];
      rtb_Delta_u[itmp] = q_bg_unsigned_idx_0 + q2_q3;
      q_bg_unsigned_idx_2 = rtP.lindi.ca.u_max[itmp] - rtP.lindi.ca.u_min[itmp];
      umax[itmp] = q2_q3;

      // MATLAB Function: '<S77>/caIndiWls' incorporates:
      //   Constant: '<S77>/Delta u_max'

      q_bg_unsigned_idx_1 = std::abs(q_bg_unsigned_idx_2);
      if (q_bg_unsigned_idx_0 > -q_bg_unsigned_idx_1) {
        rtb_DiscreteTimeIntegrator_l[itmp] = q_bg_unsigned_idx_0;
      } else {
        rtb_DiscreteTimeIntegrator_l[itmp] = -q_bg_unsigned_idx_1;
      }

      rtb_Sum2_ha[itmp] = q_bg_unsigned_idx_2;
    }

    // MATLAB Function: '<S77>/caIndiWls' incorporates:
    //   Constant: '<S77>/Delta u_max'

    // 'caIndiWls:78' umax    = min( umax, Delta_u_max );
    for (i = 0; i < 10; i++) {
      for (itmp = 0; itmp < 10; itmp++) {
        rtb_u[itmp] = std::abs(rtb_Sum2_ha[itmp]);
      }

      if (umax[i] < rtb_u[i]) {
        umin[i] = umax[i];
      } else {
        umin[i] = rtb_u[i];
      }
    }

    // 'caIndiWls:81' gamma   = ca.gamma + Delta_gamma;
    // 'caIndiWls:82' ud      = ud + Delta_u_d;
    // 'caIndiWls:83' W_v     = diag( ca.W_v + Delta_diag_W_v );
    for (stage_app_2 = 0; stage_app_2 < 9; stage_app_2++) {
      I_b[stage_app_2] = 0.0F;
    }

    I_b[0] = rtP.lindi.ca.W_v[0];
    I_b[4] = rtP.lindi.ca.W_v[1];
    I_b[8] = rtP.lindi.ca.W_v[2];

    // 'caIndiWls:85' W_u     = diag( ca.W_u );
    memset(&W_u[0], 0, 100U * sizeof(real32_T));

    // 'caIndiWls:88' W       = zeros( length(ca.W_u), 1, superiorfloat(ca.W_u) ); 
    // 'caIndiWls:90' [ Delta_u, W, iter ] = wls_alloc( B, Delta_nu, umin, umax, ... 
    // 'caIndiWls:91'     W_v, W_u, ud, gamma, u0, W, ca.i_max );
    for (i = 0; i < 10; i++) {
      W_u[i + 10 * i] = rtP.lindi.ca.W_u[i];
      umax[i] = 0.0F;
      rtb_Delta_u[i] *= 0.5F;
    }

    // Sum: '<S69>/Add2' incorporates:
    //   DiscreteIntegrator: '<S101>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S102>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S62>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S63>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S66>/Discrete-Time Integrator'
    //   DiscreteIntegrator: '<S84>/Discrete-Time Integrator y_dt'
    //   DiscreteIntegrator: '<S86>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S87>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S92>/Discrete-Time Integrator y'
    //   Gain: '<S60>/Gain3'
    //   Gain: '<S60>/Gain5'
    //   Gain: '<S64>/Gain14'
    //   Gain: '<S64>/Gain15'
    //   Gain: '<S64>/Gain16'
    //   Gain: '<S64>/Gain17'
    //   Gain: '<S64>/Gain5'
    //   Gain: '<S64>/Gain6'
    //   Gain: '<S64>/Gain7'
    //   Gain: '<S64>/Gain8'
    //   Gain: '<S83>/Gain1'
    //   Gain: '<S83>/Gain3'
    //   Gain: '<S83>/Gain5'
    //   Gain: '<S88>/Gain'
    //   Gain: '<S88>/Gain1'
    //   Gain: '<S88>/Gain3'
    //   Gain: '<S88>/Gain4'
    //   Gain: '<S99>/Gain3'
    //   Gain: '<S99>/Gain5'
    //   Product: '<S64>/Product2'
    //   Product: '<S64>/Product3'
    //   Product: '<S64>/Product4'
    //   Product: '<S88>/Product'
    //   Product: '<S88>/Product1'
    //   Sum: '<S19>/Add1'
    //   Sum: '<S19>/Add2'
    //   Sum: '<S19>/Add9'
    //   Sum: '<S21>/Add1'
    //   Sum: '<S21>/Add2'
    //   Sum: '<S21>/Add3'
    //   Sum: '<S26>/Add1'
    //   Sum: '<S26>/Add2'
    //   Sum: '<S5>/Add'
    //   Sum: '<S60>/Add'
    //   Sum: '<S60>/Add1'
    //   Sum: '<S60>/Add2'
    //   Sum: '<S64>/Add5'
    //   Sum: '<S83>/Add'
    //   Sum: '<S83>/Add1'
    //   Sum: '<S83>/Add2'
    //   Sum: '<S88>/Add2'
    //   Sum: '<S99>/Add'
    //   Sum: '<S99>/Add1'
    //   Sum: '<S99>/Add2'

    rtb_Sum2_ny[0] = ((((rtDW.DiscreteTimeIntegratory_DSTAT_m - rtb_y_k[0]) *
                        rtP.lindi.atc.k.rrat + rtP.lindi.atc.k.rang * q1_q3) +
                       (rtDW.DiscreteTimeIntegratory_DSTAT_f - rtb_y_pd[0]) *
                       rtP.lindi.atc.k.racc) + q0_q3) - (rtP.lindi.eig.b / 2.0F *
      (rtDW.DiscreteTimeIntegratory_dt_DSTA -
       rtDW.DiscreteTimeIntegratory_DSTA_hc) * rtP.lindi.eig.clp * (q0_q0 *
      0.6125F) * (rtP.lindi.eig.s * rtP.lindi.eig.b) * (1.0F / rtP.lindi.ceb.ixx)
      + rtb_y_pd[0]);
    rtb_Sum2_ny[1] = ((((rtDW.DiscreteTimeIntegratory_DSTAT_i - rtb_y_k[1]) *
                        rtP.lindi.atc.k.prat +
                        (rtDW.DiscreteTimeIntegratory_DSTAT_e - rtb_y_pd[1]) *
                        rtP.lindi.atc.k.pacc) + absxk) -
                      ((-(rtDW.DiscreteTimeIntegrator_DSTATE_h * q0_q0 *
                          rtP.lindi.ceb.m * rtP.lindi.eig.x_h *
                          (rtP.lindi.eig.s_h / (rtP.lindi.eig.s_h +
      rtP.lindi.eig.s))) + rtP.lindi.eig.x_h * t * -rtP.lindi.eig.cla_h * (q0_q0
      * 0.6125F) * (rtP.lindi.eig.s_h * rtP.lindi.eig.x_h)) * (1.0F /
      rtP.lindi.ceb.iyy) + rtb_y_pd[1])) + rtDW.nu_q_dt_ptchcntrl;
    rtb_Sum2_ny[2] = (((rtDW.DiscreteTimeIntegratory_DSTA_f0 - rtb_y_k[2]) *
                       rtP.lindi.atc.k.yrat +
                       (rtDW.DiscreteTimeIntegratory_DSTAT_o - rtb_y_pd[2]) *
                       rtP.lindi.atc.k.yacc) + q1_q2) - rtb_y_pd[2];
    for (stage_app_2 = 0; stage_app_2 < 3; stage_app_2++) {
      // Product: '<S69>/MatrixMultiply2'
      p_match_2[stage_app_2] = 0.0F;

      // Sum: '<S69>/Add2'
      q2_q3 = 0.0F;
      for (i = 0; i < 10; i++) {
        // Product: '<S69>/MatrixMultiply3' incorporates:
        //   Product: '<S69>/MatrixMultiply2'

        p2 = 3 * i + stage_app_2;

        // Sum: '<S69>/Add2' incorporates:
        //   Product: '<S69>/MatrixMultiply3'
        //   UnitDelay: '<S69>/Unit Delay2'

        q2_q3 += force_dir[p2] * rtDW.UnitDelay2_DSTATE[i];
        p_match_2[stage_app_2] += c_XYZ[p2] * rtDW.UnitDelay1_DSTATE[i];
      }

      // Sum: '<S69>/Add2' incorporates:
      //   Product: '<S69>/MatrixMultiply2'
      //   Product: '<S69>/MatrixMultiply3'
      //   UnitDelay: '<S69>/Unit Delay1'

      rtb_y_ai[stage_app_2] = (rtb_Sum2_ny[stage_app_2] + p_match_2[stage_app_2])
        - q2_q3;
    }

    // MATLAB Function: '<S77>/caIndiWls' incorporates:
    //   DiscreteIntegrator: '<S74>/Discrete-Time Integrator y'

    for (stage_app_2 = 0; stage_app_2 < 10; stage_app_2++) {
      rtb_u[stage_app_2] = (rtP.lindi.ca.u_d[stage_app_2] -
                            rtDW.DiscreteTimeIntegratory_DSTAT_l[stage_app_2]) +
        rtDW.Merge_m[stage_app_2];
    }

    // SignalConversion: '<S5>/BusConversion_InsertedFor_ap_bus_at_inport_0' incorporates:
    //   MATLAB Function: '<S77>/caIndiWls'

    rtDW.iter = wls_alloc_f(rtb_y_j5, rtb_y_ai, rtb_DiscreteTimeIntegrator_l,
      umin, I_b, W_u, rtb_u, rtP.lindi.ca.gamma, rtb_Delta_u, umax,
      rtP.lindi.ca.i_max);

    // Gain: '<S79>/Gain2' incorporates:
    //   Gain: '<S79>/Gain3'
    //   Gain: '<S80>/Gain2'
    //   Gain: '<S80>/Gain3'

    q1_q3 = rtP.lindi.servo.boost / (1.0F / rtP.lindi.servo.omega);
    for (i = 0; i < 10; i++) {
      // Sum: '<S20>/Add6' incorporates:
      //   DiscreteIntegrator: '<S73>/Discrete-Time Integrator y'

      q_bg_unsigned_idx_0 = rtb_Delta_u[i] +
        rtDW.DiscreteTimeIntegratory_DSTA_j2[i];

      // Sum: '<S79>/Add1' incorporates:
      //   DiscreteIntegrator: '<S79>/Discrete-Time Integrator'
      //   DiscreteIntegrator: '<S79>/Discrete-Time Integrator1'
      //   Gain: '<S79>/Gain'
      //   Gain: '<S79>/Gain2'
      //   Gain: '<S79>/Gain3'

      rtb_DiscreteTimeIntegrator_l[i] = (rtP.lindi.servo.boost *
        q_bg_unsigned_idx_0 + q1_q3 * rtDW.DiscreteTimeIntegrator1_DSTATE[i]) -
        q1_q3 * rtDW.DiscreteTimeIntegrator_DSTATE_l[i];
      umin[i] = q_bg_unsigned_idx_0;
    }

    for (i = 0; i < 10; i++) {
      // Sum: '<S80>/Add1' incorporates:
      //   DiscreteIntegrator: '<S80>/Discrete-Time Integrator'
      //   DiscreteIntegrator: '<S80>/Discrete-Time Integrator1'
      //   Gain: '<S80>/Gain'
      //   Gain: '<S80>/Gain2'
      //   Gain: '<S80>/Gain3'

      q2_q3 = (rtP.lindi.servo.boost * rtb_DiscreteTimeIntegrator_l[i] + q1_q3 *
               rtDW.DiscreteTimeIntegrator1_DSTAT_j[i]) - q1_q3 *
        rtDW.DiscreteTimeIntegrator_DSTATE_g[i];

      // Saturate: '<S20>/Saturation3'
      if (q2_q3 > rtP.lindi.ca.u_max[i]) {
        q_bg_unsigned_idx_1 = rtP.lindi.ca.u_max[i];
      } else if (q2_q3 < rtP.lindi.ca.u_min[i]) {
        q_bg_unsigned_idx_1 = rtP.lindi.ca.u_min[i];
      } else {
        q_bg_unsigned_idx_1 = q2_q3;
      }

      // SignalConversion: '<S5>/BusConversion_InsertedFor_ap_bus_at_inport_0'
      rtDW.u[i] = q_bg_unsigned_idx_1;
      umax[i] = q2_q3;
      rtb_u[i] = q_bg_unsigned_idx_1;
    }

    // SignalConversion: '<S5>/BusConversion_InsertedFor_ap_bus_at_inport_0' incorporates:
    //   Inport: '<Root>/cmd'
    //   Saturate: '<S20>/Saturation3'
    //   Sum: '<S20>/Add6'
    //   Sum: '<S80>/Add1'

    rtDW.u[10] = rtU.cmd.thr;

    // Gain: '<S74>/Gain' incorporates:
    //   Constant: '<S74>/d'
    //   Constant: '<S74>/omega'
    //   Product: '<S74>/Divide'

    q1_q3 = 1.0F / rtP.lindi.servo.omega * 2.0F;

    // Sum: '<S74>/Sum2' incorporates:
    //   DiscreteIntegrator: '<S74>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S74>/Discrete-Time Integrator y_dt'
    //   Product: '<S74>/Product2'
    //   Sum: '<S74>/Sum3'

    for (i = 0; i < 10; i++) {
      rtb_Sum2_ha[i] = rtb_u[i] - (rtDW.DiscreteTimeIntegratory_dt_DS_o[i] *
        q1_q3 + rtDW.DiscreteTimeIntegratory_DSTAT_l[i]);
    }

    // End of Sum: '<S74>/Sum2'

    // Product: '<S74>/omega^2' incorporates:
    //   Constant: '<S74>/omega'

    q1_q3 = rtP.lindi.servo.omega * rtP.lindi.servo.omega;

    // Gain: '<S73>/Gain' incorporates:
    //   Constant: '<S73>/d'
    //   Constant: '<S73>/omega'
    //   Gain: '<S39>/Gain'
    //   Gain: '<S40>/Gain'
    //   Gain: '<S41>/Gain'
    //   Gain: '<S42>/Gain'
    //   Product: '<S73>/Divide'

    q_bg_unsigned_idx_1 = rtP.lindi.sflt.d / rtP.lindi.sflt.omega * 2.0F;

    // Sum: '<S73>/Sum2' incorporates:
    //   DiscreteIntegrator: '<S73>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S73>/Discrete-Time Integrator y_dt'
    //   DiscreteIntegrator: '<S74>/Discrete-Time Integrator y'
    //   Gain: '<S73>/Gain'
    //   Product: '<S73>/Product2'
    //   Sum: '<S73>/Sum3'

    for (i = 0; i < 10; i++) {
      rtb_Sum2_dx[i] = rtDW.DiscreteTimeIntegratory_DSTAT_l[i] -
        (rtDW.DiscreteTimeIntegratory_dt_DS_n[i] * q_bg_unsigned_idx_1 +
         rtDW.DiscreteTimeIntegratory_DSTA_j2[i]);
    }

    // End of Sum: '<S73>/Sum2'

    // Product: '<S73>/omega^2' incorporates:
    //   Constant: '<S73>/omega'

    q2_q3 = rtP.lindi.sflt.omega * rtP.lindi.sflt.omega;

    // Sum: '<S39>/Sum2' incorporates:
    //   DiscreteIntegrator: '<S39>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S39>/Discrete-Time Integrator y_dt'
    //   Product: '<S39>/Product2'
    //   Sum: '<S39>/Sum3'

    for (stage_app_2 = 0; stage_app_2 < 9; stage_app_2++) {
      rtb_M_bg[stage_app_2] -= rtDW.DiscreteTimeIntegratory_dt_D_ni[stage_app_2]
        * q_bg_unsigned_idx_1 + rtDW.DiscreteTimeIntegratory_DSTAT_b[stage_app_2];
    }

    // End of Sum: '<S39>/Sum2'

    // Product: '<S39>/omega^2' incorporates:
    //   Constant: '<S39>/omega'

    q_bg_unsigned_idx_0 = rtP.lindi.sflt.omega * rtP.lindi.sflt.omega;

    // Product: '<S42>/omega^2' incorporates:
    //   Constant: '<S42>/omega'

    q_bg_unsigned_idx_2 = rtP.lindi.sflt.omega * rtP.lindi.sflt.omega;

    // SignalConversion: '<S5>/BusConversion_InsertedFor_ap_bus_at_inport_0' incorporates:
    //   MATLAB Function: '<S15>/Rotations matrix to Euler angles'

    rtDW.Euler_angles_f[0] = q_bg_unsigned_idx_3;

    // SignalConversion: '<S5>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.Omega_Kb_f[0] = rtb_y_k[0];

    // SignalConversion: '<S5>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.Omega_Kb_dt_f[0] = rtb_y_pd[0];

    // SignalConversion: '<S5>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.e_s_g[0] = rtDW.e_s_g_e[0];

    // SignalConversion: '<S5>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.e_s_g_dt[0] = rtDW.e_s_g_dt_i[0];

    // SignalConversion: '<S5>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.e_s_g_dt2[0] = rtDW.e_s_g_dt2_k[0];

    // SignalConversion: '<S5>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.s_g_ref[0] = rtDW.s_g_ref_d[0];

    // SignalConversion: '<S5>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.s_g[0] = rtDW.s_g_a[0];

    // SignalConversion: '<S5>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.s_g_match[0] = rtDW.p_match[0];
    rtb_y_k[0] = (rtU.measure.V_Kg[0] - (rtDW.DiscreteTimeIntegratory_dt_DS_e[0]
      * q_bg_unsigned_idx_1 + rtDW.DiscreteTimeIntegratory_DSTAT_j[0])) *
      q_bg_unsigned_idx_2;

    // SignalConversion: '<S5>/BusConversion_InsertedFor_ap_bus_at_inport_0' incorporates:
    //   DiscreteIntegrator: '<S42>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S42>/Discrete-Time Integrator y_dt'
    //   Inport: '<Root>/measure'
    //   MATLAB Function: '<S15>/Rotations matrix to Euler angles'
    //   Product: '<S42>/Product1'
    //   Product: '<S42>/Product2'
    //   Product: '<S42>/omega^2'
    //   Sum: '<S42>/Sum2'
    //   Sum: '<S42>/Sum3'

    rtDW.Euler_angles_f[1] = -Phi_i;

    // SignalConversion: '<S5>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.Omega_Kb_f[1] = rtb_y_k[1];

    // SignalConversion: '<S5>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.Omega_Kb_dt_f[1] = rtb_y_pd[1];

    // SignalConversion: '<S5>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.e_s_g[1] = rtDW.e_s_g_e[1];

    // SignalConversion: '<S5>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.e_s_g_dt[1] = rtDW.e_s_g_dt_i[1];

    // SignalConversion: '<S5>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.e_s_g_dt2[1] = rtDW.e_s_g_dt2_k[1];

    // SignalConversion: '<S5>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.s_g_ref[1] = rtDW.s_g_ref_d[1];

    // SignalConversion: '<S5>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.s_g[1] = rtDW.s_g_a[1];

    // SignalConversion: '<S5>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.s_g_match[1] = rtDW.p_match[1];
    rtb_y_k[1] = (rtU.measure.V_Kg[1] - (rtDW.DiscreteTimeIntegratory_dt_DS_e[1]
      * q_bg_unsigned_idx_1 + rtDW.DiscreteTimeIntegratory_DSTAT_j[1])) *
      q_bg_unsigned_idx_2;

    // SignalConversion: '<S5>/BusConversion_InsertedFor_ap_bus_at_inport_0' incorporates:
    //   DiscreteIntegrator: '<S39>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S42>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S42>/Discrete-Time Integrator y_dt'
    //   Inport: '<Root>/measure'
    //   MATLAB Function: '<S15>/Rotations matrix to Euler angles'
    //   Product: '<S42>/Product1'
    //   Product: '<S42>/Product2'
    //   Product: '<S42>/omega^2'
    //   Sum: '<S42>/Sum2'
    //   Sum: '<S42>/Sum3'

    rtDW.Euler_angles_f[2] = std::atan2(rtDW.DiscreteTimeIntegratory_DSTAT_b[3],
      rtDW.DiscreteTimeIntegratory_DSTAT_b[0]);

    // SignalConversion: '<S5>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.Omega_Kb_f[2] = rtb_y_k[2];

    // SignalConversion: '<S5>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.Omega_Kb_dt_f[2] = rtb_y_pd[2];

    // SignalConversion: '<S5>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.e_s_g[2] = rtDW.e_s_g_e[2];

    // SignalConversion: '<S5>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.e_s_g_dt[2] = rtDW.e_s_g_dt_i[2];

    // SignalConversion: '<S5>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.e_s_g_dt2[2] = rtDW.e_s_g_dt2_k[2];

    // SignalConversion: '<S5>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.s_g_ref[2] = rtDW.s_g_ref_d[2];

    // SignalConversion: '<S5>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.s_g[2] = rtDW.s_g_a[2];

    // SignalConversion: '<S5>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.s_g_match[2] = rtDW.p_match[2];

    // Product: '<S42>/Product1' incorporates:
    //   DiscreteIntegrator: '<S42>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S42>/Discrete-Time Integrator y_dt'
    //   Inport: '<Root>/measure'
    //   Product: '<S42>/Product2'
    //   Product: '<S42>/omega^2'
    //   Sum: '<S42>/Sum2'
    //   Sum: '<S42>/Sum3'

    q_bg_unsigned_idx_2 *= rtU.measure.V_Kg[2] -
      (rtDW.DiscreteTimeIntegratory_dt_DS_e[2] * q_bg_unsigned_idx_1 +
       rtDW.DiscreteTimeIntegratory_DSTAT_j[2]);

    // Sum: '<S41>/Sum2' incorporates:
    //   DiscreteIntegrator: '<S41>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S41>/Discrete-Time Integrator y_dt'
    //   Inport: '<Root>/measure'
    //   Product: '<S41>/Product2'
    //   Sum: '<S41>/Sum3'

    rtb_Sum2_ci[0] = rtU.measure.s_Kg[0] -
      (rtDW.DiscreteTimeIntegratory_dt_DS_p[0] * q_bg_unsigned_idx_1 +
       rtDW.DiscreteTimeIntegratory_DSTA_n2[0]);
    rtb_Sum2_ci[1] = rtU.measure.s_Kg[1] -
      (rtDW.DiscreteTimeIntegratory_dt_DS_p[1] * q_bg_unsigned_idx_1 +
       rtDW.DiscreteTimeIntegratory_DSTA_n2[1]);
    rtb_Sum2_ci[2] = rtU.measure.s_Kg[2] -
      (rtDW.DiscreteTimeIntegratory_dt_DS_p[2] * q_bg_unsigned_idx_1 +
       rtDW.DiscreteTimeIntegratory_DSTA_n2[2]);

    // Product: '<S41>/omega^2' incorporates:
    //   Constant: '<S41>/omega'

    q_bg_unsigned_idx_3 = rtP.lindi.sflt.omega * rtP.lindi.sflt.omega;

    // Sum: '<S40>/Sum2' incorporates:
    //   DiscreteIntegrator: '<S40>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S40>/Discrete-Time Integrator y_dt'
    //   Inport: '<Root>/measure'
    //   Product: '<S40>/Product2'
    //   Sum: '<Root>/Add'
    //   Sum: '<S40>/Sum3'

    rtb_y_pd[0] = rtU.measure.a_Kg[0] - (rtDW.DiscreteTimeIntegratory_dt_DS_i[0]
      * q_bg_unsigned_idx_1 + rtDW.DiscreteTimeIntegratory_DSTAT_n[0]);
    rtb_y_pd[1] = rtU.measure.a_Kg[1] - (rtDW.DiscreteTimeIntegratory_dt_DS_i[1]
      * q_bg_unsigned_idx_1 + rtDW.DiscreteTimeIntegratory_DSTAT_n[1]);
    rtb_y_pd[2] = (rtU.measure.a_Kg[2] + 9.81F) -
      (rtDW.DiscreteTimeIntegratory_dt_DS_i[2] * q_bg_unsigned_idx_1 +
       rtDW.DiscreteTimeIntegratory_DSTAT_n[2]);

    // Product: '<S40>/omega^2' incorporates:
    //   Constant: '<S40>/omega'

    rtb_omega2_e_0 = rtP.lindi.sflt.omega * rtP.lindi.sflt.omega;

    // Sum: '<S101>/Sum2' incorporates:
    //   Constant: '<S101>/d'
    //   Constant: '<S101>/omega'
    //   DiscreteIntegrator: '<S101>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S101>/Discrete-Time Integrator y_dt'
    //   Gain: '<S101>/Gain'
    //   Product: '<S101>/Divide'
    //   Product: '<S101>/Product2'
    //   Sum: '<S101>/Sum3'

    // :  y = u(end-1);
    rtb_Sum2_cx = q1_q2 - (1.0F / (2.0F / (2.0F / (rtP.lindi.servo.omega *
      rtP.lindi.servo.boost) + 2.0F / rtP.lindi.sflt.omega)) * 2.0F *
      rtDW.DiscreteTimeIntegratory_dt_D_oo +
      rtDW.DiscreteTimeIntegratory_DSTAT_o);

    // Sum: '<S102>/Sum2' incorporates:
    //   Constant: '<S102>/d'
    //   Constant: '<S102>/omega'
    //   DiscreteIntegrator: '<S100>/Discrete-Time Integrator'
    //   DiscreteIntegrator: '<S102>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S102>/Discrete-Time Integrator y_dt'
    //   Gain: '<S102>/Gain'
    //   Product: '<S102>/Divide'
    //   Product: '<S102>/Product2'
    //   Sum: '<S102>/Sum3'

    rtb_Sum2_b = rtDW.DiscreteTimeIntegrator_DSTATE_o - (1.0F / (2.0F / (2.0F /
      (rtP.lindi.servo.omega * rtP.lindi.servo.boost) + 2.0F /
      rtP.lindi.sflt.omega)) * 2.0F * rtDW.DiscreteTimeIntegratory_dt_DS_j +
      rtDW.DiscreteTimeIntegratory_DSTA_f0);

    // Sum: '<S92>/Sum2' incorporates:
    //   Constant: '<S92>/d'
    //   Constant: '<S92>/omega'
    //   DiscreteIntegrator: '<S92>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S92>/Discrete-Time Integrator y_dt'
    //   DiscreteIntegrator: '<S93>/Discrete-Time Integrator y'
    //   Gain: '<S92>/Gain'
    //   Product: '<S92>/Divide'
    //   Product: '<S92>/Product2'
    //   Sum: '<S92>/Sum3'

    rtb_Sum2_lb = rtDW.DiscreteTimeIntegratory_DSTAT_g - (1.0F /
      rtP.lindi.sflt.omega * 2.0F * rtDW.DiscreteTimeIntegratory_dt_DS_g +
      rtDW.DiscreteTimeIntegratory_DSTA_hc);

    // Sum: '<S93>/Sum2' incorporates:
    //   Constant: '<S93>/d'
    //   Constant: '<S93>/omega'
    //   DiscreteIntegrator: '<S84>/Discrete-Time Integrator y_dt'
    //   DiscreteIntegrator: '<S93>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S93>/Discrete-Time Integrator y_dt'
    //   Gain: '<S93>/Gain'
    //   Product: '<S93>/Divide'
    //   Product: '<S93>/Product2'
    //   Sum: '<S93>/Sum3'

    rtb_Sum2_o3 = rtDW.DiscreteTimeIntegratory_dt_DSTA - (1.0F /
      (rtP.lindi.servo.omega * rtP.lindi.servo.boost) * 2.0F *
      rtDW.DiscreteTimeIntegratory_dt_D_ir +
      rtDW.DiscreteTimeIntegratory_DSTAT_g);

    // Sum: '<S87>/Sum2' incorporates:
    //   Constant: '<S87>/d'
    //   Constant: '<S87>/omega'
    //   DiscreteIntegrator: '<S87>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S87>/Discrete-Time Integrator y_dt'
    //   Gain: '<S87>/Gain'
    //   Product: '<S87>/Divide'
    //   Product: '<S87>/Product2'
    //   Sum: '<S87>/Sum3'

    rtb_Sum2_n = q0_q3 - (1.0F / (2.0F / (2.0F / (rtP.lindi.servo.omega *
      rtP.lindi.servo.boost) + 2.0F / rtP.lindi.sflt.omega)) * 2.0F *
                          rtDW.DiscreteTimeIntegratory_dt_DS_l +
                          rtDW.DiscreteTimeIntegratory_DSTAT_f);

    // Sum: '<S86>/Sum2' incorporates:
    //   Constant: '<S86>/d'
    //   Constant: '<S86>/omega'
    //   DiscreteIntegrator: '<S84>/Discrete-Time Integrator y_dt'
    //   DiscreteIntegrator: '<S86>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S86>/Discrete-Time Integrator y_dt'
    //   Gain: '<S86>/Gain'
    //   Product: '<S86>/Divide'
    //   Product: '<S86>/Product2'
    //   Sum: '<S86>/Sum3'

    rtb_Sum2_po = rtDW.DiscreteTimeIntegratory_dt_DSTA - (1.0F / (2.0F / (2.0F /
      (rtP.lindi.servo.omega * rtP.lindi.servo.boost) + 2.0F /
      rtP.lindi.sflt.omega)) * 2.0F * rtDW.DiscreteTimeIntegratory_dt_DS_f +
      rtDW.DiscreteTimeIntegratory_DSTAT_m);

    // Sum: '<S85>/Sum2' incorporates:
    //   Constant: '<S85>/d'
    //   Constant: '<S85>/omega'
    //   DiscreteIntegrator: '<S84>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S85>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S85>/Discrete-Time Integrator y_dt'
    //   Gain: '<S85>/Gain'
    //   Product: '<S85>/Divide'
    //   Product: '<S85>/Product2'
    //   Sum: '<S85>/Sum3'

    rtb_Sum2_ec = rtDW.DiscreteTimeIntegratory_DSTAT_p - (1.0F / (2.0F / (2.0F /
      (rtP.lindi.servo.omega * rtP.lindi.servo.boost) + 2.0F /
      rtP.lindi.sflt.omega)) * 2.0F * rtDW.DiscreteTimeIntegratory_dt_D_pv +
      rtDW.DiscreteTimeIntegratory_DSTA_bw);

    // MATLAB Function: '<S64>/MATLAB Function1' incorporates:
    //   Constant: '<S64>/Constant6'
    //   Constant: '<S64>/Constant7'
    //   Constant: '<S64>/Constant8'

    // :  A_alpha = divideFinite(V,m)*rho/2*S*C_La;
    Phi_i = rtP.lindi.ceb.m;

    // 'divideFinite:29' if numel(B)>1
    // 'divideFinite:31' else
    // 'divideFinite:32' if abs(B)<eps
    if (std::abs(rtP.lindi.ceb.m) < 2.22044605E-16F) {
      // 'divideFinite:33' B(:) = eps;
      Phi_i = 2.22044605E-16F;
    }

    // 'divideFinite:36' C = A ./ B;
    Phi_i = q0_q0 / Phi_i * 1.225F / 2.0F * rtP.lindi.eig.s *
      rtP.lindi.eig.cla_h;

    // :  T = divideFinite( 1, A_alpha );
    // 'divideFinite:29' if numel(B)>1
    // 'divideFinite:31' else
    // 'divideFinite:32' if abs(B)<eps
    if (std::abs(Phi_i) < 2.22044605E-16F) {
      // 'divideFinite:33' B(:) = eps;
      Phi_i = 2.22044605E-16F;
    }

    // Sum: '<S67>/Sum2' incorporates:
    //   Constant: '<S67>/d'
    //   Constant: '<S67>/omega'
    //   DiscreteIntegrator: '<S67>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S67>/Discrete-Time Integrator y_dt'
    //   DiscreteIntegrator: '<S68>/Discrete-Time Integrator y'
    //   Gain: '<S67>/Gain'
    //   Product: '<S67>/Divide'
    //   Product: '<S67>/Product2'
    //   Sum: '<S67>/Sum3'

    // 'divideFinite:36' C = A ./ B;
    rtb_Sum2_dt = rtDW.DiscreteTimeIntegratory_DSTAT_d - (1.0F /
      rtP.lindi.sflt.omega * 2.0F * rtDW.DiscreteTimeIntegratory_dt_DS_b +
      rtDW.DiscreteTimeIntegratory_DSTAT_h);

    // Sum: '<S68>/Sum2' incorporates:
    //   Constant: '<S68>/d'
    //   Constant: '<S68>/omega'
    //   DiscreteIntegrator: '<S61>/Discrete-Time Integrator'
    //   DiscreteIntegrator: '<S68>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S68>/Discrete-Time Integrator y_dt'
    //   Gain: '<S68>/Gain'
    //   Product: '<S68>/Divide'
    //   Product: '<S68>/Product2'
    //   Sum: '<S68>/Sum3'

    rtb_Sum2_p = rtDW.DiscreteTimeIntegrator_DSTATE_e - (1.0F /
      (rtP.lindi.servo.omega * rtP.lindi.servo.boost) * 2.0F *
      rtDW.DiscreteTimeIntegratory_dt_D_lv +
      rtDW.DiscreteTimeIntegratory_DSTAT_d);

    // Sum: '<S62>/Sum2' incorporates:
    //   Constant: '<S62>/d'
    //   Constant: '<S62>/omega'
    //   DiscreteIntegrator: '<S62>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S62>/Discrete-Time Integrator y_dt'
    //   Gain: '<S62>/Gain'
    //   Product: '<S62>/Divide'
    //   Product: '<S62>/Product2'
    //   Sum: '<S62>/Sum3'

    rtb_Sum2_mn = absxk - (1.0F / (2.0F / (2.0F / (rtP.lindi.servo.omega *
      rtP.lindi.servo.boost) + 2.0F / rtP.lindi.sflt.omega)) * 2.0F *
      rtDW.DiscreteTimeIntegratory_dt_D_eo +
      rtDW.DiscreteTimeIntegratory_DSTAT_e);

    // Sum: '<S63>/Sum2' incorporates:
    //   Constant: '<S63>/d'
    //   Constant: '<S63>/omega'
    //   DiscreteIntegrator: '<S61>/Discrete-Time Integrator'
    //   DiscreteIntegrator: '<S63>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S63>/Discrete-Time Integrator y_dt'
    //   Gain: '<S63>/Gain'
    //   Product: '<S63>/Divide'
    //   Product: '<S63>/Product2'
    //   Sum: '<S63>/Sum3'

    rtb_Sum2_g = rtDW.DiscreteTimeIntegrator_DSTATE_e - (1.0F / (2.0F / (2.0F /
      (rtP.lindi.servo.omega * rtP.lindi.servo.boost) + 2.0F /
      rtP.lindi.sflt.omega)) * 2.0F * rtDW.DiscreteTimeIntegratory_dt_D_nr +
      rtDW.DiscreteTimeIntegratory_DSTAT_i);

    // SignalConversion: '<S5>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.wp_idx = rtDW.wp_idx_j;

    // SignalConversion: '<S5>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.stage = rtDW.stage_e;

    // SignalConversion: '<S5>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.t = rtDW.t_g;

    // SignalConversion: '<S5>/BusConversion_InsertedFor_ap_bus_at_inport_0'
    rtDW.V_A_f = q0_q0;

    // Product: '<S43>/Divide' incorporates:
    //   Constant: '<S43>/omega'
    //   Product: '<S43>/omega^2'

    q_bg_unsigned_idx_1 = 2.0F / rtP.lindi.aspd.flttc;

    // Sum: '<S43>/Sum2' incorporates:
    //   Constant: '<S43>/d'
    //   Constant: '<S43>/omega'
    //   DiscreteIntegrator: '<S43>/Discrete-Time Integrator y'
    //   DiscreteIntegrator: '<S43>/Discrete-Time Integrator y_dt'
    //   Gain: '<S43>/Gain'
    //   Inport: '<Root>/measure'
    //   Product: '<S43>/Divide'
    //   Product: '<S43>/Product2'
    //   Sum: '<S43>/Sum3'

    q0_q0 = rtU.measure.airspeed - (1.0F / q_bg_unsigned_idx_1 * 2.0F *
      rtDW.DiscreteTimeIntegratory_dt_D_gf + rtDW.DiscreteTimeIntegratory_DSTATE);

    // Update for DiscreteIntegrator: '<S43>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S43>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_IC_LOAD = 0U;
    rtDW.DiscreteTimeIntegratory_DSTATE += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_D_gf;

    // Update for DiscreteIntegrator: '<S39>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S39>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_IC_LO_a = 0U;
    for (stage_app_2 = 0; stage_app_2 < 9; stage_app_2++) {
      rtDW.DiscreteTimeIntegratory_DSTAT_b[stage_app_2] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_D_ni[stage_app_2];
    }

    // End of Update for DiscreteIntegrator: '<S39>/Discrete-Time Integrator y'

    // Update for DiscreteIntegrator: '<S41>/Discrete-Time Integrator y'
    rtDW.DiscreteTimeIntegratory_IC_LO_e = 0U;

    // Update for DiscreteIntegrator: '<S42>/Discrete-Time Integrator y'
    rtDW.DiscreteTimeIntegratory_IC_LO_i = 0U;

    // Update for DiscreteIntegrator: '<S40>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S40>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTAT_n[0] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_i[0];

    // Update for DiscreteIntegrator: '<S41>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S41>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTA_n2[0] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_p[0];

    // Update for DiscreteIntegrator: '<S42>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S42>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTAT_j[0] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_e[0];

    // Update for DiscreteIntegrator: '<S40>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S40>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTAT_n[1] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_i[1];

    // Update for DiscreteIntegrator: '<S41>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S41>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTA_n2[1] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_p[1];

    // Update for DiscreteIntegrator: '<S42>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S42>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTAT_j[1] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_e[1];

    // Update for DiscreteIntegrator: '<S40>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S40>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTAT_n[2] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_i[2];

    // Update for DiscreteIntegrator: '<S41>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S41>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTA_n2[2] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_p[2];

    // Update for DiscreteIntegrator: '<S42>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S42>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTAT_j[2] += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_e[2];

    // Update for DiscreteIntegrator: '<S38>/Discrete-Time Integrator'
    rtDW.DiscreteTimeIntegrator_DSTATE += 0.0025F;

    // Update for UnitDelay: '<S38>/Unit Delay'
    rtDW.UnitDelay_DSTATE = q1_q1;

    // Update for DiscreteIntegrator: '<S37>/Discrete-Time Integrator'
    rtDW.DiscreteTimeIntegrator_DSTATE_c += 0.0025F;

    // Update for UnitDelay: '<S37>/Unit Delay'
    rtDW.UnitDelay_DSTATE_a = q2_q2;
    for (i = 0; i < 6; i++) {
      // Update for Delay: '<S38>/Delay'
      rtDW.Delay_DSTATE[i] = rtb_y_n[i];

      // Update for Delay: '<S37>/Delay'
      rtDW.Delay_DSTATE_h[i] = rtb_y_n_g[i];
    }

    // Update for UnitDelay: '<S21>/Unit Delay'
    rtDW.UnitDelay_DSTATE_f = scale;

    // Update for DiscreteIntegrator: '<S61>/Discrete-Time Integrator'
    rtDW.DiscreteTimeIntegrator_IC_LOADI = 0U;
    rtDW.DiscreteTimeIntegrator_DSTATE_e += 0.0025F * absxk;

    // Update for DiscreteIntegrator: '<S63>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S63>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_IC_LO_m = 0U;
    rtDW.DiscreteTimeIntegratory_DSTAT_i += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_D_nr;

    // Update for DiscreteIntegrator: '<S62>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S62>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_IC_LO_j = 0U;
    rtDW.DiscreteTimeIntegratory_DSTAT_e += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_D_eo;

    // Update for DiscreteIntegrator: '<S66>/Discrete-Time Integrator' incorporates:
    //   MATLAB Function: '<S64>/MATLAB Function1'
    //   Product: '<S66>/Divide'
    //   Sum: '<S66>/Sum2'

    rtDW.DiscreteTimeIntegrator_DSTATE_h += (t -
      rtDW.DiscreteTimeIntegrator_DSTATE_h) / (1.0F / Phi_i) * 0.0025F;

    // Update for DiscreteIntegrator: '<S67>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S67>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTAT_h += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_b;

    // Update for DiscreteIntegrator: '<S84>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S84>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTAT_p += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DSTA;

    // Update for DiscreteIntegrator: '<S84>/Discrete-Time Integrator y_dt'
    rtDW.DiscreteTimeIntegratory_dt_DSTA += 0.0025F * q0_q3;

    // Update for DiscreteIntegrator: '<S85>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S85>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_IC_LO_b = 0U;
    rtDW.DiscreteTimeIntegratory_DSTA_bw += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_D_pv;

    // Update for DiscreteIntegrator: '<S86>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S86>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_IC_L_an = 0U;
    rtDW.DiscreteTimeIntegratory_DSTAT_m += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_f;

    // Update for DiscreteIntegrator: '<S87>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S87>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_IC_LO_c = 0U;
    rtDW.DiscreteTimeIntegratory_DSTAT_f += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_l;

    // Update for DiscreteIntegrator: '<S92>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S92>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTA_hc += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_g;

    // Update for UnitDelay: '<S22>/Unit Delay' incorporates:
    //   Gain: '<S22>/Gain5'
    //   MATLAB Function: '<S22>/Rudder command'
    //   SignalConversion: '<S94>/TmpSignal ConversionAt SFunction Inport1'

    rtDW.UnitDelay_DSTATE_fx = rtP.lindi.atc.rm.ydecaytc * rtb_u[9];

    // Update for DiscreteIntegrator: '<S100>/Discrete-Time Integrator'
    rtDW.DiscreteTimeIntegrator_IC_LOA_g = 0U;
    rtDW.DiscreteTimeIntegrator_DSTATE_o += 0.0025F * q1_q2;

    // Update for DiscreteIntegrator: '<S102>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S102>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_IC_L_jp = 0U;
    rtDW.DiscreteTimeIntegratory_DSTA_f0 += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_DS_j;

    // Update for DiscreteIntegrator: '<S101>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S101>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_IC_L_bc = 0U;
    rtDW.DiscreteTimeIntegratory_DSTAT_o += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_D_oo;
    for (i = 0; i < 10; i++) {
      // Update for UnitDelay: '<S69>/Unit Delay1'
      rtDW.UnitDelay1_DSTATE[i] = rtb_Delta_u[i];

      // Update for UnitDelay: '<S69>/Unit Delay2'
      rtDW.UnitDelay2_DSTATE[i] = clu[i];

      // Update for DiscreteIntegrator: '<S74>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S74>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_DSTAT_l[i] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_DS_o[i];

      // Update for DiscreteIntegrator: '<S73>/Discrete-Time Integrator y' incorporates:
      //   DiscreteIntegrator: '<S73>/Discrete-Time Integrator y_dt'

      rtDW.DiscreteTimeIntegratory_DSTA_j2[i] += 0.0025F *
        rtDW.DiscreteTimeIntegratory_dt_DS_n[i];

      // Update for DiscreteIntegrator: '<S79>/Discrete-Time Integrator1'
      rtDW.DiscreteTimeIntegrator1_DSTATE[i] += 0.0025F * umin[i];

      // Update for DiscreteIntegrator: '<S79>/Discrete-Time Integrator'
      rtDW.DiscreteTimeIntegrator_DSTATE_l[i] += 0.0025F *
        rtb_DiscreteTimeIntegrator_l[i];

      // Update for DiscreteIntegrator: '<S80>/Discrete-Time Integrator1' incorporates:
      //   DiscreteIntegrator: '<S79>/Discrete-Time Integrator'

      rtDW.DiscreteTimeIntegrator1_DSTAT_j[i] += 0.0025F *
        rtb_DiscreteTimeIntegrator_l[i];

      // Update for DiscreteIntegrator: '<S80>/Discrete-Time Integrator'
      rtDW.DiscreteTimeIntegrator_DSTATE_g[i] += 0.0025F * umax[i];

      // Update for DiscreteIntegrator: '<S74>/Discrete-Time Integrator y_dt' incorporates:
      //   Product: '<S74>/Product1'

      rtDW.DiscreteTimeIntegratory_dt_DS_o[i] += rtb_Sum2_ha[i] * q1_q3 *
        0.0025F;

      // Update for DiscreteIntegrator: '<S73>/Discrete-Time Integrator y_dt' incorporates:
      //   Product: '<S73>/Product1'
      //   Product: '<S73>/omega^2'

      rtDW.DiscreteTimeIntegratory_dt_DS_n[i] += rtb_Sum2_dx[i] * q2_q3 *
        0.0025F;
    }

    // Update for DiscreteIntegrator: '<S39>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S39>/Product1'
    //   Product: '<S39>/omega^2'

    for (stage_app_2 = 0; stage_app_2 < 9; stage_app_2++) {
      rtDW.DiscreteTimeIntegratory_dt_D_ni[stage_app_2] += rtb_M_bg[stage_app_2]
        * q_bg_unsigned_idx_0 * 0.0025F;
    }

    // End of Update for DiscreteIntegrator: '<S39>/Discrete-Time Integrator y_dt' 

    // Update for DiscreteIntegrator: '<S42>/Discrete-Time Integrator y_dt'
    rtDW.DiscreteTimeIntegratory_dt_DS_e[0] += 0.0025F * rtb_y_k[0];

    // Update for DiscreteIntegrator: '<S41>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S41>/Product1'
    //   Product: '<S41>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_DS_p[0] += rtb_Sum2_ci[0] *
      q_bg_unsigned_idx_3 * 0.0025F;

    // Update for DiscreteIntegrator: '<S40>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S40>/Product1'
    //   Product: '<S40>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_DS_i[0] += rtb_y_pd[0] * rtb_omega2_e_0 *
      0.0025F;

    // Update for DiscreteIntegrator: '<S42>/Discrete-Time Integrator y_dt'
    rtDW.DiscreteTimeIntegratory_dt_DS_e[1] += 0.0025F * rtb_y_k[1];

    // Update for DiscreteIntegrator: '<S41>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S41>/Product1'
    //   Product: '<S41>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_DS_p[1] += rtb_Sum2_ci[1] *
      q_bg_unsigned_idx_3 * 0.0025F;

    // Update for DiscreteIntegrator: '<S40>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S40>/Product1'
    //   Product: '<S40>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_DS_i[1] += rtb_y_pd[1] * rtb_omega2_e_0 *
      0.0025F;

    // Update for DiscreteIntegrator: '<S42>/Discrete-Time Integrator y_dt'
    rtDW.DiscreteTimeIntegratory_dt_DS_e[2] += 0.0025F * q_bg_unsigned_idx_2;

    // Update for DiscreteIntegrator: '<S41>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S41>/Product1'
    //   Product: '<S41>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_DS_p[2] += rtb_Sum2_ci[2] *
      q_bg_unsigned_idx_3 * 0.0025F;

    // Update for DiscreteIntegrator: '<S40>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S40>/Product1'
    //   Product: '<S40>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_DS_i[2] += rtb_y_pd[2] * rtb_omega2_e_0 *
      0.0025F;

    // Update for DiscreteIntegrator: '<S101>/Discrete-Time Integrator y_dt' incorporates:
    //   Constant: '<S101>/omega'
    //   Product: '<S101>/Product1'
    //   Product: '<S101>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_D_oo += 2.0F / (2.0F /
      (rtP.lindi.servo.omega * rtP.lindi.servo.boost) + 2.0F /
      rtP.lindi.sflt.omega) * (2.0F / (2.0F / (rtP.lindi.servo.omega *
      rtP.lindi.servo.boost) + 2.0F / rtP.lindi.sflt.omega)) * rtb_Sum2_cx *
      0.0025F;

    // Update for DiscreteIntegrator: '<S102>/Discrete-Time Integrator y_dt' incorporates:
    //   Constant: '<S102>/omega'
    //   Product: '<S102>/Product1'
    //   Product: '<S102>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_DS_j += 2.0F / (2.0F /
      (rtP.lindi.servo.omega * rtP.lindi.servo.boost) + 2.0F /
      rtP.lindi.sflt.omega) * (2.0F / (2.0F / (rtP.lindi.servo.omega *
      rtP.lindi.servo.boost) + 2.0F / rtP.lindi.sflt.omega)) * rtb_Sum2_b *
      0.0025F;

    // Update for DiscreteIntegrator: '<S92>/Discrete-Time Integrator y_dt' incorporates:
    //   Constant: '<S92>/omega'
    //   Product: '<S92>/Product1'
    //   Product: '<S92>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_DS_g += rtP.lindi.sflt.omega *
      rtP.lindi.sflt.omega * rtb_Sum2_lb * 0.0025F;

    // Update for DiscreteIntegrator: '<S93>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S93>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTAT_g += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_D_ir;

    // Product: '<S93>/omega^2' incorporates:
    //   Constant: '<S93>/omega'
    //   Product: '<S68>/omega^2'

    q1_q1 = rtP.lindi.servo.omega * rtP.lindi.servo.boost *
      (rtP.lindi.servo.omega * rtP.lindi.servo.boost);

    // Update for DiscreteIntegrator: '<S93>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S93>/Product1'
    //   Product: '<S93>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_D_ir += q1_q1 * rtb_Sum2_o3 * 0.0025F;

    // Update for DiscreteIntegrator: '<S87>/Discrete-Time Integrator y_dt' incorporates:
    //   Constant: '<S87>/omega'
    //   Product: '<S87>/Product1'
    //   Product: '<S87>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_DS_l += 2.0F / (2.0F /
      (rtP.lindi.servo.omega * rtP.lindi.servo.boost) + 2.0F /
      rtP.lindi.sflt.omega) * (2.0F / (2.0F / (rtP.lindi.servo.omega *
      rtP.lindi.servo.boost) + 2.0F / rtP.lindi.sflt.omega)) * rtb_Sum2_n *
      0.0025F;

    // Update for DiscreteIntegrator: '<S86>/Discrete-Time Integrator y_dt' incorporates:
    //   Constant: '<S86>/omega'
    //   Product: '<S86>/Product1'
    //   Product: '<S86>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_DS_f += 2.0F / (2.0F /
      (rtP.lindi.servo.omega * rtP.lindi.servo.boost) + 2.0F /
      rtP.lindi.sflt.omega) * (2.0F / (2.0F / (rtP.lindi.servo.omega *
      rtP.lindi.servo.boost) + 2.0F / rtP.lindi.sflt.omega)) * rtb_Sum2_po *
      0.0025F;

    // Update for DiscreteIntegrator: '<S85>/Discrete-Time Integrator y_dt' incorporates:
    //   Constant: '<S85>/omega'
    //   Product: '<S85>/Product1'
    //   Product: '<S85>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_D_pv += 2.0F / (2.0F /
      (rtP.lindi.servo.omega * rtP.lindi.servo.boost) + 2.0F /
      rtP.lindi.sflt.omega) * (2.0F / (2.0F / (rtP.lindi.servo.omega *
      rtP.lindi.servo.boost) + 2.0F / rtP.lindi.sflt.omega)) * rtb_Sum2_ec *
      0.0025F;

    // Update for DiscreteIntegrator: '<S67>/Discrete-Time Integrator y_dt' incorporates:
    //   Constant: '<S67>/omega'
    //   Product: '<S67>/Product1'
    //   Product: '<S67>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_DS_b += rtP.lindi.sflt.omega *
      rtP.lindi.sflt.omega * rtb_Sum2_dt * 0.0025F;

    // Update for DiscreteIntegrator: '<S68>/Discrete-Time Integrator y' incorporates:
    //   DiscreteIntegrator: '<S68>/Discrete-Time Integrator y_dt'

    rtDW.DiscreteTimeIntegratory_DSTAT_d += 0.0025F *
      rtDW.DiscreteTimeIntegratory_dt_D_lv;

    // Update for DiscreteIntegrator: '<S68>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S68>/Product1'

    rtDW.DiscreteTimeIntegratory_dt_D_lv += q1_q1 * rtb_Sum2_p * 0.0025F;

    // Update for DiscreteIntegrator: '<S62>/Discrete-Time Integrator y_dt' incorporates:
    //   Constant: '<S62>/omega'
    //   Product: '<S62>/Product1'
    //   Product: '<S62>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_D_eo += 2.0F / (2.0F /
      (rtP.lindi.servo.omega * rtP.lindi.servo.boost) + 2.0F /
      rtP.lindi.sflt.omega) * (2.0F / (2.0F / (rtP.lindi.servo.omega *
      rtP.lindi.servo.boost) + 2.0F / rtP.lindi.sflt.omega)) * rtb_Sum2_mn *
      0.0025F;

    // Update for DiscreteIntegrator: '<S63>/Discrete-Time Integrator y_dt' incorporates:
    //   Constant: '<S63>/omega'
    //   Product: '<S63>/Product1'
    //   Product: '<S63>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_D_nr += 2.0F / (2.0F /
      (rtP.lindi.servo.omega * rtP.lindi.servo.boost) + 2.0F /
      rtP.lindi.sflt.omega) * (2.0F / (2.0F / (rtP.lindi.servo.omega *
      rtP.lindi.servo.boost) + 2.0F / rtP.lindi.sflt.omega)) * rtb_Sum2_g *
      0.0025F;

    // Update for DiscreteIntegrator: '<S43>/Discrete-Time Integrator y_dt' incorporates:
    //   Product: '<S43>/Product1'
    //   Product: '<S43>/omega^2'

    rtDW.DiscreteTimeIntegratory_dt_D_gf += q_bg_unsigned_idx_1 *
      q_bg_unsigned_idx_1 * q0_q0 * 0.0025F;
  } else {
    if (rtDW.LindiPlaneAutopilot_MODE) {
      // Disable for Enabled SubSystem: '<S5>/Flight Path Smoothing'
      if (rtDW.FlightPathSmoothing_MODE) {
        rtDW.FlightPathSmoothing_MODE = false;
      }

      // End of Disable for SubSystem: '<S5>/Flight Path Smoothing'

      // Disable for Enabled SubSystem: '<S5>/NDI Position Controller'
      if (rtDW.NDIPositionController_MODE) {
        rtDW.NDIPositionController_MODE = false;
      }

      // End of Disable for SubSystem: '<S5>/NDI Position Controller'

      // Disable for Enabled SubSystem: '<S5>/Outer Loop INDI'
      if (rtDW.OuterLoopINDI_MODE) {
        rtDW.OuterLoopINDI_MODE = false;
      }

      // End of Disable for SubSystem: '<S5>/Outer Loop INDI'

      // Disable for Enabled SubSystem: '<S5>/Pitch Angle Controller'
      if (rtDW.PitchAngleController_MODE) {
        rtDW.PitchAngleController_MODE = false;
      }

      // End of Disable for SubSystem: '<S5>/Pitch Angle Controller'

      // Disable for Enabled SubSystem: '<S14>/Maneuver Load Alleviation'
      if (rtDW.ManeuverLoadAlleviation_MODE) {
        rtDW.ManeuverLoadAlleviation_MODE = false;
      }

      // End of Disable for SubSystem: '<S14>/Maneuver Load Alleviation'
      rtDW.LindiPlaneAutopilot_MODE = false;
    }
  }

  // End of RelationalOperator: '<S4>/Compare'
  // End of Outputs for SubSystem: '<Root>/LindiPlane Autopilot'

  // MATLAB Function: '<S8>/MATLAB Function'
  // :  y = zeros(11,1,superiorfloat(u));
  // :  num_flaps = (length(u)-3)/2;
  // :  y(1:num_flaps) = u(1:num_flaps);
  // :  y(5:4+num_flaps) = u(num_flaps+1:2*num_flaps);
  rtb_y[0] = rtDW.u[0];
  rtb_y[4] = rtDW.u[4];
  rtb_y[1] = rtDW.u[1];
  rtb_y[5] = rtDW.u[5];
  rtb_y[2] = rtDW.u[2];
  rtb_y[6] = rtDW.u[6];
  rtb_y[3] = rtDW.u[3];
  rtb_y[7] = rtDW.u[7];

  // :  y(end-2:end) = u(end-2:end);
  rtb_y[8] = rtDW.u[8];
  rtb_y[9] = rtDW.u[9];
  rtb_y[10] = rtDW.u[10];

  // Switch: '<S8>/Switch' incorporates:
  //   Gain: '<S8>/Gain'
  //   Gain: '<S8>/Gain7'
  //   Gain: '<S8>/Gain8'
  //   Inport: '<Root>/cmd'

  if (rtb_Compare_b) {
    for (i = 0; i < 8; i++) {
      rtb_y[i] = rtConstP.Gain_Gain_o[i] * rtU.cmd.roll;
    }

    rtb_y[8] = -rtU.cmd.pitch;
    rtb_y[9] = -rtU.cmd.yaw;
    rtb_y[10] = rtU.cmd.thr;
  }

  // End of Switch: '<S8>/Switch'

  // Outport: '<Root>/logs' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion'
  //   Gain: '<S8>/Gain1'
  //   Gain: '<S8>/Gain2'
  //   Gain: '<S8>/Gain3'
  //   Gain: '<S8>/Gain4'
  //   Gain: '<S8>/Gain5'
  //   Gain: '<S8>/Gain6'
  //   Inport: '<Root>/measure'
  //   MATLAB Function: '<S9>/Auxiliary function to define log_config in generated C++ code'
  //   SignalConversion: '<S106>/TmpSignal ConversionAt SFunction Inport2'
  //   Sum: '<Root>/Add'

  rtY.logs[0] = -rtb_y[0];
  rtY.logs[1] = -rtb_y[1];
  rtY.logs[2] = -rtb_y[2];
  rtY.logs[3] = -rtb_y[3];
  rtY.logs[4] = rtb_y[4];
  rtY.logs[5] = rtb_y[5];
  rtY.logs[6] = rtb_y[6];
  rtY.logs[7] = rtb_y[7];
  rtY.logs[8] = -rtb_y[8];
  rtY.logs[9] = -rtb_y[9];
  rtY.logs[10] = rtb_y[10];
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
  for (i = 0; i < 8; i++) {
    rtY.logs[i + 51] = rtU.measure.imu_p[i];
    rtY.logs[i + 59] = rtU.measure.imu_a_z[i];
  }

  // End of Outport: '<Root>/logs'

  // Outport: '<Root>/channels' incorporates:
  //   DataTypeConversion: '<S1>/Data Type Conversion'
  //   Gain: '<S8>/Gain1'
  //   Gain: '<S8>/Gain2'
  //   Gain: '<S8>/Gain3'
  //   Gain: '<S8>/Gain4'
  //   Gain: '<S8>/Gain5'
  //   Gain: '<S8>/Gain6'

  rtY.channels[0] = -rtb_y[0];
  rtY.channels[1] = -rtb_y[1];
  rtY.channels[2] = -rtb_y[2];
  rtY.channels[3] = -rtb_y[3];
  rtY.channels[4] = rtb_y[4];
  rtY.channels[5] = rtb_y[5];
  rtY.channels[6] = rtb_y[6];
  rtY.channels[7] = rtb_y[7];
  rtY.channels[8] = -rtb_y[8];
  rtY.channels[9] = -rtb_y[9];
  rtY.channels[10] = rtb_y[10];
  rtY.channels[11] = 0.0F;
  rtY.channels[12] = 0.0F;
  rtY.channels[13] = 0.0F;
  rtY.channels[14] = 0.0F;
  rtY.channels[15] = 0.0F;
}

// Model initialize function
void MatlabControllerClass::initialize()
{
  {
    int32_T i;

    // ConstCode for Outport: '<Root>/function_channels' incorporates:
    //   Constant: '<S1>/Constant'

    for (i = 0; i < 16; i++) {
      rtY.function_channels[i] = rtConstP.Constant_Value_n[i];
    }

    // End of ConstCode for Outport: '<Root>/function_channels'

    // SystemInitialize for Enabled SubSystem: '<Root>/LindiPlane Autopilot'
    // InitializeConditions for DiscreteIntegrator: '<S43>/Discrete-Time Integrator y' 
    rtDW.DiscreteTimeIntegratory_IC_LOAD = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S39>/Discrete-Time Integrator y' 
    rtDW.DiscreteTimeIntegratory_IC_LO_a = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S41>/Discrete-Time Integrator y' 
    rtDW.DiscreteTimeIntegratory_IC_LO_e = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S42>/Discrete-Time Integrator y' 
    rtDW.DiscreteTimeIntegratory_IC_LO_i = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S61>/Discrete-Time Integrator' 
    rtDW.DiscreteTimeIntegrator_IC_LOADI = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S63>/Discrete-Time Integrator y' 
    rtDW.DiscreteTimeIntegratory_IC_LO_m = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S62>/Discrete-Time Integrator y' 
    rtDW.DiscreteTimeIntegratory_IC_LO_j = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S85>/Discrete-Time Integrator y' 
    rtDW.DiscreteTimeIntegratory_IC_LO_b = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S86>/Discrete-Time Integrator y' 
    rtDW.DiscreteTimeIntegratory_IC_L_an = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S87>/Discrete-Time Integrator y' 
    rtDW.DiscreteTimeIntegratory_IC_LO_c = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S100>/Discrete-Time Integrator' 
    rtDW.DiscreteTimeIntegrator_IC_LOA_g = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S102>/Discrete-Time Integrator y' 
    rtDW.DiscreteTimeIntegratory_IC_L_jp = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S101>/Discrete-Time Integrator y' 
    rtDW.DiscreteTimeIntegratory_IC_L_bc = 1U;

    // SystemInitialize for Enabled SubSystem: '<S5>/Waypoint Navigation'
    // InitializeConditions for UnitDelay: '<S25>/Unit Delay'
    rtDW.UnitDelay_DSTATE_b = 1;

    // InitializeConditions for UnitDelay: '<S25>/Unit Delay2'
    rtDW.UnitDelay2_DSTATE_i = 2;

    // End of SystemInitialize for SubSystem: '<S5>/Waypoint Navigation'

    // SystemInitialize for Enabled SubSystem: '<S5>/Flight Path Smoothing'
    // InitializeConditions for DiscreteIntegrator: '<S30>/Discrete-Time Integrator y' 
    rtDW.DiscreteTimeIntegratory_IC_L_iy = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S29>/Discrete-Time Integrator y' 
    rtDW.DiscreteTimeIntegratory_IC_L_ld = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S28>/Discrete-Time Integrator y' 
    rtDW.DiscreteTimeIntegratory_IC_LO_h = 1U;

    // End of SystemInitialize for SubSystem: '<S5>/Flight Path Smoothing'

    // SystemInitialize for Enabled SubSystem: '<S5>/NDI Position Controller'
    // InitializeConditions for DiscreteIntegrator: '<S50>/Discrete-Time Integrator y' 
    rtDW.DiscreteTimeIntegratory_IC_LO_d = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S49>/Discrete-Time Integrator y' 
    rtDW.DiscreteTimeIntegratory_IC_LO_p = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S51>/Discrete-Time Integrator y' 
    rtDW.DiscreteTimeIntegratory_IC_LO_l = 1U;

    // End of SystemInitialize for SubSystem: '<S5>/NDI Position Controller'

    // SystemInitialize for Enabled SubSystem: '<S5>/Outer Loop INDI'
    // InitializeConditions for DiscreteIntegrator: '<S54>/Discrete-Time Integrator' 
    rtDW.DiscreteTimeIntegrator_IC_LOA_l = 1U;

    // End of SystemInitialize for SubSystem: '<S5>/Outer Loop INDI'

    // SystemInitialize for Enabled SubSystem: '<S5>/Pitch Angle Controller'
    // InitializeConditions for DiscreteIntegrator: '<S57>/Discrete-Time Integrator' 
    rtDW.DiscreteTimeIntegrator_IC_LOA_m = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S58>/Discrete-Time Integrator' 
    rtDW.DiscreteTimeIntegrator_IC_LO_mz = 1U;

    // InitializeConditions for DiscreteIntegrator: '<S59>/Discrete-Time Integrator y' 
    rtDW.DiscreteTimeIntegratory_IC_L_id = 1U;

    // End of SystemInitialize for SubSystem: '<S5>/Pitch Angle Controller'
    // End of SystemInitialize for SubSystem: '<Root>/LindiPlane Autopilot'
  }
}

// Constructor
MatlabControllerClass::MatlabControllerClass()
{
  AP_Param::setup_object_defaults(this, var_info_1);
  AP_Param::setup_object_defaults(this, var_info_2);
  AP_Param::setup_object_defaults(this, var_info_3);
  AP_Param::setup_object_defaults(this, var_info_4);
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
