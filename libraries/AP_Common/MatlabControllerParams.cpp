#include <AP_Param/AP_Param.h>
#include "MatlabController.h"


const AP_Param::GroupInfo MatlabControllerClass::var_info_rtP_lindi_cef[] = {
  AP_GROUPINFO("cef_clu1", 0, MatlabControllerClass, rtP.lindi.cef.clu[0], 1.194588e-01),
  AP_GROUPINFO("cef_clu2", 1, MatlabControllerClass, rtP.lindi.cef.clu[1], 1.558736e-01),
  AP_GROUPINFO("cef_clu3", 2, MatlabControllerClass, rtP.lindi.cef.clu[2], 1.522816e-01),
  AP_GROUPINFO("cef_clu4", 3, MatlabControllerClass, rtP.lindi.cef.clu[3], 1.885155e-01),
  AP_GROUPINFO("cef_clu5", 4, MatlabControllerClass, rtP.lindi.cef.clu[4], 1.885155e-01),
  AP_GROUPINFO("cef_clu6", 5, MatlabControllerClass, rtP.lindi.cef.clu[5], 1.522816e-01),
  AP_GROUPINFO("cef_clu7", 6, MatlabControllerClass, rtP.lindi.cef.clu[6], 1.558736e-01),
  AP_GROUPINFO("cef_clu8", 7, MatlabControllerClass, rtP.lindi.cef.clu[7], 1.194588e-01),
  AP_GROUPINFO("cef_clu9", 8, MatlabControllerClass, rtP.lindi.cef.clu[8], 9.253616e-01),
  AP_GROUPINFO("cef_clu10", 9, MatlabControllerClass, rtP.lindi.cef.clu[9], 8.339239e-01),
  AP_GROUPINFO("cef_s1", 10, MatlabControllerClass, rtP.lindi.cef.s[0], 3.705950e-01),
  AP_GROUPINFO("cef_s2", 11, MatlabControllerClass, rtP.lindi.cef.s[1], 3.705950e-01),
  AP_GROUPINFO("cef_s3", 12, MatlabControllerClass, rtP.lindi.cef.s[2], 3.705950e-01),
  AP_GROUPINFO("cef_s4", 13, MatlabControllerClass, rtP.lindi.cef.s[3], 3.705950e-01),
  AP_GROUPINFO("cef_s5", 14, MatlabControllerClass, rtP.lindi.cef.s[4], 3.705950e-01),
  AP_GROUPINFO("cef_s6", 15, MatlabControllerClass, rtP.lindi.cef.s[5], 3.705950e-01),
  AP_GROUPINFO("cef_s7", 16, MatlabControllerClass, rtP.lindi.cef.s[6], 3.705950e-01),
  AP_GROUPINFO("cef_s8", 17, MatlabControllerClass, rtP.lindi.cef.s[7], 3.705950e-01),
  AP_GROUPINFO("cef_s9", 18, MatlabControllerClass, rtP.lindi.cef.s[8], 5.410600e-02),
  AP_GROUPINFO("cef_s10", 19, MatlabControllerClass, rtP.lindi.cef.s[9], 4.620000e-02),
  AP_GROUPINFO("cef_rotx1", 20, MatlabControllerClass, rtP.lindi.cef.rotx[0], 0),
  AP_GROUPINFO("cef_rotx2", 21, MatlabControllerClass, rtP.lindi.cef.rotx[1], 0),
  AP_GROUPINFO("cef_rotx3", 22, MatlabControllerClass, rtP.lindi.cef.rotx[2], 0),
  AP_GROUPINFO("cef_rotx4", 23, MatlabControllerClass, rtP.lindi.cef.rotx[3], 0),
  AP_GROUPINFO("cef_rotx5", 24, MatlabControllerClass, rtP.lindi.cef.rotx[4], 0),
  AP_GROUPINFO("cef_rotx6", 25, MatlabControllerClass, rtP.lindi.cef.rotx[5], 0),
  AP_GROUPINFO("cef_rotx7", 26, MatlabControllerClass, rtP.lindi.cef.rotx[6], 0),
  AP_GROUPINFO("cef_rotx8", 27, MatlabControllerClass, rtP.lindi.cef.rotx[7], 0),
  AP_GROUPINFO("cef_rotx9", 28, MatlabControllerClass, rtP.lindi.cef.rotx[8], 0),
  AP_GROUPINFO("cef_rotx10", 29, MatlabControllerClass, rtP.lindi.cef.rotx[9], 1.570796e+00),
  AP_GROUPINFO("cef_x1", 30, MatlabControllerClass, rtP.lindi.cef.x[0], -2.580612e-02),
  AP_GROUPINFO("cef_x2", 31, MatlabControllerClass, rtP.lindi.cef.x[1], -3.036353e-02),
  AP_GROUPINFO("cef_x3", 32, MatlabControllerClass, rtP.lindi.cef.x[2], -3.932894e-02),
  AP_GROUPINFO("cef_x4", 33, MatlabControllerClass, rtP.lindi.cef.x[3], -4.228648e-02),
  AP_GROUPINFO("cef_x5", 34, MatlabControllerClass, rtP.lindi.cef.x[4], -4.228648e-02),
  AP_GROUPINFO("cef_x6", 35, MatlabControllerClass, rtP.lindi.cef.x[5], -3.932894e-02),
  AP_GROUPINFO("cef_x7", 36, MatlabControllerClass, rtP.lindi.cef.x[6], -3.036353e-02),
  AP_GROUPINFO("cef_x8", 37, MatlabControllerClass, rtP.lindi.cef.x[7], -2.580612e-02),
  AP_GROUPINFO("cef_x9", 38, MatlabControllerClass, rtP.lindi.cef.x[8], -7.130339e-01),
  AP_GROUPINFO("cef_x10", 39, MatlabControllerClass, rtP.lindi.cef.x[9], -7.427137e-01),
  AP_GROUPINFO("cef_y1", 40, MatlabControllerClass, rtP.lindi.cef.y[0], -6.571736e-01),
  AP_GROUPINFO("cef_y2", 41, MatlabControllerClass, rtP.lindi.cef.y[1], -4.808912e-01),
  AP_GROUPINFO("cef_y3", 42, MatlabControllerClass, rtP.lindi.cef.y[2], -3.157907e-01),
  AP_GROUPINFO("cef_y4", 43, MatlabControllerClass, rtP.lindi.cef.y[3], -1.542614e-01),
  AP_GROUPINFO("cef_y5", 44, MatlabControllerClass, rtP.lindi.cef.y[4], 1.542614e-01),
  AP_GROUPINFO("cef_y6", 45, MatlabControllerClass, rtP.lindi.cef.y[5], 3.157907e-01),
  AP_GROUPINFO("cef_y7", 46, MatlabControllerClass, rtP.lindi.cef.y[6], 4.808912e-01),
  AP_GROUPINFO("cef_y8", 47, MatlabControllerClass, rtP.lindi.cef.y[7], 6.571736e-01),
  AP_GROUPINFO("cef_y9", 48, MatlabControllerClass, rtP.lindi.cef.y[8], -2.081668e-17),
  AP_GROUPINFO("cef_y10", 49, MatlabControllerClass, rtP.lindi.cef.y[9], -1.387779e-17),
  AP_GROUPINFO("cef_z1", 50, MatlabControllerClass, rtP.lindi.cef.z[0], -5.000000e-02),
  AP_GROUPINFO("cef_z2", 51, MatlabControllerClass, rtP.lindi.cef.z[1], -5.000000e-02),
  AP_GROUPINFO("cef_z3", 52, MatlabControllerClass, rtP.lindi.cef.z[2], -5.000000e-02),
  AP_GROUPINFO("cef_z4", 53, MatlabControllerClass, rtP.lindi.cef.z[3], -5.000000e-02),
  AP_GROUPINFO("cef_z5", 54, MatlabControllerClass, rtP.lindi.cef.z[4], -5.000000e-02),
  AP_GROUPINFO("cef_z6", 55, MatlabControllerClass, rtP.lindi.cef.z[5], -5.000000e-02),
  AP_GROUPINFO("cef_z7", 56, MatlabControllerClass, rtP.lindi.cef.z[6], -5.000000e-02),
  AP_GROUPINFO("cef_z8", 57, MatlabControllerClass, rtP.lindi.cef.z[7], -5.000000e-02),
  AP_GROUPINFO("cef_z9", 58, MatlabControllerClass, rtP.lindi.cef.z[8], -4.000000e-02),
  AP_GROUPINFO("cef_z10", 59, MatlabControllerClass, rtP.lindi.cef.z[9], -1.000000e-01),
  AP_GROUPEND
};


const AP_Param::GroupInfo MatlabControllerClass::var_info_rtP_lindi_ceb[] = {
  AP_GROUPINFO("ceb_m", 0, MatlabControllerClass, rtP.lindi.ceb.m, 2.330000e+00),
  AP_GROUPINFO("ceb_ixx", 1, MatlabControllerClass, rtP.lindi.ceb.ixx, 2.185000e-01),
  AP_GROUPINFO("ceb_iyy", 2, MatlabControllerClass, rtP.lindi.ceb.iyy, 1.783000e-01),
  AP_GROUPINFO("ceb_izz", 3, MatlabControllerClass, rtP.lindi.ceb.izz, 3.920000e-01),
  AP_GROUPINFO("ceb_ixy", 4, MatlabControllerClass, rtP.lindi.ceb.ixy, 0),
  AP_GROUPINFO("ceb_ixz", 5, MatlabControllerClass, rtP.lindi.ceb.ixz, 0),
  AP_GROUPINFO("ceb_iyz", 6, MatlabControllerClass, rtP.lindi.ceb.iyz, 0),
  AP_GROUPINFO("ceb_scale", 7, MatlabControllerClass, rtP.lindi.ceb.scale, 1.200000e+00),
  AP_GROUPEND
};


const AP_Param::GroupInfo MatlabControllerClass::var_info_rtP_lindi_servo[] = {
  AP_GROUPINFO("servo_omega", 0, MatlabControllerClass, rtP.lindi.servo.omega, 200),
  AP_GROUPINFO("servo_d", 1, MatlabControllerClass, rtP.lindi.servo.d, 8.660000e-01),
  AP_GROUPINFO("servo_delay", 2, MatlabControllerClass, rtP.lindi.servo.delay, 7.500000e-03),
  AP_GROUPINFO("servo_boost", 3, MatlabControllerClass, rtP.lindi.servo.boost, 5.000000e-01),
  AP_GROUPEND
};


const AP_Param::GroupInfo MatlabControllerClass::var_info_rtP_lindi_sflt[] = {
  AP_GROUPINFO("sflt_omega", 0, MatlabControllerClass, rtP.lindi.sflt.omega, 50),
  AP_GROUPINFO("sflt_d", 1, MatlabControllerClass, rtP.lindi.sflt.d, 7.100000e-01),
  AP_GROUPINFO("sflt_nmGyrFlt", 2, MatlabControllerClass, rtP.lindi.sflt.numGyrFlt, 2),
  AP_GROUPEND
};


const AP_Param::GroupInfo MatlabControllerClass::var_info_rtP_lindi_aspd[] = {
  AP_GROUPINFO("aspd_flttc", 0, MatlabControllerClass, rtP.lindi.aspd.flttc, 1.000000e-01),
  AP_GROUPINFO("aspd_min", 1, MatlabControllerClass, rtP.lindi.aspd.min, 1.003482e+01),
  AP_GROUPEND
};


const AP_Param::GroupInfo MatlabControllerClass::var_info_rtP_lindi_atc[] = {
  AP_GROUPINFO("atc_k_rang", 0, MatlabControllerClass, rtP.lindi.atc.k.rang, 3.333333e+01),
  AP_GROUPINFO("atc_k_rrat", 1, MatlabControllerClass, rtP.lindi.atc.k.rrat, 10),
  AP_GROUPINFO("atc_k_racc", 2, MatlabControllerClass, rtP.lindi.atc.k.racc, 2.000000e-01),
  AP_GROUPINFO("atc_k_pang", 3, MatlabControllerClass, rtP.lindi.atc.k.pang, 3.333333e+01),
  AP_GROUPINFO("atc_k_prat", 4, MatlabControllerClass, rtP.lindi.atc.k.prat, 10),
  AP_GROUPINFO("atc_k_pacc", 5, MatlabControllerClass, rtP.lindi.atc.k.pacc, 2.000000e-01),
  AP_GROUPINFO("atc_k_yrat", 6, MatlabControllerClass, rtP.lindi.atc.k.yrat, 4.166667e+00),
  AP_GROUPINFO("atc_k_yacc", 7, MatlabControllerClass, rtP.lindi.atc.k.yacc, 2.432795e-16),
  AP_GROUPINFO("atc_rm_rfreq", 8, MatlabControllerClass, rtP.lindi.atc.rm.rfreq, 10),
  AP_GROUPINFO("atc_rm_rangmx", 9, MatlabControllerClass, rtP.lindi.atc.rm.rangmax, 70),
  AP_GROUPINFO("atc_rm_rratmx", 10, MatlabControllerClass, rtP.lindi.atc.rm.rratmax, 70),
  AP_GROUPINFO("atc_rm_pfreq", 11, MatlabControllerClass, rtP.lindi.atc.rm.pfreq, 5),
  AP_GROUPINFO("atc_rm_pangmx", 12, MatlabControllerClass, rtP.lindi.atc.rm.pangmax, 30),
  AP_GROUPINFO("atc_rm_pratmx", 13, MatlabControllerClass, rtP.lindi.atc.rm.pratmax, 60),
  AP_GROUPINFO("atc_rm_yfreq", 14, MatlabControllerClass, rtP.lindi.atc.rm.yfreq, 5),
  AP_GROUPINFO("atc_rm_yratmx", 15, MatlabControllerClass, rtP.lindi.atc.rm.yratmax, 60),
  AP_GROUPINFO("atc_rm_ydcytc", 16, MatlabControllerClass, rtP.lindi.atc.rm.ydecaytc, 4.000000e-01),
  AP_GROUPEND
};


const AP_Param::GroupInfo MatlabControllerClass::var_info_rtP_lindi_eig[] = {
  AP_GROUPINFO("eig_clp", 0, MatlabControllerClass, rtP.lindi.eig.clp, -5.588553e-01),
  AP_GROUPINFO("eig_b", 1, MatlabControllerClass, rtP.lindi.eig.b, 2),
  AP_GROUPINFO("eig_s", 2, MatlabControllerClass, rtP.lindi.eig.s, 3.705950e-01),
  AP_GROUPINFO("eig_claH", 3, MatlabControllerClass, rtP.lindi.eig.cla_h, 4.080931e+00),
  AP_GROUPINFO("eig_xH", 4, MatlabControllerClass, rtP.lindi.eig.x_h, 7.130339e-01),
  AP_GROUPINFO("eig_sH", 5, MatlabControllerClass, rtP.lindi.eig.s_h, 5.410600e-02),
  AP_GROUPINFO("eig_cla", 6, MatlabControllerClass, rtP.lindi.eig.cla, 5.093642e+00),
  AP_GROUPINFO("eig_dahda", 7, MatlabControllerClass, rtP.lindi.eig.dahda, -3.549998e-01),
  AP_GROUPINFO("eig_dahdu1", 8, MatlabControllerClass, rtP.lindi.eig.dahdu[0], 4.224818e-03),
  AP_GROUPINFO("eig_dahdu2", 9, MatlabControllerClass, rtP.lindi.eig.dahdu[1], 8.214076e-03),
  AP_GROUPINFO("eig_dahdu3", 10, MatlabControllerClass, rtP.lindi.eig.dahdu[2], 7.521633e-03),
  AP_GROUPINFO("eig_dahdu4", 11, MatlabControllerClass, rtP.lindi.eig.dahdu[3], -4.461100e-02),
  AP_GROUPINFO("eig_dahdu5", 12, MatlabControllerClass, rtP.lindi.eig.dahdu[4], -4.461100e-02),
  AP_GROUPINFO("eig_dahdu6", 13, MatlabControllerClass, rtP.lindi.eig.dahdu[5], 7.521633e-03),
  AP_GROUPINFO("eig_dahdu7", 14, MatlabControllerClass, rtP.lindi.eig.dahdu[6], 8.214076e-03),
  AP_GROUPINFO("eig_dahdu8", 15, MatlabControllerClass, rtP.lindi.eig.dahdu[7], 4.224818e-03),
  AP_GROUPINFO("eig_xcg", 16, MatlabControllerClass, rtP.lindi.eig.xcg, -4.400000e-01),
  AP_GROUPINFO("eig_xnp", 17, MatlabControllerClass, rtP.lindi.eig.xnp, -4.827121e-01),
  AP_GROUPINFO("eig_xnp0", 18, MatlabControllerClass, rtP.lindi.eig.xnp0, -5.068001e-01),
  AP_GROUPEND
};


const AP_Param::GroupInfo MatlabControllerClass::var_info_rtP_lindi_psc[] = {
  AP_GROUPINFO("psc_k_pos", 0, MatlabControllerClass, rtP.lindi.psc.k.pos, 1.027285e+00),
  AP_GROUPINFO("psc_k_vel", 1, MatlabControllerClass, rtP.lindi.psc.k.vel, 1.602564e+00),
  AP_GROUPINFO("psc_k_acc", 2, MatlabControllerClass, rtP.lindi.psc.k.acc, -8.767829e-16),
  AP_GROUPEND
};


const AP_Param::GroupInfo MatlabControllerClass::var_info_rtP_lindi_wpnav[] = {
  AP_GROUPINFO("wpnav_T", 0, MatlabControllerClass, rtP.lindi.wpnav.T, 6.000000e-01),
  AP_GROUPINFO("wpnav_wprad", 1, MatlabControllerClass, rtP.lindi.wpnav.wprad, 60),
  AP_GROUPINFO("wpnav_eposmax", 2, MatlabControllerClass, rtP.lindi.wpnav.eposmax, 20),
  AP_GROUPEND
};


const AP_Param::GroupInfo MatlabControllerClass::var_info_rtP_lindi_ca[] = {
  AP_GROUPINFO("ca_uMin1", 0, MatlabControllerClass, rtP.lindi.ca.u_min[0], -1),
  AP_GROUPINFO("ca_uMin2", 1, MatlabControllerClass, rtP.lindi.ca.u_min[1], -1),
  AP_GROUPINFO("ca_uMin3", 2, MatlabControllerClass, rtP.lindi.ca.u_min[2], -1),
  AP_GROUPINFO("ca_uMin4", 3, MatlabControllerClass, rtP.lindi.ca.u_min[3], -1),
  AP_GROUPINFO("ca_uMin5", 4, MatlabControllerClass, rtP.lindi.ca.u_min[4], -1),
  AP_GROUPINFO("ca_uMin6", 5, MatlabControllerClass, rtP.lindi.ca.u_min[5], -1),
  AP_GROUPINFO("ca_uMin7", 6, MatlabControllerClass, rtP.lindi.ca.u_min[6], -1),
  AP_GROUPINFO("ca_uMin8", 7, MatlabControllerClass, rtP.lindi.ca.u_min[7], -1),
  AP_GROUPINFO("ca_uMin9", 8, MatlabControllerClass, rtP.lindi.ca.u_min[8], -1),
  AP_GROUPINFO("ca_uMin10", 9, MatlabControllerClass, rtP.lindi.ca.u_min[9], -1),
  AP_GROUPINFO("ca_uMax1", 10, MatlabControllerClass, rtP.lindi.ca.u_max[0], 1),
  AP_GROUPINFO("ca_uMax2", 11, MatlabControllerClass, rtP.lindi.ca.u_max[1], 1),
  AP_GROUPINFO("ca_uMax3", 12, MatlabControllerClass, rtP.lindi.ca.u_max[2], 1),
  AP_GROUPINFO("ca_uMax4", 13, MatlabControllerClass, rtP.lindi.ca.u_max[3], 1),
  AP_GROUPINFO("ca_uMax5", 14, MatlabControllerClass, rtP.lindi.ca.u_max[4], 1),
  AP_GROUPINFO("ca_uMax6", 15, MatlabControllerClass, rtP.lindi.ca.u_max[5], 1),
  AP_GROUPINFO("ca_uMax7", 16, MatlabControllerClass, rtP.lindi.ca.u_max[6], 1),
  AP_GROUPINFO("ca_uMax8", 17, MatlabControllerClass, rtP.lindi.ca.u_max[7], 1),
  AP_GROUPINFO("ca_uMax9", 18, MatlabControllerClass, rtP.lindi.ca.u_max[8], 1),
  AP_GROUPINFO("ca_uMax10", 19, MatlabControllerClass, rtP.lindi.ca.u_max[9], 1),
  AP_GROUPINFO("ca_uD1", 20, MatlabControllerClass, rtP.lindi.ca.u_d[0], 0),
  AP_GROUPINFO("ca_uD2", 21, MatlabControllerClass, rtP.lindi.ca.u_d[1], 0),
  AP_GROUPINFO("ca_uD3", 22, MatlabControllerClass, rtP.lindi.ca.u_d[2], 0),
  AP_GROUPINFO("ca_uD4", 23, MatlabControllerClass, rtP.lindi.ca.u_d[3], 0),
  AP_GROUPINFO("ca_uD5", 24, MatlabControllerClass, rtP.lindi.ca.u_d[4], 0),
  AP_GROUPINFO("ca_uD6", 25, MatlabControllerClass, rtP.lindi.ca.u_d[5], 0),
  AP_GROUPINFO("ca_uD7", 26, MatlabControllerClass, rtP.lindi.ca.u_d[6], 0),
  AP_GROUPINFO("ca_uD8", 27, MatlabControllerClass, rtP.lindi.ca.u_d[7], 0),
  AP_GROUPINFO("ca_uD9", 28, MatlabControllerClass, rtP.lindi.ca.u_d[8], 0),
  AP_GROUPINFO("ca_uD10", 29, MatlabControllerClass, rtP.lindi.ca.u_d[9], 0),
  AP_GROUPINFO("ca_Wv1", 30, MatlabControllerClass, rtP.lindi.ca.W_v[0], 1),
  AP_GROUPINFO("ca_Wv2", 31, MatlabControllerClass, rtP.lindi.ca.W_v[1], 1),
  AP_GROUPINFO("ca_Wv3", 32, MatlabControllerClass, rtP.lindi.ca.W_v[2], 1),
  AP_GROUPINFO("ca_Wv4", 33, MatlabControllerClass, rtP.lindi.ca.W_v[3], 1),
  AP_GROUPINFO("ca_Wu1", 34, MatlabControllerClass, rtP.lindi.ca.W_u[0], 1),
  AP_GROUPINFO("ca_Wu2", 35, MatlabControllerClass, rtP.lindi.ca.W_u[1], 1),
  AP_GROUPINFO("ca_Wu3", 36, MatlabControllerClass, rtP.lindi.ca.W_u[2], 1),
  AP_GROUPINFO("ca_Wu4", 37, MatlabControllerClass, rtP.lindi.ca.W_u[3], 1),
  AP_GROUPINFO("ca_Wu5", 38, MatlabControllerClass, rtP.lindi.ca.W_u[4], 1),
  AP_GROUPINFO("ca_Wu6", 39, MatlabControllerClass, rtP.lindi.ca.W_u[5], 1),
  AP_GROUPINFO("ca_Wu7", 40, MatlabControllerClass, rtP.lindi.ca.W_u[6], 1),
  AP_GROUPINFO("ca_Wu8", 41, MatlabControllerClass, rtP.lindi.ca.W_u[7], 1),
  AP_GROUPINFO("ca_Wu9", 42, MatlabControllerClass, rtP.lindi.ca.W_u[8], 1.000000e-01),
  AP_GROUPINFO("ca_Wu10", 43, MatlabControllerClass, rtP.lindi.ca.W_u[9], 1),
  AP_GROUPINFO("ca_gamma", 44, MatlabControllerClass, rtP.lindi.ca.gamma, 1000),
  AP_GROUPINFO("ca_W1", 45, MatlabControllerClass, rtP.lindi.ca.W[0], 0),
  AP_GROUPINFO("ca_W2", 46, MatlabControllerClass, rtP.lindi.ca.W[1], 0),
  AP_GROUPINFO("ca_W3", 47, MatlabControllerClass, rtP.lindi.ca.W[2], 0),
  AP_GROUPINFO("ca_W4", 48, MatlabControllerClass, rtP.lindi.ca.W[3], 0),
  AP_GROUPINFO("ca_W5", 49, MatlabControllerClass, rtP.lindi.ca.W[4], 0),
  AP_GROUPINFO("ca_W6", 50, MatlabControllerClass, rtP.lindi.ca.W[5], 0),
  AP_GROUPINFO("ca_W7", 51, MatlabControllerClass, rtP.lindi.ca.W[6], 0),
  AP_GROUPINFO("ca_W8", 52, MatlabControllerClass, rtP.lindi.ca.W[7], 0),
  AP_GROUPINFO("ca_W9", 53, MatlabControllerClass, rtP.lindi.ca.W[8], 0),
  AP_GROUPINFO("ca_W10", 54, MatlabControllerClass, rtP.lindi.ca.W[9], 0),
  AP_GROUPINFO("ca_iMax", 55, MatlabControllerClass, rtP.lindi.ca.i_max, 1),
  AP_GROUPEND
};


const AP_Param::GroupInfo MatlabControllerClass::var_info_rtP_lindi_dlc[] = {
  AP_GROUPINFO("dlc_opt", 0, MatlabControllerClass, rtP.lindi.dlc.opt, 1),
  AP_GROUPINFO("dlc_flapdecay", 1, MatlabControllerClass, rtP.lindi.dlc.flapdecay, 4),
  AP_GROUPINFO("dlc_maxptch", 2, MatlabControllerClass, rtP.lindi.dlc.maxptch, 20),
  AP_GROUPEND
};


const AP_Param::GroupInfo MatlabControllerClass::var_info_rtP_lindi_mla[] = {
  AP_GROUPINFO("mla_use", 0, MatlabControllerClass, rtP.lindi.mla.use, 0),
  AP_GROUPINFO("mla_etaNp1", 1, MatlabControllerClass, rtP.lindi.mla.eta_np[0], -4.292534e-01),
  AP_GROUPINFO("mla_etaNp2", 2, MatlabControllerClass, rtP.lindi.mla.eta_np[1], 4.292534e-01),
  AP_GROUPINFO("mla_ca_Wv1", 3, MatlabControllerClass, rtP.lindi.mla.ca.W_v[0], 1),
  AP_GROUPINFO("mla_ca_Wv2", 4, MatlabControllerClass, rtP.lindi.mla.ca.W_v[1], 1),
  AP_GROUPINFO("mla_ca_Wv3", 5, MatlabControllerClass, rtP.lindi.mla.ca.W_v[2], 10),
  AP_GROUPINFO("mla_ca_Wv4", 6, MatlabControllerClass, rtP.lindi.mla.ca.W_v[3], 10),
  AP_GROUPINFO("mla_ca_Wu1", 7, MatlabControllerClass, rtP.lindi.mla.ca.W_u[0], 1),
  AP_GROUPINFO("mla_ca_Wu2", 8, MatlabControllerClass, rtP.lindi.mla.ca.W_u[1], 1),
  AP_GROUPINFO("mla_ca_Wu3", 9, MatlabControllerClass, rtP.lindi.mla.ca.W_u[2], 1),
  AP_GROUPINFO("mla_ca_Wu4", 10, MatlabControllerClass, rtP.lindi.mla.ca.W_u[3], 1),
  AP_GROUPINFO("mla_ca_Wu5", 11, MatlabControllerClass, rtP.lindi.mla.ca.W_u[4], 1),
  AP_GROUPINFO("mla_ca_Wu6", 12, MatlabControllerClass, rtP.lindi.mla.ca.W_u[5], 1),
  AP_GROUPINFO("mla_ca_Wu7", 13, MatlabControllerClass, rtP.lindi.mla.ca.W_u[6], 1),
  AP_GROUPINFO("mla_ca_Wu8", 14, MatlabControllerClass, rtP.lindi.mla.ca.W_u[7], 1),
  AP_GROUPINFO("mla_ca_Wu9", 15, MatlabControllerClass, rtP.lindi.mla.ca.W_u[8], 1.000000e-01),
  AP_GROUPINFO("mla_ca_Wu10", 16, MatlabControllerClass, rtP.lindi.mla.ca.W_u[9], 1),
  AP_GROUPINFO("mla_ca_gamma", 17, MatlabControllerClass, rtP.lindi.mla.ca.gamma, 1000),
  AP_GROUPINFO("mla_ca_iMax", 18, MatlabControllerClass, rtP.lindi.mla.ca.i_max, 1),
  AP_GROUPEND
};


const AP_Param::GroupInfo MatlabControllerClass::var_info_rtP_lindi_gust[] = {
  AP_GROUPINFO("gust_len", 0, MatlabControllerClass, rtP.lindi.gust.len, 20),
  AP_GROUPINFO("gust_mag", 1, MatlabControllerClass, rtP.lindi.gust.mag, -3),
  AP_GROUPEND
};


const AP_Param::GroupInfo MatlabControllerClass::var_info[] = {
  AP_SUBGROUPEXTENSION("", 0, MatlabControllerClass, var_info_rtP_lindi_cef),
  AP_SUBGROUPEXTENSION("", 1, MatlabControllerClass, var_info_rtP_lindi_ceb),
  AP_SUBGROUPEXTENSION("", 2, MatlabControllerClass, var_info_rtP_lindi_servo),
  AP_SUBGROUPEXTENSION("", 3, MatlabControllerClass, var_info_rtP_lindi_sflt),
  AP_SUBGROUPEXTENSION("", 4, MatlabControllerClass, var_info_rtP_lindi_aspd),
  AP_SUBGROUPEXTENSION("", 5, MatlabControllerClass, var_info_rtP_lindi_atc),
  AP_SUBGROUPEXTENSION("", 6, MatlabControllerClass, var_info_rtP_lindi_eig),
  AP_SUBGROUPEXTENSION("", 7, MatlabControllerClass, var_info_rtP_lindi_psc),
  AP_SUBGROUPEXTENSION("", 8, MatlabControllerClass, var_info_rtP_lindi_wpnav),
  AP_SUBGROUPEXTENSION("", 9, MatlabControllerClass, var_info_rtP_lindi_ca),
  AP_SUBGROUPEXTENSION("", 10, MatlabControllerClass, var_info_rtP_lindi_dlc),
  AP_SUBGROUPEXTENSION("", 11, MatlabControllerClass, var_info_rtP_lindi_mla),
  AP_SUBGROUPEXTENSION("", 12, MatlabControllerClass, var_info_rtP_lindi_gust),
  AP_GROUPEND
};
