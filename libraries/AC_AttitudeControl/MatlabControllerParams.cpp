#include <AP_Param/AP_Param.h>
#include "MatlabController.h"


const AP_Param::GroupInfo MatlabControllerClass::var_info_rtP_lindi_ca[] = {
  AP_GROUPINFO("ca_uMin", 0, MatlabControllerClass, rtP.lindi.ca.u_min, 1.000000e-01),
  AP_GROUPINFO("ca_uMax", 1, MatlabControllerClass, rtP.lindi.ca.u_max, 1),
  AP_GROUPINFO("ca_uD", 2, MatlabControllerClass, rtP.lindi.ca.u_d, 1.000000e-01),
  AP_GROUPINFO("ca_Wv1", 3, MatlabControllerClass, rtP.lindi.ca.W_v[0], 10),
  AP_GROUPINFO("ca_Wv2", 4, MatlabControllerClass, rtP.lindi.ca.W_v[1], 10),
  AP_GROUPINFO("ca_Wv3", 5, MatlabControllerClass, rtP.lindi.ca.W_v[2], 1.000000e-02),
  AP_GROUPINFO("ca_Wv4", 6, MatlabControllerClass, rtP.lindi.ca.W_v[3], 1),
  AP_GROUPINFO("ca_Wu1", 7, MatlabControllerClass, rtP.lindi.ca.W_u[0], 1),
  AP_GROUPINFO("ca_Wu2", 8, MatlabControllerClass, rtP.lindi.ca.W_u[1], 1),
  AP_GROUPINFO("ca_Wu3", 9, MatlabControllerClass, rtP.lindi.ca.W_u[2], 1),
  AP_GROUPINFO("ca_Wu4", 10, MatlabControllerClass, rtP.lindi.ca.W_u[3], 1),
  AP_GROUPINFO("ca_gamma", 11, MatlabControllerClass, rtP.lindi.ca.gamma, 1000),
  AP_GROUPINFO("ca_iMax", 12, MatlabControllerClass, rtP.lindi.ca.i_max, 100),
  AP_GROUPEND
};


const AP_Param::GroupInfo MatlabControllerClass::var_info_rtP_lindi_thr[] = {
  AP_GROUPINFO("thr_min", 0, MatlabControllerClass, rtP.lindi.thr.min, 3.110709e-01),
  AP_GROUPINFO("thr_max", 1, MatlabControllerClass, rtP.lindi.thr.max, 8.981805e-01),
  AP_GROUPEND
};


const AP_Param::GroupInfo MatlabControllerClass::var_info_rtP_lindi_cep[] = {
  AP_GROUPINFO("cep_k", 0, MatlabControllerClass, rtP.lindi.cep.k, 1.010421e-05),
  AP_GROUPINFO("cep_d", 1, MatlabControllerClass, rtP.lindi.cep.d, 2.056061e-07),
  AP_GROUPINFO("cep_x1", 2, MatlabControllerClass, rtP.lindi.cep.x[0], 1.700000e-01),
  AP_GROUPINFO("cep_x2", 3, MatlabControllerClass, rtP.lindi.cep.x[1], 1.700000e-01),
  AP_GROUPINFO("cep_x3", 4, MatlabControllerClass, rtP.lindi.cep.x[2], -1.700000e-01),
  AP_GROUPINFO("cep_x4", 5, MatlabControllerClass, rtP.lindi.cep.x[3], -1.700000e-01),
  AP_GROUPINFO("cep_y1", 6, MatlabControllerClass, rtP.lindi.cep.y[0], -1.700000e-01),
  AP_GROUPINFO("cep_y2", 7, MatlabControllerClass, rtP.lindi.cep.y[1], 1.700000e-01),
  AP_GROUPINFO("cep_y3", 8, MatlabControllerClass, rtP.lindi.cep.y[2], 1.700000e-01),
  AP_GROUPINFO("cep_y4", 9, MatlabControllerClass, rtP.lindi.cep.y[3], -1.700000e-01),
  AP_GROUPINFO("cep_z1", 10, MatlabControllerClass, rtP.lindi.cep.z[0], 0),
  AP_GROUPINFO("cep_z2", 11, MatlabControllerClass, rtP.lindi.cep.z[1], 0),
  AP_GROUPINFO("cep_z3", 12, MatlabControllerClass, rtP.lindi.cep.z[2], 0),
  AP_GROUPINFO("cep_z4", 13, MatlabControllerClass, rtP.lindi.cep.z[3], 0),
  AP_GROUPINFO("cep_a1", 14, MatlabControllerClass, rtP.lindi.cep.a[0], -1),
  AP_GROUPINFO("cep_a2", 15, MatlabControllerClass, rtP.lindi.cep.a[1], 1),
  AP_GROUPINFO("cep_a3", 16, MatlabControllerClass, rtP.lindi.cep.a[2], -1),
  AP_GROUPINFO("cep_a4", 17, MatlabControllerClass, rtP.lindi.cep.a[3], 1),
  AP_GROUPINFO("cep_nx1", 18, MatlabControllerClass, rtP.lindi.cep.nx[0], 6.123234e-17),
  AP_GROUPINFO("cep_nx2", 19, MatlabControllerClass, rtP.lindi.cep.nx[1], 6.123234e-17),
  AP_GROUPINFO("cep_nx3", 20, MatlabControllerClass, rtP.lindi.cep.nx[2], 6.123234e-17),
  AP_GROUPINFO("cep_nx4", 21, MatlabControllerClass, rtP.lindi.cep.nx[3], 6.123234e-17),
  AP_GROUPINFO("cep_ny1", 22, MatlabControllerClass, rtP.lindi.cep.ny[0], 0),
  AP_GROUPINFO("cep_ny2", 23, MatlabControllerClass, rtP.lindi.cep.ny[1], 0),
  AP_GROUPINFO("cep_ny3", 24, MatlabControllerClass, rtP.lindi.cep.ny[2], 0),
  AP_GROUPINFO("cep_ny4", 25, MatlabControllerClass, rtP.lindi.cep.ny[3], 0),
  AP_GROUPINFO("cep_ip", 26, MatlabControllerClass, rtP.lindi.cep.ip, 4.100000e-05),
  AP_GROUPINFO("cep_kt", 27, MatlabControllerClass, rtP.lindi.cep.kt, 1.037967e-02),
  AP_GROUPINFO("cep_vb", 28, MatlabControllerClass, rtP.lindi.cep.vb, 1.480000e+01),
  AP_GROUPINFO("cep_ri", 29, MatlabControllerClass, rtP.lindi.cep.ri, 1.150000e-01),
  AP_GROUPEND
};


const AP_Param::GroupInfo MatlabControllerClass::var_info_rtP_lindi_ceb[] = {
  AP_GROUPINFO("ceb_m", 0, MatlabControllerClass, rtP.lindi.ceb.m, 1.335600e+00),
  AP_GROUPINFO("ceb_ixx", 1, MatlabControllerClass, rtP.lindi.ceb.ixx, 1.485000e-02),
  AP_GROUPINFO("ceb_iyy", 2, MatlabControllerClass, rtP.lindi.ceb.iyy, 1.440000e-02),
  AP_GROUPINFO("ceb_izz", 3, MatlabControllerClass, rtP.lindi.ceb.izz, 2.493000e-02),
  AP_GROUPINFO("ceb_ixy", 4, MatlabControllerClass, rtP.lindi.ceb.ixy, 0),
  AP_GROUPINFO("ceb_ixz", 5, MatlabControllerClass, rtP.lindi.ceb.ixz, 0),
  AP_GROUPINFO("ceb_iyz", 6, MatlabControllerClass, rtP.lindi.ceb.iyz, 0),
  AP_GROUPEND
};


const AP_Param::GroupInfo MatlabControllerClass::var_info_rtP_lindi_psc[] = {
  AP_GROUPINFO("psc_rm_accumx", 0, MatlabControllerClass, rtP.lindi.psc.rm.accumax, 2.241002e+01),
  AP_GROUPINFO("psc_rm_accdmx", 1, MatlabControllerClass, rtP.lindi.psc.rm.accdmax, 8.127392e+00),
  AP_GROUPINFO("psc_rm_veldmx", 2, MatlabControllerClass, rtP.lindi.psc.rm.veldmax, 4.737703e+00),
  AP_GROUPINFO("psc_rm_velumx", 3, MatlabControllerClass, rtP.lindi.psc.rm.velumax, 1.566256e+01),
  AP_GROUPINFO("psc_rm_ccxymx", 4, MatlabControllerClass, rtP.lindi.psc.rm.accxymax, 2.978142e+01),
  AP_GROUPINFO("psc_rm_vlxymx", 5, MatlabControllerClass, rtP.lindi.psc.rm.velxymax, 1.832437e+01),
  AP_GROUPINFO("psc_rm_veltc", 6, MatlabControllerClass, rtP.lindi.psc.rm.veltc, 8.789933e-01),
  AP_GROUPINFO("psc_k_pos", 7, MatlabControllerClass, rtP.lindi.psc.k.pos, 1.866764e+00),
  AP_GROUPINFO("psc_k_vel", 8, MatlabControllerClass, rtP.lindi.psc.k.vel, 2.517112e+00),
  AP_GROUPINFO("psc_k_acc", 9, MatlabControllerClass, rtP.lindi.psc.k.acc, 3.057928e-01),
  AP_GROUPEND
};


const AP_Param::GroupInfo MatlabControllerClass::var_info_rtP_lindi_atc[] = {
  AP_GROUPINFO("atc_rm_leanmx", 0, MatlabControllerClass, rtP.lindi.atc.rm.leanmax, 1.290177e+00),
  AP_GROUPINFO("atc_rm_ywrtmx", 1, MatlabControllerClass, rtP.lindi.atc.rm.yawratemax, 6.283185e+00),
  AP_GROUPINFO("atc_rm_ywrttc", 2, MatlabControllerClass, rtP.lindi.atc.rm.yawratetc, 4.272004e-01),
  AP_GROUPINFO("atc_rm_lenfrq", 3, MatlabControllerClass, rtP.lindi.atc.rm.leanfreq, 1.191037e+01),
  AP_GROUPINFO("atc_rm_lendmp", 4, MatlabControllerClass, rtP.lindi.atc.rm.leandamp, 1),
  AP_GROUPINFO("atc_k_yaw", 5, MatlabControllerClass, rtP.lindi.atc.k.yaw, 9.068720e-01),
  AP_GROUPINFO("atc_k_yawrate", 6, MatlabControllerClass, rtP.lindi.atc.k.yawrate, 1.162248e+00),
  AP_GROUPINFO("atc_k_yawacc", 7, MatlabControllerClass, rtP.lindi.atc.k.yawacc, -5.034871e-01),
  AP_GROUPINFO("atc_k_lean", 8, MatlabControllerClass, rtP.lindi.atc.k.lean, 1.194585e+02),
  AP_GROUPINFO("atc_k_leanrat", 9, MatlabControllerClass, rtP.lindi.atc.k.leanrate, 3.008937e+01),
  AP_GROUPINFO("atc_k_leanacc", 10, MatlabControllerClass, rtP.lindi.atc.k.leanacc, 1.526316e+00),
  AP_GROUPEND
};


const AP_Param::GroupInfo MatlabControllerClass::var_info_rtP_lindi_sflt[] = {
  AP_GROUPINFO("sflt_omega", 0, MatlabControllerClass, rtP.lindi.sflt.omega, 7.423973e+01),
  AP_GROUPINFO("sflt_D", 1, MatlabControllerClass, rtP.lindi.sflt.D, 1),
  AP_GROUPEND
};


const AP_Param::GroupInfo MatlabControllerClass::var_info_rtP_lindi_wpnav[] = {
  AP_GROUPINFO("wpnav_T", 0, MatlabControllerClass, rtP.lindi.wpnav.T, 5.037625e-01),
  AP_GROUPINFO("wpnav_wprad", 1, MatlabControllerClass, rtP.lindi.wpnav.wprad, 10),
  AP_GROUPINFO("wpnav_eposmax", 2, MatlabControllerClass, rtP.lindi.wpnav.eposmax, 10),
  AP_GROUPEND
};


const AP_Param::GroupInfo MatlabControllerClass::var_info_rtP_lindi_dive[] = {
  AP_GROUPINFO("dive_amax", 0, MatlabControllerClass, rtP.lindi.dive.amax, 1.962000e+01),
  AP_GROUPINFO("dive_hend", 1, MatlabControllerClass, rtP.lindi.dive.hend, 30),
  AP_GROUPINFO("dive_thrfall", 2, MatlabControllerClass, rtP.lindi.dive.thrfall, -1),
  AP_GROUPINFO("dive_vloiter", 3, MatlabControllerClass, rtP.lindi.dive.vloiter, 8),
  AP_GROUPINFO("dive_tslow", 4, MatlabControllerClass, rtP.lindi.dive.tslow, 5),
  AP_GROUPINFO("dive_ptchslow", 5, MatlabControllerClass, rtP.lindi.dive.ptchslow, 20),
  AP_GROUPINFO("dive_kacc", 6, MatlabControllerClass, rtP.lindi.dive.kacc, 6.000000e-02),
  AP_GROUPEND
};


const AP_Param::GroupInfo MatlabControllerClass::var_info[] = {
  AP_SUBGROUPEXTENSION("", 0, MatlabControllerClass, var_info_rtP_lindi_ca),
  AP_SUBGROUPEXTENSION("", 1, MatlabControllerClass, var_info_rtP_lindi_thr),
  AP_GROUPINFO("rllptch", 2, MatlabControllerClass, rtP.lindi.rllptch, 1),
  AP_SUBGROUPEXTENSION("", 3, MatlabControllerClass, var_info_rtP_lindi_cep),
  AP_SUBGROUPEXTENSION("", 4, MatlabControllerClass, var_info_rtP_lindi_ceb),
  AP_SUBGROUPEXTENSION("", 5, MatlabControllerClass, var_info_rtP_lindi_psc),
  AP_SUBGROUPEXTENSION("", 6, MatlabControllerClass, var_info_rtP_lindi_atc),
  AP_GROUPINFO("mtc", 7, MatlabControllerClass, rtP.lindi.mtc, 4.376376e-02),
  AP_SUBGROUPEXTENSION("", 8, MatlabControllerClass, var_info_rtP_lindi_sflt),
  AP_SUBGROUPEXTENSION("", 9, MatlabControllerClass, var_info_rtP_lindi_wpnav),
  AP_SUBGROUPEXTENSION("", 10, MatlabControllerClass, var_info_rtP_lindi_dive),
  AP_GROUPEND
};
