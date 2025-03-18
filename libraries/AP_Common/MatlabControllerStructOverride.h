#include <AP_Param/AP_Param.h>


#define DEFINED_TYPEDEF_FOR_struct_pIgE2dka5KIuyGebGLL8dB_

typedef struct {
  AP_Float W_v[4];
  AP_Float W_u[10];
  AP_Float gamma;
  AP_Float i_max;
} struct_pIgE2dka5KIuyGebGLL8dB;


#define DEFINED_TYPEDEF_FOR_struct_wtF2XFOtOELBemGDyRar7G_

typedef struct {
  AP_Float pos;
  AP_Float vel;
  AP_Float acc;
} struct_wtF2XFOtOELBemGDyRar7G;


#define DEFINED_TYPEDEF_FOR_struct_iPhZVYNbhiJ7uk1NWTPeLC_

typedef struct {
  AP_Float rfreq;
  AP_Float rangmax;
  AP_Float rratmax;
  AP_Float pfreq;
  AP_Float pangmax;
  AP_Float pratmax;
  AP_Float yfreq;
  AP_Float yratmax;
  AP_Float ydecaytc;
} struct_iPhZVYNbhiJ7uk1NWTPeLC;


#define DEFINED_TYPEDEF_FOR_struct_6BRWbv67APlqiRCPZEkE1C_

typedef struct {
  AP_Float rang;
  AP_Float rrat;
  AP_Float racc;
  AP_Float pang;
  AP_Float prat;
  AP_Float pacc;
  AP_Float yrat;
  AP_Float yacc;
} struct_6BRWbv67APlqiRCPZEkE1C;


#define DEFINED_TYPEDEF_FOR_struct_zTGPqEU9oPa0g1yI2jOg2F_

typedef struct {
  AP_Float len;
  AP_Float mag;
} struct_zTGPqEU9oPa0g1yI2jOg2F;


#define DEFINED_TYPEDEF_FOR_struct_grmQpGhO6u50ZJYAi79l7E_

typedef struct {
  AP_Float use;
  AP_Float eta_np[2];
  struct_pIgE2dka5KIuyGebGLL8dB ca;
} struct_grmQpGhO6u50ZJYAi79l7E;


#define DEFINED_TYPEDEF_FOR_struct_9VdX5G3wISWD67sX4adsqF_

typedef struct {
  AP_Float opt;
  AP_Float flapdecay;
  AP_Float maxptch;
} struct_9VdX5G3wISWD67sX4adsqF;


#define DEFINED_TYPEDEF_FOR_struct_t97zePNXqrora5Yj9ePAsF_

typedef struct {
  AP_Float u_min[10];
  AP_Float u_max[10];
  AP_Float u_d[10];
  AP_Float W_v[4];
  AP_Float W_u[10];
  AP_Float gamma;
  AP_Float W[10];
  AP_Float i_max;
} struct_t97zePNXqrora5Yj9ePAsF;


#define DEFINED_TYPEDEF_FOR_struct_ZpRNaqoZiEb6XqsMOrzR0C_

typedef struct {
  AP_Float T;
  AP_Float wprad;
  AP_Float eposmax;
} struct_ZpRNaqoZiEb6XqsMOrzR0C;


#define DEFINED_TYPEDEF_FOR_struct_nUdewYMN9W2ArPMLvxYBeD_

typedef struct {
  struct_wtF2XFOtOELBemGDyRar7G k;
} struct_nUdewYMN9W2ArPMLvxYBeD;


#define DEFINED_TYPEDEF_FOR_struct_Byly2JHaAJfFWirb8CScwH_

typedef struct {
  AP_Float clp;
  AP_Float b;
  AP_Float s;
  AP_Float cla_h;
  AP_Float x_h;
  AP_Float s_h;
  AP_Float cla;
  AP_Float dahda;
  AP_Float dahdu[8];
  AP_Float xcg;
  AP_Float xnp;
  AP_Float xnp0;
} struct_Byly2JHaAJfFWirb8CScwH;


#define DEFINED_TYPEDEF_FOR_struct_zFuKxOdy6xWCgrwijIt3EB_

typedef struct {
  struct_6BRWbv67APlqiRCPZEkE1C k;
  struct_iPhZVYNbhiJ7uk1NWTPeLC rm;
} struct_zFuKxOdy6xWCgrwijIt3EB;


#define DEFINED_TYPEDEF_FOR_struct_sbKx6IyVlWG2aZGxKuo5AB_

typedef struct {
  AP_Float flttc;
  AP_Float min;
} struct_sbKx6IyVlWG2aZGxKuo5AB;


#define DEFINED_TYPEDEF_FOR_struct_XfqFc28dIjnSEgUD39JtWG_

typedef struct {
  AP_Float omega;
  AP_Float d;
  AP_Float numGyrFlt;
} struct_XfqFc28dIjnSEgUD39JtWG;


#define DEFINED_TYPEDEF_FOR_struct_D5pBfL2LPQt2lUKFw6GkNC_

typedef struct {
  AP_Float omega;
  AP_Float d;
  AP_Float delay;
  AP_Float boost;
} struct_D5pBfL2LPQt2lUKFw6GkNC;


#define DEFINED_TYPEDEF_FOR_struct_NVgzPUNgrhoXyl8OD5Cw5F_

typedef struct {
  AP_Float m;
  AP_Float ixx;
  AP_Float iyy;
  AP_Float izz;
  AP_Float ixy;
  AP_Float ixz;
  AP_Float iyz;
  AP_Float scale;
} struct_NVgzPUNgrhoXyl8OD5Cw5F;


#define DEFINED_TYPEDEF_FOR_struct_YFejTC3Mb5lyJ7XHyQfyqG_

typedef struct {
  AP_Float clu[10];
  AP_Float s[10];
  AP_Float rotx[10];
  AP_Float x[10];
  AP_Float y[10];
  AP_Float z[10];
} struct_YFejTC3Mb5lyJ7XHyQfyqG;


#define DEFINED_TYPEDEF_FOR_struct_rGAsXWLaUsbxrXrQ2KHClH_

typedef struct {
  struct_YFejTC3Mb5lyJ7XHyQfyqG cef;
  struct_NVgzPUNgrhoXyl8OD5Cw5F ceb;
  struct_D5pBfL2LPQt2lUKFw6GkNC servo;
  struct_XfqFc28dIjnSEgUD39JtWG sflt;
  struct_sbKx6IyVlWG2aZGxKuo5AB aspd;
  struct_zFuKxOdy6xWCgrwijIt3EB atc;
  struct_Byly2JHaAJfFWirb8CScwH eig;
  struct_nUdewYMN9W2ArPMLvxYBeD psc;
  struct_ZpRNaqoZiEb6XqsMOrzR0C wpnav;
  struct_t97zePNXqrora5Yj9ePAsF ca;
  struct_9VdX5G3wISWD67sX4adsqF dlc;
  struct_grmQpGhO6u50ZJYAi79l7E mla;
  struct_zTGPqEU9oPa0g1yI2jOg2F gust;
} struct_rGAsXWLaUsbxrXrQ2KHClH;
