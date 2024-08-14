#include <AP_Param/AP_Param.h>


#define DEFINED_TYPEDEF_FOR_struct_rQ7NuXJonI6pqr2AXxgEAD_

typedef struct {
  AP_Float W_v[3];
  AP_Float W_u[10];
  AP_Float gamma;
  AP_Float i_max;
} struct_rQ7NuXJonI6pqr2AXxgEAD;


#define DEFINED_TYPEDEF_FOR_struct_ZpRNaqoZiEb6XqsMOrzR0C_

typedef struct {
  AP_Float T;
  AP_Float wprad;
  AP_Float eposmax;
} struct_ZpRNaqoZiEb6XqsMOrzR0C;


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


#define DEFINED_TYPEDEF_FOR_struct_yVa4qmao2SCnxORA4Uk5SD_

typedef struct {
  AP_Float use;
  AP_Float eta_np[2];
  struct_rQ7NuXJonI6pqr2AXxgEAD ca;
} struct_yVa4qmao2SCnxORA4Uk5SD;


#define DEFINED_TYPEDEF_FOR_struct_2ZbzmTCPZOzo1j7KmkMmtF_

typedef struct {
  AP_Float u_min[10];
  AP_Float u_max[10];
  AP_Float u_d[10];
  AP_Float W_v[3];
  AP_Float W_u[10];
  AP_Float gamma;
  AP_Float W[10];
  AP_Float i_max;
} struct_2ZbzmTCPZOzo1j7KmkMmtF;


#define DEFINED_TYPEDEF_FOR_struct_pHL6i6dtl77VVkO57r2aiB_

typedef struct {
  struct_wtF2XFOtOELBemGDyRar7G k;
  struct_ZpRNaqoZiEb6XqsMOrzR0C rm;
} struct_pHL6i6dtl77VVkO57r2aiB;


#define DEFINED_TYPEDEF_FOR_struct_N3y5cN6hTzLfF948kCjh1G_

typedef struct {
  AP_Float clp;
  AP_Float b;
  AP_Float s;
  AP_Float cla_h;
  AP_Float x_h;
  AP_Float s_h;
} struct_N3y5cN6hTzLfF948kCjh1G;


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


#define DEFINED_TYPEDEF_FOR_struct_qZW4wEBszjZZRXHZEeXB7E_

typedef struct {
  AP_Float omega;
  AP_Float d;
} struct_qZW4wEBszjZZRXHZEeXB7E;


#define DEFINED_TYPEDEF_FOR_struct_gXBcg1A6eW6dnMqVoWYS8C_

typedef struct {
  AP_Float omega;
  AP_Float boost;
} struct_gXBcg1A6eW6dnMqVoWYS8C;


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


#define DEFINED_TYPEDEF_FOR_struct_MJUKigG23kBOsE76s4pAhB_

typedef struct {
  AP_Float cla[10];
  AP_Float dadf[10];
  AP_Float dfdu[10];
  AP_Float s[10];
  AP_Float rotx[10];
  AP_Float x[10];
  AP_Float y[10];
  AP_Float z[10];
  AP_Float m[10];
  AP_Float xm[10];
} struct_MJUKigG23kBOsE76s4pAhB;


#define DEFINED_TYPEDEF_FOR_struct_JfqfdhZnKx0Xf9TZzodQYG_

typedef struct {
  struct_MJUKigG23kBOsE76s4pAhB cef;
  struct_NVgzPUNgrhoXyl8OD5Cw5F ceb;
  struct_gXBcg1A6eW6dnMqVoWYS8C servo;
  struct_qZW4wEBszjZZRXHZEeXB7E sflt;
  struct_sbKx6IyVlWG2aZGxKuo5AB aspd;
  struct_zFuKxOdy6xWCgrwijIt3EB atc;
  struct_N3y5cN6hTzLfF948kCjh1G eig;
  struct_pHL6i6dtl77VVkO57r2aiB psc;
  struct_2ZbzmTCPZOzo1j7KmkMmtF ca;
  struct_yVa4qmao2SCnxORA4Uk5SD mla;
} struct_JfqfdhZnKx0Xf9TZzodQYG;
