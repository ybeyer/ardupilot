//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: MatlabController.h
//
// Code generated for Simulink model 'ArduCopter_MinnieDragonflyController'.
//
// Model version                  : 1.447
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Thu May 19 18:03:08 2022
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM 7
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#ifndef RTW_HEADER_MatlabController_h_
#define RTW_HEADER_MatlabController_h_
#include "rtwtypes.h"
#include <cmath>
#include <stdlib.h>
#include <string.h>
#ifndef ArduCopter_MinnieDragonflyController_COMMON_INCLUDES_
# define ArduCopter_MinnieDragonflyController_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 // ArduCopter_MinnieDragonflyController_COMMON_INCLUDES_ 

// Macros for accessing real-time model data structure
#ifndef DEFINED_TYPEDEF_FOR_dtoSglFlt_cmdBus_
#define DEFINED_TYPEDEF_FOR_dtoSglFlt_cmdBus_

typedef struct {
  real32_T roll;
  real32_T pitch;
  real32_T yaw;
  real32_T thr;
  real32_T s_Kg_init[3];
  real32_T yaw_init;
  uint16_T mission_change;
  real32_T waypoints[40];
  uint16_T num_waypoints;
  real32_T RC_pwm[16];
} dtoSglFlt_cmdBus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_dtoSgl_measureBus_
#define DEFINED_TYPEDEF_FOR_dtoSgl_measureBus_

typedef struct {
  real32_T omega_Kb[3];
  real32_T EulerAngles[3];
  real32_T q_bg[4];
  real32_T a_Kg[3];
  real32_T V_Kg[3];
  real32_T s_Kg[3];
  real32_T lla[3];
  real32_T rangefinder[6];
} dtoSgl_measureBus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_dtoSgl_sectionsBus_
#define DEFINED_TYPEDEF_FOR_dtoSgl_sectionsBus_

typedef struct {
  real32_T pos_x[6];
  real32_T pos_y[6];
  real32_T pos_z[6];
  real32_T vel[6];
  real32_T t;
  real32_T arc_length;
  real32_T distance;
  real32_T polynomial_degree;
} dtoSgl_sectionsBus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_dtoSgl_trajectoryBus_
#define DEFINED_TYPEDEF_FOR_dtoSgl_trajectoryBus_

typedef struct {
  real32_T num_sections_max;
  real32_T num_sections_set;
  dtoSgl_sectionsBus sections[4];
  real32_T active_section;
  real32_T current_time;
  real32_T arc_length;
  real32_T distance;
  boolean_T is_repeated_course;
  real32_T polynomial_degree;
} dtoSgl_trajectoryBus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_wz3NwgcaztLJWP7I924IQF_
#define DEFINED_TYPEDEF_FOR_struct_wz3NwgcaztLJWP7I924IQF_

typedef struct {
  real_T pos_x[6];
  real_T pos_y[6];
  real_T pos_z[6];
  real_T vel[6];
  real_T t;
  real_T arc_length;
  real_T distance;
  real_T polynomial_degree;
} struct_wz3NwgcaztLJWP7I924IQF;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_MDdRJn1JNxKkhM7NkKZACH_
#define DEFINED_TYPEDEF_FOR_struct_MDdRJn1JNxKkhM7NkKZACH_

typedef struct {
  real_T num_sections_max;
  real_T num_sections_set;
  struct_wz3NwgcaztLJWP7I924IQF sections[4];
  real_T active_section;
  real_T current_time;
  real_T arc_length;
  real_T distance;
  boolean_T is_repeated_course;
  real_T polynomial_degree;
} struct_MDdRJn1JNxKkhM7NkKZACH;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_uuVM5iCJRh0lWMg9ww4boE_
#define DEFINED_TYPEDEF_FOR_struct_uuVM5iCJRh0lWMg9ww4boE_

typedef struct {
  real_T u_min[4];
  real_T u_max[4];
  real_T u_d[4];
  real_T W_v[16];
  real_T W_u[16];
  real_T gamma;
  real_T W[4];
  real_T i_max;
} struct_uuVM5iCJRh0lWMg9ww4boE;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_bp3dTcfUy8YiLQ2C4HxdqF_
#define DEFINED_TYPEDEF_FOR_struct_bp3dTcfUy8YiLQ2C4HxdqF_

typedef struct {
  real_T veltc;
  real_T velxymax;
  real_T accxymax;
  real_T velumax;
  real_T veldmax;
  real_T accumax;
  real_T accdmax;
} struct_bp3dTcfUy8YiLQ2C4HxdqF;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_t1muYkFBpCsvhMydPIEJxF_
#define DEFINED_TYPEDEF_FOR_struct_t1muYkFBpCsvhMydPIEJxF_

typedef struct {
  real_T pos;
  real_T vel;
  real_T acc;
} struct_t1muYkFBpCsvhMydPIEJxF;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_9CO4a2DCsO6nqJ3du2LIdG_
#define DEFINED_TYPEDEF_FOR_struct_9CO4a2DCsO6nqJ3du2LIdG_

typedef struct {
  struct_bp3dTcfUy8YiLQ2C4HxdqF rm;
  struct_t1muYkFBpCsvhMydPIEJxF k;
} struct_9CO4a2DCsO6nqJ3du2LIdG;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_7u3c6UdWNLriQkni5vhKPF_
#define DEFINED_TYPEDEF_FOR_struct_7u3c6UdWNLriQkni5vhKPF_

typedef struct {
  real32_T s_g_ref[3];
  real32_T s_g_dt_ref[3];
  real32_T s_g_dt2_ref[3];
} struct_7u3c6UdWNLriQkni5vhKPF;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_rmfrApUDXU4QnHqd0c82aG_
#define DEFINED_TYPEDEF_FOR_struct_rmfrApUDXU4QnHqd0c82aG_

typedef struct {
  real32_T section_idx;
  real32_T t;
  real32_T distance;
} struct_rmfrApUDXU4QnHqd0c82aG;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_W4TkjjgcWv5Hvd5xW7OTfE_
#define DEFINED_TYPEDEF_FOR_struct_W4TkjjgcWv5Hvd5xW7OTfE_

typedef struct {
  real32_T pos_match_g[3];
  real32_T e_dist;
  real32_T e_pos_t[3];
  real32_T M_tg[9];
  real32_T section_idx;
  real32_T t;
} struct_W4TkjjgcWv5Hvd5xW7OTfE;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_4iaXGPlTyjiZKe9TW5IsIB_
#define DEFINED_TYPEDEF_FOR_struct_4iaXGPlTyjiZKe9TW5IsIB_

typedef struct {
  real32_T e_V_K[3];
  real32_T V_K_d;
  real32_T V_K;
} struct_4iaXGPlTyjiZKe9TW5IsIB;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_VcqRHRPJNpUuFzH1145utB_
#define DEFINED_TYPEDEF_FOR_struct_VcqRHRPJNpUuFzH1145utB_

typedef struct {
  dtoSgl_trajectoryBus traj;
  struct_7u3c6UdWNLriQkni5vhKPF reference;
  struct_rmfrApUDXU4QnHqd0c82aG look_ahead;
  struct_W4TkjjgcWv5Hvd5xW7OTfE matching;
  struct_4iaXGPlTyjiZKe9TW5IsIB velocity;
} struct_VcqRHRPJNpUuFzH1145utB;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_6deKI7nPlDCgAgMwbtYtGD_
#define DEFINED_TYPEDEF_FOR_struct_6deKI7nPlDCgAgMwbtYtGD_

typedef struct {
  real_T wpmax;
  boolean_T cycle;
  real_T degree;
} struct_6deKI7nPlDCgAgMwbtYtGD;

#endif

// Custom Type definition for MATLAB Function: '<S47>/DCM to quaternions'
#ifndef struct_tag_skA4KFEZ4HPkJJBOYCrevdH
#define struct_tag_skA4KFEZ4HPkJJBOYCrevdH

struct tag_skA4KFEZ4HPkJJBOYCrevdH
{
  uint32_T SafeEq;
  uint32_T Absolute;
  uint32_T NaNBias;
  uint32_T NaNWithFinite;
  uint32_T FiniteWithNaN;
  uint32_T NaNWithNaN;
};

#endif                                 //struct_tag_skA4KFEZ4HPkJJBOYCrevdH

#ifndef typedef_skA4KFEZ4HPkJJBOYCrevdH
#define typedef_skA4KFEZ4HPkJJBOYCrevdH

typedef struct tag_skA4KFEZ4HPkJJBOYCrevdH skA4KFEZ4HPkJJBOYCrevdH;

#endif                                 //typedef_skA4KFEZ4HPkJJBOYCrevdH

#ifndef struct_tag_sJCxfmxS8gBOONUZjbjUd9E
#define struct_tag_sJCxfmxS8gBOONUZjbjUd9E

struct tag_sJCxfmxS8gBOONUZjbjUd9E
{
  boolean_T CaseSensitivity;
  boolean_T StructExpand;
  char_T PartialMatching[6];
  boolean_T IgnoreNulls;
};

#endif                                 //struct_tag_sJCxfmxS8gBOONUZjbjUd9E

#ifndef typedef_sJCxfmxS8gBOONUZjbjUd9E
#define typedef_sJCxfmxS8gBOONUZjbjUd9E

typedef struct tag_sJCxfmxS8gBOONUZjbjUd9E sJCxfmxS8gBOONUZjbjUd9E;

#endif                                 //typedef_sJCxfmxS8gBOONUZjbjUd9E

#ifndef struct_emxArray_real32_T
#define struct_emxArray_real32_T

struct emxArray_real32_T
{
  real32_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray_real32_T

#ifndef typedef_emxArray_real32_T
#define typedef_emxArray_real32_T

typedef struct emxArray_real32_T emxArray_real32_T;

#endif                                 //typedef_emxArray_real32_T

#ifndef struct_emxArray_real32_T_1x5
#define struct_emxArray_real32_T_1x5

struct emxArray_real32_T_1x5
{
  real32_T data[5];
  int32_T size[2];
};

#endif                                 //struct_emxArray_real32_T_1x5

#ifndef typedef_emxArray_real32_T_1x5
#define typedef_emxArray_real32_T_1x5

typedef struct emxArray_real32_T_1x5 emxArray_real32_T_1x5;

#endif                                 //typedef_emxArray_real32_T_1x5

#ifndef struct_emxArray_real_T
#define struct_emxArray_real_T

struct emxArray_real_T
{
  real_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray_real_T

#ifndef typedef_emxArray_real_T
#define typedef_emxArray_real_T

typedef struct emxArray_real_T emxArray_real_T;

#endif                                 //typedef_emxArray_real_T

#ifndef struct_emxArray_int32_T
#define struct_emxArray_int32_T

struct emxArray_int32_T
{
  int32_T *data;
  int32_T *size;
  int32_T allocatedSize;
  int32_T numDimensions;
  boolean_T canFreeData;
};

#endif                                 //struct_emxArray_int32_T

#ifndef typedef_emxArray_int32_T
#define typedef_emxArray_int32_T

typedef struct emxArray_int32_T emxArray_int32_T;

#endif                                 //typedef_emxArray_int32_T

#ifndef struct_sKQLR94MqGzlZ6BjlviOSrB_tag
#define struct_sKQLR94MqGzlZ6BjlviOSrB_tag

struct sKQLR94MqGzlZ6BjlviOSrB_tag
{
  emxArray_real32_T_1x5 f1;
};

#endif                                 //struct_sKQLR94MqGzlZ6BjlviOSrB_tag

#ifndef typedef_b_cell_wrap_1
#define typedef_b_cell_wrap_1

typedef struct sKQLR94MqGzlZ6BjlviOSrB_tag b_cell_wrap_1;

#endif                                 //typedef_b_cell_wrap_1

// Custom Type definition for MATLAB Function: '<S23>/pad_waypoints'
#ifndef struct_tag_sknvGbOHSCWBvSX7bE0NSZG
#define struct_tag_sknvGbOHSCWBvSX7bE0NSZG

struct tag_sknvGbOHSCWBvSX7bE0NSZG
{
  real_T pos_x[6];
  real_T pos_y[6];
  real_T pos_z[6];
  real_T vel[6];
  real_T t;
  real_T arc_length;
  real_T distance;
  real_T polynomial_degree;
};

#endif                                 //struct_tag_sknvGbOHSCWBvSX7bE0NSZG

#ifndef typedef_sknvGbOHSCWBvSX7bE0NSZG
#define typedef_sknvGbOHSCWBvSX7bE0NSZG

typedef struct tag_sknvGbOHSCWBvSX7bE0NSZG sknvGbOHSCWBvSX7bE0NSZG;

#endif                                 //typedef_sknvGbOHSCWBvSX7bE0NSZG

#ifndef struct_tag_sjz2zDBbUld3vkiprQKksuC
#define struct_tag_sjz2zDBbUld3vkiprQKksuC

struct tag_sjz2zDBbUld3vkiprQKksuC
{
  real_T num_sections_max;
  real_T num_sections_set;
  sknvGbOHSCWBvSX7bE0NSZG sections[4];
  real_T active_section;
  real_T current_time;
  real_T arc_length;
  real_T distance;
  boolean_T is_repeated_course;
  real_T polynomial_degree;
};

#endif                                 //struct_tag_sjz2zDBbUld3vkiprQKksuC

#ifndef typedef_sjz2zDBbUld3vkiprQKksuC
#define typedef_sjz2zDBbUld3vkiprQKksuC

typedef struct tag_sjz2zDBbUld3vkiprQKksuC sjz2zDBbUld3vkiprQKksuC;

#endif                                 //typedef_sjz2zDBbUld3vkiprQKksuC

// Block signals and states (default storage) for system '<Root>'
typedef struct {
  dtoSgl_trajectoryBus traj;           // '<S29>/trajFromWaypoints'
  real32_T s_g_ref[3];                 // '<S27>/position controller reference from flight path' 
  real32_T s_g_dt_ref[3];              // '<S27>/position controller reference from flight path' 
  real32_T s_g_dt2_ref[3];             // '<S27>/position controller reference from flight path' 
  real32_T pos_match_g[3];             // '<S26>/flight path matching'
  real32_T UD_DSTATE[3];               // '<S8>/UD'
  real32_T DiscreteTimeIntegrator_DSTATE[4];// '<S44>/Discrete-Time Integrator'
  real32_T UD_DSTATE_o[3];             // '<S7>/UD'
  real32_T DiscreteTimeIntegrator_DSTATE_k[3];// '<S73>/Discrete-Time Integrator' 
  real32_T DiscreteTimeIntegratory_DSTATE[9];// '<S76>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_DSTAT_a[3];// '<S61>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DSTA[3];// '<S61>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_DSTAT_f[3];// '<S68>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_DSTAT_k[3];// '<S69>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_DSTAT_p[3];// '<S70>/Discrete-Time Integrator y' 
  real32_T UnitDelay1_DSTATE[4];       // '<S37>/Unit Delay1'
  real32_T DiscreteTimeIntegratory_DSTAT_e[4];// '<S39>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_DS_j[9];// '<S76>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegrator_DSTAT_n5[3];// '<S13>/Discrete-Time Integrator' 
  real32_T DiscreteTimeIntegratory_dt_DS_m[4];// '<S39>/Discrete-Time Integrator y_dt' 
  real32_T Delay_DSTATE[4];            // '<S10>/Delay'
  real32_T DiscreteTimeIntegratory_dt_DS_l[3];// '<S70>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_DS_k[3];// '<S69>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_D_kt[3];// '<S68>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegrator_DSTATE_n;// '<S67>/Discrete-Time Integrator'
  real32_T DiscreteTimeIntegrator2_DSTATE;// '<S51>/Discrete-Time Integrator2'
  real32_T DiscreteTimeIntegratory_DSTAT_h;// '<S15>/Discrete-Time Integrator y' 
  real32_T DiscreteTimeIntegratory_dt_D_kp;// '<S15>/Discrete-Time Integrator y_dt' 
  int8_T DiscreteTimeIntegrator_PrevRese;// '<S73>/Discrete-Time Integrator'
  int8_T DiscreteTimeIntegrator_PrevRe_g;// '<S13>/Discrete-Time Integrator'
  uint8_T DiscreteTimeIntegrator_IC_LOADI;// '<S73>/Discrete-Time Integrator'
  uint8_T DiscreteTimeIntegratory_IC_LOAD;// '<S61>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_LO_f;// '<S68>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_LO_b;// '<S69>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegratory_IC_LO_e;// '<S70>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegrator2_IC_LOAD;// '<S51>/Discrete-Time Integrator2'
  uint8_T DiscreteTimeIntegratory_IC_LO_g;// '<S15>/Discrete-Time Integrator y'
  uint8_T DiscreteTimeIntegrator_IC_LOA_d;// '<S13>/Discrete-Time Integrator'
  boolean_T Delay_DSTATE_d;            // '<S23>/Delay'
  boolean_T DelayInput1_DSTATE;        // '<S28>/Delay Input1'
  boolean_T Memory_PreviousInput;      // '<S16>/Memory'
} DW;

// Constant parameters (default storage)
typedef struct {
  // Computed Parameter: Constant2_Value
  //  Referenced by: '<S23>/Constant2'

  dtoSgl_trajectoryBus Constant2_Value;

  // Expression: ca
  //  Referenced by: '<S41>/MATLAB Function2'

  struct_uuVM5iCJRh0lWMg9ww4boE MATLABFunction2_ca;

  // Computed Parameter: ny_du_red_trim_Value
  //  Referenced by: '<S40>/ny_du_red_trim'

  real32_T ny_du_red_trim_Value[16];

  // Computed Parameter: ny_du_dt_Value
  //  Referenced by: '<S40>/ny_du_dt'

  real32_T ny_du_dt_Value[16];

  // Computed Parameter: uDLookupTable1_tableData
  //  Referenced by: '<S37>/1-D Lookup Table1'

  real32_T uDLookupTable1_tableData[4];

  // Computed Parameter: uDLookupTable1_bp01Data
  //  Referenced by: '<S37>/1-D Lookup Table1'

  real32_T uDLookupTable1_bp01Data[4];

  // Computed Parameter: uDLookupTable_tableData
  //  Referenced by: '<S13>/1-D Lookup Table'

  real32_T uDLookupTable_tableData[3];

  // Computed Parameter: uDLookupTable_bp01Data
  //  Referenced by: '<S13>/1-D Lookup Table'

  real32_T uDLookupTable_bp01Data[3];
} ConstP;

// External inputs (root inport signals with default storage)
typedef struct {
  dtoSglFlt_cmdBus cmd;                // '<Root>/cmd'
  dtoSgl_measureBus measure;           // '<Root>/measure'
} ExtU;

// External outputs (root outports fed by signals with default storage)
typedef struct {
  real32_T u[8];                       // '<Root>/u'
  real32_T logs[15];                   // '<Root>/logs'
} ExtY;

// Constant parameters (default storage)
extern const ConstP rtConstP;

// Class declaration for model ArduCopter_MinnieDragonflyController
class MatlabControllerClass {
  // public data and function members
 public:
  // External inputs
  ExtU rtU;

  // External outputs
  ExtY rtY;

  // model initialize function
  void initialize();

  // model step function
  void step();

  // Constructor
  MatlabControllerClass();

  // Destructor
  ~MatlabControllerClass();

  // private data and function members
 private:
  // Block signals and states
  DW rtDW;

  // private member function(s) for subsystem '<Root>'
  void emxInit_real32_T(emxArray_real32_T **pEmxArray, int32_T numDimensions);
  void emxInit_real_T(emxArray_real_T **pEmxArray, int32_T numDimensions);
  void emxEnsureCapacity_real_T(emxArray_real_T *emxArray, int32_T oldNumel);
  void emxFree_real_T(emxArray_real_T **pEmxArray);
  void emxEnsureCapacity_real32_T(emxArray_real32_T *emxArray, int32_T oldNumel);
  void emxInit_int32_T(emxArray_int32_T **pEmxArray, int32_T numDimensions);
  void emxEnsureCapacity_int32_T(emxArray_int32_T *emxArray, int32_T oldNumel);
  void emxFree_real32_T(emxArray_real32_T **pEmxArray);
  real32_T xnrm2(int32_T n, const emxArray_real32_T *x, int32_T ix0);
  void xscal(int32_T n, real32_T a, emxArray_real32_T *x, int32_T ix0);
  void xgeqp3(emxArray_real32_T *A, emxArray_real32_T *tau, emxArray_int32_T
              *jpvt);
  void emxFree_int32_T(emxArray_int32_T **pEmxArray);
  void lusolve(const emxArray_real32_T *A, emxArray_real32_T *B_1);
  void mldivide(const emxArray_real32_T *A, const emxArray_real32_T *B_0,
                emxArray_real32_T *Y);
  void polyder(const emxArray_real_T *u, emxArray_real_T *a);
  void polyInterpolation(const real32_T points[4], real32_T degree, boolean_T
    cycle, emxArray_real32_T *coeffs, real_T *num_of_splines);
  void polyder_p(const real32_T u[6], real32_T a_data[], int32_T a_size[2]);
  real32_T polyVal(const real32_T p_data[], const int32_T p_size[2], real_T x);
  real32_T integralSimpson(const b_cell_wrap_1 func_tunableEnvironment[3],
    real_T A, real_T B_2, real_T steps);
  void trajSectionGetArcLength(const real32_T traj_section_pos_x[6], const
    real32_T traj_section_pos_y[6], const real32_T traj_section_pos_z[6],
    real32_T varargin_1, real32_T *arc_length, real32_T *arc_length_dt);
  void trajSectionGetArcLength_l(const real32_T traj_section_pos_x[6], const
    real32_T traj_section_pos_y[6], const real32_T traj_section_pos_z[6], real_T
    varargin_1, real32_T *arc_length, real32_T *arc_length_dt);
  real32_T norm(const real32_T x[3]);
  void polyder_fj(const real32_T u_data[], const int32_T u_size[2], real32_T
                  a_data[], int32_T a_size[2]);
  void trajSectionGetFrenetSerretWit_k(const real32_T traj_section_pos_x[6],
    const real32_T traj_section_pos_y[6], const real32_T traj_section_pos_z[6],
    real32_T vel, real_T varargin_1, real32_T T[3], real32_T B_4[3], real32_T N
    [3], real32_T *kappa, real32_T *tau);
  void LSQFromQR(const real32_T A_data[], const int32_T A_size[2], const
                 real32_T tau_data[], const int32_T jpvt_data[], real32_T B_8[8],
                 int32_T rankA, real32_T Y_data[], int32_T *Y_size);
  real32_T xnrm2_dg(int32_T n, const real32_T x_data[], int32_T ix0);
  void xzlarf(int32_T m, int32_T n, int32_T iv0, real32_T tau, real32_T C_data[],
              int32_T ic0, real32_T work_data[]);
  void qrsolve(const real32_T A_data[], const int32_T A_size[2], const real32_T
               B_6[8], real32_T Y_data[], int32_T *Y_size);
  void mldivide_b(const real32_T A_data[], const int32_T A_size[2], const
                  real32_T B_5[8], real32_T Y_data[], int32_T *Y_size);
  boolean_T any(const boolean_T x_data[], const int32_T *x_size);
  real_T wls_alloc(const real32_T B_9[16], const real32_T v[4], const real32_T
                   umin[4], const real32_T umax[4], const real32_T Wv[16], const
                   real_T Wu[16], const real32_T ud[4], real32_T gam, real32_T
                   u[4], real_T W[4], real_T imax);
  real32_T xnrm2_d(int32_T n, const real32_T x[81], int32_T ix0);
  void xgehrd(real32_T a[81]);
  real32_T xnrm2_dl(int32_T n, const real32_T x[3]);
  real32_T xzlarfg(int32_T n, real32_T *alpha1, real32_T x[3]);
  void xdlanv2(real32_T *a, real32_T *b, real32_T *c, real32_T *d, real32_T
               *rt1r, real32_T *rt1i, real32_T *rt2r, real32_T *rt2i, real32_T
               *cs, real32_T *sn);
  int32_T eml_dlahqr(real32_T h[81]);
  void trajSectionGetPos(const real32_T traj_section_pos_x[6], const real32_T
    traj_section_pos_y[6], const real32_T traj_section_pos_z[6], real_T
    varargin_1, real32_T pos[3]);
};

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<Root>/Constant1' : Unused code path elimination
//  Block '<S3>/Constant' : Unused code path elimination
//  Block '<S7>/Data Type Duplicate' : Unused code path elimination
//  Block '<S8>/Data Type Duplicate' : Unused code path elimination
//  Block '<S24>/Add' : Unused code path elimination
//  Block '<S14>/Gain' : Unused code path elimination
//  Block '<S14>/Gain1' : Unused code path elimination
//  Block '<S77>/Data Type Duplicate' : Unused code path elimination
//  Block '<S77>/Data Type Propagation' : Unused code path elimination
//  Block '<S78>/Data Type Duplicate' : Unused code path elimination
//  Block '<S78>/Data Type Propagation' : Unused code path elimination
//  Block '<S44>/Saturation' : Eliminated Saturate block
//  Block '<S39>/Saturation' : Eliminated Saturate block
//  Block '<S12>/Reshape' : Reshape block reduction
//  Block '<S12>/Reshape1' : Reshape block reduction
//  Block '<S12>/Reshape2' : Reshape block reduction
//  Block '<S51>/Gain' : Eliminated nontunable gain of 1
//  Block '<S51>/Gain3' : Eliminated nontunable gain of 1
//  Block '<S67>/Saturation' : Eliminated Saturate block
//  Block '<S61>/Saturation' : Eliminated Saturate block
//  Block '<S51>/Reshape' : Reshape block reduction
//  Block '<S68>/Saturation' : Eliminated Saturate block
//  Block '<S69>/Saturation' : Eliminated Saturate block
//  Block '<S70>/Saturation' : Eliminated Saturate block
//  Block '<S13>/Gain' : Eliminated nontunable gain of 1
//  Block '<S13>/Gain1' : Eliminated nontunable gain of 1
//  Block '<S76>/Saturation' : Eliminated Saturate block
//  Block '<S15>/Saturation' : Eliminated Saturate block


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'ArduCopter_MinnieDragonflyController'
//  '<S1>'   : 'ArduCopter_MinnieDragonflyController/Actuator muxer'
//  '<S2>'   : 'ArduCopter_MinnieDragonflyController/Compare To Constant'
//  '<S3>'   : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly'
//  '<S4>'   : 'ArduCopter_MinnieDragonflyController/Quaternions to Rotation Matrix'
//  '<S5>'   : 'ArduCopter_MinnieDragonflyController/log muxer'
//  '<S6>'   : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Delta attitude to stick commands'
//  '<S7>'   : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Discrete Derivative'
//  '<S8>'   : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Discrete Derivative3'
//  '<S9>'   : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Flight path guidance with position controller reference'
//  '<S10>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/INDI + CA'
//  '<S11>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/MATLAB Function'
//  '<S12>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Multicopter (Reduced) Attitude INDI Controller'
//  '<S13>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/NDI position controller for copters reference model'
//  '<S14>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/NDI position controller for copters with reference input'
//  '<S15>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/PT2 discrete with saturation Reduced attitude reference model'
//  '<S16>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/detect mode switch'
//  '<S17>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/measured yaw'
//  '<S18>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/switch position reference model'
//  '<S19>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Delta attitude to stick commands/INDI Copter Acc 2 Lean Vector'
//  '<S20>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Delta attitude to stick commands/MATLAB Function'
//  '<S21>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Delta attitude to stick commands/MATLAB Function4'
//  '<S22>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Delta attitude to stick commands/no thrust if flipped'
//  '<S23>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Flight path guidance with position controller reference/Trajectory from Waypoints'
//  '<S24>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Flight path guidance with position controller reference/flight-path velocity'
//  '<S25>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Flight path guidance with position controller reference/look ahead'
//  '<S26>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Flight path guidance with position controller reference/matching'
//  '<S27>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Flight path guidance with position controller reference/reference'
//  '<S28>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Flight path guidance with position controller reference/Trajectory from Waypoints/Detect Rise Positive'
//  '<S29>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Flight path guidance with position controller reference/Trajectory from Waypoints/Subsystem2'
//  '<S30>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Flight path guidance with position controller reference/Trajectory from Waypoints/pad_waypoints'
//  '<S31>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Flight path guidance with position controller reference/Trajectory from Waypoints/Detect Rise Positive/Positive'
//  '<S32>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Flight path guidance with position controller reference/Trajectory from Waypoints/Subsystem2/trajFromWaypoints'
//  '<S33>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Flight path guidance with position controller reference/flight-path velocity/absolute flight-path velocity'
//  '<S34>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Flight path guidance with position controller reference/look ahead/flight path look ahead'
//  '<S35>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Flight path guidance with position controller reference/matching/flight path matching'
//  '<S36>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Flight path guidance with position controller reference/reference/position controller reference from flight path'
//  '<S37>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/INDI + CA/INDI high level wls control allocation'
//  '<S38>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/INDI + CA/PT1 discrete reference model'
//  '<S39>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/INDI + CA/PT2 discrete with saturation1'
//  '<S40>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/INDI + CA/control effectiveness'
//  '<S41>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/INDI + CA/INDI high level wls control allocation/INDI control allocation'
//  '<S42>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/INDI + CA/INDI high level wls control allocation/MATLAB Function'
//  '<S43>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/INDI + CA/INDI high level wls control allocation/INDI control allocation/MATLAB Function2'
//  '<S44>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/INDI + CA/PT1 discrete reference model/PT1 discrete with saturations'
//  '<S45>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/INDI + CA/control effectiveness/MATLAB Function'
//  '<S46>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Multicopter (Reduced) Attitude INDI Controller/error computation'
//  '<S47>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Multicopter (Reduced) Attitude INDI Controller/measure'
//  '<S48>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Multicopter (Reduced) Attitude INDI Controller/ny control'
//  '<S49>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Multicopter (Reduced) Attitude INDI Controller/ny from reference'
//  '<S50>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Multicopter (Reduced) Attitude INDI Controller/ny measured'
//  '<S51>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Multicopter (Reduced) Attitude INDI Controller/reference model'
//  '<S52>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Multicopter (Reduced) Attitude INDI Controller/error computation/angle error'
//  '<S53>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Multicopter (Reduced) Attitude INDI Controller/error computation/wrap angle'
//  '<S54>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Multicopter (Reduced) Attitude INDI Controller/error computation/wrap angle1'
//  '<S55>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Multicopter (Reduced) Attitude INDI Controller/measure/DCM to quaternions'
//  '<S56>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Multicopter (Reduced) Attitude INDI Controller/measure/Quaternion Reduced'
//  '<S57>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Multicopter (Reduced) Attitude INDI Controller/reference model/Lean Vector Derivative Trafo'
//  '<S58>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Multicopter (Reduced) Attitude INDI Controller/reference model/Lean Vector Derivative Trafo Delay'
//  '<S59>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Multicopter (Reduced) Attitude INDI Controller/reference model/MATLAB Function'
//  '<S60>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Multicopter (Reduced) Attitude INDI Controller/reference model/PT1 discrete reference model2'
//  '<S61>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Multicopter (Reduced) Attitude INDI Controller/reference model/PT2 discrete with saturation'
//  '<S62>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Multicopter (Reduced) Attitude INDI Controller/reference model/Subsystem'
//  '<S63>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Multicopter (Reduced) Attitude INDI Controller/reference model/Subsystem1'
//  '<S64>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Multicopter (Reduced) Attitude INDI Controller/reference model/Subsystem2'
//  '<S65>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Multicopter (Reduced) Attitude INDI Controller/reference model/lean angles 2 lean vector'
//  '<S66>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Multicopter (Reduced) Attitude INDI Controller/reference model/n ref norm'
//  '<S67>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Multicopter (Reduced) Attitude INDI Controller/reference model/PT1 discrete reference model2/PT1 discrete with saturations'
//  '<S68>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Multicopter (Reduced) Attitude INDI Controller/reference model/Subsystem/PT2 discrete with saturation'
//  '<S69>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Multicopter (Reduced) Attitude INDI Controller/reference model/Subsystem1/PT2 discrete with saturation'
//  '<S70>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/Multicopter (Reduced) Attitude INDI Controller/reference model/Subsystem2/PT2 discrete with saturation'
//  '<S71>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/NDI position controller for copters reference model/MATLAB Function1'
//  '<S72>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/NDI position controller for copters reference model/PT1 discrete reference model'
//  '<S73>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/NDI position controller for copters reference model/PT1 discrete reference model/PT1 discrete with saturations'
//  '<S74>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/NDI position controller for copters with reference input/measures'
//  '<S75>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/NDI position controller for copters with reference input/position controller'
//  '<S76>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/NDI position controller for copters with reference input/position controller/PT2 discrete with saturation'
//  '<S77>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/NDI position controller for copters with reference input/position controller/Saturation Dynamic'
//  '<S78>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/NDI position controller for copters with reference input/position controller/Saturation Dynamic1'
//  '<S79>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/NDI position controller for copters with reference input/position controller/acccntrlmax'
//  '<S80>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/NDI position controller for copters with reference input/position controller/eposmax'
//  '<S81>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/measured yaw/DCM to quaternions'
//  '<S82>'  : 'ArduCopter_MinnieDragonflyController/Copter Autopilot Dragonfly/measured yaw/Quaternion Reduced'

#endif                                 // RTW_HEADER_MatlabController_h_

//
// File trailer for generated code.
//
// [EOF]
//
