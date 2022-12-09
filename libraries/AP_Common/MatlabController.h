//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: MatlabController.h
//
// Code generated for Simulink model 'EasyGlider_ManualMode_with_waypoints_and_sysID'.
//
// Model version                  : 1.471
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Wed Aug 24 12:11:04 2022
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#ifndef RTW_HEADER_MatlabController_h_
#define RTW_HEADER_MatlabController_h_
#include "rtwtypes.h"
#include <cmath>
#include <string.h>
#ifndef EasyGlider_ManualMode_with_waypoints_and_sysID_COMMON_INCLUDES_
# define EasyGlider_ManualMode_with_waypoints_and_sysID_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 // EasyGlider_ManualMode_with_waypoints_and_sysID_COMMON_INCLUDES_ 

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
  real32_T a_bg[3];
  real32_T airspeed;
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

#ifndef DEFINED_TYPEDEF_FOR_dtoSgl_trajectoryStructBus_
#define DEFINED_TYPEDEF_FOR_dtoSgl_trajectoryStructBus_

typedef struct {
  real32_T num_sections_max;
  real32_T num_sections_set;
  dtoSgl_sectionsBus sections[10];
  real32_T active_section;
  real32_T current_time;
  real32_T arc_length;
  real32_T distance;
  boolean_T is_repeated_course;
  real32_T polynomial_degree;
} dtoSgl_trajectoryStructBus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_dtoSgl_sections_
#define DEFINED_TYPEDEF_FOR_dtoSgl_sections_

typedef struct {
  real32_T pos_x[6];
  real32_T pos_y[6];
  real32_T pos_z[6];
  real32_T vel[6];
  real32_T t;
  real32_T arc_length;
  real32_T distance;
  real32_T polynomial_degree;
} dtoSgl_sections;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_VAW73qcsmorMGOuFyCcx3B_
#define DEFINED_TYPEDEF_FOR_struct_VAW73qcsmorMGOuFyCcx3B_

typedef struct {
  real_T pos_x[6];
  real_T pos_y[6];
  real_T pos_z[6];
  real_T vel[6];
  real_T t;
  real_T arc_length;
  real_T distance;
  real32_T polynomial_degree;
} struct_VAW73qcsmorMGOuFyCcx3B;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_etLJ3O8Id6aaO0ggNE9HuD_
#define DEFINED_TYPEDEF_FOR_struct_etLJ3O8Id6aaO0ggNE9HuD_

typedef struct {
  real32_T num_sections_max;
  real_T num_sections_set;
  struct_VAW73qcsmorMGOuFyCcx3B sections[10];
  real_T active_section;
  real_T current_time;
  real_T arc_length;
  real_T distance;
  boolean_T is_repeated_course;
  real32_T polynomial_degree;
} struct_etLJ3O8Id6aaO0ggNE9HuD;

#endif

// Custom Type definition for MATLAB Function: '<S2>/trajGetMatch'
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

// Block signals and states (default storage) for system '<Root>'
typedef struct {
  dtoSgl_trajectoryStructBus UnitDelay_DSTATE;// '<S8>/Unit Delay'
  real32_T VectorConcatenate[4];       // '<S7>/Vector Concatenate'
  real32_T DiscreteTimeIntegrator1_DSTATE[3];// '<S7>/Discrete-Time Integrator1' 
  real32_T Memory2_PreviousInput[300]; // '<S8>/Memory2'
  real32_T Memory1_PreviousInput[4];   // '<S8>/Memory1'
  real32_T Memory_PreviousInput[6];    // '<S8>/Memory'
  real32_T Delay1_DSTATE;              // '<S6>/Delay1'
  real32_T Delay_DSTATE;               // '<S6>/Delay'
  real32_T DiscreteTimeIntegratory_DSTATE;// '<S21>/Discrete-Time Integrator y'
  real32_T DiscreteTimeIntegratory_dt_DSTA;// '<S21>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegratory_dt_DS_c;// '<S20>/Discrete-Time Integrator y_dt' 
  real32_T DiscreteTimeIntegrator_DSTATE;// '<S18>/Discrete-Time Integrator'
  real32_T DiscreteTimeIntegratory_DSTAT_f;// '<S20>/Discrete-Time Integrator y' 
  boolean_T DelayInput1_DSTATE;        // '<S22>/Delay Input1'
  boolean_T UnitDelay_DSTATE_l;        // '<S6>/Unit Delay'
  boolean_T DelayInput1_DSTATE_e;      // '<S14>/Delay Input1'
  boolean_T Relay_Mode;                // '<S2>/Relay'
  boolean_T Memory_PreviousInput_i;    // '<S16>/Memory'
} DW;

// Invariant block signals (default storage)
typedef const struct tag_ConstB {
  uint16_T DataTypeConversion1[8];     // '<S1>/Data Type Conversion1'
} ConstB;

// Constant parameters (default storage)
typedef struct {
  // Pooled Parameter (Mixed Expressions)
  //  Referenced by:
  //    '<S8>/Constant1'
  //    '<S8>/Unit Delay'

  dtoSgl_trajectoryStructBus pooled3;

  // Expression: signals
  //  Referenced by: '<S6>/Constant2'

  real32_T Constant2_Value[531];

  // Computed Parameter: Constant1_Value
  //  Referenced by: '<S4>/Constant1'

  real32_T Constant1_Value[9];

  // Computed Parameter: Logic_table
  //  Referenced by: '<S16>/Logic'

  boolean_T Logic_table[16];
} ConstP;

// External inputs (root inport signals with default storage)
typedef struct {
  dtoSglFlt_cmdBus cmd;                // '<Root>/cmd'
  dtoSgl_measureBus measure;           // '<Root>/measure'
} ExtU;

// External outputs (root outports fed by signals with default storage)
typedef struct {
  real32_T channels[8];                // '<Root>/channels'
  real32_T logs[15];                   // '<Root>/logs'
  uint16_T function_channels[8];       // '<Root>/function_channels'
} ExtY;

extern const ConstB rtConstB;          // constant block i/o

// Constant parameters (default storage)
extern const ConstP rtConstP;

// Class declaration for model EasyGlider_ManualMode_with_waypoints_and_sysID
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
  void trajSectionGetPos(const real32_T traj_section_pos_x[6], const real32_T
    traj_section_pos_y[6], const real32_T traj_section_pos_z[6], real_T
    varargin_1, real32_T pos[3]);
  void polyder_a(const real32_T u[6], real32_T a_data[], int32_T a_size[2]);
  real32_T polyVal(const real32_T p_data[], const int32_T p_size[2], real_T x);
  void trajSetArcLength(dtoSgl_trajectoryStructBus *traj);
  void polyder(const real_T u_data[], const int32_T u_size[2], real_T a_data[],
               int32_T a_size[2]);
  void multAx(const real32_T j[3], const real_T k[6], const real_T Ajk[18],
              const real32_T x_data[], real32_T b_data[], real_T dim);
  void __anon_fcn_a(real32_T num_of_splines, real32_T cycle, const real32_T
                    x_data[], const int32_T *x_size, real32_T varargout_1_data[],
                    int32_T *varargout_1_size);
  real32_T norm(const real32_T x_data[], const int32_T *x_size);
  void __anon_fcn(real32_T num_of_splines, real32_T cycle, const real32_T
                  x_data[], const int32_T *x_size, real32_T varargout_1_data[],
                  int32_T *varargout_1_size);
  real32_T norm_d(const real32_T x[2]);
  void ladac_lsqr_iterate(const real32_T A_tunableEnvironment[3], real32_T
    x_data[], int32_T *x_size, real32_T w_data[], int32_T *w_size, real32_T
    u_data[], int32_T *u_size, real32_T v_data[], int32_T *v_size, real32_T
    *Anorm, real32_T *alfa, real32_T *rhobar, real32_T *phibar);
  void polyInterpolationb(const real32_T points_data[], const int32_T
    points_size[2], real32_T cycle, real32_T b_data[], int32_T *b_size, real32_T
    *num_of_splines);
  void ladac_lsqr_init(const real32_T A_tunableEnvironment[3], const real32_T
                       b_data[], const int32_T *b_size, real32_T x_data[],
                       int32_T *x_size, real32_T w_data[], int32_T *w_size,
                       real32_T u_data[], int32_T *u_size, real32_T v_data[],
                       int32_T *v_size, real32_T *alfa, real32_T *rhobar,
                       real32_T *phibar);
  real32_T norm_c(const real32_T x[3]);
  void polyder_hy(const real32_T u_data[], const int32_T u_size[2], real32_T
                  a_data[], int32_T a_size[2]);
  void trajGetMatchEnhanced(const dtoSgl_trajectoryStructBus *traj, const
    real32_T position[3], real32_T *section_idx, real32_T *error, real32_T *t);
};

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S8>/Display1' : Unused code path elimination
//  Block '<S8>/Display2' : Unused code path elimination
//  Block '<S1>/Data Type Conversion' : Eliminate redundant data type conversion
//  Block '<S2>/Data Type Conversion1' : Eliminate redundant data type conversion
//  Block '<S20>/Saturation' : Eliminated Saturate block
//  Block '<S21>/2*d//omega' : Eliminated nontunable gain of 1
//  Block '<S21>/Saturation' : Eliminated Saturate block


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
//  '<Root>' : 'EasyGlider_ManualMode_with_waypoints_and_sysID'
//  '<S1>'   : 'EasyGlider_ManualMode_with_waypoints_and_sysID/Actuator Muxer ArduPlane'
//  '<S2>'   : 'EasyGlider_ManualMode_with_waypoints_and_sysID/dummy test controller'
//  '<S3>'   : 'EasyGlider_ManualMode_with_waypoints_and_sysID/log muxer'
//  '<S4>'   : 'EasyGlider_ManualMode_with_waypoints_and_sysID/dummy test controller/Attitude and Height Estimation from Distance Measurements'
//  '<S5>'   : 'EasyGlider_ManualMode_with_waypoints_and_sysID/dummy test controller/MATLAB Function'
//  '<S6>'   : 'EasyGlider_ManualMode_with_waypoints_and_sysID/dummy test controller/Subsystem1'
//  '<S7>'   : 'EasyGlider_ManualMode_with_waypoints_and_sysID/dummy test controller/Subsystem2'
//  '<S8>'   : 'EasyGlider_ManualMode_with_waypoints_and_sysID/dummy test controller/Trajectory from Waypoints1'
//  '<S9>'   : 'EasyGlider_ManualMode_with_waypoints_and_sysID/dummy test controller/rad2deg'
//  '<S10>'  : 'EasyGlider_ManualMode_with_waypoints_and_sysID/dummy test controller/rad2deg1'
//  '<S11>'  : 'EasyGlider_ManualMode_with_waypoints_and_sysID/dummy test controller/trajGetMatch'
//  '<S12>'  : 'EasyGlider_ManualMode_with_waypoints_and_sysID/dummy test controller/Attitude and Height Estimation from Distance Measurements/MATLAB Function'
//  '<S13>'  : 'EasyGlider_ManualMode_with_waypoints_and_sysID/dummy test controller/Attitude and Height Estimation from Distance Measurements/Normalize Vector'
//  '<S14>'  : 'EasyGlider_ManualMode_with_waypoints_and_sysID/dummy test controller/Subsystem1/Detect Rise Positive'
//  '<S15>'  : 'EasyGlider_ManualMode_with_waypoints_and_sysID/dummy test controller/Subsystem1/MATLAB Function'
//  '<S16>'  : 'EasyGlider_ManualMode_with_waypoints_and_sysID/dummy test controller/Subsystem1/Subsystem'
//  '<S17>'  : 'EasyGlider_ManualMode_with_waypoints_and_sysID/dummy test controller/Subsystem1/Detect Rise Positive/Positive'
//  '<S18>'  : 'EasyGlider_ManualMode_with_waypoints_and_sysID/dummy test controller/Subsystem2/Subsystem'
//  '<S19>'  : 'EasyGlider_ManualMode_with_waypoints_and_sysID/dummy test controller/Subsystem2/deg2rad'
//  '<S20>'  : 'EasyGlider_ManualMode_with_waypoints_and_sysID/dummy test controller/Subsystem2/Subsystem/PT2 discrete with saturation'
//  '<S21>'  : 'EasyGlider_ManualMode_with_waypoints_and_sysID/dummy test controller/Subsystem2/Subsystem/PT2 discrete with saturation1'
//  '<S22>'  : 'EasyGlider_ManualMode_with_waypoints_and_sysID/dummy test controller/Trajectory from Waypoints1/Detect Rise Positive'
//  '<S23>'  : 'EasyGlider_ManualMode_with_waypoints_and_sysID/dummy test controller/Trajectory from Waypoints1/pad_waypoints'
//  '<S24>'  : 'EasyGlider_ManualMode_with_waypoints_and_sysID/dummy test controller/Trajectory from Waypoints1/trajFromWaypoints'
//  '<S25>'  : 'EasyGlider_ManualMode_with_waypoints_and_sysID/dummy test controller/Trajectory from Waypoints1/Detect Rise Positive/Positive'

#endif                                 // RTW_HEADER_MatlabController_h_

//
// File trailer for generated code.
//
// [EOF]
//
