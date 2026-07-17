//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: mvdc_path_matching_types.h
//
// Code generated for Simulink model 'mvdc_path_matching'.
//
// Model version                  : 14.0
// Simulink Coder version         : 25.1 (R2025a) 21-Nov-2024
// C/C++ source code generated on : Tue Jun 30 15:12:20 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef mvdc_path_matching_types_h_
#define mvdc_path_matching_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_TUMHealthStatus_
#define DEFINED_TYPEDEF_FOR_TUMHealthStatus_

typedef uint16_T TUMHealthStatus;

// enum TUMHealthStatus
const TUMHealthStatus ERROR = 0U;      // Default value
const TUMHealthStatus WARNING = 1U;
const TUMHealthStatus OK = 2U;

#endif

#ifndef DEFINED_TYPEDEF_FOR_TUMStateEstimationState_
#define DEFINED_TYPEDEF_FOR_TUMStateEstimationState_

typedef enum {
  SE_OFF = 0,                          // Default value
  SE_NORMAL,
  SE_BYPASS,
  SE_ODOM,
  SE_FAIL
} TUMStateEstimationState;

#endif

#ifndef DEFINED_TYPEDEF_FOR_CartPos_
#define DEFINED_TYPEDEF_FOR_CartPos_

struct CartPos
{
  real_T x_m;
  real_T y_m;
  real_T psi_rad;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_VehicleDynamicState_
#define DEFINED_TYPEDEF_FOR_VehicleDynamicState_

struct VehicleDynamicState
{
  TUMHealthStatus SE_Status;
  TUMStateEstimationState SE_State;
  CartPos Pos;
  real_T z_m;
  real_T PosAccuracy[3];
  real_T VelAccuracy[3];
  real_T dPsi_radps;
  real_T vx_mps;
  real_T vy_mps;
  real_T v_mps;
  real_T beta_rad;
  real_T ax_mps2;
  real_T ay_mps2;
  boolean_T valid_IMU_b;
  real_T psi_vel_rad;
  real_T kappa_radpm;
  real_T dBeta_radps;
  real_T ddPsi_radps2;
  real_T ax_vel_mps2;
  real_T ay_vel_mps2;
  real_T lambdaFL_perc;
  real_T lambdaFR_perc;
  real_T lambdaRL_perc;
  real_T lambdaRR_perc;
  boolean_T valid_Wheelspeeds_b;
  real_T alphaFL_rad;
  real_T alphaFR_rad;
  real_T alphaRL_rad;
  real_T alphaRR_rad;
  real_T DiffFRAlpha_rad;
  real_T DeltaWheel_rad;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_TUMStrategyState_
#define DEFINED_TYPEDEF_FOR_TUMStrategyState_

typedef uint16_T TUMStrategyState;

// enum TUMStrategyState
const TUMStrategyState S_OFF = 0U;     // Default value
const TUMStrategyState S_STARTUP = 10U;
const TUMStrategyState S_STANDSTILL = 20U;
const TUMStrategyState S_STANDSTILL_TRAJSEND = 25U;
const TUMStrategyState S_DRIVING_STARTRACE = 30U;
const TUMStrategyState S_DRIVING_RACE = 31U;
const TUMStrategyState S_DRIVING_BRAKE2STOP = 40U;
const TUMStrategyState S_DRIVING_EMERGENCY = 50U;

#endif

#ifndef DEFINED_TYPEDEF_FOR_Trajectory_
#define DEFINED_TYPEDEF_FOR_Trajectory_

struct Trajectory
{
  uint32_T LapCnt;
  uint32_T TrajCnt;
  real_T s_loc_m[50];
  real_T s_glob_m[50];
  real_T x_m[50];
  real_T y_m[50];
  real_T psi_rad[50];
  real_T kappa_radpm[50];
  real_T v_mps[50];
  real_T ax_mps2[50];
  real_T banking_rad[50];
  real_T ax_lim_mps2[50];
  real_T ay_lim_mps2[50];
  real_T tube_r_m[50];
  real_T tube_l_m[50];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_TrajectoryPlanning_
#define DEFINED_TYPEDEF_FOR_TrajectoryPlanning_

struct TrajectoryPlanning
{
  TUMStrategyState Strategy_Status;
  TUMHealthStatus Strategy_CommsStatus;
  TUMHealthStatus Trajectories_CommsStatus;
  Trajectory EmergencyTrajectory;
  Trajectory PerformanceTrajectory;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_PathPos_
#define DEFINED_TYPEDEF_FOR_PathPos_

struct PathPos
{
  real_T s_m;
  real_T d_m;
  real_T psi_rad;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_TrajectoryPoint_
#define DEFINED_TYPEDEF_FOR_TrajectoryPoint_

struct TrajectoryPoint
{
  uint32_T LapCnt;
  uint32_T TrajCnt;
  uint16_T PointIdx;
  real_T s_loc_m;
  real_T s_glob_m;
  real_T x_m;
  real_T y_m;
  real_T psi_rad;
  real_T kappa_radpm;
  real_T v_mps;
  real_T ax_mps2;
  real_T banking_rad;
  real_T ax_lim_mps2;
  real_T ay_lim_mps2;
  real_T tube_r_m;
  real_T tube_l_m;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_mvdc_path_matching_debug_
#define DEFINED_TYPEDEF_FOR_mvdc_path_matching_debug_

struct mvdc_path_matching_debug
{
  boolean_T PerformanceTrajOK_b;
  boolean_T EmergencyTrajOK_b;
  PathPos PathPosData;
  TrajectoryPoint ActualTrajPoint;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_TUMPathMatchingState_
#define DEFINED_TYPEDEF_FOR_TUMPathMatchingState_

typedef enum {
  PM_OFF = 0,                          // Default value
  PM_NOTVALID = 1,
  PM_VALID = 2,
  PM_VALID_HIGHDEVIATION = 4,
  PM_VALID_VERYHIGHDEVIATION = 5,
  PM_ENDREACHED = 6
} TUMPathMatchingState;

#endif

// Parameters (default storage)
typedef struct P_mvdc_path_matching_T_ P_mvdc_path_matching_T;

// Forward declaration for rtModel
typedef struct tag_RTM_mvdc_path_matching_T RT_MODEL_mvdc_path_matching_T;

#endif                                 // mvdc_path_matching_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
