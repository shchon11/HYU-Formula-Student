//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: mvdc_mpc_types.h
//
// Code generated for Simulink model 'mvdc_mpc'.
//
// Model version                  : 1.3229
// Simulink Coder version         : 25.1 (R2025a) 21-Nov-2024
// C/C++ source code generated on : Tue Jun 30 01:49:36 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef mvdc_mpc_types_h_
#define mvdc_mpc_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_TUMHealthStatus_
#define DEFINED_TYPEDEF_FOR_TUMHealthStatus_

typedef uint16_T TUMHealthStatus;

// enum TUMHealthStatus
const TUMHealthStatus ERROR = 0U;      // Default value
const TUMHealthStatus WARNING = 1U;
const TUMHealthStatus OK = 2U;

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

#ifndef DEFINED_TYPEDEF_FOR_VehicleControl_
#define DEFINED_TYPEDEF_FOR_VehicleControl_

struct VehicleControl
{
  real_T RequestSteeringAngle_rad;
  real_T RequestLongForce_N;
  boolean_T ParkBrakeActive;
  real_T RequestThrottle_perc;
  real_T RequestBrake_bar;
  real_T RequestGear;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_ActuatorLimitations_
#define DEFINED_TYPEDEF_FOR_ActuatorLimitations_

struct ActuatorLimitations
{
  real_T SteeringAngleMax_rad;
  real_T SteeringAngleMin_rad;
  real_T DriveForceMax_N;
  real_T DriveForceMin_N;
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

#ifndef DEFINED_TYPEDEF_FOR_mvdc_tube_mpc_debug_
#define DEFINED_TYPEDEF_FOR_mvdc_tube_mpc_debug_

struct mvdc_tube_mpc_debug
{
  uint32_T tmpc_cnt;
  real_T dot_d_analytical_mps;
  real_T dot_d_numerical_mps;
  real_T be_u_abs[3];
  real_T be_l_abs[3];
  real_T error_state[3];
  real_T s_current_m;
  real_T solver_state;
  real_T solver_iteration;
  real_T solver_solveTime_s;
  real_T solver_updateTime_s;
  real_T solver_runTime_s;
  real_T solver_sfunTime_s;
  real_T solver_pri_res;
  real_T solver_dua_res;
  real_T x_real_m;
  real_T y_real_m;
  real_T psi_real_rad;
  real_T ax_real_mps2;
  real_T ay_real_mps2;
  real_T vx_real_mps;
  real_T beta_real_rad;
  real_T x_traj_m[41];
  real_T y_traj_m[41];
  real_T psi_traj_rad[41];
  real_T v_traj_mps[41];
  real_T kappa_traj_radpm[41];
  real_T ax_diff_traj_mps2m[41];
  real_T ax_traj_mps2[41];
  real_T ay_traj_mps2[41];
  real_T d_Target_m[41];
  real_T dot_d_Target_mps[41];
  real_T ax_lim_mps2[41];
  real_T ay_lim_mps2[41];
  real_T d_lim_ub_m[41];
  real_T d_lim_lb_m[41];
  real_T flag_bound_Upd;
  real_T flag_A_Upd;
  real_T flag_P_Upd;
  real_T flag_q_Upd;
  real_T flag_solve;
  real_T flag_setup;
  real_T flag_s_request;
  uint32_T tartraj_LapCnt;
  uint32_T tartraj_TrajCnt;
  real_T tartraj_s_loc_m[50];
  real_T tartraj_s_glob_m[50];
  real_T tartraj_x_m[50];
  real_T tartraj_y_m[50];
  real_T tartraj_psi_rad[50];
  real_T tartraj_kappa_radpm[50];
  real_T tartraj_v_mps[50];
  real_T tartraj_ax_mps2[50];
  real_T tartraj_banking_rad[50];
  real_T tartraj_ax_lim_mps2[50];
  real_T tartraj_ay_lim_mps2[50];
  real_T u_opt_total[242];
  real_T x_pred_m[41];
  real_T y_pred_m[41];
  real_T x_pred_left_m[41];
  real_T y_pred_left_m[41];
  real_T x_pred_right_m[41];
  real_T y_pred_right_m[41];
  real_T vx_pred_mps[41];
  real_T d_pred_m[41];
  real_T dot_d_pred_mps[41];
  real_T s_dot_pred_mps[41];
  real_T ax_pred_mps2[41];
  real_T ax_tire_pred_mps2[41];
  real_T ay_pred_mps2[41];
  real_T TireUtilizationTarget;
  real_T cost_values[4];
  real_T ax_dist_mps2[41];
  real_T ay_dist_mps2[41];
  real_T s_dot_lin_mps[41];
  real_T vx_lin_mps[41];
  real_T kappa_lin_radpm[41];
  real_T ax_LearnedBound_mps2;
  real_T ax_DataCoverage_perc;
  real_T ax_Uncertainty_mps2;
  real_T ay_LearnedBound_mps2;
  real_T ay_DataCoverage_perc;
  real_T ay_Uncertainty_mps2;
  real_T LongAcc_FFweights[2];
  real_T LatAcc_FFweights[92];
  real_T LatAcc_mean_debug[91];
  real_T LatAcc_cov_debug[91];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_mvdc_tmpc_fast_debug_
#define DEFINED_TYPEDEF_FOR_mvdc_tmpc_fast_debug_

struct mvdc_tmpc_fast_debug
{
  real_T LatAcc_bank_mps2;
  real_T LatAcc_FB_Dist_rad;
  real_T LatAcc_FB_ay_rad;
  real_T LatAcc_FB_Delta_rad;
  real_T LatAcc_FFss_rad;
  real_T LatAcc_FFdyn_rad;
  real_T LatAcc_Control_rad;
  real_T LatAcc_Target_mps2;
  real_T LatAcc_VirtualAccTarget_mps2;
  real_T LatAcc_VirtualSteeringTarget_rad;
  real_T LatAcc_FFLearned_rad;
  real_T LatAcc_DiffToNeutralSteer_rad;
  real_T LatAcc_FBStabilizer_rad;
  real_T LongAcc_FB_N;
  real_T LongAcc_FFax_N;
  real_T LongAcc_FFdist_N;
  real_T LongAcc_Control_N;
  real_T LongAcc_Target_mps2;
  real_T LongAcc_TargetFF_mps2;
  real_T LongAcc_FFLearned_N;
  real_T state_cost;
  real_T input_cost;
  real_T reg_cost;
  real_T slack_cost;
  real_T solver_iteration;
  real_T solver_state;
  real_T solver_runTime_s;
  real_T solver_pri_res;
  real_T solver_dua_res;
  uint32_T tmpc_cnt;
  real_T TireUtilizationTarget;
  real_T ax_LearnedBound_mps2;
  real_T ax_DataCoverage_perc;
  real_T ax_Uncertainty_mps2;
  real_T ay_LearnedBound_mps2;
  real_T ay_DataCoverage_perc;
  real_T ay_Uncertainty_mps2;
  real_T dot_d_analytical_mps;
  real_T dot_d_numerical_mps;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_50gomenhaqprWvYYDUcxiH_
#define DEFINED_TYPEDEF_FOR_struct_50gomenhaqprWvYYDUcxiH_

struct struct_50gomenhaqprWvYYDUcxiH
{
  real_T spread_vx_mps[7];
  real_T spread_ay_req_mps2[13];
  real_T vx_width_mps;
  real_T ay_width_mps2;
  real_T bf_vx_mps[91];
  real_T bf_ay_req_mps2[91];
  real_T w0[92];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_8SAH98NOUMyLo7unQLNhYF_
#define DEFINED_TYPEDEF_FOR_struct_8SAH98NOUMyLo7unQLNhYF_

struct struct_8SAH98NOUMyLo7unQLNhYF
{
  real_T flag_s_request;
  real_T flag_bound_Upd;
  real_T flag_A_Upd;
  real_T flag_P_Upd;
  real_T flag_q_Upd;
  real_T flag_solve;
  real_T flag_setup;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_rAzch3iZmG57fB7gP9dRwC_
#define DEFINED_TYPEDEF_FOR_struct_rAzch3iZmG57fB7gP9dRwC_

struct struct_rAzch3iZmG57fB7gP9dRwC
{
  real_T solver_solveTime_s;
  real_T solver_runTime_s;
  real_T solver_state;
  real_T solver_iteration;
  real_T solver_updateTime_s;
  real_T solver_sfunTime_s;
  real_T solver_pri_res;
  real_T solver_dua_res;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_xwphvtKqjUwTm215naeozC_
#define DEFINED_TYPEDEF_FOR_struct_xwphvtKqjUwTm215naeozC_

struct struct_xwphvtKqjUwTm215naeozC
{
  PathPos PathPos;
  real_T x_traj_m[41];
  real_T y_traj_m[41];
  real_T psi_traj_rad[41];
  real_T v_traj_mps[41];
  real_T kappa_traj_radpm[41];
  real_T ax_diff_traj_mps2m[41];
  real_T ax_traj_mps2[41];
  real_T ay_traj_mps2[41];
  real_T ax_lim_mps2[41];
  real_T ay_lim_mps2[41];
  real_T tube_r_m[41];
  real_T tube_l_m[41];
  real_T d_Target_m[41];
  real_T dot_d_Target_mps[41];
  real_T d_lim_ub_m[41];
  real_T d_lim_lb_m[41];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_cn5SO9QaZcduShzGK83fPD_
#define DEFINED_TYPEDEF_FOR_struct_cn5SO9QaZcduShzGK83fPD_

struct struct_cn5SO9QaZcduShzGK83fPD
{
  real_T dot_d_analytical_mps;
  real_T dot_d_numerical_mps;
  real_T be_u_abs[3];
  real_T be_l_abs[3];
  real_T s_current_m;
  real_T error_state[3];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_oi7PRKicCuvNrjdewxkrrG_
#define DEFINED_TYPEDEF_FOR_struct_oi7PRKicCuvNrjdewxkrrG_

struct struct_oi7PRKicCuvNrjdewxkrrG
{
  real_T u_opt_total[242];
  real_T x_pred_m[41];
  real_T y_pred_m[41];
  real_T vx_pred_mps[41];
  real_T x_pred_left_m[41];
  real_T y_pred_left_m[41];
  real_T x_pred_right_m[41];
  real_T y_pred_right_m[41];
  real_T d_pred_m[41];
  real_T dot_d_pred_mps[41];
  real_T s_dot_pred_mps[41];
  real_T ax_pred_mps2[41];
  real_T ax_tire_pred_mps2[41];
  real_T ay_pred_mps2[41];
  real_T TireUtilizationTarget;
  real_T cost_values[4];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_XmA17SeQgjzTUvOoWvOe1_
#define DEFINED_TYPEDEF_FOR_struct_XmA17SeQgjzTUvOoWvOe1_

struct struct_XmA17SeQgjzTUvOoWvOe1
{
  real_T ax_dist_mps2[41];
  real_T ay_dist_mps2[41];
  real_T vx_lin_mps[41];
  real_T s_dot_lin_mps[41];
  real_T kappa_lin_radpm[41];
  real_T UncertaintyTube[369];
  real_T v_terminal_mps;
};

#endif

// Custom Type definition for MATLAB Function: '<S37>/transformMPCResult'
#ifndef struct_sdS2AOQvsXuzPerYOOsGtUG_mvdc_mpc_T
#define struct_sdS2AOQvsXuzPerYOOsGtUG_mvdc_mpc_T

struct sdS2AOQvsXuzPerYOOsGtUG_mvdc_mpc_T
{
  char_T dummy[11];
  real_T Q[9];
  real_T R[4];
  real_T roh_u;
  real_T roh_x;
  real_T roh_lin;
  real_T roh_quad;
  real_T rdiff_ax;
  real_T rdiff_ay;
  real_T r_ax;
  real_T slack_lim_rel;
  real_T trigger_print;
  real_T trigger_Pupdate;
  real_T use_scale_units;
  real_T use_constr_precision;
  real_T N_hor;
  real_T Ts;
  real_T A_d[9];
  real_T B_d[6];
  real_T n_sys;
  real_T m_sys;
  real_T S_LQR[9];
  real_T K_LQR[6];
  real_T M_1[9];
  real_T n_constr;
  real_T n_slacks;
  real_T ns_total;
  real_T S_lin_slack[4];
  real_T S_quad_slack[16];
  real_T ABK_MPC[29766];
  real_T Ax0_MPC[369];
  real_T H_states[58564];
  real_T H_inputs[58564];
  real_T H_reg[58564];
  real_T H_slacks[58564];
  real_T f_Dax[10164];
  real_T f_Day[10164];
  real_T f_x0[29766];
  real_T f_D_deltaax[242];
  real_T f_D_deltaay[242];
  real_T f_d_m[9922];
  real_T f_dot_d_mps[9922];
  real_T n_constr_total;
  real_T n_opt_total;
  real_T osqp_m;
  real_T osqp_n;
  real_T A_ineq[88330];
  real_T l_par[365];
  real_T u_par[365];
  real_T osqp_qpar[242];
  real_T P_nnz;
  real_T P_i_lin[1804];
  real_T P_i_par[1804];
  real_T P_p_par[243];
  real_T A_nnz;
  real_T A_i_lin[7742];
  real_T A_x_par[7742];
  real_T A_i_par[7742];
  real_T A_p_par[243];
  real_T P_par[58564];
  real_T P_x_par[1804];
};

#endif                             // struct_sdS2AOQvsXuzPerYOOsGtUG_mvdc_mpc_T

// Parameters (default storage)
typedef struct P_mvdc_mpc_T_ P_mvdc_mpc_T;

// Forward declaration for rtModel
typedef struct tag_RTM_mvdc_mpc_T RT_MODEL_mvdc_mpc_T;

#endif                                 // mvdc_mpc_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
