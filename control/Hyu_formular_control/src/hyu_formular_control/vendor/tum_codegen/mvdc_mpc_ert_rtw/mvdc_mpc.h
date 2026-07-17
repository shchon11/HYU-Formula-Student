//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: mvdc_mpc.h
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
#ifndef mvdc_mpc_h_
#define mvdc_mpc_h_
#include <stdlib.h>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "mvdc_mpc_types.h"

extern "C"
{

#include "rt_nonfinite.h"

}

extern "C"
{

#include "rtGetInf.h"

}

extern "C"
{

#include "rtGetNaN.h"

}

#include "rtw_modelmap.h"
#include <stddef.h>
#include <cstring>

// Block signals (default storage)
struct B_mvdc_mpc_T {
  sdS2AOQvsXuzPerYOOsGtUG_mvdc_mpc_T expl_temp;
  sdS2AOQvsXuzPerYOOsGtUG_mvdc_mpc_T expl_temp_m;
  real_T A_ineq_data[88330];
  real_T P_pred[8464];
  real_T b_I[8464];
  int32_T iv[7742];
  int8_T b_I_c[8464];
  real_T Gain3_p[1001];                // '<S10>/Gain3'
  real_T Sign_l[1001];                 // '<S42>/Sign'
  real_T DataTypeConversion1;          // '<S37>/Data Type Conversion1'
  real_T solution[242];                // '<S37>/S-Function_OSQP'
  real_T dual_solution[365];           // '<S37>/S-Function_OSQP'
  real_T solver_solveTime_s;           // '<S37>/S-Function_OSQP'
  real_T solver_runTime_s;             // '<S37>/S-Function_OSQP'
  real_T solver_state;                 // '<S37>/S-Function_OSQP'
  real_T solver_iteration;             // '<S37>/S-Function_OSQP'
  real_T solver_updateTime_s;          // '<S37>/S-Function_OSQP'
  real_T solver_sfunTime_s;            // '<S37>/S-Function_OSQP'
  real_T solver_pri_res;               // '<S37>/S-Function_OSQP'
  real_T solver_dua_res;               // '<S37>/S-Function_OSQP'
  real_T help_vector_z[365];           // '<S37>/S-Function_OSQP'
  real_T flag_bound_Upd;               // '<S37>/S-Function_OSQP'
  real_T flag_A_Upd;                   // '<S37>/S-Function_OSQP'
  real_T flag_P_Upd;                   // '<S37>/S-Function_OSQP'
  real_T flag_q_Upd;                   // '<S37>/S-Function_OSQP'
  real_T flag_solve;                   // '<S37>/S-Function_OSQP'
  real_T flag_setup;                   // '<S37>/S-Function_OSQP'
  real_T scaling_c;                    // '<S37>/S-Function_OSQP'
  real_T scaling_D[242];               // '<S37>/S-Function_OSQP'
  real_T scaling_E[365];               // '<S37>/S-Function_OSQP'
  real_T Multiply;                     // '<S52>/Multiply'
  real_T u_opt_total[242];             // '<S37>/transformMPCResult'
  real_T x_pred_m[41];                 // '<S37>/transformMPCResult'
  real_T y_pred_m[41];                 // '<S37>/transformMPCResult'
  real_T vx_pred_mps[41];              // '<S37>/transformMPCResult'
  real_T x_pred_left_m[41];            // '<S37>/transformMPCResult'
  real_T y_pred_left_m[41];            // '<S37>/transformMPCResult'
  real_T x_pred_right_m[41];           // '<S37>/transformMPCResult'
  real_T y_pred_right_m[41];           // '<S37>/transformMPCResult'
  real_T d_pred_m[41];                 // '<S37>/transformMPCResult'
  real_T dot_d_pred_mps[41];           // '<S37>/transformMPCResult'
  real_T s_dot_pred_mps[41];           // '<S37>/transformMPCResult'
  real_T ax_pred_mps2[41];             // '<S37>/transformMPCResult'
  real_T ax_tire_pred_mps2[41];        // '<S37>/transformMPCResult'
  real_T ay_pred_mps2[41];             // '<S37>/transformMPCResult'
  real_T TireUtilizationTarget;        // '<S37>/transformMPCResult'
  real_T cost_values[4];               // '<S37>/transformMPCResult'
  real_T f[242];                       // '<S37>/prepareOptimizationProblem'
  real_T lb[365];                      // '<S37>/prepareOptimizationProblem'
  real_T ub[365];                      // '<S37>/prepareOptimizationProblem'
  real_T A_x[7742];                    // '<S37>/prepareOptimizationProblem'
  real_T be_l_abs[3];                  // '<S37>/prepareOptimizationProblem'
  real_T s_current_m;                  // '<S37>/prepareOptimizationProblem'
  real_T error_state[3];               // '<S37>/prepareOptimizationProblem'
  real_T flag_s_request;               // '<S37>/prepareLinearization'
  real_T x_traj[41];                   // '<S37>/prepareLinearization'
  real_T y_traj[41];                   // '<S37>/prepareLinearization'
  real_T psi_traj[41];                 // '<S37>/prepareLinearization'
  real_T v_traj[41];                   // '<S37>/prepareLinearization'
  real_T kappa_traj[41];               // '<S37>/prepareLinearization'
  real_T ax_diff_traj[41];             // '<S37>/prepareLinearization'
  real_T ax_traj[41];                  // '<S37>/prepareLinearization'
  real_T ay_traj[41];                  // '<S37>/prepareLinearization'
  real_T ax_lim_mps2_tartraj[41];      // '<S37>/prepareLinearization'
  real_T ay_lim_mps2_tartraj[41];      // '<S37>/prepareLinearization'
  real_T d_Target_m[41];               // '<S37>/prepareLinearization'
  real_T dot_d_Target_mps[41];         // '<S37>/prepareLinearization'
  real_T d_lim_ub_m[41];               // '<S37>/prepareLinearization'
  real_T d_lim_lb_m[41];               // '<S37>/prepareLinearization'
  real_T ax_dist_mps2[41];             // '<S37>/prepareLinearization'
  real_T ay_dist_mps2[41];             // '<S37>/prepareLinearization'
  real_T vx_lin[41];                   // '<S37>/prepareLinearization'
  real_T s_dot_lin[41];                // '<S37>/prepareLinearization'
  real_T kappa_lin[41];                // '<S37>/prepareLinearization'
  real_T Abs;                          // '<S43>/Abs'
  real_T UnitDelay;                    // '<S43>/Unit Delay'
  real_T Gain3;                        // '<S43>/Gain3'
  real_T Abs_o;                        // '<S42>/Abs'
  real_T UnitDelay_g;                  // '<S42>/Unit Delay'
  real_T Gain3_g;                      // '<S42>/Gain3'
  real_T mean_debug[91];               // '<S27>/MATLAB Function'
  real_T cov_debug[91];                // '<S27>/MATLAB Function'
  boolean_T LogicalOperator;           // '<S37>/Logical Operator'
};

// Block states (default storage) for system '<Root>'
struct DW_mvdc_mpc_T {
  real_T UnitDelay1_DSTATE;            // '<S6>/Unit Delay1'
  real_T UD_DSTATE;                    // '<S28>/UD'
  real_T UnitDelay_DSTATE[2];          // '<S31>/Unit Delay'
  real_T UnitDelay1_DSTATE_d;          // '<S7>/Unit Delay1'
  real_T UD_DSTATE_f;                  // '<S33>/UD'
  real_T DiscreteTimeIntegrator_DSTATE;// '<S9>/Discrete-Time Integrator'
  real_T UD_DSTATE_b;                  // '<S21>/UD'
  real_T UD_DSTATE_p;                  // '<S20>/UD'
  real_T UnitDelay_DSTATE_p;           // '<S17>/Unit Delay'
  real_T UD_DSTATE_fv;                 // '<S22>/UD'
  real_T UnitDelay_DSTATE_n;           // '<S19>/Unit Delay'
  real_T UD_DSTATE_e;                  // '<S24>/UD'
  real_T UnitDelay_DSTATE_o;           // '<S18>/Unit Delay'
  real_T UD_DSTATE_l;                  // '<S23>/UD'
  real_T UD_DSTATE_n;                  // '<S59>/UD'
  real_T UD_DSTATE_h;                  // '<S60>/UD'
  real_T DelayInput1_DSTATE;           // '<S50>/Delay Input1'
  real_T TappedDelay_X[1000];          // '<S43>/Tapped Delay'
  real_T UnitDelay_DSTATE_j;           // '<S43>/Unit Delay'
  real_T TappedDelay_X_c[1000];        // '<S42>/Tapped Delay'
  real_T UnitDelay_DSTATE_a;           // '<S42>/Unit Delay'
  real_T UnitDelay1_DSTATE_c[8464];    // '<S27>/Unit Delay1'
  real_T UnitDelay2_DSTATE[92];        // '<S27>/Unit Delay2'
  real_T Memory_PreviousInput_l[41];   // '<S3>/Memory'
  real_T Memory1_PreviousInput[41];    // '<S3>/Memory1'
  real_T Memory4_11_PreviousInput[41]; // '<S37>/Memory4'
  real_T Memory4_4_PreviousInput[41];  // '<S37>/Memory4'
  real_T Memory6_PreviousInput[41];    // '<S37>/Memory6'
  real_T Memory7_PreviousInput[41];    // '<S37>/Memory7'
  real_T Memory2_PreviousInput[41];    // '<S37>/Memory2'
  real_T Memory3_PreviousInput[41];    // '<S37>/Memory3'
  real_T Memory4_1_PreviousInput[242]; // '<S37>/Memory4'
  struct {
    void *wrapper;
  } SFunction_OSQP_PWORK;              // '<S37>/S-Function_OSQP'

  uint32_T Output_DSTATE;              // '<S39>/Output'
  boolean_T Delay_DSTATE[1000];        // '<S43>/Delay'
  boolean_T Delay_DSTATE_f[1000];      // '<S42>/Delay'
  int8_T DiscreteTimeIntegrator_PrevResetState;// '<S9>/Discrete-Time Integrator' 
};

// Constant parameters (default storage)
struct ConstP_mvdc_mpc_T {
  // Expression: exp(-tSSlow/P_VDC_LongAcc_FF_Ts*(0:1:sys.N_hor))/sum(exp(-tSSlow/P_VDC_LongAcc_FF_Ts*(0:1:sys.N_hor)))
  //  Referenced by: '<S32>/Gain'

  real_T Gain_rtw_collapsed_sub_expr_gFggD8XbsHGI0QdZZAiwlE_1[41];
};

// External inputs (root inport signals with default storage)
struct ExtU_mvdc_mpc_T {
  boolean_T EnableEmergency;           // '<Root>/EnableEmergency'
  Trajectory TargetTrajectory;         // '<Root>/TargetTrajectory'
  VehicleDynamicState VehicleDynamicState_p;// '<Root>/VehicleDynamicState'
  boolean_T EnableDrivingController;   // '<Root>/EnableDrivingController'
  TrajectoryPoint ActualTrajectoryPoint;// '<Root>/ActualTrajectoryPoint'
  VehicleControl ActualVehicleControl; // '<Root>/ActualVehicleControl'
  ActuatorLimitations ActuatorLimitations_e;// '<Root>/ActuatorLimitations'
};

// External outputs (root outports fed by signals with default storage)
struct ExtY_mvdc_mpc_T {
  real_T RequestSteeringAngle_rad;     // '<Root>/RequestSteeringAngle_rad'
  real_T RequestLongForce_N;           // '<Root>/RequestLongForce_N'
  TUMHealthStatus TubeMPCStatus;       // '<Root>/TubeMPCStatus'
  mvdc_tube_mpc_debug mvdc_tube_mpc_debug_c;// '<Root>/mvdc_tube_mpc_debug'
  mvdc_tmpc_fast_debug mvdc_tmpc_fast_debug_d;// '<Root>/mvdc_tmpc_fast_debug'
};

// Parameters (default storage)
struct P_mvdc_mpc_T_ {
  struct_50gomenhaqprWvYYDUcxiH acc_control_learning;// Variable: acc_control_learning
                                                        //  Referenced by:
                                                        //    '<S10>/Constant6'
                                                        //    '<S10>/Constant7'
                                                        //    '<S10>/Gain2'
                                                        //    '<S10>/Gain3'
                                                        //    '<S11>/Constant5'
                                                        //    '<S11>/Constant6'
                                                        //    '<S11>/Gain'
                                                        //    '<S11>/Gain1'
                                                        //    '<S27>/w_upd'
                                                        //    '<S27>/MATLAB Function'
                                                        //    '<S27>/Unit Delay2'

  real_T A_VehicleReference_m2;        // Variable: A_VehicleReference_m2
                                          //  Referenced by: '<S30>/Gain7'

  real_T P_VDC_ControlMargin_ax_mps2;  // Variable: P_VDC_ControlMargin_ax_mps2
                                          //  Referenced by: '<S37>/prepareLinearization'

  real_T P_VDC_ControlMargin_ay_mps2;  // Variable: P_VDC_ControlMargin_ay_mps2
                                          //  Referenced by: '<S37>/prepareLinearization'

  real_T P_VDC_Dist_ax_perc;           // Variable: P_VDC_Dist_ax_perc
                                          //  Referenced by: '<S36>/Constant3'

  real_T P_VDC_Dist_ay_perc;           // Variable: P_VDC_Dist_ay_perc
                                          //  Referenced by: '<S36>/Constant4'

  real_T P_VDC_IncreaseUncertaintyPerStep_perc;
                              // Variable: P_VDC_IncreaseUncertaintyPerStep_perc
                                 //  Referenced by: '<S37>/prepareLinearization'

  real_T P_VDC_LatAccKi;               // Variable: P_VDC_LatAccKi
                                          //  Referenced by: '<S9>/Gain2'

  real_T P_VDC_LatAccKi_Lim_perc;      // Variable: P_VDC_LatAccKi_Lim_perc
                                          //  Referenced by: '<S9>/Gain4'

  real_T P_VDC_LatAccKp_Delta;         // Variable: P_VDC_LatAccKp_Delta
                                          //  Referenced by: '<S9>/Gain1'

  real_T P_VDC_LatAccKp_ay;            // Variable: P_VDC_LatAccKp_ay
                                          //  Referenced by: '<S9>/Gain3'

  real_T P_VDC_LatAcc_BetaGain;        // Variable: P_VDC_LatAcc_BetaGain
                                          //  Referenced by: '<S10>/Gain'

  real_T P_VDC_LatAcc_FF_Ts;           // Variable: P_VDC_LatAcc_FF_Ts
                                          //  Referenced by:
                                          //    '<S21>/Gain'
                                          //    '<S22>/Gain'
                                          //    '<S23>/Gain'
                                          //    '<S24>/Gain'

  real_T P_VDC_LatAcc_Steering_Ts;     // Variable: P_VDC_LatAcc_Steering_Ts
                                          //  Referenced by:
                                          //    '<S17>/Gain1'
                                          //    '<S17>/Gain2'
                                          //    '<S17>/Gain3'
                                          //    '<S18>/Gain1'
                                          //    '<S18>/Gain2'
                                          //    '<S18>/Gain3'
                                          //    '<S19>/Gain1'
                                          //    '<S19>/Gain2'
                                          //    '<S19>/Gain3'
                                          //    '<S28>/Gain'

  real_T P_VDC_LatAcc_UndersteerComp_Ts;
                                     // Variable: P_VDC_LatAcc_UndersteerComp_Ts
                                        //  Referenced by: '<S20>/Gain'

  real_T P_VDC_LatAcc_VehicleDynamics_Ts;
                                    // Variable: P_VDC_LatAcc_VehicleDynamics_Ts
                                       //  Referenced by:
                                       //    '<S17>/Gain1'
                                       //    '<S17>/Gain2'
                                       //    '<S17>/Gain3'
                                       //    '<S18>/Gain1'
                                       //    '<S18>/Gain2'
                                       //    '<S18>/Gain3'
                                       //    '<S19>/Gain1'
                                       //    '<S19>/Gain2'
                                       //    '<S19>/Gain3'

  real_T P_VDC_LatErrorDer_Ts_s;       // Variable: P_VDC_LatErrorDer_Ts_s
                                          //  Referenced by: '<S60>/Gain'

  real_T P_VDC_LongAccKp;              // Variable: P_VDC_LongAccKp
                                          //  Referenced by: '<S7>/Gain'

  real_T P_VDC_LongAcc_FB_Ts;          // Variable: P_VDC_LongAcc_FB_Ts
                                          //  Referenced by: '<S33>/Gain'

  real_T P_VDC_LongAcc_FF_Ts;          // Variable: P_VDC_LongAcc_FF_Ts
                                          //  Referenced by: '<S32>/Gain'

  real_T P_VDC_LongAcc_LimFb_N;        // Variable: P_VDC_LongAcc_LimFb_N
                                          //  Referenced by: '<S7>/Saturation'

  real_T P_VDC_MaxTightening;          // Variable: P_VDC_MaxTightening
                                          //  Referenced by: '<S37>/prepareOptimizationProblem'

  real_T P_VDC_MinVelSlipCalc_mps;     // Variable: P_VDC_MinVelSlipCalc_mps
                                          //  Referenced by:
                                          //    '<S4>/Constant'
                                          //    '<S4>/Switch'
                                          //    '<S37>/prepareLinearization'
                                          //    '<S37>/prepareOptimizationProblem'
                                          //    '<S9>/Constant2'
                                          //    '<S9>/Switch1'
                                          //    '<S9>/Switch7'
                                          //    '<S9>/Switch8'
                                          //    '<S10>/Constant'
                                          //    '<S31>/Switch'
                                          //    '<S13>/Constant'
                                          //    '<S14>/Constant'
                                          //    '<S15>/Constant'
                                          //    '<S25>/Constant'

  real_T P_VDC_PositiveAxLimScale;     // Variable: P_VDC_PositiveAxLimScale
                                          //  Referenced by:
                                          //    '<S37>/prepareOptimizationProblem'
                                          //    '<S37>/transformMPCResult'

  real_T P_VDC_RTISQP_alpha_old;       // Variable: P_VDC_RTISQP_alpha_old
                                          //  Referenced by: '<S37>/prepareLinearization'

  real_T P_VDC_RTISQP_alpha_target;    // Variable: P_VDC_RTISQP_alpha_target
                                          //  Referenced by: '<S37>/prepareLinearization'

  real_T P_VDC_StabilizerRearSideSlip_rad;
                                   // Variable: P_VDC_StabilizerRearSideSlip_rad
                                      //  Referenced by:
                                      //    '<S9>/Saturation'
                                      //    '<S12>/Constant'

  real_T P_VDC_SteeringOvalComp_rad;   // Variable: P_VDC_SteeringOvalComp_rad
                                          //  Referenced by: '<S10>/Constant3'

  real_T P_VDC_SteeringSteadyStateUndersteerComp_rad;
                        // Variable: P_VDC_SteeringSteadyStateUndersteerComp_rad
                           //  Referenced by: '<S10>/Constant8'

  real_T P_VDC_TMPCMaxSolverIter;      // Variable: P_VDC_TMPCMaxSolverIter
                                          //  Referenced by: '<S37>/S-Function_OSQP'

  real_T P_VDC_TerminalSetBrakeTime_s; // Variable: P_VDC_TerminalSetBrakeTime_s
                                          //  Referenced by: '<S37>/prepareLinearization'

  real_T P_VDC_TuneTerminalSet_mps;    // Variable: P_VDC_TuneTerminalSet_mps
                                          //  Referenced by: '<S37>/prepareLinearization'

  real_T P_VDC_UncertaintyLearningGain;
                                      // Variable: P_VDC_UncertaintyLearningGain
                                         //  Referenced by:
                                         //    '<S42>/Gain2'
                                         //    '<S43>/Gain2'

  real_T P_VDC_UncertaintyLearningSamples;
                                   // Variable: P_VDC_UncertaintyLearningSamples
                                      //  Referenced by:
                                      //    '<S42>/Gain5'
                                      //    '<S43>/Gain5'

  real_T P_VDC_UncertaintyTarget_perc; // Variable: P_VDC_UncertaintyTarget_perc
                                          //  Referenced by:
                                          //    '<S42>/Gain'
                                          //    '<S42>/Gain1'
                                          //    '<S43>/Gain'
                                          //    '<S43>/Gain1'

  real_T P_VDC_VirtualController[6];   // Variable: P_VDC_VirtualController
                                          //  Referenced by:
                                          //    '<S37>/prepareLinearization'
                                          //    '<S37>/prepareOptimizationProblem'

  real_T P_VDC_ayFFModelLearningRate;  // Variable: P_VDC_ayFFModelLearningRate
                                          //  Referenced by:
                                          //    '<S11>/Constant'
                                          //    '<S31>/Gain9'

  real_T drag_coefficient;             // Variable: drag_coefficient
                                          //  Referenced by:
                                          //    '<S37>/prepareLinearization'
                                          //    '<S37>/prepareOptimizationProblem'
                                          //    '<S37>/transformMPCResult'
                                          //    '<S30>/Gain2'

  real_T l_front_m;                    // Variable: l_front_m
                                          //  Referenced by:
                                          //    '<S4>/Gain'
                                          //    '<S10>/Gain1'
                                          //    '<S10>/Gain4'

  real_T l_rear_m;                     // Variable: l_rear_m
                                          //  Referenced by:
                                          //    '<S4>/Gain'
                                          //    '<S10>/Gain1'
                                          //    '<S10>/Gain4'

  real_T roh_air;                      // Variable: roh_air
                                          //  Referenced by:
                                          //    '<S37>/prepareLinearization'
                                          //    '<S37>/prepareOptimizationProblem'
                                          //    '<S37>/transformMPCResult'
                                          //    '<S30>/Gain3'

  real_T tS;                           // Variable: tS
                                          //  Referenced by:
                                          //    '<S33>/Gain'
                                          //    '<S17>/Gain1'
                                          //    '<S17>/Gain2'
                                          //    '<S17>/Gain3'
                                          //    '<S18>/Gain1'
                                          //    '<S18>/Gain2'
                                          //    '<S18>/Gain3'
                                          //    '<S19>/Gain1'
                                          //    '<S19>/Gain2'
                                          //    '<S19>/Gain3'
                                          //    '<S20>/Gain'
                                          //    '<S21>/Gain'
                                          //    '<S28>/Gain'
                                          //    '<S60>/Gain'
                                          //    '<S22>/Gain'
                                          //    '<S23>/Gain'
                                          //    '<S24>/Gain'

  real_T tSSlow;                       // Variable: tSSlow
                                          //  Referenced by:
                                          //    '<S35>/Gain'
                                          //    '<S35>/Gain1'
                                          //    '<S35>/Gain2'
                                          //    '<S35>/Gain3'
                                          //    '<S32>/Gain'
                                          //    '<S16>/Gain2'

  real_T vehiclemass_kg;               // Variable: vehiclemass_kg
                                          //  Referenced by:
                                          //    '<S37>/prepareLinearization'
                                          //    '<S37>/prepareOptimizationProblem'
                                          //    '<S37>/transformMPCResult'
                                          //    '<S31>/Gain6'
                                          //    '<S32>/Gain1'

  boolean_T P_VDC_EnableNumLatErrorDer_b;// Variable: P_VDC_EnableNumLatErrorDer_b
                                            //  Referenced by: '<S37>/prepareOptimizationProblem'

  boolean_T P_VDC_EnableTubeMPC;       // Variable: P_VDC_EnableTubeMPC
                                          //  Referenced by: '<S38>/Constant'

  boolean_T P_VDC_EnableUncertaintyLearning;
                                    // Variable: P_VDC_EnableUncertaintyLearning
                                       //  Referenced by: '<S36>/Constant1'

  boolean_T P_VDC_EnableYawRateAccRepl;// Variable: P_VDC_EnableYawRateAccRepl
                                          //  Referenced by: '<S9>/Constant1'

  boolean_T P_VDC_LatAcc_EnableForesightFF;
                                     // Variable: P_VDC_LatAcc_EnableForesightFF
                                        //  Referenced by:
                                        //    '<S10>/Constant1'
                                        //    '<S10>/Constant5'

  boolean_T P_VDC_LongAcc_EnableForesightFF;
                                    // Variable: P_VDC_LongAcc_EnableForesightFF
                                       //  Referenced by: '<S32>/Constant1'

  boolean_T P_VDC_axFFModelLearningEnable;
                                      // Variable: P_VDC_axFFModelLearningEnable
                                         //  Referenced by: '<S31>/Constant3'

  boolean_T P_VDC_ayFFModelLearningEnable;
                                      // Variable: P_VDC_ayFFModelLearningEnable
                                         //  Referenced by: '<S10>/Constant2'

  real_T TransferFcnFirstOrder_ICPrevOutput;
                           // Mask Parameter: TransferFcnFirstOrder_ICPrevOutput
                              //  Referenced by: '<S60>/UD'

  real_T TransferFcnFirstOrder_ICPrevOutput_b;
                         // Mask Parameter: TransferFcnFirstOrder_ICPrevOutput_b
                            //  Referenced by: '<S28>/UD'

  real_T TransferFcnFirstOrder_ICPrevOutput_p;
                         // Mask Parameter: TransferFcnFirstOrder_ICPrevOutput_p
                            //  Referenced by: '<S33>/UD'

  real_T TransferFcnFirstOrder1_ICPrevOutput;
                          // Mask Parameter: TransferFcnFirstOrder1_ICPrevOutput
                             //  Referenced by: '<S21>/UD'

  real_T TransferFcnFirstOrder_ICPrevOutput_f;
                         // Mask Parameter: TransferFcnFirstOrder_ICPrevOutput_f
                            //  Referenced by: '<S20>/UD'

  real_T TransferFcnFirstOrder1_ICPrevOutput_c;
                        // Mask Parameter: TransferFcnFirstOrder1_ICPrevOutput_c
                           //  Referenced by: '<S22>/UD'

  real_T TransferFcnFirstOrder1_ICPrevOutput_d;
                        // Mask Parameter: TransferFcnFirstOrder1_ICPrevOutput_d
                           //  Referenced by: '<S24>/UD'

  real_T TransferFcnFirstOrder1_ICPrevOutput_dx;
                       // Mask Parameter: TransferFcnFirstOrder1_ICPrevOutput_dx
                          //  Referenced by: '<S23>/UD'

  real_T DiscreteDerivative_ICPrevScaledInput;
                         // Mask Parameter: DiscreteDerivative_ICPrevScaledInput
                            //  Referenced by: '<S59>/UD'

  real_T CompareToConstant1_const;   // Mask Parameter: CompareToConstant1_const
                                        //  Referenced by: '<S44>/Constant'

  real_T CompareToConstant2_const;   // Mask Parameter: CompareToConstant2_const
                                        //  Referenced by: '<S45>/Constant'

  real_T CompareToConstant3_const;   // Mask Parameter: CompareToConstant3_const
                                        //  Referenced by: '<S46>/Constant'

  real_T CompareToConstant4_const;   // Mask Parameter: CompareToConstant4_const
                                        //  Referenced by: '<S47>/Constant'

  real_T CompareToConstant5_const;   // Mask Parameter: CompareToConstant5_const
                                        //  Referenced by: '<S48>/Constant'

  real_T CompareToConstant7_const;   // Mask Parameter: CompareToConstant7_const
                                        //  Referenced by: '<S49>/Constant'

  real_T CompareToConstant_const;     // Mask Parameter: CompareToConstant_const
                                         //  Referenced by: '<S56>/Constant'

  real_T CompareToConstant1_const_h;
                                   // Mask Parameter: CompareToConstant1_const_h
                                      //  Referenced by: '<S57>/Constant'

  real_T CompareToConstant2_const_m;
                                   // Mask Parameter: CompareToConstant2_const_m
                                      //  Referenced by: '<S58>/Constant'

  real_T CompareToConstant1_const_c;
                                   // Mask Parameter: CompareToConstant1_const_c
                                      //  Referenced by: '<S26>/Constant'

  real_T CompareToConstant_const_l; // Mask Parameter: CompareToConstant_const_l
                                       //  Referenced by: '<S8>/Constant'

  real_T TappedDelay_vinit;            // Mask Parameter: TappedDelay_vinit
                                          //  Referenced by: '<S42>/Tapped Delay'

  real_T TappedDelay_vinit_k;          // Mask Parameter: TappedDelay_vinit_k
                                          //  Referenced by: '<S43>/Tapped Delay'

  real_T DetectIncrease_vinit;         // Mask Parameter: DetectIncrease_vinit
                                          //  Referenced by: '<S50>/Delay Input1'

  uint32_T WrapToZero_Threshold;       // Mask Parameter: WrapToZero_Threshold
                                          //  Referenced by: '<S41>/FixPt Switch'

  uint32_T CompareToConstant2_const_c;
                                   // Mask Parameter: CompareToConstant2_const_c
                                      //  Referenced by: '<S62>/Constant'

  TUMHealthStatus CompareToConstant1_const_ht;
                                  // Mask Parameter: CompareToConstant1_const_ht
                                     //  Referenced by: '<S61>/Constant'

  struct_oi7PRKicCuvNrjdewxkrrG Prediction_Y0;// Computed Parameter: Prediction_Y0
                                                 //  Referenced by: '<S37>/Prediction'

  struct_xwphvtKqjUwTm215naeozC ResampledTargetTrajectory_Y0;
                             // Computed Parameter: ResampledTargetTrajectory_Y0
                                //  Referenced by: '<S37>/ResampledTargetTrajectory'

  struct_XmA17SeQgjzTUvOoWvOe1 Linearization_Y0;// Computed Parameter: Linearization_Y0
                                                   //  Referenced by: '<S37>/Linearization'

  struct_cn5SO9QaZcduShzGK83fPD MiscDebug_Y0;// Computed Parameter: MiscDebug_Y0
                                                //  Referenced by: '<S37>/MiscDebug'

  struct_rAzch3iZmG57fB7gP9dRwC solver_details_Y0;// Computed Parameter: solver_details_Y0
                                                     //  Referenced by: '<S37>/solver_details'

  struct_8SAH98NOUMyLo7unQLNhYF exitflags_Y0;// Computed Parameter: exitflags_Y0
                                                //  Referenced by: '<S37>/exitflags'

  real_T Constant5_Value;              // Expression: 0
                                          //  Referenced by: '<S9>/Constant5'

  real_T Gain_Gain;                    // Expression: 0.5
                                          //  Referenced by: '<S9>/Gain'

  real_T Constant6_Value;              // Expression: 0
                                          //  Referenced by: '<S9>/Constant6'

  real_T Constant_Value;               // Expression: 0
                                          //  Referenced by: '<S9>/Constant'

  real_T Constant7_Value;              // Expression: 0
                                          //  Referenced by: '<S9>/Constant7'

  real_T Constant9_Value;              // Expression: 0
                                          //  Referenced by: '<S9>/Constant9'

  real_T Constant4_Value;              // Expression: 5
                                          //  Referenced by: '<S9>/Constant4'

  real_T Constant3_Value;              // Expression: 3
                                          //  Referenced by: '<S9>/Constant3'

  real_T Constant10_Value;             // Expression: 0
                                          //  Referenced by: '<S9>/Constant10'

  real_T Constant4_Value_o;            // Expression: 0
                                          //  Referenced by: '<S10>/Constant4'

  real_T P_upd_Y0[8464];
                       // Expression: 1e-10*eye(length(acc_control_learning.w0))
                          //  Referenced by: '<S27>/P_upd'

  real_T mean_debug_Y0;                // Computed Parameter: mean_debug_Y0
                                          //  Referenced by: '<S27>/mean_debug'

  real_T cov_debug_Y0;                 // Computed Parameter: cov_debug_Y0
                                          //  Referenced by: '<S27>/cov_debug'

  real_T UnitDelay1_InitialCondition[8464];
                       // Expression: 1e-10*eye(length(acc_control_learning.w0))
                          //  Referenced by: '<S27>/Unit Delay1'

  real_T Gain10_Gain;                  // Expression: -1
                                          //  Referenced by: '<S31>/Gain10'

  real_T Constant4_Value_b;            // Expression: 0
                                          //  Referenced by: '<S31>/Constant4'

  real_T Constant_Value_h;           // Expression: P_VDC_axFFModelLearningReg^2
                                        //  Referenced by: '<S31>/Constant'

  real_T Constant1_Value;              // Expression: 0
                                          //  Referenced by: '<S31>/Constant1'

  real_T Constant_Value_j;             // Expression: 0
                                          //  Referenced by: '<S7>/Constant'

  real_T Saturation1_UpperSat;         // Expression: 0
                                          //  Referenced by: '<S7>/Saturation1'

  real_T Saturation1_LowerSat;         // Expression: -inf
                                          //  Referenced by: '<S7>/Saturation1'

  real_T Saturation1_UpperSat_c;       // Expression: 0
                                          //  Referenced by: '<S42>/Saturation1'

  real_T Saturation1_LowerSat_p;       // Expression: -inf
                                          //  Referenced by: '<S42>/Saturation1'

  real_T Constant_Value_p;             // Expression: 0
                                          //  Referenced by: '<S42>/Constant'

  real_T ax_LearnedBound_mps2_Y0; // Computed Parameter: ax_LearnedBound_mps2_Y0
                                     //  Referenced by: '<S42>/ax_LearnedBound_mps2'

  real_T ax_DataCoverage_perc_Y0; // Computed Parameter: ax_DataCoverage_perc_Y0
                                     //  Referenced by: '<S42>/ax_DataCoverage_perc'

  real_T UnitDelay_InitialCondition;   // Expression: 0
                                          //  Referenced by: '<S42>/Unit Delay'

  real_T Saturation_UpperSat;          // Expression: inf
                                          //  Referenced by: '<S42>/Saturation'

  real_T Saturation_LowerSat;          // Expression: 0
                                          //  Referenced by: '<S42>/Saturation'

  real_T Constant4_Value_e;            // Expression: 1
                                          //  Referenced by: '<S42>/Constant4'

  real_T Gain3_Gain;                   // Expression: 100
                                          //  Referenced by: '<S42>/Gain3'

  real_T Saturation1_UpperSat_g;       // Expression: 0
                                          //  Referenced by: '<S43>/Saturation1'

  real_T Saturation1_LowerSat_j;       // Expression: -inf
                                          //  Referenced by: '<S43>/Saturation1'

  real_T Constant_Value_ht;            // Expression: 0
                                          //  Referenced by: '<S43>/Constant'

  real_T ay_LearnedBound_mps2_Y0; // Computed Parameter: ay_LearnedBound_mps2_Y0
                                     //  Referenced by: '<S43>/ay_LearnedBound_mps2'

  real_T ay_DataCoverage_perc_Y0; // Computed Parameter: ay_DataCoverage_perc_Y0
                                     //  Referenced by: '<S43>/ay_DataCoverage_perc'

  real_T UnitDelay_InitialCondition_a; // Expression: 0
                                          //  Referenced by: '<S43>/Unit Delay'

  real_T Saturation_UpperSat_g;        // Expression: inf
                                          //  Referenced by: '<S43>/Saturation'

  real_T Saturation_LowerSat_p;        // Expression: 0
                                          //  Referenced by: '<S43>/Saturation'

  real_T Constant4_Value_m;            // Expression: 1
                                          //  Referenced by: '<S43>/Constant4'

  real_T Gain3_Gain_a;                 // Expression: 100
                                          //  Referenced by: '<S43>/Gain3'

  real_T Memory4_11_InitialCondition;  // Expression: 0
                                          //  Referenced by: '<S37>/Memory4'

  real_T Memory4_4_InitialCondition;   // Expression: 0
                                          //  Referenced by: '<S37>/Memory4'

  real_T Memory6_InitialCondition;     // Expression: 0
                                          //  Referenced by: '<S37>/Memory6'

  real_T Memory7_InitialCondition;     // Expression: 0
                                          //  Referenced by: '<S37>/Memory7'

  real_T TSamp_WtEt;                   // Computed Parameter: TSamp_WtEt
                                          //  Referenced by: '<S59>/TSamp'

  real_T Memory2_InitialCondition;     // Expression: 0
                                          //  Referenced by: '<S37>/Memory2'

  real_T Memory3_InitialCondition;     // Expression: 0
                                          //  Referenced by: '<S37>/Memory3'

  real_T Memory4_1_InitialCondition;   // Expression: 0
                                          //  Referenced by: '<S37>/Memory4'

  real_T SFunction_OSQP_P1[242];       // Expression: sys.osqp_qpar'
                                          //  Referenced by: '<S37>/S-Function_OSQP'

  real_T SFunction_OSQP_P2;            // Expression: sys.osqp_m
                                          //  Referenced by: '<S37>/S-Function_OSQP'

  real_T SFunction_OSQP_P3;            // Expression: sys.osqp_n
                                          //  Referenced by: '<S37>/S-Function_OSQP'

  real_T SFunction_OSQP_P4[1804];      // Expression: sys.P_x_par
                                          //  Referenced by: '<S37>/S-Function_OSQP'

  real_T SFunction_OSQP_P5[1804];      // Expression: sys.P_i_par
                                          //  Referenced by: '<S37>/S-Function_OSQP'

  real_T SFunction_OSQP_P6[243];       // Expression: sys.P_p_par
                                          //  Referenced by: '<S37>/S-Function_OSQP'

  real_T SFunction_OSQP_P7[7742];      // Expression: sys.A_x_par
                                          //  Referenced by: '<S37>/S-Function_OSQP'

  real_T SFunction_OSQP_P8[7742];      // Expression: sys.A_i_par
                                          //  Referenced by: '<S37>/S-Function_OSQP'

  real_T SFunction_OSQP_P9[243];       // Expression: sys.A_p_par
                                          //  Referenced by: '<S37>/S-Function_OSQP'

  real_T SFunction_OSQP_P10[365];      // Expression: sys.l_par'
                                          //  Referenced by: '<S37>/S-Function_OSQP'

  real_T SFunction_OSQP_P11[365];      // Expression: sys.u_par'
                                          //  Referenced by: '<S37>/S-Function_OSQP'

  real_T SFunction_OSQP_P12;           // Expression: sys.trigger_print
                                          //  Referenced by: '<S37>/S-Function_OSQP'

  real_T Gain1_Gain;                   // Expression: 1/3
                                          //  Referenced by: '<S4>/Gain1'

  real_T Memory_InitialCondition;      // Expression: 0
                                          //  Referenced by: '<Root>/Memory'

  real_T UnitDelay1_InitialCondition_n;// Expression: 0
                                          //  Referenced by: '<S6>/Unit Delay1'

  real_T Memory_InitialCondition_a;    // Expression: 0
                                          //  Referenced by: '<S3>/Memory'

  real_T Memory1_InitialCondition;     // Expression: 0
                                          //  Referenced by: '<S3>/Memory1'

  real_T UnitDelay_InitialCondition_j[2];// Expression: [0; 0]
                                            //  Referenced by: '<S31>/Unit Delay'

  real_T Constant2_Value;              // Expression: 0
                                          //  Referenced by: '<S31>/Constant2'

  real_T UnitDelay1_InitialCondition_d;// Expression: 0
                                          //  Referenced by: '<S7>/Unit Delay1'

  real_T Saturation1_UpperSat_o;       // Expression: 0
                                          //  Referenced by: '<S31>/Saturation1'

  real_T Saturation1_LowerSat_g;       // Expression: -inf
                                          //  Referenced by: '<S31>/Saturation1'

  real_T Constant8_Value;              // Expression: 0
                                          //  Referenced by: '<S9>/Constant8'

  real_T DiscreteTimeIntegrator_gainval;
                           // Computed Parameter: DiscreteTimeIntegrator_gainval
                              //  Referenced by: '<S9>/Discrete-Time Integrator'

  real_T DiscreteTimeIntegrator_IC;    // Expression: 0
                                          //  Referenced by: '<S9>/Discrete-Time Integrator'

  real_T UnitDelay_InitialCondition_p; // Expression: 0
                                          //  Referenced by: '<S17>/Unit Delay'

  real_T UnitDelay_InitialCondition_jd;// Expression: 0
                                          //  Referenced by: '<S19>/Unit Delay'

  real_T UnitDelay_InitialCondition_k; // Expression: 0
                                          //  Referenced by: '<S18>/Unit Delay'

  real_T Gain4_Gain;                   // Expression: 0.5
                                          //  Referenced by: '<S30>/Gain4'

  real_T Constant_Value_a;             // Expression: 0
                                          //  Referenced by: '<S6>/Constant'

  uint32_T Constant_Value_m;           // Computed Parameter: Constant_Value_m
                                          //  Referenced by: '<S41>/Constant'

  uint32_T Output_InitialCondition;
                                  // Computed Parameter: Output_InitialCondition
                                     //  Referenced by: '<S39>/Output'

  uint32_T FixPtConstant_Value;       // Computed Parameter: FixPtConstant_Value
                                         //  Referenced by: '<S40>/FixPt Constant'

  TUMHealthStatus Constant_Value_c;    // Expression: TUMHealthStatus.OK
                                          //  Referenced by: '<S2>/Constant'

  TUMHealthStatus Constant1_Value_c;   // Expression: TUMHealthStatus.ERROR
                                          //  Referenced by: '<S2>/Constant1'

  boolean_T Delay_InitialCondition;// Computed Parameter: Delay_InitialCondition
                                      //  Referenced by: '<S42>/Delay'

  boolean_T Delay_InitialCondition_n;
                                 // Computed Parameter: Delay_InitialCondition_n
                                    //  Referenced by: '<S43>/Delay'

  boolean_T OptimizerOK_Y0;            // Computed Parameter: OptimizerOK_Y0
                                          //  Referenced by: '<S37>/OptimizerOK'

};

// Real-time Model Data Structure
struct tag_RTM_mvdc_mpc_T {
  const char_T * volatile errorStatus;
  B_mvdc_mpc_T *blockIO;
  ExtU_mvdc_mpc_T *inputs;
  ExtY_mvdc_mpc_T *outputs;
  DW_mvdc_mpc_T *dwork;

  //
  //  DataMapInfo:
  //  The following substructure contains information regarding
  //  structures generated in the model's C API.

  struct DataMapInfo_T {
    rtwCAPI_ModelMappingInfo mmi;
    void* dataAddress[168];
    int32_T* vardimsAddress[168];
    RTWLoggingFcnPtr loggingPtrs[168];
  };

  DataMapInfo_T DataMapInfo;
  const char_T* getErrorStatus() const;
  void setErrorStatus(const char_T* const volatile aErrorStatus);
  RT_MODEL_mvdc_mpc_T::DataMapInfo_T getDataMapInfo() const;
  void setDataMapInfo(RT_MODEL_mvdc_mpc_T::DataMapInfo_T aDataMapInfo);
};

// Block parameters (default storage)
#ifdef __cplusplus

extern "C"
{

#endif

  extern P_mvdc_mpc_T mvdc_mpc_P;

#ifdef __cplusplus

}

#endif

// External data declarations for dependent source files
#ifdef __cplusplus

extern "C"
{

#endif

  extern const char_T *RT_MEMORY_ALLOCATION_ERROR;

#ifdef __cplusplus

}

#endif

extern P_mvdc_mpc_T mvdc_mpc_P;        // parameters

// Constant parameters (default storage)
extern const ConstP_mvdc_mpc_T mvdc_mpc_ConstP;

#ifdef __cplusplus

extern "C"
{

#endif

  // Model entry point functions
  extern RT_MODEL_mvdc_mpc_T *mvdc_mpc(void);
  extern void mvdc_mpc_initialize(RT_MODEL_mvdc_mpc_T *const mvdc_mpc_M);
  extern void mvdc_mpc_step(RT_MODEL_mvdc_mpc_T *const mvdc_mpc_M);
  extern void mvdc_mpc_terminate(RT_MODEL_mvdc_mpc_T * mvdc_mpc_M);

#ifdef __cplusplus

}

#endif

// Function to get C API Model Mapping Static Info
extern const rtwCAPI_ModelMappingStaticInfo*
  mvdc_mpc_GetCAPIStaticMap(void);

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S22>/Data Type Duplicate' : Unused code path elimination
//  Block '<S23>/Data Type Duplicate' : Unused code path elimination
//  Block '<S24>/Data Type Duplicate' : Unused code path elimination
//  Block '<S20>/Data Type Duplicate' : Unused code path elimination
//  Block '<S21>/Data Type Duplicate' : Unused code path elimination
//  Block '<S11>/Constant1' : Unused code path elimination
//  Block '<S11>/Constant3' : Unused code path elimination
//  Block '<S11>/Divide3' : Unused code path elimination
//  Block '<S11>/Dot Product' : Unused code path elimination
//  Block '<S11>/Dot Product1' : Unused code path elimination
//  Block '<S11>/Gain9' : Unused code path elimination
//  Block '<S11>/Multiply' : Unused code path elimination
//  Block '<S11>/Sum' : Unused code path elimination
//  Block '<S11>/Sum3' : Unused code path elimination
//  Block '<S11>/Sum4' : Unused code path elimination
//  Block '<S11>/Switch' : Unused code path elimination
//  Block '<S28>/Data Type Duplicate' : Unused code path elimination
//  Block '<S11>/Unit Delay' : Unused code path elimination
//  Block '<S33>/Data Type Duplicate' : Unused code path elimination
//  Block '<S34>/Constant' : Unused code path elimination
//  Block '<S34>/Constant1' : Unused code path elimination
//  Block '<S34>/Constant2' : Unused code path elimination
//  Block '<S34>/Constant3' : Unused code path elimination
//  Block '<S39>/FixPt Data Type Propagation' : Unused code path elimination
//  Block '<S40>/FixPt Data Type Duplicate' : Unused code path elimination
//  Block '<S41>/FixPt Data Type Duplicate1' : Unused code path elimination
//  Block '<S59>/Data Type Duplicate' : Unused code path elimination
//  Block '<S60>/Data Type Duplicate' : Unused code path elimination
//  Block '<S34>/Data Type Conversion' : Eliminate redundant data type conversion
//  Block '<S34>/Signal Conversion1' : Eliminate redundant signal conversion block
//  Block '<S34>/Signal Conversion16' : Eliminate redundant signal conversion block
//  Block '<S34>/Signal Conversion17' : Eliminate redundant signal conversion block
//  Block '<S34>/Signal Conversion18' : Eliminate redundant signal conversion block
//  Block '<S34>/Signal Conversion19' : Eliminate redundant signal conversion block
//  Block '<S34>/Signal Conversion2' : Eliminate redundant signal conversion block
//  Block '<S34>/Signal Conversion20' : Eliminate redundant signal conversion block
//  Block '<S34>/Signal Conversion3' : Eliminate redundant signal conversion block
//  Block '<S34>/Signal Conversion36' : Eliminate redundant signal conversion block
//  Block '<S34>/Signal Conversion37' : Eliminate redundant signal conversion block
//  Block '<S34>/Signal Conversion38' : Eliminate redundant signal conversion block
//  Block '<S34>/Signal Conversion39' : Eliminate redundant signal conversion block
//  Block '<S34>/Signal Conversion40' : Eliminate redundant signal conversion block
//  Block '<S34>/Signal Conversion41' : Eliminate redundant signal conversion block
//  Block '<S34>/Signal Conversion42' : Eliminate redundant signal conversion block
//  Block '<S34>/Signal Conversion43' : Eliminate redundant signal conversion block
//  Block '<S34>/Signal Conversion44' : Eliminate redundant signal conversion block
//  Block '<S34>/Signal Conversion45' : Eliminate redundant signal conversion block
//  Block '<S34>/Signal Conversion48' : Eliminate redundant signal conversion block
//  Block '<S34>/Signal Conversion49' : Eliminate redundant signal conversion block
//  Block '<S5>/Signal Copy2' : Eliminate redundant signal conversion block
//  Block '<S5>/Signal Copy3' : Eliminate redundant signal conversion block
//  Block '<S5>/Signal Copy4' : Eliminate redundant signal conversion block
//  Block '<S5>/Signal Copy5' : Eliminate redundant signal conversion block


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
//  '<Root>' : 'mvdc_mpc'
//  '<S1>'   : 'mvdc_mpc/AccelerationControllers'
//  '<S2>'   : 'mvdc_mpc/Diagnosis'
//  '<S3>'   : 'mvdc_mpc/MPC'
//  '<S4>'   : 'mvdc_mpc/UpdateDynamicConstraints'
//  '<S5>'   : 'mvdc_mpc/debug'
//  '<S6>'   : 'mvdc_mpc/AccelerationControllers/LateralController'
//  '<S7>'   : 'mvdc_mpc/AccelerationControllers/LongitudinalController'
//  '<S8>'   : 'mvdc_mpc/AccelerationControllers/LateralController/Compare To Constant'
//  '<S9>'   : 'mvdc_mpc/AccelerationControllers/LateralController/Feedback'
//  '<S10>'  : 'mvdc_mpc/AccelerationControllers/LateralController/Feedforward_InverseModel'
//  '<S11>'  : 'mvdc_mpc/AccelerationControllers/LateralController/Feedforward_Learning'
//  '<S12>'  : 'mvdc_mpc/AccelerationControllers/LateralController/Feedback/Compare To Constant1'
//  '<S13>'  : 'mvdc_mpc/AccelerationControllers/LateralController/Feedback/Compare To Constant2'
//  '<S14>'  : 'mvdc_mpc/AccelerationControllers/LateralController/Feedforward_InverseModel/Compare To Constant'
//  '<S15>'  : 'mvdc_mpc/AccelerationControllers/LateralController/Feedforward_InverseModel/Compare To Constant1'
//  '<S16>'  : 'mvdc_mpc/AccelerationControllers/LateralController/Feedforward_InverseModel/EstimateLatAccDerivative'
//  '<S17>'  : 'mvdc_mpc/AccelerationControllers/LateralController/Feedforward_InverseModel/Subsystem1'
//  '<S18>'  : 'mvdc_mpc/AccelerationControllers/LateralController/Feedforward_InverseModel/Subsystem2'
//  '<S19>'  : 'mvdc_mpc/AccelerationControllers/LateralController/Feedforward_InverseModel/Subsystem3'
//  '<S20>'  : 'mvdc_mpc/AccelerationControllers/LateralController/Feedforward_InverseModel/Transfer Fcn First Order'
//  '<S21>'  : 'mvdc_mpc/AccelerationControllers/LateralController/Feedforward_InverseModel/Transfer Fcn First Order1'
//  '<S22>'  : 'mvdc_mpc/AccelerationControllers/LateralController/Feedforward_InverseModel/Subsystem1/Transfer Fcn First Order1'
//  '<S23>'  : 'mvdc_mpc/AccelerationControllers/LateralController/Feedforward_InverseModel/Subsystem2/Transfer Fcn First Order1'
//  '<S24>'  : 'mvdc_mpc/AccelerationControllers/LateralController/Feedforward_InverseModel/Subsystem3/Transfer Fcn First Order1'
//  '<S25>'  : 'mvdc_mpc/AccelerationControllers/LateralController/Feedforward_Learning/Compare To Constant'
//  '<S26>'  : 'mvdc_mpc/AccelerationControllers/LateralController/Feedforward_Learning/Compare To Constant1'
//  '<S27>'  : 'mvdc_mpc/AccelerationControllers/LateralController/Feedforward_Learning/Subsystem'
//  '<S28>'  : 'mvdc_mpc/AccelerationControllers/LateralController/Feedforward_Learning/Transfer Fcn First Order'
//  '<S29>'  : 'mvdc_mpc/AccelerationControllers/LateralController/Feedforward_Learning/Subsystem/MATLAB Function'
//  '<S30>'  : 'mvdc_mpc/AccelerationControllers/LongitudinalController/Feedforward_Aero'
//  '<S31>'  : 'mvdc_mpc/AccelerationControllers/LongitudinalController/Feedforward_Learning'
//  '<S32>'  : 'mvdc_mpc/AccelerationControllers/LongitudinalController/Feedforward_LongAcc'
//  '<S33>'  : 'mvdc_mpc/AccelerationControllers/LongitudinalController/Transfer Fcn First Order'
//  '<S34>'  : 'mvdc_mpc/MPC/Debug'
//  '<S35>'  : 'mvdc_mpc/MPC/DelayCompensation'
//  '<S36>'  : 'mvdc_mpc/MPC/DisturbanceEstimation'
//  '<S37>'  : 'mvdc_mpc/MPC/TubeMPC'
//  '<S38>'  : 'mvdc_mpc/MPC/VehicleReadyForTMPC'
//  '<S39>'  : 'mvdc_mpc/MPC/Debug/Counter Free-Running'
//  '<S40>'  : 'mvdc_mpc/MPC/Debug/Counter Free-Running/Increment Real World'
//  '<S41>'  : 'mvdc_mpc/MPC/Debug/Counter Free-Running/Wrap To Zero'
//  '<S42>'  : 'mvdc_mpc/MPC/DisturbanceEstimation/UncertaintyModelLearning_ax'
//  '<S43>'  : 'mvdc_mpc/MPC/DisturbanceEstimation/UncertaintyModelLearning_ay'
//  '<S44>'  : 'mvdc_mpc/MPC/TubeMPC/Compare To Constant1'
//  '<S45>'  : 'mvdc_mpc/MPC/TubeMPC/Compare To Constant2'
//  '<S46>'  : 'mvdc_mpc/MPC/TubeMPC/Compare To Constant3'
//  '<S47>'  : 'mvdc_mpc/MPC/TubeMPC/Compare To Constant4'
//  '<S48>'  : 'mvdc_mpc/MPC/TubeMPC/Compare To Constant5'
//  '<S49>'  : 'mvdc_mpc/MPC/TubeMPC/Compare To Constant7'
//  '<S50>'  : 'mvdc_mpc/MPC/TubeMPC/Detect Increase'
//  '<S51>'  : 'mvdc_mpc/MPC/TubeMPC/Subsystem'
//  '<S52>'  : 'mvdc_mpc/MPC/TubeMPC/Subsystem1'
//  '<S53>'  : 'mvdc_mpc/MPC/TubeMPC/prepareLinearization'
//  '<S54>'  : 'mvdc_mpc/MPC/TubeMPC/prepareOptimizationProblem'
//  '<S55>'  : 'mvdc_mpc/MPC/TubeMPC/transformMPCResult'
//  '<S56>'  : 'mvdc_mpc/MPC/TubeMPC/Subsystem/Compare To Constant'
//  '<S57>'  : 'mvdc_mpc/MPC/TubeMPC/Subsystem/Compare To Constant1'
//  '<S58>'  : 'mvdc_mpc/MPC/TubeMPC/Subsystem/Compare To Constant2'
//  '<S59>'  : 'mvdc_mpc/MPC/TubeMPC/Subsystem1/Discrete Derivative'
//  '<S60>'  : 'mvdc_mpc/MPC/TubeMPC/Subsystem1/Transfer Fcn First Order'
//  '<S61>'  : 'mvdc_mpc/MPC/VehicleReadyForTMPC/Compare To Constant1'
//  '<S62>'  : 'mvdc_mpc/MPC/VehicleReadyForTMPC/Compare To Constant2'

#endif                                 // mvdc_mpc_h_

//
// File trailer for generated code.
//
// [EOF]
//
