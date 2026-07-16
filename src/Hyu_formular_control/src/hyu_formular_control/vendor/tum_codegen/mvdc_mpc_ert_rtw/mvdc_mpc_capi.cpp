//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: mvdc_mpc_capi.cpp
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
#include <stddef.h>
#include "rtw_capi.h"
#ifdef HOST_CAPI_BUILD
#include "mvdc_mpc_capi_host.h"
#define sizeof(...)                    ((size_t)(0xFFFF))
#undef rt_offsetof
#define rt_offsetof(s,el)              ((uint16_T)(0xFFFF))
#define TARGET_CONST
#define TARGET_STRING(s)               (s)
#else                                  // HOST_CAPI_BUILD
#include "builtin_typeid_types.h"
#include "mvdc_mpc.h"
#include "mvdc_mpc_capi.h"
#include "mvdc_mpc_private.h"
#ifdef LIGHT_WEIGHT_CAPI
#define TARGET_CONST
#define TARGET_STRING(s)               ((NULL))
#else
#define TARGET_CONST                   const
#define TARGET_STRING(s)               (s)
#endif
#endif                                 // HOST_CAPI_BUILD

static rtwCAPI_BlockParameters rtBlockParameters[] = {
  // addrMapIndex, blockPath,
  //  paramName, dataTypeIndex, dimIndex, fixPtIdx

  { 0, TARGET_STRING("mvdc_mpc/Memory"),
    TARGET_STRING("InitialCondition"), 0, 0, 0 },

  { 1, TARGET_STRING("mvdc_mpc/Diagnosis/Constant"),
    TARGET_STRING("Value"), 1, 0, 0 },

  { 2, TARGET_STRING("mvdc_mpc/Diagnosis/Constant1"),
    TARGET_STRING("Value"), 1, 0, 0 },

  { 3, TARGET_STRING("mvdc_mpc/MPC/Memory"),
    TARGET_STRING("InitialCondition"), 0, 0, 0 },

  { 4, TARGET_STRING("mvdc_mpc/MPC/Memory1"),
    TARGET_STRING("InitialCondition"), 0, 0, 0 },

  { 5, TARGET_STRING("mvdc_mpc/UpdateDynamicConstraints/Gain1"),
    TARGET_STRING("Gain"), 0, 0, 0 },

  { 6, TARGET_STRING("mvdc_mpc/AccelerationControllers/LateralController/Compare To Constant"),
    TARGET_STRING("const"), 0, 0, 0 },

  { 7, TARGET_STRING("mvdc_mpc/AccelerationControllers/LateralController/Constant"),
    TARGET_STRING("Value"), 0, 0, 0 },

  { 8, TARGET_STRING("mvdc_mpc/AccelerationControllers/LateralController/Unit Delay1"),
    TARGET_STRING("InitialCondition"), 0, 0, 0 },

  { 9, TARGET_STRING("mvdc_mpc/AccelerationControllers/LongitudinalController/Transfer Fcn First Order"),
    TARGET_STRING("ICPrevOutput"), 0, 0, 0 },

  { 10, TARGET_STRING("mvdc_mpc/AccelerationControllers/LongitudinalController/Constant"),
    TARGET_STRING("Value"), 0, 0, 0 },

  { 11, TARGET_STRING("mvdc_mpc/AccelerationControllers/LongitudinalController/Saturation1"),
    TARGET_STRING("UpperLimit"), 0, 0, 0 },

  { 12, TARGET_STRING("mvdc_mpc/AccelerationControllers/LongitudinalController/Saturation1"),
    TARGET_STRING("LowerLimit"), 0, 0, 0 },

  { 13, TARGET_STRING("mvdc_mpc/AccelerationControllers/LongitudinalController/Unit Delay1"),
    TARGET_STRING("InitialCondition"), 0, 0, 0 },

  { 14, TARGET_STRING("mvdc_mpc/MPC/TubeMPC/OptimizerOK"),
    TARGET_STRING("InitialOutput"), 2, 0, 0 },

  { 15, TARGET_STRING("mvdc_mpc/MPC/TubeMPC/exitflags"),
    TARGET_STRING("InitialOutput"), 3, 0, 0 },

  { 16, TARGET_STRING("mvdc_mpc/MPC/TubeMPC/solver_details"),
    TARGET_STRING("InitialOutput"), 4, 0, 0 },

  { 17, TARGET_STRING("mvdc_mpc/MPC/TubeMPC/ResampledTargetTrajectory"),
    TARGET_STRING("InitialOutput"), 6, 0, 0 },

  { 18, TARGET_STRING("mvdc_mpc/MPC/TubeMPC/MiscDebug"),
    TARGET_STRING("InitialOutput"), 7, 0, 0 },

  { 19, TARGET_STRING("mvdc_mpc/MPC/TubeMPC/Prediction"),
    TARGET_STRING("InitialOutput"), 8, 0, 0 },

  { 20, TARGET_STRING("mvdc_mpc/MPC/TubeMPC/Linearization"),
    TARGET_STRING("InitialOutput"), 9, 0, 0 },

  { 21, TARGET_STRING("mvdc_mpc/MPC/TubeMPC/Compare To Constant1"),
    TARGET_STRING("const"), 0, 0, 0 },

  { 22, TARGET_STRING("mvdc_mpc/MPC/TubeMPC/Compare To Constant2"),
    TARGET_STRING("const"), 0, 0, 0 },

  { 23, TARGET_STRING("mvdc_mpc/MPC/TubeMPC/Compare To Constant3"),
    TARGET_STRING("const"), 0, 0, 0 },

  { 24, TARGET_STRING("mvdc_mpc/MPC/TubeMPC/Compare To Constant4"),
    TARGET_STRING("const"), 0, 0, 0 },

  { 25, TARGET_STRING("mvdc_mpc/MPC/TubeMPC/Compare To Constant5"),
    TARGET_STRING("const"), 0, 0, 0 },

  { 26, TARGET_STRING("mvdc_mpc/MPC/TubeMPC/Compare To Constant7"),
    TARGET_STRING("const"), 0, 0, 0 },

  { 27, TARGET_STRING("mvdc_mpc/MPC/TubeMPC/Detect Increase"),
    TARGET_STRING("vinit"), 0, 0, 0 },

  { 28, TARGET_STRING("mvdc_mpc/MPC/TubeMPC/Memory2"),
    TARGET_STRING("InitialCondition"), 0, 0, 0 },

  { 29, TARGET_STRING("mvdc_mpc/MPC/TubeMPC/Memory3"),
    TARGET_STRING("InitialCondition"), 0, 0, 0 },

  { 30, TARGET_STRING("mvdc_mpc/MPC/TubeMPC/Memory6"),
    TARGET_STRING("InitialCondition"), 0, 0, 0 },

  { 31, TARGET_STRING("mvdc_mpc/MPC/TubeMPC/Memory7"),
    TARGET_STRING("InitialCondition"), 0, 0, 0 },

  { 32, TARGET_STRING("mvdc_mpc/MPC/TubeMPC/S-Function_OSQP"),
    TARGET_STRING("P1"), 0, 7, 0 },

  { 33, TARGET_STRING("mvdc_mpc/MPC/TubeMPC/S-Function_OSQP"),
    TARGET_STRING("P2"), 0, 0, 0 },

  { 34, TARGET_STRING("mvdc_mpc/MPC/TubeMPC/S-Function_OSQP"),
    TARGET_STRING("P3"), 0, 0, 0 },

  { 35, TARGET_STRING("mvdc_mpc/MPC/TubeMPC/S-Function_OSQP"),
    TARGET_STRING("P4"), 0, 8, 0 },

  { 36, TARGET_STRING("mvdc_mpc/MPC/TubeMPC/S-Function_OSQP"),
    TARGET_STRING("P5"), 0, 8, 0 },

  { 37, TARGET_STRING("mvdc_mpc/MPC/TubeMPC/S-Function_OSQP"),
    TARGET_STRING("P6"), 0, 9, 0 },

  { 38, TARGET_STRING("mvdc_mpc/MPC/TubeMPC/S-Function_OSQP"),
    TARGET_STRING("P7"), 0, 10, 0 },

  { 39, TARGET_STRING("mvdc_mpc/MPC/TubeMPC/S-Function_OSQP"),
    TARGET_STRING("P8"), 0, 10, 0 },

  { 40, TARGET_STRING("mvdc_mpc/MPC/TubeMPC/S-Function_OSQP"),
    TARGET_STRING("P9"), 0, 9, 0 },

  { 41, TARGET_STRING("mvdc_mpc/MPC/TubeMPC/S-Function_OSQP"),
    TARGET_STRING("P10"), 0, 11, 0 },

  { 42, TARGET_STRING("mvdc_mpc/MPC/TubeMPC/S-Function_OSQP"),
    TARGET_STRING("P11"), 0, 11, 0 },

  { 43, TARGET_STRING("mvdc_mpc/MPC/TubeMPC/S-Function_OSQP"),
    TARGET_STRING("P12"), 0, 0, 0 },

  { 44, TARGET_STRING("mvdc_mpc/MPC/VehicleReadyForTMPC/Compare To Constant1"),
    TARGET_STRING("const"), 1, 0, 0 },

  { 45, TARGET_STRING("mvdc_mpc/MPC/VehicleReadyForTMPC/Compare To Constant2"),
    TARGET_STRING("const"), 10, 0, 0 },

  { 46, TARGET_STRING("mvdc_mpc/AccelerationControllers/LateralController/Feedback/Constant"),
    TARGET_STRING("Value"), 0, 0, 0 },

  { 47, TARGET_STRING("mvdc_mpc/AccelerationControllers/LateralController/Feedback/Constant10"),
    TARGET_STRING("Value"), 0, 0, 0 },

  { 48, TARGET_STRING("mvdc_mpc/AccelerationControllers/LateralController/Feedback/Constant3"),
    TARGET_STRING("Value"), 0, 0, 0 },

  { 49, TARGET_STRING("mvdc_mpc/AccelerationControllers/LateralController/Feedback/Constant4"),
    TARGET_STRING("Value"), 0, 0, 0 },

  { 50, TARGET_STRING("mvdc_mpc/AccelerationControllers/LateralController/Feedback/Constant5"),
    TARGET_STRING("Value"), 0, 0, 0 },

  { 51, TARGET_STRING("mvdc_mpc/AccelerationControllers/LateralController/Feedback/Constant6"),
    TARGET_STRING("Value"), 0, 0, 0 },

  { 52, TARGET_STRING("mvdc_mpc/AccelerationControllers/LateralController/Feedback/Constant7"),
    TARGET_STRING("Value"), 0, 0, 0 },

  { 53, TARGET_STRING("mvdc_mpc/AccelerationControllers/LateralController/Feedback/Constant8"),
    TARGET_STRING("Value"), 0, 0, 0 },

  { 54, TARGET_STRING("mvdc_mpc/AccelerationControllers/LateralController/Feedback/Constant9"),
    TARGET_STRING("Value"), 0, 0, 0 },

  { 55, TARGET_STRING("mvdc_mpc/AccelerationControllers/LateralController/Feedback/Discrete-Time Integrator"),
    TARGET_STRING("gainval"), 0, 0, 0 },

  { 56, TARGET_STRING("mvdc_mpc/AccelerationControllers/LateralController/Feedback/Discrete-Time Integrator"),
    TARGET_STRING("InitialCondition"), 0, 0, 0 },

  { 57, TARGET_STRING("mvdc_mpc/AccelerationControllers/LateralController/Feedback/Gain"),
    TARGET_STRING("Gain"), 0, 0, 0 },

  { 58, TARGET_STRING("mvdc_mpc/AccelerationControllers/LateralController/Feedforward_InverseModel/Transfer Fcn First Order"),
    TARGET_STRING("ICPrevOutput"), 0, 0, 0 },

  { 59, TARGET_STRING("mvdc_mpc/AccelerationControllers/LateralController/Feedforward_InverseModel/Transfer Fcn First Order1"),
    TARGET_STRING("ICPrevOutput"), 0, 0, 0 },

  { 60, TARGET_STRING("mvdc_mpc/AccelerationControllers/LateralController/Feedforward_InverseModel/Constant4"),
    TARGET_STRING("Value"), 0, 0, 0 },

  { 61, TARGET_STRING("mvdc_mpc/AccelerationControllers/LateralController/Feedforward_Learning/Compare To Constant1"),
    TARGET_STRING("const"), 0, 0, 0 },

  { 62, TARGET_STRING("mvdc_mpc/AccelerationControllers/LateralController/Feedforward_Learning/Transfer Fcn First Order"),
    TARGET_STRING("ICPrevOutput"), 0, 0, 0 },

  { 63, TARGET_STRING("mvdc_mpc/AccelerationControllers/LongitudinalController/Feedforward_Aero/Gain4"),
    TARGET_STRING("Gain"), 0, 0, 0 },

  { 64, TARGET_STRING("mvdc_mpc/AccelerationControllers/LongitudinalController/Feedforward_Learning/Constant"),
    TARGET_STRING("Value"), 0, 0, 0 },

  { 65, TARGET_STRING("mvdc_mpc/AccelerationControllers/LongitudinalController/Feedforward_Learning/Constant1"),
    TARGET_STRING("Value"), 0, 0, 0 },

  { 66, TARGET_STRING("mvdc_mpc/AccelerationControllers/LongitudinalController/Feedforward_Learning/Constant2"),
    TARGET_STRING("Value"), 0, 0, 0 },

  { 67, TARGET_STRING("mvdc_mpc/AccelerationControllers/LongitudinalController/Feedforward_Learning/Constant4"),
    TARGET_STRING("Value"), 0, 0, 0 },

  { 68, TARGET_STRING("mvdc_mpc/AccelerationControllers/LongitudinalController/Feedforward_Learning/Gain10"),
    TARGET_STRING("Gain"), 0, 0, 0 },

  { 69, TARGET_STRING("mvdc_mpc/AccelerationControllers/LongitudinalController/Feedforward_Learning/Saturation1"),
    TARGET_STRING("UpperLimit"), 0, 0, 0 },

  { 70, TARGET_STRING("mvdc_mpc/AccelerationControllers/LongitudinalController/Feedforward_Learning/Saturation1"),
    TARGET_STRING("LowerLimit"), 0, 0, 0 },

  { 71, TARGET_STRING("mvdc_mpc/AccelerationControllers/LongitudinalController/Feedforward_Learning/Unit Delay"),
    TARGET_STRING("InitialCondition"), 0, 12, 0 },

  { 72, TARGET_STRING("mvdc_mpc/MPC/Debug/Counter Free-Running/Wrap To Zero"),
    TARGET_STRING("Threshold"), 10, 0, 0 },

  { 73, TARGET_STRING("mvdc_mpc/MPC/Debug/Counter Free-Running/Output"),
    TARGET_STRING("InitialCondition"), 10, 0, 0 },

  { 74, TARGET_STRING("mvdc_mpc/MPC/DisturbanceEstimation/UncertaintyModelLearning_ax/ax_LearnedBound_mps2"),
    TARGET_STRING("InitialOutput"), 0, 0, 0 },

  { 75, TARGET_STRING("mvdc_mpc/MPC/DisturbanceEstimation/UncertaintyModelLearning_ax/ax_DataCoverage_perc"),
    TARGET_STRING("InitialOutput"), 0, 0, 0 },

  { 76, TARGET_STRING("mvdc_mpc/MPC/DisturbanceEstimation/UncertaintyModelLearning_ax/Constant"),
    TARGET_STRING("Value"), 0, 0, 0 },

  { 77, TARGET_STRING("mvdc_mpc/MPC/DisturbanceEstimation/UncertaintyModelLearning_ax/Constant4"),
    TARGET_STRING("Value"), 0, 0, 0 },

  { 78, TARGET_STRING("mvdc_mpc/MPC/DisturbanceEstimation/UncertaintyModelLearning_ax/Gain3"),
    TARGET_STRING("Gain"), 0, 0, 0 },

  { 79, TARGET_STRING("mvdc_mpc/MPC/DisturbanceEstimation/UncertaintyModelLearning_ax/Saturation"),
    TARGET_STRING("UpperLimit"), 0, 0, 0 },

  { 80, TARGET_STRING("mvdc_mpc/MPC/DisturbanceEstimation/UncertaintyModelLearning_ax/Saturation"),
    TARGET_STRING("LowerLimit"), 0, 0, 0 },

  { 81, TARGET_STRING("mvdc_mpc/MPC/DisturbanceEstimation/UncertaintyModelLearning_ax/Saturation1"),
    TARGET_STRING("UpperLimit"), 0, 0, 0 },

  { 82, TARGET_STRING("mvdc_mpc/MPC/DisturbanceEstimation/UncertaintyModelLearning_ax/Saturation1"),
    TARGET_STRING("LowerLimit"), 0, 0, 0 },

  { 83, TARGET_STRING("mvdc_mpc/MPC/DisturbanceEstimation/UncertaintyModelLearning_ax/Tapped Delay"),
    TARGET_STRING("vinit"), 0, 0, 0 },

  { 84, TARGET_STRING("mvdc_mpc/MPC/DisturbanceEstimation/UncertaintyModelLearning_ax/Delay"),
    TARGET_STRING("InitialCondition"), 2, 0, 0 },

  { 85, TARGET_STRING("mvdc_mpc/MPC/DisturbanceEstimation/UncertaintyModelLearning_ax/Unit Delay"),
    TARGET_STRING("InitialCondition"), 0, 0, 0 },

  { 86, TARGET_STRING("mvdc_mpc/MPC/DisturbanceEstimation/UncertaintyModelLearning_ay/ay_LearnedBound_mps2"),
    TARGET_STRING("InitialOutput"), 0, 0, 0 },

  { 87, TARGET_STRING("mvdc_mpc/MPC/DisturbanceEstimation/UncertaintyModelLearning_ay/ay_DataCoverage_perc"),
    TARGET_STRING("InitialOutput"), 0, 0, 0 },

  { 88, TARGET_STRING("mvdc_mpc/MPC/DisturbanceEstimation/UncertaintyModelLearning_ay/Constant"),
    TARGET_STRING("Value"), 0, 0, 0 },

  { 89, TARGET_STRING("mvdc_mpc/MPC/DisturbanceEstimation/UncertaintyModelLearning_ay/Constant4"),
    TARGET_STRING("Value"), 0, 0, 0 },

  { 90, TARGET_STRING("mvdc_mpc/MPC/DisturbanceEstimation/UncertaintyModelLearning_ay/Gain3"),
    TARGET_STRING("Gain"), 0, 0, 0 },

  { 91, TARGET_STRING("mvdc_mpc/MPC/DisturbanceEstimation/UncertaintyModelLearning_ay/Saturation"),
    TARGET_STRING("UpperLimit"), 0, 0, 0 },

  { 92, TARGET_STRING("mvdc_mpc/MPC/DisturbanceEstimation/UncertaintyModelLearning_ay/Saturation"),
    TARGET_STRING("LowerLimit"), 0, 0, 0 },

  { 93, TARGET_STRING("mvdc_mpc/MPC/DisturbanceEstimation/UncertaintyModelLearning_ay/Saturation1"),
    TARGET_STRING("UpperLimit"), 0, 0, 0 },

  { 94, TARGET_STRING("mvdc_mpc/MPC/DisturbanceEstimation/UncertaintyModelLearning_ay/Saturation1"),
    TARGET_STRING("LowerLimit"), 0, 0, 0 },

  { 95, TARGET_STRING("mvdc_mpc/MPC/DisturbanceEstimation/UncertaintyModelLearning_ay/Tapped Delay"),
    TARGET_STRING("vinit"), 0, 0, 0 },

  { 96, TARGET_STRING("mvdc_mpc/MPC/DisturbanceEstimation/UncertaintyModelLearning_ay/Delay"),
    TARGET_STRING("InitialCondition"), 2, 0, 0 },

  { 97, TARGET_STRING("mvdc_mpc/MPC/DisturbanceEstimation/UncertaintyModelLearning_ay/Unit Delay"),
    TARGET_STRING("InitialCondition"), 0, 0, 0 },

  { 98, TARGET_STRING("mvdc_mpc/MPC/TubeMPC/Subsystem/Compare To Constant"),
    TARGET_STRING("const"), 0, 0, 0 },

  { 99, TARGET_STRING("mvdc_mpc/MPC/TubeMPC/Subsystem/Compare To Constant1"),
    TARGET_STRING("const"), 0, 0, 0 },

  { 100, TARGET_STRING("mvdc_mpc/MPC/TubeMPC/Subsystem/Compare To Constant2"),
    TARGET_STRING("const"), 0, 0, 0 },

  { 101, TARGET_STRING("mvdc_mpc/MPC/TubeMPC/Subsystem1/Discrete Derivative"),
    TARGET_STRING("ICPrevScaledInput"), 0, 0, 0 },

  { 102, TARGET_STRING("mvdc_mpc/MPC/TubeMPC/Subsystem1/Transfer Fcn First Order"),
    TARGET_STRING("ICPrevOutput"), 0, 0, 0 },

  { 103, TARGET_STRING("mvdc_mpc/AccelerationControllers/LateralController/Feedforward_InverseModel/Subsystem1/Transfer Fcn First Order1"),
    TARGET_STRING("ICPrevOutput"), 0, 0, 0 },

  { 104, TARGET_STRING("mvdc_mpc/AccelerationControllers/LateralController/Feedforward_InverseModel/Subsystem1/Unit Delay"),
    TARGET_STRING("InitialCondition"), 0, 0, 0 },

  { 105, TARGET_STRING("mvdc_mpc/AccelerationControllers/LateralController/Feedforward_InverseModel/Subsystem2/Transfer Fcn First Order1"),
    TARGET_STRING("ICPrevOutput"), 0, 0, 0 },

  { 106, TARGET_STRING("mvdc_mpc/AccelerationControllers/LateralController/Feedforward_InverseModel/Subsystem2/Unit Delay"),
    TARGET_STRING("InitialCondition"), 0, 0, 0 },

  { 107, TARGET_STRING("mvdc_mpc/AccelerationControllers/LateralController/Feedforward_InverseModel/Subsystem3/Transfer Fcn First Order1"),
    TARGET_STRING("ICPrevOutput"), 0, 0, 0 },

  { 108, TARGET_STRING("mvdc_mpc/AccelerationControllers/LateralController/Feedforward_InverseModel/Subsystem3/Unit Delay"),
    TARGET_STRING("InitialCondition"), 0, 0, 0 },

  { 109, TARGET_STRING("mvdc_mpc/AccelerationControllers/LateralController/Feedforward_Learning/Subsystem/P_upd"),
    TARGET_STRING("InitialOutput"), 0, 13, 0 },

  { 110, TARGET_STRING("mvdc_mpc/AccelerationControllers/LateralController/Feedforward_Learning/Subsystem/mean_debug"),
    TARGET_STRING("InitialOutput"), 0, 0, 0 },

  { 111, TARGET_STRING("mvdc_mpc/AccelerationControllers/LateralController/Feedforward_Learning/Subsystem/cov_debug"),
    TARGET_STRING("InitialOutput"), 0, 0, 0 },

  { 112, TARGET_STRING("mvdc_mpc/AccelerationControllers/LateralController/Feedforward_Learning/Subsystem/Unit Delay1"),
    TARGET_STRING("InitialCondition"), 0, 13, 0 },

  { 113, TARGET_STRING("mvdc_mpc/MPC/Debug/Counter Free-Running/Increment Real World/FixPt Constant"),
    TARGET_STRING("Value"), 10, 0, 0 },

  { 114, TARGET_STRING("mvdc_mpc/MPC/Debug/Counter Free-Running/Wrap To Zero/Constant"),
    TARGET_STRING("Value"), 10, 0, 0 },

  { 115, TARGET_STRING("mvdc_mpc/MPC/TubeMPC/Subsystem1/Discrete Derivative/TSamp"),
    TARGET_STRING("WtEt"), 0, 0, 0 },

  {
    0, (NULL), (NULL), 0, 0, 0
  }
};

// Tunable variable parameters
static rtwCAPI_ModelParameters rtModelParameters[] = {
  // addrMapIndex, varName, dataTypeIndex, dimIndex, fixPtIndex
  { 116, TARGET_STRING("acc_control_learning"), 11, 0, 0 },

  { 117, TARGET_STRING("A_VehicleReference_m2"), 0, 0, 0 },

  { 118, TARGET_STRING("P_VDC_ControlMargin_ax_mps2"), 0, 0, 0 },

  { 119, TARGET_STRING("P_VDC_ControlMargin_ay_mps2"), 0, 0, 0 },

  { 120, TARGET_STRING("P_VDC_Dist_ax_perc"), 0, 0, 0 },

  { 121, TARGET_STRING("P_VDC_Dist_ay_perc"), 0, 0, 0 },

  { 122, TARGET_STRING("P_VDC_IncreaseUncertaintyPerStep_perc"), 0, 0, 0 },

  { 123, TARGET_STRING("P_VDC_LatAccKi"), 0, 0, 0 },

  { 124, TARGET_STRING("P_VDC_LatAccKi_Lim_perc"), 0, 0, 0 },

  { 125, TARGET_STRING("P_VDC_LatAccKp_Delta"), 0, 0, 0 },

  { 126, TARGET_STRING("P_VDC_LatAccKp_ay"), 0, 0, 0 },

  { 127, TARGET_STRING("P_VDC_LatAcc_BetaGain"), 0, 0, 0 },

  { 128, TARGET_STRING("P_VDC_LatAcc_FF_Ts"), 0, 0, 0 },

  { 129, TARGET_STRING("P_VDC_LatAcc_Steering_Ts"), 0, 0, 0 },

  { 130, TARGET_STRING("P_VDC_LatAcc_UndersteerComp_Ts"), 0, 0, 0 },

  { 131, TARGET_STRING("P_VDC_LatAcc_VehicleDynamics_Ts"), 0, 0, 0 },

  { 132, TARGET_STRING("P_VDC_LatErrorDer_Ts_s"), 0, 0, 0 },

  { 133, TARGET_STRING("P_VDC_LongAccKp"), 0, 0, 0 },

  { 134, TARGET_STRING("P_VDC_LongAcc_FB_Ts"), 0, 0, 0 },

  { 135, TARGET_STRING("P_VDC_LongAcc_FF_Ts"), 0, 0, 0 },

  { 136, TARGET_STRING("P_VDC_LongAcc_LimFb_N"), 0, 0, 0 },

  { 137, TARGET_STRING("P_VDC_MaxTightening"), 0, 0, 0 },

  { 138, TARGET_STRING("P_VDC_MinVelSlipCalc_mps"), 0, 0, 0 },

  { 139, TARGET_STRING("P_VDC_PositiveAxLimScale"), 0, 0, 0 },

  { 140, TARGET_STRING("P_VDC_RTISQP_alpha_old"), 0, 0, 0 },

  { 141, TARGET_STRING("P_VDC_RTISQP_alpha_target"), 0, 0, 0 },

  { 142, TARGET_STRING("P_VDC_StabilizerRearSideSlip_rad"), 0, 0, 0 },

  { 143, TARGET_STRING("P_VDC_SteeringOvalComp_rad"), 0, 0, 0 },

  { 144, TARGET_STRING("P_VDC_SteeringSteadyStateUndersteerComp_rad"), 0, 0, 0 },

  { 145, TARGET_STRING("P_VDC_TMPCMaxSolverIter"), 0, 0, 0 },

  { 146, TARGET_STRING("P_VDC_TerminalSetBrakeTime_s"), 0, 0, 0 },

  { 147, TARGET_STRING("P_VDC_TuneTerminalSet_mps"), 0, 0, 0 },

  { 148, TARGET_STRING("P_VDC_UncertaintyLearningGain"), 0, 0, 0 },

  { 149, TARGET_STRING("P_VDC_UncertaintyLearningSamples"), 0, 0, 0 },

  { 150, TARGET_STRING("P_VDC_UncertaintyTarget_perc"), 0, 0, 0 },

  { 151, TARGET_STRING("P_VDC_VirtualController"), 0, 18, 0 },

  { 152, TARGET_STRING("P_VDC_ayFFModelLearningRate"), 0, 0, 0 },

  { 153, TARGET_STRING("drag_coefficient"), 0, 0, 0 },

  { 154, TARGET_STRING("l_front_m"), 0, 0, 0 },

  { 155, TARGET_STRING("l_rear_m"), 0, 0, 0 },

  { 156, TARGET_STRING("roh_air"), 0, 0, 0 },

  { 157, TARGET_STRING("tS"), 0, 0, 0 },

  { 158, TARGET_STRING("tSSlow"), 0, 0, 0 },

  { 159, TARGET_STRING("vehiclemass_kg"), 0, 0, 0 },

  { 160, TARGET_STRING("P_VDC_EnableNumLatErrorDer_b"), 2, 0, 0 },

  { 161, TARGET_STRING("P_VDC_EnableTubeMPC"), 2, 0, 0 },

  { 162, TARGET_STRING("P_VDC_EnableUncertaintyLearning"), 2, 0, 0 },

  { 163, TARGET_STRING("P_VDC_EnableYawRateAccRepl"), 2, 0, 0 },

  { 164, TARGET_STRING("P_VDC_LatAcc_EnableForesightFF"), 2, 0, 0 },

  { 165, TARGET_STRING("P_VDC_LongAcc_EnableForesightFF"), 2, 0, 0 },

  { 166, TARGET_STRING("P_VDC_axFFModelLearningEnable"), 2, 0, 0 },

  { 167, TARGET_STRING("P_VDC_ayFFModelLearningEnable"), 2, 0, 0 },

  { 0, (NULL), 0, 0, 0 }
};

// Data Type Map - use dataTypeMapIndex to access this structure
static TARGET_CONST rtwCAPI_DataTypeMap rtDataTypeMap[] = {
  // cName, mwName, numElements, elemMapIndex, dataSize, slDataId, *
  //  isComplex, isPointer, enumStorageType
  { "double", "real_T", 0, 0, sizeof(real_T), (uint8_T)SS_DOUBLE, 0, 0, 0 },

  { "numeric", "TUMHealthStatus", 0, 0, sizeof(TUMHealthStatus), (uint8_T)
    SS_ENUM_TYPE, 0, 0, SS_UINT16 },

  { "unsigned char", "boolean_T", 0, 0, sizeof(boolean_T), (uint8_T)SS_BOOLEAN,
    0, 0, 0 },

  { "struct", "struct_8SAH98NOUMyLo7unQLNhYF", 7, 1, sizeof
    (struct_8SAH98NOUMyLo7unQLNhYF), (uint8_T)SS_STRUCT, 0, 0, 0 },

  { "struct", "struct_rAzch3iZmG57fB7gP9dRwC", 8, 8, sizeof
    (struct_rAzch3iZmG57fB7gP9dRwC), (uint8_T)SS_STRUCT, 0, 0, 0 },

  { "struct", "PathPos", 3, 16, sizeof(PathPos), (uint8_T)SS_STRUCT, 0, 0, 0 },

  { "struct", "struct_xwphvtKqjUwTm215naeozC", 17, 19, sizeof
    (struct_xwphvtKqjUwTm215naeozC), (uint8_T)SS_STRUCT, 0, 0, 0 },

  { "struct", "struct_cn5SO9QaZcduShzGK83fPD", 6, 36, sizeof
    (struct_cn5SO9QaZcduShzGK83fPD), (uint8_T)SS_STRUCT, 0, 0, 0 },

  { "struct", "struct_oi7PRKicCuvNrjdewxkrrG", 16, 42, sizeof
    (struct_oi7PRKicCuvNrjdewxkrrG), (uint8_T)SS_STRUCT, 0, 0, 0 },

  { "struct", "struct_XmA17SeQgjzTUvOoWvOe1", 7, 58, sizeof
    (struct_XmA17SeQgjzTUvOoWvOe1), (uint8_T)SS_STRUCT, 0, 0, 0 },

  { "unsigned int", "uint32_T", 0, 0, sizeof(uint32_T), (uint8_T)SS_UINT32, 0, 0,
    0 },

  { "struct", "struct_50gomenhaqprWvYYDUcxiH", 7, 65, sizeof
    (struct_50gomenhaqprWvYYDUcxiH), (uint8_T)SS_STRUCT, 0, 0, 0 }
};

#ifdef HOST_CAPI_BUILD
#undef sizeof
#endif

// Structure Element Map - use elemMapIndex to access this structure
static TARGET_CONST rtwCAPI_ElementMap rtElementMap[] = {
  // elementName, elementOffset, dataTypeIndex, dimIndex, fxpIndex
  { (NULL), 0, 0, 0, 0 },

  { "flag_s_request", rt_offsetof(struct_8SAH98NOUMyLo7unQLNhYF, flag_s_request),
    0, 1, 0 },

  { "flag_bound_Upd", rt_offsetof(struct_8SAH98NOUMyLo7unQLNhYF, flag_bound_Upd),
    0, 1, 0 },

  { "flag_A_Upd", rt_offsetof(struct_8SAH98NOUMyLo7unQLNhYF, flag_A_Upd), 0, 1,
    0 },

  { "flag_P_Upd", rt_offsetof(struct_8SAH98NOUMyLo7unQLNhYF, flag_P_Upd), 0, 1,
    0 },

  { "flag_q_Upd", rt_offsetof(struct_8SAH98NOUMyLo7unQLNhYF, flag_q_Upd), 0, 1,
    0 },

  { "flag_solve", rt_offsetof(struct_8SAH98NOUMyLo7unQLNhYF, flag_solve), 0, 1,
    0 },

  { "flag_setup", rt_offsetof(struct_8SAH98NOUMyLo7unQLNhYF, flag_setup), 0, 1,
    0 },

  { "solver_solveTime_s", rt_offsetof(struct_rAzch3iZmG57fB7gP9dRwC,
    solver_solveTime_s), 0, 1, 0 },

  { "solver_runTime_s", rt_offsetof(struct_rAzch3iZmG57fB7gP9dRwC,
    solver_runTime_s), 0, 1, 0 },

  { "solver_state", rt_offsetof(struct_rAzch3iZmG57fB7gP9dRwC, solver_state), 0,
    1, 0 },

  { "solver_iteration", rt_offsetof(struct_rAzch3iZmG57fB7gP9dRwC,
    solver_iteration), 0, 1, 0 },

  { "solver_updateTime_s", rt_offsetof(struct_rAzch3iZmG57fB7gP9dRwC,
    solver_updateTime_s), 0, 1, 0 },

  { "solver_sfunTime_s", rt_offsetof(struct_rAzch3iZmG57fB7gP9dRwC,
    solver_sfunTime_s), 0, 1, 0 },

  { "solver_pri_res", rt_offsetof(struct_rAzch3iZmG57fB7gP9dRwC, solver_pri_res),
    0, 1, 0 },

  { "solver_dua_res", rt_offsetof(struct_rAzch3iZmG57fB7gP9dRwC, solver_dua_res),
    0, 1, 0 },

  { "s_m", rt_offsetof(PathPos, s_m), 0, 0, 0 },

  { "d_m", rt_offsetof(PathPos, d_m), 0, 0, 0 },

  { "psi_rad", rt_offsetof(PathPos, psi_rad), 0, 0, 0 },

  { "PathPos", rt_offsetof(struct_xwphvtKqjUwTm215naeozC, PathPos), 5, 1, 0 },

  { "x_traj_m", rt_offsetof(struct_xwphvtKqjUwTm215naeozC, x_traj_m), 0, 2, 0 },

  { "y_traj_m", rt_offsetof(struct_xwphvtKqjUwTm215naeozC, y_traj_m), 0, 2, 0 },

  { "psi_traj_rad", rt_offsetof(struct_xwphvtKqjUwTm215naeozC, psi_traj_rad), 0,
    2, 0 },

  { "v_traj_mps", rt_offsetof(struct_xwphvtKqjUwTm215naeozC, v_traj_mps), 0, 2,
    0 },

  { "kappa_traj_radpm", rt_offsetof(struct_xwphvtKqjUwTm215naeozC,
    kappa_traj_radpm), 0, 2, 0 },

  { "ax_diff_traj_mps2m", rt_offsetof(struct_xwphvtKqjUwTm215naeozC,
    ax_diff_traj_mps2m), 0, 2, 0 },

  { "ax_traj_mps2", rt_offsetof(struct_xwphvtKqjUwTm215naeozC, ax_traj_mps2), 0,
    2, 0 },

  { "ay_traj_mps2", rt_offsetof(struct_xwphvtKqjUwTm215naeozC, ay_traj_mps2), 0,
    2, 0 },

  { "ax_lim_mps2", rt_offsetof(struct_xwphvtKqjUwTm215naeozC, ax_lim_mps2), 0, 2,
    0 },

  { "ay_lim_mps2", rt_offsetof(struct_xwphvtKqjUwTm215naeozC, ay_lim_mps2), 0, 2,
    0 },

  { "tube_r_m", rt_offsetof(struct_xwphvtKqjUwTm215naeozC, tube_r_m), 0, 2, 0 },

  { "tube_l_m", rt_offsetof(struct_xwphvtKqjUwTm215naeozC, tube_l_m), 0, 2, 0 },

  { "d_Target_m", rt_offsetof(struct_xwphvtKqjUwTm215naeozC, d_Target_m), 0, 2,
    0 },

  { "dot_d_Target_mps", rt_offsetof(struct_xwphvtKqjUwTm215naeozC,
    dot_d_Target_mps), 0, 2, 0 },

  { "d_lim_ub_m", rt_offsetof(struct_xwphvtKqjUwTm215naeozC, d_lim_ub_m), 0, 2,
    0 },

  { "d_lim_lb_m", rt_offsetof(struct_xwphvtKqjUwTm215naeozC, d_lim_lb_m), 0, 2,
    0 },

  { "dot_d_analytical_mps", rt_offsetof(struct_cn5SO9QaZcduShzGK83fPD,
    dot_d_analytical_mps), 0, 1, 0 },

  { "dot_d_numerical_mps", rt_offsetof(struct_cn5SO9QaZcduShzGK83fPD,
    dot_d_numerical_mps), 0, 1, 0 },

  { "be_u_abs", rt_offsetof(struct_cn5SO9QaZcduShzGK83fPD, be_u_abs), 0, 3, 0 },

  { "be_l_abs", rt_offsetof(struct_cn5SO9QaZcduShzGK83fPD, be_l_abs), 0, 3, 0 },

  { "s_current_m", rt_offsetof(struct_cn5SO9QaZcduShzGK83fPD, s_current_m), 0, 1,
    0 },

  { "error_state", rt_offsetof(struct_cn5SO9QaZcduShzGK83fPD, error_state), 0, 3,
    0 },

  { "u_opt_total", rt_offsetof(struct_oi7PRKicCuvNrjdewxkrrG, u_opt_total), 0, 4,
    0 },

  { "x_pred_m", rt_offsetof(struct_oi7PRKicCuvNrjdewxkrrG, x_pred_m), 0, 2, 0 },

  { "y_pred_m", rt_offsetof(struct_oi7PRKicCuvNrjdewxkrrG, y_pred_m), 0, 2, 0 },

  { "vx_pred_mps", rt_offsetof(struct_oi7PRKicCuvNrjdewxkrrG, vx_pred_mps), 0, 2,
    0 },

  { "x_pred_left_m", rt_offsetof(struct_oi7PRKicCuvNrjdewxkrrG, x_pred_left_m),
    0, 2, 0 },

  { "y_pred_left_m", rt_offsetof(struct_oi7PRKicCuvNrjdewxkrrG, y_pred_left_m),
    0, 2, 0 },

  { "x_pred_right_m", rt_offsetof(struct_oi7PRKicCuvNrjdewxkrrG, x_pred_right_m),
    0, 2, 0 },

  { "y_pred_right_m", rt_offsetof(struct_oi7PRKicCuvNrjdewxkrrG, y_pred_right_m),
    0, 2, 0 },

  { "d_pred_m", rt_offsetof(struct_oi7PRKicCuvNrjdewxkrrG, d_pred_m), 0, 2, 0 },

  { "dot_d_pred_mps", rt_offsetof(struct_oi7PRKicCuvNrjdewxkrrG, dot_d_pred_mps),
    0, 2, 0 },

  { "s_dot_pred_mps", rt_offsetof(struct_oi7PRKicCuvNrjdewxkrrG, s_dot_pred_mps),
    0, 2, 0 },

  { "ax_pred_mps2", rt_offsetof(struct_oi7PRKicCuvNrjdewxkrrG, ax_pred_mps2), 0,
    2, 0 },

  { "ax_tire_pred_mps2", rt_offsetof(struct_oi7PRKicCuvNrjdewxkrrG,
    ax_tire_pred_mps2), 0, 2, 0 },

  { "ay_pred_mps2", rt_offsetof(struct_oi7PRKicCuvNrjdewxkrrG, ay_pred_mps2), 0,
    2, 0 },

  { "TireUtilizationTarget", rt_offsetof(struct_oi7PRKicCuvNrjdewxkrrG,
    TireUtilizationTarget), 0, 1, 0 },

  { "cost_values", rt_offsetof(struct_oi7PRKicCuvNrjdewxkrrG, cost_values), 0, 5,
    0 },

  { "ax_dist_mps2", rt_offsetof(struct_XmA17SeQgjzTUvOoWvOe1, ax_dist_mps2), 0,
    2, 0 },

  { "ay_dist_mps2", rt_offsetof(struct_XmA17SeQgjzTUvOoWvOe1, ay_dist_mps2), 0,
    2, 0 },

  { "vx_lin_mps", rt_offsetof(struct_XmA17SeQgjzTUvOoWvOe1, vx_lin_mps), 0, 2, 0
  },

  { "s_dot_lin_mps", rt_offsetof(struct_XmA17SeQgjzTUvOoWvOe1, s_dot_lin_mps), 0,
    2, 0 },

  { "kappa_lin_radpm", rt_offsetof(struct_XmA17SeQgjzTUvOoWvOe1, kappa_lin_radpm),
    0, 2, 0 },

  { "UncertaintyTube", rt_offsetof(struct_XmA17SeQgjzTUvOoWvOe1, UncertaintyTube),
    0, 6, 0 },

  { "v_terminal_mps", rt_offsetof(struct_XmA17SeQgjzTUvOoWvOe1, v_terminal_mps),
    0, 1, 0 },

  { "spread_vx_mps", rt_offsetof(struct_50gomenhaqprWvYYDUcxiH, spread_vx_mps),
    0, 14, 0 },

  { "spread_ay_req_mps2", rt_offsetof(struct_50gomenhaqprWvYYDUcxiH,
    spread_ay_req_mps2), 0, 15, 0 },

  { "vx_width_mps", rt_offsetof(struct_50gomenhaqprWvYYDUcxiH, vx_width_mps), 0,
    1, 0 },

  { "ay_width_mps2", rt_offsetof(struct_50gomenhaqprWvYYDUcxiH, ay_width_mps2),
    0, 1, 0 },

  { "bf_vx_mps", rt_offsetof(struct_50gomenhaqprWvYYDUcxiH, bf_vx_mps), 0, 16, 0
  },

  { "bf_ay_req_mps2", rt_offsetof(struct_50gomenhaqprWvYYDUcxiH, bf_ay_req_mps2),
    0, 16, 0 },

  { "w0", rt_offsetof(struct_50gomenhaqprWvYYDUcxiH, w0), 0, 17, 0 }
};

// Dimension Map - use dimensionMapIndex to access elements of ths structure
static rtwCAPI_DimensionMap rtDimensionMap[] = {
  // dataOrientation, dimArrayIndex, numDims, vardimsIndex
  { rtwCAPI_SCALAR, 0, 2, 0 },

  { rtwCAPI_MATRIX_COL_MAJOR, 0, 2, 0 },

  { rtwCAPI_MATRIX_COL_MAJOR, 2, 2, 0 },

  { rtwCAPI_MATRIX_COL_MAJOR, 4, 2, 0 },

  { rtwCAPI_MATRIX_COL_MAJOR, 6, 2, 0 },

  { rtwCAPI_MATRIX_COL_MAJOR, 8, 2, 0 },

  { rtwCAPI_MATRIX_COL_MAJOR, 10, 2, 0 },

  { rtwCAPI_VECTOR, 12, 2, 0 },

  { rtwCAPI_VECTOR, 14, 2, 0 },

  { rtwCAPI_VECTOR, 16, 2, 0 },

  { rtwCAPI_VECTOR, 18, 2, 0 },

  { rtwCAPI_VECTOR, 20, 2, 0 },

  { rtwCAPI_VECTOR, 22, 2, 0 },

  { rtwCAPI_MATRIX_COL_MAJOR, 24, 2, 0 },

  { rtwCAPI_MATRIX_COL_MAJOR, 26, 2, 0 },

  { rtwCAPI_MATRIX_COL_MAJOR, 28, 2, 0 },

  { rtwCAPI_MATRIX_COL_MAJOR, 30, 2, 0 },

  { rtwCAPI_MATRIX_COL_MAJOR, 32, 2, 0 },

  { rtwCAPI_MATRIX_COL_MAJOR, 34, 2, 0 }
};

// Dimension Array- use dimArrayIndex to access elements of this array
static uint_T rtDimensionArray[] = {
  1,                                   // 0
  1,                                   // 1
  41,                                  // 2
  1,                                   // 3
  3,                                   // 4
  1,                                   // 5
  242,                                 // 6
  1,                                   // 7
  4,                                   // 8
  1,                                   // 9
  3,                                   // 10
  123,                                 // 11
  1,                                   // 12
  242,                                 // 13
  1,                                   // 14
  1804,                                // 15
  1,                                   // 16
  243,                                 // 17
  1,                                   // 18
  7742,                                // 19
  1,                                   // 20
  365,                                 // 21
  2,                                   // 22
  1,                                   // 23
  92,                                  // 24
  92,                                  // 25
  1,                                   // 26
  7,                                   // 27
  1,                                   // 28
  13,                                  // 29
  91,                                  // 30
  1,                                   // 31
  92,                                  // 32
  1,                                   // 33
  2,                                   // 34
  3                                    // 35
};

// Fixed Point Map
static rtwCAPI_FixPtMap rtFixPtMap[] = {
  // fracSlopePtr, biasPtr, scaleType, wordLength, exponent, isSigned
  { (NULL), (NULL), rtwCAPI_FIX_RESERVED, 0, 0, (boolean_T)0 },
};

// Sample Time Map - use sTimeIndex to access elements of ths structure
static rtwCAPI_SampleTimeMap rtSampleTimeMap[] = {
  // samplePeriodPtr, sampleOffsetPtr, tid, samplingMode
  {
    (NULL), (NULL), 0, 0
  }
};

static rtwCAPI_ModelMappingStaticInfo mmiStatic = {
  // Signals:{signals, numSignals,
  //            rootInputs, numRootInputs,
  //            rootOutputs, numRootOutputs},
  //  Params: {blockParameters, numBlockParameters,
  //           modelParameters, numModelParameters},
  //  States: {states, numStates},
  //  Maps:   {dataTypeMap, dimensionMap, fixPtMap,
  //           elementMap, sampleTimeMap, dimensionArray},
  //  TargetType: targetType

  { (NULL), 0,
    (NULL), 0,
    (NULL), 0 },

  { rtBlockParameters, 116,
    rtModelParameters, 52 },

  { (NULL), 0 },

  { rtDataTypeMap, rtDimensionMap, rtFixPtMap,
    rtElementMap, rtSampleTimeMap, rtDimensionArray },
  "float",

  { 1543852833U,
    2255478169U,
    3708576282U,
    3668923579U },
  (NULL), 0,
  (boolean_T)0
};

// Function to get C API Model Mapping Static Info
const rtwCAPI_ModelMappingStaticInfo*
  mvdc_mpc_GetCAPIStaticMap(void)
{
  return &mmiStatic;
}

// Cache pointers into DataMapInfo substructure of RTModel
#ifndef HOST_CAPI_BUILD

void mvdc_mpc_InitializeDataMapInfo(RT_MODEL_mvdc_mpc_T *const mvdc_mpc_M)
{
  // run-time setup of addresses
  void* *rtDataAddrMap;
  int32_T* *rtVarDimsAddrMap;
  rt_FREE( rtwCAPI_GetDataAddressMap( &(mvdc_mpc_M->DataMapInfo.mmi) ) );
  rtDataAddrMap = (void* *) malloc(168 * sizeof(void* ));
  if ((rtDataAddrMap) == (NULL)) {
    mvdc_mpc_M->setErrorStatus(RT_MEMORY_ALLOCATION_ERROR);
    return;
  }

  rtDataAddrMap[0] = (void* )(&mvdc_mpc_P.Memory_InitialCondition);
  rtDataAddrMap[1] = (void* )(&mvdc_mpc_P.Constant_Value_c);
  rtDataAddrMap[2] = (void* )(&mvdc_mpc_P.Constant1_Value_c);
  rtDataAddrMap[3] = (void* )(&mvdc_mpc_P.Memory_InitialCondition_a);
  rtDataAddrMap[4] = (void* )(&mvdc_mpc_P.Memory1_InitialCondition);
  rtDataAddrMap[5] = (void* )(&mvdc_mpc_P.Gain1_Gain);
  rtDataAddrMap[6] = (void* )(&mvdc_mpc_P.CompareToConstant_const_l);
  rtDataAddrMap[7] = (void* )(&mvdc_mpc_P.Constant_Value_a);
  rtDataAddrMap[8] = (void* )(&mvdc_mpc_P.UnitDelay1_InitialCondition_n);
  rtDataAddrMap[9] = (void* )(&mvdc_mpc_P.TransferFcnFirstOrder_ICPrevOutput_p);
  rtDataAddrMap[10] = (void* )(&mvdc_mpc_P.Constant_Value_j);
  rtDataAddrMap[11] = (void* )(&mvdc_mpc_P.Saturation1_UpperSat);
  rtDataAddrMap[12] = (void* )(&mvdc_mpc_P.Saturation1_LowerSat);
  rtDataAddrMap[13] = (void* )(&mvdc_mpc_P.UnitDelay1_InitialCondition_d);
  rtDataAddrMap[14] = (void* )(&mvdc_mpc_P.OptimizerOK_Y0);
  rtDataAddrMap[15] = (void* )(&mvdc_mpc_P.exitflags_Y0);
  rtDataAddrMap[16] = (void* )(&mvdc_mpc_P.solver_details_Y0);
  rtDataAddrMap[17] = (void* )(&mvdc_mpc_P.ResampledTargetTrajectory_Y0);
  rtDataAddrMap[18] = (void* )(&mvdc_mpc_P.MiscDebug_Y0);
  rtDataAddrMap[19] = (void* )(&mvdc_mpc_P.Prediction_Y0);
  rtDataAddrMap[20] = (void* )(&mvdc_mpc_P.Linearization_Y0);
  rtDataAddrMap[21] = (void* )(&mvdc_mpc_P.CompareToConstant1_const);
  rtDataAddrMap[22] = (void* )(&mvdc_mpc_P.CompareToConstant2_const);
  rtDataAddrMap[23] = (void* )(&mvdc_mpc_P.CompareToConstant3_const);
  rtDataAddrMap[24] = (void* )(&mvdc_mpc_P.CompareToConstant4_const);
  rtDataAddrMap[25] = (void* )(&mvdc_mpc_P.CompareToConstant5_const);
  rtDataAddrMap[26] = (void* )(&mvdc_mpc_P.CompareToConstant7_const);
  rtDataAddrMap[27] = (void* )(&mvdc_mpc_P.DetectIncrease_vinit);
  rtDataAddrMap[28] = (void* )(&mvdc_mpc_P.Memory2_InitialCondition);
  rtDataAddrMap[29] = (void* )(&mvdc_mpc_P.Memory3_InitialCondition);
  rtDataAddrMap[30] = (void* )(&mvdc_mpc_P.Memory6_InitialCondition);
  rtDataAddrMap[31] = (void* )(&mvdc_mpc_P.Memory7_InitialCondition);
  rtDataAddrMap[32] = (void* )(&mvdc_mpc_P.SFunction_OSQP_P1[0]);
  rtDataAddrMap[33] = (void* )(&mvdc_mpc_P.SFunction_OSQP_P2);
  rtDataAddrMap[34] = (void* )(&mvdc_mpc_P.SFunction_OSQP_P3);
  rtDataAddrMap[35] = (void* )(&mvdc_mpc_P.SFunction_OSQP_P4[0]);
  rtDataAddrMap[36] = (void* )(&mvdc_mpc_P.SFunction_OSQP_P5[0]);
  rtDataAddrMap[37] = (void* )(&mvdc_mpc_P.SFunction_OSQP_P6[0]);
  rtDataAddrMap[38] = (void* )(&mvdc_mpc_P.SFunction_OSQP_P7[0]);
  rtDataAddrMap[39] = (void* )(&mvdc_mpc_P.SFunction_OSQP_P8[0]);
  rtDataAddrMap[40] = (void* )(&mvdc_mpc_P.SFunction_OSQP_P9[0]);
  rtDataAddrMap[41] = (void* )(&mvdc_mpc_P.SFunction_OSQP_P10[0]);
  rtDataAddrMap[42] = (void* )(&mvdc_mpc_P.SFunction_OSQP_P11[0]);
  rtDataAddrMap[43] = (void* )(&mvdc_mpc_P.SFunction_OSQP_P12);
  rtDataAddrMap[44] = (void* )(&mvdc_mpc_P.CompareToConstant1_const_ht);
  rtDataAddrMap[45] = (void* )(&mvdc_mpc_P.CompareToConstant2_const_c);
  rtDataAddrMap[46] = (void* )(&mvdc_mpc_P.Constant_Value);
  rtDataAddrMap[47] = (void* )(&mvdc_mpc_P.Constant10_Value);
  rtDataAddrMap[48] = (void* )(&mvdc_mpc_P.Constant3_Value);
  rtDataAddrMap[49] = (void* )(&mvdc_mpc_P.Constant4_Value);
  rtDataAddrMap[50] = (void* )(&mvdc_mpc_P.Constant5_Value);
  rtDataAddrMap[51] = (void* )(&mvdc_mpc_P.Constant6_Value);
  rtDataAddrMap[52] = (void* )(&mvdc_mpc_P.Constant7_Value);
  rtDataAddrMap[53] = (void* )(&mvdc_mpc_P.Constant8_Value);
  rtDataAddrMap[54] = (void* )(&mvdc_mpc_P.Constant9_Value);
  rtDataAddrMap[55] = (void* )(&mvdc_mpc_P.DiscreteTimeIntegrator_gainval);
  rtDataAddrMap[56] = (void* )(&mvdc_mpc_P.DiscreteTimeIntegrator_IC);
  rtDataAddrMap[57] = (void* )(&mvdc_mpc_P.Gain_Gain);
  rtDataAddrMap[58] = (void* )(&mvdc_mpc_P.TransferFcnFirstOrder_ICPrevOutput_f);
  rtDataAddrMap[59] = (void* )(&mvdc_mpc_P.TransferFcnFirstOrder1_ICPrevOutput);
  rtDataAddrMap[60] = (void* )(&mvdc_mpc_P.Constant4_Value_o);
  rtDataAddrMap[61] = (void* )(&mvdc_mpc_P.CompareToConstant1_const_c);
  rtDataAddrMap[62] = (void* )(&mvdc_mpc_P.TransferFcnFirstOrder_ICPrevOutput_b);
  rtDataAddrMap[63] = (void* )(&mvdc_mpc_P.Gain4_Gain);
  rtDataAddrMap[64] = (void* )(&mvdc_mpc_P.Constant_Value_h);
  rtDataAddrMap[65] = (void* )(&mvdc_mpc_P.Constant1_Value);
  rtDataAddrMap[66] = (void* )(&mvdc_mpc_P.Constant2_Value);
  rtDataAddrMap[67] = (void* )(&mvdc_mpc_P.Constant4_Value_b);
  rtDataAddrMap[68] = (void* )(&mvdc_mpc_P.Gain10_Gain);
  rtDataAddrMap[69] = (void* )(&mvdc_mpc_P.Saturation1_UpperSat_o);
  rtDataAddrMap[70] = (void* )(&mvdc_mpc_P.Saturation1_LowerSat_g);
  rtDataAddrMap[71] = (void* )(&mvdc_mpc_P.UnitDelay_InitialCondition_j[0]);
  rtDataAddrMap[72] = (void* )(&mvdc_mpc_P.WrapToZero_Threshold);
  rtDataAddrMap[73] = (void* )(&mvdc_mpc_P.Output_InitialCondition);
  rtDataAddrMap[74] = (void* )(&mvdc_mpc_P.ax_LearnedBound_mps2_Y0);
  rtDataAddrMap[75] = (void* )(&mvdc_mpc_P.ax_DataCoverage_perc_Y0);
  rtDataAddrMap[76] = (void* )(&mvdc_mpc_P.Constant_Value_p);
  rtDataAddrMap[77] = (void* )(&mvdc_mpc_P.Constant4_Value_e);
  rtDataAddrMap[78] = (void* )(&mvdc_mpc_P.Gain3_Gain);
  rtDataAddrMap[79] = (void* )(&mvdc_mpc_P.Saturation_UpperSat);
  rtDataAddrMap[80] = (void* )(&mvdc_mpc_P.Saturation_LowerSat);
  rtDataAddrMap[81] = (void* )(&mvdc_mpc_P.Saturation1_UpperSat_c);
  rtDataAddrMap[82] = (void* )(&mvdc_mpc_P.Saturation1_LowerSat_p);
  rtDataAddrMap[83] = (void* )(&mvdc_mpc_P.TappedDelay_vinit);
  rtDataAddrMap[84] = (void* )(&mvdc_mpc_P.Delay_InitialCondition);
  rtDataAddrMap[85] = (void* )(&mvdc_mpc_P.UnitDelay_InitialCondition);
  rtDataAddrMap[86] = (void* )(&mvdc_mpc_P.ay_LearnedBound_mps2_Y0);
  rtDataAddrMap[87] = (void* )(&mvdc_mpc_P.ay_DataCoverage_perc_Y0);
  rtDataAddrMap[88] = (void* )(&mvdc_mpc_P.Constant_Value_ht);
  rtDataAddrMap[89] = (void* )(&mvdc_mpc_P.Constant4_Value_m);
  rtDataAddrMap[90] = (void* )(&mvdc_mpc_P.Gain3_Gain_a);
  rtDataAddrMap[91] = (void* )(&mvdc_mpc_P.Saturation_UpperSat_g);
  rtDataAddrMap[92] = (void* )(&mvdc_mpc_P.Saturation_LowerSat_p);
  rtDataAddrMap[93] = (void* )(&mvdc_mpc_P.Saturation1_UpperSat_g);
  rtDataAddrMap[94] = (void* )(&mvdc_mpc_P.Saturation1_LowerSat_j);
  rtDataAddrMap[95] = (void* )(&mvdc_mpc_P.TappedDelay_vinit_k);
  rtDataAddrMap[96] = (void* )(&mvdc_mpc_P.Delay_InitialCondition_n);
  rtDataAddrMap[97] = (void* )(&mvdc_mpc_P.UnitDelay_InitialCondition_a);
  rtDataAddrMap[98] = (void* )(&mvdc_mpc_P.CompareToConstant_const);
  rtDataAddrMap[99] = (void* )(&mvdc_mpc_P.CompareToConstant1_const_h);
  rtDataAddrMap[100] = (void* )(&mvdc_mpc_P.CompareToConstant2_const_m);
  rtDataAddrMap[101] = (void* )(&mvdc_mpc_P.DiscreteDerivative_ICPrevScaledInput);
  rtDataAddrMap[102] = (void* )(&mvdc_mpc_P.TransferFcnFirstOrder_ICPrevOutput);
  rtDataAddrMap[103] = (void* )
    (&mvdc_mpc_P.TransferFcnFirstOrder1_ICPrevOutput_c);
  rtDataAddrMap[104] = (void* )(&mvdc_mpc_P.UnitDelay_InitialCondition_p);
  rtDataAddrMap[105] = (void* )
    (&mvdc_mpc_P.TransferFcnFirstOrder1_ICPrevOutput_dx);
  rtDataAddrMap[106] = (void* )(&mvdc_mpc_P.UnitDelay_InitialCondition_k);
  rtDataAddrMap[107] = (void* )
    (&mvdc_mpc_P.TransferFcnFirstOrder1_ICPrevOutput_d);
  rtDataAddrMap[108] = (void* )(&mvdc_mpc_P.UnitDelay_InitialCondition_jd);
  rtDataAddrMap[109] = (void* )(&mvdc_mpc_P.P_upd_Y0[0]);
  rtDataAddrMap[110] = (void* )(&mvdc_mpc_P.mean_debug_Y0);
  rtDataAddrMap[111] = (void* )(&mvdc_mpc_P.cov_debug_Y0);
  rtDataAddrMap[112] = (void* )(&mvdc_mpc_P.UnitDelay1_InitialCondition[0]);
  rtDataAddrMap[113] = (void* )(&mvdc_mpc_P.FixPtConstant_Value);
  rtDataAddrMap[114] = (void* )(&mvdc_mpc_P.Constant_Value_m);
  rtDataAddrMap[115] = (void* )(&mvdc_mpc_P.TSamp_WtEt);
  rtDataAddrMap[116] = (void* )(&mvdc_mpc_P.acc_control_learning);
  rtDataAddrMap[117] = (void* )(&mvdc_mpc_P.A_VehicleReference_m2);
  rtDataAddrMap[118] = (void* )(&mvdc_mpc_P.P_VDC_ControlMargin_ax_mps2);
  rtDataAddrMap[119] = (void* )(&mvdc_mpc_P.P_VDC_ControlMargin_ay_mps2);
  rtDataAddrMap[120] = (void* )(&mvdc_mpc_P.P_VDC_Dist_ax_perc);
  rtDataAddrMap[121] = (void* )(&mvdc_mpc_P.P_VDC_Dist_ay_perc);
  rtDataAddrMap[122] = (void* )
    (&mvdc_mpc_P.P_VDC_IncreaseUncertaintyPerStep_perc);
  rtDataAddrMap[123] = (void* )(&mvdc_mpc_P.P_VDC_LatAccKi);
  rtDataAddrMap[124] = (void* )(&mvdc_mpc_P.P_VDC_LatAccKi_Lim_perc);
  rtDataAddrMap[125] = (void* )(&mvdc_mpc_P.P_VDC_LatAccKp_Delta);
  rtDataAddrMap[126] = (void* )(&mvdc_mpc_P.P_VDC_LatAccKp_ay);
  rtDataAddrMap[127] = (void* )(&mvdc_mpc_P.P_VDC_LatAcc_BetaGain);
  rtDataAddrMap[128] = (void* )(&mvdc_mpc_P.P_VDC_LatAcc_FF_Ts);
  rtDataAddrMap[129] = (void* )(&mvdc_mpc_P.P_VDC_LatAcc_Steering_Ts);
  rtDataAddrMap[130] = (void* )(&mvdc_mpc_P.P_VDC_LatAcc_UndersteerComp_Ts);
  rtDataAddrMap[131] = (void* )(&mvdc_mpc_P.P_VDC_LatAcc_VehicleDynamics_Ts);
  rtDataAddrMap[132] = (void* )(&mvdc_mpc_P.P_VDC_LatErrorDer_Ts_s);
  rtDataAddrMap[133] = (void* )(&mvdc_mpc_P.P_VDC_LongAccKp);
  rtDataAddrMap[134] = (void* )(&mvdc_mpc_P.P_VDC_LongAcc_FB_Ts);
  rtDataAddrMap[135] = (void* )(&mvdc_mpc_P.P_VDC_LongAcc_FF_Ts);
  rtDataAddrMap[136] = (void* )(&mvdc_mpc_P.P_VDC_LongAcc_LimFb_N);
  rtDataAddrMap[137] = (void* )(&mvdc_mpc_P.P_VDC_MaxTightening);
  rtDataAddrMap[138] = (void* )(&mvdc_mpc_P.P_VDC_MinVelSlipCalc_mps);
  rtDataAddrMap[139] = (void* )(&mvdc_mpc_P.P_VDC_PositiveAxLimScale);
  rtDataAddrMap[140] = (void* )(&mvdc_mpc_P.P_VDC_RTISQP_alpha_old);
  rtDataAddrMap[141] = (void* )(&mvdc_mpc_P.P_VDC_RTISQP_alpha_target);
  rtDataAddrMap[142] = (void* )(&mvdc_mpc_P.P_VDC_StabilizerRearSideSlip_rad);
  rtDataAddrMap[143] = (void* )(&mvdc_mpc_P.P_VDC_SteeringOvalComp_rad);
  rtDataAddrMap[144] = (void* )
    (&mvdc_mpc_P.P_VDC_SteeringSteadyStateUndersteerComp_rad);
  rtDataAddrMap[145] = (void* )(&mvdc_mpc_P.P_VDC_TMPCMaxSolverIter);
  rtDataAddrMap[146] = (void* )(&mvdc_mpc_P.P_VDC_TerminalSetBrakeTime_s);
  rtDataAddrMap[147] = (void* )(&mvdc_mpc_P.P_VDC_TuneTerminalSet_mps);
  rtDataAddrMap[148] = (void* )(&mvdc_mpc_P.P_VDC_UncertaintyLearningGain);
  rtDataAddrMap[149] = (void* )(&mvdc_mpc_P.P_VDC_UncertaintyLearningSamples);
  rtDataAddrMap[150] = (void* )(&mvdc_mpc_P.P_VDC_UncertaintyTarget_perc);
  rtDataAddrMap[151] = (void* )(&mvdc_mpc_P.P_VDC_VirtualController[0]);
  rtDataAddrMap[152] = (void* )(&mvdc_mpc_P.P_VDC_ayFFModelLearningRate);
  rtDataAddrMap[153] = (void* )(&mvdc_mpc_P.drag_coefficient);
  rtDataAddrMap[154] = (void* )(&mvdc_mpc_P.l_front_m);
  rtDataAddrMap[155] = (void* )(&mvdc_mpc_P.l_rear_m);
  rtDataAddrMap[156] = (void* )(&mvdc_mpc_P.roh_air);
  rtDataAddrMap[157] = (void* )(&mvdc_mpc_P.tS);
  rtDataAddrMap[158] = (void* )(&mvdc_mpc_P.tSSlow);
  rtDataAddrMap[159] = (void* )(&mvdc_mpc_P.vehiclemass_kg);
  rtDataAddrMap[160] = (void* )(&mvdc_mpc_P.P_VDC_EnableNumLatErrorDer_b);
  rtDataAddrMap[161] = (void* )(&mvdc_mpc_P.P_VDC_EnableTubeMPC);
  rtDataAddrMap[162] = (void* )(&mvdc_mpc_P.P_VDC_EnableUncertaintyLearning);
  rtDataAddrMap[163] = (void* )(&mvdc_mpc_P.P_VDC_EnableYawRateAccRepl);
  rtDataAddrMap[164] = (void* )(&mvdc_mpc_P.P_VDC_LatAcc_EnableForesightFF);
  rtDataAddrMap[165] = (void* )(&mvdc_mpc_P.P_VDC_LongAcc_EnableForesightFF);
  rtDataAddrMap[166] = (void* )(&mvdc_mpc_P.P_VDC_axFFModelLearningEnable);
  rtDataAddrMap[167] = (void* )(&mvdc_mpc_P.P_VDC_ayFFModelLearningEnable);
  rt_FREE( rtwCAPI_GetVarDimsAddressMap( &(mvdc_mpc_M->DataMapInfo.mmi) ) );
  rtVarDimsAddrMap = (int32_T* *) malloc(1 * sizeof(int32_T* ));
  if ((rtVarDimsAddrMap) == (NULL)) {
    mvdc_mpc_M->setErrorStatus(RT_MEMORY_ALLOCATION_ERROR);
    return;
  }

  rtVarDimsAddrMap[0] = (int32_T* )((NULL));

  // Set C-API version
  rtwCAPI_SetVersion(mvdc_mpc_M->DataMapInfo.mmi, 1);

  // Cache static C-API data into the Real-time Model Data structure
  rtwCAPI_SetStaticMap(mvdc_mpc_M->DataMapInfo.mmi, &mmiStatic);

  // Cache static C-API logging data into the Real-time Model Data structure
  rtwCAPI_SetLoggingStaticMap(mvdc_mpc_M->DataMapInfo.mmi, (NULL));

  // Cache C-API Data Addresses into the Real-Time Model Data structure
  rtwCAPI_SetDataAddressMap(mvdc_mpc_M->DataMapInfo.mmi, rtDataAddrMap);

  // Cache C-API Data Run-Time Dimension Buffer Addresses into the Real-Time Model Data structure 
  rtwCAPI_SetVarDimsAddressMap(mvdc_mpc_M->DataMapInfo.mmi, rtVarDimsAddrMap);

  // Cache the instance C-API logging pointer
  rtwCAPI_SetInstanceLoggingInfo(mvdc_mpc_M->DataMapInfo.mmi, (NULL));

  // Set reference to submodels
  rtwCAPI_SetChildMMIArray(mvdc_mpc_M->DataMapInfo.mmi, (NULL));
  rtwCAPI_SetChildMMIArrayLen(mvdc_mpc_M->DataMapInfo.mmi, 0);
}

#else                                  // HOST_CAPI_BUILD
#ifdef __cplusplus

extern "C"
{

#endif

  void mvdc_mpc_host_InitializeDataMapInfo(mvdc_mpc_host_DataMapInfo_T *dataMap,
    const char *path)
  {
    // Set C-API version
    rtwCAPI_SetVersion(dataMap->mmi, 1);

    // Cache static C-API data into the Real-time Model Data structure
    rtwCAPI_SetStaticMap(dataMap->mmi, &mmiStatic);

    // host data address map is NULL
    rtwCAPI_SetDataAddressMap(dataMap->mmi, (NULL));

    // host vardims address map is NULL
    rtwCAPI_SetVarDimsAddressMap(dataMap->mmi, (NULL));

    // Set Instance specific path
    rtwCAPI_SetPath(dataMap->mmi, path);
    rtwCAPI_SetFullPath(dataMap->mmi, (NULL));

    // Set reference to submodels
    rtwCAPI_SetChildMMIArray(dataMap->mmi, (NULL));
    rtwCAPI_SetChildMMIArrayLen(dataMap->mmi, 0);
  }

#ifdef __cplusplus

}

#endif
#endif                                 // HOST_CAPI_BUILD

//
// File trailer for generated code.
//
// [EOF]
//
