//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: mvdc_path_matching.h
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
#ifndef mvdc_path_matching_h_
#define mvdc_path_matching_h_
#include <stdlib.h>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "mvdc_path_matching_types.h"

extern "C"
{

#include "rt_nonfinite.h"

}

extern "C"
{

#include "rtGetNaN.h"

}

#include "rtw_modelmap.h"
#include <stddef.h>
#include <cstring>

// Block signals (default storage)
struct B_mvdc_path_matching_T {
  real_T Multiply;                     // '<S1>/Multiply'
  real_T d_m;
};

// Block states (default storage) for system '<Root>'
struct DW_mvdc_path_matching_T {
  Trajectory Delay2_DSTATE;            // '<S2>/Delay2'
  boolean_T DelayInput1_DSTATE;        // '<S5>/Delay Input1'
  boolean_T Delay2_DSTATE_j;           // '<Root>/Delay2'
};

// External inputs (root inport signals with default storage)
struct ExtU_mvdc_path_matching_T {
  VehicleDynamicState VehicleDynamicState_g;// '<Root>/VehicleDynamicState'
  TrajectoryPlanning TrajectoryPlanning_m;// '<Root>/TrajectoryPlanning'
  boolean_T EnablePathMatching;        // '<Root>/EnablePathMatching'
  boolean_T EnableEmergency;           // '<Root>/EnableEmergency'
};

// External outputs (root outports fed by signals with default storage)
struct ExtY_mvdc_path_matching_T {
  mvdc_path_matching_debug debug;      // '<Root>/debug'
  PathPos PathPos_i;                   // '<Root>/PathPos'
  TrajectoryPoint ActualTrajectoryPoint;// '<Root>/ActualTrajectoryPoint'
  Trajectory ActualTargetTrajectory;   // '<Root>/ActualTargetTrajectory'
  TUMPathMatchingState Status;         // '<Root>/Status'
};

// Parameters (default storage)
struct P_mvdc_path_matching_T_ {
  real_T P_VDC_FinalSpeedEmergency_mps;
                                      // Variable: P_VDC_FinalSpeedEmergency_mps
                                         //  Referenced by:
                                         //    '<S2>/MATLAB Function1'
                                         //    '<S2>/MATLAB Function2'

  real_T P_VDC_LateralPathDeviationDerMax_mps;
                               // Variable: P_VDC_LateralPathDeviationDerMax_mps
                                  //  Referenced by: '<Root>/Truth Table'

  real_T P_VDC_LateralPathDeviationMax_m;
                                    // Variable: P_VDC_LateralPathDeviationMax_m
                                       //  Referenced by: '<Root>/Truth Table'

  real_T drag_coefficient;             // Variable: drag_coefficient
                                          //  Referenced by:
                                          //    '<S2>/MATLAB Function1'
                                          //    '<S2>/MATLAB Function2'

  real_T roh_air;                      // Variable: roh_air
                                          //  Referenced by:
                                          //    '<S2>/MATLAB Function1'
                                          //    '<S2>/MATLAB Function2'

  real_T vehiclemass_kg;               // Variable: vehiclemass_kg
                                          //  Referenced by:
                                          //    '<S2>/MATLAB Function1'
                                          //    '<S2>/MATLAB Function2'

  boolean_T P_VDC_EnableSafetyChecks;  // Variable: P_VDC_EnableSafetyChecks
                                          //  Referenced by: '<S2>/Constant'

  boolean_T DetectChange_vinit;        // Mask Parameter: DetectChange_vinit
                                          //  Referenced by: '<S5>/Delay Input1'

  Trajectory Delay2_InitialCondition;
                                  // Computed Parameter: Delay2_InitialCondition
                                     //  Referenced by: '<S2>/Delay2'

  TrajectoryPoint ActualTrajectoryPoint_Y0;
                                 // Computed Parameter: ActualTrajectoryPoint_Y0
                                    //  Referenced by: '<S1>/ActualTrajectoryPoint'

  PathPos PathPos_Y0;                  // Computed Parameter: PathPos_Y0
                                          //  Referenced by: '<S1>/PathPos'

  real_T d_m_Y0;                       // Computed Parameter: d_m_Y0
                                          //  Referenced by: '<S1>/d_m'

  real_T d_dot_mps_Y0;                 // Computed Parameter: d_dot_mps_Y0
                                          //  Referenced by: '<S1>/d_dot_mps'

  boolean_T Constant1_Value;           // Expression: boolean(1)
                                          //  Referenced by: '<S2>/Constant1'

  boolean_T Delay2_InitialCondition_c;
                                // Computed Parameter: Delay2_InitialCondition_c
                                   //  Referenced by: '<Root>/Delay2'

};

// Real-time Model Data Structure
struct tag_RTM_mvdc_path_matching_T {
  const char_T * volatile errorStatus;
  B_mvdc_path_matching_T *blockIO;
  ExtU_mvdc_path_matching_T *inputs;
  ExtY_mvdc_path_matching_T *outputs;
  DW_mvdc_path_matching_T *dwork;

  //
  //  DataMapInfo:
  //  The following substructure contains information regarding
  //  structures generated in the model's C API.

  struct DataMapInfo_T {
    rtwCAPI_ModelMappingInfo mmi;
    void* dataAddress[15];
    int32_T* vardimsAddress[15];
    RTWLoggingFcnPtr loggingPtrs[15];
  };

  DataMapInfo_T DataMapInfo;
  const char_T* getErrorStatus() const;
  void setErrorStatus(const char_T* const volatile aErrorStatus);
  RT_MODEL_mvdc_path_matching_T::DataMapInfo_T getDataMapInfo() const;
  void setDataMapInfo(RT_MODEL_mvdc_path_matching_T::DataMapInfo_T aDataMapInfo);
};

// Block parameters (default storage)
#ifdef __cplusplus

extern "C"
{

#endif

  extern P_mvdc_path_matching_T mvdc_path_matching_P;

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

extern P_mvdc_path_matching_T mvdc_path_matching_P;// parameters

#ifdef __cplusplus

extern "C"
{

#endif

  // Model entry point functions
  extern RT_MODEL_mvdc_path_matching_T *mvdc_path_matching(void);
  extern void mvdc_path_matching_initialize(RT_MODEL_mvdc_path_matching_T *const
    mvdc_path_matching_M);
  extern void mvdc_path_matching_step(RT_MODEL_mvdc_path_matching_T *const
    mvdc_path_matching_M);
  extern void mvdc_path_matching_terminate(RT_MODEL_mvdc_path_matching_T
    * mvdc_path_matching_M);

#ifdef __cplusplus

}

#endif

// Function to get C API Model Mapping Static Info
extern const rtwCAPI_ModelMappingStaticInfo*
  mvdc_path_matching_GetCAPIStaticMap(void);

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<Root>/Signal Copy' : Eliminate redundant signal conversion block
//  Block '<Root>/Signal Copy1' : Eliminate redundant signal conversion block
//  Block '<Root>/Signal Copy2' : Eliminate redundant signal conversion block


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
//  '<Root>' : 'mvdc_path_matching'
//  '<S1>'   : 'mvdc_path_matching/Subsystem'
//  '<S2>'   : 'mvdc_path_matching/TrajectoryHandler'
//  '<S3>'   : 'mvdc_path_matching/Truth Table'
//  '<S4>'   : 'mvdc_path_matching/Subsystem/MATLAB Function'
//  '<S5>'   : 'mvdc_path_matching/TrajectoryHandler/Detect Change'
//  '<S6>'   : 'mvdc_path_matching/TrajectoryHandler/MATLAB Function1'
//  '<S7>'   : 'mvdc_path_matching/TrajectoryHandler/MATLAB Function2'

#endif                                 // mvdc_path_matching_h_

//
// File trailer for generated code.
//
// [EOF]
//
