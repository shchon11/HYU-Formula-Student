//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: mvdc_path_matching_capi.cpp
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
#include <stddef.h>
#include "rtw_capi.h"
#ifdef HOST_CAPI_BUILD
#include "mvdc_path_matching_capi_host.h"
#define sizeof(...)                    ((size_t)(0xFFFF))
#undef rt_offsetof
#define rt_offsetof(s,el)              ((uint16_T)(0xFFFF))
#define TARGET_CONST
#define TARGET_STRING(s)               (s)
#else                                  // HOST_CAPI_BUILD
#include "builtin_typeid_types.h"
#include "mvdc_path_matching.h"
#include "mvdc_path_matching_capi.h"
#include "mvdc_path_matching_private.h"
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

  { 0, TARGET_STRING("mvdc_path_matching/Delay2"),
    TARGET_STRING("InitialCondition"), 0, 0, 0 },

  { 1, TARGET_STRING("mvdc_path_matching/Subsystem/PathPos"),
    TARGET_STRING("InitialOutput"), 2, 0, 0 },

  { 2, TARGET_STRING("mvdc_path_matching/Subsystem/ActualTrajectoryPoint"),
    TARGET_STRING("InitialOutput"), 5, 0, 0 },

  { 3, TARGET_STRING("mvdc_path_matching/Subsystem/d_m"),
    TARGET_STRING("InitialOutput"), 1, 0, 0 },

  { 4, TARGET_STRING("mvdc_path_matching/Subsystem/d_dot_mps"),
    TARGET_STRING("InitialOutput"), 1, 0, 0 },

  { 5, TARGET_STRING("mvdc_path_matching/TrajectoryHandler/Detect Change"),
    TARGET_STRING("vinit"), 0, 0, 0 },

  { 6, TARGET_STRING("mvdc_path_matching/TrajectoryHandler/Constant1"),
    TARGET_STRING("Value"), 0, 0, 0 },

  { 7, TARGET_STRING("mvdc_path_matching/TrajectoryHandler/Delay2"),
    TARGET_STRING("InitialCondition"), 6, 0, 0 },

  {
    0, (NULL), (NULL), 0, 0, 0
  }
};

// Tunable variable parameters
static rtwCAPI_ModelParameters rtModelParameters[] = {
  // addrMapIndex, varName, dataTypeIndex, dimIndex, fixPtIndex
  { 8, TARGET_STRING("P_VDC_FinalSpeedEmergency_mps"), 1, 0, 0 },

  { 9, TARGET_STRING("P_VDC_LateralPathDeviationDerMax_mps"), 1, 0, 0 },

  { 10, TARGET_STRING("P_VDC_LateralPathDeviationMax_m"), 1, 0, 0 },

  { 11, TARGET_STRING("drag_coefficient"), 1, 0, 0 },

  { 12, TARGET_STRING("roh_air"), 1, 0, 0 },

  { 13, TARGET_STRING("vehiclemass_kg"), 1, 0, 0 },

  { 14, TARGET_STRING("P_VDC_EnableSafetyChecks"), 0, 0, 0 },

  { 0, (NULL), 0, 0, 0 }
};

// Data Type Map - use dataTypeMapIndex to access this structure
static TARGET_CONST rtwCAPI_DataTypeMap rtDataTypeMap[] = {
  // cName, mwName, numElements, elemMapIndex, dataSize, slDataId, *
  //  isComplex, isPointer, enumStorageType
  { "unsigned char", "boolean_T", 0, 0, sizeof(boolean_T), (uint8_T)SS_BOOLEAN,
    0, 0, 0 },

  { "double", "real_T", 0, 0, sizeof(real_T), (uint8_T)SS_DOUBLE, 0, 0, 0 },

  { "struct", "PathPos", 3, 1, sizeof(PathPos), (uint8_T)SS_STRUCT, 0, 0, 0 },

  { "unsigned int", "uint32_T", 0, 0, sizeof(uint32_T), (uint8_T)SS_UINT32, 0, 0,
    0 },

  { "unsigned short", "uint16_T", 0, 0, sizeof(uint16_T), (uint8_T)SS_UINT16, 0,
    0, 0 },

  { "struct", "TrajectoryPoint", 16, 4, sizeof(TrajectoryPoint), (uint8_T)
    SS_STRUCT, 0, 0, 0 },

  { "struct", "Trajectory", 15, 20, sizeof(Trajectory), (uint8_T)SS_STRUCT, 0, 0,
    0 }
};

#ifdef HOST_CAPI_BUILD
#undef sizeof
#endif

// Structure Element Map - use elemMapIndex to access this structure
static TARGET_CONST rtwCAPI_ElementMap rtElementMap[] = {
  // elementName, elementOffset, dataTypeIndex, dimIndex, fxpIndex
  { (NULL), 0, 0, 0, 0 },

  { "s_m", rt_offsetof(PathPos, s_m), 1, 0, 0 },

  { "d_m", rt_offsetof(PathPos, d_m), 1, 0, 0 },

  { "psi_rad", rt_offsetof(PathPos, psi_rad), 1, 0, 0 },

  { "LapCnt", rt_offsetof(TrajectoryPoint, LapCnt), 3, 0, 0 },

  { "TrajCnt", rt_offsetof(TrajectoryPoint, TrajCnt), 3, 0, 0 },

  { "PointIdx", rt_offsetof(TrajectoryPoint, PointIdx), 4, 0, 0 },

  { "s_loc_m", rt_offsetof(TrajectoryPoint, s_loc_m), 1, 0, 0 },

  { "s_glob_m", rt_offsetof(TrajectoryPoint, s_glob_m), 1, 0, 0 },

  { "x_m", rt_offsetof(TrajectoryPoint, x_m), 1, 0, 0 },

  { "y_m", rt_offsetof(TrajectoryPoint, y_m), 1, 0, 0 },

  { "psi_rad", rt_offsetof(TrajectoryPoint, psi_rad), 1, 0, 0 },

  { "kappa_radpm", rt_offsetof(TrajectoryPoint, kappa_radpm), 1, 0, 0 },

  { "v_mps", rt_offsetof(TrajectoryPoint, v_mps), 1, 0, 0 },

  { "ax_mps2", rt_offsetof(TrajectoryPoint, ax_mps2), 1, 0, 0 },

  { "banking_rad", rt_offsetof(TrajectoryPoint, banking_rad), 1, 0, 0 },

  { "ax_lim_mps2", rt_offsetof(TrajectoryPoint, ax_lim_mps2), 1, 0, 0 },

  { "ay_lim_mps2", rt_offsetof(TrajectoryPoint, ay_lim_mps2), 1, 0, 0 },

  { "tube_r_m", rt_offsetof(TrajectoryPoint, tube_r_m), 1, 0, 0 },

  { "tube_l_m", rt_offsetof(TrajectoryPoint, tube_l_m), 1, 0, 0 },

  { "LapCnt", rt_offsetof(Trajectory, LapCnt), 3, 0, 0 },

  { "TrajCnt", rt_offsetof(Trajectory, TrajCnt), 3, 0, 0 },

  { "s_loc_m", rt_offsetof(Trajectory, s_loc_m), 1, 1, 0 },

  { "s_glob_m", rt_offsetof(Trajectory, s_glob_m), 1, 1, 0 },

  { "x_m", rt_offsetof(Trajectory, x_m), 1, 1, 0 },

  { "y_m", rt_offsetof(Trajectory, y_m), 1, 1, 0 },

  { "psi_rad", rt_offsetof(Trajectory, psi_rad), 1, 1, 0 },

  { "kappa_radpm", rt_offsetof(Trajectory, kappa_radpm), 1, 1, 0 },

  { "v_mps", rt_offsetof(Trajectory, v_mps), 1, 1, 0 },

  { "ax_mps2", rt_offsetof(Trajectory, ax_mps2), 1, 1, 0 },

  { "banking_rad", rt_offsetof(Trajectory, banking_rad), 1, 1, 0 },

  { "ax_lim_mps2", rt_offsetof(Trajectory, ax_lim_mps2), 1, 1, 0 },

  { "ay_lim_mps2", rt_offsetof(Trajectory, ay_lim_mps2), 1, 1, 0 },

  { "tube_r_m", rt_offsetof(Trajectory, tube_r_m), 1, 1, 0 },

  { "tube_l_m", rt_offsetof(Trajectory, tube_l_m), 1, 1, 0 }
};

// Dimension Map - use dimensionMapIndex to access elements of ths structure
static rtwCAPI_DimensionMap rtDimensionMap[] = {
  // dataOrientation, dimArrayIndex, numDims, vardimsIndex
  { rtwCAPI_SCALAR, 0, 2, 0 },

  { rtwCAPI_VECTOR, 2, 2, 0 }
};

// Dimension Array- use dimArrayIndex to access elements of this array
static uint_T rtDimensionArray[] = {
  1,                                   // 0
  1,                                   // 1
  50,                                  // 2
  1                                    // 3
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

  { rtBlockParameters, 8,
    rtModelParameters, 7 },

  { (NULL), 0 },

  { rtDataTypeMap, rtDimensionMap, rtFixPtMap,
    rtElementMap, rtSampleTimeMap, rtDimensionArray },
  "float",

  { 2644097863U,
    2097760179U,
    2795271254U,
    1867509343U },
  (NULL), 0,
  (boolean_T)0
};

// Function to get C API Model Mapping Static Info
const rtwCAPI_ModelMappingStaticInfo*
  mvdc_path_matching_GetCAPIStaticMap(void)
{
  return &mmiStatic;
}

// Cache pointers into DataMapInfo substructure of RTModel
#ifndef HOST_CAPI_BUILD

void mvdc_path_matching_InitializeDataMapInfo(RT_MODEL_mvdc_path_matching_T *
  const mvdc_path_matching_M)
{
  // run-time setup of addresses
  void* *rtDataAddrMap;
  int32_T* *rtVarDimsAddrMap;
  rt_FREE( rtwCAPI_GetDataAddressMap( &(mvdc_path_matching_M->DataMapInfo.mmi) )
          );
  rtDataAddrMap = (void* *) malloc(15 * sizeof(void* ));
  if ((rtDataAddrMap) == (NULL)) {
    mvdc_path_matching_M->setErrorStatus(RT_MEMORY_ALLOCATION_ERROR);
    return;
  }

  rtDataAddrMap[0] = (void* )(&mvdc_path_matching_P.Delay2_InitialCondition_c);
  rtDataAddrMap[1] = (void* )(&mvdc_path_matching_P.PathPos_Y0);
  rtDataAddrMap[2] = (void* )(&mvdc_path_matching_P.ActualTrajectoryPoint_Y0);
  rtDataAddrMap[3] = (void* )(&mvdc_path_matching_P.d_m_Y0);
  rtDataAddrMap[4] = (void* )(&mvdc_path_matching_P.d_dot_mps_Y0);
  rtDataAddrMap[5] = (void* )(&mvdc_path_matching_P.DetectChange_vinit);
  rtDataAddrMap[6] = (void* )(&mvdc_path_matching_P.Constant1_Value);
  rtDataAddrMap[7] = (void* )(&mvdc_path_matching_P.Delay2_InitialCondition);
  rtDataAddrMap[8] = (void* )
    (&mvdc_path_matching_P.P_VDC_FinalSpeedEmergency_mps);
  rtDataAddrMap[9] = (void* )
    (&mvdc_path_matching_P.P_VDC_LateralPathDeviationDerMax_mps);
  rtDataAddrMap[10] = (void* )
    (&mvdc_path_matching_P.P_VDC_LateralPathDeviationMax_m);
  rtDataAddrMap[11] = (void* )(&mvdc_path_matching_P.drag_coefficient);
  rtDataAddrMap[12] = (void* )(&mvdc_path_matching_P.roh_air);
  rtDataAddrMap[13] = (void* )(&mvdc_path_matching_P.vehiclemass_kg);
  rtDataAddrMap[14] = (void* )(&mvdc_path_matching_P.P_VDC_EnableSafetyChecks);
  rt_FREE( rtwCAPI_GetVarDimsAddressMap( &(mvdc_path_matching_M->DataMapInfo.mmi)
           ) );
  rtVarDimsAddrMap = (int32_T* *) malloc(1 * sizeof(int32_T* ));
  if ((rtVarDimsAddrMap) == (NULL)) {
    mvdc_path_matching_M->setErrorStatus(RT_MEMORY_ALLOCATION_ERROR);
    return;
  }

  rtVarDimsAddrMap[0] = (int32_T* )((NULL));

  // Set C-API version
  rtwCAPI_SetVersion(mvdc_path_matching_M->DataMapInfo.mmi, 1);

  // Cache static C-API data into the Real-time Model Data structure
  rtwCAPI_SetStaticMap(mvdc_path_matching_M->DataMapInfo.mmi, &mmiStatic);

  // Cache static C-API logging data into the Real-time Model Data structure
  rtwCAPI_SetLoggingStaticMap(mvdc_path_matching_M->DataMapInfo.mmi, (NULL));

  // Cache C-API Data Addresses into the Real-Time Model Data structure
  rtwCAPI_SetDataAddressMap(mvdc_path_matching_M->DataMapInfo.mmi, rtDataAddrMap);

  // Cache C-API Data Run-Time Dimension Buffer Addresses into the Real-Time Model Data structure 
  rtwCAPI_SetVarDimsAddressMap(mvdc_path_matching_M->DataMapInfo.mmi,
    rtVarDimsAddrMap);

  // Cache the instance C-API logging pointer
  rtwCAPI_SetInstanceLoggingInfo(mvdc_path_matching_M->DataMapInfo.mmi, (NULL));

  // Set reference to submodels
  rtwCAPI_SetChildMMIArray(mvdc_path_matching_M->DataMapInfo.mmi, (NULL));
  rtwCAPI_SetChildMMIArrayLen(mvdc_path_matching_M->DataMapInfo.mmi, 0);
}

#else                                  // HOST_CAPI_BUILD
#ifdef __cplusplus

extern "C"
{

#endif

  void mvdc_path_matching_host_InitializeDataMapInfo
    (mvdc_path_matching_host_DataMapInfo_T *dataMap, const char *path)
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
