//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: mvdc_mpc_private.h
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
#ifndef mvdc_mpc_private_h_
#define mvdc_mpc_private_h_
#include "rtwtypes.h"
#include "builtin_typeid_types.h"
#include "mvdc_mpc_types.h"
#include "mvdc_mpc.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#if !defined(rt_VALIDATE_MEMORY)
#define rt_VALIDATE_MEMORY(S, ptr)     if(!(ptr)) {\
 mvdc_mpc_M->setErrorStatus(RT_MEMORY_ALLOCATION_ERROR);\
 }
#endif

#if !defined(rt_FREE)
#if !defined(_WIN32)
#define rt_FREE(ptr)                   if((ptr) != (NULL)) {\
 free((ptr));\
 (ptr) = (NULL);\
 }
#else

// Visual and other windows compilers declare free without const
#define rt_FREE(ptr)                   if((ptr) != (NULL)) {\
 free((void *)(ptr));\
 (ptr) = (NULL);\
 }
#endif
#endif

extern real_T rt_powd_snf(real_T u0, real_T u1);

#ifdef __cplusplus

extern "C"
{

#endif

#include "osqp_wrapper.h"
#ifdef __cplusplus

}

#endif
#endif                                 // mvdc_mpc_private_h_

//
// File trailer for generated code.
//
// [EOF]
//
