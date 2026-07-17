#ifndef mvdc_mpc_cap_host_h__
#define mvdc_mpc_cap_host_h__
#ifdef HOST_CAPI_BUILD
#include "rtw_capi.h"
#include "rtw_modelmap.h"

struct mvdc_mpc_host_DataMapInfo_T {
  rtwCAPI_ModelMappingInfo mmi;
};

#ifdef __cplusplus

extern "C"
{

#endif

  void mvdc_mpc_host_InitializeDataMapInfo(mvdc_mpc_host_DataMapInfo_T *dataMap,
    const char *path);

#ifdef __cplusplus

}

#endif
#endif                                 // HOST_CAPI_BUILD
#endif                                 // mvdc_mpc_cap_host_h__

// EOF: mvdc_mpc_capi_host.h
