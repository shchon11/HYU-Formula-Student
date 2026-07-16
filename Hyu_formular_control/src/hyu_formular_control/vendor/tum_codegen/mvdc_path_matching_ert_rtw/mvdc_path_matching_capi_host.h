#ifndef mvdc_path_matching_cap_host_h__
#define mvdc_path_matching_cap_host_h__
#ifdef HOST_CAPI_BUILD
#include "rtw_capi.h"
#include "rtw_modelmap.h"

struct mvdc_path_matching_host_DataMapInfo_T {
  rtwCAPI_ModelMappingInfo mmi;
};

#ifdef __cplusplus

extern "C"
{

#endif

  void mvdc_path_matching_host_InitializeDataMapInfo
    (mvdc_path_matching_host_DataMapInfo_T *dataMap, const char *path);

#ifdef __cplusplus

}

#endif
#endif                                 // HOST_CAPI_BUILD
#endif                                 // mvdc_path_matching_cap_host_h__

// EOF: mvdc_path_matching_capi_host.h
