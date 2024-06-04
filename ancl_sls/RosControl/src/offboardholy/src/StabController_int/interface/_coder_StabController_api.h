//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_StabController_api.h
//
// MATLAB Coder version            : 5.4
// C/C++ source code generated on  : 31-May-2024 01:17:58
//

#ifndef _CODER_STABCONTROLLER_API_H
#define _CODER_STABCONTROLLER_API_H

// Include Files
#include "emlrt.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void StabController(real_T x[13], real_T Kv[15], real_T param[4],
                    real_T setpoint[3], real_T u[3]);

void StabController_api(const mxArray *const prhs[4], const mxArray **plhs);

void StabController_atexit();

void StabController_initialize();

void StabController_terminate();

void StabController_xil_shutdown();

void StabController_xil_terminate();

#endif
//
// File trailer for _coder_StabController_api.h
//
// [EOF]
//
