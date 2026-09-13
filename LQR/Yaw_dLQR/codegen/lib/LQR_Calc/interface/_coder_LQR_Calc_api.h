/*
 * File: _coder_LQR_Calc_api.h
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2026-06-08 16:33:34
 */

#ifndef _CODER_LQR_CALC_API_H
#define _CODER_LQR_CALC_API_H

/* Include Files */
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#include <string.h>

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
real_T LQR_Calc(real_T theta_ref, real_T theta_now, real_T omega_now);

void LQR_Calc_api(const mxArray *const prhs[3], const mxArray **plhs);

void LQR_Calc_atexit(void);

void LQR_Calc_initialize(void);

void LQR_Calc_terminate(void);

void LQR_Calc_xil_shutdown(void);

void LQR_Calc_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_LQR_Calc_api.h
 *
 * [EOF]
 */
