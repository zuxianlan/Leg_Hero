/*
 * File: _coder_x_ddot_api.h
 *
 * MATLAB Coder version            : 5.6
 * C/C++ source code generated on  : 20-Nov-2025 20:13:58
 */

#ifndef _CODER_X_DDOT_API_H
#define _CODER_X_DDOT_API_H

/* Include Files */
#include "emlrt.h"
#include "tmwtypes.h"
#include <string.h>

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
real_T x_ddot(real_T L0, real_T T, real_T Tp, real_T theta, real_T theta1);

void x_ddot_api(const mxArray *const prhs[5], const mxArray **plhs);

void x_ddot_atexit(void);

void x_ddot_initialize(void);

void x_ddot_terminate(void);

void x_ddot_xil_shutdown(void);

void x_ddot_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_x_ddot_api.h
 *
 * [EOF]
 */
