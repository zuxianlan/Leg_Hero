/*
 * File: _coder_theta_ddot_api.h
 *
 * MATLAB Coder version            : 5.6
 * C/C++ source code generated on  : 28-Nov-2025 19:53:55
 */

#ifndef _CODER_THETA_DDOT_API_H
#define _CODER_THETA_DDOT_API_H

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
real_T theta_ddot(real_T L0, real_T T, real_T Tp, real_T theta, real_T theta1);

void theta_ddot_api(const mxArray *const prhs[5], const mxArray **plhs);

void theta_ddot_atexit(void);

void theta_ddot_initialize(void);

void theta_ddot_terminate(void);

void theta_ddot_xil_shutdown(void);

void theta_ddot_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_theta_ddot_api.h
 *
 * [EOF]
 */
