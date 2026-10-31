/*
 * File: _coder_lqr_k_calc_api.h
 *
 * MATLAB Coder version            : 5.6
 * C/C++ source code generated on  : 29-Jun-2026 22:36:08
 */

#ifndef _CODER_LQR_K_CALC_API_H
#define _CODER_LQR_K_CALC_API_H

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
void lqr_k_calc(real_T L0, real_T K[12]);

void lqr_k_calc_api(const mxArray *prhs, const mxArray **plhs);

void lqr_k_calc_atexit(void);

void lqr_k_calc_initialize(void);

void lqr_k_calc_terminate(void);

void lqr_k_calc_xil_shutdown(void);

void lqr_k_calc_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_lqr_k_calc_api.h
 *
 * [EOF]
 */
