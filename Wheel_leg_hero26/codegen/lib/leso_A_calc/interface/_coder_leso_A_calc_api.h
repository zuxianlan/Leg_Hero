/*
 * File: _coder_leso_A_calc_api.h
 *
 * MATLAB Coder version            : 5.6
 * C/C++ source code generated on  : 29-Jun-2026 21:56:23
 */

#ifndef _CODER_LESO_A_CALC_API_H
#define _CODER_LESO_A_CALC_API_H

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
void leso_A_calc(real_T L0, real_T Ae[64]);

void leso_A_calc_api(const mxArray *prhs, const mxArray **plhs);

void leso_A_calc_atexit(void);

void leso_A_calc_initialize(void);

void leso_A_calc_terminate(void);

void leso_A_calc_xil_shutdown(void);

void leso_A_calc_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_leso_A_calc_api.h
 *
 * [EOF]
 */
