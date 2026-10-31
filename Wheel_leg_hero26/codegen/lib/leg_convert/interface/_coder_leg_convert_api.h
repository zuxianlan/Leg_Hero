/*
 * File: _coder_leg_convert_api.h
 *
 * MATLAB Coder version            : 5.6
 * C/C++ source code generated on  : 18-Dec-2025 00:46:57
 */

#ifndef _CODER_LEG_CONVERT_API_H
#define _CODER_LEG_CONVERT_API_H

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
void leg_convert(real32_T F, real32_T Tp, real32_T phi1, real32_T phi4,
                 real32_T T[2]);

void leg_convert_api(const mxArray *const prhs[4], const mxArray **plhs);

void leg_convert_atexit(void);

void leg_convert_initialize(void);

void leg_convert_terminate(void);

void leg_convert_xil_shutdown(void);

void leg_convert_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_leg_convert_api.h
 *
 * [EOF]
 */
