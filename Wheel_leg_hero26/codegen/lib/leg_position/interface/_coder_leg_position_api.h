/*
 * File: _coder_leg_position_api.h
 *
 * MATLAB Coder version            : 5.6
 * C/C++ source code generated on  : 18-Dec-2025 00:23:04
 */

#ifndef _CODER_LEG_POSITION_API_H
#define _CODER_LEG_POSITION_API_H

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
void leg_position(real32_T phi1, real32_T phi4, real32_T position[2]);

void leg_position_api(const mxArray *const prhs[2], const mxArray **plhs);

void leg_position_atexit(void);

void leg_position_initialize(void);

void leg_position_terminate(void);

void leg_position_xil_shutdown(void);

void leg_position_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_leg_position_api.h
 *
 * [EOF]
 */
