/*
 * File: _coder_phi_ddot_api.h
 *
 * MATLAB Coder version            : 5.6
 * C/C++ source code generated on  : 20-Nov-2025 20:51:12
 */

#ifndef _CODER_PHI_DDOT_API_H
#define _CODER_PHI_DDOT_API_H

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
real_T phi_ddot(real_T Tp);

void phi_ddot_api(const mxArray *prhs, const mxArray **plhs);

void phi_ddot_atexit(void);

void phi_ddot_initialize(void);

void phi_ddot_terminate(void);

void phi_ddot_xil_shutdown(void);

void phi_ddot_xil_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_phi_ddot_api.h
 *
 * [EOF]
 */
