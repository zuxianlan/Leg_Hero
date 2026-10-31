/*
 * File: _coder_x_ddot_mex.h
 *
 * MATLAB Coder version            : 5.6
 * C/C++ source code generated on  : 20-Nov-2025 20:13:58
 */

#ifndef _CODER_X_DDOT_MEX_H
#define _CODER_X_DDOT_MEX_H

/* Include Files */
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[],
                                     int32_T nrhs, const mxArray *prhs[]);

emlrtCTX mexFunctionCreateRootTLS(void);

void unsafe_x_ddot_mexFunction(int32_T nlhs, mxArray *plhs[1], int32_T nrhs,
                               const mxArray *prhs[5]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_x_ddot_mex.h
 *
 * [EOF]
 */
