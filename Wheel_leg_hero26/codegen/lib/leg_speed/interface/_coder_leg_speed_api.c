/*
 * File: _coder_leg_speed_api.c
 *
 * MATLAB Coder version            : 5.6
 * C/C++ source code generated on  : 18-Dec-2025 00:30:35
 */

/* Include Files */
#include "_coder_leg_speed_api.h"
#include "_coder_leg_speed_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;

emlrtContext emlrtContextGlobal = {
    true,                                                 /* bFirstTime */
    false,                                                /* bInitialized */
    131642U,                                              /* fVersionInfo */
    NULL,                                                 /* fErrorFunction */
    "leg_speed",                                          /* fFunctionName */
    NULL,                                                 /* fRTCallStack */
    false,                                                /* bDebugMode */
    {2045744189U, 2170104910U, 2743257031U, 4284093946U}, /* fSigWrd */
    NULL                                                  /* fSigMem */
};

/* Function Declarations */
static real32_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId);

static real32_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId);

static real32_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *dphi1,
                                 const char_T *identifier);

static const mxArray *emlrt_marshallOut(const real32_T u[2]);

/* Function Definitions */
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real32_T
 */
static real32_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId)
{
  real32_T y;
  y = c_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real32_T
 */
static real32_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  real32_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)sp, msgId, src, "single|double", false,
                          0U, (const void *)&dims);
  emlrtImportArrayR2015b((emlrtConstCTX)sp, src, &ret, 4, false);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *dphi1
 *                const char_T *identifier
 * Return Type  : real32_T
 */
static real32_T emlrt_marshallIn(const emlrtStack *sp, const mxArray *dphi1,
                                 const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  real32_T y;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(dphi1), &thisId);
  emlrtDestroyArray(&dphi1);
  return y;
}

/*
 * Arguments    : const real32_T u[2]
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(const real32_T u[2])
{
  static const int32_T i = 2;
  const mxArray *m;
  const mxArray *y;
  real32_T *pData;
  y = NULL;
  m = emlrtCreateNumericArray(1, (const void *)&i, mxSINGLE_CLASS, mxREAL);
  pData = (real32_T *)emlrtMxGetData(m);
  pData[0] = u[0];
  pData[1] = u[1];
  emlrtAssign(&y, m);
  return y;
}

/*
 * Arguments    : const mxArray * const prhs[4]
 *                const mxArray **plhs
 * Return Type  : void
 */
void leg_speed_api(const mxArray *const prhs[4], const mxArray **plhs)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  real32_T speed[2];
  real32_T dphi1;
  real32_T dphi4;
  real32_T phi1;
  real32_T phi4;
  st.tls = emlrtRootTLSGlobal;
  /* Marshall function inputs */
  dphi1 = emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "dphi1");
  dphi4 = emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "dphi4");
  phi1 = emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "phi1");
  phi4 = emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "phi4");
  /* Invoke the target function */
  leg_speed(dphi1, dphi4, phi1, phi4, speed);
  /* Marshall function outputs */
  *plhs = emlrt_marshallOut(speed);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void leg_speed_atexit(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  leg_speed_xil_terminate();
  leg_speed_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void leg_speed_initialize(void)
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, NULL);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void leg_speed_terminate(void)
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * File trailer for _coder_leg_speed_api.c
 *
 * [EOF]
 */
