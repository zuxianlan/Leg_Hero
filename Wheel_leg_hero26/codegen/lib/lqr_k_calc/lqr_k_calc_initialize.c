/*
 * File: lqr_k_calc_initialize.c
 *
 * MATLAB Coder version            : 5.6
 * C/C++ source code generated on  : 29-Jun-2026 22:36:08
 */

/* Include Files */
#include "lqr_k_calc_initialize.h"
#include "lqr_k_calc_data.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : void
 */
void lqr_k_calc_initialize(void)
{
  rt_InitInfAndNaN();
  isInitialized_lqr_k_calc = true;
}

/*
 * File trailer for lqr_k_calc_initialize.c
 *
 * [EOF]
 */
