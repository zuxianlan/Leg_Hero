/*
 * File: leso_B_calc_initialize.c
 *
 * MATLAB Coder version            : 5.6
 * C/C++ source code generated on  : 29-Jun-2026 21:55:39
 */

/* Include Files */
#include "leso_B_calc_initialize.h"
#include "leso_B_calc_data.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : void
 */
void leso_B_calc_initialize(void)
{
  rt_InitInfAndNaN();
  isInitialized_leso_B_calc = true;
}

/*
 * File trailer for leso_B_calc_initialize.c
 *
 * [EOF]
 */
