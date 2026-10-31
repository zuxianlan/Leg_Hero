/*
 * File: leso_A_calc_initialize.c
 *
 * MATLAB Coder version            : 5.6
 * C/C++ source code generated on  : 29-Jun-2026 21:56:23
 */

/* Include Files */
#include "leso_A_calc_initialize.h"
#include "leso_A_calc_data.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : void
 */
void leso_A_calc_initialize(void)
{
  rt_InitInfAndNaN();
  isInitialized_leso_A_calc = true;
}

/*
 * File trailer for leso_A_calc_initialize.c
 *
 * [EOF]
 */
