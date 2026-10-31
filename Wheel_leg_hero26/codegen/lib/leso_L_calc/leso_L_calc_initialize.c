/*
 * File: leso_L_calc_initialize.c
 *
 * MATLAB Coder version            : 5.6
 * C/C++ source code generated on  : 30-Jun-2026 21:31:16
 */

/* Include Files */
#include "leso_L_calc_initialize.h"
#include "leso_L_calc_data.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : void
 */
void leso_L_calc_initialize(void)
{
  rt_InitInfAndNaN();
  isInitialized_leso_L_calc = true;
}

/*
 * File trailer for leso_L_calc_initialize.c
 *
 * [EOF]
 */
