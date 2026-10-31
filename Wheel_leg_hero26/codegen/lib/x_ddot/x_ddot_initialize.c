/*
 * File: x_ddot_initialize.c
 *
 * MATLAB Coder version            : 5.6
 * C/C++ source code generated on  : 20-Nov-2025 20:13:58
 */

/* Include Files */
#include "x_ddot_initialize.h"
#include "rt_nonfinite.h"
#include "x_ddot_data.h"

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : void
 */
void x_ddot_initialize(void)
{
  rt_InitInfAndNaN();
  isInitialized_x_ddot = true;
}

/*
 * File trailer for x_ddot_initialize.c
 *
 * [EOF]
 */
