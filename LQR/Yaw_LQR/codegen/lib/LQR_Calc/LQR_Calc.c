/*
 * File: LQR_Calc.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 2026-06-08 16:33:34
 */

/* Include Files */
#include "LQR_Calc.h"

/* Function Definitions */
/*
 * 自动生成LQR增益，无需手动修改
 *
 * Arguments    : double theta_ref
 *                double theta_now
 *                double omega_now
 * Return Type  : double
 */
double LQR_Calc(double theta_ref, double theta_now, double omega_now)
{
  double u_torque;
  /*  LQR跟踪控制律 */
  u_torque = -15.411 * (theta_ref - theta_now) - 1.7554 * (0.0 - omega_now);
  /*  力矩饱和限幅 */
  if (u_torque > 1.0) {
    u_torque = 1.0;
  } else if (u_torque < -1.0) {
    u_torque = -1.0;
  }
  return u_torque;
}

/*
 * File trailer for LQR_Calc.c
 *
 * [EOF]
 */
