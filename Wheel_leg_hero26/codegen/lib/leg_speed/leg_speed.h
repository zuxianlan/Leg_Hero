/*
 * File: leg_speed.h
 *
 * MATLAB Coder version            : 5.6
 * C/C++ source code generated on  : 18-Dec-2025 00:30:35
 */

#ifndef LEG_SPEED_H
#define LEG_SPEED_H

/* Include Files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void leg_speed(float dphi1, float dphi4, float phi1, float phi4,
                      float speed[2]);

extern void leg_speed_initialize(void);

extern void leg_speed_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for leg_speed.h
 *
 * [EOF]
 */
