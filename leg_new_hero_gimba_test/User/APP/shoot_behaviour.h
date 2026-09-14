#ifndef SHOOT_BEHAVIOUR_H
#define SHOOT_BEHAVIOUR_H

#include "shoot_task.h"

typedef enum
{
    SHOOT_DISABLE = 0,
    SHOOT_ENABLE,
    SHOOT_DEBUG_,
    SHOOT_RC_GONGDAN,
    SHOOT_RC_HUIBO,
    SHOOT_MOUSE_GONGDAN,
    SHOOT_MOUSE_HUIBO,
} shoot_behaviour_e;

typedef enum
{
    TRIGGER_NORMAL = 0,
    TRIGGER_GONGDAN,
    TRIGGER_HUIBO,
} trigger_status_t;

//外部交互数据---------------------------------------------------------------------
extern shoot_behaviour_e shoot_behaviour;
extern trigger_status_t trigger_status;

extern void shoot_motor_mode_set(shoot_control_t *shoot_mode_set);
//------------------------------------------------------------------------------

#endif
