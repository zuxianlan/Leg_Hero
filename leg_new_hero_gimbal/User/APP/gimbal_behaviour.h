#ifndef GIMBAL_BEHAVIOUR_H
#define GIMBAL_BEHAVIOUR_H

#include "gimbal_task.h"
#include "arm_math.h"


typedef enum
{
    GIMBAL_ZERO_FORCE = 0,      //云台无力
    GIMBAL_ABSOLUTE_ANGLE,      //云台绝对角控制
    GIMBAL_RELATIVE_ANGLE,      //云台相对角控制
    GIMBAL_AUTO_ATTACK,         //自瞄控制云台
    GIMBAL_REMOTE_FIRE,         //吊射模式
    GIMBAL_TURN_ROUND,          //一键调头
    GIMBAL_INIT                 //云台电机在初始的时候的控制(基本上是防腿打头)
} gimbal_behaviour_e;


extern void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set);
extern void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, gimbal_control_t *gimbal_control_set);

extern gimbal_behaviour_e gimbal_behaviour;

extern bool_t AUTO_ATTACK;
extern bool_t TURN_ROUND;

#endif
