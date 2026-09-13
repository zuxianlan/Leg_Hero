#include "gimbal_behaviour.h"
#include "vision_task.h"
#include "RC_Task.h"

//函数声明------------------------------------------------------------------------
static void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set);

static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
static void gimbal_absolute_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
static void gimbal_relative_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
static void gimbal_auto_attack_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
static void gimbal_remote_fire_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
static void gimbal_turn_round_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
static void gimbal_init_control(fp32 *add_yaw, fp32 *add_pitch, gimbal_control_t *gimbal_control_set);
//------------------------------------------------------------------------------


//重要变量------------------------------------------------------------------------
//云台行为状态机
gimbal_behaviour_e gimbal_behaviour = GIMBAL_ZERO_FORCE;

bool_t AUTO_ATTACK=0;
bool_t TURN_ROUND=0;
//------------------------------------------------------------------------------


/**
 * @brief          云台行为状态机设置.
 * @param[in]      gimbal_mode_set: 云台数据指针
 * @retval         none
 */
static void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }

    gimbal_behaviour=Get_gimbal_behaviour_mode();

}

/**
  * @brief          gimbal_set_mode函数调用在gimbal_task.c,云台行为状态机以及电机状态机设置
  * @param[out]     gimbal_mode_set: 云台数据指针
  * @retval         none
  */

void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }

    //云台行为状态机设置
    gimbal_behavour_set(gimbal_mode_set);

    // 根据云台行为状态机设置电机状态机
    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        gimbal_mode_set->gimbal_yaw_j4310.gimbal_motor_mode = GIMBAL_MOTOR_ZERO_FORCE_CONTROL;
        gimbal_mode_set->gimbal_pitch_j4340.gimbal_motor_mode = GIMBAL_MOTOR_ZERO_FORCE_CONTROL;
    }
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
    {
        gimbal_mode_set->gimbal_yaw_j4310.gimbal_motor_mode = GIMBAL_MOTOR_ABSOLUTE_ANGLE_CONTROL;
        gimbal_mode_set->gimbal_pitch_j4340.gimbal_motor_mode = GIMBAL_MOTOR_ABSOLUTE_ANGLE_CONTROL;
    }
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)
    {
        gimbal_mode_set->gimbal_yaw_j4310.gimbal_motor_mode = GIMBAL_MOTOR_ZERO_FORCE_CONTROL;
        gimbal_mode_set->gimbal_pitch_j4340.gimbal_motor_mode = GIMBAL_MOTOR_ZERO_FORCE_CONTROL;
    }
    else if (gimbal_behaviour == GIMBAL_AUTO_ATTACK)
    {
        gimbal_mode_set->gimbal_yaw_j4310.gimbal_motor_mode = GIMBAL_MOTOR_AUTO_CONTROL;
        gimbal_mode_set->gimbal_pitch_j4340.gimbal_motor_mode = GIMBAL_MOTOR_AUTO_CONTROL;
    }
    else if (gimbal_behaviour == GIMBAL_REMOTE_FIRE)
    {
        gimbal_mode_set->gimbal_yaw_j4310.gimbal_motor_mode = GIMBAL_MOTOR_REMOTE_FIRE_CONTROL;
        gimbal_mode_set->gimbal_pitch_j4340.gimbal_motor_mode = GIMBAL_MOTOR_REMOTE_FIRE_CONTROL;
    }
    else if (gimbal_behaviour == GIMBAL_TURN_ROUND)
    {
        gimbal_mode_set->gimbal_yaw_j4310.gimbal_motor_mode = GIMBAL_MOTOR_TURN_ROUND_CONTROL;
        gimbal_mode_set->gimbal_pitch_j4340.gimbal_motor_mode = GIMBAL_MOTOR_ABSOLUTE_ANGLE_CONTROL;
    }
    else if (gimbal_behaviour == GIMBAL_INIT)
    {
        gimbal_mode_set->gimbal_yaw_j4310.gimbal_motor_mode = GIMBAL_MOTOR_INIT_CONTROL;
        gimbal_mode_set->gimbal_pitch_j4340.gimbal_motor_mode = GIMBAL_MOTOR_INIT_CONTROL;
    }
}


void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, gimbal_control_t *gimbal_control_set)
{
    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        gimbal_zero_force_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
    {
        gimbal_absolute_angle_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)
    {
        gimbal_relative_angle_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_AUTO_ATTACK)
    {
        gimbal_auto_attack_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_REMOTE_FIRE)
    {
        gimbal_remote_fire_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_TURN_ROUND)
    {
        gimbal_turn_round_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_INIT)
    {
        gimbal_init_control(add_yaw, add_pitch, gimbal_control_set);
    }
}

/**
 * @brief          当云台行为模式是GIMBAL_ZERO_FORCE, 这个函数会被调用,云台控制模式是raw模式.原始模式意味着
 *                 设定值会直接发送到CAN总线上,这个函数将会设置所有为0.
 * @param[in]      add_yaw:发送yaw电机的原始值，会直接通过can 发送到电机
 * @param[in]      add_pitch:发送pitch电机的原始值，会直接通过can 发送到电机
 * @param[in]      gimbal_control_set: 云台数据指针
 * @retval         none
 */
static void gimbal_zero_force_control(fp32 *add_yaw, fp32 *add_pitch, gimbal_control_t *gimbal_control_set)
{
    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    *add_yaw = 0.0f;
    *add_pitch = 0.0f;
}

float filter_fast_change(float input)
{
    // 静态变量：保存上一次输出（滤波记忆）
    static float last_output = 0.0f;

    // 滤波系数（0~1）
    // 越小 → 滤波越强，越不怕突变，但反应越慢
    // 越大 → 滤波越弱，反应越快，但抗抖动差
    const float filter_coef = 0.8f;

    // 核心滤波公式
    float output = last_output + filter_coef * (input - last_output);

    // 保存本次结果，给下一次用
    last_output = output;

    return output;
}

/**
 * @brief          云台陀螺仪控制，电机是陀螺仪角度控制，
 * @param[out]     add_yaw: yaw轴角度控制，为角度的增量 单位 rad
 * @param[out]     add_pitch:pitch轴角度控制，为角度的增量 单位 rad
 * @param[in]      gimbal_control_set:云台数据指针
 * @retval         none
 */
static void gimbal_absolute_angle_control(fp32 *add_yaw, fp32 *add_pitch, gimbal_control_t *gimbal_control_set)
{
    static uint16_t yaw_step_count        = 0;  // 已拨动次数，每次 +1 对应 +90°
    static uint16_t last_fusi_up_to_down = 0;
    static float yaw_channel = 0, pitch_channel = 0;
    static float mouse_yaw_channel=0;

    rc_deadband_limit(RC_Control.move_channel.Left_X, yaw_channel, RC_DEADBAND);
    rc_deadband_limit(RC_Control.move_channel.Right_Y, pitch_channel, RC_DEADBAND);

    mouse_yaw_channel=RC_Control.key_mouse.Mouse_X * 32768;

    if ( abs(mouse_yaw_channel) > (10) )
    {
        mouse_yaw_channel = mouse_yaw_channel * 1.0;
    }
	else if ( abs(mouse_yaw_channel) > (8) && abs(mouse_yaw_channel) <= (10) )
	{
		mouse_yaw_channel = mouse_yaw_channel * 1.2;
	}
	else if ( abs(mouse_yaw_channel) > (6) && abs(mouse_yaw_channel) <= (8) )
	{
		mouse_yaw_channel = mouse_yaw_channel * 1.4;
	}
	else if ( abs(mouse_yaw_channel) > (4) && abs(mouse_yaw_channel) <= (6) )
	{
		mouse_yaw_channel = mouse_yaw_channel * 1.6;
	}
	else if ( abs(mouse_yaw_channel) > (2) && abs(mouse_yaw_channel) <= (4) )
	{
		mouse_yaw_channel = mouse_yaw_channel * 1.8;
	}
	else if ( abs(mouse_yaw_channel) <= (2) )
	{
		mouse_yaw_channel = mouse_yaw_channel * 2.0;
	}

    if ( abs(mouse_yaw_channel) > (1200) )
    {
        mouse_yaw_channel = 0;
    }
    else if ( abs(mouse_yaw_channel) > (100) && abs(mouse_yaw_channel) <= (1200) )
    {
        mouse_yaw_channel = filter_fast_change(mouse_yaw_channel);
    }
    else if ( abs(mouse_yaw_channel) < (10) )
    {

    }

    *add_yaw   = (yaw_channel * Rocker_Yaw_Scale) + (mouse_yaw_channel * Key_Mouse_Yaw_Scale /32768.0f);
    *add_pitch = (pitch_channel * Rocker_Pitch_Scale) + (RC_Control.key_mouse.Mouse_Y * Key_Mouse_Pitch_Scale);

    // if (fusi_up_to_down != last_fusi_up_to_down)
    // {
    //     uint16_t step = (uint16_t)(fusi_up_to_down - last_fusi_up_to_down);  // 本次新增的拨动次数
    //     last_fusi_up_to_down = fusi_up_to_down;
    //     *add_yaw += (float)step * (PI / 4.0f);
    // }

}

static void gimbal_relative_angle_control(fp32 *add_yaw, fp32 *add_pitch, gimbal_control_t *gimbal_control_set)
{
    *add_yaw = 0 ;
    *add_pitch = 0;
}

static void gimbal_auto_attack_control(fp32 *add_yaw, fp32 *add_pitch, gimbal_control_t *gimbal_control_set)
{
    // yaw pitch 轴设定值与当前值的差值
    fp32 pitch_error = 0;
    fp32 yaw_error = 0;

    // pitch轴yaw轴设定角度
    fp32 pitch_set_angle = 0;
    fp32 yaw_set_angle = 0;


    pitch_set_angle = gimbal_control_set->gimbal_vision_point->gimbal_pitch;
    yaw_set_angle = gimbal_control_set->gimbal_vision_point->gimbal_yaw;

    // 计算过去设定角度与当前角度之间的差值
    yaw_error = gimbal_control_set->gimbal_yaw_j4310.dm_normal_motor.Target_Angle - gimbal_control_set->gimbal_INS_point->Yaw;

    pitch_error = gimbal_control_set->gimbal_pitch_j4340.dm_normal_motor.Target_Angle - (-gimbal_control_set->gimbal_INS_point->Pitch);
    //  获取上位机视觉数据

    // 赋值增量
    if (yaw_set_angle && pitch_set_angle )
    {
        *add_yaw = yaw_set_angle - gimbal_control_set->gimbal_INS_point->Yaw - yaw_error;
        *add_pitch = ( pitch_set_angle - (-gimbal_control_set->gimbal_INS_point->Pitch) ) - pitch_error;
    }
    else
    {
        *add_yaw = 0.0f;
        *add_pitch = 0.0f;
    }
}

/**
 * @brief          云台吊射速度环控制
 * @param[out]     yaw: yaw轴角度控制，为角度的增量 单位 rad
 * @param[out]     pitch:pitch轴速度控制,为速度设定值
 * @param[in]      gimbal_control_set:云台数据指针
 * @retval         none
 */
static void gimbal_remote_fire_control(fp32 *add_yaw, fp32 *add_pitch, gimbal_control_t *gimbal_control_set)
{
    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    *add_yaw = 0.0f;
    *add_pitch = 0.0f;

}

static void gimbal_turn_round_control(fp32 *add_yaw, fp32 *add_pitch, gimbal_control_t *gimbal_control_set)
{
    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    static float pitch_channel = 0;

    rc_deadband_limit(RC_Control.move_channel.Right_Y, pitch_channel, RC_DEADBAND);

    *add_yaw = 0.0f;
    *add_pitch = (pitch_channel * Rocker_Pitch_Scale) + (RC_Control.key_mouse.Mouse_Y * Key_Mouse_Pitch_Scale);
}

static void gimbal_init_control(fp32 *add_yaw, fp32 *add_pitch, gimbal_control_t *gimbal_control_set)
{
    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    *add_yaw = 0.0f;
    *add_pitch = 0.0f;

}