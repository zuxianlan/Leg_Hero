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

//AI生成
 fp32  pitch_target    = 0.0f;   // pitch 目标累加器（逻辑位置，允许超限，由反弹拉回）
 int8_t pitch_move_dir = 1;      // 1: 往上加   -1: 往下减
static uint8_t pitch_step_budget = 0;  // 待走的格数，每 1ms 控制周期最多消费 1 格
// 一格 = 朝当前方向走 PITCH_STEP_RAD
// 规则：下一步仍落在 [PITCH_LIMIT_MIN, PITCH_LIMIT_MAX] 内 -> 继续走；
//       下一步会出界 -> 反向走一格（不贴墙、不留残量）
// 效果(起点0, 步长0.2)：0 -> 0.2 -> 0.4 -> 0.2 -> 0 -> -0.2 -> 0 -> ... 周期6格
//       0.55 与 -0.3 都不会被触及，实际往返区间为 [-0.2, 0.4]
static void Pitch_Bounce_Step(void)
{
    while (pitch_step_budget > 0)
    {
        pitch_step_budget--;

        if (pitch_move_dir > 0)
        {
            if ((pitch_target + PITCH_STEP_RAD) <= PITCH_LIMIT_MAX)
            {
                // 下一步仍在界内，继续往上
                pitch_target += PITCH_STEP_RAD;
            }
            else
            {
                // 下一步出界，反向向下走一格
                pitch_move_dir = -1;
                pitch_target -= PITCH_STEP_RAD;
            }
        }
        else
        {
            if ((pitch_target - PITCH_STEP_RAD) >= PITCH_LIMIT_MIN)
            {
                // 下一步仍在界内，继续往下
                pitch_target -= PITCH_STEP_RAD;
            }
            else
            {
                // 下一步出界，反向向上走一格
                pitch_move_dir = 1;
                pitch_target += PITCH_STEP_RAD;
            }
        }
    }
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
    //     *add_yaw += (float)step * PI/4.0f;
    // }
    if (fusi_up_to_down != last_fusi_up_to_down)
    {
        uint16_t step = (uint16_t)(fusi_up_to_down - last_fusi_up_to_down);  // 本次新增的拨动次数
        last_fusi_up_to_down = fusi_up_to_down;

        pitch_step_budget += (step > 1) ? 1 : (uint8_t)step;  // 按住连翻也不会一帧跳很多格

        Pitch_Bounce_Step();          // 累加器按格步进并在限位处反弹
    }

    // 拨杆/鼠标摇杆的连续量照旧，位置由下面的反弹目标给绝对量补回来
    *add_pitch += (pitch_target - gimbal_control_set->gimbal_pitch_j4340.dm_normal_motor.Target_Angle);
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