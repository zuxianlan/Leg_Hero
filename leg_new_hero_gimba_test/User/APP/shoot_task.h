#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H

#include "struct_typedef.h"
#include "user_lib.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "user_lib.h"

#include "pid.h"
#include "dvc_referee.h"
#include "gimbal_task.h"
#include "vision_task.h"
#include "gimbal_behaviour.h"
#include "RC_Task.h"
#include "dvc_dr16.h"
#include "dvc_vt13_rc.h"
#include "dvc_fusi_rc.h"

//宏定义-------------------------------------------------------------------
//发射任务延时时间 1ms
#define SHOOT_TASK_DELAY_TIME 1
// 发射任务时间转化 秒转毫秒
#define SHOOT_TASK_S_TO_MS(x) ((int32_t)((x * 1p000.0f) / (SHOOT_TASK_DELAY_TIME)))
//发射任务最大时间，以秒为单位 20 s
#define SHOOT_TASK_MAX_INIT_TIME 10
//------------------------------------------------------------------------------

//补偿系数
#define  Accel_Feedforward_Coeff   0.008f    //加速度前馈补偿系数
#define  Diff_Feedforward_Coeff    1.0f     //差分前馈补偿系数


//电机PID参数------------------------------------------------------------------------
//拨弹盘4310电机PID
//角度环
#define Trigger_MOTOR_4310_Angle_PID_KP 30.0f//20.0//25//30//35//15
#define Trigger_MOTOR_4310_Angle_PID_KI 1.0f
#define Trigger_MOTOR_4310_Angle_PID_KD 0.0f
#define Trigger_MOTOR_4310_Angle_PID_KF 0.0f
#define Trigger_MOTOR_4310_Angle_PID_MAX_IOUT (1.0f * PI)
#define Trigger_MOTOR_4310_Angle_PID_MAX_OUT (6.0f * PI)
#define Trigger_MOTOR_4310_Angle_PID_DEAD_ZONE 0.0f
#define Trigger_MOTOR_4310_Angle_I_Variable_Speed_A 0.5f
#define Trigger_MOTOR_4310_Angle_I_Variable_Speed_B 0.001f
#define Trigger_MOTOR_4310_Angle_I_Separate_Threshold 0.0f
//速度环
#define Trigger_MOTOR_4310_Speed_PID_KP 0.5f//0.5
#define Trigger_MOTOR_4310_Speed_PID_KI 0.0f
#define Trigger_MOTOR_4310_Speed_PID_KD 0.0f
#define Trigger_MOTOR_4310_Speed_PID_KF 0.0f
#define Trigger_MOTOR_4310_Speed_PID_MAX_IOUT 0.0f
#define Trigger_MOTOR_4310_Speed_PID_MAX_OUT 10.0f
#define Trigger_MOTOR_4310_Speed_PID_DEAD_ZONE 0.0f
#define Trigger_MOTOR_4310_Speed_I_Variable_Speed_A 0.0f
#define Trigger_MOTOR_4310_Speed_I_Variable_Speed_B 0.0f
#define Trigger_MOTOR_4310_Speed_I_Separate_Threshold 0.0f

//拨弹盘单纯速度环
#define Trigger_MOTOR_4310_Only_Speed_PID_KP 0.4f//0.6
#define Trigger_MOTOR_4310_Only_Speed_PID_KI 0.0f
#define Trigger_MOTOR_4310_Only_Speed_PID_KD 0.0f
#define Trigger_MOTOR_4310_Only_Speed_PID_KF 0.0f
#define Trigger_MOTOR_4310_Only_Speed_PID_MAX_IOUT 0.0f
#define Trigger_MOTOR_4310_Only_Speed_PID_MAX_OUT 7.0f
#define Trigger_MOTOR_4310_Only_Speed_PID_DEAD_ZONE 0.0f
#define Trigger_MOTOR_4310_Only_Speed_I_Variable_Speed_A 0.0f
#define Trigger_MOTOR_4310_Only_Speed_I_Variable_Speed_B 0.0f
#define Trigger_MOTOR_4310_Only_Speed_I_Separate_Threshold 0.0f


//摩擦轮3508电机PID
//rpm--------------------------------------------------------
//左摩擦轮PID
#define Fric_Left_MOTOR_Speed_PID_KP 6.5f//6.5
#define Fric_Left_MOTOR_Speed_PID_KI 0.0f
#define Fric_Left_MOTOR_Speed_PID_KD 0.0f
#define Fric_Left_MOTOR_Speed_PID_KF 0.0f
#define Fric_Left_MOTOR_Speed_PID_MAX_IOUT 0.0f
#define Fric_Left_MOTOR_Speed_PID_MAX_OUT 5000.0f//5000
#define Fric_Left_MOTOR_Speed_PID_DEAD_ZONE 0.0f
#define Fric_Left_MOTOR_Speed_I_Variable_Speed_A 0.0f
#define Fric_Left_MOTOR_Speed_I_Variable_Speed_B 0.0f
#define Fric_Left_MOTOR_Speed_I_Separate_Threshold 0.0f
//右摩擦轮PID
#define Fric_Right_MOTOR_Speed_PID_KP 6.5f
#define Fric_Right_MOTOR_Speed_PID_KI 0.0f
#define Fric_Right_MOTOR_Speed_PID_KD 0.0f
#define Fric_Right_MOTOR_Speed_PID_KF 0.0f
#define Fric_Right_MOTOR_Speed_PID_MAX_IOUT 0.0f
#define Fric_Right_MOTOR_Speed_PID_MAX_OUT 5000.0f
#define Fric_Right_MOTOR_Speed_PID_DEAD_ZONE 0.0f
#define Fric_Right_MOTOR_Speed_I_Variable_Speed_A 0.0f
#define Fric_Right_MOTOR_Speed_I_Variable_Speed_B 0.0f
#define Fric_Right_MOTOR_Speed_I_Separate_Threshold 0.0f
//-----------------------------------------------------------


//特殊机制******************************************************
//左摩擦轮角度PID
#define Fric_Left_MOTOR_Angle_PID_KP 30.0f//20.0//25//30//35//15
#define Fric_Left_MOTOR_Angle_PID_KI 0.0f
#define Fric_Left_MOTOR_Angle_PID_KD 0.0f
#define Fric_Left_MOTOR_Angle_PID_KF 0.0f
#define Fric_Left_MOTOR_Angle_PID_MAX_IOUT (6.0f * PI)
#define Fric_Left_MOTOR_Angle_PID_MAX_OUT (20.0f * PI)
#define Fric_Left_MOTOR_Angle_PID_DEAD_ZONE 0.0f
#define Fric_Left_MOTOR_Angle_I_Variable_Speed_A 0.0f
#define Fric_Left_MOTOR_Angle_I_Variable_Speed_B 0.0f
#define Fric_Left_MOTOR_Angle_I_Separate_Threshold 0.0f
//右摩擦轮角度PID
#define Fric_Right_MOTOR_Angle_PID_KP 30.0f//20.0//25//30//35//15
#define Fric_Right_MOTOR_Angle_PID_KI 0.0f
#define Fric_Right_MOTOR_Angle_PID_KD 0.0f
#define Fric_Right_MOTOR_Angle_PID_KF 0.0f
#define Fric_Right_MOTOR_Angle_PID_MAX_IOUT (6.0f * PI)
#define Fric_Right_MOTOR_Angle_PID_MAX_OUT (20.0f * PI)
#define Fric_Right_MOTOR_Angle_PID_DEAD_ZONE 0.0f
#define Fric_Right_MOTOR_Angle_I_Variable_Speed_A 0.0f
#define Fric_Right_MOTOR_Angle_I_Variable_Speed_B 0.0f
#define Fric_Right_MOTOR_Angle_I_Separate_Threshold 0.0f

//左摩擦轮角速度PID
#define Fric_Left_MOTOR_Omega_PID_KP 200.0f//0.5
#define Fric_Left_MOTOR_Omega_PID_KI 0.0f
#define Fric_Left_MOTOR_Omega_PID_KD 0.0f
#define Fric_Left_MOTOR_Omega_PID_KF 0.0f
#define Fric_Left_MOTOR_Omega_PID_MAX_IOUT 0.0f
#define Fric_Left_MOTOR_Omega_PID_MAX_OUT 7000.0f
#define Fric_Left_MOTOR_Omega_PID_DEAD_ZONE 0.0f
#define Fric_Left_MOTOR_Omega_I_Variable_Speed_A 0.0f
#define Fric_Left_MOTOR_Omega_I_Variable_Speed_B 0.0f
#define Fric_Left_MOTOR_Omega_I_Separate_Threshold 0.0f
//右摩擦轮角速度PID
#define Fric_Right_MOTOR_Omega_PID_KP 200.0f//0.5
#define Fric_Right_MOTOR_Omega_PID_KI 0.0f
#define Fric_Right_MOTOR_Omega_PID_KD 0.0f
#define Fric_Right_MOTOR_Omega_PID_KF 0.0f
#define Fric_Right_MOTOR_Omega_PID_MAX_IOUT 0.0f
#define Fric_Right_MOTOR_Omega_PID_MAX_OUT 7000.0f
#define Fric_Right_MOTOR_Omega_PID_DEAD_ZONE 0.0f
#define Fric_Right_MOTOR_Omega_I_Variable_Speed_A 0.0f
#define Fric_Right_MOTOR_Omega_I_Variable_Speed_B 0.0f
#define Fric_Right_MOTOR_Omega_I_Separate_Threshold 0.0f
//**************************************************************
//------------------------------------------------------------------------------


//结构体、枚举--------------------------------------------------------------------
//电机控制模式
typedef enum
{
    SHOOT_MOTOR_DISABLE = 0,
    SHOOT_MOTOR_ENABLE,
    SHOOT_MOTOR_DEBUG_,
    SHOOT_MOTOR_RC_GONGDAN,
    SHOOT_MOTOR_RC_HUIBO,
    SHOOT_MOTOR_MOUSE_GONGDAN,
    SHOOT_MOTOR_MOUSE_HUIBO,
} shoot_motor_mode_e;

typedef struct
{
    Motor_DM_Normal dm_normal_motor;
    Motor_DM_1_To_4 dm_1_To_4_motor;
    Motor_GM6020 GM6020_motor;
    Motor_C620 C620_motor;

    Enum_Motor_Status motor_status;
    FSM_t Motor_FSM;

}shoot_motor_t;

typedef struct
{
    //遥控器数据指针
    const DR16_Processed_Data_t *shoot_dr16_processed_data;
    const VT13_Processed_Data_t *shoot_vt13_processed_data;
    const Fusi_RC_Processed_Data_t *shoot_fusi_rc_processed_data;

    //视觉控制指针
    const shoot_vision_control_t *shoot_vision_control;

    //发射机构电机
    shoot_motor_t shoot_trigger_j4310;
    shoot_motor_t fric_left_3508;
    shoot_motor_t fric_right_3508;

    //发射机构电机控制模式--------------
    shoot_motor_mode_e trigger_mode;
    shoot_motor_mode_e last_trigger_mode;

    shoot_motor_mode_e fric_mode;
    shoot_motor_mode_e last_fric_mode;
    //------------------------------

    //摩擦轮rpm转速设定值
    int16_t fric_left_rpm_set;
    int16_t fric_right_rpm_set;

    //摩擦轮转速设定值的平均值
    fp32 fric_speed_set_average;
    //摩擦轮转速的平均值
    fp32 fric_speed_average;

    //判断摩擦轮开没开标志位
    uint8_t fric_open_flag;

    //已经打出弹丸的数量
    bool_t allow_fire_flag;
    int32_t already_bullet_counts;
    int32_t last_already_bullet_counts;
    TickType_t bullet_fire_count_time;

    //热量允许标志位
    uint8_t heat_allow_fire_flag;
    uint8_t heat_delay_allow_flag;//为防止裁判系统延迟所作

    bool_t shoot_flag;          //发射标志位
    bool_t last_shoot_flag;     //上次发射标志位

    bool_t rc_huibo_flag;       //回拨标志位
    bool_t last_rc_huibo_flag;  //上次回拨标志位

    bool_t rc_gongdan_flag;     //供弹标志位
    bool_t last_gongdan_flag;   //上次供弹标志位

    //发射系统时间
    TickType_t SHOOT_xTickCount;
    //按下回拨键的时间
    TickType_t trigger_mouse_huibo_start_time;
    //发弹后强制延迟一秒的时间
    TickType_t fire_force_delay_start_time;

} shoot_control_t;
//------------------------------------------------------------------------------


//外部交互数据-------------------------------------------------------------------
//射击控制视觉任务
bool_t shoot_control_vision_task(void);

extern bool_t mouse_huibo_flag ;//手动开启卡弹回拨标志位
extern bool_t last_mouse_huibo_flag ;

extern bool_t mouse_gongdan_flag ;//手动开启强制供弹标志位
extern bool_t last_mouse_gongdan_flag ;

extern bool_t  R_Fric;
extern shoot_control_t shoot_control;
//------------------------------------------------------------------------------

#endif
