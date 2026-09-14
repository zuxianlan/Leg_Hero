#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "main.h"
#include "cmsis_os2.h"
#include "arm_math.h"
#include "struct_typedef.h"
#include "user_lib.h"
#include "Mathh.h"
#include "FreeRTOS.h"
#include "task.h"


#include "PID_Control.h"
#include "ins_task.h"
#include "motor_dm.h"
#include "motor_dji.h"
#include "vision_task.h"
#include "CAN_comm.h"
#include "fsm.h"
#include "pid.h"
#include "second_order_linear_controller.h"
#include "RC_Task.h"
#include "dvc_dr16.h"
#include "dvc_vt13_rc.h"
#include "dvc_fusi_rc.h"
#include "leso_pitch.h"


//宏定义-------------------------------------------------------------------
#define RAD_TO_DEG (180.0 / M_PI)
//任务初始化 空闲一段时间
#define GIMBAL_TASK_INIT_TIME 400

#define rc_deadband_limit(input, output, dealine)       \
{                                                       \
    if ((input) > (dealine) || (input) < -(dealine))    \
    {                                                   \
        (output) = (input);                             \
    }                                                   \
    else                                                \
    {                                                   \
        (output) = 0;                                   \
    }                                                   \
}

//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_DEADBAND   0.035

//遥控器和键鼠控制缩放比例-------------------
#define Rocker_Yaw_Scale   (-0.0007f * PI)//(-0.0001f * PI) //(-0.0007f * PI)
#define Rocker_Pitch_Scale (0.0005f * PI)//(0.0001f * PI)  //(0.0003f * PI)

#define Key_Mouse_Yaw_Scale (-0.12f * PI)//-0.055
#define Key_Mouse_Pitch_Scale (0.055f * PI)
//--------------------------------------

//重力补偿
// 重力补偿结构体
typedef struct {
    float m;        // 电机质量 (kg)
    float g;        // 重力加速度 (m/s^2) //9.8
    float arm_len;  // 力臂长 (m)
    float phase;    // 相位偏置 (rad)，通常为 0
    float K;        // 各种增益叠合
} Gravity_Comp_t;


//LQR控制增益矩阵----------------------------------------------------------------
// YAW轴LQR增益矩阵
#define YAW_LQR_K1    9.4868//9.4868
#define YAW_LQR_K2    0.7728//0.7135


// PITCH轴LQR增益矩阵
#define PITCH_LQR_K1    33.1662
#define PITCH_LQR_K2    1.5768

//-----------------------------------------------------------------------------



//云台电机二阶控制器（马头）--------------------------------------------------------
//线性控制器前馈系数
#define YAW_FEED_FORWARD 0.0f
#define PITCH_FEED_FORWARD 0.0f
//角度误差项系数
#define K_YAW_ANGLE_ERROR 120000.0f//186000.0f
#define K_PITCH_ANGLE_ERROR 50000.0f//182000.0f
//速度项系数
#define K_YAW_ANGLE_SPEED 6000.0f//24000.0f
#define K_PITCH_ANGLE_SPEED 1000.0f//9600.0f
//最大最小输出
#define YAW_MAX_OUT 5000.0f
#define YAW_MIX_OUT -5000.0f
#define PITCH_MAX_OUT 5000.0f
#define PITCH_MIX_OUT -5000.0f
//------------------------------------------------------------------------------


//电机PID-------------------------------------------------------------------

//mit电机控制模式下的PID-----------------------------------------------
//Yaw轴4310电机PID
//角度环PID()
#define Yaw_4310_Angle_PID_KP                    24.0f//27
#define Yaw_4310_Angle_PID_KI                    0.1f
#define Yaw_4310_Angle_PID_KD                    0.0f
#define Yaw_4310_Angle_PID_KF                    0.0f
#define Yaw_4310_Angle_PID_MAX_IOUT           	 0.5f
#define Yaw_4310_Angle_PID_MAX_OUT             	 (2.0 * PI)
#define Yaw_4310_Angle_PID_DEAD_ZONE           	 0.0f
#define Yaw_4310_Angle_I_Variable_Speed_A        0.5f
#define Yaw_4310_Angle_I_Variable_Speed_B        0.01f
#define Yaw_4310_Angle_I_Separate_Threshold      0.0f
//速度环PID
#define Yaw_4310_Speed_PID_KP                   2.2f//3
#define Yaw_4310_Speed_PID_KI                   0.0f
#define Yaw_4310_Speed_PID_KD                   0.0f//0
#define Yaw_4310_Speed_PID_KF                   0.0f
#define Yaw_4310_Speed_PID_MAX_IOUT             0.0f
#define Yaw_4310_Speed_PID_MAX_OUT              2.5f
#define Yaw_4310_Speed_PID_DEAD_ZONE            0.0f
#define Yaw_4310_Speed_I_Variable_Speed_A       0.0f
#define Yaw_4310_Speed_I_Variable_Speed_B       0.0f
#define Yaw_4310_Speed_I_Separate_Threshold     0.0f


// //Pitch轴4340电机PID
// //角度环PID
// #define Pitch_4340_Angle_PID_KP                        25.8f//26
// #define Pitch_4340_Angle_PID_KI                        2.0f
// #define Pitch_4340_Angle_PID_KD                        0.0f
// #define Pitch_4340_Angle_PID_KF                        0.0f
// #define Pitch_4340_Angle_PID_MAX_IOUT             	   1.0
// #define Pitch_4340_Angle_PID_MAX_OUT               	(1 * PI)
// #define Pitch_4340_Angle_PID_DEAD_ZONE             	 0.0f
// #define Pitch_4340_Angle_I_Variable_Speed_A            0.10f
// #define Pitch_4340_Angle_I_Variable_Speed_B            0.07f
// #define Pitch_4340_Angle_I_Separate_Threshold          0.0f
// //速度环PID
// #define Pitch_4340_Speed_PID_KP                      0.6f//0.78
// #define Pitch_4340_Speed_PID_KI                      0.0f
// #define Pitch_4340_Speed_PID_KD                      0.0f
// #define Pitch_4340_Speed_PID_KF                      0.0f
// #define Pitch_4340_Speed_PID_MAX_IOUT                0.0f
// #define Pitch_4340_Speed_PID_MAX_OUT              2.5f//8
// #define Pitch_4340_Speed_PID_DEAD_ZONE             0.0f
// #define Pitch_4340_Speed_I_Variable_Speed_A          0.0f
// #define Pitch_4340_Speed_I_Variable_Speed_B          0.0f
// #define Pitch_4340_Speed_I_Separate_Threshold        0.0f


//Pitch轴4340电机PID（超抗参数）
//角度环PID
// #define Pitch_4340_Angle_PID_KP                        28.4f//26
// #define Pitch_4340_Angle_PID_KI                        4.0f
// #define Pitch_4340_Angle_PID_KD                        0.0f
// #define Pitch_4340_Angle_PID_KF                        0.0f
// #define Pitch_4340_Angle_PID_MAX_IOUT             	   1.3
// #define Pitch_4340_Angle_PID_MAX_OUT               	(1 * PI)
// #define Pitch_4340_Angle_PID_DEAD_ZONE             	 0.0f
// #define Pitch_4340_Angle_I_Variable_Speed_A            0.5f
// #define Pitch_4340_Angle_I_Variable_Speed_B            0.0001f
// #define Pitch_4340_Angle_I_Separate_Threshold          0.0f
// //速度环PID
// #define Pitch_4340_Speed_PID_KP                      1.15f//1.0
// #define Pitch_4340_Speed_PID_KI                      0.0f
// #define Pitch_4340_Speed_PID_KD                      0.0f
// #define Pitch_4340_Speed_PID_KF                      0.0f
// #define Pitch_4340_Speed_PID_MAX_IOUT                0.0f
// #define Pitch_4340_Speed_PID_MAX_OUT              6.0f//8
// #define Pitch_4340_Speed_PID_DEAD_ZONE             0.0f
// #define Pitch_4340_Speed_I_Variable_Speed_A          0.0f
// #define Pitch_4340_Speed_I_Variable_Speed_B          0.0f
// #define Pitch_4340_Speed_I_Separate_Threshold        0.0f


#define Pitch_4340_Angle_PID_KP                        24//28.4f
#define Pitch_4340_Angle_PID_KI                        3.5f
#define Pitch_4340_Angle_PID_KD                        0.0f
#define Pitch_4340_Angle_PID_KF                        0.0f
#define Pitch_4340_Angle_PID_MAX_IOUT             	   1.3
#define Pitch_4340_Angle_PID_MAX_OUT               	(1 * PI)
#define Pitch_4340_Angle_PID_DEAD_ZONE             	 0.0f
#define Pitch_4340_Angle_I_Variable_Speed_A            0.5f
#define Pitch_4340_Angle_I_Variable_Speed_B            0.0001f
#define Pitch_4340_Angle_I_Separate_Threshold          0.0f
//速度环PID
#define Pitch_4340_Speed_PID_KP                      1.15f//1.0
#define Pitch_4340_Speed_PID_KI                      0.0f
#define Pitch_4340_Speed_PID_KD                      0.0f
#define Pitch_4340_Speed_PID_KF                      0.0f
#define Pitch_4340_Speed_PID_MAX_IOUT                0.0f
#define Pitch_4340_Speed_PID_MAX_OUT              6.0f//8
#define Pitch_4340_Speed_PID_DEAD_ZONE             0.0f
#define Pitch_4340_Speed_I_Variable_Speed_A          0.0f
#define Pitch_4340_Speed_I_Variable_Speed_B          0.0f
#define Pitch_4340_Speed_I_Separate_Threshold        0.0f




//init模式下的PID--------------------------------------------------------
//Yaw轴4310电机PID
//角度环PID()
#define Yaw_4310_Init_Angle_PID_KP                    20.0f
#define Yaw_4310_Init_Angle_PID_KI                    0.0f
#define Yaw_4310_Init_Angle_PID_KD                    0.0f
#define Yaw_4310_Init_Angle_PID_KF                    0.0f
#define Yaw_4310_Init_Angle_PID_MAX_IOUT           	  0.0f
#define Yaw_4310_Init_Angle_PID_MAX_OUT               ( 1.0 * PI)
#define Yaw_4310_Init_Angle_PID_DEAD_ZONE             0.0f
#define Yaw_4310_Init_Angle_I_Variable_Speed_A        0.5f
#define Yaw_4310_Init_Angle_I_Variable_Speed_B        0.01f
#define Yaw_4310_Init_Angle_I_Separate_Threshold      0.0f
//速度环PID
#define Yaw_4310_Init_Speed_PID_KP                   1.3f
#define Yaw_4310_Init_Speed_PID_KI                   0.0f
#define Yaw_4310_Init_Speed_PID_KD                   0.0f
#define Yaw_4310_Init_Speed_PID_KF                   0.0f
#define Yaw_4310_Init_Speed_PID_MAX_IOUT            0.0f
#define Yaw_4310_Init_Speed_PID_MAX_OUT              4.0f
#define Yaw_4310_Init_Speed_PID_DEAD_ZONE            0.0f
#define Yaw_4310_Init_Speed_I_Variable_Speed_A       0.0f
#define Yaw_4310_Init_Speed_I_Variable_Speed_B       0.0f
#define Yaw_4310_Init_Speed_I_Separate_Threshold     0.0f

//Pitch轴4340电机PID
//角度环PID
#define Pitch_4340_Init_Angle_PID_KP                        14.0f
#define Pitch_4340_Init_Angle_PID_KI                        0.0f
#define Pitch_4340_Init_Angle_PID_KD                        0.0f
#define Pitch_4340_Init_Angle_PID_KF                        0.0f
#define Pitch_4340_Init_Angle_PID_MAX_IOUT             	    0.0f
#define Pitch_4340_Init_Angle_PID_MAX_OUT               	(0.5 * PI)
#define Pitch_4340_Init_Angle_PID_DEAD_ZONE             	 0.0f
#define Pitch_4340_Init_Angle_I_Variable_Speed_A            (PI / 8.0f)
#define Pitch_4340_Init_Angle_I_Variable_Speed_B            0.001f
#define Pitch_4340_Init_Angle_I_Separate_Threshold          0.0f
//速度环PID
#define Pitch_4340_Init_Speed_PID_KP                      0.07f
#define Pitch_4340_Init_Speed_PID_KI                      0.0f
#define Pitch_4340_Init_Speed_PID_KD                      0.0f
#define Pitch_4340_Init_Speed_PID_KF                      0.0f
#define Pitch_4340_Init_Speed_PID_MAX_IOUT                0.0f
#define Pitch_4340_Init_Speed_PID_MAX_OUT               8.0f
#define Pitch_4340_Init_Speed_PID_DEAD_ZONE             0.0f
#define Pitch_4340_Init_Speed_I_Variable_Speed_A          0.0f
#define Pitch_4340_Init_Speed_I_Variable_Speed_B          0.0f
#define Pitch_4340_Init_Speed_I_Separate_Threshold        0.0f
//------------------------------------------------------------------------------



//结构体、枚举---------------------------------------------------------------------
typedef enum
{
    INIT_FINISH=0,
    INIT_ING,
    INIT_START,
    NEED_INIT,
}gimbal_init_fsm_mod;

typedef enum
{
    GIMBAL_MOTOR_ZERO_FORCE_CONTROL = 0,            //云台电机无力
    GIMBAL_MOTOR_ABSOLUTE_ANGLE_CONTROL,            //云台电机绝对角控制
    GIMBAL_MOTOR_RELATIVE_ANGLE_CONTROL,            //云台电机相对角控制
    GIMBAL_MOTOR_AUTO_CONTROL,                      //自瞄控制云台电机
    GIMBAL_MOTOR_REMOTE_FIRE_CONTROL,               //云台电机在吊射模式下控制
    GIMBAL_MOTOR_TURN_ROUND_CONTROL,                //云台电机一键调头模式下绝对角控制
    GIMBAL_MOTOR_INIT_CONTROL,                      //云台电机在初始的时候的控制(基本上是防腿打头)
} gimbal_motor_mode_e;

typedef struct
{
    Motor_DM_Normal dm_normal_motor;
    Motor_DM_1_To_4 dm_1_To_4_motor;
    Motor_GM6020 GM6020_motor;
    Motor_C620 C620_motor;

    //云台电机模式
    gimbal_motor_mode_e gimbal_motor_mode;
    gimbal_motor_mode_e last_gimbal_motor_mode;

    Enum_Motor_Status motor_status;
    FSM_t Motor_FSM;

    //重力补偿结构体
    Gravity_Comp_t gravity_comp;

    //控制器-----------------------
    //二阶线性控制器
    gimbal_motor_second_order_linear_controller_t gimbal_motor_second_order_linear_controller;
    fp32 second_order_linear_controller_current_set;

    PID_control Absloute_Angle_Pid;
    PID_control Relative_Angle_Pid;

    //速度环,目前就pitch吊射考虑用
    PidTypeDef Pitch_Remote_Fire_Speed_Pid;

    //初始init模式下使用，防腿打头
    PID_control Init_Angle_Pid;
    PID_control Init_Omega_Pid;
    //-----------------------

    uint16_t offset_ecd;

    fp32 min_absolute_angle;
    fp32 max_absolute_angle;
    fp32 max_relative_angle;
    fp32 min_relative_angle;

    fp32 absolute_angle_set;
    fp32 absolute_angle;
    fp32 relative_angle_set;
    fp32 relative_angle;

}gimbal_motor_t;

typedef struct
{
    //陀螺仪结构体指针
    const INS_t *gimbal_INS_point;

    //遥控器数据指针
    const DR16_Processed_Data_t *gimbal_dr16_processed_data;
    const VT13_Processed_Data_t *gimbal_vt13_processed_data;
    const Fusi_RC_Processed_Data_t *gimbal_fusi_rc_processed_data;

    //获取视觉上位机数据
    const gimbal_vision_control_t *gimbal_vision_point;

    //云台电机
    gimbal_motor_t gimbal_yaw_j4310;
    gimbal_motor_t gimbal_pitch_j4340;

    //LESO扩张观测器启动
    uint8_t leso_pitch_first_run_flag;

    //整车能动标志位
    bool_t gimbal_move_flag;

    //按下一键掉头时的角度
    fp32 gimbal_turn_round_init_angle;

    //云台归中状态机
    FSM_t FSM_Yaw_Init;

    //归中云台标志位
    uint8_t init_flag;
    //归中云台初始角
    fp32 init_angle;

    bool_t one_key_shoot_flag;
    float one_key_shoot_pitch;

    //云台系统时间
    TickType_t GIMBAL_xTickCount;
    //按下一键调头时刻，一定时间过后，模式回归正常
    TickType_t gimbal_turn_round_start_time;

}gimbal_control_t;
//------------------------------------------------------------------------------



//外部交互数据-------------------------------------------------------------------

//云台任务结构体
extern gimbal_control_t gimbal_control;

//获取云台电机指针
extern const gimbal_motor_t *get_yaw_motor_point(void);
extern const gimbal_motor_t *get_pitch_motor_point(void);

extern float ceshi_liju;
extern float guance_leso;
extern float bili;
//------------------------------------------------------------------------------

#endif
