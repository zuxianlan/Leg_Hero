/**
  ******************************************************************************
  * @file           : robot.h
  * @author         : Chen Haoran
  * @brief          : 机器人主控类与遥控输入数据结构声明
  * @attention      : Robot 为系统唯一的全局对象，由 Task_Init() 创建；
  *                   遥控输入源在编译期通过 FUS_I6X / GAMEPAD / VT03 宏选择。
  * @date           : 2025/10/25
  ******************************************************************************
  */
#ifndef WHEEL_LEG_SYS_ROBOT_H
#define WHEEL_LEG_SYS_ROBOT_H
/* Includes ------------------------------------------------------------------*/
#include "MPC_Control.h"
#include "QuaternionEKF.h"
#include "Algorithm/FSM/fsm.h"
#include "Algorithm/Slope/slope.h"
#include "Devices/dm_mc02.h"
#include "Devices/Vofa_TCP.h"
#include "chassis/Chassis.h"
#include "Devices/INS.h"
#include "GamePad.h"
#include "Algorithm/Fusion_AHRS/Fusion_AHRS.h"
#include "Algorithm/PID/PID_Control.h"
#include "Algorithm/Fusion_AHRS/Tactical_Fusion.h"
/* Define --------------------------------------------------------------------*/

// 遥控器键盘位掩码：key 通道按位编码各按键状态
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)

// 遥控输入源选择（三选一，仅允许一个置 1）
#define FUS_I6X 1
#define GAMEPAD 0
#define VT03 0

// 遥控器拨杆档位值
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)

// 摇杆/键盘输入到目标的增益与死区配置
#define RC_to_Chassis_Yaw_Gain (3.0f)          // 摇杆 → 航向角速度增益（GAMEPAD 模式）
#define RC_to_Chassis_Leg_Gain 0.0005f         // 竖直摇杆 → 目标腿高增量增益
#define DR16_Rocker_Dead_Zone 0.1f             // 摇杆中位死区
#define RC_to_Chassis_Roll_Gain 0.001f         // 摇杆 → 目标横滚角增益
#define RC_to_Chassis_Spin_Pitch_Gain 0.1f     // 自旋模式下摇杆 → 倾斜角增益
#define Spin_Pitch_Angle_Offset 2.5f           // pitch 摆动的相位偏移(rad)，调此值校正“前后”的方向

#define MAX_Velocity 3.0f      // 前进方向最大速度 (m/s)
#define MAX_Velocity_Y 2.0f    // 横移方向最大速度 (m/s)

/* Variable && Struct --------------------------------------------------------*/

// 跳跃状态机状态
typedef enum
{
    STOP = 0,        // 停止（初始/恢复）
    BEND_LEG,        // 蹲腿蓄力
    STRETCH_LEG,     // 蹬腿起跳
    BEND_LEG_AIR,    // 空中收腿
    STRETCH_LEG_AIR  // 空中伸腿（准备落地）
} Enum_jump_fsm_mode;



/* Function Declaration ------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/

class Robot;

// 遥控输入数据（由 Remote_update() 根据输入源更新）
typedef struct
{
    float axis[6];       // 摇杆 6 通道归一化值 [-1,1]：0=前向 1=横移 2=竖直 3=横滚
    float leg_length;    // 期望腿长（预留）

    // 动作模式标志
    bool friction;       // 摩擦/刹车开关
    bool shoot;          // 射击（预留，当前恒为 false）
    bool auto_aim;       // 自瞄开关
    bool jump;           // 跳跃触发
    bool zero_force;     // 零力模式：关节随动、无力矩输出
    bool competition;    // 竞技模式：底盘跟随云台航向
    bool check_in_pos;   // 检录/初始姿态模式
    bool spin;           // 自旋模式
    bool up_staris;      // 上楼梯
    bool leg_low;        // 腿长低挡（预留未启用）
    bool leg_middle;     // 腿长中挡（预留未启用）
    bool leg_heigh;      // 腿长高挡（预留未启用）

    // 上一帧输入标志（供 StateChange 检测上升沿/下降沿）
    bool last_input_friction;
    bool last_input_auto_aim;
    bool last_input_jump;
    bool last_input_zero_force;
    bool last_input_competition;
    bool last_input_spin;
    bool last_leg_low;
    bool last_leg_middle;
    bool last_leg_heigh;

    // 键盘按键当前状态（按位解码自 key 通道）
    bool press_w;
    bool press_s;
    bool press_a;
    bool press_d;
    bool press_z;
    bool press_x;
    bool press_c;
    bool press_f;
    bool press_v;
    bool press_e;
    bool press_shift;
    bool press_ctrl;
    bool last_press_ctrl;  // 上一帧 Ctrl 状态（检测按下沿）
    bool last_press_v;     // 上一帧 V 状态
    bool last_press_e;     // 上一帧 E 状态


}Remote_Control;

// 底盘状态机：正常 / 翻转 / 离地等，用于自恢复与状态切换
class Class_FSM_Chassis : public Class_FSM
{
public:
    Robot *robot{};

    void TIM_1ms_Calculate_PeriodElapsedCallback();

private:

};

// 跳跃状态机
class Class_FSM_Jump : public Class_FSM
{
public:
    Robot *robot{};

    void TIM_1ms_Calculate_PeriodElapsedCallback();

private:

};

// 上楼梯状态机
class Class_FSM_Up_staris : public Class_FSM
{
public:
    Robot *robot{};

    void TIM_1ms_Calculate_PeriodElapsedCallback();

private:

};

class Robot
{
public:
    // 串口通信（MC02 电机控制器）
    mc_02_serial mc02_SerialPort;

    // TCP 数据监控（Vofa 上位机）
    Class_Vofa_TCP Vofa_TCP;

    // IMU 惯性测量单元数据
    class_INS INS;

    // 遥控输入数据
    Remote_Control RC{};

    // 底盘 / 跳跃 / 上楼梯 三个状态机
    Class_FSM_Chassis FSM_Chassis;
    friend class Class_FSM_Chassis;

    Class_FSM_Jump FSM_Jump;
    friend class Class_FSM_Jump;

    Class_FSM_Up_staris FSM_Up_staris;
    friend class Class_FSM_Up_staris;

    // 目标量斜坡限制器（X/Y 速度、角速度、左右腿长）
    Class_Slope Slope_X;
    Class_Slope Slope_Y;
    Class_Slope Slope_Omega;
    Class_Slope Slope_Leg_l;
    Class_Slope Slope_Leg_r;

    // 航向跟随 PID
    Class_PID PID_Fallow_yaw;

    // 底盘控制与当前控制模式
    class_Chassis Chassis;
    Enum_chassis_mode chassis_mode;

    void Init();                    // 初始化各子模块与状态机

    void Jump_Control();            // 跳跃控制

    void UP_staris_Control();       // 上楼梯控制

    void Chassis_Control();         // 底盘主控制（每个控制周期调用）

    void Check_over_turn_mod();     // 翻转状态下的腿部控制

    void Self_Rescue();             // 翻转自救

    inline float Get_Target_Omega() const;

    inline float Get_Target_Velocity() const;

    inline float Get_Target_Velocity_Y() const;

    inline float Get_Target_X() const;

    inline float Get_Target_height() const;

    inline float Get_jump_flag() const;

    // fusion::Vector read_gyro() const;

    // fusion::Vector read_accelerometer() const;

    void Fusion_AHRS_update();      // 姿态融合更新（AHRS）



private:

    class_GamePad GamePad = class_GamePad("/dev/input/event6");   // Xbox 手柄（GAMEPAD 模式）

    void Remote_update();                       // 更新遥控输入并映射为控制指令

    // 底盘控制目标量
    float Target_X = 0;            // 目标位置 X（预留）
    float Target_Velocity_Y = 0;   // 目标横移速度
    float Target_Velocity = 0;     // 目标前进速度
    float Target_left_f0 = 0;      // 左腿目标沿腿力 F0
    float Target_right_f0 = 0;     // 右腿目标沿腿力 F0
    float Target_Omega = 0;        // 目标角速度
    float Target_Roll = 0;         // 目标横滚角
    float Target_Theta = 0;        // 目标 theta 误差（腿俯仰）
    float Target_Pitch = 0.0f;     // 目标俯仰角
    float last_Target_leg = 0;     // 上一帧目标腿长
    float follow_yaw_angle = 0.0;  // 当前跟随航向角
    float Target_height = 0;       // 目标机身高度
    float Target_dheight = 0;      // 目标高度变化量
    float follow_yaw_offset = 1.9; // 跟随航向角偏移
    float target_yaw_abs = 0;      // 航向旋转目标绝对值（Ctrl 触发 180° 转向）


    bool above_flag_l = false;                 // 左腿离地标志
    bool above_flag_r = false;                 // 右腿离地标志
    bool jump_flag = false;                    // 跳跃状态标志
    bool Supercap_Accelerate_Status = false;   // 超级电容加速状态
    bool init_pos = false;                     // 初始位置标志
    bool yaw_rotate_active = false;            // 航向旋转进行中标志


    float Math_Modulus_Normalization(float x, float modulus)
    {
        float tmp;

        tmp = fmodf(x + modulus / 2.0f, modulus);

        if (tmp < 0.0f) 
        {
            tmp += modulus;
        }

        return (tmp - modulus / 2.0f);
    }

    float Get_target_angle(float x, float y, float Now_angle, float *vel)
    {
        static float target_angle;
        target_angle = atan2f(y, x);

        float tmp_delta_angle = Math_Modulus_Normalization(target_angle - Now_angle, 2.0f * M_PI);

        // 根据转向角度范围，判断是否需要就近转体
        if (-M_PI / 2.0f <= tmp_delta_angle && tmp_delta_angle <= M_PI / 2.0f)
        {
            // 在±PI / 2 之内无需反转就近转体
            target_angle = tmp_delta_angle + Now_angle;
        }
        else
        {
            // 需要旋转多一圈
            target_angle = Math_Modulus_Normalization(tmp_delta_angle + M_PI, 2.0f * M_PI) + Now_angle;
            *vel *= -1.0f;
        }

        return target_angle;
    }
};

inline float Robot::Get_Target_Omega() const
{
    return Target_Omega;
}

inline float Robot::Get_Target_Velocity() const
{
    return Target_Velocity;
}

inline float Robot::Get_Target_Velocity_Y() const
{
    return Target_Velocity_Y;
}

inline float Robot::Get_Target_X() const
{
    return Target_X;
}

inline float Robot::Get_Target_height() const
{
    return Target_height;
}

inline float Robot::Get_jump_flag() const
{
    return jump_flag;
}



#endif //WHEEL_LEG_SYS_ROBOT_H
