//
// Created by 25031 on 2026/4/12.
//

#ifndef CTRLBOARD_H7_IMU_RC_TASK_H
#define CTRLBOARD_H7_IMU_RC_TASK_H

#include "cmsis_os2.h"
#include "FreeRTOS.h"

#define RC_TASK_INIT_TIME 400

#define RC_KEY_PRESSED_OFFSET_W    ((uint16_t)1 << 0)
#define RC_KEY_PRESSED_OFFSET_S    ((uint16_t)1 << 1)
#define RC_KEY_PRESSED_OFFSET_A    ((uint16_t)1 << 2)
#define RC_KEY_PRESSED_OFFSET_D    ((uint16_t)1 << 3)
#define RC_KEY_PRESSED_OFFSET_SHIFT ((uint16_t)1 << 4)
#define RC_KEY_PRESSED_OFFSET_CTRL ((uint16_t)1 << 5)
#define RC_KEY_PRESSED_OFFSET_Q    ((uint16_t)1 << 6)
#define RC_KEY_PRESSED_OFFSET_E    ((uint16_t)1 << 7)
#define RC_KEY_PRESSED_OFFSET_R    ((uint16_t)1 << 8)
#define RC_KEY_PRESSED_OFFSET_F    ((uint16_t)1 << 9)
#define RC_KEY_PRESSED_OFFSET_G    ((uint16_t)1 << 10)
#define RC_KEY_PRESSED_OFFSET_Z    ((uint16_t)1 << 11)
#define RC_KEY_PRESSED_OFFSET_X    ((uint16_t)1 << 12)
#define RC_KEY_PRESSED_OFFSET_C    ((uint16_t)1 << 13)
#define RC_KEY_PRESSED_OFFSET_V    ((uint16_t)1 << 14)
#define RC_KEY_PRESSED_OFFSET_B    ((uint16_t)1 << 15)

typedef enum
{
    No_Control = 0,
    Remote_Control,
    Key_Mouse_Control,
} robot_control_status_t;//机器人控制状态

typedef enum
{
    no_rc=0,
    dt7_mode,
    vt13_mode,
    fusi_mode,
} rc_mode_t;//机器人遥控器模式

typedef struct
{
    float Left_X;
    float Left_Y;
    float Right_X;
    float Right_Y;

} move_channel_t;

typedef struct
{
    float Mouse_X;
    float Mouse_Y;
    float Mouse_Z;

    uint8_t Mouse_Left_Key;
    uint8_t Mouse_Right_Key;

    uint16_t Key;
} key_mouse_t;


typedef struct
{
    robot_control_status_t gimbal_control_status;
    robot_control_status_t last_gimbal_control_status;

    robot_control_status_t shoot_control_status;
    robot_control_status_t last_shoot_control_status;

    rc_mode_t rc_mode;//判断哪个遥控器在控制

    //四个移动基本通道
    move_channel_t move_channel;

    //键鼠
    key_mouse_t key_mouse;

    TickType_t RC_xTickCount;

} RC_Control_t;

extern RC_Control_t RC_Control;

uint16_t Get_gimbal_behaviour_mode(void);
uint16_t Get_shoot_behaviour_mode(void);

void Get_dt7_shoot_flag(void);
void Get_vt13_shoot_flag(void);
void Get_fusi_shoot_flag(void);

void Get_dt7_rc_gongdan_flag(void);
void Get_dt7_rc_huibo_flag(void);
void Get_vt13_rc_gongdan_flag(void);
void Get_vt13_rc_huibo_flag(void);
void Get_fusi_rc_gongdan_flag(void);
void Get_fusi_rc_huibo_flag(void);

#endif //CTRLBOARD_H7_IMU_RC_TASK_H