/**
 * @file vision_task.h
 * @author yuanluochen
 * @brief 解析视觉数据包，处理视觉观测数据，预测装甲板位置，以及计算弹道轨迹，进行弹道补偿
 * @version 0.1
 * @date 2023-03-11
 *
 * @copyright Copyright (c) 2023
 *
 */


#ifndef VISION_TASK_H
#define VISION_TASK_H

#include "INS_task.h"
#include "arm_math.h"
#include "Mathh.h"

//延时等待
#define VISION_TASK_INIT_TIME 450
//系统延时时间
#define VISION_CONTROL_TIME_MS 1
//机器人红蓝id分界值，大于该值则机器人自身为蓝色，小于这个值机器人自身为红色
#define ROBOT_RED_AND_BLUE_DIVIDE_VALUE 100.0f


//接收数据状态
typedef enum
{
    //未读取
    UNLOADED,
    //已读取
    LOADED,
}receive_state_e;

//数据起始帧类型
typedef enum
{
    //下位机发送到上位机
    LOWER_TO_HIGH_HEAD = 0x5A,
    //上位机发送到下位机
    HIGH_TO_LOWER_HEAD = 0XA5,
}data_head_type_e;

typedef enum
{
    VISION_NO_CONTROL=0,       // 自瞄没识别到
    VISION_ONLY_CONTROL_GIMBAL, // 自瞄可以控制云台但不能开火
    VISION_CONTROL_FIRE,  // 自瞄允许开火
} vision_control_mode_e;

//装甲板颜色
typedef enum
{
    RED = 0,
    BLUE = 1,
}robot_armor_color_e;

typedef enum
{
    SHOOT_ATTACK,       // 袭击
    SHOOT_READY_ATTACK, // 准备袭击
    SHOOT_STOP_ATTACK,  // 停止袭击
} shoot_command_e;

typedef struct
{
    //自瞄控制云台标志位
    bool_t vision_control_gimbal_flag;

    // 本次云台yaw轴目标数值
    fp32 gimbal_yaw;
    // 本次云台pitch轴目标数值
    fp32 gimbal_pitch;

    //云台前馈角速度
    fp32 gimbal_feed_forward_yaw_omega;
    fp32 gimbal_feed_forward_pitch_omega;

    //云台前馈加速度
    fp32 gimbal_feed_forward_yaw_accel;
    fp32 gimbal_feed_forward_pitch_accel;

} gimbal_vision_control_t;

typedef struct
{
    //自瞄控制开火标志位
    bool_t vision_control_fire_flag;

    // 自动发射命令
    shoot_command_e shoot_command;

} shoot_vision_control_t;


//均值弹速
#define BULLET_SPEED_SIZE 3
typedef struct{
    fp32 bullet_speed[BULLET_SPEED_SIZE];
    fp32 est_bullet_speed;
}bullet_speed_t;

// 视觉任务结构体
typedef struct
{
    // 绝对角指针
    const INS_t* vision_INS_point;

    // 机器人id
    uint8_t robot_id;

    // 检测装甲板的颜色(敌方装甲板的颜色)
    uint8_t detect_armor_color;

    // 弹速
    bullet_speed_t bullet_speed;

    //自瞄控制模式
    vision_control_mode_e vision_control_mode;

    // 云台电机运动命令
    gimbal_vision_control_t gimbal_vision_control;
    // 发射机构发射命令
    shoot_vision_control_t shoot_vision_control;

} vision_control_t;


//FYT码结构体------------------------------------------
typedef struct __attribute__((packed))
{
    uint8_t header;
    uint8_t mod;
    float roll;
    float pitch;
    float yaw;
    uint8_t fill;
    uint8_t ender;
}FYT_vision_send_packet_t;

typedef struct __attribute__((packed))
{
    uint8_t header;
    uint8_t fire;
    float pitch;
    float yaw;
    float distance;
    uint8_t ender;
}FYT_vision_receive_packet_t;


typedef struct
{
    // 串口发送缓冲区
    uint8_t tx_buf[50];
    // 串口接收缓冲区
    uint8_t rx_buf[50];
    // 发送给视觉结构体
    FYT_vision_send_packet_t vision_send_packet;
    // 接收到视觉结构体
    FYT_vision_receive_packet_t Rx_data;

}FYT_vision_t;
//--------------------------------------------------------

//SP码结构体-------------------------------------------------
typedef struct __attribute__((packed))
{
    uint8_t head[2];                // 字节0-1: 帧头 "SP"
    uint8_t mode;                   // 字节2: 工作模式
    float q[4];                     // 字节3-18: 四元数 (w,x,y,z)
    float yaw;                      // 字节19-22: 偏航角 (rad)
    float yaw_omega;                  // 字节23-26: 偏航角速度 (rad/s)
    float pitch;                    // 字节27-30: 俯仰角 (rad)
    float pitch_omega;                // 字节31-34: 俯仰角速度 (rad/s)
    float bullet_speed;             // 字节35-38: 子弹速度 (m/s)
    uint8_t enemy_color;
    uint16_t bullet_count;          // 字节39-40: 子弹计数
    uint16_t crc16;                 // 字节41-42: CRC16校验
}SP_vision_send_packet_t;

typedef struct __attribute__((packed))
{
    uint8_t head[2];                // 字节0-1: 帧头 "SP"
    uint8_t mode;                   // 0: 不控制  1:能控制云台但不能开火  2：能控制云台且能开火
    float yaw;                      // 字节3-6: 偏航角 (rad)
    float yaw_omega;                  // 字节7-10: 偏航角速度 (rad/s)
    float yaw_accel;                  // 字节11-14: 偏航角加速度 (rad/s?)
    float pitch;                    // 字节15-18: 俯仰角 (rad)
    float pitch_omega;                // 字节19-22: 俯仰角速度 (rad/s)
    float pitch_accel;                // 字节23-26: 俯仰角加速度 (rad/s?)
    float distance;
    uint16_t crc16;                 // 字节31-32: CRC16校验
}SP_vision_receive_packet_t;

typedef struct
{
    // 串口发送缓冲区
    uint8_t tx_buf[50];
    // 串口接收缓冲区
    uint8_t rx_buf[50];
    // 发送给视觉结构体
    SP_vision_send_packet_t vision_send_packet;
    // 接收到视觉结构体
    SP_vision_receive_packet_t Rx_data;
    SP_vision_receive_packet_t Rx_data_Origin;

}SP_vision_t;
//--------------------------------------------------------



//与外部数据交互-----------------------------------
void Vision_Task(void const *pvParameters);
void receive_decode(uint8_t* buf, uint32_t len);

// 获取上位机云台命令
const gimbal_vision_control_t *get_vision_gimbal_point(void);
// 获取上位机发射命令
const shoot_vision_control_t *get_vision_shoot_point(void);

extern vision_control_t vision_control;
extern FYT_vision_t FYT_vision;
extern SP_vision_t SP_vision;
//--------------------------------------------------

#endif // !VISION_TASK_H
