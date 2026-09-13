/**
 * @file can_comm.h
 * @author yuanluochen
 * @brief 多设备通信模块，主要用于控制板之间的通信，使用can总线实现，基于数组实现
 * @version 0.1
 * @date 2023-09-17
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef CAN_PACKEET_CONNECTION_H
#define CAN_PACKEET_CONNECTION_H

#include "struct_typedef.h"
#include "main.h"
#include "bsp_can.h"

typedef enum
{
    Motor_Status_OFFLINE = 0,
    Motor_Status_NORMAL,
    Motor_Status_LIGHT_STALL,
    Motor_Status_HEAVY_STALL,
} Enum_Motor_Status;

//智能结构体所需的数据结构体----------------------------------------------------------
//发送数据结构体
typedef struct __attribute__((packed))
{
    float gimbal_pitch_angle;
    int16_t fric_speed_set_average;
    uint8_t trigger_status :4;
    uint8_t dt7_status    :1;
    uint8_t vt13_status     :1;
    uint8_t fusi_status     :1;
    uint8_t robot_move_flag :1;
    uint8_t gimbal_control_status   :2;
    uint8_t shoot_control_status    :2;
    uint8_t auto_attack_status :1;
    uint8_t fric_status  :1;
    uint8_t turn_round_status     :2;
}Can_Chassis_Control_Tx_Data_t;


typedef struct __attribute__((packed))
{
    uint8_t ch_0;
    uint8_t ch_1;
    uint8_t ch_2;
    uint8_t ch_3;
    uint8_t ch_4;
    uint8_t sw_1;
    uint8_t sw_2;
    uint8_t press_l :4;
    uint8_t press_r :4;

}Can_DT7_1_Tx_Data_t;

typedef struct __attribute__((packed))
{
    int16_t x;
    int16_t y;
    int16_t z; //鼠标滚轮
    uint16_t key;    //键盘返回帧

}Can_DT7_2_Tx_Data_t;

typedef struct __attribute__((packed))
{
    uint8_t ch_0;
    uint8_t ch_1;
    uint8_t ch_2;
    uint8_t ch_3;
    uint8_t wheel;//拨齿

    uint8_t fn      :3;//左上小按钮
    uint8_t button  :3;//右上小按钮
    uint8_t go_home :3;//中左那个大按钮
    uint8_t mode_sw :3;//中三模式按钮
    uint8_t shutter :3;//拍照

    uint8_t mouse_left:3;//鼠标左键
    uint8_t mouse_right:3;//鼠标右键
    uint8_t mouse_middle:3;//鼠标中键

}Can_VT13_1_Tx_Data_t;

typedef struct __attribute__((packed))
{
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z; //鼠标滚轮
    uint16_t key;    //键盘返回帧

}Can_VT13_2_Tx_Data_t;

typedef struct __attribute__((packed))
{
    int16_t ch_0;
    int16_t ch_1;
    int16_t ch_2;
    int16_t ch_3;

}Can_Fusi_1_Tx_Data_t;

typedef struct __attribute__((packed))
{
    int16_t rotary_sw_0;
    int16_t rotary_sw_1;

    uint8_t sw_0;
    uint8_t sw_1;
    uint8_t sw_2;
    uint8_t sw_3;

}Can_Fusi_2_Tx_Data_t;


typedef struct __attribute__((packed))
{
    uint8_t left_fric_motor_status :4;
    uint8_t right_fric_motor_status :4;
    uint8_t yaw_motor_status;
    uint8_t pitch_motor_status;
    uint8_t trigger_motor_status;
    float pitch_angle;
}Can_Motor_Status_Tx_Data_t;

typedef struct __attribute__((packed))
{
    uint16_t fric_speed_set_average;
    uint16_t key;
    uint16_t flag;
    uint8_t trigger_status;
    uint8_t robot_move_flag :4;
    uint8_t robot_control_status :4;
}Can_Control_Tx_Data_t;

typedef struct __attribute__((packed))
{
    // uint16_t ch_1;
    // uint16_t ch_2;
    // uint16_t ch_3;
    // char sw_1 : 2;
    // char sw_2 : 2;
    // char sw_3 : 2;
    // char sw_4 : 2;
    // uint8_t reserve;

    uint16_t ch_1;
    uint16_t ch_2;
    uint16_t ch_3;
    uint16_t sw;
}Can_Remote_Tx_Data_t;

typedef struct __attribute__((packed))
{
    uint16_t ch_1;
    uint16_t ch_2;
    uint16_t ch_3;
    uint16_t sw;
}Can_VT13_Tx_Data_t;

//接受数据结构体
typedef struct __attribute__((packed))
{
    uint16_t robot_id;
    uint16_t shoot_heat;
    uint16_t shoot_speed;
    uint16_t shoot_heat_limit;
}Can_Referee_Rx_Data_t;
//------------------------------------------------------------------------------


//双板发送智能结构体----------------------------------------------------------------
typedef struct
{
    // 绑定的CAN
    Struct_CAN_Manage_Object *Can_Manage_Object;
    // 接收ID
    uint16_t CAN_Rx_ID;
    // 发送ID
    uint16_t CAN_Tx_ID;
    // 发送缓存区
    uint8_t *Tx_Data;

    //对外接口
    Can_Chassis_Control_Tx_Data_t Can_Chassis_Control_Tx_Data;

}Can_Chassis_Control_t;

typedef struct
{
    // 绑定的CAN
    Struct_CAN_Manage_Object *Can_Manage_Object;
    // 接收ID
    uint16_t CAN_Rx_ID;
    // 发送ID
    uint16_t CAN_Tx_ID;
    // 发送缓存区
    uint8_t *Tx_Data;

    //对外接口
    Can_DT7_1_Tx_Data_t Can_DT7_1_Tx_Data;

}Can_DT7_1_t;

typedef struct
{
    // 绑定的CAN
    Struct_CAN_Manage_Object *Can_Manage_Object;
    // 接收ID
    uint16_t CAN_Rx_ID;
    // 发送ID
    uint16_t CAN_Tx_ID;
    // 发送缓存区
    uint8_t *Tx_Data;

    //对外接口
    Can_DT7_2_Tx_Data_t Can_DT7_2_Tx_Data;

}Can_DT7_2_t;

typedef struct
{
    // 绑定的CAN
    Struct_CAN_Manage_Object *Can_Manage_Object;
    // 接收ID
    uint16_t CAN_Rx_ID;
    // 发送ID
    uint16_t CAN_Tx_ID;
    // 发送缓存区
    uint8_t *Tx_Data;

    //对外接口
    Can_VT13_1_Tx_Data_t Can_VT13_1_Tx_Data;

}Can_VT13_1_t;

typedef struct
{
    // 绑定的CAN
    Struct_CAN_Manage_Object *Can_Manage_Object;
    // 接收ID
    uint16_t CAN_Rx_ID;
    // 发送ID
    uint16_t CAN_Tx_ID;
    // 发送缓存区
    uint8_t *Tx_Data;

    //对外接口
    Can_VT13_2_Tx_Data_t Can_VT13_2_Tx_Data;

}Can_VT13_2_t;

typedef struct
{
    // 绑定的CAN
    Struct_CAN_Manage_Object *Can_Manage_Object;
    // 接收ID
    uint16_t CAN_Rx_ID;
    // 发送ID
    uint16_t CAN_Tx_ID;
    // 发送缓存区
    uint8_t *Tx_Data;

    //对外接口
    Can_Fusi_1_Tx_Data_t Can_Fusi_1_Tx_Data;

}Can_Fusi_1_t;

typedef struct
{
    // 绑定的CAN
    Struct_CAN_Manage_Object *Can_Manage_Object;
    // 接收ID
    uint16_t CAN_Rx_ID;
    // 发送ID
    uint16_t CAN_Tx_ID;
    // 发送缓存区
    uint8_t *Tx_Data;

    //对外接口
    Can_Fusi_2_Tx_Data_t Can_Fusi_2_Tx_Data;

}Can_Fusi_2_t;


typedef struct
{
    // 绑定的CAN
    Struct_CAN_Manage_Object *Can_Manage_Object;
    // 接收ID
    uint16_t CAN_Rx_ID;
    // 发送ID
    uint16_t CAN_Tx_ID;
    // 发送缓存区
    uint8_t *Tx_Data;

    //对外接口

    //发送内容
    Can_Motor_Status_Tx_Data_t Can_Motor_Status_Tx_Data;

}Can_Motor_Status_Data_t;

typedef struct
{
    // 绑定的CAN
    Struct_CAN_Manage_Object *Can_Manage_Object;
    // 接收ID
    uint16_t CAN_Rx_ID;
    // 发送ID
    uint16_t CAN_Tx_ID;
    // 发送缓存区
    uint8_t *Tx_Data;

    //对外接口

    //发送内容
    Can_Control_Tx_Data_t Can_Control_Tx_Data;

}Can_Control_Data_t;

typedef struct
{
    // 绑定的CAN
    Struct_CAN_Manage_Object *Can_Manage_Object;
    // 接收ID
    uint16_t CAN_Rx_ID;
    // 发送ID
    uint16_t CAN_Tx_ID;
    // 发送缓存区
    uint8_t *Tx_Data;

    //对外接口

    //发送内容
    Can_Remote_Tx_Data_t Can_Control_Tx_Data;

}Can_Remote_Data_t;

typedef struct
{
    // 绑定的CAN
    Struct_CAN_Manage_Object *Can_Manage_Object;
    // 接收ID
    uint16_t CAN_Rx_ID;
    // 发送ID
    uint16_t CAN_Tx_ID;
    // 发送缓存区
    uint8_t *Tx_Data;

    //对外接口

    //发送内容
    Can_VT13_Tx_Data_t Can_VT13_Tx_Data;
}Can_VT13_Data_t;

//------------------------------------------------------------------------------


//双板接受智能结构体----------------------------------------------------------------
typedef struct
{
    // 绑定的CAN
    Struct_CAN_Manage_Object *Can_Manage_Object;
    // 接收ID
    uint16_t CAN_Rx_ID;
    // 发送ID
    uint16_t CAN_Tx_ID;
    // 发送缓存区
    uint8_t *Tx_Data;

    //对外接口
    Can_Referee_Rx_Data_t Can_Referee_Rx_Data;

}Can_Referee_t;
//------------------------------------------------------------------------------

//双板通信发送函数
void Can_Chassis_Control_Init(Can_Chassis_Control_t *Can_Chassis_Control, FDCAN_HandleTypeDef *hdfcan, uint16_t __CAN_Tx_ID);
void Can_DT7_1_Init(Can_DT7_1_t *Can_DT7_1, FDCAN_HandleTypeDef *hdfcan, uint16_t __CAN_Tx_ID);
void Can_DT7_2_Init(Can_DT7_2_t *Can_DT7_2, FDCAN_HandleTypeDef *hdfcan, uint16_t __CAN_Tx_ID);
void Can_VT13_1_Init(Can_VT13_1_t *Can_VT13_1, FDCAN_HandleTypeDef *hdfcan, uint16_t __CAN_Tx_ID);
void Can_VT13_2_Init(Can_VT13_2_t *Can_VT13_2, FDCAN_HandleTypeDef *hdfcan, uint16_t __CAN_Tx_ID);
void Can_Fusi_1_Init(Can_Fusi_1_t *Can_Fusi_1, FDCAN_HandleTypeDef *hdfcan, uint16_t __CAN_Tx_ID);
void Can_Fusi_2_Init(Can_Fusi_2_t *Can_Fusi_2, FDCAN_HandleTypeDef *hdfcan, uint16_t __CAN_Tx_ID);

void Can_Motor_Status_Data_Init(Can_Motor_Status_Data_t *Can_Motor_Status_Data, FDCAN_HandleTypeDef *hdfcan, uint16_t __CAN_Tx_ID);
void Can_Control_Data_Init(Can_Control_Data_t *Can_Control_Data, FDCAN_HandleTypeDef *hdfcan, uint16_t __CAN_Tx_ID);
void Can_Remote_Data_Init(Can_Remote_Data_t *Can_Remote_Data, FDCAN_HandleTypeDef *hdfcan, uint16_t __CAN_Tx_ID);
void Can_VT13_Data_Init(Can_VT13_Data_t *Can_VT13_Data, FDCAN_HandleTypeDef *hdfcan, uint16_t __CAN_Tx_ID);

void Can_Chassis_Control_Data_Out_Put(Can_Chassis_Control_t *Can_Chassis_Control);
void Can_DT7_1_Data_Out_Put(Can_DT7_1_t *Can_DT7_1);
void Can_DT7_2_Data_Out_Put(Can_DT7_2_t *Can_DT7_2);
void Can_VT13_1_Data_Out_Put(Can_VT13_1_t *Can_VT13_1);
void Can_VT13_2_Data_Out_Put(Can_VT13_2_t *Can_VT13_2);
void Can_Fusi_1_Data_Out_Put(Can_Fusi_1_t *Can_Fusi_1);
void Can_Fusi_2_Data_Out_Put(Can_Fusi_2_t *Can_Fusi_2);

void Can_Motor_Status_Data_Out_Put(Can_Motor_Status_Data_t *Can_Motor_Status_Data);
void Can_Control_Data_Out_Put(Can_Control_Data_t *Can_Control_Data);
void Can_Remote_Data_Out_Put(Can_Remote_Data_t *Can_Remote_Data);
void Can_VT13_Data_Out_Put(Can_VT13_Data_t *Can_VT13_Data);

//双板通信接受函数
void Can_Referee_Init(Can_Referee_t *Can_Referee, FDCAN_HandleTypeDef *hdfcan, uint16_t __CAN_Rx_ID);

void Can_Referee_Data_Receive_Process(Can_Referee_t *Can_Referee, uint8_t *Rx_Data);

#endif // !CAN_PACKEET_CONNECTION_H
