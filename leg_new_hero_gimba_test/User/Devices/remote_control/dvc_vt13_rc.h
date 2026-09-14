#ifndef CTRLBOARD_H7_IMU_DVC_VT13_RC_H
#define CTRLBOARD_H7_IMU_DVC_VT13_RC_H

/**
 * @file dvc_vt13_rc.h
 * @brief VT13 遥控器设备驱动（模仿 dvc_dr16 风格）
 * @version 1.1
 * @date 2026-04-02
 */
#ifndef DVC_VT13_RC_H
#define DVC_VT13_RC_H

#include "bsp_usart.h"
#include "struct_typedef.h"

#define Other_RC_Control 0
#define VT13_RC_Control 1
#define VT13_Key_Mouse_Control 2

#define VT13_RXBUF_SIZE 512

#define VT13_KEY_PRESSED_OFFSET_W    ((uint16_t)1 << 0)
#define VT13_KEY_PRESSED_OFFSET_S    ((uint16_t)1 << 1)
#define VT13_KEY_PRESSED_OFFSET_A    ((uint16_t)1 << 2)
#define VT13_KEY_PRESSED_OFFSET_D    ((uint16_t)1 << 3)
#define VT13_KEY_PRESSED_OFFSET_SHIFT ((uint16_t)1 << 4)
#define VT13_KEY_PRESSED_OFFSET_CTRL ((uint16_t)1 << 5)
#define VT13_KEY_PRESSED_OFFSET_Q    ((uint16_t)1 << 6)
#define VT13_KEY_PRESSED_OFFSET_E    ((uint16_t)1 << 7)
#define VT13_KEY_PRESSED_OFFSET_R    ((uint16_t)1 << 8)
#define VT13_KEY_PRESSED_OFFSET_F    ((uint16_t)1 << 9)
#define VT13_KEY_PRESSED_OFFSET_G    ((uint16_t)1 << 10)
#define VT13_KEY_PRESSED_OFFSET_Z    ((uint16_t)1 << 11)
#define VT13_KEY_PRESSED_OFFSET_X    ((uint16_t)1 << 12)
#define VT13_KEY_PRESSED_OFFSET_C    ((uint16_t)1 << 13)
#define VT13_KEY_PRESSED_OFFSET_V    ((uint16_t)1 << 14)
#define VT13_KEY_PRESSED_OFFSET_B    ((uint16_t)1 << 15)

typedef enum
{
    VT13_RC_Status_DISABLE = 0,
    VT13_RC_Status_ENABLE  = 1
} VT13_RC_Status_e;

/**
 * @brief 原始数据结构（直接对应接收到的 21 字节）
 */
typedef struct __attribute__((packed))
{
    uint8_t soft_1;      // 0xA9
    uint8_t soft_2;      // 0x53
    uint64_t ch_0:11;
    uint64_t ch_1:11;
    uint64_t ch_2:11;
    uint64_t ch_3:11;
    uint64_t mode_sw:2;
    uint64_t go_home:1;
    uint64_t fn:1;
    uint64_t button:1;
    uint64_t wheel:11;
    uint64_t shutter:1;
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left:2;
    uint8_t mouse_right:2;
    uint8_t mouse_middle:2;
    uint16_t key;
    uint16_t crc16;
} VT13_Raw_Data_t;

/**
 * @brief 处理后数据（归一化、开关状态解析等）
 */
typedef struct
{
    float Left_X;           // 通道0，归一化 [-1,1]
    float Left_Y;
    float Right_X;
    float Right_Y;
    float Wheel;         // 拨轮，归一化 [-1,1]
    uint8_t Mode_Switch;     // 挡位开关原始值 (0-3)
    uint8_t Go_Home;     // 返航/暂停按键
    uint8_t Fn;          // Fn 按键
    uint8_t Button;      // 拍照切换按键
    uint8_t Shutter;     // 拍照/录像按键
    float Mouse_X;
    float Mouse_Y;
    float Mouse_Z;
    uint8_t Mouse_Left;
    uint8_t Mouse_Right;
    uint8_t Mouse_Middle;
    uint16_t Key;        // 键盘按键位掩码
} VT13_Processed_Data_t;

typedef enum
{
    Control_Tool = 0x0302,  //自定义控制器
    Key_Mouse = 0x0304      //原始键鼠数据
}Enum_Transmission_Command_ID;

typedef struct __attribute__((packed))
{
    //帧头
    uint8_t Frame_Header;
    uint16_t Data_Length;
    uint8_t Sequence;
    uint8_t CRC_8;
    //命令码
    Enum_Transmission_Command_ID Transmission_Command_ID;

    uint8_t Data[121];
}Struct_Transmission_UART_Data;

/**
 * @brief VT13 遥控器类结构体（模仿 Class_DR16）
 */
typedef struct
{
    Struct_UART_Manage_Object *UART_Manage_Object;

    VT13_Raw_Data_t Raw_Data;

    VT13_Processed_Data_t Processed_Data;
    VT13_Processed_Data_t Last_Processed_Data;

    float Rocker_Offset; // 摇杆中位偏移量（通常 1024）
    float Rocker_Num;    // 摇杆量程（通常 660）

    uint8_t Flag;
    uint8_t Last_Flag;

    VT13_RC_Status_e VT13_Status;
} Class_VT13_RC;

/* 外部变量声明 */
extern Class_VT13_RC class_vt13_rc;

/* 函数声明 */
void VT13_Init(Class_VT13_RC *vt13, UART_HandleTypeDef *huart);
void VT13_UART_RxCpltCallback(Class_VT13_RC *self, uint8_t *Rx_Data, uint16_t Length);
void Class_VT13_RC_TIM_100ms_Alive_PeriodElapsedCallback(Class_VT13_RC *self);

const VT13_Raw_Data_t *get_vt13_raw_data_point(void);
const VT13_Processed_Data_t *get_vt13_processed_data_point(void);

#endif // DVC_VT13_RC_H


#endif //CTRLBOARD_H7_IMU_DVC_VT13_RC_H