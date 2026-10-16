#ifndef CTRLBOARD_H7_IMU_DVC_FUSI_RC_H
#define CTRLBOARD_H7_IMU_DVC_FUSI_RC_H

#include "bsp_usart.h"
#include "struct_typedef.h"

#define FUSI_RXBUF_SIZE 50

// 断控保护标志
#define FUSI_RC_LOSS_NORMAL     ((uint8_t)0)   // 无断控
#define FUSI_RC_LOSS_PROTECT    ((uint8_t)1)   // 断控保护


// 拨动开关位置
#define FUSI_SWITCH_UP                ((uint16_t)1)
#define FUSI_SWITCH_MID               ((uint16_t)3)
#define FUSI_SWITCH_DOWN              ((uint16_t)2)

// 遥控器连接状态
typedef enum
{
    Fusi_RC_Status_DISABLE = 0,
    Fusi_RC_Status_ENABLE  = 1
} Fusi_RC_Status_e;

/**
 * @brief 拨动开关状态
 *
 */
typedef enum
{
    Fusi_Three_Stage_Switch_Status_UP = 0,
    Fusi_Three_Stage_Switch_Status_UP_TRIG_MIDDLE,
    Fusi_Three_Stage_Switch_Status_MIDDLE_TRIG_UP,
    Fusi_Three_Stage_Switch_Status_MIDDLE,
    Fusi_Three_Stage_Switch_Status_MIDDLE_TRIG_DOWN,
    Fusi_Three_Stage_Switch_Status_DOWN_TRIG_MIDDLE,
    Fusi_Three_Stage_Switch_Status_DOWN,
} Enum_Fusi_Three_Stage_Switch_Status;

typedef enum
{
    Fusi_Two_Stage_Switch_Status_UP = 0,
    Fusi_Two_Stage_Switch_Status_UP_TRIG_DOWN,
    Fusi_Two_Stage_Switch_Status_DOWN_TRIG_UP,
    Fusi_Two_Stage_Switch_Status_DOWN,
} Enum_Fusi_Two_Stage_Switch_Status;

/**
 * @brief 原始数据（直接从SBUS帧解析出的通道值）
 */
typedef struct __attribute__((packed))
{
    __attribute__((packed)) struct
    {
        int16_t ch[5];
        int16_t rotary_sw[3];
        char sw[5];
    } RC;

    __attribute__((packed)) struct
    {
        uint16_t CH[16];
    } SBUS;

} Fusi_RC_Raw_Data_t;

/**
 * @brief 处理后数据（适用于上层控制）
 */
typedef struct __attribute__((packed))
{
    float Left_X;
    float Left_Y;
    float Right_X;
    float Right_Y;
    float Rotary_Switch_Left;
    float Rotary_Switch_Right;
    uint8_t Left_Switch;
    uint8_t Left_Mid_Switch;
    uint8_t Right_Switch;
    uint8_t Right_Mid_Switch;

    Enum_Fusi_Two_Stage_Switch_Status Left_Switch_Status;
    Enum_Fusi_Two_Stage_Switch_Status Left_Mid_Switch_Status;
    Enum_Fusi_Three_Stage_Switch_Status Right_Mid_Switch_Status;
    Enum_Fusi_Two_Stage_Switch_Status Right_Switch_Status;

} Fusi_RC_Processed_Data_t;

/**
 * @brief Fusi_RC 类结构体
 */
typedef struct
{
    Struct_UART_Manage_Object *UART_Manage_Object;  // UART 管理对象

    Fusi_RC_Raw_Data_t Raw_Data;                    // 原始数据
    Fusi_RC_Processed_Data_t Processed_Data;        // 处理后数据
    Fusi_RC_Processed_Data_t Last_Processed_Data;   // 上一次处理数据（用于边沿检测）

    uint8_t Flag;       // 接收计数标志
    uint8_t Last_Flag;  // 上一次接收计数

    Fusi_RC_Status_e Fusi_RC_Status;  // 遥控器在线状态
} Class_Fusi_RC;

// 全局实例声明
extern Class_Fusi_RC class_fusi_rc;

extern uint16_t fusi_up_to_down;
extern uint16_t fusi_down_to_up;
extern uint16_t fusi_down_to_mid;
extern uint16_t fusi_mid_to_down;
extern uint16_t fusi_up_to_mid;
extern uint16_t fusi_mid_to_up;

// 函数声明
void Fusi_RC_Init(Class_Fusi_RC *rc, UART_HandleTypeDef *huart);
void Fusi_RC_UART_RxCpltCallback(Class_Fusi_RC *self, uint8_t *Rx_Data, uint16_t Length);
void Class_Fusi_TIM_1ms_Calculate_PeriodElapsedCallback(Class_Fusi_RC *self);

// 数据获取接口
const Fusi_RC_Raw_Data_t *get_fusi_rc_raw_data_point(void);
const Fusi_RC_Processed_Data_t *get_fusi_rc_processed_data_point(void);


#endif //CTRLBOARD_H7_IMU_DVC_FUSI_RC_H