#ifndef __DVC_DR16_H
#define __DVC_DR16_H

#include "bsp_usart.h"
#include "struct_typedef.h"
#include "bsp_usart.h"
#define DR16_RXBUF_SIZE 36

// 拨动开关位置
#define DR16_SWITCH_UP                ((uint16_t)1)
#define DR16_SWITCH_MID               ((uint16_t)3)
#define DR16_SWITCH_DOWN              ((uint16_t)2)

#define dr16_switch_is_down(s)       (s == DR16_SWITCH_DOWN)
#define dr16_switch_is_mid(s)        (s == DR16_SWITCH_MID)
#define dr16_switch_is_up(s)         (s == DR16_SWITCH_UP)

// 按键开关位置
#define DR16_KEY_FREE (0)
#define DR16_KEY_PRESSED (1)


#define DR16_KEY_PRESSED_OFFSET_W    ((uint16_t)1 << 0)
#define DR16_KEY_PRESSED_OFFSET_S    ((uint16_t)1 << 1)
#define DR16_KEY_PRESSED_OFFSET_A    ((uint16_t)1 << 2)
#define DR16_KEY_PRESSED_OFFSET_D    ((uint16_t)1 << 3)
#define DR16_KEY_PRESSED_OFFSET_SHIFT ((uint16_t)1 << 4)
#define DR16_KEY_PRESSED_OFFSET_CTRL ((uint16_t)1 << 5)
#define DR16_KEY_PRESSED_OFFSET_Q    ((uint16_t)1 << 6)
#define DR16_KEY_PRESSED_OFFSET_E    ((uint16_t)1 << 7)
#define DR16_KEY_PRESSED_OFFSET_R    ((uint16_t)1 << 8)
#define DR16_KEY_PRESSED_OFFSET_F    ((uint16_t)1 << 9)
#define DR16_KEY_PRESSED_OFFSET_G    ((uint16_t)1 << 10)
#define DR16_KEY_PRESSED_OFFSET_Z    ((uint16_t)1 << 11)
#define DR16_KEY_PRESSED_OFFSET_X    ((uint16_t)1 << 12)
#define DR16_KEY_PRESSED_OFFSET_C    ((uint16_t)1 << 13)
#define DR16_KEY_PRESSED_OFFSET_V    ((uint16_t)1 << 14)
#define DR16_KEY_PRESSED_OFFSET_B    ((uint16_t)1 << 15)

// 键位宏定义
#define DR16_KEY_W 0
#define DR16_KEY_S 1
#define DR16_KEY_A 2
#define DR16_KEY_D 3
#define DR16_KEY_SHIFT 4
#define DR16_KEY_CTRL 5
#define DR16_KEY_Q 6
#define DR16_KEY_E 7
#define DR16_KEY_R 8
#define DR16_KEY_F 9
#define DR16_KEY_G 10
#define DR16_KEY_Z 11
#define DR16_KEY_X 12
#define DR16_KEY_C 13
#define DR16_KEY_V 14
#define DR16_KEY_B 15


typedef enum
{
    DR16_Status_DISABLE = 0,
    DR16_Status_ENABLE  = 1
} DR16_Status_e;

/**
 * @brief 拨动开关状态
 *
 */
typedef enum
{
    DR16_Switch_Status_UP = 0,
    DR16_Switch_Status_UP_TRIG_MIDDLE,
    DR16_Switch_Status_MIDDLE_TRIG_UP,
    DR16_Switch_Status_MIDDLE,
    DR16_Switch_Status_MIDDLE_TRIG_DOWN,
    DR16_Switch_Status_DOWN_TRIG_MIDDLE,
    DR16_Switch_Status_DOWN,
} Enum_DR16_Switch_Status;

/**
 * @brief 按键状态
 *
 */
typedef enum
{
    DR16_Key_Status_FREE = 0,
    DR16_Key_Status_TRIG_FREE_PRESSED,
    DR16_Key_Status_TRIG_PRESSED_FREE,
    DR16_Key_Status_PRESSED,
} Enum_DR16_Key_Status;


// ==============================
// 这里直接用 remote_control 的完整数据结构
// ==============================
typedef struct __attribute__((packed))
{
    int16_t ch[5];
    char s[2];
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t press_l;
    uint8_t press_r;
    uint16_t key;
} DR16_Raw_Data_t;

typedef struct __attribute__((packed))
{
    float Left_X;
    float Left_Y;
    float Right_X;
    float Right_Y;
    float Yaw;
    char Left_Switch;
    char Right_Switch;
    float Mouse_X;
    float Mouse_Y;
    float Mouse_Z;
    uint8_t Mouse_Left_Key;
    uint8_t Mouse_Right_Key;
    uint16_t Key;

    Enum_DR16_Switch_Status Left_Switch_Status;
    Enum_DR16_Switch_Status Right_Switch_Status;
    Enum_DR16_Key_Status Mouse_Left_Key_Status;
    Enum_DR16_Key_Status Mouse_Right_Key_Status;
    Enum_DR16_Key_Status Keyboard_Key_Status[16];
} DR16_Processed_Data_t;

typedef struct
{
    Struct_UART_Manage_Object *UART_Manage_Object;

    DR16_Raw_Data_t Raw_Data;

    DR16_Processed_Data_t Processed_Data;
    DR16_Processed_Data_t Last_Processed_Data;

    uint8_t Flag;
    uint8_t Last_Flag;
    float Rocker_Offset;
    float Rocker_Num;

    DR16_Status_e DR16_Status;
} Class_DR16;

void DR16_Init(Class_DR16 *dr16, UART_HandleTypeDef *huart);
void DR16_UART_RxCpltCallback(Class_DR16 *self,uint8_t *Rx_Data, uint16_t Length);
void Class_DR16_TIM_100ms_Alive_PeriodElapsedCallback(Class_DR16 *self);
void Class_DR16_TIM_1ms_Calculate_PeriodElapsedCallback(Class_DR16 *self);
const DR16_Raw_Data_t *get_dr16_raw_data_point(void);
const DR16_Processed_Data_t *get_dr16_processed_data_point(void);

extern Class_DR16 class_dr16;

extern uint16_t dr16_down_to_mid;
extern uint16_t dr16_mid_to_down;
extern uint16_t dr16_up_to_mid;
extern uint16_t dr16_mid_to_up;

#endif