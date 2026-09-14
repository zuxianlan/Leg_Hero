/**
 * @file dvc_referee.c
 * @author yssickjgd (1345578933@qq.com)
 * @brief PM01裁判系统
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2024-01-30 1.1 适配1.6.1通信协议
 *
 * @copyright USTC-RoboWalker (c) 2023-2024
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "dvc_referee.h"
#include <string.h>

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

// CRC8校验码表
static const uint8_t crc_8_table[256] = {
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6d, 0x33, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35
};

// CRC16校验码表
static const uint16_t crc_16_table[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e, 0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb, 0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738, 0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5, 0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232, 0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

/* Private function declarations ---------------------------------------------*/
static uint8_t Verify_CRC_8(uint8_t *Message, uint32_t Length);
static uint16_t Verify_CRC_16(uint8_t *Message, uint32_t Length);
void Class_Referee_Data_Process(Class_Referee *self, uint16_t Length);

Class_Referee class_referee;
/* Function prototypes -------------------------------------------------------*/

void Class_Referee_Init(Class_Referee *self, UART_HandleTypeDef *huart, uint8_t __Frame_Header)
{
    // 根据UART句柄选择对应的UART管理对象（假设 UART_Manage_Object 已在外部定义）
    // 这里需要根据实际工程中的 UART 管理对象选择，简化处理：假设外部已定义全局变量 UART1_Manage_Object 等
    // 原代码中通过 UART_Manage_Object 指向相应的管理对象，此处需要与底层驱动对接。
    // 为简化，假设用户会在外部设置 UART_Manage_Object 指针，这里仅做占位。
    // 实际使用时，需要根据 huart->Instance 选择对应的 Struct_UART_Manage_Object 指针。
    // 这里留空，由用户自行实现。
    if (huart->Instance == USART1)
    {
        self->UART_Manage_Object = &UART1_Manage_Object;
    }
    else if (huart->Instance == USART2)
    {
        self->UART_Manage_Object = &UART2_Manage_Object;
    }
    else if (huart->Instance == USART3)
    {
        self->UART_Manage_Object = &UART3_Manage_Object;
    }
    else if (huart->Instance == UART4)
    {
        self->UART_Manage_Object = &UART4_Manage_Object;
    }
    else if (huart->Instance == UART5)
    {
        self->UART_Manage_Object = &UART5_Manage_Object;
    }
    else if (huart->Instance == USART6)
    {
        self->UART_Manage_Object = &UART6_Manage_Object;
    }
    else if (huart->Instance == UART7)
    {
        self->UART_Manage_Object = &UART7_Manage_Object;
    }
    else if (huart->Instance == UART8)
    {
        self->UART_Manage_Object = &UART8_Manage_Object;
    }

    self->Frame_Header = __Frame_Header;
    self->Flag = 0;
    self->Pre_Flag = 0;
    self->Sequence = 0;
    self->Referee_Status = Referee_Status_DISABLE;
    self->Referee_Trust_Status = Referee_Data_Status_ENABLE;
    memset(self->UI_Change_Flag, 0, sizeof(self->UI_Change_Flag));
    // 清空图形配置缓存
    memset(self->Graphic_Config, 0, sizeof(self->Graphic_Config));
}

/* 图形配置生成函数实现 */
/* 设置裁判系统可信状态 */
void Class_Referee_Set_Referee_Trust_Status(Class_Referee *self, Enum_Referee_Data_Status __Referee_Trust_Status)
{
    self->Referee_Trust_Status = __Referee_Trust_Status;
}

/* 清空 UI 修改标志位 */
void Class_Referee_Set_Referee_UI_Change_Flag_Clear(Class_Referee *self)
{
    for (int i = 0; i < 10; i++)
    {
        for (int j = 0; j < 10; j++)
        {
            self->UI_Change_Flag[i][j] = 0;
        }
    }
}


/* 发送 1 个图形 */
void Class_Referee_UART_Send_Interaction_UI_Graphic_1(Class_Referee *self, Struct_Referee_Data_Interaction_Graphic_Config *Graphic_1)
{
    Struct_Referee_UART_Data *tmp_buffer = (Struct_Referee_UART_Data *) self->UART_Manage_Object->Tx_Buffer;

    // 裁判系统帧头
    tmp_buffer->Frame_Header = 0xA5;
    tmp_buffer->Data_Length = sizeof(Struct_Referee_Tx_Data_Interaction_Graphic_1) - 2;
    tmp_buffer->Sequence = self->Sequence;
    tmp_buffer->CRC_8 = Verify_CRC_8((uint8_t *) tmp_buffer, 4);
    tmp_buffer->Referee_Command_ID = Referee_Command_ID_INTERACTION;

    // 交互帧头
    Struct_Referee_Tx_Data_Interaction_Graphic_1 *tmp_data = (Struct_Referee_Tx_Data_Interaction_Graphic_1 *) tmp_buffer->Data;
    tmp_data->Header = Referee_Interaction_Command_ID_UI_GRAPHIC_1;
    tmp_data->Sender = self->Referee_Rx_Data.Robot_Status.robot_id;
    tmp_data->Receiver = (Enum_Referee_Data_Robots_Client_ID)((int) self->Referee_Rx_Data.Robot_Status.robot_id + 0x100);

    // UI发一个图形帧内容
    tmp_data->Graphic[0] = *Graphic_1;

    tmp_data->CRC_16 = Verify_CRC_16((uint8_t *) tmp_buffer, 7 + tmp_buffer->Data_Length);

    HAL_UART_Transmit(self->UART_Manage_Object->UART_Handler, (uint8_t *) tmp_buffer, 7 + sizeof(Struct_Referee_Tx_Data_Interaction_Graphic_1), 100);

    self->Sequence++;
}

/* 发送 2 个图形 */
void Class_Referee_UART_Send_Interaction_UI_Graphic_2(Class_Referee *self, Struct_Referee_Data_Interaction_Graphic_Config *Graphic_1, Struct_Referee_Data_Interaction_Graphic_Config *Graphic_2)
{
    Struct_Referee_UART_Data *tmp_buffer = (Struct_Referee_UART_Data *) self->UART_Manage_Object->Tx_Buffer;

    // 裁判系统帧头
    tmp_buffer->Frame_Header = 0xA5;
    tmp_buffer->Data_Length = sizeof(Struct_Referee_Tx_Data_Interaction_Graphic_2) - 2;
    tmp_buffer->Sequence = self->Sequence;
    tmp_buffer->CRC_8 = Verify_CRC_8((uint8_t *) tmp_buffer, 4);
    tmp_buffer->Referee_Command_ID = Referee_Command_ID_INTERACTION;

    // 交互帧头
    Struct_Referee_Tx_Data_Interaction_Graphic_2 *tmp_data = (Struct_Referee_Tx_Data_Interaction_Graphic_2 *) tmp_buffer->Data;
    tmp_data->Header = Referee_Interaction_Command_ID_UI_GRAPHIC_2;
    tmp_data->Sender = self->Referee_Rx_Data.Robot_Status.robot_id;
    tmp_data->Receiver = (Enum_Referee_Data_Robots_Client_ID)((int) self->Referee_Rx_Data.Robot_Status.robot_id + 0x100);

    // UI发两个图形帧内容
    tmp_data->Graphic[0] = *Graphic_1;
    tmp_data->Graphic[1] = *Graphic_2;

    tmp_data->CRC_16 = Verify_CRC_16((uint8_t *) tmp_buffer, 7 + tmp_buffer->Data_Length);

    HAL_UART_Transmit(self->UART_Manage_Object->UART_Handler, (uint8_t *) tmp_buffer, 7 + sizeof(Struct_Referee_Tx_Data_Interaction_Graphic_2), 100);

    self->Sequence++;
}

/* 发送 5 个图形 */
void Class_Referee_UART_Send_Interaction_UI_Graphic_5(Class_Referee *self,
    Struct_Referee_Data_Interaction_Graphic_Config *Graphic_1,
    Struct_Referee_Data_Interaction_Graphic_Config *Graphic_2,
    Struct_Referee_Data_Interaction_Graphic_Config *Graphic_3,
    Struct_Referee_Data_Interaction_Graphic_Config *Graphic_4,
    Struct_Referee_Data_Interaction_Graphic_Config *Graphic_5)
{
    Struct_Referee_UART_Data *tmp_buffer = (Struct_Referee_UART_Data *) self->UART_Manage_Object->Tx_Buffer;

    // 裁判系统帧头
    tmp_buffer->Frame_Header = 0xA5;
    tmp_buffer->Data_Length = sizeof(Struct_Referee_Tx_Data_Interaction_Graphic_5) - 2;
    tmp_buffer->Sequence = self->Sequence;
    tmp_buffer->CRC_8 = Verify_CRC_8((uint8_t *) tmp_buffer, 4);
    tmp_buffer->Referee_Command_ID = Referee_Command_ID_INTERACTION;

    // 交互帧头
    Struct_Referee_Tx_Data_Interaction_Graphic_5 *tmp_data = (Struct_Referee_Tx_Data_Interaction_Graphic_5 *) tmp_buffer->Data;
    tmp_data->Header = Referee_Interaction_Command_ID_UI_GRAPHIC_5;
    tmp_data->Sender = self->Referee_Rx_Data.Robot_Status.robot_id;
    tmp_data->Receiver = (Enum_Referee_Data_Robots_Client_ID)((int) self->Referee_Rx_Data.Robot_Status.robot_id + 0x100);

    // UI发五个图形帧内容
    tmp_data->Graphic[0] = *Graphic_1;
    tmp_data->Graphic[1] = *Graphic_2;
    tmp_data->Graphic[2] = *Graphic_3;
    tmp_data->Graphic[3] = *Graphic_4;
    tmp_data->Graphic[4] = *Graphic_5;

    tmp_data->CRC_16 = Verify_CRC_16((uint8_t *) tmp_buffer, 7 + tmp_buffer->Data_Length);

    HAL_UART_Transmit(self->UART_Manage_Object->UART_Handler, (uint8_t *) tmp_buffer, 7 + sizeof(Struct_Referee_Tx_Data_Interaction_Graphic_5), 100);

    self->Sequence++;
}

/* 发送 7 个图形 */
void Class_Referee_UART_Send_Interaction_UI_Graphic_7(Class_Referee *self,
    Struct_Referee_Data_Interaction_Graphic_Config *Graphic_1,
    Struct_Referee_Data_Interaction_Graphic_Config *Graphic_2,
    Struct_Referee_Data_Interaction_Graphic_Config *Graphic_3,
    Struct_Referee_Data_Interaction_Graphic_Config *Graphic_4,
    Struct_Referee_Data_Interaction_Graphic_Config *Graphic_5,
    Struct_Referee_Data_Interaction_Graphic_Config *Graphic_6,
    Struct_Referee_Data_Interaction_Graphic_Config *Graphic_7)
{
    Struct_Referee_UART_Data *tmp_buffer = (Struct_Referee_UART_Data *) self->UART_Manage_Object->Tx_Buffer;

    // 裁判系统帧头
    tmp_buffer->Frame_Header = 0xA5;
    tmp_buffer->Data_Length = sizeof(Struct_Referee_Tx_Data_Interaction_Graphic_7) - 2;
    tmp_buffer->Sequence = self->Sequence;
    tmp_buffer->CRC_8 = Verify_CRC_8((uint8_t *) tmp_buffer, 4);
    tmp_buffer->Referee_Command_ID = Referee_Command_ID_INTERACTION;

    // 交互帧头
    Struct_Referee_Tx_Data_Interaction_Graphic_7 *tmp_data = (Struct_Referee_Tx_Data_Interaction_Graphic_7 *) tmp_buffer->Data;
    tmp_data->Header = Referee_Interaction_Command_ID_UI_GRAPHIC_7;
    tmp_data->Sender = self->Referee_Rx_Data.Robot_Status.robot_id;
    tmp_data->Receiver = (Enum_Referee_Data_Robots_Client_ID)((int) self->Referee_Rx_Data.Robot_Status.robot_id + 0x100);

    // UI发七个图形帧内容
    tmp_data->Graphic[0] = *Graphic_1;
    tmp_data->Graphic[1] = *Graphic_2;
    tmp_data->Graphic[2] = *Graphic_3;
    tmp_data->Graphic[3] = *Graphic_4;
    tmp_data->Graphic[4] = *Graphic_5;
    tmp_data->Graphic[5] = *Graphic_6;
    tmp_data->Graphic[6] = *Graphic_7;

    tmp_data->CRC_16 = Verify_CRC_16((uint8_t *) tmp_buffer, 7 + tmp_buffer->Data_Length);

    HAL_UART_Transmit(self->UART_Manage_Object->UART_Handler, (uint8_t *) tmp_buffer, 7 + sizeof(Struct_Referee_Tx_Data_Interaction_Graphic_7), 100);

    self->Sequence++;
}

/**
 * @brief 设定裁判系统UI字符串图形
 *
 * @param self 裁判系统指针
 * @param Graphic_String 字符串图形配置指针
 * @param String_Content 字符串内容指针
 * @return 对应图层指针
 */
/* 发送字符串图形 */
void Class_Referee_UART_Send_Interaction_UI_Graphic_String(Class_Referee *self,
    Struct_Referee_Data_Interaction_Graphic_Config *Graphic_String,
    const char *String_Content)
{
    Struct_Referee_UART_Data *tmp_buffer = (Struct_Referee_UART_Data *) self->UART_Manage_Object->Tx_Buffer;

    // 裁判系统帧头
    tmp_buffer->Frame_Header = 0xA5;
    tmp_buffer->Data_Length = sizeof(Struct_Referee_Tx_Data_Interaction_Graphic_String) - 2;
    tmp_buffer->Sequence = self->Sequence;
    tmp_buffer->CRC_8 = Verify_CRC_8((uint8_t *) tmp_buffer, 4);
    tmp_buffer->Referee_Command_ID = Referee_Command_ID_INTERACTION;

    // 交互帧头
    Struct_Referee_Tx_Data_Interaction_Graphic_String *tmp_data = (Struct_Referee_Tx_Data_Interaction_Graphic_String *) tmp_buffer->Data;
    tmp_data->Header = Referee_Interaction_Command_ID_UI_GRAPHIC_STRING;
    tmp_data->Sender = self->Referee_Rx_Data.Robot_Status.robot_id;
    tmp_data->Receiver = (Enum_Referee_Data_Robots_Client_ID)((int) self->Referee_Rx_Data.Robot_Status.robot_id + 0x100);
    // UI发字符串帧内容
    tmp_data->Graphic_String = *Graphic_String;
    // 清空字符串缓冲区（用 memset 替代 bzero）
    memset(tmp_data->String, 0, sizeof(tmp_data->String));
    strcpy((char *) tmp_data->String, String_Content);

    tmp_data->CRC_16 = Verify_CRC_16((uint8_t *) tmp_buffer, 7 + tmp_buffer->Data_Length);

    HAL_UART_Transmit(self->UART_Manage_Object->UART_Handler, (uint8_t *) tmp_buffer, 7 + sizeof(Struct_Referee_Tx_Data_Interaction_Graphic_String), 80);

    self->Sequence++;
}



/* 生成删除图形的配置 */
Struct_Referee_Data_Interaction_Graphic_Config *Class_Referee_Set_Referee_UI_Clear(Class_Referee *self, uint8_t Layer_Num, uint8_t Graphic_Num)
{
    self->Graphic_Config[Layer_Num][Graphic_Num].Index[0] = '0';
    self->Graphic_Config[Layer_Num][Graphic_Num].Index[1] = '0' + Layer_Num;
    self->Graphic_Config[Layer_Num][Graphic_Num].Index[2] = '0' + Graphic_Num;
    self->Graphic_Config[Layer_Num][Graphic_Num].Operation_Enum = Referee_Data_Interaction_Graphic_Operation_DELETE;
    self->UI_Change_Flag[Layer_Num][Graphic_Num] = 0;
    return &self->Graphic_Config[Layer_Num][Graphic_Num];
}

/**
 * @brief 设定裁判系统UI直线
 *
 * @param self 裁判系统指针
 * @param Layer_Num 图层编号, 0~9
 * @param Graphic_Num 图形编号, 0~9
 * @param Color 图形颜色
 * @param Line_Width 线宽
 * @param Start_X 起点x
 * @param Start_Y 起点y
 * @param End_X 终点x
 * @param End_Y 终点y
 * @return 对应图层指针
 */
/* 生成直线图形配置 */
Struct_Referee_Data_Interaction_Graphic_Config *Class_Referee_Set_Referee_UI_Line(
    Class_Referee *self,
    uint8_t Layer_Num, uint8_t Graphic_Num,
    Enum_Referee_Data_Interaction_Graphic_Color Color,
    uint32_t Line_Width,
    uint32_t Start_X, uint32_t Start_Y,
    uint32_t End_X, uint32_t End_Y)
{
    self->Graphic_Config[Layer_Num][Graphic_Num].Index[0] = '0';
    self->Graphic_Config[Layer_Num][Graphic_Num].Index[1] = '0' + Layer_Num;
    self->Graphic_Config[Layer_Num][Graphic_Num].Index[2] = '0' + Graphic_Num;

    if (self->UI_Change_Flag[Layer_Num][Graphic_Num] == 0)
    {
        self->Graphic_Config[Layer_Num][Graphic_Num].Operation_Enum = Referee_Data_Interaction_Graphic_Operation_ADD;
        self->UI_Change_Flag[Layer_Num][Graphic_Num] = 1;
    }
    else
    {
        self->Graphic_Config[Layer_Num][Graphic_Num].Operation_Enum = Referee_Data_Interaction_Graphic_Operation_CHANGE;
    }

    self->Graphic_Config[Layer_Num][Graphic_Num].Type_Enum = Referee_Data_Interaction_Graphic_Type_LINE;
    self->Graphic_Config[Layer_Num][Graphic_Num].Layer_Num = Layer_Num;
    self->Graphic_Config[Layer_Num][Graphic_Num].Color_Enum = Color;
    self->Graphic_Config[Layer_Num][Graphic_Num].Line_Width = Line_Width;
    self->Graphic_Config[Layer_Num][Graphic_Num].Start_X = Start_X;
    self->Graphic_Config[Layer_Num][Graphic_Num].Start_Y = Start_Y;
    self->Graphic_Config[Layer_Num][Graphic_Num].Details_D = End_X;
    self->Graphic_Config[Layer_Num][Graphic_Num].Details_E = End_Y;

    return &self->Graphic_Config[Layer_Num][Graphic_Num];
}


/**
 *
 * @brief 设定裁判系统UI矩形
 *
 * @param self 裁判系统指针
 * @param Layer_Num 图层编号, 0~9
 * @param Graphic_Num 图形编号, 0~9
 * @param Color 图形颜色
 * @param Line_Width 线宽
 * @param Start_X 起点x
 * @param Start_Y 起点y
 * @param End_X 终点x
 * @param End_Y 终点y
 * @return 对应图层指针
 */
/* 生成矩形图形配置 */
Struct_Referee_Data_Interaction_Graphic_Config *Class_Referee_Set_Referee_UI_Rectangle(
    Class_Referee *self,
    uint8_t Layer_Num, uint8_t Graphic_Num,
    Enum_Referee_Data_Interaction_Graphic_Color Color,
    uint32_t Line_Width,
    uint32_t Start_X, uint32_t Start_Y,
    uint32_t End_X, uint32_t End_Y)
{
    self->Graphic_Config[Layer_Num][Graphic_Num].Index[0] = '0';
    self->Graphic_Config[Layer_Num][Graphic_Num].Index[1] = '0' + Layer_Num;
    self->Graphic_Config[Layer_Num][Graphic_Num].Index[2] = '0' + Graphic_Num;

    if (self->UI_Change_Flag[Layer_Num][Graphic_Num] == 0)
    {
        self->Graphic_Config[Layer_Num][Graphic_Num].Operation_Enum = Referee_Data_Interaction_Graphic_Operation_ADD;
        self->UI_Change_Flag[Layer_Num][Graphic_Num] = 1;
    }
    else
    {
        self->Graphic_Config[Layer_Num][Graphic_Num].Operation_Enum = Referee_Data_Interaction_Graphic_Operation_CHANGE;
    }

    self->Graphic_Config[Layer_Num][Graphic_Num].Type_Enum = Referee_Data_Interaction_Graphic_Type_RECTANGLE;
    self->Graphic_Config[Layer_Num][Graphic_Num].Layer_Num = Layer_Num;
    self->Graphic_Config[Layer_Num][Graphic_Num].Color_Enum = Color;
    self->Graphic_Config[Layer_Num][Graphic_Num].Line_Width = Line_Width;
    self->Graphic_Config[Layer_Num][Graphic_Num].Start_X = Start_X;
    self->Graphic_Config[Layer_Num][Graphic_Num].Start_Y = Start_Y;
    self->Graphic_Config[Layer_Num][Graphic_Num].Details_D = End_X;
    self->Graphic_Config[Layer_Num][Graphic_Num].Details_E = End_Y;

    return &self->Graphic_Config[Layer_Num][Graphic_Num];
}

/**
 *
 * @brief 设定裁判系统UI圆形
 *
 * @param self 裁判系统指针
 * @param Layer_Num 图层编号, 0~9
 * @param Graphic_Num 图形编号, 0~9
 * @param Color 图形颜色
 * @param Line_Width 线宽
 * @param Center_X 圆心x
 * @param Center_Y 圆心y
 * @param Radius 半径
 * @return 对应图层指针
 */
/* 生成圆形图形配置 */
Struct_Referee_Data_Interaction_Graphic_Config *Class_Referee_Set_Referee_UI_Circle(
    Class_Referee *self,
    uint8_t Layer_Num, uint8_t Graphic_Num,
    Enum_Referee_Data_Interaction_Graphic_Color Color,
    uint32_t Line_Width,
    uint32_t Center_X, uint32_t Center_Y,
    uint32_t Radius)
{
    self->Graphic_Config[Layer_Num][Graphic_Num].Index[0] = '0';
    self->Graphic_Config[Layer_Num][Graphic_Num].Index[1] = '0' + Layer_Num;
    self->Graphic_Config[Layer_Num][Graphic_Num].Index[2] = '0' + Graphic_Num;

    if (self->UI_Change_Flag[Layer_Num][Graphic_Num] == 0)
    {
        self->Graphic_Config[Layer_Num][Graphic_Num].Operation_Enum = Referee_Data_Interaction_Graphic_Operation_ADD;
        self->UI_Change_Flag[Layer_Num][Graphic_Num] = 1;
    }
    else
    {
        self->Graphic_Config[Layer_Num][Graphic_Num].Operation_Enum = Referee_Data_Interaction_Graphic_Operation_CHANGE;
    }

    self->Graphic_Config[Layer_Num][Graphic_Num].Type_Enum = Referee_Data_Interaction_Graphic_Type_CIRCLE;
    self->Graphic_Config[Layer_Num][Graphic_Num].Layer_Num = Layer_Num;
    self->Graphic_Config[Layer_Num][Graphic_Num].Color_Enum = Color;
    self->Graphic_Config[Layer_Num][Graphic_Num].Line_Width = Line_Width;
    self->Graphic_Config[Layer_Num][Graphic_Num].Start_X = Center_X;
    self->Graphic_Config[Layer_Num][Graphic_Num].Start_Y = Center_Y;
    self->Graphic_Config[Layer_Num][Graphic_Num].Details_C = Radius;

    return &self->Graphic_Config[Layer_Num][Graphic_Num];
}

/**
 * @brief 设定裁判系统UI椭圆形
 *
 * @param self 裁判系统指针
 * @param Layer_Num 图层编号, 0~9
 * @param Graphic_Num 图形编号, 0~9
 * @param Color 图形颜色
 * @param Line_Width 线宽
 * @param Center_X 圆心x
 * @param Center_Y 圆心y
 * @param Length_X x半轴长度
 * @param Length_Y y半轴长度
 * @return 对应图层指针
 */
/* 生成椭圆形图形配置 */
Struct_Referee_Data_Interaction_Graphic_Config *Class_Referee_Set_Referee_UI_Oval(
    Class_Referee *self,
    uint8_t Layer_Num, uint8_t Graphic_Num,
    Enum_Referee_Data_Interaction_Graphic_Color Color,
    uint32_t Line_Width,
    uint32_t Center_X, uint32_t Center_Y,
    uint32_t Length_X, uint32_t Length_Y)
{
    self->Graphic_Config[Layer_Num][Graphic_Num].Index[0] = '0';
    self->Graphic_Config[Layer_Num][Graphic_Num].Index[1] = '0' + Layer_Num;
    self->Graphic_Config[Layer_Num][Graphic_Num].Index[2] = '0' + Graphic_Num;

    if (self->UI_Change_Flag[Layer_Num][Graphic_Num] == 0)
    {
        self->Graphic_Config[Layer_Num][Graphic_Num].Operation_Enum = Referee_Data_Interaction_Graphic_Operation_ADD;
        self->UI_Change_Flag[Layer_Num][Graphic_Num] = 1;
    }
    else
    {
        self->Graphic_Config[Layer_Num][Graphic_Num].Operation_Enum = Referee_Data_Interaction_Graphic_Operation_CHANGE;
    }

    self->Graphic_Config[Layer_Num][Graphic_Num].Type_Enum = Referee_Data_Interaction_Graphic_Type_OVAL;
    self->Graphic_Config[Layer_Num][Graphic_Num].Layer_Num = Layer_Num;
    self->Graphic_Config[Layer_Num][Graphic_Num].Color_Enum = Color;
    self->Graphic_Config[Layer_Num][Graphic_Num].Line_Width = Line_Width;
    self->Graphic_Config[Layer_Num][Graphic_Num].Start_X = Center_X;
    self->Graphic_Config[Layer_Num][Graphic_Num].Start_Y = Center_Y;
    self->Graphic_Config[Layer_Num][Graphic_Num].Details_D = Length_X;
    self->Graphic_Config[Layer_Num][Graphic_Num].Details_E = Length_Y;

    return &self->Graphic_Config[Layer_Num][Graphic_Num];
}

/**
 * @brief 设定裁判系统UI圆弧形
 *
 * @param self 裁判系统指针
 * @param Layer_Num 图层编号, 0~9
 * @param Graphic_Num 图形编号, 0~9
 * @param Color 图形颜色
 * @param Line_Width 线宽
 * @param Center_X 圆心x
 * @param Center_Y 圆心y
 * @param Angle_Start 起始角度
 * @param Angle_End 终止角度
 * @param Length_X x半轴长度
 * @param Length_Y y半轴长度
 * @return 对应图层指针
 */
/* 生成圆弧形图形配置 */
Struct_Referee_Data_Interaction_Graphic_Config *Class_Referee_Set_Referee_UI_Arc(
    Class_Referee *self,
    uint8_t Layer_Num, uint8_t Graphic_Num,
    Enum_Referee_Data_Interaction_Graphic_Color Color,
    uint32_t Line_Width,
    uint32_t Center_X, uint32_t Center_Y,
    uint32_t Angle_Start, uint32_t Angle_End,
    uint32_t Length_X, uint32_t Length_Y)
{
    self->Graphic_Config[Layer_Num][Graphic_Num].Index[0] = '0';
    self->Graphic_Config[Layer_Num][Graphic_Num].Index[1] = '0' + Layer_Num;
    self->Graphic_Config[Layer_Num][Graphic_Num].Index[2] = '0' + Graphic_Num;

    if (self->UI_Change_Flag[Layer_Num][Graphic_Num] == 0)
    {
        self->Graphic_Config[Layer_Num][Graphic_Num].Operation_Enum = Referee_Data_Interaction_Graphic_Operation_ADD;
        self->UI_Change_Flag[Layer_Num][Graphic_Num] = 1;
    }
    else
    {
        self->Graphic_Config[Layer_Num][Graphic_Num].Operation_Enum = Referee_Data_Interaction_Graphic_Operation_CHANGE;
    }

    self->Graphic_Config[Layer_Num][Graphic_Num].Type_Enum = Referee_Data_Interaction_Graphic_Type_ARC;
    self->Graphic_Config[Layer_Num][Graphic_Num].Layer_Num = Layer_Num;
    self->Graphic_Config[Layer_Num][Graphic_Num].Color_Enum = Color;
    self->Graphic_Config[Layer_Num][Graphic_Num].Line_Width = Line_Width;
    self->Graphic_Config[Layer_Num][Graphic_Num].Start_X = Center_X;
    self->Graphic_Config[Layer_Num][Graphic_Num].Start_Y = Center_Y;
    self->Graphic_Config[Layer_Num][Graphic_Num].Details_A = Angle_Start;
    self->Graphic_Config[Layer_Num][Graphic_Num].Details_B = Angle_End;
    self->Graphic_Config[Layer_Num][Graphic_Num].Details_D = Length_X;
    self->Graphic_Config[Layer_Num][Graphic_Num].Details_E = Length_Y;

    return &self->Graphic_Config[Layer_Num][Graphic_Num];
}

/**
 * @brief 设定裁判系统UI浮点数
 *
 * @param self 裁判系统指针
 * @param Layer_Num 图层编号, 0~9
 * @param Graphic_Num 图形编号, 0~9
 * @param Color 图形颜色
 * @param Line_Width 线宽
 * @param Start_X 起点x
 * @param Start_Y 起点y
 * @param Font_Width 字体大小
 * @param Float 数值
 * @return 对应图层指针
 */
/* 生成浮点数显示图形配置 */
Struct_Referee_Data_Interaction_Graphic_Config *Class_Referee_Set_Referee_UI_Float(
    Class_Referee *self,
    uint8_t Layer_Num, uint8_t Graphic_Num,
    Enum_Referee_Data_Interaction_Graphic_Color Color,
    uint32_t Line_Width,
    uint32_t Start_X, uint32_t Start_Y,
    uint32_t Font_Width,
    float Float)
{
    self->Graphic_Config[Layer_Num][Graphic_Num].Index[0] = '0';
    self->Graphic_Config[Layer_Num][Graphic_Num].Index[1] = '0' + Layer_Num;
    self->Graphic_Config[Layer_Num][Graphic_Num].Index[2] = '0' + Graphic_Num;

    if (self->UI_Change_Flag[Layer_Num][Graphic_Num] == 0)
    {
        self->Graphic_Config[Layer_Num][Graphic_Num].Operation_Enum = Referee_Data_Interaction_Graphic_Operation_ADD;
        self->UI_Change_Flag[Layer_Num][Graphic_Num] = 1;
    }
    else
    {
        self->Graphic_Config[Layer_Num][Graphic_Num].Operation_Enum = Referee_Data_Interaction_Graphic_Operation_CHANGE;
    }

    self->Graphic_Config[Layer_Num][Graphic_Num].Type_Enum = Referee_Data_Interaction_Graphic_Type_FLOAT;
    self->Graphic_Config[Layer_Num][Graphic_Num].Layer_Num = Layer_Num;
    self->Graphic_Config[Layer_Num][Graphic_Num].Color_Enum = Color;
    self->Graphic_Config[Layer_Num][Graphic_Num].Line_Width = Line_Width;
    self->Graphic_Config[Layer_Num][Graphic_Num].Start_X = Start_X;
    self->Graphic_Config[Layer_Num][Graphic_Num].Start_Y = Start_Y;
    self->Graphic_Config[Layer_Num][Graphic_Num].Details_A = Font_Width;
    int32_t *tmp_pointer = (int32_t *)((uint32_t)&self->Graphic_Config[Layer_Num][Graphic_Num] + 11);
    *tmp_pointer = (int32_t)(Float * 1000.0f);

    return &self->Graphic_Config[Layer_Num][Graphic_Num];
}

/**
 * @brief 设定裁判系统UI整型数
 *
 * @param self 裁判系统指针
 * @param Layer_Num 图层编号, 0~9
 * @param Graphic_Num 图形编号, 0~9
 * @param Color 图形颜色
 * @param Line_Width 线宽
 * @param Start_X 起点x
 * @param Start_Y 起点y
 * @param Font_Width 字体大小
 * @param Integer 数值
 * @return 对应图层指针
 */
/* 生成整数显示图形配置 */
Struct_Referee_Data_Interaction_Graphic_Config *Class_Referee_Set_Referee_UI_Integer(
    Class_Referee *self,
    uint8_t Layer_Num, uint8_t Graphic_Num,
    Enum_Referee_Data_Interaction_Graphic_Color Color,
    uint32_t Line_Width,
    uint32_t Start_X, uint32_t Start_Y,
    uint32_t Font_Width,
    int32_t Integer)
{
    self->Graphic_Config[Layer_Num][Graphic_Num].Index[0] = '0';
    self->Graphic_Config[Layer_Num][Graphic_Num].Index[1] = '0' + Layer_Num;
    self->Graphic_Config[Layer_Num][Graphic_Num].Index[2] = '0' + Graphic_Num;

    if (self->UI_Change_Flag[Layer_Num][Graphic_Num] == 0)
    {
        self->Graphic_Config[Layer_Num][Graphic_Num].Operation_Enum = Referee_Data_Interaction_Graphic_Operation_ADD;
        self->UI_Change_Flag[Layer_Num][Graphic_Num] = 1;
    }
    else
    {
        self->Graphic_Config[Layer_Num][Graphic_Num].Operation_Enum = Referee_Data_Interaction_Graphic_Operation_CHANGE;
    }

    self->Graphic_Config[Layer_Num][Graphic_Num].Type_Enum = Referee_Data_Interaction_Graphic_Type_INTEGER;
    self->Graphic_Config[Layer_Num][Graphic_Num].Layer_Num = Layer_Num;
    self->Graphic_Config[Layer_Num][Graphic_Num].Color_Enum = Color;
    self->Graphic_Config[Layer_Num][Graphic_Num].Line_Width = Line_Width;
    self->Graphic_Config[Layer_Num][Graphic_Num].Start_X = Start_X;
    self->Graphic_Config[Layer_Num][Graphic_Num].Start_Y = Start_Y;
    self->Graphic_Config[Layer_Num][Graphic_Num].Details_A = Font_Width;
    int32_t *tmp_pointer = (int32_t *)((uint32_t)&self->Graphic_Config[Layer_Num][Graphic_Num] + 11);
    *tmp_pointer = Integer;

    return &self->Graphic_Config[Layer_Num][Graphic_Num];
}

/**
 * @brief 设定裁判系统UI字符串
 *
 * @param self 裁判系统指针
 * @param Layer_Num 图层编号, 0~9
 * @param Graphic_Num 图形编号, 0~9
 * @param Color 图形颜色
 * @param Line_Width 线宽
 * @param Start_X 起点x
 * @param Start_Y 起点y
 * @param Font_Width 字体大小
 * @param String_Length 字符串长度
 * @return 对应图层指针
 */
/* 生成字符串显示图形配置（仅配置，字符串内容在发送时填充） */
Struct_Referee_Data_Interaction_Graphic_Config *Class_Referee_Set_Referee_UI_String(
    Class_Referee *self,
    uint8_t Layer_Num, uint8_t Graphic_Num,
    Enum_Referee_Data_Interaction_Graphic_Color Color,
    uint32_t Line_Width,
    uint32_t Start_X, uint32_t Start_Y,
    uint32_t Font_Width,
    uint32_t String_Length)
{
    self->Graphic_Config[Layer_Num][Graphic_Num].Index[0] = '0';
    self->Graphic_Config[Layer_Num][Graphic_Num].Index[1] = '0' + Layer_Num;
    self->Graphic_Config[Layer_Num][Graphic_Num].Index[2] = '0' + Graphic_Num;

    if (self->UI_Change_Flag[Layer_Num][Graphic_Num] == 0)
    {
        self->Graphic_Config[Layer_Num][Graphic_Num].Operation_Enum = Referee_Data_Interaction_Graphic_Operation_ADD;
        self->UI_Change_Flag[Layer_Num][Graphic_Num] = 1;
    }
    else
    {
        self->Graphic_Config[Layer_Num][Graphic_Num].Operation_Enum = Referee_Data_Interaction_Graphic_Operation_CHANGE;
    }

    self->Graphic_Config[Layer_Num][Graphic_Num].Type_Enum = Referee_Data_Interaction_Graphic_Type_STRING;
    self->Graphic_Config[Layer_Num][Graphic_Num].Layer_Num = Layer_Num;
    self->Graphic_Config[Layer_Num][Graphic_Num].Color_Enum = Color;
    self->Graphic_Config[Layer_Num][Graphic_Num].Line_Width = Line_Width;
    self->Graphic_Config[Layer_Num][Graphic_Num].Start_X = Start_X;
    self->Graphic_Config[Layer_Num][Graphic_Num].Start_Y = Start_Y;
    self->Graphic_Config[Layer_Num][Graphic_Num].Details_A = Font_Width;
    self->Graphic_Config[Layer_Num][Graphic_Num].Details_B = String_Length;

    return &self->Graphic_Config[Layer_Num][Graphic_Num];
}

/* 其他发送函数类似，省略 */

/* 回调函数 */
void Class_Referee_UART_RxCpltCallback(Class_Referee *self,uint8_t *Rx_Data, uint16_t Length)
{
    self->Flag += 1;
    Class_Referee_Data_Process(self,Length);
}

void Class_Referee_TIM_1000ms_Alive_PeriodElapsedCallback(Class_Referee *self)
{
    if (self->Flag == self->Pre_Flag)
    {
        self->Referee_Status = Referee_Status_DISABLE;
        UART_Reinit(self->UART_Manage_Object->UART_Handler);
        // 根据实际需要调用 UART 重初始化,比如掉线重新初始化UART
    }
    else
    {
        self->Referee_Status = Referee_Status_ENABLE;
    }
    self->Pre_Flag = self->Flag;
}

/* 私有函数 */
static uint8_t Verify_CRC_8(uint8_t *Message, uint32_t Length)
{
    uint8_t index;
    uint8_t check = 0xff;

    if (Message == NULL)
    {
        return check;
    }

    while (Length--)
    {
        index = *Message;
        Message++;
        check = crc_8_table[check ^ index];
    }
    return check;
}

static uint16_t Verify_CRC_16(uint8_t *Message, uint32_t Length)
{
    uint8_t index;
    uint16_t check = 0xffff;

    if (Message == NULL)
    {
        return check;
    }

    while (Length--)
    {
        index = *Message;
        Message++;
        check = ((uint16_t)(check) >> 8) ^ crc_16_table[((uint16_t)(check) ^ (uint16_t)(index)) & 0xff];
    }
    return check;
}

void Class_Referee_Data_Process(Class_Referee *self, uint16_t Length)
{
    Struct_Referee_UART_Data *tmp_buffer;

    for (int i = 0; i < Length;)
    {
        tmp_buffer = (Struct_Referee_UART_Data *)&self->UART_Manage_Object->Rx_Buffer[i];

        if (tmp_buffer->Frame_Header != self->Frame_Header)
        {
            i++;
            continue;
        }
        if (Verify_CRC_8((uint8_t *)tmp_buffer, 4) != tmp_buffer->CRC_8)
        {
            i++;
            continue;
        }
        if (Verify_CRC_16((uint8_t *)tmp_buffer, 7 + tmp_buffer->Data_Length) != *(uint16_t *)((uint32_t)tmp_buffer + 7 + tmp_buffer->Data_Length))
        {
            i += 9 + tmp_buffer->Data_Length;
            continue;
        }
        if (i + 7 + tmp_buffer->Data_Length + 2 > Length)
        {
            break;
        }

        switch (tmp_buffer->Referee_Command_ID)
        {
        case Referee_Command_ID_GAME_STATUS:
            memcpy(&self->Referee_Rx_Data.Game_Status, tmp_buffer->Data, sizeof(Struct_Referee_Rx_Data_Game_Status));
            break;
        case Referee_Command_ID_GAME_RESULT:
            memcpy(&self->Referee_Rx_Data.Game_Result, tmp_buffer->Data, sizeof(Struct_Referee_Rx_Data_Game_Result));
            break;
        case Referee_Command_ID_GAME_ROBOT_HP:
            memcpy(&self->Referee_Rx_Data.Game_Robot_HP, tmp_buffer->Data, sizeof(Struct_Referee_Rx_Data_Game_Robot_HP));
            break;
        case Referee_Command_ID_EVENT_SELF_DATA:
            memcpy(&self->Referee_Rx_Data.Event_Self_Data, tmp_buffer->Data, sizeof(Struct_Referee_Rx_Data_Event_Self_Data));
            break;
        case Referee_Command_ID_EVENT_SELF_REFEREE_WARNING:
            memcpy(&self->Referee_Rx_Data.Event_Referee_Warning, tmp_buffer->Data, sizeof(Struct_Referee_Rx_Data_Event_Referee_Warning));
            break;
        case Referee_Command_ID_EVENT_SELF_DART_STATUS:
            memcpy(&self->Referee_Rx_Data.Event_Dart_Status, tmp_buffer->Data, sizeof(Struct_Referee_Rx_Data_Event_Dart_Status));
            break;
        case Referee_Command_ID_ROBOT_STATUS:
            memcpy(&self->Referee_Rx_Data.Robot_Status, tmp_buffer->Data, sizeof(Struct_Referee_Rx_Data_Robot_Status));
            break;
        case Referee_Command_ID_ROBOT_POWER_HEAT:
            memcpy(&self->Referee_Rx_Data.Robot_Power_Heat, tmp_buffer->Data, sizeof(Struct_Referee_Rx_Data_Robot_Power_Heat));
            break;
        case Referee_Command_ID_ROBOT_POSITION:
            memcpy(&self->Referee_Rx_Data.Robot_Position, tmp_buffer->Data, sizeof(Struct_Referee_Rx_Data_Robot_Position));
            break;
        case Referee_Command_ID_ROBOT_BUFF:
            memcpy(&self->Referee_Rx_Data.Robot_Buff, tmp_buffer->Data, sizeof(Struct_Referee_Rx_Data_Robot_Buff));
            break;
        case Referee_Command_ID_ROBOT_DAMAGE:
            memcpy(&self->Referee_Rx_Data.Robot_Damage, tmp_buffer->Data, sizeof(Struct_Referee_Rx_Data_Robot_Damage));
            break;
        case Referee_Command_ID_ROBOT_BOOSTER:
            memcpy(&self->Referee_Rx_Data.Robot_Booster, tmp_buffer->Data, sizeof(Struct_Referee_Rx_Data_Robot_Booster));
            break;
        case Referee_Command_ID_ROBOT_REMAINING_AMMO:
            memcpy(&self->Referee_Rx_Data.Robot_Remaining_Ammo, tmp_buffer->Data, sizeof(Struct_Referee_Rx_Data_Robot_Remaining_Ammo));
            break;
        case Referee_Command_ID_ROBOT_RFID:
            memcpy(&self->Referee_Rx_Data.Robot_RFID, tmp_buffer->Data, sizeof(Struct_Referee_Rx_Data_Robot_RFID));
            break;
        case Referee_Command_ID_ROBOT_DART_COMMAND:
            memcpy(&self->Referee_Rx_Data.Robot_Dart_Command, tmp_buffer->Data, sizeof(Struct_Referee_Rx_Data_Robot_Dart_Command));
            break;
        case Referee_Command_ID_ROBOT_SENTRY_LOCATION:
            memcpy(&self->Referee_Rx_Data.Robot_Sentry_Location, tmp_buffer->Data, sizeof(Struct_Referee_Rx_Data_Robot_Sentry_Location));
            break;
        case Referee_Command_ID_ROBOT_RADAR_MARK:
            memcpy(&self->Referee_Rx_Data.Robot_Radar_Mark, tmp_buffer->Data, sizeof(Struct_Referee_Rx_Data_Robot_Radar_Mark));
            break;
        case Referee_Command_ID_ROBOT_SENTRY_DECISION:
            memcpy(&self->Referee_Rx_Data.Robot_Sentry_Decision, tmp_buffer->Data, sizeof(Struct_Referee_Rx_Data_Robot_Sentry_Decision));
            break;
        case Referee_Command_ID_ROBOT_RADAR_DECISION:
            memcpy(&self->Referee_Rx_Data.Robot_Radar_Decision, tmp_buffer->Data, sizeof(Struct_Referee_Rx_Data_Robot_Radar_Decision));
            break;
        default:
            break;
        }

        i += 7 + tmp_buffer->Data_Length + 2;
    }
}