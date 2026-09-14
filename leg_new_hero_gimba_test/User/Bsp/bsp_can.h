//
// Created by 25031 on 2026/4/4.
//

#ifndef CTRLBOARD_H7_IMU_BSP_CAN_H
#define CTRLBOARD_H7_IMU_BSP_CAN_H

#include "main.h"
#include "fdcan.h"

#define CAN_CLASS   0
#define CAN_FD_BRS  1

#define CAN_BR_125K 0
#define CAN_BR_200K 1
#define CAN_BR_250K 2
#define CAN_BR_500K 3
#define CAN_BR_1M   4
#define CAN_BR_2M   5
#define CAN_BR_2M5  6
#define CAN_BR_3M2  7
#define CAN_BR_4M   8
#define CAN_BR_5M   9

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2

// 滤波器编号
#define CAN_FILTER(x) ((x) << 3)

// 接收队列
#define CAN_FIFO_0 (0 << 2)
#define CAN_FIFO_1 (1 << 2)

//标准帧或扩展帧
#define CAN_STDID (0 << 1)
#define CAN_EXTID (1 << 1)

// 数据帧或遥控帧
#define CAN_DATA_TYPE (0 << 0)
#define CAN_REMOTE_TYPE (1 << 0)

/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_GIMBAL_ALL_ID = 0x1FF,

} can_msg_id_e;

typedef enum
{
    // 上板数据ID,下板收
    CAN_COMM_DOWN_Control_ID_1 = 0x520,
    CAN_COMM_DOWN_Control_ID_2 = 0x521,
    CAN_COMM_DOWN_Control_ID_3 = 0x522,

    // 下板数据ID,上板收
    CAN_COMM_UP_Control_ID = 0x620,
    CAN_COMM_UP_Referee_ID = 0x621,
} can_comm_id_e;

//rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

extern uint8_t CAN_Send_Data(FDCAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length);

typedef struct
{
	FDCAN_RxHeaderTypeDef Header;
	uint8_t Data[8];
}Struct_CAN_Rx_Buffer;

/**
 * @brief CAN通信接收回调函数数据类型
 *
 */
typedef void (*CAN_Call_Back)(Struct_CAN_Rx_Buffer *);

typedef struct
{
	FDCAN_HandleTypeDef *CAN_Handler;
	Struct_CAN_Rx_Buffer Rx_Buffer;
	CAN_Call_Back Callback_Function;
}Struct_CAN_Manage_Object;

extern Struct_CAN_Manage_Object CAN1_Manage_Object;
extern Struct_CAN_Manage_Object CAN2_Manage_Object;
extern Struct_CAN_Manage_Object CAN3_Manage_Object;


extern uint8_t CAN1_0x1ff_Tx_Data[];
extern uint8_t CAN1_0x200_Tx_Data[];
extern uint8_t CAN1_0x2ff_Tx_Data[];
extern uint8_t CAN1_0x3fe_Tx_Data[];
extern uint8_t CAN1_0x4fe_Tx_Data[];

extern uint8_t CAN2_0x1ff_Tx_Data[];
extern uint8_t CAN2_0x200_Tx_Data[];
extern uint8_t CAN2_0x2ff_Tx_Data[];
extern uint8_t CAN2_0x3fe_Tx_Data[];
extern uint8_t CAN2_0x4fe_Tx_Data[];

extern uint8_t CAN3_0x1ff_Tx_Data[];
extern uint8_t CAN3_0x200_Tx_Data[];
extern uint8_t CAN3_0x2ff_Tx_Data[];
extern uint8_t CAN3_0x3fe_Tx_Data[];
extern uint8_t CAN3_0x4fe_Tx_Data[];

//双板通信发送缓冲区
extern uint8_t CAN3_0xA1_Chassis_Control_Tx_Data[8];
extern uint8_t CAN3_0xA2_DT7_1_Tx_Data[8];
extern uint8_t CAN3_0xA3_DT7_2_Tx_Data[8];
extern uint8_t CAN3_0xA4_VT13_1_Tx_Data[8];
extern uint8_t CAN3_0xA5_VT13_2_Tx_Data[8];
extern uint8_t CAN3_0xA6_Fusi_1_Tx_Data[8];
extern uint8_t CAN3_0xA7_Fusi_2_Tx_Data[8];
extern uint8_t CAN_Board_Tx_Data[8];

void CAN_Init(FDCAN_HandleTypeDef *hcan, CAN_Call_Back Callback_Function);
void CAN_Filter_Mask_Config(FDCAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID);
void TIM_CAN_PeriodElapsedCallback(void);
void FDCAN_Set_Baud(FDCAN_HandleTypeDef *hfdcan, uint8_t mode, uint8_t baud);

#endif //CTRLBOARD_H7_IMU_BSP_CAN_H