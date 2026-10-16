#ifndef BSP_USART_H
#define BSP_USART_H
/* Includes ------------------------------------------------------------------*/

#include "stm32h7xx_hal.h"
#include "usart.h"
#include <string.h>

/* Exported macros -----------------------------------------------------------*/
// 缓冲区模式定义
#define UART_MODE_SINGLE_BUF 0  // 单缓冲区
#define UART_MODE_DOUBLE_BUF 1  // 双缓冲区

// 缓冲区字节长度
#define UART_BUFFER_SIZE 512

/* Exported types ------------------------------------------------------------*/

/**
 * @brief UART通信接收回调函数数据类型
 *
 */
typedef void (*UART_Call_Back)(uint8_t *Buffer, uint16_t Length);

/**
 * @brief UART通信处理结构体
 */
typedef struct
{
	UART_HandleTypeDef *UART_Handler;
	uint8_t Buffer_Mode;  // 0:单缓冲区 1:双缓冲区
	uint8_t Tx_Buffer[UART_BUFFER_SIZE];
	// 单缓冲区
	uint8_t Rx_Buffer[UART_BUFFER_SIZE];
	// 双缓冲区
	uint8_t Rx_Double_Buf[2][UART_BUFFER_SIZE];
	uint8_t Current_Buf_Idx;  // 当前活跃的双缓冲区索引

	uint16_t Rx_Buffer_Length;
	UART_Call_Back Callback_Function;
}Struct_UART_Manage_Object;

/* Exported variables --------------------------------------------------------*/

//extern bool init_finished;

extern Struct_UART_Manage_Object UART1_Manage_Object;
extern Struct_UART_Manage_Object UART2_Manage_Object;
extern Struct_UART_Manage_Object UART3_Manage_Object;
extern Struct_UART_Manage_Object UART4_Manage_Object;
extern Struct_UART_Manage_Object UART5_Manage_Object;
extern Struct_UART_Manage_Object UART6_Manage_Object;
extern Struct_UART_Manage_Object UART7_Manage_Object;
extern Struct_UART_Manage_Object UART8_Manage_Object;
extern Struct_UART_Manage_Object UART9_Manage_Object;
extern Struct_UART_Manage_Object UART10_Manage_Object;

/* Exported function declarations --------------------------------------------*/

void UART_Init(UART_HandleTypeDef *huart, UART_Call_Back Callback_Function, uint16_t Rx_Buffer_Length);
void UART_DoubleBuf_Init(UART_HandleTypeDef *huart, UART_Call_Back Callback_Function, uint16_t Rx_Buffer_Length);

void UART_Reinit(UART_HandleTypeDef *huart);

uint8_t UART_Send_Data(UART_HandleTypeDef *huart, uint8_t *Data, uint16_t Length);

void TIM_1ms_UART_PeriodElapsedCallback();

#endif
