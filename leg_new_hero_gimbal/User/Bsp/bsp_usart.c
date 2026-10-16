/**
 * @file drv_uart.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 仿照SCUT-Robotlab改写的UART通信初始化与配置流程
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2023-11-18 1.1 修改成cpp
 * @date 2024-05-05 1.2 新增错误中断, 24赛季定稿
 * @date 2024-08-22 2.1 新增回调函数空指针判定
 *
 * @copyright USTC-RoboWalker (c) 2023-2024
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_usart.h"
#include "task_config_and_callback.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

Struct_UART_Manage_Object UART1_Manage_Object = {0};
Struct_UART_Manage_Object UART2_Manage_Object = {0};
Struct_UART_Manage_Object UART3_Manage_Object = {0};
Struct_UART_Manage_Object UART4_Manage_Object = {0};
Struct_UART_Manage_Object UART5_Manage_Object = {0};
Struct_UART_Manage_Object UART6_Manage_Object = {0};
Struct_UART_Manage_Object UART7_Manage_Object = {0};
Struct_UART_Manage_Object UART8_Manage_Object = {0};
Struct_UART_Manage_Object UART9_Manage_Object = {0};
Struct_UART_Manage_Object UART10_Manage_Object = {0};

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/
Struct_UART_Manage_Object* Get_UART_Manage_Object(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) return &UART1_Manage_Object;
    else if (huart->Instance == USART2) return &UART2_Manage_Object;
    else if (huart->Instance == USART3) return &UART3_Manage_Object;
    else if (huart->Instance == UART4) return &UART4_Manage_Object;
    else if (huart->Instance == UART5) return &UART5_Manage_Object;
    else if (huart->Instance == USART6) return &UART6_Manage_Object;
    else if (huart->Instance == UART7) return &UART7_Manage_Object;
    else if (huart->Instance == UART8) return &UART8_Manage_Object;
    else if (huart->Instance == UART9) return &UART9_Manage_Object;
    else if (huart->Instance == USART10) return &UART10_Manage_Object;
    else return NULL;
}

/**
 * @brief 初始化UART
 *
 * @param huart UART编号
 * @param Callback_Function 处理回调函数
 * @param Rx_Buffer_Length 接收缓冲区长度
 */
void UART_Init(UART_HandleTypeDef *huart, UART_Call_Back Callback_Function, uint16_t Rx_Buffer_Length)
{

    Struct_UART_Manage_Object *uart_obj = Get_UART_Manage_Object(huart);
    if(uart_obj == NULL) return;

    uart_obj->UART_Handler = huart;
    uart_obj->Callback_Function = Callback_Function;
    uart_obj->Rx_Buffer_Length = Rx_Buffer_Length;
    uart_obj->Buffer_Mode = UART_MODE_SINGLE_BUF;
    uart_obj->Current_Buf_Idx = 0;

    HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_obj->Rx_Buffer, uart_obj->Rx_Buffer_Length);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); // 关闭半传输中断（单缓冲区不需要）

}

void UART_DoubleBuf_Init(UART_HandleTypeDef *huart, UART_Call_Back Callback_Function, uint16_t Rx_Buffer_Length)
{
    Struct_UART_Manage_Object *uart_obj = Get_UART_Manage_Object(huart);
    if(uart_obj == NULL) return;

    uart_obj->UART_Handler = huart;
    uart_obj->Callback_Function = Callback_Function;
    uart_obj->Rx_Buffer_Length = Rx_Buffer_Length;
    uart_obj->Buffer_Mode = UART_MODE_DOUBLE_BUF;
    uart_obj->Current_Buf_Idx = 0;

    // 配置UART为TOIDLE模式，启用IDLE中断
    huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
    huart->RxEventType = HAL_UART_RXEVENT_IDLE;
    huart->RxXferSize = Rx_Buffer_Length;

    // 启用DMA双缓冲区模式
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    HAL_DMAEx_MultiBufferStart(huart->hdmarx,(uint32_t)&huart->Instance->RDR,(uint32_t)uart_obj->Rx_Double_Buf[0],(uint32_t)uart_obj->Rx_Double_Buf[1],uart_obj->Rx_Buffer_Length);

}

/**
 * @brief 掉线重新初始化UART
 *
 * @param huart UART编号
 */
void UART_Reinit(UART_HandleTypeDef *huart)
{
    Struct_UART_Manage_Object *uart_obj = Get_UART_Manage_Object(huart);
    if(uart_obj == NULL) return;

    if(uart_obj->Buffer_Mode == UART_MODE_SINGLE_BUF)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_obj->Rx_Buffer, uart_obj->Rx_Buffer_Length);
    }
    else if(uart_obj->Buffer_Mode == UART_MODE_DOUBLE_BUF)
    {
        HAL_DMAEx_MultiBufferStart(huart->hdmarx,
                                   (uint32_t)&huart->Instance->RDR,
                                   (uint32_t)uart_obj->Rx_Double_Buf[0],
                                   (uint32_t)uart_obj->Rx_Double_Buf[1],
                                   uart_obj->Rx_Buffer_Length);
    }
}

/**
 * @brief 发送数据帧
 *
 * @param huart UART编号
 * @param Data 被发送的数据指针
 * @param Length 长度
 * @return uint8_t 执行状态
 */
uint8_t UART_Send_Data(UART_HandleTypeDef *huart, uint8_t *Data, uint16_t Length)
{
    return (HAL_UART_Transmit_DMA(huart, Data, Length));
}

/**
 * @brief UART的TIM定时器中断发送回调函数
 *
 */
void TIM_1ms_UART_PeriodElapsedCallback()
{

}


void UART_DoubleBuf_RxHandler(UART_HandleTypeDef *huart, uint16_t Size)
{
    Struct_UART_Manage_Object *uart_obj = Get_UART_Manage_Object(huart);
    if(uart_obj == NULL || uart_obj->Callback_Function == NULL) return;

    uint8_t *rx_buf = NULL;
    // 判断当前活跃缓冲区，切换索引
    if((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT)
    {
        // 当前是第二个缓冲区，切换到第一个
        rx_buf = uart_obj->Rx_Double_Buf[1];
        ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT);
    }
    else
    {
        // 当前是第一个缓冲区，切换到第二个
        rx_buf = uart_obj->Rx_Double_Buf[0];
        ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT;
    }
    // 重置DMA计数器
    __HAL_DMA_SET_COUNTER(huart->hdmarx, uart_obj->Rx_Buffer_Length);
    __HAL_DMA_ENABLE(huart->hdmarx);

    // 调用回调函数（传入当前缓冲区数据）
    if(Size > 0)
    {
        uart_obj->Callback_Function(rx_buf, Size);
    }
}

/**
 * @brief HAL库UART接收DMA空闲中断
 *
 * @param huart UART编号
 * @param Size 长度
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (bsp_init_finished_flag == 0) return;

    Struct_UART_Manage_Object *uart_obj = Get_UART_Manage_Object(huart);
    if(uart_obj == NULL) return;

    // 单缓冲区逻辑（原有）
    if(uart_obj->Buffer_Mode == UART_MODE_SINGLE_BUF)
    {
        if(uart_obj->Callback_Function != NULL)
        {
            uart_obj->Callback_Function(uart_obj->Rx_Buffer, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_obj->Rx_Buffer, uart_obj->Rx_Buffer_Length);
    }
    // 双缓冲区逻辑（新增）
    else if(uart_obj->Buffer_Mode == UART_MODE_DOUBLE_BUF)
    {
        UART_DoubleBuf_RxHandler(huart, Size);
    }
}

/**
 * @brief HAL库UART错误中断
 *
 * @param huart UART编号
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    UART_Reinit(huart); // 出错时重初始化
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
