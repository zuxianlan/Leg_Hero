#include "task_config_and_callback.h"
#include "BMI088driver.h"
#include "kalman_filter.h"
#include "mahony_filter.h"
#include "bsp_dwt.h"
#include "BMI088Middleware.h"
#include "bsp_tim.h"
#include "dvc_referee.h"
#include "spi.h"
#include "stm32h7xx_hal.h"
#include "gpio.h"
#include "CAN_comm.h"
#include "tim.h"
#include "buzzer.h"
#include "dvc_dr16.h"
#include "dvc_vt13_rc.h"
#include "dvc_fusi_rc.h"
#include "shoot_task.h"
#include "gimbal_task.h"
#include "can_comm_task.h"

//函数前置声明
void Task_half_ms_TIM12_Callback();
void Task_1ms_TIM15_Callback();

void Referee_UART1_Callback(uint8_t *Rx_Data, uint16_t Length);
void DR16_UART5_Callback(uint8_t *Rx_Data, uint16_t Length);
void Fusi_RC_UART5_Callback(uint8_t *Rx_Data, uint16_t Length);
void VT13_UART10_Callback(uint8_t *Rx_Data, uint16_t Length);

void CAN_Motor_Call_Back_FDCAN1(Struct_CAN_Rx_Buffer *Rx_Buffer);
void CAN_Motor_Call_Back_FDCAN2(Struct_CAN_Rx_Buffer *Rx_Buffer);
void CAN_Motor_Call_Back_FDCAN3(Struct_CAN_Rx_Buffer *Rx_Buffer);

//初始化标志位
bool_t bsp_init_finished_flag=0;
uint16_t ceshi_tim12_half_ms_counter=0;
uint16_t ceshi_tim15_1_ms_counter=0;
/**
 * @brief 初始化任务
 *
 */
void Task_Init()
{
    //初始化标志位重置
    bsp_init_finished_flag=0;

    // DWT初始化
    DWT_Init(480);

    // CAN总线初始化
    FDCAN_Set_Baud(&hfdcan2, CAN_CLASS, CAN_BR_1M);
    FDCAN_Set_Baud(&hfdcan3, CAN_CLASS, CAN_BR_1M);
    CAN_Init(&hfdcan1, CAN_Motor_Call_Back_FDCAN1);
    CAN_Init(&hfdcan2, CAN_Motor_Call_Back_FDCAN2);
    CAN_Init(&hfdcan3, CAN_Motor_Call_Back_FDCAN3);

    // UART初始化
    UART_Init(&huart1, Referee_UART1_Callback, REFEREE_RXBUF_SIZE);//裁判系统缓冲区 512
    //UART_DoubleBuf_Init(&huart5, DR16_UART5_Callback, DR16_RXBUF_SIZE);//dt7遥控器缓冲区 36
    UART_DoubleBuf_Init(&huart5, Fusi_RC_UART5_Callback, FUSI_RXBUF_SIZE);//富斯遥控器 50
    UART_Init(&huart10, VT13_UART10_Callback, VT13_RXBUF_SIZE);//vt13遥控器 512

    // 喂狗

    //机器人功能初始化----------------------
     //裁判系统初始化
     Class_Referee_Init(&class_referee,&huart1,0xA5);
     //DR16遥控器初始化
     DR16_Init(&class_dr16, &huart5);
     // //Fusi_RC初始化
      Fusi_RC_Init(&class_fusi_rc, &huart5);
     //VT13遥控器初始化
     VT13_Init(&class_vt13_rc, &huart10);
    //-------------------------------------

    // 定时器初始化
    TIM_Init(&htim12,Task_half_ms_TIM12_Callback);
    TIM_Init(&htim15,Task_1ms_TIM15_Callback);
    // 使能调度时钟
    HAL_TIM_Base_Start_IT(&htim12);
    HAL_TIM_Base_Start_IT(&htim15);
    // 使能PWM输出
    HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_2);

    //蜂鸣器初始化重置
    Buzzer_Init();

    // 标记初始化完成
    bsp_init_finished_flag=1;
}



//CAN回调函数--------------------------------------------------
/**
 * @brief FDCAN1报文回调函数
 *
 * @param Rx_Buffer FDCAN接收的信息结构体
 */
void CAN_Motor_Call_Back_FDCAN1(Struct_CAN_Rx_Buffer *Rx_Buffer)
{
    switch (Rx_Buffer->Header.Identifier)
    {
    case (0x201):
        {
            Motor_C620_CAN_RxCpltCallback(&shoot_control.fric_left_3508.C620_motor,Rx_Buffer->Data);
            break;

        }
    case (0x202):
        {
            Motor_C620_CAN_RxCpltCallback(&shoot_control.fric_right_3508.C620_motor,Rx_Buffer->Data);
            break;

        }

    }
}

/**
 * @brief FDCAN2报文回调函数
 *
 * @param Rx_Buffer FDCAN接收的信息结构体
 */
void CAN_Motor_Call_Back_FDCAN2(Struct_CAN_Rx_Buffer *Rx_Buffer)
{
    switch (Rx_Buffer->Header.Identifier)
    {
    case (0x66):
        {
            Motor_DM_Normal_CAN_RxCpltCallback(&gimbal_control.gimbal_pitch_j4340.dm_normal_motor,Rx_Buffer->Data);
            break;

        }

    }
}

/**
 * @brief FDCAN3报文回调函数
 *
 * @param Rx_Buffer FDCAN接收的信息结构体
 */
void CAN_Motor_Call_Back_FDCAN3(Struct_CAN_Rx_Buffer *Rx_Buffer)
{
    switch (Rx_Buffer->Header.Identifier)
    {

    case (0x55):
        {
            Motor_DM_Normal_CAN_RxCpltCallback(&gimbal_control.gimbal_yaw_j4310.dm_normal_motor,Rx_Buffer->Data);
            break;
        }

    case (0x77):
        {
            Motor_DM_Normal_CAN_RxCpltCallback(&shoot_control.shoot_trigger_j4310.dm_normal_motor,Rx_Buffer->Data);
            break;

        }

    case (0x20C):
        {
            Can_Referee_Data_Receive_Process(&Can_Comm.Can_Referee,Rx_Buffer->Data);
            break;

        }

    }
}
//------------------------------------------------------------





//串口回调函数--------------------------------------------------
void Referee_UART1_Callback(uint8_t *Rx_Data, uint16_t Length)
{
    Class_Referee_UART_RxCpltCallback(&class_referee, Rx_Data, Length);
}

void DR16_UART5_Callback(uint8_t *Rx_Data, uint16_t Length)
{
    DR16_UART_RxCpltCallback(&class_dr16, Rx_Data, Length);
}

void Fusi_RC_UART5_Callback(uint8_t *Rx_Data, uint16_t Length)
{
    Fusi_RC_UART_RxCpltCallback(&class_fusi_rc, Rx_Data, Length);
}

void VT13_UART10_Callback(uint8_t *Rx_Data, uint16_t Length)
{
    VT13_UART_RxCpltCallback(&class_vt13_rc, Rx_Data, Length);
}
//------------------------------------------------------------





//定时器回调函数--------------------------------------------------
void Task_half_ms_TIM12_Callback()
{
    static int mod = 0;
    mod++;

     if (mod % 10000 == 0)
     {
         ceshi_tim12_half_ms_counter++;
     }

     // if (mod % 1000 == 0)
     // {
     //     //dt7遥控器在线状态检测
     //     Class_DR16_TIM_100ms_Alive_PeriodElapsedCallback(&class_dr16);
     // }

     if (mod % 10000 == 0)
     {
         //vt13图传遥控器在线状态检测
         Class_VT13_RC_TIM_100ms_Alive_PeriodElapsedCallback(&class_vt13_rc);
     }

}

void Task_1ms_TIM15_Callback()
{
    static int mod = 0;
    mod++;

    if (mod % 1000 == 0)
    {
        ceshi_tim15_1_ms_counter++;
    }

    if (mod %20 ==0)
    {
        //DT7遥控器定时数据处理
        Class_DR16_TIM_1ms_Calculate_PeriodElapsedCallback(&class_dr16);
    }
    if (mod % 20 == 0)
    {
        //富斯遥控器定时数据处理
        Class_Fusi_TIM_1ms_Calculate_PeriodElapsedCallback(&class_fusi_rc);
    }

    if (mod % 7 == 0)
    {
        //蜂鸣器播放音效
        Buzzer_Process();
    }

}
//------------------------------------------------------------
