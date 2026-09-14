//
// Created by 25031 on 25-11-1.
//

#ifndef CAN_COMM_TASK_H
#define CAN_COMM_TASK_H

#include "can_comm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "Mathh.h"

//宏定义-------------------------------------------------------------------------
//can通信任务初始化时间 单位ms
#define CAN_COMM_TASK_INIT_TIME 500
//can通信任务运行时间间隔 单位ms
#define CAN_COMM_TASK_TIME 1
//------------------------------------------------------------------------------



//结构体、枚举---------------------------------------------------------------------
typedef struct
{
  //发送智能结构体
  Can_Chassis_Control_t Can_Chassis_Control;

  Can_DT7_1_t Can_DT7_1;
  Can_DT7_2_t Can_DT7_2;

  Can_VT13_1_t Can_VT13_1;
  Can_VT13_2_t Can_VT13_2;

  Can_Fusi_1_t Can_Fusi_1;
  Can_Fusi_2_t Can_Fusi_2;

  Can_Motor_Status_Data_t Can_Motor_Status_Data;
  Can_Control_Data_t Can_Control_Data;
  Can_Remote_Data_t Can_Remote_Data;
  Can_VT13_Data_t Can_VT13_Data;


  //接受智能结构体
  Can_Referee_t Can_Referee;

}Can_Comm_t;
//------------------------------------------------------------------------------



//外部变量交互---------------------------------------------------------------------
extern Can_Comm_t Can_Comm;
//------------------------------------------------------------------------------

#endif CAN_COMM_TASK_H
