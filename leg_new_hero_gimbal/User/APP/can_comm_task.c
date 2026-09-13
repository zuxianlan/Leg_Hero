#include "can_comm_task.h"
#include "shoot_behaviour.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "dvc_vt13_rc.h"
#include "dvc_fusi_rc.h"
#include "RC_Task.h"

//函数声明------------------------------------------------------------------------
static void CAN_Comm_Init(Can_Comm_t *Can_Comm);
static void Can_Comm_Update(Can_Comm_t *Can_Comm);
static void Can_Comm_Out_Put(Can_Comm_t *Can_Comm);
//------------------------------------------------------------------------------



//重要变量------------------------------------------------------------------------
Can_Comm_t Can_Comm;
//------------------------------------------------------------------------------



void Can_Comm_Task(void const *pvParameters)
{
   //要在云台和底盘任务开始之前完成该任务的初始化
   //双板通信初始化
   vTaskDelay(CAN_COMM_TASK_INIT_TIME);

   CAN_Comm_Init(&Can_Comm);
   while (1)
   {
      Can_Comm_Update(&Can_Comm);
      Can_Comm_Out_Put(&Can_Comm);

      osDelay(1);
  }
}

static void CAN_Comm_Init(Can_Comm_t *Can_Comm)
{
    //双板通信发送初始化
    Can_Motor_Status_Data_Init(&Can_Comm->Can_Motor_Status_Data, &hfdcan3, 0x110);
    Can_Remote_Data_Init(&Can_Comm->Can_Remote_Data, &hfdcan3, 0x88);
    Can_Control_Data_Init(&Can_Comm->Can_Control_Data, &hfdcan3, 0x99);
    Can_VT13_Data_Init(&Can_Comm->Can_VT13_Data, &hfdcan3, 0x100);

    //双板通信接收初始化
    Can_Referee_Init(&Can_Comm->Can_Referee, &hfdcan3, 0x20C);
}

static void Can_Comm_Update(Can_Comm_t *Can_Comm)
{
    Can_Comm->Can_Motor_Status_Data.Can_Motor_Status_Tx_Data.left_fric_motor_status = shoot_control.fric_left_3508.motor_status;
    Can_Comm->Can_Motor_Status_Data.Can_Motor_Status_Tx_Data.right_fric_motor_status = shoot_control.fric_right_3508.motor_status;
    Can_Comm->Can_Motor_Status_Data.Can_Motor_Status_Tx_Data.yaw_motor_status = gimbal_control.gimbal_yaw_j4310.motor_status;
    Can_Comm->Can_Motor_Status_Data.Can_Motor_Status_Tx_Data.pitch_motor_status = gimbal_control.gimbal_pitch_j4340.motor_status;
    Can_Comm->Can_Motor_Status_Data.Can_Motor_Status_Tx_Data.trigger_motor_status = shoot_control.shoot_trigger_j4310.motor_status;
    Can_Comm->Can_Motor_Status_Data.Can_Motor_Status_Tx_Data.pitch_angle = -gimbal_control.gimbal_INS_point->Pitch;


    Can_Comm->Can_Control_Data.Can_Control_Tx_Data.key = class_vt13_rc.Processed_Data.Key;
    Can_Comm->Can_Control_Data.Can_Control_Tx_Data.flag = vision_control.vision_control_mode + gimbal_control.init_flag * 10 + shoot_control.fric_open_flag * 100 + AUTO_ATTACK * 1000;
    Can_Comm->Can_Control_Data.Can_Control_Tx_Data.trigger_status = trigger_status;
    Can_Comm->Can_Control_Data.Can_Control_Tx_Data.robot_move_flag = gimbal_control.gimbal_move_flag;
    Can_Comm->Can_Control_Data.Can_Control_Tx_Data.robot_control_status = RC_Control.gimbal_control_status;
    Can_Comm->Can_Control_Data.Can_Control_Tx_Data.fric_speed_set_average = shoot_control.fric_speed_set_average;

    Can_Comm->Can_Remote_Data.Can_Control_Tx_Data.ch_1 = 10000.0f * (class_fusi_rc.Processed_Data.Left_X + 2);
    Can_Comm->Can_Remote_Data.Can_Control_Tx_Data.ch_2 = 10000.0f * (class_fusi_rc.Processed_Data.Left_Y + 2);
    Can_Comm->Can_Remote_Data.Can_Control_Tx_Data.ch_3 = 10000.0f * (class_fusi_rc.Processed_Data.Right_X + 2);
    Can_Comm->Can_Remote_Data.Can_Control_Tx_Data.sw = class_fusi_rc.Processed_Data.Left_Switch*1000 + class_fusi_rc.Processed_Data.Left_Mid_Switch*100 + class_fusi_rc.Processed_Data.Right_Mid_Switch*10 + class_fusi_rc.Processed_Data.Right_Switch;


    Can_Comm->Can_VT13_Data.Can_VT13_Tx_Data.ch_1 = 10000.0f * (class_vt13_rc.Processed_Data.Left_X + 2);
    Can_Comm->Can_VT13_Data.Can_VT13_Tx_Data.ch_2 = 10000.0f * (class_vt13_rc.Processed_Data.Left_Y + 2);
    Can_Comm->Can_VT13_Data.Can_VT13_Tx_Data.ch_3 = 10000.0f * (class_vt13_rc.Processed_Data.Right_X + 2);
    Can_Comm->Can_VT13_Data.Can_VT13_Tx_Data.sw = class_vt13_rc.Processed_Data.Mode_Switch*1000 + class_vt13_rc.Processed_Data.Shutter*100 + class_vt13_rc.Processed_Data.Go_Home*10 + class_vt13_rc.Processed_Data.Fn;

}


static void Can_Comm_Out_Put(Can_Comm_t *Can_Comm)
{
    ////对外接口给绑定的发送数据缓冲区进行数据更新,并发送数据包

    Can_Control_Data_Out_Put(&Can_Comm->Can_Control_Data);
    osDelay(3);

    Can_Remote_Data_Out_Put(&Can_Comm->Can_Remote_Data);
    osDelay(3);

    Can_VT13_Data_Out_Put(&Can_Comm->Can_VT13_Data);
    osDelay(3);

    Can_Motor_Status_Data_Out_Put(&Can_Comm->Can_Motor_Status_Data);
    osDelay(3);

}
