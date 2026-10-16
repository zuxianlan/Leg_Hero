/**
 * @file can_comm.c
 * @author yuanluochen,CHR
 * @brief 多设备通信模块，主要用于控制板之间的通信，使用can总线实现
 * @version 0.1
 * @date 2023-09-17
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "CAN_comm.h"
#include <string.h>
#include "gimbal_task.h"
#include "dvc_referee.h"

//关于双板发送------------------------------------------------------------------
void Can_Chassis_Control_Init(Can_Chassis_Control_t *Can_Chassis_Control, FDCAN_HandleTypeDef *hdfcan, uint16_t __CAN_Tx_ID)
{
    if(hdfcan->Instance == FDCAN1)
    {
        Can_Chassis_Control->Can_Manage_Object = &CAN1_Manage_Object;
    }
    else if(hdfcan->Instance == FDCAN2)
    {
        Can_Chassis_Control->Can_Manage_Object = &CAN2_Manage_Object;
    }
    else if(hdfcan->Instance == FDCAN3)
    {
        Can_Chassis_Control->Can_Manage_Object = &CAN3_Manage_Object;
    }

    Can_Chassis_Control->CAN_Tx_ID = __CAN_Tx_ID;
    Can_Chassis_Control->Tx_Data = CAN3_0xA1_Chassis_Control_Tx_Data;
}


void Can_DT7_1_Init(Can_DT7_1_t *Can_DT7_1, FDCAN_HandleTypeDef *hdfcan, uint16_t __CAN_Tx_ID)
{
    if(hdfcan->Instance == FDCAN1)
    {
        Can_DT7_1->Can_Manage_Object = &CAN1_Manage_Object;
    }
    else if(hdfcan->Instance == FDCAN2)
    {
        Can_DT7_1->Can_Manage_Object = &CAN2_Manage_Object;
    }
    else if(hdfcan->Instance == FDCAN3)
    {
        Can_DT7_1->Can_Manage_Object = &CAN3_Manage_Object;
    }

    Can_DT7_1->CAN_Tx_ID = __CAN_Tx_ID;
    Can_DT7_1->Tx_Data = CAN3_0xA2_DT7_1_Tx_Data;
}

void Can_DT7_2_Init(Can_DT7_2_t *Can_DT7_2, FDCAN_HandleTypeDef *hdfcan, uint16_t __CAN_Tx_ID)
{
    if(hdfcan->Instance == FDCAN1)
    {
        Can_DT7_2->Can_Manage_Object = &CAN1_Manage_Object;
    }
    else if(hdfcan->Instance == FDCAN2)
    {
        Can_DT7_2->Can_Manage_Object = &CAN2_Manage_Object;
    }
    else if(hdfcan->Instance == FDCAN3)
    {
        Can_DT7_2->Can_Manage_Object = &CAN3_Manage_Object;
    }

    Can_DT7_2->CAN_Tx_ID = __CAN_Tx_ID;
    Can_DT7_2->Tx_Data = CAN3_0xA3_DT7_2_Tx_Data;
}

void Can_VT13_1_Init(Can_VT13_1_t *Can_VT13_1, FDCAN_HandleTypeDef *hdfcan, uint16_t __CAN_Tx_ID)
{
    if(hdfcan->Instance == FDCAN1)
    {
        Can_VT13_1->Can_Manage_Object = &CAN1_Manage_Object;
    }
    else if(hdfcan->Instance == FDCAN2)
    {
        Can_VT13_1->Can_Manage_Object = &CAN2_Manage_Object;
    }
    else if(hdfcan->Instance == FDCAN3)
    {
        Can_VT13_1->Can_Manage_Object = &CAN3_Manage_Object;
    }

    Can_VT13_1->CAN_Tx_ID = __CAN_Tx_ID;
    Can_VT13_1->Tx_Data = CAN3_0xA4_VT13_1_Tx_Data;
}

void Can_VT13_2_Init(Can_VT13_2_t *Can_VT13_2, FDCAN_HandleTypeDef *hdfcan, uint16_t __CAN_Tx_ID)
{
    if(hdfcan->Instance == FDCAN1)
    {
        Can_VT13_2->Can_Manage_Object = &CAN1_Manage_Object;
    }
    else if(hdfcan->Instance == FDCAN2)
    {
        Can_VT13_2->Can_Manage_Object = &CAN2_Manage_Object;
    }
    else if(hdfcan->Instance == FDCAN3)
    {
        Can_VT13_2->Can_Manage_Object = &CAN3_Manage_Object;
    }

    Can_VT13_2->CAN_Tx_ID = __CAN_Tx_ID;
    Can_VT13_2->Tx_Data = CAN3_0xA5_VT13_2_Tx_Data;
}

void Can_Fusi_1_Init(Can_Fusi_1_t *Can_Fusi_1, FDCAN_HandleTypeDef *hdfcan, uint16_t __CAN_Tx_ID)
{
    if(hdfcan->Instance == FDCAN1)
    {
        Can_Fusi_1->Can_Manage_Object = &CAN1_Manage_Object;
    }
    else if(hdfcan->Instance == FDCAN2)
    {
        Can_Fusi_1->Can_Manage_Object = &CAN2_Manage_Object;
    }
    else if(hdfcan->Instance == FDCAN3)
    {
        Can_Fusi_1->Can_Manage_Object = &CAN3_Manage_Object;
    }

    Can_Fusi_1->CAN_Tx_ID = __CAN_Tx_ID;
    Can_Fusi_1->Tx_Data = CAN3_0xA6_Fusi_1_Tx_Data;
}

void Can_Fusi_2_Init(Can_Fusi_2_t *Can_Fusi_2, FDCAN_HandleTypeDef *hdfcan, uint16_t __CAN_Tx_ID)
{
    if(hdfcan->Instance == FDCAN1)
    {
        Can_Fusi_2->Can_Manage_Object = &CAN1_Manage_Object;
    }
    else if(hdfcan->Instance == FDCAN2)
    {
        Can_Fusi_2->Can_Manage_Object = &CAN2_Manage_Object;
    }
    else if(hdfcan->Instance == FDCAN3)
    {
        Can_Fusi_2->Can_Manage_Object = &CAN3_Manage_Object;
    }

    Can_Fusi_2->CAN_Tx_ID = __CAN_Tx_ID;
    Can_Fusi_2->Tx_Data = CAN3_0xA7_Fusi_2_Tx_Data;
}

void Can_Motor_Status_Data_Init(Can_Motor_Status_Data_t *Can_Motor_Status_Data, FDCAN_HandleTypeDef *hdfcan, uint16_t __CAN_Tx_ID)
{
    if(hdfcan->Instance == FDCAN1)
    {
        Can_Motor_Status_Data->Can_Manage_Object = &CAN1_Manage_Object;
    }
    else if(hdfcan->Instance == FDCAN2)
    {
        Can_Motor_Status_Data->Can_Manage_Object = &CAN2_Manage_Object;
    }
    else if(hdfcan->Instance == FDCAN3)
    {
        Can_Motor_Status_Data->Can_Manage_Object = &CAN3_Manage_Object;
    }

    Can_Motor_Status_Data->CAN_Tx_ID = __CAN_Tx_ID;
    Can_Motor_Status_Data->Tx_Data = CAN_Board_Tx_Data;
}

void Can_Control_Data_Init(Can_Control_Data_t *Can_Control_Data, FDCAN_HandleTypeDef *hdfcan, uint16_t __CAN_Tx_ID)
{
    if(hdfcan->Instance == FDCAN1)
    {
        Can_Control_Data->Can_Manage_Object = &CAN1_Manage_Object;
    }
    else if(hdfcan->Instance == FDCAN2)
    {
        Can_Control_Data->Can_Manage_Object = &CAN2_Manage_Object;
    }
    else if(hdfcan->Instance == FDCAN3)
    {
        Can_Control_Data->Can_Manage_Object = &CAN3_Manage_Object;
    }

    Can_Control_Data->CAN_Tx_ID = __CAN_Tx_ID;
    Can_Control_Data->Tx_Data = CAN_Board_Tx_Data;
}

void Can_Remote_Data_Init(Can_Remote_Data_t *Can_Remote_Data, FDCAN_HandleTypeDef *hdfcan, uint16_t __CAN_Tx_ID)
{
    if(hdfcan->Instance == FDCAN1)
    {
        Can_Remote_Data->Can_Manage_Object = &CAN1_Manage_Object;
    }
    else if(hdfcan->Instance == FDCAN2)
    {
        Can_Remote_Data->Can_Manage_Object = &CAN2_Manage_Object;
    }
    else if(hdfcan->Instance == FDCAN3)
    {
        Can_Remote_Data->Can_Manage_Object = &CAN3_Manage_Object;
    }

    Can_Remote_Data->CAN_Tx_ID = __CAN_Tx_ID;
    Can_Remote_Data->Tx_Data = CAN_Board_Tx_Data;
}

void Can_VT13_Data_Init(Can_VT13_Data_t *Can_VT13_Data, FDCAN_HandleTypeDef *hdfcan, uint16_t __CAN_Tx_ID)
{
    if(hdfcan->Instance == FDCAN1)
    {
        Can_VT13_Data->Can_Manage_Object = &CAN1_Manage_Object;
    }
    else if(hdfcan->Instance == FDCAN2)
    {
        Can_VT13_Data->Can_Manage_Object = &CAN2_Manage_Object;
    }
    else if(hdfcan->Instance == FDCAN3)
    {
        Can_VT13_Data->Can_Manage_Object = &CAN3_Manage_Object;
    }

    Can_VT13_Data->CAN_Tx_ID = __CAN_Tx_ID;
    Can_VT13_Data->Tx_Data = CAN_Board_Tx_Data;
}


void Can_Chassis_Control_Data_Out_Put(Can_Chassis_Control_t *Can_Chassis_Control)
{
    Can_Chassis_Control_Tx_Data_t *tmp_buff = (Can_Chassis_Control_Tx_Data_t *) Can_Chassis_Control->Tx_Data;

    tmp_buff->gimbal_pitch_angle=Can_Chassis_Control->Can_Chassis_Control_Tx_Data.gimbal_pitch_angle;
    tmp_buff->fric_speed_set_average=Can_Chassis_Control->Can_Chassis_Control_Tx_Data.fric_speed_set_average;
    tmp_buff->trigger_status=Can_Chassis_Control->Can_Chassis_Control_Tx_Data.trigger_status;
    tmp_buff->dt7_status=Can_Chassis_Control->Can_Chassis_Control_Tx_Data.dt7_status;
    tmp_buff->vt13_status=Can_Chassis_Control->Can_Chassis_Control_Tx_Data.vt13_status;
    tmp_buff->fusi_status=Can_Chassis_Control->Can_Chassis_Control_Tx_Data.fusi_status;
    tmp_buff->robot_move_flag=Can_Chassis_Control->Can_Chassis_Control_Tx_Data.robot_move_flag;
    tmp_buff->gimbal_control_status=Can_Chassis_Control->Can_Chassis_Control_Tx_Data.gimbal_control_status;
    tmp_buff->shoot_control_status=Can_Chassis_Control->Can_Chassis_Control_Tx_Data.shoot_control_status;
    tmp_buff->auto_attack_status=Can_Chassis_Control->Can_Chassis_Control_Tx_Data.auto_attack_status;
    tmp_buff->fric_status=Can_Chassis_Control->Can_Chassis_Control_Tx_Data.fric_status;
    tmp_buff->turn_round_status=Can_Chassis_Control->Can_Chassis_Control_Tx_Data.turn_round_status;

    CAN_Send_Data(Can_Chassis_Control->Can_Manage_Object->CAN_Handler, Can_Chassis_Control->CAN_Tx_ID, Can_Chassis_Control->Tx_Data, 8);
}


void Can_DT7_1_Data_Out_Put(Can_DT7_1_t *Can_DT7_1)
{
    Can_DT7_1_Tx_Data_t *tmp_buff = (Can_DT7_1_Tx_Data_t *) Can_DT7_1->Tx_Data;

    tmp_buff->ch_0 = Can_DT7_1->Can_DT7_1_Tx_Data.ch_0;
    tmp_buff->ch_1 = Can_DT7_1->Can_DT7_1_Tx_Data.ch_1;
    tmp_buff->ch_2 = Can_DT7_1->Can_DT7_1_Tx_Data.ch_2;
    tmp_buff->ch_3 = Can_DT7_1->Can_DT7_1_Tx_Data.ch_3;
    tmp_buff->ch_4 = Can_DT7_1->Can_DT7_1_Tx_Data.ch_4;

    tmp_buff->sw_1 = Can_DT7_1->Can_DT7_1_Tx_Data.sw_1;
    tmp_buff->sw_2 = Can_DT7_1->Can_DT7_1_Tx_Data.sw_2;

    tmp_buff->press_l = Can_DT7_1->Can_DT7_1_Tx_Data.press_l;
    tmp_buff->press_r = Can_DT7_1->Can_DT7_1_Tx_Data.press_r;

    CAN_Send_Data(Can_DT7_1->Can_Manage_Object->CAN_Handler, Can_DT7_1->CAN_Tx_ID, Can_DT7_1->Tx_Data, 8);
}

void Can_DT7_2_Data_Out_Put(Can_DT7_2_t *Can_DT7_2)
{
    Can_DT7_2_Tx_Data_t *tmp_buff = (Can_DT7_2_Tx_Data_t *) Can_DT7_2->Tx_Data;

    tmp_buff->x = Can_DT7_2->Can_DT7_2_Tx_Data.x;
    tmp_buff->y = Can_DT7_2->Can_DT7_2_Tx_Data.y;
    tmp_buff->z = Can_DT7_2->Can_DT7_2_Tx_Data.z;
    tmp_buff->key = Can_DT7_2->Can_DT7_2_Tx_Data.key;

    CAN_Send_Data(Can_DT7_2->Can_Manage_Object->CAN_Handler, Can_DT7_2->CAN_Tx_ID, Can_DT7_2->Tx_Data, 8);
}

void Can_VT13_1_Data_Out_Put(Can_VT13_1_t *Can_VT13_1)
{
    Can_VT13_1_Tx_Data_t *tmp_buff = (Can_VT13_1_Tx_Data_t *) Can_VT13_1->Tx_Data;

    tmp_buff->ch_0         = Can_VT13_1->Can_VT13_1_Tx_Data.ch_0;
    tmp_buff->ch_1         = Can_VT13_1->Can_VT13_1_Tx_Data.ch_1;
    tmp_buff->ch_2         = Can_VT13_1->Can_VT13_1_Tx_Data.ch_2;
    tmp_buff->ch_3         = Can_VT13_1->Can_VT13_1_Tx_Data.ch_3;
    tmp_buff->wheel        = Can_VT13_1->Can_VT13_1_Tx_Data.wheel;
    tmp_buff->fn           = Can_VT13_1->Can_VT13_1_Tx_Data.fn;
    tmp_buff->button       = Can_VT13_1->Can_VT13_1_Tx_Data.button;
    tmp_buff->go_home      = Can_VT13_1->Can_VT13_1_Tx_Data.go_home;
    tmp_buff->mode_sw      = Can_VT13_1->Can_VT13_1_Tx_Data.mode_sw;
    tmp_buff->shutter      = Can_VT13_1->Can_VT13_1_Tx_Data.shutter;
    tmp_buff->mouse_left   = Can_VT13_1->Can_VT13_1_Tx_Data.mouse_left;
    tmp_buff->mouse_right  = Can_VT13_1->Can_VT13_1_Tx_Data.mouse_right;
    tmp_buff->mouse_middle = Can_VT13_1->Can_VT13_1_Tx_Data.mouse_middle;

    CAN_Send_Data(Can_VT13_1->Can_Manage_Object->CAN_Handler, Can_VT13_1->CAN_Tx_ID, Can_VT13_1->Tx_Data, 8);
}

void Can_VT13_2_Data_Out_Put(Can_VT13_2_t *Can_VT13_2)
{
    Can_VT13_2_Tx_Data_t *tmp_buff = (Can_VT13_2_Tx_Data_t *) Can_VT13_2->Tx_Data;

    tmp_buff->mouse_x = Can_VT13_2->Can_VT13_2_Tx_Data.mouse_x;
    tmp_buff->mouse_y = Can_VT13_2->Can_VT13_2_Tx_Data.mouse_y;
    tmp_buff->mouse_z = Can_VT13_2->Can_VT13_2_Tx_Data.mouse_z;
    tmp_buff->key     = Can_VT13_2->Can_VT13_2_Tx_Data.key;

    CAN_Send_Data(Can_VT13_2->Can_Manage_Object->CAN_Handler, Can_VT13_2->CAN_Tx_ID, Can_VT13_2->Tx_Data, 8);
}

void Can_Fusi_1_Data_Out_Put(Can_Fusi_1_t *Can_Fusi_1)
{
    Can_Fusi_1_Tx_Data_t *tmp_buff = (Can_Fusi_1_Tx_Data_t *) Can_Fusi_1->Tx_Data;

    tmp_buff->ch_0 = Can_Fusi_1->Can_Fusi_1_Tx_Data.ch_0;
    tmp_buff->ch_1 = Can_Fusi_1->Can_Fusi_1_Tx_Data.ch_1;
    tmp_buff->ch_2 = Can_Fusi_1->Can_Fusi_1_Tx_Data.ch_2;
    tmp_buff->ch_3 = Can_Fusi_1->Can_Fusi_1_Tx_Data.ch_3;

    CAN_Send_Data(Can_Fusi_1->Can_Manage_Object->CAN_Handler, Can_Fusi_1->CAN_Tx_ID, Can_Fusi_1->Tx_Data, 8);
}

void Can_Fusi_2_Data_Out_Put(Can_Fusi_2_t *Can_Fusi_2)
{
    Can_Fusi_2_Tx_Data_t *tmp_buff = (Can_Fusi_2_Tx_Data_t *) Can_Fusi_2->Tx_Data;

    tmp_buff->rotary_sw_0 = Can_Fusi_2->Can_Fusi_2_Tx_Data.rotary_sw_0;
    tmp_buff->rotary_sw_1 = Can_Fusi_2->Can_Fusi_2_Tx_Data.rotary_sw_1;

    tmp_buff->sw_0 = Can_Fusi_2->Can_Fusi_2_Tx_Data.sw_0;
    tmp_buff->sw_1 = Can_Fusi_2->Can_Fusi_2_Tx_Data.sw_1;
    tmp_buff->sw_2 = Can_Fusi_2->Can_Fusi_2_Tx_Data.sw_2;
    tmp_buff->sw_3 = Can_Fusi_2->Can_Fusi_2_Tx_Data.sw_3;

    CAN_Send_Data(Can_Fusi_2->Can_Manage_Object->CAN_Handler, Can_Fusi_2->CAN_Tx_ID, Can_Fusi_2->Tx_Data, 8);
}

void Can_Motor_Status_Data_Out_Put(Can_Motor_Status_Data_t *Can_Motor_Status_Data)
{
    Can_Motor_Status_Tx_Data_t *tmp_buff = (Can_Motor_Status_Tx_Data_t *) Can_Motor_Status_Data->Tx_Data;

    tmp_buff->left_fric_motor_status = Can_Motor_Status_Data->Can_Motor_Status_Tx_Data.left_fric_motor_status;
    tmp_buff->right_fric_motor_status = Can_Motor_Status_Data->Can_Motor_Status_Tx_Data.right_fric_motor_status;
    tmp_buff->yaw_motor_status = Can_Motor_Status_Data->Can_Motor_Status_Tx_Data.yaw_motor_status;
    tmp_buff->pitch_motor_status = Can_Motor_Status_Data->Can_Motor_Status_Tx_Data.pitch_motor_status;
    tmp_buff->trigger_motor_status = Can_Motor_Status_Data->Can_Motor_Status_Tx_Data.trigger_motor_status;
    tmp_buff->pitch_angle = Can_Motor_Status_Data->Can_Motor_Status_Tx_Data.pitch_angle;

    CAN_Send_Data(Can_Motor_Status_Data->Can_Manage_Object->CAN_Handler, Can_Motor_Status_Data->CAN_Tx_ID, Can_Motor_Status_Data->Tx_Data, 8);
}

void Can_Control_Data_Out_Put(Can_Control_Data_t *Can_Control_Data)
{
    Can_Control_Tx_Data_t *tmp_buff = (Can_Control_Tx_Data_t *) Can_Control_Data->Tx_Data;

    tmp_buff->key = Can_Control_Data->Can_Control_Tx_Data.key;
    tmp_buff->flag = Can_Control_Data->Can_Control_Tx_Data.flag;
    tmp_buff->trigger_status = Can_Control_Data->Can_Control_Tx_Data.trigger_status;
    tmp_buff->robot_move_flag = Can_Control_Data->Can_Control_Tx_Data.robot_move_flag;
    tmp_buff->robot_control_status = Can_Control_Data->Can_Control_Tx_Data.robot_control_status;
    tmp_buff->fric_speed_set_average = Can_Control_Data->Can_Control_Tx_Data.fric_speed_set_average;

    CAN_Send_Data(Can_Control_Data->Can_Manage_Object->CAN_Handler, Can_Control_Data->CAN_Tx_ID, Can_Control_Data->Tx_Data, 8);
}

void Can_Remote_Data_Out_Put(Can_Remote_Data_t *Can_Remote_Data)
{
    Can_Remote_Tx_Data_t *tmp_buff = (Can_Remote_Tx_Data_t *) Can_Remote_Data->Tx_Data;

    tmp_buff->ch_1 = Can_Remote_Data->Can_Control_Tx_Data.ch_1;
    tmp_buff->ch_2 = Can_Remote_Data->Can_Control_Tx_Data.ch_2;
    tmp_buff->ch_3 = Can_Remote_Data->Can_Control_Tx_Data.ch_3;
    // tmp_buff->sw_1 = Can_Remote_Data->Can_Control_Tx_Data.sw_1;
    // tmp_buff->sw_2 = Can_Remote_Data->Can_Control_Tx_Data.sw_2;
    // tmp_buff->sw_3 = Can_Remote_Data->Can_Control_Tx_Data.sw_3;
    // tmp_buff->sw_4 = Can_Remote_Data->Can_Control_Tx_Data.sw_4;
    tmp_buff->sw = Can_Remote_Data->Can_Control_Tx_Data.sw;

    CAN_Send_Data(Can_Remote_Data->Can_Manage_Object->CAN_Handler, Can_Remote_Data->CAN_Tx_ID, Can_Remote_Data->Tx_Data, 8);
}

void Can_VT13_Data_Out_Put(Can_VT13_Data_t *Can_VT13_Data)
{
    Can_VT13_Tx_Data_t *tmp_buff = (Can_VT13_Tx_Data_t *) Can_VT13_Data->Tx_Data;

    tmp_buff->ch_1 = Can_VT13_Data->Can_VT13_Tx_Data.ch_1;
    tmp_buff->ch_2 = Can_VT13_Data->Can_VT13_Tx_Data.ch_2;
    tmp_buff->ch_3 = Can_VT13_Data->Can_VT13_Tx_Data.ch_3;
    tmp_buff->sw = Can_VT13_Data->Can_VT13_Tx_Data.sw;

    CAN_Send_Data(Can_VT13_Data->Can_Manage_Object->CAN_Handler, Can_VT13_Data->CAN_Tx_ID, Can_VT13_Data->Tx_Data, 8);
}
//------------------------------------------------------------------------------




//关于双板接受--------------------------------------------------------------------------
void Can_Referee_Init(Can_Referee_t *Can_Referee, FDCAN_HandleTypeDef *hdfcan, uint16_t __CAN_Rx_ID)
{
    if(hdfcan->Instance == FDCAN1)
    {
        Can_Referee->Can_Manage_Object = &CAN1_Manage_Object;
    }
    else if(hdfcan->Instance == FDCAN2)
    {
        Can_Referee->Can_Manage_Object = &CAN2_Manage_Object;
    }
    else if(hdfcan->Instance == FDCAN3)
    {
        Can_Referee->Can_Manage_Object = &CAN3_Manage_Object;
    }

    Can_Referee->CAN_Rx_ID = __CAN_Rx_ID;
    memset(&Can_Referee->Can_Referee_Rx_Data, 0, sizeof(Can_Referee->Can_Referee_Rx_Data));

}


void Can_Referee_Data_Receive_Process(Can_Referee_t *Can_Referee, uint8_t *Rx_Data)
{
    Can_Referee_Rx_Data_t *tmp_buff = (Can_Referee_Rx_Data_t *) Can_Referee->Can_Manage_Object->Rx_Buffer.Data;

    Can_Referee->Can_Referee_Rx_Data.robot_id           =  tmp_buff->robot_id;
    Can_Referee->Can_Referee_Rx_Data.shoot_heat        =  tmp_buff->shoot_heat;
    Can_Referee->Can_Referee_Rx_Data.shoot_speed      =  tmp_buff->shoot_speed;
    Can_Referee->Can_Referee_Rx_Data.shoot_heat_limit     =  tmp_buff->shoot_heat_limit;

    //外部数据接口，实例数据更新
    class_referee.Referee_Rx_Data.Robot_Status.robot_id = Can_Referee->Can_Referee_Rx_Data.robot_id;
    class_referee.Referee_Rx_Data.Robot_Power_Heat.shooter_42mm_heat = Can_Referee->Can_Referee_Rx_Data.shoot_heat;
    class_referee.Referee_Rx_Data.Robot_Booster.initial_speed = Can_Referee->Can_Referee_Rx_Data.shoot_speed/1000.0f;
    class_referee.Referee_Rx_Data.Robot_Status.shooter_heat_limit=Can_Referee->Can_Referee_Rx_Data.shoot_heat_limit;

}

//------------------------------------------------------------------------------
