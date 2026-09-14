#include "other_task.h"
#include "cmsis_os.h"

#include "vofa.h"
#include "shoot_task.h"
#include "CAN_comm.h"
#include "Led_Flow_Task.h"
#include "can_comm_task.h"
#include "dvc_vt13_rc.h"
#include "task_config_and_callback.h"
#include "RC_Task.h"
#include "shoot_behaviour.h"
#include "INS_task.h"
#include "dvc_referee.h"
#include "vision_task.h"
#include "CAN_comm.h"

//VOFA+发送结构体
SEND_Message send_message;

void UART_Send_feedback_update(SEND_Message *sendMessage);

/**
  * @brief          led RGB任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void Other_Task(void const * argument)
{
    while (1)
    {
        LED_Flow();

        UART_Send_feedback_update(&send_message);
        vofa_start(&send_message);

        osDelay(1);
    }
}


void UART_Send_feedback_update(SEND_Message *sendMessage)
{
    //云台数据------------------------------------------------
    // sendMessage->v1 = gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Rx_Data.Now_Omega;
    //   sendMessage->v2 = gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Rx_Data.Now_Angle;

    // sendMessage->v0 = gimbal_control.gimbal_INS_point->Yaw;
    // sendMessage->v1 = gimbal_control.gimbal_INS_point->Gyro[2];//yaw
    // sendMessage->v2 = -gimbal_control.gimbal_INS_point->Pitch;
    // sendMessage->v3 = gimbal_control.gimbal_INS_point->Gyro[1];//pitch

    // sendMessage->v0 = gimbal_control.gimbal_vision_point->gimbal_yaw;
    // sendMessage->v1 = gimbal_control.gimbal_INS_point->Yaw;
    // sendMessage->v2 = gimbal_control.gimbal_yaw_j4310.dm_gimbal_motor.Target_Angle;

    // sendMessage->v3 = gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Angle_PID.Target;
    // sendMessage->v4 = gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Angle_PID.Now;
    // sendMessage->v5 = gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Target_Angle-gimbal_control.gimbal_INS_point->Yaw;
    // sendMessage->v6 = gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Omega_PID.Target;
    // sendMessage->v7 = gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Omega_PID.Now;
    // sendMessage->v8 = gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Control_Torque;

    //--------------------------------------------------------

    // //摩擦轮电机数据发送-----------------------
    // sendMessage->v0 = shoot_control.fric_left_3508.motor_status;
    // sendMessage->v1 = shoot_control.fric_right_3508.motor_status;
    //
    // sendMessage->v3 = shoot_control.fric_left_3508.C620_motor.Accel_Feedforward_Torque;
    // sendMessage->v4 = shoot_control.fric_right_3508.C620_motor.Accel_Feedforward_Torque;
    //
    // sendMessage->v5 = shoot_control.fric_left_3508.C620_motor.Target_RPM;
    // sendMessage->v6 = shoot_control.fric_left_3508.C620_motor.Rx_Origin.Omega_Reverse;
    // sendMessage->v7 = -shoot_control.fric_right_3508.C620_motor.Rx_Origin.Omega_Reverse;
    //
    // sendMessage->v9 = shoot_control.fric_left_3508.C620_motor.Slip_Feedforward_Torque;
    // sendMessage->v10 = shoot_control.fric_right_3508.C620_motor.Slip_Feedforward_Torque;
    //
    // sendMessage->v12 = shoot_control.fric_left_3508.C620_motor.Out;
    // sendMessage->v13 = shoot_control.fric_right_3508.C620_motor.Out;
    // //摩擦轮电机数据发送-----------------------
    // sendMessage->v0 = shoot_control.fric_left_3508.C620_motor.Target_RPM;
    // sendMessage->v1 = shoot_control.fric_left_3508.C620_motor.Rx_Origin.Omega_Reverse;
    // sendMessage->v2 = shoot_control.fric_left_3508.C620_motor.Out;
    // sendMessage->v3 = shoot_control.fric_left_3508.C620_motor.PID_RPM.Out;
    //
    // sendMessage->v4 = shoot_control.fric_right_3508.C620_motor.Target_RPM;
    // sendMessage->v5 = -shoot_control.fric_right_3508.C620_motor.Rx_Origin.Omega_Reverse;
    // sendMessage->v6 = shoot_control.fric_right_3508.C620_motor.Out;
    // sendMessage->v7 = shoot_control.fric_right_3508.C620_motor.PID_RPM.Out;
    //
    // sendMessage->v10 = gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Rx_Data.Now_Angle;
    //sendMessage->v11 = gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Target_Angle;

    //---------------------------------------

    // // //拨弹盘电机数据发送-----------------------
    // sendMessage->v5 = shoot_control.fric_left_3508.C620_motor.Target_Angle;
    // sendMessage->v6 = shoot_control.fric_left_3508.C620_motor.Rx_Data.Now_Angle;
    // sendMessage->v7 = shoot_control.fric_left_3508.C620_motor.Target_Angle-shoot_control.fric_left_3508.C620_motor.Rx_Data.Now_Angle;
    // sendMessage->v8 = shoot_control.fric_left_3508.C620_motor.Target_Omega;
    // sendMessage->v9 = shoot_control.fric_left_3508.C620_motor.Rx_Data.Now_Omega;
    // sendMessage->v10 = shoot_control.fric_left_3508.C620_motor.Out;
    //
    //   sendMessage->v12 = 0;
    //   sendMessage->v13 = shoot_control.shoot_trigger_j4310.Motor_FSM.Now_Status;
    //   sendMessage->v14 = shoot_control.shoot_trigger_j4310.motor_status;
    //   sendMessage->v15 = 0;
    //sendMessage->v0 = gimbal_control.gimbal_vision_point->gimbal_yaw;

    // sendMessage->v0 = gimbal_control.gimbal_vision_point->gimbal_yaw;
    // sendMessage->v1 = gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Target_Angle;
    // sendMessage->v2 = gimbal_control.gimbal_INS_point->Yaw;
    // sendMessage->v3 = gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Target_Angle - gimbal_control.gimbal_INS_point->Yaw;
    // sendMessage->v4 = gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Omega_PID.Out;
    //

    //
    // sendMessage->v5 = SP_vision.vision_send_packet.yaw ;
    // sendMessage->v6 = SP_vision.vision_send_packet.yaw_omega;
    // sendMessage->v7 = SP_vision.vision_send_packet.pitch;
    // sendMessage->v8 = SP_vision.vision_send_packet.pitch_omega;
    // sendMessage->v9 = SP_vision.vision_send_packet.bullet_speed;
    // sendMessage->v10 =SP_vision.vision_send_packet.enemy_color;
    // sendMessage->v11 =SP_vision.vision_send_packet.bullet_count;
    // sendMessage->v5 = gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Angle_PID.Target;
    // sendMessage->v6 = gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Angle_PID.Now;
    // sendMessage->v7 = gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Angle_PID.Target-gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Angle_PID.Now;
    // sendMessage->v8 = gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Omega_PID.Target;
    // sendMessage->v9 = gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Omega_PID.Now;
    // sendMessage->v10 = gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Omega_PID.Out;
    // sendMessage->v11 = gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Control_Torque;

    // sendMessage->v0 = gimbal_control.gimbal_vision_point->gimbal_yaw;
    // sendMessage->v1 = gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Target_Angle;
    // sendMessage->v2 = gimbal_control.gimbal_INS_point->Yaw;
    // sendMessage->v3 = gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Target_Angle - (gimbal_control.gimbal_INS_point->Yaw);
    // sendMessage->v4 = gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Omega_PID.Out;
    //
    // sendMessage->v5 = gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Angle_PID.Target;
    // sendMessage->v6 = gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Angle_PID.Now;
    // sendMessage->v7 = gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Angle_PID.Target-gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Angle_PID.Now;
    // sendMessage->v8 = gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Omega_PID.Target;
    // sendMessage->v9 = gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Omega_PID.Now;
    // sendMessage->v10 = gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Omega_PID.Out;
    // sendMessage->v11 = gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Control_Torque;
    //
    //sendMessage->v13 = gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Rx_Data.Now_Angle;

    // sendMessage->v0 = gimbal_control.gimbal_vision_point->gimbal_pitch;
    // sendMessage->v1 = gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Target_Angle;
    // sendMessage->v2 = -gimbal_control.gimbal_INS_point->Pitch;
    // sendMessage->v3 = gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Target_Angle - (-gimbal_control.gimbal_INS_point->Pitch);
    // sendMessage->v4 = gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Omega_PID.Out;
    // //
    // sendMessage->v0 = gimbal_control.gimbal_vision_point->gimbal_yaw;
    // sendMessage->v1 = gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Target_Angle;
    // sendMessage->v2 = gimbal_control.gimbal_INS_point->Yaw;
    // sendMessage->v3 = gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Target_Angle - (gimbal_control.gimbal_INS_point->Yaw);
    // sendMessage->v4 = gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Omega_PID.Out;

    // sendMessage->v0 = gimbal_control.gimbal_vision_point->gimbal_pitch;
    // sendMessage->v1 = -gimbal_control.gimbal_INS_point->Pitch;
    // sendMessage->v2 = gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Target_Angle - (-gimbal_control.gimbal_INS_point->Pitch);
    //
    // sendMessage->v5 = gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Angle_PID.Target;
    // sendMessage->v6 = gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Angle_PID.Now;
    // sendMessage->v7 = gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Angle_PID.Target-gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Angle_PID.Now;
    // sendMessage->v8 = gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Omega_PID.Target;
    // sendMessage->v9 = gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Omega_PID.Now;
    // sendMessage->v10 = gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Omega_PID.Out;
    // sendMessage->v11 = gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Control_Torque;

    // sendMessage->v13 = gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Feedforward_Torque;
    // sendMessage->v14 = ceshi_liju;

    // sendMessage->v12 =gimbal_control.one_key_shoot_pitch;
    //摩擦轮测试
    // sendMessage->v5 = shoot_control.fric_left_3508.C620_motor.Target_RPM;
    // sendMessage->v6 = shoot_control.fric_left_3508.C620_motor.Rx_Origin.Omega_Reverse;
    // sendMessage->v7 = shoot_control.fric_left_3508.C620_motor.Out;
    // sendMessage->v8 = -shoot_control.fric_right_3508.C620_motor.Target_RPM;
    // sendMessage->v9 = -shoot_control.fric_right_3508.C620_motor.Rx_Origin.Omega_Reverse;
    // sendMessage->v10 = -shoot_control.fric_right_3508.C620_motor.Out;

    //sendMessage->v13 = SP_vision.Rx_data.distance;

    // sendMessage->v1 = SP_vision.vision_send_packet.pitch;
    // sendMessage->v2 = SP_vision.vision_send_packet.pitch;
    // sendMessage->v3 = SP_vision.vision_send_packet.pitch;
    // sendMessage->v4 = SP_vision.vision_send_packet.pitch;
    // sendMessage->v5 = SP_vision.vision_send_packet.pitch;
    // sendMessage->v6 = SP_vision.vision_send_packet.pitch;
    // sendMessage->v7 = SP_vision.vision_send_packet.pitch;

    // sendMessage->v7 = shoot_control.fric_left_rpm_set ;
    // sendMessage->v8 = shoot_control.fric_right_rpm_set;
    // sendMessage->v9 = shoot_control.fric_speed_set_average;

    //
    //
    // sendMessage->v5=class_vt13_rc.VT13_Status;
    // sendMessage->v6 = class_fusi_rc.Fusi_RC_Status;
    //
    // sendMessage->v9 = gimbal_control.gimbal_yaw_j4310.relative_angle_set;
    // sendMessage->v10 = gimbal_control.gimbal_yaw_j4310.relative_angle;
    // sendMessage->v11 = gimbal_control.init_angle;
    //
    // sendMessage->v12=gimbal_control.init_flag;
    // sendMessage->v13 = RC_Control.gimbal_control_status;
    //
    // sendMessage->v14 = ceshi_tim12_half_ms_counter;
    // sendMessage->v15 = class_vt13_rc.Processed_Data.Mode_Switch;

    //sendMessage->v1 = SP_vision.vision_send_packet.enemy_color;
    // sendMessage->v2 = SP_vision.Rx_data.pitch;
    // sendMessage->v3 = SP_vision.Rx_data.pitch_omega;
    // sendMessage->v4 = SP_vision.Rx_data.pitch_accel;
    // sendMessage->v5 = SP_vision.Rx_data.yaw;
    // sendMessage->v6 = SP_vision.Rx_data.yaw_omega;
    // sendMessage->v7 = SP_vision.Rx_data.yaw_accel;
    // uint32_t tec, rec;
    // FDCAN_ErrorCountersTypeDef error;
    // HAL_FDCAN_GetErrorCounters(&hfdcan3, &error);
    // FDCAN_ProtocolStatusTypeDef psts;
    // HAL_FDCAN_GetProtocolStatus(&hfdcan3, &psts);
    // // robot_state.shooter_cooling_value = 100;
    // // robot_state.shooter_heat_limit = 300;
    // sendMessage->v0 = error.ErrorLogging;
    // sendMessage->v1 = error.RxErrorCnt;
    // sendMessage->v2 = error.RxErrorPassive;
    // sendMessage->v3 = error.TxErrorCnt;
    // sendMessage->v4 = psts.LastErrorCode;
    //测试yawINIT状态机
    // sendMessage->v0 = gimbal_control.gimbal_yaw_j4310.relative_angle;
    // sendMessage->v1 = gimbal_control.FSM_Yaw_Init.Now_Status;
    // sendMessage->v2 = gimbal_control.gimbal_yaw_j4310.gimbal_motor_mode;
    // sendMessage->v3 = gimbal_control.gimbal_yaw_j4310.last_gimbal_motor_mode;
    // sendMessage->v4 = gimbal_control.gimbal_pitch_j4340.gimbal_motor_mode;
    // sendMessage->v5 = gimbal_control.gimbal_pitch_j4340.last_gimbal_motor_mode;
    // sendMessage->v6 = gimbal_control.gimbal_INS_point->Pitch;
    //
    //    sendMessage->v7 = gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Rx_Data.Now_Angle;
    // //   sendMessage->v8 = gimbal_control.init_angle;
    // //
    //sendMessage->v9 = shoot_control.shoot_trigger_j4310.dm_normal_motor.Rx_Data.Now_Angle;



    //发弹延迟测试
    // sendMessage->v0 = shoot_control.shoot_trigger_j4310.dm_normal_motor.Target_Angle;
    // sendMessage->v1 = shoot_control.shoot_trigger_j4310.dm_normal_motor.Rx_Data.Now_Angle;
    // sendMessage->v2 = shoot_control.shoot_trigger_j4310.dm_normal_motor.Control_Torque;
    // sendMessage->v3 = shoot_control.already_bullet_counts;
    // sendMessage->v5 = shoot_control.fric_left_3508.C620_motor.Target_RPM;
    // sendMessage->v6 = shoot_control.fric_left_3508.C620_motor.Rx_Origin.Omega_Reverse;
    // sendMessage->v7 = shoot_control.fric_right_3508.C620_motor.Target_RPM;
    // sendMessage->v8 = shoot_control.fric_right_3508.C620_motor.Rx_Origin.Omega_Reverse;

    // sendMessage->v1 =  class_referee.Referee_Rx_Data.Robot_Status.shooter_heat_limit;
    // sendMessage->v2 = class_referee.Referee_Rx_Data.Robot_Power_Heat.shooter_42mm_heat;
    // sendMessage->v3 = shoot_control.remain_heat;
    // sendMessage->v4 = shoot_control.remain_count;
    // sendMessage->v5 = shoot_control.remain_fire_count;
    // sendMessage->v6 = shoot_control.fric_speed_average;
    // sendMessage->v7 = shoot_control.already_bullet_counts;
    // sendMessage->v8 = shoot_control.shoot_flag;
    // sendMessage->v9 = shoot_control.shoot_flag;

    // sendMessage->v1 = gimbal_control.gimbal_yaw_j4310.motor_status;
    // sendMessage->v2 = gimbal_control.gimbal_pitch_j4340.motor_status;
    // sendMessage->v3 = shoot_control.shoot_trigger_j4310.motor_status;
    // sendMessage->v4 = shoot_control.fric_left_3508.motor_status;
    // sendMessage->v5 = shoot_control.fric_right_3508.motor_status;
    //
    // sendMessage->v8 = shoot_control.fric_left_3508.C620_motor.Rx_Data.Now_Current;
    // sendMessage->v9 = shoot_control.fric_right_3508.C620_motor.Rx_Data.Now_Current;

    // sendMessage->v1 = class_fusi_rc.Processed_Data.Rotary_Switch_Right;
    // sendMessage->v2 = class_fusi_rc.Processed_Data.Rotary_Switch_Left;
    // sendMessage->v3 = gimbal_behaviour;
    // sendMessage->v3 = class_fusi_rc.Processed_Data.Left_X;
    // sendMessage->v4 = class_fusi_rc.Processed_Data.Left_Y;
    // sendMessage->v5 = class_fusi_rc.Processed_Data.Right_X;
    // sendMessage->v6 = class_fusi_rc.Processed_Data.Right_Y;
    //
    // sendMessage->v7 = gimbal_control.gimbal_yaw_j4310.motor_status;
    // sendMessage->v8 = gimbal_control.gimbal_pitch_j4340.motor_status;
    // sendMessage->v9 = shoot_control.shoot_trigger_j4310.motor_status;
    // sendMessage->v10 = shoot_control.fric_left_3508.motor_status;
    // sendMessage->v11 = shoot_control.fric_right_3508.motor_status;
    //
    // sendMessage->v7 =  class_referee.Referee_Rx_Data.Robot_Status.shooter_heat_limit;
    // sendMessage->v8 = class_referee.Referee_Rx_Data.Robot_Power_Heat.shooter_42mm_heat;
    // sendMessage->v9 = shoot_control.heat_allow_fire_flag;
    // sendMessage->v10 = shoot_control.heat_delay_allow_flag;
    // sendMessage->v11 = shoot_control.fric_speed_average;
    // sendMessage->v12 = shoot_control.fric_open_flag;
    //
    // sendMessage->v1 = Can_Comm.Can_Referee.Can_Referee_Rx_Data.robot_id;
    // sendMessage->v2 = Can_Comm.Can_Referee.Can_Referee_Rx_Data.shoot_heat;
    // sendMessage->v3 = Can_Comm.Can_Referee.Can_Referee_Rx_Data.shoot_heat_limit;
    // sendMessage->v4 = Can_Comm.Can_Referee.Can_Referee_Rx_Data.shoot_speed;

    // sendMessage->v0 = gimbal_control.gimbal_vision_point->gimbal_pitch;
    // sendMessage->v1 = gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Target_Angle;
    // sendMessage->v2 = -gimbal_control.gimbal_INS_point->Pitch;
    // sendMessage->v3 = gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Target_Angle - (-gimbal_control.gimbal_INS_point->Pitch);
    // // sendMessage->v4 = gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Omega_PID.Out;
    //
    // sendMessage->v4 = gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Control_Torque;
    // sendMessage->v5 = -guance_leso;
    // sendMessage->v6 = bili;

    // // //拨弹盘电机数据发送-----------------------
    // sendMessage->v0 = shoot_control.shoot_trigger_j4310.dm_normal_motor.Target_Angle;
    // sendMessage->v1 = shoot_control.shoot_trigger_j4310.dm_normal_motor.Rx_Data.Now_Angle;
    // sendMessage->v2 = shoot_control.shoot_trigger_j4310.dm_normal_motor.Target_Angle-shoot_control.shoot_trigger_j4310.dm_normal_motor.Rx_Data.Now_Angle;
    // sendMessage->v3 = shoot_control.shoot_trigger_j4310.dm_normal_motor.Target_Omega;
    // sendMessage->v4 = shoot_control.shoot_trigger_j4310.dm_normal_motor.Rx_Data.Now_Omega;
    // sendMessage->v5 = shoot_control.shoot_trigger_j4310.dm_normal_motor.Control_Torque;
    // sendMessage->v6 = shoot_control.shoot_trigger_j4310.dm_normal_motor.Rx_Data.Now_Torque;
    //---------------------------------------

    // sendMessage->v6 = gimbal_control.gimbal_vision_point->gimbal_pitch;
    // sendMessage->v7 = gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Target_Angle;
    // sendMessage->v8 = -gimbal_control.gimbal_INS_point->Pitch;
    // sendMessage->v9 = gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Target_Angle - (-gimbal_control.gimbal_INS_point->Pitch);
    // sendMessage->v10 = gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Omega_PID.Out;
    // //
    // sendMessage->v0 = gimbal_control.gimbal_vision_point->gimbal_yaw;
    // sendMessage->v1 = gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Target_Angle;
    // sendMessage->v2 = gimbal_control.gimbal_INS_point->Yaw;
    // sendMessage->v3 = gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Target_Angle - (gimbal_control.gimbal_INS_point->Yaw);
    // sendMessage->v4 = gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Omega_PID.Out;
    //sendMessage->v0 = gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Angle_PID.Now;
    // sendMessage->v0= gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Rx_Data.Now_Angle;
    // sendMessage->v1= gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Target_Angle;
    // sendMessage->v2= gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Rx_Data.Now_Angle;
    // sendMessage->v3= -gimbal_control.gimbal_INS_point->Pitch;

    // sendMessage->v0= shoot_control.shoot_flag;
    // sendMessage->v1= shoot_control.last_shoot_flag;
    // sendMessage->v2= class_fusi_rc.Processed_Data.Right_Mid_Switch;//1 3 2
    // sendMessage->v3= class_fusi_rc.Processed_Data.Right_Switch;//1 2
    // sendMessage->v4= shoot_control.trigger_mode;
    // sendMessage->v5= fusi_up_to_down;
    // sendMessage->v6 = class_referee.Referee_Rx_Data.Robot_Status.shooter_heat_limit;

    // sendMessage->v0= gimbal_control.gimbal_INS_point->Yaw;
    // sendMessage->v1 = gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Target_Angle;
    // sendMessage->v2 = gimbal_control.gimbal_INS_point->Gyro[2];
    // sendMessage->v3 = gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Control_Torque;
    // sendMessage->v4 = gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Rx_Data.Now_Torque;
    // sendMessage->v5 = gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Rx_Data.Now_Omega;
    // sendMessage->v6 = gimbal_control.gimbal_vision_point->gimbal_feed_forward_yaw_accel;

    sendMessage->v0 = gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Target_Angle;
    sendMessage->v1 = -gimbal_control.gimbal_INS_point->Pitch;
    sendMessage->v2 = gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Feedforward_Omega;
    sendMessage->v3 = gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Feedforward_Accel;

}
