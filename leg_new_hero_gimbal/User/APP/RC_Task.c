//
// Created by 25031 on 2026/4/12.
//

#include "RC_Task.h"
#include "gimbal_task.h"
#include "gimbal_behaviour.h"
#include "shoot_task.h"
#include "shoot_behaviour.h"
#include "dvc_dr16.h"
#include "dvc_vt13_rc.h"
#include "dvc_fusi_rc.h"

//函数前置声明
static void RC_Control_Init(RC_Control_t *rc_control_init);
static void RC_move_channel_update(RC_Control_t *rc_control);
static void RC_key_mouse_update(RC_Control_t *rc_control);

RC_Control_t RC_Control;


void RC_Task(void)
{
    // 等待陀螺仪任务更新陀螺仪数据
    vTaskDelay(RC_TASK_INIT_TIME);

    RC_Control_Init(&RC_Control);

    while (1)
    {
        //更新移动通道数据
        RC_move_channel_update(&RC_Control);
        //更新键鼠数据
        RC_key_mouse_update(&RC_Control);

        osDelay(1);
    }

}

static void RC_Control_Init(RC_Control_t *rc_control_init)
{
    if (rc_control_init == NULL)
    {
        return;
    }
    memset(rc_control_init, 0, sizeof(RC_Control_t));

}

static void RC_move_channel_update(RC_Control_t *rc_control)
{
    if (rc_control == NULL)
    {
        return;
    }

    //重置移动通道数据

    //使用vt13图传遥控器数据-----------------------------------------------------------
    if (class_vt13_rc.VT13_Status == VT13_RC_Status_ENABLE && class_vt13_rc.Processed_Data.Mode_Switch!=Other_RC_Control)
    {
        //更新移动通道数据
        rc_control->move_channel.Left_X  =class_vt13_rc.Processed_Data.Left_X;
        rc_control->move_channel.Left_Y  =class_vt13_rc.Processed_Data.Left_Y;
        rc_control->move_channel.Right_X =class_vt13_rc.Processed_Data.Right_X;
        rc_control->move_channel.Right_Y =class_vt13_rc.Processed_Data.Right_Y;
    }
    //使用dt7遥控器数据-----------------------------------------------------------
    else if (class_dr16.DR16_Status == DR16_Status_ENABLE)
    {
        //更新移动通道数据
        rc_control->move_channel.Left_X =class_dr16.Processed_Data.Left_X;
        rc_control->move_channel.Left_Y =class_dr16.Processed_Data.Left_Y;
        rc_control->move_channel.Right_X=class_dr16.Processed_Data.Right_X;
        rc_control->move_channel.Right_Y=class_dr16.Processed_Data.Right_Y;
    }
    //使用富斯遥控器数据-----------------------------------------------------------
    else if (class_fusi_rc.Fusi_RC_Status == Fusi_RC_Status_ENABLE)
    {
        //更新移动通道数据
        rc_control->move_channel.Left_X =class_fusi_rc.Processed_Data.Left_X;
        rc_control->move_channel.Left_Y =class_fusi_rc.Processed_Data.Left_Y;
        rc_control->move_channel.Right_X=class_fusi_rc.Processed_Data.Right_X;
        rc_control->move_channel.Right_Y=class_fusi_rc.Processed_Data.Right_Y;
    }
    else
    {
        rc_control->move_channel.Left_X =0;
        rc_control->move_channel.Left_Y =0;
        rc_control->move_channel.Right_X=0;
        rc_control->move_channel.Right_Y=0;
    }
    //-------------------------------------------------------------------------------


    if (RC_Control.gimbal_control_status==Key_Mouse_Control)
    {
        rc_control->move_channel.Left_X =0;
        rc_control->move_channel.Left_Y =0;
        rc_control->move_channel.Right_X=0;
        rc_control->move_channel.Right_Y=0;
    }

    // if (RC_Control.gimbal_control_status==No_Control)
    // {
    //     rc_control->move_channel.Left_X =0;
    //     rc_control->move_channel.Left_Y =0;
    //     rc_control->move_channel.Right_X=0;
    //     rc_control->move_channel.Right_Y=0;
    // }
}

static void RC_key_mouse_update(RC_Control_t *rc_control)
{
    if (rc_control == NULL)
    {
        return;
    }

    //重置键鼠数据

    if (class_vt13_rc.VT13_Status == VT13_RC_Status_ENABLE && class_vt13_rc.Processed_Data.Mode_Switch!=Other_RC_Control)
    {
        rc_control->key_mouse.Mouse_X=class_vt13_rc.Processed_Data.Mouse_X;
        rc_control->key_mouse.Mouse_Y=class_vt13_rc.Processed_Data.Mouse_Y;
        rc_control->key_mouse.Mouse_Z=class_vt13_rc.Processed_Data.Mouse_Z;
        rc_control->key_mouse.Mouse_Left_Key=class_vt13_rc.Processed_Data.Mouse_Left;
        rc_control->key_mouse.Mouse_Right_Key=class_vt13_rc.Processed_Data.Mouse_Right;
        rc_control->key_mouse.Key=class_vt13_rc.Processed_Data.Key;
    }
    else if (class_dr16.DR16_Status == DR16_Status_ENABLE)
    {
        rc_control->key_mouse.Mouse_X=class_dr16.Processed_Data.Mouse_X;
        rc_control->key_mouse.Mouse_Y=class_dr16.Processed_Data.Mouse_Y;
        rc_control->key_mouse.Mouse_Z=class_dr16.Processed_Data.Mouse_Z;
        rc_control->key_mouse.Mouse_Left_Key=class_dr16.Processed_Data.Mouse_Left_Key;
        rc_control->key_mouse.Mouse_Right_Key=class_dr16.Processed_Data.Mouse_Right_Key;
        rc_control->key_mouse.Key=class_dr16.Processed_Data.Key;
    }

    if (RC_Control.gimbal_control_status==Remote_Control)
    {
        rc_control->key_mouse.Mouse_X=0;
        rc_control->key_mouse.Mouse_Y=0;
        rc_control->key_mouse.Mouse_Z=0;
        rc_control->key_mouse.Mouse_Left_Key=0;
        rc_control->key_mouse.Mouse_Right_Key=0;
        rc_control->key_mouse.Key=0;
    }

    // if (RC_Control.gimbal_control_status==No_Control)
    // {
    //     rc_control->key_mouse.Mouse_X=0;
    //     rc_control->key_mouse.Mouse_Y=0;
    //     rc_control->key_mouse.Mouse_Z=0;
    //     rc_control->key_mouse.Mouse_Left_Key=0;
    //     rc_control->key_mouse.Mouse_Right_Key=0;
    //     rc_control->key_mouse.Key=0;
    // }
}



uint16_t Get_gimbal_behaviour_mode(void)
{
    static int16_t last_key_Z = 0;
    static int16_t last_key_G = 0;
    static int16_t last_key_Q = 0;

    static bool_t turn_auto_flag = 0;
    static bool_t last_turn_auto_flag = 0;
    static bool_t gimbal_rc_mode = 1;     //遥控器控制云台flag
    static int gimbal_key_mouse_mode = 0; // 键鼠控制云台flag

    static bool_t gimbal_have_force_flag = 0;
    static bool_t last_gimbal_have_force_key = 0;

    //重置状态
    uint16_t tmp_gimbal_behaviour_mode=GIMBAL_ZERO_FORCE;
    RC_Control.gimbal_control_status=No_Control;
    AUTO_ATTACK=0;
    TURN_ROUND=0;

    // 键鼠移动开关
    if (!last_key_Z && RC_Control.key_mouse.Key & RC_KEY_PRESSED_OFFSET_Z)
    {
        gimbal_control.gimbal_move_flag = !gimbal_control.gimbal_move_flag;
    }

    //键鼠切换云台模式-------------------------------------------------------------
    if (RC_Control.key_mouse.Mouse_Right_Key)
    {
        if (vision_control.gimbal_vision_control.vision_control_gimbal_flag == 0)// 只有成功识别才进入自瞄模式
        {
            gimbal_key_mouse_mode = 0;
        }
        else
        {
            gimbal_key_mouse_mode = 1;
        }
    }
    else
    {
        gimbal_key_mouse_mode = 0;
    }
    //--------------------------------------------------------------------------


//VT13图传遥控器控制Begin-----------------------------------------------------------------------------------
    if (class_vt13_rc.VT13_Status == VT13_RC_Status_ENABLE && class_vt13_rc.Processed_Data.Mode_Switch!=Other_RC_Control)
    {
        if (last_gimbal_have_force_key==0 && class_vt13_rc.Processed_Data.Shutter)
        {
            gimbal_have_force_flag = !gimbal_have_force_flag;
        }
        //遥控器切换云台模式------------------------------------------------------------
        if (class_vt13_rc.Processed_Data.Wheel > -0.3)
        {
            turn_auto_flag = 0;
        }
        else if (class_vt13_rc.Processed_Data.Wheel < -0.7)
        {
            turn_auto_flag = 1;
        }

        if (last_turn_auto_flag == 0 && turn_auto_flag == 1)
        {
            // 遥控器控制时切换自瞄模式
            gimbal_rc_mode = !gimbal_rc_mode;
        }
        //--------------------------------------------------------------------------

        if (gimbal_behaviour == GIMBAL_TURN_ROUND) // 一键掉头时不进入其他模式
        {
            TURN_ROUND=1;

            RC_Control.gimbal_control_status = Key_Mouse_Control;
            tmp_gimbal_behaviour_mode=GIMBAL_TURN_ROUND;

            if (gimbal_control.GIMBAL_xTickCount - gimbal_control.gimbal_turn_round_start_time > 1200)
            {
                tmp_gimbal_behaviour_mode = GIMBAL_ABSOLUTE_ANGLE;
            }
        }
        else // 非一键掉头
        {
            // //开关控制 云台状态
            if (class_vt13_rc.Processed_Data.Mode_Switch == VT13_RC_Control)//进入遥控器控制模式
            {
                if (gimbal_have_force_flag)
                {
                    if (gimbal_rc_mode == 0)
                    {
                        tmp_gimbal_behaviour_mode = GIMBAL_ABSOLUTE_ANGLE;
                    }
                    else if (gimbal_rc_mode == 1)
                    {
                        tmp_gimbal_behaviour_mode = GIMBAL_AUTO_ATTACK;
                    }
                }
                else
                {
                    gimbal_rc_mode=0;
                    tmp_gimbal_behaviour_mode = GIMBAL_ZERO_FORCE;
                }

                RC_Control.gimbal_control_status = Remote_Control;
            }
            else if (class_vt13_rc.Processed_Data.Mode_Switch == VT13_Key_Mouse_Control && gimbal_control.gimbal_move_flag)
            {//进入键鼠控制模式

                if ( gimbal_control.FSM_Yaw_Init.Now_Status == INIT_ING)
                {
                    tmp_gimbal_behaviour_mode=GIMBAL_INIT;
                    gimbal_control.one_key_shoot_flag=0;
                }
                else if (gimbal_control.FSM_Yaw_Init.Now_Status == INIT_FINISH)
                {

                    if (gimbal_key_mouse_mode == 0)
                    {
                        if ( (!last_key_Q) && ( (RC_Control.key_mouse.Key & RC_KEY_PRESSED_OFFSET_Q) && !(RC_Control.key_mouse.Key & RC_KEY_PRESSED_OFFSET_CTRL) ) )
                        {
                            gimbal_control.one_key_shoot_pitch = -gimbal_control.gimbal_INS_point->Pitch;
                            gimbal_control.one_key_shoot_flag = !gimbal_control.one_key_shoot_flag;
                        }

                        tmp_gimbal_behaviour_mode = GIMBAL_ABSOLUTE_ANGLE;
                        AUTO_ATTACK = 0;
                    }
                    else if (gimbal_key_mouse_mode == 1)
                    {
                        tmp_gimbal_behaviour_mode = GIMBAL_AUTO_ATTACK;
                        AUTO_ATTACK = 1;
                        gimbal_control.one_key_shoot_flag=0;
                    }
                    // 触发一键掉头
                    if (!last_key_G && RC_Control.key_mouse.Key & RC_KEY_PRESSED_OFFSET_G)
                    {
                        TURN_ROUND=1;
                        tmp_gimbal_behaviour_mode = GIMBAL_TURN_ROUND;
                        gimbal_control.gimbal_turn_round_start_time = gimbal_control.GIMBAL_xTickCount;
                    }

                }
                else
                {
                    tmp_gimbal_behaviour_mode = GIMBAL_ZERO_FORCE;
                    gimbal_control.one_key_shoot_flag=0;
                }


                RC_Control.gimbal_control_status = Key_Mouse_Control;

            }
            else
            {
                tmp_gimbal_behaviour_mode = GIMBAL_ZERO_FORCE;

                RC_Control.gimbal_control_status = No_Control;
            }
        }
        //
        last_turn_auto_flag = turn_auto_flag;
    }

//VT13图传遥控器控制End-----------------------------------------------------------------------------------



// //DT7遥控器控制Begin-----------------------------------------------------------------------------------
//     else if (class_dr16.DR16_Status == DR16_Status_ENABLE)
//     {
//         //遥控器切换云台模式------------------------------------------------------------
//         if (gimbal_control.gimbal_dr16_processed_data->Yaw > -0.3)
//         {
//             turn_auto_flag = 0;
//         }
//         else if (gimbal_control.gimbal_dr16_processed_data->Yaw < -0.7)
//         {
//             turn_auto_flag = 1;
//         }
//
//         if (last_turn_auto_flag == 0 && turn_auto_flag == 1)
//         {
//             // 遥控器控制时切换自瞄模式
//             gimbal_rc_mode = !gimbal_rc_mode;
//         }
//         //--------------------------------------------------------------------------
//
//         if (gimbal_behaviour == GIMBAL_TURN_ROUND) // 一键掉头时不进入其他模式
//         {
//             TURN_ROUND=1;
//
//             RC_Control.gimbal_control_status = Key_Mouse_Control;
//             tmp_gimbal_behaviour_mode=GIMBAL_TURN_ROUND;
//
//             if (gimbal_control.GIMBAL_xTickCount - gimbal_control.gimbal_turn_round_start_time > 1200)
//             {
//                 tmp_gimbal_behaviour_mode = GIMBAL_ABSOLUTE_ANGLE;
//             }
//         }
//         else // 非一键掉头
//         {
//             // //开关控制 云台状态
//             if (!dr16_switch_is_down(gimbal_control.gimbal_dr16_processed_data->Left_Switch) && gimbal_control.gimbal_dr16_processed_data->Left_Switch!=0) // 左开启
//             {//进入遥控模式
//                 if (gimbal_rc_mode == 0)
//                 {
//                     tmp_gimbal_behaviour_mode = GIMBAL_ABSOLUTE_ANGLE;
//                 }
//                 else if (gimbal_rc_mode == 1)
//                 {
//                     tmp_gimbal_behaviour_mode = GIMBAL_AUTO_ATTACK;
//                 }
//
//                 // if (dr16_switch_is_mid(gimbal_control.gimbal_dr16_processed_data->Left_Switch) && dr16_switch_is_up(gimbal_control.gimbal_dr16_processed_data->Right_Switch))
//                 // {
//                 //     tmp_gimbal_behaviour_mode=GIMBAL_INIT;
//                 // }
//
//                 RC_Control.gimbal_control_status = Remote_Control;
//             }
//             else if (dr16_switch_is_down(gimbal_control.gimbal_dr16_processed_data->Left_Switch) && gimbal_control.gimbal_move_flag) // 左关闭 键鼠操作
//             {//进入键鼠控制模式
//                 if (gimbal_key_mouse_mode == 0)
//                 {
//                     tmp_gimbal_behaviour_mode = GIMBAL_ABSOLUTE_ANGLE;
//                     AUTO_ATTACK = 0;
//                 }
//                 else if (gimbal_key_mouse_mode == 1)
//                 {
//                     tmp_gimbal_behaviour_mode = GIMBAL_AUTO_ATTACK;
//                     AUTO_ATTACK = 1;
//                 }
//                 // 触发一键掉头
//                 if (!last_key_G && RC_Control.key_mouse.Key & RC_KEY_PRESSED_OFFSET_G)
//                 {
//                     TURN_ROUND=1;
//
//                     tmp_gimbal_behaviour_mode = GIMBAL_TURN_ROUND;
//                     gimbal_control.gimbal_turn_round_start_time = gimbal_control.GIMBAL_xTickCount;
//                 }
//
//                 RC_Control.gimbal_control_status = Key_Mouse_Control;
//             }
//             else
//             {
//                 tmp_gimbal_behaviour_mode = GIMBAL_ZERO_FORCE;
//                 RC_Control.gimbal_control_status = No_Control;
//             }
//         }
//         //
//         last_turn_auto_flag = turn_auto_flag;
//     }
// //DT7遥控器控制End-----------------------------------------------------------------------------------


//Fusi遥控器控制Begin-----------------------------------------------------------------------------------
    else if (class_fusi_rc.Fusi_RC_Status == Fusi_RC_Status_ENABLE)
    {
        if (class_fusi_rc.Processed_Data.Right_Mid_Switch==1 || class_fusi_rc.Processed_Data.Right_Mid_Switch==0)
        {
            tmp_gimbal_behaviour_mode = GIMBAL_ZERO_FORCE;
            RC_Control.gimbal_control_status = No_Control;
        }
        else
        {
            RC_Control.gimbal_control_status = Remote_Control;

            if ( gimbal_control.FSM_Yaw_Init.Now_Status == INIT_ING)
            {
                tmp_gimbal_behaviour_mode=GIMBAL_INIT;
            }
            else if (gimbal_control.FSM_Yaw_Init.Now_Status == INIT_FINISH)
            {
                if (class_fusi_rc.Processed_Data.Rotary_Switch_Right <= 0.5)
                {
                    tmp_gimbal_behaviour_mode = GIMBAL_ABSOLUTE_ANGLE;
                }
                else if (class_fusi_rc.Processed_Data.Rotary_Switch_Right > 0.5)
                {
                    tmp_gimbal_behaviour_mode = GIMBAL_AUTO_ATTACK;
                }
                else
                {
                    tmp_gimbal_behaviour_mode = GIMBAL_ZERO_FORCE;
                }
            }
            else
            {
                tmp_gimbal_behaviour_mode = GIMBAL_ZERO_FORCE;
            }
        }
    }

//Fusi遥控器控制End-----------------------------------------------------------------------------------



    //状态保护
    if (RC_Control.gimbal_control_status != RC_Control.last_gimbal_control_status)
    {
        //切换的时候，标志位重置，防止再进的时候直接疯掉
        turn_auto_flag = 0;
        last_turn_auto_flag = 0;
        gimbal_rc_mode = 0;
        gimbal_key_mouse_mode = 0;
        gimbal_have_force_flag=0;
        gimbal_control.one_key_shoot_flag=0;
    }

    if (RC_Control.gimbal_control_status !=Key_Mouse_Control)
    {
        //切换的时候，标志位重置，防止再进的时候直接疯掉
        gimbal_control.gimbal_move_flag=0;
        gimbal_control.one_key_shoot_flag=0;
    }

    if (RC_Control.gimbal_control_status ==No_Control)
    {
        tmp_gimbal_behaviour_mode=GIMBAL_ZERO_FORCE;
        gimbal_control.one_key_shoot_flag=0;
    }

    last_gimbal_have_force_key=class_vt13_rc.Processed_Data.Shutter;
    last_key_G = RC_Control.key_mouse.Key & RC_KEY_PRESSED_OFFSET_G;
    last_key_Z = RC_Control.key_mouse.Key & RC_KEY_PRESSED_OFFSET_Z;
    last_key_Q = RC_Control.key_mouse.Key & RC_KEY_PRESSED_OFFSET_Q;


    RC_Control.last_gimbal_control_status=RC_Control.gimbal_control_status;

    return tmp_gimbal_behaviour_mode;
}






uint16_t Get_shoot_behaviour_mode(void)
{
    static int16_t last_key_E = 0;
    static int16_t last_key_Q = 0;
    static int16_t last_key_R = 0;
    static bool_t shoot_have_force_flag = 0;
    static bool_t last_shoot_have_force_key = 0;
    static bool_t shoot_gongdan_flag = 0;
    static bool_t last_shoot_gongdan_key = 0;

    //重置状态
    uint16_t tmp_shoot_behaviour_mode=SHOOT_DISABLE;
    RC_Control.shoot_control_status=No_Control;
    RC_Control.rc_mode=no_rc;
    //重置拨弹盘状态
    trigger_status = TRIGGER_NORMAL;

    //VT13图传遥控器控制Begin-----------------------------------------------------------------------------------
    if (class_vt13_rc.VT13_Status == VT13_RC_Status_ENABLE && class_vt13_rc.Processed_Data.Mode_Switch!=Other_RC_Control)
    {
        RC_Control.rc_mode=vt13_mode;

        if (last_shoot_have_force_key==0 && class_vt13_rc.Processed_Data.Go_Home)
        {
            shoot_have_force_flag = !shoot_have_force_flag;
        }

        // //控制发射状态机------------------------------------------------------------------
        if (class_vt13_rc.Processed_Data.Mode_Switch == VT13_RC_Control)
        {
            //进入遥控器控制
            if (shoot_have_force_flag)
            {
                if (last_shoot_gongdan_key==0 && class_vt13_rc.Processed_Data.Fn)
                {
                    shoot_gongdan_flag = !shoot_gongdan_flag;
                }

                tmp_shoot_behaviour_mode = SHOOT_ENABLE;

                if (shoot_gongdan_flag==1)
                {
                    tmp_shoot_behaviour_mode = SHOOT_MOUSE_GONGDAN;
                }
            }
            else
            {
                //退出发射模式后，供弹标志位重置，为保证改供弹只在发射有力模式下，而且退出再进后，发射模式为正常模式
                shoot_gongdan_flag=0;

                tmp_shoot_behaviour_mode = SHOOT_DISABLE;
            }

            RC_Control.shoot_control_status = Remote_Control;
        }
        else if (class_vt13_rc.Processed_Data.Mode_Switch == VT13_Key_Mouse_Control )//进入键鼠控制模式
        {
            //进入键鼠控制
            if (gimbal_control.gimbal_move_flag)
            {
                //R键点控摩擦轮
                if (!last_key_R && RC_Control.key_mouse.Key & RC_KEY_PRESSED_OFFSET_R)
                {
                    R_Fric= !R_Fric;
                }

                if (R_Fric)
                {
                    tmp_shoot_behaviour_mode = SHOOT_ENABLE;
                }
                else
                {
                    tmp_shoot_behaviour_mode = SHOOT_DISABLE;
                }

                RC_Control.shoot_control_status = Key_Mouse_Control;
            }
            else
            {
                R_Fric=0;
                tmp_shoot_behaviour_mode = SHOOT_DISABLE;

                RC_Control.shoot_control_status = No_Control;
            }
        }
        else
        {
            // 无力模式
            tmp_shoot_behaviour_mode = SHOOT_DISABLE;

            RC_Control.shoot_control_status = No_Control;
        }
    }

    //VT13图传遥控器控制End-----------------------------------------------------------------------------------



    //DT7遥控器控制Begin-----------------------------------------------------------------------------------
    // else if (class_dr16.DR16_Status == DR16_Status_ENABLE)
    // {
    //     RC_Control.rc_mode=dt7_mode;
    //
    //     // //控制发射状态机------------------------------------------------------------------
    //     if ( !dr16_switch_is_down(shoot_control.shoot_dr16_processed_data->Left_Switch) && shoot_control.shoot_dr16_processed_data->Left_Switch!=0 ) //遥控器控制
    //     {
    //         //进入遥控器控制
    //         if (dr16_switch_is_up(shoot_control.shoot_dr16_processed_data->Left_Switch))
    //         {
    //             tmp_shoot_behaviour_mode = SHOOT_ENABLE;
    //         }
    //         else if (dr16_switch_is_mid(shoot_control.shoot_dr16_processed_data->Left_Switch) && (dr16_switch_is_up(shoot_control.shoot_dr16_processed_data->Right_Switch )))
    //         {
    //             tmp_shoot_behaviour_mode = SHOOT_MOUSE_GONGDAN;
    //         }
    //         else if (dr16_switch_is_mid(shoot_control.shoot_dr16_processed_data->Left_Switch) && (dr16_switch_is_mid(shoot_control.shoot_dr16_processed_data->Right_Switch )))
    //         {
    //             tmp_shoot_behaviour_mode = SHOOT_RC_HUIBO;
    //         }
    //         else if (dr16_switch_is_mid(shoot_control.shoot_dr16_processed_data->Left_Switch) && (dr16_switch_is_down(shoot_control.shoot_dr16_processed_data->Right_Switch )))
    //         {
    //             tmp_shoot_behaviour_mode = SHOOT_DISABLE;
    //         }
    //         else
    //         {
    //             tmp_shoot_behaviour_mode = SHOOT_DISABLE;
    //         }
    //
    //         RC_Control.shoot_control_status = Remote_Control;
    //     }
    //     else if ( dr16_switch_is_down(shoot_control.shoot_dr16_processed_data->Left_Switch) ) //键鼠控制
    //     {
    //         //进入键鼠控制
    //         if (gimbal_control.gimbal_move_flag)
    //         {
    //             //R键点控摩擦轮
    //             if (!last_key_R && RC_Control.key_mouse.Key & RC_KEY_PRESSED_OFFSET_R)
    //             {
    //                 R_Fric= !R_Fric;
    //             }
    //
    //             if (R_Fric)
    //             {
    //                 tmp_shoot_behaviour_mode = SHOOT_ENABLE;
    //             }
    //             else
    //             {
    //                 tmp_shoot_behaviour_mode = SHOOT_DISABLE;
    //             }
    //
    //             RC_Control.shoot_control_status = Key_Mouse_Control;
    //         }
    //         else
    //         {
    //             R_Fric=0;
    //             tmp_shoot_behaviour_mode = SHOOT_DISABLE;
    //
    //             RC_Control.shoot_control_status = No_Control;
    //         }
    //     }
    //     else
    //     {
    //         // 无力模式
    //         tmp_shoot_behaviour_mode = SHOOT_DISABLE;
    //
    //         RC_Control.shoot_control_status = No_Control;
    //     }
    //
    //     //-----------------------------------------------------------------------------
    // }
    //DT7遥控器控制End-----------------------------------------------------------------------------------


    //Fusi遥控器控制Begin-----------------------------------------------------------------------------------
    else if (class_fusi_rc.Fusi_RC_Status == Fusi_RC_Status_ENABLE)
    {
        RC_Control.rc_mode=fusi_mode;

        if (class_fusi_rc.Processed_Data.Right_Mid_Switch==1 || class_fusi_rc.Processed_Data.Right_Mid_Switch==0)
        {
            tmp_shoot_behaviour_mode = SHOOT_DISABLE;
            RC_Control.shoot_control_status = No_Control;
        }
        else
        {
            RC_Control.shoot_control_status = Remote_Control;

            if (class_fusi_rc.Processed_Data.Right_Mid_Switch==2)
            {
                if (class_fusi_rc.Processed_Data.Rotary_Switch_Left <= 0.5)
                {
                    tmp_shoot_behaviour_mode = SHOOT_ENABLE;
                }
                else if (class_fusi_rc.Processed_Data.Rotary_Switch_Left > 0.5)
                {
                    tmp_shoot_behaviour_mode = SHOOT_MOUSE_GONGDAN;;
                }
                else
                {
                    tmp_shoot_behaviour_mode = SHOOT_ENABLE;
                }
            }
            else
            {
                tmp_shoot_behaviour_mode = SHOOT_DISABLE;
            }
        }
    }

    //Fusi遥控器控制End-----------------------------------------------------------------------------------



    // // 点按开关速度环供弹
    // if ( (!last_key_E) && ( (RC_Control.key_mouse.Key & RC_KEY_PRESSED_OFFSET_E) && !(RC_Control.key_mouse.Key & RC_KEY_PRESSED_OFFSET_CTRL) ) )
    // {
    //     mouse_gongdan_flag = !mouse_gongdan_flag;
    // }


    // ///发射机构不开启时，清除标志位
    // if (gimbal_control.gimbal_move_flag==0)
    // {
    //     mouse_gongdan_flag=0;
    // }
    //
    // if (mouse_gongdan_flag)
    // {
    //     tmp_shoot_behaviour_mode=SHOOT_MOUSE_GONGDAN;
    //     trigger_status = TRIGGER_GONGDAN;
    // }

    //状态保护
    if (RC_Control.shoot_control_status != RC_Control.last_shoot_control_status)
    {
        //切换的时候，状态重置，防止再进的时候直接疯掉
        R_Fric=0;
        shoot_have_force_flag=0;
    }

    if (RC_Control.shoot_control_status ==No_Control)
    {
        tmp_shoot_behaviour_mode=SHOOT_DISABLE;
        RC_Control.rc_mode=no_rc;
        mouse_gongdan_flag=0;
    }

    last_shoot_have_force_key=class_vt13_rc.Processed_Data.Go_Home;
    last_shoot_gongdan_key=class_vt13_rc.Processed_Data.Fn;

    last_key_E =RC_Control.key_mouse.Key & RC_KEY_PRESSED_OFFSET_E;
    last_key_R =RC_Control.key_mouse.Key & RC_KEY_PRESSED_OFFSET_R;

    RC_Control.last_shoot_control_status=RC_Control.shoot_control_status;

    return tmp_shoot_behaviour_mode;
}



void Get_dt7_shoot_flag(void)
{

    // 发射模式时将shoot_flag置零, 允许发射下颗弹丸(为了单发)
    if (RC_Control.shoot_control_status==Remote_Control)
    {
        if (shoot_control.shoot_dr16_processed_data->Yaw < 0.4)
        {
            shoot_control.shoot_flag = 0;
        }
    }
    else if (R_Fric)
    {
        if (shoot_control.shoot_dr16_processed_data->Mouse_Left_Key == 0)
        {
            shoot_control.shoot_flag = 0;
        }
    }


    if (gimbal_behaviour == GIMBAL_AUTO_ATTACK)
    {
        if (shoot_control.shoot_flag == 0                                                       // 条件1:单发限制允许发射下一发
            && (shoot_control.shoot_dr16_processed_data->Yaw > 0.8 || shoot_control.shoot_dr16_processed_data->Mouse_Left_Key) // 条件2:手动触发发弹标志
            && shoot_control.shoot_vision_control->shoot_command == SHOOT_ATTACK
            && shoot_control.heat_allow_fire_flag==1
            )
        {
            shoot_control.shoot_flag = 1;
        }
    }
    else
    {
        if (shoot_control.shoot_flag == 0                                                       // 条件1:单发限制允许发射下一发
       && (shoot_control.shoot_dr16_processed_data->Yaw > 0.8 || shoot_control.shoot_dr16_processed_data->Mouse_Left_Key)
       && shoot_control.heat_allow_fire_flag==1
      ) // 条件2:手动触发发弹标志
        {
            shoot_control.shoot_flag = 1;
        }
    }

}


void Get_vt13_shoot_flag(void)
{

    //shoot_control.heat_allow_fire_flag = 1;
    // 发射模式时将shoot_flag置零, 允许发射下颗弹丸(为了单发)
    if (RC_Control.shoot_control_status==Remote_Control)
    {
        if (shoot_control.shoot_vt13_processed_data->Wheel < 0.4)
        {
            shoot_control.shoot_flag = 0;
        }
    }
    else if (R_Fric)
    {
        if (shoot_control.shoot_vt13_processed_data->Mouse_Left == 0)
        {
            shoot_control.shoot_flag = 0;
        }
    }


    if (gimbal_behaviour == GIMBAL_AUTO_ATTACK)
    {
        if (shoot_control.shoot_flag == 0                                                       // 条件1:单发限制允许发射下一发
            && (shoot_control.shoot_vt13_processed_data->Wheel > 0.8 || shoot_control.shoot_vt13_processed_data->Mouse_Left) // 条件2:手动触发发弹标志
            && shoot_control.shoot_vision_control->shoot_command == SHOOT_ATTACK
            && shoot_control.heat_allow_fire_flag==1
            )
        {
            shoot_control.shoot_flag = 1;
        }
    }
    else
    {
        if (shoot_control.shoot_flag == 0                                                       // 条件1:单发限制允许发射下一发
       && (shoot_control.shoot_vt13_processed_data->Wheel > 0.8 || shoot_control.shoot_vt13_processed_data->Mouse_Left)
       && shoot_control.heat_allow_fire_flag==1
    ) // 条件2:手动触发发弹标志
        {
            shoot_control.shoot_flag = 1;
        }
    }

}


void Get_fusi_shoot_flag(void)
{
    shoot_control.heat_allow_fire_flag=1;
    // 发射模式时将shoot_flag置零, 允许发射下颗弹丸(为了单发)
    if (RC_Control.shoot_control_status==Remote_Control)
    {
        if (class_fusi_rc.Processed_Data.Right_Switch==FUSI_SWITCH_UP || class_fusi_rc.Processed_Data.Right_Switch==FUSI_SWITCH_DOWN)
        {
            shoot_control.shoot_flag = 0;
        }
    }

    if (gimbal_behaviour == GIMBAL_AUTO_ATTACK)
    {
        if (shoot_control.shoot_flag == 0                                                       // 条件1:单发限制允许发射下一发
            && (class_fusi_rc.Processed_Data.Right_Switch_Status==Fusi_Two_Stage_Switch_Status_UP_TRIG_DOWN || class_fusi_rc.Processed_Data.Right_Switch_Status==Fusi_Two_Stage_Switch_Status_DOWN_TRIG_UP) // 条件2:手动触发发弹标志
            && shoot_control.shoot_vision_control->shoot_command == SHOOT_ATTACK
            && shoot_control.heat_allow_fire_flag==1
          )
        {
            shoot_control.shoot_flag = 1;
        }
    }
    else
    {
        if (shoot_control.shoot_flag == 0                                                       // 条件1:单发限制允许发射下一发
       && (class_fusi_rc.Processed_Data.Right_Switch_Status==Fusi_Two_Stage_Switch_Status_UP_TRIG_DOWN || class_fusi_rc.Processed_Data.Right_Switch_Status==Fusi_Two_Stage_Switch_Status_DOWN_TRIG_UP)
       && shoot_control.heat_allow_fire_flag==1
      ) // 条件2:手动触发发弹标志
        {
            shoot_control.shoot_flag = 1;
        }
    }

}



void Get_dt7_rc_gongdan_flag(void)
{
    //重置状态

    if (class_dr16.Processed_Data.Yaw > 0.5)
    {
        shoot_control.rc_gongdan_flag=1;
    }
    else
    {
        shoot_control.rc_gongdan_flag=0;
    }

}

void Get_vt13_rc_gongdan_flag(void)
{
    //重置状态

    if (class_vt13_rc.Processed_Data.Wheel > 0.5)
    {
        shoot_control.rc_gongdan_flag=1;
    }
    else
    {
        shoot_control.rc_gongdan_flag=0;
    }

}


void Get_fusi_rc_gongdan_flag(void)
{
    //重置状态
    uint16_t tmp_rc_gongdan_flag=0;

}

void Get_dt7_rc_huibo_flag(void)
{

    // 发射模式时将shoot_flag置零, 允许发射下颗弹丸(为了单发)
    if (RC_Control.shoot_control_status==Remote_Control)
    {
        if (shoot_control.shoot_dr16_processed_data->Yaw < 0.4)
        {
            shoot_control.rc_huibo_flag = 0;
        }
    }

    if ( shoot_control.shoot_flag == 0                                                       // 条件1:单发限制允许发射下一发
       && (shoot_control.shoot_dr16_processed_data->Yaw > 0.8 ) ) // 条件2:手动触发发弹标志
    {
        shoot_control.rc_huibo_flag = 1;
    }

}

void Get_vt13_rc_huibo_flag(void)
{

    // 发射模式时将shoot_flag置零, 允许发射下颗弹丸(为了单发)
    if (RC_Control.shoot_control_status==Remote_Control)
    {
        if (shoot_control.shoot_vt13_processed_data->Wheel < 0.4)
        {
            shoot_control.rc_huibo_flag = 0;
        }
    }

    if ( shoot_control.shoot_flag == 0                                                       // 条件1:单发限制允许发射下一发
       && (shoot_control.shoot_vt13_processed_data->Wheel > 0.8 ) ) // 条件2:手动触发发弹标志
    {
        shoot_control.rc_huibo_flag = 1;
    }

}


void Get_fusi_rc_huibo_flag(void)
{
    //重置状态
    uint16_t tmp_rc_huibo_flag=0;

}