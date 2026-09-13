#include "gimbal_task.h"
#include "gimbal_behaviour.h"
#include "buzzer.h"
//函数声明------------------------------------------------------------------------
static void gimbal_init(gimbal_control_t *init);
static void gimbal_set_mode(gimbal_control_t *set_mode);
static void gimbal_feedback_update(gimbal_control_t *feedback_update);
static void gimbal_mode_change_control_transit(gimbal_control_t *mode_change);
static void gimbal_set_control(gimbal_control_t *set_control);
static void gimbal_control_loop(gimbal_control_t *control_loop);
static void gimbal_output_to_motor(gimbal_control_t *gimbal_motor);

static void gimbal_yaw_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
static void gimbal_yaw_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
static void gimbal_yaw_turn_round_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
static void gimbal_yaw_init_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
static void gimbal_pitch_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
static void gimbal_pitch_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);
static void gimbal_pitch_init_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);

static void gimbal_motor_zero_force_control(gimbal_motor_t *gimbal_motor);
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor);
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor);
static void gimbal_motor_init_angle_control(gimbal_motor_t *gimbal_motor);

void GravComp_Init(Gravity_Comp_t *gravity_comp, float K, float m, float g, float arm_len, float phase);
float GravComp_Calc(Gravity_Comp_t *gravity_comp, float pitch_angle);

void Gimbal_Motor_Status_PeriodElapsedCallback(gimbal_control_t *gimbal_control);
void gimbal_motor_keep_alive(gimbal_control_t *gimbal_control);
void gimbal_motor_status_influence_output(gimbal_motor_t *gimbal_motor);
void gimbal_rc_ctrl_disable(void);

void Gimbal_FSM_TIM_Calculate_PeriodElapsedCallback(gimbal_control_t *gimbal_fsm);
void FSM_TIM_Calculate_PeriodElapsedCallback_Avoid_Yaw(FSM_t *FSM);
void FSM_TIM_Calculate_PeriodElapsedCallback_Avoid_Pitch(FSM_t *FSM);
void FSM_Gimbal_Init_TIM_Calculate_PeriodElapsedCallback(FSM_t *FSM);

//------------------------------------------------------------------------------


//重要参数宏定义----------------------------------------------------------------


//重要变量------------------------------------------------------------------------
//云台任务结构体
gimbal_control_t gimbal_control;
float ceshi_liju=0;

float guance_leso=0.0f;
float bili=0;
//------------------------------------------------------------------------------

/**
 * @brief          云台任务，间隔 GIMBAL_CONTROL_TIME 1ms
 * @param[in]      pvParameters: 空
 * @retval         none
 */
void Gimbal_Task(void const *argument)
{
    // 等待陀螺仪任务更新陀螺仪数据
    vTaskDelay(GIMBAL_TASK_INIT_TIME);

    gimbal_init(&gimbal_control);   // 云台初始化

    while (1)
    {
        gimbal_control.GIMBAL_xTickCount = xTaskGetTickCount();

        gimbal_set_mode(&gimbal_control);                           // 设置云台控制模式
        gimbal_mode_change_control_transit(&gimbal_control);     // 控制模式切换 控制数据过渡
        gimbal_feedback_update(&gimbal_control);               // 云台数据反馈
        gimbal_set_control(&gimbal_control);                       // 设置云台控制量
        gimbal_control_loop(&gimbal_control);                     // 云台控制计算

        //云台电机状态检测回调函数
        Gimbal_Motor_Status_PeriodElapsedCallback(&gimbal_control);

        // //判断遥控器是否在线
        if (RC_Control.gimbal_control_status==No_Control)
        {
            // // 遥控器断开连接，禁用所有电机
             gimbal_rc_ctrl_disable();
        }

        //输出发送到电机
        gimbal_output_to_motor(&gimbal_control);    //发送电机力矩

        osDelay(1);
    }

}

/**
 * @brief          返回yaw 电机数据指针
 * @param[in]      none
 * @retval         yaw电机指针
 */
const gimbal_motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_j4310;
}

/**
 * @brief          返回pitch 电机数据指针
 * @param[in]      none
 * @retval         pitch
 */
const gimbal_motor_t *get_pitch_motor_point(void)
{
    return &gimbal_control.gimbal_pitch_j4340;
}


/**
 * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
 * @param[out]     init:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_init(gimbal_control_t *init)
{
    // 陀螺仪数据指针获取
    init->gimbal_INS_point = get_INS_point();

    // 获取遥控器指针
    init->gimbal_dr16_processed_data = get_dr16_processed_data_point();
    init->gimbal_vt13_processed_data = get_vt13_processed_data_point();
    init->gimbal_fusi_rc_processed_data = get_fusi_rc_processed_data_point();

    // 获取上位机视觉数据指针
    init->gimbal_vision_point = get_vision_gimbal_point();

    //达妙电机初始化
    Motor_DM_Normal_Init(&init->gimbal_yaw_j4310.dm_normal_motor, &hfdcan3, 0x55, 0x05, Motor_DM_Control_Method_NORMAL_MIT, PI, 30.0f, 10.0f, 3.0f,1);
    Motor_DM_Normal_Init(&init->gimbal_pitch_j4340.dm_normal_motor, &hfdcan2, 0x66, 0x06, Motor_DM_Control_Method_NORMAL_MIT, PI, 30.0f, 10.0f, 3.0f,-1);

    // 初始化云台控制模式
    gimbal_behaviour = GIMBAL_ZERO_FORCE;

    // 初始化电机模式
    init->gimbal_yaw_j4310.gimbal_motor_mode = GIMBAL_MOTOR_ZERO_FORCE_CONTROL;
    init->gimbal_pitch_j4340.gimbal_motor_mode  = GIMBAL_MOTOR_ZERO_FORCE_CONTROL;

    // 重力补偿初始化
    GravComp_Init(&init->gimbal_pitch_j4340.gravity_comp, 1.0f, 2.218f, 9.8f, 0.02294f, 54.83f);

    //leso扩张观测器初始化
    init->leso_pitch_first_run_flag=0;

    //电机PID初始化-------------------------------------------------------------------
    //mit模式下--------------
    //yaw轴PID角度环PID
    PID_Init(&init->gimbal_yaw_j4310.dm_normal_motor.Angle_PID, Yaw_4310_Angle_PID_KP, Yaw_4310_Angle_PID_KI, Yaw_4310_Angle_PID_KD, Yaw_4310_Angle_PID_KF, Yaw_4310_Angle_PID_MAX_IOUT, Yaw_4310_Angle_PID_MAX_OUT, PID_D_T, Yaw_4310_Angle_PID_DEAD_ZONE, Yaw_4310_Angle_I_Variable_Speed_A, Yaw_4310_Angle_I_Variable_Speed_B, Yaw_4310_Angle_I_Separate_Threshold, PID_D_First_ENABLE);
    //yaw轴PID角速度环PID
    PID_Init(&init->gimbal_yaw_j4310.dm_normal_motor.Omega_PID, Yaw_4310_Speed_PID_KP, Yaw_4310_Speed_PID_KI, Yaw_4310_Speed_PID_KD, Yaw_4310_Speed_PID_KF, Yaw_4310_Speed_PID_MAX_IOUT, Yaw_4310_Speed_PID_MAX_OUT, PID_D_T, Yaw_4310_Speed_PID_DEAD_ZONE, Yaw_4310_Speed_I_Variable_Speed_A, Yaw_4310_Speed_I_Variable_Speed_B, Yaw_4310_Speed_I_Separate_Threshold, PID_D_First_ENABLE);
    //mit模式下
    //Pitch轴PID角度环PID
    PID_Init(&init->gimbal_pitch_j4340.dm_normal_motor.Angle_PID, Pitch_4340_Angle_PID_KP, Pitch_4340_Angle_PID_KI, Pitch_4340_Angle_PID_KD, Pitch_4340_Angle_PID_KF, Pitch_4340_Angle_PID_MAX_IOUT, Pitch_4340_Angle_PID_MAX_OUT, PID_D_T, Pitch_4340_Angle_PID_DEAD_ZONE, Pitch_4340_Angle_I_Variable_Speed_A, Pitch_4340_Angle_I_Variable_Speed_B, Pitch_4340_Angle_I_Separate_Threshold, PID_D_First_DISABLE);
    //Pitch轴PID角速度环PID
    PID_Init(&init->gimbal_pitch_j4340.dm_normal_motor.Omega_PID, Pitch_4340_Speed_PID_KP, Pitch_4340_Speed_PID_KI, Pitch_4340_Speed_PID_KD, Pitch_4340_Speed_PID_KF, Pitch_4340_Speed_PID_MAX_IOUT, Pitch_4340_Speed_PID_MAX_OUT, PID_D_T, Pitch_4340_Speed_PID_DEAD_ZONE, Pitch_4340_Speed_I_Variable_Speed_A, Pitch_4340_Speed_I_Variable_Speed_B, Pitch_4340_Speed_I_Separate_Threshold, PID_D_First_DISABLE);
    //----------------------

    //init模式下pid初始化----------
    //yaw轴PID角度环PID
    PID_Init(&init->gimbal_yaw_j4310.Init_Angle_Pid, Yaw_4310_Init_Angle_PID_KP, Yaw_4310_Init_Angle_PID_KI, Yaw_4310_Init_Angle_PID_KD, Yaw_4310_Init_Angle_PID_KF, Yaw_4310_Init_Angle_PID_MAX_IOUT, Yaw_4310_Init_Angle_PID_MAX_OUT, PID_D_T, Yaw_4310_Init_Angle_PID_DEAD_ZONE, Yaw_4310_Init_Angle_I_Variable_Speed_A, Yaw_4310_Init_Angle_I_Variable_Speed_B, Yaw_4310_Init_Angle_I_Separate_Threshold, PID_D_First_ENABLE);
    //yaw轴PID角速度环PID
    PID_Init(&init->gimbal_yaw_j4310.Init_Omega_Pid, Yaw_4310_Init_Speed_PID_KP, Yaw_4310_Init_Speed_PID_KI, Yaw_4310_Init_Speed_PID_KD, Yaw_4310_Init_Speed_PID_KF, Yaw_4310_Init_Speed_PID_MAX_IOUT, Yaw_4310_Init_Speed_PID_MAX_OUT, PID_D_T, Yaw_4310_Init_Speed_PID_DEAD_ZONE, Yaw_4310_Init_Speed_I_Variable_Speed_A, Yaw_4310_Init_Speed_I_Variable_Speed_B, Yaw_4310_Init_Speed_I_Separate_Threshold, PID_D_First_ENABLE);

    //Pitch轴PID角度环PID
    PID_Init(&init->gimbal_pitch_j4340.Init_Angle_Pid, Pitch_4340_Init_Angle_PID_KP, Pitch_4340_Init_Angle_PID_KI, Pitch_4340_Init_Angle_PID_KD, Pitch_4340_Init_Angle_PID_KF, Pitch_4340_Init_Angle_PID_MAX_IOUT, Pitch_4340_Init_Angle_PID_MAX_OUT, PID_D_T, Pitch_4340_Init_Angle_PID_DEAD_ZONE, Pitch_4340_Init_Angle_I_Variable_Speed_A, Pitch_4340_Init_Angle_I_Variable_Speed_B, Pitch_4340_Init_Angle_I_Separate_Threshold, PID_D_First_DISABLE);
    //Pitch轴PID角速度环PID
    PID_Init(&init->gimbal_pitch_j4340.Init_Omega_Pid, Pitch_4340_Init_Speed_PID_KP, Pitch_4340_Init_Speed_PID_KI, Pitch_4340_Init_Speed_PID_KD, Pitch_4340_Init_Speed_PID_KF, Pitch_4340_Init_Speed_PID_MAX_IOUT, Pitch_4340_Init_Speed_PID_MAX_OUT, PID_D_T, Pitch_4340_Init_Speed_PID_DEAD_ZONE, Pitch_4340_Init_Speed_I_Variable_Speed_A, Pitch_4340_Init_Speed_I_Variable_Speed_B, Pitch_4340_Init_Speed_I_Separate_Threshold, PID_D_First_DISABLE);
    //----------------------------

    //-----------------------------------------------------------------------------


    //初始化云台机构电机状态机
    FSM_Init(&init->gimbal_yaw_j4310.Motor_FSM, 4, Normal);
    FSM_Init(&init->gimbal_pitch_j4340.Motor_FSM, 4, Normal);

    //初始化Yaw轴Init状态机
    FSM_Init(&init->FSM_Yaw_Init, 4, INIT_ING);

    //机器人能动标志位初始化
    gimbal_control.gimbal_move_flag = 0;

    //一键吊射的标志位
    gimbal_control.one_key_shoot_flag=0;

    //初始化开机音效
    Buzzer_Play(startSound_1_9len, 9, SINGLE, 0);

    //云台yaw轴初始化正对角度
    init->init_angle=2.4f;

    //设置pitch轴绝对角最大值
    init->gimbal_pitch_j4340.max_absolute_angle = 0.55f;
    init->gimbal_pitch_j4340.min_absolute_angle = -0.2f;

    // 云台数据更新
    gimbal_feedback_update(init);

    // yaw轴绝对角,相对角初始化
    init->gimbal_yaw_j4310.absolute_angle_set = init->gimbal_yaw_j4310.absolute_angle;
    init->gimbal_yaw_j4310.relative_angle_set = init->gimbal_yaw_j4310.relative_angle;
    // pitch轴绝对角,相对角初始化
    init->gimbal_pitch_j4340.absolute_angle_set = init->gimbal_pitch_j4340.absolute_angle;
    init->gimbal_pitch_j4340.relative_angle_set = init->gimbal_pitch_j4340.relative_angle;

    gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Target_Angle = gimbal_control.gimbal_yaw_j4310.dm_normal_motor.INS_point->Yaw;
    gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Target_Angle = -gimbal_control.gimbal_pitch_j4340.dm_normal_motor.INS_point->Pitch;

}

/**
 * @brief          设置云台控制模式，主要在'gimbal_behaviour_mode_set'函数中改变
 * @param[out]     gimbal_set_mode:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_set_mode(gimbal_control_t *set_mode)
{
    if (set_mode == NULL)
    {
        return;
    }

    //云台模式设置
    gimbal_behaviour_mode_set(set_mode);

    //Yaw轴Init状态机改变模式，因为用到这次控制模式和上次控制模式，所以得放在设置模式之后，last_gimbal_motor_mode更新之前
    FSM_Gimbal_Init_TIM_Calculate_PeriodElapsedCallback(&gimbal_control.FSM_Yaw_Init);

}

/**
 * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
 * @param[out]     gimbal_feedback_update:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_feedback_update(gimbal_control_t *feedback_update)
{
    if (feedback_update == NULL)
    {
        return;
    }

    // 云台数据更新
    //Yaw数据更新
    feedback_update->gimbal_yaw_j4310.absolute_angle = feedback_update->gimbal_INS_point->Yaw;
    feedback_update->gimbal_yaw_j4310.relative_angle = normalizeAngleToPi_Robust(feedback_update->gimbal_yaw_j4310.dm_normal_motor.Rx_Data.Now_Angle-feedback_update->init_angle);

    //Pitch数据更新
    feedback_update->gimbal_pitch_j4340.absolute_angle = -feedback_update->gimbal_INS_point->Pitch;
}

/**
 * @brief          云台模式改变，有些参数需要改变，例如控制yaw角度设定值应该变成当前yaw角度
 * @param[out]     gimbal_mode_change:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change)
{
    if (gimbal_mode_change == NULL)
    {
        return;
    }

// yaw电机状态机切换保存数据---------------------------------------------------------
    if (gimbal_mode_change->gimbal_yaw_j4310.last_gimbal_motor_mode != GIMBAL_MOTOR_ZERO_FORCE_CONTROL && gimbal_mode_change->gimbal_yaw_j4310.gimbal_motor_mode == GIMBAL_MOTOR_ZERO_FORCE_CONTROL)
    {
        //切入云台无力模式
        gimbal_mode_change->gimbal_yaw_j4310.dm_normal_motor.Target_Angle = gimbal_mode_change->gimbal_yaw_j4310.dm_normal_motor.INS_point->Yaw;
        //yaw角度环,速度环积分项清零
        PID_Set_Integral_Error(&gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Angle_PID, 0.0f);
        PID_Set_Integral_Error(&gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Omega_PID, 0.0f);
    }
    else if (gimbal_mode_change->gimbal_yaw_j4310.last_gimbal_motor_mode != GIMBAL_MOTOR_ABSOLUTE_ANGLE_CONTROL && gimbal_mode_change->gimbal_yaw_j4310.gimbal_motor_mode == GIMBAL_MOTOR_ABSOLUTE_ANGLE_CONTROL)
    {
        //切入云台无力模式
        gimbal_mode_change->gimbal_yaw_j4310.dm_normal_motor.Target_Angle = gimbal_mode_change->gimbal_yaw_j4310.dm_normal_motor.INS_point->Yaw;
        //yaw角度环,速度环积分项清零
        PID_Set_Integral_Error(&gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Angle_PID, 0.0f);
        PID_Set_Integral_Error(&gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Omega_PID, 0.0f);
    }
    else if (gimbal_mode_change->gimbal_yaw_j4310.last_gimbal_motor_mode != GIMBAL_MOTOR_RELATIVE_ANGLE_CONTROL && gimbal_mode_change->gimbal_yaw_j4310.gimbal_motor_mode == GIMBAL_MOTOR_RELATIVE_ANGLE_CONTROL)
    {
        //切入云台相对角控制模式
        //切入云台无力模式
        gimbal_mode_change->gimbal_yaw_j4310.dm_normal_motor.Target_Angle = gimbal_mode_change->gimbal_yaw_j4310.dm_normal_motor.INS_point->Yaw;
        //yaw角度环,速度环积分项清零
        PID_Set_Integral_Error(&gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Angle_PID, 0.0f);
        PID_Set_Integral_Error(&gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Omega_PID, 0.0f);
    }
    else if (gimbal_mode_change->gimbal_yaw_j4310.last_gimbal_motor_mode != GIMBAL_MOTOR_AUTO_CONTROL && gimbal_mode_change->gimbal_yaw_j4310.gimbal_motor_mode == GIMBAL_MOTOR_AUTO_CONTROL)
    {
        //切入云台无力模式
        gimbal_mode_change->gimbal_yaw_j4310.dm_normal_motor.Target_Angle = gimbal_mode_change->gimbal_yaw_j4310.dm_normal_motor.INS_point->Yaw;
        //yaw角度环,速度环积分项清零
        PID_Set_Integral_Error(&gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Angle_PID, 0.0f);
        PID_Set_Integral_Error(&gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Omega_PID, 0.0f);
    }
    else if (gimbal_mode_change->gimbal_yaw_j4310.last_gimbal_motor_mode != GIMBAL_MOTOR_REMOTE_FIRE_CONTROL && gimbal_mode_change->gimbal_yaw_j4310.gimbal_motor_mode == GIMBAL_MOTOR_REMOTE_FIRE_CONTROL)
    {
        //切入云台无力模式
        gimbal_mode_change->gimbal_yaw_j4310.dm_normal_motor.Target_Angle = gimbal_mode_change->gimbal_yaw_j4310.dm_normal_motor.INS_point->Yaw;
        //yaw角度环,速度环积分项清零
        PID_Set_Integral_Error(&gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Angle_PID, 0.0f);
        PID_Set_Integral_Error(&gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Omega_PID, 0.0f);
    }
    else if (gimbal_mode_change->gimbal_yaw_j4310.last_gimbal_motor_mode != GIMBAL_MOTOR_TURN_ROUND_CONTROL && gimbal_mode_change->gimbal_yaw_j4310.gimbal_motor_mode == GIMBAL_MOTOR_TURN_ROUND_CONTROL)
    {
        //切入一键调头模式
        //记录一键调头初始位置
        gimbal_mode_change->gimbal_turn_round_init_angle = gimbal_mode_change->gimbal_yaw_j4310.dm_normal_motor.INS_point->Yaw; // 记录初始角度
        //yaw角度环,速度环积分项清零
        PID_Set_Integral_Error(&gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Angle_PID, 0.0f);
        PID_Set_Integral_Error(&gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Omega_PID, 0.0f);
    }
    else if (gimbal_mode_change->gimbal_yaw_j4310.last_gimbal_motor_mode != GIMBAL_MOTOR_INIT_CONTROL && gimbal_mode_change->gimbal_yaw_j4310.gimbal_motor_mode == GIMBAL_MOTOR_INIT_CONTROL)
    {
        //切入云台无力模式
        gimbal_mode_change->gimbal_yaw_j4310.dm_normal_motor.Target_Angle = gimbal_mode_change->gimbal_yaw_j4310.dm_normal_motor.INS_point->Yaw;
        //yaw角度环,速度环积分项清零
        PID_Set_Integral_Error(&gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Angle_PID, 0.0f);
        PID_Set_Integral_Error(&gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Omega_PID, 0.0f);
    }


    gimbal_mode_change->gimbal_yaw_j4310.last_gimbal_motor_mode = gimbal_mode_change->gimbal_yaw_j4310.gimbal_motor_mode;




// pitch电机状态机切换保存数据-------------------------------------------------------
    if (gimbal_mode_change->gimbal_pitch_j4340.last_gimbal_motor_mode != GIMBAL_MOTOR_ZERO_FORCE_CONTROL && gimbal_mode_change->gimbal_pitch_j4340.gimbal_motor_mode == GIMBAL_MOTOR_ZERO_FORCE_CONTROL)
    {
        //leso观测器重置
        gimbal_control.leso_pitch_first_run_flag=0;

        //切入云台无力模式
        gimbal_mode_change->gimbal_pitch_j4340.dm_normal_motor.Target_Angle = -gimbal_mode_change->gimbal_pitch_j4340.dm_normal_motor.INS_point->Pitch;
        //pitch角度环,速度环积分项清零
        PID_Set_Integral_Error(&gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Angle_PID, 0.0f);
        PID_Set_Integral_Error(&gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Omega_PID, 0.0f);
    }
    else if (gimbal_mode_change->gimbal_pitch_j4340.last_gimbal_motor_mode != GIMBAL_MOTOR_ABSOLUTE_ANGLE_CONTROL && gimbal_mode_change->gimbal_pitch_j4340.gimbal_motor_mode == GIMBAL_MOTOR_ABSOLUTE_ANGLE_CONTROL)
    {
        //leso观测器重置
        gimbal_control.leso_pitch_first_run_flag=0;

        //切入云台绝对角控制模式
        gimbal_mode_change->gimbal_pitch_j4340.dm_normal_motor.Target_Angle = -gimbal_mode_change->gimbal_pitch_j4340.dm_normal_motor.INS_point->Pitch;
        //pitch角度环,速度环积分项清零
        PID_Set_Integral_Error(&gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Angle_PID, 0.0f);
        PID_Set_Integral_Error(&gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Omega_PID, 0.0f);
    }
    else if (gimbal_mode_change->gimbal_pitch_j4340.last_gimbal_motor_mode != GIMBAL_MOTOR_AUTO_CONTROL && gimbal_mode_change->gimbal_pitch_j4340.gimbal_motor_mode == GIMBAL_MOTOR_AUTO_CONTROL)
    {
        //leso观测器重置
        gimbal_control.leso_pitch_first_run_flag=0;

        //切入自瞄控制模式
        gimbal_mode_change->gimbal_pitch_j4340.dm_normal_motor.Target_Angle = -gimbal_mode_change->gimbal_pitch_j4340.dm_normal_motor.INS_point->Pitch;
        //pitch角度环,速度环积分项清零
        PID_Set_Integral_Error(&gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Angle_PID, 0.0f);
        PID_Set_Integral_Error(&gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Omega_PID, 0.0f);
    }
    else if (gimbal_mode_change->gimbal_pitch_j4340.last_gimbal_motor_mode != GIMBAL_MOTOR_REMOTE_FIRE_CONTROL && gimbal_mode_change->gimbal_pitch_j4340.gimbal_motor_mode == GIMBAL_MOTOR_REMOTE_FIRE_CONTROL)
    {
        //leso观测器重置
        gimbal_control.leso_pitch_first_run_flag=0;

        //切入吊射模式
        gimbal_mode_change->gimbal_pitch_j4340.dm_normal_motor.Target_Angle = -gimbal_mode_change->gimbal_pitch_j4340.dm_normal_motor.INS_point->Pitch;
        //pitch角度环,速度环积分项清零
        PID_Set_Integral_Error(&gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Angle_PID, 0.0f);
        PID_Set_Integral_Error(&gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Omega_PID, 0.0f);
    }
    else if (gimbal_mode_change->gimbal_pitch_j4340.last_gimbal_motor_mode != GIMBAL_MOTOR_INIT_CONTROL && gimbal_mode_change->gimbal_pitch_j4340.gimbal_motor_mode == GIMBAL_MOTOR_INIT_CONTROL)
    {
        //leso观测器重置
        gimbal_control.leso_pitch_first_run_flag=0;

        //切入云台绝对角控制模式
        gimbal_mode_change->gimbal_pitch_j4340.dm_normal_motor.Target_Angle = -gimbal_mode_change->gimbal_pitch_j4340.dm_normal_motor.INS_point->Pitch;
        //pitch角度环,速度环积分项清零
        PID_Set_Integral_Error(&gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Angle_PID, 0.0f);
        PID_Set_Integral_Error(&gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Omega_PID, 0.0f);
    }

    gimbal_mode_change->gimbal_pitch_j4340.last_gimbal_motor_mode = gimbal_mode_change->gimbal_pitch_j4340.gimbal_motor_mode;
}

/**
 * @brief          设置云台控制设定值，控制值是通过gimbal_behaviour_control_set函数设置的
 * @param[out]     gimbal_set_control:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_set_control(gimbal_control_t *set_control)
{
    if (set_control == NULL)
    {
        return;
    }

    fp32 add_yaw_angle = 0.0f;
    fp32 add_pitch_angle = 0.0f;

    //遥控器值赋予yaw,pitch轴角度增量
    gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, set_control);

     //对yaw轴数据进行处理,然后赋予电机目标值
     if (set_control->gimbal_yaw_j4310.gimbal_motor_mode == GIMBAL_MOTOR_ZERO_FORCE_CONTROL)
     {
         // 无力模式下
         gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Target_Angle     = gimbal_control.gimbal_yaw_j4310.dm_normal_motor.INS_point->Yaw;
         gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Angle_PID.Target = gimbal_control.gimbal_yaw_j4310.dm_normal_motor.INS_point->Yaw;
         gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Angle_PID.Now    = gimbal_control.gimbal_yaw_j4310.dm_normal_motor.INS_point->Yaw;
         gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Omega_PID.Target = gimbal_control.gimbal_yaw_j4310.dm_normal_motor.INS_point->Gyro[2];
         gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Omega_PID.Now    = gimbal_control.gimbal_yaw_j4310.dm_normal_motor.INS_point->Gyro[2];
     }
     else if (set_control->gimbal_yaw_j4310.gimbal_motor_mode == GIMBAL_MOTOR_ABSOLUTE_ANGLE_CONTROL)
     {
         // 绝对角模式下，陀螺仪角度控制
         gimbal_yaw_absolute_angle_limit(&set_control->gimbal_yaw_j4310, add_yaw_angle);
     }
     else if (set_control->gimbal_yaw_j4310.gimbal_motor_mode == GIMBAL_MOTOR_RELATIVE_ANGLE_CONTROL)
     {
         // 相对角模式下，电机角度控制
         gimbal_yaw_relative_angle_limit(&set_control->gimbal_yaw_j4310, add_yaw_angle);
     }
     else if (set_control->gimbal_yaw_j4310.gimbal_motor_mode == GIMBAL_MOTOR_AUTO_CONTROL)
     {
         // AUTO模式下，陀螺仪角度控制
         gimbal_yaw_absolute_angle_limit(&set_control->gimbal_yaw_j4310, add_yaw_angle);
     }
     else if (set_control->gimbal_yaw_j4310.gimbal_motor_mode == GIMBAL_MOTOR_REMOTE_FIRE_CONTROL)
     {
         // 吊射模式下，yaw相对角控制
         gimbal_yaw_absolute_angle_limit(&set_control->gimbal_yaw_j4310, add_yaw_angle);
     }
     else if (set_control->gimbal_yaw_j4310.gimbal_motor_mode == GIMBAL_MOTOR_TURN_ROUND_CONTROL)
     {
         gimbal_yaw_turn_round_absolute_angle_limit(&set_control->gimbal_yaw_j4310,add_yaw_angle);
     }
     else if (set_control->gimbal_yaw_j4310.gimbal_motor_mode == GIMBAL_MOTOR_INIT_CONTROL)
     {
         gimbal_yaw_init_angle_limit(&set_control->gimbal_yaw_j4310,add_yaw_angle);
     }




     // 对pitch轴数据进行处理,然后赋予电机目标值
     // pitch电机模式控制
     if (set_control->gimbal_pitch_j4340.gimbal_motor_mode == GIMBAL_MOTOR_ZERO_FORCE_CONTROL)
     {
         // 无力模式下
         gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Target_Angle     =-gimbal_control.gimbal_pitch_j4340.dm_normal_motor.INS_point->Pitch;
         gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Angle_PID.Target =-gimbal_control.gimbal_pitch_j4340.dm_normal_motor.INS_point->Pitch;
         gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Angle_PID.Now    =-gimbal_control.gimbal_pitch_j4340.dm_normal_motor.INS_point->Pitch;
         gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Omega_PID.Target = gimbal_control.gimbal_pitch_j4340.dm_normal_motor.INS_point->Gyro[1];
         gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Omega_PID.Now    = gimbal_control.gimbal_pitch_j4340.dm_normal_motor.INS_point->Gyro[1];
     }
     else if (set_control->gimbal_pitch_j4340.gimbal_motor_mode == GIMBAL_MOTOR_ABSOLUTE_ANGLE_CONTROL)
     {
         // gyro模式下，陀螺仪角度控制
         gimbal_pitch_absolute_angle_limit(&set_control->gimbal_pitch_j4340, add_pitch_angle);
     }
     else if (set_control->gimbal_pitch_j4340.gimbal_motor_mode == GIMBAL_MOTOR_AUTO_CONTROL)
     {
         // AUTO模式下，陀螺仪角度控制
         gimbal_pitch_absolute_angle_limit(&set_control->gimbal_pitch_j4340, add_pitch_angle);
     }
     else if (set_control->gimbal_pitch_j4340.gimbal_motor_mode == GIMBAL_MOTOR_REMOTE_FIRE_CONTROL)
     {
         // gyro模式下，陀螺仪角度控制
         gimbal_pitch_absolute_angle_limit(&set_control->gimbal_pitch_j4340, add_pitch_angle);
     }
     else if (set_control->gimbal_pitch_j4340.gimbal_motor_mode == GIMBAL_MOTOR_INIT_CONTROL)
     {
         // gyro模式下，陀螺仪角度控制
         gimbal_pitch_init_angle_limit(&set_control->gimbal_pitch_j4340, add_pitch_angle);
     }

}

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_yaw_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
     static fp32 angle_set_yaw, temp_yaw, tmp_delta_angle = 0;

     temp_yaw = gimbal_control.gimbal_INS_point->Yaw;
     angle_set_yaw = gimbal_motor->dm_normal_motor.Target_Angle;
     tmp_delta_angle = fmod((angle_set_yaw + add) - (temp_yaw), 2.0f * PI);

     if (tmp_delta_angle > PI)
     {
         tmp_delta_angle -= 2.0f * PI;
     }
     else if (tmp_delta_angle < -PI)
     {
         tmp_delta_angle += 2.0f * PI;
     }
     gimbal_motor->dm_normal_motor.Target_Angle = gimbal_control.gimbal_INS_point->Yaw + tmp_delta_angle;
}

static void gimbal_yaw_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

    static fp32 angle_set_yaw, temp_yaw, tmp_delta_angle = 0;

    temp_yaw = gimbal_motor->dm_normal_motor.Rx_Data.Now_Angle;
    angle_set_yaw = gimbal_motor->dm_normal_motor.Target_Angle;
    tmp_delta_angle = fmod((angle_set_yaw + add) - (temp_yaw), 2.0f * PI);

    if (tmp_delta_angle > PI)
    {
        tmp_delta_angle -= 2.0f * PI;
    }
    else if (tmp_delta_angle < -PI)
    {
        tmp_delta_angle += 2.0f * PI;
    }
    gimbal_motor->dm_normal_motor.Target_Angle = gimbal_motor->dm_normal_motor.Rx_Data.Now_Angle + tmp_delta_angle;

}

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制掉头
 * @param[out]     gimbal_motor:yaw电机
 * @retval         none
 */
static void gimbal_yaw_turn_round_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    static fp32 angle_set_yaw, temp_yaw, tmp_delta_angle = 0;

    temp_yaw = gimbal_control.gimbal_INS_point->Yaw;
    angle_set_yaw = (gimbal_control.gimbal_turn_round_init_angle + 3.14f);

    tmp_delta_angle = atan2f(sinf(angle_set_yaw - temp_yaw),cosf(angle_set_yaw - temp_yaw));

    // 直接输出目标角度，无跳变
    gimbal_motor->dm_normal_motor.Target_Angle = temp_yaw + tmp_delta_angle;
}

static void gimbal_yaw_init_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    if ( fabs(gimbal_motor->relative_angle) < PI/2.0)
    {
        gimbal_motor->dm_normal_motor.Target_Angle = 0;
    }
    else
    {
        gimbal_motor->dm_normal_motor.Target_Angle = 3.14;
    }

    static fp32 angle_set_yaw, temp_yaw, tmp_delta_angle = 0;

    temp_yaw = gimbal_motor->relative_angle;
    angle_set_yaw = gimbal_motor->dm_normal_motor.Target_Angle;
    tmp_delta_angle = fmod((angle_set_yaw) - (temp_yaw), 2.0f * PI);

    if (tmp_delta_angle > PI)
    {
        tmp_delta_angle -= 2.0f * PI;
    }
    else if (tmp_delta_angle < -PI)
    {
        tmp_delta_angle += 2.0f * PI;
    }

    gimbal_motor->dm_normal_motor.Target_Angle = gimbal_motor->relative_angle + tmp_delta_angle;

    //  //初始化模式下，yaw的目标是与底盘正前方相对角为0
    // gimbal_motor->dm_normal_motor.Target_Angle = 0;

}

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_GYRO时，使用陀螺仪计算的欧拉角进行控制
 * @param[out]     gimbal_motor:pitch电机
 * @retval         将原相对角度改为相对底盘陀螺仪角度
 */
static void gimbal_pitch_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    if (gimbal_motor->dm_normal_motor.Target_Angle + add > gimbal_motor->max_absolute_angle)
    {
        // 如果是往最大机械角度控制方向
        if (add > 0.0f)
        {
            // calculate max add_angle
            // 计算出一个最大的添加角度
            add = gimbal_motor->max_absolute_angle - gimbal_motor->dm_normal_motor.Target_Angle;
        }
    }
    else if (gimbal_motor->dm_normal_motor.Target_Angle + add < gimbal_motor->min_absolute_angle)
    {
        if (add < 0.0f)
        {
            add = gimbal_motor->min_absolute_angle - gimbal_motor->dm_normal_motor.Target_Angle;
        }
    }
    else
    {
        add+=0;
    }

     //gimbal_motor->dm_normal_motor.Target_Angle = rad_format(gimbal_motor->dm_normal_motor.Target_Angle + add);

    if (gimbal_control.one_key_shoot_flag==1)
    {
        gimbal_control.one_key_shoot_pitch += RC_Control.key_mouse.Mouse_Z*Key_Mouse_Pitch_Scale;
        gimbal_motor->dm_normal_motor.Target_Angle = gimbal_control.one_key_shoot_pitch;

        if (gimbal_motor->dm_normal_motor.Target_Angle > gimbal_motor->max_absolute_angle)
        {
            gimbal_motor->dm_normal_motor.Target_Angle = gimbal_motor->max_absolute_angle;
        }
        else if (gimbal_motor->dm_normal_motor.Target_Angle < gimbal_motor->min_absolute_angle)
        {
            gimbal_motor->dm_normal_motor.Target_Angle = gimbal_motor->min_absolute_angle;
        }
        else
        {

        }
    }
    else
    {
        gimbal_motor->dm_normal_motor.Target_Angle = rad_format(gimbal_motor->dm_normal_motor.Target_Angle + add);
    }
}

static void gimbal_pitch_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    if (gimbal_motor->dm_normal_motor.Target_Angle + add > gimbal_motor->max_relative_angle)
    {
        // 如果是往最大机械角度控制方向
        if (add > 0.0f)
        {
            // calculate max add_angle
            // 计算出一个最大的添加角度
            add = gimbal_motor->max_relative_angle - gimbal_motor->dm_normal_motor.Target_Angle;
        }
    }
    else if (gimbal_motor->dm_normal_motor.Target_Angle + add < gimbal_motor->min_relative_angle)
    {
        if (add < 0.0f)
        {
            add = gimbal_motor->min_relative_angle - gimbal_motor->dm_normal_motor.Target_Angle;
        }
    }
    else
    {
        add+=0;
    }

    gimbal_motor->dm_normal_motor.Target_Angle = rad_format(gimbal_motor->dm_normal_motor.Target_Angle + add);
}

static void gimbal_pitch_init_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{

    //初始化模式下，pitch保持水平
    gimbal_motor->dm_normal_motor.Target_Angle = 0;

}

/**
 * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
 * @param[out]     gimbal_control_loop:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_control_loop(gimbal_control_t *control_loop)
{
     if (control_loop == NULL)
     {
         return;
     }
     //Yaw
     if (control_loop->gimbal_yaw_j4310.gimbal_motor_mode == GIMBAL_MOTOR_ZERO_FORCE_CONTROL)
     {
         gimbal_motor_zero_force_control(&control_loop->gimbal_yaw_j4310);
     }
     else if (control_loop->gimbal_yaw_j4310.gimbal_motor_mode == GIMBAL_MOTOR_ABSOLUTE_ANGLE_CONTROL)
     {
         gimbal_motor_absolute_angle_control(&control_loop->gimbal_yaw_j4310);
     }
     else if (control_loop->gimbal_yaw_j4310.gimbal_motor_mode == GIMBAL_MOTOR_RELATIVE_ANGLE_CONTROL)
     {
         gimbal_motor_relative_angle_control(&control_loop->gimbal_yaw_j4310);
     }
     else if (control_loop->gimbal_yaw_j4310.gimbal_motor_mode == GIMBAL_MOTOR_AUTO_CONTROL)
     {
         gimbal_motor_absolute_angle_control(&control_loop->gimbal_yaw_j4310);
     }
     else if (control_loop->gimbal_yaw_j4310.gimbal_motor_mode == GIMBAL_MOTOR_REMOTE_FIRE_CONTROL)
     {
         gimbal_motor_absolute_angle_control(&control_loop->gimbal_yaw_j4310);
     }
     else if (control_loop->gimbal_yaw_j4310.gimbal_motor_mode == GIMBAL_MOTOR_TURN_ROUND_CONTROL)
     {
         gimbal_motor_absolute_angle_control(&control_loop->gimbal_yaw_j4310);
     }
     else if (control_loop->gimbal_yaw_j4310.gimbal_motor_mode == GIMBAL_MOTOR_INIT_CONTROL)
     {
         gimbal_motor_init_angle_control(&control_loop->gimbal_yaw_j4310);
     }
     else
     {
         gimbal_motor_zero_force_control(&control_loop->gimbal_yaw_j4310);
     }



     //Pitch
     if (control_loop->gimbal_pitch_j4340.gimbal_motor_mode == GIMBAL_MOTOR_ZERO_FORCE_CONTROL)
     {
         gimbal_motor_zero_force_control(&control_loop->gimbal_pitch_j4340);
     }
     else if (control_loop->gimbal_pitch_j4340.gimbal_motor_mode == GIMBAL_MOTOR_ABSOLUTE_ANGLE_CONTROL)
     {
         gimbal_motor_absolute_angle_control(&control_loop->gimbal_pitch_j4340);     }
     else if (control_loop->gimbal_pitch_j4340.gimbal_motor_mode == GIMBAL_MOTOR_AUTO_CONTROL)
     {
         gimbal_motor_absolute_angle_control(&control_loop->gimbal_pitch_j4340);
     }
     else if (control_loop->gimbal_pitch_j4340.gimbal_motor_mode == GIMBAL_MOTOR_REMOTE_FIRE_CONTROL)
     {
         gimbal_motor_absolute_angle_control(&control_loop->gimbal_pitch_j4340);
     }
     else if (control_loop->gimbal_pitch_j4340.gimbal_motor_mode == GIMBAL_MOTOR_TURN_ROUND_CONTROL)
     {
         gimbal_motor_absolute_angle_control(&control_loop->gimbal_pitch_j4340);
     }
     else if (control_loop->gimbal_pitch_j4340.gimbal_motor_mode == GIMBAL_MOTOR_INIT_CONTROL)
     {
         gimbal_motor_init_angle_control(&control_loop->gimbal_pitch_j4340);
     }
     else
     {
         gimbal_motor_zero_force_control(&control_loop->gimbal_pitch_j4340);
     }
}

static void gimbal_motor_zero_force_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

    gimbal_motor->dm_normal_motor.Control_Torque = 0.0f;

}

static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

    if (gimbal_motor == &gimbal_control.gimbal_yaw_j4310)
    {
        if (gimbal_motor->gimbal_motor_mode == GIMBAL_MOTOR_AUTO_CONTROL)
        {
            gimbal_motor->dm_normal_motor.Feedforward_Omega=gimbal_control.gimbal_vision_point->gimbal_feed_forward_yaw_omega;
        }
        else
        {
            gimbal_motor->dm_normal_motor.Feedforward_Omega=0;
        }

        //LQR计算
        gimbal_motor->dm_normal_motor.Control_Torque = YAW_LQR_K1 * (gimbal_motor->dm_normal_motor.Target_Angle - gimbal_control.gimbal_INS_point->Yaw)
                                        + YAW_LQR_K2 * ( gimbal_control.gimbal_vision_point->gimbal_feed_forward_yaw_omega - gimbal_control.gimbal_INS_point->Gyro[2]) + 0.008*gimbal_control.gimbal_vision_point->gimbal_feed_forward_yaw_accel;
        //LQR计算
        // gimbal_motor->dm_normal_motor.Control_Torque = YAW_LQR_K1 * (gimbal_motor->dm_normal_motor.Target_Angle - gimbal_control.gimbal_INS_point->Yaw)
        //                                 + YAW_LQR_K2 * (-gimbal_control.gimbal_INS_point->Gyro[2]) + 0.008*gimbal_control.gimbal_vision_point->gimbal_feed_forward_yaw_accel;

        gimbal_motor->dm_normal_motor.Control_Torque = Math_Constrain(&gimbal_motor->dm_normal_motor.Control_Torque, -8.0f, 8.0f);


        //普通PID计算
        //Yaw_Motor_DM_Normal_TIM_1ms_Calculate_PeriodElapsedCallback(&gimbal_motor->dm_normal_motor);

    }
    else if (gimbal_motor == &gimbal_control.gimbal_pitch_j4340)
    {
        if (gimbal_motor->gimbal_motor_mode == GIMBAL_MOTOR_AUTO_CONTROL)
        {
            gimbal_motor->dm_normal_motor.Feedforward_Omega=gimbal_control.gimbal_vision_point->gimbal_feed_forward_pitch_omega;
        }
        else
        {
            gimbal_motor->dm_normal_motor.Feedforward_Omega=0;
        }

        //重力补偿前馈力矩计算
        //gimbal_motor->dm_normal_motor.Feedforward_Torque = GravComp_Calc(&gimbal_motor->gravity_comp, -gimbal_control.gimbal_INS_point->Pitch);

        if (gimbal_motor->gimbal_motor_mode == GIMBAL_MOTOR_ZERO_FORCE_CONTROL)
        {
            gimbal_motor->dm_normal_motor.Feedforward_Torque=0;
        }
        else
        {
            gimbal_motor->dm_normal_motor.Feedforward_Torque= 0.0f;
        }

        //普通PID计算
        Pitch_Motor_DM_Normal_TIM_1ms_Calculate_PeriodElapsedCallback(&gimbal_motor->dm_normal_motor);

        // //eso观测器计算--------------------------------------------
        float u_raw=0;
        float y_pitch=0;
        float disturb_feed_forward=0;

        if (gimbal_control.leso_pitch_first_run_flag==0)
        {
            LESO_Init(-gimbal_control.gimbal_INS_point->Pitch);

            gimbal_control.leso_pitch_first_run_flag=1;
        }

        if (gimbal_control.leso_pitch_first_run_flag)
        {
            // 1. 获取控制器输出
            u_raw=gimbal_motor->dm_normal_motor.Control_Torque;

            // 2. 获取IMU/编码器原始角度反馈
            y_pitch = -gimbal_control.gimbal_INS_point->Pitch;

            // 3. LESO发力，迭代计算观测状态和获得扰动前馈
            disturb_feed_forward = LESO_Update(u_raw, y_pitch) ;
        }

        guance_leso=0.03*disturb_feed_forward;
        // //------------------------------------------------------

        //gimbal_motor->dm_normal_motor.Control_Torque+=guance_leso;

        //输出限幅
        gimbal_motor->dm_normal_motor.Control_Torque = Math_Constrain(&gimbal_motor->dm_normal_motor.Control_Torque, -4.0f, 4.0f);

        //gimbal_motor->dm_normal_motor.Control_Torque = gimbal_motor->dm_normal_motor.Feedforward_Torque;
        //gimbal_motor->dm_normal_motor.Control_Torque = Math_Constrain(&gimbal_motor->dm_normal_motor.Control_Torque, -3.0f, 3.0f);
    }
}

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

    if (gimbal_motor == &gimbal_control.gimbal_yaw_j4310)
    {
    }
    else if (gimbal_motor == &gimbal_control.gimbal_pitch_j4340)
    {
        //
    }
}

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_INIT
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_init_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == &gimbal_control.gimbal_yaw_j4310)
    {
        gimbal_motor->Init_Angle_Pid.Target = gimbal_motor->dm_normal_motor.Target_Angle;
        gimbal_motor->Init_Angle_Pid.Now = gimbal_motor->relative_angle;
        PID_TIM_Adjust_PeriodElapsedCallback(&gimbal_motor->Init_Angle_Pid);

        gimbal_motor->dm_normal_motor.Target_Omega = gimbal_motor->Init_Angle_Pid.Out;

        gimbal_motor->Init_Omega_Pid.Target = gimbal_motor->dm_normal_motor.Target_Omega;
        gimbal_motor->Init_Omega_Pid.Now = gimbal_motor->dm_normal_motor.Rx_Data.Now_Omega;
        PID_TIM_Adjust_PeriodElapsedCallback(&gimbal_motor->Init_Omega_Pid);

        gimbal_motor->dm_normal_motor.Control_Torque = gimbal_motor->Init_Omega_Pid.Out * gimbal_motor->dm_normal_motor.polarity;

    }
    else if (gimbal_motor == &gimbal_control.gimbal_pitch_j4340)
    {

        gimbal_motor->Init_Angle_Pid.Target = gimbal_motor->dm_normal_motor.Target_Angle;
        gimbal_motor->Init_Angle_Pid.Now = -gimbal_motor->dm_normal_motor.INS_point->Pitch;
        PID_TIM_Adjust_PeriodElapsedCallback(&gimbal_motor->Init_Angle_Pid);

        gimbal_motor->dm_normal_motor.Target_Omega = gimbal_motor->Init_Angle_Pid.Out;

        gimbal_motor->Init_Omega_Pid.Target = gimbal_motor->dm_normal_motor.Target_Omega;
        gimbal_motor->Init_Omega_Pid.Now = gimbal_motor->dm_normal_motor.INS_point->Gyro[1];
        PID_TIM_Adjust_PeriodElapsedCallback(&gimbal_motor->Init_Omega_Pid);

        gimbal_motor->dm_normal_motor.Control_Torque = gimbal_motor->Init_Omega_Pid.Out * gimbal_motor->dm_normal_motor.polarity;

    }
}

static void gimbal_output_to_motor(gimbal_control_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
     // gimbal_motor->gimbal_yaw_j4310.dm_normal_motor.Control_Torque=0;
     // gimbal_motor->gimbal_pitch_j4340.dm_normal_motor.Control_Torque=0;

    Motor_DM_Normal_TIM_Send_PeriodElapsedCallback(&gimbal_motor->gimbal_yaw_j4310.dm_normal_motor);

    Motor_DM_Normal_TIM_Send_PeriodElapsedCallback(&gimbal_motor->gimbal_pitch_j4340.dm_normal_motor);

}

void Gimbal_Motor_Status_PeriodElapsedCallback(gimbal_control_t *gimbal_control)
{
    //电机在线状态检测
    gimbal_motor_keep_alive(gimbal_control);

    //电机堵转状态检测
    //Gimbal_FSM_TIM_Calculate_PeriodElapsedCallback(gimbal_control);

    //电机状态更新并作出控制
    gimbal_motor_status_influence_output(&gimbal_control->gimbal_yaw_j4310);
    gimbal_motor_status_influence_output(&gimbal_control->gimbal_pitch_j4340);
}

void gimbal_motor_keep_alive(gimbal_control_t *gimbal_control)
{
    static uint32_t Counter_KeepAlive = 0;

    if (Counter_KeepAlive++ > 100)
    {
        Counter_KeepAlive = 0;

        Motor_DM_Normal_TIM_Alive_PeriodElapsedCallback(&gimbal_control->gimbal_yaw_j4310.dm_normal_motor);
        Motor_DM_Normal_TIM_Alive_PeriodElapsedCallback(&gimbal_control->gimbal_pitch_j4340.dm_normal_motor);
    }

}

void gimbal_motor_status_influence_output(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

    if (gimbal_motor->dm_normal_motor.Motor_DM_Status == Motor_DM_Status_DISABLE)
    {
        gimbal_motor->motor_status = Motor_Status_OFFLINE;
    }
    else if (gimbal_motor->Motor_FSM.Now_Status == Progress )
    {
        gimbal_motor->motor_status = Motor_Status_HEAVY_STALL;
    }
    else if (gimbal_motor->dm_normal_motor.Motor_DM_Status == Motor_DM_Status_ENABLE)
    {
        gimbal_motor->motor_status = Motor_Status_NORMAL;
    }
    else
    {
        gimbal_motor->motor_status = Motor_Status_OFFLINE;
    }

    //电机状态更新并作出控制,只要不是正常的在线状态,就发送0力矩
    if (gimbal_motor->motor_status != Motor_Status_NORMAL)
    {
        Motor_DM_Normal_Zero_Force_PeriodElapsedCallback(&gimbal_motor->dm_normal_motor);
    }

}

void gimbal_rc_ctrl_disable(void)
{
    //机器人行动标志位重置成0
    gimbal_control.gimbal_move_flag=0;

    gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Target_Angle = gimbal_control.gimbal_yaw_j4310.dm_normal_motor.INS_point->Yaw;
    gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Target_Angle = -gimbal_control.gimbal_pitch_j4340.dm_normal_motor.INS_point->Pitch;

    //清除积分项并发送0力矩
    Motor_DM_Normal_Zero_Force_PeriodElapsedCallback(&gimbal_control.gimbal_yaw_j4310.dm_normal_motor);
    Motor_DM_Normal_Zero_Force_PeriodElapsedCallback(&gimbal_control.gimbal_pitch_j4340.dm_normal_motor);

}

void FSM_TIM_Calculate_PeriodElapsedCallback_Avoid_Yaw(FSM_t *FSM)
{
    FSM->Status[FSM->Now_Status].Count_Time++;

    // 自己接着编写状态转移函数
    switch (FSM->Now_Status)
    {
    case Normal:
    {
        if(fabsf(gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Rx_Data.Now_Torque) > 10.0f)
        {
            FSM_Set_Status(FSM, Suspect);
        }
        break;
    }
    case Suspect:
    {
        static int mod = 0;
        mod++;

        if( fabsf(gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Rx_Data.Now_Torque) < 1.0f)
        {
            gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Target_Angle = gimbal_control.gimbal_INS_point->Yaw;
            mod = 0;
            FSM_Set_Status(FSM, Normal);
        }

        if(mod > 50)
        {
            mod = 0;
            FSM_Set_Status(FSM, Confirm);
        }
        break;
    }
    case Confirm:
    {
        FSM_Set_Status(FSM, Progress);
        break;
    }
    case Progress:
    {
        static int mod = 0;
        mod++;

        if( mod > 1000 && fabsf(gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Rx_Data.Now_Torque) < 1.0f)
        {
            gimbal_control.gimbal_yaw_j4310.dm_normal_motor.Target_Angle = gimbal_control.gimbal_INS_point->Yaw;
            mod = 0;
            FSM_Set_Status(FSM, Normal);
        }
        break;
    }
    }
}

void FSM_TIM_Calculate_PeriodElapsedCallback_Avoid_Pitch(FSM_t *FSM)
{
    FSM->Status[FSM->Now_Status].Count_Time++;

    // 自己接着编写状态转移函数
    switch (FSM->Now_Status)
    {
    case Normal:
    {
        if(fabsf(gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Rx_Data.Now_Torque) > 4.0f)
        {
            FSM_Set_Status(FSM, Suspect);
        }
        break;
    }
    case Suspect:
    {
        static int mod = 0;
        mod++;

        if( fabsf(gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Rx_Data.Now_Torque) < 1.5f)
        {
            gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Target_Angle = -gimbal_control.gimbal_INS_point->Pitch;
            mod = 0;
            FSM_Set_Status(FSM, Normal);
        }

        if(mod > 50)
        {
            mod = 0;
            FSM_Set_Status(FSM, Confirm);
        }
        break;
    }
    case Confirm:
    {
        FSM_Set_Status(FSM, Progress);
        break;
    }
    case Progress:
    {
        static int mod = 0;
        mod++;

        if( mod > 1000 && fabsf(gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Rx_Data.Now_Torque) < 1.5f)
        {
            gimbal_control.gimbal_pitch_j4340.dm_normal_motor.Target_Angle = -gimbal_control.gimbal_INS_point->Pitch;
            mod = 0;
            FSM_Set_Status(FSM, Normal);
        }

        break;
    }
    }
}

void FSM_Gimbal_Init_TIM_Calculate_PeriodElapsedCallback(FSM_t *FSM)
{
    FSM->Status[FSM->Now_Status].Count_Time++;

    // // 自己接着编写状态转移函数
    switch (FSM->Now_Status)
    {
    case INIT_START:
        FSM_Set_Status(FSM, INIT_ING);
        break;
    case INIT_ING:
        if ( (gimbal_control.gimbal_yaw_j4310.relative_angle > -0.05f &&
            gimbal_control.gimbal_yaw_j4310.relative_angle < 0.05f) || (gimbal_control.gimbal_yaw_j4310.relative_angle>3.0 || gimbal_control.gimbal_yaw_j4310.relative_angle<-3.0) )
        {
            FSM_Set_Status(FSM, INIT_FINISH);
        }
        break;

    case INIT_FINISH:
        if ((gimbal_control.gimbal_yaw_j4310.gimbal_motor_mode != GIMBAL_MOTOR_ZERO_FORCE_CONTROL &&
            gimbal_control.gimbal_yaw_j4310.last_gimbal_motor_mode == GIMBAL_MOTOR_ZERO_FORCE_CONTROL) ||
            (gimbal_control.gimbal_pitch_j4340.gimbal_motor_mode != GIMBAL_MOTOR_ZERO_FORCE_CONTROL &&
            gimbal_control.gimbal_pitch_j4340.last_gimbal_motor_mode == GIMBAL_MOTOR_ZERO_FORCE_CONTROL)||
            gimbal_control.gimbal_INS_point->Pitch > PI/2 ||
            gimbal_control.gimbal_INS_point->Pitch < -PI/2)
        {
            FSM_Set_Status(FSM, NEED_INIT);
        }
        break;

    case NEED_INIT:
        if (gimbal_control.gimbal_INS_point->Pitch < PI/2 &&
            gimbal_control.gimbal_INS_point->Pitch > -PI/2)
        {
            FSM_Set_Status(FSM, INIT_START);
        }
        break;

    default:

        break;
    }

    if (FSM->Now_Status == INIT_FINISH)
    {
        gimbal_control.init_flag = 1;
    }
    else
    {
        gimbal_control.init_flag = 0;
    }

}

void Gimbal_FSM_TIM_Calculate_PeriodElapsedCallback(gimbal_control_t *gimbal_fsm)
{
    // //电机堵转检测
    FSM_TIM_Calculate_PeriodElapsedCallback_Avoid_Yaw(&gimbal_fsm->gimbal_yaw_j4310.Motor_FSM);
    FSM_TIM_Calculate_PeriodElapsedCallback_Avoid_Pitch(&gimbal_fsm->gimbal_pitch_j4340.Motor_FSM);
}

// 初始化
void GravComp_Init(Gravity_Comp_t *gravity_comp, float K, float m, float g, float arm_len, float phase)
{
    gravity_comp->K = K;
    gravity_comp->m = m;
    gravity_comp->g = g;
    gravity_comp->arm_len = arm_len;
    gravity_comp->phase = phase;

}

// 计算补偿力矩 (Nm)
float GravComp_Calc(Gravity_Comp_t *gravity_comp, float pitch_angle)
{
    return (gravity_comp->m * gravity_comp->g * gravity_comp->arm_len * cosf(pitch_angle - gravity_comp->phase)) * gravity_comp->K;
}


