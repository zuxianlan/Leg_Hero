#include "shoot_task.h"
#include "gimbal_behaviour.h"

//函数声明---------------------------------------------------------------------
static void shoot_init(shoot_control_t *init);
static void shoot_set_mode(shoot_control_t *set_mode);
extern void shoot_motor_mode_set(shoot_control_t *shoot_mode_set);
static void shoot_mode_change_transit(shoot_control_t *mode_change);
static void shoot_feedback_update(shoot_control_t *feedback_update);
static void shoot_set_control(shoot_control_t *set_control);
static void shoot_control_loop(shoot_control_t *control_loop);
static void shoot_output_to_motor(shoot_control_t *shoot_motor);

void shoot_fric_motor_feedforward_torque_loop(shoot_motor_t *shoot_motor);
static void bullet_fire_count_update(shoot_control_t *shoot_update);
void fire_delay_PeriodElapsedCallback(shoot_control_t *shoot);
static void remain_fire_count_calc(shoot_control_t *remain_fire_control);

void Shoot_Motor_Status_PeriodElapsedCallback(shoot_control_t *shoot_control);
void shoot_motor_keep_alive(shoot_control_t *shoot_control);
void shoot_dm_motor_status_influence_output(shoot_motor_t *shoot_motor);
void shoot_dj_motor_status_influence_output(shoot_motor_t *shoot_motor);
void shoot_rc_ctrl_disable(void);

void Shoot_FSM_TIM_Calculate_PeriodElapsedCallback(shoot_control_t *shoot_fsm);
void FSM_TIM_Calculate_PeriodElapsedCallback_Avoid_Trigger_Angle_Mode(shoot_motor_t *shoot_motor);
void FSM_TIM_Calculate_PeriodElapsedCallback_Avoid_Trigger_Speed_Mode(shoot_motor_t *shoot_motor);
void FSM_TIM_Calculate_PeriodElapsedCallback_Avoid_Fric(shoot_motor_t *shoot_motor);
//------------------------------------------------------------------------------


// 摩擦轮速度设置(角速度)
#define shoot_fric(left_speed,right_speed)                                     \
do                                                                             \
{                                                                              \
    shoot_control.fric_left_3508.C620_motor.Target_RPM = left_speed;           \
    shoot_control.fric_right_3508.C620_motor.Target_RPM = -right_speed;        \
} while (0)
//------------------------------------------------------------------------------



//热门重要参数宏定义----------------------------------------------------------------
//左摩擦轮转速
#define FRIC_LEFT_RPM_SPEED 3600////3600
//右摩擦轮转速
#define FRIC_RIGHT_RPM_SPEED 3600//3600

// 摩擦轮转速检测：高→低→高 完整循环计一次计数
#define FRIC_SPEED_THRESHOLD 3000    // 转速阈值 (RPM)
#define FIRE_DEBOUNCE_TICKS  1     // 去抖动时间 (ms)，过滤瞬时干扰
#define MIN_FIRE_INTERVAL     20    // 最小发射间隔 (ms)，防止重复计数

//------------------------------------------------------------------------------



//重要变量------------------------------------------------------------------------
bool_t mouse_huibo_flag = 0;//手动开启卡弹回拨标志位
bool_t last_mouse_huibo_flag = 0;//手动开启卡弹回拨标志位

bool_t mouse_gongdan_flag = 0;//手动开启卡弹回拨标志位
bool_t last_mouse_gongdan_flag = 0;//手动开启卡弹回拨标志位

bool_t  R_Fric = 0;
shoot_control_t shoot_control;

enum { WAIT_HIGH, ARMED, DEBOUNCING, CONFIRMED };
static uint8_t bullet_state = WAIT_HIGH;
static TickType_t state_enter_tick = 0;
static bool_t was_enabled = false;
//------------------------------------------------------------------------------


/**
 * @brief          射击任务，间隔 GIMBAL_CONTROL_TIME 1ms
 * @param[in]      pvParameters: 空
 * @retval         none
 */
void Shoot_Task(void const *pvParameters)
{
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    //射击初始化
    shoot_init(&shoot_control);
    while (1)
    {
        // 获取系统时间
        shoot_control.SHOOT_xTickCount = xTaskGetTickCount();
        //设置发射模式
        shoot_set_mode(&shoot_control);
        // 模式切换数据过渡,主要PID清除，防止数据积累引发电机反转
        shoot_mode_change_transit(&shoot_control);
        //发射数据更新
        shoot_feedback_update(&shoot_control);
        // 发射控制量设置
        shoot_set_control(&shoot_control);
        //射击控制循环
        shoot_control_loop(&shoot_control);

        //发射机构电机状态检测回调函数
        Shoot_Motor_Status_PeriodElapsedCallback(&shoot_control);

        // //判断遥控器是否在线
         if (RC_Control.shoot_control_status==No_Control)
         {
             // // 遥控器断开连接，禁用所有电机
             shoot_rc_ctrl_disable();
         }

        //输出发送到电机
        shoot_output_to_motor(&shoot_control);

        vTaskDelay(SHOOT_TASK_DELAY_TIME);
    }
}
/**
 * @brief          射击初始化，初始化PID，遥控器指针，电机指针
 * @param[in]      void
 * @retval         返回空
 */
static void shoot_init(shoot_control_t *init)
{
    // 获取遥控器指针
    init->shoot_dr16_processed_data = get_dr16_processed_data_point();
    init->shoot_vt13_processed_data = get_vt13_processed_data_point();
    init->shoot_fusi_rc_processed_data = get_fusi_rc_processed_data_point();

    // 获取视觉控制指针
    init->shoot_vision_control = get_vision_shoot_point();

 //电机初始化---------------------------------------------------------------------
    //拨弹盘电机初始化
    Motor_DM_Normal_Init(&init->shoot_trigger_j4310.dm_normal_motor, &hfdcan3, 0x77, 0x07, Motor_DM_Control_Method_NORMAL_MIT, PI, 30.0f, 10.0f, 3.0f,1);
    //拨弹盘电机PID控制模式初始化
    init->shoot_trigger_j4310.dm_normal_motor.motor_pid_control_mode=angle_pid_control;

    //左摩擦轮电机初始化
    Motor_C620_Init(&init->fric_left_3508.C620_motor, &hfdcan1, CAN1_Left_Fric_Motor_ID_0x201, Motor_DJI_Control_Method_RPM, Motor_DJI_Power_Limit_Status_DISABLE, 30.0f);
    init->fric_left_3508.C620_motor.Gearbox_Rate = 1.0f;
    //右摩擦轮电机初始化
    Motor_C620_Init(&init->fric_right_3508.C620_motor, &hfdcan1, CAN1_Right_Fric_Motor_ID_0x202, Motor_DJI_Control_Method_RPM, Motor_DJI_Power_Limit_Status_DISABLE, 30.0f);
    init->fric_right_3508.C620_motor.Gearbox_Rate = 1.0f;

 //-----------------------------------------------------------------------------


 //PID初始化---------------------------------------------------------------------
    //拨弹盘电机 PID 初始化
    PID_Init(&init->shoot_trigger_j4310.dm_normal_motor.Angle_PID, Trigger_MOTOR_4310_Angle_PID_KP, Trigger_MOTOR_4310_Angle_PID_KI, Trigger_MOTOR_4310_Angle_PID_KD, Trigger_MOTOR_4310_Angle_PID_KF, Trigger_MOTOR_4310_Angle_PID_MAX_IOUT, Trigger_MOTOR_4310_Angle_PID_MAX_OUT, 0.01f, Trigger_MOTOR_4310_Angle_PID_DEAD_ZONE, Trigger_MOTOR_4310_Angle_I_Variable_Speed_A, Trigger_MOTOR_4310_Angle_I_Variable_Speed_B, Trigger_MOTOR_4310_Angle_I_Separate_Threshold, PID_D_First_DISABLE);
    PID_Init(&init->shoot_trigger_j4310.dm_normal_motor.Omega_PID, Trigger_MOTOR_4310_Speed_PID_KP, Trigger_MOTOR_4310_Speed_PID_KI, Trigger_MOTOR_4310_Speed_PID_KD, Trigger_MOTOR_4310_Speed_PID_KF, Trigger_MOTOR_4310_Speed_PID_MAX_IOUT, Trigger_MOTOR_4310_Speed_PID_MAX_OUT, 0.01f, Trigger_MOTOR_4310_Speed_PID_DEAD_ZONE, Trigger_MOTOR_4310_Speed_I_Variable_Speed_A, Trigger_MOTOR_4310_Speed_I_Variable_Speed_B, Trigger_MOTOR_4310_Speed_I_Separate_Threshold, PID_D_First_DISABLE);
    //拨弹盘电机单纯角速度环
    PID_Init(&init->shoot_trigger_j4310.dm_normal_motor.Only_Omega_PID, Trigger_MOTOR_4310_Only_Speed_PID_KP, Trigger_MOTOR_4310_Only_Speed_PID_KI, Trigger_MOTOR_4310_Only_Speed_PID_KD, Trigger_MOTOR_4310_Only_Speed_PID_KF, Trigger_MOTOR_4310_Only_Speed_PID_MAX_IOUT, Trigger_MOTOR_4310_Only_Speed_PID_MAX_OUT, 0.01f, Trigger_MOTOR_4310_Only_Speed_PID_DEAD_ZONE, Trigger_MOTOR_4310_Only_Speed_I_Variable_Speed_A, Trigger_MOTOR_4310_Only_Speed_I_Variable_Speed_B, Trigger_MOTOR_4310_Only_Speed_I_Separate_Threshold, PID_D_First_DISABLE);

    //左摩擦轮 rpm PID初始化
    PID_Init(&init->fric_left_3508.C620_motor.PID_RPM, Fric_Left_MOTOR_Speed_PID_KP, Fric_Left_MOTOR_Speed_PID_KI, Fric_Left_MOTOR_Speed_PID_KD, Fric_Left_MOTOR_Speed_PID_KF, Fric_Left_MOTOR_Speed_PID_MAX_IOUT, Fric_Left_MOTOR_Speed_PID_MAX_OUT, 0.01f, Fric_Left_MOTOR_Speed_PID_DEAD_ZONE, Fric_Left_MOTOR_Speed_I_Variable_Speed_A, Fric_Left_MOTOR_Speed_I_Variable_Speed_B, Fric_Left_MOTOR_Speed_I_Separate_Threshold, PID_D_First_DISABLE);
    //右擦轮 rpm PID初始化
    PID_Init(&init->fric_right_3508.C620_motor.PID_RPM, Fric_Right_MOTOR_Speed_PID_KP, Fric_Right_MOTOR_Speed_PID_KI, Fric_Right_MOTOR_Speed_PID_KD, Fric_Right_MOTOR_Speed_PID_KF, Fric_Right_MOTOR_Speed_PID_MAX_IOUT, Fric_Right_MOTOR_Speed_PID_MAX_OUT, 0.01f, Fric_Right_MOTOR_Speed_PID_DEAD_ZONE, Fric_Right_MOTOR_Speed_I_Variable_Speed_A, Fric_Right_MOTOR_Speed_I_Variable_Speed_B, Fric_Right_MOTOR_Speed_I_Separate_Threshold, PID_D_First_DISABLE);

    // 更新数据
    //摩擦轮电机rpm设定值
    init->fric_left_rpm_set = FRIC_LEFT_RPM_SPEED;
    init->fric_right_rpm_set = FRIC_RIGHT_RPM_SPEED;

    init->allow_fire_flag=0;
    //已经打出弹丸的数量初始化
    init->already_bullet_counts=0;
    init->last_already_bullet_counts=0;
    //发弹时间初始化
    init->bullet_fire_count_time=init->SHOOT_xTickCount;

    //初始化发射机构电机状态机
    FSM_Init(&init->shoot_trigger_j4310.Motor_FSM, 4, Normal);
    FSM_Init(&init->fric_left_3508.Motor_FSM, 4, Normal);
    FSM_Init(&init->fric_right_3508.Motor_FSM, 4, Normal);

}

/**
 * @brief          射击模式设置
 * @param[in]      void
 * @retval         返回无
 */
static void shoot_set_mode(shoot_control_t *set_mode)
{
    if (set_mode == NULL)
    {
        return;
    }
    shoot_motor_mode_set(set_mode);
}

static void shoot_mode_change_transit(shoot_control_t *mode_change)
{
    if (mode_change == NULL)
    {
        return;
    }

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
    // 拨弹盘电机状态机切换并保存数据
    if (mode_change->last_trigger_mode != SHOOT_MOTOR_DISABLE && mode_change->trigger_mode == SHOOT_MOTOR_DISABLE)
    {
        shoot_control.shoot_trigger_j4310.dm_normal_motor.motor_pid_control_mode=angle_pid_control;
        shoot_control.shoot_trigger_j4310.dm_normal_motor.Target_Angle=shoot_control.shoot_trigger_j4310.dm_normal_motor.Rx_Data.Now_Angle;
    }
    else if (mode_change->last_trigger_mode != SHOOT_MOTOR_ENABLE && mode_change->trigger_mode == SHOOT_MOTOR_ENABLE)
    {
        shoot_control.shoot_trigger_j4310.dm_normal_motor.motor_pid_control_mode=angle_pid_control;
        shoot_control.shoot_trigger_j4310.dm_normal_motor.Target_Angle=shoot_control.shoot_trigger_j4310.dm_normal_motor.Rx_Data.Now_Angle;
    }
    else if (mode_change->last_trigger_mode != SHOOT_MOTOR_DEBUG_ && mode_change->trigger_mode == SHOOT_MOTOR_DEBUG_)
    {
        shoot_control.shoot_trigger_j4310.dm_normal_motor.motor_pid_control_mode=angle_pid_control;
        shoot_control.shoot_trigger_j4310.dm_normal_motor.Target_Angle=shoot_control.shoot_trigger_j4310.dm_normal_motor.Rx_Data.Now_Angle;
    }
    else if (mode_change->last_trigger_mode != SHOOT_MOTOR_RC_GONGDAN && mode_change->trigger_mode == SHOOT_MOTOR_RC_GONGDAN)
    {
        shoot_control.shoot_trigger_j4310.dm_normal_motor.motor_pid_control_mode=only_omega_pid_control;
        shoot_control.shoot_trigger_j4310.dm_normal_motor.Target_Angle=shoot_control.shoot_trigger_j4310.dm_normal_motor.Rx_Data.Now_Angle;
    }
    else if (mode_change->last_trigger_mode != SHOOT_MOTOR_RC_HUIBO && mode_change->trigger_mode == SHOOT_MOTOR_RC_HUIBO)
    {
        shoot_control.shoot_trigger_j4310.dm_normal_motor.motor_pid_control_mode=angle_pid_control;
        shoot_control.shoot_trigger_j4310.dm_normal_motor.Target_Angle=shoot_control.shoot_trigger_j4310.dm_normal_motor.Rx_Data.Now_Angle;
    }
    else if (mode_change->last_trigger_mode != SHOOT_MOTOR_MOUSE_GONGDAN && mode_change->trigger_mode == SHOOT_MOTOR_MOUSE_GONGDAN)
    {
        shoot_control.shoot_trigger_j4310.dm_normal_motor.motor_pid_control_mode=only_omega_pid_control;
        shoot_control.shoot_trigger_j4310.dm_normal_motor.Target_Angle=shoot_control.shoot_trigger_j4310.dm_normal_motor.Rx_Data.Now_Angle;
    }
    else if (mode_change->last_trigger_mode != SHOOT_MOTOR_MOUSE_HUIBO && mode_change->trigger_mode == SHOOT_MOTOR_MOUSE_HUIBO)
    {
        shoot_control.shoot_trigger_j4310.dm_normal_motor.motor_pid_control_mode=angle_pid_control;
        shoot_control.shoot_trigger_j4310.dm_normal_motor.Target_Angle=shoot_control.shoot_trigger_j4310.dm_normal_motor.Rx_Data.Now_Angle;
    }

    mode_change->last_trigger_mode = mode_change->trigger_mode;


    // 摩擦轮电机状态机切换并保存数据
    if (mode_change->last_fric_mode != SHOOT_MOTOR_DISABLE && mode_change->fric_mode == SHOOT_MOTOR_DISABLE)
    {

    }
    else if (mode_change->last_fric_mode != SHOOT_MOTOR_ENABLE && mode_change->fric_mode == SHOOT_MOTOR_ENABLE)
    {

    }

    mode_change->last_fric_mode = mode_change->fric_mode;

}


static void bullet_fire_count_update(shoot_control_t *shoot_update)
{
    if (shoot_update->fric_mode == SHOOT_MOTOR_ENABLE)
    {
        bool_t speed_low = (shoot_update->fric_speed_average < FRIC_SPEED_THRESHOLD);

        // 刚进入 ENABLE 模式时重置状态，避免摩擦轮启动过程被误计
        if (!was_enabled)
        {
            bullet_state = WAIT_HIGH;
        }

        switch (bullet_state)
        {
        case WAIT_HIGH:
            // 等待摩擦轮转速上升到阈值以上（启动保护）
            if (!speed_low)
            {
                bullet_state = ARMED;
            }
            break;

        case ARMED:
            // 转速正常，等待子弹通过导致转速下降
            if (speed_low)
            {
                bullet_state = DEBOUNCING;
                state_enter_tick = shoot_update->SHOOT_xTickCount;
            }
            break;

        case DEBOUNCING:
            // 转速低于阈值，去抖动确认
            if (!speed_low)
            {
                // 转速提前恢复，判定为噪声，回到 ARMED
                bullet_state = ARMED;
            }
            else if (shoot_update->SHOOT_xTickCount - state_enter_tick >= FIRE_DEBOUNCE_TICKS)
            {
                // 转速持续低于阈值，确认子弹通过
                bullet_state = CONFIRMED;
            }
            break;

        case CONFIRMED:
            // 已确认转速下降，等待转速恢复到阈值以上
            if (!speed_low)
            {
                // 完成 高→低→高 完整循环，计数+1
                if (shoot_update->SHOOT_xTickCount - shoot_update->bullet_fire_count_time >= MIN_FIRE_INTERVAL)
                {
                    shoot_update->already_bullet_counts++;
                    shoot_update->bullet_fire_count_time = shoot_update->SHOOT_xTickCount;
                }
                bullet_state = ARMED;
            }
            break;
        }

        was_enabled = true;
    }
    else
    {
        was_enabled = false;
    }
}

/**
 *
 * @brief          射击数据更新
 * @param[in]      void
 * @retval         void
 */
static void shoot_feedback_update(shoot_control_t *feedback_update)
{
    // 摩擦轮转速设定平均值
    feedback_update->fric_speed_set_average=( fabs(feedback_update->fric_left_rpm_set)+fabs(feedback_update->fric_right_rpm_set) )/2;
    // 摩擦轮转速平均值
    feedback_update->fric_speed_average=( fabs(feedback_update->fric_left_3508.C620_motor.Rx_Origin.Omega_Reverse)+fabs(feedback_update->fric_right_3508.C620_motor.Rx_Origin.Omega_Reverse) )/2;

    //弹丸计数更新
    bullet_fire_count_update(feedback_update);

    //根据摩擦轮转速判断开没开摩擦轮
    if (feedback_update->fric_speed_average > 2000)
    {
        feedback_update->fric_open_flag = 1;
    }
    else
    {
        feedback_update->fric_open_flag = 0;
    }

    //发出一发弹丸后，强制延迟一秒后允许发弹
    fire_delay_PeriodElapsedCallback(feedback_update);

    //根据热量更新可发弹数量
    remain_fire_count_calc(feedback_update);

}

/**
 * @brief          由热量计算出剩余可发弹数量
 * @param[out]     remain_fire_control:"shoot_control_t"变量指针.
 * @retval         none
 */
static void remain_fire_count_calc(shoot_control_t *remain_fire_control)
{
    static uint16_t limit=0;
    static uint16_t heat=0;

    // 计算剩余热量
    limit = class_referee.Referee_Rx_Data.Robot_Status.shooter_heat_limit;
    heat = class_referee.Referee_Rx_Data.Robot_Power_Heat.shooter_42mm_heat;

    if (heat+100 > limit)
    {
        remain_fire_control->heat_allow_fire_flag=0;
    }
    else
    {
        remain_fire_control->heat_allow_fire_flag=1;
    }

}

/**
 * @brief          手动调整摩擦轮转速
 * @param[out]     wheel_speed_control:"shoot_control_t"变量指针.
 * @retval         none
 */
static void adjust_fric_rpm(shoot_control_t *fric_rpm_control)
{
    static uint16_t KEY_CTRL_C = 0;
    static uint16_t Last_KEY_CTRL_C = 0;
    static uint16_t KEY_CTRL_X = 0;
    static uint16_t Last_KEY_CTRL_X = 0;

    KEY_CTRL_C = (RC_Control.key_mouse.Key & RC_KEY_PRESSED_OFFSET_C) && (RC_Control.key_mouse.Key & RC_KEY_PRESSED_OFFSET_CTRL);
    KEY_CTRL_X = (RC_Control.key_mouse.Key & RC_KEY_PRESSED_OFFSET_X) && (RC_Control.key_mouse.Key & RC_KEY_PRESSED_OFFSET_CTRL);

    int16_t fric_rpm_add = 0;

    //ctrl+c 摩擦轮增速
    if (!Last_KEY_CTRL_C && KEY_CTRL_C)
    {
        fric_rpm_add=20;
    }
    //ctrl+x 摩擦轮减速
    if (!Last_KEY_CTRL_X && KEY_CTRL_X)
    {
        fric_rpm_add=-20;
    }

    // 运算后赋值增量
    fric_rpm_control->fric_left_rpm_set += fric_rpm_add;
    fric_rpm_control->fric_right_rpm_set += fric_rpm_add;

    Last_KEY_CTRL_C = (RC_Control.key_mouse.Key & RC_KEY_PRESSED_OFFSET_C) && (RC_Control.key_mouse.Key & RC_KEY_PRESSED_OFFSET_CTRL);
    Last_KEY_CTRL_X = (RC_Control.key_mouse.Key & RC_KEY_PRESSED_OFFSET_X) && (RC_Control.key_mouse.Key & RC_KEY_PRESSED_OFFSET_CTRL);
}

/**
 * @brief          发射控制量设置
 * @param[in]      void
 * @retval         void
 */
static void shoot_set_control(shoot_control_t *set_control)
{
    if (set_control == NULL)
    {
        return;
    }

// 摩擦轮控制量设置----------------------------------------------------------------
    adjust_fric_rpm(set_control);

    if (set_control->fric_mode == SHOOT_MOTOR_DISABLE)
    {
        shoot_fric(0,0);
    }
    else if (set_control->fric_mode == SHOOT_MOTOR_ENABLE )
    {
        shoot_fric(set_control->fric_left_rpm_set,set_control->fric_right_rpm_set);
        //shoot_fric(0,0);
    }
    else
    {
        shoot_fric(0,0);
    }

//------------------------------------------------------------------------------


// 拨弹盘电机控制量设置-------------------------------------------------------------

    if (set_control->trigger_mode == SHOOT_MOTOR_DISABLE)
    {
        shoot_control.shoot_trigger_j4310.dm_normal_motor.Target_Angle=shoot_control.shoot_trigger_j4310.dm_normal_motor.Rx_Data.Now_Angle;
    }
    else if (set_control->trigger_mode == SHOOT_MOTOR_ENABLE)
    {
        if (RC_Control.rc_mode==vt13_mode)
        {
            Get_vt13_shoot_flag();
        }
        else if (RC_Control.rc_mode==dt7_mode)
        {
            Get_dt7_shoot_flag();
        }
        else if (RC_Control.rc_mode==fusi_mode)
        {
            Get_fusi_shoot_flag();
        }
        else
        {
            set_control->shoot_flag=0;
        }

        //发弹逻辑
        if (set_control->last_shoot_flag == 0 && set_control->shoot_flag == 1 )
        {
            set_control->shoot_trigger_j4310.dm_normal_motor.Target_Angle -= 2*PI/3;

        }

        set_control->last_shoot_flag = set_control->shoot_flag;


    }


    //调试模式--------------------------------------------------
    else if (set_control->trigger_mode == SHOOT_MOTOR_DEBUG_)
    {
        if (RC_Control.rc_mode==vt13_mode)
        {
            Get_vt13_rc_huibo_flag();
        }
        else if (RC_Control.rc_mode==dt7_mode)
        {
            Get_dt7_rc_huibo_flag();
        }
        else if (RC_Control.rc_mode==fusi_mode)
        {
            Get_fusi_rc_huibo_flag();
        }
        else
        {
            set_control->rc_huibo_flag=0;
        }

        //发弹逻辑
        if (set_control->last_rc_huibo_flag == 0 && set_control->rc_huibo_flag == 1 )
        {
            set_control->shoot_trigger_j4310.dm_normal_motor.Target_Angle -= 2*PI/3;

        }

        set_control->last_rc_huibo_flag = set_control->rc_huibo_flag;

    }
//单遥控器可控部分-----------------------------------------------------
    else if (set_control->trigger_mode == SHOOT_MOTOR_RC_GONGDAN)
    {

        shoot_control.shoot_trigger_j4310.dm_normal_motor.Target_Omega = -1.0 * PI ;
    }
	else if (set_control->trigger_mode == SHOOT_MOTOR_RC_HUIBO)
	{
	    if (RC_Control.rc_mode==vt13_mode)
	    {
	        Get_vt13_rc_huibo_flag();
	    }
	    else if (RC_Control.rc_mode==dt7_mode)
	    {
	        Get_dt7_rc_huibo_flag();
	    }
	    else if (RC_Control.rc_mode==fusi_mode)
	    {
	        Get_fusi_rc_huibo_flag();
	    }
	    else
	    {
	        set_control->rc_huibo_flag=0;
	    }

	    //发弹逻辑
	    if (set_control->last_rc_huibo_flag == 0 && set_control->rc_huibo_flag == 1 )
	    {
	        set_control->shoot_trigger_j4310.dm_normal_motor.Target_Angle += 2*PI/3;

	    }

	    set_control->last_rc_huibo_flag = set_control->rc_huibo_flag;

	}
//单键鼠可控部分----------------------------------------------------
    else if (set_control->trigger_mode == SHOOT_MOTOR_MOUSE_GONGDAN)
	{
        shoot_control.shoot_trigger_j4310.dm_normal_motor.Target_Omega = -1.0*PI ;
	}
	else if (set_control->trigger_mode == SHOOT_MOTOR_MOUSE_HUIBO)
	{
	    if (last_mouse_huibo_flag==0 && mouse_huibo_flag==1)
	    {
	        set_control->shoot_trigger_j4310.dm_normal_motor.Target_Angle += 2*PI/3;
	        set_control->trigger_mouse_huibo_start_time=set_control->SHOOT_xTickCount;
	    }

	    if( set_control->SHOOT_xTickCount - set_control->trigger_mouse_huibo_start_time > 1000)
	    {
	        mouse_huibo_flag = !mouse_huibo_flag;
	    }

	    last_mouse_huibo_flag=mouse_huibo_flag;
	}

//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------

}


/**
 * @brief          拨弹轮循环
 * @param[in]      void
 * @retval         返回无
 */
static void shoot_control_loop(shoot_control_t *control_loop)
{
    //--------------------------------------------------------------------------
    //拨弹盘电机计算回调函数
    if ( control_loop->trigger_mode==SHOOT_MOTOR_DISABLE )
    {
        shoot_control.shoot_trigger_j4310.dm_normal_motor.Feedforward_Torque=0;
        shoot_control.shoot_trigger_j4310.dm_normal_motor.Control_Torque=0;
    }
    else
    {
        shoot_control.shoot_trigger_j4310.dm_normal_motor.Feedforward_Torque = 0;//(-0.005f);
        Trigger_Motor_DM_Normal_TIM_1ms_Calculate_PeriodElapsedCallback(&control_loop->shoot_trigger_j4310.dm_normal_motor);
    }


    //摩擦轮电机计算回调函数---------------------------------------------------------------------------------
    // shoot_fric_motor_feedforward_torque_loop(&control_loop->fric_left_3508);
    // shoot_fric_motor_feedforward_torque_loop(&control_loop->fric_right_3508);

    control_loop->fric_left_3508.C620_motor.Feedforward_Torque=0.0f;
    control_loop->fric_right_3508.C620_motor.Feedforward_Torque=0.0f;

    if (control_loop->fric_mode==SHOOT_MOTOR_DISABLE)
    {
        control_loop->fric_left_3508.C620_motor.Feedforward_Torque=0.0f;
        control_loop->fric_right_3508.C620_motor.Feedforward_Torque=0.0f;
    }

    Shoot_Fric_Motor_C620_TIM_Calculate_PeriodElapsedCallback(&control_loop->fric_left_3508.C620_motor);
    Shoot_Fric_Motor_C620_TIM_Calculate_PeriodElapsedCallback(&control_loop->fric_right_3508.C620_motor);
}

void shoot_fric_motor_feedforward_torque_loop(shoot_motor_t *shoot_motor)
{
    shoot_motor->C620_motor.Accel_Feedforward_Torque = shoot_motor->C620_motor.Rx_Data.Now_Acceleration * Accel_Feedforward_Coeff;
    shoot_motor->C620_motor.Diff_Feedforward_Torque  = (shoot_control.fric_speed_average-fabs(shoot_motor->C620_motor.Rx_Origin.Omega_Reverse)) * Diff_Feedforward_Coeff;

    if (shoot_motor == &shoot_control.fric_left_3508)
    {
        shoot_motor->C620_motor.Feedforward_Torque = shoot_motor->C620_motor.Accel_Feedforward_Torque + shoot_motor->C620_motor.Diff_Feedforward_Torque;
    }
    else if (shoot_motor == &shoot_control.fric_right_3508)
    {
        shoot_motor->C620_motor.Feedforward_Torque = shoot_motor->C620_motor.Accel_Feedforward_Torque - shoot_motor->C620_motor.Diff_Feedforward_Torque;
    }
}

static void shoot_output_to_motor(shoot_control_t *shoot_motor)
{
    if (shoot_motor == NULL)
    {
        return;
    }

    //发送拨弹盘电机数据
    //shoot_motor->shoot_trigger_j4310.dm_normal_motor.Control_Torque=0;

    Motor_DM_Normal_TIM_Send_PeriodElapsedCallback(&shoot_motor->shoot_trigger_j4310.dm_normal_motor);

    //can1 发送摩擦轮电机数据
    CAN_Send_Data(&hfdcan1, 0x200, CAN1_0x200_Tx_Data, 8);

}

void Shoot_Motor_Status_PeriodElapsedCallback(shoot_control_t *shoot_control)
{
    //电机在线状态检测
    shoot_motor_keep_alive(shoot_control);

    //电机堵转状态检测
    Shoot_FSM_TIM_Calculate_PeriodElapsedCallback(shoot_control);

    //电机状态更新并作出控制
    shoot_dm_motor_status_influence_output(&shoot_control->shoot_trigger_j4310);

    shoot_dj_motor_status_influence_output(&shoot_control->fric_left_3508);
    shoot_dj_motor_status_influence_output(&shoot_control->fric_right_3508);
}

void shoot_motor_keep_alive(shoot_control_t *shoot_control)
{
    static uint32_t Counter_KeepAlive = 0;

    if (Counter_KeepAlive++ > 100)
    {
        Counter_KeepAlive=0;
        //拨弹盘电机
        Motor_DM_Normal_TIM_Alive_PeriodElapsedCallback(&shoot_control->shoot_trigger_j4310.dm_normal_motor);

        //摩擦轮
        Motor_C620_TIM_Alive_PeriodElapsedCallback(&shoot_control->fric_left_3508.C620_motor);
        Motor_C620_TIM_Alive_PeriodElapsedCallback(&shoot_control->fric_right_3508.C620_motor);
    }
}

void shoot_dm_motor_status_influence_output(shoot_motor_t *shoot_motor)
{
    if (shoot_motor == NULL)
    {
        return;
    }

    if (shoot_motor->dm_normal_motor.Motor_DM_Status == Motor_DM_Status_DISABLE)
    {
        shoot_motor->motor_status = Motor_Status_OFFLINE;
    }
    else if (shoot_motor->Motor_FSM.Now_Status == Progress )
    {
        if (shoot_motor->dm_normal_motor.motor_pid_control_mode==angle_pid_control)
        {
            shoot_motor->motor_status = Motor_Status_HEAVY_STALL;
        }
        else if (shoot_motor->dm_normal_motor.motor_pid_control_mode==only_omega_pid_control)
        {
            shoot_motor->motor_status = Motor_Status_LIGHT_STALL;
        }
        else
        {
            shoot_motor->motor_status = Motor_Status_HEAVY_STALL;
        }
    }
    else if (shoot_motor->dm_normal_motor.Motor_DM_Status == Motor_DM_Status_ENABLE)
    {
        shoot_motor->motor_status = Motor_Status_NORMAL;
    }
    else
    {
        shoot_motor->motor_status = Motor_Status_OFFLINE;
    }

    // 电机状态更新并作出控制,只要不是正常的在线状态,就发送0力矩
    // if (shoot_motor->motor_status != Motor_Status_NORMAL)
    // {
    //     Motor_DM_Normal_Zero_Force_PeriodElapsedCallback(&shoot_motor->dm_normal_motor);
    // }

}

void shoot_dj_motor_status_influence_output(shoot_motor_t *shoot_motor)
{
    if (shoot_motor == NULL)
    {
        return;
    }

    if (shoot_motor->C620_motor.CAN_Motor_Status == CAN_Motor_Status_DISABLE)
    {
        shoot_motor->motor_status = Motor_Status_OFFLINE;
    }
    else if (shoot_motor->Motor_FSM.Now_Status == Progress )
    {
        shoot_motor->motor_status = Motor_Status_HEAVY_STALL;
    }
    else if (shoot_motor->C620_motor.CAN_Motor_Status == CAN_Motor_Status_ENABLE)
    {
        shoot_motor->motor_status = Motor_Status_NORMAL;
    }
    else
    {
        shoot_motor->motor_status = Motor_Status_OFFLINE;
    }

    // // 电机状态更新并作出控制,只要不是正常的在线状态,就发送0力矩
    // if (shoot_motor->motor_status != Motor_Status_NORMAL)
    // {
    //     Motor_C620_Zero_Force_PeriodElapsedCallback(&shoot_motor->C620_motor);
    // }

}

void shoot_rc_ctrl_disable(void)
{
    shoot_control.shoot_trigger_j4310.dm_normal_motor.Target_Angle = shoot_control.shoot_trigger_j4310.dm_normal_motor.Rx_Data.Now_Angle;

    //拨弹盘电机
    Motor_DM_Normal_Zero_Force_PeriodElapsedCallback(&shoot_control.shoot_trigger_j4310.dm_normal_motor);

    //摩擦轮
    Motor_C620_Zero_Force_PeriodElapsedCallback(&shoot_control.fric_left_3508.C620_motor);
    Motor_C620_Zero_Force_PeriodElapsedCallback(&shoot_control.fric_right_3508.C620_motor);
}

void FSM_TIM_Calculate_PeriodElapsedCallback_Avoid_Trigger_Angle_Mode(shoot_motor_t *shoot_motor)
{
    shoot_motor->Motor_FSM.Status[shoot_motor->Motor_FSM.Now_Status].Count_Time++;

    // 自己接着编写状态转移函数
    switch (shoot_motor->Motor_FSM.Now_Status)
    {
    case Normal:
        {
            if(fabsf(shoot_control.shoot_trigger_j4310.dm_normal_motor.Rx_Data.Now_Torque) > 8.5f)
            {
                FSM_Set_Status(&shoot_motor->Motor_FSM, Suspect);
            }
            break;
        }
    case Suspect:
        {
            static int mod = 0;
            mod++;

            if( fabsf(shoot_control.shoot_trigger_j4310.dm_normal_motor.Rx_Data.Now_Torque) < 2.0f )
            {
                shoot_control.shoot_trigger_j4310.dm_normal_motor.Target_Angle = shoot_control.shoot_trigger_j4310.dm_normal_motor.Rx_Data.Now_Angle;
                mod = 0;
                FSM_Set_Status(&shoot_motor->Motor_FSM, Normal);
            }

            if(mod > 1000)
            {
                mod = 0;
                FSM_Set_Status(&shoot_motor->Motor_FSM, Confirm);
            }
            break;
        }
    case Confirm:
        {
            FSM_Set_Status(&shoot_motor->Motor_FSM, Progress);
            break;
        }
    case Progress:
        {
            static int mod = 0;
            mod++;

            if(mod > 1000 && fabsf(shoot_control.shoot_trigger_j4310.dm_normal_motor.Rx_Data.Now_Torque) < 2.0f)
            {
                shoot_control.shoot_trigger_j4310.dm_normal_motor.Target_Angle = shoot_control.shoot_trigger_j4310.dm_normal_motor.Rx_Data.Now_Angle;
                mod = 0;
                FSM_Set_Status(&shoot_motor->Motor_FSM, Normal);
            }
            break;
        }
    }
}

void FSM_TIM_Calculate_PeriodElapsedCallback_Avoid_Trigger_Speed_Mode(shoot_motor_t *shoot_motor)
{
    shoot_motor->Motor_FSM.Status[shoot_motor->Motor_FSM.Now_Status].Count_Time++;

    // 自己接着编写状态转移函数
    switch (shoot_motor->Motor_FSM.Now_Status)
    {
    case Normal:
        {
            if( fabsf(shoot_control.shoot_trigger_j4310.dm_normal_motor.Rx_Data.Now_Torque) > 1.2f)
            {
                FSM_Set_Status(&shoot_motor->Motor_FSM, Suspect);
            }
            break;
        }
    case Suspect:
        {
            static int mod = 0;
            mod++;

            if( fabsf(shoot_control.shoot_trigger_j4310.dm_normal_motor.Rx_Data.Now_Torque) < 1.0f)
            {
                shoot_control.shoot_trigger_j4310.dm_normal_motor.Target_Angle = shoot_control.shoot_trigger_j4310.dm_normal_motor.Rx_Data.Now_Angle;
                mod = 0;
                FSM_Set_Status(&shoot_motor->Motor_FSM, Normal);
            }

            if(mod > 1500)
            {
                mod = 0;
                FSM_Set_Status(&shoot_motor->Motor_FSM, Confirm);
            }
            break;
        }
    case Confirm:
        {
            FSM_Set_Status(&shoot_motor->Motor_FSM, Progress);
            break;
        }
    case Progress:
        {
            static int mod = 0;
            mod++;

            if(mod > 1000 && fabsf(shoot_control.shoot_trigger_j4310.dm_normal_motor.Rx_Data.Now_Torque) < 1.0f)
            {
                shoot_control.shoot_trigger_j4310.dm_normal_motor.Target_Angle = shoot_control.shoot_trigger_j4310.dm_normal_motor.Rx_Data.Now_Angle;
                mod = 0;
                FSM_Set_Status(&shoot_motor->Motor_FSM, Normal);
            }
            break;
        }
    }
}

void FSM_TIM_Calculate_PeriodElapsedCallback_Avoid_Fric(shoot_motor_t *shoot_motor)
{
    shoot_motor->Motor_FSM.Status[shoot_motor->Motor_FSM.Now_Status].Count_Time++;

    // 自己接着编写状态转移函数
    switch (shoot_motor->Motor_FSM.Now_Status)
    {
    case Normal:
        {
            if( fabsf(shoot_motor->C620_motor.Rx_Data.Now_Current) > 5.7f
              )
            {
                FSM_Set_Status(&shoot_motor->Motor_FSM, Suspect);
            }
            break;
        }
    case Suspect:
        {
            static int mod = 0;
            mod++;

            if(  fabsf(shoot_motor->C620_motor.Rx_Data.Now_Current) < 2.0f
              )
            {
                mod = 0;
                FSM_Set_Status(&shoot_motor->Motor_FSM, Normal);
            }

            if(mod > 1000)
            {
                mod = 0;
                FSM_Set_Status(&shoot_motor->Motor_FSM, Confirm);
            }
            break;
        }
    case Confirm:
        {
            FSM_Set_Status(&shoot_motor->Motor_FSM, Progress);
            break;
        }
    case Progress:
        {
            static int mod = 0;
            mod++;

            if ( mod > 1000
                && fabsf(shoot_motor->C620_motor.Rx_Data.Now_Current) < 2.0f
               )
            {
                mod = 0;
                FSM_Set_Status(&shoot_motor->Motor_FSM, Normal);
            }
            break;
        }
    }
}

void Shoot_FSM_TIM_Calculate_PeriodElapsedCallback(shoot_control_t *shoot_fsm)
{
    // //电机堵转检测

    // //拨弹盘电机堵转检测
    if (shoot_control.shoot_trigger_j4310.dm_normal_motor.motor_pid_control_mode==angle_pid_control)
    {
        //拨弹盘电机角度模式监测
        FSM_TIM_Calculate_PeriodElapsedCallback_Avoid_Trigger_Angle_Mode(&shoot_fsm->shoot_trigger_j4310);
    }
    else if (shoot_control.shoot_trigger_j4310.dm_normal_motor.motor_pid_control_mode==only_omega_pid_control)
    {
        //拨弹盘电机速度模式监测
        FSM_TIM_Calculate_PeriodElapsedCallback_Avoid_Trigger_Speed_Mode(&shoot_fsm->shoot_trigger_j4310);
    }

    //摩擦轮电机堵转检测
    FSM_TIM_Calculate_PeriodElapsedCallback_Avoid_Fric(&shoot_fsm->fric_left_3508);
    FSM_TIM_Calculate_PeriodElapsedCallback_Avoid_Fric(&shoot_fsm->fric_right_3508);
}

void fire_delay_PeriodElapsedCallback(shoot_control_t *shoot)
{

    // if (shoot->fric_speed_average < 3000)
    // {
    //     shoot->fire_force_delay_start_time=shoot->SHOOT_xTickCount;
    // }
    //
    // if ( (shoot->SHOOT_xTickCount-shoot->fire_force_delay_start_time) >2500)
    // {
        shoot->heat_delay_allow_flag=1;
    // }
    // else
    // {
    //     shoot->heat_delay_allow_flag=0;
    // }

}
