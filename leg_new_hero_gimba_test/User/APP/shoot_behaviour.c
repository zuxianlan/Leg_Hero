#include "shoot_behaviour.h"
#include "RC_Task.h"

//函数声明--------------------------------------------------------------------
static void shoot_behaviour_set(shoot_control_t *shoot_mode_set);
//-----------------------------------------------------------------------------


//重要变量-----------------------------------------------------------------------
shoot_behaviour_e shoot_behaviour = SHOOT_DISABLE;
trigger_status_t trigger_status = TRIGGER_NORMAL;
//-----------------------------------------------------------------------------

/**
 * @brief          发射行为状态机设置.
 * @param[in]      shoot_mode_set: 发射数据指针
 * @retval         none
 */
static void shoot_behaviour_set(shoot_control_t *shoot_mode_set)
{
    if (shoot_mode_set == NULL)
    {
        return;
    }

	shoot_behaviour=Get_shoot_behaviour_mode();
}

/**
 * @brief          通过逻辑判断，赋值"shoot_behaviour_mode"成哪种模式
 * @param[in]      shoot_mode_set: 发射数据
 * @retval         none
 */
void shoot_motor_mode_set(shoot_control_t *shoot_mode_set)
{
    if (shoot_mode_set == NULL)
    {
        return;
    }
    // 发射行为状态机设置
    shoot_behaviour_set(shoot_mode_set);

    // 根据发射行为状态机设置电机状态机
    if (shoot_behaviour == SHOOT_DISABLE)
    {
        shoot_mode_set->fric_mode = SHOOT_MOTOR_DISABLE;
        shoot_mode_set->trigger_mode = SHOOT_MOTOR_DISABLE;
    }
    else if (shoot_behaviour == SHOOT_ENABLE)
    {
        shoot_mode_set->fric_mode = SHOOT_MOTOR_ENABLE;
        shoot_mode_set->trigger_mode = SHOOT_MOTOR_ENABLE;

        if ( RC_Control.shoot_control_status==Key_Mouse_Control )
        {
            if (R_Fric==0 || shoot_control.fric_open_flag==0)
            {
                shoot_mode_set->trigger_mode = SHOOT_MOTOR_DISABLE;
            }
        }
    }
    else if (shoot_behaviour == SHOOT_DEBUG_)
    {
        shoot_mode_set->fric_mode = SHOOT_MOTOR_DISABLE;
        shoot_mode_set->trigger_mode = SHOOT_MOTOR_DISABLE;
    }
//遥控器版本------------------------------------------------
	else if (shoot_behaviour == SHOOT_RC_GONGDAN)
	{
	    shoot_mode_set->fric_mode = SHOOT_MOTOR_ENABLE;
	    shoot_mode_set->trigger_mode = SHOOT_MOTOR_RC_GONGDAN;
	}
	else if (shoot_behaviour == SHOOT_RC_HUIBO)
	{
	    shoot_mode_set->fric_mode = SHOOT_MOTOR_DISABLE;
	    shoot_mode_set->trigger_mode = SHOOT_MOTOR_RC_HUIBO;
	}
////键鼠版本-----------------------------------------------
	else if (shoot_behaviour == SHOOT_MOUSE_GONGDAN)
	{
	    shoot_mode_set->fric_mode = SHOOT_MOTOR_ENABLE;
	    shoot_mode_set->trigger_mode = SHOOT_MOTOR_MOUSE_GONGDAN;
	}
	else if (shoot_behaviour == SHOOT_MOUSE_HUIBO)
	{
	    shoot_mode_set->fric_mode = SHOOT_MOTOR_DISABLE;
	    shoot_mode_set->trigger_mode = SHOOT_MOTOR_MOUSE_HUIBO;
	}
//----------------------------------------------------
    else
    {
    	shoot_mode_set->fric_mode = SHOOT_MOTOR_DISABLE;
    	shoot_mode_set->trigger_mode = SHOOT_MOTOR_DISABLE;
    }

}
