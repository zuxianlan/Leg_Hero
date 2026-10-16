/**
  ******************************************************************************
  * @file           : VMC&LQR_Calc.c
  * @author         : gagami
  * @brief          : None
  * @attention      : None
  * @date           : 2025/8/7
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "VMC&LQR_Calc.h"

#include <cmath>

#include "leg_position.h"
#include "leg_speed.h"
#include "leg_convert.h"


/* Define --------------------------------------------------------------------*/

/* Enum ----------------------------------------------------------------------*/

/* Variable && Struct --------------------------------------------------------*/

/* Function Declaration ------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/

/**
 * @brief 计算theta、dtheta，供LQR使用，同时得到腿长l0
 *
 * @param
 * @return
 */
void class_vmc_leg::VMC_Calc_1()
{
    static float pitch, d_pitch;
    pitch = -INS->pitch;
    d_pitch = -INS->gyro[0];

    // 通过解算得到腿长l0和phi0
    leg_position(phi1, phi4, &L0, &phi0);
    leg_speed(d_phi1, d_phi4, phi1, phi4, &d_L0, &d_phi0);

    // 计算出夹角alpha，再由此得到theta
    alpha = phi0 - std::numbers::pi / 2.0;

    //vmc->d_phi0 = (vmc->phi0 - vmc->last_phi0) / dt;

    //得到状态量1 2
    theta = alpha - pitch;
    d_theta = (d_phi0 - d_pitch);

    // 保存上一时刻状态
    last_phi0 = phi0;

    // 计算微分与加速度
    // vmc->d_L0 = (vmc->L0 - vmc->last_L0) / dt;
    dd_L0 = (d_L0 - last_d_L0) / dt;

    last_d_L0 = d_L0;
    last_L0 = L0;

    dd_theta = (d_theta - last_d_theta) / dt;
    last_d_theta = d_theta;
}

/**
 * @brief 通过LQR以及解算得到Tp，再经由关节转换得到电机力矩
 *
 * @param
 * @return
 */
void class_vmc_leg::VMC_Calc_2()
{
    leg_convert(F0, Tp, phi1, phi4, torque_set);
    float tmp = 45.0f;
    if(torque_set[0] > tmp)
    {
        torque_set[0] = tmp;
    }
    else if(torque_set[0] < -tmp)
    {
        torque_set[0] = -tmp;
    }

    if(torque_set[1] > tmp)
    {
        torque_set[1] = tmp;
    }
    else if(torque_set[1] < -tmp)
    {
        torque_set[1] = -tmp;
    }
}

/**
 * @brief 触地检测
 *
 * @param
 * @return
 */
uint8_t class_vmc_leg::ground_detection_L()
{

    // 静力平衡方程
    FN = -F0 * cosf(theta) - Tp * sinf(theta) / L0; // 静力平衡方程（忽略惯性力+重力项）, 即支撑力=虚拟力在竖直方向的分量+虚拟力矩在竖直方向的分量*sin(theta)/L0，用于判断竖直方向运动速度
    // 牛顿第二定律
    // vmc->FN = vmc->F0*arm_cos_f32(vmc->theta) + vmc->Tp*arm_sin_f32(vmc->theta)/vmc->L0 + 0.6f*(ins->MotionAccel_n[2] - vmc->dd_L0*arm_cos_f32(vmc->theta) + 2.0f*vmc->d_L0*vmc->d_theta*arm_sin_f32(vmc->theta) + vmc->L0*vmc->dd_theta*arm_sin_f32(vmc->theta) + vmc->L0*vmc->d_theta*vmc->d_theta*arm_cos_f32(vmc->theta));

    if (FN > 20)
    {
        //判定已触地
        return 1;
        
    }
    else
    {
 
        return 0;
        
    }
    return 0;
}

uint8_t class_vmc_leg::ground_detection_R()
{
 

    // 静力平衡方程
    FN = -F0 * cosf(theta) - Tp * sinf(theta) / L0; // 静力平衡方程（忽略惯性力+重力项）, 即支撑力=虚拟力在竖直方向的分量+虚拟力矩在竖直方向的分量*sin(theta)/L0，用于判断竖直方向运动速度
    // 牛顿第二定律
    // vmc->FN = vmc->F0*arm_cos_f32(vmc->theta) + vmc->Tp*arm_sin_f32(vmc->theta)/vmc->L0 + 0.6f*(ins->MotionAccel_n[2] - vmc->dd_L0*arm_cos_f32(vmc->theta) + 2.0f*vmc->d_L0*vmc->d_theta*arm_sin_f32(vmc->theta) + vmc->L0*vmc->dd_theta*arm_sin_f32(vmc->theta) + vmc->L0*vmc->d_theta*vmc->d_theta*arm_cos_f32(vmc->theta));

    if (FN < -20)
    {
        //判定已触地
        return 1;
    }
    else
    {
       
        return 0;
        
    }

    return 0;
}
