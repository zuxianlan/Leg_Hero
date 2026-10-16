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
 * @brief ����theta��dtheta��LQR��������ͬʱ�õ��ȳ�l0
 *
 * @param
 * @return
 */
void class_vmc_leg::VMC_Calc_1()
{
    static float pitch, d_pitch;
    pitch = -INS->pitch;
    d_pitch = -INS->gyro[0];

    // ����õ��ȳ�l0��phi0
    leg_position(phi1, phi4, &L0, &phi0);
    leg_speed(d_phi1, d_phi4, phi1, phi4, &d_L0, &d_phi0);

    // ����н�alpha��֮�������������theta
    alpha = phi0 - std::numbers::pi / 2.0;

    //vmc->d_phi0 = (vmc->phi0 - vmc->last_phi0) / dt;

    //�õ�״̬����1 2
    theta = alpha - pitch;
    d_theta = (d_phi0 - d_pitch);

    // ��������
    last_phi0 = phi0;

    // ������ر���
    // vmc->d_L0 = (vmc->L0 - vmc->last_L0) / dt;
    dd_L0 = (d_L0 - last_d_L0) / dt;

    last_d_L0 = d_L0;
    last_L0 = L0;

    dd_theta = (d_theta - last_d_theta) / dt;
    last_d_theta = d_theta;
}

/**
 * @brief ͨ��LQR�Լ������õ���Tp�����Źؽڵ���������
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
 * @brief ��ؼ��
 *
 * @param
 * @return
 */
uint8_t class_vmc_leg::ground_detection_L()
{

    // ��ֹƽ���ʽ
    FN = -F0 * cosf(theta) - Tp * sinf(theta) / L0; // �Ȳ���������+��������, �����������������*��������ֱ�����˶����ٶ�
    // ţ�ٵڶ�����
    // vmc->FN = vmc->F0*arm_cos_f32(vmc->theta) + vmc->Tp*arm_sin_f32(vmc->theta)/vmc->L0 + 0.6f*(ins->MotionAccel_n[2] - vmc->dd_L0*arm_cos_f32(vmc->theta) + 2.0f*vmc->d_L0*vmc->d_theta*arm_sin_f32(vmc->theta) + vmc->L0*vmc->dd_theta*arm_sin_f32(vmc->theta) + vmc->L0*vmc->d_theta*vmc->d_theta*arm_cos_f32(vmc->theta));

    if (FN > 20)
    {
        //�����
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
 

    // ��ֹƽ���ʽ
    FN = -F0 * cosf(theta) - Tp * sinf(theta) / L0; // �Ȳ���������+��������, �����������������*��������ֱ�����˶����ٶ�
    // ţ�ٵڶ�����
    // vmc->FN = vmc->F0*arm_cos_f32(vmc->theta) + vmc->Tp*arm_sin_f32(vmc->theta)/vmc->L0 + 0.6f*(ins->MotionAccel_n[2] - vmc->dd_L0*arm_cos_f32(vmc->theta) + 2.0f*vmc->d_L0*vmc->d_theta*arm_sin_f32(vmc->theta) + vmc->L0*vmc->dd_theta*arm_sin_f32(vmc->theta) + vmc->L0*vmc->d_theta*vmc->d_theta*arm_cos_f32(vmc->theta));

    if (FN < -20)
    {
        //�����
        return 1;
    }
    else
    {
       
        return 0;
        
    }

    return 0;
}
