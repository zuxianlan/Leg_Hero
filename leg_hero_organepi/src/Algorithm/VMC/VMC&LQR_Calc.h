/**
  ******************************************************************************
  * @file           : VMC&LQR_Calc.h
  * @author         : gagami
  * @brief          : None
  * @attention      : None
  * @date           : 2025/8/7
  ******************************************************************************
  */
#ifndef VMC_LQR_CALC_H
#define VMC_LQR_CALC_H

/* Includes ------------------------------------------------------------------*/
#include <stdint-gcc.h>
#include "INS.h"
/* Define --------------------------------------------------------------------*/

/* Enum ----------------------------------------------------------------------*/

/* Variable && Struct --------------------------------------------------------*/

class class_vmc_leg
{
public:

  class_INS *INS{};

  void VMC_Calc_1();

  void VMC_Calc_2();

  uint8_t ground_detection_L();
  uint8_t ground_detection_R();

  inline void Set_phi1(float __phi1);

  inline void Set_dphi1(float __dphi1);

  inline void Set_phi4(float __phi4);

  inline void Set_dphi4(float __dphi4);

  inline void Set_F0(float __F0);

  inline void Set_Tp(float __Tp);

  inline void Set_dt(float __dt);

  inline float Get_alpha();

  inline float Get_theta();

  inline float Get_dtheta();

  inline float Get_L0();

  inline float Get_dL0();

  inline float Get_Phi0();

  inline float Get_dPhi0();

  inline float Get_torque1();

  inline float Get_torque2();

  inline float Get_F0();

  inline float Get_Fn();

  inline float Get_Tp();

private:
  float L0 = 0, phi0 = 0;//C杆的长度

  float d_phi0 = 0;//记录C杆角度phi0的变化量
  float last_phi0 = 0;//上一时刻C杆角度，用于计算角度phi0的变化量d_phi0

  float phi2 = 0, phi3 = 0;
  float phi1 = 0, phi4 = 0;
  float d_phi1 = 0, d_phi4 = 0;

  float torque_set[2]  = {};//VMC计算输出的两个电机力矩

  float F0 = 0;
  float Tp = 0;
  float F02 = 0;

  float alpha = 0;
  float theta = 0;
  float d_theta = 0;//theta的一阶导数
  float last_d_theta = 0;
  float dd_theta = 0;//theta的二阶导数

  float d_L0 = 0;//L0的一阶导数
  float dd_L0 = 0;//L0的二阶导数
  float last_L0 = 0;
  float last_d_L0 = 0;

  float FN = 0;//支撑力

  float dt = 0.001;

};

inline void class_vmc_leg::Set_phi1(float __phi1)
{
  phi1 = __phi1;
}

inline void class_vmc_leg::Set_dphi1(float __dphi1)
{
  d_phi1 = __dphi1;
}

inline void class_vmc_leg::Set_phi4(float __phi4)
{
  phi4 = __phi4;
}

inline void class_vmc_leg::Set_dphi4(float __dphi4)
{
  d_phi4 = __dphi4;
}

inline void class_vmc_leg::Set_F0(float __F0)
{
  F0 = __F0;
}

inline void class_vmc_leg::Set_Tp(float __Tp)
{
  Tp = __Tp;
}

inline void class_vmc_leg::Set_dt(float __dt)
{
  dt = __dt;
}

inline float class_vmc_leg::Get_alpha()
{  
  return alpha;
}


inline float class_vmc_leg::Get_theta()
{  
  return theta;
}

inline float class_vmc_leg::Get_dtheta()
{
  return d_theta;
}

inline float class_vmc_leg::Get_L0()
{
  return L0;
}

inline float class_vmc_leg::Get_dL0()
{
  return d_L0;
}

inline float class_vmc_leg::Get_Phi0()
{
  return phi0;
}

inline float class_vmc_leg::Get_dPhi0()
{
  return d_phi0;
}

inline float class_vmc_leg::Get_torque1()
{
  return torque_set[0];
}

inline float class_vmc_leg::Get_torque2()
{
  return torque_set[1];
}

inline float class_vmc_leg::Get_F0()
{
  return F0;
}

inline float class_vmc_leg::Get_Fn()
{
  return FN;
}

inline float class_vmc_leg::Get_Tp()
{
  return Tp;
}



#endif //VMC_LQR_CALC_H
