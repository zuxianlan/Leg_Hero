/**
  ******************************************************************************
  * @file           : INS.h
  * @author         : Chen Haoran
  * @brief          : None
  * @attention      : None
  * @date           : 2025/11/4
  ******************************************************************************
  */
#ifndef WHEEL_LEG_SYS_INS_H
#define WHEEL_LEG_SYS_INS_H
/* Includes ------------------------------------------------------------------*/
#include "Algorithm/EKF/QuaternionEKF.h"
#include "Bsp/Timer.h"
#include "Algorithm/Fusion_AHRS/Tactical_Fusion.h"
/* Define --------------------------------------------------------------------*/
#define correct_Time_define 1000
#define cheat 0

/* Enum ----------------------------------------------------------------------*/

/* Variable && Struct --------------------------------------------------------*/

/* Function Declaration ------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/

class class_INS
{
public:

    float* gyro{};
    float* accel{};

    float roll{};
    float pitch{};
    float yaw{};
    float total_yaw{};

    float roll_ahrs{};
    float pitch_ahrs{};
    float yaw_ahrs{};
    float MotionAccel_n[3] = {}; // 绝对系加速度

    static void INS_Init();

    void INS_Update();

    void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);

    void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);

private:
    int X = 0;
    int Y = 1;
    int Z = 2;

    float q[4] = {}; // 四元数估计值

    float Gyro[3] = {};  // 角速度
    float Accel[3] = {}; // 加速度
    float MotionAccel_b[3] = {}; // 机体坐标加速度

    float AccelLPF = 0.0085; // 加速度低通滤波系数

    // 加速度在绝对系的向量表示
    float xn[3] = {};
    float yn[3] = {};
    float zn[3] = {};

    float atanxz = 0.0f;
    float atanyz = 0.0f;

    int attitude_flag = 1;
    int ins_flag = 0;
    int correct_times = 0;

    float gyro_correct[3] = {};
};

#endif //WHEEL_LEG_SYS_INS_H
