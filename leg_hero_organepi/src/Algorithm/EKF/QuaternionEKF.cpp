/**
  ******************************************************************************
  * @file           : QuaternionEKF.cpp
  * @author         : Chen Haoran
  * @brief          : None
  * @attention      : None
  * @date           : 2025/10/25
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "QuaternionEKF.h"
/* Define --------------------------------------------------------------------*/

/* Variable && Struct --------------------------------------------------------*/

/* Function Declaration ------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/

using namespace Eigen;
class_Q_EKF Q_EKF;

void QuaternionEKF_Init(float lambda, float dt)
{
    Q_EKF.Quaternion_EKF_Init(lambda, dt);
}

void Quaternion_EKF_Calculate(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    Q_EKF.Quaternion_EKF_Calculate(gx, gy, gz, ax, ay, az, dt);
}

static void User_Function1()
{
    Q_EKF.QuaternionEKF_Linearization_F();
}

static void User_Function2()
{
    Q_EKF.QuaternionEKF_Set_Linearization_H();
}

static void User_Function3()
{
    Q_EKF.Quaternion_EKF_Set_linearization_Z();
}

static void User_Function4()
{
    Q_EKF.Quaternion_Chi_Square_Check();
}

static void User_Function5()
{
    Q_EKF.Quaternion_EKF_Limit();
}

class_Q_EKF::class_Q_EKF() : kf(6, 3, 0, User_Function1, User_Function2, User_Function3, User_Function4, User_Function5)
{
}

/**
 * @brief 四元数EKF初始化
 *
 * @param lambda_ 遗忘因子
 * @param dt_ 采样周期，单位s
 * @return
 */
void class_Q_EKF::Quaternion_EKF_Init(float lambda_, float dt_)
{
    // 初始化姿态四元数
    kf.x_hat << 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;

    kf.StateMinVariance << 0.001f, 0.001f, 0.001f, 0.001f, 0.001f, 0.001f;

    kf.z << 0.0f, 0.0f, 0.0f;

    kf.P << 100000.0f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f,
        0.1f, 100000.0f, 0.1f, 0.1f, 0.1f, 0.1f,
        0.1f, 0.1f, 100000.0f, 0.1f, 0.1f, 0.1f,
        0.1f, 0.1f, 0.1f, 100000.0f, 0.1f, 0.1f,
        0.1f, 0.1f, 0.1f, 0.1f, 100.0f, 0.1f,
        0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 100.0f;

    kf.F << 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f;

    kf.H << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;

    
    constexpr float Q1 = 10.0f * 0.001f;
    constexpr float Q2 = 0.001f * 0.001f;
    constexpr float R1 = 8000000.0f;

    kf.Q << Q1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, Q1, 0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, Q1, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, Q1, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, Q2, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, Q2;

    kf.R << R1, 0.0f, 0.0f,
        0.0f, R1, 0.0f,
        0.0f, 0.0f, R1;

    // 初始化参数
    dt = dt_;
    lambda = lambda_;
}

void class_Q_EKF::Quaternion_EKF_Calculate(float gx, float gy, float gz, float ax, float ay, float az, float dt_)
{
    /*   F矩阵中带*号的位置，即为设定非零的位置
     0      1*     2*     3*     4     5
     6*     7      8*     9*    10    11
    12*    13*    14     15*    16    17
    18*    19*    20*    21     22    23
    24     25     26     27     28    29
    30     31     32     33     34    35
    */

    dt = dt_;

    // 真实角速度为陀螺仪角速度减去零偏
    Gyro[0] = gx - GyroBias[0];
    Gyro[1] = gy - GyroBias[1];
    Gyro[2] = gz - GyroBias[2];

    // 设定F矩阵
    volatile float halfgxdt = 0.5f * Gyro[0] * dt;
    volatile float halfgydt = 0.5f * Gyro[1] * dt;
    volatile float halfgzdt = 0.5f * Gyro[2] * dt;

    // 然后设定状态转移矩阵F的左上角部分 4x4子矩阵，为0.5(Ohm-Ohm_bias)*deltaT，然后下面补一个2x2的单位阵(已经初始化过了)
    // 注意：predict时，F的右上角4x2块矩阵由本函数在x_hat_p计算完成后补充设定，每次predict都会用memcpy将新的x_hat_p覆盖前一时刻。此处未填充完整右上角矩阵，是因为predict阶段尚未获得此阶段的四元数姿态数据，先置为0，得到更优的四元数后再进行更新 [推断]
    kf.F = MatrixXf::Identity(6, 6);

    kf.F(0, 1) = -halfgxdt;
    kf.F(0, 2) = -halfgydt;
    kf.F(0, 3) = -halfgzdt;

    kf.F(1, 0) = halfgxdt;
    kf.F(1, 2) = halfgzdt;
    kf.F(1, 3) = -halfgydt;

    kf.F(2, 0) = halfgydt;
    kf.F(2, 1) = -halfgzdt;
    kf.F(2, 3) = halfgxdt;

    kf.F(3, 0) = halfgzdt;
    kf.F(3, 1) = halfgydt;
    kf.F(3, 2) = -halfgxdt;

    // accel对加速度进行低通滤波，减小碰撞等异常的影响
    if (UpdateCount == 0) // 第一次进入初始化一下加速度
    {
        Accel[0] = ax;
        Accel[1] = ay;
        Accel[2] = az;
        UpdateCount++;
    }

    // 一阶低通滤波
    Accel[0] = Accel[0] * accLPFcoef / (dt + accLPFcoef) + ax * dt / (dt + accLPFcoef);
    Accel[1] = Accel[1] * accLPFcoef / (dt + accLPFcoef) + ay * dt / (dt + accLPFcoef);
    Accel[2] = Accel[2] * accLPFcoef / (dt + accLPFcoef) + az * dt / (dt + accLPFcoef);

    // 设定加速度模长，并归一化一次
    accel_norm = sqrtf(Accel[0] * Accel[0]
        + Accel[1] * Accel[1]
        + Accel[2] * Accel[2]);
    volatile float accelInvNorm = 1.0f / accel_norm;

    kf.z(0) = Accel[0] * accelInvNorm;
    kf.z(1) = Accel[1] * accelInvNorm;
    kf.z(2) = Accel[2] * accelInvNorm;

    // 得到角速度模长
    gyro_norm = sqrtf(Gyro[0] * Gyro[0]
        + Gyro[1] * Gyro[1]
        + Gyro[2] * Gyro[2]);

    // 如果角速度小于设定值且加速度大小在设定范围内，则认为运动稳定，加速度可以用来修正角速度
    // 以后计算姿态更新部分只会用StableFlag来确定
    if (gyro_norm < 0.3f && accel_norm > 9.8f - 0.5f && accel_norm < 9.8f + 0.5f)
    {
        StableFlag = 1;
    }
    else
    {
        StableFlag = 0;
    }

    // 调用卡尔曼滤波计算函数
    kf.Predict();
    kf.Update();

    // 得到滤波数据，取出四元数并估计零偏漂移
    q[0] = kf.x_hat(0);
    q[1] = kf.x_hat(1);
    q[2] = kf.x_hat(2);
    q[3] = kf.x_hat(3);
    GyroBias[0] = kf.x_hat(4);
    GyroBias[1] = kf.x_hat(5);
    GyroBias[2] = 0.0f; // 此时yaw轴朝上，无法观测yaw漂移

    Roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));
    Pitch = sinf(-2.0f * (-(q[0] * q[2]) + (q[1] * q[3])));
    Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));

    // 处理yaw圈数
    if (Yaw - YawAngleLast > std::numbers::pi)
    {
        YawRoundCount--;
    }
    else if (Yaw - YawAngleLast < -std::numbers::pi)
    {
        YawRoundCount++;
    }
    YawTotalAngle = YawRoundCount * 2 * static_cast<float>(std::numbers::pi) + Yaw;
    YawAngleLast = Yaw;
}

/**
 * @brief 对状态转移矩阵F的右上角4*2的矩阵进行线性化，同时估计角速度漂移量的协方差
 *
 * @param
 * @return
 */
void class_Q_EKF::QuaternionEKF_Linearization_F()
{
    static float q0;
    static float q1;
    static float q2;
    static float q3;

    static float qInvNorm;

    q0 = kf.x_hat_p(0);
    q1 = kf.x_hat_p(1);
    q2 = kf.x_hat_p(2);
    q3 = kf.x_hat_p(3);

    qInvNorm = invSqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);

    // 四元数归一化一次
    for (uint8_t i = 0; i < 4; i++)
    {
        kf.x_hat_p(i) *= qInvNorm;
    }
    /*  F, 带*号的是此函数设定的值
     0     1     2     3     4*     5*
     6     7     8     9    10*    11*
    12    13    14    15    16*    17*
    18    19    20    21    22*    23*
    24    25    26    27    28     29
    30    31    32    33    34     35
    */
    kf.F(0, 4) = (q1 * dt) / 2.0f;
    kf.F(0, 5) = (q2 * dt) / 2.0f;

    kf.F(1, 4) = -(q0 * dt) / 2.0f;
    kf.F(1, 5) = (q3 * dt) / 2.0f;

    kf.F(2, 4) = -(q3 * dt) / 2.0f;
    kf.F(2, 5) = -(q0 * dt) / 2.0f;

    kf.F(3, 4) = (q2 * dt) / 2.0f;
    kf.F(3, 5) = -(q1 * dt) / 2.0f;

    // 增大陀螺零偏对应的协方差，防止滤波发散
    kf.P(4, 4) /= lambda;
    kf.P(5, 5) /= lambda;

    // 限幅防止发散
    if (kf.P(4, 4) > 10000)
    {
        kf.P(4, 4) = 10000;
    }
    if (kf.P(5, 5) > 10000)
    {
        kf.P(5, 5) = 10000;
    }
}

/**
 * @brief 用于更新H矩阵，在观测点处的Jacobian矩阵
 *
 * @param
 * @return
 */
void class_Q_EKF::QuaternionEKF_Set_Linearization_H()
{
    /* H矩阵中未列出的元素均为0
     0     1     2     3     4     5
     6     7     8     9    10    11
    12    13    14    15    16    17
    */
    static float double_q0;
    static float double_q1;
    static float double_q2;
    static float double_q3;

    double_q0 = 2.0f * kf.x_hat_p(0);
    double_q1 = 2.0f * kf.x_hat_p(1);
    double_q2 = 2.0f * kf.x_hat_p(2);
    double_q3 = 2.0f * kf.x_hat_p(3);

    kf.H.setZero();

    kf.H(0, 0) = -double_q2;
    kf.H(0, 1) = double_q3;
    kf.H(0, 2) = -double_q0;
    kf.H(0, 3) = double_q1;

    kf.H(1, 0) = double_q1;
    kf.H(1, 1) = double_q0;
    kf.H(1, 2) = double_q3;
    kf.H(1, 3) = double_q2;

    kf.H(2, 0) = double_q0;
    kf.H(2, 1) = -double_q1;
    kf.H(2, 2) = -double_q2;
    kf.H(2, 3) = double_q3;
}


/**
 * @brief 设定非线性函数H(X_p)，用于计算卡尔曼校正
 *
 * @param
 * @return
 */
void class_Q_EKF::Quaternion_EKF_Set_linearization_Z()
{
    static float q0;
    static float q1;
    static float q2;
    static float q3;

    q0 = kf.x_hat_p(0);
    q1 = kf.x_hat_p(1);
    q2 = kf.x_hat_p(2);
    q3 = kf.x_hat_p(3);

    // 更新非线性测量值
    kf.z_nl(0) = 2.0f * (q1 * q3 - q0 * q2);
    kf.z_nl(1) = 2.0f * (q0 * q1 + q2 * q3);
    kf.z_nl(2) = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // 计算方向余弦的值
    for (uint8_t i = 0; i < 3; i++)
    {
        OrientationCosine[i] = cosf(fabsf(kf.z_nl(i)));
    }
}

/**
 * @brief 卡方检验判断错误
 *
 * @param
 * @return
 */
void class_Q_EKF::Quaternion_Chi_Square_Check()
{
    // 卡方检验判断

    // 计算新息向量
    kf.z_err = kf.z - kf.z_nl;

    // 定义检测函数
    // D_k = H * P_p * H^T + R
    // r_k = z_err^T * D_k^-1 * z_err
    r_k = kf.z_err.transpose() * (kf.H * kf.P_p * kf.H.transpose() + kf.R).inverse() * kf.z_err;

    // rk小于0.5倍阈值，说明滤波器已经收敛
    if (r_k < 0.5f * ChiSquareTestThreshold)
    {
        ConvergeFlag = 1;
    }

    // rk较大，且滤波器已收敛，判断是否发散
    if (r_k > ChiSquareTestThreshold && ConvergeFlag)
    {
        if (StableFlag)
        {
            ErrorCount++; // 机体静止时无法通过卡方检验
        }
        else
        {
            ErrorCount = 0;
        }

        if (ErrorCount > 50)
        {
            // 滤波器发散
            ConvergeFlag = 0;
        }
        else
        {
            // 有残差未通过卡方检验，则重新预测
            // x_hat = x_hat_p
            // P = P_p
            kf.K = MatrixXf::Zero(6, 3);
        }

        AdaptiveGainScale = 1;
    }
    else
    {
        // rk越小，增益越大，滤波结果越接近预测值
        if (r_k > 0.1f * ChiSquareTestThreshold && ConvergeFlag)
        {
            AdaptiveGainScale = (ChiSquareTestThreshold - r_k) / (0.9f * ChiSquareTestThreshold);
        }
        else
        {
            AdaptiveGainScale = 1;
        }
        ErrorCount = 0;
    }

    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            kf.K(i, j) *= AdaptiveGainScale;
        }
    }
}


/**
 * @brief 限制漂移值，约束yaw轴漂移
 *
 * @param
 * @return
 */
void class_Q_EKF::Quaternion_EKF_Limit()
{
    if (ConvergeFlag)
    {
        for (uint8_t i = 4; i < 6; i++)
        {
            if (kf.x_k_out(i) > 0.01f * dt)
            {
                kf.x_k_out(i) = 0.01f * dt;
            }
            if (kf.x_k_out(i) < -0.01f * dt)
            {
                kf.x_k_out(i) = -0.01f * dt;
            }
        }
    }

    // 限制yaw轴漂移
    // kf.x_k_out(3) = 0.0f;
}


float Get_Pitch()
{
    return Q_EKF.Pitch;
}

float Get_Roll()
{
    return Q_EKF.Roll;
}

float Get_Yaw()
{
    return Q_EKF.Yaw;
}

float Get_Total_Yaw()
{
    return Q_EKF.YawTotalAngle;
}

float* Get_Accel()
{
    return Q_EKF.Accel;
}

float* Get_q()
{
    return Q_EKF.q;
}


/**
 * @brief 自定义1/sqrt(x)，速度更快
 *
 * @param x x
 * @return float
 */
float class_Q_EKF::invSqrtf(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f375a86 - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
