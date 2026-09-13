/**
  ******************************************************************************
  * @file           : KalmanFilter.cpp
  * @author         : Chen Haoran
  * @brief          : None
  * @attention      : None
  * @date           : 2025/9/9
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "KalmanFilter.h"
/* Define --------------------------------------------------------------------*/

/* Enum ----------------------------------------------------------------------*/

/* Variable && Struct --------------------------------------------------------*/

/* Function Declaration ------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/

using namespace Eigen;

// 构造函数
KalmanFilter::KalmanFilter(int StateDim, int MeasureDim, int ControlDim = 0, UserFunction UserFunc1 = nullptr,
                           UserFunction UserFunc2 = nullptr, UserFunction UserFunc3 = nullptr,
                           UserFunction UserFunc4 = nullptr,
                           UserFunction UserFunc5 = nullptr) : StateDim(StateDim),
                                                               MeasureDim(MeasureDim),
                                                               ControlDim(ControlDim), UserFunc1(UserFunc1),
                                                               UserFunc2(UserFunc2),
                                                               UserFunc3(UserFunc3), UserFunc4(UserFunc4),
                                                               UserFunc5(UserFunc5)
{
    // 初始化矩阵
    F = MatrixXf::Identity(StateDim, StateDim);
    H = MatrixXf::Zero(MeasureDim, StateDim);
    Q = MatrixXf::Zero(StateDim, StateDim);
    R = MatrixXf::Zero(MeasureDim, MeasureDim);
    P = MatrixXf::Identity(StateDim, StateDim);
    P_p = MatrixXf::Identity(StateDim, StateDim);
    K = MatrixXf::Zero(StateDim, MeasureDim);
    I = MatrixXf::Identity(StateDim, StateDim);

    x_hat_p = VectorXf::Zero(StateDim);
    x_k_out = VectorXf::Zero(StateDim);
    x_hat = VectorXf::Zero(StateDim);
    StateMinVariance = VectorXf::Zero(StateDim);
    z = VectorXf::Zero(MeasureDim);
    z_nl = VectorXf::Zero(MeasureDim);
    z_err = VectorXf::Zero(MeasureDim);

    if (ControlDim > 0)
    {
        B = MatrixXf::Zero(StateDim, ControlDim);
    }
}

/**
* @brief 初始化，设定初始值
*
* @param x0 初始观测向量
* @param p0 初始误差协方差矩阵
* @return
*/
void KalmanFilter::Init(const VectorXf& x0, const MatrixXf& p0)
{
    x_hat = x0;
    P = p0;
}

/**
* @brief 设定系统模型矩阵
*
* @param F_ 状态转移矩阵
* @param H_ 观测矩阵
* @param Q_ 过程噪声矩阵
* @param R_ 测量噪声矩阵
* @return
*/
void KalmanFilter::SetModel(const MatrixXf& F_, const MatrixXf& H_, const MatrixXf& Q_, const MatrixXf& R_)
{
    this->F = F_;
    this->H = H_;
    this->Q = Q_;
    this->R = R_;
}

/**
* @brief 设定控制矩阵B
*
* @param
* @return
*/
void KalmanFilter::SetControlB(const MatrixXf& B_)
{
    this->B = B_;
}

void KalmanFilter::Set_F(const MatrixXf& F_)
{
    this->F = F_;
}

void KalmanFilter::Set_H(const MatrixXf& H_)
{
    this->H = H_;
}

void KalmanFilter::Set_P(const MatrixXf& P_)
{
    this->P = P_;
}

void KalmanFilter::Set_x_hat_p(const VectorXf& x_hat_p_)
{
    this->x_hat_p = x_hat_p_;
}

/**
* @brief 设定最小协方差，防止过度收敛
*
* @param
* @return
*/
void KalmanFilter::SetStateMinVariance(const VectorXf& M_)
{
    this->StateMinVariance = M_;
}

/**
* @brief 设定测量量
*
* @param
* @return
*/
void KalmanFilter::Set_Z(const VectorXf& z_)
{
    this->z = z_;
}

/**
* @brief 设定非线性测量量
*
* @param
* @return
*/
void KalmanFilter::Set_Z_nl(const VectorXf& z_nl_)
{
    this->z_nl = z_nl_;
}

/**
* @brief 预测，卡尔曼滤波黄金五式的前两式
*
* @param u 当有控制量的时候输入，无控制量的时候为空
* @return
*/
void KalmanFilter::Predict(const VectorXf& u)
{
    // 先验 1. X_k = F * X_k-1 + B * U_k-1
    if (u.size() > 0 && ControlDim > 0)
    {
        x_hat_p = F * x_hat + B * u;
    }
    else
    {
        x_hat_p = F * x_hat;
    }

    if (UserFunc1 != nullptr)
    {
        UserFunc1();
    }

    // 2. 先验误差协方差 P_kp = F * P_k-1 * F^T + Q
    P_p = F * P * F.transpose() + Q;

    if (UserFunc2 != nullptr)
    {
        UserFunc2();
    }
}

/**
* @brief 校验，更新 卡尔曼滤波黄金五式后三式
*
* @param
* @return
*/
void KalmanFilter::Update()
{
    if (UserFunc3 != nullptr)
    {
        UserFunc3();
    }

    // 3. 更新计算卡尔曼增益K K = (P_kp * H^T) / (H * P_kp * H^T + R)
    K = (P_p * H.transpose()) * (H * P_p * H.transpose() + R).inverse();

    if (UserFunc4 != nullptr)
    {
        UserFunc4();
    }

    x_k_out = K * (z - H * x_hat_p);

    if (UserFunc5 != nullptr)
    {
        UserFunc5();
    }

    // 4. 后验估计 X_hat = X_hat_p + K * (Z - H * X_hat_p)
    x_hat = x_hat_p + x_k_out;

    // 5. 误差协方差矩阵更新 P = (I - K * H) * P_p
    P = (I - K * H) * P_p;

    // 避免滤波器过度收敛
    for (int i = 0; i < StateDim; i++)
    {
        if (P(i, i) < StateMinVariance(i))
        {
            P(i, i) = StateMinVariance(i);
        }
    }
}

/**
 * @brief 传出先验估计值
 *
 * @param
 * @return x_hat_p 先验估计向量
 */
VectorXf KalmanFilter::get_p_output()
{
    return x_hat_p;
}

/**
 * @brief 传出后验估计值
 *
 * @param
 * @return x_hat 后验估计向量
 */
VectorXf KalmanFilter::get_output()
{
    return x_hat;
}

/**
* @brief 传出增益乘后的值，即K * (Z - H * X_hat_p)
*
* @param
* @return x_hat 后验估计向量
*/
VectorXf KalmanFilter::get_xk_out()
{
    x_k_out = x_hat;
    return x_hat;
}

VectorXf KalmanFilter::get_P()
{
    return P;
}

/**
* @brief 根据传感器是否在线动态调整H K R矩阵的值可选择是否开启调整
*
* @param
* @return
*/
void KalmanFilter::H_K_R_Adjustment()
{
    MatrixXf tempH = MatrixXf::Zero(H.rows(), H.cols());
    MatrixXf tempR = MatrixXf::Zero(R.rows(), R.cols());
    VectorXf tempz = VectorXf::Zero(H.rows());
    uint8_t j = 0;

    for (int i = 0; i < H.rows(); i++)
    {
        if (z(i) != 0)
        {
            for (int k = 0; k < H.cols(); k++)
            {
                tempH(i - j, k) = H(i, k);
            }
            for (int k = 0; k < R.cols(); k++)
            {
                tempR(i - j, k) = R(i, k);
            }
            tempz(i - j) = z(i);
        }
        else
        {
            j++;
        }
    }

    tempH.conservativeResize(H.rows() - j, H.cols());
    tempR.conservativeResize(R.rows() - j, R.cols());
    tempz.conservativeResize(H.rows() - j);

    H.setZero();
    H.conservativeResize(tempH.rows() - j, tempH.cols());
    R.setZero();
    R.conservativeResize(R.rows() - j, R.cols());
    z.setZero();
    z.conservativeResize(H.rows() - j);

    H = tempH;
    R = tempR;
    z = tempz;

    K.conservativeResize(H.cols(), R.rows() - j);
}
