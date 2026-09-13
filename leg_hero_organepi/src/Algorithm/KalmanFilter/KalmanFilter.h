/**
  ******************************************************************************
  * @file           : KalmanFilter.h
  * @author         : Chen Haoran
  * @brief          : None
  * @attention      : None
  * @date           : 2025/9/9
  ******************************************************************************
  */

#ifndef KALMANFILTER_H
#define KALMANFILTER_H

/* Includes ------------------------------------------------------------------*/
#include <iostream>
#include <vector>
#include <Eigen/Dense>
/* Define --------------------------------------------------------------------*/

/* Enum ----------------------------------------------------------------------*/

/* Variable && Struct --------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/

typedef void(*UserFunction)();

class KalmanFilter
{
private:
    // 系统维度
    int StateDim; // 状态向量维度, 也就是预测量维度x
    int MeasureDim; // 测量向量维度z
    int ControlDim; // 控制向量维度u

    UserFunction UserFunc1;
    UserFunction UserFunc2;
    UserFunction UserFunc3;
    UserFunction UserFunc4;
    UserFunction UserFunc5;


public:
    // 系统模型矩阵
    Eigen::MatrixXf F; // 状态转移矩阵 可视为离散化后的A矩阵
    Eigen::MatrixXf B; // 控制输入矩阵
    Eigen::MatrixXf H; // 测量与状态关系矩阵
    Eigen::MatrixXf Q; // 过程噪声协方差矩阵
    Eigen::MatrixXf R; // 测量噪声协方差矩阵
    Eigen::MatrixXf P; // 误差噪声协方差矩阵
    Eigen::MatrixXf P_p; // 先验误差噪声协方差矩阵
    Eigen::MatrixXf K; // 卡尔曼增益矩阵
    Eigen::MatrixXf I; // 单位矩阵 StateDim * StateDim

    // 当前状态
    Eigen::VectorXf x_hat_p; // 先验估计值
    Eigen::VectorXf x_k_out; // 先验估计值
    Eigen::VectorXf x_hat; // 后验估计值
    Eigen::VectorXf z; // 测量向量
    Eigen::VectorXf z_nl; // 非线性向量，即H(xhat_p)
    Eigen::VectorXf z_err; // 用于卡方检验的z值误差

    // 其他变量
    Eigen::VectorXf StateMinVariance; // 最小协方差 防止过度收敛

    // 构造函数
    KalmanFilter(int StateDim, int MeasureDim, int ControlDim, UserFunction UserFunc1, UserFunction UserFunc2,
                 UserFunction UserFunc3, UserFunction UserFunc4, UserFunction UserFunc5);
    void Init(const Eigen::VectorXf& x0, const Eigen::MatrixXf& p0);
    void SetModel(const Eigen::MatrixXf& F_, const Eigen::MatrixXf& H_, const Eigen::MatrixXf& Q_,
                  const Eigen::MatrixXf& R_);
    void SetControlB(const Eigen::MatrixXf& B_);

    void Set_F(const Eigen::MatrixXf& F_);

    void Set_H(const Eigen::MatrixXf& H_);
    void Set_P(const Eigen::MatrixXf& P_);
    void Set_x_hat_p(const Eigen::VectorXf& x_hat_p_);
    void SetStateMinVariance(const Eigen::VectorXf& M_);
    void Set_Z(const Eigen::VectorXf& z_);
    void Set_Z_nl(const Eigen::VectorXf& z_nl_);
    void Predict(const Eigen::VectorXf& u = Eigen::VectorXf());
    void Update();
    Eigen::VectorXf get_p_output();
    Eigen::VectorXf get_output();
    Eigen::VectorXf get_xk_out();
    Eigen::VectorXf get_P();

private:
    void H_K_R_Adjustment();
};


#endif //KALMANFILTER_H
