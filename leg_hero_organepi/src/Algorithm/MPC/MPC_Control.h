/**
  ******************************************************************************
  * @file           : MPC_Control.h
  * @author         : gagami
  * @brief          : None
  * @attention      : None
  * @date           : 2025/9/17
  ******************************************************************************
  */
#ifndef MPC_CONTROL_H
#define MPC_CONTROL_H
/* Includes ------------------------------------------------------------------*/
#include <Eigen/Dense>
#include <qpOASES.hpp>
/* Define --------------------------------------------------------------------*/

/* Enum ----------------------------------------------------------------------*/

/* Variable && Struct --------------------------------------------------------*/

/* Function Declaration ------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/

class MPC_Controller
{
public:
    Eigen::VectorXd x;
    Eigen::VectorXd x_ref;
    Eigen::MatrixXd u_out;

    Eigen::MatrixXd A; // 系统状态矩阵
    Eigen::MatrixXd B; // 系统输入矩阵
    // MatrixXd Cc; // 系统输出矩阵
    Eigen::MatrixXd Q; // 状态权重矩阵
    Eigen::MatrixXd R; // 输入权重矩阵
    Eigen::MatrixXd F; // 终端代价矩阵
    Eigen::MatrixXd Q_bar;
    Eigen::MatrixXd R_bar;

    // 输入约束
    Eigen::VectorXd u_min;
    Eigen::VectorXd u_max;

    // 状态约束
    Eigen::VectorXd x_min;
    Eigen::VectorXd x_max;

    MPC_Controller(int n, int p, int y, int N);

    void Set_x(const Eigen::VectorXd& x_);

    void BuildPredictionMatrices();

    void BuildHessianMatrices();

    void BuildGradientVector();

    void BuildConstraintBounds();

    void MPC_Calculate();

private:
    int n; // 状态维度
    int p; // 输入维度
    int y; // 输出维度
    int N; // 预测时域
    int is_initialized;
    int solver_iterations{};

    // qpOASES问题实例
    qpOASES::QProblem qp;
    qpOASES::returnValue solver_status;

    // 输出约束
    // VectorXd y_min;
    // VectorXd y_max;

    // 预测矩阵
    Eigen::MatrixXd M; //MPC公式预测矩阵M
    Eigen::MatrixXd C; //MPC公式预测矩阵C
    // MatrixXd P; //MPC输出预测矩阵P

    //QP问题矩阵
    Eigen::MatrixXd H; // Hessian矩阵
    Eigen::VectorXd g; // 梯度向量f
    Eigen::MatrixXd A_mat; // 约束矩阵A
    Eigen::VectorXd lb;
    Eigen::VectorXd ub;
    Eigen::VectorXd lba;
    Eigen::VectorXd uba;

    // 上一次的控制输入（用于热启动）
    Eigen::VectorXd last_control;

    void initialize_qp();
};

int MPC_test();
#endif //MPC_CONTROL_H
