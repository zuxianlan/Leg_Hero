/**
  ******************************************************************************
  * @file           : MPC_Control.cpp
  * @author         : gagami
  * @brief          : None
  * @attention      : None
  * @date           : 2025/9/17
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "MPC_Control.h"
#include <iostream>

/* Define --------------------------------------------------------------------*/

/* Enum ----------------------------------------------------------------------*/

/* Variable && Struct --------------------------------------------------------*/

/* Function Declaration ------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/

using namespace Eigen;
using namespace qpOASES;

MPC_Controller::MPC_Controller(const int n, const int p, const int y, const int N) : n(n), p(p), y(y), N(N)
{
    // 初始化矩阵
    x = VectorXd::Zero(n);
    x_ref = VectorXd::Zero((N + 1) * n);
    u_out = VectorXd::Zero(N * p);

    A = MatrixXd::Identity(n, n);
    B = MatrixXd::Zero(n, p);
    Q = MatrixXd::Zero(n, n);
    R = MatrixXd::Zero(p, p);
    F = MatrixXd::Zero(n, n);
    // Cc = MatrixXd ::Identity(y, n);
    Q_bar = MatrixXd::Identity((N + 1) * n, (N + 1) * n);
    R_bar = MatrixXd::Zero(N * p, N * p);

    x_min = VectorXd::Zero(n);
    x_max = VectorXd::Zero(n);
    u_min = VectorXd::Zero(p);
    u_max = VectorXd::Zero(p);
    // y_min = VectorXd::Zero(y);
    // y_max = VectorXd::Zero(y);

    M = MatrixXd::Zero((N + 1) * n, n);
    C = MatrixXd::Zero((N + 1) * n, N * p);
    // P = MatrixXd::Zero(n, n);

    H = MatrixXd::Zero(N * p, N * p);
    g = VectorXd::Zero(N * p);
    A_mat = MatrixXd::Zero(N * (n + p), n + p);
    lb = VectorXd::Zero(N * (p));
    ub = VectorXd::Zero(N * (p));
    lba = VectorXd::Zero(N * (n + p));
    uba = VectorXd::Zero(N * (n + p));
    last_control = VectorXd::Zero(p);
    is_initialized = 0;

    // qp = QProblem(N * p, 2 * N * (n +p));
    qp = QProblem(N * p, 0);
    Options options;
    options.printLevel = PL_LOW;
    solver_status = TERMINAL_LIST_ELEMENT;
    // options.setToMPC();

    qp.setOptions(options);

    BuildPredictionMatrices();
}

void MPC_Controller::Set_x(const VectorXd& x_)
{
    x = x_;
}


void MPC_Controller::BuildPredictionMatrices()
{
    // 构建M矩阵
    // MatrixXd A_power = MatrixXd::Identity(n, n);
    // for (int i = 0; i < N + 1; i++)
    // {
    //     M.block(i * n, 0, n, n) = A_power;
    //     A_power = A * A_power;
    // }

    // // 构建C矩阵
    // for (int i = 0; i < N + 1; i++)
    // {
    //     A_power = MatrixXd::Identity(n, n);
    //     for (int j = 0; j < i; j++)
    //     {
    //         C.block(i * n, (i - j - 1) * p, n, p) = A_power * B;
    //         A_power = A * A_power;
    //     }
    // }

    // 构建 M 矩阵 (N+1)n x n
    M.block(0, 0, n, n) = MatrixXd::Identity(n, n);
    for (int i = 1; i <= N; i++)
    {
        M.block(i * n, 0, n, n) = A * M.block((i - 1) * n, 0, n, n);
    }

    // 构建 C 矩阵 (N+1)n x Np
    C.setZero();
    for (int i = 1; i <= N; i++) // 从第1步预测开始有控制输入影响
    {
        for (int j = 0; j < i; j++)
        {
            // C 矩阵的第 i 个状态块受第 j 个输入块的影响系数为 A^(i-j-1) * B
            MatrixXd A_pow = MatrixXd::Identity(n, n);
            for(int k=0; k < (i-j-1); k++) A_pow = A * A_pow;
            
            C.block(i * n, j * p, n, p) = A_pow * B;
        }
    }
}


void MPC_Controller::BuildHessianMatrices()
{
    // // 构建Hessian矩阵
    // for (int i = 0; i < N; i++)
    // {
    //     Q_bar.block(i * n, i * n, n, n) = Q;
    // }
    // Q_bar.block((N) * n, (N) * n, n, n) = F; // 终端代价

    // for (int i = 0; i < N; i++)
    // {
    //     R_bar.block(i * p, i * p, p, p) = R;
    // }

    // H = (C.transpose() * Q_bar * C) + R_bar;

    // 确保 Q_bar 维度为 (N+1)n x (N+1)n
    Q_bar.setZero(); 
    for (int i = 0; i < N; i++)
    {
        Q_bar.block(i * n, i * n, n, n) = Q;
    }
    // 最后一个块使用终端代价 F
    Q_bar.block(N * n, N * n, n, n) = F; 

    // R_bar 维度为 Np x Np
    R_bar.setZero();
    for (int i = 0; i < N; i++)
    {
        R_bar.block(i * p, i * p, p, p) = R;
    }

    // 标准 QP 形式: H = 2 * (C' * Q_bar * C + R_bar)
    // qpOASES 求解的是 0.5 * x' * H * x，所以这里建议乘 2 或者保持一致
    H = (C.transpose() * Q_bar * C) + R_bar;
}


void MPC_Controller::BuildGradientVector()
{
//     // 构建Gradient向量
//     g = VectorXd::Zero(N * p);

//     g = C.transpose() * Q_bar * (M * x - x_ref);

    // 标准 QP 梯度: g = 2 * C' * Q_bar * (M * x - x_ref)
    // 注意：x_ref 的维度必须是 (N+1)n，包含从 k=0 到 k=N 的所有参考状态
    g = C.transpose() * Q_bar * (M * x - x_ref);
}

void MPC_Controller::BuildConstraintBounds()
{
    // for (int i = 0; i < N; i++)
    // {
    //     A_mat.block(i * (n + p), 0, n + p, n + p) = MatrixXd::Identity(n + p, n + p);
    // }
    //
    // for (int i = 0; i < N; i++)
    // {
    // lb.segment(i * (n + p), n) = x_min;
    // lb.segment(i * (n + p) + n, p) = u_min;
    // ub.segment(i * (n + p), n) = x_max;
    // ub.segment(i * (n + p) + n, p) = u_max;
    // }
    //
    // lba = lb;
    // uba = ub;

    for (int i = 0; i < N; i++)
    {
        lb.segment(i * p, p) = u_min;
        ub.segment(i * p, p) = u_max;
    }
}

void MPC_Controller::initialize_qp()
{
    // 构建梯度向量和约束边界
    BuildGradientVector();
    BuildConstraintBounds();

    int nWSR = 100;

    // 初始化QP问题

    solver_status = qp.init(
        H.data(),
        g.data(),
        nullptr,
        lb.data(),
        ub.data(),
        nullptr,
        nullptr,
        nWSR);

    solver_iterations = nWSR;

    is_initialized = 1;
}

void MPC_Controller::MPC_Calculate()
{
    // 构建梯度向量和约束边界
    BuildGradientVector();
    BuildConstraintBounds();

    if (is_initialized)
    {
        int nWSR = 100;

        solver_status = qp.hotstart(
            g.data(), lb.data(), ub.data(),
            nullptr, nullptr, nWSR
        );

        if (solver_status != qpOASES::SUCCESSFUL_RETURN)
        {
            initialize_qp();
        }
    }
    else
    {
        initialize_qp();
    }

    qp.getPrimalSolution(u_out.data());

    is_initialized = true;

}
