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
    // 鍒濆�嬪寲鐭╅樀
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
    // 鏋勫缓M鐭╅樀
    // MatrixXd A_power = MatrixXd::Identity(n, n);
    // for (int i = 0; i < N + 1; i++)
    // {
    //     M.block(i * n, 0, n, n) = A_power;
    //     A_power = A * A_power;
    // }

    // // 鏋勫缓C鐭╅樀
    // for (int i = 0; i < N + 1; i++)
    // {
    //     A_power = MatrixXd::Identity(n, n);
    //     for (int j = 0; j < i; j++)
    //     {
    //         C.block(i * n, (i - j - 1) * p, n, p) = A_power * B;
    //         A_power = A * A_power;
    //     }
    // }

    // 鏋勫缓 M 鐭╅樀 (N+1)n x n
    M.block(0, 0, n, n) = MatrixXd::Identity(n, n);
    for (int i = 1; i <= N; i++)
    {
        M.block(i * n, 0, n, n) = A * M.block((i - 1) * n, 0, n, n);
    }

    // 鏋勫缓 C 鐭╅樀 (N+1)n x Np
    C.setZero();
    for (int i = 1; i <= N; i++) // 浠庣��1姝ラ�勬祴寮€濮嬫湁鎺у埗杈撳叆褰卞搷
    {
        for (int j = 0; j < i; j++)
        {
            // C 鐭╅樀鐨勭�� i 涓�鐘舵€佸潡鍙楃�� j 涓�杈撳叆鍧楃殑褰卞搷绯绘暟涓� A^(i-j-1) * B
            MatrixXd A_pow = MatrixXd::Identity(n, n);
            for(int k=0; k < (i-j-1); k++) A_pow = A * A_pow;
            
            C.block(i * n, j * p, n, p) = A_pow * B;
        }
    }
}


void MPC_Controller::BuildHessianMatrices()
{
    // // 鏋勫缓Hessian鐭╅樀
    // for (int i = 0; i < N; i++)
    // {
    //     Q_bar.block(i * n, i * n, n, n) = Q;
    // }
    // Q_bar.block((N) * n, (N) * n, n, n) = F; // 缁堢��浠ｄ环

    // for (int i = 0; i < N; i++)
    // {
    //     R_bar.block(i * p, i * p, p, p) = R;
    // }

    // H = (C.transpose() * Q_bar * C) + R_bar;

    // 纭�淇� Q_bar 缁村害涓� (N+1)n x (N+1)n
    Q_bar.setZero(); 
    for (int i = 0; i < N; i++)
    {
        Q_bar.block(i * n, i * n, n, n) = Q;
    }
    // 鏈€鍚庝竴涓�鍧椾娇鐢ㄧ粓绔�浠ｄ环 F
    Q_bar.block(N * n, N * n, n, n) = F; 

    // R_bar 缁村害涓� Np x Np
    R_bar.setZero();
    for (int i = 0; i < N; i++)
    {
        R_bar.block(i * p, i * p, p, p) = R;
    }

    // 鏍囧噯 QP 褰㈠紡: H = 2 * (C' * Q_bar * C + R_bar)
    // qpOASES 姹傝В鐨勬槸 0.5 * x' * H * x锛屾墍浠ヨ繖閲屽缓璁�涔� 2 鎴栬€呬繚鎸佷竴鑷�
    H = (C.transpose() * Q_bar * C) + R_bar;
}


void MPC_Controller::BuildGradientVector()
{
//     // 鏋勫缓Gradient鍚戦噺
//     g = VectorXd::Zero(N * p);

//     g = C.transpose() * Q_bar * (M * x - x_ref);

    // 鏍囧噯 QP 姊�搴�: g = 2 * C' * Q_bar * (M * x - x_ref)
    // 娉ㄦ剰锛歺_ref 鐨勭淮搴﹀繀椤绘槸 (N+1)n锛屽寘鍚�浠� k=0 鍒� k=N 鐨勬墍鏈夊弬鑰冪姸鎬�
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
    // 鏋勫缓姊�搴﹀悜閲忓拰绾︽潫杈圭晫
    BuildGradientVector();
    BuildConstraintBounds();

    int nWSR = 100;

    // 鍒濆�嬪寲QP闂�棰�

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
    // 鏋勫缓姊�搴﹀悜閲忓拰绾︽潫杈圭晫
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
