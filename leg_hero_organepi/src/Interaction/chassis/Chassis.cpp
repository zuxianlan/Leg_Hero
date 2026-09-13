/**
  ******************************************************************************
  * @file           : Chassis.cpp
  * @author         : Chen Haoran
  * @brief          : None
  * @attention      : None
  * @date           : 2025/11/4
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Chassis.h"
#include <ranges>
#include "Algorithm/VMC/lqr_k_calc.h"
#include "Algorithm/LESO/leso_A_calc.h"
#include "Algorithm/LESO/leso_B_calc.h"
#include "Algorithm/LESO/leso_L_calc.h"
/* Define --------------------------------------------------------------------*/

/* Enum ----------------------------------------------------------------------*/

/* Variable && Struct --------------------------------------------------------*/

/* Function Declaration ------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/
float P[40][6] = 
{ 
	{-0.9114, -17.336, 4.5823, 17.454, 6.1675, -7.2869},
    {-1.4125, -18.315, 6.2039, 19.623, 3.9653, -8.7543},
    {-8.7714, 18.754, -12.309, -18.845, 4.3306, 14.373},
    {-2.3138, 5.4872, -3.9126, -5.0018, 1.2468, 4.5794},
    {-3.7196, -74.88, 17.691, 50.724, 19.991, -25.904},
    {-0.2118, -6.3291, 0.82563, 0.87253, 3.2788, -1.7858},
    {-2.1176, 0.082412, -37.85, 4.5016, 29.005, 30.37},
    {0.025268, -1.0123, -2.6438, 1.8442, -1.3215, 1.3436},
    {-46.761, 26.77, 68.96, 57.526, -69.581, -49.709},
    {-5.2271, 2.9101, 10.285, 5.2689, -9.4969, -8.932},
    {-0.9114, 4.5823, -17.336, -7.2869, 6.1675, 17.454},
    {-1.4125, 6.2039, -18.315, -8.7543, 3.9653, 19.623},
    {8.7714, 12.309, -18.754, -14.373, -4.3306, 18.845},
    {2.3138, 3.9126, -5.4872, -4.5794, -1.2468, 5.0018},
    {-2.1176, -37.85, 0.082412, 30.37, 29.005, 4.5016},
    {0.025268, -2.6438, -1.0123, 1.3436, -1.3215, 1.8442},
    {-3.7196, 17.691, -74.88, -25.904, 19.991, 50.724},
    {-0.2118, 0.82563, -6.3291, -1.7858, 3.2788, 0.87253},
    {-46.761, 68.96, 26.77, -49.709, -69.581, 57.526},
    {-5.2271, 10.285, 2.9101, -8.932, -9.4969, 5.2689},
    {19.974, 38.725, -75.991, -123.17, 83.706, 58.191},
    {24.558, 35.834, -94.78, -134.3, 115.03, 71.896},
    {-6.2838, -76.229, -19.68, 134.15, -72.052, 31.923},
    {-1.5012, -24.271, -4.5508, 40.743, -23.401, 6.5058},
    {61.078, 135.33, -52.778, -256.22, 204.99, 38.185},
    {6.8868, 6.6418, -6.4843, -7.3449, 3.8276, 6.3082},
    {12.623, -45.549, -76.566, 44.223, -164.84, 35.273},
    {0.56421, 3.0834, 0.53641, -11.23, -0.46667, -8.8167},
    {-17.042, -862.17, 48.374, 854.23, 270.18, -45.067},
    {1.9139, -64.232, -9.383, 42.744, 46.631, 9.854},
    {19.974, -75.991, 38.725, 58.191, 83.706, -123.17},
    {24.558, -94.78, 35.834, 71.896, 115.03, -134.3},
    {6.2838, 19.68, 76.229, -31.923, 72.052, -134.15},
    {1.5012, 4.5508, 24.271, -6.5058, 23.401, -40.743},
    {12.623, -76.566, -45.549, 35.273, -164.84, 44.223},
    {0.56421, 0.53641, 3.0834, -8.8167, -0.46667, -11.23},
    {61.078, -52.778, 135.33, 38.185, 204.99, -256.22},
    {6.8868, -6.4843, 6.6418, 6.3082, 3.8276, -7.3449},
    {-17.042, 48.374, -862.17, -45.067, 270.18, 854.23},
    {1.9139, -9.383, -64.232, 9.854, 46.631, 42.744}
};

void first_order_filter_calc(float x, float *y, float *last_y, float alpha)
{
    *y = alpha * x + (1-alpha) * *last_y;
    *last_y = *y;
}

float Max_Output(float num,float max)
{
	if(num>=max) return max;
	else if(num<=-max) return -max;
	else return num;
}

float Float_Math_Constrain(float *x, float Min, float Max)
{
    if (*x < Min)
    {
        *x = Min;
    }
    else if (*x > Max)
    {
        *x = Max;
    }
    return (*x);
}

float power_calculate(float K_0, float K_1, float K_2, float A, float Current, float Omega)
{
    return (K_0 * Current * Omega + K_1 * Omega * Omega + K_2 * Current * Current + A);
}

float Float_Math_Abs(float x)
{
	return ((x > 0) ? x : -x);
}

void normalizeAngle(float *angle) 
{
    float normalized = std::fmod(*angle, 2.0 * M_PI);
    
    // 确保角度在 [0, 2π)
    if (normalized < 0) {
        normalized += 2.0 * M_PI;
    }
    
    // 如果角度大于等于 π，减去 2π 得到 [-π, π)
    if (normalized >= M_PI) {
        normalized -= 2.0 * M_PI;
    }
    
    *angle = normalized;
}

void class_Chassis::Init()
{
    /******************* pid *******************/
    PID_Omega.Init(5.2f, 0.01f, 0.0001f, 0.0f, 0.0f, 5.0f, 0.002f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_DISABLE);

    PID_tp.Init(90.0f, 0.0f, 0.0f, 0.00f, 0.0f, 5.0f, 0.002f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);

    PID_tp_omega.Init(1.5f, 0.0f, 0.0f, 0.00f, 0.0f, 10.0f, 0.002f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);

    PID_roll.Init(80.6f, 0.00f, 0.001f, 0.0f, 0.0f, 200.5f, 0.002f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);

    PID_roll_omega.Init(8.5f, 0.0f, 0.00f, 0.0f, 0.0f, 350.5f, 0.002f, 0.0f, 0.0f, 0.0f, 0.0f, PID_D_First_ENABLE);

    /******************* kf *******************/
    kf.x_hat << 0.0, 0.0;
    kf.StateMinVariance << 0.03, 0.03;
    kf.P << 1.0, 0.0,
            0.0, 1.0;
    kf.F << 1.0, 0.001,
            0.0, 1.0;
    kf.H << 1.0, 0.0,
            0.0, 1.0;
    kf.Q << 1, 0.0,
            0.0, 1;
    kf.R << 100.0, 0.0,
            0.0, 100.0;

    /******************* mpc *******************/
    MPC_body.u_min << -150, -150;
    MPC_body.u_max << 150, 150;
    MPC_body.x_min << -M_PI / 4.0f, -0.5, 0.1, -1.0;
    MPC_body.x_max << M_PI / 4.0f, 0.5, 0.38, 1.0;
    MPC_body.Q << 1.5e7,0,0,0,
                  0,1e3,0,0,
                  0,0,1.5e7,0,
                  0,0,0,1e3;

    MPC_body.F << 1.5e8,0,0,0,
                  0,1e4,0,0,
                  0,0,1.5e8,0,
                  0,0,0,1e4;

    MPC_body.R << 0.9,0,
                  0,0.9;

    // MPC_body.R << 0.9e1,0,
    //               0,0.9e1;              

    MPC_body.A << 1,dt_mpc,0,0,
                  0,1,0,0,
                  0,0,1,dt_mpc,
                  0,0,0,1-(dt_mpc*b/m);

    MPC_body.B << 0,0,
                  -(r*dt_mpc/I_r), r*dt_mpc/I_r,
                  0,0,
                  dt_mpc/m, dt_mpc/m;

    // MPC_body.A <<
    // 1,  0.0100, 0, 0,
    //    0,  1,      0, 0,
    //    0,  0,      1, 0.0102,
    //    0,  0,      0, 1.0408;
    // MPC_body.B <<
    // -5.3333e-6,  5.3333e-6,
    //    -0.0011,     0.0011,
    //     2.0269e-6,  2.0269e-6,
    //     4.0811e-4,  4.0811e-4;



    MPC_body.BuildPredictionMatrices();
    MPC_body.BuildHessianMatrices();


    for(int i=0; i<2; i++)
    {
        Motor_Wheel[i].Power_K_0 = 0.2962f;
        Motor_Wheel[i].Power_K_1 = .0005f;
        Motor_Wheel[i].Power_K_2 = 0.1519f;
        Motor_Wheel[i].Power_A = 1.3544f;
    }

    LESO_left.Init(z0);

    A <<
1.0004,      0.0010,        0.0,         0.0,      3.4499e-5,   1.1499e-8,
0.7210,      1.0004,        0.0,         0.0,      0.0690,      3.4499e-5,
-1.0650e-4, -3.5499e-8,      1.0,    1.0000e-3,     -3.6157e-6,  -1.2051e-9,
-0.2130,    -1.0650e-4,      0.0,         1.0,     -0.0072,     -3.6157e-6,
8.5516e-5,   2.8504e-8,      0.0,         0.0,      1.0001,      0.0010,
0.1710,      8.5516e-5,      0.0,         0.0,      0.1162,      1.0001;

    B <<
    -2.3442e-05,   4.8960e-06,
-0.0469,       0.0098,
7.6118e-06,   -1.2675e-06,
0.0152,       -0.0025,
-2.5785e-06,   2.5192e-06,
-0.0052,       0.0050;

    L_Gein <<
0.1158,  0.0001,  0.0002, -0.0000,  0.0003,  0.0036,
1.4292,  0.1944, -0.0146, -0.0092,  0.0929,  0.0029,
0.0005, -0.0078,  0.0216,  0.0005,  0.0006,  0.0001,
-0.4357, -0.0332,  0.0030,  0.0877, -0.0074,  0.0038,
0.0005,  0.0003,  0.0003,  0.0023,  0.0998,  0.0018,
0.2601,  0.0078, -0.0010,  0.0036,  0.1660,  0.1211,
0.0116, -0.0747,  0.0110,  0.0128,  0.0034,  0.0612,
0.0194, -0.0026, -0.0019,  0.0488,  0.0010,  0.3254;

    Ae.block(0,0,6,6) = A;
    Ae.block(0,6,6,2) = B;
    Ae.block(6,6,2,2) = Eigen::MatrixXf::Identity(2,2);

    Be.block(0,0,6,2) = B;

    Ce.block(0,0,6,6) = Eigen::MatrixXf::Identity(6,6);

    LESO_left.SetMatrices(Ae, Be, Ce);
    LESO_left.SetObserverGain(L_Gein);

    LESO_right.Init(z0);
    LESO_right.SetMatrices(Ae, Be, Ce);
    LESO_right.SetObserverGain(L_Gein);
}


void class_Chassis::feedback_update()
{

    normalizeAngle(&mc_02->Rx_Data.Motor_joint[0].Position);
    normalizeAngle(&mc_02->Rx_Data.Motor_joint[1].Position);
    normalizeAngle(&mc_02->Rx_Data.Motor_joint[2].Position);
    normalizeAngle(&mc_02->Rx_Data.Motor_joint[3].Position);

    theta_err = left_leg.Get_theta() - right_leg.Get_theta();
    dtheta_err = left_leg.Get_dtheta() - right_leg.Get_dtheta();

    pitch_l = -(INS->pitch - Target_Pitch);
    d_pitch_l = -INS->gyro[0];

    // 根据腿长l0获得状态：theta d_theta
    left_leg.Set_phi1(static_cast<float>(std::numbers::pi) + mc_02->Rx_Data.Motor_joint[1].Position);
    left_leg.Set_dphi1(mc_02->Rx_Data.Motor_joint[1].omega);
    left_leg.Set_phi4(mc_02->Rx_Data.Motor_joint[0].Position);
    left_leg.Set_dphi4(mc_02->Rx_Data.Motor_joint[0].omega);
    left_leg.VMC_Calc_1();

    right_leg.Set_phi1(static_cast<float>(std::numbers::pi) - mc_02->Rx_Data.Motor_joint[2].Position);
    right_leg.Set_dphi1(-mc_02->Rx_Data.Motor_joint[2].omega);
    right_leg.Set_phi4(-mc_02->Rx_Data.Motor_joint[3].Position);
    right_leg.Set_dphi4(-mc_02->Rx_Data.Motor_joint[3].omega);
    right_leg.VMC_Calc_1();

    
    // Nthetal = static_cast<float>(std::numbers::pi) + mc_02->Rx_Data.Motor_joint[1].Position - static_cast<float>(std::numbers::pi)/2 - left_leg.Get_theta();
    // Nthetar = static_cast<float>(std::numbers::pi) - mc_02->Rx_Data.Motor_joint[2].Position - static_cast<float>(std::numbers::pi)/2 - right_leg.Get_theta();
    Nthetal = M_PI - mc_02->Rx_Data.Motor_joint[0].Position + left_leg.Get_alpha() - M_PI / 2;
    Nthetar = M_PI + mc_02->Rx_Data.Motor_joint[3].Position + right_leg.Get_alpha() - M_PI / 2;

    normalizeAngle(&Nthetal);
    normalizeAngle(&Nthetar);

    Now_height = (left_leg.Get_L0()*cosf(INS->roll) + right_leg.Get_L0()*cosf(INS->roll))/2;
    Now_dheight = (left_leg.Get_dL0()*cosf(INS->roll) + right_leg.Get_dL0()*cosf(INS->roll))/2;

    Fitting_K_Calc(Fitting_K,P,left_leg.Get_L0(),right_leg.Get_L0());

    err[0] = (X_filter - Target_X);                  
    err[1] = (Velocity_filter - Target_Velocity);
    // err[2] = Max_Output((Yaw_Angle - Target_Yaw), 1.25);               
    err[2] = 0;               
    err[3] = (INS->gyro[2] - Target_Omega);
    err[4] = Max_Output((left_leg.Get_theta() - Target_Theta), 0.5);  
    err[5] = (left_leg.Get_dtheta() - 0);
    err[6] = Max_Output((right_leg.Get_theta() - Target_Theta), 0.5); 
    err[7] = (right_leg.Get_dtheta() - 0);
    err[8] = Max_Output((INS->pitch - 0), 1.0);                       
    err[9] = (INS->gyro[0] - 0);


    // lambda1 = fabs(pitch_l) * 0.45;

}

void class_Chassis::Zero_Force_Control()
{
    PID_Omega.Set_Integral_Error(0);
    PID_tp.Set_Integral_Error(0);

    left_leg.Set_Tp(0);
    left_leg.Set_F0(0);
    left_leg.VMC_Calc_1();
    left_leg.VMC_Calc_2();
    T_L = 0;
    T_L_limit = 0;
    Tp_L = 0;
    right_leg.Set_Tp(0);
    right_leg.Set_F0(0);
    right_leg.VMC_Calc_1();
    right_leg.VMC_Calc_2();
    T_R = 0;
    T_R_limit = 0;
    Tp_R = 0;

    X_filter = 0;
    last_Target_X = 0;
}


void class_Chassis::Follow_Gimbal_Control()
{
    PID_tp.Set_Target(0);
    PID_tp.Set_Now(theta_err);
    PID_tp.TIM_Calculate_PeriodElapsedCallback();

    PID_tp_omega.Set_Target(PID_tp.Get_Out());
    PID_tp_omega.Set_Now(dtheta_err);
    PID_tp_omega.TIM_Calculate_PeriodElapsedCallback();

    MPC_Body_Calc();

    /******************* left_leg *******************/

    // 计算不同腿长下的LQR增益矩阵K
    float LQR_Tmp[14] = {0.0f};
    lqr_k_calc(left_leg.Get_L0(), LQR_Tmp);

    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            LQR_K_L[j][i] = LQR_Tmp[i*2 + j];
        }
    }

    if (chassis_fsm_mode == ABOVE_GROUND)
    {
        
        Target_Theta = 0.0;
        M = 180.0;
        F_mc = 0.0;
    
        PID_Omega.Set_Out(0.0);
    }
    else
    {
        Target_Theta = 0.0;
        M = 180.0;
        // 转弯离心力补偿
        F_mc = Velocity_filter * INS->gyro[2] * 28.0f;
    }
    
    Float_Math_Constrain(&F_mc, 0, 100.0f);
    spring_forcel = 100 - 10*(1 - (left_leg.Get_L0()-0.1)/0.28);
    spring_forcer = 100 - 10*(1 - (right_leg.Get_L0()-0.1)/0.28);

    // 定义阈值
    const float STOP_CMD_THRESH = 0.05f;   // 目标速度阈值
    const float STOP_ACT_THRESH = 0.5f;   // 实际速度阈值
    const float MOVE_CMD_THRESH = 0.01f;    // 运动阈值（带回差，防止抖动）

    // 停止标志位逻辑：同时判断指令目标和实际速度才能确认为完全停止
    if (fabs(Target_Velocity) < STOP_CMD_THRESH && fabs(Velocity_filter) < STOP_ACT_THRESH) {
        stop_flag = true;
    } else if (fabs(Target_Velocity) > MOVE_CMD_THRESH) {
        stop_flag = false;
    }
    

    // 重新计算 Target_X
    if (stop_flag) {
    // 如果刚进入停止状态，把目标位置赋值为当前位置，避免倒车
    if (!last_stop_flag) {
        X_filter = 0.0;
        Target_X = X_filter;
        last_Target_X = X_filter;
    } else {
        Target_X = last_Target_X;  // 保持
        Target_Theta = 0.05;
        
    }
    } else {
    // 运动状态，位置清零
    X_filter = 0.0;
    Target_Theta = 0.0;
    Target_X = X_filter;
    last_Target_X = X_filter;      // 重新赋值，方便停止时使用
    }

    

    LQR_Calc();

    T_L = T[0];

    left_leg.Set_Tp(T[2]- PID_tp_omega.Get_Out());

    left_leg.Set_F0(MPC_body.u_out(0) - spring_forcel + M / cosf(left_leg.Get_theta()) + Jump_force - F_mc + 5.0f);

    left_leg.VMC_Calc_2();

    T_R = T[1];
    
    right_leg.Set_Tp(T[3]+ PID_tp_omega.Get_Out());

    right_leg.Set_F0(-MPC_body.u_out(1) + spring_forcer - M / cosf(right_leg.Get_theta()) - Jump_force - F_mc - 0.0f);

    right_leg.VMC_Calc_2();
    
    last_Target_X = Target_X;
    last_stop_flag = stop_flag;

}

void class_Chassis::Check_in_Control()
{
    /******************* left_leg *******************/

    static int mod = 0;
    // mod++;
    T_L = -Target_Velocity;
    left_leg.Set_Tp(0);
    ll = 0 - (1 - pow(left_leg.Get_L0()/0.34, 2));
    if(ll < 0)
    {
        ll = 0;
    }
    left_leg.Set_F0(-ll);
    left_leg.VMC_Calc_2();

  
    // T_R = 0;
    // right_leg.Set_Tp(0);

    // T_R =  + T_R_Speed + T_R_YAW;
    T_R =  -Target_Velocity;
    // T_R =  + 0;
    
    right_leg.Set_Tp(0);
    rr = 0 - (1 - right_leg.Get_L0()/0.34);
    if(rr < 0)
    {
        rr = 0;
    }
    right_leg.Set_F0(rr);
    right_leg.VMC_Calc_2();

    X_filter = 0;

}


void class_Chassis::Control_leg()
{
    PID_Omega.Set_Integral_Error(0);

    PID_tp.Set_Target(0);
    PID_tp.Set_Now(theta_err);
    PID_tp.TIM_Calculate_PeriodElapsedCallback();

    PID_tp_omega.Set_Target(PID_tp.Get_Out());
    PID_tp_omega.Set_Now(dtheta_err);
    PID_tp_omega.TIM_Calculate_PeriodElapsedCallback();


    left_leg.VMC_Calc_1();
    T_L = 0;
    right_leg.VMC_Calc_1();
    T_R = 0;
    X_filter = 0;

    // 如果目标高度发生变化（差值超限），重新生成一次轨迹规划
    if (fabs(Target_height - last_Target_height) > 0.001f) 
    {
        height_planner.MoveTo(Now_height, Target_height, Now_dheight, Target_dheight, 2.5f); // 0.5秒内完成动作
        last_Target_height = Target_height;
    }

    // 获取当前平均参考高度
    if (height_planner.is_planning) 
    {
        height_planner.Step(dt, current_ref_height, current_ref_dheight);
    } else 
    {
        current_ref_height = Target_height;
        current_ref_dheight = 0.0f;
    }

    MPC_body.x(0) = INS->roll;
    MPC_body.x(1) = INS->gyro[1];
    MPC_body.x(2) = Now_height;
    MPC_body.x(3) = Now_dheight;
    MPC_body.x_ref.setZero();
    for (int i = 0; i < N; i++)
    {
        MPC_body.x_ref(i*4) = INS->roll;
        MPC_body.x_ref(i*4+1) = 0.0;
        MPC_body.x_ref(i*4+2) = current_ref_height;
        MPC_body.x_ref(i*4+3) = current_ref_dheight;
    }
    MPC_body.MPC_Calculate();
    left_leg.Set_Tp(4.0 - PID_tp_omega.Get_Out());
    T_L = 0.0;

    spring_forcel = 130 - 10*(1 - (left_leg.Get_L0()-0.1)/0.24);
    spring_forcer = 130 - 10*(1 - (right_leg.Get_L0()-0.1)/0.24);

    left_leg.Set_F0(MPC_body.u_out(0) / 1.5 - spring_forcel);
    left_leg.VMC_Calc_2();


    right_leg.Set_Tp(4.0 + PID_tp_omega.Get_Out());
    T_R = 0.0;
    
    right_leg.Set_F0(-MPC_body.u_out(1) / 1.5 + spring_forcer);
    right_leg.VMC_Calc_2();

    // left_leg.Set_Tp(0.0);
    // left_leg.Set_F0( - spring_force * cos(Nthetal));
    // left_leg.VMC_Calc_2();

    // right_leg.Set_Tp(0.0);
    // right_leg.Set_F0(+ spring_force * cos(Nthetar));
    // right_leg.VMC_Calc_2();
    

}

void class_Chassis::Control_leg_left(float target)
{
    if(target != 0)
    {
        // LQR力矩计算
        T_L = 0.0;
        Tp_L = (left_leg.Get_dtheta() - target) * Fitting_K[2][5];

        left_leg.Set_Tp(Tp_L);
        left_leg.Set_F0(0.0);
        left_leg.VMC_Calc_2();
    }
    else
    {
        left_leg.Set_Tp(0.0);
        left_leg.Set_F0(0.0);
        left_leg.VMC_Calc_2();
    }

}

void class_Chassis::Control_leg_right(float target)
{
    if(target != 0)
    {
        // LQR力矩计算
        T_R = 0.0;
        Tp_R = (right_leg.Get_dtheta() - target) * Fitting_K[3][7];

        right_leg.Set_Tp(Tp_R);
        right_leg.Set_F0(0.0);
        right_leg.VMC_Calc_2();
    }
    else
    {
        right_leg.Set_Tp(0.0);
        right_leg.Set_F0(0.0);
        right_leg.VMC_Calc_2();
    }
}

void class_Chassis::UP_Staris_Control_leg()
{
    // left_leg.Set_Tp(-0.0);
    // left_leg.Set_F0( 0.0);
    // left_leg.VMC_Calc_2();

    // right_leg.Set_Tp(-0.0);
    // right_leg.Set_F0(0.0);
    // right_leg.VMC_Calc_2();

    PID_Omega.Set_Integral_Error(0);
    PID_tp.Set_Target(0);
    PID_tp.Set_Now(theta_err);
    PID_tp.TIM_Calculate_PeriodElapsedCallback();

    PID_tp_omega.Set_Target(PID_tp.Get_Out());
    PID_tp_omega.Set_Now(dtheta_err);
    PID_tp_omega.TIM_Calculate_PeriodElapsedCallback();

    left_leg.VMC_Calc_1();
    T_L = 0;
    right_leg.VMC_Calc_1();
    T_R = 0;
    X_filter = 0;

    // 如果目标高度发生变化（差值超限），重新生成一次轨迹规划
    if (fabs(Target_height - last_Target_height) > 0.001f) 
    {
        height_planner.MoveTo(Now_height, Target_height, Now_dheight, 0.25, 2.5f); // 0.5秒内完成动作
        last_Target_height = Target_height;
    }

    // 获取当前平均参考高度
    if (height_planner.is_planning) 
    {
        height_planner.Step(dt, current_ref_height, current_ref_dheight);
    } else 
    {
        current_ref_height = Target_height;
        current_ref_dheight = 0.0f;
    }

    MPC_body.x(0) = INS->roll;
    MPC_body.x(1) = INS->gyro[1];
    MPC_body.x(2) = Now_height;
    MPC_body.x(3) = Now_dheight;
    MPC_body.x_ref.setZero();
    for (int i = 0; i < N; i++)
    {
        MPC_body.x_ref(i*4) = 0.0;
        MPC_body.x_ref(i*4+1) = 0.0;
        MPC_body.x_ref(i*4+2) = current_ref_height;
        MPC_body.x_ref(i*4+3) = current_ref_dheight;
    }
    MPC_body.MPC_Calculate();
    left_leg.Set_Tp(-1.0 - PID_tp_omega.Get_Out());
    T_L = 0.0;

    spring_forcel = 30 - 10*(1 - (left_leg.Get_L0()-0.1)/0.24);
    spring_forcer = 30 - 10*(1 - (right_leg.Get_L0()-0.1)/0.24);

    left_leg.Set_F0(MPC_body.u_out(0) / 3.5 - spring_forcel - 0);
    // left_leg.Set_F0( - spring_forceL * sin(Nthetal));
    left_leg.VMC_Calc_2();


    right_leg.Set_Tp(-1.0 + PID_tp_omega.Get_Out());

    T_R = 0.0;
    
    right_leg.Set_F0(-MPC_body.u_out(1) / 3.5 + spring_forcer + 0);
    // right_leg.Set_F0(+ spring_forceR * sin(Nthetar));
    right_leg.VMC_Calc_2();

    X_filter = 0;
    last_Target_X = 0;
    
}

void class_Chassis::Self_Observe()
{
    static float omega_l, omega_r = 0.0f;
    static float speed_l, speed_r = 0.0f;
    static float average_speed = 0.0f;

    omega_l = mc_02->Rx_Data.Motor_wheel[0].omega - INS->gyro[0] + left_leg.Get_dtheta();
    speed_l = 0.06 * omega_l + left_leg.Get_L0() * left_leg.Get_dtheta() * cosf(left_leg.Get_theta()) + left_leg.Get_dL0() * sinf(left_leg.Get_theta());

    omega_r = mc_02->Rx_Data.Motor_wheel[1].omega - INS->gyro[0] + right_leg.Get_dtheta();
    speed_r = 0.06 * omega_r + right_leg.Get_L0() * right_leg.Get_dtheta() * cosf(right_leg.Get_theta()) + right_leg.Get_dL0() * sinf(right_leg.Get_theta());

    average_speed = -(speed_l - speed_r) / 2.0f;

    kf.z << average_speed,
            INS->MotionAccel_n[1];

    kf.Predict();
    kf.Update();

    Velocity_filter = kf.x_hat(0);
   
    X_filter = X_filter + Velocity_filter * (static_cast<float>(1)/1000.0f);

    Slip_Detection_And_Suppression();

}

void class_Chassis::Slip_Detection_And_Suppression()
{
    float w_l = mc_02->Rx_Data.Motor_wheel[0].omega;
    float w_r = mc_02->Rx_Data.Motor_wheel[1].omega;

    float alpha_l = (w_l - last_wheel_omega_l) / dt;
    float alpha_r = (w_r - last_wheel_omega_r) / dt;
    last_wheel_omega_l = w_l;
    last_wheel_omega_r = w_r;

    float v_wheel_l = w_l * 0.06f;
    float v_wheel_r = -w_r * 0.06f;

    float v_body = Velocity_filter;

    float slip_err_l = fabsf(v_wheel_l) - fabsf(v_body) - 0.5f;
    float slip_err_r = fabsf(v_wheel_r) - fabsf(v_body) - 0.5f;

    bool slip_l = (slip_err_l > 2.5f && fabsf(v_wheel_l) > 2.5f)
               || (fabsf(alpha_l) > 250.0f && fabsf(v_wheel_l) > 2.0f);
    bool slip_r = (slip_err_r > 2.5f && fabsf(v_wheel_r) > 2.5f)
               || (fabsf(alpha_r) > 250.0f && fabsf(v_wheel_r) > 2.0f);

    if (slip_l)
        slip_factor_l *= 0.3f;
    else
        slip_factor_l += (1.0f - slip_factor_l) * 0.05f;

    if (slip_r)
        slip_factor_r *= 0.3f;
    else
        slip_factor_r += (1.0f - slip_factor_r) * 0.05f;

    Float_Math_Constrain(&slip_factor_l, 0.0f, 1.0f);
    Float_Math_Constrain(&slip_factor_r, 0.0f, 1.0f);
}

void class_Chassis::Ground_Detection()
{
    above_flag_l = left_leg.ground_detection_L();
    above_flag_r = right_leg.ground_detection_R();
}

float Chassis_Wheel_Power_Limit_Control(float wheel_available_power, float wheel_consume_power)
{
    if (wheel_consume_power <= wheel_available_power)
    {
        // 功率未超限
        return (1.0f);
    }
    else
    {
        // 按比例降低功率
        return (wheel_available_power / wheel_consume_power);
    }
}


void Wheel_Power_Control(motor_power *motor)
{
    // 估算功率消耗值
    motor->Power_Estimate = power_calculate(motor->Power_K_0, motor->Power_K_1, motor->Power_K_2, motor->Power_A, motor->Target_Current, motor->Now_Omega);

    // 判断估算功率是否超限
    if (motor->Power_Estimate > 0.0f)
    {
        if (motor->Power_Factor >= 1.0f)
        {
            // 功率充足
        }
        else
        {
            // 需要限制功率

            // 根据功率限制公式求解一元二次方程
            float a = motor->Power_K_2;
            float b = motor->Power_K_0 * motor->Now_Omega;
            float c = motor->Power_A + motor->Power_K_1 * motor->Now_Omega * motor->Now_Omega - motor->Power_Factor * motor->Power_Estimate;
            float delta, h;
            delta = b * b - 4 * a * c;
            if (delta < 0.0f)
            {
                // 无解
                motor->Target_Current = 0.0f;
            }
            else
            {
                h = sqrtf32(delta);
                float result_1, result_2;
                result_1 = (-b + h) / (2.0f * a);
                result_2 = (-b - h) / (2.0f * a);

                // 功率限制候选值，取与当前电流同号或绝对值较小的解
                if ((result_1 > 0.0f && result_2 < 0.0f) || (result_1 < 0.0f && result_2 > 0.0f))
                {
                    if ((motor->Target_Current > 0.0f && result_1 > 0.0f) || (motor->Target_Current < 0.0f && result_1 < 0.0f))
                    {
                        motor->Target_Current = result_1;
                    }
                    else
                    {
                        motor->Target_Current = result_2;
                    }
                }
                else
                {
                    if (Float_Math_Abs(result_1) < Float_Math_Abs(result_2))
                    {
                        motor->Target_Current = result_1;
                    }
                    else
                    {
                        motor->Target_Current = result_2;
                    }
                }
            }
        }
    }
}


void class_Chassis::Chassis_Power_Control()
{
    // 功率限制参数
    float available_power = 150.0f;
    float consume_power = 0.0f;
    float wheel_consume_power = 0.0f;
    float wheel_power_single[2];
    static uint8_t mod = 0;
    mod++;
  
    // if(mod == 100)
    // {   //100ms执行一次
    //     Chassis_Power_Control->PID_Power_buffer.Target = 40.0f;
    //     Chassis_Power_Control->PID_Power_buffer.Now = power_heat_data.buffer_energy;
    //     PID_TIM_Adjust_PeriodElapsedCallback(&Chassis_Power_Control->PID_Power_buffer);
    //     PID_Set_Integral_Error(&Chassis_Power_Control->PID_Power_buffer, 0.0f);
    //     mod = 0;
    //     // 通过PID对可用功率进行动态调整
    //     available_power += Chassis_Power_Control->PID_Power_buffer.Out;
    // }

    // 获取各轮估算功率值
    wheel_power_single[0] = Motor_Wheel[0].Power_Estimate;
    wheel_power_single[1] = Motor_Wheel[1].Power_Estimate;
    for (int i = 0; i < 2; i++)
    {
        // 消耗功率累加，负功率补充可用功率
        if (wheel_power_single[i] > 0)
        {
            consume_power += wheel_power_single[i];
            wheel_consume_power += wheel_power_single[i];
        }
        else
        {
            available_power += -wheel_power_single[i];
        }
    }

    // 计算车轮功率限制系数
    Wheel_Factor = Chassis_Wheel_Power_Limit_Control(available_power, wheel_consume_power);
    for (int i = 0; i < 2; i++)
    {
        if (wheel_power_single[i] > 0)
        {
            Motor_Wheel[i].Power_Factor = Wheel_Factor;
        }
        else
        {
            Motor_Wheel[i].Power_Factor = 1.0f;
        }
    }
    // std::cout<<Wheel_Factor<<std::endl;

  
    Motor_Wheel[0].Now_Omega = mc_02->Rx_Data.Motor_wheel[0].omega;
    Motor_Wheel[0].Target_Current = (T_L_Speed + T_L_YAW) / 0.3;
    // Motor_Wheel[0].Target_Current = mc_02->Rx_Data.current[0];

    Motor_Wheel[1].Now_Omega = -mc_02->Rx_Data.Motor_wheel[1].omega;
    Motor_Wheel[1].Target_Current = (T_R_Speed - T_R_YAW) / 0.3;
    // Motor_Wheel[1].Target_Current = mc_02->Rx_Data.current[1];

    Wheel_Power_Control(&Motor_Wheel[0]);
    Wheel_Power_Control(&Motor_Wheel[1]);

    T_L_limit = Motor_Wheel[0].Target_Current * 0.3;
    T_R_limit = Motor_Wheel[1].Target_Current * 0.3;

    // k1 = T_L_limit / (T_L_Speed + T_L_YAW);
    // k2 = T_R_limit / (T_R_Speed + T_R_YAW);

    Float_Math_Constrain(&k1, 0.0f, 1.0f);
    Float_Math_Constrain(&k2, 0.0f, 1.0f);

    T_L = (LQR_K_L[0][0]*(left_leg.Get_theta())
                +LQR_K_L[0][1]*(left_leg.Get_dtheta())
                +LQR_K_L[0][2]*(X_filter - Target_X) * k1
                +LQR_K_L[0][3]*(Velocity_filter - Target_Velocity) * k1
                +LQR_K_L[0][4]*(pitch_l) 
                +LQR_K_L[0][5]*(d_pitch_l))+ PID_Omega.Get_Out();


    // T_L += PID_Omega.Get_Out() * k1;
    T_L = T_L_M + T_L_limit;

    T_R = (LQR_K_R[0][0]*(right_leg.Get_theta())
                +LQR_K_R[0][1]*(right_leg.Get_dtheta())
                +LQR_K_R[0][2]*(X_filter - Target_X) * k2
                +LQR_K_R[0][3]*(Velocity_filter - Target_Velocity) * k2
                +LQR_K_R[0][4]*(pitch_l)
                +LQR_K_R[0][5]*(d_pitch_l))- PID_Omega.Get_Out();


    // T_R += - PID_Omega.Get_Out() * k2;
    T_R = T_R_M + T_R_limit;

    left_leg.Set_Tp(Tp_L - PID_tp_omega.Get_Out());

    left_leg.VMC_Calc_2();

    right_leg.Set_Tp(Tp_R + PID_tp_omega.Get_Out());

    right_leg.VMC_Calc_2();
}


void class_Chassis::MPC_Body_Calc()
{
    if (fabs(Target_height - last_Target_height) > 0.001f) 
    {
        height_planner.MoveTo(Now_height, Target_height, Now_dheight, Target_dheight, 0.05f); // 0.5秒内完成动作
        last_Target_height = Target_height;
    }

    // 获取当前平均参考高度
    if (height_planner.is_planning) 
    {
        height_planner.Step(dt, current_ref_height, current_ref_dheight);
    } else 
    {
        current_ref_height = Target_height;
        current_ref_dheight = 0.0f;
    }

    MPC_body.x_ref.setZero();

    if(chassis_fsm_mode == ABOVE_GROUND)
    {
        MPC_body.x(0) = 0;
        MPC_body.x(1) = 0;
        MPC_body.x(2) = Now_height;
        MPC_body.x(3) = Now_dheight;

        Target_Roll = 0;
    }
    else
    {
        MPC_body.x(0) = INS->roll;
        MPC_body.x(1) = INS->gyro[1];
        MPC_body.x(2) = Now_height;
        MPC_body.x(3) = Now_dheight;
    }

    for (int i = 0; i < N+1; i++)
    {
        MPC_body.x_ref(i*4) = Target_Roll;
        MPC_body.x_ref(i*4+1) = 0.0;
        MPC_body.x_ref(i*4+2) = Target_height;
        MPC_body.x_ref(i*4+3) = Target_dheight;
    }

    MPC_body.MPC_Calculate();
    

}

void class_Chassis::LQR_Calc()
{
    for(int i=0;i<=3;i++)
    {
        if (chassis_fsm_mode == ABOVE_GROUND)
        {
            if (i>1)
            {
                T[i] = err[4]*Fitting_K[i][4]*2.0 + err[5]*Fitting_K[i][5]*0.6
                        + err[6]*Fitting_K[i][6]*2.0 + err[7]*Fitting_K[i][7]*0.6;
            }
            else
            {
                T[i] = 0;
            }
        }
        else
        {
            if (spin_mode)
            {
                T[i] = err[0] * 0                      + err[1] * Fitting_K[i][1]
                     - err[2] * 0                      + err[3] * Fitting_K[i][3]
                     + err[4] * Fitting_K[i][4] * 1.2f + err[5] * Fitting_K[i][5] * 1.2f
                     + err[6] * Fitting_K[i][6] * 1.2f + err[7] * Fitting_K[i][7] * 1.2f
                     + err[8] * Fitting_K[i][8] * 3.0f + err[9] * Fitting_K[i][9] * 1.4f;
            }
            else
            {
                T[i] = err[0] * Fitting_K[i][0] + Max_Output(err[1] * Fitting_K[i][1], 16)
                     - Max_Output(err[2] * Fitting_K[i][2], 15.0f) + err[3] * Fitting_K[i][3]
                     + err[4] * Fitting_K[i][4] + err[5] * Fitting_K[i][5]
                     + err[6] * Fitting_K[i][6] + err[7] * Fitting_K[i][7]
                     + err[8] * Fitting_K[i][8] + err[9] * Fitting_K[i][9];
            }
        }
    }
}

void class_Chassis::LESO_UPdate_Left()
{
    float tmp_A[64], tmp_B[16], tmp_L[48] = {};
    leso_A_calc(left_leg.Get_L0(), tmp_A);
    leso_B_calc(left_leg.Get_L0(), tmp_B);
    leso_L_calc(left_leg.Get_L0(), tmp_L);


    for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            Ae(j, i) = tmp_A[i*8 + j];
        }
    }

    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            Be(j, i) = tmp_B[i*8 + j];
        }
    }

    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            L_Gein(j, i) = tmp_L[i*8 + j];
        }
    }

    LESO_left.SetMatrices(Ae,Be,Ce);
    LESO_left.SetObserverGain(L_Gein);

    u_left <<
    T_L_M + T_L_Speed, Tp_L;

    y_left <<
    left_leg.Get_theta(), left_leg.Get_dtheta(), X_filter, Velocity_filter, pitch_l, d_pitch_l;
    
    LESO_left.Update(u_left, y_left);
    d_hat_left = LESO_left.get_disturbance_estimate();
    // d_hat_left(0) = stop_flag ? 0.0f : d_hat_left(0);
    // d_hat_left(1) = stop_flag ? 0.0f : d_hat_left(1);

    // first_order_filter_calc(d_hat_left(0), &d_hat_left(0), &last_d_hat_left(0), dt / (dt + 1 / (2 * M_PI * 1.0f)));
    // first_order_filter_calc(d_hat_left(1), &d_hat_left(1), &last_d_hat_left(1), dt / (dt + 1 / (2 * M_PI * 1.0f)));

    LESO_dhat[0] = d_hat_left(0);
    LESO_dhat[1] = d_hat_left(1);
}

void class_Chassis::LESO_UPdate_Right()
{
    float tmp_A[64], tmp_B[16], tmp_L[48] = {};
    leso_A_calc(right_leg.Get_L0(), tmp_A);
    leso_B_calc(right_leg.Get_L0(), tmp_B);
    leso_L_calc(right_leg.Get_L0(), tmp_L);


    for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            Ae(j, i) = tmp_A[i*8 + j];
        }
    }

    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            Be(j, i) = tmp_B[i*8 + j];
        }
    }

    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            L_Gein(j, i) = tmp_L[i*8 + j];
        }
    }

    LESO_right.SetMatrices(Ae,Be,Ce);
    LESO_right.SetObserverGain(L_Gein);

    u_right <<
    T_R_M + T_R_Speed, Tp_R;

    y_right <<
    right_leg.Get_theta(), right_leg.Get_dtheta(), X_filter, Velocity_filter, pitch_l, d_pitch_l;
    
    LESO_right.Update(u_right, y_right);
    d_hat_right = LESO_right.get_disturbance_estimate();
    // d_hat_right(0) = stop_flag ? 0.0f : d_hat_right(0);
    // d_hat_right(1) = stop_flag ? 0.0f : d_hat_right(1);

    // first_order_filter_calc(d_hat_right(0), &d_hat_right(0), &d_hat_right(0), dt / (dt + 1 / (2 * M_PI * 1.0f)));
    // first_order_filter_calc(d_hat_right(1), &d_hat_right(1), &d_hat_right(1), dt / (dt + 1 / (2 * M_PI * 1.0f)));

    LESO_dhat[2] = d_hat_right(0);
    LESO_dhat[3] = d_hat_right(1);
}


