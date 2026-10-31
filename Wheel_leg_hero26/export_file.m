clc;
clear;

l0 = 0.07;

result = lqr_k_calc(l0);
Ae = leso_A_calc(l0);
Be = leso_B_calc(l0);
L = leso_L_calc(l0);


L0 = 0.07;
T = 0;
Tp = 0;
theta = 0;
theta1 = 0;
result2 = x_ddot(L0, T, Tp, theta, theta1);
result3 = theta_ddot(L0, T, Tp, theta, theta1);
resultr = phi_ddot(Tp);