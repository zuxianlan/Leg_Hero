function u_torque = LQR_Calc(theta_ref, theta_now, omega_now)
%#codegen
% 自动生成LQR增益，无需手动修改
K1 = 9.4868;
K2 = 0.5796;
u_max = 1.0000;
u_min = -1.0000;

e_theta = theta_ref - theta_now;
e_omega = 0.0 - omega_now;

% LQR跟踪控制律
u_torque = -K1 * e_theta - K2 * e_omega;

% 力矩饱和限幅
if u_torque > u_max
    u_torque = u_max;
elseif u_torque < u_min
    u_torque = u_min;
end
end
