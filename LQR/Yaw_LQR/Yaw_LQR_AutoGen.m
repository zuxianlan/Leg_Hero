%% 云台Yaw轴跟踪LQR 全自动生成整套代码
clear; clc; close all;

% =====================【唯一手动修改区】=====================
Q = diag([900, 3]);   % [角度误差权重q1, 角速度权重q2]
R = 10;              % 控制力矩权重
J = 0.008;             % 云台转动惯量 kg·m²
B = 0.1;             % 粘性阻尼 N·m·s/rad
u_max = 1.0;          % 最大输出力矩限幅
u_min = -1.0;
% ==========================================================

%% 1. 构建连续状态空间
A = [0,      1;
     0, -B/J];
Bm = [0; 1/J];
C  = [1, 0];
D  = 0;

%% 2. 求解LQR最优反馈增益K
[K, P] = lqr(A, Bm, Q, R);
K1 = K(1);
K2 = K(2);
fprintf('====================计算结果====================\n');
fprintf('K1(角度误差增益)   =  %.4f\n', K1);
fprintf('K2(角速度误差增益) =  %.4f\n', K2);
fprintf('================================================\n');

%% 3. 自动覆写LQR_Calc.m函数，填充最新K1、K2
fileID = fopen('LQR_Calc.m','w');
fprintf(fileID,'function u_torque = LQR_Calc(theta_ref, theta_now, omega_now)\n');
fprintf(fileID,'%%#codegen\n');
fprintf(fileID,'%% 自动生成LQR增益，无需手动修改\n');
fprintf(fileID,'K1 = %.4f;\n',K1);
fprintf(fileID,'K2 = %.4f;\n',K2);
fprintf(fileID,'u_max = %.4f;\n',u_max);
fprintf(fileID,'u_min = %.4f;\n',u_min);
fprintf(fileID,'\n');
fprintf(fileID,'e_theta = theta_ref - theta_now;\n');
fprintf(fileID,'e_omega = 0.0 - omega_now;\n');
fprintf(fileID,'\n');
fprintf(fileID,'%% LQR跟踪控制律\n');
fprintf(fileID,'u_torque = -K1 * e_theta - K2 * e_omega;\n');
fprintf(fileID,'\n');
fprintf(fileID,'%% 力矩饱和限幅\n');
fprintf(fileID,'if u_torque > u_max\n');
fprintf(fileID,'    u_torque = u_max;\n');
fprintf(fileID,'elseif u_torque < u_min\n');
fprintf(fileID,'    u_torque = u_min;\n');
fprintf(fileID,'end\n');
fprintf(fileID,'end\n');
fclose(fileID);
disp('已自动更新 LQR_Calc.m 内K1/K2数值');

%% 4. 闭环仿真验证跟踪效果
%{
A_cl = A - Bm * K;                % 闭环状态矩阵 (2x2)
B_cl = Bm * K1;                   % 输入矩阵，将参考信号接入 (2x1)
sys_closed = ss(A_cl, B_cl, C, D);

% 仿真时间与分段角度指令
t_step = 0.001;
t_total = 6;
t = (0:t_step:t_total)';          % 列向量
theta_ref = zeros(size(t));       % 列向量
theta_ref(t>1)  = 1.0;            % 1s 目标1rad
theta_ref(t>3)  = -0.8;           % 3s 目标-0.8rad

% 强制确保列向量（防止意外）
theta_ref = theta_ref(:);
t = t(:);

% 仿真运行
[y_out, t_out, x_state] = lsim(sys_closed, theta_ref, t);
theta_act = x_state(:,1);
omega_act = x_state(:,2);
err_theta = theta_ref - theta_act;

%}

%% 5. 绘图输出曲线

% figure('Name','角度跟踪&误差曲线','Color','w');
% subplot(2,1,1);
% plot(t_out, theta_act, 'b-', 'LineWidth',1.2); hold on;
% plot(t_out, theta_ref, 'r--', 'LineWidth',1.2);
% legend('实际角度θ','参考角度θ_r','Location','best');
% ylabel('角度 (rad)'); grid on; title('Yaw轴 LQR角度跟踪');
% 
% subplot(2,1,2);
% plot(t_out, err_theta, 'g-','LineWidth',1.2);
% ylabel('跟踪误差 e(rad)'); xlabel('时间 t(s)'); grid on;
% 
% figure('Name','角速度曲线','Color','w');
% plot(t_out, omega_act, 'm-','LineWidth',1.2);
% ylabel('角速度 ω (rad/s)'); xlabel('t(s)'); grid on;

