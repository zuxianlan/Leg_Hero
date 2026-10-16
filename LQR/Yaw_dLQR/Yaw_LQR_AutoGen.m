%% 云台Yaw轴跟踪LQR 全自动生成整套代码（★离散系统版 dt = 0.001s）
%  改动说明：
%    1. 用零阶保持(ZOH)把连续模型离散化：Ad = expm(A*T), Bd = ∫expm(A*σ)dσ*Bm
%    2. 增益改用【离散】代数Riccati方程求解：Kd = dlqr(Ad, Bd, Q, R)
%    3. 生成的控制律仍是 u = -K1*e_theta - K2*e_omega（误差形式，跟踪关系不变）
%    4. 新增：离散设计 ↔ 连续等效增益对比、闭环极点打印、Riccati残差自检
clear; clc; close all;

% =====================【唯一手动修改区】=====================
Q = diag([5000, 20]);   % [角度误差权重q1, 角速度权重q2]
R = 1;               % 控制力矩权重
J = 0.0005;              % 云台转动惯量 kg·m²
B_damp = 0.01;          % ★粘性阻尼 N·m·s/rad（改名，避免与输入矩阵 B 冲突）
u_max = 8.0;            % 最大输出力矩限幅
u_min = -8.0;
T = 0.001;              % ★采样周期 dt (s)
% ==========================================================

%% 1. 构建连续状态空间  x = [theta; omega]
A  = [0,       1;
      0, -B_damp/J];
Bm = [0; 1/J];
C  = [1, 0];
D  = 0;

%% 2. 连续 → 离散（零阶保持，精确闭式解）
%  对 A = [0 1; 0 -b]，记 b = B_damp/J：
%     Ad = [1, (1-e^{-bT})/b ; 0, e^{-bT}]
%     Bd = [ (T - (1-e^{-bT})/b)/b ; (1-e^{-bT})/b ]
b   = B_damp/J;
eT  = exp(-b*T);
Ad  = [1, (1-eT)/b;
       0, eT];
Bd  = [(T-(1-eT)/b)/b;
       (1-eT)/b];

% 用 c2d 独立校核（应与上面闭式解完全一致）
sys_c = ss(A, Bm, C, D);
sys_d = c2d(sys_c, T, 'zoh');
fprintf('Ad 闭式解与 c2d 最大偏差 = %.3e\n', max(max(abs(Ad - sys_d.A))));
fprintf('Bd 闭式解与 c2d 最大偏差 = %.3e\n', max(max(abs(Bd - sys_d.B))));

%% 3. 求解【离散】LQR 最优反馈增益 Kd
[Kd, Pd, Poles_d] = dlqr(Ad, Bd, Q, R);
K1d = Kd(1);
K2d = Kd(2);

% 离散等效到连续的增益与极点（仅供与旧连续版对比参考）
K1c = K1d / T;
Poles_s = log(Poles_d) / T;

fprintf('====================计算结果（离散 dt=%.3fs）====================\n', T);
fprintf('K1d(角度误差增益)   =  %.4f    (连续等效 %.4f)\n', K1d, K1c);
fprintf('K2d(角速度误差增益) =  %.4f\n', K2d);
fprintf('离散闭环极点 lam    =  %.6f , %.6f\n', real(Poles_d(1)), real(Poles_d(2)));
fprintf('  |lam| (均应<1)    =  %.6f , %.6f\n', abs(Poles_d(1)), abs(Poles_d(2)));
fprintf('等效s平面极点       =  %.3f , %.3f  (rad/s)\n', real(Poles_s(1)), real(Poles_s(2)));
fprintf('离散Riccati残差     =  %.3e\n', norm(Ad'*Pd*Ad - Pd - Ad'*Pd*Bd*((R+Bd'*Pd*Bd)\Bd'*Pd*Ad) + Q));
fprintf('================================================================\n');

%% 4. 自动覆写 LQR_Calc.m，填充最新 K1d、K2d
fileID = fopen('LQR_Calc.m','w');
fprintf(fileID,'function u_torque = LQR_Calc(theta_ref, theta_now, omega_now)\n');
fprintf(fileID,'%%%%#codegen\n');
fprintf(fileID,'%%%% 自动生成LQR增益(离散 dt=%.4fs)，无需手动修改\n', T);
fprintf(fileID,'K1 = %.4f;\n', K1d);
fprintf(fileID,'K2 = %.4f;\n', K2d);
fprintf(fileID,'u_max = %.4f;\n', u_max);
fprintf(fileID,'u_min = %.4f;\n', u_min);
fprintf(fileID,'\n');
fprintf(fileID,'e_theta = theta_ref - theta_now;\n');
fprintf(fileID,'e_omega = 0.0 - omega_now;\n');
fprintf(fileID,'\n');
fprintf(fileID,'%%%% LQR跟踪控制律\n');
fprintf(fileID,'u_torque = -K1 * e_theta - K2 * e_omega;\n');
fprintf(fileID,'\n');
fprintf(fileID,'%%%% 力矩饱和限幅\n');
fprintf(fileID,'if u_torque > u_max\n');
fprintf(fileID,'    u_torque = u_max;\n');
fprintf(fileID,'elseif u_torque < u_min\n');
fprintf(fileID,'    u_torque = u_min;\n');
fprintf(fileID,'end\n');
fprintf(fileID,'end\n');
fclose(fileID);
disp('已自动更新 LQR_Calc.m 内K1/K2数值（离散）');

%% 5. 闭环仿真验证跟踪效果（离散）
A_cl = Ad - Bd*Kd;                % 离散闭环状态矩阵 (2x2)
B_cl = Bd*K1d;                    % 参考信号接入 (2x1)
sys_closed = ss(A_cl, B_cl, C, D, T);

t_step  = T;
t_total = 6;
t = (0:t_step:t_total)';
theta_ref = zeros(size(t));
theta_ref(t>1) = 1.0;             % 1s 目标 1rad
theta_ref(t>3) = -0.8;            % 3s 目标 -0.8rad

[y_out, t_out, x_state] = lsim(sys_closed, theta_ref, t);
theta_act = x_state(:,1);
omega_act = x_state(:,2);
err_theta = theta_ref - theta_act;
% 
% figure('Name','离散LQR 角度跟踪&误差曲线','Color','w');
% subplot(2,1,1);
% plot(t_out, theta_act, 'b-', 'LineWidth',1.2); hold on;
% plot(t_out, theta_ref, 'r--', 'LineWidth',1.2);
% legend('实际角度\theta','参考角度\theta_r','Location','best');
% ylabel('角度 (rad)'); grid on;
% title(sprintf('Yaw轴 离散LQR角度跟踪 (dt=%.4fs)', T));
% subplot(2,1,2);
% plot(t_out, err_theta, 'g-','LineWidth',1.2);
% ylabel('跟踪误差 e(rad)'); xlabel('时间 t(s)'); grid on;
% 
% figure('Name','角速度曲线','Color','w');
% plot(t_out, omega_act, 'm-','LineWidth',1.2);
% ylabel('角速度 \omega (rad/s)'); xlabel('t(s)'); grid on;
